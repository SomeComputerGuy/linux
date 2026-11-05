// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver
 *
 * AVS1 (GB/T 20090.2 JiZhun) and AVS+ (GY/T 257.1 Guangdian) decoding on
 * the AVS+ decoder of the RK3588. The block sits next to the VPU121, with
 * the register layout of a Hantro G1 decoder, and shares its interrupt
 * line, clocks, power domain and IOMMU with the VDPU2 decoder of VPU121.
 * It is driven as a third function of that device, so it never runs at the
 * same time as the VDPU2 decoder or the VEPU2 encoder.
 *
 * Userspace parses the sequence and picture headers, the hardware parses
 * the slices. A field coded picture takes two hardware runs, one per
 * field, both in one job: the second one is started from the interrupt
 * handler of the first.
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-mem2mem.h>

#include "hantro.h"
#include "hantro_hw.h"
#include "rockchip_avsd_regs.h"

/* Slices have start codes 00 00 01 00 to 00 00 01 af */
#define AVS_SLICE_MAX_START_CODE	0xaf

#define AVSD_NO_REF	((dma_addr_t)0)

static inline void avsd_write(struct hantro_dev *vpu, u32 val,
			      unsigned int reg)
{
	vpu_debug(6, "avsd 0x%04x = 0x%08x\n", reg, val);
	writel_relaxed(val, vpu->avsd_base + reg * 4);
}

static inline u32 avsd_read(struct hantro_dev *vpu, unsigned int reg)
{
	u32 val = readl(vpu->avsd_base + reg * 4);

	vpu_debug(6, "avsd 0x%04x : 0x%08x\n", reg, val);
	return val;
}

struct avsd_job {
	const struct v4l2_ctrl_avs_sequence *seq;
	const struct v4l2_ctrl_avs_picture *pic;
	const struct v4l2_ctrl_avs_weight_quant *wq;
	/* R0, R1, R2 of the uAPI, NULL when absent */
	const struct v4l2_avs_ref_pic *r0, *r1, *r2;
	dma_addr_t dst, ref0, ref1, mv_cur, mv_col;
	unsigned int stride;
	bool field, b, avsp, r0_frame;
};

static unsigned int avsd_dist(unsigned int a, unsigned int b)
{
	unsigned int d = (2 * a - 2 * b) & 0x1ff;

	return d ? d : 2;
}

static void avsd_set_half(u32 *regs, unsigned int reg, unsigned int i,
			  unsigned int val)
{
	unsigned int shift = (i & 1) * 16;

	regs[reg] &= ~(0xffffU << shift);
	regs[reg] |= (val & 0xffff) << shift;
}

/* Distance and its inverse to the reference block i of the current picture */
static void avsd_cur(u32 *regs, unsigned int i, unsigned int dist)
{
	avsd_set_half(regs, AVSD_REG_REF_DIST_CUR(i), i, dist);
	avsd_set_half(regs, AVSD_REG_REF_INVD_CUR(i), i, 512 / dist);
}

/* Inverse distances of the co-located picture, for direct mode */
static void avsd_col(u32 *regs, unsigned int i, unsigned int invd)
{
	avsd_set_half(regs, AVSD_REG_REF_INVD_COL(i), i, invd);
}

/*
 * Distance from R0 to its own second reference, as it was computed when R0
 * was decoded: from R2 if R0 had two references, else from R1.
 */
static unsigned int avsd_anchor_dist(const struct avsd_job *j)
{
	if (j->r0 && j->r2)
		return avsd_dist(j->r0->picture_distance, j->r2->picture_distance);
	if (j->r0 && j->r1)
		return avsd_dist(j->r0->picture_distance, j->r1->picture_distance);
	return 2;
}

/*
 * The reference distances of the current picture, in units of fields
 * (2 * picture_distance, modulo 512), with one entry per reference field,
 * and their inverses scaled by 512 (and by 16384 for the co-located
 * picture). A missing reference leaves the previous distance, as it did on
 * the vendor's implementation, and a zero distance becomes 2.
 */
static void avsd_set_distances(const struct avsd_job *j, u32 *regs,
			       bool first)
{
	unsigned int pd = j->pic->picture_distance;
	unsigned int t = 2, f;

	if (!j->field) {
		if (j->b) {
			if (j->r0)
				t = avsd_dist(j->r0->picture_distance, pd);
			avsd_cur(regs, 2, t);
			avsd_cur(regs, 3, t);
			if (j->r1)
				t = avsd_dist(pd, j->r1->picture_distance);
			avsd_cur(regs, 0, t);
			avsd_cur(regs, 1, t);
			if (j->r0 && j->r1)
				t = avsd_dist(j->r0->picture_distance,
					      j->r1->picture_distance);
			avsd_col(regs, 0, 16384 / t);
			avsd_col(regs, 1, 16384 / t);
			f = avsd_anchor_dist(j);
			avsd_col(regs, 2, 16384 / f);
			avsd_col(regs, 3, 16384 / f);
		} else {
			if (j->r0)
				t = avsd_dist(pd, j->r0->picture_distance);
			avsd_cur(regs, 0, t);
			avsd_cur(regs, 1, t);
			if (j->r1)
				t = avsd_dist(pd, j->r1->picture_distance);
			avsd_cur(regs, 2, t);
			avsd_cur(regs, 3, t);
		}
		return;
	}

	if (j->b) {
		if (j->r0)
			t = avsd_dist(j->r0->picture_distance, pd);
		avsd_cur(regs, 2, first ? t : t - 1);
		avsd_cur(regs, 3, first ? t + 1 : t);
		if (j->r1)
			t = avsd_dist(pd, j->r1->picture_distance);
		avsd_cur(regs, 0, first ? t - 1 : t);
		avsd_cur(regs, 1, first ? t : t + 1);
		if (j->r0 && j->r1)
			t = avsd_dist(j->r0->picture_distance,
				      j->r1->picture_distance);
		f = avsd_anchor_dist(j);
		if (first || (j->pic->flags & V4L2_AVS_PICTURE_FLAG_PB_FIELD_ENHANCED)) {
			avsd_col(regs, 0, 16384 / (t - 1));
			avsd_col(regs, 1, 16384 / t);
			avsd_col(regs, 2, 16384 / (f - 1));
		} else {
			avsd_col(regs, 0, 16384);
			avsd_col(regs, 1, 16384 / t);
			avsd_col(regs, 2, 16384 / (t + 1));
		}
		avsd_col(regs, 3, 16384 / f);
	} else {
		if (j->r0)
			t = avsd_dist(pd, j->r0->picture_distance);
		if (first) {
			avsd_cur(regs, 0, t - 1);
		} else {
			avsd_cur(regs, 0, 1);
			avsd_cur(regs, 2, t + 1);
		}
		avsd_cur(regs, 1, t);
		if (j->r1)
			t = avsd_dist(pd, j->r1->picture_distance);
		if (first)
			avsd_cur(regs, 2, t - 1);
		avsd_cur(regs, 3, t);
	}
}

/* Derive the six weighting parameters from the coded syntax */
static void avsd_weight_quant(const struct v4l2_ctrl_avs_weight_quant *wq,
			      u8 wqp[V4L2_AVS_WEIGHT_QUANT_NUM_PARAMS])
{
	static const u8 base[2][V4L2_AVS_WEIGHT_QUANT_NUM_PARAMS] = {
		{ 128, 98, 106, 116, 116, 128 },
		{ 135, 143, 143, 160, 160, 213 },
	};
	unsigned int i;

	for (i = 0; i < V4L2_AVS_WEIGHT_QUANT_NUM_PARAMS; i++) {
		switch (wq->weighting_quant_param_index) {
		case 1:
			wqp[i] = base[1][i] + wq->weighting_quant_param_delta1[i];
			break;
		case 2:
			wqp[i] = base[0][i] + wq->weighting_quant_param_delta2[i];
			break;
		default:
			wqp[i] = base[0][i];
			break;
		}
	}
}

/*
 * Registers of one hardware run, all but the stream ones. @first is the
 * first (or only) field of the picture.
 */
static void avsd_set_regs(const struct avsd_job *j, u32 *regs, bool first)
{
	const struct v4l2_ctrl_avs_picture *pic = j->pic;
	const struct v4l2_ctrl_avs_sequence *seq = j->seq;
	unsigned int mb_w = DIV_ROUND_UP(seq->horizontal_size, 16);
	unsigned int mb_h = DIV_ROUND_UP(seq->vertical_size, 16);
	bool tff = pic->flags & V4L2_AVS_PICTURE_FLAG_TOP_FIELD_FIRST;
	bool top = first == tff;
	dma_addr_t refs[4], mv;
	unsigned int i;
	u32 reg;

	memset(regs, 0, HANTRO_AVS_NUM_REGS * sizeof(*regs));

	regs[AVSD_REG_CONFIG] = AVSD_CFG_MAX_BURST(16) |
				AVSD_CFG_OUT_ENDIAN |
				AVSD_CFG_CLK_GATE_E |
				AVSD_CFG_OUTSWAP32_E |
				AVSD_CFG_INSWAP32_E |
				AVSD_CFG_STRENDIAN_E |
				AVSD_CFG_STRSWAP32_E |
				AVSD_CFG_TIMEOUT_E |
				AVSD_CFG_AXI_RD_ID(0xff);

	reg = AVSD_CTRL_DEC_MODE(AVSD_DEC_MODE_AVS);
	if (j->field) {
		reg |= AVSD_CTRL_PIC_INTERLACE_E | AVSD_CTRL_PIC_FIELDMODE_E;
		if (top)
			reg |= AVSD_CTRL_PIC_TOPFIELD_E;
	}
	if (j->b)
		reg |= AVSD_CTRL_PIC_B_E;
	if (pic->picture_type != V4L2_AVS_PICTURE_TYPE_I)
		reg |= AVSD_CTRL_PIC_INTER_E;
	if (pic->flags & V4L2_AVS_PICTURE_FLAG_FIXED_PICTURE_QP)
		reg |= AVSD_CTRL_PIC_FIXED_QUANT;
	if (pic->flags & V4L2_AVS_PICTURE_FLAG_LOOP_FILTER_DISABLE)
		reg |= AVSD_CTRL_FILTERING_DIS;
	if (pic->flags & V4L2_AVS_PICTURE_FLAG_SKIP_MODE)
		reg |= AVSD_CTRL_SKIP_MODE;
	/*
	 * I and P pictures write the motion vectors that the following B
	 * pictures use for direct mode. In JiZhun mode only the second field
	 * of an I picture, which is coded like a P field, has any.
	 */
	if (j->avsp ? !j->b :
	    (pic->picture_type == V4L2_AVS_PICTURE_TYPE_P ||
	     (pic->picture_type == V4L2_AVS_PICTURE_TYPE_I && !first)))
		reg |= AVSD_CTRL_WRITE_MVS_E;
	regs[AVSD_REG_CTRL] = reg;

	reg = AVSD_PIC_MB_WIDTH(mb_w) | AVSD_PIC_MB_HEIGHT_P(mb_h);
	if (pic->flags & V4L2_AVS_PICTURE_FLAG_PICTURE_REFERENCE)
		reg |= AVSD_PIC_REFER_FLAG;
	regs[AVSD_REG_PIC_SIZE] = reg;

	regs[AVSD_REG_STREAM_CFG] = AVSD_ALPHA_OFFSET(pic->alpha_c_offset) |
				    AVSD_BETA_OFFSET(pic->beta_offset);
	regs[AVSD_REG_STREAM_LEN] = AVSD_INIT_QP(pic->picture_qp);
	regs[AVSD_REG_EXT] = AVSD_H_EXT((seq->vertical_size + 15) >> 12);

	if (j->avsp) {
		const struct v4l2_ctrl_avs_weight_quant *wq = j->wq;

		reg = AVSD_AVSP_E;
		if (pic->flags & V4L2_AVS_PICTURE_FLAG_AEC)
			reg |= AVSD_AEC_E;
		if (pic->flags & V4L2_AVS_PICTURE_FLAG_NO_FORWARD_REFERENCE)
			reg |= AVSD_NO_FWD_REF_E;
		if (pic->flags & V4L2_AVS_PICTURE_FLAG_PB_FIELD_ENHANCED)
			reg |= AVSD_PB_FIELD_ENHANCE_E;
		if (pic->flags & V4L2_AVS_PICTURE_FLAG_WEIGHTING_QUANT) {
			u8 wqp[V4L2_AVS_WEIGHT_QUANT_NUM_PARAMS];

			avsd_weight_quant(wq, wqp);
			reg |= AVSD_WEIGHT_QP_E |
			       AVSD_WEIGHT_QP_MODEL(wq->weighting_quant_model) |
			       AVSD_WEIGHT_QP_0(wqp[0]);
			if (!(wq->flags & V4L2_AVS_WEIGHT_QUANT_FLAG_CHROMA_QUANT_PARAM_DISABLE))
				reg |= AVSD_QP_DELTA_CB(wq->chroma_quant_param_delta_cb) |
				       AVSD_QP_DELTA_CR(wq->chroma_quant_param_delta_cr);
			regs[AVSD_REG_REF_DELTA] = AVSD_WEIGHT_QP_1(wqp[1]);
			regs[AVSD_REG_WEIGHT_QP] = AVSD_WEIGHT_QP_2(wqp[2]) |
						   AVSD_WEIGHT_QP_3(wqp[3]) |
						   AVSD_WEIGHT_QP_4(wqp[4]) |
						   AVSD_WEIGHT_QP_5(wqp[5]);
		}
		regs[AVSD_REG_AVSP] = reg;

		/*
		 * Field B pictures: offsets of the reference fields, in
		 * fields (1 = +2, 6 = -2), for the forward and backward
		 * references of the current picture and the co-located one.
		 */
		if (j->field && j->b) {
			reg = AVSD_REF_DELTA_COL(0, 2) | AVSD_REF_DELTA_COL(2, 2);
			if (first)
				reg |= AVSD_REF_DELTA_CUR(0, 2);
			else
				reg |= AVSD_REF_DELTA_CUR(2, 6) |
				       AVSD_REF_DELTA_CUR(3, 6);
			regs[AVSD_REG_REF_DELTA] |= reg;
		}
	}

	/* Output: the second field starts one line down */
	regs[AVSD_REG_OUT_BASE] = j->dst + (j->field && !top ? j->stride : 0);

	/*
	 * References, by field: the second field of an I or P picture can
	 * refer to the first one, in the output buffer.
	 */
	if (j->field && !first && !j->b) {
		refs[0] = j->dst;
		refs[1] = j->ref0;
		refs[2] = j->ref0;
		refs[3] = j->ref1;
	} else {
		refs[0] = j->ref0;
		refs[1] = j->ref0;
		refs[2] = j->ref1;
		refs[3] = j->ref1;
	}
	/* The direct mode needs to know whether R0 is field coded */
	if (!j->r0_frame) {
		refs[2] |= AVSD_REF_FIELD_E;
		refs[3] |= AVSD_REF_FIELD_E | AVSD_REF_TOP_E;
	}
	for (i = 0; i < 4; i++)
		regs[AVSD_REG_REF_BASE(i)] = refs[i];

	if (!(j->r0 && (j->r0->flags & V4L2_AVS_REF_FLAG_INTRA)) ||
	    (!first && !j->r0_frame))
		regs[AVSD_REG_ANCHOR] = AVSD_PREV_ANC_TYPE;

	avsd_set_distances(j, regs, first);

	/* Motion vectors: the second field uses the second half */
	mv = j->b ? j->mv_col : j->mv_cur;
	regs[AVSD_REG_DIR_MV_BASE2] = j->avsp ? mv : 0;
	if (!first && !(j->b && j->r0_frame)) {
		unsigned int rows = (pic->flags & V4L2_AVS_PICTURE_FLAG_PROGRESSIVE_FRAME) ?
				    mb_h : 2 * DIV_ROUND_UP(seq->vertical_size, 32);

		mv += ALIGN(mb_w * rows / 2, 2) * 16;
	}
	regs[AVSD_REG_DIR_MV_BASE] = mv;

	regs[AVSD_REG_PRED_BC_TAP_A] = (7 << 2) | (1 << 12) | (0x3ffU << 22);
	regs[AVSD_REG_PRED_BC_TAP_B] = (1 << 12) | (7 << 22);
	regs[AVSD_REG_PRED_BC_TAP_C] = (5 << 2) | (5 << 12) | (0x3ffU << 22);
	regs[AVSD_REG_APF] = AVSD_APF_THRESHOLD(8);
}

static void avsd_set_stream(struct hantro_avs_dec_hw_ctx *avs, u32 *regs,
			    u32 offset)
{
	u32 base = offset & ~7;

	regs[AVSD_REG_STREAM_BASE] = avs->strm_dma + base;
	regs[AVSD_REG_STREAM_CFG] |= AVSD_STRM_START_BIT(8 * (offset & 7));
	regs[AVSD_REG_STREAM_LEN] |= AVSD_STREAM_LEN(avs->strm_size - base);
}

static void avsd_start(struct hantro_dev *vpu, const u32 *regs)
{
	unsigned int i;

	for (i = AVSD_REG_CONFIG; i < HANTRO_AVS_NUM_REGS; i++)
		avsd_write(vpu, regs[i], i);
	/* Start the decoder after all the other registers are written. */
	wmb();
	writel(AVSD_INT_DEC_E, vpu->avsd_base + AVSD_REG_INTERRUPT * 4);
}

static dma_addr_t avsd_ref(struct hantro_ctx *ctx,
			   const struct v4l2_avs_ref_pic *ref)
{
	if (!ref)
		return AVSD_NO_REF;
	return hantro_get_ref(ctx, ref->timestamp);
}

static const struct v4l2_avs_ref_pic *
avsd_valid_ref(const struct v4l2_ctrl_avs_picture *pic, unsigned int i)
{
	return (pic->refs[i].flags & V4L2_AVS_REF_FLAG_VALID) ? &pic->refs[i] : NULL;
}

int rockchip_avsd_avs_dec_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->core;
	struct hantro_avs_dec_hw_ctx *avs = &ctx->avs_dec;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	const struct v4l2_ctrl_avs_picture *pic;
	struct avsd_job j = { };
	unsigned int mb_w, mb_h;
	dma_addr_t r0_addr;
	size_t mv_off;

	hantro_start_prepare_run(ctx);

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	j.seq = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_AVS_SEQUENCE);
	j.pic = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_AVS_PICTURE);
	j.wq = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_AVS_WEIGHT_QUANT);
	if (WARN_ON(!j.seq || !j.pic || !j.wq))
		goto err;
	pic = j.pic;

	if (j.seq->horizontal_size > ctx->src_fmt.width ||
	    j.seq->vertical_size > ctx->src_fmt.height) {
		vpu_debug(0, "AVS picture %ux%u larger than the format\n",
			  j.seq->horizontal_size, j.seq->vertical_size);
		goto err;
	}

	avs->strm_dma = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	avs->strm_vaddr = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
	avs->strm_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);
	if (pic->data_offset >= avs->strm_size)
		goto err;

	j.field = !(pic->flags & V4L2_AVS_PICTURE_FLAG_PICTURE_STRUCTURE);
	j.b = pic->picture_type == V4L2_AVS_PICTURE_TYPE_B;
	j.avsp = j.seq->profile_id == V4L2_AVS_PROFILE_GUANGDIAN;
	if (j.b) {
		j.r0 = avsd_valid_ref(pic, V4L2_AVS_REF_BACKWARD);
		j.r1 = avsd_valid_ref(pic, V4L2_AVS_REF_FORWARD);
		j.r2 = avsd_valid_ref(pic, V4L2_AVS_REF_SECOND_FORWARD);
	} else {
		j.r0 = avsd_valid_ref(pic, V4L2_AVS_REF_FORWARD);
		j.r1 = avsd_valid_ref(pic, V4L2_AVS_REF_SECOND_FORWARD);
	}
	j.r0_frame = j.r0 && (j.r0->flags & V4L2_AVS_REF_FLAG_PICTURE_STRUCTURE);

	j.dst = hantro_get_dec_buf_addr(ctx, &dst_buf->vb2_buf);
	j.stride = ctx->dst_fmt.plane_fmt[0].bytesperline;

	/* A missing reference is replaced by the output, as a stream error */
	j.ref0 = avsd_ref(ctx, avsd_valid_ref(pic, V4L2_AVS_REF_FORWARD));
	if (!j.ref0)
		j.ref0 = j.dst;
	j.ref1 = avsd_ref(ctx, avsd_valid_ref(pic, j.b ? V4L2_AVS_REF_BACKWARD :
						    V4L2_AVS_REF_SECOND_FORWARD));
	if (!j.ref1)
		j.ref1 = j.ref0;

	/* Motion vectors are stored after the picture, in its buffer */
	mb_w = MB_WIDTH(ctx->src_fmt.width);
	mb_h = MB_HEIGHT(ctx->src_fmt.height);
	mv_off = 384 * mb_w * mb_h;
	j.mv_cur = j.dst + mv_off;
	r0_addr = avsd_ref(ctx, j.r0);
	j.mv_col = (r0_addr ? r0_addr : j.dst) + mv_off;

	avsd_set_regs(&j, avs->regs[0], true);
	avsd_set_stream(avs, avs->regs[0], pic->data_offset);
	if (j.field)
		avsd_set_regs(&j, avs->regs[1], false);
	avs->second_field = j.field;

	hantro_end_prepare_run(ctx);

	avsd_start(vpu, avs->regs[0]);

	return 0;

err:
	hantro_end_prepare_run(ctx);
	return -EINVAL;
}

/*
 * The second field starts at the first start code after the point where
 * the hardware stopped reading the first one. Look from 8 bytes before it,
 * as the hardware reads ahead in words.
 */
static int avsd_second_field_offset(struct hantro_dev *vpu,
				    struct hantro_avs_dec_hw_ctx *avs)
{
	const u8 *data = avs->strm_vaddr;
	u32 stop = avsd_read(vpu, AVSD_REG_STREAM_BASE);
	u32 pos, end = avs->strm_size;

	if (!data || stop < avs->strm_dma || stop - avs->strm_dma > end)
		return -EINVAL;

	pos = stop - avs->strm_dma;
	pos -= min(pos, 8U);
	for (; pos + 3 < end; pos++) {
		if (data[pos] || data[pos + 1] || data[pos + 2] != 1)
			continue;
		if (data[pos + 3] > AVS_SLICE_MAX_START_CODE)
			return -EINVAL;
		return pos;
	}

	return -EINVAL;
}

/*
 * Called for every interrupt of the shared decoder line. Returns IRQ_NONE
 * when the AVS+ decoder didn't raise it.
 */
irqreturn_t rockchip_avsd_irq(struct hantro_dev *vpu)
{
	enum vb2_buffer_state state;
	struct hantro_ctx *ctx;
	u32 status;
	int offset;

	if (!vpu->avsd_base || !vpu->avsd_id)
		return IRQ_NONE;

	status = avsd_read(vpu, AVSD_REG_INTERRUPT);
	if (!(status & AVSD_INT_IRQ))
		return IRQ_NONE;

	avsd_write(vpu, 0, AVSD_REG_INTERRUPT);
	avsd_write(vpu, AVSD_CFG_CLK_GATE_E, AVSD_REG_CONFIG);

	state = (status & AVSD_INT_RDY) ? VB2_BUF_STATE_DONE :
					  VB2_BUF_STATE_ERROR;
	ctx = vpu->curr_ctx;
	if (!ctx || ctx->vpu_src_fmt->codec_mode != HANTRO_MODE_AVS_DEC) {
		dev_warn_ratelimited(vpu->dev,
				     "AVS+ decoder interrupt without a job: %08x\n",
				     status);
		return IRQ_HANDLED;
	}

	if (state == VB2_BUF_STATE_DONE && ctx->avs_dec.second_field) {
		struct hantro_avs_dec_hw_ctx *avs = &ctx->avs_dec;

		avs->second_field = false;
		offset = avsd_second_field_offset(vpu, avs);
		if (offset < 0) {
			state = VB2_BUF_STATE_ERROR;
		} else {
			/* If the watchdog already runs, it finishes the job */
			if (!cancel_delayed_work(&vpu->watchdog_work))
				return IRQ_HANDLED;
			avsd_set_stream(avs, avs->regs[1], offset);
			avsd_start(vpu, avs->regs[1]);
			schedule_delayed_work(&vpu->watchdog_work,
					      msecs_to_jiffies(2000));
			return IRQ_HANDLED;
		}
	}

	hantro_irq_done(vpu, state);

	return IRQ_HANDLED;
}

void rockchip_avsd_avs_dec_reset(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->core;
	u32 val;

	avsd_write(vpu, AVSD_INT_IRQ_DIS, AVSD_REG_INTERRUPT);
	avsd_write(vpu, AVSD_CFG_CLK_GATE_E, AVSD_REG_CONFIG);
	writel(1, vpu->avsd_base + AVSD_SOFT_RESET);
	if (readl_poll_timeout_atomic(vpu->avsd_base + AVSD_SOFT_RESET, val,
				      !val, 1, 100))
		dev_warn(vpu->dev, "AVS+ decoder soft reset timed out\n");
	avsd_write(vpu, 0, AVSD_REG_INTERRUPT);
}

/*
 * Read the identification and synthesis configuration registers once,
 * before the first AVS decode. The block is not in the datasheet: a zero
 * or all-ones ID means it is absent or not powered, and AVS decoding is
 * refused.
 */
static int avsd_check_hw(struct hantro_dev *vpu)
{
	u32 id, cfg, cfg2;
	int ret;

	if (vpu->avsd_id)
		return 0;

	ret = pm_runtime_resume_and_get(vpu->dev);
	if (ret < 0)
		return ret;
	ret = clk_bulk_enable(vpu->variant->num_clocks, vpu->clocks);
	if (ret) {
		pm_runtime_put_autosuspend(vpu->dev);
		return ret;
	}

	id = readl(vpu->avsd_base + AVSD_REG_ID * 4);
	cfg = readl(vpu->avsd_base + AVSD_REG_SYNTH_CFG * 4);
	cfg2 = readl(vpu->avsd_base + AVSD_REG_SYNTH_CFG_2 * 4);

	clk_bulk_disable(vpu->variant->num_clocks, vpu->clocks);
	pm_runtime_put_autosuspend(vpu->dev);

	dev_info(vpu->dev, "AVS+ decoder: id %08x (product %04x), config %08x %08x\n",
		 id, AVSD_ID_PRODUCT(id), cfg, cfg2);
	if (!id || id == ~0U)
		return -ENODEV;

	vpu->avsd_id = id;
	return 0;
}

int rockchip_avsd_avs_dec_init(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;

	if (!vpu->avsd_base)
		return -ENODEV;

	ctx->avs_dec.second_field = false;
	return avsd_check_hw(vpu);
}

void rockchip_avsd_avs_dec_exit(struct hantro_ctx *ctx)
{
}

/*
 * Map the registers of the AVS+ decoder, if the device tree describes them
 * as the "avsd" range of the VPU121 node.
 */
int rockchip_avsd_hw_init(struct hantro_dev *vpu)
{
	struct platform_device *pdev = vpu->pdev;
	void __iomem *base;

	if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, "avsd"))
		return 0;

	base = devm_platform_ioremap_resource_byname(pdev, "avsd");
	if (IS_ERR(base))
		return PTR_ERR(base);

	vpu->avsd_base = base;
	return 0;
}
