// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip VDPU381 AVS2 backend
 *
 * The core parses the slices of the picture. Software parses the sequence
 * and picture headers, and gives the core a bit-packed copy of the syntax
 * elements it needs (the "picture header table"), the weighting quantization
 * matrices, and the picture order counts and buffers of the references.
 */

#include <linux/build_bug.h>
#include <linux/sizes.h>

#include <media/v4l2-mem2mem.h>

#include "rkvdec.h"
#include "rkvdec-ccu.h"
#include "rkvdec-rcb.h"
#include "rkvdec-vdpu381-regs.h"

/* Picture types as the core knows them */
enum vdpu381_avs2_pic_type {
	VDPU381_AVS2_PIC_I	= 0,
	VDPU381_AVS2_PIC_P	= 1,
	VDPU381_AVS2_PIC_B	= 2,
	VDPU381_AVS2_PIC_F	= 3,
	VDPU381_AVS2_PIC_G	= 4,	/* background picture, output */
	VDPU381_AVS2_PIC_GB	= 5,	/* background picture, not output */
	VDPU381_AVS2_PIC_S	= 6,	/* P picture predicted from the background */
};

/* The picture header table: 1408 bits, read in 128-bit units */
#define AVS2_HEADER_SIZE	176
/* The 4x4 and the 8x8 weighting quantization matrices */
#define AVS2_WQ_SIZE		(16 + 64)

/* The tables of one job, see &rkvdec_job.slot */
struct rkvdec_avs2_priv_tbl {
	u8 header[SZ_4K];
	u8 wq_matrix[SZ_4K];
};

struct rkvdec_avs2_ctx {
	struct rkvdec_aux_buf		priv_tbl[RKVDEC_MAX_JOBS];
	struct rkvdec_vdpu381_regs_avs2	regs;
};

struct rkvdec_avs2_run {
	struct rkvdec_run base;
	const struct v4l2_ctrl_avs2_sequence *seq;
	const struct v4l2_ctrl_avs2_picture *pic;
	const struct v4l2_ctrl_avs2_weight_quant *wq;
	const struct v4l2_ctrl_avs2_alf *alf;
};

static_assert(sizeof(struct rkvdec_vdpu381_regs_avs2_params) == 49 * sizeof(u32));

/* Fills the table from bit 0 of byte 0 on, each field from its LSB. */
struct avs2_bit_writer {
	u8 *buf;
	unsigned int pos;
	unsigned int size;	/* in bits */
};

static void avs2_put_bits(struct avs2_bit_writer *bw, u32 val, unsigned int len)
{
	unsigned int i;

	if (WARN_ON_ONCE(bw->pos + len > bw->size))
		return;

	for (i = 0; i < len; i++, bw->pos++)
		if (val & BIT(i))
			bw->buf[bw->pos / 8] |= BIT(bw->pos % 8);
}

static u8 avs2_hw_pic_type(const struct v4l2_ctrl_avs2_picture *pic)
{
	switch (pic->picture_type) {
	case V4L2_AVS2_PICTURE_TYPE_I:
		if (!(pic->flags & V4L2_AVS2_PICTURE_FLAG_BACKGROUND_PICTURE))
			return VDPU381_AVS2_PIC_I;
		if (pic->flags & V4L2_AVS2_PICTURE_FLAG_BACKGROUND_PICTURE_OUTPUT)
			return VDPU381_AVS2_PIC_G;
		return VDPU381_AVS2_PIC_GB;
	case V4L2_AVS2_PICTURE_TYPE_P:
		if (pic->flags & V4L2_AVS2_PICTURE_FLAG_BACKGROUND_PRED)
			return VDPU381_AVS2_PIC_S;
		return VDPU381_AVS2_PIC_P;
	case V4L2_AVS2_PICTURE_TYPE_B:
		return VDPU381_AVS2_PIC_B;
	default:
		return VDPU381_AVS2_PIC_F;
	}
}

/*
 * The core takes the derived centre tap of each filter: the coded value
 * plus 64 minus twice the sum of the other taps (GB/T 33475.2 9.12.2).
 */
static void avs2_put_alf_filter(struct avs2_bit_writer *bw, const s16 *coeff)
{
	int sum = 0;
	unsigned int i;

	for (i = 0; i < V4L2_AVS2_ALF_NUM_COEFFS - 1; i++) {
		avs2_put_bits(bw, coeff[i], 7);
		sum += 2 * coeff[i];
	}
	avs2_put_bits(bw, coeff[i] + 64 - sum, 7);
}

static void assemble_hw_header(struct rkvdec_avs2_run *run, u8 *buf)
{
	const struct v4l2_ctrl_avs2_sequence *seq = run->seq;
	const struct v4l2_ctrl_avs2_picture *pic = run->pic;
	const struct v4l2_ctrl_avs2_alf *alf = run->alf;
	struct avs2_bit_writer bw = {
		.buf = buf,
		.size = AVS2_HEADER_SIZE * 8,
	};
	u32 sf = seq->flags, pf = pic->flags;
	bool alf_y = false, alf_cb = false, alf_cr = false;
	unsigned int bit_depth_minus8 = 2 * (seq->encoding_precision - 1);
	unsigned int i;

	memset(buf, 0, AVS2_HEADER_SIZE);

	if (sf & V4L2_AVS2_SEQUENCE_FLAG_ALF_ENABLE) {
		alf_y = alf->flags & V4L2_AVS2_ALF_FLAG_LUMA_ENABLE;
		alf_cb = alf->flags & V4L2_AVS2_ALF_FLAG_CB_ENABLE;
		alf_cr = alf->flags & V4L2_AVS2_ALF_FLAG_CR_ENABLE;
	}

	/* Sequence header */
	avs2_put_bits(&bw, seq->chroma_format, 2);
	avs2_put_bits(&bw, ALIGN(seq->horizontal_size, 8), 16);
	avs2_put_bits(&bw, ALIGN(seq->vertical_size, 8), 16);
	avs2_put_bits(&bw, bit_depth_minus8, 3);
	avs2_put_bits(&bw, bit_depth_minus8, 3);
	avs2_put_bits(&bw, seq->lcu_size, 3);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_PROGRESSIVE_SEQUENCE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_FIELD_CODED_SEQUENCE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_MHP_SKIP_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_DHP_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_WSM_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_AMP_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_NSQT_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_NSIP_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_SECONDARY_TRANSFORM_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_SAO_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_ALF_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_PMVR_ENABLE), 1);
	avs2_put_bits(&bw, !!(sf & V4L2_AVS2_SEQUENCE_FLAG_CROSS_SLICE_LOOPFILTER_ENABLE), 1);

	/* Picture header */
	avs2_put_bits(&bw, avs2_hw_pic_type(pic), 3);
	avs2_put_bits(&bw, pic->num_ref_pics, 3);
	avs2_put_bits(&bw, !!(pf & V4L2_AVS2_PICTURE_FLAG_BACKGROUND_REFERENCE), 1);
	/* bottom field picture: field coded sequences are not supported */
	avs2_put_bits(&bw, 0, 1);
	avs2_put_bits(&bw, !!(pf & V4L2_AVS2_PICTURE_FLAG_FIXED_PICTURE_QP), 1);
	avs2_put_bits(&bw, pic->picture_qp, 7);
	avs2_put_bits(&bw, !!(pf & V4L2_AVS2_PICTURE_FLAG_LOOP_FILTER_DISABLE), 1);
	avs2_put_bits(&bw, pic->alpha_c_offset, 5);
	avs2_put_bits(&bw, pic->beta_offset, 5);
	avs2_put_bits(&bw, pic->chroma_quant_param_delta_cb, 6);
	avs2_put_bits(&bw, pic->chroma_quant_param_delta_cr, 6);
	avs2_put_bits(&bw, !!(pf & V4L2_AVS2_PICTURE_FLAG_WEIGHT_QUANT_ENABLE), 1);

	/* Adaptive loop filter: only the filters that are used */
	avs2_put_bits(&bw, alf_y, 1);
	avs2_put_bits(&bw, alf_cb, 1);
	avs2_put_bits(&bw, alf_cr, 1);

	if (alf_y) {
		unsigned int num = clamp_t(unsigned int, alf->num_luma_filters, 1,
					   V4L2_AVS2_ALF_MAX_LUMA_FILTERS);

		avs2_put_bits(&bw, num - 1, 4);
		for (i = 0; i < V4L2_AVS2_ALF_MAX_LUMA_FILTERS; i++)
			avs2_put_bits(&bw, alf->luma_region_filter[i], 4);
		for (i = 0; i < num; i++)
			avs2_put_alf_filter(&bw, alf->luma_coeff[i]);
	}
	if (alf_cb)
		avs2_put_alf_filter(&bw, alf->chroma_coeff[0]);
	if (alf_cr)
		avs2_put_alf_filter(&bw, alf->chroma_coeff[1]);
}

/*
 * The core reads the matrices transposed (column by column), the 4x4 and
 * then the 8x8 one. Only read when the picture enables weighting
 * quantization.
 */
static void assemble_hw_wq_matrix(struct rkvdec_avs2_run *run, u8 *buf)
{
	const struct v4l2_ctrl_avs2_weight_quant *wq = run->wq;
	unsigned int i, j, n = 0;

	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			buf[n++] = wq->matrix_4x4[j * 4 + i];
	for (i = 0; i < 8; i++)
		for (j = 0; j < 8; j++)
			buf[n++] = wq->matrix_8x8[j * 8 + i];
}

static struct vb2_buffer *avs2_get_ref_buf(struct rkvdec_ctx *ctx,
					   struct rkvdec_avs2_run *run,
					   unsigned int i)
{
	struct vb2_queue *cap_q = &ctx->fh.m2m_ctx->cap_q_ctx.q;

	return vb2_find_buffer(cap_q, run->pic->ref_pics[i].timestamp);
}

static void rkvdec_avs2_write_regs(struct rkvdec_ctx *ctx)
{
	struct rkvdec_core *core = ctx->core;
	struct rkvdec_avs2_ctx *avs2_ctx = ctx->priv;
	struct rkvdec_vdpu381_regs_avs2 *regs = &avs2_ctx->regs;

	rkvdec_memcpy_toio(core->regs + OFFSET_COMMON_REGS, &regs->common,
			   sizeof(regs->common));
	rkvdec_memcpy_toio(core->regs + OFFSET_CODEC_PARAMS_REGS,
			   &regs->avs2_param, sizeof(regs->avs2_param));
	rkvdec_memcpy_toio(core->regs + OFFSET_COMMON_ADDR_REGS,
			   &regs->common_addr, sizeof(regs->common_addr));
	rkvdec_memcpy_toio(core->regs + OFFSET_CODEC_ADDR_REGS,
			   &regs->avs2_addr, sizeof(regs->avs2_addr));
	rkvdec_memcpy_toio(core->regs + OFFSET_POC_HIGHBIT_REGS,
			   &regs->avs2_highpoc, sizeof(regs->avs2_highpoc));
}

static void config_registers(struct rkvdec_ctx *ctx, struct rkvdec_avs2_run *run)
{
	const struct v4l2_ctrl_avs2_picture *pic = run->pic;
	struct rkvdec_avs2_ctx *avs2_ctx = ctx->priv;
	struct rkvdec_vdpu381_regs_avs2 *regs = &avs2_ctx->regs;
	const struct rkvdec_aux_buf *tbl = &avs2_ctx->priv_tbl[ctx->job->slot];
	const struct v4l2_pix_format_mplane *dst_fmt = &ctx->decoded_fmt.fmt.pix_mp;
	struct vb2_v4l2_buffer *src_buf = run->base.bufs.src;
	struct vb2_v4l2_buffer *dst_buf = run->base.bufs.dst;
	struct vb2_buffer *first_ref = NULL;
	dma_addr_t rlc_addr, dst_addr;
	u32 hor_virstride, y_virstride, pixels, stream_len;
	u8 pic_type = avs2_hw_pic_type(pic);
	unsigned int i;

	memset(regs, 0, sizeof(*regs));

	regs->common.reg009_dec_mode.dec_mode = VDPU381_MODE_AVS2;

	regs->common.reg011_important_en.buf_empty_en = 1;
	regs->common.reg011_important_en.dec_clkgate_e = 1;
	regs->common.reg011_important_en.dec_timeout_e = 1;

	regs->common.reg012_secondary_en.wr_ddr_align_en = 1;
	regs->common.reg012_secondary_en.colmv_compress_en = 1;
	regs->common.reg012_secondary_en.collect_info_en = 1;
	regs->common.reg012_secondary_en.scanlist_addr_valid_en = 1;

	regs->common.reg013_en_mode_set.timeout_mode = 1;
	regs->common.reg013_en_mode_set.stmerror_waitdecfifo_empty = 1;
	regs->common.reg013_en_mode_set.cur_pic_is_idr =
		pic_type == VDPU381_AVS2_PIC_I || pic_type == VDPU381_AVS2_PIC_G ||
		pic_type == VDPU381_AVS2_PIC_GB;

	/*
	 * The core starts at the first slice start code, from a 16-byte
	 * aligned address and a bit offset.
	 */
	rlc_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0) +
		   pic->data_offset;
	regs->common.reg015_stream_param_set.strm_start_bit = 8 * (rlc_addr & 15);
	rlc_addr &= ~(dma_addr_t)15;
	stream_len = vb2_get_plane_payload(&src_buf->vb2_buf, 0) -
		     ALIGN_DOWN(pic->data_offset, 16);
	regs->common.reg016_stream_len = stream_len;

	hor_virstride = dst_fmt->plane_fmt[0].bytesperline;
	y_virstride = hor_virstride * dst_fmt->height;
	regs->common.reg018_y_hor_stride.y_hor_virstride = hor_virstride / 16;
	regs->common.reg019_uv_hor_stride.uv_hor_virstride = hor_virstride / 16;
	regs->common.reg020_y_stride.y_virstride = y_virstride / 16;

	regs->common.reg026_block_gating_en.inter_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.filterd_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.strmd_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.mcp_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.busifd_auto_gating_e = 0;
	regs->common.reg026_block_gating_en.dec_ctrl_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.intra_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.mc_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.transd_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.sram_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.cru_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.reg_cfg_gating_en = 1;

	pixels = dst_fmt->height * dst_fmt->width;
	if (pixels < RKVDEC_1080P_PIXELS)
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_1080p;
	else if (pixels < RKVDEC_4K_PIXELS)
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_4K;
	else if (pixels < RKVDEC_8K_PIXELS)
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_8K;
	else
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_MAX;

	rkvdec_vdpu381_set_ccu_regs(ctx, &regs->common);

	regs->avs2_param.reg065_cur_poc = pic->poc;
	rkvdec_job_add_poc(ctx, pic->poc, false);

	/* A missing reference is replaced with the first one found. */
	for (i = 0; i < pic->num_ref_pics && !first_ref; i++)
		first_ref = avs2_get_ref_buf(ctx, run, i);
	if (!first_ref)
		first_ref = &dst_buf->vb2_buf;

	for (i = 0; i < pic->num_ref_pics; i++) {
		struct vb2_buffer *ref = avs2_get_ref_buf(ctx, run, i) ?: first_ref;
		dma_addr_t ref_dma = vb2_dma_contig_plane_dma_addr(ref, 0);

		rkvdec_job_add_ref(ctx, ref);
		rkvdec_job_add_poc(ctx, pic->ref_pics[i].poc, true);

		regs->avs2_addr.reg164_180_ref_base[i] = ref_dma;
		regs->avs2_addr.reg182_198_colmv_base[i] = ref_dma + ctx->colmv_offset;
		regs->avs2_param.reg067_098_ref_poc[i] = pic->ref_pics[i].poc;
		regs->avs2_param.reg099_100_ref_info[i] = VDPU381_AVS2_REF_USED;
	}

	regs->common_addr.rlc_base = rlc_addr;
	regs->common_addr.rlcwrite_base = rlc_addr;

	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	regs->common_addr.decout_base = dst_addr;
	regs->common_addr.error_ref_base = dst_addr;
	regs->common_addr.colmv_cur_base = dst_addr + ctx->colmv_offset;

	for (i = 0; i < rkvdec_rcb_buf_count(ctx); i++)
		regs->common_addr.rcb_base[i] = rkvdec_rcb_buf_dma_addr(ctx, i);

	/* The header table length, in 128-bit units minus one */
	regs->avs2_param.reg105_head_len.head_len = AVS2_HEADER_SIZE / 16 - 1;
	regs->avs2_addr.reg161_pps_base = tbl->dma +
		offsetof(struct rkvdec_avs2_priv_tbl, header);
	regs->avs2_addr.reg181_scanlist_addr = tbl->dma +
		offsetof(struct rkvdec_avs2_priv_tbl, wq_matrix);

	rkvdec_avs2_write_regs(ctx);
}

static int rkvdec_avs2_validate_seq(struct rkvdec_ctx *ctx,
				    const struct v4l2_ctrl_avs2_sequence *seq)
{
	/* Only 4:2:0 */
	if (seq->chroma_format != 1)
		return -EINVAL;

	/* Only frame pictures */
	if (seq->flags & V4L2_AVS2_SEQUENCE_FLAG_FIELD_CODED_SEQUENCE)
		return -EINVAL;

	/* The pictures are written with the coded sample precision. */
	if (seq->sample_precision > seq->encoding_precision)
		return -EINVAL;

	if (seq->horizontal_size > ctx->coded_fmt.fmt.pix_mp.width ||
	    seq->vertical_size > ctx->coded_fmt.fmt.pix_mp.height)
		return -EINVAL;

	return 0;
}

static void rkvdec_avs2_run_preamble(struct rkvdec_ctx *ctx,
				     struct rkvdec_avs2_run *run)
{
	struct v4l2_ctrl *ctrl;

	rkvdec_run_preamble(ctx, &run->base);

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_AVS2_SEQUENCE);
	run->seq = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_AVS2_PICTURE);
	run->pic = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_AVS2_WEIGHT_QUANT);
	run->wq = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_AVS2_ALF);
	run->alf = ctrl ? ctrl->p_cur.p : NULL;
}

static int rkvdec_avs2_run(struct rkvdec_ctx *ctx)
{
	struct rkvdec_core *core = ctx->core;
	struct rkvdec_avs2_ctx *avs2_ctx = ctx->priv;
	struct rkvdec_avs2_priv_tbl *tbl = avs2_ctx->priv_tbl[ctx->job->slot].cpu;
	struct rkvdec_avs2_run run;
	int ret;

	rkvdec_avs2_run_preamble(ctx, &run);

	if (WARN_ON(!run.seq || !run.pic || !run.wq || !run.alf) ||
	    run.pic->data_offset >= vb2_get_plane_payload(&run.base.bufs.src->vb2_buf, 0) ||
	    rkvdec_avs2_validate_seq(ctx, run.seq)) {
		rkvdec_run_postamble(ctx, &run.base);
		return -EINVAL;
	}

	assemble_hw_header(&run, tbl->header);
	if (run.pic->flags & V4L2_AVS2_PICTURE_FLAG_WEIGHT_QUANT_ENABLE)
		assemble_hw_wq_matrix(&run, tbl->wq_matrix);

	config_registers(ctx, &run);

	rkvdec_run_postamble(ctx, &run.base);

	/* Before the watchdog: this may wait for the earlier frames. */
	ret = rkvdec_job_wait_poc_alias(ctx);
	if (ret)
		return ret;

	rkvdec_schedule_watchdog(core, avs2_ctx->regs.common.reg032_timeout_threshold);

	rkvdec_ccu_start(ctx);
	writel(VDPU381_DEC_E_BIT, core->regs + VDPU381_REG_DEC_E);

	return 0;
}

static int rkvdec_avs2_start(struct rkvdec_ctx *ctx)
{
	struct rkvdec_dev *rkvdec = ctx->dev;
	struct rkvdec_avs2_ctx *avs2_ctx;
	struct v4l2_ctrl *ctrl;
	unsigned int i;
	int ret;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_AVS2_SEQUENCE);
	if (!ctrl)
		return -EINVAL;

	ret = rkvdec_avs2_validate_seq(ctx, ctrl->p_new.p_avs2_sequence);
	if (ret)
		return ret;

	avs2_ctx = kzalloc_obj(*avs2_ctx);
	if (!avs2_ctx)
		return -ENOMEM;

	for (i = 0; i < ctx->num_job_slots; i++) {
		struct rkvdec_aux_buf *buf = &avs2_ctx->priv_tbl[i];

		buf->size = sizeof(struct rkvdec_avs2_priv_tbl);
		buf->cpu = dma_alloc_coherent(rkvdec->main_core->dev, buf->size,
					      &buf->dma, GFP_KERNEL);
		if (!buf->cpu) {
			ret = -ENOMEM;
			goto err_free_tbl;
		}
	}

	ctx->priv = avs2_ctx;
	return 0;

err_free_tbl:
	while (i--) {
		struct rkvdec_aux_buf *buf = &avs2_ctx->priv_tbl[i];

		dma_free_coherent(rkvdec->main_core->dev, buf->size, buf->cpu,
				  buf->dma);
	}
	kfree(avs2_ctx);
	return ret;
}

static void rkvdec_avs2_stop(struct rkvdec_ctx *ctx)
{
	struct rkvdec_avs2_ctx *avs2_ctx = ctx->priv;
	struct rkvdec_dev *rkvdec = ctx->dev;
	unsigned int i;

	for (i = 0; i < ctx->num_job_slots; i++) {
		struct rkvdec_aux_buf *buf = &avs2_ctx->priv_tbl[i];

		dma_free_coherent(rkvdec->main_core->dev, buf->size, buf->cpu,
				  buf->dma);
	}
	kfree(avs2_ctx);
}

static int rkvdec_avs2_adjust_fmt(struct rkvdec_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 2;

	return 0;
}

static int rkvdec_avs2_try_ctrl(struct rkvdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_AVS2_SEQUENCE)
		return rkvdec_avs2_validate_seq(ctx, ctrl->p_new.p_avs2_sequence);

	return 0;
}

static enum rkvdec_image_fmt rkvdec_avs2_get_image_fmt(struct rkvdec_ctx *ctx,
							struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_avs2_sequence *seq = ctrl->p_new.p_avs2_sequence;

	if (ctrl->id != V4L2_CID_STATELESS_AVS2_SEQUENCE)
		return RKVDEC_IMG_FMT_ANY;

	if (seq->encoding_precision == 2)
		return RKVDEC_IMG_FMT_420_10BIT;

	return RKVDEC_IMG_FMT_420_8BIT;
}

const struct rkvdec_coded_fmt_ops rkvdec_vdpu381_avs2_fmt_ops = {
	.adjust_fmt = rkvdec_avs2_adjust_fmt,
	.start = rkvdec_avs2_start,
	.stop = rkvdec_avs2_stop,
	.run = rkvdec_avs2_run,
	.try_ctrl = rkvdec_avs2_try_ctrl,
	.get_image_fmt = rkvdec_avs2_get_image_fmt,
};
