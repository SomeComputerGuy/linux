// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver: VPU2 (VDPU2) MPEG-4 Part 2, H.263 and
 * Sorenson Spark decoding
 */

#include <media/v4l2-mem2mem.h>
#include "hantro.h"
#include "hantro_hw.h"

#define VDPU_SWREG(nr)			((nr) * 4)

#define VDPU_REG_QTABLE_BASE		VDPU_SWREG(61)
#define VDPU_REG_DIR_MV_BASE		VDPU_SWREG(62)
#define VDPU_REG_DEC_OUT_BASE		VDPU_SWREG(63)
#define VDPU_REG_RLC_VLC_BASE		VDPU_SWREG(64)
#define VDPU_REG_REFER0_BASE		VDPU_SWREG(131)
#define VDPU_REG_REFER2_BASE		VDPU_SWREG(134)
#define VDPU_REG_REFER3_BASE		VDPU_SWREG(135)
#define VDPU_REG_REFER1_BASE		VDPU_SWREG(148)

/* swreg 50 */
#define VDPU_REG_FILTERING_DIS(v)	((v) ? BIT(8) : 0)
/* swreg 51 */
#define VDPU_REG_INIT_QP(v)		(((v) << 25) & GENMASK(30, 25))
#define VDPU_REG_STREAM_LEN(v)		(((v) << 0) & GENMASK(23, 0))
/* swreg 52 */
#define VDPU_REG_APF_THRESHOLD(v)	(((v) << 17) & GENMASK(30, 17))
/* swreg 53 */
#define VDPU_REG_DEC_MODE(v)		(((v) << 0) & GENMASK(3, 0))
#define VDPU_DEC_MODE_MPEG4		1
#define VDPU_DEC_MODE_H263		2
/* swreg 54 */
#define VDPU_REG_DEC_STRENDIAN_E(v)	((v) ? BIT(5) : 0)
#define VDPU_REG_DEC_STRSWAP32_E(v)	((v) ? BIT(4) : 0)
#define VDPU_REG_DEC_OUTSWAP32_E(v)	((v) ? BIT(3) : 0)
#define VDPU_REG_DEC_INSWAP32_E(v)	((v) ? BIT(2) : 0)
#define VDPU_REG_DEC_OUT_ENDIAN(v)	((v) ? BIT(1) : 0)
#define VDPU_REG_DEC_IN_ENDIAN(v)	((v) ? BIT(0) : 0)
/* swreg 56 */
#define VDPU_REG_DEC_MAX_BURST(v)	(((v) << 16) & GENMASK(20, 16))
/* swreg 57 */
#define VDPU_REG_PIC_INTERLACE_E(v)	((v) ? BIT(17) : 0)
#define VDPU_REG_PIC_B_E(v)		((v) ? BIT(15) : 0)
#define VDPU_REG_PIC_INTER_E(v)		((v) ? BIT(14) : 0)
#define VDPU_REG_SORENSON_E(v)		((v) ? BIT(11) : 0)
#define VDPU_REG_WRITE_MVS_E(v)		((v) ? BIT(10) : 0)
#define VDPU_REG_DEC_TIMEOUT_E(v)	((v) ? BIT(5) : 0)
#define VDPU_REG_DEC_CLK_GATE_E(v)	((v) ? BIT(4) : 0)
#define VDPU_REG_DEC_E(v)		((v) ? BIT(0) : 0)
/* swreg 59 and 153: quarter sample interpolation filter */
#define VDPU_REG_PRED_FLT_TAP_0_0(v)	(((v) & 0x3ff) << 22)
#define VDPU_REG_PRED_FLT_TAP_0_1(v)	(((v) & 0x3ff) << 12)
#define VDPU_REG_PRED_FLT_TAP_0_2(v)	(((v) & 0x3ff) << 2)
#define VDPU_REG_PRED_FLT_TAP_0_3(v)	(((v) & 0x3ff) << 22)
/* swreg 120 */
#define VDPU_REG_PIC_MB_WIDTH(v)	(((v) << 23) & GENMASK(31, 23))
#define VDPU_REG_MB_WIDTH_OFF(v)	(((v) << 19) & GENMASK(22, 19))
#define VDPU_REG_PIC_MB_HEIGHT_P(v)	(((v) << 11) & GENMASK(18, 11))
#define VDPU_REG_MB_HEIGHT_OFF(v)	(((v) << 7) & GENMASK(10, 7))
#define VDPU_REG_TOPFIELDFIRST_E(v)	((v) ? BIT(5) : 0)
/* swreg 122 */
#define VDPU_REG_STRM_START_BIT(v)	(((v) << 26) & GENMASK(31, 26))
#define VDPU_REG_SYNC_MARKER_E(v)	((v) ? BIT(25) : 0)
#define VDPU_REG_QUANT_TYPE_1_E(v)	((v) ? BIT(24) : 0)
#define VDPU_REG_INTRADC_VLC_THR(v)	(((v) << 16) & GENMASK(18, 16))
#define VDPU_REG_VOP_TIME_INCR(v)	(((v) << 0) & GENMASK(15, 0))
/* swreg 136 */
#define VDPU_REG_ALT_SCAN_FLAG_E(v)	((v) ? BIT(19) : 0)
#define VDPU_REG_FCODE_FWD_HOR(v)	(((v) << 15) & GENMASK(18, 15))
#define VDPU_REG_FCODE_FWD_VER(v)	(((v) << 11) & GENMASK(14, 11))
#define VDPU_REG_FCODE_BWD_HOR(v)	(((v) << 7) & GENMASK(10, 7))
#define VDPU_REG_FCODE_BWD_VER(v)	(((v) << 3) & GENMASK(6, 3))
#define VDPU_REG_QUARTER_SAMPLE(v)	((v) ? BIT(2) : 0)
#define VDPU_REG_ROUNDING(v)		((v) ? BIT(1) : 0)
#define VDPU_REG_PREV_ANC_TYPE(v)	((v) ? BIT(0) : 0)
/* swregs 137-139: TRB / TRD ratios of the direct mode */
#define VDPU_REG_TRB_PER_TRD_D0		VDPU_SWREG(137)
#define VDPU_REG_TRB_PER_TRD_DM1	VDPU_SWREG(138)
#define VDPU_REG_TRB_PER_TRD_D1		VDPU_SWREG(139)

/*
 * Writes the registers which are common to both modes: everything but the
 * picture and the buffers.
 *
 * The decoder register range is shared by all decoding modes, and the
 * vendor driver programs all of it for every picture. Clear the registers
 * which these modes do not use, so that no value left by a job of another
 * codec takes effect. The interrupt status (55), enable (57), soft reset
 * (58) and the read-only registers (66-70) are left alone.
 */
static void rockchip_vpu2_mpeg4_h263_common(struct hantro_dev *vpu,
					    unsigned int mode, u8 qp,
					    dma_addr_t strm, u32 strm_len,
					    u32 start_bit)
{
	unsigned int i;
	u32 reg;

	for (i = 50; i <= 158; i++) {
		if (i == 55 || i == 57 || i == 58 || (i >= 66 && i <= 70))
			continue;
		vdpu_write_relaxed(vpu, 0, VDPU_SWREG(i));
	}

	vdpu_write_relaxed(vpu, VDPU_REG_FILTERING_DIS(1), VDPU_SWREG(50));

	reg = VDPU_REG_INIT_QP(qp) | VDPU_REG_STREAM_LEN(strm_len);
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(51));

	vdpu_write_relaxed(vpu, VDPU_REG_APF_THRESHOLD(1), VDPU_SWREG(52));
	vdpu_write_relaxed(vpu, VDPU_REG_DEC_MODE(mode), VDPU_SWREG(53));

	reg = VDPU_REG_DEC_STRENDIAN_E(1) |
	      VDPU_REG_DEC_STRSWAP32_E(1) |
	      VDPU_REG_DEC_OUTSWAP32_E(1) |
	      VDPU_REG_DEC_INSWAP32_E(1) |
	      VDPU_REG_DEC_OUT_ENDIAN(1) |
	      VDPU_REG_DEC_IN_ENDIAN(1);
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(54));

	vdpu_write_relaxed(vpu, VDPU_REG_DEC_MAX_BURST(16), VDPU_SWREG(56));

	/* The quarter sample filter: -1, 3, -6, 20, 20, -6, 3, -1 */
	reg = VDPU_REG_PRED_FLT_TAP_0_0(-1) |
	      VDPU_REG_PRED_FLT_TAP_0_1(3) |
	      VDPU_REG_PRED_FLT_TAP_0_2(-6);
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(59));
	vdpu_write_relaxed(vpu, VDPU_REG_PRED_FLT_TAP_0_3(20), VDPU_SWREG(153));

	vdpu_write_relaxed(vpu, strm, VDPU_REG_RLC_VLC_BASE);
}

/*
 * The hardware starts decoding at the first macroblock, from a 64-bit
 * aligned address and a bit offset into it. data_bit_offset counts from
 * the start code at the beginning of the OUTPUT buffer.
 */
static int rockchip_vpu2_mpeg4_h263_stream(struct vb2_buffer *src_buf,
					   u32 data_bit_offset,
					   dma_addr_t *strm, u32 *strm_len,
					   u32 *start_bit)
{
	u32 payload = vb2_get_plane_payload(src_buf, 0);
	u32 offset = (data_bit_offset / 8) & ~7;

	if (data_bit_offset / 8 >= payload)
		return -EINVAL;

	*strm = vb2_dma_contig_plane_dma_addr(src_buf, 0) + offset;
	*strm_len = payload - offset;
	*start_bit = data_bit_offset & 63;

	return 0;
}

static void rockchip_vpu2_mpeg4_h263_start(struct hantro_ctx *ctx, u32 reg57)
{
	struct hantro_dev *vpu = ctx->dev;

	reg57 |= VDPU_REG_DEC_TIMEOUT_E(1) | VDPU_REG_DEC_CLK_GATE_E(1);
	vdpu_write_relaxed(vpu, reg57, VDPU_SWREG(57));

	/* Kick the watchdog and start decoding */
	hantro_end_prepare_run(ctx);

	vdpu_write(vpu, reg57 | VDPU_REG_DEC_E(1), VDPU_SWREG(57));
}

int rockchip_vpu2_mpeg4_dec_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	const struct v4l2_ctrl_mpeg4_vol *vol;
	const struct v4l2_ctrl_mpeg4_vop *vop;
	const struct v4l2_ctrl_mpeg4_quantisation *quant;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t dst_addr, fwd_addr = 0, bwd_addr = 0, mv_addr, strm;
	unsigned int width, height, mv_offset;
	u32 strm_len, start_bit, quirks = 0, reg;
	u32 *quirks_ctrl;
	bool exact, b_vop;
	int ret;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	hantro_start_prepare_run(ctx);

	vol = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG4_VOL);
	vop = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG4_VOP);
	quant = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG4_QUANTISATION);
	quirks_ctrl = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG4_QUIRKS);
	if (quirks_ctrl)
		quirks = *quirks_ctrl;

	width = vol->video_object_layer_width;
	height = vol->video_object_layer_height;
	if (!width || !height || width > ctx->src_fmt.width ||
	    height > ctx->src_fmt.height)
		return -EINVAL;

	ret = rockchip_vpu2_mpeg4_h263_stream(&src_buf->vb2_buf,
					      vop->data_bit_offset, &strm,
					      &strm_len, &start_bit);
	if (ret)
		return ret;

	rockchip_vpu2_mpeg4_h263_common(vpu, VDPU_DEC_MODE_MPEG4,
					vop->vop_quant, strm, strm_len,
					start_bit);

	b_vop = vop->vop_coding_type == V4L2_MPEG4_VOP_CODING_TYPE_B;

	/*
	 * Some encoders extend the reference VOPs from their exact size
	 * instead of from the macroblock aligned one: give the hardware the
	 * size of the last macroblock column and row then.
	 */
	exact = quirks & V4L2_MPEG4_QUIRK_EDGE_EXACT_SIZE;
	reg = VDPU_REG_PIC_MB_WIDTH(MB_WIDTH(width)) |
	      VDPU_REG_MB_WIDTH_OFF(exact ? width & 15 : 0) |
	      VDPU_REG_PIC_MB_HEIGHT_P(MB_HEIGHT(height)) |
	      VDPU_REG_MB_HEIGHT_OFF(exact ? height & 15 : 0) |
	      VDPU_REG_TOPFIELDFIRST_E((vol->flags & V4L2_MPEG4_VOL_FLAG_INTERLACED) &&
				       (vop->flags & V4L2_MPEG4_VOP_FLAG_TOP_FIELD_FIRST));
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(120));

	reg = VDPU_REG_STRM_START_BIT(start_bit) |
	      VDPU_REG_SYNC_MARKER_E(!(vol->flags & V4L2_MPEG4_VOL_FLAG_RESYNC_MARKER_DISABLE)) |
	      VDPU_REG_QUANT_TYPE_1_E(vol->flags & V4L2_MPEG4_VOL_FLAG_QUANT_TYPE) |
	      VDPU_REG_INTRADC_VLC_THR(vop->intra_dc_vlc_thr) |
	      VDPU_REG_VOP_TIME_INCR(vol->vop_time_increment_resolution);
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(122));

	reg = VDPU_REG_ALT_SCAN_FLAG_E(vop->flags & V4L2_MPEG4_VOP_FLAG_ALTERNATE_VERTICAL_SCAN) |
	      VDPU_REG_FCODE_FWD_HOR(vop->vop_fcode_forward) |
	      VDPU_REG_FCODE_FWD_VER(vop->vop_fcode_forward) |
	      VDPU_REG_QUARTER_SAMPLE(vol->flags & V4L2_MPEG4_VOL_FLAG_QUARTER_SAMPLE);
	if (b_vop)
		reg |= VDPU_REG_FCODE_BWD_HOR(vop->vop_fcode_backward) |
		       VDPU_REG_FCODE_BWD_VER(vop->vop_fcode_backward) |
		       VDPU_REG_PREV_ANC_TYPE(vop->backward_ref_vop_coding_type !=
					      V4L2_MPEG4_VOP_CODING_TYPE_I);
	else
		reg |= VDPU_REG_ROUNDING(vop->flags & V4L2_MPEG4_VOP_FLAG_ROUNDING_TYPE);
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(136));

	hantro_mpeg4_dec_copy_qtable(ctx->mpeg4_dec.qtable.cpu, quant);
	vdpu_write_relaxed(vpu, ctx->mpeg4_dec.qtable.dma, VDPU_REG_QTABLE_BASE);

	/* Buffers */
	dst_addr = hantro_get_dec_buf_addr(ctx, &dst_buf->vb2_buf);
	vdpu_write_relaxed(vpu, dst_addr, VDPU_REG_DEC_OUT_BASE);

	switch (vop->vop_coding_type) {
	case V4L2_MPEG4_VOP_CODING_TYPE_B:
		bwd_addr = hantro_get_ref(ctx, vop->backward_ref_ts);
		fallthrough;
	case V4L2_MPEG4_VOP_CODING_TYPE_P:
		fwd_addr = hantro_get_ref(ctx, vop->forward_ref_ts);
		break;
	}
	if (!fwd_addr)
		fwd_addr = dst_addr;
	if (!bwd_addr)
		bwd_addr = dst_addr;

	vdpu_write_relaxed(vpu, fwd_addr, VDPU_REG_REFER0_BASE);
	vdpu_write_relaxed(vpu, fwd_addr, VDPU_REG_REFER1_BASE);
	vdpu_write_relaxed(vpu, b_vop ? bwd_addr : dst_addr, VDPU_REG_REFER2_BASE);
	vdpu_write_relaxed(vpu, b_vop ? bwd_addr : dst_addr, VDPU_REG_REFER3_BASE);

	/*
	 * P-VOPs store their motion vectors after their own picture, B-VOPs
	 * read those of their backward reference for the direct mode.
	 */
	mv_offset = 384 * MB_WIDTH(ctx->src_fmt.width) *
		    MB_HEIGHT(ctx->src_fmt.height);
	mv_addr = (b_vop ? bwd_addr : dst_addr) + mv_offset;
	vdpu_write_relaxed(vpu, mv_addr, VDPU_REG_DIR_MV_BASE);

	if (b_vop) {
		vdpu_write_relaxed(vpu,
				   hantro_mpeg4_direct_ratio(vop->trb, vop->trd),
				   VDPU_REG_TRB_PER_TRD_D0);
		vdpu_write_relaxed(vpu,
				   hantro_mpeg4_direct_ratio(vop->trb_field + 1,
							     vop->trd_field + 1),
				   VDPU_REG_TRB_PER_TRD_D1);
		vdpu_write_relaxed(vpu,
				   hantro_mpeg4_direct_ratio(vop->trb_field - 1,
							     vop->trd_field - 1),
				   VDPU_REG_TRB_PER_TRD_DM1);
	}

	reg = VDPU_REG_PIC_INTERLACE_E(vol->flags & V4L2_MPEG4_VOL_FLAG_INTERLACED) |
	      VDPU_REG_PIC_B_E(b_vop) |
	      VDPU_REG_PIC_INTER_E(vop->vop_coding_type != V4L2_MPEG4_VOP_CODING_TYPE_I) |
	      VDPU_REG_WRITE_MVS_E(vop->vop_coding_type == V4L2_MPEG4_VOP_CODING_TYPE_P);
	rockchip_vpu2_mpeg4_h263_start(ctx, reg);

	return 0;
}

int rockchip_vpu2_h263_dec_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	const struct v4l2_ctrl_h263_picture *pic;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t dst_addr, fwd_addr = 0, strm;
	u32 strm_len, start_bit, reg;
	bool spark, inter;
	int ret;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	hantro_start_prepare_run(ctx);

	pic = hantro_get_ctrl(ctx, V4L2_CID_STATELESS_H263_PICTURE);
	spark = ctx->vpu_src_fmt->fourcc == V4L2_PIX_FMT_SPK_SLICE;

	if (!pic->width || !pic->height || pic->width > ctx->src_fmt.width ||
	    pic->height > ctx->src_fmt.height || (pic->spk_version && !spark))
		return -EINVAL;

	ret = rockchip_vpu2_mpeg4_h263_stream(&src_buf->vb2_buf,
					      pic->data_bit_offset, &strm,
					      &strm_len, &start_bit);
	if (ret)
		return ret;

	rockchip_vpu2_mpeg4_h263_common(vpu, VDPU_DEC_MODE_H263, pic->pquant,
					strm, strm_len, start_bit);

	inter = pic->picture_coding_type == V4L2_H263_PICTURE_CODING_TYPE_P;

	/* References are extended from the macroblock aligned size. */
	reg = VDPU_REG_PIC_MB_WIDTH(MB_WIDTH(pic->width)) |
	      VDPU_REG_PIC_MB_HEIGHT_P(MB_HEIGHT(pic->height));
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(120));

	/* GOB headers are parsed by the hardware. */
	reg = VDPU_REG_STRM_START_BIT(start_bit) |
	      VDPU_REG_SYNC_MARKER_E(1);
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(122));

	reg = VDPU_REG_FCODE_FWD_HOR(1) |
	      VDPU_REG_FCODE_FWD_VER(1) |
	      VDPU_REG_ROUNDING(inter && (pic->flags & V4L2_H263_PICTURE_FLAG_ROUNDING_TYPE));
	vdpu_write_relaxed(vpu, reg, VDPU_SWREG(136));

	dst_addr = hantro_get_dec_buf_addr(ctx, &dst_buf->vb2_buf);
	vdpu_write_relaxed(vpu, dst_addr, VDPU_REG_DEC_OUT_BASE);

	if (inter)
		fwd_addr = hantro_get_ref(ctx, pic->forward_ref_ts);
	if (!fwd_addr)
		fwd_addr = dst_addr;
	vdpu_write_relaxed(vpu, fwd_addr, VDPU_REG_REFER0_BASE);
	vdpu_write_relaxed(vpu, fwd_addr, VDPU_REG_REFER1_BASE);

	/*
	 * Sorenson Spark version 1 codes the escaped transform coefficients
	 * with its own 7 or 11 bit levels; version 0 is plain H.263.
	 */
	reg = VDPU_REG_PIC_INTER_E(inter) |
	      VDPU_REG_SORENSON_E(spark && pic->spk_version == 1);
	rockchip_vpu2_mpeg4_h263_start(ctx, reg);

	return 0;
}
