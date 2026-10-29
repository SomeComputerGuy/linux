// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver
 *
 * Rockchip VEPU2 H.264 stateless encoder
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *	Alpha Lin <Alpha.Lin@rock-chips.com>
 *	Jung Zhao <jung.zhao@rock-chips.com>
 *
 * The register programming, the mode decision tables and the CABAC table
 * layout come from rk3399_vpu_hw_h264e.c of the ChromeOS kernel
 * (chromiumos/third_party/kernel, chromeos-4.4 branch).
 *
 * The bitstream headers, the rate control and the reference handling are
 * done by the V4L2 H.264 stateless encode core:
 *
 * - The SPS, PPS and access unit delimiter are written by the CPU at the
 *   start of the destination buffer. The hardware appends to them: the
 *   stream base address must be 64-bit aligned, so the last bytes of the
 *   headers are passed in the STR_HDR_REM registers and the hardware starts
 *   writing after them, at STREAM_START_OFFSET bits.
 *
 * - The hardware writes the slice NAL unit, including its start code and
 *   slice header, which has no picture order count (pic_order_cnt_type 2)
 *   and a 16-bit frame_num. It has no reference list modification and no
 *   adaptive reference picture marking, and every picture is a reference,
 *   so the hardware can only encode I and P pictures with a single
 *   reference picture, with the sliding window process.
 *
 * - The hardware reads the picture to encode, a single reference picture
 *   and writes the reconstructed picture, both in a semi-planar 4:2:0
 *   format with the dimensions aligned to macroblocks. These are internal
 *   to the driver and managed by the encode core.
 */

#include <linux/bitfield.h>
#include <linux/unaligned.h>
#include <media/v4l2-h264-enc.h>
#include <media/v4l2-mem2mem.h>

#include "hantro.h"
#include "hantro_hw.h"
#include "rockchip_vpu2_regs.h"

#define VEPU2_H264_ENC_MAX_QP			51
/* The frame_num written by the hardware in slice headers has 16 bits. */
#define VEPU2_H264_ENC_LOG2_MAX_FRAME_NUM_MINUS4	12
/* Above this number of macroblocks, quarter-pixel motion search is off. */
#define VEPU2_H264_ENC_QPEL_MAX_MBS		3600

#define VEPU2_H264_PROFILE_IDC_BASELINE		66
#define VEPU2_H264_PROFILE_IDC_MAIN		77
#define VEPU2_H264_PROFILE_IDC_HIGH		100

/*
 * The CABAC table has 464 bytes for each QP and slice type, of which the
 * first 460 hold the context states.
 */
#define VEPU2_H264_CABAC_CTX_COUNT		460
#define VEPU2_H264_CABAC_CTX_STRIDE		464
#define VEPU2_H264_CABAC_TBL_SIZE		(52 * 2 * VEPU2_H264_CABAC_CTX_STRIDE)

/* The status register holds half of the sum of the macroblock QPs. */
#define VEPU2_REG_QP_SUM_DIV2_MASK		GENMASK(31, 11)

/* Mode decision tables, indexed by QP. */
static const u8 h264_prev_mode_favor[52] = {
	7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18,
	19, 20, 21, 22, 24, 25, 27, 29, 30, 32, 34, 36, 38, 41, 43, 46,
	49, 51, 55, 58, 61, 65, 69, 73, 78, 82, 87, 93, 98, 104, 110,
	117, 124, 132, 140
};

/* sqrt(2^((qp-12)/3))*8 */
static const u16 h264_diff_mv_penalty[52] = {
	2, 2, 3, 3, 3, 4, 4, 4, 5, 6,
	6, 7, 8, 9, 10, 11, 13, 14, 16, 18,
	20, 23, 26, 29, 32, 36, 40, 45, 51, 57,
	64, 72, 81, 91, 102, 114, 128, 144, 161, 181,
	203, 228, 256, 287, 323, 362, 406, 456, 512, 575,
	645, 724
};

/* 31*sqrt(2^((qp-12)/3))/4 */
static const u16 h264_diff_mv_penalty4p[52] = {
	2, 2, 2, 3, 3, 3, 4, 4, 5, 5,
	6, 7, 8, 9, 10, 11, 12, 14, 16, 17,
	20, 22, 25, 28, 31, 35, 39, 44, 49, 55,
	62, 70, 78, 88, 98, 110, 124, 139, 156, 175,
	197, 221, 248, 278, 312, 351, 394, 442, 496, 557,
	625, 701
};

static const u16 h264_intra16_favor[52] = {
	24, 24, 24, 26, 27, 30, 32, 35, 39, 43, 48, 53, 58, 64, 71, 78,
	85, 93, 102, 111, 121, 131, 142, 154, 167, 180, 195, 211, 229,
	248, 271, 296, 326, 361, 404, 457, 523, 607, 714, 852, 1034,
	1272, 1588, 2008, 2568, 3318, 4323, 5672, 7486, 9928, 13216,
	17648
};

static const u16 h264_inter_favor[52] = {
	40, 40, 41, 42, 43, 44, 45, 48, 51, 53, 55, 60, 62, 67, 69, 72,
	78, 84, 90, 96, 110, 120, 135, 152, 170, 189, 210, 235, 265,
	297, 335, 376, 420, 470, 522, 572, 620, 670, 724, 770, 820,
	867, 915, 970, 1020, 1076, 1132, 1180, 1230, 1275, 1320, 1370
};

static const u8 h264_skip_sad_penalty[52] = {
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 233, 205, 182, 163,
	146, 132, 120, 109, 100,  92,  84,  78,  71,  66,  61,  56,  52,  48,
	44,  41,  38,  35,  32,  30,  27,  25,  23,  21,  19,  17,  15,  14,
	12,  11,   9,   8,   7,   5,   4,   3,   2,   1
};

static inline void vepu2_h264_set(struct hantro_h264_enc_hw_ctx *h264_ctx,
				  u32 reg, u32 val)
{
	h264_ctx->regs[reg / 4] = val;
}

/* Length of the signed Exp-Golomb code of a positive value. */
static unsigned int vepu2_h264_se_len(unsigned int val)
{
	return 2 * fls(val ? 2 * val : 1) - 1;
}

static unsigned int vepu2_h264_luma_size(struct hantro_ctx *ctx)
{
	return MB_WIDTH(ctx->src_fmt.width) * MB_DIM *
	       MB_HEIGHT(ctx->src_fmt.height) * MB_DIM;
}

/*
 * Fill the CABAC context initialization table: for each cabac_init_idc, QP
 * and slice type (intra, then inter), the initial state of the 460 contexts,
 * computed as in ITU-T H.264 section 9.3.1.1, as (pStateIdx << 1) | valMPS.
 * The hardware reads it as big-endian 64-bit words.
 */
static void vepu2_h264_enc_init_cabac_table(u8 *tbl)
{
	unsigned int idc, qp, inter, i, group;

	for (idc = 0; idc < 3; idc++) {
		u8 *t = tbl + idc * VEPU2_H264_CABAC_TBL_SIZE;

		for (qp = 0; qp <= VEPU2_H264_ENC_MAX_QP; qp++) {
			for (inter = 0; inter < 2; inter++) {
				group = inter ? idc + 1 : 0;

				for (i = 0; i < VEPU2_H264_CABAC_CTX_COUNT; i++) {
					unsigned int k = group *
						VEPU2_H264_CABAC_CTX_COUNT + i;
					u32 word = hantro_h264_cabac_table[k / 2];
					u16 pair = k & 1 ? word & 0xffff :
							   word >> 16;
					int m = (s8)(pair >> 8);
					int n = (s8)(pair & 0xff);
					int state;

					state = clamp(((m * (int)qp) >> 4) + n,
						      1, 126);

					t[(qp * 2 + inter) *
					  VEPU2_H264_CABAC_CTX_STRIDE + i] =
						state <= 63 ? (63 - state) << 1 :
							      ((state - 64) << 1) | 1;
				}
			}
		}

		for (i = 0; i < VEPU2_H264_CABAC_TBL_SIZE; i += 8)
			put_unaligned_be64(get_unaligned_le64(t + i), t + i);
	}
}

static int
rockchip_vpu2_h264_enc_state_constrain(struct v4l2_h264_enc *enc,
				       struct v4l2_h264_enc_state *state)
{
	struct v4l2_ctrl_h264_sps *sps = &state->sps;
	struct v4l2_ctrl_h264_pps *pps = &state->pps;
	struct v4l2_ctrl_h264_encode_params *encode = &state->encode;

	/* SPS */

	switch (sps->profile_idc) {
	case VEPU2_H264_PROFILE_IDC_BASELINE:
	case VEPU2_H264_PROFILE_IDC_MAIN:
	case VEPU2_H264_PROFILE_IDC_HIGH:
		break;
	default:
		sps->profile_idc = VEPU2_H264_PROFILE_IDC_MAIN;
		break;
	}

	if (!sps->level_idc)
		sps->level_idc = 40;

	sps->chroma_format_idc = 1;
	sps->bit_depth_luma_minus8 = 0;
	sps->bit_depth_chroma_minus8 = 0;
	sps->flags &= ~(V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE |
			V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS |
			V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);
	sps->flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY |
		      V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;

	/* Hardware slice headers carry a 16-bit frame_num and no POC. */
	sps->log2_max_frame_num_minus4 = VEPU2_H264_ENC_LOG2_MAX_FRAME_NUM_MINUS4;
	sps->pic_order_cnt_type = 2;

	/* A single reference picture is supported. */
	if (sps->max_num_ref_frames > 1)
		sps->max_num_ref_frames = 1;

	/* PPS */

	if (sps->profile_idc == VEPU2_H264_PROFILE_IDC_BASELINE)
		pps->flags &= ~V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;

	if (sps->profile_idc != VEPU2_H264_PROFILE_IDC_HIGH)
		pps->flags &= ~V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;

	/* Hardware slice headers always have the deblocking filter syntax. */
	pps->flags |= V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;
	pps->num_ref_idx_l0_default_active_minus1 = 0;
	pps->num_ref_idx_l1_default_active_minus1 = 0;
	pps->pic_init_qp_minus26 = clamp_t(s8, pps->pic_init_qp_minus26,
					   -26, 25);
	pps->chroma_qp_index_offset = clamp_t(s8, pps->chroma_qp_index_offset,
					      -12, 12);

	/* Encode */

	/* Every picture is a reference, there are no long-term references. */
	if (!encode->nal_ref_idc)
		encode->nal_ref_idc = 1;

	encode->flags &= ~(V4L2_H264_ENCODE_FLAG_LONG_TERM_REFERENCE |
			   V4L2_H264_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE |
			   V4L2_H264_ENCODE_FLAG_FIELD_PIC |
			   V4L2_H264_ENCODE_FLAG_BOTTOM_FIELD |
			   V4L2_H264_ENCODE_FLAG_NO_OUTPUT_OF_PRIOR_PICS);
	encode->num_ref_idx_l0_active_minus1 = 0;
	encode->num_ref_idx_l1_active_minus1 = 0;
	encode->pic_order_cnt_lsb = 0;
	encode->delta_pic_order_cnt_bottom = 0;
	encode->delta_pic_order_cnt0 = 0;
	encode->delta_pic_order_cnt1 = 0;
	encode->frame_num &= GENMASK(VEPU2_H264_ENC_LOG2_MAX_FRAME_NUM_MINUS4 +
				     3, 0);

	/* The hardware has 4 bits for idr_pic_id. */
	encode->idr_pic_id &= 0xf;

	if (encode->cabac_init_idc > 2)
		encode->cabac_init_idc = 0;

	/* With a single slice, idc 2 is the same as idc 0. */
	if (encode->disable_deblocking_filter_idc != 1)
		encode->disable_deblocking_filter_idc = 0;

	encode->slice_alpha_c0_offset_div2 =
		clamp_t(s8, encode->slice_alpha_c0_offset_div2, -6, 6);
	encode->slice_beta_offset_div2 =
		clamp_t(s8, encode->slice_beta_offset_div2, -6, 6);

	return 0;
}

static int
rockchip_vpu2_h264_enc_rec_buffer_alloc(struct v4l2_h264_enc *enc,
					struct v4l2_h264_enc_rec_buffer *buffer)
{
	struct hantro_ctx *ctx = enc->private_data;
	struct hantro_aux_buf *rec_buf;

	rec_buf = kzalloc_obj(*rec_buf);
	if (!rec_buf)
		return -ENOMEM;

	/* Semi-planar 4:2:0, with the chroma plane after the luma one. */
	rec_buf->size = vepu2_h264_luma_size(ctx) * 3 / 2;
	rec_buf->attrs = DMA_ATTR_NO_KERNEL_MAPPING;
	rec_buf->cpu = dma_alloc_attrs(ctx->dev->dev, rec_buf->size,
				       &rec_buf->dma, GFP_KERNEL,
				       rec_buf->attrs);
	if (!rec_buf->cpu) {
		kfree(rec_buf);
		return -ENOMEM;
	}

	buffer->private_data = rec_buf;

	return 0;
}

static int
rockchip_vpu2_h264_enc_rec_buffer_free(struct v4l2_h264_enc *enc,
				       struct v4l2_h264_enc_rec_buffer *buffer)
{
	struct hantro_ctx *ctx = enc->private_data;
	struct hantro_aux_buf *rec_buf = buffer->private_data;

	if (!rec_buf)
		return -EINVAL;

	dma_free_attrs(ctx->dev->dev, rec_buf->size, rec_buf->cpu,
		       rec_buf->dma, rec_buf->attrs);
	kfree(rec_buf);
	buffer->private_data = NULL;

	return 0;
}

static const struct v4l2_h264_enc_ops rockchip_vpu2_h264_enc_ops = {
	.state_constrain	= rockchip_vpu2_h264_enc_state_constrain,
	.rec_buffer_alloc	= rockchip_vpu2_h264_enc_rec_buffer_alloc,
	.rec_buffer_free	= rockchip_vpu2_h264_enc_rec_buffer_free,
};

int rockchip_vpu2_h264_enc_init(struct hantro_ctx *ctx)
{
	struct hantro_h264_enc_hw_ctx *h264_ctx = &ctx->h264_enc;
	struct hantro_aux_buf *cabac_tbl = &h264_ctx->cabac_tbl;
	struct v4l2_h264_enc *enc = &h264_ctx->enc;
	struct device *dev = ctx->dev->dev;
	int ret;

	/*
	 * The hardware encodes whole macroblocks from the source picture, the
	 * coded size (the crop rectangle of the source) must fall in the last
	 * ones.
	 */
	if (ALIGN(ctx->dst_fmt.width, MB_DIM) != ctx->src_fmt.width ||
	    ALIGN(ctx->dst_fmt.height, MB_DIM) != ctx->src_fmt.height) {
		vpu_err("coded size %ux%u doesn't match the source size %ux%u\n",
			ctx->dst_fmt.width, ctx->dst_fmt.height,
			ctx->src_fmt.width, ctx->src_fmt.height);
		return -EINVAL;
	}

	memset(h264_ctx, 0, sizeof(*h264_ctx));

	cabac_tbl->size = 3 * VEPU2_H264_CABAC_TBL_SIZE;
	cabac_tbl->cpu = dma_alloc_coherent(dev, cabac_tbl->size,
					    &cabac_tbl->dma, GFP_KERNEL);
	if (!cabac_tbl->cpu)
		return -ENOMEM;

	vepu2_h264_enc_init_cabac_table(cabac_tbl->cpu);

	enc->ops = &rockchip_vpu2_h264_enc_ops;
	enc->private_data = ctx;
	enc->format_mplane = &ctx->dst_fmt;
	enc->timeperframe = &ctx->dst_timeperframe;
	enc->ctrl_handler = &ctx->ctrl_handler;
	enc->ref_slots_count_init = 1;
	enc->flags = V4L2_H264_ENC_FLAG_INTER_PRED |
		     V4L2_H264_ENC_FLAG_HW_SLICE_HEADER;

	ret = v4l2_h264_enc_init(enc);
	if (ret) {
		dma_free_coherent(dev, cabac_tbl->size, cabac_tbl->cpu,
				  cabac_tbl->dma);
		cabac_tbl->cpu = NULL;
	}

	return ret;
}

void rockchip_vpu2_h264_enc_exit(struct hantro_ctx *ctx)
{
	struct hantro_h264_enc_hw_ctx *h264_ctx = &ctx->h264_enc;
	struct hantro_aux_buf *cabac_tbl = &h264_ctx->cabac_tbl;

	v4l2_h264_enc_exit(&h264_ctx->enc);

	if (cabac_tbl->cpu)
		dma_free_coherent(ctx->dev->dev, cabac_tbl->size,
				  cabac_tbl->cpu, cabac_tbl->dma);
	cabac_tbl->cpu = NULL;
}

static int vepu2_h264_enc_set_buffers(struct hantro_ctx *ctx,
				      struct vb2_v4l2_buffer *src_buf,
				      struct vb2_v4l2_buffer *dst_buf)
{
	struct hantro_h264_enc_hw_ctx *h264_ctx = &ctx->h264_enc;
	struct v4l2_h264_enc *enc = &h264_ctx->enc;
	struct v4l2_h264_enc_ref *ref = &enc->ref;
	struct v4l2_pix_format_mplane *src_fmt = &ctx->src_fmt;
	unsigned int luma_size = vepu2_h264_luma_size(ctx);
	struct hantro_aux_buf *rec_buf, *ref_buf;
	unsigned int hdr_size, size;
	dma_addr_t dst_dma, src_dma[3];
	u32 hdr_rem_msb = 0, hdr_rem_lsb = 0;
	u8 *dst;
	int i;

	/* Source picture, the planes can be in a single buffer or not. */
	for (i = 0; i < src_fmt->num_planes; i++)
		src_dma[i] = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, i);

	if (src_fmt->num_planes == 1 &&
	    ctx->vpu_src_fmt->enc_fmt != ROCKCHIP_VPU_ENC_FMT_YUYV422 &&
	    ctx->vpu_src_fmt->enc_fmt != ROCKCHIP_VPU_ENC_FMT_UYVY422) {
		size = src_fmt->plane_fmt[0].bytesperline * src_fmt->height;
		src_dma[1] = src_dma[0] + size;
		src_dma[2] = src_dma[1] + size / 4;
	}

	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_IN_PLANE_0, src_dma[0]);
	if (ctx->vpu_src_fmt->enc_fmt == ROCKCHIP_VPU_ENC_FMT_YUV420SP ||
	    ctx->vpu_src_fmt->enc_fmt == ROCKCHIP_VPU_ENC_FMT_YUV420P)
		vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_IN_PLANE_1, src_dma[1]);
	if (ctx->vpu_src_fmt->enc_fmt == ROCKCHIP_VPU_ENC_FMT_YUV420P)
		vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_IN_PLANE_2, src_dma[2]);

	/*
	 * Output stream: the hardware appends to the headers written by the
	 * CPU, from a 64-bit aligned address. The bytes of the headers after
	 * that address go to the STR_HDR_REM registers.
	 */
	hdr_size = v4l2_h264_enc_rbsp_bytes_count(&enc->rbsp);
	h264_ctx->strm_offset = round_down(hdr_size, 8);
	size = vb2_plane_size(&dst_buf->vb2_buf, 0);
	if (size <= h264_ctx->strm_offset + 8)
		return -ENOSPC;

	dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	if (hdr_size > h264_ctx->strm_offset) {
		u8 rem[8] = { 0 };

		memcpy(rem, dst + h264_ctx->strm_offset,
		       hdr_size - h264_ctx->strm_offset);
		hdr_rem_msb = get_unaligned_be32(rem);
		hdr_rem_lsb = get_unaligned_be32(rem + 4);
	}

	dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_OUTPUT_STREAM,
		       dst_dma + h264_ctx->strm_offset);
	vepu2_h264_set(h264_ctx, VEPU_REG_STR_BUF_LIMIT,
		       size - h264_ctx->strm_offset);
	vepu2_h264_set(h264_ctx, VEPU_REG_STR_HDR_REM_MSB, hdr_rem_msb);
	vepu2_h264_set(h264_ctx, VEPU_REG_STR_HDR_REM_LSB, hdr_rem_lsb);

	/* Reconstructed picture and reference picture. */
	rec_buf = ref->buffer_current.private_data;
	ref_buf = rec_buf;
	if (ref->l0_active_count)
		ref_buf = ref->buffers[ref->l0[0].index].private_data;

	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_REC_LUMA, rec_buf->dma);
	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_REC_CHROMA,
		       rec_buf->dma + luma_size);
	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_REF_LUMA, ref_buf->dma);
	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_REF_CHROMA,
		       ref_buf->dma + luma_size);

	vepu2_h264_set(h264_ctx, VEPU_REG_ADDR_CABAC_TBL,
		       h264_ctx->cabac_tbl.dma +
		       enc->state_active.encode.cabac_init_idc *
		       VEPU2_H264_CABAC_TBL_SIZE);

	return (hdr_size - h264_ctx->strm_offset) * 8;
}

static void vepu2_h264_enc_set_params(struct hantro_ctx *ctx,
				      unsigned int first_free_bit)
{
	struct hantro_h264_enc_hw_ctx *h264_ctx = &ctx->h264_enc;
	struct v4l2_h264_enc *enc = &h264_ctx->enc;
	struct v4l2_h264_enc_state *state = &enc->state_active;
	const struct v4l2_ctrl_h264_pps *pps = &state->pps;
	const struct v4l2_ctrl_h264_encode_params *encode = &state->encode;
	struct v4l2_pix_format_mplane *src_fmt = &ctx->src_fmt;
	unsigned int mb_width = MB_WIDTH(src_fmt->width);
	unsigned int mb_height = MB_HEIGHT(src_fmt->height);
	unsigned int qp = enc->rc.qp;
	unsigned int overfill_r, overfill_b, row_length;
	unsigned int scaler, skip_penalty;
	u32 reg;
	int i;

	/* Source picture. */
	row_length = src_fmt->plane_fmt[0].bytesperline;
	if (ctx->vpu_src_fmt->enc_fmt == ROCKCHIP_VPU_ENC_FMT_YUYV422 ||
	    ctx->vpu_src_fmt->enc_fmt == ROCKCHIP_VPU_ENC_FMT_UYVY422)
		row_length /= 2;

	vepu2_h264_set(h264_ctx, VEPU_REG_INPUT_LUMA_INFO,
		       VEPU_REG_IN_IMG_CTRL_ROW_LEN(row_length));

	/* Crop inside the right-most and bottom-most macroblocks. */
	overfill_r = (src_fmt->width - ctx->dst_fmt.width) / 4;
	overfill_b = src_fmt->height - ctx->dst_fmt.height;

	scaler = max(1U, 200 / (mb_width + mb_height));
	skip_penalty = min(255U, h264_skip_sad_penalty[qp] * scaler);

	vepu2_h264_set(h264_ctx, VEPU_REG_ENC_OVER_FILL_STRM_OFFSET,
		       VEPU_REG_STREAM_START_OFFSET(first_free_bit) |
		       VEPU_REG_SKIP_MACROBLOCK_PENALTY(skip_penalty) |
		       VEPU_REG_IN_IMG_CTRL_OVRFLR_D4(overfill_r) |
		       VEPU_REG_IN_IMG_CTRL_OVRFLB(overfill_b));

	vepu2_h264_set(h264_ctx, VEPU_REG_ENC_CTRL1,
		       VEPU_REG_IN_IMG_CTRL_FMT(ctx->vpu_src_fmt->enc_fmt) |
		       VEPU_REG_IN_IMG_ROTATE_MODE(0));

	vepu2_h264_set(h264_ctx, VEPU_REG_DATA_ENDIAN,
		       VEPU_REG_OUTPUT_SWAP32 | VEPU_REG_OUTPUT_SWAP16 |
		       VEPU_REG_OUTPUT_SWAP8 | VEPU_REG_INPUT_SWAP8 |
		       VEPU_REG_INPUT_SWAP16 | VEPU_REG_INPUT_SWAP32);

	vepu2_h264_set(h264_ctx, VEPU_REG_AXI_CTRL,
		       VEPU_REG_AXI_CTRL_BURST_LEN(16));

	/* Coding tools. */
	reg = VEPU_REG_CABAC_INIT_IDC(encode->cabac_init_idc) |
	      VEPU_REG_H264_INTER4X4_MODE |
	      VEPU_REG_H264_SLICE_SIZE(0);
	if (mb_width * mb_height > VEPU2_H264_ENC_QPEL_MAX_MBS)
		reg |= VEPU_REG_DISABLE_QUARTER_PIXEL_MV;
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		reg |= VEPU_REG_ENTROPY_CODING_MODE;
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE)
		reg |= VEPU_REG_H264_TRANS8X8_MODE;
	vepu2_h264_set(h264_ctx, VEPU_REG_ENC_CTRL0, reg);

	/* Slice header and picture parameters. */
	reg = VEPU_REG_PPS_INIT_QP(pps->pic_init_qp_minus26 + 26) |
	      VEPU_REG_SLICE_FILTER_ALPHA(encode->slice_alpha_c0_offset_div2) |
	      VEPU_REG_SLICE_FILTER_BETA(encode->slice_beta_offset_div2) |
	      VEPU_REG_CHROMA_QP_OFFSET(pps->chroma_qp_index_offset) |
	      VEPU_REG_IDR_PIC_ID(encode->idr_pic_id);
	if (encode->disable_deblocking_filter_idc == 1)
		reg |= VEPU_REG_FILTER_DISABLE;
	if (pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED)
		reg |= VEPU_REG_CONSTRAINED_INTRA_PREDICTION;
	vepu2_h264_set(h264_ctx, VEPU_REG_ENC_CTRL2, reg);

	vepu2_h264_set(h264_ctx, VEPU_REG_ENC_CTRL3,
		       VEPU_REG_PPS_ID(pps->pic_parameter_set_id) |
		       VEPU_REG_INTRA_PRED_MODE(h264_prev_mode_favor[qp]) |
		       VEPU_REG_FRAME_NUM(encode->frame_num));

	/*
	 * Constant QP for the whole picture: the macroblock rate control,
	 * with its checkpoints and MAD based QP adjustment, is disabled.
	 */
	vepu2_h264_set(h264_ctx, VEPU_REG_QP_VAL,
		       VEPU_REG_H264_LUMA_INIT_QP(qp) |
		       VEPU_REG_H264_QP_MAX(qp) |
		       VEPU_REG_H264_QP_MIN(qp) |
		       VEPU_REG_H264_CHKPT_DISTANCE(0));

	/* Mode decision. */
	vepu2_h264_set(h264_ctx, VEPU_REG_INTRA_INTER_MODE,
		       VEPU_REG_INTRA16X16_MODE(h264_intra16_favor[qp]) |
		       VEPU_REG_INTER_MODE(h264_inter_favor[qp]));

	vepu2_h264_set(h264_ctx, VEPU_REG_MV_PENALTY,
		       VEPU_REG_1MV_PENALTY(h264_diff_mv_penalty[qp]) |
		       VEPU_REG_QMV_PENALTY(h264_diff_mv_penalty[qp]) |
		       VEPU_REG_4MV_PENALTY(h264_diff_mv_penalty4p[qp]) |
		       VEPU_REG_SPLIT_MV_MODE_EN);

	vepu2_h264_set(h264_ctx, VEPU_REG_MVC_RELATE,
		       VEPU_REG_ZERO_MV_FAVOR_D2(10));

	/* Motion vector difference penalties. */
	for (i = 0; i < 128; i += 4) {
		u32 dmv = 0, dmv_qpel = 0;
		int j;

		for (j = 0; j < 4; j++) {
			unsigned int len = min(255U, vepu2_h264_se_len(i + j));

			dmv |= VEPU_REG_DMV_PENALTY_TABLE_BIT(i + j, 3 - j);
			dmv_qpel |= VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(len,
									   3 - j);
		}

		vepu2_h264_set(h264_ctx, VEPU_REG_DMV_PENALTY_TBL(i / 4), dmv);
		vepu2_h264_set(h264_ctx, VEPU_REG_DMV_Q_PIXEL_PENALTY_TBL(i / 4),
			       dmv_qpel);
	}

	/* Intra areas and regions of interest are placed outside the picture. */
	vepu2_h264_set(h264_ctx, VEPU_REG_INTRA_AREA_CTRL,
		       VEPU_REG_INTRA_AREA_TOP(mb_height) |
		       VEPU_REG_INTRA_AREA_BOTTOM(mb_height) |
		       VEPU_REG_INTRA_AREA_LEFT(mb_width) |
		       VEPU_REG_INTRA_AREA_RIGHT(mb_width));
	vepu2_h264_set(h264_ctx, VEPU_REG_ROI1,
		       VEPU_REG_ROI1_TOP_MB(mb_height) |
		       VEPU_REG_ROI1_BOTTOM_MB(mb_height) |
		       VEPU_REG_ROI1_LEFT_MB(mb_width) |
		       VEPU_REG_ROI1_RIGHT_MB(mb_width));
	vepu2_h264_set(h264_ctx, VEPU_REG_ROI2,
		       VEPU_REG_ROI2_TOP_MB(mb_height) |
		       VEPU_REG_ROI2_BOTTOM_MB(mb_height) |
		       VEPU_REG_ROI2_LEFT_MB(mb_width) |
		       VEPU_REG_ROI2_RIGHT_MB(mb_width));

	vepu2_h264_set(h264_ctx, VEPU_REG_INTERRUPT,
		       VEPU_REG_INTERRUPT_TIMEOUT_EN);
}

int rockchip_vpu2_h264_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->core;
	struct hantro_h264_enc_hw_ctx *h264_ctx = &ctx->h264_enc;
	struct v4l2_h264_enc *enc = &h264_ctx->enc;
	const struct v4l2_ctrl_h264_encode_params *encode;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	unsigned int mb_width = MB_WIDTH(ctx->src_fmt.width);
	unsigned int mb_height = MB_HEIGHT(ctx->src_fmt.height);
	int first_free_bit;
	u32 reg;
	int ret, i;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	hantro_start_prepare_run(ctx);

	if (!vb2_plane_vaddr(&dst_buf->vb2_buf, 0))
		return -ENOMEM;

	/* Prepare the state, the references, the QP and write the headers. */
	ret = v4l2_h264_enc_step(enc, dst_buf);
	if (ret)
		return ret;

	encode = &enc->state_active.encode;

	/*
	 * All the encoder registers are programmed for each run, the ones that
	 * aren't used are cleared.
	 */
	memset(h264_ctx->regs, 0, sizeof(h264_ctx->regs));

	first_free_bit = vepu2_h264_enc_set_buffers(ctx, src_buf, dst_buf);
	if (first_free_bit < 0)
		return first_free_bit;

	vepu2_h264_enc_set_params(ctx, first_free_bit);

	/* Select the H.264 encoder before writing the other registers. */
	vepu_write_relaxed(vpu, VEPU_REG_ENCODE_FORMAT_H264,
			   VEPU_REG_ENCODE_START);

	for (i = 0; i < ROCKCHIP_VPU2_H264_ENC_NUM_REGS; i++) {
		if (i == VEPU_REG_ENCODE_START / 4)
			continue;

		vepu_write_relaxed(vpu, h264_ctx->regs[i], i * 4);
	}

	reg = VEPU_REG_MB_HEIGHT(mb_height) |
	      VEPU_REG_MB_WIDTH(mb_width) |
	      VEPU_REG_ENCODE_FORMAT_H264 |
	      VEPU_REG_ENCODE_ENABLE;
	if (encode->slice_type == V4L2_H264_SLICE_TYPE_I)
		reg |= VEPU_REG_FRAME_TYPE_INTRA;
	else
		reg |= VEPU_REG_FRAME_TYPE_INTER;

	/* Kick the watchdog and start encoding. */
	hantro_end_prepare_run(ctx);
	vepu_write(vpu, reg, VEPU_REG_ENCODE_START);

	return 0;
}

void rockchip_vpu2_h264_enc_done(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->core;
	struct hantro_h264_enc_hw_ctx *h264_ctx = &ctx->h264_enc;
	struct v4l2_h264_enc *enc = &h264_ctx->enc;
	const struct v4l2_ctrl_h264_encode_params *encode =
		&enc->state_active.encode;
	struct vb2_v4l2_buffer *dst_buf = hantro_get_dst_buf(ctx);
	unsigned int hdr_size = v4l2_h264_enc_rbsp_bytes_count(&enc->rbsp);
	unsigned int mbs = MB_WIDTH(ctx->src_fmt.width) *
			   MB_HEIGHT(ctx->src_fmt.height);
	u32 bits, qp_sum;
	size_t bytesused;
	u8 *nal;

	/* The stream size is in bits, from the stream base address. */
	bits = vepu_read(vpu, VEPU_REG_STR_BUF_LIMIT);
	bytesused = h264_ctx->strm_offset + bits / 8;

	qp_sum = FIELD_GET(VEPU2_REG_QP_SUM_DIV2_MASK,
			   vepu_read(vpu, VEPU_REG_QP_SUM_DIV2)) * 2;
	vpu_debug(4, "H.264 frame %u: %zu bytes, average QP %u (target %u)\n",
		  encode->frame_num, bytesused, qp_sum / mbs, enc->rc.qp);

	if (bytesused <= hdr_size + 5 ||
	    bytesused > vb2_plane_size(&dst_buf->vb2_buf, 0)) {
		vpu_err("invalid H.264 stream size %zu\n", bytesused);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		return;
	}

	/*
	 * The hardware writes the nal_ref_idc of the slice NAL unit, use the
	 * one requested instead. It is never zero, so that this doesn't change
	 * the syntax of the slice header.
	 */
	nal = (u8 *)vb2_plane_vaddr(&dst_buf->vb2_buf, 0) + hdr_size;
	if (nal[0] == 0 && nal[1] == 0 && nal[2] == 0 && nal[3] == 1)
		nal[4] = (nal[4] & ~0x60) | ((encode->nal_ref_idc & 3) << 5);

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0, bytesused);

	v4l2_h264_enc_complete(enc, dst_buf);
}
