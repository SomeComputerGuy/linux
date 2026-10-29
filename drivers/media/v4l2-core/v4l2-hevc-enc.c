// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 HEVC Encode Core
 *
 * Kernel side of the stateless HEVC encoding uAPI: turns the HEVC encode
 * controls into the parameter sets and slice segment headers of each
 * picture, keeps the reconstructed reference pictures, resolves the
 * reference picture set given by userspace and selects the QP.
 *
 * Section and table numbers refer to ITU-T Rec. H.265 (V10, 07/2024).
 */

#include <linux/bitops.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/sort.h>
#include <linux/v4l2-controls.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-hevc.h>
#include <media/v4l2-hevc-enc.h>
#include <media/v4l2-hevc-enc-rbsp.h>
#include <media/videobuf2-v4l2.h>

#define RBSP_UPDATE_AUD			BIT(0)
#define RBSP_UPDATE_PARAMETER_SETS	BIT(1)

/* Table A.8: MaxLumaPs and MaxLumaSr (main tier is enough to pick a level) */
static const struct {
	u8 level_idc;
	u32 max_luma_ps;
	u64 max_luma_sr;
} hevc_levels[] = {
	{  30,    36864,     552960 },
	{  60,   122880,    3686400 },
	{  63,   245760,    7372800 },
	{  90,   552960,   16588800 },
	{  93,   983040,   33177600 },
	{ 120,  2228224,   66846720 },
	{ 123,  2228224,  133693440 },
	{ 150,  8912896,  267386880 },
	{ 153,  8912896,  534773760 },
	{ 156,  8912896, 1069547520 },
	{ 180, 35651584, 1069547520 },
	{ 183, 35651584, 2139095040 },
	{ 186, 35651584, 4278190080ULL },
};

/* V4L2_MPEG_VIDEO_HEVC_LEVEL_* to general_level_idc */
static const u8 hevc_level_idc[] = {
	30, 60, 63, 90, 93, 120, 123, 150, 153, 156, 180, 183, 186,
};

static struct v4l2_ctrl *enc_ctrl(struct v4l2_hevc_enc *enc, u32 id)
{
	return v4l2_ctrl_find(enc->ctrl_handler, id);
}

static int enc_ctrl_val(struct v4l2_hevc_enc *enc, u32 id, int def)
{
	struct v4l2_ctrl *ctrl = enc_ctrl(enc, id);

	return ctrl ? ctrl->cur.val : def;
}

static int rec_buffer_alloc(struct v4l2_hevc_enc *enc,
			    struct v4l2_hevc_enc_rec_buffer *buffer)
{
	int ret;

	if (!enc->ops || !enc->ops->rec_buffer_alloc)
		return -EOPNOTSUPP;

	ret = enc->ops->rec_buffer_alloc(enc, buffer);
	if (ret)
		return ret;

	buffer->allocated = true;

	return 0;
}

static void rec_buffer_free(struct v4l2_hevc_enc *enc,
			    struct v4l2_hevc_enc_rec_buffer *buffer)
{
	if (!buffer->allocated)
		return;

	if (enc->ops && enc->ops->rec_buffer_free)
		enc->ops->rec_buffer_free(enc, buffer);

	buffer->allocated = false;
}

static int ref_slots_grow(struct v4l2_hevc_enc *enc, unsigned int count)
{
	struct v4l2_hevc_enc_ref *ref = &enc->ref;
	int ret;

	count = min_t(unsigned int, count, ARRAY_SIZE(ref->dpb));

	/*
	 * Slots are only ever added, which avoids reallocating when the DPB
	 * size changes back and forth, at the expense of memory.
	 */
	while (ref->slots_count < count) {
		ret = rec_buffer_alloc(enc, &ref->dpb[ref->slots_count].buffer);
		if (ret)
			return ret;

		ref->slots_count++;
	}

	return 0;
}

static void rec_buffers_free(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_ref *ref = &enc->ref;
	unsigned int i;

	rec_buffer_free(enc, &ref->buffer_current);

	for (i = 0; i < ref->slots_count; i++)
		rec_buffer_free(enc, &ref->dpb[i].buffer);

	ref->slots_count = 0;
}

/**
 * v4l2_hevc_enc_init() - initialise the HEVC encode core
 * @enc: the core context, with the driver fields set
 *
 * Call it when streaming starts. It allocates the reconstruction buffer of
 * the current picture and the initial reference slots.
 *
 * Return: 0 or a negative error code.
 */
int v4l2_hevc_enc_init(struct v4l2_hevc_enc *enc)
{
	unsigned int slots = 0;
	int ret;

	if ((!enc->format && !enc->format_mplane) || !enc->timeperframe ||
	    !enc->ctrl_handler)
		return -EINVAL;

	memset(&enc->state_active, 0, sizeof(enc->state_active));
	memset(&enc->state_next, 0, sizeof(enc->state_next));
	memset(&enc->ref, 0, sizeof(enc->ref));
	enc->state_serial = 0;

	if (!enc->max_l0_refs)
		enc->max_l0_refs = 1;
	if (!enc->max_l1_refs)
		enc->max_l1_refs = 1;

	ret = rec_buffer_alloc(enc, &enc->ref.buffer_current);
	if (ret)
		return ret;

	if (enc->flags & (V4L2_HEVC_ENC_FLAG_INTER_PRED |
			  V4L2_HEVC_ENC_FLAG_INTER_BIPRED))
		slots = enc->ref_slots_count_init ?: 2;

	ret = ref_slots_grow(enc, slots);
	if (ret)
		rec_buffers_free(enc);

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_init);

/**
 * v4l2_hevc_enc_exit() - release the HEVC encode core
 * @enc: the core context
 */
void v4l2_hevc_enc_exit(struct v4l2_hevc_enc *enc)
{
	rec_buffers_free(enc);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_exit);

static void enc_format(struct v4l2_hevc_enc *enc, unsigned int *width,
		       unsigned int *height, unsigned int *colorspace,
		       unsigned int *quantization, unsigned int *xfer_func,
		       unsigned int *ycbcr_enc)
{
	if (enc->format_mplane) {
		*width = enc->format_mplane->width;
		*height = enc->format_mplane->height;
		*colorspace = enc->format_mplane->colorspace;
		*quantization = enc->format_mplane->quantization;
		*xfer_func = enc->format_mplane->xfer_func;
		*ycbcr_enc = enc->format_mplane->ycbcr_enc;
	} else {
		*width = enc->format->width;
		*height = enc->format->height;
		*colorspace = enc->format->colorspace;
		*quantization = enc->format->quantization;
		*xfer_func = enc->format->xfer_func;
		*ycbcr_enc = enc->format->ycbcr_enc;
	}
}

static void prepare_colour(struct v4l2_hevc_sps_video *video,
			   unsigned int colorspace, unsigned int quantization,
			   unsigned int xfer_func, unsigned int ycbcr_enc)
{
	if (colorspace == V4L2_COLORSPACE_DEFAULT &&
	    quantization == V4L2_QUANTIZATION_DEFAULT &&
	    xfer_func == V4L2_XFER_FUNC_DEFAULT &&
	    ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT)
		return;

	video->flags |= V4L2_HEVC_SPS_VIDEO_FLAG_VUI_PARAMETERS_PRESENT |
			V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_SIGNAL_TYPE_PRESENT |
			V4L2_HEVC_SPS_VIDEO_FLAG_VUI_COLOUR_DESCRIPTION_PRESENT;
	video->video_format = V4L2_HEVC_VUI_VIDEO_FORMAT_UNSPECIFIED;

	if (quantization == V4L2_QUANTIZATION_FULL_RANGE)
		video->flags |= V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_FULL_RANGE;

	switch (colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_SMPTE170M;
		break;
	case V4L2_COLORSPACE_SMPTE240M:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_SMPTE240M;
		break;
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_SRGB:
	case V4L2_COLORSPACE_JPEG:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_BT709;
		break;
	case V4L2_COLORSPACE_470_SYSTEM_M:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_BT470_SYSTEM_M;
		break;
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_BT470_SYSTEM_BG;
		break;
	case V4L2_COLORSPACE_BT2020:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_BT2020;
		break;
	case V4L2_COLORSPACE_DCI_P3:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_SMPTE431;
		break;
	default:
		video->colour_primaries = V4L2_HEVC_VUI_COLOUR_UNSPECIFIED;
		break;
	}

	switch (xfer_func) {
	case V4L2_XFER_FUNC_709:
		if (colorspace == V4L2_COLORSPACE_SMPTE170M)
			video->transfer_characteristics =
				V4L2_HEVC_VUI_TRANSFER_SMPTE170M;
		else
			video->transfer_characteristics =
				V4L2_HEVC_VUI_TRANSFER_BT709;
		break;
	case V4L2_XFER_FUNC_SRGB:
		video->transfer_characteristics = V4L2_HEVC_VUI_TRANSFER_SRGB;
		break;
	case V4L2_XFER_FUNC_SMPTE240M:
		video->transfer_characteristics =
			V4L2_HEVC_VUI_TRANSFER_SMPTE240M;
		break;
	case V4L2_XFER_FUNC_NONE:
		video->transfer_characteristics = V4L2_HEVC_VUI_TRANSFER_LINEAR;
		break;
	case V4L2_XFER_FUNC_SMPTE2084:
		video->transfer_characteristics =
			V4L2_HEVC_VUI_TRANSFER_SMPTE2084;
		break;
	default:
		video->transfer_characteristics =
			V4L2_HEVC_VUI_TRANSFER_UNSPECIFIED;
		break;
	}

	switch (ycbcr_enc) {
	case V4L2_YCBCR_ENC_601:
		if (colorspace == V4L2_COLORSPACE_SMPTE170M)
			video->matrix_coeffs = V4L2_HEVC_VUI_MATRIX_SMPTE170M;
		else
			video->matrix_coeffs =
				V4L2_HEVC_VUI_MATRIX_BT470_SYSTEM_BG;
		break;
	case V4L2_YCBCR_ENC_709:
		video->matrix_coeffs = V4L2_HEVC_VUI_MATRIX_BT709;
		break;
	case V4L2_YCBCR_ENC_SMPTE240M:
		video->matrix_coeffs = V4L2_HEVC_VUI_MATRIX_SMPTE240M;
		break;
	case V4L2_YCBCR_ENC_BT2020:
		video->matrix_coeffs = V4L2_HEVC_VUI_MATRIX_BT2020_NCL;
		break;
	case V4L2_YCBCR_ENC_BT2020_CONST_LUM:
		video->matrix_coeffs = V4L2_HEVC_VUI_MATRIX_BT2020_CL;
		break;
	default:
		video->matrix_coeffs = V4L2_HEVC_VUI_MATRIX_UNSPECIFIED;
		break;
	}
}

static void prepare_ptl(struct v4l2_hevc_enc *enc,
			struct v4l2_hevc_enc_state *state)
{
	struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	struct v4l2_hevc_ptl *ptl = &state->sps_video.ptl;
	struct v4l2_fract *tpf = &state->timeperframe;
	u64 luma_ps = (u64)sps->pic_width_in_luma_samples *
		     sps->pic_height_in_luma_samples;
	u64 luma_sr = 0;
	unsigned int i;
	int profile, level;

	profile = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       sps->bit_depth_luma_minus8 ?
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10 :
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);

	/*
	 * A Main or Main Still Picture stream also conforms to the profiles
	 * that contain it (A.3.2, A.3.4), as common encoders signal.
	 */
	switch (profile) {
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
		ptl->general_profile_idc = V4L2_HEVC_PROFILE_IDC_MAIN_STILL_PICTURE;
		ptl->general_profile_compatibility_flags =
			BIT(31 - V4L2_HEVC_PROFILE_IDC_MAIN) |
			BIT(31 - V4L2_HEVC_PROFILE_IDC_MAIN_10) |
			BIT(31 - V4L2_HEVC_PROFILE_IDC_MAIN_STILL_PICTURE);
		break;
	case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
		ptl->general_profile_idc = V4L2_HEVC_PROFILE_IDC_MAIN_10;
		ptl->general_profile_compatibility_flags =
			BIT(31 - V4L2_HEVC_PROFILE_IDC_MAIN_10);
		break;
	default:
		ptl->general_profile_idc = V4L2_HEVC_PROFILE_IDC_MAIN;
		ptl->general_profile_compatibility_flags =
			BIT(31 - V4L2_HEVC_PROFILE_IDC_MAIN) |
			BIT(31 - V4L2_HEVC_PROFILE_IDC_MAIN_10);
		break;
	}

	ptl->general_tier_flag =
		enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_TIER,
			     V4L2_MPEG_VIDEO_HEVC_TIER_MAIN) ==
		V4L2_MPEG_VIDEO_HEVC_TIER_HIGH;
	ptl->flags = V4L2_HEVC_PTL_FLAG_PROGRESSIVE_SOURCE |
		     V4L2_HEVC_PTL_FLAG_FRAME_ONLY_CONSTRAINT;

	level = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_LEVEL, -1);
	if (level >= 0 && level < ARRAY_SIZE(hevc_level_idc)) {
		ptl->general_level_idc = hevc_level_idc[level];
		return;
	}

	/* Without a level control, pick the lowest level that fits. */
	if (tpf->numerator)
		luma_sr = div_u64(luma_ps * tpf->denominator, tpf->numerator);

	for (i = 0; i < ARRAY_SIZE(hevc_levels); i++) {
		u64 max_dim_sq = 8ULL * hevc_levels[i].max_luma_ps;

		if (luma_ps <= hevc_levels[i].max_luma_ps &&
		    luma_sr <= hevc_levels[i].max_luma_sr &&
		    (u64)sps->pic_width_in_luma_samples *
		    sps->pic_width_in_luma_samples <= max_dim_sq &&
		    (u64)sps->pic_height_in_luma_samples *
		    sps->pic_height_in_luma_samples <= max_dim_sq)
			break;
	}

	ptl->general_level_idc =
		hevc_levels[min_t(unsigned int, i,
				  ARRAY_SIZE(hevc_levels) - 1)].level_idc;
}

static int prepare_sps(struct v4l2_hevc_enc *enc,
		       struct v4l2_hevc_enc_state *state)
{
	struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	struct v4l2_hevc_sps_video *video = &state->sps_video;
	unsigned int width, height, colorspace, quantization, xfer, ycbcr;
	unsigned int min_cb_size, coded_width, coded_height;
	struct v4l2_rect visible;
	struct v4l2_ctrl *ctrl;

	ctrl = enc_ctrl(enc, V4L2_CID_STATELESS_HEVC_SPS);
	if (!ctrl)
		return -EINVAL;

	memcpy(sps, ctrl->p_cur.p_hevc_sps, sizeof(*sps));

	enc_format(enc, &width, &height, &colorspace, &quantization, &xfer,
		   &ycbcr);

	/* Only 4:2:0 and a single set of each parameter set are supported. */
	sps->chroma_format_idc = 1;
	sps->video_parameter_set_id = min_t(u8, sps->video_parameter_set_id, 15);
	sps->seq_parameter_set_id = min_t(u8, sps->seq_parameter_set_id, 15);
	sps->flags &= ~V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE;

	/* Reference picture sets are always coded in the slice header. */
	sps->num_short_term_ref_pic_sets = 0;
	sps->num_long_term_ref_pics_sps = 0;

	if (!(enc->flags & V4L2_HEVC_ENC_FLAG_SCALING_LIST))
		sps->flags &= ~V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED;

	if (!(enc->flags & (V4L2_HEVC_ENC_FLAG_INTER_PRED |
			    V4L2_HEVC_ENC_FLAG_INTER_BIPRED))) {
		sps->sps_max_dec_pic_buffering_minus1 = 0;
		sps->flags &= ~(V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED |
				V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT);
	}

	sps->sps_max_sub_layers_minus1 =
		min_t(u8, sps->sps_max_sub_layers_minus1,
		      V4L2_HEVC_MAX_SUB_LAYERS - 1);
	sps->sps_max_dec_pic_buffering_minus1 =
		min_t(u8, sps->sps_max_dec_pic_buffering_minus1,
		      V4L2_HEVC_DPB_ENTRIES_NUM_MAX - 1);
	sps->sps_max_num_reorder_pics =
		min(sps->sps_max_num_reorder_pics,
		    sps->sps_max_dec_pic_buffering_minus1);
	sps->log2_max_pic_order_cnt_lsb_minus4 =
		min_t(u8, sps->log2_max_pic_order_cnt_lsb_minus4, 12);

	/* 16x16 to 64x64 coding tree blocks (A.4.1), 8x8 minimum CBs. */
	sps->log2_min_luma_coding_block_size_minus3 =
		min_t(u8, sps->log2_min_luma_coding_block_size_minus3, 3);
	state->ctb_log2_size = sps->log2_min_luma_coding_block_size_minus3 + 3 +
			       sps->log2_diff_max_min_luma_coding_block_size;
	state->ctb_log2_size = clamp(state->ctb_log2_size, 4U, 6U);
	sps->log2_diff_max_min_luma_coding_block_size =
		state->ctb_log2_size - 3 -
		sps->log2_min_luma_coding_block_size_minus3;

	/* The coded size is the CAPTURE size, aligned to the minimum CB. */
	min_cb_size = 8 << sps->log2_min_luma_coding_block_size_minus3;
	coded_width = ALIGN(width, min_cb_size);
	coded_height = ALIGN(height, min_cb_size);
	sps->pic_width_in_luma_samples = coded_width;
	sps->pic_height_in_luma_samples = coded_height;

	state->width_ctbs = DIV_ROUND_UP(coded_width, 1 << state->ctb_log2_size);
	state->height_ctbs = DIV_ROUND_UP(coded_height,
					  1 << state->ctb_log2_size);

	memset(video, 0, sizeof(*video));

	/* The conformance window is in chroma samples for 4:2:0 (7.4.3.2.1). */
	if (enc->crop)
		visible = *enc->crop;
	else
		visible = (struct v4l2_rect){ 0, 0, width, height };

	if (visible.left < 0 || visible.top < 0 || visible.left & 1 ||
	    visible.top & 1 || visible.width & 1 || visible.height & 1 ||
	    visible.left + visible.width > coded_width ||
	    visible.top + visible.height > coded_height)
		return -EINVAL;

	if (visible.left || visible.top || visible.width != coded_width ||
	    visible.height != coded_height) {
		video->flags |= V4L2_HEVC_SPS_VIDEO_FLAG_CONFORMANCE_WINDOW;
		video->conf_win_left_offset = visible.left / 2;
		video->conf_win_top_offset = visible.top / 2;
		video->conf_win_right_offset =
			(coded_width - visible.left - visible.width) / 2;
		video->conf_win_bottom_offset =
			(coded_height - visible.top - visible.height) / 2;
	}

	/* Required with a single sub-layer (7.4.3.2.1). */
	if (!sps->sps_max_sub_layers_minus1)
		video->flags |= V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING;

	if (state->timeperframe.numerator && state->timeperframe.denominator) {
		video->flags |= V4L2_HEVC_SPS_VIDEO_FLAG_VPS_TIMING_INFO_PRESENT |
				V4L2_HEVC_SPS_VIDEO_FLAG_VUI_PARAMETERS_PRESENT |
				V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TIMING_INFO_PRESENT;
		video->num_units_in_tick = state->timeperframe.numerator;
		video->time_scale = state->timeperframe.denominator;
	}

	prepare_colour(video, colorspace, quantization, xfer, ycbcr);
	prepare_ptl(enc, state);

	return 0;
}

static int prepare_pps(struct v4l2_hevc_enc *enc,
		       struct v4l2_hevc_enc_state *state)
{
	struct v4l2_ctrl_hevc_pps *pps = &state->pps;
	const struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	int qp_bd_offset = 6 * sps->bit_depth_luma_minus8;
	struct v4l2_ctrl *ctrl;
	unsigned int i, sum;

	ctrl = enc_ctrl(enc, V4L2_CID_STATELESS_HEVC_PPS);
	if (!ctrl)
		return -EINVAL;

	memcpy(pps, ctrl->p_cur.p_hevc_pps, sizeof(*pps));

	pps->pic_parameter_set_id = min_t(u8, pps->pic_parameter_set_id, 63);
	pps->init_qp_minus26 = clamp_t(s8, pps->init_qp_minus26,
				       -(26 + qp_bd_offset), 25);
	pps->num_ref_idx_l0_default_active_minus1 =
		min_t(u8, pps->num_ref_idx_l0_default_active_minus1,
		      enc->max_l0_refs - 1);
	pps->num_ref_idx_l1_default_active_minus1 =
		min_t(u8, pps->num_ref_idx_l1_default_active_minus1,
		      enc->max_l1_refs - 1);
	pps->diff_cu_qp_delta_depth =
		min_t(u8, pps->diff_cu_qp_delta_depth,
		      sps->log2_diff_max_min_luma_coding_block_size);

	/* Weighted prediction tables are not supported. */
	pps->flags &= ~(V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED |
			V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED);

	/*
	 * Entry points are only known after the encode run, so a slice
	 * header written by the kernel cannot have any.
	 */
	if (!(enc->flags & V4L2_HEVC_ENC_FLAG_TILES) ||
	    !(enc->flags & V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER))
		pps->flags &= ~V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
	if (!(enc->flags & V4L2_HEVC_ENC_FLAG_ENTROPY_CODING_SYNC) ||
	    !(enc->flags & V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER))
		pps->flags &= ~V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED;

	if (!(pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED)) {
		pps->num_tile_columns_minus1 = 0;
		pps->num_tile_rows_minus1 = 0;
		pps->flags &= ~(V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING |
				V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED);
	} else {
		if (pps->num_tile_columns_minus1 >= state->width_ctbs ||
		    pps->num_tile_rows_minus1 >= state->height_ctbs ||
		    (!pps->num_tile_columns_minus1 &&
		     !pps->num_tile_rows_minus1))
			return -EINVAL;

		if (!(pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)) {
			for (i = 0, sum = 0; i < pps->num_tile_columns_minus1; i++)
				sum += pps->column_width_minus1[i] + 1;
			if (sum >= state->width_ctbs)
				return -EINVAL;

			for (i = 0, sum = 0; i < pps->num_tile_rows_minus1; i++)
				sum += pps->row_height_minus1[i] + 1;
			if (sum >= state->height_ctbs)
				return -EINVAL;
		}
	}

	return 0;
}

static int prepare_encode(struct v4l2_hevc_enc *enc,
			  struct v4l2_hevc_enc_state *state)
{
	struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	const struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	struct v4l2_ctrl *ctrl;

	ctrl = enc_ctrl(enc, V4L2_CID_STATELESS_HEVC_ENCODE_PARAMS);
	if (!ctrl)
		return -EINVAL;

	memcpy(encode, ctrl->p_cur.p_hevc_encode_params, sizeof(*encode));

	/* The bitstream starts with an IRAP picture. */
	if (!enc->state_serial && !v4l2_hevc_nal_is_irap(encode->nal_unit_type))
		return -EINVAL;

	if (encode->nuh_temporal_id_plus1 - 1 > sps->sps_max_sub_layers_minus1)
		return -EINVAL;

	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_B &&
	    !(enc->flags & V4L2_HEVC_ENC_FLAG_INTER_BIPRED))
		return -EINVAL;

	/* Parameters for a P picture are also valid for an I picture. */
	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_P &&
	    !(enc->flags & V4L2_HEVC_ENC_FLAG_INTER_PRED))
		encode->slice_type = V4L2_HEVC_SLICE_TYPE_I;

	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_I)
		encode->flags &=
			~(V4L2_HEVC_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE |
			  V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED);

	encode->pic_parameter_set_id = state->pps.pic_parameter_set_id;

	if (!(sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED))
		encode->flags &= ~V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED;

	if (!(sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET))
		encode->flags &= ~(V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_LUMA |
				   V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_CHROMA);

	if (!(state->pps.flags &
	      V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED))
		encode->flags &=
			~(V4L2_HEVC_ENCODE_FLAG_DEBLOCKING_FILTER_OVERRIDE |
			  V4L2_HEVC_ENCODE_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED);

	if (!(state->pps.flags &
	      V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT)) {
		encode->slice_cb_qp_offset = 0;
		encode->slice_cr_qp_offset = 0;
	}

	return 0;
}

static void prepare_rc(struct v4l2_hevc_enc *enc,
		       struct v4l2_hevc_enc_state *state)
{
	int qp_bd_offset = 6 * state->sps.bit_depth_luma_minus8;
	struct v4l2_ctrl *ctrl;

	state->frame_rc_enable =
		enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 0);
	state->qp_min = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
				     -qp_bd_offset);
	state->qp_max = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP, 51);
	state->qp_i = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, 30);
	state->qp_p = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
				   state->qp_i);
	state->qp_b = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP,
				   state->qp_p);

	state->bitrate_mode = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
					   V4L2_MPEG_VIDEO_BITRATE_MODE_CQ);

	ctrl = enc_ctrl(enc, V4L2_CID_MPEG_VIDEO_CONSTANT_QUALITY);
	if (ctrl) {
		state->quality = ctrl->cur.val;
		state->quality_min = ctrl->minimum;
		state->quality_max = ctrl->maximum;
	}

	state->bitrate = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_BITRATE, 0);
	state->bitrate_peak = enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
					   state->bitrate);
}

static int state_prepare(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state_next;
	struct v4l2_hevc_enc_state *active = &enc->state_active;
	int ret;

	memset(state, 0, sizeof(*state));
	state->timeperframe = *enc->timeperframe;

	ret = prepare_sps(enc, state);
	if (ret)
		return ret;

	ret = prepare_pps(enc, state);
	if (ret)
		return ret;

	ret = prepare_encode(enc, state);
	if (ret)
		return ret;

	prepare_rc(enc, state);

	if (enc->ops && enc->ops->state_constrain) {
		ret = enc->ops->state_constrain(enc, state);
		if (ret)
			return ret;
	}

	/*
	 * A new SPS can only be activated by an IRAP picture (7.4.2.4.2):
	 * keep the active one until then.
	 */
	if (enc->state_serial &&
	    !v4l2_hevc_nal_is_irap(state->encode.nal_unit_type) &&
	    (memcmp(&state->sps, &active->sps, sizeof(state->sps)) ||
	     memcmp(&state->sps_video, &active->sps_video,
		    sizeof(state->sps_video)))) {
		state->sps = active->sps;
		state->sps_video = active->sps_video;
		state->ctb_log2_size = active->ctb_log2_size;
		state->width_ctbs = active->width_ctbs;
		state->height_ctbs = active->height_ctbs;
	}

	return 0;
}

static int state_commit(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state_next;
	struct v4l2_ctrl *ctrl;
	int ret;

	/* Report the parameter sets that are actually used. */
	ctrl = enc_ctrl(enc, V4L2_CID_STATELESS_HEVC_SPS);
	ret = v4l2_ctrl_s_ctrl_compound(ctrl, V4L2_CTRL_TYPE_HEVC_SPS,
					&state->sps);
	if (ret)
		return ret;

	ctrl = enc_ctrl(enc, V4L2_CID_STATELESS_HEVC_PPS);
	ret = v4l2_ctrl_s_ctrl_compound(ctrl, V4L2_CTRL_TYPE_HEVC_PPS,
					&state->pps);
	if (ret)
		return ret;

	return 0;
}

static unsigned int rbsp_update(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state_next;
	struct v4l2_hevc_enc_state *active = &enc->state_active;
	unsigned int update = 0;

	if (enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_AU_DELIMITER, 0))
		update |= RBSP_UPDATE_AUD;

	if (!enc->state_serial ||
	    memcmp(&state->sps, &active->sps, sizeof(state->sps)) ||
	    memcmp(&state->sps_video, &active->sps_video,
		   sizeof(state->sps_video)) ||
	    memcmp(&state->pps, &active->pps, sizeof(state->pps)))
		update |= RBSP_UPDATE_PARAMETER_SETS;

	if (v4l2_hevc_nal_is_irap(state->encode.nal_unit_type) &&
	    enc_ctrl_val(enc, V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 0))
		update |= RBSP_UPDATE_PARAMETER_SETS;

	return update;
}

struct ref_entry {
	unsigned int rps_index;
	unsigned int dpb_index;
	s32 delta;
	bool used;
};

static int ref_entry_cmp(const void *a, const void *b)
{
	const struct ref_entry *ra = a, *rb = b;

	/* Closest pictures first (7.4.8). */
	if (abs(ra->delta) != abs(rb->delta))
		return abs(ra->delta) < abs(rb->delta) ? -1 : 1;

	return 0;
}

static int ref_entry_lt_cmp(const void *a, const void *b)
{
	const struct ref_entry *ra = a, *rb = b;

	/* By increasing DeltaPocMsbCycleLt, kept in delta. */
	if (ra->delta != rb->delta)
		return ra->delta < rb->delta ? -1 : 1;

	return 0;
}

static int ref_find(struct v4l2_hevc_enc *enc, u64 timestamp)
{
	struct v4l2_hevc_enc_ref *ref = &enc->ref;
	unsigned int i;

	for (i = 0; i < ref->slots_count; i++)
		if (ref->dpb[i].active && ref->dpb[i].timestamp == timestamp)
			return i;

	return -ENOENT;
}

/*
 * Build reference picture list 0 or 1 (8.3.4) from the current RPS, without
 * modification: the temporary list repeats StCurrBefore, StCurrAfter and
 * LtCurr (StCurrAfter first for list 1) until it has the active number of
 * entries.
 */
static unsigned int ref_list_build(u8 *list, unsigned int count,
				   const u8 *first, unsigned int first_count,
				   const u8 *second, unsigned int second_count,
				   const u8 *lt, unsigned int lt_count)
{
	unsigned int total = first_count + second_count + lt_count;
	unsigned int i, j;

	if (!total)
		return 0;

	for (i = 0; i < count;) {
		for (j = 0; j < first_count && i < count; j++)
			list[i++] = first[j];
		for (j = 0; j < second_count && i < count; j++)
			list[i++] = second[j];
		for (j = 0; j < lt_count && i < count; j++)
			list[i++] = lt[j];
	}

	return count;
}

static u8 ref_rps_index(const struct v4l2_ctrl_hevc_encode_params *encode,
			const struct v4l2_hevc_enc_ref *ref,
			unsigned int dpb_index)
{
	unsigned int i;

	for (i = 0; i < encode->num_refs; i++)
		if (ref->rps_dpb[i] == dpb_index)
			return i;

	return U8_MAX;
}

static int ref_step(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state_active;
	struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	const struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	const struct v4l2_ctrl_hevc_pps *pps = &state->pps;
	struct v4l2_hevc_enc_ref *ref = &enc->ref;
	struct v4l2_hevc_slice_header *sh = &enc->slice;
	struct v4l2_ctrl_hevc_encode_feedback *fb = &enc->feedback;
	struct ref_entry neg[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	struct ref_entry pos[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	struct ref_entry lt[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u8 curr_before[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u8 curr_after[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u8 curr_lt[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	unsigned int num_neg = 0, num_pos = 0, num_lt = 0;
	unsigned int num_before = 0, num_after = 0, num_curr_lt = 0;
	u32 max_poc_lsb = BIT(sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
	s32 poc = encode->pic_order_cnt_val;
	s32 poc_msb = poc - (s32)(poc & (max_poc_lsb - 1));
	unsigned int l0_count, l1_count, total, i;
	int ret;

	memset(sh, 0, sizeof(*sh));
	ref->l0_count = 0;
	ref->l1_count = 0;
	ref->collocated = -1;

	/* The current picture and the pictures it keeps (A.4.2). */
	if (enc->flags & (V4L2_HEVC_ENC_FLAG_INTER_PRED |
			  V4L2_HEVC_ENC_FLAG_INTER_BIPRED)) {
		ret = ref_slots_grow(enc,
				     sps->sps_max_dec_pic_buffering_minus1 + 1);
		if (ret)
			return ret;
	}

	/* BLA and IDR pictures empty the DPB, so their POC MSB is 0. */
	if (v4l2_hevc_nal_is_bla(encode->nal_unit_type) ||
	    v4l2_hevc_nal_is_idr(encode->nal_unit_type)) {
		if (poc < 0 || poc >= max_poc_lsb)
			return -EINVAL;
		if (v4l2_hevc_nal_is_bla(encode->nal_unit_type))
			encode->num_refs = 0;
	} else if (!enc->state_serial) {
		/* A CRA picture starting the bitstream (8.3.1). */
		if (poc < 0 || poc >= max_poc_lsb)
			return -EINVAL;
	} else if (abs(poc - ref->prev_tid0_poc) >= max_poc_lsb / 2) {
		/* The decoder could not derive the POC MSB (8.3.1). */
		return -EINVAL;
	}

	if (encode->num_refs > sps->sps_max_dec_pic_buffering_minus1)
		return -EINVAL;

	for (i = 0; i < encode->num_refs; i++) {
		const struct v4l2_hevc_encode_ref *r = &encode->refs[i];
		bool used = r->flags & V4L2_HEVC_ENCODE_REF_FLAG_USED_BY_CURR_PIC;
		struct v4l2_hevc_enc_dpb_entry *entry;
		struct ref_entry e;
		int index;

		index = ref_find(enc, r->timestamp);
		if (index < 0)
			return -EINVAL;

		entry = &ref->dpb[index];
		ref->rps_dpb[i] = index;

		e.rps_index = i;
		e.dpb_index = index;
		e.used = used;
		e.delta = entry->pic_order_cnt_val - poc;

		if (!e.delta)
			return -EINVAL;

		if (r->flags & V4L2_HEVC_ENCODE_REF_FLAG_LONG_TERM) {
			s32 ref_msb = entry->pic_order_cnt_val -
				      (s32)(entry->pic_order_cnt_val &
					    (max_poc_lsb - 1));

			if (!(sps->flags &
			      V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) ||
			    ref_msb > poc_msb)
				return -EINVAL;

			e.delta = (poc_msb - ref_msb) / (s32)max_poc_lsb;
			lt[num_lt++] = e;
			continue;
		}

		/* A long-term picture cannot become short-term again. */
		if (entry->long_term || abs(e.delta) > S16_MAX)
			return -EINVAL;

		if (e.delta < 0)
			neg[num_neg++] = e;
		else
			pos[num_pos++] = e;
	}

	if (enc->rps_limits &&
	    (num_neg > enc->rps_limits->num_negative_pics ||
	     num_pos > enc->rps_limits->num_positive_pics ||
	     num_lt > enc->rps_limits->num_long_term_pics))
		return -EINVAL;

	sort(neg, num_neg, sizeof(*neg), ref_entry_cmp, NULL);
	sort(pos, num_pos, sizeof(*pos), ref_entry_cmp, NULL);
	sort(lt, num_lt, sizeof(*lt), ref_entry_lt_cmp, NULL);

	sh->st_rps.num_negative_pics = num_neg;
	for (i = 0; i < num_neg; i++) {
		s32 prev = i ? neg[i - 1].delta : 0;

		sh->st_rps.delta_poc_s0_minus1[i] = prev - neg[i].delta - 1;
		if (neg[i].used) {
			sh->st_rps.used_by_curr_pic_s0 |= BIT(i);
			curr_before[num_before++] = neg[i].dpb_index;
		}
	}

	sh->st_rps.num_positive_pics = num_pos;
	for (i = 0; i < num_pos; i++) {
		s32 prev = i ? pos[i - 1].delta : 0;

		sh->st_rps.delta_poc_s1_minus1[i] = pos[i].delta - prev - 1;
		if (pos[i].used) {
			sh->st_rps.used_by_curr_pic_s1 |= BIT(i);
			curr_after[num_after++] = pos[i].dpb_index;
		}
	}

	sh->num_long_term_pics = num_lt;
	for (i = 0; i < num_lt; i++) {
		struct v4l2_hevc_lt_pic *lt_pic = &sh->lt[i];
		s32 ref_poc = ref->dpb[lt[i].dpb_index].pic_order_cnt_val;

		lt_pic->poc_lsb_lt = ref_poc & (max_poc_lsb - 1);
		lt_pic->used_by_curr_pic_lt = lt[i].used;
		/* Always signal the MSB, so no lsb can be ambiguous. */
		lt_pic->delta_poc_msb_present = true;
		/* DeltaPocMsbCycleLt accumulates over the entries (7-52). */
		lt_pic->delta_poc_msb_cycle_lt =
			lt[i].delta - (i ? lt[i - 1].delta : 0);
		if (lt[i].used)
			curr_lt[num_curr_lt++] = lt[i].dpb_index;
	}

	total = num_before + num_after + num_curr_lt;

	/* BLA and CRA pictures have NumPicTotalCurr 0, P/B slices do not. */
	if ((v4l2_hevc_nal_is_irap(encode->nal_unit_type) && total) ||
	    (encode->slice_type != V4L2_HEVC_SLICE_TYPE_I && !total))
		return -EINVAL;

	/* Reference picture lists. */

	if (encode->flags & V4L2_HEVC_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE) {
		l0_count = encode->num_ref_idx_l0_active_minus1 + 1;
		l1_count = encode->num_ref_idx_l1_active_minus1 + 1;
	} else {
		l0_count = pps->num_ref_idx_l0_default_active_minus1 + 1;
		l1_count = pps->num_ref_idx_l1_default_active_minus1 + 1;
	}

	l0_count = min(l0_count, enc->max_l0_refs);
	l1_count = min(l1_count, enc->max_l1_refs);

	if (encode->slice_type != V4L2_HEVC_SLICE_TYPE_I)
		ref->l0_count = ref_list_build(ref->l0, l0_count,
					       curr_before, num_before,
					       curr_after, num_after,
					       curr_lt, num_curr_lt);
	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_B)
		ref->l1_count = ref_list_build(ref->l1, l1_count,
					       curr_after, num_after,
					       curr_before, num_before,
					       curr_lt, num_curr_lt);

	if (ref->l0_count &&
	    (ref->l0_count != pps->num_ref_idx_l0_default_active_minus1 + 1 ||
	     (ref->l1_count &&
	      ref->l1_count != pps->num_ref_idx_l1_default_active_minus1 + 1))) {
		encode->flags |= V4L2_HEVC_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE;
		encode->num_ref_idx_l0_active_minus1 = ref->l0_count - 1;
		encode->num_ref_idx_l1_active_minus1 =
			ref->l1_count ? ref->l1_count - 1 : 0;
	} else {
		encode->flags &= ~V4L2_HEVC_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE;
	}

	if (encode->flags & V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED) {
		bool from_l0 = encode->slice_type != V4L2_HEVC_SLICE_TYPE_B ||
			       encode->flags &
			       V4L2_HEVC_ENCODE_FLAG_COLLOCATED_FROM_L0;
		unsigned int count = from_l0 ? ref->l0_count : ref->l1_count;

		if (encode->collocated_ref_idx >= count)
			encode->collocated_ref_idx = 0;

		ref->collocated = from_l0 ? ref->l0[encode->collocated_ref_idx] :
					    ref->l1[encode->collocated_ref_idx];
	}

	/* Feedback: list entries as indices in the encode params refs[]. */
	fb->num_ref_idx_l0_active = ref->l0_count;
	fb->num_ref_idx_l1_active = ref->l1_count;
	for (i = 0; i < ref->l0_count; i++)
		fb->ref_idx_l0[i] = ref_rps_index(encode, ref, ref->l0[i]);
	for (i = 0; i < ref->l1_count; i++)
		fb->ref_idx_l1[i] = ref_rps_index(encode, ref, ref->l1[i]);

	return 0;
}

static int rc_step(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state_active;
	int qp_bd_offset = 6 * state->sps.bit_depth_luma_minus8;
	u8 slice_type = state->encode.slice_type;
	unsigned int range;
	int qp, ret;

	if (!state->frame_rc_enable) {
		if (slice_type == V4L2_HEVC_SLICE_TYPE_I)
			qp = state->qp_i;
		else if (slice_type == V4L2_HEVC_SLICE_TYPE_P)
			qp = state->qp_p;
		else
			qp = state->qp_b;
	} else if (state->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ) {
		/* Map the quality range linearly onto the QP range. */
		range = state->quality_max - state->quality_min;
		qp = state->qp_max;
		if (range)
			qp -= (state->quality - state->quality_min) *
			      (state->qp_max - state->qp_min) / range;
	} else {
		if (!enc->rc_ops || !enc->rc_ops->estimate)
			return -EOPNOTSUPP;

		ret = enc->rc_ops->estimate(enc, state, &qp);
		if (ret)
			return ret;
	}

	qp = clamp(qp, state->qp_min, state->qp_max);
	enc->qp = clamp(qp, -qp_bd_offset, 51);

	return 0;
}

static void slice_step(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state_active;
	const struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	struct v4l2_hevc_slice_header *sh = &enc->slice;
	u32 max_poc_lsb = BIT(state->sps.log2_max_pic_order_cnt_lsb_minus4 + 4);

	/* The reference picture sets were filled by ref_step(). */
	sh->nal_unit_type = encode->nal_unit_type;
	sh->nuh_temporal_id_plus1 = encode->nuh_temporal_id_plus1;
	sh->slice_type = encode->slice_type;
	sh->slice_segment_address = 0;
	sh->slice_pic_order_cnt_lsb = encode->pic_order_cnt_val &
				      (max_poc_lsb - 1);
	sh->num_ref_idx_l0_active_minus1 = encode->num_ref_idx_l0_active_minus1;
	sh->num_ref_idx_l1_active_minus1 = encode->num_ref_idx_l1_active_minus1;
	sh->collocated_ref_idx = encode->collocated_ref_idx;
	sh->five_minus_max_num_merge_cand = encode->five_minus_max_num_merge_cand;
	sh->slice_qp_delta = enc->qp - 26 - state->pps.init_qp_minus26;
	sh->slice_cb_qp_offset = encode->slice_cb_qp_offset;
	sh->slice_cr_qp_offset = encode->slice_cr_qp_offset;
	sh->slice_beta_offset_div2 = encode->slice_beta_offset_div2;
	sh->slice_tc_offset_div2 = encode->slice_tc_offset_div2;
	sh->flags = encode->flags;
}

static int rbsp_step(struct v4l2_hevc_enc *enc, struct vb2_v4l2_buffer *buffer,
		     unsigned int update)
{
	struct v4l2_hevc_enc_state *state = &enc->state_active;
	struct v4l2_hevc_enc_rbsp *rbsp = &enc->rbsp;
	u8 tid1 = state->encode.nuh_temporal_id_plus1;
	unsigned int hw = enc->flags;
	bool start_code;
	u8 pic_type;
	u8 *data;

	enc->header_size = 0;

	if (!(update & RBSP_UPDATE_AUD) || hw & V4L2_HEVC_ENC_FLAG_HW_AUD)
		update &= ~RBSP_UPDATE_AUD;
	if ((hw & V4L2_HEVC_ENC_FLAG_HW_VPS) && (hw & V4L2_HEVC_ENC_FLAG_HW_SPS) &&
	    (hw & V4L2_HEVC_ENC_FLAG_HW_PPS))
		update &= ~RBSP_UPDATE_PARAMETER_SETS;
	if (!update && hw & V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER)
		return 0;

	data = vb2_plane_vaddr(&buffer->vb2_buf, 0);
	if (!data)
		return -EINVAL;

	v4l2_hevc_enc_rbsp_init(rbsp, data,
				vb2_plane_size(&buffer->vb2_buf, 0));

	start_code = enc_ctrl_val(enc, V4L2_CID_STATELESS_HEVC_START_CODE,
				  V4L2_STATELESS_HEVC_START_CODE_ANNEX_B) ==
		     V4L2_STATELESS_HEVC_START_CODE_ANNEX_B;

	if (update & RBSP_UPDATE_AUD) {
		if (state->encode.slice_type == V4L2_HEVC_SLICE_TYPE_I)
			pic_type = V4L2_HEVC_AUD_PIC_TYPE_I;
		else if (state->encode.slice_type == V4L2_HEVC_SLICE_TYPE_P)
			pic_type = V4L2_HEVC_AUD_PIC_TYPE_PI;
		else
			pic_type = V4L2_HEVC_AUD_PIC_TYPE_BPI;

		v4l2_hevc_enc_rbsp_aud(rbsp, start_code, pic_type, tid1);
	}

	if (update & RBSP_UPDATE_PARAMETER_SETS) {
		if (!(hw & V4L2_HEVC_ENC_FLAG_HW_VPS))
			v4l2_hevc_enc_rbsp_vps(rbsp, start_code, &state->sps,
					       &state->sps_video);
		if (!(hw & V4L2_HEVC_ENC_FLAG_HW_SPS))
			v4l2_hevc_enc_rbsp_sps(rbsp, start_code, &state->sps,
					       &state->sps_video);
		if (!(hw & V4L2_HEVC_ENC_FLAG_HW_PPS))
			v4l2_hevc_enc_rbsp_pps(rbsp, start_code, &state->sps,
					       &state->pps);
	}

	if (!(hw & V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER))
		v4l2_hevc_enc_rbsp_slice_header(rbsp, start_code, &state->sps,
						&state->pps, &enc->slice);

	enc->header_size = v4l2_hevc_enc_rbsp_bytes(rbsp);

	return v4l2_hevc_enc_rbsp_error(rbsp);
}

static void state_debug(struct v4l2_hevc_enc *enc)
{
	const struct v4l2_hevc_enc_state *state = &enc->state_active;
	const struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;

	pr_debug("v4l2-hevc-enc: %c nal %u tid %u poc %d qp %d refs %u (l0 %u, l1 %u), %ux%u ctbs\n",
		 v4l2_hevc_slice_type_char(encode->slice_type),
		 encode->nal_unit_type, encode->nuh_temporal_id_plus1 - 1,
		 encode->pic_order_cnt_val, enc->qp, encode->num_refs,
		 enc->ref.l0_count, enc->ref.l1_count, state->width_ctbs,
		 state->height_ctbs);
}

/**
 * v4l2_hevc_enc_step() - prepare the encode run of a picture
 * @enc: the core context
 * @buffer: the CAPTURE buffer, whose timestamp is already the one of the
 *	    OUTPUT buffer being encoded
 *
 * Call it from the m2m device_run callback, after the request controls
 * were applied. It accommodates and reports the parameters, resolves the
 * reference picture set, selects the QP and writes the headers at the
 * start of @buffer. The driver then programs the hardware from
 * &v4l2_hevc_enc.state_active, &v4l2_hevc_enc.ref (current reconstruction
 * buffer, reference picture lists and collocated picture),
 * &v4l2_hevc_enc.slice and &v4l2_hevc_enc.qp, and has it write from
 * &v4l2_hevc_enc.header_size on. Nothing in the reference state changes
 * until v4l2_hevc_enc_complete(), so a failed run can simply be dropped.
 *
 * Context: process context, takes the control handler lock.
 * Return: 0 or a negative error code.
 */
int v4l2_hevc_enc_step(struct v4l2_hevc_enc *enc,
		       struct vb2_v4l2_buffer *buffer)
{
	unsigned int update;
	int ret;

	ret = state_prepare(enc);
	if (ret)
		return ret;

	update = rbsp_update(enc);

	/* Parameter sets of a run that did not complete are sent again. */
	if (enc->parameter_sets_pending)
		update |= RBSP_UPDATE_PARAMETER_SETS;

	ret = state_commit(enc);
	if (ret)
		return ret;

	memcpy(&enc->state_active, &enc->state_next, sizeof(enc->state_active));

	if (update & RBSP_UPDATE_PARAMETER_SETS)
		enc->parameter_sets_pending = true;

	memset(&enc->feedback, 0, sizeof(enc->feedback));

	ret = ref_step(enc);
	if (ret)
		return ret;

	ret = rc_step(enc);
	if (ret)
		return ret;

	slice_step(enc);

	ret = rbsp_step(enc, buffer, update);
	if (ret)
		return ret;

	state_debug(enc);

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_step);

/*
 * Set nuh_temporal_id_plus1 in the NAL unit headers the hardware wrote.
 * The second header byte is never 0, before or after, so this cannot
 * create or remove a start code emulation.
 */
static void patch_temporal_id(u8 *data, unsigned int size, u8 tid1)
{
	unsigned int i;

	for (i = 0; i + 4 < size; i++) {
		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;

		if (((data[i + 3] >> 1) & 0x3f) < V4L2_HEVC_NAL_VPS)
			data[i + 4] = (data[i + 4] & ~0x7) | tid1;

		i += 4;
	}
}

static int ref_complete(struct v4l2_hevc_enc *enc, u64 timestamp)
{
	struct v4l2_hevc_enc_state *state = &enc->state_active;
	const struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	struct v4l2_hevc_enc_ref *ref = &enc->ref;
	struct v4l2_hevc_enc_dpb_entry *entry;
	bool keep[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = { };
	unsigned int i;

	/* Pictures that are not in the RPS are no longer references. */
	for (i = 0; i < encode->num_refs; i++) {
		keep[ref->rps_dpb[i]] = true;
		if (encode->refs[i].flags & V4L2_HEVC_ENCODE_REF_FLAG_LONG_TERM)
			ref->dpb[ref->rps_dpb[i]].long_term = true;
	}

	for (i = 0; i < ref->slots_count; i++)
		if (!keep[i])
			ref->dpb[i].active = false;

	/* Intra-only hardware keeps no reference pictures. */
	if (!(encode->flags & V4L2_HEVC_ENCODE_FLAG_REFERENCE) ||
	    !ref->slots_count)
		return 0;

	for (i = 0; i < ref->slots_count; i++)
		if (!ref->dpb[i].active)
			break;

	if (WARN_ON(i == ref->slots_count))
		return -ENOSPC;

	entry = &ref->dpb[i];
	swap(entry->buffer, ref->buffer_current);
	entry->timestamp = timestamp;
	entry->pic_order_cnt_val = encode->pic_order_cnt_val;
	entry->active = true;
	entry->long_term = false;

	return 0;
}

/**
 * v4l2_hevc_enc_complete() - complete the encode run of a picture
 * @enc: the core context
 * @buffer: the CAPTURE buffer, with its payload set
 * @stats: statistics measured by the hardware, or NULL
 *
 * Call it once the hardware finished encoding the picture successfully,
 * before completing the request. It updates the reference pictures and
 * the rate control, sets the buffer flags and sets the feedback control so
 * that it gets stored in the request.
 *
 * Context: process context, takes the control handler lock.
 * Return: 0 or a negative error code.
 */
int v4l2_hevc_enc_complete(struct v4l2_hevc_enc *enc,
			   struct vb2_v4l2_buffer *buffer,
			   const struct v4l2_stateless_encode_stats *stats)
{
	struct v4l2_hevc_enc_state *state = &enc->state_active;
	const struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	struct v4l2_ctrl_hevc_encode_feedback *fb = &enc->feedback;
	unsigned long bytesused = vb2_get_plane_payload(&buffer->vb2_buf, 0);
	u64 timestamp = buffer->vb2_buf.timestamp;
	struct v4l2_ctrl *ctrl;
	u8 nal_unit_type = encode->nal_unit_type;
	u8 *data;
	int ret;

	if (WARN_ON(bytesused < enc->header_size))
		return -EINVAL;

	if (encode->nuh_temporal_id_plus1 != 1 &&
	    enc->flags & V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER &&
	    !(enc->flags & V4L2_HEVC_ENC_FLAG_HW_TEMPORAL_ID)) {
		data = vb2_plane_vaddr(&buffer->vb2_buf, 0);
		if (!data)
			return -EINVAL;

		patch_temporal_id(data + enc->header_size,
				  bytesused - enc->header_size,
				  encode->nuh_temporal_id_plus1);
	}

	buffer->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			   V4L2_BUF_FLAG_BFRAME);
	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_I)
		buffer->flags |= V4L2_BUF_FLAG_KEYFRAME;
	else if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_P)
		buffer->flags |= V4L2_BUF_FLAG_PFRAME;
	else
		buffer->flags |= V4L2_BUF_FLAG_BFRAME;

	ret = ref_complete(enc, timestamp);
	if (ret)
		return ret;

	/* prevTid0Pic (8.3.1) */
	if (encode->nuh_temporal_id_plus1 == 1 &&
	    !v4l2_hevc_nal_is_leading(nal_unit_type) &&
	    !v4l2_hevc_nal_is_sub_layer_non_ref(nal_unit_type))
		enc->ref.prev_tid0_poc = encode->pic_order_cnt_val;

	if (enc->rc_ops && enc->rc_ops->complete)
		enc->rc_ops->complete(enc, state, bytesused, stats);

	fb->timestamp = timestamp;
	fb->pic_order_cnt_val = encode->pic_order_cnt_val;
	fb->nal_unit_type = nal_unit_type;
	fb->nuh_temporal_id_plus1 = encode->nuh_temporal_id_plus1;
	fb->slice_type = encode->slice_type;
	fb->slice_qp = enc->qp;
	if (stats)
		fb->stats = *stats;
	fb->stats.header_size = enc->header_size;
	fb->flags = 0;
	if (encode->flags & V4L2_HEVC_ENCODE_FLAG_REFERENCE)
		fb->flags |= V4L2_HEVC_ENCODE_FEEDBACK_FLAG_REFERENCE;

	ctrl = enc_ctrl(enc, V4L2_CID_STATELESS_HEVC_ENCODE_FEEDBACK);
	if (ctrl) {
		ret = v4l2_ctrl_s_ctrl_compound(ctrl,
						V4L2_CTRL_TYPE_HEVC_ENCODE_FEEDBACK,
						fb);
		if (ret)
			return ret;
	}

	enc->parameter_sets_pending = false;
	enc->state_serial++;

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_complete);

MODULE_DESCRIPTION("V4L2 HEVC Encode Core");
MODULE_LICENSE("GPL");
