// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 HEVC Encode RBSP
 *
 * Section and table numbers refer to ITU-T Rec. H.265 (V10, 07/2024).
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/limits.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>
#include <media/v4l2-hevc.h>
#include <media/v4l2-hevc-enc-rbsp.h>

#define EMULATION_PREVENTION_THREE_BYTE	0x03

/**
 * v4l2_hevc_enc_rbsp_init() - start writing NAL units to a buffer
 * @rbsp: the writer
 * @data: the destination buffer
 * @size: the size of @data in bytes
 */
void v4l2_hevc_enc_rbsp_init(struct v4l2_hevc_enc_rbsp *rbsp, u8 *data,
			     unsigned int size)
{
	memset(rbsp, 0, sizeof(*rbsp));
	rbsp->data = data;
	rbsp->size = size;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_init);

/**
 * v4l2_hevc_enc_rbsp_bytes() - number of bytes written so far
 * @rbsp: the writer
 *
 * Bits that do not fill a byte yet are not counted. All the NAL unit and
 * slice header writers end byte-aligned.
 *
 * Return: the number of bytes written, emulation prevention bytes included.
 */
unsigned int v4l2_hevc_enc_rbsp_bytes(const struct v4l2_hevc_enc_rbsp *rbsp)
{
	return rbsp->offset;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_bytes);

/**
 * v4l2_hevc_enc_rbsp_error() - first error of the writer
 * @rbsp: the writer
 *
 * Return: 0, -ENOSPC when the buffer is too small, or -EINVAL when a value
 * cannot be represented or the parameters are not supported.
 */
int v4l2_hevc_enc_rbsp_error(const struct v4l2_hevc_enc_rbsp *rbsp)
{
	return rbsp->error;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_error);

static void rbsp_fail(struct v4l2_hevc_enc_rbsp *rbsp, int error)
{
	if (!rbsp->error)
		rbsp->error = error;
}

static void rbsp_emit(struct v4l2_hevc_enc_rbsp *rbsp, u8 byte)
{
	if (rbsp->offset >= rbsp->size) {
		rbsp_fail(rbsp, -ENOSPC);
		return;
	}

	rbsp->data[rbsp->offset++] = byte;
}

/*
 * Write one byte of a NAL unit. Inside the payload, a byte in the range
 * 0x00 to 0x03 that follows two zero bytes gets an
 * emulation_prevention_three_byte in front of it (7.4.2).
 */
static void rbsp_put_byte(struct v4l2_hevc_enc_rbsp *rbsp, u8 byte)
{
	if (rbsp->error)
		return;

	if (rbsp->epb && rbsp->zero_bytes >= 2 &&
	    byte <= EMULATION_PREVENTION_THREE_BYTE) {
		rbsp_emit(rbsp, EMULATION_PREVENTION_THREE_BYTE);
		rbsp->zero_bytes = 0;
	}

	rbsp_emit(rbsp, byte);

	if (byte)
		rbsp->zero_bytes = 0;
	else
		rbsp->zero_bytes++;
}

/**
 * v4l2_hevc_enc_rbsp_bits() - write a fixed-length value, u(n)
 * @rbsp: the writer
 * @value: the value, only the @count low bits are written
 * @count: the number of bits, up to 32
 */
void v4l2_hevc_enc_rbsp_bits(struct v4l2_hevc_enc_rbsp *rbsp, u32 value,
			     unsigned int count)
{
	if (rbsp->error || !count)
		return;

	if (WARN_ON(count > 32)) {
		rbsp_fail(rbsp, -EINVAL);
		return;
	}

	rbsp->cache = (rbsp->cache << count) | (value & GENMASK(count - 1, 0));
	rbsp->cache_bits += count;

	while (rbsp->cache_bits >= 8) {
		rbsp->cache_bits -= 8;
		rbsp_put_byte(rbsp, rbsp->cache >> rbsp->cache_bits);
	}

	rbsp->cache &= GENMASK_ULL(7, 0) >> (8 - rbsp->cache_bits);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_bits);

static void rbsp_flag(struct v4l2_hevc_enc_rbsp *rbsp, bool value)
{
	v4l2_hevc_enc_rbsp_bits(rbsp, value, 1);
}

static void rbsp_flags(struct v4l2_hevc_enc_rbsp *rbsp, u64 flags, u64 flag)
{
	rbsp_flag(rbsp, !!(flags & flag));
}

/**
 * v4l2_hevc_enc_rbsp_ue() - write an unsigned Exp-Golomb value, ue(v) (9.2)
 * @rbsp: the writer
 * @value: the value, up to 2^32 - 2
 */
void v4l2_hevc_enc_rbsp_ue(struct v4l2_hevc_enc_rbsp *rbsp, u32 value)
{
	u64 code = (u64)value + 1;
	unsigned int len = fls64(code);

	if (value == U32_MAX) {
		rbsp_fail(rbsp, -EINVAL);
		return;
	}

	/* len - 1 leading zero bits, then code on len bits. */
	v4l2_hevc_enc_rbsp_bits(rbsp, 0, len - 1);
	v4l2_hevc_enc_rbsp_bits(rbsp, code, len);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_ue);

/**
 * v4l2_hevc_enc_rbsp_se() - write a signed Exp-Golomb value, se(v) (9.2.2)
 * @rbsp: the writer
 * @value: the value, from -(2^31 - 1) to 2^31 - 1
 */
void v4l2_hevc_enc_rbsp_se(struct v4l2_hevc_enc_rbsp *rbsp, s32 value)
{
	if (value == S32_MIN) {
		rbsp_fail(rbsp, -EINVAL);
		return;
	}

	if (value > 0)
		v4l2_hevc_enc_rbsp_ue(rbsp, 2 * (u32)value - 1);
	else
		v4l2_hevc_enc_rbsp_ue(rbsp, 2 * (u32)-value);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_se);

/**
 * v4l2_hevc_enc_rbsp_trailing_bits() - write rbsp_trailing_bits() (7.3.2.11)
 * @rbsp: the writer
 *
 * A one bit followed by zero bits up to the next byte boundary. This is
 * also the bit pattern of byte_alignment() (7.3.2.12), which ends slice
 * segment headers.
 */
void v4l2_hevc_enc_rbsp_trailing_bits(struct v4l2_hevc_enc_rbsp *rbsp)
{
	rbsp_flag(rbsp, 1);

	if (rbsp->cache_bits)
		v4l2_hevc_enc_rbsp_bits(rbsp, 0, 8 - rbsp->cache_bits);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_trailing_bits);

/**
 * v4l2_hevc_enc_rbsp_nal_begin() - start a NAL unit
 * @rbsp: the writer
 * @start_code: whether to write an Annex B start code (with zero_byte)
 * @nal_unit_type: nal_unit_type
 * @nuh_temporal_id_plus1: nuh_temporal_id_plus1
 *
 * Writes the optional start code and the NAL unit header (7.3.1.2), with
 * nuh_layer_id equal to 0, then enables emulation prevention for the
 * payload.
 */
void v4l2_hevc_enc_rbsp_nal_begin(struct v4l2_hevc_enc_rbsp *rbsp,
				  bool start_code, u8 nal_unit_type,
				  u8 nuh_temporal_id_plus1)
{
	if (WARN_ON(rbsp->cache_bits) || nal_unit_type > 63 ||
	    !nuh_temporal_id_plus1 || nuh_temporal_id_plus1 > 7) {
		rbsp_fail(rbsp, -EINVAL);
		return;
	}

	rbsp->epb = false;

	if (start_code)
		v4l2_hevc_enc_rbsp_bits(rbsp, 0x00000001, 32);

	/* forbidden_zero_bit, nal_unit_type, nuh_layer_id */
	v4l2_hevc_enc_rbsp_bits(rbsp, nal_unit_type << 1, 8);
	v4l2_hevc_enc_rbsp_bits(rbsp, nuh_temporal_id_plus1, 8);

	rbsp->epb = true;
	rbsp->zero_bytes = 0;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_nal_begin);

/**
 * v4l2_hevc_enc_rbsp_nal_end() - end a NAL unit with rbsp_trailing_bits()
 * @rbsp: the writer
 */
void v4l2_hevc_enc_rbsp_nal_end(struct v4l2_hevc_enc_rbsp *rbsp)
{
	v4l2_hevc_enc_rbsp_trailing_bits(rbsp);
	rbsp->epb = false;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_nal_end);

/**
 * v4l2_hevc_enc_rbsp_aud() - write an access unit delimiter (7.3.2.5)
 * @rbsp: the writer
 * @start_code: whether to write an Annex B start code
 * @pic_type: pic_type, see V4L2_HEVC_AUD_PIC_TYPE_{}
 * @nuh_temporal_id_plus1: TemporalId of the access unit, plus 1
 *
 * Return: 0 or the first error of the writer.
 */
int v4l2_hevc_enc_rbsp_aud(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   u8 pic_type, u8 nuh_temporal_id_plus1)
{
	if (pic_type > V4L2_HEVC_AUD_PIC_TYPE_BPI)
		rbsp_fail(rbsp, -EINVAL);

	v4l2_hevc_enc_rbsp_nal_begin(rbsp, start_code, V4L2_HEVC_NAL_AUD,
				     nuh_temporal_id_plus1);
	v4l2_hevc_enc_rbsp_bits(rbsp, pic_type, 3);
	v4l2_hevc_enc_rbsp_nal_end(rbsp);

	return rbsp->error;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_aud);

/* profile_tier_level(1, max_sub_layers_minus1) (7.3.3) */
static void rbsp_ptl(struct v4l2_hevc_enc_rbsp *rbsp,
		     const struct v4l2_hevc_ptl *ptl,
		     unsigned int max_sub_layers_minus1)
{
	unsigned int i;

	v4l2_hevc_enc_rbsp_bits(rbsp, ptl->general_profile_space, 2);
	rbsp_flag(rbsp, ptl->general_tier_flag);
	v4l2_hevc_enc_rbsp_bits(rbsp, ptl->general_profile_idc, 5);
	v4l2_hevc_enc_rbsp_bits(rbsp, ptl->general_profile_compatibility_flags,
				32);
	rbsp_flags(rbsp, ptl->flags, V4L2_HEVC_PTL_FLAG_PROGRESSIVE_SOURCE);
	rbsp_flags(rbsp, ptl->flags, V4L2_HEVC_PTL_FLAG_INTERLACED_SOURCE);
	rbsp_flags(rbsp, ptl->flags, V4L2_HEVC_PTL_FLAG_NON_PACKED_CONSTRAINT);
	rbsp_flags(rbsp, ptl->flags, V4L2_HEVC_PTL_FLAG_FRAME_ONLY_CONSTRAINT);

	/*
	 * The 43 bits of general_reserved_zero_43bits, or of the format range
	 * extensions constraint flags which are all 0 for the profiles that
	 * can be signalled, then general_inbld_flag or
	 * general_reserved_zero_bit.
	 */
	v4l2_hevc_enc_rbsp_bits(rbsp, 0, 32);
	v4l2_hevc_enc_rbsp_bits(rbsp, 0, 11);
	v4l2_hevc_enc_rbsp_bits(rbsp, 0, 1);

	v4l2_hevc_enc_rbsp_bits(rbsp, ptl->general_level_idc, 8);

	/* sub_layer_profile_present_flag and sub_layer_level_present_flag */
	for (i = 0; i < max_sub_layers_minus1; i++)
		v4l2_hevc_enc_rbsp_bits(rbsp, 0, 2);

	/* reserved_zero_2bits */
	if (max_sub_layers_minus1)
		for (i = max_sub_layers_minus1; i < 8; i++)
			v4l2_hevc_enc_rbsp_bits(rbsp, 0, 2);
}

/*
 * The sub-layer ordering info is written once, for the highest sub-layer,
 * when there are several sub-layers: the lower ones infer the same values.
 * With a single sub-layer, sub_layer_ordering_info_present_flag is set like
 * common encoders do; the bits are the same apart from the flag.
 */
static void rbsp_sub_layer_ordering(struct v4l2_hevc_enc_rbsp *rbsp,
				    const struct v4l2_ctrl_hevc_sps *sps)
{
	rbsp_flag(rbsp, !sps->sps_max_sub_layers_minus1);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->sps_max_dec_pic_buffering_minus1);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->sps_max_num_reorder_pics);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->sps_max_latency_increase_plus1);
}

static void rbsp_timing_info(struct v4l2_hevc_enc_rbsp *rbsp,
			     const struct v4l2_hevc_sps_video *video)
{
	bool poc_proportional = video->flags &
		V4L2_HEVC_SPS_VIDEO_FLAG_VUI_POC_PROPORTIONAL_TO_TIMING;

	v4l2_hevc_enc_rbsp_bits(rbsp, video->num_units_in_tick, 32);
	v4l2_hevc_enc_rbsp_bits(rbsp, video->time_scale, 32);
	rbsp_flag(rbsp, poc_proportional);
	if (poc_proportional)
		v4l2_hevc_enc_rbsp_ue(rbsp,
				      video->num_ticks_poc_diff_one_minus1);
}

/**
 * v4l2_hevc_enc_rbsp_vps() - write a video parameter set (7.3.2.1)
 * @rbsp: the writer
 * @start_code: whether to write an Annex B start code
 * @sps: the SPS the VPS describes
 * @sps_video: the VPS/SPS fields not in @sps
 *
 * The VPS describes a single layer with the sub-layers of @sps and no HRD
 * parameters.
 *
 * Return: 0 or the first error of the writer.
 */
int v4l2_hevc_enc_rbsp_vps(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_sps_video *sps_video)
{
	u32 flags = sps_video->flags;
	bool timing = flags & V4L2_HEVC_SPS_VIDEO_FLAG_VPS_TIMING_INFO_PRESENT;

	if (sps->video_parameter_set_id > 15 ||
	    sps->sps_max_sub_layers_minus1 >= V4L2_HEVC_MAX_SUB_LAYERS)
		rbsp_fail(rbsp, -EINVAL);

	v4l2_hevc_enc_rbsp_nal_begin(rbsp, start_code, V4L2_HEVC_NAL_VPS, 1);

	v4l2_hevc_enc_rbsp_bits(rbsp, sps->video_parameter_set_id, 4);
	rbsp_flag(rbsp, 1);			/* vps_base_layer_internal_flag */
	rbsp_flag(rbsp, 1);			/* vps_base_layer_available_flag */
	v4l2_hevc_enc_rbsp_bits(rbsp, 0, 6);	/* vps_max_layers_minus1 */
	v4l2_hevc_enc_rbsp_bits(rbsp, sps->sps_max_sub_layers_minus1, 3);
	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING);
	v4l2_hevc_enc_rbsp_bits(rbsp, 0xffff, 16); /* vps_reserved_0xffff_16bits */

	rbsp_ptl(rbsp, &sps_video->ptl, sps->sps_max_sub_layers_minus1);
	rbsp_sub_layer_ordering(rbsp, sps);

	v4l2_hevc_enc_rbsp_bits(rbsp, 0, 6);	/* vps_max_layer_id */
	v4l2_hevc_enc_rbsp_ue(rbsp, 0);		/* vps_num_layer_sets_minus1 */

	rbsp_flag(rbsp, timing);
	if (timing) {
		rbsp_timing_info(rbsp, sps_video);
		v4l2_hevc_enc_rbsp_ue(rbsp, 0);	/* vps_num_hrd_parameters */
	}

	rbsp_flag(rbsp, 0);			/* vps_extension_flag */

	v4l2_hevc_enc_rbsp_nal_end(rbsp);

	return rbsp->error;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_vps);

/* vui_parameters() (E.2.1), without HRD parameters */
static void rbsp_vui(struct v4l2_hevc_enc_rbsp *rbsp,
		     const struct v4l2_hevc_sps_video *video)
{
	u32 flags = video->flags;

	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_ASPECT_RATIO_INFO_PRESENT);
	if (flags & V4L2_HEVC_SPS_VIDEO_FLAG_VUI_ASPECT_RATIO_INFO_PRESENT) {
		v4l2_hevc_enc_rbsp_bits(rbsp, video->aspect_ratio_idc, 8);
		if (video->aspect_ratio_idc ==
		    V4L2_HEVC_VUI_ASPECT_RATIO_IDC_EXTENDED) {
			v4l2_hevc_enc_rbsp_bits(rbsp, video->sar_width, 16);
			v4l2_hevc_enc_rbsp_bits(rbsp, video->sar_height, 16);
		}
	}

	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_OVERSCAN_INFO_PRESENT);
	if (flags & V4L2_HEVC_SPS_VIDEO_FLAG_VUI_OVERSCAN_INFO_PRESENT)
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_OVERSCAN_APPROPRIATE);

	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_SIGNAL_TYPE_PRESENT);
	if (flags & V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_SIGNAL_TYPE_PRESENT) {
		v4l2_hevc_enc_rbsp_bits(rbsp, video->video_format, 3);
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_FULL_RANGE);
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_COLOUR_DESCRIPTION_PRESENT);
		if (flags &
		    V4L2_HEVC_SPS_VIDEO_FLAG_VUI_COLOUR_DESCRIPTION_PRESENT) {
			v4l2_hevc_enc_rbsp_bits(rbsp, video->colour_primaries,
						8);
			v4l2_hevc_enc_rbsp_bits(rbsp,
						video->transfer_characteristics,
						8);
			v4l2_hevc_enc_rbsp_bits(rbsp, video->matrix_coeffs, 8);
		}
	}

	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_CHROMA_LOC_INFO_PRESENT);
	if (flags & V4L2_HEVC_SPS_VIDEO_FLAG_VUI_CHROMA_LOC_INFO_PRESENT) {
		v4l2_hevc_enc_rbsp_ue(rbsp,
				      video->chroma_sample_loc_type_top_field);
		v4l2_hevc_enc_rbsp_ue(rbsp,
				      video->chroma_sample_loc_type_bottom_field);
	}

	rbsp_flag(rbsp, 0);	/* neutral_chroma_indication_flag */
	rbsp_flag(rbsp, 0);	/* field_seq_flag */
	rbsp_flag(rbsp, 0);	/* frame_field_info_present_flag */
	rbsp_flag(rbsp, 0);	/* default_display_window_flag */

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TIMING_INFO_PRESENT);
	if (flags & V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TIMING_INFO_PRESENT) {
		rbsp_timing_info(rbsp, video);
		rbsp_flag(rbsp, 0);	/* vui_hrd_parameters_present_flag */
	}

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_VIDEO_FLAG_VUI_BITSTREAM_RESTRICTION);
	if (flags & V4L2_HEVC_SPS_VIDEO_FLAG_VUI_BITSTREAM_RESTRICTION) {
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TILES_FIXED_STRUCTURE);
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_MOTION_VECTORS_OVER_PIC_BOUNDARIES);
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_VIDEO_FLAG_VUI_RESTRICTED_REF_PIC_LISTS);
		v4l2_hevc_enc_rbsp_ue(rbsp, video->min_spatial_segmentation_idc);
		v4l2_hevc_enc_rbsp_ue(rbsp, video->max_bytes_per_pic_denom);
		v4l2_hevc_enc_rbsp_ue(rbsp, video->max_bits_per_min_cu_denom);
		v4l2_hevc_enc_rbsp_ue(rbsp,
				      video->log2_max_mv_length_horizontal);
		v4l2_hevc_enc_rbsp_ue(rbsp, video->log2_max_mv_length_vertical);
	}
}

/**
 * v4l2_hevc_enc_rbsp_sps() - write a sequence parameter set (7.3.2.2)
 * @rbsp: the writer
 * @start_code: whether to write an Annex B start code
 * @sps: the SPS
 * @sps_video: the VPS/SPS fields not in @sps
 *
 * Short-term reference picture sets and long-term reference pictures
 * cannot be signalled in the SPS (num_short_term_ref_pic_sets and
 * num_long_term_ref_pics_sps must be 0), and scaling lists, when enabled,
 * are the default ones.
 *
 * Return: 0 or the first error of the writer.
 */
int v4l2_hevc_enc_rbsp_sps(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_sps_video *sps_video)
{
	u64 flags = sps->flags;
	bool conformance_window = sps_video->flags &
		V4L2_HEVC_SPS_VIDEO_FLAG_CONFORMANCE_WINDOW;
	bool vui = sps_video->flags &
		V4L2_HEVC_SPS_VIDEO_FLAG_VUI_PARAMETERS_PRESENT;

	if (sps->video_parameter_set_id > 15 ||
	    sps->seq_parameter_set_id > 15 ||
	    sps->sps_max_sub_layers_minus1 >= V4L2_HEVC_MAX_SUB_LAYERS ||
	    sps->chroma_format_idc > 3 ||
	    sps->num_short_term_ref_pic_sets ||
	    sps->num_long_term_ref_pics_sps)
		rbsp_fail(rbsp, -EINVAL);

	v4l2_hevc_enc_rbsp_nal_begin(rbsp, start_code, V4L2_HEVC_NAL_SPS, 1);

	v4l2_hevc_enc_rbsp_bits(rbsp, sps->video_parameter_set_id, 4);
	v4l2_hevc_enc_rbsp_bits(rbsp, sps->sps_max_sub_layers_minus1, 3);
	rbsp_flags(rbsp, sps_video->flags,
		   V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING);
	rbsp_ptl(rbsp, &sps_video->ptl, sps->sps_max_sub_layers_minus1);

	v4l2_hevc_enc_rbsp_ue(rbsp, sps->seq_parameter_set_id);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->chroma_format_idc);
	if (sps->chroma_format_idc == 3)
		rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->pic_width_in_luma_samples);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->pic_height_in_luma_samples);

	rbsp_flag(rbsp, conformance_window);
	if (conformance_window) {
		v4l2_hevc_enc_rbsp_ue(rbsp, sps_video->conf_win_left_offset);
		v4l2_hevc_enc_rbsp_ue(rbsp, sps_video->conf_win_right_offset);
		v4l2_hevc_enc_rbsp_ue(rbsp, sps_video->conf_win_top_offset);
		v4l2_hevc_enc_rbsp_ue(rbsp, sps_video->conf_win_bottom_offset);
	}

	v4l2_hevc_enc_rbsp_ue(rbsp, sps->bit_depth_luma_minus8);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->bit_depth_chroma_minus8);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->log2_max_pic_order_cnt_lsb_minus4);

	rbsp_sub_layer_ordering(rbsp, sps);

	v4l2_hevc_enc_rbsp_ue(rbsp, sps->log2_min_luma_coding_block_size_minus3);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->log2_diff_max_min_luma_coding_block_size);
	v4l2_hevc_enc_rbsp_ue(rbsp,
			      sps->log2_min_luma_transform_block_size_minus2);
	v4l2_hevc_enc_rbsp_ue(rbsp,
			      sps->log2_diff_max_min_luma_transform_block_size);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->max_transform_hierarchy_depth_inter);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->max_transform_hierarchy_depth_intra);

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED);
	if (flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED)
		rbsp_flag(rbsp, 0);	/* sps_scaling_list_data_present_flag */

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_AMP_ENABLED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET);

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_PCM_ENABLED);
	if (flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		v4l2_hevc_enc_rbsp_bits(rbsp,
					sps->pcm_sample_bit_depth_luma_minus1,
					4);
		v4l2_hevc_enc_rbsp_bits(rbsp,
					sps->pcm_sample_bit_depth_chroma_minus1,
					4);
		v4l2_hevc_enc_rbsp_ue(rbsp,
				      sps->log2_min_pcm_luma_coding_block_size_minus3);
		v4l2_hevc_enc_rbsp_ue(rbsp,
				      sps->log2_diff_max_min_pcm_luma_coding_block_size);
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED);
	}

	v4l2_hevc_enc_rbsp_ue(rbsp, 0);		/* num_short_term_ref_pic_sets */

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT);
	if (flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT)
		v4l2_hevc_enc_rbsp_ue(rbsp, 0);	/* num_long_term_ref_pics_sps */

	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED);

	rbsp_flag(rbsp, vui);
	if (vui)
		rbsp_vui(rbsp, sps_video);

	rbsp_flag(rbsp, 0);			/* sps_extension_present_flag */

	v4l2_hevc_enc_rbsp_nal_end(rbsp);

	return rbsp->error;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_sps);

/**
 * v4l2_hevc_enc_rbsp_pps() - write a picture parameter set (7.3.2.3)
 * @rbsp: the writer
 * @start_code: whether to write an Annex B start code
 * @sps: the SPS the PPS refers to
 * @pps: the PPS
 *
 * Scaling lists are never signalled in the PPS and no PPS extension is
 * written.
 *
 * Return: 0 or the first error of the writer.
 */
int v4l2_hevc_enc_rbsp_pps(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_ctrl_hevc_pps *pps)
{
	u64 flags = pps->flags;
	unsigned int i;

	if (pps->pic_parameter_set_id > 63 ||
	    pps->num_extra_slice_header_bits > 7 ||
	    (flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED &&
	     (pps->num_tile_columns_minus1 >=
	      ARRAY_SIZE(pps->column_width_minus1) ||
	      pps->num_tile_rows_minus1 >= ARRAY_SIZE(pps->row_height_minus1))))
		rbsp_fail(rbsp, -EINVAL);

	v4l2_hevc_enc_rbsp_nal_begin(rbsp, start_code, V4L2_HEVC_NAL_PPS, 1);

	v4l2_hevc_enc_rbsp_ue(rbsp, pps->pic_parameter_set_id);
	v4l2_hevc_enc_rbsp_ue(rbsp, sps->seq_parameter_set_id);
	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT);
	v4l2_hevc_enc_rbsp_bits(rbsp, pps->num_extra_slice_header_bits, 3);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT);
	v4l2_hevc_enc_rbsp_ue(rbsp, pps->num_ref_idx_l0_default_active_minus1);
	v4l2_hevc_enc_rbsp_ue(rbsp, pps->num_ref_idx_l1_default_active_minus1);
	v4l2_hevc_enc_rbsp_se(rbsp, pps->init_qp_minus26);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED);

	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED);
	if (flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED)
		v4l2_hevc_enc_rbsp_ue(rbsp, pps->diff_cu_qp_delta_depth);

	v4l2_hevc_enc_rbsp_se(rbsp, pps->pps_cb_qp_offset);
	v4l2_hevc_enc_rbsp_se(rbsp, pps->pps_cr_qp_offset);
	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_TILES_ENABLED);
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED);

	if (flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED && !rbsp->error) {
		bool uniform = flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING;

		v4l2_hevc_enc_rbsp_ue(rbsp, pps->num_tile_columns_minus1);
		v4l2_hevc_enc_rbsp_ue(rbsp, pps->num_tile_rows_minus1);
		rbsp_flag(rbsp, uniform);
		if (!uniform) {
			for (i = 0; i < pps->num_tile_columns_minus1; i++)
				v4l2_hevc_enc_rbsp_ue(rbsp,
						      pps->column_width_minus1[i]);
			for (i = 0; i < pps->num_tile_rows_minus1; i++)
				v4l2_hevc_enc_rbsp_ue(rbsp,
						      pps->row_height_minus1[i]);
		}
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED);
	}

	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED);

	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT);
	if (flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED);
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER);
		if (!(flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER)) {
			v4l2_hevc_enc_rbsp_se(rbsp, pps->pps_beta_offset_div2);
			v4l2_hevc_enc_rbsp_se(rbsp, pps->pps_tc_offset_div2);
		}
	}

	rbsp_flag(rbsp, 0);			/* pps_scaling_list_data_present_flag */
	rbsp_flags(rbsp, flags, V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT);
	v4l2_hevc_enc_rbsp_ue(rbsp, pps->log2_parallel_merge_level_minus2);
	rbsp_flags(rbsp, flags,
		   V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT);
	rbsp_flag(rbsp, 0);			/* pps_extension_present_flag */

	v4l2_hevc_enc_rbsp_nal_end(rbsp);

	return rbsp->error;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_pps);

/* st_ref_pic_set(num_short_term_ref_pic_sets) (7.3.7), explicit form */
static void rbsp_st_rps(struct v4l2_hevc_enc_rbsp *rbsp,
			const struct v4l2_hevc_st_rps *rps,
			unsigned int st_rps_idx)
{
	unsigned int i;

	if (rps->num_negative_pics + rps->num_positive_pics >
	    V4L2_HEVC_MAX_ST_RPS_PICS) {
		rbsp_fail(rbsp, -EINVAL);
		return;
	}

	if (st_rps_idx)
		rbsp_flag(rbsp, 0);	/* inter_ref_pic_set_prediction_flag */

	v4l2_hevc_enc_rbsp_ue(rbsp, rps->num_negative_pics);
	v4l2_hevc_enc_rbsp_ue(rbsp, rps->num_positive_pics);

	for (i = 0; i < rps->num_negative_pics; i++) {
		v4l2_hevc_enc_rbsp_ue(rbsp, rps->delta_poc_s0_minus1[i]);
		rbsp_flag(rbsp, rps->used_by_curr_pic_s0 & BIT(i));
	}

	for (i = 0; i < rps->num_positive_pics; i++) {
		v4l2_hevc_enc_rbsp_ue(rbsp, rps->delta_poc_s1_minus1[i]);
		rbsp_flag(rbsp, rps->used_by_curr_pic_s1 & BIT(i));
	}
}

static unsigned int num_pic_total_curr(const struct v4l2_hevc_slice_header *sh)
{
	unsigned int count;
	unsigned int i;

	count = hweight32(sh->st_rps.used_by_curr_pic_s0 &
			  (BIT(sh->st_rps.num_negative_pics) - 1));
	count += hweight32(sh->st_rps.used_by_curr_pic_s1 &
			   (BIT(sh->st_rps.num_positive_pics) - 1));

	for (i = 0; i < sh->num_long_term_pics; i++)
		count += sh->lt[i].used_by_curr_pic_lt;

	return count;
}

/**
 * v4l2_hevc_enc_rbsp_slice_header() - write a slice segment header (7.3.6)
 * @rbsp: the writer
 * @start_code: whether to write an Annex B start code
 * @sps: the active SPS
 * @pps: the active PPS
 * @sh: the slice segment header syntax element values
 *
 * Writes the NAL unit header, then slice_segment_header() and
 * byte_alignment(): the caller (usually the hardware) appends
 * slice_segment_data() and the trailing bits. The short-term reference
 * picture set is always coded in the slice header. Reference picture list
 * modification is written as "no modification" and weighted prediction
 * tables are not supported.
 *
 * Return: 0 or the first error of the writer.
 */
int v4l2_hevc_enc_rbsp_slice_header(struct v4l2_hevc_enc_rbsp *rbsp,
				    bool start_code,
				    const struct v4l2_ctrl_hevc_sps *sps,
				    const struct v4l2_ctrl_hevc_pps *pps,
				    const struct v4l2_hevc_slice_header *sh)
{
	unsigned int ctb_log2_size = sps->log2_min_luma_coding_block_size_minus3 +
				     3 + sps->log2_diff_max_min_luma_coding_block_size;
	unsigned int width_ctbs = DIV_ROUND_UP(sps->pic_width_in_luma_samples,
					       1 << ctb_log2_size);
	unsigned int height_ctbs = DIV_ROUND_UP(sps->pic_height_in_luma_samples,
						1 << ctb_log2_size);
	unsigned int poc_lsb_bits = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
	bool first = !sh->slice_segment_address;
	bool dependent = sh->flags &
			 V4L2_HEVC_SLICE_HEADER_FLAG_DEPENDENT_SLICE_SEGMENT;
	bool inter = sh->slice_type != V4L2_HEVC_SLICE_TYPE_I;
	bool bipred = sh->slice_type == V4L2_HEVC_SLICE_TYPE_B;
	bool temporal_mvp = false;
	bool sao_luma = false;
	bool sao_chroma = false;
	bool deblocking_disabled;
	u64 flags = sh->flags;
	u64 pps_flags = pps->flags;
	unsigned int i;

	if (sh->slice_type > V4L2_HEVC_SLICE_TYPE_I ||
	    (first && dependent) ||
	    sh->num_long_term_pics > V4L2_HEVC_MAX_LT_PICS ||
	    (sh->num_long_term_pics &&
	     !(sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT)) ||
	    sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE ||
	    (sh->slice_type == V4L2_HEVC_SLICE_TYPE_P &&
	     pps_flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) ||
	    (bipred && pps_flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) ||
	    sh->slice_segment_address >= width_ctbs * height_ctbs ||
	    (sh->num_entry_point_offsets && !sh->entry_point_offset_minus1))
		rbsp_fail(rbsp, -EINVAL);

	if (rbsp->error)
		return rbsp->error;

	v4l2_hevc_enc_rbsp_nal_begin(rbsp, start_code, sh->nal_unit_type,
				     sh->nuh_temporal_id_plus1);

	rbsp_flag(rbsp, first);
	if (v4l2_hevc_nal_is_irap(sh->nal_unit_type))
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_ENCODE_FLAG_NO_OUTPUT_OF_PRIOR_PICS);
	v4l2_hevc_enc_rbsp_ue(rbsp, pps->pic_parameter_set_id);

	if (!first) {
		if (pps_flags & V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED)
			rbsp_flag(rbsp, dependent);
		v4l2_hevc_enc_rbsp_bits(rbsp, sh->slice_segment_address,
					order_base_2(width_ctbs * height_ctbs));
	}

	deblocking_disabled =
		pps_flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;

	if (dependent)
		goto entry_points;

	/* slice_reserved_flag[] */
	v4l2_hevc_enc_rbsp_bits(rbsp, 0, pps->num_extra_slice_header_bits);
	v4l2_hevc_enc_rbsp_ue(rbsp, sh->slice_type);

	if (pps_flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)
		rbsp_flag(rbsp, !(flags & V4L2_HEVC_ENCODE_FLAG_NO_PIC_OUTPUT));

	if (!v4l2_hevc_nal_is_idr(sh->nal_unit_type)) {
		v4l2_hevc_enc_rbsp_bits(rbsp, sh->slice_pic_order_cnt_lsb,
					poc_lsb_bits);

		/* short_term_ref_pic_set_sps_flag */
		rbsp_flag(rbsp, 0);
		rbsp_st_rps(rbsp, &sh->st_rps, sps->num_short_term_ref_pic_sets);

		if (sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) {
			v4l2_hevc_enc_rbsp_ue(rbsp, sh->num_long_term_pics);
			for (i = 0; i < sh->num_long_term_pics; i++) {
				const struct v4l2_hevc_lt_pic *lt = &sh->lt[i];

				v4l2_hevc_enc_rbsp_bits(rbsp, lt->poc_lsb_lt,
							poc_lsb_bits);
				rbsp_flag(rbsp, lt->used_by_curr_pic_lt);
				rbsp_flag(rbsp, lt->delta_poc_msb_present);
				if (lt->delta_poc_msb_present)
					v4l2_hevc_enc_rbsp_ue(rbsp,
							      lt->delta_poc_msb_cycle_lt);
			}
		}

		if (sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) {
			temporal_mvp = flags &
				V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
			rbsp_flag(rbsp, temporal_mvp);
		}
	}

	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) {
		sao_luma = flags & V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_LUMA;
		rbsp_flag(rbsp, sao_luma);
		if (sps->chroma_format_idc) {
			sao_chroma = flags &
				     V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_CHROMA;
			rbsp_flag(rbsp, sao_chroma);
		}
	}

	if (inter) {
		bool override = flags &
			V4L2_HEVC_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE;
		unsigned int l0_active = pps->num_ref_idx_l0_default_active_minus1;
		unsigned int l1_active = pps->num_ref_idx_l1_default_active_minus1;

		rbsp_flag(rbsp, override);
		if (override) {
			l0_active = sh->num_ref_idx_l0_active_minus1;
			v4l2_hevc_enc_rbsp_ue(rbsp, l0_active);
			if (bipred) {
				l1_active = sh->num_ref_idx_l1_active_minus1;
				v4l2_hevc_enc_rbsp_ue(rbsp, l1_active);
			}
		}

		if (pps_flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT &&
		    num_pic_total_curr(sh) > 1) {
			/* ref_pic_list_modification_flag_l0 and _l1 */
			rbsp_flag(rbsp, 0);
			if (bipred)
				rbsp_flag(rbsp, 0);
		}

		if (bipred)
			rbsp_flags(rbsp, flags, V4L2_HEVC_ENCODE_FLAG_MVD_L1_ZERO);

		if (pps_flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT)
			rbsp_flags(rbsp, flags, V4L2_HEVC_ENCODE_FLAG_CABAC_INIT);

		if (temporal_mvp) {
			bool from_l0 = !bipred ||
				       flags & V4L2_HEVC_ENCODE_FLAG_COLLOCATED_FROM_L0;

			if (bipred)
				rbsp_flag(rbsp, from_l0);

			if ((from_l0 && l0_active) || (!from_l0 && l1_active))
				v4l2_hevc_enc_rbsp_ue(rbsp,
						      sh->collocated_ref_idx);
		}

		v4l2_hevc_enc_rbsp_ue(rbsp, sh->five_minus_max_num_merge_cand);
	}

	v4l2_hevc_enc_rbsp_se(rbsp, sh->slice_qp_delta);

	if (pps_flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) {
		v4l2_hevc_enc_rbsp_se(rbsp, sh->slice_cb_qp_offset);
		v4l2_hevc_enc_rbsp_se(rbsp, sh->slice_cr_qp_offset);
	}

	if (pps_flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) {
		bool override = flags &
			V4L2_HEVC_ENCODE_FLAG_DEBLOCKING_FILTER_OVERRIDE;

		rbsp_flag(rbsp, override);
		if (override) {
			deblocking_disabled = flags &
				V4L2_HEVC_ENCODE_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;
			rbsp_flag(rbsp, deblocking_disabled);
			if (!deblocking_disabled) {
				v4l2_hevc_enc_rbsp_se(rbsp,
						      sh->slice_beta_offset_div2);
				v4l2_hevc_enc_rbsp_se(rbsp,
						      sh->slice_tc_offset_div2);
			}
		}
	}

	if (pps_flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED &&
	    (sao_luma || sao_chroma || !deblocking_disabled))
		rbsp_flags(rbsp, flags,
			   V4L2_HEVC_ENCODE_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED);

entry_points:
	if (pps_flags & (V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
			 V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) {
		v4l2_hevc_enc_rbsp_ue(rbsp, sh->num_entry_point_offsets);
		if (sh->num_entry_point_offsets) {
			unsigned int offset_len;
			u32 offset_max = 0;

			for (i = 0; i < sh->num_entry_point_offsets; i++)
				offset_max = max(offset_max,
						 sh->entry_point_offset_minus1[i]);

			offset_len = offset_max ? fls(offset_max) : 1;

			/* offset_len_minus1 */
			v4l2_hevc_enc_rbsp_ue(rbsp, offset_len - 1);
			for (i = 0; i < sh->num_entry_point_offsets; i++)
				v4l2_hevc_enc_rbsp_bits(rbsp,
							sh->entry_point_offset_minus1[i],
							offset_len);
		}
	}

	/* slice_segment_header_extension_length */
	if (pps_flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT)
		v4l2_hevc_enc_rbsp_ue(rbsp, 0);

	/* byte_alignment() */
	v4l2_hevc_enc_rbsp_trailing_bits(rbsp);

	return rbsp->error;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_slice_header);

MODULE_DESCRIPTION("V4L2 HEVC Encode RBSP");
MODULE_LICENSE("GPL");
