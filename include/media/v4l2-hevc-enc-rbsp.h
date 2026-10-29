/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * V4L2 HEVC Encode RBSP
 *
 * Writes HEVC NAL units (VPS, SPS, PPS, access unit delimiter and slice
 * segment headers) into a buffer, with Annex B start codes and emulation
 * prevention.
 */

#ifndef _MEDIA_V4L2_HEVC_ENC_RBSP_H
#define _MEDIA_V4L2_HEVC_ENC_RBSP_H

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>

/**
 * struct v4l2_hevc_enc_rbsp - HEVC NAL unit writer
 *
 * @data: start of the destination buffer
 * @size: size of the destination buffer in bytes
 * @offset: number of bytes written to @data
 * @cache: bits not yet written to @data
 * @cache_bits: number of valid bits in @cache, always less than 8 between
 *		calls
 * @zero_bytes: number of consecutive zero bytes written last, used for
 *		emulation prevention
 * @epb: whether emulation prevention is active (inside a NAL unit payload)
 * @error: first error that happened, sticky until the next init. Writes
 *	   after an error are ignored.
 */
struct v4l2_hevc_enc_rbsp {
	u8 *data;
	unsigned int size;
	unsigned int offset;
	u64 cache;
	unsigned int cache_bits;
	unsigned int zero_bytes;
	bool epb;
	int error;
};

#define V4L2_HEVC_PTL_FLAG_PROGRESSIVE_SOURCE		0x1
#define V4L2_HEVC_PTL_FLAG_INTERLACED_SOURCE		0x2
#define V4L2_HEVC_PTL_FLAG_NON_PACKED_CONSTRAINT	0x4
#define V4L2_HEVC_PTL_FLAG_FRAME_ONLY_CONSTRAINT	0x8

/**
 * struct v4l2_hevc_ptl - HEVC general profile, tier and level (7.3.3)
 *
 * Sub-layer profile and level information is never written.
 *
 * @general_profile_space: general_profile_space
 * @general_tier_flag: general_tier_flag
 * @general_profile_idc: general_profile_idc
 * @general_level_idc: general_level_idc (30 times the level number)
 * @general_profile_compatibility_flags: general_profile_compatibility_flag[j]
 *					 is bit (31 - j)
 * @flags: see V4L2_HEVC_PTL_FLAG_{}
 */
struct v4l2_hevc_ptl {
	u8 general_profile_space;
	u8 general_tier_flag;
	u8 general_profile_idc;
	u8 general_level_idc;
	u32 general_profile_compatibility_flags;
	u32 flags;
};

#define V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING			BIT(0)
#define V4L2_HEVC_SPS_VIDEO_FLAG_CONFORMANCE_WINDOW			BIT(1)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VPS_TIMING_INFO_PRESENT		BIT(2)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_PARAMETERS_PRESENT			BIT(3)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_ASPECT_RATIO_INFO_PRESENT		BIT(4)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_OVERSCAN_INFO_PRESENT		BIT(5)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_OVERSCAN_APPROPRIATE		BIT(6)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_SIGNAL_TYPE_PRESENT		BIT(7)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_FULL_RANGE			BIT(8)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_COLOUR_DESCRIPTION_PRESENT		BIT(9)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_CHROMA_LOC_INFO_PRESENT		BIT(10)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TIMING_INFO_PRESENT		BIT(11)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_POC_PROPORTIONAL_TO_TIMING		BIT(12)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_BITSTREAM_RESTRICTION		BIT(13)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TILES_FIXED_STRUCTURE		BIT(14)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_MOTION_VECTORS_OVER_PIC_BOUNDARIES	BIT(15)
#define V4L2_HEVC_SPS_VIDEO_FLAG_VUI_RESTRICTED_REF_PIC_LISTS		BIT(16)

/**
 * struct v4l2_hevc_sps_video - HEVC VPS/SPS fields not in v4l2_ctrl_hevc_sps
 *
 * The parts of the parameter sets that describe the video rather than the
 * coding tools. The stateless encode core derives them from the format,
 * the frame interval and the stateful codec controls.
 *
 * @ptl: general profile, tier and level
 * @flags: see V4L2_HEVC_SPS_VIDEO_FLAG_{}
 * @conf_win_left_offset: conformance window left offset, in chroma samples
 * @conf_win_right_offset: conformance window right offset, in chroma samples
 * @conf_win_top_offset: conformance window top offset, in chroma samples
 * @conf_win_bottom_offset: conformance window bottom offset, in chroma
 *			    samples
 * @aspect_ratio_idc: aspect_ratio_idc (Table E.1)
 * @sar_width: sar_width, with V4L2_HEVC_VUI_ASPECT_RATIO_IDC_EXTENDED
 * @sar_height: sar_height, with V4L2_HEVC_VUI_ASPECT_RATIO_IDC_EXTENDED
 * @video_format: video_format (Table E.2)
 * @colour_primaries: colour_primaries (Table E.3)
 * @transfer_characteristics: transfer_characteristics (Table E.4)
 * @matrix_coeffs: matrix_coeffs (Table E.5)
 * @chroma_sample_loc_type_top_field: chroma_sample_loc_type_top_field
 * @chroma_sample_loc_type_bottom_field: chroma_sample_loc_type_bottom_field
 * @num_units_in_tick: num_units_in_tick of the VUI and VPS timing info
 * @time_scale: time_scale of the VUI and VPS timing info
 * @num_ticks_poc_diff_one_minus1: num_ticks_poc_diff_one_minus1
 * @min_spatial_segmentation_idc: min_spatial_segmentation_idc
 * @max_bytes_per_pic_denom: max_bytes_per_pic_denom
 * @max_bits_per_min_cu_denom: max_bits_per_min_cu_denom
 * @log2_max_mv_length_horizontal: log2_max_mv_length_horizontal
 * @log2_max_mv_length_vertical: log2_max_mv_length_vertical
 */
struct v4l2_hevc_sps_video {
	struct v4l2_hevc_ptl ptl;
	u32 flags;

	u16 conf_win_left_offset;
	u16 conf_win_right_offset;
	u16 conf_win_top_offset;
	u16 conf_win_bottom_offset;

	u8 aspect_ratio_idc;
	u16 sar_width;
	u16 sar_height;

	u8 video_format;
	u8 colour_primaries;
	u8 transfer_characteristics;
	u8 matrix_coeffs;

	u8 chroma_sample_loc_type_top_field;
	u8 chroma_sample_loc_type_bottom_field;

	u32 num_units_in_tick;
	u32 time_scale;
	u32 num_ticks_poc_diff_one_minus1;

	u16 min_spatial_segmentation_idc;
	u8 max_bytes_per_pic_denom;
	u8 max_bits_per_min_cu_denom;
	u8 log2_max_mv_length_horizontal;
	u8 log2_max_mv_length_vertical;
};

/**
 * struct v4l2_hevc_st_rps - HEVC short-term reference picture set (7.3.7)
 *
 * Always written explicitly (inter_ref_pic_set_prediction_flag = 0).
 *
 * @num_negative_pics: num_negative_pics
 * @num_positive_pics: num_positive_pics
 * @used_by_curr_pic_s0: bit i is used_by_curr_pic_s0_flag[i]
 * @used_by_curr_pic_s1: bit i is used_by_curr_pic_s1_flag[i]
 * @delta_poc_s0_minus1: delta_poc_s0_minus1[]
 * @delta_poc_s1_minus1: delta_poc_s1_minus1[]
 */
struct v4l2_hevc_st_rps {
	u8 num_negative_pics;
	u8 num_positive_pics;
	u16 used_by_curr_pic_s0;
	u16 used_by_curr_pic_s1;
	u16 delta_poc_s0_minus1[16];
	u16 delta_poc_s1_minus1[16];
};

/**
 * struct v4l2_hevc_lt_pic - HEVC long-term picture of a slice header
 *
 * @poc_lsb_lt: poc_lsb_lt[i]
 * @used_by_curr_pic_lt: used_by_curr_pic_lt_flag[i]
 * @delta_poc_msb_present: delta_poc_msb_present_flag[i]
 * @delta_poc_msb_cycle_lt: delta_poc_msb_cycle_lt[i]
 */
struct v4l2_hevc_lt_pic {
	u16 poc_lsb_lt;
	bool used_by_curr_pic_lt;
	bool delta_poc_msb_present;
	u32 delta_poc_msb_cycle_lt;
};

/* Slice header flags beyond the V4L2_HEVC_ENCODE_FLAG_{} uAPI flags. */
#define V4L2_HEVC_SLICE_HEADER_FLAG_DEPENDENT_SLICE_SEGMENT	BIT_ULL(32)

/**
 * struct v4l2_hevc_slice_header - HEVC slice segment header (7.3.6)
 *
 * The values of the syntax elements, resolved. Elements that the active
 * SPS/PPS make absent are ignored.
 *
 * @nal_unit_type: nal_unit_type
 * @nuh_temporal_id_plus1: nuh_temporal_id_plus1
 * @slice_type: slice_type
 * @slice_segment_address: slice_segment_address, 0 for the first slice
 *			   segment of the picture
 * @slice_pic_order_cnt_lsb: slice_pic_order_cnt_lsb
 * @st_rps: short-term reference picture set, coded in the slice header
 * @num_long_term_pics: num_long_term_pics
 * @lt: long-term pictures
 * @num_ref_idx_l0_active_minus1: num_ref_idx_l0_active_minus1
 * @num_ref_idx_l1_active_minus1: num_ref_idx_l1_active_minus1
 * @collocated_ref_idx: collocated_ref_idx
 * @five_minus_max_num_merge_cand: five_minus_max_num_merge_cand
 * @slice_qp_delta: slice_qp_delta
 * @slice_cb_qp_offset: slice_cb_qp_offset
 * @slice_cr_qp_offset: slice_cr_qp_offset
 * @slice_beta_offset_div2: slice_beta_offset_div2
 * @slice_tc_offset_div2: slice_tc_offset_div2
 * @num_entry_point_offsets: num_entry_point_offsets
 * @entry_point_offset_minus1: entry_point_offset_minus1[], must point to
 *			       @num_entry_point_offsets values when non-zero
 * @flags: V4L2_HEVC_ENCODE_FLAG_{} and V4L2_HEVC_SLICE_HEADER_FLAG_{}
 */
struct v4l2_hevc_slice_header {
	u8 nal_unit_type;
	u8 nuh_temporal_id_plus1;
	u8 slice_type;
	u32 slice_segment_address;
	u16 slice_pic_order_cnt_lsb;

	struct v4l2_hevc_st_rps st_rps;
	u8 num_long_term_pics;
	struct v4l2_hevc_lt_pic lt[16];

	u8 num_ref_idx_l0_active_minus1;
	u8 num_ref_idx_l1_active_minus1;
	u8 collocated_ref_idx;
	u8 five_minus_max_num_merge_cand;
	s8 slice_qp_delta;
	s8 slice_cb_qp_offset;
	s8 slice_cr_qp_offset;
	s8 slice_beta_offset_div2;
	s8 slice_tc_offset_div2;

	u32 num_entry_point_offsets;
	const u32 *entry_point_offset_minus1;

	u64 flags;
};

void v4l2_hevc_enc_rbsp_init(struct v4l2_hevc_enc_rbsp *rbsp, u8 *data,
			     unsigned int size);
unsigned int v4l2_hevc_enc_rbsp_bytes(const struct v4l2_hevc_enc_rbsp *rbsp);
int v4l2_hevc_enc_rbsp_error(const struct v4l2_hevc_enc_rbsp *rbsp);

void v4l2_hevc_enc_rbsp_bits(struct v4l2_hevc_enc_rbsp *rbsp, u32 value,
			     unsigned int count);
void v4l2_hevc_enc_rbsp_ue(struct v4l2_hevc_enc_rbsp *rbsp, u32 value);
void v4l2_hevc_enc_rbsp_se(struct v4l2_hevc_enc_rbsp *rbsp, s32 value);
void v4l2_hevc_enc_rbsp_trailing_bits(struct v4l2_hevc_enc_rbsp *rbsp);
void v4l2_hevc_enc_rbsp_nal_begin(struct v4l2_hevc_enc_rbsp *rbsp,
				  bool start_code, u8 nal_unit_type,
				  u8 nuh_temporal_id_plus1);
void v4l2_hevc_enc_rbsp_nal_end(struct v4l2_hevc_enc_rbsp *rbsp);

int v4l2_hevc_enc_rbsp_aud(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   u8 pic_type, u8 nuh_temporal_id_plus1);
int v4l2_hevc_enc_rbsp_vps(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_sps_video *sps_video);
int v4l2_hevc_enc_rbsp_sps(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_sps_video *sps_video);
int v4l2_hevc_enc_rbsp_pps(struct v4l2_hevc_enc_rbsp *rbsp, bool start_code,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_ctrl_hevc_pps *pps);
int v4l2_hevc_enc_rbsp_slice_header(struct v4l2_hevc_enc_rbsp *rbsp,
				    bool start_code,
				    const struct v4l2_ctrl_hevc_sps *sps,
				    const struct v4l2_ctrl_hevc_pps *pps,
				    const struct v4l2_hevc_slice_header *sh);

#endif
