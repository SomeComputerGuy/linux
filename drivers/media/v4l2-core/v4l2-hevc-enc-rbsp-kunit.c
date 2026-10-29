// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * KUnit tests for the V4L2 HEVC encode RBSP writer
 *
 * The expected bitstreams come from an independent reference writer
 * (bit lists per syntax element, then a separate emulation prevention
 * pass), and were parsed back with an independent syntax parser. The VPS
 * of configuration A also matches the VPS commonly found in x265 output for
 * a single-layer Main level 3.1 stream with 5 DPB pictures and 2 reorder
 * pictures.
 */

#include <kunit/test.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/v4l2-controls.h>
#include <media/v4l2-hevc.h>
#include <media/v4l2-hevc-enc-rbsp.h>

#define BUF_SIZE	256

/* Expected bitstreams */

static const u8 vps_a[] = {
	0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0x0c, 0x01, 0xff, 0xff, 0x01, 0x60,
	0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00,
	0x5d, 0x95, 0x98, 0x09,
};

static const u8 sps_a[] = {
	0x00, 0x00, 0x00, 0x01, 0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03,
	0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x5d, 0xa0, 0x02,
	0x80, 0x80, 0x2d, 0x16, 0x59, 0x59, 0xa4, 0x93, 0x6b, 0x9a, 0x80, 0x80,
	0x80, 0x82, 0x00, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x03, 0x00, 0x32,
	0x10,
};

static const u8 pps_a[] = {
	0x00, 0x00, 0x00, 0x01, 0x44, 0x01, 0xc1, 0x71, 0x81, 0x12,
};

static const u8 vps_b[] = {
	0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0x1c, 0x01, 0xff, 0xff, 0x01, 0x60,
	0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00,
	0x5a, 0xbc, 0x09,
};

static const u8 sps_b[] = {
	0x00, 0x00, 0x00, 0x01, 0x42, 0x01, 0x11, 0x01, 0x60, 0x00, 0x00, 0x03,
	0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x5a, 0x48, 0x01,
	0x40, 0x80, 0x5c, 0x7c, 0xbe, 0xfb, 0x91, 0x21, 0x77, 0xaf, 0x88,
};

static const u8 pps_b[] = {
	0x00, 0x00, 0x00, 0x01, 0x44, 0x01, 0x4b, 0x5a, 0x89, 0xe8, 0xa6, 0x89,
	0x2f, 0x8a, 0x97, 0x40,
};

static const u8 vps_c[] = {
	0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0x2c, 0x05, 0xff, 0xff, 0x22, 0x20,
	0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00,
	0x99, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00, 0x03, 0x03, 0xe9, 0x00, 0x00,
	0xea, 0x60, 0xe8,
};

static const u8 sps_c[] = {
	0x00, 0x00, 0x00, 0x01, 0x42, 0x01, 0x25, 0x22, 0x20, 0x00, 0x00, 0x03,
	0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x99, 0x00, 0x00,
	0x68, 0x00, 0x78, 0x08, 0x00, 0x87, 0x13, 0x63, 0x46, 0xe4, 0x91, 0xba,
	0x5f, 0xfe, 0x00, 0x08, 0x00, 0x07, 0x6e, 0x12, 0x20, 0x13, 0x6c, 0x20,
	0x00, 0x00, 0x7d, 0x20, 0x00, 0x1d, 0x4c, 0x1b, 0xed, 0x04, 0x02, 0x08,
};

static const u8 pps_c[] = {
	0x00, 0x00, 0x00, 0x01, 0x44, 0x01, 0x6c, 0x06, 0x10, 0x2f, 0x0d, 0xa0,
	0x50, 0x28, 0x11, 0x29, 0x20,
};

static const u8 slice_a_idr[] = {
	0x00, 0x00, 0x00, 0x01, 0x26, 0x01, 0xaf, 0x22,
};

static const u8 slice_a_p[] = {
	0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0xd0, 0x41, 0xf4, 0xc6, 0x7c,
};

static const u8 slice_b_b[] = {
	0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x20, 0x29, 0x19, 0x2d, 0x50, 0x2b,
	0x48, 0xcb, 0x4e, 0x21, 0x64, 0x16, 0x18, 0xfe, 0x78, 0x06, 0xc0,
};

static const u8 slice_b_dep[] = {
	0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x28, 0x57,
};

static const u8 slice_b_cra[] = {
	0x00, 0x00, 0x00, 0x01, 0x2a, 0x01, 0xd0, 0xe0, 0xe7, 0xfc,
};

static const u8 slice_c_tsa[] = {
	0x00, 0x00, 0x00, 0x01, 0x04, 0x03, 0xb4, 0x1f, 0xfe, 0x55, 0x10, 0xa6,
	0x6c, 0x7b, 0x80,
};

static const u8 aud[] = {
	0x00, 0x00, 0x00, 0x01, 0x46, 0x02, 0x30,
};

static const u8 exp_golomb[] = {
	0xa6, 0x42, 0x98, 0xe2, 0x04, 0xa6, 0x42, 0xc7, 0x0f, 0xde, 0xad, 0xbe,
	0xef, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff,
};

/* Configurations */

#define MAIN_PTL(level)							\
	{								\
		.general_profile_idc = V4L2_HEVC_PROFILE_IDC_MAIN,	\
		.general_profile_compatibility_flags = 0x60000000,	\
		.general_level_idc = (level),				\
		.flags = V4L2_HEVC_PTL_FLAG_PROGRESSIVE_SOURCE |	\
			 V4L2_HEVC_PTL_FLAG_FRAME_ONLY_CONSTRAINT,	\
	}

/* A: 720p Main, CTB 64, SAO, AMP, TMVP, VUI with colour and timing */
static const struct v4l2_ctrl_hevc_sps sps_a_cfg = {
	.pic_width_in_luma_samples = 1280,
	.pic_height_in_luma_samples = 720,
	.log2_max_pic_order_cnt_lsb_minus4 = 4,
	.sps_max_dec_pic_buffering_minus1 = 4,
	.sps_max_num_reorder_pics = 2,
	.sps_max_latency_increase_plus1 = 5,
	.log2_diff_max_min_luma_coding_block_size = 3,
	.log2_diff_max_min_luma_transform_block_size = 3,
	.chroma_format_idc = 1,
	.flags = V4L2_HEVC_SPS_FLAG_AMP_ENABLED |
		 V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET |
		 V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED |
		 V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED,
};

static const struct v4l2_hevc_sps_video video_a_cfg = {
	.ptl = MAIN_PTL(93),
	.flags = V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_PARAMETERS_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_SIGNAL_TYPE_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_COLOUR_DESCRIPTION_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TIMING_INFO_PRESENT,
	.video_format = V4L2_HEVC_VUI_VIDEO_FORMAT_UNSPECIFIED,
	.colour_primaries = V4L2_HEVC_VUI_COLOUR_BT709,
	.transfer_characteristics = V4L2_HEVC_VUI_TRANSFER_BT709,
	.matrix_coeffs = V4L2_HEVC_VUI_MATRIX_BT709,
	.num_units_in_tick = 1,
	.time_scale = 25,
};

static const struct v4l2_ctrl_hevc_pps pps_a_cfg = {
	.flags = V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED |
		 V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED,
};

/*
 * B: 640x360 coded as 640x368 with a conformance window, CTB 32, PCM,
 * long-term references, and a PPS with most optional slice header syntax
 * enabled: dependent slices, output flag, extra bits, CABAC init, chroma QP
 * offsets, 2x2 uniform tiles, deblocking override, list modification and
 * header extension.
 */
static const struct v4l2_ctrl_hevc_sps sps_b_cfg = {
	.video_parameter_set_id = 1,
	.seq_parameter_set_id = 1,
	.pic_width_in_luma_samples = 640,
	.pic_height_in_luma_samples = 368,
	.sps_max_dec_pic_buffering_minus1 = 2,
	.log2_diff_max_min_luma_coding_block_size = 2,
	.log2_diff_max_min_luma_transform_block_size = 3,
	.max_transform_hierarchy_depth_inter = 1,
	.max_transform_hierarchy_depth_intra = 1,
	.pcm_sample_bit_depth_luma_minus1 = 7,
	.pcm_sample_bit_depth_chroma_minus1 = 7,
	.log2_diff_max_min_pcm_luma_coding_block_size = 1,
	.chroma_format_idc = 1,
	.flags = V4L2_HEVC_SPS_FLAG_PCM_ENABLED |
		 V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED |
		 V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT |
		 V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED,
};

static const struct v4l2_hevc_sps_video video_b_cfg = {
	.ptl = MAIN_PTL(90),
	.flags = V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING |
		 V4L2_HEVC_SPS_VIDEO_FLAG_CONFORMANCE_WINDOW,
	.conf_win_bottom_offset = 4,
};

static const struct v4l2_ctrl_hevc_pps pps_b_cfg = {
	.pic_parameter_set_id = 1,
	.num_extra_slice_header_bits = 2,
	.num_ref_idx_l0_default_active_minus1 = 1,
	.init_qp_minus26 = -4,
	.diff_cu_qp_delta_depth = 1,
	.pps_cb_qp_offset = -2,
	.pps_cr_qp_offset = 3,
	.num_tile_columns_minus1 = 1,
	.num_tile_rows_minus1 = 1,
	.pps_beta_offset_div2 = -2,
	.pps_tc_offset_div2 = 1,
	.log2_parallel_merge_level_minus2 = 2,
	.flags = V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED |
		 V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT |
		 V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED |
		 V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT |
		 V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED |
		 V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED |
		 V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED |
		 V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT |
		 V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
		 V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING |
		 V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED |
		 V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED |
		 V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT |
		 V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED |
		 V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT |
		 V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT,
};

/*
 * C: 2160p60 Main 10 high tier level 5.1, 3 temporal sub-layers, default
 * scaling lists, full VUI (extended SAR, BT.2020 PQ full range, chroma
 * location, timing, bitstream restriction), VPS timing, 3x2 non-uniform
 * tiles with wavefront parallel processing and deblocking disabled.
 */
static const struct v4l2_ctrl_hevc_sps sps_c_cfg = {
	.video_parameter_set_id = 2,
	.seq_parameter_set_id = 2,
	.pic_width_in_luma_samples = 3840,
	.pic_height_in_luma_samples = 2160,
	.bit_depth_luma_minus8 = 2,
	.bit_depth_chroma_minus8 = 2,
	.log2_max_pic_order_cnt_lsb_minus4 = 12,
	.sps_max_dec_pic_buffering_minus1 = 5,
	.log2_diff_max_min_luma_coding_block_size = 3,
	.log2_diff_max_min_luma_transform_block_size = 3,
	.max_transform_hierarchy_depth_inter = 2,
	.max_transform_hierarchy_depth_intra = 2,
	.chroma_format_idc = 1,
	.sps_max_sub_layers_minus1 = 2,
	.flags = V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED |
		 V4L2_HEVC_SPS_FLAG_AMP_ENABLED |
		 V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED |
		 V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED,
};

static const struct v4l2_hevc_sps_video video_c_cfg = {
	.ptl = {
		.general_tier_flag = 1,
		.general_profile_idc = V4L2_HEVC_PROFILE_IDC_MAIN_10,
		.general_profile_compatibility_flags = 0x20000000,
		.general_level_idc = 153,
		.flags = V4L2_HEVC_PTL_FLAG_PROGRESSIVE_SOURCE |
			 V4L2_HEVC_PTL_FLAG_FRAME_ONLY_CONSTRAINT,
	},
	.flags = V4L2_HEVC_SPS_VIDEO_FLAG_TEMPORAL_ID_NESTING |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VPS_TIMING_INFO_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_PARAMETERS_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_ASPECT_RATIO_INFO_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_OVERSCAN_INFO_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_SIGNAL_TYPE_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_VIDEO_FULL_RANGE |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_COLOUR_DESCRIPTION_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_CHROMA_LOC_INFO_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TIMING_INFO_PRESENT |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_POC_PROPORTIONAL_TO_TIMING |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_BITSTREAM_RESTRICTION |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_TILES_FIXED_STRUCTURE |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_MOTION_VECTORS_OVER_PIC_BOUNDARIES |
		 V4L2_HEVC_SPS_VIDEO_FLAG_VUI_RESTRICTED_REF_PIC_LISTS,
	.aspect_ratio_idc = V4L2_HEVC_VUI_ASPECT_RATIO_IDC_EXTENDED,
	.sar_width = 4,
	.sar_height = 3,
	.video_format = V4L2_HEVC_VUI_VIDEO_FORMAT_UNSPECIFIED,
	.colour_primaries = V4L2_HEVC_VUI_COLOUR_BT2020,
	.transfer_characteristics = V4L2_HEVC_VUI_TRANSFER_SMPTE2084,
	.matrix_coeffs = V4L2_HEVC_VUI_MATRIX_BT2020_NCL,
	.chroma_sample_loc_type_top_field = 2,
	.chroma_sample_loc_type_bottom_field = 2,
	.num_units_in_tick = 1001,
	.time_scale = 60000,
	.max_bytes_per_pic_denom = 2,
	.max_bits_per_min_cu_denom = 1,
	.log2_max_mv_length_horizontal = 15,
	.log2_max_mv_length_vertical = 15,
};

static const struct v4l2_ctrl_hevc_pps pps_c_cfg = {
	.pic_parameter_set_id = 2,
	.init_qp_minus26 = 8,
	.diff_cu_qp_delta_depth = 2,
	.num_tile_columns_minus1 = 2,
	.num_tile_rows_minus1 = 1,
	.column_width_minus1 = { 19, 19 },
	.row_height_minus1 = { 16 },
	.flags = V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED |
		 V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
		 V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED |
		 V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT |
		 V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER,
};

static const u32 entry_points_b[] = { 99, 1999, 6 };
static const u32 entry_points_c[] = { 49, 59 };

struct hevc_rbsp_slice_case {
	const char *name;
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	struct v4l2_hevc_slice_header sh;
	const u8 *expected;
	unsigned int expected_size;
};

#define SLICE_CASE(n, s, p, e, ...)					\
	{								\
		.name = n, .sps = s, .pps = p,				\
		.sh = { __VA_ARGS__ },					\
		.expected = e, .expected_size = sizeof(e),		\
	}

static const struct hevc_rbsp_slice_case slice_cases[] = {
	SLICE_CASE("A IDR", &sps_a_cfg, &pps_a_cfg, slice_a_idr,
		   .nal_unit_type = V4L2_HEVC_NAL_IDR_W_RADL,
		   .nuh_temporal_id_plus1 = 1,
		   .slice_type = V4L2_HEVC_SLICE_TYPE_I,
		   .slice_qp_delta = 2,
		   .flags = V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_LUMA |
			    V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_CHROMA),
	SLICE_CASE("A P", &sps_a_cfg, &pps_a_cfg, slice_a_p,
		   .nal_unit_type = V4L2_HEVC_NAL_TRAIL_R,
		   .nuh_temporal_id_plus1 = 1,
		   .slice_type = V4L2_HEVC_SLICE_TYPE_P,
		   .slice_pic_order_cnt_lsb = 8,
		   .st_rps = {
			.num_negative_pics = 2,
			.used_by_curr_pic_s0 = 0x1,
			.delta_poc_s0_minus1 = { 0, 1 },
		   },
		   .five_minus_max_num_merge_cand = 2,
		   .slice_qp_delta = -3,
		   .flags = V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED |
			    V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_LUMA |
			    V4L2_HEVC_ENCODE_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED),
	SLICE_CASE("B B", &sps_b_cfg, &pps_b_cfg, slice_b_b,
		   .nal_unit_type = V4L2_HEVC_NAL_TRAIL_R,
		   .nuh_temporal_id_plus1 = 1,
		   .slice_type = V4L2_HEVC_SLICE_TYPE_B,
		   .slice_segment_address = 5,
		   .slice_pic_order_cnt_lsb = 3,
		   .st_rps = {
			.num_negative_pics = 1,
			.num_positive_pics = 1,
			.used_by_curr_pic_s0 = 0x1,
			.used_by_curr_pic_s1 = 0x1,
			.delta_poc_s0_minus1 = { 0 },
			.delta_poc_s1_minus1 = { 1 },
		   },
		   .num_long_term_pics = 1,
		   .lt = { { .poc_lsb_lt = 0, .delta_poc_msb_present = true,
			     .delta_poc_msb_cycle_lt = 1 } },
		   .num_ref_idx_l0_active_minus1 = 1,
		   .num_ref_idx_l1_active_minus1 = 1,
		   .collocated_ref_idx = 1,
		   .slice_cb_qp_offset = 1,
		   .slice_cr_qp_offset = -1,
		   .slice_beta_offset_div2 = 2,
		   .slice_tc_offset_div2 = -2,
		   .num_entry_point_offsets = ARRAY_SIZE(entry_points_b),
		   .entry_point_offset_minus1 = entry_points_b,
		   .flags = V4L2_HEVC_ENCODE_FLAG_NO_PIC_OUTPUT |
			    V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED |
			    V4L2_HEVC_ENCODE_FLAG_NUM_REF_IDX_ACTIVE_OVERRIDE |
			    V4L2_HEVC_ENCODE_FLAG_MVD_L1_ZERO |
			    V4L2_HEVC_ENCODE_FLAG_CABAC_INIT |
			    V4L2_HEVC_ENCODE_FLAG_DEBLOCKING_FILTER_OVERRIDE |
			    V4L2_HEVC_ENCODE_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED),
	SLICE_CASE("B dependent", &sps_b_cfg, &pps_b_cfg, slice_b_dep,
		   .nal_unit_type = V4L2_HEVC_NAL_TRAIL_R,
		   .nuh_temporal_id_plus1 = 1,
		   .slice_type = V4L2_HEVC_SLICE_TYPE_B,
		   .slice_segment_address = 10,
		   .flags = V4L2_HEVC_SLICE_HEADER_FLAG_DEPENDENT_SLICE_SEGMENT),
	SLICE_CASE("B CRA", &sps_b_cfg, &pps_b_cfg, slice_b_cra,
		   .nal_unit_type = V4L2_HEVC_NAL_CRA_NUT,
		   .nuh_temporal_id_plus1 = 1,
		   .slice_type = V4L2_HEVC_SLICE_TYPE_I,
		   .slice_qp_delta = -1,
		   .flags = V4L2_HEVC_ENCODE_FLAG_NO_OUTPUT_OF_PRIOR_PICS |
			    V4L2_HEVC_ENCODE_FLAG_DEBLOCKING_FILTER_OVERRIDE |
			    V4L2_HEVC_ENCODE_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED),
	SLICE_CASE("C TSA", &sps_c_cfg, &pps_c_cfg, slice_c_tsa,
		   .nal_unit_type = V4L2_HEVC_NAL_TSA_N,
		   .nuh_temporal_id_plus1 = 3,
		   .slice_type = V4L2_HEVC_SLICE_TYPE_P,
		   .slice_pic_order_cnt_lsb = 4095,
		   .st_rps = {
			.num_negative_pics = 1,
			.used_by_curr_pic_s0 = 0x1,
			.delta_poc_s0_minus1 = { 1 },
		   },
		   .five_minus_max_num_merge_cand = 1,
		   .slice_qp_delta = 5,
		   .num_entry_point_offsets = ARRAY_SIZE(entry_points_c),
		   .entry_point_offset_minus1 = entry_points_c),
};

static void hevc_rbsp_expect(struct kunit *test, struct v4l2_hevc_enc_rbsp *rbsp,
			     int ret, const u8 *expected, unsigned int size)
{
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, v4l2_hevc_enc_rbsp_bytes(rbsp), size);
	KUNIT_EXPECT_MEMEQ(test, rbsp->data, expected, size);
}

static u8 *hevc_rbsp_buffer(struct kunit *test, struct v4l2_hevc_enc_rbsp *rbsp)
{
	u8 *buf = kunit_kzalloc(test, BUF_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);
	v4l2_hevc_enc_rbsp_init(rbsp, buf, BUF_SIZE);

	return buf;
}

/* Exp-Golomb codes, and no emulation prevention outside NAL units. */
static void hevc_rbsp_exp_golomb(struct kunit *test)
{
	struct v4l2_hevc_enc_rbsp rbsp;
	unsigned int i;
	static const s32 se[] = { 1, -1, 2, -2, 0, 7, -7 };

	hevc_rbsp_buffer(test, &rbsp);

	for (i = 0; i < 9; i++)
		v4l2_hevc_enc_rbsp_ue(&rbsp, i);
	for (i = 0; i < ARRAY_SIZE(se); i++)
		v4l2_hevc_enc_rbsp_se(&rbsp, se[i]);
	v4l2_hevc_enc_rbsp_bits(&rbsp, 0xdeadbeef, 32);
	v4l2_hevc_enc_rbsp_ue(&rbsp, 0xfffffffe);
	v4l2_hevc_enc_rbsp_trailing_bits(&rbsp);

	hevc_rbsp_expect(test, &rbsp, v4l2_hevc_enc_rbsp_error(&rbsp),
			 exp_golomb, sizeof(exp_golomb));
}

struct hevc_rbsp_epb_case {
	const char *name;
	u8 in[8];
	unsigned int in_size;
	u8 out[12];
	unsigned int out_size;
};

static const struct hevc_rbsp_epb_case epb_cases[] = {
	{ "00 00 00", { 0, 0, 0 }, 3, { 0, 0, 3, 0 }, 4 },
	{ "00 00 01", { 0, 0, 1 }, 3, { 0, 0, 3, 1 }, 4 },
	{ "00 00 02", { 0, 0, 2 }, 3, { 0, 0, 3, 2 }, 4 },
	{ "00 00 03", { 0, 0, 3 }, 3, { 0, 0, 3, 3 }, 4 },
	{ "00 00 04", { 0, 0, 4 }, 3, { 0, 0, 4 }, 3 },
	{ "5 zeros", { 0, 0, 0, 0, 0 }, 5, { 0, 0, 3, 0, 0, 3, 0 }, 7 },
	{ "00 00 03 00", { 0, 0, 3, 0 }, 4, { 0, 0, 3, 3, 0 }, 5 },
	{ "00 01 00 00 00 01", { 0, 1, 0, 0, 0, 1 }, 6,
	  { 0, 1, 0, 0, 3, 0, 1 }, 7 },
	{ "00 00", { 0, 0 }, 2, { 0, 0 }, 2 },
};

/*
 * Emulation prevention inside a NAL unit payload, with the bytes written
 * as fields that do not line up with byte boundaries (3 + 5 + 16 bits).
 */
static void hevc_rbsp_epb(struct kunit *test)
{
	struct v4l2_hevc_enc_rbsp rbsp;
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(epb_cases); i++) {
		const struct hevc_rbsp_epb_case *c = &epb_cases[i];
		u8 out[2 + ARRAY_SIZE(c->out)];

		hevc_rbsp_buffer(test, &rbsp);

		/* TRAIL_N header: 0x00 0x01, not subject to prevention. */
		v4l2_hevc_enc_rbsp_nal_begin(&rbsp, false, V4L2_HEVC_NAL_TRAIL_N,
					     1);
		for (j = 0; j + 3 <= c->in_size; j += 3) {
			u32 v = c->in[j] << 16 | c->in[j + 1] << 8 | c->in[j + 2];

			v4l2_hevc_enc_rbsp_bits(&rbsp, v >> 21, 3);
			v4l2_hevc_enc_rbsp_bits(&rbsp, v >> 16, 5);
			v4l2_hevc_enc_rbsp_bits(&rbsp, v, 16);
		}
		for (; j < c->in_size; j++)
			v4l2_hevc_enc_rbsp_bits(&rbsp, c->in[j], 8);

		out[0] = 0x00;
		out[1] = 0x01;
		memcpy(&out[2], c->out, c->out_size);

		KUNIT_EXPECT_EQ_MSG(test, v4l2_hevc_enc_rbsp_error(&rbsp), 0,
				    "%s", c->name);
		KUNIT_EXPECT_EQ_MSG(test, v4l2_hevc_enc_rbsp_bytes(&rbsp),
				    c->out_size + 2, "%s", c->name);
		KUNIT_EXPECT_MEMEQ_MSG(test, rbsp.data, out, c->out_size + 2,
				       "%s", c->name);
	}
}

static void hevc_rbsp_errors(struct kunit *test)
{
	struct v4l2_hevc_enc_rbsp rbsp;
	struct v4l2_ctrl_hevc_sps sps = sps_a_cfg;
	u8 buf[8];

	/* A buffer that is too small. */
	v4l2_hevc_enc_rbsp_init(&rbsp, buf, sizeof(buf));
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_vps(&rbsp, true, &sps,
						     &video_a_cfg),
			-ENOSPC);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_bytes(&rbsp), sizeof(buf));

	/* The error is sticky. */
	v4l2_hevc_enc_rbsp_bits(&rbsp, 1, 1);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_error(&rbsp), -ENOSPC);

	/* Values that cannot be represented. */
	hevc_rbsp_buffer(test, &rbsp);
	v4l2_hevc_enc_rbsp_ue(&rbsp, U32_MAX);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_error(&rbsp), -EINVAL);

	hevc_rbsp_buffer(test, &rbsp);
	v4l2_hevc_enc_rbsp_se(&rbsp, S32_MIN);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_error(&rbsp), -EINVAL);

	/* Parameter sets the writer cannot express. */
	sps.num_short_term_ref_pic_sets = 1;
	hevc_rbsp_buffer(test, &rbsp);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_sps(&rbsp, true, &sps,
						     &video_a_cfg),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_bytes(&rbsp), 0);
}

struct hevc_rbsp_ps_case {
	const char *name;
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_hevc_sps_video *video;
	const struct v4l2_ctrl_hevc_pps *pps;
	const u8 *vps, *sps_exp, *pps_exp;
	unsigned int vps_size, sps_size, pps_size;
};

#define PS_CASE(n, x)							\
	{								\
		.name = n, .sps = &sps_##x##_cfg,			\
		.video = &video_##x##_cfg, .pps = &pps_##x##_cfg,	\
		.vps = vps_##x, .vps_size = sizeof(vps_##x),		\
		.sps_exp = sps_##x, .sps_size = sizeof(sps_##x),	\
		.pps_exp = pps_##x, .pps_size = sizeof(pps_##x),	\
	}

static const struct hevc_rbsp_ps_case ps_cases[] = {
	PS_CASE("A", a),
	PS_CASE("B", b),
	PS_CASE("C", c),
};

static void hevc_rbsp_ps_desc(const struct hevc_rbsp_ps_case *c, char *desc)
{
	strscpy(desc, c->name, KUNIT_PARAM_DESC_SIZE);
}

KUNIT_ARRAY_PARAM(hevc_rbsp_ps, ps_cases, hevc_rbsp_ps_desc);

static void hevc_rbsp_vps_test(struct kunit *test)
{
	const struct hevc_rbsp_ps_case *c = test->param_value;
	struct v4l2_hevc_enc_rbsp rbsp;
	int ret;

	hevc_rbsp_buffer(test, &rbsp);
	ret = v4l2_hevc_enc_rbsp_vps(&rbsp, true, c->sps, c->video);
	hevc_rbsp_expect(test, &rbsp, ret, c->vps, c->vps_size);
}

static void hevc_rbsp_sps_test(struct kunit *test)
{
	const struct hevc_rbsp_ps_case *c = test->param_value;
	struct v4l2_hevc_enc_rbsp rbsp;
	int ret;

	hevc_rbsp_buffer(test, &rbsp);
	ret = v4l2_hevc_enc_rbsp_sps(&rbsp, true, c->sps, c->video);
	hevc_rbsp_expect(test, &rbsp, ret, c->sps_exp, c->sps_size);
}

static void hevc_rbsp_pps_test(struct kunit *test)
{
	const struct hevc_rbsp_ps_case *c = test->param_value;
	struct v4l2_hevc_enc_rbsp rbsp;
	int ret;

	hevc_rbsp_buffer(test, &rbsp);
	ret = v4l2_hevc_enc_rbsp_pps(&rbsp, true, c->sps, c->pps);
	hevc_rbsp_expect(test, &rbsp, ret, c->pps_exp, c->pps_size);
}

static void hevc_rbsp_slice_desc(const struct hevc_rbsp_slice_case *c,
				 char *desc)
{
	strscpy(desc, c->name, KUNIT_PARAM_DESC_SIZE);
}

KUNIT_ARRAY_PARAM(hevc_rbsp_slice, slice_cases, hevc_rbsp_slice_desc);

static void hevc_rbsp_slice_test(struct kunit *test)
{
	const struct hevc_rbsp_slice_case *c = test->param_value;
	struct v4l2_hevc_enc_rbsp rbsp;
	int ret;

	hevc_rbsp_buffer(test, &rbsp);
	ret = v4l2_hevc_enc_rbsp_slice_header(&rbsp, true, c->sps, c->pps,
					      &c->sh);
	hevc_rbsp_expect(test, &rbsp, ret, c->expected, c->expected_size);
}

/* Several NAL units in a row, as at the start of an access unit. */
static void hevc_rbsp_access_unit(struct kunit *test)
{
	struct v4l2_hevc_enc_rbsp rbsp;
	unsigned int offset = 0;
	u8 *buf;
	int ret;

	buf = hevc_rbsp_buffer(test, &rbsp);

	ret = v4l2_hevc_enc_rbsp_aud(&rbsp, true, V4L2_HEVC_AUD_PIC_TYPE_PI, 2);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_MEMEQ(test, buf + offset, aud, sizeof(aud));
	offset += sizeof(aud);

	ret = v4l2_hevc_enc_rbsp_vps(&rbsp, true, &sps_c_cfg, &video_c_cfg);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_MEMEQ(test, buf + offset, vps_c, sizeof(vps_c));
	offset += sizeof(vps_c);

	ret = v4l2_hevc_enc_rbsp_sps(&rbsp, true, &sps_c_cfg, &video_c_cfg);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_MEMEQ(test, buf + offset, sps_c, sizeof(sps_c));
	offset += sizeof(sps_c);

	ret = v4l2_hevc_enc_rbsp_pps(&rbsp, true, &sps_c_cfg, &pps_c_cfg);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_MEMEQ(test, buf + offset, pps_c, sizeof(pps_c));
	offset += sizeof(pps_c);

	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_bytes(&rbsp), offset);

	/* Without start codes, each NAL unit loses its first four bytes. */
	v4l2_hevc_enc_rbsp_init(&rbsp, buf, BUF_SIZE);
	ret = v4l2_hevc_enc_rbsp_pps(&rbsp, false, &sps_c_cfg, &pps_c_cfg);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, v4l2_hevc_enc_rbsp_bytes(&rbsp),
			sizeof(pps_c) - 4);
	KUNIT_EXPECT_MEMEQ(test, buf, pps_c + 4, sizeof(pps_c) - 4);
}

static struct kunit_case hevc_rbsp_test_cases[] = {
	KUNIT_CASE(hevc_rbsp_exp_golomb),
	KUNIT_CASE(hevc_rbsp_epb),
	KUNIT_CASE(hevc_rbsp_errors),
	KUNIT_CASE_PARAM(hevc_rbsp_vps_test, hevc_rbsp_ps_gen_params),
	KUNIT_CASE_PARAM(hevc_rbsp_sps_test, hevc_rbsp_ps_gen_params),
	KUNIT_CASE_PARAM(hevc_rbsp_pps_test, hevc_rbsp_ps_gen_params),
	KUNIT_CASE_PARAM(hevc_rbsp_slice_test, hevc_rbsp_slice_gen_params),
	KUNIT_CASE(hevc_rbsp_access_unit),
	{}
};

static struct kunit_suite hevc_rbsp_test_suite = {
	.name = "v4l2-hevc-enc-rbsp",
	.test_cases = hevc_rbsp_test_cases,
};

kunit_test_suite(hevc_rbsp_test_suite);

MODULE_DESCRIPTION("KUnit tests for the V4L2 HEVC encode RBSP writer");
MODULE_LICENSE("GPL");
