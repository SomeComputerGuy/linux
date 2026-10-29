/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Helper functions and ITU-T Rec. H.265 syntax definitions for HEVC stateless
 * codecs.
 */

#ifndef _MEDIA_V4L2_HEVC_H
#define _MEDIA_V4L2_HEVC_H

#include <linux/minmax.h>
#include <media/v4l2-ctrls.h>

/**
 * v4l2_hevc_pps_num_tile_columns - number of HEVC tile columns, bounded
 * @pps: the V4L2 HEVC PPS control
 *
 * Return the number of tile columns (num_tile_columns_minus1 + 1) clamped to
 * the capacity of column_width_minus1[]. The control validation already
 * rejects out-of-range counts; this keeps the consuming drivers bounded too.
 */
static inline unsigned int
v4l2_hevc_pps_num_tile_columns(const struct v4l2_ctrl_hevc_pps *pps)
{
	return min_t(unsigned int, pps->num_tile_columns_minus1 + 1,
		     ARRAY_SIZE(pps->column_width_minus1));
}

/**
 * v4l2_hevc_pps_num_tile_rows - number of HEVC tile rows, bounded
 * @pps: the V4L2 HEVC PPS control
 *
 * Return the number of tile rows (num_tile_rows_minus1 + 1) clamped to the
 * capacity of row_height_minus1[].
 */
static inline unsigned int
v4l2_hevc_pps_num_tile_rows(const struct v4l2_ctrl_hevc_pps *pps)
{
	return min_t(unsigned int, pps->num_tile_rows_minus1 + 1,
		     ARRAY_SIZE(pps->row_height_minus1));
}

/* ITU-T Rec. H.265 Table 7-1: NAL unit type codes */
#define V4L2_HEVC_NAL_TRAIL_N		0
#define V4L2_HEVC_NAL_TRAIL_R		1
#define V4L2_HEVC_NAL_TSA_N		2
#define V4L2_HEVC_NAL_TSA_R		3
#define V4L2_HEVC_NAL_STSA_N		4
#define V4L2_HEVC_NAL_STSA_R		5
#define V4L2_HEVC_NAL_RADL_N		6
#define V4L2_HEVC_NAL_RADL_R		7
#define V4L2_HEVC_NAL_RASL_N		8
#define V4L2_HEVC_NAL_RASL_R		9
#define V4L2_HEVC_NAL_RSV_VCL_N14	14
#define V4L2_HEVC_NAL_BLA_W_LP		16
#define V4L2_HEVC_NAL_BLA_W_RADL	17
#define V4L2_HEVC_NAL_BLA_N_LP		18
#define V4L2_HEVC_NAL_IDR_W_RADL	19
#define V4L2_HEVC_NAL_IDR_N_LP		20
#define V4L2_HEVC_NAL_CRA_NUT		21
#define V4L2_HEVC_NAL_RSV_IRAP_VCL23	23
#define V4L2_HEVC_NAL_VPS		32
#define V4L2_HEVC_NAL_SPS		33
#define V4L2_HEVC_NAL_PPS		34
#define V4L2_HEVC_NAL_AUD		35
#define V4L2_HEVC_NAL_EOS		36
#define V4L2_HEVC_NAL_EOB		37
#define V4L2_HEVC_NAL_FD		38
#define V4L2_HEVC_NAL_PREFIX_SEI	39
#define V4L2_HEVC_NAL_SUFFIX_SEI	40

/* ITU-T Rec. H.265 Annex A: general_profile_idc */
#define V4L2_HEVC_PROFILE_IDC_MAIN		1
#define V4L2_HEVC_PROFILE_IDC_MAIN_10		2
#define V4L2_HEVC_PROFILE_IDC_MAIN_STILL_PICTURE	3
#define V4L2_HEVC_PROFILE_IDC_RANGE_EXTENSIONS	4

/* ITU-T Rec. H.265 Table 7-2: pic_type in access unit delimiters */
#define V4L2_HEVC_AUD_PIC_TYPE_I	0
#define V4L2_HEVC_AUD_PIC_TYPE_PI	1
#define V4L2_HEVC_AUD_PIC_TYPE_BPI	2

/* ITU-T Rec. H.265 Table E.1: aspect_ratio_idc */
#define V4L2_HEVC_VUI_ASPECT_RATIO_IDC_EXTENDED	255

/* ITU-T Rec. H.265 Table E.2: video_format */
#define V4L2_HEVC_VUI_VIDEO_FORMAT_UNSPECIFIED	5

/* ITU-T Rec. H.265 Tables E.3 to E.5 (ITU-T H.273 code points) */
#define V4L2_HEVC_VUI_COLOUR_BT709		1
#define V4L2_HEVC_VUI_COLOUR_UNSPECIFIED	2
#define V4L2_HEVC_VUI_COLOUR_BT470_SYSTEM_M	4
#define V4L2_HEVC_VUI_COLOUR_BT470_SYSTEM_BG	5
#define V4L2_HEVC_VUI_COLOUR_SMPTE170M		6
#define V4L2_HEVC_VUI_COLOUR_SMPTE240M		7
#define V4L2_HEVC_VUI_COLOUR_BT2020		9
#define V4L2_HEVC_VUI_COLOUR_SMPTE431		11

#define V4L2_HEVC_VUI_TRANSFER_BT709		1
#define V4L2_HEVC_VUI_TRANSFER_UNSPECIFIED	2
#define V4L2_HEVC_VUI_TRANSFER_SMPTE170M	6
#define V4L2_HEVC_VUI_TRANSFER_SMPTE240M	7
#define V4L2_HEVC_VUI_TRANSFER_LINEAR		8
#define V4L2_HEVC_VUI_TRANSFER_SRGB		13
#define V4L2_HEVC_VUI_TRANSFER_SMPTE2084	16

#define V4L2_HEVC_VUI_MATRIX_BT709		1
#define V4L2_HEVC_VUI_MATRIX_UNSPECIFIED	2
#define V4L2_HEVC_VUI_MATRIX_BT470_SYSTEM_BG	5
#define V4L2_HEVC_VUI_MATRIX_SMPTE170M		6
#define V4L2_HEVC_VUI_MATRIX_SMPTE240M		7
#define V4L2_HEVC_VUI_MATRIX_BT2020_NCL		9
#define V4L2_HEVC_VUI_MATRIX_BT2020_CL		10

/* Maximum number of temporal sub-layers (sps_max_sub_layers_minus1 + 1) */
#define V4L2_HEVC_MAX_SUB_LAYERS	7

/* Maximum number of pictures in a short-term RPS (7.4.8) */
#define V4L2_HEVC_MAX_ST_RPS_PICS	16

/* Maximum number of long-term pictures signalled in a slice header */
#define V4L2_HEVC_MAX_LT_PICS		16

static inline bool v4l2_hevc_nal_is_irap(unsigned int nal_unit_type)
{
	return nal_unit_type >= V4L2_HEVC_NAL_BLA_W_LP &&
	       nal_unit_type <= V4L2_HEVC_NAL_RSV_IRAP_VCL23;
}

static inline bool v4l2_hevc_nal_is_idr(unsigned int nal_unit_type)
{
	return nal_unit_type == V4L2_HEVC_NAL_IDR_W_RADL ||
	       nal_unit_type == V4L2_HEVC_NAL_IDR_N_LP;
}

static inline bool v4l2_hevc_nal_is_bla(unsigned int nal_unit_type)
{
	return nal_unit_type >= V4L2_HEVC_NAL_BLA_W_LP &&
	       nal_unit_type <= V4L2_HEVC_NAL_BLA_N_LP;
}

/*
 * Sub-layer non-reference picture (3.132): even nal_unit_type values in the
 * VCL range below RSV_VCL_N14.
 */
static inline bool v4l2_hevc_nal_is_sub_layer_non_ref(unsigned int nal_unit_type)
{
	return nal_unit_type <= V4L2_HEVC_NAL_RSV_VCL_N14 &&
	       !(nal_unit_type & 1);
}

static inline bool v4l2_hevc_nal_is_leading(unsigned int nal_unit_type)
{
	return nal_unit_type >= V4L2_HEVC_NAL_RADL_N &&
	       nal_unit_type <= V4L2_HEVC_NAL_RASL_R;
}

static inline char v4l2_hevc_slice_type_char(unsigned char slice_type)
{
	switch (slice_type) {
	case V4L2_HEVC_SLICE_TYPE_B:
		return 'B';
	case V4L2_HEVC_SLICE_TYPE_P:
		return 'P';
	case V4L2_HEVC_SLICE_TYPE_I:
		return 'I';
	default:
		return 'X';
	}
}

#endif /* _MEDIA_V4L2_HEVC_H */
