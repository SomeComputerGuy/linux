/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * V4L2 HEVC Encode Core
 */

#ifndef _MEDIA_V4L2_HEVC_ENC_H
#define _MEDIA_V4L2_HEVC_ENC_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <media/v4l2-hevc-enc-rbsp.h>

struct v4l2_ctrl_handler;
struct vb2_v4l2_buffer;

/* The hardware can encode P slices. */
#define V4L2_HEVC_ENC_FLAG_INTER_PRED		BIT(0)
/* The hardware can encode B slices. */
#define V4L2_HEVC_ENC_FLAG_INTER_BIPRED		BIT(1)
/* The hardware writes these NAL units or headers itself. */
#define V4L2_HEVC_ENC_FLAG_HW_AUD		BIT(2)
#define V4L2_HEVC_ENC_FLAG_HW_VPS		BIT(3)
#define V4L2_HEVC_ENC_FLAG_HW_SPS		BIT(4)
#define V4L2_HEVC_ENC_FLAG_HW_PPS		BIT(5)
#define V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER	BIT(6)
/*
 * The NAL unit headers the hardware writes carry nuh_temporal_id_plus1.
 * Without it, the core patches them after the encode run.
 */
#define V4L2_HEVC_ENC_FLAG_HW_TEMPORAL_ID	BIT(7)
/* The hardware supports the default scaling lists. */
#define V4L2_HEVC_ENC_FLAG_SCALING_LIST		BIT(8)
/* The hardware supports tiles and wavefront parallel processing. */
#define V4L2_HEVC_ENC_FLAG_TILES		BIT(9)
#define V4L2_HEVC_ENC_FLAG_ENTROPY_CODING_SYNC	BIT(10)

struct v4l2_hevc_enc;

/**
 * struct v4l2_hevc_enc_rps_limits - reference picture set size limits
 *
 * Hardware that writes slice headers from registers can only express a
 * limited reference picture set.
 *
 * @num_negative_pics: maximum num_negative_pics
 * @num_positive_pics: maximum num_positive_pics
 * @num_long_term_pics: maximum num_long_term_pics
 */
struct v4l2_hevc_enc_rps_limits {
	u8 num_negative_pics;
	u8 num_positive_pics;
	u8 num_long_term_pics;
};

/**
 * struct v4l2_hevc_enc_rec_buffer - reconstruction buffer
 *
 * @private_data: driver data for the buffer (e.g. its DMA addresses)
 * @allocated: whether the driver allocated the buffer
 */
struct v4l2_hevc_enc_rec_buffer {
	void *private_data;
	bool allocated;
};

/**
 * struct v4l2_hevc_enc_dpb_entry - reconstructed picture kept as reference
 *
 * @buffer: the reconstruction buffer of the slot
 * @timestamp: timestamp of the OUTPUT buffer the picture was encoded from
 * @pic_order_cnt_val: picture order count of the picture
 * @active: whether the slot holds a reference picture
 * @long_term: whether the picture is marked as used for long-term reference
 */
struct v4l2_hevc_enc_dpb_entry {
	struct v4l2_hevc_enc_rec_buffer buffer;
	u64 timestamp;
	s32 pic_order_cnt_val;
	bool active;
	bool long_term;
};

/**
 * struct v4l2_hevc_enc_ref - reference picture state
 *
 * @buffer_current: reconstruction buffer of the picture being encoded
 * @dpb: reference picture slots
 * @slots_count: number of allocated slots in @dpb
 * @rps_dpb: for each entry of the encode params reference picture set,
 *	     the index of its slot in @dpb
 * @l0: reference picture list 0, as indices in @dpb
 * @l0_count: number of entries in @l0 (num_ref_idx_l0_active_minus1 + 1)
 * @l1: reference picture list 1, as indices in @dpb
 * @l1_count: number of entries in @l1
 * @collocated: index in @dpb of the collocated picture, when temporal
 *		motion vector prediction is enabled
 * @prev_tid0_poc: picture order count of the previous TemporalId 0 picture
 *		   (prevTid0Pic, 8.3.1)
 */
struct v4l2_hevc_enc_ref {
	struct v4l2_hevc_enc_rec_buffer buffer_current;
	struct v4l2_hevc_enc_dpb_entry dpb[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	unsigned int slots_count;

	u8 rps_dpb[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u8 l0[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	unsigned int l0_count;
	u8 l1[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	unsigned int l1_count;
	int collocated;

	s32 prev_tid0_poc;
};

/**
 * struct v4l2_hevc_enc_state - parameters of an encode run
 *
 * @sps: the sequence parameter set, after accommodation
 * @sps_video: the VPS/SPS fields not in @sps
 * @pps: the picture parameter set, after accommodation
 * @encode: the encode parameters, after accommodation
 * @timeperframe: the frame interval
 * @ctb_log2_size: CtbLog2SizeY
 * @width_ctbs: PicWidthInCtbsY
 * @height_ctbs: PicHeightInCtbsY
 * @frame_rc_enable: V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE
 * @qp_min: V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP
 * @qp_max: V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP
 * @qp_i: V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP
 * @qp_p: V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP
 * @qp_b: V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP
 * @bitrate_mode: V4L2_CID_MPEG_VIDEO_BITRATE_MODE
 * @quality: V4L2_CID_MPEG_VIDEO_CONSTANT_QUALITY
 * @quality_min: minimum of V4L2_CID_MPEG_VIDEO_CONSTANT_QUALITY
 * @quality_max: maximum of V4L2_CID_MPEG_VIDEO_CONSTANT_QUALITY
 * @bitrate: V4L2_CID_MPEG_VIDEO_BITRATE
 * @bitrate_peak: V4L2_CID_MPEG_VIDEO_BITRATE_PEAK
 */
struct v4l2_hevc_enc_state {
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_hevc_sps_video sps_video;
	struct v4l2_ctrl_hevc_pps pps;
	struct v4l2_ctrl_hevc_encode_params encode;

	struct v4l2_fract timeperframe;

	unsigned int ctb_log2_size;
	unsigned int width_ctbs;
	unsigned int height_ctbs;

	bool frame_rc_enable;
	int qp_min;
	int qp_max;
	int qp_i;
	int qp_p;
	int qp_b;

	int bitrate_mode;
	unsigned int quality;
	unsigned int quality_min;
	unsigned int quality_max;
	unsigned int bitrate;
	unsigned int bitrate_peak;
};

/**
 * struct v4l2_hevc_enc_ops - driver callbacks
 *
 * @state_constrain: adjust the state of the next encode run to what the
 *		     hardware supports. Called after the core accommodated
 *		     the controls.
 * @rec_buffer_alloc: allocate a reconstruction buffer (with whatever side
 *		      buffers the hardware keeps with a reference picture)
 * @rec_buffer_free: free a reconstruction buffer
 */
struct v4l2_hevc_enc_ops {
	int (*state_constrain)(struct v4l2_hevc_enc *enc,
			       struct v4l2_hevc_enc_state *state);
	int (*rec_buffer_alloc)(struct v4l2_hevc_enc *enc,
				struct v4l2_hevc_enc_rec_buffer *buffer);
	void (*rec_buffer_free)(struct v4l2_hevc_enc *enc,
				struct v4l2_hevc_enc_rec_buffer *buffer);
};

/**
 * struct v4l2_hevc_enc_rc_ops - driver rate control
 *
 * The core selects the QP itself for the direct QP and constant quality
 * modes. These callbacks provide the constant and variable bitrate modes.
 *
 * @estimate: return the QP of the next picture in @qp
 * @complete: account for a completed picture
 */
struct v4l2_hevc_enc_rc_ops {
	int (*estimate)(struct v4l2_hevc_enc *enc,
			const struct v4l2_hevc_enc_state *state, int *qp);
	void (*complete)(struct v4l2_hevc_enc *enc,
			 const struct v4l2_hevc_enc_state *state,
			 unsigned long bytesused,
			 const struct v4l2_stateless_encode_stats *stats);
};

/**
 * struct v4l2_hevc_enc - HEVC stateless encode core context
 *
 * Fields up to @flags are set by the driver before v4l2_hevc_enc_init().
 * The driver reads @state_active, @ref, @slice, @qp and @header_size after
 * v4l2_hevc_enc_step() to program the hardware.
 *
 * @ops: driver callbacks
 * @rc_ops: driver rate control, optional
 * @private_data: driver data
 * @format: CAPTURE format, if single-planar
 * @format_mplane: CAPTURE format, if multi-planar
 * @crop: visible rectangle within the coded size, optional
 * @timeperframe: frame interval
 * @ctrl_handler: control handler with the stateless HEVC controls
 * @ref_slots_count_init: number of reference slots to allocate at init,
 *			  0 for the default
 * @max_l0_refs: maximum number of entries in reference picture list 0
 *		 the hardware supports, 0 for 1
 * @max_l1_refs: maximum number of entries in reference picture list 1
 *		 the hardware supports, 0 for 1
 * @rps_limits: reference picture set size limits of the hardware, or NULL
 * @flags: see V4L2_HEVC_ENC_FLAG_{}
 * @state_active: state of the current encode run
 * @state_next: state being prepared
 * @state_serial: number of pictures encoded since init
 * @ref: reference picture state
 * @rbsp: NAL unit writer
 * @slice: slice segment header of the current picture
 * @qp: SliceQpY of the current picture
 * @header_size: number of bytes the core wrote at the start of the
 *		 CAPTURE buffer. The hardware writes from that offset.
 * @parameter_sets_pending: parameter sets were written for a picture that
 *			    did not complete yet
 * @feedback: feedback of the current picture
 */
struct v4l2_hevc_enc {
	const struct v4l2_hevc_enc_ops *ops;
	const struct v4l2_hevc_enc_rc_ops *rc_ops;
	void *private_data;

	struct v4l2_pix_format *format;
	struct v4l2_pix_format_mplane *format_mplane;
	const struct v4l2_rect *crop;
	struct v4l2_fract *timeperframe;
	struct v4l2_ctrl_handler *ctrl_handler;
	unsigned int ref_slots_count_init;
	unsigned int max_l0_refs;
	unsigned int max_l1_refs;
	const struct v4l2_hevc_enc_rps_limits *rps_limits;
	unsigned int flags;

	struct v4l2_hevc_enc_state state_active;
	struct v4l2_hevc_enc_state state_next;
	unsigned int state_serial;

	struct v4l2_hevc_enc_ref ref;
	struct v4l2_hevc_enc_rbsp rbsp;
	struct v4l2_hevc_slice_header slice;
	int qp;
	unsigned int header_size;
	bool parameter_sets_pending;

	struct v4l2_ctrl_hevc_encode_feedback feedback;
};

int v4l2_hevc_enc_init(struct v4l2_hevc_enc *enc);
void v4l2_hevc_enc_exit(struct v4l2_hevc_enc *enc);
int v4l2_hevc_enc_step(struct v4l2_hevc_enc *enc,
		       struct vb2_v4l2_buffer *buffer);
int v4l2_hevc_enc_complete(struct v4l2_hevc_enc *enc,
			   struct vb2_v4l2_buffer *buffer,
			   const struct v4l2_stateless_encode_stats *stats);

static inline struct v4l2_hevc_enc_rec_buffer *
v4l2_hevc_enc_ref_buffer(struct v4l2_hevc_enc *enc, unsigned int dpb_index)
{
	return &enc->ref.dpb[dpb_index].buffer;
}

#endif
