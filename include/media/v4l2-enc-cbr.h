/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * V4L2 stateless encoder constant bitrate rate control
 *
 * A codec independent frame-level rate control for the constant bitrate
 * mode of the stateless encode cores.
 */

#ifndef _MEDIA_V4L2_ENC_CBR_H
#define _MEDIA_V4L2_ENC_CBR_H

#include <linux/types.h>
#include <linux/videodev2.h>

/* Pictures of the peak bitrate window: one second, up to 128 pictures. */
#define V4L2_ENC_CBR_PEAK_PICS	128U

/**
 * struct v4l2_enc_cbr - constant bitrate rate control state
 * @qp_delta_intra: QP of the I pictures below the QP of the P pictures
 * @bitrate: bitrate the state was set up for, 0 before the first picture
 * @nominal: nominal picture size in bits (bitrate times frame interval)
 * @qp_q8: QP of the P pictures in 1/256 units, -1 before the first picture
 * @qp_intra_ref_q8: @qp_q8 when the previous I picture was coded, -1 before
 *		     the first I picture
 * @qp_intra_add_q8: QP increase of the I pictures after I pictures larger
 *		     than their share of the GOP at the largest I/P ratio
 * @qp_pic: QP of the current picture
 * @qp_reenc: QP increase of the second pass of the current picture, 0 if
 *	      it was coded once
 * @ratio: I/P picture size ratio in 1/16 units
 * @gop: measured IDR period in pictures, 0 until two I pictures were seen
 * @since_i: pictures coded since the last I picture
 * @seen_i: an I picture was coded
 * @intra: the current picture is an I picture
 * @p_bits: running mean of the P picture sizes
 * @i_bits: size of the last I picture
 * @buffer_bits: accumulated difference between the coded and nominal sizes
 * @window_bits: sum of the coded sizes over about three seconds (each size
 *		 decays by 1/(3 * fps) per picture)
 * @plan_bits: planned size of the current picture
 * @target_bits: size target of the current picture
 * @bitrate_peak: peak bitrate in bits per second, 0 for a constant bitrate
 * @qp_peak_add: QP increase of the current picture for the peak bitrate
 * @i_qp: QP of the last I picture
 * @p_last_bits: size of the last P picture
 * @p_qp: QP of the last P picture
 * @peak_bits: sizes of the last pictures, for the peak bitrate
 * @peak_index: next entry of @peak_bits
 * @peak_count: valid entries of @peak_bits
 *
 * Only @qp_delta_intra is set by the user, after v4l2_enc_cbr_reset(), and
 * @bitrate_peak by v4l2_enc_cbr_set_peak().
 * @target_bits is valid from v4l2_enc_cbr_begin() of a picture until its
 * v4l2_enc_cbr_end().
 */
struct v4l2_enc_cbr {
	int qp_delta_intra;

	u32 bitrate;
	u64 nominal;
	int qp_q8;
	int qp_intra_ref_q8;
	int qp_intra_add_q8;
	int qp_pic;
	int qp_reenc;
	unsigned int ratio;
	unsigned int gop;
	unsigned int since_i;
	bool seen_i;
	bool intra;
	u64 p_bits;
	u64 i_bits;
	s64 buffer_bits;
	u64 window_bits;
	u64 plan_bits;
	u64 target_bits;

	u32 bitrate_peak;
	int qp_peak_add;
	int i_qp;
	u64 p_last_bits;
	int p_qp;
	u32 peak_bits[V4L2_ENC_CBR_PEAK_PICS];
	unsigned int peak_index;
	unsigned int peak_count;
};

void v4l2_enc_cbr_reset(struct v4l2_enc_cbr *cbr, int qp_delta_intra);
void v4l2_enc_cbr_set_peak(struct v4l2_enc_cbr *cbr, u32 bitrate_peak);
int v4l2_enc_cbr_begin(struct v4l2_enc_cbr *cbr, u32 bitrate,
		       const struct v4l2_fract *timeperframe,
		       unsigned int samples, bool intra, int qp_min,
		       int qp_max);
int v4l2_enc_cbr_qp(const struct v4l2_enc_cbr *cbr);
bool v4l2_enc_cbr_reencode(struct v4l2_enc_cbr *cbr, u64 bits,
			   const struct v4l2_fract *timeperframe, int qp,
			   int qp_max);
void v4l2_enc_cbr_end(struct v4l2_enc_cbr *cbr, u64 bits,
		      const struct v4l2_fract *timeperframe, int qp_min,
		      int qp_max);

#endif
