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

/**
 * struct v4l2_enc_cbr - constant bitrate rate control state
 * @qp_delta_intra: QP of the I pictures below the QP of the P pictures
 * @bitrate: bitrate the state was set up for, 0 before the first picture
 * @nominal: nominal picture size in bits (bitrate times frame interval)
 * @qp_q8: QP of the P pictures in 1/256 units, -1 before the first picture
 * @ratio: I/P picture size ratio in 1/16 units
 * @gop: measured IDR period in pictures, 0 until two I pictures were seen
 * @since_i: pictures coded since the last I picture
 * @seen_i: an I picture was coded
 * @intra: the current picture is an I picture
 * @p_bits: running mean of the P picture sizes
 * @i_bits: size of the last I picture
 * @buffer_bits: accumulated difference between the coded and nominal sizes
 * @plan_bits: planned size of the current picture
 * @target_bits: size target of the current picture
 *
 * Only @qp_delta_intra is set by the user, after v4l2_enc_cbr_reset().
 * @target_bits is valid from v4l2_enc_cbr_begin() of a picture until its
 * v4l2_enc_cbr_end().
 */
struct v4l2_enc_cbr {
	int qp_delta_intra;

	u32 bitrate;
	u64 nominal;
	int qp_q8;
	unsigned int ratio;
	unsigned int gop;
	unsigned int since_i;
	bool seen_i;
	bool intra;
	u64 p_bits;
	u64 i_bits;
	s64 buffer_bits;
	u64 plan_bits;
	u64 target_bits;
};

void v4l2_enc_cbr_reset(struct v4l2_enc_cbr *cbr, int qp_delta_intra);
int v4l2_enc_cbr_begin(struct v4l2_enc_cbr *cbr, u32 bitrate,
		       const struct v4l2_fract *timeperframe,
		       unsigned int samples, bool intra, int qp_min,
		       int qp_max);
int v4l2_enc_cbr_qp(const struct v4l2_enc_cbr *cbr);
void v4l2_enc_cbr_end(struct v4l2_enc_cbr *cbr, u64 bits,
		      const struct v4l2_fract *timeperframe, int qp_min,
		      int qp_max);

#endif
