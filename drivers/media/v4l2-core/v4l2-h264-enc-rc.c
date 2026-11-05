// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 H.264 Encode Rate Control
 *
 * Copyright (C) 2025-2026 Paul Kocialkowski <paulk@sys-base.io>
 */

#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <media/v4l2-h264-enc.h>

int v4l2_h264_enc_rc_init(struct v4l2_h264_enc_rc *rc)
{
	rc->qp = 0;
	rc->mode = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_h264_enc_rc_init);

void v4l2_h264_enc_rc_exit(struct v4l2_h264_enc_rc *rc)
{
	if (rc->mode) {
		v4l2_h264_enc_rc_op(rc, exit);
		v4l2_h264_enc_rc_mode_op(rc, exit);
		rc->mode = NULL;
	}
}
EXPORT_SYMBOL_GPL(v4l2_h264_enc_rc_exit);

int v4l2_h264_enc_rc_stats_collect(struct v4l2_h264_enc_rc *rc,
				   unsigned long bytesused)
{
	struct v4l2_h264_enc_state *state = rc->state;
	struct v4l2_ctrl_h264_encode_params *encode = &state->encode;
	struct v4l2_fract *timeperframe = &state->timeperframe;
	struct v4l2_h264_enc_rc_stats *stats = &rc->stats;
	struct v4l2_h264_enc_rc_stats_type *type;
	unsigned long bitrate;
	long size_error;
	long size_drift;
	long bitrate_error;
	long bitrate_drift;
	unsigned int index;

	/* Collect individual and total frame sizes. */

	index = stats->index;

	if (stats->count == V4L2_H264_ENC_RC_STATS_SIZE_COUNT)
		stats->size_total -= stats->size[index];

	stats->slice_type[index] = encode->slice_type;
	stats->size[index] = bytesused;
	stats->size_total += bytesused;

	if (stats->count < V4L2_H264_ENC_RC_STATS_SIZE_COUNT)
		stats->count++;

	stats->index = (index + 1) % V4L2_H264_ENC_RC_STATS_SIZE_COUNT;

	bitrate = 8 * stats->size_total * timeperframe->denominator /
		  stats->count / timeperframe->numerator;

	size_error = (long)bytesused - (long)rc->size;
	size_drift = 100 * size_error / (long)rc->size;

	bitrate_error = (long)bitrate - (long)state->bitrate;
	bitrate_drift = 100 * bitrate_error / (long)state->bitrate;

	pr_debug("+ v4l2-h264-rc: stats");
	pr_debug("  size: %lu B, target: %lu B, error: %ld B, drift: %ld pc",
		 bytesused, rc->size, size_error, size_drift);
	pr_debug("  bitrate: %lu b/s, target: %u b/s, error: %ld b/s, drift: %ld pc",
		 bitrate, state->bitrate, bitrate_error, bitrate_drift);

	/* Collect per-type size and qp information. */

	if (encode->slice_type == V4L2_H264_SLICE_TYPE_I)
		type = &stats->intra;
	else if (encode->slice_type == V4L2_H264_SLICE_TYPE_P)
		type = &stats->pred;
	else if (encode->slice_type == V4L2_H264_SLICE_TYPE_B)
		type = &stats->bipred;
	else
		return -EINVAL;

	index = type->index;

	type->target[index] = rc->size;
	type->size[index] = bytesused;
	type->qp[index] = rc->qp;

	if (type->count < V4L2_H264_ENC_RC_STATS_TYPE_COUNT)
		type->count++;

	type->index = (index + 1) % V4L2_H264_ENC_RC_STATS_TYPE_COUNT;

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_h264_enc_rc_stats_collect);

static int direct_estimate(struct v4l2_h264_enc_rc *rc, unsigned int *qp)
{
	struct v4l2_h264_enc_state *state = rc->state;
	struct v4l2_ctrl_h264_encode_params *encode = &state->encode;

	if (encode->slice_type == V4L2_H264_SLICE_TYPE_I)
		*qp = state->qp_i;
	else if (encode->slice_type == V4L2_H264_SLICE_TYPE_P)
		*qp = state->qp_p;
	else if (encode->slice_type == V4L2_H264_SLICE_TYPE_B)
		*qp = state->qp_b;
	else
		return -EINVAL;

	return 0;
}

static const struct v4l2_h264_enc_rc_mode direct_mode = {
	.name		= "direct qp",
	.type		= V4L2_H264_ENC_RC_TYPE_DIRECT,
	.estimate	= direct_estimate,
};

static int cq_estimate(struct v4l2_h264_enc_rc *rc, unsigned int *qp)
{
	struct v4l2_h264_enc_state *state = rc->state;
	struct v4l2_ctrl_h264_encode_params *encode = &state->encode;
	/* TODO: Use some reasonable PSNR to QP mapping. */
	unsigned int qp_range_intra[2] = { 12, 44 };
	unsigned int qp_range_inter[2] = { 16, 48 };
	unsigned int qp_min, qp_max;
	unsigned int qp_diff_range;
	unsigned int quality_diff;
	unsigned int quality_diff_range;

	if (encode->slice_type == V4L2_H264_SLICE_TYPE_I) {
		qp_min = qp_range_intra[0];
		qp_max = qp_range_intra[1];
	} else {
		qp_min = qp_range_inter[0];
		qp_max = qp_range_inter[1];
	}

	qp_diff_range = qp_max - qp_min;

	quality_diff = state->quality - state->quality_min;
	quality_diff_range = state->quality_max - state->quality_min;

	*qp = qp_min + qp_diff_range * quality_diff / quality_diff_range;

	return 0;
}

static const struct v4l2_h264_enc_rc_mode cq_mode = {
	.name		= "constant quality",
	.type		= V4L2_H264_ENC_RC_TYPE_CQ,
	.estimate	= cq_estimate,
};

/*
 * Constant bitrate mode, with the codec independent rate control of
 * v4l2-enc-cbr: I pictures 2 QP below the P pictures (the default of the
 * Rockchip MPP library for H.264).
 */
#define CBR_QP_DELTA_INTRA	2

static int cbr_init(struct v4l2_h264_enc_rc *rc)
{
	v4l2_enc_cbr_reset(&rc->cbr, CBR_QP_DELTA_INTRA);

	return 0;
}

static int cbr_measure(struct v4l2_h264_enc_rc *rc, unsigned long *size)
{
	struct v4l2_h264_enc_state *state = rc->state;
	bool intra = state->encode.slice_type == V4L2_H264_SLICE_TYPE_I;
	unsigned int samples = state->width_mbs * state->height_mbs * 256;
	int ret;

	ret = v4l2_enc_cbr_begin(&rc->cbr, state->bitrate,
				 &state->timeperframe, samples, intra,
				 state->qp_min, state->qp_max);
	if (ret)
		return ret;

	*size = max_t(unsigned long, rc->cbr.target_bits / 8, 1);

	return 0;
}

static int cbr_estimate(struct v4l2_h264_enc_rc *rc, unsigned int *qp)
{
	*qp = clamp(v4l2_enc_cbr_qp(&rc->cbr), 0, 51);

	return 0;
}

static int cbr_complete(struct v4l2_h264_enc_rc *rc, unsigned long bytesused)
{
	struct v4l2_h264_enc_state *state = rc->state;

	v4l2_enc_cbr_end(&rc->cbr, (u64)bytesused * 8, &state->timeperframe,
			 state->qp_min, state->qp_max);

	return v4l2_h264_enc_rc_stats_collect(rc, bytesused);
}

static const struct v4l2_h264_enc_rc_mode cbr_mode = {
	.name		= "constant bitrate",
	.type		= V4L2_H264_ENC_RC_TYPE_CBR,
	.init		= cbr_init,
	.measure	= cbr_measure,
	.estimate	= cbr_estimate,
	.complete	= cbr_complete,
};

static const struct v4l2_h264_enc_rc_mode *modes[] = {
	&direct_mode,
	&cq_mode,
	&cbr_mode,
};

static int mode_prepare(struct v4l2_h264_enc_rc *rc)
{
	struct v4l2_h264_enc_state *state = rc->state;
	const struct v4l2_h264_enc_rc_mode *mode = NULL;
	unsigned int i;
	int type;
	int ret;

	if (!state->frame_rc_enable)
		type = V4L2_H264_ENC_RC_TYPE_DIRECT;
	else if (state->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ)
		type = V4L2_H264_ENC_RC_TYPE_CQ;
	else if (state->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR)
		type = V4L2_H264_ENC_RC_TYPE_CBR;
	else if (state->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
		type = V4L2_H264_ENC_RC_TYPE_VBR;
	else
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		if (modes[i] && modes[i]->type == type) {
			mode = modes[i];
			break;
		}
	}

	if (!mode)
		return -EINVAL;

	memset(&rc->stats, 0, sizeof(rc->stats));

	rc->mode = mode;

	ret = v4l2_h264_enc_rc_op(rc, init);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	ret = v4l2_h264_enc_rc_mode_op(rc, init);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	return 0;
}

int v4l2_h264_enc_rc_mode_update(struct v4l2_h264_enc_rc *rc)
{
	if (!rc->mode)
		return 0;

	v4l2_h264_enc_rc_op(rc, exit);
	v4l2_h264_enc_rc_mode_op(rc, exit);

	rc->mode = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_h264_enc_rc_mode_update);

int v4l2_h264_enc_rc_step(struct v4l2_h264_enc_rc *rc,
			  struct v4l2_h264_enc_state *state)
{
	unsigned int qp;
	int ret;

	rc->state = state;

	if (!rc->mode) {
		ret = mode_prepare(rc);
		if (ret)
			return ret;
	}

	ret = v4l2_h264_enc_rc_mode_op(rc, measure, &rc->size);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	/* Estimate using driver implementation first. */
	ret = v4l2_h264_enc_rc_op(rc, estimate, &qp);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	/* Fallback to common implementation otherwise. */
	if (ret == -EOPNOTSUPP) {
		ret = v4l2_h264_enc_rc_mode_op(rc, estimate, &qp);
		if (ret)
			return ret;
	}

	rc->qp = clamp(qp, state->qp_min, state->qp_max);

	return 0;

}
EXPORT_SYMBOL_GPL(v4l2_h264_enc_rc_step);

int v4l2_h264_enc_rc_complete(struct v4l2_h264_enc_rc *rc,
			      unsigned long bytesused)
{
	int ret;

	if (!rc->mode)
		return -EINVAL;

	ret = v4l2_h264_enc_rc_op(rc, complete, bytesused);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	ret = v4l2_h264_enc_rc_mode_op(rc, complete, bytesused);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_h264_enc_rc_complete);

MODULE_DESCRIPTION("V4L2 H.264 Encode Rate Control");
MODULE_AUTHOR("Paul Kocialkowski <paulk@sys-base.io>");
MODULE_LICENSE("GPL");
