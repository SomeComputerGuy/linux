// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 H.264 Encode Rate Control
 *
 * Copyright (C) 2025-2026 Paul Kocialkowski <paulk@sys-base.io>
 */

#include <linux/int_log.h>
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
 * Constant bitrate mode
 *
 * One QP state is kept for the stream: it is the QP of the P (and B)
 * pictures, and I pictures are coded a fixed number of steps below it, so
 * that the quality of the reference pictures stays even across GOPs. Each
 * picture gets a size target from a GOP plan, in which an I picture is
 * "ratio" times the size of a P picture, with the ratio and the IDR period
 * measured on the stream. The difference between the coded size so far and
 * the size the plan expects at this point of the GOP is spread over the next
 * pictures, so that the bitrate converges to the target.
 *
 * After each picture the QP state moves by a fraction of the QP error
 * 6 * log2(size / target) (a QP step of 6 halves the size): slowly for small
 * errors, so that the QP does not follow the size noise from picture to
 * picture, and quickly for errors of several QP steps (a scene change, a
 * poor start).
 */

/* The accumulated size error is spread over this many pictures. */
#define CBR_WINDOW		8
/* Limit of the accumulated size error, in seconds of the bitrate */
#define CBR_BUFFER_SECONDS	3
/* I/P size ratio, 1/16 units: initial value (10x) and limits (1.5x .. 30x) */
#define CBR_RATIO_INIT		160
#define CBR_RATIO_MIN		24
#define CBR_RATIO_MAX		480
/* P pictures after an I picture when the I/P size ratio is measured */
#define CBR_RATIO_PICS		4
/* QP of the I pictures below the QP of the P pictures */
#define CBR_QP_DELTA_INTRA	2
/* QP errors up to this are corrected by 1/8 per picture, 1/256 units */
#define CBR_ERR_SLOW		(3 * 256)
/*
 * Largest QP increase and decrease per picture, 1/256 units. Decreases are
 * slower, so that pictures that are much smaller than their target, such as
 * repeated pictures, don't pull the QP down far.
 */
#define CBR_STEP_UP_MAX		(4 * 256)
#define CBR_STEP_DOWN_MAX	(2 * 256)

/* log2(x) in 8.8 fixed point */
static int cbr_log2_q8(u64 x)
{
	unsigned int shift = 0;

	if (!x)
		return 0;

	while (x > U32_MAX) {
		x >>= 1;
		shift++;
	}

	return (int)(intlog2(x) >> 16) + shift * 256;
}

static unsigned int cbr_fps(const struct v4l2_fract *timeperframe)
{
	if (!timeperframe->numerator || !timeperframe->denominator)
		return 30;

	return max(DIV_ROUND_CLOSEST(timeperframe->denominator,
				     timeperframe->numerator), 1U);
}

static u64 cbr_nominal(u32 bitrate, const struct v4l2_fract *timeperframe)
{
	if (!timeperframe->numerator || !timeperframe->denominator)
		return max(bitrate / 30, 1U);

	return max_t(u64, div_u64((u64)bitrate * timeperframe->numerator,
				  timeperframe->denominator), 1);
}

/* Planned size of an I or P picture of the current GOP */
static u64 cbr_plan(const struct v4l2_h264_enc_rc_cbr *cbr, unsigned int fps,
		    bool intra)
{
	unsigned int gop = cbr->gop;
	u64 p;

	/* Until the second I picture, assume at least two seconds. */
	if (!gop)
		gop = max(cbr->since_i + 1, 2 * fps);
	if (gop <= 1)
		return cbr->nominal;

	p = div_u64(cbr->nominal * gop * 16, cbr->ratio + 16 * (gop - 1));
	if (intra)
		p = div_u64(p * cbr->ratio, 16);

	return max_t(u64, p, 1);
}

/*
 * Start QP of an I picture from its size in bits per luma sample:
 * 20.3 - 6 * log2(bits per sample), clipped to 15..45. This is the relation
 * the Rockchip MPP library uses for its first picture at 1080p.
 */
static int cbr_qp_initial(u64 bits, unsigned int samples)
{
	int qp_q8 = 20 * 256 + 77 +
		    6 * (cbr_log2_q8(samples) - cbr_log2_q8(max_t(u64, bits, 1)));

	return clamp(DIV_ROUND_CLOSEST(qp_q8, 256), 15, 45);
}

static int cbr_init(struct v4l2_h264_enc_rc *rc)
{
	memset(&rc->cbr, 0, sizeof(rc->cbr));

	return 0;
}

static int cbr_measure(struct v4l2_h264_enc_rc *rc, unsigned long *size)
{
	struct v4l2_h264_enc_state *state = rc->state;
	struct v4l2_h264_enc_rc_cbr *cbr = &rc->cbr;
	struct v4l2_fract *timeperframe = &state->timeperframe;
	bool intra = state->encode.slice_type == V4L2_H264_SLICE_TYPE_I;
	unsigned int fps = cbr_fps(timeperframe);
	u64 nominal = cbr_nominal(state->bitrate, timeperframe);
	unsigned int samples = state->width_mbs * state->height_mbs * 256;
	s64 target, expected = 0;
	int qp;

	if (!state->bitrate)
		return -EINVAL;

	if (!cbr->bitrate) {
		cbr->ratio = CBR_RATIO_INIT;
		cbr->qp_q8 = -1;
	}

	/* A new bitrate keeps the QP state and the measured statistics. */
	if (cbr->bitrate != state->bitrate || cbr->nominal != nominal) {
		cbr->bitrate = state->bitrate;
		cbr->nominal = nominal;
		cbr->buffer_bits = 0;
	}

	if (intra) {
		/* An I picture closes the GOP: its length is the IDR period. */
		if (cbr->seen_i)
			cbr->gop = cbr->since_i + 1;
		cbr->since_i = 0;
		cbr->seen_i = true;
	}

	cbr->plan_bits = cbr_plan(cbr, fps, intra);

	if (cbr->qp_q8 < 0) {
		/* First picture: start QP from the bits per sample. */
		qp = cbr_qp_initial(cbr_plan(cbr, fps, true), samples) +
		     CBR_QP_DELTA_INTRA;
		cbr->qp_q8 = clamp_t(int, qp, state->qp_min, state->qp_max) *
			     256;
	}

	/*
	 * The buffer holds the coded minus the nominal sizes. Within a GOP the
	 * plan expects it to go up by the excess of the I picture and back
	 * down to zero by the savings of the P pictures until the next I
	 * picture. Spread the difference to that over the next pictures.
	 */
	if (!intra && cbr->seen_i)
		expected = (s64)cbr_plan(cbr, fps, true) - (s64)nominal +
			   (s64)cbr->since_i *
			   ((s64)cbr->plan_bits - (s64)nominal);

	target = (s64)cbr->plan_bits -
		 div_s64(cbr->buffer_bits - max_t(s64, expected, 0),
			 CBR_WINDOW);
	cbr->target_bits = clamp_t(s64, target,
				   max_t(u64, cbr->plan_bits / 4, 8),
				   cbr->plan_bits * 2);
	cbr->intra = intra;

	pr_debug("+ v4l2-h264-enc-rc: cbr measure");
	pr_debug("  nominal: %llu bits, plan: %llu bits, target: %llu bits",
		 cbr->nominal, cbr->plan_bits, cbr->target_bits);
	pr_debug("  buffer: %lld bits, expected: %lld bits, gop: %u, ratio: %u",
		 cbr->buffer_bits, expected, cbr->gop, cbr->ratio);

	*size = max_t(unsigned long, cbr->target_bits / 8, 1);

	return 0;
}

static int cbr_estimate(struct v4l2_h264_enc_rc *rc, unsigned int *qp)
{
	struct v4l2_h264_enc_rc_cbr *cbr = &rc->cbr;
	int value;

	value = DIV_ROUND_CLOSEST(cbr->qp_q8, 256);
	if (cbr->intra)
		value -= CBR_QP_DELTA_INTRA;

	*qp = clamp(value, 0, 51);

	return 0;
}

static void cbr_update(struct v4l2_h264_enc_rc *rc, u64 bits)
{
	struct v4l2_h264_enc_state *state = rc->state;
	struct v4l2_h264_enc_rc_cbr *cbr = &rc->cbr;
	s64 limit = cbr->nominal * CBR_BUFFER_SECONDS *
		    cbr_fps(&state->timeperframe);
	unsigned int ratio;
	int err, step;

	if (!cbr->bitrate || !cbr->target_bits)
		return;

	cbr->buffer_bits = clamp_t(s64, cbr->buffer_bits + (s64)bits -
				   (s64)cbr->nominal, -limit, limit);

	/* QP error of the picture, 1/256 units */
	err = 6 * (cbr_log2_q8(bits) - cbr_log2_q8(cbr->target_bits));

	step = err / 8;
	if (err > CBR_ERR_SLOW)
		step += (err - CBR_ERR_SLOW) / 2;
	else if (err < -CBR_ERR_SLOW)
		step += (err + CBR_ERR_SLOW) / 2;
	step = clamp(step, -CBR_STEP_DOWN_MAX, CBR_STEP_UP_MAX);

	if (!cbr->intra) {
		cbr->qp_q8 += step;
		cbr->p_bits = cbr->p_bits ? (7 * cbr->p_bits + bits) / 8 : bits;
		cbr->since_i++;

		/* I/P size ratio, once a few P pictures were coded */
		if (cbr->since_i == CBR_RATIO_PICS && cbr->i_bits) {
			ratio = div64_u64(cbr->i_bits * 16, cbr->p_bits);
			ratio = clamp(ratio, CBR_RATIO_MIN, CBR_RATIO_MAX);
			cbr->ratio = (cbr->ratio + ratio + 1) / 2;
		}
	} else {
		/*
		 * The size of an I picture mostly reflects the I/P ratio
		 * estimate: only all-intra streams steer the QP fully with it.
		 */
		cbr->i_bits = bits;
		cbr->qp_q8 += cbr->gop == 1 ? step : step / 4;
	}

	cbr->qp_q8 = clamp_t(int, cbr->qp_q8, state->qp_min * 256,
			     state->qp_max * 256);

	pr_debug("+ v4l2-h264-enc-rc: cbr update");
	pr_debug("  size: %llu bits, target: %llu bits, qp error: %d/256, qp: %d/256",
		 bits, cbr->target_bits, err, cbr->qp_q8);
}

static int cbr_complete(struct v4l2_h264_enc_rc *rc, unsigned long bytesused)
{
	cbr_update(rc, (u64)bytesused * 8);

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
