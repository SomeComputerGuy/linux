// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip VEPU580 (RKVENC) constant bitrate frame rate control
 *
 * One QP state is kept for the stream: it is the QP of the P pictures, and
 * I pictures are coded a fixed number of steps below it, so the quality of
 * the reference pictures stays even across GOPs. Each picture gets a size
 * target from a GOP plan, in which an I picture is "ratio" times the size
 * of a P picture, with the ratio and the IDR period measured on the
 * stream. The difference between the coded size so far and the size the
 * plan expects at this point of the GOP is spread over the next pictures.
 *
 * After each picture the QP state moves by a fraction of the QP error
 * 6 * log2(size / target) (a QP step of 6 halves the size): slowly for
 * small errors, so that the QP does not follow the size noise from picture
 * to picture, and quickly for errors of several QP steps (a scene change,
 * a poor start). The hardware row rate control corrects the QP within the
 * picture on top of that.
 *
 * The behaviour follows the frame-level CBR of the Rockchip MPP library
 * (single QP state, I/P QP delta, learned I/P size ratio, start QP from
 * the bits per pixel).
 */

#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/minmax.h>

#include "rkvenc.h"

/* The accumulated size error is spread over this many pictures. */
#define RKVENC_RC_WINDOW		8
/* Limit of the accumulated size error, in nominal picture sizes */
#define RKVENC_RC_BUFFER_MAX		32
/* I/P size ratio limits, 1/16 units (1.5x .. 30x) */
#define RKVENC_RC_RATIO_MIN		24
#define RKVENC_RC_RATIO_MAX		480
/* P pictures after an I picture when the I/P size ratio is measured */
#define RKVENC_RC_RATIO_PICS		4
/* QP errors up to this are corrected by 1/8 per picture, 1/256 units */
#define RKVENC_RC_ERR_SLOW		(3 * 256)
/* Largest QP change per picture, 1/256 units */
#define RKVENC_RC_STEP_MAX		(4 * 256)

/* log2(x) in 8.8 fixed point, linear between powers of two */
int rkvenc_log2_q8(u64 x)
{
	unsigned int n;

	if (!x)
		return 0;

	n = ilog2(x);
	if (n >= 8)
		return n * 256 + (int)((x >> (n - 8)) - 256);

	return n * 256 + (int)(((x << 8) >> n) - 256);
}

static unsigned int rkvenc_rc_fps(const struct v4l2_fract *tpf)
{
	if (!tpf->numerator || !tpf->denominator)
		return 30;

	return max(DIV_ROUND_CLOSEST(tpf->denominator, tpf->numerator), 1U);
}

static u64 rkvenc_rc_nominal(u32 bitrate, const struct v4l2_fract *tpf)
{
	if (!tpf->numerator || !tpf->denominator)
		return max(bitrate / 30, 1U);

	return max_t(u64, div_u64((u64)bitrate * tpf->numerator,
				  tpf->denominator), 1);
}

/* Planned size of an I or P picture of the current GOP */
static u64 rkvenc_rc_plan(const struct rkvenc_rc *rc, unsigned int fps,
			  bool intra)
{
	unsigned int gop = rc->gop;
	u64 p;

	/* Until the second I picture, assume at least two seconds. */
	if (!gop)
		gop = max(rc->since_i + 1, 2 * fps);
	if (gop <= 1)
		return rc->nominal;

	p = div_u64(rc->nominal * gop * 16, rc->ratio + 16 * (gop - 1));
	if (intra)
		p = div_u64(p * rc->ratio, 16);

	return max_t(u64, p, 1);
}

/**
 * rkvenc_rc_estimate() - QP and size target of the next picture
 * @rc: rate control state, zeroed before the first picture
 * @codec: codec parameters
 * @bitrate: target bitrate in bits per second
 * @tpf: frame interval
 * @pixels: luma samples per picture
 * @intra: the picture is an I picture
 * @qp_min: lowest QP
 * @qp_max: highest QP
 *
 * The size target is left in @rc->target_bits for the hardware row rate
 * control.
 *
 * Return: the QP of the picture, or a negative error code.
 */
int rkvenc_rc_estimate(struct rkvenc_rc *rc, const struct rkvenc_rc_codec *codec,
		       u32 bitrate, const struct v4l2_fract *tpf,
		       unsigned int pixels, bool intra, int qp_min, int qp_max)
{
	unsigned int fps = rkvenc_rc_fps(tpf);
	u64 nominal = rkvenc_rc_nominal(bitrate, tpf);
	s64 target, expected = 0;
	int qp;

	if (!bitrate)
		return -EINVAL;

	if (!rc->bitrate) {
		rc->ratio = codec->ratio_init;
		rc->qp_q8 = -1;
	}

	/* A new bitrate keeps the QP state and the measured statistics. */
	if (rc->bitrate != bitrate || rc->nominal != nominal) {
		rc->bitrate = bitrate;
		rc->nominal = nominal;
		rc->buffer_bits = 0;
	}

	if (intra) {
		/* An I picture closes the GOP: its length is the IDR period. */
		if (rc->seen_i)
			rc->gop = rc->since_i + 1;
		rc->since_i = 0;
		rc->seen_i = true;
	}

	rc->plan_bits = rkvenc_rc_plan(rc, fps, intra);

	if (rc->qp_q8 < 0) {
		/* First picture: start QP from the bits per pixel. */
		qp = codec->qp_initial(rkvenc_rc_plan(rc, fps, true), pixels) +
		     codec->ip_delta;
		rc->qp_q8 = clamp(qp, qp_min, qp_max) * 256;
	}

	/*
	 * The buffer holds the coded minus the nominal sizes. Within a GOP the
	 * plan expects it to go up by the excess of the I picture and back
	 * down to zero by the savings of the P pictures until the next I
	 * picture. Spread the difference to that over the next pictures.
	 */
	if (!intra && rc->seen_i)
		expected = (s64)rkvenc_rc_plan(rc, fps, true) - (s64)nominal +
			   (s64)rc->since_i * ((s64)rc->plan_bits - (s64)nominal);

	target = (s64)rc->plan_bits -
		 div_s64(rc->buffer_bits - max_t(s64, expected, 0),
			 RKVENC_RC_WINDOW);
	rc->target_bits = clamp_t(s64, target, max_t(u64, rc->plan_bits / 4, 1),
				  rc->plan_bits * 2);
	rc->intra = intra;

	qp = DIV_ROUND_CLOSEST(rc->qp_q8, 256);
	if (intra)
		qp -= codec->ip_delta;

	return clamp(qp, qp_min, qp_max);
}

/**
 * rkvenc_rc_complete() - account for a coded picture
 * @rc: rate control state
 * @bits: coded size of the picture
 * @qp_min: lowest QP
 * @qp_max: highest QP
 */
void rkvenc_rc_complete(struct rkvenc_rc *rc, u64 bits, int qp_min, int qp_max)
{
	s64 limit = rc->nominal * RKVENC_RC_BUFFER_MAX;
	int err, step;

	if (!rc->bitrate || !rc->target_bits)
		return;

	rc->buffer_bits = clamp_t(s64, rc->buffer_bits + (s64)bits -
				  (s64)rc->nominal, -limit, limit);

	/* QP error of the picture, 1/256 units */
	err = 6 * (rkvenc_log2_q8(bits) - rkvenc_log2_q8(rc->target_bits));

	step = err / 8;
	if (err > RKVENC_RC_ERR_SLOW)
		step += (err - RKVENC_RC_ERR_SLOW) / 2;
	else if (err < -RKVENC_RC_ERR_SLOW)
		step += (err + RKVENC_RC_ERR_SLOW) / 2;
	step = clamp(step, -RKVENC_RC_STEP_MAX, RKVENC_RC_STEP_MAX);

	if (!rc->intra) {
		rc->qp_q8 += step;
		rc->p_bits = rc->p_bits ? (7 * rc->p_bits + bits) / 8 : bits;
		rc->since_i++;

		/* I/P size ratio, once a few P pictures were coded */
		if (rc->since_i == RKVENC_RC_RATIO_PICS && rc->i_bits) {
			unsigned int ratio = div64_u64(rc->i_bits * 16, rc->p_bits);

			ratio = clamp(ratio, RKVENC_RC_RATIO_MIN, RKVENC_RC_RATIO_MAX);
			rc->ratio = (rc->ratio + ratio + 1) / 2;
		}
	} else {
		/*
		 * The size of an I picture mostly reflects the I/P ratio
		 * estimate: only all-intra streams steer the QP fully with it.
		 */
		rc->i_bits = bits;
		rc->qp_q8 += rc->gop == 1 ? step : step / 4;
	}

	rc->qp_q8 = clamp(rc->qp_q8, qp_min * 256, qp_max * 256);
}
