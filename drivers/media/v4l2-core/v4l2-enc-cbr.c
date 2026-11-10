// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 stateless encoder constant bitrate rate control
 *
 * One QP state is kept for the stream: it is the QP of the P (and B)
 * pictures, and I pictures are coded a fixed number of steps below it, so
 * that the quality of the reference pictures stays even across GOPs. The
 * I pictures take the mean of the state now and at the previous I picture,
 * so that a drift of the state over a GOP (static content, where P pictures
 * are cheap) moves the size of the next I picture by half as much, and an
 * I picture that exceeded its share of the GOP at the largest I/P size
 * ratio raises the QP of the following I pictures until they fit. Each
 * picture gets a size target from a GOP plan, in which an I picture is
 * "ratio" times the size of a P picture, with the ratio and the IDR period
 * measured on the stream. The difference between the coded size so far and
 * the size the plan expects at this point of the GOP is spread over the next
 * pictures, so that the bitrate converges to the target.
 *
 * A picture far above its size target that also moves the bitrate of the
 * last three seconds well away from the target can be coded a second time
 * with a higher QP (the re-encode of the Rockchip MPP library): I pictures
 * above 1.5 times their target, P pictures above 3 times (a scene change).
 *
 * After each picture the QP state moves by a fraction of the QP error
 * 6 * log2(size / target) (a QP step of 6 halves the size): slowly for small
 * errors, so that the QP does not follow the size noise from picture to
 * picture, and quickly for errors of several QP steps (a scene change, a
 * poor start).
 *
 * The QP to size relation (6 QP steps per halving) is the same for H.264
 * and HEVC, so the model does not depend on the codec. Only the QP delta of
 * the I pictures is a parameter.
 */

#include <linux/export.h>
#include <linux/int_log.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <media/v4l2-enc-cbr.h>

/* The accumulated size error is spread over this many pictures. */
#define CBR_WINDOW		8
/* Limit of the accumulated size error, in seconds of the bitrate */
#define CBR_BUFFER_SECONDS	3
/* I/P size ratio, 1/16 units: initial value (10x) and limits (1.5x .. 30x) */
#define CBR_RATIO_INIT		160
#define CBR_RATIO_MIN		24
#define CBR_RATIO_MAX		480
/*
 * The I QP increase after oversized I pictures: at most 4 QP (1/256 units)
 * per I picture for an I picture twice its largest share, and in all at most
 * a quarter QP per step of the QP state below 45, up to 6 QP (more where the
 * QP is low, where a QP step changes the size by fewer bits). It raises the
 * current state in the mean of cbr_qp_intra(), so half of it reaches the I
 * pictures.
 */
#define CBR_INTRA_ADD_STEP	(4 * 256)
#define CBR_INTRA_ADD_QP_END	45
#define CBR_INTRA_ADD_MAX	(6 * 256)
/* P pictures after an I picture when the I/P size ratio is measured */
#define CBR_RATIO_PICS		4
/* QP errors up to this are corrected by 1/8 per picture, 1/256 units */
#define CBR_ERR_SLOW		(3 * 256)
/*
 * Largest QP increase and decrease per picture, 1/256 units. Decreases are
 * slower, so that pictures that are much smaller than their target, such as
 * repeated pictures, don't pull the QP down far.
 */
#define CBR_STEP_UP_MAX		(4 * 256)
#define CBR_STEP_DOWN_MAX	(2 * 256)
/*
 * Re-encode: size thresholds in halves of the target (I 1.5x, P 3x), the
 * least excess over the target (1/20 of the bits of the three second
 * window, i.e. 0.15 s of the bitrate) and the least deviation of the window
 * from the target (1/10).
 */
#define CBR_REENC_THR_INTRA	3
#define CBR_REENC_THR_INTER	6
#define CBR_REENC_JUMP_DIV	20
#define CBR_REENC_DEV_DIV	10

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

/* Planned size of an I or P picture of the current GOP at an I/P ratio */
static u64 cbr_plan_ratio(const struct v4l2_enc_cbr *cbr, unsigned int fps,
			  bool intra, unsigned int ratio)
{
	unsigned int gop = cbr->gop;
	u64 p;

	/* Until the second I picture, assume at least two seconds. */
	if (!gop)
		gop = max(cbr->since_i + 1, 2 * fps);
	if (gop <= 1)
		return cbr->nominal;

	p = div_u64(cbr->nominal * gop * 16, ratio + 16 * (gop - 1));
	if (intra)
		p = div_u64(p * ratio, 16);

	return max_t(u64, p, 1);
}

static u64 cbr_plan(const struct v4l2_enc_cbr *cbr, unsigned int fps,
		    bool intra)
{
	return cbr_plan_ratio(cbr, fps, intra, cbr->ratio);
}

/*
 * QP of an I picture: the mean of the QP state now, raised by the increase
 * after oversized I pictures, and the state at the previous I picture (the
 * current state weighs 70% when it rose from 33 or below to 35 or above:
 * content that got much harder), less the I QP delta. All-intra streams use
 * the state alone, which then follows the I pictures fully.
 */
static int cbr_qp_intra(const struct v4l2_enc_cbr *cbr)
{
	int cur = cbr->qp_q8, ref = cbr->qp_intra_ref_q8, q8;

	if (cbr->gop == 1 || ref < 0) {
		q8 = cur;
	} else {
		cur += cbr->qp_intra_add_q8;
		if (cur >= 35 * 256 && ref <= 33 * 256)
			q8 = (3 * ref + 7 * cur) / 10;
		else
			q8 = (ref + cur) / 2;
	}

	return DIV_ROUND_CLOSEST(q8, 256) - cbr->qp_delta_intra;
}

/*
 * Start QP of an I picture from its size in bits per luma sample:
 * 20.3 - 6 * log2(bits per sample), clipped to 15..45. This is the relation
 * the Rockchip MPP library uses for its first picture at 1080p, for H.264
 * and HEVC alike.
 */
static int cbr_qp_initial(u64 bits, unsigned int samples)
{
	int qp_q8 = 20 * 256 + 77 +
		    6 * (cbr_log2_q8(samples) - cbr_log2_q8(max_t(u64, bits, 1)));

	return clamp(DIV_ROUND_CLOSEST(qp_q8, 256), 15, 45);
}

/**
 * v4l2_enc_cbr_reset() - reset the rate control state
 * @cbr: the rate control state
 * @qp_delta_intra: QP of the I pictures below the QP of the P pictures
 *
 * The next picture starts the rate control afresh.
 */
void v4l2_enc_cbr_reset(struct v4l2_enc_cbr *cbr, int qp_delta_intra)
{
	memset(cbr, 0, sizeof(*cbr));
	cbr->qp_delta_intra = qp_delta_intra;
}
EXPORT_SYMBOL_GPL(v4l2_enc_cbr_reset);

/**
 * v4l2_enc_cbr_begin() - set up the size target of the next picture
 * @cbr: the rate control state
 * @bitrate: target bitrate in bits per second
 * @timeperframe: frame interval
 * @samples: number of luma samples of a picture
 * @intra: the picture is an I picture
 * @qp_min: lowest QP
 * @qp_max: highest QP
 *
 * Sets @cbr->target_bits. The QP of the picture is then given by
 * v4l2_enc_cbr_qp(). A change of the bitrate keeps the QP state and the
 * measured statistics.
 *
 * Return: 0, or -EINVAL if @bitrate is 0.
 */
int v4l2_enc_cbr_begin(struct v4l2_enc_cbr *cbr, u32 bitrate,
		       const struct v4l2_fract *timeperframe,
		       unsigned int samples, bool intra, int qp_min,
		       int qp_max)
{
	unsigned int fps = cbr_fps(timeperframe);
	u64 nominal = cbr_nominal(bitrate, timeperframe);
	s64 target, expected = 0;
	int qp;

	if (!bitrate)
		return -EINVAL;

	if (!cbr->bitrate) {
		cbr->ratio = CBR_RATIO_INIT;
		cbr->qp_q8 = -1;
		cbr->qp_intra_ref_q8 = -1;
	}

	if (cbr->bitrate != bitrate || cbr->nominal != nominal) {
		cbr->bitrate = bitrate;
		cbr->nominal = nominal;
		cbr->buffer_bits = 0;
		cbr->window_bits = nominal * CBR_BUFFER_SECONDS * fps;
	}

	if (intra) {
		/* An I picture closes the GOP: its length is the IDR period. */
		if (cbr->seen_i) {
			/*
			 * The second picture of an all-intra stream: the size
			 * of the first one at its QP tells the QP of the
			 * nominal size better than the start estimate, which
			 * assumed a GOP with P pictures.
			 */
			if (!cbr->gop && !cbr->since_i)
				cbr->qp_q8 = clamp_t(int, (cbr->qp_pic +
						 cbr->qp_delta_intra) * 256 +
						 6 * (cbr_log2_q8(cbr->i_bits) -
						      cbr_log2_q8(nominal)),
						 (qp_min + cbr->qp_delta_intra) * 256,
						 (qp_max + cbr->qp_delta_intra) * 256);
			cbr->gop = cbr->since_i + 1;
		}
		cbr->since_i = 0;
		cbr->seen_i = true;
	}

	cbr->plan_bits = cbr_plan(cbr, fps, intra);

	if (cbr->qp_q8 < 0) {
		/* First picture: start QP from the bits per sample. */
		qp = cbr_qp_initial(cbr_plan(cbr, fps, true), samples) +
		     cbr->qp_delta_intra;
		cbr->qp_q8 = clamp_t(int, qp, qp_min, qp_max) * 256;
	}

	cbr->qp_pic = intra ? cbr_qp_intra(cbr) :
			      DIV_ROUND_CLOSEST(cbr->qp_q8, 256);

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
	cbr->qp_reenc = 0;

	pr_debug("+ v4l2-enc-cbr: begin");
	pr_debug("  nominal: %llu bits, plan: %llu bits, target: %llu bits",
		 cbr->nominal, cbr->plan_bits, cbr->target_bits);
	pr_debug("  buffer: %lld bits, expected: %lld bits, gop: %u, ratio: %u",
		 cbr->buffer_bits, expected, cbr->gop, cbr->ratio);
	pr_debug("  intra: %d, picture QP %d, I state ref %d/256, I add %d/256",
		 intra, cbr->qp_pic, cbr->qp_intra_ref_q8,
		 cbr->qp_intra_add_q8);

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_enc_cbr_begin);

/**
 * v4l2_enc_cbr_qp() - QP of the picture set up by v4l2_enc_cbr_begin()
 * @cbr: the rate control state
 *
 * Return: the QP, not limited to the QP range of the codec.
 */
int v4l2_enc_cbr_qp(const struct v4l2_enc_cbr *cbr)
{
	return cbr->qp_pic;
}
EXPORT_SYMBOL_GPL(v4l2_enc_cbr_qp);

/**
 * v4l2_enc_cbr_reencode() - decide on coding the picture a second time
 * @cbr: the rate control state
 * @bits: size of the coded picture in bits
 * @timeperframe: frame interval
 * @qp: QP the picture was coded with, within the QP limits of the codec
 * @qp_max: highest QP of the picture
 *
 * Called after a picture was coded and before v4l2_enc_cbr_end(), by
 * drivers that can code the same picture again. A picture is coded at most
 * twice. When it returns true, v4l2_enc_cbr_qp() gives the higher QP of the
 * second pass, which keeps the size target; v4l2_enc_cbr_end() is then
 * called with the size of the second pass only.
 *
 * Return: true if the picture should be coded again.
 */
bool v4l2_enc_cbr_reencode(struct v4l2_enc_cbr *cbr, u64 bits,
			   const struct v4l2_fract *timeperframe, int qp,
			   int qp_max)
{
	unsigned int fps = cbr_fps(timeperframe);
	u64 n = CBR_BUFFER_SECONDS * fps;
	u64 window = cbr->nominal * n;
	u64 thr, ins;
	int add;

	if (!cbr->bitrate || !cbr->target_bits || cbr->qp_reenc)
		return false;

	thr = div_u64(cbr->target_bits * (cbr->intra ? CBR_REENC_THR_INTRA :
					  CBR_REENC_THR_INTER), 2);
	if (bits <= thr)
		return false;

	/* The picture alone moves the window by more than 5% ... */
	if (bits - cbr->target_bits <= div_u64(window, CBR_REENC_JUMP_DIV))
		return false;

	/* ... and the window is more than 10% off the target. */
	ins = cbr->window_bits - div64_u64(cbr->window_bits, n) + bits;
	if (abs_diff(ins, window) <= div_u64(window, CBR_REENC_DEV_DIV))
		return false;

	if (bits > 2 * thr)
		add = 3;
	else if (2 * bits > 3 * thr)
		add = 2;
	else
		add = 1;

	if (qp + add > qp_max)
		return false;

	cbr->qp_pic = qp + add;
	cbr->qp_reenc = add;

	pr_debug("+ v4l2-enc-cbr: re-encode");
	pr_debug("  size: %llu bits, target: %llu bits, window: %llu bits, picture QP %d",
		 bits, cbr->target_bits, ins, cbr->qp_pic);

	return true;
}
EXPORT_SYMBOL_GPL(v4l2_enc_cbr_reencode);

/**
 * v4l2_enc_cbr_end() - account for a coded picture
 * @cbr: the rate control state
 * @bits: size of the coded picture in bits
 * @timeperframe: frame interval
 * @qp_min: lowest QP
 * @qp_max: highest QP
 */
void v4l2_enc_cbr_end(struct v4l2_enc_cbr *cbr, u64 bits,
		      const struct v4l2_fract *timeperframe, int qp_min,
		      int qp_max)
{
	unsigned int fps = cbr_fps(timeperframe);
	s64 limit = cbr->nominal * CBR_BUFFER_SECONDS * fps;
	unsigned int ratio;
	int err, step, shift;
	s64 cap, add;

	if (!cbr->bitrate || !cbr->target_bits)
		return;

	cbr->buffer_bits = clamp_t(s64, cbr->buffer_bits + (s64)bits -
				   (s64)cbr->nominal, -limit, limit);
	/* Sum of the picture sizes over about three seconds */
	cbr->window_bits = cbr->window_bits + bits -
			   div64_u64(cbr->window_bits,
				     CBR_BUFFER_SECONDS * fps);

	/*
	 * QP error of the picture, 1/256 units. A re-encoded picture was coded
	 * above the QP of the state: its error at the state counts that too.
	 */
	err = 6 * (cbr_log2_q8(bits) - cbr_log2_q8(cbr->target_bits)) +
	      cbr->qp_reenc * 256;

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

		/*
		 * Above its share of the GOP at the largest I/P ratio, an I
		 * picture is more than the P pictures of the plan can make
		 * up for: raise the QP of the next I pictures by up to 4 QP
		 * per picture (at twice the share), and lower the increase
		 * again as I pictures come out below the share.
		 */
		cap = cbr_plan_ratio(cbr, fps, true, CBR_RATIO_MAX);
		if (cbr->gop != 1 && (bits > cap || cbr->qp_intra_add_q8)) {
			add = div64_s64(((s64)bits - cap) * CBR_INTRA_ADD_STEP,
					cap);
			add = clamp_t(s64, add, -CBR_INTRA_ADD_STEP,
				      CBR_INTRA_ADD_STEP) +
			      cbr->qp_intra_add_q8;
			cbr->qp_intra_add_q8 =
				clamp_t(s64, add, 0,
					clamp(CBR_INTRA_ADD_QP_END * 64 -
					      cbr->qp_q8 / 4, 0,
					      CBR_INTRA_ADD_MAX));
		}

		/* The state the next I picture takes the mean with */
		cbr->qp_intra_ref_q8 = cbr->qp_q8;
		cbr->qp_q8 += cbr->gop == 1 ? step : step / 4;
	}

	/*
	 * The state is the QP of the P pictures. All-intra streams code their
	 * pictures at the state less the I QP delta, so they need the QP range
	 * shifted up by it to reach the highest QP.
	 */
	shift = cbr->gop == 1 ? cbr->qp_delta_intra : 0;
	cbr->qp_q8 = clamp_t(int, cbr->qp_q8, (qp_min + shift) * 256,
			     (qp_max + shift) * 256);

	pr_debug("+ v4l2-enc-cbr: end");
	pr_debug("  size: %llu bits, target: %llu bits, qp error: %d/256, qp: %d/256",
		 bits, cbr->target_bits, err, cbr->qp_q8);
}
EXPORT_SYMBOL_GPL(v4l2_enc_cbr_end);

MODULE_DESCRIPTION("V4L2 stateless encoder constant bitrate rate control");
MODULE_LICENSE("GPL");
