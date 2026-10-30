// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip Video Decoder VP9 backend
 *
 * Copyright (C) 2025 Venkata Atchuta Bheemeswara Sarma Darbha <vdarbha0473@gmail.com>
 *
 * Copyright (C) 2019 Collabora, Ltd.
 *	Boris Brezillon <boris.brezillon@collabora.com>
 * Copyright (C) 2021 Collabora, Ltd.
 *	Andrzej Pietrasiewicz <andrzej.p@collabora.com>
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *	Alpha Lin <Alpha.Lin@rock-chips.com>
 */

/*
 * The VDPU381 does the probability handling of VP9 itself: it reads the
 * probabilities of the frame context the frame uses, applies the forward
 * updates of the compressed header, which the driver passes in an update
 * table, adapts the probabilities to the symbols of the frame (backward
 * adaptation) and saves them to the frame context. Every stream has four
 * frame context buffers, which only the hardware writes, and a table of the
 * default probabilities for the contexts that were reset.
 *
 * For following the VP9 spec, start reading this code from rkvdec_vp9_run().
 */

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-vp9.h>

#include "rkvdec-rcb.h"
#include "rkvdec.h"
#include "rkvdec-ccu.h"
#include "rkvdec-vdpu381-regs.h"
#include "rkvdec-vp9-common.h"

/* Size of a probability table, as the hardware reads and writes it */
#define VDPU381_VP9_PROB_TBL_SIZE	4864
/* Size of the symbol counts the hardware writes */
#define VDPU381_VP9_COUNT_TBL_SIZE	13208
/*
 * VP9 segmap size: ceil(width/64) * ceil(height/64) * 256 / 4
 * For 4K (3840x2160): 60 * 34 * 64 = 130,560 bytes
 * For 8K (7680x4320): 120 * 68 * 64 = 522,240 bytes
 * Use 8K-safe value to cover all supported resolutions.
 */
#define RKVDEC_VP9_MAX_SEGMAP_SIZE	524288

#define VDPU381_VP9_NUM_FRAME_CTXS	4

/*
 * Probability tables are made of 128-bit units. The coefficient probabilities
 * of a transform size and plane type (6 bands x 6 contexts x 3 nodes) take 27
 * bytes of every 32, in four rows: the same holds for their update flags,
 * 27 bits of every 32.
 */
#define VDPU381_VP9_COEF_ROW_LEN	27
#define VDPU381_VP9_COEF_ROW_SIZE	32
#define VDPU381_VP9_COEF_PLANE_SIZE	128

/* First 5 x 128 bits of the probability and update tables */
struct vdpu381_vp9_probs_head {
	u8 partition[16][3];
	u8 seg_pred[3];
	u8 seg_tree[7];
	u8 skip[3];
	u8 tx32[2][3];
	u8 tx16[2][2];
	u8 tx8[2][1];
	u8 is_inter[4];
	u8 reserved[3];
} __packed;

/*
 * The probabilities of a frame context, in the default table and in the
 * frame contexts the hardware saves. The update table of an inter frame
 * holds the updates in the same layout.
 */
struct vdpu381_vp9_frame_probs {
	struct vdpu381_vp9_probs_head head;
	u8 y_mode[4][9];
	u8 comp_mode[5];
	u8 comp_ref[5];
	u8 single_ref[5][2];
	u8 inter_mode[7][3];
	u8 interp_filter[4][2];
	u8 reserved0[11];
	/* [ref][tx size][plane type] */
	u8 coef[2][4][2][VDPU381_VP9_COEF_PLANE_SIZE];
	/* three modes per row, the last row holds mode 9 */
	u8 uv_mode[4][32];
	struct v4l2_vp9_frame_mv_context mv;
	u8 reserved1[27];
} __packed;

/* Bit flags telling which probabilities are updated, then the updates */
struct vdpu381_vp9_inter_updates {
	u8 flags[320];
	struct vdpu381_vp9_frame_probs delta;
} __packed;

/*
 * Intra frames code no mode probabilities: the update table holds the key
 * frame probabilities of the partition and intra modes instead.
 */
struct vdpu381_vp9_intra_updates {
	u8 flags[272];
	struct vdpu381_vp9_probs_head head;
	u8 coef_intra[4][2][VDPU381_VP9_COEF_PLANE_SIZE];
	struct rkvdec_vp9_intra_mode_probs intra_mode[10];
	u8 coef_inter[4][2][VDPU381_VP9_COEF_PLANE_SIZE];
} __packed;

union vdpu381_vp9_prob_tbl {
	struct vdpu381_vp9_frame_probs probs;
	u8 raw[ALIGN(VDPU381_VP9_PROB_TBL_SIZE, SZ_4K)];
};

/* Tables of a stream */
struct vdpu381_vp9_stream_tbl {
	union vdpu381_vp9_prob_tbl default_probs;
	/* Written by the hardware only */
	union vdpu381_vp9_prob_tbl ctx_probs[VDPU381_VP9_NUM_FRAME_CTXS];
	u8 segmap[2][RKVDEC_VP9_MAX_SEGMAP_SIZE];
};

/* Tables of a job, the hardware reads and writes them while it decodes */
struct vdpu381_vp9_job_tbl {
	union {
		struct vdpu381_vp9_inter_updates inter;
		struct vdpu381_vp9_intra_updates intra;
		u8 raw[ALIGN(VDPU381_VP9_PROB_TBL_SIZE, SZ_4K)];
	} updates;
	u8 counts[ALIGN(VDPU381_VP9_COUNT_TBL_SIZE, SZ_4K)];
};

struct rkvdec_vp9_frame_info {
	u32 valid : 1;
	/* segmentation map holding the segment IDs after this frame */
	u32 segmapid : 1;
	/* segmentation was enabled on a frame since the last intra frame */
	u32 segmap_valid : 1;
	u32 frame_context_idx : 2;
	u32 reference_mode : 2;
	u32 tx_mode : 3;
	u32 interpolation_filter : 3;
	u32 flags;
	u16 width;
	u16 height;
	u64 timestamp;
	struct v4l2_vp9_segmentation seg;
	struct v4l2_vp9_loop_filter lf;
};

struct rkvdec_vp9_ctx {
	struct rkvdec_aux_buf stream_tbl;
	/* One table per job, see &rkvdec_job.slot */
	struct rkvdec_aux_buf job_tbl[RKVDEC_MAX_JOBS];
	/* The frame contexts the hardware saved, the others hold the defaults */
	bool ctx_saved[VDPU381_VP9_NUM_FRAME_CTXS];
	struct rkvdec_vp9_frame_info cur;
	struct rkvdec_vp9_frame_info last;
	struct rkvdec_vdpu381_regs_vp9 regs;
};

struct vdpu381_vp9_flag_writer {
	u8 *flags;
	unsigned int pos;
};

static void vdpu381_vp9_skip_flags(struct vdpu381_vp9_flag_writer *fw,
				   unsigned int n)
{
	fw->pos += n;
}

static void vdpu381_vp9_set_flag(struct vdpu381_vp9_flag_writer *fw,
				 unsigned int pos)
{
	fw->flags[pos / 8] |= BIT(pos % 8);
}

/*
 * The compressed header codes the update of a probability as an index into
 * inv_map_table[] (spec 6.3.5). V4L2 passes the table entry, the hardware
 * wants the index back. The first 20 entries are 7 + 13 * i, the others are
 * the remaining values in increasing order.
 */
static u8 vdpu381_vp9_update_index(u8 v)
{
	if (v >= 7 && !((v - 7) % 13))
		return (v - 7) / 13;

	return 20 + v - 1 - (v + 6) / 13;
}

/*
 * Writes the flags of @n probabilities, and the updates to @delta in the
 * hardware form. A zero update means that the probability isn't updated,
 * and @upd is NULL if none of them can be. Motion vector updates are new
 * probabilities, which the hardware takes as they are (@mv).
 */
static void vdpu381_vp9_put_updates(struct vdpu381_vp9_flag_writer *fw,
				    u8 *delta, const u8 *upd, unsigned int n,
				    bool mv)
{
	unsigned int i;

	for (i = 0; upd && i < n; i++) {
		if (!upd[i])
			continue;

		vdpu381_vp9_set_flag(fw, fw->pos + i);
		if (delta)
			delta[i] = mv ? upd[i] : vdpu381_vp9_update_index(upd[i]);
	}

	vdpu381_vp9_skip_flags(fw, n);
}

/* The coefficient updates of one transform size and plane type */
static void vdpu381_vp9_put_coef_updates(struct vdpu381_vp9_flag_writer *fw,
					 u8 *plane, const u8 upd[6][6][3])
{
	const u8 *u = upd ? &upd[0][0][0] : NULL;
	unsigned int i, off;

	for (i = 0; u && i < 6 * 6 * 3; i++) {
		if (!u[i])
			continue;

		off = i / VDPU381_VP9_COEF_ROW_LEN * VDPU381_VP9_COEF_ROW_SIZE +
		      i % VDPU381_VP9_COEF_ROW_LEN;
		vdpu381_vp9_set_flag(fw, fw->pos + off);
		plane[off] = vdpu381_vp9_update_index(u[i]);
	}

	vdpu381_vp9_skip_flags(fw, VDPU381_VP9_COEF_PLANE_SIZE);
}

/*
 * The coefficient updates of all transform sizes and plane types, first those
 * of intra blocks, then those of inter blocks (@ref). The compressed header
 * codes no updates for the transform sizes the frame doesn't use (spec 6.3.7).
 */
static void vdpu381_vp9_put_coefs(struct vdpu381_vp9_flag_writer *fw,
				  u8 planes[4][2][VDPU381_VP9_COEF_PLANE_SIZE],
				  const struct v4l2_ctrl_vp9_compressed_hdr *hdr,
				  unsigned int ref)
{
	unsigned int i, j;

	for (i = 0; i < 4; i++) {
		bool coded = i <= hdr->tx_mode;

		for (j = 0; j < 2; j++)
			vdpu381_vp9_put_coef_updates(fw, planes[i][j],
						     coded ? hdr->coef[i][j][ref] : NULL);
	}
}

/* The skip and transform size updates, coded by intra and inter frames */
static void vdpu381_vp9_put_tx_skip(struct vdpu381_vp9_flag_writer *fw,
				    struct vdpu381_vp9_probs_head *head,
				    const struct v4l2_ctrl_vp9_compressed_hdr *hdr)
{
	bool tx_select = hdr->tx_mode == V4L2_VP9_TX_MODE_SELECT;

	vdpu381_vp9_put_updates(fw, head->skip, hdr->skip, 3, false);
	vdpu381_vp9_put_updates(fw, &head->tx32[0][0],
				tx_select ? &hdr->tx32[0][0] : NULL, 6, false);
	vdpu381_vp9_put_updates(fw, &head->tx16[0][0],
				tx_select ? &hdr->tx16[0][0] : NULL, 4, false);
	vdpu381_vp9_put_updates(fw, &head->tx8[0][0],
				tx_select ? &hdr->tx8[0][0] : NULL, 2, false);
}

static void vdpu381_vp9_put_seg_probs(struct vdpu381_vp9_probs_head *head,
				      const struct v4l2_ctrl_vp9_frame *dec_params)
{
	memcpy(head->seg_pred, dec_params->seg.pred_probs, sizeof(head->seg_pred));
	memcpy(head->seg_tree, dec_params->seg.tree_probs, sizeof(head->seg_tree));
}

static void vdpu381_vp9_write_intra_updates(struct vdpu381_vp9_intra_updates *tbl,
					    const struct v4l2_ctrl_vp9_frame *dec_params,
					    const struct v4l2_ctrl_vp9_compressed_hdr *hdr)
{
	struct vdpu381_vp9_flag_writer fw = { .flags = tbl->flags };
	unsigned int i, j, k, idx;

	/* No partition, segmentation and is_inter updates */
	vdpu381_vp9_skip_flags(&fw, 48 + 3 + 7);
	vdpu381_vp9_put_tx_skip(&fw, &tbl->head, hdr);
	vdpu381_vp9_skip_flags(&fw, 4 + 3);

	vdpu381_vp9_put_coefs(&fw, tbl->coef_intra, hdr, 0);
	vdpu381_vp9_put_coefs(&fw, tbl->coef_inter, hdr, 1);
	WARN_ON(fw.pos > sizeof(tbl->flags) * 8);

	memcpy(tbl->head.partition, v4l2_vp9_kf_partition_probs,
	       sizeof(tbl->head.partition));
	vdpu381_vp9_put_seg_probs(&tbl->head, dec_params);

	/* Rows of 27 probabilities, padded to 32 bytes */
	for (i = 0; i < ARRAY_SIZE(v4l2_vp9_kf_y_mode_prob); i++) {
		const u8 *p = &v4l2_vp9_kf_y_mode_prob[i][0][0];

		for (j = 0, idx = 0; j < sizeof(v4l2_vp9_kf_y_mode_prob[0]); j++) {
			tbl->intra_mode[i].y_mode[idx++] = p[j];
			if (!((j + 1) % VDPU381_VP9_COEF_ROW_LEN))
				idx += VDPU381_VP9_COEF_ROW_SIZE - VDPU381_VP9_COEF_ROW_LEN;
		}
	}

	/* 23 bytes in each of the first entries */
	for (k = 0; k < sizeof(v4l2_vp9_kf_uv_mode_prob); k++) {
		const u8 *p = &v4l2_vp9_kf_uv_mode_prob[0][0];

		tbl->intra_mode[k / 23].uv_mode[k % 23] = p[k];
	}
}

static void vdpu381_vp9_write_inter_updates(struct vdpu381_vp9_inter_updates *tbl,
					    const struct v4l2_ctrl_vp9_frame *dec_params,
					    const struct v4l2_ctrl_vp9_compressed_hdr *hdr)
{
	struct vdpu381_vp9_flag_writer fw = { .flags = tbl->flags };
	struct vdpu381_vp9_frame_probs *delta = &tbl->delta;
	struct v4l2_vp9_frame_mv_context *dmv = &delta->mv;
	const struct v4l2_vp9_mv_probs *mv = &hdr->mv;
	u8 ref_mode = dec_params->reference_mode;
	bool hp = dec_params->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV;
	bool filter = dec_params->interpolation_filter ==
		      V4L2_VP9_INTERP_FILTER_SWITCHABLE;

	/* Same conditions as v4l2_vp9_fw_update_probs() */
	vdpu381_vp9_put_updates(&fw, &delta->head.partition[0][0],
				&hdr->partition[0][0], 48, false);
	vdpu381_vp9_skip_flags(&fw, 3 + 7);
	vdpu381_vp9_put_tx_skip(&fw, &delta->head, hdr);
	vdpu381_vp9_put_updates(&fw, delta->head.is_inter, hdr->is_inter, 4, false);
	vdpu381_vp9_skip_flags(&fw, 3);

	vdpu381_vp9_put_updates(&fw, &delta->y_mode[0][0], &hdr->y_mode[0][0],
				36, false);
	vdpu381_vp9_put_updates(&fw, delta->comp_mode,
				ref_mode == V4L2_VP9_REFERENCE_MODE_SELECT ?
				hdr->comp_mode : NULL, 5, false);
	vdpu381_vp9_put_updates(&fw, delta->comp_ref,
				ref_mode != V4L2_VP9_REFERENCE_MODE_SINGLE_REFERENCE ?
				hdr->comp_ref : NULL, 5, false);
	vdpu381_vp9_put_updates(&fw, &delta->single_ref[0][0],
				ref_mode != V4L2_VP9_REFERENCE_MODE_COMPOUND_REFERENCE ?
				&hdr->single_ref[0][0] : NULL, 10, false);
	vdpu381_vp9_put_updates(&fw, &delta->inter_mode[0][0],
				&hdr->inter_mode[0][0], 21, false);
	vdpu381_vp9_put_updates(&fw, &delta->interp_filter[0][0],
				filter ? &hdr->interp_filter[0][0] : NULL, 8, false);
	vdpu381_vp9_skip_flags(&fw, 11);

	vdpu381_vp9_put_coefs(&fw, delta->coef[0], hdr, 0);
	vdpu381_vp9_put_coefs(&fw, delta->coef[1], hdr, 1);

	/* The compressed header has no UV mode updates */
	vdpu381_vp9_skip_flags(&fw, 4 * 32);

	vdpu381_vp9_put_updates(&fw, dmv->joint, mv->joint, 3, true);
	vdpu381_vp9_put_updates(&fw, dmv->sign, mv->sign, 2, true);
	vdpu381_vp9_put_updates(&fw, &dmv->classes[0][0], &mv->classes[0][0],
				20, true);
	vdpu381_vp9_put_updates(&fw, dmv->class0_bit, mv->class0_bit, 2, true);
	vdpu381_vp9_put_updates(&fw, &dmv->bits[0][0], &mv->bits[0][0], 20, true);
	vdpu381_vp9_put_updates(&fw, &dmv->class0_fr[0][0][0],
				&mv->class0_fr[0][0][0], 12, true);
	vdpu381_vp9_put_updates(&fw, &dmv->fr[0][0], &mv->fr[0][0], 6, true);
	vdpu381_vp9_put_updates(&fw, dmv->class0_hp, hp ? mv->class0_hp : NULL,
				2, true);
	vdpu381_vp9_put_updates(&fw, dmv->hp, hp ? mv->hp : NULL, 2, true);
	WARN_ON(fw.pos > sizeof(tbl->flags) * 8);

	vdpu381_vp9_put_seg_probs(&delta->head, dec_params);
	/* The Rockchip MPP fills the unused part of the last UV mode row so */
	memset(&delta->uv_mode[3][9], 0xff, sizeof(delta->uv_mode[3]) - 9);
}

/* Writes the update table of the frame to the table of the job. */
static void vdpu381_vp9_write_updates(struct rkvdec_ctx *ctx,
				      const struct rkvdec_vp9_run *run,
				      bool intra_only)
{
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct vdpu381_vp9_job_tbl *tbl = vp9_ctx->job_tbl[ctx->job->slot].cpu;

	memset(&tbl->updates, 0, VDPU381_VP9_PROB_TBL_SIZE);

	if (intra_only)
		vdpu381_vp9_write_intra_updates(&tbl->updates.intra,
						run->decode_params, run->prob_updates);
	else
		vdpu381_vp9_write_inter_updates(&tbl->updates.inter,
						run->decode_params, run->prob_updates);
}

/* The probabilities of a frame context that was reset (spec 10.5). */
static void vdpu381_vp9_write_default_probs(struct vdpu381_vp9_frame_probs *probs)
{
	const struct v4l2_vp9_frame_context *def = &v4l2_vp9_default_probs;
	unsigned int i, j, k;

	memcpy(probs->head.partition, def->partition, sizeof(probs->head.partition));
	memcpy(probs->head.skip, def->skip, sizeof(probs->head.skip));
	memcpy(probs->head.tx32, def->tx32, sizeof(probs->head.tx32));
	memcpy(probs->head.tx16, def->tx16, sizeof(probs->head.tx16));
	memcpy(probs->head.tx8, def->tx8, sizeof(probs->head.tx8));
	memcpy(probs->head.is_inter, def->is_inter, sizeof(probs->head.is_inter));

	memcpy(probs->y_mode, def->y_mode, sizeof(probs->y_mode));
	memcpy(probs->comp_mode, def->comp_mode, sizeof(probs->comp_mode));
	memcpy(probs->comp_ref, def->comp_ref, sizeof(probs->comp_ref));
	memcpy(probs->single_ref, def->single_ref, sizeof(probs->single_ref));
	memcpy(probs->inter_mode, def->inter_mode, sizeof(probs->inter_mode));
	memcpy(probs->interp_filter, def->interp_filter,
	       sizeof(probs->interp_filter));

	for (i = 0; i < ARRAY_SIZE(def->coef); i++)
		for (j = 0; j < ARRAY_SIZE(def->coef[0]); j++)
			for (k = 0; k < ARRAY_SIZE(def->coef[0][0]); k++)
				write_coeff_plane(def->coef[i][j][k],
						  probs->coef[k][i][j]);

	for (i = 0; i < ARRAY_SIZE(def->uv_mode); i++)
		memcpy(&probs->uv_mode[i / 3][(i % 3) * 9], def->uv_mode[i],
		       sizeof(def->uv_mode[0]));

	probs->mv = def->mv;
}

static void config_ref_registers(struct rkvdec_ctx *ctx,
				 const struct rkvdec_vp9_run *run,
				 struct rkvdec_decoded_buffer *ref_buf,
				 int i)
{
	unsigned int aligned_pitch, aligned_height, y_len;
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct rkvdec_vdpu381_regs_vp9 *regs = &vp9_ctx->regs;

	/*
	 * Every CAPTURE buffer has the layout of the decoded format, whatever
	 * the size of the frame stored in it, so plane offsets come from the
	 * format while the frame size registers use the frame's own size.
	 */
	aligned_height = round_up(ctx->decoded_fmt.fmt.pix_mp.height, 64);

	switch (i) {
	case 0:
		regs->vp9_param.reg107.vp9_frameheight_last = ref_buf->vp9.height;
		regs->vp9_param.reg106.vp9_framewidth_last = ref_buf->vp9.width;
		break;
	case 1:
		regs->vp9_param.reg109.vp9_frameheight_golden = ref_buf->vp9.height;
		regs->vp9_param.reg108.vp9_framewidth_golden = ref_buf->vp9.width;
		break;
	case 2:
		regs->vp9_param.reg111.vp9_frameheight_altref = ref_buf->vp9.height;
		regs->vp9_param.reg110.vp9_framewidth_altref = ref_buf->vp9.width;
		break;
	}

	switch (i) {
	case 0:
		regs->vp9_addr.vp9_referlast_base =
			vb2_dma_contig_plane_dma_addr(&ref_buf->base.vb.vb2_buf, 0);
		break;
	case 1:
		regs->vp9_addr.vp9_refergolden_base =
			vb2_dma_contig_plane_dma_addr(&ref_buf->base.vb.vb2_buf, 0);
		break;
	case 2:
		regs->vp9_addr.vp9_referalfter_base =
			vb2_dma_contig_plane_dma_addr(&ref_buf->base.vb.vb2_buf, 0);
		break;
	}

	if (&ref_buf->base.vb == run->base.bufs.dst)
		return;

	/* Real buffer stride, not display-width-derived (see config_registers). */
	aligned_pitch = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	y_len = aligned_height * aligned_pitch;

	switch (i) {
	case 0:
		regs->vp9_param.reg79.vp9_lastfy_hor_virstride = aligned_pitch / 16;
		regs->vp9_param.reg80.vp9_lastfuv_hor_virstride = aligned_pitch / 16;
		regs->vp9_param.reg85.vp9_lastfy_virstride = y_len / 16;
		break;
	case 1:
		regs->vp9_param.reg81.vp9_goldenfy_hor_virstride = aligned_pitch / 16;
		regs->vp9_param.reg82.vp9_goldenuv_hor_virstride = aligned_pitch / 16;
		regs->vp9_param.reg86.vp9_goldeny_virstride = y_len / 16;
		break;
	case 2:
		regs->vp9_param.reg83.vp9_altreffy_hor_virstride = aligned_pitch / 16;
		regs->vp9_param.reg84.vp9_altreff_uv_hor_virstride = aligned_pitch / 16;
		regs->vp9_param.reg87.vp9_altrefy_virstride = y_len / 16;
		break;
	}
}

static void config_seg_registers(struct rkvdec_ctx *ctx, unsigned int segid)
{
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct rkvdec_vdpu381_regs_vp9 *regs = &vp9_ctx->regs;
	const struct v4l2_vp9_segmentation *seg;
	s16 feature_val;
	int feature_id;

	seg = vp9_ctx->last.valid ? &vp9_ctx->last.seg : &vp9_ctx->cur.seg;
	feature_id = V4L2_VP9_SEG_LVL_ALT_Q;
	if (v4l2_vp9_seg_feat_enabled(seg->feature_enabled, feature_id, segid)) {
		feature_val = seg->feature_data[segid][feature_id];
		regs->vp9_param.reg67_74[segid].vp9_segid_frame_qp_delta_en = 1;
		regs->vp9_param.reg67_74[segid].vp9_segid_frame_qp_delta = feature_val;
	}

	feature_id = V4L2_VP9_SEG_LVL_ALT_L;
	if (v4l2_vp9_seg_feat_enabled(seg->feature_enabled, feature_id, segid)) {
		feature_val = seg->feature_data[segid][feature_id];
		regs->vp9_param.reg67_74[segid].vp9_segid_frame_loopfilter_value_en = 1;
		regs->vp9_param.reg67_74[segid].vp9_segid_frame_loopfilter_value = feature_val;
	}

	feature_id = V4L2_VP9_SEG_LVL_REF_FRAME;
	if (v4l2_vp9_seg_feat_enabled(seg->feature_enabled, feature_id, segid)) {
		feature_val = seg->feature_data[segid][feature_id];
		regs->vp9_param.reg67_74[segid].vp9_segid_referinfo_en = 1;
		regs->vp9_param.reg67_74[segid].vp9_segid_referinfo = feature_val;
	}

	feature_id = V4L2_VP9_SEG_LVL_SKIP;
	regs->vp9_param.reg67_74[segid].vp9_segid_frame_skip_en =
		v4l2_vp9_seg_feat_enabled(seg->feature_enabled, feature_id, segid);

	regs->vp9_param.reg67_74[segid].vp9_segid_abs_delta = !segid &&
		(seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE);
}

static void update_ctx_cur_info(struct rkvdec_vp9_ctx *vp9_ctx,
				struct rkvdec_decoded_buffer *buf,
				const struct v4l2_ctrl_vp9_frame *dec_params)
{
	vp9_ctx->cur.valid = true;
	vp9_ctx->cur.reference_mode = dec_params->reference_mode;
	vp9_ctx->cur.interpolation_filter = dec_params->interpolation_filter;
	vp9_ctx->cur.flags = dec_params->flags;
	vp9_ctx->cur.width = dec_params->frame_width_minus_1 + 1;
	vp9_ctx->cur.height = dec_params->frame_height_minus_1 + 1;
	vp9_ctx->cur.timestamp = buf->base.vb.vb2_buf.timestamp;
	vp9_ctx->cur.seg = dec_params->seg;
	vp9_ctx->cur.lf = dec_params->lf;
}

static void rkvdec_write_regs(struct rkvdec_ctx *ctx)
{
	struct rkvdec_core *core = ctx->core;
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;

	rkvdec_memcpy_toio(core->regs + OFFSET_COMMON_REGS,
			   &vp9_ctx->regs.common,
			   sizeof(vp9_ctx->regs.common));
	rkvdec_memcpy_toio(core->regs + OFFSET_CODEC_PARAMS_REGS,
			   &vp9_ctx->regs.vp9_param,
			   sizeof(vp9_ctx->regs.vp9_param));
	rkvdec_memcpy_toio(core->regs + OFFSET_COMMON_ADDR_REGS,
			   &vp9_ctx->regs.common_addr,
			   sizeof(vp9_ctx->regs.common_addr));
	rkvdec_memcpy_toio(core->regs + OFFSET_CODEC_ADDR_REGS,
			   &vp9_ctx->regs.vp9_addr,
			   sizeof(vp9_ctx->regs.vp9_addr));
}

static dma_addr_t get_mv_base_addr(struct rkvdec_ctx *ctx, struct rkvdec_decoded_buffer *buf)
{
	unsigned int aligned_pitch, aligned_height, yuv_len;

	aligned_height = round_up(ctx->decoded_fmt.fmt.pix_mp.height, 64);

	aligned_pitch = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	yuv_len = (aligned_height * aligned_pitch * 3) / 2;

	return vb2_dma_contig_plane_dma_addr(&buf->base.vb.vb2_buf, 0) +
	       yuv_len;
}

/*
 * Selects the frame context the hardware reads the probabilities from and
 * saves them to, see 6.1.2 refresh_probs() and the frame context handling of
 * 6.2 uncompressed_header() (setup_past_independence()) in the spec.
 */
static void config_prob_registers(struct rkvdec_ctx *ctx,
				  const struct rkvdec_vp9_run *run,
				  bool intra_only)
{
	const struct v4l2_ctrl_vp9_frame *dec_params = run->decode_params;
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct rkvdec_vdpu381_regs_vp9 *regs = &vp9_ctx->regs;
	dma_addr_t ctx_probs = vp9_ctx->stream_tbl.dma +
			       offsetof(struct vdpu381_vp9_stream_tbl, ctx_probs);
	struct rkvdec_aux_buf *job_buf = &vp9_ctx->job_tbl[ctx->job->slot];
	bool error_resilient = dec_params->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT;
	bool parallel = dec_params->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE;
	bool refresh = dec_params->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX;
	unsigned int fctx_idx = dec_params->frame_context_idx;
	unsigned int i;

	/*
	 * setup_past_independence(): the reset contexts hold the default
	 * probabilities until the hardware saves new ones to them. The
	 * frame then uses context 0, which isn't reset by an intra-only
	 * frame with reset_frame_context 0 or 1.
	 */
	if (intra_only || error_resilient) {
		if (dec_params->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME ||
		    error_resilient ||
		    dec_params->reset_frame_context == V4L2_VP9_RESET_FRAME_CTX_ALL) {
			for (i = 0; i < VDPU381_VP9_NUM_FRAME_CTXS; i++)
				vp9_ctx->ctx_saved[i] = false;
		} else if (dec_params->reset_frame_context ==
			   V4L2_VP9_RESET_FRAME_CTX_SPEC) {
			vp9_ctx->ctx_saved[fctx_idx] = false;
		}
		fctx_idx = 0;
	}
	vp9_ctx->cur.frame_context_idx = fctx_idx;

	/*
	 * The probability buffers are told apart by index + 1 in the
	 * multi-core control register, 0 for the default table.
	 */
	if (vp9_ctx->ctx_saved[fctx_idx]) {
		regs->vp9_addr.vp9_last_prob_base = ctx_probs +
			fctx_idx * sizeof(union vdpu381_vp9_prob_tbl);
		regs->common.reg028_multiply_core_ctrl.vp9_rd_prob_idx = fctx_idx + 1;
	} else {
		regs->vp9_addr.vp9_last_prob_base = vp9_ctx->stream_tbl.dma +
			offsetof(struct vdpu381_vp9_stream_tbl, default_probs);
		regs->common.reg028_multiply_core_ctrl.vp9_rd_prob_idx = 0;
	}

	regs->vp9_addr.vp9_update_prob_wr_bas = ctx_probs +
		fctx_idx * sizeof(union vdpu381_vp9_prob_tbl);
	regs->common.reg028_multiply_core_ctrl.vp9_wr_prob_idx = fctx_idx + 1;
	if (refresh)
		vp9_ctx->ctx_saved[fctx_idx] = true;

	regs->vp9_addr.vp9_delta_prob_base = job_buf->dma +
		offsetof(struct vdpu381_vp9_job_tbl, updates);
	regs->vp9_addr.vp9_count_base = job_buf->dma +
		offsetof(struct vdpu381_vp9_job_tbl, counts);

	regs->vp9_param.reg103.vp9_prob_update_en = 1;
	regs->vp9_param.reg103.vp9_intra_only_flag = intra_only;
	if (!intra_only) {
		regs->vp9_param.reg103.vp9_txfmmode_rfsh_en =
			vp9_ctx->cur.tx_mode == V4L2_VP9_TX_MODE_SELECT;
		regs->vp9_param.reg103.vp9_interp_filter_switch_en =
			dec_params->interpolation_filter ==
			V4L2_VP9_INTERP_FILTER_SWITCHABLE;
	}
	regs->vp9_param.reg103.vp9_ref_mode_rfsh_en = 1;
	regs->vp9_param.reg103.vp9_single_ref_rfsh_en = 1;
	regs->vp9_param.reg103.vp9_comp_ref_rfsh_en = 1;
	/* Backward adaptation, then save_probs() of the adapted probabilities */
	regs->vp9_param.reg103.vp9_refresh_en = !error_resilient && !parallel;
	regs->vp9_param.reg103.vp9_prob_save_en = refresh;
	regs->vp9_param.reg103.vp9_allow_high_precision_mv =
		!!(dec_params->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV);
	/* Selects the coefficient update factor (spec 8.4.3) */
	regs->vp9_param.reg103.vp9_last_key_frame_flag =
		vp9_ctx->last.valid &&
		(vp9_ctx->last.flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME);
}

static void config_registers(struct rkvdec_ctx *ctx,
			     const struct rkvdec_vp9_run *run)
{
	unsigned int y_len, aligned_height, aligned_pitch, stream_len;
	const struct v4l2_ctrl_vp9_frame *dec_params;
	struct rkvdec_decoded_buffer *ref_bufs[3];
	struct rkvdec_decoded_buffer *dst, *last, *mv_ref;
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct rkvdec_vdpu381_regs_vp9 *regs = &vp9_ctx->regs;
	dma_addr_t segmap_base = vp9_ctx->stream_tbl.dma +
				 offsetof(struct vdpu381_vp9_stream_tbl, segmap);
	const struct v4l2_vp9_segmentation *seg;
	u32 pixels;

	dma_addr_t rlc_addr, dst_addr;
	bool seg_enabled, segmap_update, last_segmap_valid;
	bool intra_only;
	unsigned int i;

	dec_params = run->decode_params;
	dst = vb2_to_rkvdec_decoded_buf(&run->base.bufs.dst->vb2_buf);
	ref_bufs[0] = get_ref_buf_vp9(ctx, &dst->base.vb, dec_params->last_frame_ts);
	ref_bufs[1] = get_ref_buf_vp9(ctx, &dst->base.vb, dec_params->golden_frame_ts);
	ref_bufs[2] = get_ref_buf_vp9(ctx, &dst->base.vb, dec_params->alt_frame_ts);

	if (vp9_ctx->last.valid)
		last = get_ref_buf_vp9(ctx, &dst->base.vb, vp9_ctx->last.timestamp);
	else
		last = dst;

	update_dec_buf_info(dst, dec_params);
	update_ctx_cur_info(vp9_ctx, dst, dec_params);
	seg = &dec_params->seg;

	intra_only = !!(dec_params->flags &
			(V4L2_VP9_FRAME_FLAG_KEY_FRAME |
			 V4L2_VP9_FRAME_FLAG_INTRA_ONLY));

	/*
	 * The register set is kept in the context and only partially written
	 * below: start from a clean set so that nothing leaks from the
	 * previous frame.
	 */
	memset(regs, 0, sizeof(*regs));

	regs->common.reg009_dec_mode.dec_mode = VDPU381_MODE_VP9;
	rkvdec_vdpu381_set_ccu_regs(ctx, &regs->common);

	config_prob_registers(ctx, run, intra_only);

	/* Set config */
	regs->common.reg011_important_en.buf_empty_en = 1;
	regs->common.reg011_important_en.dec_clkgate_e = 1;
	regs->common.reg011_important_en.dec_timeout_e = 1;

	aligned_height = round_up(ctx->decoded_fmt.fmt.pix_mp.height, 64);

	/* use capture buffer's bytesperline as hw stride, same as HEVC. */
	aligned_pitch = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	y_len = aligned_height * aligned_pitch;

	pixels = ctx->decoded_fmt.fmt.pix_mp.height * ctx->decoded_fmt.fmt.pix_mp.width;

	regs->common.reg018_y_hor_stride.y_hor_virstride = aligned_pitch / 16;
	regs->common.reg019_uv_hor_stride.uv_hor_virstride = aligned_pitch / 16;
	regs->common.reg020_y_stride.y_virstride = y_len / 16;

	stream_len = vb2_get_plane_payload(&run->base.bufs.src->vb2_buf, 0);

	regs->common.reg016_stream_len = stream_len;

	/* Activate block gating */
	regs->common.reg026_block_gating_en.inter_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.filterd_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.strmd_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.mcp_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.busifd_auto_gating_e = 0;
	regs->common.reg026_block_gating_en.dec_ctrl_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.intra_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.mc_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.transd_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.sram_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.cru_auto_gating_e = 1;
	regs->common.reg026_block_gating_en.reg_cfg_gating_en = 1;

	/* Set timeout threshold */
	if (pixels < RKVDEC_1080P_PIXELS)
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_1080p;
	else if (pixels < RKVDEC_4K_PIXELS)
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_4K;
	else if (pixels < RKVDEC_8K_PIXELS)
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_8K;
	else
		regs->common.reg032_timeout_threshold = RKVDEC_TIMEOUT_MAX;

	/*
	 * Two segmentation maps are used in turn: the hardware reads the
	 * segment IDs left by the previous frames from segidlast and writes
	 * the ones of the current frame to segidcur. A frame that does not
	 * code a new map (segmentation disabled, or update_map == 0) leaves
	 * the previous map in place for the following frames, as required by
	 * the spec. Intra-only, error resilient and resized frames start
	 * from a reset map.
	 */
	seg_enabled = seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED;
	segmap_update = intra_only || !vp9_ctx->last.valid ||
			(dec_params->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) ||
			vp9_ctx->last.width != vp9_ctx->cur.width ||
			vp9_ctx->last.height != vp9_ctx->cur.height ||
			(seg_enabled &&
			 (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP));
	vp9_ctx->cur.segmapid = vp9_ctx->last.segmapid ^ segmap_update;
	regs->vp9_param.reg75.vp9_segment_id_update = segmap_update;

	/*
	 * The map left by the previous frames is only meaningful if
	 * segmentation was used since the last intra frame.
	 */
	last_segmap_valid = !intra_only && vp9_ctx->last.valid &&
			    vp9_ctx->last.segmap_valid;
	vp9_ctx->cur.segmap_valid = seg_enabled || last_segmap_valid;

	for (i = 0; i < ARRAY_SIZE(ref_bufs); i++)
		config_ref_registers(ctx, run, ref_bufs[i], i);

	for (i = 0; i < 8; i++)
		config_seg_registers(ctx, i);

	regs->vp9_param.reg76.vp9_tx_mode = vp9_ctx->cur.tx_mode;
	regs->vp9_param.reg76.vp9_frame_reference_mode = dec_params->reference_mode;

	if (!intra_only) {
		const struct v4l2_vp9_loop_filter *lf;

		if (vp9_ctx->last.valid)
			lf = &vp9_ctx->last.lf;
		else
			lf = &vp9_ctx->cur.lf;

		for (i = 0; i < ARRAY_SIZE(lf->ref_deltas); i++)
			regs->vp9_param.reg94.vp9_ref_deltas_lastframe |=
				(lf->ref_deltas[i] & 0x7f) << (7 * i);

		for (i = 0; i < ARRAY_SIZE(lf->mode_deltas); i++)
			regs->vp9_param.reg75.vp9_mode_deltas_lastframe |=
				(lf->mode_deltas[i] & 0x7f) << (7 * i);
	}

	regs->vp9_param.reg75.segmentation_enable_lstframe = last_segmap_valid;

	regs->vp9_param.reg75.vp9_last_showframe =
		vp9_ctx->last.valid &&
		vp9_ctx->last.flags & V4L2_VP9_FRAME_FLAG_SHOW_FRAME;

	regs->vp9_param.reg75.vp9_last_intra_only =
		vp9_ctx->last.valid &&
		vp9_ctx->last.flags &
		(V4L2_VP9_FRAME_FLAG_KEY_FRAME | V4L2_VP9_FRAME_FLAG_INTRA_ONLY);

	regs->vp9_param.reg75.vp9_last_widhheight_eqcur =
		vp9_ctx->last.valid &&
		vp9_ctx->last.width == vp9_ctx->cur.width &&
		vp9_ctx->last.height == vp9_ctx->cur.height;

	regs->vp9_param.reg78_vp9_stream_size = stream_len;

	for (i = 0; !intra_only && i < ARRAY_SIZE(ref_bufs); i++) {
		unsigned int refw = ref_bufs[i]->vp9.width;
		unsigned int refh = ref_bufs[i]->vp9.height;
		u32 hscale, vscale;

		hscale = (refw << 14) /	dst->vp9.width;
		vscale = (refh << 14) / dst->vp9.height;

		switch (i) {
		case 0:
			regs->vp9_param.reg88.vp9_lref_hor_scale = hscale;
			regs->vp9_param.reg89.vp9_lref_ver_scale = vscale;
			break;
		case 1:
			regs->vp9_param.reg90.vp9_gref_hor_scale = hscale;
			regs->vp9_param.reg91.vp9_gref_ver_scale = vscale;
			break;
		case 2:
			regs->vp9_param.reg92.vp9_aref_hor_scale = hscale;
			regs->vp9_param.reg93.vp9_aref_ver_scale = vscale;
			break;
		}
	}

	/* Set rlc base address (input stream) */
	rlc_addr = vb2_dma_contig_plane_dma_addr(&run->base.bufs.src->vb2_buf, 0);
	regs->common_addr.rlc_base = rlc_addr;
	regs->common_addr.rlcwrite_base = rlc_addr;

	/* Set output base address */
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst->base.vb.vb2_buf, 0);
	regs->common_addr.decout_base = dst_addr;
	regs->common_addr.error_ref_base = dst_addr;

	/* Set colmv address */
	regs->common_addr.colmv_cur_base = dst_addr + ctx->colmv_offset;

	/* Set RCB addresses */
	for (i = 0; i < rkvdec_rcb_buf_count(ctx); i++)
		regs->common_addr.rcb_base[i] = rkvdec_rcb_buf_dma_addr(ctx, i);

	regs->vp9_addr.vp9_segidlast_base = segmap_base +
		RKVDEC_VP9_MAX_SEGMAP_SIZE * vp9_ctx->last.segmapid;
	regs->vp9_addr.avp9_segidcur_base = segmap_base +
		RKVDEC_VP9_MAX_SEGMAP_SIZE * !vp9_ctx->last.segmapid;

	if (!intra_only &&
	    !(dec_params->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) &&
	    vp9_ctx->last.valid)
		mv_ref = last;
	else
		mv_ref = dst;

	regs->vp9_addr.vp9_refcolmv_base = get_mv_base_addr(ctx, mv_ref);

	rkvdec_write_regs(ctx);
}

static int validate_dec_params(struct rkvdec_ctx *ctx,
			       const struct v4l2_ctrl_vp9_frame *dec_params)
{
	unsigned int aligned_width, aligned_height;

	aligned_width = round_up(dec_params->frame_width_minus_1 + 1, 64);
	aligned_height = round_up(dec_params->frame_height_minus_1 + 1, 64);

	/*
	 * VP9 can change resolution on inter frames, which then predict from
	 * references of the previous size. Frames that fit in the allocated
	 * CAPTURE buffers are decoded in place so that those references stay
	 * available; userspace must reallocate only when a frame is larger.
	 */
	if (aligned_width > ctx->decoded_fmt.fmt.pix_mp.width ||
	    aligned_height > ctx->decoded_fmt.fmt.pix_mp.height) {
		dev_err(ctx->core->dev,
			"unexpected bitstream resolution %dx%d\n",
			dec_params->frame_width_minus_1 + 1,
			dec_params->frame_height_minus_1 + 1);
		return -EINVAL;
	}

	return 0;
}

static int rkvdec_vp9_run_preamble(struct rkvdec_ctx *ctx,
				   struct rkvdec_vp9_run *run)
{
	const struct v4l2_ctrl_vp9_frame *dec_params;
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct v4l2_ctrl *ctrl;
	int ret;

	/* v4l2-specific stuff */
	rkvdec_run_preamble(ctx, &run->base);

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_VP9_FRAME);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	dec_params = ctrl->p_cur.p;

	ret = validate_dec_params(ctx, dec_params);
	if (ret)
		return ret;

	run->decode_params = dec_params;

	/*
	 * Userspace parsed the compressed header (spec 6.3): the hardware
	 * applies the probability updates itself.
	 */
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP9_COMPRESSED_HDR);
	if (WARN_ON(!ctrl))
		return -EINVAL;
	run->prob_updates = ctrl->p_cur.p;
	vp9_ctx->cur.tx_mode = run->prob_updates->tx_mode;

	return 0;
}

static int rkvdec_vp9_run(struct rkvdec_ctx *ctx)
{
	struct rkvdec_core *core = ctx->core;
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct rkvdec_vp9_run run = { };
	bool intra_only;
	int ret;

	ret = rkvdec_vp9_run_preamble(ctx, &run);

	if (ret) {
		rkvdec_run_postamble(ctx, &run.base);

		return ret;
	}

	intra_only = !!(run.decode_params->flags &
			(V4L2_VP9_FRAME_FLAG_KEY_FRAME |
			 V4L2_VP9_FRAME_FLAG_INTRA_ONLY));

	/* Prepare the probability updates. */
	vdpu381_vp9_write_updates(ctx, &run, intra_only);

	/* Configure hardware registers. */
	config_registers(ctx, &run);

	/*
	 * Everything the next frame needs from this one is known now, the
	 * hardware keeps the probabilities and segment IDs.
	 */
	vp9_ctx->last = vp9_ctx->cur;

	rkvdec_run_postamble(ctx, &run.base);

	rkvdec_schedule_watchdog(core, vp9_ctx->regs.common.reg032_timeout_threshold);

	rkvdec_ccu_start(ctx);
	writel(VDPU381_DEC_E_BIT, core->regs + VDPU381_REG_DEC_E);

	return 0;
}

static int rkvdec_vp9_validate_frame(const struct v4l2_ctrl_vp9_frame *frame)
{
	if (frame->profile == 1 || frame->profile == 3)
		return -EINVAL;

	if (frame->bit_depth != 8 && frame->bit_depth != 10)
		return -EINVAL;

	return 0;
}

static int rkvdec_vp9_try_ctrl(struct rkvdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_VP9_FRAME)
		return rkvdec_vp9_validate_frame(ctrl->p_new.p_vp9_frame);

	return 0;
}

static void rkvdec_vp9_free_tbls(struct rkvdec_ctx *ctx)
{
	struct rkvdec_vp9_ctx *vp9_ctx = ctx->priv;
	struct device *dev = ctx->dev->main_core->dev;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vp9_ctx->job_tbl); i++) {
		struct rkvdec_aux_buf *buf = &vp9_ctx->job_tbl[i];

		if (buf->cpu)
			dma_free_coherent(dev, buf->size, buf->cpu, buf->dma);
	}

	if (vp9_ctx->stream_tbl.cpu)
		dma_free_coherent(dev, vp9_ctx->stream_tbl.size,
				  vp9_ctx->stream_tbl.cpu, vp9_ctx->stream_tbl.dma);
}

static int rkvdec_vp9_alloc_tbl(struct rkvdec_ctx *ctx,
				struct rkvdec_aux_buf *buf, size_t size)
{
	buf->cpu = dma_alloc_coherent(ctx->dev->main_core->dev, size, &buf->dma,
				      GFP_KERNEL);
	if (!buf->cpu)
		return -ENOMEM;

	buf->size = size;
	return 0;
}

static int rkvdec_vp9_start(struct rkvdec_ctx *ctx)
{
	struct vdpu381_vp9_stream_tbl *stream_tbl;
	struct rkvdec_vp9_ctx *vp9_ctx;
	struct v4l2_ctrl *ctrl;
	unsigned int i;
	int ret;

	BUILD_BUG_ON(sizeof(struct vdpu381_vp9_probs_head) != 5 * 16);
	BUILD_BUG_ON(sizeof(struct vdpu381_vp9_frame_probs) != 153 * 16);
	BUILD_BUG_ON(sizeof(struct vdpu381_vp9_inter_updates) > VDPU381_VP9_PROB_TBL_SIZE);
	BUILD_BUG_ON(sizeof(struct vdpu381_vp9_intra_updates) > VDPU381_VP9_PROB_TBL_SIZE);

	/* frame header */
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP9_FRAME);
	if (!ctrl)
		return -EINVAL;

	ret = rkvdec_vp9_validate_frame(ctrl->p_new.p_vp9_frame);
	if (ret)
		return ret;

	vp9_ctx = kzalloc_obj(*vp9_ctx);
	if (!vp9_ctx)
		return -ENOMEM;

	ctx->priv = vp9_ctx;

	ret = rkvdec_vp9_alloc_tbl(ctx, &vp9_ctx->stream_tbl, sizeof(*stream_tbl));
	if (ret)
		goto err_free;

	stream_tbl = vp9_ctx->stream_tbl.cpu;
	vdpu381_vp9_write_default_probs(&stream_tbl->default_probs.probs);

	for (i = 0; i < ctx->num_job_slots; i++) {
		ret = rkvdec_vp9_alloc_tbl(ctx, &vp9_ctx->job_tbl[i],
					   sizeof(struct vdpu381_vp9_job_tbl));
		if (ret)
			goto err_free;
	}

	return 0;

err_free:
	rkvdec_vp9_free_tbls(ctx);
	kfree(vp9_ctx);
	return ret;
}

static void rkvdec_vp9_stop(struct rkvdec_ctx *ctx)
{
	rkvdec_vp9_free_tbls(ctx);
	kfree(ctx->priv);
}

static int rkvdec_vp9_adjust_fmt(struct rkvdec_ctx *ctx,
				 struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *fmt = &f->fmt.pix_mp;

	fmt->num_planes = 1;
	if (!fmt->plane_fmt[0].sizeimage)
		fmt->plane_fmt[0].sizeimage = fmt->width * fmt->height * 2;
	return 0;
}

static enum rkvdec_image_fmt
rkvdec_vp9_get_image_fmt(struct rkvdec_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_vp9_frame *frame = ctrl->p_new.p_vp9_frame;

	if (ctrl->id != V4L2_CID_STATELESS_VP9_FRAME)
		return RKVDEC_IMG_FMT_ANY;

	switch (frame->bit_depth) {
	case 8:
		return RKVDEC_IMG_FMT_420_8BIT;
	case 10:
		return RKVDEC_IMG_FMT_420_10BIT;
	default:
		return RKVDEC_IMG_FMT_ANY;
	}
}

const struct rkvdec_coded_fmt_ops rkvdec_vdpu381_vp9_fmt_ops = {
	.adjust_fmt = rkvdec_vp9_adjust_fmt,
	.start = rkvdec_vp9_start,
	.stop = rkvdec_vp9_stop,
	.run = rkvdec_vp9_run,
	.try_ctrl = rkvdec_vp9_try_ctrl,
	.get_image_fmt = rkvdec_vp9_get_image_fmt,
};
