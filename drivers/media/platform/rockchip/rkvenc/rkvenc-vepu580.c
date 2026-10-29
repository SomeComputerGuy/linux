// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip VEPU580 (RKVENC) common frame programming
 *
 * The register meanings and the default tuning values below were derived
 * from the Rockchip MPP user space library (hal_h264e_vepu580.c,
 * vepu5xx_common.c, hal_h264e_vepu580_tune.c), which is the only
 * documentation of this block. The values are expressed as named fields
 * or as formulas instead of register dumps.
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/minmax.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include "rkvenc.h"

unsigned int rkvenc_row_rc_qp_range = 2;
module_param_named(row_rc_qp_range, rkvenc_row_rc_qp_range, uint, 0644);
MODULE_PARM_DESC(row_rc_qp_range,
		 "CBR: maximum per-row QP change by the hardware rate control (0 = frame-level rate control only)");

int rkvenc_alloc_aux(struct rkvenc_dev *rkvenc, struct rkvenc_aux_buf *buf,
		     size_t size, bool kernel_mapping)
{
	struct device *dev = rkvenc->main_core->dev;

	buf->attrs = kernel_mapping ? 0 : DMA_ATTR_NO_KERNEL_MAPPING;
	buf->size = size;
	buf->cpu = dma_alloc_attrs(dev, size, &buf->dma, GFP_KERNEL, buf->attrs);
	if (!buf->cpu)
		return -ENOMEM;

	return 0;
}

void rkvenc_free_aux(struct rkvenc_dev *rkvenc, struct rkvenc_aux_buf *buf)
{
	if (!buf->cpu)
		return;

	dma_free_attrs(rkvenc->main_core->dev, buf->size, buf->cpu, buf->dma,
		       buf->attrs);
	buf->cpu = NULL;
}

dma_addr_t rkvenc_dst_addr(struct rkvenc_run *run)
{
	return vb2_dma_contig_plane_dma_addr(&run->dst->vb2_buf, 0);
}

size_t rkvenc_dst_size(struct rkvenc_run *run)
{
	return vb2_plane_size(&run->dst->vb2_buf, 0);
}

static void vepu580_setup_ctl(struct rkvenc_core *core)
{
	/*
	 * Bitstream writes are byte swapped within 64 bits (bsw_bus_edin = 7);
	 * every other port uses the natural order.
	 */
	rkvenc_set(core, VEPU580_DTRNS_MAP,
		   FIELD_PREP(VEPU580_DTRNS_MAP_BSW_BUS_EDIN, 7) |
		   VEPU580_DTRNS_MAP_AFBC_BSIZE);
	rkvenc_set(core, VEPU580_DTRNS_CFG, VEPU580_DTRNS_CFG_DSPR_OTSD);
	rkvenc_set(core, VEPU580_FUNC_EN,
		   VEPU580_FUNC_EN_CKE | VEPU580_FUNC_EN_RESETN_HW_EN |
		   VEPU580_FUNC_EN_ENC_DONE_TMVP_EN);
}

static void vepu580_setup_src(struct rkvenc_ctx *ctx, struct rkvenc_core *core,
			      struct rkvenc_run *run)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->src_fmt.fmt.pix_mp;
	const struct rkvenc_raw_fmt *fmt = ctx->raw_fmt;
	const struct v4l2_format_info *info = v4l2_format_info(pix->pixelformat);
	struct vb2_buffer *vb = &run->src->vb2_buf;
	unsigned int width = ctx->coded_fmt.fmt.pix_mp.width;
	unsigned int height = ctx->coded_fmt.fmt.pix_mp.height;
	unsigned int align = ctx->coded_fmt_desc->size_align;
	unsigned int ystride = pix->plane_fmt[0].bytesperline;
	unsigned int cstride = 0;
	dma_addr_t addr[3] = { 0 };
	unsigned int i;

	rkvenc_set(core, VEPU580_ENC_RSL,
		   FIELD_PREP(VEPU580_ENC_RSL_PIC_WD8_M1, ALIGN(width, align) / 8 - 1) |
		   FIELD_PREP(VEPU580_ENC_RSL_PIC_HD8_M1, ALIGN(height, align) / 8 - 1));
	rkvenc_set(core, VEPU580_SRC_FILL,
		   FIELD_PREP(VEPU580_SRC_FILL_PIC_WFILL, ALIGN(width, align) - width) |
		   FIELD_PREP(VEPU580_SRC_FILL_PIC_HFILL, ALIGN(height, align) - height));

	/* MPP sets src_range for every YUV input; out_fmt selects 4:2:0. */
	rkvenc_set(core, VEPU580_SRC_FMT,
		   FIELD_PREP(VEPU580_SRC_FMT_SRC_CFMT, fmt->hw_fmt) |
		   (fmt->rbuv_swap ? VEPU580_SRC_FMT_RBUV_SWAP : 0) |
		   VEPU580_SRC_FMT_SRC_RANGE | VEPU580_SRC_FMT_OUT_FMT);

	/* BT.601 limited range RGB to YUV, unused for YUV input. */
	rkvenc_set(core, VEPU580_SRC_UDFY,
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_R, 66) |
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_G, 129) |
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_B, 25));
	rkvenc_set(core, VEPU580_SRC_UDFU,
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_R, -38 & 0x1ff) |
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_G, -74 & 0x1ff) |
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_B, 112));
	rkvenc_set(core, VEPU580_SRC_UDFV,
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_R, 112) |
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_G, -94 & 0x1ff) |
		   FIELD_PREP(VEPU580_SRC_UDF_WGT_B, -18 & 0x1ff));
	rkvenc_set(core, VEPU580_SRC_UDFO,
		   FIELD_PREP(VEPU580_SRC_UDFO_OFST_Y, 16) |
		   FIELD_PREP(VEPU580_SRC_UDFO_OFST_U, 128) |
		   FIELD_PREP(VEPU580_SRC_UDFO_OFST_V, 128));

	/* Plane addresses */
	for (i = 0; i < pix->num_planes; i++)
		addr[i] = vb2_dma_contig_plane_dma_addr(vb, i);

	if (info->comp_planes > 1) {
		if (pix->num_planes > 1)
			cstride = pix->plane_fmt[1].bytesperline;
		else if (info->comp_planes == 2)
			cstride = ystride * info->bpp[1] / info->hdiv;
		else
			cstride = ystride / info->hdiv;
	}

	if (pix->num_planes == 1 && info->comp_planes > 1) {
		addr[1] = addr[0] + ystride * pix->height;
		if (info->comp_planes > 2)
			addr[2] = addr[1] + cstride * pix->height / info->vdiv;
	}

	rkvenc_set(core, VEPU580_ADR_SRC0, addr[0]);
	rkvenc_set(core, VEPU580_ADR_SRC1, addr[1]);
	rkvenc_set(core, VEPU580_ADR_SRC2, addr[2]);
	rkvenc_set(core, VEPU580_SRC_STRD0, FIELD_PREP(VEPU580_SRC_STRD0_STRD, ystride));
	rkvenc_set(core, VEPU580_SRC_STRD1, FIELD_PREP(VEPU580_SRC_STRD1_STRD, cstride));
	rkvenc_set(core, VEPU580_PIC_OFST, 0);
	rkvenc_set(core, VEPU580_SRC_PROC, 0);
}

static void vepu580_setup_bitstream(struct rkvenc_core *core,
				    struct rkvenc_run *run)
{
	dma_addr_t base = rkvenc_dst_addr(run);
	size_t size = rkvenc_dst_size(run);

	/*
	 * The bitstream buffer is a ring: bottom/top are its bounds (top is the
	 * address of the last byte), bsbr the read pointer and bsbs the write
	 * start. Software headers occupy the first hdr_bytes bytes.
	 */
	rkvenc_set(core, VEPU580_BSBB_ADDR, base);
	rkvenc_set(core, VEPU580_BSBT_ADDR, base + size - 1);
	rkvenc_set(core, VEPU580_BSBR_ADDR, base);
	rkvenc_set(core, VEPU580_ADR_BSBS, base + run->hdr_bytes);
}

/*
 * Clear the shadow registers and program everything that is not codec
 * specific: control class, source picture, bitstream buffer and slices.
 */
void vepu580_setup_common(struct rkvenc_ctx *ctx, struct rkvenc_core *core,
			  struct rkvenc_run *run)
{
	memset(core->shadow, 0, VEPU580_SHADOW_SIZE);

	vepu580_setup_ctl(core);
	vepu580_setup_src(ctx, core, run);
	vepu580_setup_bitstream(core, run);

	/* Single slice, deblocking across slice boundaries allowed. */
	rkvenc_set(core, VEPU580_SLI_SPLT, 0);
	rkvenc_set(core, VEPU580_SLI_CFG, VEPU580_SLI_CFG_SLI_CRS_EN);

	/* No external line buffer: widths above 4096 are rejected for now. */
	rkvenc_set(core, VEPU580_EBUFT_ADDR, 0);
	rkvenc_set(core, VEPU580_EBUFB_ADDR, 0);
}

/*
 * Size the coarse integer motion estimation (CIME) cache for a search
 * window of +-4*srch_h by +-4*srch_v pixels and a zero global MV.
 */
static void vepu580_setup_cime_cache(struct rkvenc_core *core,
				     unsigned int width_mbs,
				     unsigned int srch_h, unsigned int srch_v)
{
	const unsigned int cache_max = 2464;
	unsigned int s = 4 * srch_h, h = 4 * srch_v;
	unsigned int cols = ALIGN(width_mbs * 16, 64) / 64;
	unsigned int rx = (16 - s % 16) % 16;
	unsigned int ry = (4 - h % 4) % 4;
	unsigned int cur_w, rows0, rama_h, ramb_h;

	cur_w = min(DIV_ROUND_UP(rx + 2 * s, 16) + 1, cols);
	rows0 = DIV_ROUND_UP(ry + 2 * h, 4) + 1;

	rama_h = 1;
	ramb_h = rows0;
	while (rama_h < rows0 &&
	       cache_max > (rama_h - 1) * cols * 4 + ramb_h * 4 * cur_w) {
		rama_h++;
		ramb_h = ramb_h > 2 ? ramb_h - 1 : 1;
	}

	if (rows0 == 1) {
		rama_h++;
		ramb_h = 0;
	}

	if (cache_max < (rama_h - 1) * cols * 4 + ramb_h * 4 * cur_w)
		rama_h--;

	rkvenc_set(core, VEPU580_ME_CACH,
		   FIELD_PREP(VEPU580_ME_CACH_CME_RAMA_MAX,
			      (rama_h - 1) * cols + cur_w) |
		   FIELD_PREP(VEPU580_ME_CACH_CME_RAMA_H, rama_h) |
		   FIELD_PREP(VEPU580_ME_CACH_CME_LINEBUF_W, cols));
}

void vepu580_setup_me(struct rkvenc_core *core, unsigned int width_mbs,
		      unsigned int height_mbs, unsigned int level_idc,
		      bool intra)
{
	unsigned int blk_w = 44, blk_h = 28, hal;
	unsigned int mv_limit;

	/* Level 1.0 / 1b restrict the vertical range (H.264 table A-1). */
	if (level_idc == 9 || level_idc == 10)
		blk_h = 12;

	if (blk_w / 4 * 2 > (width_mbs * 2 + 1) / 2)
		blk_w = (width_mbs * 2 + 1) / 2 / 2 * 4;

	hal = ALIGN(height_mbs * 16, 64);
	if (blk_h / 4 > hal / 128 * 4)
		blk_h = hal / 128 * 16;

	rkvenc_set(core, VEPU580_ME_RNGE,
		   FIELD_PREP(VEPU580_ME_RNGE_CME_SRCH_H, blk_w / 4) |
		   FIELD_PREP(VEPU580_ME_RNGE_CME_SRCH_V, blk_h / 4) |
		   FIELD_PREP(VEPU580_ME_RNGE_RME_SRCH_H, 7) |
		   FIELD_PREP(VEPU580_ME_RNGE_RME_SRCH_V, 5));

	if (level_idc > 20)
		mv_limit = 2;
	else if (level_idc >= 11)
		mv_limit = 1;
	else
		mv_limit = 0;

	rkvenc_set(core, VEPU580_ME_CFG,
		   FIELD_PREP(VEPU580_ME_CFG_PMV_MDST_H, intra ? 0 : 5) |
		   FIELD_PREP(VEPU580_ME_CFG_PMV_MDST_V, intra ? 0 : 5) |
		   FIELD_PREP(VEPU580_ME_CFG_MV_LIMIT, mv_limit) |
		   FIELD_PREP(VEPU580_ME_CFG_PMV_NUM, 2));

	vepu580_setup_cime_cache(core, width_mbs, blk_w / 4, blk_h / 4);
}

/*
 * H.264 RDO lambda per QP, growing by 2^(1/3) per QP step. The hardware
 * table is indexed by QP 0..51 and starts at entry 6 of this list.
 */
static const u32 vepu580_h264_lambda[58] = {
	3, 5, 6, 7, 9, 11, 14, 18, 22, 28, 36, 45, 57, 72, 91, 115, 145, 182,
	230, 290, 365, 460, 580, 731, 921, 1161, 1462, 1843, 2322, 2925, 3686,
	4644, 5851, 7372, 9289, 11703, 14745, 18578, 23407, 29491, 37156,
	46814, 58982, 74313, 93628, 117964, 148626, 187257, 235929, 297252,
	374514, 471859, 594505, 749029, 943718, 1189010, 1498059, 1887436,
};

/*
 * Chroma distortion weights (about 16 * 2^(k/3), saturated to 18 bits),
 * packed three per two registers. Intra frames start at entry 0, inter
 * frames at entry 6.
 */
static const u32 vepu580_chroma_klut[45] = {
	16, 20, 25, 32, 40, 50, 64, 80, 101, 128, 161, 203, 256, 322, 406,
	512, 645, 812, 1024, 1290, 1625, 2048, 2580, 3250, 4096, 5160, 6501,
	8192, 10321, 13003, 16384, 20642, 26007, 32768, 41285, 52015, 65535,
	82570, 104030, 131070, 165141, 208896, 262143, 262143, 0,
};

/* Texture adaptive QP: MADI thresholds and QP steps. */
static const u8 vepu580_aq_tthd[16] = {
	0, 0, 0, 0, 3, 3, 5, 5, 8, 8, 8, 15, 15, 20, 25, 35,
};

static const s8 vepu580_aq_step_i[16] = {
	-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 8,
};

static const s8 vepu580_aq_step_p[16] = {
	-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 7, 8,
};

static u32 pack4(u32 a, u32 b, u32 c, u32 d)
{
	return (a & 0xff) | (b & 0xff) << 8 | (c & 0xff) << 16 |
	       (d & 0xff) << 24;
}

static u32 pack2x12(u32 lo, u32 hi)
{
	return (lo & 0xfff) | (hi & 0xfff) << 16;
}

static void vepu580_setup_klut(struct rkvenc_core *core, bool intra)
{
	const u32 *w = &vepu580_chroma_klut[intra ? 0 : 6];
	unsigned int i;

	rkvenc_set(core, VEPU580_KLUT_OFST, intra ? 0 : 3);

	for (i = 0; i < VEPU580_KLUT_WGT_NUM; i += 2) {
		const u32 *t = &w[i / 2 * 3];

		rkvenc_set(core, VEPU580_KLUT_WGT(i),
			   (t[0] & 0x3ffff) | (t[1] & 0x1ff) << 23);
		rkvenc_set(core, VEPU580_KLUT_WGT(i + 1),
			   (t[1] >> 9) | t[2] << 14);
	}
}

void vepu580_setup_h264_tuning(struct rkvenc_dev *rkvenc,
			       struct rkvenc_core *core, bool intra)
{
	const s8 *aq_step = intra ? vepu580_aq_step_i : vepu580_aq_step_p;
	unsigned int i;

	/* Intra prediction cost / mode decision */
	rkvenc_set(core, VEPU580_IPRD_CSTS,
		   FIELD_PREP(VEPU580_IPRD_CSTS_VTHD_Y, 9) |
		   FIELD_PREP(VEPU580_IPRD_CSTS_VTHD_C, 63));

	vepu580_setup_klut(core, intra);

	rkvenc_set(core, VEPU580_MADI_CFG, FIELD_PREP(VEPU580_MADI_CFG_MADI_THD, 25));
	rkvenc_set(core, VEPU580_MD_SAD_THD, pack4(4, 9, 15, 0));
	rkvenc_set(core, VEPU580_MADI_THD, pack4(4, 9, 15, 0));

	for (i = 0; i < 4; i++) {
		rkvenc_set(core, VEPU580_AQ_TTHD(i),
			   pack4(vepu580_aq_tthd[4 * i], vepu580_aq_tthd[4 * i + 1],
				 vepu580_aq_tthd[4 * i + 2], vepu580_aq_tthd[4 * i + 3]));
		rkvenc_set(core, VEPU580_AQ_STEP(i),
			   pack4(aq_step[4 * i] & 0x3f, aq_step[4 * i + 1] & 0x3f,
				 aq_step[4 * i + 2] & 0x3f, aq_step[4 * i + 3] & 0x3f));
	}

	/* Intra cost thresholds (1, 4, 9, 36) and weights */
	rkvenc_set(core, VEPU580_LVL32_INTRA_CST_THD0, pack2x12(1, 4));
	rkvenc_set(core, VEPU580_LVL32_INTRA_CST_THD1, pack2x12(9, 36));
	rkvenc_set(core, VEPU580_LVL16_INTRA_CST_THD0, pack2x12(1, 4));
	rkvenc_set(core, VEPU580_LVL16_INTRA_CST_THD1, pack2x12(9, 36));
	rkvenc_set(core, VEPU580_LVL8_INTRA_CST_THD0, pack2x12(1, 4));
	rkvenc_set(core, VEPU580_LVL8_INTRA_CST_THD1, pack2x12(9, 36));
	rkvenc_set(core, VEPU580_LVL16_INTRA_UL_CST_THD, 0);
	rkvenc_set(core, VEPU580_LVL32_INTRA_CST_WGT0, pack4(48, 60, 40, 48));
	rkvenc_set(core, VEPU580_LVL32_INTRA_CST_WGT1, pack4(48, 60, 40, 48));
	rkvenc_set(core, VEPU580_LVL16_INTRA_CST_WGT0, pack4(48, 60, 40, 48));
	rkvenc_set(core, VEPU580_LVL16_INTRA_CST_WGT1, pack4(36, 42, 28, 32));

	/* Quantisation dead zone: 683/1024 (I) or 583/1024, 171/1024 (P) */
	rkvenc_set(core, VEPU580_RDO_QUANT,
		   FIELD_PREP(VEPU580_RDO_QUANT_F_BIAS_I, intra ? 683 : 583) |
		   FIELD_PREP(VEPU580_RDO_QUANT_F_BIAS_P, 171));

	/* Adaptive transform (H.264 only) */
	if (intra) {
		rkvenc_set(core, VEPU580_ATR_THD0, pack2x12(1, 4));
		rkvenc_set(core, VEPU580_ATR_THD1,
			   FIELD_PREP(VEPU580_ATR_THD1_THD2, 36) |
			   FIELD_PREP(VEPU580_ATR_THD1_THDQP, 45));
		rkvenc_set(core, VEPU580_LVL16_ATR_WGT, pack4(16, 16, 16, 0));
		rkvenc_set(core, VEPU580_LVL8_ATR_WGT, pack4(22, 21, 20, 0));
		rkvenc_set(core, VEPU580_LVL4_ATR_WGT, pack4(20, 18, 16, 0));
	} else {
		rkvenc_set(core, VEPU580_ATR_THD0, pack2x12(4, 16));
		rkvenc_set(core, VEPU580_ATR_THD1,
			   FIELD_PREP(VEPU580_ATR_THD1_THD2, 81) |
			   FIELD_PREP(VEPU580_ATR_THD1_THDQP, 45));
		rkvenc_set(core, VEPU580_LVL16_ATR_WGT, pack4(22, 19, 16, 0));
		rkvenc_set(core, VEPU580_LVL8_ATR_WGT, pack4(22, 19, 18, 0));
		rkvenc_set(core, VEPU580_LVL4_ATR_WGT, pack4(22, 19, 16, 0));
	}

	/*
	 * Motion search quality / cost multipliers. Where MPP's tuning differs
	 * per scene mode, the values of its default (non-IPC) mode are used.
	 */
	rkvenc_set(core, VEPU580_CIME_SQI_CFG,
		   VEPU580_CIME_SQI_CFG_SAD_BIG_BLOCK |
		   VEPU580_CIME_SQI_CFG_PMV_SET_ZERO |
		   FIELD_PREP(VEPU580_CIME_SQI_CFG_PMV_NUM, 3));
	rkvenc_set(core, VEPU580_CIME_SQI_THD,
		   FIELD_PREP(VEPU580_CIME_SQI_THD_MVD_TH0, 32) |
		   FIELD_PREP(VEPU580_CIME_SQI_THD_MVD_TH1, 80) |
		   FIELD_PREP(VEPU580_CIME_SQI_THD_MVD_TH2, 128));
	rkvenc_set(core, VEPU580_CIME_SQI_MULTI0,
		   FIELD_PREP(VEPU580_CIME_SQI_MULTI_LO, 4) |
		   FIELD_PREP(VEPU580_CIME_SQI_MULTI_HI, 7));
	rkvenc_set(core, VEPU580_CIME_SQI_MULTI1,
		   FIELD_PREP(VEPU580_CIME_SQI_MULTI_LO, 20) |
		   FIELD_PREP(VEPU580_CIME_SQI_MULTI_HI, 20));
	rkvenc_set(core, VEPU580_RIME_SQI_THD,
		   FIELD_PREP(VEPU580_RIME_SQI_THD_CIME_SAD_TH0, 50) |
		   FIELD_PREP(VEPU580_RIME_SQI_THD_MVD_TH0, 3) |
		   FIELD_PREP(VEPU580_RIME_SQI_THD_MVD_TH1, 8));
	rkvenc_set(core, VEPU580_RIME_SQI_MULTI,
		   FIELD_PREP(VEPU580_RIME_SQI_MULTI0, 4) |
		   FIELD_PREP(VEPU580_RIME_SQI_MULTI1, 16) |
		   FIELD_PREP(VEPU580_RIME_SQI_MULTI2, 64));
	rkvenc_set(core, VEPU580_FME_SQI_THD0,
		   FIELD_PREP(VEPU580_FME_SQI_THD0_PU16_TH, 2));
	rkvenc_set(core, VEPU580_FME_SQI_THD1,
		   FIELD_PREP(VEPU580_FME_SQI_THD1_MOVE_LAMBDA, 1));

	/* RDO lambda, intra prediction weights stay zero for H.264 */
	for (i = 0; i < 52; i++)
		rkvenc_set(core, VEPU580_RDO_WGTA_QP(i), vepu580_h264_lambda[i + 6]);

	/* RDO intra / skip thresholds and weights */
	rkvenc_set(core, VEPU580_RDO_SQI_CFG, VEPU580_RDO_SQI_CFG_ATF_PSKIP_EN);
	rkvenc_set(core, VEPU580_RDO_INTRA_CIME_THD0, pack2x12(28, 44));
	rkvenc_set(core, VEPU580_RDO_INTRA_CIME_THD1, pack2x12(72, 0));
	rkvenc_set(core, VEPU580_RDO_INTRA_VAR_THD(0), pack2x12(25, 64));
	rkvenc_set(core, VEPU580_RDO_INTRA_VAR_THD(1), pack2x12(25, 64));
	rkvenc_set(core, VEPU580_RDO_INTRA_VAR_THD(2), pack2x12(70, 100));
	rkvenc_set(core, VEPU580_RDO_INTRA_VAR_THD(3), pack2x12(70, 100));
	rkvenc_set(core, VEPU580_RDO_INTRA_ATF_WGT(0), pack4(22, 21, 20, 0));
	rkvenc_set(core, VEPU580_RDO_INTRA_ATF_WGT(1), pack4(21, 20, 19, 0));
	rkvenc_set(core, VEPU580_RDO_INTRA_ATF_WGT(2), pack4(20, 19, 18, 0));
	rkvenc_set(core, VEPU580_RDO_INTRA_ATF_WGT(3), pack4(16, 16, 16, 0));
	rkvenc_set(core, VEPU580_RDO_SKIP_CIME_THD0, pack2x12(10, 8));
	rkvenc_set(core, VEPU580_RDO_SKIP_CIME_THD1, pack2x12(15, 25));
	rkvenc_set(core, VEPU580_RDO_SKIP_VAR_THD(0), pack2x12(25, 40));
	rkvenc_set(core, VEPU580_RDO_SKIP_VAR_THD(1), pack2x12(25, 40));
	rkvenc_set(core, VEPU580_RDO_SKIP_VAR_THD(2), pack2x12(70, 100));
	rkvenc_set(core, VEPU580_RDO_SKIP_VAR_THD(3), pack2x12(70, 100));
	rkvenc_set(core, VEPU580_RDO_SKIP_ATF_WGT(0), pack4(20, 16, 16, 16));
	rkvenc_set(core, VEPU580_RDO_SKIP_ATF_WGT(1), pack4(16, 16, 16, 0));
	rkvenc_set(core, VEPU580_RDO_SKIP_ATF_WGT(2), pack4(16, 16, 16, 0));
	rkvenc_set(core, VEPU580_RDO_SKIP_ATF_WGT(3), pack4(16, 16, 16, 0));

	/* Flat quantisation tables */
	memcpy(&core->shadow[VEPU580_SCL_START / 4], rkvenc->h264_scl,
	       VEPU580_SCL_H264_WORDS * sizeof(u32));
}

/*
 * Frame-level QP with the hardware row rate control: each block row
 * (macroblock row for H.264, CTU row for HEVC) gets target_bits /
 * num_blocks bits per block; the cumulative deviation from that target
 * selects one of nine QP adjustments, bounded by qp_range around the frame
 * QP and by [qp_min, qp_max].
 */
const struct vepu580_row_rc vepu580_h264_row_rc = {
	.qp_adj = { -2, -1, 0, 1, 2, 0, 0, 0, 1 },
	.dthd_outer = 4,
	.aq_mode = false,
};

void vepu580_setup_row_rc(struct rkvenc_core *core,
			  const struct vepu580_row_rc *cfg, unsigned int qp,
			  unsigned int qp_min, unsigned int qp_max,
			  unsigned int qp_range, u64 target_bits,
			  unsigned int num_blocks, unsigned int blocks_per_row)
{
	u64 ebit16 = div_u64(target_bits << 4, num_blocks);
	s32 row_bits, thd;
	unsigned int i;

	if (ebit16 >= 0x100000)
		ebit16 = 0x50000;

	row_bits = (ebit16 * blocks_per_row) >> 4;
	thd = 5 * row_bits / 16;

	rkvenc_or(core, VEPU580_ENC_PIC, FIELD_PREP(VEPU580_ENC_PIC_PIC_QP, qp));
	rkvenc_set(core, VEPU580_RC_CFG,
		   VEPU580_RC_CFG_RC_EN | VEPU580_RC_CFG_AQ_EN |
		   (cfg->aq_mode ? VEPU580_RC_CFG_AQ_MODE : 0) |
		   FIELD_PREP(VEPU580_RC_CFG_RC_CTU_NUM, blocks_per_row));
	rkvenc_set(core, VEPU580_RC_QP,
		   FIELD_PREP(VEPU580_RC_QP_RANGE, qp_range) |
		   FIELD_PREP(VEPU580_RC_QP_MAX, qp_max) |
		   FIELD_PREP(VEPU580_RC_QP_MIN, qp_min));
	rkvenc_set(core, VEPU580_RC_TGT, FIELD_PREP(VEPU580_RC_TGT_CTU_EBIT, ebit16));

	rkvenc_set(core, VEPU580_RC_ADJ0, 0);
	rkvenc_set(core, VEPU580_RC_ADJ1, 0);
	for (i = 0; i < 9; i++)
		rkvenc_or(core, i < 5 ? VEPU580_RC_ADJ0 : VEPU580_RC_ADJ1,
			  VEPU580_RC_ADJ_QP(i < 5 ? i : i - 5, cfg->qp_adj[i]));

	rkvenc_set(core, VEPU580_RC_DTHD(0), -cfg->dthd_outer * thd);
	rkvenc_set(core, VEPU580_RC_DTHD(1), -thd);
	rkvenc_set(core, VEPU580_RC_DTHD(2), thd);
	rkvenc_set(core, VEPU580_RC_DTHD(3), cfg->dthd_outer * thd);
	for (i = 4; i < 9; i++)
		rkvenc_set(core, VEPU580_RC_DTHD(i), 0x7fffffff);

	for (i = 0; i < 4; i++)
		rkvenc_set(core, VEPU580_ROI_QTHD(i), 0);
	/* All eight ROI areas use the frame QP bounds, qpmap_mode = 1. */
	rkvenc_set(core, VEPU580_ROI_QTHD(0),
		   qp_min | qp_max << 6 | qp_min << 12 | qp_max << 18 |
		   qp_min << 24);
	rkvenc_set(core, VEPU580_ROI_QTHD(1),
		   qp_max | qp_min << 6 | qp_max << 12 | qp_min << 18 |
		   qp_max << 24);
	rkvenc_set(core, VEPU580_ROI_QTHD(2),
		   qp_min | qp_max << 6 | qp_min << 12 | qp_max << 18 |
		   qp_min << 24);
	rkvenc_set(core, VEPU580_ROI_QTHD(3), qp_max | 1 << 30);
}

void vepu580_setup_fixed_qp(struct rkvenc_core *core, unsigned int qp)
{
	rkvenc_or(core, VEPU580_ENC_PIC, FIELD_PREP(VEPU580_ENC_PIC_PIC_QP, qp));
	rkvenc_set(core, VEPU580_RC_CFG, 0);
	rkvenc_set(core, VEPU580_RC_QP,
		   FIELD_PREP(VEPU580_RC_QP_MAX, qp) |
		   FIELD_PREP(VEPU580_RC_QP_MIN, qp));
	rkvenc_set(core, VEPU580_ROI_QTHD(0),
		   qp | qp << 6 | qp << 12 | qp << 18 | qp << 24);
	rkvenc_set(core, VEPU580_ROI_QTHD(1),
		   qp | qp << 6 | qp << 12 | qp << 18 | qp << 24);
	rkvenc_set(core, VEPU580_ROI_QTHD(2),
		   qp | qp << 6 | qp << 12 | qp << 18 | qp << 24);
	rkvenc_set(core, VEPU580_ROI_QTHD(3), qp | 1 << 30);
}

/*
 * H.264 8x8 forward quantisation multipliers (ITU-T H.264 8.5.12.1
 * normAdjust8x8 counterparts) for QP % 6, per position class v0..v5.
 */
static const u16 h264_quant8[6][6] = {
	{ 13107, 11428, 20972, 12222, 16777, 15481 },
	{ 11916, 10826, 19174, 11058, 14980, 14290 },
	{ 10082,  8943, 15978,  9675, 12710, 11985 },
	{  9362,  8228, 14913,  8931, 11984, 11259 },
	{  8192,  7346, 13159,  7740, 10486,  9777 },
	{  7282,  6428, 11570,  6830,  9118,  8640 },
};

static unsigned int h264_quant8_class(unsigned int i, unsigned int j)
{
	if (i % 4 == 0 && j % 4 == 0)
		return 0;
	if (i % 2 == 1 && j % 2 == 1)
		return 1;
	if (i % 4 == 2 && j % 4 == 2)
		return 2;
	if ((i % 4 == 0 && j % 2 == 1) || (i % 2 == 1 && j % 4 == 0))
		return 3;
	if ((i % 4 == 0 && j % 4 == 2) || (i % 4 == 2 && j % 4 == 0))
		return 4;
	return 5;
}

/*
 * Build the scaling list block for flat matrices: three groups of six 8x8
 * tables (QP % 6 = 0..5, raster order, two 16-bit entries per word). The
 * first group holds the multipliers, the second their low byte and the
 * third their high byte, which is the layout the MPP library programs.
 */
u32 *vepu580_gen_h264_scl(struct device *dev)
{
	u16 *tab;
	unsigned int g, q, i, j;

	tab = devm_kcalloc(dev, VEPU580_SCL_H264_WORDS, sizeof(u32), GFP_KERNEL);
	if (!tab)
		return NULL;

	for (g = 0; g < 3; g++)
		for (q = 0; q < 6; q++)
			for (i = 0; i < 8; i++)
				for (j = 0; j < 8; j++) {
					u16 v = h264_quant8[q][h264_quant8_class(i, j)];

					if (g == 1)
						v &= 0xff;
					else if (g == 2)
						v >>= 8;
					tab[(g * 6 + q) * 64 + i * 8 + j] = v;
				}

	return (u32 *)tab;
}
