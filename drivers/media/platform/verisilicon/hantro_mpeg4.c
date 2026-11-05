// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver: MPEG-4 Part 2 and H.263 helpers
 */

#include <linux/math64.h>

#include "hantro.h"
#include "hantro_hw.h"

/* Raster position of the coefficients in zigzag scanning order */
static const u8 zigzag[64] = {
	0,   1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63
};

/*
 * The hardware takes the intra matrix, then the non-intra matrix, both in
 * raster order. The uAPI carries them in zigzag scanning order.
 */
void hantro_mpeg4_dec_copy_qtable(u8 *qtable,
				  const struct v4l2_ctrl_mpeg4_quantisation *q)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zigzag); i++) {
		qtable[zigzag[i]] = q->intra_quantiser_matrix[i];
		qtable[zigzag[i] + 64] = q->non_intra_quantiser_matrix[i];
	}
}

/*
 * num / den as an unsigned fraction with 27 fractional bits, rounded up,
 * the form of the TRB / TRD ratios of the direct mode. 0 if it does not
 * exist.
 */
u32 hantro_mpeg4_direct_ratio(s32 num, s32 den)
{
	if (num < 0 || den <= 0 || num > den)
		return 0;

	return div_u64(((u64)num << 27) + den - 1, den);
}

int hantro_mpeg4_dec_init(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_aux_buf *qtable = &ctx->mpeg4_dec.qtable;

	qtable->size = ARRAY_SIZE(zigzag) * 2;
	qtable->cpu = dma_alloc_coherent(vpu->dev, qtable->size, &qtable->dma,
					 GFP_KERNEL);
	if (!qtable->cpu)
		return -ENOMEM;

	return 0;
}

void hantro_mpeg4_dec_exit(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_aux_buf *qtable = &ctx->mpeg4_dec.qtable;

	dma_free_coherent(vpu->dev, qtable->size, qtable->cpu, qtable->dma);
}
