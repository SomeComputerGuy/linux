// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rockchip VDPU720 JPEG decoder driver: header checks and hardware programming
 *
 * The register and table layouts were derived from the Rockchip RK3588 BSP
 * and from the Rockchip MPP library; no code was taken from either.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/string.h>

#include "rkjpegd.h"

bool rkjpegd_cap_fourcc_valid(u32 fourcc,
			      enum v4l2_jpeg_chroma_subsampling subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		return fourcc == V4L2_PIX_FMT_GREY;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return fourcc == V4L2_PIX_FMT_NV12;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return fourcc == V4L2_PIX_FMT_NV16 || fourcc == V4L2_PIX_FMT_NV12;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
		return fourcc == V4L2_PIX_FMT_NV24 || fourcc == V4L2_PIX_FMT_NV12;
	default:
		return false;
	}
}

u32 rkjpegd_native_fourcc(enum v4l2_jpeg_chroma_subsampling subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		return V4L2_PIX_FMT_GREY;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return V4L2_PIX_FMT_NV16;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
		return V4L2_PIX_FMT_NV24;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
	default:
		return V4L2_PIX_FMT_NV12;
	}
}

/* Annex K.3 tables: 16 code length counts followed by the values */
static const u8 *const rkjpegd_default_huff[4] = {
	[RKJPEGD_HUFF_DC(0)] = v4l2_jpeg_ref_table_luma_dc_ht,
	[RKJPEGD_HUFF_DC(1)] = v4l2_jpeg_ref_table_chroma_dc_ht,
	[RKJPEGD_HUFF_AC(0)] = v4l2_jpeg_ref_table_luma_ac_ht,
	[RKJPEGD_HUFF_AC(1)] = v4l2_jpeg_ref_table_chroma_ac_ht,
};

static int rkjpegd_copy_huff(struct rkjpegd_huff *huff, const u8 *table,
			     unsigned int max_values)
{
	unsigned int count = 0, codes = 0;
	unsigned int i;

	memset(huff, 0, sizeof(*huff));

	/* every code of length i + 1 must fit in i + 1 bits */
	for (i = 0; i < 16; i++) {
		codes = (codes << 1) + table[i];
		if (codes > (2U << i))
			return -EINVAL;
		count += table[i];
	}
	if (!count || count > max_values)
		return -EINVAL;

	memcpy(huff->bits, table, 16);
	memcpy(huff->vals, table + 16, count);

	return 0;
}

int rkjpegd_parse_header(void *buf, size_t len, struct rkjpegd_hdr *hdr)
{
	struct v4l2_jpeg_reference qtables[V4L2_JPEG_MAX_TABLES] = {};
	struct v4l2_jpeg_reference htables[V4L2_JPEG_MAX_TABLES] = {};
	struct v4l2_jpeg_scan_header scan = {};
	struct v4l2_jpeg_header header = {
		.scan = &scan,
		.quantization_tables = qtables,
		.huffman_tables = htables,
	};
	const struct v4l2_jpeg_frame_header *frame = &header.frame;
	unsigned int i;
	int ret;

	ret = v4l2_jpeg_parse_header(buf, len, &header);
	if (ret < 0)
		return ret;

	/* SOF0 or SOF1, 8-bit, and a scan with all components interleaved */
	if (!header.sof.start || frame->precision != 8 ||
	    !frame->width || !frame->height ||
	    frame->width > RKJPEGD_MAX_SIZE || frame->height > RKJPEGD_MAX_SIZE)
		return -EINVAL;

	if (header.ecs_offset >= len)
		return -EINVAL;

	switch (frame->subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		if (frame->num_components != 1 ||
		    frame->component[0].horizontal_sampling_factor !=
		    frame->component[0].vertical_sampling_factor)
			return -EINVAL;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
	case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
		if (frame->num_components != 3)
			return -EINVAL;
		/* Adobe APP14 transform 0 means RGB components */
		if (header.app14_tf == V4L2_JPEG_APP14_TF_CMYK_RGB)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (scan.num_components != frame->num_components)
		return -EINVAL;

	memset(hdr, 0, sizeof(*hdr));
	hdr->width = frame->width;
	hdr->height = frame->height;
	hdr->subsampling = frame->subsampling;
	hdr->num_components = frame->num_components;
	hdr->restart_interval = header.restart_interval;
	hdr->ecs_offset = header.ecs_offset;

	for (i = 0; i < frame->num_components; i++) {
		const struct v4l2_jpeg_scan_component_spec *sc =
			&scan.component[i];
		u8 tq = frame->component[i].quantization_table_selector;

		if (sc->component_selector !=
		    frame->component[i].component_identifier)
			return -EINVAL;
		if (tq >= V4L2_JPEG_MAX_TABLES || !qtables[tq].start ||
		    qtables[tq].length != RKJPEGD_QTBL_ENTRIES)
			return -EINVAL;
		if (sc->dc_entropy_coding_table_selector > 1 ||
		    sc->ac_entropy_coding_table_selector > 1)
			return -EINVAL;

		hdr->tq[i] = tq;
		hdr->td[i] = sc->dc_entropy_coding_table_selector;
		hdr->ta[i] = sc->ac_entropy_coding_table_selector;
	}

	for (i = 0; i < V4L2_JPEG_MAX_TABLES; i++)
		if (qtables[i].start)
			memcpy(hdr->qtbl[i], qtables[i].start,
			       RKJPEGD_QTBL_ENTRIES);

	/* use the Annex K tables for any table the stream does not define */
	for (i = 0; i < ARRAY_SIZE(hdr->huff); i++) {
		const u8 *table = htables[i].start ?: rkjpegd_default_huff[i];
		unsigned int max = i < RKJPEGD_HUFF_AC(0) ?
				   RKJPEGD_DC_MAX_VALUES : RKJPEGD_AC_MAX_VALUES;

		ret = rkjpegd_copy_huff(&hdr->huff[i], table, max);
		if (ret)
			return ret;
	}

	return 0;
}

static void rkjpegd_write_qtbl(__le16 *dst, const u8 *zigzag)
{
	unsigned int i;

	/* the hardware wants the coefficients in raster order */
	for (i = 0; i < RKJPEGD_QTBL_ENTRIES; i++)
		dst[v4l2_jpeg_zigzag_scan_index[i]] = cpu_to_le16(zigzag[i]);
}

/*
 * Each Huffman table is described by the first code of each code length
 * and by the number of codes up to and including each length, packed two
 * per 16-bit word. The first entry, which would always be zero, holds the
 * last 16-bit code instead.
 */
static void rkjpegd_write_mincode(__le16 *dst, const struct rkjpegd_huff *huff)
{
	u16 mincode[16];
	u8 count[16];
	unsigned int code = 0, total = 0;
	unsigned int i;

	for (i = 0; i < 16; i++) {
		mincode[i] = code;
		code += huff->bits[i];
		total += huff->bits[i];
		count[i] = total;
		code <<= 1;
	}

	if (huff->bits[15])
		mincode[0] = mincode[15] + huff->bits[15] - 1;
	else
		mincode[0] = mincode[15];

	for (i = 0; i < 16; i++)
		dst[i] = cpu_to_le16(mincode[i]);
	for (i = 0; i < 8; i++)
		dst[16 + i] = cpu_to_le16(count[2 * i] | count[2 * i + 1] << 8);
}

static unsigned int rkjpegd_huff_count(const struct rkjpegd_huff *huff)
{
	unsigned int i, count = 0;

	for (i = 0; i < 16; i++)
		count += huff->bits[i];

	return count;
}

static void rkjpegd_write_tables(struct rkjpegd_dev *jpegd,
				 const struct rkjpegd_hdr *hdr)
{
	__le16 *qtbl = jpegd->tables + RKJPEGD_QTBL_OFFSET;
	__le16 *mincode = jpegd->tables + RKJPEGD_HTBL_MINCODE_OFFSET;
	u8 *values = jpegd->tables + RKJPEGD_HTBL_VALUE_OFFSET;
	unsigned int i;

	memset(jpegd->tables, 0, RKJPEGD_TABLES_SIZE);

	for (i = 0; i < hdr->num_components; i++) {
		const struct rkjpegd_huff *dc =
			&hdr->huff[RKJPEGD_HUFF_DC(hdr->td[i])];
		const struct rkjpegd_huff *ac =
			&hdr->huff[RKJPEGD_HUFF_AC(hdr->ta[i])];
		u8 *val = values + i * RKJPEGD_HTBL_VALUE_SIZE;

		rkjpegd_write_qtbl(qtbl + i * RKJPEGD_QTBL_ENTRIES,
				   hdr->qtbl[hdr->tq[i]]);

		rkjpegd_write_mincode(mincode + i * RKJPEGD_HTBL_MINCODE_ENTRIES,
				      dc);
		rkjpegd_write_mincode(mincode + i * RKJPEGD_HTBL_MINCODE_ENTRIES +
				      RKJPEGD_HTBL_MINCODE_ENTRIES / 2, ac);

		memcpy(val, dc->vals, rkjpegd_huff_count(dc));
		memcpy(val + RKJPEGD_HTBL_AC_VALUE_OFFSET, ac->vals,
		       rkjpegd_huff_count(ac));
	}
}

void rkjpegd_hw_run(struct rkjpegd_dev *jpegd, struct rkjpegd_ctx *ctx,
		    const struct rkjpegd_hdr *hdr, dma_addr_t src, size_t len,
		    dma_addr_t dst)
{
	const struct v4l2_pix_format_mplane *fmt = &ctx->cap_fmt;
	u32 stride = fmt->plane_fmt[0].bytesperline / RKJPEGD_ALIGN;
	u32 uv_stride, mode, out_fmt, sys, tsel_q, tsel_h;
	u32 strm_off = ALIGN_DOWN(hdr->ecs_offset, RKJPEGD_ALIGN);
	u32 mcu_w, mcu_h, val;
	bool fill_down;

	switch (hdr->subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		mode = RKJPEGD_MODE_400;
		mcu_w = 8;
		mcu_h = 8;
		uv_stride = 0;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		mode = RKJPEGD_MODE_420;
		mcu_w = 16;
		mcu_h = 16;
		uv_stride = stride;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		mode = RKJPEGD_MODE_422;
		mcu_w = 16;
		mcu_h = 8;
		uv_stride = stride;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
	default:
		mode = RKJPEGD_MODE_444;
		mcu_w = 8;
		mcu_h = 8;
		uv_stride = stride * 2;
		break;
	}

	if (fmt->pixelformat == rkjpegd_native_fourcc(hdr->subsampling)) {
		out_fmt = RKJPEGD_OUT_FMT_NATIVE;
		fill_down = mode == RKJPEGD_MODE_420;
	} else {
		/* NV12 conversion of 4:2:2 and 4:4:4 */
		out_fmt = RKJPEGD_OUT_FMT_NV12;
		uv_stride = stride;
		fill_down = true;
	}

	/*
	 * The output buffer is padded to 16 pixels in both directions; let the
	 * hardware fill the padding when the last MCU is smaller than that.
	 */
	if (ALIGN(hdr->height, mcu_h) < ALIGN(hdr->height, RKJPEGD_ALIGN))
		fill_down = true;

	sys = FIELD_PREP(RKJPEGD_SYS_OUT_FMT, out_fmt);
	if (fill_down)
		sys |= RKJPEGD_SYS_FILL_DOWN_E;
	if (ALIGN(hdr->width, mcu_w) < ALIGN(hdr->width, RKJPEGD_ALIGN))
		sys |= RKJPEGD_SYS_FILL_RIGHT_E;

	/* Cb and Cr may share their tables, or each have their own */
	if (hdr->num_components == 1) {
		tsel_q = 1;
		tsel_h = 1;
	} else {
		tsel_q = hdr->tq[2] == hdr->tq[1] ? 2 : 3;
		tsel_h = hdr->td[2] == hdr->td[1] &&
			 hdr->ta[2] == hdr->ta[1] ? 2 : 3;
	}

	rkjpegd_write_tables(jpegd, hdr);

	rkjpegd_write(jpegd, RKJPEGD_REG_SYS, sys);
	rkjpegd_write(jpegd, RKJPEGD_REG_PIC_SIZE,
		      FIELD_PREP(RKJPEGD_PIC_WIDTH_M1, hdr->width - 1) |
		      FIELD_PREP(RKJPEGD_PIC_HEIGHT_M1, hdr->height - 1));

	val = FIELD_PREP(RKJPEGD_PIC_JPEG_MODE, mode) |
	      FIELD_PREP(RKJPEGD_PIC_QTABLES_SEL, tsel_q) |
	      FIELD_PREP(RKJPEGD_PIC_HTABLES_SEL, tsel_h);
	if (hdr->restart_interval)
		val |= RKJPEGD_PIC_DRI_E |
		       FIELD_PREP(RKJPEGD_PIC_DRI_MCU_M1,
				  hdr->restart_interval - 1);
	rkjpegd_write(jpegd, RKJPEGD_REG_PIC_FMT, val);

	rkjpegd_write(jpegd, RKJPEGD_REG_HOR_VIRSTRIDE,
		      FIELD_PREP(RKJPEGD_Y_HOR_VIRSTRIDE, stride & 0xffff) |
		      FIELD_PREP(RKJPEGD_UV_HOR_VIRSTRIDE, uv_stride));
	rkjpegd_write(jpegd, RKJPEGD_REG_Y_VIRSTRIDE,
		      fmt->plane_fmt[0].bytesperline * fmt->height);

	val = FIELD_PREP(RKJPEGD_QTBL_LEN,
			 tsel_q * RKJPEGD_QTBL_SIZE / 16 - 1) |
	      FIELD_PREP(RKJPEGD_HTBL_MINCODE_LEN,
			 tsel_h * RKJPEGD_HTBL_MINCODE_SIZE / 16 - 1) |
	      FIELD_PREP(RKJPEGD_HTBL_VALUE_LEN,
			 tsel_h * RKJPEGD_HTBL_VALUE_SIZE / 16 - 1);
	if (stride > 0xffff)
		val |= RKJPEGD_Y_HOR_VIRSTRIDE_H;
	rkjpegd_write(jpegd, RKJPEGD_REG_TBL_LEN, val);

	rkjpegd_write(jpegd, RKJPEGD_REG_STRM_LEN,
		      FIELD_PREP(RKJPEGD_STRM_START_BYTE,
				 hdr->ecs_offset - strm_off) |
		      FIELD_PREP(RKJPEGD_STRM_LEN,
				 DIV_ROUND_UP(len - strm_off, 16) - 1));

	rkjpegd_write(jpegd, RKJPEGD_REG_QTBL_BASE,
		      jpegd->tables_dma + RKJPEGD_QTBL_OFFSET);
	rkjpegd_write(jpegd, RKJPEGD_REG_HTBL_MINCODE_BASE,
		      jpegd->tables_dma + RKJPEGD_HTBL_MINCODE_OFFSET);
	rkjpegd_write(jpegd, RKJPEGD_REG_HTBL_VALUE_BASE,
		      jpegd->tables_dma + RKJPEGD_HTBL_VALUE_OFFSET);
	rkjpegd_write(jpegd, RKJPEGD_REG_STRM_BASE, src + strm_off);
	rkjpegd_write(jpegd, RKJPEGD_REG_OUT_BASE, dst);

	/* skip stray 0xffff sequences and unexpected markers */
	rkjpegd_write(jpegd, RKJPEGD_REG_STRM_ERROR,
		      RKJPEGD_ERR_PRC_MODE |
		      FIELD_PREP(RKJPEGD_ERR_FFFF_MODE, RKJPEGD_ERR_MODE_SKIP) |
		      FIELD_PREP(RKJPEGD_ERR_OTHER_MARK_MODE,
				 RKJPEGD_ERR_MODE_SKIP));
	rkjpegd_write(jpegd, RKJPEGD_REG_STRM_MASK, 0);
	rkjpegd_write(jpegd, RKJPEGD_REG_CLK_GATE, RKJPEGD_CLK_GATE_ALL);

	/* writel() orders the table and register writes before the start */
	writel(RKJPEGD_INT_DEC_E | RKJPEGD_INT_TIMEOUT_E |
	       RKJPEGD_INT_BUF_EMPTY_E, jpegd->regs + RKJPEGD_REG_INT);
}

static void rkjpegd_iommu_restore(struct rkjpegd_dev *jpegd)
{
	int ret;

	if (!jpegd->empty_domain)
		return;

	/*
	 * A CRU reset may also reset the IOMMU. Attaching an empty domain and
	 * detaching it again makes the IOMMU driver reprogram the page table
	 * of the default domain.
	 */
	ret = iommu_attach_device(jpegd->empty_domain, jpegd->dev);
	if (ret) {
		dev_warn(jpegd->dev, "cannot attach empty domain: %d\n", ret);
		return;
	}
	iommu_detach_device(jpegd->empty_domain, jpegd->dev);
}

int rkjpegd_hw_reset(struct rkjpegd_dev *jpegd)
{
	u32 status = rkjpegd_read(jpegd, RKJPEGD_REG_INT);
	int ret;

	/* an idle core only accepts a soft reset with this bit set */
	if (!(status & RKJPEGD_INT_DEC_E))
		rkjpegd_write(jpegd, RKJPEGD_REG_SYS, RKJPEGD_SYS_FORCE_SOFTRST);

	writel(status | RKJPEGD_INT_SOFT_RST_EN, jpegd->regs + RKJPEGD_REG_INT);
	ret = readl_poll_timeout(jpegd->regs + RKJPEGD_REG_INT, status,
				 status & RKJPEGD_INT_SOFT_RST_RDY, 1, 1000);
	if (ret) {
		dev_warn(jpegd->dev, "soft reset timed out, resetting core\n");
		ret = reset_control_assert(jpegd->rstc);
		if (!ret) {
			udelay(5);
			ret = reset_control_deassert(jpegd->rstc);
		}
		rkjpegd_iommu_restore(jpegd);
	}

	rkjpegd_write(jpegd, RKJPEGD_REG_INT, 0);
	rkjpegd_write(jpegd, RKJPEGD_REG_SYS, 0);

	return ret;
}
