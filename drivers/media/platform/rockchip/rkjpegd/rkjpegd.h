/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Rockchip VDPU720 JPEG decoder driver
 */
#ifndef RKJPEGD_H_
#define RKJPEGD_H_

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/reset.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#define RKJPEGD_NAME			"rockchip-jpegd"

/* Registers */
#define RKJPEGD_REG_ID			0x000
#define   RKJPEGD_ID_VPU720		0xdb1f0006

#define RKJPEGD_REG_INT			0x004
#define   RKJPEGD_INT_DEC_E		BIT(0)
#define   RKJPEGD_INT_IRQ_DIS		BIT(1)
#define   RKJPEGD_INT_TIMEOUT_E		BIT(2)
#define   RKJPEGD_INT_BUF_EMPTY_E	BIT(3)
#define   RKJPEGD_INT_SOFT_RST_EN	BIT(5)
#define   RKJPEGD_INT_IRQ_RAW		BIT(6)
#define   RKJPEGD_INT_IRQ		BIT(8)
#define   RKJPEGD_INT_RDY_STA		BIT(9)
#define   RKJPEGD_INT_BUS_STA		BIT(10)
#define   RKJPEGD_INT_ERROR_STA		BIT(11)
#define   RKJPEGD_INT_TIMEOUT_STA	BIT(12)
#define   RKJPEGD_INT_BUF_EMPTY_STA	BIT(13)
#define   RKJPEGD_INT_SOFT_RST_RDY	BIT(14)
#define   RKJPEGD_INT_ERR_MASK		(RKJPEGD_INT_BUS_STA | \
					 RKJPEGD_INT_ERROR_STA | \
					 RKJPEGD_INT_TIMEOUT_STA | \
					 RKJPEGD_INT_BUF_EMPTY_STA)

#define RKJPEGD_REG_SYS			0x008
#define   RKJPEGD_SYS_OUT_CBCR_SWAP	BIT(9)
#define   RKJPEGD_SYS_SCALEDOWN		GENMASK(13, 12)
#define   RKJPEGD_SYS_FORCE_SOFTRST	BIT(17)
#define   RKJPEGD_SYS_FILL_DOWN_E	BIT(24)
#define   RKJPEGD_SYS_FILL_RIGHT_E	BIT(25)
#define   RKJPEGD_SYS_OUT_TILE		BIT(26)
#define   RKJPEGD_SYS_OUT_FMT		GENMASK(29, 27)
#define     RKJPEGD_OUT_FMT_NATIVE	0
#define     RKJPEGD_OUT_FMT_RGB888	1
#define     RKJPEGD_OUT_FMT_RGB565	2
#define     RKJPEGD_OUT_FMT_NV12	3
#define     RKJPEGD_OUT_FMT_YUYV	4

#define RKJPEGD_REG_PIC_SIZE		0x00c
#define   RKJPEGD_PIC_WIDTH_M1		GENMASK(15, 0)
#define   RKJPEGD_PIC_HEIGHT_M1		GENMASK(31, 16)

#define RKJPEGD_REG_PIC_FMT		0x010
#define   RKJPEGD_PIC_JPEG_MODE		GENMASK(2, 0)
#define     RKJPEGD_MODE_400		0
#define     RKJPEGD_MODE_411		1
#define     RKJPEGD_MODE_420		2
#define     RKJPEGD_MODE_422		3
#define     RKJPEGD_MODE_440		4
#define     RKJPEGD_MODE_444		5
#define   RKJPEGD_PIC_PIXEL_DEPTH	GENMASK(6, 4)
#define   RKJPEGD_PIC_QTABLES_SEL	GENMASK(9, 8)
#define   RKJPEGD_PIC_HTABLES_SEL	GENMASK(13, 12)
#define   RKJPEGD_PIC_DRI_E		BIT(15)
#define   RKJPEGD_PIC_DRI_MCU_M1	GENMASK(31, 16)

/* Horizontal strides, in units of 16 bytes */
#define RKJPEGD_REG_HOR_VIRSTRIDE	0x014
#define   RKJPEGD_Y_HOR_VIRSTRIDE	GENMASK(15, 0)
#define   RKJPEGD_UV_HOR_VIRSTRIDE	GENMASK(31, 16)

/* Luma plane size in bytes (offset of the chroma plane), 16 byte aligned */
#define RKJPEGD_REG_Y_VIRSTRIDE		0x018

/* Table lengths, in units of 16 bytes minus one */
#define RKJPEGD_REG_TBL_LEN		0x01c
#define   RKJPEGD_QTBL_LEN		GENMASK(4, 0)
#define   RKJPEGD_HTBL_MINCODE_LEN	GENMASK(12, 8)
#define   RKJPEGD_HTBL_VALUE_LEN	GENMASK(21, 16)
#define   RKJPEGD_Y_HOR_VIRSTRIDE_H	BIT(24)

/* Stream length in units of 16 bytes minus one, start byte offset */
#define RKJPEGD_REG_STRM_LEN		0x020
#define   RKJPEGD_STRM_START_BYTE	GENMASK(3, 0)
#define   RKJPEGD_STRM_LEN		GENMASK(31, 4)

#define RKJPEGD_REG_QTBL_BASE		0x024
#define RKJPEGD_REG_HTBL_MINCODE_BASE	0x028
#define RKJPEGD_REG_HTBL_VALUE_BASE	0x02c
#define RKJPEGD_REG_STRM_BASE		0x030
#define RKJPEGD_REG_OUT_BASE		0x034

#define RKJPEGD_REG_STRM_ERROR		0x038
#define   RKJPEGD_ERR_PRC_MODE		BIT(0)
#define   RKJPEGD_ERR_FFFF_MODE		GENMASK(6, 5)
#define   RKJPEGD_ERR_OTHER_MARK_MODE	GENMASK(8, 7)
#define     RKJPEGD_ERR_MODE_SKIP	2

#define RKJPEGD_REG_STRM_MASK		0x03c

#define RKJPEGD_REG_CLK_GATE		0x040
#define   RKJPEGD_CLK_GATE_ALL		GENMASK(7, 0)

#define RKJPEGD_REG_DBG_MCU_POS		0x080
#define RKJPEGD_REG_DBG_ERR		0x084
#define RKJPEGD_REG_PERF_WORKING_CNT	0x09c

/*
 * Table buffer layout: tables for up to three components, 8-bit sample
 * precision only. Each base address must be 64 byte aligned.
 */
#define RKJPEGD_NUM_TBL			3
#define RKJPEGD_QTBL_ENTRIES		64
#define RKJPEGD_QTBL_SIZE		(RKJPEGD_QTBL_ENTRIES * sizeof(u16))
/* min codes and accumulated code counts, for DC and for AC */
#define RKJPEGD_HTBL_MINCODE_ENTRIES	48
#define RKJPEGD_HTBL_MINCODE_SIZE	(RKJPEGD_HTBL_MINCODE_ENTRIES * sizeof(u16))
#define RKJPEGD_HTBL_VALUE_SIZE		192
#define RKJPEGD_HTBL_AC_VALUE_OFFSET	16
#define RKJPEGD_QTBL_OFFSET		0
#define RKJPEGD_HTBL_MINCODE_OFFSET	(RKJPEGD_QTBL_OFFSET + \
					 RKJPEGD_NUM_TBL * RKJPEGD_QTBL_SIZE)
#define RKJPEGD_HTBL_VALUE_OFFSET	(RKJPEGD_HTBL_MINCODE_OFFSET + \
					 ALIGN(RKJPEGD_NUM_TBL * \
					       RKJPEGD_HTBL_MINCODE_SIZE, 64))
#define RKJPEGD_TABLES_SIZE		(RKJPEGD_HTBL_VALUE_OFFSET + \
					 RKJPEGD_NUM_TBL * \
					 RKJPEGD_HTBL_VALUE_SIZE)

#define RKJPEGD_MIN_SIZE		1
#define RKJPEGD_MAX_SIZE		16384
#define RKJPEGD_DEF_WIDTH		1920
#define RKJPEGD_DEF_HEIGHT		1080
#define RKJPEGD_ALIGN			16
#define RKJPEGD_MIN_OUT_SIZE		SZ_1M
#define RKJPEGD_DEF_OUT_MAX_SIZE	SZ_32M
#define RKJPEGD_MAX_OUT_SIZE		SZ_256M

#define RKJPEGD_DC_MAX_VALUES		12
#define RKJPEGD_AC_MAX_VALUES		162

#define RKJPEGD_WATCHDOG_MS		2000
#define RKJPEGD_AUTOSUSPEND_MS		100

struct rkjpegd_huff {
	u8 bits[16];
	u8 vals[RKJPEGD_AC_MAX_VALUES];
};

/* Huffman table index: class (DC = 0, AC = 1) * 2 + destination */
#define RKJPEGD_HUFF_DC(th)		(th)
#define RKJPEGD_HUFF_AC(th)		(2 + (th))

/*
 * Information extracted from a JPEG header. Tables are copied so that the
 * header does not have to be parsed again when programming the hardware.
 */
struct rkjpegd_hdr {
	u16 width;
	u16 height;
	enum v4l2_jpeg_chroma_subsampling subsampling;
	u8 num_components;
	u16 restart_interval;
	u32 ecs_offset;
	/* per frame component: quantisation and Huffman table selectors */
	u8 tq[RKJPEGD_NUM_TBL];
	u8 td[RKJPEGD_NUM_TBL];
	u8 ta[RKJPEGD_NUM_TBL];
	/* quantisation tables, zigzag order */
	u8 qtbl[V4L2_JPEG_MAX_TABLES][RKJPEGD_QTBL_ENTRIES];
	/* DC0, DC1, AC0, AC1 */
	struct rkjpegd_huff huff[4];
};

struct rkjpegd_buf {
	/* must be first */
	struct v4l2_m2m_buffer m2m_buf;
	bool valid;
	struct rkjpegd_hdr hdr;
};

static inline struct rkjpegd_buf *to_rkjpegd_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct rkjpegd_buf, m2m_buf.vb);
}

enum rkjpegd_state {
	/* no header parsed yet */
	RKJPEGD_STATE_INIT,
	/* decoding */
	RKJPEGD_STATE_RUNNING,
	/* waiting for the client to reconfigure the CAPTURE queue */
	RKJPEGD_STATE_SOURCE_CHANGE,
};

struct rkjpegd_ctx;

struct rkjpegd_dev {
	/*
	 * Not first: v4l2_device_unregister() clears the device's driver
	 * data when it points to the v4l2_device.
	 */
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	/* serialises ioctls and queue operations */
	struct mutex lock;
	void __iomem *regs;
	int irq;
	struct clk_bulk_data *clocks;
	int num_clocks;
	struct reset_control *rstc;
	struct iommu_domain *empty_domain;
	struct delayed_work watchdog_work;
	u32 hw_id;
	/* context of the job running on the hardware */
	struct rkjpegd_ctx *curr_ctx;
	u32 irq_status;

	void *tables;
	dma_addr_t tables_dma;
};

struct rkjpegd_ctx {
	struct v4l2_fh fh;
	struct rkjpegd_dev *jpegd;
	enum rkjpegd_state state;

	struct v4l2_pix_format_mplane out_fmt;
	struct v4l2_pix_format_mplane cap_fmt;
	/* the client chose the CAPTURE pixel format with S_FMT */
	bool cap_fourcc_set;
	/* visible size and subsampling the CAPTURE format was set up for */
	u32 visible_width;
	u32 visible_height;
	enum v4l2_jpeg_chroma_subsampling subsampling;
	bool header_known;

	u32 out_sequence;
	u32 cap_sequence;
};

static inline struct rkjpegd_ctx *file_to_rkjpegd_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct rkjpegd_ctx, fh);
}

static inline void rkjpegd_write(struct rkjpegd_dev *jpegd, u32 reg, u32 val)
{
	writel_relaxed(val, jpegd->regs + reg);
}

static inline u32 rkjpegd_read(struct rkjpegd_dev *jpegd, u32 reg)
{
	return readl_relaxed(jpegd->regs + reg);
}

int rkjpegd_parse_header(void *buf, size_t len, struct rkjpegd_hdr *hdr);
void rkjpegd_hw_run(struct rkjpegd_dev *jpegd, struct rkjpegd_ctx *ctx,
		    const struct rkjpegd_hdr *hdr, dma_addr_t src, size_t len,
		    dma_addr_t dst);
int rkjpegd_hw_reset(struct rkjpegd_dev *jpegd);
bool rkjpegd_cap_fourcc_valid(u32 fourcc,
			      enum v4l2_jpeg_chroma_subsampling subsampling);
u32 rkjpegd_native_fourcc(enum v4l2_jpeg_chroma_subsampling subsampling);

#endif /* RKJPEGD_H_ */
