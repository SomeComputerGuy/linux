/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Rockchip IEP2 deinterlacer V4L2 mem2mem driver
 */

#ifndef __ROCKCHIP_IEP_H__
#define __ROCKCHIP_IEP_H__

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#define IEP_NAME		"rockchip-iep"

/* Processing is done on 16x4 tiles */
#define IEP_TILE_W		16
#define IEP_TILE_H		4

#define IEP_MIN_WIDTH		32U
#define IEP_MAX_WIDTH		1920U
#define IEP_MIN_HEIGHT		16U
#define IEP_MAX_HEIGHT		1088U

/* Stride registers are 16 bits wide, in units of 32-bit words */
#define IEP_MAX_STRIDE		(0xffff * 4)
#define IEP_STRIDE_ALIGN	16

/* Context buffers, sizes as allocated by the Rockchip BSP / MPP */
#define IEP_MD_BUF_SIZE		(IEP_MAX_WIDTH * IEP_MAX_HEIGHT)
#define IEP_MV_BUF_SIZE		(120 * 480)
#define IEP_ROI_BUF_SIZE	(120 * 272)

struct iep_fmt {
	u32 fourcc;
	u8 hw_fmt;
	u8 hw_swap;
	/* vertical chroma subsampling factor */
	u8 vdiv;
};

struct iep_dev {
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device vfd;
	struct device *dev;
	/* serialises ioctls and vb2 queue operations */
	struct mutex mutex;

	void __iomem *regs;
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *rstc;
	struct iommu_domain *empty_domain;

	void *roi_cpu;
	dma_addr_t roi_dma;

	struct delayed_work watchdog_work;
	/* protects busy and irq_status */
	spinlock_t irq_lock;
	/* a job has been started and has not completed or timed out yet */
	bool busy;
	u32 irq_status;
	/* shared interrupts not raised by the IEP during the current job */
	unsigned int foreign_irqs;
};

struct iep_ctx {
	struct v4l2_fh fh;
	struct iep_dev *iep;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *field_rate_ctrl;

	const struct iep_fmt *fmt;
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;

	/* V4L2_CID_ROCKCHIP_IEP_* control values */
	u32 deint_mode;
	bool field_rate;

	/* motion detection / motion vector history, per context */
	void *md_cpu;
	dma_addr_t md_dma;
	void *mv_cpu;
	dma_addr_t mv_dma;

	/* previous OUTPUT buffer, kept as temporal reference */
	struct vb2_v4l2_buffer *prev;
	/* timestamp of the previous OUTPUT buffer, for field-rate output */
	u64 prev_ts;
	bool prev_ts_valid;

	/* buffers of the job in progress */
	struct vb2_v4l2_buffer *job_src;
	struct vb2_v4l2_buffer *job_dst[2];
	unsigned int job_ndst;
	/* keep job_src as the reference for the next job */
	bool job_keep_src;

	u32 out_sequence;
	u32 cap_sequence;
};

#endif /* __ROCKCHIP_IEP_H__ */
