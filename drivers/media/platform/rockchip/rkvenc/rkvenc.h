/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip VEPU580 (RKVENC) stateless video encoder driver
 */

#ifndef RKVENC_H_
#define RKVENC_H_

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-h264-enc.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "rkvenc-regs.h"

struct rkvenc_ctx;
struct rkvenc_core;

/**
 * struct rkvenc_aux_buf - driver-internal DMA buffer
 * @cpu: kernel address, or NULL when allocated without a kernel mapping
 * @dma: device address (IOVA in the shared IOMMU domain)
 * @size: size in bytes
 * @attrs: DMA attributes used for the allocation
 */
struct rkvenc_aux_buf {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	unsigned long attrs;
};

/**
 * struct rkvenc_rec_buf - reconstructed / reference picture
 * @pixel: FBC header followed by the FBC body
 * @thumb: 1/4 x 1/4 down-scaled luma used by the coarse motion search
 * @colmv: collocated motion vectors (HEVC TMVP only)
 * @fbc_hdr_size: offset of the FBC body inside @pixel
 */
struct rkvenc_rec_buf {
	struct rkvenc_aux_buf pixel;
	struct rkvenc_aux_buf thumb;
	struct rkvenc_aux_buf colmv;
	size_t fbc_hdr_size;
};

/**
 * struct rkvenc_feedback - per-frame values read back from the status block
 */
struct rkvenc_feedback {
	u32 int_sta;
	u64 bs_len;
	u64 sse;
	u32 qp_sum;
	u32 madi;
	u32 madp;
	u32 intra_blocks;
	u32 cycles;
};

/**
 * struct rkvenc_run - state of one hardware job
 */
struct rkvenc_run {
	struct vb2_v4l2_buffer *src;
	struct vb2_v4l2_buffer *dst;
	/* Bytes written by software at the start of @dst (parameter sets). */
	unsigned int hdr_bytes;
};

/**
 * struct rkvenc_codec_ops - per-codec backend
 * @adjust_fmt: adjust the coded (CAPTURE) format
 * @start: called on STREAMON of both queues, allocates codec state
 * @stop: called on STREAMOFF, frees codec state
 * @run: prepare the job: produce software headers and fill @core's shadow
 *	 registers (codec syntax, rate control and references)
 * @done: called after the hardware finished; @fb holds the status read-back.
 *	  The backend sets the CAPTURE payload and may change @state.
 */
struct rkvenc_codec_ops {
	int (*adjust_fmt)(struct rkvenc_ctx *ctx, struct v4l2_format *f);
	int (*start)(struct rkvenc_ctx *ctx);
	void (*stop)(struct rkvenc_ctx *ctx);
	int (*run)(struct rkvenc_ctx *ctx, struct rkvenc_core *core,
		   struct rkvenc_run *run);
	void (*done)(struct rkvenc_ctx *ctx, struct rkvenc_run *run,
		     const struct rkvenc_feedback *fb,
		     enum vb2_buffer_state *state);
};

struct rkvenc_ctrls {
	const struct v4l2_ctrl_config *ctrls;
	unsigned int num_ctrls;
};

/**
 * struct rkvenc_coded_fmt_desc - coded (CAPTURE) format description
 */
struct rkvenc_coded_fmt_desc {
	u32 fourcc;
	struct v4l2_frmsize_stepwise frmsize;
	const struct rkvenc_ctrls *ctrls;
	const struct rkvenc_codec_ops *ops;
};

/**
 * struct rkvenc_raw_fmt - raw (OUTPUT) format and its hardware mapping
 */
struct rkvenc_raw_fmt {
	u32 fourcc;
	u8 hw_fmt;
	u8 rbuv_swap;
};

struct rkvenc_dev;

/**
 * struct rkvenc_core - one VEPU580 hardware core
 */
struct rkvenc_core {
	struct device *dev;
	struct rkvenc_dev *rkvenc;
	void __iomem *regs;
	struct clk_bulk_data *clocks;
	int num_clocks;
	struct clk *core_clk;
	struct reset_control *rstc;
	struct iommu_domain *empty_domain;
	struct delayed_work watchdog_work;
	/* Context whose job is running, protected by rkvenc->cores_lock. */
	struct rkvenc_ctx *curr_ctx;
	struct rkvenc_run run;
	/* Shadow of the frame configuration registers. */
	u32 *shadow;
	unsigned long irq_count;
	int id;
};

/**
 * struct rkvenc_dev - V4L2 device, shared by all cores
 */
struct rkvenc_dev {
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct device *dev;
	/* Serializes ioctls. */
	struct mutex vdev_lock;
	struct rkvenc_core **cores;
	unsigned int num_cores;
	unsigned int core_count;
	unsigned long idle_cores;
	/* Protects idle_cores and core->curr_ctx. */
	spinlock_t cores_lock;
	struct rkvenc_core *main_core;
	struct iommu_domain *iommu_domain;
	/* The cores are bound and the V4L2 devices registered. */
	bool bound;
	/* H.264 flat quantisation tables, generated once. */
	u32 *h264_scl;
};

/**
 * struct rkvenc_h264_ctx - H.264 per-context state
 */
struct rkvenc_h264_ctx {
	struct v4l2_h264_enc enc;
	bool hw_rc;
};

struct rkvenc_ctx {
	struct v4l2_fh fh;
	struct rkvenc_dev *dev;
	struct v4l2_format coded_fmt;
	struct v4l2_format src_fmt;
	const struct rkvenc_coded_fmt_desc *coded_fmt_desc;
	const struct rkvenc_raw_fmt *raw_fmt;
	struct v4l2_fract timeperframe;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct rkvenc_core *core;
	struct v4l2_ctrl *avg_qp_ctrl;
	void *priv;
	bool started;
};

static inline struct rkvenc_ctx *file_to_rkvenc_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct rkvenc_ctx, fh);
}

static inline void rkvenc_set(struct rkvenc_core *core, u32 reg, u32 val)
{
	core->shadow[reg / 4] = val;
}

static inline void rkvenc_or(struct rkvenc_core *core, u32 reg, u32 val)
{
	core->shadow[reg / 4] |= val;
}

/* Shared VEPU580 programming helpers (rkvenc-vepu580.c). */
int rkvenc_alloc_aux(struct rkvenc_dev *rkvenc, struct rkvenc_aux_buf *buf,
		     size_t size, bool kernel_mapping);
void rkvenc_free_aux(struct rkvenc_dev *rkvenc, struct rkvenc_aux_buf *buf);
void vepu580_setup_common(struct rkvenc_ctx *ctx, struct rkvenc_core *core,
			  struct rkvenc_run *run);
void vepu580_setup_me(struct rkvenc_core *core, unsigned int width_mbs,
		      unsigned int height_mbs, unsigned int level_idc,
		      bool intra);
void vepu580_setup_h264_tuning(struct rkvenc_dev *rkvenc,
			       struct rkvenc_core *core, bool intra);
void vepu580_setup_row_rc(struct rkvenc_core *core, unsigned int qp,
			  unsigned int qp_min, unsigned int qp_max,
			  unsigned int qp_range, u64 target_bits,
			  unsigned int num_blocks, unsigned int blocks_per_row);
void vepu580_setup_fixed_qp(struct rkvenc_core *core, unsigned int qp);
u32 *vepu580_gen_h264_scl(struct device *dev);
dma_addr_t rkvenc_dst_addr(struct rkvenc_run *run);
size_t rkvenc_dst_size(struct rkvenc_run *run);

extern const struct rkvenc_coded_fmt_desc rkvenc_h264_fmt_desc;

#endif /* RKVENC_H_ */
