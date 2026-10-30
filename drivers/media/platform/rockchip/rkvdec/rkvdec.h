/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip Video Decoder driver
 *
 * Copyright (C) 2019 Collabora, Ltd.
 *
 * Based on rkvdec driver by Google LLC. (Tomasz Figa <tfiga@chromium.org>)
 * Based on s5p-mfc driver by Samsung Electronics Co., Ltd.
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 */
#ifndef RKVDEC_H_
#define RKVDEC_H_

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/spinlock.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#define RKVDEC_QUIRK_DISABLE_QOS	BIT(0)

#define RKVDEC_1080P_PIXELS		(1920 * 1088)
#define RKVDEC_4K_PIXELS		(4096 * 2304)
#define RKVDEC_8K_PIXELS		(7680 * 4320)

/* Maximum number of frames of one context decoded at the same time */
#define RKVDEC_MAX_JOBS			2

struct rkvdec_ctx;
struct rkvdec_core;
struct rkvdec_ccu;
struct rkvdec_rcb_config;

struct rkvdec_ctrl_desc {
	struct v4l2_ctrl_config cfg;
};

struct rkvdec_ctrls {
	const struct rkvdec_ctrl_desc *ctrls;
	unsigned int num_ctrls;
};

struct rkvdec_run {
	struct {
		struct vb2_v4l2_buffer *src;
		struct vb2_v4l2_buffer *dst;
	} bufs;
};

struct rkvdec_vp9_decoded_buffer_info {
	/* Info needed when the decoded frame serves as a reference frame. */
	unsigned short width;
	unsigned short height;
	unsigned int bit_depth : 4;
	/* Tells the frame apart from the other frames of the stream in the CCU */
	u16 poc;
};

struct rkvdec_decoded_buffer {
	/* Must be the first field in this struct. */
	struct v4l2_m2m_buffer base;

	union {
		struct rkvdec_vp9_decoded_buffer_info vp9;
	};
};

static inline struct rkvdec_decoded_buffer *
vb2_to_rkvdec_decoded_buf(struct vb2_buffer *buf)
{
	return container_of(buf, struct rkvdec_decoded_buffer,
			    base.vb.vb2_buf);
}

struct rkvdec_core;

struct rkvdec_variant_ops {
	irqreturn_t (*irq_handler)(struct rkvdec_core *core);
	bool (*reset)(struct rkvdec_core *core);
	bool (*wait_stopped)(struct rkvdec_core *core);
	u32 (*colmv_size)(u16 width, u16 height);
	void (*flatten_matrices)(u8 *output, const u8 *input, int matrices, int row_length);
};

struct rkvdec_variant {
	unsigned int num_regs;
	const struct rkvdec_coded_fmt_desc *coded_fmts;
	size_t num_coded_fmts;
	const struct rcb_size_info *rcb_sizes;
	size_t num_rcb_sizes;
	const struct rkvdec_variant_ops *ops;
	bool has_single_reg_region;
	unsigned int quirks;
	bool has_ccu;
	bool has_cache_ctrl;
};

struct rkvdec_coded_fmt_ops {
	int (*adjust_fmt)(struct rkvdec_ctx *ctx,
			  struct v4l2_format *f);
	int (*start)(struct rkvdec_ctx *ctx);
	void (*stop)(struct rkvdec_ctx *ctx);
	int (*run)(struct rkvdec_ctx *ctx);
	void (*done)(struct rkvdec_ctx *ctx, struct vb2_v4l2_buffer *src_buf,
		     struct vb2_v4l2_buffer *dst_buf,
		     enum vb2_buffer_state result);
	int (*try_ctrl)(struct rkvdec_ctx *ctx, struct v4l2_ctrl *ctrl);
	enum rkvdec_image_fmt (*get_image_fmt)(struct rkvdec_ctx *ctx,
					       struct v4l2_ctrl *ctrl);
};

enum rkvdec_image_fmt {
	RKVDEC_IMG_FMT_ANY = 0,
	RKVDEC_IMG_FMT_420_8BIT,
	RKVDEC_IMG_FMT_420_10BIT,
	RKVDEC_IMG_FMT_422_8BIT,
	RKVDEC_IMG_FMT_422_10BIT,
};

struct rkvdec_decoded_fmt_desc {
	u32 fourcc;
	enum rkvdec_image_fmt image_fmt;
};

/*
 * The next frame of a stream may be decoded while the previous one, which it
 * may predict from, is still being decoded on another core: the hardware
 * synchronizes the reference reads with the core coordination unit.
 */
#define RKVDEC_FMT_FLAG_PIPELINE	BIT(0)

struct rkvdec_coded_fmt_desc {
	u32 fourcc;
	struct v4l2_frmsize_stepwise frmsize;
	const struct rkvdec_ctrls *ctrls;
	const struct rkvdec_coded_fmt_ops *ops;
	unsigned int num_decoded_fmts;
	const struct rkvdec_decoded_fmt_desc *decoded_fmts;
	u32 subsystem_flags;
	u32 flags;
};

/* Largest film index, which tells the streams apart in the CCU */
#define RKVDEC_MAX_FILM_IDX		1023

/* Maximum number of reference frames of a job */
#define RKVDEC_MAX_REFS			17

/**
 * struct rkvdec_job - a frame being decoded
 * @node: entry in the &rkvdec_ctx.jobs list
 * @ctx: the context the frame belongs to
 * @core: the core decoding the frame, NULL once the hardware is done
 * @src: the OUTPUT buffer
 * @dst: the CAPTURE buffer
 * @state: the result of the decoding
 * @slot: index of the job in &rkvdec_ctx.job_slots, selects the per-job
 *	  auxiliary buffers of the coded format
 * @detached: the buffers were taken off the m2m queues and the m2m job is
 *	      finished, so the next frame may start before this one is done
 * @hw_done: the hardware is done with the frame
 * @busy: the m2m state of the job is being changed, don't complete it
 * @started: the hardware was started, or the job failed before that
 * @dst_shared: an earlier job wrote @dst too (the other field of the frame)
 *		and left returning it to this one
 * @dst_error: an earlier job that wrote @dst failed
 * @num_refs: number of entries in @refs
 * @refs: the CAPTURE buffers the frame predicts from
 * @num_pocs: number of entries in @pocs
 * @pocs: the POCs of the frame (H.264: top and bottom field)
 * @field_pic: an H.264 field picture
 * @num_ref_pocs: number of entries in @ref_pocs
 * @ref_pocs: the POCs of the references of the frame
 */
struct rkvdec_job {
	struct list_head node;
	struct rkvdec_ctx *ctx;
	struct rkvdec_core *core;
	struct vb2_v4l2_buffer *src;
	struct vb2_v4l2_buffer *dst;
	enum vb2_buffer_state state;
	unsigned int slot;
	bool detached;
	bool hw_done;
	bool busy;
	bool started;
	bool dst_shared;
	bool dst_error;
	unsigned int num_refs;
	const struct vb2_buffer *refs[RKVDEC_MAX_REFS];
	unsigned int num_pocs;
	s32 pocs[2];
	bool field_pic;
	unsigned int num_ref_pocs;
	s32 ref_pocs[2 * RKVDEC_MAX_REFS];
};

struct rkvdec_core {
	struct device *dev;
	struct rkvdec_dev *rkvdec;
	const struct rkvdec_variant *variant;
	struct clk_bulk_data *clocks;
	unsigned int num_clocks;
	struct clk *axi_clk;
	void __iomem *regs;
	void __iomem *link;
	struct delayed_work watchdog_work;
	struct reset_control *resets;
	int irq;
	bool broken;
	bool stopped;
	struct gen_pool *sram_pool;
	struct iommu_domain *empty_domain;
	bool needs_reset;
	struct rkvdec_rcb_config *rcb_config;
	struct rkvdec_ctx *curr_ctx;
	struct rkvdec_job *job;
	void __iomem *cache;
	int id;
	int ccu_port;
};

struct rkvdec_dev {
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex vdev_lock; /* serializes ioctls */
	const struct rkvdec_variant *variant;
	struct rkvdec_core **cores;
	int core_count;
	struct rkvdec_core **available_cores;
	unsigned int available_core_count;
	spinlock_t cores_lock; /* protects the cores, the job lists and held_job */
	struct rkvdec_core *main_core;
	struct iommu_domain *iommu_global_domain;
	struct rkvdec_job *held_job;
	struct rkvdec_ccu *ccu;
	struct ida film_ida;
	int next_film_idx;
	bool ccu_reset_pending;
	struct rkvdec_ctx *deferred_ctx;
	wait_queue_head_t reset_wq;
	struct work_struct reset_work;
};

struct rkvdec_ctx {
	struct v4l2_fh fh;
	struct v4l2_format coded_fmt;
	struct v4l2_format decoded_fmt;
	const struct rkvdec_coded_fmt_desc *coded_fmt_desc;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct rkvdec_dev *dev;
	enum rkvdec_image_fmt image_fmt;
	u32 colmv_offset;
	/* The core and job being set up, only valid in &rkvdec_coded_fmt_ops.run */
	struct rkvdec_core *core;
	struct rkvdec_job *job;
	struct rkvdec_job job_slots[RKVDEC_MAX_JOBS];
	unsigned int num_job_slots;
	unsigned long busy_slots;
	struct list_head jobs;
	unsigned int num_jobs;
	unsigned int num_completers; /* pin the context while completing jobs */
	wait_queue_head_t jobs_wq;
	/* completes the jobs in decoding order, serializes it with new jobs */
	struct mutex done_lock;
	int film_idx;
	void *priv;
	u8 has_sps_st_rps: 1;
	u8 has_sps_lt_rps: 1;
};

static inline struct rkvdec_ctx *file_to_rkvdec_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct rkvdec_ctx, fh);
}

enum rkvdec_alloc_type {
	RKVDEC_ALLOC_DMA  = 0,
	RKVDEC_ALLOC_SRAM = 1,
};

struct rkvdec_aux_buf {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	enum rkvdec_alloc_type type;
};

void rkvdec_run_preamble(struct rkvdec_ctx *ctx, struct rkvdec_run *run);
void rkvdec_run_postamble(struct rkvdec_ctx *ctx, struct rkvdec_run *run);
void rkvdec_memcpy_toio(void __iomem *dst, void *src, size_t len);
void rkvdec_schedule_watchdog(struct rkvdec_core *core, u32 timeout_threshold);
void rkvdec_job_add_ref(struct rkvdec_ctx *ctx, const struct vb2_buffer *ref);
void rkvdec_job_add_poc(struct rkvdec_ctx *ctx, s32 poc, bool ref);
int rkvdec_job_wait_poc_alias(struct rkvdec_ctx *ctx);

void rkvdec_quirks_disable_qos(struct rkvdec_ctx *ctx);

/* RKVDEC ops */
extern const struct rkvdec_coded_fmt_ops rkvdec_h264_fmt_ops;
extern const struct rkvdec_coded_fmt_ops rkvdec_hevc_fmt_ops;
extern const struct rkvdec_coded_fmt_ops rkvdec_vp9_fmt_ops;

/* VDPU381 ops */
extern const struct rkvdec_coded_fmt_ops rkvdec_vdpu381_h264_fmt_ops;
extern const struct rkvdec_coded_fmt_ops rkvdec_vdpu381_hevc_fmt_ops;
extern const struct rkvdec_coded_fmt_ops rkvdec_vdpu381_vp9_fmt_ops;

/* VDPU383 ops */
extern const struct rkvdec_coded_fmt_ops rkvdec_vdpu383_h264_fmt_ops;
extern const struct rkvdec_coded_fmt_ops rkvdec_vdpu383_hevc_fmt_ops;

#endif /* RKVDEC_H_ */
