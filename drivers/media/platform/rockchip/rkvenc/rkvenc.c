// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip VEPU580 (RKVENC) stateless video encoder driver
 *
 * The VEPU580 found in the RK3588 has two identical cores, each behind its
 * own IOMMU. Every core is a component bound to one virtual master device
 * that owns the V4L2 m2m device; jobs from different contexts run on
 * whichever core is idle (frame-level scheduling across streams, the
 * dual-core handshake is not used).
 *
 * The power, clock, reset and error recovery sequences follow the Rockchip
 * BSP mpp_rkvenc2 driver as found in its out-of-tree port.
 */

#include <linux/bitfield.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>

#include "rkvenc.h"

#define RKVENC_NAME			"rockchip-venc"
#define RKVENC_SW_TIMEOUT_MS		1000

static const struct rkvenc_raw_fmt rkvenc_raw_fmts[] = {
	{ V4L2_PIX_FMT_NV12, VEPU580_FMT_YUV420SP, 0 },
	{ V4L2_PIX_FMT_NV12M, VEPU580_FMT_YUV420SP, 0 },
	{ V4L2_PIX_FMT_NV21, VEPU580_FMT_YUV420SP, 1 },
	{ V4L2_PIX_FMT_YUV420, VEPU580_FMT_YUV420P, 0 },
	{ V4L2_PIX_FMT_YUV420M, VEPU580_FMT_YUV420P, 0 },
	{ V4L2_PIX_FMT_YUYV, VEPU580_FMT_YUYV422, 0 },
	{ V4L2_PIX_FMT_UYVY, VEPU580_FMT_UYVY422, 0 },
};

static const struct rkvenc_coded_fmt_desc *rkvenc_coded_fmts[] = {
	&rkvenc_h264_fmt_desc,
};

static const struct rkvenc_coded_fmt_desc *rkvenc_find_coded_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rkvenc_coded_fmts); i++)
		if (rkvenc_coded_fmts[i]->fourcc == fourcc)
			return rkvenc_coded_fmts[i];

	return NULL;
}

static const struct rkvenc_raw_fmt *rkvenc_find_raw_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rkvenc_raw_fmts); i++)
		if (rkvenc_raw_fmts[i].fourcc == fourcc)
			return &rkvenc_raw_fmts[i];

	return NULL;
}

static void rkvenc_fill_coded_fmt(struct rkvenc_ctx *ctx,
				  struct v4l2_pix_format_mplane *pix)
{
	const struct rkvenc_coded_fmt_desc *desc;
	u32 min_size;

	desc = rkvenc_find_coded_fmt(pix->pixelformat);
	if (!desc) {
		desc = rkvenc_coded_fmts[0];
		pix->pixelformat = desc->fourcc;
	}

	v4l2_apply_frmsize_constraints(&pix->width, &pix->height,
				       &desc->frmsize);

	pix->num_planes = 1;
	pix->field = V4L2_FIELD_NONE;
	pix->plane_fmt[0].bytesperline = 0;

	/* Enough for an intra frame at a very low QP. */
	min_size = ALIGN(pix->width, 16) * ALIGN(pix->height, 16) * 3 / 2;
	min_size = max_t(u32, min_size, SZ_256K);
	if (pix->plane_fmt[0].sizeimage < min_size / 4)
		pix->plane_fmt[0].sizeimage = min_size;
	pix->plane_fmt[0].sizeimage = PAGE_ALIGN(pix->plane_fmt[0].sizeimage);
}

static void rkvenc_fill_raw_fmt(struct v4l2_pix_format_mplane *pix,
				u32 width, u32 height)
{
	if (!rkvenc_find_raw_fmt(pix->pixelformat))
		pix->pixelformat = rkvenc_raw_fmts[0].fourcc;

	/*
	 * Lines are padded to 16 bytes and the allocation covers the
	 * macroblock aligned height, in case the encoder reads the padding
	 * rows. Planes follow the visible height.
	 */
	v4l2_fill_pixfmt_mp(pix, pix->pixelformat, ALIGN(width, 16),
			    ALIGN(height, 16));
	if (pix->num_planes == 1) {
		const struct v4l2_format_info *info =
			v4l2_format_info(pix->pixelformat);

		/* Single plane formats need their planes at the visible height. */
		if (info && info->comp_planes > 1) {
			struct v4l2_pix_format_mplane tmp = *pix;

			v4l2_fill_pixfmt_mp(&tmp, pix->pixelformat,
					    ALIGN(width, 16), height);
			pix->plane_fmt[0].sizeimage =
				max(pix->plane_fmt[0].sizeimage,
				    tmp.plane_fmt[0].sizeimage +
				    pix->plane_fmt[0].bytesperline * 16);
		}
	}
	pix->width = width;
	pix->height = height;
	pix->field = V4L2_FIELD_NONE;
}

static void rkvenc_reset_fmts(struct rkvenc_ctx *ctx)
{
	struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	struct v4l2_pix_format_mplane *raw = &ctx->src_fmt.fmt.pix_mp;

	memset(&ctx->coded_fmt, 0, sizeof(ctx->coded_fmt));
	ctx->coded_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	coded->pixelformat = rkvenc_coded_fmts[0]->fourcc;
	coded->width = 1920;
	coded->height = 1080;
	rkvenc_fill_coded_fmt(ctx, coded);
	ctx->coded_fmt_desc = rkvenc_coded_fmts[0];

	memset(&ctx->src_fmt, 0, sizeof(ctx->src_fmt));
	ctx->src_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	raw->pixelformat = rkvenc_raw_fmts[0].fourcc;
	rkvenc_fill_raw_fmt(raw, coded->width, coded->height);
	ctx->raw_fmt = &rkvenc_raw_fmts[0];

	ctx->timeperframe.numerator = 1;
	ctx->timeperframe.denominator = 30;
}

/* ---- ioctls ---- */

static int rkvenc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, RKVENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, "rockchip,vepu580", sizeof(cap->card));
	return 0;
}

static int rkvenc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	const struct rkvenc_coded_fmt_desc *desc;

	if (fsize->index)
		return -EINVAL;

	desc = rkvenc_find_coded_fmt(fsize->pixel_format);
	if (!desc) {
		if (!rkvenc_find_raw_fmt(fsize->pixel_format))
			return -EINVAL;
		desc = rkvenc_coded_fmts[0];
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = desc->frmsize;
	return 0;
}

static int rkvenc_enum_frameintervals(struct file *file, void *priv,
				      struct v4l2_frmivalenum *fival)
{
	if (fival->index)
		return -EINVAL;

	if (!rkvenc_find_coded_fmt(fival->pixel_format) &&
	    !rkvenc_find_raw_fmt(fival->pixel_format))
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	fival->stepwise.min.numerator = 1;
	fival->stepwise.min.denominator = 240;
	fival->stepwise.max.numerator = 1;
	fival->stepwise.max.denominator = 1;
	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 1;
	return 0;
}

static int rkvenc_enum_capture_fmt(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(rkvenc_coded_fmts))
		return -EINVAL;

	f->pixelformat = rkvenc_coded_fmts[f->index]->fourcc;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int rkvenc_enum_output_fmt(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(rkvenc_raw_fmts))
		return -EINVAL;

	f->pixelformat = rkvenc_raw_fmts[f->index].fourcc;
	return 0;
}

static int rkvenc_g_capture_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);

	*f = ctx->coded_fmt;
	return 0;
}

static int rkvenc_g_output_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);

	*f = ctx->src_fmt;
	return 0;
}

static int rkvenc_try_capture_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);
	const struct rkvenc_coded_fmt_desc *desc;

	rkvenc_fill_coded_fmt(ctx, &f->fmt.pix_mp);

	desc = rkvenc_find_coded_fmt(f->fmt.pix_mp.pixelformat);
	if (desc->ops->adjust_fmt)
		return desc->ops->adjust_fmt(ctx, f);

	return 0;
}

static int rkvenc_try_output_fmt(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct rkvenc_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	u32 width = pix->width, height = pix->height;

	v4l2_apply_frmsize_constraints(&width, &height, &desc->frmsize);
	rkvenc_fill_raw_fmt(pix, width, height);

	return 0;
}

static int rkvenc_s_capture_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct v4l2_pix_format_mplane *raw = &ctx->src_fmt.fmt.pix_mp;
	int ret;

	if (ctx->started || vb2_is_busy(v4l2_m2m_get_vq(m2m_ctx, f->type)))
		return -EBUSY;

	ret = rkvenc_try_capture_fmt(file, priv, f);
	if (ret)
		return ret;

	/* Both queues must always describe the same picture size. */
	if (vb2_is_busy(v4l2_m2m_get_src_vq(m2m_ctx)) &&
	    (f->fmt.pix_mp.width != raw->width ||
	     f->fmt.pix_mp.height != raw->height))
		return -EBUSY;

	ctx->coded_fmt = *f;
	ctx->coded_fmt_desc = rkvenc_find_coded_fmt(f->fmt.pix_mp.pixelformat);

	/* The raw format follows the coded size. */
	if (!vb2_is_busy(v4l2_m2m_get_src_vq(m2m_ctx)))
		rkvenc_fill_raw_fmt(raw, f->fmt.pix_mp.width,
				    f->fmt.pix_mp.height);

	return 0;
}

static int rkvenc_s_output_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	int ret;

	if (ctx->started || vb2_is_busy(v4l2_m2m_get_vq(m2m_ctx, f->type)))
		return -EBUSY;

	ret = rkvenc_try_output_fmt(file, priv, f);
	if (ret)
		return ret;

	if (vb2_is_busy(v4l2_m2m_get_dst_vq(m2m_ctx)) &&
	    (f->fmt.pix_mp.width != coded->width ||
	     f->fmt.pix_mp.height != coded->height))
		return -EBUSY;

	ctx->src_fmt = *f;
	ctx->raw_fmt = rkvenc_find_raw_fmt(f->fmt.pix_mp.pixelformat);

	/* The coded format takes the size and colorimetry of the input. */
	if (!vb2_is_busy(v4l2_m2m_get_dst_vq(m2m_ctx))) {
		coded->width = f->fmt.pix_mp.width;
		coded->height = f->fmt.pix_mp.height;
		rkvenc_fill_coded_fmt(ctx, coded);
	}
	coded->colorspace = f->fmt.pix_mp.colorspace;
	coded->xfer_func = f->fmt.pix_mp.xfer_func;
	coded->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	coded->quantization = f->fmt.pix_mp.quantization;

	return 0;
}

static int rkvenc_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *sp)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);

	if (!V4L2_TYPE_IS_OUTPUT(sp->type))
		return -EINVAL;

	sp->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	sp->parm.output.timeperframe = ctx->timeperframe;
	return 0;
}

static int rkvenc_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *sp)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(file);
	struct v4l2_fract *tpf = &sp->parm.output.timeperframe;

	if (!V4L2_TYPE_IS_OUTPUT(sp->type))
		return -EINVAL;

	if (!tpf->numerator || !tpf->denominator) {
		tpf->numerator = 1;
		tpf->denominator = 30;
	}

	ctx->timeperframe = *tpf;
	sp->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	return 0;
}

static const struct v4l2_ioctl_ops rkvenc_ioctl_ops = {
	.vidioc_querycap = rkvenc_querycap,
	.vidioc_enum_framesizes = rkvenc_enum_framesizes,
	.vidioc_enum_frameintervals = rkvenc_enum_frameintervals,

	.vidioc_try_fmt_vid_cap_mplane = rkvenc_try_capture_fmt,
	.vidioc_try_fmt_vid_out_mplane = rkvenc_try_output_fmt,
	.vidioc_s_fmt_vid_out_mplane = rkvenc_s_output_fmt,
	.vidioc_s_fmt_vid_cap_mplane = rkvenc_s_capture_fmt,
	.vidioc_g_fmt_vid_out_mplane = rkvenc_g_output_fmt,
	.vidioc_g_fmt_vid_cap_mplane = rkvenc_g_capture_fmt,
	.vidioc_enum_fmt_vid_out = rkvenc_enum_output_fmt,
	.vidioc_enum_fmt_vid_cap = rkvenc_enum_capture_fmt,

	.vidioc_g_parm = rkvenc_g_parm,
	.vidioc_s_parm = rkvenc_s_parm,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

/* ---- vb2 ---- */

static struct v4l2_format *rkvenc_queue_fmt(struct rkvenc_ctx *ctx,
					    struct vb2_queue *vq)
{
	return V4L2_TYPE_IS_OUTPUT(vq->type) ? &ctx->src_fmt : &ctx->coded_fmt;
}

static int rkvenc_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format_mplane *pix = &rkvenc_queue_fmt(ctx, vq)->fmt.pix_mp;
	unsigned int i;

	if (*num_planes) {
		if (*num_planes != pix->num_planes)
			return -EINVAL;

		for (i = 0; i < pix->num_planes; i++)
			if (sizes[i] < pix->plane_fmt[i].sizeimage)
				return -EINVAL;

		return 0;
	}

	*num_planes = pix->num_planes;
	for (i = 0; i < pix->num_planes; i++)
		sizes[i] = pix->plane_fmt[i].sizeimage;

	return 0;
}

static int rkvenc_buf_prepare(struct vb2_buffer *vb)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *pix =
		&rkvenc_queue_fmt(ctx, vb->vb2_queue)->fmt.pix_mp;
	unsigned int i;

	for (i = 0; i < pix->num_planes; i++) {
		if (vb2_plane_size(vb, i) < pix->plane_fmt[i].sizeimage)
			return -EINVAL;
	}

	return 0;
}

static void rkvenc_buf_queue(struct vb2_buffer *vb)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int rkvenc_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static void rkvenc_buf_request_complete(struct vb2_buffer *vb)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static void rkvenc_queue_cleanup(struct vb2_queue *vq, u32 state)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			break;

		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req,
					   &ctx->ctrl_hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static int rkvenc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(q);
	const struct rkvenc_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	int ret;

	if (ctx->started)
		return 0;

	if (ctx->src_fmt.fmt.pix_mp.width != ctx->coded_fmt.fmt.pix_mp.width ||
	    ctx->src_fmt.fmt.pix_mp.height != ctx->coded_fmt.fmt.pix_mp.height)
		ret = -EINVAL;
	else
		ret = desc->ops->start ? desc->ops->start(ctx) : 0;
	if (ret) {
		rkvenc_queue_cleanup(q, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	ctx->started = true;
	return 0;
}

static void rkvenc_stop_streaming(struct vb2_queue *q)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(q);
	const struct rkvenc_coded_fmt_desc *desc = ctx->coded_fmt_desc;

	/* v4l2_m2m_streamoff() already waited for this context's job. */
	if (ctx->started) {
		if (desc->ops->stop)
			desc->ops->stop(ctx);
		ctx->started = false;
	}

	rkvenc_queue_cleanup(q, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops rkvenc_queue_ops = {
	.queue_setup = rkvenc_queue_setup,
	.buf_prepare = rkvenc_buf_prepare,
	.buf_queue = rkvenc_buf_queue,
	.buf_out_validate = rkvenc_buf_out_validate,
	.buf_request_complete = rkvenc_buf_request_complete,
	.start_streaming = rkvenc_start_streaming,
	.stop_streaming = rkvenc_stop_streaming,
};

static int rkvenc_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct rkvenc_ctx *ctx = priv;
	struct rkvenc_dev *rkvenc = ctx->dev;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &rkvenc_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES |
			    DMA_ATTR_NO_KERNEL_MAPPING;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &rkvenc->vdev_lock;
	src_vq->dev = rkvenc->v4l2_dev.dev;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	/* The CPU writes the parameter sets: CAPTURE needs a kernel mapping. */
	/*
	 * MMAP only: the headers are written through the coherent kernel
	 * mapping; imported DMABUFs would need CPU cache maintenance.
	 */
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &rkvenc_queue_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &rkvenc->vdev_lock;
	dst_vq->dev = rkvenc->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

/* ---- hardware ---- */

static void rkvenc_write_range(struct rkvenc_core *core, u32 start, u32 end)
{
	u32 reg;

	for (reg = start; reg < end; reg += 4)
		writel_relaxed(core->shadow[reg / 4], core->regs + reg);
}

static u32 rkvenc_wdg_threshold(struct rkvenc_core *core)
{
	/* Hardware watchdog per picture size, as in the BSP driver. */
	static const struct {
		u32 pixels;
		u32 ms;
	} timeouts[] = {
		{ 1920 * 1088, 50 },
		{ 2560 * 1440, 100 },
		{ 4096 * 2304, 200 },
		{ 8192 * 8192, 400 },
	};
	unsigned long rate = core->core_clk ? clk_get_rate(core->core_clk) : 0;
	u32 rsl = core->shadow[VEPU580_ENC_RSL / 4];
	u32 pixels = (FIELD_GET(VEPU580_ENC_RSL_PIC_WD8_M1, rsl) + 1) *
		     (FIELD_GET(VEPU580_ENC_RSL_PIC_HD8_M1, rsl) + 1) * 64;
	u32 ms = 800;
	unsigned int i;
	u64 thd;

	if (!rate)
		return VEPU580_ENC_WDG_VS_LOAD_THD;

	for (i = 0; i < ARRAY_SIZE(timeouts); i++) {
		if (pixels <= timeouts[i].pixels) {
			ms = timeouts[i].ms;
			break;
		}
	}

	/*
	 * The BSP counts in units of 256 core clock cycles, its out-of-tree
	 * port in units of 1024. Using the former errs on the long side.
	 */
	thd = (u64)ms * (rate / 256000);
	return min_t(u64, thd, VEPU580_ENC_WDG_VS_LOAD_THD);
}

static void rkvenc_hw_start(struct rkvenc_core *core)
{
	/*
	 * VEPU580 erratum: force-clear the core before every frame or stale
	 * state may cause IOMMU page faults.
	 */
	writel(VEPU580_ENC_CLR_FORCE, core->regs + VEPU580_ENC_CLR);
	udelay(5);
	writel(0, core->regs + VEPU580_ENC_CLR);

	writel_relaxed(VEPU580_DBG_AXIP0_CLR, core->regs + VEPU580_DBG_AXIP0_CMD);

	writel_relaxed(VEPU580_INT_ENC_DONE | VEPU580_INT_ERROR,
		       core->regs + VEPU580_INT_EN);
	writel_relaxed(0, core->regs + VEPU580_INT_MSK);
	writel_relaxed(VEPU580_INT_ALL, core->regs + VEPU580_INT_CLR);

	rkvenc_write_range(core, VEPU580_CTL_START, VEPU580_CTL_END);
	writel_relaxed(FIELD_PREP(VEPU580_ENC_WDG_VS_LOAD_THD,
				  rkvenc_wdg_threshold(core)),
		       core->regs + VEPU580_ENC_WDG);
	rkvenc_write_range(core, VEPU580_FRAME_START, VEPU580_FRAME_END);
	rkvenc_write_range(core, VEPU580_KLUT_START, VEPU580_KLUT_END);
	rkvenc_write_range(core, VEPU580_PARAM_START, VEPU580_PARAM_END);
	rkvenc_write_range(core, VEPU580_RDO_START, VEPU580_RDO_END);
	rkvenc_write_range(core, VEPU580_SCL_START, VEPU580_SCL_END);

	schedule_delayed_work(&core->watchdog_work,
			      msecs_to_jiffies(RKVENC_SW_TIMEOUT_MS));

	/* writel() orders all the relaxed writes above before the start. */
	writel(FIELD_PREP(VEPU580_ENC_STRT_VEPU_CMD, 1),
	       core->regs + VEPU580_ENC_STRT);
}

static void rkvenc_iommu_restore(struct rkvenc_core *core)
{
	struct iommu_domain *domain = core->rkvenc->iommu_domain;
	int ret;

	if (!core->empty_domain || !domain)
		return;

	/*
	 * A CRU reset also clears the IOMMU. Attaching an empty domain and
	 * then the shared domain again reprograms the page table base and
	 * flushes the TLB of this core's IOMMU.
	 */
	iommu_detach_device(domain, core->dev);
	ret = iommu_attach_device(core->empty_domain, core->dev);
	if (ret)
		dev_warn(core->dev, "cannot attach empty domain: %d\n", ret);
	iommu_detach_device(core->empty_domain, core->dev);
	ret = iommu_attach_device(domain, core->dev);
	if (ret)
		dev_warn(core->dev, "cannot attach shared domain: %d\n", ret);
}

static void rkvenc_hw_reset(struct rkvenc_core *core)
{
	u32 sta;
	int ret;

	/* Report safe-clear completion in the status, without an interrupt. */
	writel(VEPU580_INT_ALL, core->regs + VEPU580_INT_MSK);
	writel(VEPU580_INT_ALL, core->regs + VEPU580_INT_EN);
	writel(VEPU580_ENC_CLR_SAFE, core->regs + VEPU580_ENC_CLR);
	ret = readl_poll_timeout_atomic(core->regs + VEPU580_INT_STA, sta,
					sta & VEPU580_INT_SCLR_DONE, 1, 100);
	writel(VEPU580_ENC_CLR_FORCE, core->regs + VEPU580_ENC_CLR);
	udelay(5);
	writel(0, core->regs + VEPU580_ENC_CLR);
	writel(VEPU580_INT_ALL, core->regs + VEPU580_INT_CLR);
	writel(0, core->regs + VEPU580_INT_STA);

	if (ret && core->rstc) {
		dev_warn(core->dev, "safe clear timed out, resetting the core\n");
		reset_control_assert(core->rstc);
		udelay(5);
		reset_control_deassert(core->rstc);
		rkvenc_iommu_restore(core);
	}
}

static void rkvenc_read_feedback(struct rkvenc_core *core,
				 struct rkvenc_feedback *fb)
{
	void __iomem *r = core->regs;
	u32 sse_bsl = readl_relaxed(r + VEPU580_ST_SSE_BSL);

	fb->bs_len = readl_relaxed(r + VEPU580_ST_BS_LGTH_L32) |
		     (u64)FIELD_GET(VEPU580_ST_BS_LGTH_H8, sse_bsl) << 32;
	fb->sse = (u64)readl_relaxed(r + VEPU580_ST_SSE_H32) << 16 |
		  FIELD_GET(VEPU580_ST_SSE_L16, sse_bsl);
	fb->qp_sum = readl_relaxed(r + VEPU580_ST_QP_SUM);
	fb->madi = readl_relaxed(r + VEPU580_ST_MADI);
	fb->madp = readl_relaxed(r + VEPU580_ST_MADP);
	fb->intra_blocks = readl_relaxed(r + VEPU580_ST_PNUM_I16) +
			   readl_relaxed(r + VEPU580_ST_PNUM_I8) +
			   readl_relaxed(r + VEPU580_ST_PNUM_I4);
	fb->cycles = readl_relaxed(r + VEPU580_DBG_FRAME_CYC);
}

static void rkvenc_release_core(struct rkvenc_core *core)
{
	struct rkvenc_dev *rkvenc = core->rkvenc;

	guard(spinlock_irqsave)(&rkvenc->cores_lock);
	core->curr_ctx = NULL;
	set_bit(core->id, &rkvenc->idle_cores);
}

static struct rkvenc_core *rkvenc_acquire_core(struct rkvenc_dev *rkvenc,
					       struct rkvenc_ctx *ctx)
{
	struct rkvenc_core *core;
	unsigned int id;

	guard(spinlock_irqsave)(&rkvenc->cores_lock);
	id = find_first_bit(&rkvenc->idle_cores, rkvenc->core_count);
	if (id >= rkvenc->core_count)
		return NULL;

	clear_bit(id, &rkvenc->idle_cores);
	core = rkvenc->cores[id];
	core->curr_ctx = ctx;
	return core;
}

static void rkvenc_job_finish(struct rkvenc_core *core,
			      enum vb2_buffer_state state,
			      const struct rkvenc_feedback *fb, bool powered)
{
	struct rkvenc_ctx *ctx = core->curr_ctx;
	struct rkvenc_run *run = &core->run;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = run->src, *dst = run->dst;
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct rkvenc_feedback empty = { };

	if (ctx->coded_fmt_desc->ops->done)
		ctx->coded_fmt_desc->ops->done(ctx, run, fb ? fb : &empty, &state);

	if (state != VB2_BUF_STATE_DONE)
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);

	if (powered) {
		pm_runtime_mark_last_busy(core->dev);
		pm_runtime_put_autosuspend(core->dev);
	}

	/*
	 * Everything needed from core->run is in locals now: once the core is
	 * idle another context's device_run() may take it.
	 */
	ctx->core = NULL;
	rkvenc_release_core(core);

	v4l2_m2m_src_buf_remove_by_buf(m2m_ctx, src);
	v4l2_m2m_dst_buf_remove_by_buf(m2m_ctx, dst);
	if (req)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	/* CAPTURE first: the request completes with the OUTPUT buffer. */
	v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_job_finish(ctx->dev->m2m_dev, m2m_ctx);
}

static void rkvenc_device_run(void *priv)
{
	struct rkvenc_ctx *ctx = priv;
	struct rkvenc_dev *rkvenc = ctx->dev;
	const struct rkvenc_codec_ops *ops = ctx->coded_fmt_desc->ops;
	struct vb2_v4l2_buffer *src, *dst;
	struct media_request *req;
	struct rkvenc_core *core;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	core = rkvenc_acquire_core(rkvenc, ctx);
	if (WARN_ON(!core)) {
		if (src->vb2_buf.req_obj.req)
			v4l2_ctrl_request_complete(src->vb2_buf.req_obj.req,
						   &ctx->ctrl_hdl);
		v4l2_m2m_buf_done_and_job_finish(rkvenc->m2m_dev,
						 ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		return;
	}

	ctx->core = core;
	core->run.src = src;
	core->run.dst = dst;
	core->run.hdr_bytes = 0;

	req = src->vb2_buf.req_obj.req;
	if (req)
		v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);

	/* The reference handling keys reconstructed frames by timestamp. */
	v4l2_m2m_buf_copy_metadata(src, dst);

	ret = pm_runtime_resume_and_get(core->dev);
	if (ret < 0) {
		dev_err(core->dev, "cannot power up: %d\n", ret);
		rkvenc_job_finish(core, VB2_BUF_STATE_ERROR, NULL, false);
		return;
	}

	ret = ops->run(ctx, core, &core->run);
	if (ret) {
		dev_dbg(core->dev, "cannot prepare job: %d\n", ret);
		rkvenc_job_finish(core, VB2_BUF_STATE_ERROR, NULL, true);
		return;
	}

	rkvenc_hw_start(core);
}

static int rkvenc_job_ready(void *priv)
{
	struct rkvenc_ctx *ctx = priv;

	return ctx->started;
}

static const struct v4l2_m2m_ops rkvenc_m2m_ops = {
	.device_run = rkvenc_device_run,
	.job_ready = rkvenc_job_ready,
};

static irqreturn_t rkvenc_irq_thread(int irq, void *priv)
{
	struct rkvenc_core *core = priv;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	struct rkvenc_feedback fb = { };
	u32 sta;

	if (pm_runtime_get_if_active(core->dev) <= 0)
		return IRQ_NONE;

	sta = readl(core->regs + VEPU580_INT_STA);
	writel(sta, core->regs + VEPU580_INT_CLR);
	writel(0, core->regs + VEPU580_INT_STA);

	if (!sta) {
		pm_runtime_put_noidle(core->dev);
		return IRQ_NONE;
	}

	core->irq_count++;

	/* Prevent a watchdog interrupt storm. */
	if (sta & VEPU580_INT_WDG)
		writel(VEPU580_INT_WDG, core->regs + VEPU580_INT_MSK);

	if (!(sta & (VEPU580_INT_ENC_DONE | VEPU580_INT_ERROR)) ||
	    !core->curr_ctx || !cancel_delayed_work(&core->watchdog_work)) {
		pm_runtime_put_noidle(core->dev);
		return IRQ_HANDLED;
	}

	fb.int_sta = sta;
	rkvenc_read_feedback(core, &fb);

	if (sta & VEPU580_INT_ERROR) {
		dev_err(core->dev, "encode error, irq status %#x\n", sta);
		state = VB2_BUF_STATE_ERROR;
		rkvenc_hw_reset(core);
	}

	pm_runtime_put_noidle(core->dev);
	rkvenc_job_finish(core, state, &fb, true);

	return IRQ_HANDLED;
}

static void rkvenc_watchdog_func(struct work_struct *work)
{
	struct rkvenc_core *core = container_of(to_delayed_work(work),
						struct rkvenc_core,
						watchdog_work);

	if (!core->curr_ctx)
		return;

	dev_err(core->dev, "frame encoding timed out, irq status %#x\n",
		readl(core->regs + VEPU580_INT_STA));
	rkvenc_hw_reset(core);
	rkvenc_job_finish(core, VB2_BUF_STATE_ERROR, NULL, true);
}

/* ---- file operations ---- */

static int rkvenc_init_ctrls(struct rkvenc_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrl_hdl;
	unsigned int i, j, nctrls = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(rkvenc_coded_fmts); i++)
		nctrls += rkvenc_coded_fmts[i]->ctrls->num_ctrls;

	v4l2_ctrl_handler_init(hdl, nctrls);

	for (i = 0; i < ARRAY_SIZE(rkvenc_coded_fmts); i++) {
		const struct rkvenc_ctrls *ctrls = rkvenc_coded_fmts[i]->ctrls;

		for (j = 0; j < ctrls->num_ctrls; j++)
			v4l2_ctrl_new_custom(hdl, &ctrls->ctrls[j], NULL);
	}

	ctx->avg_qp_ctrl = v4l2_ctrl_find(hdl, V4L2_CID_MPEG_VIDEO_AVERAGE_QP);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	ret = v4l2_ctrl_handler_setup(hdl);
	if (ret) {
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	ctx->fh.ctrl_handler = hdl;
	return 0;
}

static int rkvenc_open(struct file *filp)
{
	struct rkvenc_dev *rkvenc = video_drvdata(filp);
	struct rkvenc_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = rkvenc;
	rkvenc_reset_fmts(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ret = rkvenc_init_ctrls(ctx);
	if (ret)
		goto err_free_ctx;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(rkvenc->m2m_dev, ctx,
					    rkvenc_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_ctrls;
	}

	v4l2_fh_add(&ctx->fh, filp);
	return 0;

err_free_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
err_free_ctx:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int rkvenc_release(struct file *filp)
{
	struct rkvenc_ctx *ctx = file_to_rkvenc_ctx(filp);

	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations rkvenc_fops = {
	.owner = THIS_MODULE,
	.open = rkvenc_open,
	.release = rkvenc_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int rkvenc_request_validate(struct media_request *req)
{
	unsigned int count = vb2_request_buffer_cnt(req);

	if (!count)
		return -ENOENT;
	if (count > 1)
		return -EINVAL;

	return vb2_request_validate(req);
}

static const struct media_device_ops rkvenc_media_ops = {
	.req_validate = rkvenc_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

static int rkvenc_v4l2_init(struct rkvenc_dev *rkvenc)
{
	struct device *dev = rkvenc->main_core->dev;
	int ret;

	ret = v4l2_device_register(dev, &rkvenc->v4l2_dev);
	if (ret)
		return ret;

	rkvenc->m2m_dev = v4l2_m2m_init(&rkvenc_m2m_ops);
	if (IS_ERR(rkvenc->m2m_dev)) {
		ret = PTR_ERR(rkvenc->m2m_dev);
		goto err_unregister_v4l2;
	}

	rkvenc->mdev.dev = dev;
	strscpy(rkvenc->mdev.model, RKVENC_NAME, sizeof(rkvenc->mdev.model));
	/* Must match the bus_info VIDIOC_QUERYCAP reports for the vdev. */
	snprintf(rkvenc->mdev.bus_info, sizeof(rkvenc->mdev.bus_info),
		 "platform:%s", dev_name(dev));
	media_device_init(&rkvenc->mdev);
	rkvenc->mdev.ops = &rkvenc_media_ops;
	rkvenc->v4l2_dev.mdev = &rkvenc->mdev;

	rkvenc->vdev.lock = &rkvenc->vdev_lock;
	rkvenc->vdev.v4l2_dev = &rkvenc->v4l2_dev;
	rkvenc->vdev.fops = &rkvenc_fops;
	rkvenc->vdev.release = video_device_release_empty;
	rkvenc->vdev.vfl_dir = VFL_DIR_M2M;
	rkvenc->vdev.device_caps = V4L2_CAP_STREAMING |
				   V4L2_CAP_VIDEO_M2M_MPLANE;
	rkvenc->vdev.ioctl_ops = &rkvenc_ioctl_ops;
	video_set_drvdata(&rkvenc->vdev, rkvenc);
	strscpy(rkvenc->vdev.name, RKVENC_NAME, sizeof(rkvenc->vdev.name));

	ret = video_register_device(&rkvenc->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_cleanup_mc;

	ret = v4l2_m2m_register_media_controller(rkvenc->m2m_dev, &rkvenc->vdev,
						 MEDIA_ENT_F_PROC_VIDEO_ENCODER);
	if (ret)
		goto err_unregister_vdev;

	ret = media_device_register(&rkvenc->mdev);
	if (ret)
		goto err_unregister_mc;

	v4l2_m2m_set_max_parallel_jobs(rkvenc->m2m_dev, rkvenc->core_count);
	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(rkvenc->m2m_dev);
err_unregister_vdev:
	video_unregister_device(&rkvenc->vdev);
err_cleanup_mc:
	media_device_cleanup(&rkvenc->mdev);
	v4l2_m2m_release(rkvenc->m2m_dev);
err_unregister_v4l2:
	v4l2_device_unregister(&rkvenc->v4l2_dev);
	return ret;
}

static void rkvenc_v4l2_cleanup(struct rkvenc_dev *rkvenc)
{
	media_device_unregister(&rkvenc->mdev);
	v4l2_m2m_unregister_media_controller(rkvenc->m2m_dev);
	video_unregister_device(&rkvenc->vdev);
	media_device_cleanup(&rkvenc->mdev);
	v4l2_m2m_release(rkvenc->m2m_dev);
	v4l2_device_unregister(&rkvenc->v4l2_dev);
}

/* ---- cores ---- */

static int rkvenc_core_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct rkvenc_core *core = dev_get_drvdata(dev);
	struct rkvenc_dev *rkvenc = data;
	int ret;

	if (rkvenc->core_count >= rkvenc->num_cores)
		return -EINVAL;

	/*
	 * All cores share the first core's DMA domain, so that buffers
	 * allocated or imported through the first core are mapped in every
	 * core's IOMMU and a job can run on any core.
	 */
	if (iommu_get_domain_for_dev(dev)) {
		if (!rkvenc->iommu_domain) {
			rkvenc->iommu_domain = iommu_get_domain_for_dev(dev);
		} else {
			ret = iommu_attach_device(rkvenc->iommu_domain, dev);
			if (ret) {
				dev_err(dev, "cannot attach shared IOMMU domain: %d\n",
					ret);
				return ret;
			}
		}
	}

	core->id = rkvenc->core_count;
	core->rkvenc = rkvenc;
	rkvenc->cores[core->id] = core;
	if (!core->id)
		rkvenc->main_core = core;
	rkvenc->core_count++;
	set_bit(core->id, &rkvenc->idle_cores);

	dev_info(dev, "registered encoder core %d\n", core->id);
	return 0;
}

static void rkvenc_core_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct rkvenc_core *core = dev_get_drvdata(dev);
	struct rkvenc_dev *rkvenc = data;

	cancel_delayed_work_sync(&core->watchdog_work);

	if (core->id && rkvenc->iommu_domain)
		iommu_detach_device(rkvenc->iommu_domain, dev);

	clear_bit(core->id, &rkvenc->idle_cores);
	rkvenc->cores[core->id] = NULL;
	rkvenc->core_count--;
	core->rkvenc = NULL;
}

static const struct component_ops rkvenc_core_ops = {
	.bind = rkvenc_core_bind,
	.unbind = rkvenc_core_unbind,
};

static void rkvenc_free_empty_domain(void *data)
{
	iommu_domain_free(data);
}

static int rkvenc_core_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rkvenc_core *core;
	int ret, irq;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	INIT_DELAYED_WORK(&core->watchdog_work, rkvenc_watchdog_func);

	core->shadow = devm_kzalloc(dev, VEPU580_SHADOW_SIZE, GFP_KERNEL);
	if (!core->shadow)
		return -ENOMEM;

	core->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(core->regs))
		return PTR_ERR(core->regs);

	ret = devm_clk_bulk_get_all(dev, &core->clocks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot get clocks\n");
	core->num_clocks = ret;

	core->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(core->core_clk))
		return dev_err_probe(dev, PTR_ERR(core->core_clk),
				     "cannot get core clock\n");

	core->rstc = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(core->rstc))
		return dev_err_probe(dev, PTR_ERR(core->rstc),
				     "cannot get resets\n");

	if (iommu_get_domain_for_dev(dev)) {
		core->empty_domain = iommu_paging_domain_alloc(dev);
		if (IS_ERR(core->empty_domain)) {
			dev_warn(dev, "cannot allocate empty IOMMU domain\n");
			core->empty_domain = NULL;
		} else {
			ret = devm_add_action_or_reset(dev,
						       rkvenc_free_empty_domain,
						       core->empty_domain);
			if (ret)
				return ret;
		}
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "cannot set DMA mask\n");

	vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	platform_set_drvdata(pdev, core);

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, rkvenc_irq_thread,
					IRQF_ONESHOT, dev_name(dev), core);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request IRQ\n");

	ret = component_add(dev, &rkvenc_core_ops);
	if (ret)
		return dev_err_probe(dev, ret, "cannot add component\n");

	return 0;
}

static void rkvenc_core_remove(struct platform_device *pdev)
{
	struct rkvenc_core *core = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &rkvenc_core_ops);
	cancel_delayed_work_sync(&core->watchdog_work);
}

static int rkvenc_core_runtime_resume(struct device *dev)
{
	struct rkvenc_core *core = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(core->num_clocks, core->clocks);
}

static int rkvenc_core_runtime_suspend(struct device *dev)
{
	struct rkvenc_core *core = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(core->num_clocks, core->clocks);
	return 0;
}

static const struct dev_pm_ops rkvenc_core_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(rkvenc_core_runtime_suspend, rkvenc_core_runtime_resume,
		       NULL)
};

static const struct of_device_id rkvenc_core_of_match[] = {
	{ .compatible = "rockchip,rk3588-vepu580" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rkvenc_core_of_match);

static struct platform_driver rkvenc_core_driver = {
	.probe = rkvenc_core_probe,
	.remove = rkvenc_core_remove,
	.driver = {
		.name = RKVENC_NAME "-core",
		.of_match_table = rkvenc_core_of_match,
		.pm = pm_ptr(&rkvenc_core_pm_ops),
	},
};

/* ---- master ---- */

static int rkvenc_master_bind(struct device *dev)
{
	struct rkvenc_dev *rkvenc = dev_get_drvdata(dev);
	int ret;

	ret = component_bind_all(dev, rkvenc);
	if (ret)
		return ret;

	if (!rkvenc->main_core) {
		ret = -ENODEV;
		goto err_unbind;
	}

	ret = rkvenc_v4l2_init(rkvenc);
	if (ret)
		goto err_unbind;

	rkvenc->bound = true;
	return 0;

err_unbind:
	component_unbind_all(dev, rkvenc);
	rkvenc->main_core = NULL;
	rkvenc->iommu_domain = NULL;
	return ret;
}

static void rkvenc_master_unbind(struct device *dev)
{
	struct rkvenc_dev *rkvenc = dev_get_drvdata(dev);

	/* Let running jobs finish before the cores go away. */
	v4l2_m2m_suspend(rkvenc->m2m_dev);
	rkvenc->bound = false;
	rkvenc_v4l2_cleanup(rkvenc);
	component_unbind_all(dev, rkvenc);
	rkvenc->main_core = NULL;
	rkvenc->iommu_domain = NULL;
}

static const struct component_master_ops rkvenc_master_ops = {
	.bind = rkvenc_master_bind,
	.unbind = rkvenc_master_unbind,
};

static int rkvenc_master_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *np;
	struct rkvenc_dev *rkvenc;
	unsigned int num_cores = 0;

	for_each_matching_node(np, rkvenc_core_of_match) {
		if (!of_device_is_available(np))
			continue;

		component_match_add_release(dev, &match, component_release_of,
					    component_compare_of,
					    of_node_get(np));
		num_cores++;
	}

	if (!match)
		return -ENODEV;

	if (num_cores > BITS_PER_LONG)
		num_cores = BITS_PER_LONG;

	rkvenc = devm_kzalloc(dev, sizeof(*rkvenc), GFP_KERNEL);
	if (!rkvenc)
		return -ENOMEM;

	rkvenc->cores = devm_kcalloc(dev, num_cores, sizeof(*rkvenc->cores),
				     GFP_KERNEL);
	if (!rkvenc->cores)
		return -ENOMEM;

	rkvenc->h264_scl = vepu580_gen_h264_scl(dev);
	if (!rkvenc->h264_scl)
		return -ENOMEM;

	rkvenc->dev = dev;
	rkvenc->num_cores = num_cores;
	mutex_init(&rkvenc->vdev_lock);
	spin_lock_init(&rkvenc->cores_lock);
	dev_set_drvdata(dev, rkvenc);

	return component_master_add_with_match(dev, &rkvenc_master_ops, match);
}

static void rkvenc_master_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &rkvenc_master_ops);
}

/*
 * The master device is registered after the cores, so it is suspended
 * before and resumed after them: pause the job queue here so that no job
 * is running when the cores are suspended.
 */
static int rkvenc_master_suspend(struct device *dev)
{
	struct rkvenc_dev *rkvenc = dev_get_drvdata(dev);

	if (rkvenc && rkvenc->bound)
		v4l2_m2m_suspend(rkvenc->m2m_dev);

	return 0;
}

static int rkvenc_master_resume(struct device *dev)
{
	struct rkvenc_dev *rkvenc = dev_get_drvdata(dev);

	if (rkvenc && rkvenc->bound)
		v4l2_m2m_resume(rkvenc->m2m_dev);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(rkvenc_master_pm_ops, rkvenc_master_suspend,
				rkvenc_master_resume);

static struct platform_driver rkvenc_master_driver = {
	.probe = rkvenc_master_probe,
	.remove = rkvenc_master_remove,
	.driver = {
		.name = RKVENC_NAME,
		.pm = pm_sleep_ptr(&rkvenc_master_pm_ops),
	},
};

static struct platform_device *rkvenc_master_pdev;

static bool rkvenc_has_available_core(void)
{
	struct device_node *np;

	for_each_matching_node(np, rkvenc_core_of_match) {
		if (of_device_is_available(np)) {
			of_node_put(np);
			return true;
		}
	}

	return false;
}

static int __init rkvenc_init(void)
{
	int ret;

	ret = platform_driver_register(&rkvenc_core_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&rkvenc_master_driver);
	if (ret)
		goto err_unregister_core;

	if (!rkvenc_has_available_core())
		return 0;

	rkvenc_master_pdev = platform_device_register_simple(RKVENC_NAME,
							     PLATFORM_DEVID_NONE,
							     NULL, 0);
	if (IS_ERR(rkvenc_master_pdev)) {
		ret = PTR_ERR(rkvenc_master_pdev);
		rkvenc_master_pdev = NULL;
		goto err_unregister_master;
	}

	return 0;

err_unregister_master:
	platform_driver_unregister(&rkvenc_master_driver);
err_unregister_core:
	platform_driver_unregister(&rkvenc_core_driver);
	return ret;
}
module_init(rkvenc_init);

static void __exit rkvenc_exit(void)
{
	platform_device_unregister(rkvenc_master_pdev);
	platform_driver_unregister(&rkvenc_master_driver);
	platform_driver_unregister(&rkvenc_core_driver);
}
module_exit(rkvenc_exit);

MODULE_DESCRIPTION("Rockchip VEPU580 stateless video encoder driver");
MODULE_LICENSE("GPL");
