// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rockchip VDPU720 JPEG decoder driver
 *
 * The VDPU720 only decodes the entropy coded segment of a baseline JPEG
 * image, so the headers are parsed here and the decoder is exposed as a
 * stateful V4L2 memory-to-memory decoder.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "rkjpegd.h"

static const u32 rkjpegd_cap_fourccs[] = {
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV16,
	V4L2_PIX_FMT_NV24,
	V4L2_PIX_FMT_GREY,
};

static bool rkjpegd_is_cap_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rkjpegd_cap_fourccs); i++)
		if (rkjpegd_cap_fourccs[i] == fourcc)
			return true;

	return false;
}

/*
 * The decoder does not change the colorimetry: keep what the client sets on
 * OUTPUT (UVC cameras for instance report limited range), default to JFIF,
 * which is sRGB with full range BT.601 YCbCr, and copy it to CAPTURE.
 */
static void rkjpegd_set_out_colorimetry(struct v4l2_pix_format_mplane *pix)
{
	pix->field = V4L2_FIELD_NONE;
	if (pix->colorspace == V4L2_COLORSPACE_DEFAULT ||
	    pix->colorspace > V4L2_COLORSPACE_DCI_P3) {
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_SRGB;
	}
}

/* The hardware writes whole 16x16 blocks: pad the frame accordingly */
static void rkjpegd_fill_cap_fmt(struct rkjpegd_ctx *ctx,
				 struct v4l2_pix_format_mplane *pix,
				 u32 fourcc, u32 width, u32 height)
{
	const struct v4l2_pix_format_mplane *out = &ctx->out_fmt;

	width = clamp(width, RKJPEGD_MIN_SIZE, RKJPEGD_MAX_SIZE);
	height = clamp(height, RKJPEGD_MIN_SIZE, RKJPEGD_MAX_SIZE);

	v4l2_fill_pixfmt_mp(pix, fourcc, ALIGN(width, RKJPEGD_ALIGN),
			    ALIGN(height, RKJPEGD_ALIGN));
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = out->colorspace;
	pix->ycbcr_enc = out->ycbcr_enc;
	pix->quantization = out->quantization;
	pix->xfer_func = out->xfer_func;
}

static void rkjpegd_fill_out_fmt(struct v4l2_pix_format_mplane *pix)
{
	u32 sizeimage = pix->plane_fmt[0].sizeimage;
	u32 min_size;

	pix->pixelformat = V4L2_PIX_FMT_JPEG;
	pix->width = clamp(pix->width, RKJPEGD_MIN_SIZE, RKJPEGD_MAX_SIZE);
	pix->height = clamp(pix->height, RKJPEGD_MIN_SIZE, RKJPEGD_MAX_SIZE);
	pix->num_planes = 1;
	pix->flags = 0;
	rkjpegd_set_out_colorimetry(pix);

	/* at least what a 4:2:0 frame of that size takes uncompressed */
	min_size = clamp_t(u32, ALIGN(pix->width, RKJPEGD_ALIGN) *
				ALIGN(pix->height, RKJPEGD_ALIGN) * 3 / 2,
			   RKJPEGD_MIN_OUT_SIZE, RKJPEGD_DEF_OUT_MAX_SIZE);
	memset(pix->plane_fmt, 0, sizeof(pix->plane_fmt));
	pix->plane_fmt[0].sizeimage =
		PAGE_ALIGN(clamp_t(u32, sizeimage, min_size,
				   RKJPEGD_MAX_OUT_SIZE));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

/*
 * Set up the CAPTURE format for a newly parsed header. Keep the client's
 * pixel format if it chose one that works for this stream.
 */
static void rkjpegd_apply_header(struct rkjpegd_ctx *ctx,
				 const struct rkjpegd_hdr *hdr)
{
	u32 fourcc = ctx->cap_fmt.pixelformat;

	if (!ctx->cap_fourcc_set ||
	    !rkjpegd_cap_fourcc_valid(fourcc, hdr->subsampling))
		fourcc = rkjpegd_native_fourcc(hdr->subsampling);

	rkjpegd_fill_cap_fmt(ctx, &ctx->cap_fmt, fourcc, hdr->width, hdr->height);
	ctx->visible_width = hdr->width;
	ctx->visible_height = hdr->height;
	ctx->subsampling = hdr->subsampling;
	ctx->header_known = true;
}

static bool rkjpegd_header_changed(struct rkjpegd_ctx *ctx,
				   const struct rkjpegd_hdr *hdr)
{
	return hdr->width != ctx->visible_width ||
	       hdr->height != ctx->visible_height ||
	       hdr->subsampling != ctx->subsampling;
}

static void rkjpegd_queue_src_change(struct rkjpegd_ctx *ctx)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static void rkjpegd_queue_eos(struct rkjpegd_ctx *ctx)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS,
	};

	v4l2_event_queue_fh(&ctx->fh, &ev);
}

/* ioctls */

static int rkjpegd_querycap(struct file *file, void *priv,
			    struct v4l2_capability *cap)
{
	strscpy(cap->driver, RKJPEGD_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Rockchip VDPU720 JPEG decoder", sizeof(cap->card));

	return 0;
}

static int rkjpegd_enum_fmt_vid_out(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_JPEG;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;

	return 0;
}

static int rkjpegd_enum_fmt_vid_cap(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);
	unsigned int i, n = 0;
	u32 native;

	if (!ctx->header_known) {
		if (f->index >= ARRAY_SIZE(rkjpegd_cap_fourccs))
			return -EINVAL;
		f->pixelformat = rkjpegd_cap_fourccs[f->index];
		return 0;
	}

	/* the native layout first, then the conversions */
	native = rkjpegd_native_fourcc(ctx->subsampling);
	if (f->index == 0) {
		f->pixelformat = native;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(rkjpegd_cap_fourccs); i++) {
		u32 fourcc = rkjpegd_cap_fourccs[i];

		if (fourcc == native ||
		    !rkjpegd_cap_fourcc_valid(fourcc, ctx->subsampling))
			continue;
		if (++n == f->index) {
			f->pixelformat = fourcc;
			return 0;
		}
	}

	return -EINVAL;
}

static int rkjpegd_enum_framesizes(struct file *file, void *priv,
				   struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_JPEG &&
	    !rkjpegd_is_cap_fourcc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = RKJPEGD_MIN_SIZE;
	fsize->stepwise.max_width = RKJPEGD_MAX_SIZE;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = RKJPEGD_MIN_SIZE;
	fsize->stepwise.max_height = RKJPEGD_MAX_SIZE;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int rkjpegd_g_fmt_vid_out(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);

	f->fmt.pix_mp = ctx->out_fmt;

	return 0;
}

static int rkjpegd_g_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);

	f->fmt.pix_mp = ctx->cap_fmt;

	return 0;
}

static int rkjpegd_try_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	rkjpegd_fill_out_fmt(&f->fmt.pix_mp);

	return 0;
}

static int rkjpegd_try_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	u32 fourcc = pix->pixelformat;

	if (ctx->header_known) {
		/* the size comes from the stream */
		if (!rkjpegd_cap_fourcc_valid(fourcc, ctx->subsampling))
			fourcc = rkjpegd_native_fourcc(ctx->subsampling);
		rkjpegd_fill_cap_fmt(ctx, pix, fourcc, ctx->visible_width,
				     ctx->visible_height);
	} else {
		if (!rkjpegd_is_cap_fourcc(fourcc))
			fourcc = ctx->cap_fmt.pixelformat;
		rkjpegd_fill_cap_fmt(ctx, pix, fourcc, pix->width, pix->height);
	}

	return 0;
}

static int rkjpegd_s_fmt_vid_out(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	rkjpegd_fill_out_fmt(pix);
	ctx->out_fmt = *pix;

	/*
	 * A new OUTPUT format starts a new stream: forget the old header and
	 * propagate the size to the CAPTURE queue unless it is in use.
	 */
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!vb2_is_busy(vq)) {
		ctx->header_known = false;
		ctx->state = RKJPEGD_STATE_INIT;
		ctx->visible_width = pix->width;
		ctx->visible_height = pix->height;
		rkjpegd_fill_cap_fmt(ctx, &ctx->cap_fmt, ctx->cap_fmt.pixelformat,
				     pix->width, pix->height);
	}

	return 0;
}

static int rkjpegd_s_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = rkjpegd_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	ctx->cap_fmt = f->fmt.pix_mp;
	ctx->cap_fourcc_set = true;
	if (!ctx->header_known) {
		ctx->visible_width = ctx->cap_fmt.width;
		ctx->visible_height = ctx->cap_fmt.height;
	}

	return 0;
}

static int rkjpegd_g_selection(struct file *file, void *priv,
			       struct v4l2_selection *s)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	s->r.left = 0;
	s->r.top = 0;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.width = ctx->cap_fmt.width;
		s->r.height = ctx->cap_fmt.height;
		break;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.width = ctx->visible_width;
		s->r.height = ctx->visible_height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rkjpegd_subscribe_event(struct v4l2_fh *fh,
				   const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static int rkjpegd_decoder_cmd(struct file *file, void *priv,
			       struct v4l2_decoder_cmd *dc)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, dc);
	if (ret)
		return ret;

	ret = v4l2_m2m_decoder_cmd(file, m2m_ctx, dc);
	if (ret)
		return ret;

	if (dc->cmd == V4L2_DEC_CMD_START) {
		/*
		 * The client keeps its CAPTURE buffers after a resolution
		 * change; buffers that turn out to be too small are returned
		 * with an error.
		 */
		if (ctx->state == RKJPEGD_STATE_SOURCE_CHANGE)
			ctx->state = RKJPEGD_STATE_RUNNING;
		vb2_clear_last_buffer_dequeued(&m2m_ctx->cap_q_ctx.q);
		v4l2_m2m_try_schedule(m2m_ctx);
	}

	return 0;
}

static const struct v4l2_ioctl_ops rkjpegd_ioctl_ops = {
	.vidioc_querycap = rkjpegd_querycap,
	.vidioc_enum_framesizes = rkjpegd_enum_framesizes,

	.vidioc_enum_fmt_vid_out = rkjpegd_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out_mplane = rkjpegd_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out_mplane = rkjpegd_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out_mplane = rkjpegd_s_fmt_vid_out,

	.vidioc_enum_fmt_vid_cap = rkjpegd_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane = rkjpegd_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane = rkjpegd_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane = rkjpegd_s_fmt_vid_cap,

	.vidioc_g_selection = rkjpegd_g_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = rkjpegd_decoder_cmd,

	.vidioc_subscribe_event = rkjpegd_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* vb2 queue operations */

static int rkjpegd_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct rkjpegd_ctx *ctx = vb2_get_drv_priv(vq);
	const struct v4l2_pix_format_mplane *pix;

	pix = V4L2_TYPE_IS_OUTPUT(vq->type) ? &ctx->out_fmt : &ctx->cap_fmt;

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < pix->plane_fmt[0].sizeimage ?
		       -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = pix->plane_fmt[0].sizeimage;

	return 0;
}

static int rkjpegd_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct rkjpegd_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rkjpegd_buf *buf;
	void *vaddr;

	if (V4L2_TYPE_IS_CAPTURE(vq->type)) {
		if (vb2_plane_size(vb, 0) < ctx->cap_fmt.plane_fmt[0].sizeimage)
			return -EINVAL;
		return 0;
	}

	vbuf->field = V4L2_FIELD_NONE;

	/*
	 * Streams the hardware cannot decode are not refused here: the
	 * buffer is returned with an error once it has been queued.
	 */
	buf = to_rkjpegd_buf(vbuf);
	vaddr = vb2_plane_vaddr(vb, 0);
	buf->valid = vaddr &&
		     !rkjpegd_parse_header(vaddr, vb2_get_plane_payload(vb, 0),
					   &buf->hdr);
	if (!buf->valid)
		dev_dbg(ctx->jpegd->dev, "unsupported or invalid JPEG header\n");

	return 0;
}

static bool rkjpegd_cap_compatible(struct rkjpegd_ctx *ctx,
				   const struct v4l2_pix_format_mplane *old)
{
	return old->pixelformat == ctx->cap_fmt.pixelformat &&
	       old->width == ctx->cap_fmt.width &&
	       old->height == ctx->cap_fmt.height;
}

static void rkjpegd_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct rkjpegd_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vq->type)) {
		if (vb2_is_streaming(vq) && v4l2_m2m_dst_buf_is_last(m2m_ctx)) {
			vb2_set_plane_payload(vb, 0, 0);
			vbuf->field = V4L2_FIELD_NONE;
			vbuf->sequence = ctx->cap_sequence++;
			v4l2_m2m_last_buffer_done(m2m_ctx, vbuf);
			return;
		}
		v4l2_m2m_buf_queue(m2m_ctx, vbuf);
		return;
	}

	if (!to_rkjpegd_buf(vbuf)->valid) {
		vbuf->sequence = ctx->out_sequence++;
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	/* the first header sets up the CAPTURE format */
	if (ctx->state == RKJPEGD_STATE_INIT) {
		struct v4l2_pix_format_mplane old = ctx->cap_fmt;
		struct vb2_queue *cap_q = v4l2_m2m_get_dst_vq(m2m_ctx);

		rkjpegd_apply_header(ctx, &to_rkjpegd_buf(vbuf)->hdr);
		rkjpegd_queue_src_change(ctx);
		if (vb2_is_streaming(cap_q) && !rkjpegd_cap_compatible(ctx, &old))
			ctx->state = RKJPEGD_STATE_SOURCE_CHANGE;
		else
			ctx->state = RKJPEGD_STATE_RUNNING;
	}

	v4l2_m2m_buf_queue(m2m_ctx, vbuf);
}

static int rkjpegd_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct rkjpegd_ctx *ctx = vb2_get_drv_priv(vq);

	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, vq);

	return 0;
}

static void rkjpegd_stop_streaming(struct vb2_queue *vq)
{
	struct rkjpegd_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}

	v4l2_m2m_update_stop_streaming_state(m2m_ctx, vq);

	/*
	 * Reset the sequence counters here rather than in start_streaming():
	 * buffers queued before STREAMON may already have been numbered.
	 */
	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->out_sequence = 0;
	else
		ctx->cap_sequence = 0;

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		if (v4l2_m2m_has_stopped(m2m_ctx))
			rkjpegd_queue_eos(ctx);
	} else if (ctx->state == RKJPEGD_STATE_SOURCE_CHANGE) {
		/* the client is reconfiguring the CAPTURE queue */
		ctx->state = RKJPEGD_STATE_RUNNING;
	}
}

static const struct vb2_ops rkjpegd_qops = {
	.queue_setup = rkjpegd_queue_setup,
	.buf_prepare = rkjpegd_buf_prepare,
	.buf_queue = rkjpegd_buf_queue,
	.start_streaming = rkjpegd_start_streaming,
	.stop_streaming = rkjpegd_stop_streaming,
};

static int rkjpegd_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct rkjpegd_ctx *ctx = priv;
	struct rkjpegd_dev *jpegd = ctx->jpegd;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &rkjpegd_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct rkjpegd_buf);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &jpegd->lock;
	src_vq->dev = jpegd->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &rkjpegd_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &jpegd->lock;
	dst_vq->dev = jpegd->dev;

	return vb2_queue_init(dst_vq);
}

/* m2m operations */

static void rkjpegd_job_done(struct rkjpegd_dev *jpegd,
			     enum vb2_buffer_state state)
{
	struct rkjpegd_ctx *ctx = jpegd->curr_ctx;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;

	jpegd->curr_ctx = NULL;

	src = v4l2_m2m_src_buf_remove(m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(m2m_ctx);

	v4l2_m2m_buf_copy_metadata(src, dst);
	src->sequence = ctx->out_sequence++;
	dst->sequence = ctx->cap_sequence++;
	dst->field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&dst->vb2_buf, 0,
			      ctx->cap_fmt.plane_fmt[0].sizeimage);

	if (v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src)) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		rkjpegd_queue_eos(ctx);
		v4l2_m2m_mark_stopped(m2m_ctx);
	}

	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);

	pm_runtime_put_autosuspend(jpegd->dev);
	v4l2_m2m_job_finish(jpegd->m2m_dev, m2m_ctx);
}

static void rkjpegd_device_run(void *priv)
{
	struct rkjpegd_ctx *ctx = priv;
	struct rkjpegd_dev *jpegd = ctx->jpegd;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;
	const struct rkjpegd_hdr *hdr;
	int ret;

	src = v4l2_m2m_next_src_buf(m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	hdr = &to_rkjpegd_buf(src)->hdr;

	/*
	 * A new size or subsampling: return an empty CAPTURE buffer flagged
	 * LAST and wait for the client to pick up the new format. This
	 * buffer stays queued and is decoded after the reconfiguration.
	 */
	if (rkjpegd_header_changed(ctx, hdr)) {
		rkjpegd_apply_header(ctx, hdr);
		ctx->state = RKJPEGD_STATE_SOURCE_CHANGE;

		dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
		vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		dst->field = V4L2_FIELD_NONE;
		dst->sequence = ctx->cap_sequence++;
		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);

		rkjpegd_queue_src_change(ctx);
		v4l2_m2m_job_finish(jpegd->m2m_dev, m2m_ctx);
		return;
	}

	jpegd->curr_ctx = ctx;

	ret = pm_runtime_resume_and_get(jpegd->dev);
	if (ret < 0) {
		dev_err(jpegd->dev, "cannot resume device: %d\n", ret);
		pm_runtime_get_noresume(jpegd->dev);
		rkjpegd_job_done(jpegd, VB2_BUF_STATE_ERROR);
		return;
	}

	if (vb2_plane_size(&dst->vb2_buf, 0) <
	    ctx->cap_fmt.plane_fmt[0].sizeimage) {
		rkjpegd_job_done(jpegd, VB2_BUF_STATE_ERROR);
		return;
	}

	schedule_delayed_work(&jpegd->watchdog_work,
			      msecs_to_jiffies(RKJPEGD_WATCHDOG_MS));

	rkjpegd_hw_run(jpegd, ctx, hdr,
		       vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0),
		       vb2_get_plane_payload(&src->vb2_buf, 0),
		       vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0));
}

static int rkjpegd_job_ready(void *priv)
{
	struct rkjpegd_ctx *ctx = priv;

	return ctx->state == RKJPEGD_STATE_RUNNING;
}

static const struct v4l2_m2m_ops rkjpegd_m2m_ops = {
	.device_run = rkjpegd_device_run,
	.job_ready = rkjpegd_job_ready,
};

/* interrupt and watchdog */

static irqreturn_t rkjpegd_irq(int irq, void *priv)
{
	struct rkjpegd_dev *jpegd = priv;
	u32 status;

	status = rkjpegd_read(jpegd, RKJPEGD_REG_INT);
	if (!(status & RKJPEGD_INT_IRQ_RAW))
		return IRQ_NONE;

	/* acknowledge the interrupt and stop the core */
	rkjpegd_write(jpegd, RKJPEGD_REG_INT, 0);
	jpegd->irq_status = status;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t rkjpegd_irq_thread(int irq, void *priv)
{
	struct rkjpegd_dev *jpegd = priv;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	u32 status = jpegd->irq_status;

	/* the watchdog already handled this job */
	if (!cancel_delayed_work(&jpegd->watchdog_work))
		return IRQ_HANDLED;

	if (!(status & RKJPEGD_INT_IRQ) || !(status & RKJPEGD_INT_RDY_STA) ||
	    (status & RKJPEGD_INT_ERR_MASK)) {
		dev_dbg(jpegd->dev,
			"decode error: status %#x, error %#x, mcu %#x\n",
			status, rkjpegd_read(jpegd, RKJPEGD_REG_DBG_ERR),
			rkjpegd_read(jpegd, RKJPEGD_REG_DBG_MCU_POS));
		state = VB2_BUF_STATE_ERROR;
		rkjpegd_hw_reset(jpegd);
	} else if (jpegd->hw_id < RKJPEGD_ID_VPU720 &&
		   !(status & RKJPEGD_INT_SOFT_RST_RDY)) {
		/* older cores need a reset when this bit is clear */
		rkjpegd_hw_reset(jpegd);
	}

	if (jpegd->curr_ctx)
		rkjpegd_job_done(jpegd, state);

	return IRQ_HANDLED;
}

static void rkjpegd_watchdog(struct work_struct *work)
{
	struct rkjpegd_dev *jpegd = container_of(to_delayed_work(work),
						 struct rkjpegd_dev,
						 watchdog_work);

	dev_err(jpegd->dev, "frame processing timed out (status %#x)\n",
		rkjpegd_read(jpegd, RKJPEGD_REG_INT));

	disable_irq(jpegd->irq);
	rkjpegd_hw_reset(jpegd);
	enable_irq(jpegd->irq);

	if (jpegd->curr_ctx)
		rkjpegd_job_done(jpegd, VB2_BUF_STATE_ERROR);
}

/* file operations */

static int rkjpegd_open(struct file *file)
{
	struct rkjpegd_dev *jpegd = video_drvdata(file);
	struct rkjpegd_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->jpegd = jpegd;
	v4l2_fh_init(&ctx->fh, video_devdata(file));

	ctx->out_fmt.width = RKJPEGD_DEF_WIDTH;
	ctx->out_fmt.height = RKJPEGD_DEF_HEIGHT;
	rkjpegd_fill_out_fmt(&ctx->out_fmt);
	rkjpegd_fill_cap_fmt(ctx, &ctx->cap_fmt, V4L2_PIX_FMT_NV12,
			     RKJPEGD_DEF_WIDTH, RKJPEGD_DEF_HEIGHT);
	ctx->visible_width = RKJPEGD_DEF_WIDTH;
	ctx->visible_height = RKJPEGD_DEF_HEIGHT;
	ctx->state = RKJPEGD_STATE_INIT;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpegd->m2m_dev, ctx,
					    rkjpegd_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free;
	}

	v4l2_fh_add(&ctx->fh, file);

	return 0;

err_free:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int rkjpegd_release(struct file *file)
{
	struct rkjpegd_ctx *ctx = file_to_rkjpegd_ctx(file);
	struct rkjpegd_dev *jpegd = ctx->jpegd;

	mutex_lock(&jpegd->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&jpegd->lock);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations rkjpegd_fops = {
	.owner = THIS_MODULE,
	.open = rkjpegd_open,
	.release = rkjpegd_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* probe and power management */

static int rkjpegd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rkjpegd_dev *jpegd;
	struct video_device *vfd;
	int ret;

	jpegd = devm_kzalloc(dev, sizeof(*jpegd), GFP_KERNEL);
	if (!jpegd)
		return -ENOMEM;

	jpegd->dev = dev;
	platform_set_drvdata(pdev, jpegd);
	mutex_init(&jpegd->lock);
	INIT_DELAYED_WORK(&jpegd->watchdog_work, rkjpegd_watchdog);

	jpegd->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(jpegd->regs))
		return PTR_ERR(jpegd->regs);

	jpegd->num_clocks = devm_clk_bulk_get_all(dev, &jpegd->clocks);
	if (jpegd->num_clocks < 0)
		return dev_err_probe(dev, jpegd->num_clocks,
				     "cannot get clocks\n");

	jpegd->rstc = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(jpegd->rstc))
		return dev_err_probe(dev, PTR_ERR(jpegd->rstc),
				     "cannot get resets\n");

	jpegd->irq = platform_get_irq(pdev, 0);
	if (jpegd->irq < 0)
		return jpegd->irq;

	ret = devm_request_threaded_irq(dev, jpegd->irq, rkjpegd_irq,
					rkjpegd_irq_thread, 0,
					dev_name(dev), jpegd);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "cannot set DMA mask\n");

	jpegd->tables = dmam_alloc_coherent(dev, RKJPEGD_TABLES_SIZE,
					    &jpegd->tables_dma, GFP_KERNEL);
	if (!jpegd->tables)
		return -ENOMEM;

	if (iommu_get_domain_for_dev(dev)) {
		jpegd->empty_domain = iommu_paging_domain_alloc(dev);
		if (IS_ERR(jpegd->empty_domain)) {
			dev_warn(dev, "cannot allocate empty IOMMU domain\n");
			jpegd->empty_domain = NULL;
		}
	}

	pm_runtime_set_autosuspend_delay(dev, RKJPEGD_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto err_pm;
	jpegd->hw_id = rkjpegd_read(jpegd, RKJPEGD_REG_ID);
	rkjpegd_write(jpegd, RKJPEGD_REG_INT, 0);
	pm_runtime_put_autosuspend(dev);

	ret = v4l2_device_register(dev, &jpegd->v4l2_dev);
	if (ret)
		goto err_pm;

	jpegd->m2m_dev = v4l2_m2m_init(&rkjpegd_m2m_ops);
	if (IS_ERR(jpegd->m2m_dev)) {
		ret = PTR_ERR(jpegd->m2m_dev);
		goto err_v4l2;
	}

	vfd = &jpegd->vdev;
	strscpy(vfd->name, RKJPEGD_NAME, sizeof(vfd->name));
	vfd->fops = &rkjpegd_fops;
	vfd->ioctl_ops = &rkjpegd_ioctl_ops;
	vfd->release = video_device_release_empty;
	vfd->lock = &jpegd->lock;
	vfd->v4l2_dev = &jpegd->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(vfd, jpegd);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	dev_info(dev, "hardware id %#x, registered as %s\n", jpegd->hw_id,
		 video_device_node_name(vfd));

	return 0;

err_m2m:
	v4l2_m2m_release(jpegd->m2m_dev);
err_v4l2:
	/* runtime PM callbacks need the driver data, see rkjpegd_remove() */
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	v4l2_device_unregister(&jpegd->v4l2_dev);
	goto err_free_domain;
err_pm:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
err_free_domain:
	if (jpegd->empty_domain)
		iommu_domain_free(jpegd->empty_domain);
	return ret;
}

static void rkjpegd_remove(struct platform_device *pdev)
{
	struct rkjpegd_dev *jpegd = platform_get_drvdata(pdev);

	video_unregister_device(&jpegd->vdev);
	cancel_delayed_work_sync(&jpegd->watchdog_work);
	v4l2_m2m_release(jpegd->m2m_dev);

	/*
	 * Suspend and disable runtime PM while the driver data is valid:
	 * the runtime PM callbacks use it, and v4l2_device_unregister()
	 * may clear it.
	 */
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	v4l2_device_unregister(&jpegd->v4l2_dev);
	if (jpegd->empty_domain)
		iommu_domain_free(jpegd->empty_domain);
}

static int rkjpegd_runtime_suspend(struct device *dev)
{
	struct rkjpegd_dev *jpegd = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(jpegd->num_clocks, jpegd->clocks);

	return 0;
}

static int rkjpegd_runtime_resume(struct device *dev)
{
	struct rkjpegd_dev *jpegd = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(jpegd->num_clocks, jpegd->clocks);
}

static int rkjpegd_suspend(struct device *dev)
{
	struct rkjpegd_dev *jpegd = dev_get_drvdata(dev);

	v4l2_m2m_suspend(jpegd->m2m_dev);

	return pm_runtime_force_suspend(dev);
}

static int rkjpegd_resume(struct device *dev)
{
	struct rkjpegd_dev *jpegd = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	v4l2_m2m_resume(jpegd->m2m_dev);

	return 0;
}

static const struct dev_pm_ops rkjpegd_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rkjpegd_suspend, rkjpegd_resume)
	RUNTIME_PM_OPS(rkjpegd_runtime_suspend, rkjpegd_runtime_resume, NULL)
};

static const struct of_device_id rkjpegd_of_match[] = {
	{ .compatible = "rockchip,rk3588-jpeg-decoder" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rkjpegd_of_match);

static struct platform_driver rkjpegd_driver = {
	.probe = rkjpegd_probe,
	.remove = rkjpegd_remove,
	.driver = {
		.name = RKJPEGD_NAME,
		.of_match_table = rkjpegd_of_match,
		.pm = pm_ptr(&rkjpegd_pm_ops),
	},
};
module_platform_driver(rkjpegd_driver);

MODULE_DESCRIPTION("Rockchip VDPU720 JPEG decoder driver");
MODULE_LICENSE("GPL");
