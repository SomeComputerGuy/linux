// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip Video Decoder driver
 *
 * Copyright (C) 2019 Collabora, Ltd.
 *
 * Based on rkvdec driver by Google LLC. (Tomasz Figa <tfiga@chromium.org>)
 * Based on s5p-mfc driver by Samsung Electronics Co., Ltd.
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 */

#include <linux/hw_bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>

#include "rkvdec.h"
#include "rkvdec-ccu.h"
#include "rkvdec-regs.h"
#include "rkvdec-vdpu381-regs.h"
#include "rkvdec-vdpu383-regs.h"
#include "rkvdec-rcb.h"

static bool rkvdec_image_fmt_match(enum rkvdec_image_fmt fmt1,
				   enum rkvdec_image_fmt fmt2)
{
	return fmt1 == fmt2 || fmt2 == RKVDEC_IMG_FMT_ANY ||
	       fmt1 == RKVDEC_IMG_FMT_ANY;
}

static bool rkvdec_image_fmt_changed(struct rkvdec_ctx *ctx,
				     enum rkvdec_image_fmt image_fmt)
{
	if (image_fmt == RKVDEC_IMG_FMT_ANY)
		return false;

	return ctx->image_fmt != image_fmt;
}

static u32 rkvdec_enum_decoded_fmt(struct rkvdec_ctx *ctx, int index,
				   enum rkvdec_image_fmt image_fmt)
{
	const struct rkvdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	int fmt_idx = -1;
	unsigned int i;

	if (WARN_ON(!desc))
		return 0;

	for (i = 0; i < desc->num_decoded_fmts; i++) {
		if (!rkvdec_image_fmt_match(desc->decoded_fmts[i].image_fmt,
					    image_fmt))
			continue;
		fmt_idx++;
		if (index == fmt_idx)
			return desc->decoded_fmts[i].fourcc;
	}

	return 0;
}

static bool rkvdec_is_valid_fmt(struct rkvdec_ctx *ctx, u32 fourcc,
				enum rkvdec_image_fmt image_fmt)
{
	const struct rkvdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	unsigned int i;

	for (i = 0; i < desc->num_decoded_fmts; i++) {
		if (rkvdec_image_fmt_match(desc->decoded_fmts[i].image_fmt,
					   image_fmt) &&
		    desc->decoded_fmts[i].fourcc == fourcc)
			return true;
	}

	return false;
}

static u32 rkvdec_colmv_size(u16 width, u16 height)
{
	return 128 * DIV_ROUND_UP(width, 16) * DIV_ROUND_UP(height, 16);
}

/*
 * The VDPU381 stores co-located motion vectors compressed, in segments of
 * 64x16 pixels, with a segment header (rows padded to 16 headers)
 * and up to 1 KiB of payload. That's the bound the vendor code allocates;
 * how much is used depends on the content, and can be more than the
 * uncompressed layout of rkvdec_colmv_size() takes.
 */
static u32 vdpu381_colmv_size(u16 width, u16 height)
{
	u32 seg_w = DIV_ROUND_UP(width, 64);
	u32 seg_h = DIV_ROUND_UP(height, 16);

	return ALIGN(ALIGN(seg_w, 16) * seg_h + seg_w * seg_h * SZ_1K, 128);
}

static u32 vdpu383_colmv_size(u16 width, u16 height)
{
	return ALIGN(width, 64) * ALIGN(height, 16);
}

static void rkvdec_fill_decoded_pixfmt(struct rkvdec_ctx *ctx,
				       struct v4l2_pix_format_mplane *pix_mp)
{
	const struct rkvdec_variant *variant = ctx->dev->variant;
	struct v4l2_plane_pix_format *plane = &pix_mp->plane_fmt[0];
	u32 aligned_bpl;

	v4l2_fill_pixfmt_mp(pix_mp, pix_mp->pixelformat, pix_mp->width, pix_mp->height);

	/* ensure bpl is 64 byte aligned and scale sizeimage accordingly. */
	aligned_bpl = ALIGN(plane->bytesperline, 64);
	if (aligned_bpl != plane->bytesperline) {
		plane->sizeimage = plane->sizeimage / plane->bytesperline * aligned_bpl;
		plane->bytesperline = aligned_bpl;
	}

	ctx->colmv_offset = plane->sizeimage;
	plane->sizeimage += variant->ops->colmv_size(pix_mp->width, pix_mp->height);
}

static void rkvdec_reset_fmt(struct rkvdec_ctx *ctx, struct v4l2_format *f,
			     u32 fourcc)
{
	memset(f, 0, sizeof(*f));
	f->fmt.pix_mp.pixelformat = fourcc;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void rkvdec_reset_decoded_fmt(struct rkvdec_ctx *ctx)
{
	struct v4l2_format *f = &ctx->decoded_fmt;
	u32 fourcc;

	fourcc = rkvdec_enum_decoded_fmt(ctx, 0, ctx->image_fmt);
	rkvdec_reset_fmt(ctx, f, fourcc);
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->fmt.pix_mp.width = ctx->coded_fmt.fmt.pix_mp.width;
	f->fmt.pix_mp.height = ctx->coded_fmt.fmt.pix_mp.height;
	rkvdec_fill_decoded_pixfmt(ctx, &f->fmt.pix_mp);
}

static int rkvdec_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rkvdec_ctx *ctx = container_of(ctrl->handler, struct rkvdec_ctx, ctrl_hdl);
	const struct rkvdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;

	if (desc->ops->try_ctrl)
		return desc->ops->try_ctrl(ctx, ctrl);

	return 0;
}

static int rkvdec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rkvdec_ctx *ctx = container_of(ctrl->handler, struct rkvdec_ctx, ctrl_hdl);
	const struct rkvdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	enum rkvdec_image_fmt image_fmt;
	struct vb2_queue *vq;

	if (ctrl->id == V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS) {
		ctx->has_sps_st_rps |= !!(ctrl->has_changed);
		return 0;
	}

	if (ctrl->id == V4L2_CID_STATELESS_HEVC_EXT_SPS_LT_RPS) {
		ctx->has_sps_lt_rps |= !!(ctrl->has_changed);
		return 0;
	}

	/* Check if this change requires a capture format reset */
	if (!desc->ops->get_image_fmt)
		return 0;

	image_fmt = desc->ops->get_image_fmt(ctx, ctrl);
	if (rkvdec_image_fmt_changed(ctx, image_fmt)) {
		vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (vb2_is_busy(vq))
			return -EBUSY;

		ctx->image_fmt = image_fmt;
		rkvdec_reset_decoded_fmt(ctx);
	}

	return 0;
}

static const struct v4l2_ctrl_ops rkvdec_ctrl_ops = {
	.try_ctrl = rkvdec_try_ctrl,
	.s_ctrl = rkvdec_s_ctrl,
};

static const struct rkvdec_ctrl_desc rkvdec_hevc_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		.cfg.flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY,
		.cfg.type = V4L2_CTRL_TYPE_HEVC_SLICE_PARAMS,
		.cfg.dims = { 600 },
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SPS,
		.cfg.ops = &rkvdec_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.cfg.max = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.cfg.def = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.cfg.min = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.cfg.max = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10,
		.cfg.def = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1,
	},
};

static const struct rkvdec_ctrls rkvdec_hevc_ctrls = {
	.ctrls = rkvdec_hevc_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(rkvdec_hevc_ctrl_descs),
};

static const struct rkvdec_ctrl_desc vdpu38x_hevc_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SPS,
		.cfg.ops = &rkvdec_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.cfg.max = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.cfg.def = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.cfg.min = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.cfg.max = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10,
		.cfg.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE),
		.cfg.def = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS,
		.cfg.ops = &rkvdec_ctrl_ops,
		.cfg.dims = { 64 },
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_EXT_SPS_LT_RPS,
		.cfg.ops = &rkvdec_ctrl_ops,
		.cfg.dims = { 32 },
	},
};

static const struct rkvdec_ctrls vdpu38x_hevc_ctrls = {
	.ctrls = vdpu38x_hevc_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(vdpu38x_hevc_ctrl_descs),
};

static const struct rkvdec_decoded_fmt_desc rkvdec_hevc_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = RKVDEC_IMG_FMT_420_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV15,
		.image_fmt = RKVDEC_IMG_FMT_420_10BIT,
	},
};

static const struct rkvdec_ctrl_desc rkvdec_h264_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SPS,
		.cfg.ops = &rkvdec_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.cfg.max = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.cfg.def = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_START_CODE,
		.cfg.min = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
		.cfg.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA,
		.cfg.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED) |
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE),
		.cfg.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.cfg.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
	},
};

static const struct rkvdec_ctrls rkvdec_h264_ctrls = {
	.ctrls = rkvdec_h264_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(rkvdec_h264_ctrl_descs),
};

static const struct rkvdec_ctrl_desc vdpu38x_h264_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SPS,
		.cfg.ops = &rkvdec_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.cfg.max = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.cfg.def = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_H264_START_CODE,
		.cfg.min = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.cfg.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
		.cfg.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA,
		.cfg.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED) |
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE),
		.cfg.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.cfg.max = V4L2_MPEG_VIDEO_H264_LEVEL_6_0,
	},
};

static const struct rkvdec_ctrls vdpu38x_h264_ctrls = {
	.ctrls = vdpu38x_h264_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(vdpu38x_h264_ctrl_descs),
};

static const struct rkvdec_decoded_fmt_desc rkvdec_h264_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = RKVDEC_IMG_FMT_420_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV15,
		.image_fmt = RKVDEC_IMG_FMT_420_10BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.image_fmt = RKVDEC_IMG_FMT_422_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV20,
		.image_fmt = RKVDEC_IMG_FMT_422_10BIT,
	},
};

static const struct rkvdec_ctrl_desc rkvdec_vp9_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_VP9_FRAME,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_VP9_COMPRESSED_HDR,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_VP9_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.cfg.max = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.cfg.def = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
	},
};

static const struct rkvdec_ctrls rkvdec_vp9_ctrls = {
	.ctrls = rkvdec_vp9_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(rkvdec_vp9_ctrl_descs),
};

static const struct rkvdec_decoded_fmt_desc rkvdec_vp9_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = RKVDEC_IMG_FMT_420_8BIT,
	},
};

static const struct rkvdec_ctrl_desc vdpu381_vp9_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_VP9_FRAME,
		.cfg.ops = &rkvdec_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_VP9_COMPRESSED_HDR,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_VP9_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.cfg.max = V4L2_MPEG_VIDEO_VP9_PROFILE_2,
		.cfg.def = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		/* profiles 1/3 are 4:2:2/4:4:4 which VDPU381 cannot decode */
		.cfg.menu_skip_mask = BIT(V4L2_MPEG_VIDEO_VP9_PROFILE_1),
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_VP9_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_VP9_LEVEL_1_0,
		.cfg.max = V4L2_MPEG_VIDEO_VP9_LEVEL_6_1,
	},
};

static const struct rkvdec_ctrls vdpu381_vp9_ctrls = {
	.ctrls = vdpu381_vp9_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(vdpu381_vp9_ctrl_descs),
};

static const struct rkvdec_decoded_fmt_desc rkvdec_vdpu381_vp9_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = RKVDEC_IMG_FMT_420_8BIT,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV15,
		.image_fmt =  RKVDEC_IMG_FMT_420_10BIT,
	},
};

static const struct rkvdec_coded_fmt_desc rkvdec_coded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 64,
			.min_height = 64,
			.max_height = 2304,
			.step_height = 16,
		},
		.ctrls = &rkvdec_hevc_ctrls,
		.ops = &rkvdec_hevc_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_hevc_decoded_fmts),
		.decoded_fmts = rkvdec_hevc_decoded_fmts,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 64,
			.min_height = 48,
			.max_height = 2560,
			.step_height = 16,
		},
		.ctrls = &rkvdec_h264_ctrls,
		.ops = &rkvdec_h264_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_h264_decoded_fmts),
		.decoded_fmts = rkvdec_h264_decoded_fmts,
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
	},
	{
		.fourcc = V4L2_PIX_FMT_VP9_FRAME,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 64,
			.min_height = 64,
			.max_height = 2304,
			.step_height = 64,
		},
		.ctrls = &rkvdec_vp9_ctrls,
		.ops = &rkvdec_vp9_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_vp9_decoded_fmts),
		.decoded_fmts = rkvdec_vp9_decoded_fmts,
	}
};

static const struct rkvdec_coded_fmt_desc rk3288_coded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 4096,
			.step_width = 64,
			.min_height = 64,
			.max_height = 2304,
			.step_height = 16,
		},
		.ctrls = &rkvdec_hevc_ctrls,
		.ops = &rkvdec_hevc_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_hevc_decoded_fmts),
		.decoded_fmts = rkvdec_hevc_decoded_fmts,
	}
};

static const struct rkvdec_coded_fmt_desc vdpu381_coded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 65472,
			.step_width = 64,
			.min_height = 64,
			.max_height = 65472,
			.step_height = 16,
		},
		.ctrls = &vdpu38x_hevc_ctrls,
		.ops = &rkvdec_vdpu381_hevc_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_hevc_decoded_fmts),
		.decoded_fmts = rkvdec_hevc_decoded_fmts,
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
		.flags = RKVDEC_FMT_FLAG_PIPELINE,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width =  65520,
			.step_width = 64,
			.min_height = 64,
			.max_height =  65520,
			.step_height = 16,
		},
		.ctrls = &vdpu38x_h264_ctrls,
		.ops = &rkvdec_vdpu381_h264_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_h264_decoded_fmts),
		.decoded_fmts = rkvdec_h264_decoded_fmts,
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
		.flags = RKVDEC_FMT_FLAG_PIPELINE,
	},
	{
		.fourcc = V4L2_PIX_FMT_VP9_FRAME,
		.frmsize = {
			.min_width = 64,
			.max_width = 65472,
			.step_width = 64,
			.min_height = 64,
			.max_height = 65472,
			.step_height = 64,
		},
		.ctrls = &vdpu381_vp9_ctrls,
		.ops = &rkvdec_vdpu381_vp9_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_vdpu381_vp9_decoded_fmts),
		.decoded_fmts = rkvdec_vdpu381_vp9_decoded_fmts,
	}
};

static const struct rkvdec_coded_fmt_desc vdpu383_coded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width = 65472,
			.step_width = 64,
			.min_height = 64,
			.max_height = 65472,
			.step_height = 16,
		},
		.ctrls = &vdpu38x_hevc_ctrls,
		.ops = &rkvdec_vdpu383_hevc_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_hevc_decoded_fmts),
		.decoded_fmts = rkvdec_hevc_decoded_fmts,
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.frmsize = {
			.min_width = 64,
			.max_width =  65520,
			.step_width = 64,
			.min_height = 64,
			.max_height =  65520,
			.step_height = 16,
		},
		.ctrls = &vdpu38x_h264_ctrls,
		.ops = &rkvdec_vdpu383_h264_fmt_ops,
		.num_decoded_fmts = ARRAY_SIZE(rkvdec_h264_decoded_fmts),
		.decoded_fmts = rkvdec_h264_decoded_fmts,
		.subsystem_flags = VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF,
	},
};

static const struct rkvdec_coded_fmt_desc *
rkvdec_enum_coded_fmt_desc(struct rkvdec_ctx *ctx, int index)
{
	const struct rkvdec_variant *variant = ctx->dev->variant;
	int fmt_idx = -1;
	unsigned int i;

	for (i = 0; i < variant->num_coded_fmts; i++) {
		fmt_idx++;
		if (index == fmt_idx)
			return &variant->coded_fmts[i];
	}

	return NULL;
}

static const struct rkvdec_coded_fmt_desc *
rkvdec_find_coded_fmt_desc(struct rkvdec_ctx *ctx, u32 fourcc)
{
	const struct rkvdec_variant *variant = ctx->dev->variant;
	unsigned int i;

	for (i = 0; i < variant->num_coded_fmts; i++) {
		if (variant->coded_fmts[i].fourcc == fourcc)
			return &variant->coded_fmts[i];
	}

	return NULL;
}

static void rkvdec_reset_coded_fmt(struct rkvdec_ctx *ctx)
{
	struct v4l2_format *f = &ctx->coded_fmt;

	ctx->coded_fmt_desc = rkvdec_enum_coded_fmt_desc(ctx, 0);
	rkvdec_reset_fmt(ctx, f, ctx->coded_fmt_desc->fourcc);

	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f->fmt.pix_mp.width = ctx->coded_fmt_desc->frmsize.min_width;
	f->fmt.pix_mp.height = ctx->coded_fmt_desc->frmsize.min_height;

	if (ctx->coded_fmt_desc->ops->adjust_fmt)
		ctx->coded_fmt_desc->ops->adjust_fmt(ctx, f);
}

static int rkvdec_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	const struct rkvdec_coded_fmt_desc *desc;

	if (fsize->index != 0)
		return -EINVAL;

	desc = rkvdec_find_coded_fmt_desc(ctx, fsize->pixel_format);
	if (!desc)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 1;
	fsize->stepwise.max_width = desc->frmsize.max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = desc->frmsize.max_height;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int rkvdec_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct video_device *vdev = video_devdata(file);

	strscpy(cap->driver, "rkvdec", sizeof(cap->driver));
	strscpy(cap->card, vdev->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:rkvdec");
	return 0;
}

static int rkvdec_try_capture_fmt(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	const struct rkvdec_coded_fmt_desc *coded_desc;

	/*
	 * The codec context should point to a coded format desc, if the format
	 * on the coded end has not been set yet, it should point to the
	 * default value.
	 */
	coded_desc = ctx->coded_fmt_desc;
	if (WARN_ON(!coded_desc))
		return -EINVAL;

	if (!rkvdec_is_valid_fmt(ctx, pix_mp->pixelformat, ctx->image_fmt))
		pix_mp->pixelformat = rkvdec_enum_decoded_fmt(ctx, 0,
							      ctx->image_fmt);

	/* Always apply the frmsize constraint of the coded end. */
	pix_mp->width = max(pix_mp->width, ctx->coded_fmt.fmt.pix_mp.width);
	pix_mp->height = max(pix_mp->height, ctx->coded_fmt.fmt.pix_mp.height);
	v4l2_apply_frmsize_constraints(&pix_mp->width,
				       &pix_mp->height,
				       &coded_desc->frmsize);

	rkvdec_fill_decoded_pixfmt(ctx, pix_mp);
	pix_mp->field = V4L2_FIELD_NONE;

	return 0;
}

static int rkvdec_try_output_fmt(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	const struct rkvdec_coded_fmt_desc *desc;

	desc = rkvdec_find_coded_fmt_desc(ctx, pix_mp->pixelformat);
	if (!desc) {
		desc = rkvdec_enum_coded_fmt_desc(ctx, 0);
		pix_mp->pixelformat = desc->fourcc;
	}

	v4l2_apply_frmsize_constraints(&pix_mp->width,
				       &pix_mp->height,
				       &desc->frmsize);

	pix_mp->field = V4L2_FIELD_NONE;
	/* All coded formats are considered single planar for now. */
	pix_mp->num_planes = 1;

	if (desc->ops->adjust_fmt) {
		int ret;

		ret = desc->ops->adjust_fmt(ctx, f);
		if (ret)
			return ret;
	}

	return 0;
}

static int rkvdec_s_capture_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	struct vb2_queue *vq;
	int ret;

	/* Change not allowed if queue is busy */
	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = rkvdec_try_capture_fmt(file, priv, f);
	if (ret)
		return ret;

	ctx->decoded_fmt = *f;
	return 0;
}

static int rkvdec_s_output_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	const struct rkvdec_coded_fmt_desc *desc;
	struct v4l2_format *cap_fmt;
	struct vb2_queue *peer_vq, *vq;
	int ret;

	/*
	 * In order to support dynamic resolution change, the decoder admits
	 * a resolution change, as long as the pixelformat remains. Can't be
	 * done if streaming.
	 */
	vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (vb2_is_streaming(vq) ||
	    (vb2_is_busy(vq) &&
	     f->fmt.pix_mp.pixelformat != ctx->coded_fmt.fmt.pix_mp.pixelformat))
		return -EBUSY;

	/*
	 * Since format change on the OUTPUT queue will reset the CAPTURE
	 * queue, we can't allow doing so when the CAPTURE queue has buffers
	 * allocated.
	 */
	peer_vq = v4l2_m2m_get_vq(m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vb2_is_busy(peer_vq))
		return -EBUSY;

	ret = rkvdec_try_output_fmt(file, priv, f);
	if (ret)
		return ret;

	desc = rkvdec_find_coded_fmt_desc(ctx, f->fmt.pix_mp.pixelformat);
	if (!desc)
		return -EINVAL;
	ctx->coded_fmt_desc = desc;
	ctx->coded_fmt = *f;

	/*
	 * Current decoded format might have become invalid with newly
	 * selected codec, so reset it to default just to be safe and
	 * keep internal driver state sane. User is mandated to set
	 * the decoded format again after we return, so we don't need
	 * anything smarter.
	 *
	 * Note that this will propagates any size changes to the decoded format.
	 */
	ctx->image_fmt = RKVDEC_IMG_FMT_ANY;

	/* VP9 bit depth is per-frame, not up-front like HEVC's SPS;
	 * default to 8-bit/NV12 until get_image_fmt() switches it.
	 */
	if (desc->fourcc == V4L2_PIX_FMT_VP9_FRAME)
		ctx->image_fmt = RKVDEC_IMG_FMT_420_8BIT;

	rkvdec_reset_decoded_fmt(ctx);

	/* Propagate colorspace information to capture. */
	cap_fmt = &ctx->decoded_fmt;
	cap_fmt->fmt.pix_mp.colorspace = f->fmt.pix_mp.colorspace;
	cap_fmt->fmt.pix_mp.xfer_func = f->fmt.pix_mp.xfer_func;
	cap_fmt->fmt.pix_mp.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	cap_fmt->fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;

	/* Enable format specific queue features */
	vq->subsystem_flags |= desc->subsystem_flags;

	return 0;
}

static int rkvdec_g_output_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);

	*f = ctx->coded_fmt;
	return 0;
}

static int rkvdec_g_capture_fmt(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);

	*f = ctx->decoded_fmt;
	return 0;
}

static int rkvdec_enum_output_fmt(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	const struct rkvdec_coded_fmt_desc *desc;

	desc = rkvdec_enum_coded_fmt_desc(ctx, f->index);
	if (!desc)
		return -EINVAL;

	f->pixelformat = desc->fourcc;
	return 0;
}

static int rkvdec_enum_capture_fmt(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(file);
	u32 fourcc;

	fourcc = rkvdec_enum_decoded_fmt(ctx, f->index, ctx->image_fmt);
	if (!fourcc)
		return -EINVAL;

	f->pixelformat = fourcc;
	return 0;
}

static const struct v4l2_ioctl_ops rkvdec_ioctl_ops = {
	.vidioc_querycap = rkvdec_querycap,
	.vidioc_enum_framesizes = rkvdec_enum_framesizes,

	.vidioc_try_fmt_vid_cap_mplane = rkvdec_try_capture_fmt,
	.vidioc_try_fmt_vid_out_mplane = rkvdec_try_output_fmt,
	.vidioc_s_fmt_vid_out_mplane = rkvdec_s_output_fmt,
	.vidioc_s_fmt_vid_cap_mplane = rkvdec_s_capture_fmt,
	.vidioc_g_fmt_vid_out_mplane = rkvdec_g_output_fmt,
	.vidioc_g_fmt_vid_cap_mplane = rkvdec_g_capture_fmt,
	.vidioc_enum_fmt_vid_out = rkvdec_enum_output_fmt,
	.vidioc_enum_fmt_vid_cap = rkvdec_enum_capture_fmt,

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

	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
};

static int rkvdec_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f;
	unsigned int i;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		f = &ctx->coded_fmt;
	else
		f = &ctx->decoded_fmt;

	if (*num_planes) {
		if (*num_planes != f->fmt.pix_mp.num_planes)
			return -EINVAL;

		for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
			if (sizes[i] < f->fmt.pix_mp.plane_fmt[i].sizeimage)
				return -EINVAL;
		}
	} else {
		*num_planes = f->fmt.pix_mp.num_planes;
		for (i = 0; i < f->fmt.pix_mp.num_planes; i++)
			sizes[i] = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	return 0;
}

static int rkvdec_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f;
	unsigned int i;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		f = &ctx->coded_fmt;
	else
		f = &ctx->decoded_fmt;

	for (i = 0; i < f->fmt.pix_mp.num_planes; ++i) {
		u32 sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;

		if (vb2_plane_size(vb, i) < sizeimage)
			return -EINVAL;
	}

	/*
	 * Buffer's bytesused must be written by driver for CAPTURE buffers.
	 * (for OUTPUT buffers, if userspace passes 0 bytesused, v4l2-core sets
	 * it to buffer length).
	 */
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		vb2_set_plane_payload(vb, 0, f->fmt.pix_mp.plane_fmt[0].sizeimage);

	return 0;
}

static void rkvdec_buf_queue(struct vb2_buffer *vb)
{
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int rkvdec_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void rkvdec_buf_request_complete(struct vb2_buffer *vb)
{
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static void rkvdec_ctx_wait_jobs(struct rkvdec_ctx *ctx)
{
	wait_event(ctx->jobs_wq, !READ_ONCE(ctx->num_jobs));
}

static int rkvdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(q);
	const struct rkvdec_coded_fmt_desc *desc;
	int ret = 0;

	if (V4L2_TYPE_IS_CAPTURE(q->type))
		return 0;

	desc = ctx->coded_fmt_desc;
	if (WARN_ON(!desc))
		return -EINVAL;

	ctx->num_job_slots = 1;
	if (ctx->dev->ccu && (desc->flags & RKVDEC_FMT_FLAG_PIPELINE))
		ctx->num_job_slots = min_t(unsigned int, ctx->dev->core_count,
					   RKVDEC_MAX_JOBS);

	if (desc->ops->start)
		ret = desc->ops->start(ctx);

	return ret;
}

static void rkvdec_queue_cleanup(struct vb2_queue *vq, u32 state)
{
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(vq);

	while (true) {
		struct vb2_v4l2_buffer *vbuf;

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

static void rkvdec_stop_streaming(struct vb2_queue *q)
{
	struct rkvdec_ctx *ctx = vb2_get_drv_priv(q);

	/* The jobs still being decoded own buffers of both queues. */
	rkvdec_ctx_wait_jobs(ctx);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		const struct rkvdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;

		if (WARN_ON(!desc))
			return;

		if (desc->ops->stop)
			desc->ops->stop(ctx);
	}

	rkvdec_queue_cleanup(q, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops rkvdec_queue_ops = {
	.queue_setup = rkvdec_queue_setup,
	.buf_prepare = rkvdec_buf_prepare,
	.buf_queue = rkvdec_buf_queue,
	.buf_out_validate = rkvdec_buf_out_validate,
	.buf_request_complete = rkvdec_buf_request_complete,
	.start_streaming = rkvdec_start_streaming,
	.stop_streaming = rkvdec_stop_streaming,
};

static int rkvdec_request_validate(struct media_request *req)
{
	unsigned int count;

	count = vb2_request_buffer_cnt(req);
	if (!count)
		return -ENOENT;
	else if (count > 1)
		return -EINVAL;

	return vb2_request_validate(req);
}

static const struct media_device_ops rkvdec_media_ops = {
	.req_validate = rkvdec_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

/*
 * Job scheduling
 *
 * A decode job starts when the m2m framework calls device_run() and gets a
 * free core. Once the hardware is started, the buffers of the job are taken
 * off the m2m queues and the m2m job is finished as soon as another core is
 * free, so that the next job, of this context or another one, can run on that
 * core while this one is still being decoded. With a single free core, the
 * m2m job is held (rkvdec_dev.held_job) until a core becomes free. The m2m
 * framework runs one m2m job at a time, so there's at most one held job and
 * device_run() always finds a free core.
 *
 * The jobs of a context complete in decoding order. Whether the next frame of
 * a context may start before the previous one is done is decided by
 * rkvdec_job_ready().
 */
static struct rkvdec_job *rkvdec_job_start(struct rkvdec_ctx *ctx)
{
	struct rkvdec_dev *rkvdec = ctx->dev;
	struct rkvdec_core *core;
	struct rkvdec_job *job;
	unsigned int slot;

	guard(spinlock_irqsave)(&rkvdec->cores_lock);

	slot = find_first_zero_bit(&ctx->busy_slots, ctx->num_job_slots);
	if (!rkvdec->available_core_count || slot >= ctx->num_job_slots)
		return NULL;

	core = rkvdec->available_cores[--rkvdec->available_core_count];
	core->curr_ctx = ctx;

	job = &ctx->job_slots[slot];
	memset(job, 0, sizeof(*job));
	job->ctx = ctx;
	job->core = core;
	job->slot = slot;
	job->state = VB2_BUF_STATE_ERROR;
	job->busy = true;
	core->job = job;

	__set_bit(slot, &ctx->busy_slots);
	list_add_tail(&job->node, &ctx->jobs);
	ctx->num_jobs++;

	return job;
}

/* Takes the buffers of @job off the m2m queues and finishes the m2m job. */
static void rkvdec_job_detach(struct rkvdec_job *job)
{
	struct rkvdec_ctx *ctx = job->ctx;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;
	bool detached;

	detached = v4l2_m2m_buf_detach_and_job_finish(m2m_ctx->m2m_dev, m2m_ctx,
						      &src, &dst);
	WARN_ON(detached && (src != job->src || dst != job->dst));

	scoped_guard(spinlock_irqsave, &ctx->dev->cores_lock) {
		job->detached = detached;
		job->busy = false;
	}
}

static void rkvdec_job_complete(struct rkvdec_job *job)
{
	struct rkvdec_ctx *ctx = job->ctx;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;

	if (ctx->coded_fmt_desc->ops->done)
		ctx->coded_fmt_desc->ops->done(ctx, job->src, job->dst, job->state);

	if (job->detached) {
		/* Return the CAPTURE buffer first, see v4l2_m2m_buf_done_and_job_finish() */
		v4l2_m2m_buf_done(job->dst, job->state);
		v4l2_m2m_buf_done(job->src, job->state);
	} else {
		v4l2_m2m_buf_done_and_job_finish(m2m_ctx->m2m_dev, m2m_ctx,
						 job->state);
	}
}

/* Completes the finished jobs at the head of the job list of @ctx. */
static void rkvdec_ctx_complete_jobs(struct rkvdec_ctx *ctx)
{
	struct rkvdec_dev *rkvdec = ctx->dev;
	bool completed = false, schedule = false;

	mutex_lock(&ctx->done_lock);
	while (true) {
		struct rkvdec_job *job;
		bool detached;

		scoped_guard(spinlock_irqsave, &rkvdec->cores_lock) {
			job = list_first_entry_or_null(&ctx->jobs,
						       struct rkvdec_job, node);
			if (job && (!job->hw_done || job->busy))
				job = NULL;
			if (job)
				list_del(&job->node);
		}
		if (!job)
			break;

		detached = job->detached;
		rkvdec_job_complete(job);

		scoped_guard(spinlock_irqsave, &rkvdec->cores_lock) {
			__clear_bit(job->slot, &ctx->busy_slots);
			ctx->num_jobs--;
		}
		/*
		 * A detached job has no m2m job anymore that would schedule the
		 * next one when it completes: let the m2m framework check
		 * rkvdec_job_ready() again.
		 */
		schedule |= detached;
		completed = true;
	}
	mutex_unlock(&ctx->done_lock);

	if (completed)
		wake_up(&ctx->jobs_wq);
	if (schedule)
		v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
}

/*
 * The cores connected to the CCU are kept powered while one of them works, as
 * the Rockchip BSP does: the CCU synchronizes the core with the other ones.
 */
static int rkvdec_ccu_power_get(struct rkvdec_dev *rkvdec)
{
	int i, ret;

	ret = rkvdec_ccu_get(rkvdec->ccu);
	if (ret < 0)
		return ret;

	for (i = 0; i < rkvdec->core_count; i++) {
		ret = pm_runtime_resume_and_get(rkvdec->cores[i]->dev);
		if (ret < 0)
			goto err_put;
	}

	return 0;

err_put:
	while (i--)
		pm_runtime_put_autosuspend(rkvdec->cores[i]->dev);
	rkvdec_ccu_put(rkvdec->ccu);
	return ret;
}

static void rkvdec_ccu_power_put(struct rkvdec_dev *rkvdec)
{
	int i;

	for (i = 0; i < rkvdec->core_count; i++) {
		pm_runtime_mark_last_busy(rkvdec->cores[i]->dev);
		pm_runtime_put_autosuspend(rkvdec->cores[i]->dev);
	}
	rkvdec_ccu_put(rkvdec->ccu);
}

/*
 * Called when the hardware is done with the job of @core, or when the job
 * couldn't be started. Releases the core and completes the job when the
 * jobs decoded before it are done.
 */
static void rkvdec_job_hw_done(struct rkvdec_core *core,
			       enum vb2_buffer_state state, bool pm_put)
{
	struct rkvdec_job *job, *held = NULL;
	struct rkvdec_dev *rkvdec;
	struct rkvdec_ctx *ctx;

	ctx = core->curr_ctx;
	if (WARN_ON(!ctx))
		return;
	rkvdec = ctx->dev;

	scoped_guard(spinlock_irqsave, &rkvdec->cores_lock) {
		job = core->job;
		core->job = NULL;
		core->curr_ctx = NULL;
		rkvdec->available_cores[rkvdec->available_core_count++] = core;

		job->core = NULL;
		job->state = state;
		job->hw_done = true;
		wake_up(&ctx->jobs_wq);

		/* A core is free now: the held m2m job can be finished. */
		held = rkvdec->held_job;
		rkvdec->held_job = NULL;
		if (held)
			held->busy = true;
	}

	if (pm_put) {
		if (rkvdec->ccu)
			rkvdec_ccu_power_put(rkvdec);
		pm_runtime_mark_last_busy(core->dev);
		pm_runtime_put_autosuspend(core->dev);
	}

	if (held) {
		rkvdec_job_detach(held);
		if (held->ctx != ctx)
			rkvdec_ctx_complete_jobs(held->ctx);
	}

	rkvdec_ctx_complete_jobs(ctx);
}

/*
 * Called at the end of device_run(), once the hardware runs the job: lets
 * the next m2m job start if a core is free for it.
 */
static void rkvdec_job_started(struct rkvdec_job *job)
{
	struct rkvdec_ctx *ctx = job->ctx;
	struct rkvdec_dev *rkvdec = ctx->dev;
	bool detach = false;

	scoped_guard(spinlock_irqsave, &rkvdec->cores_lock) {
		if (job->src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) {
			/* Keeps the CAPTURE buffer: complete it the usual way. */
			job->busy = false;
		} else if (rkvdec->available_core_count) {
			detach = true;
		} else {
			rkvdec->held_job = job;
			job->busy = false;
		}
	}

	if (detach)
		rkvdec_job_detach(job);

	/* The hardware may already be done. */
	rkvdec_ctx_complete_jobs(ctx);
}

void rkvdec_job_add_ref(struct rkvdec_ctx *ctx, const struct vb2_buffer *ref)
{
	struct rkvdec_job *job = ctx->job;
	unsigned int i;

	if (!ref)
		return;

	for (i = 0; i < job->num_refs; i++)
		if (job->refs[i] == ref)
			return;

	if (!WARN_ON(job->num_refs >= ARRAY_SIZE(job->refs)))
		job->refs[job->num_refs++] = ref;
}

void rkvdec_job_add_poc(struct rkvdec_ctx *ctx, s32 poc, bool ref)
{
	struct rkvdec_job *job = ctx->job;

	if (ref) {
		if (!WARN_ON(job->num_ref_pocs >= ARRAY_SIZE(job->ref_pocs)))
			job->ref_pocs[job->num_ref_pocs++] = poc;
	} else if (!WARN_ON(job->num_pocs >= ARRAY_SIZE(job->pocs))) {
		job->pocs[job->num_pocs++] = poc;
	}
}

/*
 * Does an earlier frame of the stream that is still being decoded have a
 * reference with a POC of the frame being set up? The cores identify
 * references by POC: that earlier frame would wait for this one, which in
 * turn waits for it.
 */
static bool rkvdec_job_poc_alias(struct rkvdec_ctx *ctx)
{
	struct rkvdec_job *job = ctx->job, *prev;
	unsigned int i, j;

	guard(spinlock_irqsave)(&ctx->dev->cores_lock);

	list_for_each_entry(prev, &ctx->jobs, node) {
		if (prev == job)
			break;
		if (prev->hw_done)
			continue;
		for (i = 0; i < prev->num_ref_pocs; i++)
			for (j = 0; j < job->num_pocs; j++)
				if (prev->ref_pocs[i] == job->pocs[j])
					return true;
	}

	return false;
}

/*
 * POCs repeat in a stream (IDR pictures, memory_management_control_operation
 * 5, CRA): before starting a frame in CCU mode, wait for the earlier frames
 * that would take it for one of their references.
 */
void rkvdec_job_wait_poc_alias(struct rkvdec_ctx *ctx)
{
	if (!rkvdec_ctx_uses_ccu(ctx) || !rkvdec_job_poc_alias(ctx))
		return;

	wait_event(ctx->jobs_wq, !rkvdec_job_poc_alias(ctx));
}

/* Is @buf a reference of a frame of @ctx that is still being decoded? */
static bool rkvdec_ctx_buf_is_ref(struct rkvdec_ctx *ctx,
				  const struct vb2_buffer *buf)
{
	struct rkvdec_job *job;
	unsigned int i;

	list_for_each_entry(job, &ctx->jobs, node)
		for (i = 0; i < job->num_refs; i++)
			if (job->refs[i] == buf)
				return true;

	return false;
}

static int rkvdec_job_ready(void *priv)
{
	struct rkvdec_ctx *ctx = priv;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_next_src_buf(m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(m2m_ctx);

	guard(spinlock_irqsave)(&ctx->dev->cores_lock);

	if (!ctx->num_jobs)
		return 1;

	/* Only formats with more than one job slot can be pipelined. */
	if (ctx->num_jobs >= ctx->num_job_slots || !rkvdec_ctx_uses_ccu(ctx))
		return 0;

	/* A CAPTURE buffer shared by two jobs (two fields): one at a time. */
	if (!src || !dst || dst->is_held ||
	    (src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF))
		return 0;

	/*
	 * Userspace may queue a CAPTURE buffer again as soon as no future frame
	 * predicts from it, possibly before the frames being decoded are done
	 * reading it.
	 */
	if (rkvdec_ctx_buf_is_ref(ctx, &dst->vb2_buf))
		return 0;

	return 1;
}

void rkvdec_run_preamble(struct rkvdec_ctx *ctx, struct rkvdec_run *run)
{
	struct media_request *src_req;

	memset(run, 0, sizeof(*run));

	run->bufs.src = ctx->job->src;
	run->bufs.dst = ctx->job->dst;

	/* Apply request(s) controls if needed. */
	src_req = run->bufs.src->vb2_buf.req_obj.req;
	if (src_req)
		v4l2_ctrl_request_setup(src_req, &ctx->ctrl_hdl);

	v4l2_m2m_buf_copy_metadata(run->bufs.src, run->bufs.dst);
}

void rkvdec_run_postamble(struct rkvdec_ctx *ctx, struct rkvdec_run *run)
{
	struct media_request *src_req = run->bufs.src->vb2_buf.req_obj.req;

	if (src_req)
		v4l2_ctrl_request_complete(src_req, &ctx->ctrl_hdl);
}

void rkvdec_quirks_disable_qos(struct rkvdec_ctx *ctx)
{
	struct rkvdec_core *core = ctx->core;
	u32 reg;

	/* Set undocumented swreg_block_gating_e field */
	reg = readl(core->regs + RKVDEC_REG_QOS_CTRL);
	reg &= GENMASK(31, 16);
	reg |= 0xEFFF;
	writel(reg, core->regs + RKVDEC_REG_QOS_CTRL);
}

void rkvdec_memcpy_toio(void __iomem *dst, void *src, size_t len)
{
#ifdef CONFIG_ARM64
	__iowrite32_copy(dst, src, len / 4);
#else
	memcpy_toio(dst, src, len);
#endif
}

void rkvdec_schedule_watchdog(struct rkvdec_core *core, u32 timeout_threshold)
{
	/* Set watchdog at 2 times the hardware timeout threshold */
	u32 watchdog_time;
	unsigned long axi_rate = clk_get_rate(core->axi_clk);

	/* The core starts its job right after this. */
	WRITE_ONCE(core->stopped, false);

	if (axi_rate)
		watchdog_time = 2 * div_u64(1000 * (u64)timeout_threshold, axi_rate);
	else
		watchdog_time = 2000;

	schedule_delayed_work(&core->watchdog_work, msecs_to_jiffies(watchdog_time));
}

/*
 * Enables the read caches of the core, and drops what they hold: the
 * reference frames of this job may have been rewritten since they were
 * cached, by another core or as the destination of an earlier job.
 */
static void rkvdec_core_reset_cache(struct rkvdec_core *core)
{
	unsigned int i;

	if (!core->cache)
		return;

	for (i = 0; i < VDPU381_NUM_CACHES; i++) {
		writel(VDPU381_CACHE_CFG_CACHEABLE | VDPU381_CACHE_CFG_RD_ALLOCATE |
		       VDPU381_CACHE_CFG_LINE_64B, core->cache + VDPU381_CACHE_CFG(i));
		writel(VDPU381_CACHE_CLR_ALL, core->cache + VDPU381_CACHE_CLR(i));
	}
}

static void rkvdec_device_run(void *priv)
{
	struct rkvdec_ctx *ctx = priv;
	const struct rkvdec_coded_fmt_desc *desc = ctx->coded_fmt_desc;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct rkvdec_core *core;
	struct rkvdec_job *job;
	int ret;

	job = rkvdec_job_start(ctx);
	if (WARN_ON(!job)) {
		v4l2_m2m_buf_done_and_job_finish(m2m_ctx->m2m_dev, m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		return;
	}

	core = job->core;
	job->src = v4l2_m2m_next_src_buf(m2m_ctx);
	job->dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	ctx->core = core;
	ctx->job = job;

	/* A core that couldn't be stopped after a failure must not run. */
	if (READ_ONCE(core->broken)) {
		rkvdec_job_hw_done(core, VB2_BUF_STATE_ERROR, false);
		goto out;
	}

	ret = pm_runtime_resume_and_get(core->dev);
	if (ret < 0) {
		rkvdec_job_hw_done(core, VB2_BUF_STATE_ERROR, false);
		goto out;
	}

	if (ctx->dev->ccu) {
		ret = rkvdec_ccu_power_get(ctx->dev);
		if (ret < 0) {
			pm_runtime_put_autosuspend(core->dev);
			rkvdec_job_hw_done(core, VB2_BUF_STATE_ERROR, false);
			goto out;
		}
	}

	if (WARN_ON(!desc))
		goto err;

	if (ctx->dev->ccu)
		rkvdec_ccu_prepare(ctx);

	rkvdec_core_reset_cache(core);

	if (!rkvdec_rcb_buf_validate_size(ctx)) {
		rkvdec_free_rcb(ctx->dev, core);

		ret = rkvdec_allocate_rcb(ctx->dev, core,
					  ctx->decoded_fmt.fmt.pix_mp.width,
					  ctx->decoded_fmt.fmt.pix_mp.height,
					  ctx->dev->variant->rcb_sizes,
					  ctx->dev->variant->num_rcb_sizes);
		if (ret)
			goto err;
	}

	ret = desc->ops->run(ctx);
	if (ret)
		goto err;

	ctx->core = NULL;
	ctx->job = NULL;
	rkvdec_job_started(job);
	return;

err:
	cancel_delayed_work(&core->watchdog_work);
	rkvdec_job_hw_done(core, VB2_BUF_STATE_ERROR, true);
out:
	ctx->core = NULL;
	ctx->job = NULL;
	rkvdec_job_started(job);
}

static const struct v4l2_m2m_ops rkvdec_m2m_ops = {
	.device_run = rkvdec_device_run,
	.job_ready = rkvdec_job_ready,
};

static int rkvdec_queue_init(void *priv,
			     struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct rkvdec_ctx *ctx = priv;
	struct rkvdec_dev *rkvdec = ctx->dev;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &rkvdec_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;

	/*
	 * Driver does mostly sequential access, so sacrifice TLB efficiency
	 * for faster allocation. Also, no CPU access on the source queue,
	 * so no kernel mapping needed.
	 */
	src_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES |
			    DMA_ATTR_NO_KERNEL_MAPPING;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &rkvdec->vdev_lock;
	src_vq->dev = rkvdec->v4l2_dev.dev;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->bidirectional = true;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->dma_attrs = DMA_ATTR_ALLOC_SINGLE_PAGES |
			    DMA_ATTR_NO_KERNEL_MAPPING;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &rkvdec_queue_ops;
	dst_vq->buf_struct_size = sizeof(struct rkvdec_decoded_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &rkvdec->vdev_lock;
	dst_vq->dev = rkvdec->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

static int rkvdec_add_ctrls(struct rkvdec_ctx *ctx,
			    const struct rkvdec_ctrls *ctrls)
{
	unsigned int i;

	for (i = 0; i < ctrls->num_ctrls; i++) {
		const struct v4l2_ctrl_config *cfg = &ctrls->ctrls[i].cfg;

		v4l2_ctrl_new_custom(&ctx->ctrl_hdl, cfg, ctx);
		if (ctx->ctrl_hdl.error)
			return ctx->ctrl_hdl.error;
	}

	return 0;
}

static int rkvdec_init_ctrls(struct rkvdec_ctx *ctx)
{
	const struct rkvdec_variant *variant = ctx->dev->variant;
	unsigned int i, nctrls = 0;
	int ret;

	for (i = 0; i < variant->num_coded_fmts; i++)
		nctrls += variant->coded_fmts[i].ctrls->num_ctrls;

	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, nctrls);

	for (i = 0; i < variant->num_coded_fmts; i++) {
		ret = rkvdec_add_ctrls(ctx, variant->coded_fmts[i].ctrls);
		if (ret)
			goto err_free_handler;
	}

	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);
	if (ret)
		goto err_free_handler;

	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	return ret;
}

static int rkvdec_open(struct file *filp)
{
	struct rkvdec_dev *rkvdec = video_drvdata(filp);
	struct rkvdec_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = rkvdec;
	INIT_LIST_HEAD(&ctx->jobs);
	init_waitqueue_head(&ctx->jobs_wq);
	mutex_init(&ctx->done_lock);
	ctx->num_job_slots = 1;

	/*
	 * Don't hand the film index of a stream that just ended to the next
	 * one: start after the last one given out.
	 */
	ret = ida_alloc_range(&rkvdec->film_ida, READ_ONCE(rkvdec->next_film_idx),
			      RKVDEC_MAX_FILM_IDX, GFP_KERNEL);
	if (ret < 0)
		ret = ida_alloc_range(&rkvdec->film_ida, 1, RKVDEC_MAX_FILM_IDX,
				      GFP_KERNEL);
	if (ret < 0)
		goto err_free_ctx;
	ctx->film_idx = ret;
	WRITE_ONCE(rkvdec->next_film_idx, ret < RKVDEC_MAX_FILM_IDX ? ret + 1 : 1);

	rkvdec_reset_coded_fmt(ctx);
	rkvdec_reset_decoded_fmt(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(filp));

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(rkvdec->m2m_dev, ctx,
					    rkvdec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_film;
	}

	ret = rkvdec_init_ctrls(ctx);
	if (ret)
		goto err_cleanup_m2m_ctx;

	v4l2_fh_add(&ctx->fh, filp);

	return 0;

err_cleanup_m2m_ctx:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

err_free_film:
	ida_free(&rkvdec->film_ida, ctx->film_idx);

err_free_ctx:
	kfree(ctx);
	return ret;
}

static int rkvdec_release(struct file *filp)
{
	struct rkvdec_ctx *ctx = file_to_rkvdec_ctx(filp);

	v4l2_fh_del(&ctx->fh, filp);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	ida_free(&ctx->dev->film_ida, ctx->film_idx);
	mutex_destroy(&ctx->done_lock);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations rkvdec_fops = {
	.owner = THIS_MODULE,
	.open = rkvdec_open,
	.release = rkvdec_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int rkvdec_v4l2_init(struct rkvdec_dev *rkvdec)
{
	int ret;
	struct device *dev = rkvdec->main_core->dev;

	ret = v4l2_device_register(dev, &rkvdec->v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register V4L2 device\n");
		return ret;
	}

	rkvdec->m2m_dev = v4l2_m2m_init(&rkvdec_m2m_ops);
	if (IS_ERR(rkvdec->m2m_dev)) {
		v4l2_err(&rkvdec->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(rkvdec->m2m_dev);
		goto err_unregister_v4l2;
	}

	rkvdec->mdev.dev = dev;
	strscpy(rkvdec->mdev.model, "rkvdec", sizeof(rkvdec->mdev.model));
	strscpy(rkvdec->mdev.bus_info, "platform:rkvdec",
		sizeof(rkvdec->mdev.bus_info));
	media_device_init(&rkvdec->mdev);
	rkvdec->mdev.ops = &rkvdec_media_ops;
	rkvdec->v4l2_dev.mdev = &rkvdec->mdev;

	rkvdec->vdev.lock = &rkvdec->vdev_lock;
	rkvdec->vdev.v4l2_dev = &rkvdec->v4l2_dev;
	rkvdec->vdev.fops = &rkvdec_fops;
	rkvdec->vdev.release = video_device_release_empty;
	rkvdec->vdev.vfl_dir = VFL_DIR_M2M;
	rkvdec->vdev.device_caps = V4L2_CAP_STREAMING |
				   V4L2_CAP_VIDEO_M2M_MPLANE;
	rkvdec->vdev.ioctl_ops = &rkvdec_ioctl_ops;
	video_set_drvdata(&rkvdec->vdev, rkvdec);
	strscpy(rkvdec->vdev.name, "rkvdec", sizeof(rkvdec->vdev.name));

	ret = video_register_device(&rkvdec->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&rkvdec->v4l2_dev, "Failed to register video device\n");
		goto err_cleanup_mc;
	}

	ret = v4l2_m2m_register_media_controller(rkvdec->m2m_dev, &rkvdec->vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		v4l2_err(&rkvdec->v4l2_dev,
			 "Failed to initialize V4L2 M2M media controller\n");
		goto err_unregister_vdev;
	}

	ret = media_device_register(&rkvdec->mdev);
	if (ret) {
		v4l2_err(&rkvdec->v4l2_dev, "Failed to register media device\n");
		goto err_unregister_mc;
	}

	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(rkvdec->m2m_dev);

err_unregister_vdev:
	video_unregister_device(&rkvdec->vdev);

err_cleanup_mc:
	media_device_cleanup(&rkvdec->mdev);
	v4l2_m2m_release(rkvdec->m2m_dev);

err_unregister_v4l2:
	v4l2_device_unregister(&rkvdec->v4l2_dev);
	return ret;
}

static void rkvdec_v4l2_cleanup(struct rkvdec_dev *rkvdec)
{
	media_device_unregister(&rkvdec->mdev);
	v4l2_m2m_unregister_media_controller(rkvdec->m2m_dev);
	video_unregister_device(&rkvdec->vdev);
	media_device_cleanup(&rkvdec->mdev);
	v4l2_m2m_release(rkvdec->m2m_dev);
	v4l2_device_unregister(&rkvdec->v4l2_dev);
}

/*
 * Reprograms the IOMMU of @core with the domain the cores share. Only called
 * once the core is stopped (see rkvdec_core_recover()): each attach goes
 * through the IOMMU's bypass state (paging disabled), so a core that still
 * had a bus transaction pending, such as one held by a page fault, would
 * have it go out to the physical address of its IOVA. Returns false if the
 * shared domain couldn't be attached again.
 */
static bool rkvdec_iommu_restore(struct rkvdec_core *core)
{
	struct iommu_domain *domain = core->rkvdec->iommu_global_domain;
	int ret;

	if (!core->empty_domain || !domain)
		return true;

	/*
	 * Attaching the domain that is already attached does nothing: go
	 * through an empty domain, so that the IOMMU is reset and programmed
	 * again with the shared domain and its mappings.
	 */
	iommu_detach_device(domain, core->dev);
	ret = iommu_attach_device(core->empty_domain, core->dev);
	if (ret)
		dev_warn(core->dev, "Cannot attach empty domain: %d\n", ret);
	else
		iommu_detach_device(core->empty_domain, core->dev);
	ret = iommu_attach_device(domain, core->dev);
	if (ret)
		dev_err(core->dev, "Cannot attach global domain: %d\n", ret);

	return !ret;
}

/*
 * Soft resets a core whose job failed or timed out and clears its interrupt
 * status. Returns true if the core is idle: reset, and done with its bus
 * transactions.
 */
static bool rk3399_reset(struct rkvdec_core *core)
{
	u32 status;
	int ret;

	writel(RKVDEC_IRQ_DIS | RKVDEC_SOFTRST_EN_P, core->regs + RKVDEC_REG_INTERRUPT);
	ret = readl_poll_timeout(core->regs + RKVDEC_REG_INTERRUPT, status,
				 status & RKVDEC_SOFTRESET_RDY, 1, 100);
	writel(RKVDEC_IRQ_DIS, core->regs + RKVDEC_REG_INTERRUPT);

	return !ret;
}

static bool vdpu381_reset(struct rkvdec_core *core)
{
	u32 status;
	int ret;

	writel(VDPU381_DEC_IRQ_DISABLE | VDPU381_SOFT_RESET_EN,
	       core->regs + VDPU381_REG_IMPORTANT_EN);
	ret = readl_poll_timeout(core->regs + VDPU381_REG_STA_INT, status,
				 status & VDPU381_STA_INT_SOFTRESET_RDY, 1, 100);
	writel(0, core->regs + VDPU381_REG_STA_INT);
	if (ret)
		return false;

	/* The soft reset doesn't wait for the bus transactions of the core. */
	return !readl_poll_timeout(core->regs + VDPU381_REG_DEBUG_INT, status,
				   status & VDPU381_DEBUG_INT_BUS_IDLE, 1, 1000);
}

/* Waits for a core that is still decoding to report the end of its job. */
static bool vdpu381_wait_stopped(struct rkvdec_core *core)
{
	u32 status;

	return !readl_poll_timeout(core->regs + VDPU381_REG_STA_INT, status,
				   status, 100, 100 * USEC_PER_MSEC);
}

static bool vdpu383_reset(struct rkvdec_core *core)
{
	writel(FIELD_PREP_WM16(VDPU383_INT_EN_IRQ | VDPU383_INT_EN_LINE_IRQ, 0),
	       core->link + VDPU383_LINK_INT_EN);
	writel(FIELD_PREP_WM16(VDPU383_STA_INT_ALL, 0), core->link + VDPU383_LINK_STA_INT);

	/* No soft reset known: only the reset lines stop the core. */
	return false;
}

/*
 * Stops a core whose job failed or timed out, before the job is completed
 * and before anything else uses the core. The core may still be decoding,
 * or stuck with a bus transaction held by a page fault of its IOMMU: it
 * would keep writing to buffers that go back to userspace, and its IOMMU
 * can't be reinitialized under it.
 *
 * As the Rockchip BSP does, soft reset the core, which lets it finish its
 * bus transactions, then pulse its reset lines. On RK3588, both the soft
 * reset and the AXI reset line reset the IOMMU of the core too, leaving it
 * in bypass (the core also soft resets itself when its hardware timeout
 * fires). A core whose watchdog fired while it was still decoding would go
 * on accessing memory untranslated: as the BSP only resets cores that
 * reported the end of their job, give it time to do so first. Reprogram the
 * IOMMU last, before the core can be started again.
 *
 * Must be called with the interrupt of the core masked. Returns false if
 * the core couldn't be stopped or its IOMMU couldn't be programmed: it is
 * then marked broken and never started again, since its bus accesses could
 * reach memory untranslated.
 */
static bool rkvdec_core_recover(struct rkvdec_core *core)
{
	struct rkvdec_ccu *ccu = core->rkvdec->ccu;
	bool idle;
	int ret;

	/* Force it idle in the CCU, so that the other cores don't wait for it. */
	if (ccu)
		rkvdec_ccu_disconnect(ccu, core);

	if (!READ_ONCE(core->stopped) && core->variant->ops->wait_stopped &&
	    !core->variant->ops->wait_stopped(core))
		dev_warn(core->dev, "the core is still busy, resetting it anyway\n");
	WRITE_ONCE(core->stopped, true);

	idle = core->variant->ops->reset(core);
	if (core->resets) {
		if (!idle)
			dev_warn(core->dev, "soft reset failed, pulsing the reset lines\n");
		ret = reset_control_assert(core->resets);
		if (!ret) {
			udelay(5);
			ret = reset_control_deassert(core->resets);
		}
		idle = !ret;
	}

	if (!idle)
		dev_err(core->dev, "could not stop the core\n");
	else if (!rkvdec_iommu_restore(core))
		idle = false;

	if (!idle) {
		dev_err(core->dev, "disabling the core\n");
		WRITE_ONCE(core->broken, true);
	} else if (ccu) {
		/* Clear its error state in the CCU and connect it again. */
		rkvdec_ccu_reconnect(ccu, core);
	}

	return idle;
}

/*
 * Completes the job of @core with @state, unless the watchdog already did or
 * the core has no job: the interrupt status is acknowledged in any case, a
 * late interrupt (after a timeout) or a spurious one doesn't belong to a job.
 * A failed job leaves the core in an unknown state: it is stopped before the
 * job is completed. The interrupt of the core is masked while its threaded
 * handler runs.
 */
static void rkvdec_irq_job_done(struct rkvdec_core *core,
				enum vb2_buffer_state state)
{
	struct rkvdec_ctx *ctx;

	/* Whatever the job and its state, the core is done with it. */
	WRITE_ONCE(core->stopped, true);

	/*
	 * The watchdog is pending only while the core has a job, and whoever
	 * cancels it owns the job. Only then take the context: until then, the
	 * watchdog may complete the job and the next job may start.
	 */
	if (!cancel_delayed_work(&core->watchdog_work))
		return;

	ctx = READ_ONCE(core->curr_ctx);
	if (WARN_ON(!ctx))
		return;

	if (state == VB2_BUF_STATE_ERROR)
		rkvdec_core_recover(core);
	rkvdec_job_hw_done(core, state, true);
}

static irqreturn_t rk3399_irq_handler(struct rkvdec_core *core)
{
	u32 status;

	status = readl(core->regs + RKVDEC_REG_INTERRUPT);
	writel(0, core->regs + RKVDEC_REG_INTERRUPT);

	rkvdec_irq_job_done(core, (status & RKVDEC_RDY_STA) ?
			    VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);

	return IRQ_HANDLED;
}

static irqreturn_t vdpu381_irq_handler(struct rkvdec_core *core)
{
	u32 status;

	status = readl(core->regs + VDPU381_REG_STA_INT);
	writel(0, core->regs + VDPU381_REG_STA_INT);

	rkvdec_irq_job_done(core, (status & VDPU381_STA_INT_DEC_RDY_STA) ?
			    VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);

	return IRQ_HANDLED;
}

static irqreturn_t vdpu383_irq_handler(struct rkvdec_core *core)
{
	u32 status;

	status = readl(core->link + VDPU383_LINK_STA_INT);
	writel(FIELD_PREP_WM16(VDPU383_STA_INT_ALL, 0), core->link + VDPU383_LINK_STA_INT);
	/* On vdpu383, the interrupts must be disabled */
	writel(FIELD_PREP_WM16(VDPU383_INT_EN_IRQ | VDPU383_INT_EN_LINE_IRQ, 0),
	       core->link + VDPU383_LINK_INT_EN);

	rkvdec_irq_job_done(core, (status & VDPU383_STA_INT_DEC_RDY_STA) ?
			    VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);

	return IRQ_HANDLED;
}

static irqreturn_t rkvdec_irq_handler(int irq, void *priv)
{
	struct rkvdec_core *core = priv;

	return core->variant->ops->irq_handler(core);
}

/*
 * Flip one or more matrices along their main diagonal and flatten them
 * before writing it to the memory.
 * Convert:
 * ABCD         AEIM
 * EFGH     =>  BFJN     =>     AEIMBFJNCGKODHLP
 * IJKL         CGKO
 * MNOP         DHLP
 */
static void transpose_and_flatten_matrices(u8 *output, const u8 *input,
					   int matrices, int row_length)
{
	int i, j, row, x_offset, matrix_offset, rot_index, y_offset, matrix_size, new_value;

	matrix_size = row_length * row_length;
	for (i = 0; i < matrices; i++) {
		row = 0;
		x_offset = 0;
		matrix_offset = i * matrix_size;
		for (j = 0; j < matrix_size; j++) {
			y_offset = j - (row * row_length);
			rot_index = y_offset * row_length + x_offset;
			new_value = *(input + i * matrix_size + j);
			output[matrix_offset + rot_index] = new_value;
			if ((j + 1) % row_length == 0) {
				row += 1;
				x_offset += 1;
			}
		}
	}
}

/*
 * VDPU383 needs a specific order:
 * The 8x8 flatten matrix is based on 4x4 blocks.
 * Each 4x4 block is written separately in order.
 *
 * Base data    =>  Transposed    VDPU383 transposed
 *
 * ABCDEFGH         AIQYaiqy      AIQYBJRZ
 * IJKLMNOP         BJRZbjrz      CKS0DLT1
 * QRSTUVWX         CKS0cks6      aiqybjrz
 * YZ012345     =>  DLT1dlt7      cks6dlt7
 * abcdefgh         EMU2emu8      EMU2FNV3
 * ijklmnop         FNV3fnv9      GOW4HPX5
 * qrstuvwx         GOW4gow#      emu8fnv9
 * yz6789#$         HPX5hpx$      gow#hpx$
 *
 * As the function reads block of 4x4 it can be used for both 4x4 and 8x8 matrices.
 *
 */
static void vdpu383_flatten_matrices(u8 *output, const u8 *input, int matrices, int row_length)
{
	u8 block;
	int i, j, matrix_offset, matrix_size, new_value, input_idx, line_offset, block_offset;

	matrix_size = row_length * row_length;
	for (i = 0; i < matrices; i++) {
		matrix_offset = i * matrix_size;
		for (j = 0; j < matrix_size; j++) {
			block = j / 16;
			line_offset = (j % 16) / 4;
			block_offset = (block & 1) * 32 + (block & 2) * 2;
			input_idx = ((j % 4) * row_length) + line_offset + block_offset;

			new_value = *(input + i * matrix_size + input_idx);

			output[matrix_offset + j] = new_value;
		}
	}
}

static void rkvdec_watchdog_func(struct work_struct *work)
{
	struct rkvdec_core *core;
	struct rkvdec_ctx *ctx;

	core = container_of(to_delayed_work(work), struct rkvdec_core,
			      watchdog_work);
	ctx = core->curr_ctx;
	if (ctx) {
		dev_err_ratelimited(core->dev, "Frame processing timed out!\n");
		/* No interrupt handler on this core while it's stopped. */
		disable_irq(core->irq);
		rkvdec_core_recover(core);
		enable_irq(core->irq);
		rkvdec_job_hw_done(core, VB2_BUF_STATE_ERROR, true);
	}
}

static const struct rkvdec_variant_ops rk3399_variant_ops = {
	.irq_handler = rk3399_irq_handler,
	.reset = rk3399_reset,
	.colmv_size = rkvdec_colmv_size,
	.flatten_matrices = transpose_and_flatten_matrices,
};

static const struct rkvdec_variant rk3288_rkvdec_variant = {
	.num_regs = 68,
	.coded_fmts = rk3288_coded_fmts,
	.num_coded_fmts = ARRAY_SIZE(rk3288_coded_fmts),
	.ops = &rk3399_variant_ops,
	.has_single_reg_region = true,
};

static const struct rkvdec_variant rk3328_rkvdec_variant = {
	.num_regs = 109,
	.coded_fmts = rkvdec_coded_fmts,
	.num_coded_fmts = ARRAY_SIZE(rkvdec_coded_fmts),
	.ops = &rk3399_variant_ops,
	.has_single_reg_region = true,
	.quirks = RKVDEC_QUIRK_DISABLE_QOS,
};

static const struct rkvdec_variant rk3399_rkvdec_variant = {
	.num_regs = 78,
	.coded_fmts = rkvdec_coded_fmts,
	.num_coded_fmts = ARRAY_SIZE(rkvdec_coded_fmts),
	.ops = &rk3399_variant_ops,
	.has_single_reg_region = true,
};

static const struct rcb_size_info vdpu381_rcb_sizes[] = {
	{6,	PIC_WIDTH},	// intrar
	{1,	PIC_WIDTH},	// transdr (Is actually 0.4*pic_width)
	{1,	PIC_HEIGHT},	// transdc (Is actually 0.1*pic_height)
	{3,	PIC_WIDTH},	// streamdr
	{6,	PIC_WIDTH},	// interr
	{3,	PIC_HEIGHT},	// interc
	{22,	PIC_WIDTH},	// dblkr
	{6,	PIC_WIDTH},	// saor
	{11,	PIC_WIDTH},	// fbcr
	{67,	PIC_HEIGHT},	// filtc col
};

static const struct rkvdec_variant_ops vdpu381_variant_ops = {
	.irq_handler = vdpu381_irq_handler,
	.reset = vdpu381_reset,
	.wait_stopped = vdpu381_wait_stopped,
	.colmv_size = vdpu381_colmv_size,
	.flatten_matrices = transpose_and_flatten_matrices,
};

static const struct rkvdec_variant vdpu381_variant = {
	.coded_fmts = vdpu381_coded_fmts,
	.num_coded_fmts = ARRAY_SIZE(vdpu381_coded_fmts),
	.rcb_sizes = vdpu381_rcb_sizes,
	.num_rcb_sizes = ARRAY_SIZE(vdpu381_rcb_sizes),
	.ops = &vdpu381_variant_ops,
	.has_ccu = true,
	.has_cache_ctrl = true,
};

static const struct rcb_size_info vdpu383_rcb_sizes[] = {
	{6,	PIC_WIDTH},	// streamd
	{6,	PIC_WIDTH},	// streamd_tile
	{12,	PIC_WIDTH},	// inter
	{12,	PIC_WIDTH},	// inter_tile
	{16,	PIC_WIDTH},	// intra
	{10,	PIC_WIDTH},	// intra_tile
	{120,	PIC_WIDTH},	// filterd
	{120,	PIC_WIDTH},	// filterd_protect
	{120,	PIC_WIDTH},	// filterd_tile_row
	{180,	PIC_HEIGHT},	// filterd_tile_col
};

static const struct rkvdec_variant_ops vdpu383_variant_ops = {
	.irq_handler = vdpu383_irq_handler,
	.reset = vdpu383_reset,
	.colmv_size = vdpu383_colmv_size,
	.flatten_matrices = vdpu383_flatten_matrices,
};

static const struct rkvdec_variant vdpu383_variant = {
	.coded_fmts = vdpu383_coded_fmts,
	.num_coded_fmts = ARRAY_SIZE(vdpu383_coded_fmts),
	.rcb_sizes = vdpu383_rcb_sizes,
	.num_rcb_sizes = ARRAY_SIZE(vdpu383_rcb_sizes),
	.ops = &vdpu383_variant_ops,
};

static const struct of_device_id of_rkvdec_match[] = {
	{
		.compatible = "rockchip,rk3288-vdec",
		.data = &rk3288_rkvdec_variant,
	},
	{
		.compatible = "rockchip,rk3328-vdec",
		.data = &rk3328_rkvdec_variant,
	},
	{
		.compatible = "rockchip,rk3399-vdec",
		.data = &rk3399_rkvdec_variant,
	},
	{
		.compatible = "rockchip,rk3588-vdec",
		.data = &vdpu381_variant,
	},
	{
		.compatible = "rockchip,rk3576-vdec",
		.data = &vdpu383_variant,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_rkvdec_match);

static int rkvdec_core_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rkvdec_core *core = platform_get_drvdata(pdev);
	struct rkvdec_dev *rkvdec = data;
	int id, ret;

	id = rkvdec->core_count;
	core->id = id;
	core->rkvdec = rkvdec;
	rkvdec->cores[id] = core;

	if (id == 0)
		rkvdec->main_core = core;

	if (iommu_get_domain_for_dev(dev)) {
		if (!rkvdec->iommu_global_domain) {
			rkvdec->iommu_global_domain = iommu_get_domain_for_dev(dev);
			if (IS_ERR(rkvdec->iommu_global_domain)) {
				rkvdec->iommu_global_domain = NULL;
				dev_warn_once(dev, "cannot get global domain\n");
			}
		}

		if (rkvdec->iommu_global_domain) {
			ret = iommu_attach_device(rkvdec->iommu_global_domain, dev);
			if (ret)
				dev_warn(dev, "cannot attach global domain to core %d\n", id);
		}
	}

	scoped_guard(spinlock_irqsave, &rkvdec->cores_lock)
		rkvdec->available_cores[rkvdec->available_core_count++] = core;
	rkvdec->core_count++;

	dev_info(dev, "Registered core %d\n", id);

	return 0;
}

static const struct component_ops rkvdec_core_ops = {
	.bind = rkvdec_core_bind,
};

static int rkvdec_core_probe(struct platform_device *pdev)
{
	const struct rkvdec_variant *variant;
	struct rkvdec_core *core;
	int ret, irq;

	if (!pdev->dev.of_node)
		return -ENODEV;

	variant = of_device_get_match_data(&pdev->dev);
	if (!variant)
		return dev_err_probe(&pdev->dev, -ENODEV, "failed to get match data\n");

	core = devm_kzalloc(&pdev->dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = &pdev->dev;
	core->variant = variant;
	core->stopped = true;
	core->ccu_port = -1;
	INIT_DELAYED_WORK(&core->watchdog_work, rkvdec_watchdog_func);

	if (variant->has_ccu) {
		struct of_phandle_args args;

		if (!of_parse_phandle_with_fixed_args(pdev->dev.of_node,
						      "rockchip,ccu", 1, 0,
						      &args)) {
			core->ccu_port = args.args[0];
			of_node_put(args.np);
		}
	}

	ret = devm_clk_bulk_get_all_enabled(&pdev->dev, &core->clocks);
	if (ret < 0)
		return ret;
	core->num_clocks = ret;

	core->axi_clk = devm_clk_get(&pdev->dev, "axi");

	if (variant->has_single_reg_region) {
		core->regs = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(core->regs))
			return PTR_ERR(core->regs);
	} else {
		core->regs = devm_platform_ioremap_resource_byname(pdev, "function");
		if (IS_ERR(core->regs))
			return PTR_ERR(core->regs);

		core->link = devm_platform_ioremap_resource_byname(pdev, "link");
		if (IS_ERR(core->link))
			return PTR_ERR(core->link);

		if (variant->has_cache_ctrl) {
			core->cache = devm_platform_ioremap_resource_byname(pdev, "cache");
			if (IS_ERR(core->cache))
				return PTR_ERR(core->cache);
		}
	}

	if (iommu_get_domain_for_dev(&pdev->dev)) {
		core->empty_domain = iommu_paging_domain_alloc(core->dev);

		if (IS_ERR(core->empty_domain)) {
			dev_warn(core->dev, "cannot alloc new empty domain\n");
			core->empty_domain = NULL;
		}
	}

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Could not set DMA coherent mask.\n");
		return ret;
	}

	vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));

	core->resets = devm_reset_control_array_get_optional_exclusive(&pdev->dev);
	if (IS_ERR(core->resets))
		return dev_err_probe(&pdev->dev, PTR_ERR(core->resets),
				     "failed to get the resets\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	core->irq = irq;

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					rkvdec_irq_handler, IRQF_ONESHOT,
					dev_name(&pdev->dev), core);
	if (ret) {
		dev_err(&pdev->dev, "Could not request core IRQ\n");
		return ret;
	}

	core->sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram", 0);
	if (!core->sram_pool && variant->num_rcb_sizes > 0)
		dev_info(&pdev->dev, "No sram node, RCB will be stored in RAM\n");

	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	platform_set_drvdata(pdev, core);

	ret = component_add(&pdev->dev, &rkvdec_core_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register component: %d\n", ret);
		goto err_disable_runtime_pm;
	}

	return 0;
err_disable_runtime_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (core->empty_domain)
		iommu_domain_free(core->empty_domain);

	return ret;
}

static void rkvdec_core_remove(struct platform_device *pdev)
{
	struct rkvdec_core *core = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &rkvdec_core_ops);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	if (core->empty_domain)
		iommu_domain_free(core->empty_domain);
}

static int rkvdec_core_runtime_resume(struct device *dev)
{
	struct rkvdec_core *core = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(core->num_clocks, core->clocks);
}

static int rkvdec_core_runtime_suspend(struct device *dev)
{
	struct rkvdec_core *core = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(core->num_clocks, core->clocks);

	return 0;
}

static const struct dev_pm_ops rkvdec_core_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(rkvdec_core_runtime_suspend, rkvdec_core_runtime_resume, NULL)
};

static struct platform_driver rkvdec_core_pdrv = {
	.probe = rkvdec_core_probe,
	.remove = rkvdec_core_remove,
	.driver = {
		   .name = "rkvdec-core",
		   .of_match_table = of_rkvdec_match,
		   .pm = pm_ptr(&rkvdec_core_pm_ops),
	},
};

/*
 * Returns the CCU the available cores are connected to, if they all are
 * connected to the same available one.
 */
static struct device_node *rkvdec_find_ccu_node(const char *compatible)
{
	struct device_node *core_node, *ccu_node = NULL;

	for_each_compatible_node(core_node, NULL, compatible) {
		struct device_node *np;

		if (!of_device_is_available(core_node))
			continue;

		np = of_parse_phandle(core_node, "rockchip,ccu", 0);
		if (!np || (ccu_node && np != ccu_node)) {
			of_node_put(np);
			of_node_put(ccu_node);
			of_node_put(core_node);
			return NULL;
		}

		of_node_put(ccu_node);
		ccu_node = np;
	}

	if (ccu_node && !of_device_is_available(ccu_node)) {
		of_node_put(ccu_node);
		return NULL;
	}

	return ccu_node;
}

static int rkvdec_bind(struct device *dev)
{
	struct rkvdec_dev *rkvdec = dev_get_drvdata(dev);
	int ret;

	ret = component_bind_all(dev, rkvdec);
	if (ret) {
		dev_err(dev, "component bind failed\n");
		return ret;
	}

	if (rkvdec->variant->has_ccu) {
		const struct of_device_id *match_desc = dev->platform_data;
		struct device_node *np;

		np = rkvdec_find_ccu_node(match_desc->compatible);
		if (np) {
			rkvdec->ccu = rkvdec_ccu_find(np);
			of_node_put(np);
		}
	}

	if (rkvdec->ccu) {
		unsigned long ports = 0;
		int i;

		for (i = 0; i < rkvdec->core_count; i++) {
			int port = rkvdec->cores[i]->ccu_port;

			if (port < 0 || port >= 16 || (ports & BIT(port)))
				break;
			ports |= BIT(port);
		}

		if (i < rkvdec->core_count) {
			dev_warn(rkvdec->cores[i]->dev,
				 "invalid CCU port, not using the CCU\n");
			rkvdec_ccu_release(rkvdec->ccu);
			rkvdec->ccu = NULL;
		} else {
			dev_info(dev, "%d cores, decoding with the CCU\n",
				 rkvdec->core_count);
		}
	}

	ret = rkvdec_v4l2_init(rkvdec);
	if (ret)
		goto err_unbind;

	return 0;

err_unbind:
	if (rkvdec->ccu)
		rkvdec_ccu_release(rkvdec->ccu);
	rkvdec->ccu = NULL;
	component_unbind_all(dev, NULL);
	return ret;
}

static void rkvdec_unbind(struct device *dev)
{
	struct rkvdec_dev *rkvdec = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < rkvdec->core_count; i++)
		cancel_delayed_work_sync(&rkvdec->cores[i]->watchdog_work);

	rkvdec_v4l2_cleanup(rkvdec);

	for (i = 0; i < rkvdec->core_count; i++)
		rkvdec_free_rcb(rkvdec, rkvdec->cores[i]);

	if (rkvdec->ccu)
		rkvdec_ccu_release(rkvdec->ccu);
	rkvdec->ccu = NULL;

	component_unbind_all(dev, NULL);
}

static const struct component_master_ops rkvdec_master_ops = {
	.bind = rkvdec_bind,
	.unbind = rkvdec_unbind,
};

static int rkvdec_probe(struct platform_device *pdev)
{
	const struct of_device_id *match_desc = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *core_node;
	struct rkvdec_dev *rkvdec;
	unsigned int num_cores = 0;

	if (!match_desc)
		return dev_err_probe(dev, -ENODEV, "missing platform data\n");

	for_each_compatible_node(core_node, NULL, match_desc->compatible) {
		if (!of_device_is_available(core_node))
			continue;

		of_node_get(core_node);
		component_match_add_release(dev, &match, component_release_of,
					    component_compare_of, core_node);
		num_cores++;
	}

	if (!match)
		return dev_err_probe(dev, -ENODEV,
				     "no matching available component devices found\n");

	rkvdec = devm_kzalloc(dev, sizeof(*rkvdec), GFP_KERNEL);
	if (!rkvdec)
		return -ENOMEM;

	rkvdec->cores = devm_kcalloc(dev, num_cores, sizeof(*rkvdec->cores),
				     GFP_KERNEL);
	if (!rkvdec->cores)
		return -ENOMEM;

	rkvdec->available_cores = devm_kcalloc(dev, num_cores,
					       sizeof(*rkvdec->available_cores),
					       GFP_KERNEL);
	if (!rkvdec->available_cores)
		return -ENOMEM;

	rkvdec->variant = match_desc->data;
	if (!rkvdec->variant)
		return dev_err_probe(dev, -ENODEV, "failed to get match data\n");

	mutex_init(&rkvdec->vdev_lock);
	spin_lock_init(&rkvdec->cores_lock);
	ida_init(&rkvdec->film_ida);
	rkvdec->next_film_idx = 1;

	dev_set_drvdata(dev, rkvdec);

	return component_master_add_with_match(dev, &rkvdec_master_ops, match);
}

static void rkvdec_remove(struct platform_device *pdev)
{
	struct rkvdec_dev *rkvdec = platform_get_drvdata(pdev);

	component_master_del(&pdev->dev, &rkvdec_master_ops);
	ida_destroy(&rkvdec->film_ida);
}

static struct platform_driver rkvdec_pdrv = {
	.probe = rkvdec_probe,
	.remove = rkvdec_remove,
	.driver = {
		.name = "rkvdec",
	},
};

static bool rkvdec_of_has_available_node(const char *compat)
{
	struct device_node *node;

	for_each_compatible_node(node, NULL, compat) {
		if (of_device_is_available(node)) {
			of_node_put(node);
			return true;
		}
	}

	return false;
}

static int rkvdec_create_platform_device(struct platform_device **ppdev,
					 const struct of_device_id *match)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(match->compatible, PLATFORM_DEVID_NONE);
	if (!pdev)
		return -ENOMEM;

	ret = platform_device_add_data(pdev, match, sizeof(*match));
	if (ret)
		goto free_platform_device;

	ret = platform_device_add(pdev);
	if (ret)
		goto free_platform_device;

	ret = device_driver_attach(&rkvdec_pdrv.driver, &pdev->dev);
	if (ret)
		goto del_platform_device;

	*ppdev = pdev;

	return 0;

del_platform_device:
	platform_device_del(pdev);
free_platform_device:
	platform_device_put(pdev);
	return ret;
}

static struct platform_device *master_pdevs[ARRAY_SIZE(of_rkvdec_match) - 1];

static int __init rkvdec_init(void)
{
	unsigned int i;
	int ret;

	ret = platform_driver_register(&rkvdec_core_pdrv);
	if (ret)
		return ret;

	ret = platform_driver_register(&rkvdec_ccu_pdrv);
	if (ret)
		goto unregister_core_driver;

	ret = platform_driver_register(&rkvdec_pdrv);
	if (ret)
		goto unregister_ccu_driver;

	for (i = 0; i < ARRAY_SIZE(master_pdevs); i++) {
		if (!rkvdec_of_has_available_node(of_rkvdec_match[i].compatible))
			continue;

		ret = rkvdec_create_platform_device(&master_pdevs[i],
						    &of_rkvdec_match[i]);
		if (ret)
			goto unregister_platform_devices;
	}

	return 0;

unregister_platform_devices:
	for (i = 0; i < ARRAY_SIZE(master_pdevs); i++) {
		if (master_pdevs[i]) {
			platform_device_unregister(master_pdevs[i]);
			master_pdevs[i] = NULL;
		}
	}
	platform_driver_unregister(&rkvdec_pdrv);
unregister_ccu_driver:
	platform_driver_unregister(&rkvdec_ccu_pdrv);
unregister_core_driver:
	platform_driver_unregister(&rkvdec_core_pdrv);
	return ret;
}
module_init(rkvdec_init);

static void __exit rkvdec_exit(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(master_pdevs); i++) {
		if (master_pdevs[i]) {
			platform_device_unregister(master_pdevs[i]);
			master_pdevs[i] = NULL;
		}
	}
	platform_driver_unregister(&rkvdec_pdrv);
	platform_driver_unregister(&rkvdec_ccu_pdrv);
	platform_driver_unregister(&rkvdec_core_pdrv);
}
module_exit(rkvdec_exit);

MODULE_AUTHOR("Boris Brezillon <boris.brezillon@collabora.com>");
MODULE_DESCRIPTION("Rockchip Video Decoder driver");
MODULE_LICENSE("GPL v2");
