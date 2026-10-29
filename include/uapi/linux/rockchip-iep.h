/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * Rockchip IEP (image enhancement processor) deinterlacer controls
 */

#ifndef _UAPI_LINUX_ROCKCHIP_IEP_H
#define _UAPI_LINUX_ROCKCHIP_IEP_H

#include <linux/v4l2-controls.h>

/**
 * V4L2_CID_ROCKCHIP_IEP_DEINT_MODE - deinterlacing algorithm
 *
 * Menu control, values from &enum v4l2_rockchip_iep_deint_mode.
 */
#define V4L2_CID_ROCKCHIP_IEP_DEINT_MODE	(V4L2_CID_USER_ROCKCHIP_IEP_BASE + 0)

/**
 * enum v4l2_rockchip_iep_deint_mode - deinterlacing algorithm
 *
 * @V4L2_ROCKCHIP_IEP_DEINT_MODE_BOB: intra-field interpolation (bob). Each
 *	output frame is built from a single field.
 * @V4L2_ROCKCHIP_IEP_DEINT_MODE_MOTION_ADAPTIVE: motion-adaptive and
 *	motion-compensated interpolation using the previous and current
 *	frames. The previous OUTPUT buffer is held by the driver until the
 *	next one has been processed.
 */
enum v4l2_rockchip_iep_deint_mode {
	V4L2_ROCKCHIP_IEP_DEINT_MODE_BOB = 0,
	V4L2_ROCKCHIP_IEP_DEINT_MODE_MOTION_ADAPTIVE = 1,
};

/**
 * V4L2_CID_ROCKCHIP_IEP_FIELD_RATE - output one frame per field
 *
 * Boolean control. When set, every interlaced OUTPUT buffer produces two
 * progressive CAPTURE buffers (doubled frame rate, e.g. 50i to 50p). The
 * second CAPTURE buffer gets a timestamp half a frame period after the
 * first one. When clear, one CAPTURE buffer is produced per OUTPUT buffer.
 */
#define V4L2_CID_ROCKCHIP_IEP_FIELD_RATE	(V4L2_CID_USER_ROCKCHIP_IEP_BASE + 1)

#endif /* _UAPI_LINUX_ROCKCHIP_IEP_H */
