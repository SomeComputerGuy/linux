/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip RK3588 video decoder core coordination unit
 */
#ifndef RKVDEC_CCU_H_
#define RKVDEC_CCU_H_

#include <linux/of.h>
#include <linux/platform_device.h>

struct rkvdec_core;
struct rkvdec_ctx;
struct rkvdec_vdpu381_regs_common;

extern struct platform_driver rkvdec_ccu_pdrv;

struct rkvdec_ccu *rkvdec_ccu_find(struct device_node *np);
void rkvdec_ccu_release(struct rkvdec_ccu *ccu);

int rkvdec_ccu_get(struct rkvdec_ccu *ccu);
void rkvdec_ccu_put(struct rkvdec_ccu *ccu);
bool rkvdec_ctx_uses_ccu(struct rkvdec_ctx *ctx);
void rkvdec_ccu_prepare(struct rkvdec_ctx *ctx);
void rkvdec_ccu_start(struct rkvdec_ctx *ctx);
void rkvdec_ccu_disconnect(struct rkvdec_ccu *ccu, struct rkvdec_core *core);
void rkvdec_ccu_reconnect(struct rkvdec_ccu *ccu, struct rkvdec_core *core);
void rkvdec_vdpu381_set_ccu_regs(struct rkvdec_ctx *ctx,
				 struct rkvdec_vdpu381_regs_common *common);

#endif /* RKVDEC_CCU_H_ */
