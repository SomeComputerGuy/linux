// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3588 video decoder core coordination unit (CCU)
 *
 * The two VDPU381 cores of the RK3588 can decode consecutive frames of one
 * stream at the same time. A core that is connected to the CCU tells it which
 * stream (film index) and which part of a frame it has written so far, and
 * the reads of reference pixels and co-located motion vectors from a frame
 * that another core is still decoding wait until that part is written.
 *
 * The register layout follows the Rockchip BSP driver (mpp_rkvdec2_link.c).
 * The core bit registers use the upper 16 bits as write enable mask.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "rkvdec.h"
#include "rkvdec-ccu.h"
#include "rkvdec-vdpu381-regs.h"

#define RKVDEC_CCU_WORK			0x018
#define RKVDEC_CCU_WORK_EN		BIT(0)
#define RKVDEC_CCU_WORK_MODE		0x040
#define RKVDEC_CCU_WORK_MODE_TASK	BIT(0)
#define RKVDEC_CCU_CORE_WORK		0x044
#define RKVDEC_CCU_CORE_STA		0x048
#define RKVDEC_CCU_CORE_IDLE		0x04c
#define RKVDEC_CCU_CORE_ERR		0x054
/* Sets the bit of a core in the CORE_* registers */
#define RKVDEC_CCU_CORE_SET(port)	(0x10001U << (port))

/* Register in the link block of each core */
#define VDPU381_LINK_CTRL		0x000
#define VDPU381_LINK_CORE_WORK_MODE	BIT(16)
#define VDPU381_LINK_CCU_WORK_MODE	BIT(17)

struct rkvdec_ccu {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data *clocks;
	int num_clocks;
	unsigned int users;
};

int rkvdec_ccu_get(struct rkvdec_ccu *ccu)
{
	return pm_runtime_resume_and_get(ccu->dev);
}

void rkvdec_ccu_put(struct rkvdec_ccu *ccu)
{
	pm_runtime_mark_last_busy(ccu->dev);
	pm_runtime_put_autosuspend(ccu->dev);
}

/*
 * Only formats whose back-end programs what the cores synchronize on (the
 * POCs of the frame and of its references, for VP9 also those of the frames
 * that saved the probabilities and the segment IDs it reads), see
 * RKVDEC_FMT_FLAG_PIPELINE, run in CCU mode.
 */
bool rkvdec_ctx_uses_ccu(struct rkvdec_ctx *ctx)
{
	return ctx->dev->ccu &&
	       (ctx->coded_fmt_desc->flags & RKVDEC_FMT_FLAG_PIPELINE);
}

/*
 * Connects the core of the job being set up to the CCU, or disconnects it for
 * a format that doesn't use the CCU, before the registers are written.
 */
void rkvdec_ccu_prepare(struct rkvdec_ctx *ctx)
{
	struct rkvdec_ccu *ccu = ctx->dev->ccu;
	struct rkvdec_core *core = ctx->core;

	if (!rkvdec_ctx_uses_ccu(ctx)) {
		writel(0, core->link + VDPU381_LINK_CTRL);
		writel(BIT(core->ccu_port + 16), ccu->regs + RKVDEC_CCU_CORE_WORK);
		return;
	}

	writel(VDPU381_LINK_CORE_WORK_MODE | VDPU381_LINK_CCU_WORK_MODE,
	       core->link + VDPU381_LINK_CTRL);
	writel(RKVDEC_CCU_WORK_EN, ccu->regs + RKVDEC_CCU_WORK);
	writel(RKVDEC_CCU_WORK_MODE_TASK, ccu->regs + RKVDEC_CCU_WORK_MODE);
	writel(RKVDEC_CCU_CORE_SET(core->ccu_port), ccu->regs + RKVDEC_CCU_CORE_WORK);
}

/*
 * Forces a core that is being reset idle in the CCU, so that the other core
 * doesn't wait for it, and connects it again with its error state cleared,
 * as the BSP's soft-CCU reset does.
 */
void rkvdec_ccu_disconnect(struct rkvdec_ccu *ccu, struct rkvdec_core *core)
{
	writel(RKVDEC_CCU_CORE_SET(core->ccu_port), ccu->regs + RKVDEC_CCU_CORE_IDLE);
}

void rkvdec_ccu_reconnect(struct rkvdec_ccu *ccu, struct rkvdec_core *core)
{
	writel(BIT(core->ccu_port + 16), ccu->regs + RKVDEC_CCU_CORE_ERR);
	writel(BIT(core->ccu_port + 16), ccu->regs + RKVDEC_CCU_CORE_IDLE);
}

/* Tells the CCU that the core of the job being set up starts decoding. */
void rkvdec_ccu_start(struct rkvdec_ctx *ctx)
{
	struct rkvdec_ccu *ccu = ctx->dev->ccu;

	if (rkvdec_ctx_uses_ccu(ctx))
		writel(RKVDEC_CCU_CORE_SET(ctx->core->ccu_port),
		       ccu->regs + RKVDEC_CCU_CORE_STA);
}

void rkvdec_vdpu381_set_ccu_regs(struct rkvdec_ctx *ctx,
				 struct rkvdec_vdpu381_regs_common *common)
{
	if (!rkvdec_ctx_uses_ccu(ctx))
		return;

	/* Tells the frames of different streams apart. */
	common->reg028_multiply_core_ctrl.film_idx = ctx->film_idx;
	/*
	 * Waiting for the other core is expected: don't reset the core when a
	 * reference or co-located motion vector read takes long.
	 */
	common->reg013_en_mode_set.req_timeout_rst_sel = 1;
}

/*
 * Returns the CCU of @np if its driver is bound, NULL otherwise: the decoder
 * then works without it. Release it with rkvdec_ccu_release().
 */
struct rkvdec_ccu *rkvdec_ccu_find(struct device_node *np)
{
	struct platform_device *pdev;
	struct rkvdec_ccu *ccu;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;

	device_lock(&pdev->dev);
	ccu = platform_get_drvdata(pdev);
	if (ccu)
		ccu->users++;
	device_unlock(&pdev->dev);

	if (!ccu)
		put_device(&pdev->dev);

	return ccu;
}

void rkvdec_ccu_release(struct rkvdec_ccu *ccu)
{
	struct device *dev = ccu->dev;

	device_lock(dev);
	ccu->users--;
	device_unlock(dev);
	put_device(dev);
}

static int rkvdec_ccu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct rkvdec_ccu *ccu;
	int ret;

	ccu = devm_kzalloc(dev, sizeof(*ccu), GFP_KERNEL);
	if (!ccu)
		return -ENOMEM;

	ccu->dev = dev;
	ccu->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ccu->regs))
		return PTR_ERR(ccu->regs);

	ccu->num_clocks = devm_clk_bulk_get_all(dev, &ccu->clocks);
	if (ccu->num_clocks < 0)
		return dev_err_probe(dev, ccu->num_clocks, "failed to get clocks\n");

	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to get reset\n");

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, ccu);

	return 0;
}

static void rkvdec_ccu_remove(struct platform_device *pdev)
{
	struct rkvdec_ccu *ccu = platform_get_drvdata(pdev);

	/* The decoder is unbound before the CCU when the module is removed. */
	WARN_ON(ccu->users);
	platform_set_drvdata(pdev, NULL);
}

static int rkvdec_ccu_runtime_resume(struct device *dev)
{
	struct rkvdec_ccu *ccu = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ccu->num_clocks, ccu->clocks);
}

static int rkvdec_ccu_runtime_suspend(struct device *dev)
{
	struct rkvdec_ccu *ccu = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ccu->num_clocks, ccu->clocks);

	return 0;
}

static const struct dev_pm_ops rkvdec_ccu_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(rkvdec_ccu_runtime_suspend, rkvdec_ccu_runtime_resume, NULL)
};

static const struct of_device_id rkvdec_ccu_of_match[] = {
	{ .compatible = "rockchip,rk3588-vdec-ccu" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rkvdec_ccu_of_match);

struct platform_driver rkvdec_ccu_pdrv = {
	.probe = rkvdec_ccu_probe,
	.remove = rkvdec_ccu_remove,
	.driver = {
		.name = "rkvdec-ccu",
		.of_match_table = rkvdec_ccu_of_match,
		.pm = pm_ptr(&rkvdec_ccu_pm_ops),
	},
};
