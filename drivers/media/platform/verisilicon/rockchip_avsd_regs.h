/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hantro VPU codec driver
 *
 * Registers of the AVS+ decoder found next to the VPU121 of the RK3588
 * (and the VPU of the RK3328). It has the register layout of the Hantro G1
 * decoder, with AVS+ extensions.
 */

#ifndef ROCKCHIP_AVSD_REGS_H_
#define ROCKCHIP_AVSD_REGS_H_

#define AVSD_NUM_REGS				60

#define AVSD_REG_ID				0
#define     AVSD_ID_PRODUCT(x)			(((x) >> 16) & 0xffff)

#define AVSD_REG_INTERRUPT			1
#define     AVSD_INT_DEC_E			BIT(0)
#define     AVSD_INT_IRQ_DIS			BIT(4)
#define     AVSD_INT_IRQ			BIT(8)
#define     AVSD_INT_RDY			BIT(12)
#define     AVSD_INT_BUS			BIT(13)
#define     AVSD_INT_BUFFER			BIT(14)
#define     AVSD_INT_ASO			BIT(15)
#define     AVSD_INT_ERROR			BIT(16)
#define     AVSD_INT_TIMEOUT			BIT(18)

#define AVSD_REG_CONFIG				2
#define     AVSD_CFG_MAX_BURST(x)		((u32)((x) & 0x1f) << 0)
#define     AVSD_CFG_OUT_ENDIAN			BIT(8)
#define     AVSD_CFG_IN_ENDIAN			BIT(9)
#define     AVSD_CFG_CLK_GATE_E			BIT(10)
#define     AVSD_CFG_OUTSWAP32_E		BIT(19)
#define     AVSD_CFG_INSWAP32_E			BIT(20)
#define     AVSD_CFG_STRENDIAN_E		BIT(21)
#define     AVSD_CFG_STRSWAP32_E		BIT(22)
#define     AVSD_CFG_TIMEOUT_E			BIT(23)
#define     AVSD_CFG_AXI_RD_ID(x)		((u32)((x) & 0xff) << 24)

#define AVSD_REG_CTRL				3
#define     AVSD_CTRL_WRITE_MVS_E		BIT(12)
#define     AVSD_CTRL_PIC_FIXED_QUANT		BIT(13)
#define     AVSD_CTRL_FILTERING_DIS		BIT(14)
#define     AVSD_CTRL_PIC_TOPFIELD_E		BIT(19)
#define     AVSD_CTRL_PIC_INTER_E		BIT(20)
#define     AVSD_CTRL_PIC_B_E			BIT(21)
#define     AVSD_CTRL_PIC_FIELDMODE_E		BIT(22)
#define     AVSD_CTRL_PIC_INTERLACE_E		BIT(23)
#define     AVSD_CTRL_SKIP_MODE			BIT(26)
#define     AVSD_CTRL_DEC_MODE(x)		((u32)((x) & 0xf) << 28)
#define     AVSD_DEC_MODE_AVS			11

#define AVSD_REG_PIC_SIZE			4
#define     AVSD_PIC_REFER_FLAG			BIT(0)
#define     AVSD_PIC_MB_HEIGHT_P(x)		((u32)((x) & 0xff) << 11)
#define     AVSD_PIC_MB_WIDTH(x)		((u32)((x) & 0x1ff) << 23)

#define AVSD_REG_STREAM_CFG			5
#define     AVSD_BETA_OFFSET(x)			((u32)((x) & 0x1f) << 0)
#define     AVSD_ALPHA_OFFSET(x)		((u32)((x) & 0x1f) << 5)
#define     AVSD_STRM_START_BIT(x)		((u32)((x) & 0x3f) << 26)

#define AVSD_REG_STREAM_LEN			6
#define     AVSD_STREAM_LEN(x)			((u32)((x) & 0x1ffffff) << 0)
#define     AVSD_INIT_QP(x)			((u32)((x) & 0x3f) << 25)

#define AVSD_REG_EXT				7
#define     AVSD_H_EXT(x)			((u32)((x) & 0x1) << 25)

#define AVSD_REG_STREAM_BASE			12
#define AVSD_REG_OUT_BASE			13
#define AVSD_REG_REF_BASE(i)			(14 + (i))
#define     AVSD_REF_FIELD_E			BIT(1)
#define     AVSD_REF_TOP_E			BIT(0)

#define AVSD_REG_ANCHOR				18
#define     AVSD_PREV_ANC_TYPE			BIT(0)

/* Pairs of 16-bit values: _0 in the low half, _1 in the high half. */
#define AVSD_REG_REF_INVD_CUR(i)		(28 + (i) / 2)
#define AVSD_REG_REF_DIST_CUR(i)		(30 + (i) / 2)
#define AVSD_REG_REF_INVD_COL(i)		(32 + (i) / 2)

#define AVSD_REG_PRED_BC_TAP_A			34
#define AVSD_REG_PRED_BC_TAP_B			35
#define AVSD_REG_DIR_MV_BASE			41

#define AVSD_REG_REF_DELTA			42
#define     AVSD_REF_DELTA_CUR(i, x)		(((x) & 0x7) << (9 - 3 * (i)))
#define     AVSD_REF_DELTA_COL(i, x)		(((x) & 0x7) << (21 - 3 * (i)))
#define     AVSD_WEIGHT_QP_1(x)			((u32)((x) & 0xff) << 24)

#define AVSD_REG_WEIGHT_QP			43
#define     AVSD_WEIGHT_QP_2(x)			((u32)((x) & 0xff) << 24)
#define     AVSD_WEIGHT_QP_3(x)			((u32)((x) & 0xff) << 16)
#define     AVSD_WEIGHT_QP_4(x)			((u32)((x) & 0xff) << 8)
#define     AVSD_WEIGHT_QP_5(x)			((u32)((x) & 0xff) << 0)

#define AVSD_REG_AVSP				44
#define     AVSD_WEIGHT_QP_0(x)			((u32)((x) & 0xff) << 0)
#define     AVSD_QP_DELTA_CR(x)			((u32)((x) & 0x3f) << 8)
#define     AVSD_QP_DELTA_CB(x)			((u32)((x) & 0x3f) << 14)
#define     AVSD_PB_FIELD_ENHANCE_E		BIT(20)
#define     AVSD_NO_FWD_REF_E			BIT(21)
#define     AVSD_AEC_E				BIT(22)
#define     AVSD_WEIGHT_QP_MODEL(x)		((u32)((x) & 0x3) << 23)
#define     AVSD_WEIGHT_QP_E			BIT(25)
#define     AVSD_AVSP_E				BIT(26)

#define AVSD_REG_DIR_MV_BASE2			45
#define AVSD_REG_PRED_BC_TAP_C			49

#define AVSD_REG_APF				55
#define     AVSD_APF_THRESHOLD(x)		((u32)((x) & 0x3fff) << 0)

/* Read-only synthesis configuration */
#define AVSD_REG_SYNTH_CFG			50
#define AVSD_REG_SYNTH_CFG_2			54

/* Byte offset of the soft reset register */
#define AVSD_SOFT_RESET				0x194

#endif /* ROCKCHIP_AVSD_REGS_H_ */
