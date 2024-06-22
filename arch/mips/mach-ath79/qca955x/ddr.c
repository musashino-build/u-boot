// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 INAGAKI Hiroshi <musashino.open@gmail.com>
 *
 * Based on QCA956x support and QSDK
 */

#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/addrspace.h>
#include <asm/types.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <mach/ar71xx_regs.h>
#include <mach/ath79.h>

#define DDR_CONFIG_CAS_LATENCY_MSB_MASK		BIT(31)
#define DDR_CONFIG_OPEN_PAGE_MASK		BIT(30)
#define DDR_CONFIG_CAS_LATENCY_MASK		GENMASK(29, 27)
#define DDR_CONFIG_TMRD_MASK			GENMASK(26, 23)
#define DDR_CONFIG_TRFC_MASK			GENMASK(22, 17)
#define DDR_CONFIG_TRRD_MASK			GENMASK(16, 13)
#define DDR_CONFIG_TRP_MASK			GENMASK(12, 9)
#define DDR_CONFIG_TRCD_MASK			GENMASK(8, 5)
#define DDR_CONFIG_TRAS_MASK			GENMASK(4, 0)

#define DDR_CONFIG2_HALF_WIDTH_LOW_MASK		BIT(31)
#define DDR_CONFIG2_SWAP_A26_A27_MASK		BIT(30)
#define DDR_CONFIG2_GATE_OPEN_LATENCY_MASK	GENMASK(29, 26)
#define DDR_CONFIG2_TWTR_MASK			GENMASK(25, 21)
#define DDR_CONFIG2_TRTP_MASK			GENMASK(20, 17)
#define DDR_CONFIG2_TRTW_MASK			GENMASK(16, 12)
#define DDR_CONFIG2_TWR_MASK			GENMASK(11, 8)
#define DDR_CONFIG2_CKE_MASK			BIT(7)
#define DDR_CONFIG2_CNTL_OE_EN_MASK		BIT(5)
#define DDR_CONFIG2_BURST_LENGTH_MASK		GENMASK(3, 0)

#define DDR_CTL_CONFIG_SRAM_TSEL_MASK		GENMASK(31, 30)
#define DDR_CTL_CONFIG_GE0_SRAM_SYNC_MASK	BIT(20)
#define DDR_CTL_CONFIG_GE1_SRAM_SYNC_MASK	BIT(19)
#define DDR_CTL_CONFIG_USB_SRAM_SYNC_MASK	BIT(18)
#define DDR_CTL_CONFIG_PCIE_SRAM_SYNC_MASK	BIT(17)
#define DDR_CTL_CONFIG_WMAC_SRAM_SYNC_MASK	BIT(16)
#define DDR_CTL_CONFIG_MISC_SRC1_SRAM_SYNC_MASK	BIT(15)
#define DDR_CTL_CONFIG_MISC_SRC2_SRAM_SYNC_MASK	BIT(14)
#define DDR_CTL_CONFIG_PAD_DDR2_SEL_MASK	BIT(6)
#define DDR_CTL_CONFIG_CPU_DDR_SYNC_MASK	BIT(2)
#define DDR_CTL_CONFIG_HALF_WIDTH_MASK		BIT(1)

#define RST_BOOTSTRAP_DDR_WIDTH_MASK		BIT(3)

#define PMU2_PGM_MASK				BIT(21)
#define PMU2_LDO_TUNE_MASK			GENMASK(20, 19)

/*
* DDR2                      DDR1
* 0x40c3   25MHz            0x4186   25Mhz
* 0x4138   40MHz            0x4270   40Mhz
*/
#define CFG_DDR2_REFRESH_VAL 0x4138

#define CFG_DDR2_CONFIG_VAL	\
		DDR_CONFIG_CAS_LATENCY_MSB_MASK | DDR_CONFIG_OPEN_PAGE_MASK | \
		FIELD_PREP(DDR_CONFIG_CAS_LATENCY_MASK, 0x4) | \
		FIELD_PREP(DDR_CONFIG_TMRD_MASK, 0xf) | \
		FIELD_PREP(DDR_CONFIG_TRFC_MASK, 0x15) | \
		FIELD_PREP(DDR_CONFIG_TRRD_MASK, 0x7) | \
		FIELD_PREP(DDR_CONFIG_TRP_MASK, 0x9) | \
		FIELD_PREP(DDR_CONFIG_TRCD_MASK, 0x9) | \
		FIELD_PREP(DDR_CONFIG_TRAS_MASK, 0x1b)

#define CFG_DDR2_CONFIG2_VAL	\
		DDR_CONFIG2_HALF_WIDTH_LOW_MASK | /* SWAP_A26_A27 is off */ \
		FIELD_PREP(DDR_CONFIG2_GATE_OPEN_LATENCY_MASK, 0xb) | \
		FIELD_PREP(DDR_CONFIG2_TWTR_MASK, 0x15) | \
		FIELD_PREP(DDR_CONFIG2_TRTP_MASK, 0x9) | \
		FIELD_PREP(DDR_CONFIG2_TRTW_MASK, 0xe) | \
		FIELD_PREP(DDR_CONFIG2_TWR_MASK, 0x1) | \
		DDR_CONFIG2_CKE_MASK | DDR_CONFIG2_CNTL_OE_EN_MASK | \
		FIELD_PREP(DDR_CONFIG2_BURST_LENGTH_MASK, 0x8)

#define CFG_DDR2_CONFIG3_VAL			0x0000000a
#define CFG_DDR2_EXT_MODE_VAL1			0x782
#define CFG_DDR2_EXT_MODE_VAL2			0x402
#define CFG_DDR2_MODE_VAL_INIT			0x153
#define CFG_DDR2_MODE_VAL			0x53
#define CFG_DDR2_TAP_VAL			0x10
#define CFG_DDR2_EN_TWL_VAL			0x00001e7d
#define CFG_DDR2_RD_DATA_THIS_CYCLE_VAL_16	0xffff
#define CFG_DDR2_RD_DATA_THIS_CYCLE_VAL_32	0xff

#define CFG_DDR_CTL_CONFIG	\
		FIELD_PREP(DDR_CTL_CONFIG_SRAM_TSEL_MASK, 0x1) | \
		DDR_CTL_CONFIG_GE0_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_GE1_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_USB_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_PCIE_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_WMAC_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_MISC_SRC1_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_MISC_SRC2_SRAM_SYNC_MASK | \
		DDR_CTL_CONFIG_PAD_DDR2_SEL_MASK /* for DDR2 */
		/* CPU_DDR_SYNC is off (CPU clk != DDR clk) */

DECLARE_GLOBAL_DATA_PTR;

void qca955x_ddr_init(void)
{
	u32 cycle_val = CFG_DDR2_RD_DATA_THIS_CYCLE_VAL_32;
	u32 ctl_config = CFG_DDR_CTL_CONFIG;
	void __iomem *ddr_regs = map_physmem(AR71XX_DDR_CTRL_BASE, AR71XX_DDR_CTRL_SIZE,
			       MAP_NOCACHE);
	void __iomem *srif_regs = map_physmem(QCA955X_SRIF_BASE, QCA955X_SRIF_SIZE,
			       MAP_NOCACHE);

	/* 16bit */
	if (!(ath79_get_bootstrap() & RST_BOOTSTRAP_DDR_WIDTH_MASK))
	{
		ctl_config |= DDR_CTL_CONFIG_HALF_WIDTH_MASK;
		cycle_val = CFG_DDR2_RD_DATA_THIS_CYCLE_VAL_16;
	}

	writel(0x10, ddr_regs + AR71XX_DDR_REG_CONTROL);
	udelay(10);

	writel(0x20, ddr_regs + AR71XX_DDR_REG_CONTROL);
	udelay(10);

	writel(ctl_config, ddr_regs + QCA955X_DDR_REG_CTL_CONF);
	udelay(10);

	writel(cycle_val, ddr_regs + AR71XX_DDR_REG_RD_CYCLE);
	udelay(100);

	writel(0x74444444, ddr_regs + QCA955X_DDR_REG_BURST);
	udelay(100);

	writel(0x44444444, ddr_regs + QCA955X_DDR_REG_BURST2);
	udelay(100);

	writel(0xfffff, ddr_regs + QCA955X_DDR_REG_TIMEOUT_MAX);
	udelay(100);

	writel(CFG_DDR2_CONFIG_VAL, ddr_regs + AR71XX_DDR_REG_CONFIG);
	udelay(100);

	writel(CFG_DDR2_CONFIG2_VAL, ddr_regs + AR71XX_DDR_REG_CONFIG2);
	udelay(100);

	writel(CFG_DDR2_CONFIG3_VAL, ddr_regs + QCA955X_DDR_REG_CONFIG3);
	udelay(100);

	writel(CFG_DDR2_EN_TWL_VAL, ddr_regs + QCA955X_DDR_REG_DDR2_CONFIG);
	udelay(100);

	writel(CFG_DDR2_CONFIG2_VAL | 0x80,
		ddr_regs + AR71XX_DDR_REG_CONFIG2);	/* CKE Enable */
	udelay(100);

	writel(0x8, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* Precharge */
	udelay(10);

	writel(0x10, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* EMR2 */
	udelay(10);

	writel(0x20, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* EMR3 */
	udelay(10);

	/* EMR DLL enable, Reduced Driver Impedance control, Differential DQS disabled */
	writel(CFG_DDR2_EXT_MODE_VAL2, ddr_regs + AR71XX_DDR_REG_EMR);
	udelay(100);

	writel(0x2, ddr_regs + AR71XX_DDR_REG_CONTROL); /* EMR write */
	udelay(10);

	writel(CFG_DDR2_MODE_VAL_INIT, ddr_regs + AR71XX_DDR_REG_MODE);
	udelay(1000);

	writel(0x1, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* MR Write */
	udelay(10);

	writel(0x8, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* Precharge */
	udelay(10);

	writel(0x4, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* Auto Refresh */
	udelay(10);

	writel(0x4, ddr_regs + AR71XX_DDR_REG_CONTROL);	/* Auto Refresh */
	udelay(10);

	/* Issue MRS to remove DLL out-of-reset */
	writel(CFG_DDR2_MODE_VAL, ddr_regs + AR71XX_DDR_REG_MODE);
	udelay(100);

	writel(0x1, ddr_regs + AR71XX_DDR_REG_CONTROL); /* MR write */
	udelay(100);

	writel(CFG_DDR2_EXT_MODE_VAL1, ddr_regs + AR71XX_DDR_REG_EMR);
	udelay(100);

	writel(0x2, ddr_regs + AR71XX_DDR_REG_CONTROL); /* EMR write */
	udelay(100);

	writel(CFG_DDR2_EXT_MODE_VAL2, ddr_regs + AR71XX_DDR_REG_EMR);
	udelay(100);

	writel(0x2, ddr_regs + AR71XX_DDR_REG_CONTROL); /* EMR write */
	udelay(100);

	writel(CFG_DDR2_REFRESH_VAL, ddr_regs + AR71XX_DDR_REG_REFRESH);
	udelay(100);

	writel(CFG_DDR2_TAP_VAL, ddr_regs + AR71XX_DDR_REG_TAP_CTRL0);
	writel(CFG_DDR2_TAP_VAL, ddr_regs + AR71XX_DDR_REG_TAP_CTRL1);
	writel(CFG_DDR2_TAP_VAL, ddr_regs + QCA955X_DDR_REG_TAP_CTRL2);
	writel(CFG_DDR2_TAP_VAL, ddr_regs + QCA955X_DDR_REG_TAP_CTRL3);

	writel(0x633c8176, srif_regs + QCA955X_SRIF_PMU1_REG);
	/* Set DDR2 Voltage to 1.8 volts */
	writel(PMU2_PGM_MASK | FIELD_PREP(PMU2_LDO_TUNE_MASK, 0x3),
	       srif_regs + QCA955X_SRIF_PMU2_REG);
}
