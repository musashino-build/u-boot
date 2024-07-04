// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 INAGAKI Hiroshi <musashino.open@gmail.com>
 *
 * based on QCA956x support and QSDK
 */

#include <clock_legacy.h>
#include <log.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/addrspace.h>
#include <asm/types.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <mach/ar71xx_regs.h>
#include <mach/ath79.h>
#include <wait_bit.h>

#define QCA955X_PLL_CPU_CONFIG_OUTDIV_FMASK		\
		QCA955X_PLL_CPU_CONFIG_OUTDIV_MASK	\
		<< QCA955X_PLL_CPU_CONFIG_OUTDIV_SHIFT
#define QCA955X_PLL_CPU_CONFIG_RANGE_FMASK		\
		QCA955X_PLL_CPU_CONFIG_RANGE_MASK	\
		<< QCA955X_PLL_CPU_CONFIG_RANGE_SHIFT
#define QCA955X_PLL_CPU_CONFIG_REFDIV_FMASK		\
		QCA955X_PLL_CPU_CONFIG_REFDIV_MASK	\
		<< QCA955X_PLL_CPU_CONFIG_REFDIV_SHIFT
#define QCA955X_PLL_CPU_CONFIG_NINT_FMASK		\
		QCA955X_PLL_CPU_CONFIG_NINT_MASK	\
		<< QCA955X_PLL_CPU_CONFIG_NINT_SHIFT
#define QCA955X_PLL_CPU_CONFIG_NFRAC_FMASK		\
		QCA955X_PLL_CPU_CONFIG_NFRAC_MASK	\
		<< QCA955X_PLL_CPU_CONFIG_NFRAC_SHIFT

#define QCA955X_PLL_DDR_CONFIG_OUTDIV_FMASK		\
		QCA955X_PLL_DDR_CONFIG_OUTDIV_MASK	\
		<< QCA955X_PLL_DDR_CONFIG_OUTDIV_SHIFT
#define QCA955X_PLL_DDR_CONFIG_RANGE_FMASK		\
		QCA955X_PLL_DDR_CONFIG_RANGE_MASK	\
		<< QCA955X_PLL_DDR_CONFIG_RANGE_SHIFT
#define QCA955X_PLL_DDR_CONFIG_REFDIV_FMASK		\
		QCA955X_PLL_DDR_CONFIG_REFDIV_MASK	\
		<< QCA955X_PLL_DDR_CONFIG_REFDIV_SHIFT
#define QCA955X_PLL_DDR_CONFIG_NINT_FMASK		\
		QCA955X_PLL_DDR_CONFIG_NINT_MASK	\
		<< QCA955X_PLL_DDR_CONFIG_NINT_SHIFT
#define QCA955X_PLL_DDR_CONFIG_NFRAC_FMASK		\
		QCA955X_PLL_DDR_CONFIG_NFRAC_MASK	\
		<< QCA955X_PLL_DDR_CONFIG_NFRAC_SHIFT

#define QCA955X_PLL_CLK_CTRL_AHB_POST_DIV_FMASK		\
		QCA955X_PLL_CLK_CTRL_AHB_POST_DIV_MASK	\
		<< QCA955X_PLL_CLK_CTRL_AHB_POST_DIV_SHIFT
#define QCA955X_PLL_CLK_CTRL_DDR_POST_DIV_FMASK		\
		QCA955X_PLL_CLK_CTRL_DDR_POST_DIV_MASK	\
		<< QCA955X_PLL_CLK_CTRL_DDR_POST_DIV_SHIFT
#define QCA955X_PLL_CLK_CTRL_CPU_POST_DIV_FMASK		\
		QCA955X_PLL_CLK_CTRL_CPU_POST_DIV_MASK	\
		<< QCA955X_PLL_CLK_CTRL_CPU_POST_DIV_SHIFT

#define QCA955X_PLL_DDR_DIT_UPD_CNT_FMASK		\
		QCA955X_PLL_DDR_DIT_UPD_CNT_MASK	\
		<< QCA955X_PLL_DDR_DIT_UPD_CNT_SHIFT
#define QCA955X_PLL_DDR_DIT_NFRAC_STEP_FMASK		\
		QCA955X_PLL_DDR_DIT_NFRAC_STEP_MASK	\
		<< QCA955X_PLL_DDR_DIT_NFRAC_STEP_SHIFT
#define QCA955X_PLL_DDR_DIT_NFRAC_MIN_FMASK		\
		QCA955X_PLL_DDR_DIT_NFRAC_MIN_MASK	\
		<< QCA955X_PLL_DDR_DIT_NFRAC_MIN_SHIFT
#define QCA955X_PLL_DDR_DIT_NFRAC_MAX_FMASK		\
		QCA955X_PLL_DDR_DIT_NFRAC_MAX_MASK	\
		<< QCA955X_PLL_DDR_DIT_NFRAC_MAX_SHIFT

#define QCA955X_PLL_CPU_DIT_UPD_CNT_FMASK		\
		QCA955X_PLL_CPU_DIT_UPD_CNT_MASK	\
		<< QCA955X_PLL_CPU_DIT_UPD_CNT_SHIFT
#define QCA955X_PLL_CPU_DIT_NFRAC_STEP_FMASK		\
		QCA955X_PLL_CPU_DIT_NFRAC_STEP_MASK	\
		<< QCA955X_PLL_CPU_DIT_NFRAC_STEP_SHIFT
#define QCA955X_PLL_CPU_DIT_NFRAC_MIN_FMASK		\
		QCA955X_PLL_CPU_DIT_NFRAC_MIN_MASK	\
		<< QCA955X_PLL_CPU_DIT_NFRAC_MIN_SHIFT
#define QCA955X_PLL_CPU_DIT_NFRAC_MAX_FMASK		\
		QCA955X_PLL_CPU_DIT_NFRAC_MAX_MASK	\
		<< QCA955X_PLL_CPU_DIT_NFRAC_MAX_SHIFT

#define PLL_SRIF_DPLL2_KI_MASK				GENMASK(29, 26)
#define PLL_SRIF_DPLL2_KD_MASK				GENMASK(25, 19)
#define PLL_SRIF_DPLL2_PLL_PWD_MASK			BIT(16)
#define PLL_SRIF_DPLL2_DELTA_MASK			GENMASK(12, 7)

#define PLL_SRIF_DPLL2_DEFAULT		\
		FIELD_PREP(PLL_SRIF_DPLL2_KI_MASK, 0x4) | \
		FIELD_PREP(PLL_SRIF_DPLL2_KD_MASK, 0x60) | \
		FIELD_PREP(PLL_SRIF_DPLL2_PLL_PWD_MASK, 0x1) | \
		FIELD_PREP(PLL_SRIF_DPLL2_DELTA_MASK, 0x1e)

DECLARE_GLOBAL_DATA_PTR;

static u32 qca955x_get_xtal(void)
{
	u32 val;

	val = ath79_get_bootstrap();
	if (val & QCA955X_BOOTSTRAP_REF_CLK_40)
		return 40000000;
	else
		return 25000000;
}

int get_serial_clock(void)
{
	return qca955x_get_xtal();
}

void qca955x_pll_init(void)
{
	void __iomem *srif_regs = map_physmem(QCA955X_SRIF_BASE,
					      QCA955X_SRIF_SIZE, MAP_NOCACHE);
	void __iomem *pll_regs = map_physmem(AR71XX_PLL_BASE,
					     AR71XX_PLL_SIZE, MAP_NOCACHE);

	/* 10.33.2 Baseband DPLL2 */
	writel(PLL_SRIF_DPLL2_DEFAULT,
	       srif_regs + QCA955X_SRIF_BB_DPLL2_REG);

	/* 10.33.2 PCIE DPLL2 */
	writel(PLL_SRIF_DPLL2_DEFAULT,
	       srif_regs + QCA955X_SRIF_PCIE_DPLL2_REG);

	/* 10.33.2 DDR DPLL2 */
	writel(PLL_SRIF_DPLL2_DEFAULT,
	       srif_regs + QCA955X_SRIF_DDR_DPLL2_REG);

	/* 10.33.2 CPU DPLL2 */
	writel(PLL_SRIF_DPLL2_DEFAULT,
	       srif_regs + QCA955X_SRIF_CPU_DPLL2_REG);

	/* pll_bypass_set */
	setbits_be32(pll_regs + QCA955X_PLL_CLK_CTRL_REG,
		     QCA955X_PLL_CLK_CTRL_CPU_PLL_BYPASS);
	setbits_be32(pll_regs + QCA955X_PLL_CLK_CTRL_REG,
		     QCA955X_PLL_CLK_CTRL_DDR_PLL_BYPASS);
	setbits_be32(pll_regs + QCA955X_PLL_CLK_CTRL_REG,
		     QCA955X_PLL_CLK_CTRL_AHB_PLL_BYPASS);

	/* init_cpu_pll (720_600_240) */
	writel(QCA955X_PLL_CPU_CONFIG_PLLPWD |
	       FIELD_PREP(QCA955X_PLL_CPU_CONFIG_OUTDIV_FMASK, 0x0) |
	       FIELD_PREP(QCA955X_PLL_CPU_CONFIG_RANGE_FMASK, 0x1) |
	       FIELD_PREP(QCA955X_PLL_CPU_CONFIG_REFDIV_FMASK, 0x1) |
	       FIELD_PREP(QCA955X_PLL_CPU_CONFIG_NINT_FMASK, 0x12),
	       pll_regs + QCA955X_PLL_CPU_CONFIG_REG);

	/* init_ddr_pll (720_600_240) */
	writel(QCA955X_PLL_DDR_CONFIG_PLLPWD |
	       FIELD_PREP(QCA955X_PLL_DDR_CONFIG_OUTDIV_FMASK, 0x0) |
	       FIELD_PREP(QCA955X_PLL_DDR_CONFIG_RANGE_FMASK, 0x1) |
	       FIELD_PREP(QCA955X_PLL_DDR_CONFIG_REFDIV_FMASK, 0x1) |
	       FIELD_PREP(QCA955X_PLL_DDR_CONFIG_NINT_FMASK, 0xf),
	       pll_regs + QCA955X_PLL_DDR_CONFIG_REG);

	/* init_ahb_pll (720_600_240) */
	writel(/* use CPU PLL for AHB (0) */
	       /* use DDR PLL for DDR (0) */
	       /* use CPU PLL for CPU (0) */
	       FIELD_PREP(QCA955X_PLL_CLK_CTRL_AHB_POST_DIV_FMASK, 0x2) |
	       FIELD_PREP(QCA955X_PLL_CLK_CTRL_DDR_POST_DIV_FMASK, 0x0) |
	       FIELD_PREP(QCA955X_PLL_CLK_CTRL_CPU_POST_DIV_FMASK, 0x0) |
	       QCA955X_PLL_CLK_CTRL_AHB_PLL_BYPASS |
	       QCA955X_PLL_CLK_CTRL_DDR_PLL_BYPASS |
	       QCA955X_PLL_CLK_CTRL_CPU_PLL_BYPASS,
	       pll_regs + QCA955X_PLL_CLK_CTRL_REG);

	/* pll_pwd_unset */
	clrbits_be32(pll_regs + QCA955X_PLL_CPU_CONFIG_REG,
		     QCA955X_PLL_CPU_CONFIG_PLLPWD);
	clrbits_be32(pll_regs + QCA955X_PLL_DDR_CONFIG_REG,
		     QCA955X_PLL_DDR_CONFIG_PLLPWD);

	/* outdiv_unset */
	clrbits_be32(pll_regs + QCA955X_PLL_CPU_CONFIG_REG,
		     QCA955X_PLL_CPU_CONFIG_OUTDIV_FMASK);
	clrbits_be32(pll_regs + QCA955X_PLL_DDR_CONFIG_REG,
		     QCA955X_PLL_DDR_CONFIG_OUTDIV_FMASK);

	/* pll_bypass_unset */
	clrbits_be32(pll_regs + QCA955X_PLL_CLK_CTRL_REG,
		     QCA955X_PLL_CLK_CTRL_CPU_PLL_BYPASS);
	clrbits_be32(pll_regs + QCA955X_PLL_CLK_CTRL_REG,
		     QCA955X_PLL_CLK_CTRL_DDR_PLL_BYPASS);
	clrbits_be32(pll_regs + QCA955X_PLL_CLK_CTRL_REG,
		     QCA955X_PLL_CLK_CTRL_AHB_PLL_BYPASS);

	/* ddr_pll_dither_unset */
	writel(FIELD_PREP(QCA955X_PLL_DDR_DIT_UPD_CNT_FMASK, 0xf) |
	       FIELD_PREP(QCA955X_PLL_DDR_DIT_NFRAC_STEP_FMASK, 0x1) |
	       FIELD_PREP(QCA955X_PLL_DDR_DIT_NFRAC_MIN_FMASK, 0x0) |
	       FIELD_PREP(QCA955X_PLL_DDR_DIT_NFRAC_MAX_FMASK, 0x3ff),
	       pll_regs + QCA955X_PLL_DDR_DIT_FRAC_REG);

	/* cpu_pll_dither_unset */
	writel(FIELD_PREP(QCA955X_PLL_CPU_DIT_UPD_CNT_FMASK, 0xf) |
	       FIELD_PREP(QCA955X_PLL_CPU_DIT_NFRAC_STEP_FMASK, 0x1) |
	       FIELD_PREP(QCA955X_PLL_CPU_DIT_NFRAC_MIN_FMASK, 0x0) |
	       FIELD_PREP(QCA955X_PLL_CPU_DIT_NFRAC_MAX_FMASK, 0x3f),
	       pll_regs + QCA955X_PLL_CPU_DIT_FRAC_REG);
}

int get_clocks(void)
{
	void __iomem *regs;
	u32 pll, cpu_pll, ddr_pll, clk_ctrl;
	u32 cpu_clk, ddr_clk, ahb_clk;
	u32 outdiv, refdiv, nint, nfrac, postdiv;
	u32 xtal = qca955x_get_xtal();

	regs = map_physmem(AR71XX_PLL_BASE, AR71XX_PLL_SIZE,
			   MAP_NOCACHE);
	pll = readl(regs + QCA955X_PLL_CPU_CONFIG_REG);
	outdiv = FIELD_GET(QCA955X_PLL_CPU_CONFIG_OUTDIV_FMASK, pll);
	refdiv = FIELD_GET(QCA955X_PLL_CPU_CONFIG_REFDIV_FMASK, pll);
	nint   = FIELD_GET(QCA955X_PLL_CPU_CONFIG_NINT_FMASK, pll);
	nfrac  = FIELD_GET(QCA955X_PLL_CPU_CONFIG_NFRAC_FMASK, pll);

	cpu_pll = (xtal * (nint + (nfrac >> 9))) / (refdiv * (1 << outdiv));

	pll = readl(regs + QCA955X_PLL_DDR_CONFIG_REG);
	outdiv = FIELD_GET(QCA955X_PLL_DDR_CONFIG_OUTDIV_FMASK, pll);
	refdiv = FIELD_GET(QCA955X_PLL_DDR_CONFIG_REFDIV_FMASK, pll);
	nint   = FIELD_GET(QCA955X_PLL_DDR_CONFIG_NINT_FMASK, pll);
	nfrac  = FIELD_GET(QCA955X_PLL_DDR_CONFIG_NFRAC_FMASK, pll);

	ddr_pll = (xtal * (nint + (nfrac >> 9))) / (refdiv * (1 << outdiv));

	clk_ctrl = readl(regs + QCA955X_PLL_CLK_CTRL_REG);

	postdiv = FIELD_GET(QCA955X_PLL_CLK_CTRL_CPU_POST_DIV_FMASK, clk_ctrl);
	if (clk_ctrl & QCA955X_PLL_CLK_CTRL_CPU_PLL_BYPASS)
		cpu_clk = xtal;
	else if (clk_ctrl & QCA955X_PLL_CLK_CTRL_CPUCLK_FROM_CPUPLL)
		cpu_clk = ddr_pll / (postdiv + 1);
	else
		cpu_clk = cpu_pll / (postdiv + 1);

	postdiv = FIELD_GET(QCA955X_PLL_CLK_CTRL_DDR_POST_DIV_FMASK, clk_ctrl);
	if (clk_ctrl & QCA955X_PLL_CLK_CTRL_DDR_PLL_BYPASS)
		ddr_clk = xtal;
	else if (clk_ctrl & QCA955X_PLL_CLK_CTRL_DDRCLK_FROM_DDRPLL)
		ddr_clk = cpu_pll / (postdiv + 1);
	else
		ddr_clk = ddr_pll / (postdiv + 1);

	postdiv = FIELD_GET(QCA955X_PLL_CLK_CTRL_AHB_POST_DIV_FMASK, clk_ctrl);
	if (clk_ctrl & QCA955X_PLL_CLK_CTRL_AHB_PLL_BYPASS)
		ahb_clk = xtal;
	else if (clk_ctrl & QCA955X_PLL_CLK_CTRL_AHBCLK_FROM_DDRPLL)
		ahb_clk = ddr_pll;
	else
		ahb_clk = cpu_pll;
	ahb_clk /= (postdiv + 1);

	gd->cpu_clk = cpu_clk;
	gd->mem_clk = ddr_clk;
	gd->bus_clk = ahb_clk;

	debug("cpu_clk=%u, ddr_clk=%u, bus_clk=%u\n",
	      cpu_clk, ddr_clk, ahb_clk);

	return 0;
}

ulong get_bus_freq(ulong dummy)
{
	if (!gd->bus_clk)
		get_clocks();
	return gd->bus_clk;
}

ulong get_ddr_freq(ulong dummy)
{
	if (!gd->mem_clk)
		get_clocks();
	return gd->mem_clk;
}
