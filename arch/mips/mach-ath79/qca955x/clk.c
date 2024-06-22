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

#define CPU_PLL_CONFIG_PLLPWD_MASK			BIT(30)
#define CPU_PLL_CONFIG_OUTDIV_MASK			GENMASK(21, 19)
#define CPU_PLL_CONFIG_RANGE_MASK			GENMASK(18, 17)
#define CPU_PLL_CONFIG_REFDIV_MASK			GENMASK(16, 12)
#define CPU_PLL_CONFIG_NINT_MASK			GENMASK(11, 6)
#define CPU_PLL_CONFIG_NFRAC_MASK			GENMASK(5, 0)

#define DDR_PLL_CONFIG_PLLPWD_MASK			BIT(30)
#define DDR_PLL_CONFIG_OUTDIV_MASK			GENMASK(25, 23)
#define DDR_PLL_CONFIG_RANGE_MASK			GENMASK(22, 21)
#define DDR_PLL_CONFIG_REFDIV_MASK			GENMASK(20, 16)
#define DDR_PLL_CONFIG_NINT_MASK			GENMASK(15, 10)
#define DDR_PLL_CONFIG_NFRAC_MASK			GENMASK(9, 0)

#define CPU_DDR_CLOCK_CONTROL_AHBCLK_FROM_DDRPLL_MASK	BIT(24)
#define CPU_DDR_CLOCK_CONTROL_DDRCLK_FROM_DDRPLL_MASK	BIT(21)
#define CPU_DDR_CLOCK_CONTROL_CPUCLK_FROM_CPUPLL_MASK	BIT(20)
#define CPU_DDR_CLOCK_CONTROL_AHB_POST_DIV_MASK		GENMASK(19, 15)
#define CPU_DDR_CLOCK_CONTROL_DDR_POST_DIV_MASK		GENMASK(14, 10)
#define CPU_DDR_CLOCK_CONTROL_CPU_POST_DIV_MASK		GENMASK(9, 5)
#define CPU_DDR_CLOCK_CONTROL_AHB_PLL_BYPASS_MASK	BIT(4)
#define CPU_DDR_CLOCK_CONTROL_DDR_PLL_BYPASS_MASK	BIT(3)
#define CPU_DDR_CLOCK_CONTROL_CPU_PLL_BYPASS_MASK	BIT(2)

#define DDR_DIT_FRAC_DITHER_EN_MASK			BIT(31)
#define DDR_DIT_FRAC_UPD_CNT_MASK			GENMASK(30, 27)
#define DDR_DIT_FRAC_NFRAC_STEP_MASK			GENMASK(26, 20)
#define DDR_DIT_FRAC_NFRAC_MIN_MASK			GENMASK(19, 10)
#define DDR_DIT_FRAC_NFRAC_MAX_MASK			GENMASK(9, 0)

#define CPU_DIT_FRAC_DITHER_EN_MASK			BIT(31)
#define CPU_DIT_FRAC_UPD_CNT_MASK			GENMASK(23, 18)
#define CPU_DIT_FRAC_NFRAC_STEP_MASK			GENMASK(17, 12)
#define CPU_DIT_FRAC_NFRAC_MIN_MASK			GENMASK(11, 6)
#define CPU_DIT_FRAC_NFRAC_MAX_MASK			GENMASK(5, 0)

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
	u32 val;

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
	val = readl(pll_regs + QCA955X_PLL_CLK_CTRL_REG);
	val |= CPU_DDR_CLOCK_CONTROL_CPU_PLL_BYPASS_MASK;
	writel(val, pll_regs + QCA955X_PLL_CLK_CTRL_REG);
	val |= CPU_DDR_CLOCK_CONTROL_DDR_PLL_BYPASS_MASK;
	writel(val, pll_regs + QCA955X_PLL_CLK_CTRL_REG);
	val |= CPU_DDR_CLOCK_CONTROL_AHB_PLL_BYPASS_MASK;
	writel(val, pll_regs + QCA955X_PLL_CLK_CTRL_REG);

	/* init_cpu_pll (720_600_240) */
	val = readl(pll_regs + QCA955X_PLL_CPU_CONFIG_REG);
	val &= ~(CPU_PLL_CONFIG_OUTDIV_MASK | CPU_PLL_CONFIG_RANGE_MASK |
		 CPU_PLL_CONFIG_REFDIV_MASK | CPU_PLL_CONFIG_NINT_MASK);
	val |= CPU_PLL_CONFIG_PLLPWD_MASK |
	       FIELD_PREP(CPU_PLL_CONFIG_OUTDIV_MASK, 0x0) |
	       FIELD_PREP(CPU_PLL_CONFIG_RANGE_MASK, 0x1) |
	       FIELD_PREP(CPU_PLL_CONFIG_REFDIV_MASK, 0x1) |
	       FIELD_PREP(CPU_PLL_CONFIG_NINT_MASK, 0x12);
	writel(val, pll_regs + QCA955X_PLL_CPU_CONFIG_REG);

	/* init_ddr_pll (720_600_240) */
	val = readl(pll_regs + QCA955X_PLL_DDR_CONFIG_REG);
	val &= ~(DDR_PLL_CONFIG_OUTDIV_MASK | DDR_PLL_CONFIG_RANGE_MASK |
		 DDR_PLL_CONFIG_REFDIV_MASK | DDR_PLL_CONFIG_NINT_MASK);
	val |= DDR_PLL_CONFIG_PLLPWD_MASK |
	       FIELD_PREP(DDR_PLL_CONFIG_OUTDIV_MASK, 0x0) |
	       FIELD_PREP(DDR_PLL_CONFIG_RANGE_MASK, 0x1) |
	       FIELD_PREP(DDR_PLL_CONFIG_REFDIV_MASK, 0x1) |
	       FIELD_PREP(DDR_PLL_CONFIG_NINT_MASK, 0xf);
	writel(val, pll_regs + QCA955X_PLL_DDR_CONFIG_REG);

	/* init_ahb_pll (720_600_240) */
	writel(/* use CPU PLL for AHB (0) */
	       /* use DDR PLL for DDR (0) */
	       /* use CPU PLL for CPU (0) */
	       FIELD_PREP(CPU_DDR_CLOCK_CONTROL_AHB_POST_DIV_MASK, 0x2) |
	       FIELD_PREP(CPU_DDR_CLOCK_CONTROL_DDR_POST_DIV_MASK, 0x0) |
	       FIELD_PREP(CPU_DDR_CLOCK_CONTROL_CPU_POST_DIV_MASK, 0x0) |
	       CPU_DDR_CLOCK_CONTROL_AHB_PLL_BYPASS_MASK |
	       CPU_DDR_CLOCK_CONTROL_DDR_PLL_BYPASS_MASK |
	       CPU_DDR_CLOCK_CONTROL_CPU_PLL_BYPASS_MASK,
	       pll_regs + QCA955X_PLL_CLK_CTRL_REG);

	/* pll_pwd_unset */
	val = readl(pll_regs + QCA955X_PLL_CPU_CONFIG_REG);
	writel(val & ~CPU_PLL_CONFIG_PLLPWD_MASK,
	       pll_regs + QCA955X_PLL_CPU_CONFIG_REG);
	val = readl(pll_regs + QCA955X_PLL_DDR_CONFIG_REG);
	writel(val & ~DDR_PLL_CONFIG_PLLPWD_MASK,
	       pll_regs + QCA955X_PLL_DDR_CONFIG_REG);

	/* outdiv_unset */
	val = readl(pll_regs + QCA955X_PLL_CPU_CONFIG_REG);
	writel(val & ~CPU_PLL_CONFIG_OUTDIV_MASK,
	       pll_regs + QCA955X_PLL_CPU_CONFIG_REG);
	val = readl(pll_regs + QCA955X_PLL_DDR_CONFIG_REG);
	writel(val & ~DDR_PLL_CONFIG_OUTDIV_MASK,
	       pll_regs + QCA955X_PLL_DDR_CONFIG_REG);

	/* pll_bypass_unset */
	val = readl(pll_regs + QCA955X_PLL_CLK_CTRL_REG);
	val &= ~CPU_DDR_CLOCK_CONTROL_CPU_PLL_BYPASS_MASK;
	writel(val, pll_regs + QCA955X_PLL_CLK_CTRL_REG);
	val &= ~CPU_DDR_CLOCK_CONTROL_DDR_PLL_BYPASS_MASK;
	writel(val, pll_regs + QCA955X_PLL_CLK_CTRL_REG);
	val &= ~CPU_DDR_CLOCK_CONTROL_AHB_PLL_BYPASS_MASK;
	writel(val, pll_regs + QCA955X_PLL_CLK_CTRL_REG);

	/* ddr_pll_dither_unset */
	writel(FIELD_PREP(DDR_DIT_FRAC_UPD_CNT_MASK, 0xf) |
	       FIELD_PREP(DDR_DIT_FRAC_NFRAC_STEP_MASK, 0x1) |
	       FIELD_PREP(DDR_DIT_FRAC_NFRAC_MIN_MASK, 0x0) |
	       FIELD_PREP(DDR_DIT_FRAC_NFRAC_MAX_MASK, 0x3ff),
	       pll_regs + QCA955X_PLL_DDR_DIT_FRAC_REG);

	/* cpu_pll_dither_unset */
	writel(FIELD_PREP(CPU_DIT_FRAC_UPD_CNT_MASK, 0xf) |
	       FIELD_PREP(CPU_DIT_FRAC_NFRAC_STEP_MASK, 0x1) |
	       FIELD_PREP(CPU_DIT_FRAC_NFRAC_MIN_MASK, 0x0) |
	       FIELD_PREP(CPU_DIT_FRAC_NFRAC_MAX_MASK, 0x3f),
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
	outdiv = FIELD_GET(CPU_PLL_CONFIG_OUTDIV_MASK, pll);
	refdiv = FIELD_GET(CPU_PLL_CONFIG_REFDIV_MASK, pll);
	nint    = FIELD_GET(CPU_PLL_CONFIG_NINT_MASK, pll);
	nfrac   = FIELD_GET(CPU_PLL_CONFIG_NFRAC_MASK, pll);

	cpu_pll = (xtal * (nint + (nfrac >> 9))) / (refdiv * (1 << outdiv));

	pll = readl(regs + QCA955X_PLL_DDR_CONFIG_REG);
	outdiv = FIELD_GET(DDR_PLL_CONFIG_OUTDIV_MASK, pll);
	refdiv = FIELD_GET(DDR_PLL_CONFIG_REFDIV_MASK, pll);
	nint    = FIELD_GET(DDR_PLL_CONFIG_NINT_MASK, pll);
	nfrac   = FIELD_GET(DDR_PLL_CONFIG_NFRAC_MASK, pll);

	ddr_pll = (xtal * (nint + (nfrac >> 9))) / (refdiv * (1 << outdiv));

	clk_ctrl = readl(regs + QCA955X_PLL_CLK_CTRL_REG);

	postdiv = FIELD_GET(CPU_DDR_CLOCK_CONTROL_CPU_POST_DIV_MASK, clk_ctrl);
	if (clk_ctrl & CPU_DDR_CLOCK_CONTROL_CPU_PLL_BYPASS_MASK)
		cpu_clk = xtal;
	else if (clk_ctrl & CPU_DDR_CLOCK_CONTROL_CPUCLK_FROM_CPUPLL_MASK)
		cpu_clk = ddr_pll / (postdiv + 1);
	else
		cpu_clk = cpu_pll / (postdiv + 1);

	postdiv = FIELD_GET(CPU_DDR_CLOCK_CONTROL_DDR_POST_DIV_MASK, clk_ctrl);
	if (clk_ctrl & CPU_DDR_CLOCK_CONTROL_DDR_PLL_BYPASS_MASK)
		ddr_clk = xtal;
	else if (clk_ctrl & CPU_DDR_CLOCK_CONTROL_DDRCLK_FROM_DDRPLL_MASK)
		ddr_clk = cpu_pll / (postdiv + 1);
	else
		ddr_clk = ddr_pll / (postdiv + 1);

	postdiv = FIELD_GET(CPU_DDR_CLOCK_CONTROL_AHB_POST_DIV_MASK, clk_ctrl);
	if (clk_ctrl & CPU_DDR_CLOCK_CONTROL_AHB_PLL_BYPASS_MASK)
		ahb_clk = xtal;
	else if (clk_ctrl & CPU_DDR_CLOCK_CONTROL_AHBCLK_FROM_DDRPLL_MASK)
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
