// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) INAGAKI Hiroshi <musashino.open@gmail.com>
 */

#include <init.h>
#include <asm/io.h>
#include <asm/addrspace.h>
#include <asm/types.h>
#include <mach/ath79.h>
#include <mach/ar71xx_regs.h>
#include <mach/ddr.h>
#include <debug_uart.h>

static void aterm_pinmux_config(void)
{
	void __iomem *regs = map_physmem(AR71XX_GPIO_BASE,
					 AR71XX_GPIO_SIZE, MAP_NOCACHE);

	/* Disable JTAG */
	writel(0x2, regs + QCA955X_GPIO_REG_FUNC);

	/* Configure default GPIO OE/SET regs */
	writel(0xa6031f, regs + AR71XX_GPIO_REG_OE);
	writel(0x402800, regs + AR71XX_GPIO_REG_SET);

	/* Configure pin multiplexing */
	writel(0x00000000, regs + QCA955X_GPIO_REG_OUT_FUNC0);
	writel(0x0c080900, regs + QCA955X_GPIO_REG_OUT_FUNC1);
	writel(0x00160000, regs + QCA955X_GPIO_REG_OUT_FUNC2);
	writel(0x00000000, regs + QCA955X_GPIO_REG_OUT_FUNC3);
	writel(0x00000000, regs + QCA955X_GPIO_REG_OUT_FUNC4);
	writel(0x00000000, regs + QCA955X_GPIO_REG_OUT_FUNC5);
	writel(0x00000908, regs + QCA955X_GPIO_REG_IN_ENABLE0);
}

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
void board_debug_uart_init(void)
{
	aterm_pinmux_config();
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
#ifndef CONFIG_DEBUG_UART_BOARD_INIT
	aterm_pinmux_config();
#endif

#if !CONFIG_IS_ENABLED(SKIP_LOWLEVEL_INIT)
	qca955x_pll_init();
	qca955x_ddr_init();
#endif

	return 0;
}
#endif
