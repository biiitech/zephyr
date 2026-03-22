/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_rcc

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/dt-bindings/clock/ch5xx-clocks.h>

#include <hal/wch/ch5xx/ch5xx.h>

#if DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch5xx_pll_clock)
#define CLK_SRC CH5XX_RCC_SRC_PLL
#elif DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch5xx_hse_clock)
#define CLK_SRC CH5XX_RCC_SRC_HSE
#elif DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch5xx_lsi_clock)
#define CLK_SRC CH5XX_RCC_SRC_LSI
#endif

struct wch_ch5xx_rcc_config {
	uint8_t divider;
	bool stop_on_halt;
};

static int clock_control_wch_ch5xx_rcc_on(const struct device *dev, clock_control_subsys_t sys)
{
	/* Nothing to do, clocks are default enabled */
	return 0;
}

static int clock_control_wch_ch5xx_rcc_get_rate(const struct device *dev,
						clock_control_subsys_t sys, uint32_t *rate)
{
	uint8_t clk_id = (uintptr_t)sys;
	switch (clk_id) {
	case CH5XX_CLOCK_DEBUG:
		*rate = DT_PROP(DT_NODELABEL(clk_hse), clock_frequency);
		break;
	case CH5XX_CLOCK_FLASH_HCLK:
		*rate = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
		break;
	case CH5XX_CLOCK_XROM:
		if (CLK_SRC == CH5XX_RCC_SRC_PLL) {
			/* PLL 600MHz / 2 for XROM when PLL is selected */
			*rate = DT_PROP(DT_NODELABEL(pll), clock_frequency) / 2;
		} else {
			/* 2x HSE (32MHz * 2 = 64MHz) for XROM otherwise */
			*rate = 2 * DT_PROP(DT_NODELABEL(clk_hse), clock_frequency);
		}
		break;
	case CH5XX_CLOCK_USB:
		*rate = (DT_PROP(DT_NODELABEL(pll), clock_frequency) / 25) * 2;
		break;
	case CH5XX_CLOCK_BLE:
	case CH5XX_CLOCK_TIMER:
	case CH5XX_CLOCK_SPI:
	case CH5XX_CLOCK_PWM:
	case CH5XX_CLOCK_USART1:
	case CH5XX_CLOCK_I2C:
	case CH5XX_CLOCK_CMP:
	case CH5XX_CLOCK_AESC:
		*rate = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
		break;
	}
	return 0;
}

static DEVICE_API(clock_control, clock_control_wch_ch5xx_rcc_api) = {
	.on = clock_control_wch_ch5xx_rcc_on,
	.get_rate = clock_control_wch_ch5xx_rcc_get_rate,
};

static int clock_control_wch_ch5xx_rcc_init(const struct device *dev)
{
	const struct wch_ch5xx_rcc_config *config = dev->config;

	/* enable and configure HSE if enabled */
	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH5XX_HSE_CLOCK_ENABLED)) {
		/* pull load capacitance and rated current from device tree and set tuning
		 * parameters */
		ch5xx_rcc_set_hse_tuning_param(DT_PROP(DT_NODELABEL(clk_hse), hse_load_capacitance),
					       DT_PROP(DT_NODELABEL(clk_hse), hse_rated_current));
	}

	/* configure clock control */
	ch5xx_rcc_set_lsi_enable(IS_ENABLED(CONFIG_DT_HAS_WCH_CH5XX_LSI_CLOCK_ENABLED));
	ch5xx_rcc_set_pll_enable(IS_ENABLED(CONFIG_DT_HAS_WCH_CH5XX_PLL_CLOCK_ENABLED));
	ch5xx_rcc_set_hse_enable(IS_ENABLED(CONFIG_DT_HAS_WCH_CH5XX_HSE_CLOCK_ENABLED));

	/* configure HSE behavior in stop mode */
	ch5xx_rcc_keep_hse_on_in_stop(!config->stop_on_halt);

	/* set system clock source and divider */
	ch5xx_rcc_set_pll_divider(config->divider);
	ch5xx_rcc_set_sys_clock_source(CLK_SRC);

	return 0;
}

#define CLOCK_CONTROL_WCH_CH5XX_RCC_INIT(idx)                                                      \
	static const struct wch_ch5xx_rcc_config clock_control_wch_ch5xx_rcc_##idx##_config = {    \
		.divider = DT_PROP(DT_DRV_INST(idx), divider),                                     \
		.stop_on_halt = DT_PROP(DT_DRV_INST(idx), stop_on_halt),                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, clock_control_wch_ch5xx_rcc_init, NULL, NULL,                   \
			      &clock_control_wch_ch5xx_rcc_##idx##_config, PRE_KERNEL_1,           \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,                                  \
			      &clock_control_wch_ch5xx_rcc_api);

/* There is only ever one RCC */
CLOCK_CONTROL_WCH_CH5XX_RCC_INIT(0)
