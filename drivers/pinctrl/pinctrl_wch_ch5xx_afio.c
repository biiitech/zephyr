/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_afio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/ch5xx-pinctrl.h>
#include <hal/wch/ch5xx/ch5xx.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	for (int i = 0; i < pin_cnt; i++, pins++) {
		uint32_t remap = (pins->config >> CH5XX_PINCTRL_AFIO_REMAP_SHIFT) & 0x0F;
		uint32_t bits = (pins->config >> CH5XX_PINCTRL_AFIO_BITS_SHIFT) & 0x0F;
		uint32_t pos = (pins->config >> CH5XX_PINCTRL_AFIO_POS_SHIFT) & 0x01F;
		uint32_t pin = (pins->config >> CH5XX_PINCTRL_PIN_SHIFT) & 0x01F;
		uint32_t port = (pins->config >> CH5XX_PINCTRL_PORT_SHIFT) & 0x01;

		/* set alternate function value */
		ch5xx_afio_remap(remap, ((1 << bits) - 1), pos);

		/* configure in output mode with high drive */
		if ((pins->output_enable)) {
			ch5xx_gpio_cfg_out(&GPIO->PORT[port], pin);
			ch5xx_gpio_set_high_drive(&GPIO->PORT[port], pin, pins->high_drive);
		} else {
			ch5xx_gpio_cfg_in(&GPIO->PORT[port], pin);
			ch5xx_gpio_cfg_bias(&GPIO->PORT[port], pin, pins->bias_pull_up,
					    pins->bias_pull_down);
		}
	}
	return 0;
}
