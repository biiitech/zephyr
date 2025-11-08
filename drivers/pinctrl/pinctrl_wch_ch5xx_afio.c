/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_afio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/ch5xx-pinctrl.h>
#include <hal_ch32fun.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	for (int i = 0; i < pin_cnt; i++, pins++) {
		uint32_t remap = (pins->config >> CH5XX_PINCTRL_AFIO_REMAP_SHIFT) & 0x0F;
		uint32_t bits = (pins->config >> CH5XX_PINCTRL_AFIO_BITS_SHIFT) & 0x0F;
		uint32_t pos = (pins->config >> CH5XX_PINCTRL_AFIO_POS_SHIFT) & 0x01F;
		uint32_t pin = (pins->config >> CH5XX_PINCTRL_PIN_SHIFT) & 0x01F;
		uint32_t port = (pins->config >> CH5XX_PINCTRL_PORT_SHIFT) & 0x01;

		bits = ((1 << bits) - 1) << pos;
		remap = remap << pos;

		// /* set alternate function value */
		AFIO->ALTERNATE &= ~bits;
		AFIO->ALTERNATE |= remap;

		/* configure in output mode with high drive */
		if ((pins->output_enable)) {
			GPIO->PORT[port].DIR |= (1 << pin);
			if (pins->high_drive) {
				/* configure in output mode with high drive */
				GPIO->PORT[port].PD_DRV |= (1 << pin);
			}
		} else {
			GPIO->PORT[port].DIR &= ~(1 << pin);
			if (pins->bias_pull_up) {
				GPIO->PORT[port].PU |= (1 << pin);
				GPIO->PORT[port].PD_DRV &= ~(1 << pin);
			} else if (pins->bias_pull_down) {
				GPIO->PORT[port].PD_DRV |= (1 << pin);
				GPIO->PORT[port].PU &= ~(1 << pin);
			}
		}
	}
	return 0;
}
