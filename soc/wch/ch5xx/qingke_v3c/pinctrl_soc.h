/*
 * Copyright (c) 2025 BIII Tech LLP.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PINCTRL_SOC_H__
#define __PINCTRL_SOC_H__

/**
 * @brief Type to hold a pin's pinctrl configuration.
 */
struct ch5xx_pinctrl_soc_pin {
	uint32_t config: 24;
	bool bias_pull_up: 1;
	bool bias_pull_down: 1;
	bool high_drive: 1;
	bool output_enable: 1;
};

typedef struct ch5xx_pinctrl_soc_pin pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.config = DT_PROP_BY_IDX(node_id, prop, idx),                                      \
		.bias_pull_up = DT_PROP(node_id, bias_pull_up),                                    \
		.bias_pull_down = DT_PROP(node_id, bias_pull_down),                                \
		.output_enable = DT_PROP(node_id, output_enable),                                  \
		.high_drive = DT_PROP(node_id, high_drive),                                        \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,           \
				Z_PINCTRL_STATE_PIN_INIT)}

#endif
