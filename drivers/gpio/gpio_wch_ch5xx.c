/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/irq.h>
#include <hal/wch/ch5xx/ch5xx.h>

#define DT_DRV_COMPAT wch_ch5xx_gpio

struct gpio_wch_ch5xx_config {
	struct gpio_driver_config common;
	GPIO_PORT_TypeDef *const port;
#ifdef CONFIG_GPIO_WCH_CH5XX_GPIO_INTERRUPTS
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct gpio_wch_ch5xx_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

static int gpio_wch_ch5xx_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	if ((flags & GPIO_OUTPUT) != 0) {
		/* configure as output */
		ch5xx_gpio_cfg_out(config->port, pin);
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			ch5xx_gpio_set(config->port, pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			ch5xx_gpio_clear(config->port, pin);
		}
	} else if ((flags & GPIO_INPUT) != 0) {
		/* configure as input */
		ch5xx_gpio_cfg_in(config->port, pin);
		if ((flags & GPIO_PULL_UP) != 0) {
			ch5xx_gpio_cfg_bias(config->port, pin, true, false);
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			ch5xx_gpio_cfg_bias(config->port, pin, false, true);
		} else {
			ch5xx_gpio_cfg_bias(config->port, pin, false, false);
		}
	} else {
		ch5xx_gpio_cfg_in(config->port, pin);
		ch5xx_gpio_cfg_bias(config->port, pin, false, false);
	}

	return 0;
}

static int gpio_wch_ch5xx_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	*value = ch5xx_gpio_read(config->port);

	return 0;
}

static int gpio_wch_ch5xx_port_set_masked_raw(const struct device *dev, uint32_t mask,
					      uint32_t value)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	ch5xx_gpio_write_masked(config->port, mask, value);

	return 0;
}

static int gpio_wch_ch5xx_port_set_bits_raw(const struct device *dev, uint32_t pins)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	ch5xx_gpio_set_masked(config->port, pins);

	return 0;
}

static int gpio_wch_ch5xx_port_clear_bits_raw(const struct device *dev, uint32_t pins)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	ch5xx_gpio_clear_masked(config->port, pins);

	return 0;
}

static int gpio_wch_ch5xx_port_toggle_bits(const struct device *dev, uint32_t pins)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	ch5xx_gpio_toggle_masked(config->port, pins);

	return 0;
}

#if defined(CONFIG_GPIO_WCH_CH5XX_GPIO_INTERRUPTS)

static void gpio_wch_ch5xx_isr(const struct device *dev)
{
	const struct gpio_wch_ch5xx_config *const config = dev->config;
	struct gpio_wch_ch5xx_data *const data = dev->data;
	uint32_t line = ch5xx_gpio_read_interrupt_flag(config->port);

	gpio_fire_callbacks(&data->callbacks, dev, line);
}

static int gpio_wch_ch5xx_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
						  enum gpio_int_mode mode,
						  enum gpio_int_trig trigger)
{
	const struct gpio_wch_ch5xx_config *config = dev->config;

	switch (mode) {
	case GPIO_INT_MODE_DISABLED:
		ch5xx_gpio_set_irq_enable(config->port, pin, false);
		break;
	case GPIO_INT_MODE_LEVEL:
		switch (trigger) {
		case GPIO_INT_TRIG_LOW:
			ch5xx_gpio_clear(config->port, pin);
			break;
		case GPIO_INT_TRIG_HIGH:
			ch5xx_gpio_set(config->port, pin);
			break;
		default:
			return -ENOTSUP;
		}
		ch5xx_gpio_set_irq_on_level(config->port, pin, true);
		ch5xx_gpio_set_irq_enable(config->port, pin, true);
		break;
	case GPIO_INT_MODE_EDGE:
		switch (trigger) {
		case GPIO_INT_TRIG_LOW:
			ch5xx_gpio_clear(config->port, pin);
			ch5xx_gpio_set_irq_edge_follow(config->port, pin, false);
			break;
		case GPIO_INT_TRIG_HIGH:
			ch5xx_gpio_set(config->port, pin);
			ch5xx_gpio_set_irq_edge_follow(config->port, pin, false);
			break;
		case GPIO_INT_TRIG_BOTH:
			ch5xx_gpio_set_irq_edge_follow(config->port, pin, true);
			break;
		default:
			return -ENOTSUP;
		}

		ch5xx_gpio_set_irq_on_edge(config->port, pin, true);
		ch5xx_gpio_set_irq_enable(config->port, pin, true);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_wch_ch5xx_manage_callback(const struct device *dev, struct gpio_callback *callback,
					  bool set)
{
	struct gpio_wch_ch5xx_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

#endif /* CONFIG_GPIO_WCH_CH5XX_GPIO_INTERRUPTS */

static DEVICE_API(gpio, gpio_wch_ch5xx_driver_api) = {
	.pin_configure = gpio_wch_ch5xx_configure,
	.port_get_raw = gpio_wch_ch5xx_port_get_raw,
	.port_set_masked_raw = gpio_wch_ch5xx_port_set_masked_raw,
	.port_set_bits_raw = gpio_wch_ch5xx_port_set_bits_raw,
	.port_clear_bits_raw = gpio_wch_ch5xx_port_clear_bits_raw,
	.port_toggle_bits = gpio_wch_ch5xx_port_toggle_bits,
#if defined(CONFIG_GPIO_WCH_CH5XX_GPIO_INTERRUPTS)
	.pin_interrupt_configure = gpio_wch_ch5xx_pin_interrupt_configure,
	.manage_callback = gpio_wch_ch5xx_manage_callback,
#endif
};

static int gpio_wch_ch5xx_init(const struct device *dev)
{
#if CONFIG_GPIO_WCH_CH5XX_GPIO_INTERRUPTS
	const struct gpio_wch_ch5xx_config *config = dev->config;
	config->irq_config_func(dev);
#endif
	return 0;
}

#ifdef CONFIG_GPIO_WCH_CH5XX_GPIO_INTERRUPTS
#define GPIO_WCH_CH5XX_IRQ_HANDLER_DECL(idx)                                                       \
	static void gpio_wch_ch5xx_irq_config_func_##idx(const struct device *dev);

#define GPIO_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)                                                       \
	.irq_config_func = gpio_wch_ch5xx_irq_config_func_##idx,

#define GPIO_WCH_CH5XX_IRQ_HANDLER(idx)                                                            \
	static void gpio_wch_ch5xx_irq_config_func_##idx(const struct device *dev)                 \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), gpio_wch_ch5xx_isr,     \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}
#else
#define GPIO_WCH_CH5XX_IRQ_HANDLER_DECL(idx)
#define GPIO_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)
#define GPIO_WCH_CH5XX_IRQ_HANDLER(idx)
#endif

#define GPIO_CH5XX_INIT(idx)                                                                       \
	GPIO_WCH_CH5XX_IRQ_HANDLER_DECL(idx)                                                       \
	static const struct gpio_wch_ch5xx_config gpio_wch_ch5xx_##idx##_config = {                \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),             \
			},                                                                         \
		.port = (GPIO_PORT_TypeDef *const)DT_INST_REG_ADDR(idx),                           \
		GPIO_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)};                                             \
                                                                                                   \
	static struct gpio_wch_ch5xx_data gpio_wch_ch5xx_##idx##_data;                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, gpio_wch_ch5xx_init, NULL, &gpio_wch_ch5xx_##idx##_data,        \
			      &gpio_wch_ch5xx_##idx##_config, PRE_KERNEL_1,                        \
			      CONFIG_GPIO_INIT_PRIORITY, &gpio_wch_ch5xx_driver_api);              \
	GPIO_WCH_CH5XX_IRQ_HANDLER(idx)

DT_INST_FOREACH_STATUS_OKAY(GPIO_CH5XX_INIT)
