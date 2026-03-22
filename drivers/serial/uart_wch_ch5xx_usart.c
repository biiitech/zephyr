/*
 * Copyright (c) 2025 BIII Tech LLP.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_usart

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include <hal/wch/ch5xx/ch5xx.h>

struct usart_wch_ch5xx_config {
	uint8_t parity;
	uint32_t current_speed;
	UART_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct usart_wch_ch5xx_data {
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_callback_user_data_t cb;
	void *user_data;
#elif defined(CONFIG_UART_ASYNC_API)
	/* Device reference for work handler */
	const struct device *dev;
	enum uart_rx_stop_reason errors;

	/* we'll do high priority task in isr and low priority task in work handler */
	struct k_work_delayable isr_work_handler;
	int32_t rx_timeout_ms;

	/* async callback */
	uart_callback_t cb;
	void *user_data;

	/* TX Buffers */
	const uint8_t *tx_buf;
	int tx_len;
	int tx_idx;

	/* RX Buffers */
	uint8_t *rx_buf[2];
	int rx_len[2];
	int cur_rx_idx;
	int last_rx_idx;
#endif
};

static int usart_wch_ch5xx_err_check(const struct device *dev);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

static int usart_wch_ch5xx_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	uint8_t sent = 0;

	/* We are using FIFO, Let's wait for space in TX-FIFO */
	while (ch5xx_uart_tx_fifo_ready(config->regs) && (sent < len)) {
		/* push to TX-FIFO */
		ch5xx_uart_tx_fifo_push(config->regs, tx_data[sent]);
		sent++;
	}

	return sent;
}

static int usart_wch_ch5xx_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	uint8_t read = 0;

	/* We are using FIFO, read from RX-FIFO */
	while (ch5xx_uart_rx_fifo_ready(config->regs) && (read < size)) {
		/* read from RX-FIFO */
		rx_data[read] = ch5xx_uart_rx_fifo_pop(config->regs);
		read++;
	}

	return read;
}

static void usart_wch_ch5xx_irq_tx_enable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	ch5xx_uart_irq_set_tx_enable(config->regs, true);
}

static void usart_wch_ch5xx_irq_tx_disable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	ch5xx_uart_irq_set_tx_enable(config->regs, false);
}

static int usart_wch_ch5xx_irq_tx_ready(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	return ch5xx_uart_tx_fifo_ready(config->regs);
}

static void usart_wch_ch5xx_irq_rx_enable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	ch5xx_uart_irq_set_rx_enable(config->regs, true);
}

static void usart_wch_ch5xx_irq_rx_disable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	ch5xx_uart_irq_set_rx_enable(config->regs, false);
}

static int usart_wch_ch5xx_irq_tx_complete(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	return ch5xx_uart_tx_fifo_empty(config->regs);
}

static int usart_wch_ch5xx_irq_rx_ready(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	return ch5xx_uart_rx_fifo_ready(config->regs);
}

static void usart_wch_ch5xx_irq_err_enable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	ch5xx_uart_irq_set_line_status_enable(config->regs, true);
}

static void usart_wch_ch5xx_irq_err_disable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	ch5xx_uart_irq_set_line_status_enable(config->regs, false);
}

static int usart_wch_ch5xx_irq_is_pending(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

	return ch5xx_uart_irq_is_pending(config->regs);
}

static int usart_wch_ch5xx_irq_update(const struct device *dev)
{
	return 1;
}

static void usart_wch_ch5xx_irq_callback_set(const struct device *dev,
					     uart_irq_callback_user_data_t cb, void *user_data)
{
	struct usart_wch_ch5xx_data *data = dev->data;

	data->cb = cb;
	data->user_data = user_data;
}

#elif defined(CONFIG_UART_ASYNC_API)

#define IS_TX_ACTIVE(data) ((data)->tx_buf != NULL)

#define SET_TX_BUFFER(data, config, buf, len)                                                      \
	do {                                                                                       \
		(data)->tx_buf = (buf);                                                            \
		(data)->tx_len = (len);                                                            \
		(data)->tx_idx = 0;                                                                \
		while (ch5xx_uart_tx_fifo_ready((config)->regs) &&                                 \
		       ((data)->tx_idx < (data)->tx_len)) {                                        \
			ch5xx_uart_tx_fifo_push((config)->regs, (data)->tx_buf[(data)->tx_idx++]); \
		}                                                                                  \
		ch5xx_uart_irq_set_tx_enable((config)->regs, true);                                \
	} while (0)

#define RESET_TX_BUFFER(data, config)                                                              \
	do {                                                                                       \
		(data)->tx_buf = NULL;                                                             \
		(data)->tx_len = 0;                                                                \
		(data)->tx_idx = 0;                                                                \
	} while (0)

#define IS_RX_ACTIVE(data) ((data)->rx_buf[0] != NULL)

#define IS_RX_DOUBLE_BUFFERED(data) ((data)->rx_buf[1] != NULL)

#define SET_RX_BUFFER(data, config, idx, buf, len)                                                 \
	do {                                                                                       \
		(data)->rx_buf[(idx)] = (buf);                                                     \
		(data)->rx_len[(idx)] = (len);                                                     \
		if ((idx) == 0) {                                                                  \
			(data)->cur_rx_idx = 0;                                                    \
			(data)->last_rx_idx = 0;                                                   \
			ch5xx_uart_irq_set_rx_enable((config)->regs, true);                        \
		}                                                                                  \
	} while (0)

#define RESET_RX_BUFFER(data, config, idx)                                                         \
	do {                                                                                       \
		(data)->rx_buf[(idx)] = NULL;                                                      \
		(data)->rx_len[(idx)] = 0;                                                         \
		if ((idx) == 0) {                                                                  \
			(data)->last_rx_idx = 0;                                                   \
			(data)->cur_rx_idx = 0;                                                    \
		}                                                                                  \
	} while (0)

#define SWAP_RX_BUFFER(data)                                                                       \
	do {                                                                                       \
		(data)->rx_buf[0] = (data)->rx_buf[1];                                             \
		(data)->rx_len[0] = (data)->rx_len[1];                                             \
		(data)->rx_buf[1] = NULL;                                                          \
		(data)->rx_len[1] = 0;                                                             \
		(data)->last_rx_idx = 0;                                                           \
		(data)->cur_rx_idx = 0;                                                            \
	} while (0)

static int usart_wch_ch5xx_async_callback_set(const struct device *dev, uart_callback_t callback,
					      void *user_data)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	data->cb = callback;
	data->user_data = user_data;
	return 0;
}

static int usart_wch_ch5xx_tx(const struct device *dev, const uint8_t *buf, size_t len,
			      int32_t timeout)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;

	if (IS_TX_ACTIVE(data)) {
		return -EBUSY;
	}

	if (len == 0 || buf == NULL) {
		return -EINVAL;
	}

	SET_TX_BUFFER(data, config, buf, len);

	return 0;
}

static int usart_wch_ch5xx_tx_abort(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	struct uart_event ev = {0};

	if (!IS_TX_ACTIVE(data)) {
		return 0;
	}

	ch5xx_uart_irq_set_tx_enable(config->regs, false);

	ev.type = UART_TX_ABORTED;
	ev.data.tx.buf = data->tx_buf;
	ev.data.tx.len = data->tx_idx;
	data->cb(dev, &ev, data->user_data);

	RESET_TX_BUFFER(data, config);

	return 0;
}

static int usart_wch_ch5xx_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				     int32_t timeout)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;

	if (IS_RX_ACTIVE(data)) {
		return -EBUSY;
	}
	if (buf == NULL || len == 0) {
		return -EINVAL;
	}

	SET_RX_BUFFER(data, config, 0, buf, len);
	data->rx_timeout_ms = timeout;

	/* Start RX timeout if requested */
	if (timeout > 0) {
		k_work_schedule(&data->isr_work_handler, K_MSEC(timeout));
	}

	return 0;
}

static int usart_wch_ch5xx_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;

	if ((!IS_RX_ACTIVE(data)) || (!buf) || (len == 0)) {
		return -EINVAL;
	}
	if (IS_RX_DOUBLE_BUFFERED(data)) {
		return -EBUSY;
	}
	SET_RX_BUFFER(data, config, 1, buf, len);
	return 0;
}

static int usart_wch_ch5xx_rx_disable(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	struct uart_event ev = {0};

	if (!IS_RX_ACTIVE(data)) {
		return 0;
	}

	/* disable RX interrupt */
	ch5xx_uart_irq_set_rx_enable(config->regs, false);

	if (data->cur_rx_idx > data->last_rx_idx) {
		ev.type = UART_RX_RDY;
		ev.data.rx.buf = data->rx_buf[0];
		ev.data.rx.offset = data->last_rx_idx;
		ev.data.rx.len = data->cur_rx_idx - data->last_rx_idx;
		data->cb(data->dev, &ev, data->user_data);
	}

	/* send RX disabled event */
	memset(&ev, 0, sizeof(ev));
	ev.type = UART_RX_DISABLED;
	data->cb(data->dev, &ev, data->user_data);

	RESET_RX_BUFFER(data, config, 0);
	RESET_RX_BUFFER(data, config, 1);

	return 0;
}

static void usart_wch_ch5xx_isr_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct usart_wch_ch5xx_data *data =
		CONTAINER_OF(dwork, struct usart_wch_ch5xx_data, isr_work_handler);
	const struct usart_wch_ch5xx_config *config = data->dev->config;
	struct uart_event ev = {0};
	bool rx_buf_full;
	bool rx_hw_timeout;

	/* if we were transmitting */
	if (data->tx_buf) {
		if (data->tx_idx >= data->tx_len) {
			ev.type = UART_TX_DONE;
			ev.data.tx.buf = data->tx_buf;
			ev.data.tx.len = data->tx_len;
			data->cb(data->dev, &ev, data->user_data);
			RESET_TX_BUFFER(data, config);
		}
	}

	if (data->cur_rx_idx > data->last_rx_idx) {
		/* if we have started receiving data and second buffer is not available */
		if (data->last_rx_idx == 0 && data->rx_buf[1] == NULL) {
			ev.type = UART_RX_BUF_REQUEST;
			data->cb(data->dev, &ev, data->user_data);
		}
		/* Correct buffer full logic for shell compatibility */
		rx_buf_full = (data->cur_rx_idx >= data->rx_len[0]);
		rx_hw_timeout = ch5xx_uart_has_rx_timeout(config->regs);
		if (rx_buf_full || rx_hw_timeout) {
			ev.type = UART_RX_RDY;
			ev.data.rx.buf = data->rx_buf[0];
			ev.data.rx.offset = data->last_rx_idx;
			ev.data.rx.len = data->cur_rx_idx - data->last_rx_idx;
			data->cb(data->dev, &ev, data->user_data);
			data->last_rx_idx = data->cur_rx_idx;
		}
		if (rx_buf_full) {
			memset(&ev, 0, sizeof(ev));
			ev.type = UART_RX_BUF_RELEASED;
			ev.data.rx_buf.buf = data->rx_buf[0];
			data->cb(data->dev, &ev, data->user_data);
			SWAP_RX_BUFFER(data);
		}
	}

	if (data->errors) {
		ev.type = UART_RX_STOPPED;
		ev.data.rx_stop.reason = data->errors;
		data->cb(data->dev, &ev, data->user_data);
		data->errors = 0;
		RESET_RX_BUFFER(data, config, 0);
		RESET_RX_BUFFER(data, config, 1);
	} else {
		/* If we have a buffer available, schedule timeout */
		if ((data->rx_buf[0] != NULL) && (data->rx_timeout_ms > 0)) {
			k_work_schedule(&data->isr_work_handler, K_MSEC(data->rx_timeout_ms));
		} else {
			k_work_cancel_delayable(&data->isr_work_handler);
		}
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

static void usart_wch_ch5xx_isr(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	if (data->cb) {
		data->cb(dev, data->user_data);
	} else {
		/* No callback registered, disable all interrupts */
		ch5xx_uart_irq_set_rx_enable(config->regs, false);
		ch5xx_uart_irq_set_tx_enable(config->regs, false);
		ch5xx_uart_irq_set_line_status_enable(config->regs, false);
	}
#elif defined(CONFIG_UART_ASYNC_API)
	data->errors = usart_wch_ch5xx_err_check(dev);

	if (data->errors) {
		/*  RX Line state error reported stop rx */
		ch5xx_uart_irq_set_rx_enable(config->regs, false);
		return;
	}

	/* if we are supposed to receive data */
	if ((data->rx_buf[0]) && (data->rx_len[0] > 0)) {
		while (ch5xx_uart_rx_fifo_ready(config->regs) &&
		       (data->cur_rx_idx < data->rx_len[0])) {
			data->rx_buf[0][data->cur_rx_idx++] = ch5xx_uart_rx_fifo_pop(config->regs);
		}
	}

	/* if are supposed to send, the fill fifo */
	if (data->tx_buf) {
		while (ch5xx_uart_tx_fifo_ready(config->regs) && (data->tx_idx < data->tx_len)) {
			ch5xx_uart_tx_fifo_push(config->regs, data->tx_buf[data->tx_idx++]);
		}
	}

	k_work_schedule(&data->isr_work_handler, K_NO_WAIT);
#endif
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int usart_wch_ch5xx_poll_in(const struct device *dev, unsigned char *ch)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

#if CONFIG_UART_ASYNC_API
	struct usart_wch_ch5xx_data *data = dev->data;

	if (IS_RX_ACTIVE(data)) {
		return -EBUSY;
	}
#endif
	/* We are using FIFO, so let's see if we have a byte in RX-FIFO */
	if (!ch5xx_uart_rx_fifo_ready(config->regs)) {
		return -EAGAIN;
	}

	/* Read from RX-FIFO */
	*ch = ch5xx_uart_rx_fifo_pop(config->regs);
	return 0;
}

static void usart_wch_ch5xx_poll_out(const struct device *dev, unsigned char ch)
{
	const struct usart_wch_ch5xx_config *config = dev->config;

#if CONFIG_UART_ASYNC_API
	struct usart_wch_ch5xx_data *data = dev->data;

	if (data->tx_buf) {
		return;
	}
#endif
	/* We are using FIFO, Let's wait for space in TX-FIFO */
	while (!ch5xx_uart_tx_fifo_ready(config->regs)) {
	}

	/* push to TX-FIFO */
	ch5xx_uart_tx_fifo_push(config->regs, ch);
}

static int usart_wch_ch5xx_err_check(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	enum uart_rx_stop_reason errors = 0;
	uint8_t status;

	/* check line error via IIR/LSR */
	if (ch5xx_uart_has_line_error(config->regs)) {
		status = ch5xx_uart_line_status_get(config->regs);
		if (ch5xx_uart_line_status_has_parity_error(status)) {
			errors |= UART_ERROR_PARITY;
		}
		if (ch5xx_uart_line_status_has_break_error(status)) {
			errors |= UART_BREAK;
		}
		if (ch5xx_uart_line_status_has_framing_error(status)) {
			errors |= UART_ERROR_FRAMING;
		}
		if (ch5xx_uart_line_status_has_overrun_error(status)) {
			errors |= UART_ERROR_OVERRUN;
		}
	}

	return errors;
}

static int usart_wch_ch5xx_init(const struct device *dev)
{
	int err;
	const struct usart_wch_ch5xx_config *config = dev->config;
#if defined(CONFIG_UART_ASYNC_API)
	struct usart_wch_ch5xx_data *data = dev->data;
#endif

	/* reset peripheral */
	ch5xx_uart_reset(config->regs);

	ch5xx_uart_set_baud_rate(config->regs, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
				 config->current_speed);

	ch5xx_uart_set_fifo_enable(config->regs, true);
	/* use higher RX threshold to reduce interrupt storm */
	ch5xx_uart_set_fifo_rx_threshold(config->regs, 7);

	/* 8 data bits, 1 start, 1 stop */
	ch5xx_uart_set_data_bits(config->regs, 8);
	ch5xx_uart_set_stop_bits(config->regs, 1);
	switch (config->parity) {
	case UART_CFG_PARITY_NONE:
		ch5xx_uart_set_parity(config->regs, CH5XX_UART_PARITY_NONE);
		break;
	case UART_CFG_PARITY_EVEN:
		ch5xx_uart_set_parity(config->regs, CH5XX_UART_PARITY_EVEN);
		break;
	case UART_CFG_PARITY_ODD:
		ch5xx_uart_set_parity(config->regs, CH5XX_UART_PARITY_ODD);
		break;
	default:
		return -EINVAL;
	}

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

#if defined(CONFIG_UART_ASYNC_API) || defined(CONFIG_UART_INTERRUPT_DRIVEN)
#if defined(CONFIG_UART_ASYNC_API)
	data->dev = dev;
	k_work_init_delayable(&data->isr_work_handler, usart_wch_ch5xx_isr_work_handler);
#endif
	ch5xx_uart_irq_set_line_status_enable(config->regs, true);
	ch5xx_uart_irq_set_rx_enable(config->regs, true);
	ch5xx_uart_set_irq_enable(config->regs, true);
	config->irq_config_func(dev);
#endif
	ch5xx_uart_tx_enable((config)->regs, true);

	return 0;
}

static int usart_wch_ch5xx_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	UART_TypeDef *regs = config->regs;
	uint8_t stop_bits;
	uint8_t parity;

	if (!cfg) {
		return -EINVAL;
	}

	/* Map Zephyr stop-bits enum (0-3) to HAL count (1 or 2) */
	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		stop_bits = 1;
		break;
	case UART_CFG_STOP_BITS_2:
		stop_bits = 2;
		break;
	default:
		return -ENOTSUP;
	}

	/* Map Zephyr parity enum to CH5XX HAL enum */
	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		parity = CH5XX_UART_PARITY_NONE;
		break;
	case UART_CFG_PARITY_ODD:
		parity = CH5XX_UART_PARITY_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		parity = CH5XX_UART_PARITY_EVEN;
		break;
	default:
		return -ENOTSUP;
	}

	ch5xx_uart_set_baud_rate(regs, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, cfg->baudrate);
	/* Map Zephyr data-bits enum (0=5bit … 3=8bit) to actual count (+5) */
	ch5xx_uart_set_data_bits(regs, cfg->data_bits + 5);
	ch5xx_uart_set_stop_bits(regs, stop_bits);
	ch5xx_uart_set_parity(regs, parity);
	return 0;
}

static int usart_wch_ch5xx_config_get(const struct device *dev, struct uart_config *cfg)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	if (!cfg) {
		return -EINVAL;
	}
	/* These are not read from hardware, but from config struct */
	cfg->baudrate = config->current_speed;
	cfg->parity = config->parity;
	cfg->data_bits = UART_CFG_DATA_BITS_8;
	cfg->stop_bits = UART_CFG_STOP_BITS_1;
	cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	return 0;
}

static const struct uart_driver_api usart_wch_ch5xx_driver_api = {
	.poll_in = usart_wch_ch5xx_poll_in,
	.poll_out = usart_wch_ch5xx_poll_out,
	.err_check = usart_wch_ch5xx_err_check,
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	.fifo_fill = usart_wch_ch5xx_fifo_fill,
	.fifo_read = usart_wch_ch5xx_fifo_read,
	.irq_tx_enable = usart_wch_ch5xx_irq_tx_enable,
	.irq_tx_disable = usart_wch_ch5xx_irq_tx_disable,
	.irq_tx_ready = usart_wch_ch5xx_irq_tx_ready,
	.irq_rx_enable = usart_wch_ch5xx_irq_rx_enable,
	.irq_rx_disable = usart_wch_ch5xx_irq_rx_disable,
	.irq_tx_complete = usart_wch_ch5xx_irq_tx_complete,
	.irq_rx_ready = usart_wch_ch5xx_irq_rx_ready,
	.irq_err_enable = usart_wch_ch5xx_irq_err_enable,
	.irq_err_disable = usart_wch_ch5xx_irq_err_disable,
	.irq_is_pending = usart_wch_ch5xx_irq_is_pending,
	.irq_update = usart_wch_ch5xx_irq_update,
	.irq_callback_set = usart_wch_ch5xx_irq_callback_set,
#endif
#if defined(CONFIG_UART_ASYNC_API)
	.callback_set = usart_wch_ch5xx_async_callback_set,
	.tx = usart_wch_ch5xx_tx,
	.tx_abort = usart_wch_ch5xx_tx_abort,
	.rx_enable = usart_wch_ch5xx_rx_enable,
	.rx_buf_rsp = usart_wch_ch5xx_rx_buf_rsp,
	.rx_disable = usart_wch_ch5xx_rx_disable,
#endif
#if defined(CONFIG_UART_USE_RUNTIME_CONFIGURE)
	.configure = usart_wch_ch5xx_configure,
	.config_get = usart_wch_ch5xx_config_get,
#endif
};

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define USART_WCH_CH5XX_IRQ_HANDLER_DECL(idx)                                                      \
	static void usart_wch_ch5xx_irq_config_func_##idx(const struct device *dev);
#define USART_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)                                                      \
	.irq_config_func = usart_wch_ch5xx_irq_config_func_##idx,
#define USART_WCH_CH5XX_IRQ_HANDLER(idx)                                                           \
	static void usart_wch_ch5xx_irq_config_func_##idx(const struct device *dev)                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), usart_wch_ch5xx_isr,    \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}
#else
#define USART_WCH_CH5XX_IRQ_HANDLER_DECL(idx)
#define USART_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)
#define USART_WCH_CH5XX_IRQ_HANDLER(idx)
#endif

#define USART_WCH_CH5XX_INIT(idx)                                                                  \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	USART_WCH_CH5XX_IRQ_HANDLER_DECL(idx)                                                      \
	static struct usart_wch_ch5xx_data usart_wch_ch5xx_##idx##_data;                           \
	static const struct usart_wch_ch5xx_config usart_wch_ch5xx_##idx##_config = {              \
		.regs = (UART_TypeDef *)DT_INST_REG_ADDR(idx),                                     \
		.current_speed = DT_INST_PROP(idx, current_speed),                                 \
		.parity = DT_INST_ENUM_IDX(idx, parity),                                           \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                    \
		USART_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)};                                            \
	DEVICE_DT_INST_DEFINE(idx, &usart_wch_ch5xx_init, NULL, &usart_wch_ch5xx_##idx##_data,     \
			      &usart_wch_ch5xx_##idx##_config, PRE_KERNEL_1,                       \
			      CONFIG_SERIAL_INIT_PRIORITY, &usart_wch_ch5xx_driver_api);           \
	USART_WCH_CH5XX_IRQ_HANDLER(idx)

DT_INST_FOREACH_STATUS_OKAY(USART_WCH_CH5XX_INIT)
