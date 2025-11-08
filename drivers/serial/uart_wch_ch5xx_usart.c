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

#include <hal_ch32fun.h>

struct usart_wch_ch5xx_config {
	uint8_t parity;
	uint32_t current_speed;
	CH5XX_UART_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct usart_wch_ch5xx_data {
	uart_irq_callback_user_data_t cb;
	void *user_data;

#if defined(CONFIG_UART_ASYNC_API)
	/* async callback */
	uart_callback_t async_cb;
	void *async_user_data;

	/* TX async state */
	const uint8_t *tx_buf;
	size_t tx_len;
	size_t tx_idx;
	bool tx_active;

	/* RX async state */
	uint8_t *rx_buf;
	size_t rx_len;
	size_t rx_idx;
	bool rx_active;
#endif
};

static int usart_wch_ch5xx_err_check(const struct device *dev);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

static int usart_wch_ch5xx_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;
	uint8_t sent = 0;

	/* We are using FIFO, Let's wait for space in TX-FIFO */
	while ((regs->TFC < UART_FIFO_SIZE) && (sent < len)) {
		/* push to TX-FIFO */
		regs->DR = tx_data[sent];
		sent++;
	}

	return sent;
}

static int usart_wch_ch5xx_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;
	uint8_t read = 0;

	/* We are using FIFO, Let's wait for space in TX-FIFO */
	while ((regs->RFC > 0) && (read < size)) {
		/* read from RX-FIFO */
		rx_data[read] = regs->DR;
		read++;
	}

	return read;
}

static void usart_wch_ch5xx_irq_tx_enable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	regs->IER |= RB_IER_THR_EMPTY;
}

static void usart_wch_ch5xx_irq_tx_disable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	regs->IER &= ~RB_IER_THR_EMPTY;
}

static int usart_wch_ch5xx_irq_tx_ready(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	return (regs->TFC < UART_FIFO_SIZE);
}

static void usart_wch_ch5xx_irq_rx_enable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	regs->IER |= RB_IER_RECV_RDY;
}

static void usart_wch_ch5xx_irq_rx_disable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	regs->IER &= ~RB_IER_RECV_RDY;
}

static int usart_wch_ch5xx_irq_tx_complete(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	return (regs->TFC == 0);
}

static int usart_wch_ch5xx_irq_rx_ready(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	return (regs->RFC > 0);
}

static void usart_wch_ch5xx_irq_err_enable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	regs->IER |= RB_IER_LINE_STAT;
}

static void usart_wch_ch5xx_irq_err_disable(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	regs->IER &= ~RB_IER_LINE_STAT;
}

static int usart_wch_ch5xx_irq_is_pending(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	return ((regs->IIR & RB_IIR_NO_INT) == 0);
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

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if defined(CONFIG_UART_ASYNC_API)

/* Helper to emit a uart_event to user callback */
static inline void emit_async_event_tx_done(const struct device *dev, size_t len)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	struct uart_event ev;

	(void)memset(&ev, 0, sizeof(ev));
	ev.type = UART_TX_DONE;
	ev.data.tx.buf = data->tx_buf;
	ev.data.tx.len = len;
	if (data->async_cb) {
		data->async_cb(dev, &ev, data->async_user_data);
	}
}

static inline void emit_async_event_tx_aborted(const struct device *dev, size_t len)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	struct uart_event ev;

	(void)memset(&ev, 0, sizeof(ev));
	ev.type = UART_TX_ABORTED;
	ev.data.tx.buf = data->tx_buf;
	ev.data.tx.len = len;
	if (data->async_cb) {
		data->async_cb(dev, &ev, data->async_user_data);
	}
}

static inline void emit_async_event_rx_ready(const struct device *dev, size_t off, size_t len)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	struct uart_event ev;

	(void)memset(&ev, 0, sizeof(ev));
	ev.type = UART_RX_RDY;
	ev.data.rx.buf = data->rx_buf;
	ev.data.rx.offset = off;
	ev.data.rx.len = len;
	if (data->async_cb) {
		data->async_cb(dev, &ev, data->async_user_data);
	}
}

static inline void emit_async_event_rx_disabled(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	struct uart_event ev;

	(void)memset(&ev, 0, sizeof(ev));
	ev.type = UART_RX_DISABLED;
	if (data->async_cb) {
		data->async_cb(dev, &ev, data->async_user_data);
	}
}

static int usart_wch_ch5xx_async_callback_set(const struct device *dev, uart_callback_t callback,
					      void *user_data)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	data->async_cb = callback;
	data->async_user_data = user_data;
	return 0;
}

static int usart_wch_ch5xx_tx(const struct device *dev, const uint8_t *buf, size_t len,
			      int32_t timeout)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	if (len == 0 || buf == NULL) {
		return -EINVAL;
	}

	if (data->tx_active) {
		return -EBUSY;
	}

	data->tx_buf = buf;
	data->tx_len = len;
	data->tx_idx = 0;
	data->tx_active = true;

	while ((regs->TFC < UART_FIFO_SIZE) && (data->tx_idx < data->tx_len)) {
		regs->DR = data->tx_buf[data->tx_idx++];
	}

	regs->IER |= RB_IER_THR_EMPTY;

	return 0;
}

static int usart_wch_ch5xx_tx_abort(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	if (!data->tx_active) {
		return 0;
	}

	regs->IER &= ~RB_IER_THR_EMPTY;

	size_t sent = data->tx_idx;
	data->tx_active = false;

	emit_async_event_tx_aborted(dev, sent);

	return 0;
}

static int usart_wch_ch5xx_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				     int32_t timeout)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	if (data->rx_active) {
		return -EBUSY;
	}
	if (buf == NULL || len == 0) {
		return -EINVAL;
	}

	data->rx_buf = buf;
	data->rx_len = len;
	data->rx_idx = 0;
	data->rx_active = true;

	/* enable RX interrupt */
	regs->IER |= RB_IER_RECV_RDY;

	return 0;
}

static int usart_wch_ch5xx_rx_disable(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	if (!data->rx_active) {
		return 0;
	}

	/* disable RX interrupt */
	regs->IER &= ~RB_IER_RECV_RDY;
	data->rx_active = false;

	emit_async_event_rx_disabled(dev);
	return 0;
}

#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

static void usart_wch_ch5xx_isr(const struct device *dev)
{
	struct usart_wch_ch5xx_data *data = dev->data;
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

#if defined(CONFIG_UART_ASYNC_API)
	bool consumed = false;
	enum uart_rx_stop_reason errors = usart_wch_ch5xx_err_check(dev);

	/* RX error */
	if (data->rx_active && errors) {
		consumed = true;
		regs->IER &= ~RB_IER_RECV_RDY;
		data->rx_active = false;

		struct uart_event ev = {
			.type = UART_RX_STOPPED,
			.data.rx_stop.reason = errors,
		};
		data->async_cb(dev, &ev, data->async_user_data);
	}

	/* RX normal data */
	if (data->rx_active && (regs->RFC > 0)) {
		consumed = true;
		size_t start = data->rx_idx;

		while ((regs->RFC > 0) && (data->rx_idx < data->rx_len)) {
			data->rx_buf[data->rx_idx++] = regs->DR;
		}

		size_t got = data->rx_idx - start;
		if (got > 0) {
			emit_async_event_rx_ready(dev, start, got);
		}

		if (data->rx_idx >= data->rx_len) {
			regs->IER &= ~RB_IER_RECV_RDY;
			data->rx_active = false;
			emit_async_event_rx_disabled(dev);
		}
	}

	/* TX error */
	if (data->tx_active && errors) {
		consumed = true;
		regs->IER &= ~RB_IER_THR_EMPTY;

		size_t sent = data->tx_idx;
		data->tx_active = false;
		emit_async_event_tx_aborted(dev, sent);
	}

	/* TX normal */
	if (data->tx_active && (regs->TFC < UART_FIFO_SIZE)) {
		consumed = true;
		while ((regs->TFC < UART_FIFO_SIZE) && (data->tx_idx < data->tx_len)) {
			regs->DR = data->tx_buf[data->tx_idx++];
		}

		if (data->tx_idx >= data->tx_len) {
			regs->IER &= ~RB_IER_THR_EMPTY;
			data->tx_active = false;
			emit_async_event_tx_done(dev, data->tx_len);
		}
	}

	/* only allow interrupt callback trigger if async API hasn't consumed the interrupt */
	if (consumed) {
		return;
	}
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	if (data->cb) {
		data->cb(dev, data->user_data);
	} else {
		/* No callback registered, disable all interrupts */
		regs->IER = 0;
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

static int usart_wch_ch5xx_poll_in(const struct device *dev, unsigned char *ch)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	/* We are using FIFO, so let's see if we have a byte in RX-FIFO */
	if (regs->RFC == 0) {
		return -EAGAIN;
	}

	/* Read from RX-FIFO */
	*ch = regs->DR;
	return 0;
}

static void usart_wch_ch5xx_poll_out(const struct device *dev, unsigned char ch)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;

	/* We are using FIFO, Let's wait for space in TX-FIFO */
	while (regs->TFC >= UART_FIFO_SIZE) {
		k_yield();
	}

	/* push to TX-FIFO */
	regs->DR = ch;
}

static int usart_wch_ch5xx_err_check(const struct device *dev)
{
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;
	enum uart_rx_stop_reason errors = 0;
	uint8_t status;

	/* check line error via IIR/LSR */
	if ((regs->IIR & RB_IIR_INT_MASK) == RB_IIR_P0_LINE_ERR) {
		status = regs->LSR;
		if (status & RB_LSR_PAR_ERR) {
			errors |= UART_ERROR_PARITY;
		}
		if (status & RB_LSR_BREAK_ERR) {
			errors |= UART_BREAK;
		}
		if (status & RB_LSR_FRAME_ERR) {
			errors |= UART_ERROR_FRAMING;
		}
		if (status & RB_LSR_OVER_ERR) {
			errors |= UART_ERROR_OVERRUN;
		}
	}

	return errors;
}

static int usart_wch_ch5xx_init(const struct device *dev)
{
	int err;
	const struct usart_wch_ch5xx_config *config = dev->config;
	CH5XX_UART_TypeDef *regs = config->regs;
	uint8_t reg_val;

	/* reset peripheral */
	regs->IER = RB_IER_RESET;

	/* Baud rate = Fsys * 2 / R8_UART_DIV / 16 / R16_UART_DL
	 * So, R16_UART_DL = Fsys * 2 / R8_UART_DIV / 16 / Baud rate
	 * if R8_UART_DIV = 1, then
	 * R16_UART_DL = Fsys / 8 / Baud rate = (Fsys >> 3) / Baud rate
	 */
	regs->DIV = 1;
	regs->DLL = (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >> 3) / config->current_speed;

	/* Enable FIFO, reset RX/TX FIFOs, set RX trigger level to 2 byte */
	regs->FCR = (RB_FCR_FIFO_EN | RB_FCR_RX_FIFO_CLR | RB_FCR_TX_FIFO_CLR | RB_FCR_TRIG_02);

	/* 8 data bits, 1 start, 1 stop */
	reg_val = RB_LCR_WORD_SZ;
	/* Enable parity bit */
	if (config->parity != UART_CFG_PARITY_NONE) {
		reg_val |= RB_LCR_PAR_EN;
		if (config->parity == UART_CFG_PARITY_EVEN) {
			reg_val |= RB_LCR_PAR_MOD_EVN;
		} else if (config->parity == UART_CFG_PARITY_ODD) {
			reg_val |= RB_LCR_PAR_MOD_ODD;
		} else {
			return -EINVAL;
		}
	}
	regs->LCR = reg_val;

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	regs->IER = RB_IER_TXD_EN | RB_IER_LINE_STAT | RB_IER_RECV_RDY;
	config->irq_config_func(dev);
	regs->MCR |= RB_MCR_INT_OE;
#else
	regs->IER = RB_IER_TXD_EN;
#endif

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
	.rx_disable = usart_wch_ch5xx_rx_disable,
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
		.regs = (CH5XX_UART_TypeDef *)DT_INST_REG_ADDR(idx),                               \
		.current_speed = DT_INST_PROP(idx, current_speed),                                 \
		.parity = DT_INST_ENUM_IDX(idx, parity),                                           \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                    \
		USART_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)};                                            \
	DEVICE_DT_INST_DEFINE(idx, &usart_wch_ch5xx_init, NULL, &usart_wch_ch5xx_##idx##_data,     \
			      &usart_wch_ch5xx_##idx##_config, PRE_KERNEL_1,                       \
			      CONFIG_SERIAL_INIT_PRIORITY, &usart_wch_ch5xx_driver_api);           \
	USART_WCH_CH5XX_IRQ_HANDLER(idx)

DT_INST_FOREACH_STATUS_OKAY(USART_WCH_CH5XX_INIT)
