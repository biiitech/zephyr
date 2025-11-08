/*
 * drivers/spi/spi_wch_ch5xx.c
 *
 * CH5XX SPI0 driver using internal SPI DMA (half-duplex) with Zephyr SPI_ASYNC_API.
 *
 * - Single-fragment spi_buf_set supported (scatter/gather NOT implemented).
 * - Internal peripheral DMA: program DMA_BEG/DMA_END, set direction, enable DMA.
 * - ISR chains TX -> RX when both tx and rx buffers provided (no runtime threads).
 * - Completion: k_work for async callback, k_sem for synchronous wrapper.
 *
 * Before using:
 *  - Verify register and bit macro names against your hal_ch32fun.h / CH5XXSFR.h
 *  - Ensure device tree node for SPI provides irq (DT_INST_IRQN / priority).
 *  - Adjust compute_spi_divider() to match CH5XX SPI clock formula if needed.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_spi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <errno.h>
#include <sys/__assert.h>
#include <sys/atomic.h>

#include "hal_ch32fun.h" /* must define CH5XX_SPI_TypeDef and register fields */

/* ------------------------------------------------------------------------- */
/* Configuration knobs */
#ifndef CH57X_SPI_DMA_TIMEOUT_US
#define CH57X_SPI_DMA_TIMEOUT_US 500000U /* 500 ms default timeout for sync wrapper */
#endif

/* ------------------------------------------------------------------------- */
/* Hardware register / bit helpers - match these to your HAL if names differ */
/* These names match the datasheet-style fields we discussed earlier.        */
/* Replace with the exact symbols from CH5XXSFR.h or hal_ch32fun.h if needed. */

#ifndef RB_SPI_DMA_ENABLE
#define RB_SPI_DMA_ENABLE BIT(0) /* enable internal DMA engine */
#endif

#ifndef RB_SPI_FIFO_DIR
#define RB_SPI_FIFO_DIR BIT(1) /* FIFO direction: 0=TX,1=RX (example) */
#endif

#ifndef RB_SPI_IE_DMA_END
#define RB_SPI_IE_DMA_END BIT(3) /* interrupt enable: DMA end */
#endif

#ifndef RB_SPI_IF_DMA_END
#define RB_SPI_IF_DMA_END BIT(3) /* interrupt flag: DMA end (write 1 to clear) */
#endif

#ifndef SPI_FIFO_SIZE
#define SPI_FIFO_SIZE 8
#endif

/* Substitute register names if your HAL uses R8_/R16_ prefixes */
#define SPI_CTRL_MOD(regs)   ((regs)->CTRL_MOD)
#define SPI_INTER_EN(regs)   ((regs)->INTER_EN)
#define SPI_INT_FLAG(regs)   ((regs)->INT_FLAG)
#define SPI_FIFO_COUNT(regs) ((regs)->FIFO_COUNT)
#define SPI_BUFFER(regs)     ((regs)->BUFFER) /* data buffer reg */
#define SPI_DMA_BEG(regs)    ((regs)->DMA_BEG)
#define SPI_DMA_END(regs)    ((regs)->DMA_END)
#define SPI_CLOCK_DIV(regs)  ((regs)->CLOCK_DIV)
#define SPI_STATUS(regs)     ((regs)->STATUS) /* optional status register, TXE/RXNE bits etc */

/* ------------------------------------------------------------------------- */
/* Device config & runtime data */

struct spi_wch_ch5xx_config {
	CH5XX_SPI_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
	int irq_line;
	uint32_t irq_priority;
};

struct spi_wch_ch5xx_data {
	/* async callback: signature is (const struct device *dev, int status, void *user_data)
	 * For compatibility, we call with status==0 on success, <0 on error.
	 */
	spi_callback_t async_cb;
	void *async_user_data;

	/* transfer context */
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t tx_len;
	size_t rx_len;

	/* transfer state */
	atomic_t busy;
	bool tx_phase; /* true when current phase was TX (so ISR can chain to RX) */

	/* sync wrapper semaphore */
	struct k_sem sync_sem;

	/* work item to invoke async callback from system workqueue */
	struct k_work callback_work;
	int callback_status; /* status to pass to callback */
};

/* Convenience accessor */
static inline CH5XX_SPI_TypeDef *SPI_REGS(const struct device *dev)
{
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	return cfg->regs;
}

/* Forward declarations */
static int spi_wch_ch5xx_configure(const struct device *dev, const struct spi_config *config);
static int spi_wch_ch5xx_transceive(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs);
#ifdef CONFIG_SPI_ASYNC
static int spi_wch_ch5xx_transceive_async(const struct device *dev, const struct spi_config *config,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs);
static int spi_wch_ch5xx_callback_set(const struct device *dev, spi_callback_t cb, void *user_data);
#endif

/* Helper: extract single-fragment buffer (simple driver) */
static int extract_single_buf_set(const struct spi_buf_set *set, const struct spi_buf **out_buf,
				  size_t *out_len)
{
	if (!set || set->count == 0) {
		*out_buf = NULL;
		*out_len = 0;
		return 0;
	}
	if (set->count > 1) {
		/* scatter-gather not supported in this simple implementation */
		return -ENOTSUP;
	}
	*out_buf = &set->buffers[0];
	*out_len = set->buffers[0].len;
	return 0;
}

/* Compute SPI clock divider - adjust to CH5XX formula if different */
static uint8_t compute_spi_divider(uint32_t sysclk_hz, uint32_t target_hz)
{
	if (target_hz == 0 || sysclk_hz == 0) {
		return 0;
	}
	uint32_t div = sysclk_hz / target_hz;
	if (div == 0) {
		div = 1;
	}
	if (div > 0xFF) {
		div = 0xFF;
	}
	return (uint8_t)div;
}

/* Initialize peripheral + runtime data */
static int spi_wch_ch5xx_init(const struct device *dev)
{
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	CH5XX_SPI_TypeDef *regs = cfg->regs;
	struct spi_wch_ch5xx_data *d = dev->data;
	int err;

	/* apply pinctrl state */
	err = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	/* disable interrupts and DMA */
	SPI_INTER_EN(regs) = 0;
	SPI_CTRL_MOD(regs) &= ~RB_SPI_DMA_ENABLE;

	/* init runtime */
	k_sem_init(&d->sync_sem, 0, 1);
	atomic_set(&d->busy, 0);
	d->async_cb = NULL;
	d->async_user_data = NULL;
	d->tx_buf = NULL;
	d->rx_buf = NULL;
	d->tx_len = d->rx_len = 0;
	d->tx_phase = false;
	k_work_init(&d->callback_work, NULL); /* we set handler later per-instance */

	/* connect IRQ if present */
	if (cfg->irq_line >= 0) {
		IRQ_CONNECT(cfg->irq_line, cfg->irq_priority, spi_wch_ch5xx_transceive_async,
			    DEVICE_GET(dev), 0);
		irq_enable(cfg->irq_line);
	}

	return 0;
}

/* Configure SPI (mode, word size, frequency) */
static int spi_wch_ch5xx_configure(const struct device *dev, const struct spi_config *config)
{
	CH5XX_SPI_TypeDef *regs = SPI_REGS(dev);

	if (!config) {
		return -EINVAL;
	}

	/* Example configuration:
	 * - set master/slave bit
	 * - set CPOL/CPHA
	 * - set word size (we assume 8-bit common)
	 * - program clock divider
	 *
	 * Replace bit placements with exact bits from your CH57x headers.
	 */
	uint8_t ctrl = 0;

	if (config->operation & SPI_OP_MODE_MASTER) {
		ctrl |= (1 << 0); /* TODO: replace with actual master bit */
	}
	if (config->operation & SPI_MODE_CPOL) {
		ctrl |= (1 << 1); /* TODO: CPOL bit */
	}
	if (config->operation & SPI_MODE_CPHA) {
		ctrl |= (1 << 2); /* TODO: CPHA bit */
	}

	/* word size - assume 8-bit unless 16 requested */
	if (SPI_WORD_SIZE_GET(config->operation) == 16) {
		ctrl |= (1 << 3); /* TODO: set 16-bit mode bit if exists */
	}

	SPI_CTRL_MOD(regs) = ctrl;

	/* clock div */
	uint8_t div = compute_spi_divider(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, config->frequency);
	SPI_CLOCK_DIV(regs) = div;

	return 0;
}

/* Helper: program internal DMA and start (direction: true=TX, false=RX) */
static int spi_wch_ch5xx_start_dma(const struct device *dev, const void *buf, size_t len, bool tx)
{
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	CH5XX_SPI_TypeDef *regs = cfg->regs;

	if (!buf || len == 0) {
		return -EINVAL;
	}

	/* set FIFO direction first */
	if (tx) {
		SPI_CTRL_MOD(regs) &= ~RB_SPI_FIFO_DIR; /* 0 -> TX direction (example) */
	} else {
		SPI_CTRL_MOD(regs) |= RB_SPI_FIFO_DIR; /* 1 -> RX direction */
	}

	/* program DMA addresses - datasheet uses begin & end addresses */
	SPI_DMA_BEG(regs) = (uint32_t)buf;
	SPI_DMA_END(regs) = (uint32_t)((const uint8_t *)buf + len);

	/* clear DMA end flag (write 1 to clear convention) */
	SPI_INT_FLAG(regs) = RB_SPI_IF_DMA_END;

	/* enable DMA end interrupt */
	SPI_INTER_EN(regs) |= RB_SPI_IE_DMA_END;

	/* enable internal DMA engine */
	SPI_CTRL_MOD(regs) |= RB_SPI_DMA_ENABLE;

	return 0;
}

/* Work handler to call user async callback from system workqueue.
 * We put the handler here and set it to instance-specific function below.
 */
static void spi_wch_ch5xx_callback_work_handler(struct k_work *work)
{
	struct spi_wch_ch5xx_data *d = CONTAINER_OF(work, struct spi_wch_ch5xx_data, callback_work);
	/* call user callback with status passed in callback_status (0 == success) */
	if (d->async_cb) {
		d->async_cb(NULL, d->callback_status, d->async_user_data);
	}
}

/* ISR: handle DMA_END and chain TX->RX when both buffers provided */
static void spi_wch_ch5xx_isr(const struct device *dev)
{
	struct spi_wch_ch5xx_data *d = dev->data;
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	CH5XX_SPI_TypeDef *regs = cfg->regs;
	uint8_t flags = SPI_INT_FLAG(regs);

	/* DMA end */
	if (flags & RB_SPI_IF_DMA_END) {
		/* clear flag */
		SPI_INT_FLAG(regs) = RB_SPI_IF_DMA_END;

		/* disable DMA engine for this pass */
		SPI_CTRL_MOD(regs) &= ~RB_SPI_DMA_ENABLE;

		/* disable DMA end interrupt */
		SPI_INTER_EN(regs) &= ~RB_SPI_IE_DMA_END;

		/* If we just finished a TX pass and an RX buffer exists, kick RX DMA */
		if (d->tx_phase && d->rx_buf && d->rx_len > 0) {
			/* chain into RX pass */
			d->tx_phase = false;
			/* start RX DMA (rx buffer & rx_len previously set by transceive_async) */
			(void)spi_wch_ch5xx_start_dma(dev, d->rx_buf, d->rx_len, false);
			/* return — final completion will come when RX DMA ends */
			return;
		}

		/* Otherwise final completion (TX-only or RX-only, or RX after TX done) */
		atomic_set(&d->busy, 0);

		/* schedule callback via workqueue with status 0 */
		d->callback_status = 0;
		k_work_submit(&d->callback_work);

		/* wake sync waiter if any */
		k_sem_give(&d->sync_sem);
	}
}

/* Synchronous wrapper: starts async transfer and waits */
static int spi_wch_ch5xx_transceive(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	int rc;
#ifdef CONFIG_SPI_ASYNC
	struct spi_wch_ch5xx_data *d = dev->data;

	rc = spi_wch_ch5xx_configure(dev, config);
	if (rc) {
		return rc;
	}

	rc = spi_wch_ch5xx_transceive_async(dev, config, tx_bufs, rx_bufs);
	if (rc) {
		return rc;
	}

	/* wait for completion (timeout) */
	if (k_sem_take(&d->sync_sem, K_USEC(CH57X_SPI_DMA_TIMEOUT_US)) != 0) {
		/* timeout */
		atomic_set(&d->busy, 0);
		return -ETIMEDOUT;
	}
	return 0;
#else
	/* if async not enabled in build, implement a simple polling transceive (no DMA) */
	CH5XX_SPI_TypeDef *regs = SPI_REGS(dev);
	const struct spi_buf *txb = NULL, *rxb = NULL;
	size_t tx_len = 0, rx_len = 0, len, i;

	if (tx_bufs && tx_bufs->count) {
		txb = &tx_bufs->buffers[0];
		tx_len = txb->len;
	}
	if (rx_bufs && rx_bufs->count) {
		rxb = &rx_bufs->buffers[0];
		rx_len = rxb->len;
	}
	if (tx_len && rx_len) {
		len = max(tx_len, rx_len);
	} else {
		len = tx_len ? tx_len : rx_len;
	}

	for (i = 0; i < len; i++) {
		/* wait TXE, write, wait RXNE, read - replace bits by actual names */
		while (!(SPI_STATUS(regs) & (1 << 0))) {
		}
		SPI_BUFFER(regs) = txb ? ((const uint8_t *)txb->buf)[i] : 0xFFU;
		while (!(SPI_STATUS(regs) & (1 << 1))) {
		}
		uint16_t val = SPI_BUFFER(regs);
		if (rxb && i < rx_len) {
			((uint8_t *)rxb->buf)[i] = (uint8_t)val;
		}
	}
	return 0;
#endif
}

#ifdef CONFIG_SPI_ASYNC
/* Async transceive: set up context and start first pass (TX or RX) */
static int spi_wch_ch5xx_transceive_async(const struct device *dev, const struct spi_config *config,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs)
{
	struct spi_wch_ch5xx_data *d = dev->data;
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	const struct spi_buf *txb = NULL, *rxb = NULL;
	size_t tx_len = 0, rx_len = 0;
	int rc;

	/* simple single-fragment support */
	rc = extract_single_buf_set(tx_bufs, &txb, &tx_len);
	if (rc) {
		return rc;
	}
	rc = extract_single_buf_set(rx_bufs, &rxb, &rx_len);
	if (rc) {
		return rc;
	}

	/* lock */
	if (atomic_get(&d->busy)) {
		return -EBUSY;
	}
	atomic_set(&d->busy, 1);

	/* save buffers */
	d->tx_buf = txb ? txb->buf : NULL;
	d->rx_buf = rxb ? rxb->buf : NULL;
	d->tx_len = tx_len;
	d->rx_len = rx_len;

	/* next behavior:
	 * - tx_len && !rx_len -> TX-only: start TX DMA and finish on DMA_END
	 * - rx_len && !tx_len -> RX-only: start RX DMA and finish on DMA_END
	 * - tx_len && rx_len -> TX then RX: start TX DMA, ISR will chain to RX DMA
	 */
	if (tx_len && !rx_len) {
		d->tx_phase = true;
		/* start TX DMA */
		return spi_wch_ch5xx_start_dma(dev, d->tx_buf, d->tx_len, true);
	} else if (rx_len && !tx_len) {
		d->tx_phase = false;
		return spi_wch_ch5xx_start_dma(dev, d->rx_buf, d->rx_len, false);
	} else if (tx_len && rx_len) {
		/* TX then RX */
		d->tx_phase = true;
		/* start TX pass, ISR will start RX when TX DMA END occurs */
		return spi_wch_ch5xx_start_dma(dev, d->tx_buf, d->tx_len, true);
	}

	atomic_set(&d->busy, 0);
	return -EINVAL;
}

#endif /* CONFIG_SPI_ASYNC */

static int spi_wch_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_wch_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_wch_ch5xx_init(const struct device *dev)
{
	int err;
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;

	err = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/* Driver API binding */
static DEVICE_API(spi, spi_wch_ch5xx_driver_api) = {
	.transceive = spi_wch_ch5xx_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_wch_ch5xx_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_wch_release,
};

/* Device instantiation */
#define SPI_WCH_CH5XX_INIT(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct spi_wch_ch5xx_data spi_wch_ch5xx_data_##inst;                                \
	static const struct spi_wch_ch5xx_config spi_wch_ch5xx_cfg_##inst = {                      \
		.regs = (CH5XX_SPI_TypeDef *)DT_INST_REG_ADDR(inst),                               \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.irq_line = DT_INST_IRQN(inst),                                                    \
		.irq_priority = DT_INST_IRQ(inst, priority),                                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &spi_wch_ch5xx_init, NULL, &spi_wch_ch5xx_data_##inst,         \
			      &spi_wch_ch5xx_cfg_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
			      &spi_wch_ch5xx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_WCH_CH5XX_INIT)
