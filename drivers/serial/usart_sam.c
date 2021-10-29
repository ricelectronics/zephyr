/*
 * Copyright (c) 2018 Justin Watson
 * Copyright (c) 2016 Piotr Mienkowski
 * Copyright (c) 2021 RIC Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_usart

/** @file
 * @brief USART driver for Atmel SAM MCU family.
 *
 * Note:
 * - Only basic USART features sufficient to support printf functionality
 *   are currently implemented.
 */

#include <errno.h>
#include <device.h>
#include <devicetree.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>
#include <drivers/dma.h>

#define SAM_DMA_NODE DT_NODELABEL(xdmac)

#define LOG_LEVEL CONFIG_SERIAL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(usart_sam);

/* Device constant configuration parameters */
struct usart_sam_dev_cfg {
	Usart *regs;
	uint32_t periph_id;
	struct soc_gpio_pin pin_rx;
	struct soc_gpio_pin pin_tx;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API
	uart_irq_config_func_t	irq_config_func;
#endif
#if CONFIG_UART_ASYNC_API
	const struct device *dma_dev;
	uint8_t tx_dma_slot;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_slot;
	uint8_t rx_dma_channel;
#endif
};

/* Device run time data */
struct usart_sam_dev_data {
	uint32_t baud_rate;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API
	uart_irq_callback_user_data_t irq_cb;	/* Interrupt Callback */
	void *cb_data;	/* Interrupt Callback Arg */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_ASYNC_API
	const struct device *dev;
	const struct usart_sam_dev_cfg *cfg;

	struct dma_config dma_cfg_rx;
	struct dma_config dma_cfg_tx;

	struct dma_block_config dma_blk_rx;
	struct dma_block_config dma_blk_tx;

	uart_callback_t async_cb;
	void *async_cb_data;

	struct k_work_delayable tx_timeout_work;
	const uint8_t *tx_buf;
	size_t tx_len;

	uint8_t *rx_buf;
	size_t rx_len;
	size_t rx_processed_len;
	uint8_t *rx_next_buf;
	size_t rx_next_len;
	bool rx_waiting_for_irq;
	bool rx_timeout_from_isr;
	uint16_t timeout;
	bool rx_dma_active;
#endif
};

#define DEV_CFG(dev) \
	((const struct usart_sam_dev_cfg *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct usart_sam_dev_data *const)(dev)->data)


static int baudrate_set(Usart *const usart, uint32_t baudrate,
			uint32_t mck_freq_hz);

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API

static void _enable_rx_timeout(Usart *const usart, const struct usart_sam_dev_data *data, uint16_t ms)
{
	// Enable the reciever timeout interrupt with whatever timeout was programmed
	//US_RTOR / baudrate = timeout in seconds
	float count = data->baud_rate * ms / 1000;
	usart->US_RTOR = (uint16_t)(count + 0.5);
	usart->US_IER = US_IER_TIMEOUT;
	//For whatever reason this symbol is undefined: US_CR_STTTO;
	usart->US_CR = BIT(11);
}

static void _disable_rx_timeout(Usart * const usart)
{
	usart->US_IDR = US_IDR_TIMEOUT;
}

static void _enable_rx_int(Usart *const usart)
{
	// Enable the regular recieve IRQ
	// It will catch the first new transmission so DMA can be enabled again
	usart->US_IER = US_IER_RXRDY;
}

static void _disable_rx_int(Usart * const usart)
{
	usart->US_IDR = US_IDR_RXRDY;
}

#endif

#if CONFIG_UART_ASYNC_API

// This is called when a dma transfer is complete (all data has been transfered into the uart)
static void uart_sam_dma_tx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct usart_sam_dev_data *const dev_data =
		(struct usart_sam_dev_data *const) arg;

	dev_data->tx_len = 0;
}

static int uart_sam_tx_halt(struct usart_sam_dev_data *dev_data)
{
	const struct usart_sam_dev_cfg *const cfg = dev_data->cfg;
	int key = irq_lock();
	size_t tx_active = dev_data->tx_len;
	struct dma_status st;

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = dev_data->tx_buf,
			.len = 0U,
		},
	};

	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0U;

	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);

	irq_unlock(key);

	if (dma_get_status(cfg->dma_dev, cfg->tx_dma_channel, &st) == 0) {
		evt.data.tx.len = tx_active - st.pending_length;
	}

	if (tx_active) {
		if (dev_data->async_cb) {
			dev_data->async_cb(dev_data->dev,
					   &evt, dev_data->async_cb_data);
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static void uart_sam_tx_timeout(struct k_work *work)
{
	struct usart_sam_dev_data *dev_data = CONTAINER_OF(work,
							   struct usart_sam_dev_data, tx_timeout_work);

	uart_sam_tx_halt(dev_data);
}

static void uart_sam_notify_rx_processed(const struct device *dev)
{
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	struct dma_status st;
	size_t processed = 0;

	if (!dev_data->async_cb) {
		return;
	}


	if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel, &st) == 0
		&& st.pending_length != 0U) {
		processed = (dev_data->rx_len + dev_data->rx_processed_len)
			- st.pending_length;
	}

	if (dev_data->rx_processed_len == processed) {
		return;
	}

	dev_data->rx_processed_len = processed - dev_data->rx_processed_len;

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = dev_data->rx_buf,
			.offset = 0,
			.len = dev_data->rx_processed_len,
		},
	};

	LOG_DBG("rx proccessed count total: %d", dev_data->rx_processed_len);

	if (dev_data->rx_next_len) {
		// On suspending we want to switch to the next buffer if there is one available
		dev_data->rx_buf = dev_data->rx_next_buf;
		dev_data->rx_len = dev_data->rx_next_len;
		SCB_CleanInvalidateDCache_by_Addr(dev_data->rx_buf, 128);
		dev_data->rx_next_buf = NULL;
		dev_data->rx_next_len = 0U;
	} else {
		// otherwise just wipe the remaining buf out, a new one must be supplied
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0U;
	}

	dev_data->async_cb(dev_data->dev,
			   &evt, dev_data->async_cb_data);

	dev_data->rx_processed_len = 0;
}

static void uart_sam_dma_rx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	LOG_DBG("dma rx transfer complete");

	struct usart_sam_dev_data *const dev_data =
		(struct usart_sam_dev_data *const)arg;
	const struct device *dev = dev_data->dev;
	const struct usart_sam_dev_cfg *const cfg = dev_data->cfg;
	Usart *const usart = cfg->regs;
	int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	uart_sam_notify_rx_processed(dev);

	/* No next buffer, so end the transfer */
	if (!dev_data->rx_next_len) {
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0U;

		if (dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
		return;
	}

	dev_data->rx_buf = dev_data->rx_next_buf;
	dev_data->rx_len = dev_data->rx_next_len;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0U;
	dev_data->rx_processed_len = 0U;

	dma_reload(dma_dev, cfg->rx_dma_channel,
		   (uint32_t)(&(usart->US_RHR)),
		   (uint32_t)dev_data->rx_buf, dev_data->rx_len);

	/* Otherwise, start the transfer immediately. */
	dma_start(dma_dev, cfg->rx_dma_channel);

	irq_unlock(key);
}

#endif


static int usart_sam_init(const struct device *dev)
{
	int retval;
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	Usart *const usart = cfg->regs;

	/* Enable USART clock in PMC */
	soc_pmc_peripheral_enable(cfg->periph_id);

	/* Connect pins to the peripheral */
	soc_gpio_configure(&cfg->pin_rx);
	soc_gpio_configure(&cfg->pin_tx);

	/* Reset and disable USART */
	usart->US_CR =   US_CR_RSTRX | US_CR_RSTTX
		       | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

	/* Disable Interrupts */
	usart->US_IDR = 0xFFFFFFFF;

	/* 8 bits of data, no parity, 1 stop bit in normal mode */
	usart->US_MR =   US_MR_NBSTOP_1_BIT
		       | US_MR_PAR_NO
		       | US_MR_CHRL_8_BIT
		       | US_MR_USCLKS_MCK
		       | US_MR_CHMODE_NORMAL;

	/* Set baud rate */
	retval = baudrate_set(usart, dev_data->baud_rate,
			      SOC_ATMEL_SAM_MCK_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}

	/* Enable receiver and transmitter */
	usart->US_CR = US_CR_RXEN | US_CR_TXEN;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


#ifdef CONFIG_UART_ASYNC_API
	dev_data->dev = dev;
	dev_data->cfg = cfg;
	while (!device_is_ready(cfg->dma_dev)) {
		k_sleep(K_MSEC(1));
	}

	k_work_init_delayable(&dev_data->tx_timeout_work, uart_sam_tx_timeout);

	if (cfg->tx_dma_channel != 0xFFU) {

		dev_data->dma_blk_tx.block_size = 1;
		dev_data->dma_blk_tx.dest_address = (uint32_t)(&(usart->US_THR));
		dev_data->dma_blk_tx.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dev_data->dma_blk_tx.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		dev_data->dma_cfg_tx.channel_direction = MEMORY_TO_PERIPHERAL;
		dev_data->dma_cfg_tx.complete_callback_en = 0;
		dev_data->dma_cfg_tx.source_burst_length = 1;
		dev_data->dma_cfg_tx.dest_burst_length = 1;
		dev_data->dma_cfg_tx.source_data_size = 1;
		dev_data->dma_cfg_tx.dest_data_size = 1;
		dev_data->dma_cfg_tx.user_data = dev_data;
		dev_data->dma_cfg_tx.dma_callback = uart_sam_dma_tx_done;
		dev_data->dma_cfg_tx.block_count = 1;
		dev_data->dma_cfg_tx.head_block = &dev_data->dma_blk_tx;
		dev_data->dma_cfg_tx.dma_slot = cfg->tx_dma_slot;

		retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel,
				    &dev_data->dma_cfg_tx);
		if (retval != 0) {
			return retval;
		}
	}

	if (cfg->rx_dma_channel != 0xFFU) {
		dev_data->dma_blk_rx.block_size = 1;
		dev_data->dma_blk_rx.source_address = (uint32_t)(&(usart->US_RHR));
		dev_data->dma_blk_rx.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dev_data->dma_blk_rx.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		dev_data->dma_cfg_rx.channel_direction = PERIPHERAL_TO_MEMORY;
		dev_data->dma_cfg_rx.complete_callback_en = 0;
		dev_data->dma_cfg_rx.source_burst_length = 1;
		dev_data->dma_cfg_rx.dest_burst_length = 1;
		dev_data->dma_cfg_rx.source_data_size = 1;
		dev_data->dma_cfg_rx.dest_data_size = 1;
		dev_data->dma_cfg_rx.user_data = dev_data;
		dev_data->dma_cfg_rx.dma_callback = uart_sam_dma_rx_done;
		dev_data->dma_cfg_rx.block_count = 1;
		dev_data->dma_cfg_rx.head_block = &dev_data->dma_blk_rx;
		dev_data->dma_cfg_rx.dma_slot = cfg->rx_dma_slot;

		retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel,
				    &dev_data->dma_cfg_rx);
		if (retval != 0) {
			return retval;
		}
	}

#endif
	return 0;
}

static int usart_sam_poll_in(const struct device *dev, unsigned char *c)
{
	Usart *const usart = DEV_CFG(dev)->regs;

	if (!(usart->US_CSR & US_CSR_RXRDY)) {
		return -EBUSY;
	}

	/* got a character */
	*c = (unsigned char)usart->US_RHR;

	return 0;
}

static void usart_sam_poll_out(const struct device *dev, unsigned char c)
{
	Usart *const usart = DEV_CFG(dev)->regs;

	/* Wait for transmitter to be ready */
	while (!(usart->US_CSR & US_CSR_TXRDY)) {
	}

	/* send a character */
	usart->US_THR = (uint32_t)c;
}

static int usart_sam_err_check(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;
	int errors = 0;

	if (usart->US_CSR & US_CSR_OVRE) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (usart->US_CSR & US_CSR_PARE) {
		errors |= UART_ERROR_PARITY;
	}

	if (usart->US_CSR & US_CSR_FRAME) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}

static int baudrate_set(Usart *const usart, uint32_t baudrate,
			uint32_t mck_freq_hz)
{
	uint32_t divisor;

	__ASSERT(baudrate,
		 "baud rate has to be bigger than 0");
	__ASSERT(mck_freq_hz/16U >= baudrate,
		 "MCK frequency is too small to set required baud rate");

	divisor = mck_freq_hz / 16U / baudrate;

	if (divisor > 0xFFFF) {
		return -EINVAL;
	}

	usart->US_BRGR = US_BRGR_CD(divisor);

	return 0;
}

#if CONFIG_UART_INTERRUPT_DRIVEN
static int usart_sam_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int size)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	/* Wait for transmitter to be ready. */
	while ((usart->US_CSR & US_CSR_TXRDY) == 0) {
	}

	usart->US_THR = *tx_data;

	return 1;
}

static int usart_sam_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int size)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;
	int bytes_read;

	bytes_read = 0;

	while (bytes_read < size) {
		if (usart->US_CSR & US_CSR_RXRDY) {
			rx_data[bytes_read] = usart->US_RHR;
			bytes_read++;
		} else {
			break;
		}
	}

	return bytes_read;
}

static void usart_sam_irq_tx_enable(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	usart->US_IER = US_IER_TXRDY;
}

static void usart_sam_irq_tx_disable(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	usart->US_IDR = US_IDR_TXRDY;
}

static int usart_sam_irq_tx_ready(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	return (usart->US_CSR & US_CSR_TXRDY);
}

static void usart_sam_irq_rx_enable(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	usart->US_IER = US_IER_RXRDY;
}

static void usart_sam_irq_rx_disable(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	usart->US_IDR = US_IDR_RXRDY;
}

static int usart_sam_irq_tx_complete(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	return !(usart->US_CSR & US_CSR_TXRDY);
}

static int usart_sam_irq_rx_ready(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	return (usart->US_CSR & US_CSR_RXRDY);
}

static void usart_sam_irq_err_enable(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	usart->US_IER = US_IER_OVRE | US_IER_FRAME | US_IER_PARE;
}

static void usart_sam_irq_err_disable(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	usart->US_IDR = US_IDR_OVRE | US_IDR_FRAME | US_IDR_PARE;
}

static int usart_sam_irq_is_pending(const struct device *dev)
{
	volatile Usart * const usart = DEV_CFG(dev)->regs;

	return (usart->US_IMR & (US_IMR_TXRDY | US_IMR_RXRDY)) &
		(usart->US_CSR & (US_CSR_TXRDY | US_CSR_RXRDY));
}

static int usart_sam_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void usart_sam_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);

	dev_data->irq_cb = cb;
	dev_data->cb_data = cb_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API

static int _rx_dma_resume(const struct device *dev)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	Usart * const usart = DEV_CFG(dev)->regs;
	int retval;

	__ASSERT(!dev_data->rx_dma_active, "Should only be called if DMA is inactive!");

	if (cfg->rx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	__ASSERT(dev_data->timeout != 0, "fixme: logic bug, timeout cannot be 0");

	int key = irq_lock();

	if (!dev_data->rx_buf || !dev_data->rx_len) {
		return -ENOBUFS;
	}

	retval = dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
			    (uint32_t)(&(usart->US_RHR)),
			    (uint32_t)(&dev_data->rx_buf[dev_data->rx_processed_len]),
			    dev_data->rx_len-dev_data->rx_processed_len);
	if (retval != 0) {
		LOG_ERR("Failed to reload RX DMA");
		goto err;
	}

	retval = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
	if (retval != 0) {
		LOG_ERR("Failed to start RX DMA");
		goto err;
	}
	dev_data->rx_dma_active = true;

	irq_unlock(key);
	LOG_DBG("rx dma resume");
	return 0;

err:
	irq_unlock(key);
	return retval;
}

static int uart_sam_callback_set(const struct device *dev,
				  uart_callback_t callback,
				  void *user_data)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);

	dev_data->async_cb = callback;
	dev_data->async_cb_data = user_data;

	return 0;
}

static int uart_sam_tx(const struct device *dev, const uint8_t *buf,
			size_t len,
			int32_t timeout)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	Usart * const usart = DEV_CFG(dev)->regs;
	int retval;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	int key = irq_lock();

	if (dev_data->tx_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	// Adjust for block length
	dev_data->dma_blk_tx.block_size = len;
	dev_data->dma_blk_tx.source_address = (uint32_t)buf;

	dev_data->tx_buf = buf;
	dev_data->tx_len = len;

	irq_unlock(key);

	retval = dma_reload(cfg->dma_dev, cfg->tx_dma_channel, (uint32_t)buf,
			    (uint32_t)(&(usart->US_THR)), len);
	if (retval != 0U) {
		return retval;
	}

	if (timeout != SYS_FOREVER_MS) {
		k_work_reschedule(&dev_data->tx_timeout_work,
				      K_MSEC(timeout));
	}

	return dma_start(cfg->dma_dev, cfg->tx_dma_channel);
err:
	irq_unlock(key);
	return retval;
}

static int uart_sam_tx_abort(const struct device *dev)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	k_work_cancel_delayable(&dev_data->tx_timeout_work);

	return uart_sam_tx_halt(dev_data);
}

static int uart_sam_rx_enable(const struct device *dev, uint8_t *buf,
			       size_t len,
			       int32_t timeout)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	Usart * const usart = DEV_CFG(dev)->regs;
	int retval;

	if (dev_data->rx_dma_active) {
		LOG_WRN("rx DMA is already active");
		return 0;
	}

	if (cfg->rx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	if (timeout == 0) {
		return -EINVAL;
	}

	int key = irq_lock();

	if (dev_data->rx_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	retval = dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
			    (uint32_t)(&(usart->US_RHR)),
			    (uint32_t)buf, len);
	if (retval != 0) {
		LOG_ERR("Failed to reload rx DMA");
		goto err;
	}

	dev_data->rx_buf = buf;
	dev_data->rx_len = len;
	dev_data->rx_processed_len = 0U;

	_enable_rx_timeout(usart, dev_data, timeout);
	_disable_rx_int(usart);
	dev_data->timeout = timeout;

	retval = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
	if (retval != 0) {
		LOG_ERR("Failed to start RX DMA");
		goto err;
	}

	dev_data->rx_dma_active = true;

	irq_unlock(key);
	return 0;

err:
	irq_unlock(key);
	return retval;
}

static int uart_sam_rx_buf_rsp(const struct device *dev, uint8_t *buf,
				size_t len)
{
	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	int key = irq_lock();
	int retval = 0;

	if (dev_data->rx_len == 0U) {
		dev_data->rx_buf = buf;
		dev_data->rx_len = len;
	} else if (dev_data->rx_next_len != 0U) {
		dev_data->rx_next_buf = buf;
		dev_data->rx_next_len = len;
	} else {
		LOG_WRN("logic bug: no new buffer is required!");
	}

	irq_unlock(key);
	return 0;
}

static int _rx_dma_suspend(const struct device *dev)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	int retval = 0;

	__ASSERT(dev_data->rx_dma_active, "Should only be called if DMA is active!");

	int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return -EINVAL;
	}

	retval = dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
	if (retval != 0) {
		LOG_ERR("Failed to stop RX DMA");
		irq_unlock(key);
		return retval;
	}

	dev_data->rx_dma_active = false;

	uart_sam_notify_rx_processed(dev);

	struct uart_event evt = {
		.type = UART_RX_DISABLED,
	};
	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	irq_unlock(key);
	LOG_DBG("rx dma suspend");
	return 0;
}

static int uart_sam_rx_disable(const struct device *dev)
{
	return _rx_dma_suspend(dev);
}
#endif


#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API

static void usart_sam_isr(const struct device *dev)
{
	struct usart_sam_dev_data *const dev_data = DEV_DATA(dev);
#if CONFIG_UART_INTERRUPT_DRIVEN
	if (dev_data->irq_cb) {
		dev_data->irq_cb(dev, dev_data->cb_data);
	}
#endif
#if CONFIG_UART_ASYNC_API
	// First check which interrupt was triggered
	const struct usart_sam_dev_cfg *const cfg = DEV_CFG(dev);
	Usart *const usart = cfg->regs;

	if (usart->US_CSR & US_CSR_RXRDY) {
		/* On RX recieve enable DMA again */
		__ASSERT(dev_data->rx_processed_len == 0, "bug: this can only be the first character");

		int key = irq_lock();

		if (dev_data->rx_len) {
			dev_data->rx_buf[0] = usart->US_RHR & 0x0000000F;
			dev_data->rx_processed_len++;
		} else {
			LOG_WRN("No buffer to write rx data!");
		}

		int ret = _rx_dma_resume(dev);

		if (ret == -ENOBUFS || dev_data->rx_len == dev_data->rx_processed_len) {
			struct uart_event evt = {
				.type = UART_RX_BUF_REQUEST,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		} else {
			_disable_rx_int(usart);
			_enable_rx_timeout(usart, dev_data, dev_data->timeout);
		}

		irq_unlock(key);
		return;
	}

	if (usart->US_CSR & US_CSR_TIMEOUT) {
		/* On RX timeout disable DMA, and flush the bytes */
		int key = irq_lock();
		_disable_rx_timeout(usart);
		_rx_dma_suspend(dev);
		_enable_rx_int(usart);
		irq_unlock(key);
		return;
	}
	LOG_DBG("unknown ISR: %x:", usart->US_CSR);
	#endif
}
#endif

static const struct uart_driver_api usart_sam_driver_api = {
	.poll_in = usart_sam_poll_in,
	.poll_out = usart_sam_poll_out,
	.err_check = usart_sam_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_sam_fifo_fill,
	.fifo_read = usart_sam_fifo_read,
	.irq_tx_enable = usart_sam_irq_tx_enable,
	.irq_tx_disable = usart_sam_irq_tx_disable,
	.irq_tx_ready = usart_sam_irq_tx_ready,
	.irq_rx_enable = usart_sam_irq_rx_enable,
	.irq_rx_disable = usart_sam_irq_rx_disable,
	.irq_tx_complete = usart_sam_irq_tx_complete,
	.irq_rx_ready = usart_sam_irq_rx_ready,
	.irq_err_enable = usart_sam_irq_err_enable,
	.irq_err_disable = usart_sam_irq_err_disable,
	.irq_is_pending = usart_sam_irq_is_pending,
	.irq_update = usart_sam_irq_update,
	.irq_callback_set = usart_sam_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_ASYNC_API
	.callback_set = uart_sam_callback_set,
	.tx = uart_sam_tx,
	.tx_abort = uart_sam_tx_abort,
	.rx_enable = uart_sam_rx_enable,
	.rx_buf_rsp = uart_sam_rx_buf_rsp,
	.rx_disable = uart_sam_rx_disable,
#endif
};

#if CONFIG_UART_ASYNC_API
#define UART_SAM_DMA_CHANNELS(n)				\
	.dma_dev = DEVICE_DT_GET(SAM_DMA_NODE),			\
	.tx_dma_slot = DT_INST_PROP_OR(n, tx_dma_slot, 0),	\
	.tx_dma_channel = DT_INST_PROP_OR(n, tx_dma_ch, 0),	\
	.rx_dma_slot = DT_INST_PROP_OR(n, rx_dma_slot, 0),	\
	.rx_dma_channel = DT_INST_PROP_OR(n, rx_dma_ch, 0),
#else
#define UART_SAM_DMA_CHANNELS(n)
#endif


#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API
#define USART_SAM_IRQ_CFG_FUNC(n)					\
static void usart##n##_sam_irq_config_func(const struct device *port)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(n),					\
			DT_INST_IRQ(n, priority),			\
			usart_sam_isr,					\
			DEVICE_DT_INST_GET(n), 0);			\
	irq_enable(DT_INST_IRQN(n));					\
}

#define USART_SAM_IRQ_CFG_FUNC_INIT(n)					\
	.irq_config_func = usart##n##_sam_irq_config_func
#else
#define USART_SAM_IRQ_CFG_FUNC(n)
#define USART_SAM_IRQ_CFG_FUNC_INIT(n)
#endif

#define USART_SAM_DECLARE_CFG(n)					\
	static const struct usart_sam_dev_cfg usart##n##_sam_config = {	\
		.regs = (Usart *)DT_INST_REG_ADDR(n),			\
		.periph_id = DT_INST_PROP(n, peripheral_id),		\
		.pin_rx = ATMEL_SAM_DT_INST_PIN(n, 0),			\
		.pin_tx = ATMEL_SAM_DT_INST_PIN(n, 1),			\
		UART_SAM_DMA_CHANNELS(n)				\
		USART_SAM_IRQ_CFG_FUNC_INIT(n)				\
	};

#define USART_SAM_INIT(n)						\
									\
	USART_SAM_IRQ_CFG_FUNC(n)					\
									\
	static struct usart_sam_dev_data usart##n##_sam_data = {	\
		.baud_rate = DT_INST_PROP(n, current_speed),		\
		.timeout = 0						\
	};								\
									\
	USART_SAM_DECLARE_CFG(n)					\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			    &usart_sam_init, NULL,			\
			    &usart##n##_sam_data,			\
			    &usart##n##_sam_config, PRE_KERNEL_1,	\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &usart_sam_driver_api);

DT_INST_FOREACH_STATUS_OKAY(USART_SAM_INIT)
