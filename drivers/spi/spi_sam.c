/*
 * Copyright (c) 2017 Google LLC.
 * Copyright (c) 2018 qianfan Zhao.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_sam);

#include "spi_context.h"
#include <errno.h>
#include <device.h>
#include <drivers/spi.h>
#ifdef CONFIG_SPI_SAM_DMA
#include <drivers/dma.h>
#endif
#include <soc.h>

#define SAM_SPI_CHIP_SELECT_COUNT			4

#ifdef CONFIG_SPI_SAM_DMA
#define SAM_SPI_TX_DMA	0
#define SAM_SPI_RX_DMA	1
#endif

/* Device constant configuration parameters */
struct spi_sam_config {
	Spi *regs;
	uint32_t periph_id;
	uint32_t num_pins;
	#ifdef CONFIG_SPI_SAM_DMA
	const struct device *dma[2];
	uint8_t dma_channel[2];
	uint8_t dma_slot[2];
	#endif
	struct soc_gpio_pin pins[];
};

/* Device run time data */
struct spi_sam_data {
	struct spi_context 	ctx;
	#ifdef CONFIG_SPI_SAM_DMA
	struct k_sem		dma_sem[2];
	struct dma_config 	dma_cfg[2];
	struct dma_block_config dma_blk[2];
	const struct device 	*dma_dev[2];
	uint8_t 		dma_channel[2];
	bool			dma_en[2];
	#endif
};

static int spi_slave_to_mr_pcs(int slave)
{
	int pcs[SAM_SPI_CHIP_SELECT_COUNT] = {0x0, 0x1, 0x3, 0x7};

	/* SPI worked in fixed perieral mode(SPI_MR.PS = 0) and disabled chip
	 * select decode(SPI_MR.PCSDEC = 0), based on Atmel | SMART ARM-based
	 * Flash MCU DATASHEET 40.8.2 SPI Mode Register:
	 * PCS = xxx0    NPCS[3:0] = 1110
	 * PCS = xx01    NPCS[3:0] = 1101
	 * PCS = x011    NPCS[3:0] = 1011
	 * PCS = 0111    NPCS[3:0] = 0111
	 */

	return pcs[slave];
}

#ifdef CONFIG_SPI_SAM_DMA
int spi_sam_config_dma_xfer(struct spi_sam_data *data, int8_t inst, uint8_t *buffer, uint32_t len) {

	if (inst != SAM_SPI_TX_DMA && inst != SAM_SPI_RX_DMA) {
		LOG_ERR("invalid DMA instance");
		return -EINVAL;
	}

	if (!buffer) {
		LOG_ERR("buffer cannot be NULL");
		return -EINVAL;
	}

	if (!data->dma_en[inst]) {
		LOG_ERR("DMA is not enabled, cannot configure buffer");
		return -ENODEV;
	}

	data->dma_blk[inst].block_size = len;
	if (inst == SAM_SPI_RX_DMA) {
		data->dma_blk[inst].dest_address = (uint32_t)buffer;
	} else if (inst == SAM_SPI_TX_DMA) {
		data->dma_blk[inst].source_address = (uint32_t)buffer;
	}
	return dma_config(data->dma_dev[inst], data->dma_channel[inst], &data->dma_cfg[inst]);
}
#endif

static int spi_sam_configure(const struct device *dev,
			     const struct spi_config *config)
{
	const struct spi_sam_config *cfg = dev->config;
	struct spi_sam_data *data = dev->data;
	Spi *regs = cfg->regs;
	uint32_t spi_mr = 0U, spi_csr = 0U;
	int div;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		/* Slave mode is not implemented. */
		return -ENOTSUP;
	}

	if (config->slave > (SAM_SPI_CHIP_SELECT_COUNT - 1)) {
		LOG_ERR("Slave %d is greater than %d",
			config->slave, SAM_SPI_CHIP_SELECT_COUNT - 1);
		return -EINVAL;
	}

	/* Set master mode, disable mode fault detection, set fixed peripheral
	 * select mode.
	 */
	spi_mr |= (SPI_MR_MSTR | SPI_MR_MODFDIS);
	spi_mr |= SPI_MR_PCS(spi_slave_to_mr_pcs(config->slave));

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		spi_csr |= SPI_CSR_CPOL;
	}

	if ((config->operation & SPI_MODE_CPHA) == 0U) {
		spi_csr |= SPI_CSR_NCPHA;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		return -ENOTSUP;
	} else {
		spi_csr |= SPI_CSR_BITS(SPI_CSR_BITS_8_BIT);
	}

	/* Use the requested or next highest possible frequency */
	div = SOC_ATMEL_SAM_MCK_FREQ_HZ / config->frequency;
	div = CLAMP(div, 1, UINT8_MAX);
	spi_csr |= SPI_CSR_SCBR(div);

	regs->SPI_CR = SPI_CR_SPIDIS; /* Disable SPI */
	regs->SPI_MR = spi_mr;
	regs->SPI_CSR[config->slave] = spi_csr;
	regs->SPI_CR = SPI_CR_SPIEN; /* Enable SPI */

	data->ctx.config = config;
	spi_context_cs_configure(&data->ctx);

	return 0;
}

static bool spi_sam_transfer_ongoing(struct spi_sam_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static void spi_sam_shift_master(Spi *regs, struct spi_sam_data *data)
{
	uint8_t tx;
	uint8_t rx;

	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(uint8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0U;
	}

	while ((regs->SPI_SR & SPI_SR_TDRE) == 0) {
	}

	regs->SPI_TDR = SPI_TDR_TD(tx);
	spi_context_update_tx(&data->ctx, 1, 1);

	while ((regs->SPI_SR & SPI_SR_RDRF) == 0) {
	}

	rx = (uint8_t)regs->SPI_RDR;

	if (spi_context_rx_buf_on(&data->ctx)) {
		*data->ctx.rx_buf = rx;
	}
	spi_context_update_rx(&data->ctx, 1, 1);
}

/* Finish any ongoing writes and drop any remaining read data */
static void spi_sam_finish(Spi *regs)
{
	while ((regs->SPI_SR & SPI_SR_TXEMPTY) == 0) {
	}

	while (regs->SPI_SR & SPI_SR_RDRF) {
		(void)regs->SPI_RDR;
	}
}

/* Fast path that transmits a buf */
static void spi_sam_fast_tx(const struct device *dev, const struct spi_buf *tx_buf)
{
	const struct spi_sam_config *cfg = dev->config;
	//struct spi_sam_data *data = dev->data;
	Spi *regs = cfg->regs;

	/* Note: using DMA for TX transfers does not work, there is some kind of bug with witht he chip
	 * if (!data->dma_en[SAM_SPI_TX_DMA] || tx_buf->len == 1) {
 	 */
		const uint8_t *p = tx_buf->buf;
		const uint8_t *pend = (uint8_t *)tx_buf->buf + tx_buf->len;
		uint8_t ch;

		while (p != pend) {
			ch = *p++;

			while ((regs->SPI_SR & SPI_SR_TDRE) == 0) {
			}

			regs->SPI_TDR = SPI_TDR_TD(ch);
		}
		spi_sam_finish(regs);
	/*} else {

		SCB_CleanDCache_by_Addr(tx_buf->buf, tx_buf->len);
		int res = spi_sam_config_dma_xfer(data, SAM_SPI_TX_DMA, tx_buf->buf, tx_buf->len);
		if (0 > res) {
			LOG_ERR("(TX) config TX DMA failed %d", res);
			return;
		}

		res = dma_start(data->dma_dev[SAM_SPI_TX_DMA], data->dma_channel[SAM_SPI_TX_DMA]);
		if (0 > res) {
			LOG_ERR("starting TX DMA failed %d", res);
			return;
		}

		res = k_sem_take(&data->dma_sem[SAM_SPI_TX_DMA], K_MSEC(10));
		SCB_InvalidateDCache_by_Addr(tx_buf->buf, tx_buf->len);
		if (0 > res) {
			LOG_ERR("waiting for DMA timeout %d", res);
			dma_stop(data->dma_dev[SAM_SPI_TX_DMA], data->dma_channel[SAM_SPI_TX_DMA]);
		}
	}*/
}

static int spi_sam_wait_for_rx(Spi *regs) {
	uint32_t flags = 0;
	while (1) {
		flags = regs->SPI_SR & (SPI_SR_RDRF | SPI_SR_OVRES);
		if (flags != 0) {
			break;
		}
	}

	if (flags & SPI_SR_OVRES) {
		LOG_WRN("RX buffer overflow");
		return -EIO;
	}

	return 0;
}
/* Fast path that reads into a buf */
static int spi_sam_fast_rx(const struct device *dev, const struct spi_buf *rx_buf)
{
	const struct spi_sam_config *cfg = dev->config;
	struct spi_sam_data *data = dev->data;
	Spi *regs = cfg->regs;
	uint8_t *rx = rx_buf->buf;
	int len = rx_buf->len;
	int res = 0;

	/* See the comment in spi_sam_fast_txrx re: interleaving. */
	#ifdef CONFIG_SPI_SAM_DMA
	if (!data->dma_en[SAM_SPI_RX_DMA]) {
	#endif
		/* Write the first byte */
		regs->SPI_TDR = SPI_TDR_TD(0);
		len--;

		if (len <= 0) {
			return -ENOBUFS;
		}

		while (len) {
			while ((regs->SPI_SR & SPI_SR_TDRE) == 0) {
			}

			/* Load byte N+1 into the transmit register */
			regs->SPI_TDR = SPI_TDR_TD(0);
			len--;

			/* Read byte N+0 from the receive register */
			res = spi_sam_wait_for_rx(regs);
			if (0 > res) {
				goto done;
			}
			*rx++ = (uint8_t)regs->SPI_RDR;
		}

		/* Read the final incoming byte */
		res = spi_sam_wait_for_rx(regs);
		if (0 > res) {
			goto done;
		}
		*rx = (uint8_t)regs->SPI_RDR;
	#ifdef CONFIG_SPI_SAM_DMA
	} else {
		SCB_CleanDCache_by_Addr(rx_buf->buf, rx_buf->len);
		uint8_t zero = 0;
		res = spi_sam_config_dma_xfer(data, SAM_SPI_TX_DMA, &zero, rx_buf->len);
		if (0 > res) {
			LOG_ERR("(RX) config TX DMA failed %d", res);
			return res;
		}

		res = spi_sam_config_dma_xfer(data, SAM_SPI_RX_DMA, rx_buf->buf, rx_buf->len);
		if (0 > res) {
			LOG_ERR("config RX DMA failed %d", res);
			return res;
		}

		res = dma_start(data->dma_dev[SAM_SPI_TX_DMA], data->dma_channel[SAM_SPI_TX_DMA]);
		if (0 > res) {
			LOG_ERR("starting TX DMA failed %d", res);
			return res;
		}

		res = dma_start(data->dma_dev[SAM_SPI_RX_DMA], data->dma_channel[SAM_SPI_RX_DMA]);
		if (0 > res) {
			LOG_ERR("starting RX DMA failed %d", res);
			return res;
		}

		// wait for dma complete signal here
		res = k_sem_take(&data->dma_sem[SAM_SPI_TX_DMA], K_MSEC(10));
		if (0 > res) {
			LOG_ERR("waiting for DMA timeout %d", res);
			dma_stop(data->dma_dev[SAM_SPI_TX_DMA], data->dma_channel[SAM_SPI_TX_DMA]);
		}

		res = k_sem_take(&data->dma_sem[SAM_SPI_RX_DMA], K_MSEC(10));
		if (0 > res) {
			LOG_ERR("waiting for DMA timeout %d", res);
			dma_stop(data->dma_dev[SAM_SPI_RX_DMA], data->dma_channel[SAM_SPI_RX_DMA]);
		}
		SCB_InvalidateDCache_by_Addr(rx_buf->buf, rx_buf->len);
		return res;
	}
	#endif

done:
	spi_sam_finish(regs);
	return res;
}

/* Fast path that writes and reads bufs of the same length */
static int spi_sam_fast_txrx(const struct device *dev,
			      const struct spi_buf *tx_buf,
			      const struct spi_buf *rx_buf)
{
	const struct spi_sam_config *cfg = dev->config;
	struct spi_sam_data *data = dev->data;
	Spi *regs = cfg->regs;

	const uint8_t *tx = tx_buf->buf;
	const uint8_t *txend = (uint8_t *)tx_buf->buf + tx_buf->len;
	uint8_t *rx = rx_buf->buf;
	size_t len = rx_buf->len;
	int res;

	if (len == 0) {
		return -ENOBUFS;
	}

	/*
	 * The code below interleaves the transmit writes with the
	 * receive reads to keep the bus fully utilised.  The code is
	 * equivalent to:
	 *
	 * Transmit byte 0
	 * Loop:
	 * - Transmit byte n+1
	 * - Receive byte n
	 * Receive the final byte
	 */

	/* Write the first byte */
	regs->SPI_TDR = SPI_TDR_TD(*tx++);

	#ifdef CONFIG_SPI_SAM_DMA
	if (!data->dma_en[SAM_SPI_RX_DMA]) {
	#endif
		while (tx != txend) {
			while ((regs->SPI_SR & SPI_SR_TDRE) == 0) {
			}

			/* Load byte N+1 into the transmit register.  TX is
			* single buffered and we have at most one byte in
			* flight so skip the DRE check.
			*/
			regs->SPI_TDR = SPI_TDR_TD(*tx++);

			/* Read byte N+0 from the receive register */
			res = spi_sam_wait_for_rx(regs);
			if (0 > res) {
				goto done;
			}
			*rx++ = (uint8_t)regs->SPI_RDR;
		}

		/* Read the final incoming byte */
		res = spi_sam_wait_for_rx(regs);
		if (0 > res) {
			goto done;
		}
		*rx = (uint8_t)regs->SPI_RDR;
	#ifdef CONFIG_SPI_SAM_DMA
	} else {
		/* This is untested but should work */
		SCB_CleanDCache_by_Addr(rx_buf->buf, rx_buf->len);

		int res = spi_sam_config_dma_xfer(data, SAM_SPI_RX_DMA, rx_buf->buf, rx_buf->len);
		if (0 > res) {
			LOG_ERR("config RX DMA failed %d", res);
			return res;
		}

		res = dma_start(data->dma_dev[SAM_SPI_RX_DMA], data->dma_channel[SAM_SPI_RX_DMA]);
		if (0 > res) {
			LOG_ERR("starting RX DMA failed %d", res);
			return res;
		}

		while (tx != txend) {
			while ((regs->SPI_SR & SPI_SR_TDRE) == 0) {}

			/* Load byte N+1 into the transmit register.  TX is
			* single buffered and we have at most one byte in
			* flight so skip the DRE check.
			*/
			regs->SPI_TDR = SPI_TDR_TD(*tx++);
		}

		res = k_sem_take(&data->dma_sem[SAM_SPI_RX_DMA], K_MSEC(10));
		if (0 > res) {
			LOG_ERR("waiting for DMA timeout %d", res);
			dma_stop(data->dma_dev[SAM_SPI_RX_DMA], data->dma_channel[SAM_SPI_RX_DMA]);
		}
		SCB_InvalidateDCache_by_Addr(rx_buf->buf, rx_buf->len);
		return res;
	}
	#endif
done:
	spi_sam_finish(regs);
	return res;
}

/* Fast path where every overlapping tx and rx buffer is the same length */
static int spi_sam_fast_transceive(const struct device *dev,
				    const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;

	if (tx_bufs) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	while (tx_count != 0 && rx_count != 0) {
		if (tx->buf == NULL) {
			int ret = spi_sam_fast_rx(dev, rx);
			if (ret != 0) {
				LOG_WRN("(fast_rx) timeout while reading");
				return ret;
			}
		} else if (rx->buf == NULL) {
			spi_sam_fast_tx(dev, tx);
		} else {
			int ret = spi_sam_fast_txrx(dev, tx, rx);
			if (ret != 0) {
				LOG_WRN("(fast_txrx) timeout while reading");
				return ret;
			}
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	for (; tx_count != 0; tx_count--) {
		spi_sam_fast_tx(dev, tx++);
	}

	for (; rx_count != 0; rx_count--) {
		int ret = spi_sam_fast_rx(dev, rx++);
		if (ret != 0) {
			LOG_WRN("(fast_rx) timeout while reading");
			return ret;
		}
	}
	return 0;
}

/* Returns true if the request is suitable for the fast
 * path. Specifically, the bufs are a sequence of:
 *
 * - Zero or more RX and TX buf pairs where each is the same length.
 * - Zero or more trailing RX only bufs
 * - Zero or more trailing TX only bufs
 */
static bool spi_sam_is_regular(const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;
	size_t tx_count = 0;
	size_t rx_count = 0;

	if (tx_bufs) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	if (!tx || !rx) {
		return true;
	}

	while (tx_count != 0 && rx_count != 0) {
		if (tx->len != rx->len) {
			return false;
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	return true;
}

static int spi_sam_transceive(const struct device *dev,
			      const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs,
			      const struct spi_buf_set *rx_bufs)
{
	const struct spi_sam_config *cfg = dev->config;
	struct spi_sam_data *data = dev->data;
	Spi *regs = cfg->regs;
	int err;

	spi_context_lock(&data->ctx, false, NULL, config);

	err = spi_sam_configure(dev, config);
	if (err != 0) {
		goto done;
	}

	spi_context_cs_control(&data->ctx, true);

	/* This driver special cases the common send only, receive
	 * only, and transmit then receive operations.	This special
	 * casing is 4x faster than the spi_context() routines
	 * and allows the transmit and receive to be interleaved.
	 */
	if (spi_sam_is_regular(tx_bufs, rx_bufs)) {
		err = spi_sam_fast_transceive(dev, config, tx_bufs, rx_bufs);
		if (err != 0) {
			spi_context_cs_control(&data->ctx, false);
			goto done;
		}
	} else {
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

		do {
			spi_sam_shift_master(regs, data);
		} while (spi_sam_transfer_ongoing(data));
	}

	spi_context_cs_control(&data->ctx, false);

done:
	spi_context_release(&data->ctx, err);
	return err;
}

static int spi_sam_transceive_sync(const struct device *dev,
				    const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	return spi_sam_transceive(dev, config, tx_bufs, rx_bufs);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_sam_transceive_async(const struct device *dev,
				     const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	/* TODO: implement asyc transceive */
	return -ENOTSUP;
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_sam_release(const struct device *dev,
			   const struct spi_config *config)
{
	struct spi_sam_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_SAM_DMA
static void spi_sam_dma_callback(const struct device *dma, void *callback_arg,
			     uint32_t channel, int error_code)
{
	struct device *dev = (struct device *)callback_arg;
	struct spi_sam_data *data = dev->data;

	if (channel == data->dma_channel[SAM_SPI_RX_DMA]) {
		k_sem_give(&data->dma_sem[SAM_SPI_RX_DMA]);
	} else if (channel == data->dma_channel[SAM_SPI_TX_DMA]) {
		k_sem_give(&data->dma_sem[SAM_SPI_TX_DMA]);
	} else {
		LOG_WRN("unknown DMA channel %d", channel);
	}

	if (error_code != 0) {
		if (channel == data->dma_channel[SAM_SPI_RX_DMA]) {
			LOG_WRN("error on RX DMA %d", error_code);
		} else if (channel == data->dma_channel[SAM_SPI_TX_DMA]) {
			LOG_WRN("error on TX DMA %d", error_code);
		}
	}
}
#endif

static int spi_sam_init(const struct device *dev)
{
	const struct spi_sam_config *cfg = dev->config;
	struct spi_sam_data *data = dev->data;
	Spi *regs = cfg->regs;

	soc_pmc_peripheral_enable(cfg->periph_id);

	soc_gpio_list_configure(cfg->pins, cfg->num_pins);

	spi_context_unlock_unconditionally(&data->ctx);

	/* The device will be configured and enabled when transceive
	 * is called.
	 */

#ifdef CONFIG_SPI_SAM_DMA
	data->dma_en[SAM_SPI_TX_DMA] = false;
	data->dma_en[SAM_SPI_RX_DMA] = false;

	int8_t tx_idx = -1;
	int8_t rx_idx = -1;
	if (cfg->dma[0]) {
		LOG_DBG("dma[0] channel is configured for %s", dev->name);
		if (cfg->dma_slot[0] == DMA_PERID_SPI0_TX || cfg->dma_slot[0] == DMA_PERID_SPI1_TX) {
			LOG_DBG("dma[0] channel is configured for TX");
			tx_idx = 0;
		} else if (cfg->dma_slot[0] == DMA_PERID_SPI0_RX || cfg->dma_slot[0] == DMA_PERID_SPI1_RX) {
			LOG_DBG("dma[0] channel is configured for RX");
			rx_idx = 0;
		}
	}

	if (cfg->dma[1]) {
		LOG_DBG("dma[1] channel is configured for %s", dev->name);
		if (cfg->dma_slot[1] == DMA_PERID_SPI0_TX || cfg->dma_slot[1] == DMA_PERID_SPI1_TX) {
			LOG_DBG("dma[1] channel is configured for TX");
			tx_idx = 1;
		} else if (cfg->dma_slot[1] == DMA_PERID_SPI0_RX || cfg->dma_slot[1] == DMA_PERID_SPI1_RX) {
			LOG_DBG("dma[1] channel is configured for RX");
			rx_idx = 1;
		}
	}

	if (rx_idx != -1) {
		struct dma_block_config dma_blk = {
			.source_address = (uint32_t)&(regs->SPI_RDR),
			.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
			.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT
		};
		data->dma_blk[SAM_SPI_RX_DMA] = dma_blk;

		struct dma_config dma_cfg = {
			.dma_slot = cfg->dma_slot[rx_idx],
			.channel_direction = PERIPHERAL_TO_MEMORY,
			.complete_callback_en = 1,
			.error_callback_en = 1,
			.source_data_size = 1,
			.dest_data_size = 1,
			.source_burst_length = 1,
			.dest_burst_length = 1,
			.block_count = 1,
			.head_block = &data->dma_blk[SAM_SPI_RX_DMA],
			.user_data = (void *)dev,
			.dma_callback = spi_sam_dma_callback,
		};

		data->dma_cfg[SAM_SPI_RX_DMA] = dma_cfg;
		data->dma_dev[SAM_SPI_RX_DMA] = cfg->dma[rx_idx];
		data->dma_channel[SAM_SPI_RX_DMA] = cfg->dma_channel[rx_idx];
		k_sem_init(&data->dma_sem[SAM_SPI_RX_DMA], 0, 1);
		data->dma_en[SAM_SPI_RX_DMA] = true;
	}

	if (tx_idx != -1) {
		// Note: There seems to be a bug with with SPI TX DMA channel
		struct dma_block_config dma_blk = {
			.dest_address = (uint32_t)&(regs->SPI_TDR),
			.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
			.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE
		};
		data->dma_blk[SAM_SPI_TX_DMA] = dma_blk;

		struct dma_config dma_cfg = {
			.dma_slot = cfg->dma_slot[tx_idx],
			.channel_direction = MEMORY_TO_PERIPHERAL,
			.complete_callback_en = 1,
			.error_callback_en = 1,
			.source_data_size = 1,
			.dest_data_size = 1,
			.source_burst_length = 1,
			.dest_burst_length = 1,
			.block_count = 1,
			.head_block = &data->dma_blk[SAM_SPI_TX_DMA],
			.user_data = (void *)dev,
			.dma_callback = spi_sam_dma_callback,
		};

		data->dma_cfg[SAM_SPI_TX_DMA] = dma_cfg;
		data->dma_dev[SAM_SPI_TX_DMA] = cfg->dma[tx_idx];
		data->dma_channel[SAM_SPI_TX_DMA] = cfg->dma_channel[tx_idx];

		k_sem_init(&data->dma_sem[SAM_SPI_TX_DMA], 0, 1);
		data->dma_en[SAM_SPI_TX_DMA] = true;
	}
#endif

	return 0;
}

static const struct spi_driver_api spi_sam_driver_api = {
	.transceive = spi_sam_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_sam_transceive_async,
#endif
	.release = spi_sam_release,
};

#ifdef CONFIG_SPI_SAM_DMA
#define SPI_SAM_GET_DMA(inst, idx)						\
	COND_CODE_0(DT_INST_PROP_OR(inst, dmas, idx),				\
	NULL,									\
	(.dma[idx] = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_IDX(inst, idx)),	\
	.dma_channel[idx] = DT_INST_DMAS_CELL_BY_IDX(inst, idx, channel),	\
	.dma_slot[idx] = DT_INST_DMAS_CELL_BY_IDX(inst, idx, perid)))		\

#else
#define SPI_SAM_GET_DMA(inst, idx)
#endif

#ifdef CONFIG_SPI_SAM_DMA
#define SPI_SAM_DEFINE_CONFIG(n)					\
	static const struct spi_sam_config spi_sam_config_##n = {	\
		.regs = (Spi *)DT_INST_REG_ADDR(n),			\
		.periph_id = DT_INST_PROP(n, peripheral_id),		\
		.num_pins = ATMEL_SAM_DT_INST_NUM_PINS(n),		\
		SPI_SAM_GET_DMA(n, 0),					\
		SPI_SAM_GET_DMA(n, 1),					\
		.pins = ATMEL_SAM_DT_INST_PINS(n),			\
	}

#else
#define SPI_SAM_DEFINE_CONFIG(n)					\
	static const struct spi_sam_config spi_sam_config_##n = {	\
		.regs = (Spi *)DT_INST_REG_ADDR(n),			\
		.periph_id = DT_INST_PROP(n, peripheral_id),		\
		.num_pins = ATMEL_SAM_DT_INST_NUM_PINS(n),		\
		.pins = ATMEL_SAM_DT_INST_PINS(n),			\
	}

#endif

#define SPI_SAM_DEVICE_INIT(n)						\
	SPI_SAM_DEFINE_CONFIG(n);					\
	static struct spi_sam_data spi_sam_dev_data_##n = {		\
		SPI_CONTEXT_INIT_LOCK(spi_sam_dev_data_##n, ctx),	\
		SPI_CONTEXT_INIT_SYNC(spi_sam_dev_data_##n, ctx),	\
	};								\
	DEVICE_DT_INST_DEFINE(n, &spi_sam_init, NULL,			\
			    &spi_sam_dev_data_##n,			\
			    &spi_sam_config_##n, POST_KERNEL,		\
			    CONFIG_SPI_INIT_PRIORITY, &spi_sam_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SAM_DEVICE_INIT)
