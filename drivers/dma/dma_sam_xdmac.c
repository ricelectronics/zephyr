/*
 * Copyright (c) 2017 comsuisse AG
 * Copyright (c) 2021 RIC Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_xdmac

/** @file
 * @brief Atmel SAM MCU family Direct Memory Access (XDMAC) driver.
 */

#include <errno.h>
#include <sys/__assert.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <soc.h>
#include <drivers/dma.h>
#include "dma_sam_xdmac.h"

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(dma_sam_xdmac);

#define XDMAC_INT_ERR (XDMAC_CIE_RBIE | XDMAC_CIE_WBIE | XDMAC_CIE_ROIE)
#define DMA_CHANNELS_NO  XDMACCHID_NUMBER

/* DMA channel configuration */
struct sam_xdmac_channel_cfg {
	void *user_data;
	dma_callback_t callback;
	uint32_t data_size;
	uint32_t block_size;
	uint32_t start_addr_dest;
	uint32_t start_addr_source;
	struct sam_xdmac_linked_list_desc_view1 block[2];
	uint32_t channel_direction;
};

/* Device constant configuration parameters */
struct sam_xdmac_dev_cfg {
	Xdmac *regs;
	void (*irq_config)(void);
	uint8_t periph_id;
	uint8_t irq_id;
};

/* Device run time data */
struct sam_xdmac_dev_data {
	struct sam_xdmac_channel_cfg dma_channels[DMA_CHANNELS_NO];
};

#define DEV_NAME(dev) ((dev)->name)
#define DEV_CFG(dev) \
	((const struct sam_xdmac_dev_cfg *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct sam_xdmac_dev_data *const)(dev)->data)

static void sam_xdmac_isr(const struct device *dev)
{
	const struct sam_xdmac_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct sam_xdmac_dev_data *const dev_data = DEV_DATA(dev);
	Xdmac *const xdmac = dev_cfg->regs;
	struct sam_xdmac_channel_cfg *channel_cfg;
	uint32_t isr_status;
	uint32_t err;

	/* Get global interrupt status */
	isr_status = xdmac->XDMAC_GIS;

	for (int channel = 0; channel < DMA_CHANNELS_NO; channel++) {
		if (!(isr_status & (1 << channel))) {
			continue;
		}

		channel_cfg = &dev_data->dma_channels[channel];

		/* Get channel errors */
		err = xdmac->XDMAC_CHID[channel].XDMAC_CIS & XDMAC_INT_ERR;

		/* Execute callback */
		if (channel_cfg->callback) {
			channel_cfg->callback(dev, channel_cfg->user_data,
					      channel, err);
		}
	}
}

int sam_xdmac_channel_configure(const struct device *dev, uint32_t channel,
				struct sam_xdmac_channel_config *param)
{
	const struct sam_xdmac_dev_cfg *const dev_cfg = DEV_CFG(dev);
	Xdmac *const xdmac = dev_cfg->regs;

	if (channel >= DMA_CHANNELS_NO) {
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (xdmac->XDMAC_GS & (XDMAC_GS_ST0 << channel)) {
		return -EBUSY;
	}

	/* Disable all channel interrupts */
	xdmac->XDMAC_CHID[channel].XDMAC_CID = 0xFF;
	/* Clear pending Interrupt Status bit(s) */
	(void)xdmac->XDMAC_CHID[channel].XDMAC_CIS;

	/* NOTE:
	 * Setting channel configuration is not required for linked list view 2
	 * to 3 modes. It is done anyway to keep the code simple. It has no
	 * negative impact on the DMA functionality.
	 */

	/* Set channel configuration */
	xdmac->XDMAC_CHID[channel].XDMAC_CC = param->cfg;

	/* Set data stride memory pattern */
	xdmac->XDMAC_CHID[channel].XDMAC_CDS_MSP = param->ds_msp;
	/* Set source microblock stride */
	xdmac->XDMAC_CHID[channel].XDMAC_CSUS = param->sus;
	/* Set destination microblock stride */
	xdmac->XDMAC_CHID[channel].XDMAC_CDUS = param->dus;

	/* Enable selected channel interrupts */
	xdmac->XDMAC_CHID[channel].XDMAC_CIE = param->cie;

	return 0;
}

int sam_xdmac_transfer_configure(const struct device *dev, uint32_t channel,
				 struct sam_xdmac_transfer_config *param)
{
	const struct sam_xdmac_dev_cfg *const dev_cfg = DEV_CFG(dev);
	Xdmac *const xdmac = dev_cfg->regs;

	if (channel >= DMA_CHANNELS_NO) {
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (xdmac->XDMAC_GS & (XDMAC_GS_ST0 << channel)) {
		return -EBUSY;
	}

	/* NOTE:
	 * Setting source, destination address is not required for linked list
	 * view 1 to 3 modes. It is done anyway to keep the code simple. It has
	 * no negative impact on the DMA functionality.
	 */

	/* Set source address */
	xdmac->XDMAC_CHID[channel].XDMAC_CSA = param->sa;
	/* Set destination address */
	xdmac->XDMAC_CHID[channel].XDMAC_CDA = param->da;

	if ((param->ndc & XDMAC_CNDC_NDE) == XDMAC_CNDC_NDE_DSCR_FETCH_DIS) {
		/*
		 * Linked List is disabled, configure additional transfer
		 * parameters.
		 */

		/* Set length of data in the microblock */
		xdmac->XDMAC_CHID[channel].XDMAC_CUBC = param->ublen;
		/* Set block length: block length is (blen+1) microblocks */
		xdmac->XDMAC_CHID[channel].XDMAC_CBC = param->blen;
	} else {
		/*
		 * Linked List is enabled, configure additional transfer
		 * parameters.
		 */

		/* Set next descriptor address */
		xdmac->XDMAC_CHID[channel].XDMAC_CNDA = param->nda;
	}

	/* Set next descriptor configuration */
	xdmac->XDMAC_CHID[channel].XDMAC_CNDC = param->ndc;

	return 0;
}

static int sam_xdmac_config(const struct device *dev, uint32_t channel,
			    struct dma_config *cfg)
{
	struct sam_xdmac_dev_data *const dev_data = DEV_DATA(dev);
	struct sam_xdmac_channel_config channel_cfg;
	struct sam_xdmac_transfer_config transfer_cfg;
	uint32_t burst_size;
	uint32_t data_size;
	int ret;

	if (channel >= DMA_CHANNELS_NO) {
		return -EINVAL;
	}

	__ASSERT_NO_MSG(cfg->source_data_size == cfg->dest_data_size);
	__ASSERT_NO_MSG(cfg->source_burst_length == cfg->dest_burst_length);

	if (cfg->source_data_size != 1U && cfg->source_data_size != 2U &&
	    cfg->source_data_size != 4U) {
		LOG_ERR("Invalid 'source_data_size' value");
		return -EINVAL;
	}

	burst_size = find_msb_set(cfg->source_burst_length) - 1;
	LOG_DBG("burst_size=%d", burst_size);
	data_size = find_msb_set(cfg->source_data_size) - 1;
	dev_data->dma_channels[channel].data_size = data_size;
	LOG_DBG("data_size=%d", data_size);

	switch (cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		channel_cfg.cfg =
			  XDMAC_CC_TYPE_MEM_TRAN
			| XDMAC_CC_MBSIZE(burst_size == 0U ? 0 : burst_size - 1)
			| XDMAC_CC_SAM_INCREMENTED_AM
			| XDMAC_CC_DAM_INCREMENTED_AM;
		break;
	case MEMORY_TO_PERIPHERAL:
		channel_cfg.cfg =
			  XDMAC_CC_TYPE_PER_TRAN
			| XDMAC_CC_CSIZE(burst_size)
			| XDMAC_CC_DSYNC_MEM2PER
			| XDMAC_CC_SAM_INCREMENTED_AM
			| XDMAC_CC_DAM_FIXED_AM;
		break;
	case PERIPHERAL_TO_MEMORY:
		channel_cfg.cfg =
			  XDMAC_CC_TYPE_PER_TRAN
			| XDMAC_CC_CSIZE(burst_size)
			| XDMAC_CC_DSYNC_PER2MEM
			| XDMAC_CC_SAM_FIXED_AM
			| XDMAC_CC_DAM_INCREMENTED_AM;
		break;
	default:
		LOG_ERR("'channel_direction' value %d is not supported",
			    cfg->channel_direction);
		return -EINVAL;
	}

	channel_cfg.cfg |=
		  XDMAC_CC_DWIDTH(data_size)
		  // TODO: Probably want to make this configurable, will be important if
		  // a lot of DMA channels are used to prevent saturation of the port
		| XDMAC_CC_SIF_AHB_IF1
#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_dtcm), okay)
		/* Can only write to dtcm through port 0 */
		| XDMAC_CC_DIF_AHB_IF0
#else
		| XDMAC_CC_DIF_AHB_IF1
#endif
		| XDMAC_CC_PERID(cfg->dma_slot);
	channel_cfg.ds_msp = 0U;
	channel_cfg.sus = 0U;
	channel_cfg.dus = 0U;
	channel_cfg.cie =
		  (cfg->complete_callback_en ? XDMAC_CIE_BIE : XDMAC_CIE_LIE)
		| (cfg->error_callback_en ? XDMAC_INT_ERR : 0);

	ret = sam_xdmac_channel_configure(dev, channel, &channel_cfg);
	if (ret < 0) {
		return ret;
	}

	dev_data->dma_channels[channel].callback = cfg->dma_callback;
	dev_data->dma_channels[channel].user_data = cfg->user_data;
	dev_data->dma_channels[channel].block_size = cfg->head_block->block_size;
	dev_data->dma_channels[channel].start_addr_dest = cfg->head_block->dest_address;
	dev_data->dma_channels[channel].start_addr_source = cfg->head_block->source_address;
	dev_data->dma_channels[channel].channel_direction = cfg->channel_direction;

	(void)memset(&transfer_cfg, 0, sizeof(transfer_cfg));


	if (cfg->block_count == 2U) {

		if (!cfg->head_block ||
		    !cfg->head_block->dest_address ||
		    !cfg->head_block->source_address ||
		    !cfg->head_block->next_block ||
		    !cfg->head_block->next_block->dest_address ||
		    !cfg->head_block->next_block->source_address
		    ) {
			LOG_ERR("Invalid block addresses");
			return -EINVAL;
		}
		/* set circular pointers */
		dev_data->dma_channels[channel].block[0].mbr_nda =
			(uint32_t)&dev_data->dma_channels[channel].block[1];
		dev_data->dma_channels[channel].block[1].mbr_nda =
			(uint32_t)&dev_data->dma_channels[channel].block[0];

		/* ublen */
		uint32_t ndc = 0;
		uint32_t ubc = cfg->head_block->block_size >> data_size;
		/* enable destination update */
		if (cfg->channel_direction == MEMORY_TO_MEMORY ||
		    cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
			ndc |= BIT(2);
			ubc |= BIT(26);
		}
		/* enable source update */
		if (cfg->channel_direction == MEMORY_TO_MEMORY ||
		    cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
			ndc |= BIT(1);
			ubc |= BIT(25);
		}
		/* enable next descriptor */
		ndc |= BIT(0);
		ubc |= BIT(24);

		/* use descriptor view 1 */
		ndc |= BIT(3);
		ubc |= BIT(27);

		dev_data->dma_channels[channel].block[0].mbr_ubc = ubc;
		dev_data->dma_channels[channel].block[1].mbr_ubc = ubc;

		/* Note: the buffers being pointed to must be in a no cache region, TCM or
		 * cache coherence must be manually managed (only tested in no cache region)
		 */
		dev_data->dma_channels[channel].block[0].mbr_da =
			cfg->head_block->dest_address;
		dev_data->dma_channels[channel].block[1].mbr_da =
			cfg->head_block->next_block->dest_address;

		dev_data->dma_channels[channel].block[0].mbr_sa =
			cfg->head_block->source_address;
		dev_data->dma_channels[channel].block[1].mbr_sa =
			cfg->head_block->next_block->source_address;

		/* TODO: Might need to make the DMA port configurable */
		transfer_cfg.nda = (uint32_t)&dev_data->dma_channels[channel].block[0];
		transfer_cfg.ndc = ndc;
	} else if (cfg->block_count != 1) {
		LOG_ERR("Only single block or 2 blocks (circular) is supported");
		return -EINVAL;
	}

	transfer_cfg.sa = cfg->head_block->source_address;
	transfer_cfg.da = cfg->head_block->dest_address;
	transfer_cfg.ublen = cfg->head_block->block_size >> data_size;

	ret = sam_xdmac_transfer_configure(dev, channel, &transfer_cfg);

	return ret;
}

static int sam_xdmac_transfer_reload(const struct device *dev, uint32_t channel,
				     uint32_t src, uint32_t dst, size_t size)
{
	// This is needed to insure proper opperation
	SCB_CleanInvalidateDCache_by_Addr((void *)src, size);

	struct sam_xdmac_dev_data *const dev_data = DEV_DATA(dev);
	struct sam_xdmac_transfer_config transfer_cfg = {
		.sa = src,
		.da = dst,
		.ublen = size >> dev_data->dma_channels[channel].data_size,
	};

	dev_data->dma_channels[channel].block_size = size;
	dev_data->dma_channels[channel].start_addr_dest = dst;
	dev_data->dma_channels[channel].start_addr_source = src;

	return sam_xdmac_transfer_configure(dev, channel, &transfer_cfg);
}

int sam_xdmac_transfer_start(const struct device *dev, uint32_t channel)
{
	struct sam_xdmac_dev_data *const dev_data = DEV_DATA(dev);
	Xdmac *const xdmac = DEV_CFG(dev)->regs;

	if (channel >= DMA_CHANNELS_NO) {
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (xdmac->XDMAC_GS & (XDMAC_GS_ST0 << channel)) {
		return -EBUSY;
	}

	/* Enable channel interrupt */
	xdmac->XDMAC_GIE = XDMAC_GIE_IE0 << channel;

	/* Clear the cache or there is undefiened behavior */
	switch (dev_data->dma_channels[channel].channel_direction) {
		case MEMORY_TO_MEMORY:
		case PERIPHERAL_TO_MEMORY:
			SCB_CleanInvalidateDCache_by_Addr(
				(volatile void *)dev_data->dma_channels[channel].start_addr_dest,
				dev_data->dma_channels[channel].block_size);
			break;
		default:
			//nothing to do for other cases
			break;
	}

	/* Enable channel */
	xdmac->XDMAC_GE = XDMAC_GE_EN0 << channel;

	return 0;
}

int sam_xdmac_transfer_stop(const struct device *dev, uint32_t channel)
{
	Xdmac *const xdmac = DEV_CFG(dev)->regs;

	if (channel >= DMA_CHANNELS_NO) {
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (!(xdmac->XDMAC_GS & (XDMAC_GS_ST0 << channel))) {
		return 0;
	}

	/* Disable channel */
	xdmac->XDMAC_GD = XDMAC_GD_DI0 << channel;
	/* Disable channel interrupt */
	xdmac->XDMAC_GID = XDMAC_GID_ID0 << channel;
	/* Disable all channel interrupts */
	xdmac->XDMAC_CHID[channel].XDMAC_CID = 0xFF;
	/* Clear the pending Interrupt Status bit(s) */
	(void)xdmac->XDMAC_CHID[channel].XDMAC_CIS;

	return 0;
}

static int sam_xdmac_transfer_get_status(const struct device *dev, uint32_t channel,
				  struct dma_status *status)
{
	struct sam_xdmac_dev_data *const dev_data = DEV_DATA(dev);
	Xdmac *const xdmac = DEV_CFG(dev)->regs;

	if (channel >= DMA_CHANNELS_NO) {
		return -EINVAL;
	}

	//TODO: make this mirror if the dma request is started or stopped
	status->busy = false;

	// Required for proper opperation with cache enabled
	SCB_CleanInvalidateDCache_by_Addr(&dev_data->dma_channels[channel], sizeof(struct sam_xdmac_channel_cfg));

	status->dir = dev_data->dma_channels[channel].channel_direction;

	if (status->dir == PERIPHERAL_TO_MEMORY || status->dir == MEMORY_TO_MEMORY) {
		// Buff size - bytes already transferred
		status->pending_length = dev_data->dma_channels[channel].block_size -
			(xdmac->XDMAC_CHID[channel].XDMAC_CDA - dev_data->dma_channels[channel].start_addr_dest);
	} else {
		status->pending_length =  dev_data->dma_channels[channel].block_size -
			(xdmac->XDMAC_CHID[channel].XDMAC_CSA - dev_data->dma_channels[channel].start_addr_source);
	}

	return 0;
}

static int sam_xdmac_initialize(const struct device *dev)
{
	const struct sam_xdmac_dev_cfg *const dev_cfg = DEV_CFG(dev);
	Xdmac *const xdmac = dev_cfg->regs;

	/* Configure interrupts */
	dev_cfg->irq_config();

	/* Enable module's clock */
	soc_pmc_peripheral_enable(dev_cfg->periph_id);

	/* Disable all channels */
	xdmac->XDMAC_GD = UINT32_MAX;
	/* Disable all channel interrupts */
	xdmac->XDMAC_GID = UINT32_MAX;

	/* Enable module's IRQ */
	irq_enable(dev_cfg->irq_id);

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}

static const struct dma_driver_api sam_xdmac_driver_api = {
	.config = sam_xdmac_config,
	.reload = sam_xdmac_transfer_reload,
	.start = sam_xdmac_transfer_start,
	.stop = sam_xdmac_transfer_stop,
	.get_status = sam_xdmac_transfer_get_status,
};

/* DMA0 */

static void dma0_sam_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), sam_xdmac_isr,
		    DEVICE_DT_INST_GET(0), 0);
}

static const struct sam_xdmac_dev_cfg dma0_sam_config = {
	.regs = (Xdmac *)DT_INST_REG_ADDR(0),
	.irq_config = dma0_sam_irq_config,
	.periph_id = DT_INST_PROP(0, peripheral_id),
	.irq_id = DT_INST_IRQN(0),
};

static struct sam_xdmac_dev_data dma0_sam_data;

DEVICE_DT_INST_DEFINE(0, &sam_xdmac_initialize, NULL,
		    &dma0_sam_data, &dma0_sam_config, POST_KERNEL,
		    CONFIG_DMA_INIT_PRIORITY, &sam_xdmac_driver_api);
