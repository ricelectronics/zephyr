/*
 * Copyright (c) 2017 Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_i2c_twihs

/** @file
 * @brief I2C bus (TWIHS) driver for Atmel SAM MCU family.
 *
 * Only I2C Master Mode with 7 bit addressing is currently supported.
 */


#include <errno.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/i2c.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_sam_twihs);

#include "i2c-priv.h"

/** I2C bus speed [Hz] in Standard Mode */
#define BUS_SPEED_STANDARD_HZ         100000U
/** I2C bus speed [Hz] in Fast Mode */
#define BUS_SPEED_FAST_HZ             400000U
/** I2C bus speed [Hz] in High Speed Mode */
#define BUS_SPEED_HIGH_HZ            3400000U
/* Maximum value of Clock Divider (CKDIV) */
#define CKDIV_MAX                          7

/* Device constant configuration parameters */
struct i2c_sam_twihs_dev_cfg {
	Twihs *regs;
	void (*irq_config)(void);
	uint32_t bitrate;
	const struct soc_gpio_pin *pin_list;
	uint8_t pin_list_size;
	uint8_t periph_id;
	uint8_t irq_id;
};

struct twihs_msg {
	/* Buffer containing data to read or write */
	uint8_t *buf;
	/* Length of the buffer */
	uint32_t len;
	/* Index of the next byte to be read/written from/to the buffer */
	uint32_t idx;
	/* Value of TWIHS_SR at the end of the message */
	uint32_t twihs_sr;
	/* Transfer flags as defined in the i2c.h file */
	uint8_t flags;
};

/* Device run time data */
struct i2c_sam_twihs_dev_data {
	struct k_sem sem;
	struct i2c_msg *msgs;
	uint8_t msgs_count;
	volatile uint8_t msgs_idx;
	volatile uint8_t msg_idx;
	uint32_t twihs_sr;
	uint16_t addr;
};

#define DEV_NAME(dev) ((dev)->name)
#define DEV_CFG(dev) \
	((const struct i2c_sam_twihs_dev_cfg *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct i2c_sam_twihs_dev_data *const)(dev)->data)

static int i2c_clk_set(Twihs *const twihs, uint32_t speed)
{
	uint32_t ck_div = 0U;
	uint32_t cl_div;
	bool div_completed = false;

	/*  From the datasheet "TWIHS Clock Waveform Generator Register"
	 *  T_low = ( ( CLDIV × 2^CKDIV ) + 3 ) × T_MCK
	 */
	while (!div_completed) {
		cl_div =   ((SOC_ATMEL_SAM_MCK_FREQ_HZ / (speed * 2U)) - 3)
			 / (1 << ck_div);

		if (cl_div <= 255U) {
			div_completed = true;
		} else {
			ck_div++;
		}
	}

	if (ck_div > CKDIV_MAX) {
		LOG_ERR("Failed to configure I2C clock");
		return -EIO;
	}

	/* Set I2C bus clock duty cycle to 50% */
	twihs->TWIHS_CWGR = TWIHS_CWGR_CLDIV(cl_div) | TWIHS_CWGR_CHDIV(cl_div)
			    | TWIHS_CWGR_CKDIV(ck_div);

	return 0;
}

static int i2c_sam_twihs_configure(const struct device *dev, uint32_t config)
{
	const struct i2c_sam_twihs_dev_cfg *const dev_cfg = DEV_CFG(dev);
	Twihs *const twihs = dev_cfg->regs;
	uint32_t bitrate;
	int ret;

	if (!(config & I2C_MODE_MASTER)) {
		LOG_ERR("Master Mode is not enabled");
		return -EIO;
	}

	if (config & I2C_ADDR_10_BITS) {
		LOG_ERR("I2C 10-bit addressing is currently not supported");
		LOG_ERR("Please submit a patch");
		return -EIO;
	}

	/* Configure clock */
	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_STANDARD:
		bitrate = BUS_SPEED_STANDARD_HZ;
		break;
	case I2C_SPEED_FAST:
		bitrate = BUS_SPEED_FAST_HZ;
		break;
	default:
		LOG_ERR("Unsupported I2C speed value");
		return -EIO;
	}

	/* Setup clock waveform */
	ret = i2c_clk_set(twihs, bitrate);
	if (ret < 0) {
		return ret;
	}

	/* Disable Slave Mode */
	twihs->TWIHS_CR = TWIHS_CR_SVDIS;

	/* Enable Master Mode */
	twihs->TWIHS_CR = TWIHS_CR_MSEN | TWIHS_CR_FIFOEN;

	return 0;
}

static inline void write_msg_start(Twihs *const twihs, struct i2c_sam_twihs_dev_data *const dev_data, struct i2c_msg * msg)
{
	twihs->TWIHS_MMR = TWIHS_MMR_DADR(dev_data->addr);

	/* Write first data byte on I2C bus */
	twihs->TWIHS_THR = msg->buf[dev_data->msg_idx++];

	/* Enable Transmit Ready and Transmission Completed interrupts */
	twihs->TWIHS_IER = TWIHS_IER_TXRDY | TWIHS_IER_TXCOMP | TWIHS_IER_NACK;
}

static inline void read_msg_start(Twihs *const twihs, uint8_t len, uint16_t addr)
{
	twihs->TWIHS_MMR = TWIHS_MMR_MREAD | TWIHS_MMR_DADR(addr);

	/* Enable Receive Ready and Transmission Completed interrupts */
	twihs->TWIHS_IER = TWIHS_IER_RXRDY | TWIHS_IER_TXCOMP | TWIHS_IER_NACK;

	/* In single data byte read the START and STOP must both be set */
	if (len == 1U) {
		twihs->TWIHS_CR = TWIHS_CR_START | TWIHS_CR_STOP;
	} else {
		/* Start the transfer by sending START condition */
		twihs->TWIHS_CR = TWIHS_CR_START;
	}
}

static int i2c_sam_twihs_send_msg(const struct device *dev)
{
	const struct i2c_sam_twihs_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2c_sam_twihs_dev_data *const dev_data = DEV_DATA(dev);
	Twihs *const twihs = dev_cfg->regs;

	__ASSERT(dev_data->msgs, "No messages to send");
	__ASSERT(dev_data->msgs_count > dev_data->msgs_idx, "Message out of range");

	dev_data->msg_idx = 0;

	// Get the first message
	struct i2c_msg * msg = &dev_data->msgs[dev_data->msgs_idx];

	if ((msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		read_msg_start(twihs, msg->len, dev_data->addr);
	} else {
		write_msg_start(twihs, dev_data, msg);
	}
	return 0;
}

static int i2c_sam_twihs_transfer(const struct device *dev,
				  struct i2c_msg *msgs,
				  uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_sam_twihs_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2c_sam_twihs_dev_data *const dev_data = DEV_DATA(dev);
	Twihs *const twihs = dev_cfg->regs;

	__ASSERT_NO_MSG(msgs);
	if (!num_msgs) {
		return 0;
	}

	/* Clear pending interrupts, such as NACK. */
	(void)twihs->TWIHS_SR;

	/* Set number of internal address bytes to 0, not used. */
	twihs->TWIHS_IADR = 0;

	dev_data->msgs = msgs;
	dev_data->msgs_count = num_msgs;
	dev_data->msgs_idx = 0;
	dev_data->addr = addr;

	while(dev_data->msgs_idx < dev_data->msgs_count) {
		//struct twihs_msg *msg = &dev_data->msgs[dev_data->idx];

		i2c_sam_twihs_send_msg(dev);

		int res = k_sem_take(&dev_data->sem, K_FOREVER);
		if (dev_data->twihs_sr > 0) {
			/* Something went wrong */
			LOG_ERR("Transfer error: %x", dev_data->twihs_sr);
			return -EIO;
		} else if (res != 0) {
			/* Timeout */
			LOG_ERR("Transfer timeout");
			return -EAGAIN;
		}
	}
	return 0;
}

static int i2c_sam_twihs_recover_bus(const struct device *dev) {
	const struct i2c_sam_twihs_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2c_sam_twihs_dev_data *const dev_data = DEV_DATA(dev);
	Twihs *const twihs = dev_cfg->regs;

	/* Send the Clear Bus Command */
	twihs->TWIHS_CR = TWIHS_CR_CLEAR | TWIHS_CR_LOCKCLR;
}

static void i2c_sam_twihs_isr(const struct device *dev)
{
	const struct i2c_sam_twihs_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2c_sam_twihs_dev_data *const dev_data = DEV_DATA(dev);
	Twihs *const twihs = dev_cfg->regs;
	struct i2c_msg * msg = &dev_data->msgs[dev_data->msgs_idx];
	uint32_t isr_status;

	/* Retrieve interrupt status */
	isr_status = twihs->TWIHS_SR & twihs->TWIHS_IMR;
	if (isr_status == 0) {
		return;
	}

	/* Not Acknowledged */
	if (isr_status & TWIHS_SR_NACK) {
		dev_data->twihs_sr = isr_status;
		goto tx_comp;
	}

	/* Byte received */
	if (isr_status & TWIHS_SR_RXRDY) {
		/* We disable interrupts here because the timing is very sensitive */
		int key = irq_lock();
		msg->buf[dev_data->msg_idx++] = twihs->TWIHS_RHR;
		if (dev_data->msg_idx == msg->len - 1U) {
			/* Send STOP condition */
			twihs->TWIHS_CR = TWIHS_CR_STOP;
		} else if (msg->len == 1U) {
			/* Send STOP condition */
			twihs->TWIHS_CR = TWIHS_CR_STOP;
		}
		irq_unlock(key);
	}

	/* Byte sent */
	if (isr_status & TWIHS_SR_TXRDY) {
		if (dev_data->msg_idx == msg->len) {
			if (msg->flags & I2C_MSG_STOP) {
				/* Send STOP condition */
				twihs->TWIHS_CR = TWIHS_CR_STOP;
				/* Disable Transmit Ready interrupt */
				twihs->TWIHS_IDR = TWIHS_IDR_TXRDY;
			} else if (msg->flags & I2C_MSG_RESTART) {
					/* Disable Transmit Ready interrupt */
					twihs->TWIHS_IDR = TWIHS_IDR_TXRDY;

					/* We disable interrupts here because the timing is very sensitive */
					int key = irq_lock();
					dev_data->msgs_idx++;
					i2c_sam_twihs_send_msg(dev);
					irq_unlock(key);
			} else {
				/* Transmission completed */
				goto tx_comp;
			}
		} else {
			twihs->TWIHS_THR = msg->buf[dev_data->msg_idx++];
		}
	}

	/* Transmission completed */
	if (isr_status & TWIHS_SR_TXCOMP) {
		goto tx_comp;
	}

	return;

tx_comp:
	/* Disable all enabled interrupts */
	twihs->TWIHS_IDR = twihs->TWIHS_IMR;
	/* We are done */
	dev_data->msgs_idx++;
	k_sem_give(&dev_data->sem);
}

static int i2c_sam_twihs_initialize(const struct device *dev)
{
	const struct i2c_sam_twihs_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2c_sam_twihs_dev_data *const dev_data = DEV_DATA(dev);
	Twihs *const twihs = dev_cfg->regs;
	uint32_t bitrate_cfg;
	int ret;

	/* Configure interrupts */
	dev_cfg->irq_config();

	/* Initialize semaphore */
	k_sem_init(&dev_data->sem, 0, 1);

	/* Connect pins to the peripheral */
	soc_gpio_list_configure(dev_cfg->pin_list, dev_cfg->pin_list_size);

	/* Enable module's clock */
	soc_pmc_peripheral_enable(dev_cfg->periph_id);

	/* Reset the module */
	twihs->TWIHS_CR = TWIHS_CR_SWRST;

	bitrate_cfg = i2c_map_dt_bitrate(dev_cfg->bitrate);

	ret = i2c_sam_twihs_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to initialize %s device", DEV_NAME(dev));
		return ret;
	}

	/* Enable module's IRQ */
	irq_enable(dev_cfg->irq_id);

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}

static const struct i2c_driver_api i2c_sam_twihs_driver_api = {
	.configure = i2c_sam_twihs_configure,
	.transfer = i2c_sam_twihs_transfer,
	.recover_bus = i2c_sam_twihs_recover_bus,
};

#define I2C_TWIHS_SAM_INIT(n)						\
	static void i2c##n##_sam_irq_config(void)			\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	\
			    i2c_sam_twihs_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
	}								\
									\
	static const struct soc_gpio_pin pins_twihs##n[] = ATMEL_SAM_DT_INST_PINS(n); \
									\
	static const struct i2c_sam_twihs_dev_cfg i2c##n##_sam_config = {\
		.regs = (Twihs *)DT_INST_REG_ADDR(n),			\
		.irq_config = i2c##n##_sam_irq_config,			\
		.periph_id = DT_INST_PROP(n, peripheral_id),		\
		.irq_id = DT_INST_IRQN(n),				\
		.pin_list = pins_twihs##n,				\
		.pin_list_size = ARRAY_SIZE(pins_twihs##n),		\
		.bitrate = DT_INST_PROP(n, clock_frequency),		\
	};								\
									\
	static struct i2c_sam_twihs_dev_data i2c##n##_sam_data;		\
									\
	DEVICE_DT_INST_DEFINE(n, &i2c_sam_twihs_initialize, 		\
			    NULL,					\
			    &i2c##n##_sam_data, &i2c##n##_sam_config,	\
			    POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	\
			    &i2c_sam_twihs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_TWIHS_SAM_INIT)
