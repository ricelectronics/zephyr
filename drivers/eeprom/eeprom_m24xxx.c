/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for Atmel AT24 I2C and Atmel AT25 SPI EEPROMs.
 */
#define DT_DRV_COMPAT st_m24xxx

#include <drivers/eeprom.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_EEPROM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(eeprom_m24xxx);

struct eeprom_m24xxx_config {
	const struct device *bus_dev;
	uint16_t i2c_addr;
	struct gpio_dt_spec wp_gpio;
	size_t size;
	//size_t pagesize;
	//uint8_t addr_width;
	//bool readonly;
	//uint16_t timeout;
	eeprom_api_read read_fn;
	eeprom_api_write write_fn;
};

struct eeprom_m24xxx_data {
	struct k_mutex lock;
};

static inline int eeprom_m24xxx_write_protect(const struct device *dev)
{
	const struct eeprom_m24xxx_config *config = dev->config;

	if (!config->wp_gpio.port) {
		return 0;
	}

	return gpio_pin_set(config->wp_gpio.port, config->wp_gpio.pin, 1);
}

static inline int eeprom_m24xxx_write_enable(const struct device *dev)
{
	const struct eeprom_m24xxx_config *config = dev->config;

	if (!config->wp_gpio.port) {
		return 0;
	}

	return gpio_pin_set(config->wp_gpio.port, config->wp_gpio.pin, 0);
}

static int eeprom_m24xxx_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	struct eeprom_m24xxx_data *data = dev->data;
	uint8_t *pbuf = buf;
	int ret;

	if (!len) {
		return 0;
	}
	/*
	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}
	*/
	k_mutex_lock(&data->lock, K_FOREVER);
	while (len) {
		ret = config->read_fn(dev, offset, pbuf, len);
		if (ret < 0) {
			printk("failed to read EEPROM (err %d)", ret);
			k_mutex_unlock(&data->lock);
			return ret;
		}

		pbuf += ret;
		offset += ret;
		len -= ret;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static size_t eeprom_m24xxx_limit_write_count(const struct device *dev, off_t offset, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	size_t count = len;
	off_t page_boundary;
	/*
	// We can at most write one page at a time
	if (count > config->pagesize) {
		count = config->pagesize;
	}

	// Writes can not cross a page boundary
	page_boundary = ROUND_UP(offset + 1, config->pagesize);
	if (offset + count > page_boundary) {
		count = page_boundary - offset;
	}
	*/
	return count;
}

static int eeprom_m24xxx_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	struct eeprom_m24xxx_data *data = dev->data;
	const uint8_t *pbuf = buf;
	int ret;
	/*
	if (config->readonly) {
		LOG_WRN("attempt to write to read-only device");
		return -EACCES;
	}

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		printk("attempt to write past device boundary");
		return -EINVAL;
	}
	*/
	k_mutex_lock(&data->lock, K_FOREVER);

	ret = eeprom_m24xxx_write_enable(dev);
	if (ret) {
		printk("failed to write-enable EEPROM (err %d)", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	while (len) {
		ret = config->write_fn(dev, offset, pbuf, len);
		if (ret < 0) {
			printk("failed to write to EEPROM (err %d)", ret);
			eeprom_m24xxx_write_protect(dev);
			k_mutex_unlock(&data->lock);
			return ret;
		}

		pbuf += ret;
		offset += ret;
		len -= ret;
	}

	ret = eeprom_m24xxx_write_protect(dev);
	if (ret) {
		printk("failed to write-protect EEPROM (err %d)", ret);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static size_t eeprom_m24xxx_size(const struct device *dev)
{
	const struct eeprom_m24xxx_config *config = dev->config;

	return config->size;
}

#ifdef CONFIG_EEPROM_AT24

/**
 * @brief translate an offset to a device address / offset pair
 *
 * It allows to address several devices as a continuous memory region
 * but also to address higher part of eeprom for chips
 * with more than 2^(addr_width) adressable word.
 */
static uint16_t eeprom_at24_translate_offset(const struct device *dev, off_t *offset)
{
	const struct eeprom_m24xxx_config *config = dev->config;

	const uint16_t addr_incr = *offset >> config->addr_width;
	*offset &= BIT_MASK(config->addr_width);

	return config->bus.i2c_addr + addr_incr;
}

static size_t eeprom_at24_adjust_read_count(const struct device *dev, off_t offset, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	const size_t remainder = BIT(config->addr_width) - offset;

	if (len > remainder) {
		len = remainder;
	}

	return len;
}

static int eeprom_at24_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	int64_t timeout;
	uint8_t addr[2];
	uint16_t bus_addr;
	int err;

	bus_addr = eeprom_at24_translate_offset(dev, &offset);

	if (config->addr_width == 16) {
		sys_put_be16(offset, addr);
	} else {
		addr[0] = offset & BIT_MASK(8);
	}

	len = eeprom_at24_adjust_read_count(dev, offset, len);

	/*
	 * A write cycle may be in progress so reads must be attempted
	 * until the current write cycle should be completed.
	 */
	timeout = k_uptime_get() + config->timeout;
	while (1) {
		int64_t now = k_uptime_get();
		err = i2c_write_read(config->bus_dev, bus_addr, addr, config->addr_width / 8, buf,
				     len);
		if (!err || now > timeout) {
			break;
		}
		k_sleep(K_MSEC(1));
	}

	if (err < 0) {
		return err;
	}

	return len;
}

static int eeprom_at24_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	int count = eeprom_m24xxx_limit_write_count(dev, offset, len);
	uint8_t block[config->addr_width / 8 + count];
	int64_t timeout;
	uint16_t bus_addr;
	int i = 0;
	int err;

	bus_addr = eeprom_at24_translate_offset(dev, &offset);

	/*
	 * Not all I2C EEPROMs support repeated start so the the
	 * address (offset) and data (buf) must be provided in one
	 * write transaction (block).
	 */
	if (config->addr_width == 16) {
		block[i++] = offset >> 8;
	}
	block[i++] = offset;
	memcpy(&block[i], buf, count);

	/*
	 * A write cycle may already be in progress so writes must be
	 * attempted until the previous write cycle should be
	 * completed.
	 */
	timeout = k_uptime_get() + config->timeout;
	while (1) {
		int64_t now = k_uptime_get();
		err = i2c_write(config->bus_dev, block, sizeof(block), bus_addr);
		if (!err || now > timeout) {
			break;
		}
		k_sleep(K_MSEC(1));
	}

	if (err < 0) {
		return err;
	}

	return count;
}
#endif /* CONFIG_EEPROM_AT24 */

#ifdef CONFIG_EEPROM_AT25
static int eeprom_at25_rdsr(const struct device *dev, uint8_t *status)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	uint8_t rdsr[2] = { EEPROM_AT25_RDSR, 0 };
	uint8_t sr[2];
	int err;
	const struct spi_buf tx_buf = {
		.buf = rdsr,
		.len = sizeof(rdsr),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = sr,
		.len = sizeof(sr),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	err = spi_transceive(config->bus_dev, &config->bus.spi_cfg, &tx, &rx);
	if (!err) {
		*status = sr[1];
	}

	return err;
}

static int eeprom_at25_wait_for_idle(const struct device *dev)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	int64_t timeout;
	uint8_t status;
	int err;

	timeout = k_uptime_get() + config->timeout;
	while (1) {
		int64_t now = k_uptime_get();
		err = eeprom_at25_rdsr(dev, &status);
		if (err) {
			printk("Could not read status register (err %d)", err);
			return err;
		}

		if (!(status & EEPROM_AT25_STATUS_WIP)) {
			return 0;
		}
		if (now > timeout) {
			break;
		}
		k_sleep(K_MSEC(1));
	}

	return -EBUSY;
}

static int eeprom_at25_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	struct eeprom_m24xxx_data *data = dev->data;
	size_t cmd_len = 1 + config->addr_width / 8;
	uint8_t cmd[4] = { EEPROM_AT25_READ, 0, 0, 0 };
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = cmd_len,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_bufs[2] = {
		{
			.buf = NULL,
			.len = cmd_len,
		},
		{
			.buf = buf,
			.len = len,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	paddr = &cmd[1];
	switch (config->addr_width) {
	case 24:
		*paddr++ = offset >> 16;
		__fallthrough;
	case 16:
		*paddr++ = offset >> 8;
		__fallthrough;
	case 8:
		*paddr++ = offset;
		break;
	default:
		__ASSERT(0, "invalid address width");
	}

	err = eeprom_at25_wait_for_idle(dev);
	if (err) {
		printk("EEPROM idle wait failed (err %d)", err);
		k_mutex_unlock(&data->lock);
		return err;
	}

	err = spi_transceive(config->bus_dev, &config->bus.spi_cfg, &tx, &rx);
	if (err < 0) {
		return err;
	}

	return len;
}

static int eeprom_at25_wren(const struct device *dev)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	uint8_t cmd = EEPROM_AT25_WREN;
	const struct spi_buf tx_buf = {
		.buf = &cmd,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write(config->bus_dev, &config->bus.spi_cfg, &tx);
}

static int eeprom_at25_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct eeprom_m24xxx_config *config = dev->config;
	int count = eeprom_m24xxx_limit_write_count(dev, offset, len);
	uint8_t cmd[4] = { EEPROM_AT25_WRITE, 0, 0, 0 };
	size_t cmd_len = 1 + config->addr_width / 8;
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_bufs[2] = {
		{
			.buf = cmd,
			.len = cmd_len,
		},
		{
			.buf = (void *)buf,
			.len = count,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	paddr = &cmd[1];
	switch (config->addr_width) {
	case 24:
		*paddr++ = offset >> 16;
		__fallthrough;
	case 16:
		*paddr++ = offset >> 8;
		__fallthrough;
	case 8:
		*paddr++ = offset;
		break;
	default:
		__ASSERT(0, "invalid address width");
	}

	err = eeprom_at25_wait_for_idle(dev);
	if (err) {
		printk("EEPROM idle wait failed (err %d)", err);
		return err;
	}

	err = eeprom_at25_wren(dev);
	if (err) {
		printk("failed to disable write protection (err %d)", err);
		return err;
	}

	err = spi_transceive(config->bus_dev, &config->bus.spi_cfg, &tx, NULL);
	if (err) {
		return err;
	}

	return count;
}
#endif /* CONFIG_EEPROM_AT25 */

static int eeprom_m24xxx_init(const struct device *dev)
{
	printk("device init...\n");
	const struct eeprom_m24xxx_config *config = dev->config;
	struct eeprom_m24xxx_data *data = dev->data;
	int err;

	k_mutex_init(&data->lock);

	if (!device_is_ready(config->bus_dev)) {
		printk("parent bus device not ready");
		return -EINVAL;
	}

	if (config->wp_gpio.port) {
		if (!device_is_ready(config->wp_gpio.port)) {
			printk("wp gpio device not ready");
			return -EINVAL;
		}

		err = gpio_pin_configure_dt(&config->wp_gpio, GPIO_OUTPUT_ACTIVE);
		if (err) {
			printk("failed to configure WP GPIO pin (err %d)", err);
			return err;
		}
	}

	return 0;
}

static const struct eeprom_driver_api eeprom_m24xxx_api = {
	.read = eeprom_m24xxx_read,
	.write = eeprom_m24xxx_write,
	.size = eeprom_m24xxx_size,
};
/*
#define INST_DT_M24XXX(inst, t) DT_INST(inst, st_m24##t)

#define EEPROM_M24XXX_DEVICE(n, t)                                                                 \
	static const struct eeprom_m24xxx_config eeprom_m24##t##_config_##n = {                    \
		.bus_dev = DEVICE_DT_GET(DT_BUS(INST_DT_M24XXX(n, t))),                            \
		.i2c_addr = DT_REG_ADDR(INST_DT_M24XXX(n, t)),                                     \
		.wp_gpio = GPIO_DT_SPEC_GET_OR(INST_DT_M24XXX(n, t), wp_gpios, { 0 }),             \
		.size = DT_PROP(INST_DT_M24XXX(n, t), size),                                       \
		.pagesize = DT_PROP(INST_DT_M24XXX(n, t), pagesize),                               \
		.addr_width = DT_PROP(INST_DT_M24XXX(n, t), address_width),                        \
		.readonly = DT_PROP(INST_DT_M24XXX(n, t), read_only),                              \
		.timeout = DT_PROP(INST_DT_M24XXX(n, t), timeout),                                 \
		.read_fn = eeprom_m24##t##_read,                                                   \
		.write_fn = eeprom_m24##t##_write,                                                 \
	};                                                                                         \
	static struct eeprom_m24xxx_data eeprom_m24##t##_data_##n;                                 \
	DEVICE_DT_DEFINE(INST_DT_M24XXX(n, t), &eeprom_m24xxx_init, NULL,                          \
			 &eeprom_m24##t##_data_##n, &eeprom_m24##t##_config_##n, POST_KERNEL,      \
			 CONFIG_EEPROM_m24xxx_INIT_PRIORITY, &eeprom_m24xxx_api)

#define EEPROM_M24M02_DEVICE(n) EEPROM_M24XXX_DEVICE(n, m02)

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_M24XXX_FOREACH(t, inst_expr)                                                       \
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(st_m24##t), CALL_WITH_ARG, inst_expr)
*/

//#define EEPROM_M24XXX_DEVICE
static const struct eeprom_m24xxx_config eeprom_m24xxx_config_n = {
	.bus_dev = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
	.wp_gpio = GPIO_DT_SPEC_INST_GET_OR(0, wp_gpios, { 0 }),
	.size = DT_INST_PROP(0, size),
	//.pagesize = DT_INST_PROP(0, pagesize),
	//.addr_width = DT_INST_PROP(0, address_width),
	//.readonly = DT_INST_PROP(0, read_only),
	//.timeout = DT_INST_PROP(0, timeout),
	.read_fn = eeprom_m24xxx_read,
	.write_fn = eeprom_m24xxx_write,
};

static struct eeprom_m24xxx_data eeprom_m24xxx_data_n;
DEVICE_DT_INST_DEFINE(0, &eeprom_m24xxx_init, NULL, &eeprom_m24xxx_data_n, &eeprom_m24xxx_config_n,
		      POST_KERNEL, CONFIG_EEPROM_M24XXX_INIT_PRIORITY, &eeprom_m24xxx_api);
