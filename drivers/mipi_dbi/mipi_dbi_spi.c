/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_mipi_dbi_spi

#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mipi_dbi_spi, CONFIG_MIPI_DBI_LOG_LEVEL);

struct mipi_dbi_spi_config {
	/* SPI hardware used to send data */
	const struct device *spi_dev;
	/* Command/Data gpio */
	const struct gpio_dt_spec cmd_data;
	/* Reset GPIO */
	const struct gpio_dt_spec reset;
};

struct mipi_dbi_spi_data {
	/* Used for 3 wire mode */
	uint16_t spi_byte;
	struct k_spinlock lock;
};

/* Expands to 1 if the node does not have the `write-only` property */
#define _WRITE_ONLY_ABSENT(n) (!DT_INST_PROP(n, write_only)) |

/* This macro will evaluate to 1 if any of the nodes with zephyr,mipi-dbi-spi
 * lack a `write-only` property. The intention here is to allow the entire
 * command_read function to be optimized out when it is not needed.
 */
#define MIPI_DBI_SPI_READ_REQUIRED DT_INST_FOREACH_STATUS_OKAY(_WRITE_ONLY_ABSENT) 0
uint32_t var = MIPI_DBI_SPI_READ_REQUIRED;

/* In Type C mode 1 MIPI BIT communication, the 9th bit of the word
 * (first bit sent in each word) indicates if the word is a command or
 * data. Typically 0 indicates a command and 1 indicates data, but some
 * displays may vary.
 */
#define MIPI_DBI_DC_BIT BIT(9)

static int mipi_dbi_spi_write_helper(const struct device *dev,
				     const struct mipi_dbi_config *dbi_config,
				     bool cmd_present, uint8_t cmd,
				     const uint8_t *data_buf, size_t len)
{
	const struct mipi_dbi_spi_config *config = dev->config;
	struct mipi_dbi_spi_data *data = dev->data;
	struct spi_buf buffer;
	struct spi_buf_set buf_set = {
		.buffers = &buffer,
		.count = 1,
	};
	int ret = 0;
	k_spinlock_key_t spinlock_key = k_spin_lock(&data->lock);

	if (dbi_config->mode == MIPI_DBI_MODE_SPI_3WIRE &&
	    IS_ENABLED(CONFIG_MIPI_DBI_SPI_3WIRE)) {
		struct spi_config tmp_cfg;
		/* We have to emulate 3 wire mode by packing the data/command
		 * bit into the upper bit of the SPI transfer.
		 * switch SPI to 9 bit mode, and write the transfer
		 */
		memcpy(&tmp_cfg, &dbi_config->config, sizeof(tmp_cfg));
		tmp_cfg.operation &= ~SPI_WORD_SIZE_MASK;
		tmp_cfg.operation |= SPI_WORD_SET(9);

		/* Send command */
		if (cmd_present) {
			data->spi_byte = cmd;
			buffer.buf = &data->spi_byte;
			buffer.len = 1;
			ret = spi_write(config->spi_dev, &tmp_cfg, &buf_set);
			if (ret < 0) {
				goto out;
			}
		}
		/* Write data, byte by byte */
		for (size_t i = 0; i < len; i++) {
			data->spi_byte = MIPI_DBI_DC_BIT | data_buf[i];
			ret = spi_write(config->spi_dev, &tmp_cfg, &buf_set);
			if (ret < 0) {
				goto out;
			}
		}
	} else if (dbi_config->mode == MIPI_DBI_MODE_SPI_4WIRE) {
		/* 4 wire mode is much simpler. We just toggle the
		 * command/data GPIO to indicate if we are sending
		 * a command or data
		 */
		buffer.buf = &cmd;
		buffer.len = sizeof(cmd);

		if (cmd_present) {
			/* Set CD pin low for command */
			gpio_pin_set_dt(&config->cmd_data, 0);
			ret = spi_write(config->spi_dev, &dbi_config->config,
					&buf_set);
			if (ret < 0) {
				goto out;
			}
		}

		if (len > 0) {
			buffer.buf = (void *)data_buf;
			buffer.len = len;

			/* Set CD pin high for data */
			gpio_pin_set_dt(&config->cmd_data, 1);
			ret = spi_write(config->spi_dev, &dbi_config->config,
					&buf_set);
			if (ret < 0) {
				goto out;
			}
		}
	} else {
		/* Otherwise, unsupported mode */
		ret = -ENOTSUP;
	}
out:
	k_spin_unlock(&data->lock, spinlock_key);
	return ret;
}

static int mipi_dbi_spi_command_write(const struct device *dev,
				      const struct mipi_dbi_config *dbi_config,
				      uint8_t cmd, const uint8_t *data_buf,
				      size_t len)
{
	return mipi_dbi_spi_write_helper(dev, dbi_config, true, cmd,
					 data_buf, len);
}

static int mipi_dbi_spi_write_transfer(const struct device *dev,
				       const struct mipi_dbi_config *dbi_config,
				       const uint8_t *data_buf, size_t len)
{
	return mipi_dbi_spi_write_helper(dev, dbi_config, false, 0x0,
					 data_buf, len);
}

#if MIPI_DBI_SPI_READ_REQUIRED

static int mipi_dbi_spi_command_read(const struct device *dev,
				     const struct mipi_dbi_config *dbi_config,
				     uint8_t cmd, uint8_t *response,
				     size_t len)
{
	const struct mipi_dbi_spi_config *config = dev->config;
	struct mipi_dbi_spi_data *data = dev->data;
	struct spi_buf buffer;
	struct spi_buf_set buf_set = {
		.buffers = &buffer,
		.count = 1,
	};
	int ret;
	k_spinlock_key_t spinlock_key = k_spin_lock(&data->lock);

	if (dbi_config->mode == MIPI_DBI_MODE_SPI_3WIRE &&
	    IS_ENABLED(CONFIG_MIPI_DBI_SPI_3WIRE)) {
		struct spi_config cmd_config, data_config;
		/* We have to emulate 3 wire mode by packing the data/command
		 * bit into the upper bit of the SPI transfer.
		 * switch SPI to 9 bit mode, and write the transfer
		 */
		memcpy(&cmd_config, &dbi_config->config, sizeof(cmd_config));
		memcpy(&data_config, &dbi_config->config, sizeof(data_config));
		cmd_config.operation &= ~SPI_WORD_SIZE_MASK;
		cmd_config.operation |= SPI_WORD_SET(9);
		data_config.operation &= ~SPI_WORD_SIZE_MASK;
		data_config.operation |= SPI_WORD_SET(8);

		/* Send command */
		data->spi_byte = cmd;
		buffer.buf = &data->spi_byte;
		buffer.len = 1;
		ret = spi_write(config->spi_dev, &cmd_config, &buf_set);
		if (ret < 0) {
			goto out;
		}
		/* Now, we can switch to 8 bit mode, and read data */
		buffer.buf = (void *)response;
		buffer.len = len;
		ret = spi_read(config->spi_dev, &data_config, &buf_set);
	} else if (dbi_config->mode == MIPI_DBI_MODE_SPI_4WIRE) {
		/* 4 wire mode is much simpler. We just toggle the
		 * command/data GPIO to indicate if we are sending
		 * a command or data
		 */
		buffer.buf = &cmd;
		buffer.len = sizeof(cmd);

		/* Set CD pin low for command */
		gpio_pin_set_dt(&config->cmd_data, 0);
		ret = spi_write(config->spi_dev, &dbi_config->config, &buf_set);
		if (ret < 0) {
			goto out;
		}

		buffer.buf = (void *)response;
		buffer.len = len;

		/* Set CD pin high for data */
		gpio_pin_set_dt(&config->cmd_data, 1);
		ret = spi_read(config->spi_dev, &dbi_config->config,
			       &buf_set);
	} else {
		/* Otherwise, unsupported mode */
		ret = -ENOTSUP;
	}
out:
	k_spin_unlock(&data->lock, spinlock_key);
	return ret;
}

#endif /* MIPI_DBI_SPI_READ_REQUIRED */

static int mipi_dbi_spi_reset(const struct device *dev, uint32_t delay)
{
	const struct mipi_dbi_spi_config *config = dev->config;
	int ret;

	if (config->reset.port == NULL) {
		return -ENOTSUP;
	}

	ret = gpio_pin_set_dt(&config->reset, 0);
	if (ret < 0) {
		return ret;
	}
	k_msleep(delay);
	return gpio_pin_set_dt(&config->reset, 1);
}

static int mipi_dbi_spi_init(const struct device *dev)
{
	const struct mipi_dbi_spi_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->spi_dev)) {
		LOG_ERR("SPI device is not ready");
		return -ENODEV;
	}

	if (config->cmd_data.port) {
		ret = gpio_pin_configure_dt(&config->cmd_data, GPIO_OUTPUT);
		if (ret < 0) {
			LOG_ERR("Could not configure command/data GPIO (%d)", ret);
			return ret;
		}
	}

	if (config->reset.port) {
		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}
	}

	return 0;
}

static struct mipi_dbi_driver_api mipi_dbi_spi_driver_api = {
	.reset = mipi_dbi_spi_reset,
	.command_write = mipi_dbi_spi_command_write,
	.write_transfer = mipi_dbi_spi_write_transfer,
#if MIPI_DBI_SPI_READ_REQUIRED
	.command_read = mipi_dbi_spi_command_read,
#endif
};

#define MIPI_DBI_SPI_INIT(n)							\
	static const struct mipi_dbi_spi_config					\
	    mipi_dbi_spi_config_##n = {						\
		    .spi_dev = DEVICE_DT_GET(					\
				    DT_INST_PHANDLE(n, spi_dev)),		\
		    .cmd_data = GPIO_DT_SPEC_INST_GET_OR(n, dc_gpios, {}),	\
		    .reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {}),	\
	};									\
	static struct mipi_dbi_spi_data mipi_dbi_spi_data_##n;			\
										\
	DEVICE_DT_INST_DEFINE(n, mipi_dbi_spi_init, NULL,			\
			&mipi_dbi_spi_data_##n,					\
			&mipi_dbi_spi_config_##n,				\
			POST_KERNEL,						\
			CONFIG_MIPI_DBI_INIT_PRIORITY,				\
			&mipi_dbi_spi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MIPI_DBI_SPI_INIT)
