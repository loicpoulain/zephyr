/**
 * Copyright (c) 2018 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_WIFI_LEVEL
#define SYS_LOG_DOMAIN "dev/eswifi"
#if (SYS_LOG_LEVEL > SYS_LOG_LEVEL_OFF)
#define NET_LOG_ENABLED 1
#endif

#include <logging/sys_log.h>

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <string.h>
#include <errno.h>
#include <gpio.h>
#include <spi.h>

#include "eswifi.h"

#define ESWIFI_SPI_THREAD_STACK_SIZE 1024
K_THREAD_STACK_MEMBER(eswifi_spi_poll_stack, ESWIFI_SPI_THREAD_STACK_SIZE);

struct eswifi_spi_data {
	struct device *dev;
	struct eswifi_gpio csn;
	struct eswifi_gpio dr;
	struct k_thread poll_thread;
};

static struct eswifi_spi_data eswifi_spi0; /* Static instance */

static struct spi_config spi_conf = {
	.frequency = 2000000,
	.operation = (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(16) |
		      SPI_LINES_SINGLE),
	.slave     = 0,
	.cs	   = NULL,
};

static inline void eswifi_spi_cs(struct eswifi_spi_data *spi, bool select)
{
	gpio_pin_write(spi->csn.dev, spi->csn.pin, select ? 0 : 1);
	k_sleep(10);
}

static bool eswifi_spi_cmddata_ready(struct eswifi_spi_data *spi)
{
	int value;

	gpio_pin_read(spi->dr.dev, spi->dr.pin, &value);

	return value ? true : false;
}

static int eswifi_spi_wait_cmddata_ready(struct eswifi_spi_data *spi)
{
	do {
		k_sleep(1);
	} while (!eswifi_spi_cmddata_ready(spi));

	return 0;
}

static int eswifi_spi_write(struct eswifi_dev *eswifi, char *data, size_t dlen)
{
	struct eswifi_spi_data *spi = eswifi->bus_data;
	struct spi_buf spi_tx_buf[1];
	struct spi_buf_set spi_tx;
	int status;

	eswifi_spi_wait_cmddata_ready(spi);

	spi_tx_buf[0].buf = data;
	spi_tx_buf[0].len = dlen;
	spi_tx.buffers = spi_tx_buf;
	spi_tx.count = ARRAY_SIZE(spi_tx_buf);

	k_sleep(10);

	status = spi_write(spi->dev, &spi_conf, &spi_tx);
	if (status) {
		SYS_LOG_ERR("SPI write error %d", status);
	} else {
		status = dlen;
	}

	return status;
}

static int eswifi_spi_read(struct eswifi_dev *eswifi, char *data, size_t dlen)
{
	struct eswifi_spi_data *spi = eswifi->bus_data;
	struct spi_buf spi_rx_buf[1];
	struct spi_buf_set spi_rx;
	int status;

	spi_rx_buf[0].buf = data;
	spi_rx_buf[0].len = dlen / 2; /* number of words ? */
	spi_rx.buffers = spi_rx_buf;
	spi_rx.count = ARRAY_SIZE(spi_rx_buf);

	if (!eswifi_spi_cmddata_ready(spi)) {
		return 0;
	}
	k_sleep(10);

	status = spi_read(spi->dev, &spi_conf, &spi_rx);
	if (status) {
		SYS_LOG_ERR("SPI read error %d", status);
	} else {
		status = dlen;
	}

	return status;
}

static int eswifi_spi_request(struct eswifi_dev *eswifi, char *cmd, size_t clen,
			      char *rsp, size_t rlen)
{
	struct eswifi_spi_data *spi = eswifi->bus_data;
	char tmp[2];

	SYS_LOG_DBG("cmd=%p (%u byte), rsp=%p (%u byte)", cmd, clen, rsp, rlen);

	if (!cmd)
		goto data;

	/* CMD/DATA READY pin signals the Command Phase */
	eswifi_spi_wait_cmddata_ready(spi);

	/* Start */
	eswifi_spi_cs(spi, true);

	if (clen % 2) { /* Add padding if necessary */
		/* cmd is a string so cmd[clen] is 0x00 */
		cmd[clen] = 0x0a;
		clen++;
	}

	eswifi_spi_write(eswifi, cmd, clen);

	/* End of cmd */
	eswifi_spi_cs(spi, false);

data:
	eswifi_spi_wait_cmddata_ready(spi);
	eswifi_spi_cs(spi, true);
	eswifi_spi_read(eswifi, rsp, rlen);
	k_sleep(1);

	while (eswifi_spi_cmddata_ready(spi)) {
		eswifi_spi_read(eswifi, tmp, 2);
		k_sleep(1);
	}

	eswifi_spi_cs(spi, false);

	SYS_LOG_DBG("success");

	return 0;
}

static void eswifi_spi_read_msg(struct eswifi_dev *eswifi)
{
	char cmd[] = "MR\r";
	int err;

	eswifi_lock(eswifi);

	err = eswifi_request(eswifi, cmd, strlen(cmd),
			     eswifi->buf, sizeof(eswifi->buf));
	if (err || !__is_at_ok(eswifi->buf)) {
		SYS_LOG_ERR("Unable to read msg %d %s",err, eswifi->buf);
		eswifi_unlock(eswifi);
	}

	eswifi_unlock(eswifi);
}

static void eswifi_spi_poll_thread(void *p1)
{
	struct eswifi_dev *eswifi = p1;

	while (1) {
		k_sleep(K_MSEC(1000));
		eswifi_spi_read_msg(eswifi);
	}
}

int eswifi_spi_init(struct eswifi_dev *eswifi) {
	struct eswifi_spi_data *spi = &eswifi_spi0; /* Static instance */

	/* SPI DEV */
	spi->dev = device_get_binding("SPI_3");
	if (!spi->dev) {
		SYS_LOG_ERR("Failed to initialize SPI driver");
		return -ENODEV;
	}

	/* SPI DATA READY PIN */
	spi->dr.dev = device_get_binding(ESWIFI0_DATA_GPIOS_CONTROLLER);
	if (!spi->dr.dev) {
		SYS_LOG_ERR("Failed to initialize GPIO driver: %s",
			    ESWIFI0_DATA_GPIOS_CONTROLLER);
		return -ENODEV;
	}
	spi->dr.pin = ESWIFI0_DATA_GPIOS_PIN;
	gpio_pin_configure(spi->dr.dev, spi->dr.pin, GPIO_DIR_IN);

	/* SPI CHIP SELECT PIN */
	spi->csn.dev = device_get_binding(ESWIFI0_CSN_GPIOS_CONTROLLER);
	if (!spi->csn.dev) {
		SYS_LOG_ERR("Failed to initialize GPIO driver: %s",
			    ESWIFI0_CSN_GPIOS_CONTROLLER);
		return -ENODEV;
	}
	spi->csn.pin = ESWIFI0_CSN_GPIOS_PIN;
	gpio_pin_configure(spi->csn.dev, spi->csn.pin, GPIO_DIR_OUT);

	eswifi->bus_data = spi;

	SYS_LOG_DBG("success");

	k_thread_create(&spi->poll_thread, eswifi_spi_poll_stack,
			ESWIFI_SPI_THREAD_STACK_SIZE,
			(k_thread_entry_t)eswifi_spi_poll_thread, eswifi, NULL,
			NULL, K_PRIO_COOP(CONFIG_WIFI_ESWIFI_THREAD_PRIO), 0,
			K_NO_WAIT);

	return 0;
}

struct eswifi_bus_ops eswifi_bus_ops_spi = {
	.init = eswifi_spi_init,
	.read = eswifi_spi_read,
	.write = eswifi_spi_write,
	.request = eswifi_spi_request,
};
