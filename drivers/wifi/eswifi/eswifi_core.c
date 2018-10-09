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
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/net_l2.h>
#include <net/net_context.h>
#include <net/net_offload.h>
#include <net/wifi_mgmt.h>

#include <net/ethernet.h>
#include <net_private.h>
#include <net/net_core.h>
#include <net/net_pkt.h>
#include <net/net_l2.h>

#include <stdio.h>
#include <stdlib.h>

#include <misc/printk.h>

#include "eswifi.h"

#define ESWIFI_WORKQUEUE_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(eswifi_work_q_stack, ESWIFI_WORKQUEUE_STACK_SIZE);

static struct eswifi_dev eswifi0; /* static instance */

static int eswifi_reset(struct eswifi_dev *eswifi)
{
	gpio_pin_write(eswifi->resetn.dev, eswifi->resetn.pin, 0);
	k_sleep(10);
	gpio_pin_write(eswifi->resetn.dev, eswifi->resetn.pin, 1);
	gpio_pin_write(eswifi->wakeup.dev, eswifi->wakeup.pin, 1);
	k_sleep(500);

	/* fetch the cursor */
	eswifi_request(eswifi, NULL, 0, eswifi->buf, sizeof(eswifi->buf));

	return 0;
}

static inline int __parse_ssid(char *str, char *ssid)
{
	/* fnt => '"SSID"' */

	if (!*str || (*str != '"'))
		return -EINVAL;

	str++;
	while (*str && (*str != '"')) {
		*ssid++ = *str++;
	}

	*ssid = '\0';

	if (*str != '"')
		return -EINVAL;

	return -EINVAL;
}

static void __parse_scan_res(char *str, struct wifi_scan_result *res)
{
	int field = 0;

	/* fmt => #001,"SSID",F4:CA:E5:E7:5E:2C,-94,54.0,Infrastructure,WEP,2.4GHz,1 */

	while (*str) {
		if (*str != ','){
			str++;
			continue;
		}

		if (!*++str)
			break;

		switch (++field) {
		case 1: /* SSID */
			__parse_ssid(str, res->ssid);
			res->ssid_length = strlen(res->ssid);
			str += res->ssid_length;
		case 2: /* mac addr */
			break;
		case 3: /* RSSI */
			res->rssi = atoi(str);
			break;
		case 4: /* bitrate */
			break;
		case 5: /* mode */
			break;
		case 6: /* security */
			if (!strncmp(str, "Open", 4)) {
				res->security = WIFI_SECURITY_TYPE_NONE;
			} else {
				res->security = WIFI_SECURITY_TYPE_PSK;
			}
			break;
		case 7: /* band */
			break;
		case 8: /* channel */
			res->channel = atoi(str);
			break;
		}

	}
}

struct eswifi_dev *eswifi_by_iface_idx(u8_t iface)
{
	/* only one instance */
	SYS_LOG_DBG("%d", iface);
	return &eswifi0;
}

static int __parse_ipv4_address(char *str, char *ssid, uint8_t ip[4])
{
	unsigned int byte = -1;

	/* fmt => [JOIN   ] SSID,192.168.2.18,0,0 */
	while (*str) {
		if (byte == -1) {
			if (!strncmp(str, ssid, strlen(ssid))) {
				byte = 0;
				str += strlen(ssid);
			}
			str++;
			continue;
		}

		ip[byte++] = atoi(str);
		while (*str && *str++ != '.');
	}

	return 0;
}

bool __is_at_ok(char *str)
{
	char at_ok[] = "OK\r\n> ";

	while (*str) {
		if (!strncmp(str++, at_ok, sizeof(at_ok) - 1))
			return true;
	}

	return false;
}

static int eswifi_scan(struct eswifi_dev *eswifi)
{
	char cmd[] = "F0\r";
	int i;

	SYS_LOG_DBG("");

	eswifi_lock(eswifi);

	eswifi_request(eswifi, cmd, strlen(cmd), eswifi->buf, sizeof(eswifi->buf));

	for (i = 0; i < sizeof(eswifi->buf); i++) {
		if (eswifi->buf[i] == '#') {
			struct wifi_scan_result res = {0};

			__parse_scan_res(&eswifi->buf[i], &res);

			eswifi->scan_cb(eswifi->iface, 0, &res);
			k_yield();

			while (eswifi->buf[i] != '\n')
				i++;
		}
	}

	eswifi_unlock(eswifi);

	return 0;
}

static int eswifi_connect(struct eswifi_dev *eswifi)
{
	char connect[] = "C0\r";
	struct in_addr addr;
	int err;

	SYS_LOG_DBG("Connecting to %s (pass=%s)", eswifi->sta.ssid,
		    eswifi->sta.pass);

	eswifi_lock(eswifi);

	/* Set SSID */
	sprintf(eswifi->buf, "C1=%s\r", eswifi->sta.ssid);
	err = eswifi_request(eswifi, eswifi->buf, strlen(eswifi->buf),
			     eswifi->buf, sizeof(eswifi->buf));
	if (err || !__is_at_ok(eswifi->buf)) {
		SYS_LOG_ERR("Unable to set SSID");
		goto error;
	}

	/* Set passphrase */
	sprintf(eswifi->buf, "C2=%s\r", eswifi->sta.pass);
	err = eswifi_request(eswifi, eswifi->buf, strlen(eswifi->buf),
			     eswifi->buf, sizeof(eswifi->buf));
	if (err || !__is_at_ok(eswifi->buf)) {
		SYS_LOG_ERR("Unable to set passphrase");
		goto error;
	}

	/* Set Security type */
	sprintf(eswifi->buf, "C3=%u\r", eswifi->sta.security);
	err = eswifi_request(eswifi, eswifi->buf, strlen(eswifi->buf),
			     eswifi->buf, sizeof(eswifi->buf));
	if (err || !__is_at_ok(eswifi->buf)) {
		SYS_LOG_ERR("Unable to configure security");
		goto error;
	}

	/* Join Network */
	err = eswifi_request(eswifi, connect, strlen(connect),
			     eswifi->buf, sizeof(eswifi->buf));
	if (err || !__is_at_ok(eswifi->buf)) {
		SYS_LOG_ERR("Unable to join network");
		goto error;
	}

	/* Any IP assigned ? (dhcp offload or manually) */
	err = __parse_ipv4_address(eswifi->buf, eswifi->sta.ssid,
				   (uint8_t *)&addr.s4_addr);
	if (err) {
		SYS_LOG_ERR("Unable to retrieve IP address");
		goto error;
	}

	SYS_LOG_DBG("ip = %d.%d.%d.%d", addr.s4_addr[0], addr.s4_addr[1],
		    addr.s4_addr[2], addr.s4_addr[3]);

	net_if_ipv4_addr_add(eswifi->iface, &addr, NET_ADDR_DHCP, 0);

	SYS_LOG_DBG("Connected!");

	eswifi_unlock(eswifi);
	return 0;

error:
	eswifi_unlock(eswifi);
	return -EIO;
}

static int eswifi_disconnect(struct eswifi_dev *eswifi)
{
	char disconnect[] = "CD\r";
	int err;

	SYS_LOG_DBG("");

	eswifi_lock(eswifi);

	err = eswifi_request(eswifi, disconnect, strlen(disconnect),
			     eswifi->buf, sizeof(eswifi->buf));
	if (err || !__is_at_ok(eswifi->buf)) {
		SYS_LOG_ERR("Unable to disconnect network");
		eswifi_unlock(eswifi);
		return -EIO;
	}

	eswifi_unlock(eswifi);

	return 0;
}

static void eswifi_request_work(struct k_work *item)
{
	struct eswifi_dev *eswifi;
	int err;

	SYS_LOG_DBG("");

	eswifi = CONTAINER_OF(item, struct eswifi_dev, request_work);

	switch (eswifi->req) {
	case ESWIFI_REQ_CONNECT:
		err = eswifi_connect(eswifi);
		wifi_mgmt_raise_connect_result_event(eswifi->iface, err);
		break;
	case ESWIFI_REQ_DISCONNECT:
		err = eswifi_disconnect(eswifi);
		wifi_mgmt_raise_disconnect_result_event(eswifi->iface, err);
		break;
	case ESWIFI_REQ_SCAN:
		eswifi_scan(eswifi);
		break;
	case ESWIFI_REQ_NONE:
	default:
		break;
	}
}

static int eswifi_get_mac_addr(struct eswifi_dev *eswifi, uint8_t addr[6])
{
	char cmd[] = "Z5\r";
	int err, i, byte = 0;

	err = eswifi_request(eswifi, cmd, strlen(cmd), eswifi->buf,
			     sizeof(eswifi->buf));
	if (err)
		return err;

	for (i = 0; i < sizeof(eswifi->buf); i++) {
		if (i < 2)
			continue;

		if (eswifi->buf[i] == ':' ||
		    ((byte == 5) && (eswifi->buf[i] == '\r'))) {
			addr[byte++] = strtol(&eswifi->buf[i - 2], NULL, 16);
		}
	}

	if (byte != 6)
		return -EIO;

	return 0;
}

static void eswifi_iface_init(struct net_if *iface)
{
	struct eswifi_dev *eswifi = &eswifi0;
	uint8_t mac[6];
	int err;

	SYS_LOG_ERR("IFACE INIT %p %p\n", iface, iface->if_dev);

	err = eswifi_reset(eswifi);
	if (err) {
		SYS_LOG_ERR("Unable to reset device");
		return;
	}

	err = eswifi_get_mac_addr(eswifi, mac);
	if (err) {
		SYS_LOG_ERR("Unable to read MAC address");
		return;
	}

	SYS_LOG_DBG("MAC Address %02X:%02X:%02X:%02X:%02X:%02X",
		    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	net_if_set_link_addr(iface, mac, sizeof(mac), NET_LINK_ETHERNET);

	eswifi->iface = iface;
	eswifi_offload_init(eswifi);
}

static int eswifi_mgmt_scan(struct device *dev, scan_result_cb_t cb)
{
	struct eswifi_dev *eswifi = dev->driver_data;

	SYS_LOG_DBG("");

	eswifi_lock(eswifi);
	eswifi->scan_cb = cb;
	eswifi->req = ESWIFI_REQ_SCAN;
	eswifi_unlock(eswifi);

	k_work_submit_to_queue(&eswifi->work_q, &eswifi->request_work);

	return 0;
}

static int eswifi_mgmt_disconnect(struct device *dev)
{
	struct eswifi_dev *eswifi = dev->driver_data;

	SYS_LOG_DBG("");

	eswifi_lock(eswifi);
	eswifi->req = ESWIFI_REQ_DISCONNECT;
	eswifi_unlock(eswifi);

	k_work_submit_to_queue(&eswifi->work_q, &eswifi->request_work);

	return 0;
}

static int eswifi_mgmt_connect(struct device *dev,
			       struct wifi_connect_req_params *params)
{
	struct eswifi_dev *eswifi = dev->driver_data;

	SYS_LOG_DBG("");

	eswifi_lock(eswifi);

	memcpy(eswifi->sta.ssid, params->ssid, params->ssid_length);
	eswifi->sta.ssid[params->ssid_length] = '\0';

	switch (params->security) {
	case WIFI_SECURITY_TYPE_NONE:
		eswifi->sta.pass[0] = '\0';
		eswifi->sta.security = ESWIFI_SEC_OPEN;
		break;
	case WIFI_SECURITY_TYPE_PSK:
		memcpy(eswifi->sta.pass, params->psk, params->psk_length);
		eswifi->sta.pass[params->psk_length] = '\0';
		eswifi->sta.security = ESWIFI_SEC_WPA2_MIXED;
		break;
	default:
		SYS_LOG_ERR("Unsupported security type %d", params->security);
		eswifi_unlock(eswifi);
		return -EINVAL;
	}

	eswifi->req = ESWIFI_REQ_CONNECT;
	k_work_submit_to_queue(&eswifi->work_q, &eswifi->request_work);

	eswifi_unlock(eswifi);

	return 0;
}

static int eswifi_init(struct device *dev)
{
	struct eswifi_dev *eswifi = dev->driver_data;

	SYS_LOG_DBG("");

	eswifi->role = ESWIFI_ROLE_CLIENT;
	k_mutex_init(&eswifi->mutex);

	eswifi->bus = &eswifi_bus_ops_spi;
	eswifi->bus->init(eswifi);

	eswifi->resetn.dev = device_get_binding(ESWIFI0_RESETN_GPIOS_CONTROLLER);
	if (!eswifi->resetn.dev) {
		SYS_LOG_ERR("Failed to initialize GPIO driver: %s",
			    ESWIFI0_RESETN_GPIOS_CONTROLLER);
	}
	eswifi->resetn.pin = ESWIFI0_RESETN_GPIOS_PIN;
	gpio_pin_configure(eswifi->resetn.dev, eswifi->resetn.pin, GPIO_DIR_OUT);

	eswifi->wakeup.dev = device_get_binding(ESWIFI0_WAKEUP_GPIOS_CONTROLLER);
	if (!eswifi->wakeup.dev) {
		SYS_LOG_ERR("Failed to initialize GPIO driver: %s",
			    ESWIFI0_WAKEUP_GPIOS_CONTROLLER);
	}
	eswifi->wakeup.pin = ESWIFI0_WAKEUP_GPIOS_PIN;
	gpio_pin_configure(eswifi->wakeup.dev, eswifi->wakeup.pin,
			   GPIO_DIR_OUT);
	gpio_pin_write(eswifi->wakeup.dev, eswifi->wakeup.pin, 1);

	k_work_q_start(&eswifi->work_q, eswifi_work_q_stack,
		       K_THREAD_STACK_SIZEOF(eswifi_work_q_stack),
		       CONFIG_SYSTEM_WORKQUEUE_PRIORITY - 1);

	k_work_init(&eswifi->request_work, eswifi_request_work);

	return 0;
}

static const struct net_wifi_mgmt_offload eswifi_offload_api = {
	.iface_api.init = eswifi_iface_init,
	.iface_api.send = NULL,
	.scan		= eswifi_mgmt_scan,
	.connect	= eswifi_mgmt_connect,
	.disconnect	= eswifi_mgmt_disconnect,
};

NET_DEVICE_OFFLOAD_INIT(eswifi_mgmt, CONFIG_WIFI_ESWIFI_NAME,
			eswifi_init, &eswifi0, NULL,
			CONFIG_WIFI_INIT_PRIORITY, &eswifi_offload_api, 1500);
