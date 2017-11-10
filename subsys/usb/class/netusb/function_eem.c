/*
 * Copyright (c) 2017 Linaro Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define NET_LOG_ENABLED 1
#define SYS_LOG_LEVEL 3
#define SYS_LOG_DOMAIN "function/eem"
#include <logging/sys_log.h>

#include <net_private.h>
#include <zephyr.h>
#include <usb_device.h>
#include <usb_common.h>

#include <net/net_pkt.h>

#include "netusb.h"

struct eem_cmd_pkt_hdr {
        u16_t bm_cmd_param:11;
        u16_t bm_cmd:3;
        u16_t bm_res:1;
        u16_t bm_type:1;
} __packed;

enum {
        CMD_ECHO,
        CMD_ECHO_RSP,
        CMD_SUSPEND_HINT,
        CMD_RSP_HINT,
        CMD_RSP_CMPLT_HINT,
        CMD_TICKLE,
};

static struct net_pkt *rx_pkt;
static u8_t tx_buf[1500]; /* MTU SIZE */
static u16_t rx_hdr;
static u16_t rx_exp;

static inline u16_t eem_pkt_size(u16_t hdr)
{
        if (hdr & BIT(15)) {
                return hdr & 0x07ff;
        } else {
                return (hdr & 0x3fff);
        }
}

static int eem_send(struct net_pkt *pkt)
{
        u8_t sentinel[4] = { 0xde, 0xad, 0xbe, 0xef };
        u16_t *hdr = (u16_t *)&tx_buf[0];
        struct net_buf *frag;
        int len, b_idx = 0;

        len = net_pkt_ll_reserve(pkt) + net_pkt_get_len(pkt) + sizeof(sentinel);

        /* EEM packets can be split across USB packets but shall not be split
         * across USB transfers. Current usb_write implementation supports
         * only one buffer by transfer.
         */

        /* Add EEM header */
        *hdr = sys_cpu_to_le16(0x3FFF & len);
        b_idx += sizeof(u16_t);

        /* Add Ethernet Header */
        memcpy(&tx_buf[b_idx], net_pkt_ll(pkt), net_pkt_ll_reserve(pkt));
        b_idx += net_pkt_ll_reserve(pkt);

        /* generate transfer buffer */
        for (frag = pkt->frags; frag; frag = frag->frags) {
                memcpy(&tx_buf[b_idx], frag->data, frag->len);
                b_idx += frag->len;
        }

        /* Add crc-sentinel */
        memcpy(&tx_buf[b_idx], sentinel, sizeof(sentinel));
        b_idx += sizeof(sentinel);

        /* start transfer */
        usb_write(CONFIG_CDC_EEM_IN_EP_ADDR, tx_buf, b_idx, NULL);

        return 0;
}

static void net_pkt_trim(struct net_pkt *pkt, size_t len)
{
        struct net_buf *frag;
        int total = 0;

        for (frag = pkt->frags; frag; frag = frag->frags) {
                total += frag->len;
                if (total > len) {
                        frag->len -= total - len;
                        break;
                }
        }

        /* remove all following frags */
        while (frag && frag->frags) {
                net_pkt_frag_del(pkt, frag, frag->frags);
        }
}

static void eem_recv_cmd(struct net_pkt *pkt)
{

}

static void eem_bulk_out(u8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
        do {
                u8_t buf[CONFIG_CDC_EEM_BULK_EP_MPS];
                u32_t rlen;

                /* TODO: don't use extra buf but directly read to net_buf(s) */
                /* TODO: move this in thread context !? */

                if ((rx_exp == 0) && (rx_pkt == NULL)) { /* new pkt */
                        /* read hdr */
                        usb_read(ep, (void *)&rx_hdr, sizeof(rx_hdr), &rlen);
                        if (rlen == 0)
                                break;

                        rx_hdr = sys_le16_to_cpu(rx_hdr);
                        rx_exp = eem_pkt_size(rx_hdr);

                        rx_pkt = net_pkt_get_reserve_rx(0, K_NO_WAIT);
                        if (!rx_pkt) {
                                SYS_LOG_ERR("Unable to alloc net pkt");
                                /* skip rest of the packet (rx_exp) */
                        }
                }

                usb_read(ep, buf, min(sizeof(buf), rx_exp), &rlen);
                if (rlen == 0) { /* no more data to read */
                        break;
                }

                rx_exp -= rlen;

                if (!rx_pkt) { /* skipped data */
                        continue;
                }

                if (!net_pkt_append_all(rx_pkt, rlen, buf, K_NO_WAIT)) {
                        SYS_LOG_ERR("Not enough space, discard packet...");
                        net_pkt_unref(rx_pkt);
                        rx_pkt = NULL;
                        return;
                }

                if (rx_exp == 0) { /* pkt complete */
                        if (rx_hdr & BIT(15)) {
                                eem_recv_cmd(rx_pkt);
                                net_pkt_unref(rx_pkt);
                        } else {
                                /* Remove cdc-sentinel */
                                net_pkt_trim(rx_pkt,
                                             net_pkt_get_len(rx_pkt) - 4);
                                netusb_recv(rx_pkt);
                        }
                        rx_exp = 0;
                        rx_pkt = NULL;
                }
        } while (1);
}

static void eem_bulk_in(u8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
#if VERBOSE_DEBUG
	SYS_LOG_DBG("EP 0x%x status %d", ep, ep_status);
#endif
}

static struct usb_ep_cfg_data eem_ep_data[] = {
	{
		.ep_cb = eem_bulk_out,
		.ep_addr = CONFIG_CDC_EEM_OUT_EP_ADDR
	},
	{
		.ep_cb = eem_bulk_in,
		.ep_addr = CONFIG_CDC_EEM_IN_EP_ADDR
	},
};

struct netusb_function eem_function = {
	.connect_media = NULL,
	.class_handler = NULL,
	.send_pkt = eem_send,
	.num_ep = ARRAY_SIZE(eem_ep_data),
	.ep = eem_ep_data,
};
