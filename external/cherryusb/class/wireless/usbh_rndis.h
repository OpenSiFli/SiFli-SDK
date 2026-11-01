/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBH_RNDIS_H
#define USBH_RNDIS_H

#include "usb_cdc.h"

struct usbh_rndis {
    struct usbh_hubport *hport;
    struct usb_endpoint_descriptor *bulkin;  /* Bulk IN endpoint */
    struct usb_endpoint_descriptor *bulkout; /* Bulk OUT endpoint */
    struct usb_endpoint_descriptor *intin;   /* INTR endpoint */
    struct usbh_urb bulkin_urb;              /* Bulk IN urb */
    struct usbh_urb bulkout_urb;             /* Bulk OUT urb */
    struct usbh_urb intin_urb;               /* INTR IN urb */

    uint8_t ctrl_intf; /* Control interface number */
    uint8_t data_intf; /* Data interface number */
    uint8_t minor;

    uint32_t request_id;
    uint32_t tx_offset;
    uint32_t max_transfer_pkts; /* max packets in one transfer */
    uint32_t max_transfer_size; /* max size in one transfer */

    uint32_t link_speed;
    bool connect_status;
    uint8_t mac[6];

    void *user_data;
};

#ifdef __cplusplus
extern "C" {
#endif

int usbh_rndis_get_connect_status(struct usbh_rndis *rndis_class);
/* Cached RNDIS link state maintained by the rx thread; not a live query. */
int usbh_rndis_is_link_up(void);
int usbh_rndis_keepalive(struct usbh_rndis *rndis_class);

#ifdef USBHOST_RNDIS_NO_NETIF
/**
 * @brief Weak hook called in the usbh_rndis_rx thread for every ethernet
 *        frame received over RNDIS when the lwIP netif is not registered
 *        (USBHOST_RNDIS_NO_NETIF).
 *
 * The frame must be consumed synchronously during the call: @p buf is only
 * valid until the hook returns (the driver reuses the receive buffer for the
 * next transfer), and the hook must not block, or the USB bulk IN path
 * stalls. The default weak implementation drops the frame; bridge
 * applications override it to forward the frame.
 *
 * @param buf ethernet frame data, starting at the ethernet header; valid
 *            only during the call
 * @param len frame length in bytes
 */
void usbh_rndis_on_raw_rx(uint8_t *buf, uint32_t len);
#endif

void usbh_rndis_run(struct usbh_rndis *rndis_class);
void usbh_rndis_stop(struct usbh_rndis *rndis_class);

uint8_t *usbh_rndis_get_eth_txbuf(void);
int usbh_rndis_eth_output(uint32_t buflen);
void usbh_rndis_eth_input(uint8_t *buf, uint32_t buflen);
void usbh_rndis_rx_thread(CONFIG_USB_OSAL_THREAD_SET_ARGV);

#ifdef __cplusplus
}
#endif

#endif /* USBH_RNDIS_H */
