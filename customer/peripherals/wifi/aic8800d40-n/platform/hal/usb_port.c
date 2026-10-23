/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB platform port stub implementation (Plan B).
 *
 * Compared with the Linux reference driver (aicwf_usb.c), USB does not need the following SDIO concepts:
 *   - CMD52/CMD53 register access (USB uses bulk URBs)
 *   - claim_host/release_host (USB has no SDIO host concept)
 *   - readsb/writesb (USB uses URB bulk in/out)
 *   - release_ep2 (SDIO function 2 release, no such concept in USB)
 *   - host_enable_isr/set_irq_handler/interrupt_init/isr (USB uses URB completion)
 *
 * This file implements the platform interfaces declared in usb_port.h (CO -> Port downstream).
 * Port -> CO upstream callbacks (aicwf_usb_rx_complete / aicwf_usb_tx_complete)
 * are implemented in usb_co.c and invoked by the Port layer on URB completion.
 *
 * Each interface is accompanied by a Linux reference implementation (wrapped in #if 0) for comparison during real implementation.
 * Reference Linux source: aicwf_usb.c (under android_driver/USB/driver_fw/.../aic8800_fdrv/)
 */

// ==========================================================================
// [PLATFORM] RT-Thread USB Host headers
// ==========================================================================
#include <rtthread.h>
#include <usb.h>
#include <hcd.h>
#include "sys_api.h"
// ==========================================================================
// [PLATFORM] AIC project internal headers below, normally no need to modify
// ==========================================================================
#include "usb_port.h"
#include "usb_def.h"
#include "plat_port.h"
#include "rtos_port.h"
#include <errno.h>
#include <string.h>
#include "aic_log.h"

/* Boundary: the platform layer must never include host internal headers (layer_common/inc, etc.).
 * Port -> CO upstream interfaces (aicwf_usb_rx_complete / aicwf_usb_tx_complete, etc.)
 * are implemented by host usb_co.c; the symbols live in the driver library; declarations go in usb_port.h (public porting interface).
 * TODO: [AIC8800-PORT] complete the following in usb_co.c during USB bring-up:
 *       rx_buf_get/put, tx_queue, tx_flowctrl, rx_submit_all_urb_ symbols,
 *       and declare them publicly in usb_port.h. */

#ifdef CONFIG_USB_SUPPORT

/* USB device IDs for AIC8800 series
 * Boot ROM:  a69c:8d80  → load FW → soft disconnect
 * Operational: a69c:8d81 (WiFi+BT) / a69c:8d83 (WiFi only)
 * Keep legacy 349d:* IDs for older modules. */
#define AIC_VENDOR_ID               0xa69c
#define AIC_VENDOR_ID_AIC_V2        0x368b   /* AIC8800D80X2 and newer batches (V2) */
#define AIC_PRODUCT_ID_D80          0x8d80   /* bootrom */
#define AIC_PRODUCT_ID_D80_WIFI_BT  0x8d81   /* after FW */
#define AIC_PRODUCT_ID_D80_WIFI     0x8d83   /* after FW, WiFi only */
#define AIC_PRODUCT_ID_D80X2        0x8d90   /* AIC8800D80X2 (V2, 2T2R) */
#define AIC_PRODUCT_ID_D40X2        0x8d92   /* AIC8800D40X2 (V2), shares the D80X2 driver */
#define AIC_PRODUCT_ID_D80N         0x8d45   /* AIC8800D80N (Linux USB PID) */
#define AIC_VENDOR_ID_LEGACY        0x349d
#define AIC_PRODUCT_ID_D80_LEGACY   0x8800
#define AIC_PRODUCT_ID_DC           0xc18d
#define AIC_PRODUCT_ID_DW           0xc18d
#define AIC_PRODUCT_ID_DC_USB       0x88dc

/* TX URB watermarks (flow control) — tight embedded memory, start with a smaller pool for bring-up */
#define AICWF_USB_TX_LOW_WATER      8
#define AICWF_USB_TX_HIGH_WATER     48
#define AICWF_USB_TX_URBS           64
#define AICWF_USB_MAX_PKT_SIZE      2048

/* Number of RX URBs */
#define AICWF_USB_RX_URBS           16

/* AIC8800 USB endpoint default addresses (resolved dynamically at probe; these are the defaults) */
#define AIC_USB_BULK_IN_EP_ADDR     0x81
#define AIC_USB_BULK_OUT_EP_ADDR    0x01

uint32_t USB_DEVICEID = AIC_PRODUCT_ID_D80;

/* Export variables referenced by usb_port.h */
int  msgcfm_poll_en = 1;
uint8_t usb_unexcept_test = 0;
struct aic_usb_dev g_aic_usb_dev = {0,};

/* Runtime chipmatch globals + getter — the same symbol set as sdio_port.c;
 * only one *_port.c is built per bus type, so no duplicate definitions (declarations in sys_api.h).
 * Filled at probe time with the aicwf_usb_pid_to_chipid() result;
 * AICWF_CHIPID_UNMATCHED means no match; aicwf_get_chipid() in wifi_if.c
 * prefers the runtime match result. */
uint16_t g_runtime_chipid = AICWF_CHIPID_UNMATCHED;
uint16_t g_runtime_vid = 0;
uint16_t g_runtime_pid = 0;

uint16_t aicwf_get_runtime_chipid(void)
{
    return g_runtime_chipid;
}

/* RT-Thread USB device management */
/* RTOS primitives all go through the rtos_port abstraction (rtos_semaphore/aicwf_rtos_*);
 * RT-Thread only hosts the USB host stack (usb.h/hcd.h); rt_ primitives are no longer used directly */
static struct usb_device    *s_udev = NULL;
static struct usb_interface *s_intf = NULL;
static unsigned int         s_bulk_in_pipe = 0;
static unsigned int         s_bulk_out_pipe = 0;
/* 8d81 WLAN if has 4 bulk endpoints: the second pair is MSG (command/CFM), matching Linux MSG_EP */
static unsigned int         s_msg_in_pipe = 0;
static unsigned int         s_msg_out_pipe = 0;
static rtos_semaphore       s_msg_sema;
static int                  s_msg_busy = 0;
static int                  s_msg_finished = 0;

/* TX/RX buffers and URB pool */
static struct urb           *s_tx_urbs[AICWF_USB_TX_URBS];
static uint8_t              *s_tx_bufs[AICWF_USB_TX_URBS];
static uint8_t              s_tx_urb_busy[AICWF_USB_TX_URBS];
static rtos_semaphore       s_tx_free_sema;
static uint32_t             s_tx_free_count = AICWF_USB_TX_URBS;

static struct urb           *s_rx_urbs[AICWF_USB_RX_URBS];
static uint8_t              *s_rx_bufs[AICWF_USB_RX_URBS];
static uint8_t              s_rx_urb_busy[AICWF_USB_RX_URBS];
static unsigned int         s_rx_pipe[AICWF_USB_RX_URBS]; /* bulk_in or msg_in */
static int                  s_func_inited = 0;
/* After FW start_app the chip soft-disconnects and re-enumerates as 8d81/8d83 */
static volatile int         s_expect_reconnect = 0;
static rtos_semaphore       s_reconnect_sem;
static int                  s_reconnect_sem_inited = 0;

/* Prototype for the CO layer init function (called from probe) */
extern int aicwf_usb_dev_init(struct aic_usb_dev *usbdev);

extern unsigned char aicwifi_usb_rxdata_mode;

#define DRV_NAME_USB "AIC8800D_USB"

/* Use RT-Thread's usb_rcvbulkpipe / usb_sndbulkpipe from <usb.h>
 * (RT-Thread pipe format uses __create_pipe(dev,ep) with devnum in bits 8-14,
 *  endpoint in bits 15-18, unlike Linux's simpler format) */
#define usb_endpoint_num(ep_desc) ((ep_desc)->bEndpointAddress & 0x0f)
#define usb_endpoint_dir_in(ep_desc) (((ep_desc)->bEndpointAddress & USB_DIR_IN) != 0)
#define usb_endpoint_xfer_bulk(ep_desc) (((ep_desc)->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)

/* ================================================================ */
/*  CO -> Port downstream interface implementation                  */
/* ================================================================ */

/* ================================================================ */
/*  USB function initialization                                    */
/*  Corresponding Linux:                                           */
/*    aicwf_usb_probe (call order)                                 */
/*      -> aicwf_parse_usb       (endpoint parsing)                */
/*      -> aicwf_usb_init        (URB pool + sync primitives)      */
/* ================================================================ */

#if 0
/* ==================== Linux reference implementation ==================== */

/* (1) endpoint parsing — walk the interface descriptor to find bulk in/out endpoints
 *     set up bulk_in_pipe / bulk_out_pipe (optional msg_in_pipe / msg_out_pipe)
 *     no register is configured; endpoint numbers are discovered from the descriptor */
static int aicwf_parse_usb(struct aic_usb_dev *usb_dev, struct usb_interface *interface)
{
    struct usb_interface_descriptor *interface_desc;
    struct usb_host_interface *host_interface;
    struct usb_endpoint_descriptor *endpoint;
    struct usb_device *usb = usb_dev->udev;
    int i, endpoints;
    u8 endpoint_num;
    int ret = 0;

    usb_dev->bulk_in_pipe = 0;
    usb_dev->bulk_out_pipe = 0;
#ifdef CONFIG_USB_MSG_OUT_EP
    usb_dev->msg_out_pipe = 0;
#endif
#ifdef CONFIG_USB_MSG_IN_EP
    usb_dev->msg_in_pipe = 0;
#endif

    host_interface = &interface->altsetting[0];
    interface_desc = &host_interface->desc;
    endpoints = interface_desc->bNumEndpoints;

    if (usb->descriptor.bNumConfigurations != 1)
    {
        ret = -ENODEV;
        goto exit;
    }
#ifndef CONFIG_USB_BT
    if (usb->descriptor.bDeviceClass != 0x00)
    {
        ret = -ENODEV;
        goto exit;
    }
#endif
    if (usb->actconfig->desc.bNumInterfaces != 1)
    {
        ret = -ENODEV;
        goto exit;
    }

    for (i = 0; i < endpoints; i++)
    {
        endpoint = &host_interface->endpoint[i].desc;
        endpoint_num = usb_endpoint_num(endpoint);

        if (usb_endpoint_dir_in(endpoint) &&
                usb_endpoint_xfer_bulk(endpoint))
        {
            if (!usb_dev->bulk_in_pipe)
            {
                usb_dev->bulk_in_pipe = usb_rcvbulkpipe(usb, endpoint_num);
            }
#ifdef CONFIG_USB_MSG_IN_EP
            else if (!usb_dev->msg_in_pipe)
            {
                if (usb_dev->chipid != PRODUCT_ID_AIC8801)
                    usb_dev->msg_in_pipe = usb_rcvbulkpipe(usb, endpoint_num);
            }
#endif
        }

        if (usb_endpoint_dir_out(endpoint) &&
                usb_endpoint_xfer_bulk(endpoint))
        {
            if (!usb_dev->bulk_out_pipe)
            {
                usb_dev->bulk_out_pipe = usb_sndbulkpipe(usb, endpoint_num);
            }
#ifdef CONFIG_USB_MSG_OUT_EP
            else if (!usb_dev->msg_out_pipe)
            {
                usb_dev->msg_out_pipe = usb_sndbulkpipe(usb, endpoint_num);
            }
#endif
        }
    }
exit:
    return ret;
}

/* (2) RX URB allocation — AICWF_USB_RX_URBS=20 bulk IN URBs, added to rx_free_list */
static int aicwf_usb_alloc_rx_urb(struct aic_usb_dev *usb_dev)
{
    int i;
    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        struct aicwf_usb_buf *usb_buf = &usb_dev->usb_rx_buf[i];
        usb_buf->usbdev = usb_dev;
        usb_buf->urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!usb_buf->urb)
        {
            goto err;
        }
        list_add_tail(&usb_buf->list, &usb_dev->rx_free_list);
    }
    return 0;
err:
    aicwf_usb_free_urb(&usb_dev->rx_free_list, &usb_dev->rx_free_lock);
    return -ENOMEM;
}

/* (3) TX URB allocation — AICWF_USB_TX_URBS=200 bulk OUT URBs, added to tx_free_list
 *     an extra aggregation skb is allocated in CONFIG_USB_TX_AGGR mode */
static int aicwf_usb_alloc_tx_urb(struct aic_usb_dev *usb_dev)
{
    int i;
    for (i = 0; i < AICWF_USB_TX_URBS; i++)
    {
        struct aicwf_usb_buf *usb_buf = &usb_dev->usb_tx_buf[i];
        usb_buf->usbdev = usb_dev;
        usb_buf->urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!usb_buf->urb)
        {
            goto err;
        }
#ifdef CONFIG_USB_TX_AGGR
        usb_buf->skb = dev_alloc_skb(MAX_USB_AGGR_TXPKT_LEN);
#endif
        list_add_tail(&usb_buf->list, &usb_dev->tx_free_list);
        (usb_dev->tx_free_count)++;
    }
    return 0;
err:
    aicwf_usb_free_urb(&usb_dev->tx_free_list, &usb_dev->tx_free_lock);
    return -ENOMEM;
}

/* (4) URB pool + sync primitive initialization
 *     - init_waitqueue_head (msg_wait)
 *     - init_usb_anchor (rx_submitted)
 *     - spin_lock_init (tx_free_lock/tx_post_lock/rx_free_lock/tx_flow_lock)
 *     - INIT_LIST_HEAD (rx_free_list/tx_free_list/tx_post_list)
 *     - allocate the RX/TX URB pools + msg_out_urb
 *     - no register is configured */
static int aicwf_usb_init(struct aic_usb_dev *usb_dev)
{
    int ret = 0;

    usb_dev->tbusy = false;
    usb_dev->state = USB_DOWN_ST;

    init_waitqueue_head(&usb_dev->msg_wait);
    init_usb_anchor(&usb_dev->rx_submitted);

    spin_lock_init(&usb_dev->tx_free_lock);
    spin_lock_init(&usb_dev->tx_post_lock);
    spin_lock_init(&usb_dev->rx_free_lock);
    spin_lock_init(&usb_dev->tx_flow_lock);

    INIT_LIST_HEAD(&usb_dev->rx_free_list);
    INIT_LIST_HEAD(&usb_dev->tx_free_list);
    INIT_LIST_HEAD(&usb_dev->tx_post_list);

    atomic_set(&rx_urb_cnt, 0);
    usb_dev->tx_free_count = 0;
    usb_dev->tx_post_count = 0;

    ret = aicwf_usb_alloc_rx_urb(usb_dev);
    if (ret) goto error;

    ret = aicwf_usb_alloc_tx_urb(usb_dev);
    if (ret) goto error;

    usb_dev->msg_out_urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!usb_dev->msg_out_urb)
    {
        ret = -ENOMEM;  /* FIXME: original code used ENOMEM (positive), should be -ENOMEM */
        goto error;
    }

    INIT_WORK(&usb_dev->rx_urb_work, aicwf_usb_rx_urb_work);
    return ret;
error:
    aicwf_usb_deinit(usb_dev);
    return ret;
}
#endif /* 0 — end of Linux reference implementation */

static uint16_t aicwf_usb_pid_to_chipid(uint16_t vid, uint16_t pid)
{
    switch (pid)
    {
    case AIC_PRODUCT_ID_D80:
    case AIC_PRODUCT_ID_D80_WIFI_BT:
    case AIC_PRODUCT_ID_D80_WIFI:
    case AIC_PRODUCT_ID_D80_LEGACY:
        USB_DEVICEID = pid;
        return PRODUCT_ID_AIC8800D80;
    case AIC_PRODUCT_ID_D80X2:
    case AIC_PRODUCT_ID_D40X2: /* D40X2 (0x8D92) shares the driver with D80X2 (matching Linux) */
        /* Match Linux aicloadfw_chipmatch: X2 series PIDs must pair with VID 0x368B;
         * a VID mismatch is reported as unmatched (no fallback to D80, avoiding a wrong D80 firmware load) */
        if (vid == AIC_VENDOR_ID_AIC_V2)
        {
            USB_DEVICEID = pid;
            return PRODUCT_ID_AIC8800D80X2;
        }
        USB_DEVICEID = pid;
        return AICWF_CHIPID_UNMATCHED;
    case AIC_PRODUCT_ID_D80N:
        USB_DEVICEID = pid;
        return PRODUCT_ID_AIC8800D80N;
    case AIC_PRODUCT_ID_DC:
    case AIC_PRODUCT_ID_DC_USB:
        USB_DEVICEID = pid;
        return PRODUCT_ID_AIC8800DC;
    default:
        USB_DEVICEID = AIC_PRODUCT_ID_D80;
        return PRODUCT_ID_AIC8800D80;
    }
}

/* TX URB completion callback (HCD giveback runs in ISR) */
static void aicwf_usb_tx_complete_cb(struct urb *urb)
{
    uint8_t ep = (uint8_t)(long)urb->context;
    int status = urb->status;
    int i;

    /* Mark the URB as idle */
    for (i = 0; i < AICWF_USB_TX_URBS; i++)
    {
        if (s_tx_urbs[i] == urb)
        {
            s_tx_urb_busy[i] = 0;
            break;
        }
    }

    s_tx_free_count++;
    aicwf_rtos_semaphore_signal(s_tx_free_sema, true);

    if (ep == USB_EP_MSG)
    {
        s_msg_finished = (status == 0);
        aicwf_rtos_semaphore_signal(s_msg_sema, true);
    }

    aicwf_usb_tx_complete(ep, status);
}

/* RX URB completion callback (HCD giveback runs in ISR)
 * Keep protocol processing short: hand the data to the CO layer to copy and queue and wake the RX task, then resubmit the URB immediately. */
static void aicwf_usb_rx_complete_cb(struct urb *urb)
{
    uint8_t *buf = (uint8_t *)urb->transfer_buffer;
    uint32_t len = urb->actual_length;
    int idx = -1;
    int i;
    int status = urb->status;

    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        if (s_rx_urbs[i] == urb)
        {
            idx = i;
            s_rx_urb_busy[i] = 0;
            break;
        }
    }

    if (status == 0 && len > 0)
    {
        /* Match the SDIO bus-level trace point (DRV_TRG_SDIO@sdio_port.c): bracket the RX data hand-off to the CO layer */
        aicwf_plat_gpio_trigger(DRV_TRG_USB, 1);
        aicwf_usb_rx_complete(buf, len);
        aicwf_plat_gpio_trigger(DRV_TRG_USB, 0);
    }
    else if (status != 0 && status != -ENOENT &&
             status != -ECONNRESET && status != -ENODEV &&
             status != -ETIMEDOUT)
    {
        DBG_USB_ERR("%s: RX URB error status=%d\n", __func__, status);
    }

    /*
     * Only stop resubmitting on a real disconnect or error.
     * Note: while s_expect_reconnect is armed the device still exchanges data on
     * bootrom (FW block-write CFM); RX URBs must keep being submitted, otherwise
     * the RX pool drains after ~AICWF_USB_RX_URBS successful receives and commands time out.
     */
    if (status != 0 || s_udev == NULL)
    {
        return;
    }

    if (idx >= 0)
    {
        unsigned int pipe = s_rx_pipe[idx] ? s_rx_pipe[idx] : s_bulk_in_pipe;
        s_rx_urb_busy[idx] = 1;
        usb_fill_bulk_urb(s_rx_urbs[idx], s_udev, pipe,
                          s_rx_bufs[idx], AICWF_USB_MAX_PKT_SIZE,
                          aicwf_usb_rx_complete_cb, NULL);
        usb_submit_urb(s_rx_urbs[idx], 0);
    }
}

/*
 * 8d81 (WiFi+BT) enumerates as 3 interfaces: WLAN is Vendor 0xff/0xff/0xff,
 * the others are BT. Only accept WLAN, otherwise WiFi commands go to the BT port
 * (causing bt acl rx len err on the chip side and no CFM for MM_SET_STACK_START).
 * Matches the interface class filtering in Linux aicwf_parse_usb().
 */
static int aicwf_usb_is_wlan_intf(struct usb_interface *intf)
{
    struct usb_interface_descriptor *d;

    if (intf == NULL || intf->altsetting == NULL)
        return 0;
    d = &intf->altsetting[0].desc;
    return (d->bInterfaceClass == USB_CLASS_VENDOR_SPEC &&
            d->bInterfaceSubClass == 0xff &&
            d->bInterfaceProtocol == 0xff);
}

/* Parse the USB interface endpoints */
static int aicwf_parse_usb_endpoints(struct usb_interface *intf)
{
    struct usb_host_interface *altsetting = &intf->altsetting[0];
    struct usb_endpoint_descriptor *ep_desc;
    int i;
    int bulk_in_found = 0, bulk_out_found = 0;
    int msg_in_found = 0, msg_out_found = 0;

    s_msg_in_pipe = 0;
    s_msg_out_pipe = 0;

    DBG_USB_INF("%s: if#%u class=%02x/%02x/%02x nEp=%u\n", __func__,
                altsetting->desc.bInterfaceNumber,
                altsetting->desc.bInterfaceClass,
                altsetting->desc.bInterfaceSubClass,
                altsetting->desc.bInterfaceProtocol,
                altsetting->desc.bNumEndpoints);

    for (i = 0; i < altsetting->desc.bNumEndpoints; i++)
    {
        ep_desc = &altsetting->endpoint[i].desc;
        DBG_USB_INF("%s: ep[%d] addr=0x%02x attr=0x%02x\n", __func__,
                    i, ep_desc->bEndpointAddress, ep_desc->bmAttributes);

        if (usb_endpoint_is_bulk_in(ep_desc))
        {
            uint8_t ep_addr = ep_desc->bEndpointAddress & 0x0f;
            if (!bulk_in_found)
            {
                s_bulk_in_pipe = usb_rcvbulkpipe(s_udev, ep_addr);
                bulk_in_found = 1;
                DBG_USB_INF("%s: found bulk IN ep 0x%02x\n", __func__,
                            ep_desc->bEndpointAddress);
            }
            else if (!msg_in_found)
            {
                s_msg_in_pipe = usb_rcvbulkpipe(s_udev, ep_addr);
                msg_in_found = 1;
                DBG_USB_INF("%s: found MSG IN ep 0x%02x\n", __func__,
                            ep_desc->bEndpointAddress);
            }
        }

        if (usb_endpoint_is_bulk_out(ep_desc))
        {
            uint8_t ep_addr = ep_desc->bEndpointAddress & 0x0f;
            if (!bulk_out_found)
            {
                s_bulk_out_pipe = usb_sndbulkpipe(s_udev, ep_addr);
                bulk_out_found = 1;
                DBG_USB_INF("%s: found bulk OUT ep 0x%02x\n", __func__,
                            ep_desc->bEndpointAddress);
            }
            else if (!msg_out_found)
            {
                s_msg_out_pipe = usb_sndbulkpipe(s_udev, ep_addr);
                msg_out_found = 1;
                DBG_USB_INF("%s: found MSG OUT ep 0x%02x\n", __func__,
                            ep_desc->bEndpointAddress);
            }
        }
    }

    if (!bulk_in_found || !bulk_out_found)
    {
        DBG_USB_ERR("%s: bulk endpoints not found\n", __func__);
        return -ENODEV;
    }

    return 0;
}

/**
 * aicwf_usb_func_init - USB function initialization (RT-Thread implementation)
 *
 * Called only by aicwf_usb_host_init() (wifi_init_cmd path).
 * probe only claims the device; this allocates the URB pool and prepares TX/RX.
 * If the driver was not matched during enumeration, claim by VID/PID once here.
 */
int aicwf_usb_func_init(uint16_t chipid, struct aic_usb_dev *usbdev)
{
    int i;
    int ret;

    if (usbdev == NULL)
    {
        DBG_USB_ERR("%s: NULL usbdev\n", __func__);
        return -EINVAL;
    }

    if (s_func_inited)
    {
        DBG_USB_INF("%s: already inited, skip\n", __func__);
        return 0;
    }

    if (s_udev == NULL)
    {
        struct usb_device *hub = usb_get_root_hub();
        struct usb_device *dev = usb_find_child_by_vidpid(hub,
                                 AIC_VENDOR_ID, AIC_PRODUCT_ID_D80);
        if (dev == NULL)
        {
            dev = usb_find_child_by_vidpid(hub,
                                           AIC_VENDOR_ID_LEGACY, AIC_PRODUCT_ID_D80_LEGACY);
        }
        DBG_USB_INF("%s: bypass claim dev=%p hub=%p config=%p\n",
                    __func__, dev, hub, dev ? dev->config : NULL);
        if (dev && dev->config && dev->config[0].interface[0])
        {
            /* Only claim the device; do not call usb_device_add (it recurses into probe) */
            aicwf_usb_set_device(dev, dev->config[0].interface[0]);
            usb_set_intfdata(dev->config[0].interface[0], &g_aic_usb_dev);
        }
        if (s_udev == NULL)
        {
            DBG_USB_ERR("%s: USB device not probed yet\n", __func__);
            return -ENODEV;
        }
    }

    DBG_USB_INF("%s: chipid=0x%04x vid=%04x pid=%04x\n", __func__, chipid,
                s_udev->descriptor.idVendor, s_udev->descriptor.idProduct);

    /* Initialize sync primitives (rtos_port abstraction; counting semaphore: max=pool depth, init=63 since URB0 is reserved for MSG) */
    aicwf_rtos_semaphore_create(&s_msg_sema, "aic_msg", 1, 0);
    aicwf_rtos_semaphore_create(&s_tx_free_sema, "aic_txfree",
                                AICWF_USB_TX_URBS, AICWF_USB_TX_URBS - 1);
    s_msg_busy = 0;
    s_msg_finished = 0;
    s_tx_free_count = AICWF_USB_TX_URBS - 1;

    /* Allocate the TX URB pool (index 0 reserved for MSG) */
    memset(s_tx_urb_busy, 0, sizeof(s_tx_urb_busy));
    for (i = 0; i < AICWF_USB_TX_URBS; i++)
    {
        s_tx_urbs[i] = usb_alloc_urb(0, 0);
        if (s_tx_urbs[i] == NULL)
        {
            DBG_USB_ERR("%s: alloc tx urb %d failed\n", __func__, i);
            ret = -ENOMEM;
            goto err_free;
        }
        s_tx_bufs[i] = (uint8_t *)aicwf_rtos_malloc(AICWF_USB_MAX_PKT_SIZE);
        if (s_tx_bufs[i] == NULL)
        {
            ret = -ENOMEM;
            goto err_free;
        }
        s_tx_urb_busy[i] = 0;
    }

    /* Allocate the RX URB pool */
    memset(s_rx_urb_busy, 0, sizeof(s_rx_urb_busy));
    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        s_rx_urbs[i] = usb_alloc_urb(0, 0);
        if (s_rx_urbs[i] == NULL)
        {
            DBG_USB_ERR("%s: alloc rx urb %d failed\n", __func__, i);
            ret = -ENOMEM;
            goto err_free;
        }
        s_rx_bufs[i] = (uint8_t *)aicwf_rtos_malloc(AICWF_USB_MAX_PKT_SIZE);
        if (s_rx_bufs[i] == NULL)
        {
            ret = -ENOMEM;
            goto err_free;
        }
        s_rx_urb_busy[i] = 0;
    }

    s_func_inited = 1;
    if (!s_reconnect_sem_inited)
    {
        aicwf_rtos_semaphore_create(&s_reconnect_sem, "aic_reconn", 1, 0);
        s_reconnect_sem_inited = 1;
    }
    DBG_USB_INF("%s: URB pool ready (tx=%d rx=%d)\n",
                __func__, AICWF_USB_TX_URBS, AICWF_USB_RX_URBS);
    return 0;

err_free:
    for (i = 0; i < AICWF_USB_TX_URBS; i++)
    {
        if (s_tx_urbs[i]) usb_free_urb(s_tx_urbs[i]);
        if (s_tx_bufs[i]) aicwf_rtos_free(s_tx_bufs[i]);
    }
    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        if (s_rx_urbs[i]) usb_free_urb(s_rx_urbs[i]);
        if (s_rx_bufs[i]) aicwf_rtos_free(s_rx_bufs[i]);
    }
    return ret;
}

/**
 * aicwf_usb_set_device - called by USB probe to set the USB device handle
 * This function must be called from the platform USB driver probe callback
 */
int aicwf_usb_set_device(struct usb_device *udev, struct usb_interface *intf)
{
    int ret;

    s_udev = udev;
    s_intf = intf;

    ret = aicwf_parse_usb_endpoints(intf);
    if (ret != 0)
    {
        s_udev = NULL;
        s_intf = NULL;
        return ret;
    }

    return 0;
}

#if 0  /* Original stub implementation — superseded by the full implementation below */
int aicwf_usb_drv_register(void)
{
    DBG_USB_INF("%s: AIC8800 USB WiFi driver ready\n", __func__);
    return 0;
}
#endif /* 0 — RT-Thread USB driver registration, see INIT_APP_EXPORT at the end of the file */

#if 0
/* ==================== Linux reference implementation ==================== */
/* TX URB submission — usb_fill_bulk_urb + usb_submit_urb
 * used by the msg path (aicwf_usb_bus_txmsg) and the data path (aicwf_usb_bus_txdata) */
static int aicwf_usb_bus_txmsg(struct device *dev, u8 *msg, uint len)
{
    struct aicwf_bus *bus_if = dev_get_drvdata(dev);
    struct aic_usb_dev *usb_dev = bus_if->bus_priv.usb;
    int ret = 0;
    int retry = 0;

    if (usb_dev->state != USB_UP_ST)
        return -EIO;

    if (test_and_set_bit(0, &usb_dev->msg_busy))
        return -EIO;

    usb_dev->msg_finished = false;
    memcpy(usb_dev->msg_buf, msg, len);

    usb_fill_bulk_urb(usb_dev->msg_out_urb,
                      usb_dev->udev, usb_dev->bulk_out_pipe,
                      usb_dev->msg_buf, len,
                      aicwf_usb_tx_complete, usb_dev);

    usb_dev->msg_out_urb->transfer_flags |= URB_ZERO_PACKET;

    ret = usb_submit_urb(usb_dev->msg_out_urb, GFP_ATOMIC);
    if (ret)
    {
        usb_err("usb_submit_urb fail %d\n", ret);
        usb_dev->msg_busy = 0;
        return ret;
    }

    /* Wait synchronously for msg completion */
    ret = wait_event_timeout(usb_dev->msg_wait,
                             usb_dev->msg_finished, msecs_to_jiffies(CMD_TX_TIMEOUT));
    if (!ret)
    {
        usb_err("txmsg timeout\n");
        ret = -EIO;
    }
    usb_dev->msg_busy = 0;
    return ret;
}

static int aicwf_usb_bus_txdata(struct device *dev, struct sk_buff *skb)
{
    struct aicwf_bus *bus_if = dev_get_drvdata(dev);
    struct aic_usb_dev *usb_dev = bus_if->bus_priv.usb;
    struct aicwf_usb_buf *usb_buf;
    int ret = 0;
    u16 *hdr;

    /* Flow control: wait for a free TX URB */
    spin_lock_bh(&usb_dev->tx_free_lock);
    if (usb_dev->tx_free_count < AICWF_USB_TX_LOW_WATER)
    {
        spin_unlock_bh(&usb_dev->tx_free_lock);
        return -EIO;
    }
    usb_buf = list_first_entry_or_null(&usb_dev->tx_free_list,
                                       struct aicwf_usb_buf, list);
    if (!usb_buf)
    {
        spin_unlock_bh(&usb_dev->tx_free_lock);
        return -EIO;
    }
    list_del(&usb_buf->list);
    usb_dev->tx_free_count--;
    spin_unlock_bh(&usb_dev->tx_free_lock);

    /* Prepend the 4-byte usb_header */
    hdr = (u16 *)skb_push(skb, 4);
    hdr[0] = cpu_to_le16(skb->len);
    hdr[1] = cpu_to_le16(USB_TYPE_DATA);

    usb_fill_bulk_urb(usb_buf->urb,
                      usb_dev->udev, usb_dev->bulk_out_pipe,
                      skb->data, skb->len,
                      aicwf_usb_tx_complete, usb_buf);
    usb_buf->urb->transfer_flags |= URB_ZERO_PACKET;

    ret = usb_submit_urb(usb_buf->urb, GFP_ATOMIC);
    if (ret)
    {
        /* Return the URB to the free list */
        spin_lock_bh(&usb_dev->tx_free_lock);
        list_add_tail(&usb_buf->list, &usb_dev->tx_free_list);
        usb_dev->tx_free_count++;
        spin_unlock_bh(&usb_dev->tx_free_lock);
    }
    return ret;
}
#endif /* 0 — end of Linux reference implementation */

/**
 * aicwf_usb_submit_tx_urb - submit a TX bulk OUT URB (RT-Thread implementation)
 */
int aicwf_usb_submit_tx_urb(uint8_t ep, uint8_t *buf, uint32_t len)
{
    struct urb *urb;
    int ret;
    int i, idx;

    if (buf == NULL)
    {
        DBG_USB_ERR("%s: NULL buf\n", __func__);
        return -EINVAL;
    }
    if (len == 0 || len > AICWF_USB_MAX_PKT_SIZE)
    {
        DBG_USB_ERR("%s: invalid len %u\n", __func__, len);
        return -EINVAL;
    }
    if (s_udev == NULL)
    {
        DBG_USB_ERR("%s: no USB device\n", __func__);
        return -ENODEV;
    }

    if (ep == USB_EP_MSG)
    {
        unsigned int pipe = s_msg_out_pipe ? s_msg_out_pipe : s_bulk_out_pipe;

        if (s_msg_busy)
        {
            return -EIO;
        }
        s_msg_busy = 1;
        s_msg_finished = 0;

        urb = s_tx_urbs[0];
        memcpy(s_tx_bufs[0], buf, len);
        s_tx_urb_busy[0] = 1;
        usb_fill_bulk_urb(urb, s_udev, pipe,
                          s_tx_bufs[0], len,
                          aicwf_usb_tx_complete_cb, (void *)(long)USB_EP_MSG);
        urb->transfer_flags |= URB_ZERO_PACKET;
        ret = usb_submit_urb(urb, 0);
        if (ret)
        {
            s_msg_busy = 0;
            s_tx_urb_busy[0] = 0;
            return ret;
        }
    }
    else
    {
        if (s_tx_free_count < AICWF_USB_TX_LOW_WATER)
        {
            return -EAGAIN;
        }

        idx = -1;
        for (i = 1; i < AICWF_USB_TX_URBS; i++)
        {
            if (s_tx_urbs[i] && s_tx_bufs[i] && !s_tx_urb_busy[i])
            {
                idx = i;
                break;
            }
        }
        if (idx < 0)
        {
            return -ENOMEM;
        }

        memcpy(s_tx_bufs[idx], buf, len);
        s_tx_urb_busy[idx] = 1;
        urb = s_tx_urbs[idx];
        usb_fill_bulk_urb(urb, s_udev, s_bulk_out_pipe,
                          s_tx_bufs[idx], len,
                          aicwf_usb_tx_complete_cb, (void *)(long)USB_EP_DATA);
        urb->transfer_flags |= URB_ZERO_PACKET;
        ret = usb_submit_urb(urb, 0);
        if (ret)
        {
            s_tx_urb_busy[idx] = 0;
            return ret;
        }
        s_tx_free_count--;
    }

    return 0;
}

/**
 * aicwf_usb_submit_tx_data - single DATA packet: prepend the 4-byte USB header while copying
 * matches Linux aicwf_usb_bus_txdata / skb_push(4)
 */
int aicwf_usb_submit_tx_data(uint8_t *payload, uint32_t payload_len)
{
    struct urb *urb;
    int ret;
    int i, idx;
    uint32_t total;

    if (payload == NULL || payload_len == 0)
        return -EINVAL;
    total = payload_len + 4;
    if (total > AICWF_USB_MAX_PKT_SIZE)
    {
        DBG_USB_ERR("%s: len too big %u\n", __func__, total);
        return -EINVAL;
    }
    if (s_udev == NULL)
        return -ENODEV;
    if (s_tx_free_count < AICWF_USB_TX_LOW_WATER)
        return -EAGAIN;

    idx = -1;
    for (i = 1; i < AICWF_USB_TX_URBS; i++)
    {
        if (s_tx_urbs[i] && s_tx_bufs[i] && !s_tx_urb_busy[i])
        {
            idx = i;
            break;
        }
    }
    if (idx < 0)
        return -ENOMEM;

    s_tx_bufs[idx][0] = (uint8_t)(total & 0xff);
    s_tx_bufs[idx][1] = (uint8_t)((total >> 8) & 0x0f);
    s_tx_bufs[idx][2] = 0x01; /* USB_TYPE_DATA */
    s_tx_bufs[idx][3] = 0x00;
    memcpy(s_tx_bufs[idx] + 4, payload, payload_len);
    s_tx_urb_busy[idx] = 1;
    urb = s_tx_urbs[idx];
    usb_fill_bulk_urb(urb, s_udev, s_bulk_out_pipe,
                      s_tx_bufs[idx], total,
                      aicwf_usb_tx_complete_cb, (void *)(long)USB_EP_DATA);
    urb->transfer_flags |= URB_ZERO_PACKET;
    ret = usb_submit_urb(urb, 0);
    if (ret)
    {
        s_tx_urb_busy[idx] = 0;
        return ret;
    }
    s_tx_free_count--;
    return 0;
}

/**
 * aicwf_usb_wait_tx_msg_complete - wait synchronously for TX msg completion (RT-Thread implementation)
 */
int aicwf_usb_wait_tx_msg_complete(uint32_t timeout_ms)
{
    int ret;

    /* timeout is in ms, converted to ticks by the rtos_port abstraction; returns 0 on success */
    ret = aicwf_rtos_semaphore_wait(s_msg_sema, (int)timeout_ms);
    s_msg_busy = 0;

    if (ret != AIC_RTOS_SUCCESS || !s_msg_finished)
    {
        return -ETIMEDOUT;
    }
    return 0;
}

/**
 * aicwf_usb_tx_free_count - query the number of free TX URBs
 */
uint32_t aicwf_usb_tx_free_count(void)
{
    return s_tx_free_count;
}

#if 0
/* ==================== Linux reference implementation ==================== */
/* RX URB submission — usb_fill_bulk_urb(bulk_in_pipe) + usb_submit_urb
 * bulk IN URBs stay resident; the complete callback aicwf_usb_rx_complete handles received data
 * this is the core mechanism that replaces the SDIO interrupt in USB */
static int aicwf_usb_submit_rx_urb(struct aic_usb_dev *usb_dev,
                                   struct aicwf_usb_buf *usb_buf)
{
    struct sk_buff *skb;
    int ret;

    if (!usb_buf || !usb_dev)
        return -1;
    if (usb_dev->state != USB_UP_ST)
    {
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        return -1;
    }

    skb = __dev_alloc_skb(aicwf_usb_max_pkt_size, GFP_ATOMIC);
    if (!skb)
    {
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        return -1;
    }
    usb_buf->skb = skb;

    usb_fill_bulk_urb(usb_buf->urb,
                      usb_dev->udev,
                      usb_dev->bulk_in_pipe,
                      skb->data, aicwf_usb_max_pkt_size,
                      aicwf_usb_rx_complete, usb_buf);

    usb_buf->usbdev = usb_dev;
    usb_anchor_urb(usb_buf->urb, &usb_dev->rx_submitted);
    ret = usb_submit_urb(usb_buf->urb, GFP_ATOMIC);
    if (ret)
    {
        usb_unanchor_urb(usb_buf->urb);
        aicwf_dev_skb_free(usb_buf->skb);
        usb_buf->skb = NULL;
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        mdelay(100);
        return -1;
    }
    else
    {
        atomic_inc(&rx_urb_cnt);
    }
    return 0;
}

static void aicwf_usb_rx_submit_all_urb(struct aic_usb_dev *usb_dev)
{
    struct aicwf_usb_buf *usb_buf;
    if (usb_dev->state != USB_UP_ST)
    {
        return;
    }
    while ((usb_buf = aicwf_usb_rx_buf_get(usb_dev)) != NULL)
    {
        if (aicwf_usb_submit_rx_urb(usb_dev, usb_buf))
        {
            return;
        }
    }
    usb_dev->rx_prepare_ready = true;
}
#endif /* 0 — end of Linux reference implementation */

/**
 * aicwf_usb_rx_submit_all - submit all RX bulk IN URBs (RT-Thread implementation)
 */
int aicwf_usb_rx_submit_all(void)
{
    int i;
    int ret;
    int msg_rx_n = 0;

    if (s_udev == NULL)
    {
        return -ENODEV;
    }

    /*
     * With MSG IN: the upper half of the RX URBs uses msg_in (for CFM), the lower half uses data bulk_in.
     * On 8d80 bootrom with only 2 EPs, msg_in=0 and everything goes through bulk_in.
     */
    if (s_msg_in_pipe)
        msg_rx_n = AICWF_USB_RX_URBS / 2;

    DBG_USB_INF("%s: submit %d RX URBs (data=%d msg=%d)\n", __func__,
                AICWF_USB_RX_URBS, AICWF_USB_RX_URBS - msg_rx_n, msg_rx_n);

    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        unsigned int pipe;

        if (!(s_rx_urbs[i] && s_rx_bufs[i] && !s_rx_urb_busy[i]))
            continue;

        if (s_msg_in_pipe && i >= (AICWF_USB_RX_URBS - msg_rx_n))
            pipe = s_msg_in_pipe;
        else
            pipe = s_bulk_in_pipe;

        s_rx_pipe[i] = pipe;
        s_rx_urb_busy[i] = 1;
        usb_fill_bulk_urb(s_rx_urbs[i], s_udev, pipe,
                          s_rx_bufs[i], AICWF_USB_MAX_PKT_SIZE,
                          aicwf_usb_rx_complete_cb, NULL);
        ret = usb_submit_urb(s_rx_urbs[i], 0);
        if (ret)
        {
            DBG_USB_ERR("%s: submit rx urb %d failed %d\n", __func__, i, ret);
            s_rx_urb_busy[i] = 0;
        }
    }

    return 0;
}

#if 0
/* ==================== Linux reference implementation ==================== */
/* URB cancel logic — from aicwf_usb_cancel_all_urbs_ (aicwf_usb.c:1920)
 * used to cancel all pending URBs on disconnect/bus_stop.
 *
 * Note: RTOSes usually lack the Linux anchor mechanism (usb_kill_anchored_urbs);
 * when porting, maintain your own submitted URB list and walk it on disconnect,
 * calling the platform's usb_kill_urb equivalent API (cancel and wait for completion).
 *
 * TX URB cancel: walk tx_post_list (submitted but not yet completed URBs)
 *   - call usb_kill_urb for each one (wait synchronously for complete to return)
 *   - in CONFIG_USB_NO_TRANS_DMA_MAP mode, also free the DMA buffer
 * RX URB cancel: usb_kill_anchored_urbs(&rx_submitted)
 *   - cancel all bulk IN URBs anchored to rx_submitted
 * msg_out_urb cancel: usb_kill_urb(msg_out_urb) (separate msg URB)
 */
static void aicwf_usb_cancel_all_urbs_(struct aic_usb_dev *usb_dev)
{
    struct aicwf_usb_buf *usb_buf, *tmp;
    unsigned long flags;

    if (usb_dev->msg_out_urb)
        usb_kill_urb(usb_dev->msg_out_urb);

    spin_lock_irqsave(&usb_dev->tx_post_lock, flags);
    list_for_each_entry_safe(usb_buf, tmp, &usb_dev->tx_post_list, list)
    {
        spin_unlock_irqrestore(&usb_dev->tx_post_lock, flags);
        if (!usb_buf->urb)
        {
            usb_err("bad usb_buf\n");
            spin_lock_irqsave(&usb_dev->tx_post_lock, flags);
            break;
        }
        usb_kill_urb(usb_buf->urb);
#if defined CONFIG_USB_NO_TRANS_DMA_MAP
        if (usb_buf->data_buf)
        {
            usb_free_coherent(usb_buf->usbdev->udev, DATA_BUF_MAX,
                              usb_buf->data_buf, usb_buf->data_dma_trans_addr);
            usb_buf->data_buf = NULL;
            usb_buf->data_dma_trans_addr = 0x0;
        }
#endif
        spin_lock_irqsave(&usb_dev->tx_post_lock, flags);
    }
    spin_unlock_irqrestore(&usb_dev->tx_post_lock, flags);

    usb_kill_anchored_urbs(&usb_dev->rx_submitted);
#ifdef CONFIG_USB_MSG_IN_EP
    if (usb_dev->msg_in_pipe)
    {
        usb_kill_anchored_urbs(&usb_dev->msg_rx_submitted);
    }
#endif
}

/* URB pool release — from aicwf_usb_deinit (aicwf_usb.c:1993)
 * called after cancel_all_urbs to free the URB pool memory.
 * Note: deinit only frees the URB pool, it does not kill (kill is done by cancel_all_urbs).
 */
static void aicwf_usb_free_urb(struct list_head *q, spinlock_t *qlock)
{
    struct aicwf_usb_buf *usb_buf, *tmp;
    unsigned long flags;

    spin_lock_irqsave(qlock, flags);
    list_for_each_entry_safe(usb_buf, tmp, q, list)
    {
        spin_unlock_irqrestore(qlock, flags);
        if (!usb_buf->urb) break;
#ifdef CONFIG_USB_TX_AGGR
        if (usb_buf->skb) dev_kfree_skb(usb_buf->skb);
#endif
        usb_free_urb(usb_buf->urb);
        list_del_init(&usb_buf->list);
        spin_lock_irqsave(qlock, flags);
    }
    spin_unlock_irqrestore(qlock, flags);
}

static void aicwf_usb_deinit(struct aic_usb_dev *usbdev)
{
    cancel_work_sync(&usbdev->rx_urb_work);
    aicwf_usb_free_urb(&usbdev->rx_free_list, &usbdev->rx_free_lock);
    aicwf_usb_free_urb(&usbdev->tx_free_list, &usbdev->tx_free_lock);
#ifdef CONFIG_USB_MSG_IN_EP
    if (usbdev->msg_in_pipe)
    {
        cancel_work_sync(&usbdev->msg_rx_urb_work);
        aicwf_usb_free_urb(&usbdev->msg_rx_free_list, &usbdev->msg_rx_free_lock);
    }
#endif
    usb_free_urb(usbdev->msg_out_urb);
}

/* Full disconnect flow — from aicwf_usb_disconnect (aicwf_usb.c:2559)
 * call order: bus_deinit -> usb_deinit (free URB pool) -> release the dev structure
 * Note: aicwf_deinit_sem works together with RX complete so that disconnect waits for
 *       all pending URB completions to return, avoiding NULL pointer dereferences.
 */
static void aicwf_usb_disconnect(struct usb_interface *intf)
{
    struct aic_usb_dev *usb_dev = usb_get_intfdata(intf);

    if (g_rwnx_plat->wait_disconnect_cb == false)
    {
        atomic_set(&aicwf_deinit_atomic, 0);
        down(&aicwf_deinit_sem);
    }
    if (!usb_dev) return;

    aicwf_bus_deinit(usb_dev->dev);
    aicwf_usb_deinit(usb_dev);

#ifdef CONFIG_GPIO_WAKEUP
    rwnx_unregister_hostwake_irq(usb_dev->dev);
#endif
    kfree(usb_dev->bus_if);
    vfree(usb_dev->usb_tx_buf);
    vfree(usb_dev->usb_rx_buf);
    kfree(usb_dev);
    up(&aicwf_deinit_sem);
    atomic_set(&aicwf_deinit_atomic, 1);
}
#endif /* 0 — end of Linux URB kill/deinit reference implementation */

/**
 * aicwf_usb_kill_all_tx_urb - cancel all TX URBs (RT-Thread implementation)
 */
void aicwf_usb_kill_all_tx_urb(void)
{
    int i;

    DBG_USB_INF("%s\n", __func__);

    for (i = 0; i < AICWF_USB_TX_URBS; i++)
    {
        if (s_tx_urbs[i])
        {
            usb_kill_urb(s_tx_urbs[i]);
        }
    }
    s_msg_busy = 0;
    s_msg_finished = 0;
    s_tx_free_count = AICWF_USB_TX_URBS;
}

/**
 * aicwf_usb_kill_all_rx_urb - cancel all RX URBs (RT-Thread implementation)
 */
void aicwf_usb_kill_all_rx_urb(void)
{
    int i;

    DBG_USB_INF("%s\n", __func__);

    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        if (s_rx_urbs[i])
        {
            usb_kill_urb(s_rx_urbs[i]);
        }
    }
}

/**
 * aicwf_usb_deinit - release the URB pool and resources
 */
void aicwf_usb_deinit(void)
{
    int i;

    aicwf_usb_kill_all_tx_urb();
    aicwf_usb_kill_all_rx_urb();

    if (s_msg_sema)
    {
        aicwf_rtos_semaphore_delete(s_msg_sema);
        s_msg_sema = NULL;
    }
    if (s_tx_free_sema)
    {
        aicwf_rtos_semaphore_delete(s_tx_free_sema);
        s_tx_free_sema = NULL;
    }

    for (i = 0; i < AICWF_USB_TX_URBS; i++)
    {
        if (s_tx_urbs[i])
        {
            usb_free_urb(s_tx_urbs[i]);
            s_tx_urbs[i] = NULL;
        }
        if (s_tx_bufs[i])
        {
            aicwf_rtos_free(s_tx_bufs[i]);
            s_tx_bufs[i] = NULL;
        }
    }

    for (i = 0; i < AICWF_USB_RX_URBS; i++)
    {
        if (s_rx_urbs[i])
        {
            usb_free_urb(s_rx_urbs[i]);
            s_rx_urbs[i] = NULL;
        }
        if (s_rx_bufs[i])
        {
            aicwf_rtos_free(s_rx_bufs[i]);
            s_rx_bufs[i] = NULL;
        }
    }

    s_udev = NULL;
    s_intf = NULL;
    s_bulk_in_pipe = 0;
    s_bulk_out_pipe = 0;
    s_msg_in_pipe = 0;
    s_msg_out_pipe = 0;
    s_func_inited = 0;
}

/* ================================================================ */
/*  Port -> CO upstream callbacks — Linux reference implementation     */
/*  Implemented in usb_co.c, called on Port layer URB completion    */
/* ================================================================ */

#if 0
/* ==================== Linux reference implementation ==================== */
/* TX URB completion callback — from aicwf_usb_tx_complete (aicwf_usb.c:214)
 * called when a Port layer bulk OUT URB completes, to notify the CO layer.
 *
 * Core logic:
 *   1. free the skb associated with usb_buf (non-AGGR) or the aligned buf
 *   2. recycle usb_buf into tx_free_list via aicwf_usb_tx_queue (tx_free_count++)
 *   3. flow-control wakeup: if tx_free_count > HIGH_WATER and tbusy is set, clear tbusy and
 *      call aicwf_usb_tx_flowctrl(rwnx_hw, false) to wake the TX queue
 *
 * RTOS port: call aicwf_usb_tx_complete(ep, status) from the platform URB completion callback
 *            the CO layer handles msg wait wakeup and flow-control state
 */
static void aicwf_usb_tx_complete(struct urb *urb)
{
    unsigned long flags;
    struct aicwf_usb_buf *usb_buf = (struct aicwf_usb_buf *)urb->context;
    struct aic_usb_dev *usb_dev = usb_buf->usbdev;
#ifndef CONFIG_USB_TX_AGGR
    struct sk_buff *skb;
#endif

    usb_txc_sta_flowctrl(usb_buf, usb_dev);

#ifdef CONFIG_USB_ALIGN_DATA
    if (usb_buf->usb_align_data)
    {
        kfree(usb_buf->usb_align_data);
        usb_buf->usb_align_data = NULL;
    }
#endif
#ifndef CONFIG_USB_TX_AGGR
    if (usb_buf->cfm == false)
    {
        skb = usb_buf->skb;
        dev_kfree_skb_any(skb);
    }
#if !defined CONFIG_USB_NO_TRANS_DMA_MAP
    else
    {
        u8 *buf = (u8 *)usb_buf->skb;
        kfree(buf);
    }
#endif
    usb_buf->skb = NULL;
#else
    usb_buf->aggr_cnt = 0;
#endif
    aicwf_usb_tx_queue(usb_dev, &usb_dev->tx_free_list, usb_buf,
                       &usb_dev->tx_free_count, &usb_dev->tx_free_lock);

    spin_lock_irqsave(&usb_dev->tx_flow_lock, flags);
    if (usb_dev->tx_free_count > AICWF_USB_TX_HIGH_WATER)
    {
        if (usb_dev->tbusy)
        {
            usb_dev->tbusy = false;
            aicwf_usb_tx_flowctrl(usb_dev->rwnx_hw, false);
        }
    }
    spin_unlock_irqrestore(&usb_dev->tx_flow_lock, flags);
}

/* RX URB completion callback — from aicwf_usb_rx_complete (aicwf_usb.c:351, non-PREALLOC version)
 * called when a Port layer bulk IN URB completes, to report received data to the CO layer.
 *
 * Core logic (non-PREALLOC, non-REASSEMBLE simplified path):
 *   1. error check: urb->status != 0 or actual_length == 0 -> free skb + resubmit URB
 *   2. length check: actual_length > transfer_buffer_length -> free + resubmit
 *   3. queue data: skb_put(skb, actual_length) -> aicwf_rxframe_enqueue(rxq, skb)
 *   4. wake RX task: atomic_inc(rx_cnt); if rx_cnt==1 (queue went from empty to non-empty) ->
 *      complete(&busrx_trgg) wakes usb_busrx_thread (which calls aicwf_process_rxframes)
 *   5. URB recycle: aicwf_usb_rx_buf_put (return to rx_free_list) + resubmit URB
 *
 * Key: RX URBs are **always** resubmitted in complete (continuous receive model).
 *       An RTOS port must submit another receive at the end of the callback so the RX pipeline never stalls.
 *
 * CONFIG_USB_RX_REASSEMBLE: large-packet cross-URB reassembly (omitted in this reference block, see the original file)
 */
static void aicwf_usb_rx_complete(struct urb *urb)
{
    struct aicwf_usb_buf *usb_buf = (struct aicwf_usb_buf *)urb->context;
    struct aic_usb_dev *usb_dev = usb_buf->usbdev;
    struct aicwf_rx_priv *rx_priv = usb_dev->rx_priv;
    struct sk_buff *skb = NULL;
    unsigned long flags = 0;

    skb = usb_buf->skb;
    usb_buf->skb = NULL;
    atomic_dec(&rx_urb_cnt);

    if (!usb_dev->rwnx_hw)
    {
        aicwf_dev_skb_free(skb);
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        return;
    }
    if (urb->actual_length > urb->transfer_buffer_length)
    {
        aicwf_dev_skb_free(skb);
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        aicwf_usb_rx_submit_all_urb_(usb_dev);
        return;
    }
    if (urb->status != 0 || !urb->actual_length)
    {
        aicwf_dev_skb_free(skb);
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        if (urb->status < 0)
        {
            return;  /* error: do not resubmit, wait for disconnect synchronization */
        }
        else
        {
            aicwf_usb_rx_submit_all_urb_(usb_dev);  /* resubmit */
            return;
        }
    }

    if (usb_dev->state == USB_UP_ST)
    {
        skb_put(skb, urb->actual_length);

        spin_lock_irqsave(&rx_priv->rxqlock, flags);
        if (!aicwf_rxframe_enqueue(usb_dev->dev, &rx_priv->rxq, skb))
        {
            spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
            aicwf_dev_skb_free(skb);
            aicwf_usb_rx_buf_put(usb_dev, usb_buf);
            aicwf_usb_rx_submit_all_urb_(usb_dev);
            return;
        }
        spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
        atomic_inc(&rx_priv->rx_cnt);

        /* rx_cnt==1: queue went from empty to non-empty, wake the RX processing thread */
        if (atomic_read(&rx_priv->rx_cnt) == 1)
        {
            complete(&rx_priv->usbdev->bus_if->busrx_trgg);
        }

        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
        aicwf_usb_rx_submit_all_urb_(usb_dev);  /* continuous receive: resubmit URB */
    }
    else
    {
        aicwf_dev_skb_free(skb);
        aicwf_usb_rx_buf_put(usb_dev, usb_buf);
    }
}
#endif /* 0 — end of Port -> CO callback Linux reference implementation */

/* ---- OOB (normally unused on USB, kept for special platforms) ---- */

#ifdef CONFIG_OOB
void aicwf_usb_oob_enable(void)
{
    DBG_USB_INF("%s: stub (no-op)\n", __func__);
}
#endif /* CONFIG_OOB */

/* ================================================================ */
/*  USB Host Driver registration (RT-Thread USB core)              */
/* ================================================================ */

/**
 * aicwf_usb_probe - USB device insertion probe callback
 *
 * Only claims the device and parses endpoints; no URB allocation here.
 * The URB pool and bus start are handled by wifi_init_cmd -> aicwf_usb_host_init.
 * On re-enumeration after the FW soft-disconnect, the reconnect recovery path is taken.
 */
static int aicwf_usb_probe(struct usb_interface *intf,
                           const struct usb_device_id *id)
{
    int ret;
    uint16_t chipid;
    uint16_t pid = intf->usb_dev->descriptor.idProduct;

    (void)id;

    DBG_USB_INF("%s: AIC8800 USB detected, vid=0x%04x pid=0x%04x expect_reconn=%d\n",
                __func__, intf->usb_dev->descriptor.idVendor, pid,
                s_expect_reconnect);

    /* Skip non-WLAN interfaces such as BT. Return 0: this driver does not claim it and enumeration continues;
     * do not clear s_expect_reconnect, leave it for the real WLAN if to finish reconnection. */
    if (!aicwf_usb_is_wlan_intf(intf))
    {
        struct usb_interface_descriptor *d = &intf->altsetting[0].desc;
        DBG_USB_INF("%s: skip non-WLAN if#%u class=%02x/%02x/%02x\n",
                    __func__, d->bInterfaceNumber,
                    d->bInterfaceClass, d->bInterfaceSubClass,
                    d->bInterfaceProtocol);
#ifdef AIC_FEATURE_BT
        /* The RT-Thread USB core matches only the first driver; the BT if must be handed over here */
        {
            extern int aic_btusb_try_bind(struct usb_interface * i);
            if (d->bInterfaceClass == USB_CLASS_WIRELESS_CONTROLLER)
                (void)aic_btusb_try_bind(intf);
        }
#endif
        return 0;
    }

    /* Re-enumeration after the FW soft-disconnect: restore pipes and notify the waiting thread */
    if (s_expect_reconnect)
    {
        ret = aicwf_usb_set_device(intf->usb_dev, intf);
        if (ret != 0)
        {
            DBG_USB_ERR("%s: reconnect parse endpoints failed\n", __func__);
            return ret;
        }
        chipid = aicwf_usb_pid_to_chipid(intf->usb_dev->descriptor.idVendor, pid);
        g_aic_usb_dev.chipid = chipid;
        g_runtime_vid = intf->usb_dev->descriptor.idVendor;
        g_runtime_pid = pid;
        g_runtime_chipid = chipid;
        usb_set_intfdata(intf, &g_aic_usb_dev);

        /* Resubmit RX URBs */
        aicwf_usb_rx_submit_all();

        s_expect_reconnect = 0;
        if (s_reconnect_sem_inited)
        {
            aicwf_rtos_semaphore_signal(s_reconnect_sem, false);
        }
        DBG_USB_INF("%s: reconnect claim success pid=0x%04x chipid=%u\n",
                    __func__, pid, chipid);
        return 0;
    }

    if (s_udev != NULL)
    {
        DBG_USB_INF("%s: already claimed, skip\n", __func__);
        return 0;
    }

    ret = aicwf_usb_set_device(intf->usb_dev, intf);
    if (ret != 0)
    {
        DBG_USB_ERR("%s: parse endpoints failed\n", __func__);
        return ret;
    }

    chipid = aicwf_usb_pid_to_chipid(intf->usb_dev->descriptor.idVendor, pid);
    g_aic_usb_dev.chipid = chipid;
    g_runtime_vid = intf->usb_dev->descriptor.idVendor;
    g_runtime_pid = pid;
    g_runtime_chipid = chipid;
    usb_set_intfdata(intf, &g_aic_usb_dev);

    DBG_USB_INF("%s: claim success chipid=%u\n", __func__, chipid);
    return 0;
}

/**
 * aicwf_usb_disconnect - USB device removal disconnect callback
 *
 * Soft-disconnect after FW start_app: only clear the device handle and stop URBs, keep the URB pool for re-enumeration.
 * Real removal: full deinit.
 */
static void aicwf_usb_disconnect(struct usb_interface *intf)
{
    DBG_USB_INF("%s: disconnected expect_reconn=%d\n",
                __func__, s_expect_reconnect);

    aicwf_usb_kill_all_tx_urb();
    aicwf_usb_kill_all_rx_urb();

    usb_set_intfdata(intf, NULL);
    s_udev = NULL;
    s_intf = NULL;
    s_bulk_in_pipe = 0;
    s_bulk_out_pipe = 0;
    s_msg_in_pipe = 0;
    s_msg_out_pipe = 0;

    if (!s_expect_reconnect)
    {
        /* Real removal: free the URB pool */
        aicwf_usb_deinit();
    }
}

/**
 * aicwf_usb_expect_reconnect - called before start_app to prepare for the soft-disconnect
 */
void aicwf_usb_expect_reconnect(void)
{
    if (!s_reconnect_sem_inited)
    {
        aicwf_rtos_semaphore_create(&s_reconnect_sem, "aic_reconn", 1, 0);
        s_reconnect_sem_inited = 1;
    }
    /* Drain any leftover signal (timeout=0 means non-blocking trytake semantics) */
    while (aicwf_rtos_semaphore_wait(s_reconnect_sem, 0) == AIC_RTOS_SUCCESS)
    {
        /* drain */
    }
    s_expect_reconnect = 1;
    DBG_USB_INF("%s: armed\n", __func__);
}

/**
 * aicwf_usb_wait_reconnect - wait for re-enumeration after the soft-disconnect
 * @timeout_ms: timeout in milliseconds
 * Return: 0 on success, negative on timeout/failure
 */
int aicwf_usb_wait_reconnect(uint32_t timeout_ms)
{
    int ret;
    uint32_t elapsed = 0;
    const uint32_t slice_ms = 200;
    struct usb_device *hub;
    struct usb_hcd *hcd;
    int power_cycled = 0;

    if (!s_reconnect_sem_inited)
    {
        DBG_USB_ERR("%s: sem not inited\n", __func__);
        return -ENODEV;
    }

    /* Already claimed by probe reconnect */
    if (s_udev != NULL && !s_expect_reconnect)
    {
        DBG_USB_INF("%s: already reclaimed\n", __func__);
        aicwf_rtos_msleep(500);
        return 0;
    }

    DBG_USB_INF("%s: waiting up to %u ms for re-enumeration\n",
                __func__, timeout_ms);

    hub = usb_get_root_hub();
    hcd = (hub && hub->bus) ? bus_to_hcd(hub->bus) : NULL;

    while (elapsed < timeout_ms)
    {
        if (s_udev != NULL && !s_expect_reconnect)
        {
            DBG_USB_INF("%s: reconnected ok (elapsed=%u)\n",
                        __func__, elapsed);
            /* Wait for the chip's open done / host_ready (a few hundred ms) before sending LMAC commands */
            aicwf_rtos_msleep(500);
            return 0;
        }

        /* Actively poll the root hub; do not scan ports too often to avoid disturbing enumeration */
        if (hcd)
        {
            usb_hcd_poll_rh_status(hcd);
        }
        if (hub && ((elapsed / slice_ms) % 5 == 0))
        {
            usb_hub_force_port_scan(hub);
        }

        ret = aicwf_rtos_semaphore_wait(s_reconnect_sem, (int)slice_ms);
        if (ret == AIC_RTOS_SUCCESS)
        {
            DBG_USB_INF("%s: reconnected ok\n", __func__);
            aicwf_rtos_msleep(500);
            return 0;
        }

        /* Every ~1s: dump port status + bypass lookup for 8d81/8d83 */
        if (hub && s_udev == NULL && ((elapsed / slice_ms) % 5 == 4))
        {
            uint16_t pst = 0, pch = 0;
            static const uint16_t pids[] =
            {
                AIC_PRODUCT_ID_D80_WIFI_BT,
                AIC_PRODUCT_ID_D80_WIFI,
            };
            unsigned i;

            if (usb_hub_get_port_status(hub, 1, &pst, &pch) == 0)
            {
                DBG_USB_INF("%s: port1 status=0x%04x change=0x%04x @%ums\n",
                            __func__, pst, pch, elapsed);
                /* Restore VBUS immediately if it was lost unexpectedly (bit8=PORT_POWER) */
                if (!(pst & 0x0100))
                {
                    DBG_USB_INF("%s: PORT_POWER clear, force power-on\n",
                                __func__);
                    usb_hub_port_power(hub, 1, 1);
                }
            }

            for (i = 0; i < sizeof(pids) / sizeof(pids[0]); i++)
            {
                struct usb_device *dev =
                    usb_find_child_by_vidpid(hub, AIC_VENDOR_ID, pids[i]);
                if (dev && dev->config)
                {
                    struct usb_interface *wif = NULL;
                    unsigned ifn;
                    /* Find WLAN on a multi-interface device; do not assume interface[0] (usually BT) */
                    for (ifn = 0; ifn < USB_MAXINTERFACES; ifn++)
                    {
                        struct usb_interface *cand =
                                dev->config[0].interface[ifn];
                        if (cand && aicwf_usb_is_wlan_intf(cand))
                        {
                            wif = cand;
                            break;
                        }
                    }
                    if (wif == NULL)
                        continue;
                    aicwf_usb_set_device(dev, wif);
                    usb_set_intfdata(wif, &g_aic_usb_dev);
                    aicwf_usb_rx_submit_all();
                    s_expect_reconnect = 0;
                    DBG_USB_INF("%s: bypass reclaim ok pid=0x%04x if#%u @%ums\n",
                                __func__, pids[i],
                                wif->altsetting[0].desc.bInterfaceNumber,
                                elapsed);
                    /* Wait for FW host_ready / open done */
                    aicwf_rtos_msleep(500);
                    return 0;
                }
            }
            DBG_USB_INF("%s: still waiting... %u/%u ms (udev=%p)\n",
                        __func__, elapsed, timeout_ms, s_udev);
        }

        /*
         * Still no device after ~5s: power-cycle VBUS once (the HCD must allow Set POWER while disconnected).
         * A power-cycle that comes too early may interrupt module bootstrap; verify the PORT_POWER bit after power-on.
         */
        if (hub && !power_cycled && elapsed >= 5000 && s_udev == NULL)
        {
            uint16_t pst = 0, pch = 0;

            DBG_USB_INF("%s: port1 power-cycle to recover device\n",
                        __func__);
            usb_hub_port_power(hub, 1, 0);
            aicwf_rtos_msleep(500);
            if (usb_hub_get_port_status(hub, 1, &pst, &pch) == 0)
            {
                DBG_USB_INF("%s: after power-off status=0x%04x\n",
                            __func__, pst);
            }
            usb_hub_port_power(hub, 1, 1);
            aicwf_rtos_msleep(200);
            if (usb_hub_get_port_status(hub, 1, &pst, &pch) == 0)
            {
                DBG_USB_INF("%s: after power-on status=0x%04x\n",
                            __func__, pst);
                if (!(pst & 0x0100))
                {
                    DBG_USB_ERR("%s: POWER still clear, retry once\n",
                                __func__);
                    usb_hub_port_power(hub, 1, 1);
                    aicwf_rtos_msleep(100);
                    if (usb_hub_get_port_status(hub, 1, &pst, &pch) == 0)
                    {
                        DBG_USB_INF("%s: retry power-on status=0x%04x\n",
                                    __func__, pst);
                    }
                }
            }
            if (hcd)
            {
                usb_hcd_poll_rh_status(hcd);
            }
            usb_hub_force_port_scan(hub);
            power_cycled = 1;
        }

        elapsed += slice_ms;
    }

    s_expect_reconnect = 0;
    DBG_USB_ERR("%s: timeout after %u ms\n", __func__, timeout_ms);
    return -ETIMEDOUT;
}

/* USB device ID table — bootrom 8d80 + operational 8d81/8d83 */
static const struct usb_device_id aicwf_usb_ids[] =
{
    { USB_DEVICE(AIC_VENDOR_ID, AIC_PRODUCT_ID_D80) },
    { USB_DEVICE(AIC_VENDOR_ID, AIC_PRODUCT_ID_D80_WIFI_BT) },
    { USB_DEVICE(AIC_VENDOR_ID, AIC_PRODUCT_ID_D80_WIFI) },
    { USB_DEVICE(AIC_VENDOR_ID, AIC_PRODUCT_ID_D80N) },
    { USB_DEVICE(AIC_VENDOR_ID_AIC_V2, AIC_PRODUCT_ID_D80X2) },
    { USB_DEVICE(AIC_VENDOR_ID_AIC_V2, AIC_PRODUCT_ID_D40X2) },
    { USB_DEVICE(AIC_VENDOR_ID, AIC_PRODUCT_ID_DC_USB) },
    { USB_DEVICE(AIC_VENDOR_ID_LEGACY, AIC_PRODUCT_ID_D80_LEGACY) },
    { USB_DEVICE(AIC_VENDOR_ID_LEGACY, AIC_PRODUCT_ID_DC) },
    { } /* Terminating entry */
};

/* USB driver structure */
static struct usb_driver aicwf_usb_driver =
{
    .name = DRV_NAME_USB,
    .probe = aicwf_usb_probe,
    .disconnect = aicwf_usb_disconnect,
    .id_table = aicwf_usb_ids,
};

/**
 * aicwf_usb_drv_register - register the USB driver with the RT-Thread USB core
 */
int aicwf_usb_drv_register(void)
{
    int ret;

    DBG_USB_INF("%s: registering AIC8800 USB WiFi driver\n", __func__);

    ret = usb_register_driver(&aicwf_usb_driver);
    if (ret != 0)
    {
        DBG_USB_ERR("%s: usb_register_driver failed %d\n", __func__, ret);
        return ret;
    }

#ifdef AIC_FEATURE_BT
    {
        extern int aic_btusb_drv_register(void);
        if (aic_btusb_drv_register() != 0)
            DBG_USB_ERR("%s: aic_btusb register failed\n", __func__);
    }
#endif

    DBG_USB_INF("%s: driver registered successfully\n", __func__);
    return 0;
}

/**
 * aicwf_usb_drv_unregister - unregister the USB driver
 */
void aicwf_usb_drv_unregister(void)
{
    DBG_USB_INF("%s: unregistering AIC8800 USB driver\n", __func__);
    usb_deregister(&aicwf_usb_driver);
}

/* Auto init — board-level board_startup.c calls aicwf_usb_drv_register() before usb_init() */

#endif /* CONFIG_USB_SUPPORT */
