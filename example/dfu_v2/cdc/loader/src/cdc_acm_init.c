/**
 * @file cdc_acm_init.c
 * @brief USB CDC ACM initialization for DFU V2 subprogram
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "bf0_hal.h"
#include <string.h>

#ifdef DFU_V2_USE_CDC_TRANSPORT
#include "dfu_v2.h"
#include "transport/dfu_transport_cdc.h"
#endif

/*============================================================================
 * Endpoint Addresses
 *============================================================================*/

#define CDC_IN_EP  0x85
#define CDC_OUT_EP 0x03
#define CDC_INT_EP 0x86

/*============================================================================
 * USB Device Identifiers
 *============================================================================*/

#define USBD_VID           0x38f4
#define USBD_PID           0x0001
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*============================================================================
 * Configuration
 *============================================================================*/

#define USB_CONFIG_SIZE    (9 + CDC_ACM_DESCRIPTOR_LEN)
#define CDC_MAX_MPS        64

/* Mailbox message IDs for connection state (handled in main.c) */
#define USB_CDC_DFU_CONNECTED      10
#define USB_CDC_DFU_DISCONNECTED   11

/*============================================================================
 * External: APP mailbox from main.c
 *============================================================================*/

extern rt_mailbox_t g_bt_app_mb;

/*============================================================================
 * USB Descriptors (identical to V1.5)
 *============================================================================*/

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01,
                               USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP,
                            CDC_MAX_MPS, 0x02)
};

static const uint8_t device_quality_descriptor[] = {
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02,
    0x00, 0x00, 0x00,
    0x40,
    0x00,
    0x00
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "SiFli",
    "DFU V2 OTA",
    "2026030300",
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index >= sizeof(string_descriptors) / sizeof(string_descriptors[0]))
        return NULL;
    return string_descriptors[index];
}

static const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};

/*============================================================================
 * CDC ACM State
 *============================================================================*/

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_MAX_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[512];

volatile bool cdc_ep_tx_busy = false;
volatile bool cdc_configured = false;
volatile uint8_t cdc_dtr_enable = 0;
static volatile bool cdc_last_connected_state = false;

/*============================================================================
 * Connection State Notification
 *============================================================================*/

static void cdc_notify_connection_state(bool connected)
{
    if (g_bt_app_mb != RT_NULL)
    {
        rt_mb_send(g_bt_app_mb,
                   connected ? USB_CDC_DFU_CONNECTED : USB_CDC_DFU_DISCONNECTED);
    }
}

/*============================================================================
 * USB Event Handler
 *============================================================================*/

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event)
    {
    case USBD_EVENT_RESET:
        cdc_configured = false;
        break;

    case USBD_EVENT_DISCONNECTED:
        cdc_configured = false;
        cdc_dtr_enable = 0;
        if (cdc_last_connected_state)
        {
            cdc_last_connected_state = false;
            cdc_notify_connection_state(false);
        }
        break;

    case USBD_EVENT_CONFIGURED:
        cdc_ep_tx_busy = false;
        cdc_configured = true;
        /* Start first read on bulk OUT endpoint */
        usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
        break;

    default:
        break;
    }
}

/*============================================================================
 * CDC Endpoint Callbacks
 *============================================================================*/

/**
 * @brief Bulk OUT callback — USB data received from host
 *
 * Key difference from V1.5: instead of parsing frames here in ISR,
 * we feed raw bytes directly to V2 transport's ring buffer.
 * Frame parsing is done by dfu_protocol.c in main loop context.
 */
void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if (nbytes > 0)
    {
#ifdef DFU_V2_USE_CDC_TRANSPORT
        /* Feed raw bytes to V2 transport ring buffer.
         * This replaces V1.5's per-byte frame parsing in ISR. */
        dfu_transport_cdc_feed_raw(cdc_read_buffer, nbytes);
#endif
    }

    /* Continue receiving — always re-arm the endpoint */
    usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
    {
        /* Send ZLP for exact multiple of MPS */
        usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);
    }
    else
    {
        cdc_ep_tx_busy = false;
    }
}

static struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

static struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

/*============================================================================
 * Send Data to Host
 *============================================================================*/

int cdc_acm_send(const uint8_t *data, uint32_t len)
{
    if (!cdc_configured || !cdc_dtr_enable)
        return -1;

    if (len > sizeof(cdc_write_buffer))
        len = sizeof(cdc_write_buffer);

    memcpy(cdc_write_buffer, data, len);

    cdc_ep_tx_busy = true;
    usbd_ep_start_write(0, CDC_IN_EP, cdc_write_buffer, len);

    /* Wait for TX complete with timeout */
    uint32_t timeout = 1000;
    while (cdc_ep_tx_busy && timeout > 0)
    {
        rt_thread_mdelay(1);
        timeout--;
    }

    return cdc_ep_tx_busy ? -1 : (int)len;
}

bool cdc_acm_is_ready(void)
{
    return cdc_configured && cdc_dtr_enable;
}

/*============================================================================
 * Hardware Transmit — overrides weak function in dfu_transport_cdc.c
 *============================================================================*/

int dfu_cdc_hw_transmit(const uint8_t *data, uint16_t length)
{
    if (!cdc_configured || !cdc_dtr_enable)
        return -1;

    int ret = cdc_acm_send(data, (uint32_t)length);
    return (ret > 0) ? 0 : -1;
}

/*============================================================================
 * CDC ACM Control Callbacks
 *============================================================================*/

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;

    cdc_dtr_enable = dtr ? 1 : 0;
    bool is_connected = (cdc_configured && cdc_dtr_enable);

    if (is_connected && !cdc_last_connected_state)
    {
        cdc_last_connected_state = true;
        cdc_notify_connection_state(true);
    }
    else if (!is_connected && cdc_last_connected_state)
    {
        cdc_last_connected_state = false;
        cdc_notify_connection_state(false);
    }
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    (void)busid;
    (void)intf;
    (void)rts;
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf,
                                   struct cdc_line_coding *line_coding)
{
    (void)busid;
    (void)intf;
    (void)line_coding;
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf,
                                   struct cdc_line_coding *line_coding)
{
    (void)busid;
    (void)intf;
    line_coding->dwDTERate = 1000000;
    line_coding->bDataBits = 8;
    line_coding->bParityType = 0;
    line_coding->bCharFormat = 0;
}

/*============================================================================
 * Initialization
 *============================================================================*/

int cdc_acm_init(uint8_t busid, uintptr_t reg_base)
{
    usbd_desc_register(busid, &cdc_descriptor);

    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);

    return 0;
}

/*============================================================================
 * Auto Initialization
 *============================================================================*/

static int cdc_acm_auto_init(void)
{
    return cdc_acm_init(0, (uintptr_t)USBC_BASE);
}
INIT_APP_EXPORT(cdc_acm_auto_init);
