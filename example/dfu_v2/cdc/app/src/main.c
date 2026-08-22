/**
 * @file main.c
 * @brief CDC DFU V2 Example — User Application
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#define LOG_TAG "app"
#include <log.h>

/*============================================================================
 * Version — Change this to verify OTA success
 *============================================================================*/

#define APP_VERSION     "1.0.9"

/*============================================================================
 * DFU Trigger Protocol
 *
 * PC tool sends 4-byte magic over USB-CDC to tell user app:
 * "please reboot into DFU mode".
 *
 * Trigger:  0xAA 0x55 0xDF 0x00
 * ACK:      0xAA 0x55 0xDF 0x01  (then reboot)
 *============================================================================*/

#define DFU_TRIGGER_MAGIC_0     0xAA
#define DFU_TRIGGER_MAGIC_1     0x55
#define DFU_TRIGGER_MAGIC_2     0xDF
#define DFU_TRIGGER_CMD_ENTER   0x00
#define DFU_TRIGGER_RSP_ACK     0x01

/* Bootloader checks firmware_file_info in DFU_PAN_LOADER partition tail.
 * See dfu_pan_macro.h and bootloader main.c: boot_images_help() */

/*============================================================================
 * CherryUSB CDC ACM
 *============================================================================*/

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#define CDC_IN_EP   0x85
#define CDC_OUT_EP  0x03
#define CDC_INT_EP  0x86

#define USBD_VID            0x38f4
#define USBD_PID            0x0001
#define USBD_MAX_POWER      100
#define USB_CONFIG_SIZE     (9 + CDC_ACM_DESCRIPTOR_LEN)
#define CDC_MAX_MPS         64

/* --- Descriptors (callback style, matching SDK CherryUSB API) --- */

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
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },   /* Language */
    "SiFli",                         /* Manufacturer */
    "CDC DFU Example",               /* Product */
    "20260304",                      /* Serial */
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
    .device_descriptor_callback         = device_descriptor_callback,
    .config_descriptor_callback         = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback         = string_descriptor_callback,
};

/* --- State --- */

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_MAX_MPS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[64];

static volatile bool g_usb_configured = false;
static volatile bool g_dfu_trigger_requested = false;

/* --- Endpoint callbacks --- */

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    /* Check for DFU trigger magic: AA 55 DF 00 */
    if (nbytes >= 4 &&
        cdc_read_buffer[0] == DFU_TRIGGER_MAGIC_0 &&
        cdc_read_buffer[1] == DFU_TRIGGER_MAGIC_1 &&
        cdc_read_buffer[2] == DFU_TRIGGER_MAGIC_2 &&
        cdc_read_buffer[3] == DFU_TRIGGER_CMD_ENTER)
    {
        /* Send ACK */
        cdc_write_buffer[0] = DFU_TRIGGER_MAGIC_0;
        cdc_write_buffer[1] = DFU_TRIGGER_MAGIC_1;
        cdc_write_buffer[2] = DFU_TRIGGER_MAGIC_2;
        cdc_write_buffer[3] = DFU_TRIGGER_RSP_ACK;
        usbd_ep_start_write(busid, CDC_IN_EP, cdc_write_buffer, 4);

        g_dfu_trigger_requested = true;
    }

    /* Re-arm for next reception */
    usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
}

/* --- CDC control callbacks (required by CherryUSB) --- */

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;
    (void)dtr;
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
    line_coding->dwDTERate = 115200;
    line_coding->bDataBits = 8;
    line_coding->bParityType = 0;
    line_coding->bCharFormat = 0;
}

/* --- Endpoint and interface structs --- */

static struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out,
};

static struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in,
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

/* --- Event handler --- */

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event)
    {
    case USBD_EVENT_CONFIGURED:
        g_usb_configured = true;
        usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, CDC_MAX_MPS);
        break;
    case USBD_EVENT_RESET:
    case USBD_EVENT_DISCONNECTED:
    case USBD_EVENT_SUSPEND:
        g_usb_configured = false;
        break;
    default:
        break;
    }
}

/* --- Init --- */

static void cdc_init(void)
{
    usbd_desc_register(0, &cdc_descriptor);
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &intf0));
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &intf1));
    usbd_add_endpoint(0, &cdc_out_ep);
    usbd_add_endpoint(0, &cdc_in_ep);
    usbd_initialize(0, USBC_BASE, usbd_event_handler);
}

/*============================================================================
 * DFU V2 — User application uses middleware API to enter DFU mode
 *============================================================================*/

#include "dfu_app.h"

/*============================================================================
 * LVGL UI
 *============================================================================*/

#ifdef PKG_USING_LITTLEVGL2RTT
#include "littlevgl2rtt.h"
#include "lvgl.h"

static void ui_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1A1A2E), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "CDC DFU EXAMPLE");
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(0x00D4FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_title, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *lbl_brand = lv_label_create(scr);
    lv_label_set_text(lbl_brand, "SIFLI");
    lv_obj_set_style_text_color(lbl_brand, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_brand, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_brand, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *lbl_ver = lv_label_create(scr);
    lv_label_set_text(lbl_ver, "Ver: " APP_VERSION);
    lv_obj_set_style_text_color(lbl_ver, lv_color_hex(0x88FF88), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_ver, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_ver, LV_ALIGN_CENTER, 0, 35);
}
#endif

/*============================================================================
 * Main
 *============================================================================*/

int main(void)
{
    LOG_I("=== CDC DFU Example App ===");
    LOG_I("Version: %s", APP_VERSION);

    cdc_init();
    LOG_I("USB-CDC initialized, waiting for DFU trigger...");

#ifdef PKG_USING_LITTLEVGL2RTT
    /* Must init LVGL display driver before any lv_* calls */
    rt_err_t ret = littlevgl2rtt_init("lcd");
    if (ret == RT_EOK)
    {
        ui_create();
        LOG_I("UI created");
    }
    else
    {
        LOG_E("LVGL init failed: %d", ret);
    }
#endif

    while (1)
    {
        if (g_dfu_trigger_requested)
        {
            g_dfu_trigger_requested = false;
            dfu_enter_dfu_mode();
        }

#ifdef PKG_USING_LITTLEVGL2RTT
        {
            rt_uint32_t ms = lv_task_handler();
            rt_thread_mdelay(ms);
        }
#else
        rt_thread_mdelay(50);
#endif
    }

    return RT_EOK;
}