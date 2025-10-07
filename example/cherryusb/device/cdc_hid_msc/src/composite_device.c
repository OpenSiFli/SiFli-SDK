/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "usbd_core.h"
#include "usbd_msc.h"
#include "usbd_cdc_acm.h"
#include "usbd_hid.h"
#include "composite_device.h"
#include <stdio.h>
#include <string.h>
#include "bf0_hal.h"

/* USB Device Status */
static bool usb_device_initialized = false;
static bool usb_device_configured = false;

/* Endpoint addresses - Use same allocation as working MSC example */
// MSC endpoints
#define MSC_IN_EP  0x85
#define MSC_OUT_EP 0x02

// CDC ACM endpoints  
#define CDC_IN_EP  0x87
#define CDC_OUT_EP 0x03
#define CDC_INT_EP 0x81

// HID endpoints
#define HID_INT_EP 0x86

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#define HID_MAX_MPS 64
#else
#define CDC_MAX_MPS 64
#define HID_MAX_MPS 64
#endif

#define MSC_MAX_MPS 64

/* Descriptor lengths */
#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN + CDC_ACM_DESCRIPTOR_LEN + HID_KEYBOARD_DESCRIPTOR_LEN)

/* Global buffers - declare before use */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_keyboard_report[8];
volatile bool cdc_tx_busy_flag = false;
volatile bool hid_tx_busy_flag = false;
volatile uint8_t cdc_dtr_enable = 0;

/* HID Report Descriptor for Keyboard */
static const uint8_t hid_keyboard_report_desc[] = {
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    0x05, 0x07, // Usage Page (Key Codes)
    0x19, 0xE0, // Usage Minimum (224)
    0x29, 0xE7, // Usage Maximum (231)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x75, 0x01, // Report Size (1)
    0x95, 0x08, // Report Count (8)
    0x81, 0x02, // Input (Data, Variable, Absolute)
    0x95, 0x01, // Report Count (1)
    0x75, 0x08, // Report Size (8)
    0x81, 0x01, // Input (Constant)
    0x95, 0x05, // Report Count (5)
    0x75, 0x01, // Report Size (1)
    0x05, 0x08, // Usage Page (Page# for LEDs)
    0x19, 0x01, // Usage Minimum (1)
    0x29, 0x05, // Usage Maximum (5)
    0x91, 0x02, // Output (Data, Variable, Absolute)
    0x95, 0x01, // Report Count (1)
    0x75, 0x03, // Report Size (3)
    0x91, 0x01, // Output (Constant)
    0x95, 0x06, // Report Count (6)
    0x75, 0x08, // Report Size (8)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x65, // Logical Maximum (101)
    0x05, 0x07, // Usage Page (Key codes)
    0x19, 0x00, // Usage Minimum (0)
    0x29, 0x65, // Usage Maximum (101)
    0x81, 0x00, // Input (Data, Array)
    0xC0        // End Collection
};

/* Device descriptor */
static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01),
};

/* Configuration descriptor */
static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x04, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    
    /* MSC Interface */
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x02),
    
    /* CDC ACM Interface */
    CDC_ACM_DESCRIPTOR_INIT(0x01, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x03),
    
    /* HID Keyboard Interface */
    HID_KEYBOARD_DESCRIPTOR_INIT(0x03, 0x01, sizeof(hid_keyboard_report_desc), HID_INT_EP, HID_MAX_MPS, 0x01),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const char *string_descriptors[] = {
    (const char[]){0x09, 0x04}, /* Langid */
    "SiFli",                    /* Manufacturer */
    "SiFli Composite Device",   /* Product */
    "2025010101",               /* Serial Number */
};

/* Descriptor callback functions */
static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor composite_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

/* USB Event Handler */
static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            usb_device_configured = false;
            break;
        case USBD_EVENT_CONNECTED:
            rt_kprintf("USB connected\n");
            break;
        case USBD_EVENT_DISCONNECTED:
            rt_kprintf("USB disconnected\n");
            usb_device_configured = false;
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            rt_kprintf("USB configured - device ready!\n");
            usb_device_configured = true;
            /* Setup CDC ACM first out ep read transfer */
            usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, sizeof(cdc_read_buffer));
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;
        default:
            break;
    }
}

/* USB Device Status Check Function */
bool composite_device_is_ready(uint8_t busid)
{
    return usb_device_initialized && usb_device_configured;
}

/* ================ MSC RAM Implementation ================ */

#define README_CONTENTS \
"SiFli USB Composite Device Demo!\r\n" \
"Features:\r\n" \
"- MSC: Virtual Disk (RAM-based)\r\n" \
"- CDC ACM: Virtual COM Port\r\n" \
"- HID: Virtual Keyboard\r\n"

/* 8KB is the smallest size that windows allow to mount */
enum
{
    DISK_BLOCK_NUM  = 16,
    DISK_BLOCK_SIZE = 512
};

static uint8_t msc_disk[DISK_BLOCK_NUM][DISK_BLOCK_SIZE] =
{
    /*------------- Block0: Boot Sector -------------*/
    /* byte_per_sector    = DISK_BLOCK_SIZE; fat12_sector_num_16  = DISK_BLOCK_NUM;
     * sector_per_cluster = 1; reserved_sectors = 1;
     * fat_num            = 1; fat12_root_entry_num = 16;
     * sector_per_fat     = 1; sector_per_track = 1; head_num = 1; hidden_sectors = 0;
     * drive_number       = 0x80; media_type = 0xf8; extended_boot_signature = 0x29;
     * filesystem_type    = "FAT12   "; volume_serial_number = 0x1234; volume_label = "SiFli MSC";
     */
    {
        0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, 0x00, 0x02, 0x01, 0x01, 0x00,
        0x01, 0x10, 0x00, DISK_BLOCK_NUM, 0x00, 0xF8, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x29, 0x34, 0x12, 0x00, 0x00, 'S', 'i', 'F', 'l', 'i',
        ' ', 'M', 'S', 'C', ' ', ' ', 0x46, 0x41, 0x54, 0x31, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00,

        /* Zero up to 2 last bytes of FAT magic code */
        [64 ... 509] = 0x00,
        0x55, 0xAA
    },

    /*------------- Block1: FAT12 Table -------------*/
    {
        0xF8, 0xFF, 0xFF, 0xFF, 0x0F /* first 2 entries must be F8FF, third entry is cluster end of readme file */
    },

    /*------------- Block2: Root Directory -------------*/
    {
        /* first entry is volume label */
        'S', 'i', 'F', 'l', 'i', ' ', 'M', 'S', 'C', ' ', ' ', 0x08, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x6D, 0x65, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        /* second entry is readme file */
        'R', 'E', 'A', 'D', 'M', 'E', ' ', ' ', 'T', 'X', 'T', 0x20, 0x00, 0xC6, 0x52, 0x6D,
        0x65, 0x43, 0x65, 0x43, 0x00, 0x00, 0x88, 0x6D, 0x65, 0x43, 0x02, 0x00,
        sizeof(README_CONTENTS) - 1, 0x00, 0x00, 0x00 /* readme's files size (4 Bytes) */
    },

    /*------------- Block3: Readme Content -------------*/
    README_CONTENTS
};

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    *block_num = DISK_BLOCK_NUM;
    *block_size = DISK_BLOCK_SIZE;
}

int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    if (sector < DISK_BLOCK_NUM) {
        memcpy(buffer, &msc_disk[sector][0], length);
        return 0;
    }
    return -1;
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    if (sector < DISK_BLOCK_NUM) {
        memcpy(&msc_disk[sector][0], buffer, length);
        return 0;
    }
    return -1;
}

/* ================ CDC ACM Implementation ================ */

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("CDC actual out len:%d\r\n", (unsigned int)nbytes);
    // Echo back received data
    if (!cdc_tx_busy_flag && nbytes > 0) {
        cdc_tx_busy_flag = true;
        memcpy(cdc_write_buffer, cdc_read_buffer, nbytes);
        usbd_ep_start_write(busid, CDC_IN_EP, cdc_write_buffer, nbytes);
    }
    /* Setup next out ep read transfer */
    usbd_ep_start_read(busid, CDC_OUT_EP, cdc_read_buffer, sizeof(cdc_read_buffer));
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("CDC actual in len:%d\r\n", (unsigned int)nbytes);

    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        if (!cdc_tx_busy_flag) { 
            usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);
        }
    } else {
        cdc_tx_busy_flag = false;
    }
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    if (dtr) {
        cdc_dtr_enable = 1;
    } else {
        cdc_dtr_enable = 0;
    }
}

/* ================ HID Keyboard Implementation ================ */

void usbd_hid_int_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    hid_tx_busy_flag = false;
}

/* ASCII to HID keycode conversion table */
static const uint8_t ascii_to_hid[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x00-0x07
    0x2a, 0x2b, 0x28, 0x00, 0x00, 0x28, 0x00, 0x00, // 0x08-0x0f (backspace, tab, enter)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x10-0x17
    0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, 0x00, // 0x18-0x1f (escape)
    0x2c, 0x1e, 0x34, 0x20, 0x21, 0x22, 0x24, 0x34, // 0x20-0x27 ( !"#$%&')
    0x26, 0x27, 0x25, 0x2e, 0x36, 0x2d, 0x37, 0x38, // 0x28-0x2f ()*+,-./)
    0x27, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, // 0x30-0x37 (01234567)
    0x25, 0x26, 0x33, 0x33, 0x36, 0x2e, 0x37, 0x38, // 0x38-0x3f (89:;<=>?)
    0x1f, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, // 0x40-0x47 (@ABCDEFG)
    0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, // 0x48-0x4f (HIJKLMNO)
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, // 0x50-0x57 (PQRSTUVW)
    0x1b, 0x1c, 0x1d, 0x2f, 0x31, 0x30, 0x23, 0x2d, // 0x58-0x5f (XYZ[\]^_)
    0x35, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, // 0x60-0x67 (`abcdefg)
    0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, // 0x68-0x6f (hijklmno)
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, // 0x70-0x77 (pqrstuvw)
    0x1b, 0x1c, 0x1d, 0x2f, 0x31, 0x30, 0x35, 0x4c, // 0x78-0x7f (xyz{|}~DEL)
};

void hid_keyboard_send_char(uint8_t busid, char ch)
{
    if (hid_tx_busy_flag || !composite_device_is_ready(busid)) return;
    
    uint8_t keycode = 0;
    uint8_t modifier = 0;
    
    // Convert ASCII to HID keycode
    if (ch >= 0 && ch < sizeof(ascii_to_hid)) {
        keycode = ascii_to_hid[(uint8_t)ch];
        
        // Handle uppercase letters and special characters that need shift
        if (ch >= 'A' && ch <= 'Z') {
            modifier = HID_MODIFIER_LSHIFT;
        } else if (ch == '!' || ch == '@' || ch == '#' || ch == '$' || ch == '%' || 
                   ch == '^' || ch == '&' || ch == '*' || ch == '(' || ch == ')') {
            modifier = HID_MODIFIER_LSHIFT;
        }
    }
    
    if (keycode != 0) {
        // Send key press
        memset(hid_keyboard_report, 0, sizeof(hid_keyboard_report));
        hid_keyboard_report[0] = modifier;
        hid_keyboard_report[2] = keycode;
        
        hid_tx_busy_flag = true;
        usbd_ep_start_write(busid, HID_INT_EP, hid_keyboard_report, sizeof(hid_keyboard_report));
        
        // Wait for completion
        while (hid_tx_busy_flag) {
            rt_thread_mdelay(1);
        }
        
        // Send key release
        memset(hid_keyboard_report, 0, sizeof(hid_keyboard_report));
        hid_tx_busy_flag = true;
        usbd_ep_start_write(busid, HID_INT_EP, hid_keyboard_report, sizeof(hid_keyboard_report));
        
        while (hid_tx_busy_flag) {
            rt_thread_mdelay(1);
        }
        
        // Small delay between characters
        rt_thread_mdelay(10);
    }
}

void hid_keyboard_send_string(uint8_t busid, const char* str)
{
    if (!str || !composite_device_is_ready(busid)) {
        rt_kprintf("HID: Device not ready or invalid string\n");
        return;
    }
    
    rt_kprintf("HID: Sending string: %s\n", str);
    
    for (int i = 0; str[i] != '\0'; i++) {
        hid_keyboard_send_char(busid, str[i]);
    }
}

/* HID class callbacks */
void usbd_hid_get_report(uint8_t busid, uint8_t intf, uint8_t report_id, uint8_t report_type, uint8_t **data, uint32_t *len)
{
    *data = hid_keyboard_report;
    *len = sizeof(hid_keyboard_report);
}

/* Interface and endpoint structures */
static struct usbd_interface intf0; // MSC
static struct usbd_interface intf1; // CDC ACM Control
static struct usbd_interface intf2; // CDC ACM Data  
static struct usbd_interface intf3; // HID Keyboard

struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

struct usbd_endpoint hid_in_ep = {
    .ep_addr = HID_INT_EP,
    .ep_cb = usbd_hid_int_in
};

/* Main initialization function */
void composite_device_init(uint8_t busid, uint32_t reg_base)
{
    rt_kprintf("Initializing USB Composite Device...\n");

    // Initialize CDC write buffer
    const uint8_t test_data[10] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30 };
    memcpy(&cdc_write_buffer[0], test_data, 10);
    memset(&cdc_write_buffer[10], 'a', 2038);

    // Register descriptor
    usbd_desc_register(busid, &composite_descriptor);

    // Add MSC interface
    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));

    // Add CDC ACM interfaces
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf2));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);

    // Add HID interface
    usbd_add_interface(busid, usbd_hid_init_intf(busid, &intf3, hid_keyboard_report_desc, sizeof(hid_keyboard_report_desc)));
    usbd_add_endpoint(busid, &hid_in_ep);

    // Initialize USB
    usbd_initialize(busid, reg_base, usbd_event_handler);

    // Mark as initialized
    usb_device_initialized = true;

    rt_kprintf("USB Composite Device initialized successfully!\n");
    rt_kprintf("- MSC (Virtual Disk): Available (RAM-based, 8KB)\n");
    rt_kprintf("- CDC ACM (Virtual COM): Available\n");
    rt_kprintf("- HID Keyboard: Available\n");
    rt_kprintf("Waiting for USB enumeration...\n");
}