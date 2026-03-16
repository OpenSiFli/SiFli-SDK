/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USB_CONFIG_H
#define USB_CONFIG_H

/* ================ USB common Configuration ================ */

#include <rtthread.h>

#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

/* Enable print with color */
#define CONFIG_USB_PRINTF_COLOR_ENABLE

/* data align size when use dma or use dcache */
#define CONFIG_USB_ALIGN_SIZE 4

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* ================= USB Device Stack Configuration ================ */

/* Ep0 in and out transfer buffer */
#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 256
#endif

/* enable advance desc register api */
#define CONFIG_USBDEV_ADVANCE_DESC

/* ================= USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1
#endif

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 8
#endif

/* ================= USB Host Stack Configuration ================ */

/* USB Host相关配置 - 定义为最小值以节省内存 */
#ifndef CONFIG_USBHOST_MAX_ENDPOINTS
#define CONFIG_USBHOST_MAX_ENDPOINTS 2
#endif

#ifndef CONFIG_USBHOST_DEV_NAMELEN
#define CONFIG_USBHOST_DEV_NAMELEN 16
#endif

#ifndef CONFIG_USBHOST_MAX_INTF_ALTSETTINGS
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS 1
#endif

#ifndef CONFIG_USBHOST_MAX_INTERFACES
#define CONFIG_USBHOST_MAX_INTERFACES 1
#endif

#ifndef CONFIG_USBHOST_MAX_EHPORTS
#define CONFIG_USBHOST_MAX_EHPORTS 1
#endif

#ifndef CONFIG_USBHOST_MAX_BUS
#define CONFIG_USBHOST_MAX_BUS 1
#endif

#ifndef CONFIG_USBHOST_MAX_EXTHUBS
#define CONFIG_USBHOST_MAX_EXTHUBS 1
#endif

#ifndef CONFIG_USBHOST_MAX_CDC_ACM_CLASS
#define CONFIG_USBHOST_MAX_CDC_ACM_CLASS 1
#endif

#ifndef CONFIG_USBHOST_MAX_HID_CLASS
#define CONFIG_USBHOST_MAX_HID_CLASS 1
#endif

#ifndef CONFIG_USBHOST_MAX_MSC_CLASS
#define CONFIG_USBHOST_MAX_MSC_CLASS 1
#endif

#ifndef CONFIG_USBHOST_MAX_AUDIO_CLASS
#define CONFIG_USBHOST_MAX_AUDIO_CLASS 1
#endif

#ifndef CONFIG_USBHOST_MAX_VIDEO_CLASS
#define CONFIG_USBHOST_MAX_VIDEO_CLASS 1
#endif

/* ---------------- MUSB Configuration for SiFli ---------------- */
#define CONFIG_USB_MUSB_SIFLI

/* ================= MUSB 端点配置 ================ */
#ifndef CONFIG_USB_MUSB_EP_NUM
#define CONFIG_USB_MUSB_EP_NUM 8
#endif

/* 对于 SF32LB52x 系列，根据具体芯片型号配置 */
#undef CONFIG_USB_HS

/* ================= HID Device Configuration ================ */
#ifndef CONFIG_USBDEV_HID_MAX_INTF
#define CONFIG_USBDEV_HID_MAX_INTF 1
#endif

/* ================= CDC Device Configuration ================ */
#ifndef CONFIG_USBDEV_CDC_ACM_MAX_INTF
#define CONFIG_USBDEV_CDC_ACM_MAX_INTF 2
#endif

/* USB Memory Alignment Macros */
#ifndef USB_MEM_ALIGNX
#define USB_MEM_ALIGNX __attribute__((aligned(CONFIG_USB_ALIGN_SIZE)))
#endif

/* Helper macros - 仅在CherryUSB未定义时才定义 */
#ifndef WBVAL
#define WBVAL(x) (x & 0xFF), ((x >> 8) & 0xFF)
#endif

#ifndef DBVAL
#define DBVAL(x) (x & 0xFF), ((x >> 8) & 0xFF), ((x >> 16) & 0xFF), ((x >> 24) & 0xFF)
#endif

#endif /* USB_CONFIG_H */