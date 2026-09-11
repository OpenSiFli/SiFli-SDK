/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

/* ================ USB common Configuration ================ */

#include <rtthread.h>

#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)

#ifndef CONFIG_USB_DBG_LEVEL
    #define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

/* Enable print with color */
#define CONFIG_USB_PRINTF_COLOR_ENABLE

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* data align size when use dma or use dcache */
#ifdef CONFIG_USB_DCACHE_ENABLE
    #define CONFIG_USB_ALIGN_SIZE 32 // 32 or 64
#else
    #define CONFIG_USB_ALIGN_SIZE 4
#endif

/* ================= USB HOST Stack Configuration ================== */

#define CONFIG_USBHOST_MAX_RHPORTS          1
#define CONFIG_USBHOST_MAX_EXTHUBS          0
#define CONFIG_USBHOST_MAX_EHPORTS          4
#define CONFIG_USBHOST_MAX_INTERFACES       8
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS 2
#define CONFIG_USBHOST_MAX_ENDPOINTS        4

#define CONFIG_USBHOST_MAX_CDC_ACM_CLASS 4
#define CONFIG_USBHOST_MAX_HID_CLASS     4
#define CONFIG_USBHOST_MAX_MSC_CLASS     2
#define CONFIG_USBHOST_MAX_AUDIO_CLASS   1
#define CONFIG_USBHOST_MAX_VIDEO_CLASS   1

#define CONFIG_USBHOST_DEV_NAMELEN 16

#ifndef CONFIG_USBHOST_PSC_PRIO
    #define CONFIG_USBHOST_PSC_PRIO 4
#endif
#ifndef CONFIG_USBHOST_PSC_STACKSIZE
    #define CONFIG_USBHOST_PSC_STACKSIZE 2048
#endif

/* Ep0 max transfer buffer */
#ifndef CONFIG_USBHOST_REQUEST_BUFFER_LEN
    #define CONFIG_USBHOST_REQUEST_BUFFER_LEN 2048
#endif

#ifndef CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT
    #define CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT 5000
#endif

/* This parameter affects usb performance, and depends on (TCP_WND)tcp receive windows size,
 * you can change to 2K ~ 16K and must be larger than TCP RX windows size in order to avoid being overflow.
 */
#ifndef CONFIG_USBHOST_RNDIS_ETH_MAX_RX_SIZE
    #define CONFIG_USBHOST_RNDIS_ETH_MAX_RX_SIZE (16384)
#endif

/* Because lwip do not support multi pbuf at a time, so increasing this variable has no performance improvement */
#ifndef CONFIG_USBHOST_RNDIS_ETH_MAX_TX_SIZE
    #define CONFIG_USBHOST_RNDIS_ETH_MAX_TX_SIZE (2048)
#endif

/* ================ USB Host Port Configuration ==================*/
#ifndef CONFIG_USBHOST_MAX_BUS
    #define CONFIG_USBHOST_MAX_BUS 1
#endif

/* ---------------- MUSB Configuration ---------------- */
#define CONFIG_USB_MUSB_PIPE_NUM 8
#define CONFIG_USB_MUSB_WITHOUT_MULTIPOINT

#ifndef usb_phyaddr2ramaddr
    #define usb_phyaddr2ramaddr(addr) (addr)
#endif

#ifndef usb_ramaddr2phyaddr
    #define usb_ramaddr2phyaddr(addr) (addr)
#endif

#define CONFIG_USB_MUSB_SIFLI

#ifdef SOC_SF32LB58X
    #define CONFIG_USB_HS
#endif

#endif
