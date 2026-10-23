/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-owned USB type definitions.
 * Provides USB constants without platform dependencies.
 *
 * Note: USB endpoint numbers and vendor/product IDs are not defined here.
 *   - Endpoint numbers are discovered dynamically at runtime by the platform
 *     USB subsystem (cf. Linux aicwf_usb.c::aicwf_parse_usb walking the endpoint
 *     descriptors), and should not be hardcoded as constants.
 *   - Vendor/product IDs are used by the platform USB driver at registration
 *     (usb_device_id table); that is a platform-layer responsibility and does
 *     not belong in a host common header.
 *
 * USB_RX_BUF_COUNT/SIZE are defined in usb_co.h (counterpart of sdio_co.h for
 * SDIO) and are not duplicated here.
 */

#ifndef _AIC_USB_TYPES_H_
#define _AIC_USB_TYPES_H_

#include <stdint.h>

/* USB TX aggregation limits (mirror SDIO defines) */
#define USB_TX_SLIST_MAX        136

/* USB scatter-gather list (parallel to sdio_sg_list) */
typedef struct
{
    void     *addr;
    uint32_t  len;
} usb_sg_list;

#endif /* _AIC_USB_TYPES_H_ */
