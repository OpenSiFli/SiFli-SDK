/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file framed_proto_link.h
 * @brief Link side of the framed message transport.
 *
 * Two kinds of things live here:
 *  - link type queries, plus the assembly hooks that the Bluetooth serial
 *    transmission service provides on other platforms but this SDK does not;
 *  - the hooks the application TCP relay uses to take over the channel.
 */

#ifndef __FRAMED_PROTO_LINK_H
#define __FRAMED_PROTO_LINK_H

#include <stdint.h>

#ifdef BSP_BLE_SIBLES
    #include "bf0_ble_gap.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Links this transport can run over. */
typedef enum
{
    FRAMED_PROTO_LINK_BLE,
    FRAMED_PROTO_LINK_SPP,
    FRAMED_PROTO_LINK_PAN,
    FRAMED_PROTO_LINK_WIFI,     /**< application TCP relay over WiFi. */
} framed_proto_link_type_t;

/**
 * @brief Transmission parameters reported by the link.
 */
typedef struct
{
    uint8_t handle;         /**< transmission channel handle. */
    uint16_t len;           /**< data length. */
    uint8_t *data;          /**< parameter block carried by the link. */
} framed_proto_link_configure_t;

enum
{
    /* Not reported by the sibles service of this SDK, whose
     * ble_serial_tran_event_t ends at BLE_SERIAL_TRAN_OPEN_FAILED (5). The
     * values only have to stay clear of it, so that a product extending the
     * link can forward them without a translation table. */
    FRAMED_PROTO_LINK_CONFIGURE = 6,
    FRAMED_PROTO_LINK_DATA_ERROR = 7,
};

/** @brief Link type currently carrying the transport. */
uint8_t framed_proto_link_type_get(void);

/** @brief Tell the transport that input arrives whole-frame. */
void framed_proto_link_assemble_mode_no_free_set(void);

/** @brief Clear whatever the streaming assembly left behind. */
void framed_proto_link_assemble_clean(void);

#ifdef BSP_BLE_SIBLES
/**
 * @brief Address of the peer currently connected over BLE.
 * @return peer address; all-zero when not connected
 */
bd_addr_t *framed_proto_get_peer_addr(void);
#endif

/* -------------------------------------------------------------------------
 * Application TCP relay hooks
 *
 * The weak defaults live in framed_proto_link.c so a plain-BLE product still
 * links; a product that moves the transport onto WiFi just provides strong
 * symbols.
 * ---------------------------------------------------------------------- */

/** @brief Non-zero when the application TCP relay has taken over the channel. */
uint8_t framed_proto_pan_is_active(void);

/**
 * @brief Non-zero when the relay runs on WiFi rather than Bluetooth PAN.
 *
 * Lets the upper layer pick WiFi-specific transport parameters without
 * peeking at lwIP netif names.
 */
uint8_t framed_proto_pan_is_wifi(void);

/** @brief Send one already-framed chunk through the application TCP relay. */
int framed_proto_pan_send_data(const void *data, uint32_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* __FRAMED_PROTO_LINK_H */
