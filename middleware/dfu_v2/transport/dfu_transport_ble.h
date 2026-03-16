/**
 * @file dfu_transport_ble.h
 * @brief DFU V2 BLE Transport — Adapter between mobile APP and Host Mode
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_TRANSPORT_BLE_H__
#define __DFU_TRANSPORT_BLE_H__

#include <rtthread.h>
#include "dfu_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * BLE Serial Transmission Constants
 *============================================================================*/

/** BLE Serial Transmission category ID for DFU service */
#ifndef BLE_DFU_CATEID
#define BLE_DFU_CATEID  0x01
#endif

/** Per-packet hash size in image_body_hdr (SHA-256) */
#define DFU_BLE_PKT_HASH_SIZE   32

/** image_body_hdr total size: hash[32] + offset[4] */
#define DFU_BLE_BODY_HDR_SIZE   (DFU_BLE_PKT_HASH_SIZE + 4)

/** Hash+sig wrapper sizes for INIT_REQ control packet */
#define DFU_BLE_CTRL_HASH_SIZE  32
#define DFU_BLE_CTRL_SIG_SIZE   256

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize BLE Transport
 *
 * Registers the BLE Serial Transmission callback for BLE_DFU_CATEID,
 * stores the APP mailbox handle for posting DFU_MSG_BLE_DATA.
 *
 * @param mb  APP layer mailbox handle. BLE callback will post
 *            DFU_MSG_BLE_DATA to this mailbox when data arrives.
 * @return 0 on success, negative on failure
 */
int dfu_transport_ble_init(rt_mailbox_t mb);

/**
 * @brief Deinitialize BLE Transport
 *
 * Cleans up internal state. The BLE Serial Transmission callback
 * is statically registered via BLE_SERIAL_TRAN_EXPORT and cannot
 * be dynamically unregistered, but this function resets internal
 * state so incoming data is ignored after deinit.
 */
void dfu_transport_ble_deinit(void);

/*============================================================================
 * Transport Instance
 *============================================================================*/

/**
 * @brief Get the BLE Push Transport instance
 *
 * Returns a pointer to the static dfu_push_transport_t vtable.
 * The send() function forwards response frames via
 * ble_serial_tran_send_data().
 *
 * @return Push Transport vtable pointer, or NULL if not initialized
 */
const dfu_push_transport_t *dfu_transport_ble_get_instance(void);

/*============================================================================
 * Message Processing (called from DFU main loop)
 *============================================================================*/

/**
 * @brief Process a pending BLE message
 *
 * Called by APP main loop after receiving DFU_MSG_BLE_DATA from mailbox.
 * Retrieves the buffered message, performs protocol translation
 * (strip security wrappers, verify hash, reformat fields), and feeds
 * the result to Host Mode via dfu_mode_host_feed_message().
 *
 * @return 0 on success, -1 if no data or error
 */
int dfu_transport_ble_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_TRANSPORT_BLE_H__ */