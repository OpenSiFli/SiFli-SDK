/**
 * @file dfu_mode_host.h
 * @brief DFU V2 Host-Initiated Mode Interface
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_MODE_HOST_H__
#define __DFU_MODE_HOST_H__

#include "dfu_mode.h"
#include "../transport/dfu_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize Host-Initiated Mode
 *
 * Binds a Push Transport for sending response frames, and creates
 * the internal Protocol parser instance.
 *
 * @param transport            Push Transport vtable (e.g. CDC send). Must not be NULL.
 * @param is_message_transport true for message-boundary transports (BLE) that
 *                             don't need AA55/CRC16 framing on response path.
 *                             false for byte-stream transports (CDC, UART).
 * @param transport_name       Short name for logging ("cdc", "ble", etc.)
 * @param callback             Event callback for APP layer (may be NULL)
 * @param user_data            Context for callback
 * @return 0 on success, negative on failure
 */
int dfu_mode_host_init(const dfu_push_transport_t *transport,
                       bool is_message_transport,
                       const char *transport_name,
                       dfu_mode_callback_t callback,
                       void *user_data);

/**
 * @brief Deinitialize Host-Initiated Mode
 *
 * Destroys internal parser, clears state. If an upgrade session is
 * active, calls dfu_engine_abort() first.
 */
void dfu_mode_host_deinit(void);

/*============================================================================
 * Data Entry Points
 *============================================================================*/

/**
 * @brief Feed raw byte stream data (for byte-stream Transports)
 *
 * Called by APP main loop after receiving DFU_MSG_CDC_DATA or
 * DFU_MSG_UART_DATA from the mailbox. The data may be a partial frame,
 * one complete frame, or multiple frames concatenated.
 *
 * Internally:
 *   1. Bytes are fed to the Protocol parser (AA55 frame state machine).
 *   2. Each complete frame triggers dispatch_command().
 *   3. dispatch_command() checks Engine abort flag before processing.
 *   4. Appropriate Engine API is called, response frame is built
 *      and sent via transport->send().
 *
 * @param data  Raw bytes from Transport buffer
 * @param len   Number of bytes
 * @return 0 on success, negative on error
 *
 * @note Must be called from APP main loop context, NOT from ISR.
 */
int dfu_mode_host_feed_data(const uint8_t *data, uint32_t len);

/**
 * @brief Feed a complete protocol message (for message-boundary Transports)
 *
 * Called by APP main loop after receiving DFU_MSG_BLE_DATA from the mailbox.
 * The message is already a complete protocol unit (msg_id + length + payload),
 * matching the dfu_tran_protocol_t format from the existing protocol.
 *
 * Skips frame assembly — goes directly to dispatch_command().
 *
 * @param data  Complete message (msg_id(2B) + length(2B) + payload(NB))
 * @param len   Total message length
 * @return 0 on success, negative on error
 *
 * @note Must be called from APP main loop context, NOT from ISR.
 */
int dfu_mode_host_feed_message(const uint8_t *data, uint32_t len);

/*============================================================================
 * Status Query
 *============================================================================*/

/**
 * @brief Check if Host Mode has an active upgrade session
 *
 * @return true if session is active, false if idle
 */
bool dfu_mode_host_is_busy(void);

/**
 * @brief Set the image ID of the currently running partition
 *
 * When Host Mode encounters this img_id in IMG_SEND_START, it enters
 * skip mode: ACKs all packets for that image without writing to flash.
 * This prevents overwriting the partition the program is running from.
 *
 * Examples:
 *   - User APP runs from HCPU partition → set self_img_id = 0 (HCPU)
 *   - DFU subprogram runs from DFU partition → set self_img_id = 6 (OTA_MANAGER)
 *
 * @param img_id  Image ID to skip, or 0xFF to disable filtering
 */
void dfu_mode_host_set_self_img_id(uint8_t img_id);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_MODE_HOST_H__ */