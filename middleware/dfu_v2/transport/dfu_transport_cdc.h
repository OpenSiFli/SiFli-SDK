/**
 * @file dfu_transport_cdc.h
 * @brief DFU V2 CDC (USB) Transport
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_TRANSPORT_CDC_H__
#define __DFU_TRANSPORT_CDC_H__

#include <rtthread.h>
#include "dfu_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize CDC Transport
 *
 * Sets up the internal ring buffer and mailbox reference.
 * Does NOT initialize USB hardware — that is done by cdc_acm_init.c
 * (CherryUSB), which calls feed_raw() in its ISR callback.
 *
 * @param mb  APP layer mailbox handle. CDC ISR will post DFU_MSG_CDC_DATA
 *            (defined in dfu_v2.h) to this mailbox when data arrives.
 * @return 0 on success, negative on failure
 */
int dfu_transport_cdc_init(rt_mailbox_t mb);

/**
 * @brief Deinitialize CDC Transport
 *
 * Resets ring buffer. Does NOT deinit USB hardware.
 */
void dfu_transport_cdc_deinit(void);

/*============================================================================
 * ISR-side: Raw Data Feed (called from cdc_acm_init.c ISR)
 *============================================================================*/

/**
 * @brief Feed raw bytes from USB ISR into transport ring buffer
 *
 * Called by cdc_acm_init.c's usbd_cdc_acm_bulk_out() callback
 * in ISR/callback context. Copies data into ring buffer and posts
 * DFU_MSG_CDC_DATA to the APP mailbox.
 *
 * @param data  Raw bytes from USB bulk OUT endpoint
 * @param len   Number of bytes
 * @return Number of bytes actually buffered (may be < len on overflow)
 */
int dfu_transport_cdc_feed_raw(const uint8_t *data, uint32_t len);

/*============================================================================
 * APP-side: Receive Data Retrieval
 *============================================================================*/

/**
 * @brief Get received data from internal buffer
 *
 * Called by APP main loop after receiving DFU_MSG_CDC_DATA from mailbox.
 *
 * @param[out] buf      Destination buffer (caller-provided)
 * @param[in]  max_len  Maximum bytes to copy
 * @return Positive = actual bytes copied, 0 = empty, Negative = error
 */
int dfu_transport_cdc_get_data(uint8_t *buf, uint32_t max_len);

/*============================================================================
 * Transport Instance
 *============================================================================*/

/**
 * @brief Get the CDC Push Transport instance
 *
 * Returns vtable with send() pointing to dfu_cdc_hw_transmit().
 *
 * @return Push Transport vtable pointer, or NULL if not initialized
 */
const dfu_push_transport_t *dfu_transport_cdc_get_instance(void);

/*============================================================================
 * Hardware Transmit (implemented in cdc_acm_init.c)
 *============================================================================*/

/**
 * @brief Send data to USB host via CherryUSB CDC bulk IN endpoint
 *
 * Weak function — must be overridden by cdc_acm_init.c in the subprogram.
 *
 * @param data  Frame bytes to send
 * @param len   Frame length
 * @return 0 on success, -1 on failure
 */
int dfu_cdc_hw_transmit(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_TRANSPORT_CDC_H__ */
