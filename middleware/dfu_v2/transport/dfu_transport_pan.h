/**
 * @file dfu_transport_pan.h
 * @brief DFU V2 PAN HTTP Transport
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_TRANSPORT_PAN_H__
#define __DFU_TRANSPORT_PAN_H__

#include "dfu_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize PAN HTTP Transport
 *
 * No external resources needed at init time — the HTTP session is
 * created lazily when open() is called.
 *
 * @return 0 on success, negative on failure
 */
int dfu_transport_pan_init(void);

/**
 * @brief Deinitialize PAN HTTP Transport
 *
 * Closes any active HTTP session if present.
 */
void dfu_transport_pan_deinit(void);

/*============================================================================
 * Transport Instance
 *============================================================================*/

/**
 * @brief Get the PAN Pull Transport instance
 *
 * Returns a pointer to the static dfu_pull_transport_t vtable.
 *
 * Typical usage:
 *   const dfu_pull_transport_t *pan = dfu_transport_pan_get_instance();
 *   dfu_mode_device_init(pan, callback, user_data);
 *
 * @return Pull Transport vtable pointer, or NULL if not initialized
 */
const dfu_pull_transport_t *dfu_transport_pan_get_instance(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_TRANSPORT_PAN_H__ */
