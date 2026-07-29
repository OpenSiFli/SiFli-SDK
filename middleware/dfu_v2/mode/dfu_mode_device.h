/**
 * @file dfu_mode_device.h
 * @brief DFU V2 Device-Initiated Mode Interface
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_MODE_DEVICE_H__
#define __DFU_MODE_DEVICE_H__

#include "dfu_mode.h"
#include "../transport/dfu_transport.h"
#include "../engine/dfu_engine_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize Device-Initiated Mode
 *
 * Binds a Pull Transport for data retrieval.
 *
 * @param transport  Pull Transport vtable (e.g. PAN HTTP). Must not be NULL.
 * @param callback   Event callback for APP layer (may be NULL)
 * @param user_data  Context for callback
 * @return 0 on success, negative on failure
 */
int dfu_mode_device_init(const dfu_pull_transport_t *transport,
                         dfu_mode_callback_t callback,
                         void *user_data);

/**
 * @brief Deinitialize Device-Initiated Mode
 *
 * If upgrade is running, calls dfu_engine_abort() and closes Transport.
 */
void dfu_mode_device_deinit(void);

/*============================================================================
 * Upgrade Control
 *============================================================================*/

/**
 * @brief Start device-initiated upgrade
 *
 * Launches the complete upgrade flow in the calling thread context:
 *   For each image:
 *     1. engine->begin_session()  — check resume
 *     2. engine->erase()          — erase flash (or skip if resume)
 *     3. loop: transport->read() + engine->write_data()
 *     4. engine->verify()         — CRC + optional signature
 *     5. engine->commit()         — finalize
 *
 * Progress and completion are reported via the registered callback.
 * If engine->abort() is called during execution (e.g. from another
 * thread via MSG_ABORT), the download loop detects the ABORTED error
 * on the next write_data() return and stops.
 *
 * @param file_info  Firmware file descriptor (from server query or APP)
 * @param url        Download URL (passed to transport->open())
 *
 * @return 0 if upgrade completed successfully
 *         negative on failure (error details via callback EVT_ERROR)
 *
 * @note This function is BLOCKING — it runs the entire download loop
 *       in the caller's context. Typical usage: APP creates a dedicated
 *       thread for device-initiated upgrade and calls this from there,
 *       or posts DFU_MSG_PAN_START to the main loop which then calls it.
 */
int dfu_mode_device_start(const dfu_file_info_t *file_info,
                          const char *url);

/*============================================================================
 * Status Query
 *============================================================================*/

/**
 * @brief Check if Device Mode is currently running an upgrade
 *
 * @return true if upgrade is in progress, false if idle
 */
bool dfu_mode_device_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_MODE_DEVICE_H__ */