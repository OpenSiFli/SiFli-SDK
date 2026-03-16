/**
 * @file dfu_v2.h
 * @brief DFU V2 — Public API for OTA Subprogram
 *
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_V2_H__
#define __DFU_V2_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Layer Headers
 *============================================================================*/

#include "engine/dfu_engine_types.h"
#include "engine/dfu_engine.h"
#include "mode/dfu_mode.h"
#include "transport/dfu_transport.h"

/*============================================================================
 * Mailbox Message Codes
 *============================================================================*/

/**
 * @brief Middleware message codes (0x01 ~ 0x3F)
 *
 * Handled by dfu_process(). Transport ISR callbacks post these to
 * the mailbox returned by dfu_get_mailbox().
 *
 * Subprogram-specific messages (e.g. PAN BT events) should use codes
 * >= 0x100 to avoid collision, defined in their own headers.
 */
typedef enum
{
    /* Transport data arrival */
    DFU_MSG_CDC_DATA        = 0x01,     /**< CDC: raw bytes in ring buffer */
    DFU_MSG_BLE_DATA        = 0x02,     /**< BLE: complete message arrived */
    DFU_MSG_UART_DATA       = 0x03,     /**< UART: raw bytes in ring buffer */

    /* Transport connection events */
    DFU_MSG_BLE_DISCONNECTED = 0x10,    /**< BLE: serial channel closed */

    /* User actions */
    DFU_MSG_ABORT           = 0x20,     /**< Abort requested (button/ISR) */

    /* Engine status (for UI observers) */
    DFU_MSG_STATE_CHANGED   = 0x30,     /**< Engine state changed */
    DFU_MSG_PROGRESS        = 0x31,     /**< Engine progress update */
} dfu_msg_t;

/*============================================================================
 * Event Callback
 *============================================================================*/

typedef enum
{
    DFU_EVT_STATE_CHANGED,
    DFU_EVT_PROGRESS,
    DFU_EVT_IMG_COMPLETE,
    DFU_EVT_ALL_COMPLETE,
    DFU_EVT_ERROR,
    DFU_EVT_ABORTED,
} dfu_event_t;

typedef struct
{
    dfu_event_t event;
    union
    {
        struct {
            dfu_engine_state_t old_state;
            dfu_engine_state_t new_state;
            dfu_engine_err_t   error;
        } state;

        struct {
            uint32_t bytes_received;
            uint32_t bytes_total;
            uint8_t  percent;
        } progress;

        struct {
            uint8_t img_id;
        } img_complete;

        struct {
            dfu_engine_err_t error;
        } err;
    };
} dfu_event_param_t;

/**
 * @brief Event callback — called in main loop thread context.
 */
typedef int (*dfu_event_cb_t)(const dfu_event_param_t *param,
                                 void *user_data);

/*============================================================================
 * Configuration
 *============================================================================*/

typedef struct
{
    dfu_event_cb_t   callback;       /**< Event callback (may be NULL) */
    void               *user_data;      /**< Passed to callback */
} dfu_config_t;

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize the DFU V2 subsystem
 *
 * Creates the internal mailbox, initializes Engine, Transport(s), and
 * Mode(s) per Kconfig. Does NOT create any thread.
 *
 * @param cfg  Configuration (callback + user_data)
 * @return 0 on success, negative on failure
 */
int dfu_init(const dfu_config_t *cfg);

/**
 * @brief Deinitialize the DFU V2 subsystem
 */
void dfu_deinit(void);

/*============================================================================
 * Mailbox
 *============================================================================*/

/**
 * @brief Get the DFU mailbox handle
 *
 * Subprogram uses this for rt_mb_recv() in its main loop.
 * Transport callbacks use this to post data-arrival notifications.
 * Scene modules (e.g. pan_app.c) use this to post BT events.
 */
rt_mailbox_t dfu_get_mailbox(void);

/*============================================================================
 * Message Processing
 *============================================================================*/

/**
 * @brief Process one mailbox message
 *
 * Handles middleware-known messages (BLE_DATA, CDC_DATA, ABORT, etc.).
 *
 * @param msg  Message from rt_mb_recv()
 * @return 0 if handled, -1 if not recognized
 */
int dfu_process(rt_ubase_t msg);

/*============================================================================
 * Install Check
 *============================================================================*/

/**
 * @brief Check and execute pending firmware installation
 *
 * If a previous download left a pending install flag, performs the
 * installation and jumps to user firmware (does NOT return).
 *
 * @return 0 if no pending install, negative on error
 */
int dfu_check_install(void);

/*============================================================================
 * Device Mode — Active Download
 *============================================================================*/

typedef struct
{
    const char *url;            /**< Download URL */
    uint32_t    flash_addr;     /**< Target flash address */
    uint32_t    file_size;      /**< Expected file size */
    uint32_t    file_crc;       /**< Expected CRC32 */
    uint32_t    region_size;    /**< Flash region size */
    uint8_t     img_id;         /**< Image type ID */
} dfu_file_t;

typedef struct
{
    const dfu_file_t *files;
    int                  file_count;
} dfu_download_req_t;

/**
 * @brief Execute device-initiated firmware download (blocking)
 *
 * Downloads all files via Pull Transport → Engine → flash.
 * Runs in caller's thread context. Progress callbacks fire here.
 *
 * @param req  Download request
 * @return 0 on success, negative on failure
 */
int dfu_download(const dfu_download_req_t *req);

/**
 * @brief Set install flag for bootloader
 *
 * Call after successful download. Bootloader installs on next reboot.
 *
 * @return 0 on success, negative on failure
 */
int dfu_set_install_flag(void);

/*============================================================================
 * Utility
 *============================================================================*/

/**
 * @brief Request abort (ISR-safe, posts DFU_MSG_ABORT)
 *
 * @return 0 on success, negative on failure
 */
int dfu_abort(void);

/**
 * @brief Query current progress
 *
 * @param received  Bytes received so far
 * @param total     Total firmware size in bytes
 * @return 0 on success, negative on failure
 */
int dfu_get_progress(uint32_t *received, uint32_t *total);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_V2_H__ */