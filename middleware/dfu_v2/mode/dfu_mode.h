/**
 * @file dfu_mode.h
 * @brief DFU V2 Mode — Common Types
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_MODE_H__
#define __DFU_MODE_H__

#include <stdint.h>
#include <stdbool.h>
#include "../engine/dfu_engine_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Mode Event Types
 *============================================================================*/

/**
 * @brief Events reported by Mode layer to APP layer
 *
 * Mode layer uses a callback to notify APP of high-level upgrade events.
 * These are Mode-level events, NOT Engine state transitions — Mode
 * translates Engine states into meaningful lifecycle events.
 */
typedef enum
{
    DFU_MODE_EVT_STARTED = 0,       /**< Upgrade session started */
    DFU_MODE_EVT_PROGRESS,          /**< Progress update (periodic) */
    DFU_MODE_EVT_IMG_COMPLETE,      /**< One image completed (multi-image) */
    DFU_MODE_EVT_ALL_COMPLETE,      /**< All images done, upgrade successful */
    DFU_MODE_EVT_ERROR,             /**< Error occurred, upgrade failed */
    DFU_MODE_EVT_ABORTED,           /**< Upgrade aborted by user */
} dfu_mode_event_t;

/*============================================================================
 * Mode Event Parameter
 *============================================================================*/

/**
 * @brief Event payload passed to Mode callback
 */
typedef struct
{
    dfu_mode_event_t    event;      /**< Event type */
    dfu_engine_err_t    error;      /**< Error code (valid for EVT_ERROR) */
    dfu_progress_t      progress;   /**< Progress (valid for EVT_PROGRESS) */
    uint8_t             img_id;     /**< Image ID (valid for EVT_IMG_COMPLETE) */
} dfu_mode_event_param_t;

/*============================================================================
 * Mode Callback
 *============================================================================*/

/**
 * @brief Mode event callback type
 *
 * Registered at Mode init. Called in the Mode execution context
 * (APP main loop thread). Safe to do rt_mb_send, LOG, set flags.
 *
 * @param param      Event parameters
 * @param user_data  User context pointer set at init time
 */
typedef void (*dfu_mode_callback_t)(const dfu_mode_event_param_t *param,
                                    void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_MODE_H__ */