/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _OTA_UI_H_
#define _OTA_UI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <rtthread.h>

/**
 * @brief UI message types for OTA UI updates
 *
 * Enumeration of all possible message types that can be sent to the OTA UI
 * task to update various UI elements during the OTA process.
 */
typedef enum
{
    UI_MSG_UPDATE_BLE,      /**< Update Bluetooth connection status */
    UI_MSG_UPDATE_NET,      /**< Update network connection status */
    UI_MSG_UPDATE_PROGRESS, /**< Update OTA progress percentage */
    UI_MSG_UPDATE_FILES,    /**< Update current file being processed */
    UI_MSG_UPDATE_PROGRESS_COLOR, /**< Update progress indicator color */
    UI_MSG_UPDATE_BUTTON,         /**< Update button text */
    UI_MSG_SHOW_FAILURE_POPUP,    /**< Show OTA failure popup */
    UI_MSG_SHOW_SUCCESS_POPUP     /**< Show OTA success popup */
} ui_msg_type_t;

/**
 * @brief UI message structure
 *
 * Structure used to pass messages to the OTA UI task containing
 * the message type and associated data string.
 */
typedef struct
{
    ui_msg_type_t type; /**< Type of UI update message */
    char *data; /**< Data associated with the message (can be NULL) */
} ui_msg_t;

// Define color status
#define PROGRESS_COLOR_NORMAL "normal"
#define PROGRESS_COLOR_SUCCESS "success"
#define PROGRESS_COLOR_ERROR "error"
#define UI_MSG_DATA_BLE_CONNECTED "ble"
#define UI_MSG_DATA_BLE_DISCONNECTED "ble_close"
#define UI_MSG_DATA_NET_CONNECTED "net"
#define UI_MSG_DATA_NET_DISCONNECTED "net_close"

/**
 * @brief OTA UI task main thread
 *
 * Initialize LittlevGL graphics library, create UI interface and message
 * queue, and handle UI update messages from other modules. Including
 * Bluetooth connection status, network status, OTA progress updates, etc.
 *
 * @param args Task parameters (not used)
 */
void dfu_pan_ui_task(void *args);

/**
 * @brief Send update message to OTA UI message queue
 *
 * Send UI update messages of specified type to the UI task's message queue,
 * which will be processed by the UI task to update the interface.
 *
 * @param type Message type (such as Bluetooth status update, network status
 * update, progress update, etc.)
 * @param string Message data string pointer
 */
void dfu_pan_ui_update_message(ui_msg_type_t type, char *string);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_UI_H_ */
