/*
 * @file dfu_ui.h
 * @brief DFU V2 OTA UI Module Header
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
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
 */
typedef enum
{
    UI_MSG_UPDATE_BLE,            /**< Update Bluetooth status (kept for compat) */
    UI_MSG_UPDATE_NET,            /**< Update network status   (kept for compat) */
    UI_MSG_UPDATE_USB,            /**< Update USB connection status */
    UI_MSG_UPDATE_PROGRESS,       /**< Update OTA progress percentage ("0"-"100") */
    UI_MSG_UPDATE_FILES,          /**< Update status text string */
    UI_MSG_UPDATE_PROGRESS_COLOR, /**< Update progress ring color */
    UI_MSG_UPDATE_BUTTON,         /**< (reserved, unused) */
    UI_MSG_UPDATE_SOURCE,         /**< Update DFU source indicator (PAN/USB) */
    UI_MSG_SHOW_FAILURE_POPUP,    /**< Show failure state */
    UI_MSG_SHOW_SUCCESS_POPUP     /**< Show success state */
} ui_msg_type_t;

/**
 * @brief UI message structure
 */
typedef struct
{
    ui_msg_type_t type;
    char *data;
} ui_msg_t;

/* Progress ring color tokens */
#define PROGRESS_COLOR_NORMAL  "normal"
#define PROGRESS_COLOR_SUCCESS "success"
#define PROGRESS_COLOR_ERROR   "error"

/* Bluetooth status (kept for compat, UI ignores) */
#define UI_MSG_DATA_BLE_CONNECTED    "ble"
#define UI_MSG_DATA_BLE_DISCONNECTED "ble_close"

/* Network status (kept for compat, UI ignores) */
#define UI_MSG_DATA_NET_CONNECTED    "net"
#define UI_MSG_DATA_NET_DISCONNECTED "net_close"

/* USB status */
#define UI_MSG_DATA_USB_CONNECTED    "usb"
#define UI_MSG_DATA_USB_DISCONNECTED "usb_close"

/* DFU source indicators */
#define UI_MSG_DATA_SOURCE_NONE "none"
#define UI_MSG_DATA_SOURCE_PAN  "pan"
#define UI_MSG_DATA_SOURCE_USB  "usb"

/**
 * @brief OTA UI task entry — runs LVGL event loop
 * @param args unused
 */
void dfu_ui_task(void *args);

/**
 * @brief Send a UI update message (thread-safe, non-blocking)
 * @param type  message type
 * @param string  payload string (will be rt_strdup'd), may be NULL
 */
void dfu_ui_update_message(ui_msg_type_t type, char *string);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_UI_H_ */
