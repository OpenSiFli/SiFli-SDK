/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __COMPOSITE_DEVICE_H__
#define __COMPOSITE_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* USB Device IDs */
#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/* HID Modifier Keys */
#define HID_MODIFIER_LCTRL  0x01
#define HID_MODIFIER_LSHIFT 0x02
#define HID_MODIFIER_LALT   0x04
#define HID_MODIFIER_LGUI   0x08
#define HID_MODIFIER_RCTRL  0x10
#define HID_MODIFIER_RSHIFT 0x20
#define HID_MODIFIER_RALT   0x40
#define HID_MODIFIER_RGUI   0x80

/* Function declarations */

/**
 * @brief Initialize USB composite device
 * 
 * This function initializes a USB composite device with MSC, CDC ACM, and HID interfaces.
 * The device will appear as:
 * - Mass Storage Class (MSC): Virtual disk (RAM-based, 8KB)
 * - CDC ACM: Virtual COM port for serial communication
 * - HID: Virtual keyboard for sending keystrokes
 * 
 * @param busid USB bus ID (typically 0)
 * @param reg_base USB controller register base address
 */
void composite_device_init(uint8_t busid, uint32_t reg_base);

/**
 * @brief Check if USB composite device is ready
 * 
 * This function checks if the USB composite device has been initialized
 * and is ready to send data.
 * 
 * @param busid USB bus ID (typically 0)
 * @return true if device is ready, false otherwise
 */
bool composite_device_is_ready(uint8_t busid);

/**
 * @brief Send a string via HID keyboard interface
 * 
 * This function sends a text string by simulating keyboard keystrokes.
 * Each character is sent as a key press followed by key release.
 * 
 * @param busid USB bus ID (typically 0)
 * @param str Null-terminated string to send via HID keyboard
 * 
 * @note The function will block until all characters are sent.
 *       There's a small delay between characters for proper recognition.
 *       Make sure to call composite_device_is_ready() first to check if device is initialized.
 */
void hid_keyboard_send_string(uint8_t busid, const char* str);

/**
 * @brief Send a single character via HID keyboard interface
 * 
 * This function sends a single character by simulating keyboard keystroke.
 * Handles both regular characters and those requiring modifier keys (like uppercase).
 * 
 * @param busid USB bus ID (typically 0)
 * @param ch Character to send via HID keyboard
 * 
 * @note The function will block until the character is sent.
 */
void hid_keyboard_send_char(uint8_t busid, char ch);

#ifdef __cplusplus
}
#endif

#endif /* __COMPOSITE_DEVICE_H__ */