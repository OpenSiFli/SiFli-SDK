/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SWT6621S_WLAN_PORT_H__
#define __SWT6621S_WLAN_PORT_H__
/**
 * @brief Default device name for SWT6621S WiFi module.
 *
 * Used to find/open the WiFi device in the system. Can be overridden
 * by defining `SKW_DEV_NAME` before including this header.
 */
#ifndef SKW_DEV_NAME
    #define SKW_DEV_NAME "wifi_swt6621"
#endif
/**
 * @brief Wakeup output GPIO pin number.
 *
 * This pin outputs a short pulse to signal the WiFi module during
 * sleep/wakeup handshakes.
 */
#ifndef WIFI_WAKEUP_OUT_PIN
    #error "Please define WIFI_WAKEUP_OUT_PIN in your board config"
#endif
/**
 * @brief Wakeup input GPIO pin number.
 *
 * This pin is read to detect the WiFi module's wakeup state. Rising-edge
 * interrupts may be attached to this pin.
 */
#ifndef WIFI_WAKEUP_IN_PIN
    #error "Please define WIFI_WAKEUP_IN_PIN in your board config"
#endif
/**
 * @brief WiFi power control GPIO pin number.
 *
 * This pin controls power to the WiFi module (enable/disable) during
 * suspend/resume sequences.
 */
#ifndef WIFI_POWER_PIN
    #error "Please define WIFI_POWER_PIN in your board config"
#endif
/* Suspend/Resume callback definitions */
/**
 * @brief Suspend callback signature.
 * @param arg User-provided argument passed to the callback (optional).
 */
typedef void (*wifi_suspend_cb_t)(void *arg);
/**
 * @brief Resume callback signature.
 * @param arg User-provided argument passed to the callback (optional).
 */
typedef int (*wifi_resume_cb_t)(void *arg);
/**
 * @brief Detect-sleep callback signature.
 * @param arg User-provided argument passed to the callback (optional).
 */
typedef void (*wifi_detect_slp_cb_t)(void *arg);

/**
 * @brief Firmware code reader callback.
 *
 * @param arg       User context pointer, passed through from wifi_ctl.
 * @param boot_mode Zero-terminated string describing current boot mode
 *                  (for example, selects which firmware image to use).
 * @param lseek     Offset (in bytes) from the beginning of the firmware
 *                  image; the callee should read from this position.
 * @return          0 on success; negative value on failure.
 */
typedef int (*get_fwk_code_cb_t)(void *arg, char *boot_mode, uint32_t lseek);

/**
 * @brief Query maximum firmware size for a given boot mode.
 *
 * @param arg       User context pointer, passed through from wifi_ctl.
 * @param boot_mode Zero-terminated string that identifies a boot mode
 *                  / firmware slot.
 * @return          Positive max size (in bytes) or 0/negative on error.
 */
typedef int (*get_fwk_maxsize_cb_t)(void *arg, char *boot_mode);

/**
 * @brief Callback to get RAM dump output path.
 *
 * @param arg       User context pointer, passed through from wifi_ctl.
 * @param path_buf  Caller-provided buffer where the callee should write
 *                  a zero-terminated filesystem path to store RAM dump.
 * @return          0 on success; negative value on failure.
 */
typedef int (*wifi_get_ram_dump_path_t)(void *arg, char *path_buf);

/**
 * @brief Callback to adjust classic BT AFH channel map for a WiFi channel.
 *
 * @param arg       User context pointer, passed through from wifi_ctl.
 * @param wifi_ch   WiFi channel number (e.g. 1~14 for 2.4G, 36+ for 5G)
 *                  that should be protected from BT traffic.
 * @return          0 on success; negative value on failure.
 */
typedef int (*wifi_set_bt_channel_map_t)(void *arg, int wifi_ch);

/**
 * @brief Callback to adjust BLE channel map for a WiFi channel.
 *
 * @param arg       User context pointer, passed through from wifi_ctl.
 * @param wifi_ch   WiFi channel number around which BLE data channels
 *                  may need to be disabled or de-prioritized.
 * @return          0 on success; negative value on failure.
 */
typedef int (*wifi_set_ble_channel_map_t)(void *arg, int wifi_ch);

/**
 * @brief Container for a suspend callback and its argument.
 */
struct wifi_suspend_cb
{
    /** Callback to execute on suspend. */
    wifi_suspend_cb_t cb;
    /** Opaque user argument passed to the callback. */
    void *arg;
};

/**
 * @brief Container for a resume callback and its argument.
 */
struct wifi_resume_cb
{
    /** Callback to execute on resume. */
    wifi_resume_cb_t cb;
    /** Opaque user argument passed to the callback. */
    void *arg;
};
/**
 * @brief Container for a detect-sleep callback and its argument.
 */
struct wifi_detect_slp_cb
{
    /** Callback to execute when detecting sleep/wakeup handshake. */
    wifi_detect_slp_cb_t cb;
    /** Opaque user argument passed to the callback. */
    void *arg;
};
struct get_fwk_code_cb
{
    /** Callback used to fetch WiFi firmware code chunk by chunk. */
    get_fwk_code_cb_t cb;
    /** Opaque user argument passed to get_fwk_code_cb_t. */
    void *arg;
};
struct get_fwk_maxsize_cb
{
    /** Callback used to query maximum firmware size for a boot mode. */
    get_fwk_maxsize_cb_t cb;
    /** Opaque user argument passed to get_fwk_maxsize_cb_t. */
    void *arg;
};

struct wifi_get_ram_dump_path_cb
{
    /** Callback used to obtain WiFi RAM dump output path. */
    wifi_get_ram_dump_path_t cb;
    /** Opaque user argument passed to wifi_get_ram_dump_path_t. */
    void *arg;
};
struct wifi_set_channel_map_cb
{
    /** Callbacks used to adjust BT/BLE channel maps according to WiFi channel. */
    wifi_set_bt_channel_map_t bt_cb;
    wifi_set_ble_channel_map_t ble_cb;
    /** Opaque user argument passed to both channel-map callbacks. */
    void *arg;
};

enum wifi_device_flag
{
    WIFI_DEVICE_NULL,
    WIFI_DEVICE_INITING,
    WIFI_DEVICE_SCANING,
    WIFI_DEVICE_CONNECTING,
    WIFI_DEVICE_IDLE,
};
enum wifi_sdio_num_flag
{
    WIFI_SDIO_SDHCI1,
    WIFI_SDIO_SDHCI2,
    WIFI_SDIO_LINE,
};
enum wifi_boand_flag
{
    WIFI_BOAND_2G,
    WIFI_BOAND_2G_5G,
};
/**
 * @brief Aggregated WiFi control callbacks.
 *
 * Holds suspend, resume, and detect-sleep callbacks that manage the
 * power and handshake flow of the WiFi module.
 */
struct wifi_ctl
{
    struct wifi_suspend_cb suspend;
    struct wifi_resume_cb resume;
    struct wifi_detect_slp_cb detect_slp;
    struct get_fwk_code_cb get_fwk_code;
    struct get_fwk_maxsize_cb get_fwk_maxsize;
    struct wifi_get_ram_dump_path_cb get_ram_dump_path;
    struct wifi_set_channel_map_cb set_channel_map;
    uint32_t flag_t;
    uint32_t band_2g_5g;
};
/**
 * @brief Initialize the SWT6621S WLAN management layer.
 *
 * Sets up control structures and any required resources for WiFi
 * management. Should be called during application initialization.
 *
 * @return 0 on success; non-zero on failure.
 */
int swt6621s_wlan_mgnt_init(void);

#endif