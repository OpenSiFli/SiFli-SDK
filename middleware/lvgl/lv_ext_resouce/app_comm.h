/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   app_comm.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _APP_COMM_H_
#define _APP_COMM_H_
#include <stdbool.h>
#include "data_service.h"
#include "string.h"
#include "service_comm.h"
#include "rtdevice.h"
#include "lv_ext_resource_manager.h"

#define LCD_DEVICE_NAME             "lcd"
#define OTA_ERR_FONT                "ota_err"

#ifndef FACTORY_CFG_ID_OTA
    #define FACTORY_CFG_ID_OTA          0xFE
#endif

#define WATCH_DEVICE_NAME           "SIFLI"
#define CPU_MODEL                   "SF32LB551"
#define PRODUCT_SOFTWARE_VER        "1.0.0"

enum
{
    BACKLIGHT_ONE_LEVEL = 1,        /**< LCD brightness level 1            */
    BACKLIGHT_TWO_LEVEL,            /**< LCD brightness level 2            */
    BACKLIGHT_THREE_LEVEL,          /**< LCD brightness level 3            */
    BACKLIGHT_FOUR_LEVEL,           /**< LCD brightness level 4            */
    BACKLIGHT_FIVE_LEVEL,           /**< LCD brightness level 5            */
    BACKLIGHT_SIX_LEVEL,            /**< LCD brightness level 6            */
    BACKLIGHT_SEVEN_LEVEL,          /**< LCD brightness level 7            */
    BACKLIGHT_EIGHT_LEVEL,          /**< LCD brightness level 8            */
    BACKLIGHT_NINE_LEVEL,           /**< LCD brightness level 9            */
    BACKLIGHT_TEN_LEVEL,            /**< LCD brightness level 10            */
};

#define BACKLIGHT_LEVEL_MAX         (BACKLIGHT_TEN_LEVEL)
#define BACKLIGHT_LEVEL_MIN         (BACKLIGHT_ONE_LEVEL)
#define BACKLIGHT_MANUAL_TIMEOUT    (10*60*1000)

#define OTA_BLE_MASK            0x1000
#define OTA_TF_MASK             0x2000
#define OTA_PAN_MASK            0x4000

typedef struct
{
    uint8_t device_name[20];
#ifdef BSP_BLE_SIBLES
    uint8_t device_mac[18];
#endif
    uint8_t device_cpu[12];
    uint8_t sw_version[18];
    uint8_t build_date[20];
} device_info_t;

typedef struct
{
    uint32_t    idle_time_limit;    /**< If  input device is idle for a duration idle_time_limit(ms), GUI will enter sleep */
    /*whether screen lock is paused*/
    bool        enable;             /**< enable lock screen or not*/
} screen_lock_ctrl_t;

typedef struct
{
    uint32_t    manual_adj_tick;    /**< manually adjust lcd light tick*/
    bool        enable;             /**< enable auto adjust light or not*/
} screen_light_ctrl_t;

//app_get_strxxx, used for multi_language.
#define STR_NULL                                        0
#define app_get_strid(key_name, name)                   LV_EXT_STR_ID(key_name)
#define app_get_str(key_name, name)                     LV_EXT_STR_GET(LV_EXT_STR_ID(key_name))
#define app_get_str_from_id(key_id)                     LV_EXT_STR_GET(key_id)
#define app_get_str_from_lang_pack(lang_pack, key_name) LV_EXT_STR_GET_BY_KEY_LANGPACK(lang_pack, LV_EXT_STR_ID(key_name))

/**
  * @brief  Turn off LCD, LCD power off.
  */
void        app_lcd_off(void);

/**
  * @brief  Turn on LCD, LCD power on.
  */
void        app_lcd_on(void);

/**
 * @brief  Notify LCPU, the HCPU has been powered on.
           In dual core (non 52x), due to the need for low battery detection on the HCPU during power on,
           the battery measurement of the LCPU is opened here..
 */
void        app_lcpu_pwr_on(void);

/**
 * @brief  Notify LCPU, the HCPU is about to shutdown.
 * @param  type Interface of type app_lcpu_t.
 */
void        app_lcpu_pwr_off(uint16_t type);

/**
 * @brief  After notify LCPU when the HCPU is about to shutdown, waiting for LCPU response.
 */
void        app_lcpu_pwr_off_cb(void);


/**
 * @brief  Obtain relevant information about the device
 * @param  device_info Pointer to device information
 */
void        app_device_info_get(device_info_t *device_info);

/**
 * @brief  Determine if the lock screen time has timed out
 * @retval bool Timeout or not
 */
bool        app_screen_lock_time_is_end(void);

/**
 * @brief  Set whether to enable the lock screen function
 */
void        app_screen_lock_enable(bool enable);

/**
 * @brief  Set lock screen time length and store it in NVM.
 * @param  ms Lock screen time length, unit ms
 * @retval uint32_t Previous lock screen time length before setting
 */
uint32_t    app_screen_lock_time_set(uint32_t ms);

/**
 * @brief  Temporarily set lock screen time length, not store it in NVM.
 * @param  ms Lock screen time length, unit ms
 * @retval uint32_t Previous lock screen time length before setting
 */
uint32_t    app_screen_lock_time_temp_set(uint32_t ms);

/**
 * @brief  Update system time.
 * @param  time New system time
 */
void        app_sys_time_update(sys_time_t *time);

/**
 * @brief  Reset the system clock, the system clock will be restored to its default state.
 */
void        app_sys_time_reset(void);

/**
 * @brief  Update system time using time_t's time stamp.
 * @param  timestamp New system time
 */
void        app_sys_timestamp_update(time_t timestamp);

/**
 * @brief  Open hindi(Devanagari) with font name Devanagari _hindi_name.
           HINDI_LANG_PREV_CONVERT_SUPPORT: Static text uses pre conversion to improve display speed
 */
void        app_hindi_font_active(void);

/**
 * @brief  close hindi(Devanagari) font.
 */
void        app_hindi_font_deactive(void);

/**
 * @brief  Clean TP buffer in driver.
 */
void        app_tp_buffer_clean(void);

/**
 * @brief  Enable TP wakeup function or not.
           Due to tp pin maybe placed in LCPU, so need send to LCPU.
 * @param  enable Enable or not.
 */
void        app_tp_wakeup_enable(uint8_t enable);

/**
 * @brief  Clean frame buffer.
 */
void        app_frame_buffer_clean(void);

/**
 * @brief  Initalize local language.
           1) Get language list(ex_lang) from flash's language packet if exist.
           2) Set default language.
 */
void        app_locale_lang_init(void);

/**
 * @brief  Updated and set local language.
 * @param  locale Locale language name.
 */
void        app_locale_lang_update(const char *locale);

/**
 * @brief  Set LCD display brightness.
 * @param  level Brightness level.
 */
void        app_lcd_backlight_set(uint8_t level);

/**
 * @brief  Get the name of currently running application name.
 * @retval string Currently running application name
 */
char       *app_current_app_name_get(void);

/**
 * @brief  Enable watchdog or not.
 * @param  enable Enable watchdog or not.
 */
void        app_watchdog_enable(uint8_t enable);

/**
 * @brief  Pet watchdog to avoid watchdog timeout.
 */
void        app_watchdog_pet(void);

/**
 * @brief  Set OTA status and save it.
 * @param  state OTA status.
 * @retval rt_err_t Set status result
 */
rt_err_t    app_ota_status_set(uint16_t state);

/**
 * @brief  Get OTA status.
 * @retval uint16_t Current OTA state.
 */
uint16_t    app_ota_status_get(void);

/**
 * @brief  Get version mode, release mode or debug mode.
           For release mode, some exception can't be asserted
           For debug mode, watchdog maybe close.
 * @retval int 0: debug mode; 1: release mode.
 */
int         app_version_is_release_mode(void);

/**
 * @brief  Set version mode.
 * @param  mode Version mode, 0: debug mode; 1: release mode.
 */
void        app_version_release_mode_set(bool mode);

/**
 * @brief  Waiting GPU done.
           It will be called when power_off animation procedure.
 */
void        app_gpu_wait_done(void);

/**
 * @brief  Convert time to format string. hour : minute: second with format %02d:%02d:%02d
 * @retval string time string with %02d:%02d:%02d (hour : minute: second).
 */
char       *app_convert_time_formats_str(bool need_sec);

/**
 * @brief  Initialize font, deal ota abnoraml.
 */
void        app_font_open(void);

/**
 * @brief  check file is valid or not.
 */
bool app_path_check_valid(const char *path);

/**
 * @brief  check path is valid or not.
 */
bool app_file_check_valid(const char *file);

/**
 * @brief  check path is sd card or not.
 */
bool app_path_check_is_sd(const char *path);

/**
 * @brief   Get the value of the O_DIRECTORY macro (adapted to DFS environment)
 * @details This function encapsulates the logic for obtaining O_DIRECTORY.
 *          It returns the actual value of the macro only when the DFS is enabled,
 *          and returns 0 when DFS is disabled. Meanwhile, it avoids the problem of inconsistent
 *          O_DIRECTORY values between GCC and KEIL compilers.
 * @return  int     Return value description
 *          - O_DIRECTORY: RT_USING_DFS is defined (DFS enabled), returns the actual value of the macro
 *          - 0: RT_USING_DFS is not defined (DFS disabled), returns the default value
 */
int app_get_o_directory(void);

/**
 * @brief  Delete qjs application(aod/app/wf).
 * @param  id id to delete.
 * @retval 0 : success, others: failed
 */
int app_qjs_del_app(const char *id);

/**
 * @brief  Delete micropython application(aod/app/wf).
 * @param  id id to delete.
 * @retval 0 : success, others: failed
 */
int app_mpy_del_app(const char *id);

/**
 * @brief  Retrieve the registered power_on application. This application can be set through app_set_reg_power_on_app or obtained from NVM.
 * @retval The name of power on application
 */
const char *app_get_reg_power_on_app(void);

/**
 * @brief  Set the power on application through this interface and immediately store it in the NVM.
 *         This application will serve as the first application to be entered after the device power on.
 */
void app_set_reg_power_on_app(const char *main_app);

/**
 * @brief  Retrieve the registered main application. This application can be set through app_set_reg_main or obtained from NVM.
 * @retval The name of main application
 */
const char *app_get_reg_main_app(void);

/**
 * @brief  Set the main application through this interface and immediately store it in the NVM.
 *         Pressing the home key will return to this application.
 */
void app_set_reg_main_app(const char *main_app);

/**
 * @brief  Retrieve the registered tlv application. This application can be set through app_set_reg_tlv or obtained from NVM.
 * @retval The name of tlv application
 */
const char *app_get_reg_tlv_app(void);

/**
 * @brief  Set the tlv application through this interface and immediately store it in the NVM.
 *         When Gui is in the main app, pressing the home key will enterto this application.
 */
void app_set_reg_tlv_app(const char *tlv_app);

/**
 * @brief  Retrieve the first app that launches on startup.
 *         First, try to obtain the application stored in NVM.
 *         If it is unsuccessful and there is no built-in application,
 *         then try to obtain an external application as the first application to start up.
 */
void app_get_poweron_app(void);

/**
 * @brief  Check app is built_in_app by thumbnail
 * @param  thumb thumbnail or icon for application
 * @retval true: built in, false: non built in
 */
bool app_is_built_in(const char *thumb);

/**
 * @brief  Close all external fonts and switch the fonts of all texts back to the initial state.
 */
void app_text_font_reload(void);

/**
 * @brief  Update the font of the specified application and page.
 *         If both the application name and the page name are null,
 *         then update all existing texts.
 * @param  app_id application's name
 * @param  page_id subpage's name
 * @param  new_font the specified font, if it is null, nothing will be done.
 */
void app_text_font_update(const char *app_id, const char *page_id, const char *new_font);

/**
 * @brief  This function is used to remove all occurrences of the substring 'sub' from the string 'src'.
 * @param  src source string
 * @param  sub the string to be removed.
 */
void app_remove_substr(char *src, const char *sub);

/**
 * @brief  This function is used to make dir level by level.
 * @param  path dir to create
 * @param  RT_EOK: success, othewise failed
 */
int app_mkdir(const char *path);


/**
 * @brief  Format partion
 * @param  device_name partion name
 * @param  0 : success, oterwise fail
 */
int app_mkfs_partition(const char *device_name);

/**
 * @brief Delete all external application except the built-in
 * @retval None
 */
void app_del_ext_all(void);

/**
 * @brief Get backlight level by lux
 * @param  lux lux to convert
 * @param  level backlight level
 */
uint8_t app_get_backlight_level_by_lux(uint16_t lux);

/**
 * @brief Record the time of manual brightness adjustment.
          If the current time is greater than the manual brightness adjustment timeout threshold,
          the brightness will be adjusted automatically.
 * @param manual_tick manually adjust ticj
 */
void app_screen_light_adjust_set(uint32_t manual_tick);

/**
 * @brief  Set screen light auto ajust
 * @param  enable Enable watchdog or not.
 */
void app_screen_light_auto_set(uint8_t enable);

/**
 * @brief  Determine if the screen light manual adjust time has timed out
 * @retval bool Timeout or not
 */
bool app_screen_light_adjust_is_timeout(void);

/**
 * @brief  To determine whether solution is builtin_res
 * @retval 0 : success, others: failed
 */
bool solution_is_builtin_res(void);

size_t app_get_file_size(const char *file);

/**
 * @brief Get file open mode from string
 * @retval  O flag to open file
 */
int app_get_file_open_mode(const char *mode);

/**
 * @brief Get active netdev name
 * @retval active netdev name
 */
const char *app_get_active_netdev_name(void);

/**
 * @brief Get netdev is internet up
 * @param name netdev name
 * @retval true for internet up, false for internet down
 */
bool app_get_netdev_internet_up(const char *name);

#endif
