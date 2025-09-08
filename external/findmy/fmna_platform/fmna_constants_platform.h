/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_constants_platform_h
#define fmna_constants_platform_h

#include "bf0_ble_gap.h"
#include "ble_connection_manager.h"
#include "fmna_config.h"

#define LOG_TAG "FMNA"
#include "log.h"

//#define FMNA_MANUFACTURER_NAME                  "Acme"
//#define FMNA_MODEL_NAME                         "nRF52 DK PCA10040"

#define FMNA_MANUFACTURER_NAME                  "SiFli Technologies (Nanjing) Co., Ltd."
#define FMNA_MODEL_NAME                         "iGS800"

#define FMNA_PID                                0xCAFE
#define FMNA_HARDWARE_VERSION                   "1"
#define FMNA_HARDWARE_VERSION                   "1"

#define BLE_GAP_SEC_KEY_LEN                     GAP_KEY_LEN

#define FMNA_LOG_ERROR(...)                     LOG_E(__VA_ARGS__)
#define FMNA_LOG_WARNING(...)                   LOG_W(__VA_ARGS__)
#define FMNA_LOG_INFO(...)                      LOG_I(__VA_ARGS__)
#define FMNA_LOG_DEBUG(...)                     LOG_D(__VA_ARGS__)

#define BLE_CONN_HANDLE_INVALID 0xFFFF
#define BLE_CONN_HANDLE_ALL 0xFFFF

#define FMNA_LOG_HEXDUMP_INFO(p_data, len)      LOG_HEX("fmna-hex", 16, (uint8_t *)p_data, len)
#define FMNA_LOG_HEXDUMP_DEBUG(p_data, len)     LOG_HEX("fmna-hex", 16, (uint8_t *)p_data, len)

#define MAX_SUPPORTED_CONNECTIONS               MAX_CONNECTION_LINK_NUM
#define GATT_MAX_MTU_SIZE                       NRF_SDH_BLE_GATT_MAX_MTU_SIZE

#define CONN_HANDLE_INVALID                     BLE_CONN_HANDLE_INVALID
#define CONN_HANDLE_ALL                         BLE_CONN_HANDLE_ALL
#define GAP_SEC_KEY_LEN                         BLE_GAP_SEC_KEY_LEN

typedef enum
{
    SF_FMNA_SUCCESS = 0,
    SF_FMNA_ERROR_INTERNAL,
    SF_FMNA_ERROR_INVALID_STATE,
    SF_FMNA_ERROR_INVALID_LENGTH,
    SF_FMNA_ERROR_INVALID_DATA,
    SF_FMNA_ERROR_NULL
} SF_FMNA_error_t;


#define FMNA_SUCCESS                            SF_FMNA_SUCCESS               ///< Successful command
#define FMNA_ERROR_INTERNAL                     SF_FMNA_ERROR_INTERNAL        ///< Internal Error
#define FMNA_ERROR_INVALID_STATE                SF_FMNA_ERROR_INVALID_STATE   ///< Invalid state, operation disallowed in this state
#define FMNA_ERROR_INVALID_LENGTH               SF_FMNA_ERROR_INVALID_LENGTH  ///< Invalid Length
#define FMNA_ERROR_INVALID_DATA                 SF_FMNA_ERROR_INVALID_DATA    ///< Invalid Data
#define FMNA_ERROR_NULL                         SF_FMNA_ERROR_NULL            ///< Null Pointer

#define APP_ERROR_CHECK(ERR_CODE) { \
    if (ERR_CODE != 0) { \
        LOG_I("ERROR CHECK %d", ERR_CODE); \
    } \
}

#define APP_ERROR_CHECK_BOOL(ERR_CODE) { \
    if (ERR_CODE == 0) { \
         LOG_I("ERROR CHECK BOOL %d", ERR_CODE); \
    } \
}

#define FMNA_ERROR_CHECK(ERR_CODE)              APP_ERROR_CHECK(ERR_CODE)

// Macro for calling error handler function if supplied boolean value is false.
#define FMNA_ERROR_CHECK_BOOL(ERR_CODE)         APP_ERROR_CHECK_BOOL(ERR_CODE)

// #define APP_TIMER_TICKS do { } while(0)

#define APP_TIMER_TICKS(ms) (rt_tick_from_millisecond(ms))

#define MSEC_TO_TIMER_TICKS(ms)                     APP_TIMER_TICKS(ms)

//MARK: Nordic SDK required macros.
#define SEC_PARAM_BOND                          1                         /**< Perform bonding. */
#define SEC_PARAM_MITM                          0                         /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                          0                         /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                      0                         /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES               BLE_GAP_IO_CAPS_NONE      /**< No I/O capabilities. */
#define SEC_PARAM_OOB                           0                         /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE                  7                         /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE                  16                        /**< Maximum encryption key size in octets. */

/** Priority of the application BLE event handler. */
#define FMNA_BLE_CONN_CFG_TAG                    1                         /**< A tag identifying the SoftDevice BLE configuration. */
#define FMNA_BLE_OBSERVER_PRIO                   3

#define ret_code_t                              uint8_t
#define fmna_ret_code_t                         ret_code_t


#define ADV_TYPE_SERVICE_DATA                   BLE_GAP_AD_TYPE_SERVICE_DATA

#define APP_TIMER_DEF         OS_TIMER_DECLAR


#define PRODUCT_DATA_VAL_TEMP                             {0x4C, 0x24, 0xC4, 0xDF, 0x77, 0x95, 0xB0, 0x14}

#define ACCESSORY_CATEGORY_FINDER                           1

#define MANA_TEMP  {0x57, 0x65, 0x73, 0x6f, 0x6c, 0x76, 0x65, 0x74}
#define MODEL_NAMLE_TEMP {0x4e, 0x20, 0x54, 0x61, 0x67}

#define FMNA_MANUFACTURER_NAME_SIFLI                  "SiFli"
#define FMNA_MODEL_NAME_SIFLI                         "SF32-56X"


#endif /* fmna_constants_platform_h */
