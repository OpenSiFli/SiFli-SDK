/**
  ******************************************************************************
  * @file   bf0_sibles_advertising.h
  * @author Sifli software development team
  * @brief Header file - Sibles advertising APIs.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#ifndef BF0_SIBLES_ADVERTISING_H_
#define BF0_SIBLES_ADVERTISING_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_ble_common.h"
#include "bf0_ble_gap.h"

/** @defgroup sibles_advertising Sibles advertising
  * @ingroup sibles_service
  * @{
  */

/**
 * @brief Declar sibles advertising context.
 */
#define SIBLES_ADVERTISING_CONTEXT_DECLAR(context) \
            sibles_advertising_context_t _##context; \
            sibles_advertising_context_t * context = &_##context; \
            static int sibles_advertising_evt_handler##_##context(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context) \
            { \
                return sibles_advertising_evt_handler(event_id, data, len, context); \
            } \
            BLE_EVENT_REGISTER(sibles_advertising_evt_handler##_##context, &_##context);


/**
 * @brief Advertising mode.
 */
enum sibles_adv_mode_t
{
    SIBLES_ADV_CONNECT_MODE,                 /**< Connected mode use connectable and scannable undirected advertising
                                                  to expect a peer device to connect .*/
    SIBLES_ADV_BROADCAST_MODE,               /**< Broadcast mode use non-connectable and non-scannable undirected advertising
                                                  to broadcast data. */
    SIBLES_ADV_DIRECTED_CONNECT_MODE,        /**< Directed Connect mode use connectable directed advertising to expect
                                                  the directed peer device to connect. */
    SIBLES_ADV_EXTENDED_MODE,                /**< BLE advertising extension use connectable or scannable advertising to
                                                  broadcast up to 251 bytes*/
    SIBLES_ADV_PERIODIC_MODE,                /**< periodic advertising*/
};

/**
 * @brief Advertising running mode.
 */
enum sibles_adv_running_mode_t
{
    SIBLES_ADV_MODE_FAST_CONNECT,            /**< Normal connectable advertising in SIBLES_ADV_CONNECT_MODE. */
    SIBLES_ADV_MODE_BACKGROUD_CONNECT,       /**< Bakcground connectable advertising in SIBLES_ADV_CONNECT_MODE. */
    SIBLES_ADV_MODE_BROADCAST,               /**< Non- connectable advertising in SIBLES_ADV_BROADCAST_MODE. */
    SIBLES_ADV_MODE_HIGH_DUTY_DIRECTED,      /**< High duty cycle directed advertising in SIBLES_ADV_DIRECTED_CONNECT_MODE. */
    SIBLES_ADV_MODE_LOW_DUTY_DIRECTED,       /**< Low duty cycle advertising in SIBLES_ADV_DIRECTED_CONNECT_MODE. */
    SIBLES_ADV_MODE_EXTENDED,                /**< extended advertising in SIBLES_ADV_EXTENDED_MODE. */
    SIBLES_ADV_MODE_PERIODIC,                /**< periodic advertising in SIBLES_ADV_PERIODIC_MODE. */
};



/**
 * @brief Advertising error code.
 */
enum sibles_adv_err_t
{
    SIBLES_ADV_NO_ERR,                       /**< No error. */
    SIBLES_ADV_NOT_ALLOWED,                  /**< Action is not allowed due to state wrongly. */
    SIBLES_ADV_PARAM_INVALID,                /**< Parameter error. */
    SIBLES_ADV_DATA_LENGTH_EXCEED,           /**< Data length exceed maximum value. */
};



/**
 * @brief Advertising event.
 */
enum sibles_adv_event_t
{
    SIBLES_ADV_EVT_ADV_STARTED,               /**< Advertising started. */
    SIBLES_ADV_EVT_ADV_STOPPED,               /**< Advertising stopped. */
    SIBLES_ADV_EVT_REQUEST_PEER_DEVICE_ADDR,  /**< Request user provide peer device addr for directed advertising. */
    SIBLES_ADV_EVT_REQUEST_SET_WHITE_LIST,    /**< Request user set while list for advertising filter policy. No data
                                                   as input parameter */
};



/**
 * @brief Configuration of connectable advertising.
 */
typedef struct
{
    uint32_t duration;                       /**< Advertising duration(in unit of 10ms).0 indicates infinte till
                                                  disabled it */
    uint32_t interval;                       /**< Advertising interval(in unit of 625us). Must be greater than 20ms. */
    uint8_t  backgroud_mode_enabled;         /**< Enabled backgroud mode, advertising will alternate between normal and
                                                  background . */
    uint32_t backgroud_duration;             /**< Backgroud advertising duration(in unit of 10ms). */
    uint32_t backgroud_interval;             /**< Backgroud advertising interval(in unit of 625us). */
    uint32_t is_repeated;                    /**< Whether re-start advertising when duration comes to an end. If
                                                  backgroud mode enabled, will repate after an alternation. */
} sibles_adv_connect_mode_config_t;

/**
 * @brief Configuration of non-connectable advertising.
 */
typedef struct
{
    uint32_t duration;                       /**< Advertising duration(in unit of 10ms). */
    uint32_t interval;                       /**< Advertising interval(in unit of 625us). Must be greater than 20ms. */
    uint8_t scannable_enable;                /**< some android phone only find device enable scannable if adv in broadcast mode. */
} sibles_adv_broadcast_mode_config_t;

/**
 * @brief Configuration of directed connectable advertising.
 */
typedef struct
{
    uint8_t high_duty_cycle_enabled;         /**< High duty cycle enabled, the duration and interval will not use. */
    uint32_t duration;                       /**< Advertising duration(in unit of 10ms). */
    uint32_t interval;                       /**< Advertising interval(in unit of 625us). Must be greater than 20ms. */
} sibles_adv_directed_connect_mode_config_t;

typedef struct
{
    uint32_t duration;                       /**< Advertising duration(in unit of 10ms). */
    uint32_t interval;                       /**< Advertising interval(in unit of 625us). Must be greater than 20ms. */
    uint8_t max_skip;                        /**< Maximum number of advertising events the controller can skip before sending the */
    uint8_t phy;                             /**< Indicate on which PHY secondary advertising has to be performed (@see enum gapm_phy_type) */
    uint8_t adv_sid;                         /**< Advertising SID */
    uint8_t connectable_enable;              /**< For extended adv, shall not be both connectable and scannable */
} sibles_adv_extended_mode_config_t;


typedef struct
{
    uint32_t duration;                       /**< Advertising duration(in unit of 10ms). */
    uint32_t interval;                       /**< Advertising interval(in unit of 625us). Must be greater than 20ms. */
    uint8_t max_skip;                        /**< Maximum number of advertising events the controller can skip before sending the */
    uint8_t phy;                             /**< Indicate on which PHY secondary advertising has to be performed (@see enum gapm_phy_type) */
    uint8_t adv_sid;                         /**< Advertising SID */
    uint8_t connectable_enable;              /**< For extended adv, shall not be both connectable and scannable */
    uint16_t adv_intv_min;  /**< Minimum periodic advertising interval (in unit of 1.25ms). Must be greater than 20ms. */
    uint16_t adv_intv_max;  /**< Maximum periodic advertising interval (in unit of 1.25ms). Must be greater than 20ms. */
} sibles_adv_periodic_mode_config_t;



/**
 * @brief Configuration of different advertising mode (@see enum sibles_adv_mode_t).
 */
typedef union
{
    sibles_adv_connect_mode_config_t conn_config;                 /**< Connected advertising configure. */
    sibles_adv_broadcast_mode_config_t broadcast_config;          /**< Non-connected advertising configure. */
    sibles_adv_directed_connect_mode_config_t directed_config;    /**< Directed connected advertising configure. */
    sibles_adv_extended_mode_config_t extended_config;            /**< Extended advertising configure. */
    sibles_adv_periodic_mode_config_t periodic_config;            /**< Periodic advertising configure which only valid under SIBLES_ADV_PERIODIC_MODE. */
} sibles_adv_mode_config_t;

typedef struct
{
    uint8_t adv_mode;                         /**< Advertising mode for different usage (@see enum sibles_adv_mode_t). */
    sibles_adv_mode_config_t mode_config;     /**< Configuration for differernt advertising mode. */

    int8_t max_tx_pwr;                        /**< Maximum power level at which the advertising packets have to be
                                                   transmitted. */
    uint8_t is_auto_restart;                  /**< Whether restart advertising after connection disconencted. */
    uint8_t white_list_enable;                /**< Enable white_list for filter policy. */
    uint8_t is_rsp_data_duplicate;            /**< Whether scan response data is same as adv data. */

} sibles_adv_config_t;


/**
 * @brief The structure of device local name in advertising data.
 */
typedef struct
{
    uint8_t name_len;                             /**< Name length. */
    char    name[__ARRAY_EMPTY];                  /**< Name string. */
} sibles_adv_type_name_t;

/**
 * @brief The structure of service uuid.
 */
typedef struct
{
    uint8_t uuid_len;                             /**< Service uuid len. */
    union
    {
        uint8_t uuid_16[ATT_UUID_16_LEN];         /**< 16-bit uuid. */
        uint8_t uuid_32[ATT_UUID_32_LEN];         /**< 32-bit uuid. */
        uint8_t uuid_128[ATT_UUID_128_LEN];       /**< 128-bit uuid. */
    } uuid;
} sibles_adv_uuid_t;;

/**
 * @brief The structure of service uuid list in advertising data.
 */
typedef struct
{
    uint8_t count;                                 /**< Service count. */
    sibles_adv_uuid_t uuid_list[__ARRAY_EMPTY];                  /**< Service uuid list. */
} sibles_adv_type_srv_uuid_t;

/**
 * @brief The structure of service data in advertising data.
 */
typedef struct
{
    sibles_adv_uuid_t uuid;                        /**< Service uuid. */
    uint8_t data_len;                              /**< Additional data length. */
    uint8_t additional_data[__ARRAY_EMPTY];        /**< Additional data. */
} sibles_adv_type_srv_data_t;

/**
 * @brief The structure of perferred slave conneciton interval in advertising data.
 */
typedef struct
{
    uint16_t min_interval;                          /**< Minimum interval. */
    uint16_t max_interval;                          /**< Maximum interval. */
} sibles_adv_type_conn_interval_t;

/**
 * @brief The structure of manufacturer specific dat in advertising data.
 */
typedef struct
{
    uint16_t company_id;                            /**< Company id assigned by Bluetooth Sig. */
    uint8_t data_len;                               /**< Additional data length. */
    uint8_t additional_data[__ARRAY_EMPTY];         /**< Additional data. */
} sibles_adv_type_manufacturer_data_t;

/**
 * @brief The structure of customized advertising data.
 */
typedef struct
{
    uint8_t len;                                   /**< data len. */
    uint8_t data[__ARRAY_EMPTY];                   /**< Advertising data conformed to data format in Bluetooth Core Spec */
} sibles_adv_type_customize_t;

/**
 * @brief The structure of advertising data filed defined in Supplement to the Bluetooth Core Spec.
 */
typedef struct
{
    uint8_t disc_mode;                                          /**< Discovery mode(@see enum gapm_adv_disc_mode) to set flags field. */
    uint8_t flags;                                              /**< flags, only available when disc_mode is set to GAPM_ADV_MODE_CUSTOMIZE*/
    uint8_t *tx_pwr_level;                                      /**< Tx power level field. */
    uint16_t *appearance;                                       /**< Appearance field. */
    uint16_t *advertising_interval;                             /**< Advertising field. */
    sibles_adv_type_name_t *shortened_name;                     /**< Local name field with shortened name. */
    sibles_adv_type_name_t *completed_name;                     /**< Local name field with copmleted name. */
    sibles_adv_type_srv_uuid_t *completed_uuid;                 /**< Service UUID filed with completed uuid. */
    sibles_adv_type_srv_uuid_t *incompleted_uuid;               /**< Service UUID filed with incompleted uuid. */
    sibles_adv_type_srv_data_t *srv_data;                       /**< Service data filed. */
    sibles_adv_type_conn_interval_t *preferred_conn_interval;   /**< Slave connection interval rangefiled. */
    sibles_adv_type_manufacturer_data_t *manufacturer_data;     /**< Manufacturer specified data filed. */
    sibles_adv_type_customize_t *customized_data;               /**< Customized advertiting data not included in above
                                                                     filed. */
} sibles_adv_data_t;


typedef struct
{
    uint16_t len;
    uint8_t data[__ARRAY_EMPTY];
} sibles_periodic_adv_t;


/**
 * @brief The structure of advertising configure.
 */
typedef uint8_t (*sibles_advertising_event)(uint8_t event, void *context, void *data);

/**
 * @brief The structure of advertising configure.
 */
typedef struct
{
    uint8_t own_addr_type;                    /**< Own addr type. @see enum gapm_own_addr. */
    ble_gap_addr_t peer_addr;
    sibles_adv_config_t config;                /**< Advertising parameter configuration. */

    sibles_adv_data_t adv_data;               /**< Advertising data. */
    sibles_adv_data_t rsp_data;               /**< Scan response data. */
    sibles_periodic_adv_t *periodic_data;      /**< Periodic data. */

    sibles_advertising_event evt_handler;     /**< User callback to listen events(@see enum sibles_adv_event_t). */

} sibles_advertising_para_t;


typedef struct
{
    uint8_t len;
    uint8_t data[__ARRAY_EMPTY];
} sibles_gap_adv_data_t;


/**
 * @brief The structure of advertising configure.
 */
typedef struct
{
    uint8_t state;
    uint8_t adv_mode;
    uint8_t adv_running_mode;
    uint8_t adv_idx;
    uint8_t conn_idx;
    uint8_t backgroud_adv;
    uint8_t adv_transist;
    sibles_adv_config_t config;
    ble_gap_adv_parameter_t adv_para;
    ble_gap_adv_data_t *adv_data;
    ble_gap_adv_data_t *scan_rsp_data;
    ble_gap_adv_data_t *periodic_data;

    sibles_advertising_event evt_handler;
} sibles_advertising_context_t;



/**
 * @brief The structure of #SIBLES_ADV_EVT_REQUEST_PEER_DEVICE_ADDR.
 */
typedef ble_gap_addr_t sibles_adv_evt_request_peer_addr_t;

/**
 * @brief The structure of #SIBLES_ADV_EVT_ADV_STARTED.
 */
typedef struct
{
    uint8_t status;                           /**< Advertising started status. error code is from #hl_err */
    uint8_t adv_mode;                         /**< Running adv mode (@see enum sibles_adv_running_mode_t). */
} sibles_adv_evt_startted_t;

/**
 * @brief The structure of #SIBLES_ADV_EVT_ADV_STOPPED.
 */
typedef struct
{
    uint8_t reason;                           /**< Advertising started status. error code is from #hl_err */
    uint8_t adv_mode;                         /**< Running adv mode (@see enum sibles_adv_running_mode_t). */
} sibles_adv_evt_stopped_t;


/**
 * @brief Advertising configuration init.
   @param[in] context service context defined by #SIBLES_ADVERTISING_CONTEXT_DECLAR.
   @param[in] para configure advertising behavior.
   @retval result result(@see enum sibles_adv_err_t).
 */
uint8_t sibles_advertising_init(sibles_advertising_context_t *context, sibles_advertising_para_t *para);

/**
 * @brief Start advertising.
   @param[in] context service context defined by #SIBLES_ADVERTISING_CONTEXT_DECLAR.
   @retval result result(@see enum sibles_adv_err_t).
 */
uint8_t sibles_advertising_start(sibles_advertising_context_t *context);

/**
 * @brief Stop advertising.
   @param[in] context service context defined by #SIBLES_ADVERTISING_CONTEXT_DECLAR.
   @retval result result(@see sibles_adv_err_t).
 */
uint8_t sibles_advertising_stop(sibles_advertising_context_t *context);

/**
 * @brief Reconfigure parameter of advertising, only could be called init completed but advertising not started.
   @param[in] context service context defined by #SIBLES_ADVERTISING_CONTEXT_DECLAR.
   @param[in] config to cofingure adveritsing parameters.
   @retval result result(@see sibles_adv_err_t).
 */
uint8_t sibles_advertising_reconfig(sibles_advertising_context_t *context, sibles_adv_config_t *config);


uint8_t sibles_advertising_delete(sibles_advertising_context_t *context);

/**
 * @brief Update advertising and scan response data, only could be called init completed and not starting or stopping
          advertsing..
   @param[in] context service context defined by #SIBLES_ADVERTISING_CONTEXT_DECLAR.
   @param[in] adv_data advertising data.
   @param[in] rsp_data scan response data.
   @retval result result(@see sibles_adv_err_t).
 */
uint8_t sibles_advertising_update_adv_and_scan_rsp_data(sibles_advertising_context_t *context,
        sibles_adv_data_t *adv_data,
        sibles_adv_data_t *rsp_data);


uint8_t sibles_advertising_update_periodic_data(sibles_advertising_context_t *context,
        sibles_periodic_adv_t *periodic_data);


int sibles_advertising_evt_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context);


/**
* @}
*/


#endif // BF0_SIBLES_ADVERTISING_H_


