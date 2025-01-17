/**
  ******************************************************************************
  * @file   bf0_ble_ancs.h
  * @author Sifli software development team
  * @brief Header file - Sibles ANCS service.
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

#ifndef __BF0_BLE_ANCS_H
#define __BF0_BLE_ANCS_H

#include "bf0_ble_common.h"


/** @defgroup profile BLE profiles
  * @ingroup sibles
  * @{
  */


/**
 * @defgroup ANCS ANCS
 * @{
 */


/**
 * @brief ANCS error type.
 */
enum ble_ancs_err
{
    BLE_ANCS_ERR_NO_ERR,                                /**< No error.*/
    BLE_ANCS_ERR_NO_CONNECTION,                       /**< No connection yet. */
    BLE_ANCS_ERR_SEARCH_REMOTE_SERVICE_FAILED,        /**< Remote device doesn't support ANCS NP. */
    BLE_ANCS_ERR_REGISTER_REMOTE_DEVICE_FAILED,       /**< Register remote deivce failed. */
    BLE_ANCS_ERR_REJECTED,                            /**< Current action is rejected due to, etc, the action is on-going. */
};

/**
 * @brief ANCS events that notify user.
 */
enum ble_ancs_event
{
    BLE_ANCS_ENABLE_RSP = BLE_ANCS_TYPE,/**< Enable result response. */
    BLE_ANCS_NOTIFICATION_IND,          /**< Notification indication. */
    BLE_ANCS_GET_APP_ATTR_RSP,          /**< Get APP attribute response. */
};

/**
 * @brief Category ID of IOS notification classify.
 */
typedef enum
{
    BLE_ANCS_CATEGORY_ID_OTHER,                 /**< Others. */
    BLE_ANCS_CATEGORY_ID_INCOMING_CALL,         /**< Incoming call. */
    BLE_ANCS_CATEGORY_ID_MISSED_CALL,           /**< Missed call. */
    BLE_ANCS_CATEGORY_ID_VOICE_MAIL,            /**< Voice call. */
    BLE_ANCS_CATEGORY_ID_SOCIAL,                /**< Social software. */
    BLE_ANCS_CATEGORY_ID_SCHEDULE,              /**< Schedule. */
    BLE_ANCS_CATEGORY_ID_EMAIL,                 /**< Email. */
    BLE_ANCS_CATEGORY_ID_NEWS,                  /**< News. */
    BLE_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,    /**< Health and fitness. */
    BLE_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE,  /**< Business and finance. */
    BLE_ANCS_CATEGORY_ID_LOCATION,              /**< Location. */
    BLE_ANCS_CATEGORY_ID_ENTERTAINMENT,         /**< Entertainment. */
} ble_ancs_category_id_t;

/**
 * @brief Category ID Mask of IOS notification classify.
 */
typedef enum
{
    BLE_ANCS_CATEGORY_ID_MASK_OTHER                = 1 << BLE_ANCS_CATEGORY_ID_OTHER,                /**< Others mask. */
    BLE_ANCS_CATEGORY_ID_MASK_INCOMING_CALL        = 1 << BLE_ANCS_CATEGORY_ID_INCOMING_CALL,        /**< Incoming call mask. */
    BLE_ANCS_CATEGORY_ID_MASK_MISSED_CALL          = 1 << BLE_ANCS_CATEGORY_ID_MISSED_CALL,          /**< Missed call mask. */
    BLE_ANCS_CATEGORY_ID_MASK_VOICE_MAIL           = 1 << BLE_ANCS_CATEGORY_ID_VOICE_MAIL,           /**< Voice call mask. */
    BLE_ANCS_CATEGORY_ID_MASK_SOCIAL               = 1 << BLE_ANCS_CATEGORY_ID_SOCIAL,               /**< Social software mask. */
    BLE_ANCS_CATEGORY_ID_MASK_SCHEDULE             = 1 << BLE_ANCS_CATEGORY_ID_SCHEDULE,             /**< Schedule mask. */
    BLE_ANCS_CATEGORY_ID_MASK_EMAIL                = 1 << BLE_ANCS_CATEGORY_ID_EMAIL,                /**< Email mask. */
    BLE_ANCS_CATEGORY_ID_MASK_NEWS                 = 1 << BLE_ANCS_CATEGORY_ID_NEWS,                 /**< News mask. */
    BLE_ANCS_CATEGORY_ID_MASK_HEALTH_AND_FITNESS   = 1 << BLE_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,   /**< Health and fitness mask. */
    BLE_ANCS_CATEGORY_ID_MASK_BUSINESS_AND_FINANCE = 1 << BLE_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE, /**< Business and finance mask. */
    BLE_ANCS_CATEGORY_ID_MASK_LOCATION             = 1 << BLE_ANCS_CATEGORY_ID_LOCATION,             /**< Location mask. */
    BLE_ANCS_CATEGORY_ID_MASK_ENTERTAINMENT        = 1 << BLE_ANCS_CATEGORY_ID_ENTERTAINMENT,        /**< Entertainment mask. */
    BLE_ANCS_CATEGORY_ID_MASK_ALL                  = 0xFFFF,                                           /**< Mask all categroy. */
} ble_ancs_category_id_mask_t;


/**
 * @brief Event id indicats whether the notification added, modified or removed.
 */
typedef enum
{
    BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED,       /**< Notification is added. */
    BLE_ANCS_EVENT_ID_NOTIFICATION_MODIFIED,    /**< Notification is modified. */
    BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED,     /**< Notification is removed. */
} ble_ancs_event_id_t;


/**
 * @brief Event flags indicate event specificities.
 */
typedef enum
{
    BLE_ANCS_EVENT_FLAG_SILENT = 1 << 0,          /**< Slient. */
    BLE_ANCS_EVENT_FLAG_IMPORTANT = 1 << 1,       /**< Important. */
    BLE_ANCS_EVENT_FLAG_PRE_EXISTING = 1 << 2,    /**< Pre-existing. */
    BLE_ANCS_EVENT_FLAG_POSITIVE_ACTION = 1 << 3, /**< Positive action. */
    BLE_ANCS_EVENT_FLAG_NEGATIVE_ACTION = 1 << 4, /**< Negative action. */
} ble_ancs_event_flag_t;

/**
 * @brief Command ID indicates the command.
 */
typedef enum
{
    BLE_ANCS_COMMAND_ID_GET_NOTIFICATION_ATTR,       /**< Get notifcation attribute. */
    BLE_ANCS_COMMAND_ID_GET_APP_ATTR,                /**< Get app attribute. */
    BLE_ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION, /**< Perform notification action. */
} ble_ancs_command_id_t;

/**
 * @brief Notification attribute ID.
 */
typedef enum
{
    BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID,                 /**< APP ID. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE,                  /**< Notification title. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_SUB_TITLE,              /**< Notification sub-title. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE,                /**< Message. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE,           /**< Message size. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_DATE,                   /**< Date. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABLE,  /**< Positive action label. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABLE,  /**< Negative action label. */
} ble_ancs_notification_attr_id_t;


/**
 * @brief Mask of Notification attribute ID.
 */
typedef enum
{
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_APP_ID                = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID,                 /**< APP ID mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_TITLE                 = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE,                  /**< Notification title mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_SUB_TITLE             = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_SUB_TITLE,              /**< Notification sub-title mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_MESSAGE               = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE,                /**< Message mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_MESSAGE_SIZE          = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE,           /**< Message size mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_DATE                  = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_DATE,                   /**< Date mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_POSITIVE_ACTION_LABLE = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABLE,  /**< Positive action label mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_NEGATIVE_ACTION_LABLE = 1 << BLE_ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABLE,  /**< Negative action label mask. */
    BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_ALL                   = 0xFFFF,                                                    /**< Mask all. */
} ble_ancs_notification_attr_mask_id_t;


/**
 * @brief Action ID of perform action.
 */
typedef enum
{
    BLE_ACTION_ID_POSITIVE,                     /**< Positive action. */
    BLE_ACTION_ID_NEGATIVE,                     /**< Negative action. */
} ble_ancs_action_id_val_t;

/**
 * @brief APP attribute ID.
 */
typedef enum
{
    BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME,          /**< Display name. */
} ble_ancs_app_attr_id_val_t;

/**
 * @brief The structure of #BLE_ANCS_ENABLE_RSP.
 */
typedef struct
{
    uint8_t conn_idx;      /**< Connection index */
    uint8_t result;        /**< @see enum ble_ancs_err. */
} ble_ancs_enable_rsp_t;

#ifndef BSP_USING_PC_SIMULATOR
/**
 * @brief The structure of attribute value.
 */
typedef __PACKED_STRUCT
{
    uint8_t attr_id;                /**< Attribute id. */
    uint16_t len;                   /**< Attribute length. */
    uint8_t data[];    /**< Attribute content. */
} ble_ancs_attr_value_t;
#else
//for pc simulation. becuase using array, need align to 4 bytes
typedef struct
{
    uint8_t attr_id;                /**< Attribute id. */
    uint16_t len;                   /**< Attribute length. */
    uint8_t data[128];    /**< Attribute content. */
} ble_ancs_attr_value_t;
#endif


/**
 * @brief The structure of #BLE_ANCS_GET_APP_ATTR_RSP.
 */
typedef struct
{
    uint8_t *app_id;                /**< APP ID. */
    uint8_t app_id_len;             /**< APP ID length. */
    uint8_t attr_count;             /**< Attribute count. */
    ble_ancs_attr_value_t *value;   /**< Attribute value. */
} ble_ancs_get_app_attr_rsp_t;


/**
 * @brief The structure of #BLE_ANCS_NOTIFICATION_IND.
 */
typedef struct
{
    uint8_t evt_id;                 /**< Event ID(@see enum ble_ancs_event_id_t) */
    uint8_t evt_flag;               /**< Event flag(@see enum ble_ancs_event_flag_t). */
    uint8_t cate_id;                /**< Category ID(@see enum ble_ancs_category_id_t). */
    uint8_t cate_count;             /**< Category count. */
    uint32_t noti_uid;              /**< Noti UID. */
    uint8_t attr_count;             /**< Notification attribute count. */
    ble_ancs_attr_value_t *value;   /**< Notification attribute value. */
} ble_ancs_noti_attr_t;


/**
 * @brief Enable ble ancs profile service, it will search ANCS service from peer device. It should be called after
 connection establishment.
   @param[in] conn_idx connection index.
   @retval result. 0 is successful..
 */
uint8_t ble_ancs_enable(uint8_t conn_idx);

/**
 * @brief Set ble ancs category mask to filter dedicated category.
   @param[in] mask categroy mask(@see enum ble_ancs_category_id_mask_t).
 */
void ble_ancs_category_mask_set(uint16_t mask);


/**
 * @brief Configure notification attribte id and according length.
   @param[in] attr_index notification attribute index(@see enum ble_ancs_notification_attr_id_t).
   @param[in] enable enable or disable attribute index.
   @param[in] len the maximum length of associated attribute index.
 */
void ble_ancs_attr_enable(uint8_t attr_index, uint8_t enable, uint16_t len);

/**
 * @brief Configure app attribute id and according length.
   @param[in] app_index app attribute index(@see enum ble_ancs_app_attr_id_val_t).
   @param[in] enable enable or disable attribute index.
   @param[in] len the maximum length of associated attribute index.
 */
void ble_ancs_app_enable(uint8_t app_index, uint8_t enable, uint16_t len);

/**
 * @brief Perform notification action with speicifed notification UID.
   @param[in] noti_uid notification UID.
   @param[in] action_id action id(@see enum ble_ancs_action_id_val_t).
 */
void ble_ancs_perform_notification_action(uint32_t noti_uid, ble_ancs_action_id_val_t action_id);

/**
 * @brief Acquire app attribute via app ID.
   @param[in] app_id app ID.
   @param[in] app_id_len app ID length.
 */
int32_t ble_ancs_get_app_attr(uint8_t *app_id, uint8_t app_id_len);

/**
 * @brief Set CCCD of ancs.
   @param[in] is_enable is enabled cccd.
 */
uint8_t ble_ancs_cccd_enable(uint8_t is_enable);

/**
* @}
*/

/**
* @}
*/



#endif //__BF0_BLE_ANCS_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
