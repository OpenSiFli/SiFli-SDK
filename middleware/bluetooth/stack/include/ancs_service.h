/**
  ******************************************************************************
  * @file   ancs_service.h
  * @author Sifli software development team
  * @brief Header file - ANCS service as data service provider.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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


#ifndef __ANCS_SERVICE_H
#define __ANCS_SERVICE_H

#include "bf0_ble_ancs.h"


#define __ARRAY_EMPTY_


#define ANCS_SERVICE_DEFAULT_ATTR_MASK (BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_APP_ID | \
                                        BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_TITLE  | \
                                        BLE_ANCS_NOTIFICATION_ATTR_ID_MASK_MESSAGE)

#define ANCS_SERVICE_DEFAULT_CATE_MASK (BLE_ANCS_CATEGORY_ID_MASK_ALL)

#define ANCS_SERVICE_TITLE_LEN (100)
#define ANCS_SERVICE_SUBTITLE_LEN (100)
#define ANCS_SERVICE_MESSAGE_LEN (1024)
#define ANCS_SERVICE_MAX_APP (10)


#define ANCS_APP_ID_LEN (35)
#define ANCS_APP_NAME_LEN (20)



typedef enum
{
    ANCS_SERVICE_SET_ATTRIBUTE_MASK,
    ANCS_SERVICE_SET_CATEGORY_MASK,
    ANCS_SERVICE_ENABLE_CCCD,
    ANCS_SERVICE_PERFORM_NOTIFY_ACTION
} ancs_service_command_t;



typedef struct
{
    ancs_service_command_t command;
    union
    {
        uint16_t attr_mask;
        uint16_t cate_mask;
        uint8_t enable_cccd;
        struct
        {
            uint32_t uid;
            ble_ancs_action_id_val_t act_id;
        } action;
    } data;
} ancs_service_config_t;


typedef struct
{
    uint8_t evt_id;                 /**< Event ID(@see enum ble_ancs_event_id_t) */
    uint8_t evt_flag;               /**< Event flag(@see enum ble_ancs_event_flag_t). */
    uint8_t cate_id;                /**< Category ID(@see enum ble_ancs_category_id_t). */
    uint8_t cate_count;             /**< Category count. */
    uint32_t noti_uid;              /**< Noti UID. */
    uint8_t attr_count;             /**< Notification attribute count. */
    ble_ancs_attr_value_t value[__ARRAY_EMPTY_];   /**< Notification attribute value. */
} ancs_service_noti_attr_t;



#endif // __ANCS_SERVICE_H


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

