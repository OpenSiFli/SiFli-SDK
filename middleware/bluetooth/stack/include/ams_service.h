/**
  ******************************************************************************
  * @file   ams_service.h
  * @author Sifli software development team
  * @brief Header file - AMS service as data service provider.
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


#ifndef __AMS_SERVICE_H
#define __AMS_SERVICE_H

#include "bf0_ble_ams.h"

#ifndef BSP_USING_PC_SIMULATOR
    #define __ARRAY_EMPTY_
#endif

#define AMS_SERVICE_DEFAULT_PLAYER_MASK (BLE_AMS_PLAYER_ATTR_ID_ALL_MASK)
#define AMS_SERVICE_DEFAULT_QUEUE_MASK (BLE_AMS_QUEUE_ATTR_ID_ALL_MASK)
#define AMS_SERVICE_DEFAULT_TRACK_MASK (BLE_AMS_TRACK_ATTR_ID_ALL_MASK)



typedef enum
{
    AMS_SERVICE_SET_PLAYER_ATTRIBUTE_MASK,
    AMS_SERVICE_SET_QUEUE_ATTRIBUTE_MASK,
    AMS_SERVICE_SET_TRACK_ATTRIBUTE_MASK,
    AMS_SERVICE_SEND_REMOTE_COMMAND,
    AMS_SERVICE_ENABLE_CCCD,
} ams_service_command_t;



typedef struct
{
    ams_service_command_t command;
    union
    {
        uint8_t player_mask;
        uint8_t queue_mask;
        uint8_t track_mask;
        ble_ams_cmd_t remote_cmd;
        uint8_t enable_cccd;
    } data;
} ams_service_config_t;

#endif // __AMS_SERVICE_H


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

