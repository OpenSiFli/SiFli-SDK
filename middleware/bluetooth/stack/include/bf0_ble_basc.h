/**
  ******************************************************************************
  * @file   bf0_ble_basc.h
  * @author Sifli software development team
  * @brief Header file - Sibles battery service collector definition.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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

#ifndef __BF0_BLE_BASC_H
#define __BF0_BLE_BASC_H

#include "bf0_ble_common.h"


enum ble_basc_event
{
    //Remote Battry Level Notification see @ble_basc_bat_lev_t
    BLE_BASC_BAT_LEV_NOTIFY = BLE_BAS_TYPE,
    //Read remote Battry Level response see @ble_basc_bat_lev_t
    BLE_BASC_READ_BAT_LEV_RSP,
};


/**
 * @brief The structure of BLE_BASC_BAT_LEV_NOTIFY and BLE_BASC_READ_BAT_LEV_RSP.
 */
typedef struct
{
    // battery level
    uint8_t lev;
} ble_basc_bat_lev_t;


/**
 * @brief initiate battery service
   @param[in] is_enable whether enabled.
 */
void ble_basc_init(uint8_t enable);


/**
 * @brief Enable CCCD notify of battery level.
   @param[in] conn_idx connection index.
   @param[in] is_enable whether enabled.
   @retval result SIBLES_WRITE_NO_ERR is successful.
 */
int8_t ble_basc_enable_bat_lev_notify(uint8_t conn_idx, uint8_t enable);


/**
 * @brief read remote battery level.
   @param[in] conn_idx connection index.
   @retval result 0 is successful.
 */
int8_t ble_basc_read_battery_level(uint8_t conn_idx);


/**
 * @brief search remote battery service .
   @param[in] conn_idx connection index.
   @retval result 0 is successful.
 */
int8_t ble_basc_enable(uint8_t conn_idx);

#endif //__BF0_BLE_BASC_H