/**
  ******************************************************************************
  * @file   bf0_ble_hci.h
  * @author Sifli software development team
  * @brief Header file - Bluetooth HCI interface.
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

#ifndef __BF0_BLE_HCI_H
#define __BF0_BLE_HCI_H


/*
 * INCLUDE FILES
 ****************************************************************************************
    */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles_internal.h"

/*
 * DEFINES
 ****************************************************************************************
 */


enum ble_hci_event_t
{
    BLE_HCI_SEND_COMMAND_CNF = BLE_HCI_TYPE,
    BLE_HCI_EVENT_IND,
    BLE_HCI_SEND_DATA_CNF,
    BLE_HCI_DATA_IND
};


typedef struct
{
    uint8_t event_id;
    uint8_t len;
    uint8_t *data;
} ble_hci_evt_t;

typedef struct
{
    uint16_t opcode;
    uint8_t len;
    uint8_t data[__ARRAY_EMPTY];
} ble_hci_command_t;




typedef struct
{
    ///Connection handle & Data Flags
    uint16_t handle: 12;
    uint16_t pb_flag: 2;
    uint16_t bc_flag: 2;
    ///Data length in number of bytes
    uint16_t len;
    uint8_t data[__ARRAY_EMPTY];
} ble_hci_data_t;



uint8_t ble_hci_send_command(ble_hci_command_t *command);

uint8_t ble_hci_send_data(ble_hci_data_t *data);

void sifli_hci_process(void);


#endif // __BF0_BLE_HCI_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
