/**
  ******************************************************************************
  * @file   dfu_port_srv.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
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


#ifndef __DFU_PORT_SRV_H
#define __DFU_PORT_SRV_H

#include "data_service.h"

typedef enum
{
    BLE_DFU_SEND_DATA = MSG_SERVICE_CUSTOM_ID_BEGIN,

    /* Response. */
    BLE_DFU_SEND_DATA_RSP = BLE_DFU_SEND_DATA | RSP_MSG_TYPE,

    BLE_DFU_REBOOT_AFTER_DISCONNECT = BLE_DFU_SEND_DATA + 1,
    BLE_DFU_REBOOT_AFTER_DISCONNECT_RSP = BLE_DFU_REBOOT_AFTER_DISCONNECT | RSP_MSG_TYPE,

    BLE_DFU_REBOOT_DISCONNECT = BLE_DFU_REBOOT_AFTER_DISCONNECT + 1,
    BLE_DFU_REBOOT_DISCONNECT_RSP = BLE_DFU_REBOOT_DISCONNECT | RSP_MSG_TYPE,
} ble_dfu_service_message_t;


typedef struct
{
    uint8_t event;
    uint16_t len;
    uint8_t data[0];
} ble_dfu_service_data_t;

typedef struct
{
    uint8_t handle;         /**< Handle for the transmission channel. */
    uint8_t cate_id;        /**< CategoryID for different user. */
    uint16_t len;           /**< Data length. */
    uint8_t data[0];          /**< Transmission data. */
} ble_dfu_service_send_data_t;

#endif // __DFU_PORT_SRV_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
