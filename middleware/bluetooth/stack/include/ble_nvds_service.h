/**
  ******************************************************************************
  * @file   ble_nvds_service.h
  * @author Sifli software development team
  * @brief Header file - BLE NVDS service via data service.
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


#ifndef __BLE_NVDS_SERVICE_H
#define __BLE_NVDS_SERVICE_H
#include "data_service.h"

typedef enum
{
    BLE_NVDS_SERVICE_READ = MSG_SERVICE_CUSTOM_ID_BEGIN,
    BLE_NVDS_SERVICE_READ_TAG,
    BLE_NVDS_SERVICE_WRITE,
    BLE_NVDS_SERVICE_WRITE_TAG,
    BLE_NVDS_SERVICE_FLUSH,
    BLE_NVDS_SERVICE_UPDATE_ADDR,

    /* Response. */
    BLE_NVDS_SERVICE_READ_RSP = BLE_NVDS_SERVICE_READ | RSP_MSG_TYPE,
    BLE_NVDS_SERVICE_READ_TAG_RSP = BLE_NVDS_SERVICE_READ_TAG | RSP_MSG_TYPE,
    BLE_NVDS_SERVICE_WRITE_RSP = BLE_NVDS_SERVICE_WRITE | RSP_MSG_TYPE,
    BLE_NVDS_SERVICE_WRITE_TAG_RSP = BLE_NVDS_SERVICE_WRITE_TAG | RSP_MSG_TYPE,
    BLE_NVDS_SERVICE_FLUSH_COMPLETED = BLE_NVDS_SERVICE_FLUSH | RSP_MSG_TYPE,
    BLE_NVDS_SERVICE_UPDATE_ADDR_RSP = BLE_NVDS_SERVICE_UPDATE_ADDR | RSP_MSG_TYPE
} ble_nvds_service_message_t;

uint8_t ble_nvds_read_mac_address(uint8_t *addr, uint8_t len);

#endif // __BLE_NVDS_SERVICE_H
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

