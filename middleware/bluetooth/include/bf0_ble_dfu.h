/**
  ******************************************************************************
  * @file   bf0_ble_dfu.h
  * @author Sifli software development team
  * @brief Header file - Sibles dfu service interface.
 *
  ******************************************************************************
*/
/**
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


#ifndef __BF0_SIBLES_DFU_H
#define __BF0_SIBLES_DFU_H

typedef struct
{
    uint8_t img_id;
    uint8_t img_size;
} ble_dfu_start_request_t;

typedef struct
{
    uint8_t result;
} ble_dfu_start_request_response_t;

typedef struct
{
    uint8_t percent;
} ble_dfu_packet_recv_t;

typedef struct
{
    uint8_t result;
} ble_dfu_end_t;

typedef enum
{
    BLE_DFU_EVENT_SUCCESSED,
    BLE_DFU_EVENT_POSTPONE,
    BLE_DFU_EVENT_FAILED,
} ble_dfu_event_ack_t;



typedef enum
{
    BLE_DFU_START_REQUEST,
    BLE_DFU_PACKET_RECV,
    BLE_DFU_END,
} ble_dfu_event_t;

typedef uint8_t (*ble_dfu_callback)(uint16_t event, void *param);

void ble_dfu_register(ble_dfu_callback callback);

void ble_dfu_respond_start_request(ble_dfu_start_request_response_t *rsp);





#endif // __BF0_SIBLES_DFU_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

