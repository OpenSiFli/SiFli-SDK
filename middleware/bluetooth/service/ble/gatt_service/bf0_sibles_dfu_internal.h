/**
  ******************************************************************************
  * @file   bf0_sibles_dfu_internal.h
  * @author Sifli software development team
  * @brief Header file - Sibles dfu internal interface.
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


#ifndef __BF0_SIBLES_DFU_INTERNAL_H
#define __BF0_SIBLES_DFU_INTERNAL_H

#include "dfu.h"

#define BLE_DFU_PACKET_SIZE 128

#define BLE_DFU_HEADER_LEN 348  // 32 + 316
#define BLE_DFU_BODY_LEN 548 // 32 + 4+ 512
#define BLE_DFU_HCPU_FLAG 146

#ifndef __ARRAY_EMPTY
    #define __ARRAY_EMPTY
#endif

typedef enum
{
    BLE_DFU_STATE_IDLE,
    BLE_DFU_STATE_NEGOTIATING,
    BLE_DFU_STATE_READY,
    BLE_DFU_STATE_RECEVING,
} ble_dfu_state_t;

typedef enum
{
    BLE_DFU_TYPE_VER_INQUIRY,
    BLE_DFU_TYPE_VER_INQUIRY_RSP,
    BLE_DFU_TYPE_NEG_REQ,
    BLE_DFU_TYPE_NEG_RSP,
    BLE_DFU_TYPE_RESUME_REQ,
    BLE_DFU_TYPE_RESUME_RSP,
    BLE_DFU_TYPE_PACKET_SEND_REQ,
    BLE_DFU_TYPE_PACKET_SEND_RSP,
    BLE_DFU_TYPE_PACKET,
    BLE_DFU_TYPE_PACKET_CFM,
    BLE_DFU_TYPE_PACKET_SEND_END
} ble_dfu_type_t;

typedef struct
{
    uint32_t total_len;
    uint32_t curr_len;
    uint16_t curr_count;
    uint16_t packet_size;
    uint8_t img_id;
} ble_dfu_info_t;

typedef struct
{
    uint32_t processed_len;
    uint8_t *dfu_data;
} ble_dfu_data_t;

typedef __PACKED_STRUCT
{
    // reserved first
    uint8_t img_count;
    uint8_t img_id[__ARRAY_EMPTY];
} ble_dfu_ver_inquiry_t;

typedef __PACKED_STRUCT
{
    uint8_t img_id;
    uint8_t img_state;
    uint8_t img_curr_ver[DFU_VERSION_LEN];
    uint8_t img_target_ver[DFU_VERSION_LEN];
    uint32_t img_target_len;
} ble_dfu_ver_info_t;

typedef __PACKED_STRUCT
{
    uint8_t msg_type;
    uint8_t img_count;
    ble_dfu_ver_info_t info[__ARRAY_EMPTY];
} ble_dfu_ver_inquiry_rsp_t;

typedef __PACKED_STRUCT
{
    uint16_t packet_size;
} ble_dfu_neg_req_t;

typedef __PACKED_STRUCT
{
    uint8_t type;
    uint8_t result;
    uint16_t packet_size;
} ble_dfu_neg_rsp_t;

#ifdef BSP_USING_DFU_COMPRESS
typedef __PACKED_STRUCT
{
    uint8_t flashid;
    struct image_header_compress_resume data;
} ble_dfu_resume_req_t;
#endif

typedef __PACKED_STRUCT
{
    uint8_t result;
} ble_dfu_resume_rsp_t;

typedef __PACKED_STRUCT
{
    uint32_t total_len;
    uint8_t img_id;
} ble_dfu_packet_send_req_t;

typedef __PACKED_STRUCT
{
    uint8_t type;
    uint8_t result;
} ble_dfu_packet_send_rsp_t;

typedef __PACKED_STRUCT
{
    uint16_t count;
    uint16_t size;
    uint8_t packet[__ARRAY_EMPTY];
} ble_dfu_packet_t;

typedef __PACKED_STRUCT
{
    uint8_t type;
    uint8_t result;
} ble_dfu_packet_cfm_t;


typedef struct
{
    ble_dfu_state_t state;
    ble_dfu_info_t info;
    ble_dfu_callback callback;
    uint8_t conn_idx;
    ble_dfu_data_t data;
} ble_dfu_env_t;

#endif // __BF0_SIBLES_DFU_INTERNAL_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

