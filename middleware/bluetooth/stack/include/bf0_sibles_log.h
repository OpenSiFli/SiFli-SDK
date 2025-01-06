/**
  ******************************************************************************
  * @file   bf0_sibles_log.h
  * @author Sifli software development team
  * @brief Header file - Sibles serial transfer service.
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


#ifdef BSP_BLE_LOG

#ifndef _BF0_SIBLES_LOG_H
#define _BF0_SIBLES_LOG_H

#define MAX_PACKCT_SIZE 243
#define SERIAL_TRANS_HEADER 4
#define BLE_LOG_SEND_HEADER 5

#define BLE_LOG_VERSION 1

enum ble_log_command
{
    BLE_LOG_COMMAND_DISABLE,
    BLE_LOG_COMMAND_ENABLE,
    BLE_LOG_COMMAND_GET,
};

enum ble_log_status
{
    BLE_LOG_STATUS_OK,
    BLE_LOG_STATUS_DISABLE,
};

enum ble_log_state
{
    BLE_LOG_STATE_NONE,
    BLE_LOG_STATE_TRANSPORT,
};

typedef struct
{
    uint32_t start_addr;
    uint32_t total_size;
    uint32_t receive_size;
    uint32_t receive_index;
} ble_phone_send_data;

typedef struct
{
    uint8_t is_open;
    /* Handle for serial tran service. */
    uint8_t handle;

    uint8_t conn_idx;
    uint8_t conn_state;

    uint16_t mtu;

    // totol log data size
    uint32_t size;
    uint8_t *addr;

    uint32_t send_index;
    uint8_t command;
    uint8_t state;

    ble_phone_send_data receive_data;
} ble_log_env_t;

typedef enum
{
    BLE_LOG_CLEAR,
    BLE_LOG_GET,
    BLE_LOG_GET_SEND_START,
    BLE_LOG_GET_SEND_PACKET,
    BLE_LOG_GET_SEND_END,
    BLE_LOG_STATUS_INQUIRY,
    BLE_LOG_STATUS_REPLY,
    BLE_LOG_GET_SEND_PACKET_RSP,
    BLE_LOG_PHONE_DATA_SEND_START_REQ,
    BLE_LOG_PHONE_DATA_SEND_START_RSP,
    BLE_LOG_PHONE_DATA_SEND,
    BLE_LOG_PHONE_DATA_SEND_END,
    BLE_LOG_TSDB_GET,
    BLE_LOG_TSDB_METRICS_GET,
    BLE_LOG_TSDB_SWITCH_RESULT,

    BLE_LOG_GET_SEND_FINISH = 15,
    BLE_LOG_ON_OFF = 16,

    BLE_LOG_ASSERT_GET = 17,
    BLE_LOG_ASSERT_CLEAR,
    BLE_LOG_ASSERT_ON_OFF,

    BLE_LOG_METRICS_GET = 20,
    BLE_LOG_METRICS_CLEAR,
    BLE_LOG_METRICS_ON_OFF,

    BLE_LOG_MONKEY_GET = 23,
    BLE_LOG_MONKEY_CLEAR,

    BLE_LOG_OP_FINISH = 25,

    BLE_AUDIO_DUMP_ENABLE = 30,
    BLE_AUDIO_DUMP_GET_RSP = 31,
    BLE_AUDIO_DUMP_CLEAR = 32,
    BLE_AUDIO_DUMP_DELAY = 33,
    BLE_AUDIO_DUMP_COMMAND = 34,
    BLE_AUDIO_DUMP_GET = 35,
    BLE_MEM_GET_REQ,
    BLE_MEM_GET_RSP,
    BLE_MEM_SET,

    BLE_HCI_LOG_ON_OFF = 39,
    BLE_HCI_LOG_CLEAR = 40,
    BLE_HCI_LOG_GET = 41,

    BLE_DEV_INFO_GET = 42,
    BLE_DEV_REMOTE_INFO_GET_RSP = 43,

    BLE_TX_POWER_SET = 44,

    BLE_ASSERT_CMD = 45,
    BLE_FINSH_CMD = 46,

    BLE_LOG_INQUIRY_REQ = 47,
    BLE_LOG_INQUIRY_RSP = 48,
} ble_log_protocol_msg_id_t;

typedef enum
{
    BLE_AUDIO_DUMP_DOWLINK,
    BLE_AUDIO_DUMP_DOWLINK_AGC,
    BLE_AUDIO_DUMP_ADUPRC,
    BLE_AUDIO_DUMP_DC_OUTPUT,
    BLE_AUDIO_DUMP_RAMP_IN_OUTPUT,
    BLE_AUDIO_DUMP_AECM_INPUT1,
    BLE_AUDIO_DUMP_AECM_INPUT2,
    BLE_AUDIO_DUMP_AECM_OUTPUT,
    BLE_AUDIO_DUMP_ANC_OUTPUT,
    BLE_AUDIO_DUMP_AGC_OUTPUT,
    BLE_AUDIO_DUMP_RAMP_OUT_OUTPUT,
} ble_audio_dump_type;

typedef struct
{
    uint16_t message_id; /* ref @ble_log_protocol_msg_id_t */
    uint16_t length;
    uint8_t data[0];
} ble_log_protocol_t;

/**
 * get ble log state
 * return: BLE_LOG_STATE_TRANSPORT if is transporting log to phone
 * BLE_LOG_STATE_NONE for no log work for now
 */
uint8_t ble_log_get_transport_state();

#endif // _BF0_SIBLES_LOG_H

#endif // BSP_BLE_LOG

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

