/**
  ******************************************************************************
  * @file   bf0_sibles_watchface.h
  * @author Sifli software development team
  * @brief Header file - Sibles watchface transport service.
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


#ifdef BSP_BLE_WATCH_FACE

#ifndef _BF0_SIBLES_WATCHFACE_H
#define _BF0_SIBLES_WATCHFACE_H

#define MAX_PACKCT_SIZE 243
#define SERIAL_TRANS_HEADER 4

#define PHONE_TYPE_IOS 1
#define PHONE_TYPE_ANDROID 2

#define MAX_DATA_LEN 10*1024
#define MAX_IOS_DATA_LEN 10*1024

#define WF_VERSION 3
#define MAX_UPDATE_REPEAT 3

#define WATCHFACE_SYNC_TIMEOUT 12000
#define MD5_LEN 32
#define WF_SEM_WAIT_TIME 2000

typedef enum
{
    WATCHFACE_FILE_TYPE_WATCHFACE,
    WATCHFACE_FILE_TYPE_MULTIPLE_LANGUAGE,
    WATCHFACE_FILE_TYPE_BACKGROUND_PIC,
    WATCHFACE_FILE_TYPE_CUSTOMIZED,
    WATCHFACE_FILE_TYPE_MUSIC,
    WATCHFACE_FILE_TYPE_WATCHFACE_JS,
    WATCHFACE_FILE_TYPE_EQ_FILE,
    WATCHFACE_FILE_TYPE_PHOTO_PREVIEW,
} watchface_file_type_t;

typedef enum
{
    WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_COMPLETE,
    WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_START,
    WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_CONTINUE,
    WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_LAST,
} watchface_photo_preview_packet_type_t;

typedef enum
{
    WATCHFACE_EVENT_SUCCESSED,
    WATCHFACE_EVENT_POSTPONE,
    WATCHFACE_EVENT_FAILED,
} watchface_event_ack_t;

typedef enum
{
    WATCHFACE_SYNC_TYPE_FILE,
    WATCHFACE_SYNC_TYPE_RSP,
} watchface_sync_type_t;

typedef watchface_event_ack_t (*watchface_callback)(uint16_t event, uint16_t length, void *param);

typedef struct
{
    uint8_t is_sync_on;
    uint32_t last_index;
} watchface_sync_t;

typedef struct
{
    uint8_t is_open;
    /* Handle for serial tran service. */
    uint8_t handle;

    uint8_t state;
    uint8_t conn_idx;

    uint8_t update_state;
    uint8_t update_repeat;

    uint16_t mtu;

    uint8_t status;
    uint8_t current_type;
    uint8_t phone_type;
    uint16_t file_type;

    uint32_t current_index;
    uint32_t receive_size;
    uint32_t total_size;

    watchface_sync_t sync;
    watchface_callback callback;

    rt_sem_t sem;
} ble_watchface_env_t;

typedef enum
{
    BLE_WF_UPDATE_NONE,
    BLE_WF_UPDATE_UPDATING,
} ble_wf_update_state_t;

typedef enum
{
    BLE_WATCHFACE_IDLE,
    BLE_WATCHFACE_PREPARE,
    BLE_WATCHFACE_FILE_START,
    BLE_WATCHFACE_FILE_DOWNLOAD,
    BLE_WATCHFACE_FILE_PROCESS,
    BLE_WATCHFACE_FILE_END,
    BLE_WATCHFACE_END,
    BLE_WATCHFACE_FILE_PRE_START,
} ble_watchface_state_t;

typedef enum
{
    BLE_WATCHFACE_START_REQ,
    BLE_WATCHFACE_START_RSP,
    BLE_WATCHFACE_FILE_SEND_START_REQ,
    BLE_WATCHFACE_FILE_SEND_START_RSP,
    BLE_WATCHFACE_FILE_SEND_DATA,
    BLE_WATCHFACE_FILE_SEND_DATA_RSP,
    BLE_WATCHFACE_FILE_SEND_END_REQ,
    BLE_WATCHFACE_FILE_SEND_END_RSP,
    BLE_WATCHFACE_END_REQ,
    BLE_WATCHFACE_END_RSP,
    BLE_WATCHFACE_LOSE_CHECK_REQ,
    BLE_WATCHFACE_LOSE_CHECK_RSP,
    BLE_WATCHFACE_ABORT_CMD,
    BLE_WATCHFACE_FILE_INFO_REQ,
    BLE_WATCHFACE_FILE_INFO_RSP,
    BLE_WATCHFACE_FILE_PHOTO_PREVIEW_DATA,
    BLE_WATCHFACE_FILE_PHOTO_PREVIEW_DATA_RSP,
} ble_watchface_protocol_msg_id_t;

typedef enum
{
    BLE_WATCHFACE_STATUS_OK,
    BLE_WATCHFACE_STATUS_GENERAL_ERROR,
    BLE_WATCHFACE_STATUS_RECEIVE_ERROR,
    BLE_WATCHFACE_STATUS_LEN_ERROR,
    BLE_WATCHFACE_STATUS_INDEX_ERROR,
    BLE_WATCHFACE_STATUS_STATE_ERROR,
    BLE_WATCHFACE_STATUS_DISCONNECT,
    BLE_WATCHFACE_STATUS_DOWNLOAD_NOT_ONGOING,
    BLE_WATCHFACE_STATUS_REMOTE_ABORT,
    BLE_WATCHFACE_STATUS_REMOTE_FILE_SIZE_CHECK_ERROR,
    BLE_WATCHFACE_STATUS_APP_ERROR = 20,
    BLE_WATCHFACE_STATUS_FILE_SIZE_ALIGNED_MISSING,
    BLE_WATCHFACE_STATUS_FILE_PATH_ERROR,
    BLE_WATCHFACE_STATUS_FILE_TYPE_ERROR,
    BLE_WATCHFACE_STATUS_MEM_MALLOC_ERROR,
    BLE_WATCHFACE_STATUS_FILE_OPEN_ERROR,
    BLE_WATCHFACE_STATUS_FILE_INFO_ERROR,
    BLE_WATCHFACE_STATUS_FILE_WRITE_ERROR,
    BLE_WATCHFACE_STATUS_FILE_CLOSE_ERROR,
    BLE_WATCHFACE_STATUS_FILE_EXTENSION_ERROR,
    BLE_WATCHFACE_STATUS_MKDIR_ERROR,
    BLE_WATCHFACE_STATUS_SWITCH_DIRECTORY_ERROR,
    BLE_WATCHFACE_STATUS_BLE_PARAMETERS_NULL,
    BLE_WATCHFACE_STATUS_MEM_NOT_CONTINUOUS,
    BLE_WATCHFACE_STATUS_FILE_SIZE_ERROR,
    BLE_WATCHFACE_STATUS_CRC_INIT_ERROR,
    BLE_WATCHFACE_STATUS_CRC_CALCULATE_ERROR,
    BLE_WATCHFACE_STATUS_SPACE_ERROR,
    BLE_WATCHFACE_STATUS_TIMEOUT_ERROR,
    BLE_WATCHFACE_STATUS_UI_INVALID,
    BLE_WATCHFACE_STATUS_BLE_DISCONNECT,
    BLE_WATCHFACE_STATUS_USER_ABORT,
    BLE_WATCHFACE_STATUS_RECV_DATA_TIMEOUT,
} ble_watchface_status_id_t;

typedef struct
{
    uint16_t message_id; /* ref @ble_watchface_protocol_msg_id_t */
    uint16_t length;
    uint8_t data[0];
} ble_watchface_protocol_t;


// for callback

typedef enum
{
    WATCHFACE_APP_START,
    WATCHFACE_APP_FILE_START,
    WATCHFACE_APP_FILE_DOWNLOAD,
    WATCHFACE_APP_FILE_END,
    WATCHFACE_APP_END,
    WATCHFACE_APP_ERROR,
    WATCHFACE_APP_FILE_INFO,
    WATCHFACE_APP_FILE_PHOTO_PREVIEW_DOWNLOAD,
} ble_watchface_app_event_t;

typedef struct
{
    uint32_t event;
    uint16_t type;
    uint32_t all_files_len;
} ble_watchface_start_ind_t;

typedef struct
{
    uint32_t event;
    uint32_t file_len;
    uint32_t to_use_space;
    uint16_t file_name_len;
    uint8_t *file_name;
} ble_watchface_file_start_ind_t;

typedef struct
{
    uint32_t event;
    uint32_t file_blocks;
    uint8_t md5[MD5_LEN];
} ble_watchface_file_info_ind_t;

typedef enum
{
    WATCHFACE_RESUME_RESTART,
    WATCHFACE_RESUME_ENABLE,
    WATCHFACE_RESUME_OLD_VERSION,
} ble_watchface_resume_state_t;

typedef struct
{
    uint32_t event;
    uint32_t data_len;
    uint8_t *data;
} ble_watchface_file_download_ind_t;

typedef struct
{
    uint32_t event;
    uint8_t packet_type;

    uint16_t file_name_len;
    uint8_t *file_name;
    uint32_t file_len;

    uint32_t data_len;
    uint8_t *data;
} ble_watchface_photo_preview_file_download_ind_t;

typedef struct
{
    uint32_t event;
    uint16_t error_type;
} ble_watchface_error_ind_t;

typedef struct
{
    uint32_t event;
    uint8_t end_status;
} ble_watchface_file_end_ind_t;

typedef struct
{
    uint32_t event;
} ble_watchface_end_ind_t;


void watchface_register(watchface_callback callback);

void ble_watchface_send_start_rsp(uint16_t result);

void ble_watchface_file_start_rsp(uint16_t result);

void ble_watchface_send_start_rsp_file_info(uint16_t result, uint16_t block_length, uint32_t block_left);

void ble_watchface_file_download_rsp(uint16_t result);

void ble_watchface_file_end_rsp(uint16_t result);

void ble_watchface_end_rsp(uint16_t result);

uint8_t ble_watchface_abort();

uint8_t ble_watchface_get_state();

void ble_watchface_file_info_rsp(uint16_t result);

void ble_watchface_file_photo_preview_download_rsp(uint16_t result);

void ble_watchface_file_info_rsp_with_resume(uint16_t result, uint16_t resume_state, uint16_t resume_count);

/**
 * @brief Users can develop their own state checks
 */
uint16_t ble_watchface_customize_state_check();

#endif // _BF0_SIBLES_WATCHFACE_H

#endif // BSP_BLE_WATCH_FACE

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

