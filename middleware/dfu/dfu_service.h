/**
  ******************************************************************************
  * @file   dfu_service.h
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


#ifndef __DFU_SERVICE_H
#define __DFU_SERVICE_H

#include <stdio.h>
#include <stdint.h>
#include "dfu_protocol.h"

typedef enum
{
    DFU_APP_START_REQUEST,
    DFU_APP_RESUME_REQUEST,
    DFU_APP_RESUME_RESPONSE,
    DFU_APP_IMAGE_DL_START_IND,
    DFU_APP_IMAGE_DL_ROPGRESS_IND,
    DFU_APP_IMAGE_DL_COMPLETED_IND,
    DFU_APP_DL_END_AND_INSTALL_START_IND,
    DFU_APP_INSTALL_PROGRESS_IND,
    DFU_APP_INSTALL_COMPLETD_IND,
    DFU_APP_ERROR_DISCONNECT_IND,
    DFU_APP_RESET_IND,
    DFU_APP_TO_USER,
    DFU_APP_RES_INIT_REQUEST,
    DFU_APP_RES_INIT_COMPLETED,
    DFU_APP_RES_FILE_START_IND,
    DFU_APP_RES_FILE_DATA_IND,
    DFU_APP_RES_FILE_END_IND,
    DFU_APP_RES_FILE_TOTAL_END_IND,
    DFU_APP_REBOOT_INSTALL_IND,
} dfu_app_event_t;

typedef enum
{
    OTA_APP_NORMAL = 0,
    OTA_APP_RES_OVERWRITE_ABNORMAL,
    OTA_APP_WHOLE_BIN_ABNORMAL,
    OTA_APP_INSTALL_IMG_FAIL,
    OTA_APP_INSTALL_RES_FAIL,
    OTA_APP_FILE_LIST_RENAME_FAIL,
    OTA_APP_RES_WAIT_INSTALL,
} ota_app_mode_t;

typedef enum
{
    DFU_APP_SUCCESSED,
    DFU_APP_FAIED,
} dfu_app_status_t;

typedef enum
{
    DFU_EVENT_SUCCESSED,
    DFU_EVENT_POSTPONE,
    DFU_EVENT_FAILED,
} dfu_event_ack_t;

typedef enum
{
    DFU_APP_RESUME_RESTART,
    DFU_APP_RESUME_USE_BLE,
    DFU_APP_RESUME_USE_APP,
} dfu_app_resume_t;


typedef dfu_event_ack_t (*dfu_callback)(uint16_t event, void *param);



typedef dfu_event_ack_t (*dfu_callback_ext)(uint16_t event, uint16_t len, void *param);


typedef struct
{
    dfu_id_t dfu_id;
    uint8_t is_boot;
} dfu_app_start_request_t;

typedef struct
{
    uint16_t event;
    dfu_id_t dfu_id;
    uint8_t is_nand_overwrite;
} dfu_app_start_request_ext_t;

typedef struct
{
    dfu_id_t dfu_id;
    uint8_t is_boot;
} dfu_app_resume_request_t;


typedef struct
{
    uint32_t total_imgs_num;
    uint32_t curr_img_total_len;
    uint8_t curr_img_id;
} dfu_app_img_dl_start_ind_t;

typedef struct
{
    uint16_t event;
    uint32_t total_imgs_num;
    uint32_t curr_img_total_len;
    uint8_t curr_img_id;
} dfu_app_img_dl_start_ind_ext_t;


typedef struct
{
    uint8_t img_id;
    uint32_t curr_img_recv_length;
} dfu_app_img_dl_progress_ind_t;

typedef struct
{
    uint16_t event;
    uint8_t img_id;
    uint32_t curr_img_recv_length;
} dfu_app_img_dl_progress_ind_ext_t;


typedef struct
{
    uint8_t img_id;
} dfu_app_img_dl_completed_ind_t;

typedef struct
{
    uint16_t event;
    uint8_t img_id;
} dfu_app_img_dl_completed_ind_ext_t;


typedef struct
{
    uint32_t total_imgs_len;
} dfu_app_img_install_start_ind_t;

typedef struct
{
    uint16_t event;
    uint32_t total_imgs_len;
} dfu_app_img_install_start_ind_ext_t;


typedef struct
{
    dfu_app_status_t result;
} dfu_app_img_install_completed_ind_t;

typedef struct
{
    uint16_t event;
    dfu_app_status_t result;
    uint8_t include_hcpu;
} dfu_app_img_install_completed_ind_ext_t;


typedef struct
{
    uint8_t img_id;
    uint8_t curr_state;
    uint8_t ota_mode;
} dfu_app_error_disconnect_ind_t;

uint8_t dfu_ctrl_reset_handler(void);

void dfu_register(dfu_callback callback);

void dfu_register_ext(dfu_callback_ext callback);

void dfu_respond_start_request(dfu_event_ack_t result);

void run_img(uint8_t *dest);


typedef struct
{
    uint16_t event;
    uint32_t file_count;
    uint32_t file_size;
    uint16_t version_len;
    // 0 is not resume, 1 is resume
    uint8_t resume_status;
    uint32_t resume_count;
    uint8_t *version;
} dfu_file_init_ind_t;

typedef struct
{
    // init response result
    uint8_t result;
    // see @dfu_app_resume_t
    uint8_t resume_command;

    uint16_t fs_block;

    uint32_t resume_count;
    uint32_t resume_length;
} dfu_file_init_response_resume_info_t;

typedef struct
{
    uint16_t event;
    uint8_t resume_result;
    uint32_t remote_block;
} dfu_file_init_completed_t;


typedef struct
{
    uint16_t event;
    uint32_t file_len;
    uint16_t file_name_len;
    uint8_t *file_name;
} dfu_file_start_ind_t;

typedef struct
{
    uint16_t event;
    uint16_t data_len;
    uint8_t *data;
} dfu_file_data_ind_t;

typedef struct
{
    uint16_t event;
} dfu_file_end_ind_t;

typedef struct
{
    uint16_t event;
    uint16_t hcpu_upgrade;
} dfu_file_total_end_ind_t;

typedef struct
{
    uint16_t event;
} dfu_file_error_ind_t;

typedef struct
{
    uint32_t current_len;
    uint32_t total_len;
} dfu_app_img_install_progress_ind_t;
void dfu_file_init_response(dfu_file_init_response_resume_info_t *info);

void dfu_file_start_response(uint8_t result);

void dfu_file_packet_response(uint8_t result, uint16_t is_last);

void dfu_file_end_response(uint8_t result);

void dfu_file_total_end_reponse(uint8_t result);

void dfu_ctrl_set_mode(uint8_t mode);

#endif //__DFU_SERVICE_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
