/**
  ******************************************************************************
  * @file   dfu_protocol.h
  * @author Sifli software development team
  * @brief Header file - Sibles serial transfer service.
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


#ifndef __DFU_PROTOCOL_H
#define __DFU_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>

#include "rtconfig.h"

#include "dfu.h"

#define OTA_CODE_VERSION 2

typedef enum
{
    DFU_ERR_NO_ERR,
    DFU_ERR_GENERAL_ERR,
    DFU_ERR_PARAMETER_INVALID,
    DFU_ERR_SPACE_NOT_ENOUGH,
    DFU_ERR_NOT_READY,
    DFU_ERR_HW_VER_ERR,
    DFU_ERR_SDK_VER_ERR,
    DFU_ERR_FW_VER_ERR,
    DFU_ERR_FW_INVALID,
    DFU_ERR_CONTROL_PACKET_INVALID,
    DFU_ERR_OTA_ONGOING,
    DFU_ERR_USER_REJECT,
    DFU_ERR_UNEXPECT_STATE, // remote should disconnect and init again
    DFU_ERR_INDEX_ERROR,
    DFU_ERR_UPDATING,
    DFU_ERR_MISS_PACKET,
    DFU_ERR_FLASH,
    DFU_ERR_INTERNAL = 0x40,
    DFU_ERR_POSTPONE = DFU_ERR_INTERNAL,
} dfu_err_type_t;

typedef enum
{
    DFU_INIT_REQUEST,
    DFU_INIT_RESPONSE,
    DFU_INIT_COMPLETED_IND,
    DFU_RESUME_REQUEST,
    DFU_RESUME_RESPONSE,
    DFU_RESUME_COMPLETED_IND,
    DFU_IMAGE_SEND_START,
    DFU_IMAGE_SEND_START_RESPONSE,
    DFU_IMAGE_SEND_END,
    DFU_IMAGE_SEND_END_RESPONSE,
    DFU_IMAGE_SEND_PACKET,
    DFU_IMAGE_SEND_PACKET_RESPONSE,
    DFU_TRANSMISSION_END,
    DFU_END_IND,
    DFU_FORCE_INIT_REQUEST,
    DFU_CONNECTION_PRIORITY_CHECK,
    DFU_RETRANSMISSION_REQUEST,
    DFU_RETRANSMISSION_RESPONSE,
    DFU_READ_VERSION_REQUEST = 18,
    DFU_READ_VERSION_RESPONSE,
    DFU_COMMAND_POWER_OFF,
    DFU_IMAGE_FILE_INIT_REQUEST = 21,
    DFU_IMAGE_FILE_INIT_RESPONSE,
    DFU_IMAGE_FILE_INIT_COMPLETED,
    DFU_IMAGE_FILE_START,
    DFU_IMAGE_FILE_START_RESPONSE,
    DFU_IMAGE_FILE_PACKET,
    DFU_IMAGE_FILE_PACKET_RESPONSE,
    DFU_IMAGE_FILE_END,
    DFU_IMAGE_FILE_END_RESPONSE,
    DFU_IMAGE_FILE_TOTAL_END,
    DFU_IMAGE_FILE_TOTAL_END_RESPONSE,
    DFU_INIT_REQUEST_EXT = 32,
    DFU_INIT_RESPONSE_EXT,
    DFU_INIT_COMPLETED_IND_EXT,
    DFU_LINK_LOSE_CHECK_REQ,
    DFU_LINK_LOSE_CHECK_RSP,
    DFU_ABORT_COMMAND = 37,
    DFU_IMAGE_OFFLINE_START_REQ = 38,
    DFU_IMAGE_OFFLINE_START_RSP = 39,
    DFU_IMAGE_OFFLINE_PACKET_REQ = 40,
    DFU_IMAGE_OFFLINE_PACKET_RSP = 41,
    DFU_IMAGE_OFFLINE_END_REQ = 42,
    DFU_IMAGE_OFFLINE_END_RSP = 43,
} dfu_protocol_msg_id_t;


typedef enum
{
    DFU_ID_CODE,
    DFU_ID_CODE_MIX,
    DFU_ID_OTA_MANAGER,
    DFU_ID_CODE_FULL_BACKUP,
    DFU_ID_CODE_BACKGROUND,
    DFU_ID_DL,
    DFU_ID_CODE_DOWNLOAD_IN_HCPU = 10,
    DFU_ID_CODE_RES_DOWNLOAD_IN_HCPU,
} dfu_id_t;

typedef enum
{
    DFU_DES_NONE,
    DFU_DES_RUNNING_ON_HCPU1,
    DFU_DES_RUNNING_ON_HCPU2,
    DFU_DES_SWITCH_TO_HCPU1,
    DFU_DES_SWITCH_TO_HCPU2,
    DFU_DES_UPDATE_HCPU1,
    DFU_DES_UPDATE_HCPU2,
} dfu_des;

#ifdef OTA_55X
typedef enum
{
    DFU_IMG_ID_HCPU,
    DFU_IMG_ID_LCPU,
    DFU_IMG_ID_PATCH,
    DFU_IMG_ID_RES,
    DFU_IMG_ID_FONT,
    DFU_IMG_ID_EX,
    DFU_IMG_ID_OTA_MANAGER,
    DFU_IMG_ID_TINY_FONT,
    DFU_IMG_ID_RES_UPGRADE,
    DFU_IMG_ID_PATCH_TEMP,
    DFU_IMG_ID_CTRL_PACKET = 10,
    DFU_IMG_ID_BOOTLOADER,
    DFU_IMG_ID_MAX = DFU_IMG_ID_BOOTLOADER,
} dfu_img_id_t;
#endif


#ifdef OTA_56X_NAND
typedef enum
{
    DFU_IMG_ID_NAND_HCPU,
    DFU_IMG_ID_LCPU,
    DFU_IMG_ID_NAND_HCPU_PATCH,
    DFU_IMG_ID_RES, //ROOT
    DFU_IMG_ID_LCPU_PATCH,
    DFU_IMG_ID_DYN,
    DFU_IMG_ID_MUSIC,
    DFU_IMG_ID_PIC,
    DFU_IMG_ID_FONT,
    DFU_IMG_ID_RING,
    DFU_IMG_ID_LANG,
    DFU_IMG_ID_MAX = DFU_IMG_ID_LANG,
} dfu_img_id_t;
#endif


typedef struct
{
    uint16_t message_id; /* ref @dfu_protocol_msg_id_t */
    uint16_t length;
    uint8_t data[0];
} dfu_tran_protocol_t;


/*
 * Fully control packet structure.
 * --------------------------------------------
 * |Hash(32B) | Control packet content | Signature(256B) |
 * -------------------------------------------
*/

typedef struct
{
    uint8_t dfu_ID;
    uint32_t HW_version;
    uint32_t SDK_version;
    uint32_t FW_version;
    uint8_t FW_key[DFU_KEY_SIZE];
    uint16_t image_header_len;
    uint8_t image_header[0];
} dfu_control_packet_t;

typedef struct
{
    uint8_t sig[DFU_SIG_SIZE];
    uint32_t length;
    uint16_t flag;
    uint8_t img_id;
} dfu_image_header_int_t;

typedef struct
{
    uint16_t blk_size;
    uint8_t img_count;
    dfu_image_header_int_t img_header[0];
} dfu_code_image_header_t;

typedef struct
{
    uint16_t result;
    uint8_t is_boot;
} dfu_init_response_t;

typedef struct
{
    uint8_t is_start;
} dfu_init_completed_ind_t;

typedef struct
{
    uint16_t result;
    uint8_t is_boot;
    uint8_t is_restart;
    uint32_t curr_packet_num;
    uint8_t curr_img;
    uint8_t num_of_rsp;
} dfu_resume_response_t;

typedef struct
{
    uint16_t result;
    uint8_t resume_status;
    uint8_t is_restart;
    uint32_t curr_packet_num;
    uint8_t curr_img;
    uint8_t num_of_rsp;
    uint8_t is_boot;
    uint8_t ver;
} dfu_init_response_ext_t;


typedef struct
{
    uint8_t is_start;
} dfu_resume_completed_ind_t;


typedef struct
{
    uint32_t img_length;
    uint32_t total_pkt_num;
    uint8_t num_of_rsp;
    uint8_t img_id;
} dfu_image_send_start_t;

typedef struct
{
    uint16_t result;
    uint8_t end_send;
    uint8_t extra;
} dfu_image_send_start_response_t;

typedef struct
{
    uint8_t img_id;
    uint8_t is_more_image;
} dfu_image_send_end_t;

typedef struct
{
    uint16_t result;
} dfu_image_send_end_response_t;


/*
 * Fully image packet structure.
 * -----------------------------
 * |Hash(32B) | packet content |
 * -----------------------------
*/

typedef struct
{
    uint8_t img_id;
    uint16_t pkt_idx;
    uint16_t size;
    uint8_t packet[0];
} dfu_image_send_packet_t;

typedef struct
{
    uint32_t pkt_idx;
    uint16_t img_id;
    uint16_t size;
    uint8_t packet[0];
} dfu_image_send_packet_v2_t;
typedef struct
{
    uint16_t result;
} dfu_image_send_packet_response_t;

typedef struct
{
    uint8_t reserved;
} dfu_tranmission_end_t;

typedef struct
{
    uint8_t result;
} dfu_end_int_t;

typedef struct
{
    uint16_t result;
    uint16_t new_num_of_rsp;
    uint32_t retransmission_packet_num;
} dfu_retransmission_response_t;

typedef struct
{
    uint16_t result;
    uint16_t len;
    uint8_t version[DFU_PART_VERSION_LEN];
} dfu_read_version_response_t;

typedef struct
{
    uint32_t file_count;
    uint32_t file_total_len;
    uint16_t phone_type;
    uint16_t version_len;
    uint8_t version[0];
} dfu_image_file_total_start_t;

typedef struct
{
    uint16_t result;
    uint16_t resume_status;
    uint32_t file_count;
    uint8_t ver;
    uint8_t reserved;
    uint16_t fs_block;
} dfu_image_file_init_response_t;

typedef struct
{
    uint8_t resume;
    uint32_t remote_block;
} dfu_image_file_init_completed_t;

typedef struct
{
    uint16_t total_file_count;
    uint16_t num_of_rsp;
    uint32_t file_length;
    uint16_t file_pkt_num;
    uint16_t file_name_len;
    uint8_t file_name[0];
} dfu_image_file_start_t;

typedef struct
{
    uint16_t result;
} dfu_image_file_start_response_t;

typedef struct
{
    uint16_t packet_index;
    uint16_t packet_length;
    uint8_t file_data[0];
} dfu_image_file_packet_t;

typedef struct
{
    uint16_t result;
    uint16_t new_num_of_rsp;
    uint32_t current_file_index;
} dfu_image_file_packet_response_t;

typedef struct
{
    uint16_t count;
} dfu_image_file_end_t;

typedef struct
{
    uint16_t result;
} dfu_image_file_end_response_t;

typedef struct
{
    uint16_t hcpu_upgrade;
} dfu_image_file_total_end_t;

typedef struct
{
    uint16_t result;
} dfu_image_file_total_end_response_t;

typedef struct
{
    uint16_t result;
    uint16_t new_num_of_rsp;
    uint32_t current_file_index;
} dfu_link_lose_check_req_t;

typedef struct
{
    uint16_t result;
} dfu_link_lose_check_rsp_t;

typedef struct
{
    uint32_t file_len;
    uint32_t packet_count;
    uint32_t crc_value;
} dfu_image_offline_start_req_t;

typedef struct
{
    uint16_t result;
    uint16_t reserved;
    uint32_t completed_count;
} dfu_image_offline_start_rsp_t;

typedef struct
{
    uint32_t packet_index;
    uint32_t data_len;
    uint32_t crc;
    uint8_t data[0];
} dfu_image_offline_packet_t;

typedef struct
{
    uint16_t result;
    uint8_t retransmission;
    uint8_t reserved;
    uint32_t completed_count;
} dfu_image_offline_packet_rsp_t;

typedef struct
{
    uint16_t reserved;
} dfu_image_offline_end_req_t;

typedef struct
{
    uint16_t result;
} dfu_image_offline_end_rsp_t;

typedef struct
{
    uint16_t reason;
} dfu_abort_command_t;

#endif //__DFU_PROTOCOL_H


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
