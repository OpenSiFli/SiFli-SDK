/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file framed_proto_msg.h
 * @brief Wire layout of every protocol message.
 *
 * Definitions shared byte for byte with the phone app. No storage concept and
 * no product policy here, only layout.
 */

#ifndef __FRAMED_PROTO_MSG_H
#define __FRAMED_PROTO_MSG_H

#include <stdint.h>

#ifndef DFU_MD5_LEN
    #define DFU_MD5_LEN                 16
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t error_type;
    uint16_t reserved_1;
    uint16_t reserved_2;
} dfu_file_continual_packet_error_t;

typedef struct
{
    uint16_t mode;
    uint16_t reserved_1;
    uint16_t reserved_2;
} dfu_file_continual_packet_mode_change_t;

typedef struct
{
    uint16_t ota_version;
    uint16_t type;
} dfu_version_request_t;

typedef struct
{
    uint16_t result;
    uint16_t local_version;
    uint8_t need_reboot;
} dfu_version_response_t;

typedef struct
{
    uint8_t is_reboot;
} dfu_reboot_command_t;

typedef struct
{
    uint8_t phone_type;
    uint8_t file_align;
    uint16_t ota_version;
    uint16_t type;
    uint32_t file_count;
    uint32_t file_length; // control packets excluded
    uint8_t md5[DFU_MD5_LEN];
} dfu_file_init_req_ext_t;

typedef struct
{
    uint16_t result;
    uint16_t local_version;
    uint8_t num_of_rsp_file;
    uint8_t num_of_rsp_bin;
    uint16_t resume_state;
    uint32_t resume_file_index;
    uint16_t fs_block;
    uint16_t packet_length_file;
    uint16_t packet_length_bin;
} dfu_file_init_rsp_ext_t;

typedef struct
{
    uint8_t resume_result;
    uint8_t res_file_with_path;
    uint16_t file_name_len;
    uint32_t block_size;
    uint8_t file_name[0];
} dfu_file_init_completed_req_t;

typedef struct
{
    uint16_t file_type;

    // resource files only
    uint16_t file_index;
    uint16_t file_name_length;
    uint32_t already_send_length;
    uint8_t *file_name;

    // image files only
    uint8_t resume_state;
    uint16_t image_id;
    uint32_t target_address;

    // shared by both
    uint32_t file_length;
    uint32_t file_packet_count;
} dfu_file_start_request_ext_t;

typedef struct
{
    uint16_t result;
} dfu_file_start_response_ext_t;

typedef struct
{
    uint16_t result;
    uint8_t resume_state;
    uint8_t num_of_rsp_bin;
    uint32_t packck_index;
    uint16_t image_id;
} dfu_file_start_response_ctrl_ext_t;

typedef struct
{
    uint32_t packet_index;
    uint16_t image_id_or_file_index;
    uint16_t data_length;
    uint8_t file_type;
    uint8_t packet[0];
} dfu_file_packet_ext_t;

typedef struct
{
    uint32_t file_length;
    uint32_t index;
    uint16_t data_length;
    uint8_t type;
    uint8_t reserved;
    uint8_t data[0];
} dfu_file_continual_packet_t;

typedef struct
{
    uint16_t result;
    uint32_t new_index;
} dfu_file_packet_response_ext_t;

#if defined(__cplusplus)
static_assert(sizeof(dfu_file_packet_response_ext_t) == 8,
              "dfu_file_packet_response_ext_t wire layout changed");
#else
_Static_assert(sizeof(dfu_file_packet_response_ext_t) == 8,
               "dfu_file_packet_response_ext_t wire layout changed");
#endif

typedef struct
{
    uint8_t more_image;
    uint8_t file_type;
    uint16_t image_id_or_file_index;
} dfu_file_end_request_ext_t;

typedef struct
{
    uint16_t result;
} dfu_file_end_response_ext_t;

typedef struct
{
    uint16_t result;
} dfu_file_total_end_response_ext_t;

typedef struct
{
    uint16_t result;
} dfu_ctrl_abort_command_t;

typedef struct
{
    uint16_t result;
    uint16_t reserved;
    uint32_t left_block;
} dfu_app_file_init_completed_rsp_t;

#ifdef __cplusplus
}
#endif

#endif /* __FRAMED_PROTO_MSG_H */