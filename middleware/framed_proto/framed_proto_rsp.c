/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file framed_proto_rsp.c
 * @brief Response builders of the framed message protocol.
 *
 * Every function here only lays a message out and hands it to the link. It
 * decides nothing: the values come from the caller. That keeps the wire format
 * in one place, shared by every product built on this channel, while the
 * question of *which* value to answer stays where the knowledge lives - the
 * firmware update engine, or a streaming application such as screen cast.
 */

#include <string.h>

#include "framed_proto.h"
#include "framed_proto_msg.h"

void framed_proto_rsp_version(uint16_t result, uint8_t need_reboot)
{
    dfu_version_response_t *rsp =
        (dfu_version_response_t *)framed_proto_msg_alloc(FRAMED_PROTO_VERSION_RESPONSE,
                sizeof(dfu_version_response_t));

    rsp->result = result;
    rsp->local_version = FRAMED_PROTO_CODE_VERSION;
    rsp->need_reboot = need_reboot;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_init(const framed_proto_rsp_file_init_t *p)
{
    dfu_file_init_rsp_ext_t *rsp =
        (dfu_file_init_rsp_ext_t *)framed_proto_msg_alloc(FRAMED_PROTO_FILE_INIT_RESPONSE,
                sizeof(dfu_file_init_rsp_ext_t));

    rsp->result = p->result;

    /* An answer that reports a failure carries no parameters; the peer must
     * not read them. Nothing else than the result is filled in that case. */
    if (p->result == FRAMED_PROTO_RESULT_OK)
    {
        rsp->local_version = FRAMED_PROTO_CODE_VERSION;
        rsp->resume_state = p->resume_state;
        rsp->resume_file_index = p->resume_index;
        rsp->fs_block = p->fs_block;
        rsp->num_of_rsp_file = p->num_of_rsp_file;
        rsp->num_of_rsp_bin = p->num_of_rsp_bin;
        rsp->packet_length_file = p->packet_length_file;
        rsp->packet_length_bin = p->packet_length_bin;
    }

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_init_completed(uint16_t result, uint32_t left_block)
{
    dfu_app_file_init_completed_rsp_t *rsp =
        (dfu_app_file_init_completed_rsp_t *)framed_proto_msg_alloc(
            FRAMED_PROTO_FILE_INIT_COMPLETED_RESPONSE, sizeof(dfu_app_file_init_completed_rsp_t));

    rsp->result = result;
    rsp->left_block = left_block;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_start_ctrl(uint16_t result, uint8_t resume_state,
                                      uint8_t num_of_rsp, uint32_t packet_index,
                                      uint16_t image_id)
{
    dfu_file_start_response_ctrl_ext_t *rsp =
        (dfu_file_start_response_ctrl_ext_t *)framed_proto_msg_alloc(
            FRAMED_PROTO_FILE_START_RESPONSE, sizeof(dfu_file_start_response_ctrl_ext_t));

    rsp->result = result;
    rsp->resume_state = resume_state;
    rsp->num_of_rsp_bin = num_of_rsp;
    rsp->packck_index = packet_index;
    rsp->image_id = image_id;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_start(uint16_t result)
{
    dfu_file_start_response_ext_t *rsp =
        (dfu_file_start_response_ext_t *)framed_proto_msg_alloc(FRAMED_PROTO_FILE_START_RESPONSE,
                sizeof(dfu_file_start_response_ext_t));

    rsp->result = result;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_packet(uint16_t result, uint32_t new_index)
{
    dfu_file_packet_response_ext_t *rsp =
        (dfu_file_packet_response_ext_t *)framed_proto_msg_alloc(
            FRAMED_PROTO_FILE_PACKET_RESPONSE, sizeof(dfu_file_packet_response_ext_t));

    rsp->result = result;
    rsp->new_index = new_index;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_end(uint16_t result)
{
    dfu_file_end_response_ext_t *rsp =
        (dfu_file_end_response_ext_t *)framed_proto_msg_alloc(FRAMED_PROTO_FILE_END_RESPONSE,
                sizeof(dfu_file_end_response_ext_t));

    rsp->result = result;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_file_total_end(uint16_t result)
{
    dfu_file_total_end_response_ext_t *rsp =
        (dfu_file_total_end_response_ext_t *)framed_proto_msg_alloc(
            FRAMED_PROTO_FILE_TOTAL_END_RESPONSE, sizeof(dfu_file_total_end_response_ext_t));

    rsp->result = result;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_continual_error(uint16_t error_type)
{
    dfu_file_continual_packet_error_t *rsp =
        (dfu_file_continual_packet_error_t *)framed_proto_msg_alloc(
            FRAMED_PROTO_FILE_CONTINUAL_PACKET_ERROR, sizeof(dfu_file_continual_packet_error_t));

    rsp->error_type = error_type;
    rsp->reserved_1 = 0;
    rsp->reserved_2 = 0;

    framed_proto_msg_send((uint8_t *)rsp);
}

void framed_proto_rsp_continual_mode(uint16_t mode, uint16_t reserved1, uint16_t reserved2)
{
    dfu_file_continual_packet_mode_change_t *rsp =
        (dfu_file_continual_packet_mode_change_t *)framed_proto_msg_alloc(
            FRAMED_PROTO_FILE_CONTINUAL_PACKET_MODE_CHANGE,
            sizeof(dfu_file_continual_packet_mode_change_t));

    rsp->mode = mode;
    rsp->reserved_1 = reserved1;
    rsp->reserved_2 = reserved2;

    framed_proto_msg_send((uint8_t *)rsp);
}
