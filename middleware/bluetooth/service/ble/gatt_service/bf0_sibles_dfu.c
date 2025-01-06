/**
  ******************************************************************************
  * @file   bf0_sibles_dfu.c
  * @author Sifli software development team
  * @brief Sibles ble DFU service source.
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

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "os_adaptor.h"
#include "bf0_sibles.h"
#include "bf0_sibles_serial_trans_service.h"
#if 0
#ifdef BSP_USING_DFU
#include "dfu.h"

#include "bf0_ble_dfu.h"
#include "bf0_sibles_dfu_internal.h"
#define LOG_TAG "ble_dfu"
#include "log.h"

static ble_dfu_env_t g_ble_dfu_env;

static ble_dfu_env_t *ble_dfu_get_env(void)
{
    return &g_ble_dfu_env;
}



void ble_dfu_register(ble_dfu_callback callback)
{
    ble_dfu_env_t *env = ble_dfu_get_env();
    env->callback = callback;
}

void ble_dfu_respond_start_request(ble_dfu_start_request_response_t *rsp)
{
    ble_dfu_packet_send_rsp_t packet_rsp;
    ble_dfu_env_t *env = ble_dfu_get_env();

    packet_rsp.result = rsp->result;
    packet_rsp.type = BLE_DFU_TYPE_PACKET_SEND_RSP;
    if (rsp->result == 0)
        env->state = BLE_DFU_STATE_RECEVING;
    // respond to peer device
    ble_serial_tran_data_t rsp_header;
    rsp_header.cate_id = BLE_DFU_CATEID;
    rsp_header.len = sizeof(ble_dfu_packet_send_rsp_t);
    rsp_header.data = (uint8_t *)&packet_rsp;
    ble_serial_tran_send_data(env->conn_idx, &rsp_header);

}

static int ble_dfu_process_header(ble_dfu_packet_t *packet)
{
    ble_dfu_env_t *env = ble_dfu_get_env();
    uint8_t *header_data;
    uint16_t over_len;
    int ret = 0;

    over_len = packet->size + env->data.processed_len - BLE_DFU_HEADER_LEN;
    header_data = rt_malloc(sizeof(struct dfu_hdr) + BLE_DFU_HEADER_LEN);
    //LOG_I("ble_dfu_process_header processed %d, overlen %d, ", env->data.processed_len, over_len);

    rt_memcpy(&env->data.dfu_data[env->data.processed_len], packet->packet, BLE_DFU_HEADER_LEN - env->data.processed_len);

    *header_data = DFU_IMG_HDR_ENC; //command
    *(header_data + 1) = BLE_DFU_HCPU_FLAG;
    rt_memcpy(&header_data[sizeof(struct dfu_hdr)], env->data.dfu_data, BLE_DFU_HEADER_LEN);
#ifdef BSP_USING_DFU_COMPRESS
    ret = dfu_receive_pkt(BLE_DFU_HEADER_LEN + sizeof(struct dfu_hdr), header_data);
#endif
    LOG_I("ble_dfu_process_header ret: (%d)", ret);

    rt_free(header_data);

    LOG_I("ble_dfu_process_header overlen: (%d)", over_len);
    rt_memcpy(env->data.dfu_data, packet->packet + (packet->size - over_len), over_len);
    env->data.processed_len = over_len;
    return ret;
}

static int ble_dfu_process_body(ble_dfu_packet_t *packet)
{
    uint8_t *body_data, *temp_data;
    int ret = 0;

    ble_dfu_env_t *env = ble_dfu_get_env();
    if (env->data.processed_len + packet->size < BLE_DFU_BODY_LEN)
    {
        rt_memcpy(&env->data.dfu_data[env->data.processed_len], packet->packet, packet->size);
        env->data.processed_len += packet->size;
    }
    else
    {

        // TODO: env->data have multiple body
        rt_memcpy(&env->data.dfu_data[env->data.processed_len], packet->packet, BLE_DFU_BODY_LEN - env->data.processed_len);

        temp_data = rt_malloc(env->data.processed_len + packet->size);
        rt_memcpy(temp_data, env->data.dfu_data, env->data.processed_len);
        rt_memcpy(temp_data + env->data.processed_len, packet->packet, packet->size);

        uint16_t over_len;
        over_len = packet->size + env->data.processed_len - BLE_DFU_BODY_LEN;
        body_data = rt_malloc(sizeof(struct dfu_hdr) + BLE_DFU_BODY_LEN);
        *body_data = DFU_IMG_BODY_ENC;
        *(body_data + 1) = BLE_DFU_HCPU_FLAG;
        rt_memcpy(&body_data[sizeof(struct dfu_hdr)], temp_data, BLE_DFU_BODY_LEN);
#ifdef BSP_USING_DFU_COMPRESS
        ret = dfu_receive_pkt(BLE_DFU_BODY_LEN + sizeof(struct dfu_hdr), body_data);
#endif
        rt_free(body_data);
        rt_memcpy(env->data.dfu_data, temp_data + BLE_DFU_BODY_LEN, over_len);
        rt_free(temp_data);

        env->data.processed_len = over_len;
    }
    return ret;
}

static int ble_dfu_process_end(ble_dfu_packet_t *packet)
{
    ble_dfu_env_t *env = ble_dfu_get_env();
    LOG_I("ble_dfu_process_end, remain %d, new %d", env->data.processed_len, packet->size);
    uint16_t over_len;
    uint8_t *last_body;
    int ret = 0;

    if (packet->size + env->data.processed_len >= BLE_DFU_BODY_LEN)
    {
        ble_dfu_process_body(packet);
    }
    else
    {
        rt_memcpy(&env->data.dfu_data[env->data.processed_len], packet->packet, packet->size);
        env->data.processed_len += packet->size;
    }

    if (env->data.processed_len != 0)
    {
        last_body = rt_malloc(sizeof(struct dfu_hdr) + env->data.processed_len);
        *last_body = DFU_IMG_BODY_ENC;
        *(last_body + 1) = BLE_DFU_HCPU_FLAG;
        rt_memcpy(&last_body[sizeof(struct dfu_hdr)], env->data.dfu_data, env->data.processed_len);
#ifdef BSP_USING_DFU_COMPRESS
        ret = dfu_receive_pkt(env->data.processed_len + sizeof(struct dfu_hdr), last_body);
#endif
        LOG_I("ble_dfu_process_last body %d", ret);
        rt_free(last_body);
    }
    return ret;
}

static int ble_dfu_send_end()
{
    uint8_t *end_data;
    int ret = 0;

    // send dfu end
    end_data = rt_malloc(sizeof(struct dfu_hdr));
    *end_data = DFU_END;
    *(end_data + 1) = BLE_DFU_HCPU_FLAG;
#ifdef BSP_USING_DFU_COMPRESS
    ret = dfu_receive_pkt(sizeof(struct dfu_hdr), end_data);
#endif
    LOG_I("ble_dfu_send_end %d", ret);
    rt_free(end_data);
    return ret;
}


void ble_dfu_serial_callback(uint8_t conn_idx, ble_serial_tran_data_t *data)
{
    if (data->cate_id != BLE_DFU_CATEID)
        return;
    ble_dfu_env_t *env = ble_dfu_get_env();

    uint8_t type = *data->data;
    LOG_I("dfu,type(%d), data(%x,%x,%x,%x)\r\n", type, *(data->data + 1), *(data->data + 2), *(data->data + 3), *(data->data + 4));

    switch (type)
    {
    case BLE_DFU_TYPE_VER_INQUIRY:
    {
        // remote device confirm wthether need upgrade via local version.
        ble_dfu_ver_inquiry_t *inq = (ble_dfu_ver_inquiry_t *)(data->data + 1);
        ble_dfu_ver_inquiry_rsp_t *rsp = malloc(sizeof(ble_dfu_ver_inquiry_rsp_t) +  inq->img_count * sizeof(ble_dfu_ver_info_t));
        OS_ASSERT(rsp);
        uint8_t img_count = 0;
        rsp->msg_type = BLE_DFU_TYPE_VER_INQUIRY_RSP;
        rsp->img_count = inq->img_count;
        while (img_count < inq->img_count)
        {
            struct image_header_enc *hdr = dfu_get_img_info(inq->img_id[img_count]);
            rsp->info[img_count].img_id = inq->img_id[img_count];
            if (hdr->length)
            {
#ifdef BSP_USING_DFU_COMPRESS
                // only support download one img one time.
                uint32_t i = g_dfu_compress_config->img_count;
                while (i--)
                {
                    if (g_dfu_compress_config->imgs[i].compress_img_id == inq->img_id[img_count])
                    {
                        rsp->info[img_count].img_state = g_dfu_compress_config->imgs[i].state;

                        strncpy((char *)rsp->info[img_count].img_target_ver, (const char
                                *)g_dfu_compress_config->imgs[i].img.enc_img.ver, DFU_VERSION_LEN);
                        // If downloaded before, otherwise the current_img_len should be 0
                        rsp->info[img_count].img_target_len = g_dfu_compress_config->imgs[i].current_img_len;
                    }
                }
#endif
                strncpy((char *)rsp->info[img_count].img_curr_ver, (const char *)hdr->ver, DFU_VERSION_LEN);
            }
            else
            {
                rsp->info[img_count].img_state = 0;
            }
            img_count++;

        }
        env->conn_idx = conn_idx;
        ble_serial_tran_data_t rsp_header;
        rsp_header.cate_id = BLE_DFU_CATEID;
        rsp_header.len = sizeof(ble_dfu_ver_inquiry_rsp_t) + inq->img_count * sizeof(ble_dfu_ver_info_t);
        rsp_header.data = (uint8_t *)rsp;
        int ret = ble_serial_tran_send_data(conn_idx, &rsp_header);
        LOG_I("inq rsp ret %d\r\n", ret);
        break;
    }
    case BLE_DFU_TYPE_NEG_REQ:
    {
        ble_dfu_neg_req_t *req = (ble_dfu_neg_req_t *)(data->data + 1);
        ble_dfu_neg_rsp_t rsp;
        LOG_I("neg len %d\r\n", req->packet_size);
        if (req->packet_size <= BLE_DFU_PACKET_SIZE)
        {
            rsp.result = 0;
            rsp.packet_size = req->packet_size;
            rsp.type = BLE_DFU_TYPE_NEG_RSP;
            // Negotiating successfully.
            env->info.packet_size = req->packet_size;
            env->state = BLE_DFU_STATE_READY;
        }
        else
        {
            rsp.result = 1;// Not allowed
            rsp.type = BLE_DFU_TYPE_NEG_RSP;

            rsp.packet_size = BLE_DFU_PACKET_SIZE;
        }
        ble_serial_tran_data_t rsp_header;
        rsp_header.cate_id = BLE_DFU_CATEID;
        rsp_header.len = sizeof(ble_dfu_neg_rsp_t);
        rsp_header.data = (uint8_t *)&rsp;
        int ret = ble_serial_tran_send_data(conn_idx, &rsp_header);
        LOG_I("ret %d\r\n", ret);

        // respond peer device
        break;
    }
    case BLE_DFU_TYPE_RESUME_REQ:
    {
        ble_dfu_resume_rsp_t rsp;
        int ret;
#ifdef BSP_USING_DFU_COMPRESS
        ble_dfu_resume_req_t *req = (ble_dfu_resume_req_t *)(data->data + 1);
        ret = dfu_receive_resume(req->flashid, (uint8_t *)&req->data, sizeof(struct image_header_compress_resume));

        if (ret != DFU_SUCCESS)
            rsp.result = 1;
        else
            env->state = BLE_DFU_STATE_READY;
#else
        rsp.result = 2;
#endif
        ble_serial_tran_data_t rsp_header;
        rsp_header.cate_id = BLE_DFU_CATEID;
        rsp_header.len = sizeof(ble_dfu_resume_rsp_t);
        rsp_header.data = (uint8_t *)&rsp;
        ret = ble_serial_tran_send_data(conn_idx, &rsp_header);
        LOG_I("resume ret %d\r\n", ret);

        break;
    }
    case BLE_DFU_TYPE_PACKET_SEND_REQ:
    {
        ble_dfu_packet_send_req_t *req = (ble_dfu_packet_send_req_t *)(data->data + 1);
        LOG_I("packet req len(%d)\r\n", req->total_len);
        ble_dfu_packet_send_rsp_t rsp;
        ble_dfu_start_request_t app_req;
        uint8_t ret;
        app_req.img_id = req->img_id;
        app_req.img_size = req->total_len;
        rsp.result = 0;
        if (env->callback)
        {
            ret = env->callback(BLE_DFU_START_REQUEST, &app_req);

            if (ret == BLE_DFU_EVENT_POSTPONE || ret == BLE_DFU_EVENT_SUCCESSED)
            {
                env->info.total_len = req->total_len;
                env->info.curr_len = 0;
                env->info.img_id = req->img_id;

                // alloc data to store packet image
                env->data.dfu_data = rt_malloc(BLE_DFU_BODY_LEN + sizeof(uint32_t));
                env->data.processed_len = 0;
                rt_memset(env->data.dfu_data, 0, BLE_DFU_BODY_LEN);

                if (ret == BLE_DFU_EVENT_POSTPONE)
                    break;
            }
            if (ret == BLE_DFU_EVENT_FAILED)
                rsp.result = 1;
        }
        rsp.type = BLE_DFU_TYPE_PACKET_SEND_RSP;
        env->state = BLE_DFU_STATE_RECEVING;
        // else don;t allowed

        // respond to peer device
        ble_serial_tran_data_t rsp_header;
        rsp_header.cate_id = BLE_DFU_CATEID;
        rsp_header.len = sizeof(ble_dfu_packet_send_rsp_t);
        rsp_header.data = (uint8_t *)&rsp;
        ble_serial_tran_send_data(conn_idx, &rsp_header);

        if (ret == BLE_DFU_EVENT_FAILED)
        {
            env->state = BLE_DFU_STATE_IDLE;
            memset(&env->info, 0, sizeof(ble_dfu_info_t));
        }

        break;
    }
    case BLE_DFU_TYPE_PACKET:
    {
        uint8_t result = 0;
        ble_dfu_packet_t *packet = (ble_dfu_packet_t *)(data->data + 1);
        LOG_I("packet count(%d), len(%d)\r\n", packet->count, packet->size);
        int ret;
        if (env->state != BLE_DFU_STATE_RECEVING)
        {
            // Should not handle
            result = 1;
        }
        else if (env->info.total_len < env->info.curr_len + packet->size)
        {
            // Should terminte the DFU.
            //result = 2;
        }
        else if ((env->info.curr_count + 1) != packet->count)
        {
            // Received unexpected packet;
            LOG_I("curr count(%d)\r\n", env->info.curr_count);
            //result = 3;
        }
        if (result == 0)
        {
            env->info.curr_count++;
            env->info.curr_len += packet->size;

            // TODO: receive
            if (packet->size < env->info.packet_size)
            {
                // last packet
                LOG_I("dfu last packet");
                ret = ble_dfu_process_end(packet);
            }
            else if (env->info.curr_len < BLE_DFU_HEADER_LEN)
            {
                // header len not enough
                rt_memcpy(&env->data.dfu_data[env->data.processed_len], packet->packet, packet->size);
                env->data.processed_len += packet->size;
                LOG_I("header not enough, processed %d", env->data.processed_len);
            }
            else if (env->info.curr_len - packet->size >= BLE_DFU_HEADER_LEN)
            {
                // header processed, should process body
                ret = ble_dfu_process_body(packet);
            }
            else
            {
                LOG_I("process header %d, %d", env->info.curr_len, env->data.processed_len);
                ret = ble_dfu_process_header(packet);
            }
#ifdef BSP_USING_DFU
            //ret = dfu_receive_pkt(packet->size, packet->packet);
            LOG_I("curr count(%d), current_len(%d), result(%d)\r\n",
                  env->info.curr_count, env->info.curr_len, ret);

            if ((env->info.curr_count % 4) && env->info.curr_len < env->info.total_len)
            {
                // Only respond every 4 packet.
                //break;
            }
            if (ret == DFU_FAIL)
                result = 4;
#endif
        }

        if (result != 0)
        {
            // Stopped
            ble_dfu_end_t end;
            end.result = result;
            if (env->callback)
                env->callback(BLE_DFU_END, &end);
            env->state = BLE_DFU_STATE_IDLE;
            memset(&env->info, 0, sizeof(ble_dfu_info_t));
        }
        else
        {
            ble_dfu_packet_recv_t pkt;
            pkt.percent = env->info.curr_len / env->info.total_len;
            if (env->callback)
                env->callback(BLE_DFU_PACKET_RECV, &pkt);
        }
        // Respond peer device the result.
        ble_serial_tran_data_t rsp_header;
        ble_dfu_packet_cfm_t cfm;
        rsp_header.cate_id = BLE_DFU_CATEID;
        rsp_header.len = sizeof(ble_dfu_packet_send_rsp_t);
        rsp_header.data = (uint8_t *)&cfm;
        cfm.result = result;
        cfm.type = BLE_DFU_TYPE_PACKET_CFM;
        ble_serial_tran_send_data(conn_idx, &rsp_header);
        break;
    }
    case BLE_DFU_TYPE_PACKET_SEND_END:
    {
        LOG_I("BLE DFU Ended\r\n");
        //env->state = BLE_DFU_STATE_READY;
        memset(&env->info, 0, sizeof(ble_dfu_info_t));
        // Stopped
        ble_dfu_end_t end;
        end.result = 0;
        if (env->callback)
            env->callback(BLE_DFU_END, &end);
        env->state = BLE_DFU_STATE_IDLE;
        memset(&env->info, 0, sizeof(ble_dfu_info_t));
        rt_free(env->data.dfu_data);
        ble_dfu_send_end();
        break;
    }
    default:
        break;
    }
}

BLE_SERIAL_TRAN_EXPORT(BLE_DFU_CATEID, ble_dfu_serial_callback);

#endif
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
