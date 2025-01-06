/**
  ******************************************************************************
  * @file   dfu_port_srv.c
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "board.h"
#include "os_adaptor.h"

#include "dfu_port_srv.h"

#include "bf0_sibles.h"
#include "bf0_sibles_serial_trans_service.h"
#include "bf0_ble_gap.h"


#ifdef BSP_USING_BLE_DFU_PORT_SVC


#define BLE_DFU_CATEID 0x01

static OS_THREAD_DECLAR(g_dfu_srv_tid);
static OS_MESSAGE_QUEUE_DECLAR(g_dfu_srv_mq);

static datas_handle_t g_dfu_srv_hdl;
static uint8_t client_num;

#define DFU_SERVICE_TASK_STACK_SIZE  (4096)

#ifdef SOC_BF0_HCPU
    ALIGN(RT_ALIGN_SIZE)
    static uint8_t dfu_service_stack[DFU_SERVICE_TASK_STACK_SIZE];
#endif /* SOC_BF0_HCPU */

typedef struct
{
    uint8_t is_power_on;
    uint8_t is_connected;
    uint8_t conn_idx;
    uint8_t reboot_after_disconnect;
    data_msg_t msg;
} dfu_port_svc_env_t;


static dfu_port_svc_env_t g_dfu_port_srv;

static dfu_port_svc_env_t *dfu_port_svc_get_env(void)
{
    return &g_dfu_port_srv;
}


void ble_dfu_service_serial_callback(uint8_t event, uint8_t *data)
{
    if (!data || !client_num)
        return;
    ble_dfu_service_data_t *data1 = NULL;
    uint16_t len = 0;
    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
    {
        len = sizeof(ble_serial_open_t);
        data1 = malloc(len + sizeof(ble_dfu_service_data_t));
        OS_ASSERT(data1);
        data1->event = BLE_SERIAL_TRAN_OPEN;
        data1->len = len;
        memcpy(data1->data, data, len);
    }
    break;
    case BLE_SERIAL_TRAN_DATA:
    {
        ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)data;
        len = sizeof(ble_serial_tran_data_t) + t_data->len;
        data1 = malloc(len + sizeof(ble_dfu_service_data_t));
        OS_ASSERT(data1);
        data1->event = BLE_SERIAL_TRAN_DATA;
        data1->len = len;
        memcpy(data1->data, t_data, sizeof(ble_serial_tran_data_t));
        memcpy(data1->data + sizeof(ble_serial_tran_data_t), t_data->data, t_data->len);
    }
    break;
    case BLE_SERIAL_TRAN_CLOSE:
    {
        len = sizeof(ble_serial_close_t);
        data1 = malloc(len + sizeof(ble_dfu_service_data_t));
        OS_ASSERT(data1);
        data1->event = BLE_SERIAL_TRAN_CLOSE;
        data1->len = len;
        memcpy(data1->data, data, len);
    }
    break;
    default:
        break;
    }

    if (data1)
    {
        //
        datas_push_data_to_client(g_dfu_srv_hdl, len + sizeof(ble_dfu_service_data_t), (uint8_t *)data1);
        free(data1);
    }
}

#ifdef BSP_BLE_SIBLES
    BLE_SERIAL_TRAN_EXPORT(BLE_DFU_CATEID, ble_dfu_service_serial_callback);
#endif // BSP_BLE_SIBLES

int ble_dfu_protocol_svc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    dfu_port_svc_env_t *env = dfu_port_svc_get_env();

    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        env->is_power_on = 1;
        env->is_connected = 0;
        env->reboot_after_disconnect = 0;
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->is_connected = 1;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        env->is_connected = 0;
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;

        if (env->reboot_after_disconnect == 1)
        {

            env->reboot_after_disconnect = 0;

            // reboot on hcpu
            //HAL_PMU_Reboot();

            data_msg_t msg = env->msg;
            datas_send_response(g_dfu_srv_hdl, &msg, 0);
        }
        break;
    }
    default:
        break;
    }
    return 0;

}


#ifdef BSP_BLE_SIBLES
    BLE_EVENT_REGISTER(ble_dfu_protocol_svc_event_handler, NULL);
#endif // BSP_BLE_SIBLES

void dfu_port_svc_disconnect()
{
    dfu_port_svc_env_t *env = dfu_port_svc_get_env();
    ble_gap_disconnect_t conn;
    conn.conn_idx = env->conn_idx;
    conn.reason = 0x16;
    ble_gap_disconnect(&conn);
}

void dfu_port_svc_set_reboot_after_disconnect()
{
    dfu_port_svc_env_t *env = dfu_port_svc_get_env();
    if (env->is_connected == 0)
    {
        data_msg_t msg = env->msg;
        datas_send_response(g_dfu_srv_hdl, &msg, 0);
    }
    env->reboot_after_disconnect = 1;
}



static int32_t dfu_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{

    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        // Just support one subscriber, do nothing.
        client_num++;
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        // Just support one subscriber, do nothing.
        client_num--;
        break;
    }
    case BLE_DFU_SEND_DATA:
    {
        ble_dfu_service_send_data_t *data = (ble_dfu_service_send_data_t *)(data_service_get_msg_body(msg));
        ble_serial_tran_data_t t_data;
        t_data.cate_id = data->cate_id;
        t_data.handle = data->handle;
        t_data.len = data->len;
        t_data.data = data->data;
        ble_serial_tran_send_data(&t_data);
        break;
    }
    case BLE_DFU_REBOOT_AFTER_DISCONNECT:
    {
        dfu_port_svc_env_t *env = dfu_port_svc_get_env();
        env->msg = *msg;
        dfu_port_svc_set_reboot_after_disconnect();
        break;
    }
    case BLE_DFU_REBOOT_DISCONNECT:
    {
        dfu_port_svc_disconnect();
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }
    return 0;
}


static data_service_config_t dfu_service_cb =
{
    .max_client_num = 1,
    .queue = RT_NULL,
    .msg_handler = dfu_service_msg_handler,
};



int dfu_service_register(void)
{
    os_message_queue_create(g_dfu_srv_mq, 15, sizeof(data_msg_t), NULL, 0);
    dfu_service_cb.queue = (rt_mq_t)(((os_handle_t)g_dfu_srv_mq)->handle);
#ifdef SOC_BF0_HCPU
    os_thread_create(g_dfu_srv_tid, data_service_entry, dfu_service_cb.queue, dfu_service_stack, sizeof(dfu_service_stack), RT_MAIN_THREAD_PRIORITY + 2, 10);
#else
    os_thread_create(g_dfu_srv_tid, data_service_entry, dfu_service_cb.queue, NULL, DFU_SERVICE_TASK_STACK_SIZE, RT_MAIN_THREAD_PRIORITY + 2, 10);
#endif
    g_dfu_srv_hdl = datas_register("DFUS", &dfu_service_cb);

    return 0;

}

#ifdef BSP_BLE_SIBLES
    INIT_COMPONENT_EXPORT(dfu_service_register);
#endif // BSP_BLE_SIBLES

#endif // BSP_USING_BLE_DFU_PORT_SVC

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

