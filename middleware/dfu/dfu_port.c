/**
  ******************************************************************************
  * @file   dfu_port.c
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

#include "dfu_internal.h"

#ifdef BSP_BLE_SIBLES
    #include "bf0_sibles_internal.h"
    #include "bf0_sibles_serial_trans_service.h"
    #include "att.h"
    #include "bf0_ble_gap.h"
    #include "bf0_sibles.h"
    #include "bf0_ble_common.h"
    #include "bf0_sibles_advertising.h"
    #include "ble_connection_manager.h"
    #define BLE_DFU_CATEID 0x01
#elif defined(BSP_USING_DATA_SVC)
    #include "dfu_port_srv.h"
#endif

#define LOG_TAG "DFU_P"
#include "log.h"




typedef struct
{
    uint8_t is_open;
#if !defined(BSP_BLE_SIBLES) && defined(BSP_USING_DATA_SVC)
    /* Handle for serial tran data service. */
    uint8_t srv_handle;
    uint8_t is_subscribed;
#endif // BSP_USING_DATA_SVC
    /* Handle for serial tran service. */
    uint8_t handle;
    struct
    {
        uint8_t is_power_on;
        uint8_t is_connected;
        uint8_t conn_idx;
        uint8_t reboot_after_disconnect;
        uint8_t is_last_packet_send;
        uint8_t last_tx_packet;
        struct
        {
#ifdef BSP_BLE_SIBLES
            bd_addr_t peer_addr;
#endif
            uint16_t mtu;
            uint16_t conn_interval;
            uint8_t peer_addr_type;
        } conn_para;
        rt_sem_t sema_handle;
    } rc;

} dfu_protocol_port_env_t;


static OS_THREAD_DECLAR(g_dfu_tid);
static OS_SEM_DECLAR(g_dfu_sem);

static dfu_protocol_port_env_t g_dfu_protocol_port;

static dfu_protocol_port_env_t *dfu_protocol_get_env(void)
{
    return &g_dfu_protocol_port;
}

#if !defined(BSP_BLE_SIBLES) && defined(BSP_USING_DATA_SVC)
    static int8_t ble_dfu_service_send_data(dfu_protocol_port_env_t *env, ble_serial_tran_data_t *t_data);
#endif

__INLINE dfu_tran_protocol_t *dfu_packet2msg(void const *param_ptr)
{
    return (dfu_tran_protocol_t *)(((uint8_t *)param_ptr) - offsetof(dfu_tran_protocol_t, data));
}

#ifdef BSP_BLE_SIBLES

#ifdef DFU_OTA_MANAGER
    SIBLES_ADVERTISING_CONTEXT_DECLAR(g_dfu_advertising_context);
#endif // DFU_OTA_MANAGER

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}



#define DEFAULT_LOCAL_NAME "OTA"
/* Enable advertise via advertising service. */
static void dfu_protocol_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf(local_name, 31, "SIFLI_APP-%x-%x-%x-%x-%x-%x", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    //para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed. Due to name is too long to put adv data, put it to rsp data.*/
    para.rsp_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.rsp_data.completed_name->name, local_name, para.rsp_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_advertising_event;
#ifdef DFU_OTA_MANAGER
    ret = sibles_advertising_init(g_dfu_advertising_context, &para);
#endif // DFU_OTA_MANAGER

    if (ret == SIBLES_ADV_NO_ERR)
    {
#ifdef DFU_OTA_MANAGER
        sibles_advertising_start(g_dfu_advertising_context);
#endif // DFU_OTA_MANAGER
    }

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}

void dfu_protocol_reset_entry(void *param)
{
    uint32_t evt;
    uint32_t ptr;
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();

    while (1)
    {
        if (g_dfu_sem)
            os_sem_take(g_dfu_sem, RT_WAITING_FOREVER);
        {
            env->rc.is_power_on = 1;
            env->rc.conn_para.mtu = 23; /* Default value. */
            env->rc.reboot_after_disconnect = 0;
            /* First enable connectable adv then enable non-connectable. */
            // delay to make sure resolving list add finish
            HAL_Delay(200);
            dfu_protocol_advertising_start();
            /* Enable serial transmission service. */
#ifdef BSP_BLE_SERIAL_TRANSMISSION
            ble_serial_tran_init();
#endif
            LOG_I("receive BLE power on!\r\n");
        }
    }
}
#endif // BSP_BLE_SIBLES

int8_t dfu_protocol_reset_env_prepare(void)
{
#ifdef BSP_BLE_SIBLES
    sifli_ble_enable();
#ifdef OS_ADAPTOR_V2
    g_dfu_sem = os_sem_create("dfu_sem", 0);
    g_dfu_tid = os_thread_create("dfu_tid", dfu_protocol_reset_entry, NULL, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 8, 10);
#else
    os_sem_create(g_dfu_sem, 0);
    os_thread_create(g_dfu_tid, dfu_protocol_reset_entry, NULL, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 8, 10);
#endif
    return 0;
#else // BSP_BLE_SIBLES
    return -1;
#endif // BSP_BLE_SIBLES
}

int8_t dfu_protocol_session_close(void)
{
    LOG_W("dfu_protocol_session_close");
#ifdef BSP_BLE_SIBLES
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();

    if (env->rc.is_connected)
    {
        env->rc.is_connected = 0;
        ble_gap_disconnect_t conn;
        conn.conn_idx = env->rc.conn_idx;
        conn.reason = 0x16;
        ble_gap_disconnect(&conn);
    }
    /* Do nothing if link not existed. */
    return 0;
#else // BSP_BLE_SIBLES
    return -1;
#endif // BSP_BLE_SIBLES
}

#ifdef BSP_BLE_SIBLES
int ble_dfu_protocol_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();

    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (g_dfu_sem)
            os_sem_release(g_dfu_sem);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->rc.conn_idx = ind->conn_idx;
        env->rc.is_connected = 1;
        env->rc.conn_para.conn_interval = ind->con_interval;
        env->rc.conn_para.peer_addr_type = ind->peer_addr_type;
        env->rc.conn_para.peer_addr = ind->peer_addr;
        if (ind->role == 0)
            LOG_E("Peripheral should be slave!!!");

        LOG_I("Peer device(%x-%x-%x-%x-%x-%x) connected", env->rc.conn_para.peer_addr.addr[5],
              env->rc.conn_para.peer_addr.addr[4],
              env->rc.conn_para.peer_addr.addr[3],
              env->rc.conn_para.peer_addr.addr[2],
              env->rc.conn_para.peer_addr.addr[1],
              env->rc.conn_para.peer_addr.addr[0]);

        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        env->rc.conn_para.conn_interval = ind->con_interval;
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->rc.conn_para.mtu = ind->mtu;
        LOG_I("Exchanged MTU size: %d", ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        env->rc.is_connected = 0;
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
#ifdef DFU_OTA_MANAGER
        LOG_W("BLE_GAP_DISCONNECTED_IND(%d)", ind->reason);
#endif
        if (env->rc.reboot_after_disconnect == 1)
        {
            LOG_I("Going to reboot");
            env->rc.reboot_after_disconnect = 0;
            drv_reboot();
        }
        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        if (env->rc.is_last_packet_send == 1)
        {
#ifdef BLUETOOTH
            if (sibles_get_tx_pkts() > env->rc.last_tx_packet)
            {
                env->rc.is_last_packet_send = 0;
                //HAL_sw_breakpoint();
                HAL_Delay(500);
                dfu_ctrl_last_packet_handler();
            }
#endif
        }
        break;
    }
    default:
        break;
    }
    return 0;

}

BLE_EVENT_REGISTER(ble_dfu_protocol_event_handler, NULL);
#endif // BSP_BLE_SIBLES

uint8_t *dfu_protocol_packet_buffer_alloc(dfu_protocol_msg_id_t msg_id, uint16_t length)
{
    dfu_tran_protocol_t *packet = malloc(sizeof(dfu_tran_protocol_t) + length);
    OS_ASSERT(packet);
    memset(packet, 0, sizeof(dfu_tran_protocol_t) + length);
    packet->message_id = msg_id;
    packet->length = length;
    return (uint8_t *)&packet->data;
}



int8_t dfu_protocol_packet_send(uint8_t *data)
{
    if (!data)
        return -1;
#if defined (BLUETOOTH) && defined(BSP_BLE_SERIAL_TRANSMISSION)
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    ble_serial_tran_data_t t_data;
    dfu_tran_protocol_t *msg = dfu_packet2msg(data);
    t_data.cate_id = BLE_DFU_CATEID;
    t_data.handle = env->handle;
    t_data.data = (uint8_t *)msg;
    t_data.len = msg->length + sizeof(dfu_tran_protocol_t);
#if !defined(BSP_BLE_SIBLES) && defined(BSP_USING_DATA_SVC)
    ble_dfu_service_send_data(env, &t_data);
#else
    ble_serial_tran_send_data(&t_data);
#endif
    free(msg);
#endif
    return 0;
}

#ifdef BSP_BLE_SERIAL_TRANSMISSION
static void ble_dfu_serial_callback(uint8_t event, uint8_t *data)
{
    if (!data)
        return;

    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
    {
        ble_serial_open_t *open = (ble_serial_open_t *)data;
        env->is_open = 1;
        env->handle = open->handle;
    }
    break;
    case BLE_SERIAL_TRAN_DATA:
    {
        ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)data;
        if (env->handle == t_data->handle
                && t_data->cate_id == BLE_DFU_CATEID)
        {
#ifdef OTA_55X
            dfu_protocol_packet_handler((dfu_tran_protocol_t *)t_data->data, t_data->len);
#endif

#ifdef OTA_56X_NAND
            dfu_protocol_packet_handler_ext((dfu_tran_protocol_t *)t_data->data, t_data->len);
#endif
        }
    }
    break;
    case BLE_SERIAL_TRAN_CLOSE:
    {
        ble_serial_close_t *close = (ble_serial_close_t *)data;
        if (env->handle == close->handle)
        {
            env->is_open = 0;
            dfu_protocol_close_handler();
        }
    }
    case BLE_SERIAL_TRAN_ERROR:
    {
        ble_serial_tran_error_t *error = (ble_serial_tran_error_t *)data;
        if (env->handle == error->handle)
        {
            dfu_serial_transport_error_handle(error->error);
        }
    }
    break;
    default:
        break;
    }


}
#endif

// For dual core.
#if defined(BSP_BLE_SIBLES) && defined(BSP_BLE_SERIAL_TRANSMISSION)
BLE_SERIAL_TRAN_EXPORT(BLE_DFU_CATEID, ble_dfu_serial_callback);
#elif defined(BSP_USING_DATA_SVC)

static int8_t ble_dfu_service_send_reboot_after_disconnect()
{
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    data_msg_t msg;
    rt_err_t ret;
    ble_dfu_service_send_data_t *body = (ble_dfu_service_send_data_t *)data_service_init_msg(&msg, BLE_DFU_REBOOT_AFTER_DISCONNECT, sizeof(ble_dfu_service_send_data_t));
    OS_ASSERT(body);
    body->handle = env->handle;
    body->cate_id = BLE_DFU_CATEID;
    body->len = 0;
    ret = datac_send_msg(env->srv_handle, &msg);
    if (ret != RT_EOK)
        return -2;

    return 0;
}

static int8_t ble_dfu_service_send_disconnect()
{
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    data_msg_t msg;
    rt_err_t ret;
    ble_dfu_service_send_data_t *body = (ble_dfu_service_send_data_t *)data_service_init_msg(&msg, BLE_DFU_REBOOT_DISCONNECT, sizeof(ble_dfu_service_send_data_t));
    OS_ASSERT(body);
    body->handle = env->handle;
    body->cate_id = BLE_DFU_CATEID;
    body->len = 0;
    ret = datac_send_msg(env->srv_handle, &msg);
    if (ret != RT_EOK)
        return -2;

    return 0;
}

static int8_t ble_dfu_service_send_data(dfu_protocol_port_env_t *env, ble_serial_tran_data_t *t_data)
{
    if (t_data == NULL)
        return -1;

    data_msg_t msg;
    rt_err_t ret;
    ble_dfu_service_send_data_t *body = (ble_dfu_service_send_data_t *)data_service_init_msg(&msg, BLE_DFU_SEND_DATA, sizeof(ble_dfu_service_send_data_t) + t_data->len);
    OS_ASSERT(body);
    body->handle = t_data->handle;
    body->cate_id = t_data->cate_id;
    body->len = t_data->len;
    memcpy(body->data, t_data->data, t_data->len);
    ret = datac_send_msg(env->srv_handle, &msg);
    if (ret != RT_EOK)
        return -2;

    return 0;
}


static int ble_dfu_serial_service_callback(data_callback_arg_t *arg)
{
    OS_ASSERT(arg);
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        if (env->srv_handle == rsp->handle
                && rsp->result == 0)
            env->is_subscribed = 1;
        else
            LOG_I("Subscribed dfu service failed %d.", rsp->result);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        env->is_subscribed = 0;
        break;
    }
    case MSG_SERVICE_DATA_NTF_IND:
    {
        ble_dfu_service_data_t *srv_data = (ble_dfu_service_data_t *)arg->data;
        OS_ASSERT(srv_data);
        if (srv_data->event == BLE_SERIAL_TRAN_DATA)
        {
            ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)&srv_data->data;
            t_data->data = (uint8_t *)t_data + sizeof(ble_serial_tran_data_t);
            ble_dfu_serial_callback(srv_data->event, (uint8_t *)t_data);
        }
        else
            ble_dfu_serial_callback(srv_data->event, srv_data->data);
    }
    break;
    case BLE_DFU_REBOOT_AFTER_DISCONNECT_RSP:
    {
        LOG_I("Going to reboot");
        drv_reboot();
        break;
    }
    default:
        break;
    }
    return 0;
}


// For dual core.
//BLE_SERIAL_TRAN_EXPORT(BLE_DFU_CATEID, ble_dfu_serial_callback);

int ble_dfu_serial_service_subscribe(void)
{
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();

    env->srv_handle = datac_open();
    OS_ASSERT(env->srv_handle != DATA_CLIENT_INVALID_HANDLE);

    datac_subscribe(env->srv_handle, "DFUS", ble_dfu_serial_service_callback, 0);
    return 0;
}


INIT_APP_EXPORT(ble_dfu_serial_service_subscribe);
#else
//#error "Should enable sibles or data service."
#endif

void dfu_set_reboot_after_disconnect()
{
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
#if !defined(BSP_BLE_SIBLES) && defined(BSP_USING_DATA_SVC)
    LOG_I("set reboot to port svc");
    ble_dfu_service_send_reboot_after_disconnect();
#else
    if (env->rc.is_connected == 0)
    {
        LOG_I("already disconnected, reboot immediately");
        drv_reboot();
    }
    env->rc.reboot_after_disconnect = 1;
    LOG_I("set rebbot");
#endif
}

uint8_t dfu_set_last_packet_wait()
{
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
#ifdef BLUETOOTH
    if (sibles_get_tx_pkts() < env->rc.last_tx_packet)
    {
        env->rc.is_last_packet_send = 1;
        env->rc.last_tx_packet = sibles_get_tx_pkts();
        return 0;
    }
    else
    {
        // last packet already send
        return 1;
    }
#else
    return 1;
#endif
}

uint8_t dfu_record_current_tx()
{
#ifdef BLUETOOTH
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    env->rc.last_tx_packet = sibles_get_tx_pkts();

    if (env->rc.is_connected == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
#else
    return 0;
#endif
}

void dfu_port_svc_session_close()
{
#if !defined(BSP_BLE_SIBLES) && defined(BSP_USING_DATA_SVC)
    LOG_I("set disconnet to port svc");
    ble_dfu_service_send_disconnect();
#endif
}

uint16_t ble_dfu_protocl_get_supervision_timeout()
{
#ifdef BSP_BLE_CONNECTION_MANAGER
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    cm_conneciont_parameter_value_t *data;
    uint16_t ret_d;
    data = malloc(sizeof(cm_conneciont_parameter_value_t));
    connection_manager_get_connetion_parameter(env->rc.conn_idx, (uint8_t *)data);
    ret_d = data->supervision_timeout;
    free(data);
    if (ret_d < DFU_SYNC_GET_DEFAULT_TIMEOUT)
    {
        ret_d = DFU_SYNC_GET_DEFAULT_TIMEOUT;
    }
    return ret_d;
#else
    return DFU_SYNC_GET_DEFAULT_TIMEOUT;
#endif
}

void ble_dfu_request_connection_priority()
{
#ifdef BSP_BLE_CONNECTION_MANAGER
    dfu_protocol_port_env_t *env = dfu_protocol_get_env();
    cm_conneciont_parameter_value_t *data;
    uint16_t interval;
    uint16_t latency;
    data = malloc(sizeof(cm_conneciont_parameter_value_t));
    connection_manager_get_connetion_parameter(env->rc.conn_idx, (uint8_t *)data);
    interval = data->interval;
    latency = data->slave_latency;
    free(data);
    LOG_I("ble_dfu_request_connection_priority %d", interval);
    // TODO: find a better way to decide to update or not
    if (interval > DFU_INTERVAL_SLOW || latency > 0)
    {
        LOG_I("going to update for dfu");
        connection_manager_update_parameter(env->rc.conn_idx, CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE, NULL);
    }
#endif
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
