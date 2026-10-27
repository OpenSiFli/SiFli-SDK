/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file framed_proto_port.c
 * @brief Framed message transport: session, framing and dispatch.
 *
 * Owns the transmission channel: it brings the Bluetooth link up, keeps the
 * channel handle, frames outbound messages and hands inbound messages to the
 * registered consumer. It does not interpret a single message id.
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "rtconfig.h"
#include <rtthread.h>
#include "board.h"
#include "os_adaptor.h"

#include "framed_proto.h"
#include "framed_proto_link.h"

#ifdef BSP_BLE_SIBLES
    #include "bf0_sibles.h"
    #include "bf0_sibles_internal.h"
    #include "bf0_sibles_serial_trans_service.h"
    #include "att.h"
    #include "bf0_ble_gap.h"
    #include "bf0_ble_common.h"
    #include "ble_connection_manager.h"
#endif

#define LOG_TAG "FRMPROTO"
#include "log.h"

/* Bytes wrapped around a message when it goes out over the application TCP
 * relay: 4B magic + 4B length + 1B category + 3B reserved + 4B CRC. */
#define FRAMED_PROTO_PAN_FRAME_EXTRA    16
#define FRAMED_PROTO_PAN_FRAME_MAGIC    0x53454346

/* One link event, laid out the same way the BLE serial-transmission service
 * reports it, so a sibles event can be dispatched without a copy. */
typedef struct
{
    uint8_t handle;
} framed_proto_open_t;

typedef struct
{
    uint8_t handle;
    uint8_t cate_id;
    uint16_t len;
    uint8_t *data;
} framed_proto_frame_t;

typedef struct
{
    uint8_t handle;
} framed_proto_close_t;

typedef struct
{
    uint8_t handle;
    uint8_t cate_id;
    uint8_t error;
} framed_proto_error_t;

enum
{
    FRAMED_PROTO_EVT_OPEN,
    FRAMED_PROTO_EVT_DATA,
    FRAMED_PROTO_EVT_CLOSE,
    FRAMED_PROTO_EVT_ERROR,
    FRAMED_PROTO_EVT_CONFIGURE,
};

static uint32_t framed_proto_crc32_mpeg2(const void *data, uint32_t len)
{
    CRC_HandleTypeDef crc_handle = {0};

    crc_handle.Instance = hwp_crc;
    if (HAL_CRC_Init(&crc_handle) != HAL_OK)
    {
        return 0;
    }
    HAL_CRC_Setmode(&crc_handle, CRC_32_MPEG_2);
    return HAL_CRC_Calculate(&crc_handle, (uint8_t *)data, len);
}

typedef struct
{
    uint8_t is_open;
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
#if defined(BSP_BLE_SIBLES)
            bd_addr_t peer_addr;
#endif
            uint16_t mtu;
            uint16_t conn_interval;
            uint8_t peer_addr_type;
        } conn_para;
        rt_sem_t sema_handle;
    } rc;

} framed_proto_env_t;

static OS_THREAD_DECLAR(g_framed_proto_tid);
static OS_SEM_DECLAR(g_framed_proto_sem);

static framed_proto_env_t g_framed_proto_env;
static framed_proto_handler_t g_framed_proto_handler;
static framed_proto_sink_t g_framed_proto_sink;
static uint8_t g_framed_proto_sink_set;

void framed_proto_set_handler(const framed_proto_handler_t *handler)
{
    if (handler)
        g_framed_proto_handler = *handler;
    else
        memset(&g_framed_proto_handler, 0, sizeof(g_framed_proto_handler));
}

void framed_proto_set_sink(const framed_proto_sink_t *sink)
{
    if (sink)
    {
        g_framed_proto_sink = *sink;
        g_framed_proto_sink_set = 1;
    }
    else
    {
        memset(&g_framed_proto_sink, 0, sizeof(g_framed_proto_sink));
        g_framed_proto_sink_set = 0;
    }
}

uint8_t framed_proto_is_stream(void)
{
    return (g_framed_proto_sink.is_stream && g_framed_proto_sink.is_stream()) ? 1 : 0;
}

const framed_proto_sink_t *framed_proto_get_sink(void)
{
    return g_framed_proto_sink_set ? &g_framed_proto_sink : NULL;
}

static framed_proto_env_t *framed_proto_get_env(void)
{
    return &g_framed_proto_env;
}

#if defined(BSP_BLE_SIBLES)
bd_addr_t *framed_proto_get_peer_addr(void)
{
    return &g_framed_proto_env.rc.conn_para.peer_addr;
}
#endif

static inline framed_proto_msg_t *framed_proto_buf2msg(void const *param_ptr)
{
    return (framed_proto_msg_t *)(((uint8_t *)param_ptr) - offsetof(framed_proto_msg_t, data));
}

static framed_proto_ready_cb_t g_framed_proto_ready_cb;

void framed_proto_set_ready_cb(framed_proto_ready_cb_t cb)
{
    g_framed_proto_ready_cb = cb;
}

#ifdef BSP_BLE_SIBLES
static void framed_proto_reset_entry(void *param)
{
    framed_proto_env_t *env = framed_proto_get_env();

    while (1)
    {
        if (g_framed_proto_sem)
            os_sem_take(g_framed_proto_sem, RT_WAITING_FOREVER);
        {
            env->rc.is_power_on = 1;
            env->is_open = 0;
            env->rc.conn_para.mtu = 23; /* Default value. */
            env->rc.reboot_after_disconnect = 0;
            /* First enable connectable adv then enable non-connectable. */
            // delay to make sure resolving list add finish
            HAL_Delay(200);
            /* Advertising is started by the product. */
            if (g_framed_proto_ready_cb)
            {
                g_framed_proto_ready_cb();
            }
            /* Enable serial transmission service. */
#ifdef BSP_BLE_SERIAL_TRANSMISSION
            ble_serial_tran_init();
#endif
            LOG_I("receive BLE power on!\r\n");
        }
    }
}
#endif // BSP_BLE_SIBLES

int8_t framed_proto_init(void)
{
#if defined(BSP_BLE_SIBLES)

#ifdef OS_ADAPTOR_V2
    g_framed_proto_sem = os_sem_create("frm_sem", 0);
    g_framed_proto_tid = os_thread_create("frm_tid", framed_proto_reset_entry, NULL, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 8, 10);
#else
    os_sem_create(g_framed_proto_sem, 0);
    os_thread_create(g_framed_proto_tid, framed_proto_reset_entry, NULL, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 8, 10);
#endif

    sifli_ble_enable();
    return 0;
#else // BSP_BLE_SIBLES
    return -1;
#endif // BSP_BLE_SIBLES
}

int8_t framed_proto_session_close(void)
{
    LOG_W("framed_proto_session_close");
#if defined(BSP_BLE_SIBLES)
    framed_proto_env_t *env = framed_proto_get_env();

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
static int framed_proto_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    framed_proto_env_t *env = framed_proto_get_env();

    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (g_framed_proto_sem)
            os_sem_release(g_framed_proto_sem);
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
#if defined(BSP_BLE_CONNECTION_MANAGER)
        if ((ind->con_interval > FRAMED_PROTO_INTERVAL_SLOW || ind->con_latency > 0)
                && g_framed_proto_handler.is_busy && g_framed_proto_handler.is_busy())
        {
            connection_manager_update_parameter(env->handle, CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE, NULL);
        }
#endif
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
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;

        if (ind->conn_idx != env->rc.conn_idx)
        {
            LOG_I("disconnect for idx %d", ind->conn_idx);
            break;
        }
        env->rc.is_connected = 0;
        LOG_W("BLE_GAP_DISCONNECTED_IND(%d)", ind->reason);
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
#if defined(BSP_BLE_SIBLES)
            if (sibles_get_tx_pkts() > env->rc.last_tx_packet)
            {
                env->rc.is_last_packet_send = 0;
                HAL_Delay(500);
                if (g_framed_proto_handler.on_tx_done)
                    g_framed_proto_handler.on_tx_done();
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

BLE_EVENT_REGISTER(framed_proto_ble_event_handler, NULL);
#endif // BSP_BLE_SIBLES

uint8_t *framed_proto_msg_alloc(uint16_t msg_id, uint16_t length)
{
    framed_proto_msg_t *packet;

    if (length > UINT16_MAX - sizeof(framed_proto_msg_t))
        return NULL;

    packet = malloc(sizeof(framed_proto_msg_t) + length);

    OS_ASSERT(packet);
    memset(packet, 0, sizeof(framed_proto_msg_t) + length);
    packet->message_id = msg_id;
    packet->length = length;
    return (uint8_t *)&packet->data;
}

void framed_proto_msg_free(uint8_t *msg)
{
    if (msg)
        free(framed_proto_buf2msg(msg));
}

int8_t framed_proto_msg_send(uint8_t *data)
{
    int8_t ret = 0;
    framed_proto_msg_t *msg;
    framed_proto_frame_t frame;
    uint8_t tran_type;

    if (!data)
        return -1;

    msg = framed_proto_buf2msg(data);

    {
        framed_proto_env_t *env = framed_proto_get_env();

        frame.handle = env->handle;
        frame.cate_id = FRAMED_PROTO_CATE_ID;
        frame.len = msg->length + sizeof(framed_proto_msg_t);
        frame.data = (uint8_t *)msg;
    }

    tran_type = framed_proto_link_type_get();

    if (tran_type == FRAMED_PROTO_LINK_PAN || tran_type == FRAMED_PROTO_LINK_WIFI)
    {
        /* Application TCP relay: wrap the payload in the frame the relay and
         * the phone app expect. */
        uint32_t magic = FRAMED_PROTO_PAN_FRAME_MAGIC;
        uint32_t data_len = (uint32_t)frame.len;
        uint32_t data_crc;
        uint8_t *packet = (uint8_t *)malloc(frame.len + FRAMED_PROTO_PAN_FRAME_EXTRA);

        if (packet == NULL)
        {
            free(msg);
            return -1;
        }
        memset(packet, 0, frame.len + FRAMED_PROTO_PAN_FRAME_EXTRA);
        memcpy(packet, &magic, 4);
        memcpy(packet + 4, &data_len, 4);
        *(packet + 8) = frame.cate_id;
        memcpy(packet + 12, frame.data, frame.len);
        data_crc = framed_proto_crc32_mpeg2(frame.data, frame.len);
        memcpy(packet + 12 + frame.len, &data_crc, 4);

        {
            int sent = framed_proto_pan_send_data(packet,
                                                  frame.len + FRAMED_PROTO_PAN_FRAME_EXTRA);
            if (sent != (int)(frame.len + FRAMED_PROTO_PAN_FRAME_EXTRA))
            {
                LOG_E("pan send failed %d, expect %d", sent,
                      frame.len + FRAMED_PROTO_PAN_FRAME_EXTRA);
                ret = -1;
            }
        }
        free(packet);
    }
    else
    {
#ifdef BSP_BLE_SERIAL_TRANSMISSION
        ble_serial_tran_data_t ble_data;

        ble_data.handle = frame.handle;
        ble_data.cate_id = frame.cate_id;
        ble_data.len = frame.len;
        ble_data.data = frame.data;

        if (ble_serial_tran_send_data(&ble_data) < 0)
        {
            LOG_E("ble send failed");
            ret = -1;
        }
#else
        /* No BLE transport and the relay is not up: the message cannot be
         * delivered, and the buffer is still ours to release. */
        LOG_E("framed_proto_msg_send: no transport built in");
        ret = -1;
#endif
    }

    free(msg);
    return ret;
}

/* ------------------------------------------------------------------------
 * Transport-neutral dispatch
 *
 * Every link hands its events over here. The BLE service delivers its own
 * structures, whose layout matches framed_proto_open_t / _frame_t / _close_t
 * / _error_t field for field, so no copy is needed; the application TCP
 * relay builds the same structures locally.
 * ---------------------------------------------------------------------- */
static void framed_proto_dispatch(uint8_t event, void *data)
{
    framed_proto_env_t *env = framed_proto_get_env();

    if (!data)
        return;

    switch (event)
    {
    case FRAMED_PROTO_EVT_OPEN:
    {
        framed_proto_open_t *open = (framed_proto_open_t *)data;

        if (env->is_open == 0)
        {
            env->is_open = 1;
            env->handle = open->handle;
        }
        break;
    }
    case FRAMED_PROTO_EVT_DATA:
    {
        framed_proto_frame_t *t_data = (framed_proto_frame_t *)data;

        if (env->handle != t_data->handle &&
                t_data->cate_id == FRAMED_PROTO_CATE_ID)
        {
            LOG_I("update env handle %d -> %d", env->handle, t_data->handle);
            env->handle = t_data->handle;

            if (env->rc.is_connected == 0)
            {
                env->rc.is_connected = 1;
            }
        }

        if (env->handle == t_data->handle
                && t_data->cate_id == FRAMED_PROTO_CATE_ID)
        {
            framed_proto_msg_t *msg = (framed_proto_msg_t *)t_data->data;

            if (t_data->len < sizeof(framed_proto_msg_t)
                    || msg->length > t_data->len - sizeof(framed_proto_msg_t))
            {
                LOG_E("malformed frame, len %d", t_data->len);
                break;
            }
            if (g_framed_proto_handler.on_msg)
                g_framed_proto_handler.on_msg(msg, t_data->len);
        }
        break;
    }
    case FRAMED_PROTO_EVT_CLOSE:
    {
        framed_proto_close_t *close = (framed_proto_close_t *)data;

        if (env->handle == close->handle)
        {
            env->is_open = 0;
            env->rc.is_connected = 0;
            if (g_framed_proto_handler.on_close)
                g_framed_proto_handler.on_close();
        }
        break;
    }
    case FRAMED_PROTO_EVT_ERROR:
    {
        framed_proto_error_t *error = (framed_proto_error_t *)data;

        if (env->handle == error->handle)
        {
            if (g_framed_proto_handler.on_error)
                g_framed_proto_handler.on_error(error->error);
        }
        break;
    }
    case FRAMED_PROTO_EVT_CONFIGURE:
    {
        framed_proto_link_configure_t *cfg = (framed_proto_link_configure_t *)data;

        if (env->handle == cfg->handle)
        {
            if (g_framed_proto_handler.on_configure)
                g_framed_proto_handler.on_configure(cfg->len, cfg->data);
        }
        break;
    }
    default:
        break;
    }
}

#ifdef BSP_BLE_SIBLES
/* Thin adapter: translate the sibles event ids into the transport-neutral
 * ones and pass the payload through untouched. */
static void framed_proto_serial_callback(uint8_t event, uint8_t *data)
{
    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
        framed_proto_dispatch(FRAMED_PROTO_EVT_OPEN, data);
        break;
    case BLE_SERIAL_TRAN_DATA:
        framed_proto_dispatch(FRAMED_PROTO_EVT_DATA, data);
        break;
    case BLE_SERIAL_TRAN_CLOSE:
        framed_proto_dispatch(FRAMED_PROTO_EVT_CLOSE, data);
        break;
    case BLE_SERIAL_TRAN_ERROR:
        framed_proto_dispatch(FRAMED_PROTO_EVT_ERROR, data);
        break;
    case FRAMED_PROTO_LINK_CONFIGURE:
        framed_proto_dispatch(FRAMED_PROTO_EVT_CONFIGURE, data);
        break;
    default:
        break;
    }
}

#ifdef BSP_BLE_SERIAL_TRANSMISSION
    BLE_SERIAL_TRAN_EXPORT(FRAMED_PROTO_CATE_ID, framed_proto_serial_callback);
#endif
#endif /* BSP_BLE_SIBLES */

/* ------------------------------------------------------------------------
 * Application TCP relay entry points: the relay forwards whole frames /
 * errors / disconnection here; they mirror what the BLE service delivers
 * through the SerialTranExport table.  handle 0 stands for the TCP channel
 * (it has no GATT conn_idx); the DATA case above re-binds env->handle by
 * itself.
 * ---------------------------------------------------------------------- */
void framed_proto_rx_frame(uint8_t cate_id, uint8_t *payload, uint16_t len)
{
    framed_proto_frame_t frame;

    frame.handle = 0;
    frame.cate_id = cate_id;
    frame.len = len;
    frame.data = payload;
    framed_proto_dispatch(FRAMED_PROTO_EVT_DATA, &frame);
}

void framed_proto_rx_error(uint8_t error)
{
    framed_proto_error_t err;

    err.handle = 0;
    err.cate_id = FRAMED_PROTO_CATE_ID;
    err.error = error;
    framed_proto_dispatch(FRAMED_PROTO_EVT_ERROR, &err);
}

void framed_proto_rx_closed(void)
{
    framed_proto_close_t close;

    close.handle = 0;
    framed_proto_dispatch(FRAMED_PROTO_EVT_CLOSE, &close);
}

void framed_proto_set_reboot_after_disconnect(void)
{
    framed_proto_env_t *env = framed_proto_get_env();

    if (env->rc.is_connected == 0)
    {
        LOG_I("already disconnected, reboot immediately");
        drv_reboot();
    }
    env->rc.reboot_after_disconnect = 1;
    LOG_I("set reboot");
}

uint8_t framed_proto_last_packet_wait(void)
{
    framed_proto_env_t *env = framed_proto_get_env();
#if defined(BSP_BLE_SIBLES)
    /* Only a plain BLE link counts packets in the transmit queue; SPP, PAN and
     * the TCP relay all drain elsewhere. */
    if (framed_proto_link_type_get() != FRAMED_PROTO_LINK_BLE)
    {
        return 1;
    }

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

uint8_t framed_proto_record_tx(void)
{
#if defined(BSP_BLE_SIBLES)
    framed_proto_env_t *env = framed_proto_get_env();
    env->rc.last_tx_packet = sibles_get_tx_pkts();

    if (env->rc.is_connected == 0)
    {
        if (framed_proto_link_type_get() != FRAMED_PROTO_LINK_BLE)
        {
            return 1;
        }
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

uint16_t framed_proto_get_supervision_timeout(void)
{
    framed_proto_env_t *env = framed_proto_get_env();
    uint16_t ret_d = FRAMED_PROTO_SYNC_DEFAULT_TIMEOUT;
#ifdef BSP_BLE_CONNECTION_MANAGER
    cm_conneciont_parameter_value_t *data;
    data = malloc(sizeof(cm_conneciont_parameter_value_t));
    if (data)
    {
        memset(data, 0, sizeof(*data));
        connection_manager_get_connetion_parameter(env->handle, (uint8_t *)data);
        ret_d = data->supervision_timeout;
        free(data);
    }
#endif
    if (ret_d < FRAMED_PROTO_SYNC_DEFAULT_TIMEOUT)
    {
        ret_d = FRAMED_PROTO_SYNC_DEFAULT_TIMEOUT;
    }
    return ret_d;
}

void framed_proto_request_connection_priority(void)
{
    framed_proto_env_t *env = framed_proto_get_env();
    uint16_t interval = 0;
    uint16_t latency = 0;
#ifdef BSP_BLE_CONNECTION_MANAGER
    cm_conneciont_parameter_value_t *data;
    data = malloc(sizeof(cm_conneciont_parameter_value_t));
    if (data)
    {
        memset(data, 0, sizeof(*data));
        connection_manager_get_connetion_parameter(env->handle, (uint8_t *)data);
        interval = data->interval;
        latency = data->slave_latency;
        free(data);
    }
#endif

#if defined(BSP_BLE_SERIAL_TRANSMISSION)
    if (framed_proto_link_type_get() != FRAMED_PROTO_LINK_BLE)
    {
        return;
    }
#endif

    LOG_I("framed_proto_request_connection_priority %d", interval);
    if (interval > FRAMED_PROTO_INTERVAL_SLOW || latency > 0)
    {
        LOG_I("going to update for proto");
#if defined(BSP_BLE_CONNECTION_MANAGER)
        connection_manager_update_parameter(env->handle, CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE, NULL);
#endif
    }

}

void framed_proto_recovery_connection(void)
{
    framed_proto_env_t *env = framed_proto_get_env();
    if (env->rc.is_connected == 0)
    {
        return;
    }

#if defined(BSP_BLE_SERIAL_TRANSMISSION)
    if (framed_proto_link_type_get() != FRAMED_PROTO_LINK_BLE)
    {
        return;
    }
#endif

    LOG_I("framed_proto_recovery_connection");

#if defined(BSP_BLE_CONNECTION_MANAGER)
    connection_manager_update_parameter(env->handle, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
#endif
}
