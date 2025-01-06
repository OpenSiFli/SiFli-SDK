#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include "os_adaptor.h"

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"

#include "bf0_sibles_console.h"
#include "bf0_ble_common.h"

#include "data_service.h"

#include "bf0_sibles_serial_trans_service.h"

#ifndef DFU_OTA_MANAGER
#ifdef SIBLES_SERIAL_DEVICE
#include "msh.h"
#define LOG_TAG "sibles"
#include "log.h"


#ifndef BLE_INVALID_CHANHDL
    #define BLE_INVALID_CHANHDL                      (0xFF)
#endif

typedef struct
{
    uint16_t command_len;
    uint8_t srv_handle;
    uint8_t is_subscribed;
    uint8_t handle;
    struct rt_device device;
    struct rt_ringbuffer read_fifo;
    uint8_t name[RT_NAME_MAX];
    uint8_t pool[SIBLE_SERIAL_POOL_SIZE];
} ble_console_env_t;

typedef struct
{
    uint8_t event;
    uint16_t len;
    uint8_t data[0];
} ble_console_data_t;



static ble_console_env_t g_console_env;
static rt_device_t old_console;
rt_timer_t g_console_time_handle;

static ble_console_env_t *ble_console_get_env(void)
{
    return &g_console_env;
}

static void console_timeout_handler(void *parameter)
{
    ble_console_env_t *env = ble_console_get_env();
    msh_set_console((const char *)env->name);
}


int ble_console_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_console_env_t *env = ble_console_get_env();

    switch (event_id)
    {
    case BLE_GAP_CONNECTED_IND:
    {
        old_console = rt_console_get_device();

        ulog_tag_lvl_filter_set(LOG_TAG, LOG_LVL_ASSERT);
#ifdef  SIBLES_SERIAL_DEVICE_AUTO
        // immediately set console will cause error.
        if (!g_console_time_handle)
        {
            g_console_time_handle = rt_timer_create("ble_console", console_timeout_handler, NULL,
                                                    rt_tick_from_millisecond(600), RT_TIMER_FLAG_SOFT_TIMER);
        }
        else
        {
            rt_timer_stop(g_console_time_handle);
        }
        rt_timer_start(g_console_time_handle);
#endif

        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;

#ifdef  SIBLES_SERIAL_DEVICE_AUTO
        if (g_console_time_handle)
        {
            rt_timer_stop(g_console_time_handle);
        }
#endif

        if (old_console)
        {
            msh_set_console((const char *)old_console->parent.name);
        }
        ulog_tag_lvl_filter_set(LOG_TAG, ulog_global_filter_lvl_get());
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_console_handler, NULL);

static rt_size_t _ble_dev_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    ble_console_env_t *env = ble_console_get_env();
#ifdef SIBLES_IGNORE_CONSOLE_ECHO
    env->command_len = size;
#endif
    return rt_ringbuffer_get(&env->read_fifo, buffer, size);
}

static rt_size_t _ble_dev_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    ble_console_env_t *env = ble_console_get_env();
    if (!buffer || env->handle == BLE_INVALID_CHANHDL)
        return 0;
    ble_serial_tran_data_t t_data;
    t_data.cate_id = BLE_CONSOLE_CATEID;
    t_data.handle = env->handle;
    t_data.data = (uint8_t *)buffer;
    t_data.len = size;

#if 1
    rt_device_write(old_console, pos, buffer, size);
#endif

#ifdef SIBLES_IGNORE_CONSOLE_ECHO
    if (env->command_len > 0)
    {
        env->command_len--;
        return 0;
    }
#endif

    return ble_serial_tran_send_data(&t_data);
}

void ble_console_serial_callback(uint8_t event, uint8_t *data)
{

    if (!data)
        return;

    ble_console_env_t *env = ble_console_get_env();
    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
    {
        ble_serial_open_t *open = (ble_serial_open_t *)data;

        // only set for 1st device
        if (env->handle == BLE_INVALID_CHANHDL)
        {
            env->handle = open->handle;
        }
    }
    break;
    case BLE_SERIAL_TRAN_DATA:
    {
        ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)data;
        if (env->handle == t_data->handle && t_data->cate_id == BLE_CONSOLE_CATEID)
        {
            rt_ringbuffer_put(&env->read_fifo, t_data->data, t_data->len);
            if (env->device.rx_indicate)
            {
                env->device.rx_indicate(&(env->device), t_data->len);
            }
        }
    }
    break;
    case BLE_SERIAL_TRAN_CLOSE:
    {
        ble_serial_close_t *close = (ble_serial_close_t *)data;
        if (env->handle == close->handle)
        {
            env->handle = BLE_INVALID_CHANHDL;
        }
    }
    break;
    default:
        break;
    }
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops ble_serial_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _ble_dev_read,
    _ble_dev_write,
    RT_NULL,
};
#endif

int rt_ble_serial_dev_register(const char *name)
{
    rt_err_t result = RT_EOK;
    ble_console_env_t *env = ble_console_get_env();

    env->device.type = RT_Device_Class_Miscellaneous;
    env->device.rx_indicate = RT_NULL;
    env->device.tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    env->device.ops         = &ble_serial_ops;
#else
    env->device.init        = RT_NULL;
    env->device.open        = RT_NULL;
    env->device.close       = RT_NULL;
    env->device.read        = _ble_dev_read;
    env->device.write       = _ble_dev_write;
    env->device.control     = RT_NULL;
#endif
    env->device.user_data = (void *)NULL;
    env->handle = BLE_INVALID_CHANHDL;
    rt_ringbuffer_init(&env->read_fifo, (uint8_t *)env->pool, SIBLE_SERIAL_POOL_SIZE);
    strncpy((char *)env->name, (const char *)name, RT_NAME_MAX);
    result = rt_device_register(&env->device, name, RT_DEVICE_FLAG_RDWR);

    return (int)result;
}

static int ble_console_serial_service_callback(data_callback_arg_t *arg)
{
    OS_ASSERT(arg);
    ble_console_env_t *env = ble_console_get_env();
    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        if (env->srv_handle == rsp->handle
                && rsp->result == 0)
            env->is_subscribed = 1;
        else
            LOG_I("Subscribed console service failed %d.", rsp->result);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        env->is_subscribed = 0;
        break;
    }
    case MSG_SERVICE_DATA_NTF_IND:
    {
        ble_console_data_t *srv_data = (ble_console_data_t *)arg->data;
        OS_ASSERT(srv_data);
        if (srv_data->event == BLE_SERIAL_TRAN_DATA)
        {
            ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)&srv_data->data;
            t_data->data = (uint8_t *)t_data + sizeof(ble_serial_tran_data_t);
            ble_console_serial_callback(srv_data->event, (uint8_t *)t_data);
        }
        else
        {
            ble_console_serial_callback(srv_data->event, srv_data->data);
        }
        break;
    }

    default:
        break;
    }
    return 0;
}

int ble_console_serial_service_subscribe(void)
{
    ble_console_env_t *env = ble_console_get_env();

    env->srv_handle = datac_open();
    OS_ASSERT(env->srv_handle != DATA_CLIENT_INVALID_HANDLE);
    rt_ble_serial_dev_register("bleuart");

    datac_subscribe(env->srv_handle, "CONSOLES", ble_console_serial_service_callback, 0);
    return 0;
}

BLE_SERIAL_TRAN_EXPORT(BLE_CONSOLE_CATEID, ble_console_serial_callback);

INIT_APP_EXPORT(ble_console_serial_service_subscribe);

#endif
#endif // DFU_OTA_MANAGER
