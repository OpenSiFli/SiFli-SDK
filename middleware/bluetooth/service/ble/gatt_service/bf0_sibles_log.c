#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "os_adaptor.h"
#include "bf0_hal_hlp.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_serial_trans_service.h"
#include "att.h"
#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_log.h"

#ifdef BSP_BLE_LOG


#ifdef BSP_BLE_CONNECTION_MANAGER
    #include "ble_connection_manager.h"
#endif

#ifndef BSP_USING_PC_SIMULATOR
//#define ULOG_BACKEND_USING_FS 1
#if defined (RT_USING_DFS) && (defined(ULOG_BACKEND_USING_FS) || defined(BSP_USING_LVGL_INPUT_AGENT) || defined(SAVE_ASSERT_CONTEXT_IN_FLASH) || defined(MC_BACKEND_USING_FILE))
    #define LOG_USING_FS 1
    #include <dfs_posix.h>
#endif

#ifndef DFU_OTA_MANAGER
#define LOG_TAG "BLE_LOG"
#include "log.h"
#ifdef MC_BACKEND_USING_FILE
    #include "metrics_collector.h"
#endif

#if defined(ULOG_BACKEND_USING_RAM)
    #include "ram_be.h"
#endif


static ble_log_env_t g_ble_log;

static void ble_audio_send_thread();

RT_WEAK void audio_dump_enable(uint8_t type)
{
}
RT_WEAK void audio_dump_clear()
{
}
RT_WEAK void webrtc_set_delay(uint16_t u16_delay)
{
}

RT_WEAK char **get_audio_dump_files()
{
    return NULL;
}

RT_WEAK void audio_command_process(uint8_t *cmd)
{
}


static ble_log_env_t *ble_log_get_env(void)
{
    return &g_ble_log;
}

#if 0
    extern void clear_assert();
    extern uint32_t ble_log_get_assert(uint32_t *addr);
    extern uint32_t ble_wfile_get_mem(uint32_t *addr);
    extern uint32_t ble_wfile_to_mem(uint8_t *src, uint32_t addr, uint32_t len);
    extern uint32_t ble_get_tsdb_log(uint32_t *addr);
    extern uint32_t ble_get_tsdb_metrics(uint32_t *addr);
#endif

static void ble_log_send_data_response(uint32_t size);

#ifdef ULOG_BACKEND_USING_FS
    extern int ble_log_onoff(int flag);      //flag: 0->off  1->on       2->return on/off
    extern int ble_log_clear();      //delete file or erase flash
    extern int ble_log_type_get();       //retun: -1:no assert   0:write flash   1:use FS
    extern int ble_log_mem_get(uint32_t *addr, uint32_t *len);  //return:  0:err  1:ok
    extern const char *ble_log_file_get(); //return:   NULL:err  other:file full path


#endif

#ifdef SAVE_ASSERT_CONTEXT_IN_FLASH
    extern int ble_assert_onoff(int flag);  //flag: 0->off  1->on       2->return on/off
    extern int ble_assert_clear();       //delete file or erase flash
    extern int ble_assert_type_get();       //retun: -1:no assert   0:write flash   1:use FS
    extern int ble_assert_mem_get(uint32_t *addr, uint32_t *len);  //return:  0:err  1:ok
    extern const char *ble_assert_file_get();   //return:   NULL:err  other:file full path
#endif


static int ble_log_data_send(uint8_t *raw_data, uint16_t size)
{
    ble_log_env_t *env = ble_log_get_env();

    ble_serial_tran_data_t t_data;
    t_data.cate_id = BLE_LOG_CATEID;
    t_data.handle = env->handle;
    t_data.data = (uint8_t *)raw_data;
    t_data.len = size;
    return ble_serial_tran_send_data(&t_data);
}

static void ble_log_send_advance(uint8_t *raw_data, uint32_t size)
{
    ble_log_env_t *env = ble_log_get_env();
    uint32_t send_index = 0;
    uint8_t *send_data;
    uint32_t i = 1;

    // every packet raw data length
    uint16_t current_len;

    if (env->mtu > MAX_PACKCT_SIZE + 3)
    {
        current_len = MAX_PACKCT_SIZE - SERIAL_TRANS_HEADER - BLE_LOG_SEND_HEADER;
    }
    else
    {
        current_len = env->mtu - SERIAL_TRANS_HEADER - BLE_LOG_SEND_HEADER - 3;
    }

    while (send_index < size)
    {
        if (send_index + current_len > size)
        {
            // process last packet
            current_len = size - send_index;
        }
        send_data = raw_data + send_index;
        uint16_t packet_len = current_len + SERIAL_TRANS_HEADER + BLE_LOG_SEND_HEADER;

        uint8_t *f_data = (uint8_t *)bt_mem_alloc(packet_len);
        BT_OOM_ASSERT(f_data);

        // build serial header
        *f_data = BLE_LOG_CATEID;
        *(f_data + 1) = 0;

        uint16_t log_packet_len = current_len + BLE_LOG_SEND_HEADER;
        rt_memcpy(f_data + 2, &log_packet_len, 2);

        // build ble log send header
        *(f_data + 4) = BLE_LOG_GET_SEND_PACKET;

        env->send_index++;
        rt_memcpy(f_data + 5, &env->send_index, 4);

        rt_memcpy(f_data + 9, send_data, current_len);

        int ret;
        if (env->conn_state == 1)
        {
            ret = ble_serial_tran_send_data_advance(env->handle, f_data, packet_len);
        }
        bt_mem_free(f_data);

        send_index += current_len;
        i++;
    }
}

#if 0
static void ble_log_send_status_result(uint32_t size)
{
    uint16_t data_len = 1 + sizeof(uint32_t);
    uint8_t *send_data = (uint8_t *)rt_malloc(data_len);
    *send_data = BLE_LOG_STATUS_REPLY;
    rt_memcpy(send_data + 1, &size, sizeof(uint32_t));
    ble_log_data_send(send_data, data_len);
    rt_free(send_data);
}

static void ble_log_send_data_begin(uint32_t size)
{
    uint8_t start_flag = BLE_LOG_GET_SEND_START;
    uint8_t *start_data = (uint8_t *)rt_malloc(sizeof(uint32_t) + sizeof(uint8_t));
    rt_memcpy(start_data, &start_flag, sizeof(uint8_t));

    rt_memcpy(start_data + sizeof(uint8_t), &size, sizeof(uint32_t));
    ble_log_data_send(start_data, sizeof(uint32_t) + sizeof(uint8_t));
    rt_free(start_data);
}
#endif


static void ble_log_send_data_end()
{
    // in some situation will be blocked by log packet
    // uint8_t end_flag = BLE_LOG_GET_SEND_END;
    // ble_log_data_send(&end_flag, 1);

    ble_log_env_t *env = ble_log_get_env();

    uint16_t data_len = 1;
    uint16_t len = SERIAL_TRANS_HEADER + data_len;
    uint8_t *f_data = (uint8_t *)bt_mem_alloc(len);
    BT_OOM_ASSERT(f_data);
    if (f_data)
    {
        *f_data = BLE_LOG_CATEID;
        *(f_data + 1) = 0;
        rt_memcpy(f_data + 2, &data_len, 2);
        *(f_data + 4) = BLE_LOG_GET_SEND_END;

        ble_serial_tran_send_data_advance(env->handle, f_data, len);
    }
}

static void ble_log_send_data_finish()
{
    // in some situation will be blocked by log packet
    // uint8_t end_flag = BLE_LOG_GET_SEND_END;
    // ble_log_data_send(&end_flag, 1);

    ble_log_env_t *env = ble_log_get_env();

    uint16_t data_len = 1;
    uint16_t len = SERIAL_TRANS_HEADER + data_len;
    uint8_t *f_data = (uint8_t *)bt_mem_alloc(len);
    BT_OOM_ASSERT(f_data);
    if (f_data)
    {
        *f_data = BLE_LOG_CATEID;
        *(f_data + 1) = 0;
        rt_memcpy(f_data + 2, &data_len, 2);
        *(f_data + 4) = BLE_LOG_GET_SEND_FINISH;

        ble_serial_tran_send_data_advance(env->handle, f_data, len);
    }
}

static void ble_log_buf_send(rt_uint8_t *data, rt_uint32_t len)
{
    int iCnt = 0;
    int block_len = 2048;
    for (iCnt = 0; iCnt < len / block_len; iCnt++)
    {
        ble_log_send_advance(&data[iCnt * block_len], block_len);
    }

    if (len % block_len)
    {
        ble_log_send_advance(&data[iCnt * block_len], len % block_len);
    }
    return;
}

static void ble_ram_be_log_send(void)
{
#if defined(ULOG_BACKEND_USING_RAM)
    rt_uint32_t data_len;
    ble_log_env_t *env = ble_log_get_env();
    ulog_ram_be_buf_t *ram_be = ulog_ram_be_buf_get((rt_uint32_t *)&data_len);

    if (!ram_be->full)
    {
        ble_log_send_data_response(ram_be->wr_offset);
        env->send_index = 0;
        ble_log_buf_send(&ram_be->buf[0], ram_be->wr_offset);
    }
    else
    {
        ble_log_send_data_response(ram_be->buf_size);
        env->send_index = 0;
        ble_log_buf_send(&ram_be->buf[ram_be->wr_offset], ram_be->buf_size - ram_be->wr_offset);
        ble_log_buf_send(&ram_be->buf[0], ram_be->wr_offset);
    }

    ble_log_send_data_end();

#endif
    return;
}

static void ble_log_connection_update()
{
    LOG_I("ble_log_connection_update");
#ifdef BSP_BLE_CONNECTION_MANAGER
    ble_log_env_t *env = ble_log_get_env();

    cm_conneciont_parameter_value_t data;
    uint16_t interval;
    uint16_t latency;
    connection_manager_get_connetion_parameter(env->conn_idx, (uint8_t *)&data);
    interval = data.interval;
    latency = data.slave_latency;

    if (interval > 20 || latency > 0)
    {
        LOG_I("going to update for ble log");
        connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE, NULL);
    }
#endif
}

static void ble_log_connection_recover()
{
    LOG_I("ble_log_connection_recover");
#ifdef BSP_BLE_CONNECTION_MANAGER
    ble_log_env_t *env = ble_log_get_env();
    connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
#endif
}

uint8_t ble_log_get_transport_state()
{
    ble_log_env_t *env = ble_log_get_env();
    return env->state;
}

static void ble_minidump_info_send(void)
{
#if defined(SAVE_MINIDUMP_INFO)
    int iCnt = 0;
    int block_len = 2048;
    uint32_t minidump_info_len = assert_minidump_get_size();
    uint8_t *data = bt_mem_alloc(block_len);
    BT_OOM_ASSERT(data);
    if (data)
    {
        uint32_t addr = assert_minidump_get_addr();

        for (iCnt = 0; iCnt < minidump_info_len / block_len; iCnt++)
        {
            memset(data, 0, block_len);
            assert_minidump_read(addr + iCnt * block_len, data, block_len);
            ble_log_send_advance(data, block_len);
        }

        if (minidump_info_len % block_len)
        {
            memset(data, 0, block_len);
            assert_minidump_read(addr + iCnt * block_len, data,  minidump_info_len % block_len);
            ble_log_send_advance(data, minidump_info_len % block_len);
        }
        bt_mem_free(data);
    }
#endif
    return;
}


static void ble_send_thread(void *param)
{
    const char *path_mc;
    int mem_type = 1;
    uint32_t flash_addr = 0;
    uint32_t flash_len = 0;
    ble_log_env_t *env = ble_log_get_env();

#ifdef ULOG_BACKEND_USING_FS
    int ble_log_status = 0;
#endif

    if (env->command == BLE_LOG_GET)
    {
#ifdef ULOG_BACKEND_USING_FS
        ble_log_status = ble_log_onoff(BLE_LOG_COMMAND_GET);
        if (ble_log_status != BLE_LOG_COMMAND_DISABLE)
        {
            ble_log_onoff(BLE_LOG_COMMAND_DISABLE);
        }

        mem_type = ble_log_type_get();
        if (mem_type == 0)
        {
            ble_log_mem_get(&flash_addr, &flash_len);
        }
        else if (mem_type == 1)
        {
            path_mc = ble_log_file_get();
        }
#elif defined(ULOG_BACKEND_USING_RAM)
        ulog_pause(1);
        flash_len = ULOG_RAM_BE_BUF_SIZE;
        mem_type = 2;
#endif

    }


#ifdef SAVE_ASSERT_CONTEXT_IN_FLASH
    if (env->command == BLE_LOG_ASSERT_GET)
    {
        mem_type = ble_assert_type_get();
        if (mem_type == 0)
        {
            ble_assert_mem_get(&flash_addr, &flash_len);
        }
        else if (mem_type == 1)
        {
            path_mc = ble_assert_file_get();
        }

    }
#endif

#ifdef MC_BACKEND_USING_FILE
    if (env->command == BLE_LOG_METRICS_GET)
    {
        mem_type = 1;
        mc_flush();
        path_mc = mc_get_path();
    }
#endif

    LOG_I("curr cmd %d", env->command);

    if (mem_type == 1 && path_mc != NULL) //FS
    {
#ifdef LOG_USING_FS
        int current_read_len = 0;
        int processed_len = 0;
        int block_len = 2048;

        LOG_I("FILE full name %s", path_mc);

        env->state = BLE_LOG_STATE_TRANSPORT;
        ble_log_connection_update();

        int fptr = open(path_mc, O_RDONLY);
        if (fptr != -1)
        {
            int minidump_info_len = 0;
            int file_len = lseek(fptr, 0, SEEK_END);
            lseek(fptr, 0, SEEK_SET);

#if defined(SAVE_MINIDUMP_INFO)
            minidump_info_len = assert_minidump_get_size();
#endif
            ble_log_send_data_response(file_len + minidump_info_len);
            env->send_index = 0;
            LOG_I("log file len %d minidump_info_len:%d", file_len, minidump_info_len);
            ble_minidump_info_send();
            uint8_t *data = bt_mem_alloc(block_len);
            BT_OOM_ASSERT(data);
            if (data)
            {
                while (processed_len < file_len)
                {
                    if (processed_len + block_len > file_len)
                    {
                        current_read_len = file_len - processed_len;
                    }
                    else
                    {
                        current_read_len = block_len;
                    }

                    int ret = read(fptr, data, current_read_len);

                    if (ret != current_read_len)
                    {
                        LOG_E("read failed with %d", ret);
                        break;
                    }

                    ble_log_send_advance(data, current_read_len);
                    processed_len += current_read_len;
                    lseek(fptr, processed_len, SEEK_SET);
                }
                LOG_I("log file len end %d", processed_len);

                bt_mem_free(data);
            }

            if (file_len != 0)
            {
                ble_log_send_data_end();
            }

            close(fptr);
        }
        else
        {
            LOG_I("open failed %s", path_mc);
        }
#endif
    }
    else if (mem_type == 0 && flash_len > 0) //MEM
    {
        int minidump_info_len = 0;
        env->state = BLE_LOG_STATE_TRANSPORT;
        ble_log_connection_update();

#if defined(SAVE_MINIDUMP_INFO)
        minidump_info_len = assert_minidump_get_size();
#endif

        uint8_t *data = (uint8_t *)flash_addr;
        ble_log_send_data_response(flash_len + minidump_info_len);
        env->send_index = 0;
        LOG_I("assert addr 0x%08x, len %d", flash_addr, flash_len);
        ble_minidump_info_send();
        ble_log_buf_send(data, flash_len);

        LOG_I("log file len end %d", flash_len);

        ble_log_send_data_end();
    }
    else if (mem_type == 2 && flash_len > 0) //MEM
    {
        env->state = BLE_LOG_STATE_TRANSPORT;
        ble_log_connection_update();

        ble_ram_be_log_send();
    }

    env->state = BLE_LOG_STATE_NONE;
    ble_log_connection_recover();

    ble_log_send_data_finish();

    if (env->command == BLE_LOG_GET)
    {
#ifdef ULOG_BACKEND_USING_FS
        if (ble_log_status == BLE_LOG_COMMAND_ENABLE)
        {
            ble_log_onoff(BLE_LOG_COMMAND_ENABLE);
        }
#elif defined(ULOG_BACKEND_USING_RAM)
        ulog_pause(0);
#endif
    }
}

static void ble_log_start_send_thread()
{
    rt_thread_t tid;
    tid = rt_thread_create("ble_log_send", ble_send_thread, NULL, 2048, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);

}

#if 0
static void ble_phone_log_send_start_rsp()
{
    uint16_t data_len = 2;
    uint8_t *send_data = (uint8_t *)rt_malloc(data_len);
    *send_data = BLE_LOG_PHONE_DATA_SEND_START_RSP;
    uint8_t status = 0;
    *(send_data + 1) = status;
    ble_log_data_send(send_data, data_len);
    rt_free(send_data);
}

static void ble_log_send_tsdb_result()
{
    uint8_t command = BLE_LOG_TSDB_SWITCH_RESULT;
    uint8_t result = 0;
    uint8_t *send_data = (uint8_t *)rt_malloc(sizeof(uint8_t) + sizeof(uint8_t));
    rt_memcpy(send_data, &command, sizeof(uint8_t));

    rt_memcpy(send_data + sizeof(uint8_t), &result, sizeof(uint8_t));
    ble_log_data_send(send_data, sizeof(uint8_t) + sizeof(uint8_t));
    rt_free(send_data);
}

static void ble_log_get_data_send()
{
    ble_log_env_t *env = ble_log_get_env();
    ble_log_send_data_begin(env->size);
    if (env->size != 0)
    {
        ble_log_start_send_thread();
    }
}
#endif

static void ble_log_send_data_response(uint32_t size)
{
    uint8_t start_flag = BLE_LOG_GET_SEND_START;
    uint8_t *start_data = (uint8_t *)bt_mem_alloc(sizeof(uint32_t) + sizeof(uint8_t));
    BT_OOM_ASSERT(start_data);
    if (start_data)
    {
        rt_memcpy(start_data, &start_flag, sizeof(uint8_t));

        rt_memcpy(start_data + sizeof(uint8_t), &size, sizeof(uint32_t));
        ble_log_data_send(start_data, sizeof(uint32_t) + sizeof(uint8_t));
        bt_mem_free(start_data);
    }
}

#ifdef LOG_USING_FS
static void ble_audio_dump_get_rsp(uint32_t size, char *file_name)
{
    uint8_t command = BLE_AUDIO_DUMP_GET_RSP;
    uint8_t *start_data = (uint8_t *)bt_mem_alloc(sizeof(uint32_t) + sizeof(uint8_t) + strlen(file_name));
    BT_OOM_ASSERT(start_data);
    if (start_data)
    {
        rt_memcpy(start_data, &command, sizeof(uint8_t));

        rt_memcpy(start_data + sizeof(uint8_t), &size, sizeof(uint32_t));
        rt_memcpy(start_data + sizeof(uint8_t) + sizeof(uint32_t), file_name, strlen(file_name));
        ble_log_data_send(start_data, sizeof(uint32_t) + sizeof(uint8_t) + strlen(file_name));
        bt_mem_free(start_data);
    }
}
#endif

static void ble_audio_send_thread(void *param)
{
    ble_log_env_t *env = ble_log_get_env();

#ifdef LOG_USING_FS
    int current_read_len = 0;
    int processed_len = 0;
    int block_len = 2048;
    int file_num = 0;


    char **files = get_audio_dump_files();

    env->state = BLE_LOG_STATE_TRANSPORT;
    ble_log_connection_update();

    int i = 0;
    while (files && files[i] != NULL)
    {
        char *file_name = files[i];

        LOG_I("ble_audio_send_thread %s", file_name);
        int fptr = open(file_name, O_RDONLY);

        current_read_len = 0;
        processed_len = 0;

        if (fptr >= 0)
        {
            int file_len = lseek(fptr, 0, SEEK_END);
            lseek(fptr, 0, SEEK_SET);

            ble_audio_dump_get_rsp(file_len, file_name);
            env->send_index = 0;
            LOG_I("log file len %d, index %d", file_len, i);

            uint8_t *data = bt_mem_alloc(block_len);
            BT_OOM_ASSERT(data);
            if (data)
            {
                while (processed_len < file_len)
                {
                    if (processed_len + block_len > file_len)
                    {
                        current_read_len = file_len - processed_len;
                    }
                    else
                    {
                        current_read_len = block_len;
                    }

                    int ret = read(fptr, data, current_read_len);

                    if (ret != current_read_len)
                    {
                        LOG_E("read failed with %d", ret);
                        break;
                    }

                    ble_log_send_advance(data, current_read_len);
                    processed_len += current_read_len;
                    lseek(fptr, processed_len, SEEK_SET);
                }
                LOG_I("log file len end %d", processed_len);

                bt_mem_free(data);
            }

            if (file_len != 0)
            {
                ble_log_send_data_end();
            }

            close(fptr);
        }
        else
        {
            LOG_I("open failed with %d, index %d", file_num, i);
        }
        i++;
    }
#endif

    env->state = BLE_LOG_STATE_NONE;
    ble_log_connection_recover();

    ble_log_send_data_finish();
}

static void ble_audio_start_send_thread(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("ble_audio_send", ble_audio_send_thread, NULL, 2048, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);
}


static void ble_audio_dump_get_proccess(void)
{
    // TODO: get len
    //uint32_t len = 0;
    //ble_audio_dump_get_rsp(len);
    //ble_log_send_data_finish();

    ble_audio_start_send_thread();
}

static void ble_mem_get_rsp(uint32_t value)
{
    uint8_t *send_data;
    uint16_t send_size = 5;

    send_data = bt_mem_alloc(send_size);
    BT_OOM_ASSERT(send_data);
    if (send_data)
    {
        *send_data = BLE_MEM_GET_RSP;
        memcpy(send_data + 1, &value, sizeof(uint32_t));

        ble_log_data_send(send_data, send_size);
        bt_mem_free(send_data);
    }
}

#define MAX_WRITE_LEN 16

static void ble_mem_get_handler(uint16_t msg_len, uint8_t *msg_data)
{
    LOG_HEX("BLE_MEM_GET_REQ", 16, msg_data, msg_len);
    uint8_t *addr_val;
    addr_val = bt_mem_alloc(msg_len + 1);
    BT_OOM_ASSERT(addr_val);
    if (addr_val)
    {
        memcpy(addr_val, msg_data, msg_len);
        *(addr_val + msg_len) = 0;

        char *addr_str = (char *)addr_val;
        LOG_I("addr %s", addr_str);

        uint32_t *address = (uint32_t *)atoh(addr_str);

        uint32_t val = *address;

        LOG_I("val 0x%x", val);
        ble_mem_get_rsp(val);
        bt_mem_free(addr_val);
    }

    ble_log_send_data_finish();
}

static void ble_mem_set_handler(uint16_t msg_len, uint8_t *msg_data)
{
    LOG_HEX("BLE_MEM_SET", 16, msg_data, msg_len);

    uint16_t addr_len;
    uint16_t val_len;

    memcpy(&addr_len, msg_data, sizeof(uint16_t));
    memcpy(&val_len, msg_data + sizeof(uint16_t) + addr_len, sizeof(uint16_t));


    uint8_t *addr_data;
    uint8_t *val_data;

    addr_data = bt_mem_alloc(addr_len + 1);
    val_data = bt_mem_alloc(val_len + 1);
    BT_OOM_ASSERT(addr_data);
    BT_OOM_ASSERT(val_data);

    if (addr_data && val_data)
    {
        memcpy(addr_data, msg_data + sizeof(uint16_t), addr_len);
        memcpy(val_data, msg_data + sizeof(uint16_t) + sizeof(uint16_t) + addr_len, val_len);

        *(addr_data + addr_len) =
            0;
        *(val_data + val_len) =
            0;

        char *addr_str = (char *)addr_data;
        char *val_str = (char *)val_data;

        LOG_I("addr %s", addr_str);
        LOG_I("val %s", val_str);

        uint32_t *address = (uint32_t *)atoh(addr_str);

        static uint8_t data[MAX_WRITE_LEN];

        hex2data(val_str, (uint8_t *)data, val_len);
        LOG_HEX("BLE_MEM_SET data", 16, data, val_len);

        uint8_t data_order[4];
        for (int i = 0; i < 4; i++)
        {
            data_order[i] = data[4 - 1 - i];
        }

        uint32_t final_data;
        memcpy(&final_data, &data_order, sizeof(uint32_t));
        LOG_I("final data %d", final_data);

        *address = final_data;

        bt_mem_free(addr_data);
        bt_mem_free(val_data);
    }
    else
    {
        if (addr_data)
            bt_mem_free(addr_data);

        if (val_data)
            bt_mem_free(val_data);
    }

    ble_log_send_data_finish();
}

static void ble_log_inquiry_handler()
{
    LOG_I("ble_log_inquiry_handler");

    uint8_t *send_data;
    uint16_t send_size = 7;
    uint8_t send_index = 1;

    send_data = bt_mem_alloc(send_size);
    if (send_data)
    {
        *send_data = BLE_LOG_INQUIRY_RSP;

        *(send_data + send_index) = BLE_LOG_VERSION;

        send_index++;
        *(send_data + send_index) = 4;

        // TODO: get real value
        uint8_t general_log_enable = 0;
        uint8_t metrics_enable = 0;
        uint8_t assert_save_enable = 0;
        uint8_t hci_log_enable = 0;

#if defined(ULOG_BACKEND_USING_RAM) || defined(ULOG_BACKEND_USING_FS)
        general_log_enable = 1;
#endif

#ifdef SAVE_ASSERT_CONTEXT_IN_FLASH
        assert_save_enable = 1;
#endif

#ifdef MC_BACKEND_USING_FILE
        metrics_enable = 1;
#endif

#ifdef LOG_ON_CONSOLE
        hci_log_enable = 1;
#endif

        send_index++;
        *(send_data + send_index) = general_log_enable;

        send_index++;
        *(send_data + send_index) = metrics_enable;

        send_index++;
        *(send_data + send_index) = assert_save_enable;

        send_index++;
        *(send_data + send_index) = hci_log_enable;

        ble_log_data_send(send_data, send_size);
        bt_mem_free(send_data);

    }

    ble_log_send_data_finish();
}

static void ble_log_packet_handler(ble_log_protocol_t *msg, uint16_t length)
{
    ble_log_env_t *env = ble_log_get_env();
    LOG_I("ble_log_packet_handler %d", msg->message_id);
    switch (msg->message_id)
    {

    case BLE_LOG_GET:
    {
        env->command = BLE_LOG_GET;
        ble_log_start_send_thread();
        break;
    }
    case BLE_LOG_CLEAR:
    {
#ifdef ULOG_BACKEND_USING_FS
        ble_log_clear();
#endif
        ble_log_send_data_finish();
        break;
    }
    case BLE_LOG_ON_OFF:
    {
#ifdef ULOG_BACKEND_USING_FS
        uint8_t op = msg->data[0];
        ble_log_onoff(op);
#endif
        ble_log_send_data_finish();
        break;
    }

#ifdef SAVE_ASSERT_CONTEXT_IN_FLASH
    case BLE_LOG_ASSERT_GET:
    {
        env->command = BLE_LOG_ASSERT_GET;
        ble_log_start_send_thread();
        break;
    }
    case BLE_LOG_ASSERT_CLEAR:
    {
        ble_assert_clear();
        ble_log_send_data_finish();
        break;
    }
    case BLE_LOG_ASSERT_ON_OFF:
    {
        uint8_t op = msg->data[0];
        ble_assert_onoff(op);
        ble_log_send_data_finish();
        break;
    }
#endif
    case BLE_AUDIO_DUMP_GET:
    {
        ble_audio_dump_get_proccess();
        break;
    }
    case BLE_AUDIO_DUMP_CLEAR:
    {
        audio_dump_clear();
        ble_log_send_data_finish();
        break;
    }
    case BLE_AUDIO_DUMP_DELAY:
    {
        uint16_t delay;
        memcpy(&delay, msg->data, sizeof(uint16_t));
        webrtc_set_delay(delay);
        ble_log_send_data_finish();
        break;
    }
    case BLE_AUDIO_DUMP_COMMAND:
    {
        uint8_t *audio_command;
        audio_command = bt_mem_alloc(msg->length);
        BT_OOM_ASSERT(audio_command);
        if (audio_command)
        {
            memcpy(audio_command, msg->data, msg->length);
            audio_command_process(audio_command);
            bt_mem_free(audio_command);
        }
        ble_log_send_data_finish();
        break;
    }
    case BLE_AUDIO_DUMP_ENABLE:
    {
        uint8_t file_type = msg->data[0];
        LOG_I("audio dump index=%d", file_type);
        audio_dump_enable(file_type);
        ble_log_send_data_finish();
        break;
    }
    case BLE_MEM_GET_REQ:
    {
        ble_mem_get_handler(msg->length, msg->data);
        break;
    }
    case BLE_MEM_SET:
    {
        ble_mem_set_handler(msg->length, msg->data);
        break;
    }
    case BLE_LOG_METRICS_GET:
    {
        env->command = BLE_LOG_METRICS_GET;
        ble_log_start_send_thread();
        break;
    }
    case BLE_ASSERT_CMD:
    {
        RT_ASSERT(0);
        break;
    }
    case BLE_FINSH_CMD:
    {
        //uint8_t *finsh_command;
        //finsh_command = malloc(msg->length);
        //memcpy(finsh_command, msg->data, msg->length);
        //LOG_HEX("ble finsh", 16, finsh_command, msg->length);
        LOG_HEX("ble finsh", 16, msg->data, msg->length);
        // TODO: process command
#ifdef FINSH_USING_MSH
        extern int msh_exec(char *cmd, rt_size_t length);
        msh_exec((char *)(msg->data), msg->length - 1);
#else
        LOG_I("no define FINSH_USING_MSH, do nothing !");
#endif
        //free(finsh_command);
        ble_log_send_data_finish();
        break;
    }
    case BLE_HCI_LOG_ON_OFF:
    {
        uint8_t op = msg->data[0];
        LOG_I("HCI ONOFF %d", op);
        // TODO: process command, 0 is off, 1 is on
#if defined(BF0_HCPU) && defined(SOC_SF32LB52X)
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif
        if (op >= 1)
        {
            op = 1;
#if defined(ULOG_BACKEND_USING_RAM) && defined(HCI_ON_LOG)
            extern void ulog_ram_buf_init(rt_bool_t realloc, rt_uint32_t size);
            ulog_ram_buf_init(RT_TRUE, HCI_LOG_SIZE);
#endif
        }
        else
        {
#if defined(ULOG_BACKEND_USING_RAM) && defined(HCI_ON_LOG)
            extern void ulog_ram_buf_init(rt_bool_t realloc, rt_uint32_t size);
            ulog_ram_buf_init(RT_TRUE, ULOG_RAM_BE_BUF_SIZE);
#endif
        }
#ifdef BSP_BLE_NVDS_SYNC
        extern void sifli_hci_log_enable(bool is_on);
        sifli_hci_log_enable(op);
#endif

#if defined(BF0_HCPU) && defined(SOC_SF32LB52X)
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif

        ble_log_send_data_finish();
        break;
    }
    case BLE_LOG_INQUIRY_REQ:
    {
        ble_log_inquiry_handler();
        break;
    }
    default:
        break;
    }
}

static void ble_log_serial_callback(ble_serial_tran_event_t event, uint8_t *data)
{
    if (!data)
        return;

    ble_log_env_t *env = ble_log_get_env();
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
                && t_data->cate_id == BLE_LOG_CATEID)
        {
            ble_log_packet_handler((ble_log_protocol_t *)t_data->data, t_data->len);
        }
    }
    break;
    case BLE_SERIAL_TRAN_CLOSE:
    {
        ble_serial_close_t *close = (ble_serial_close_t *)data;
        if (env->handle == close->handle)
        {
            env->is_open = 0;
        }
    }
    break;
    default:
        break;
    }
}

int ble_log_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_log_env_t *env = ble_log_get_env();

    switch (event_id)
    {
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_state = 1;
        env->state = BLE_LOG_STATE_NONE;
        env->mtu = 23;
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->mtu = ind->mtu;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        env->conn_state = 0;
        env->state = BLE_LOG_STATE_NONE;
        break;
    }
    default:
        break;
    }
    return 0;

}

BLE_EVENT_REGISTER(ble_log_ble_event_handler, NULL);


BLE_SERIAL_TRAN_EXPORT(BLE_LOG_CATEID, ble_log_serial_callback);
#endif // DFU_OTA_MANAGER
#endif // BSP_USING_PC_SIMULATOR

#endif //BSP_BLE_LOG

