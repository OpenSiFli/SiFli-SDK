/**
  ******************************************************************************
  * @file   ble_nvds_service.c
  * @author Sifli software development team
  * @brief Source file - Data service for ble nvds.
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
#include "stdlib.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "os_adaptor.h"
#include "bf0_ble_common.h"
#include "bf0_sibles_nvds.h"
#include "drv_common.h"
#include "data_service.h"
#include "ble_nvds_service.h"
#include "mem_section.h"
#include "log.h"
#include "mem_section.h"


#ifdef PKG_USING_EASYFLASH
    #include "ef_cfg.h"
#endif

#ifdef EF_USING_ENV
    #include "easyflash.h"
#elif defined (PKG_USING_FLASHDB)
    #include "flashdb.h"
#endif


#define SIB_NVDS_ERR_CHECK(ret) \
    if (!(ret)) \
        break;
#define SIFLI_NVDS_DB   "SIF_BLE"
#define SIFLI_NVDS_PARTIAL "ble"
#define SIFLI_NVDS_KEY_STACK "SIF_STACK"
#define SIFLI_NVDS_KEY_APP "SIF_APP"
#define SIFLI_NVDS_KEY_CM "SIF_CM"
#define SIBLE_NVDS_POOL_NUM 2

#define BLE_NVDS_INIT_TASK_PRI 20
#define BLE_NVDS_READ_PRI 13
#define BLE_DEFAULT_BDADDR  {{0x12, 0x34, 0x56, 0x78, 0xab, 0xcd}}


#ifdef BSP_USING_PSRAM
    #define BLE_NVDS_KVDB_SECT_BEGIN L2_CACHE_RET_BSS_SECT_BEGIN(ble_kvdb)
    #define BLE_NVDS_KVDB_SECT_END L2_CACHE_RET_BSS_SECT_END
#else
    #define BLE_NVDS_KVDB_SECT_BEGIN L1_NON_RET_BSS_SECT_BEGIN(ble_kvdb)
    #define BLE_NVDS_KVDB_SECT_END   L1_NON_RET_BSS_SECT_END
#endif

typedef struct
{
    datas_handle_t service;
    rt_thread_t tid;
    uint8_t ref_count;
    uint8_t flash_ready;
} ble_nvds_service_env_t;


typedef struct
{
    uint8_t *buffer;
    uint16_t buffer_len;
    uint8_t is_dirty;
} sibles_nvds_temp_buffer_t;

static ble_nvds_service_env_t g_ble_nvds_srv;
static rt_mq_t g_nvds_srv_mq;
static sibles_nvds_temp_buffer_t g_sible_nvds_temp_buf[SIBLE_NVDS_POOL_NUM];
static int32_t ble_nvds_service_msg_handler(datas_handle_t service, data_msg_t *msg);
static uint8_t *sifli_nvds_read_int(sifli_nvds_type_t type, uint16_t *len);
static uint8_t sifli_nvds_write_int(sifli_nvds_type_t type, uint16_t len, uint8_t *ptr);
static uint8_t sifli_nvds_update_tag_value(sifli_nvds_write_tag_t *tag);

static struct rt_thread nvds_thread;
ALIGN(RT_ALIGN_SIZE)
L1_NON_RET_BSS_SECT_BEGIN(nvds_thread_stack)
static char nvds_thread_stack[2048];
L1_NON_RET_BSS_SECT_END



/*
 * default value description:
    0x0D, 02, 0x64, 0x19: Control pre-wakeup time for the sleep of BT subsysm in LCPU. The value is different in RC10K and LXT32K.
    0x12, 0x01, 0x01: Control maximum sleep duration of BT subsystem. the last 0x01 means 10s in BLE only and 30s in dual mode. 0 means 500ms.
    0x2F, 0x04, 0x20, 0x00, 0x00, 0x00: Control the log in Contoller and changed to 0x20, 0x00, 0x09, 0x0 will enable HCI log defautly.
    0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0x AB, 0xCD: The default bd addres
*/

#ifdef FDB_USING_KVDB

static const uint8_t g_ble_slp_default_rc10k[] = {0x0D, 0x02, 0x64, 0x19, 0x12, 0x01, 0x01, 0x2F, 0x04, 0x20, 0x00, 0x00, 0x00,
                                                  0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD
                                                 };

static const uint8_t g_ble_slp_default_lxt32k[] = {0x0D, 0x02, 0xAC, 0x0D, 0x12, 0x01, 0x01, 0x2F, 0x04, 0x20, 0x00, 0x00, 0x00,
                                                   0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD
                                                  };

BLE_NVDS_KVDB_SECT_BEGIN
static struct fdb_kvdb g_ble_db;
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    static uint32_t g_ble_db_cache[256];
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */
BLE_NVDS_KVDB_SECT_END

static fdb_kvdb_t p_ble_db = &g_ble_db;
const static bd_addr_t g_default_bdaddr = BLE_DEFAULT_BDADDR;




static struct fdb_default_kv_node default_ble_kv_set[] =
{
    {SIFLI_NVDS_KEY_STACK, (void *)g_ble_slp_default_lxt32k, sizeof(g_ble_slp_default_lxt32k)},
};
#endif

static data_service_config_t ble_nvds_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .msg_handler = ble_nvds_service_msg_handler,
};



static ble_nvds_service_env_t *ble_nvds_service_get_env(void)
{
    return &g_ble_nvds_srv;
}


size_t sifli_nvds_flash_read(const char *key, void *value_buf, size_t buf_len)
{
    size_t read_len = 0;
#ifdef EF_USING_ENV
    read_len = ef_get_env_blob(key, value_buf, buf_len, NULL);
#elif defined(PKG_USING_FLASHDB)
    struct fdb_blob blob;
    read_len = fdb_kv_get_blob(p_ble_db, key, fdb_blob_make(&blob, value_buf, buf_len));
#endif
    return read_len;
}


uint8_t sifli_nvds_flash_write(const char *key, const void *value_buf, size_t buf_len)
{
    uint8_t ret = NVDS_TAG_NOT_DEFINED;
#ifdef EF_USING_ENV
    EfErrCode ef_ret = ef_set_env_blob(key, value_buf, buf_len);
    if (ef_ret != EF_NO_ERR)
        ret = NVDS_FAIL;
    else
        ret = NVDS_OK;
#elif defined(PKG_USING_FLASHDB)
    struct fdb_blob  blob;
    fdb_err_t err;
    err = fdb_kv_set_blob(p_ble_db, key, fdb_blob_make(&blob, value_buf, buf_len));
    ret = err == FDB_NO_ERR ? NVDS_OK : NVDS_FAIL;
#endif
    return ret;
}

static sifli_nvds_type_t sifli_nvds_type_tag_check(uint8_t tag)
{
    uint8_t check_ret = 0;
    if (tag >= NVDS_STACK_TAG_BD_ADDRESS && tag < NVDS_STACK_TAG_APP_SPECIFIC_FIRST)
        check_ret = SIFLI_NVDS_TYPE_STACK;
    else if (tag >= NVDS_STACK_TAG_APP_SPECIFIC_FIRST && tag <= NVDS_STACK_TAG_APP_SPECIFIC_LAST)
        check_ret = SIFLI_NVDS_TYPE_APP;

    return check_ret;

}


static sifli_nvds_tag_value_t *sifli_nvds_get_tag_value_via_buffer(uint8_t *buffer, uint16_t buffer_len, uint8_t tag)
{
    uint16_t len = buffer_len, val_len;
    sifli_nvds_tag_value_t *val = NULL, *val1 = NULL;

    val = (sifli_nvds_tag_value_t *)buffer;
    while (len && val->len)
    {
        if (val->tag == tag)
        {
            return val;
        }
        val_len = sizeof(sifli_nvds_tag_value_t) + val->len;
        if (len >= val_len)
            len  -= val_len;
        else
            break;
        val = (sifli_nvds_tag_value_t *)((uint8_t *)val + val_len);
    }
    return NULL;
}





static uint8_t sifli_nvds_modify_tag(uint8_t *buffer_pool, uint16_t *buffer_pool_len, uint16_t max_len,
                                     sifli_nvds_tag_value_t *old_tag, sifli_nvds_tag_value_t *new_tag)
{
    uint8_t is_modified = 0;
    if (old_tag->len != new_tag->len)
    {
        /* 1. Check whether could added new tag. */
        if ((new_tag->len > old_tag->len) &&
                (new_tag->len - old_tag->len + *buffer_pool_len > max_len))
            return is_modified;
        /* 2. Remove old_tag. */
        uint16_t old_tag_len = old_tag->len + sizeof(sifli_nvds_tag_value_t);
        sifli_nvds_tag_value_t *next_tag = (sifli_nvds_tag_value_t *)((uint8_t *)old_tag + old_tag->len);
        if (next_tag->len != 0)
            memcpy((uint8_t *)old_tag, (uint8_t *)next_tag, *buffer_pool_len - old_tag_len);
        else
            memset((uint8_t *)old_tag, 0, old_tag_len);
        /* 3. Add new tag. */
        uint16_t new_tag_len = new_tag->len + sizeof(sifli_nvds_tag_value_t);
        memcpy((uint8_t *)buffer_pool + *buffer_pool_len - old_tag_len, new_tag, new_tag_len);
        *buffer_pool_len = *buffer_pool_len - old_tag_len + new_tag_len;
        is_modified = 1;
    }
    else if (memcmp(old_tag->value, new_tag->value, old_tag->len) != 0)
    {
        memcpy(old_tag->value, new_tag->value, old_tag->len);
        is_modified = 1;
    }
    return is_modified;
}


static uint8_t sifli_nvds_add_tag(uint8_t *buffer_pool, uint16_t *buffer_pool_len, uint16_t max_len,
                                  sifli_nvds_tag_value_t *new_tag)
{
    uint8_t is_added = 0;
    uint16_t new_tag_len = sizeof(sifli_nvds_tag_value_t) + new_tag->len;
    if (*buffer_pool_len + new_tag_len > max_len)
        return is_added;
    memcpy((uint8_t *)buffer_pool + *buffer_pool_len, new_tag, new_tag_len);
    is_added = 1;
    *buffer_pool_len = *buffer_pool_len + new_tag_len;

    return is_added;
}


static uint8_t *sifli_nvds_read_int(sifli_nvds_type_t type, uint16_t *len)
{
    uint8_t *ptr = NULL;
    size_t read_len = 0;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        ptr = malloc(SIFLI_NVDS_KEY_LEN_STACK);
        OS_ASSERT(ptr);
        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_STACK);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_STACK, ptr, SIFLI_NVDS_KEY_LEN_STACK);
        // Just assume SIFLI_NVDS_KEY_LEN_STACK is the maximum len
        OS_ASSERT(read_len < SIFLI_NVDS_KEY_LEN_STACK);
        break;
    }
    case SIFLI_NVDS_TYPE_APP:
    {
        ptr = malloc(SIFLI_NVDS_KEY_LEN_APP);
        OS_ASSERT(ptr);
        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_APP);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_APP, ptr, SIFLI_NVDS_KEY_LEN_APP);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        OS_ASSERT(read_len < SIFLI_NVDS_KEY_LEN_APP);
        break;
    }
    case SIFLI_NVDS_TYPE_CM:
    {
        ptr = malloc(SIFLI_NVDS_KEY_LEN_CM);
        OS_ASSERT(ptr);
        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_CM);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_CM, ptr, SIFLI_NVDS_KEY_LEN_CM);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        OS_ASSERT(read_len < SIFLI_NVDS_KEY_LEN_CM);
        break;
    }
    default:
        break;
    }
    *len = (uint16_t)read_len;

    return ptr;
}


static uint8_t sifli_nvds_write_int(sifli_nvds_type_t type, uint16_t len, uint8_t *ptr)
{
    uint8_t ret = NVDS_OK;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        if (len > SIFLI_NVDS_KEY_LEN_STACK)
        {
            ret = NVDS_FAIL;
            break;
        }
        ret = sifli_nvds_flash_write(SIFLI_NVDS_KEY_STACK, ptr, len);
        break;
    }
    case SIFLI_NVDS_TYPE_APP:
    {
        if (len > SIFLI_NVDS_KEY_LEN_APP)
        {
            ret = NVDS_FAIL;
            break;
        }
        ret = sifli_nvds_flash_write(SIFLI_NVDS_KEY_APP, ptr, len);
        break;
    }
    case SIFLI_NVDS_TYPE_CM:
    {
        if (len > SIFLI_NVDS_KEY_LEN_CM)
        {
            ret = NVDS_FAIL;
            break;
        }
        ret = sifli_nvds_flash_write(SIFLI_NVDS_KEY_CM, ptr, len);
        break;
    }
    default:
        ret = NVDS_TAG_NOT_DEFINED;
        break;
    }
    return ret;
}

static uint8_t ble_nvds_update_address_srv(sifli_nvds_update_addr_req_t *req)
{
    uint8_t ret = NVDS_FAIL;

    if (!req || req->u_type == BLE_UPDATE_NO_UPDATE)
        return ret;

    {
        uint8_t addr_loc[NVDS_STACK_LEN_BD_ADDRESS];
        ret = ble_nvds_read_mac_address(addr_loc, NVDS_STACK_LEN_BD_ADDRESS);
        if (ret == NVDS_OK)
        {
            if (memcmp((void *)&req->addr, (void *)addr_loc, NVDS_STACK_LEN_BD_ADDRESS) == 0)
            {
                return ret;
            }
            if (memcmp((void *)&g_default_bdaddr, (void *)addr_loc, NVDS_STACK_LEN_BD_ADDRESS) == 0)
            {
                req->u_type = BLE_UPDATE_ALWAYS;
            }
        }

        sifli_nvds_write_tag_t *update_tag = malloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
        if (update_tag)
        {
            update_tag->is_flush = req->is_flush;
            update_tag->type = req->u_type;
            update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
            update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;
            memcpy((void *)update_tag->value.value, (void *)&req->addr, NVDS_STACK_LEN_BD_ADDRESS);

            ret = sifli_nvds_update_tag_value(update_tag);
            free(update_tag);
        }
        else
            ret = NVDS_NO_SPACE_AVAILABLE;
    }

    return ret;
}



static void sifli_nvds_flush_int(void)
{
    uint8_t ret;
    for (uint32_t i = 0; i < SIBLE_NVDS_POOL_NUM; i++)
    {
        if (g_sible_nvds_temp_buf[i].buffer)
        {
            if (g_sible_nvds_temp_buf[i].is_dirty)
            {
                ret = sifli_nvds_write_int(i + 1, g_sible_nvds_temp_buf[i].buffer_len, g_sible_nvds_temp_buf[i].buffer);
                LOG_HEX("nvds_cache", 16, (uint8_t *)g_sible_nvds_temp_buf[i].buffer, g_sible_nvds_temp_buf[i].buffer_len);
                if (ret != NVDS_OK)
                    LOG_E("nvds(%d) flush failed", i);
            }
            free(g_sible_nvds_temp_buf[i].buffer);
            memset(&g_sible_nvds_temp_buf[i], 0, sizeof(sibles_nvds_temp_buffer_t));
        }
    }
}


static uint8_t sifli_nvds_update_tag_value(sifli_nvds_write_tag_t *tag)
{
    uint8_t ret = NVDS_FAIL;
    uint8_t is_dirty = 0;
    uint16_t buffer_pool_max_len[] = {SIFLI_NVDS_KEY_LEN_STACK, SIFLI_NVDS_KEY_LEN_APP};
    uint8_t index;
    do
    {
        sifli_nvds_type_t ble_type;
        uint16_t read_len;
        uint8_t *buffer;

        if (tag->type == BLE_UPDATE_NO_UPDATE)
            break;

        SIB_NVDS_ERR_CHECK(tag);

        ble_type =  sifli_nvds_type_tag_check(tag->value.tag);
        SIB_NVDS_ERR_CHECK(ble_type);

        index = ble_type - 1;
        if (g_sible_nvds_temp_buf[index].buffer == NULL)
            g_sible_nvds_temp_buf[index].buffer = sifli_nvds_read_int(ble_type, &g_sible_nvds_temp_buf[index].buffer_len);
        SIB_NVDS_ERR_CHECK(g_sible_nvds_temp_buf[index].buffer);

        read_len = g_sible_nvds_temp_buf[index].buffer_len;
        buffer =  g_sible_nvds_temp_buf[index].buffer;

        if (read_len)
        {
            sifli_nvds_tag_value_t *search_tag = sifli_nvds_get_tag_value_via_buffer(buffer, read_len, tag->value.tag);
            if (search_tag)
            {
                /* Only update the tag for SIFLI_NVDS_UPDATE_ALWAYS. */
                if (tag->type == BLE_UPDATE_ALWAYS)
                {
                    is_dirty = sifli_nvds_modify_tag(buffer, &g_sible_nvds_temp_buf[index].buffer_len,
                                                     buffer_pool_max_len[index], search_tag, &tag->value);
                }
                ret = NVDS_OK;
                break;
            }
        }

        is_dirty = sifli_nvds_add_tag(buffer, &g_sible_nvds_temp_buf[index].buffer_len, buffer_pool_max_len[index],
                                      &tag->value);
        ret = NVDS_OK;
    }
    while (0);

    if (ret == NVDS_OK)
    {
        g_sible_nvds_temp_buf[index].is_dirty |= is_dirty;
    }

    if (tag->is_flush)
        sifli_nvds_flush_int();
    return ret;

}


static uint8_t sifli_nvds_read_tag_value_int(uint8_t tag, uint16_t *length, uint8_t *tag_buffer)
{
    uint8_t ret = NVDS_FAIL;
    sifli_nvds_tag_value_t *tag_val = NULL;
    uint8_t *buffer = NULL;
    do
    {
        SIB_NVDS_ERR_CHECK((length != NULL) && (tag_buffer != NULL));

        uint8_t ble_type = sifli_nvds_type_tag_check(tag);
        SIB_NVDS_ERR_CHECK(ble_type);

        uint16_t read_len;
        buffer = sifli_nvds_read_int(ble_type, &read_len);
        SIB_NVDS_ERR_CHECK(buffer);

        tag_val = sifli_nvds_get_tag_value_via_buffer(buffer, read_len, tag);

    }
    while (0);

    if (tag_val)
    {
        uint16_t len = tag_val->len >= *length ? *length : tag_val->len;
        memcpy(tag_buffer, tag_val->value, len);
        *length = len;
        ret = NVDS_OK;
    }

    if (buffer)
        free(buffer);
    return ret;
}


static rt_err_t ble_nvds_task_priority_increase(ble_nvds_service_env_t *env)
{
    uint8_t read_priority = BLE_NVDS_READ_PRI;
    return rt_thread_control(env->tid, RT_THREAD_CTRL_CHANGE_PRIORITY, &read_priority);;
}


static rt_err_t ble_nvds_task_priority_recovery(ble_nvds_service_env_t *env)
{
    uint8_t read_priority = BLE_NVDS_INIT_TASK_PRI;
    return rt_thread_control(env->tid, RT_THREAD_CTRL_CHANGE_PRIORITY, &read_priority);;
}


static int32_t ble_nvds_service_subscribe(datas_handle_t service)
{
    ble_nvds_service_env_t *env = ble_nvds_service_get_env();

    env->ref_count++;
    return 0;
}

static int32_t ble_nvds_service_unsubscribe(datas_handle_t service)
{
    ble_nvds_service_env_t *env = ble_nvds_service_get_env();

    if (env->ref_count > 0)
        env->ref_count--;
    return 0;
}

static int32_t ble_nvds_service_config(datas_handle_t service, void *config)
{
    return 0;
}



static int32_t ble_nvds_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    ble_nvds_service_env_t *env = ble_nvds_service_get_env();

    LOG_I("nvds msg_id(%d)", msg->msg_id);
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        ble_nvds_service_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        ble_nvds_service_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        int32_t result = ble_nvds_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_TX_REQ:
    {
        break;
    }
    case MSG_SERVICE_RX_REQ:
    {
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        break;
    }
    case BLE_NVDS_SERVICE_READ:
    {
        /* Increase task priority to avoid read blocking. */
        ble_nvds_task_priority_increase(env);
        sifli_nvds_get_value_t *data = (sifli_nvds_get_value_t *)(data_service_get_msg_body(msg));
        RT_ASSERT(data);
        uint16_t read_len;
        sifli_nvds_get_value_cnf_t *cnf;
        uint8_t *ptr = sifli_nvds_read_int(data->type, &read_len);
        uint16_t rsp_len = sizeof(sifli_nvds_get_value_cnf_t) + read_len;
        cnf = malloc(rsp_len);
        RT_ASSERT(cnf);

        cnf->type = data->type;
        cnf->len = read_len;
        if (read_len && ptr)
        {
            memcpy(cnf->value, ptr, read_len);
            cnf->status = NVDS_OK;
        }
        else
            cnf->status = NVDS_TAG_NOT_DEFINED;
        datas_send_response_data(service, msg, (uint32_t)rsp_len, (uint8_t *)cnf);
        free(cnf);
        if (ptr)
            free(ptr);
        /* Recovery origin priority. */
        ble_nvds_task_priority_recovery(env);
        break;
    }
    case BLE_NVDS_SERVICE_READ_TAG:
    {
        uint8_t ret;
        sifli_nvds_read_tag_t *tag = (sifli_nvds_read_tag_t *)(data_service_get_msg_body(msg));
        RT_ASSERT(tag);

        /* malloc maximum buffer */
        sifli_nvds_read_tag_cnf_t *cnf = malloc(sizeof(sifli_nvds_read_tag_cnf_t) + tag->length);
        RT_ASSERT(cnf);

        cnf->tag = tag->tag;
        cnf->status = sifli_nvds_read_tag_value_int(tag->tag, &tag->length, (uint8_t *)&cnf->buffer);
        cnf->length = tag->length;

        datas_send_response_data(service, msg, (uint32_t)(sizeof(sifli_nvds_read_tag_cnf_t) + cnf->length), (uint8_t *)cnf);
        free(cnf);
        break;
    }
    case BLE_NVDS_SERVICE_WRITE:
    {
        sifli_nvds_set_value_t *data = (sifli_nvds_set_value_t *)(data_service_get_msg_body(msg));
        RT_ASSERT(data);
        sifli_nvds_set_value_cnf_t cnf;

        cnf.type = data->type;
        cnf.status = sifli_nvds_write_int(data->type, data->len, data->value);

        datas_send_response_data(service, msg, (uint32_t)sizeof(sifli_nvds_set_value_cnf_t), (uint8_t *)&cnf);
        break;
    }
    case BLE_NVDS_SERVICE_WRITE_TAG:
    {
        sifli_nvds_write_tag_t *data = (sifli_nvds_write_tag_t *)(data_service_get_msg_body(msg));
        sifli_nvds_write_tag_cnf_t tag_cnf;
        tag_cnf.tag = data->value.tag;
        tag_cnf.status = sifli_nvds_update_tag_value(data);
        datas_send_response_data(service, msg, (uint32_t)sizeof(sifli_nvds_write_tag_cnf_t), (uint8_t *)&tag_cnf);
        break;
    }
    case BLE_NVDS_SERVICE_FLUSH:
    {
        sifli_nvds_flush_int();
        datas_send_response(service, msg, RT_EOK);
        break;
    }
    case BLE_NVDS_SERVICE_UPDATE_ADDR:
    {
        sifli_nvds_update_addr_req_t *req = (sifli_nvds_update_addr_req_t *)(data_service_get_msg_body(msg));
        sifli_nvds_update_addr_rsp_t cnf;
        uint8_t res = ble_nvds_update_address_srv(req);
        LOG_D("update addr res %d", res);
        cnf.status = res;
        datas_send_response_data(service, msg, (uint32_t)sizeof(sifli_nvds_update_addr_rsp_t), (uint8_t *)&cnf);
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}

void ble_db_init(ble_nvds_service_env_t *env)
{
#ifdef FDB_USING_KVDB
    struct fdb_default_kv default_kv;
    fdb_err_t err;

    RT_ASSERT(env);
#ifndef BSP_USING_PC_SIMULATOR
    if (HAL_LXT_DISABLED())
    {
        default_ble_kv_set[0].value = (void *)g_ble_slp_default_rc10k;
        default_ble_kv_set[0].value_len = sizeof(g_ble_slp_default_rc10k);
    }
#endif
    default_kv.kvs = default_ble_kv_set;
    default_kv.num = sizeof(default_ble_kv_set) / sizeof(default_ble_kv_set[0]);
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    default_kv.kv_cache_pool = g_ble_db_cache;
    default_kv.kv_cache_pool_size = sizeof(g_ble_db_cache);
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */

    memset(p_ble_db, 0, sizeof(*p_ble_db));
    err = fdb_kvdb_init(p_ble_db, SIFLI_NVDS_DB, SIFLI_NVDS_PARTIAL, &default_kv, NULL);
    if (err != FDB_NO_ERR)
        LOG_E("nvds init failed !!!");
    else
        env->flash_ready = 1;
#endif
}

uint8_t ble_nvds_read_mac_address(uint8_t *addr, uint8_t len)
{
    uint8_t ret = NVDS_FAIL;
    if (!addr || len != NVDS_STACK_LEN_BD_ADDRESS)
        return ret;

    uint16_t tag_len = NVDS_STACK_LEN_BD_ADDRESS;
    return sifli_nvds_read_tag_value_int(NVDS_STACK_TAG_BD_ADDRESS, &tag_len, addr);
}

#ifdef NVDS_AUTO_UPDATE_MAC_ADDRESS
static void ble_nvds_auto_update_address(void)
{
    uint8_t type = *(volatile uint8_t *)AUTO_FLASH_MAC_ADDRESS;
    uint8_t len = *(volatile uint8_t *)(AUTO_FLASH_MAC_ADDRESS + 1);
    volatile uint8_t *data_ptr = (volatile uint8_t *)(AUTO_FLASH_MAC_ADDRESS + 2);
    uint8_t addr[NVDS_STACK_LEN_BD_ADDRESS];
    if (type == 0x01 && len == NVDS_STACK_LEN_BD_ADDRESS)
    {
        uint8_t ret = ble_nvds_read_mac_address(addr, NVDS_STACK_LEN_BD_ADDRESS);
        if (ret == NVDS_OK)
        {
            if (memcmp((void *)data_ptr, (void *)addr, NVDS_STACK_LEN_BD_ADDRESS) == 0)
            {
                return;
            }
        }

        sifli_nvds_write_tag_t *update_tag = malloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);

        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
        update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;
        memcpy((void *)update_tag->value.value, (void *)data_ptr, NVDS_STACK_LEN_BD_ADDRESS);

        sifli_nvds_update_tag_value(update_tag);
        free(update_tag);
    }
}
#endif

#define EXT_WAKEUP_TIME_LXT32K   3500
#define EXT_WAKEUP_TIME_RC10K    4500


void ble_nvds_update_wakeup_time(void)
{
    uint16_t wakeup_time;

    uint16_t tag_len = NVDS_STACK_LEN_EXT_WAKEUP_TIME;
    uint8_t ret = sifli_nvds_read_tag_value_int(NVDS_STACK_TAG_EXT_WAKEUP_TIME, &tag_len, (uint8_t *)&wakeup_time);
    if (ret == NVDS_OK)
    {
        LOG_I("read sleep time %d", wakeup_time);
#ifndef BSP_USING_PC_SIMULATOR
        if (HAL_LXT_DISABLED())
        {
            if (wakeup_time == EXT_WAKEUP_TIME_RC10K)
                return;
        }
        else
#endif
        {
            if (wakeup_time == EXT_WAKEUP_TIME_LXT32K)
                return;
        }
    }
#ifndef BSP_USING_PC_SIMULATOR
    if (HAL_LXT_DISABLED())
        wakeup_time = EXT_WAKEUP_TIME_RC10K;
    else
#endif
        wakeup_time = EXT_WAKEUP_TIME_LXT32K;

    sifli_nvds_write_tag_t *update_tag = malloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_EXT_WAKEUP_TIME);

    update_tag->is_flush = 1;
    update_tag->type = BLE_UPDATE_ALWAYS;
    update_tag->value.tag = NVDS_STACK_TAG_EXT_WAKEUP_TIME;
    update_tag->value.len = NVDS_STACK_LEN_EXT_WAKEUP_TIME;
    memcpy((void *)update_tag->value.value, (void *)&wakeup_time, NVDS_STACK_LEN_EXT_WAKEUP_TIME);

    sifli_nvds_update_tag_value(update_tag);
    free(update_tag);
}


int ble_nvds_service_register(void)
{
    ble_nvds_service_env_t *env = ble_nvds_service_get_env();
    rt_err_t err;

    ble_db_init(env);

    if (env->flash_ready)
    {
#ifdef NVDS_AUTO_UPDATE_MAC_ADDRESS
        ble_nvds_auto_update_address();
#endif
        ble_nvds_update_wakeup_time();
        rt_thread_t tid;
        g_nvds_srv_mq = rt_mq_create("nvds_srv", sizeof(data_msg_t), 15, RT_IPC_FLAG_FIFO);
        ble_nvds_service_cb.queue = g_nvds_srv_mq;
        env->tid = &nvds_thread;
        err = rt_thread_init(&nvds_thread, "nvds_srv", data_service_entry, g_nvds_srv_mq,
                             nvds_thread_stack, sizeof(nvds_thread_stack), BLE_NVDS_INIT_TASK_PRI, 10);
        RT_ASSERT(RT_EOK == err);
        rt_thread_startup(env->tid);
        LOG_D("tid %x", env->tid);
        env->service = datas_register("BLE_NV", &ble_nvds_service_cb);
    }

    return 0;
}
INIT_PRE_APP_EXPORT(ble_nvds_service_register);

#if defined(BSP_USING_PM) && !defined(BSP_USING_PSRAM)
static int ble_pm_suspend(const struct rt_device *device, uint8_t mode)
{
    int r = RT_EOK;

    return r;
}

void ble_pm_resume(const struct rt_device *device, uint8_t mode)
{
    if ((PM_SLEEP_MODE_STANDBY != mode)
            && (PM_SLEEP_MODE_DEEP != mode))
    {
        return;
    }

    ble_db_init(ble_nvds_service_get_env());

    return;
}


const struct rt_device_pm_ops ble_pm_op =
{
    .suspend = ble_pm_suspend,
    .resume = ble_pm_resume,
};


int ble_pm_init(void)
{
    rt_pm_device_register(NULL, &ble_pm_op);

    return 0;
}
INIT_APP_EXPORT(ble_pm_init);
#endif /* BSP_USING_PM */






void sifli_nvds_flash_reset(const char *key)
{
#ifdef PKG_USING_FLASHDB
    if (!key)
        fdb_kv_set_default(p_ble_db);
    else
        fdb_kv_del(p_ble_db, key);
#endif
}

static void silif_nvds_reset(sifli_nvds_type_t type)
{
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        sifli_nvds_flash_reset(SIFLI_NVDS_KEY_STACK);
        break;
    }
    case SIFLI_NVDS_TYPE_APP:
    {
        sifli_nvds_flash_reset(SIFLI_NVDS_KEY_APP);
        break;
    }
    default:
        break;
    }

}


/**
    nvds write <type> <total len> <offset> <data in hex string, such as 123ABC, data will be {0x12, 0x3A, 0xBC}>
    nvds read  <type> <total len>
*/
static void nvds(uint8_t argc, char **argv)
{
    char *value = NULL;

    if (argc > 1)
    {
        if (strcmp(argv[1], "write") == 0)
        {
            sifli_nvds_type_t type;
            static uint8_t *data = NULL;
            uint16_t len, offset;
            type = atoi(argv[2]);
            len = atoi(argv[3]);
            offset = atoi(argv[4]);
            if (data == NULL)
                data = malloc(len);
            if (len > offset)
            {
                hex2data(argv[5], data + offset, len - offset);
                LOG_I("Cache to buff, offset=%d, length=%d\n", offset, strlen(argv[5]) >> 1);
                offset += (strlen(argv[5]) >> 1);
            }
            if (len == offset)
            {
                LOG_I("Write to flash, length=%d\n", len);
                LOG_HEX("nvds_write", 16, (uint8_t *)data, len);
                sifli_nvds_write_int(type, len, data);
                free(data);
                data = NULL;
            }
        }
        else if (strcmp(argv[1], "read") == 0)
        {
            sifli_nvds_type_t type;
            uint8_t *data = NULL;
            uint16_t len, offset;
            type = atoi(argv[2]);
            data = sifli_nvds_read_int(type, &len);
            HAL_DBG_print_data((char *)data, 0, len);
            LOG_HEX("nvds_read", 16, (uint8_t *)data, len);
            free(data);
        }
        else if (strcmp(argv[1], "get_mac") == 0)
        {
            uint8_t addr[NVDS_STACK_LEN_BD_ADDRESS] = {0};
            ble_nvds_read_mac_address(addr, NVDS_STACK_LEN_BD_ADDRESS);
            LOG_HEX("MAC-addr", 16, (uint8_t *)addr, NVDS_STACK_LEN_BD_ADDRESS);
        }
        else if (strcmp(argv[1], "reset") == 0)
        {
            sifli_nvds_type_t type = atoi(argv[2]);
            silif_nvds_reset(type);
        }
        else if (strcmp(argv[1], "reset_all") == 0)
        {
            sifli_nvds_flash_reset(NULL);
        }
        else if (strcmp(argv[1], "update") == 0)
        {
            if (strcmp(argv[2], "addr") == 0)
            {
                uint16_t len, offset;
                len = atoi(argv[3]);
                if (len != NVDS_STACK_LEN_BD_ADDRESS)
                    return;

                sifli_nvds_write_tag_t *update_tag = malloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
                if (update_tag == NULL)
                    return;
                hex2data(argv[4], update_tag->value.value, NVDS_STACK_LEN_BD_ADDRESS);
                update_tag->is_flush = 1;
                update_tag->type = BLE_UPDATE_ALWAYS;
                update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
                update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;

                sifli_nvds_update_tag_value(update_tag);
                free(update_tag);
            }
            else if (strcmp(argv[2], "hci_log") == 0)
            {
                uint8_t is_on = atoi(argv[3]);
                uint32_t config[] = {0x20, 0x090020};

                if (is_on > 1)
                    is_on = 1;

                sifli_nvds_write_tag_t *update_tag = malloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_TRACER_CONFIG);
                if (update_tag == NULL)
                    return;
                memcpy((void *)update_tag->value.value, (void *)&config[is_on], NVDS_STACK_LEN_TRACER_CONFIG);
                update_tag->is_flush = 1;
                update_tag->type = BLE_UPDATE_ALWAYS;
                update_tag->value.tag = NVDS_STACK_TAG_TRACER_CONFIG;
                update_tag->value.len = NVDS_STACK_LEN_TRACER_CONFIG;

                sifli_nvds_update_tag_value(update_tag);
                free(update_tag);

            }
        }
    }
}
MSH_CMD_EXPORT(nvds, Sifli NVDS command);


