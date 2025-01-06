/**
  ******************************************************************************
  * @file   bf0_sibles_nvds.c
  * @author Sifli software development team
  * @brief SIFLI BLE utility source.
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
#include "bf0_sibles_internal.h"
#include "bf0_sibles_util.h"
#include "bf0_sibles_nvds.h"
#include "bf0_sibles.h"
#ifdef BSP_USING_PC_SIMULATOR
    #include "bf0_hal_hlp.h"
#else
    #include "drv_common.h"
#endif
// BSP_USING_PC_SIMULATOR
#include "mem_section.h"

#if defined (BSP_USING_SPI_FLASH) && defined(BF0_HCPU)
    #include "drv_flash.h"
#endif


#define LOG_TAG "nvds"
#include "log.h"


// Support in different core.


#ifdef BSP_BLE_NVDS_SYNC


#ifdef PKG_USING_EASYFLASH
    #include "ef_cfg.h"
#endif

#ifdef EF_USING_ENV
    #include "easyflash.h"
#elif defined (PKG_USING_FLASHDB)
    #include "flashdb.h"
    #include "fal.h"
#endif

#ifdef FDB_USING_FILE_MODE
    #include "dfs_posix.h"
#endif /* FDB_USING_FILE_MODE */


#if defined(NVDS_AUTO_UPDATE_MAC_ADDRESS) && defined (BSP_USING_SPI_FLASH) && defined(BF0_HCPU)
    #define NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE
    #include "bf0_sys_cfg.h"
#endif // defined(NVDS_AUTO_UPDATE_MAC_ADDRESS) && defined (BSP_USING_SPI_FLASH) && defined(BF0_HCPU)

#define SIFLI_NVDS_DB   "SIF_BLE"
#define SIFLI_NVDS_PARTIAL "ble"
#define SIFLI_NVDS_KEY_STACK "SIF_STACK"
#define SIFLI_NVDS_KEY_APP "SIF_APP"
#define SIFLI_NVDS_KEY_CM "SIF_CM"
#define SIFLI_NVDS_KEY_BT_HOST "SIF_BT"
#define SIFLI_NVDS_KEY_BT_CM "SIF_BT_CM"
#define BLE_DEFAULT_BDADDR  {{0x12, 0x34, 0x56, 0x78, 0xab, 0xcd}}



#define SIBLE_NVDS_POOL_NUM 2
#define SIB_NVDS_ERR_CHECK(ret) \
    if (!(ret)) \
        break;

#ifdef BSP_USING_PSRAM
    #define BLE_NVDS_KVDB_SECT_BEGIN  L2_CACHE_RET_BSS_SECT_BEGIN(ble_kvdb)
    #define BLE_NVDS_KVDB_SECT_END    L2_CACHE_RET_BSS_SECT_END
#else
    #define BLE_NVDS_KVDB_SECT_BEGIN L1_NON_RET_BSS_SECT_BEGIN(ble_kvdb)
    #define BLE_NVDS_KVDB_SECT_END   L1_NON_RET_BSS_SECT_END
#endif




typedef struct
{
    uint8_t *buffer;
    uint16_t buffer_len;
    uint8_t is_dirty;
} sibles_nvds_temp_buffer_t;

typedef struct
{
    uint8_t is_init;
} bt_nvds_env_t;


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
*/
static sibles_nvds_temp_buffer_t g_sible_nvds_temp_buf[SIBLE_NVDS_POOL_NUM];
static bt_nvds_env_t g_bt_nvds_env;


/*
 * default value description:
    0x0D, 02, 0x64, 0x19: Control pre-wakeup time for the sleep of BT subsysm in LCPU. The value is different in RC10K and LXT32K.
    0x12, 0x01, 0x01: Control maximum sleep duration of BT subsystem. the last 0x01 means 10s in BLE only and 30s in dual mode. 0 means 500ms.
    0x2F, 0x04, 0x20, 0x00, 0x00, 0x00: Control the log in Contoller and changed to 0x20, 0x00, 0x09, 0x0 will enable HCI log defautly.
    0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0x AB, 0xCD: The default bd addres
    0x15, 0x01, 0x01: Internal usage, for scheduling.
*/


static const uint8_t g_ble_slp_default_rc10k[] = {0x0D, 0x02, 0x64, 0x19, 0x12, 0x01, 0x01, 0x2F, 0x04, 0x20, 0x00, 0x00, 0x00,

                                                  0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0x15, 0x01, 0x01
                                                 };

#ifdef SOC_SF32LB55X
static const uint8_t g_ble_slp_default_lxt32k[] = {0x0D, 0x02, 0xAC, 0x0D, 0x12, 0x01, 0x01, 0x2F, 0x04, 0x20, 0x00, 0x00, 0x00,
                                                   0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD
                                                  };
#else
static const uint8_t g_ble_slp_default_lxt32k[] = {0x2F, 0x04, 0x20, 0x00, 0x00, 0x00, 0x01, 0x06, 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0x15, 0x01, 0x01
                                                  };
#endif

#ifdef FDB_USING_KVDB
BLE_NVDS_KVDB_SECT_BEGIN
static struct fdb_kvdb g_ble_db;
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    static uint32_t g_ble_db_cache[256];
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */

BLE_NVDS_KVDB_SECT_END

static fdb_kvdb_t p_ble_db = &g_ble_db;



static struct fdb_default_kv_node default_ble_kv_set[] =
{
    {SIFLI_NVDS_KEY_STACK, (void *)g_ble_slp_default_lxt32k, sizeof(g_ble_slp_default_lxt32k)},
};
#endif

const static bd_addr_t g_default_bdaddr = BLE_DEFAULT_BDADDR;


static uint8_t *sifli_nvds_read_int(sifli_nvds_type_t type, uint16_t *len);
static void sifli_nvds_init_db(void);
static uint8_t sifli_nvds_construct_default_val(sifli_nvds_type_t type, uint8_t *ptr, uint16_t *len);


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/


/*
 * Env implement
 ****************************************************************************************
*/

static bt_nvds_env_t *sifli_nvds_get_env(void)
{
    return &g_bt_nvds_env;
}

static uint8_t sifli_nvds_is_init(void)
{
    return g_bt_nvds_env.is_init;
}


/*
 * Porting APIs
 ****************************************************************************************
*/


#ifdef FDB_USING_KVDB
static struct rt_mutex ble_flash_mutex;

static void lock(fdb_db_t db)
{
    rt_err_t err = rt_mutex_take(&ble_flash_mutex, RT_WAITING_FOREVER);
    //rt_kprintf("lock\n");
}

// when db_unlock() run,this fun run. of course fdb_kvdb_control(kvdb_msg, FDB_KVDB_CTRL_SET_LOCK, unlock); nust be init
static void unlock(fdb_db_t db)
{
    rt_err_t err = rt_mutex_release(&ble_flash_mutex);
    //rt_kprintf("unlock\n");
}
#endif

// Return read size, 0 means return failed
__WEAK size_t sifli_nvds_flash_adaptor_read(const char *key, void *value_buf, size_t buf_len)
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


// Return NVDS_STATUS
__WEAK uint8_t sifli_nvds_flash_adaptor_write(const char *key, const void *value_buf, size_t buf_len)
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


// Return NVDS_STATUS
__WEAK uint8_t sifli_nvds_flash_adaptor_init(void)
{
    uint8_t ret = NVDS_FAIL;
#ifdef FDB_USING_KVDB

    fdb_err_t err;

    memset(p_ble_db, 0, sizeof(*p_ble_db));

    fdb_kvdb_control(p_ble_db, FDB_KVDB_CTRL_SET_LOCK, lock);
    fdb_kvdb_control(p_ble_db, FDB_KVDB_CTRL_SET_UNLOCK, unlock);

    do
    {
        const char *path = SIFLI_NVDS_PARTIAL;
#ifdef FDB_USING_FILE_MODE
        const struct fal_partition *fal = fal_partition_find(SIFLI_NVDS_PARTIAL);
        RT_ASSERT(fal);

        /*Compatible ble nvds path problems*/
        if (0 != access(path, 0))
        {
#ifdef SOLUTION_WATCH
            path = fal->path_name;
#endif
        }

        int sec_size = PKG_FLASHDB_ERASE_GRAN;
        int max_size = fal->len;
        bool file_mode = true;
        LOG_I("ble db init: sector_size %d size %d", sec_size, max_size);
        fdb_kvdb_control(p_ble_db, FDB_KVDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
        fdb_kvdb_control(p_ble_db, FDB_KVDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
        fdb_kvdb_control(p_ble_db, FDB_KVDB_CTRL_SET_FILE_MODE, (void *)&file_mode);
        if (0 != access(path, 0) && 0 != mkdir(path, 0))
        {
            LOG_E("create db %s fail", SIFLI_NVDS_PARTIAL);
            break;
        }

#endif /* FDB_USING_FILE_MODE */

        err = fdb_kvdb_init(p_ble_db, SIFLI_NVDS_DB, path, NULL, NULL);
        if (err != FDB_NO_ERR)
        {
            LOG_E("nvds init failed !!!");
            break;
        }

        ret = NVDS_OK;
    }
    while (0);

#endif
    return ret;
}


__WEAK uint8_t sifli_nvds_flash_adaptor_delete(const char *key)
{
    uint8_t ret = NVDS_FAIL;
#ifdef PKG_USING_FLASHDB
    if (!key)
        fdb_kv_set_default(p_ble_db);
    else
        fdb_kv_del(p_ble_db, key);
    ret = NVDS_OK;
#endif
    return ret;
}



/*
 * NVDS opeartion
 ****************************************************************************************
*/


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


uint8_t sifli_nvds_write_tag_value_via_buffer(uint8_t *buffer, uint16_t *buffer_len, uint16_t max_len, sifli_nvds_write_tag_t *tag)
{
    uint8_t ret = NVDS_FAIL;
    uint8_t is_dirty = 0;
    do
    {
        SIB_NVDS_ERR_CHECK((tag != 0) && (buffer != 0) && (buffer_len != 0) && (*buffer_len != 0));

        if (tag->type == BLE_UPDATE_NO_UPDATE)
            break;

        {
            sifli_nvds_tag_value_t *search_tag = sifli_nvds_get_tag_value_via_buffer(buffer, *buffer_len, tag->value.tag);
            if (search_tag)
            {
                /* Only update the tag for SIFLI_NVDS_UPDATE_ALWAYS. */
                if (tag->type == BLE_UPDATE_ALWAYS)
                {
                    is_dirty = sifli_nvds_modify_tag(buffer, buffer_len,
                                                     max_len, search_tag, &tag->value);
                }
                ret = NVDS_OK;
                break;
            }
        }


        is_dirty = sifli_nvds_add_tag(buffer, buffer_len, max_len, &tag->value);
        ret = NVDS_OK;
    }
    while (0);

    return ret;

}

uint8_t sifli_nvds_write_tag_value(sifli_nvds_write_tag_t *tag)
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
        sifli_nvds_flush();

    return ret;
}



uint8_t sifli_nvds_read_tag_value(sifli_nvds_read_tag_t *tag, uint8_t *tag_buffer)
{
    int8_t ret = NVDS_FAIL;
    sifli_nvds_tag_value_t *tag_val = NULL;
    uint8_t *buffer = NULL;
    do
    {
        SIB_NVDS_ERR_CHECK((tag != NULL) && (tag_buffer != NULL));

        uint8_t ble_type = sifli_nvds_type_tag_check(tag->tag);
        SIB_NVDS_ERR_CHECK(ble_type);

        uint16_t read_len;
        buffer = sifli_nvds_read_int(ble_type, &read_len);
        SIB_NVDS_ERR_CHECK(buffer);

        tag_val = sifli_nvds_get_tag_value_via_buffer(buffer, read_len, tag->tag);

    }
    while (0);

    if (tag_val)
    {
        uint16_t len = tag_val->len >= tag->length ? tag->length : tag_val->len;
        memcpy(tag_buffer, tag_val->value, len);
        tag->length = len;
        ret = NVDS_OK;
    }

    if (buffer)
        bt_mem_free(buffer);
    return ret;
}


uint8_t sifli_nvds_flush(void)
{
    uint8_t ret;
    for (uint32_t i = 0; i < SIBLE_NVDS_POOL_NUM; i++)
    {
        if (g_sible_nvds_temp_buf[i].buffer)
        {
            if (g_sible_nvds_temp_buf[i].is_dirty)
            {
                ret = sifli_nvds_write(i + 1, g_sible_nvds_temp_buf[i].buffer_len, g_sible_nvds_temp_buf[i].buffer);
                LOG_HEX("nvds_cache", 16, (uint8_t *)g_sible_nvds_temp_buf[i].buffer, g_sible_nvds_temp_buf[i].buffer_len);
                if (ret != NVDS_OK)
                    LOG_E("nvds(%d) flush failed", i);
            }
            bt_mem_free(g_sible_nvds_temp_buf[i].buffer);
            memset(&g_sible_nvds_temp_buf[i], 0, sizeof(sibles_nvds_temp_buffer_t));
        }
    }
    return NVDS_OK;
}

static uint8_t *sifli_nvds_read_int(sifli_nvds_type_t type, uint16_t *len)
{
    uint8_t *ptr = NULL;
    size_t read_len = 0;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        ptr = bt_mem_alloc(SIFLI_NVDS_KEY_LEN_STACK);
        if (ptr == NULL)
            break;

        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_STACK);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_STACK, ptr, SIFLI_NVDS_KEY_LEN_STACK);
        // Just assume SIFLI_NVDS_KEY_LEN_STACK is the maximum len
        OS_ASSERT(read_len <= SIFLI_NVDS_KEY_LEN_STACK);
        break;
    }
    case SIFLI_NVDS_TYPE_APP:
    {
        ptr = bt_mem_alloc(SIFLI_NVDS_KEY_LEN_APP);
        if (ptr == NULL)
            break;

        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_APP);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_APP, ptr, SIFLI_NVDS_KEY_LEN_APP);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        OS_ASSERT(read_len <= SIFLI_NVDS_KEY_LEN_APP);
        break;
    }
    case SIFLI_NVDS_TYPE_CM:
    {
        ptr = bt_mem_alloc(SIFLI_NVDS_KEY_LEN_CM);
        if (ptr == NULL)
            break;

        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_CM);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_CM, ptr, SIFLI_NVDS_KEY_LEN_CM);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        OS_ASSERT(read_len <= SIFLI_NVDS_KEY_LEN_CM);
        break;
    }
    case SIFLI_NVDS_TYPE_BT_CM:
    {
        ptr = bt_mem_alloc(SIFLI_NVDS_KEY_LEN_BT_CM);
        if (ptr == NULL)
            break;

        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_BT_CM);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_BT_CM, ptr, SIFLI_NVDS_KEY_LEN_BT_CM);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        OS_ASSERT(read_len <= SIFLI_NVDS_KEY_LEN_BT_CM);
        break;
    }
    case SIFLI_NVDS_TYPE_BT_HOST:
    {
        ptr = bt_mem_alloc(*len);
        if (ptr == NULL)
            break;

        memset(ptr, 0, *len);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_BT_HOST, ptr, *len);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        OS_ASSERT(read_len <= *len);

        break;
    }
    default:
        break;
    }
    *len = (uint16_t)read_len;

    return ptr;
}





size_t sifli_nvds_flash_read(const char *key, void *value_buf, size_t buf_len)
{
    size_t read_len = 0;
    if (sifli_nvds_is_init())
        read_len = sifli_nvds_flash_adaptor_read(key, value_buf, buf_len);
    return read_len;
}



uint8_t sifli_nvds_flash_write(const char *key, const void *value_buf, size_t buf_len)
{
    uint8_t ret = NVDS_FAIL;
    if (sifli_nvds_is_init())
        ret = sifli_nvds_flash_adaptor_write(key, value_buf, buf_len);

    return ret;
}


// Return NVDS_STATUS
uint8_t sifli_nvds_flash_init(void)
{
    return sifli_nvds_flash_adaptor_init();
}


uint8_t sifli_nvds_flash_delete(const char *key)
{
    return sifli_nvds_flash_adaptor_delete(key);
}



uint8_t sifli_nvds_read(sifli_nvds_type_t type, uint16_t *len, uint8_t *buffer)
{
    uint8_t ret = NVDS_FAIL;
    do
    {
        if (len == NULL || buffer == NULL)
            break;

        uint8_t *ptr = sifli_nvds_read_int(type, len);
        if (ptr == NULL)
            break;

        if (*len != 0)
        {
            memcpy(buffer, ptr, *len);
            ret = NVDS_OK;
        }

        if (ptr)
            bt_mem_free(ptr);
    }
    while (0);
    return ret;
}

void sifli_nvds_flash_reset(const char *key)
{
    sifli_nvds_flash_delete(key);
    // Only init stack key
    sifli_nvds_init_db();
}

uint8_t sifli_nvds_get_default_vaule(uint8_t *ptr, uint16_t *len)
{
    return sifli_nvds_construct_default_val(SIFLI_NVDS_TYPE_STACK, ptr, len);
}

uint8_t sifli_nvds_write(sifli_nvds_type_t type, uint16_t len, uint8_t *ptr)
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
    case SIFLI_NVDS_TYPE_BT_CM:
    {
        if (len > SIFLI_NVDS_KEY_LEN_BT_CM)
        {
            ret = NVDS_FAIL;
            break;
        }
        ret = sifli_nvds_flash_write(SIFLI_NVDS_KEY_BT_CM, ptr, len);
        break;
    }
    case SIFLI_NVDS_TYPE_BT_HOST:
    {
        ret = sifli_nvds_flash_write(SIFLI_NVDS_KEY_BT_HOST, ptr, len);
        break;
    }
    default:
        ret = NVDS_TAG_NOT_DEFINED;
        break;
    }
    return ret;
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

#ifdef SOC_SF32LB55X
void sifli_nvds_handler(void *header, uint8_t *data_ptr, uint16_t param_len)
{
    uint16_t msg_id = ((sibles_msg_para_t *)header)->id;
    switch (msg_id)
    {
    case APP_SIFLI_NVDS_GET_REQ:
    {
        sifli_nvds_get_value_cnf_t *cnf;
        sifli_nvds_get_value_t *val = (sifli_nvds_get_value_t *)data_ptr;

        uint16_t len;
        uint8_t *ptr = sifli_nvds_read_int(val->type, &len);
        cnf = (sifli_nvds_get_value_cnf_t *)sifli_msg_alloc(APP_SIFLI_NVDS_GET_CNF,
                TASK_ID_APP, sifli_get_stack_id(),
                sizeof(sifli_nvds_get_value_cnf_t) + len);
        cnf->len = len;
        cnf->type = val->type;
        if (len && ptr)
        {
            memcpy(cnf->value, ptr, len);
            cnf->status = NVDS_OK;
        }
        else
            cnf->status = NVDS_TAG_NOT_DEFINED;

        if (ptr)
            bt_mem_free(ptr);

        sifli_msg_send(cnf);
        //sifli_msg_free(cnf);
        break;
    }
    case APP_SIFLI_NVDS_SET_REQ:
    {
        sifli_nvds_set_value_cnf_t *cnf = (sifli_nvds_set_value_cnf_t *)sifli_msg_alloc(APP_SIFLI_NVDS_SET_CNF,
                                          TASK_ID_APP, sifli_get_stack_id(),
                                          sizeof(sifli_nvds_set_value_cnf_t));
        sifli_nvds_set_value_t *val = (sifli_nvds_set_value_t *)data_ptr;
        cnf->type = val->type;
        cnf->status = sifli_nvds_write(val->type, val->len, val->value);

        sifli_msg_send(cnf);
        //sifli_msg_free(cnf);
        break;
    }
    default:
        break;
    }
}
#endif //SOC_SF32LB55X

uint8_t ble_nvds_read_mac_address(uint8_t *addr, uint8_t len)
{
    uint8_t ret = NVDS_FAIL;
    if (!addr || len != NVDS_STACK_LEN_BD_ADDRESS)
        return ret;

    sifli_nvds_read_tag_t tag;
    tag.tag = NVDS_STACK_TAG_BD_ADDRESS;
    tag.length = NVDS_STACK_LEN_BD_ADDRESS;
    return sifli_nvds_read_tag_value(&tag, addr);
}


uint8_t ble_nvds_update_address(bd_addr_t *addr, ble_common_update_type_t u_type, uint8_t is_flush)
{
    uint8_t ret = NVDS_FAIL;

    if (!addr || u_type == BLE_UPDATE_NO_UPDATE)
        return ret;

    LOG_D("NVDS Update Addr local addr %02x-%02x-%02x-%02x-%02x-%02x", addr->addr[0], addr->addr[1], addr->addr[2], addr->addr[3], addr->addr[4], addr->addr[5]);
    {
        uint8_t addr_loc[NVDS_STACK_LEN_BD_ADDRESS];
        ret = ble_nvds_read_mac_address(addr_loc, NVDS_STACK_LEN_BD_ADDRESS);
        if (ret == NVDS_OK)
        {
            if (memcmp((void *)addr, (void *)addr_loc, NVDS_STACK_LEN_BD_ADDRESS) == 0)
            {
                return ret;
            }
            if (memcmp((void *)&g_default_bdaddr, (void *)addr_loc, NVDS_STACK_LEN_BD_ADDRESS) == 0)
            {
                u_type = BLE_UPDATE_ALWAYS;
            }
        }

        sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
        if (update_tag)
        {
            update_tag->is_flush = is_flush;
            update_tag->type = u_type;
            update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
            update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;
            memcpy((void *)update_tag->value.value, (void *)addr, NVDS_STACK_LEN_BD_ADDRESS);

            ret = sifli_nvds_write_tag_value(update_tag);
            bt_mem_free(update_tag);
        }
        else
            ret = NVDS_NO_SPACE_AVAILABLE;
    }

    return ret;
}


#ifdef NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE
static void ble_nvds_auto_update_address(void)
{

    uint8_t addr[NVDS_STACK_LEN_BD_ADDRESS];
    uint8_t res = rt_flash_config_read(FACTORY_CFG_ID_MAC, (uint8_t *)addr, NVDS_STACK_LEN_BD_ADDRESS);
    LOG_D("NVDS Update Addr res %d, addr %02x-%02x-%02x-%02x-%02x-%02x", res, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    if (res == NVDS_STACK_LEN_BD_ADDRESS)
    {
        uint8_t addr_loc[NVDS_STACK_LEN_BD_ADDRESS];
        uint8_t ret = ble_nvds_read_mac_address(addr_loc, NVDS_STACK_LEN_BD_ADDRESS);
        if (ret == NVDS_OK)
        {
            if (memcmp((void *)addr, (void *)addr_loc, NVDS_STACK_LEN_BD_ADDRESS) == 0)
            {
                return;
            }
        }

        sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
        if (update_tag)
        {
            update_tag->is_flush = 1;
            update_tag->type = BLE_UPDATE_ALWAYS;
            update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
            update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;
            memcpy((void *)update_tag->value.value, (void *)addr, NVDS_STACK_LEN_BD_ADDRESS);

            sifli_nvds_write_tag_value(update_tag);
            bt_mem_free(update_tag);
        }
        else
        {
            LOG_E("update addr OOM");
        }

    }
}

static void ble_nvds_auto_update_address_via_buffer(uint8_t *buffer, uint16_t *buffer_len, uint16_t max_len)
{
    uint8_t addr[NVDS_STACK_LEN_BD_ADDRESS];
    uint8_t res = rt_flash_config_read(FACTORY_CFG_ID_MAC, (uint8_t *)addr, NVDS_STACK_LEN_BD_ADDRESS);
    if (res == NVDS_STACK_LEN_BD_ADDRESS)
    {
        sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
        if (update_tag)
        {
            update_tag->is_flush = 1;
            update_tag->type = BLE_UPDATE_ALWAYS;
            update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
            update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;
            memcpy((void *)update_tag->value.value, (void *)addr, NVDS_STACK_LEN_BD_ADDRESS);

            sifli_nvds_write_tag_value_via_buffer(buffer, buffer_len, max_len, update_tag);
            bt_mem_free(update_tag);
        }
        else
        {
            LOG_E("update addr via buffer OOM");
        }
    }
}
#endif

#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X))
    #if LCPU_FLASH_CODE_SIZE > 0
        #define EXT_WAKEUP_TIME_LXT32K   5500
        #define EXT_WAKEUP_TIME_RC10K    6500
    #else
        #define EXT_WAKEUP_TIME_LXT32K   4500
        #define EXT_WAKEUP_TIME_RC10K    5500
    #endif /* LCPU_FLASH_CODE_SIZE  */
#else
    #define EXT_WAKEUP_TIME_LXT32K   3500
    #define EXT_WAKEUP_TIME_RC10K    4500
#endif


static void ble_nvds_update_wakeup_time_via_buffer(uint8_t *buffer, uint16_t *buffer_len, uint16_t max_len)
{
    uint16_t wakeup_time;

#ifndef BSP_USING_PC_SIMULATOR
    if (HAL_LXT_DISABLED())
        wakeup_time = EXT_WAKEUP_TIME_RC10K;
    else
#endif
        wakeup_time = EXT_WAKEUP_TIME_LXT32K;


    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_EXT_WAKEUP_TIME);
    BT_OOM_ASSERT(update_tag);

    if (update_tag)
    {
        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_EXT_WAKEUP_TIME;
        update_tag->value.len = NVDS_STACK_LEN_EXT_WAKEUP_TIME;
        memcpy((void *)update_tag->value.value, (void *)&wakeup_time, NVDS_STACK_LEN_EXT_WAKEUP_TIME);

        sifli_nvds_write_tag_value_via_buffer(buffer, buffer_len, max_len, update_tag);
        bt_mem_free(update_tag);
    }
}


void ble_nvds_update_wakeup_time(void)
{
    uint16_t wakeup_time;

    sifli_nvds_read_tag_t tag;
    tag.tag = NVDS_STACK_TAG_EXT_WAKEUP_TIME;
    tag.length = NVDS_STACK_LEN_EXT_WAKEUP_TIME;
    uint8_t ret = sifli_nvds_read_tag_value(&tag, (uint8_t *)&wakeup_time);
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
#endif //
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

    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_EXT_WAKEUP_TIME);
    BT_OOM_ASSERT(update_tag);

    if (update_tag)
    {
        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_EXT_WAKEUP_TIME;
        update_tag->value.len = NVDS_STACK_LEN_EXT_WAKEUP_TIME;
        memcpy((void *)update_tag->value.value, (void *)&wakeup_time, NVDS_STACK_LEN_EXT_WAKEUP_TIME);

        sifli_nvds_write_tag_value(update_tag);
        bt_mem_free(update_tag);
    }
}


#if defined(SOC_SF32LB56X)
void ble_nvds_enable_btc_assert_msg(uint8_t enable)
{
    uint8_t is_enable;

    sifli_nvds_read_tag_t tag;
    tag.tag = NVDS_STACK_TAG_ASSERT_MSG_ENABLE;
    tag.length = NVDS_STACK_LEN_ASSERT_MSG_ENABLE;
    uint8_t ret = sifli_nvds_read_tag_value(&tag, (uint8_t *)&is_enable);
    if (ret == NVDS_OK)
    {
        LOG_I("read assert config %d", is_enable);
        if (is_enable == enable)
            return;
    }

    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_ASSERT_MSG_ENABLE);
    BT_OOM_ASSERT(update_tag);

    if (update_tag)
    {
        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_ASSERT_MSG_ENABLE;
        update_tag->value.len = NVDS_STACK_LEN_ASSERT_MSG_ENABLE;
        memcpy((void *)update_tag->value.value, (void *)&enable, NVDS_STACK_LEN_ASSERT_MSG_ENABLE);

        sifli_nvds_write_tag_value(update_tag);
        bt_mem_free(update_tag);
    }

}
#endif

#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X))
#define XTAL_TIME_PREPARE_TIME (1920)
void ble_nvds_update_xtal_time_via_buffer(uint8_t *buffer, uint16_t *buffer_len, uint16_t max_len)
{
    uint16_t wakeup_time = XTAL_TIME_PREPARE_TIME;

    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_OSC_WAKEUP_TIME);

    BT_OOM_ASSERT(update_tag);
    if (update_tag)
    {
        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_OSC_WAKEUP_TIME;
        update_tag->value.len = NVDS_STACK_LEN_OSC_WAKEUP_TIME;
        memcpy((void *)update_tag->value.value, (void *)&wakeup_time, NVDS_STACK_LEN_OSC_WAKEUP_TIME);

        sifli_nvds_write_tag_value_via_buffer(buffer, buffer_len, max_len, update_tag);
        bt_mem_free(update_tag);
    }
}


void ble_nvds_update_xtal_time(void)
{
    uint16_t wakeup_time;

    sifli_nvds_read_tag_t tag;
    tag.tag = NVDS_STACK_TAG_OSC_WAKEUP_TIME;
    tag.length = NVDS_STACK_LEN_OSC_WAKEUP_TIME;
    uint8_t ret = sifli_nvds_read_tag_value(&tag, (uint8_t *)&wakeup_time);
    if (ret == NVDS_OK)
    {
        LOG_I("read sleep time %d", wakeup_time);
#ifndef BSP_USING_PC_SIMULATOR
        if (HAL_LXT_DISABLED())
        {
            if (wakeup_time == XTAL_TIME_PREPARE_TIME)
                return;
        }
        else
#endif //
        {
            if (wakeup_time == XTAL_TIME_PREPARE_TIME)
                return;
        }
    }
#ifndef BSP_USING_PC_SIMULATOR
    if (HAL_LXT_DISABLED())
        wakeup_time = XTAL_TIME_PREPARE_TIME;
    else
#endif
        wakeup_time = XTAL_TIME_PREPARE_TIME;

    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_OSC_WAKEUP_TIME);
    BT_OOM_ASSERT(update_tag);

    if (update_tag)
    {
        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_STACK_TAG_OSC_WAKEUP_TIME;
        update_tag->value.len = NVDS_STACK_LEN_OSC_WAKEUP_TIME;
        memcpy((void *)update_tag->value.value, (void *)&wakeup_time, NVDS_STACK_LEN_OSC_WAKEUP_TIME);

        sifli_nvds_write_tag_value(update_tag);
        bt_mem_free(update_tag);
    }
}
#endif // SOC_SF32LB56X



static uint8_t sifli_nvds_construct_default_val(sifli_nvds_type_t type, uint8_t *ptr, uint16_t *len)
{
    uint8_t ret = NVDS_NOT_SUPPORT;
    if (type == SIFLI_NVDS_TYPE_STACK)
    {
        do
        {
            uint16_t default_len;
            uint8_t *default_ptr;
#ifndef BSP_USING_PC_SIMULATOR
            if (HAL_LXT_DISABLED())
            {
                default_len = sizeof(g_ble_slp_default_rc10k);
                default_ptr = (uint8_t *)g_ble_slp_default_rc10k;
            }
            else
#endif
            {
                default_len = sizeof(g_ble_slp_default_lxt32k);
                default_ptr = (uint8_t *)g_ble_slp_default_lxt32k;
            }

            if (*len < default_len)
            {
                ret = NVDS_PARAM_LOCKED;
                break;
            }

            memcpy(ptr, default_ptr, default_len);

#ifdef NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE
            ble_nvds_auto_update_address_via_buffer(ptr, &default_len, *len);
#endif

            ble_nvds_update_wakeup_time_via_buffer(ptr, &default_len, *len);
#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X))
            ble_nvds_update_xtal_time_via_buffer(ptr, &default_len, *len);
#endif
            ret = NVDS_OK;
            *len = default_len;
        }
        while (0);

        //memset()
    }
    return ret;
}

static void sifli_nvds_init_db(void)
{
    uint16_t len;
    uint8_t *ptr  = sifli_nvds_read_int(SIFLI_NVDS_TYPE_STACK, &len);
    if (ptr)
    {
        if (len == 0)
        {
            uint16_t con_len = SIFLI_NVDS_KEY_LEN_STACK;
            sifli_nvds_construct_default_val(SIFLI_NVDS_TYPE_STACK, ptr, &con_len);
            sifli_nvds_write(SIFLI_NVDS_TYPE_STACK, con_len, ptr);
        }
        bt_mem_free(ptr);
    }
}

void ble_db_init(void)
{
    bt_nvds_env_t *env = sifli_nvds_get_env();
    env->is_init = 0;
    uint8_t ret = sifli_nvds_flash_init();
    if (ret == NVDS_OK)
    {
        env->is_init = 1;
        sifli_nvds_init_db();
    }
}



void sifli_nvds_init(void)
{
#ifdef FDB_USING_KVDB
    rt_mutex_init(&ble_flash_mutex, "ble_flash_mutex", RT_IPC_FLAG_FIFO);
#endif
    ble_db_init();
#ifdef NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE
    ble_nvds_auto_update_address();
#endif
    ble_nvds_update_wakeup_time();
#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X))
    ble_nvds_update_xtal_time();
#endif
}


static void sifli_trc_config(uint32_t config)
{
// TODO: Only 55x not support config HCI immediatelly. Lib has already supported, to avoid assert due to combination of new src and old lib. Just keep old src.
// #if !defined(SOC_SF32LB55X)
#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)
    sibles_set_trc_cfg(SIBLES_TRC_CUSTOMIZE, config);
#endif
    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_TRACER_CONFIG);
    if (update_tag == NULL)
        return;
    memcpy((void *)update_tag->value.value, (void *)&config, NVDS_STACK_LEN_TRACER_CONFIG);
    update_tag->is_flush = 1;
    update_tag->type = BLE_UPDATE_ALWAYS;
    update_tag->value.tag = NVDS_STACK_TAG_TRACER_CONFIG;
    update_tag->value.len = NVDS_STACK_LEN_TRACER_CONFIG;

    sifli_nvds_write_tag_value(update_tag);
    bt_mem_free(update_tag);
}

void sifli_hci_log_enable(bool is_on)
{
#if defined(SOC_SF32LB55X)
    uint32_t config[] = {0x20, 0x090020};
#else
    uint32_t config[] = {0x20, 0x190C20};
#endif
    sifli_trc_config(config[is_on]);
}

void sifli_trc_log_enable(uint32_t config)
{
    sifli_trc_config(config);
}

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

    ble_db_init();

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


/**
    nvds write <type> <total len> <offset> <data in hex string, such as 123ABC, data will be {0x12, 0x3A, 0xBC}>
    nvds read  <type> <total len>
*/
static void nvds(uint8_t argc, char **argv)
{
    char *value = NULL;

    /// Ensure cancel LP request in the following commands
#if defined(BF0_HCPU) && defined(SOC_SF32LB52X)
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif
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
                data = bt_mem_alloc(len);

            if (data)
            {
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
                    sifli_nvds_write(type, len, data);
                    bt_mem_free(data);
                    data = NULL;
                }
            }
            else
                LOG_W("OOM");
        }
        else if (strcmp(argv[1], "read") == 0)
        {
            sifli_nvds_type_t type;
            uint8_t *data = NULL;
            uint16_t len;
            type = atoi(argv[2]);
            data = sifli_nvds_read_int(type, &len);
            if (data)
            {
                HAL_DBG_print_data((char *)data, 0, len);
                LOG_HEX("nvds_read", 16, (uint8_t *)data, len);
                bt_mem_free(data);
            }
        }
        else if (strcmp(argv[1], "read_lk") == 0)
        {
            uint8_t count = 0;
            uint8_t db_name[7] = {0};
            uint8_t *ptr = bt_mem_alloc(50);
            if (ptr)
            {
                while (count < 5)
                {
                    rt_snprintf((char *)db_name, 7, "SC_DB%1d", count++);
                    size_t len = sifli_nvds_flash_read((const char *)db_name, (void *)ptr, 50);
                    LOG_I("rd len %d", len);
                    LOG_HEX("lk_read", 16, ptr, len);
                }
                bt_mem_free(ptr);
            }
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
#ifdef BT_FINSH
        else if (strcmp(argv[1], "reset_lk") == 0)
        {
            extern void sc_clean_all_link_key(void);
            sc_clean_all_link_key();
        }
#endif //BT_FINSH
        else if (strcmp(argv[1], "update") == 0)
        {
            if (strcmp(argv[2], "addr") == 0)
            {
                uint16_t len;
                len = atoi(argv[3]);
                do
                {
                    if (len != NVDS_STACK_LEN_BD_ADDRESS)
                        break;

                    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_STACK_LEN_BD_ADDRESS);
                    if (update_tag == NULL)
                        break;
                    hex2data(argv[4], update_tag->value.value, NVDS_STACK_LEN_BD_ADDRESS);
                    update_tag->is_flush = 1;
                    update_tag->type = BLE_UPDATE_ALWAYS;
                    update_tag->value.tag = NVDS_STACK_TAG_BD_ADDRESS;
                    update_tag->value.len = NVDS_STACK_LEN_BD_ADDRESS;

                    sifli_nvds_write_tag_value(update_tag);
                    bt_mem_free(update_tag);
                }
                while (0);
            }
            else if (strcmp(argv[2], "hci_log") == 0)
            {
                uint8_t is_on = atoi(argv[3]);

                if (is_on > 1)
                    is_on = 1;

                sifli_hci_log_enable(is_on);
            }
            else if (strcmp(argv[2], "trc_cfg") == 0)
            {
                uint32_t config = atoh(argv[3]);
                LOG_I("config %x", config);
                sifli_trc_log_enable(config);
            }
        }
        else if (strcmp(argv[1], "flush") == 0)
        {
#if !defined(SOC_SF32LB55X)
            void bt_stack_nvds_update(void);
            bt_stack_nvds_update();
#endif // !SOC_SF32LB55X
        }
#if defined(SOC_SF32LB56X)
        else if (strcmp(argv[1], "btc_assert") == 0)
        {
            uint8_t enable = atoi(argv[2]);
            ble_nvds_enable_btc_assert_msg(enable);
        }
#endif // SOC_SF32LB56X

    }

#if defined(BF0_HCPU) && defined(SOC_SF32LB52X)
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif

}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(nvds, Sifli NVDS command);
#endif // RT_USING_FINSH

#else // BSP_BLE_NVDS_SYNC

#define BLE_DEFAULT_BDADDR  {{0x01, 0x23, 0x45, 0x67, 0x89, 0xAB}}
#include "data_service.h"
#include "ble_nvds_service.h"


#define DS_HANDLE_INVALID_CHECK(handle, ret) \
        if (handle == DATA_CLIENT_INVALID_HANDLE) \
            return ret;

#define DS_HANDLE_INVALID_CHECK_NO_RET(handle) \
        if (handle == DATA_CLIENT_INVALID_HANDLE) \
            return;

OS_SEM_DECLAR(g_sible_nvds_sema);

typedef struct
{
    datac_handle_t srv_handle;
    uint8_t is_subscribed;
    struct
    {
        bd_addr_t bd_addr;
    } info;
} sifli_nvds_env_t;

static sifli_nvds_env_t g_sifli_nvds_env =
{
    .srv_handle = DATA_CLIENT_INVALID_HANDLE,
    .info.bd_addr = BLE_DEFAULT_BDADDR
};
//const struct bd_addr_t g_default_bdaddr = BLE_DEFAULT_BDADDR;

static sifli_nvds_env_t *sifli_nvds_get_env(void)
{
    return &g_sifli_nvds_env;
}


static sifli_nvds_tag_value_t *sifli_nvds_get_tag_value_via_buffer(uint8_t *buffer, uint16_t buffer_len, uint8_t tag)
{
    uint16_t len = buffer_len, val_len;
    sifli_nvds_tag_value_t *val = NULL;

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


static void sifli_nvds_ble_env_prepare(sifli_nvds_env_t *env, sifli_nvds_get_value_cnf_t *value)
{
    switch (value->type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        // Just need get the bd_addr
        sifli_nvds_tag_value_t *data = sifli_nvds_get_tag_value_via_buffer(value->value, value->len, NVDS_STACK_TAG_BD_ADDRESS);
        if (data && data->len == NVDS_STACK_LEN_BD_ADDRESS)
        {
            memcpy(env->info.bd_addr.addr, (void *)data->value, NVDS_STACK_LEN_BD_ADDRESS);
        }
        break;
    }
    default:
        break;
    }

}


static int sifli_nvds_callback(data_callback_arg_t *arg)
{
    OS_ASSERT(arg);
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        env->is_subscribed = 1;
        os_sem_release(g_sible_nvds_sema);
        LOG_I("Subscribed nvds service ret %d.", rsp->result);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        env->is_subscribed = 0;
        os_sem_delete(g_sible_nvds_sema);
        break;
    }
    case BLE_NVDS_SERVICE_READ_RSP:
    {
        sifli_nvds_get_value_cnf_t *rsp = (sifli_nvds_get_value_cnf_t *)arg->data;
        OS_ASSERT(rsp);
        if (rsp->type == SIFLI_NVDS_TYPE_STACK ||
                rsp->type == SIFLI_NVDS_TYPE_APP)
        {
            sifli_nvds_ble_env_prepare(env, rsp);
            sifli_nvds_get_value_cnf_t *cnf = (sifli_nvds_get_value_cnf_t *)sifli_msg_alloc(APP_SIFLI_NVDS_GET_CNF,
                                              TASK_ID_APP, sifli_get_stack_id(),
                                              sizeof(sifli_nvds_get_value_cnf_t) +
                                              rsp->len);
            memcpy(cnf, rsp, sizeof(sifli_nvds_get_value_cnf_t) + rsp->len);

            sifli_msg_send(cnf);
        }
        else
        {
            ble_event_publish(BLE_NVDS_ASYNC_READ_CNF, rsp, sizeof(sifli_nvds_get_value_cnf_t) + rsp->len);
        }
        break;
    }
    case BLE_NVDS_SERVICE_WRITE_RSP:
    {
        sifli_nvds_set_value_cnf_t *rsp = (sifli_nvds_set_value_cnf_t *)arg->data;
        OS_ASSERT(rsp);
        if (rsp->type == SIFLI_NVDS_TYPE_STACK ||
                rsp->type == SIFLI_NVDS_TYPE_APP)
        {
            sifli_nvds_set_value_cnf_t *cnf = (sifli_nvds_set_value_cnf_t *)sifli_msg_alloc(APP_SIFLI_NVDS_SET_CNF,
                                              TASK_ID_APP, sifli_get_stack_id(),
                                              sizeof(sifli_nvds_set_value_cnf_t));
            memcpy(cnf, rsp, sizeof(sifli_nvds_set_value_cnf_t));

            sifli_msg_send(cnf);
        }
        else
        {
            ble_event_publish(BLE_NVDS_ASYNC_WRITE_CNF, rsp, sizeof(sifli_nvds_set_value_cnf_t));
        }
        break;
    }
    case BLE_NVDS_SERVICE_READ_TAG_RSP:
    {
        sifli_nvds_read_tag_cnf_t *rsp = (sifli_nvds_read_tag_cnf_t *)arg->data;
        OS_ASSERT(rsp);
        ble_event_publish(BLE_NVDS_ASYNC_READ_TAG_CNF, rsp, sizeof(sifli_nvds_read_tag_cnf_t) + rsp->length);
        break;
    }
    case BLE_NVDS_SERVICE_WRITE_TAG_RSP:
    {
        sifli_nvds_write_tag_cnf_t *rsp = (sifli_nvds_write_tag_cnf_t *)arg->data;
        OS_ASSERT(rsp);
        ble_event_publish(BLE_NVDS_ASYNC_WRITE_TAG_CNF, rsp, sizeof(sifli_nvds_write_tag_cnf_t));
        break;
    }
    case BLE_NVDS_SERVICE_FLUSH_COMPLETED:
    {
        ble_event_publish(BLE_NVDS_AYSNC_FLUSH_TAG_CNF, NULL, 0);
        break;
    }
    case BLE_NVDS_SERVICE_UPDATE_ADDR_RSP:
    {
        sifli_nvds_update_addr_rsp_t *rsp = (sifli_nvds_update_addr_rsp_t *)arg->data;
        OS_ASSERT(rsp);
        ble_event_publish(BLE_NVDS_AYSNC_UPDATE_ADDR_CNF, rsp, sizeof(sifli_nvds_update_addr_rsp_t));
        break;
    }
    default:
        break;
    }
    return 0;
}


uint8_t sifli_nvds_write(sifli_nvds_type_t type, uint16_t len, uint8_t *ptr)
{
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    DS_HANDLE_INVALID_CHECK(env->srv_handle, NVDS_FAIL);

    data_msg_t msg;
    uint8_t ret = NVDS_FAIL;
    rt_err_t ret1;

    sifli_nvds_set_value_t *body = (sifli_nvds_set_value_t *)data_service_init_msg(&msg, BLE_NVDS_SERVICE_WRITE, sizeof(sifli_nvds_set_value_t) + len);
    OS_ASSERT(body);

    body->type = type;
    body->len = len;
    memcpy(body->value, ptr, len);

    ret1 = datac_send_msg(env->srv_handle, &msg);
    if (ret1 == RT_EOK)
        ret = NVDS_PENDING;
    return ret;
}


uint8_t sifli_nvds_read(sifli_nvds_type_t type, uint16_t *len, uint8_t *buffer)
{
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    DS_HANDLE_INVALID_CHECK(env->srv_handle, (uint8_t)NULL);

    data_msg_t msg;
    uint8_t ret = NVDS_FAIL;
    rt_err_t ret1;

    sifli_nvds_get_value_t *body = (sifli_nvds_get_value_t *)data_service_init_msg(&msg, BLE_NVDS_SERVICE_READ, sizeof(sifli_nvds_get_value_t));
    OS_ASSERT(body);

    body->type = type;

    ret1 = datac_send_msg(env->srv_handle, &msg);
    if (ret1 == RT_EOK)
        ret = NVDS_PENDING;
    return ret;

}




uint8_t sifli_nvds_read_tag_value(sifli_nvds_read_tag_t *tag, uint8_t *tag_buffer)
{
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    DS_HANDLE_INVALID_CHECK(env->srv_handle, NVDS_FAIL);
    data_msg_t msg;
    uint8_t ret = NVDS_FAIL;
    rt_err_t ret1;

    if (tag == NULL || tag_buffer == NULL)
        return NVDS_PARAM_LOCKED;

    // BD_ADDRESS will saved in local.
    if (tag->tag == NVDS_STACK_TAG_BD_ADDRESS)
    {
        memcpy(tag_buffer, env->info.bd_addr.addr, NVDS_STACK_LEN_BD_ADDRESS);
        return NVDS_OK;
    }
    uint8_t *body = data_service_init_msg(&msg, BLE_NVDS_SERVICE_READ_TAG, sizeof(sifli_nvds_read_tag_t));
    OS_ASSERT(body);
    memcpy(body, tag, sizeof(sifli_nvds_read_tag_t));

    ret1 = datac_send_msg(env->srv_handle, &msg);
    if (ret1 == RT_EOK)
        ret = NVDS_PENDING;
    return ret;
}

uint8_t sifli_nvds_write_tag_value(sifli_nvds_write_tag_t *tag)
{
    if (tag == NULL)
        return NVDS_PARAM_LOCKED;

    sifli_nvds_env_t *env = sifli_nvds_get_env();
    DS_HANDLE_INVALID_CHECK(env->srv_handle, NVDS_FAIL);
    data_msg_t msg;
    uint8_t ret = NVDS_FAIL;
    rt_err_t ret1;
    uint8_t *body = data_service_init_msg(&msg, BLE_NVDS_SERVICE_WRITE_TAG, sizeof(sifli_nvds_write_tag_t) + tag->value.len);
    OS_ASSERT(body);
    memcpy(body, tag, sizeof(sifli_nvds_write_tag_t) + tag->value.len);
    ret1 = datac_send_msg(env->srv_handle, &msg);
    if (ret1 == RT_EOK)
        ret = NVDS_PENDING;
    return ret;
}

uint8_t ble_nvds_update_address(bd_addr_t *addr, ble_common_update_type_t u_type, uint8_t is_flush)
{

    if (addr == NULL)
        return NVDS_PARAM_LOCKED;

    sifli_nvds_env_t *env = sifli_nvds_get_env();
    DS_HANDLE_INVALID_CHECK(env->srv_handle, NVDS_FAIL);
    data_msg_t msg;
    uint8_t ret = NVDS_FAIL;
    rt_err_t ret1;
    sifli_nvds_update_addr_req_t *body = (sifli_nvds_update_addr_req_t *)
                                         data_service_init_msg(&msg,
                                                 BLE_NVDS_SERVICE_UPDATE_ADDR,
                                                 sizeof(sifli_nvds_update_addr_req_t));
    OS_ASSERT(body);
    memcpy((void *)&body->addr, (void *)addr, sizeof(bd_addr_t));
    body->u_type = u_type;
    body->is_flush = is_flush;
    ret1 = datac_send_msg(env->srv_handle, &msg);
    if (ret1 == RT_EOK)
        ret = NVDS_PENDING;
    return ret;
}

void sifli_nvds_handler(void *header, uint8_t *data_ptr, uint16_t param_len)
{
    uint16_t msg_id = ((sibles_msg_para_t *)header)->id;
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    rt_err_t ret;



    LOG_I("nvds msg_id %d", msg_id);
    switch (msg_id)
    {
    case APP_SIFLI_NVDS_GET_REQ:
    {

        sifli_nvds_get_value_t *val = (sifli_nvds_get_value_t *)data_ptr;
        if (env->srv_handle == DATA_CLIENT_INVALID_HANDLE)
        {
            sifli_nvds_get_value_cnf_t *cnf = (sifli_nvds_get_value_cnf_t *)sifli_msg_alloc(APP_SIFLI_NVDS_GET_CNF,
                                              TASK_ID_APP, sifli_get_stack_id(),
                                              sizeof(sifli_nvds_get_value_cnf_t));
            cnf->len = 0;
            cnf->type = val->type;
            cnf->status = NVDS_TAG_NOT_DEFINED;
            sifli_msg_send(cnf);
        }
        else
        {
            data_msg_t msg;
            uint8_t *body = data_service_init_msg(&msg, BLE_NVDS_SERVICE_READ, sizeof(sifli_nvds_get_value_t));
            OS_ASSERT(body);
            memcpy(body, val, sizeof(sifli_nvds_get_value_t));
            ret = datac_send_msg(env->srv_handle, &msg);
            LOG_I("get_msg %d", ret);
        }
        break;
    }
    case APP_SIFLI_NVDS_SET_REQ:
    {
        sifli_nvds_set_value_t *val = (sifli_nvds_set_value_t *)data_ptr;
        if (env->srv_handle == DATA_CLIENT_INVALID_HANDLE)
        {
            sifli_nvds_set_value_cnf_t *cnf = (sifli_nvds_set_value_cnf_t *)sifli_msg_alloc(APP_SIFLI_NVDS_SET_CNF,
                                              TASK_ID_APP, sifli_get_stack_id(),
                                              sizeof(sifli_nvds_set_value_cnf_t));
            cnf->type = val->type;
            cnf->status = NVDS_TAG_NOT_DEFINED;
            sifli_msg_send(cnf);
        }
        else
        {
            data_msg_t msg;

            uint8_t *body = data_service_init_msg(&msg, BLE_NVDS_SERVICE_WRITE, sizeof(sifli_nvds_set_value_t) + val->len);
            OS_ASSERT(body);
            memcpy(body, val, sizeof(sifli_nvds_set_value_t) + val->len);
            ret = datac_send_msg(env->srv_handle, &msg);
            LOG_I("set_msg %d", ret);
            break;
        }
    }
    default:
        break;
    }
}

uint8_t sifli_nvds_flush(void)
{
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    data_msg_t msg;
    uint8_t *body = data_service_init_msg(&msg, BLE_NVDS_SERVICE_FLUSH, 0);
    OS_ASSERT(body);

    datac_send_msg(env->srv_handle, &msg);
    return NVDS_PENDING;
}




void sifli_nvds_init(void)
{
    sifli_nvds_env_t *env = sifli_nvds_get_env();
    os_sem_create(g_sible_nvds_sema, 0);
    env->srv_handle = datac_open();
    OS_ASSERT(DATA_CLIENT_INVALID_HANDLE != env->srv_handle);
    datac_subscribe(env->srv_handle, "BLE_NV", sifli_nvds_callback, 0);
    if (!env->is_subscribed)
    {
        LOG_I("Wait nvds service.");
        os_sem_take(g_sible_nvds_sema, OS_WAIT_FORVER);
    }
}



#endif // SIBLES_NVDS_ASYNC

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
