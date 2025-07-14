/**
  ******************************************************************************
  * @file   dfu_ctrl_ext_sol.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2025,  Sifli Technology
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
#include <rthw.h>
#include "board.h"
#include "rtconfig.h"
#include "os_adaptor.h"

#ifdef OTA_56X_NAND
#include "dfu.h"
#include "dfu_internal.h"
#include "bf0_sibles_serial_trans_service.h"

#include "flashdb.h"
#ifdef FDB_USING_FILE_MODE
    #include "dfs_posix.h"
#endif /* FDB_USING_FILE_MODE */

#define LOG_TAG "DFUCTRLEXT"
#include "log.h"
#ifdef BSP_USING_EPIC
    #include "drv_epic.h"
#endif /* BSP_USING_EPIC */

DFU_NON_RET_SECT_BEGIN
//static uint8_t dfu_temp[DFU_MAX_BLK_SIZE];
static uint8_t dfu_ext_temp_key[DFU_KEY_SIZE];
static struct fdb_kvdb g_dfu_ext_db;
static fdb_kvdb_t p_dfu_ext_db = &g_dfu_ext_db;
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    static uint32_t g_dfu_ext_db_cache[512];
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */
DFU_NON_RET_SECT_END


static dfu_ctrl_ext_env_t g_dfu_ctrl_ext_env;
static void dfu_ctrl_error_handle(dfu_ctrl_ext_env_t *env);
static rt_thread_t ble_dfu_flash_write_thread_start();
static void dfu_power_on_check();
static uint8_t g_dfu_nv_init = 0;

static void dfu_ctrl_ext_init_env(void)
{
    memset((void *)&g_dfu_ctrl_ext_env, 0, sizeof(g_dfu_ctrl_ext_env));
}

static dfu_ctrl_ext_env_t *dfu_ctrl_ext_get_env(void)
{
    return &g_dfu_ctrl_ext_env;
}
uint8_t g_dfu_progress_mode = DFU_PROGRESS_TOTAL;

static const uint8_t g_dfu_ext_p_default[] = {0x0};

static struct fdb_default_kv_node default_dfu_ext_kv_set[] =
{
    {DFU_DOWNLOAD_ENV, (void *)g_dfu_ext_p_default, sizeof(g_dfu_ext_p_default)},
};


#ifdef FDB_USING_KVDB
static fdb_err_t dfu_ext_db_init(void)
{
    struct fdb_default_kv default_kv;
    p_dfu_ext_db = &g_dfu_ext_db;
    default_kv.kvs = default_dfu_ext_kv_set;
    default_kv.num = sizeof(default_dfu_ext_kv_set) / sizeof(default_dfu_ext_kv_set[0]);
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    default_kv.kv_cache_pool = g_dfu_ext_db_cache;
    default_kv.kv_cache_pool_size = sizeof(g_dfu_ext_db_cache);
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */

    memset(p_dfu_ext_db, 0, sizeof(*p_dfu_ext_db));

    const char *path = DFU_DB_PARTIAL;
#ifdef FDB_USING_FILE_MODE
#include "fal.h"
    const struct fal_partition *fal = fal_partition_find(DFU_DB_PARTIAL);
    RT_ASSERT(fal);
#ifdef SOLUTION_WATCH
    path = fal->path_name;
#endif
    int sec_size = PKG_FLASHDB_ERASE_GRAN;
    int max_size = 16 * 1024;
    bool file_mode = true;
    rt_kprintf("dfu_db_init: sector_size %d size %d\n", sec_size, max_size);
    fdb_kvdb_control(p_dfu_ext_db, FDB_KVDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
    fdb_kvdb_control(p_dfu_ext_db, FDB_KVDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
    fdb_kvdb_control(p_dfu_ext_db, FDB_KVDB_CTRL_SET_FILE_MODE, (void *)&file_mode);
    if (0 != access(path, 0) && 0 != mkdir(path, 0))
    {
        rt_kprintf("create db %s fail\n", DFU_DB);
        return -1;
    }

#endif /* FDB_USING_FILE_MODE */

    return fdb_kvdb_init(p_dfu_ext_db, DFU_DB, path, &default_kv, NULL);
}
#endif

void dfu_ctrl_update_prog_info_ext(dfu_ctrl_ext_env_t *env)
{
#ifdef PKG_USING_FLASHDB
    struct fdb_blob  blob;
    // TODO: set blob
    fdb_err_t err;
    err = fdb_kv_set_blob(p_dfu_ext_db, DFU_DOWNLOAD_ENV, fdb_blob_make(&blob, &env->prog, sizeof(dfu_download_progress_ext_t)));
    err = FDB_NO_ERR;
    OS_ASSERT(err == FDB_NO_ERR);
#else
#error "FlashDB shall be enabled"
#endif
}

static void check_backup_ftab(dfu_ctrl_ext_env_t *env)
{
    struct sec_configuration *backup_sec_config;
    uint16_t sec_len = sizeof(struct sec_configuration);
    uint8_t des_state;
    dfu_nand_info ota_info;

    LOG_I("check_backup_ftab %d", sec_len);
    backup_sec_config = malloc(sec_len);
    OS_ASSERT(backup_sec_config);
    dfu_packet_read_flash_ext(DFU_BACKUP_FTAB_ADDR, 0, (uint8_t *)backup_sec_config, sec_len);
    dfu_packet_read_flash_ext(DFU_BACKUP_FTAB_ADDR, DFU_INFO_OFFSET, (uint8_t *)&ota_info, sizeof(dfu_nand_info));

    if (backup_sec_config->magic != SEC_CONFIG_MAGIC || ota_info.magic != SEC_CONFIG_MAGIC)
    {
        LOG_I("ftab not ready, running on hcpu1");

        env->running_hcpu = DFU_NAND_HCPU_FIRST_ADDR;
        env->back_up_hcpu = DFU_NAND_HCPU_BACK_UP_ADDR;
    }
    else
    {
        if (ota_info.running_target == DFU_DES_RUNNING_ON_HCPU1)
        {
            LOG_I("now on hcpu1");
            env->running_hcpu = DFU_NAND_HCPU_FIRST_ADDR;
            env->back_up_hcpu = DFU_NAND_HCPU_BACK_UP_ADDR;
        }
        else if (ota_info.running_target == DFU_DES_RUNNING_ON_HCPU2)
        {
            LOG_I("now on hcpu2");
            env->running_hcpu = DFU_NAND_HCPU_BACK_UP_ADDR;
            env->back_up_hcpu = DFU_NAND_HCPU_FIRST_ADDR;
        }
    }
    free(backup_sec_config);
}


void init_flash_des()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    uint8_t des_state = 0;
#ifdef OTA_NAND_ONLY
    check_backup_ftab(env);
#else
    des_state = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    LOG_I("init_flash_des %d", des_state);

    if (des_state == DFU_DES_UPDATE_HCPU1)
    {
        des_state = DFU_DES_RUNNING_ON_HCPU1;
        HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, des_state);
        //dfu_ctrl_update_prog_info_ext(env);
    }
    else if (des_state == DFU_DES_UPDATE_HCPU2)
    {
        des_state = DFU_DES_RUNNING_ON_HCPU2;
        HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, des_state);
        //dfu_ctrl_update_prog_info_ext(env);
    }

    if (des_state == DFU_DES_RUNNING_ON_HCPU1)
    {
        env->running_hcpu = DFU_NAND_HCPU_FIRST_ADDR;
        env->back_up_hcpu = DFU_NAND_HCPU_BACK_UP_ADDR;
    }
    else if (des_state == DFU_DES_RUNNING_ON_HCPU2)
    {
        env->running_hcpu = DFU_NAND_HCPU_BACK_UP_ADDR;
        env->back_up_hcpu = DFU_NAND_HCPU_FIRST_ADDR;
    }

    LOG_I("init_flash_des2 %d", des_state);
#endif
    LOG_I("init_flash_des %d, 0x%x, 0x%x", des_state, env->running_hcpu, env->back_up_hcpu);
}

int dfu_ctrl_ext_init(void)
{
    if (g_dfu_nv_init == 0)
    {
        g_dfu_nv_init = 1;
    }
    else
    {
        return 0;
    }
    dfu_ctrl_ext_init_env();
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
#ifdef FDB_USING_KVDB
    fdb_err_t err;
    struct fdb_blob blob;
    size_t read_len;
    uint8_t ota_update = 0;
    dfu_vesion_t old, cur;
    do
    {
        err = dfu_ext_db_init();
        if (err != FDB_NO_ERR)
        {
            LOG_E("nvds init failed !!!");
            break;
        }
        read_len = fdb_kv_get_blob(p_dfu_ext_db, DFU_DOWNLOAD_ENV, fdb_blob_make(&blob, &env->prog, sizeof(dfu_download_progress_ext_t)));
        if (read_len != sizeof(dfu_download_progress_ext_t))
            OS_ASSERT(1);

        ota_update = env->prog.ota_update;
        rt_memcpy(&old, &env->prog.old_version, sizeof(old));
        rt_memcpy(&cur, &env->prog.new_version, sizeof(cur));

        env->is_init = 1;
#ifdef CFG_BOOTLOADER
        env->mode = DFU_CTRL_BOOTLOADER;
#else
        env->mode = DFU_CTRL_NORMAL_MODE;
#endif

        init_flash_des();
        dfu_power_on_check();

        env->dfu_flash_thread = NULL;
        env->mb_handle = NULL;

#ifdef OTA_MODEM_RECORD
        uint8_t temp_state = env->prog.modem_ota_state;
#endif
        env->is_mount = 0;
        if (env->prog.state == DFU_CTRL_INSTALL)
        {
            // dfu fail, clear process
            LOG_E("ota interrupt, clear !!!");
            memset(&env->prog, 0, sizeof(dfu_download_progress_ext_t));
            dfu_ctrl_update_prog_info_ext(env);
        }

        if (env->prog.res_state == DFU_CTRL_UPDATING)
        {
            env->prog.res_state = DFU_CTRL_UPDATED;
        }


        if (env->prog.state == DFU_CTRL_UPDATING)
        {
            // dfu reboot
            // dfu_update_img_header(env);
            if (env->prog.res_state == DFU_CTRL_UPDATED)
            {
                memset(&env->prog, 0, sizeof(dfu_download_progress_ext_t));
                LOG_D("clear due to res success");
                dfu_ctrl_update_prog_info_ext(env);
                env->prog.state = DFU_CTRL_UPDATED;
                env->prog.res_state = DFU_CTRL_UPDATED;
            }
            else
            {
                LOG_D("clear due to success");
                memset(&env->prog, 0, sizeof(dfu_download_progress_ext_t));
                dfu_ctrl_update_prog_info_ext(env);
                env->prog.state = DFU_CTRL_UPDATED;
            }
        }
        else
        {
            env->prog.state = DFU_CTRL_IDLE;
            env->prog.res_state = DFU_CTRL_IDLE;
        }

#ifdef OTA_MODEM_RECORD
        env->prog.modem_ota_state = temp_state;
#endif

    }
    while (0);
    // debug use
    //env->prog.state = DFU_CTRL_FORCE_UPDATE;

    dfu_set_ota_update_state(ota_update);
    dfu_set_ota_version(&old, &cur);

#endif
    LOG_I("dfu_ctrl_ext_init");
    dfu_sec_init();
    return 0;

}

#ifdef FDB_USING_FILE_MODE
    INIT_PRE_APP_EXPORT(dfu_ctrl_ext_init);
#else
    INIT_ENV_EXPORT(dfu_ctrl_ext_init);
#endif /* FDB_USING_FILE_MODE */


uint8_t dfu_get_res_state()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    return env->prog.res_state;
}

uint8_t dfu_get_image_state()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    return env->prog.state;
}

void dfu_set_ota_mode(uint8_t mode)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->prog.ota_mode = mode;
    dfu_ctrl_update_prog_info_ext(env);
}

uint8_t dfu_get_ota_mode()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (!env->is_init)
    {
        return 0xFF;
    }
    return env->prog.ota_mode;
}

uint8_t dfu_get_download_state()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (env->prog.res_state == DFU_CTRL_TRAN_START || env->prog.state == DFU_CTRL_TRAN_START)
    {
        return 1;
    }
    return 0;
}

dfu_vesion_t dfu_get_ota_old_version()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    return env->prog.old_version;
}

dfu_vesion_t dfu_get_ota_new_version()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    return env->prog.new_version;
}

uint8_t dfu_get_ota_update_state()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    return env->prog.ota_update;
}

void dfu_set_ota_version(dfu_vesion_t *old_version, dfu_vesion_t *new_version)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (old_version)
    {
        memcpy(&env->prog.old_version, old_version, sizeof(dfu_vesion_t));
    }
    if (new_version)
    {
        memcpy(&env->prog.new_version, new_version, sizeof(dfu_vesion_t));
    }
    //dfu_ctrl_update_prog_info_ext(env);
}

void dfu_set_ota_update_state(uint8_t ota_update)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->prog.ota_update = ota_update;
    //dfu_ctrl_update_prog_info_ext(env);
}

#ifdef OTA_MODEM_RECORD
uint8_t dfu_get_modem_state()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (!env->is_init)
    {
        return 0xFF;
    }
    return env->prog.modem_ota_state;
}

uint8_t dfu_set_modem_state(uint8_t state)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->prog.modem_ota_state = state;
    dfu_ctrl_update_prog_info_ext(env);
    return state;
}
#endif

static int32_t offu32(uint32_t val)
{
    uint32_t op;
    op = val - 1;
    int32_t origin = 0;
    uint32_t temp = 0;

    if (!((val >> 24) & 0x80))
    {
        return val;
    }

    for (int i = 0; i < 31; i++)
    {
        temp = op >> i;
        temp = ~temp;
        temp &= 0x01;
        temp = temp << i;
        origin += temp;
    }

    if ((op >> 24) & 0x80)
    {
        origin = -origin;
    }

    return origin;
}


// should be psram sec
/*
APP_L2_CACHE_RET_OTA_SECT_BEGIN(ota_psram_ret_cache)
ALIGN(4) static uint8_t ctrl_data[50 * 1024];
ALIGN(4) static uint8_t diff_data[100 * 1024];

//ALIGN(4) static uint8_t diff_data[27 * 100 * 1024];
ALIGN(4) static uint8_t extra_data[50 * 1024];
ALIGN(4) static uint8_t com_diff_data[200 * 1024];
APP_L2_CACHE_RET_OTA_SECT_END
*/



// for build
uint8_t ctrl_data[100];
uint8_t diff_data[100];
uint8_t extra_data[100];
uint8_t com_diff_data[100];

int g_a = 0;

static int ota_patch_install()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    LOG_I("ota_patch_install");

    uint8_t buf[8];
    int32_t old_pos, new_pos;
    int32_t ctrl[3];
    int32_t i;

    old_pos = 0;
    new_pos = 0;

    uint32_t old_size = env->patch_state.old_file_len;
    uint32_t new_size = env->patch_state.new_file_len;

    uint32_t ctrl_index = 0;
    uint32_t diff_index = 0;
    uint32_t extra_index = 0;

    uint32_t ctrl_temp;
    uint32_t new_buf_size;
    uint32_t old_buf_size;

    uint8_t *new_buf;
    uint8_t *old_buf;


    while (new_pos < new_size)
    {
        // read ctrl data
        for (i = 0; i <= 2; i++)
        {
            memcpy(&ctrl_temp, ctrl_data + ctrl_index, sizeof(int32_t));
            ctrl_index += sizeof(int32_t);
            ctrl[i] = offu32(ctrl_temp);
        }

        if (ctrl[0] < 0 || ctrl[0] > 2147483647 ||
                ctrl[1] < 0 || ctrl[1] > 2147483647 ||
                new_pos + ctrl[0] > new_size)
        {
            return -1;
        }

        new_buf = malloc(ctrl[0]);
        OS_ASSERT(new_buf);

        // read diff string
        memcpy(new_buf, diff_data + diff_index, ctrl[0]);
        diff_index += ctrl[0];


        // read old buffer
        old_buf = malloc(ctrl[0]);
        OS_ASSERT(old_buf);
        dfu_packet_read_flash_ext(env->running_hcpu, old_pos, old_buf, ctrl[0]);


        // add old data to diff string
        for (i = 0; i < ctrl[0]; i++)
        {
            if ((old_pos + i >= 0) && (old_pos + i < old_size))
            {
                new_buf[i] += old_buf[i];
            }
        }

        free(old_buf);
        // write diff to flash
        // dfu_packet_write_flash_ext(DFU_NAND_HCPU_BACK_UP_ADDR, new_pos, new_buf, ctrl[0]);
        free(new_buf);

        new_pos += ctrl[0];
        old_pos += ctrl[0];

        if (new_pos + ctrl[1] > new_size)
        {
            return -1;
        }

        new_buf = malloc(ctrl[1]);
        OS_ASSERT(ctrl[1]);
        // read extra data
        memcpy(new_buf, extra_data + extra_index, ctrl[1]);
        extra_index += ctrl[1];

        // write diff to flash
        // dfu_packet_write_flash_ext(DFU_NAND_HCPU_BACK_UP_ADDR, new_pos, new_buf, ctrl[1]);
        free(new_buf);

        new_pos += ctrl[1];
        old_pos += ctrl[2];
    }




    return 0;
}
/*
#define CB_IS_IN_ITCM_RANGE(addr)    ((((addr) >= HPSYS_ITCM_BASE) && ((addr) < HPSYS_ITCM_END)) ? true : false)

#define CB_IS_IN_RETM_RANGE(addr)    ((((addr) >= HPSYS_RETM_BASE) && ((addr) < HPSYS_RETM_END)) ? true : false)

#define CB_IS_IN_EZIP_ADDR_RANGE(addr)  (!CB_IS_IN_ITCM_RANGE((addr)) && !CB_IS_IN_RETM_RANGE((addr)))
*/

#define CB_IS_IN_EZIP_ADDR_RANGE(addr)  (true)
static void uncompress_patch(uint8_t *compress_buf, uint32_t packet_len, uint8_t *uncompress_buf)
{
    int r;
    // compress_buf may not 4 byte aligned, so malloc new one.
    /*
        uint8_t *temp_compress_buf;
        uint32_t compress_len = packet_len;
        temp_compress_buf = malloc(compress_len);
        OS_ASSERT(temp_compress_buf);
        memcpy(temp_compress_buf, compress_buf + sizeof(uint32_t), compress_len);
    */
#ifdef BSP_USING_EPIC
    drv_epic_take(RT_WAITING_FOREVER);
#endif
    if (CB_IS_IN_EZIP_ADDR_RANGE((uint32_t)uncompress_buf) && CB_IS_IN_EZIP_ADDR_RANGE((uint32_t)compress_buf))
    {
        EZIP_DecodeConfigTypeDef config;
        EZIP_HandleTypeDef ezip_handle = {0};

        config.input = (uint8_t *)((uint32_t)compress_buf + sizeof(uint32_t));
        config.output = uncompress_buf;
        config.start_x = 0;
        config.start_y = 0;
        config.width = 0;
        config.height = 0;
        config.work_mode = HAL_EZIP_MODE_GZIP;
        config.output_mode = HAL_EZIP_OUTPUT_AHB;
        // ezip_handle.Instance = hwp_ezip;
        HAL_EZIP_Init(&ezip_handle);

        // disbale interrupt
        register rt_base_t ret;
        ret = rt_hw_interrupt_disable();
        r = HAL_EZIP_Decode(&ezip_handle, &config);
        rt_hw_interrupt_enable(ret);
        RT_ASSERT(HAL_OK == r);
    }
    else
    {
        OS_ASSERT(0);
    }
#ifdef BSP_USING_EPIC
    drv_epic_release();
#endif
    //free(temp_compress_buf);

}


#define BSDIFF_HEADER_LEN 40
static uint8_t load_patch_file()
{
    LOG_I("load_patch_file");
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();

    uint8_t header[BSDIFF_HEADER_LEN];

    // read from flash
    dfu_packet_read_flash_ext(DFU_NAND_PATCH_DOWNLOAD_ADDR, 0, header, BSDIFF_HEADER_LEN);

    if (memcmp(header, "BSDIFF40", 8) != 0)
    {
        LOG_I("MAGIC ERROR");
        return DFU_ERR_GENERAL_ERR;
    }

    uint32_t com_ctrl_len;
    uint32_t com_diff_len;
    uint32_t com_extra_len;

    uint32_t ctrl_len;
    uint32_t diff_len;
    uint32_t extra_len;
    memcpy(&com_ctrl_len, header + 8, sizeof(uint32_t));
    memcpy(&com_diff_len, header + 8 + sizeof(uint32_t), sizeof(uint32_t));
    memcpy(&com_extra_len, header + 8 + sizeof(uint32_t) * 2, sizeof(uint32_t));

    memcpy(&env->patch_state.old_file_len, header + 8 + sizeof(uint32_t) * 3, sizeof(uint32_t));
    memcpy(&env->patch_state.new_file_len, header + 8 + sizeof(uint32_t) * 4, sizeof(uint32_t));

    memcpy(&ctrl_len, header + 8 + sizeof(uint32_t) * 5, sizeof(uint32_t));
    memcpy(&diff_len, header + 8 + sizeof(uint32_t) * 6, sizeof(uint32_t));
    memcpy(&extra_len, header + 8 + sizeof(uint32_t) * 7, sizeof(uint32_t));

    LOG_I("load_patch_file ctrl %d, diff %d, extra %d", ctrl_len, diff_len, extra_len);
    LOG_I("load_patch_file compress ctrl %d, diff %d, extra %d", com_ctrl_len, com_diff_len, com_extra_len);

    uint32_t buf_len;
    uint8_t *compress_data;
    //uint8_t *uncompress_data;

    buf_len = com_ctrl_len;
    if (com_ctrl_len % 2048 != 0)
    {
        buf_len = ((com_ctrl_len / 2048) + 1) * 2048;
    }
    buf_len = 4096;
    LOG_I("uncompress ctrl data %d", buf_len);
    compress_data = malloc(buf_len);
    //uncompress_data = malloc(6000);

    OS_ASSERT(compress_data);
    //OS_ASSERT(uncompress_data);
    memset(compress_data, 0, buf_len);

    // read from flash,
    dfu_packet_read_flash_ext(DFU_NAND_PATCH_DOWNLOAD_ADDR, BSDIFF_HEADER_LEN, com_diff_data, com_ctrl_len);

    //LOG_I("uncompress ctrl data 0x%x 0x%x 0x%x 0x%x", *compress_data, *(compress_data+1), *(compress_data+2),*(compress_data+3));
    //LOG_I("uncompress ctrl data 0x%x", compress_data);
    LOG_I("uncompress ctrl data 0x%x", &com_diff_data[0]);
    LOG_I("uncompress ctrl data 0x%x", &ctrl_data[0]);

    //uint8_t *utemp_data = malloc(6*1024);

    SCB_InvalidateDCache_by_Addr((void *)compress_data, buf_len);
    SCB_InvalidateDCache_by_Addr(com_diff_data, 200 * 1024);
    SCB_InvalidateDCache_by_Addr(ctrl_data, 100 * 1024);
    uncompress_patch(com_diff_data, com_ctrl_len, ctrl_data);
    free(compress_data);

    LOG_I("uncompress ctrl data");



    LOG_I("uncompress diff");


    //compress_data = malloc(buf_len);
    //OS_ASSERT(compress_data);
    // read from flash,
    dfu_packet_read_flash_ext(DFU_NAND_PATCH_DOWNLOAD_ADDR, BSDIFF_HEADER_LEN + com_ctrl_len, com_diff_data, com_diff_len);

    SCB_InvalidateDCache_by_Addr(com_diff_data, 200 * 1024);
    SCB_InvalidateDCache_by_Addr(diff_data, 512 * 1024);
    uncompress_patch(com_diff_data, com_diff_len, diff_data);
    //free(compress_data);
    LOG_I("uncompress diff data");



    LOG_I("uncompress extra");
    buf_len = com_extra_len;

    if (com_extra_len % 2048 != 0)
    {
        buf_len = ((com_extra_len / 2048) + 1) * 2048;
    }

    compress_data = malloc(buf_len);
    OS_ASSERT(compress_data);
    // read from flash,
    dfu_packet_read_flash_ext(DFU_NAND_PATCH_DOWNLOAD_ADDR, BSDIFF_HEADER_LEN + com_ctrl_len + com_diff_len, compress_data, com_extra_len);

    //SCB_InvalidateDCache_by_Addr((void *)compress_data, buf_len);
    SCB_InvalidateDCache_by_Addr(extra_data, 100 * 1024);
    uncompress_patch(compress_data, com_extra_len, extra_data);
    LOG_I("uncompress extra data finis"
         );
    free(compress_data);
    LOG_I("uncompress extra data");



    return 0;
}

/* Convert to the structure dfu_code_image_header_t. */
static uint16_t dfu_ctrl_get_img_header_len(uint8_t *data)
{
    uint8_t *img_header = data + 15 + DFU_KEY_SIZE;
    uint8_t img_count = *(img_header + 2);
    return (uint16_t)(sizeof(dfu_code_image_header_t) + img_count * sizeof(dfu_image_header_int_t));
}

/*
 * Control packet content structure.
 * -----------------------------------------------------------------------------------------------------------------------------
 * |DFUID(1B) | HW_version(4B)| SDK_version(4B) | FW_version(4B) | DFU KEY(32B) | img_header_len(2B) | img_header(Varaible) |
 * -----------------------------------------------------------------------------------------------------------------------------
*/

static dfu_control_packet_t *dfu_ctrl_ctrl_header_alloc(uint8_t *data, uint16_t packed_len)
{
    uint8_t *ori_data = data;
    uint16_t image_header_len = dfu_ctrl_get_img_header_len(data);
    dfu_control_packet_t *packet = malloc(sizeof(dfu_control_packet_t) + image_header_len);
    OS_ASSERT(packet);
    dfu_code_image_header_t *img_header = (dfu_code_image_header_t *)(&packet->image_header);

    packet->dfu_ID = *data++;

    memcpy((uint8_t *)&packet->HW_version, data, sizeof(packet->HW_version));
    data += sizeof(packet->HW_version);

    memcpy((uint8_t *)&packet->SDK_version, data, sizeof(packet->SDK_version));
    data += sizeof(packet->SDK_version);

    memcpy((uint8_t *)&packet->FW_version, data, sizeof(packet->FW_version));
    data += sizeof(packet->FW_version);

    memcpy(packet->FW_key, data, sizeof(packet->FW_key));
    data += sizeof(packet->FW_key);

    packet->image_header_len = image_header_len;
    data += 2;

    memcpy((uint8_t *)&img_header->blk_size, data, sizeof(img_header->blk_size));
    data += sizeof(img_header->blk_size);

    img_header->img_count = *data++;

    for (uint32_t i = 0; i < img_header->img_count; i++)
    {
        memcpy(img_header->img_header[i].sig, data, sizeof(img_header->img_header[i].sig));
        data += sizeof(img_header->img_header[i].sig);

        memcpy((uint8_t *)&img_header->img_header[i].length, data, sizeof(img_header->img_header[i].length));
        data += sizeof(img_header->img_header[i].length);

        memcpy((uint8_t *)&img_header->img_header[i].flag, data, sizeof(img_header->img_header[i].flag));
        data += sizeof(img_header->img_header[i].flag);

        img_header->img_header[i].img_id = *data++;
    }
    OS_ASSERT(packed_len == (data - ori_data));
    return packet;

}

static void dfu_ctrl_ctrl_header_free(uint8_t *data)
{
    free(data);
}

/* Signature following packet. */
static uint8_t dfu_ctrl_packet_check(dfu_ctrl_ext_env_t *env, dfu_control_packet_t *packet, uint16_t total_len)
{
    uint8_t ret = DFU_ERR_GENERAL_ERR;
    do
    {

        /* Step 1. Check dfuID */
        if (packet->dfu_ID > DFU_ID_DL)
            break;

        /* Step 2. Check HW ID. */
        uint32_t hw_ver = dfu_ctrl_get_current_HW_version();

        //TODO: Should define how to compare HW version

        /* Step 3. Chekc SDK ID. */
        uint32_t sdk_ver = dfu_ctrl_get_current_SDK_version();
        /* Expected SDK version should not large than current SDK. */
        if (packet->SDK_version > sdk_ver)
        {
            ret = DFU_ERR_SDK_VER_ERR;
            break;
        }

        /* Following steps is related to dfuID. */
        if (packet->dfu_ID == DFU_ID_CODE || packet->dfu_ID == DFU_ID_CODE_MIX)
        {
            /* Step 4. Check FW version. */
            if (dfu_ctrl_compare_FW_version() == 0)
            {
                ret = DFU_ERR_FW_VER_ERR;
                break;
            }
            /* Step 5. Check OTA download state. */
            if (env->prog.FW_state >= DFU_CTRL_FW_DOWNLOADING)
            {
                ret = DFU_ERR_OTA_ONGOING;
                break;
            }
            /* Step 6. Check image info. */
            //TODO Should get flash size
            {
                dfu_code_image_header_t *header = (dfu_code_image_header_t *)&packet->image_header;
                // TODO: header check
                //ret = dfu_ctrl_packet_img_header_check(env, header);
                ret = DFU_ERR_NO_ERR;
                if (ret != DFU_ERR_NO_ERR)
                    break;
            }

            /* All verificaition PASS.*/
            ret = DFU_ERR_NO_ERR;
        }


    }
    while (0);
    return ret;

}

static uint8_t check_current_img_need_overwrite(int img_id)
{
    if (img_id == DFU_IMG_ID_RES ||
            img_id == DFU_IMG_ID_DYN ||
            img_id == DFU_IMG_ID_MUSIC ||
            img_id == DFU_IMG_ID_PIC ||
            img_id == DFU_IMG_ID_FONT ||
            img_id == DFU_IMG_ID_RING ||
            img_id == DFU_IMG_ID_LANG)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


static uint16_t dfu_ctrl_packet_code_fw_handler(dfu_ctrl_ext_env_t *env, dfu_control_packet_t *ctrl_packet, uint16_t len)
{
    LOG_I("dfu_ctrl_packet_code_fw_handler");
    uint16_t status = DFU_ERR_NO_ERR;
    do
    {
        /* If user callback doesn't register, always reject. */
        if (!env->callback)
        {
            LOG_I("dfu_ctrl_packet_code_fw_handler reject");
            //status = DFU_ERR_USER_REJECT;
            //break;
        }

        status = dfu_ctrl_packet_check(env, ctrl_packet, len);
        LOG_I("dfu_ctrl_packet_code_fw_handler dfu_ctrl_packet_check %d", status);
        if (status != DFU_ERR_NO_ERR)
        {
            LOG_W("ctrl packet check");
            break;
        }

        /* Notify user */
        dfu_app_start_request_ext_t app_req;
        dfu_event_ack_t ret;
        app_req.dfu_id = ctrl_packet->dfu_ID;
        app_req.event = DFU_APP_START_REQUEST;
        app_req.is_nand_overwrite = 0;
        uint16_t len = sizeof(dfu_app_start_request_ext_t);

        dfu_code_image_header_t *header = (dfu_code_image_header_t *)&ctrl_packet->image_header;
        env->prog.state = DFU_CTRL_NEG;
        env->prog.dfu_ID = ctrl_packet->dfu_ID;

        uint8_t *aes_out;
        aes_out = rt_malloc(DFU_KEY_SIZE);
        rt_memset(aes_out, 0, DFU_KEY_SIZE);

        SCB_InvalidateDCache_by_Addr(aes_out, DFU_KEY_SIZE);
        SCB_InvalidateICache_by_Addr(aes_out, DFU_KEY_SIZE);

        sifli_hw_enc(ctrl_packet->FW_key, aes_out, DFU_KEY_SIZE);
        rt_memcpy(env->prog.FW_key, aes_out, DFU_KEY_SIZE);
        rt_free(aes_out);


        env->prog.FW_version = ctrl_packet->FW_version;
        env->prog.SDK_version = ctrl_packet->SDK_version;
        env->prog.HW_version = ctrl_packet->HW_version;
        env->prog.fw_context.code_img.blk_size = header->blk_size;
        env->prog.fw_context.code_img.img_count = header->img_count;
        rt_memcpy((uint8_t *)&env->prog.fw_context.code_img.img_header, (uint8_t *)&header->img_header, sizeof(dfu_image_header_int_t) * header->img_count);

        for (int i = 0; i < header->img_count; i++)
        {
            if (check_current_img_need_overwrite(env->prog.fw_context.code_img.img_header[i].img_id) == 1)
            {
                app_req.is_nand_overwrite = 1;
                break;
            }
        }
        // TODO: user rsp

        if (env->callback)
        {
            ret = env->callback(DFU_APP_START_REQUEST, len, &app_req);
        }
        ret = DFU_EVENT_SUCCESSED;

        if (ret == DFU_EVENT_FAILED)
        {
            status = DFU_ERR_USER_REJECT;
            break;
        }

        /* If user choose reponse later, dfu should not ack immediately. */
        if (ret == DFU_EVENT_POSTPONE)
        {
            status = DFU_ERR_POSTPONE;
        }

    }
    while (0);

    LOG_I("dfu_ctrl_packet_code_fw_handler %d", status);
    return status;
}

static void dfu_ctrl_request_init_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len, uint8_t is_force)
{
    LOG_I("dfu_ctrl_request_init_handler");
    env->prog.state = DFU_CTRL_IDLE;
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    uint8_t status = DFU_ERR_GENERAL_ERR;

    /* 1. Hash and verify. */
    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    data += sizeof(struct image_cfg_hdr);
    len -= sizeof(struct image_cfg_hdr);
    uint8_t *sig = malloc(DFU_SIG_SIZE);
    memcpy(sig, data + len - DFU_SIG_SIZE, DFU_SIG_SIZE);
    uint8_t *packet = (uint8_t *)dfu_dec_verify(NULL, 0, data, data, len - DFU_SIG_SIZE, hdr->hash);
    do
    {
        if (!env->is_init)
        {
            status = DFU_ERR_NOT_READY;
            break;
        }
        if (!packet)
        {
            LOG_W("Ctrl packet parser failed!");
            break;
        }

        /* First check the signature.*/
        if (dfu_ctrl_ctrl_header_sig_verify_ext(packet, len, sig) < 0)
        {
            // TODO: verify sig
            LOG_E("dfu_ctrl_ctrl_header_sig_verify_ext FAIL!!!!!!!!!!!!!!");
            status = DFU_ERR_CONTROL_PACKET_INVALID;
            break;
        }

        uint8_t dfu_ID = *packet;
        switch (state)
        {
        /* In normal mode. */
        case DFU_CTRL_IDLE:
        case DFU_CTRL_FORCE_UPDATE:
        {
            /* OTA mode could have IDLE state if no need to reboot. */
            //if (env->mode != DFU_CTRL_NORMAL_MODE)
            //break;
            if (dfu_ID == DFU_ID_CODE ||
                    dfu_ID == DFU_ID_CODE_MIX)
            {
                /* ignore current download state. */
                if (is_force)
                {
                    env->prog.FW_state = DFU_CTRL_FW_NO_STATE;

                    // if set this under user bin, next reboot will clear so can jump to user if
                    // hcpu fail, but will not jump if downloading under force update
                    env->is_force_update = 1;
                }
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_handler(env, ctrl_packet, len);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
            }
            else
            {
                //TODO Support other OTA method, before implment it, just reponse err.
            }
            break;
        }
        default:
        {
            status = DFU_ERR_UNEXPECT_STATE;
            break;
        }
        }


    }
    while (0);

    // set to none as code ota will going
    //env->ota_state.state = OTA_STATE_NONE;

    LOG_I("dfu_ctrl_request_init_handler %d", status);

    if (sig)
        free(sig);
    /* Prepare response. */
    if (status != DFU_ERR_POSTPONE)
    {
        dfu_init_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE, dfu_init_response_t);
        rsp->result = status;
        rsp->is_boot = 0;
        /*
                if (status == DFU_ERR_NO_ERR)
                {
                    // set for no confirm from user
                    if (rsp->is_boot)
                    {
                        dfu_set_reboot_after_disconnect();
                    }
                }
        */
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (status == DFU_ERR_UNEXPECT_STATE)
        {
            int ret = dfu_protocol_session_close();
            if (ret == -1)
            {
                dfu_port_svc_session_close();
            }
        }
    }

}




OS_TIMER_DECLAR(g_dfu_sync_timer);
static void dfu_link_sync_end();

static void dfu_link_lose_check_req(uint32_t current_index, uint16_t new_num_of_rsp)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (env->is_abort)
    {
        return;
    }
    LOG_I("dfu_link_lose_check_req index %d, rsp %d", current_index, new_num_of_rsp);

    if (env->retrans_state.success_rsp_count == 0 || env->retrans_state.retry_count > 2)
    {
        env->retrans_state.success_rsp_count = 0;
        env->retrans_state.retry_count = 0;
        // reduce immediately
        if (new_num_of_rsp % 2 == 0)
        {
            new_num_of_rsp = new_num_of_rsp / 2;
        }
        else
        {
            new_num_of_rsp = 1;
        }
        LOG_I("dfu_link_lose_check_req new rsp %d", new_num_of_rsp);
    }
    else
    {
        env->retrans_state.retry_count++;
    }

    env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp = new_num_of_rsp;

    dfu_link_lose_check_req_t *res_rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_LINK_LOSE_CHECK_REQ, dfu_link_lose_check_req_t);
    res_rsp->result = DFU_ERR_NO_ERR;
    res_rsp->current_file_index = current_index;
    res_rsp->new_num_of_rsp = new_num_of_rsp;
    dfu_protocol_packet_send((uint8_t *)res_rsp);
}

static bool dfu_link_sync_check()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    uint32_t current_value = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num;
    if (env->sync_size == current_value)
    {
        LOG_I("dfu_link_sync_check error %d", env->sync_size);
        dfu_link_lose_check_req(env->sync_size, env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp);
        dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
        return false;
    }
    env->sync_size = current_value;
    return true;
}

static void dfu_sync_timer_handler(void *para)
{
    dfu_link_sync_end();

    bool ret = dfu_link_sync_check();
    if (ret)
    {
        dfu_link_sync_start(DFU_SYNC_TYPE_DOWNLOAD);
    }
}

static void dfu_link_lose_check_rsp(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_link_lose_check_rsp");
    dfu_link_lose_check_rsp_t *ind = (dfu_link_lose_check_rsp_t *)data;
    dfu_link_sync_end();

    if (env->prog.FW_state == DFU_CTRL_FW_DOWNLOADING)
    {
        LOG_I("dfu_link_lose_check_rsp start image");
        dfu_link_sync_start(DFU_SYNC_TYPE_DOWNLOAD);
    }
    else
    {
        // start file timer since download resume
        LOG_I("dfu_link_lose_check_rsp start file");
        dfu_link_sync_start(DFU_SYNC_TYPE_FILE);

    }
}

static void dfu_rsp_sync_handler(void *para)
{
    LOG_W("remote lost");
    dfu_link_sync_end();
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_ctrl_error_handle(env);
}


/*
static void dfu_file_sync_handler(void *para)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_link_sync_end();

    dfu_image_file_packet_response_t *res_rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_LINK_LOSE_CHECK_REQ, dfu_image_file_packet_response_t);
    res_rsp->result = DFU_ERR_INDEX_ERROR;
    res_rsp->current_file_index = env->res_prog.single_file_packet_download_count;
    res_rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
    dfu_protocol_packet_send((uint8_t *)res_rsp);

    dfu_link_sync_start(DFU_SYNC_TYPE_FILE);
}
*/

static void dfu_file_sync_handler(void *para)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_link_sync_end();
    uint32_t current_index = env->res_prog.single_file_packet_download_count;
    uint32_t new_num_of_rsp;

    if (env->prog.state == DFU_CTRL_TRAN_START)
    {
        current_index = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num;
        new_num_of_rsp = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp;
    }
    else if (env->prog.res_state == DFU_CTRL_TRAN_START)
    {
        current_index = env->res_prog.single_file_packet_download_count;
        new_num_of_rsp = env->res_prog.num_of_rsp;
    }
    else
    {
        // not in download mode, may disconnect
        LOG_E("unexpect sync handler state");
        dfu_ctrl_error_handle(env);
        return;
    }

    if (current_index < env->res_prog.num_of_rsp || env->retrans_state.retry_count > 2)
    {
        // first group miss

        if (new_num_of_rsp % 2 == 0)
        {
            new_num_of_rsp = new_num_of_rsp / 2;
        }
        else
        {
            new_num_of_rsp = 1;
        }
    }
    else
    {
        env->retrans_state.retry_count++;
    }

    dfu_link_lose_check_req(current_index, new_num_of_rsp);
    dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
}



void dfu_link_sync_start(uint8_t sync_type)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();

    // when using retransmission, we need a long timer
    // uint32_t timeout = ble_dfu_protocl_get_supervision_timeout();
    // timeout = timeout * 10;

    if (env->is_sync_timer_on == 1)
    {
        LOG_I("dfu_link_sync_start, timer already start");
        return;
    }

    env->is_sync_timer_on = 1;

    if (sync_type == DFU_SYNC_TYPE_DOWNLOAD)
    {
        os_timer_create(g_dfu_sync_timer, dfu_sync_timer_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_dfu_sync_timer, DFU_SYNC_DOWNLOAD_TIMER);
    }
    else if (sync_type == DFU_SYNC_TYPE_FILE)
    {
        os_timer_create(g_dfu_sync_timer, dfu_file_sync_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_dfu_sync_timer, DFU_SYNC_FILE_DOWNLOAD_TIMER);
    }
    else if (sync_type == DFU_SYNC_TYPE_RSP)
    {
        os_timer_create(g_dfu_sync_timer, dfu_rsp_sync_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_dfu_sync_timer, DFU_SYNC_RSP_TIMER);
    }
}

static void dfu_link_sync_end()
{
    LOG_I("dfu_link_sync_end");
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (env->is_sync_timer_on == 0)
    {
        LOG_I("dfu_link_sync_end, no timer to stop");
        return;
    }
    env->is_sync_timer_on = 0;
    os_timer_stop(g_dfu_sync_timer);
    os_timer_delete(g_dfu_sync_timer);
}

static void dfu_update_prog_thread()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_ctrl_update_prog_info_ext(env);
}

static void dfu_start_update_prog_info_thread()
{
    rt_thread_t tid;
    tid = rt_thread_create("dfu_prog_info", dfu_update_prog_thread, NULL, 4096, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);
}

static void dfu_flash_exit_msg_send()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (env->mb_handle)
    {
        flash_write_t *fwrite;
        fwrite = rt_malloc(sizeof(flash_write_t));
        OS_ASSERT(fwrite);
        fwrite->msg_type = DFU_FLASH_MSG_TYPE_EXIT;
        rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
    }
}

static void dfu_ctrl_error_handle(dfu_ctrl_ext_env_t *env)
{
    dfu_flash_exit_msg_send();
    if (env->prog.state == DFU_CTRL_IDLE && env->prog.res_state == DFU_CTRL_IDLE)
    {
        return;
    }

    if (env->prog.state == DFU_CTRL_UPDATING && env->prog.res_state == DFU_CTRL_UPDATING)
    {
        return;
    }
    if (env->is_abort)
    {
        // repeate
        return;
    }

#ifdef OTA_NAND_ONLY
    dfu_sec_config_free();
#endif
    LOG_E("dfu_ctrl_error_handle");

    dfu_file_error_ind_t ind;
    if (env->callback)
    {
        ind.event = DFU_APP_ERROR_DISCONNECT_IND;
        uint16_t len = sizeof(dfu_file_error_ind_t);
        env->callback(DFU_APP_ERROR_DISCONNECT_IND, len, &ind);
    }

    env->prog.state = DFU_CTRL_IDLE;
    env->prog.res_state = DFU_CTRL_IDLE;

    dfu_start_update_prog_info_thread();
    env->is_abort = 1;
}

static void dfu_img_send_start_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    dfu_link_sync_end();
    uint16_t state = env->prog.state;
    dfu_image_send_start_t *s_data = (dfu_image_send_start_t *)data;
    dfu_img_info_t *curr_img = &env->prog.fw_context.code_img.curr_img_info;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    uint16_t status = DFU_ERR_GENERAL_ERR;
    LOG_I("dfu_img_send_start_handler %d, %d, %d, %d", s_data->img_id, state, env->prog.FW_state, curr_img->img_state);
    uint16_t callback_len;
    if (env->dfu_flash_thread == NULL)
    {
        env->dfu_flash_thread = ble_dfu_flash_write_thread_start();
    }

    do
    {
        if (s_data->img_id > DFU_IMG_ID_MAX)
            break;

        switch (state)
        {
        case DFU_CTRL_NEG: // for resume in dfu mode
        case DFU_CTRL_PREPARE_START:
        {
            dfu_app_img_dl_start_ind_ext_t ind;
            LOG_I("dfu_img_send_start_handler fw state %d", env->prog.FW_state);
            /* Download is ongoing. */
            if (env->prog.FW_state == DFU_CTRL_FW_DOWNLOADING &&
                    curr_img->img_state > DFU_CTRL_IMG_STATE_IDLE)
            {
                LOG_I("dfu_img_send_start_info curr id: %d, len: %d, rsp: %d, total: %d", curr_img->img_id,
                      curr_img->img_length, curr_img->img_info.dl_info.num_of_rsp, curr_img->img_info.dl_info.total_pkt_num);
                env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                status = DFU_ERR_PARAMETER_INVALID;

                if (curr_img->img_id != s_data->img_id)
                    break;
                if (curr_img->img_length != s_data->img_length)
                    break;
                if (curr_img->img_info.dl_info.num_of_rsp != s_data->num_of_rsp)
                    break;
                if (curr_img->img_info.dl_info.total_pkt_num != s_data->total_pkt_num)
                    break;

                env->retrans_state.last_rsp_pkt_num = 0;
                env->retrans_state.last_rsp_img_length = 0;
                env->retrans_state.success_rsp_count = 0;
                env->retrans_state.retry_count = 0;
                env->prog.FW_state = DFU_CTRL_FW_DOWNLOADING;

                LOG_I("Download is ongoing");
                status = DFU_ERR_GENERAL_ERR;
            }
            else
            {
                curr_img->img_id = s_data->img_id;
                curr_img->img_length = s_data->img_length;
                curr_img->img_info.dl_info.num_of_rsp = s_data->num_of_rsp;

                env->retrans_state.last_rsp_pkt_num = 0;
                env->retrans_state.last_rsp_img_length = 0;
                env->retrans_state.success_rsp_count = 0;
                env->retrans_state.retry_count = 0;

                curr_img->img_info.dl_info.curr_img_length = 0;
                curr_img->img_info.dl_info.curr_pkt_num = 0;
                curr_img->img_info.dl_info.total_pkt_num = s_data->total_pkt_num;
                curr_img->header = dfu_img_get_img_header_by_img_id_ext(env, curr_img->img_id);
                curr_img->img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
                OS_ASSERT(curr_img->header);

                if (s_data->img_id == DFU_IMG_ID_LCPU)
                {
                    if (curr_img->header->length + DFU_LCPU_PATCH_DOWNLOAD_SIZE > DFU_NAND_PATCH_DOWNLOAD_SIZE)
                    {
                        LOG_I("over size: %d, %d", curr_img->header->length, DFU_NAND_PATCH_DOWNLOAD_SIZE);
                        OS_ASSERT(0);
                    }
                    curr_img->header->img_id = DFU_IMG_ID_LCPU_PATCH;
                    LOG_I("current id: %d,", curr_img->header->img_id);
                    dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length + DFU_LCPU_PATCH_DOWNLOAD_SIZE);
                    curr_img->header->img_id = DFU_IMG_ID_LCPU;
                }
                else if (s_data->img_id == DFU_IMG_ID_LCPU_PATCH)
                {
                    uint8_t is_erase = 0;
                    for (uint32_t i = 0; i < env->prog.fw_context.code_img.img_count; i++)
                    {
                        if (img_header[i].img_id == DFU_IMG_ID_LCPU)
                        {
                            LOG_I("already erase");
                            is_erase = 1;
                            break;
                        }
                    }

                    if (is_erase == 0)
                    {
                        dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length);
                    }
                }
                else
                {
                    dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length);
                }

                env->prog.FW_state = DFU_CTRL_FW_DOWNLOADING;
            }
            uint8_t *aes_in;
            uint8_t *aes_out;
            aes_in = rt_malloc(DFU_KEY_SIZE);
            aes_out = rt_malloc(DFU_KEY_SIZE);
            rt_memcpy(aes_in, env->prog.FW_key, DFU_KEY_SIZE);
            sifli_hw_dec_key(aes_in, aes_out, DFU_KEY_SIZE);
            rt_memcpy(dfu_ext_temp_key, aes_out, DFU_KEY_SIZE);

            rt_free(aes_in);
            rt_free(aes_out);

            env->prog.state = DFU_CTRL_TRAN_START;
            ind.total_imgs_num = env->prog.fw_context.code_img.img_count;
            ind.curr_img_id = s_data->img_id;
            ind.curr_img_total_len = s_data->img_length;
            ind.event = DFU_APP_IMAGE_DL_START_IND;
            callback_len = sizeof(dfu_app_img_dl_start_ind_ext_t);

            if (g_dfu_progress_mode == DFU_PROGRESS_TOTAL)
            {
                uint32_t all_size = 0;
                env->transported_size = 0;
                for (uint32_t i = 0; i < env->prog.fw_context.code_img.img_count; i++)
                {
                    if (img_header[i].img_id < ind.curr_img_id)
                    {
                        env->transported_size += img_header[i].length;
                    }
                    all_size += img_header[i].length;
                }
                ind.curr_img_total_len = all_size;
            }

            if (env->callback)
            {
                env->callback(DFU_APP_IMAGE_DL_START_IND, callback_len, &ind);
            }

            if (check_current_img_need_overwrite(curr_img->img_id))
            {
                env->prog.ota_mode = 2;
            }

            if (env->is_force_update && curr_img->img_id == DFU_IMG_ID_NAND_HCPU)
            {
                // do nothing so will not reboot in ota mode download hcpu.
            }
            else
            {
                dfu_ctrl_update_prog_info_ext(env);
            }

            status = DFU_ERR_NO_ERR;
        }
        break;
        case DFU_CTRL_TRAN_START:
        {

            dfu_app_img_dl_start_ind_ext_t ind;
            /* More than one image. */
            curr_img->img_id = s_data->img_id;
            curr_img->img_length = s_data->img_length;
            curr_img->img_info.dl_info.num_of_rsp = s_data->num_of_rsp;
            curr_img->img_info.dl_info.total_pkt_num = s_data->total_pkt_num;
            curr_img->header = dfu_img_get_img_header_by_img_id_ext(env, curr_img->img_id);
            curr_img->img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
            curr_img->img_info.dl_info.curr_pkt_num = 0;
            curr_img->img_info.dl_info.curr_img_length = 0;

            if (check_current_img_need_overwrite(curr_img->img_id))
            {
                env->prog.ota_mode = 2;
            }

            dfu_ctrl_update_prog_info_ext(env);

            if (s_data->img_id == DFU_IMG_ID_LCPU)
            {
                if (curr_img->header->length + DFU_LCPU_PATCH_DOWNLOAD_SIZE > DFU_NAND_PATCH_DOWNLOAD_SIZE)
                {
                    LOG_I("over size: %d, %d", curr_img->header->length, DFU_NAND_PATCH_DOWNLOAD_SIZE);
                    OS_ASSERT(0);
                }
                curr_img->header->img_id = DFU_IMG_ID_LCPU_PATCH;
                dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length + DFU_LCPU_PATCH_DOWNLOAD_SIZE);
                curr_img->header->img_id = DFU_IMG_ID_LCPU;
            }
            else if (s_data->img_id == DFU_IMG_ID_LCPU_PATCH)
            {
                uint8_t is_erase = 0;
                for (uint32_t i = 0; i < env->prog.fw_context.code_img.img_count; i++)
                {
                    if (img_header[i].img_id == DFU_IMG_ID_LCPU)
                    {
                        LOG_I("already erase");
                        is_erase = 1;
                        break;
                    }
                }

                if (is_erase == 0)
                {
                    dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length);
                }
            }
            else
            {
                dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length);
            }

            ind.total_imgs_num = env->prog.fw_context.code_img.img_count;
            ind.curr_img_id = s_data->img_id;
            ind.curr_img_total_len = s_data->img_length;
            ind.event = DFU_APP_IMAGE_DL_START_IND;
            callback_len = sizeof(dfu_app_img_dl_start_ind_ext_t);

            if (g_dfu_progress_mode == DFU_PROGRESS_TOTAL)
            {
                uint32_t all_size = 0;
                env->transported_size = 0;
                for (uint32_t i = 0; i < env->prog.fw_context.code_img.img_count; i++)
                {
                    if (img_header[i].img_id < ind.curr_img_id)
                    {
                        env->transported_size += img_header[i].length;
                    }
                    all_size += img_header[i].length;
                }
                ind.curr_img_total_len = all_size;
            }

            if (env->callback)
            {
                env->callback(DFU_APP_IMAGE_DL_START_IND, callback_len, &ind);
            }
            status = DFU_ERR_NO_ERR;
        }
        break;
        default:
            break;
        }
    }
    while (0);

    if (status == DFU_ERR_NO_ERR)
    {
        env->sync_size = 0;
        dfu_link_sync_start(DFU_SYNC_TYPE_DOWNLOAD);

        env->prog.image_download_state[curr_img->img_id] = 0;
    }

    // TODO: if status != DFU_ERR_NO_ERR, hci disconnect send earlier than start rsp
    dfu_image_send_start_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_START_RESPONSE, dfu_image_send_start_response_t);
    rsp->result = status;
    dfu_protocol_packet_send((uint8_t *)rsp);

    if (status != DFU_ERR_NO_ERR)
    {
        LOG_E("dfu_img_send_start_handler rsp error: %d", status);
        dfu_ctrl_error_handle(env);
    }
    else
    {
        ble_dfu_request_connection_priority();
    }
}

static void dfu_verification_handle()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;

    uint16_t status = dfu_img_verification_ext(env);
    if (status == DFU_ERR_NO_ERR)
    {
        curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED;
        dfu_app_img_dl_completed_ind_ext_t ind;
        ind.img_id = curr_info->img_id;
        ind.event = DFU_APP_IMAGE_DL_COMPLETED_IND;
        uint16_t len = sizeof(dfu_app_img_dl_completed_ind_ext_t);
        if (env->callback)
        {
            env->callback(DFU_APP_IMAGE_DL_COMPLETED_IND, len, (void *)&ind);
        }
    }
    else
        curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL;


    if (status == DFU_ERR_NO_ERR)
    {
        env->prog.image_download_state[curr_info->img_id] = 1;
    }

    LOG_I("dfu_img_send_end response");
    dfu_image_send_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_END_RESPONSE, dfu_image_send_end_response_t);
    rsp->result = status;
    dfu_protocol_packet_send((uint8_t *)rsp);

    //TODO error handle for packet err
    if (status != DFU_ERR_NO_ERR)
    {
        LOG_E("dfu_img_send_end_handler rsp error: %d", status);
        dfu_ctrl_error_handle(env);
    }
    else
    {
        dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
    }
}

static void ble_dfu_flash_write()
{
    LOG_I("ble_dfu_flash_write: try to recv a mail");
    int thread_run = 1;

    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->mb_handle = rt_mb_create("dfu_flash", 12, RT_IPC_FLAG_FIFO);

    while (thread_run)
    {
        flash_write_t *fwrite;
        uint32_t p;
        int ret = 0;
        if (rt_mb_recv(env->mb_handle, (rt_uint32_t *)&p, RT_WAITING_FOREVER) == RT_EOK)
        {
            //LOG_I("ble_dfu_flash_write %d", p);
            fwrite = (flash_write_t *)p;

            switch (fwrite->msg_type)
            {
            case DFU_FLASH_MSG_TYPE_DATA:
                // LOG_I("ble_dfu_flash_write OFFSET %d, SIZE %d, addr 0x%x", fwrite->offset, fwrite->size, fwrite);
                ret = dfu_packet_write_flash(fwrite->heade, fwrite->offset, fwrite->data, fwrite->size);
                rt_free(fwrite);

                if (ret != 0)
                {
                    LOG_I("ble_dfu_flash_write %d", ret);
                    OS_ASSERT(0);
                }
                break;
            case DFU_FLASH_MSG_TYPE_EXIT:
                LOG_I("DFU_FLASH_MSG_TYPE_EXIT");
                thread_run = 0;
                //ble_dfu_flash_thread_exit();
                rt_free(fwrite);
                break;
            case DFU_FLASH_MSG_TYPE_VERIFY:
                LOG_I("DFU_FLASH_MSG_TYPE_VERIFY");
                dfu_verification_handle();
                rt_free(fwrite);
                break;
            }
        }
    }

    if (env->mb_handle)
    {
        LOG_I("mb delete");
        rt_mb_delete(env->mb_handle);
        env->mb_handle = NULL;
    }

}

static rt_thread_t ble_dfu_flash_write_thread_start()
{
    LOG_I("ble_dfu_flash_write_thread_start");
    rt_thread_t tid;
    tid = rt_thread_create("ble_dfu_flash", ble_dfu_flash_write, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 5, 10);
    rt_thread_startup(tid);
    return tid;
}

static int dfu_packet_download(dfu_ctrl_ext_env_t *env, uint8_t img_id, uint8_t *data, int size)
{
    int r = -1;
    struct image_body_hdr *hdr = (struct image_body_hdr *)data;
    uint8_t key[DFU_KEY_SIZE] = {0};
    uint8_t *d = NULL;
    rt_err_t mb_ret;
    dfu_image_header_int_t *img_hdr = env->prog.fw_context.code_img.curr_img_info.header;

    size -= sizeof(struct image_body_hdr);
    data += sizeof(struct image_body_hdr);

    //sifli_hw_dec_key(env->prog.FW_key, key, DFU_KEY_SIZE);

    if (img_hdr->flag & DFU_FLAG_ENC)
    {
        uint8_t *dfu_temp;
        dfu_temp = rt_malloc(DFU_MAX_BLK_SIZE);
        OS_ASSERT(dfu_temp);
        d = dfu_dec_verify(dfu_ext_temp_key, hdr->offset, data, dfu_temp, size, hdr->hash);
        if (d)
        {
            if (env->mb_handle)
            {
                r = 0;
                flash_write_t *fwrite;
                fwrite = rt_malloc(sizeof(flash_write_t));
                OS_ASSERT(fwrite);
                fwrite->heade = img_hdr;
                fwrite->offset = hdr->offset;
                memcpy(fwrite->data, data, size);
                fwrite->size = size;
                fwrite->msg_type = DFU_FLASH_MSG_TYPE_DATA;

                //LOG_I("fwrite %d, %d, 0x%x", fwrite->offset, fwrite->size, fwrite);
                mb_ret = rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
                while (mb_ret != RT_EOK)
                {
                    LOG_I("MB RET %d", mb_ret);
                    //OS_ASSERT(0);
                    rt_thread_mdelay(100);
                    mb_ret = rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
                }
            }
            else
            {
                LOG_I("no mb");
                r = dfu_packet_write_flash(img_hdr, hdr->offset, data, size);
            }
        }
        else
        {
            LOG_E("body verify fail\n");
        }
        rt_free(dfu_temp);
    }
    else
    {
        if (dfu_integrate_verify(data, size, hdr->hash) == 0)
        {
            d = data;
        }
        if (d)
        {
            if (env->mb_handle)
            {
                r = 0;
                flash_write_t *fwrite;
                fwrite = rt_malloc(sizeof(flash_write_t));
                OS_ASSERT(fwrite);
                fwrite->heade = img_hdr;
                fwrite->offset = hdr->offset;
                memcpy(fwrite->data, data, size);
                fwrite->size = size;
                fwrite->msg_type = DFU_FLASH_MSG_TYPE_DATA;

                //LOG_I("fwrite %d, %d, 0x%x", fwrite->offset, fwrite->size, fwrite);
                mb_ret = rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
                while (mb_ret != RT_EOK)
                {
                    LOG_I("MB RET %d", mb_ret);
                    //OS_ASSERT(0);
                    rt_thread_mdelay(100);
                    mb_ret = rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
                }
            }
            else
            {
                LOG_I("no mb");
                r = dfu_packet_write_flash(img_hdr, hdr->offset, data, size);
            }
        }
        else
        {
            LOG_E("body verify fail\n");
        }
    }

    // Read image key
    //key = &(g_sec_config->imgs[coreid].key[0]);
    return r;
}

static void dfu_img_send_packet_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    dfu_image_send_packet_v2_t *packet;
    dfu_image_send_packet_t *packet_old;
    uint8_t packet_img_id;
    uint32_t packet_index;
    uint32_t packet_size;
    uint8_t *packet_data;
    if (env->remote_version >= 102)
    {
        packet = (dfu_image_send_packet_v2_t *)data;
        packet_data = packet->packet;
        packet_img_id = packet->img_id;
        packet_index = packet->pkt_idx;
        packet_size = packet->size;
    }
    else
    {
        packet_old = (dfu_image_send_packet_t *)data;
        packet_data = packet_old->packet;
        packet_img_id = packet_old->img_id;
        packet_index = packet_old->pkt_idx;
        packet_size = packet_old->size;
    }
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    uint16_t status = DFU_ERR_GENERAL_ERR;
    do
    {
        if (packet_img_id != curr_info->img_id)
        {
            status = DFU_ERR_PARAMETER_INVALID;
            break;
        }

        switch (state)
        {
        case DFU_CTRL_TRAN_START:
        {
            dfu_app_img_dl_progress_ind_ext_t ind;
            ind.event = DFU_APP_IMAGE_DL_ROPGRESS_IND;
            uint16_t len = sizeof(dfu_app_img_dl_progress_ind_ext_t);

            LOG_D("packet count(%d), len(%d)\r\n", packet_index, packet_size);
            int ret;
            if (curr_info->img_length < curr_info->img_info.dl_info.curr_img_length + packet_size)
            {
                break;
            }
            else if ((curr_info->img_info.dl_info.curr_pkt_num + 1) != packet_index)
            {
                LOG_W("error curr count(%d)\r\n", curr_info->img_info.dl_info.curr_pkt_num);

                // packet index error,
                // try lose check to continue
                LOG_W("stay link \r\n");
                dfu_link_lose_check_req(curr_info->img_info.dl_info.curr_pkt_num, curr_info->img_info.dl_info.num_of_rsp);
                dfu_link_sync_start(DFU_SYNC_TYPE_RSP);

                status = DFU_ERR_NO_ERR;

                break;
            }
            //else if (packet->size > env->prog.fw_context.code_img.blk_size)
            //break;

            ret = dfu_packet_download(env, curr_info->img_id, packet_data, packet_size);
            if (ret == -1)
            {
                LOG_W("packet error (%d), retry", curr_info->img_info.dl_info.curr_pkt_num);
                dfu_link_lose_check_req(curr_info->img_info.dl_info.curr_pkt_num, curr_info->img_info.dl_info.num_of_rsp);
                dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
                status = DFU_ERR_NO_ERR;
            }

            if (ret != 0)
            {
                LOG_E("dfu_packet_download fail!");
                break;
            }

            status = DFU_ERR_NO_ERR;
            curr_info->img_info.dl_info.curr_pkt_num++;
            curr_info->img_info.dl_info.curr_img_length += packet_size;

            ind.img_id = curr_info->img_id;
            ind.curr_img_recv_length = curr_info->img_info.dl_info.curr_img_length;

            if (g_dfu_progress_mode == DFU_PROGRESS_TOTAL)
            {
                ind.curr_img_recv_length = ind.curr_img_recv_length + env->transported_size - curr_info->img_info.dl_info.curr_pkt_num * sizeof(struct image_body_hdr);
            }

            // TODO: progress
            if (env->callback)
            {
                env->callback(DFU_APP_IMAGE_DL_ROPGRESS_IND, len, &ind);
            }


        }
        break;
        default:
            break;
        }
    }
    while (0);

    if (status == DFU_ERR_NO_ERR)
    {
        if (curr_info->img_info.dl_info.curr_pkt_num % curr_info->img_info.dl_info.num_of_rsp != 0 &&
                curr_info->img_info.dl_info.curr_pkt_num != curr_info->img_info.dl_info.total_pkt_num)
            return;
    }

    env->retrans_state.last_rsp_pkt_num = curr_info->img_info.dl_info.curr_pkt_num;
    env->retrans_state.last_rsp_img_length = curr_info->img_info.dl_info.curr_img_length;
    env->retrans_state.success_rsp_count++;
    env->retrans_state.retry_count = 0;

    dfu_image_send_packet_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_PACKET_RESPONSE, dfu_image_send_packet_response_t);
    rsp->result = status;
    dfu_protocol_packet_send((uint8_t *)rsp);


    if (status != DFU_ERR_NO_ERR)
    {
        dfu_ctrl_error_handle(env);
    }
}

static void dfu_img_send_end_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_img_send_end_handler");
    dfu_link_sync_end();
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    dfu_image_send_end_t *packet = (dfu_image_send_end_t *)data;
    uint16_t status = DFU_ERR_GENERAL_ERR;

    do
    {
        if (packet->img_id != curr_info->img_id)
        {
            status = DFU_ERR_PARAMETER_INVALID;
            break;
        }

        if (curr_info->img_info.dl_info.curr_pkt_num != curr_info->img_info.dl_info.total_pkt_num)
        {
            LOG_E("end packet error %d, %d", curr_info->img_info.dl_info.curr_pkt_num, curr_info->img_info.dl_info.total_pkt_num);
            status = DFU_ERR_PARAMETER_INVALID;
            break;
        }

        switch (state)
        {
        case DFU_CTRL_TRAN_START:
        {
            if (env->mb_handle)
            {
                flash_write_t *fwrite;
                fwrite = rt_malloc(sizeof(flash_write_t));
                OS_ASSERT(fwrite);
                fwrite->msg_type = DFU_FLASH_MSG_TYPE_VERIFY;

                rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
                return;
            }

            status = dfu_img_verification_ext(env);
            if (status == DFU_ERR_NO_ERR)
            {
                curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED;
                dfu_app_img_dl_completed_ind_ext_t ind;
                ind.img_id = curr_info->img_id;
                ind.event = DFU_APP_IMAGE_DL_COMPLETED_IND;
                uint16_t len = sizeof(dfu_app_img_dl_completed_ind_ext_t);
                if (env->callback)
                {
                    env->callback(DFU_APP_IMAGE_DL_COMPLETED_IND, len, (void *)&ind);
                }
            }
            else
                curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL;
            break;
        }
        default:
            break;
        }

    }
    while (0);

    if (status == DFU_ERR_NO_ERR)
    {
        env->prog.image_download_state[curr_info->img_id] = 1;
    }

    LOG_I("dfu_img_send_end response");
    dfu_image_send_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_END_RESPONSE, dfu_image_send_end_response_t);
    rsp->result = status;
    dfu_protocol_packet_send((uint8_t *)rsp);

    //TODO error handle for packet err
    if (status != DFU_ERR_NO_ERR)
    {
        LOG_E("dfu_img_send_end_handler rsp error: %d", status);
        dfu_ctrl_error_handle(env);
    }
    else
    {
        dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
    }
}

static void dfu_ctrl_install_completed(dfu_ctrl_ext_env_t *env, uint16_t status)
{
    dfu_app_img_install_completed_ind_ext_t ind;
    ind.event = DFU_APP_INSTALL_COMPLETD_IND;
    ind.result = status == DFU_ERR_NO_ERR ? DFU_APP_SUCCESSED : DFU_APP_FAIED;
    uint16_t len = sizeof(dfu_app_img_install_completed_ind_ext_t);
    ind.include_hcpu = 0;
    //env->callback(DFU_APP_INSTALL_COMPLETD_IND, &ind);

    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    for (uint32_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_NAND_HCPU_PATCH ||
                dl_header->img_header[i].img_id == DFU_IMG_ID_NAND_HCPU)
        {
            uint32_t hcpu_len = dl_header->img_header[i].length;
            LOG_I("hcpu len 0x%X", hcpu_len);
            HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, hcpu_len);

            ind.include_hcpu = 1;
            dfu_running_image_switch();
            break;
        }
    }

    if (status == DFU_ERR_NO_ERR)
    {
        //env->callback(DFU_APP_TO_USER, NULL);
        env->prog.state = DFU_CTRL_UPDATING;
        env->prog.FW_state = DFU_CTRL_FW_INSTALLED;
        env->is_force_update = 0;
        dfu_ctrl_update_prog_info_ext(env);

        // TODO: update nand
        dfu_update_img_header_ext(env);

        // process link disconnect after success
        env->is_abort = 1;
    }
    else
        /* Should disconnect link and enter force update state. */
    {
        env->prog.state = DFU_CTRL_IDLE;
        env->is_force = 1;
    }

    if (env->callback)
    {
        env->callback(DFU_APP_INSTALL_COMPLETD_IND, len, &ind);
    }
    if (status != DFU_ERR_NO_ERR)
    {
        dfu_ctrl_error_handle(env);
    }
    else
    {
        HAL_PMU_Reboot();
    }
}

static int check_patch_install(dfu_ctrl_ext_env_t *env)
{
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    uint8_t key[DFU_KEY_SIZE] = {0};
    int r = DFU_SUCCESS;

    for (uint32_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_NAND_HCPU_PATCH)
        {
            LOG_I("check_patch_install HCPU PATCH");
            /*
            load_patch_file();
            dfu_packet_erase_flash_ext(DFU_NAND_HCPU_BACK_UP_ADDR, 0, DFU_NAND_HCPU_BACK_UP_SIZE);
            int ret = ota_patch_install();

            if (ret == DFU_SUCCESS)
            {
                uint8_t des_state = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
                if (des_state == DFU_DES_RUNNING_ON_HCPU1)
                {
                    des_state = DFU_DES_SWITCH_TO_HCPU2;
                }
                else if (des_state == DFU_DES_RUNNING_ON_HCPU2)
                {
                    des_state = DFU_DES_SWITCH_TO_HCPU1;
                }
                else
                {
                    OS_ASSERT(0);
                }
                HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, des_state);
            }
            */

            break;
        }
        if (env->is_abort)
        {
            r = DFU_FAIL;
        }
    }

    return r;
}

static int dfu_update_lcpu(dfu_ctrl_ext_env_t *env, dfu_image_header_int_t img_header)
{
    uint32_t addr_offset = 0;
    uint32_t download_addr;
    if (img_header.img_id == DFU_IMG_ID_LCPU)
    {
        download_addr = DFU_LCPU_DOWNLOAD_ADDR;
    }
    else if (img_header.img_id == DFU_IMG_ID_LCPU_PATCH)
    {
        download_addr = DFU_LCPU_PATCH_DOWNLOAD_ADDR;
    }
    else
    {
        download_addr = 0xFFFFFFFF;
    }

    LOG_I("image id %d", img_header.img_id);
    uint32_t target_addr;
    if (img_header.img_id == DFU_IMG_ID_LCPU)
    {
#if !defined(SF32LB52X)
        target_addr = LCPU_FLASH_CODE_START_ADDR;
#endif
    }
    else if (img_header.img_id == DFU_IMG_ID_LCPU_PATCH)
    {
        target_addr = LCPU_PATCH_START_ADDR;
    }
    else
    {
        target_addr = 0xFFFFFFFF;
    }

    LOG_I("target addr 0x%x", target_addr);

    int ret;
    ret = dfu_packet_erase_flash_ext(target_addr, 0, img_header.length, DFU_FLASH_TYPE_NOR);
    LOG_I("erase target addr %d", ret);
    OS_ASSERT(ret == 0);

    uint32_t offset = 0;
    uint8_t *buf;
    uint32_t packet_size = 2048;

    buf = rt_malloc(packet_size);
    while (offset < img_header.length)
    {
        if (offset + 2048 > img_header.length)
        {
            packet_size = img_header.length - offset;
        }
        else
        {
            packet_size = 2048;
        }

        dfu_packet_read_flash_ext(download_addr, offset, buf, packet_size);
        ret = dfu_packet_write_flash_ext(target_addr, offset, buf, packet_size, DFU_FLASH_TYPE_NOR);
        OS_ASSERT(ret == 0);
        //LOG_I("write target addr ret %d, addr 0x%x", ret, target_addr);

        offset += packet_size;
    }
    rt_free(buf);

    return ret;
}

static int dfu_lcpu_recovery(dfu_ctrl_ext_env_t *env)
{
    LOG_I("dfu_lcpu_recovery");
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    int ret = 0;
    uint8_t halt = 0;

    for (uint8_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU ||
                dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU_PATCH)
        {
            if (halt == 0)
            {
                LOG_I("dfu_check_lcpu_upgrade halt and reset lcpu");
                halt = 1;

                // no need tos reset when power on
                //HAL_RCC_Reset_DMAC2_and_MPI5();
                //HAL_RCC_Reset_and_Halt_LCPU(0);

                // reset mpi controller
                //HAL_QSPIEX_FLASH_RESET2(hwp_mpi5);

                extern int rt_hw_flash5_init();
                int init_ret = rt_hw_flash5_init();
                LOG_I("dfu_check_lcpu_upgrade init flash5 ret %d", init_ret);
                if (init_ret != 1)
                {
                    OS_ASSERT(0);
                }
            }
            LOG_I("dfu_check_lcpu_upgrade lcpu id %d", dl_header->img_header[i].img_id);

            ret = dfu_update_lcpu(env, dl_header->img_header[i]);
        }
    }
    env->prog.lcpu_state = LCPU_INSTALL_UPDATED;
    dfu_ctrl_update_prog_info_ext(env);
    return ret;
}

static int dfu_check_lcpu_upgrade(dfu_ctrl_ext_env_t *env)
{
    LOG_I("dfu_check_lcpu_upgrade");
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    int ret = 0;
    uint8_t halt = 0;
    uint8_t update_lcpu = 0;

    for (uint8_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU ||
                dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU_PATCH)
        {
            LOG_I("update lcpu");
            update_lcpu = 1;
            break;
        }
    }

    for (uint8_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU ||
                dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU_PATCH)
        {
            if (halt == 0)
            {
                LOG_I("dfu_check_lcpu_upgrade halt and reset lcpu");
                halt = 1;
#if !defined(SF32LB52X)
#if defined(SF32LB56X)
                HAL_RCC_Reset_DMAC2_and_MPI5();
#endif
#if defined(SF32LB58X)
                HAL_RCC_Reset_DMAC3_and_MPI5();
#endif

                HAL_RCC_Reset_and_Halt_LCPU(0);

                // reset mpi controller
                HAL_QSPIEX_FLASH_RESET2(hwp_mpi5);
#endif
                extern int rt_hw_flash5_init();
                int init_ret = rt_hw_flash5_init();
                LOG_I("dfu_check_lcpu_upgrade init flash5 ret %d", init_ret);
                if (init_ret != 1)
                {
                    OS_ASSERT(0);
                }
            }
            LOG_I("dfu_check_lcpu_upgrade lcpu id %d", dl_header->img_header[i].img_id);

            ret = dfu_update_lcpu(env, dl_header->img_header[i]);
        }
    }

    if (update_lcpu)
    {
        if (env->prog.lcpu_state == LCPU_INSTALL_UPDATING)
        {
            env->prog.lcpu_state = LCPU_INSTALL_UPDATED;
        }
        dfu_ctrl_update_prog_info_ext(env);
    }

    return ret;
}

uint8_t update_backup_ftab()
{
    dfu_nand_info dfu_info;
    uint8_t ota_des = DFU_DES_NONE;
    struct sec_configuration *backup_sec_config;
    uint32_t sec_len = sizeof(struct sec_configuration);

    LOG_I("update_backup_ftab %d", sec_len);
    backup_sec_config = malloc(sec_len);

    //uint8_t des_state = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    dfu_packet_read_flash_ext(DFU_BACKUP_FTAB_ADDR, 0, (uint8_t *)backup_sec_config, sec_len);
    dfu_packet_read_flash_ext(DFU_BACKUP_FTAB_ADDR, DFU_INFO_OFFSET, (uint8_t *)&dfu_info, sizeof(dfu_nand_info));

    if (backup_sec_config->magic != SEC_CONFIG_MAGIC || dfu_info.magic != SEC_CONFIG_MAGIC)
    {
        LOG_I("update_backup_ftab back up ftab not ready");
        dfu_packet_read_flash_ext(DFU_FTAB_ADDR, 0, (uint8_t *)backup_sec_config, sec_len);
        ota_des = DFU_DES_RUNNING_ON_HCPU1;
    }

    int hcpu1_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU);
    int hcpu2_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2);

    struct image_header_enc running_image;
    dfu_packet_read_flash_ext((uint32_t)backup_sec_config->running_imgs[CORE_HCPU], 0, (uint8_t *)&running_image, sizeof(struct image_header_enc));


    LOG_I("running addr 0x%x", backup_sec_config->running_imgs[CORE_HCPU]);

    //LOG_I("backup_sec_config addr 0x%x", &(backup_sec_config));
    LOG_I("hcpu1 addr 0x%x", &(backup_sec_config->imgs[hcpu1_img_idx]));
    LOG_I("hcpu2 addr 0x%x", &(backup_sec_config->imgs[hcpu2_img_idx]));

    uint32_t hcpu_offet = &(backup_sec_config->imgs[hcpu2_img_idx]) - &(backup_sec_config->imgs[hcpu1_img_idx]);

    if (ota_des == DFU_DES_NONE)
    {

        LOG_I("ota_flash %d", dfu_info.running_target);
        ota_des = dfu_info.running_target;
    }


    if (ota_des == DFU_DES_RUNNING_ON_HCPU1)
    {
        LOG_I("set to hcpu2");
        ota_des = DFU_DES_RUNNING_ON_HCPU2;
        backup_sec_config->running_imgs[CORE_HCPU] = (struct image_header_enc *)(backup_sec_config->running_imgs[CORE_HCPU] + hcpu_offet);
    }
    else if (ota_des == DFU_DES_RUNNING_ON_HCPU2)
    {
        LOG_I("set to hcpu1");
        ota_des = DFU_DES_RUNNING_ON_HCPU1;
        backup_sec_config->running_imgs[CORE_HCPU] = (struct image_header_enc *)(backup_sec_config->running_imgs[CORE_HCPU] - hcpu_offet);
    }
    else
    {
        LOG_I("no image right");
    }

    LOG_I("new hcpu addr 0x%x", backup_sec_config->running_imgs[CORE_HCPU]);

    //OS_ASSERT(0);
    dfu_packet_erase_flash_ext(DFU_BACKUP_FTAB_ADDR, 0, 0x20000, DFU_FLASH_TYPE_NAND);

    uint32_t offset = 0;
    uint32_t block_len = 2048;

    while (offset < sec_len)
    {
        if (offset + block_len > sec_len)
        {
            // last packet
            uint32_t left_len;
            uint8_t *buff;
            left_len = sec_len - offset;
            buff = malloc(left_len);

            dfu_packet_write_flash_ext(DFU_BACKUP_FTAB_ADDR, offset, (uint8_t *)backup_sec_config + offset, left_len, DFU_FLASH_TYPE_NAND);
            offset += block_len;
            free(buff);
        }
        else
        {
            dfu_packet_write_flash_ext(DFU_BACKUP_FTAB_ADDR, offset, (uint8_t *)backup_sec_config + offset, block_len, DFU_FLASH_TYPE_NAND);
            offset += block_len;
        }
    }

    free(backup_sec_config);

    dfu_info.running_target  = ota_des;
    dfu_info.magic = SEC_CONFIG_MAGIC;

    dfu_packet_write_flash_ext(DFU_BACKUP_FTAB_ADDR, DFU_INFO_OFFSET, (uint8_t *)&dfu_info, sizeof(dfu_nand_info), DFU_FLASH_TYPE_NAND);
    return ota_des;
}

static void dfu_resume_reset(uint8_t state)
{
    LOG_I("dfu_resume_reset");
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->prog.state = state;
    env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
    dfu_ctrl_update_prog_info_ext(env);
}

static void dfu_power_on_check()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    uint8_t need_reboot = 0;
    uint8_t update_state = DFU_CTRL_FORCE_UPDATE;

    if (env->is_init == 0)
    {
        LOG_W("dfu_power_on_check not ready");
        return;
    }

    if (env->prog.lcpu_state == LCPU_INSTALL_UPDATING)
    {
        LOG_E("LCPU not ready");
        dfu_lcpu_recovery(env);
        need_reboot = 1;
    }

    if (env->prog.hcpu_target == DFU_NAND_HCPU_FIRST_ADDR || env->prog.hcpu_target == DFU_NAND_HCPU_BACK_UP_ADDR)
    {
        if (env->running_hcpu == env->prog.hcpu_target)
        {
            // running ok
        }
        else
        {
            LOG_E("hcpu not expected");
            dfu_running_image_switch();
            need_reboot = 1;
        }
    }

    if (env->prog.state == DFU_CTRL_INSTALL)
    {
        LOG_I("set ready for no res");
        update_state = DFU_CTRL_UPDATING;
    }

    if (need_reboot)
    {
        dfu_resume_reset(update_state);
        HAL_PMU_Reboot();
    }
}

uint8_t dfu_running_image_switch()
{
    uint8_t des_state = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
#ifdef OTA_NAND_ONLY
    des_state = update_backup_ftab();
#else
    LOG_I("dfu_running_image_switch current state %d", des_state);

    if (des_state == DFU_DES_RUNNING_ON_HCPU1)
    {
        des_state = DFU_DES_SWITCH_TO_HCPU2;
    }
    else if (des_state == DFU_DES_RUNNING_ON_HCPU2)
    {
        des_state = DFU_DES_SWITCH_TO_HCPU1;
    }
    else
    {
        LOG_I("dfu_running_image_switch unexpect state %d", des_state);
        OS_ASSERT(0);
        // HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, DFU_DES_RUNNING_ON_HCPU1);
        // des_state = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    }

    HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, des_state);
#endif
    LOG_I("dfu_running_image_switch set state %d", des_state);
    return des_state;
}

void dfu_update_hcpu_switch_lcpu_state_and_ota_mode(dfu_ctrl_ext_env_t *env)
{
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;

    dfu_app_img_install_start_ind_ext_t ind_over;
    ind_over.total_imgs_len = 0;
    ind_over.event = DFU_APP_DL_END_AND_INSTALL_START_IND;
    uint16_t len = sizeof(dfu_app_img_install_start_ind_ext_t);
    if (env->callback)
    {
        env->callback(DFU_APP_DL_END_AND_INSTALL_START_IND, len, &ind_over);
    }
    uint8_t update_info = 0;

    for (uint8_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU ||
                dl_header->img_header[i].img_id == DFU_IMG_ID_LCPU_PATCH)
        {
            LOG_I("update lcpu");
            env->prog.lcpu_state = LCPU_INSTALL_UPDATING;
            update_info = 1;
            break;
        }
    }

    for (uint32_t i = 0; i < dl_header->img_count; i++)
    {
        if (dl_header->img_header[i].img_id == DFU_IMG_ID_NAND_HCPU_PATCH ||
                dl_header->img_header[i].img_id == DFU_IMG_ID_NAND_HCPU)
        {
            update_info = 1;
            env->prog.total_receive_len = env->back_up_hcpu;
            dfu_running_image_switch();
            break;
        }
    }

    if (update_info)
    {
        LOG_I("dfu_update_hcpu_switch_lcpu_state_and_ota_mode update info");
        dfu_ctrl_update_prog_info_ext(env);
    }
}

void dfu_ctrl_last_packet_handler()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    uint8_t status = DFU_ERR_FW_INVALID;
    int ret = dfu_check_lcpu_upgrade(env);
    LOG_I("lcpu result %d", ret);
    if (ret == DFU_SUCCESS)
    {
        status = DFU_ERR_NO_ERR;
    }
    dfu_ctrl_install_completed(env, status);
}

uint8_t dfu_image_download_complete_check(dfu_ctrl_ext_env_t *env)
{
    uint8_t download_count = 0;
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;

    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        LOG_I("dfu_image_download_complete_check %d, %d", i, env->prog.image_download_state[i]);
    }

    for (int i = 0; i < dl_header->img_count; i++)
    {
        uint8_t current_id = dl_header->img_header[i].img_id;
        if (env->prog.image_download_state[current_id] == 1)
        {
            download_count++;
        }
        else
        {
            LOG_E("image id %d not download", current_id);
        }
    }

    if (download_count == dl_header->img_count)
    {
        return DFU_ERR_NO_ERR;
    }
    else
    {
        return DFU_ERR_PARAMETER_INVALID;
    }
}

static void dfu_img_tramission_end_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_img_tramission_end_handler");
    dfu_link_sync_end();
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    dfu_tranmission_end_t *packet = (dfu_tranmission_end_t *)data;
    uint16_t status = DFU_ERR_GENERAL_ERR;
    dfu_flash_exit_msg_send();

    switch (state)
    {
    case DFU_CTRL_TRAN_START:
    {
        dfu_app_img_install_start_ind_ext_t ind_over;
        ind_over.total_imgs_len = 0;
        ind_over.event = DFU_APP_DL_END_AND_INSTALL_START_IND;
        uint16_t len = sizeof(dfu_app_img_install_start_ind_ext_t);
        if (env->callback)
        {
            env->callback(DFU_APP_DL_END_AND_INSTALL_START_IND, len, &ind_over);
        }
        int ret;

        ret = check_patch_install(env);

        if (ret == DFU_SUCCESS)
        {
            status = DFU_ERR_NO_ERR;
        }

        // TODO FINAL
        status = dfu_image_download_complete_check(env);
        //dfu_update_hcpu_switch_lcpu_state_and_ota_mode(env);
        dfu_record_current_tx();
        dfu_end_int_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_END_IND, dfu_end_int_t);
        rsp->result = status;
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (status == DFU_ERR_NO_ERR)
        {
            // only update lcpu if hcpu success
            if (dfu_set_last_packet_wait() == 0)
            {
                // wait last packet send then we process install completed
            }
            else
            {
                // last packet already send, process now
                LOG_I("dfu_img_tramission_end");
                ret = dfu_check_lcpu_upgrade(env);
                if (ret == DFU_SUCCESS)
                {
                    status = DFU_ERR_NO_ERR;
                }
                else
                {
                    status = DFU_ERR_FW_INVALID;
                }
                LOG_I("lcpu result %d", ret);
                dfu_ctrl_install_completed(env, status);
            }
        }
        else
        {
            dfu_ctrl_install_completed(env, status);
        }
    }
    break;
    default:
        break;
    }

}

static void dfu_init_completed_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_init_completed_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_init_completed_ind_t *ind = (dfu_init_completed_ind_t *)data;
    LOG_I("dfu_init_completed_handler %d", ind->is_start);
    /* Not handle not start case. */
    OS_ASSERT(ind->is_start);

    switch (state)
    {
    case DFU_CTRL_NEG:
    {
        env->prog.state = DFU_CTRL_PREPARE_START;
        dfu_ctrl_update_prog_info_ext(env);
    }
    break;
    default:
        break;
    }

}

static void dfu_resume_completed_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_resume_completed_ind_t *ind = (dfu_resume_completed_ind_t *)data;
    /* Not handle not start case. */
    OS_ASSERT(ind->is_start);

    switch (state)
    {
    case DFU_CTRL_NEG:
    {
        env->prog.state = DFU_CTRL_PREPARE_START;
        dfu_ctrl_update_prog_info_ext(env);
    }
    break;
    default:
        break;
    }

}

static uint16_t dfu_ctrl_packet_check_compare(dfu_ctrl_ext_env_t *env, dfu_control_packet_t *packet, uint16_t total_len)
{
    uint16_t status = DFU_ERR_PARAMETER_INVALID;
    do
    {
        dfu_download_progress_ext_t *progress = &env->prog;
        if (packet->dfu_ID != progress->dfu_ID)
        {
            LOG_E("dfu ID error %d, %d", packet->dfu_ID, progress->dfu_ID);
            break;
        }
        if (packet->HW_version != progress->HW_version)
        {
            LOG_E("hw version error %d, %d", packet->HW_version, progress->HW_version);
            break;
        }
        if (packet->SDK_version != progress->SDK_version)
        {
            LOG_E("sdk version error %d, %d", packet->SDK_version, progress->SDK_version);
            break;
        }
        if (packet->FW_version != progress->FW_version)
        {
            LOG_E("fw version error %d, %d", packet->FW_version, progress->FW_version);
            break;
        }

        uint8_t *aes_in;
        uint8_t *aes_out;
        aes_in = rt_malloc(DFU_KEY_SIZE);
        aes_out = rt_malloc(DFU_KEY_SIZE);

        SCB_InvalidateDCache_by_Addr(aes_in, DFU_KEY_SIZE);
        SCB_InvalidateICache_by_Addr(aes_in, DFU_KEY_SIZE);
        SCB_InvalidateDCache_by_Addr(aes_out, DFU_KEY_SIZE);
        SCB_InvalidateICache_by_Addr(aes_out, DFU_KEY_SIZE);

        rt_memcpy(aes_in, env->prog.FW_key, DFU_KEY_SIZE);
        sifli_hw_dec_key(aes_in, aes_out, DFU_KEY_SIZE);
        rt_memcpy(dfu_ext_temp_key, aes_out, DFU_KEY_SIZE);
        rt_free(aes_in);
        rt_free(aes_out);

        if (memcmp(packet->FW_key, dfu_ext_temp_key, sizeof(packet->FW_key)) != 0)
        {
            LOG_HEX("packet FW_key", 16, (uint8_t *)packet->FW_key, DFU_KEY_SIZE);
            LOG_HEX("dfu_ext_temp_key", 16, (uint8_t *)dfu_ext_temp_key, DFU_KEY_SIZE);
            LOG_E("dfu temp key error");
            break;
        }
        dfu_code_image_header_t *header = (dfu_code_image_header_t *)&packet->image_header;
        dfu_dl_image_header_t *dl_hdr = (dfu_dl_image_header_t *)&progress->fw_context.code_img;

        if (header->blk_size != dl_hdr->blk_size)
        {
            LOG_E("block size error %d, %d", header->blk_size, dl_hdr->blk_size);
            break;
        }

        if (header->img_count != dl_hdr->img_count)
        {
            LOG_E("image count error %d, %d", header->img_count, dl_hdr->img_count);
            break;
        }

        uint32_t i;
        for (i = 0; i < header->img_count; i++)
        {
            if (header->img_header[i].img_id != dl_hdr->img_header[i].img_id)
            {
                LOG_E("img id error %d, %d, %d", i, header->img_header[i].img_id, dl_hdr->img_header[i].img_id);
                break;
            }

            if (header->img_header[i].flag != dl_hdr->img_header[i].flag)
            {
                LOG_E("img flag error %d, %d, %d", i, header->img_header[i].flag, dl_hdr->img_header[i].flag);
                break;
            }

            if (memcmp(header->img_header[i].sig, dl_hdr->img_header[i].sig, DFU_SIG_SIZE) != 0)
            {
                LOG_E("img sig error %d", i);
                break;
            }
        }

        if (i != header->img_count)
        {
            LOG_E("img count error %d, %d", i, header->img_count);
            break;
        }

        status = DFU_ERR_NO_ERR;
    }
    while (0);

    return status;
}

static uint16_t dfu_ctrl_packet_code_fw_compare(dfu_ctrl_ext_env_t *env, dfu_control_packet_t *ctrl_packet, uint16_t len)
{
    uint16_t status;
    do
    {
        /* If user callback doesn't register, always reject. */

        if (!env->callback)
        {
            // TODO: local debug
            status = DFU_ERR_USER_REJECT;
            break;
        }

        status = dfu_ctrl_packet_check_compare(env, ctrl_packet, len);
        // TODO: local debug
        //break;


        /* Notify user */
        env->prog.state = DFU_CTRL_NEG;
        dfu_app_start_request_ext_t app_req;
        dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;
        app_req.dfu_id = ctrl_packet->dfu_ID;
        app_req.is_nand_overwrite = 0;
        app_req.event = DFU_APP_START_REQUEST;

        dfu_code_image_header_t *header = (dfu_code_image_header_t *)&ctrl_packet->image_header;
        for (int i = 0; i < header->img_count; i++)
        {
            if (check_current_img_need_overwrite(env->prog.fw_context.code_img.img_header[i].img_id) == 1)
            {
                app_req.is_nand_overwrite = 1;
                break;
            }
        }

        uint16_t len = sizeof(dfu_app_start_request_ext_t);
        if (env->callback)
        {
            ret = env->callback(DFU_APP_START_REQUEST, len, &app_req);
        }

        if (ret == DFU_EVENT_FAILED)
        {
            status = DFU_ERR_USER_REJECT;
            break;
        }

        /* If user choose reponse later, dfu should not ack immediately. */
        if (ret == DFU_EVENT_POSTPONE)
        {
            status = DFU_ERR_POSTPONE;
        }

    }
    while (0);

    return status;
}

/* Check contrl packet whether invalidate. */
static void dfu_resume_request_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    uint8_t status = DFU_ERR_GENERAL_ERR;

    /* 1. Hash and verify. */
    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    data += sizeof(struct image_cfg_hdr);
    len -= sizeof(struct image_cfg_hdr);
    uint8_t *sig = malloc(DFU_SIG_SIZE);
    memcpy(sig, data + len - DFU_SIG_SIZE, DFU_SIG_SIZE);
    uint8_t *packet = (uint8_t *)dfu_dec_verify(NULL, 0, data, data, len - DFU_SIG_SIZE, hdr->hash);
    do
    {
        if (!env->is_init)
        {
            status = DFU_ERR_NOT_READY;
            break;
        }
        if (!packet)
        {
            LOG_W("Ctrl packet parser failed!");
            break;
        }

        /* First check the signature.*/
        if (dfu_ctrl_ctrl_header_sig_verify_ext(packet, len, sig) < 0)
        {
            status = DFU_ERR_CONTROL_PACKET_INVALID;
            break;
        }

        uint8_t dfu_ID = *packet;
        switch (state)
        {
        case DFU_CTRL_IDLE:
        case DFU_CTRL_FORCE_UPDATE:
        {
            if (dfu_ID == DFU_ID_CODE ||
                    dfu_ID == DFU_ID_CODE_MIX)
            {
                /* ignore current download state. */
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_compare(env, ctrl_packet, len);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
            }
            else
            {
                //TODO Support other OTA method, before implment it, just reponse err.
            }
            break;
        }
        default:
        {

            break;
        }
        }


    }
    while (0);

    if (sig)
        free(sig);
    /* Prepare response. */
    if (status != DFU_ERR_POSTPONE)
    {
        dfu_resume_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_RESUME_RESPONSE, dfu_resume_response_t);
        rsp->result = status;
        if (status == DFU_ERR_NO_ERR)
        {
            dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;
            rsp->is_restart = env->prog.FW_state > DFU_CTRL_FW_DOWNLOADING ? 1 : 0;
            if (!rsp->is_restart && env->prog.FW_state == DFU_CTRL_FW_DOWNLOADING)
            {
                if (dl_hdr->curr_img_info.img_state == DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL)
                {
                    dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_IDLE;
                }
                else if (dl_hdr->curr_img_info.img_state == DFU_CTRL_IMG_STATE_DOWNLOADED)
                {
                    /* Find next image. */
                    uint32_t i = 0;
                    for (i = 0; i < dl_hdr->img_count; i++)
                    {
                        if (dl_hdr->img_header[i].img_id == dl_hdr->curr_img_info.img_id)
                            break;
                    }
                    if (i >= dl_hdr->img_count)
                    {
                        env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                        rsp->is_restart = 1; // No img_count is correct so just re-transmission.
                    }
                    else if (i == dl_hdr->img_count - 1)
                    {
                        /* The last img already downloaded just need send current info .*/
                    }
                    else
                    {
                        /* Wait remote device send from beginning. */
                        dl_hdr->curr_img_info.img_id = dl_hdr->img_header[i + 1].img_id;
                        dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_IDLE;
                        dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num = 0;
                    }
                }

                /* In downloading state should not have installing sub-state. */
                OS_ASSERT(dl_hdr->curr_img_info.img_state < DFU_CTRL_IMG_STATE_INSTALLING);

                if (!rsp->is_restart)
                {
                    rsp->curr_img = dl_hdr->curr_img_info.img_id;
                    rsp->curr_packet_num = dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num;
                }
            }
            rsp->is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;
            rsp->num_of_rsp = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp;
        }
        dfu_protocol_packet_send((uint8_t *)rsp);
    }
}

static void dfu_ctrl_request_init_handler_ext(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_ctrl_request_init_handler_ext");
    env->prog.state = DFU_CTRL_IDLE;
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    uint8_t status = DFU_ERR_GENERAL_ERR;
    env->resume_status = 0;

    env->dfu_flash_thread = NULL;
    env->mb_handle = NULL;


#ifdef OTA_NAND_ONLY
    dfu_sec_config_malloc();
#endif

    /* 1. Hash and verify. */
    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    data += sizeof(struct image_cfg_hdr);
    len -= sizeof(struct image_cfg_hdr);
    uint8_t *sig = malloc(DFU_SIG_SIZE);
    memcpy(sig, data + len - DFU_SIG_SIZE, DFU_SIG_SIZE);
    uint8_t *packet = (uint8_t *)dfu_dec_verify(NULL, 0, data, data, len - DFU_SIG_SIZE, hdr->hash);
    do
    {
        if (!env->is_init)
        {
            status = DFU_ERR_NOT_READY;
            break;
        }
        if (!packet)
        {
            LOG_W("Ctrl packet parser failed!");
            break;
        }

        /* First check the signature.*/
        if (dfu_ctrl_ctrl_header_sig_verify_ext(packet, len, sig) < 0)
        {
            // TODO: verify sig
            LOG_E("dfu_ctrl_ctrl_header_sig_verify_ext FAIL!!!!!!!!!!!!!!");
            status = DFU_ERR_CONTROL_PACKET_INVALID;
            break;
        }

        LOG_I("dfu_ctrl_request_init_handler_ext state %d, fw state %d", state, env->prog.FW_state);
        uint8_t dfu_ID = *packet;
        switch (state)
        {
        /* In normal mode. */
        // TODO: local debug
        // case DFU_CTRL_UPDATING:
        case DFU_CTRL_IDLE:
        case DFU_CTRL_UPDATED:
        case DFU_CTRL_FORCE_UPDATE:
        {
            /* OTA mode could have IDLE state if no need to reboot. */
            //if (env->mode != DFU_CTRL_NORMAL_MODE)
            //break;
            if (dfu_ID == DFU_ID_CODE ||
                    dfu_ID == DFU_ID_CODE_MIX)
            {
                if (env->prog.FW_state >= DFU_CTRL_FW_DOWNLOADING)
                {
                    dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                    status = dfu_ctrl_packet_code_fw_compare(env, ctrl_packet, len);
                    LOG_I("dfu_ctrl_request_init_handler_ext status %d", status);
                    dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
                    if (status == DFU_ERR_NO_ERR)
                    {
                        // can resume
                        env->prog.state = DFU_CTRL_NEG;
                        env->resume_status = 1;
                        break;
                    }
                    else
                    {
                        // resume fail, then we check if we can normal start
                        LOG_I("try to resume failed with %d", status);
                    }
                }

                /* ignore current download state. */
                env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_handler(env, ctrl_packet, len);
                LOG_I("dfu_ctrl_request_init_handler_ext handle status %d,", status);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);

                if (status == DFU_ERR_NO_ERR)
                {
                    env->prog.state = DFU_CTRL_NEG;
                }
            }
            else
            {
                //TODO Support other OTA method, before implment it, just reponse err.
                LOG_I("invalid dfu id");
            }
            break;
        }
        case DFU_CTRL_TRAN_START:
        {
            if (env->prog.FW_state >= DFU_CTRL_FW_DOWNLOADING)
            {
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_compare(env, ctrl_packet, len);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
                if (status == DFU_ERR_NO_ERR)
                {
                    // can resume
                    env->resume_status = 1;
                    break;
                }
            }
        }
        default:
        {
            status = DFU_ERR_UNEXPECT_STATE;
            break;
        }
        }


    }
    while (0);

    // set to none as code ota will going
    //env->ota_state.state = OTA_STATE_NONE;
    if (sig)
        free(sig);

    if (env->resume_status == 1)
    {
        dfu_init_response_ext_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE_EXT, dfu_init_response_ext_t);
        rsp->resume_status = 1;
        rsp->result = status;
        rsp->is_boot = 0;
        if (status == DFU_ERR_NO_ERR)
        {
            dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;
            rsp->is_restart = env->prog.FW_state > DFU_CTRL_FW_DOWNLOADING ? 1 : 0;
            if (!rsp->is_restart && env->prog.FW_state == DFU_CTRL_FW_DOWNLOADING)
            {
                if (dl_hdr->curr_img_info.img_state == DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL)
                {
                    LOG_I("DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL");
                    dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_IDLE;
                    rsp->is_restart = 1;
                }
                else if (dl_hdr->curr_img_info.img_state == DFU_CTRL_IMG_STATE_DOWNLOADED)
                {
                    LOG_I("DFU_CTRL_IMG_STATE_DOWNLOADED");
                    uint8_t last_id = 0;
                    for (int j = 0; j < dl_hdr->img_count; j++)
                    {
                        if (dl_hdr->img_header[j].img_id > last_id)
                        {
                            last_id = dl_hdr->img_header[j].img_id;
                        }
                    }
                    /* Find next image. */
                    uint32_t i = 0;
                    for (i = 0; i < dl_hdr->img_count; i++)
                    {
                        if (dl_hdr->img_header[i].img_id == dl_hdr->curr_img_info.img_id)
                            break;
                    }
                    if (i >= dl_hdr->img_count)
                    {
                        env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                        rsp->is_restart = 1; // No img_count is correct so just re-transmission.
                    }
                    else if (dl_hdr->curr_img_info.img_id == last_id)
                    {
                        /* The last img already downloaded just need send current info .*/
                    }
                    else
                    {
                        uint8_t next_id = DFU_IMG_ID_MAX + 1;
                        for (int j = 0; j < dl_hdr->img_count; j++)
                        {
                            if (dl_hdr->img_header[j].img_id > dl_hdr->curr_img_info.img_id && dl_hdr->img_header[j].img_id < next_id)
                            {
                                next_id = dl_hdr->img_header[j].img_id;
                            }
                        }
                        if (next_id == DFU_IMG_ID_MAX + 1)
                        {
                            LOG_I("next id fail to find");
                            env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                            rsp->is_restart = 1;
                        }
                        else
                        {
                            LOG_I("next id %d", next_id);
                            /* Wait remote device send from beginning. */
                            dl_hdr->curr_img_info.img_id = next_id;
                            dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_IDLE;
                            dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num = 0;
                        }
                    }
                }

                /* In downloading state should not have installing sub-state. */
                OS_ASSERT(dl_hdr->curr_img_info.img_state < DFU_CTRL_IMG_STATE_INSTALLING);

                if (!rsp->is_restart)
                {
                    rsp->curr_img = dl_hdr->curr_img_info.img_id;
                    rsp->curr_packet_num = dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num;
                }
            }

            rsp->num_of_rsp = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp;

            if (env->is_mount == 1 && check_current_img_need_overwrite(dl_hdr->curr_img_info.img_id))
            {
                rsp->curr_img = dl_hdr->curr_img_info.img_id;
                rsp->curr_packet_num = 0;
                env->is_mount = 0;

                dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_IDLE;
                dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num = 0;
            }
        }
        dfu_protocol_packet_send((uint8_t *)rsp);
        return;
    }
    LOG_I("dfu_ctrl_request_init_handler %d", status);

    /* Prepare response. */
    if (status != DFU_ERR_POSTPONE)
    {
        dfu_init_response_ext_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE_EXT, dfu_init_response_ext_t);
        rsp->result = status;
        rsp->resume_status = 0;

        dfu_protocol_packet_send((uint8_t *)rsp);

        if (status == DFU_ERR_OTA_ONGOING)
        {
            LOG_I("clear state");
            env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
        }
        if (status != DFU_ERR_NO_ERR)
        {
            dfu_ctrl_error_handle(env);
        }
    }

}

static void dfu_init_completed_handler_ext(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_init_completed_handler_ext");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_init_completed_ind_t *ind = (dfu_init_completed_ind_t *)data;
    LOG_I("dfu_init_completed_handler_ext %d", ind->is_start);
    /* Not handle not start case. */
    //OS_ASSERT(ind->is_start);

    if (env->resume_status == 1 && ind->is_start == 1)
    {
        LOG_I("use resume");
    }
    else
    {
        env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
        memset(env->prog.image_download_state, 0, DFU_IMG_ID_MAX);
    }

    switch (state)
    {
    case DFU_CTRL_NEG:
    {
        env->prog.state = DFU_CTRL_PREPARE_START;
        dfu_ctrl_update_prog_info_ext(env);
    }
    break;
    default:
        LOG_I("dfu_init_completed_handler_ext %d", state);
        env->prog.state = DFU_CTRL_PREPARE_START;
        dfu_ctrl_update_prog_info_ext(env);
        break;
    }

}


static uint8_t dfu_flie_compare_check(dfu_ctrl_ext_env_t *env, dfu_image_file_total_start_t *remote_data)
{
    uint8_t check_status = 0;
    if (env->prog.res_file_total_num != remote_data->file_count)
    {
        LOG_I(
            "dfu_flie_compare_check1 %d, %d", env->prog.res_file_total_num, remote_data->file_count);
        return check_status;
    }
    if (env->prog.res_file_total_len != remote_data->file_total_len)
    {
        LOG_I(
            "dfu_flie_compare_check2 %d, %d", env->prog.res_file_total_len, remote_data->file_total_len);
        return check_status;
    }
    // TODO: VERSION CHECK
    //if (env-<)
    check_status = 1;
    return check_status;
}

static uint8_t dfu_file_fw_handler(dfu_ctrl_ext_env_t *env, dfu_image_file_total_start_t *s_data)
{
    uint16_t status = 0;
    do
    {
        if (!env->callback)
        {
            // TODO: local debug
            status = DFU_ERR_USER_REJECT;
            break;
        }


        dfu_file_init_ind_t app_req;
        dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;

        app_req.event = DFU_APP_RES_INIT_REQUEST;
        app_req.file_count = env->prog.res_file_total_num;
        app_req.file_size = env->prog.res_file_total_len;
        app_req.version_len = s_data->version_len;
        app_req.version = malloc(app_req.version_len);
        app_req.resume_status = env->resume_status;
        app_req.resume_count = env->prog.res_file_dowload_count;
        memcpy(app_req.version, s_data->version, s_data->version_len);
        uint8_t len = sizeof(dfu_file_init_ind_t) + app_req.version_len;
        // TODO: local debug
        ret = env->callback(DFU_APP_RES_INIT_REQUEST, len, &app_req);
        free(app_req.version);

        if (ret == DFU_EVENT_FAILED)
        {
            status = DFU_ERR_USER_REJECT;
            return status;
        }

        if (ret == DFU_EVENT_POSTPONE)
        {
            status = DFU_ERR_POSTPONE;
        }
    }
    while (0);

    return status;
}

static void dfu_file_init_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    uint16_t res_state = env->prog.res_state;
    dfu_image_file_total_start_t *s_data = (dfu_image_file_total_start_t *)data;
    LOG_I("dfu_file_init_handler %d %d", state, res_state);

    uint32_t remote_version;
    memcpy(&remote_version, s_data->version, s_data->version_len);
    LOG_I("dfu_file_init_handler remote version %d", remote_version);
    env->remote_version = remote_version;
    // remote is unaligned
    uint32_t file_count;
    uint32_t file_total_len;

    memcpy(&file_count, data, sizeof(uint32_t));
    memcpy(&file_total_len, data + sizeof(uint32_t), sizeof(uint32_t));

    uint16_t result = DFU_ERR_GENERAL_ERR;
    env->resume_status = 0;
    env->is_abort = 0;

    if (env->is_sync_timer_on)
    {
        LOG_I("clear at init");
        dfu_link_sync_end();
    }

    do
    {
        switch (state)
        {
        case DFU_CTRL_NEG:
        case DFU_CTRL_PREPARE_START:
        case DFU_CTRL_INSTALL:
            result = DFU_ERR_OTA_ONGOING;
            break;
        case DFU_CTRL_TRAN_START:
        {
            // send error to ui, then send init
            dfu_file_error_ind_t ind_on;
            if (env->callback)
            {
                ind_on.event = DFU_APP_ERROR_DISCONNECT_IND;
                uint16_t len = sizeof(dfu_file_error_ind_t);
                env->callback(DFU_APP_ERROR_DISCONNECT_IND, len, &ind_on);
            }
            break;
        }
        }

        if (result != DFU_ERR_GENERAL_ERR)
        {
            break;
        }

        switch (res_state)
        {
        case DFU_CTRL_UPDATING:
        case DFU_CTRL_UPDATED:
        case DFU_CTRL_IDLE:
            env->prog.res_state = DFU_CTRL_NEG;
            result = DFU_ERR_NO_ERR;
            LOG_I("dfu_file_init_handler remote count %d", s_data->file_count);
            env->prog.res_file_total_num = file_count;
            env->prog.res_file_total_len = file_total_len;

            if (file_total_len == 0 && file_count == 0)
            {
                env->prog.res_state = DFU_CTRL_UPDATED;
            }

            result = dfu_file_fw_handler(env, s_data);
            //TDOO: check total len
            break;
        case DFU_CTRL_NEG:
        case DFU_CTRL_PREPARE_START:
        case DFU_CTRL_TRAN_START:
        case DFU_CTRL_INSTALL:
            env->prog.res_state = DFU_CTRL_IDLE;
            result = DFU_ERR_OTA_ONGOING;
            break;
        //case DFU_CTRL_UPDATING:
        //    result = DFU_ERR_UPDATING;
        //    break;
        default:
            result = DFU_ERR_GENERAL_ERR;
        }
    }
    while (0);

    if (result != DFU_ERR_POSTPONE)
    {
        dfu_image_file_init_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_INIT_RESPONSE, dfu_image_file_init_response_t);

        rsp->result = result;
        rsp->ver = OTA_CODE_VERSION;
        rsp->resume_status = env->resume_status;
        if (rsp->resume_status == 1)
        {
            rsp->file_count = env->prog.res_file_dowload_count;
        }
        else
        {
            rsp->file_count = 0;
        }
        LOG_I("dfu_file_init_handler remote RSP %d, %d, %d", result, rsp->resume_status, rsp->file_count);
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (rsp->result == DFU_ERR_NO_ERR)
        {
            return;
        }

        if (result == DFU_ERR_OTA_ONGOING)
        {
            dfu_file_error_ind_t ind;
            if (env->callback)
            {
                ind.event = DFU_APP_ERROR_DISCONNECT_IND;
                uint16_t len = sizeof(dfu_file_error_ind_t);
                env->callback(DFU_APP_ERROR_DISCONNECT_IND, len, &ind);
            }
        }

        if (rsp->result != DFU_ERR_UPDATING)
        {
            dfu_ctrl_error_handle(env);
        }
    }
}

void dfu_file_init_response(dfu_file_init_response_resume_info_t *info)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_image_file_init_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_INIT_RESPONSE, dfu_image_file_init_response_t);
    rsp->result = info->result;
    uint32_t resume_count = 0;
    rsp->fs_block = info->fs_block;
    rsp->ver = OTA_CODE_VERSION;

    if (env->resume_status == 1)
    {
        if (info->resume_command == DFU_APP_RESUME_RESTART)
        {
            env->resume_status = 0;
        }
        else if (info->resume_command == DFU_APP_RESUME_USE_BLE)
        {
            env->resume_status = 0;
        }
        else if (info->resume_command == DFU_APP_RESUME_USE_APP)
        {
            resume_count = info->resume_count;
            env->prog.res_file_dowload_count = info->resume_count;
            env->prog.total_receive_len = info->resume_length;
        }
    }

    // response with DFU_ERR_UPDATING, phone should skip res update
    if (rsp->result == DFU_ERR_UPDATING)
    {
        env->prog.res_state = DFU_CTRL_UPDATING;
    }
    else
    {

    }

    rsp->resume_status = env->resume_status;
    rsp->file_count = resume_count;
    dfu_protocol_packet_send((uint8_t *)rsp);

    if (rsp->result == DFU_ERR_NO_ERR)
    {
        return;
    }

    if (rsp->result != DFU_ERR_UPDATING)
    {
        dfu_ctrl_error_handle(env);
    }
}

static void dfu_file_init_completed_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    //LOG_I("dfu_file_init_completed_handler");
    DFU_ERR_CHECK(env && data);


    uint16_t res_state = env->prog.res_state;
    //dfu_image_file_init_completed_t *s_date = (dfu_image_file_init_completed_t *)data;
    dfu_image_file_init_completed_t *s_date = rt_malloc(sizeof(dfu_image_file_init_completed_t));
    s_date->resume = *data;
    s_date->remote_block = 0;

    if (len >= 5)
    {
        rt_memcpy((uint8_t *)&s_date->remote_block, data + 1, sizeof(uint32_t));
    }

    LOG_I("dfu_file_init_completed_handler %d, %d, %d, %d", res_state, s_date->resume, env->resume_status, s_date->remote_block);
    uint8_t resume_result;

    switch (res_state)
    {
    case DFU_CTRL_NEG:
        env->prog.res_state = DFU_CTRL_PREPARE_START;
        if (s_date->resume && env->resume_status)
        {
            // resume
            resume_result = 1;
        }
        else
        {
            env->prog.res_file_dowload_count = 0;
            env->prog.total_receive_len = 0;
            resume_result = 0;
        }

        dfu_file_init_completed_t ind;
        if (env->callback)
        {
            ind.event = DFU_APP_RES_INIT_COMPLETED;
            ind.resume_result = resume_result;
            ind.remote_block = s_date->remote_block;
            uint16_t len = sizeof(dfu_file_init_completed_t);
            env->callback(DFU_APP_RES_INIT_COMPLETED, len, &ind);
        }
        dfu_ctrl_update_prog_info_ext(env);
        break;
    default:
        break;
    }

    rt_free(s_date);
}

static void dfu_file_start_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_file_start_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t result = DFU_ERR_GENERAL_ERR;
    uint16_t res_state = env->prog.res_state;
    dfu_image_file_start_t *file_start = (dfu_image_file_start_t *)data;

    switch (res_state)
    {
    case DFU_CTRL_PREPARE_START:
        env->prog.res_state = DFU_CTRL_TRAN_START;
        result = DFU_ERR_NO_ERR;
        break;
    case DFU_CTRL_TRAN_START:
        if (file_start->total_file_count != env->prog.res_file_dowload_count + 1)
        {
            LOG_I(
                "dfu_file_start_handler %d, %d", file_start->total_file_count, env->prog.res_file_dowload_count);
            break;
        }

        env->prog.res_state = DFU_CTRL_TRAN_START;
        result = DFU_ERR_NO_ERR;
        break;
    default:
        break;
    }

    dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;
    // TODO: process name and path
    if (result == DFU_ERR_NO_ERR)
    {
        LOG_I("dfu_file_start_handler num of rsp %d", file_start->num_of_rsp);
        env->res_prog.single_file_packet_download_count = 0;
        env->res_prog.single_file_packet_download_len = 0;
        env->res_prog.num_of_rsp = file_start->num_of_rsp;
        env->res_prog.single_file_packet_num = file_start->file_pkt_num;
        env->res_prog.single_file_len = file_start->file_length;

        dfu_file_start_ind_t ind;
        if (env->callback)
        {
            ind.event = DFU_APP_RES_FILE_START_IND;
            ind.file_len = env->res_prog.single_file_len;
            ind.file_name_len = file_start->file_name_len;
            ind.file_name = malloc(file_start->file_name_len);
            memcpy(ind.file_name, file_start->file_name, file_start->file_name_len);
            uint16_t len = sizeof(dfu_file_start_ind_t) + file_start->file_name_len;
            ret = env->callback(DFU_APP_RES_FILE_START_IND, len, &ind);
            free(ind.file_name);
        }
    }

    if (ret != DFU_EVENT_POSTPONE)
    {
        LOG_I("dfu_file_start_handler %d", result);
        dfu_image_file_start_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_START_RESPONSE, dfu_image_file_start_response_t);

        rsp->result = result;
        dfu_protocol_packet_send((uint8_t *)rsp);
        if (result == DFU_ERR_NO_ERR)
        {
            dfu_link_sync_start(DFU_SYNC_TYPE_FILE);
        }

        if (result != DFU_ERR_NO_ERR)
        {
            dfu_ctrl_error_handle(env);
        }
    }
}

void dfu_file_start_response(uint8_t result)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_image_file_start_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_START_RESPONSE, dfu_image_file_start_response_t);

    rsp->result = result;
    dfu_protocol_packet_send((uint8_t *)rsp);
    if (result == DFU_ERR_NO_ERR)
    {
        ble_dfu_request_connection_priority();
        dfu_link_sync_start(DFU_SYNC_TYPE_FILE);
    }

    if (result != DFU_ERR_NO_ERR)
    {
        dfu_ctrl_error_handle(env);
    }
}


static void dfu_file_packet_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    //LOG_I("dfu_file_file_packet_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t result = DFU_ERR_GENERAL_ERR;
    uint16_t res_state = env->prog.res_state;
    dfu_image_file_packet_t *file_packet = (dfu_image_file_packet_t *)data;
    dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;

    switch (res_state)
    {
    case DFU_CTRL_TRAN_START:
    {
        if (env->res_prog.single_file_packet_download_count + 1 != file_packet->packet_index)
        {
            LOG_I("dfu_file_file_packet send index error");
            result = DFU_ERR_INDEX_ERROR;
            break;
        }

        env->res_prog.single_file_packet_download_count++;
        env->res_prog.single_file_packet_download_len += file_packet->packet_length;

        // TODO: WRITE TO FLASH/FILE
        dfu_file_data_ind_t ind;
        if (env->callback)
        {
            ind.event = DFU_APP_RES_FILE_DATA_IND;
            ind.data_len = file_packet->packet_length;
            ind.data = malloc(file_packet->packet_length);
            uint16_t len = sizeof(dfu_file_data_ind_t) + ind.data_len;
            memcpy(ind.data, file_packet->file_data, file_packet->packet_length);
            ret = env->callback(DFU_APP_RES_FILE_DATA_IND, len, &ind);
            free(ind.data);
        }

        result = DFU_ERR_NO_ERR;
        break;
    }
    default:
        LOG_I("dfu_file_file_packet state error %d", res_state);
        break;
    }


    dfu_image_file_packet_response_t *rsp;
    if (ret != DFU_EVENT_POSTPONE)
    {
        if (result == DFU_ERR_NO_ERR)
        {
            if (env->res_prog.single_file_packet_download_count == env->res_prog.single_file_packet_num)
            {
                // last packet
                dfu_link_sync_end();
                rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
                rsp->result = result;
                rsp->current_file_index = env->res_prog.single_file_packet_download_count;
                rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
                LOG_I("dfu_file_file_packet send rsp1 %d", rsp->result);
                dfu_protocol_packet_send((uint8_t *)rsp);
                return;
            }

            if (env->res_prog.num_of_rsp == 0)
                return;

            if (env->res_prog.single_file_packet_download_count % env->res_prog.num_of_rsp == 0)
            {
                dfu_link_sync_end();
                if (result == DFU_ERR_NO_ERR)
                {
                    dfu_link_sync_start(DFU_SYNC_TYPE_FILE);
                }

                LOG_I("dfu_file_file_packet send rsp %d", result);
                rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
                rsp->result = result;
                rsp->current_file_index = env->res_prog.single_file_packet_download_count;
                rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
                dfu_protocol_packet_send((uint8_t *)rsp);
                return;
            }

            return;
        }
    }

    if (ret == DFU_EVENT_POSTPONE && result == DFU_ERR_NO_ERR)
    {
        return;
    }


    LOG_I("dfu_file_file_packet send rsp2 %d", result);
    rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
    rsp->result = result;
    rsp->current_file_index = env->res_prog.single_file_packet_download_count;
    rsp->new_num_of_rsp = env->res_prog.num_of_rsp;



    if (result == DFU_ERR_INDEX_ERROR)
    {
        // TODO: may change num_of_rsp;
        dfu_protocol_packet_send((uint8_t *)rsp);
        return;
    }
    else
    {
        // some error that can not continue

        dfu_protocol_packet_send((uint8_t *)rsp);
        dfu_ctrl_error_handle(env);
    }
}


void dfu_file_packet_response(uint8_t result, uint16_t is_last)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    dfu_image_file_packet_response_t *rsp;
    if (result == DFU_ERR_NO_ERR)
    {
        if (env->res_prog.single_file_packet_download_count == env->res_prog.single_file_packet_num && is_last)
        {
            // last packet
            dfu_link_sync_end();
            LOG_I("dfu_file_packet_response 1");
            rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
            rsp->result = result;
            rsp->current_file_index = env->res_prog.single_file_packet_download_count;
            rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
            dfu_protocol_packet_send((uint8_t *)rsp);
            return;
        }

        if (env->res_prog.num_of_rsp == 0)
        {
            LOG_I("dfu_file_packet_response 2");
            return;
        }

        if (env->res_prog.single_file_packet_download_count % env->res_prog.num_of_rsp == 0)
        {
            LOG_I("dfu_file_packet_response 3");
            dfu_link_sync_end();
            if (result == DFU_ERR_NO_ERR)
            {
                dfu_link_sync_start(DFU_SYNC_TYPE_FILE);
            }
            rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
            rsp->result = result;
            rsp->current_file_index = env->res_prog.single_file_packet_download_count;
            rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
            dfu_protocol_packet_send((uint8_t *)rsp);
            return;
        }
    }
    else
    {
        LOG_W("dfu_file_packet_response %d", result);
        dfu_link_sync_end();

        rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
        rsp->result = result;
        rsp->current_file_index = env->res_prog.single_file_packet_download_count;
        rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
        dfu_protocol_packet_send((uint8_t *)rsp);

        dfu_ctrl_error_handle(env);
    }

}

static void dfu_file_end_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_file_end_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t result = DFU_ERR_GENERAL_ERR;
    uint16_t res_state = env->prog.res_state;
    dfu_image_file_end_t *file_end = (dfu_image_file_end_t *)data;
    dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;

    switch (res_state)
    {
    case DFU_CTRL_TRAN_START:
    {
        if (env->res_prog.single_file_packet_download_count != env->res_prog.single_file_packet_num)
        {
            LOG_I("dfu_file_end_handler1 %d, %d", env->res_prog.single_file_packet_download_count, env->res_prog.single_file_packet_num);
            break;
        }

        if (env->res_prog.single_file_packet_download_len != env->res_prog.single_file_len)
        {
            LOG_I("dfu_file_end_handler2 %d, %d", env->res_prog.single_file_packet_download_len, env->res_prog.single_file_len);
            break;
        }

        if (file_end->count == env->prog.res_file_dowload_count + 1)
        {
            // TODO: may add other check method
            result = DFU_ERR_NO_ERR;

            dfu_file_end_ind_t ind;
            if (env->callback)
            {
                ind.event = DFU_APP_RES_FILE_END_IND;
                len = sizeof(dfu_file_end_ind_t);
                ret = env->callback(DFU_APP_RES_FILE_END_IND, len, &ind);
            }
        }
        else
        {
            LOG_I("dfu_file_end_handler3 %d, %d", file_end->count, env->prog.res_file_dowload_count);
        }
        break;
    }
    }

    if (ret != DFU_EVENT_POSTPONE)
    {
        if (result == DFU_ERR_NO_ERR)
        {
            env->prog.res_file_dowload_count++;
            env->prog.total_receive_len += env->res_prog.single_file_len;
            dfu_ctrl_update_prog_info_ext(env);
        }

        dfu_image_file_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_END_RESPONSE, dfu_image_file_end_response_t);
        rsp->result = result;
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (result != DFU_ERR_NO_ERR)
        {
            dfu_ctrl_error_handle(env);
        }
    }
}

void dfu_file_end_response(uint8_t result)

{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();

    if (result == DFU_ERR_NO_ERR)
    {
        env->prog.res_file_dowload_count++;
        env->prog.total_receive_len += env->res_prog.single_file_len;
        // dfu_ctrl_update_prog_info_ext(env);
    }

    dfu_image_file_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_END_RESPONSE, dfu_image_file_end_response_t);
    rsp->result = result;
    dfu_protocol_packet_send((uint8_t *)rsp);

    if (result != DFU_ERR_NO_ERR)
    {
        dfu_ctrl_error_handle(env);
    }
}

static void dfu_file_total_end_handler(dfu_ctrl_ext_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_file_total_end_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t result = DFU_ERR_GENERAL_ERR;
    uint16_t res_state = env->prog.res_state;
    dfu_image_file_total_end_t *total_end = (dfu_image_file_total_end_t *)data;
    LOG_I("dfu_file_total_end_handler %d", total_end->hcpu_upgrade);
    dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;

    switch (res_state)
    {
    case DFU_CTRL_TRAN_START:
        env->prog.res_state = DFU_CTRL_INSTALL;

        if (env->prog.total_receive_len != env->prog.res_file_total_len)
        {
            LOG_E("dfu_file_total_end_handler len %d, %d", env->prog.total_receive_len, env->prog.res_file_total_len);
            break;
        }
        if (env->prog.res_file_dowload_count != env->prog.res_file_total_num)
        {
            LOG_E("dfu_file_total_end_handler count %d, %d",  env->prog.res_file_dowload_count, env->prog.res_file_total_num);
            break;
        }



        dfu_ctrl_update_prog_info_ext(env);
        dfu_file_total_end_ind_t ind;
        // TODO: do install
        if (env->callback)
        {
            ind.event = DFU_APP_RES_FILE_TOTAL_END_IND;
            ind.hcpu_upgrade = total_end->hcpu_upgrade;
            uint16_t len = sizeof(dfu_file_total_end_ind_t);
            ret = env->callback(DFU_APP_RES_FILE_TOTAL_END_IND, len, &ind);
        }

        if (total_end->hcpu_upgrade == 0)
        {
            env->prog.state = DFU_CTRL_UPDATING;
        }

        result = DFU_ERR_NO_ERR;
        break;
    }

    if (ret != DFU_EVENT_POSTPONE)
    {
        env->prog.res_state = DFU_CTRL_UPDATING;
        dfu_ctrl_update_prog_info_ext(env);
        dfu_image_file_total_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_TOTAL_END_RESPONSE, dfu_image_file_total_end_response_t);
        rsp->result = result;
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (result != DFU_ERR_NO_ERR)
        {
            dfu_ctrl_error_handle(env);
        }
    }
}

void dfu_file_total_end_reponse(uint8_t result)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (result == DFU_ERR_NO_ERR)
    {
        env->prog.res_state = DFU_CTRL_UPDATING;
    }
    else
    {
        env->prog.state = DFU_CTRL_IDLE;
        env->prog.res_state = DFU_CTRL_IDLE;

    }

    dfu_ctrl_update_prog_info_ext(env);
    dfu_image_file_total_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_TOTAL_END_RESPONSE, dfu_image_file_total_end_response_t);
    rsp->result = result;
    dfu_protocol_packet_send((uint8_t *)rsp);

    if (result != DFU_ERR_NO_ERR)
    {
        dfu_ctrl_error_handle(env);
    }
}

void dfu_protocol_abort_command(uint16_t reason)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    LOG_I("dfu_protocol_abort_command %d", reason);
    dfu_link_sync_end();
    dfu_abort_command_t *cmd = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_ABORT_COMMAND, dfu_abort_command_t);
    cmd->reason = reason;
    dfu_protocol_packet_send((uint8_t *)cmd);

    dfu_ctrl_error_handle(env);
    env->is_abort = 1;
}

void dfu_fs_mount_status_set()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->is_mount = 1;
}

void dfu_protocol_packet_handler_ext(dfu_tran_protocol_t *msg, uint16_t length)
{
    DFU_ERR_CHECK(msg);
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();

    uint8_t is_force = 0;
    env->current_command = msg->message_id;

    switch (msg->message_id)
    {
    case DFU_FORCE_INIT_REQUEST:
        is_force = 1;
    case DFU_INIT_REQUEST:
    {
        dfu_ctrl_request_init_handler(env, msg->data, msg->length, is_force);
    }
    break;
    case DFU_INIT_COMPLETED_IND:
    {
        dfu_init_completed_handler(env, msg->data, msg->length);
    }
    break;
    case DFU_RESUME_REQUEST:
    {
        dfu_resume_request_handler(env, msg->data, msg->length);
    }
    break;
    case DFU_RESUME_COMPLETED_IND:
    {
        dfu_resume_completed_handler(env, msg->data, msg->length);
    }
    break;
    case DFU_IMAGE_SEND_START:
    {
        dfu_img_send_start_handler(env, msg->data, msg->length);
    }
    break;
    case DFU_IMAGE_SEND_PACKET:
    {
        dfu_img_send_packet_handler(env, msg->data, msg->length);
    }
    break;
    case DFU_IMAGE_SEND_END:
    {
        dfu_img_send_end_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_TRANSMISSION_END:
    {
        dfu_img_tramission_end_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_CONNECTION_PRIORITY_CHECK:
    {
        break;
    }
    case DFU_RETRANSMISSION_REQUEST:
    {
        //dfu_img_retransmission_request_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_READ_VERSION_REQUEST:
    {
        //dfu_read_version_request_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_COMMAND_POWER_OFF:
    {
        //dfu_power_off_command_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_FILE_INIT_REQUEST:
    {
        dfu_file_init_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_FILE_INIT_COMPLETED:
    {
        dfu_file_init_completed_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_FILE_START:
    {
        dfu_file_start_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_FILE_PACKET:
    {
        dfu_file_packet_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_FILE_END:
    {
        dfu_file_end_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_FILE_TOTAL_END:
    {
        dfu_file_total_end_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_INIT_REQUEST_EXT:
    {
        dfu_ctrl_request_init_handler_ext(env, msg->data, msg->length);
        break;
    }
    case DFU_INIT_COMPLETED_IND_EXT:
    {
        dfu_init_completed_handler_ext(env, msg->data, msg->length);
        break;
    }
    case DFU_LINK_LOSE_CHECK_RSP:
    {
        dfu_link_lose_check_rsp(env, msg->data, msg->length);
        break;
    }
    default:
        break;
    }


}

void dfu_serial_transport_error_handle(uint8_t error)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    LOG_I("dfu_serial_transport_error_handle %d", env->current_command);

    switch (env->current_command)
    {
    case DFU_IMAGE_SEND_PACKET:
    {
        /*
        dfu_image_send_packet_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_PACKET_RESPONSE, dfu_image_send_packet_response_t);
        rsp->result = DFU_ERR_GENERAL_ERR;
        dfu_protocol_packet_send((uint8_t *)rsp);
        */

        dfu_link_lose_check_req(env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num, env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp);
        break;
    }

    case DFU_IMAGE_FILE_PACKET:
    {
        /*
        dfu_image_file_packet_response_t *res_rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_FILE_PACKET_RESPONSE, dfu_image_file_packet_response_t);
        res_rsp->result = DFU_ERR_INDEX_ERROR;
        res_rsp->current_file_index = env->res_prog.single_file_packet_download_count;
        res_rsp->new_num_of_rsp = env->res_prog.num_of_rsp;
        dfu_protocol_packet_send((uint8_t *)res_rsp);
        */

        dfu_link_lose_check_req(env->res_prog.single_file_packet_download_count, env->res_prog.num_of_rsp);
        break;
    }
    default:
    {
        dfu_ctrl_error_handle(env);
    }

        // TODO: handle other step's error later
    }

}

void dfu_protocol_close_handler(void)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    // for less print
    if (env->is_sync_timer_on)
    {
        dfu_link_sync_end();
    }
    dfu_ctrl_error_handle(env);
}


uint8_t dfu_flash_addr_get(uint8_t img_id, dfu_flash_info_t *info)
{
#ifdef OTA_SECTION_CHANGE
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    if (env->prog.download_flash_addr[img_id] == 0)
    {
        info->addr = 0;
        info->size = 0;
        return DFU_ERR_GENERAL_ERR;
    }
    info->addr = env->prog.download_flash_addr[img_id];
    info->size = env->prog.download_flash_size[img_id];
    return DFU_ERR_NO_ERR;
#else
    return DFU_ERR_GENERAL_ERR;
#endif
}

void dfu_register_ext(dfu_callback_ext callback)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    env->callback = callback;
}

void dfu_register(dfu_callback callback)
{
    // for build
    return;
}

static void dfu_ctrl_packet_postpone_handler(dfu_ctrl_ext_env_t *env, uint16_t status)
{
    if (status == DFU_ERR_USER_REJECT)
        memset(&env->prog, 0, sizeof(dfu_download_progress_ext_t));
}

void dfu_respond_start_request(dfu_event_ack_t result)
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();

    /* Not allowed postpone in this API. */
    OS_ASSERT(result != DFU_EVENT_POSTPONE);

    uint16_t status = DFU_ERR_USER_REJECT;
    if (result == DFU_EVENT_SUCCESSED)
    {
        status = DFU_ERR_NO_ERR;
    }

    /* Postpone handle*/
    if (env->prog.dfu_ID == DFU_ID_CODE ||
            env->prog.dfu_ID == DFU_ID_CODE_MIX ||
            env->prog.dfu_ID == DFU_ID_OTA_MANAGER)
        dfu_ctrl_packet_postpone_handler(env, status);


    dfu_init_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE, dfu_init_response_t);
    rsp->result = status;
    rsp->is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;

    LOG_W("dfu_respond_start_request status %d, is reboot %d", status, rsp->is_boot);
    if (status == DFU_ERR_NO_ERR && rsp->is_boot == 1)
    {
        dfu_set_reboot_after_disconnect();
    }
    dfu_protocol_packet_send((uint8_t *)rsp);
}

uint32_t dfu_get_hcpu_download_addr()
{
    dfu_ctrl_ext_env_t *env = dfu_ctrl_ext_get_env();
    return env->back_up_hcpu;
}


static void dfu_cmd(uint8_t argc, char **argv)
{
    char *value = NULL;

    if (argc > 1)
    {
        if (strcmp(argv[1], "reset") == 0)
        {
            fdb_kv_set_default(p_dfu_ext_db);
        }
        else if (strcmp(argv[1], "ver") == 0)
        {
            LOG_I("XXXXXXXXXXXXXXXX");
        }
    }
}
MSH_CMD_EXPORT(dfu_cmd, DFU command);

#endif /* OTA_56X_NAND */
