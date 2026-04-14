/*
 * SPDX-FileCopyrightText: 2021-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include <rthw.h>
#include "board.h"
#include "rtconfig.h"

#include "bf0_hal_rtc.h"
#include "bf0_hal.h"


#ifdef OTA_55X
#include "dfu.h"
#include "os_adaptor.h"

#include "dfu_protocol.h"
#include "dfu_service.h"
#include "dfu_internal.h"


#include "flashdb.h"
#ifdef FDB_USING_FILE_MODE
    #include "dfs_posix.h"
#endif /* FDB_USING_FILE_MODE */

#ifdef PKG_USING_WEBCLIENT
    #include "lwip/api.h"
    #include "lwip/dns.h"
    #include <webclient.h>
#endif


#define LOG_TAG "DFUCTRL"
#include "log.h"


DFU_NON_RET_SECT_BEGIN
//static uint8_t dfu_temp[DFU_MAX_BLK_SIZE];
static uint8_t dfu_temp_key[DFU_KEY_SIZE];
static struct fdb_kvdb g_dfu_db;
static fdb_kvdb_t p_dfu_db = &g_dfu_db;
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    static uint32_t g_dfu_db_cache[256];
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */
DFU_NON_RET_SECT_END

OS_SEM_DECLAR(g_dfu_ctrl_sem);


static rt_thread_t ble_dfu_package_flash_thread_start();

static dfu_ctrl_env_t g_dfu_ctrl_env;


static void dfu_ctrl_init_env(void)
{
    memset((void *)&g_dfu_ctrl_env, 0, sizeof(g_dfu_ctrl_env));
}

static dfu_ctrl_env_t *dfu_ctrl_get_env(void)
{
    return &g_dfu_ctrl_env;
}

void dfu_ctrl_update_prog_info(dfu_ctrl_env_t *env)
{
#ifdef PKG_USING_FLASHDB
    struct fdb_blob  blob;
    fdb_err_t err;
    err = fdb_kv_set_blob(p_dfu_db, DFU_DOWNLOAD_ENV, fdb_blob_make(&blob, &env->prog, sizeof(dfu_download_progress_t)));
    OS_ASSERT(err == FDB_NO_ERR);
#else
#error "FlashDB shall be enabled"
#endif
}

static const uint8_t g_dfu_p_default[] = {0x0};

static struct fdb_default_kv_node default_dfu_kv_set[] =
{
    {DFU_DOWNLOAD_ENV, (void *)g_dfu_p_default, sizeof(g_dfu_p_default)},
};

#ifdef FDB_USING_KVDB
static fdb_err_t dfu_db_init(void)
{
    //HAL_sw_breakpoint();
    struct fdb_default_kv default_kv;
    p_dfu_db = &g_dfu_db;
    default_kv.kvs = default_dfu_kv_set;
    default_kv.num = sizeof(default_dfu_kv_set) / sizeof(default_dfu_kv_set[0]);
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    default_kv.kv_cache_pool = g_dfu_db_cache;
    default_kv.kv_cache_pool_size = sizeof(g_dfu_db_cache);
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */

    memset(p_dfu_db, 0, sizeof(*p_dfu_db));

    const char *path = DFU_DB_PARTIAL;
#ifdef FDB_USING_FILE_MODE
#include "fal.h"
    const struct fal_partition *fal = fal_partition_find(DFU_DB_PARTIAL);
    RT_ASSERT(fal);
#ifdef SOLUTION_WATCH
    path = fal->path_name;
#endif
    int sec_size = PKG_FLASHDB_ERASE_GRAN;
    int max_size = (int)fal->len;
    bool file_mode = true;
    rt_kprintf("dfu_db_init: sector_size %d size %d\n", sec_size, max_size);
    fdb_kvdb_control(p_dfu_db, FDB_KVDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
    fdb_kvdb_control(p_dfu_db, FDB_KVDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
    fdb_kvdb_control(p_dfu_db, FDB_KVDB_CTRL_SET_FILE_MODE, (void *)&file_mode);

    if (0 != access(path, 0) && 0 != mkdir(path, 0))
    {
        rt_kprintf("create db %s fail\n", DFU_DB);
        return -1;
    }

#endif /* FDB_USING_FILE_MODE */

    return fdb_kvdb_init(p_dfu_db, DFU_DB, path, &default_kv, NULL);
}
#endif

void dfu_ctrl_set_mode(uint8_t mode)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->mode = mode;
}

void run_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}

int dfu_ctrl_init(void)
{
    dfu_ctrl_init_env();
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
#ifdef FDB_USING_KVDB
    fdb_err_t err;
    struct fdb_blob blob;
    size_t read_len;

    do
    {
        err = dfu_db_init();
        if (err != FDB_NO_ERR)
        {
            LOG_E("nvds init failed !!!");
            break;
        }
        read_len = fdb_kv_get_blob(p_dfu_db, DFU_DOWNLOAD_ENV, fdb_blob_make(&blob, &env->prog, sizeof(dfu_download_progress_t)));
        if (read_len != sizeof(dfu_download_progress_t))
            OS_ASSERT(1);
        env->is_init = 1;
#ifdef DFU_OTA_MANAGER
        env->mode = DFU_CTRL_OTA_MODE;
        LOG_D("backup mode %d", dfu_get_backup_mode());
        LOG_I("ota state %d", env->prog.state);

        if (dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM)
        {
            dfu_image_state_reset_psram_info(env);
        }

        uint32_t ver_addr = (uint32_t)&OTA_VERSION;

        // OTA_VERSION is 4 byte align
        // we can fill some other data in first 2 bits later
        ver_addr = ver_addr >> 2;
        HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, ver_addr);
#else
        env->mode = DFU_CTRL_NORMAL_MODE;
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, DFU_FORCE_MODE_NONE);
#endif
    }
    while (0);
    // debug use
    //env->prog.state = DFU_CTRL_FORCE_UPDATE;
    if (env->mode == DFU_CTRL_NORMAL_MODE &&
            (env->prog.state == DFU_CTRL_UPDATING ||
             env->prog.state == DFU_CTRL_PACKAGE_INSTALL))
    {
        LOG_I("dfu finish");
        env->prog.state = DFU_CTRL_IDLE;
        env->is_force = 0;

        //dfu_update_img_header(env);
        memset(&env->prog, 0, sizeof(dfu_download_progress_t));

        dfu_ctrl_update_prog_info(env);
    }
#endif
    env->dfu_flash_thread = NULL;
    env->mb_handle = NULL;
    dfu_sec_init();
    return 0;
}
#ifdef FDB_USING_FILE_MODE
    INIT_PRE_APP_EXPORT(dfu_ctrl_init);
#else
    INIT_ENV_EXPORT(dfu_ctrl_init);
#endif /* FDB_USING_FILE_MODE */


int dfu_write_storage_data(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    int r = -1;
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    // LOG_I("dfu_get_storage_data %d, %d", header->img_id, header->flag);
    r = dfu_packet_write_flash(header, offset, data, size);
    return r;
}

void dfu_read_storage_data(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    // LOG_I("dfu_get_storage_data %d, %d", header->img_id, header->flag);
    dfu_packet_read_flash(header, offset, data, size);
}




static void dfu_image_package_start_rsp(dfu_ctrl_env_t *env, uint16_t result, uint32_t completed_count)
{
    LOG_I("dfu_image_package_start_rsp %d, %d", result, completed_count);
    dfu_image_package_start_rsp_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_PACKAGE_START_RSP, dfu_image_package_start_rsp_t);
    rsp->result = result;
    if (env->remote_version == 0)
    {
        env->rsp_frequency = 1;
    }
    else
    {
        env->rsp_frequency = DFU_DOWNLOAD_FREQUENCY;
    }

    rsp->response_frequency = env->rsp_frequency;
    rsp->reserved = 0;
    rsp->completed_count = completed_count;
    dfu_protocol_packet_send((uint8_t *)rsp);
}

static void dfu_image_package_start_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_image_package_start_handler");

    uint16_t status = DFU_ERR_NO_ERR;
    uint8_t erase_directly = 0;

    uint8_t *aligned_data = malloc(len);
    memcpy(aligned_data, data, len);

    uint32_t file_len;
    uint32_t packet_count;
    uint32_t crc_value;
    env->remote_version = 0;

    //LOG_I("dfu_image_package_start_handler LEN %d", len);
    if (len == sizeof(dfu_image_package_start_req_t))
    {
        LOG_I("old version");
        dfu_image_package_start_req_t *req = (dfu_image_package_start_req_t *)aligned_data;
        file_len = req->file_len;
        packet_count = req->packet_count;
        crc_value = req->crc_value;
    }
    else
    {
        dfu_image_package_start_req_new_t *req_new = (dfu_image_package_start_req_new_t *)aligned_data;
        LOG_I("remote version %d", req_new->version);
        env->remote_version = req_new->version;
        file_len = req_new->file_len;
        packet_count = req_new->packet_count;
        crc_value = req_new->crc_value;
    }



    if (env->dfu_flash_thread == NULL)
    {
        LOG_I("use dfu thread");

        if (g_dfu_ctrl_sem)
        {
            os_sem_delete(g_dfu_ctrl_sem);
            g_dfu_ctrl_sem = NULL;
        }
#ifdef OS_ADAPTOR_V2
        g_dfu_ctrl_sem = os_sem_create("dfu_ctrl_sem", 0);
#else
        os_sem_create(g_dfu_ctrl_sem, 0);
#endif
        env->dfu_flash_thread = ble_dfu_package_flash_thread_start();

        if (g_dfu_ctrl_sem)
        {
            os_sem_take(g_dfu_ctrl_sem, RT_WAITING_FOREVER);
            LOG_I("sem taken");
        }
    }

    uint32_t completed_count = 0;
    // check resume
    if (file_len == env->prog.all_length &&
            packet_count == env->prog.all_count &&
            crc_value == env->prog.crc)
    {
        LOG_I("resume at %d", env->prog.current_count);
        completed_count = env->prog.current_count;
    }

    do
    {
        if (completed_count == 0)
        {
            uint32_t size = file_len;
            uint32_t align_size;

            int8_t flash_type = dfu_get_flash_type(DFU_DOWNLOAD_REGION_START_ADDR);

            if (flash_type == DFU_FLASH_TYPE_NOR)
            {
#ifdef SOC_SF32LB55X
                // erase should 8k aligned
                align_size = 0x2000;
#else
                // none 55x 4k aligned
                align_size = 0x1000;
#endif
            }
            else if (flash_type == DFU_FLASH_TYPE_NAND)
            {
                align_size = 0x20000;
            }
            else if (flash_type == DFU_FLASH_TYPE_EMMC)
            {
                align_size = 1;
            }

            if (size % align_size != 0)
            {
                size = (size + align_size) / align_size * align_size;
            }
            LOG_I("dfu_image_package_start_handler erase 0x%x, 0x%x", DFU_DOWNLOAD_REGION_START_ADDR, size);

            if (size > DFU_DOWNLOAD_REGION_SIZE)
            {
                status = DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }

            env->prog.all_length = file_len;
            env->prog.all_count = packet_count;
            env->prog.current_count = 0;
            env->prog.crc = crc_value;

            if (env->mb_handle)
            {
                flash_write_package_t *fwrite;
                fwrite = rt_malloc(sizeof(flash_write_package_t));
                OS_ASSERT(fwrite);

                fwrite->base_addr = DFU_DOWNLOAD_REGION_START_ADDR;
                fwrite->offset = 0;
                fwrite->size = size;
                fwrite->msg_type = DFU_FLASH_MSG_TYPE_ERASE;

                //LOG_I("fwrite %d, %d, 0x%x", fwrite->offset, fwrite->size, fwrite);
                rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
            }
            else
            {
                erase_directly = 1;
                int ret = dfu_flash_erase(DFU_DOWNLOAD_REGION_START_ADDR, size);
                if (ret != 0)
                {
                    status = DFU_ERR_FLASH;
                }
            }
        }
    }
    while (0);

    if (status == DFU_ERR_NO_ERR)
    {
        env->prog.state = DFU_CTRL_TRAN_START;
    }
    free(aligned_data);

    if (completed_count != 0 || (completed_count == 0 && erase_directly == 1) || status != DFU_ERR_NO_ERR)
    {
        dfu_image_package_start_rsp(env, status, completed_count);
    }
}

static void dfu_package_image_packet_rsp(dfu_ctrl_env_t *env, uint16_t result, uint8_t retrans, uint32_t completed_count)
{
    if (result != DFU_ERR_NO_ERR)
    {
        LOG_I("dfu_package_image_packet_rsp %d, %d, %d", result, retrans, completed_count);
    }
    else
    {
        if (completed_count % env->rsp_frequency != 0 && completed_count != env->prog.all_count)
        {
            return;
        }
    }

    dfu_image_package_packet_rsp_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_PACKAGE_PACKET_RSP, dfu_image_package_packet_rsp_t);
    rsp->result = result;
    rsp->retransmission = retrans;
    rsp->reserved = 0;
    rsp->completed_count = completed_count;
    dfu_protocol_packet_send((uint8_t *)rsp);
}

static void dfu_package_image_packet_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    uint8_t status = DFU_ERR_NO_ERR;
    uint8_t retrans = 0;

    do
    {
        if (env->prog.state != DFU_CTRL_TRAN_START)
        {
            status = DFU_ERR_NOT_READY;
            break;
        }

        dfu_image_package_packet_t *packet = (dfu_image_package_packet_t *)data;
        if (packet->packet_index != env->prog.current_count + 1)
        {
            LOG_I
            ("dfu_package_image_packet_handler index error, expect %d, receive %d", env->prog.current_count + 1, packet->packet_index);
            status = DFU_ERR_INDEX_ERROR;
            retrans = 1;
            break;
        }

        uint32_t cal_crc = dfu_crc32mpeg2(packet->data, packet->data_len);

        if (cal_crc != packet->crc)
        {
            status = DFU_ERR_GENERAL_ERR;
            retrans = 1;
            break;
        }

        // last packet
        if (env->prog.current_count + 1 == env->prog.all_count)
        {
            if (env->prog.all_length != env->prog.current_count * 2048 + packet->data_len)
            {
                status = DFU_ERR_GENERAL_ERR;
                retrans = 0;
                break;
            }
        }

        LOG_D("dfu_image_package_packet_handler %d", env->prog.current_count);
        if (env->mb_handle)
        {
            flash_write_package_t *fwrite;
            fwrite = rt_malloc(sizeof(flash_write_package_t));
            OS_ASSERT(fwrite);

            fwrite->base_addr = DFU_DOWNLOAD_REGION_START_ADDR;
            fwrite->offset = 2048 * env->prog.current_count;
            fwrite->size = packet->data_len;
            fwrite->msg_type = DFU_FLASH_MSG_TYPE_DATA;
            rt_memcpy(fwrite->data, packet->data, packet->data_len);

            rt_err_t mb_ret = rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
            env->prog.current_count++;
            while (mb_ret != RT_EOK)
            {
                LOG_I("MB RET %d", mb_ret);
                rt_thread_mdelay(100);
                rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
            }
            return;
        }
        else
        {
            dfu_flash_write(DFU_DOWNLOAD_REGION_START_ADDR + 2048 * env->prog.current_count, packet->data, packet->data_len);

            env->prog.current_count++;
        }
    }
    while (0);
    dfu_package_image_packet_rsp(env, status, retrans, env->prog.current_count);
}

static void dfu_image_package_end_rsp(dfu_ctrl_env_t *env, uint16_t result)
{
    LOG_I("dfu_image_package_end_rsp");
    dfu_image_package_end_rsp_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_PACKAGE_END_RSP, dfu_image_package_end_rsp_t);
    rsp->result = result;
    dfu_protocol_packet_send((uint8_t *)rsp);
}

static dfu_package_install_packet_t *dfu_package_header_alloc(uint32_t data_addr)
{
    //uint32_t data_addr = DFU_RES_FLASH_CODE_START_ADDR;

    uint8_t remote_version;
    uint16_t image_count;
    uint32_t image_maglc;

    dfu_flash_read(data_addr, (uint8_t *)&image_maglc, 4);
    dfu_flash_read(data_addr + 4, (uint8_t *)&remote_version, 1);
    dfu_flash_read(data_addr + 4 + 2, (uint8_t *)&image_count, 2);

    if (image_maglc != SEC_CONFIG_MAGIC)
    {
        LOG_E("magic error");
        return NULL;
    }

    LOG_I("dfu_package_header_alloc image_count info_len %d, ver %d", image_count, remote_version);

    dfu_package_install_packet_t *packet = malloc(sizeof(dfu_package_install_packet_t) + sizeof(dfu_package_image_info_t) * image_count);

    uint16_t info_len = 6 * image_count;
    uint16_t read_len = sizeof(dfu_package_install_packet_t) + info_len;
    uint8_t *p_data = malloc(read_len);

    uint8_t *data = p_data;

    OS_ASSERT(p_data);
    OS_ASSERT(packet);

    dfu_flash_read(data_addr, data, read_len);

    memcpy((uint8_t *)&packet->magic, data, 4);
    data += 4;

    memcpy((uint8_t *)&packet->version, data, 1);
    data += 1;

    memcpy((uint8_t *)&packet->install_state, data, 1);
    data += 1;

    memcpy((uint8_t *)&packet->image_count, data, 2);
    data += 2;

    memcpy((uint8_t *)&packet->crc, data, 4);
    data += 4;


    dfu_package_image_info_t *image_info = (dfu_package_image_info_t *)(&packet->image_info);

    uint32_t offset = 12 + info_len;
    for (uint8_t i = 0; i < packet->image_count; i++)
    {
        memcpy((uint8_t *)&image_info[i].id, data, 1);
        data += 1;

        memcpy((uint8_t *)&image_info[i].flag, data, 1);
        data += 1;

        memcpy((uint8_t *)&image_info[i].len, data, 4);
        data += 4;

        image_info[i].offset = offset;
        offset += image_info[i].len;
    }

    // for (uint8_t i = 0; i < packet->image_count; i++)
    // {
    //     // DEBUG
    //     LOG_I("count %d, id %d, offset %d, len %d", i, image_info[i].id, image_info[i].offset, image_info[i].len);
    // }

    free(p_data);

    return packet;
}

static void dfu_package_header_free(uint8_t *data)
{
    free(data);
}

static void dfu_install_flag_update(uint8_t is_installing)
{
    int8_t flash_type = dfu_get_flash_type(DFU_DOWNLOAD_REGION_START_ADDR);

    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
        uint8_t current_state;
        dfu_flash_read(DFU_DOWNLOAD_REGION_START_ADDR + 5, &current_state, 1);
        if (current_state != 0xFF && current_state != DFU_PACKAGE_INSTALL)
        {
            LOG_W("install state not init %d", current_state);
            return;
        }

        if (is_installing != 0)
        {
            // 1111 1111 -> 0111 1111
            current_state = DFU_PACKAGE_INSTALL;
        }
        else
        {
            // 0111 1111 -> 0011 1111
            current_state = DFU_PACKAGE_INSTALL_FINISH;
        }

        dfu_flash_write(DFU_DOWNLOAD_REGION_START_ADDR + 5, &current_state, 1);
        LOG_I("dfu_install_flag_update update to %d", is_installing);
    }
    else
    {
        LOG_I("dfu_install_flag_update addr 0x%x", DFU_INFO_REGION_START_ADDR);

        if (DFU_INFO_REGION_START_ADDR == FLASH_UNINIT_32)
        {
            return;
        }
        dfu_install_info install_info;
        dfu_flash_read(DFU_INFO_REGION_START_ADDR, (uint8_t *)&install_info, sizeof(dfu_install_info));

        if (install_info.magic != SEC_CONFIG_MAGIC)
        {
            install_info.magic = SEC_CONFIG_MAGIC;
            install_info.version = 0;
        }

        if (is_installing != 0)
        {
            install_info.install_state = DFU_PACKAGE_INSTALL;
        }
        else
        {
            install_info.install_state = DFU_PACKAGE_INSTALL_FINISH;
        }

        dfu_flash_erase(DFU_INFO_REGION_START_ADDR, 0x20000);
        dfu_flash_write(DFU_INFO_REGION_START_ADDR, (uint8_t *)&install_info, sizeof(dfu_install_info));
        LOG_I("dfu_install_flag_update update to %d", is_installing);
    }
}

static uint8_t dfu_package_install(uint8_t type)
{
    //HAL_sw_breakpoint();

#if defined(BSP_USING_SDIO) || defined(BSP_USING_SDMMC1) || defined(BSP_USING_SDMMC2)
    if (type == PACKAGE_INSTALL_TYPE_OTA_MANAGER)
    {
        // wait for emmc init
        // TODO: find a better way
        rt_thread_mdelay(2000);
    }
#endif

    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    uint32_t download_base = DFU_DOWNLOAD_REGION_START_ADDR;
    uint8_t install_result = DFU_ERR_GENERAL_ERR;

    dfu_package_install_packet_t *packet = dfu_package_header_alloc(download_base);
    do
    {
        if (!packet)
        {
            LOG_I("alloc failed");
            break;
        }

        dfu_package_image_info_t *image_info = (dfu_package_image_info_t *)(&packet->image_info);

        uint32_t image_length = 0;
        for (int i = 0; i < packet->image_count; i++)
        {
            image_length += image_info[i].len;
        }
        LOG_I("dfu_package_install len %d header %d", image_length, (12 + 6 * packet->image_count));

        uint32_t offset;
        offset = 12 + 6 * packet->image_count;

        uint32_t cal_crc = 0xFFFFFFFF; // 初始值
        uint8_t buffer[2048]; // Flash读取缓冲区

        uint32_t crc_addr = DFU_DOWNLOAD_REGION_START_ADDR + offset;
        uint32_t addr_offset = 0;
        uint32_t check_len = 0;
        uint32_t process_len = 0;
        uint32_t read_len = 2048;

        while (addr_offset <= image_length)
        {
            dfu_flash_read(crc_addr + addr_offset, buffer, read_len);

            if (process_len + read_len > image_length)
            {
                check_len = image_length - process_len;
            }
            else
            {
                check_len = read_len;
            }

            cal_crc = crc32_update(cal_crc, buffer, check_len);
            process_len += check_len;

            addr_offset += read_len;
        }

        // align crc, for further use
        // while (addr_offset <= image_length) {
        //     dfu_flash_read(crc_addr + addr_offset, buffer, read_len);

        //     if (process_len + read_len > image_length)
        //     {
        //         check_len = image_length - process_len;
        //     } else {
        //         check_len = read_len;
        //     }

        //     if (addr_offset == 0)
        //     {
        //         crc = crc32_update(crc, buffer + offset, check_len - offset);
        //         process_len += check_len - offset;
        //     } else {
        //         crc = crc32_update(crc, buffer, check_len);
        //         process_len += check_len;
        //     }

        //     addr_offset += read_len;
        // }

        LOG_I("crc cal len %d", process_len);

        if (cal_crc != packet->crc)
        {
            LOG_I("crc error, 0x%x, 0x%x", cal_crc, packet->crc);
            dfu_package_header_free((uint8_t *)packet);
            break;
        }

        if (type == PACKAGE_INSTALL_TYPE_IMAGE)
        {
            dfu_install_flag_update(1);
        }

        for (int i = 0; i < packet->image_count; i++)
        {
            if (type == PACKAGE_INSTALL_TYPE_IMAGE)
            {
                if (image_info[i].id != DFU_IMG_ID_OTA_MANAGER)
                {
                    dfu_image_install_flash_package(env, image_info[i].id, image_info[i].len, image_info[i].offset, image_info[i].flag);
                }
            }
            else
            {
                if (image_info[i].id == DFU_IMG_ID_OTA_MANAGER)
                {
                    LOG_I("update ota manager");
                    dfu_image_install_flash_package(env, image_info[i].id, image_info[i].len, image_info[i].offset, image_info[i].flag);
                }
            }
        }

        LOG_I("install finish");
        if (type == PACKAGE_INSTALL_TYPE_IMAGE)
        {
            dfu_install_flag_update(0);
        }

        install_result = DFU_ERR_NO_ERR;
        dfu_package_header_free((uint8_t *)packet);
    }
    while (0);

    if (type == PACKAGE_INSTALL_TYPE_IMAGE)
    {
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, DFU_FORCE_MODE_REBOOT_TO_USER);
        env->prog.state = DFU_CTRL_PACKAGE_INSTALL;
        dfu_ctrl_update_prog_info(env);
        HAL_PMU_Reboot();
    }
    return install_result;
}

uint8_t dfu_package_install_set()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    uint8_t ret = dfu_package_install(PACKAGE_INSTALL_TYPE_OTA_MANAGER);

    LOG_I("dfu_package_install_set %d", ret);
    if (ret == DFU_ERR_NO_ERR)
    {
        env->prog.state = DFU_CTRL_PACKAGE_INSTALL_PREPARE;
        dfu_ctrl_update_prog_info(env);
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, DFU_FORCE_MODE_REBOOT_TO_PACKAGE_OTA_MANAGER);
    }

    return ret;
}

static void clear_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        __ISB();
    }
}

void dfu_bootjump(void)
{
    uint32_t target_addr;
    uint32_t target_size;
#ifdef DFU_OTA_MANAGER
    target_addr = HCPU_FLASH_CODE_START_ADDR;
    target_size = HCPU_FLASH_CODE_SIZE;
#else
    target_addr = DFU_FLASH_CODE_START_ADDR;
    target_size = DFU_FLASH_CODE_SIZE;
#endif

#ifdef RT_USING_WDT
    // TODO: Deinit watch log should be implmented in user bin
    extern void rt_hw_watchdog_deinit(void);
    rt_hw_watchdog_deinit();
#endif // RT_USING_WDT

    uint32_t i;
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    register rt_base_t ret;
    ret = rt_hw_interrupt_disable();
    clear_interrupt_setting();
    rt_hw_interrupt_enable(ret);

#ifdef DFU_OTA_MANAGER
    dfu_bootjump_sec_config(env, (uint8_t *)HCPU_FLASH_CODE_START_ADDR);
#endif

    for (i = 0; i < 8; i++)
        NVIC->ICER[0] = 0xFFFFFFFF;
    for (i = 0; i < 8; i++)
        NVIC->ICPR[0] = 0xFFFFFFFF;
    SysTick->CTRL = 0;
    SCB->ICSR |= SCB_ICSR_PENDNMICLR_Msk;
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTACT_Msk | SCB_SHCSR_BUSFAULTACT_Msk | SCB_SHCSR_MEMFAULTACT_Msk);

    if (CONTROL_SPSEL_Msk & __get_CONTROL())
    {
        __set_MSP(__get_PSP());
        __set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
    }

    SCB->VTOR = (uint32_t)target_addr;
#ifdef PSRAM_XIP
    memcpy((void *)PSRAM_BASE, (void *)target_addr, target_size);
    run_img((uint8_t *)PSRAM_BASE);
#else
    run_img((uint8_t *)target_addr);
#endif

}

static void dfu_ctrl_boot_to_user_fw(void)
{
    if (CONTROL_nPRIV_Msk & __get_CONTROL())
    {
        __asm("SVC #0");
    }
    else
    {
        dfu_bootjump();
    }
}


static void ble_dfu_install_process()
{
    dfu_package_install_set();

#ifdef SF32LB55X
    dfu_bootjump();
#else
    dfu_set_reboot_after_disconnect();
    dfu_protocol_session_close();
#endif
}

static void ble_dfu_start_install_thread()
{
    rt_thread_t tid;
    tid = rt_thread_create("ble_dfu_install", ble_dfu_install_process, NULL, 4096, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);
}

#ifdef PKG_USING_WEBCLIENT
static int dfu_pan_get(const char *url)
{
    rt_kprintf("dfu_pan_get\n");
    struct webclient_session *session = RT_NULL;
    int rc = WEBCLIENT_OK;
    int resp_status = 0;


    //check_dns();

    /* 创建会话 */
    session = webclient_session_create(1024);
    if (!session)
    {
        rt_kprintf("create session failed!\n");
        return -RT_ENOMEM;
    }

    /* 发送 GET 请求 */
    if ((resp_status = webclient_get(session, url)) != 200)
    {
        rt_kprintf("GET request failed, response code: %d\n", resp_status);
        rc = -RT_ERROR;
        goto __exit;
    }

    dfu_flash_erase(DFU_DOWNLOAD_REGION_START_ADDR, DFU_DOWNLOAD_REGION_SIZE);

    /* 分段接收数据 */
    size_t offset = 0;
    while (offset < session->content_length)
    {
        char buffer[2048];
        int bytes_read = webclient_read(session, buffer, sizeof(buffer));

        if (bytes_read <= 0)
            break;

        dfu_flash_write(DFU_DOWNLOAD_REGION_START_ADDR + offset, buffer, bytes_read);
        offset += bytes_read;

        rt_kprintf("Downloading: %d%%\r",
                   (int)(offset * 100 / session->content_length));
    }

    rt_kprintf("\nDownload complete! Saved to\n");

__exit:
    webclient_close(session);
    return rc;
}

static void download_thread_entry(void *param)
{
    const char *url = (const char *)param;
    dfu_pan_get(url);
    ble_dfu_start_install_thread();
}

void bt_dfu_pan_download(const char *url)
{
    rt_thread_t tid;
    tid = rt_thread_create("http_dl", download_thread_entry, (void *)url, 4096, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);
}
#endif

static void dfu_image_package_end_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_image_package_end_handler");
    uint8_t status = DFU_ERR_NO_ERR;
    do
    {
        if (env->prog.state != DFU_CTRL_TRAN_START)
        {
            status = DFU_ERR_NOT_READY;
            break;
        }

        dfu_image_package_end_req_t *packet = (dfu_image_package_end_req_t *)data;
        if (env->prog.all_count != env->prog.current_count)
        {
            status = DFU_ERR_FW_INVALID;
            break;
        }
    }
    while (0);

    if (env->mb_handle)
    {
        flash_write_package_t *fwrite;
        fwrite = rt_malloc(sizeof(flash_write_package_t));
        OS_ASSERT(fwrite);
        fwrite->msg_type = DFU_FLASH_MSG_TYPE_EXIT;
        rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
    }

    dfu_image_package_end_rsp(env, status);
    // directly install may cause bluetooth stack overflow
    ble_dfu_start_install_thread();
}


static void ble_dfu_package_flash()
{
    LOG_I("ble_dfu_flash_write: try to recv a mail");
    int thread_run = 1;
    uint8_t status;

    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->mb_handle = rt_mb_create("dfu_flash", 12, RT_IPC_FLAG_FIFO);

    if (g_dfu_ctrl_sem)
    {
        os_sem_release(g_dfu_ctrl_sem);
    }

    while (thread_run)
    {
        flash_write_package_t *fwrite;
        uint32_t p;
        int ret = 0;
        if (rt_mb_recv(env->mb_handle, (rt_uint32_t *)&p, RT_WAITING_FOREVER) == RT_EOK)
        {
            //LOG_I("ble_dfu_flash_write %d", p);
            fwrite = (flash_write_package_t *)p;

            switch (fwrite->msg_type)
            {
            case DFU_FLASH_MSG_TYPE_DATA:
                //LOG_I("ble_dfu_flash_write OFFSET %d, SIZE %d, addr 0x%x",fwrite->offset, fwrite->size, fwrite->base_addr + fwrite->offset);
                dfu_flash_write(fwrite->base_addr + fwrite->offset, fwrite->data, fwrite->size);

                rt_free(fwrite);

                if (ret != 0)
                {
                    LOG_I("ble_dfu_flash_write %d", ret);
                    OS_ASSERT(0);
                }
                dfu_package_image_packet_rsp(env, 0, 0, env->prog.current_count);
                break;
            case DFU_FLASH_MSG_TYPE_EXIT:
                LOG_I("DFU_FLASH_MSG_TYPE_EXIT");
                thread_run = 0;
                rt_free(fwrite);
                break;
            case DFU_FLASH_MSG_TYPE_ERASE:
                LOG_I("DFU_FLASH_MSG_TYPE_ERASE");
                int ret = dfu_flash_erase(fwrite->base_addr, fwrite->size);
                if (ret != 0)
                {
                    status = DFU_ERR_FLASH;
                }
                else
                {
                    status = DFU_ERR_NO_ERR;
                }
                rt_free(fwrite);
                dfu_image_package_start_rsp(env, status, 0);
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
    env->dfu_flash_thread = NULL;
}

static rt_thread_t ble_dfu_package_flash_thread_start()
{
    LOG_I("ble_dfu_package_flash_thread_start");
    rt_thread_t tid;
    tid = rt_thread_create("ble_dfu_package", ble_dfu_package_flash, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 5, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    return tid;
}

void dfu_protocol_packet_handler(dfu_tran_protocol_t *msg, uint16_t length)
{
    DFU_ERR_CHECK(msg);
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->current_command = msg->message_id;

    switch (msg->message_id)
    {
    case DFU_IMAGE_PACKAGE_START_REQ:
    {
        dfu_image_package_start_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_PACKAGE_PACKET_REQ:
    {
        dfu_package_image_packet_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_IMAGE_PACKAGE_END_REQ:
    {
        dfu_image_package_end_handler(env, msg->data, msg->length);
        break;
    }
    default:
        break;
    }
}

void dfu_ctrl_last_packet_handler()
{

}


static void dfu_flash_exit_msg_send()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    rt_thread_t thread;
    thread = env->dfu_flash_thread;

    if (thread != RT_NULL)
    {
        if (env->mb_handle)
        {
            flash_write_package_t *fwrite;
            fwrite = rt_malloc(sizeof(flash_write_package_t));
            OS_ASSERT(fwrite);
            fwrite->msg_type = DFU_FLASH_MSG_TYPE_EXIT;
            rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
        }
    }
}

static uint8_t dfu_ctrl_error_handle(dfu_ctrl_env_t *env)
{
    LOG_E("dfu_ctrl_error_handle");
    dfu_flash_exit_msg_send();
    if (env->mode == DFU_CTRL_OTA_MODE)
    {
        env->prog.state = DFU_CTRL_IDLE;
    }
    else
    {

    }
    return 0;
}

void dfu_protocol_close_handler(void)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (env->prog.state == DFU_CTRL_TRAN_START)
    {
        dfu_ctrl_error_handle(env);
    }
}

void dfu_serial_transport_error_handle(uint8_t error)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    switch (env->current_command)
    {
    case DFU_IMAGE_SEND_PACKET:
    {
        if (env->prog.state == DFU_CTRL_TRAN_START)
        {
            dfu_package_image_packet_rsp(env, DFU_ERR_INDEX_ERROR, 1, env->prog.current_count);
        }
        break;
    }
    default:
    {
        LOG_I("dfu_serial_transport_error_handle %d", env->current_command);
        dfu_ctrl_error_handle(env);
    }

        // TODO: handle other step's error later
    }
}

void dfu_register(dfu_callback callback)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->callback = callback;
}

uint8_t dfu_flash_addr_get(uint8_t img_id, dfu_flash_info_t *info)
{
    return DFU_ERR_GENERAL_ERR;
}

uint8_t dfu_ctrl_reset_handler(void)
{
    //HAL_sw_breakpoint();
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    RT_ASSERT(env->mode == DFU_CTRL_OTA_MODE);
    uint16_t state = env->prog.state;

    uint8_t status = DFU_ERR_GENERAL_ERR;
    uint8_t is_jump = 0;
    LOG_I("dfu_ctrl_reset_handler %d", state);

    switch (state)
    {
    case DFU_CTRL_PACKAGE_INSTALL_PREPARE:
    {
        is_jump = 0;
        dfu_package_install(PACKAGE_INSTALL_TYPE_IMAGE);
        break;
    }
    default:
        is_jump = 1;
        break;
    }

    if (is_jump)
    {
#ifdef SF32LB55X
        dfu_ctrl_boot_to_user_fw();
#else
        dfu_install_flag_update(0);
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, DFU_FORCE_MODE_REBOOT_TO_USER);
        env->prog.state = DFU_CTRL_IDLE;
        dfu_ctrl_update_prog_info(env);
        HAL_PMU_Reboot();
#endif
    }

    return 0;
}

#endif