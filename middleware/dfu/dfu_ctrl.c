/**
  ******************************************************************************
  * @file   dfu_ctrl.c
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


#define LOG_TAG "DFUCTRL"
#include "log.h"

#ifdef OTA_USING_SOL_FUNC
    #include "app_db.h"
    #include "app_comm.h"
    #include "app_mem.h"
    #include "app_pm.h"
#endif

OS_TIMER_DECLAR(g_dfu_timer);

OS_TIMER_DECLAR(g_dfu_sync_timer);

#if (defined(DFU_OTA_MANAGER) && defined(BSP_USING_PSRAM))
    OTA_L2_RET_SECT_BEGIN
    ALIGN(4) static uint8_t ota_hcpu_download_buffer[2300 * 1024];
    ALIGN(4) static uint8_t ota_lcpu_download_buffer[350 * 1024];
    ALIGN(4) static uint8_t ota_lcpu_rom_patch_download_buffer[16 * 1024];
    OTA_L2_RET_SECT_END
#else
    static uint8_t ota_hcpu_download_buffer[0];
    static uint8_t ota_lcpu_download_buffer[0];
    static uint8_t ota_lcpu_rom_patch_download_buffer[0];
#endif

DFU_NON_RET_SECT_BEGIN
//static uint8_t dfu_temp[DFU_MAX_BLK_SIZE];
static uint8_t dfu_temp_key[DFU_KEY_SIZE];
static struct fdb_kvdb g_dfu_db;
static fdb_kvdb_t p_dfu_db = &g_dfu_db;
#if (FDB_KV_CACHE_TABLE_SIZE == 1)
    static uint32_t g_dfu_db_cache[256];
#endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */
DFU_NON_RET_SECT_END

static dfu_ctrl_env_t g_dfu_ctrl_env;

dfu_control_packet_t *temp_ctrl_packet = NULL;
dfu_dl_image_header_t *temp_header = NULL;
uint8_t *ota_manager_download_buffer = NULL;



static void dfu_ctrl_install_completed(dfu_ctrl_env_t *env, uint16_t status);
static uint8_t dfu_ctrl_error_handle(dfu_ctrl_env_t *env);
static void dfu_link_sync_end();
static void dfu_link_lose_check_req(uint32_t current_index, uint16_t new_num_of_rsp);
static rt_thread_t ble_dfu_flash_write_thread_start();
static void dfu_flash_exit_msg_send();


// TODO: find a better way to set this mode
uint8_t g_dfu_progress_mode = DFU_PROGRESS_TOTAL;
uint8_t g_end_send_mode = DFU_END_SEND;

static void dfu_ctrl_init_env(void)
{
    dfu_control_packet_t *temp_ctrl_packet = NULL;
    dfu_dl_image_header_t *temp_header = NULL;
    uint8_t *ota_manager_download_buffer = NULL;
    memset((void *)&g_dfu_ctrl_env, 0, sizeof(g_dfu_ctrl_env));
}

static dfu_ctrl_env_t *dfu_ctrl_get_env(void)
{
    return &g_dfu_ctrl_env;
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

    dfu_bootjump_sec_config(env, (uint8_t *)HCPU_FLASH_CODE_START_ADDR);

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

    SCB->VTOR = (uint32_t)HCPU_FLASH_CODE_START_ADDR;
#ifdef PSRAM_XIP
    memcpy((void *)PSRAM_BASE, (void *)HCPU_FLASH_CODE_START_ADDR, HCPU_FLASH_CODE_SIZE);
    run_img((uint8_t *)PSRAM_BASE);
#else
    run_img((uint8_t *)HCPU_FLASH_CODE_START_ADDR);
#endif

}


void dfu_ctrl_boot_to_user_fw(void)
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

static bool dfu_check_image_need_force_update();

static bool check_current_img_need_force_update(int img_id)
{
    /*
    if (img_id == DFU_IMG_ID_RES ||
            img_id == DFU_IMG_ID_FONT ||
            img_id == DFU_IMG_ID_RES_UPGRADE ||
            img_id == DFU_IMG_ID_LCPU ||
            img_id == DFU_IMG_ID_PATCH ||
            img_id == DFU_IMG_ID_EX ||
            img_id == DFU_IMG_ID_TINY_FONT)
    {
        return true;
    }
    else
    {
        return false;
    }
    */

    if (dfu_check_image_need_force_update())
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t dfu_get_image_state()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (!env->is_init)
    {
        return 0xFF;
    }
    return env->prog.state;
}

void dfu_set_ota_mode(uint8_t mode)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->prog.ota_mode = mode;
    dfu_ctrl_update_prog_info(env);
}

uint8_t dfu_get_ota_mode()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (!env->is_init)
    {
        return 0xFF;
    }
    return env->prog.ota_mode;
}

#ifdef OTA_MODEM_RECORD
uint8_t dfu_get_modem_state()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (!env->is_init)
    {
        return 0xFF;
    }
    return env->prog.modem_ota_state;
}

uint8_t dfu_set_modem_state(uint8_t state)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->prog.modem_ota_state = state;
    dfu_ctrl_update_prog_info(env);
}
#endif

void dfu_protocol_abort_command(uint16_t reason)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    LOG_I("dfu_protocol_abort_command %d", reason);
    dfu_link_sync_end();
    dfu_abort_command_t *cmd = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_ABORT_COMMAND, dfu_abort_command_t);
    cmd->reason = reason;
    dfu_protocol_packet_send((uint8_t *)cmd);

    dfu_ctrl_error_handle(env);
}

static bool check_current_img_is_normal_mode_id(int img_id)
{
    if (img_id == DFU_IMG_ID_OTA_MANAGER || img_id == DFU_IMG_ID_BOOTLOADER)
    {
        return true;
    }
    return false;
}

uint8_t dfu_get_backup_mode()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    return env->prog.backup_mode;
}

static bool dfu_check_image_need_force_update()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        if (env->prog.image_state[i] >= DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE)
        {
            return true;
        }
    }

    return false;
}

void dfu_set_backup_mode()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->prog.backup_mode = 0;
#ifndef DFU_USING_DOWNLOAD_BACKUP
    env->prog.backup_mode = DFU_BACKUP_MODE_OVERWRITE;
#else

#ifdef DFU_USING_BACK_UP_FLASH
    env->prog.backup_mode = DFU_BACKUP_MODE_FLASH;
#endif

#ifdef DFU_USING_BACK_UP_PSRAM
    env->prog.backup_mode = DFU_BACKUP_MODE_PSRAM;
#endif

#if (defined(DFU_USING_BACK_UP_FLASH) && defined(DFU_USING_BACK_UP_PSRAM))
    LOG_E("dfu_set_backup_mode error, check config");
    OS_ASSERT(0);
#endif
#endif
}


static void dfu_image_state_init(dfu_ctrl_env_t *env)
{
    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        env->prog.image_state[i] = DFU_IMAGE_STATE_NONE;
    }
}

static void dfu_image_state_update_download_info(dfu_ctrl_env_t *env, dfu_dl_image_header_t *header)
{
    for (int i = 0; i < header->img_count; i++)
    {
        uint8_t image_id = header->img_header[i].img_id;
        env->prog.image_state[image_id] = DFU_IMAGE_STATE_READY;
    }
}

static void dfu_image_state_reset_psram_info(dfu_ctrl_env_t *env)
{
    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        if (env->prog.image_state[i] == DFU_IMAGE_STATE_DOWNLOAD_FINISH_TO_PSRAM ||
                env->prog.image_state[i] == DFU_IMAGE_STATE_DOWNLOADING_TO_PSRAM)
        {
            env->prog.image_state[i] = DFU_IMAGE_STATE_RESET_TO_PSRAM;
        }
    }
}

static uint8_t dfu_image_state_get_resume_mode(dfu_ctrl_env_t *env, uint8_t target_image_id)
{
    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        if (env->prog.image_state[i] == DFU_IMAGE_STATE_RESET_TO_PSRAM)
        {
            if (i < target_image_id)
            {
                return 1;
            }
        }
    }

    return 0;
}

static void dfu_image_set_download_begin(dfu_ctrl_env_t *env, dfu_image_header_int_t *header)
{
    if (header->flag & DFU_FLAG_COMPRESS)
    {
        if (dfu_get_backup_mode() == DFU_BACKUP_MODE_FLASH)
        {
            env->prog.image_state[header->img_id] = DFU_IMAGE_STATE_DOWNLOADING_TO_FLASH;
        }
        else if (dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM)
        {
            env->prog.image_state[header->img_id] = DFU_IMAGE_STATE_DOWNLOADING_TO_PSRAM;
        }
        else
        {
            LOG_E("dfu_image_set_download_begin error flag %d, mode %d", header->flag, dfu_get_backup_mode());
            OS_ASSERT(0);
        }
    }
    else
    {
        env->prog.image_state[header->img_id] = DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE;
    }

}

static void dfu_image_set_download_end(dfu_ctrl_env_t *env, uint8_t image_id)
{
    if (env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOADING_TO_FLASH ||
            env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOADING_TO_PSRAM ||
            env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE)
    {
        env->prog.image_state[image_id]++;
    }
    else
    {
        LOG_W("unexpect state %d, %d", env->prog.image_state[image_id], image_id);
    }
}

static void dfu_image_set_download_end_fail(dfu_ctrl_env_t *env, uint8_t image_id)
{
    if (env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOADING_TO_FLASH ||
            env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOAD_FINISH_TO_FLASH)
    {
        env->prog.image_state[image_id] = DFU_IMAGE_STATE_DOWNLOADING_TO_FLASH;
    }

    if (env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOADING_TO_PSRAM ||
            env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOAD_FINISH_TO_PSRAM)
    {
        env->prog.image_state[image_id] = DFU_IMAGE_STATE_DOWNLOADING_TO_PSRAM;
    }

    if (env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE ||
            env->prog.image_state[image_id] == DFU_IMAGE_STATE_DOWNLOAD_FINISH_OVERWRITE)
    {
        env->prog.image_state[image_id] = DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE;
    }
}




void dfu_read_storage_data(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    if (env->prog.ota_lite == 1)
    {
        dfu_packet_read_flash(header, offset, data, size);
    }
    else
    {
        if (dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM && (header->flag & DFU_FLAG_COMPRESS))
        {
            //LOG_I("GET PSRAM id %d", header->img_id);
            switch (header->img_id)
            {
            case DFU_IMG_ID_HCPU:
            {
                memcpy(data, ota_hcpu_download_buffer + offset, size);
                break;
            }
            case DFU_IMG_ID_LCPU:
            {
                memcpy(data, ota_lcpu_download_buffer + offset, size);
                break;
            }
            case DFU_IMG_ID_PATCH:
            {
                memcpy(data, ota_lcpu_rom_patch_download_buffer + offset, size);
                break;
            }
            case DFU_IMG_ID_OTA_MANAGER:
            {
                memcpy(data, ota_manager_download_buffer + offset, size);
                break;
            }
            case DFU_IMG_ID_BOOTLOADER:
            {
                memcpy(data, ota_manager_download_buffer + DFU_OTA_MANAGER_COM_SIZE + offset, size);
                break;
            }
            }
        }
        else
        {
            //LOG_I("dfu_get_storage_data %d, %d", header->img_id, header->flag);
            dfu_packet_read_flash(header, offset, data, size);
        }
    }
}

int dfu_write_storage_data(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    int r = -1;
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

#ifdef OTA_NOR_OTA_MANAGER_LITE
    r = dfu_packet_write_flash(header, offset, data, size);
    return r;
#endif

    if (dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM && (header->flag & DFU_FLAG_COMPRESS))
    {
        r = 0;
        //LOG_I("GET PSRAM id %d", header->img_id);
        switch (header->img_id)
        {
        case DFU_IMG_ID_HCPU:
        {
            memcpy(ota_hcpu_download_buffer + offset, data, size);
            break;
        }
        case DFU_IMG_ID_LCPU:
        {
            memcpy(ota_lcpu_download_buffer + offset, data, size);
            break;
        }
        case DFU_IMG_ID_PATCH:
        {
            memcpy(ota_lcpu_rom_patch_download_buffer + offset, data, size);
            break;
        }
        case DFU_IMG_ID_OTA_MANAGER:
        {
            memcpy(ota_manager_download_buffer + offset, data, size);
            break;
        }
        case DFU_IMG_ID_BOOTLOADER:
        {
            memcpy(ota_manager_download_buffer + DFU_OTA_MANAGER_COM_SIZE + offset, data, size);
            break;
        }
        }
    }
    else
    {
        //LOG_I("dfu_get_storage_data %d, %d", header->img_id, header->flag);
        r = dfu_packet_write_flash(header, offset, data, size);
    }
    return r;
}

void dfu_flash_erase_handle(void *para)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->flash_erase_state = DFU_FLASH_ERASE;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    dfu_img_info_t *curr_img = &env->prog.fw_context.code_img.curr_img_info;
    uint8_t status = DFU_ERR_NO_ERR;

    LOG_I("dfu_flash_erase_handle id %d, size %d", curr_img->header->img_id, curr_img->header->length);
    int ret = dfu_packet_erase_flash(curr_img->header, 0, curr_img->header->length);
    LOG_I("dfu_flash_erase_handle error %d", ret);

    if (env->flash_erase_state == DFU_FLASH_ERASE_INTERRUPT)
    {
        LOG_I("not send rsp");
        env->flash_erase_state = DFU_FLASH_NONE;
        return;
    }

    env->flash_erase_state = DFU_FLASH_NONE;

    if (ret != DFU_ERR_NO_ERR)
    {
        status = DFU_ERR_FLASH;
    }

    dfu_app_img_dl_start_ind_t ind;
    ind.total_imgs_num = env->prog.fw_context.code_img.img_count;
    ind.curr_img_id = curr_img->header->img_id;
    ind.curr_img_total_len = curr_img->header->length;

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

    if (status == DFU_ERR_NO_ERR)
    {
        if (env->callback)
            env->callback(DFU_APP_IMAGE_DL_START_IND, &ind);

        env->sync_size = 0;
        dfu_link_sync_start(DFU_SYNC_TYPE_DOWNLOAD);
    }

    dfu_ctrl_update_prog_info(env);
    LOG_E("dfu_img_send_start_handler rsp: %d", status);

    dfu_image_send_start_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_START_RESPONSE, dfu_image_send_start_response_t);
    rsp->result = status;
    rsp->end_send = g_end_send_mode;
    dfu_protocol_packet_send((uint8_t *)rsp);

    if (status != DFU_ERR_NO_ERR)
    {
        if (check_current_img_need_force_update(curr_img->img_id))
        {
            env->is_force = 1;
        }
        dfu_ctrl_error_handle(env);
    }
    else
    {
        ble_dfu_request_connection_priority();
    }
}

static rt_thread_t ble_dfu_flash_erase_thread_start()
{
    LOG_I("ble_dfu_flash_erase_thread_start %d", RT_MAIN_THREAD_PRIORITY + 5);
    rt_thread_t tid;
    tid = rt_thread_create("ble_dfu_flash", dfu_flash_erase_handle, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 5, 10);
    rt_thread_startup(tid);
    return tid;
}

int dfu_clear_storage_data(dfu_image_header_int_t *header, uint32_t offset, uint32_t size)
{
    int ret;
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    LOG_I("dfu_clear_storage_data mode %d, id %d, flag %d", dfu_get_backup_mode(), header->img_id, header->flag);

#ifdef OTA_NOR_OTA_MANAGER_LITE
    // TODO: malloc psram mem
    if (env->prog.dfu_ID == DFU_ID_CODE || env->prog.dfu_ID == DFU_ID_CODE_MIX)
    {
        LOG_I("erase lite");
        int ret;
        ret = dfu_packet_erase_flash(header, offset, size);
        return ret;
    }
#endif

    if (dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM && (header->flag & DFU_FLAG_COMPRESS))
    {
        // PSRAM, do nothing
        ret = 0;
    }
    else
    {
        if (env->mode == DFU_CTRL_NORMAL_MODE)
        {
            int ret;
            ret = dfu_packet_erase_flash(header, offset, size);
        }
        else if (env->mode == DFU_CTRL_OTA_MODE)
        {
            ble_dfu_flash_erase_thread_start();
            LOG_I("after ble_dfu_flash_erase_thread_start");
            ret = ERASE_PROCESSING;
        }
    }
    return ret;
}


void dfu_ota_bootloader_ram_run_flash_init()
{
    dfu_ram_info info;
    uint32_t addr = DFU_RAM_RUN_STATE_START_ADDR;
    dfu_flash_read(addr, (uint8_t *)&info, sizeof(dfu_ram_info));

    if (info.magic != SEC_CONFIG_MAGIC)
    {
        LOG_I("dfu_ota_bootloader_ram_run_flash_init not ready");
        // first use not ready
        info.magic = SEC_CONFIG_MAGIC;
        info.dfu_state = DFU_RAM_STATE_NONE;
        info.ota_addr = 0;
        dfu_flash_erase(addr, 4096);
        dfu_flash_write(addr, (uint8_t *)&info, sizeof(dfu_ram_info));
        return;
    }

    LOG_I("dfu_ota_bootloader_ram_run_flash_init %d", info.dfu_state);
    if (info.dfu_state == DFU_RAM_STATE_UPDATED)
    {
        // ota success
    }
    else if (info.dfu_state == DFU_RAM_STATE_UPDATING)
    {
        // illegal state
        // updating should be processed in ota manager
    }
    else if (info.dfu_state == DFU_RAM_STATE_UPDATE_FAIL)
    {
        // ota fail
    }
    else if (info.dfu_state == DFU_RAM_STATE_NONE)
    {
        // no ota
    }
}

void dfu_ota_bootloader_ram_run_set(uint8_t state)
{
    LOG_I("dfu_ota_bootloader_ram_run_set %d", state);
    dfu_ram_info info;
    uint32_t addr = DFU_RAM_RUN_STATE_START_ADDR;
    // dfu_flash_read(addr, (uint8_t *)&info, sizeof(dfu_ram_info));

    info.magic = SEC_CONFIG_MAGIC;
    info.ota_addr = 0x120a2000;
    info.dfu_state = state;
    dfu_flash_erase(addr, 4096);

    dfu_flash_write(addr, (uint8_t *)&info, sizeof(dfu_ram_info));
}

static uint8_t dfu_ctrl_packet_img_header_check(dfu_ctrl_env_t *env, dfu_code_image_header_t *header)
{
    uint8_t ret = DFU_ERR_NO_ERR;
    dfu_flash_info_t info;
    uint32_t flash_size;
    for (uint32_t i = 0; i < header->img_count; i++)
    {
        if (dfu_flash_addr_get(header->img_header[i].img_id, &info) == DFU_ERR_NO_ERR)
        {
            flash_size = info.size;
            ret = header->img_header[i].length <= flash_size ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
            break;
        }
        else
        {
            switch (header->img_header[i].img_id)
            {
            case DFU_IMG_ID_HCPU:
            {
                ret = header->img_header[i].length <= HCPU_FLASH_CODE_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }
#ifndef SOC_SF32LB52X
            case DFU_IMG_ID_LCPU:
            {
                ret = header->img_header[i].length <= LCPU_FLASH_CODE_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }
            case DFU_IMG_ID_PATCH:
            {
                ret = header->img_header[i].length <= LCPU_PATCH_TOTAL_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }
#endif
            case DFU_IMG_ID_RES:
            {
                ret = header->img_header[i].length <= HCPU_FLASH2_IMG_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }
            case DFU_IMG_ID_FONT:
            {
                ret = header->img_header[i].length <= HCPU_FLASH2_FONT_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }
            case DFU_IMG_ID_TINY_FONT:
            {
#ifdef HCPU_FLASH2_TINY_FONT_START_ADDR
                ret = header->img_header[i].length <= HCPU_FLASH2_TINY_FONT_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
#else
                LOG_W("tiny not found");
                ret = DFU_ERR_SPACE_NOT_ENOUGH;
#endif
                break;

            }
            case DFU_IMG_ID_RES_UPGRADE:
            {
                ret = header->img_header[i].length <= HCPU_FLASH2_IMG_UPGRADE_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
            }
            case DFU_IMG_ID_EX:
            {
#ifdef HCPU_FS_ROOT_START_ADDR
                ret = header->img_header[i].length <= HCPU_FS_ROOT_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
#else
                LOG_W("ex/fs root not found");
                ret = DFU_ERR_SPACE_NOT_ENOUGH;
#endif
                break;
            }
#ifdef OTA_NOR_OTA_MANAGER_LITE
            case DFU_IMG_ID_OTA_MANAGER:
                ret = header->img_header[i].length <= DFU_FLASH_CODE_SIZE ? DFU_ERR_NO_ERR : DFU_ERR_SPACE_NOT_ENOUGH;
                break;
#endif
            default:
                ret = DFU_ERR_PARAMETER_INVALID;
                break;
            }
        }

        if (ret != DFU_ERR_NO_ERR)
            break;
    }

    return ret;
}

void SVC_Handler(void)
{
    __asm(
        ".global SVC_Handler_Main\n"
        "TST lr, #4\n"
        "ITE EQ\n"
        "MRSEQ r0, MSP\n"
        "MRSNE r0, PSP\n"
        "B SVC_Handler_Main\n"
    );

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
static uint8_t dfu_ctrl_packet_check(dfu_ctrl_env_t *env, dfu_control_packet_t *packet, uint16_t total_len)
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
                ret = dfu_ctrl_packet_img_header_check(env, header);
                if (ret != DFU_ERR_NO_ERR)
                    break;
            }

            /* All verificaition PASS.*/
            ret = DFU_ERR_NO_ERR;
        }

        if (packet->dfu_ID == DFU_ID_OTA_MANAGER)
        {
            ret = DFU_ERR_NO_ERR;
        }

    }
    while (0);
    return ret;

}


static void dfu_ctrl_packet_postpone_handler(dfu_ctrl_env_t *env, uint16_t status)
{
    if (status == DFU_ERR_USER_REJECT)
        memset(&env->prog, 0, sizeof(dfu_download_progress_t));
}

static uint16_t dfu_ctrl_packet_code_fw_handler(dfu_ctrl_env_t *env, dfu_control_packet_t *ctrl_packet, uint16_t len)
{
    uint16_t status;
    do
    {
        /* If user callback doesn't register, always reject. */
        if (!env->callback)
        {
            status = DFU_ERR_USER_REJECT;
            break;
        }

        status = dfu_ctrl_packet_check(env, ctrl_packet, len);
        if (status != DFU_ERR_NO_ERR)
        {
            LOG_W("ctrl packet check");
            break;
        }

        /* Notify user */
        dfu_app_start_request_t app_req;
        dfu_event_ack_t ret;
        app_req.dfu_id = ctrl_packet->dfu_ID;
        app_req.is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;
        env->ota_state.dfu_ID = ctrl_packet->dfu_ID;

        if (ctrl_packet->dfu_ID == DFU_ID_OTA_MANAGER)
        {
            dfu_code_image_header_t *header = (dfu_code_image_header_t *)&ctrl_packet->image_header;

            ret = env->callback(DFU_APP_START_REQUEST, &app_req);
            if (ret == DFU_EVENT_FAILED)
            {
                status = DFU_ERR_USER_REJECT;
                return status;
            }

            uint8_t *aes_out_m;
            aes_out_m = rt_malloc(DFU_KEY_SIZE);
            rt_memset(aes_out_m, 0, DFU_KEY_SIZE);

            SCB_InvalidateDCache_by_Addr(aes_out_m, DFU_KEY_SIZE);
            SCB_InvalidateICache_by_Addr(aes_out_m, DFU_KEY_SIZE);

            sifli_hw_enc(ctrl_packet->FW_key, aes_out_m, DFU_KEY_SIZE);
            rt_memcpy(env->ota_state.FW_key, aes_out_m, DFU_KEY_SIZE);
            rt_free(aes_out_m);

            env->ota_state.FW_version = ctrl_packet->FW_version;
            env->ota_state.SDK_version = ctrl_packet->SDK_version;
            env->ota_state.HW_version = ctrl_packet->HW_version;
            env->ota_state.fw_context.code_img.blk_size = header->blk_size;
            env->ota_state.fw_context.code_img.img_count = header->img_count;
            rt_memcpy((uint8_t *)&env->ota_state.fw_context.code_img.img_header, (uint8_t *)&header->img_header, sizeof(dfu_image_header_int_t) * header->img_count);

            if (ret == DFU_EVENT_POSTPONE)
            {
                status = DFU_ERR_POSTPONE;
            }

            env->prog.dfu_ID = ctrl_packet->dfu_ID;
            return status;
        }

#ifdef OTA_NOR_OTA_MANAGER_LITE
        if (env->mode == DFU_CTRL_NORMAL_MODE &&
                (ctrl_packet->dfu_ID == DFU_ID_CODE || ctrl_packet->dfu_ID == DFU_ID_CODE_MIX))
        {
            app_req.dfu_id = DFU_ID_CODE_DOWNLOAD_IN_HCPU;

            dfu_code_image_header_t *header = (dfu_code_image_header_t *)&ctrl_packet->image_header;
            for (int i = 0; i < header->img_count; i++)
            {
                if (header->img_header[i].img_id > DFU_IMG_ID_PATCH)
                {
                    app_req.dfu_id = DFU_ID_CODE_RES_DOWNLOAD_IN_HCPU;
                    break;
                }
            }
        }
#endif
        ret = env->callback(DFU_APP_START_REQUEST, &app_req);

        if (ret == DFU_EVENT_FAILED)
        {
            status = DFU_ERR_USER_REJECT;
            break;
        }
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
        LOG_I("update ctrl packet");

        /* If user choose reponse later, dfu should not ack immediately. */
        if (ret == DFU_EVENT_POSTPONE)
        {
            status = DFU_ERR_POSTPONE;
        }

    }
    while (0);

    return status;
}


static uint16_t dfu_ctrl_packet_check_compare(dfu_ctrl_env_t *env, dfu_control_packet_t *packet, uint16_t total_len)
{
    uint16_t status = DFU_ERR_PARAMETER_INVALID;
    do
    {
        dfu_download_progress_t *progress = &env->prog;
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
        rt_memcpy(dfu_temp_key, aes_out, DFU_KEY_SIZE);
        rt_free(aes_in);
        rt_free(aes_out);

        if (memcmp(packet->FW_key, dfu_temp_key, sizeof(packet->FW_key)) != 0)
        {
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
                LOG_E("flag id error %d, %d, %d", i, header->img_header[i].flag, dl_hdr->img_header[i].flag);
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

static uint16_t dfu_ctrl_packet_code_fw_compare(dfu_ctrl_env_t *env, dfu_control_packet_t *ctrl_packet, uint16_t len)
{
    uint16_t status;
    do
    {
        /* If user callback doesn't register, always reject. */

        if (env->mode == DFU_CTRL_NORMAL_MODE && !env->callback)
        {
            status = DFU_ERR_USER_REJECT;
            break;
        }

        status = dfu_ctrl_packet_check_compare(env, ctrl_packet, len);
        if (status != DFU_ERR_NO_ERR)
            break;

        /* Notify user */
        env->prog.state = DFU_CTRL_NEG;
        dfu_app_resume_request_t app_req;
        dfu_event_ack_t ret = DFU_EVENT_SUCCESSED;
        app_req.dfu_id = ctrl_packet->dfu_ID;
        app_req.is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;

#ifdef OTA_NOR_OTA_MANAGER_LITE
        if (env->mode == DFU_CTRL_NORMAL_MODE &&
                (ctrl_packet->dfu_ID == DFU_ID_CODE || ctrl_packet->dfu_ID == DFU_ID_CODE_MIX))
        {
            app_req.dfu_id = DFU_ID_CODE_DOWNLOAD_IN_HCPU;

            dfu_code_image_header_t *header = (dfu_code_image_header_t *)&ctrl_packet->image_header;
            for (int i = 0; i < header->img_count; i++)
            {
                if (header->img_header[i].img_id > DFU_IMG_ID_PATCH)
                {
                    app_req.dfu_id = DFU_ID_CODE_RES_DOWNLOAD_IN_HCPU;
                    break;
                }
            }
        }
#endif

        if (env->callback)
            ret = env->callback(DFU_APP_RESUME_REQUEST, &app_req);

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


static void dfu_ctrl_request_init_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len, uint8_t is_force)
{
    LOG_I("dfu_ctrl_request_init_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    uint8_t status = DFU_ERR_GENERAL_ERR;
    uint8_t is_update_ota = 0;

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
        if (dfu_ctrl_ctrl_header_sig_verify(env, packet, len, sig) < 0)
        {
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
            else if (dfu_ID == DFU_ID_OTA_MANAGER)
            {
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_handler(env, ctrl_packet, len);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
                if (env->mode == DFU_CTRL_NORMAL_MODE)
                {
                    is_update_ota = 1;
                }


                /*
                                dfu_init_response_t *ota_rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE, dfu_init_response_t);
                                if (env->mode == DFU_CTRL_NORMAL_MODE)
                                {
                                    ota_rsp->result = DFU_ERR_NO_ERR;
                                    ota_rsp->is_boot = 0;
                                    env->ota_state.state = OTA_STATE_PREPARE;
                                    is_update_ota = 1;
                                }
                                else
                                {
                                    // shall not update ota manager in ota mode
                                    ota_rsp->result = DFU_ERR_NOT_READY;
                                    ota_rsp->is_boot = 0;
                                    env->ota_state.state = OTA_STATE_NONE;
                                }

                                dfu_protocol_packet_send((uint8_t *)ota_rsp);
                */
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

    if (is_update_ota)
    {
        LOG_I("return for choose %d", status);
        //return;
    }

    // set to none as code ota will going
    //env->ota_state.state = OTA_STATE_NONE;

    if (sig)
        free(sig);
    /* Prepare response. */
    if (status != DFU_ERR_POSTPONE)
    {
        dfu_init_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE, dfu_init_response_t);
        rsp->result = status;
        rsp->is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;

        if (status == DFU_ERR_NO_ERR)
        {
            // set for no confirm from user
            if (rsp->is_boot)
            {
                dfu_set_reboot_after_disconnect();
            }
        }

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

static void dfu_ctrl_request_init_handler_ext(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_ctrl_request_init_handler_ext");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    uint8_t status = DFU_ERR_GENERAL_ERR;
    uint8_t is_update_ota = 0;
    env->resume_status = 0;
    env->resume_async = 0;

    if (env->dfu_flash_thread)
    {
        dfu_flash_exit_msg_send();
    }

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

        if (env->is_sync_timer_on)
        {
            LOG_I("clear at init");
            dfu_link_sync_end();
        }

        /* First check the signature.*/
        if (dfu_ctrl_ctrl_header_sig_verify(env, packet, len, sig) < 0)
        {
            status = DFU_ERR_CONTROL_PACKET_INVALID;
            break;
        }

        LOG_I("dfu_ctrl_request_init_handler_ext state %d", state);

        uint8_t dfu_ID = *packet;
        switch (state)
        {
        /* In normal mode. */
        case DFU_CTRL_IDLE:
        case DFU_CTRL_FORCE_UPDATE:
        case DFU_CTRL_REBOOT_INSTALL_PREPARE:
        {
            /* OTA mode could have IDLE state if no need to reboot. */
            //if (env->mode != DFU_CTRL_NORMAL_MODE)
            //break;
            if (dfu_ID == DFU_ID_CODE ||
                    dfu_ID == DFU_ID_CODE_MIX)
            {
#ifndef OTA_NOR_OTA_MANAGER_LITE
                if (env->mode == DFU_CTRL_NORMAL_MODE && dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM)
                {
                    env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                }
#endif

                if (env->prog.FW_state >= DFU_CTRL_FW_DOWNLOADING)
                {
                    dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                    status = dfu_ctrl_packet_code_fw_compare(env, ctrl_packet, len);
                    LOG_I("dfu_ctrl_request_init_handler_ext status %d", status);
                    dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);

                    if (status == DFU_ERR_POSTPONE)
                    {
                        env->resume_async = 1;
                        status = DFU_ERR_NO_ERR;
                    }
                    if (status == DFU_ERR_NO_ERR)
                    {
                        // can resume
                        env->prog.state = DFU_CTRL_NEG;
                        env->resume_status = 1;
                        LOG_I("dfu_ctrl_request_init_handler_ext ready to resume");

                        if (temp_ctrl_packet)
                        {
                            dfu_ctrl_ctrl_header_free((uint8_t *)temp_ctrl_packet);
                            temp_ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                        }
                        else
                        {
                            temp_ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                        }

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
                // if set this under user bin, next reboot will clear so can jump to user if
                // hcpu fail, but will not jump if downloading under force update
                env->is_force_update = 1;
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_handler(env, ctrl_packet, len);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
            }
            else if (dfu_ID == DFU_ID_OTA_MANAGER)
            {
                dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);
                status = dfu_ctrl_packet_code_fw_handler(env, ctrl_packet, len);
                dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
                if (env->mode == DFU_CTRL_NORMAL_MODE)
                {
                    is_update_ota = 1;
                }
                /*
                                dfu_init_response_t *ota_rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE, dfu_init_response_t);
                                if (env->mode == DFU_CTRL_NORMAL_MODE)
                                {
                                    ota_rsp->result = DFU_ERR_NO_ERR;
                                    ota_rsp->is_boot = 0;
                                    env->ota_state.state = OTA_STATE_PREPARE;
                                    is_update_ota = 1;
                                }
                                else
                                {
                                    // shall not update ota manager in ota mode
                                    ota_rsp->result = DFU_ERR_NOT_READY;
                                    ota_rsp->is_boot = 0;
                                    env->ota_state.state = OTA_STATE_NONE;
                                }

                                dfu_protocol_packet_send((uint8_t *)ota_rsp);
                */
            }
            else
            {
                //TODO Support other OTA method, before implment it, just reponse err.
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

    if (is_update_ota)
    {
        LOG_I("return for choose %d", status);
        //return;
    }

    // set to none as code ota will going
    //env->ota_state.state = OTA_STATE_NONE;

    if (sig)
        free(sig);

    if (env->resume_status == 1)
    {
        if (status == DFU_ERR_NO_ERR)
        {
            dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;
            env->resume_is_restart = env->prog.FW_state > DFU_CTRL_FW_DOWNLOADING ? 1 : 0;
            if (!env->resume_is_restart && env->prog.FW_state == DFU_CTRL_FW_DOWNLOADING)
            {
                if (dl_hdr->curr_img_info.img_state == DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL)
                {
                    LOG_I("DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL");
                    dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_IDLE;
                    env->resume_is_restart = 1;
                }
                else if (dl_hdr->curr_img_info.img_state == DFU_CTRL_IMG_STATE_DOWNLOADED)
                {
                    LOG_I("DFU_CTRL_IMG_STATE_DOWNLOADED");
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
                        env->resume_is_restart = 1;
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

                LOG_I("dfu init resume info restart %d, id %d, num %d, rsp %d", env->resume_is_restart,
                      dl_hdr->curr_img_info.img_id, dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num,
                      dl_hdr->curr_img_info.img_info.dl_info.num_of_rsp);
            }

        }

    }

    if (env->resume_async)
    {
        status = DFU_ERR_POSTPONE;
    }
    /* Prepare response. */
    if (status != DFU_ERR_POSTPONE)
    {
        dfu_init_response_ext_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE_EXT, dfu_init_response_ext_t);
        rsp->result = status;
#ifdef OTA_NOR_OTA_MANAGER_LITE
        rsp->is_boot = 0;
#else
        rsp->is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;
#endif
        rsp->resume_status = env->resume_status;
        rsp->ver = OTA_CODE_VERSION;

        if (env->resume_status == 1 && dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM)
        {
            if (dfu_image_state_get_resume_mode(env, env->prog.fw_context.code_img.curr_img_info.img_id))
            {
                rsp->resume_status = 2;
            }
        }

        if (env->resume_status == 1 && !env->resume_is_restart)
        {
            dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;
            rsp->curr_img = dl_hdr->curr_img_info.img_id;
            rsp->curr_packet_num = dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num;
            rsp->num_of_rsp = dl_hdr->curr_img_info.img_info.dl_info.num_of_rsp;

            env->resume_image_index = dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num;
            env->resume_image_id = dl_hdr->curr_img_info.img_id;
        }

        if (status == DFU_ERR_NO_ERR)
        {
            // set for no confirm from user
            if (rsp->is_boot)
            {
                dfu_set_reboot_after_disconnect();
            }
        }

        dfu_protocol_packet_send((uint8_t *)rsp);

        if (status == DFU_ERR_UNEXPECT_STATE)
        {
            env->prog.state = DFU_ERR_NO_ERR;
            int ret = dfu_protocol_session_close();
            if (ret == -1)
            {
                dfu_port_svc_session_close();
            }
        }
    }

}


static void dfu_init_completed_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_init_completed_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_init_completed_ind_t *ind = (dfu_init_completed_ind_t *)data;
    /* Not handle not start case. */
    OS_ASSERT(ind->is_start);


    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        // only user mode refresh this
        dfu_set_backup_mode();
        LOG_D("ota backup mode %d", dfu_get_backup_mode());
    }

    switch (state)
    {
    case DFU_CTRL_NEG:
    {
#ifdef OTA_ERROR_HANDLE_IN_USR
        if (env->is_force_update == 1)
            env->prog.state = DFU_CTRL_PREPARE_START_FORCE;
        else
#endif
            env->prog.state = DFU_CTRL_PREPARE_START;

        if (env->mode == DFU_CTRL_NORMAL_MODE)
        {
            if (env->ota_state.dfu_ID != DFU_ID_OTA_MANAGER)
            {
                if (env->is_reboot_timer_on == 1)
                {
                    env->is_reboot_timer_on = 0;
                    os_timer_stop(g_dfu_timer);
                    os_timer_delete(g_dfu_timer);
                }
            }
            /* Save current status to flash. */
            dfu_ctrl_update_prog_info(env);

            // dfu_set_reboot_after_disconnect();

            int ret = dfu_protocol_session_close();
            if (ret == -1)
            {
                dfu_port_svc_session_close();
            }

            // should reboot after disconnect
            /* Prepare reboot device. */
            // HAL_PMU_Reboot();
        }
        else
        {
            /* W4 image send start in OTA mode. */
        }
    }
    break;
    default:
        break;
    }

}

static void dfu_init_completed_handler_ext(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_init_completed_handler_ext");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_init_completed_ind_t *ind = (dfu_init_completed_ind_t *)data;
    LOG_I("dfu_init_completed_handler_ext %d %d", ind->is_start, env->resume_status);
    /* Not handle not start case. */
    //OS_ASSERT(ind->is_start);

    if (env->resume_status == 1 && ind->is_start == 1)
    {
        LOG_I("use resume");
    }
    else
    {
        env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
    }

    if (env->resume_status == 1 && ind->is_start == 0)
    {
        // should update header
        if (temp_ctrl_packet)
        {
            dfu_code_image_header_t *header = (dfu_code_image_header_t *)&temp_ctrl_packet->image_header;
            env->prog.state = DFU_CTRL_NEG;
            env->prog.dfu_ID = temp_ctrl_packet->dfu_ID;

            uint8_t *aes_out;
            aes_out = rt_malloc(DFU_KEY_SIZE);
            rt_memset(aes_out, 0, DFU_KEY_SIZE);

            SCB_InvalidateDCache_by_Addr(aes_out, DFU_KEY_SIZE);
            SCB_InvalidateICache_by_Addr(aes_out, DFU_KEY_SIZE);

            sifli_hw_enc(temp_ctrl_packet->FW_key, aes_out, DFU_KEY_SIZE);
            rt_memcpy(env->prog.FW_key, aes_out, DFU_KEY_SIZE);
            rt_free(aes_out);

            env->prog.FW_version = temp_ctrl_packet->FW_version;
            env->prog.SDK_version = temp_ctrl_packet->SDK_version;
            env->prog.HW_version = temp_ctrl_packet->HW_version;
            env->prog.fw_context.code_img.blk_size = header->blk_size;
            env->prog.fw_context.code_img.img_count = header->img_count;
            rt_memcpy((uint8_t *)&env->prog.fw_context.code_img.img_header, (uint8_t *)&header->img_header, sizeof(dfu_image_header_int_t) * header->img_count);

            dfu_ctrl_ctrl_header_free((uint8_t *)temp_ctrl_packet);
            temp_ctrl_packet = NULL;

            LOG_I("update ctrl packet init complete");
        }
    }

    if (!(env->resume_status == 1 && ind->is_start == 1))
    {
        dfu_image_state_init(env);
        dfu_image_state_update_download_info(env, &env->prog.fw_context.code_img);
        env->resume_status = 0;
    }
    else
    {
        env->resume_status = 1;
        if (env->mode == DFU_CTRL_OTA_MODE && dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM)
        {
            if (temp_header == NULL)
            {
                temp_header = malloc(sizeof(dfu_dl_image_header_t));
                OS_ASSERT(temp_header);
            }
            memcpy(temp_header, &env->prog.fw_context.code_img, sizeof(dfu_dl_image_header_t));
        }
    }

    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        // only user mode refresh this
        dfu_set_backup_mode();
        LOG_D("ota backup mode %d", dfu_get_backup_mode());
    }

    switch (state)
    {
    case DFU_CTRL_NEG:
    {
        env->prog.state = DFU_CTRL_PREPARE_START;

        if (env->mode == DFU_CTRL_NORMAL_MODE)
        {
#ifdef OTA_NOR_OTA_MANAGER_LITE
            if (env->mode == DFU_CTRL_NORMAL_MODE)
            {
                env->prog.ota_lite = 1;
            }
            dfu_ctrl_update_prog_info(env);
#else
            if (env->mode == DFU_CTRL_NORMAL_MODE)
            {
                env->prog.ota_lite = 0;
            }
            if (env->ota_state.dfu_ID != DFU_ID_OTA_MANAGER)
            {
                if (env->is_reboot_timer_on == 1)
                {
                    env->is_reboot_timer_on = 0;
                    os_timer_stop(g_dfu_timer);
                    os_timer_delete(g_dfu_timer);
                }
            }
            /* Save current status to flash. */
            dfu_ctrl_update_prog_info(env);

            // dfu_set_reboot_after_disconnect();

            int ret = dfu_protocol_session_close();
            if (ret == -1)
            {
                dfu_port_svc_session_close();
            }
#endif
        }
        else
        {
            dfu_ctrl_update_prog_info(env);
        }

    }
    break;
    default:
        LOG_I("dfu_init_completed_handler_ext %d", state);
        if (env->ota_state.dfu_ID != DFU_ID_OTA_MANAGER)
        {
            env->prog.state = DFU_CTRL_PREPARE_START;
            dfu_ctrl_update_prog_info(env);
        }
        break;
    }
}

/* Check contrl packet whether invalidate. */
static void dfu_resume_request_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
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
        if (dfu_ctrl_ctrl_header_sig_verify(env, packet, len, sig) < 0)
        {
            status = DFU_ERR_CONTROL_PACKET_INVALID;
            break;
        }

        uint8_t dfu_ID = *packet;
        switch (state)
        {
        case DFU_CTRL_TRAN_START:
            dfu_link_sync_end();
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
            status = DFU_ERR_UNEXPECT_STATE;
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
                        dl_hdr->curr_img_info.img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
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
        LOG_I("dfu_resume_response %d", rsp->result);
        dfu_protocol_packet_send((uint8_t *)rsp);
    }
}


static void dfu_resume_completed_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
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

#ifdef OTA_ERROR_HANDLE_IN_USR
        if (env->is_force_update == 1)
            env->prog.state = DFU_CTRL_PREPARE_START_FORCE;
        else
#endif
            env->prog.state = DFU_CTRL_PREPARE_START;

        if (env->mode == DFU_CTRL_NORMAL_MODE)
        {
            /* Save current status to flash. */
            dfu_ctrl_update_prog_info(env);

            dfu_set_reboot_after_disconnect();
            int ret = dfu_protocol_session_close();
            if (ret == -1)
            {
                dfu_port_svc_session_close();
            }

            // should reboot after disconnect
            /* Prepare reboot device. */
            // HAL_PMU_Reboot();
        }
        else
        {
            /* W4 image send start in OTA mode. */
        }
    }
    break;
    default:
        break;
    }

}

static void dfu_img_send_start_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_image_send_start_t *s_data = (dfu_image_send_start_t *)data;
    dfu_img_info_t *curr_img = &env->prog.fw_context.code_img.curr_img_info;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    LOG_I("dfu_img_send_start_handler %d, %d, %d, %d", s_data->img_id, state, env->prog.FW_state, curr_img->img_state);
    uint16_t status = DFU_ERR_GENERAL_ERR;
    uint8_t image_retrans = DFU_IMAGE_RETRANS_NONE;
    int erase_ret = 0;

    do
    {
        // update ota manager
        if (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.state == OTA_STATE_PREPARE)
        {
            curr_img = &env->ota_state.fw_context.code_img.curr_img_info;
            if (!check_current_img_is_normal_mode_id(s_data->img_id))
            {
                break;
            }

#ifdef DFU_USING_DOWNLOAD_BACKUP
#ifdef DFU_USING_BACK_UP_PSRAM
#ifdef OTA_USING_SOL_FUNC
            if (!ota_manager_download_buffer)
            {
                uint32_t buffer_size = DFU_OTA_MANAGER_COM_SIZE + DFU_BOOTLOADER_COM_SIZE;
                LOG_I("alloc size %d for ota manager buff", buffer_size);
                ota_manager_download_buffer = app_cache_alloc(buffer_size, IMAGE_CACHE_PSRAM);
            }

            OS_ASSERT(ota_manager_download_buffer);
#else
            LOG_E("enable sol fun to alloc psram");
            OS_ASSERT(0);
#endif
#endif
#else
            LOG_E("can not update ota manager without backup");
            OS_ASSERT(0);
#endif

            LOG_I("dfu_img_send_start_handler update ota");
            // clear ota state as we are going to use hcpu download flash
            env->prog.state = DFU_CTRL_IDLE;
            env->prog.FW_state = DFU_CTRL_FW_NO_STATE;

            dfu_app_img_dl_start_ind_t ota_ind;
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
            if (!check_current_img_is_normal_mode_id(curr_img->img_id))
            {
                LOG_W("only ota manager is allowed");
                return;
            }

            curr_img->header = dfu_img_get_img_header_by_img_id(env, curr_img->img_id);
            curr_img->img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
            OS_ASSERT(curr_img->header);
            dfu_clear_storage_data(curr_img->header, 0, curr_img->header->length);
            env->ota_state.state = OTA_STATE_DOWNLOADING;

            status = DFU_ERR_NO_ERR;
            dfu_image_send_start_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_START_RESPONSE, dfu_image_send_start_response_t);
            rsp->result = status;
            rsp->end_send = g_end_send_mode;
            dfu_protocol_packet_send((uint8_t *)rsp);

            dfu_app_img_dl_start_ind_t ind;
            ind.total_imgs_num = env->ota_state.fw_context.code_img.img_count;
            ind.curr_img_id = s_data->img_id;
            ind.curr_img_total_len = s_data->img_length;

            env->sync_size = 0;
            dfu_link_sync_start(DFU_SYNC_TYPE_DOWNLOAD);

            if (env->callback)
                env->callback(DFU_APP_IMAGE_DL_START_IND, &ind);
            return;
        }

#ifndef OTA_NOR_OTA_MANAGER_LITE
        if (env->mode != DFU_CTRL_OTA_MODE)
            break;
#else
        if (env->dfu_flash_thread == NULL)
        {
            env->dfu_flash_thread = ble_dfu_flash_write_thread_start();
        }
#endif

        if (s_data->img_id > DFU_IMG_ID_MAX)
            break;

        LOG_I("dfu_img_send_start_handler mode %d", dfu_get_backup_mode());
        LOG_I("dfu_img_send_start_handler image state %d", env->prog.image_state[s_data->img_id]);

#ifndef OTA_NOR_OTA_MANAGER_LITE
        if (dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM && env->resume_status == 1)
        {
            if (env->prog.image_state[s_data->img_id] == DFU_IMAGE_STATE_RESET_TO_PSRAM)
            {
                image_retrans = DFU_IMAGE_RETRANS_NONE;
                LOG_I("PSRAM content, need download again");
                if (env->prog.FW_state == DFU_CTRL_FW_DOWNLOADING)
                {
                    if (state == DFU_CTRL_NEG || state == DFU_CTRL_PREPARE_START)
                    {
                        env->prog.FW_state = DFU_CTRL_FW_NO_STATE;
                        LOG_I("dfu_img_send_start_handler clear state");
                    }
                }
            }
            else
            {
                if (env->prog.image_state[s_data->img_id] == DFU_IMAGE_STATE_DOWNLOAD_FINISH_OVERWRITE)
                {
                    //LOG_D("resume packet %d, %d", env->resume_image_index, s_data->total_pkt_num);
                    //LOG_D("resume id %d, %d", env->resume_image_id, s_data->img_id);
                    if (env->resume_image_index == s_data->total_pkt_num && env->resume_image_id == s_data->img_id)
                    {
                        LOG_I("resume last packet, not skip");
                        // resume last packet
                        env->prog.image_state[s_data->img_id] = DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE;
                    }
                    else
                    {
                        image_retrans = DFU_IMAGE_RETRANS_SKIP;
                        LOG_I("flash or overwrite complete content, skip");
                        if (state == DFU_CTRL_NEG || state == DFU_CTRL_PREPARE_START)
                        {
                            state = DFU_CTRL_TRAN_START;
                        }
                    }
                }

                if (env->prog.image_state[s_data->img_id] == DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE)
                {
                    if (temp_header == NULL)
                    {
                        status = DFU_ERR_INTERNAL;
                        break;
                    }

                    if (s_data->img_id != temp_header->curr_img_info.img_id)
                    {
                        LOG_E("not target image, send from beginning");
                        if (state == DFU_CTRL_NEG || state == DFU_CTRL_PREPARE_START)
                        {
                            state = DFU_CTRL_TRAN_START;
                        }
                    }
                    else
                    {
                        LOG_I("process resume flash downloading");

                        memcpy(&env->prog.fw_context.code_img, temp_header, sizeof(dfu_dl_image_header_t));
                        if (state == DFU_CTRL_TRAN_START)
                        {
                            LOG_I("update state");
                            state = DFU_CTRL_PREPARE_START;
                        }
                        free(temp_header);
                        temp_header = NULL;
                    }
                }
            }
        }
#endif
        LOG_I("dfu_img_send_start_handler state %d", state);

        switch (state)
        {
        case DFU_CTRL_NEG: // for resume in dfu mode
        case DFU_CTRL_PREPARE_START:
#ifdef OTA_ERROR_HANDLE_IN_USR
        case DFU_CTRL_PREPARE_START_FORCE:
#endif
        {
            dfu_app_img_dl_start_ind_t ind;
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
                curr_img->header = dfu_img_get_img_header_by_img_id(env, curr_img->img_id);
                curr_img->img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
                OS_ASSERT(curr_img->header);
                dfu_image_set_download_begin(env, curr_img->header);
                erase_ret = dfu_clear_storage_data(curr_img->header, 0, curr_img->header->length);
                env->prog.FW_state = DFU_CTRL_FW_DOWNLOADING;
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
            rt_memcpy(dfu_temp_key, aes_out, DFU_KEY_SIZE);

            rt_free(aes_in);
            rt_free(aes_out);

            env->prog.state = DFU_CTRL_TRAN_START;

            if (erase_ret == ERASE_PROCESSING)
            {
                return;
            }

            ind.total_imgs_num = env->prog.fw_context.code_img.img_count;
            ind.curr_img_id = s_data->img_id;
            ind.curr_img_total_len = s_data->img_length;

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
                env->callback(DFU_APP_IMAGE_DL_START_IND, &ind);

            if (env->is_force_update && curr_img->img_id == DFU_IMG_ID_HCPU)
            {
                // do nothing so will not reboot in ota mode download hcpu.
            }
            else
            {
                dfu_ctrl_update_prog_info(env);
            }

            if (curr_img->img_id > DFU_IMG_ID_PATCH)
            {
                env->prog.ota_mode = 2;
            }

            status = DFU_ERR_NO_ERR;
        }
        break;
        case DFU_CTRL_TRAN_START:
        {
            if (image_retrans == DFU_IMAGE_RETRANS_SKIP)
            {
                status = DFU_ERR_NO_ERR;
                break;
            }
            dfu_app_img_dl_start_ind_t ind;
            /* More than one image. */
            curr_img->img_id = s_data->img_id;
            curr_img->img_length = s_data->img_length;
            curr_img->img_info.dl_info.num_of_rsp = s_data->num_of_rsp;
            curr_img->img_info.dl_info.total_pkt_num = s_data->total_pkt_num;
            curr_img->header = dfu_img_get_img_header_by_img_id(env, curr_img->img_id);
            curr_img->img_state = DFU_CTRL_IMG_STATE_DOWNLOADING;
            curr_img->img_info.dl_info.curr_pkt_num = 0;
            curr_img->img_info.dl_info.curr_img_length = 0;
            dfu_image_set_download_begin(env, curr_img->header);
            dfu_ctrl_update_prog_info(env);
            erase_ret = dfu_clear_storage_data(curr_img->header, 0, curr_img->header->length);
            if (erase_ret == ERASE_PROCESSING)
            {
                return;
            }

            ind.total_imgs_num = env->prog.fw_context.code_img.img_count;
            ind.curr_img_id = s_data->img_id;
            ind.curr_img_total_len = s_data->img_length;

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
                env->callback(DFU_APP_IMAGE_DL_START_IND, &ind);

            if (curr_img->img_id > DFU_IMG_ID_PATCH)
            {
                env->prog.ota_mode = 2;
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
    rsp->end_send = g_end_send_mode;
    rsp->extra = image_retrans;
    dfu_protocol_packet_send((uint8_t *)rsp);

    if (status != DFU_ERR_NO_ERR)
    {
        LOG_E("dfu_img_send_start_handler rsp error: %d", status);

        if (check_current_img_need_force_update(curr_img->img_id))
        {
            env->is_force = 1;
        }
        dfu_ctrl_error_handle(env);
    }
    else
    {
        ble_dfu_request_connection_priority();
    }
}

static void dfu_flash_exit_msg_send()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (env->mb_handle)
    {
        flash_write_t *fwrite;
        fwrite = rt_malloc(sizeof(flash_write_t));
        OS_ASSERT(fwrite);
        fwrite->msg_type = DFU_FLASH_MSG_TYPE_EXIT;
        rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
    }
}

static void dfu_verification_handle()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    uint8_t status = DFU_ERR_GENERAL_ERR;

    status = dfu_img_verification(env);
    if (status == DFU_ERR_NO_ERR)
    {
        curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED;
        dfu_app_img_dl_completed_ind_t ind;
        ind.img_id = curr_info->img_id;
        if (env->callback)
            env->callback(DFU_APP_IMAGE_DL_COMPLETED_IND, (void *)&ind);
    }
    else
    {
        curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL;
    }

    if (status == DFU_ERR_NO_ERR)
    {
        env->prog.image_download_state[curr_info->img_id] = 1;
    }

    LOG_I("dfu_img_send_end response mb");
    dfu_image_send_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_END_RESPONSE, dfu_image_send_end_response_t);
    rsp->result = status;
    dfu_protocol_packet_send((uint8_t *)rsp);

    //TODO error handle for packet err
    if (status != DFU_ERR_NO_ERR)
    {
        LOG_E("dfu_img_send_end_handler rsp error: %d", status);

        if (check_current_img_need_force_update(curr_info->img_id))
        {
            env->is_force = 1;
        }
        dfu_ctrl_error_handle(env);
    }
    else
    {
        dfu_image_set_download_end(env, curr_info->img_id);
    }
}

static void ble_dfu_flash_write()
{
    LOG_I("ble_dfu_flash_write: try to recv a mail");
    int thread_run = 1;

    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
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
                //LOG_I("ble_dfu_flash_write id %d, flag %d, OFFSET %d, SIZE %d, addr 0x%x", fwrite->heade.img_id, fwrite->heade.img_id,fwrite->offset, fwrite->size, fwrite);
                ret = dfu_write_storage_data(fwrite->heade, fwrite->offset, fwrite->data, fwrite->size);
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
    env->dfu_flash_thread = NULL;
}

static rt_thread_t ble_dfu_flash_write_thread_start()
{
    LOG_I("ble_dfu_flash_write_thread_start");
    rt_thread_t tid;
    tid = rt_thread_create("ble_dfu_flash", ble_dfu_flash_write, NULL, 4096, RT_MAIN_THREAD_PRIORITY + 5, 10);
    rt_thread_startup(tid);
    return tid;
}

static int dfu_packet_download(dfu_ctrl_env_t *env, uint8_t img_id, uint8_t *data, int size)
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
        d = dfu_dec_verify(dfu_temp_key, hdr->offset, data, dfu_temp, size, hdr->hash);
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
                if (mb_ret != RT_EOK)
                {
                    LOG_I("MB RET %d", mb_ret);
                    OS_ASSERT(0);
                }
            }
            else
            {
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
                if (mb_ret != RT_EOK)
                {
                    LOG_I("MB RET %d", mb_ret);
                    OS_ASSERT(0);
                }
            }
            else
            {
                r = dfu_write_storage_data(img_hdr, hdr->offset, data, size);
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

static int dfu_packet_download_ota_manager(dfu_ctrl_env_t *env, uint8_t img_id, uint8_t *data, int size)
{
    int r = -1;
    struct image_body_hdr *hdr = (struct image_body_hdr *)data;
    uint8_t key[DFU_KEY_SIZE] = {0};
    uint8_t *d = NULL;
    dfu_image_header_int_t *img_hdr = env->ota_state.fw_context.code_img.curr_img_info.header;

    size -= sizeof(struct image_body_hdr);
    data += sizeof(struct image_body_hdr);

    //sifli_hw_dec_key(env->prog.FW_key, key, DFU_KEY_SIZE);

    if (img_hdr->flag & DFU_FLAG_ENC)
    {
        uint8_t *dfu_temp;
        dfu_temp = rt_malloc(DFU_MAX_BLK_SIZE);
        OS_ASSERT(dfu_temp);
        d = dfu_dec_verify(dfu_temp_key, hdr->offset, data, dfu_temp, size, hdr->hash);
        if (d)
        {
            r = dfu_write_storage_data(img_hdr, hdr->offset, data, size);
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
            r = dfu_write_storage_data(img_hdr, hdr->offset, data, size);
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


uint8_t dfu_get_progress_mode()
{
    return g_dfu_progress_mode;
}

static void dfu_img_send_packet_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
    dfu_image_send_packet_t *packet = (dfu_image_send_packet_t *)data;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    uint16_t status = DFU_ERR_GENERAL_ERR;
    do
    {
        if (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.state == OTA_STATE_DOWNLOADING)
        {
            status = DFU_ERR_NO_ERR;

            curr_info = &env->ota_state.fw_context.code_img.curr_img_info;
            //LOG_I("update ota manager packet count(%d), len(%d)\r\n", packet->pkt_idx, packet->size);
            int ret;
            if (curr_info->img_length < curr_info->img_info.dl_info.curr_img_length + packet->size)
            {
                break;
            }
            else if ((curr_info->img_info.dl_info.curr_pkt_num + 1) != packet->pkt_idx)
            {
                LOG_W("error curr count(%d)\r\n", curr_info->img_info.dl_info.curr_pkt_num);

                if (curr_info->img_info.dl_info.num_of_rsp != 0)
                {
                    // add for ios, so link will not disconnect,
                    // retransmission can still work.
                    LOG_W("stay link \r\n");
                    dfu_link_lose_check_req(curr_info->img_info.dl_info.curr_pkt_num, curr_info->img_info.dl_info.num_of_rsp);
                    dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
                    status = DFU_ERR_NO_ERR;
                }
                break;
            }
            //else if (packet->size > env->prog.fw_context.code_img.blk_size)
            //break;

            ret = dfu_packet_download_ota_manager(env, curr_info->img_id, packet->packet, packet->size);

            if (ret != 0)
            {
                status = DFU_ERR_GENERAL_ERR;
                dfu_image_send_packet_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_PACKET_RESPONSE, dfu_image_send_packet_response_t);
                rsp->result = status;
                dfu_protocol_packet_send((uint8_t *)rsp);
                return;
            }

            curr_info->img_info.dl_info.curr_pkt_num++;
            curr_info->img_info.dl_info.curr_img_length += packet->size;

            dfu_app_img_dl_progress_ind_t ind;
            ind.img_id = curr_info->img_id;
            ind.curr_img_recv_length = curr_info->img_info.dl_info.curr_img_length;
            if (env->callback)
            {
                env->callback(DFU_APP_IMAGE_DL_ROPGRESS_IND, &ind);
            }

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
            return;
        }

#ifndef OTA_NOR_OTA_MANAGER_LITE
        if (env->mode != DFU_CTRL_OTA_MODE)
            break;
#endif

        if (packet->img_id != curr_info->img_id)
        {
            status = DFU_ERR_PARAMETER_INVALID;
            break;
        }

        switch (state)
        {
        case DFU_CTRL_TRAN_START:
        {
            dfu_app_img_dl_progress_ind_t ind;

            LOG_D("packet count(%d), len(%d)\r\n", packet->pkt_idx, packet->size);
            int ret;
            if (curr_info->img_length < curr_info->img_info.dl_info.curr_img_length + packet->size)
            {
                break;
            }
            else if ((curr_info->img_info.dl_info.curr_pkt_num + 1) != packet->pkt_idx)
            {
                LOG_W("error curr count(%d)\r\n", curr_info->img_info.dl_info.curr_pkt_num);
                dfu_link_lose_check_req(curr_info->img_info.dl_info.curr_pkt_num, curr_info->img_info.dl_info.num_of_rsp);
                dfu_link_sync_start(DFU_SYNC_TYPE_RSP);
                status = DFU_ERR_NO_ERR;
                break;
            }
            //else if (packet->size > env->prog.fw_context.code_img.blk_size)
            //break;

            ret = dfu_packet_download(env, curr_info->img_id, packet->packet, packet->size);

            if (ret != 0)
            {
                LOG_E("dfu_packet_download fail!");
                break;
            }

            status = DFU_ERR_NO_ERR;
            curr_info->img_info.dl_info.curr_pkt_num++;
            curr_info->img_info.dl_info.curr_img_length += packet->size;

            ind.img_id = curr_info->img_id;
            ind.curr_img_recv_length = curr_info->img_info.dl_info.curr_img_length;

            if (g_dfu_progress_mode == DFU_PROGRESS_TOTAL)
            {
                ind.curr_img_recv_length = ind.curr_img_recv_length + env->transported_size - curr_info->img_info.dl_info.curr_pkt_num * sizeof(struct image_body_hdr);
            }

            env->callback(DFU_APP_IMAGE_DL_ROPGRESS_IND, &ind);


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
        LOG_I("dfu img packet status %d", status);
        if (check_current_img_need_force_update(curr_info->img_id))
        {
            env->is_force = 1;
        }
        dfu_ctrl_error_handle(env);
    }
}


static void dfu_img_send_end_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
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
        if (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.state == OTA_STATE_DOWNLOADING)
        {

            curr_info = &env->ota_state.fw_context.code_img.curr_img_info;
            status = dfu_img_verification(env);

            if (status == DFU_ERR_NO_ERR)
            {
                curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED;
                env->ota_state.state = OTA_STATE_DOWNLOADED;

                if (packet->is_more_image)
                {
                    LOG_I("more image");
                    env->ota_state.state = OTA_STATE_PREPARE;
                }

                dfu_app_img_dl_completed_ind_t ind;
                ind.img_id = curr_info->img_id;
                if (env->callback)
                    env->callback(DFU_APP_IMAGE_DL_COMPLETED_IND, (void *)&ind);
            }
            else
            {
                curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL;
            }


            dfu_image_send_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_END_RESPONSE, dfu_image_send_end_response_t);
            rsp->result = status;
            dfu_protocol_packet_send((uint8_t *)rsp);
            return;
        }
#ifndef OTA_NOR_OTA_MANAGER_LITE
        if (env->mode != DFU_CTRL_OTA_MODE)
            break;
#endif
        if (packet->img_id != curr_info->img_id)
        {
            status = DFU_ERR_PARAMETER_INVALID;
            break;
        }

        switch (state)
        {
        case DFU_CTRL_TRAN_START:
        {
#ifdef OTA_NOR_OTA_MANAGER_LITE111
            // temp debug
            if (env->mode != DFU_CTRL_OTA_MODE)
            {
                status = DFU_ERR_NO_ERR;
                break;
            }
#else
            if (env->mb_handle)
            {
                flash_write_t *fwrite;
                fwrite = rt_malloc(sizeof(flash_write_t));
                OS_ASSERT(fwrite);
                fwrite->msg_type = DFU_FLASH_MSG_TYPE_VERIFY;

                rt_mb_send(env->mb_handle, (rt_uint32_t)fwrite);
                return;
            }

            status = dfu_img_verification(env);
            if (status == DFU_ERR_NO_ERR)
            {
                curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED;
                dfu_app_img_dl_completed_ind_t ind;
                ind.img_id = curr_info->img_id;
                if (env->callback)
                    env->callback(DFU_APP_IMAGE_DL_COMPLETED_IND, (void *)&ind);
            }
            else
                curr_info->img_state = DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL;
            break;
#endif
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

    dfu_image_send_end_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_IMAGE_SEND_END_RESPONSE, dfu_image_send_end_response_t);
    rsp->result = status;
    dfu_protocol_packet_send((uint8_t *)rsp);

    //TODO error handle for packet err
    if (status != DFU_ERR_NO_ERR)
    {
        LOG_E("dfu_img_send_end_handler rsp error: %d", status);

        if (check_current_img_need_force_update(curr_info->img_id))
        {
            env->is_force = 1;
        }
        dfu_ctrl_error_handle(env);
    }
    else
    {
        dfu_image_set_download_end(env, curr_info->img_id);
    }

}

void dfu_ctrl_last_packet_handler()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
#ifdef OTA_NOR_OTA_MANAGER_LITE
    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        //HAL_PMU_Reboot();
        dfu_app_img_install_start_ind_t ind_over;
        ind_over.total_imgs_len = 0;
        if (env->callback)
        {
            env->callback(DFU_APP_DL_END_AND_INSTALL_START_IND, &ind_over);
        }
        return;
    }
#endif

    if (env->prog.state != DFU_CTRL_INSTALL)
    {
        LOG_E("fail to handle last packet due to state error %d", env->prog.state);

        dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
        for (uint32_t i = 0; i < dl_header->img_count; i++)
        {
            dfu_image_set_download_end_fail(env, dl_header->img_header[i].img_id);
        }
        return;
    }
    uint8_t status = DFU_SUCCESS;
    dfu_ctrl_install_completed(env, status);
}

uint8_t dfu_offline_install_set(uint32_t ctrl_addr, uint32_t ctrl_len)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    // ctrl packet data
    uint8_t *ctrl_data;
    uint8_t *data;

    // ctrl packet length
    uint32_t len = ctrl_len;
    ctrl_data = rt_malloc(len);
    OS_ASSERT(ctrl_data);
    dfu_flash_read(ctrl_addr, ctrl_data, len);

    data = ctrl_data;

    uint8_t status = DFU_ERR_GENERAL_ERR;

    struct image_cfg_hdr *hdr = (struct image_cfg_hdr *)data;
    data += sizeof(struct image_cfg_hdr);
    len -= sizeof(struct image_cfg_hdr);
    uint8_t *sig = malloc(DFU_SIG_SIZE);
    memcpy(sig, data + len - DFU_SIG_SIZE, DFU_SIG_SIZE);
    uint8_t *packet = (uint8_t *)dfu_dec_verify(NULL, 0, data, data, len - DFU_SIG_SIZE, hdr->hash);

    if (!packet)
    {
        LOG_W("Ctrl packet parser failed!");
        rt_free(ctrl_data);
        return status;
    }

    if (dfu_ctrl_ctrl_header_sig_verify(env, packet, len, sig) < 0)
    {
        status = DFU_ERR_CONTROL_PACKET_INVALID;
        LOG_W("Ctrl packet incalid!");
        rt_free(ctrl_data);
        return status;
    }

    dfu_control_packet_t *ctrl_packet = dfu_ctrl_ctrl_header_alloc(data, len - DFU_SIG_SIZE);

    status = dfu_ctrl_packet_check(env, ctrl_packet, len);
    if (status != DFU_ERR_NO_ERR)
    {
        LOG_W("ctrl packet check");
        rt_free(ctrl_data);
        return status;
    }

    dfu_code_image_header_t *header = (dfu_code_image_header_t *)&ctrl_packet->image_header;
    env->prog.dfu_ID = ctrl_packet->dfu_ID;

    uint8_t *aes_out;
    aes_out = rt_malloc(DFU_KEY_SIZE);
    rt_memset(aes_out, 0, DFU_KEY_SIZE);
    sifli_hw_enc(ctrl_packet->FW_key, aes_out, DFU_KEY_SIZE);
    rt_memcpy(env->prog.FW_key, aes_out, DFU_KEY_SIZE);
    rt_free(aes_out);

    env->prog.FW_version = ctrl_packet->FW_version;
    env->prog.SDK_version = ctrl_packet->SDK_version;
    env->prog.HW_version = ctrl_packet->HW_version;
    env->prog.fw_context.code_img.blk_size = header->blk_size;
    env->prog.fw_context.code_img.img_count = header->img_count;
    rt_memcpy((uint8_t *)&env->prog.fw_context.code_img.img_header, (uint8_t *)&header->img_header, sizeof(dfu_image_header_int_t) * header->img_count);

    LOG_I("offline install set");
    env->prog.state = DFU_CTRL_OFFLINE_INSTALL_PREPARE;
    dfu_ctrl_ctrl_header_free((uint8_t *)ctrl_packet);
    rt_free(ctrl_data);
    return status;
}

void dfu_offline_reboot()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    if (env->prog.state != DFU_CTRL_OFFLINE_INSTALL_PREPARE)
    {
        LOG_I("dfu_offline_reboot, not expect state %d", env->prog.state);
        return;
    }

    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        // old flash backup
        //env->prog.hcpu_overwrite_enable = 0;
        //LOG_D("ota hcpu overwrite %d", env->prog.hcpu_overwrite_enable);

        // new backup set
        dfu_set_backup_mode();
        LOG_D("ota backup mode %d", dfu_get_backup_mode());

#ifdef OTA_USING_SOL_FUNC
        app_power_off_action(POWER_OFF_BY_DFU);
#else
        HAL_PMU_Reboot();
#endif
    }
}

static dfu_offline_install_packet_t *dfu_offline_header_alloc(uint32_t data_addr)
{
    //uint32_t data_addr = DFU_RES_FLASH_CODE_START_ADDR;

    uint32_t info_len;
    uint8_t image_count;

    dfu_flash_read(data_addr + sizeof(uint32_t) + sizeof(uint8_t), (uint8_t *)&info_len, sizeof(uint32_t));
    dfu_flash_read(data_addr + sizeof(uint8_t), (uint8_t *)&image_count, sizeof(uint8_t));

    LOG_I("dfu_offline_header_alloc image_count info_len %d, %d", image_count, info_len);

    dfu_offline_install_packet_t *packet = malloc(sizeof(dfu_offline_install_packet_t) + sizeof(dfu_offline_image_info_t) * image_count);

    uint8_t *p_data = malloc(sizeof(dfu_offline_install_packet_t) + info_len);

    uint8_t *data = p_data;

    OS_ASSERT(p_data);
    OS_ASSERT(packet);

    dfu_flash_read(data_addr, data, sizeof(dfu_offline_install_packet_t) + info_len);

    memcpy((uint8_t *)&packet->magic, data, sizeof(uint32_t));
    data += sizeof(uint32_t);

    if (packet->magic != SEC_CONFIG_MAGIC)
    {
        LOG_E("magic error");
        free(p_data);
        free(packet);
        return NULL;
    }

    memcpy((uint8_t *)&packet->image_count, data, sizeof(uint8_t));
    data += sizeof(uint8_t);

    memcpy((uint8_t *)&packet->info_len, data, sizeof(uint32_t));
    data += sizeof(uint32_t);

    dfu_offline_image_info_t *image_info = (dfu_offline_image_info_t *)(&packet->image_info);

    uint32_t offset = 9 + info_len;
    for (uint8_t i = 0; i < packet->image_count; i++)
    {
        memcpy((uint8_t *)&image_info[i].id, data, sizeof(uint8_t));
        data += sizeof(uint8_t);

        memcpy((uint8_t *)&image_info[i].len, data, sizeof(uint32_t));
        data += sizeof(uint32_t);

        image_info[i].offset = offset;
        offset += image_info[i].len;
    }

    /*
        for (uint8_t i = 0; i < packet->image_count; i++)
        {
            // DEBUG
            LOG_I("count %d, id %d, offset %d, len %d", i, image_info[i].id, image_info[i].offset, image_info[i].len);
        }
    */
    free(p_data);

    return packet;
}

static void dfu_offline_header_free(uint8_t *data)
{
    free(data);
}

static void dfu_reboot_install()
{
    LOG_I("dfu_reboot_install");
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->prog.hcpu_available = DFU_IMAGE_AVAILABLE_NONE;
    dfu_ctrl_update_prog_info(env);
    //HAL_sw_breakpoint();

    int ret = dfu_img_install(env);
    //int ret = DFU_ERR_NO_ERR;
    if (ret == DFU_ERR_NO_ERR)
    {
        dfu_ota_bootloader_ram_run_set(DFU_RAM_STATE_UPDATED);
        env->prog.state = DFU_CTRL_UPDATING;
        env->prog.FW_state = DFU_CTRL_FW_INSTALLED;
        env->is_force_update = 0;
        dfu_flash_addr_reset(0);
        dfu_ctrl_update_prog_info(env);
        dfu_update_img_header(env);
        LOG_I("dfu_ctrl_boot_to_user_fw");
        env->prog.hcpu_available = DFU_IMAGE_AVAILABLE;
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 1);
        drv_reboot();
        //dfu_ctrl_boot_to_user_fw();
    }
    else
    {
        env->is_force = 1;
        dfu_ctrl_error_handle(env);
        // HAL_PMU_Reboot();
    }
}

static void dfu_offline_install()
{
    LOG_I("dfu_offline_install");
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    uint32_t download_base = DFU_RES_FLASH_CODE_START_ADDR;

    dfu_offline_install_packet_t *packet = dfu_offline_header_alloc(download_base);
    if (!packet)
    {
        LOG_I("alloc failed");
        return;
    }

    dfu_offline_image_info_t *image_info = (dfu_offline_image_info_t *)(&packet->image_info);

    for (uint8_t i = 0; i < packet->image_count; i++)
    {
        //LOG_I("count %d, id %d, offset %d, len %d", i, image_info[i].id, image_info[i].offset, image_info[i].len);
        if (image_info[i].id == DFU_IMG_ID_HCPU)
        {
            LOG_I("hcpu offset set %d", image_info[i].offset);
            set_image_offset(image_info[i].offset);
        }
    }
    dfu_offline_header_free((uint8_t *)packet);

    // HAL_sw_breakpoint();

    int ret = dfu_img_install(env);

    LOG_I("dfu_img_install %d", ret);

    if (ret == DFU_ERR_NO_ERR)
    {
        env->prog.state = DFU_CTRL_UPDATING;
        env->prog.FW_state = DFU_CTRL_FW_INSTALLED;
        env->is_force_update = 0;
        dfu_flash_addr_reset(0);
        dfu_ctrl_update_prog_info(env);
        dfu_update_img_header(env);
        LOG_I("dfu_ctrl_boot_to_user_fw");
        dfu_ctrl_boot_to_user_fw();
    }
    else
    {
        env->is_force = 1;
        dfu_ctrl_error_handle(env);
        HAL_PMU_Reboot();
    }
}

void dfu_offline_install_start()
{
    LOG_I("dfu_offline_install_start");
    uint32_t download_base = DFU_RES_FLASH_CODE_START_ADDR;
    dfu_offline_install_packet_t *packet = dfu_offline_header_alloc(download_base);
    if (!packet)
    {
        LOG_I("alloc failed");
        return;
    }

    dfu_offline_image_info_t *image_info = (dfu_offline_image_info_t *)(&packet->image_info);
    for (uint8_t i = 0; i < packet->image_count; i++)
    {
        //LOG_I("count %d, id %d, offset %d, len %d", i, image_info[i].id, image_info[i].offset, image_info[i].len);
        if (image_info[i].id == DFU_IMG_ID_CTRL_PACKET)
        {
            LOG_I("offline ctrl check");
            dfu_offline_install_set(download_base + image_info[i].offset, image_info[i].len);
        }
    }
    dfu_offline_header_free((uint8_t *)packet);

    dfu_offline_reboot();
}

static uint8_t dfu_image_download_complete_check(dfu_ctrl_env_t *env)
{
    uint8_t download_count = 0;
    dfu_dl_image_header_t *dl_header = &env->prog.fw_context.code_img;
    uint8_t ota_manager_download = 0;

    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        LOG_I("dfu_image_download_complete_check %d, %d", i, env->prog.image_download_state[i]);

        if (env->prog.image_download_state[DFU_IMG_ID_OTA_MANAGER] == 1)
        {
            ota_manager_download = 1;
        }
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


#ifdef OTA_NOR_OTA_MANAGER_LITE
    if (ota_manager_download == 0)
    {
        return DFU_ERR_NOT_READY;
    }
#endif

    if (download_count == dl_header->img_count)
    {
        return DFU_ERR_NO_ERR;
    }
    else
    {
        return DFU_ERR_PARAMETER_INVALID;
    }
}

static void dfu_img_tramission_end_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_img_tramission_end_handler");
    DFU_ERR_CHECK(env && data);
    uint16_t state = env->prog.state;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    dfu_tranmission_end_t *packet = (dfu_tranmission_end_t *)data;
    uint16_t status = DFU_ERR_GENERAL_ERR;

#ifdef OTA_NOR_OTA_MANAGER_LITE
    if (env->mode == DFU_CTRL_NORMAL_MODE &&
            (env->prog.dfu_ID == DFU_ID_CODE || env->prog.dfu_ID == DFU_ID_CODE_MIX))
    {
        env->prog.state = DFU_CTRL_REBOOT_INSTALL_PREPARE;
        dfu_ctrl_update_prog_info(env);
        status = DFU_ERR_NO_ERR;

        status = dfu_image_download_complete_check(env);

        if (status == DFU_ERR_NO_ERR)
        {
            dfu_record_current_tx();
        }

        dfu_end_int_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_END_IND, dfu_end_int_t);
        rsp->result = status;
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (status == DFU_ERR_NO_ERR)
        {
            dfu_ota_bootloader_ram_run_set(DFU_RAM_STATE_UPDATING);
        }
        else
        {
            dfu_ctrl_error_handle(env);
        }

        if (g_end_send_mode == DFU_END_SEND)
        {
            if (status != DFU_ERR_NO_ERR)
            {
                // install fail, no need to wait;
            }
            else
            {
                if (dfu_set_last_packet_wait() == 0)
                {
                    // wait last packet send then we process install completed
                }
                else
                {
                    // last packet already send, process now
                    LOG_I("dfu_img_tramission_end, reboot");
                    //HAL_PMU_Reboot();
                    dfu_app_img_install_start_ind_t ind_over;
                    ind_over.total_imgs_len = 0;
                    if (env->callback)
                    {
                        env->callback(DFU_APP_DL_END_AND_INSTALL_START_IND, &ind_over);
                    }
                    return;
                }
            }
        }
        else
        {
            //HAL_PMU_Reboot();
        }
    }
#else

    if (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.state == OTA_STATE_DOWNLOADED)
    {
        int ret;
        env->ota_state.state = OTA_STATE_INTALLING;
        LOG_I("dfu_img_tramission_end_handler ota");

        dfu_app_img_install_start_ind_t ind_over;
        ind_over.total_imgs_len = 0;
        if (env->callback)
        {
            env->callback(DFU_APP_DL_END_AND_INSTALL_START_IND, &ind_over);
        }

        ret = dfu_img_install(env);
        LOG_I("INSTALL RET %d", ret);
        if (ret == DFU_SUCCESS)
        {
            status = DFU_ERR_NO_ERR;
            fdb_kv_set_default(p_dfu_db);
            env->ota_state.state = OTA_STATE_INSTALLED;
        }
        else
        {
            status = DFU_ERR_FW_INVALID;
        }

        dfu_app_img_install_completed_ind_t ind;
        ind.result = status == DFU_ERR_NO_ERR ? DFU_APP_SUCCESSED : DFU_APP_FAIED;
        if (env->callback)
        {
            env->callback(DFU_APP_INSTALL_COMPLETD_IND, &ind);
        }

#ifdef OTA_USING_SOL_FUNC
        if (ota_manager_download_buffer)
        {
            app_cache_free(ota_manager_download_buffer);
        }
        ota_manager_download_buffer = NULL;
#endif

        dfu_end_int_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_END_IND, dfu_end_int_t);
        rsp->result = status;
        dfu_protocol_packet_send((uint8_t *)rsp);
        return;
    }

    switch (state)
    {
    case DFU_CTRL_TRAN_START:
    {
        dfu_app_img_install_start_ind_t ind;
        int ret;
        /* Enter install mode. */
        env->prog.state = DFU_CTRL_INSTALL;
        env->prog.FW_state = DFU_CTRL_FW_INSTALLING;
        env->is_force = 1;
        dfu_ctrl_update_prog_info(env);
        ind.total_imgs_len = 0;
        for (uint32_t i = 0; i < env->prog.fw_context.code_img.img_count; i++)
            if (img_header->flag & DFU_FLAG_COMPRESS)
                ind.total_imgs_len += img_header[i].length;
        env->callback(DFU_APP_DL_END_AND_INSTALL_START_IND, &ind);
#ifndef OTA_NOR_OTA_MANAGER_LITE
        ret = dfu_img_install(env);
#else
        ret = DFU_SUCCESS;
#endif
        if (ret != DFU_SUCCESS)
            status = DFU_ERR_FW_INVALID;
        else
            status = DFU_ERR_NO_ERR;

        /* Notify remote device */
        dfu_record_current_tx();
        dfu_end_int_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_END_IND, dfu_end_int_t);
        rsp->result = status;
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (g_end_send_mode == DFU_END_SEND)
        {
            if (status != DFU_ERR_NO_ERR)
            {
                // install fail, no need to wait;
                dfu_ctrl_install_completed(env, status);
            }
            else
            {
                if (dfu_set_last_packet_wait() == 0)
                {
                    // wait last packet send then we process install completed
                }
                else
                {
                    // last packet already send, process now
                    LOG_I("dfu_img_tramission_end");
                    dfu_ctrl_install_completed(env, status);
                }
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
#endif
}

static void dfu_img_retransmission_request_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_img_retransmission_request_handler");
    DFU_ERR_CHECK(env);
    uint16_t state = env->prog.state;
    dfu_image_header_int_t *img_header = env->prog.fw_context.code_img.img_header;
    uint16_t status = DFU_ERR_GENERAL_ERR;
    dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;

    if (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.state == OTA_STATE_DOWNLOADING)
    {
        curr_info = &env->ota_state.fw_context.code_img.curr_img_info;
    }

    if (state == DFU_CTRL_TRAN_START ||
            (env->mode == DFU_CTRL_NORMAL_MODE && env->ota_state.state == OTA_STATE_DOWNLOADING))
    {
        status = DFU_ERR_NO_ERR;
        dfu_retransmission_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(
                DFU_RETRANSMISSION_RESPONSE, dfu_retransmission_response_t);

        rsp->result = status;
        rsp->retransmission_packet_num = env->retrans_state.last_rsp_pkt_num;

        uint8_t current_rsp_num = curr_info->img_info.dl_info.num_of_rsp;

        if (env->retrans_state.success_rsp_count == 0 || env->retrans_state.retry_count > 2)
        {
            env->retrans_state.success_rsp_count = 0;
            env->retrans_state.retry_count = 0;
            // reduce immediately
            if (current_rsp_num % 2 == 0)
            {
                current_rsp_num = current_rsp_num / 2;
            }
            else
            {
                current_rsp_num = 1;
            }
        }
        else
        {
            env->retrans_state.retry_count++;
        }

        curr_info->img_info.dl_info.num_of_rsp = current_rsp_num;
        curr_info->img_info.dl_info.curr_pkt_num = env->retrans_state.last_rsp_pkt_num;
        curr_info->img_info.dl_info.curr_img_length = env->retrans_state.last_rsp_img_length;

        LOG_I("dfu_img_retransmission_request_handler, new rsp: %d, retrans packet num %d",
              current_rsp_num, env->retrans_state.last_rsp_pkt_num);

        rsp->new_num_of_rsp = current_rsp_num;
        dfu_protocol_packet_send((uint8_t *)rsp);

        if (env->is_sync_timer_on)
        {
            os_timer_stop(g_dfu_sync_timer);
            os_timer_start(g_dfu_sync_timer, DFU_SYNC_DOWNLOAD_TIMER);
        }
    }
}

static void dfu_read_version_request_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    uint16_t req_index;
    memcpy(&req_index, data, sizeof(uint16_t));
    LOG_I("dfu_read_version_request_handler %d, %d", req_index, len);

#ifdef OTA_USING_SOL_FUNC
    device_info_t device_info;
    app_db_get_device_info(&device_info);

    dfu_read_version_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_READ_VERSION_RESPONSE, dfu_read_version_response_t);

    if ((req_index + 1) * DFU_PART_VERSION_LEN >= DFU_MAX_VERSION_LEN)
    {
        rsp->len = DFU_MAX_VERSION_LEN - req_index * DFU_PART_VERSION_LEN;
        memcpy(rsp->version, device_info.sw_version + req_index * DFU_PART_VERSION_LEN, rsp->len);
        rsp->result = 0;
    }
    else
    {
        memcpy(rsp->version, device_info.sw_version + req_index * DFU_PART_VERSION_LEN, DFU_PART_VERSION_LEN);
        rsp->result = req_index + 1;
        rsp->len = DFU_PART_VERSION_LEN;
    }
    dfu_protocol_packet_send((uint8_t *)rsp);
#else
    dfu_read_version_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_READ_VERSION_RESPONSE, dfu_read_version_response_t);
    rsp->result = 0;
    rsp->len = 0;
    memset(rsp->version, 0, DFU_PART_VERSION_LEN);

    dfu_protocol_packet_send((uint8_t *)rsp);
#endif
}

static void dfu_power_off_command_handler(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_power_off_command_handler");
#ifdef OTA_USING_SOL_FUNC
    app_power_off_action(POWER_OFF);
#endif
}

static void dfu_psram_cr_set()
{
    // hwp_pmuc->CR |= PMUC_CR_PIN_RET;
}

static void dfu_ctrl_install_completed(dfu_ctrl_env_t *env, uint16_t status)
{
    dfu_app_img_install_completed_ind_t ind;
    ind.result = status == DFU_ERR_NO_ERR ? DFU_APP_SUCCESSED : DFU_APP_FAIED;
    env->callback(DFU_APP_INSTALL_COMPLETD_IND, &ind);

    int ret = dfu_img_install_lcpu_rom_patch(env);
    LOG_I("dfu_img_install_lcpu_rom_patch %d", ret);

    if (status == DFU_ERR_NO_ERR)
    {
        env->callback(DFU_APP_TO_USER, NULL);
        env->prog.state = DFU_CTRL_UPDATING;
        env->prog.FW_state = DFU_CTRL_FW_INSTALLED;
        env->is_force_update = 0;
        dfu_image_state_init(env);
        dfu_flash_addr_reset(0);
        dfu_ctrl_update_prog_info(env);
        dfu_update_img_header(env);
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 1);
        drv_reboot();
        //dfu_ctrl_boot_to_user_fw();
    }
    else
        /* Should disconnect link and enter force update state. */
    {
        env->is_force = 1;
        dfu_ctrl_error_handle(env);
    }
}


void dfu_protocol_close_handler(void)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    dfu_img_info_t *curr_img = &env->prog.fw_context.code_img.curr_img_info;
    if (env->prog.state == DFU_CTRL_TRAN_START || env->prog.state == DFU_CTRL_PREPARE_START
#ifdef OTA_ERROR_HANDLE_IN_USR
            || env->prog.state == DFU_CTRL_PREPARE_START_FORCE
#endif
       )
    {
        if (check_current_img_need_force_update(curr_img->img_id))
        {
            env->is_force = 1;
        }
    }
    if (env->prog.state == DFU_CTRL_FORCE_UPDATE)
    {
        env->is_force = 1;
    }

    dfu_app_error_disconnect_ind_t ind;
    ind.img_id = curr_img->img_id;

    ind.ota_mode = env->mode;
    if (env->mode == DFU_CTRL_OTA_MODE)
    {
        ind.curr_state = env->prog.state;
    }
    else if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        ind.curr_state = env->ota_state.state;
        env->ota_state.state = OTA_STATE_NONE;
    }

    if (env->flash_erase_state == DFU_FLASH_ERASE)
    {
        LOG_I("disconnect while erase");
        env->flash_erase_state = DFU_FLASH_ERASE_INTERRUPT;
    }

    env->callback(DFU_APP_ERROR_DISCONNECT_IND, &ind);
    dfu_ctrl_error_handle(env);
}

static void dfu_link_lose_check_rsp(dfu_ctrl_env_t *env, uint8_t *data, uint16_t len)
{
    LOG_I("dfu_link_lose_check_rsp");
    dfu_link_lose_check_rsp_t *ind = (dfu_link_lose_check_rsp_t *)data;
    dfu_link_sync_end();

    dfu_link_sync_start(DFU_SYNC_TYPE_DOWNLOAD);
}

void dfu_protocol_packet_handler(dfu_tran_protocol_t *msg, uint16_t length)
{
    DFU_ERR_CHECK(msg);
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->current_command = msg->message_id;

    uint8_t is_force = 0;
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
        dfu_img_retransmission_request_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_READ_VERSION_REQUEST:
    {
        dfu_read_version_request_handler(env, msg->data, msg->length);
        break;
    }
    case DFU_COMMAND_POWER_OFF:
    {
        dfu_power_off_command_handler(env, msg->data, msg->length);
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
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    switch (env->current_command)
    {
    case DFU_IMAGE_SEND_PACKET:
    {
        dfu_link_lose_check_req(env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num, env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp);
        break;
    }
    default:
    {
        LOG_I("dfu_serial_transport_error_handle %d", env->current_command);

        dfu_img_info_t *curr_img = &env->prog.fw_context.code_img.curr_img_info;
        if (check_current_img_need_force_update(curr_img->img_id))
        {
            env->is_force = 1;
        }
        dfu_ctrl_error_handle(env);
    }

        // TODO: handle other step's error later
    }

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





static void dfu_ctrl_enter_force_update_state_of_OTA_mode(dfu_ctrl_env_t *env)
{
    /* Only OTA mode could enter force update state. */
#ifndef OTA_ERROR_HANDLE_IN_USR
    OS_ASSERT(env->mode == DFU_CTRL_OTA_MODE);
#endif
    env->prog.state = DFU_CTRL_FORCE_UPDATE;
    /* Disconnect and wait connect again. */
    dfu_ctrl_update_prog_info(env);
#ifdef OTA_ERROR_HANDLE_IN_USR
    dfu_set_reboot_after_disconnect();
#else
    dfu_protocol_session_close();
#endif
}

static void dfu_ctrl_timer_2_normal_mode_handler(void *para)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    os_timer_stop(g_dfu_timer);
    os_timer_delete(g_dfu_timer);
    // dfu_ctrl_boot_to_user_fw();
    HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 1);
    drv_reboot();
}


static void dfu_ctrl_enter_idle_state_of_normal_mode(dfu_ctrl_env_t *env)
{
    env->prog.state = DFU_CTRL_IDLE;
    if (env->mode == DFU_CTRL_OTA_MODE)
    {
        env->callback(DFU_APP_TO_USER, NULL);
        dfu_flash_addr_reset(0);
        dfu_ctrl_update_prog_info(env);
        //TODO reboot module to disable related modules
        // os_timer_create(g_dfu_timer, dfu_ctrl_timer_2_normal_mode_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        // os_timer_start(g_dfu_timer, 500);
        // dfu_ctrl_boot_to_user_fw();
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 1);
        drv_reboot();
    }
    else
    {
#ifdef OTA_USING_SOL_FUNC
        if (ota_manager_download_buffer)
        {
            app_cache_free(ota_manager_download_buffer);
        }
        ota_manager_download_buffer = NULL;
#endif
    }
}

static uint8_t dfu_ctrl_error_handle(dfu_ctrl_env_t *env)
{
    LOG_E("dfu_ctrl_error_handle");
    dfu_flash_exit_msg_send();
    dfu_link_sync_end();
    if (env->mode == DFU_CTRL_OTA_MODE && (env->is_force || env->is_force_update))
        dfu_ctrl_enter_force_update_state_of_OTA_mode(env);
    else
        dfu_ctrl_enter_idle_state_of_normal_mode(env);
    return 0;
}


static void dfu_ctrl_timer_handler(void *para)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    uint16_t state = env->prog.state;
    os_timer_stop(g_dfu_timer);
    os_timer_delete(g_dfu_timer);
    /* If state doesn't change till timeout , back to idle or force update state. */
    if (state == DFU_CTRL_PREPARE_START
#ifdef OTA_ERROR_HANDLE_IN_USR
            || state == DFU_CTRL_PREPARE_START_FORCE
#endif
       )
    {
        dfu_ctrl_error_handle(env);

    }
}

static void dfu_link_lose_check_req(uint32_t current_index, uint16_t new_num_of_rsp)
{
    LOG_I("dfu_link_lose_check_req index %d, rsp %d", current_index, new_num_of_rsp);
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
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

    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        env->ota_state.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp = new_num_of_rsp;
    }
    else
    {
        env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp = new_num_of_rsp;
    }

    dfu_link_lose_check_req_t *res_rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_LINK_LOSE_CHECK_REQ, dfu_link_lose_check_req_t);
    res_rsp->result = DFU_ERR_NO_ERR;
    res_rsp->current_file_index = current_index;
    res_rsp->new_num_of_rsp = new_num_of_rsp;
    dfu_protocol_packet_send((uint8_t *)res_rsp);
}

static void dfu_rsp_sync_handler(void *para)
{
    LOG_W("remote lost");
    dfu_link_sync_end();
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    dfu_img_info_t *curr_img = &env->prog.fw_context.code_img.curr_img_info;
    if (check_current_img_need_force_update(curr_img->img_id))
    {
        env->is_force = 1;
    }

    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        dfu_protocol_session_close();
    }

    dfu_ctrl_error_handle(env);
}


static bool dfu_link_sync_check()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    uint32_t current_value;
    uint8_t current_rsp;

    if (env->mode == DFU_CTRL_NORMAL_MODE)
    {
        if (env->ota_state.dfu_ID == DFU_ID_OTA_MANAGER)
        {
            current_value = env->ota_state.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num;
            current_rsp = env->ota_state.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp;
        }
        else
        {
            current_value = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num;
            current_rsp = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp;
        }
    }
    else
    {
        current_value = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.curr_pkt_num;
        current_rsp = env->prog.fw_context.code_img.curr_img_info.img_info.dl_info.num_of_rsp;
    }

    if (env->sync_size == current_value)
    {
        LOG_I("dfu_link_sync_check error %d", env->sync_size);
        dfu_link_lose_check_req(env->sync_size, current_rsp);
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

static void dfu_sync_erase_timer_handler(void *para)
{
    dfu_link_sync_end();
    int ret = dfu_protocol_session_close();
    if (ret == -1)
    {
        dfu_port_svc_session_close();
    }
}


void dfu_link_sync_start(uint8_t sync_type)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

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
    else if (sync_type == DFU_SYNC_TYPE_RSP)
    {
        os_timer_create(g_dfu_sync_timer, dfu_rsp_sync_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_dfu_sync_timer, DFU_SYNC_FILE_DOWNLOAD_TIMER);
    }
}


static void dfu_link_sync_end()
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (env->is_sync_timer_on == 0)
    {
        LOG_I("dfu_link_sync_end, no timer to stop");
        return;
    }
    env->is_sync_timer_on = 0;
    os_timer_stop(g_dfu_sync_timer);
    os_timer_delete(g_dfu_sync_timer);
}


void dfu_register(dfu_callback callback)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->callback = callback;
}

uint8_t dfu_flash_addr_set(uint8_t img_id, uint32_t addr, uint32_t size)
{
#ifdef OTA_SECTION_CHANGE
    if (img_id > DFU_IMG_ID_MAX)
    {
        return DFU_ERR_GENERAL_ERR;
    }

    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    env->prog.download_flash_addr[img_id] = addr;
    env->prog.download_flash_size[img_id] = size;
    //dfu_ctrl_update_prog_info(env);
    return DFU_ERR_NO_ERR;
#else
    return DFU_ERR_GENERAL_ERR;
#endif // OTA_SECTION_CHANGE
}

uint8_t dfu_flash_addr_get(uint8_t img_id, dfu_flash_info_t *info)
{
#ifdef OTA_SECTION_CHANGE
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
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
#endif // OTA_SECTION_CHANGE
}

void dfu_flash_addr_reset(uint8_t mode)
{
#ifdef OTA_SECTION_CHANGE
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    for (int i = 0; i < DFU_IMG_ID_MAX; i++)
    {
        env->prog.download_flash_addr[i] = 0;
        env->prog.download_flash_size[i] = 0;
    }
    if (mode == 1)
    {
        dfu_ctrl_update_prog_info(env);
    }
#endif // OTA_SECTION_CHANGE
}

static void dfu_ctrl_reboot_handler(void *para)
{
    os_timer_stop(g_dfu_timer);
    os_timer_delete(g_dfu_timer);
    drv_reboot();
}

void dfu_respond_start_request_ext(dfu_event_ack_t result)
{
    LOG_I("dfu_respond_start_request_ext");
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

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

    if (env->mode == DFU_CTRL_NORMAL_MODE && result == DFU_EVENT_SUCCESSED)
    {
        if (env->ota_state.dfu_ID != DFU_ID_OTA_MANAGER)
        {
#ifndef OTA_NOR_OTA_MANAGER_LITE
            env->is_reboot_timer_on = 1;
            os_timer_create(g_dfu_timer, dfu_ctrl_reboot_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            os_timer_start(g_dfu_timer, DFU_REBOOT_TIMER);
#endif
        }
    }

    if (env->ota_state.dfu_ID == DFU_ID_OTA_MANAGER)
    {
        if (status == DFU_ERR_USER_REJECT)
        {
            memset(&env->ota_state, 0, sizeof(dfu_ota_manager_update_state_t));
        }
    }

    dfu_init_response_ext_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE_EXT, dfu_init_response_ext_t);
    dfu_dl_image_header_t *dl_hdr = &env->prog.fw_context.code_img;

    rsp->ver = OTA_CODE_VERSION;
    rsp->result = status;
    rsp->resume_status = env->resume_status;
#ifdef OTA_NOR_OTA_MANAGER_LITE
    rsp->is_boot = 0;
#else
    rsp->is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;
#endif
    if (env->ota_state.dfu_ID == DFU_ID_OTA_MANAGER)
    {
        rsp->is_boot = 0;
        env->ota_state.state = OTA_STATE_PREPARE;
    }

    if (env->resume_status == 1 && dfu_get_backup_mode() == DFU_BACKUP_MODE_PSRAM)
    {
        if (dfu_image_state_get_resume_mode(env, env->prog.fw_context.code_img.curr_img_info.img_id))
        {
            rsp->resume_status = 2;
        }
    }

    if (env->resume_status == 1 && !env->resume_is_restart)
    {
        rsp->curr_img = dl_hdr->curr_img_info.img_id;
        rsp->curr_packet_num = dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num;
        rsp->num_of_rsp = dl_hdr->curr_img_info.img_info.dl_info.num_of_rsp;

        env->resume_image_index = dl_hdr->curr_img_info.img_info.dl_info.curr_pkt_num;
        env->resume_image_id = dl_hdr->curr_img_info.img_id;
    }

    LOG_W("dfu_respond_start_request status %d, is reboot %d", status, rsp->is_boot);
    if (status == DFU_ERR_NO_ERR && rsp->is_boot == 1)
    {
        dfu_set_reboot_after_disconnect();
    }
    dfu_protocol_packet_send((uint8_t *)rsp);
}


void dfu_respond_start_request(dfu_event_ack_t result)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    if (env->prog.state == DFU_CTRL_OFFLINE_INSTALL_PREPARE)
    {
        env->prog.state = DFU_CTRL_OFFLINE_INSTALL;

        /* Save current status to flash. */
        dfu_ctrl_update_prog_info(env);
        HAL_PMU_Reboot();
        return;
    }

    /* Not allowed postpone in this API. */
    OS_ASSERT(result != DFU_EVENT_POSTPONE);

    if (env->current_command == DFU_INIT_REQUEST_EXT)
    {
        dfu_respond_start_request_ext(result);
        return;
    }

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

    if (env->mode == DFU_CTRL_NORMAL_MODE && result == DFU_EVENT_SUCCESSED)
    {
        if (env->ota_state.dfu_ID != DFU_ID_OTA_MANAGER)
        {
            env->is_reboot_timer_on = 1;
            os_timer_create(g_dfu_timer, dfu_ctrl_reboot_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            os_timer_start(g_dfu_timer, DFU_REBOOT_TIMER);
        }
    }

    if (env->ota_state.dfu_ID == DFU_ID_OTA_MANAGER)
    {
        if (status == DFU_ERR_USER_REJECT)
        {
            memset(&env->ota_state, 0, sizeof(dfu_ota_manager_update_state_t));
        }
    }

    dfu_init_response_t *rsp = DFU_PROTOCOL_PKT_BUFF_ALLOC(DFU_INIT_RESPONSE, dfu_init_response_t);
    rsp->result = status;
    rsp->is_boot = env->mode == DFU_CTRL_NORMAL_MODE ? 1 : 0;
    if (env->ota_state.dfu_ID == DFU_ID_OTA_MANAGER)
    {
        rsp->is_boot = 0;
        env->ota_state.state = OTA_STATE_PREPARE;
    }
    LOG_W("dfu_respond_start_request status %d, is reboot %d", status, rsp->is_boot);
    if (status == DFU_ERR_NO_ERR && rsp->is_boot == 1)
    {
        dfu_set_reboot_after_disconnect();
    }
    dfu_protocol_packet_send((uint8_t *)rsp);
}

void run_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}

#ifdef SF32LB52X
static void rc10k_timeout_handler(void *parameter)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    if (!HAL_RTC_LXT_ENABLED())
    {
        rt_event_send(env->rc10k_event, 1);
    }
    else
    {
        rt_timer_stop(env->rc10k_time_handle);
    }
}
static void rc10_task_entry(void *param)
{
    rt_uint32_t evt;
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();

    while (1)
    {
        rt_event_recv(env->rc10k_event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
        LOG_W("DFU RC10K cur:%d, ave:%d", HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR), HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE));
    }
}
#endif

uint8_t dfu_ctrl_reset_handler(void)
{
    dfu_ctrl_env_t *env = dfu_ctrl_get_env();
    RT_ASSERT(env->mode == DFU_CTRL_OTA_MODE);
    uint16_t state = env->prog.state;

    uint8_t status = DFU_ERR_GENERAL_ERR;
    uint8_t is_jump = 0;
    LOG_I("dfu_ctrl_reset_handler %d", state);
    dfu_ota_bootloader_ram_run_set(DFU_RAM_STATE_UPDATE_FAIL);

    if (HAL_Get_backup(RTC_BAKCUP_OTA_FORCE_MODE) == 1)
    {
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 0);
        dfu_flash_addr_reset(0);
        dfu_ctrl_boot_to_user_fw();
    }

    if (env->prog.ota_lite == 1)
    {
        if (env->prog.hcpu_available == DFU_IMAGE_AVAILABLE && env->prog.res_available == DFU_IMAGE_AVAILABLE)
        {
            if (state >= DFU_CTRL_NEG && state <= DFU_CTRL_UPDATED)
            {
                state = DFU_CTRL_IDLE;
                env->prog.state = DFU_CTRL_IDLE;
                dfu_ctrl_update_prog_info(env);
            }
        }
    }

    switch (state)
    {
        /* Prepare start state  will W4 DFU_IMAGE_SEND_START. */
#ifdef OTA_ERROR_HANDLE_IN_USR
    case DFU_CTRL_PREPARE_START_FORCE:
        env->is_force_update = 1;
#endif
    case DFU_CTRL_PREPARE_START:
    {
        /* Start timer to avoid corner case. */
        os_timer_create(g_dfu_timer, dfu_ctrl_timer_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_dfu_timer, DFU_PREPARE_START_TIMER);
    }
    break;
    /* The two state treat as disconnection case. */
    case DFU_CTRL_TRAN_START:
    {
        dfu_img_info_t *curr_info = &env->prog.fw_context.code_img.curr_img_info;
        if (check_current_img_need_force_update(curr_info->img_id))
        {
            env->is_force = 1;
        }
        //dfu_ctrl_error_handle(env);
        env->prog.state = DFU_CTRL_FORCE_UPDATE;
        dfu_ctrl_update_prog_info(env);
    }
    break;
    case DFU_CTRL_INSTALL:
    case DFU_CTRL_UPDATING:
    {
        /* The status is still updating after reboot that it indicates the FW image has something wrong, enter force update status. */
        env->is_force = 1;
        //dfu_ctrl_error_handle(env);
        env->prog.state = DFU_CTRL_FORCE_UPDATE;
        dfu_ctrl_update_prog_info(env);
    }
    break;
    case DFU_CTRL_FORCE_UPDATE:
    {
        // Move prepare env to the end of this function.
        env->is_force = 1;
    }
    break;
    case DFU_CTRL_OFFLINE_INSTALL:
    {
        if (env->callback)
        {
            env->callback(DFU_APP_OFFLINE_INSTALL_IND, NULL);
        }
        is_jump = 0;
        dfu_offline_install();
        break;
    }
    case DFU_CTRL_REBOOT_INSTALL_PREPARE:
    {
        dfu_reboot_install();
        break;
    }
    default:
        is_jump = 1;
        break;
    }

    if (is_jump)
    {
        dfu_flash_addr_reset(0);
        dfu_ctrl_boot_to_user_fw();
    }
    else
    {
#ifdef OTA_ERROR_HANDLE_IN_USR
        if (env->is_force)
        {
            HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 1);
            dfu_ctrl_boot_to_user_fw();
        }
#endif

#ifdef SF32LB52X
        env->rc10k_event = rt_event_create("rc10_evt", RT_IPC_FLAG_FIFO);
        env->task_handle = rt_thread_create("rc10_cal", rc10_task_entry, NULL,
                                            1024, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
        if (RT_NULL != env->task_handle)
            rt_thread_startup(env->task_handle);
        else
            RT_ASSERT(0);

        env->rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                                  rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
        rt_timer_start(env->rc10k_time_handle);
#endif
        dfu_protocol_reset_env_prepare();
        env->callback(DFU_APP_RESET_IND, NULL);
    }

    return 0;
}

int dfu_get_version(uint8_t *version, uint8_t size)
{
    if (size > DFU_OTA_VERSION_LEN_MAX || version == NULL)
    {
        return DFU_ERR_GENERAL_ERR;
    }

    uint32_t addr = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    addr = addr << 2;
    uint8_t *ver;
    ver = (uint8_t *)addr;

    LOG_I("dfu_get_version addr: 0x%x", addr);

    memcpy(version, ver, size);
    return DFU_ERR_NO_ERR;
}

void SVC_Handler_Main(unsigned int *svc_args)
{
    unsigned int svc_number;
    svc_number = ((char *)svc_args[6])[-2];
    switch (svc_number)
    {
    case 0:
    {
        __set_CONTROL(__get_CONTROL() & ~CONTROL_nPRIV_Msk);
        dfu_bootjump();
    }
    break;
    default:
        break;
    }

}



static const uint8_t g_dfu_p_default[] = {0x0};

static struct fdb_default_kv_node default_dfu_kv_set[] =
{
    {DFU_DOWNLOAD_ENV, (void *)g_dfu_p_default, sizeof(g_dfu_p_default)},
};


#ifdef FDB_USING_KVDB
static fdb_err_t dfu_db_init(void)
{
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
    int max_size = 16 * 1024;
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

#ifdef DFU_OTA_MANAGER
    extern uint8_t OTA_VERSION[];
#endif

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
        HAL_Set_backup(RTC_BAKCUP_OTA_FORCE_MODE, 0);
#endif
    }
    while (0);
    // debug use
    //env->prog.state = DFU_CTRL_FORCE_UPDATE;
    if (env->mode == DFU_CTRL_NORMAL_MODE &&
            env->prog.state == DFU_CTRL_UPDATING)
    {
        env->prog.state = DFU_CTRL_IDLE;
        env->is_force = 0;
#ifdef OTA_MODEM_RECORD
        uint8_t temp_state = env->prog.modem_ota_state;
#endif
        //dfu_update_img_header(env);
        memset(&env->prog, 0, sizeof(dfu_download_progress_t));

#ifdef OTA_MODEM_RECORD
        env->prog.modem_ota_state = temp_state;
#endif
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

#if 0//defined(BSP_USING_PM) && !defined(BSP_USING_PSRAM)
static int dfu_pm_suspend(const struct rt_device *device, uint8_t mode)
{
    int r = RT_EOK;

    return r;
}

void dfu_pm_resume(const struct rt_device *device, uint8_t mode)
{
    if ((PM_SLEEP_MODE_STANDBY != mode)
            && (PM_SLEEP_MODE_DEEP != mode))
    {
        return;
    }

    dfu_db_init();

    return;
}


const struct rt_device_pm_ops dfu_pm_op =
{
    .suspend = dfu_pm_suspend,
    .resume = dfu_pm_resume,
};


int dfu_pm_init(void)
{
    rt_pm_device_register(NULL, &dfu_pm_op);

    return 0;
}
INIT_APP_EXPORT(dfu_pm_init);
#endif /* BSP_USING_PM */


static void dfu_cmd(uint8_t argc, char **argv)
{
    char *value = NULL;

    if (argc > 1)
    {
        if (strcmp(argv[1], "reset") == 0)
        {
            fdb_kv_set_default(p_dfu_db);
        }
        else if (strcmp(argv[1], "version") == 0)
        {
            uint8_t *v = malloc(8);
            dfu_get_version(v, 8);
            LOG_HEX("ota ver", 16, v, 8);
            free(v);
        }
        else if (strcmp(argv[1], "offline_check") == 0)
        {
            dfu_offline_install_start();
        }
        else if (strcmp(argv[1], "ram_init") == 0)
        {
            dfu_ota_bootloader_ram_run_flash_init();
        }
        else if (strcmp(argv[1], "ram_set") == 0)
        {
            int val = atoi(argv[2]);
            dfu_ota_bootloader_ram_run_set(val);
        }
    }
}
MSH_CMD_EXPORT(dfu_cmd, DFU command);

#endif /* OTA_55X */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
