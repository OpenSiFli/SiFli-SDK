/**
  ******************************************************************************
  * @file   dfu_internal.h
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


#ifndef __DFU_INTERNAL_H
#define __DFU_INTERNAL_H

#include "rtthread.h"
#include <stdio.h>
#include <stdint.h>
#include "dfu_protocol.h"
#include "dfu_service.h"
#include "data_service.h"
#include "mem_section.h"

#define DFU_DB "SIF_DFU"
#define DFU_DB_PARTIAL "dfu"
#define DFU_DOWNLOAD_ENV "dfu_env"

#define X_VER_MAX_DIGIT 2
#define Y_VER_MAX_DIGIT 3
#define Z_VER_MAX_DIGIT 3

#define X_VER_OFFSET (Y_VER_MAX_DIGIT + Z_VER_MAX_DIGIT)
#define Y_VER_OFFSET (Z_VER_MAX_DIGIT)
#define Z_VER_OFFSET 0

#define DFU_REBOOT_TIMER 6000
#define DFU_PREPARE_START_TIMER 30000

// wait ios to support link lose check, we can change to 6s
#define DFU_SYNC_DOWNLOAD_TIMER 6000
#define DFU_SYNC_ERASE_TIMER 240000
#define DFU_SYNC_FILE_DOWNLOAD_TIMER 12000
#define DFU_SYNC_RSP_TIMER 6000

#define DFU_OTA_MANAGER_COM_SIZE ((DFU_FLASH_CODE_SIZE * 7 / 10 / 4 + 1) * 4)
#define DFU_BOOTLOADER_COM_SIZE ((FLASH_BOOT_LOADER_SIZE * 7 / 10 / 4 + 1) * 4)

// upgrade in user bin, now have otaManager, bootloader
#define DFU_USER_BIN_UPGRADE_BIN_SIZE 2

#define DFU_SYNC_GET_DEFAULT_TIMEOUT 1000

#define DFU_INTERVAL_SLOW 20
#define DFU_OTA_VERSION_LEN_MAX 32
#define DFU_HASH_VERIFY_WDT_PET_FREQUENCY 20

#define DFU_EMMC_ADDR_RANGE 0xA0000000
#define DFU_EMMC_ADDR_RANGE_FLAG 0xF0000000

#define DFU_PROTOCOL_PKT_BUFF_ALLOC(msg_id, msg_struct) \
    (msg_struct *)dfu_protocol_packet_buffer_alloc(msg_id, sizeof(msg_struct));




#ifdef BSP_USING_PSRAM
    #define DFU_NON_RET_SECT_BEGIN L2_CACHE_RET_BSS_SECT_BEGIN(kvdb)
    #define DFU_NON_RET_SECT_END   L2_CACHE_RET_BSS_SECT_END
#else
    #define DFU_NON_RET_SECT_BEGIN L1_NON_RET_BSS_SECT_BEGIN(dfu_not)
    #define DFU_NON_RET_SECT_END   L1_NON_RET_BSS_SECT_END
#endif







#ifdef HCPU_PATCH_BURN_ADDR
    #define DFU_NAND_PATCH_DOWNLOAD_ADDR HCPU_PATCH_BURN_ADDR
    #define DFU_NAND_HCPU_FIRST_ADDR HCPU_FLASH_CODE_BURN_ADDR
    #define DFU_NAND_HCPU_BACK_UP_ADDR HCPU_FLASH_CODE_Y_BURN_ADDR

    #define DFU_NAND_HCPU_DOWNLOAD_SIZE HCPU_PATCH_SIZE
    #define DFU_NAND_HCPU_BACK_UP_SIZE HCPU_FLASH_CODE_Y_SIZE
    #define DFU_NAND_PATCH_DOWNLOAD_SIZE HCPU_PATCH_SIZE
#else

    #ifdef HCPU_FLASH_CODE_LOAD_REGION_START_ADDR
        #define DFU_NAND_PATCH_DOWNLOAD_ADDR 0
        #define DFU_NAND_HCPU_FIRST_ADDR HCPU_FLASH_CODE_LOAD_REGION_START_ADDR
        #define DFU_NAND_HCPU_BACK_UP_ADDR HCPU_FLASH_CODE_LOAD_REGION2_START_ADDR

        #define DFU_NAND_HCPU_DOWNLOAD_SIZE HCPU_FLASH_CODE_LOAD_REGION_SIZE
        #define DFU_NAND_HCPU_BACK_UP_SIZE HCPU_FLASH_CODE_LOAD_REGION2_SIZE
        #define DFU_NAND_PATCH_DOWNLOAD_SIZE 0
    #else
        #define DFU_NAND_PATCH_DOWNLOAD_ADDR 0
        #define DFU_NAND_HCPU_FIRST_ADDR 0x14000000
        #define DFU_NAND_HCPU_BACK_UP_ADDR 0x14200000

        #define DFU_NAND_HCPU_DOWNLOAD_SIZE 0x200000
        #define DFU_NAND_HCPU_BACK_UP_SIZE 0x200000
        #define DFU_NAND_PATCH_DOWNLOAD_SIZE 0
    #endif
#endif

// 32K for lcpu rom patch download
#define DFU_LCPU_PATCH_DOWNLOAD_SIZE 0x8000
#define DFU_LCPU_DOWNLOAD_ADDR DFU_NAND_PATCH_DOWNLOAD_ADDR + DFU_LCPU_PATCH_DOWNLOAD_SIZE
#define DFU_LCPU_PATCH_DOWNLOAD_ADDR DFU_NAND_PATCH_DOWNLOAD_ADDR


// change size later
#define APP_L2_CACHE_RET_OTA_SECT_BEGIN(section_name)        L2_CACHE_RET_OTA_SECT_BEGIN(section_name)
/** L2 cachable retained bss section end*/
#define APP_L2_CACHE_RET_OTA_SECT_END                        L2_CACHE_RET_OTA_SECT_END


/** L2 cachable retained bss section begin*/
#define L2_CACHE_RET_OTA_SECT_BEGIN(section_name)        SECTION_ZIDATA_BEGIN(section_name)
/** L2 cachable retained bss section end*/
#define L2_CACHE_RET_OTA_SECT_END                        SECTION_ZIDATA_END

#ifdef BSP_USING_PSRAM
    #define OTA_L2_RET_SECT_BEGIN L2_RET_SECT_BEGIN(ota_psram_ret_cache)
    #define OTA_L2_RET_SECT_END L2_RET_SECT_END

    #define L2_RET_SECT_BEGIN(section_name)  SECTION_ZIDATA_BEGIN(.l2_ota_ret_data_##section_name)
    #define L2_RET_SECT_END SECTION_ZIDATA_END
#endif

typedef enum
{
    DFU_SYNC_TYPE_DOWNLOAD,
    DFU_SYNC_TYPE_ERASE,
    DFU_SYNC_TYPE_FILE,
    DFU_SYNC_TYPE_RSP,
} dfu_sync_timer_t;

typedef enum
{
    DFU_CTRL_IDLE,
    DFU_CTRL_NEG,
    DFU_CTRL_PREPARE_START,
    DFU_CTRL_TRAN_START,
    DFU_CTRL_INSTALL,
    DFU_CTRL_UPDATING,
    DFU_CTRL_FORCE_UPDATE,
    DFU_CTRL_PREPARE_START_FORCE,
    DFU_CTRL_UPDATED,
    DFU_CTRL_OFFLINE_INSTALL_PREPARE,
    DFU_CTRL_OFFLINE_INSTALL,
    DFU_CTRL_REBOOT_INSTALL_PREPARE,
    DFU_CTRL_REBOOT_INSTALL,
    DFU_CTRL_PACKAGE_INSTALL_PREPARE,
    DFU_CTRL_PACKAGE_INSTALL,
} dfu_ctrl_state_t;

typedef enum
{
    DFU_CTRL_NORMAL_MODE,
    DFU_CTRL_OTA_MODE,
    DFU_CTRL_BOOTLOADER
} dfu_ctrl_mode_t;

typedef enum
{
    DFU_CTRL_FW_NO_STATE,
    DFU_CTRL_FW_DOWNLOADING,
    DFU_CTRL_FW_INSTALLING,
    DFU_CTRL_FW_INSTALLED,
} dfu_ctrl_fw_state_t;

typedef enum
{
    DFU_CTRL_IMG_STATE_IDLE,
    DFU_CTRL_IMG_STATE_DOWNLOADING,
    DFU_CTRL_IMG_STATE_DOWNLOADED_FAIL,
    DFU_CTRL_IMG_STATE_DOWNLOADED,
    DFU_CTRL_IMG_STATE_INSTALLING,
    DFU_CTRL_IMG_STATE_INSTALLED,
} dfu_ctrl_img_state_t;

typedef enum
{
    DFU_PROGRESS_PART,
    DFU_PROGRESS_TOTAL,
} dfu_progress_mode_t;

typedef enum
{
    DFU_END_NO_SEND,
    DFU_END_SEND,
    DFU_END_RECEIVE,
} dfu_end_mode_t;

typedef enum
{
    DFU_FLASH_TYPE_NAND,
    DFU_FLASH_TYPE_NOR,
    DFU_FLASH_TYPE_EMMC,
} dfu_flash_type_t;

typedef enum
{
    OTA_STATE_NONE, // normal state
    OTA_STATE_PREPARE, // after receive init packet
    OTA_STATE_DOWNLOADING,
    OTA_STATE_DOWNLOADED,
    OTA_STATE_INTALLING,
    OTA_STATE_INSTALLED,
} dfu_ota_state;

typedef enum
{
    DFU_IMAGE_STATE_NONE,
    DFU_IMAGE_STATE_READY,
    DFU_IMAGE_STATE_DOWNLOADING_TO_FLASH,
    DFU_IMAGE_STATE_DOWNLOAD_FINISH_TO_FLASH,
    DFU_IMAGE_STATE_RESET_TO_PSRAM,
    DFU_IMAGE_STATE_DOWNLOADING_TO_PSRAM,
    DFU_IMAGE_STATE_DOWNLOAD_FINISH_TO_PSRAM,
    DFU_IMAGE_STATE_DOWNLOADING_OVERWRITE,
    DFU_IMAGE_STATE_DOWNLOAD_FINISH_OVERWRITE,
} dfu_image_state_t;

typedef enum
{
    DFU_BACKUP_MODE_OVERWRITE = 1,
    DFU_BACKUP_MODE_FLASH,
    DFU_BACKUP_MODE_PSRAM,
} dfu_backup_mode_t;

typedef enum
{
    DFU_IMAGE_RETRANS_NONE,
    DFU_IMAGE_RETRANS_SKIP,
} dfu_retrans_state_t;

typedef enum
{
    DFU_IMAGE_AVAILABLE,
    DFU_IMAGE_AVAILABLE_NONE,
} dfu_image_available_state_t;

#define DFU_ERR_CHECK(x) \
    if (!x) \
        return;


struct img_header_compress_info
{
    uint32_t    total_len;
    uint32_t    pksize;
};


typedef struct
{
    uint32_t curr_img_length;
    uint32_t total_pkt_num;
    uint32_t curr_pkt_num;
    uint8_t num_of_rsp;
} dfu_dl_img_info_t;

typedef enum
{
    ERASE_PROCESSING = 1,
} dfu_erase_state;

typedef struct
{
    uint32_t curr_img_length;
} dfu_install_img_info_t;


typedef struct
{
    uint32_t img_length;
    dfu_image_header_int_t *header;
    uint8_t img_id;
    dfu_ctrl_img_state_t img_state;
    union
    {
        dfu_dl_img_info_t dl_info;
        dfu_install_img_info_t in_info;
    } img_info;
} dfu_img_info_t;

typedef enum
{
    LCPU_INSTALL_NO_UPDATE = 0,
    LCPU_INSTALL_UPDATING = 1,
    LCPU_INSTALL_UPDATED = 2,
} lcpu_cpu_install;

typedef enum
{
    PACKAGE_INSTALL_TYPE_IMAGE,
    PACKAGE_INSTALL_TYPE_OTA_MANAGER,
} dfu_package_install_type_t;

typedef struct
{
    uint16_t blk_size;
    uint8_t img_count;
    dfu_image_header_int_t img_header[DFU_IMG_ID_MAX];
    dfu_img_info_t curr_img_info;
} dfu_dl_image_header_t;

typedef struct
{
    uint16_t blk_size;
    uint8_t img_count;
    dfu_image_header_int_t img_header[DFU_USER_BIN_UPGRADE_BIN_SIZE];
    dfu_img_info_t curr_img_info;
} dfu_dl_ota_image_header_t;


typedef struct
{
    uint16_t state;
    uint8_t dfu_ID;
    uint8_t FW_state;
    uint32_t HW_version;
    uint32_t SDK_version;
    uint32_t FW_version;
    uint8_t FW_key[DFU_KEY_SIZE];
    uint8_t backup_mode;
    uint8_t image_state[DFU_IMG_ID_MAX];

    uint8_t hcpu_available;
    uint8_t res_available;
    uint8_t ota_lite;
    // use for ota lite sol record hcpu or mix
    uint8_t ota_mode;

    uint32_t all_length;
    uint32_t all_count;
    uint32_t current_count;
    uint32_t crc;

#ifdef OTA_MODEM_RECORD
    uint8_t modem_ota_state;
#endif

    uint8_t image_download_state[DFU_IMG_ID_MAX];

#ifdef OTA_SECTION_CHANGE
    uint32_t download_flash_addr[DFU_IMG_ID_MAX];
    uint32_t download_flash_size[DFU_IMG_ID_MAX];
#endif // OTA_SECTION_CHANGE
    union
    {
        dfu_dl_image_header_t code_img;
    } fw_context;
} dfu_download_progress_t;

typedef struct
{
    // res ota
    uint32_t single_file_len;
    uint32_t single_file_packet_num;
    uint32_t single_file_packet_download_count;
    uint32_t single_file_packet_download_len;
    uint8_t num_of_rsp;

} dfu_res_download_progress_t;

typedef struct
{
    uint32_t version;
    uint32_t update_info;
    char build_hash[8];
    char build_time[16];
} dfu_vesion_t;

typedef struct
{
    // target hcpu
    uint8_t des_state;

    // hcpu, mix, res
    uint8_t dfu_ID;

    // hcpu download state
    uint8_t state;
    // res download state
    uint8_t res_state;

    uint8_t ota_mode;

    // sol use
    uint8_t ota_update;

#ifdef OTA_MODEM_RECORD
    uint8_t modem_ota_state;
#endif

    // sol use
    dfu_vesion_t old_version;
    dfu_vesion_t new_version;

    // sol resume
    uint32_t res_file_total_len;
    uint32_t res_file_total_num;

    // use as lcpu_state
    uint32_t res_file_dowload_count;
    // use as hcpu_target
    uint32_t total_receive_len;

    uint32_t SOL_version;

    uint8_t FW_state;
    uint32_t HW_version;
    uint32_t SDK_version;
    uint32_t FW_version;
    uint8_t FW_key[DFU_KEY_SIZE];

    uint8_t image_download_state[DFU_IMG_ID_MAX];

    uint8_t lcpu_state;
    uint32_t hcpu_target;

#ifdef OTA_SECTION_CHANGE
    uint32_t download_flash_addr[DFU_IMG_ID_MAX];
    uint32_t download_flash_size[DFU_IMG_ID_MAX];
#endif
    union
    {
        dfu_dl_image_header_t code_img;
    } fw_context;
} dfu_download_progress_ext_t;


typedef struct
{
    // @ dfu_ota_state
    uint16_t state;
    uint8_t dfu_ID;
    uint32_t HW_version;
    uint32_t SDK_version;
    uint32_t FW_version;
    uint8_t FW_key[DFU_KEY_SIZE];
    union
    {
        dfu_dl_ota_image_header_t code_img;
    } fw_context;
} dfu_ota_manager_update_state_t;

typedef struct
{
    // use to control iOS retransmission
    // for interrupt situation
    uint8_t retry_count;
    uint32_t success_rsp_count;

    uint32_t last_rsp_pkt_num;
    uint32_t last_rsp_img_length;
} dfu_retransmission_state_t;

typedef struct
{
    uint8_t is_init;
    uint8_t is_force;

    uint8_t current_command;

    // set to prevent reboot to user bin when force update.
    uint8_t is_force_update;

    // use to send total progress
    uint32_t transported_size;

    uint32_t sync_size;
    uint8_t is_sync_timer_on;

    uint8_t flash_erase_state;

    uint8_t resume_async;
    uint8_t resume_status;
    uint8_t resume_is_restart;
    uint8_t is_reboot_timer_on;

    uint8_t resume_image_id;
    uint32_t resume_image_index;

    dfu_ctrl_mode_t mode;
    dfu_callback callback;
    dfu_download_progress_t prog;

    dfu_ota_manager_update_state_t ota_state;
    dfu_retransmission_state_t retrans_state;

    rt_mailbox_t mb_handle;
    rt_thread_t dfu_flash_thread;
#ifdef SF32LB52X
    rt_timer_t  rc10k_time_handle;
    rt_thread_t task_handle;
    rt_event_t  rc10k_event;
#endif
} dfu_ctrl_env_t;

typedef struct
{
    // patch file length
    uint32_t patch_len;

    // new file len
    uint32_t new_file_len;
    uint32_t old_file_len;

    // crtl 0
    uint32_t ctrl[3];

    uint32_t old_pos;
    uint32_t new_pos;









    // record download information
    uint8_t using_patch;
    uint32_t download_addr;
    uint32_t download_size;

    uint32_t ctrl_addr;
    uint32_t ctrl_count;
    uint32_t *ctrl_size;

    uint32_t diff_addr;
    uint32_t diff_size;
    uint32_t diff_index;

    // record diff has read
    // uint8_t diff_index;
    // uint32_t diff_buf_total_len;
    // uint32_t diff_buf_used_len;
    // uint32_t diff_buf_pos;


    // uint32_t extra_addr;
    // uint32_t extra_count;
    // uint32_t *extra_size;

    // uint8_t extra_index;
    // uint32_t extra_buf_total_len;
    // uint32_t extra_buf_used_len;
    // uint32_t extra_buf_pos;


    // // record image's flash addr has read
    // uint32_t image_addr;
    // uint32_t image_start_addr;

    // uint32_t image_buf_start_pos;
    // uint32_t image_buf_end_pos;
    // uint32_t image_buf_len;

    // // new file last write
    // uint32_t image_write_pos;

    // uint32_t old_size;
    // uint32_t new_size;


    // uint32_t erase_start_pos;
    // uint32_t erase_buf_pos;
    // uint32_t erase_len;

    //uint32_t old_size
    //uint32_t dest_len;
} dfu_patch_state_t;


typedef struct
{
    uint8_t is_init;
    uint8_t is_force;

    uint8_t current_command;
    // our status
    uint8_t resume_status;

    uint8_t is_abort;
    uint8_t is_mount;

    uint32_t remote_version;
    // remote command
    uint32_t running_hcpu;
    uint32_t back_up_hcpu;

    // use to send total progress
    uint32_t transported_size;

    uint32_t sync_size;
    uint8_t is_sync_timer_on;
    uint8_t is_force_update;

    dfu_ctrl_mode_t mode;
    dfu_callback_ext callback;

    dfu_res_download_progress_t res_prog;
    dfu_download_progress_ext_t prog;

    //dfu_ota_manager_update_state_t ota_state;
    dfu_retransmission_state_t retrans_state;
    dfu_patch_state_t patch_state;

    rt_mailbox_t mb_handle;
    rt_thread_t dfu_flash_thread;
} dfu_ctrl_ext_env_t;

typedef enum
{
    DFU_FLASH_MSG_TYPE_DATA,
    DFU_FLASH_MSG_TYPE_EXIT,
    DFU_FLASH_MSG_TYPE_VERIFY,
    DFU_FLASH_MSG_TYPE_ERASE,
} dfu_flash_msg_t;

typedef enum
{
    DFU_FLASH_NONE,
    DFU_FLASH_ERASE,
    DFU_FLASH_ERASE_INTERRUPT,
} dfu_flash_erase_value_t;

typedef struct
{
    uint8_t msg_type;
    dfu_image_header_int_t *heade;
    uint32_t offset;
#ifdef OTA_55X
    uint8_t data[512];
#else
    uint8_t data[2048];
#endif
    uint32_t size;
} flash_write_t;


typedef struct
{
    uint8_t msg_type;
    uint32_t base_addr;
    uint32_t offset;
    uint32_t size;
    uint8_t data[2048];
} flash_write_package_t;


typedef struct
{
    uint32_t addr;
    uint32_t size;
} dfu_flash_info_t;

typedef struct
{
    uint32_t magic;
    uint8_t image_count;
    uint32_t info_len;
    uint8_t image_info[0];
} dfu_offline_install_packet_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t install_state;
    uint16_t image_count;
    uint32_t crc;
    uint8_t image_info[0];
} dfu_package_install_packet_t;


typedef struct
{
    uint8_t id;
    uint32_t offset;
    uint32_t len;
} dfu_offline_image_info_t;

typedef struct
{
    uint8_t id;
    uint8_t flag;
    uint32_t offset;
    uint32_t len;
} dfu_package_image_info_t;


int dfu_packet_erase_flash(dfu_image_header_int_t *header, uint32_t offset, uint32_t size);

int dfu_packet_write_flash(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size);

int dfu_packet_read_flash(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size);

int8_t dfu_get_flashid_by_imgid(uint8_t img_id);
int dfu_packet_erase_flash_ext(uint32_t dest, uint32_t offset, uint32_t size, uint8_t type);
int dfu_packet_write_flash_ext(uint32_t dest, uint32_t offset, uint8_t *data, uint32_t size, uint8_t type);
int dfu_packet_read_flash_ext(uint32_t dest, uint32_t offset, uint8_t *data, uint32_t size);

uint32_t dfu_ctrl_convert_ver_2_digit(uint8_t *ver, uint8_t state);

uint32_t dfu_ctrl_parser_SDK_version(uint8_t *str);

uint32_t dfu_ctrl_get_current_HW_version(void);

uint32_t dfu_ctrl_get_current_SDK_version(void);

uint8_t dfu_ctrl_compare_FW_version(void);

dfu_image_header_int_t *dfu_img_get_img_header_by_img_id(dfu_ctrl_env_t *env, uint8_t img_id);
dfu_image_header_int_t *dfu_img_get_img_header_by_img_id_ext(dfu_ctrl_ext_env_t *env, uint8_t img_id);
dfu_image_header_int_t *dfu_img_get_img_header_by_img_id_ota_only(dfu_ctrl_env_t *env, uint8_t img_id);


void dfu_update_img_header(dfu_ctrl_env_t *env);
void dfu_update_img_header_ext(dfu_ctrl_ext_env_t *env);


int8_t dfu_ctrl_ctrl_header_sig_verify(dfu_ctrl_env_t *env, uint8_t *packet, uint16_t total_len, uint8_t *sig);
int8_t dfu_ctrl_ctrl_header_sig_verify_ext(uint8_t *packet, uint16_t total_len, uint8_t *sig);

int dfu_encrypt_packet(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size, uint8_t *dfu_key);

uint8_t dfu_img_verification(dfu_ctrl_env_t *env);
uint8_t dfu_img_verification_ext(dfu_ctrl_ext_env_t *env);

uint8_t *dfu_dec_verify(uint8_t *key, uint32_t offset,
                        uint8_t *in_data, uint8_t *out_data, int size, uint8_t *hash);


int8_t dfu_integrate_verify(uint8_t *in_data, int size, uint8_t *hash);

int dfu_img_install(dfu_ctrl_env_t *env);

int dfu_img_install_lcpu_rom_patch(dfu_ctrl_env_t *env);

int8_t dfu_protocol_packet_send(uint8_t *data);

uint8_t *dfu_protocol_packet_buffer_alloc(dfu_protocol_msg_id_t msg_id, uint16_t length);

int8_t dfu_protocol_reset_env_prepare(void);

void dfu_protocol_packet_handler(dfu_tran_protocol_t *msg, uint16_t length);
void dfu_protocol_packet_handler_ext(dfu_tran_protocol_t *msg, uint16_t length);

void dfu_ctrl_update_prog_info(dfu_ctrl_env_t *env);

void dfu_sec_init(void);

void dfu_sec_config_malloc();

void dfu_sec_config_free();

int8_t dfu_protocol_session_close(void);

void dfu_bootjump_sec_config(dfu_ctrl_env_t *env, uint8_t *dest);

uint8_t is_addr_in_flash(uint32_t addr);

void dfu_protocol_close_handler(void);

void dfu_set_reboot_after_disconnect();

void dfu_port_svc_session_close();

uint8_t dfu_get_progress_mode();

void dfu_link_sync_start(uint8_t sync_type);

uint16_t ble_dfu_protocl_get_supervision_timeout();

void ble_dfu_request_connection_priority();

uint8_t dfu_flash_addr_get(uint8_t img_id, dfu_flash_info_t *info);
uint8_t dfu_flash_addr_set(uint8_t img_id, uint32_t addr, uint32_t size);
void dfu_flash_addr_reset(uint8_t mode);

uint8_t dfu_get_hcpu_overwrite_mode();
uint8_t dfu_set_hcpu_overwrite_mode(uint8_t mode);

int dfu_flash_read(uint32_t addr, uint8_t *data, int size);

int dfu_flash_write(uint32_t addr, uint8_t *data, int size);
int dfu_flash_erase(uint32_t dest, uint32_t size);


int dfu_get_version(uint8_t *version, uint8_t size);

uint8_t dfu_ctrl_ext_get_target_hcpu();

void dfu_serial_transport_error_handle(uint8_t error);

uint32_t dfu_get_hcpu_download_addr();

void dfu_protocol_abort_command(uint16_t reason);

uint8_t dfu_running_image_switch();

uint8_t dfu_set_last_packet_wait();

void dfu_ctrl_last_packet_handler();

uint8_t dfu_record_current_tx();

// get file res state, see @dfu_ctrl_state_t
uint8_t dfu_get_res_state();

// get image state, see @dfu_ctrl_state_t
uint8_t dfu_get_image_state();

void dfu_set_ota_mode(uint8_t mode);

uint8_t dfu_get_ota_mode();

void dfu_fs_mount_status_set();

void dfu_read_storage_data(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size);

void dfu_set_ota_version(dfu_vesion_t *old_version, dfu_vesion_t *new_version);

void dfu_set_ota_update_state(uint8_t ota_update);

dfu_vesion_t dfu_get_ota_old_version();

dfu_vesion_t dfu_get_ota_new_version();

uint8_t dfu_get_ota_update_state();

void set_image_offset(uint32_t offset);

void dfu_offline_install_start();

#ifdef OTA_MODEM_RECORD
    uint8_t dfu_get_modem_state();
    uint8_t dfu_set_modem_state(uint8_t state);
#endif

uint8_t dfu_get_download_state();

int dfu_image_install_flash_package(dfu_ctrl_env_t *env, uint8_t image_id, uint32_t length, uint32_t image_offset, uint8_t image_flag);

uint32_t dfu_crc32mpeg2(uint8_t *data, uint32_t len);

int8_t dfu_get_flash_type(uint32_t dest);

uint32_t dfu_get_download_addr_by_imgid(uint8_t img_id, uint8_t flag);

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);

#endif //__DFU_INTERNAL_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
