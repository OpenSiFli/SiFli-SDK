/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <board.h>
#include <string.h>
#include "stdlib.h"
#include "register.h"
#include "../dfu/dfu.h"
#include "boot_flash.h"
#include "secboot.h"
#include "../dfu/dfu_protocol.h"
#include "transport.h"
#include "cdc_acm_transport.h"
#include "serial_transport.h"

#define CMD_BUF_SIZE 32
#define CMD_ARG_MAX 3

#define BOOT_MODE_DELAY 100000

typedef struct
{
    uint8_t pos;
    char buf[CMD_BUF_SIZE];
} cmd_buf_t;

typedef void (*ram_hook_handler)(void);

uint32_t rx_data_buf_len(void);
uint32_t rx_data_buf_get(uint8_t *buf, uint32_t len);

static cmd_buf_t cdc_acm_cmd_buf;
static cmd_buf_t serial_cmd_buf;
static uint8_t dfu_data[DFU_MAX_BLK_SIZE];

void HAL_MspInit(void);

extern flash_read_func g_flash_read;
extern flash_write_func g_flash_write;
extern flash_erase_func g_flash_erase;
struct sec_configuration g_temp_sec_config;
struct sec_configuration *temp_sec_config;

ALIGN(4) struct sec_configuration sec_config_cache;

#ifdef CFG_BOOTROM
    __attribute__((used))
    const char *const bootrom_ver = "v1.1";
#endif /* CFG_BOOTROM */

void boot_ram(void)
{
    sboot_standby_boot_tbl_t *boot_tbl = (sboot_standby_boot_tbl_t *)hwp_hpsys_aon->RESERVE0;
    ram_hook_handler hook;
    uint32_t ram_code_size;

    if (boot_tbl)
    {
        hook = (ram_hook_handler)(boot_tbl->code_jump_addr);
        if (sboot_ctx.sec_en)
        {
            ram_code_size = boot_tbl->code_size;
            if (!sifli_verify_img_cmac_hash(NULL, (uint8_t *)boot_tbl->code_start_addr, ram_code_size,
                                            boot_tbl->code_cmac_hash))
            {
                hook = NULL;
                /* verify failure */
                printf("ram hook verify failure\n");
                while (1);
            }
        }
        if (hook)
        {
            hook();
        }
    }
}


//#define BOOT_TEST
#ifdef BOOT_TEST
void boot_test(void)
{
    uint32_t delay = (100 << BOOT_PU_Delay_Pos) | (200 << BOOT_PD_Delay_Pos);
    if (HAL_Get_backup(RTC_BACKUP_BOOTOPT + 1) == 0)
    {
        HAL_Set_backup(RTC_BACKUP_BOOTOPT, delay);
        HAL_Set_backup(RTC_BACKUP_BOOTOPT + 1, 1);
        HAL_PMU_Reboot();
    }
    else
    {
        boot_uart_tx(hwp_usart1, (uint8_t *)"E", 1);
        __asm("B .");
    }
}
#else
#define boot_test()
#endif

#ifndef CFG_BOOTROM
static void update_sec_flash(struct sec_configuration *sec_config)
{
    uint32_t flash_addr = g_config_addr;
    //uint32_t size = (sizeof(struct sec_configuration) + SPI_NAND_PAGE_SIZE * 2) & (~(SPI_NAND_PAGE_SIZE * 2 - 1));
    uint32_t size = sizeof(struct sec_configuration);
    if (size % 4096 != 0)
    {
        size = (size + 4096) / 4096 * 4096;
    }
    g_flash_erase(flash_addr, size);
    g_flash_read(flash_addr, (const int8_t *)sec_config, sizeof(struct sec_configuration));
}

static void select_boot()
{
    uint8_t hcpu_des = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    uint32_t hcpu_len = HAL_Get_backup(RTC_BAKCUP_OTA_FORCE_MODE);
    temp_sec_config = &g_temp_sec_config;

    struct sec_configuration *flash_config = (struct sec_configuration *)g_config_addr;
    //memcpy((const int8_t *)temp_sec_config, (uint8_t *)MPI5_MEM_BASE, sizeof(struct sec_configuration));
    g_flash_read(g_config_addr, (const int8_t *)temp_sec_config, sizeof(struct sec_configuration));

    int hcpu1_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU);
    int hcpu2_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2);
    // HAL_sw_breakpoint();

    if (sec_config_cache.running_imgs[CORE_HCPU] == &(flash_config->imgs[hcpu1_img_idx]))
    {
        // now we are running on hcpu1
        switch (hcpu_des)
        {
        case DFU_DES_RUNNING_ON_HCPU1:
        {
            // normal power on hcpu1
            break;
        }
        case DFU_DES_SWITCH_TO_HCPU2:
        {
            //HAL_sw_breakpoint();
            // switch to hcpu2

            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);

            hcpu_des = DFU_DES_UPDATE_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            if (hcpu_len != 0)
            {
                temp_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2)].length = hcpu_len;
            }

            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_UPDATE_HCPU2:
        {
            // switch to hcpu2 fail, back to hcpu1
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_NONE:
        {
            // first power on, use current
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
            break;
        }
        default:
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
        }

    }
    else if (sec_config_cache.running_imgs[CORE_HCPU] == &(flash_config->imgs[hcpu2_img_idx]))
    {
        // now we are running on hcpu2
        switch (hcpu_des)
        {
        case DFU_DES_RUNNING_ON_HCPU2:
        {
            // normal power on hcpu2
            break;
        }
        case DFU_DES_SWITCH_TO_HCPU1:
        {
            // switch to hcpu1
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            hcpu_des = DFU_DES_UPDATE_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            if (hcpu_len != 0)
            {
                temp_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU)].length = hcpu_len;
            }

            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_UPDATE_HCPU1:
        {
            // switch to hcpu1 fail, back to hcpu2
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_NONE:
        {
            // first power on, use current
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
            break;
        }
        default:
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
        }
    }
}
#endif /* CFG_BOOTROM */


void boot_images_help()
{
    if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
    {
#ifdef CFG_BOOTROM
        if (sec_config_cache.running_imgs[CORE_BL] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_BL] - g_config_addr - 0x1000) / sizeof(struct image_header_enc)
                           + DFU_FLASH_IMG_LCPU;
            dfu_boot_img_in_flash(flash_id);
        }
#else

        dfu_install_info info = {0};
        dfu_install_info info_ext = {0};

        if (DFU_DOWNLOAD_REGION_START_ADDR != FLASH_UNINIT_32)
        {
            g_flash_read(DFU_DOWNLOAD_REGION_START_ADDR, (const int8_t *)&info, sizeof(dfu_install_info));
        }
        if (DFU_INFO_REGION_START_ADDR != FLASH_UNINIT_32)
        {
            g_flash_read(DFU_INFO_REGION_START_ADDR, (const int8_t *)&info_ext, sizeof(dfu_install_info));
        }
        if (info.magic == SEC_CONFIG_MAGIC && info_ext.magic == SEC_CONFIG_MAGIC)
        {
            info = info_ext;
        }

        select_boot();

#ifdef DFU_LOADER_START_ADDR
        /* UART-DFU trigger (at_app AT+OTA): needs_update=1 at the DFU_LOADER
         * trailer (end - 0x2000, field offset 288) -> boot the DFU loader image
         * (LCPU slot) instead of main. Compiled only for boards whose ptab
         * defines a DFU_LOADER partition. */
        if (DFU_LOADER_START_ADDR != FLASH_UNINIT_32 &&
            DFU_LOADER_SIZE != FLASH_UNINIT_32)
        {
            uint32_t dfu_needs_update = 0;
            g_flash_read(DFU_LOADER_START_ADDR + DFU_LOADER_SIZE - 0x2000 + 288,
                         (const int8_t *)&dfu_needs_update, sizeof(dfu_needs_update));
            if (dfu_needs_update == 1)
            {
                sec_config_cache.running_imgs[CORE_HCPU] = (struct image_header_enc *) & (((struct sec_configuration *)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)]);
            }
        }
#endif

        if (DFU_DOWNLOAD_REGION_START_ADDR != FLASH_UNINIT_32)
        {
            if ((HAL_Get_backup(RTC_BAKCUP_OTA_FORCE_MODE) == DFU_FORCE_MODE_REBOOT_TO_PACKAGE_OTA_MANAGER) ||
                    (info.magic == SEC_CONFIG_MAGIC) && (info.install_state == DFU_PACKAGE_INSTALL))
            {
                sec_config_cache.running_imgs[CORE_HCPU] = (struct image_header_enc *) & (((struct sec_configuration *)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)]);
            }
        }

        board_init_psram();
        /* load extended img if present, such as ACPU img */
        if (sec_config_cache.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
        {
            dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
        }

        if (sec_config_cache.running_imgs[CORE_HCPU] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_HCPU] - g_config_addr - 0x1000) / sizeof(struct image_header_enc) + DFU_FLASH_IMG_LCPU;

            dfu_boot_img_in_flash(flash_id);
        }
#endif
    }
}

#ifdef __CC_ARM                                                 /*ARMCC*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /*ARMCLANG*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif __ICCARM__                                                /*IAR**/
#error Not support yet
#else                                                           /*GCC*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int _write(int fd, char *ptr, int len)
{
    if (fd > 2)
    {
        return -1;
    }
    boot_uart_tx(hwp_usart1, (uint8_t *)ptr, len);
    return len;
}
#endif

PUTCHAR_PROTOTYPE
{
    boot_uart_tx(hwp_usart1, (uint8_t *)&ch, 1);
    return ch;
}

#ifdef CFG_BOOTROM
static uint32_t cmd_split(char *cmd, uint32_t length, char *argv[CMD_ARG_MAX])
{
    char *ptr;
    uint32_t position;
    uint32_t argc;
    uint32_t i;

    ptr = cmd;
    position = 0;
    argc = 0;

    while (position < length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && position < length)
        {
            *ptr = '\0';
            ptr ++;
            position ++;
        }

        if (argc >= CMD_ARG_MAX)
        {
            printf("Too many args ! We only Use:\n");
            for (i = 0; i < argc; i++)
            {
                printf("%s ", argv[i]);
            }
            printf("\n");
            break;
        }

        if (position >= length) break;

        /* handle string */
        if (*ptr == '"')
        {
            ptr ++;
            position ++;
            argv[argc] = ptr;
            argc ++;

            /* skip this string */
            while (*ptr != '"' && position < length)
            {
                if (*ptr == '\\')
                {
                    if (*(ptr + 1) == '"')
                    {
                        ptr ++;
                        position ++;
                    }
                }
                ptr ++;
                position ++;
            }
            if (position >= length) break;

            /* skip '"' */
            *ptr = '\0';
            ptr ++;
            position ++;
        }
        else
        {
            argv[argc] = ptr;
            argc ++;
            while ((*ptr != ' ' && *ptr != '\t') && position < length)
            {
                ptr ++;
                position ++;
            }
            if (position >= length) break;
        }
    }

    return argc;
}

int32_t handle_cmd_dfu_recv(transport_t *transport, uint32_t argc, char **argv)
{
    int r = DFU_FAIL;
    int32_t offset = 0;
    int32_t delta;

    if (argc == 2)
    {
        int len = atoi(argv[1]);
        if (len < DFU_MAX_BLK_SIZE)
        {
            while (offset < len)
            {
                delta = xport_rx_buf_get(transport, &dfu_data[offset], len - offset);
                offset += delta;
            }
            r = dfu_receive_pkt(len, dfu_data);
        }
    }
    if (r == DFU_SUCCESS)
    {
        xport_send(transport, "OK\n", 3);
    }
    else
    {
        xport_send(transport, "Fail\n", 5);
    }

    return r;

}


void handle_cmd(transport_t *transport, char *cmd, uint32_t len)
{
    char *argv[CMD_ARG_MAX];
    uint32_t argc;

    if (len == 0)
    {
        return;
    }

    /* strim the beginning of command */
    while ((*cmd  == ' ') || (*cmd == '\t'))
    {
        cmd++;
        len--;
    }

    if (len == 0)
    {
        return;
    }

    memset(argv, 0x0, sizeof(argv));
    argc = cmd_split(cmd, len, argv);
    if (argc == 0)
    {
        return;
    }

    if (strncmp((const char *)argv[0], "dfu_recv", 8) == 0)
    {
        handle_cmd_dfu_recv(transport, argc, argv);
    }
}

void handle_rx_data(transport_t *transport, cmd_buf_t *cmd_buf)
{
    char ch;
    uint32_t len;

    HAL_ASSERT(transport && cmd_buf);

    while (1)
    {
        len = xport_rx_buf_get(transport, (uint8_t *)&ch, 1);
        if (len == 0)
        {
            break;
        }
        if ((ch == '\r') || (ch == '\n'))
        {
            handle_cmd(transport, cmd_buf->buf, cmd_buf->pos);
            memset(cmd_buf->buf, 0, CMD_BUF_SIZE);
            cmd_buf->pos = 0;
        }
        if (cmd_buf->pos >= CMD_BUF_SIZE)
        {
            cmd_buf->pos = 0;
        }
        if (((ch >= 'a') && (ch <= 'z'))
                || ((ch >= 'A') && (ch <= 'Z'))
                || ((ch >= '0') && (ch <= '9'))
                || (ch == ' ') || (ch == '\t') || (ch == '_'))
        {
            cmd_buf->buf[cmd_buf->pos] = ch;
            cmd_buf->pos++;
            /* leave one byte for null terminator */
            if (cmd_buf->pos >= CMD_BUF_SIZE)
            {
                cmd_buf->pos = 0;
            }
        }
    }
}

static bool boot_is_bootmode(void)
{
    bool is_bootmode;

    if (hwp_hpsys_cfg->BMR)
    {
        is_bootmode = true;
    }
    else
    {
        is_bootmode = false;
    }

    return is_bootmode;
}

#endif /* CFG_BOOTROM */

#ifdef CFG_BOOTROM
static void print_boot_info(void)
{
    printf("SFBL\n");
    HAL_Delay_us(BOOT_MODE_DELAY);      // Wait for boot_mode options.
}

static void print_uid(void)
{
    uint8_t uid[EFUSE_UID_BYTE_SIZE];
    int r;
    r = sifli_hw_efuse_read(EFUSE_UID, uid, EFUSE_UID_BYTE_SIZE);
    if (EFUSE_UID_BYTE_SIZE == r)
    {
        r--;
        for (; r >= 0; r--)
        {
            printf("%02X", uid[r]);
        }
        printf("\n");
    }
}
#endif /* CFG_BOOTROM */

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    int main(void)
#elif defined(__ICCARM__)
    int __low_level_init(void)
#elif defined(__GNUC__)
    int entry(void)
#endif
{
#ifdef CFG_BOOTROM
    uint32_t boot_opt;

    /* pull up power control pin by default */
    HAL_PIN_CompileTimeSet(MPI_POWER_PAD, MPI_POWER_PAD_FUNC, PIN_PULLUP, 1);
    HAL_PMU_ClearPadRetention(MPI_POWER_PAD);

    /* disable RTO of UART PIN */
    HAL_PMU_ClearPadRetention(PAD_PA18);
    HAL_PMU_ClearPadRetention(PAD_PA19);

    sboot_init();

    /* init AES_ACC as normal mode */
    __HAL_SYSCFG_CLEAR_SECURITY();

    /* If ram hook existed, just jump to ram. */
    boot_ram();

    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_Delay_us(0);

    print_boot_info();

    /* init pmcu ldo en ss register according to efuse setting */
    board_ldo_en_ss_init();

    if (!boot_is_bootmode())
    {
        board_init();

        board_boot_device = board_boot_from();

        if (boot_device_init())
        {
            /* set default delay value to 5*100us for nor flash reset */
            boot_opt = HAL_Get_backup(RTC_BACKUP_BOOTOPT);
            MODIFY_REG(boot_opt, RTC_BACKUP_BOOTOPT_NOR_RESET_DELAY_Msk, MAKE_REG_VAL2(5, RTC_BACKUP_BOOTOPT_NOR_RESET_DELAY));
            HAL_Set_backup(RTC_BACKUP_BOOTOPT, boot_opt);
            boot_images_help();
        }
    }

    print_uid();

    dfu_init();

    /* USB pin */
    HAL_PIN_Set_Analog(PAD_PA35, 1);                    // USB_DP
    HAL_PIN_Set_Analog(PAD_PA36, 1);                    // USB_DM

    cdc_acm_init(0, (uintptr_t)USBC_BASE);

    serial_transport_init();

    while (1)
    {
        if (xport_rx_buf_len(&usb_cdc_acm_transport) > 0)
        {
            handle_rx_data(&usb_cdc_acm_transport, &cdc_acm_cmd_buf);
        }
        if (xport_rx_buf_len(&serial_transport) > 0)
        {
            handle_rx_data(&serial_transport, &serial_cmd_buf);
        }
    }
#else
    sboot_init();

    board_init();

    board_boot_device = board_boot_from();

    if (boot_device_init())
    {
        boot_images_help();
    }

    while (1)
    {
    }
#endif /* CFG_BOOTROM */

    return HAL_OK;
}


