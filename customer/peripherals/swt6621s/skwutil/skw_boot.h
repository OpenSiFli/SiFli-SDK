/*****************************************************************
 *Copyright (C) 2021 Seekwave Tech Inc.
 *Filename : skw_sdio.h
 *Authors:seekwave platform
 *
 * This software is licensed under the terms of the the GNU
 * General Public License version 2, as published by the Free
 * Software Foundation, and may be copied, distributed, and
 * modified under those terms.
 *
 * This program is distributed in the hope that it will be usefull,
 * but without any warranty;without even the implied warranty of
 * merchantability or fitness for a partcular purpose. See the
 * GUN General Public License for more details.
 * **************************************************************/
#ifndef __SKW_BOOT_H__
#define __SKW_BOOT_H__

#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>

#include "boot_config.h"
#include "skw_sdio.h"

/****************************************************************
 *Description:the skwsdio log define and the skwsdio data debug,
 *Func: skwsdio_log, skwsdio_err, skwsdio_data_pr;
 *Calls:
 *Call By:
 *Input: skwsdio log debug informations
 *Output:
 *Return：
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-08-25
 * **************************************************************/
#define skwboot_log(fmt, args...) \
    LOG_D("[SKWBOOT]:" fmt, ## args)

#define skwboot_err(fmt, args...) \
    LOG_E("[SKWBOOT_ERR]:" fmt, ## args)

#define skwboot_warn(fmt, args...) \
    LOG_D("[SKWBOOT_WARN]:" fmt, ## args)


/**********************sdio boot interface start******************/

#define SKW_BOOT_START_ADDR         0x100000
#define SKW_CHIP_ID                 0x40000000  //SV6160 chip id
#define SKW_PACKET_NUM_OFFSET       508

/*add the 32bit*4 128bit */
struct img_head_data_t
{
    unsigned int index;
    unsigned int dl_addr;
    unsigned int data_size;
    unsigned int write_addr;
};
/*add the 32bit*4 128bit */
struct img_dl_data
{
    unsigned int dl_addr;
    unsigned int dl_info; /*type and the size*/
    unsigned int write_addr;
};

#define CRC_16_L_SEED   0x80
#define CRC_16_L_POLYNOMIAL  0x8000
#define CRC_16_POLYNOMIAL  0x1021
#define IRAM_CRC_OFFSET  0
#define DRAM_CRC_OFFSET  0

#define SKW_FIRST_BOOT  0
#define SKW_BSP_BOOT    1
#define SKW_WIFI_BOOT   2
#define SKW_BT_BOOT     3
#define RECOVERY_BOOT   4
#define NONE_BOOT       5
/*slp reg add the ap send the irq to cp reg*/
#define SKW_SDIO_PD_DL_AP2CP_BSP        0x160 //download done  or first boot setup addrn
#define SDIOHAL_PD_DL_AP2CP_WIFI        0x161
#define SDIOHAL_PD_DL_AP2CP_BT          0x162
#define SDIOHAL_PD_DL_ALL               0x163
#define SKW_SDIO_DL_POWERON_MODULE      0x164 //Poweron CP Moudle  1 WIFI 2:BT
#define SKW_SDIO_PLD_DMA_TYPE           0x165
#define SDIOHAL_CPLOG_TO_AP_SWITCH      0x166
#define SKW_SDIO_CP_SLP_SWITCH          0x167 //Turn on/off the CP slp feature 1:dis slp 0:enb slp
#define SKW_SDIO_CREDIT_TO_CP           0x168

// CP signal 3
#define SKW_SDIO_RX_CHANNEL_FTL0        0x16C
#define SKW_SDIO_RX_CHANNEL_FTL1        0x16D

/*slp reg get the cp dl state reg*/
#define SKW_SDIO_DL_CP2AP_BSP           0x180 //poweron OK ? 1: WIFI 2:BT
#define SKW_SDIO_CP2AP_FIFO_IND         0x181 //CP_RX FIFO Empty Indiacation.
#define SDIOHAL_PD_DL_CP2AP_BT          0x182
#define SDIOHAL_PD_DL_CP2AP_ALL         0x183
#define SDIOHAL_PD_DL_CP2AP_SIG4        0x184
#define SDIOHAL_PD_DL_CP2AP_SIG5        0x185
#define SDIOHAL_PD_DL_CP2AP_SIG6        0x186
#define SDIOHAL_PD_DL_CP2AP_SIG7        0x187

#define SKWSDIO_AP2CP_IRQ               0x1b0  //AP to CP interrupt and used BIT4 set 1 :fifth bit

#define NV_CMFG_OFFSET  0x8
#define NV_CMFG_SIZE    0xC
#define NV_PNFG_OFFSET  0x10
#define NV_PNFG_SIZE    0x14
#define NV_HEADER_SIZE  0x20

#define PN_CNT  20
#define PN_FUNCSEL_ONEGRP_CNT   10
#define PN_FUNC_SEL0_OFFSET 0x5c
#define PN_FUNC_SEL1_OFFSET 0x60
#define SKW_PINREG_BASE 0x40102000
#define SKW_DL_FLAG_BASE 0x40100030
#define SKW_DL_FLAG_BIT_MASK BIT(8)

#define BIT_PN_DSLP_EN_START    17
#define BIT_PN_DSLP_EN_END  21
#define BIT_DRV_STREN_START 14
#define BIT_DRV_STREN_END   16
#define BIT_NORMAL_WP_START 8
#define BIT_NORMAL_WP_END   10
#define BIT_SCHMITT_START   11
#define BIT_SCHMITT_END 11
#define BIT_SLP_WP_START    2
#define BIT_SLP_WP_END  4
#define BIT_SLEEP_IE_OE_START   0
#define BIT_SLEEP_IE_OE_END 1

enum dma_type_en
{
    ADMA = 1,
    SDMA,
};
enum skw_service_ops
{
    SKW_NO_SERVICE = 0,
    SKW_WIFI_START,
    SKW_WIFI_STOP,
    SKW_BT_START,
    SKW_BT_STOP,
};

enum skw_subsys
{
    SKW_BOOT = 0,
    SKW_BSP,
    SKW_WIFI,
    SKW_BT,
    SKW_ALL,
};


struct seekwave_device
{
    struct  rt_device_t *pdev;
    char *skw_nv_name;
    char *iram_file_path;
    char *dram_file_path;
    char *img_data;
    char *iram_img_data;
    char *dram_img_data;
    char *dl_base_img;//
    char *nv_mem_data;
    char *nv_mem_cmfg_data;
    char *nv_mem_pnfg_data;
    void *dl_bin;//load the img
    int (*wifi_start)(void);
    int (*bt_start)(void);
    int (*wifi_stop)(void);
    int (*bt_stop)(void);
    int (*skw_dloader_module)(int service_index);
    u32 nv_mem_cmfg_size;
    u32 nv_mem_pnfg_size;
    unsigned short iram_crc_val;
    unsigned short dram_crc_val;
    unsigned short nvmem_crc_val;
    unsigned int iram_dl_addr;
    unsigned int iram_dl_size;
    unsigned int iram_crc_offset;
    unsigned int iram_crc_en;
    unsigned int dram_dl_addr;
    unsigned int dram_dl_size;
    unsigned int dram_crc_offset;
    unsigned int dram_crc_en;
    unsigned int setup_addr;//setup address
    unsigned int save_setup_addr;//send the setup address register
    unsigned int first_dl_flag;
    unsigned int first_boot_flag;
    unsigned int dl_module;
    unsigned int dma_type_addr;//1:ADMA,2:SDMA
    unsigned int dma_type;//1:ADMA,2:SDMA
    unsigned int slp_disable;//0:disable,1:enable
    unsigned int slp_disable_addr;
    unsigned int head_addr;
    unsigned int tail_addr;
    unsigned int bsp_head_addr;
    unsigned int bsp_tail_addr;
    unsigned int wifi_head_addr;
    unsigned int wifi_tail_addr;
    unsigned int bt_head_addr;
    unsigned int bt_tail_addr;
    unsigned int nv_mem_addr;
    unsigned int nv_mem_size;
    unsigned int nv_head_addr;
    unsigned int nv_tail_addr;
    unsigned int nv_data_size;
    unsigned int nvmem_crc_en;
    unsigned int nvmem_crc_offset;
    unsigned int chip_id;
    unsigned int fpga_debug;
    unsigned int bt_antenna;
    unsigned int dl_offset_addr;
    unsigned int dl_base_addr;
    unsigned int dl_addr;//
    unsigned int dl_acount_addr;
    unsigned int dl_size;
    int img_size;
    int bsp_index_count;
    int bt_index_count;
    int wifi_index_count;
    int all_dl_count;
    int bt_dl_count;
    int wifi_dl_count;
    int host_gpio;/*GPIO0_A3*/
    int chip_gpio;/*GPIO2_D2*/
    int chip_en;/*GPIO0_B1*/
    int bt_service_state;
    int wifi_service_state;
    int service_ops;
    int dl_done_signal;
    int gpio_out;//host wakeup gpio0:/*GPIO0_A3*/,chip_wakeup gpio2:/*GPIO2_D2*/
    int gpio_in;//host wakeup gpio 0
    int gpio_val;
    int gpio_next_val;

};
#endif
