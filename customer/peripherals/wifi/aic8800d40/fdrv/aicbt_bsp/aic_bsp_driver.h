/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC_BSP_DRIVER_H
#define _AIC_BSP_DRIVER_H


#include "aic_fw.h"

#define RAM_FMAC_FW_ADDR            0x00120000
#define FW_RAM_ADID_BASE_ADDR       0x00161928
#define FW_RAM_ADID_BASE_ADDR_U03   0x00161928
#define FW_RAM_PATCH_BASE_ADDR      0x00100000
#define RAM_8800DC_U01_ADID_ADDR    0x00101788
#define RAM_8800DC_U02_ADID_ADDR    0x001017d8
#define RAM_8800DC_FW_PATCH_ADDR    0x00184000
#define FW_RESET_START_ADDR         0x40500128
#define FW_RESET_START_VAL          0x40
#define FW_ADID_FLAG_ADDR           0x40500150
#define FW_ADID_FLAG_VAL            0x01
#define FW_8800D80_RAM_ADID_BASE_ADDR_U02       0x00201940
#define FW_8800D80_RAM_PATCH_BASE_ADDR          0x001e0000

#define AICBT_PT_TAG                "AICBT_PT_TAG"


/*****************************************************************************
 * Addresses within RWNX_ADDR_CPU
 *****************************************************************************/
#define RAM_LMAC_FW_ADDR               0x00150000

#define ROM_FMAC_FW_ADDR               0x00010000
#define ROM_FMAC_PATCH_ADDR            0x00180000

#define RWNX_MAC_FW_RF_BASE_NAME_8800DC   "lmacfw_rf_8800dc.bin"

#ifdef CONFIG_FOR_IPCOM
#define RWNX_MAC_PATCH_BASE_NAME_8800DC        "fmacfw_patch_8800dc_ipc"
#define RWNX_MAC_PATCH_NAME2_8800DC RWNX_MAC_PATCH_BASE_NAME_8800DC".bin"
#else
#define RWNX_MAC_PATCH_BASE_NAME_8800DC        "fmacfw_patch_8800dc"
#define RWNX_MAC_PATCH_NAME2_8800DC RWNX_MAC_PATCH_BASE_NAME_8800DC".bin"
#define RWNX_MAC_PATCH_NAME2_8800DC_U02 RWNX_MAC_PATCH_BASE_NAME_8800DC"_u02.bin"
#endif

#define RWNX_MAC_PATCH_TABLE_NAME_8800DC "fmacfw_patch_tbl_8800dc"
#define RWNX_MAC_PATCH_TABLE_8800DC RWNX_MAC_PATCH_TABLE_NAME_8800DC ".bin"
#define RWNX_MAC_PATCH_TABLE_8800DC_U02 RWNX_MAC_PATCH_TABLE_NAME_8800DC "_u02.bin"

#define RWNX_MAC_RF_PATCH_BASE_NAME_8800DC     "fmacfw_rf_patch_8800dc"
#define RWNX_MAC_RF_PATCH_NAME_8800DC RWNX_MAC_RF_PATCH_BASE_NAME_8800DC".bin"
#define FW_USERCONFIG_NAME_8800DC         "aic_userconfig_8800dc.txt"

enum aicbt_patch_table_type {
	AICBT_PT_INF  = 0x00,
	AICBT_PT_TRAP = 0x1,
	AICBT_PT_B4,
	AICBT_PT_BTMODE,
	AICBT_PT_PWRON,
	AICBT_PT_AF,
	AICBT_PT_VER,
};

enum aicbt_btport_type {
	AICBT_BTPORT_NULL,
	AICBT_BTPORT_MB,
	AICBT_BTPORT_UART,
};

/*  btmode
 * used for force bt mode,if not AICBSP_MODE_NULL
 * efuse valid and vendor_info will be invalid, even has beed set valid
*/
enum aicbt_btmode_type {
	AICBT_BTMODE_BT_ONLY_SW = 0x0,    // bt only mode with switch
	AICBT_BTMODE_BT_WIFI_COMBO,       // wifi/bt combo mode
	AICBT_BTMODE_BT_ONLY,             // bt only mode without switch
	AICBT_BTMODE_BT_ONLY_TEST,        // bt only test mode
	AICBT_BTMODE_BT_WIFI_COMBO_TEST,  // wifi/bt combo test mode
	AICBT_BTMODE_BT_ONLY_COANT,       // bt only mode with no external switch
	AICBT_MODE_NULL = 0xFF,           // invalid value
};

/*  uart_baud
 * used for config uart baud when btport set to uart,
 * otherwise meaningless
*/
enum aicbt_uart_baud_type {
	AICBT_UART_BAUD_115200     = 115200,
	AICBT_UART_BAUD_921600     = 921600,
	AICBT_UART_BAUD_1_5M       = 1500000,
	AICBT_UART_BAUD_3_25M      = 3250000,
};

enum aicbt_uart_flowctrl_type {
	AICBT_UART_FLOWCTRL_DISABLE = 0x0,    // uart without flow ctrl
	AICBT_UART_FLOWCTRL_ENABLE,           // uart with flow ctrl
};

enum aicbsp_cpmode_type {
	AICBSP_CPMODE_WORK,
	AICBSP_CPMODE_TEST,
	AICBSP_CPMODE_MAX,
};

enum chip_rev {
	CHIP_REV_U01 = 1,
	CHIP_REV_U02 = 3,
	CHIP_REV_U03 = 7,
	CHIP_REV_U04 = 7,
};
///aic bt tx pwr lvl :lsb->msb: first byte, min pwr lvl; second byte, max pwr lvl;
///pwr lvl:20(min), 30 , 40 , 50 , 60(max)
#define AICBT_TXPWR_LVL            0x00006020
#define AICBT_TXPWR_LVL_8800dc            0x00006f2f

#define AICBSP_HWINFO_DEFAULT       (-1)
#define AICBSP_CPMODE_DEFAULT       AICBSP_CPMODE_WORK
#define AICBSP_FWLOG_EN_DEFAULT     0

#define AICBT_BTMODE_DEFAULT        AICBT_BTMODE_BT_ONLY_SW
#define AICBT_BTPORT_DEFAULT        AICBT_BTPORT_UART
#define AICBT_UART_BAUD_DEFAULT     AICBT_UART_BAUD_921600
#define AICBT_UART_FC_DEFAULT       AICBT_UART_FLOWCTRL_ENABLE
#define AICBT_LPM_ENABLE_DEFAULT    0
#define AICBT_TXPWR_LVL_DEFAULT     AICBT_TXPWR_LVL

#define FEATURE_SDIO_CLOCK          50000000 // 0: default, other: target clock rate
#define FEATURE_SDIO_PHASE          2        // 0: default, 2: 180°

struct aicbt_patch_table {
	char     *name;
	uint32_t type;
	uint32_t *data;
	uint32_t len;
	struct aicbt_patch_table *next;
};

struct aicbt_info_t {
	uint32_t btmode;
	uint32_t btport;
	uint32_t uart_baud;
	uint32_t uart_flowctrl;
	uint32_t lpm_enable;
	uint32_t txpwr_lvl;
};

struct aicbt_patch_info_t {
    uint32_t info_len;
//base len start
    uint32_t adid_addrinf;
    uint32_t addr_adid;
    uint32_t patch_addrinf;
    uint32_t addr_patch;
    uint32_t reset_addr;
    uint32_t reset_val;
    uint32_t adid_flag_addr;
    uint32_t adid_flag;
//base len end
//ext patch nb
    uint32_t ext_patch_nb_addr;
    uint32_t ext_patch_nb;
    uint32_t *ext_patch_param;
};

struct aicbsp_firmware {
	const char *desc;
	const enum aic_fw bt_adid;
	const enum aic_fw bt_patch;
	const enum aic_fw bt_table;
	const enum aic_fw wl_fw;
};

struct aicbsp_info_t {
	int hwinfo;
	int hwinfo_r;
	uint32_t cpmode;
	uint32_t chip_rev;
	bool fwlog_en;
};

extern struct aicbsp_info_t aicbsp_info;
//extern struct mutex aicbsp_power_lock;
extern const struct aicbsp_firmware *aicbsp_firmware_list;
extern const struct aicbsp_firmware fw_u02[];
extern const struct aicbsp_firmware fw_u03[];
extern const struct aicbsp_firmware fw_8800dc_u01[];
extern const struct aicbsp_firmware fw_8800dc_u02[];

#endif
