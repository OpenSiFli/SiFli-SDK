/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * @file rwnx_platorm.h
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */

#ifndef _RWNX_PLATFORM_H_
#define _RWNX_PLATFORM_H_

#include "lmac_msg.h"

struct rwnx_hw;

/**
 * struct rwnx_plat - Operation pointers for RWNX PCI platform
 *
 * @pci_dev: pointer to pci dev
 * @enabled: Set if embedded platform has been enabled (i.e. fw loaded and
 *          ipc started)
 * @enable: Configure communication with the fw (i.e. configure the transfers
 *         enable and register interrupt)
 * @disable: Stop communication with the fw
 * @deinit: Free all ressources allocated for the embedded platform
 * @get_address: Return the virtual address to access the requested address on
 *              the platform.
 * @ack_irq: Acknowledge the irq at link level.
 * @get_config_reg: Return the list (size + pointer) of registers to restore in
 * order to reload the platform while keeping the current configuration.
 *
 * @priv Private data for the link driver
 */
struct rwnx_plat {
    #if 0
    struct pci_dev *pci_dev;

#ifdef AICWF_SDIO_SUPPORT
	struct aic_sdio_dev *sdiodev;
#endif

#ifdef AICWF_USB_SUPPORT
    struct aic_usb_dev *usbdev;
#endif
    #endif
    bool enabled;
};

extern u8 aic8800dc_rf_flag;
extern u8 aic8800dc_calib_flag;
extern struct rwnx_plat *g_rwnx_plat;

int rwnx_platform_init(struct rwnx_plat *rwnx_plat, void **platform_data);
void rwnx_platform_deinit(struct rwnx_hw *rwnx_hw);

int rwnx_platform_on(struct rwnx_hw *rwnx_hw);
void rwnx_platform_off(struct rwnx_hw *rwnx_hw);

int rwnx_send_reboot(struct rwnx_hw *rwnx_hw);

int aicwf_misc_ram_init_8800dc(struct rwnx_hw *rwnx_hw);
int aicwf_patch_table_load(struct rwnx_hw *rwnx_hw);

#ifdef CONFIG_DPD
#define ROM_FMAC_CALIB_ADDR            0x00130000
#define FW_PATH_MAX_LEN 200

typedef struct {
    uint32_t bit_mask[3];
    uint32_t reserved;
    uint32_t dpd_high[96];
    uint32_t dpd_11b[96];
    uint32_t dpd_low[96];
    uint32_t idac_11b[48];
    uint32_t idac_high[48];
    uint32_t idac_low[48];
    uint32_t loft_res[18];
    uint32_t rx_iqim_res[16];
} rf_misc_ram_t;

typedef struct {
    uint32_t bit_mask[4];
    uint32_t dpd_high[96];
    uint32_t loft_res[18];
} rf_misc_ram_lite_t;

#define MEMBER_SIZE(type, member)   sizeof(((type *)0)->member)
#define DPD_RESULT_SIZE_8800DC      sizeof(rf_misc_ram_lite_t)

extern rf_misc_ram_lite_t dpd_res;
int aicwf_plat_calib_load_8800dc(struct rwnx_hw *rwnx_hw);
int aicwf_dpd_calib_8800dc(struct rwnx_hw *rwnx_hw,  rf_misc_ram_lite_t *dpd_res);
int aicwf_dpd_result_load_8800dc(struct rwnx_hw *rwnx_hw, rf_misc_ram_lite_t *dpd_res);
int aicwf_dpd_result_apply_8800dc(struct rwnx_hw *rwnx_hw, rf_misc_ram_lite_t *dpd_res);
uint32_t aicwf_dpd_get_result(void);
#endif /* CONFIG_DPD */


#ifdef CONFIG_LOAD_USERCONFIG
void aicwf_userconfig_init(void);
void get_nvram_txpwr_idx(txpwr_idx_conf_t *txpwr_idx);
void get_nvram_txpwr_ofst(txpwr_ofst_conf_t *txpwr_ofst);
void get_nvram_xtal_cap(xtal_cap_conf_t *xtal_cap);
void get_userconfig_txpwr_lvl(txpwr_lvl_conf_t *txpwr_lvl);
void get_userconfig_txpwr_lvl_v2(txpwr_lvl_conf_v2_t *txpwr_lvl_v2);
void get_userconfig_txpwr_lvl_v3(txpwr_lvl_conf_v3_t *txpwr_lvl_v3);
void get_userconfig_txpwr_ofst(txpwr_ofst_conf_t *txpwr_ofst);
void get_userconfig_txpwr_ofst2x(txpwr_ofst2x_conf_t *txpwr_ofst2x);
void get_userconfig_xtal_cap(xtal_cap_conf_t *xtal_cap);
#endif /* CONFIG_LOAD_USERCONFIG */

void rwnx_get_chip_info(uint8* info);
uint8_t *rwnx_get_fw_ver(void);


#endif /* _RWNX_PLATFORM_H_ */