/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _RWNX_PLATFORM_H_
#define _RWNX_PLATFORM_H_

#include "lmac_msg.h"
#include "aicwf_sdio.h"

enum AICWF_IC
{
    PRODUCT_ID_AIC8800 = 0,
    PRODUCT_ID_AIC8801,
    PRODUCT_ID_AIC8800MC,
    PRODUCT_ID_AIC8800DC,
    PRODUCT_ID_AIC8800DW,
    PRODUCT_ID_AIC8800M40,
    PRODUCT_ID_AIC8800D80,
};

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
struct rwnx_plat
{
#if 0
    struct pci_dev *pci_dev;

#ifdef AICWF_SDIO_SUPPORT
    struct aic_sdio_dev *sdiodev;
#endif

#ifdef AICWF_USB_SUPPORT
    struct aic_usb_dev *usbdev;
#endif
#endif
    struct aic_sdio_dev *sdiodev;
    bool enabled;

#if 0
    int (*enable)(struct rwnx_hw *rwnx_hw);
    int (*disable)(struct rwnx_hw *rwnx_hw);
    void (*deinit)(struct rwnx_plat *rwnx_plat);
    u8 *(*get_address)(struct rwnx_plat *rwnx_plat, int addr_name,
                       unsigned int offset);
    void (*ack_irq)(struct rwnx_plat *rwnx_plat);
    int (*get_config_reg)(struct rwnx_plat *rwnx_plat, const u32 **list);

    u8 priv[0] __aligned(sizeof(void *));
#endif
};

extern struct rwnx_plat *g_rwnx_plat;

int rwnx_platform_init(struct rwnx_plat *rwnx_plat, void **platform_data);
void rwnx_platform_deinit(struct rwnx_hw *rwnx_hw);

int rwnx_platform_on(struct rwnx_hw *rwnx_hw);
void rwnx_platform_off(struct rwnx_hw *rwnx_hw);

int aicwf_misc_ram_init_8800dc(struct rwnx_hw *rwnx_hw);
int aicwf_patch_table_load(struct rwnx_hw *rwnx_hw);

#if 0 //(defined(CONFIG_DPD) && !defined(CONFIG_FORCE_DPD_CALIB))
    #define FW_DPDRESULT_NAME_8800DC        "aic_dpdresult_8800dc.bin"
#endif

#ifdef CONFIG_DPD
#define ROM_FMAC_CALIB_ADDR            0x00130000
#define FW_PATH_MAX_LEN 200

typedef struct
{
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

typedef struct
{
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
#if 0 //ifndef CONFIG_FORCE_DPD_CALIB
    //int aicwf_dpd_result_load_8800dc(struct rwnx_hw *rwnx_hw, rf_misc_ram_lite_t *dpd_res);
    int aicwf_dpd_result_write_8800dc(void *buf, int buf_len);
#endif
#endif


#ifdef CONFIG_LOAD_USERCONFIG
    void get_nvram_txpwr_idx(txpwr_idx_conf_t *txpwr_idx);
    void get_nvram_txpwr_ofst(txpwr_ofst_conf_t *txpwr_ofst);
    void get_nvram_xtal_cap(xtal_cap_conf_t *xtal_cap);


    void get_userconfig_txpwr_lvl(txpwr_lvl_conf_t *txpwr_lvl);
    void get_userconfig_txpwr_lvl_v2(txpwr_lvl_conf_v2_t *txpwr_lvl_v2);
    void get_userconfig_txpwr_lvl_v3(txpwr_lvl_conf_v3_t *txpwr_lvl_v3);
    void get_userconfig_txpwr_ofst(txpwr_ofst_conf_t *txpwr_ofst);
    void get_userconfig_txpwr_ofst2x(txpwr_ofst2x_conf_t *txpwr_ofst2x);
    void get_userconfig_xtal_cap(xtal_cap_conf_t *xtal_cap);
#endif

#endif /* _RWNX_PLATFORM_H_ */
