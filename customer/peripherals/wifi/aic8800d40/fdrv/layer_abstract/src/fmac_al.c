/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "aic_fw.h"
#include "rwnx_main.h"
#include "rwnx_platform.h"

uint8_t aic_get_chip_sub_id(void)
{
    return chip_sub_id;
}

uint8_t aic_get_chip_mcu_id(void)
{
    return chip_mcu_id;
}

uint8_t aic_get_aic8800dc_rf_flag(void)
{
    return aic8800dc_rf_flag;
}

uint8_t aic_get_chip_id(void)
{
    return chip_id;
}

int aic_is_chip_id_h(void)
{
    return IS_CHIP_ID_H() ? 1 : 0;
}

void aic_set_chip_fw_match(uint32_t value)
{
    chip_fw_match = value;
}

#if defined(CONFIG_AIC8801_SUPPORT)
void *fmac_aic8800d_fw_ptr_get(void)
{
    return aic8800d_fw_ptr_get();
}

uint32_t fmac_aic8800d_fw_size_get(void)
{
    return aic8800d_fw_size_get();
}

#if defined(CONFIG_WIFI_MODE_RFTEST)
void *fmac_aic8800d_rf_fw_ptr_get(void)
{
    return aic8800d_rf_fw_ptr_get();
}

uint32_t fmac_aic8800d_rf_fw_size_get(void)
{
    return aic8800d_rf_fw_size_get();
}
#endif /* CONFIG_WIFI_MODE_RFTEST */
#endif

#if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
void *fmac_aic8800dc_u01_fw_ptr_get(void)
{
    return aic8800dc_u01_fw_ptr_get();
}

uint32_t fmac_aic8800dc_u01_fw_size_get(void)
{
    return aic8800dc_u01_fw_size_get();
}

void *fmac_aic8800dc_u02_fw_ptr_get(void)
{
    return aic8800dc_u02_fw_ptr_get();
}

uint32_t fmac_aic8800dc_u02_fw_size_get(void)
{
    return aic8800dc_u02_fw_size_get();
}

void *fmac_aic8800dc_u02_patch_tbl_ptr_get(void)
{
    return aic8800dc_u02_patch_tbl_ptr_get();
}

uint32_t fmac_aic8800dc_u02_patch_tbl_size_get(void)
{
    return aic8800dc_u02_patch_tbl_size_get();
}

void *fmac_aic8800dc_u02_calib_fw_ptr_get(void)
{
    return aic8800dc_u02_calib_fw_ptr_get();
}

uint32_t fmac_aic8800dc_u02_calib_fw_size_get(void)
{
    return aic8800dc_u02_calib_fw_size_get();
}

void *fmac_aic8800dc_h_u02_fw_ptr_get(void)
{
    return aic8800dc_h_u02_fw_ptr_get();
}

uint32_t fmac_aic8800dc_h_u02_fw_size_get(void)
{
    return aic8800dc_h_u02_fw_size_get();
}

uint32_t fmac_aic8800dc_h_u02_patch_tbl_size_get(void)
{
    return aic8800dc_h_u02_patch_tbl_size_get();
}

void *fmac_aic8800dc_h_u02_patch_tbl_ptr_get(void)
{
    return aic8800dc_h_u02_patch_tbl_ptr_get();
}

void *fmac_aic8800dc_h_u02_calib_fw_ptr_get(void)
{
    return aic8800dc_h_u02_calib_fw_ptr_get();
}

uint32_t fmac_aic8800dc_h_u02_calib_fw_size_get(void)
{
    return aic8800dc_h_u02_calib_fw_size_get();
}
#if defined(CONFIG_WIFI_MODE_RFTEST)
void *fmac_aic8800dc_rf_lmacfw_ptr_get(void)
{
    return aic8800dc_rf_lmacfw_ptr_get();
}

uint32_t fmac_aic8800dc_rf_lmacfw_size_get(void)
{
    return aic8800dc_rf_lmacfw_size_get();
}

void *fmac_aic8800dc_rf_fmacfw_ptr_get(void)
{
    return aic8800dc_rf_fmacfw_ptr_get();
}

uint32_t fmac_aic8800dc_rf_fmacfw_size_get(void)
{
    return aic8800dc_rf_fmacfw_size_get();
}
#endif /* CONFIG_WIFI_MODE_RFTEST */
#endif

#if defined(CONFIG_AIC8800D80_SUPPORT)
void *fmac_aic8800d80_fw_ptr_get(void)
{
    return aic8800d80_fw_ptr_get();
}

uint32_t fmac_aic8800d80_fw_size_get(void)
{
    return aic8800d80_fw_size_get();
}

void *fmac_aic8800d80_h_u02_fw_ptr_get(void)
{
    return aic8800d80_h_u02_fw_ptr_get();
}

uint32_t fmac_aic8800d80_h_u02_fw_size_get(void)
{
    return aic8800d80_h_u02_fw_size_get();
}

void *fmac_aic8800d80_u02_fw_ptr_get(void)
{
    return aic8800d80_u02_fw_ptr_get();
}

uint32_t fmac_aic8800d80_u02_fw_size_get(void)
{
    return aic8800d80_u02_fw_size_get();
}

void *fmac_aic8800d80_u02_rf_fw_ptr_get(void)
{
    return aic8800d80_u02_rf_fw_ptr_get();
}

uint32_t fmac_aic8800d80_u02_rf_fw_size_get(void)
{
    return aic8800d80_u02_rf_fw_size_get();
}

void *fmac_aic8800m40_fw_ptr_get(void)
{
    return aic8800m40_fw_ptr_get();
}

uint32_t fmac_aic8800m40_fw_size_get(void)
{
    return aic8800m40_fw_size_get();
}
#endif