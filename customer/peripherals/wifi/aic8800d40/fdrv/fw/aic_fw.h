/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC_FW_H_
#define _AIC_FW_H_

#include "aic_plat_types.h"


enum aic_fw {
    FW_UNKNOWN = 0,
    FMACFW,
    FMACFW_RF,
    FW_ADID,
    FW_PATCH,
    FW_PATCH_TABLE,
    FW_ADID_U03,
    FW_PATCH_U03,
    FW_PATCH_TABLE_U03,
    FMACFW_8800DC,
    FMACFW_RF_8800DC,
    FMACFW_PATCH_8800DC,
    FMACFW_RF_PATCH_8800DC,
    LMACFW_RF_8800DC,
    FMACFW_PATCH_8800DC_U02,
    FMACFW_PATCH_TBL_8800DC_U02,
    FW_ADID_8800DC,
    FW_PATCH_8800DC,
    FW_PATCH_TABLE_8800DC,
    FW_ADID_8800DC_U02,
    FW_PATCH_8800DC_U02,
    FW_PATCH_TABLE_8800DC_U02,
    FW_ADID_8800D80,
    FW_ADID_8800D80_U02,
    FMACFW_8800D80,
    FMACFW_8800D80_U02,
    FMACFW_RF_8800D80,
    FMACFW_RF_8800D80_U02,
    FW_PATCH_8800D80,
    FW_PATCH_8800D80_U02,
    FW_PATCH_TABLE_8800D80,
    FW_PATCH_TABLE_8800D80_U02,
    FW_PATCH_8800DC_U02_EXT,
	FW_PATCH_8800D80_U02_EXT,
    FW_ADID_8800DC_U02H,
    FW_PATCH_8800DC_U02H,
    FW_PATCH_TABLE_8800DC_U02H,

};

void *aic8800d_fw_ptr_get(void);
uint32_t aic8800d_fw_size_get(void);
void *aic8800d_rf_fw_ptr_get(void);
uint32_t aic8800d_rf_fw_size_get(void);
void *aic8800dc_u01_fw_ptr_get(void);
uint32_t aic8800dc_u01_fw_size_get(void);
void *aic8800dc_u02_fw_ptr_get(void);
uint32_t aic8800dc_u02_fw_size_get(void);
void *aic8800dc_u02_calib_fw_ptr_get(void);
uint32_t aic8800dc_u02_calib_fw_size_get(void);
void *aic8800dc_h_u02_fw_ptr_get(void);
uint32_t aic8800dc_h_u02_fw_size_get(void);
void *aic8800dc_h_u02_calib_fw_ptr_get(void);
uint32_t aic8800dc_h_u02_calib_fw_size_get(void);
void *aic8800dc_rf_lmacfw_ptr_get(void);
uint32_t aic8800dc_rf_lmacfw_size_get(void);
void *aic8800dc_rf_fmacfw_ptr_get(void);
uint32_t aic8800dc_rf_fmacfw_size_get(void);
void *aic8800dc_u02_patch_tbl_ptr_get(void);
uint32_t aic8800dc_h_u02_patch_tbl_size_get(void);
void *aic8800dc_h_u02_patch_tbl_ptr_get(void);
uint32_t aic8800dc_u02_patch_tbl_size_get(void);
void *aic8800d80_fw_ptr_get(void);
uint32_t aic8800d80_fw_size_get(void);
void *aic8800d80_h_u02_fw_ptr_get(void);
uint32_t aic8800d80_h_u02_fw_size_get(void);
void *aic8800d80_u02_fw_ptr_get(void);
uint32_t aic8800d80_u02_fw_size_get(void);
void *aic8800d80_u02_rf_fw_ptr_get(void);
uint32_t aic8800d80_u02_rf_fw_size_get(void);
void *aic8800m40_fw_ptr_get(void);
uint32_t aic8800m40_fw_size_get(void);

#endif
