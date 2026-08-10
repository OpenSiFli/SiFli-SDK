/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _FMAC_H_
#define _FMAC_H_

#include "aic_plat_types.h"

uint8_t aic_get_chip_sub_id(void);
uint8_t aic_get_chip_mcu_id(void);
uint8_t aic_get_aic8800dc_rf_flag(void);
uint8_t aic_get_chip_id(void);
int aic_is_chip_id_h(void);
void aic_set_chip_fw_match(uint32_t value);

void *fmac_aic8800d_fw_ptr_get(void);
uint32_t fmac_aic8800d_fw_size_get(void);
void *fmac_aic8800d_rf_fw_ptr_get(void);
uint32_t fmac_aic8800d_rf_fw_size_get(void);
void *fmac_aic8800dc_u01_fw_ptr_get(void);
uint32_t fmac_aic8800dc_u01_fw_size_get(void);
void *fmac_aic8800dc_u02_fw_ptr_get(void);
uint32_t fmac_aic8800dc_u02_fw_size_get(void);
void *fmac_aic8800dc_u02_calib_fw_ptr_get(void);
uint32_t fmac_aic8800dc_u02_calib_fw_size_get(void);
void *fmac_aic8800dc_h_u02_fw_ptr_get(void);
uint32_t fmac_aic8800dc_h_u02_fw_size_get(void);
void *fmac_aic8800dc_h_u02_calib_fw_ptr_get(void);
uint32_t fmac_aic8800dc_h_u02_calib_fw_size_get(void);
void *fmac_aic8800dc_rf_lmacfw_ptr_get(void);
uint32_t fmac_aic8800dc_rf_lmacfw_size_get(void);
void *fmac_aic8800dc_rf_fmacfw_ptr_get(void);
uint32_t fmac_aic8800dc_rf_fmacfw_size_get(void);
void *fmac_aic8800dc_u02_patch_tbl_ptr_get(void);
uint32_t fmac_aic8800dc_h_u02_patch_tbl_size_get(void);
void *fmac_aic8800dc_h_u02_patch_tbl_ptr_get(void);
uint32_t fmac_aic8800dc_u02_patch_tbl_size_get(void);
void *fmac_aic8800d80_fw_ptr_get(void);
uint32_t fmac_aic8800d80_fw_size_get(void);
void *fmac_aic8800d80_h_u02_fw_ptr_get(void);
uint32_t fmac_aic8800d80_h_u02_fw_size_get(void);
void *fmac_aic8800d80_u02_fw_ptr_get(void);
uint32_t fmac_aic8800d80_u02_fw_size_get(void);
void *fmac_aic8800d80_u02_rf_fw_ptr_get(void);
uint32_t fmac_aic8800d80_u02_rf_fw_size_get(void);
void *fmac_aic8800m40_fw_ptr_get(void);
uint32_t fmac_aic8800m40_fw_size_get(void);

#endif /*  */