/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC_LOADFW_H_
#define _AIC_LOADFW_H_

#include "aic_porting_types.h"

int aicwf_rwnx_load_firmware(uint16_t chipid, int mode, uint32_t **fw_buf);
void aicwf_rwnx_restore_firmware(uint32_t **fw_buf);
int aicwf_rwnx_load_calib(uint16_t chipid, uint32_t **fw_buf);
int aicwf_rwnx_load_patch_tbl(uint16_t chipid, uint32_t **fw_buf);
uint32_t aicwf_get_wifi_fw_type(void);
int aicwf_rftest_fw_available(void);

#endif /* _AIC_LOADFW_H_ */

