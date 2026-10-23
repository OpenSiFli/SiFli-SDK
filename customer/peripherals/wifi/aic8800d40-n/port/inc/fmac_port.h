/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shim header replacing platform/fmac/fmac_port.h
 * Firmware loading function declarations.
 */
#ifndef _AIC_LOADFW_H_
#define _AIC_LOADFW_H_

#include "aic_porting_types.h"

int aicwf_rwnx_load_firmware(uint16_t chipid, int mode, uint32_t **fw_buf);
void aicwf_rwnx_restore_firmware(uint32_t **fw_buf);
int aicwf_rwnx_load_calib(uint16_t chipid, uint32_t **fw_buf);
int aicwf_rwnx_load_patch_tbl(uint16_t chipid, uint32_t **fw_buf);
uint32_t aicwf_get_wifi_fw_type(void);
int aicwf_rftest_fw_available(void);   /* [AIC8800-PORT][SiFli] restored: the shim missed this declaration from platform/fmac/fmac_port.h:17 (an implicit declaration is an error with GCC14) */

#endif /* _AIC_LOADFW_H_ */
