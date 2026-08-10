/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _AIC_LOADFW_H_
#define _AIC_LOADFW_H_

#include "aic_plat_types.h"

#ifdef CONFIG_LOAD_FW_FROM_FILESYSTEM

#elif defined(CONFIG_LOAD_FW_FROM_FLASH)
uint8_t get_wifi_fw_type(void);
#else

#endif

int rwnx_load_firmware(uint16_t chipid, int mode, u32 **fw_buf);
void rwnx_restore_firmware(u32 **fw_buf);
int rwnx_load_calib(uint16_t chipid, u32 **fw_buf);
int rwnx_load_patch_tbl(uint16_t chipid, u32 **fw_buf);

#endif /* _AIC_LOADFW_H_ */

