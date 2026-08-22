/**
 * @file dfu_app.h
 * @brief DFU V2 — User Application API
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_APP_H__
#define __DFU_APP_H__

#include "dfu_macro.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enter DFU mode by writing trigger flags and rebooting
 *
 * Writes a dfu_fw_info entry at the tail of DFU_V2_LOADER partition
 * with needs_update=1 and correct magic. Then reboots.
 * Bootloader will detect the flag and jump to DFU subprogram.
 *
 * Does not return.
 */
void dfu_enter_dfu_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_APP_H__ */
