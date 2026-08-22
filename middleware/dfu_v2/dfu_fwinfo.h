/**
 * @file dfu_fwinfo.h
 * @brief DFU V2 — Firmware Info API
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_FWINFO_H__
#define __DFU_FWINFO_H__

#include "dfu_macro.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * API
 *============================================================================*/

/**
 * @brief Write firmware info for one image to flash
 * @param index  Image index (0 ~ DFU_MAX_FW_FILES-1)
 * @param info   Firmware info to write
 * @return 0 on success, -1 on failure
 */
int dfu_fwinfo_set(int index, const struct dfu_fw_info *info);

/**
 * @brief Read firmware info for one image from flash
 * @param index  Image index
 * @param info   Output buffer
 * @return 0 on success, -1 on failure
 */
int dfu_fwinfo_get(int index, struct dfu_fw_info *info);

/**
 * @brief Mark all valid entries as needs_update=1 with magic
 * @return 0 on success, -1 on failure
 */
int dfu_fwinfo_set_update_flags(void);

/** @brief Clear all firmware info entries */
void dfu_fwinfo_clear(void);

/** Backward compatibility wrappers — called by dfu_engine.c */
int dfu_core_set_firmware_info(int index, const struct dfu_fw_info *fw_info);
int dfu_core_set_update_flags(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_FWINFO_H__ */