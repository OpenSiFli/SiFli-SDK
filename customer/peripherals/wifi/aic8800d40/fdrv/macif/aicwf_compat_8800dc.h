/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AICWF_COMPAT_8800DC_H_
#define _AICWF_COMPAT_8800DC_H_

#include "rwnx_defs.h"

void aicwf_patch_wifi_setting_8800dc(struct rwnx_hw *rwnx_hw);
void aicwf_patch_config_8800dc(struct rwnx_hw *rwnx_hw);
int system_config_8800dc(struct rwnx_hw *rwnx_hw);

#endif /* _AICWF_COMPAT_8800DC_H_ */
