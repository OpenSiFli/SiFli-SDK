/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aic_bsp_driver.h"

const struct aicbsp_firmware *aicbsp_firmware_list = fw_u03;

const struct aicbsp_firmware fw_u02[] = {
	[AICBSP_CPMODE_WORK] = {
		.desc          = "normal work mode(sdio u02)",
		.bt_adid       = FW_ADID,
		.bt_patch      = FW_PATCH,
		.bt_table      = FW_PATCH_TABLE,
		.wl_fw         = FMACFW,
	},
	[AICBSP_CPMODE_TEST] = {
		.desc          = "rf test mode(sdio u02)",
		.bt_adid       = FW_ADID,
		.bt_patch      = FW_PATCH,
		.bt_table      = FW_PATCH_TABLE,
		.wl_fw         = FMACFW_RF,
	},
};

const struct aicbsp_firmware fw_u03[] = {
	[AICBSP_CPMODE_WORK] = {
		.desc          = "normal work mode(sdio u03/u04)",
		.bt_adid       = FW_ADID_U03,
		.bt_patch      = FW_PATCH_U03,
		.bt_table      = FW_PATCH_TABLE_U03,
		#ifdef CONFIG_MCU_MESSAGE
		.wl_fw         = FMACFW_8800M_CUSTMSG,
		#else
		.wl_fw         = FMACFW,
		#endif
	},

	[AICBSP_CPMODE_TEST] = {
		.desc          = "rf test mode(sdio u03/u04)",
		.bt_adid       = FW_ADID_U03,
		.bt_patch      = FW_PATCH_U03,
		.bt_table      = FW_PATCH_TABLE_U03,
		.wl_fw         = FMACFW_RF,
	},
};

struct aicbsp_info_t aicbsp_info = {
	.hwinfo_r = AICBSP_HWINFO_DEFAULT,
	.hwinfo   = AICBSP_HWINFO_DEFAULT,
	.cpmode   = AICBSP_CPMODE_DEFAULT,
	.fwlog_en = AICBSP_FWLOG_EN_DEFAULT,
};
