/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "fmacfw.h"
#include "fmacfw_rf.h"
#include "fmacfw_patch_8800dc_u02.h"
#include "fmacfw_patch_tbl_8800dc_u02.h"
#include "fmacfw_calib_8800dc_u02.h"
#include "fmacfw_patch_8800dc_h_u02.h"
#include "fmacfw_patch_tbl_8800dc_h_u02.h"
#include "fmacfw_calib_8800dc_h_u02.h"
#include "lmacfw_rf_8800dc.h"
#include "fmacfw_8800d80_u02.h"
#include "fmacfw_8800d80_h_u02.h"
#include "lmacfw_rf_8800d80_u02.h"
#include "fmacfw_8800m40.h"
#include "rtos_port.h"

#include "fw_adid_u03.h"
#include "fw_patch_u03.h"
#include "fw_patch_table_u03.h"

#include "fw_adid_8800dc_u02.h"
#include "fw_patch_8800dc_u02.h"
#include "fw_patch_table_8800dc_u02.h"
#include "fw_patch_8800dc_u02_ext0.h"

#include "fw_adid_8800dc_u02h.h"
#include "fw_patch_8800dc_u02h.h"
#include "fw_patch_table_8800dc_u02h.h"

#include "fw_adid_8800d80_u02.h"
#include "fw_patch_8800d80_u02.h"
#include "fw_patch_table_8800d80_u02.h"
#include "fw_patch_8800d80_u02_ext0.h"
#include "aic_fw.h"
#include "aic_plat_log.h"


void *aic_fw_ptr_get(enum aic_fw name)
{
    void *ptr = NULL;

    switch (name) {
#ifdef CONFIG_BT_SUPPORT
    case FW_ADID_U03:
        ptr = fw_adid_u03;
        break;
    case FW_PATCH_U03:
        ptr = fw_patch_u03;
        break;
    case FW_PATCH_TABLE_U03:
        ptr = fw_patch_table_u03;
        break;

    case FW_ADID_8800DC_U02:
        AIC_LOG_PRINTF("FW_ADID_8800DC_U02\n");
        ptr = fw_adid_8800dc_u02;
        break;
    case FW_PATCH_8800DC_U02:
        AIC_LOG_PRINTF("FW_PATCH_8800DC_U02\n");
        ptr = fw_patch_8800dc_u02;
        break;
	case FW_PATCH_TABLE_8800DC_U02:
        AIC_LOG_PRINTF("FW_PATCH_TABLE_8800DC_U02\n");
		ptr = fw_patch_table_8800dc_u02;
		break;
	case FW_PATCH_8800DC_U02_EXT:
		AIC_LOG_PRINTF("FW_PATCH_8800DC_U02_EXT\n");
		ptr = fw_patch_8800dc_u02_ext0;
		break;

    case FW_ADID_8800DC_U02H:
        AIC_LOG_PRINTF("FW_ADID_8800DC_U02\n");
        ptr = fw_adid_8800dc_u02h;
        break;
    case FW_PATCH_8800DC_U02H:
        AIC_LOG_PRINTF("FW_PATCH_8800DC_U02\n");
        ptr = fw_patch_8800dc_u02h;
        break;
	case FW_PATCH_TABLE_8800DC_U02H:
        AIC_LOG_PRINTF("FW_PATCH_TABLE_8800DC_U02\n");
		ptr = fw_patch_table_8800dc_u02h;
		break;

    case FW_ADID_8800D80_U02:
        AIC_LOG_PRINTF("FW_ADID_8800D80_U02\n");
        ptr = fw_adid_8800d80_u02;
        break;
    case FW_PATCH_8800D80_U02:
        AIC_LOG_PRINTF("FW_ADID_8800D80_U02\n");
        ptr = fw_patch_8800d80_u02;
        break;
	case FW_PATCH_TABLE_8800D80_U02:
        AIC_LOG_PRINTF("FW_ADID_8800D80_U02\n");
		ptr = fw_patch_table_8800d80_u02;
		break;
	case FW_PATCH_8800D80_U02_EXT:
		AIC_LOG_PRINTF("FW_ADID_8800D80_U02\n");
		ptr = fw_patch_8800d80_u02_ext0;
		break;
#endif
    default:
        AIC_LOG_PRINTF("PTR is NULL\n");
        ptr = NULL;
        break;
    }

    return ptr;
}

uint32_t aic_fw_size_get(enum aic_fw name)
{
    uint32_t size = 0;

    switch (name) {
#ifdef CONFIG_BT_SUPPORT
    case FW_ADID_U03:
        size = sizeof(fw_adid_u03);
        break;
    case FW_PATCH_U03:
        size = sizeof(fw_patch_u03);
        break;
    case FW_PATCH_TABLE_U03:
        size = sizeof(fw_patch_table_u03);
        break;
#if 0
    case FW_ADID_8800D80:
        size = sizeof(fw_adid_8800d80);
        break;
#endif
    case FW_ADID_8800D80_U02:
        size = sizeof(fw_adid_8800d80_u02);
        break;
    case FW_PATCH_8800D80_U02:
        size = sizeof(fw_patch_8800d80_u02);
        break;
	case FW_PATCH_TABLE_8800D80_U02:
		size = sizeof(fw_patch_table_8800d80_u02);
		break;
	case FW_PATCH_8800D80_U02_EXT:
		size = sizeof(fw_patch_8800d80_u02_ext0);
		break;
#endif
    default:
        size = 0;
        break;
    }

    return size;
}

#if defined(CONFIG_AIC8801_SUPPORT)
void *aic8800d_fw_ptr_get(void)
{
    return (void *)fmacfw;
}

uint32_t aic8800d_fw_size_get(void)
{
    return sizeof(fmacfw);
}

#if defined(CONFIG_WIFI_MODE_RFTEST)
void *aic8800d_rf_fw_ptr_get(void)
{
    return (void *)fmacfw_rf;
}

uint32_t aic8800d_rf_fw_size_get(void)
{
    return sizeof(fmacfw_rf);
}
#endif /* CONFIG_WIFI_MODE_RFTEST */
#endif

#if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
void *aic8800dc_u01_fw_ptr_get(void)
{
    return NULL;
}

uint32_t aic8800dc_u01_fw_size_get(void)
{
    return 0;
}

void *aic8800dc_u02_fw_ptr_get(void)
{
    return (void *)fmacfw_patch_8800dc_u02;
}

uint32_t aic8800dc_u02_fw_size_get(void)
{
    return sizeof(fmacfw_patch_8800dc_u02);
}

void *aic8800dc_u02_patch_tbl_ptr_get(void)
{
    return (void *)fmacfw_patch_tbl_8800dc_u02;
}

uint32_t aic8800dc_u02_patch_tbl_size_get(void)
{
    return sizeof(fmacfw_patch_tbl_8800dc_u02);
}

void *aic8800dc_u02_calib_fw_ptr_get(void)
{
    return (void *)fmacfw_calib_8800dc_u02;
}

uint32_t aic8800dc_u02_calib_fw_size_get(void)
{
    return sizeof(fmacfw_calib_8800dc_u02);
}

void *aic8800dc_h_u02_fw_ptr_get(void)
{
    return (void *)fmacfw_patch_8800dc_h_u02;
}

uint32_t aic8800dc_h_u02_fw_size_get(void)
{
    return sizeof(fmacfw_patch_8800dc_h_u02);
}

uint32_t aic8800dc_h_u02_patch_tbl_size_get(void)
{
    return sizeof(fmacfw_patch_tbl_8800dc_h_u02);
}

void *aic8800dc_h_u02_patch_tbl_ptr_get(void)
{
    return (void *)fmacfw_patch_tbl_8800dc_h_u02;
}

void *aic8800dc_h_u02_calib_fw_ptr_get(void)
{
    return (void *)fmacfw_calib_8800dc_h_u02;
}

uint32_t aic8800dc_h_u02_calib_fw_size_get(void)
{
    return sizeof(fmacfw_calib_8800dc_h_u02);
}
#if defined(CONFIG_WIFI_MODE_RFTEST)
void *aic8800dc_rf_lmacfw_ptr_get(void)
{
    return (void *)lmacfw_rf_8800dc;
}

uint32_t aic8800dc_rf_lmacfw_size_get(void)
{
    return sizeof(lmacfw_rf_8800dc);
}

void *aic8800dc_rf_fmacfw_ptr_get(void)
{
    return NULL;
}

uint32_t aic8800dc_rf_fmacfw_size_get(void)
{
    return 0;
}
#endif /* CONFIG_WIFI_MODE_RFTEST */
#endif

#if defined(CONFIG_AIC8800D80_SUPPORT)
void *aic8800d80_fw_ptr_get(void)
{
    return NULL;
}

uint32_t aic8800d80_fw_size_get(void)
{
    return 0;
}

void *aic8800d80_h_u02_fw_ptr_get(void)
{
    return (void *)fmacfw_8800d80_h_u02;
}

uint32_t aic8800d80_h_u02_fw_size_get(void)
{
    return sizeof(fmacfw_8800d80_h_u02);
}

void *aic8800d80_u02_fw_ptr_get(void)
{
    return (void *)fmacfw_8800d80_u02;
}

uint32_t aic8800d80_u02_fw_size_get(void)
{
    return sizeof(fmacfw_8800d80_u02);
}

void *aic8800d80_u02_rf_fw_ptr_get(void)
{
    return (void *)lmacfw_rf_8800d80_u02;
}

uint32_t aic8800d80_u02_rf_fw_size_get(void)
{
    return sizeof(lmacfw_rf_8800d80_u02);
}

void *aic8800m40_fw_ptr_get(void)
{
    return (void *)fmacfw_8800m40;
}

uint32_t aic8800m40_fw_size_get(void)
{
    return sizeof(fmacfw_8800m40);
}
#endif
