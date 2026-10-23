/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform firmware data provider & loader - merged from aic_fw.c + fmac_port.c.
 * Provides firmware binary data to host driver through aic_fw_port interface,
 * and implements firmware loading strategies (filemode / flashmode / innermode).
 */

#include <stdint.h>
#include "rtos_port.h"
#include "aic_cfg.h"
#include "aic_log.h"
#include "sys_api.h"
#include "wifi_api.h"
#include "aic_fw_port.h"
#include "wifi_def.h"

#if CONFIG_LOAD_FW == 2   /* innermode: firmware embedded as C array, firmware array headers needed */
    /* WiFi firmware binary headers */
    #if defined(CONFIG_AIC8801)
        #include "fw/fmacfw.h"
        #if defined(CONFIG_WIFI_MODE_RFTEST)
            #include "fw/fmacfw_rf.h"
        #endif
    #endif

    #if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
        #if !defined(CONFIG_AIC8800DC_H) && !defined(CONFIG_AIC8800DW_H)
            /* DC/DW/DL base (sub_id 0/1) firmware */
            #include "fw/fmacfw_patch_8800dc_u02.h"
            #include "fw/fmacfw_patch_tbl_8800dc_u02.h"
            #include "fw/fmacfw_calib_8800dc_u02.h"
        #else
            #if defined(AIC_HCI_OVER_SDIO)
                /* DC_H mailbox HCI needs the hbt fmac (plain h_u02 has no TDLS_SDIO_BT). */
                #include "fw/fmacfw_patch_8800dc_hbt_u02.h"
                #include "fw/fmacfw_patch_tbl_8800dc_hbt_u02.h"
                #include "fw/fmacfw_calib_8800dc_hbt_u02.h"
                #define fmacfw_patch_8800dc_h_u02      fmacfw_patch_8800dc_hbt_u02
                #define fmacfw_patch_tbl_8800dc_h_u02  fmacfw_patch_tbl_8800dc_hbt_u02
                #define fmacfw_calib_8800dc_h_u02      fmacfw_calib_8800dc_hbt_u02
            #else
                /* DC_H/DW_H/DL_H (sub_id 2) firmware — saves Flash, excludes non-H firmware */
                #include "fw/fmacfw_patch_8800dc_h_u02.h"
                #include "fw/fmacfw_patch_tbl_8800dc_h_u02.h"
                #include "fw/fmacfw_calib_8800dc_h_u02.h"
            #endif
        #endif
        #if defined(CONFIG_WIFI_MODE_RFTEST)
            #include "fw/lmacfw_rf_8800dc.h"
        #endif
    #endif

    #if defined(CONFIG_AIC8800D80)
        #if !defined(CONFIG_AIC8800D80_H)
            /* D80 base (U01/U02/U03) firmware */
            #if defined(AIC_HCI_OVER_SDIO)
                /* SDIO mailbox HCI needs the BT-capable fmac: plain d80_u02 lacks the
                * TDLS_SDIO_BT message, so HCI triggers a FW ke_task_handler_get assertion
                * (idx < TASK_DESC[type].idx_max) (matches Linux aic_bsp_main.c CONFIG_SDIO_BT) */
                #include "fw/fmacfwbt_8800d80_u02.h"
                #define fmacfw_8800d80_u02      fmacfwbt_8800d80_u02
            #else
                #include "fw/fmacfw_8800d80_u02.h"
            #endif
            #if defined(CONFIG_WIFI_MODE_RFTEST)
                #include "fw/lmacfw_rf_8800d80_u02.h"
            #endif
        #else
            /* D80_H (H version) firmware — selected by bus + IPC scenario (aligned with Linux) */
            #if defined(CONFIG_USB_SUPPORT)
                /* USB H firmware (binary differs from SDIO) */
                #if defined(CONFIG_FOR_IPCAM)
                    #include "fw/fmacfw_8800d80_h_u02_usb_ipc.h"
                #else
                    #include "fw/fmacfw_8800d80_h_u02_usb.h"
                #endif
            #else
                /* SDIO H firmware */
                #if defined(CONFIG_FOR_IPCAM)
                    #include "fw/fmacfw_8800d80_h_u02_ipc.h"
                #else
                    #include "fw/fmacfw_8800d80_h_u02.h"
                #endif
            #endif
            #if defined(CONFIG_WIFI_MODE_RFTEST)
                #include "fw/lmacfw_rf_8800d80_u02.h"
            #endif
        #endif
    #endif

    #if defined(CONFIG_AIC8800D80N)
        /* D80N WiFi firmware: patch_load multi-stage loading (see Linux aic8800d80n_compat.c) */
        /* U02 (chip_sub_id==1) — version used by the current chip */
        #include "fw/fmacfw_patch_8800d80n_u02.h"
        #include "fw/fmacfw_patch_tbl_8800d80n_u02.h"
        #include "fw/fmacfw_cinit_8800d80n_u02.h"
        #include "fw/fmacfw_calib_8800d80n_u02.h"
        #if defined(CONFIG_AIC8800D80N_U02C)
            /* U02C (chip_sub_id!=1) — compiled only when CONFIG_AIC8800D80N_U02C is defined */
            #include "fw/fmacfw_patch_8800d80n_u02c.h"
            #include "fw/fmacfw_patch_tbl_8800d80n_u02c.h"
            #include "fw/fmacfw_cinit_8800d80n_u02c.h"
            #include "fw/fmacfw_calib_8800d80n_u02c.h"
        #endif /* CONFIG_AIC8800D80N_U02C */
        #if defined(CONFIG_WIFI_MODE_RFTEST)
            #include "fw/lmacfw_rf_8800d80n.h"
            #include "fw/lmacfw_rf_8800d80n_i.h"
        #endif
    #endif

    #if defined(CONFIG_AIC8800D80X2)
        /* D80X2 (U05 baseline) WiFi firmware */
        #include "fw/fmacfw_8800d80x2.h"
        #if defined(CONFIG_WIFI_MODE_RFTEST)
            #include "fw/lmacfw_rf_8800d80x2.h"
        #endif
    #endif

    /* BT firmware binary headers */
    #ifdef CONFIG_BT_SUPPORT
        #if defined(CONFIG_AIC8801)
            #include "fw/fw_patch_u03.h"
            #include "fw/fw_patch_table_u03.h"
            #include "fw/fw_adid_u03.h"
        #endif
        #if defined(CONFIG_AIC8800D80)
            #include "fw/fw_adid_8800d80_u02.h"
            #include "fw/fw_patch_8800d80_u02.h"
            #include "fw/fw_patch_table_8800d80_u02.h"
            #include "fw/fw_patch_8800d80_u02_ext0.h"
            #include "fw/fw_patch_8800d80_u02_ext1.h"   /* ext patch of BT patch table id=1 */
        #endif
        #if defined(CONFIG_AIC8800D80N)
            #include "fw/fw_adid_8800d80n_u02.h"
            #include "fw/fw_patch_8800d80n_u02.h"
            #include "fw/fw_patch_table_8800d80n_u02.h"
            #include "fw/fw_patch_8800d80n_u02_ext0.h"
            #if defined(CONFIG_AIC8800D80N_U02C)
                #include "fw/fw_adid_8800d80n_u02c.h"
                #include "fw/fw_patch_8800d80n_u02c.h"
                #include "fw/fw_patch_table_8800d80n_u02c.h"
                #include "fw/fw_patch_8800d80n_u02c_ext0.h"
            #endif /* CONFIG_AIC8800D80N_U02C */
            #if defined(CONFIG_WIFI_MODE_RFTEST)
                /* D80N RFTEST BT firmware (the _rf_ variant used by Linux CPMODE_TEST) */
                #include "fw/fw_patch_8800d80n_rf_u02.h"
                #include "fw/fw_patch_8800d80n_rf_u02_ext0.h"
                #include "fw/fw_patch_table_8800d80n_rf_u02.h"
                #if defined(CONFIG_AIC8800D80N_U02C)
                    #include "fw/fw_patch_8800d80n_rf_u02c.h"
                    #include "fw/fw_patch_8800d80n_rf_u02c_ext0.h"
                    #include "fw/fw_patch_table_8800d80n_rf_u02c.h"
                #endif /* CONFIG_AIC8800D80N_U02C */
            #endif /* CONFIG_WIFI_MODE_RFTEST */
        #endif
        #if defined(CONFIG_AIC8800D80X2)
            #include "fw/fw_adid_8800d80x2_u05.h"
            #include "fw/fw_patch_8800d80x2_u05.h"
            #include "fw/fw_patch_table_8800d80x2_u05.h"
            #include "fw/fw_patch_8800d80x2_u05_ext0.h"
            #include "fw/fw_patch_8800d80x2_u05_ext1.h"
        #endif
        #if defined(CONFIG_AIC8800DC_H)
            #include "fw/fw_adid_8800dc_u02h.h"
            #include "fw/fw_patch_8800dc_u02h.h"
            #include "fw/fw_patch_table_8800dc_u02h.h"
        #endif
    #endif /* CONFIG_BT_SUPPORT */
#endif /* CONFIG_LOAD_FW == 2 */

/* ==========================================================================
 * Firmware data provider (from former aic_fw.c)
 * ========================================================================== */

/**
 * @brief Get the firmware data pointer
 *
 * Return the firmware array pointer embedded at build time for the given ID.
 * Covers WiFi firmware (per chip model) and BT firmware.
 *
 * @param id firmware type ID
 * @return firmware data pointer, NULL=not found
 */
#if CONFIG_LOAD_FW == 2
void *aic_fw_ptr_get(enum aic_fw id)
{
    switch (id)
    {
#if defined(CONFIG_AIC8801)
    case FMACFW:
        return (void *)fmacfw;
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF:
        return (void *)fmacfw_rf;
#endif
#endif

#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
    case FMACFW_PATCH_8800DC:
        return NULL;
#if !defined(CONFIG_AIC8800DC_H) && !defined(CONFIG_AIC8800DW_H)
    case FMACFW_PATCH_8800DC_U02:
        return (void *)fmacfw_patch_8800dc_u02;
    case FMACFW_PATCH_TBL_8800DC_U02:
        return (void *)fmacfw_patch_tbl_8800dc_u02;
    case FMACFW_CALIB_8800DC_U02:
        return (void *)fmacfw_calib_8800dc_u02;
#else
    case FMACFW_PATCH_8800DC_H_U02:
        return (void *)fmacfw_patch_8800dc_h_u02;
    case FMACFW_PATCH_TBL_8800DC_H_U02:
        return (void *)fmacfw_patch_tbl_8800dc_h_u02;
    case FMACFW_CALIB_8800DC_H_U02:
        return (void *)fmacfw_calib_8800dc_h_u02;
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case LMACFW_RF_8800DC:
        return (void *)lmacfw_rf_8800dc;
    case FMACFW_RF_PATCH_8800DC:
        return NULL;
#endif
#endif

#if defined(CONFIG_AIC8800D80)
#if !defined(CONFIG_AIC8800D80_H)
    case FMACFW_8800D80:
        return NULL;
    case FMACFW_8800D80_U02:
        return (void *)fmacfw_8800d80_u02;
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF_8800D80_U02:
        return (void *)lmacfw_rf_8800d80_u02;
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF_8800D80:
        return NULL;
#endif
#else
#if defined(CONFIG_USB_SUPPORT)
    case FMACFW_8800D80_H_U02_USB:
        return (void *)fmacfw_8800d80_h_u02_usb;
#if defined(CONFIG_FOR_IPCAM)
    case FMACFW_8800D80_H_U02_USB_IPC:
        return (void *)fmacfw_8800d80_h_u02_usb_ipc;
#endif
#else
    case FMACFW_8800D80_H_U02:
        return (void *)fmacfw_8800d80_h_u02;
#if defined(CONFIG_FOR_IPCAM)
    case FMACFW_8800D80_H_U02_IPC:
        return (void *)fmacfw_8800d80_h_u02_ipc;
#endif
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF_8800D80_U02:
        return (void *)lmacfw_rf_8800d80_u02;
#endif
#endif
#endif

#if defined(CONFIG_AIC8800D80N)
    /* U02 (current chip version) */
    case FMACFW_PATCH_8800D80N_U02:
        return (void *)fmacfw_patch_8800d80n_u02;
    case FMACFW_PATCH_TBL_8800D80N_U02:
        return (void *)fmacfw_patch_tbl_8800d80n_u02;
    case FMACFW_CINIT_8800D80N_U02:
        return (void *)fmacfw_cinit_8800d80n_u02;
    case FMACFW_CALIB_8800D80N_U02:
        return (void *)fmacfw_calib_8800d80n_u02;
        /* U02C — compiled only when CONFIG_AIC8800D80N_U02C is defined */
#if defined(CONFIG_AIC8800D80N_U02C)
    case FMACFW_PATCH_8800D80N_U02C:
        return (void *)fmacfw_patch_8800d80n_u02c;
    case FMACFW_PATCH_TBL_8800D80N_U02C:
        return (void *)fmacfw_patch_tbl_8800d80n_u02c;
    case FMACFW_CINIT_8800D80N_U02C:
        return (void *)fmacfw_cinit_8800d80n_u02c;
    case FMACFW_CALIB_8800D80N_U02C:
        return (void *)fmacfw_calib_8800d80n_u02c;
#endif /* CONFIG_AIC8800D80N_U02C */
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case LMACFW_RF_8800D80N:
        return (void *)lmacfw_rf_8800d80n;
    /* 8819i (upper 2 bits of chip_id = 0xC0) RF test firmware */
    case LMACFW_RF_8800D80N_I:
        return (void *)lmacfw_rf_8800d80n_i;
#endif
#endif

#if defined(CONFIG_AIC8800D80X2)
    case FMACFW_8800D80X2:
        return (void *)fmacfw_8800d80x2;
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case LMACFW_RF_8800D80X2:
        return (void *)lmacfw_rf_8800d80x2;
#endif
#endif

#ifdef CONFIG_BT_SUPPORT
#if defined(CONFIG_AIC8801)
    case FW_ADID_U03:
        return (void *)fw_adid_u03;
    case FW_PATCH_U03:
        return (void *)fw_patch_u03;
    case FW_PATCH_TABLE_U03:
        return (void *)fw_patch_table_u03;
#endif
#if defined(CONFIG_AIC8800D80)
    case FW_ADID_8800D80_U02:
        return (void *)fw_adid_8800d80_u02;
    case FW_PATCH_8800D80_U02:
        return (void *)fw_patch_8800d80_u02;
    case FW_PATCH_TABLE_8800D80_U02:
        return (void *)fw_patch_table_8800d80_u02;
    case FW_PATCH_8800D80_U02_EXT:
        return (void *)fw_patch_8800d80_u02_ext0;
    case FW_PATCH_8800D80_U02_EXT1:
        return (void *)fw_patch_8800d80_u02_ext1;
#endif
#if defined(CONFIG_AIC8800D80N)
    case FW_ADID_8800D80N_U02:
        return (void *)fw_adid_8800d80n_u02;
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_ADID_8800D80N_U02C:
        return (void *)fw_adid_8800d80n_u02c;
#endif /* CONFIG_AIC8800D80N_U02C */
    case FW_PATCH_8800D80N_U02:
        return (void *)fw_patch_8800d80n_u02;
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_8800D80N_U02C:
        return (void *)fw_patch_8800d80n_u02c;
#endif /* CONFIG_AIC8800D80N_U02C */
    case FW_PATCH_TABLE_8800D80N_U02:
        return (void *)fw_patch_table_8800d80n_u02;
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_TABLE_8800D80N_U02C:
        return (void *)fw_patch_table_8800d80n_u02c;
#endif /* CONFIG_AIC8800D80N_U02C */
    case FW_PATCH_8800D80N_U02_EXT:
        return (void *)fw_patch_8800d80n_u02_ext0;
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_8800D80N_U02C_EXT:
        return (void *)fw_patch_8800d80n_u02c_ext0;
#endif /* CONFIG_AIC8800D80N_U02C */
#if defined(CONFIG_WIFI_MODE_RFTEST)
    /* D80N RFTEST BT firmware (the _rf_ variant used by Linux CPMODE_TEST) */
    case FW_PATCH_8800D80N_RF_U02:
        return (void *)fw_patch_8800d80n_rf_u02;
    case FW_PATCH_8800D80N_RF_U02_EXT:
        return (void *)fw_patch_8800d80n_rf_u02_ext0;
    case FW_PATCH_TABLE_8800D80N_RF_U02:
        return (void *)fw_patch_table_8800d80n_rf_u02;
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST) && defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_8800D80N_RF_U02C:
        return (void *)fw_patch_8800d80n_rf_u02c;
    case FW_PATCH_8800D80N_RF_U02C_EXT:
        return (void *)fw_patch_8800d80n_rf_u02c_ext0;
    case FW_PATCH_TABLE_8800D80N_RF_U02C:
        return (void *)fw_patch_table_8800d80n_rf_u02c;
#endif
#endif
#if defined(CONFIG_AIC8800D80X2)
    case FW_ADID_8800D80X2_U05:
        return (void *)fw_adid_8800d80x2_u05;
    case FW_PATCH_8800D80X2_U05:
        return (void *)fw_patch_8800d80x2_u05;
    case FW_PATCH_TABLE_8800D80X2_U05:
        return (void *)fw_patch_table_8800d80x2_u05;
    case FW_PATCH_8800D80X2_U05_EXT0:
        return (void *)fw_patch_8800d80x2_u05_ext0;
    case FW_PATCH_8800D80X2_U05_EXT1:
        return (void *)fw_patch_8800d80x2_u05_ext1;
#endif
#if defined(CONFIG_AIC8800DC_H)
    case FW_ADID_8800DC_U02H:
        return (void *)fw_adid_8800dc_u02h;
    case FW_PATCH_8800DC_U02H:
        return (void *)fw_patch_8800dc_u02h;
    case FW_PATCH_TABLE_8800DC_U02H:
        return (void *)fw_patch_table_8800dc_u02h;
#endif
#endif /* CONFIG_BT_SUPPORT */

    default:
        DBG_FMAC_INF("FW ptr: unknown id %d\n", id);
        return NULL;
    }
}

/**
 * @brief Get the firmware data size
 * @param id firmware type ID
 * @return firmware size in bytes, 0=not found
 */
uint32_t aic_fw_size_get(enum aic_fw id)
{
    switch (id)
    {
#if defined(CONFIG_AIC8801)
    case FMACFW:
        return sizeof(fmacfw);
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF:
        return sizeof(fmacfw_rf);
#endif
#endif

#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
    case FMACFW_PATCH_8800DC:
        return 0;
#if !defined(CONFIG_AIC8800DC_H) && !defined(CONFIG_AIC8800DW_H)
    case FMACFW_PATCH_8800DC_U02:
        return sizeof(fmacfw_patch_8800dc_u02);
    case FMACFW_PATCH_TBL_8800DC_U02:
        return sizeof(fmacfw_patch_tbl_8800dc_u02);
    case FMACFW_CALIB_8800DC_U02:
        return sizeof(fmacfw_calib_8800dc_u02);
#else
    case FMACFW_PATCH_8800DC_H_U02:
        return sizeof(fmacfw_patch_8800dc_h_u02);
    case FMACFW_PATCH_TBL_8800DC_H_U02:
        return sizeof(fmacfw_patch_tbl_8800dc_h_u02);
    case FMACFW_CALIB_8800DC_H_U02:
        return sizeof(fmacfw_calib_8800dc_h_u02);
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case LMACFW_RF_8800DC:
        return sizeof(lmacfw_rf_8800dc);
    case FMACFW_RF_PATCH_8800DC:
        return 0;
#endif
#endif

#if defined(CONFIG_AIC8800D80)
#if !defined(CONFIG_AIC8800D80_H)
    case FMACFW_8800D80:
        return 0;
    case FMACFW_8800D80_U02:
        return sizeof(fmacfw_8800d80_u02);
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF_8800D80_U02:
        return sizeof(lmacfw_rf_8800d80_u02);
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF_8800D80:
        return 0;
#endif
#else
#if defined(CONFIG_USB_SUPPORT)
    case FMACFW_8800D80_H_U02_USB:
        return sizeof(fmacfw_8800d80_h_u02_usb);
#if defined(CONFIG_FOR_IPCAM)
    case FMACFW_8800D80_H_U02_USB_IPC:
        return sizeof(fmacfw_8800d80_h_u02_usb_ipc);
#endif
#else
    case FMACFW_8800D80_H_U02:
        return sizeof(fmacfw_8800d80_h_u02);
#if defined(CONFIG_FOR_IPCAM)
    case FMACFW_8800D80_H_U02_IPC:
        return sizeof(fmacfw_8800d80_h_u02_ipc);
#endif
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case FMACFW_RF_8800D80_U02:
        return sizeof(lmacfw_rf_8800d80_u02);
#endif
#endif
#endif

#if defined(CONFIG_AIC8800D80N)
    /* U02 (current chip version) */
    case FMACFW_PATCH_8800D80N_U02:
        return sizeof(fmacfw_patch_8800d80n_u02);
    case FMACFW_PATCH_TBL_8800D80N_U02:
        return sizeof(fmacfw_patch_tbl_8800d80n_u02);
    case FMACFW_CINIT_8800D80N_U02:
        return sizeof(fmacfw_cinit_8800d80n_u02);
    case FMACFW_CALIB_8800D80N_U02:
        return sizeof(fmacfw_calib_8800d80n_u02);
        /* U02C — compiled only when CONFIG_AIC8800D80N_U02C is defined */
#if defined(CONFIG_AIC8800D80N_U02C)
    case FMACFW_PATCH_8800D80N_U02C:
        return sizeof(fmacfw_patch_8800d80n_u02c);
    case FMACFW_PATCH_TBL_8800D80N_U02C:
        return sizeof(fmacfw_patch_tbl_8800d80n_u02c);
    case FMACFW_CINIT_8800D80N_U02C:
        return sizeof(fmacfw_cinit_8800d80n_u02c);
    case FMACFW_CALIB_8800D80N_U02C:
        return sizeof(fmacfw_calib_8800d80n_u02c);
#endif /* CONFIG_AIC8800D80N_U02C */
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case LMACFW_RF_8800D80N:
        return sizeof(lmacfw_rf_8800d80n);
    /* 8819i (upper 2 bits of chip_id = 0xC0) RF test firmware */
    case LMACFW_RF_8800D80N_I:
        return sizeof(lmacfw_rf_8800d80n_i);
#endif
#endif

#if defined(CONFIG_AIC8800D80X2)
    case FMACFW_8800D80X2:
        return sizeof(fmacfw_8800d80x2);
#if defined(CONFIG_WIFI_MODE_RFTEST)
    case LMACFW_RF_8800D80X2:
        return sizeof(lmacfw_rf_8800d80x2);
#endif
#endif

#ifdef CONFIG_BT_SUPPORT
#if defined(CONFIG_AIC8801)
    case FW_ADID_U03:
        return sizeof(fw_adid_u03);
    case FW_PATCH_U03:
        return sizeof(fw_patch_u03);
    case FW_PATCH_TABLE_U03:
        return sizeof(fw_patch_table_u03);
#endif
#if defined(CONFIG_AIC8800D80)
    case FW_ADID_8800D80_U02:
        return sizeof(fw_adid_8800d80_u02);
    case FW_PATCH_8800D80_U02:
        return sizeof(fw_patch_8800d80_u02);
    case FW_PATCH_TABLE_8800D80_U02:
        return sizeof(fw_patch_table_8800d80_u02);
    case FW_PATCH_8800D80_U02_EXT:
        return sizeof(fw_patch_8800d80_u02_ext0);
    case FW_PATCH_8800D80_U02_EXT1:
        return sizeof(fw_patch_8800d80_u02_ext1);
#endif
#if defined(CONFIG_AIC8800D80N)
    case FW_ADID_8800D80N_U02:
        return sizeof(fw_adid_8800d80n_u02);
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_ADID_8800D80N_U02C:
        return sizeof(fw_adid_8800d80n_u02c);
#endif /* CONFIG_AIC8800D80N_U02C */
    case FW_PATCH_8800D80N_U02:
        return sizeof(fw_patch_8800d80n_u02);
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_8800D80N_U02C:
        return sizeof(fw_patch_8800d80n_u02c);
#endif /* CONFIG_AIC8800D80N_U02C */
    case FW_PATCH_TABLE_8800D80N_U02:
        return sizeof(fw_patch_table_8800d80n_u02);
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_TABLE_8800D80N_U02C:
        return sizeof(fw_patch_table_8800d80n_u02c);
#endif /* CONFIG_AIC8800D80N_U02C */
    case FW_PATCH_8800D80N_U02_EXT:
        return sizeof(fw_patch_8800d80n_u02_ext0);
#if defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_8800D80N_U02C_EXT:
        return sizeof(fw_patch_8800d80n_u02c_ext0);
#endif /* CONFIG_AIC8800D80N_U02C */
#if defined(CONFIG_WIFI_MODE_RFTEST)
    /* D80N RFTEST BT firmware (the _rf_ variant used by Linux CPMODE_TEST) */
    case FW_PATCH_8800D80N_RF_U02:
        return sizeof(fw_patch_8800d80n_rf_u02);
    case FW_PATCH_8800D80N_RF_U02_EXT:
        return sizeof(fw_patch_8800d80n_rf_u02_ext0);
    case FW_PATCH_TABLE_8800D80N_RF_U02:
        return sizeof(fw_patch_table_8800d80n_rf_u02);
#endif
#if defined(CONFIG_WIFI_MODE_RFTEST) && defined(CONFIG_AIC8800D80N_U02C)
    case FW_PATCH_8800D80N_RF_U02C:
        return sizeof(fw_patch_8800d80n_rf_u02c);
    case FW_PATCH_8800D80N_RF_U02C_EXT:
        return sizeof(fw_patch_8800d80n_rf_u02c_ext0);
    case FW_PATCH_TABLE_8800D80N_RF_U02C:
        return sizeof(fw_patch_table_8800d80n_rf_u02c);
#endif
#endif
#if defined(CONFIG_AIC8800D80X2)
    case FW_ADID_8800D80X2_U05:
        return sizeof(fw_adid_8800d80x2_u05);
    case FW_PATCH_8800D80X2_U05:
        return sizeof(fw_patch_8800d80x2_u05);
    case FW_PATCH_TABLE_8800D80X2_U05:
        return sizeof(fw_patch_table_8800d80x2_u05);
    case FW_PATCH_8800D80X2_U05_EXT0:
        return sizeof(fw_patch_8800d80x2_u05_ext0);
    case FW_PATCH_8800D80X2_U05_EXT1:
        return sizeof(fw_patch_8800d80x2_u05_ext1);
#endif
#if defined(CONFIG_AIC8800DC_H)
    case FW_ADID_8800DC_U02H:
        return sizeof(fw_adid_8800dc_u02h);
    case FW_PATCH_8800DC_U02H:
        return sizeof(fw_patch_8800dc_u02h);
    case FW_PATCH_TABLE_8800DC_U02H:
        return sizeof(fw_patch_table_8800dc_u02h);
#endif
#endif /* CONFIG_BT_SUPPORT */

    default:
        return 0;
    }
}
#else /* CONFIG_LOAD_FW != 2: firmware array not embedded, file/Flash mode does not use this interface */
void *aic_fw_ptr_get(enum aic_fw id)
{
    (void)id;
    return NULL;
}

uint32_t aic_fw_size_get(enum aic_fw id)
{
    (void)id;
    return 0;
}
#endif /* CONFIG_LOAD_FW == 2 */

/* ==========================================================================
 * Firmware loader (from former fmac_port.c)
 * ========================================================================== */

#if CONFIG_LOAD_FW == 0   /* filemode */
#include <unistd.h>
#include <fcntl.h>

/**
 * @brief Read a firmware binary file from the file system
 *
 * Look up the file named fw_name under AICBSP_FW_PATH and read its
 * entire content into a dynamically allocated buffer.
 *
 * @param fw_name firmware file name
 * @param fw_buf  output firmware buffer pointer
 * @return firmware size in bytes, 0=failure
 */
int aicwf_rwnx_read_fw_bin(char *fw_name, uint32_t **fw_buf)
{
    char path[128];
    void *buffer;
    int fd, n;
    off_t length;

    *fw_buf = NULL;
    if (!fw_name)
        return 0;

    snprintf(path, 128, "%s/%s", AICBSP_FW_PATH, fw_name);

    fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        DBG_FMAC_INF("Can not open the file %s\n", path);
    }
    else
    {
        length = lseek(fd, 0, SEEK_END);
        lseek(fd, 0, SEEK_SET);
        buffer = aicwf_rtos_malloc(length);
        if (buffer)
        {
            n = read(fd, buffer, length);
            if (n != length)
            {
                DBG_FMAC_INF("Can not read fw %s\n", path);
                aicwf_rtos_free(buffer);
                close(fd);
                return 0;
            }
            *fw_buf = buffer;
            return length;
        }
        close(fd);
    }

    return 0;
}

/**
 * @brief Load WiFi firmware
 *
 * Select the firmware file name based on chip ID and mode, then load it
 * from the file system.
 *
 * @param chipid chip product ID
 * @param mode   WiFi mode
 * @param fw_buf output firmware buffer pointer
 * @return firmware size in bytes, 0=failure
 */
int aicwf_rwnx_load_firmware(uint16_t chipid, int mode, uint32_t **fw_buf)
{
    int size = 0;
    char *fw_name = NULL;
    *fw_buf = NULL;

    if (chipid == PRODUCT_ID_AIC8801)
    {
        if (mode == WIFI_MODE_RFTEST)
        {
            fw_name = "fmacfw_rf.bin";
        }
        else
        {
            fw_name = "fmacfw.bin";
        }
    }
    else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 0)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                if (aic_get_aic8800dc_rf_flag() == 0)
                {
                    fw_name = "lmacfw_rf_8800dc.bin";
                }
                else if (aic_get_aic8800dc_rf_flag() == 1)
                {
                    fw_name = "fmacfw_rf_patch_8800dc.bin";
                }
            }
            else
            {
                fw_name = "fmacfw_patch_8800dc.bin";
            }
        }
        else if (aic_get_chip_sub_id() == 1)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800dc.bin";
            }
            else
            {
                fw_name = "fmacfw_patch_8800dc_u02.bin";
            }
        }
    }
    else if (chipid == PRODUCT_ID_AIC8800D80)
    {
        if (aic_get_chip_id_h())
        {
            /* H version: firmware differs by bus + IPC (see Linux aic_bsp_main.c) */
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800d80_u02.bin";
            }
            else
            {
#if defined(CONFIG_USB_SUPPORT)
#if defined(CONFIG_FOR_IPCAM)
                fw_name = "fmacfw_8800d80_h_u02_usb_ipc.bin";
#else
                fw_name = "fmacfw_8800d80_h_u02_usb.bin";
#endif
#else
#if defined(CONFIG_FOR_IPCAM)
                fw_name = "fmacfw_8800d80_h_u02_ipc.bin";
#else
                fw_name = "fmacfw_8800d80_h_u02.bin";
#endif
#endif
            }
        }
        else if (aic_get_chip_id() == CHIP_REV_U01)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800d80.bin";
            }
            else
            {
                fw_name = "fmacfw_8800d80.bin";
            }
        }
        else
        {
            /* U02/U03 and later versions */
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800d80_u02.bin";
            }
            else
            {
                fw_name = "fmacfw_8800d80_u02.bin";
            }
        }
    }
    else if (chipid == PRODUCT_ID_AIC8800D80N)
    {
        /* D80N main FMAC reuses D80 (Linux: D80N wl_fw = fmacfw_8800d80.bin) */
        if (mode == WIFI_MODE_RFTEST)
        {
            fw_name = "lmacfw_rf_8800d80n.bin";
        }
        else
        {
            fw_name = "fmacfw_8800d80_u02.bin";
        }
    }

    return aicwf_rwnx_read_fw_bin(fw_name, fw_buf);
}

/**
 * @brief Load patch table firmware
 *
 * Only DC/DW series chips need the patch table.
 *
 * @param chipid chip product ID
 * @param fw_buf output firmware buffer pointer
 * @return firmware size in bytes, 0=not needed or failure
 */
int aicwf_rwnx_load_patch_tbl(uint16_t chipid, uint32_t **fw_buf)
{
    int size = 0;
    char *fw_name = NULL;

    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 1)
        {
            fw_name = "fmacfw_patch_tbl_8800dc_u02.bin";
        }
    }

    return aicwf_rwnx_read_fw_bin(fw_name, fw_buf);
}

/**
 * @brief Release the firmware buffer
 * @param fw_buf pointer to the firmware buffer pointer
 */
void aicwf_rwnx_restore_firmware(uint32_t **fw_buf)
{
    aicwf_rtos_free(*fw_buf);
    *fw_buf = NULL;
}

/* filemode: no flash firmware type detection, fallback returns 0 */
uint32_t aicwf_get_wifi_fw_type(void)
{
    return 0;
}

#elif CONFIG_LOAD_FW == 1  /* flashmode */
#include "ReliableData.h"
#include "MRD.h"
#include "FlashPartition.h"

extern int RDWIFI_ReadFmSize(int type, int *length, unsigned int *fmaddr, int *headerlen);
extern void Rdisk_FlashRead(unsigned int flashaddress, unsigned char *buffer, unsigned int size);

extern unsigned int LzmaUncompressLen(const unsigned char *src);
int LzmaUncompress(unsigned char *dest, size_t  *destLen, const unsigned char *src, size_t  *srcLen);

#if (defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW))
    #define WIFI_LZMA_LEN               (128*1024)
#elif defined(CONFIG_AIC8800D80)
    #define WIFI_LZMA_LEN               (256*1024)
#endif
static void *wifibin = NULL;
static uint8_t rdwifi_fw_type = 0;

/**
 * @brief Check whether firmware data is in LZMA compressed format
 * @param src firmware data pointer
 * @return 1=compressed, 0=uncompressed
 */
static unsigned char aicwf_isLzmaCompressed(const unsigned char *src)
{
    unsigned int len = 0;
    int ii = 0;
    unsigned char *temp = NULL;
    unsigned char header_fixed[5] = {0x5d, 0x00, 0x00, 0x10, 0x00};

    temp = (unsigned char *)src;
    for (ii = 0; ii < sizeof(header_fixed); ii++)
    {
        if (*(temp + ii) != header_fixed[ii])
            break;
    }
    if (ii == sizeof(header_fixed))
        return 1;
    else
        return 0;
}

/**
 * @brief Get the WiFi firmware type (flash mode)
 * @return firmware type flags
 */
uint32_t aicwf_get_wifi_fw_type(void)
{
    return rdwifi_fw_type;
}

/**
 * @brief Detect the WiFi firmware type from Flash partitions
 *
 * Read the firmware type flags from the WiFi and MRD areas in Flash to
 * determine whether normal firmware or RF test firmware is stored.
 *
 * @return firmware type flags (bit0=normal firmware, bit1=RF test firmware, bit2=calibration area firmware)
 */
unsigned char RDWIFI_Fw_type(void)
{
    FlashLayoutConfInfo *pFlashLayout = GetFlashLayoutConfig();
    int   headersize = (sizeof(RD_BUFFER_HEADER) + sizeof(RD_ENTRY));
    UINT8  *WiFiArea = (UINT8 *)aicwf_rtos_malloc(headersize);
    UINT8 *pRAMbuf = NULL;
    RD_ENTRY entry = {0};
    RD_BUFFER_HEADER header = {0};

    Rdisk_FlashRead(pFlashLayout->WIFIFirmAddress, WiFiArea, headersize);

    pRAMbuf = WiFiArea + sizeof(RD_BUFFER_HEADER);

    memcpy(&entry, (RD_ENTRY *)pRAMbuf, sizeof(RD_ENTRY));

    if (entry.entryType == MRD_WIFI_NORMAL_FM)
    {
        rdwifi_fw_type |= 0x01;         //nomal fw in wifi area
    }
    else if (entry.entryType == MRD_WIFI_CALI_FM)
    {
        rdwifi_fw_type |= 0x02;         //rf test fw in wifi area
    }

    if (pFlashLayout->WIFICalFirmAddress != 0xFFFFFFFF)
    {
        rdwifi_fw_type |= 0x04;         //rf rest fw in cal area
    }

    Rdisk_FlashRead(pFlashLayout->MRDStartAddress, WiFiArea, headersize);

    pRAMbuf = WiFiArea + sizeof(RD_BUFFER_HEADER);

    memcpy(&entry, (RD_ENTRY *)pRAMbuf, sizeof(RD_ENTRY));

    if (entry.entryType == MRD_WIFI_NORMAL_FM)
    {
        rdwifi_fw_type |= 0x08;       //patch and cali bin for rf rest
    }


    aicwf_rtos_free(WiFiArea);

    DBG_FMAC_INF("%s: WIFIFirm=%x,MRDStart=%x,type=%x", __func__,
                 pFlashLayout->WIFIFirmAddress, pFlashLayout->MRDStartAddress, rdwifi_fw_type);

    return rdwifi_fw_type;
}


/**
 * @brief Read WiFi firmware from Flash partitions (internal)
 *
 * Walk the Flash MRD partition to find firmware with the given file name and
 * type; decompress automatically if the firmware is LZMA compressed.
 *
 * @param fw_name firmware file name
 * @param fw_buf  output firmware buffer pointer
 * @param type    firmware type (1=normal firmware, 2=RF calibration firmware)
 * @return firmware size, negative=failure
 */
static int __aicwf_RDWIFI_ReadFm(char *fw_name, uint32_t **fw_buf, unsigned char type)
{
    RD_ENTRY entry = {0};
    RD_BUFFER_HEADER header = {0};
    UINT8 *pRAMbuf = NULL;
    int  ret = -1;
    int len, hlen;
    unsigned int fwaddr = 0, offset = 0;
    void *buffer;

    int   headersize = (sizeof(RD_BUFFER_HEADER) + sizeof(RD_ENTRY));
    UINT8  *WiFiArea = (UINT8 *)aicwf_rtos_malloc(headersize);

    FlashLayoutConfInfo *pFlashLayout = GetFlashLayoutConfig();

    if (NULL == WiFiArea)
    {
        DBG_FMAC_ERR("malloc wifiarea error");
        return -1;
    }

    fwaddr = pFlashLayout->WIFIFirmAddress;

    if (pFlashLayout->WIFICalFirmAddress != 0xFFFFFFFF)
    {
        if ((type == 2) && (!(rdwifi_fw_type & 0x02)))
            fwaddr = pFlashLayout->WIFICalFirmAddress;
    }

    if ((rdwifi_fw_type & 0x02) && (rdwifi_fw_type & 0x08) && (type == 1))
    {
        fwaddr = pFlashLayout->MRDStartAddress;
    }

    DBG_FMAC_INF("Read WiFi Firmware--->:%s, fwaddr=%x, type:%d", fw_name, fwaddr, type);

    hlen = sizeof(RD_BUFFER_HEADER);
    do
    {

        Rdisk_FlashRead(fwaddr, WiFiArea, headersize);
        pRAMbuf = WiFiArea + hlen;

        {
            UINT32 entry_size = 0;
            UINT8  pad_size = 0;
            UINT32 file_size = 0;

            memcpy(&entry, (RD_ENTRY *)pRAMbuf, sizeof(RD_ENTRY));
            entry_size = (entry.fileAndPadSize & 0x00FFFFFF);
            pad_size = (UINT8)(((entry.fileAndPadSize & 0xFF000000) >> 24));
            DBG_FMAC_INF("name:%s:%d,%d", entry.file_name, entry_size, pad_size);

            //check type
            if (type == 1 && entry.entryType == MRD_WIFI_NORMAL_FM ||
                    type == 2 && entry.entryType == MRD_WIFI_CALI_FM)
            {
                file_size = entry_size - pad_size;
                if (file_size == 0)
                {
                    ret = -2;
                    goto end;
                }
            }
            else
            {
                ret = -2;
                goto end;
            }

            //find file name
            if (!strcmp(fw_name, entry.file_name))
            {

                buffer = aicwf_rtos_malloc(file_size);
                if (buffer != NULL)
                {
                    Rdisk_FlashRead(fwaddr + headersize - 4, buffer, file_size);
                    if (aicwf_isLzmaCompressed(buffer))
                    {
                        unsigned int wificombosize = LzmaUncompressLen(buffer);
                        void *wifibin = aicwf_rtos_malloc(wificombosize);
                        unsigned int combolzmalen = WIFI_LZMA_LEN;
                        ret = LzmaUncompress(wifibin, &wificombosize, buffer, &combolzmalen);
                        if (!ret)
                        {
                            *fw_buf = wifibin;
                            ret = wificombosize;
                            aicwf_rtos_free(buffer);

                            DBG_FMAC_INF("decomp WiFi Firmware--->:%d %d\n", wificombosize, **fw_buf);
                        }
                        else
                        {
                            ret = -3;
                            aicwf_rtos_free(buffer);
                            aicwf_rtos_free(wifibin);
                        }
                    }
                    else
                    {
                        *fw_buf = buffer;
                        ret = file_size;
                    }

                }
                else
                {
                    ret = -4;
                }
                break;
            }
            else
            {
                fwaddr += (entry_size + headersize - 4);
                hlen = 0;
                headersize = sizeof(RD_ENTRY);
            }

        }
    }
    while (1);
end:
    aicwf_rtos_free(WiFiArea);

    return ret;
}


static int aicwf_rwnx_read_fw_bin(char *fw_name, uint32_t **fw_buf, unsigned char mode)
{
    int ret;

    ret = __aicwf_RDWIFI_ReadFm(fw_name, fw_buf, mode);

    if (ret <= 0)
    {
        DBG_FMAC_INF("aicwf_rwnx_read_fw_bin: %d\n", ret);
        aic_set_chip_fw_match(ret);
        return 0;
    }
    else
    {
        aic_set_chip_fw_match(1);
        return ret;
    }
}

/**
 * @brief Load WiFi firmware
 *
 * Select the firmware file name based on chip ID and mode, then load it
 * from the file system.
 *
 * @param chipid chip product ID
 * @param mode   WiFi mode
 * @param fw_buf output firmware buffer pointer
 * @return firmware size in bytes, 0=failure
 */
int aicwf_rwnx_load_firmware(uint16_t chipid, int mode, uint32_t **fw_buf)
{
    unsigned char local_mode;
    char *fw_name = NULL;
    *fw_buf = NULL;

    if (chipid == PRODUCT_ID_AIC8801)
    {
        if (mode == WIFI_MODE_RFTEST)
        {
            fw_name = "fmacfw_rf.bin";
        }
        else
        {
            fw_name = "fmacfw.bin";
        }
    }
    else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 0)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                if (aic_get_aic8800dc_rf_flag() == 0)
                {
                    fw_name = "lmacfw_rf_8800dc.bin";
                }
                else if (aic_get_aic8800dc_rf_flag() == 1)
                {
                    fw_name = "fmacfw_rf_patch_8800dc.bin";
                }
            }
            else
            {
                fw_name = "fmacfw_patch_8800dc.bin";
            }
        }
        else if (aic_get_chip_sub_id() == 1)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800dc.bin";
            }
            else
            {
                fw_name = "fmacfw_patch_8800dc_u02.bin";
            }
        }
        else if (aic_get_chip_sub_id() == 2)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800dc.bin";
            }
            else
            {
#if defined(AIC_HCI_OVER_SDIO)
                fw_name = "fmacfw_patch_8800dc_hbt_u02.bin";
#else
                fw_name = "fmacfw_patch_8800dc_h_u02.bin";
#endif
            }
        }
    }
    else if (chipid == PRODUCT_ID_AIC8800D80)
    {
        if (aic_get_chip_id_h())
        {
            /* H version: firmware differs by bus + IPC (see Linux aic_bsp_main.c) */
            if (mode == WIFI_MODE_RFTEST)
            {
                fw_name = "lmacfw_rf_8800d80_u02.bin";
            }
            else
            {
#if defined(CONFIG_USB_SUPPORT)
#if defined(CONFIG_FOR_IPCAM)
                fw_name = "fmacfw_8800d80_h_u02_usb_ipc.bin";
#else
                fw_name = "fmacfw_8800d80_h_u02_usb.bin";
#endif
#else
#if defined(CONFIG_FOR_IPCAM)
                fw_name = "fmacfw_8800d80_h_u02_ipc.bin";
#else
                fw_name = "fmacfw_8800d80_h_u02.bin";
#endif
#endif
            }
        }
        else
        {
            if (aic_get_chip_id() == CHIP_REV_U01)
            {
                if (mode == WIFI_MODE_RFTEST)
                {
                    fw_name = "lmacfw_rf_8800d80.bin";
                }
                else
                {
                    fw_name = "fmacfw_8800d80.bin";
                }
            }
            else
            {
                if (mode == WIFI_MODE_RFTEST)
                {
                    fw_name = "lmacfw_rf_8800d80_u02.bin";
                }
                else
                {
                    fw_name = "fmacfw_8800d80_u02.bin";
                }
            }
        }
    }
    else if (chipid == PRODUCT_ID_AIC8800D80N)
    {
        /* D80N main FMAC reuses D80 (Linux: D80N wl_fw = fmacfw_8800d80.bin) */
        if (mode == WIFI_MODE_RFTEST)
        {
            fw_name = "lmacfw_rf_8800d80n.bin";
        }
        else
        {
            fw_name = "fmacfw_8800d80_u02.bin";
        }
    }

    if (mode == WIFI_MODE_RFTEST)
        local_mode = 2;
    else
        local_mode = 1;

    return aicwf_rwnx_read_fw_bin(fw_name, fw_buf, local_mode);
}

/**
 * @brief Load patch table firmware
 *
 * Only DC/DW series chips need the patch table.
 *
 * @param chipid chip product ID
 * @param fw_buf output firmware buffer pointer
 * @return firmware size in bytes, 0=not needed or failure
 */
int aicwf_rwnx_load_patch_tbl(uint16_t chipid, uint32_t **fw_buf)
{
    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 1)
        {
            return aicwf_rwnx_read_fw_bin("fmacfw_patch_tbl_8800dc_u02.bin", fw_buf, 1);
        }
        else if (aic_get_chip_sub_id() == 2)
        {
#if defined(AIC_HCI_OVER_SDIO)
            return aicwf_rwnx_read_fw_bin("fmacfw_patch_tbl_8800dc_hbt_u02.bin", fw_buf, 1);
#else
            return aicwf_rwnx_read_fw_bin("fmacfw_patch_tbl_8800dc_h_u02.bin", fw_buf, 1);
#endif
        }
    }

    return 0;
}

/**
 * @brief Load calibration firmware (flash mode)
 *
 * Only DC/DW series chips need calibration data.
 *
 * @param chipid chip product ID
 * @param fw_buf output firmware buffer pointer
 * @return firmware size, 0=not needed or failure
 */
int aicwf_rwnx_load_calib(uint16_t chipid, uint32_t **fw_buf)
{
    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 1)
        {
            return aicwf_rwnx_read_fw_bin("fmacfw_calib_8800dc_u02.bin", fw_buf, 1);
        }
        else if (aic_get_chip_sub_id() == 2)
        {
#if defined(AIC_HCI_OVER_SDIO)
            return aicwf_rwnx_read_fw_bin("fmacfw_calib_8800dc_hbt_u02.bin", fw_buf, 1);
#else
            return aicwf_rwnx_read_fw_bin("fmacfw_calib_8800dc_h_u02.bin", fw_buf, 1);
#endif
        }
    }

    return 0;
}

/**
 * @brief Release the firmware buffer
 * @param fw_buf pointer to the firmware buffer pointer
 */
void aicwf_rwnx_restore_firmware(uint32_t **fw_buf)
{
    aicwf_rtos_free(*fw_buf);
    *fw_buf = NULL;
}
#elif CONFIG_LOAD_FW == 2
/* innermode: read from array */

/**
 * @brief Load WiFi firmware
 *
 * Select the firmware file name based on chip ID and mode, then load it
 * from the file system.
 *
 * @param chipid chip product ID
 * @param mode   WiFi mode
 * @param fw_buf output firmware buffer pointer
 * @return firmware size in bytes, 0=failure
 */
int aicwf_rwnx_load_firmware(uint16_t chipid, int mode, uint32_t **fw_buf)
{
    int size = 0;
    *fw_buf = NULL;

    if (chipid == PRODUCT_ID_AIC8801)
    {
#if defined(CONFIG_AIC8801)
        if (mode == WIFI_MODE_RFTEST)
        {
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_RF);
            size = aic_fw_size_get(FMACFW_RF);
        }
        else
        {
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW);
            size = aic_fw_size_get(FMACFW);
        }
#endif /* CONFIG_AIC8801 */
    }
    else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
        if (aic_get_chip_sub_id() == 0)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
                if (aic_get_aic8800dc_rf_flag() == 0)
                {
                    *fw_buf = (uint32_t *)aic_fw_ptr_get(LMACFW_RF_8800DC);
                    size = aic_fw_size_get(LMACFW_RF_8800DC);
                }
                else if (aic_get_aic8800dc_rf_flag() == 1)
                {
                    *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_RF_PATCH_8800DC);
                    size = aic_fw_size_get(FMACFW_RF_PATCH_8800DC);
                }
            }
            else
            {
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_PATCH_8800DC);
                size = aic_fw_size_get(FMACFW_PATCH_8800DC);
            }
        }
        else if (aic_get_chip_sub_id() == 1)
        {
            if (aic_get_aic8800dc_calib_flag() == 0)
            {
                if (mode == WIFI_MODE_RFTEST)
                {
                    DBG_FMAC_INF("aic load lmacfw_rf_8800dc.bin\n\r");
                    *fw_buf = (uint32_t *)aic_fw_ptr_get(LMACFW_RF_8800DC);
                    size = aic_fw_size_get(LMACFW_RF_8800DC);
                }
                else
                {
                    DBG_FMAC_INF("aic load fmacfw_patch_8800dc_u02.bin\n\r");
                    *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_PATCH_8800DC_U02);
                    size = aic_fw_size_get(FMACFW_PATCH_8800DC_U02);
                }
            }
            else if (aic_get_aic8800dc_calib_flag() == 1)
            {
                DBG_FMAC_INF("aic load fmacfw_calib_8800dc_u02.bin\n\r");
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_CALIB_8800DC_U02);
                size = aic_fw_size_get(FMACFW_CALIB_8800DC_U02);
            }
        }
        else if (aic_get_chip_sub_id() == 2)
        {
            if (aic_get_aic8800dc_calib_flag() == 0)
            {
                if (mode == WIFI_MODE_RFTEST)
                {
                    DBG_FMAC_INF("aic load lmacfw_rf_8800dc.bin\n\r");
                    *fw_buf = (uint32_t *)aic_fw_ptr_get(LMACFW_RF_8800DC);
                    size = aic_fw_size_get(LMACFW_RF_8800DC);
                }
                else
                {
                    DBG_FMAC_INF("aic load fmacfw_patch_8800dc_h_u02.bin\n\r");
                    *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_PATCH_8800DC_H_U02);
                    size = aic_fw_size_get(FMACFW_PATCH_8800DC_H_U02);
                }
            }
            else if (aic_get_aic8800dc_calib_flag() == 1)
            {
                DBG_FMAC_INF("aic load fmacfw_calib_8800dc_h_u02.bin\n\r");
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_CALIB_8800DC_H_U02);
                size = aic_fw_size_get(FMACFW_CALIB_8800DC_H_U02);
            }
        }
#endif /* CONFIG_AIC8800DC or CONFIG_AIC8800DW */
    }
    else if (chipid == PRODUCT_ID_AIC8800D80)
    {
#if defined(CONFIG_AIC8800D80)
        if (aic_get_chip_id_h())
        {
            /* H version: firmware differs by bus + IPC (see Linux aic_bsp_main.c) */
            if (mode == WIFI_MODE_RFTEST)
            {
#if defined(CONFIG_WIFI_MODE_RFTEST)
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_RF_8800D80_U02);
                size = aic_fw_size_get(FMACFW_RF_8800D80_U02);
#endif
            }
            else
            {
#if defined(CONFIG_USB_SUPPORT)
#if defined(CONFIG_FOR_IPCAM)
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80_H_U02_USB_IPC);
                size = aic_fw_size_get(FMACFW_8800D80_H_U02_USB_IPC);
#else
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80_H_U02_USB);
                size = aic_fw_size_get(FMACFW_8800D80_H_U02_USB);
#endif
#else
#if defined(CONFIG_FOR_IPCAM)
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80_H_U02_IPC);
                size = aic_fw_size_get(FMACFW_8800D80_H_U02_IPC);
#else
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80_H_U02);
                size = aic_fw_size_get(FMACFW_8800D80_H_U02);
#endif
#endif
            }
        }
        else if (aic_get_chip_id() == CHIP_REV_U01)
        {
            if (mode == WIFI_MODE_RFTEST)
            {
#if defined(CONFIG_WIFI_MODE_RFTEST)
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_RF_8800D80);
                size = aic_fw_size_get(FMACFW_RF_8800D80);
#endif
            }
            else
            {
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80);
                size = aic_fw_size_get(FMACFW_8800D80);
            }
        }
        else
        {
            /* U02/U03 and later versions */
            if (mode == WIFI_MODE_RFTEST)
            {
#if defined(CONFIG_WIFI_MODE_RFTEST)
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_RF_8800D80_U02);
                size = aic_fw_size_get(FMACFW_RF_8800D80_U02);
#endif
            }
            else
            {
                *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80_U02);
                size = aic_fw_size_get(FMACFW_8800D80_U02);
            }
        }
#endif /* CONFIG_AIC8800D80 */
    }
    else if (chipid == PRODUCT_ID_AIC8800D80N)
    {
#if defined(CONFIG_AIC8800D80N)
        if (mode == WIFI_MODE_RFTEST)
        {
#if defined(CONFIG_WIFI_MODE_RFTEST)
            *fw_buf = (uint32_t *)aic_fw_ptr_get(LMACFW_RF_8800D80N);
            size = aic_fw_size_get(LMACFW_RF_8800D80N);
#endif
        }
        else
        {
            /* D80N main FMAC reuses D80 (Linux: D80N wl_fw = fmacfw_8800d80.bin) */
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80_U02);
            size = aic_fw_size_get(FMACFW_8800D80_U02);
        }
#endif /* CONFIG_AIC8800D80N */
    }
    else if (chipid == PRODUCT_ID_AIC8800D80X2)
    {
#if defined(CONFIG_AIC8800D80X2)
        if (mode == WIFI_MODE_RFTEST)
        {
#if defined(CONFIG_WIFI_MODE_RFTEST)
            *fw_buf = (uint32_t *)aic_fw_ptr_get(LMACFW_RF_8800D80X2);
            size = aic_fw_size_get(LMACFW_RF_8800D80X2);
#endif
        }
        else
        {
            /* D80X2 U05 baseline main FMAC: fmacfw_8800d80x2.bin */
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_8800D80X2);
            size = aic_fw_size_get(FMACFW_8800D80X2);
        }
#endif /* CONFIG_AIC8800D80X2 */
    }
    return size;
}

/**
 * @brief Load patch table firmware
 *
 * Only DC/DW series chips need the patch table.
 *
 * @param chipid chip product ID
 * @param fw_buf output firmware buffer pointer
 * @return firmware size in bytes, 0=not needed or failure
 */
int aicwf_rwnx_load_patch_tbl(uint16_t chipid, uint32_t **fw_buf)
{
    int size = 0;

    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
        if (aic_get_chip_sub_id() == 1)
        {
            DBG_FMAC_INF("aic load fmacfw_patch_tbl_8800dc_u02.bin\n\r");
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_PATCH_TBL_8800DC_U02);
            size = aic_fw_size_get(FMACFW_PATCH_TBL_8800DC_U02);
        }
        else if (aic_get_chip_sub_id() == 2)
        {
            DBG_FMAC_INF("aic load fmacfw_patch_tbl_8800dc_h_u02.bin\n\r");
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_PATCH_TBL_8800DC_H_U02);
            size = aic_fw_size_get(FMACFW_PATCH_TBL_8800DC_H_U02);
        }
#endif /* CONFIG_AIC8800DC or CONFIG_AIC8800DW */
    }
    return size;
}

/**
 * @brief Load calibration firmware (flash mode)
 *
 * Only DC/DW series chips need calibration data.
 *
 * @param chipid chip product ID
 * @param fw_buf output firmware buffer pointer
 * @return firmware size, 0=not needed or failure
 */
int aicwf_rwnx_load_calib(uint16_t chipid, uint32_t **fw_buf)
{
    int size = 0;
#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 1)
        {
            DBG_FMAC_INF("aic load fmacfw_calib_8800dc_u02.bin\n\r");
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_CALIB_8800DC_U02);
            size = aic_fw_size_get(FMACFW_CALIB_8800DC_U02);
        }
        else if (aic_get_chip_sub_id() == 2)
        {
            DBG_FMAC_INF("aic load fmacfw_calib_8800dc_h_u02.bin\n\r");
            *fw_buf = (uint32_t *)aic_fw_ptr_get(FMACFW_CALIB_8800DC_H_U02);
            size = aic_fw_size_get(FMACFW_CALIB_8800DC_H_U02);
        }
    }
#endif /* CONFIG_AIC8800DC or CONFIG_AIC8800DW */
    return size;
}

/**
 * @brief Release the firmware buffer
 * @param fw_buf pointer to the firmware buffer pointer
 */
void aicwf_rwnx_restore_firmware(uint32_t **fw_buf)
{
    *fw_buf = NULL;
}

/* innermode: no flash firmware type detection, fallback returns 0 */
uint32_t aicwf_get_wifi_fw_type(void)
{
    return 0;
}

#endif

/**
 * @brief Check whether RF test firmware is available (platform layer added it via RFTEST switch)
 *
 * innermode: check per build-time chip model that the RF firmware array was compiled in.
 * filemode/flashmode: reuse fw_type bit detection (valid for flash; filemode has none, returns 0).
 *
 * @return 1=available, 0=unavailable
 */
int aicwf_rftest_fw_available(void)
{
#if CONFIG_LOAD_FW == 2
#if defined(CONFIG_AIC8801)
    return (aic_fw_ptr_get(FMACFW_RF) != NULL && aic_fw_size_get(FMACFW_RF) > 0) ? 1 : 0;
#elif defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
    return (aic_fw_ptr_get(LMACFW_RF_8800DC) != NULL && aic_fw_size_get(LMACFW_RF_8800DC) > 0) ? 1 : 0;
#elif defined(CONFIG_AIC8800D80)
    return (aic_fw_ptr_get(FMACFW_RF_8800D80_U02) != NULL && aic_fw_size_get(FMACFW_RF_8800D80_U02) > 0) ? 1 : 0;
#elif defined(CONFIG_AIC8800D80N)
    return (aic_fw_ptr_get(LMACFW_RF_8800D80N) != NULL && aic_fw_size_get(LMACFW_RF_8800D80N) > 0) ? 1 : 0;
#elif defined(CONFIG_AIC8800D80X2)
    return (aic_fw_ptr_get(LMACFW_RF_8800D80X2) != NULL && aic_fw_size_get(LMACFW_RF_8800D80X2) > 0) ? 1 : 0;
#else
    return 0;
#endif
#else
    return (aicwf_get_wifi_fw_type() & 0x02) ? 1 : 0;
#endif
}
