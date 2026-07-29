/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rwnx_platform.h"
//#include "reg_access.h"
//#include "hal_desc.h"
#include "rwnx_main.h"
#include "rwnx_msg_tx.h"
//#include "rwnx_utils.h"
#include "aicwf_config.h"
#include "rwnx_defs.h"
#include "osal_debug.h"

#ifdef AICWF_SDIO_SUPPORT
    //#include "aicwf_sdio.h"
#endif

#ifdef AICWF_USB_SUPPORT
    //#include "aicwf_usb.h"
#endif

struct rwnx_plat *g_rwnx_plat = NULL;

extern u8 chip_id;
extern u8 chip_sub_id;
extern u8 chip_mcu_id;
extern u8 aic8800dc_rf_flag;
u8 aic8800dc_calib_flag = 0;

#ifdef CONFIG_DOWNLOAD_FW

int aicwf_misc_ram_init_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    uint32_t cfg_base = 0x10164;
    struct dbg_mem_read_cfm cfm;
    uint32_t misc_ram_addr;
    uint32_t misc_ram_size = 12;
    int i;
    if (rwnx_hw->mode == WIFI_MODE_RFTEST)
    {
        cfg_base = RAM_LMAC_FW_ADDR + 0x0164;
    }
    // init misc ram
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x14, &cfm);
    if (ret)
    {
        DBG_MACIF_INF("rf misc ram[0x%x] rd fail: %d\n", cfg_base + 0x14, ret);
        return ret;
    }
    misc_ram_addr = cfm.memdata;
    DBG_MACIF_INF("misc_ram_addr=%x\n", misc_ram_addr);
    for (i = 0; i < (misc_ram_size / 4); i++)
    {
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, misc_ram_addr + i * 4, 0);
        if (ret)
        {
            DBG_MACIF_INF("rf misc ram[0x%x] wr fail: %d\n",  misc_ram_addr + i * 4, ret);
            return ret;
        }
    }
    return ret;
}

static int rwnx_load_firmware(struct rwnx_hw *rwnx_hw, u32 **fw_buf)
{
    int size = 0;

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
    {
#if defined(CONFIG_AIC8801)
        if (rwnx_hw->mode == WIFI_MODE_RFTEST)
        {
            *fw_buf = (u32 *)aic8800d_rf_fw_ptr_get();
            size = aic8800d_rf_fw_size_get();
        }
        else
        {
            *fw_buf = (u32 *)aic8800d_fw_ptr_get();
            size = aic8800d_fw_size_get();
        }
#endif
    }
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    {
#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
        if (chip_sub_id == 0)
        {
            if (rwnx_hw->mode == WIFI_MODE_RFTEST)
            {
                if (aic8800dc_rf_flag == 0)
                {
                    *fw_buf = (u32 *)aic8800dc_rf_lmacfw_ptr_get();     //lmacfw_rf_8800dc.bin
                    size = aic8800dc_rf_lmacfw_size_get();
                }
                else if (aic8800dc_rf_flag == 1)
                {
                    *fw_buf = (u32 *)aic8800dc_rf_fmacfw_ptr_get();     //fmacfw_rf_patch_8800dc.bin
                    size = aic8800dc_rf_fmacfw_size_get();
                }
            }
            else
            {
                *fw_buf = (u32 *)aic8800dc_u01_fw_ptr_get();            //fmacfw_patch_8800dc.bin
                size = aic8800dc_u01_fw_size_get();
            }
        }
        else if (chip_sub_id == 1)
        {
            if (aic8800dc_calib_flag == 0)
            {
                if (rwnx_hw->mode == WIFI_MODE_RFTEST)
                {
                    DBG_MACIF_INF("aic load lmacfw_rf_8800dc.bin\n\r");
                    *fw_buf = (u32 *)aic8800dc_rf_lmacfw_ptr_get();         //lmacfw_rf_8800dc.bin
                    size = aic8800dc_rf_lmacfw_size_get();
                }
                else
                {
                    DBG_MACIF_INF("aic load fmacfw_patch_8800dc_u02.bin\n\r");
                    *fw_buf = (u32 *)aic8800dc_u02_fw_ptr_get();            //fmacfw_patch_8800dc_u02.bin
                    size = aic8800dc_u02_fw_size_get();
                }
            }
            else if (aic8800dc_calib_flag == 1)
            {
                DBG_MACIF_INF("aic load fmacfw_calib_8800dc_u02.bin\n\r");
                *fw_buf = (u32 *)aic8800dc_u02_calib_fw_ptr_get();            //fmacfw_calib_8800dc_u02.bin
                size = aic8800dc_u02_calib_fw_size_get();
            }
        }
        else if (chip_sub_id == 2)
        {
            if (aic8800dc_calib_flag == 0)
            {
                if (rwnx_hw->mode == WIFI_MODE_RFTEST)
                {
                    DBG_MACIF_INF("aic load lmacfw_rf_8800dc.bin\n\r");
                    *fw_buf = (u32 *)aic8800dc_rf_lmacfw_ptr_get();         //lmacfw_rf_8800dc.bin
                    size = aic8800dc_rf_lmacfw_size_get();
                }
                else
                {
                    DBG_MACIF_INF("aic load fmacfw_patch_8800dc_h_u02.bin\n\r");
                    *fw_buf = (u32 *)aic8800dc_h_u02_fw_ptr_get();          //fmacfw_patch_8800dc_h_u02.bin
                    size = aic8800dc_h_u02_fw_size_get();
                }
            }
            else if (aic8800dc_calib_flag == 1)
            {
                DBG_MACIF_INF("aic load fmacfw_calib_8800dc_h_u02.bin\n\r");
                *fw_buf = (u32 *)aic8800dc_h_u02_calib_fw_ptr_get();            //fmacfw_calib_8800dc_h_u02.bin
                size = aic8800dc_h_u02_calib_fw_size_get();
            }
        }
#endif
    }
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
    {
#if defined(CONFIG_AIC8800D80)
        if (chip_id == CHIP_REV_U01)
        {
            if (rwnx_hw->mode == WIFI_MODE_RFTEST)
            {
                *fw_buf = (u32 *)aic8800d80_rf_fw_ptr_get();
                size = aic8800d80_rf_fw_size_get();
            }
            else
            {
                *fw_buf = (u32 *)aic8800d80_fw_ptr_get();
                size = aic8800d80_fw_size_get();
            }
        }
        else if (chip_id == CHIP_REV_U02 || chip_id == CHIP_REV_U03)
        {
            if (rwnx_hw->mode == WIFI_MODE_RFTEST)
            {
                *fw_buf = (u32 *)aic8800d80_u02_rf_fw_ptr_get();
                size = aic8800d80_u02_rf_fw_size_get();
            }
            else
            {
                *fw_buf = (u32 *)aic8800d80_u02_fw_ptr_get();
                size = aic8800d80_u02_fw_size_get();
            }
        }
#endif
    }
    return size;
}

static int rwnx_load_patch_tbl(struct rwnx_hw *rwnx_hw, u32 **fw_buf)
{
    int size = 0;

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    {
#if defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
        if (chip_sub_id == 1)
        {
            DBG_MACIF_INF("aic load fmacfw_patch_tbl_8800dc_u02.bin\n\r");
            *fw_buf = (u32 *)aic8800dc_u02_patch_tbl_ptr_get();         //fmacfw_patch_tbl_8800dc_u02.bin
            size = aic8800dc_u02_patch_tbl_size_get();
        }
        else if (chip_sub_id == 2)
        {
            DBG_MACIF_INF("aic load fmacfw_patch_tbl_8800dc_h_u02.bin\n\r");
            *fw_buf = (u32 *)aic8800dc_h_u02_patch_tbl_ptr_get();       //fmacfw_patch_tbl_8800dc_h_u02.bin
            size = aic8800dc_h_u02_patch_tbl_size_get();
        }
#endif
    }
    return size;
}

static void rwnx_restore_firmware(u32 **fw_buf)
{
    *fw_buf = NULL;
}

/* buffer is allocated by kzalloc */
static int rwnx_request_firmware_common(struct rwnx_hw *rwnx_hw, u32 **buffer)
{
    int size;
    if (rwnx_hw->fw_patch == 0)
        size = rwnx_load_firmware(rwnx_hw, buffer);
    else if (rwnx_hw->fw_patch == 1)
        size = rwnx_load_patch_tbl(rwnx_hw, buffer);
    return size;
}

static void rwnx_release_firmware_common(u32 **buffer)
{
    rwnx_restore_firmware(buffer);
}

#ifdef CONFIG_DPD
rf_misc_ram_lite_t dpd_res = {0,};
#endif

int aicwf_patch_table_load(struct rwnx_hw *rwnx_hw)
{
    int err = 0;
    unsigned int i = 0, size;
    u32 *dst;
    u8 *describle;
    u32 fmacfw_patch_tbl_8800dc_u02_describe_size = 124;
    u32 fmacfw_patch_tbl_8800dc_u02_describe_base;//read from patch_tbl

    rwnx_hw->fw_patch = 1;

    /* Copy the file on the Embedded side */
    //DBG_MACIF_INF("### Upload %s \n", filename);

    size = rwnx_request_firmware_common(rwnx_hw, &dst);
    if (!dst)
    {
        DBG_MACIF_INF("No such file or directory\n");
        return -1;
    }
    if (size <= 0)
    {
        DBG_MACIF_INF("wrong size of firmware file\n");
        dst = NULL;
        err = -1;
    }

    DBG_MACIF_INF("tbl size = %d \n", size);

    fmacfw_patch_tbl_8800dc_u02_describe_base = dst[0];
    DBG_MACIF_INF("FMACFW_PATCH_TBL_8800DC_U02_DESCRIBE_BASE = %x \n", fmacfw_patch_tbl_8800dc_u02_describe_base);

    if (!err && (i < size))
    {
        err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fmacfw_patch_tbl_8800dc_u02_describe_base, fmacfw_patch_tbl_8800dc_u02_describe_size + 4, dst);
        if (err)
        {
            DBG_MACIF_INF("write describe information fail \n");
        }

        describle = (u8 *)rtos_malloc(fmacfw_patch_tbl_8800dc_u02_describe_size);
        memcpy(describle, &dst[1], fmacfw_patch_tbl_8800dc_u02_describe_size);
        DBG_MACIF_INF("%s", describle);
        rtos_free(describle);
        describle = NULL;
    }

    if (!err && (i < size))
    {
        for (i = (128 / 4); i < (size / 4); i += 2)
        {
            DBG_MACIF_INF("patch_tbl:  %x  %x\n", dst[i], dst[i + 1]);
            err = rwnx_send_dbg_mem_write_req(rwnx_hw, dst[i], dst[i + 1]);
        }
        if (err)
        {
            DBG_MACIF_INF("bin upload fail: %x, err:%d\r\n", dst[i], err);
        }
    }

    if (dst)
    {
        rwnx_release_firmware_common(&dst);
    }

    return err;
}

//#ifndef CONFIG_ROM_PATCH_EN
/**
 * rwnx_plat_bin_fw_upload_2() - Load the requested binary FW into embedded side.
 *
 * @rwnx_hw: Main driver data
 * @fw_addr: Address where the fw must be loaded
 * @filename: Name of the fw.
 *
 * Load a fw, stored as a binary file, into the specified address
 */
int rwnx_plat_bin_fw_upload_2(struct rwnx_hw *rwnx_hw, u32 fw_addr)
{
    int err = 0;
    unsigned int i = 0, size;
    u32 *dst;
    const int BLOCK_SIZE = 1024;//480;//512;//

    /* Copy the file on the Embedded side */
    DBG_MACIF_INF("\n### Upload firmware, @ = %x\n", fw_addr);

    rwnx_hw->fw_patch = 0;

    size = rwnx_request_firmware_common(rwnx_hw, &dst);
    if (size <= 0)
    {
        DBG_MACIF_INF("wrong size of firmware file\n");
        dst = NULL;
        err = -1;
    }

    DBG_MACIF_INF("\n### dst=%p, size=%d\n", dst, size);
    if (size > BLOCK_SIZE)
    {
        for (; i < (size - BLOCK_SIZE); i += BLOCK_SIZE)
        {
            DBG_MACIF_INF("wr blk 0: %p -> %x\r\n", dst + i / 4, fw_addr + i);
            err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fw_addr + i, BLOCK_SIZE, dst + i / 4);
            if (err)
            {
                DBG_MACIF_INF("bin upload fail: %x, err:%d\r\n", fw_addr + i, err);
                break;
            }
        }
    }
    if (!err && (i < size))
    {
        DBG_MACIF_INF("wr blk 1: %p -> %x\r\n", dst + i / 4, fw_addr + i);
        err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fw_addr + i, size - i, dst + i / 4);
        if (err)
        {
            DBG_MACIF_INF("bin upload fail: %x, err:%d\r\n", fw_addr + i, err);
        }
    }

    if (dst)
    {
        rwnx_release_firmware_common(&dst);
    }

    return err;
}
//#endif /* !CONFIG_ROM_PATCH_EN */

//#ifndef CONFIG_ROM_PATCH_EN
/**
 * rwnx_plat_fmac_load() - Load FW code
 *
 * @rwnx_hw: Main driver data
 */
int rwnx_plat_fmac_load(struct rwnx_hw *rwnx_hw)
{
    int ret;

    RWNX_DBG(RWNX_FN_ENTRY_STR);
    ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, RAM_FMAC_FW_ADDR);
    return ret;
}
//#endif /* !CONFIG_ROM_PATCH_EN */

u8 aic8800dc_rf_flag = 0;
int aicwf_plat_patch_load_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (rwnx_hw->mode != WIFI_MODE_RFTEST)
    {
#if !defined(CONFIG_FPGA_VERIFICATION)
        if (chip_sub_id == 0)
        {
            DBG_MACIF_INF("dcdw_u01 is loaing ###############\n");
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR_U01);
        }
        else if (chip_sub_id >= 1)
        {
            DBG_MACIF_INF("dcdw_u02/dcdw_h_u02 is loaing ###############\n");
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR);
            if (ret)
            {
                DBG_MACIF_INF("dcdw_u02/dcdw_h_u02 patch load fail: %d\n", ret);
                return ret;
            }
#ifdef CONFIG_DPD
#if 1 //ifdef CONFIG_FORCE_DPD_CALIB
            if (1)
            {
                DBG_MACIF_INF("dpd calib & write\n");
                ret = aicwf_dpd_calib_8800dc(rwnx_hw, &dpd_res);
                if (ret)
                {
                    DBG_MACIF_INF("dpd calib fail: %d\n", ret);
                    return ret;
                }
            }
#else
            if (is_file_exist(FW_DPDRESULT_NAME_8800DC) == 1)
            {
                DBG_MACIF_INF("dpd bin load\n");
                ret = aicwf_dpd_result_load_8800dc(rwnx_hw, &dpd_res);;
                if (ret)
                {
                    DBG_MACIF_INF("load dpd bin fail: %d\n", ret);
                    return ret;
                }
                ret = aicwf_dpd_result_apply_8800dc(rwnx_hw, &dpd_res);
                if (ret)
                {
                    DBG_MACIF_INF("apply dpd bin fail: %d\n", ret);
                    return ret;
                }

            }
#endif
            else
#endif
            {
                ret = aicwf_misc_ram_init_8800dc(rwnx_hw);
                if (ret)
                {
                    DBG_MACIF_INF("misc ram init fail: %d\n", ret);
                    return ret;
                }
            }
        }
        else
        {
            DBG_MACIF_INF("unsupported id: %d\n", chip_sub_id);
        }
#endif
    }
    else if (rwnx_hw->mode == WIFI_MODE_RFTEST)
    {
        rwnx_hw->mode = WIFI_MODE_UNKNOWN;
        DBG_MACIF_INF("dcdw_u02/dcdw_h_u02 is loaing ###############\n");
        ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR);
        if (ret)
        {
            DBG_MACIF_INF("dcdw_u02/dcdw_h_u02 patch load fail: %d\n", ret);
            return ret;
        }
#ifdef CONFIG_DPD
#if 1 //ifdef CONFIG_FORCE_DPD_CALIB
        if (1)
        {
            DBG_MACIF_INF("dpd calib & write\n");
            ret = aicwf_dpd_calib_8800dc(rwnx_hw, &dpd_res);
            if (ret)
            {
                DBG_MACIF_INF("dpd calib fail: %d\n", ret);
                return ret;
            }
        }
#endif
#endif
        rwnx_hw->mode = WIFI_MODE_RFTEST;
        DBG_MACIF_INF("%s load rftest bin\n", __func__);
        if (chip_sub_id == 0)
        {
            aic8800dc_rf_flag = 1;
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR);
        }
        if (!ret)
        {
            aic8800dc_rf_flag = 0;
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, RAM_LMAC_FW_ADDR);
        }
    }

    return ret;
}

static int rwnx_plat_patch_load(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

    RWNX_DBG(RWNX_FN_ENTRY_STR);
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
            rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    {
#if 0
        if (chip_sub_id == 1)
        {
            aicwf_misc_ram_init_8800dc(rwnx_hw);
        }
#endif
        DBG_MACIF_INF("rwnx_plat_patch_loading\n");
        ret = aicwf_plat_patch_load_8800dc(rwnx_hw);
    }
    return ret;
}
#endif

/**
 * rwnx_platform_reset() - Reset the platform
 *
 * @rwnx_plat: platform data
 */
static int rwnx_platform_reset(struct rwnx_plat *rwnx_plat)
{
    return 0;
}

/**
 * rwnx_platform_on() - Start the platform
 *
 * @rwnx_hw: Main driver data
 * @config: Config to restore (NULL if nothing to restore)
 *
 * It starts the platform :
 * - load fw and ucodes
 * - initialize IPC
 * - boot the fw
 * - enable link communication/IRQ
 *
 * Called by 802.11 part
 */
int rwnx_platform_on(struct rwnx_hw *rwnx_hw)
{
    //#ifndef CONFIG_ROM_PATCH_EN
#ifdef CONFIG_DOWNLOAD_FW
    int ret;
#endif
    //#endif
    struct rwnx_plat *rwnx_plat = rwnx_hw->plat;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (rwnx_plat->enabled)
        return 0;

    // cmd mgr
    rwnx_cmd_mgr_init(&rwnx_hw->cmd_mgr);

#ifdef CONFIG_DOWNLOAD_FW
    if (rwnx_hw->chipid != PRODUCT_ID_AIC8800DC && rwnx_hw->chipid != PRODUCT_ID_AIC8800DW)
    {
        ret = rwnx_plat_fmac_load(rwnx_hw);
        if (ret)
            return ret;
    }
    else
    {
        rwnx_plat_patch_load(rwnx_hw);
    }
#endif

#ifdef CONFIG_LOAD_USERCONFIG
    //rwnx_plat_userconfig_load(rwnx_hw);
#endif

    rwnx_plat->enabled = true;

    return 0;
}

/**
 * rwnx_platform_off() - Stop the platform
 *
 * @rwnx_hw: Main driver data
 * @config: Updated with pointer to config, to be able to restore it with
 * rwnx_platform_on(). It's up to the caller to free the config. Set to NULL
 * if configuration is not needed.
 *
 * Called by 802.11 part
 */
void rwnx_platform_off(struct rwnx_hw *rwnx_hw)
{
    rwnx_platform_reset(rwnx_hw->plat);
    rwnx_cmd_mgr_deinit(&rwnx_hw->cmd_mgr);
    rwnx_hw->plat->enabled = false;
}

/**
 * rwnx_platform_init() - Initialize the platform
 *
 * @rwnx_plat: platform data (already updated by platform driver)
 * @platform_data: Pointer to store the main driver data pointer (aka rwnx_hw)
 *                That will be set as driver data for the platform driver
 * Return: 0 on success, < 0 otherwise
 *
 * Called by the platform driver after it has been probed
 */
int rwnx_platform_init(struct rwnx_plat *rwnx_plat, void **platform_data)
{
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    rwnx_plat->enabled = false;
    g_rwnx_plat = rwnx_plat;

#if defined(CONFIG_RAWDATA_MODE)
    // rdata drv
    rwnx_rdata_init(rwnx_plat, platform_data);
#endif

#if defined(CONFIG_VNET_MODE)
    // vnet drv
    rwnx_vnet_init(rwnx_plat, platform_data);
#endif

#if defined CONFIG_RWNX_FULLMAC
    //return rwnx_fdrv_init(rwnx_plat, platform_data);
#endif
    return 0;
}

/**
 * rwnx_platform_deinit() - Deinitialize the platform
 *
 * @rwnx_hw: main driver data
 *
 * Called by the platform driver after it is removed
 */
void rwnx_platform_deinit(struct rwnx_hw *rwnx_hw)
{
    RWNX_DBG(RWNX_FN_ENTRY_STR);

#if defined(CONFIG_RAWDATA_MODE)
    // rdata drv
    rwnx_rdata_deinit(rwnx_hw);
#endif

#if defined(CONFIG_VNET_MODE)
    // vnet drv
    rwnx_vnet_deinit(rwnx_hw);
#endif

#if defined CONFIG_RWNX_FULLMAC
    //rwnx_fdrv_deinit(rwnx_hw);
#endif
}

#ifdef CONFIG_DPD
int aicwf_plat_calib_load_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    aic8800dc_calib_flag = 1;
    if (chip_sub_id >= 1)
    {
        ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_CALIB_ADDR);
        if (ret)
        {
            DBG_MACIF_INF("load rftest bin fail: %d\n", ret);
            return ret;
        }
    }
    aic8800dc_calib_flag = 0;
    return ret;
}

int aicwf_dpd_calib_8800dc(struct rwnx_hw *rwnx_hw, rf_misc_ram_lite_t *dpd_res)
{
    int ret = 0;
    uint32_t fw_addr, boot_type;

    DBG_MACIF_INF("%s\n", __func__);

    ret = aicwf_plat_calib_load_8800dc(rwnx_hw);
    if (ret)
    {
        DBG_MACIF_INF("load calib bin fail: %d\n", ret);
        return ret;
    }
    /* fw start */
    fw_addr = 0x00130009;
    boot_type = 4;//HOST_START_APP_FNCALL;
    DBG_MACIF_INF("Start app: %08x, %d\n", fw_addr, boot_type);
    ret = rwnx_send_dbg_start_app_req(rwnx_hw, fw_addr, boot_type);
    if (ret)
    {
        DBG_MACIF_INF("start app fail: %d\n", ret);
        return ret;
    }
    {
        // read dpd res
        const uint32_t cfg_base = 0x10164;
        struct dbg_mem_read_cfm cfm;
        uint32_t misc_ram_addr;
        uint32_t ram_base_addr, ram_word_cnt;
        int i;
        ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x14, &cfm);
        if (ret)
        {
            DBG_MACIF_INF("rf misc ram[0x%x] rd fail: %d\n", cfg_base + 0x14, ret);
            return ret;
        }
        misc_ram_addr = cfm.memdata;
        // bit_mask
        ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, bit_mask);
        ram_word_cnt = (MEMBER_SIZE(rf_misc_ram_t, bit_mask) + MEMBER_SIZE(rf_misc_ram_t, reserved)) / 4;
        for (i = 0; i < ram_word_cnt; i++)
        {
            ret = rwnx_send_dbg_mem_read_req(rwnx_hw, ram_base_addr + i * 4, &cfm);
            if (ret)
            {
                DBG_MACIF_INF("bit_mask[0x%x] rd fail: %d\n",  ram_base_addr + i * 4, ret);
                return ret;
            }
            dpd_res->bit_mask[i] = cfm.memdata;
        }
        // dpd_high
        ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, dpd_high);
        ram_word_cnt = MEMBER_SIZE(rf_misc_ram_t, dpd_high) / 4;
        for (i = 0; i < ram_word_cnt; i++)
        {
            ret = rwnx_send_dbg_mem_read_req(rwnx_hw, ram_base_addr + i * 4, &cfm);
            if (ret)
            {
                DBG_MACIF_INF("bit_mask[0x%x] rd fail: %d\n",  ram_base_addr + i * 4, ret);
                return ret;
            }
            dpd_res->dpd_high[i] = cfm.memdata;
        }
        // loft_res
        ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, loft_res);
        ram_word_cnt = MEMBER_SIZE(rf_misc_ram_t, loft_res) / 4;
        for (i = 0; i < ram_word_cnt; i++)
        {
            ret = rwnx_send_dbg_mem_read_req(rwnx_hw, ram_base_addr + i * 4, &cfm);
            if (ret)
            {
                DBG_MACIF_INF("bit_mask[0x%x] rd fail: %d\n",  ram_base_addr + i * 4, ret);
                return ret;
            }
            dpd_res->loft_res[i] = cfm.memdata;
        }
    }
    return ret;
}

int aicwf_dpd_result_apply_8800dc(struct rwnx_hw *rwnx_hw, rf_misc_ram_lite_t *dpd_res)
{
    int ret = 0;
    uint32_t cfg_base = 0x10164;
    struct dbg_mem_read_cfm cfm;
    uint32_t misc_ram_addr;
    uint32_t ram_base_addr, ram_byte_cnt;
    DBG_MACIF_INF("bit_mask[1]=%x\n", dpd_res->bit_mask[1]);
    if (dpd_res->bit_mask[1] == 0)
    {
        DBG_MACIF_INF("void dpd_res, bypass it.\n");
        return 0;
    }
    if (rwnx_hw->mode == WIFI_MODE_RFTEST)
    {
        cfg_base = RAM_LMAC_FW_ADDR + 0x0164;
    }
    if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x14, &cfm)))
    {
        DBG_MACIF_INF("rf misc ram[0x%x] rd fail: %d\n", cfg_base + 0x14, ret);
        return ret;
    }
    misc_ram_addr = cfm.memdata;
    DBG_MACIF_INF("misc_ram_addr: %x\n", misc_ram_addr);
    /* Copy dpd_res on the Embedded side */
    // bit_mask
    DBG_MACIF_INF("bit_mask[0]=%x\n", dpd_res->bit_mask[0]);
    ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, bit_mask);
    ram_byte_cnt = MEMBER_SIZE(rf_misc_ram_t, bit_mask) + MEMBER_SIZE(rf_misc_ram_t, reserved);
    ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ram_base_addr, ram_byte_cnt, (u32 *)&dpd_res->bit_mask[0]);
    if (ret)
    {
        DBG_MACIF_INF("bit_mask wr fail: %x, ret:%d\r\n", ram_base_addr, ret);
        return ret;
    }
    // dpd_high
    DBG_MACIF_INF("dpd_high[0]=%x\n", dpd_res->dpd_high[0]);
    ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, dpd_high);
    ram_byte_cnt = MEMBER_SIZE(rf_misc_ram_t, dpd_high);
    ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ram_base_addr, ram_byte_cnt, (u32 *)&dpd_res->dpd_high[0]);
    if (ret)
    {
        DBG_MACIF_INF("dpd_high wr fail: %x, ret:%d\r\n", ram_base_addr, ret);
        return ret;
    }
    // loft_res
    DBG_MACIF_INF("loft_res[0]=%x\n", dpd_res->loft_res[0]);
    ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, loft_res);
    ram_byte_cnt = MEMBER_SIZE(rf_misc_ram_t, loft_res);
    ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ram_base_addr, ram_byte_cnt, (u32 *)&dpd_res->loft_res[0]);
    if (ret)
    {
        DBG_MACIF_INF("loft_res wr fail: %x, ret:%d\r\n", ram_base_addr, ret);
        return ret;
    }
    return ret;
}

#endif


