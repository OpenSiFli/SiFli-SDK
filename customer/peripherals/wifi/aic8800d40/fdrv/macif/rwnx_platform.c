/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * @file rwnx_platform.c
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */

#include "reg_access.h"
#include "rwnx_main.h"
#include "rwnx_defs.h"
#include "rwnx_utils.h"
#include "rwnx_msg_tx.h"
#include "rwnx_platform.h"
#include "fmac_port.h"
#include "userconfig.h"
#include "sys_al.h"
#include "wifi_al.h"
#include "wifi_def.h"
#include "aic_plat_log.h"

//Parser state
#define INIT 0
#define CMD 1
#define PRINT 2
#define GET_VALUE 3

static u8 fw_ver_str[64];
u8 aic8800dc_rf_flag = 0;
u8 aic8800dc_calib_flag = 0;
struct rwnx_plat rwnx_plat_obj;
struct rwnx_plat *g_rwnx_plat = NULL;

void rwnx_get_chip_info(uint8* info)
{
	if(info != NULL)
	{
		info[0] = chip_id;
		info[1] = chip_sub_id;
		info[2] = chip_fw_match;
	}
}

uint8_t *rwnx_get_fw_ver(void)
{
    return fw_ver_str;
}

int aicwf_misc_ram_init_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    uint32_t cfg_base = 0x10164;
    struct dbg_mem_read_cfm cfm;
    uint32_t misc_ram_addr;
    uint32_t misc_ram_size = 12;
    int i;
	if (rwnx_hw->mode == WIFI_MODE_RFTEST) {
		cfg_base = RAM_LMAC_FW_ADDR + 0x0164;
	}
    // init misc ram
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x14, &cfm);
    if (ret) {
        AIC_LOG_PRINTF("rf misc ram[0x%x] rd fail: %d\n", cfg_base + 0x14, ret);
        return ret;
    }
    misc_ram_addr = cfm.memdata;
    AIC_LOG_PRINTF("misc_ram_addr=%x\n", misc_ram_addr);
    for (i = 0; i < (misc_ram_size / 4); i++) {
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, misc_ram_addr + i * 4, 0);
        if (ret) {
            AIC_LOG_PRINTF("rf misc ram[0x%x] wr fail: %d\n",  misc_ram_addr + i * 4, ret);
            return ret;
        }
    }
    return ret;
}

/* buffer is allocated by kzalloc */
static int rwnx_request_firmware_common(struct rwnx_hw *rwnx_hw, u32** buffer)
{
    int size = 0;
    if (rwnx_hw->fw_patch == 0)
        size = rwnx_load_firmware(rwnx_hw->chipid, rwnx_hw->mode, buffer);
    else if (rwnx_hw->fw_patch == 1)
        size = rwnx_load_patch_tbl(rwnx_hw->chipid, buffer);
    else if (rwnx_hw->fw_patch == 2)
        size = rwnx_load_calib(rwnx_hw->chipid, buffer);

    return size;
}

static void rwnx_release_firmware_common(u32** buffer)
{
    rwnx_restore_firmware(buffer);
}

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
    //AIC_LOG_PRINTF("### Upload %s \n", filename);

    size = rwnx_request_firmware_common(rwnx_hw, &dst);
    if (!dst) {
        AIC_LOG_PRINTF("No such file or directory\n");
        return -1;
    }
    if (size <= 0) {
            AIC_LOG_PRINTF("wrong size of firmware file\n");
            dst = NULL;
            err = -1;
    }

    AIC_LOG_PRINTF("tbl size = %d \n",size);

    fmacfw_patch_tbl_8800dc_u02_describe_base = dst[0];
    AIC_LOG_PRINTF("FMACFW_PATCH_TBL_8800DC_U02_DESCRIBE_BASE = %x \n",fmacfw_patch_tbl_8800dc_u02_describe_base);

    if (!err && (i < size)) {
        err=rwnx_send_dbg_mem_block_write_req(rwnx_hw, fmacfw_patch_tbl_8800dc_u02_describe_base, fmacfw_patch_tbl_8800dc_u02_describe_size + 4, dst);
        if(err){
            AIC_LOG_PRINTF("write describe information fail \n");
        }

        describle = (u8 *)rtos_malloc(fmacfw_patch_tbl_8800dc_u02_describe_size);
        memcpy(describle,&dst[1],fmacfw_patch_tbl_8800dc_u02_describe_size);
        memcpy(fw_ver_str, &dst[1], fmacfw_patch_tbl_8800dc_u02_describe_size > 63 ?
                                    63 : fmacfw_patch_tbl_8800dc_u02_describe_size);
        AIC_LOG_PRINTF("%s",describle);
        rtos_free(describle);
        describle=NULL;
    }

    if (!err && (i < size)) {
        for (i =(128/4); i < (size/4); i +=2) {
            AIC_LOG_PRINTF("patch_tbl:  %x  %x\n", dst[i], dst[i+1]);
            err = rwnx_send_dbg_mem_write_req(rwnx_hw, dst[i], dst[i+1]);
        }
        if (err) {
            AIC_LOG_PRINTF("bin upload fail: %x, err:%d\r\n", dst[i], err);
        }
    }

    if (dst) {
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
    AIC_LOG_PRINTF("\n### Upload firmware, @ = %x\n", fw_addr);

    // Read bin file
    size = rwnx_request_firmware_common(rwnx_hw, &dst);
    if (size <= 0) {
            DBG_MACIF_ERR("wrong size of firmware file\n");
            dst = NULL;
            err = -1;
    }

    // Write to fw
    AIC_LOG_PRINTF("\n### dst=%p, size=%d\n", dst, size);
    if (size > BLOCK_SIZE) {
        for (; i < (size - BLOCK_SIZE); i += BLOCK_SIZE) {
            DBG_MACIF_VRB("wr blk 0: %p -> %x\r\n", dst + i / 4, fw_addr + i);
            err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fw_addr + i, BLOCK_SIZE, dst + i / 4);
            if (err) {
                DBG_MACIF_ERR("bin upload fail: %x, err:%d\r\n", fw_addr + i, err);
                break;
            }
        }
    }
    if (!err && (i < size)) {
        DBG_MACIF_VRB("wr blk 1: %p -> %x\r\n", dst + i / 4, fw_addr + i);
        err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fw_addr + i, size - i, dst + i / 4);
        if (err) {
            DBG_MACIF_ERR("bin upload fail: %x, err:%d\r\n", fw_addr + i, err);
        }
    }

    if (dst) {
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
    u32 fw_addr;

    RWNX_DBG(RWNX_FN_ENTRY_STR);
	rwnx_hw->fw_patch = 0;
    if ((rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) &&
        chip_mcu_id && (rwnx_hw->mode != WIFI_MODE_RFTEST)) {
        fw_addr = RAM_FMAC_WD4M_FW_ADDR;
    } else {
        fw_addr = RAM_FMAC_FW_ADDR;
    }
    ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, fw_addr);
    return ret;
}
//#endif /* !CONFIG_ROM_PATCH_EN */

#ifdef CONFIG_DPD
rf_misc_ram_lite_t dpd_res = {0,};
#endif
int aicwf_plat_patch_load_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (rwnx_hw->mode != WIFI_MODE_RFTEST) {
        #if !defined(CONFIG_FPGA_VERIFICATION)
        if (chip_sub_id == 0) {
            AIC_LOG_PRINTF("dcdw_u01 is loaing ###############\n");
			rwnx_hw->fw_patch = 0;
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR_U01);
        } else if (chip_sub_id >= 1) {
            AIC_LOG_PRINTF("dcdw_u02/dcdw_h_u02 is loaing ###############\n");
			rwnx_hw->fw_patch = 0;
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR);
			if (ret) {
				AIC_LOG_PRINTF("dcdw_u02/dcdw_h_u02 patch load fail: %d\n", ret);
				return ret;
			}
            #ifdef CONFIG_DPD
            #ifdef CONFIG_FORCE_DPD_CALIB
			if (1) {
				AIC_LOG_PRINTF("dpd calib & write\n");
				ret = aicwf_dpd_calib_8800dc(rwnx_hw, &dpd_res);
				if (ret) {
					AIC_LOG_PRINTF("dpd calib fail: %d\n", ret);
					return ret;
				}
			}
            #else /* CONFIG_FORCE_DPD_CALIB */
			if (is_file_exist(FW_DPDRESULT_NAME_8800DC) == 1) {
				AIC_LOG_PRINTF("dpd bin load\n");
				ret = aicwf_dpd_result_load_8800dc(rwnx_hw, &dpd_res);;
				if (ret) {
					AIC_LOG_PRINTF("load dpd bin fail: %d\n", ret);
					return ret;
				}
				ret = aicwf_dpd_result_apply_8800dc(rwnx_hw, &dpd_res);
				if (ret) {
					AIC_LOG_PRINTF("apply dpd bin fail: %d\n", ret);
					return ret;
				}

			}
            #endif /* CONFIG_FORCE_DPD_CALIB */
			else
            #endif /* CONFIG_DPD */
			{
				ret = aicwf_misc_ram_init_8800dc(rwnx_hw);
				if (ret) {
					AIC_LOG_PRINTF("misc ram init fail: %d\n", ret);
					return ret;
				}
			}
        } else {
            AIC_LOG_PRINTF("unsupported id: %d\n", chip_sub_id);
        }
        #endif /* CONFIG_FPGA_VERIFICATION */
    } else if (rwnx_hw->mode == WIFI_MODE_RFTEST) {
        #ifdef CONFIG_LOAD_FW_FROM_FLASH
    	if((!(get_wifi_fw_type() & 0x02)) || (get_wifi_fw_type() & 0x08))
        #endif
    	{
	    	rwnx_hw->mode = WIFI_MODE_UNKNOWN;
			AIC_LOG_PRINTF("dcdw_u02/dcdw_h_u02 is loaing ###############\n");
			rwnx_hw->fw_patch = 0;
	        ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR);
			if (ret) {
				AIC_LOG_PRINTF("dcdw_u02/dcdw_h_u02 patch load fail: %d\n", ret);
				return ret;
			}

	        #ifdef CONFIG_DPD
	        #ifdef CONFIG_FORCE_DPD_CALIB
			if (1) {
				AIC_LOG_PRINTF("dpd calib & write\n");
				ret = aicwf_dpd_calib_8800dc(rwnx_hw, &dpd_res);
				if (ret) {
					AIC_LOG_PRINTF("dpd calib fail: %d\n", ret);
					return ret;
				}
			}
	        #endif /* CONFIG_FORCE_DPD_CALIB */
	        #endif /* CONFIG_DPD */
    	}
		rwnx_hw->mode = WIFI_MODE_RFTEST;
		AIC_LOG_PRINTF("%s load rftest bin\n", __func__);
        if (chip_sub_id == 0) {
            aic8800dc_rf_flag = 1;
			rwnx_hw->fw_patch = 0;
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_PATCH_ADDR);
        }
        if (!ret) {
            aic8800dc_rf_flag = 0;
			rwnx_hw->fw_patch = 0;
            ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, RAM_LMAC_FW_ADDR);
        }
    }

    return ret;
}

static int rwnx_plat_patch_load(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    RWNX_DBG(RWNX_FN_ENTRY_STR);
    if(rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW){
        #if 0
        if (chip_sub_id == 1) {
            aicwf_misc_ram_init_8800dc(rwnx_hw);
        }
        #endif

        AIC_LOG_PRINTF("rwnx_plat_patch_loading\n");
        ret = aicwf_plat_patch_load_8800dc(rwnx_hw);
    }
    return ret;
}

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
    //struct rwnx_plat *rwnx_plat = rwnx_hw->plat;
    struct rwnx_plat *rwnx_plat = &rwnx_plat_obj;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (rwnx_plat->enabled)
        return 0;

    #ifdef CONFIG_DOWNLOAD_FW
    if (rwnx_hw->chipid != PRODUCT_ID_AIC8800DC && rwnx_hw->chipid != PRODUCT_ID_AIC8800DW) {
        ret = rwnx_plat_fmac_load(rwnx_hw);

    } else {
        ret = rwnx_plat_patch_load(rwnx_hw);
    }
	if (ret)
		return ret;
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
    //rwnx_platform_reset(rwnx_hw->plat);
    //rwnx_hw->plat->enabled = false;
    struct rwnx_plat *rwnx_plat = &rwnx_plat_obj;
    rwnx_platform_reset(rwnx_plat);
    rwnx_plat->enabled = false;
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

#if defined CONFIG_RWNX_FULLMAC
    //return rwnx_fdrv_init(rwnx_plat, platform_data);
    return 0;
#endif
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

#if defined CONFIG_RWNX_FULLMAC
    //rwnx_fdrv_deinit(rwnx_hw);
#endif
}

int rwnx_send_reboot(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    u32 delay = 100; //100ms

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    ret = rwnx_send_dbg_start_app_req(rwnx_hw, delay, HOST_START_APP_REBOOT);
    return ret;
}

#ifdef CONFIG_DPD
int aicwf_plat_calib_load_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	aic8800dc_calib_flag = 1;
	rwnx_hw->fw_patch = 2;
    if (chip_sub_id >= 1) {

		rwnx_hw->fw_patch = 2;
        ret = rwnx_plat_bin_fw_upload_2(rwnx_hw, ROM_FMAC_CALIB_ADDR);
        if (ret) {
            AIC_LOG_PRINTF("load rftest bin fail: %d\n", ret);
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

	AIC_LOG_PRINTF("%s\n", __func__);

    ret = aicwf_plat_calib_load_8800dc(rwnx_hw);
    if (ret) {
        AIC_LOG_PRINTF("load calib bin fail: %d\n", ret);
        return ret;
    }
    /* fw start */
    fw_addr = 0x00130009;
    boot_type = HOST_START_APP_FNCALL;
    AIC_LOG_PRINTF("Start app: %08x, %d\n", fw_addr, boot_type);
    ret = rwnx_send_dbg_start_app_req(rwnx_hw, fw_addr, boot_type);
    if (ret) {
        AIC_LOG_PRINTF("start app fail: %d\n", ret);
        return ret;
    }
    { // read dpd res
        const uint32_t cfg_base = 0x10164;
        struct dbg_mem_read_cfm cfm;
        uint32_t misc_ram_addr;
        uint32_t ram_base_addr, ram_word_cnt;
        int i;
        ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x14, &cfm);
        if (ret) {
            AIC_LOG_PRINTF("rf misc ram[0x%x] rd fail: %d\n", cfg_base + 0x14, ret);
            return ret;
        }
        misc_ram_addr = cfm.memdata;
        // bit_mask
        ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, bit_mask);
        ram_word_cnt = (MEMBER_SIZE(rf_misc_ram_t, bit_mask) + MEMBER_SIZE(rf_misc_ram_t, reserved)) / 4;
        for (i = 0; i < ram_word_cnt; i++) {
            ret = rwnx_send_dbg_mem_read_req(rwnx_hw, ram_base_addr + i * 4, &cfm);
            if (ret) {
                AIC_LOG_PRINTF("bit_mask[0x%x] rd fail: %d\n",  ram_base_addr + i * 4, ret);
                return ret;
            }
            dpd_res->bit_mask[i] = cfm.memdata;
        }
        // dpd_high
        ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, dpd_high);
        ram_word_cnt = MEMBER_SIZE(rf_misc_ram_t, dpd_high) / 4;
        for (i = 0; i < ram_word_cnt; i++) {
            ret = rwnx_send_dbg_mem_read_req(rwnx_hw, ram_base_addr + i * 4, &cfm);
            if (ret) {
                AIC_LOG_PRINTF("bit_mask[0x%x] rd fail: %d\n",  ram_base_addr + i * 4, ret);
                return ret;
            }
            dpd_res->dpd_high[i] = cfm.memdata;
        }
        // loft_res
        ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, loft_res);
        ram_word_cnt = MEMBER_SIZE(rf_misc_ram_t, loft_res) / 4;
        for (i = 0; i < ram_word_cnt; i++) {
            ret = rwnx_send_dbg_mem_read_req(rwnx_hw, ram_base_addr + i * 4, &cfm);
            if (ret) {
                AIC_LOG_PRINTF("bit_mask[0x%x] rd fail: %d\n",  ram_base_addr + i * 4, ret);
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
    AIC_LOG_PRINTF("bit_mask[1]=%x\n", dpd_res->bit_mask[1]);
    if (dpd_res->bit_mask[1] == 0) {
        AIC_LOG_PRINTF("void dpd_res, bypass it.\n");
        return 0;
    }
    if (rwnx_hw->mode == WIFI_MODE_RFTEST) {
        cfg_base = RAM_LMAC_FW_ADDR + 0x0164;
    }
    if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, cfg_base + 0x14, &cfm))) {
        AIC_LOG_PRINTF("rf misc ram[0x%x] rd fail: %d\n", cfg_base + 0x14, ret);
        return ret;
    }
    misc_ram_addr = cfm.memdata;
    AIC_LOG_PRINTF("misc_ram_addr: %x\n", misc_ram_addr);
    /* Copy dpd_res on the Embedded side */
    // bit_mask
    AIC_LOG_PRINTF("bit_mask[0]=%x\n", dpd_res->bit_mask[0]);
    ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, bit_mask);
    ram_byte_cnt = MEMBER_SIZE(rf_misc_ram_t, bit_mask) + MEMBER_SIZE(rf_misc_ram_t, reserved);
    ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ram_base_addr, ram_byte_cnt, (u32 *)&dpd_res->bit_mask[0]);
    if (ret) {
        AIC_LOG_PRINTF("bit_mask wr fail: %x, ret:%d\r\n", ram_base_addr, ret);
        return ret;
    }
    // dpd_high
    AIC_LOG_PRINTF("dpd_high[0]=%x\n", dpd_res->dpd_high[0]);
    ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, dpd_high);
    ram_byte_cnt = MEMBER_SIZE(rf_misc_ram_t, dpd_high);
    ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ram_base_addr, ram_byte_cnt, (u32 *)&dpd_res->dpd_high[0]);
    if (ret) {
        AIC_LOG_PRINTF("dpd_high wr fail: %x, ret:%d\r\n", ram_base_addr, ret);
        return ret;
    }
    // loft_res
    AIC_LOG_PRINTF("loft_res[0]=%x\n", dpd_res->loft_res[0]);
    ram_base_addr = misc_ram_addr + offsetof(rf_misc_ram_t, loft_res);
    ram_byte_cnt = MEMBER_SIZE(rf_misc_ram_t, loft_res);
    ret = rwnx_send_dbg_mem_block_write_req(rwnx_hw, ram_base_addr, ram_byte_cnt, (u32 *)&dpd_res->loft_res[0]);
    if (ret) {
        AIC_LOG_PRINTF("loft_res wr fail: %x, ret:%d\r\n", ram_base_addr, ret);
        return ret;
    }
    return ret;
}

uint32_t aicwf_dpd_get_result(void)
{
    return dpd_res.bit_mask[1];
}
#endif

//8800D userconfig
typedef struct
{
    txpwr_idx_conf_t txpwr_idx;
    txpwr_ofst_conf_t txpwr_ofst;
    xtal_cap_conf_t xtal_cap;
} nvram_info_t;

//8800DCDW userconfig
typedef struct
{
    txpwr_lvl_conf_t txpwr_lvl;
    txpwr_lvl_conf_v2_t txpwr_lvl_v2;
    txpwr_lvl_conf_v3_t txpwr_lvl_v3;
    txpwr_ofst_conf_t txpwr_ofst;
    txpwr_ofst2x_conf_t txpwr_ofst2x;
    xtal_cap_conf_t xtal_cap;
} userconfig_info_t;

nvram_info_t nvram_info = {NULL,};
userconfig_info_t userconfig_info  = {NULL,};

void aicwf_userconfig_init(void)
{
    nvram_info.txpwr_idx.enable = aic_nvram_info.txpwr_idx.enable;
    nvram_info.txpwr_idx.dsss   = aic_nvram_info.txpwr_idx.dsss;
    nvram_info.txpwr_idx.ofdmlowrate_2g4 = aic_nvram_info.txpwr_idx.ofdmlowrate_2g4;
    nvram_info.txpwr_idx.ofdm64qam_2g4   = aic_nvram_info.txpwr_idx.ofdm64qam_2g4;
    nvram_info.txpwr_idx.ofdm256qam_2g4  = aic_nvram_info.txpwr_idx.ofdm256qam_2g4;
    nvram_info.txpwr_idx.ofdm1024qam_2g4 = aic_nvram_info.txpwr_idx.ofdm1024qam_2g4;
    nvram_info.txpwr_idx.ofdmlowrate_5g  = aic_nvram_info.txpwr_idx.ofdmlowrate_5g;
    nvram_info.txpwr_idx.ofdm64qam_5g    = aic_nvram_info.txpwr_idx.ofdm64qam_5g;
    nvram_info.txpwr_idx.ofdm256qam_5g   = aic_nvram_info.txpwr_idx.ofdm256qam_5g;
    nvram_info.txpwr_idx.ofdm1024qam_5g  = aic_nvram_info.txpwr_idx.ofdm1024qam_5g;

    nvram_info.txpwr_ofst.enable       = aic_nvram_info.txpwr_ofst.enable;
    nvram_info.txpwr_ofst.chan_1_4     = aic_nvram_info.txpwr_ofst.chan_1_4;
    nvram_info.txpwr_ofst.chan_5_9     = aic_nvram_info.txpwr_ofst.chan_5_9;
    nvram_info.txpwr_ofst.chan_10_13   = aic_nvram_info.txpwr_ofst.chan_10_13;
    nvram_info.txpwr_ofst.chan_36_64   = aic_nvram_info.txpwr_ofst.chan_36_64;
    nvram_info.txpwr_ofst.chan_100_120 = aic_nvram_info.txpwr_ofst.chan_100_120;
    nvram_info.txpwr_ofst.chan_122_140 = aic_nvram_info.txpwr_ofst.chan_122_140;
    nvram_info.txpwr_ofst.chan_142_165 = aic_nvram_info.txpwr_ofst.chan_142_165;

    nvram_info.xtal_cap.enable        = aic_nvram_info.xtal_cap.enable;
    nvram_info.xtal_cap.xtal_cap      = aic_nvram_info.xtal_cap.xtal_cap;
    nvram_info.xtal_cap.xtal_cap_fine = aic_nvram_info.xtal_cap.xtal_cap_fine;

    userconfig_info.txpwr_lvl.enable = aic_userconfig_info.txpwr_lvl.enable;
    userconfig_info.txpwr_lvl.dsss   = aic_userconfig_info.txpwr_lvl.dsss;
    userconfig_info.txpwr_lvl.ofdmlowrate_2g4 = aic_userconfig_info.txpwr_lvl.ofdm1024qam_2g4;
    userconfig_info.txpwr_lvl.ofdm64qam_2g4   = aic_userconfig_info.txpwr_lvl.ofdm64qam_2g4;
    userconfig_info.txpwr_lvl.ofdm256qam_2g4  = aic_userconfig_info.txpwr_lvl.ofdm256qam_2g4;
    userconfig_info.txpwr_lvl.ofdm1024qam_2g4 = aic_userconfig_info.txpwr_lvl.ofdm1024qam_2g4;
    userconfig_info.txpwr_lvl.ofdmlowrate_5g  = aic_userconfig_info.txpwr_lvl.ofdmlowrate_5g;
    userconfig_info.txpwr_lvl.ofdm64qam_5g    = aic_userconfig_info.txpwr_lvl.ofdm64qam_5g;
    userconfig_info.txpwr_lvl.ofdm256qam_5g   = aic_userconfig_info.txpwr_lvl.ofdm256qam_5g;
    userconfig_info.txpwr_lvl.ofdm1024qam_5g  = aic_userconfig_info.txpwr_lvl.ofdm1024qam_5g;

    userconfig_info.txpwr_lvl_v2.enable = aic_userconfig_info.txpwr_lvl_v2.enable;
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[0]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[0];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[1]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[1];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[2]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[2];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[3]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[3];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[4]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[4];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[5]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[5];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[6]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[6];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[7]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[7];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[8]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[8];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[9]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[9];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[10] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[10];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[11] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11b_11ag_2g4[11];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[0] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[0];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[1] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[1];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[2] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[2];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[3] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[3];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[4] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[4];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[5] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[5];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[6] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[6];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[7] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[7];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[8] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[8];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[9] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11n_11ac_2g4[9];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[0]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[0];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[1]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[1];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[2]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[2];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[3]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[3];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[4]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[4];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[5]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[5];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[6]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[6];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[7]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[7];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[8]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[8];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[9]  = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[9];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[10] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[10];
    userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[11] = aic_userconfig_info.txpwr_lvl_v2.pwrlvl_11ax_2g4[11];

    userconfig_info.txpwr_lvl_v3.enable = aic_userconfig_info.txpwr_lvl_v3.enable;
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[0]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[0];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[1]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[1];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[2]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[2];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[3]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[3];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[4]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[4];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[5]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[5];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[6]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[6];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[7]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[7];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[8]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[8];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[9]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[9];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[10] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[10];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[11] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11b_11ag_2g4[11];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[0] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[0];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[1] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[1];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[2] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[2];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[3] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[3];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[4] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[4];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[5] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[5];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[6] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[6];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[7] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[7];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[8] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_2g4[8];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[0]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[0];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[1]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[1];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[2]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[2];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[3]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[3];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[4]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[4];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[5]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[5];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[6]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[6];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[7]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[7];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[8]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[8];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[9]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[9];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[10] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[10];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[11] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_2g4[11];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[0]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[0];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[1]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[1];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[2]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[2];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[3]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[3];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[4]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[4];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[5]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[5];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[6]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[6];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[7]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[7];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[8]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[8];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[9]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[9];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[10] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[10];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[11] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11a_5g[11];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[0] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[0];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[1] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[1];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[2] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[2];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[3] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[3];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[4] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[4];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[5] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[5];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[6] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[6];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[7] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[7];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[8] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[8];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[9] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11n_11ac_5g[9];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[0]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[0];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[1]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[1];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[2]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[2];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[3]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[3];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[4]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[4];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[5]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[5];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[6]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[6];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[7]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[7];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[8]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[8];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[9]  = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[9];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[10] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[10];
    userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[11] = aic_userconfig_info.txpwr_lvl_v3.pwrlvl_11ax_5g[11];

    userconfig_info.txpwr_ofst.enable = aic_userconfig_info.txpwr_ofst.enable;
    userconfig_info.txpwr_ofst.chan_1_4 = aic_userconfig_info.txpwr_ofst.chan_1_4;
    userconfig_info.txpwr_ofst.chan_5_9 = aic_userconfig_info.txpwr_ofst.chan_5_9;
    userconfig_info.txpwr_ofst.chan_10_13 = aic_userconfig_info.txpwr_ofst.chan_10_13;
    userconfig_info.txpwr_ofst.chan_36_64 = aic_userconfig_info.txpwr_ofst.chan_36_64;
    userconfig_info.txpwr_ofst.chan_100_120 = aic_userconfig_info.txpwr_ofst.chan_100_120;
    userconfig_info.txpwr_ofst.chan_122_140 = aic_userconfig_info.txpwr_ofst.chan_122_140;
    userconfig_info.txpwr_ofst.chan_142_165 = aic_userconfig_info.txpwr_ofst.chan_142_165;

    userconfig_info.txpwr_ofst2x.enable = aic_userconfig_info.txpwr_ofst2x.enable;
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[0][0] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[0][0];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[0][1] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[0][1];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[0][2] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[0][2];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[1][0] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[1][0];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[1][1] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[1][1];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[1][2] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[1][2];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[2][0] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[2][0];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[2][1] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[2][1];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[2][2] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_2g4[2][2];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][0] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][0];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][1] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][1];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][2] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][2];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][3] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][3];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][4] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][4];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][5] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[0][5];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][0] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][0];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][1] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][1];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][2] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][2];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][3] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][3];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][4] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][4];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][5] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[1][5];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][0] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][0];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][1] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][1];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][2] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][2];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][3] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][3];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][4] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][4];
    userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][5] = aic_userconfig_info.txpwr_ofst2x.pwrofst2x_tbl_5g[2][5];

    userconfig_info.xtal_cap.enable        = aic_userconfig_info.xtal_cap.enable;
    userconfig_info.xtal_cap.xtal_cap      = aic_userconfig_info.xtal_cap.xtal_cap;
    userconfig_info.xtal_cap.xtal_cap_fine = aic_userconfig_info.xtal_cap.xtal_cap_fine;
}


#ifdef CONFIG_LOAD_USERCONFIG
static int rwnx_atoi(char *value)
{
    int len = 0;
    int i = 0;
    int result = 0;
    int flag = 1;

    if (value[0] == '-') {
        flag = -1;
        value++;
    }
    len = strlen(value);

    for (i = 0;i < len ;i++) {
        result = result * 10;
        if (value[i] >= 48 && value[i] <= 57) {
            result += value[i] - 48;
        } else {
            result = 0;
            break;
        }
    }

    return result * flag;
}

void get_nvram_txpwr_idx(txpwr_idx_conf_t *txpwr_idx)
{
    txpwr_idx->enable           = nvram_info.txpwr_idx.enable;
    txpwr_idx->dsss             = nvram_info.txpwr_idx.dsss;
    txpwr_idx->ofdmlowrate_2g4  = nvram_info.txpwr_idx.ofdmlowrate_2g4;
    txpwr_idx->ofdm64qam_2g4    = nvram_info.txpwr_idx.ofdm64qam_2g4;
    txpwr_idx->ofdm256qam_2g4   = nvram_info.txpwr_idx.ofdm256qam_2g4;
    txpwr_idx->ofdm1024qam_2g4  = nvram_info.txpwr_idx.ofdm1024qam_2g4;
    txpwr_idx->ofdmlowrate_5g   = nvram_info.txpwr_idx.ofdmlowrate_5g;
    txpwr_idx->ofdm64qam_5g     = nvram_info.txpwr_idx.ofdm64qam_5g;
    txpwr_idx->ofdm256qam_5g    = nvram_info.txpwr_idx.ofdm256qam_5g;
    txpwr_idx->ofdm1024qam_5g   = nvram_info.txpwr_idx.ofdm1024qam_5g;

    AIC_LOG_PRINTF("%s:enable:%d\r\n",          __func__, txpwr_idx->enable);
    AIC_LOG_PRINTF("%s:dsss:%d\r\n",            __func__, txpwr_idx->dsss);
    AIC_LOG_PRINTF("%s:ofdmlowrate_2g4:%d\r\n", __func__, txpwr_idx->ofdmlowrate_2g4);
    AIC_LOG_PRINTF("%s:ofdm64qam_2g4:%d\r\n",   __func__, txpwr_idx->ofdm64qam_2g4);
    AIC_LOG_PRINTF("%s:ofdm256qam_2g4:%d\r\n",  __func__, txpwr_idx->ofdm256qam_2g4);
    AIC_LOG_PRINTF("%s:ofdm1024qam_2g4:%d\r\n", __func__, txpwr_idx->ofdm1024qam_2g4);
    AIC_LOG_PRINTF("%s:ofdmlowrate_5g:%d\r\n",  __func__, txpwr_idx->ofdmlowrate_5g);
    AIC_LOG_PRINTF("%s:ofdm64qam_5g:%d\r\n",    __func__, txpwr_idx->ofdm64qam_5g);
    AIC_LOG_PRINTF("%s:ofdm256qam_5g:%d\r\n",   __func__, txpwr_idx->ofdm256qam_5g);
    AIC_LOG_PRINTF("%s:ofdm1024qam_5g:%d\r\n",  __func__, txpwr_idx->ofdm1024qam_5g);
}

void get_nvram_txpwr_ofst(txpwr_ofst_conf_t *txpwr_ofst)
{
    txpwr_ofst->enable       = nvram_info.txpwr_ofst.enable;
    txpwr_ofst->chan_1_4     = nvram_info.txpwr_ofst.chan_1_4;
    txpwr_ofst->chan_5_9     = nvram_info.txpwr_ofst.chan_5_9;
    txpwr_ofst->chan_10_13   = nvram_info.txpwr_ofst.chan_10_13;
    txpwr_ofst->chan_36_64   = nvram_info.txpwr_ofst.chan_36_64;
    txpwr_ofst->chan_100_120 = nvram_info.txpwr_ofst.chan_100_120;
    txpwr_ofst->chan_122_140 = nvram_info.txpwr_ofst.chan_122_140;
    txpwr_ofst->chan_142_165 = nvram_info.txpwr_ofst.chan_142_165;

    AIC_LOG_PRINTF("%s:enable      :%d\r\n", __func__, txpwr_ofst->enable);
    AIC_LOG_PRINTF("%s:chan_1_4    :%d\r\n", __func__, txpwr_ofst->chan_1_4);
    AIC_LOG_PRINTF("%s:chan_5_9    :%d\r\n", __func__, txpwr_ofst->chan_5_9);
    AIC_LOG_PRINTF("%s:chan_10_13  :%d\r\n", __func__, txpwr_ofst->chan_10_13);
    AIC_LOG_PRINTF("%s:chan_36_64  :%d\r\n", __func__, txpwr_ofst->chan_36_64);
    AIC_LOG_PRINTF("%s:chan_100_120:%d\r\n", __func__, txpwr_ofst->chan_100_120);
    AIC_LOG_PRINTF("%s:chan_122_140:%d\r\n", __func__, txpwr_ofst->chan_122_140);
    AIC_LOG_PRINTF("%s:chan_142_165:%d\r\n", __func__, txpwr_ofst->chan_142_165);
}

void get_nvram_xtal_cap(xtal_cap_conf_t *xtal_cap)
{
    *xtal_cap = nvram_info.xtal_cap;

    AIC_LOG_PRINTF("%s:enable       :%d\r\n", __func__, xtal_cap->enable);
    AIC_LOG_PRINTF("%s:xtal_cap     :%d\r\n", __func__, xtal_cap->xtal_cap);
    AIC_LOG_PRINTF("%s:xtal_cap_fine:%d\r\n", __func__, xtal_cap->xtal_cap_fine);
}

void rwnx_plat_nvram_set_value(char *command, char *value)
{
    //TODO send command
    AIC_LOG_PRINTF("%s:command=%s value=%s\n", __func__, command, value);
    if (!strcmp(command, "enable")) {
        nvram_info.txpwr_idx.enable = rwnx_atoi(value);
    } else if (!strcmp(command, "dsss")) {
        nvram_info.txpwr_idx.dsss = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdmlowrate_2g4")) {
        nvram_info.txpwr_idx.ofdmlowrate_2g4 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdm64qam_2g4")) {
        nvram_info.txpwr_idx.ofdm64qam_2g4 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdm256qam_2g4")) {
        nvram_info.txpwr_idx.ofdm256qam_2g4 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdm1024qam_2g4")) {
        nvram_info.txpwr_idx.ofdm1024qam_2g4 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdmlowrate_5g")) {
        nvram_info.txpwr_idx.ofdmlowrate_5g = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdm64qam_5g")) {
        nvram_info.txpwr_idx.ofdm64qam_5g = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdm256qam_5g")) {
        nvram_info.txpwr_idx.ofdm256qam_5g = rwnx_atoi(value);
    } else if (!strcmp(command, "ofdm1024qam_5g")) {
        nvram_info.txpwr_idx.ofdm1024qam_5g = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_enable")) {
        nvram_info.txpwr_ofst.enable = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_1_4")) {
        nvram_info.txpwr_ofst.chan_1_4 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_5_9")) {
        nvram_info.txpwr_ofst.chan_5_9 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_10_13")) {
        nvram_info.txpwr_ofst.chan_10_13 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_36_64")) {
        nvram_info.txpwr_ofst.chan_36_64 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_100_120")) {
        nvram_info.txpwr_ofst.chan_100_120 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_122_140")) {
        nvram_info.txpwr_ofst.chan_122_140 = rwnx_atoi(value);
    } else if (!strcmp(command, "ofst_chan_142_165")) {
        nvram_info.txpwr_ofst.chan_142_165 = rwnx_atoi(value);
    } else if (!strcmp(command, "xtal_enable")) {
        nvram_info.xtal_cap.enable = rwnx_atoi(value);
    } else if (!strcmp(command, "xtal_cap")) {
        nvram_info.xtal_cap.xtal_cap = rwnx_atoi(value);
    } else if (!strcmp(command, "xtal_cap_fine")) {
        nvram_info.xtal_cap.xtal_cap_fine = rwnx_atoi(value);
    } else {
        AIC_LOG_PRINTF("invalid cmd: %s\n", command);
    }
}

void rwnx_plat_userconfig_parsing(char *buffer, int size)
{
    int i = 0;
    int parse_state = 0;
    char command[30];
    char value[100];
    int char_counter = 0;

    memset(command, 0, 30);
    memset(value, 0, 100);

    for (i = 0; i < size; i++) {
        //Send command or print nvram log when char is \r or \n
        if (buffer[i] == 0x0a || buffer[i] == 0x0d) {
            if (command[0] != 0 && value[0] != 0) {
                if (parse_state == PRINT) {
                    AIC_LOG_PRINTF("%s:%s\r\n", __func__, value);
                } else if (parse_state == GET_VALUE) {
                    rwnx_plat_nvram_set_value(command, value);
                }
            }
            //Reset command value and char_counter
            memset(command, 0, 30);
            memset(value, 0, 100);
            char_counter = 0;
            parse_state = INIT;
            continue;
        }

        //Switch parser state
        if (parse_state == INIT) {
            if (buffer[i] == '#') {
                parse_state = PRINT;
                continue;
            } else if (buffer[i] == 0x0a || buffer[i] == 0x0d) {
                parse_state = INIT;
                continue;
            } else {
                parse_state = CMD;
            }
        }

        //Fill data to command and value
        if (parse_state == PRINT) {
            command[0] = 0x01;
            value[char_counter] = buffer[i];
            char_counter++;
        } else if (parse_state == CMD) {
            if (command[0] != 0 && buffer[i] == '=') {
                parse_state = GET_VALUE;
                char_counter = 0;
                continue;
            }
            command[char_counter] = buffer[i];
            char_counter++;
        } else if (parse_state == GET_VALUE) {
            value[char_counter] = buffer[i];
            char_counter++;
        }
    }
}


void get_userconfig_txpwr_lvl(txpwr_lvl_conf_t *txpwr_lvl)
{
    txpwr_lvl->enable           = userconfig_info.txpwr_lvl.enable;
    txpwr_lvl->dsss             = userconfig_info.txpwr_lvl.dsss;
    txpwr_lvl->ofdmlowrate_2g4  = userconfig_info.txpwr_lvl.ofdmlowrate_2g4;
    txpwr_lvl->ofdm64qam_2g4    = userconfig_info.txpwr_lvl.ofdm64qam_2g4;
    txpwr_lvl->ofdm256qam_2g4   = userconfig_info.txpwr_lvl.ofdm256qam_2g4;
    txpwr_lvl->ofdm1024qam_2g4  = userconfig_info.txpwr_lvl.ofdm1024qam_2g4;
    txpwr_lvl->ofdmlowrate_5g   = userconfig_info.txpwr_lvl.ofdmlowrate_5g;
    txpwr_lvl->ofdm64qam_5g     = userconfig_info.txpwr_lvl.ofdm64qam_5g;
    txpwr_lvl->ofdm256qam_5g    = userconfig_info.txpwr_lvl.ofdm256qam_5g;
    txpwr_lvl->ofdm1024qam_5g   = userconfig_info.txpwr_lvl.ofdm1024qam_5g;

    AIC_LOG_PRINTF("%s:enable:%d\r\n",          __func__, txpwr_lvl->enable);
    AIC_LOG_PRINTF("%s:dsss:%d\r\n",            __func__, txpwr_lvl->dsss);
    AIC_LOG_PRINTF("%s:ofdmlowrate_2g4:%d\r\n", __func__, txpwr_lvl->ofdmlowrate_2g4);
    AIC_LOG_PRINTF("%s:ofdm64qam_2g4:%d\r\n",   __func__, txpwr_lvl->ofdm64qam_2g4);
    AIC_LOG_PRINTF("%s:ofdm256qam_2g4:%d\r\n",  __func__, txpwr_lvl->ofdm256qam_2g4);
    AIC_LOG_PRINTF("%s:ofdm1024qam_2g4:%d\r\n", __func__, txpwr_lvl->ofdm1024qam_2g4);
    AIC_LOG_PRINTF("%s:ofdmlowrate_5g:%d\r\n",  __func__, txpwr_lvl->ofdmlowrate_5g);
    AIC_LOG_PRINTF("%s:ofdm64qam_5g:%d\r\n",    __func__, txpwr_lvl->ofdm64qam_5g);
    AIC_LOG_PRINTF("%s:ofdm256qam_5g:%d\r\n",   __func__, txpwr_lvl->ofdm256qam_5g);
    AIC_LOG_PRINTF("%s:ofdm1024qam_5g:%d\r\n",  __func__, txpwr_lvl->ofdm1024qam_5g);
}

void get_userconfig_txpwr_lvl_v2(txpwr_lvl_conf_v2_t *txpwr_lvl_v2)
{
    *txpwr_lvl_v2 = userconfig_info.txpwr_lvl_v2;

    AIC_LOG_PRINTF("%s:enable:%d\r\n",               __func__, txpwr_lvl_v2->enable);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_1m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[0]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_2m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[1]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_5m5_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[2]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_11m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[3]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_6m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[4]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_9m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[5]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_12m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[6]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_18m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[7]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_24m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[8]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_36m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[9]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_48m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[10]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_54m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[11]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs0_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[0]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs1_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[1]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs2_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[2]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs3_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[3]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs4_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[4]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs5_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[5]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs6_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[6]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs7_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[7]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs8_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[8]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs9_2g4:%d\r\n",__func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[9]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs0_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[0]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs1_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[1]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs2_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[2]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs3_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[3]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs4_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[4]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs5_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[5]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs6_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[6]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs7_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[7]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs8_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[8]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs9_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[9]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs10_2g4:%d\r\n",   __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[10]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs11_2g4:%d\r\n",   __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[11]);
}

void get_userconfig_txpwr_lvl_v3(txpwr_lvl_conf_v3_t *txpwr_lvl_v3)
{
    *txpwr_lvl_v3 = userconfig_info.txpwr_lvl_v3;

    AIC_LOG_PRINTF("%s:enable:%d\r\n",               __func__, txpwr_lvl_v3->enable);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_1m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[0]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_2m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[1]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_5m5_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[2]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_11m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[3]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_6m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[4]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_9m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[5]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_12m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[6]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_18m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[7]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_24m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[8]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_36m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[9]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_48m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[10]);
    AIC_LOG_PRINTF("%s:lvl_11b_11ag_54m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[11]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs0_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[0]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs1_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[1]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs2_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[2]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs3_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[3]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs4_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[4]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs5_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[5]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs6_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[6]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs7_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[7]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs8_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[8]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs9_2g4:%d\r\n",__func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[9]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs0_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[0]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs1_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[1]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs2_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[2]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs3_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[3]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs4_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[4]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs5_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[5]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs6_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[6]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs7_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[7]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs8_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[8]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs9_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[9]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs10_2g4:%d\r\n",   __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[10]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs11_2g4:%d\r\n",   __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[11]);

    AIC_LOG_PRINTF("%s:lvl_11a_1m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[0]);
    AIC_LOG_PRINTF("%s:lvl_11a_2m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[1]);
    AIC_LOG_PRINTF("%s:lvl_11a_5m5_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[2]);
    AIC_LOG_PRINTF("%s:lvl_11a_11m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[3]);
    AIC_LOG_PRINTF("%s:lvl_11a_6m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[4]);
    AIC_LOG_PRINTF("%s:lvl_11a_9m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[5]);
    AIC_LOG_PRINTF("%s:lvl_11a_12m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[6]);
    AIC_LOG_PRINTF("%s:lvl_11a_18m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[7]);
    AIC_LOG_PRINTF("%s:lvl_11a_24m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[8]);
    AIC_LOG_PRINTF("%s:lvl_11a_36m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[9]);
    AIC_LOG_PRINTF("%s:lvl_11a_48m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[10]);
    AIC_LOG_PRINTF("%s:lvl_11a_54m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[11]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs0_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[0]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs1_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[1]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs2_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[2]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs3_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[3]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs4_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[4]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs5_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[5]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs6_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[6]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs7_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[7]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs8_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[8]);
    AIC_LOG_PRINTF("%s:lvl_11n_11ac_mcs9_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[9]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs0_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[0]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs1_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[1]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs2_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[2]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs3_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[3]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs4_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[4]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs5_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[5]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs6_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[6]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs7_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[7]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs8_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[8]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs9_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[9]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs10_5g:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[10]);
    AIC_LOG_PRINTF("%s:lvl_11ax_mcs11_5g:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[11]);
}

void get_userconfig_txpwr_ofst(txpwr_ofst_conf_t *txpwr_ofst)
{
    txpwr_ofst->enable       = userconfig_info.txpwr_ofst.enable;
    txpwr_ofst->chan_1_4     = userconfig_info.txpwr_ofst.chan_1_4;
    txpwr_ofst->chan_5_9     = userconfig_info.txpwr_ofst.chan_5_9;
    txpwr_ofst->chan_10_13   = userconfig_info.txpwr_ofst.chan_10_13;
    txpwr_ofst->chan_36_64   = userconfig_info.txpwr_ofst.chan_36_64;
    txpwr_ofst->chan_100_120 = userconfig_info.txpwr_ofst.chan_100_120;
    txpwr_ofst->chan_122_140 = userconfig_info.txpwr_ofst.chan_122_140;
    txpwr_ofst->chan_142_165 = userconfig_info.txpwr_ofst.chan_142_165;

    AIC_LOG_PRINTF("%s:enable      :%d\r\n", __func__, txpwr_ofst->enable);
    AIC_LOG_PRINTF("%s:chan_1_4    :%d\r\n", __func__, txpwr_ofst->chan_1_4);
    AIC_LOG_PRINTF("%s:chan_5_9    :%d\r\n", __func__, txpwr_ofst->chan_5_9);
    AIC_LOG_PRINTF("%s:chan_10_13  :%d\r\n", __func__, txpwr_ofst->chan_10_13);
    AIC_LOG_PRINTF("%s:chan_36_64  :%d\r\n", __func__, txpwr_ofst->chan_36_64);
    AIC_LOG_PRINTF("%s:chan_100_120:%d\r\n", __func__, txpwr_ofst->chan_100_120);
    AIC_LOG_PRINTF("%s:chan_122_140:%d\r\n", __func__, txpwr_ofst->chan_122_140);
    AIC_LOG_PRINTF("%s:chan_142_165:%d\r\n", __func__, txpwr_ofst->chan_142_165);
}

void get_userconfig_txpwr_ofst2x(txpwr_ofst2x_conf_t *txpwr_ofst2x)
{
    int type, ch_grp;
    *txpwr_ofst2x = userconfig_info.txpwr_ofst2x;
#if 0
    AIC_LOG_PRINTF("%s:enable      :%d\r\n", __func__, txpwr_ofst2x->enable);
    AIC_LOG_PRINTF("pwrofst2x 2.4g: [0]:11b, [1]:ofdm_highrate, [2]:ofdm_lowrate\n"
        "  chan=" "\t1-4" "\t5-9" "\t10-13");
    for (type = 0; type < 3; type++) {
        AIC_LOG_PRINTF("\n  [%d] =", type);
        for (ch_grp = 0; ch_grp < 3; ch_grp++) {
            AIC_LOG_PRINTF("\t%d", txpwr_ofst2x->pwrofst2x_tbl_2g4[type][ch_grp]);
        }
    }
    AIC_LOG_PRINTF("\npwrofst2x 5g: [0]:ofdm_lowrate, [1]:ofdm_highrate, [2]:ofdm_midrate\n"
        "  chan=" "\t36-50" "\t51-64" "\t98-114" "\t115-130" "\t131-146" "\t147-166");
    for (type = 0; type < 3; type++) {
        AIC_LOG_PRINTF("\n  [%d] =", type);
        for (ch_grp = 0; ch_grp < 6; ch_grp++) {
            AIC_LOG_PRINTF("\t%d", txpwr_ofst2x->pwrofst2x_tbl_5g[type][ch_grp]);
        }
    }
    AIC_LOG_PRINTF("\n");
#endif
}

void get_userconfig_xtal_cap(xtal_cap_conf_t *xtal_cap)
{
    *xtal_cap = userconfig_info.xtal_cap;

    AIC_LOG_PRINTF("%s:enable       :%d\r\n", __func__, xtal_cap->enable);
    AIC_LOG_PRINTF("%s:xtal_cap     :%d\r\n", __func__, xtal_cap->xtal_cap);
    AIC_LOG_PRINTF("%s:xtal_cap_fine:%d\r\n", __func__, xtal_cap->xtal_cap_fine);
}
#endif

