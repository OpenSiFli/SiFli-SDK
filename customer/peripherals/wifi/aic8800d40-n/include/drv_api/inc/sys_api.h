/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SYS_DESIRE_H_
#define _SYS_DESIRE_H_

#include "compiler.h"
#include "aic_porting_types.h"
// 优先级/栈宏已迁移至 plat_port.h，由 platform 层统一维护
#include "plat_port.h"

#define CHIP_REV_U01                0x1
#define CHIP_REV_U02                0x3
#define CHIP_REV_U03                0x7
#define CHIP_SUB_REV_U04            0x20

typedef enum
{
    PRODUCT_ID_AIC8801 = 0,
    PRODUCT_ID_AIC8800DC,
    PRODUCT_ID_AIC8800DW,
    PRODUCT_ID_AIC8800D80,
    PRODUCT_ID_AIC8800D80N,
    PRODUCT_ID_AIC8800D80X2,
} AICWF_IC;

#define HOST_BUS_SLEEP              0
#define HOST_BUS_ACTIVE             1

typedef enum
{
    HOST_TYPE_DATA_FW2SOC   = 0X00,
    HOST_TYPE_DATA_SOC2FW   = 0X01,
    HOST_TYPE_CFG           = 0X10,
    HOST_TYPE_CFG_CMD_RSP   = 0X11,
    HOST_TYPE_CFG_DATA_CFM  = 0X12,
    HOST_TYPE_CFG_PRINT     = 0X13   /* FW 日志打印 (SDIO/USB 共用) */
} host_type;

void wifi_chip_status_set(void);
void wifi_chip_status_clr(void);
uint8_t wifi_chip_status_get(void);

/* Chip identification — backed by host driver globals */
uint8_t aic_get_chip_sub_id(void);
uint8_t aic_get_aic8800dc_rf_flag(void);
uint8_t aic_get_aic8800dc_calib_flag(void);
uint8_t aic_get_chip_id(void);
uint8_t aic_get_chip_id_h(void);
void aic_set_chip_fw_match(uint32_t value);

/* Runtime chipmatch — 通过 SDIO CIS 读取硬件实际 VID/PID 动态匹配 chipid,
 * 在 __aicwf_aic_drv_init() probe 回调内自动完成, 供 aicwf_get_chipid() 优先使用.
 * 未匹配返回 AICWF_CHIPID_UNMATCHED (fallback 到编译时 CONFIG 宏).
 * 注: 不能用 0 作哨兵 — PRODUCT_ID_AIC8801 枚举值即 0, 会与"未匹配"冲突. */
#define AICWF_CHIPID_UNMATCHED 0xFFFF
uint16_t aicwf_get_runtime_chipid(void);

/* Pwrctl runtime switch — SDIO/USB unified; facade of aicwf_pwrctl_enable_set/get
 * (hal_co.c). Call aic_pwrctl_enable_set(0) in platform init to disable all
 * pwrctl flows (task init/wakeup/sleep/pktcnt). Default = 1 (enabled). */
#if defined(CONFIG_SDIO_BUS_PWRCTRL) || defined(CONFIG_USB_SUPPORT)
    void aic_pwrctl_enable_set(uint8_t enable);
    uint8_t aic_pwrctl_enable_get(void);
#endif

#endif /* _SYS_DESIRE_H_ */

