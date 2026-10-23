/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Chip description port interface - 单库统一 (one host lib for all chip families).
 *
 * host 侧不再使用 CONFIG_AIC88xx 型号宏: 全部型号差异收敛到本描述表,
 * 由 platform 提供各型号静态数据 (platform/fmac/aic_chip_desc_port.c),
 * host 运行时按 chipid 查表分发。
 *
 * 先例: aic_fw_port.h (platform 提供固件数据, host 按 fw_id 取用)。
 */

#ifndef _AIC_CHIP_DESC_H_
#define _AIC_CHIP_DESC_H_

#include "aic_porting_types.h"

/* Chip product id — 与 host drv_api/inc/sys_api.h AICWF_IC 枚举值一致。
 * port 头不依赖驱动内部头, 此处独立定义。 */
#define AIC_CHIP_ID_8801        0
#define AIC_CHIP_ID_8800DC      1
#define AIC_CHIP_ID_8800DW      2
#define AIC_CHIP_ID_8800D80     3
#define AIC_CHIP_ID_8800D80N    4
#define AIC_CHIP_ID_8800D80X2   5
#define AIC_CHIP_ID_UNMATCHED   0xFFFF

struct aic_chip_desc
{
    /* 对应的芯片 product id (AIC_CHIP_ID_xxx) */
    uint16_t chipid;

    /* ---- SDIO/USB 总线协议行为 ----
     * (原 CONFIG 宏: sdio_co.c / usb_co.c / hal_co.c) */

    /* TX header[3] = CRC8(ponl-107); 0 时填 0 (reserved)。
     * D80 系: 1; 8801/DC/DW: 0。hal msg push 与 sdio data header 共用。 */
    uint8_t tx_hdr_crc8;

    /* SDIO flowctrl 寄存器读取后需 & SDIOWIFI_FLOWCTRL_MASK_REG。
     * D80 系: 0; 8801/DC/DW: 1。 */
    uint8_t sdio_fc_reg_mask;

    /* func_flag_tx 初值 (D80 系 false, 其余 true)。
     * func_flag_rx 恒为 true, 不进表。 */
    uint8_t func_flag_tx_init;

    /* ---- SDIO 电源管理 ---- */

    /* bus wakeup 触发寄存器值: 8801/DC/DW = 0x01, D80 系 = 0x11 */
    uint8_t sdio_wakeup_val;

    /* bus sleep 设置后需轮询确认 (func2 writeb + 读回等待):
     * 8801/DC/DW = 1; D80 系 = 0 (仅 func1 writeb) */
    uint8_t sdio_sleep_ack_poll;

    /* ---- SDIO RX 中断/状态读取 ---- */

    /* RX intstatus 从 misc_int_status_reg 读 (D80 系),
     * 0 时从 block_cnt_reg 读 (8801/DC/DW) */
    uint8_t sdio_misc_int_path;

    /* RX 长度按 intmaskf2 (func2/byte/block mode) 解析, D80 系: 1 */
    uint8_t sdio_intmaskf2_decode;

    /* ---- 8801 专属 ---- */

    /* msg TX cfm 轮询路径 (原 CONFIG_AIC8801 send_pkt 分支) */
    uint8_t msgcfm_poll_path;

    /* ---- FW 协议行为 ---- */

    /* mm_get_sta_info_req 变长打包时附加 more_info 字节
     * (原 CONFIG_AIC8800D80/D80N; D80X2 不带) */
    uint8_t sta_info_more_info;

    /* ---- 射频能力 ---- */

    /* 支持 5GHz 频段 (原 USE_5G 编译宏, aic_cfg.mk 仅 D80 系定义):
     * D80/D80N/D80X2 = 1; 8801/DC/DW = 0。
     * 控制信道表/监管域 5G 条目下发与 band 配置门控。 */
    uint8_t supports_5g;
};

/* 查表接口 — platform 实现 (aic_chip_desc_port.c)。
 * 未收录的 chipid 返回 NULL。 */
const struct aic_chip_desc *aic_chip_desc_lookup(uint16_t chipid);

/* ---- host 侧缓存包装 (host/driver/utils/src/aic_chip_desc.c) ----
 * probe 确定 chipid 后缓存, 此后各层免查表直接取用。 */
void aic_chip_desc_cache_set(uint16_t chipid);
const struct aic_chip_desc *aic_chip_desc_get(void);
/* 缓存的 chipid (AIC_CHIP_ID_xxx); 未缓存返回 AIC_CHIP_ID_UNMATCHED */
uint16_t aic_chip_desc_chipid_get(void);

#endif /* _AIC_CHIP_DESC_H_ */
