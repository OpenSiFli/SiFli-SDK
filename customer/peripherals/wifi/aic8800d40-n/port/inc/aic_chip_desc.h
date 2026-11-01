/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Chip description port interface - single library (one host lib for all chip families).
 *
 * The host side no longer uses CONFIG_AIC88xx model macros: all model differences
 * are folded into this description table, the platform provides the static data
 * per model (platform/fmac/aic_chip_desc_port.c), and the host dispatches by
 * chipid lookup at runtime.
 *
 * Precedent: aic_fw_port.h (platform provides firmware data, host fetches by fw_id).
 */

#ifndef _AIC_CHIP_DESC_H_
#define _AIC_CHIP_DESC_H_

#include "aic_porting_types.h"

/* Chip product id — consistent with the AICWF_IC enum in host drv_api/inc/sys_api.h.
 * The port header does not depend on driver-internal headers, so it is defined here. */
#define AIC_CHIP_ID_8801        0
#define AIC_CHIP_ID_8800DC      1
#define AIC_CHIP_ID_8800DW      2
#define AIC_CHIP_ID_8800D80     3
#define AIC_CHIP_ID_8800D80N    4
#define AIC_CHIP_ID_8800D80X2   5
#define AIC_CHIP_ID_UNMATCHED   0xFFFF

struct aic_chip_desc
{
    /* corresponding chip product id (AIC_CHIP_ID_xxx) */
    uint16_t chipid;

    /* ---- SDIO/USB bus protocol behavior ----
     * (former CONFIG macros: sdio_co.c / usb_co.c / hal_co.c) */

    /* TX header[3] = CRC8(ponl-107); filled with 0 (reserved) when 0.
     * D80 family: 1; 8801/DC/DW: 0. Shared by hal msg push and sdio data header. */
    uint8_t tx_hdr_crc8;

    /* The SDIO flowctrl register read must be ANDed with SDIOWIFI_FLOWCTRL_MASK_REG.
     * D80 family: 0; 8801/DC/DW: 1. */
    uint8_t sdio_fc_reg_mask;

    /* Initial value of func_flag_tx (false on the D80 family, true otherwise).
     * func_flag_rx is always true and is not in the table. */
    uint8_t func_flag_tx_init;

    /* ---- SDIO power management ---- */

    /* bus wakeup trigger register value: 8801/DC/DW = 0x01, D80 family = 0x11 */
    uint8_t sdio_wakeup_val;

    /* bus sleep must be polled for acknowledgement after setting
     * (func2 writeb + read back and wait):
     * 8801/DC/DW = 1; D80 family = 0 (func1 writeb only) */
    uint8_t sdio_sleep_ack_poll;

    /* ---- SDIO RX interrupt/status read ---- */

    /* RX intstatus is read from misc_int_status_reg on the D80 family,
     * and from block_cnt_reg when 0 (8801/DC/DW) */
    uint8_t sdio_misc_int_path;

    /* RX length is decoded from intmaskf2 (func2/byte/block mode), D80 family: 1 */
    uint8_t sdio_intmaskf2_decode;

    /* ---- 8801 specific ---- */

    /* msg TX cfm polling path (former CONFIG_AIC8801 send_pkt branch) */
    uint8_t msgcfm_poll_path;

    /* ---- msg channel flow control ---- */

    /* msg (hal msg) is sent without flow control (no func1 flow_ctrl_reg credit
     * query). The D80N FW does not maintain msg fc (that register is always 0),
     * so it must be set to 1 to send directly, matching the model special case in
     * Linux aicsdio.c aicwf_sdio_tx_msg.
     * Default 0 = flow control (original behavior): when a platform tree desc
     * table has not synced this field the behavior is unchanged, so there is no
     * silent regression. On DC/DW the flow control is decided dynamically by the
     * func2 release (func_flag_tx) and is unrelated to this field
     * (see sdio_co.c sdio_msg_fc_enabled()). */
    uint8_t msg_fc_bypass;

    /* ---- FW protocol behavior ---- */

    /* append the more_info byte when packing the variable-length mm_get_sta_info_req
     * (former CONFIG_AIC8800D80/D80N; not carried by D80X2) */
    uint8_t sta_info_more_info;

    /* ---- RF capability ---- */

    /* supports the 5GHz band (former USE_5G compile macro, defined for the D80
     * family only in aic_cfg.mk):
     * D80/D80N/D80X2 = 1; 8801/DC/DW = 0.
     * Gates the 5G entries of the control channel table / regulatory domain and
     * the band configuration. */
    uint8_t supports_5g;
};

/* Lookup interface — implemented by platform (aic_chip_desc_port.c).
 * Returns NULL for chipids not covered. */
const struct aic_chip_desc *aic_chip_desc_lookup(uint16_t chipid);

/* ---- host-side cache wrapper (host/driver/utils/src/aic_chip_desc.c) ----
 * cached after probe determines chipid; all layers then use it without lookup. */
void aic_chip_desc_cache_set(uint16_t chipid);
const struct aic_chip_desc *aic_chip_desc_get(void);
/* cached chipid (AIC_CHIP_ID_xxx); returns AIC_CHIP_ID_UNMATCHED if not cached */
uint16_t aic_chip_desc_chipid_get(void);

#endif /* _AIC_CHIP_DESC_H_ */
