/**
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * aic_chip_desc platform data implementation.
 *
 * Under the single-library unified architecture this file is the sole data home
 * of the former host-side CONFIG_AIC88xx model macros: the bus protocol behavior,
 * power management and protocol differences of each model are all expressed as
 * static data tables.
 * Trimming: when a product uses only one model, just delete the other table
 * entries (the host prints a notice when a lookup fails).
 *
 * Data source (original macro location, verified one by one during migration):
 * - tx_hdr_crc8:       sdio_co.c tx header[3] / hal_co.c aicwf_hal_msg_push
 * - sdio_fc_reg_mask:  sdio_co.c flowctrl read then & SDIOWIFI_FLOWCTRL_MASK_REG
 * - func_flag_tx_init: sdio_co.c / usb_co.c func_flag_tx static initial value
 * - sdio_wakeup_val:   sdio_co.c aicwf_sdio_bus_wakeup wakeup_reg_val
 * - sdio_sleep_ack_poll: sdio_co.c aicwf_sdio_bus_sleep_allow
 * - sdio_misc_int_path / sdio_intmaskf2_decode: sdio_co.c rx intstatus read
 * - msgcfm_poll_path:  sdio_co.c send_pkt 8801 cfm polling branch
 * - msg_fc_bypass:     sdio_co.c aicwf_sdio_tx_msg flow-control bypass (D80N=1 only,
 *                      matching the Linux aicsdio.c tx_msg model check; default 0=flow control)
 * - sta_info_more_info: lmac_msg.h mm_get_sta_info_req.more_info (variable-length packing)
 * - supports_5g:      USE_5G macro (fhost_config.c channel table/rwnx_main.c stack_start)
 */

#include "aic_chip_desc.h"

static const struct aic_chip_desc aic_chip_desc_table[] =
{
    {
        .chipid             = AIC_CHIP_ID_8801,
        .tx_hdr_crc8        = 0,
        .sdio_fc_reg_mask   = 1,
        .func_flag_tx_init  = 1,
        .sdio_wakeup_val    = 0x01,
        .sdio_sleep_ack_poll = 1,
        .sdio_misc_int_path = 0,
        .sdio_intmaskf2_decode = 0,
        .msgcfm_poll_path   = 1,
        .msg_fc_bypass      = 0,
        .sta_info_more_info = 0,
        .supports_5g        = 0,
    },
    {
        .chipid             = AIC_CHIP_ID_8800DC,
        .tx_hdr_crc8        = 0,
        .sdio_fc_reg_mask   = 1,
        .func_flag_tx_init  = 1,
        .sdio_wakeup_val    = 0x01,
        .sdio_sleep_ack_poll = 1,
        .sdio_misc_int_path = 0,
        .sdio_intmaskf2_decode = 0,
        .msgcfm_poll_path   = 0,
        .msg_fc_bypass      = 0,
        .sta_info_more_info = 0,
        .supports_5g        = 0,
    },
    {
        .chipid             = AIC_CHIP_ID_8800DW,
        .tx_hdr_crc8        = 0,
        .sdio_fc_reg_mask   = 1,
        .func_flag_tx_init  = 1,
        .sdio_wakeup_val    = 0x01,
        .sdio_sleep_ack_poll = 1,
        .sdio_misc_int_path = 0,
        .sdio_intmaskf2_decode = 0,
        .msgcfm_poll_path   = 0,
        .msg_fc_bypass      = 0,
        .sta_info_more_info = 0,
        .supports_5g        = 0,
    },
    {
        .chipid             = AIC_CHIP_ID_8800D80,
        .tx_hdr_crc8        = 1,
        .sdio_fc_reg_mask   = 0,
        .func_flag_tx_init  = 0,
        .sdio_wakeup_val    = 0x11,
        .sdio_sleep_ack_poll = 0,
        .sdio_misc_int_path = 1,
        .sdio_intmaskf2_decode = 1,
        .msgcfm_poll_path   = 0,
        .msg_fc_bypass      = 0,
        .sta_info_more_info = 1,
        .supports_5g        = 1,
    },
    {
        .chipid             = AIC_CHIP_ID_8800D80N,
        .tx_hdr_crc8        = 1,
        .sdio_fc_reg_mask   = 0,
        .func_flag_tx_init  = 0,
        .sdio_wakeup_val    = 0x11,
        .sdio_sleep_ack_poll = 0,
        .sdio_misc_int_path = 1,
        .sdio_intmaskf2_decode = 1,
        .msgcfm_poll_path   = 0,
        .msg_fc_bypass      = 1,
        .sta_info_more_info = 1,
        .supports_5g        = 1,
    },
    {
        .chipid             = AIC_CHIP_ID_8800D80X2,
        .tx_hdr_crc8        = 1,
        .sdio_fc_reg_mask   = 0,
        .func_flag_tx_init  = 0,
        .sdio_wakeup_val    = 0x11,
        .sdio_sleep_ack_poll = 0,
        .sdio_misc_int_path = 1,
        .sdio_intmaskf2_decode = 1,
        .msgcfm_poll_path   = 0,
        .msg_fc_bypass      = 0,
        .sta_info_more_info = 0,   /* X2 has no more_info (original macro only for D80/D80N) */
        .supports_5g        = 1,
    },
};

const struct aic_chip_desc *aic_chip_desc_lookup(uint16_t chipid)
{
    uint16_t i;

    for (i = 0; i < sizeof(aic_chip_desc_table) / sizeof(aic_chip_desc_table[0]); i++)
    {
        if (aic_chip_desc_table[i].chipid == chipid)
        {
            return &aic_chip_desc_table[i];
        }
    }
    return NULL;
}
