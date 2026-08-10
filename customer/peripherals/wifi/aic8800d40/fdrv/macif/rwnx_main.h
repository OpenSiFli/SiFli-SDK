/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file rtos_main.h
 *
 * @brief Declarations related to the WiFi stack integration within an RTOS.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

#ifndef RTOS_MAIN_H_
#define RTOS_MAIN_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rtos_port.h"
#include "rwnx_defs.h"

extern u8 chip_id;
extern u8 chip_sub_id;
extern u8 chip_mcu_id;
extern u8 chip_fw_match;
extern int adap_test;
extern struct rwnx_hw *g_rwnx_hw;

#define CHIP_ID_H_MASK  0xC0
#define IS_CHIP_ID_H()  ((chip_id & CHIP_ID_H_MASK) == CHIP_ID_H_MASK)

void rwnx_frame_parser(char* tag, char* data, unsigned long len);
void rwnx_data_dump(char* tag, void* data, unsigned long len);
void rwnx_buffer_dump(char* tag, u8 *data, u32_l len);

int rwnx_fdrv_init(struct rwnx_hw *rwnx_hw);
int rwnx_fdrv_deinit(struct rwnx_hw *rwnx_hw);
int rwnx_read_efuse_mac(struct rwnx_hw *rwnx_hw);
int rwnx_read_local_mac(void);
uint32_t rwnx_fmac_remote_sta_max_get(struct rwnx_hw *rwnx_hw);
#endif // RTOS_MAIN_H_