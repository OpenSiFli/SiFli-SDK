/**
  ******************************************************************************
  * @file   rom_config.h
  * @author Sifli software development team
  * @brief Header file - Define BCPU ROM configure APIs.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef ROM_CONIFG_H
#define ROM_CONIFG_H

#ifdef SOC_SF32LB55X
    #include "ipc_queue.h"
#endif
#include "ble_stack.h"
#include "rwip.h"
#ifdef BSP_USING_PC_SIMULATOR
    #include "compiler.h"
#endif


#define BLE_CONTROLLER_ENABLE_MASK (1 << 0)
#define BT_CONTROLLER_ENABLE_MASK (1 << 1)


typedef struct
{
    // bit 0: BLE, 1: BT
    uint8_t controller_enable_bit;
    uint8_t lld_prog_delay;
    uint8_t lld_prog_delay_min;
    uint8_t default_sleep_mode;
    uint8_t default_sleep_enabled;
    uint8_t default_xtal_enabled;
    uint8_t default_rc_cycle;
    uint8_t default_swprofiling_cfg;
    uint32_t max_sleep_time;
} rom_config_t;

typedef struct
{
    uint8_t bt_max_acl;
    uint8_t bt_max_sco;
    uint8_t ble_max_act;
    uint8_t ble_max_ral;
    uint8_t ble_max_iso;
    uint8_t is_bt_on;
    uint8_t is_ble_on;
    uint8_t ble_rx_desc;
    uint8_t bt_rx_desc;
} bluetooth_act_configt_t;


extern uint16_t g_rom_em_offset[ROM_EM_END];

uint8_t rom_config_get_ble_controller_enabled(void);
uint8_t rom_config_get_bt_controller_enabled(void);

uint8_t rom_config_get_controller_enabled(void);
uint8_t rom_config_get_lld_prog_delay(void);
uint8_t rom_config_get_lld_prog_delay_min(void);
uint8_t rom_config_get_default_sleep_mode(void);
uint8_t rom_config_get_default_sleep_enabled(void);
uint8_t rom_config_get_default_xtal_enabled(void);
uint8_t rom_config_get_default_rc_cycle(void);
uint32_t rom_config_get_max_sleep_time(void);
uint8_t rom_config_get_default_swprofiling(void);


void rom_config_set_ble_service_working_core(uint8_t working_core);
void rom_config_set_lld_prog_delay(uint8_t lld_prog_delay);
void rom_config_set_lld_prog_delay_min(uint8_t lld_prog_delay_min);
void rom_config_set_default_sleep_mode(uint8_t default_sleep_mode);
void rom_config_set_default_sleep_enabled(uint8_t default_sleep_enabled);
void rom_config_set_default_xtal_enabled(uint8_t default_xtal_enabled);
void rom_config_set_default_rc_cycle(uint8_t default_rc_cycle);
void rom_config_set_max_sleep_time(uint32_t max_sleep_time);
void rom_config_set_tl(const struct bt_eif_api *tl, uint8_t idx);
const struct bt_eif_api *rom_config_get_tl(uint8_t idx);


__INLINE uint16_t rom_config_get_em_offset(rom_em_offset_t type)
{
    return g_rom_em_offset[type];
}

//uint16_t rom_config_get_em_offset(rom_em_offset_t type);

#ifdef SOC_SF32LB55X
    ipc_queue_cfg_t *ble_ipc_buffer_get(void);
#endif

rt_device_t ble_uart_handle_get(uint8_t index);

ble_mem_config_t *ble_memory_config_get(void);

int8_t ble_max_nb_of_cp_get(void);

bluetooth_act_configt_t *rom_config_get_default_link_config(void);

uint8_t rom_config_get_ble_max_act(void);

#endif //ROM_CONIFG_H


