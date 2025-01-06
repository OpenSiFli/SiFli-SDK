/**
  ******************************************************************************
  * @file   bf0_sibles_eq_config.h
  * @author Sifli software development team
  * @brief Header file - Sibles EQ config.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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

#ifndef BF0_SIBLES_EQ_CONFIG_H_
#define BF0_SIBLES_EQ_CONFIG_H_

#include <stdint.h>
#include <stdlib.h>
#include "bf0_ble_common.h"

typedef uint32_t eq_state_t;

enum
{
    BLE_EQ_TYPE_CALL = 0,
    BLE_EQ_TYPE_MUSIC = 1,
};

eq_state_t ble_eq_set_adc(int8_t val);
eq_state_t ble_eq_get_adc(void);
eq_state_t ble_eq_set_dac(uint8_t type, int8_t val);
eq_state_t ble_eq_get_dac(uint8_t type);
eq_state_t ble_eq_set_max_dac_level(uint8_t type, int8_t val);
eq_state_t ble_eq_get_max_dac_level(uint8_t type);
eq_state_t ble_eq_set_volume_lvl_via_idx(uint8_t type, uint8_t idx, int8_t val);
eq_state_t ble_eq_get_volume_lvl_via_idx(uint8_t type, uint8_t idx);
eq_state_t ble_eq_set_default_volume(uint8_t type, uint8_t idx, int8_t val);
eq_state_t ble_eq_get_version(void);
eq_state_t ble_eq_set_parameter(uint8_t type, uint8_t idx, uint8_t *para, uint8_t len);
eq_state_t ble_eq_get_parameter(uint8_t type, uint8_t idx);
eq_state_t ble_eq_set_state(uint8_t type, uint8_t state);
eq_state_t ble_eq_start(uint8_t is_start);
void ble_eq_register(uint8_t conn_idx);



#endif //BF0_SIBLES_EQ_CONFIG_H_

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
