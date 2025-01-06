/**
  ******************************************************************************
  * @file   bluetooth_int.h
  * @author Sifli software development team
  * @brief SIFLI Bluetooth stack internal header.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
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


#ifndef __BLUETOOTH_INT_H
#define __BLUETOOTH_INT_H

extern void rom_config_set_default_xtal_enabled(uint8_t default_xtal_enabled);
extern void rom_config_set_default_rc_cycle(uint8_t default_rc_cycle);
extern void rom_config_set_lld_prog_delay(uint8_t lld_prog_delay);
extern void     ble_standby_sleep_after_handler_rom();
extern int32_t ble_standby_sleep_pre_handler_rom();
extern void bluetooth_idle_hook_func(void);
extern int bluetooth_stack_suspend(void);
extern int32_t ble_standby_sleep_pre_handler_rom();
extern void     ble_standby_sleep_after_handler_rom();
extern void rf_ptc_config(uint8_t is_reset);
extern void bluetooth_config(void);


void *bt_mem_alloc(rt_size_t size);
void *bt_mem_calloc(rt_size_t count, rt_size_t nbytes);
void bt_mem_free(void *ptr);




#endif //__BLUETOOTH_INT_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
