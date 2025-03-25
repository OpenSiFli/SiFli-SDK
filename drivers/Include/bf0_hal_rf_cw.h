/**
  ******************************************************************************
  * @file   bf0_hal_rf_cw.h
  * @author Sifli software development team
  * @brief   bt rf single carrier test Header File
  ******************************************************************************
*/
/**
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

#ifndef BT_RF_CW_H
#define BT_RF_CW_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  RF set power interface,just used for RF test
  * @param[in]  type, 0:ble or br, 1:edr. only used for 52x
  * @param[in]  tx power,dbm unit
  * @param[out]     return param,0:sucess, other:error
  */
extern uint8_t btdm_rf_power_set(uint8_t type, int8_t txpwr);

/**
  * @brief  bt rf single carrier test function.
  * @param[in]  is_start: start test or stop test. 1:start test;0:stop test
  * @param[in]  pa is transmit power. dbm unit
  * @param[in]  set channel N, means 2402+ N MHz.
  */
extern void cw_config_bt(uint8_t is_start, uint8_t pa, uint8_t channel);



#ifdef __cplusplus
}
#endif

#endif /* BT_RF_CW_H */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/