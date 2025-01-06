/**
  ******************************************************************************
  * @file   bf0_sibles_fmpt.c
  * @author Sifli software development team
  * @brief Header file - Sibles BLE FMP target implmentation.
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

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "bf0_sibles_ias.h"
#include "bf0_ble_fmpt.h"


typedef struct
{
    ble_fmpt_callback callback;
} ble_fmpt_env_t;


static ble_fmpt_env_t g_findt_env;

static ble_fmpt_env_t *ble_fmpt_get_env(void)
{
    return &g_findt_env;
}

static void ble_fmpt_set_cbk(uint8_t conn_idx, uint8_t alert_lvl)
{
    ble_fmpt_env_t *env = ble_fmpt_get_env();

    env->callback(conn_idx, alert_lvl);
}

void ble_fmpt_init(ble_fmpt_callback callback)
{
    ble_fmpt_env_t *env = ble_fmpt_get_env();
    int8_t ret = ble_ias_init(ble_fmpt_set_cbk);
    //RT_ASSERT(ret >= 0);
    env->callback = callback;
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
