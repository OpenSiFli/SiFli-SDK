/**
  ******************************************************************************
  * @file   drv_audprc.h
  * @author Sifli software development team
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

#ifndef __DRV_AUDIO_PRC_H__
#define __DRV_AUDIO_PRC_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_common.h>

#define MUTE_UNDER_MIN_VOLUME  (-256)

int rt_bf0_audio_prc_init(void);
uint8_t bf0_audprc_get_tx_rbf_en();
uint8_t bf0_audprc_get_tx_channel();
void bf0_audprc_set_tx_channel(uint8_t chan);

uint8_t bf0_audprc_get_rx_channel();
void bf0_audprc_set_rx_channel(uint8_t chan);

void bf0_audprc_device_write(rt_device_t dev, rt_off_t    pos, const void *buffer, rt_size_t   size);

void bf0_audprc_dma_restart(uint16_t chann_used);
int eq_get_music_volumex2(uint8_t level);
int eq_get_tel_volumex2(uint8_t level);
int8_t eq_get_decrease_level(int is_tel, int volumex2);
uint8_t eq_is_working();
int eq_get_default_volumex2();
void bf0_audprc_eq_enable_offline(uint8_t is_enable);
void bf0_audprc_stop();

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
