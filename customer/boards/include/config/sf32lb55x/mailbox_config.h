/**
  ******************************************************************************
  * @file   mailbox_config.h
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

#ifndef __MAILBOX_CONFIG_H__
#define __MAILBOX_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

// Between LCPU and HCPU
#define MAILBOX_L2H_CH1_CONFIG                      \
    {                                            \
       .handle.Instance     = L2H_MAILBOX,       \
       .channel             = MAILBOX_CHANNEL_1, \
       .core                = CORE_ID_LCPU,      \
       .irqn                = LCPU2HCPU_IRQn,    \
       .name                = "mb_l2h1",         \
       .device.buffer_size  = LCPU2HCPU_MB_CH1_BUF_SIZE,              \
       .device.tx_buffer_addr = LCPU2HCPU_MB_CH1_BUF_START_ADDR,      \
       .device.rx_buffer_addr = LCPU2HCPU_MB_CH1_BUF_START_ADDR,      \
    }


#define MAILBOX_H2L_CH1_CONFIG                      \
    {                                            \
       .handle.Instance     = H2L_MAILBOX,       \
       .channel             = MAILBOX_CHANNEL_1, \
       .core                = CORE_ID_HCPU,      \
       .irqn                = HCPU2LCPU_IRQn,    \
       .name                = "mb_h2l1",        \
       .device.buffer_size  = HCPU2LCPU_MB_CH1_BUF_SIZE,              \
       .device.tx_buffer_addr = HCPU2LCPU_MB_CH1_BUF_START_ADDR,     \
       .device.rx_buffer_addr = HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR),     \
    }

#define MAILBOX_L2H_CH2_CONFIG                      \
    {                                            \
       .handle.Instance     = L2H_MAILBOX,       \
       .channel             = MAILBOX_CHANNEL_2, \
       .core                = CORE_ID_LCPU,      \
       .irqn                = LCPU2HCPU_IRQn,    \
       .name                = "mb_l2h2",         \
       .device.buffer_size  = LCPU2HCPU_MB_CH2_BUF_SIZE,              \
       .device.tx_buffer_addr = LCPU2HCPU_MB_CH2_BUF_START_ADDR,      \
       .device.rx_buffer_addr = LCPU2HCPU_MB_CH2_BUF_START_ADDR,      \
    }


#define MAILBOX_H2L_CH2_CONFIG                      \
    {                                            \
       .handle.Instance     = H2L_MAILBOX,       \
       .channel             = MAILBOX_CHANNEL_2, \
       .core                = CORE_ID_HCPU,      \
       .irqn                = HCPU2LCPU_IRQn,    \
       .name                = "mb_h2l2",        \
       .device.buffer_size  = HCPU2LCPU_MB_CH2_BUF_SIZE,              \
       .device.tx_buffer_addr = HCPU2LCPU_MB_CH2_BUF_START_ADDR,     \
       .device.rx_buffer_addr = HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR),     \
    }



#ifdef SOC_BF0_HCPU
#define BIDIR_MB_1_CONFIG                \
    {                                   \
        .name = "bdmb_hl1",            \
        .rx_dev_name = "mb_l2h1",       \
        .tx_dev_name = "mb_h2l1",       \
    }
#define BIDIR_MB_3_CONFIG                \
    {                                   \
        .name = "bdmb_hl2",            \
        .rx_dev_name = "mb_l2h2",       \
        .tx_dev_name = "mb_h2l2",       \
    }

#ifdef SOC_BF0_LCPU
#define BIDIR_MB_1_CONFIG                \
    {                                   \
        .name = "bdmb_lh1",            \
        .rx_dev_name = "mb_h2l1",       \
        .tx_dev_name = "mb_l2h1",       \
    }
#define BIDIR_MB_3_CONFIG                \
    {                                   \
        .name = "bdmb_lh2",            \
        .rx_dev_name = "mb_h2l2",       \
        .tx_dev_name = "mb_l2h2",       \
    }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAILBOX_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
