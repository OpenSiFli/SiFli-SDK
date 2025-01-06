/**
  ******************************************************************************
  * @file   drv_i2c.h
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

#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_common.h>

typedef struct bf0_i2c_config
{
    const char *device_name;
    I2C_TypeDef *Instance;
    IRQn_Type irq_type;
    uint8_t core;
    struct dma_config *dma_rx;
    struct dma_config *dma_tx;
} bf0_i2c_config_t;

typedef struct bf0_i2c
{
    struct rt_i2c_bus_device bus;
    I2C_HandleTypeDef handle;
    bf0_i2c_config_t *bf0_i2c_cfg;
    struct
    {
        DMA_HandleTypeDef dma_rx;
        DMA_HandleTypeDef dma_tx;
    } dma;
    rt_uint8_t i2c_dma_flag;
} bf0_i2c_t;


enum
{
#ifdef BSP_USING_I2C1
    I2C1_INDEX,
#endif
#ifdef BSP_USING_I2C2
    I2C2_INDEX,
#endif
#ifdef BSP_USING_I2C3
    I2C3_INDEX,
#endif
#ifdef BSP_USING_I2C4
    I2C4_INDEX,
#endif
#ifdef BSP_USING_I2C5
    I2C5_INDEX,
#endif
#ifdef BSP_USING_I2C6
    I2C6_INDEX,
#endif
#ifdef BSP_USING_I2C7
    I2C7_INDEX,
#endif
    I2C_MAX,
};


int rt_hw_i2c_init(void);

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
