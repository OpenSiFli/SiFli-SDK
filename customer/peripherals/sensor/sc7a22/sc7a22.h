/**
  ******************************************************************************
  * @file   sc7a22.h
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

#ifndef __SC7A22_H__
#define __SC7A22_H__
#include <stdint.h>
#include "sc7a22_reg.h"


typedef enum
{
    SC7A22_ODR_DISABLE   =  0,
    SC7A22_ODR_1Hz6      =  1,
    SC7A22_ODR_12Hz5     =  2,
    SC7A22_ODR_25Hz      =  3,
    SC7A22_ODR_50Hz      =  4,
    SC7A22_ODR_100Hz     =  5,
    SC7A22_ODR_200Hz     =  6,
    SC7A22_ODR_400Hz     =  7,
    SC7A22_ODR_800Hz     =  8,
    SC7A22_ODR_1k6Hz     =  9,
    SC7A22_ODR_2k56Hz    = 10,
    SC7A22_ODR_4k257Hz   = 11,
} sc7a22_odr_t;


/*******************************************************************************/

int sc7a22_init(void);
uint32_t sc7a22_get_bus_handle(void);
uint8_t sc7a22_get_dev_addr(void);
uint8_t sc7a22_get_dev_id(void);
int sc7a22_open(void);
int sc7a22_close(void);

int sc7a22_set_fifo_mode(uint8_t val);
uint8_t sc7a22_get_fifo_count(void);
int sc7a22_read_fifo(uint8_t *buf, int len);
int sc7a20_read_fifo(uint8_t *buf, int len);
int sc7a22_set_fifo_threshold(int thd);
int sc7a22_self_check(void);

#endif /* __SC7A22_H__*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
