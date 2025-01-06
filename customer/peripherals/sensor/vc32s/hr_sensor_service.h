/**
  ******************************************************************************
  * @file   hr_sensor_service.h
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

#ifndef _HR_SENSOR_SERVICE_H_
#define _HR_SENSOR_SERVICE_H_

#include "board.h"
#include "sensor.h"
#include "vcHr02Hci.h"
#if defined (HR_ALGO_USING_GSENSOR_DATA)
    #include "gsensor_service.h"
#endif

#define PPG_BUFF_SIZE           40              //ppg buff szie
#define HR_PERIOD_TIMER         800     //vc32s every  800ms get fifo data
#define HR_ALGO_PEROID          800

typedef struct
{
    uint8_t status;
    uint8_t hr;
    uint8_t bp_l;
    uint8_t bp_h;
    uint8_t spo2;
} hr_algo_result_t;

typedef struct
{
    uint8_t hr_id;
    uint8_t type;
} hr_sensor_info_t;

/* hr device structure */
struct hr_sensor_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};

typedef struct
{
    bool            vcFifoReadFlag;
    bool            vcPsFlag;
    uint8_t         wearstatus;
    uint8_t         envValue[3];
    uint8_t         SampleRate;
    int16_t         ppgValue[PPG_BUFF_SIZE];
} hr_raw_data_t;

rt_err_t vc32s_int_open(void);
void vc32s_int_close(void);
void vc32sStart(vcHr02_t *pVcHr02, vcHr02Mode_t vcHr02WorkMode);

#endif  // SENSOR_GOODIX_GH3011_H__
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
