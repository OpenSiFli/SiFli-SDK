/**
  ******************************************************************************
  * @file   baro_service.h
  * @author Sifli software development team
  * @brief Sifli barometer service interface
  * @{
  ******************************************************************************
*/
/*
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

#ifndef BAROMETER_SERVICE_H
#define BAROMETER_SERVICE_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "data_service_subscriber.h"

enum {
    MSG_SERVICE_BARO_RANGE_REQ             = (MSG_SERVICE_CUSTOM_ID_BEGIN + 0x10),
    MSG_SERVICE_BARO_RANGE_RSP             = MSG_SERVICE_BARO_RANGE_REQ | RSP_MSG_TYPE,
    MSG_SERVICE_ALTITUDE_RANGE_REQ        = MSG_SERVICE_BARO_RANGE_REQ + 1, 
    MSG_SERVICE_ALTITUDE_RANGE_RSP        = MSG_SERVICE_ALTITUDE_RANGE_REQ | RSP_MSG_TYPE,
};

#define BAROS_BARO_RANGE_LEN            (2*4)
#define BAROS_ALTI_RANGE_LEN            (3*4)

typedef struct 
{
    uint32_t cur_baro;
    uint32_t max_baro;
    uint32_t min_baro;
    int32_t cur_alti;
    int32_t max_alti;
    int32_t min_alti;
} custom_baro_data_table_t;

/// @}  file

#endif  /*BAROMETER_SERVICE_H*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
