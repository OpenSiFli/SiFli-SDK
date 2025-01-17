/**
  ******************************************************************************
  * @file   bf0_sibles_weather_service.h
  * @author Sifli software development team
  * @brief Sibles ble weather service header file.
 *
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

#ifndef __BF0_SIBLES_WEATHER_SERVICE_h
#define __BF0_SIBLES_WEATHER_SERVICE_h


typedef struct
{
    uint8_t city_id[9];
    int8_t high_temp;
    int8_t low_temp;
    uint8_t type;
    uint8_t day;
    uint8_t week;
} ble_weather_srv_data_t;

typedef struct
{
    uint8_t city_id[9];
    uint8_t city_name[10];
} ble_weather_srv_city_id_name_t;


typedef enum
{
    BLE_WEATHER_SRV_TYPE_SUN,
    BLE_WEATHER_SRV_TYPE_RAIN,
    BLE_WEATHER_SRV_TYPE_CLOUD,
    BLE_WEATHER_SRV_TYPE_FOG,
} ble_weather_srv_type_t;

typedef struct
{
    uint8_t type_name[8];
} ble_weather_type_name_t;



typedef enum
{
    BLE_WEATHER_SRV_WEEK_MON,
    BLE_WEATHER_SRV_WEEK_TUE,
    BLE_WEATHER_SRV_WEEK_WEN,
    BLE_WEATHER_SRV_WEEK_THUR,
    BLE_WEATHER_SRV_WEEK_FRI,
    BLE_WEATHER_SRV_WEEK_SAT,
    BLE_WEATHER_SRV_WEEK_SUN,
} ble_weather_srv_week_t;

typedef struct
{
    uint8_t week_name[5];
} ble_weather_week_name_t;


const uint8_t *ble_weather_srv_get_city_name(uint8_t *city_id);

int ble_weather_srv_register(void);

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
