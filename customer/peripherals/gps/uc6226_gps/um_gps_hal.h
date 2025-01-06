/**
  ******************************************************************************
  * @file   um_gps_hal.h
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

#ifndef _GPS_HAL_H_
#define _GPS_HAL_H_

#include "gps_types.h"

typedef struct
{
    int             prn;
    float           cno;
    float           elevation; /* degrees */
    float           azimuth; /* degrees */
} gpsSvInfo_t;

typedef struct
{
    uint16_t             num_svs;
    gpsSvInfo_t     sv_list[GPS_MAX_SVS];
    uint32_t             ephemeris_mask;
    uint32_t             almanac_mask;
    uint32_t             used_in_fix_mask;
} gpsSvStatus_t;

typedef struct
{
    uint16_t             num_svs_gln;
    gpsSvInfo_t     gln_sv_list[GLN_MAX_SVS];
    uint32_t             gln_ephemeris_mask;
    uint32_t             gln_almanac_mask;
    uint32_t             gln_used_in_fix_mask;
} glnSvStatus_t;

typedef struct
{
    gpsSvStatus_t gps_sv_status;
    glnSvStatus_t gln_sv_status;
} gnssSvStatus_t;

typedef struct
{
    double             utcTime;
    uint16_t             msgLen;
    int8_t              nmeaInfo[GPS_NMEA_STRING_MAXLEN];
} gpsNmea_t;


enum
{
    GPS_HSM_STARTMODE_NORMAL,
    GPS_HSM_STARTMODE_TEST_COLD,
    GPS_HSM_STARTMODE_TEST_HOT,
    GPS_HSM_STARTMODE_TEST_WARM,
};

/*
 * Android status
 */
typedef enum
{
    GPS_HAL_STATUS_ON,
    GPS_HAL_STATUS_OFF,
} gpsHsmStatus_t;

enum
{
    GPS_HAL_STATE_INVALID  = 0x00000001,
    GPS_HAL_STATE_DESTROY  = 0x00000010,
    GPS_HAL_STATE_INIT     = 0x00000100,
    GPS_HAL_STATE_STARTED  = 0x00001000,
    GPS_HAL_STATE_STOPPED  = 0x00010000,
};


int gps_send_cmd(uint8_t type, char *cmd, uint32_t len);
int gps_hal_init(void);
int gps_hal_start(GpsCallbacks *callbacks);
int gps_hal_stop(void);
uint32_t gps_hal_set_state(uint32_t state);
uint32_t gps_hal_get_state();
int gps_status_update(gpsHsmStatus_t hsmStatus);
int gps_hal_update_svstatus(GnssSvInfo sv_list[]);
int gps_hal_update_location(GpsLocation fix);
int gps_hal_update_nmea(char *pbuf);
int gps_hal_cleanup(void);
int gps_hal_set_local_baud(int baud);
uint32_t gps_hal_get_uart(void);

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
