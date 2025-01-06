/**
  ******************************************************************************
  * @file   um_gps_nmea.h
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

#ifndef _GPS_NMEA_H_
#define _GPS_NMEA_H_

#include "gps_types.h"

enum
{
    GNSS_MAX_SVS_COUNT = 64u, // 64
};

enum
{
    NMEA_GGA = 0x0001,
    NMEA_GLL = 0x0002,
    NMEA_GSA = 0x0004,
    NMEA_GSV = 0x0008,
    NMEA_RMC = 0x0010,
    NMEA_VTG = 0x0020,
    NMEA_ZDA = 0x0040,
    NAV_POS  = 0x0080,
    NAV_VEL  = 0x0100,
    NAV_TIME = 0x0200,
    NAV_ACC  = 0x0400,
};

enum
{
    GNSS_SV_FLAGS_NONE = 0,
    GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA = 1, // (1 << 0)
    GNSS_SV_FLAGS_HAS_ALMANAC_DATA = 2, // (1 << 1)
    GNSS_SV_FLAGS_USED_IN_FIX = 4, // (1 << 2)
    GNSS_SV_FLAGS_HAS_CARRIER_FREQUENCY = 8, // (1 << 3)
};

enum
{
    GPS_LOCATION_HAS_LAT_LONG = 1, // 0x0001
    GPS_LOCATION_HAS_ALTITUDE = 2, // 0x0002
    GPS_LOCATION_HAS_SPEED = 4, // 0x0004
    GPS_LOCATION_HAS_BEARING = 8, // 0x0008
    GPS_LOCATION_HAS_HORIZONTAL_ACCURACY = 16, // 0x0010
    GPS_LOCATION_HAS_VERTICAL_ACCURACY = 32, // 0x0020
    GPS_LOCATION_HAS_SPEED_ACCURACY = 64, // 0x0040
    GPS_LOCATION_HAS_BEARING_ACCURACY = 128, // 0x0080
};

#define GPS_LOCATION_HAS_ACCURACY GPS_LOCATION_HAS_HORIZONTAL_ACCURACY

/* commands sent to the gps thread */
enum
{
    POS_MODE_GPS = 1,
    POS_MODE_BD = 2,
    POS_MODE_GN = 3,
};

typedef struct
{
    int valid;
    double systime;
    GpsUtcTime timestamp;
} UmTimemap_t;

typedef struct
{
    int     pos;
    int     overflow;
    struct tm utc_time;
    GpsLocation  fix;
    GnssSvInfo sv_list[MAX_SVS];
    GnssSvInfo sv_list_tmp[MAX_SVS];
    int     sv_status_changed;
    int     sv_status_commit;
    char    in[NMEA_MAX_SIZE];
    int     gsa_fixed;
    UmTimemap_t timemap;
    int     nmea_mode;
} NmeaReader;

void nmea_reader_init();
void um_gps_nmea_parse(char *buf, uint32_t len);

#endif /* _GPS_NMEA_H_ */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
