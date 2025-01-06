/**
  ******************************************************************************
  * @file   um_gps_if.h
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

#ifndef __GPS_HOST_IF_H__
#define __GPS_HOST_IF_H__

#include "gps_types.h"


#define GPS_CMD_MAXLEN      (64)


enum
{
    MSG_DELETE_AIDING = 1,
    MSG_SET_INTERVAL,
    MSG_SET_POS_MODE,
    MSG_GET_FWINFO,
    MSG_SET_BAUD,
    MSG_SAVE_CFG,
    MSG_REQ_ASSIST,
    MSG_WAKEUP,
    MSG_SLEEP,
    MSG_NMEA_VER,
    MSG_NMEA_OUTPUT,
    MSG_START_DEBUG,
    MSG_OUTPUT_PPS,
};

enum
{
    START_MODE_HOT,
    START_MODE_COLD,
    START_MODE_WARM,
};

enum
{
    GPS_DELETE_EPHEMERIS = 1, // 0x0001
    GPS_DELETE_ALMANAC = 2, // 0x0002
    GPS_DELETE_POSITION = 4, // 0x0004
    GPS_DELETE_TIME = 8, // 0x0008
    GPS_DELETE_IONO = 16, // 0x0010
    GPS_DELETE_ALL = 65535, // 0xFFFF
};

enum
{
    SATELLITE_SYSTEM_GPS_L1  = 0x00000001,
    SATELLITE_SYSTEM_GPS_L5  = 0x00000004,
    SATELLITE_SYSTEM_BDS_B1  = 0x00000010,
    SATELLITE_SYSTEM_BDS_B3  = 0x00000040,
    SATELLITE_SYSTEM_GLN_L1  = 0x00000100,
    SATELLITE_SYSTEM_GLN_L2  = 0x00000200,
    SATELLITE_SYSTEM_GAL_E1  = 0x00001000,
    SATELLITE_SYSTEM_GAL_E5a = 0x00002000,
    SATELLITE_SYSTEM_GAL_E5b = 0x00004000,
    SATELLITE_SYSTEM_SBAS    = 0x00100000,
};

/*
 * Create_GPS_Task
 *
 * create gps deamon task
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int Create_GPS_Task();

/*
 * um_gps_start
 *
 * Start GPS
 *
 * Param:
 *     callbacks: pointer to struct GpsCallbacks,
 *                which include callback functions who are interested in GPS location, Satellite Infomation and status
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_start(GpsCallbacks *callbacks);

/*
 * um_gps_stop
 *
 * stop GPS Engine
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_stop();


/*
 * um_gps_set_freq
 *
 * set gps engine positioning frequency, 1000 ms per time by default
 * only can be called after gps engine was started
 *
 * Param:
 *     freq: U16
 *         positioning frequency in milliseconds, only support 1000, 500, 200
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_set_freq(uint16_t freq);

/*
 * um_gps_delete_aiding_data
 *
 * delete different aiding data,such as ephemeris, time, position, almanac, etc.
 * only can be called after gps engine was started
 *
 * Param:
 *      flags: U32
 *             can choose from below:
 *                GPS_DELETE_EPHEMERIS = 1, // 0x0001
 *                GPS_DELETE_ALMANAC = 2, // 0x0002
 *                GPS_DELETE_POSITION = 4, // 0x0004
 *                GPS_DELETE_TIME = 8, // 0x0008
 *                GPS_DELETE_IONO = 16, // 0x0010
 *                GPS_DELETE_ALL = 65535, // 0xFFFF
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_delete_aiding_data(uint16_t flags);

/*
 * um_gps_set_position_mode
 *
 * select different satellite systems for positioning
 * only can be called after gps engine was started
 *
 * Param:
 *     mode: U32
 *             the satellite system can be chosed depends on Firmware
 *                bit0  - GPS L1
 *                bit2  - GPS L5
 *                bit4  - BDS B1
 *                bit6  - BDS B3
 *                bit8  - Glonass L1
 *                bit9 - Glonass L2
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_set_position_mode(uint32_t mode);

/*
 * um_gps_set_start_mode
 *
 * set gps start mode, cold/warm/hot start
 *
 * Param:
 *     mode: U16
 *           0 -- START_MODE_HOT
 *           1 -- START_MODE_COLD
 *           2 -- START_MODE_WARM
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_set_start_mode(uint16_t mode);

/*
 * um_gps_req_assist
 *
 * request assist infomation from internet
 *
 * Param:
 *     type: U8
 *           0 -- GPS
 *           1 -- BDS
 *           2 -- GLONASS
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_req_assist(uint8_t type);

/*
 * um_gps_fw_info
 *
 * print Firmware infomation
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_fw_info();

/*
 * um_gps_set_baud
 *
 * set the baud rate of data
 * only can be called after gps engine was started
 *
 * Param:
 *     baud: U32
 *     the baudrate can be supported:
 *                9600
 *                14400
 *                19200
 *                33600
 *                38400
 *                57600
 *                115200
 *                230400
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_set_baud(uint32_t baud);

/*
 * um_gps_set_nmea_output
 *
 * enable and disable the NMEA message output, set the output frequency
 * only can be called after gps engine was started
 *
 * Param:
 *     type: U8
             0 - NMEA message
             1 - Navigation Message
       flag : U8
 *            0 - GGA
 *            1 - GLL
 *            2 - GSA
 *            3 - GSV
 *            4 - RMC
 *            5 - VTG
 *            6 - ZDA
 *      freq: U8
 *          0   -- disable
 *          1~5 -- once every 1~5 seconds
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_set_nmea_output(uint8_t type, uint8_t flag, uint8_t freq);

/*
 * um_gps_set_nmea_version
 *
 * set the nmea sentence output format
 *
 * Param:
 *     version: U8
 *             48 - NMEA v3.0
 *             81 - NMEA v4.1
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_set_nmea_version(uint8_t version);

/*
 * um_gps_pps_output
 *
 * enable GPS chip output date, time and hardware timing pulse
 *
 * Param:
 *     flag: U8
 *           the meaning of each bit as follow:
 *           bit0: 0 -- close time pulse output; 1 -- open time pulse output
 *           bit1: 0 -- edge rise align with each second; 1 -- edge down align with each second
             bit2: 0 -- ouput time pulse only when time was valid;  1 -- always ouput time pulse infomation
 *           bit3: 0 -- do not print out TIMTP message; 1 -- print out TIMTP message;
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_pps_output(uint8_t flag);

/*
 * um_gps_start_debug
 *
 * start debug mode, Firmware will output more debug infomation
 * suggest to set the baud rate above 115200 bps
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_start_debug();

/*
 * um_gps_init
 *
 * start gps module, wait location callback
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_init(void);

/*
 * um_gps_get_location
 *
 * Get north latitude and east longititude degree and altitude
 *
 * Param:
 *     lati: double
 *
 *     longiti:double
 *
 *     alti: double
 *
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_get_location(double *lati, double *longiti, double *alti);

/*
 * um_gps_open
 *
 * start gps module, wait location callback
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_open(void);

/*
 * um_gps_close
 *
 * stop gps module
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    -1 -- fail
 */
int um_gps_close(void);

/*
 * um_gps_self_check
 *
 * check hardware connection
 *
 * Param:
 *     none
 *
 * return value
 *    0  -- succeed
 *    other -- fail
 */
int um_gps_self_check(void);

#endif // __GPS_HOST_IF_H__
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
