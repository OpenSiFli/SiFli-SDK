/**
  ******************************************************************************
  * @file   um_gps_nmea.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include <rtthread.h>
#include <math.h>
#include "board.h"

#include "um_gps_nmea.h"
#include "um_gps_hal.h"

//#define DRV_DEBUG
#define LOG_TAG              "drv.gps"
#include <drv_log.h>


//GpsCallbacks umGpsCallbacks;
NmeaReader nmeaReader;

#define  MAX_NMEA_TOKENS  32

typedef struct
{
    const char  *p;
    const char  *end;
} Token;

typedef struct
{
    int     count;
    Token   tokens[MAX_NMEA_TOKENS];
} NmeaTokenizer;

#define SECS_IN_DAY (24*60*60)

static int nmea_tokenizer_init(NmeaTokenizer  *t, const char  *p, const char  *end)
{
    int    count = 0;
    const char  *q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n')
    {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p + 3 && end[-3] == '*')
    {
        end -= 3;
    }

    while (p < end)
    {
        q = p;

        q = memchr(p, ',', end - p);
        if (q == NULL)
            q = end;

        if (count < MAX_NMEA_TOKENS)
        {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }

        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token nmea_tokenizer_get(NmeaTokenizer  *t, int  index)
{
    Token  tok;
    static const char  *dummy = "";

    if (index < 0 || index >= t->count)
    {
        tok.p = tok.end = dummy;
    }
    else
        tok = t->tokens[index];

    return tok;
}


static int str2int(const char  *p, const char  *end)
{
    int   result = 0;
    int   len    = end - p;

    if (len == 0)
    {
        return -1;
    }

    for (; len > 0; len--, p++)
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result * 10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double str_to_double(const char *s, char *unused)
{
    (void)unused;

    const char    *p     = s;
    double         value = 0.0;
    double         factor;

    while (p && isspace(*p))
        p++;

    while (p && isdigit(*p))
    {
        value = value * 10 + (*p++ - '0');
    }

    if (*p == '.')
    {
        factor = 1.;

        p++;
        while (p && isdigit(*p))
        {
            factor *= 0.1;
            value  += (*p++ - '0') * factor;
        }
    }

    return value;
}

/*
 * this function only calculate the milliseconds since Jan 1, 1970
 * parameter tm is UTC time
 */
time_t mktime(struct tm *tm)
{
    long totalSecs = 0;
    long sec1;
    long sec2;
    unsigned char leapYearCnt = 0;
    unsigned char yearsNotInLeap = 0;
    unsigned short   dayInYear;

    int leapYearDaysOfMonthList[] =
    {
        31,
        29,
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    };

    int daysOfMonthList[] =
    {
        31,
        28,
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    };

    leapYearCnt = (tm->tm_year - 1972) / 4;
    yearsNotInLeap = (tm->tm_year - 1972) % 4 + 2;  /* 1970 and 1971 is not leap year */
    totalSecs = leapYearCnt * 1461 * SECS_IN_DAY; /* seconds in four years */
    if (yearsNotInLeap > 2)
    {
        sec1 = 366 * SECS_IN_DAY + (yearsNotInLeap - 1) * 365 * SECS_IN_DAY;
    }
    else
    {
        sec1 = yearsNotInLeap * 365 * SECS_IN_DAY;
    }

    dayInYear = 0;
    if (yearsNotInLeap != 2)
    {
        for (int i = 0; i < tm->tm_mon - 1; i ++)
        {
            dayInYear += daysOfMonthList[i];
        }
    }
    else
    {
        for (int i = 0; i < tm->tm_mon - 1; i ++)
        {
            dayInYear += leapYearDaysOfMonthList[i];
        }
    }

    dayInYear += (tm->tm_mday - 1);
    sec2 = dayInYear * SECS_IN_DAY + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;

    totalSecs += sec1 + sec2;

    return totalSecs;

}

static double str2float(const char  *p, const char  *end)
{
    int   len    = end - p + 1;
    char  temp[32];

    if (len == 0)
    {
        return -1.0;
    }

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy(temp, p, len);
    temp[len] = 0;
    return str_to_double(temp, NULL);
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
void nmea_reader_init()
{
    memset(&nmeaReader, 0, sizeof(NmeaReader));

    nmeaReader.pos      = 0;
    nmeaReader.overflow = 0;
    nmeaReader.utc_time.tm_mday = -1;
    nmeaReader.utc_time.tm_mon = -1;
    nmeaReader.utc_time.tm_year = -1;
    nmeaReader.fix.flags = 0;
}


static int nmea_reader_get_timestamp(NmeaReader  *r, Token  tok, time_t *timestamp)
{
    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_time.tm_year < 0)
    {
        return -1;
    }

    r->utc_time.tm_hour    = str2int(tok.p,   tok.p + 2);
    r->utc_time.tm_min     = str2int(tok.p + 2, tok.p + 4);
    r->utc_time.tm_sec     = (int)str2float(tok.p + 4, tok.end);
    r->utc_time.tm_isdst   = -1;

    *timestamp = mktime(&(r->utc_time));

    return 0;
}

static int nmea_reader_update_time(NmeaReader  *r, Token  tok)
{
    time_t timestamp = 0;
    int ret = nmea_reader_get_timestamp(r, tok, &timestamp);
    if (0 == ret)
        r->fix.timestamp = (long long)timestamp * 1000;
    return ret;
}

static int nmea_reader_update_cdate(NmeaReader  *r, Token  tok_d, Token tok_m, Token tok_y)
{

    if ((tok_d.p + 2 > tok_d.end) ||
            (tok_m.p + 2 > tok_m.end) ||
            (tok_y.p + 4 > tok_y.end))
        return -1;


    r->utc_time.tm_mday = str2int(tok_d.p,   tok_d.p + 2);
    r->utc_time.tm_mon = str2int(tok_m.p, tok_m.p + 2);
    r->utc_time.tm_year = str2int(tok_y.p, tok_y.end + 4);

    return 0;
}

static int nmea_reader_update_date(NmeaReader  *r, Token  date, Token  mtime)
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end)
    {

        /* no date info, will use host time in _update_time function
        */
    }
    /* normal case */
    day  = str2int(tok.p, tok.p + 2);
    mon  = str2int(tok.p + 2, tok.p + 4);
    year = str2int(tok.p + 4, tok.p + 6) + 2000;

    if ((day | mon | year) < 0)
    {
        return -1;
    }

    r->utc_time.tm_year  = year;
    r->utc_time.tm_mon   = mon;
    r->utc_time.tm_mday   = day;

    return nmea_reader_update_time(r, mtime);
}

static int nmea_reader_update_timemap(NmeaReader *r,
                                      Token       systime_tok,
                                      Token       timestamp_tok)
{
    int ret;
    time_t timestamp;

    if (systime_tok.p >= systime_tok.end ||
            timestamp_tok.p >= timestamp_tok.end)
    {
        r->timemap.valid = 0;
        return -1;
    }

    ret = nmea_reader_get_timestamp(r, timestamp_tok, &timestamp);
    if (ret)
    {
        r->timemap.valid = 0;
        return ret;
    }

    r->timemap.valid = 1;
    r->timemap.systime = str2float(systime_tok.p, systime_tok.end);
    r->timemap.timestamp = (GpsUtcTime)((long long)timestamp * 1000);
    return 0;
}

static double convert_from_hhmm(Token  tok)
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees * 100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}

static int nmea_reader_update_latlong(NmeaReader  *r,
                                      Token        latitude,
                                      char         latitudeHemi,
                                      Token        longitude,
                                      char         longitudeHemi)
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end)
    {
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end)
    {
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    return 0;
}


static int nmea_reader_update_altitude(NmeaReader  *r,
                                       Token        altitude,
                                       Token        units)
{
    Token   tok = altitude;

    (void)units;

    if (tok.p >= tok.end)
        return -1;

    r->fix.altitude = str2float(tok.p, tok.end);
    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    return 0;
}

static int nmea_reader_update_accuracy(NmeaReader  *r,
                                       Token        accuracy)
{
    Token   tok = accuracy;

    if (tok.p >= tok.end)
        return -1;

    //tok is cep*cc, we only want cep
    r->fix.accuracy = str2float(tok.p, tok.end);

    if (r->fix.accuracy == 99.99)
    {
        return 0;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int nmea_reader_update_bearing(NmeaReader  *r,
                                      Token        bearing)
{

    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.bearing  = str2float(tok.p, tok.end);
    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    return 0;
}


static int nmea_reader_update_speed(NmeaReader  *r,
                                    Token        speed)
{
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.speed    = str2float(tok.p, tok.end);
    r->fix.speed   *= 0.514444;    // fix for Speed Unit form Knots to Meters per Second
    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    return 0;
}

/*
 * $GPGGA,133456,0000.348,S,00000.348,E,2,07,1.2,10.7,M,48.8,M,7,0001*5A
 *  |                                                               |
 *  `---------------------------------------------------------------`-- xor value
 */
#define NMEA_MINLEN 12
int nmea_verify_packet(char *buf, int len)
{
    char sum = 0, crc;
    char *bufend = buf + len;

    if (len < NMEA_MINLEN) return -1;

    if (buf[0] != '$' && bufend[-3] != '*')
    {
        return -1;
    }

    buf++;
    len -= 4;   /* 3 + 1 */

    while (len--)
        sum ^= buf[len];

    crc = (char)strtoul(&bufend[-2], NULL, 16);

    return sum != crc;
}

static void nmea_reader_parse()
{
    /*
     *  we received a complete sentence in NmeaReader
     *  now parse it to generate a new GPS fix or SV information ...
     */
    NmeaReader *r = &nmeaReader;
    NmeaTokenizer  tzer[1];
    Token          tok;
    int rtype;
    char *pbuf = r->in;
    int len = strlen(pbuf);

    /**********************************************
    N/A G1B1 COM1
    PN N/A
    SN N/A
    HWVer V0.0
    FWVer R3.1.0Build1979
    Copyright (c), Unicore Communications Inc.
    All rights reserved.
    **********************************************/
    if (len < NMEA_MINLEN)
    {
        //if(strcasestr(pbuf, "$OK") || strcasestr(pbuf, "$Fail")) {
        if (strstr(pbuf, "$OK") || strstr(pbuf, "$Fail"))
        {
            LOG_D("%s", pbuf);
        }
        return;
    }

    // legal check
    if (nmea_verify_packet(pbuf, len - 2) != 0)
    {
        //if(strcasestr(pbuf, "N/A G1B1 COM1") || strcasestr(pbuf, "PN N/A") || strcasestr(pbuf, "SN N/A") || strcasestr(pbuf, "FWVer"))
        if (strstr(pbuf, "N/A G1B1 COM1") || strstr(pbuf, "PN N/A") || strstr(pbuf, "SN N/A") || strstr(pbuf, "FWVer"))
        {
            LOG_D("%s", pbuf);
        }
        else
        {
            return;
        }
    }

    gps_hal_update_nmea(pbuf);

    nmea_tokenizer_init(tzer, pbuf, pbuf + len);

    tok = nmea_tokenizer_get(tzer, 0);

    if (tok.p + 5 > tok.end)
    {
        LOG_D("short sentences");/*  */
        return;
    }

    // check for RNSS type
    if (!memcmp(tok.p, "BD", 2) || !memcmp(tok.p, "GB", 2))
        rtype = GNSS_CONSTELLATION_BEIDOU;
    else
        rtype = GNSS_CONSTELLATION_GPS;
    // ignore first two characters.
    tok.p += 2;
    //LOG_D("%s\n", tok.p);

    if (!memcmp(tok.p, "GGA", 3))
    {
        // GPS fix
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer, 6);

        if (tok_fixstaus.p[0] > '0')
        {

            Token  tok_time          = nmea_tokenizer_get(tzer, 1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer, 2);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 3);
            Token  tok_longitude     = nmea_tokenizer_get(tzer, 4);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 5);
            Token  tok_altitude      = nmea_tokenizer_get(tzer, 9);
            Token  tok_altitudeUnits = nmea_tokenizer_get(tzer, 10);
            nmea_reader_update_time(r, tok_time);
            nmea_reader_update_latlong(r, tok_latitude,
                                       tok_latitudeHemi.p[0],
                                       tok_longitude,
                                       tok_longitudeHemi.p[0]);
            nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
        }

    }
    else if (!memcmp(tok.p, "GLL", 3))
    {
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer, 6);

        if (tok_fixstaus.p[0] == 'A')
        {

            Token  tok_latitude      = nmea_tokenizer_get(tzer, 1);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 2);
            Token  tok_longitude     = nmea_tokenizer_get(tzer, 3);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 4);
            Token  tok_time          = nmea_tokenizer_get(tzer, 5);
            nmea_reader_update_time(r, tok_time);
            nmea_reader_update_latlong(r, tok_latitude,
                                       tok_latitudeHemi.p[0],
                                       tok_longitude,
                                       tok_longitudeHemi.p[0]);
        }
    }
    else if (!memcmp(tok.p, "GSA", 3))
    {
        Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
        int i;

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1')
        {

            Token tok_accuracy      = nmea_tokenizer_get(tzer, 15);
            nmea_reader_update_accuracy(r, tok_accuracy);
            //
            // h30/h40/h41 GPS or BD only:
            //      $GPGSA,A,3,2,5,6,7,9,13,20,30,25,29,15,,1.445,0.924,1.111*0D
            // h40 hybrid:
            //      $GNGSA,A,3,166,168,169,171,173,174,161,162,163,164,165,167,0.688,0.837,0.554*24
            // h41 hybrid:
            //      $GNGSA,A,3,56,58,59,61,63,64,51,52,53,54,55,57,1.084,0.689,0.837*2E
            // h51 Hybrid:
            //      $GNGSA,A,3,27,01,07,08,09,11,16,10,30,28,,,1.12,0.72,0.86,1*05  ==> sysid=1, GPS
            //      $GNGSA,A,3,01,02,03,04,05,07,08,10,13,,,,1.12,0.72,0.86,4*00    ==> sysid=4, BDS
            // h51 GPS or BD only:
            //      $GPGSA,A,3,02,03,06,09,12,17,19,23,28,25,,,1.34,0.85,1.04,1*1E
            //      $GBGSA,A,3,02,03,06,09,12,17,19,23,28,25,,,1.34,0.85,1.04,1*1E

            Token tok_systemID = nmea_tokenizer_get(tzer, tzer->count - 1);
            if (tzer->count == 19 && tok_systemID.p != tok_systemID.end)
            {
                if (tok_systemID.p[0] == '1' && tok_systemID.p[1] == '*')
                    rtype = GNSS_CONSTELLATION_GPS;
                else if (tok_systemID.p[0] == '4' && tok_systemID.p[1] == '*')
                    rtype = GNSS_CONSTELLATION_BEIDOU;
            }

            for (i = 3; i <= 14; ++i)
            {

                Token  tok_prn  = nmea_tokenizer_get(tzer, i);
                int prn = str2int(tok_prn.p, tok_prn.end);
                int n = 0;

                /* available for PRN 1-255 */
                // TODO
                // check PRN for BD2 satellites not starting from 160, or greater than 192
                // check PRN for GLONASS or GALILEO satellites PRN numbering
                if (prn > 0 && prn < 256)
                {
                    // h41 hybrid will use GN and has prn between 51~87
                    // BD2: 1~37 or 51~87 or 161~197
                    if (50 < prn && prn < 88)
                    {
                        rtype = GNSS_CONSTELLATION_BEIDOU;
                        prn -= 50;
                    }

                    if (rtype == GNSS_CONSTELLATION_BEIDOU && prn < 160)
                    {
                        prn += 160;
                    }
                    r->sv_list_tmp[prn].flags |= GNSS_SV_FLAGS_USED_IN_FIX;
                    r->sv_list_tmp[prn].svid = prn;

                    /* mark this parameter to identify the GSA is in fixed state */
                    r->gsa_fixed = 1;
                }
            } // end for(;;)

        }
        else
        {
            if (r->gsa_fixed == 1)
            {
                for (i = 0; i < MAX_SVS; i++)
                {
                    r->sv_list[i].flags &= ~GNSS_SV_FLAGS_USED_IN_FIX;
                    r->sv_list_tmp[i].flags &= ~GNSS_SV_FLAGS_USED_IN_FIX;
                }

                r->gsa_fixed = 0;
                r->sv_status_commit = 1;
            }
        }
    }
    else if (!memcmp(tok.p, "GSV", 3))
    {
        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

        Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
        Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);
        LOG_D("noSatellites = %d\n", noSatellites);

        if (noSatellites > 0)
        {

            int sentence = str2int(tok_sentence.p, tok_sentence.end);
            int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
            static int num_svs;
            int i;
            int prn;

            static int ff = 0;  // first found
            if (sentence == 1)
            {
                for (i = 0; i < MAX_SVS; i++)
                {
                    if (r->sv_list_tmp[i].constellation == rtype) r->sv_list_tmp[i].svid = 0;
                }
                num_svs = 0;
                ff = 1;
            }
            else
            {
                if (!ff)
                {
                    LOG_D("partial GSV, skip.");
                    return;
                }
            }

            i = 0;

            while (i < 4 && num_svs < noSatellites)
            {


                Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);

                prn = str2int(tok_prn.p, tok_prn.end);
                if (rtype == GNSS_CONSTELLATION_BEIDOU)
                {
                    // BD2: 1~37 or 51~87 or 161~197
                    if (50 < prn && prn < 88) prn -= 50;

                    if (prn < 160) prn += 160;
                }
                r->sv_list_tmp[prn].svid = prn;
                r->sv_list_tmp[prn].elevation = str2float(tok_elevation.p, tok_elevation.end);
                r->sv_list_tmp[prn].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
                r->sv_list_tmp[prn].c_n0_dbhz = str2float(tok_snr.p, tok_snr.end);
                r->sv_list_tmp[prn].constellation = rtype;

                i++;
                num_svs++;
            }

            if (sentence == totalSentences)
            {
                //memcpy(r->sv_status[rtype].sv_list, r->sv_status_tmp[rtype].sv_list, sizeof(GpsSvInfo)*GPS_MAX_SVS);
                //r->sv_status[rtype].num_svs = r->sv_status_tmp[rtype].num_svs;

                // $GPGSV,2,1,...
                // $GPGSV,2,2,... commit=1
                // $BDGSV,2,1,...
                // $BDGSV,2,2,... commit=0,changed=1
                // if no GSV, then trigger changed in RMC
                r->sv_status_commit = 1;

                if ((r->nmea_mode != POS_MODE_GN)
                        || ((r->nmea_mode == POS_MODE_GN) && (rtype == GNSS_CONSTELLATION_BEIDOU)))
                {

                    memcpy(r->sv_list, r->sv_list_tmp, sizeof(r->sv_list));
                    r->sv_status_commit = 0;
                    r->sv_status_changed = 1;
                }

                num_svs = 0;
                ff = 0;
                if (r->sv_status_changed)
                {
                    gps_hal_update_svstatus(r->sv_list);
                }
            }
        }
    }
    else if (!memcmp(tok.p, "RMC", 3))       //NMEA first line
    {
        // SEE ALSO "GSV" COMMENTS
        // EPOCH begin, if last epoch not sent out, mark it as changed
        if (r->sv_status_commit)
        {
            memcpy(r->sv_list, r->sv_list_tmp, sizeof(r->sv_list));
            r->sv_status_commit = 0;
            r->sv_status_changed = 1;
        }
        if (!memcmp(tok.p - 2, "GNR", 3))
        {
            r->nmea_mode = POS_MODE_GN;
        }
        else if (!memcmp(tok.p - 2, "BDR", 3) || !memcmp(tok.p - 2, "GBR", 3))
        {
            r->nmea_mode = POS_MODE_BD;
        }
        else
        {
            r->nmea_mode = POS_MODE_GPS;
        }

        Token  tok_fixStatus     = nmea_tokenizer_get(tzer, 2);

        if (tok_fixStatus.p[0] == 'A')
        {
            Token  tok_time          = nmea_tokenizer_get(tzer, 1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer, 3);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 4);
            Token  tok_longitude     = nmea_tokenizer_get(tzer, 5);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 6);
            Token  tok_speed         = nmea_tokenizer_get(tzer, 7);
            Token  tok_bearing       = nmea_tokenizer_get(tzer, 8);
            Token  tok_date          = nmea_tokenizer_get(tzer, 9);

            nmea_reader_update_date(r, tok_date, tok_time);
            nmea_reader_update_latlong(r, tok_latitude,
                                       tok_latitudeHemi.p[0],
                                       tok_longitude,
                                       tok_longitudeHemi.p[0]);
            nmea_reader_update_bearing(r, tok_bearing);
            nmea_reader_update_speed(r, tok_speed);

            if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG)
            {
                gps_hal_update_location(r->fix);
                r->fix.flags = 0;
            }
        }
        memset(r->sv_list_tmp, 0, sizeof(r->sv_list_tmp));    // clear the history sv info
    }
    else if (!memcmp(tok.p, "VTG", 3))
    {
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer, 9);

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
        {
            Token  tok_bearing       = nmea_tokenizer_get(tzer, 1);
            Token  tok_speed         = nmea_tokenizer_get(tzer, 5);
            nmea_reader_update_bearing(r, tok_bearing);
            nmea_reader_update_speed(r, tok_speed);
        }

    }
    else if (!memcmp(tok.p, "ZDA", 3))
    {
        Token  tok_time;
        Token  tok_year  = nmea_tokenizer_get(tzer, 4);

        if (tok_year.p[0] != '\0')
        {

            Token  tok_day   = nmea_tokenizer_get(tzer, 2);
            Token  tok_mon   = nmea_tokenizer_get(tzer, 3);
            nmea_reader_update_cdate(r, tok_day, tok_mon, tok_year);
        }

        tok_time  = nmea_tokenizer_get(tzer, 1);

        if (tok_time.p[0] != '\0')
        {
            nmea_reader_update_time(r, tok_time);
        }

    }
    else
    {
        tok.p -= 2;
    }

}

void um_gps_nmea_parse(char *buf, uint32_t len)
{
    char *end = NULL;
    int length = 0;
    NmeaReader *reader = &nmeaReader;

    while (len > 0 && *buf != '\0')
    {
        end = strchr(buf, '\n');
        if (end != NULL)
        {
            length = end - buf + 1;
            memcpy(reader->in + reader->pos, buf, length);
            reader->pos += length;
            *(reader->in + reader->pos) = '\0';
            nmea_reader_parse();
            len -= length;
            reader->pos = 0;
        }
        else
        {
            int i = 0;
            while (*(buf + i) != 0)
            {
                *(reader->in + i) = *(buf + i);
                i++;
            }
            reader->pos += i;
            break;
        }
        buf += length;
    }
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
