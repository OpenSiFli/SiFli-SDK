/**
  ******************************************************************************
  * @file   um_gps_if.c
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtthread.h>
#include <math.h>

#include "board.h"
#include "um_gps_hal.h"
#include "um_gps_nmea.h"
#include "um_gps_if.h"

//#define DRV_DEBUG
#define LOG_TAG              "drv.gps"
#include <drv_log.h>


static char *cmd = NULL;
GpsLocation ggps_info;
GpsCallbacks gps_cb;

static int um_gps_gpio_set(int en)
{
    struct rt_device_pin_status st;
    struct rt_device_pin_mode m;
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found\n");
        return -1;
    }
    if (en)
    {
        rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
#ifdef GPS_POWER_BIT
        // used power gpio, power up when enable
        if (GPS_POWER_BIT != 0xff)
        {
            // set reset pin to output mode
            m.pin = GPS_POWER_BIT;
            m.mode = PIN_MODE_OUTPUT;
            rt_device_control(device, 0, &m);
            st.pin = GPS_POWER_BIT;
            st.status = 1;
            rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
        }
#endif // GPS_POWER_BIT

        // set reset pin to output mode
        m.pin = GPS_GPIO_BIT;
        m.mode = PIN_MODE_OUTPUT;
        rt_device_control(device, 0, &m);
        // reset
        st.pin = GPS_GPIO_BIT;
        st.status = 0;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

        st.pin = GPS_GPIO_BIT;
        st.status = 1;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    }
    else
    {
        // reset
        st.pin = GPS_GPIO_BIT;
        st.status = 0;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

        rt_device_close(device);
    }

    return 0;
}
int um_gps_start(GpsCallbacks *callbacks)
{
    LOG_D("enter %s", __FUNCTION__);

    int ret = 0;

    if (cmd != NULL) // cmd alloced, it opened before?
        return -1;

    um_gps_gpio_set(1);
    ret = gps_hal_start(callbacks);

    gps_status_update(GPS_HAL_STATUS_ON);

    cmd = (char *)malloc(GPS_CMD_MAXLEN);
    if (cmd == NULL)
        ret = -2;

    //um_gps_set_freq(1000);

    LOG_D("leave %s, %d", __FUNCTION__, ret);

    return ret;
}

int um_gps_stop()
{
    int ret = 0;

    ret = gps_hal_stop();
    gps_status_update(GPS_HAL_STATUS_OFF);

    if (cmd)
    {
        free(cmd);
        cmd = NULL;
    }
    um_gps_gpio_set(0);

    LOG_D("%s", __FUNCTION__);

    return ret;
}

uint8_t bd_check_eor(char *buf, int len)
{
    uint8_t csum = 0;

    while (len--)
        csum ^= buf[len];

    return csum;
}


int um_gps_set_freq(uint16_t freq)
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;

    sprintf(cmd, "$CFGNAV,1000,%d,3", freq);
    length = strlen(cmd);
    sum = bd_check_eor(cmd + 1, length - 1);
    sprintf(cmd + length, "*%02X\r\n", sum);
    ret = gps_send_cmd(MSG_SET_INTERVAL, cmd, strlen(cmd));
    if (ret == 0)
    {
        sprintf(cmd, "$CFGSAVE\r\n");
        ret = gps_send_cmd(MSG_SAVE_CFG, cmd, strlen(cmd));
        if (ret != 0)
            LOG_D("Set cmd MSG_SAVE_CFG fail with %d\n", ret);
    }
    else
        LOG_D("Set cmd MSG_SET_INTERVAL fail with %d\n", ret);

    return ret;
}

int um_gps_get_freq()
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;

    sprintf(cmd, "$CFGNAV\r\n");
    ret = gps_send_cmd(MSG_SET_INTERVAL, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("SET command MSG_SET_INTERVAL fail %d\n", ret);

    return ret;
}

int um_gps_set_nmea_version(uint8_t version)
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;

    sprintf(cmd, "$CFGNMEA,h%02X", version);
    length = strlen(cmd);
    sum = bd_check_eor(cmd + 1, length - 1);
    sprintf(cmd + length, "*%02X\r\n", sum);
    ret = gps_send_cmd(MSG_NMEA_VER, cmd, strlen(cmd));
    if (ret == 0)
    {
        sprintf(cmd, "$CFGSAVE\r\n");
        ret = gps_send_cmd(MSG_SAVE_CFG, cmd, strlen(cmd));
        if (ret != 0)
            LOG_D("Set cmd MSG_SAVE_CFG fail with %d\n", ret);
    }
    else
        LOG_D("Set cmd MSG_NMEA_VER fail with %d\n", ret);

    return ret;
}

int um_gps_set_position_mode(uint32_t mode)
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;

    // BD: $CFGSYS,h10*5E
    // GN: $CFGSYS,h11*5F
    if (mode)
    {
        sprintf(cmd, "$CFGSYS,h%02X", mode);
        length = strlen(cmd);
        sum = bd_check_eor(cmd + 1, length - 1);
        sprintf(cmd + length, "*%02X\r\n", sum);
    }
    else
    {
        sprintf(cmd, "$CFGSYS\r\n");
    }

    ret = gps_send_cmd(MSG_SET_POS_MODE, cmd, strlen(cmd));
    if (ret == 0)
    {
        sprintf(cmd, "$CFGSAVE\r\n");
        ret = gps_send_cmd(MSG_SAVE_CFG, cmd, strlen(cmd));
        if (ret != 0)
            LOG_D("Set cmd MSG_SAVE_CFG fail with %d\n", ret);
    }
    else
        LOG_D("Set cmd MSG_SET_POS_MODE fail with %d\n", ret);

    return ret;
}

int um_gps_delete_aiding_data(uint16_t flags)
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;
    uint8_t bits = 0;

    if (flags == GPS_DELETE_ALL)
    {
        bits |= 0x95;
    }
    else if (flags & GPS_DELETE_EPHEMERIS)
    {
        bits |= 0x01;
    }
    else if (flags & (GPS_DELETE_POSITION | GPS_DELETE_TIME))
    {
        bits |= 0x04;
    }
    else if (flags & GPS_DELETE_IONO)
    {
        bits |= 0x10;
    }
    else if (flags & GPS_DELETE_ALMANAC)
    {
        bits |= 0x80;
    }
    memset(&cmd, 0, sizeof(cmd));
    sprintf(cmd, "$RESET,0,h%02X", bits);
    length = strlen(cmd);
    sum = bd_check_eor(cmd + 1, length - 1);
    sprintf(cmd + length, "*%02X\r\n", sum);
    ret = gps_send_cmd(MSG_DELETE_AIDING, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_DELETE_AIDING fail with %d\n", ret);

    return ret;
}

int um_gps_set_start_mode(uint16_t mode)
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;
    uint8_t bits = START_MODE_HOT;  //default

    if (mode == START_MODE_COLD)
    {
        bits |= 0xFF;
    }
    else if (mode == START_MODE_WARM)
    {
        bits |= 0x01;
    }

    memset(&cmd, 0, sizeof(cmd));
    sprintf(cmd, "$RESET,0,h%02X", bits);
    length = strlen(cmd);
    sum = bd_check_eor(cmd + 1, length - 1);
    sprintf(cmd + length, "*%02X\r\n", sum);
    ret = gps_send_cmd(MSG_DELETE_AIDING, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_DELETE_AIDING fail with %d\n", ret);

    return ret;
}

int um_gps_req_assist(uint8_t type)
{
    int ret = -1;

    sprintf(cmd, "ASSIST");
    ret = gps_send_cmd(MSG_REQ_ASSIST, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_REQ_ASSIST fail with %d\n", ret);

    return ret;
}

int um_gps_fw_info()
{
    int ret = -1;

    sprintf(cmd, "$PDTINFO\r\n");
    ret = gps_send_cmd(MSG_GET_FWINFO, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_GET_FWINFO fail with %d\n", ret);

    return ret;
}

int um_gps_set_baud(uint32_t baud)
{
    int ret = -1;
    uint8_t sum;
    uint8_t len = 0;

    sprintf(cmd, "$CFGPRT,,0,%d,129,3", baud);
    len = strlen(cmd);
    sum = bd_check_eor(cmd + 1, len - 1);
    sprintf(cmd + len, "*%02X\r\n", sum);
    ret = gps_send_cmd(MSG_SET_BAUD, cmd, strlen(cmd));
    if (ret != 0)
    {
        return ret;
    }
    // add a delay here to make sure new baud work
    rt_thread_delay(10);
    sprintf(cmd, "$CFGSAVE\r\n");
    ret = gps_send_cmd(MSG_SAVE_CFG, cmd, strlen(cmd));
    if (ret == 0)
    {
        // make sure update local serial baud to new value !!!!
        gps_hal_set_local_baud(baud);
    }
    else
        LOG_D("Set cmd MSG_SAVE_CFG fail with %d\n", ret);

    return ret;
}

int um_gps_set_nmea_output(uint8_t type, uint8_t flag, uint8_t freq)
{
    int ret = -1;
    uint8_t sum;
    uint8_t length = 0;

    if (flag >= 8)
    {
        sprintf(cmd, "$CFGMSG,%d,,%d", type, freq);
    }
    else
    {
        sprintf(cmd, "$CFGMSG,%d,%d,%d", type, flag, freq);
    }
    length = strlen(cmd);
    sum = bd_check_eor(cmd + 1, length - 1);
    sprintf(cmd + length, "*%02X\r\n", sum);

    ret = gps_send_cmd(MSG_NMEA_OUTPUT, cmd, strlen(cmd));

    if (ret != 0)
    {
        LOG_D("Set cmd MSG_NMEA_OUTPUT fail with %d\n", ret);
        return ret;
    }

    sprintf(cmd, "$CFGSAVE\r\n");
    ret = gps_send_cmd(MSG_SAVE_CFG, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_SAVE_CFG fail with %d\n", ret);

    return ret;
}

int um_gps_start_debug()
{
    int ret = -1;

    sprintf(cmd, "$CFGDEBUG,1,1\r\n");
    ret = gps_send_cmd(MSG_START_DEBUG, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_START_DEBUG fail with %d\n", ret);

    return ret;
}

int um_gps_pps_output(uint8_t flag)
{
    int ret = -1;

    sprintf(cmd, "$CFGTP,1000000,500000,%d,0,800,0\r\n", flag);
    ret = gps_send_cmd(MSG_OUTPUT_PPS, cmd, strlen(cmd));
    if (ret != 0)
        return ret;

    sprintf(cmd, "$CFGSAVE\r\n");
    ret = gps_send_cmd(MSG_SAVE_CFG, cmd, strlen(cmd));
    if (ret != 0)
        LOG_D("Set cmd MSG_SAVE_CFG fail with %d\n", ret);

    return ret;
}

static void gps_loc_cb(GpsLocation *param)
{
    if (param != NULL)
    {
        LOG_D("loc: longit = %.6f, latitude=%.6f, altitude=%f, speed = %f\n", param->longitude, param->latitude, param->altitude, param->speed);
        ggps_info.longitude = param->longitude;
        ggps_info.latitude = param->latitude;
        ggps_info.speed = param->speed;
        ggps_info.size = param->size;
        ggps_info.altitude = param->altitude;
    }
}

static void gps_sta_cb(GpsStatus *param)
{
    //if(param != NULL)
    //    LOG_I("sta_cb: size = %d, status = 0x%x\n",param->size,param->status);
}

static void gps_gnss_sv_cb(GnssSvStatus *param)
{
    //if(param != NULL)
    //    LOG_I("gnss_sv_cb: size=%d, svs = %d\n",param->size, param->num_svs);
}

static void gps_nmea_cb(GpsUtcTime timestamp, const char *nmea, int length)
{
    //if(nmea != NULL)
    //    LOG_I("nmea_cb: %s, timestamp = %d, len=%d\n",nmea,timestamp,length);
}

int um_gps_open(void)
{
    int res = 0;
    res = um_gps_start(&gps_cb);

    res = um_gps_set_freq(1000);

    return res;
}

int um_gps_close(void)
{
    return um_gps_stop();
}

int um_gps_init(void)
{
    int res = 0;
    gps_cb.gnss_sv_status_cb = gps_gnss_sv_cb;
    gps_cb.location_cb = gps_loc_cb;
    gps_cb.nmea_cb = gps_nmea_cb;
    gps_cb.status_cb = gps_sta_cb;
    gps_cb.size = sizeof(GpsCallbacks);

    ggps_info.latitude = -360.00;
    ggps_info.longitude = -360.00;
    ggps_info.altitude = -1000.00;
    ggps_info.speed = 0.0;
    ggps_info.size = 0;

    //res = um_gps_start(&gps_cb);
    //um_gps_set_freq(1000);

    return res;
}

int um_gps_get_location(double *lati, double *longiti, double *alti)
{
    //if(ggps_info.size == 0)
    //    return 1;

    *lati = ggps_info.latitude;
    *longiti = ggps_info.longitude;
    *alti = ggps_info.altitude;

    return 0;
}

int um_gps_self_check(void)
{
    struct rt_device_pin_status st;
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found\n");
        return -1;
    }
    st.pin = GPS_GPIO_BIT;
    st.status = 0;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));

    if (st.status == 0)
        return -2;

    return 0;
}

#define GPS_CMD_TEST
#ifdef GPS_CMD_TEST


int cmd_gps(int argc, char *argv[])
{
    double lat, lon, alt;
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        um_gps_init();
        int res = um_gps_open();
        LOG_I("GPS open %d\n", res);
    }
    else if (strcmp(argv[1], "-gps") == 0)
    {
        if (um_gps_get_location(&lat, &lon, &alt) == 0)
            LOG_I("Get location North %.6f, East %.6f, high %f\n", lat, lon, alt);
        else
            LOG_I("Get location fail\n");
    }
    else if (strcmp(argv[1], "-ver") == 0)
    {
        um_gps_fw_info();
    }
    else if (strcmp(argv[1], "-frq") == 0)
    {
        um_gps_get_freq();
    }
    else if (strcmp(argv[1], "-bd") == 0)
    {
        int res;
        int value = atoi(argv[2]);
        res = um_gps_set_baud(value);
        if (res == 0)
            LOG_I("Set baud to %d done\n", value);
        else
            LOG_I("Set baud to %d fail\n", value);
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_gps, __cmd_gps, Test hw gps);

#endif //GPS_CMD_TEST
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
