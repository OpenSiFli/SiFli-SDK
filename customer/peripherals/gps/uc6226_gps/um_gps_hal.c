/**
  ******************************************************************************
  * @file   um_gps_hal.c
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

#include <time.h>
#include <string.h>
#include <rtthread.h>
#include <math.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "board.h"

#include "um_gps_hal.h"
#include "um_gps_nmea.h"

//#define DRV_DEBUG
#define LOG_TAG              "drv.gps"
#include <drv_log.h>


#define MAX_CMD_LEN            256

//#define GPS_UART_NAME           "uart2"
//#define GPS_GPIO_BIT    (11)

enum
{
    UM_GPS_NONE            = 0x00000000,
    UM_GPS_STATUS_UPDATE   = 0x00000001,
    UM_GPS_LOCATION_UPDATE = 0x00000010,
    UM_GPS_SVSTATUS_UPDATE = 0x00000100,
    UM_GPS_NMEA_UPDATE     = 0x00001000,
    UM_GPS_CMD_REQUEST     = 0x00010000,
    UM_GPS_DL_FIRMWARE     = 0x00100000,
    UM_GPS_THREAD_EXIT     = 0x80000000,
} um_gps_event_report;

static uint32_t gpsHalState = GPS_HAL_STATE_INVALID;
rt_device_t gps_uart = NULL;

GpsCallbacks umGpsCallbacks;
static GpsLocation umGpsLocation;
static GnssSvStatus umGnssSvStatus;
/*
 * NMEA sentence report so frequently that will occupy a lot resource
 */
//static gpsNmea_t umGpsNmea;
static GpsStatus umGpsStatus;


//void gps_uart_cb(rt_device_t *uart, rt_uint32_t len);
static void gps_thread_entry(void *param);

static struct rt_semaphore rx_sem;
rt_thread_t gps_thread = NULL;

static rt_err_t gps_uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    return RT_EOK;
}


/*
 * to do: support injecting assistance data
 */
int gps_send_cmd(uint8_t type, char *cmd, uint32_t len)
{
    int ret = -1;
    if (gps_uart != NULL)
    {
        rt_size_t value = rt_device_write(gps_uart, 0, cmd, len);
        if (value > 0)
            ret = 0;
    }
    else
    {
        LOG_E("GPS uart not connect\n");
    }
    return ret;
}

static void cleanup_callback()
{
    umGpsCallbacks.size = 0;
    umGpsCallbacks.gnss_sv_status_cb = NULL;
    umGpsCallbacks.nmea_cb = NULL;
    umGpsCallbacks.location_cb = NULL;
    umGpsCallbacks.status_cb = NULL;

    return;
}

int gps_hal_init()
{
    nmea_reader_init();

    gpsHalState = GPS_HAL_STATE_INIT;

    return 0;
}

int gps_hal_start(GpsCallbacks *callbacks)
{
    int ret = 0;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    gps_uart = rt_device_find(GPS_UART_NAME);
    if (gps_uart != NULL)
    {
        LOG_D("gps uart %s device find\n", GPS_UART_NAME);
        rt_err_t err = rt_device_open(gps_uart, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
        //rt_err_t err = rt_device_open(gps_uart, RT_DEVICE_FLAG_RDWR);
        LOG_D("Open gps uart %d\n", err);
        //config.baud_rate = 4800;
        config.baud_rate = 9600;
        //config.baud_rate = 14400;
        //config.baud_rate = 19200;
        //config.baud_rate = 38400;
        //config.baud_rate = 57600;
        //config.baud_rate = 115200;
        //config.baud_rate = 1000000;
        err = rt_device_control(gps_uart, RT_DEVICE_CTRL_CONFIG, &config);

        LOG_D("uart device config %d\n", err);
    }

    if (gpsHalState & GPS_HAL_STATE_DESTROY)
    {
        ret = -1;
        LOG_D("GPS distroied\n");
    }
    else if (gpsHalState & GPS_HAL_STATE_STARTED)
    {
        ret = 0;
        LOG_D("GPS UART started before\n");
    }
    else if (gps_uart == NULL)
    {
        ret = -1;
        LOG_E("GPS UART NOT CONNECT\n");
    }
    else
    {
        umGpsCallbacks = *callbacks;

        //gps_uart_open(gps_uart_cb);
        rt_device_set_rx_indicate(gps_uart, gps_uart_input);
        //rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
        gpsHalState = GPS_HAL_STATE_STARTED;
        gps_thread = rt_thread_create("gps", gps_thread_entry, NULL, 4096, RT_MAIN_THREAD_PRIORITY, 10);
        if (gps_thread != NULL)
            rt_thread_startup(gps_thread);
        else
            ret = -1;
    }
    LOG_D("gps hal started with %d\n", ret);
    return ret;
}

int gps_hal_set_local_baud(int baud)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    int res = -1;

    if (gps_uart != NULL)
    {
        config.baud_rate = baud;
        res = rt_device_control(gps_uart, RT_DEVICE_CTRL_CONFIG, &config);
        if (res == 0)
            LOG_D("set gps uart baud to %d\n", baud);
    }

    return res;
}

int gps_hal_stop(void)
{
    int ret = -1;

    if (gpsHalState & GPS_HAL_STATE_DESTROY)
    {
        ret = -1;
    }
    else if (gpsHalState & GPS_HAL_STATE_STOPPED)
    {
        ret = 0;
    }
    else
    {
        if ((gpsHalState & GPS_HAL_STATE_STARTED)
                || (gpsHalState & GPS_HAL_STATE_INIT))
        {
            //gps_uart_close(true);
            rt_device_close(gps_uart);
            gps_uart = NULL;
        }
        if (gps_thread != NULL)
        {
            rt_thread_delete(gps_thread);
            gps_thread = NULL;
        }
        gpsHalState = GPS_HAL_STATE_STOPPED;
        ret = 0;
    }

    return ret;
}

uint32_t gps_hal_get_uart(void)
{
    return (uint32_t)gps_uart;
}

int gps_hal_cleanup(void)
{
    gpsHalState = GPS_HAL_STATE_DESTROY;
    cleanup_callback();
    return 0;
}

uint32_t gps_hal_set_state(uint32_t state)
{
    gpsHalState |= state;

    return gpsHalState;
}

uint32_t gps_hal_get_state()
{
    return gpsHalState;
}

int gps_status_update(gpsHsmStatus_t hsmStatus)
{
    umGpsStatus.size = sizeof(umGpsStatus);
    switch (hsmStatus)
    {
    case GPS_HAL_STATUS_ON:
        umGpsStatus.status = GPS_STATUS_ENGINE_ON;
        break;

    case GPS_HAL_STATUS_OFF:
        umGpsStatus.status = GPS_STATUS_ENGINE_OFF;
        break;

    default:
        umGpsStatus.status = GPS_STATUS_NONE;
        break;
    }

    return 0;
}

int gps_hal_update_svstatus(GnssSvInfo sv_list[])
{
    GnssSvStatus sv_status;
    int i = 0;
    int n = 0;

    memset(&sv_status, 0, sizeof(sv_status));
    for (i = 0; i < MAX_SVS; i++)
    {
        if (sv_list[i].svid)
        {
            // Tweak Beidou PRN
            if (sv_list[i].constellation == GNSS_CONSTELLATION_BEIDOU
                    && sv_list[i].svid > 160)
                sv_list[i].svid -= 160;

            memcpy(&sv_status.gnss_sv_list[n], sv_list + i, sizeof(GnssSvInfo));
            sv_status.gnss_sv_list[n].size = sizeof(GnssSvInfo);
            n++;
        }
    }

    sv_status.num_svs = n;
    sv_status.size = sizeof(sv_status);

    if (umGpsCallbacks.gnss_sv_status_cb == NULL)
    {
        LOG_D("svstatus call back is null\n");
    }
    else
    {
        umGpsCallbacks.gnss_sv_status_cb(&sv_status);
    }

    return 0;
}

int gps_hal_update_location(GpsLocation fix)
{
    if (umGpsCallbacks.location_cb == NULL)
    {
        LOG_D("location call back is null\n");
    }
    else
    {
        umGpsCallbacks.location_cb(&fix);
    }

    return 0;
}

int gps_hal_update_nmea(char *pbuf)
{
    if (umGpsCallbacks.nmea_cb != NULL)
    {
        /*
                struct timeval tv;
                unsigned long long mytimems;
                gettimeofday(&tv,NULL);
                mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        */
        //LOG_D("gps_hal_update_nmea %d\n",strlen(pbuf));
        umGpsCallbacks.nmea_cb(0, pbuf, strlen(pbuf));
    }

    return 0;
}

#define GPS_RX_BUF_LEN          (256)
static char gps_rx_buf[GPS_RX_BUF_LEN];
static void gps_thread_entry(void *param)
{
    char ch;
    int cnt = 0;
    LOG_D("gps_thread_entry\n");
    while (1)
    {
        while (rt_device_read(gps_uart, -1, &ch, 1) != 1)
        {
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        gps_rx_buf[cnt++] = ch;

        if (ch == '\n' || ch == '\0')
        {
            if (cnt < GPS_RX_BUF_LEN)
                gps_rx_buf[cnt] = '\0';
            LOG_D("gps_thread_entry: %s\n", gps_rx_buf);
            um_gps_nmea_parse(gps_rx_buf, strlen(gps_rx_buf));
            cnt = 0;
            memset(gps_rx_buf, 0, GPS_RX_BUF_LEN);
        }
    }
}
#if 0
/*
 * design as flip buffer
 */
static uint8_t gps_nmea_buf_1[1024];
static uint8_t gps_nmea_buf_2[1024];
static uint8_t buf_flag = 1;
void gps_uart_cb(rt_device_t *uart, rt_uint32_t len)
{
    int length = -1;

    if (len > 0)
    {
        LOG_D("rev uart data %d\n", len);
        if (buf_flag != 0)
        {
            length = rt_device_read(gps_uart, -1, &gps_nmea_buf_1, len);
        }
        else
        {
            length = rt_device_read(gps_uart, -1, &gps_nmea_buf_2, len);
        }
        if (length > 0)
        {

            if (buf_flag != 0)
            {
                *(gps_nmea_buf_1 + length) = 0;
                buf_flag = 0;
                um_gps_nmea_parse((char *)gps_nmea_buf_1, length);
            }
            else
            {
                *(gps_nmea_buf_2 + length) = 0;
                buf_flag = 1;
                um_gps_nmea_parse((char *)gps_nmea_buf_2, length);
            }
        }
    }
}
#endif
INIT_COMPONENT_EXPORT(gps_hal_init);


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
