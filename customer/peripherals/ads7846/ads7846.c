/**
  ******************************************************************************
  * @file   ads7846.c
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

#include <rtthread.h>
#include "board.h"
#include "drv_spi.h"
#include "ads7846.h"
#include "touch.h"

/* Define -------------------------------------------------------------------*/

//#define DRV_DEBUG
#define LOG_TAG              "drv.ads"
#include <drv_log.h>

#define ADS_READ_X_ADDR                 (0x90)
#define ADS_READ_Y_ADDR                 (0xD0)
#define ADS_TOUCH_REG                   (0x40008018)

#if (TSC_MODE == 0)
    #define ADS_MAX_WIDTH                   (240)
    #define ADS_MAX_HEIGHT                  (320)
#else
    #define ADS_MAX_WIDTH                   (320)
    #define ADS_MAX_HEIGHT                  (240)
#endif
// rotate to left with 90, 180, 270
// rotate to left with 360 for mirror
//#define ROTATE_LEFT                 (90)

/* function and value-----------------------------------------------------------*/

static short ads7846_count_x(unsigned char *xp);
static short ads7846_count_y(unsigned char *xp);
static void ads7846_correct_pos(touch_msg_t ppos);

static struct rt_spi_device *ads_hspi = NULL;

bool touch_read(touch_msg_t p_msg)
{
    int cnt, res;
    unsigned char in_val[3];
    unsigned char out_val[3];
    volatile int irq;

    res = 0;
    irq = *((volatile int *)ADS_TOUCH_REG);    // get touch irq or level from gpio?
    //LOG_I("irq = %d\n",irq);
    if (ads_hspi && p_msg)
    {
        p_msg->event = (irq == 0) ? TOUCH_EVENT_UP : TOUCH_EVENT_DOWN;
        if (!irq)
        {
            // take bus and cs
            rt_spi_take_bus(ads_hspi);
            //rt_spi_take(ads_hspi);

            // get x positon
            in_val[0] = (unsigned char)ADS_READ_X_ADDR;
            in_val[1] = 0;
            in_val[2] = 0;

            cnt = rt_spi_transfer(ads_hspi, in_val, out_val, 3);

            if (cnt != 0)
            {
                LOG_D("outx 0x%02x, 0x%02x, 0x%02x\n", out_val[0], out_val[1], out_val[2]);
                p_msg->x = ads7846_count_x(out_val);
            }
            else
            {
                LOG_I("get x fail\n");
                res = 1;
            }

            // get y position
            in_val[0] = (unsigned char)ADS_READ_Y_ADDR;
            in_val[1] = 0;
            in_val[2] = 0;

            cnt = rt_spi_transfer(ads_hspi, in_val, out_val, 3);
            if (cnt != 0)
            {
                LOG_D("outy 0x%02x, 0x%02x, 0x%02x\n", out_val[0], out_val[1], out_val[2]);
                p_msg->y = ads7846_count_y(out_val);;
            }
            else
            {
                LOG_I("get y fail\n");
                res = 2;
            }

            ads7846_correct_pos(p_msg);

            // release spi bus
            rt_spi_release(ads_hspi);
            rt_spi_release_bus(ads_hspi);
        }
        else
        {
            res = 1;
            LOG_D("button not pressed\n");
        }
    }
    else
    {
        //LOG_I("spi or handle error\n");
        res = 1;
    }

    // to meet lvgl cb request,always return false
    return false;
}


static void ads7846_init(void)
{
    rt_device_t spi_bus = NULL;
    struct rt_spi_configuration cfg;

    if (ads_hspi != NULL)    // Initialized
    {
        LOG_D("ads7846 initialized before\n");
        return;
    }

    spi_bus = rt_device_find(TOUCH_DEVICE_NAME);
    if (spi_bus)
    {
        rt_device_open(spi_bus, RT_DEVICE_FLAG_RDWR);
    }
    else
    {
        LOG_I("spi1 bus not find\n");
        return;
    }
    ads_hspi = (struct rt_spi_device *)rt_device_find("ads_spi");
    if (ads_hspi != NULL)
    {
        cfg.data_width = 8;
        cfg.max_hz = 50 * 1000 * 1000; // 50m
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER | RT_SPI_CPHA | RT_SPI_CPOL;
        rt_spi_configure(ads_hspi, &cfg);
    }
    else
    {
        LOG_I("SPI MASTER not found, ads7846 init failed\n");
        return;
    }
    LOG_D("ads7846 initial success\n");
}

static short ads7846_count_x(unsigned char *data)
{
    int temp = (((data[1] << 8) | data[2]) >> 3) & 0xfff;
    //LOG_I("org x = %d\n",temp);

    return (short)temp;
}
static short ads7846_count_y(unsigned char *data)
{
    int temp = (((data[1] << 8) | data[2]) >> 3) & 0xfff;
    //LOG_I("org x = %d\n",temp);

    return (short)temp;
}

static void ads7846_correct_pos(touch_msg_t ppos)
{
#if (TSC_MODE == 0)

    // to be correct
    if (ppos->x > 240)
    {
        ppos->x = (ppos->x - 240) / 15;     // 0 ~ 239
    }
    else
    {
        ppos->x = 0;
    }
    if (ppos->y > 220)
    {
        ppos->y = ((ppos->y - 220) * 2) / 21; // 0 ~ 319
    }
    else
    {
        ppos->y = 0;
    }

#else
    // for new lcd, reg x to 0 ~ 319, y to 0 ~ 239
    // ADS_MAX_WIDTH and ADS_MAX_HEIGHT should changed.
    if (ppos->x > 240)
    {
        ppos->x = ((ppos->x - 240) << 2) / 45;     // 0 ~ 319
    }
    else
    {
        ppos->x = 0;
    }
    if (ppos->y > 220)
    {
        ppos->y = (ppos->y - 220) / 14;   // 0 ~ 239
    }
    else
    {
        ppos->y = 0;
    }
    ppos->y = ADS_MAX_HEIGHT - ppos->y - 1;
#endif

    if (ppos->x >= ADS_MAX_WIDTH)
    {
        ppos->x = ADS_MAX_WIDTH - 1;
    }
    if (ppos->y >= ADS_MAX_HEIGHT)
    {
        ppos->y = ADS_MAX_HEIGHT - 1;
    }

#if(ROTATE_LEFT == 90)
    short temp;
    temp = ppos->x;
    ppos->x = ADS_MAX_HEIGHT - ppos->y - 1;
    ppos->y = temp;
#elif (ROTATE_LEFT == 180)
    ppos->x = ADS_MAX_HEIGHT - ppos->x - 1;
    ppos->y = ADS_MAX_HEIGHT - ppos->y - 1;
#endif  // 90

    LOG_D("x = %d, y = %d\n", ppos->x, ppos->y);

    return;
}


static int rt_ads7846_spi_init(void)
{
    struct rt_spi_device *pdev = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));

    int r = rt_spi_bus_attach_device(pdev, "ads_spi", TOUCH_DEVICE_NAME, NULL);

    ads7846_init();

    return r;
}
INIT_COMPONENT_EXPORT(rt_ads7846_spi_init);


//#define ADS7846_FUNC_TEST
#ifdef ADS7846_FUNC_TEST

int cmd_adstest(int argc, char *argv[])
{
    touch_data_t post = {0};
    int res, looper;

    if (argc > 1)
    {
        looper = atoi(argv[1]);
    }
    else
    {
        looper = 0x0fffffff;
    }

    ads7846_init();
    while (looper != 0)
    {
        res = touch_read(&post);
        if (post.state)
        {
            LOG_I("x = %d, y = %d\n", post.point.x, post.point.y);
        }

        looper--;
        rt_thread_delay(100);
    }
    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_adstest, __cmd_adstest, Test hw ads);
#endif  /* ADS7846_FUNC_TEST */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
