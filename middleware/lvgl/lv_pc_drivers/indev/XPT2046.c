/**
  ******************************************************************************
  * @file   XPT2046.c
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

#include "XPT2046.h"
#if USE_XPT2046

#include <stddef.h>
#include LV_DRV_INDEV_INCLUDE
#include LV_DRV_DELAY_INCLUDE

/*********************
 *      DEFINES
 *********************/
#define CMD_X_READ  0b10010000
#define CMD_Y_READ  0b11010000

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void xpt2046_corr(int16_t *x, int16_t *y);
static void xpt2046_avg(int16_t *x, int16_t *y);

/**********************
 *  STATIC VARIABLES
 **********************/
int16_t avg_buf_x[XPT2046_AVG];
int16_t avg_buf_y[XPT2046_AVG];
uint8_t avg_last;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the XPT2046
 */
void xpt2046_init(void)
{

}

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no ore data to be read
 */
bool xpt2046_read(lv_indev_data_t *data)
{
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    bool valid = true;
    uint8_t buf;

    int16_t x = 0;
    int16_t y = 0;

    uint8_t irq = LV_DRV_INDEV_IRQ_READ;

    if (irq == 0)
    {
        LV_DRV_INDEV_SPI_CS(0);

        LV_DRV_INDEV_SPI_XCHG_BYTE(CMD_X_READ);         /*Start x read*/

        buf = LV_DRV_INDEV_SPI_XCHG_BYTE(0);           /*Read x MSB*/
        x = buf << 8;
        buf = LV_DRV_INDEV_SPI_XCHG_BYTE(CMD_Y_READ);  /*Until x LSB converted y command can be sent*/
        x += buf;

        buf =  LV_DRV_INDEV_SPI_XCHG_BYTE(0);   /*Read y MSB*/
        y = buf << 8;

        buf =  LV_DRV_INDEV_SPI_XCHG_BYTE(0);   /*Read y LSB*/
        y += buf;

        /*Normalize Data*/
        x = x >> 3;
        y = y >> 3;
        xpt2046_corr(&x, &y);
        xpt2046_avg(&x, &y);

        last_x = x;
        last_y = y;

        LV_DRV_INDEV_SPI_CS(1);
    }
    else
    {
        x = last_x;
        y = last_y;
        avg_last = 0;
        valid = false;
    }

    data->point.x = x;
    data->point.y = y;
    data->state = valid == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;

    return valid;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void xpt2046_corr(int16_t *x, int16_t *y)
{
#if XPT2046_XY_SWAP != 0
    int16_t swap_tmp;
    swap_tmp = *x;
    *x = *y;
    *y = swap_tmp;
#endif

    if ((*x) > XPT2046_X_MIN)(*x) -= XPT2046_X_MIN;
    else(*x) = 0;

    if ((*y) > XPT2046_Y_MIN)(*y) -= XPT2046_Y_MIN;
    else(*y) = 0;

    (*x) = (uint32_t)((uint32_t)(*x) * XPT2046_HOR_RES) /
           (XPT2046_X_MAX - XPT2046_X_MIN);

    (*y) = (uint32_t)((uint32_t)(*y) * XPT2046_VER_RES) /
           (XPT2046_Y_MAX - XPT2046_Y_MIN);

#if XPT2046_X_INV != 0
    (*x) =  XPT2046_HOR_RES - (*x);
#endif

#if XPT2046_Y_INV != 0
    (*y) =  XPT2046_VER_RES - (*y);
#endif


}


static void xpt2046_avg(int16_t *x, int16_t *y)
{
    /*Shift out the oldest data*/
    uint8_t i;
    for (i = XPT2046_AVG - 1; i > 0 ; i--)
    {
        avg_buf_x[i] = avg_buf_x[i - 1];
        avg_buf_y[i] = avg_buf_y[i - 1];
    }

    /*Insert the new point*/
    avg_buf_x[0] = *x;
    avg_buf_y[0] = *y;
    if (avg_last < XPT2046_AVG) avg_last++;

    /*Sum the x and y coordinates*/
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for (i = 0; i < avg_last ; i++)
    {
        x_sum += avg_buf_x[i];
        y_sum += avg_buf_y[i];
    }

    /*Normalize the sums*/
    (*x) = (int32_t)x_sum / avg_last;
    (*y) = (int32_t)y_sum / avg_last;
}

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
