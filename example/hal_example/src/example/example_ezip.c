/**
  ******************************************************************************
  * @file   example_ezip.c
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
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"
#ifdef HAL_EZIP_MODULE_ENABLED

#ifndef hwp_ezip
    #define hwp_ezip hwp_ezip1
#endif

/* Example Description:
 *
 * foregound image size is 88*88, area (10,5)~(97,92),
 * background size is 100*100, area (0,0)~(99,99),
 *
 * blend ezip format foreground image with background image(black color).
 * 1. The full size ezip format image is copy to the background image area (10,5)~(97,92).
 *
 * updated background image
 * (0,0)
 * +---------+-------------+
 * |                       |
 * |   (10,5)              |
 * |     +-----------+     |
 * |    | copied full|     |
 * |    |  size image|     |
 * |    +------------+     |
 * |              (97,92)  |
 * |                       |
 * +-----------------------+
 *                        (99,99)
 *
 *
 * 2. The left part of ezip format image is copy to the backgound image area (10,5)~(49,54).
 *
 * (0,0)
 * +---------+-------------+
 * |                       |
 * |   (10,5)              |
 *      +------------+     |
 * |    |   copied   |     |
 * |    | partial img|     |
 * |    +------------+     |
 * |              (49,54)  |
 * |                       |
 * +-----------------------+
 *                        (99,99)
 *
 *
 */

#define IMG_WIDTH        (88)
#define IMG_HEIGHT       (88)
#define OUTPUT_WIDTH     (100)
#define OUTPUT_HEIGHT    (100)
/* RGB565, 2bytes each pixel */
#define PIXEL_SIZE       (2)


#define MAKE_ARGB8888_COLOR(a,r,g,b) ((((a)&0xFF)<<24) | (((r)&0xFF)<<16)| (((g)&0xFF)<<8) | ((b)&0xFF))
#define GET_ARGB8888_ALPHA(color)   (((color) >> 24) & 0xFF)
#define GET_ARGB8888_RED(color)     (((color) >> 16) & 0xFF)
#define GET_ARGB8888_GREEN(color)   (((color) >> 8) & 0xFF)
#define GET_ARGB8888_BLUE(color)    ((color) & 0xFF)

#define MAKE_RGB565_COLOR(r,g,b) ((((r)&0xF8)<<8)| (((g)&0xFC)<<3) | (((b)&0xF8)>>3))
#define GET_RGB565_RED(color)     (((color) >> 11) & 0x1F)
#define GET_RGB565_GREEN(color)   (((color) >> 5) & 0x3F)
#define GET_RGB565_BLUE(color)    ((color) & 0x1F)

#define MAKE_RGB888_COLOR(r,g,b) ((((r)&0xFF)<<16)| (((g)&0xFF)<<8) | ((b)&0xFF))
#define GET_RGB888_RED(color)     (((color) >> 16) & 0xFF)
#define GET_RGB888_GREEN(color)   (((color) >> 8) & 0xFF)
#define GET_RGB888_BLUE(color)    ((color) & 0xFF)

typedef struct
{
    uint8_t *data;
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    uint16_t total_width;
    uint16_t total_height;
    uint8_t format;
} img_buf_cmp_param_t;


static EPIC_HandleTypeDef epic_handle;
static EZIP_HandleTypeDef ezip_handle;
volatile static uint8_t epic_done_flag;
ALIGN(4)
static uint8_t fg_img_buf[] =
{
#include "img_clock_565A_s_ezip.dat"
};
ALIGN(4)
static uint8_t bg_img_buf[OUTPUT_WIDTH * OUTPUT_HEIGHT * PIXEL_SIZE];

static uint8_t cmp_buf[] =
{
#include "img_clock_565A_s_ezip_cmp.dat"
};

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void init_epic(void)
{
    // Initialize driver and enable EPIC IRQ
    HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EPIC_IRQn);

#ifdef HAL_EZIP_MODULE_ENABLED
    HAL_NVIC_SetPriority(EZIP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EZIP_IRQn);

    epic_handle.hezip = &ezip_handle;
    epic_handle.hezip->Instance = hwp_ezip;
    HAL_EZIP_Init(epic_handle.hezip);
#endif  /* HAL_EZIP_MODULE_ENABLED */

    epic_handle.Instance = hwp_epic;
    HAL_EPIC_Init(&epic_handle);
}

static void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    /* set flag to indicate epic operation done */
    epic_done_flag = 1;
}

void EZIP_IRQHandler(void)
{
    ENTER_INTERRUPT();

    HAL_EZIP_IRQHandler(&ezip_handle);

    LEAVE_INTERRUPT();
}

uint32_t calc_blended_color(uint32_t bg_color, uint32_t fg_color, uint8_t opacity, uint32_t color_format)
{
    uint8_t r, g, b;
    uint32_t color = 0;

    /* use same round operation as EPIC does */
    if ((EPIC_COLOR_RGB565 == color_format)
            || (EPIC_COLOR_ARGB8565 == color_format))
    {
        r = (GET_RGB565_RED(fg_color) * opacity + GET_RGB565_RED(bg_color) * (255 - opacity) + 16) >> 5;
        g = (GET_RGB565_GREEN(fg_color) * opacity + GET_RGB565_GREEN(bg_color) * (255 - opacity) + 32) >> 6;
        b = (GET_RGB565_BLUE(fg_color) * opacity + GET_RGB565_BLUE(bg_color) * (255 - opacity) + 16) >> 5;
        color = MAKE_RGB565_COLOR(r, g, b);
    }
    else if ((EPIC_COLOR_RGB888 == color_format)
             || (EPIC_COLOR_ARGB8888 == color_format))
    {
        r = (GET_RGB888_RED(fg_color) * opacity + GET_RGB888_RED(bg_color) * (255 - opacity) + 127) >> 8;
        g = (GET_RGB888_GREEN(fg_color) * opacity + GET_RGB888_GREEN(bg_color) * (255 - opacity) + 127) >> 8;
        b = (GET_RGB888_BLUE(fg_color) * opacity + GET_RGB888_BLUE(bg_color) * (255 - opacity) + 127) >> 8;
        color = MAKE_RGB888_COLOR(r, g, b);
    }
    else
    {
        RT_ASSERT(0);
    }

    return color;
}

static bool cmp_data(img_buf_cmp_param_t *buf0, img_buf_cmp_param_t *buf1)
{
    uint32_t i, j;
    uint8_t buf0_pixel_size;
    bool buf0_alpha;
    uint8_t buf1_pixel_size;
    uint8_t cmp_pixel_size;
    uint8_t *data0;
    uint8_t *data1;
    uint32_t color;
    uint32_t fg_color;
    uint32_t check_color;
    int32_t diff;


    if ((EPIC_COLOR_RGB565 != buf1->format)
            && (EPIC_COLOR_RGB888 != buf1->format))
    {
        return false;
    }

    /* check whether format is compatible */
    if (buf1->format != buf0->format)
    {
        if (((EPIC_COLOR_ARGB8565 == buf0->format)
                || (EPIC_COLOR_RGB565 == buf0->format))
                && (EPIC_COLOR_RGB888 == buf1->format))
        {
            return false;
        }
        if ((EPIC_COLOR_RGB565 == buf1->format)
                && ((EPIC_COLOR_ARGB8888 == buf0->format)
                    || (EPIC_COLOR_RGB888 == buf0->format)))
        {
            return false;
        }
    }

    if ((buf0->x1 - buf0->x0) != (buf1->x1 - buf1->x0))
    {
        return false;
    }

    if ((buf0->y1 - buf0->y0) != (buf1->y1 - buf1->y0))
    {
        return false;
    }

    if (EPIC_COLOR_RGB565 == buf0->format)
    {
        buf0_pixel_size = 2;
        buf0_alpha = false;
    }
    else if (EPIC_COLOR_ARGB8565 == buf0->format)
    {
        buf0_pixel_size = 3;
        buf0_alpha = true;
    }
    else if (EPIC_COLOR_RGB888 == buf0->format)
    {
        buf0_pixel_size = 3;
        buf0_alpha = false;
    }
    else if (EPIC_COLOR_ARGB8888 == buf0->format)
    {
        buf0_pixel_size = 4;
        buf0_alpha = true;
    }
    else
    {
        return false;
    }

    if (EPIC_COLOR_RGB565 == buf1->format)
    {
        buf1_pixel_size = 2;
    }
    else if (EPIC_COLOR_RGB888 == buf1->format)
    {
        buf1_pixel_size = 3;
    }
    else
    {
        return false;
    }

    if (buf0_pixel_size < buf1_pixel_size)
    {
        cmp_pixel_size = buf0_pixel_size;
    }
    else
    {
        cmp_pixel_size = buf1_pixel_size;
    }

    for (i = 0; i < (buf0->y1 - buf0->y0 + 1); i++)
    {
        data0 = buf0->data + buf0_pixel_size * ((buf0->y0 + i) * buf0->total_width + buf0->x0);
        data1 = buf1->data + buf1_pixel_size * ((buf1->y0 + i) * buf1->total_width + buf1->x0);

        for (j = buf0->x0; j <= buf0->x1; j++)
        {
            //LOG_I("buf0:(%d,%d)(%d,%d)", buf0->y0 + i, j, buf1->y0 + i, (j - buf0->x0 + buf1->x0));
            if (EPIC_COLOR_RGB565 == buf1->format)
            {
                fg_color = data0[0] | ((uint32_t)data0[1] << 8);
                check_color = data1[0] | ((uint32_t)data1[1] << 8);
            }
            else
            {
                fg_color = data0[0] | ((uint32_t)data0[1] << 8) | ((uint32_t)data0[2] << 16);
                check_color = data1[0] | ((uint32_t)data1[1] << 8) | ((uint32_t)data1[2] << 16);
            }
            if (buf0_alpha)
            {
                color = calc_blended_color(0, fg_color, data0[cmp_pixel_size], buf1->format);
            }
            else
            {
                color = fg_color;
            }
            if (EPIC_COLOR_RGB565 == buf1->format)
            {
                diff = (int32_t)GET_RGB565_RED(color) - (int32_t)GET_RGB565_RED(check_color);
                if ((diff > 1) || (diff < -1))
                {
                    LOG_I("R[%x][%x]: %x %x %d", &data0[0], &data1[0], color, check_color, diff);
                    return false;
                }
                diff = (int32_t)GET_RGB565_GREEN(color) - (int32_t)GET_RGB565_GREEN(check_color);
                if ((diff > 1) || (diff < -1))
                {
                    LOG_I("G[%x][%x]: %x %x %d", &data0[0], &data1[0], color, check_color, diff);
                    return false;
                }
                diff = (int32_t)GET_RGB565_BLUE(color) - (int32_t)GET_RGB565_BLUE(check_color);
                if ((diff > 1) || (diff < -1))
                {
                    LOG_I("B[%x][%x]: %x %x %d", &data0[0], &data1[0], color, check_color, diff);
                    return false;
                }
            }
            else
            {
                diff = (int32_t)GET_RGB888_RED(color) - (int32_t)GET_RGB888_RED(check_color);
                if ((diff > 1) || (diff < -1))
                {
                    LOG_I("R[%x][%x]: %x %x %d", &data0[0], &data1[0], color, check_color, diff);
                    return false;
                }
                diff = (int32_t)GET_RGB888_GREEN(color) - (int32_t)GET_RGB888_GREEN(check_color);
                if ((diff > 1) || (diff < -1))
                {
                    LOG_I("G[%x][%x]: %x %x %d", &data0[0], &data1[0], color, check_color, diff);
                    return false;
                }
                diff = (int32_t)GET_RGB888_BLUE(color) - (int32_t)GET_RGB888_BLUE(check_color);
                if ((diff > 1) || (diff < -1))
                {
                    LOG_I("B[%x][%x]: %x %x %d", &data0[0], &data1[0], color, check_color, diff);
                    return false;
                }
            }
            data0 += buf0_pixel_size;
            data1 += buf1_pixel_size;
        }
    }

    return true;
}

static void blend_full_ezip_img(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;
    img_buf_cmp_param_t buf0;
    img_buf_cmp_param_t buf1;

    /* foreground image, its coordinate (10,5)~(97,92), resolution is 88*88 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 5;
    fg_img.width = IMG_WIDTH;
    fg_img.height = IMG_HEIGHT;
    fg_img.total_width = IMG_WIDTH;
    fg_img.color_mode = EPIC_COLOR_EZIP;
    fg_img.color_en = false;

    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = OUTPUT_WIDTH;
    bg_img.height = OUTPUT_HEIGHT;
    bg_img.total_width = OUTPUT_WIDTH;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    /* init as black */
    memset(bg_img_buf, 0, sizeof(bg_img_buf));

    HAL_EPIC_BlendDataInit(&output_img);
    output_img.data = bg_img_buf;
    /* output area topleft coordinate */
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    /* output area width */
    output_img.width = OUTPUT_WIDTH;
    /* output area height */
    output_img.height = OUTPUT_HEIGHT;
    /* output buffer width, it's different from output_img.width */
    output_img.total_width = OUTPUT_WIDTH;
    output_img.color_mode = EPIC_COLOR_RGB565;
    output_img.color_en = false;

    /* set complete callback */
    epic_handle.XferCpltCallback = epic_cplt_callback;

    /* no rotation and scaling, opacity 100
     * start EPIC in interrupt mode
     */
    HAL_EPIC_RotDataInit(&trans_cfg);
    trans_cfg.pivot_x = IMG_WIDTH / 2;
    trans_cfg.pivot_y = IMG_HEIGHT / 2;

    epic_done_flag = 0;
    ret = HAL_EPIC_Rotate(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, EPIC_LAYER_OPAQUE);

    LOG_I("blending done");

    buf0.data = cmp_buf;
    buf0.x0 = 0;
    buf0.y0 = 0;
    buf0.x1 = fg_img.width - 1;
    buf0.y1 = fg_img.height - 1;
    buf0.total_width = IMG_WIDTH;
    buf0.total_height = IMG_HEIGHT;
    buf0.format = EPIC_COLOR_ARGB8565;

    buf1.data = bg_img_buf;
    buf1.x0 = fg_img.x_offset;
    buf1.y0 = fg_img.y_offset;
    buf1.x1 = fg_img.x_offset + fg_img.width - 1;
    buf1.y1 = fg_img.y_offset + fg_img.height - 1;
    buf1.total_width = OUTPUT_WIDTH;
    buf1.total_height = OUTPUT_HEIGHT;
    buf1.format = EPIC_COLOR_RGB565;

    if (cmp_data(&buf0, &buf1))
    {
        LOG_I("check done");
    }
    else
    {
        LOG_I("check error");
        uassert_true_ret(false);
    }
}

static void blend_partial_ezip_img(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;
    uint32_t buffer_start_offset;
    img_buf_cmp_param_t buf0;
    img_buf_cmp_param_t buf1;


    /* foreground image, its coordinate (10,5)~(97,92), resolution is 88*88 */
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 5;
    fg_img.width = IMG_WIDTH;
    fg_img.height = IMG_HEIGHT;
    fg_img.total_width = IMG_WIDTH;
    fg_img.color_mode = EPIC_COLOR_EZIP;
    fg_img.color_en = false;

    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    /* init as black */
    memset(bg_img_buf, 0, sizeof(bg_img_buf));

    /* output image, share the same buffer as bg_img_buf,
       output area is (10,5)~(49,54), buffer size is 100*100 */
    /* topleft pixel is (10, 5), skip (5*100+10) pixels */
    buffer_start_offset = (5 - 0) * OUTPUT_WIDTH * PIXEL_SIZE + (10 - 0) * PIXEL_SIZE;
    output_img.data = (uint8_t *)((uint32_t)bg_img_buf + buffer_start_offset);
    /* output area topleft coordinate */
    output_img.x_offset = 10;
    output_img.y_offset = 5;
    /* output area width */
    output_img.width = 40;
    /* output area height */
    output_img.height = 50;
    /* output buffer width, it's different from output_img.width */
    output_img.total_width = OUTPUT_WIDTH;
    output_img.color_mode = EPIC_COLOR_RGB565;
    output_img.color_en = false;

    /* set complete callback */
    epic_handle.XferCpltCallback = epic_cplt_callback;

    /* no rotation and scaling, opacity 100
     * start EPIC in interrupt mode
     */
    HAL_EPIC_RotDataInit(&trans_cfg);
    trans_cfg.pivot_x = IMG_WIDTH / 2;
    trans_cfg.pivot_y = IMG_HEIGHT / 2;

    epic_done_flag = 0;
    ret = HAL_EPIC_Rotate(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, EPIC_LAYER_OPAQUE);

    LOG_I("blending done");

    buf0.data = cmp_buf;
    buf0.x0 = 0;
    buf0.y0 = 0;
    buf0.x1 = output_img.width - 1;
    buf0.y1 = output_img.height - 1;
    buf0.total_width = IMG_WIDTH;
    buf0.total_height = IMG_HEIGHT;
    buf0.format = EPIC_COLOR_ARGB8565;

    buf1.data = bg_img_buf;
    buf1.x0 = fg_img.x_offset;
    buf1.y0 = fg_img.y_offset;
    buf1.x1 = fg_img.x_offset + output_img.width - 1;
    buf1.y1 = fg_img.y_offset + output_img.height - 1;
    buf1.total_width = OUTPUT_WIDTH;
    buf1.total_height = OUTPUT_HEIGHT;
    buf1.format = EPIC_COLOR_RGB565;

    if (cmp_data(&buf0, &buf1))
    {
        LOG_I("check done");
    }
    else
    {
        LOG_I("check error");
        uassert_true_ret(false);
    }
}

static void testcase(int argc, char **argv)
{
    init_epic();
    blend_full_ezip_img();
    blend_partial_ezip_img();
}

UTEST_TC_EXPORT(testcase, "example_ezip", utest_tc_init, utest_tc_cleanup, 10);



#endif /* HAL_EZIP_MODULE_ENABLED */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
