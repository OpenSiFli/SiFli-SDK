/**
  ******************************************************************************
  * @file   example_epic.c
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

#ifdef HAL_EPIC_MODULE_ENABLED

/* Example Description:
 *
 * blend foreground image(white color) with background image(black color),
 * update pixel color in background image buffer,
 * foregound image size is 50*60, area (10,20)~(59,79),
 * background size is 100*100, area (0,0)~(99,99),
 * blending area is (5,10)~(44,59), color in blending area intersects with foreground area changes to blended color 0x630C in RGB565,
 * other pixels in background image buffer are still color black
 *
 * background (100*100)
 * (0,0)
 * +---------+-------------+
 * |                       |
 * |   (10,20)             |
 * |     +-----------+     |
 * |    | foreground |     |
 * |    |            |     |
 * |    +------------+     |
 * |              (59,79)  |
 * |                       |
 * +-----------------------+
 *                        (99,99)
 *
 *
 */


#define IMG_WIDTH        (50)
#define IMG_HEIGHT       (60)
#define OUTPUT_WIDTH     (100)
#define OUTPUT_HEIGHT    (100)
/* RGB565, 2bytes each pixel */
#define PIXEL_SIZE       (2)


static EPIC_HandleTypeDef epic_handle;
#ifdef HAL_EZIP_MODULE_ENABLED
    static EZIP_HandleTypeDef ezip_handle;
#endif /* HAL_EZIP_MODULE_ENABLED */

volatile static uint8_t epic_done_flag;
ALIGN(4)
uint8_t fg_img_buf[IMG_WIDTH * IMG_HEIGHT * PIXEL_SIZE];
ALIGN(4)
uint8_t bg_img_buf[OUTPUT_WIDTH * OUTPUT_HEIGHT * PIXEL_SIZE];

#ifndef hwp_ezip
    #define hwp_ezip hwp_ezip1
#endif


void EPIC_IRQHandler(void)
{
    ENTER_INTERRUPT();

    HAL_EPIC_IRQHandler(&epic_handle);

    LEAVE_INTERRUPT();
}


static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

void init_epic(void)
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

void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    /* set flag to indicate epic operation done */
    epic_done_flag = 1;
}

/* blend the foreground with background image using 100 opacity (0 is transparent, 255 is opaque)
 * output buffer is same as background image buffer, usually they're both frame buffer.
 *
 */
void blend_img(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;
    uint32_t buffer_start_offset;

    /* foreground image, its coordinate (10,20)~(59,79), buffer size is 50*60 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    fg_img.width = 50;
    fg_img.height = 60;
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    /* init as white */
    memset(fg_img_buf, 0xFF, sizeof(fg_img_buf));

    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
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
       output area is (5,10)~(44,59), buffer size is 100*100 */
    /* topleft pixel is (5, 10), skip (10*100+5) pixels */
    HAL_EPIC_BlendDataInit(&output_img);
    buffer_start_offset = (10 - 0) * OUTPUT_WIDTH * PIXEL_SIZE + (5 - 0) * PIXEL_SIZE;
    output_img.data = (uint8_t *)((uint32_t)bg_img_buf + buffer_start_offset);
    /* output area topleft coordinate */
    output_img.x_offset = 5;
    output_img.y_offset = 10;
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

    epic_done_flag = 0;
    ret = HAL_EPIC_Rotate_IT(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, 100);

    /* wait for complete */
    while (0 == epic_done_flag)
    {
    }

    LOG_I("blending done");
}


static void testcase(int argc, char **argv)
{
    init_epic();
    blend_img();
}

UTEST_TC_EXPORT(testcase, "example_epic", utest_tc_init, utest_tc_cleanup, 10);


#endif /* HAL_EPIC_MODULE_ENABLED */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
