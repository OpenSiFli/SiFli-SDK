/**
  ******************************************************************************
  * @file   drv_lcd_private.c
  * @author Sifli software development team
  * @brief  drv_lcd & drv_ram_lcd common codes.
  *
* *****************************************************************************
**/
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
#include <rtdevice.h>
#include "drv_lcd.h"
#include "drv_io.h"
#include "drv_common.h"
#include "drv_lcd_private.h"
#include "drv_ext_dma.h"
#include "mem_section.h"
#include "string.h"

#define  DBG_LEVEL            DBG_ERROR  //DBG_LOG //
#define LOG_TAG                "drv.lcdp"
#include "log.h"


extern LCDC_HandleTypeDef *get_drv_lcd_handler(void);
#ifdef APP_BSP_TEST
    extern LCDC_HandleTypeDef *get_ram_lcd_handler(void);
#endif /* APP_BSP_TEST */


#ifdef BSP_USING_LCDC

void LCDC1_IRQHandler(void)
{
    LCDC_HandleTypeDef *lcdc = NULL;

    rt_interrupt_enter();

#ifdef APP_BSP_TEST
    lcdc = get_ram_lcd_handler();
#endif /* APP_BSP_TEST */

    if (NULL == lcdc)
        lcdc =  get_drv_lcd_handler();

    RT_ASSERT(RT_NULL != lcdc);
    HAL_LCDC_IRQHandler(lcdc);

    if (lcdc->ErrorCode)
    {
        LOG_E("LCDC error code %x", lcdc->ErrorCode);

        lcdc->ErrorCode = 0;
    }
    rt_interrupt_leave();
}

void LCDC2_IRQHandler(void)
{
    LCDC_HandleTypeDef *lcdc;

    rt_interrupt_enter();

    lcdc =  get_drv_lcd_handler();
    RT_ASSERT(RT_NULL != lcdc);

    HAL_LCDC_IRQHandler(lcdc);
    rt_interrupt_leave();
}

#ifdef BSP_LCDC_USING_RAMLESS_QADSPI

void GPTIM1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_RAMLESS_LCD_GPTIM_A_IRQHandler();
    rt_interrupt_leave();
}


void GPTIM2_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_RAMLESS_LCD_GPTIM_B_IRQHandler();
    rt_interrupt_leave();
}

#endif /* BSP_LCDC_USING_RAMLESS_QADSPI */
#endif


#if defined(BSP_USING_RAMLESS_LCD) \
    || (defined(LCDC_SUPPORT_LINE_DONE_IRQ) && defined(SF32LB55X))
void PTC1_IRQHandler(void)
{
    LCDC_HandleTypeDef *lcdc;

    rt_interrupt_enter();

    lcdc =  get_drv_lcd_handler();
    RT_ASSERT(RT_NULL != lcdc);

    HAL_LCDC_PTC_IRQHandler(lcdc);
    rt_interrupt_leave();
}
#endif


#ifdef HAL_DSI_MODULE_ENABLED

void DSIHOST_IRQHandler(void)
{
    LCDC_HandleTypeDef *lcdc;

    rt_interrupt_enter();
    lcdc =  get_drv_lcd_handler();
    RT_ASSERT(RT_NULL != lcdc);

    HAL_DSI_IRQHandler(&lcdc->hdsi);
    rt_interrupt_leave();
}


void HAL_DSI_ErrorCallback(DSI_HandleTypeDef *hdsi)
{
    LOG_E("DSI error code %x", hdsi->ErrorCode);
}

#endif /* HAL_DSI_MODULE_ENABLED */


HAL_LCDC_PixelFormat rt_lcd_format_to_hal_lcd_format(uint16_t rt_color_format)
{
    switch (rt_color_format)
    {
    case RTGRAPHIC_PIXEL_FORMAT_MONO:
        return LCDC_PIXEL_FORMAT_MONO;
    case RTGRAPHIC_PIXEL_FORMAT_RGB332:
        return LCDC_PIXEL_FORMAT_RGB332;
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        return LCDC_PIXEL_FORMAT_RGB565;
    case RTGRAPHIC_PIXEL_FORMAT_RGB666:
        return LCDC_PIXEL_FORMAT_RGB666;
    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        return LCDC_PIXEL_FORMAT_RGB888;
    case RTGRAPHIC_PIXEL_FORMAT_ARGB888:
        return LCDC_PIXEL_FORMAT_ARGB888;
    case RTGRAPHIC_PIXEL_FORMAT_ARGB565:
        return LCDC_PIXEL_FORMAT_ARGB565;

    case RTGRAPHIC_PIXEL_FORMAT_A8:
        return LCDC_PIXEL_FORMAT_A8;

    case RTGRAPHIC_PIXEL_FORMAT_L8:
        return LCDC_PIXEL_FORMAT_L8;

    default:
        RT_ASSERT(0); //unknow format
        return 0;
    }

}



rt_err_t rt_lcd_layer_control(LCD_DrvTypeDef *p_drv_lcd, int cmd, void *args)
{
    switch (cmd)
    {
    case RTGRAPHIC_CTRL_SELECT_LAYER:
        if (args) p_drv_lcd->select_layer = *((HAL_LCDC_LayerDef *)args);
        break;

    case RTGRAPHIC_CTRL_DISABLE_LAYER:
        if (args) HAL_LCDC_LayerDisable(&p_drv_lcd->hlcdc, *((HAL_LCDC_LayerDef *)args));
        break;

    case RTGRAPHIC_CTRL_ENABLE_LAYER:
        if (args) HAL_LCDC_LayerEnable(&p_drv_lcd->hlcdc, *((HAL_LCDC_LayerDef *)args));
        break;

    case RTGRAPHIC_CTRL_SET_BG_COLOR:
        if (args)
        {
            LCDC_AColorDef c = *((LCDC_AColorDef *)args);

            HAL_LCDC_SetBgColor(&p_drv_lcd->hlcdc, c.r, c.g, c.b);
        }
        break;

    case RTGRAPHIC_CTRL_SET_BUF_FORMAT:
        if (args)
        {
            uint16_t new_color_format = *((uint16_t *)args);

            p_drv_lcd->buf_format = new_color_format;
            HAL_LCDC_LayerSetFormat(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer,
                                    rt_lcd_format_to_hal_lcd_format(new_color_format));
        }
        break;


    case RTGRAPHIC_CTRL_SET_LAYER:
        if (args)
        {
            graphic_layer_info_t *p_cfg = (graphic_layer_info_t *)args;
            if (p_cfg->cmpr_rate > 0)
                HAL_LCDC_LayerSetCmpr(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, p_cfg->cmpr_rate);
            else
                HAL_LCDC_LayerSetCmpr(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, 0);


            p_drv_lcd->buf_format = p_cfg->color_format;
            HAL_LCDC_LayerSetFormat(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer,
                                    rt_lcd_format_to_hal_lcd_format(p_cfg->color_format));

            if (rt_lcd_format_to_hal_lcd_format(p_cfg->color_format) == LCDC_PIXEL_FORMAT_L8)
            {
                HAL_LCDC_LayerSetLTab(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, (const LCDC_AColorDef *) p_cfg->pL8_Tab);
            }
            else if (rt_lcd_format_to_hal_lcd_format(p_cfg->color_format) == LCDC_PIXEL_FORMAT_A8)
            {
                HAL_LCDC_LayerSetBgColor(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, p_cfg->A8_Bg.r, p_cfg->A8_Bg.g, p_cfg->A8_Bg.b);
            }


            HAL_LCDC_LayerRotate(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, p_cfg->rotate);

            HAL_LCDC_LayerSetDataExt(&p_drv_lcd->hlcdc,  p_drv_lcd->select_layer, (uint8_t *)p_cfg->pixel, p_cfg->x0, p_cfg->y0, p_cfg->x1, p_cfg->y1, p_cfg->total_width);
        }
        break;



    /**********************************************/
    /******************Test only  START**********/
    /**********************************************/
    case SF_GRAPHIC_CTRL_LCDC_LAYER_BUF_CMPR_RATE:
        if (args)
        {
            uint8_t cmpr_rate = *((uint8_t *)args);

            if (cmpr_rate)
                HAL_LCDC_LayerSetCmpr(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, cmpr_rate);
            else
                HAL_LCDC_LayerSetCmpr(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, 0);
        }
        break;

    case SF_GRAPHIC_CTRL_LCDC_LAYER_INFO_SET:
        if (args)
        {
            sf_graphic_lcdc_layer_t *p_cfg = (sf_graphic_lcdc_layer_t *)args;
            if (p_cfg->cmpr_rate > 0)
                HAL_LCDC_LayerSetCmpr(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, p_cfg->cmpr_rate);
            else
                HAL_LCDC_LayerSetCmpr(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, 0);


            p_drv_lcd->buf_format = p_cfg->color_format;
            HAL_LCDC_LayerSetFormat(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer,
                                    rt_lcd_format_to_hal_lcd_format(p_cfg->color_format));

            if (rt_lcd_format_to_hal_lcd_format(p_cfg->color_format) == LCDC_PIXEL_FORMAT_L8)
            {
                HAL_LCDC_LayerSetLTab(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, (const LCDC_AColorDef *) p_cfg->pL8_Tab);
            }
            else if (rt_lcd_format_to_hal_lcd_format(p_cfg->color_format) == LCDC_PIXEL_FORMAT_A8)
            {
                HAL_LCDC_LayerSetBgColor(&p_drv_lcd->hlcdc, p_drv_lcd->select_layer, p_cfg->A8_Bg.r, p_cfg->A8_Bg.g, p_cfg->A8_Bg.b);
            }
        }
        break;
    /**********************************************/
    /******************Test only  END************/
    /**********************************************/

    default:
        LOG_E("Unknow cmd %d", cmd);
        RT_ASSERT(0);
        break;
    }

    return RT_EOK;
}

