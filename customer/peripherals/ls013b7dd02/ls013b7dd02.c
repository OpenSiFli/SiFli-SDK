/**
  ******************************************************************************
  * @file   ls013b7dd02.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for ls013b7dd02 LCD.
  * @attention
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
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#include "log.h"









#ifdef ROW_OFFSET_PLUS
    #define ROW_OFFSET  (ROW_OFFSET_PLUS)
#else
    #define ROW_OFFSET  (0)
#endif




/**
  * @brief LS013B7DD02 chip IDs
  */
#define THE_LCD_ID                  0x1d1

/**
  * @brief  LS013B7DD02 Size
  */
#define  THE_LCD_PIXEL_WIDTH    ((uint16_t)260)
#define  THE_LCD_PIXEL_HEIGHT   ((uint16_t)260)






/**
  * @brief  LS013B7DD02 Registers
  */
#define REG_LCD_ID             0x04
#define REG_SLEEP_IN           0x10
#define REG_SLEEP_OUT          0x11
#define REG_PARTIAL_DISPLAY    0x12
#define REG_DISPLAY_INVERSION  0x21
#define REG_DISPLAY_OFF        0x28
#define REG_DISPLAY_ON         0x29
#define REG_WRITE_RAM          0x2C
#define REG_READ_RAM           0x2E
#define REG_CASET              0x2A
#define REG_RASET              0x2B


#define REG_TEARING_EFFECT     0x35

#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A
#define REG_WBRIGHT            0x51






#define REG_VDV_VRH_EN         0xC2
#define REG_VDV_SET            0xC4



























//#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif







static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_JDI_PARALLEL,
    .freq = 60,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .jdi = {
            .bank_col_head = 0,
            .valid_columns = THE_LCD_PIXEL_WIDTH,
            .bank_col_tail = 8,

            .bank_row_head = 0,
            .valid_rows = THE_LCD_PIXEL_HEIGHT,
            .bank_row_tail = 4,
        },
    },

};










/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    /* Initialize LS013B7DD02 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    HAL_Delay_us(10);
    BSP_LCD_Reset(1);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    return THE_LCD_ID;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
    //LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
#ifdef SF32LB58X
//HAL_LCDC_JDI_LP_CLK
    hwp_lptim3->ARR = 32768 / lcdc_int_cfg.freq;
    hwp_lptim3->CMP = hwp_lptim3->ARR / 2;
    hwp_lptim3->CR |= LPTIM_CR_ENABLE;
    hwp_lptim3->CR |= LPTIM_CR_CNTSTRT;


    MODIFY_REG(hwp_lpsys_aon->CR1, LPSYS_AON_CR1_PBR_SEL0_Msk, 2 << LPSYS_AON_CR1_PBR_SEL0_Pos);
    MODIFY_REG(hwp_lpsys_aon->CR1, LPSYS_AON_CR1_PBR_SEL1_Msk, 3 << LPSYS_AON_CR1_PBR_SEL1_Pos);

    hwp_rtc->PBR3R |= 3 << RTC_PBR3R_SEL_Pos; //frp
    hwp_rtc->PBR4R |= 2 << RTC_PBR4R_SEL_Pos; //xfrp
    hwp_rtc->PBR5R |= 3 << RTC_PBR5R_SEL_Pos; //vcom
#endif /* SF32LB58X */
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
    //LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    HAL_LCDC_SetROIArea(hlcdc, 0, Ypos0, THE_LCD_PIXEL_WIDTH - 1, Ypos1); //Not support partical columns
}




static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
}






static const LCD_DrvOpsDef LS013B7DD02_drv =
{
    LCD_Init,
    LCD_ReadID,
    LCD_DisplayOn,
    LCD_DisplayOff,

    LCD_SetRegion,
    NULL,
    LCD_WriteMultiplePixels,

    NULL,

    NULL,
    NULL,
    NULL,
    NULL
};


LCD_DRIVER_EXPORT2(ls013b7dd02, THE_LCD_ID, &lcdc_int_cfg,
                   &LS013B7DD02_drv, 1);











/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
