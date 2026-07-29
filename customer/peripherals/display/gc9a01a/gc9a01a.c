/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
  * @brief GC9A01A chip IDs
  */
#define THE_LCD_ID                  0x85

/**
  * @brief  GC9A01A Size
  */
#define  THE_LCD_PIXEL_WIDTH    ((uint16_t)240)
#define  THE_LCD_PIXEL_HEIGHT   ((uint16_t)240)

/**
  * @brief  GC9A01A Registers
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
#define REG_NORMAL_DISPLAY     0x36
#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A
#define REG_WBRIGHT            0x51

#define REG_PORCH_CTRL         0xB2

#define REG_GATE_CTRL          0xB7
#define REG_VCOM_SET           0xBB
#define REG_LCM_CTRL           0xC0
#define REG_VDV_VRH_EN         0xC2
#define REG_VDV_SET            0xC4

#define REG_FR_CTRL            0xC6
#define REG_POWER_CTRL         0xD0
#define REG_PV_GAMMA_CTRL      0xE0
#define REG_NV_GAMMA_CTRL      0xE1
#define REG_SPI2EN             0xE7

//#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_SPI_DCX_1DATA,
    .freq = 60000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
#ifdef LCD_GC9A01A_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
            .readback_from_Dx = 0,
        },
    },

};

/**
  * @brief  spi read/write mode
  * @param  enable: false - write spi mode |  true - read spi mode
  * @retval None
  */
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable)
{
    if (HAL_LCDC_IS_SPI_IF(lcdc_int_cfg.lcd_itf))
    {
        if (enable)
        {
            HAL_LCDC_SetFreq(hlcdc, 4000000); //read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq); //Restore normal frequency
        }

    }
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    rt_kprintf("GC9A01A_Init entry!!\n");
    /* Initialize GC9A01A low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(1);//Reset LCD
    LCD_DRIVER_DELAY_MS(20);
    BSP_LCD_Reset(0);
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    //GC9A01_HSD1.28_SPI4W_240x240
    LCD_DRIVER_DELAY_MS(100);
    LCD_WriteReg(hlcdc, 0xEF, (uint8_t *)NULL, 0);
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0xEB, parameter, 1);

    LCD_WriteReg(hlcdc, 0xFE, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, 0xEF, (uint8_t *)NULL, 0);
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0xEB, parameter, 1);

    parameter[0] = 0x40;
    LCD_WriteReg(hlcdc, 0x84, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x85, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x86, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x87, parameter, 1);

    parameter[0] = 0x0A;
    LCD_WriteReg(hlcdc, 0x88, parameter, 1);

    parameter[0] = 0x21;
    LCD_WriteReg(hlcdc, 0x89, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x8A, parameter, 1);

    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0x8B, parameter, 1);

    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x8C, parameter, 1);

    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x8D, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x8E, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x8F, parameter, 1);

    parameter[0] = 0x00;
    parameter[1] = 0x20;
    LCD_WriteReg(hlcdc, 0xB6, parameter, 2);

    parameter[0] = 0x08;
    LCD_WriteReg(hlcdc, 0x36, parameter, 1);

    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0x3A, parameter, 1);

    parameter[0] = 0x08;
    parameter[1] = 0x08;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    LCD_WriteReg(hlcdc, 0x90, parameter, 4);

    parameter[0] = 0x06;
    LCD_WriteReg(hlcdc, 0xBD, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xBC, parameter, 1);

    parameter[0] = 0x60;
    parameter[1] = 0x01;
    parameter[2] = 0x04;
    LCD_WriteReg(hlcdc, 0xFF, parameter, 3);

    parameter[0] = 0x13;
    LCD_WriteReg(hlcdc, 0xC3, parameter, 1);
    parameter[0] = 0x13;
    LCD_WriteReg(hlcdc, 0xC4, parameter, 1);

    parameter[0] = 0x22;
    LCD_WriteReg(hlcdc, 0xC9, parameter, 1);

    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xBE, parameter, 1);

    parameter[0] = 0x10;
    parameter[1] = 0x0E;
    LCD_WriteReg(hlcdc, 0xE1, parameter, 2);

    parameter[0] = 0x21;
    parameter[1] = 0x0c;
    parameter[2] = 0x02;
    LCD_WriteReg(hlcdc, 0xDF, parameter, 3);

    parameter[0] = 0x45;
    parameter[1] = 0x09;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    parameter[4] = 0x26;
    parameter[5] = 0x2A;
    LCD_WriteReg(hlcdc, 0xF0, parameter, 6);

    parameter[0] = 0x43;
    parameter[1] = 0x70;
    parameter[2] = 0x72;
    parameter[3] = 0x36;
    parameter[4] = 0x37;
    parameter[5] = 0x6F;
    LCD_WriteReg(hlcdc, 0xF1, parameter, 6);

    parameter[0] = 0x45;
    parameter[1] = 0x09;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    parameter[4] = 0x26;
    parameter[5] = 0x2A;
    LCD_WriteReg(hlcdc, 0xF2, parameter, 6);

    parameter[0] = 0x43;
    parameter[1] = 0x70;
    parameter[2] = 0x72;
    parameter[3] = 0x36;
    parameter[4] = 0x37;
    parameter[5] = 0x6F;
    LCD_WriteReg(hlcdc, 0xF3, parameter, 6);

    parameter[0] = 0x1B;
    parameter[1] = 0x0B;
    LCD_WriteReg(hlcdc, 0xED, parameter, 2);

    parameter[0] = 0x77;
    LCD_WriteReg(hlcdc, 0xAE, parameter, 1);

    parameter[0] = 0x63;
    LCD_WriteReg(hlcdc, 0xCD, parameter, 1);

    parameter[0] = 0x07;
    parameter[1] = 0x07;
    parameter[2] = 0x04;
    parameter[3] = 0x0E;
    parameter[4] = 0x0F;
    parameter[5] = 0x09;
    parameter[6] = 0x07;
    parameter[7] = 0x08;
    parameter[8] = 0x03;
    LCD_WriteReg(hlcdc, 0x70, parameter, 9);

    parameter[0] = 0x34;
    LCD_WriteReg(hlcdc, 0xE8, parameter, 1);

    parameter[0] = 0x18;
    parameter[1] = 0x0D;
    parameter[2] = 0x71;
    parameter[3] = 0xED;
    parameter[4] = 0x70;
    parameter[5] = 0x70;
    parameter[6] = 0x18;
    parameter[7] = 0x0F;
    parameter[8] = 0x71;
    parameter[9] = 0xEF;
    parameter[10] = 0x70;
    parameter[11] = 0x70;
    LCD_WriteReg(hlcdc, 0x62, parameter, 12);

    parameter[0] = 0x18;
    parameter[1] = 0x11;
    parameter[2] = 0x71;
    parameter[3] = 0xF1;
    parameter[4] = 0x70;
    parameter[5] = 0x70;
    parameter[6] = 0x18;
    parameter[7] = 0x13;
    parameter[8] = 0x71;
    parameter[9] = 0xF3;
    parameter[10] = 0x70;
    parameter[11] = 0x70;
    LCD_WriteReg(hlcdc, 0x63, parameter, 12);

    parameter[0] = 0x28;
    parameter[1] = 0x29;
    parameter[2] = 0xF1;
    parameter[3] = 0x01;
    parameter[4] = 0xF1;
    parameter[5] = 0x00;
    parameter[6] = 0x07;
    LCD_WriteReg(hlcdc, 0x64, parameter, 7);

    parameter[0] = 0x3C;
    parameter[1] = 0x00;
    parameter[2] = 0xCD;
    parameter[3] = 0x67;
    parameter[4] = 0x45;
    parameter[5] = 0x45;
    parameter[6] = 0x10;
    parameter[7] = 0x00;
    parameter[8] = 0x00;
    parameter[9] = 0x00;
    LCD_WriteReg(hlcdc, 0x66, parameter, 10);

    parameter[0] = 0x00;
    parameter[1] = 0x3C;
    parameter[2] = 0x00;
    parameter[3] = 0x00;
    parameter[4] = 0x00;
    parameter[5] = 0x01;
    parameter[6] = 0x54;
    parameter[7] = 0x10;
    parameter[8] = 0x32;
    parameter[9] = 0x98;
    LCD_WriteReg(hlcdc, 0x67, parameter, 10);

    parameter[0] = 0x10;
    parameter[1] = 0x85;
    parameter[2] = 0x80;
    parameter[3] = 0x00;
    parameter[4] = 0x00;
    parameter[5] = 0x4E;
    parameter[6] = 0x00;
    LCD_WriteReg(hlcdc, 0x74, parameter, 7);

    parameter[0] = 0x3e;
    parameter[1] = 0x07;
    LCD_WriteReg(hlcdc, 0x98, parameter, 2);

    LCD_WriteReg(hlcdc, 0x35, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, 0x21, (uint8_t *)NULL, 0);

    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);
    LCD_DRIVER_DELAY_MS(50);

    LCD_WriteReg(hlcdc, 0x29, (uint8_t *)NULL, 0);
    LCD_DRIVER_DELAY_MS(60);

//    /* Sleep In Command */
//    LCD_WriteReg(hlcdc, REG_SLEEP_IN, (uint8_t *)NULL, 0);
//    /* Wait for 10ms */
//    LCD_DRIVER_DELAY_MS(10);

//    /* SW Reset Command */
//    LCD_WriteReg(hlcdc, 0x01, (uint8_t *)NULL, 0);
//    /* Wait for 200ms */
//    LCD_DRIVER_DELAY_MS(200);

//    /* Sleep Out Command */
//    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
//    /* Wait for 120ms */
//    LCD_DRIVER_DELAY_MS(120);

//    /* Normal display for Driver Down side */
//    parameter[0] = 0x00;
//    LCD_WriteReg(hlcdc, REG_NORMAL_DISPLAY, parameter, 1);

//    /* Color mode 16bits/pixel */
//    parameter[0] = 0x55;
//    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

//    /* Display inversion On */
//    LCD_WriteReg(hlcdc, REG_DISPLAY_INVERSION, (uint8_t *)NULL, 0);

#if 0
    /* set LCD RAM buffer to black */
    /* Set Column address CASET */
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    parameter[3] = THE_LCD_PIXEL_WIDTH - 1;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);
    /* Set Row address RASET */
    parameter[0] = 0x00;
    parameter[1] = ROW_OFFSET;
    parameter[2] = ((THE_LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) >> 8) & 0xFF;
    parameter[3] = ((THE_LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) & 0xFF);
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)NULL, 0);

    for (uint32_t i = 0; i < THE_LCD_PIXEL_WIDTH * THE_LCD_PIXEL_WIDTH; i++)
    {
        LCD_IO_WriteData16(0);
    }
#endif

//    /* Enable SPI 2 data lane mode */
//    parameter[0] = 0x10;
//    LCD_WriteReg(hlcdc, REG_SPI2EN, parameter, 1);

#if 0

    /*--------------- GC9A01A Frame rate setting -------------------------------*/
    /* PORCH control setting */
    parameter[0] = 0x0C;
    parameter[1] = 0x0C;
    parameter[2] = 0x00;
    parameter[3] = 0x33;
    parameter[4] = 0x33;
    LCD_WriteReg(hlcdc, REG_PORCH_CTRL, parameter, 5);

    /* GATE control setting */
    parameter[0] = 0x35;
    LCD_WriteReg(hlcdc, REG_GATE_CTRL, parameter, 1);

    /*--------------- GC9A01A Power setting ------------------------------------*/
    /* VCOM setting */
    parameter[0] = 0x1F;
    LCD_WriteReg(hlcdc, REG_VCOM_SET, parameter, 1);

    /* LCM Control setting */
    parameter[0] = 0x2C;
    LCD_WriteReg(hlcdc, REG_LCM_CTRL, parameter, 1);

    /* VDV and VRH Command Enable */
    parameter[0] = 0x01;
    parameter[1] = 0xC3;
    LCD_WriteReg(hlcdc, REG_VDV_VRH_EN, parameter, 2);

    /* VDV Set */
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, REG_VDV_SET, parameter, 1);

    /* Frame Rate Control in normal mode */
    parameter[0] = 0x0F;
    LCD_WriteReg(hlcdc, REG_FR_CTRL, parameter, 1);

    /* Power Control */
    parameter[0] = 0xA4;
    parameter[1] = 0xA1;
    LCD_WriteReg(hlcdc, REG_POWER_CTRL, parameter, 1);

#endif

//    TODO: configuration can be different though the LCD chip is same
//#ifdef LCD_USING_ROUND_TYPE1

//    /*--------------- GC9A01A Gamma setting ------------------------------------*/
//    /* Positive Voltage Gamma Control */
//    parameter[0] = 0x70;
//    parameter[1] = 0x04;
//    parameter[2] = 0x08;
//    parameter[3] = 0x09;
//    parameter[4] = 0x09;
//    parameter[5] = 0x05;
//    parameter[6] = 0x2A;
//    parameter[7] = 0x33;
//    parameter[8] = 0x41;
//    parameter[9] = 0x07;
//    parameter[10] = 0x13;
//    parameter[11] = 0x13;
//    parameter[12] = 0x29;
//    parameter[13] = 0x2F;
//    LCD_WriteReg(hlcdc, REG_PV_GAMMA_CTRL, parameter, 14);

//    /* Negative Voltage Gamma Control */
//    parameter[0] = 0x70;
//    parameter[1] = 0x03;
//    parameter[2] = 0x09;
//    parameter[3] = 0x0A;
//    parameter[4] = 0x09;
//    parameter[5] = 0x06;
//    parameter[6] = 0x2B;
//    parameter[7] = 0x34;
//    parameter[8] = 0x41;
//    parameter[9] = 0x07;
//    parameter[10] = 0x12;
//    parameter[11] = 0x14;
//    parameter[12] = 0x28;
//    parameter[13] = 0x2E;
//    LCD_WriteReg(hlcdc, REG_NV_GAMMA_CTRL, parameter, 14);

//#else

//    /*--------------- GC9A01A Gamma setting ------------------------------------*/
//    /* Positive Voltage Gamma Control */
//    parameter[0] = 0xD0;
//    parameter[1] = 0x13;
//    parameter[2] = 0x1A;
//    parameter[3] = 0x0A;
//    parameter[4] = 0x0A;
//    parameter[5] = 0x26;
//    parameter[6] = 0x3F;
//    parameter[7] = 0x54;
//    parameter[8] = 0x54;
//    parameter[9] = 0x18;
//    parameter[10] = 0x14;
//    parameter[11] = 0x14;
//    parameter[12] = 0x30;
//    parameter[13] = 0x33;
//    LCD_WriteReg(hlcdc, REG_PV_GAMMA_CTRL, parameter, 14);

//    /* Negative Voltage Gamma Control */
//    parameter[0] = 0xD0;
//    parameter[1] = 0x13;
//    parameter[2] = 0x1A;
//    parameter[3] = 0x0A;
//    parameter[4] = 0x0A;
//    parameter[5] = 0x26;
//    parameter[6] = 0x3F;
//    parameter[7] = 0x54;
//    parameter[8] = 0x54;
//    parameter[9] = 0x1A;
//    parameter[10] = 0x16;
//    parameter[11] = 0x16;
//    parameter[12] = 0x32;
//    parameter[13] = 0x35;
//    LCD_WriteReg(hlcdc, REG_NV_GAMMA_CTRL, parameter, 14);
//#endif

#if 0

    /* Set Column address CASET */
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    parameter[3] = THE_LCD_PIXEL_WIDTH - 1;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);
    /* Set Row address RASET */
    parameter[0] = 0x00;
    parameter[1] = ROW_OFFSET;
    parameter[2] = ((THE_LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) >> 8) & 0xFF;
    parameter[3] = ((THE_LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) & 0xFF);
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)NULL, 0);

    hwp_lcdc->CANVAS_BG = (0xFF << LCD_IF_CANVAS_BG_RED_Pos) | (0 << LCD_IF_CANVAS_BG_GREEN_Pos) | (0 << LCD_IF_CANVAS_BG_BLUE_Pos);
    hwp_lcdc->CANVAS_TL_POS = (0    << LCD_IF_CANVAS_TL_POS_X0_Pos) | (0  << LCD_IF_CANVAS_TL_POS_Y0_Pos);
    hwp_lcdc->CANVAS_BR_POS = (239  << LCD_IF_CANVAS_BR_POS_X1_Pos) | ((239) << LCD_IF_CANVAS_BR_POS_Y1_Pos);

    hwp_lcdc->COMMAND = 0x1;
#endif

//    /* Set frame control, refresh rate: 82Hz*/
//    parameter[0] = 0x7;
//    LCD_WriteReg(hlcdc, REG_FR_CTRL, parameter, 1);

//    /* Tearing Effect Line On: Option (00h:VSYNC Only, 01h:VSYNC & HSYNC ) */
//    parameter[0] = 0x00;
//    LCD_WriteReg(hlcdc, REG_TEARING_EFFECT, parameter, 1);

//    /* Display ON command */
//    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
//  rt_kprintf("GC9A01A_Init end!!\n");

}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 4);
    rt_kprintf(" GC9A01A_ReadID:0x%x\n", data);
    //GC9A01A_ReadID:0x40c0d9ff
    data = 0x8181b3;
    rt_kprintf(" GC9A01A_ReadID1:0x%x\n", data);

    return data;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
    LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t   parameter[4];

//  rt_kprintf("LCD_SetRegion X0:%d,Y0:0x%d,X1:%d,Y1:%d\n", Xpos0, Ypos0, Xpos1, Ypos1);

    HAL_LCDC_SetROIArea(hlcdc, Xpos0, Ypos0, Xpos1, Ypos1);

    Ypos0 += ROW_OFFSET;
    Ypos1 += ROW_OFFSET;

    parameter[0] = (Xpos0) >> 8;
    parameter[1] = (Xpos0) & 0xFF;
    parameter[2] = (Xpos1) >> 8;
    parameter[3] = (Xpos1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

    parameter[0] = (Ypos0) >> 8;
    parameter[1] = (Ypos0) & 0xFF;
    parameter[2] = (Ypos1) >> 8;
    parameter[3] = (Ypos1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);
}

/**
  * @brief  Writes pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
static void LCD_WritePixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos, const uint8_t *RGBCode)
{
    uint8_t data = 0;

    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

//  rt_kprintf("RGBCode:0x%x,X0:%d,Y0:0x%d,X1:%d,Y1:%d\n",RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData2Reg_IT(hlcdc, REG_WRITE_RAM, 1);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
    HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize)
{
    uint32_t rd_data = 0;

    LCD_ReadMode(hlcdc, true);

    HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);

    LCD_ReadMode(hlcdc, false);

    return rd_data;
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    uint8_t   parameter[2];
    uint32_t c;
    uint32_t ret_v;

    parameter[0] = 0x66;
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);

    /*
        read ram need 7 dummy cycle, and it's result is 24bit color which format is:

        6bit red + 2bit dummy + 6bit green + 2bit dummy + 6bit blue + 2bit dummy

    */
    c =  LCD_ReadData(hlcdc, REG_READ_RAM, 4);
    c <<= 1;
    //c >>= lcdc_int_cfg.dummy_clock; //revert fixed dummy cycle

    switch (lcdc_int_cfg.color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB565:
        parameter[0] = 0x55;
        ret_v = (uint32_t)(((c >> 8) & 0xF800) | ((c >> 5) & 0x7E0) | ((c >> 3) & 0X1F));
        break;

    case LCDC_PIXEL_FORMAT_RGB666:
        parameter[0] = 0x66;
        ret_v = (uint32_t)(((c >> 6) & 0x3F000) | ((c >> 4) & 0xFC0) | ((c >> 2) & 0X3F));
        break;

    case LCDC_PIXEL_FORMAT_RGB888:
        /*
           pretend that GC9A01A can support RGB888,

           treated as RGB666 actually(6bit R + 2bit dummy + 6bit G + 2bit dummy + 6bit B + 2bit dummy )

           lcdc NOT support RGB666
        */
        /*
            st7789 NOT support RGB888：

            fill 2bit dummy bit with 2bit MSB of color
        */
        parameter[0] = 0x66;
        ret_v = (uint32_t)((c & 0xFCFCFC) | ((c >> 6) & 0x030303));
        break;

    default:
        RT_ASSERT(0);
        break;
    }

    //rt_kprintf("GC9A01A_ReadPixel %x -> %x\n",c, ret_v);

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    return ret_v;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];

    /*

    Control interface color format
    ‘011’ = 12bit/pixel ‘101’ = 16bit/pixel ‘110’ = 18bit/pixel ‘111’ = 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x55;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    default:
        RT_ASSERT(0);
        return; //unsupport
        break;

    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);
    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((uint16_t)UINT8_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &br, 1);
}

static const LCD_DrvOpsDef GC9A01A_drv =
{
    LCD_Init,
    LCD_ReadID,
    LCD_DisplayOn,
    LCD_DisplayOff,

    LCD_SetRegion,
    LCD_WritePixel,
    LCD_WriteMultiplePixels,

    LCD_ReadPixel,

    LCD_SetColorMode,
    LCD_SetBrightness,
    NULL,
    NULL

};

LCD_DRIVER_EXPORT2(GC9A01A, 0x8181b3, &lcdc_int_cfg,
                   &GC9A01A_drv, 1);

