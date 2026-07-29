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
  * @brief gc9307 chip IDs
  */
#define THE_LCD_ID                  0x85

/**
  * @brief  gc9307 Registers
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

static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_SPI_DCX_1DATA,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
#ifdef LCD_GC9307_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
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
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    rt_kprintf("LCD_Init entry!!\n");
    /* Initialize gc9307 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);
    LCD_DRIVER_DELAY_MS(120);

    //GC9307_SpiWriteCmd(0xfe);
    LCD_WriteReg(hlcdc, 0xfe, (uint8_t *)NULL, 0);
    //GC9307_SpiWriteCmd(0xef);
    LCD_WriteReg(hlcdc, 0xef, (uint8_t *)NULL, 0);

//  GC9307_SpiWriteCmd(0x36);
//  GC9307_SpiWriteData(0x48);
    parameter[0] = 0x48;
    LCD_WriteReg(hlcdc, 0x36, parameter, 1);

    //GC9307_SpiWriteCmd(0x3a);
    //GC9307_SpiWriteData(0x05);
    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0x3a, parameter, 1);

    //GC9307_SpiWriteCmd(0x84);
    //GC9307_SpiWriteData(0x40);
    parameter[0] = 0x40;
    LCD_WriteReg(hlcdc, 0x84, parameter, 1);

    //GC9307_SpiWriteCmd(0x86);
    //GC9307_SpiWriteData(0x98);
    parameter[0] = 0x98;
    LCD_WriteReg(hlcdc, 0x86, parameter, 1);

    //GC9307_SpiWriteCmd(0x89);
    //GC9307_SpiWriteData(0x03);
    parameter[0] = 0x03;
    LCD_WriteReg(hlcdc, 0x89, parameter, 1);

    //GC9307_SpiWriteCmd(0x8b);
    //GC9307_SpiWriteData(0x80);
    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0x8b, parameter, 1);

    //GC9307_SpiWriteCmd(0x8d);
    //GC9307_SpiWriteData(0x33);
    parameter[0] = 0x33;
    LCD_WriteReg(hlcdc, 0x8d, parameter, 1);

    //GC9307_SpiWriteCmd(0x8e);
    //GC9307_SpiWriteData(0x0f);
    parameter[0] = 0x0f;
    LCD_WriteReg(hlcdc, 0x8e, parameter, 1);

    //GC9307_SpiWriteCmd(0xB6);
    //GC9307_SpiWriteData(0x00);
    //GC9307_SpiWriteData(0x00);
    //GC9307_SpiWriteData(0x23);
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x23;
    LCD_WriteReg(hlcdc, 0xB6, parameter, 3);

    //GC9307_SpiWriteCmd(0xe8);
    //GC9307_SpiWriteData(0x14);
    //GC9307_SpiWriteData(0x57);
    parameter[0] = 0x14;
    parameter[1] = 0x57;
    LCD_WriteReg(hlcdc, 0xe8, parameter, 2);

    //GC9307_SpiWriteCmd(0xff);
    //GC9307_SpiWriteData(0x62);
    parameter[0] = 0x62;
    LCD_WriteReg(hlcdc, 0xff, parameter, 1);

    //GC9307_SpiWriteCmd(0x99);
    //GC9307_SpiWriteData(0x3e);
    parameter[0] = 0x3e;
    LCD_WriteReg(hlcdc, 0x99, parameter, 1);

    //GC9307_SpiWriteCmd(0x9d);
    //GC9307_SpiWriteData(0x4b);
    parameter[0] = 0x4b;
    LCD_WriteReg(hlcdc, 0x9d, parameter, 1);

    //GC9307_SpiWriteCmd(0x98);
    //GC9307_SpiWriteData(0x3e);
    parameter[0] = 0x3e;
    LCD_WriteReg(hlcdc, 0x98, parameter, 1);

    //GC9307_SpiWriteCmd(0x9c);
    //GC9307_SpiWriteData(0x4b);
    parameter[0] = 0x4b;
    LCD_WriteReg(hlcdc, 0x9c, parameter, 1);

    //GC9307_SpiWriteCmd(0xc3);
    //GC9307_SpiWriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xc3, parameter, 1);

    //GC9307_SpiWriteCmd(0xc4);
    //GC9307_SpiWriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xc4, parameter, 1);

    //GC9307_SpiWriteCmd(0xc9);
    //GC9307_SpiWriteData(0x05);
    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0xc9, parameter, 1);

    //GC9307_SpiWriteCmd(0xF0);
    //GC9307_SpiWriteData(0xc7);
    //GC9307_SpiWriteData(0x08);
    //GC9307_SpiWriteData(0x08);
    //GC9307_SpiWriteData(0x08);
    //GC9307_SpiWriteData(0x05);
    //GC9307_SpiWriteData(0x2e);
    parameter[0] = 0xc7;
    parameter[1] = 0x08;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    parameter[4] = 0x05;
    parameter[5] = 0x2e;
    LCD_WriteReg(hlcdc, 0xF0, parameter, 6);

    //GC9307_SpiWriteCmd(0xF1);
    //GC9307_SpiWriteData(0x45);
    //GC9307_SpiWriteData(0x8f);
    //GC9307_SpiWriteData(0x6f);
    //GC9307_SpiWriteData(0x33);
    //GC9307_SpiWriteData(0x36);
    //GC9307_SpiWriteData(0x4f);
    parameter[0] = 0x45;
    parameter[1] = 0x8f;
    parameter[2] = 0x6f;
    parameter[3] = 0x33;
    parameter[4] = 0x36;
    parameter[5] = 0x4f;
    LCD_WriteReg(hlcdc, 0xF1, parameter, 6);

    //GC9307_SpiWriteCmd(0xF2);
    //GC9307_SpiWriteData(0xc7);
    //GC9307_SpiWriteData(0x08);
    //GC9307_SpiWriteData(0x08);
    //GC9307_SpiWriteData(0x08);
    //GC9307_SpiWriteData(0x05);
    //GC9307_SpiWriteData(0x2e);
    parameter[0] = 0xc7;
    parameter[1] = 0x08;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    parameter[4] = 0x05;
    parameter[5] = 0x2e;
    LCD_WriteReg(hlcdc, 0xF2, parameter, 6);

    //GC9307_SpiWriteCmd(0xF3);
    //GC9307_SpiWriteData(0x45);
    //GC9307_SpiWriteData(0x8f);
    //GC9307_SpiWriteData(0x6f);
    //GC9307_SpiWriteData(0x33);
    //GC9307_SpiWriteData(0x36);
    //GC9307_SpiWriteData(0x4f);
    parameter[0] = 0x45;
    parameter[1] = 0x8f;
    parameter[2] = 0x6f;
    parameter[3] = 0x33;
    parameter[4] = 0x36;
    parameter[5] = 0x4f;
    LCD_WriteReg(hlcdc, 0xF3, parameter, 6);
    ////GC9307_SpiWriteCmd(0xE9);
    ////GC9307_SpiWriteData(0x08);//2DATA
    //parameter[0] = 0x08;
    //LCD_WriteReg(hlcdc, 0xE9, parameter, 1); //2DATA

#if 1 //GC9307+BOE1.83IPS(WV018LZQ-N80-3QP1)-C.txt  
    //GC9307_SpiWriteCmd(0x8E);
    //GC9307_SpiWriteData(0xFF);
    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x8E, parameter, 1);

    //GC9307_SpiWriteCmd(0xA7);
    //GC9307_SpiWriteData(0x58);
    parameter[0] = 0x58;
    LCD_WriteReg(hlcdc, 0xA7, parameter, 1);
#endif
    //GC9307_SpiWriteCmd(0x35);
    //GC9307_SpiWriteData(0x00);//TE
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, REG_TEARING_EFFECT, parameter, 1);     //TE

    //GC9307_SpiWriteCmd(0x44);
    //GC9307_SpiWriteData(0x00);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x44, parameter, 1);

    //GC9307_SpiWriteData(0x00);//TE scan line
    LCD_WriteReg(hlcdc, 0x00, (uint8_t *)NULL, 0);
    //GC9307_SpiWriteCmd(0x11);
    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);

    //mdelay(120);
    LCD_DRIVER_DELAY_MS(120);
    //GC9307_SpiWriteCmd(0x29);
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
    //GC9307_SpiWriteCmd(0x2c);
    LCD_WriteReg(hlcdc, 0x2c, (uint8_t *)NULL, 0);

    rt_kprintf("LCD_Init over!!\n");

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
    rt_kprintf(" gc9307_ReadID:0x%x\n", data);
    //gc9307_ReadID:0x40c0d9ff
    data = ((data << 1) >> 8) & 0xFFFFFF;
    rt_kprintf(" gc9307_ReadID1:0x%x\n", data); //gc9307_ReadID1:0x9307

    return 0x8181b3;
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

    rt_kprintf("LCD_SetRegion X0:%d,Y0:0x%d,X1:%d,Y1:%d\n", Xpos0, Ypos0, Xpos1, Ypos1);

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

    rt_kprintf("RGBCode:0x%x,X0:%d,Y0:0x%d,X1:%d,Y1:%d\n", RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
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
           pretend that gc9307 can support RGB888,

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

    //rt_kprintf("gc9307_ReadPixel %x -> %x\n",c, ret_v);

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

static const LCD_DrvOpsDef gc9307_drv =
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

LCD_DRIVER_EXPORT2(gc9307, 0x8181b3, &lcdc_int_cfg,
                   &gc9307_drv, 2);

