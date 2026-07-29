/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"
#include "log.h"
#include "string.h"
#include <rtthread.h>

#ifdef ROW_OFFSET_PLUS
    #define ROW_OFFSET (ROW_OFFSET_PLUS)
#else
    #define ROW_OFFSET (0)
#endif

/**
 * @brief gc9107 chip IDs
 */
#define THE_LCD_ID 0x009107

/**
 * @brief  gc9107 Registers
 */
#define REG_LCD_ID 0x04
#define REG_SLEEP_IN 0x10
#define REG_SLEEP_OUT 0x11
#define REG_PARTIAL_DISPLAY 0x12
#define REG_DISPLAY_INVERSION 0x21
#define REG_DISPLAY_OFF 0x28
#define REG_DISPLAY_ON 0x29
#define REG_WRITE_RAM 0x2C
#define REG_READ_RAM 0x2E
#define REG_CASET 0x2A
#define REG_RASET 0x2B
#define REG_TEARING_EFFECT 0x35
#define REG_IDLE_MODE_OFF 0x38
#define REG_IDLE_MODE_ON 0x39
#define REG_COLOR_MODE 0x3A
#define REG_VDV_VRH_EN 0xC2
#define REG_VDV_SET 0xC4

// #define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINTF(...) LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg,
                         uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue,
                             uint8_t ReadSize);

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_SPI_DCX_1DATA,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg =
    {
        .spi =
        {
            .dummy_clock = 0,
#ifdef LCD_GC9107_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif
            .vsyn_polarity = 1,
            // default_vbp=2, frame rate=82, delay=115us,
            // TODO: use us to define delay instead of cycle,
            // delay_cycle=115*48
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
            HAL_LCDC_SetFreq(hlcdc, 4000000); // read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc,
                             lcdc_int_cfg.freq); // Restore normal frequency
        }
    }
}

static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[14];

    rt_kprintf("LCD_Init entry!!\n");
    /* Initialize gc9107 low level bus layer
     * ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0); // Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);
    LCD_DRIVER_DELAY_MS(120);

    /* Sleep Out Command */
    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
    /* Wait for 120ms */
    LCD_DRIVER_DELAY_MS(120);

    LCD_WriteReg(hlcdc, 0xFE, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, 0xEF, (uint8_t *)NULL, 0);

    parameter[0] = 0xC0;
    LCD_WriteReg(hlcdc, 0xB0, parameter, 1);

    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0xB1, parameter, 1);

    parameter[0] = 0x27;
    LCD_WriteReg(hlcdc, 0xB2, parameter, 1);

    parameter[0] = 0x13;
    LCD_WriteReg(hlcdc, 0xB3, parameter, 1);

    parameter[0] = 0x19;
    LCD_WriteReg(hlcdc, 0xB6, parameter, 1);

    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0xB7, parameter, 1);

    parameter[0] = 0xC8;
    LCD_WriteReg(hlcdc, 0xAC, parameter, 1);

    parameter[0] = 0x0F;
    LCD_WriteReg(hlcdc, 0xAB, parameter, 1);

    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0x3A, parameter, 1);

    parameter[0] = 0x04;
    LCD_WriteReg(hlcdc, 0xB4, parameter, 1);

    parameter[0] = 0x08;
    LCD_WriteReg(hlcdc, 0xA8, parameter, 1);

    parameter[0] = 0x08;
    LCD_WriteReg(hlcdc, 0xB8, parameter, 1);

    parameter[0] = 0x02;
    LCD_WriteReg(hlcdc, 0xEA, parameter, 1);

    parameter[0] = 0x2A;
    LCD_WriteReg(hlcdc, 0xE8, parameter, 1);

    parameter[0] = 0x47;
    LCD_WriteReg(hlcdc, 0xE9, parameter, 1);

    parameter[0] = 0x5F;
    LCD_WriteReg(hlcdc, 0xE7, parameter, 1);

    parameter[0] = 0x21;
    LCD_WriteReg(hlcdc, 0xC6, parameter, 1);

    parameter[0] = 0x15;
    LCD_WriteReg(hlcdc, 0xC7, parameter, 1);

    /*--------------- GC9107 Gamma setting
     * ------------------------------------*/
    /* Positive Voltage Gamma Control */
    parameter[0] = 0x1d;
    parameter[1] = 0x38;
    parameter[2] = 0x09;
    parameter[3] = 0x4D;
    parameter[4] = 0x92;
    parameter[5] = 0x2F;
    parameter[6] = 0x35;
    parameter[7] = 0x52;
    parameter[8] = 0x1E;
    parameter[9] = 0x0C;
    parameter[10] = 0x04;
    parameter[11] = 0x12;
    parameter[12] = 0x14;
    parameter[13] = 0x1f;
    LCD_WriteReg(hlcdc, 0xf0, parameter, 14);

    /* Negative Voltage Gamma Control */
    parameter[0] = 0x16;
    parameter[1] = 0x40;
    parameter[2] = 0x1c;
    parameter[3] = 0x54;
    parameter[4] = 0xa9;
    parameter[5] = 0x2d;
    parameter[6] = 0x2e;
    parameter[7] = 0x56;
    parameter[8] = 0x10;
    parameter[9] = 0x0d;
    parameter[10] = 0x0c;
    parameter[11] = 0x1a;
    parameter[12] = 0x14;
    parameter[13] = 0x1e;
    LCD_WriteReg(hlcdc, 0xf1, parameter, 14);

    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0xff;

    LCD_WriteReg(hlcdc, 0xF4, parameter, 3);

    parameter[0] = 0xff;
    parameter[1] = 0xff;
    LCD_WriteReg(hlcdc, 0xba, parameter, 2);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0X36, parameter, 1);

    LCD_WriteReg(hlcdc, 0X11, (uint8_t *)NULL, 0);
    // mdelay(120);
    LCD_DRIVER_DELAY_MS(120);
    // st7789v3_SpiWriteCmd(0x29);
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
    // st7789v3_SpiWriteCmd(0x2c);
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
    return 0x009107;
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

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0,
                          uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t parameter[4];

    // rt_kprintf("LCD_SetRegion X0:%d,Y0:0x%d,X1:%d,Y1:%d\n", Xpos0, Ypos0,
    // Xpos1, Ypos1);

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
static void LCD_WritePixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos,
                           uint16_t Ypos, const uint8_t *RGBCode)
{
    uint8_t data = 0;

    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc,
                                    const uint8_t *RGBCode, uint16_t Xpos0,
                                    uint16_t Ypos0, uint16_t Xpos1,
                                    uint16_t Ypos1)
{
    uint32_t size;

    // rt_kprintf("RGBCode:0x%x,X0:%d,Y0:0x%d,X1:%d,Y1:%d\n", RGBCode, Xpos0,
    // Ypos0, Xpos1, Ypos1);
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode,
                          Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData2Reg_IT(hlcdc, REG_WRITE_RAM, 1);
}

/**
 * @brief  Writes  to the selected LCD register.
 * @param  LCD_Reg: address of the selected register.
 * @retval None
 */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg,
                         uint8_t *Parameters, uint32_t NbParameters)
{
    HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
}

/**
 * @brief  Reads the selected LCD Register.
 * @param  RegValue: Address of the register to read
 * @param  ReadSize: Number of bytes to read
 * @retval LCD Register Value.
 */
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue,
                             uint8_t ReadSize)
{
    uint32_t rd_data = 0;

    LCD_ReadMode(hlcdc, true);

    HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);

    LCD_ReadMode(hlcdc, false);

    return rd_data;
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos,
                              uint16_t Ypos)
{
    uint8_t parameter[2];
    uint32_t c;
    uint32_t ret_v;

    parameter[0] = 0x66;
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);

    /*
        read ram need 7 dummy cycle, and it's result is 24bit color which format
       is:

        6bit red + 2bit dummy + 6bit green + 2bit dummy + 6bit blue + 2bit dummy

    */
    c = LCD_ReadData(hlcdc, REG_READ_RAM, 4);
    c <<= 1;
    // c >>= lcdc_int_cfg.dummy_clock; //revert fixed dummy cycle

    switch (lcdc_int_cfg.color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB565:
        parameter[0] = 0x55;
        ret_v = (uint32_t)(((c >> 8) & 0xF800) | ((c >> 5) & 0x7E0) |
                           ((c >> 3) & 0X1F));
        break;

    case LCDC_PIXEL_FORMAT_RGB666:
        parameter[0] = 0x66;
        ret_v = (uint32_t)(((c >> 6) & 0x3F000) | ((c >> 4) & 0xFC0) |
                           ((c >> 2) & 0X3F));
        break;

    case LCDC_PIXEL_FORMAT_RGB888:
        /*
           pretend that gc9107 can support RGB888,

           treated as RGB666 actually(6bit R + 2bit dummy + 6bit G + 2bit dummy
           + 6bit B + 2bit dummy )

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
    // rt_kprintf("gc9107_ReadPixel %x -> %x\n",c, ret_v);
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    return ret_v;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t parameter[2];
    /*

    Control interface color format
    ‘011’ = 12bit/pixel ‘101’ = 16bit/pixel ‘110’ = 18bit/pixel ‘111’ = 16M
    truncated

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
        return; // unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);
    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    rt_device_t device = rt_device_find("lcdlight");
    if (device)
    {
        rt_err_t err = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
        uint8_t val = br;
        rt_device_write(device, 0, &val, 1);
        rt_device_close(device);
    }
    else
    {
        LOG_E("Can't find device lcdlight!");
    }
}

static const LCD_DrvOpsDef gc9107_drv = {LCD_Init,
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

LCD_DRIVER_EXPORT2(gc9107, 0x009107, &lcdc_int_cfg, &gc9107_drv, 2);

