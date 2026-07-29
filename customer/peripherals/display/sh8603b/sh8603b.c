/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-FileCopyrightText: 2025 Another <amir1387aht@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#include "log.h"
#include "bf0_hal_lcdc.h"


#define ROW_OFFSET 0x00
#define COL_OFFSET 0x00


/**
  * @brief SH8603B chip IDs - Updated based on datasheet
  */
#define THE_LCD_ID                  0x009C03  // Updated for SH8603B

/**
  * @brief  SH8603B Size
  */
#define  THE_LCD_PIXEL_WIDTH    410
#define  THE_LCD_PIXEL_HEIGHT   494


/**
  * @brief  SH8603B Registers - Updated based on datasheet
  */
#define REG_SW_RESET           0x01
#define REG_LCD_ID             0x04
#define REG_DSI_ERR            0x05
#define REG_POWER_MODE         0x0A
#define REG_SLEEP_IN           0x10
#define REG_SLEEP_OUT          0x11

#define REG_DISPLAY_OFF        0x28
#define REG_DISPLAY_ON         0x29
#define REG_WRITE_RAM          0x2C
#define REG_READ_RAM           0x2E
#define REG_CASET              0x2A
#define REG_RASET              0x2B

#define REG_TEARING_EFFECT     0x35
#define REG_WRITE_TEAR_SCANLINE 0x44

#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/

/* Manufacturer Command Set Protection */
#define REG_PASSWD1            0xF0
#define REG_PAGE_REG           0xFF

/* Page specific commands */
#define REG_DISP_CTRL          0xB0  /* Page 4 */
#define REG_PWR_CTRL1          0xB0  /* Page 5 */


#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...) LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif


#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_spi =
{
    .lcd_itf = QAD_SPI_ITF,
    .freq = 60000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
            .syn_mode = HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);

//#define SH8603B_LCD_TEST
#ifdef SH8603B_LCD_TEST
static rt_err_t lcd_init(int argc, char **argv)
{
    SH8603B_Init();
    return 0;
}

MSH_CMD_EXPORT(lcd_init, lcd_init);

static rt_err_t lcd_rreg(int argc, char **argv)
{
    uint32_t data;

    uint16_t i, reg, len;

    reg = strtoul(argv[1], 0, 16);
    len = strtoul(argv[2], 0, 16);

    data = LCD_ReadData(reg, len);

    DEBUG_PRINTF("\nSH8603B_Read reg[%x] %d(byte), result=%08x\n", reg, len, data);

    return 0;
}
MSH_CMD_EXPORT(lcd_rreg, lcd_rreg);

static rt_err_t lcd_wreg(int argc, char **argv)
{
    uint8_t parameter[4];

    uint32_t data;
    uint16_t reg, i;

    reg = strtoul(argv[1], 0, 16);

    for (i = 2; i < argc; i++)
    {
        parameter[i - 2] = strtoul(argv[i], 0, 16);
    }

    LCD_WriteReg(hlcdc, reg, parameter, argc - 2);
    DEBUG_PRINTF("\nSH8603B_Write reg[%x] %d(byte) done.\n", reg, argc - 2);

    return 0;
}
MSH_CMD_EXPORT(lcd_wreg, lcd_wreg);
uint8_t SH8603B_dsip_mode_value = 0;

static rt_err_t lcd_cfg(int argc, char **argv)
{

    if (strcmp(argv[1], "nodcx1d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_1DATA;
    else if (strcmp(argv[1], "nodcx2d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_2DATA;
    else if (strcmp(argv[1], "nodcx4d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_4DATA;

    else if (strcmp(argv[1], "dcx1d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_1DATA;
    else if (strcmp(argv[1], "dcx2d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_2DATA;
    else if (strcmp(argv[1], "dcx4d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_4DATA;

    if (strcmp(argv[2], "rgb565") == 0)
        lcdc_int_cfg_spi.color_mode = LCDC_PIXEL_FORMAT_RGB565;
    else if (strcmp(argv[2], "rgb888") == 0)
        lcdc_int_cfg_spi.color_mode = LCDC_PIXEL_FORMAT_RGB888;

    lcdc_int_cfg_spi.freq = 1000000 * strtoul(argv[3], 0, 10);

    if (strcmp(argv[4], "default") == 0)
        SH8603B_dsip_mode_value = 0x80;
    else if (strcmp(argv[4], "1p1t") == 0)
        SH8603B_dsip_mode_value = 0x81 | (0 << 4);
    else if (strcmp(argv[4], "1p1t_2w") == 0)
        SH8603B_dsip_mode_value = 0x81 | (2 << 4);
    else if (strcmp(argv[4], "2p3t_2w") == 0)
        SH8603B_dsip_mode_value = 0x81 | (3 << 4);

    lcdc_int_cfg_spi.cfg.spi.dummy_clock = strtoul(argv[5], 0, 10);

    DEBUG_PRINTF("\nlcd_cfg itf=%d, colormode=%d, freq=%d, disp_m=%x\n", lcdc_int_cfg_spi.lcd_itf,
                 lcdc_int_cfg_spi.color_mode,
                 lcdc_int_cfg_spi.freq,
                 SH8603B_dsip_mode_value);

    return 0;
}

MSH_CMD_EXPORT(lcd_cfg, lcd_cfg);
#endif /* DSI_TEST */


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
            HAL_LCDC_SetFreq(hlcdc, 2000000); // read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq); // Restore normal frequency
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
    uint8_t parameter[32];

    // Copy LCD configuration
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_spi, sizeof(lcdc_int_cfg));
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    // Reset sequence
    BSP_LCD_Reset(0);        // Hardware reset low
    LCD_DRIVER_DELAY_MS(10); // Wait
    BSP_LCD_Reset(1);        // Hardware reset high
    LCD_DRIVER_DELAY_MS(160);// Wait for LCD to be ready

    LOG_I("SH8603B_Init \n");

    // Sleep out
    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, NULL, 0);
    LCD_DRIVER_DELAY_MS(120);

    // Enter Manufacturer Command Set - Password
    parameter[0] = 0xA5;
    parameter[1] = 0xA5;
    LCD_WriteReg(hlcdc, REG_PASSWD1, parameter, 2);

    // Select Page 4 for display control
    parameter[0] = 0x5A;  // Page password
    parameter[1] = 0x04;  // Page 4
    LCD_WriteReg(hlcdc, REG_PAGE_REG, parameter, 2);

    // Display Control (Page 4, B0h) - Set frame size
    // Configure for 410x494 display
    parameter[0] = 0x01;  // Video mode, etc.
    parameter[1] = 0x20;  // Color mode settings
    parameter[2] = 0x24;  // HBP/HFP
    parameter[3] = 0x05;  // Clock divider
    parameter[4] = 0x01;  // Frame size V[9:8]
    parameter[5] = 0xEE;  // Frame size V[7:0] = 494
    parameter[6] = 0x01;  // Frame size H[9:8]
    parameter[7] = 0x9A;  // Frame size H[7:0] = 410
    LCD_WriteReg(hlcdc, REG_DISP_CTRL, parameter, 8);

    // Column address set (0x2A)
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x01;
    parameter[3] = 0x99;  // 409 (410-1)
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

    // Page address set (0x2B)
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x01;
    parameter[3] = 0xED;  // 493 (494-1)
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

    // Tearing Effect Scan Line (0x44)
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    LCD_WriteReg(hlcdc, REG_WRITE_TEAR_SCANLINE, parameter, 2);

    // TE Mode (0x35)
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, REG_TEARING_EFFECT, parameter, 1);

    // Interface Pixel Format (0x3A)
    parameter[0] = 0x55;  // 16 bits/pixel (RGB565)
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    // Brightness (0x51)
    parameter[0] = 0xFF;  // Max brightness
    LCD_WriteReg(hlcdc, REG_WBRIGHT, parameter, 1);

    // Display Control (0x53) - Write Display Brightness
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, 0x53, parameter, 1);

    // Exit Manufacturer Command Set
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    LCD_WriteReg(hlcdc, REG_PASSWD1, parameter, 2);

    parameter[0] = 0x00;//00:60hz - 01:55hz - 02:50hz
    LCD_WriteReg(hlcdc, 0x91, parameter, 1); // set frame rate

    // Display ON
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, NULL, 0);
    LCD_DRIVER_DELAY_MS(80);

    rt_kprintf("SH8603B_Init_Finish\r\n");

    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    HAL_LCDC_SetROIArea(hlcdc, 0, 0, 410, 494);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

#ifdef LCD_SH8603B_VSYNC_ENABLE
    HAL_LCDC_Next_Frame_TE(hlcdc, 1);
#endif

    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, NULL, 0);
}


/**
 * @brief  Disables the Display.
 * @param  None
 * @retval LCD Register Value.
 */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    DEBUG_PRINTF("\r\n lcd_error  SH8603B_ReadID 0x%x \n", data);

    data = THE_LCD_ID; // Return expected ID for SH8603B
    return data;
}

/**
 * @brief  Enables the Display.
 * @param  None
 * @retval None
 */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("lcd_debug  SH8603B_DisplayOn\r\n");
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
    rt_kprintf("lcd_debug run SH8603B_DisplayOff\r\n");
    LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t parameter[4];

    HAL_LCDC_SetROIArea(hlcdc, Xpos0, Ypos0, Xpos1, Ypos1);

    Xpos0 += COL_OFFSET;
    Xpos1 += COL_OFFSET;

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
    //rt_kprintf("lcd_debug SH8603B_WriteMultiplePixels:%d,y0:%d,x1:%d,y1:%d\n",Xpos0,Ypos0,Xpos1,Ypos1);
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    }
    else
    {
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, REG_WRITE_RAM, 1);
    }
}

/**
 * @brief  Writes  to the selected LCD register.
 * @param  LCD_Reg: address of the selected register.
 * @retval None
 */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
#if 0
    if (LCD_Reg == REG_CASET)
    {
        DEBUG_PRINTF("SH8603B_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("SH8603B_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
#endif

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        uint32_t cmd;

        cmd = (0x02 << 24) | (LCD_Reg << 8);

        if (0 != NbParameters)
        {
            /* Send command's parameters if any */
            HAL_LCDC_WriteU32Reg(hlcdc, cmd, Parameters, NbParameters);
        }
        else
        {
            uint32_t v = 0;
            HAL_LCDC_WriteU32Reg(hlcdc, cmd, (uint8_t *)&v, 1);
        }
    }
    else
    {
        HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
    }
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
    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);
    }
    else
    {
        HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);
    }
    LCD_ReadMode(hlcdc, false);
    return rd_data;
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    uint8_t r, g, b;
    uint32_t ret_v, read_value;

    DEBUG_PRINTF("SH8603B NOT support read pixel!");

    return 0;

    DEBUG_PRINTF("SH8603B_ReadPixel[%d,%d]\n", Xpos, Ypos);

    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);

    read_value = LCD_ReadData(hlcdc, REG_READ_RAM, 4);
    DEBUG_PRINTF("result: [%x]\n", read_value);

    b = (read_value >> 0) & 0xFF;
    g = (read_value >> 8) & 0xFF;
    r = (read_value >> 16) & 0xFF;

    DEBUG_PRINTF("r=%d, g=%d, b=%d \n", r, g, b);

    switch (lcdc_int_cfg.color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB565:
        ret_v = (uint32_t)(((r << 11) & 0xF800) | ((g << 5) & 0x7E0) | ((b >> 3) & 0X1F));
        break;

    case LCDC_PIXEL_FORMAT_RGB888:
        ret_v = (uint32_t)(((r << 16) & 0xFF0000) | ((g << 8) & 0xFF00) | ((b) & 0XFF));
        break;

    default:
        RT_ASSERT(0);
        break;
    }

    return ret_v;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t parameter[2];

    /*
    Control interface color format
    '011' = 12bit/pixel '101' = 16bit/pixel '110' = 18bit/pixel '111' = 16M truncated
    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x55;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0x77;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; // unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    uint32_t data = LCD_ReadData(hlcdc, 0x0C, 1);
    DEBUG_PRINTF("\nSH8603B_color_format 0x%x \n", data);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define SH8603B_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)SH8603B_BRIGHTNESS_MAX * br / 100);
    rt_kprintf("lcd_debug run  SH8603B_SetBrightness br = %d, bright = %d\r\n", br, bright);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}


static const LCD_DrvOpsDef SH8603B_drv =
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

LCD_DRIVER_EXPORT2(SH8603B, THE_LCD_ID, &lcdc_int_cfg, &SH8603B_drv, 1);