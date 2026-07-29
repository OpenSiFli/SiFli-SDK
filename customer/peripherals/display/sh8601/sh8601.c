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
#include "bf0_hal_lcdc.h"

#define ROW_OFFSET (0x00)
#define COL_OFFSET (0x00)

/**
  * @brief SH8601 chip IDs
  */
#define THE_LCD_ID                  0x009C01

/**
  * @brief  SH8601 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (466)
#define  THE_LCD_PIXEL_HEIGHT   (466)

/**
  * @brief  SH8601 Registers
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

#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...) LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_spi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,                  // 0: QAD-SPI/SPI3   1:SPI4
            .syn_mode = HAL_LCDC_SYNC_VER, //HAL_LCDC_SYNC_DISABLE, // HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            // default_vbp=2, frame rate=82, delay=115us,
            // TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);

//#define SH8601_LCD_TEST
#ifdef SH8601_LCD_TEST
static rt_err_t lcd_init(int argc, char **argv)
{
    SH8601_Init();
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

    DEBUG_PRINTF("\nSH8601_Read reg[%x] %d(byte), result=%08x\n", reg, len, data);

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
    DEBUG_PRINTF("\nSH8601_Write reg[%x] %d(byte) done.\n", reg, argc - 2);

    return 0;
}
MSH_CMD_EXPORT(lcd_wreg, lcd_wreg);
uint8_t SH8601_dsip_mode_value = 0;

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

    /*
        bit 0:  DUAL SPI MODE enable
        bit[5:4] DAUL SPI MODE Selection
                  00: 1P1T for 1 wire
                  10: 1P1T for 2 wire
                  11: 2P3T for 2 wire
                  01: reserve

    */
    if (strcmp(argv[4], "default") == 0)
        SH8601_dsip_mode_value = 0x80;
    else if (strcmp(argv[4], "1p1t") == 0)
        SH8601_dsip_mode_value = 0x81 | (0 << 4);
    else if (strcmp(argv[4], "1p1t_2w") == 0)
        SH8601_dsip_mode_value = 0x81 | (2 << 4);
    else if (strcmp(argv[4], "2p3t_2w") == 0)
        SH8601_dsip_mode_value = 0x81 | (3 << 4);

    lcdc_int_cfg_spi.cfg.spi.dummy_clock = strtoul(argv[5], 0, 10);

    DEBUG_PRINTF("\nlcd_cfg itf=%d, colormode=%d, freq=%d, disp_m=%x\n", lcdc_int_cfg_spi.lcd_itf,
                 lcdc_int_cfg_spi.color_mode,
                 lcdc_int_cfg_spi.freq,
                 SH8601_dsip_mode_value);

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
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_spi, sizeof(lcdc_int_cfg));

    /* Initialize SH8601 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0); // Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    /* Wait for 200ms */
    LCD_DRIVER_DELAY_MS(160);

    LOG_I("SH8601_Init \n");

    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);
    LCD_DRIVER_DELAY_MS(10);

    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x01;
    parameter[3] = 0xD1;
    LCD_WriteReg(hlcdc, 0x2a, parameter, 4);
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x01;
    parameter[3] = 0xD1;
    LCD_WriteReg(hlcdc, 0x2b, parameter, 4);

    parameter[0] = 0x01;
    parameter[1] = 0xD1;
    LCD_WriteReg(hlcdc, 0x44, parameter, 2);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, parameter, 1); // enable TE

    parameter[0] = 0x55;
    LCD_WriteReg(hlcdc, 0x3A, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x51, parameter, 1); // ser brightness

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x4A, parameter, 1); // set back proch

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x63, parameter, 1); // set back proch

    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, 0x53, parameter, 1); // set back proch

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xC4, parameter, 1); // set back proch

    LCD_WriteReg(hlcdc, 0x29, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(80);

    rt_kprintf("sh8610 init finish\r\n");

    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    HAL_LCDC_SetROIArea(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH, THE_LCD_PIXEL_HEIGHT);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
#ifdef LCD_SH8601A_VSYNC_ENABLE
    HAL_LCDC_Next_Frame_TE(hlcdc, 1);
#endif

    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

}

/**
 * @brief  Disables the Display.
 * @param  None
 * @retval LCD Register Value.
 */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;
    /*
        data = LCD_ReadData(hlcdc,REG_CASET, 4);
        DEBUG_PRINTF("\REG_CASET 0x%x \n", data);

        data = LCD_ReadData(hlcdc,REG_RASET, 4);
        DEBUG_PRINTF("\REG_RASET 0x%x \n", data);
    */
    data = LCD_ReadData(hlcdc, REG_LCD_ID, 2);
    DEBUG_PRINTF("\r\n lcd_error  SH8601_ReadID 0x%x \n", data);
    //RT_ASSERT(data);
    data = THE_LCD_ID; // GC FAE: NOT support read id now
    return data;
}

/**
 * @brief  Enables the Display.
 * @param  None
 * @retval None
 */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("lcd_debug  SH8601_DisplayOn\r\n");
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
    rt_kprintf("lcd_debug run SH8601_DisplayOff\r\n");
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
    //rt_kprintf("lcd_debug SH8601_WriteMultiplePixels:%d,y0:%d,x1:%d,y1:%d\n",Xpos0,Ypos0,Xpos1,Ypos1);
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
        DEBUG_PRINTF("SH8601_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("SH8601_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
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

    DEBUG_PRINTF("SH8601 NOT support read pixel!");

    return 0;

    DEBUG_PRINTF("SH8601_ReadPixel[%d,%d]\n", Xpos, Ypos);

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

    /*
       (8bit R + 3bit dummy + 8bit G + 3bit dummy + 8bit B)

    */
    case LCDC_PIXEL_FORMAT_RGB888:
        ret_v = (uint32_t)(((r << 16) & 0xFF0000) | ((g << 8) & 0xFF00) | ((b) & 0XFF));
        break;

    default:
        RT_ASSERT(0);
        break;
    }

    // LCD_WriteReg(hlcdc,REG_COLOR_MODE, parameter, 1);

    return ret_v;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t parameter[2];

    /*

    Control interface color format
    ??011?? = 12bit/pixel ??101?? = 16bit/pixel ??110?? = 18bit/pixel ??111?? = 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x75;
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

    uint32_t data = LCD_ReadData(hlcdc, 0xc, 1);
    DEBUG_PRINTF("\nSH8601_color_format 0x%x \n", data);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define SH8601_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)SH8601_BRIGHTNESS_MAX * br / 100);
    rt_kprintf("lcd_debug run  SH8601_SetBrightness br = %d, bright = %d\r\n");
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}

static const LCD_DrvOpsDef SH8601_drv =
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

LCD_DRIVER_EXPORT2(SH8601, THE_LCD_ID, &lcdc_int_cfg,
                   &SH8601_drv, 1);

