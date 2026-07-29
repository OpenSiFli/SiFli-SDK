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
#include "bf0_hal.h"

#define ROW_OFFSET  (0x00)
//#define COL_OFFSET  (0x00)
#define COL_OFFSET  (0x0E)  //xulin modify

/**
  * @brief ICN3311 chip IDs
  */
#define THE_LCD_ID                  0x1133

/**
  * @brief  ICN3311 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (454)
#define  THE_LCD_PIXEL_HEIGHT   (454)

/**
  * @brief  ICN3311 Registers
  */
#define REG_SW_RESET           0x01
#define REG_LCD_ID             0x04
#define REG_DSI_ERR            0x05
#define REG_POWER_MODE         0x0A
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

#define REG_WBRIGHT            0x51 /* Write brightness*/

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*icn3311 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) //((x) = (x) & (~1))
#define LCD_ALIGN1(x) //((x) = (0 == ((x) & 1)) ? (x - 1) : x)

#ifdef BSP_LCDC_USING_DSI

static const LCDC_InitTypeDef lcdc_int_cfg_dsi =
{
    .lcd_itf = LCDC_INTF_DSI,
    .freq = DSI_FREQ_480MHZ, //ICN3311 RGB565 only support 320Mbps,  RGB888 support 500Mbps
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {

        .dsi = {

            .Init = {
                .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,
                .NumberOfLanes = DSI_ONE_DATA_LANE,
                .TXEscapeCkdiv = 0x4,
            },

            .CmdCfg = {
                .VirtualChannelID      = 0,
                .CommandSize           = 0xFFFF,
#if 0//def LCD_ICN3311_VSYNC_ENABLE
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE,     //Open TE
#else
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Close TE
#endif /* LCD_ICN3311_VSYNC_ENABLE */
            },

            .PhyTimings = {
                .ClockLaneHS2LPTime = 35,
                .ClockLaneLP2HSTime = 35,
                .DataLaneHS2LPTime = 35,
                .DataLaneLP2HSTime = 35,
                .DataLaneMaxReadTime = 0,
                .StopWaitTime = 0,
            },

            .HostTimeouts = {
                .TimeoutCkdiv = 1,
                .HighSpeedTransmissionTimeout = 0,
                .LowPowerReceptionTimeout = 0,
                .HighSpeedReadTimeout = 0,
                .LowPowerReadTimeout = 0,
                .HighSpeedWriteTimeout = 0,
                //.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE,
                .LowPowerWriteTimeout = 0,
                .BTATimeout = 0,
            },

            .LPCmd = {
                .LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE,
                .LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE,
                .LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE,
                .LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE,
                .LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE,
                .LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE,
                .LPGenLongWrite        = DSI_LP_GLW_ENABLE,
                .LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE,
                .LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE,
                .LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE,
                .LPDcsLongWrite        = DSI_LP_DLW_DISABLE,
                .LPMaxReadPacket       = DSI_LP_MRDP_ENABLE,
                .AcknowledgeRequest    = DSI_ACKNOWLEDGE_DISABLE, //disable LCD error reports
            },

            .vsyn_delay_us = 1000,
        },
    },
};
#endif /* BSP_LCDC_USING_DSI */

#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_spi =
{
    .lcd_itf = QAD_SPI_ITF, //LCDC_INTF_SPI_NODCX_1DATA,
    .freq = 24000000,
    //.color_mode = LCDC_PIXEL_FORMAT_RGB888,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0, //0: QAD-SPI/SPI3   1:SPI4
            .syn_mode = HAL_LCDC_SYNC_DISABLE, //HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;

static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);

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
            HAL_LCDC_SetFreq(hlcdc, 2000000); //read mode min cycle 300ns
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

#ifdef BSP_LCDC_USING_DSI
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dsi, sizeof(lcdc_int_cfg));
#else
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_spi, sizeof(lcdc_int_cfg));
    rt_kprintf("\n ICN3311_Init lcdc_int_cfg_spi\n");
#endif

    /* Initialize ICN3311 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    LCD_DRIVER_DELAY_MS(10); //delay 10 ms

    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(20); //delay 20 ms
    //LCD_DRIVER_DELAY_MS(100); //delay 100 ms
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(10); //LCD must at sleep in mode after power on, 10ms is enough
    rt_kprintf("\n ICN3311_Init \n");
#if 0

    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);
    LCD_DRIVER_DELAY_MS(80);

    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x01;
    parameter[3] = 0xc5;
    LCD_WriteReg(hlcdc, 0x2a, parameter, 4);
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x01;
    parameter[3] = 0xc5;
    LCD_WriteReg(hlcdc, 0x2b, parameter, 4);

    parameter[0] = 0x01;
    parameter[1] = 0xc2;
    LCD_WriteReg(hlcdc, 0x44, parameter, 2);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, parameter, 1); //enable TE

    //parameter[0] = 0x40;
    //LCD_WriteReg(hlcdc, 0x36, parameter, 1); //enable revert

    parameter[0] = 0x28;
    LCD_WriteReg(hlcdc, 0x53, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, 0x51, parameter, 1); //ser brightness

    parameter[0] = 0x06;
    LCD_WriteReg(hlcdc, 0xb1, parameter, 1); //set back proch

    LCD_DRIVER_DELAY_MS(50);
    LCD_WriteReg(hlcdc, 0x29, (uint8_t *)NULL, 0);
    LCD_DRIVER_DELAY_MS(80);

#if 0//Bist mode
    parameter[0] = 0x5a;
    parameter[1] = 0x5a;
    LCD_WriteReg(hlcdc, 0xc0, parameter, 2);

    parameter[0] = 0x81;
    LCD_WriteReg(hlcdc, 0xba, parameter, 1);

    while (1);
#endif

    //LCD_WriteReg(hlcdc,0x23, (uint8_t *)NULL, 0);

    /* Tearing Effect Line On: Option (00h:VSYNC Only, 01h:VSYNC & HSYNC ) */
    //parameter[0] = 0x02;
    // LCD_WriteReg(hlcdc,REG_TEARING_EFFECT, parameter, 1);
#else
    /*
    FE,00
    C4,80

    FE,00
    3A,55
    35,00
    53,20
    51,FF
    63,FF
    2A,00,00,10,01,7F
    2B,00,00,00,01,BF

    11,00
    Delay_ms(120);
    29,00
    */

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);
    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0xC4, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);

    parameter[0] = 0x55;
    LCD_WriteReg(hlcdc, 0x3A, parameter, 1);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, parameter, 1);
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, 0x53, parameter, 1);

    parameter[0] = 0xff;
    LCD_WriteReg(hlcdc, 0x51, parameter, 1);
    parameter[0] = 0xff;
    LCD_WriteReg(hlcdc, 0x63, parameter, 1);

    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x10;
    parameter[3] = 0x01;
    parameter[4] = 0x7f;
    LCD_WriteReg(hlcdc, 0x2A, parameter, 5);
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    parameter[3] = 0x01;
    parameter[4] = 0xbf;
    LCD_WriteReg(hlcdc, 0x2B, parameter, 5);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x11, parameter, 1);

#if 0//BIST mode        
    parameter[0] = 0xD0;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);

    //bist en
    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0x4E, parameter, 1);

    parameter[0] = 0x40;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);
    parameter[0] = 0x54;
    LCD_WriteReg(hlcdc, 0xAF, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);
    parameter[0] = 0x0B;
    LCD_WriteReg(hlcdc, 0xC2, parameter, 1);
#endif
//sleep out+display on
    LCD_DRIVER_DELAY_MS(120);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x29, parameter, 1);

//  LCD_WriteReg(hlcdc, 0x29, (uint8_t *)NULL, 0);
#endif

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
    data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    rt_kprintf("\nICN3311_ReadID 0x%x \n", data);
    data = LCD_ReadData(hlcdc, REG_RASET, 4);
    rt_kprintf("\nICN3311_Read REG_RASET 0x%x \n", data);

#if 0
    data = LCD_ReadData(hlcdc, 0x0a, 4);
    DEBUG_PRINTF("\nICN3311_Read  0a 0x%x \n", data);
#endif
    if (data)
    {
        DEBUG_PRINTF("LCD module use ICN3311 IC \n");
        data = THE_LCD_ID;
    }
    data = THE_LCD_ID;

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
    rt_kprintf("\n ICN3311_DisplayOn\n");
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
    LCD_ALIGN2(Xpos0);
    LCD_ALIGN2(Ypos0);
    LCD_ALIGN1(Xpos1);
    LCD_ALIGN1(Ypos1);

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
    uint8_t   parameter[4];

    LCD_ALIGN2(Xpos);
    LCD_ALIGN2(Ypos);
    rt_kprintf("\n ICN3311_WritePixel xpos=%d, ypos=%d\n", Xpos, Ypos);

    if ((Xpos >= THE_LCD_PIXEL_WIDTH) || (Ypos >= THE_LCD_PIXEL_HEIGHT))
    {
        return;
    }

    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;
    LCD_ALIGN2(Xpos0);
    LCD_ALIGN2(Ypos0);
    LCD_ALIGN1(Xpos1);
    LCD_ALIGN1(Ypos1);

    uint32_t data;
    //data = LCD_ReadData(hlcdc, 0x05, 3);
    //DEBUG_PRINTF("DSI ERROR= 0x%x \n", data);

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
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
    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
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
    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
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
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("ICN3311_ReadPixel[%d,%d]\n", Xpos, Ypos);

    LCD_ALIGN2(Xpos);
    LCD_ALIGN2(Ypos);

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

    //LCD_WriteReg(hlcdc,REG_COLOR_MODE, parameter, 1);

    return ret_v;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];

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
        return; //unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define ICN3311_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)ICN3311_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}

static const LCD_DrvOpsDef ICN3311_drv =
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
    LCD_SetBrightness
};

LCD_DRIVER_EXPORT2(icn3311, THE_LCD_ID, &lcdc_int_cfg,
                   &ICN3311_drv, 1);

