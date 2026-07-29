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

#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x0E)

/**
  * @brief RM69330 chip IDs
  */
#define THE_LCD_ID                  0x8000

/**
  * @brief  RM69330 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (454)
#define  THE_LCD_PIXEL_HEIGHT   (454)

/**
  * @brief  RM69330 Registers
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

#define RM69330_SET_DISP_MODE      0xC4

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*rm69330 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)

#ifdef BSP_LCDC_USING_DSI

static const LCDC_InitTypeDef lcdc_int_cfg_dsi =
{
    .lcd_itf = LCDC_INTF_DSI,
    .freq = DSI_FREQ_480Mbps, //RM69330 RGB565 only support 320Mbps,  RGB888 support 500Mbps
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
                .TearingEffectSource   = DSI_TE_DSILINK,
#ifdef LCD_RM69330_VSYNC_ENABLE
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE,     //Open TE
#else
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Close TE
#endif /* LCD_RM69330_VSYNC_ENABLE */
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

            .vsyn_delay_us = 0,
        },
    },
};
#endif /* BSP_LCDC_USING_DSI */

#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_qspi =
{
    .lcd_itf = QAD_SPI_ITF,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0, //0: QAD-SPI/SPI3   1:SPI4
            .syn_mode = HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static const LCDC_InitTypeDef lcdc_int_cfg_3spi_1data =
{
    .lcd_itf = LCDC_INTF_SPI_NODCX_1DATA,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
            .syn_mode = HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static const LCDC_InitTypeDef lcdc_int_cfg_3spi_2data =
{
    .lcd_itf = LCDC_INTF_SPI_NODCX_2DATA,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
            .syn_mode = HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static const LCDC_InitTypeDef lcdc_int_cfg_4spi_1data =
{
    .lcd_itf = LCDC_INTF_SPI_DCX_1DATA,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
            .syn_mode = HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static const LCDC_InitTypeDef lcdc_int_cfg_dbi =
{
    .lcd_itf = LCDC_INTF_DBI_8BIT_B,
    .freq = 36000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .dbi = {
            .RD_polarity = 0,
            .WR_polarity = 0,
            .RS_polarity = 0,
            .CS_polarity = 0,
#ifdef LCD_RM69330_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_RM69330_VSYNC_ENABLE */
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;

static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);

#ifdef RM69330_DEBUG

void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    rt_kprintf(fmt, args);
    rt_kputs("\r\n");
    va_end(args);
}
#endif /* RM69330_DEBUG */

//#define RM69330_LCD_TEST
#ifdef RM69330_LCD_TEST
uint8_t rm69330_dsip_mode_value = 0;

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
        rm69330_dsip_mode_value = 0x80;
    else if (strcmp(argv[4], "1p1t") == 0)
        rm69330_dsip_mode_value = 0x81 | (0 << 4);
    else if (strcmp(argv[4], "1p1t_2w") == 0)
        rm69330_dsip_mode_value = 0x81 | (2 << 4);
    else if (strcmp(argv[4], "2p3t_2w") == 0)
        rm69330_dsip_mode_value = 0x81 | (3 << 4);

    lcdc_int_cfg_spi.cfg.spi.dummy_clock = strtoul(argv[5], 0, 10);

    DEBUG_PRINTF("\nlcd_cfg itf=%d, colormode=%d, freq=%d, disp_m=%x\n", lcdc_int_cfg_spi.lcd_itf,
                 lcdc_int_cfg_spi.color_mode,
                 lcdc_int_cfg_spi.freq,
                 rm69330_dsip_mode_value
                );

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
    if (HAL_LCDC_IS_SPI_IF(lcdc_int_cfg.lcd_itf) || HAL_LCDC_IS_DBI_IF(lcdc_int_cfg.lcd_itf))
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
#elif defined(BSP_LCDC_USING_DBI)
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dbi, sizeof(lcdc_int_cfg));
#else
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_3spi_1data, sizeof(lcdc_int_cfg));
#endif /* BSP_LCDC_USING_DSI */

    /* Initialize RM69330 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(10); //LCD must at sleep in mode after power on, 10ms is enough

    if (HAL_LCDC_IS_DBI_IF(lcdc_int_cfg.lcd_itf))
    {
        parameter[0] = 0x00;
        parameter[1] = 0x21;
        HAL_LCDC_WriteU16Reg(hlcdc, 0x6A40, parameter, 2);
    }
    else
    {
        /* SW Reset Command */
        //LCD_WriteReg(hlcdc,REG_SW_RESET, (uint8_t *)NULL, 0);

        /* Wait for 200ms */
        //LCD_DRIVER_DELAY_MS(200);
        parameter[0] = 0x01;
        //LCD_WriteReg(hlcdc,0xFE, parameter, 1);  //Page 0
        {
            parameter[0] = 19; //0,1,19
            //LCD_WriteReg(hlcdc,0x6a, parameter, 1); //Turn on power ic
        }

        parameter[0] = 0x07;
        LCD_WriteReg(hlcdc, 0xFE, parameter, 1); //page 6
        parameter[0] = 0x04;
        LCD_WriteReg(hlcdc, 0x15, parameter, 1); //SRAM read adjust control
        parameter[0] = 0x00;
        LCD_WriteReg(hlcdc, 0xFE, parameter, 1); //User cmd

    }

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, parameter, 1); //enable TE

    if (LCDC_PIXEL_FORMAT_RGB888 == lcdc_int_cfg.color_mode)
        parameter[0] = 0x77; //24bit rgb
    else if (LCDC_PIXEL_FORMAT_RGB565 == lcdc_int_cfg.color_mode)
        parameter[0] = 0x75; //16bit rgb
    else
        RT_ASSERT(0); //fix me
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    parameter[0] = 0xFF;
    LCD_WriteReg(hlcdc, REG_WBRIGHT, parameter, 1); //ser brightness

    if (HAL_LCDC_IS_SPI_IF(lcdc_int_cfg.lcd_itf))
    {
        switch (lcdc_int_cfg.lcd_itf)
        {
        /*
            bit 0:  DUAL SPI MODE enable
            bit[5:4] DAUL SPI MODE Selection
                      00: 1P1T for 1 wire
                      10: 1P1T for 2 wire
                      11: 2P3T for 2 wire
                      01: reserve

        */
        case LCDC_INTF_SPI_NODCX_2DATA:
            parameter[0] = (2 << 4) | 0x01;
            break;

        case LCDC_INTF_SPI_DCX_2DATA:
        case LCDC_INTF_SPI_NODCX_4DATA:
            RT_ASSERT(0); //not support
            break;

        default:
            parameter[0] = 0;
            break;
        }
        parameter[0] |= 0x80; //SPI write GRAM
#ifdef RM69330_LCD_TEST
        if (rm69330_dsip_mode_value) parameter[0] = rm69330_dsip_mode_value;
#endif /* RM69330_LCD_TEST */
        LCD_WriteReg(hlcdc, RM69330_SET_DISP_MODE, parameter, 1); //set_DSPI Mode
    }

    /* Wait for 110ms */
    LCD_DRIVER_DELAY_MS(110);

    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(5); //Delay 5 ms after sleep out

    /* Display ON command */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(50); //Wait TE signal ready

    //LCD_WriteReg(hlcdc,0x23, (uint8_t *)NULL, 0);

    /* Tearing Effect Line On: Option (00h:VSYNC Only, 01h:VSYNC & HSYNC ) */
    //parameter[0] = 0x02;
    // LCD_WriteReg(hlcdc,REG_TEARING_EFFECT, parameter, 1);

#if 0

    /*Clear gram*/
    HAL_LCDC_Enable_TE(hlcdc, 0);
    LCD_SetRegion(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH - 1, THE_LCD_PIXEL_HEIGHT - 1);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 255, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, REG_WRITE_RAM, 1);

    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
#ifdef LCD_RM69330_VSYNC_ENABLE
    HAL_LCDC_Enable_TE(hlcdc, 1);
#endif /* LCD_RM69330_VSYNC_ENABLE */

    LCD_DRIVER_DELAY_MS(1000);
#endif /* 0 */

    {
        uint32_t data;
        /*

            Bit Symbol Description Comment
            D7 BSTON Booster Voltage Status "1"=Booster on, "0"=Booster off
            D6 IDMON Idle Mode On/Off "1" = Idle Mode On,        "0" = Idle Mode Off
            D5 PTLON Partial Mode On/Off "1" = Partial Mode On,        "0" = Partial Mode Off
            D4 SLPON Sleep In/Out "1" = Sleep Out,        "0" = Sleep In
            D3 NORON Display Normal Mode On/Off        "1" = Normal Display,        "0" = Partial Display
            D2 DISON Display On/Off "1" = Display On,        "0" = Display Off
            D1 Reserved 0
            D0 Reserved 0
        */
        data = LCD_ReadData(hlcdc, REG_POWER_MODE, 4);
        DEBUG_PRINTF("PowerMode 0x%x", data);
    }

}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
#ifdef DSI_HW_TEST
    return THE_LCD_ID;
#else
    uint32_t data;
    /*
        data = LCD_ReadData(hlcdc,REG_CASET, 4);
        DEBUG_PRINTF("\REG_CASET 0x%x \n", data);

        data = LCD_ReadData(hlcdc,REG_RASET, 4);
        DEBUG_PRINTF("\REG_RASET 0x%x \n", data);
    */
    data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    DEBUG_PRINTF("\nRM69330_ReadID 0x%x \n", data);

    if (data)
    {
        DEBUG_PRINTF("LCD module 0xa23099 use RM69330 IC \n");
        data = THE_LCD_ID;
    }

    return data;
#endif /* DSI_HW_TEST */

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

    RT_ASSERT(0 == (Xpos0 & 1));
    RT_ASSERT(0 == (Ypos0 & 1));
    RT_ASSERT(1 == (Xpos1 & 1));
    RT_ASSERT(1 == (Ypos1 & 1));

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

    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    }
    else if (HAL_LCDC_IS_DBI_IF(lcdc_int_cfg.lcd_itf))
    {
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, (REG_WRITE_RAM << 8), 2);
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
        DEBUG_PRINTF("RM69330_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("RM69330_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
#endif

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_WriteU32Reg(hlcdc, ((0x02 << 24) | (LCD_Reg << 8)), Parameters, NbParameters);
    }
    else if (HAL_LCDC_IS_DBI_IF(lcdc_int_cfg.lcd_itf))
    {
        uint8_t i;

        LCD_Reg = LCD_Reg << 8;

        if (0 == NbParameters)
        {
            HAL_LCDC_WriteU16Reg(hlcdc, LCD_Reg, NULL, 0);
        }
        else
        {
            for (i = 0; i < NbParameters; i++)
            {
                uint8_t v[2];
                v[0] = 0;
                v[1] = Parameters[i];

                HAL_LCDC_WriteU16Reg(hlcdc, LCD_Reg + i, v, 2);
            }
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
    else if (HAL_LCDC_IS_DBI_IF(lcdc_int_cfg.lcd_itf))
    {
        uint8_t i;

        RegValue = RegValue << 8;
        for (i = 0; i < ReadSize; i++)
        {
            uint16_t v;
            HAL_LCDC_ReadU16Reg(hlcdc, RegValue + i, (uint8_t *)&v, 1);

            rd_data = (rd_data << 8) | (v & 0xFF);
        }
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
    DEBUG_PRINTF("NOT support read pixel\n");

    return 0; //Not support read pixel

#if 0
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("RM69330_ReadPixel[%d,%d]\n", Xpos, Ypos);

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
#endif /* 0 */
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];

    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x75;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565_SWAP;

#ifdef BSP_LCDC_USING_DSI
        if (HAL_LCDC_IS_DSI_IF(lcdc_int_cfg.lcd_itf) && (lcdc_int_cfg.freq > DSI_FREQ_336Mbps))
            HAL_LCDC_SetFreq(hlcdc, DSI_FREQ_336Mbps);  //RGB565 only support 320Mbps,  RGB888 support 500Mbps
#endif /* BSP_LCDC_USING_DSI */
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0x77;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
#ifdef BSP_LCDC_USING_DSI
        HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq);  //Revert freq in case of change in RGB565
#endif /* BSP_LCDC_USING_DSI */
        break;

    default:
        return; //unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define RM69330_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)RM69330_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}

/**
  * @brief  Enable the Display idle mode.
  * @param  None
  * @retval None
  */
static void LCD_IdleModeOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Idle mode On */
    LCD_WriteReg(hlcdc, REG_IDLE_MODE_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display idle mode.
  * @param  None
  * @retval None
  */
static void LCD_IdleModeOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Idle mode Off */
    LCD_WriteReg(hlcdc, REG_IDLE_MODE_OFF, (uint8_t *)NULL, 0);
}

static void LCD_TimeoutDbg(LCDC_HandleTypeDef *hlcdc)
{
}

static void LCD_TimeoutReset(LCDC_HandleTypeDef *hlcdc)
{
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_mdelay(100);
    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);

    LCD_Init(hlcdc);
}

static uint32_t LCD_ESDCehck(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    /*

        Bit Symbol Description Comment
        D7 BSTON Booster Voltage Status "1"=Booster on, "0"=Booster off
        D6 IDMON Idle Mode On/Off "1" = Idle Mode On,        "0" = Idle Mode Off
        D5 PTLON Partial Mode On/Off "1" = Partial Mode On,        "0" = Partial Mode Off
        D4 SLPON Sleep In/Out "1" = Sleep Out,        "0" = Sleep In
        D3 NORON Display Normal Mode On/Off        "1" = Normal Display,        "0" = Partial Display
        D2 DISON Display On/Off "1" = Display On,        "0" = Display Off
        D1 Reserved 0
        D0 Reserved 0
    */
    data = LCD_ReadData(hlcdc, REG_POWER_MODE, 4);
    DEBUG_PRINTF("RM69330_ESD 0x%x", data);

    return 0x80 & data ? 0 : 1;
}

static const LCD_DrvOpsDef RM69330_drv =
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
    LCD_IdleModeOn,
    LCD_IdleModeOff,
    NULL,
    LCD_TimeoutDbg,
    LCD_TimeoutReset,
    LCD_ESDCehck
};

LCD_DRIVER_EXPORT2(rm69330, THE_LCD_ID, &lcdc_int_cfg,
                   &RM69330_drv, 2);

