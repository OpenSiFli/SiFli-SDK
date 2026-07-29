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

#define LOG_TAG                "rm69090"
#include "log.h"

#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x10)

/**
  * @brief RM69090 chip IDs
  */
#define THE_LCD_ID                  0x1190a7

/**
  * @brief  RM69090 Registers
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
#define RM69090_SCAN_DIRECTION_CTRL   0x36
#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/

#define RM69090_SET_DISP_MODE      0xC4

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*rm69090 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)

#ifdef BSP_LCDC_USING_DSI

#if 0//def APP_BSP_TEST  //Keep two data lanes for bsp test
    #define  RM69090_DSI_FREQ       DSI_FREQ_240Mbps
    #define  RM69090_DSI_DATALANES  DSI_TWO_DATA_LANES
#else
    #define  RM69090_DSI_FREQ       DSI_FREQ_480Mbps
    #define  RM69090_DSI_DATALANES  DSI_ONE_DATA_LANE
#endif /* APP_BSP_TEST */

static const LCDC_InitTypeDef lcdc_int_cfg_dsi =
{
    .lcd_itf = LCDC_INTF_DSI,
    .freq = RM69090_DSI_FREQ, //RM69090 RGB565 only support 320Mbps,  RGB888 support 500Mbps
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {

        .dsi = {

            .Init = {
                .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,
                .NumberOfLanes = RM69090_DSI_DATALANES,
                .TXEscapeCkdiv = 0x4,
            },

            .CmdCfg = {
                .VirtualChannelID      = 0,
                .CommandSize           = 0xFFFF,
                .TearingEffectSource   = DSI_TE_DSILINK,
#ifdef LCD_RM69090_VSYNC_ENABLE
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE,     //Open TE
#else
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Close TE
#endif /* LCD_RM69090_VSYNC_ENABLE */
            },

            .PhyTimings = {
                .ClockLaneHS2LPTime = 35,
                .ClockLaneLP2HSTime = 35,
                .DataLaneHS2LPTime = 35,
                .DataLaneLP2HSTime = 35,
                .DataLaneMaxReadTime = 0,
                .StopWaitTime = 0, //10
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

static const LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
#ifdef LCD_RM69090_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_RM69090_VSYNC_ENABLE */
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
        },
    },

};

static const LCDC_InitTypeDef lcdc_int_cfg_3spi_1data =
{
    .lcd_itf = LCDC_INTF_SPI_NODCX_1DATA,
    .freq = 4000000,
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
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

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

static LCDC_InitTypeDef lcdc_int_cfg;

static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);

#if 0//def DSI_TEST
MSH_CMD_EXPORT(RM69090_Init, RM69090_Init);

static rt_err_t lcd_rreg(int argc, char **argv)
{

    uint16_t reg, len;

    reg = strtoul(argv[1], 0, 16);
    len = strtoul(argv[2], 0, 16);

    if (len > 4)
    {
        DEBUG_PRINTF("read length > 4\n");
    }
    else
    {
        uint8_t *data;
        uint32_t i, ret_val;
        ret_val = LCD_ReadData(reg, len);

        data = (uint8_t *) &ret_val;

        DEBUG_PRINTF("\nRM69090_Read reg[%x] %d(byte)\n", reg, len);

        for (i = 0; i < len; i++)
            DEBUG_PRINTF("result[%d]=0x%x\n", i, data[i]);
    }

    return 0;
}
MSH_CMD_EXPORT(lcd_rreg, lcd_rreg);

static rt_err_t lcd_wreg(int argc, char **argv)
{
    uint8_t   parameter[4];

    uint32_t data;
    uint16_t reg, i;

    reg = strtoul(argv[1], 0, 16);

    for (i = 2; i < argc; i++)
    {
        parameter[i - 2] = strtoul(argv[i], 0, 16);
    }

    LCD_WriteReg(hlcdc, reg, parameter, argc - 2);
    DEBUG_PRINTF("\nRM69090_Write reg[%x] %d(byte) done.\n", reg, argc - 2);

    return 0;

}
MSH_CMD_EXPORT(lcd_wreg, lcd_wreg);

uint32_t my_debug_pwl, my_debug_pwh;
uint8_t  dual_spi_cfg;

static rt_err_t spi_cfg(int argc, char **argv)
{

    switch (strtoul(argv[1], 0, 16))
    {
    case 0x01:
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_1DATA;
        break;

    case 0x02:
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_2DATA;
        break;

    case 0x04:
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_4DATA;
        break;

    case 0x11:
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_1DATA;
        break;

    case 0x12:
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_2DATA;
        break;

    case 0x14:
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_4DATA;
        break;
    }

    dual_spi_cfg = strtoul(argv[2], 0, 16);

    return 0;

}
MSH_CMD_EXPORT(spi_cfg, spi_cfg);

#endif /* DSI_TEST */

#ifdef QAD_SPI_USE_GPIO_CS
    void gpio_cs_enable(void);
    void gpio_cs_disable(void);
#endif /* QAD_SPI_USE_GPIO_CS */

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

#ifdef BSP_LCDC_USING_QADSPI
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_qadspi, sizeof(lcdc_int_cfg));
#elif defined(BSP_LCDC_USING_DSI)
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dsi, sizeof(lcdc_int_cfg));
#else
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_3spi_1data, sizeof(lcdc_int_cfg));
#endif /* BSP_LCDC_USING_QADSPI */

    /* Initialize RM69090 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(10); //LCD must at sleep in mode after power on, 10ms is enough.

    /* SW Reset Command */
    //LCD_WriteReg(hlcdc,REG_SW_RESET, (uint8_t *)NULL, 0);

    /* Wait for 200ms */
    //LCD_DRIVER_DELAY_MS(200);

    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1); //Page 0
    {
        parameter[0] = 19; //0,1,19
        LCD_WriteReg(hlcdc, 0x6a, parameter, 1); //Turn on power ic

        if (HAL_LCDC_IS_DSI_IF(lcdc_int_cfg.lcd_itf))
        {
#ifdef BSP_LCDC_USING_DSI
            if (DSI_TWO_DATA_LANES == lcdc_int_cfg.cfg.dsi.Init.NumberOfLanes)
                parameter[0] = 0x01;
            else
                parameter[0] = 0x00;

            LCD_WriteReg(hlcdc, 0x0d, parameter, 1); //enable 2 lane
#else
            RT_ASSERT(0);
#endif
        }
    }

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1); //User cmd
    {
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
            case LCDC_INTF_SPI_NODCX_1DATA:
                parameter[0] = (0 << 4) | 0x01;
                break;
            case LCDC_INTF_SPI_NODCX_2DATA:
                parameter[0] = (2 << 4) | 0x01;
                break;

            default:
                parameter[0] = 0;
                break;
            }
            parameter[0] |= 0x80; //SPI write GRAM
            //parameter[0] = dual_spi_cfg;
            LCD_WriteReg(hlcdc, RM69090_SET_DISP_MODE, parameter, 1); //set_DSPI Mode
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
        //parameter[0] = 0x00;
        //LCD_WriteReg(hlcdc,REG_TEARING_EFFECT, parameter, 1);
    }

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

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 4);
    DEBUG_PRINTF("RM69090_ReadID 0x%x", data);
    data = ((data << 24) & 0xFF000000)
           | ((data <<  8) & 0x00FF0000)
           | ((data >>  8) & 0x0000FF00)
           | ((data >> 24) & 0x000000FF);

    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf) //Not support read LCD ID with QASPI interface
        return THE_LCD_ID;

    if (data != UINT32_MAX)
        return THE_LCD_ID;
    else
        return ~THE_LCD_ID;

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
        DEBUG_PRINTF("RM69090_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("RM69090_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
#endif

#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_WriteU32Reg(hlcdc, ((0x02 << 24) | (LCD_Reg << 8)), Parameters, NbParameters);
    }
    else
    {
        HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
    }

#ifdef QAD_SPI_USE_GPIO_CS
    HAL_Delay_us(20); //wait lcdc sending single data finished
    gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

}

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize)
{
    HAL_StatusTypeDef err;
    uint32_t rd_data = 0;
#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

    LCD_ReadMode(hlcdc, true);
    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        err = HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);
    }
    else
    {
        err = HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);
    }
    LCD_ReadMode(hlcdc, false);
#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

    if (err != HAL_OK)
        return UINT32_MAX;
    else
        return rd_data;
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    DEBUG_PRINTF("NOT support read pixel\n");

    return 0; //Not support read pixel

#if 0
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("RM69090_ReadPixel[%d,%d]\n", Xpos, Ypos);

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

    /*

    Control interface color format
    ��011�� = 12bit/pixel ��101�� = 16bit/pixel ��110�� = 18bit/pixel ��111�� = 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x75;
        if (HAL_LCDC_IS_DSI_IF(lcdc_int_cfg.lcd_itf))
            lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565_SWAP;
        else
            lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
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

#define RM69090_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)RM69090_BRIGHTNESS_MAX * br / 100);
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

static void LCD_Rotate(LCDC_HandleTypeDef *hlcdc, LCD_DrvRotateTypeDef degrees)
{
    if (LCD_ROTATE_0_DEGREE == degrees)
    {
        uint8_t v;

        v = 0 << 6;
        LCD_WriteReg(hlcdc, RM69090_SCAN_DIRECTION_CTRL, (uint8_t *)&v, 1);
    }
#if defined(LCDC_SUPPORT_V_MIRROR)&&!defined(LCDC_SUPPORT_H_MIRROR)
    else if (LCD_ROTATE_180_DEGREE == degrees)
    {
        uint8_t v;

        v = 1 << 6;
        LCD_WriteReg(hlcdc, RM69090_SCAN_DIRECTION_CTRL, (uint8_t *)&v, 1);
    }
#endif /* LCDC_SUPPORT_V_MIRROR && !LCDC_SUPPORT_H_MIRROR*/
    else
    {
        LOG_E("Unsupport rotate %d degress", degrees);
    }
}

static const LCD_DrvOpsDef RM69090_drv =
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
    LCD_Rotate
};

LCD_DRIVER_EXPORT2(rm69090, THE_LCD_ID, &lcdc_int_cfg,
                   &RM69090_drv, 2);

