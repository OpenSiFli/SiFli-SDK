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
  * @brief ST7701S chip IDs
  */
#define THE_LCD_ID                  0x8000

/**
  * @brief  ST7701S IC Size
  */
#define  THE_LCD_PIXEL_WIDTH    (480)
#define  THE_LCD_PIXEL_HEIGHT   (800)

/**
  * @brief  ST7701S Registers
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

/*st7701s start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)

static const LCDC_InitTypeDef lcdc_int_cfg_dsi =
{
    .lcd_itf = LCDC_INTF_DSI_VIDEO,
    .freq = DSI_FREQ_528Mbps,
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {

        .dsi = {

            .Init = {
                .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,
                .NumberOfLanes = DSI_TWO_DATA_LANES,
                .TXEscapeCkdiv = 0x4,
            },

            .CmdCfg = {
                .VirtualChannelID      = 0,
                .CommandSize           = 0xFFFF,
                .TearingEffectSource   = DSI_TE_DSILINK,
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Close TE
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
                .TimeoutCkdiv = 4,
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
                .AcknowledgeRequest    = DSI_ACKNOWLEDGE_ENABLE, //disable LCD error reports
            },

            .VidCfg = {
                .Mode = DSI_VID_MODE_NB_EVENTS,

                .VS_width      = 4,    //VLW
                .HS_width      = 12,   //HLW

                .VBP = 20,
                .VAH = LCD_HOR_RES_MAX,
                .VFP = 20,

                .HBP = 30,
                .HAW = LCD_VER_RES_MAX,
                .HFP = 30,

                .interrupt_line_num = 1,
            },
        },
    },
};

#define MAX_CMD_LEN 18

#define GP_COMMAD_PA(...)   {__VA_ARGS__}

static const uint8_t lcd_init_cmds1[][MAX_CMD_LEN] =
{

//---------------------------------------Bank0 Setting-------------------------------------------------//
//------------------------------------Display Control setting----------------------------------------------//
    GP_COMMAD_PA(6, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10),
    GP_COMMAD_PA(3, 0xC0, 0x63, 0x00),
    GP_COMMAD_PA(3, 0xC1, 0x11, 0x02),
    GP_COMMAD_PA(3, 0xC2, 0x31, 0x08),
    GP_COMMAD_PA(2, 0xCC, 0x10),
    GP_COMMAD_PA(17, 0xB0, 0x40, 0x01, 0x46, 0x0D, 0x13, 0x09, 0x05, 0x09, 0x09, 0x1B, 0x07, 0x15, 0x12, 0x4C, 0x10, 0xC8),
    GP_COMMAD_PA(17, 0xB1, 0x40, 0x02, 0x86, 0x0D, 0x13, 0x09, 0x05, 0x09, 0x09, 0x1F, 0x07, 0x15, 0x12, 0x15, 0x19, 0x08),
//---------------------------------------End Gamma Setting----------------------------------------------//
//------------------------------------End Display Control setting----------------------------------------//
//-----------------------------------------Bank0 Setting End---------------------------------------------//
//-------------------------------------------Bank1 Setting---------------------------------------------------//
//-------------------------------- Power Control Registers Initial --------------------------------------//
    GP_COMMAD_PA(6, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11),
    GP_COMMAD_PA(2, 0xB0, 0x50),
//-------------------------------------------Vcom Setting---------------------------------------------------//
    GP_COMMAD_PA(2, 0xB1, 0x68),
//-----------------------------------------End Vcom Setting-----------------------------------------------//
    GP_COMMAD_PA(2, 0xB2, 0x07),
    GP_COMMAD_PA(2, 0xB3, 0x80),
    GP_COMMAD_PA(2, 0xB5, 0x47),
    GP_COMMAD_PA(2, 0xB7, 0x85),
    GP_COMMAD_PA(2, 0xB8, 0x21),
    GP_COMMAD_PA(2, 0xB9, 0x10),
    GP_COMMAD_PA(2, 0xC1, 0x78),
    GP_COMMAD_PA(2, 0xC2, 0x78),
    GP_COMMAD_PA(2, 0xD0, 0x88),
//---------------------------------End Power Control Registers Initial -------------------------------//
};

static const uint8_t lcd_init_cmds2[][MAX_CMD_LEN] =
{
//---------------------------------------------GIP Setting----------------------------------------------------//
    GP_COMMAD_PA(4, 0xE0, 0x00, 0x00, 0x02),
    GP_COMMAD_PA(12, 0xE1, 0x08, 0x00, 0x0A, 0x00, 0x07, 0x00, 0x09, 0x00, 0x00, 0x33, 0x33),
    GP_COMMAD_PA(14, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    GP_COMMAD_PA(5, 0xE3, 0x00, 0x00, 0x33, 0x33),
    GP_COMMAD_PA(3, 0xE4, 0x44, 0x44),
    GP_COMMAD_PA(17, 0xE5, 0x0E, 0x2D, 0xA0, 0xA0, 0x10, 0x2D, 0xA0, 0xA0, 0x0A, 0x2D, 0xA0, 0xA0, 0x0C, 0x2D, 0xA0, 0xA0),
    GP_COMMAD_PA(5, 0xE6, 0x00, 0x00, 0x33, 0x33),
    GP_COMMAD_PA(3, 0xE7, 0x44, 0x44),
    GP_COMMAD_PA(17, 0xE8, 0x0D, 0x2D, 0xA0, 0xA0, 0x0F, 0x2D, 0xA0, 0xA0, 0x09, 0x2D, 0xA0, 0xA0, 0x0B, 0x2D, 0xA0, 0xA0),
    GP_COMMAD_PA(8, 0xEB, 0x02, 0x01, 0xE4, 0xE4, 0x44, 0x00, 0x40),
    GP_COMMAD_PA(3, 0xEC, 0x02, 0x01),
    GP_COMMAD_PA(17, 0xED, 0xAB, 0x89, 0x76, 0x54, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x10, 0x45, 0x67, 0x98, 0xBA),
//--------------------------------------------End GIP Setting-----------------------------------------------//
//------------------------------ Power Control Registers Initial End-----------------------------------//
//------------------------------------------Bank1 Setting----------------------------------------------------//
    GP_COMMAD_PA(6, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00),

    GP_COMMAD_PA(1, 0x11),

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

}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    /* Initialize ST7701S to cmd mode    ----------------------------------*/
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dsi, sizeof(lcdc_int_cfg));
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(10); //LCD must at sleep in mode after power on, 10ms is enough

    for (uint32_t i = 0; i < sizeof(lcd_init_cmds1) / MAX_CMD_LEN; i++)
    {
        LCD_WriteReg(hlcdc, lcd_init_cmds1[i][1], (uint8_t *)&lcd_init_cmds1[i][2], lcd_init_cmds1[i][0] - 1);
    }

    LCD_DRIVER_DELAY_MS(100);

    for (uint32_t i = 0; i < sizeof(lcd_init_cmds2) / MAX_CMD_LEN; i++)
    {
        LCD_WriteReg(hlcdc, lcd_init_cmds2[i][1], (uint8_t *)&lcd_init_cmds2[i][2], lcd_init_cmds2[i][0] - 1);
    }

    LCD_DRIVER_DELAY_MS(120); //Delay 5 ms after sleep out

    /* Display ON command */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(10); //Wait TE signal ready

#if 0

    /*Clear gram*/
    HAL_LCDC_Enable_TE(hlcdc, 0);
    LCD_SetRegion(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH - 1, THE_LCD_PIXEL_HEIGHT - 1);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 255, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, REG_WRITE_RAM, 1);

    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
#ifdef LCD_ST7701S_VSYNC_ENABLE
    HAL_LCDC_Enable_TE(hlcdc, 1);
#endif /* LCD_ST7701S_VSYNC_ENABLE */

    LCD_DRIVER_DELAY_MS(1000);
#endif /* 0 */

    {
        uint32_t data = 0;
        /*

            Bit Description Value
            D7 Booster Voltage Status “1”=Booster On, “0”=Booster Off
            D6 Not Defined Set to “0” (not used)
            D5 Not Defined Set to “0” (not used)
            D4 Sleep In/Out “1” = Sleep Out Mode, “0” = Sleep In Mode
            D3 Not Defined Set to “1” (not used)
            D2 Display On/Off “1” = Display is On, “0” = Display is Off
            D1 Not Defined Set to “0” (not used)
            D0 Not Defined Set to “0” (not used)

        */
        data = LCD_ReadData(hlcdc, REG_POWER_MODE, 1);
        DEBUG_PRINTF("PowerMode 0x%x", data);
    }

    HAL_LCDC_SetROIArea(hlcdc, 0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);

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
    DEBUG_PRINTF("\nST7701S_ReadID 0x%x \n", data);

    if (data)
    {
        DEBUG_PRINTF("LCD module use ST7701S IC \n");
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
    //LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    /*Invalid*/
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
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
    if (0)
    {
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
    DEBUG_PRINTF("ST7701S_ReadPixel[%d,%d]\n", Xpos, Ypos);

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

#define ST7701S_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)ST7701S_BRIGHTNESS_MAX * br / 100);
    //LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
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

static const LCD_DrvOpsDef ST7701S_drv =
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
    LCD_IdleModeOff
};

LCD_DRIVER_EXPORT2(st7701s, THE_LCD_ID, &lcdc_int_cfg,
                   &ST7701S_drv, 2);

