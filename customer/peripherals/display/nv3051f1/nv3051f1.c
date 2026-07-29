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
#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //
#define LOG_TAG                "nv3051f1"

#include "log.h"

#define LCD_ID                  0x8000

/**
  * @brief  LCD IC Size
  */
#define  LCD_IC_PIXEL_WIDTH    (800)
#define  LCD_IC_PIXEL_HEIGHT   (2048)

/**
  * @brief  LCD Registers
  */
#define REG_LCD_ID             0x04
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
#define REG_WBRIGHT            0x51

#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x0E)

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_E(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*start colume & row must can be divided by 2, and roi width&height too.*/
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

                .VS_width      = 2,    //VLW
                .HS_width      = 2,   //HLW

                .VBP = 14,
                .VAH = LCD_HOR_RES_MAX,
                .VFP = 16,

                .HBP = 44,
                .HAW = LCD_VER_RES_MAX,
                .HFP = 46,

                .interrupt_line_num = 1,
            },
        },
    },
};

#define MAX_CMD_LEN 2

static const uint8_t H034A01_NV3051_800x800_init_cmds[][MAX_CMD_LEN] =
{
    {0xFF, 0x30},
    {0xFF, 0x52},
    {0xFF, 0x01},
    {0xE3, 0x00},
    {0x08, 0x0E},
    {0x25, 0x0f},
    {0x28, 0x0F},
    {0x29, 0xC3},
    {0x2a, 0x1F},
    {0x37, 0x9C}, //5.5V
    {0x38, 0xA7}, //-5.5V
    {0x39, 0x51}, //vcom
    {0x44, 0x00},
    {0x49, 0x3C},
    {0x59, 0xfe},
    {0x5c, 0x00},
    {0x80, 0x20},
    {0x91, 0x77},
    {0x92, 0x77},
    {0x98, 0xCA},
    {0x99, 0x51},
    {0x9a, 0xC1},
    {0x9B, 0x59},
    {0xA0, 0x55},
    {0xA1, 0x50},
    {0xA4, 0x9C},
    {0xA7, 0x02},
    {0xA8, 0x01},
    {0xA9, 0x21},
    {0xAA, 0xFC},
    {0xAB, 0x28},
    {0xAC, 0x06},
    {0xAD, 0x06},
    {0xAE, 0x06},
    {0xAF, 0x03},
    {0xB0, 0x08},
    {0xB1, 0x26},
    {0xB2, 0x28},
    {0xB3, 0x28},
    {0xB4, 0x33},
    {0xB5, 0x08},
    {0xB6, 0x26},
    {0xB7, 0x08},
    {0xB8, 0x26},

    {0xF0, 0x00}, //BIST 07-0F
    {0xF6, 0xC0},
    {0xC0, 0x00}, //ADD FOR ESD
    {0xC1, 0x00},
    {0xC3, 0x0F},
    {0x2C, 0x22},
    {0xFF, 0x30},
    {0xFF, 0x52},
    {0xFF, 0x02},
    {0xB1, 0x04},
    {0xD1, 0x0E},
    {0xB4, 0x2C},
    {0xD4, 0x36},
    {0xB2, 0x04},
    {0xD2, 0x0C},
    {0xB3, 0x34},
    {0xD3, 0x2C},
    {0xB6, 0x12},
    {0xD6, 0x18},
    {0xB7, 0x37},
    {0xD7, 0x3D},
    {0xC1, 0x04},
    {0xE1, 0x08},
    {0xB8, 0x0A},
    {0xD8, 0x0C},
    {0xB9, 0x01},
    {0xD9, 0x03},
    {0xBD, 0x13},
    {0xDD, 0x13},
    {0xBC, 0x11},
    {0xDC, 0x11},
    {0xBB, 0x0F},
    {0xDB, 0x0F},
    {0xBA, 0x10},
    {0xDA, 0x10},
    {0xBE, 0x16},
    {0xDE, 0x1E},
    {0xBF, 0x0D},
    {0xDF, 0x15},
    {0xC0, 0x13},
    {0xE0, 0x1D},
    {0xB5, 0x37},
    {0xD5, 0x32},
    {0xB0, 0x02},
    {0xD0, 0x05},

    {0xFF, 0x30},
    {0xFF, 0x52},
    {0xFF, 0x03},

    {0x08, 0x85},
    {0x09, 0x86},
    {0x0A, 0x83},
    {0x0B, 0x84},
    {0x20, 0x00},
    {0x21, 0x00},
    {0x22, 0x00},
    {0x23, 0x00},
    {0x27, 0x03},

    {0x28, 0x33},
    {0x29, 0x33},

    {0x2A, 0x27},
    {0x2b, 0x27},

    {0x34, 0x11},
    {0x35, 0x01},
    {0x36, 0x01},
    {0x37, 0x03},

    {0x40, 0x81},
    {0x41, 0x82},
    {0x42, 0x01},
    {0x43, 0x80},

    {0x44, 0x33},
    {0x45, 0x25},
    {0x46, 0x24},
    {0x47, 0x33},
    {0x48, 0x27},
    {0x49, 0x26},

    {0x50, 0x85},
    {0x51, 0x86},
    {0x52, 0x83},
    {0x53, 0x84},

    {0x54, 0x33},
    {0x55, 0x21},
    {0x56, 0x20},
    {0x57, 0x33},
    {0x58, 0x23},
    {0x59, 0x22},

    {0x7e, 0x03},
    {0x7f, 0x02},

    {0x80, 0x0f},
    {0x81, 0x0f},
    {0x82, 0x0e},
    {0x83, 0x0e},
    {0x84, 0x02},
    {0x85, 0x04},
    {0x86, 0x04},
    {0x87, 0x05},
    {0x88, 0x05},
    {0x89, 0x06},
    {0x8A, 0x06},
    {0x8B, 0x07},
    {0x8C, 0x07},
    {0x8D, 0x0f},
    {0x8E, 0x0f},
    {0x8F, 0x0d},
    {0x90, 0x0d},
    {0x91, 0x0c},
    {0x92, 0x0c},
    {0x93, 0x00},
    {0x94, 0x01},
    {0x96, 0x0f},
    {0x97, 0x0f},
    {0x98, 0x0e},
    {0x99, 0x0e},
    {0x9A, 0x02},
    {0x9B, 0x04},
    {0x9C, 0x04},
    {0x9D, 0x05},
    {0x9E, 0x05},
    {0x9F, 0x06},
    {0xA0, 0x06},
    {0xA1, 0x07},
    {0xA2, 0x07},
    {0xA3, 0x0f},
    {0xA4, 0x0f},
    {0xA5, 0x0d},
    {0xA6, 0x0d},
    {0xA7, 0x0c},
    {0xA8, 0x0c},
    {0xA9, 0x00},
    {0xAA, 0x01},

    {0xff, 0x30},
    {0xff, 0x52},
    {0xff, 0x00},

    {0x36, 0x02},
    {0x53, 0x2c},
};

static LCDC_InitTypeDef lcdc_int_cfg;

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
  * @brief  spi read/write mode
  * @param  enable: false - write spi mode |  true - read spi mode
  * @retval None
  */
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable)
{

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

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];
    uint32_t data = 0;

    /* Initialize LCD to cmd mode    ----------------------------------*/
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dsi, sizeof(lcdc_int_cfg));
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(10); //LCD must at sleep in mode after power on, 10ms is enough

    //LCD_WriteReg(hlcdc, 0x29, (uint8_t *)parameter, 8);
    //LCD_WriteReg(hlcdc, 0x29, (uint8_t *)parameter, 6);

    for (uint32_t i = 0; i < sizeof(H034A01_NV3051_800x800_init_cmds) / sizeof(H034A01_NV3051_800x800_init_cmds[0]); i++)
    {
        LCD_WriteReg(hlcdc, H034A01_NV3051_800x800_init_cmds[i][0], (uint8_t *)&H034A01_NV3051_800x800_init_cmds[i][1], 1);
    }

    /* Display ON command */
    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(200); //Delay 5 ms after sleep out

    /* Display ON command */
    LCD_WriteReg(hlcdc, 0x29, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(120); //Wait TE signal ready

#if 0

    /*Clear gram*/
    HAL_LCDC_Enable_TE(hlcdc, 0);
    LCD_SetRegion(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH - 1, THE_LCD_PIXEL_HEIGHT - 1);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 255, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, REG_WRITE_RAM, 1);

    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
#ifdef LCD_LCD_VSYNC_ENABLE
    HAL_LCDC_Enable_TE(hlcdc, 1);
#endif /* LCD_LCD_VSYNC_ENABLE */

    LCD_DRIVER_DELAY_MS(1000);
#endif /* 0 */

    data = LCD_ReadData(hlcdc, 0x52, 1);
    DEBUG_PRINTF("\nLCD_ReadBrightness 0x%x \n", data);

    {
        /*

            Bit Description Value
            D7 Booster Voltage Status "1"=Booster On, "0"=Booster Off
            D6 Not Defined Set to "0" (not used)
            D5 Not Defined Set to "0" (not used)
            D4 Sleep In/Out "1" = Sleep Out Mode, "0" = Sleep In Mode
            D3 Not Defined Set to "1" (not used)
            D2 Display On/Off "1" = Display is On, "0" = Display is Off
            D1 Not Defined Set to "0" (not used)
            D0 Not Defined Set to "0" (not used)

        */
        data = LCD_ReadData(hlcdc, REG_POWER_MODE, 1);
        DEBUG_PRINTF("PowerMode 0x%x", data);//0x9c
    }

    HAL_LCDC_SetROIArea(hlcdc, 0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);

    parameter[0] = 0xff;
    LCD_WriteReg(hlcdc, 0x51, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x36, parameter, 1);

    data = LCD_ReadData(hlcdc, 0x52, 1);
    DEBUG_PRINTF("\nLCD_ReadBrightness 0x%x \n", data);

    data = LCD_ReadData(hlcdc, 0x0b, 1);
    DEBUG_PRINTF("\nLCD_0bh= 0x%x \n", data);

    data = LCD_ReadData(hlcdc, 0x0c, 1);
    DEBUG_PRINTF("\nLCD_0ch= 0x%x \n", data);

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
    DEBUG_PRINTF("\nLCD_ReadID 0x%x \n", data);//0X25230

    if (data)
    {
        DEBUG_PRINTF("LCD module use LCD IC \n");
        data = LCD_ID;
    }

    data = LCD_ID;

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
#if 0
    uint32_t size;
    static bool video_mode = true;

    if (!video_mode)
    {
        video_mode = true;
        /* Switch to video mode ----------------------------------*/
        //DEBUG_PRINTF("Switch to video mode \n");
        memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dsi_video, sizeof(lcdc_int_cfg));
        memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
        HAL_LCDC_Init(hlcdc);
    }
#endif /* 0 */

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    DEBUG_PRINTF("NOT support read pixel\n");

    return 0; //Not support read pixel
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

        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0x77;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    //LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define LCD_BRIGHTNESS_MAX 0xFF

static void     LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)LCD_BRIGHTNESS_MAX * br / 100);
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

static void  TimeoutDbg(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    data = LCD_ReadData(hlcdc, REG_POWER_MODE, 1);
    DEBUG_PRINTF("TimeoutDbg PowerMode 0x%x", data);//0x9c
}

static void TimeoutReset(LCDC_HandleTypeDef *hlcdc)
{
    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(100);
    BSP_LCD_Reset(1);
    LCD_DRIVER_DELAY_MS(10);

    LCD_Init(hlcdc);
}

static uint32_t  ESDCehck(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    data = LCD_ReadData(hlcdc, REG_POWER_MODE, 1);
    DEBUG_PRINTF("ESDCehck PowerMode 0x%x", data);//0x9c

    return 0;

}

static const LCD_DrvOpsDef LCD_drv =
{
    .Init = LCD_Init,
    .ReadID = LCD_ReadID,
    .DisplayOn = LCD_DisplayOn,
    .DisplayOff = LCD_DisplayOff,

    .SetRegion = LCD_SetRegion,
    .WritePixel = LCD_WritePixel,
    .WriteMultiplePixels = LCD_WriteMultiplePixels,

    .ReadPixel = LCD_ReadPixel,

    .SetColorMode = LCD_SetColorMode,
    .SetBrightness = LCD_SetBrightness,
    .IdleModeOn = LCD_IdleModeOn,
    .IdleModeOff = LCD_IdleModeOff,

    .TimeoutDbg = TimeoutDbg,
    .TimeoutReset = TimeoutReset,
    .ESDDetect = ESDCehck,
};

LCD_DRIVER_EXPORT2(nv3051f1, LCD_ID, &lcdc_int_cfg,
                   &LCD_drv, 2);

