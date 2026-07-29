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
#define LOG_TAG                "jd9365da"

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
    .color_mode = LCDC_PIXEL_FORMAT_RGB888, //LCDC_PIXEL_FORMAT_RGB888

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
                .HS_width      = 4,   //HLW

                .VBP = 18,//12
                .VAH = LCD_VER_RES_MAX,
                .VFP = 20,

                .HBP = 26,//24
                .HAW = LCD_HOR_RES_MAX,
                .HFP = 26,

                .interrupt_line_num = 1,
            },
        },
    },
};

#define MAX_CMD_LEN 2

static const uint8_t H080A11HDIFT4C30_V0_1__800x1280_init_cmds[][MAX_CMD_LEN] =
{
    {0xE0, 0x00}, //Page 00

    {0xE1, 0x93},
    {0xE2, 0x65},
    {0xE3, 0xF8},
    {0x80, 0x01}, /*DSI INITO[1:0]: Lane Num.    2'b00 1 lane    2'b01 2 lane    2'b10 3 lane    2'b11 4 lane*/

    {0xE0, 0x01}, //Page 01
    {0x03, 0x00},
    {0x04, 0x2F},

    {0x17, 0x10},
    {0x18, 0x0F},
    {0x19, 0x01},
    {0x1A, 0x10},
    {0x1B, 0x0F},
    {0x1C, 0x01},

    {0x24, 0xFE},
    {0x25, 0x20},
    {0x35, 0x23},
    {0x37, 0x09},

    {0x38, 0x04},
    {0x39, 0x08},
    {0x3A, 0x12},
    {0x3C, 0x78},
    {0x3D, 0xFF},
    {0x3E, 0xFF},
    {0x3F, 0xFF},

    {0x40, 0x06},
    {0x41, 0xA0},
    {0x43, 0x14},
    {0x44, 0x0F},
    {0x45, 0x30},
    {0x4B, 0x04},

    {0x0C, 0x74},
    {0x55, 0x02},
    {0x57, 0x65},
    {0x59, 0x0A},
    {0x5A, 0x28},
    {0x5B, 0x0F},

    {0x5D, 0x7C},
    {0x5E, 0x64},
    {0x5F, 0x53},
    {0x60, 0x45},
    {0x61, 0x3E},
    {0x62, 0x2E},
    {0x63, 0x31},
    {0x64, 0x19},
    {0x65, 0x31},
    {0x66, 0x30},
    {0x67, 0x2E},
    {0x68, 0x4A},
    {0x69, 0x35},
    {0x6A, 0x3B},
    {0x6B, 0x2C},
    {0x6C, 0x2A},
    {0x6D, 0x1F},
    {0x6E, 0x13},
    {0x6F, 0x0C},
    {0x70, 0x7C},
    {0x71, 0x64},
    {0x72, 0x53},
    {0x73, 0x45},
    {0x74, 0x3E},
    {0x75, 0x2E},
    {0x76, 0x31},
    {0x77, 0x19},
    {0x78, 0x31},
    {0x79, 0x30},
    {0x7A, 0x2E},
    {0x7B, 0x4A},
    {0x7C, 0x35},
    {0x7D, 0x3B},
    {0x7E, 0x2C},
    {0x7F, 0x2A},
    {0x80, 0x1F},
    {0x81, 0x13},
    {0x82, 0x0C},

    {0xE0, 0x02}, //Page 02

    {0x00, 0x5E},
    {0x01, 0x5F},
    {0x02, 0x57},
    {0x03, 0x58},
    {0x04, 0x44},
    {0x05, 0x46},
    {0x06, 0x48},
    {0x07, 0x4A},
    {0x08, 0x40},
    {0x09, 0x5F},
    {0x0A, 0x5F},
    {0x0B, 0x5F},
    {0x0C, 0x5F},
    {0x0D, 0x5F},
    {0x0E, 0x5F},
    {0x0F, 0x50},
    {0x10, 0x5F},
    {0x11, 0x5F},
    {0x12, 0x5F},
    {0x13, 0x5F},
    {0x14, 0x5F},
    {0x15, 0x5F},

    {0x16, 0x5E},
    {0x17, 0x5F},
    {0x18, 0x57},
    {0x19, 0x58},
    {0x1A, 0x45},
    {0x1B, 0x47},
    {0x1C, 0x49},
    {0x1D, 0x4B},
    {0x1E, 0x41},
    {0x1F, 0x5F},
    {0x20, 0x5F},
    {0x21, 0x5F},
    {0x22, 0x5F},
    {0x23, 0x5F},
    {0x24, 0x5F},
    {0x25, 0x51},
    {0x26, 0x5F},
    {0x27, 0x5F},
    {0x28, 0x5F},
    {0x29, 0x5F},
    {0x2A, 0x5F},
    {0x2B, 0x5F},

    {0x2C, 0x1F},
    {0x2D, 0x1E},
    {0x2E, 0x17},
    {0x2F, 0x18},
    {0x30, 0x0B},
    {0x31, 0x09},
    {0x32, 0x07},
    {0x33, 0x05},
    {0x34, 0x11},
    {0x35, 0x1F},
    {0x36, 0x1F},
    {0x37, 0x1F},
    {0x38, 0x1F},
    {0x39, 0x1F},
    {0x3A, 0x1F},
    {0x3B, 0x01},
    {0x3C, 0x1F},
    {0x3D, 0x1F},
    {0x3E, 0x1F},
    {0x3F, 0x1F},
    {0x40, 0x1F},
    {0x41, 0x1F},

    {0x42, 0x1F},
    {0x43, 0x1E},
    {0x44, 0x17},
    {0x45, 0x18},
    {0x46, 0x0A},
    {0x47, 0x08},
    {0x48, 0x06},
    {0x49, 0x04},
    {0x4A, 0x10},
    {0x4B, 0x1F},
    {0x4C, 0x1F},
    {0x4D, 0x1F},
    {0x4E, 0x1F},
    {0x4F, 0x1F},
    {0x50, 0x1F},
    {0x51, 0x00},
    {0x52, 0x1F},
    {0x53, 0x1F},
    {0x54, 0x1F},
    {0x55, 0x1F},
    {0x56, 0x1F},
    {0x57, 0x1F},

    {0x58, 0x40},
    {0x59, 0x00},
    {0x5A, 0x00},
    {0x5B, 0x30},
    {0x5C, 0x0B},
    {0x5D, 0x30},
    {0x5E, 0x01},
    {0x5F, 0x02},
    {0x60, 0x30},
    {0x61, 0x03},
    {0x62, 0x04},
    {0x63, 0x1C},
    {0x64, 0x6A},
    {0x65, 0x75},
    {0x66, 0x0F},
    {0x67, 0x73},
    {0x68, 0x0D},
    {0x69, 0x1C},
    {0x6A, 0x6A},
    {0x6B, 0x00},
    {0x6C, 0x00},
    {0x6D, 0x00},
    {0x6E, 0x00},
    {0x6F, 0x88},
    {0x70, 0x00},
    {0x71, 0x00},
    {0x72, 0x06},
    {0x73, 0x7B},
    {0x74, 0x00},
    {0x75, 0xBB},
    {0x76, 0x01},
    {0x77, 0x0D},
    {0x78, 0x24},
    {0x79, 0x00},
    {0x7A, 0x00},
    {0x7B, 0x00},
    {0x7C, 0x00},
    {0x7D, 0x03},
    {0x7E, 0x7B},

    {0xE0, 0x04}, //Page 04
    {0x02, 0x23},
    {0x09, 0x10},
    {0x0E, 0x4A},
    {0x36, 0x49},

    {0xE0, 0x00}, //Page 00

}
;

static LCDC_InitTypeDef lcdc_int_cfg;

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

    HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);

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
    LCD_DRIVER_DELAY_MS(100);
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(200); //LCD must at sleep in mode after power on, 10ms is enough

    //LCD_WriteReg(hlcdc, 0x29, (uint8_t *)parameter, 8);
    //LCD_WriteReg(hlcdc, 0x29, (uint8_t *)parameter, 6);

    for (uint32_t i = 0; i < sizeof(H080A11HDIFT4C30_V0_1__800x1280_init_cmds) / sizeof(H080A11HDIFT4C30_V0_1__800x1280_init_cmds[0]); i++)
    {
        LCD_WriteReg(hlcdc, H080A11HDIFT4C30_V0_1__800x1280_init_cmds[i][0], (uint8_t *)&H080A11HDIFT4C30_V0_1__800x1280_init_cmds[i][1], 1);
    }

    /* Display ON command */
    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(800); //Delay 5 ms after sleep out

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
        DEBUG_PRINTF("0ah=0x%x", data);//0x9c
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
    //LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
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

LCD_DRIVER_EXPORT2(jd9365da, LCD_ID, &lcdc_int_cfg,
                   &LCD_drv, 2);

