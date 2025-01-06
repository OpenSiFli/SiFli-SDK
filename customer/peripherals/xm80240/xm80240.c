/**
  ******************************************************************************
  * @file   xm80240.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for XM80240 LCD.
  * @attention
  ******************************************************************************
*/


#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#include "log.h"









#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)





/**
  * @brief XM80240 chip IDs
  */
#define THE_LCD_ID                  0x1190a7

/**
  * @brief  XM80240 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (390)
#define  THE_LCD_PIXEL_HEIGHT   (450)






/**
  * @brief  XM80240 Registers
  */
//#define REG_SW_RESET           0x01
#define REG_LCD_ID             0xA1
//#define REG_DSI_ERR            0x05
//#define REG_POWER_MODE         0x0A
#define REG_SLEEP_IN           0x10
#define REG_SLEEP_OUT          0x11
//#define REG_PARTIAL_DISPLAY    0x12
//#define REG_DISPLAY_INVERSION  0x21
#define REG_DISPLAY_OFF        0x28
#define REG_DISPLAY_ON         0x29
#define REG_WRITE_RAM          0x2C
#define REG_READ_RAM           0x2E
#define REG_CASET              0x2A
#define REG_RASET              0x2B




//#define REG_TEARING_EFFECT     0x35

//#define REG_IDLE_MODE_OFF      0x38
//#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/



































#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*xm80240 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)











#ifdef BSP_LCDC_USING_DSI

#if 0//def APP_BSP_TEST  //Keep two data lanes for bsp test
    #define  XM80240_DSI_FREQ       DSI_FREQ_240Mbps
    #define  XM80240_DSI_DATALANES  DSI_TWO_DATA_LANES
#else
    #define  XM80240_DSI_FREQ       DSI_FREQ_480Mbps
    #define  XM80240_DSI_DATALANES  DSI_ONE_DATA_LANE
#endif /* APP_BSP_TEST */



static const LCDC_InitTypeDef lcdc_int_cfg_dsi =
{
    .lcd_itf = LCDC_INTF_DSI,
    .freq = XM80240_DSI_FREQ, //XM80240 RGB565 only support 320Mbps,  RGB888 support 500Mbps
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {

        .dsi = {

            .Init = {
                .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,
                .NumberOfLanes = XM80240_DSI_DATALANES,
                .TXEscapeCkdiv = 0x4,
            },

            .CmdCfg = {
                .VirtualChannelID      = 0,
                .CommandSize           = 0xFFFF,
#ifdef LCD_XM80240_VSYNC_ENABLE
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE,     //Open TE
#else
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Close TE
#endif /* LCD_XM80240_VSYNC_ENABLE */
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
    .freq = 48000000,        //XM80240 RGB565 only support 48000000,  RGB888 support 60000000
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,//LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
#if 1 //#ifdef LCD_XM80240_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_XM80240_VSYNC_ENABLE */
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
MSH_CMD_EXPORT(XM80240_Init, XM80240_Init);


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

        DEBUG_PRINTF("\nXM80240_Read reg[%x] %d(byte)\n", reg, len);

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
    DEBUG_PRINTF("\nXM80240_Write reg[%x] %d(byte) done.\n", reg, argc - 2);

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

#define MAX_CMD_LEN        18


static const uint8_t lcd_init_cmds[][MAX_CMD_LEN] =
{
    //#==== Initial Code Start =======
    //# Set XM Command Password 1
    {0x00, 1, 0x00},
    {0xFF, 3, 0x02, 0x40, 0x01},

    //#Set XM Command Password 2
    {0x00, 1, 0x80},
    {0xFF, 2, 0x02, 0x40},

    {0x00, 1, 0x80},
    {0xC1, 1, 0xE0},


//      {0x00, 1 ,0x81},
//      {0xA2, 1 ,0x0D},


    {0x00, 1, 0x99},
    {0xA4, 1, 0x70},


    {0x00, 1, 0x82},
    {0xB0, 1, 0x80},

    {0x00, 1, 0xA6},
    {0xC0, 1, 0x30},

    {0x00, 1, 0x80},
    {0xA5, 1, 0x1b},

//      {0x00, 1 ,0xA1},
//      {0xA4, 1 ,0x80},


    //#======== x_resolution 320 TM1.41 ========


    {0x00, 1, 0xA1},
    {0xC0, 4, 0x01, 0x86, 0x01, 0xC2}, //#390x450
    //{0xC0, 4 ,0x01, 0x40, 0x01, 0x7C}, //320*380

    {0x00, 1, 0xA0},
    {0xA5, 8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00},
    {0x00, 1, 0xA8},
    {0xA5, 8, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F},
    {0x00, 1, 0x91},
    {0xA5, 2, 0x77, 0x60},
    {0x00, 1, 0x94},
    {0xA5, 1, 0x40},
    {0x00, 1, 0xC0},
    {0xA5, 8, 0xFF, 0xF4, 0x3F, 0xFF, 0x73, 0x10, 0x01, 0x37},
    {0x00, 1, 0xC8},
    {0xA5, 3, 0x74, 0x00, 0x80},
    {0x00, 1, 0xB0},
    {0xA5, 1, 0x00},


    {0x00, 1, 0x82},
    {0xB2, 2, 0x00, 0x02},


    {0x00, 1, 0x80},
    {0xB2, 2, 0x00, 0x79}, //#44.8

    {0x00, 1, 0x80},
    {0xb4, 11, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71, 0x00, 0x00, 0x09},

    {0x00, 1, 0x90},
    {0xb4, 11, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71, 0x00, 0x00, 0x09},


    {0x00, 1, 0x80},
    {0xb5, 7, 0x80, 0x00, 0x3C, 0x50, 0x35, 0x00, 0x81},


    {0x00, 1, 0x87},
    {0xb5, 7, 0x81, 0x00, 0x3C, 0x50, 0x35, 0x00, 0x81},


    {0x00, 1, 0x80},
    {0xb6, 6, 0x83, 0x06, 0x00, 0x00, 0x00, 0x00},


    {0x00, 1, 0x86},
    {0xb6, 6, 0x82, 0x81, 0x00, 0x00, 0x00, 0x00},


    {0x00, 1, 0x80},
    {0xbc, 16, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x15, 0x00, 0x0E},
    {0x00, 1, 0x90},
    {0xbc, 16, 0x06, 0x00, 0x05, 0x00, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},


    {0x00, 1, 0xa0},
    {0xbc, 16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x2F, 0x2E, 0x2D, 0x2C, 0x2B, 0x16, 0x15, 0x0D},
    {0x00, 1, 0xb0},
    {0xbc, 16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x2F, 0x2E, 0x2D, 0x2C, 0x2B, 0x06, 0x05, 0x0E},


    {0x00, 1, 0x80},
    {0xb9, 16, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0x00, 0x56, 0x00, 0x56},
    {0x00, 1, 0x90},
    {0xb9, 16, 0x55, 0x00, 0x55, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    {0x00, 1, 0xa0},
    {0xb9, 16, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0x00, 0x95, 0x00, 0x95},
    {0x00, 1, 0xb0},
    {0xb9, 16, 0x95, 0x00, 0x95, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},


    {0x00, 1, 0x80},
    {0xba, 11, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa},


    {0x00, 1, 0x8A},
    {0xB2, 1, 0x09},


    {0x00, 1, 0x89},
    {0xB2, 1, 0x00},


    {0x00, 1, 0x8B},
    {0xB2, 1, 0x03},


    {0x00, 1, 0xB0},
    {0xC9, 5, 0xff, 0xff, 0xff, 0xff, 0xff},
    //#==============

    //##ERP5532A PowerIC Ctrl
    {0x00, 1, 0xA0},
    {0xC1, 1, 0x20}, //#swire enable

    {0x00, 1, 0xB0},
    {0xC1, 1, 0x10},

    {0x00, 1, 0xB3},
    {0xC1, 1, 0x00},

    {0x00, 1, 0xAB},
    {0xC1, 1, 0x54}, //#ELVDD=3.3V 43pulse

    {0x00, 1, 0xA9},
    {0xC1, 1, 0x2B}, //#ELVSS=-3.3V 84pulse
    //#======== voltage ========

    {0x00, 1, 0x94},
    {0xAB, 1, 0xAD},  //# VGL=-6.5V

    {0x00, 1, 0x96},
    {0xAB, 1, 0xA3},  //# VGH=-6.5V

    {0x00, 1, 0xF1},
    {0xA4, 2, 0x28, 0x1E},  //# VGLO=-6V VGH0=6V

    {0x00, 1, 0xB0},
    {0xA4, 4, 0x01, 0x92, 0x00, 0x75}, //# VGMP=5.0V VGSP=0.7V

    {0x00, 1, 0xA3},
    {0xA4, 1, 0x1E},  //# VREFN=-3V



    //#======== Visual effect ========

    //##Gamma Code4
    {0x00, 1, 0x80},
    {0xD1, 16, 0x00, 0x53, 0x56, 0x5B, 0x60, 0x64, 0x68, 0x6C, 0x6F, 0x72, 0x75, 0x77, 0x7D, 0x81, 0x86, 0x8E},

    {0x00, 1, 0x90},
    {0xD1, 16, 0x95, 0x9C, 0xA3, 0xAA, 0xB0, 0xB6, 0xBC, 0xC2, 0xC8, 0xCD, 0xD4, 0x01, 0x14, 0x77, 0x52, 0x42},

    {0x00, 1, 0xA0},
    {0xD1, 9, 0x17, 0x26, 0x12, 0x67, 0x50, 0x00, 0x10, 0x07, 0x00},

    {0x00, 1, 0x80},
    {0xD2, 16, 0x00, 0x55, 0x58, 0x60, 0x66, 0x6A, 0x6E, 0x72, 0x75, 0x78, 0x7B, 0x7D, 0x83, 0x87, 0x8C, 0x94},

    {0x00, 1, 0x90},
    {0xD2, 16, 0x9B, 0xA2, 0xA9, 0xAF, 0xB5, 0xBB, 0xC1, 0xC7, 0xCD, 0xD3, 0xD9, 0x01, 0x65, 0x25, 0x30, 0x31},

    {0x00, 1, 0xA0},
    {0xD2, 9,  0x15, 0x06, 0x01, 0x64, 0x46, 0x66, 0x66, 0x42, 0x00},

    {0x00, 1, 0x80},
    {0xD3, 16, 0x00, 0x6B, 0x6C, 0x72, 0x77, 0x7B, 0x7E, 0x82, 0x85, 0x89, 0x8B, 0x8E, 0x94, 0x99, 0x9E, 0xA7},

    {0x00, 1, 0x90},
    {0xD3, 16, 0xAF, 0xB7, 0xBE, 0xC5, 0xCC, 0xD2, 0xD9, 0xE0, 0xE6, 0xED, 0xF4, 0x00, 0x73, 0x22, 0x72, 0x71},

    {0x00, 1, 0xA0},
    {0xD3, 9, 0x52, 0x01, 0x30, 0x02, 0x22, 0x05, 0x50, 0x64, 0x30},



    {0x00, 1, 0xE0},
    {0xD0, 2, 0x10, 0x42},

    {0x00, 1, 0xE2},
    {0xD0, 8, 0x5E, 0x68, 0x5E, 0x4A, 0x00, 0x00, 0x00, 0x00},

    {0x00, 1, 0xEA},
    {0xD0, 8, 0x5D, 0x67, 0x5D, 0x67, 0x00, 0x00, 0x00, 0x00},

    {0x00, 1, 0xF3},
    {0xD0, 4, 0x00, 0x00, 0x00, 0x00},



    {0x00, 1, 0xC0},
    {0xD0, 1, 0x00},

    {0x00, 1, 0xD6},
    {0xD0, 1, 0x06},



    {0x00, 1, 0x9a},
    {0xa5, 1, 0x00},

    {0x00, 1, 0x9d},
    {0xa5, 1, 0x10},

    {0x00, 1, 0x80},
    {0xAC, 1, 0x60},

    //#==== EM cmd51 =======
    {0x00, 1, 0xe5},
    {0xb6, 1, 0x00},

    {0x00, 1, 0xb0},
    {0xC9, 5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},

    {0x00, 1, 0xa0},
    {0xC9, 5, 0x00, 0x20, 0x7F, 0xBF, 0xFF},
    //#==========ADD==========

    {0x00, 1, 0xF1},
    {0xB6, 1, 0x40},

    {0x00, 1, 0xF0},
    {0xA4, 1, 0xA3},



    {0x00, 1, 0x83},
    {0xA6, 1, 0x83},

    {0x00, 1, 0x85},
    {0xA6, 1, 0x8D},

    {0x00, 1, 0x86},
    {0xA6, 6, 0x84, 0x8D, 0xC5, 0xC6, 0xC5, 0xC7},

    {0x00, 1, 0xA0},
    {0xA6, 2, 0xC8, 0xC8},

    {0x00, 1, 0xA4},
    {0xA6, 4, 0x00, 0x00, 0x00, 0x00},

    {0x00, 1, 0x90},
    {0xA6, 3, 0x8A, 0x8A, 0x8A},

    {0x00, 1, 0x94},
    {0xA6, 2, 0x8B, 0x8D},

    {0x00, 1, 0xC4},
    {0xA6, 2, 0x0D, 0x00},


    {0x00, 1, 0xC0},
    {0xA6, 2, 0x8A, 0x8D},

    {0x00, 1, 0x8C},
    {0xA6, 2, 0xC9, 0xC9},

    {0x00, 1, 0xA2},
    {0xA6, 2, 0xC9, 0xC9},

    //####
    {0x00, 1, 0xB0},
    {0xA6, 2, 0x1B, 0xB0},


    {0x00, 1, 0xC3},
    {0xA6, 1, 0x86},


    {0x00, 1, 0xc2},
    {0xb2, 1, 0x81},

    {0x00, 1, 0xf9},
    {0xb6, 1, 0x01},


    {0x00, 1, 0x9a},
    {0xa5, 1, 0x00},


    {0x00, 1, 0xA1},
    {0xC1, 1, 0x0B},

    {0x00, 1, 0xA3},
    {0xC1, 1, 0x02},


    {0x00, 1, 0xB0},
    {0xB2, 6, 0x00, 0x93, 0x00, 0x08, 0x00, 0x08},


    {0x00, 1, 0xF1},
    {0xB6, 1, 0x40},

    {0x00, 1, 0xF0},
    {0xA4, 1, 0xA3},

    {0x00, 1, 0x80},
    {0xFF, 2, 0x00, 0x00},

    {0x00, 1, 0x00},
    {0xFF, 3, 0x00, 0x00, 0x00},

    //#==== Initial Code End =======
    {0x3A, 1, 0xD5},   //0xD5 565
    {0x35, 1, 0x00}, //TE
    {0x44, 2, 0x00, 0x5A},
};



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


    /* Initialize XM80240 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    HAL_Delay_us(20);
    BSP_LCD_Reset(1);

    LCD_DRIVER_DELAY_MS(100); //LCD must at sleep in mode after power on, 10ms is enough.


    /* SW Reset Command */
    //LCD_WriteReg(hlcdc,REG_SW_RESET, (uint8_t *)NULL, 0);

    for (int i = 0; i < sizeof(lcd_init_cmds) / MAX_CMD_LEN; i++)
    {
        //rt_kprintf("write %d,cmd=0x%x,len=%d\n",i,(int)lcd_init_cmds[i][0], (int)lcd_init_cmds[i][1]);
        //HAL_DBG_print_data((char*)&(lcd_init_cmds[i][2]),0,(int)lcd_init_cmds[i][1]);
        LCD_WriteReg(hlcdc, lcd_init_cmds[i][0], (uint8_t *)&lcd_init_cmds[i][2], lcd_init_cmds[i][1]);

        //__asm("B .");
    }

    /* Wait for 110ms */
    LCD_WriteReg(hlcdc, 0x11, NULL, 0);
    LCD_DRIVER_DELAY_MS(20);
    LCD_WriteReg(hlcdc, 0x29, NULL, 0);
    LCD_DRIVER_DELAY_MS(50);

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
    DEBUG_PRINTF("\nXM80240_ReadID 0x%x \n", data);
    data = ((data << 24) & 0xFF000000)
           | ((data <<  8) & 0x00FF0000)
           | ((data >>  8) & 0x0000FF00)
           | ((data >> 24) & 0x000000FF);


    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        return THE_LCD_ID;
    }
    return THE_LCD_ID;

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
    LCD_ALIGN2(Xpos);
    LCD_ALIGN2(Ypos);

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
        DEBUG_PRINTF("XM80240_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("XM80240_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
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
        HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);
    }
    else
    {
        HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);
    }
    LCD_ReadMode(hlcdc, false);
#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

    return rd_data;
}



static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("XM80240_ReadPixel[%d,%d]\n", Xpos, Ypos);

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

    /*

    Control interface color format
    ?11?= 12bit/pixel ?01?= 16bit/pixel ?10?= 18bit/pixel ?11?= 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0xD5;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0xF7;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);
    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define XM80240_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)XM80240_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}



static const LCD_DrvOpsDef XM80240_drv =
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




LCD_DRIVER_EXPORT2(xm80240, THE_LCD_ID, &lcdc_int_cfg,
                   &XM80240_drv, 2);






/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
