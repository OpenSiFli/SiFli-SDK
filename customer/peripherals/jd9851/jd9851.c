/**
  ******************************************************************************
  * @file   jd9851.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for JD9851 LCD.
  * @attention
  ******************************************************************************
*/


#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#define DBG_TAG               "jd9851"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>










#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)



/**
  * @brief JD9851 chip IDs
  */
#define THE_LCD_ID                  0x1190a7

/**
  * @brief  JD9851 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (240)
#define  THE_LCD_PIXEL_HEIGHT   (286)






/**
  * @brief  JD9851 Registers
  */
//#define REG_SW_RESET           0x01
#define REG_LCD_ID             0xA1
//#define REG_DSI_ERR            0x05
#define REG_POWER_MODE         0x0A
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




#define REG_TEARING_EFFECT     0x35

//#define REG_IDLE_MODE_OFF      0x38
//#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/





































#define LCD_3V3_POWER_PIN               55
//#define LCD_1V8_POWER_PIN               47
#define LCD_BACKLIGHT_POWER_PIN         47
#define LCD_RESET_PIN                   44     // GPIO_A44  jd9851


#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*jd9851 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)










#ifdef BSP_LCDC_USING_DSI

#if 0//def APP_BSP_TEST  //Keep two data lanes for bsp test
    #define  JD9851_DSI_FREQ       DSI_FREQ_240MHZ
    #define  JD9851_DSI_DATALANES  DSI_TWO_DATA_LANES
#else
    #define  JD9851_DSI_FREQ       DSI_FREQ_480MHZ
    #define  JD9851_DSI_DATALANES  DSI_ONE_DATA_LANE
#endif /* APP_BSP_TEST */



static const LCDC_InitTypeDef lcdc_int_cfg_dsi =
{
    .lcd_itf = LCDC_INTF_DSI,
    .freq = JD9851_DSI_FREQ, //JD9851 RGB565 only support 320Mbps,  RGB888 support 500Mbps
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {

        .dsi = {

            .Init = {
                .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,
                .NumberOfLanes = JD9851_DSI_DATALANES,
                .TXEscapeCkdiv = 0x4,
            },

            .CmdCfg = {
                .VirtualChannelID      = 0,
                .CommandSize           = 0xFFFF,
#ifdef LCD_JD9851_VSYNC_ENABLE
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE,     //Open TE
#else
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Close TE
#endif /* LCD_JD9851_VSYNC_ENABLE */
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
    .freq = 48000000,        //JD9851 RGB565 only support 48000000,  RGB888 support 60000000
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,//LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
#ifdef LCD_JD9851_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_JD9851_VSYNC_ENABLE */
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
    .lcd_itf = LCDC_INTF_SPI_DCX_1DATA,
    .freq = 24000000,
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


static LCDC_InitTypeDef lcdc_int_cfg;

static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);










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


static void JD9851_Drv_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[32];

    /* Initialize JD9851 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    rt_pin_mode(LCD_3V3_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_3V3_POWER_PIN, 1); /* LCD_3V3 ON */
#if 0
    rt_pin_mode(LCD_1V8_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_1V8_POWER_PIN, 0); /* LCD_1V8 ON */
#endif
    //rt_pin_mode(LCD_BACKLIGHT_POWER_PIN, PIN_MODE_OUTPUT);
    //rt_pin_write(LCD_BACKLIGHT_POWER_PIN, 0);

    //BSP_LCD_Reset(0);//Reset LCD
    rt_pin_mode(LCD_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_RESET_PIN, 0);
    HAL_Delay_us(20);
    //BSP_LCD_Reset(1);
    rt_pin_write(LCD_RESET_PIN, 1);

    /* Wait for 200ms */
    LCD_DRIVER_DELAY_MS(100);


    parameter[0] = 0x98;
    parameter[1] = 0x51;
    parameter[2] = 0xE9;
    LCD_WriteReg(hlcdc, 0xDF, parameter, 3);

    parameter[1] = 0x00;
    LCD_WriteReg(hlcdc, 0xDE, parameter, 1);

    parameter[0] = 0x1E;
    parameter[1] = 0x7D;
    parameter[2] = 0x1E;
    parameter[3] = 0x2B;
    LCD_WriteReg(hlcdc, 0xB7, parameter, 4);

    parameter[0] = 0x3F;      //0x3F
    parameter[1] = 0x37;  //0x3A
    parameter[2] = 0x38;  //0x36
    parameter[3] = 0x38;  //0x38
    parameter[4] = 0x3B;  //0x3A
    parameter[5] = 0x3F;  //0x3C
    parameter[6] = 0x3C;  //0x36
    parameter[7] = 0x3B;  //0x35
    parameter[8] = 0x3A;  //0x32
    parameter[9] = 0x38;  //0x30
    parameter[10] = 0x34;  //0x2A
    parameter[11] = 0x26;  //0x1A
    parameter[12] = 0x1E;  //0x16
    parameter[13] = 0x13;  //0x0B
    parameter[14] = 0x16;  //0x04
    parameter[15] = 0x0E;  //0x0E
    parameter[16] = 0x3F;    //0x3F
    parameter[17] = 0x37;  //0x3A
    parameter[18] = 0x38;  //0x36
    parameter[19] = 0x38;  //0x38
    parameter[20] = 0x3B;  //0x3A
    parameter[21] = 0x3F;  //0x3C
    parameter[22] = 0x3C;  //0x36
    parameter[23] = 0x3B;  //0x35
    parameter[24] = 0x3A;  //0x32
    parameter[25] = 0x38;  //0x30
    parameter[26] = 0x34;  //0x2A
    parameter[27] = 0x26;  //0x1A
    parameter[28] = 0x1E;  //0x16
    parameter[29] = 0x13;  //0x0B
    parameter[30] = 0x16;  //0x04
    parameter[31] = 0x0E;  //0x0E
    LCD_WriteReg(hlcdc, 0xC8, parameter, 32);

    parameter[0] = 0x23;
    parameter[1] = 0x28;
    parameter[2] = 0xCC;
    LCD_WriteReg(hlcdc, 0xB9, parameter, 3);

    parameter[0] = 0x47;
    parameter[1] = 0x7F;
    parameter[2] = 0x30;
    parameter[3] = 0xC0;
    parameter[4] = 0x5C;
    parameter[5] = 0x60;
    parameter[6] = 0x40;
    parameter[7] = 0x70;
    LCD_WriteReg(hlcdc, 0xBB, parameter, 8);

    parameter[0] = 0x38;
    parameter[1] = 0x3C;
    LCD_WriteReg(hlcdc, 0xBC, parameter, 2);

    parameter[0] = 0x11;
    parameter[1] = 0x20;
    LCD_WriteReg(hlcdc, 0xC0, parameter, 2);

    parameter[0] = 0x12;
    LCD_WriteReg(hlcdc, 0xC1, parameter, 1);

    parameter[0] = 0x08;
    parameter[1] = 0x00;
    parameter[2] = 0x0A;
    parameter[3] = 0x10;
    parameter[4] = 0x08;
    parameter[5] = 0x54;
    parameter[6] = 0x45;
    parameter[7] = 0x71;
    parameter[8] = 0x2C;
    LCD_WriteReg(hlcdc, 0xC3, parameter, 9);

    parameter[0] = 0x00;
    parameter[1] = 0xA0;
    parameter[2] = 0x79;
    parameter[3] = 0x0E;
    parameter[4] = 0x0A;
    parameter[5] = 0x16;
    parameter[6] = 0x79;
    parameter[7] = 0x0E;
    parameter[8] = 0x0A;
    parameter[9] = 0x16;
    parameter[10] = 0x79;
    parameter[11] = 0x0E;
    parameter[12] = 0x0A;
    parameter[13] = 0x16;
    parameter[14] = 0x82;
    parameter[15] = 0x00;
    parameter[16] = 0x03;
    LCD_WriteReg(hlcdc, 0xC4, parameter, 17);

    parameter[0] = 0x04;
    parameter[1] = 0x0C;
    parameter[2] = 0x6B;
    parameter[3] = 0x0F;
    parameter[4] = 0x01;
    parameter[5] = 0x03;
    LCD_WriteReg(hlcdc, 0xD0, parameter, 6);

    parameter[0] = 0x00;
    parameter[1] = 0x00;
    LCD_WriteReg(hlcdc, 0xD7, parameter, 2);

    parameter[0] = 0x02;
    LCD_WriteReg(hlcdc, 0xDE, parameter, 1);

    parameter[0] = 0x1D;
    parameter[1] = 0xA0;
    parameter[2] = 0x2F;
    parameter[3] = 0x04;
    parameter[4] = 0x24;
    LCD_WriteReg(hlcdc, 0xB8, parameter, 5);

    parameter[0] = 0x30;
    parameter[1] = 0xF8;
    parameter[2] = 0x93;
    parameter[4] = 0x6E;
    LCD_WriteReg(hlcdc, 0xB2, parameter, 5);

    parameter[0] = 0x10;
    parameter[1] = 0x66;
    parameter[2] = 0x66;
    parameter[3] = 0x01;
    LCD_WriteReg(hlcdc, 0xC1, parameter, 4);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xDE, parameter, 1);

    LCD_WriteReg(hlcdc, 0x11, (uint8_t *)NULL, 0);
    LCD_DRIVER_DELAY_MS(120);

    parameter[0] = 0x02;
    LCD_WriteReg(hlcdc, 0xDE, parameter, 1);

    parameter[0] = 0x01;    //10MHz
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    LCD_WriteReg(hlcdc, 0xC5, parameter, 3);

    parameter[0] = 0x10;
    parameter[1] = 0x20;
    parameter[2] = 0xF4;
    LCD_WriteReg(hlcdc, 0xCA, parameter, 3);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xDE, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, parameter, 1);

    parameter[0] = 0x05;    //rgb 666 bit
    LCD_WriteReg(hlcdc, 0x3A, parameter, 1);

    LCD_WriteReg(hlcdc, 0x29, (uint8_t *)NULL, 0);

    /* Wait for 100ms */
    LCD_DRIVER_DELAY_MS(20);

    return;
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
 */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{

#ifdef BSP_LCDC_USING_QADSPI
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_qadspi, sizeof(lcdc_int_cfg));
#elif defined(BSP_LCDC_USING_DSI)
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_dsi, sizeof(lcdc_int_cfg));
#else
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_3spi_1data, sizeof(lcdc_int_cfg));
#endif

    JD9851_Drv_Init(hlcdc);
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
    DEBUG_PRINTF("\nJD9851_ReadID 0x%x \n", data);
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
        DEBUG_PRINTF("JD9851_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("JD9851_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
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
        uint32_t cmd;

        if (REG_WRITE_RAM == LCD_Reg)
        {
            cmd = (0x32 << 24) | (LCD_Reg << 8);
        }
        else
        {
            cmd = (0x02 << 24) | (LCD_Reg << 8);
        }

        HAL_LCDC_WriteU32Reg(hlcdc, cmd, Parameters, NbParameters);
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
    DEBUG_PRINTF("JD9851_ReadPixel[%d,%d]\n", Xpos, Ypos);

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

#define LCD_BACKLIGHT_USING_PWM
#define JD9851_BRIGHTNESS_MAX 0xFF

#ifdef LCD_BACKLIGHT_USING_PWM
    #define LCD_BACKLIGHT_PWM_DEV_NAME "pwm2"
    #define LCD_BACKLIGHT_PWM_PERIOD (1 * 1000 * 1000)
    #define LCD_BACKLIGHT_PWM_CHANNEL 4
#endif

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
#ifndef LCD_BACKLIGHT_USING_PWM
    /* PA70 Backlight ,PA47 1V8_EN*/
    uint8_t bright = (uint8_t)((int)JD9851_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
    rt_pin_mode(LCD_BACKLIGHT_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_BACKLIGHT_POWER_PIN, 1);
    LOG_I("JD9851_SetBrightness,br:%d\n", br);
    /* PA70 Backlight */

#else

    /* PA47 Backlight PWM,PA70_NC*/
    rt_uint32_t pulse = br * LCD_BACKLIGHT_PWM_PERIOD / 100;
    struct rt_device_pwm *device = RT_NULL;

    device = (struct rt_device_pwm *)rt_device_find(LCD_BACKLIGHT_PWM_DEV_NAME);
    if (!device)
    {
        LOG_I("find pwm:LCD_BACKLIGHT_PWM_DEV_NAME err!", br, pulse);
        return;
    }

    rt_pwm_set(device, LCD_BACKLIGHT_PWM_CHANNEL, LCD_BACKLIGHT_PWM_PERIOD, pulse);
    rt_pwm_enable(device, LCD_BACKLIGHT_PWM_CHANNEL);

    LOG_I("JD9851_SetBrightness,br:%d,pulse:%d\n", br, pulse);
    /* PA47 Backlight PWM */
#endif
}





static const LCD_DrvOpsDef JD9851_drv =
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

LCD_DRIVER_EXPORT2(jd9851, THE_LCD_ID, &lcdc_int_cfg,
                   &JD9851_drv, 2);








/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
