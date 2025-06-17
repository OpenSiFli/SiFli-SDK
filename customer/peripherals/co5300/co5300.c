/**
  ******************************************************************************
  * @file   co5300.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for CO5300 LCD.
  * @attention
  ******************************************************************************
*/


#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"
#define DBG_TAG               "co5300"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>





#ifdef AM196Q410502LK_196_410x502
    #define COL_OFFSET (22)
    #define ROW_OFFSET (0)
#elif AM178Q368448LK_178_368x448
    #define COL_OFFSET (16)
    #define ROW_OFFSET (0)
#elif AM151Q466466LK_151_466x466_C
    #define COL_OFFSET (6)
    #define ROW_OFFSET (0)
#elif AM200Q460460LK_200_460x460
    #define COL_OFFSET (10)
    #define ROW_OFFSET (0)
#elif H0198S005AMT005_V0_195_410x502
    #define COL_OFFSET (44)
    #define ROW_OFFSET (0)
#else
    #define ROW_OFFSET  (0x00)
    #define COL_OFFSET  (0x00)
#endif

/**
  * @brief CO5300 chip IDs
  */

#ifdef AM196Q410502LK_196_410x502
    #define LCD_ID                  0x530001
#elif defined(AM151Q466466LK_151_466x466_C)
    #define LCD_ID                  0x530001
#elif defined(AM200Q460460LK_200_460x460)
    #define LCD_ID                  0x530001
#elif defined(H0198S005AMT005_V0_195_410x502)
    #define LCD_ID                  0x530001
#elif defined(AM178Q368448LK_178_368x448)
    #define LCD_ID                  0x530001
#else
    #define LCD_ID                  0x331100
#endif

/**
  * @brief  CO5300 Size
  */
#define  LCD_PIXEL_WIDTH    (LCD_HOR_RES_MAX)
#define  LCD_PIXEL_HEIGHT   (LCD_VER_RES_MAX)

/**
 *  @brief LCD_OrientationTypeDef
 *  Possible values of Display Orientation
 */
#define REG_ORIENTATION_PORTRAIT         (0x00) /* Portrait orientation choice of LCD screen  */
#define REG_ORIENTATION_LANDSCAPE        (0x01) /* Landscape orientation choice of LCD screen */
#define REG_ORIENTATION_LANDSCAPE_ROT180 (0x02) /* Landscape rotated 180 orientation choice of LCD screen */

/**
  * @brief  CO5300 Registers
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
#define REG_PART_CASET         0x30
#define REG_PART_RASET         0x31
#define REG_TEARING_EFFECT_OFF 0x34
#define REG_TEARING_EFFECT_ON  0x35
#define REG_MADCTL             0x36
#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A
#define REG_CONTINUE_WRITE_RAM 0x3C
#define REG_WBRIGHT            0x51 /* Write brightness*/
#define REG_RBRIGHT            0x52 /* Read brightness*/
#define REG_WRITE_CTRL_DISPLAY 0x53
#define REG_WRHBMDISBV         0x63
#define REG_SET_DISPLAY_MODE   0xC2
#define REG_SET_SPI_MODE       0xC4
#define REG_PASSWD1            0xF4
#define REG_PASSWD2            0xF5
#define REG_CMD_PAGE_SWITCH    0xFE

#define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)

#define REG_BRIGHTNESS_MAX 0xFF

#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA
#if defined(LCD_MAX_CLK_FREQ)
    .freq = LCD_MAX_CLK_FREQ,        //CO5300 RGB565 only support 50000000,  RGB888 support 60000000
#else
    .freq = 50000000, //CO5300 RGB565 only support 50000000,  RGB888 support 60000000
#endif

    .color_mode = LCDC_PIXEL_FORMAT_RGB565,//LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
#ifdef LCD_CO5300_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_CO5300_VSYNC_ENABLE */
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
        },
    },

};


static LCDC_InitTypeDef lcdc_int_cfg;

static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc);
static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1);
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

static void LCD_Clear(LCDC_HandleTypeDef *hlcdc)
{
    /*Clear gram*/
    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    LCD_SetRegion(hlcdc, 0, 0, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

}


static void LCD_Drv_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];
    /* Initialize CO5300 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(1);
    rt_thread_delay(10);
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_delay(10);
    BSP_LCD_Reset(1);
    rt_thread_delay(50);

    if (LCD_ReadID(hlcdc) !=  LCD_ID)
    {
        return;
    }

#ifdef AM196Q410502LK_196_410x502
    parameter[0] = 0x00;
#elif defined(AM151Q466466LK_151_466x466_C)
    parameter[0] = 0x00;
#elif defined(AM178Q368448LK_178_368x448)
    parameter[0] = 0x00;
#elif defined(AM200Q460460LK_200_460x460)
    parameter[0] = 0x00;
#elif defined(H0198S005AMT005_V0_195_410x502)
    parameter[0] = 0x00;
#else
    parameter[0] = 0x20;
#endif
    LCD_WriteReg(hlcdc, REG_CMD_PAGE_SWITCH, parameter, 1); //Pass word unlock
    parameter[0] = 0x5A;
    LCD_WriteReg(hlcdc, REG_PASSWD1, parameter, 1);
    parameter[0] = 0x59;
    LCD_WriteReg(hlcdc, REG_PASSWD2, parameter, 1);

    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, REG_CMD_PAGE_SWITCH, parameter, 1); //Pass word lock
    parameter[0] = 0xA5;
    LCD_WriteReg(hlcdc, REG_PASSWD1, parameter, 1);
    parameter[0] = 0xA5;
    LCD_WriteReg(hlcdc, REG_PASSWD2, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, REG_CMD_PAGE_SWITCH, parameter, 1);
    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, REG_SET_SPI_MODE, parameter, 1);
    parameter[0] = 0x55;
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, REG_TEARING_EFFECT_ON, parameter, 1);
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, REG_WRITE_CTRL_DISPLAY, parameter, 1);
    //parameter[0] = 0x10;
    //ICNA3310_WriteReg(hlcdc, REG_WBRIGHT, parameter, 1);

    parameter[0] = 0xff;
    LCD_WriteReg(hlcdc, REG_WRHBMDISBV, parameter, 1);

    parameter[0] = (COL_OFFSET >> 8) & 0xFF;
    parameter[1] = COL_OFFSET & 0xFF;
    parameter[2] = ((LCD_PIXEL_WIDTH + COL_OFFSET - 1) >> 8) & 0xFF;
    parameter[3] = (LCD_PIXEL_WIDTH + COL_OFFSET - 1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);
    parameter[0] = (ROW_OFFSET >> 8) & 0xFF;
    parameter[1] = ROW_OFFSET & 0xFF;
    parameter[2] = ((LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) >> 8) & 0xFF;
    parameter[3] = (LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
    //sleep out+display on
    rt_thread_delay(120);
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
    rt_thread_delay(70);
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
#endif /* BSP_LCDC_USING_QADSPI */

    LCD_Drv_Init(hlcdc);
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
    rt_kprintf("\nCO5300_ReadID 0x%x \n", data);
    if (data == LCD_ID)
        DEBUG_PRINTF("LCD module use CO5300 IC \n");
#ifdef AM196Q410502LK_196_410x502
    data = LCD_ID; // force to return CO5300 ID
#elif defined(AM178Q368448LK_178_368x448)
    data = LCD_ID;
#elif defined(AM151Q466466LK_151_466x466_C)
    data = LCD_ID;
#elif defined(AM200Q460460LK_200_460x460)
    data = LCD_ID;
#elif defined(H0198S005AMT005_V0_195_410x502)
    data = LCD_ID;
#endif
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
    HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);

}


/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
    uint32_t cmd;

    if ((REG_WRITE_RAM == LCD_Reg) || (REG_CONTINUE_WRITE_RAM == LCD_Reg))
    {
        cmd = (0x32 << 24) | (LCD_Reg << 8);
    }
    else
    {
        cmd = (0x02 << 24) | (LCD_Reg << 8);
    }

    HAL_LCDC_WriteU32Reg(hlcdc, cmd, Parameters, NbParameters);

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

    HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);


    LCD_ReadMode(hlcdc, false);

    return rd_data;
}



static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("CO5300_ReadPixel[%d,%d]\n", Xpos, Ypos);

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

    return ret_v;
}


static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];

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

static void  LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)REG_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}

/**
  * @brief  Enable the Display idle mode.
  * @param  None
  * @retval None
  */
static void LCD_IdleModeOn(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);

    /* Idle mode On */
    LCD_WriteReg(hlcdc, REG_IDLE_MODE_ON, NULL, 0);
}

/**
  * @brief  Disables the Display idle mode.
  * @param  None
  * @retval None
  */
static void LCD_IdleModeOff(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xFE, parameter, 1);

    /* Idle mode Off */
    LCD_WriteReg(hlcdc, REG_IDLE_MODE_OFF, NULL, 0);

}


static const LCD_DrvOpsDef CO5300_drv =
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

};

LCD_DRIVER_EXPORT(co5300, LCD_ID, &lcdc_int_cfg,
                  &CO5300_drv,
                  LCD_PIXEL_WIDTH,
                  LCD_PIXEL_HEIGHT,
                  2);
