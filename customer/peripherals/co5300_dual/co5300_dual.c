/**
  ******************************************************************************
  * @file   co5300_dual.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for CO5300_dual LCD.
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

/**
  * @brief CO5300 chip IDs
  */

#define LCD_ID                   0x331100

/**
  * @brief  CO5300 Size
  */
#define  REG_LCD_PIXEL_WIDTH    390
#define  REG_LCD_PIXEL_HEIGHT   450

/**
 *  @brief LCD_OrientationTypeDef
 *  Possible values of Display Orientation
 */
#define REG_ORIENTATION_PORTRAIT         (0x00) /* Portrait orientation choice of LCD screen  */
#define REG_ORIENTATION_LANDSCAPE        (0x01) /* Landscape orientation choice of LCD screen */
#define REG_ORIENTATION_LANDSCAPE_ROT180 (0x02) /* Landscape rotated 180 orientation choice of LCD screen */

/**
  * @brief  REG Registers
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

#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)

#define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)



#define BSP_LCD_PIN 01
#define LEFT_LCD_CS_PIN  03
#define RIGHT_LCD_CS_PIN 39   //定义两个屏幕的片选引脚：
#define BSP_LCD_PIN_1 25
#define RIGHT_LCD_RESET_PIN 29




#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA
    .freq = 48000000,        //CO5300 RGB565 only support 50000000,  RGB888 support 60000000
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

static void HAL_GPIO_Set(uint16_t pin, int value);
static uint16_t left_lcd_pin_pin = LEFT_LCD_CS_PIN;
static uint16_t right_lcd_pin_pin = RIGHT_LCD_CS_PIN;
static uint16_t Region_Xpos0, Region_Ypos0, Region_Xpos1, Region_Ypos1;
static uint16_t current_lcd_cs_pin;
static LCDC_InitTypeDef lcdc_int_cfg;

static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc);
static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1);
static void LCD_WriteReg_both(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);

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
    LCD_SetRegion(hlcdc, 0, 0, REG_LCD_PIXEL_WIDTH, REG_LCD_PIXEL_HEIGHT);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

}



static void BSP_RIGHT_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(RIGHT_LCD_RESET_PIN, high1_low0, 1);
}
static void BSP_LCD_ResetSide(uint8_t pin)
{
    if (pin == left_lcd_pin_pin)
    {
        BSP_LCD_Reset(1);
        rt_thread_delay(10);
        BSP_LCD_Reset(0);//Reset LCD
        rt_thread_delay(10);
        BSP_LCD_Reset(1);
        rt_thread_delay(50);
    }
    else if (pin == right_lcd_pin_pin)
    {
        BSP_RIGHT_LCD_Reset(1);
        rt_thread_delay(10);
        BSP_RIGHT_LCD_Reset(0);
        rt_thread_delay(10);
        BSP_RIGHT_LCD_Reset(1);
        rt_thread_delay(50);

    }
}
static void LCD_Drv_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    BSP_LCD_ResetSide(current_lcd_cs_pin);
    // if (LCD_ReadID(hlcdc) !=  LCD_ID)
    // {
    //     return;
    // }
    parameter[0] = 0x20;
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
    parameter[2] = ((REG_LCD_PIXEL_WIDTH + COL_OFFSET - 1) >> 8) & 0xFF;
    parameter[3] = (REG_LCD_PIXEL_WIDTH + COL_OFFSET - 1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);
    parameter[0] = (ROW_OFFSET >> 8) & 0xFF;
    parameter[1] = ROW_OFFSET & 0xFF;
    parameter[2] = ((REG_LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) >> 8) & 0xFF;
    parameter[3] = (REG_LCD_PIXEL_HEIGHT + ROW_OFFSET - 1) & 0xFF;
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
#endif

    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);
    rt_pin_mode(BSP_LCD_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BSP_LCD_PIN, PIN_HIGH);

    HAL_PIN_Set(PAD_PA25, GPIO_A25, PIN_NOPULL, 1);
    rt_pin_mode(BSP_LCD_PIN_1, PIN_MODE_OUTPUT);
    rt_pin_write(BSP_LCD_PIN_1, PIN_HIGH);

    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);
    rt_pin_mode(LEFT_LCD_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LEFT_LCD_CS_PIN, PIN_HIGH);

    HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_NOPULL, 1);
    rt_pin_mode(RIGHT_LCD_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RIGHT_LCD_CS_PIN, PIN_HIGH);

    HAL_PIN_Set(PAD_PA29, GPIO_A29, PIN_NOPULL, 1);
    rt_pin_mode(RIGHT_LCD_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RIGHT_LCD_RESET_PIN, PIN_HIGH);
    /* Initialize CO5300 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    current_lcd_cs_pin = left_lcd_pin_pin;
    LCD_Drv_Init(hlcdc);

    current_lcd_cs_pin = right_lcd_pin_pin;
    LCD_Drv_Init(hlcdc);

}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t left_data, right_data;
    current_lcd_cs_pin = left_lcd_pin_pin;
    left_data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    rt_kprintf("\nLeft CO5300_ReadID 0x%x \n", left_data);

    current_lcd_cs_pin = right_lcd_pin_pin;
    right_data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    rt_kprintf("\nRight CO5300_ReadID 0x%x \n", right_data);
    if (left_data == LCD_ID)
    {
        DEBUG_PRINTF("LCD module use Left CO5300 IC \n");
        return left_data;
    }
    else if (right_data == LCD_ID)
    {
        DEBUG_PRINTF("LCD module use Right CO5300 IC \n");
        return right_data;
    }
    else
    {
        return RT_FALSE;
    }

}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
    LCD_WriteReg_both(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
    LCD_WriteReg_both(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    Region_Xpos0 = Xpos0;
    Region_Ypos0 = Ypos0;
    Region_Xpos1 = Xpos1;
    Region_Ypos1 = Ypos1;
}

static bool LCDC_SetROIArea(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[4];

    if (current_lcd_cs_pin == left_lcd_pin_pin)
    {

        if (Region_Xpos0 <= REG_LCD_PIXEL_WIDTH) //
        {
            //Set LCD recieve area
            parameter[0] = (Region_Xpos0) >> 8;
            parameter[1] = (Region_Xpos0) & 0xFF;
            parameter[2] = (REG_LCD_PIXEL_WIDTH - 1) >> 8;
            parameter[3] = (REG_LCD_PIXEL_WIDTH - 1) & 0xFF;
            LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

            parameter[0] = (Region_Ypos0) >> 8;
            parameter[1] = (Region_Ypos0) & 0xFF;
            parameter[2] = (Region_Ypos1) >> 8;
            parameter[3] = (Region_Ypos1) & 0xFF;
            LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

            HAL_LCDC_SetROIArea(hlcdc, Region_Xpos0, Region_Ypos0, REG_LCD_PIXEL_WIDTH - 1, Region_Ypos1);

            return RT_TRUE;
        }
        else
        {
            return RT_FALSE;
        }
    }
    else if (current_lcd_cs_pin == right_lcd_pin_pin)
    {

        if (Region_Xpos1 >= REG_LCD_PIXEL_WIDTH && Region_Xpos1 < REG_LCD_PIXEL_WIDTH * 2) //
        {
            uint16_t right_X_start = 0;
            uint16_t right_X_end = Region_Xpos1 - REG_LCD_PIXEL_WIDTH;

            parameter[0] = (right_X_start >> 8) & 0xFF;
            parameter[1] = right_X_start & 0xFF;
            parameter[2] = (right_X_end >> 8) & 0xFF;
            parameter[3] = right_X_end & 0xFF;
            LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

            parameter[0] = (Region_Ypos0 >> 8) & 0xFF;
            parameter[1] = Region_Ypos0 & 0xFF;
            parameter[2] = (Region_Ypos1 >> 8) & 0xFF;
            parameter[3] = Region_Ypos1 & 0xFF;
            LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

            HAL_LCDC_SetROIArea(hlcdc, REG_LCD_PIXEL_WIDTH, Region_Ypos0, Region_Xpos1, Region_Ypos1);

            return RT_TRUE;
        }
        else
        {
            return RT_FALSE;
        }
    }
    else
    {
        return RT_FALSE;
    }

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

static void HAL_GPIO_Set(uint16_t pin, int value)
{

    rt_pin_write(pin, (value != 0) ? PIN_HIGH : PIN_LOW);
}

static void (* Ori_XferCpltCallback)(struct __LCDC_HandleTypeDef *lcdc);

static void Right_LCD_SendLayerDataCpltCbk(LCDC_HandleTypeDef *hlcdc)
{
    HAL_GPIO_Set(current_lcd_cs_pin, 1);
    Ori_XferCpltCallback(hlcdc);
}


static void Left_LCD_SendLayerDataCpltCbk(LCDC_HandleTypeDef *hlcdc)
{
    HAL_GPIO_Set(current_lcd_cs_pin, 1);

    current_lcd_cs_pin = right_lcd_pin_pin;
    if (LCDC_SetROIArea(hlcdc))
    {
        //Send right LCD

        hlcdc->XferCpltCallback = Right_LCD_SendLayerDataCpltCbk;

        HAL_GPIO_Set(current_lcd_cs_pin, 0);
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    }
    else
    {
        Ori_XferCpltCallback(hlcdc);
    }
}


static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

    Ori_XferCpltCallback = hlcdc->XferCpltCallback;

    current_lcd_cs_pin = left_lcd_pin_pin;
    if (LCDC_SetROIArea(hlcdc))
    {
        hlcdc->XferCpltCallback = Left_LCD_SendLayerDataCpltCbk;
        HAL_GPIO_Set(current_lcd_cs_pin, 0);
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    }
    else
    {
        current_lcd_cs_pin = right_lcd_pin_pin;
        if (LCDC_SetROIArea(hlcdc))
        {
            hlcdc->XferCpltCallback = Right_LCD_SendLayerDataCpltCbk;
            HAL_GPIO_Set(current_lcd_cs_pin, 0);
            HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
        }
        else
        {
            RT_ASSERT(0);
        }
    }

}


/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{

    HAL_GPIO_Set(current_lcd_cs_pin, 0);

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

    HAL_GPIO_Set(current_lcd_cs_pin, 1);
}

static void LCD_WriteReg_both(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
    uint16_t lcd_pins[] = {left_lcd_pin_pin, right_lcd_pin_pin};

    for (int i = 0; i < 2; i++)
    {
        uint16_t current_pin = lcd_pins[i];

        HAL_GPIO_Set(current_pin, 0);

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

        HAL_GPIO_Set(current_pin, 1);
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
    HAL_GPIO_Set(current_lcd_cs_pin, 0);
    LCD_ReadMode(hlcdc, true);

    HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);

    LCD_ReadMode(hlcdc, false);
    HAL_GPIO_Set(current_lcd_cs_pin, 1);
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
    LCD_WriteReg_both(hlcdc, REG_COLOR_MODE, parameter, 1);
    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define REG_BRIGHTNESS_MAX 0xFF

static void  LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{

    uint8_t bright = (uint8_t)((int)REG_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg_both(hlcdc, REG_WBRIGHT, &bright, 1);
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
    LCD_WriteReg_both(hlcdc, 0xFE, parameter, 1);

    /* Idle mode On */
    LCD_WriteReg_both(hlcdc, REG_IDLE_MODE_ON, NULL, 0);
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
    LCD_WriteReg_both(hlcdc, 0xFE, parameter, 1);

    /* Idle mode Off */
    LCD_WriteReg_both(hlcdc, REG_IDLE_MODE_OFF, NULL, 0);

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
                  REG_LCD_PIXEL_WIDTH,
                  REG_LCD_PIXEL_HEIGHT,
                  2);

