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
#include "drv_gpio.h"
#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //
#define LOG_TAG                "nv3052c"

#include "log.h"

#define LCD_ID                 0x85

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

#define ROW_OFFSET  (0)

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = AUTO_SELECTED_DPI_INTFACE,
    .freq = 35 * 1000 * 1000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {
        .dpi = {
            .PCLK_polarity = 0,
            .DE_polarity   = 0,
            .VS_polarity   = 1,
            .HS_polarity   = 1,
            .PCLK_force_on = 0,

            .VS_width      = 5,    //VLW
            .HS_width      = 2,   //HLW

            .VBP = 15,   //VBP
            .VAH = 720,
            .VFP = 16,   //VFP

            .HBP = 44,   //HBP
            .HAW = 720,
            .HFP = 44,   //HFP

            .interrupt_line_num = 1,
        },
    },
};

static void PA_Write(uint16_t gpio, uint16_t val)
{
    HAL_GPIO_WritePin(hwp_gpio1, gpio, (GPIO_PinState)val);
}

#define LCD_USING_SOFT_SPI 1

#ifdef LCD_USING_SOFT_SPI

typedef struct
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[60];
} init_config;

#define LCD_CS_Pin           LCD_EXTRA_SPI_CS_PIN
#define LCD_CLK_Pin          LCD_EXTRA_SPI_CLK_PIN
#define LCD_MOSI_Pin         LCD_EXTRA_SPI_MOSI_PIN

#define LCD_SPI_CS(a)   HAL_GPIO_WritePin(GET_GPIO_INSTANCE(LCD_CS_Pin), GET_GPIOx_PIN(LCD_CS_Pin), (a>0)?GPIO_PIN_SET:GPIO_PIN_RESET)

#define SPI_DCLK(a)     HAL_GPIO_WritePin(GET_GPIO_INSTANCE(LCD_CLK_Pin), GET_GPIOx_PIN(LCD_CLK_Pin), (a>0)?GPIO_PIN_SET:GPIO_PIN_RESET)

#define SPI_SDA(a)      HAL_GPIO_WritePin(GET_GPIO_INSTANCE(LCD_MOSI_Pin), GET_GPIOx_PIN(LCD_MOSI_Pin), (a>0)?GPIO_PIN_SET:GPIO_PIN_RESET)

static void lcd_gpio_init(void)
{
    /* GPIO Ports Clock Enable */

    rt_pin_mode(LCD_CLK_Pin,  PIN_MODE_OUTPUT);
    rt_pin_mode(LCD_MOSI_Pin, PIN_MODE_OUTPUT);
    rt_pin_mode(LCD_CS_Pin,   PIN_MODE_OUTPUT);

    LCD_SPI_CS(1);
    SPI_DCLK(1);
    SPI_SDA(1);
}

static void lcd_spi_config(void)
{
    lcd_gpio_init();

    LCD_SPI_CS(1);
    SPI_DCLK(1);
}

static void spi_io_byte_write(uint8_t byte)
{
    uint8_t n;

    for (n = 0; n < 8; n++)
    {
        if (byte & 0x80)
        {
            SPI_SDA(1);
        }
        else
        {
            SPI_SDA(0);
        }
        byte <<= 1;

        SPI_DCLK(0);
        SPI_DCLK(1);
    }
}

static void spi_io_comm_write(uint8_t cmd)  //9bit transmit mode
{
    LCD_SPI_CS(1);
    LCD_SPI_CS(0);
    SPI_SDA(0);
    SPI_DCLK(0);
    SPI_DCLK(1);
    spi_io_byte_write(cmd);
    LCD_SPI_CS(1);
}

static void spi_io_data_write(uint8_t tem_data)  //9bit transmit mode
{
    LCD_SPI_CS(0);
    SPI_SDA(1);
    SPI_DCLK(0);
    SPI_DCLK(1);
    spi_io_byte_write(tem_data);
    LCD_SPI_CS(1);
}

static void send_config(uint32_t command, uint32_t size, const uint8_t *data)
{
    uint8_t num = 0;
    spi_io_comm_write(command);
    for (num = 0; num < size; num++)
    {
        spi_io_data_write(data[num]);
    }
}

#define buf_size  167
static const init_config lcd_init_cmds[] =
{
    {0xFF, 1, 0x30},
    {0xFF, 1, 0x52},
    {0xFF, 1, 0x01},
    {0xE3, 1, 0x00},
    {0x0A, 1, 0x01},
    {0x23, 1, 0xA0}, //RGB MODE
    {0x25, 1, 0x14},
    {0x29, 1, 0x02},
    {0x2A, 1, 0xCF},
    {0x38, 1, 0x9C},
    {0x39, 1, 0xA7},
    {0x3A, 1, 0x33}, //VCOM
    {0x91, 1, 0x77},
    {0x92, 1, 0x77},
    {0x99, 1, 0x52},
    {0x9B, 1, 0x5B}, //page2
    {0xA0, 1, 0x55},
    {0xA1, 1, 0x50},
    {0xA4, 1, 0x9C},
    {0xA7, 1, 0x02},
    {0xA8, 1, 0x01},
    {0xA9, 1, 0x01},
    {0xAA, 1, 0xFC},
    {0xAB, 1, 0x28},
    {0xAC, 1, 0x06},
    {0xAD, 1, 0x06},
    {0xAE, 1, 0x06},
    {0xAF, 1, 0x03},
    {0xB0, 1, 0x08},
    {0xB1, 1, 0x26},
    {0xB2, 1, 0x28},
    {0xB3, 1, 0x28},
    {0xB4, 1, 0x03},
    {0xB5, 1, 0x08},
    {0xB6, 1, 0x26},
    {0xB7, 1, 0x08},
    {0xB8, 1, 0x26},
    {0xFF, 1, 0x30},
    {0xFF, 1, 0x52},
    {0xFF, 1, 0x02},
    {0xB0, 1, 0x02},
    {0xB1, 1, 0x31},
    {0xB2, 1, 0x24},
    {0xB3, 1, 0x30},
    {0xB4, 1, 0x38},
    {0xB5, 1, 0x3E},
    {0xB6, 1, 0x26},
    {0xB7, 1, 0x3E},
    {0xB8, 1, 0x0a},
    {0xB9, 1, 0x00},
    {0xBA, 1, 0x11},
    {0xBB, 1, 0x11},
    {0xBC, 1, 0x13},
    {0xBD, 1, 0x14},
    {0xBE, 1, 0x18},
    {0xBF, 1, 0x11}, //page3
    {0xC0, 1, 0x16},
    {0xC1, 1, 0x00},
    {0xD0, 1, 0x05},
    {0xD1, 1, 0x30},
    {0xD2, 1, 0x25},
    {0xD3, 1, 0x35},
    {0xD4, 1, 0x34},
    {0xD5, 1, 0x3B},
    {0xD6, 1, 0x26},
    {0xD7, 1, 0x3D},
    {0xD8, 1, 0x0a},
    {0xD9, 1, 0x00},
    {0xDA, 1, 0x12},
    {0xDB, 1, 0x10},
    {0xDC, 1, 0x12},
    {0xDD, 1, 0x14},
    {0xDE, 1, 0x18},
    {0xDF, 1, 0x11},
    {0xE0, 1, 0x15},
    {0xE1, 1, 0x00},
    {0xFF, 1, 0x30},
    {0xFF, 1, 0x52},
    {0xFF, 1, 0x03},
    {0x08, 1, 0x09},
    {0x09, 1, 0x0A},
    {0x0A, 1, 0x0B},
    {0x0B, 1, 0x0C},
    {0x28, 1, 0x22},
    {0x2A, 1, 0xEC},
    {0x2B, 1, 0xEC},
    {0x30, 1, 0x00},
    {0x31, 1, 0x00},
    {0x32, 1, 0x00},
    {0x33, 1, 0x00},
    {0x34, 1, 0x61},
    {0x35, 1, 0xD4},
    {0x36, 1, 0x24},
    {0x37, 1, 0x03},
    {0x40, 1, 0x0D},
    {0x41, 1, 0x0E}, //page4
    {0x42, 1, 0x0F},
    {0x43, 1, 0x10},
    {0x44, 1, 0x22},
    {0x45, 1, 0xE1},
    {0x46, 1, 0xE2},
    {0x47, 1, 0x22},
    {0x48, 1, 0xE3},
    {0x49, 1, 0xE4},
    {0x50, 1, 0x11},
    {0x51, 1, 0x12},
    {0x52, 1, 0x13},
    {0x53, 1, 0x14},
    {0x54, 1, 0x22},
    {0x55, 1, 0xE5},
    {0x56, 1, 0xE6},
    {0x57, 1, 0x22},
    {0x58, 1, 0xE7},
    {0x59, 1, 0xE8},
    {0x80, 1, 0x05},
    {0x81, 1, 0x1E},
    {0x82, 1, 0x02},
    {0x83, 1, 0x04},
    {0x84, 1, 0x1E},
    {0x85, 1, 0x1E},
    {0x86, 1, 0x1f},
    {0x87, 1, 0x1f},
    {0x88, 1, 0x0E},
    {0x89, 1, 0x10},
    {0x8A, 1, 0x0A},
    {0x8B, 1, 0x0C},
    {0x96, 1, 0x05},
    {0x97, 1, 0x1E},
    {0x98, 1, 0x01},
    {0x99, 1, 0x03},
    {0x9A, 1, 0x1E},
    {0x9B, 1, 0x1E},
    {0x9C, 1, 0x1f},
    {0x9D, 1, 0x1f},
    {0x9E, 1, 0x0D},
    {0x9F, 1, 0x0F}, //page5
    {0xA0, 1, 0x09},
    {0xA1, 1, 0x0B},
    {0xB0, 1, 0x05},
    {0xB1, 1, 0x1F},
    {0xB2, 1, 0x03},
    {0xB3, 1, 0x01},
    {0xB4, 1, 0x1E},
    {0xB5, 1, 0x1E},
    {0xB6, 1, 0x1f},
    {0xB7, 1, 0x1E},
    {0xB8, 1, 0x0B},
    {0xB9, 1, 0x09},
    {0xBA, 1, 0x0F},
    {0xBB, 1, 0x0D},
    {0xC6, 1, 0x05},
    {0xC7, 1, 0x1F},
    {0xC8, 1, 0x04},
    {0xC9, 1, 0x02},
    {0xCA, 1, 0x1E},
    {0xCB, 1, 0x1E},
    {0xCC, 1, 0x1f},
    {0xCD, 1, 0x1E},
    {0xCE, 1, 0x0C},
    {0xCF, 1, 0x0A},
    {0xD0, 1, 0x10},
    {0xD1, 1, 0x0E},
    {0xFF, 1, 0x30},
    {0xFF, 1, 0x52},
    {0xFF, 1, 0x00},
    {0x36, 1, 0x02}, //反扫09
    //(0x3A,1,0x66);//18BIT
    {0x11, 1, 0x00},
    //Delay_ms(200};
    //{0x29,1,0x00},
    //Delay_ms(100},
};
#endif

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    /* Initialize LCD low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(1);
    rt_thread_delay(10);
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_delay(5);
    BSP_LCD_Reset(1);
    rt_thread_delay(80);
#ifdef LCD_USING_SOFT_SPI

    rt_kprintf("LCD_Init soft spi\n");

    lcd_spi_config();

    uint8_t i = 0;
    init_config *init = (init_config *)&lcd_init_cmds[0];

    for (i = 0; i < buf_size; i++) //init LCD reg
    {
        send_config(init->cmd, init->len, init->data);
        init++;
    }
    rt_thread_delay(60);
    spi_io_comm_write(0x29);  //Display on
    rt_thread_delay(60);
#endif
    rt_kprintf("LCD_Init end\n");
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
#ifdef LCD_USING_SOFT_SPI
    return LCD_ID;
#else
    uint8_t data[16];
//    LCD_spi_read(REG_LCD_ID, 1, data, 3);
    rt_kprintf("LCD_ReadID 0x%x%x%x%x\n", data[0], data[1], data[2], data[3]);

    return LCD_ID;
#endif
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
    //LCD_WriteReg(hlcdc, LCD_DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
    //LCD_WriteReg(hlcdc, LCD_DISPLAY_OFF, (uint8_t *)NULL, 0);
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
    /*Invalid*/
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
//    rt_kprintf(" SetRegion hlcdc:0x%x,Xpos0:%d, Ypos0:%d, Xpos1:%d, Ypos1:%d\n",hlcdc, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SetROIArea(hlcdc, 0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1); //Not support partical columns
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;
//    rt_kprintf("WriteMultiplePixels RGBCode:0x%x,Xpos0:%d, Ypos0:%d, Xpos1:%d, Ypos1:%d\n",RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
//    DEBUG_PRINTF("NOT support read pixel\n");
    return 0; //Not support read pixel
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];

    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

/**
  * @brief  Enable the Display idle mode.
  * @param  None
  * @retval None
  */
static void LCD_IdleModeOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Idle mode On */
//    LCD_WriteReg(hlcdc, REG_IDLE_MODE_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display idle mode.
  * @param  None
  * @retval None
  */
static void LCD_IdleModeOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Idle mode Off */
//    LOG_I(hlcdc, REG_IDLE_MODE_OFF, (uint8_t *)NULL, 0);
}
static void  TimeoutDbg(LCDC_HandleTypeDef *hlcdc)
{
//    uint32_t data;

//    data = LCD_ReadData(hlcdc, REG_POWER_MODE, 1);
//    LOG_I("TimeoutDbg PowerMode 0x%x", data);//0x9c
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
//    uint32_t data;

//    data = LCD_ReadData(hlcdc, REG_POWER_MODE, 1);
//    LOG_I("ESDCehck PowerMode 0x%x", data);//0x9c

    return 0;
}

#define LCD_BRIGHTNESS_MAX 0xFF

static void     LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
//    uint8_t bright = (uint8_t)((int)LCD_BRIGHTNESS_MAX * br / 100);
//    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
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

LCD_DRIVER_EXPORT2(nv3052c, LCD_ID, &lcdc_int_cfg,
                   &LCD_drv, 2);

