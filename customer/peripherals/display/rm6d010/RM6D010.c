/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"




#define  DBG_LEVEL            DBG_ERROR  //DBG_LOG //
#define LOG_TAG              "drv.RM6D010"
#include <drv_log.h>
#define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)

#define RM6D010_TP_INT 80

#define MAX_IO_BUFFER_LEN 32










#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x10)



/**
  * @brief RM6D010 chip IDs
  */
#define THE_LCD_ID                  0x1190a7

/**
  * @brief  RM6D010 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (412)
#define  THE_LCD_PIXEL_HEIGHT   (412)






/**
  * @brief  RM6D010 Registers
  */
#define REG_SW_RESET           0x01
#define REG_LCD_ID             0x04
#define RM6D010_READ_MODE          0x09
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



































/*RM6D010 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)








#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA
    .freq = 40000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
#ifdef LCD_RM6D010_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_RM6D010_VSYNC_ENABLE */
            .vsyn_polarity = 0,
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
            HAL_LCDC_SetFreq(hlcdc, 1000000); //read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq); //Restore normal frequency
        }
    }
}
#define MAX_CMD_LEN 6
#if (0)
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    va_end(args);
}
#endif

static const uint8_t LCD_Qicai_parameter[] =
{
    0xFE, 0x01,
    0x05, 0x12,
    0x06, 0x70,
    0x0D, 0x00,
    0x0E, 0x81,
    0x0F, 0x81,
    0x10, 0x11,
    0x11, 0x81,
    0x12, 0x81,
    0x13, 0x80,
    0x14, 0x80,
    0x15, 0x81,
    0x16, 0x81,
    0x18, 0x66,
    0x19, 0x88,
    0x1A, 0x10,
    0x78, 0x95,
    0x7A, 0x95,
    0x99, 0x19,
    0x9A, 0x19,
    0x70, 0x55,
    0x57, 0x00,
    0x8B, 0x40,
    0xA7, 0x10,
    0x25, 0x03,
    0x26, 0x30,
    0x27, 0x08,
    0x28, 0x08,
    0x2A, 0x03,
    0x2B, 0x30,
    0x2D, 0x08,
    0x2F, 0x08,
    0x30, 0x43,
    0xFE, 0x01,
    0x3a, 0x00,
    0x3b, 0x00,
    0x3d, 0x12,
    0x3f, 0x43,
    0x40, 0x12,
    0x41, 0x0a,
    0x6D, 0x90,
    0x72, 0x1A,
    0x73, 0x13,
    0xFE, 0x01,
    0x1B, 0x03,
    0xFE, 0x01,
    0x42, 0x14,
    0x43, 0x41,
    0x44, 0x52,
    0x45, 0x25,
    0x46, 0x36,
    0x47, 0x63,
    0x4C, 0x41,
    0x4D, 0x14,
    0x4E, 0x25,
    0x4F, 0x52,
    0x50, 0x63,
    0x51, 0x36,
    0x56, 0x63,
    0x58, 0x36,
    0x59, 0x25,
    0x5A, 0x52,
    0x5B, 0x41,
    0x5C, 0x14,
    0x61, 0x36,
    0x62, 0x63,
    0x63, 0x52,
    0x64, 0x25,
    0x65, 0x14,
    0x66, 0x41,
    0xFE, 0x02,
    0xEF, 0x01,
    0x9B, 0x20,
    0x9C, 0x68,
    0x9D, 0x02,
    0x32, 0x01,
    0x33, 0xDC,
    0x67, 0x01,
    0x68, 0xE7,
    0x99, 0x01,
    0x9A, 0x9F,
    0x30, 0x01,
    0x31, 0xE5,
    0x65, 0x01,
    0x66, 0xF0,
    0x97, 0x01,
    0x98, 0xA9,
    0x2D, 0x01,
    0x2F, 0xE6,
    0x63, 0x01,
    0x64, 0xF4,
    0x95, 0x01,
    0x96, 0xAD,
    0x2A, 0x01,
    0x2B, 0xED,
    0x61, 0x01,
    0x62, 0xF9,
    0x93, 0x01,
    0x94, 0xB4,
    0x28, 0x01,
    0x29, 0xF5,
    0x5F, 0x02,
    0x60, 0x00,
    0x91, 0x01,
    0x92, 0xBC,
    0x26, 0x01,
    0x27, 0xFE,
    0x5D, 0x02,
    0x5E, 0x09,
    0x8F, 0x01,
    0x90, 0xC7,
    0x24, 0x02,
    0x25, 0x07,
    0x5B, 0x02,
    0x5C, 0x10,
    0x8D, 0x01,
    0x8E, 0xCF,
    0x22, 0x02,
    0x23, 0x0E,
    0x59, 0x02,
    0x5A, 0x19,
    0x8B, 0x01,
    0x8C, 0xDA,
    0x20, 0x02,
    0x21, 0x1E,
    0x56, 0x02,
    0x58, 0x27,
    0x89, 0x01,
    0x8A, 0xE9,
    0x1E, 0x02,
    0x1F, 0x2F,
    0x54, 0x02,
    0x55, 0x37,
    0x87, 0x01,
    0x88, 0xFE,
    0x1C, 0x02,
    0x1D, 0x41,
    0x52, 0x02,
    0x53, 0x47,
    0x85, 0x02,
    0x86, 0x11,
    0x1A, 0x02,
    0x1B, 0x51,
    0x50, 0x02,
    0x51, 0x57,
    0x83, 0x02,
    0x84, 0x24,
    0x18, 0x02,
    0x19, 0x62,
    0x4E, 0x02,
    0x4F, 0x67,
    0x81, 0x02,
    0x82, 0x37,
    0x16, 0x02,
    0x17, 0x74,
    0x4C, 0x02,
    0x4D, 0x78,
    0x7F, 0x02,
    0x80, 0x4B,
    0x14, 0x02,
    0x15, 0x86,
    0x4A, 0x02,
    0x4B, 0x89,
    0x7D, 0x02,
    0x7E, 0x5F,
    0x12, 0x02,
    0x13, 0x99,
    0x48, 0x02,
    0x49, 0x9C,
    0x7B, 0x02,
    0x7C, 0x75,
    0x10, 0x02,
    0x11, 0xAF,
    0x46, 0x02,
    0x47, 0xB0,
    0x79, 0x02,
    0x7A, 0x8C,
    0x0E, 0x02,
    0x0F, 0xC6,
    0x44, 0x02,
    0x45, 0xC6,
    0x77, 0x02,
    0x78, 0xA6,
    0x0C, 0x02,
    0x0D, 0xD4,
    0x42, 0x02,
    0x43, 0xD3,
    0x75, 0x02,
    0x76, 0xB5,
    0x0A, 0x02,
    0x0B, 0xE3,
    0x40, 0x02,
    0x41, 0xE1,
    0x73, 0x02,
    0x74, 0xC5,
    0x08, 0x02,
    0x09, 0xF6,
    0x3D, 0x02,
    0x3F, 0xF1,
    0x71, 0x02,
    0x72, 0xD6,
    0x06, 0x03,
    0x07, 0x12,
    0x3A, 0x03,
    0x3B, 0x06,
    0x6F, 0x02,
    0x70, 0xE9,
    0x04, 0x03,
    0x05, 0x1C,
    0x38, 0x03,
    0x39, 0x15,
    0x6D, 0x02,
    0x6E, 0xF9,
    0x02, 0x03,
    0x03, 0x40,
    0x36, 0x03,
    0x37, 0x2F,
    0x6B, 0x02,
    0x6C, 0xFC,
    0x00, 0x03,
    0x01, 0xFF,
    0x34, 0x03,
    0x35, 0xFF,
    0x69, 0x03,
    0x6A, 0xFF,
    0xF0, 0x01,
    0xEF, 0x00,
    0xF0, 0x00,
    0xFE, 0x01,
    0xA2, 0x00,
    0xFE, 0x07,
    0x16, 0x02,
    0xFE, 0x04,
    0x63, 0x00,
    0xFE, 0x04,
    0x5D, 0x11,
    0xFE, 0x04,
    0x5e, 0x8f,
    0x5f, 0x10,
    0x60, 0x29,
    0x61, 0xf3,
    0x62, 0xff,
    0x00, 0xdd,
    0x01, 0x00,
    0x02, 0x02,
    0x03, 0x00,
    0x04, 0x10,
    0x05, 0x01,
    0x06, 0x10,
    0x07, 0x16,
    0x08, 0x00,
    0x09, 0xdd,
    0x0a, 0x00,
    0x0b, 0x02,
    0x0c, 0x00,
    0x0d, 0x10,
    0x0e, 0x02,
    0x0f, 0x10,
    0x10, 0x16,
    0x11, 0x00,
    0x12, 0xdc,
    0x13, 0x00,
    0x14, 0x02,
    0x15, 0x00,
    0x16, 0x00,
    0x17, 0x01,
    0x18, 0x72,
    0x19, 0x44,
    0x1a, 0x00,
    0x1b, 0xdc,
    0x1c, 0x00,
    0x1d, 0x02,
    0x1e, 0x00,
    0x1f, 0x00,
    0x20, 0x02,
    0x21, 0x72,
    0x22, 0x44,
    0x23, 0x00,
    0x4c, 0x89,
    0x4d, 0x00,
    0x4e, 0x00,
    0x4f, 0x00,
    0x50, 0x01,
    0x51, 0x01,
    0x52, 0x01,
    0x53, 0x8a,
    0x54, 0x00,
    0x55, 0x03,
    0x56, 0x01,
    0x58, 0x01,
    0x59, 0x00,
    0x65, 0x02,
    0x66, 0x05,
    0x67, 0x00,
    0xFE, 0x01,
    0x1D, 0x03,
    0x1E, 0x03,
    0x1F, 0x03,
    0x20, 0x03,
    0xFE, 0x01,
    0x36, 0x00,
    0x87, 0xB3,
    0x6B, 0xF8,
    0x37, 0x10,
    0xFE, 0x00,
    0xC4, 0x80,
    0x35, 0x00,
    0x3A, 0x55,
    0x51, 0x20
};

//Define OLED solution:368x448
static const uint8_t LCD_Qicai2[] =
{
    0x00, 0x00, 0x01, 0x6F
};//0x2A,X:368
static const uint8_t LCD_Qicai3[] =
{
    0x00, 0x00, 0x01, 0xBF
};//0x2B,Y:448

static void RM6D010_Init_SPI_Mode(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];
    int i, j;
    uint8_t *p_tmp = (uint8_t *)LCD_Qicai_parameter;
    BSP_LCD_PowerUp();

    /* Initialize RM6D010 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);

    /* Wait for 100ms */
    LCD_DRIVER_DELAY_MS(100);

    for (i = 0; i < sizeof(LCD_Qicai_parameter) / 2; i++)
    {
        //rt_kprintf("write %d,cmd=0x%x,len=%d\n",i,(int)lcd_init_cmds[i][0], (int)lcd_init_cmds[i][1]);
        //HAL_DBG_print_data((char*)&(lcd_init_cmds[i][2]),0,(int)lcd_init_cmds[i][1]);
        LCD_WriteReg(hlcdc, *p_tmp, (uint8_t *)(p_tmp + 1), 1);
        HAL_Delay_us(100);
        p_tmp += 2;
        //__asm("B .");
    }

    p_tmp = (uint8_t *)LCD_Qicai2;
    LCD_WriteReg(hlcdc, 0x2A, p_tmp, 4);
    p_tmp = (uint8_t *)LCD_Qicai3;
    LCD_WriteReg(hlcdc, 0x2B, p_tmp, 4);

    LCD_WriteReg(hlcdc, 0x11, NULL, 0);

    LCD_DRIVER_DELAY_MS(120);

    /*Clear gram*/
    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    HAL_LCDC_SetROIArea(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH, THE_LCD_PIXEL_HEIGHT);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

    LCD_WriteReg(hlcdc, 0x29, NULL, 0);
    LCD_DRIVER_DELAY_MS(20);

}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("RM6D010_Init \n");
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_qadspi, sizeof(lcdc_int_cfg));
    {
        RM6D010_Init_SPI_Mode(hlcdc);
    }
    rt_kprintf("RM6D010_Init done!\n");
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
    rt_kprintf("\nRM6D010_ReadID 0x%x \n", data);
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
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("RM6D010_DisplayOn \n");
    /* Display On */
    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

//    RM6D010tp_reset();
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("RM6D010_DisplayOff \n");
    /* Display Off */
    LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, REG_SLEEP_IN, (uint8_t *)NULL, 0);

}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t   parameter[4];
    LCD_ALIGN2(Xpos0);
    LCD_ALIGN2(Ypos0);
    LCD_ALIGN1(Xpos1);
    LCD_ALIGN1(Ypos1);
    //rt_kprintf("LCD_SetRegion Xpos0 %d, Ypos0 %d, Xpos1 %d, Ypos1 %d\r\n", Xpos0, Ypos0, Xpos1, Ypos1);

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
#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_WriteU32Reg(hlcdc, ((0x02 << 24) | (LCD_Reg << 8)), Parameters, NbParameters);
    }
    else
        HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);

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
        HAL_LCDC_ReadU32Reg(hlcdc, ((0x0B << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);
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
    DEBUG_PRINTF("RM6D010_ReadPixel[%d,%d]\n", Xpos, Ypos);

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


    return;

    /*

    Control interface color format
    ‘011’ = 12bit/pixel ‘101’ = 16bit/pixel ‘110’ = 18bit/pixel ‘111’ = 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x55;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;
    case RTGRAPHIC_PIXEL_FORMAT_RGB666:
        parameter[0] = 0x66;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB666;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0x77;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    uint32_t data = LCD_ReadData(hlcdc, 0xc, 1);
    DEBUG_PRINTF("\nRM6D010_color_format 0x%x \n", data);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define RM6D010_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = br;
//  bright=255;//Set max for test.
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
    rt_kprintf("RM6D010_SetBrightness val=%d \n", br);


}



static const LCD_DrvOpsDef RM6D010_drv =
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


LCD_DRIVER_EXPORT2(RM6D010, THE_LCD_ID, &lcdc_int_cfg,
                   &RM6D010_drv, 2);



/****************************************************************************************************/
