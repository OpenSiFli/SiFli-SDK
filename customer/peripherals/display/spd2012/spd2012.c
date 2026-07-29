/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "spd2012.h"
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#define  DBG_LEVEL            DBG_INFO //DBG_LOG  //
#define LOG_TAG              "drv.spd2012"
#include <drv_log.h>
#define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)

#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)

/*spd2012 start colume & row must can be divided by 4, and roi width&height too.*/
#define SPD2012_CHECK_START_XY(xy)  RT_ASSERT(0 == ((xy) & 0x3))
#define SPD2012_CHECK_END_XY(xy)    RT_ASSERT((xy) == ((xy) | 0x3))

#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
#ifdef LCD_SPD2012_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_RM69090_VSYNC_ENABLE */
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 120,
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

#define lcd_set_page(page) {0xff,3,0x20,0x10,page}
#define MAX_CMD_LEN 6

#if 1   //LK 20210524

static const uint8_t lcd_init_cmds[][MAX_CMD_LEN] =
{

//OTP issue
    lcd_set_page(0x43), //page43
    {0x04, 1, 0x70},

    lcd_set_page(0x45), //page45
    {0x03, 1, 0x64},

    lcd_set_page(0x10), //page10
    {0x0C, 1, 0x11},
    {0x10, 1, 0x02},
    {0x11, 1, 0x11},
    {0x15, 1, 0x42},
    {0x16, 1, 0x11},
    {0x1A, 1, 0x02},
    {0x1B, 1, 0x11},
    {0x61, 1, 0x80},
    {0x62, 1, 0x80},
    {0x54, 1, 0x44},
    {0x58, 1, 0x88},
    {0x5C, 1, 0xcc},
    {0x20, 1, 0x80},
    {0x21, 1, 0x81},
    {0x22, 1, 0x31},
    {0x23, 1, 0x20},
    {0x24, 1, 0x11},
    {0x25, 1, 0x11},
    {0x26, 1, 0x12},
    {0x27, 1, 0x12},
    {0x30, 1, 0x80},
    {0x31, 1, 0x81},
    {0x32, 1, 0x31},
    {0x33, 1, 0x20},
    {0x34, 1, 0x11},
    {0x35, 1, 0x11},
    {0x36, 1, 0x12},
    {0x37, 1, 0x12},
    {0x41, 1, 0x11},
    {0x42, 1, 0x22},
    {0x43, 1, 0x33},
    {0x49, 1, 0x11},
    {0x4A, 1, 0x22},
    {0x4B, 1, 0x33},

    lcd_set_page(0x15), //page15
    {0x00, 1, 0x00},
    {0x01, 1, 0x00},
    {0x02, 1, 0x00},
    {0x03, 1, 0x00},
    {0x04, 1, 0x10},
    {0x05, 1, 0x0C},
    {0x06, 1, 0x23},
    {0x07, 1, 0x22},
    {0x08, 1, 0x21},
    {0x09, 1, 0x20},
    {0x0A, 1, 0x33},
    {0x0B, 1, 0x32},
    {0x0C, 1, 0x34},
    {0x0D, 1, 0x35},
    {0x0E, 1, 0x01},
    {0x0F, 1, 0x01},
    {0x20, 1, 0x00},
    {0x21, 1, 0x00},
    {0x22, 1, 0x00},
    {0x23, 1, 0x00},
    {0x24, 1, 0x0C},
    {0x25, 1, 0x10},
    {0x26, 1, 0x20},
    {0x27, 1, 0x21},
    {0x28, 1, 0x22},
    {0x29, 1, 0x23},
    {0x2A, 1, 0x33},
    {0x2B, 1, 0x32},
    {0x2C, 1, 0x34},
    {0x2D, 1, 0x35},
    {0x2E, 1, 0x01},
    {0x2F, 1, 0x01},

    lcd_set_page(0x16), //page16
    {0x00, 1, 0x00},
    {0x01, 1, 0x00},
    {0x02, 1, 0x00},
    {0x03, 1, 0x00},
    {0x04, 1, 0x08},
    {0x05, 1, 0x04},
    {0x06, 1, 0x19},
    {0x07, 1, 0x18},
    {0x08, 1, 0x17},
    {0x09, 1, 0x16},
    {0x0A, 1, 0x33},
    {0x0B, 1, 0x32},
    {0x0C, 1, 0x34},
    {0x0D, 1, 0x35},
    {0x0E, 1, 0x01},
    {0x0F, 1, 0x01},
    {0x20, 1, 0x00},
    {0x21, 1, 0x00},
    {0x22, 1, 0x00},
    {0x23, 1, 0x00},
    {0x24, 1, 0x04},
    {0x25, 1, 0x08},
    {0x26, 1, 0x16},
    {0x27, 1, 0x17},
    {0x28, 1, 0x18},
    {0x29, 1, 0x19},
    {0x2A, 1, 0x33},
    {0x2B, 1, 0x32},
    {0x2C, 1, 0x34},
    {0x2D, 1, 0x35},
    {0x2E, 1, 0x01},
    {0x2F, 1, 0x01},

    lcd_set_page(0x12), //page12
    {0x00, 1, 0x99},
    //{0x1E, 1, 0x00}
    {0x1F, 1, 0xDC},//V0.4: GVDDN
    {0x21, 1, 0x80},//Vcom

    {0x10, 1, 0x0F},
    {0x15, 1, 0x0F},
    {0x29, 1, 0xA7},//for Vcom<-1.7V
    {0x2A, 1, 0x28},//VGHO 28:13V
    {0x2B, 1, 0x1E},//VGLO 1E:-11V
    {0x2C, 1, 0x26},
    {0x2D, 1, 0x28},//VGHO 28:13V
    {0x2E, 1, 0x1E},//VGLO 1E:-11V

    lcd_set_page(0xA0), //pageA0
    {0x08, 1, 0xDC},//GVDDP

    lcd_set_page(0x40), //page40
    {0x86, 1, 0x00},
    //V0.5
    {0x83, 1, 0xC4},

    lcd_set_page(0x42), //page42
    //{0x05, 1, 0x2c
    {0x05, 1, 0x3D},
    {0x06, 1, 0x03},

    lcd_set_page(0x11), //page11
    {0x50, 1, 0x01},
    {0x60, 1, 0x01},
    {0x65, 1, 0x03},
    {0x66, 1, 0x38},
    {0x67, 1, 0x04},
    {0x68, 1, 0x34},
    {0x69, 1, 0x03},
    {0x61, 1, 0x03},
    {0x62, 1, 0x38},
    {0x63, 1, 0x04},
    {0x64, 1, 0x34},
    {0x0A, 1, 0x11},
    {0x0B, 1, 0x14},
    {0x0c, 1, 0x14},
    {0x55, 1, 0x06},
    //V0.5
    {0x30, 1, 0xEE},

    lcd_set_page(0x12), //page12
    {0x0D, 1, 0x66},

    lcd_set_page(0x17), //page17
    {0x39, 1, 0x3c},
    //V0.5
    {0x11, 1, 0xAA},
    {0x16, 1, 0x12},
    {0x0B, 1, 0xC3},
    {0x10, 1, 0x0E},
    {0x14, 1, 0xAA},
    {0x18, 1, 0xA0},
    {0x1A, 1, 0x80},
    {0x1F, 1, 0x80},

    lcd_set_page(0x31),//Gamma page31
    {0x00, 1, 0x00},//L255
    {0x01, 1, 0x00},
    {0x02, 1, 0x00},//L254
    {0x03, 1, 0x09},
    {0x04, 1, 0x00},//L252
    {0x05, 1, 0x1c},
    {0x06, 1, 0x00},//L250
    {0x07, 1, 0x36},
    {0x08, 1, 0x00},//L248
    {0x09, 1, 0x3d},
    {0x0a, 1, 0x00},//L246
    {0x0b, 1, 0x54},
    {0x0c, 1, 0x00},//L244
    {0x0d, 1, 0x62},
    {0x0e, 1, 0x00},//L242
    {0x0f, 1, 0x72},
    {0x10, 1, 0x00},//L240
    {0x11, 1, 0x79},
    {0x12, 1, 0x00},//L231
    {0x13, 1, 0xa6},
    {0x14, 1, 0x00},//L223
    {0x15, 1, 0xd0},
    {0x16, 1, 0x01},//L207
    {0x17, 1, 0x0e},
    {0x18, 1, 0x01},//L191
    {0x19, 1, 0x3d},
    {0x1a, 1, 0x01},//L159
    {0x1b, 1, 0x7b},//L159(0x18b)
    {0x1c, 1, 0x01},//L127
    {0x1d, 1, 0xcf},//L127(0x1cf)
    {0x1e, 1, 0x02},//L95
    {0x1f, 1, 0x0E},//L95(0x20E)
    {0x20, 1, 0x02},//L63
    {0x21, 1, 0x53},//L63(0x253)
    {0x22, 1, 0x02},//L47
    {0x23, 1, 0x80},//L47(0x280)
    {0x24, 1, 0x02},//L31
    {0x25, 1, 0xC2},//L31(0x2c2)
    {0x26, 1, 0x02},//L23
    {0x27, 1, 0xFA},//L23(0x2FA)
    {0x28, 1, 0x03},//L15
    {0x29, 1, 0x3E},//L15(0x370)
    {0x2a, 1, 0x03},//L13
    {0x2b, 1, 0x52},//L13(0x3a0)
    {0x2c, 1, 0x03},//L11
    {0x2d, 1, 0x70},
    {0x2e, 1, 0x03},//L9
    {0x2f, 1, 0x8E},
    {0x30, 1, 0x03},//L7
    {0x31, 1, 0xA2},
    {0x32, 1, 0x03},//L5
    {0x33, 1, 0xBA},
    {0x34, 1, 0x03},//L3
    {0x35, 1, 0xCF},
    {0x36, 1, 0x03},//L1
    {0x37, 1, 0xe8},
    {0x38, 1, 0x03},//L0
    {0x39, 1, 0xf0},

    lcd_set_page(0x32),//Gamma page32
    {0x00, 1, 0x00},//L255
    {0x01, 1, 0x00},
    {0x02, 1, 0x00},//L254
    {0x03, 1, 0x09},
    {0x04, 1, 0x00},//L252
    {0x05, 1, 0x1c},
    {0x06, 1, 0x00},//L250
    {0x07, 1, 0x36},
    {0x08, 1, 0x00},//L248
    {0x09, 1, 0x3d},
    {0x0a, 1, 0x00},//L246
    {0x0b, 1, 0x54},
    {0x0c, 1, 0x00},//L244
    {0x0d, 1, 0x62},
    {0x0e, 1, 0x00},//L242
    {0x0f, 1, 0x72},
    {0x10, 1, 0x00},//L240
    {0x11, 1, 0x79},
    {0x12, 1, 0x00},//L231
    {0x13, 1, 0xa6},
    {0x14, 1, 0x00},//L223
    {0x15, 1, 0xd0},
    {0x16, 1, 0x01},//L207
    {0x17, 1, 0x0e},
    {0x18, 1, 0x01},//L191
    {0x19, 1, 0x3d},
    {0x1a, 1, 0x01},//L159
    {0x1b, 1, 0x7b},//L159(0x18b)
    {0x1c, 1, 0x01},//L127
    {0x1d, 1, 0xcf},//L127(0x1cf)
    {0x1e, 1, 0x02},//L95
    {0x1f, 1, 0x0E},//L95(0x20E)
    {0x20, 1, 0x02},//L63
    {0x21, 1, 0x53},//L63(0x253)
    {0x22, 1, 0x02},//L47
    {0x23, 1, 0x80},//L47(0x280)
    {0x24, 1, 0x02},//L31
    {0x25, 1, 0xC2},//L31(0x2c2)
    {0x26, 1, 0x02},//L23
    {0x27, 1, 0xFA},//L23(0x2FA)
    {0x28, 1, 0x03},//L15
    {0x29, 1, 0x3E},//L15(0x370)
    {0x2a, 1, 0x03},//L13
    {0x2b, 1, 0x52},//L13(0x3a0)
    {0x2c, 1, 0x03},//L11
    {0x2d, 1, 0x70},
    {0x2e, 1, 0x03},//L9
    {0x2f, 1, 0x8E},
    {0x30, 1, 0x03},//L7
    {0x31, 1, 0xA2},
    {0x32, 1, 0x03},//L5
    {0x33, 1, 0xBA},
    {0x34, 1, 0x03},//L3
    {0x35, 1, 0xCF},
    {0x36, 1, 0x03},//L1
    {0x37, 1, 0xe8},
    {0x38, 1, 0x03},//L0
    {0x39, 1, 0xf0},

    //for change pattern star isuue
    lcd_set_page(0x2D), //page2D
    {0x01, 1, 0x3E},//0x3e

    /*
    //VDD
    lcd_set_page(0x12), //page12
    {0x06, 1, 0x07}, //0x04

    //1Hz
    lcd_set_page(0x11),//page11
    {0x1A, 1, 0x09},
    //
    */

    //V0.5
    lcd_set_page(0x18), //page18
    {0x01, 1, 0x01},
    {0x00, 1, 0x1E},

    lcd_set_page(0x43), //page43
    {0x03, 1, 0x04},

    //====20210218 from Elmer for Touch setting
    lcd_set_page(0x50), //page50
    {0x05, 1, 0x00},//(00) 0x08 for touch auto run
    {0x00, 1, 0xA6},
    {0x01, 1, 0xA6},
    {0x08, 1, 0x55},

    lcd_set_page(0x44), //page44
    {0x09, 1, 0x0E},
    {0x0A, 1, 0x0F},

    //========================

    lcd_set_page(0x00), //page00

    {0x36, 1, 0x00},
    {0x3A, 1, 0x05},

    {0x35, 1, 0x00},

    {0x2A, 4, 0x00, 0x00, 0x01, 0x63},
    {0x2B, 4, 0x00, 0x00, 0x01, 0x8F},

    {0x11, 1, 0x00}
};

#else

static const uint8_t lcd_init_cmds[][MAX_CMD_LEN] =
{
    {0xE4, 1, 0x2},
    lcd_set_page(0x10),
    {0x0C, 1, 0x11},
    {0x10, 1, 0x02},
    {0x11, 1, 0x11},
    {0x15, 1, 0x42},
    {0x16, 1, 0x11},
    {0x1A, 1, 0x02},
    {0x1B, 1, 0x11},
    {0x61, 1, 0x80},
    {0x62, 1, 0x80},
    {0x54, 1, 0x44},
    {0x58, 1, 0x88},
    {0x5C, 1, 0xCC},
    {0x20, 1, 0x80},
    {0x21, 1, 0x81},
    {0x22, 1, 0x31},
    {0x23, 1, 0x20},
    {0x24, 1, 0x11},
    {0x25, 1, 0x11},
    {0x26, 1, 0x12},
    {0x27, 1, 0x12},
    {0x30, 1, 0x80},
    {0x31, 1, 0x81},
    {0x32, 1, 0x31},
    {0x33, 1, 0x20},
    {0x34, 1, 0x11},
    {0x35, 1, 0x11},
    {0x36, 1, 0x12},
    {0x37, 1, 0x12},
    {0x41, 1, 0x11},
    {0x42, 1, 0x22},
    {0x43, 1, 0x33},
    {0x49, 1, 0x11},
    {0x4A, 1, 0x22},
    {0x4B, 1, 0x33},

    lcd_set_page(0x15),
    {0x00, 1, 0x00},
    {0x01, 1, 0x00},
    {0x02, 1, 0x00},
    {0x03, 1, 0x00},
    {0x04, 1, 0x10},
    {0x05, 1, 0x0C},
    {0x06, 1, 0x23},
    {0x07, 1, 0x22},
    {0x08, 1, 0x21},
    {0x09, 1, 0x20},
    {0x0A, 1, 0x33},
    {0x0B, 1, 0x32},
    {0x0C, 1, 0x34},
    {0x0D, 1, 0x35},
    {0x0E, 1, 0x01},
    {0x0F, 1, 0x01},
    {0x20, 1, 0x00},
    {0x21, 1, 0x00},
    {0x22, 1, 0x00},
    {0x23, 1, 0x00},
    {0x24, 1, 0x0C},
    {0x25, 1, 0x10},
    {0x26, 1, 0x20},
    {0x27, 1, 0x21},
    {0x28, 1, 0x22},
    {0x29, 1, 0x23},
    {0x2A, 1, 0x33},
    {0x2B, 1, 0x32},
    {0x2C, 1, 0x34},
    {0x2D, 1, 0x35},
    {0x2E, 1, 0x01},
    {0x2F, 1, 0x01},

    lcd_set_page(0x16),
    {0x00, 1, 0x00},
    {0x01, 1, 0x00},
    {0x02, 1, 0x00},
    {0x03, 1, 0x00},
    {0x04, 1, 0x08},
    {0x05, 1, 0x04},
    {0x06, 1, 0x19},
    {0x07, 1, 0x18},
    {0x08, 1, 0x17},
    {0x09, 1, 0x16},
    {0x0A, 1, 0x33},
    {0x0B, 1, 0x32},
    {0x0C, 1, 0x34},
    {0x0D, 1, 0x35},
    {0x0E, 1, 0x01},
    {0x0F, 1, 0x01},
    {0x20, 1, 0x00},
    {0x21, 1, 0x00},
    {0x22, 1, 0x00},
    {0x23, 1, 0x00},
    {0x24, 1, 0x04},
    {0x25, 1, 0x08},
    {0x26, 1, 0x16},
    {0x27, 1, 0x17},
    {0x28, 1, 0x18},
    {0x29, 1, 0x19},
    {0x2A, 1, 0x33},
    {0x2B, 1, 0x32},
    {0x2C, 1, 0x34},
    {0x2D, 1, 0x35},
    {0x2E, 1, 0x01},
    {0x2F, 1, 0x01},

    lcd_set_page(0x12),
    {0x00, 1, 0x99},
    {0x2A, 1, 0x28},
    {0x2B, 1, 0x0F},
    {0x2C, 1, 0x16},
    {0x2D, 1, 0x28},
    {0x2E, 1, 0x0F},

    lcd_set_page(0xA0),
    {0x08, 1, 0xDC},

    lcd_set_page(0x45),
    {0x03, 1, 0x64},

    lcd_set_page(0x40),
    {0x86, 1, 0x00},

    lcd_set_page(0x00),
    {REG_CASET, 4, 0x00, 0x00, 0x01, 0x63},  // Set Column (0->0x163(355))
    {REG_RASET, 4, 0x00, 0x00, 0x01, 0x8F},  // Set Row (0->0x18F(399))

    lcd_set_page(0x42),
    {0x05, 1, 0x2C},

    lcd_set_page(0x11),
    {0x50, 1, 0x11},

    lcd_set_page(0x12),
    {0x0D, 1, 0x66},

    lcd_set_page(0x17),
    {0x39, 1, 0x3C},

    lcd_set_page(0x11),
    {0x60, 1, 0x01},
    {0x65, 1, 0x03},
    {0x66, 1, 0x38},
    {0x67, 1, 0x04},
    {0x68, 1, 0x34},
    {0x69, 1, 0x03},
    {0x61, 1, 0x03},
    {0x62, 1, 0x38},
    {0x63, 1, 0x04},
    {0x64, 1, 0x34},
    {0x0A, 1, 0x11},
    {0x0B, 1, 0x14},
    {0x0C, 1, 0x14},
    {0x55, 1, 0x06},

    lcd_set_page(0x42),
    {0x05, 1, 0x3D},
    {0x06, 1, 0x33},
#if 0
    lcd_set_page(0x00),
    {0x36, 1, 0x00},
    {0x3A, 1, 0x05},

    lcd_set_page(0x50),
    {0x05, 1, 0x00},
    lcd_set_page(0x00),
    lcd_set_page(0x50),
    {0x00, 1, 0xA6},
    {0x01, 1, 0xA6},
    lcd_set_page(0x00),
    lcd_set_page(0x50),
    {0x08, 1, 0x55},
#endif

#if 1 // For Touch
    lcd_set_page(0x50),
    {0x05, 1, 0x00},
    {0x00, 1, 0xA6},
    {0x01, 1, 0xA6},
    {0x08, 1, 0x55},
    lcd_set_page(0x44),
    {0x09, 1, 0x0E},
    {0x0A, 1, 0x0F},
#endif

    lcd_set_page(0x00),
    {0x3A, 1, 0x75},                         // Set to 565
    {0x51, 2, 0xff, 0x3f},                   // Set brightness
    {REG_SLEEP_OUT, 0},                  // Sleep Out, turn off sleep mode
};
#endif

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

static void SPD2012_Init_SPI_Mode(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];
    int i, j;

    /* Initialize SPD2012 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(1);
    BSP_LCD_Reset(1);

    /* Wait for 20ms */
    LCD_DRIVER_DELAY_MS(20);

    for (i = 0; i < sizeof(lcd_init_cmds) / MAX_CMD_LEN; i++)
    {
        //rt_kprintf("write %d,cmd=0x%x,len=%d\n",i,(int)lcd_init_cmds[i][0], (int)lcd_init_cmds[i][1]);
        //HAL_DBG_print_data((char*)&(lcd_init_cmds[i][2]),0,(int)lcd_init_cmds[i][1]);
        LCD_WriteReg(hlcdc, lcd_init_cmds[i][0], (uint8_t *)&lcd_init_cmds[i][2], lcd_init_cmds[i][1]);

        //__asm("B .");
    }
    LCD_DRIVER_DELAY_MS(20);

    /*Clear gram*/
    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    HAL_LCDC_SetROIArea(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH, THE_LCD_PIXEL_HEIGHT);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

    LCD_WriteReg(hlcdc, 0x29, NULL, 0);

}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_qadspi, sizeof(lcdc_int_cfg));

    for (uint32_t retry = 0; retry < 10; retry++)
    {
        {
            SPD2012_Init_SPI_Mode(hlcdc);
        }
        LOG_I("SPD2012_Init\r\n");
        {
            uint32_t data;

            data = LCD_ReadData(hlcdc, REG_POWER_MODE, 2);
            DEBUG_PRINTF("\nSPD2012_Read POWER MODE 0x%x \n", data);

            if (data != 0)
            {
                break;//Success
            }
            else
            {
                LOG_I("SPD2012_Init retry %d", retry);
                BSP_LCD_PowerDown();
                LCD_DRIVER_DELAY_MS(100 * retry);
                BSP_LCD_PowerUp();
            }
        }
    }
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
    DEBUG_PRINTF("\nSPD2012_ReadID 0x%x \n", data);
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
    /* Display On */
    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

#ifdef BSP_USING_TOUCHD
    spd2012tp_reset();
#endif /* BSP_USING_TOUCHD */
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
    LCD_WriteReg(hlcdc, REG_SLEEP_IN, (uint8_t *)NULL, 0);

}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t   parameter[4];
    SPD2012_CHECK_START_XY(Xpos0);
    SPD2012_CHECK_START_XY(Ypos0);
    SPD2012_CHECK_END_XY(Xpos1);
    SPD2012_CHECK_END_XY(Ypos1);

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
    return 0; //Not supprted

#if 0
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("SPD2012_ReadPixel[%d,%d]\n", Xpos, Ypos);

    SPD2012_CHECK_START_XY(Xpos);
    SPD2012_CHECK_START_XY(Ypos);

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

    return;

    /*

    Control interface color format
    ��011�� = 12bit/pixel ��101�� = 16bit/pixel ��110�� = 18bit/pixel ��111�� = 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x75;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
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
    DEBUG_PRINTF("\nSPD2012_color_format 0x%x \n", data);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    //uint8_t bright = (uint8_t)((int)SPD2012_BRIGHTNESS_MAX * br / 100);
    //LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);

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

static const LCD_DrvOpsDef SPD2012_drv =
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
    NULL,
    NULL
};
LCD_DRIVER_EXPORT2(SPD2012, THE_LCD_ID, &lcdc_int_cfg,
                   &SPD2012_drv, 4);

/****************************************************************************************************/

#ifdef BSP_USING_TOUCHD

/********************************spd2012 tp********************************/
#include "drv_touch.h"
static struct touch_drivers spd2012tp_driver;
static struct rt_i2c_bus_device *spd2012tp_i2c_bus = NULL;
static uint8_t handshake_flag = 0;
static uint8_t tp_inited = 0;

static rt_err_t spd2012tp_i2c_write(uint8_t *buf, uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = SPD2012_I2C_ADDR;    /* slave address */
    msgs.flags = RT_I2C_WR;        /* write flag */
    msgs.buf   = buf;              /* Send data pointer */
    msgs.len   = len;

    if (rt_i2c_transfer(spd2012tp_i2c_bus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_err_t spd2012tp_i2c_read(uint8_t *buf, uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = SPD2012_I2C_ADDR;    /* Slave address */
    msgs[0].flags = RT_I2C_RD;        /* Read flag */
    msgs[0].buf   = buf;              /* Read data pointer */
    msgs[0].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(spd2012tp_i2c_bus, msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_size_t spd2012tp_i2c_xfer(uint8_t *reg, uint16_t wlen, uint8_t *rbuf, uint16_t rlen)
{
    struct rt_i2c_msg msgs[2];
    rt_size_t res = 0;

    msgs[0].addr  = SPD2012_I2C_ADDR;    /* slave address */
    msgs[0].flags = RT_I2C_WR;        /* write flag */
    msgs[0].buf   = reg;              /* Send data pointer */
    msgs[0].len   = wlen;

    msgs[1].addr  = SPD2012_I2C_ADDR;    /* slave address */
    msgs[1].flags = RT_I2C_RD;        /* write flag */
    msgs[1].buf   = rbuf;              /* Send data pointer */
    msgs[1].len   = rlen;

    res = rt_i2c_transfer(spd2012tp_i2c_bus, msgs, 2);

    return res;
}

void spd2012tp_reset(void)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;
    rt_err_t ret;

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at spd2012tp reset\r\n");
        return ;
    }
    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    //set mode
    m.pin = SPD2012_TP_RST;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);
    //set value H/L/H
#if 0
    st.pin = SPD2012_TP_RST;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_thread_mdelay(4);
#endif
    st.pin = SPD2012_TP_RST;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_thread_mdelay(10);

    st.pin = SPD2012_TP_RST;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    handshake_flag = 1;

    LOG_I("spd2012tp_reset \r\n");

}

static void spd2012tp_get_pos(touch_msg_t p_msg, uint8_t *reg, uint8_t reg_len, uint8_t *rdbuf, uint8_t rd_len)
{
    reg[0] = 0x00;
    reg[1] = 0x03;
    rt_size_t ret = spd2012tp_i2c_xfer(&reg[0], reg_len, &rdbuf[0], rd_len);    //size 10, single finger

    if (ret == 0) rt_memset(&rdbuf[0], 0, rd_len);

    //for(uint8_t i=0; i<10; i++)
    //rt_kprintf("spd2012tp_get_pos %d 0x%x\r\n",i ,rdbuf[i]);

    if (rdbuf[8]) // weight state
    {
        p_msg->event = TOUCH_EVENT_DOWN;
        //rt_kprintf("press_state = 1\r\n");
    }
    else
    {
        p_msg->event = TOUCH_EVENT_UP;
        //rt_kprintf("press_state = 0\r\n");
    }

    if (rdbuf[0] == 0x12) //point info
    {
        p_msg->x = (rdbuf[7] & 0xf0) << 4 | rdbuf[5];
        p_msg->y = (rdbuf[7] & 0x0f) << 8 | rdbuf[6];
    }
    else
    {
        p_msg->x = 0;
        p_msg->y = 0;
    }

    LOG_D("spd2012tp_get_pos  state %d,   x %d y %d", p_msg->event, p_msg->x, p_msg->y);

}

void spd2012tp_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("spd2012tp touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(spd2012tp_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    uint8_t retry;
    uint8_t reg[2];
    uint8_t rbuf[4];
    uint8_t wbuf[4];
    uint8_t rdbuf[10];
    uint16_t len;

    rt_err_t ret = RT_ERROR;

    LOG_D("spd2012tp read_point");

    rt_touch_irq_pin_enable(1);

    reg[0] = 0x20;
    reg[1] = 0x00;
    if (handshake_flag == 1)
    {
        for (retry = 10; retry > 0; retry--, rt_thread_mdelay(20))
        {
            //bios startup
            spd2012tp_i2c_xfer(&reg[0], sizeof(reg), &rbuf[0], sizeof(rbuf));
            //rt_kprintf("bios startup: 0x%x 0x%x 0x%x 0x%x\r\n",rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
            if (rbuf[1] == 0x40)
            {
                wbuf[0] = 0x02;
                wbuf[1] = 0x00;
                wbuf[2] = 0x01;
                wbuf[3] = 0x00;
                spd2012tp_i2c_write(wbuf, sizeof(wbuf));   //clear int
                wbuf[0] = 0x04;
                wbuf[1] = 0x00;
                wbuf[2] = 0x01;
                wbuf[3] = 0x00;
                spd2012tp_i2c_write(wbuf, sizeof(wbuf));      // cpu start

                break;
            }
            else
            {
                LOG_E("tp boot busy = %x, %x, %x\r\n", rbuf[1], rbuf[2], rbuf[3]);
            }
        }
        handshake_flag++;

        p_msg->event = TOUCH_EVENT_UP;
        p_msg->x     = 0;
        p_msg->y     = 0;
        ret = RT_ERROR;
    }
    else if (handshake_flag == 2)
    {
        //cpu startup
        for (retry = 10; retry > 0; retry--, rt_thread_mdelay(20))
        {
            spd2012tp_i2c_xfer(&reg[0], sizeof(reg), &rbuf[0], sizeof(rbuf));

            if (rbuf[1] == 0x30)
            {
                wbuf[0] = 0x50;
                wbuf[1] = 0x00;
                wbuf[2] = 0x00;
                wbuf[3] = 0x00;
                spd2012tp_i2c_write(wbuf, sizeof(wbuf));   //touch mode
                wbuf[0] = 0x46;
                wbuf[1] = 0x00;
                wbuf[2] = 0x00;
                wbuf[3] = 0x00;
                spd2012tp_i2c_write(wbuf, sizeof(wbuf));      // touch start
                wbuf[0] = 0x02;
                wbuf[1] = 0x00;
                wbuf[2] = 0x01;
                wbuf[3] = 0x00;
                spd2012tp_i2c_write(wbuf, sizeof(wbuf));      // clear int

                break;
            }
            else
            {
                LOG_E("cpu startup: 0x%x 0x%x 0x%x 0x%x\r\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
            }
        }
        handshake_flag = 0;

        p_msg->event = TOUCH_EVENT_UP;
        p_msg->x     = 0;
        p_msg->y     = 0;
        ret = RT_ERROR;

    }
    else // read pos
    {
        spd2012tp_i2c_xfer(&reg[0], sizeof(reg), &rbuf[0], sizeof(rbuf));

        len = rbuf[3] << 8 |  rbuf[2];
        if (len > 0)
        {
            spd2012tp_get_pos(p_msg, &reg[0], sizeof(reg), &rdbuf[0], sizeof(rdbuf));
        }

        for (retry = 10; retry > 0; retry--, rt_thread_mdelay(20))
        {
            reg[0] = 0xfc;
            reg[1] = 0x02;
            spd2012tp_i2c_xfer(&reg[0], sizeof(reg), &rdbuf[0], sizeof(rdbuf) - 2); //size 8

            if (rdbuf[5] == 0x81)
            {
                LOG_E("SDP status 0x%x\r\n", rdbuf[5]);
            }
            else
            {
                break;
            }
        }

        if (rdbuf[5] == 0x82)
        {
            wbuf[0] = 0x02;
            wbuf[1] = 0x00;
            wbuf[2] = 0x01;
            wbuf[3] = 0x00;
            spd2012tp_i2c_write(wbuf, sizeof(wbuf));      // clear int
        }
        else if (rdbuf[5] == 0x00)
        {
            spd2012tp_get_pos(p_msg, &reg[0], sizeof(reg), &rdbuf[0], sizeof(rdbuf));
            wbuf[0] = 0x02;
            wbuf[1] = 0x00;
            wbuf[2] = 0x01;
            wbuf[3] = 0x00;
            spd2012tp_i2c_write(wbuf, sizeof(wbuf));      // clear int
        }

        ret = RT_EEMPTY; //No more tp data
    }

    return ret;
}

static rt_err_t init(void)
{
    struct touch_message msg;

    LOG_I("spd2012tp init");

    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;
    rt_err_t ret = RT_EOK;

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at spd2012tp int\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    /* Setup touch irq pin */
    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, spd2012tp_irq_handler, NULL);
    rt_touch_irq_pin_enable(1); //Must enable before read I2C

    /* Send reset pulse */
    m.pin = SPD2012_TP_RST;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);
    //set value H/L/H
#if 0
    st.pin = SPD2012_TP_RST;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_thread_mdelay(4);
#endif
    st.pin = SPD2012_TP_RST;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_thread_mdelay(10);

    st.pin = SPD2012_TP_RST;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    handshake_flag = 1;

    LOG_I("spd2012tp_reset \r\n");

    LOG_I("spd2012tp init OK");

    return ret;
}

static rt_err_t deinit(void)
{
    LOG_I("spd2012tp deinit");

    rt_touch_irq_pin_enable(0);

    return RT_EOK;
}

static rt_bool_t probe(void)
{
    rt_err_t err;

    spd2012tp_i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(TOUCH_DEVICE_NAME);
    if (RT_Device_Class_I2CBUS != spd2012tp_i2c_bus->parent.type)
    {
        spd2012tp_i2c_bus = NULL;
    }
    if (spd2012tp_i2c_bus)
    {
        rt_device_open((rt_device_t)spd2012tp_i2c_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    }
    else
    {
        LOG_E("bus not find\n");
        return RT_FALSE;
    }

    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 5000,
            .max_hz = 400000,
        };

        rt_i2c_configure(spd2012tp_i2c_bus, &configuration);
    }

    LOG_I("spd2012tp probe OK");

    return RT_TRUE;
}

static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};

static int rt_spd2012tp_init(void)
{
    spd2012tp_driver.probe = probe;
    spd2012tp_driver.ops = &ops;
    spd2012tp_driver.user_data = RT_NULL;
    spd2012tp_driver.isr_sem = rt_sem_create("spd2012tp", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&spd2012tp_driver);

    return 0;
}

INIT_COMPONENT_EXPORT(rt_spd2012tp_init);
#endif /* BSP_USING_TOUCHD */

