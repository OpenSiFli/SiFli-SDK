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

#ifdef ROW_OFFSET_PLUS
    #define ROW_OFFSET  (ROW_OFFSET_PLUS)
#else
    #define ROW_OFFSET  (0)
#endif

/**
  * @brief ST77903 chip IDs
  */
#define THE_LCD_ID                  0x85

/**
  * @brief  ST77903 Size
  */
#define  THE_LCD_PIXEL_WIDTH    ((uint16_t)400)
#define  THE_LCD_PIXEL_HEIGHT   ((uint16_t)400)

/**
  * @brief  ST77903 Registers
  */
#define REG_LCD_ID             0x04
#define REG_POWER_MODE         0x09
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
#define REG_WBRIGHT            0x51

#define REG_VDV_VRH_EN         0xC2
#define REG_VDV_SET            0xC4

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static uint32_t LCD_ESDCehck(LCDC_HandleTypeDef *hlcdc);
static void LCD_TimeoutDbg(LCDC_HandleTypeDef *hlcdc);
static void LCD_TimeoutReset(LCDC_HandleTypeDef *hlcdc);

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_SPI_DCX_4DATA_AUX,
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0,
            .syn_mode = HAL_LCDC_SYNC_DISABLE,

            /*

                1line data   ----(delay > 40us)---------------> 1line data  (Vporch line same too)

                1 frame      ------(delay > 1ms)--------------->   1 frame

            */
            .frame_cmd = 0xD8006100,
            .frame_gap = 1100,     //In microsecond
            .porch_cmd = 0xD8006000,
            .line_cmd = 0xDE006000,
#if 1//defined(LCD_USING_ED_LB55_77903_LHZL_320X320_QADSPI_LB551)
            .porch_line_gap = 40, //55,      //In microsecond
            .data_line_gap  = 11, //55,     //In microsecond
            .front_porch = 8,   //Front porch line
            .back_porch = 8,    //Back porch line
#else
            .porch_line_gap = 40,      //In microsecond
            .data_line_gap  = 5,     //In microsecond
            .front_porch = 8,   //Front porch line
            .back_porch = 8,    //Back porch line
#endif
            .backlight_pwm = 0,
            .backlight_polarity = 1,

        },
    },

};

#define MAX_CMD_LEN 16
#if 1//defined(LCD_USING_ED_LB55SPI17601_QADSPI_SS6600)
static const uint8_t lcd_init_cmds[][MAX_CMD_LEN] =
{
    { 0xf0, 1,  0xc3},
    { 0xf0, 1,  0x96},
    { 0xf0, 1,  0xa5},
    //{ 0xe9, 1,  0x20},
    { 0xe7, 4,  0x80, 0x77, 0x1f, 0xcc},
    { 0xc1, 4,  0x11, 0x09, 0xaa, 0x11},
    { 0xc2, 4,  0x11, 0x09, 0xaa, 0x11},
    { 0xc3, 4,  0x44, 0x03, 0x33, 0x04},
    { 0xc4, 4,  0x44, 0x03, 0x33, 0x04},
    { 0xc5, 1,  0x4d},
    { 0xd6, 1,  0x00},
    { 0xd7, 1,  0x00},
    { 0xe0, 14, 0xf0, 0x0d, 0x14, 0x0a, 0x09, 0x06, 0x3a, 0x43, 0x51, 0x07, 0x14, 0x15, 0x30, 0x35},
    { 0xe1, 14, 0xf0, 0x0d, 0x13, 0x0b, 0x09, 0x07, 0x3b, 0x43, 0x50, 0x08, 0x14, 0x15, 0x30, 0x35},
    { 0xe5, 14, 0xb9, 0xf5, 0xb5, 0x55, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22},
    { 0xe6, 14, 0xb9, 0xf5, 0xb5, 0x55, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22},
    { 0xe7, 1,  0x80},
    { 0xec, 6,  0x00, 0x55, 0x00, 0x00, 0x00, 0x88},
    { 0x36, 1,  0x08},
    { 0x3a, 1,  0x05},  //0x07-rgb888  05-rgb565
    { 0xb1, 2,  0xfe, 0xdf},
    { 0xb2, 1,  0x09},
    { 0xb3, 1,  0x01},
    { 0xb4, 1,  0x01},
    { 0xb5, 4,  0x00, 0x08, 0x00, 0x08},
    { 0xb6, 2,  0xbd, 0x27},
    { 0xa4, 2,  0xc0, 0x6b},
    { 0xa5, 9,  0x11, 0x53, 0x00, 0x00, 0x20, 0x15, 0x2a, 0xba, 0x02},
    { 0xa6, 9,  0x11, 0x53, 0x00, 0x00, 0x20, 0x15, 0x2a, 0xba, 0x02},
    { 0xba, 7,  0x58, 0x0a, 0x34, 0x10, 0x22, 0x01, 0x00},
    { 0xbb, 8,  0x00, 0x33, 0x00, 0x2c, 0x83, 0x07, 0x18, 0x00},
    { 0xbc, 8,  0x00, 0x33, 0x00, 0x2c, 0x83, 0x07, 0x18, 0x00},
    { 0xbd, 11, 0x21, 0x12, 0xff, 0xff, 0x67, 0x58, 0x85, 0x76, 0xab, 0xff, 0x03},
    { 0x35, 1,  0x00},
    { 0xed, 1,  0xc3},
    { 0xd9, 1,  0x22},
    { 0x36, 1,  0x0c}
};

//Display  color bar to check LCD ok.
static const uint8_t bist_cmds[][MAX_CMD_LEN] =
{
    {0xf0, 1,  0xa5},
    {0xb0, 1,  0xa5},
    {0xcc, 9, 0x40, 0x00, 0x3f, 0x01, 0x06, 0x06, 0x55, 0x55, 0x00},
};
#else
static const uint8_t lcd_init_cmds[][MAX_CMD_LEN] =
{
    { 0xf0, 1,  0xc3},
    { 0xf0, 1,  0x96},
    { 0xf0, 1,  0xa5},
    { 0xe9, 1,  0x20},
    { 0xe7, 4,  0x80, 0x77, 0x1f, 0xcc},
    { 0xc1, 4,  0x77, 0x07, 0xcf, 0x16},
    { 0xc2, 4,  0x77, 0x07, 0xcf, 0x16},
    { 0xc3, 4,  0x22, 0x02, 0x22, 0x04},
    { 0xc4, 4,  0x22, 0x02, 0x22, 0x04},
    { 0xc5, 1,  0xed},
    { 0xe0, 14, 0x87, 0x09, 0x0c, 0x06, 0x05, 0x03, 0x29, 0x32, 0x49, 0x0f, 0x1b, 0x17, 0x2a, 0x2f},
    { 0xe1, 14, 0x87, 0x09, 0x0c, 0x06, 0x05, 0x03, 0x29, 0x32, 0x49, 0x0f, 0x1b, 0x17, 0x2a, 0x2f},
    { 0xe5, 14, 0xbe, 0xf5, 0xb1, 0x22, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22},
    { 0xe6, 14, 0xbe, 0xf5, 0xb1, 0x22, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22},
    { 0xec, 2,  0x40, 0x03},
    { 0x36, 1,  0x0c},
    { 0x3a, 1,  0x05},  //0x07-rgb888  05-rgb565
    { 0xb2, 1,  0x00},
    { 0xb3, 1,  0x01},
    { 0xb4, 1,  0x00},
    { 0xb5, 4,  0x00, 0x08, 0x00, 0x08},
    { 0xa5, 9,  0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x2a, 0x8a, 0x02},
    { 0xa6, 9,  0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x2a, 0x8a, 0x02},
    { 0xba, 7,  0x0a, 0x5a, 0x23, 0x10, 0x25, 0x02, 0x00},
    { 0xbb, 8,  0x00, 0x30, 0x00, 0x2c, 0x82, 0x87, 0x18, 0x00},
    { 0xbc, 8,  0x00, 0x30, 0x00, 0x2c, 0x82, 0x87, 0x18, 0x00},
    { 0xbd, 11, 0xa1, 0xb2, 0x2b, 0x1a, 0x56, 0x43, 0x34, 0x65, 0xff, 0xff, 0x0f}

};

//Display  color bar to check LCD ok.
static const uint8_t bist_cmds[][MAX_CMD_LEN] =
{
    {0xb0, 1,  0xa5},
    {0xcc, 9, 0x40, 0x00, 0x3f, 0x00, 0x14, 0x14, 0x20, 0x20, 0x03},
};

#endif

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
            HAL_LCDC_SetFreq(hlcdc, 4000000); //read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq); //Restore normal frequency
        }
    }
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    /* Initialize ST77903 low level bus layer ----------------------------------*/
    {
        //Calculate data line gap
        uint32_t cycle_per_us = lcdc_int_cfg.freq / 1000000;
        uint32_t data_cost_us = ((LCD_HOR_RES_MAX * 4 /*RGB565*/) + 32 /*32bit cmd*/) / cycle_per_us;

        lcdc_int_cfg.cfg.spi.data_line_gap = (data_cost_us > 41) ? 0 : (41 - data_cost_us);
    }
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(30);
    BSP_LCD_Reset(1);

    /* Wait for 120ms */
    LCD_DRIVER_DELAY_MS(120);

    for (int i = 0; i < sizeof(lcd_init_cmds) / MAX_CMD_LEN; i++)
    {
        //rt_kprintf("write %d,cmd=0x%x,len=%d\n",i,(int)lcd_init_cmds[i][0], (int)lcd_init_cmds[i][1]);
        //HAL_DBG_print_data((char*)&(lcd_init_cmds[i][2]),0,(int)lcd_init_cmds[i][1]);
        LCD_WriteReg(hlcdc, lcd_init_cmds[i][0], (uint8_t *)&lcd_init_cmds[i][2], lcd_init_cmds[i][1]);

        //__asm("B .");
    }

    /* Tearing Effect Line On: Option (00h:VSYNC Only, 01h:VSYNC & HSYNC ) */
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, REG_TEARING_EFFECT, parameter, 1);

    LCD_WriteReg(hlcdc, REG_DISPLAY_INVERSION, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
    /* Wait for 120ms */
    LCD_DRIVER_DELAY_MS(120);

    /* Display ON command */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
    /* Wait for 120ms */
    LCD_DRIVER_DELAY_MS(120);

#if 0
    for (int i = 0; i < sizeof(bist_cmds) / MAX_CMD_LEN; i++)
    {
        LCD_WriteReg(hlcdc, bist_cmds[i][0], (uint8_t *)&bist_cmds[i][2], bist_cmds[i][1]);

    }
    __asm("B .");
#endif /* 1 */

    HAL_LCDC_SetROIArea(hlcdc, 0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);

    {
        uint32_t data;

        data = LCD_ReadData(hlcdc, REG_LCD_ID, 4);
        DEBUG_PRINTF("ST77903_ReadID 0x%x \n", data);

        data = LCD_ReadData(hlcdc, REG_POWER_MODE, 4);
        DEBUG_PRINTF("ST77903_ReadPowerMode 0x%x \n", data);
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

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 4);
    DEBUG_PRINTF("ST77903_ReadID 0x%x \n", data);
    data = ((data << 1) >> 8) & 0xFFFFFF;

    return THE_LCD_ID;
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
    /*
        0x2A & 0X2B is invalid for ST77903
    */
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

#if 0

void ST77903_WriteMultiplePixels_mcu(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;
    if ((Xpos0 >= THE_LCD_PIXEL_WIDTH) || (Ypos0 >= THE_LCD_PIXEL_HEIGHT)
            || (Xpos1 >= THE_LCD_PIXEL_WIDTH) || (Ypos1 >= THE_LCD_PIXEL_HEIGHT))
    {
        return;
    }

    if ((Xpos0 > Xpos1) || (Ypos0 > Ypos1))
    {
        return;
    }
    if (LCDC_INTF_SPI_DCX_4DATA_AUX == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

        while (1)
        {
            /*

                1line data   ----(must > 40us, include Vporch line)------> 1line data

                1 frame      ------(must > 1ms)--------------->   1 frame

            */
            HAL_LCDC_WriteU32Reg(hlcdc, 0xDE006100, 0, 0);
            HAL_Delay_us(40); //Must delay 40us

            for (uint32_t back_porch = 7; back_porch > 0; back_porch--)
            {
                HAL_LCDC_WriteU32Reg(hlcdc, 0xDE006000, 0, 0);
                HAL_Delay_us(40);//Must delay 40us
            }

            for (uint16_t row = Ypos0; row < Ypos1; row++)
            {
                HAL_LCDC_SetROIArea(hlcdc, Xpos0, row, Xpos1, row);
                HAL_LCDC_SendLayerData2Reg_IT(hlcdc, 0xDE006000, 4);
                //Must delay 40us
            }

            for (uint32_t front_porch = 8; front_porch > 0; front_porch--)
            {
                HAL_LCDC_WriteU32Reg(hlcdc, 0xDE006000, 0, 0);
                HAL_Delay_us(40);  //Must delay 40us
            }

            LCD_DRIVER_DELAY_MS(1); //Must delay 1ms
        }

    }
    else
    {
        HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, REG_WRITE_RAM, 1);
    }

}
#endif

/*
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[128];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    rt_kputs("\r\n");
    va_end(args);
}
*/
//use ptc
static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    //RT_ASSERT((Xpos1 - Xpos0 + 1) == THE_LCD_PIXEL_WIDTH);

    rt_base_t level = rt_hw_interrupt_disable(); //In case RAMLESS FSM is running
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
    rt_hw_interrupt_enable(level);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
    uint32_t cmd;

    cmd = (0xDE << 24) | (LCD_Reg << 8);

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

    HAL_LCDC_ReadU32Reg(hlcdc, ((0xDD << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);

    LCD_ReadMode(hlcdc, false);

    return rd_data;
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    return 0;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{

}

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{

    LOG_I("Set lcdlight %d", br);
    uint8_t bright = (uint8_t)((uint16_t)UINT8_MAX * br / 100);
    //LCD_WriteReg(hlcdc, REG_WBRIGHT, &br, 1);

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

static void LCD_TimeoutDbg(LCDC_HandleTypeDef *hlcdc)
{
}

static void LCD_TimeoutReset(LCDC_HandleTypeDef *hlcdc)
{
    LOG_I("ST77903_TimeoutReset");

    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_mdelay(100);
    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);

    LCD_Init(hlcdc);
}

static uint32_t LCD_ESDCehck(LCDC_HandleTypeDef *hlcdc)
{
#ifndef SF32LB55X
    HAL_StatusTypeDef err;
    uint32_t data;
    uint8_t   parameter[6];
#ifdef HAL_RAMLESS_LCD_ENABLED
    if (0 == hlcdc->running) return 0;
#else
#error "Current core not supported RAMLESS LCD?"
#endif /* HAL_RAMLESS_LCD_ENABLED */

    parameter[0] = 0x58;
    parameter[1] = 0xf5;
    parameter[2] = 0x66;
    parameter[3] = 0x33;
    parameter[4] = 0x22;
    parameter[5] = 0x20;
    err = HAL_LCDC_RAMLESS_WriteDatasStart(hlcdc, 0xDE00E500, 4, &parameter[0], 6);
    RT_ASSERT(HAL_OK == err);
    LCD_DRIVER_DELAY_MS(20);
    err = HAL_LCDC_RAMLESS_WriteDatasEnd(hlcdc);
    RT_ASSERT(HAL_OK == err);

    parameter[0] = 0x58;
    parameter[1] = 0xf5;
    parameter[2] = 0x66;
    parameter[3] = 0x33;
    parameter[4] = 0x22;
    parameter[5] = 0x25;
    err = HAL_LCDC_RAMLESS_WriteDatasStart(hlcdc, 0xDE00E500, 4, &parameter[0], 6);
    RT_ASSERT(HAL_OK == err);
    LCD_DRIVER_DELAY_MS(20);
    err = HAL_LCDC_RAMLESS_WriteDatasEnd(hlcdc);
    RT_ASSERT(HAL_OK == err);

    err = HAL_LCDC_RAMLESS_ReadDatasStart(hlcdc, 4000000, 0xDD000a00, 4, 4);
    RT_ASSERT(HAL_OK == err);
    LCD_DRIVER_DELAY_MS(20);
    err = HAL_LCDC_RAMLESS_ReadDatasEnd(hlcdc, (uint8_t *)&data, 4);
    LOG_I("ST77903_ESD_CHECK 0a 0x%x \n", data);

    err = HAL_LCDC_RAMLESS_ReadDatasStart(hlcdc, 4000000, 0xDD000900, 4, 1);
    RT_ASSERT(HAL_OK == err);
    LCD_DRIVER_DELAY_MS(20);
    err = HAL_LCDC_RAMLESS_ReadDatasEnd(hlcdc, (uint8_t *)&data, 1);
    LOG_I("ST77903_ESD_CHECK 09 0x%x \n", data);
#endif /* SF32LB55X */

    return 0;

}

static const LCD_DrvOpsDef ST77903_drv =
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
    NULL,
    NULL,
    LCD_TimeoutDbg,
    LCD_TimeoutReset,
    LCD_ESDCehck,
};

LCD_DRIVER_EXPORT2(st77903, THE_LCD_ID, &lcdc_int_cfg,
                   &ST77903_drv, 1);

