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
  * @brief LS013B7DD02 chip IDs
  */
#define THE_LCD_ID                  0x1d1

/**
  * @brief  LS013B7DD02 Size
  */
#define  THE_LCD_PIXEL_WIDTH    ((uint16_t)260)
#define  THE_LCD_PIXEL_HEIGHT   ((uint16_t)260)

/**
  * @brief  LS013B7DD02 Registers
  */
#define REG_LCD_ID             0x04
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

//#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_JDI_PARALLEL,
    .freq = 746268, //HCK frequency
    .color_mode = LCDC_PIXEL_FORMAT_RGB565, // Useless parameter for JDI PARALLEL interface

    .cfg = {
        .jdi = {
            .bank_col_head = 0,
            .valid_columns = THE_LCD_PIXEL_WIDTH,
            .bank_col_tail = 8,

            .bank_row_head = 0,
            .valid_rows = THE_LCD_PIXEL_HEIGHT,
            .bank_row_tail = 4,

            .enb_start_col = 32,
            .enb_end_col = 95,
        },
    },

};

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    /* Initialize LS013B7DD02 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    HAL_Delay_us(10);
    BSP_LCD_Reset(1);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    return THE_LCD_ID;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On, enable the FRP&XFRP output */
#ifdef JDI_FRP_LPPWM_INTERFACE_NAME
    struct rt_device_pwm *device = (struct rt_device_pwm *)rt_device_find(JDI_FRP_LPPWM_INTERFACE_NAME);
    if (!device)
    {
        LOG_E("Can not find FRP LPPWM device:%s", JDI_FRP_LPPWM_INTERFACE_NAME);
    }
    else
    {
        if (0 == (device->parent.open_flag & RT_DEVICE_OFLAG_OPEN))
        {
            rt_device_open((struct rt_device *)device, RT_DEVICE_OFLAG_RDWR);
            rt_pwm_set(device, 1, 16 * 1000 * 1000, 8 * 1000 * 1000); // Set period to 16ms, pulse to 8ms
            rt_pwm_enable(device, 1);
        }
    }
#endif
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off, disable the FRP&XFRP output */
#ifdef JDI_FRP_LPPWM_INTERFACE_NAME
    struct rt_device_pwm *device = (struct rt_device_pwm *)rt_device_find(JDI_FRP_LPPWM_INTERFACE_NAME);
    if (!device)
    {
        LOG_E("Can not find FRP LPPWM device:%s", JDI_FRP_LPPWM_INTERFACE_NAME);
    }
    else
    {
        if (device->parent.open_flag & RT_DEVICE_OFLAG_OPEN)
        {
            rt_pwm_disable(device, 1);
            rt_device_close((struct rt_device *)device);
        }
    }
#endif
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    HAL_LCDC_SetROIArea(hlcdc, 0, Ypos0, THE_LCD_PIXEL_WIDTH - 1, Ypos1); //Not support partical columns
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
}

static const LCD_DrvOpsDef LS013B7DD02_drv =
{
    LCD_Init,
    LCD_ReadID,
    LCD_DisplayOn,
    LCD_DisplayOff,

    LCD_SetRegion,
    NULL,
    LCD_WriteMultiplePixels,

    NULL,

    NULL,
    NULL,
    NULL,
    NULL
};

LCD_DRIVER_EXPORT2(ls013b7dd02, THE_LCD_ID, &lcdc_int_cfg,
                   &LS013B7DD02_drv, 1);

