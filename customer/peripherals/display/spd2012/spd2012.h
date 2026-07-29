/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPD2012_H
#define __SPD2012_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "rtconfig.h"
/** @addtogroup BSP
  * @{
  */
#ifdef TOUCH_RESET_PIN
#define  SPD2012_TP_RST    TOUCH_RESET_PIN
#else
#define  SPD2012_TP_RST    86  //Obsolete
#endif /* TOUCH_RESET_PIN */

#define  SPD2012_I2C_ADDR           (0x53)  /* I2C slave address */

/** @addtogroup Components
  * @{
  */

/** @addtogroup SPD2012
  * @{
  */

/** @defgroup SPD2012_Exported_Types
  * @{
  */
/**
  * @}
  */

/** @defgroup SPD2012_Exported_Constants
  * @{
  */

/**
  * @brief SPD2012 chip IDs
  */
#define THE_LCD_ID                  0x1190a7

/**
  * @brief  SPD2012 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (454)
#define  THE_LCD_PIXEL_HEIGHT   (454)

/**
  * @brief  SPD2012 Registers
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
#define REG_VSCRDEF            0x33 /* Vertical Scroll Definition */
#define REG_VSCSAD             0x37 /* Vertical Scroll Start Address of RAM */
#define REG_TEARING_EFFECT     0x35
#define REG_NORMAL_DISPLAY     0x36
#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A
#define REG_CONTINUE_WRITE_RAM 0x3C
#define REG_WBRIGHT            0x51 /* Write brightness*/
#define REG_RBRIGHT            0x53 /* Read brightness*/
#define REG_PORCH_CTRL         0xB2
#define REG_FRAME_CTRL         0xB3
#define REG_GATE_CTRL          0xB7
#define REG_VCOM_SET           0xBB
#define REG_LCM_CTRL           0xC0
#define REG_SET_TIME_SRC       0xC2
#define REG_SET_DISP_MODE      0xC4
#define REG_VCOMH_OFFSET_SET   0xC5
#define REG_FR_CTRL            0xC6
#define REG_POWER_CTRL         0xD0
#define REG_PV_GAMMA_CTRL      0xE0
#define REG_NV_GAMMA_CTRL      0xE1
#define REG_SPI2EN             0xE7

/**
  * @}
  */

/** @defgroup SPD2012_Exported_Functions
  * @{
  */

void SPD2012TP_INIT(void);
void spd2012tp_reset(void);

/* LCD driver structure */

#ifdef __cplusplus
}
#endif

#endif /* __SPD2012_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

