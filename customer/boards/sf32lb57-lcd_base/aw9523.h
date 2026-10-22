/*
 * AW9523B I2C GPIO 扩展器驱动
 *
 * 本板连接: I2C1 (SCL=PA22, SDA=PA23), 7bit 地址 0x58 (AD0=AD1=GND)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AW9523_H__
#define __AW9523_H__

#include <stdint.h>

#define AW9523_I2C_NAME     "i2c1"
#define AW9523_I2C_ADDR     0x59        /* 7bit 地址, AD1=GND AD0=VBAT */

/* 寄存器 */
#define AW9523_REG_INPUT0   0x00
#define AW9523_REG_INPUT1   0x01
#define AW9523_REG_OUTPUT0  0x02
#define AW9523_REG_OUTPUT1  0x03
#define AW9523_REG_CONFIG0  0x04
#define AW9523_REG_CONFIG1  0x05
#define AW9523_REG_INT0     0x06
#define AW9523_REG_INT1     0x07
#define AW9523_REG_ID       0x10
#define AW9523_REG_GCR      0x11
#define AW9523_REG_LEDMODE0 0x12
#define AW9523_REG_LEDMODE1 0x13

/* 本板引脚分配: 0xPp, P=端口(0/1) p=位(0~7) */
#define AW9523_PIN_USB_DET      0x00    /* P0_0 输入, USB VBUS 检测 */
#define AW9523_PIN_TF_DET       0x01    /* P0_1 输入, TF 卡插拔检测 */
#define AW9523_PIN_PDM_PWREN    0x04    /* P0_4 */
#define AW9523_PIN_AU_PA_EN     0x05    /* P0_5 */
#define AW9523_PIN_DVP_PDN      0x06    /* P0_6 */
#define AW9523_PIN_HST_WAK_WF   0x07    /* P0_7 */
#define AW9523_PIN_2G4_RFSW     0x10    /* P1_0 */
#define AW9523_PIN_LED_R        0x11    /* P1_1 */
#define AW9523_PIN_LED_G        0x12    /* P1_2 */
#define AW9523_PIN_LED_B        0x13    /* P1_3 */
#define AW9523_PIN_WF_PWRKEY    0x14    /* P1_4 */
#define AW9523_PIN_WF_PWREN     0x15    /* P1_5 */
#define AW9523_PIN_CTP_RST      0x16    /* P1_6 */
#define AW9523_PIN_LCD_RST      0x17    /* P1_7 */

int aw9523_init(void);
void aw9523_set_pin(uint8_t pin, uint8_t val);
uint8_t aw9523_get_pin(uint8_t pin);

#endif /* __AW9523_H__ */
