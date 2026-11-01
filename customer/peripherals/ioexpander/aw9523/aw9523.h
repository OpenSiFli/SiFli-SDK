/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AW9523_H__
#define __AW9523_H__

#include <stdint.h>

/*
 * On this board the expander sits on I2C1 (SCL=PA22, SDA=PA23), 7-bit
 * address 0x59 (AD1=GND, AD0=VBAT). Override the macros before including
 * this header to reuse the driver on another board.
 */
#ifndef AW9523_I2C_NAME
#define AW9523_I2C_NAME     "i2c1"
#endif
#ifndef AW9523_I2C_ADDR
#define AW9523_I2C_ADDR     0x59
#endif

/* Registers */
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

int aw9523_init(void);
int aw9523_set_pin(uint8_t pin, uint8_t val);
uint8_t aw9523_get_pin(uint8_t pin);

#endif /* __AW9523_H__ */
