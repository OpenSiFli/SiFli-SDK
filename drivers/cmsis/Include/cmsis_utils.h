/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CMSIS_UTILS_H
#define __CMSIS_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAKE_REG_VAL(val, mask, offset)                                        \
    ((((uint32_t)(val)) << (offset)) & (mask))

#define MAKE_REG_VAL2(val, bits_name)                                          \
    ((((uint32_t)(val)) << (bits_name##_Pos)) & (bits_name##_Msk))

#define GET_REG_VAL(reg, mask, offset)                                         \
    ((((uint32_t)(reg)) & (mask)) >> (offset))

#define GET_REG_VAL2(reg, bits_name)                                           \
    ((((uint32_t)(reg)) & (bits_name##_Msk)) >> (bits_name##_Pos))

#define SET_BIT(REG, BIT) ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT) ((REG) & (BIT))

#define CLEAR_REG(REG) ((REG) = (0x0))

#define WRITE_REG(REG, VAL) ((REG) = (VAL))

#define READ_REG(REG) ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)                                    \
    WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#ifdef __cplusplus
}
#endif

#endif /* __CMSIS_UTILS_H */
