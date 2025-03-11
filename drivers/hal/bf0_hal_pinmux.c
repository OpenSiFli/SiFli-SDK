/**
  ******************************************************************************
  * @file   bf0_hal_pinmux.c
  * @author Sifli software development team
  * @brief   GPIO HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
 *
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "bf0_hal.h"
#include "bf0_hal_pinmux.h"
#include "bf0_pin_const.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup PIMNUX PINMUX
  * @brief PINMUX HAL module driver
  * @{
  */

#if defined(HAL_PINMUX_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#if defined(SF32LB56X) || defined(SF32LB52X)
    #define HAL_PINMUX_EXT_ENABLED
#endif /* SF32LB56X || SF32LB52X */

/**
  * @brief  Get pin base.
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval Pin Group base address.
  */
__HAL_ROM_USED volatile uint32_t *HAL_PIN_Get_Base(int hcpu)
{
    volatile uint32_t *pin;
    if (hcpu)
    {
        HAL_RCC_HCPU_enable(HPSYS_RCC_ENR1_PINMUX1, 1);
        pin = (volatile uint32_t *)PINMUX1_BASE;
    }
    else
    {
        HAL_RCC_LCPU_enable(LPSYS_RCC_ENR1_PINMUX2, 1);
        pin = (volatile uint32_t *)PINMUX2_BASE;
    }
    return pin;
}

/**
  * @brief  Select pin function only.
  * @param  pad: physical pin
  * @param  func: Pin function.
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  */
__HAL_ROM_USED void HAL_PIN_Select(int pad, int func, int hcpu)
{
    volatile uint32_t *pin = HAL_PIN_Get_Base(hcpu);
    uint32_t val;
#ifdef hwp_pbr
    if (!hcpu && (pad >= PAD_PBR0))
    {
        pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
        val = *pin;
        val &= ~(RTC_PBR0R_SEL_Msk);
        val |= MAKE_REG_VAL(func, RTC_PBR0R_SEL_Msk, RTC_PBR0R_SEL_Pos);
        *pin = val;
    }
    else
#endif /* hwp_pbr */
    {
        pin += (pad - 1);
        val = *pin;
        val &= ~(HPSYS_PINMUX_PAD_PA00_FSEL_Msk);
        val |= (func << HPSYS_PINMUX_PAD_PA00_FSEL_Pos);
        *pin = val;
    }
}

#ifdef SF32LB56X
static pin_function HAL_PIN_SetUartFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < USART1_RXD)
    {
        return PIN_FUNC_UNDEF;
    }
    group_idx = ((func - USART1_RXD) >> 2);
    if (group_idx >= 6)
    {
        return PIN_FUNC_UNDEF;
    }
    if (group_idx < 3) /* USART1~USART3 */
    {
        pinr = &hwp_hpsys_cfg->USART1_PINR;
        pinr += group_idx;
        pinmux_func = PA00_I2C_UART;
        pad_base = PAD_PA00;
    }
    else               /* USART4~USART6 */
    {
        pinr = &hwp_lpsys_cfg->USART4_PINR;
        pinr += (group_idx - 3);
        pinmux_func = PB00_I2C_UART;
        pad_base = PAD_PB00;
    }

    /* normalize to the first group */
    func -= (group_idx << 2);
    if (USART1_RXD == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_RXD_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_RXD_PIN_Pos;
        }
        else
        {

            mask = LPSYS_CFG_USART4_PINR_RXD_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_RXD_PIN_Pos;
        }
    }
    else if (USART1_TXD == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_TXD_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_TXD_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_USART4_PINR_TXD_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_TXD_PIN_Pos;
        }
    }
    else if (USART1_RTS == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_RTS_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_RTS_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_USART4_PINR_RTS_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_RTS_PIN_Pos;
        }
    }
    else if (USART1_CTS == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_CTS_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_CTS_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_USART4_PINR_CTS_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_CTS_PIN_Pos;
        }
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));

        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}


static pin_function HAL_PIN_SetI2cFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < I2C1_SCL)
    {
        return PIN_FUNC_UNDEF;
    }
    group_idx = ((func - I2C1_SCL) >> 1);
    if (group_idx >= 7)
    {
        return PIN_FUNC_UNDEF;
    }
    if (group_idx < 4)  /* I2C1~I2C4 */
    {
        pinr = &hwp_hpsys_cfg->I2C1_PINR;
        pinr += group_idx;
        pinmux_func = PA00_I2C_UART;
        pad_base = PAD_PA00;
    }
    else                /* I2C5~I2C7 */
    {
        pinr = &hwp_lpsys_cfg->I2C5_PINR;
        pinr += (group_idx - 4);
        pinmux_func = PB00_I2C_UART;
        pad_base = PAD_PB00;
    }

    /* normalize to the first group */
    func -= (group_idx << 1);
    if (I2C1_SCL == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_I2C1_PINR_SCL_PIN_Msk;
            pos = HPSYS_CFG_I2C1_PINR_SCL_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_I2C5_PINR_SCL_PIN_Msk;
            pos = LPSYS_CFG_I2C5_PINR_SCL_PIN_Pos;
        }
    }
    else if (I2C1_SDA == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_I2C1_PINR_SDA_PIN_Msk;
            pos = HPSYS_CFG_I2C1_PINR_SDA_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_I2C5_PINR_SDA_PIN_Msk;
            pos = LPSYS_CFG_I2C5_PINR_SDA_PIN_Pos;
        }
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}

static pin_function HAL_PIN_SetGptimFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    __IO uint32_t *etr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < GPTIM1_CH1)
    {
        return PIN_FUNC_UNDEF;
    }

    group_idx = ((func - GPTIM1_CH1) / 5);
    if (group_idx >= 5)
    {
        return PIN_FUNC_UNDEF;
    }

    if (group_idx < 2)  /* GPTIM1~GPTIM2 */
    {
        pinr = &hwp_hpsys_cfg->GPTIM1_PINR;
        pinr += group_idx;

        etr = &hwp_hpsys_cfg->ETR_PINR;
        pos = HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos + (group_idx * (HPSYS_CFG_ETR_PINR_ETR2_PIN_Pos - HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos));
        mask = HPSYS_CFG_ETR_PINR_ETR1_PIN_Msk << (pos - HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos);

        pinmux_func = PA00_TIM;
        pad_base = PAD_PA00;
    }
    else                /* GPTIM3~GPTIM5 */
    {
        pinr = &hwp_lpsys_cfg->GPTIM3_PINR;
        pinr += group_idx - 2;

        etr = &hwp_lpsys_cfg->ETR_PINR;
        pos = LPSYS_CFG_ETR_PINR_ETR3_PIN_Pos + ((group_idx - 2) * (LPSYS_CFG_ETR_PINR_ETR4_PIN_Pos - LPSYS_CFG_ETR_PINR_ETR3_PIN_Pos));
        mask = LPSYS_CFG_ETR_PINR_ETR3_PIN_Msk << (pos - LPSYS_CFG_ETR_PINR_ETR3_PIN_Pos);

        pinmux_func = PB00_TIM;
        pad_base = PAD_PB00;
    }

    /* normalize to the first group */
    func -= (group_idx * 5);
    if ((GPTIM1_CH1 <= func) && (GPTIM1_CH4 >= func))
    {
        if (PAD_PA00 == pad_base)
        {
            pos = HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos + (func - GPTIM1_CH1) * (HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Pos - HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos);
            mask = HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Msk << (pos - HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos);
        }
        else
        {
            pos = LPSYS_CFG_GPTIM3_PINR_CH1_PIN_Pos + (func - GPTIM1_CH1) * (LPSYS_CFG_GPTIM3_PINR_CH2_PIN_Pos - LPSYS_CFG_GPTIM3_PINR_CH1_PIN_Pos);
            mask = LPSYS_CFG_GPTIM3_PINR_CH1_PIN_Msk << (pos - LPSYS_CFG_GPTIM3_PINR_CH1_PIN_Pos);
        }
    }
    else if (GPTIM1_ETR == func)
    {
        /* do nothing, mask and pos have been set before */
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        if (GPTIM1_ETR == func)
        {
            MODIFY_REG(*etr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        }
        else
        {
            MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        }
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}

static pin_function HAL_PIN_SetLptimFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < LPTIM1_IN)
    {
        return PIN_FUNC_UNDEF;
    }
    group_idx = ((func - LPTIM1_IN) / 3);
    if ((1 == group_idx) || (group_idx >= 3))
    {
        return PIN_FUNC_UNDEF;
    }

    if (0 == group_idx)   /* LPTIM1 */
    {
        pinr = &hwp_hpsys_cfg->LPTIM1_PINR;
        pinmux_func = PA00_TIM;
        pad_base = PAD_PA00;
    }
    else                  /* LPTIM3 */
    {
        pinr = &hwp_lpsys_cfg->LPTIM3_PINR;
        pinr += group_idx - 2;
        pinmux_func = PB00_TIM;
        pad_base = PAD_PB00;
    }

    /* normalize to the first group */
    func -= (group_idx * 3);
    if (LPTIM1_IN == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_LPTIM1_PINR_IN_PIN_Msk;
            pos = HPSYS_CFG_LPTIM1_PINR_IN_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_LPTIM3_PINR_IN_PIN_Msk;
            pos = LPSYS_CFG_LPTIM3_PINR_IN_PIN_Pos;
        }
    }
    else if (LPTIM1_OUT == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Msk;
            pos = HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_LPTIM3_PINR_OUT_PIN_Msk;
            pos = LPSYS_CFG_LPTIM3_PINR_OUT_PIN_Pos;
        }
    }
    else if (LPTIM1_ETR == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Msk;
            pos = HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_LPTIM3_PINR_ETR_PIN_Msk;
            pos = LPSYS_CFG_LPTIM3_PINR_ETR_PIN_Pos;
        }
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}

static pin_function HAL_PIN_SetHpsysModuleFunc(int pad, pin_function func)
{
    if ((func >= USART1_RXD) && (func <= USART3_RTS))
    {
        func = HAL_PIN_SetUartFunc(pad, func);
    }
    else if ((func >= I2C1_SCL) && (func <= I2C4_SDA))
    {
        func = HAL_PIN_SetI2cFunc(pad, func);
    }
    else if ((func >= GPTIM1_CH1) && (func <= GPTIM2_ETR))
    {
        func = HAL_PIN_SetGptimFunc(pad, func);
    }
    else if ((func >= LPTIM1_IN) && (func <= LPTIM1_ETR))
    {
        func = HAL_PIN_SetLptimFunc(pad, func);
    }

    return func;
}

static pin_function HAL_PIN_SetLpsysModuleFunc(int pad, pin_function func)
{
    if ((func >= USART4_RXD) && (func <= USART6_RTS))
    {
        func = HAL_PIN_SetUartFunc(pad, func);
    }
    else if ((func >= I2C5_SCL) && (func <= I2C7_SDA))
    {
        func = HAL_PIN_SetI2cFunc(pad, func);
    }
    else if ((func >= GPTIM3_CH1) && (func <= GPTIM5_ETR))
    {
        func = HAL_PIN_SetGptimFunc(pad, func);
    }
    else if ((func >= LPTIM3_IN) && (func <= LPTIM3_ETR))
    {
        func = HAL_PIN_SetLptimFunc(pad, func);
    }

    return func;
}

#endif /* SF32LB56X */


#ifdef SF32LB52X
static pin_function HAL_PIN_SetUartFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < USART1_RXD)
    {
        return PIN_FUNC_UNDEF;
    }
    group_idx = ((func - USART1_RXD) >> 2);
    if (group_idx >= 5)
    {
        return PIN_FUNC_UNDEF;
    }
    if (group_idx < 3) /* USART1~USART3 */
    {
        pinr = &hwp_hpsys_cfg->USART1_PINR;
        pinr += group_idx;
        pinmux_func = PA00_I2C_UART;
        pad_base = PAD_PA00;
    }
    else               /* USART4~USART5 */
    {
        pinr = &hwp_lpsys_cfg->USART4_PINR;
        pinr += (group_idx - 3);
        pinmux_func = PB00_UART;
        pad_base = PAD_PB00;
    }

    /* normalize to the first group */
    func -= (group_idx << 2);
    if (USART1_RXD == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_RXD_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_RXD_PIN_Pos;
        }
        else
        {

            mask = LPSYS_CFG_USART4_PINR_RXD_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_RXD_PIN_Pos;
        }
    }
    else if (USART1_TXD == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_TXD_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_TXD_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_USART4_PINR_TXD_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_TXD_PIN_Pos;
        }
    }
    else if (USART1_RTS == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_RTS_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_RTS_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_USART4_PINR_RTS_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_RTS_PIN_Pos;
        }
    }
    else if (USART1_CTS == func)
    {
        if (PAD_PA00 == pad_base)
        {
            mask = HPSYS_CFG_USART1_PINR_CTS_PIN_Msk;
            pos = HPSYS_CFG_USART1_PINR_CTS_PIN_Pos;
        }
        else
        {
            mask = LPSYS_CFG_USART4_PINR_CTS_PIN_Msk;
            pos = LPSYS_CFG_USART4_PINR_CTS_PIN_Pos;
        }
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));

        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}


static pin_function HAL_PIN_SetI2cFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < I2C1_SCL)
    {
        return PIN_FUNC_UNDEF;
    }
    group_idx = ((func - I2C1_SCL) >> 1);
    /* I2C1~I2C4 */
    if (group_idx >= 4)
    {
        return PIN_FUNC_UNDEF;
    }

    pinr = &hwp_hpsys_cfg->I2C1_PINR;
    pinr += group_idx;
    pinmux_func = PA00_I2C_UART;
    pad_base = PAD_PA00;

    /* normalize to the first group */
    func -= (group_idx << 1);
    if (I2C1_SCL == func)
    {
        mask = HPSYS_CFG_I2C1_PINR_SCL_PIN_Msk;
        pos = HPSYS_CFG_I2C1_PINR_SCL_PIN_Pos;
    }
    else if (I2C1_SDA == func)
    {
        mask = HPSYS_CFG_I2C1_PINR_SDA_PIN_Msk;
        pos = HPSYS_CFG_I2C1_PINR_SDA_PIN_Pos;
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}

static pin_function HAL_PIN_SetGptimFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    __IO uint32_t *etr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < GPTIM1_CH1)
    {
        return PIN_FUNC_UNDEF;
    }

    group_idx = ((func - GPTIM1_CH1) / 5);
    /* GPTIM1~GPTIM2 */
    if (group_idx >= 2)
    {
        return PIN_FUNC_UNDEF;
    }

    pinr = &hwp_hpsys_cfg->GPTIM1_PINR;
    pinr += group_idx;

    etr = &hwp_hpsys_cfg->ETR_PINR;
    pos = HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos + (group_idx * (HPSYS_CFG_ETR_PINR_ETR2_PIN_Pos - HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos));
    mask = HPSYS_CFG_ETR_PINR_ETR1_PIN_Msk << (pos - HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos);

    pinmux_func = PA00_TIM;
    pad_base = PAD_PA00;

    /* normalize to the first group */
    func -= (group_idx * 5);
    if ((GPTIM1_CH1 <= func) && (GPTIM1_CH4 >= func))
    {
        pos = HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos + (func - GPTIM1_CH1) * (HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Pos - HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos);
        mask = HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Msk << (pos - HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos);
    }
    else if (GPTIM1_ETR == func)
    {
        /* do nothing, mask and pos have been set before */
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        if (GPTIM1_ETR == func)
        {
            MODIFY_REG(*etr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        }
        else
        {
            MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        }
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}

static pin_function HAL_PIN_SetLptimFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint8_t group_idx;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;

    if (func < LPTIM1_IN)
    {
        return PIN_FUNC_UNDEF;
    }
    group_idx = ((func - LPTIM1_IN) / 3);
    /* LPTIM1~LPTIM2 */
    if (group_idx >= 2)
    {
        return PIN_FUNC_UNDEF;
    }

    pinr = &hwp_hpsys_cfg->LPTIM1_PINR;
    pinr += group_idx;
    pinmux_func = PA00_TIM;
    pad_base = PAD_PA00;

    /* normalize to the first group */
    func -= (group_idx * 3);
    if (LPTIM1_IN == func)
    {
        mask = HPSYS_CFG_LPTIM1_PINR_IN_PIN_Msk;
        pos = HPSYS_CFG_LPTIM1_PINR_IN_PIN_Pos;
    }
    else if (LPTIM1_OUT == func)
    {
        mask = HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Msk;
        pos = HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Pos;
    }
    else if (LPTIM1_ETR == func)
    {
        mask = HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Msk;
        pos = HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Pos;
    }
    else
    {
        func = PIN_FUNC_UNDEF;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}


static pin_function HAL_PIN_SetAtimFunc(int pad, pin_function func)
{
    __IO uint32_t *pinr;
    uint32_t mask;
    uint32_t pos;
    pin_function pinmux_func;
    int pad_base;
    uint32_t func_offset;

    if ((func < ATIM1_CH1)
            || (func > ATIM1_BKIN2))
    {
        return PIN_FUNC_UNDEF;
    }

    pinmux_func = PA00_TIM;
    pad_base = PAD_PA00;

    func_offset = func - ATIM1_CH1;
    if (func < ATIM1_ETR)
    {
        if (0 == (func_offset & 1))  /* CHx */
        {
            pos = HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Pos + (func_offset >> 1) * (HPSYS_CFG_ATIM1_PINR1_CH2_PIN_Pos - HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Pos);
            mask = HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Msk << (pos - HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Pos);
            pinr = &hwp_hpsys_cfg->ATIM1_PINR1;
        }
        else  /*CHxN*/
        {
            pos = HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Pos + (func_offset >> 1) * (HPSYS_CFG_ATIM1_PINR2_CH2N_PIN_Pos - HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Pos);
            mask = HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Msk << (pos - HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Pos);
            pinr = &hwp_hpsys_cfg->ATIM1_PINR2;
        }
    }
    else
    {
        if (ATIM1_ETR == func)
        {
            pos = HPSYS_CFG_ATIM1_PINR3_ETR_PIN_Pos;
            mask = HPSYS_CFG_ATIM1_PINR3_ETR_PIN_Msk;
        }
        else if (ATIM1_BKIN == func)
        {
            pos = HPSYS_CFG_ATIM1_PINR3_BK_PIN_Pos;
            mask = HPSYS_CFG_ATIM1_PINR3_BK_PIN_Msk;
        }
        else
        {
            pos = HPSYS_CFG_ATIM1_PINR3_BK2_PIN_Pos;
            mask = HPSYS_CFG_ATIM1_PINR3_BK2_PIN_Msk;
        }
        pinr = &hwp_hpsys_cfg->ATIM1_PINR3;
    }

    if (PIN_FUNC_UNDEF != func)
    {
        MODIFY_REG(*pinr, mask, MAKE_REG_VAL(pad - pad_base, mask, pos));
        /* change function for pinmux */
        func = pad - pad_base + pinmux_func;
    }

    return func;
}


static pin_function HAL_PIN_SetHpsysModuleFunc(int pad, pin_function func)
{
    if ((func >= USART1_RXD) && (func <= USART3_RTS))
    {
        func = HAL_PIN_SetUartFunc(pad, func);
    }
    else if ((func >= I2C1_SCL) && (func <= I2C4_SDA))
    {
        func = HAL_PIN_SetI2cFunc(pad, func);
    }
    else if ((func >= GPTIM1_CH1) && (func <= GPTIM2_ETR))
    {
        func = HAL_PIN_SetGptimFunc(pad, func);
    }
    else if ((func >= LPTIM1_IN) && (func <= LPTIM2_ETR))
    {
        func = HAL_PIN_SetLptimFunc(pad, func);
    }
    else if ((func >= ATIM1_CH1) && (func <= ATIM1_BKIN2))
    {
        func = HAL_PIN_SetAtimFunc(pad, func);
    }

    return func;
}

static pin_function HAL_PIN_SetLpsysModuleFunc(int pad, pin_function func)
{
    if ((func >= USART4_RXD) && (func <= USART5_RTS))
    {
        func = HAL_PIN_SetUartFunc(pad, func);
    }

    return func;
}


#endif /* SF32LB52X  */

#ifdef SF32LB58X
static void HAL_PIN_SetAonPE(int pad, int flags, int hcpu)
{
    uint32_t val;
    int offset;
    volatile uint32_t *reg;
    uint32_t mask;

    if (hcpu)
    {
        reg = &hwp_rtc->PAWKUP;
        offset = pad - PAD_PA64;
    }
    else
    {
        reg = &hwp_rtc->PBWKUP;
        offset = pad - PAD_PB54;
    }

    if ((offset >= 0) && (offset <= 5))
    {
        val = 0;
        if (flags & HPSYS_PINMUX_PAD_PA00_PE_Msk)
        {
            val |= 1UL << (offset + RTC_PAWKUP_PE_Pos);
        }
        if (flags & HPSYS_PINMUX_PAD_PA00_PS_Msk)
        {
            val |= 1UL << (offset + RTC_PAWKUP_PS_Pos);
        }

        mask = (1UL << (offset + RTC_PAWKUP_PE_Pos)) | (1UL << (offset + RTC_PAWKUP_PS_Pos));
        MODIFY_REG(*reg, mask, val);
    }
}
#endif /* SF32LB58X */

#ifdef SF32LB56X
static void HAL_PIN_SetAonPE(int pad, int flags, int hcpu)
{
    uint32_t val;
    int offset;
    volatile uint32_t *reg;
    uint32_t mask;

    if (hcpu)
    {
        reg = &hwp_rtc->PAWKUP;
        offset = pad - PAD_PA50;
    }
    else
    {
        reg = &hwp_rtc->PBWKUP;
        offset = pad - PAD_PB32;
    }

    if ((offset >= 0) && (offset <= 4))
    {
        val = 0;
        if (flags & HPSYS_PINMUX_PAD_PA00_PE_Msk)
        {
            val |= 1UL << (offset + RTC_PAWKUP_PE_Pos);
        }
        if (flags & HPSYS_PINMUX_PAD_PA00_PS_Msk)
        {
            val |= 1UL << (offset + RTC_PAWKUP_PS_Pos);
        }

        mask = (1UL << (offset + RTC_PAWKUP_PE_Pos)) | (1UL << (offset + RTC_PAWKUP_PS_Pos));
        MODIFY_REG(*reg, mask, val);
    }
}
#endif /* SF32LB56X */

#ifdef SF32LB52X
static void HAL_PIN_SetAonPE(int pad, int flags, int hcpu)
{
    int offset;
    uint32_t mask;

    /* AON_PA28~AON_PA33 are disconnected to PAD and tied to GND, no need to change pull setting */
    if (hcpu && (pad >= PAD_PA34) && (pad <= PAD_PA44))
    {
        offset = pad - PAD_PA28;
        mask = 1UL << (offset + RTC_PAWK2R_PS_Pos);

        if (flags & HPSYS_PINMUX_PAD_PA00_PS_Msk)
        {
            MODIFY_REG(hwp_rtc->PAWK2R, mask, mask);
        }
        else
        {
            MODIFY_REG(hwp_rtc->PAWK2R, mask, 0);
        }

        mask = 1UL << (offset + RTC_PAWK1R_PE_Pos);
        if (flags & HPSYS_PINMUX_PAD_PA00_PE_Msk)
        {
            MODIFY_REG(hwp_rtc->PAWK1R, mask, mask);
        }
        else
        {
            MODIFY_REG(hwp_rtc->PAWK1R, mask, 0);
        }
    }
#ifdef hwp_pbr
    else if (hcpu && (pad >= PAD_PA24) && (pad <= PAD_PA27))
    {
        volatile uint32_t *reg;
        offset = pad - PAD_PA24;
        reg = (uint32_t *)hwp_pbr + offset;
        mask = RTC_PBR0R_PS_Msk;
        if (flags & HPSYS_PINMUX_PAD_PA00_PS_Msk)
        {
            MODIFY_REG(*reg, mask, mask);
        }
        else
        {
            MODIFY_REG(*reg, mask, 0);
        }
        mask = RTC_PBR0R_PE_Msk;
        if (flags & HPSYS_PINMUX_PAD_PA00_PE_Msk)
        {
            MODIFY_REG(*reg, mask, mask);
        }
        else
        {
            MODIFY_REG(*reg, mask, 0);
        }
    }
#endif /* hwp_pbr */
}
#endif /* SF32LB52X */


__weak int HAL_PIN_Func2Idx(int pad, pin_function func,  int hcpu)
{
    int i;
    unsigned short *func_array;

    func_array = (unsigned short *)(hcpu ? pin_pad_func_hcpu[pad] : pin_pad_func_lcpu[pad]);

    for (i = 0; i < PIN_FUNC_SEL_NUM; i++)
        if (func_array[i] == func)
            break;
    return i;
}

__weak pin_function HAL_PIN_Idx2Func(int pad, int idx,  int hcpu)
{
    pin_function r;

    if (hcpu)
        r = pin_pad_func_hcpu[pad][idx];
    else
        r = pin_pad_func_lcpu[pad][idx];

    return r;
}


/**
  * @brief  Set pin function.
  * @param  pad: physical pin
  * @param  func: Pin function.
  * @param  flags: flag of the pin (pullup/pulldown)
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, otherwise 0
  */
__HAL_ROM_USED int HAL_PIN_Set(int pad, pin_function func, int flags, int hcpu)
{
    volatile uint32_t *pin;
    int r = -1, i;
    uint32_t val;


    if (pad)
    {
        pin = HAL_PIN_Get_Base(hcpu);
        if (hcpu)
        {
#ifdef SF32LB56X
            if ((hwp_pmuc->LXT_CR & PMUC_LXT_CR_EN)
                    && ((PAD_PA55 == pad) || (PAD_PA56 == pad)))
            {
                /* XTAL32 is used , PA55 and PA56 are used by XTAL32, see HAL_PMU_EnableXTAL32 */
                return r;
            }
#endif /* SF32LB56X */

#ifdef HAL_PINMUX_EXT_ENABLED
            func = HAL_PIN_SetHpsysModuleFunc(pad, func);
#endif /* HAL_PINMUX_EXT_ENABLED */
        }
        else
        {
#ifdef HAL_PINMUX_EXT_ENABLED
            func = HAL_PIN_SetLpsysModuleFunc(pad, func);
#endif /* HAL_PINMUX_EXT_ENABLED */

        }
        i = HAL_PIN_Func2Idx(pad, func, hcpu);

        if (i < PIN_FUNC_SEL_NUM)
        {

#ifdef SF32LB55X
            if (hcpu && (PAD_PA49 == pad) && (DBG_DO8 == func))
            {
                /* to make UART1_TX work */
                MODIFY_REG(hwp_hpsys_cfg->DBGR, HPSYS_CFG_DBGR_BITEN_H_Msk,
                           MAKE_REG_VAL(1, HPSYS_CFG_DBGR_BITEN_H_Msk, HPSYS_CFG_DBGR_BITEN_H_Pos));
                MODIFY_REG(hwp_hpsys_cfg->DBGR, HPSYS_CFG_DBGR_SEL_H_Msk,
                           MAKE_REG_VAL(2, HPSYS_CFG_DBGR_SEL_H_Msk, HPSYS_CFG_DBGR_SEL_H_Pos));
            }
#endif

#ifndef SF32LB55X
            HAL_PIN_SetAonPE(pad, flags, hcpu);
#endif /* !SF32LB55X */

#ifdef hwp_pbr
            if (!hcpu && (pad >= PAD_PBR0))
            {
                pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
                val = *pin;
                val &= ~(RTC_PBR0R_SEL_Msk | RTC_PBR0R_PE_Msk | RTC_PBR0R_PS_Msk);
                if (flags & HPSYS_PINMUX_PAD_PA00_PE_Msk)
                {
                    val |= RTC_PBR0R_PE_Msk;
                }
                if (flags & HPSYS_PINMUX_PAD_PA00_PS_Msk)
                {
                    val |= RTC_PBR0R_PS_Msk;
                }
                i = MAKE_REG_VAL(i, RTC_PBR0R_SEL_Msk, RTC_PBR0R_SEL_Pos);
                *pin = val | i;
                r = 0;
            }
            else
#endif /* hwp_pbr */
            {
                val = *(pin + pad - 1);
                val &= ~(HPSYS_PINMUX_PAD_PA00_FSEL_Msk | HPSYS_PINMUX_PAD_PA00_PE_Msk
                         | HPSYS_PINMUX_PAD_PA00_PS_Msk);
                flags &= (HPSYS_PINMUX_PAD_PA00_PS_Msk | HPSYS_PINMUX_PAD_PA00_PE_Msk);
                i &= HPSYS_PINMUX_PAD_PA00_FSEL_Msk;
                /* Always enable IE.
                 * IE might be disabled before entering sleep, due to pinmux backup/restore mechanism, it would keep disabled after wakeup.
                 * Then IE could not be enabled by calling HAL_PIN_Set, HAL_PIN_SetMode needs to be called as well.
                 * To avoid such redundancy, alwasy enable IE for normal function except pin for analog function.
                 */
                *(pin + pad - 1) = val | i | flags | HPSYS_PINMUX_PAD_PA00_IE_Msk;
                r = 0;
            }
        }
    }
    return r;
}

/**
  * @brief  Set pin for analog function, fix for ROM patch, avoid pin_const update.
  * @param  pad: physical pin
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, otherwise 0
  */
// for SF32LB55X pin function is fixed to 10 for analog function, PS/PE should be 0(NOPULL).
#define PIN_ANALOG_FUNC 0xf

int HAL_PIN_Set_Analog(int pad, int hcpu)
{
    volatile uint32_t *pin;
    uint32_t val;

#ifdef hwp_pbr
    if (!hcpu && (pad >= PAD_PBR0))
    {
        pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
        val = *pin;
        val &= ~(RTC_PBR0R_SEL_Msk | RTC_PBR0R_PE_Msk | RTC_PBR0R_IE_Msk | RTC_PBR0R_OE_Msk);
        *pin = val | MAKE_REG_VAL(PIN_ANALOG_FUNC, RTC_PBR0R_SEL_Msk, RTC_PBR0R_SEL_Pos);
    }
    else
#endif /* hwp_pbr */
    {
        pin = HAL_PIN_Get_Base(hcpu);
        val = *(pin + pad - 1);
        val &= ~(HPSYS_PINMUX_PAD_PA00_FSEL_Msk | HPSYS_PINMUX_PAD_PA00_PE_Msk | HPSYS_PINMUX_PAD_PA00_IE_Msk);
        *(pin + pad - 1) = val | PIN_ANALOG_FUNC;

#ifndef SF32LB55X
        HAL_PIN_SetAonPE(pad, 0, hcpu);
#endif /* !SF32LB55X */
    }

    return 0;
}


/**
  * @brief  Update pin flags.
  * @param  pad: physical pin
  * @param  flags: Flags for the pin to update
  * @param  mask: Mask of the flags
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, otherwise 0
  */
__HAL_ROM_USED int HAL_PIN_Update(int pad, uint32_t flags, uint32_t mask, int hcpu)
{
    volatile uint32_t *pin;
    int r = -1;
    uint32_t val;

#ifdef hwp_pbr
    if (!hcpu && (pad >= PAD_PBR0))
    {
        return r;
    }
#endif /* hwp_pbr */

    if (pad)
    {
        if (hcpu)
        {
            HAL_RCC_HCPU_enable(HPSYS_RCC_ENR1_PINMUX1, 1);
            pin = (volatile uint32_t *)PINMUX1_BASE;
        }
        else
        {
            HAL_RCC_LCPU_enable(LPSYS_RCC_ENR1_PINMUX2, 1);
            pin = (volatile uint32_t *)PINMUX2_BASE;
        }
        val = *(pin + pad - 1);
        val &= ~(mask);
        val |= flags;
        *(pin + pad - 1) = val;
        r = 0;
    }
    return r;
}

/**
  * @brief  Get pin function.
  * @param  pad: physical pin
  * @param  p_func: Pointer of variable to save pin function.
  * @param  p_flags: Pointer of varibale to save pin mode
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @retval -1 if invalid, else function idx(>= 0)
  */
__HAL_ROM_USED int HAL_PIN_Get(int pad, pin_function *p_func, PIN_ModeTypeDef *p_mode, int hcpu)
{
    volatile uint32_t *pin;
    int r = -1, i;
    uint32_t val, flags;

    if (pad)
    {
        if (hcpu)
        {
            HAL_RCC_HCPU_enable(HPSYS_RCC_ENR1_PINMUX1, 1);
            pin = (volatile uint32_t *)PINMUX1_BASE;
        }
        else
        {
            HAL_RCC_LCPU_enable(LPSYS_RCC_ENR1_PINMUX2, 1);
            pin = (volatile uint32_t *)PINMUX2_BASE;
        }

#ifdef hwp_pbr
        if (!hcpu && (pad >= PAD_PBR0))
        {
            pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
            val = *pin;
            i = GET_REG_VAL(val, RTC_PBR0R_SEL_Msk, RTC_PBR0R_SEL_Pos);
            flags = 0;
            if (val & RTC_PBR0R_PE_Msk)
            {
                flags |= HPSYS_PINMUX_PAD_PA00_PE_Msk;
            }
            if (val & RTC_PBR0R_PS_Msk)
            {
                flags |= HPSYS_PINMUX_PAD_PA00_PS_Msk;
            }
            if (val & RTC_PBR0R_IE_Msk)
            {
                flags |= HPSYS_PINMUX_PAD_PA00_IE_Msk;
            }
        }
        else
#endif /* hwp_pbr */
        {
            val = *(pin + pad - 1);
            i = GET_REG_VAL(val, HPSYS_PINMUX_PAD_PA00_FSEL_Msk, HPSYS_PINMUX_PAD_PA00_FSEL_Pos);
            flags = val & (HPSYS_PINMUX_PAD_PA00_PE_Msk | HPSYS_PINMUX_PAD_PA00_PS_Msk
                           | HPSYS_PINMUX_PAD_PA00_IE_Msk);
        }

        switch (flags)
        {
        case 0:
        {
            *p_mode = PIN_ANALOG_INPUT;//or PIN_DIGITAL_O_NORMAL
            break;
        }

        case (HPSYS_PINMUX_PAD_PA00_IE):
        {
            *p_mode = PIN_DIGITAL_IO_NORMAL;
            break;
        }

        case (HPSYS_PINMUX_PAD_PA00_IE
                        | HPSYS_PINMUX_PAD_PA00_PS | HPSYS_PINMUX_PAD_PA00_PE):
            {

                *p_mode = PIN_DIGITAL_IO_PULLUP;
                break;
            }
        case (HPSYS_PINMUX_PAD_PA00_IE
                        | HPSYS_PINMUX_PAD_PA00_PE):
            {
                *p_mode = PIN_DIGITAL_IO_PULLDOWN;
                break;
            }

        case (HPSYS_PINMUX_PAD_PA00_PS | HPSYS_PINMUX_PAD_PA00_PE):
        {
            *p_mode = PIN_DIGITAL_O_PULLUP;
            break;
        }
        default:
        {
            *p_mode = 0xFF;
            break;
        }
        }


        if (i < PIN_FUNC_SEL_NUM)
        {
            *p_func = HAL_PIN_Idx2Func(pad, i, hcpu);
            r = i;
        }
        else
        {
            *p_func = PIN_FUNC_MAX;
        }

    }
    return r;
}


/**
  * @brief  Set pin DS0.
  * @param  pad physical pin
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @param  set 1: select, 0: deselect
  * @retval -1 if invalid, otherwise 0
  */
__HAL_ROM_USED int HAL_PIN_Set_DS0(int pad, int hcpu, uint8_t set)
{
    volatile uint32_t *pin;
    int r = -1;

    if (pad)
    {
        if (hcpu)
        {
            pin = (volatile uint32_t *)PINMUX1_BASE;
        }
        else
        {
            pin = (volatile uint32_t *)PINMUX2_BASE;
        }
#ifdef hwp_pbr
        if (!hcpu && (pad >= PAD_PBR0))
        {
            pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
            if (set)
            {
                *pin |= RTC_PBR0R_DS0;
            }
            else
            {
                *pin &= ~RTC_PBR0R_DS0;
            }
        }
        else
#endif /* hwp_pbr */
        {
            if (set)
            {
                *(pin + pad - 1) |= HPSYS_PINMUX_PAD_PA00_DS0;
            }
            else
            {
                *(pin + pad - 1) &= ~HPSYS_PINMUX_PAD_PA00_DS0;
            }
        }
        r = 0;
    }
    return r;
}

/**
  * @brief  Set pin DS1.
  * @param  pad physical pin
  * @param  hcpu: 1: pin for hcpu; 0: pin for lcpu
  * @param  set 1: select, 0: deselect
  * @retval -1 if invalid, otherwise 0
  */
__HAL_ROM_USED int HAL_PIN_Set_DS1(int pad, int hcpu, uint8_t set)
{
    volatile uint32_t *pin;
    int r = -1;

    if (pad)
    {
        if (hcpu)
        {
            pin = (volatile uint32_t *)PINMUX1_BASE;
        }
        else
        {
            pin = (volatile uint32_t *)PINMUX2_BASE;
        }
#ifdef hwp_pbr
        if (!hcpu && (pad >= PAD_PBR0))
        {
            pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
            if (set)
            {
                *pin |= RTC_PBR0R_DS1;
            }
            else
            {
                *pin &= ~RTC_PBR0R_DS1;
            }
        }
        else
#endif /* hwp_pbr */
        {
            if (set)
            {
                *(pin + pad - 1) |= HPSYS_PINMUX_PAD_PA00_DS1;
            }
            else
            {
                *(pin + pad - 1) &= ~HPSYS_PINMUX_PAD_PA00_DS1;
            }
        }
        r = 0;
    }
    return r;
}


__HAL_ROM_USED int HAL_PIN_SetMode(int pad, int hcpu, PIN_ModeTypeDef mode)
{
    volatile uint32_t *pin;
    int r = -1;
    uint32_t val;
    uint32_t mask;
    uint32_t reg;

    if (pad)
    {
#ifdef SF32LB56X
        if (hcpu && (hwp_pmuc->LXT_CR & PMUC_LXT_CR_EN)
                && ((PAD_PA55 == pad) || (PAD_PA56 == pad)))
        {
            /* XTAL32 is used , PA55 and PA56 are used by XTAL32, see HAL_PMU_EnableXTAL32 */
            return r;
        }
#endif /* SF32LB56X */

        if (hcpu)
        {
            pin = (volatile uint32_t *)PINMUX1_BASE;
        }
        else
        {
            pin = (volatile uint32_t *)PINMUX2_BASE;
        }


#ifdef hwp_pbr
        if (!hcpu && (pad >= PAD_PBR0))
        {
            pin = (uint32_t *)hwp_pbr + (pad - PAD_PBR0);
            reg = *pin;
            mask = (RTC_PBR0R_IE_Msk | RTC_PBR0R_PS_Msk | RTC_PBR0R_PE_Msk);

            switch (mode)
            {
            case PIN_ANALOG_INPUT:
            {
                mask |= RTC_PBR0R_OE_Msk;
                val = 0;
                break;
            }
            case PIN_DIGITAL_IO_NORMAL:
            {
                val = RTC_PBR0R_IE;
                break;
            }
            case PIN_DIGITAL_IO_PULLUP:
            {
                val = RTC_PBR0R_IE | RTC_PBR0R_PS | RTC_PBR0R_PE;
                break;
            }
            case PIN_DIGITAL_IO_PULLDOWN:
            {
                val = RTC_PBR0R_IE | RTC_PBR0R_PE;
                break;
            }
            case PIN_DIGITAL_O_NORMAL:
            {
                val = 0;
                break;
            }
            case PIN_DIGITAL_O_PULLUP:
            {
                val = RTC_PBR0R_PS | RTC_PBR0R_PE;
                break;
            }
            default:
            {
                HAL_ASSERT(0);
                break;
            }
            }

            reg &= ~mask;
            reg |= (val & mask);

            *pin = reg;
        }
        else
#endif /* hwp_pbr */
        {
            reg = *(pin + pad - 1);
            mask = (HPSYS_PINMUX_PAD_PA00_IE_Msk | HPSYS_PINMUX_PAD_PA00_PS_Msk | HPSYS_PINMUX_PAD_PA00_PE_Msk);


            switch (mode)
            {
            case PIN_ANALOG_INPUT:
            {
                val = 0;
                break;
            }
            case PIN_DIGITAL_IO_NORMAL:
            {
                val = HPSYS_PINMUX_PAD_PA00_IE;
                break;
            }
            case PIN_DIGITAL_IO_PULLUP:
            {
                val = HPSYS_PINMUX_PAD_PA00_IE
                      | HPSYS_PINMUX_PAD_PA00_PS | HPSYS_PINMUX_PAD_PA00_PE;
                break;
            }
            case PIN_DIGITAL_IO_PULLDOWN:
            {
                val = HPSYS_PINMUX_PAD_PA00_IE | HPSYS_PINMUX_PAD_PA00_PE;
                break;
            }
            case PIN_DIGITAL_O_NORMAL:
            {
                val = 0;
                break;
            }
            case PIN_DIGITAL_O_PULLUP:
            {
                val = HPSYS_PINMUX_PAD_PA00_PS | HPSYS_PINMUX_PAD_PA00_PE;
                break;
            }
            default:
            {
                HAL_ASSERT(0);
                break;
            }
            }

            reg &= ~mask;
            reg |= (val & mask);

            *(pin + pad - 1) = reg;

#ifndef SF32LB55X
            HAL_PIN_SetAonPE(pad, val, hcpu);
#endif /* !SF32LB55X */
        }
        r = 0;
    }
    return r;
}



#ifdef SF32LB55X

__HAL_ROM_USED void HAL_PIN_Set_Dual_flash1(void)
{
    HAL_PIN_Set(PAD_SIP00, QSPI1_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SIP01, QSPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP02, QSPI1_DIO6, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_SIP10, QSPI1_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SIP11, QSPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SIP12, QSPI1_DIO7, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_Set_Single_flash2(void)
{
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_Set_Dual_flash2(void)
{
    // for CS/CLK, flash2 chip 2 use the same pin with chip 1
    HAL_PIN_Set(PAD_PA47, QSPI2_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI2_DIO6, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI2_DIO7, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_Set_Dual_flash1_default(void)
{
    HAL_PIN_Set(PAD_SIP00, 0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_SIP01, 0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP02, 0, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_SIP10, 0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP11, 0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SIP12, 0, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_Set_Single_flash2_default(void)
{
    HAL_PIN_Set(PAD_PA60, GPIO_A60, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA61, GPIO_A61, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA63, GPIO_A63, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA65, GPIO_A65, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, GPIO_A66, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA68, GPIO_A68, PIN_PULLDOWN, 1);

}

__HAL_ROM_USED void HAL_PIN_Set_Dual_flash2_default(void)
{
    // for different FPGA, default pin function is different, change it later if needed
    HAL_PIN_Set(PAD_PA47, GPIO_A47, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA51, GPIO_A51, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA55, GPIO_A55, PIN_PULLDOWN, 1);
}
__HAL_ROM_USED void HAL_PIN_SetFlash3(void)
{
    HAL_PIN_Set(PAD_PA44, QSPI3_CLK, PIN_NOPULL, 1);            // QSPI3
    HAL_PIN_Set(PAD_PA45, QSPI3_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, QSPI3_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI3_DIO3, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_SetFlash4(void)
{
    HAL_PIN_Set(PAD_PB32, QSPI4_CLK, PIN_NOPULL, 0);        // QSPI4
    HAL_PIN_Set(PAD_PB33, QSPI4_CS, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB35, QSPI4_DIO0, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB36, QSPI4_DIO1, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB37, QSPI4_DIO2, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB07, QSPI4_DIO3, PIN_PULLUP, 0);

    HAL_PIN_Set(PAD_PB18, GPIO_B18, PIN_PULLUP, 0);     // QSPI4_EN
}


#elif defined(SF32LB58X)//SF32LB55X 

__HAL_ROM_USED void HAL_PIN_SetFlash4(void)
{
    HAL_PIN_Set(PAD_PA39, MPI4_CLK, PIN_NOPULL, 1);        // QSPI4
    HAL_PIN_Set(PAD_PA30, MPI4_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, MPI4_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA37, MPI4_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, MPI4_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA38, MPI4_DIO3, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_SetFlash1_WithMode(int mode)
{
    // recover all pin status first
    HAL_PIN_Set(PAD_SA00, SAIO_D0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA01, SAIO_D1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, SAIO_D2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA03, SAIO_D3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA04, SAIO_D4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA05, SAIO_D5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA06, SAIO_D6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, SAIO_D7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA08, SAIO_D8, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA09, SAIO_D9,   PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, SAIO_D10,  PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA11, SAIO_D11, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA12, SAIO_D12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA13, SAIO_D13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA14, SAIO_D14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA15, SAIO_D15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA16, SAIO_D16, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA17, SAIO_D17, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA18, SAIO_D18, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA19, SAIO_D19, PIN_PULLDOWN, 1);

    switch (mode)
    {
    case MPI1_PIN_HPSRAM:     //_MPI1_PIN_HPSRAM
        HAL_PIN_Set(PAD_SA00, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA01, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_DQS0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO8, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO9, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA13, MPI1_DIO10, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO11, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA15, MPI1_DIO12, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA16, MPI1_DIO13, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA17, MPI1_DIO14, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA18, MPI1_DIO15, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA19, MPI1_DQS1, PIN_PULLDOWN, 1);
        break;
    case MPI1_PIN_OPSRAM:     //_MPI1_PIN_OPSRAM
        HAL_PIN_Set(PAD_SA04, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA13, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA15, MPI1_DQS0, PIN_PULLDOWN, 1);
        break;
    case MPI1_PIN_P32:     //_MPI1_PIN_P32
        HAL_PIN_Set(PAD_SA01, MPI1_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_DIO0,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO3, PIN_PULLUP, 1);
        break;
    case MPI1_PIN_P64:     //_MPI1_PIN_P64
        HAL_PIN_Set(PAD_SA01, MPI1_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA13, MPI1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO0, PIN_NOPULL, 1);
        break;
    case MPI1_PIN_G32:     //_MPI1_PIN_G32
        HAL_PIN_Set(PAD_SA07, MPI1_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA13, MPI1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO0, PIN_NOPULL, 1);
        break;
    case MPI1_PIN_G64:     //_MPI1_PIN_G64
        HAL_PIN_Set(PAD_SA07, MPI1_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO0, PIN_NOPULL, 1);
        break;
    default:
        break;
    }

}

__HAL_ROM_USED void HAL_PIN_SetFlash2_WithMode(int mode)
{
    switch (mode)
    {
    case MPI2_PIN_HPSRAM:     //_MPI2_PIN_HPSRAM
        HAL_PIN_Set(PAD_SB00, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB01, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DQS0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO8, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DIO9, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB13, MPI2_DIO10, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB14, MPI2_DIO11, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB15, MPI2_DIO12, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB16, MPI2_DIO13, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB17, MPI2_DIO14, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB18, MPI2_DIO15, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB19, MPI2_DQS1, PIN_PULLDOWN, 1);
        break;
    case MPI2_PIN_OPSRAM:     //_MPI2_PIN_OPSRAM
        HAL_PIN_Set(PAD_SB04, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB13, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB14, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB15, MPI2_DQS0, PIN_PULLDOWN, 1);
        // reset others pin to default
        HAL_PIN_Set(PAD_SB00, SBIO_D0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB01, SBIO_D1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, SBIO_D2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, SBIO_D3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, SBIO_D8, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB16, SBIO_D16, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB17, SBIO_D17, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB18, SBIO_D18, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB19, SBIO_D19, PIN_PULLDOWN, 1);
        break;
    default:
        break;
    }
}

#elif defined(SF32LB56X)

__weak uint32_t BSP_Get_Sip1_Mode()
{
    return 0;
}

__weak uint32_t BSP_Get_Sip2_Mode()
{
    return 0;
}


// auto cfg sipa/sipb pinmux, need call after BSP_System_Config to avoid sip info not ready
__HAL_ROM_USED int HAL_SIP_AUTO_CFG(void)
{
    int sip1 = BSP_Get_Sip1_Mode();
    int sip2 = BSP_Get_Sip2_Mode();

    HAL_PIN_SetFlash1_WithMode(sip1);
    HAL_PIN_SetFlash2_WithMode(sip2);

    if ((sip1 == 0) && (sip2 == 0))
        return 1;

    return 0;
}

__HAL_ROM_USED void HAL_PIN_SetFlash1_WithMode(int mode)
{
    SFPIN_SIP1_MODE idx = (SFPIN_SIP1_MODE)(mode & 0xff);
    switch (idx)
    {
    case SFPIN_SIP1_APM_XCA64:
        HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);

        HAL_PIN_Set_Analog(PAD_SA00, 1);
        HAL_PIN_Set_Analog(PAD_SA05, 1);
        break;
    case SFPIN_SIP1_APM_LEG32:
        HAL_PIN_Set(PAD_SA00, MPI1_DM, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_CLKB,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DQS, PIN_PULLDOWN, 1);
        break;
    case SFPIN_SIP1_WINB_HYP64:
    case SFPIN_SIP1_WINB_HYP32:
        HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_CLKB,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);

        HAL_PIN_Set_Analog(PAD_SA00, 1);
        break;
    default:
        return;
    }

    HAL_PIN_Set_Analog(PAD_SB00, 1);
    HAL_PIN_Set_Analog(PAD_SB01, 1);
    HAL_PIN_Set_Analog(PAD_SB02, 1);
    HAL_PIN_Set_Analog(PAD_SB03, 1);
    HAL_PIN_Set_Analog(PAD_SB04, 1);
    HAL_PIN_Set_Analog(PAD_SB05, 1);
    HAL_PIN_Set_Analog(PAD_SB06, 1);
    HAL_PIN_Set_Analog(PAD_SB07, 1);
    HAL_PIN_Set_Analog(PAD_SB08, 1);
    HAL_PIN_Set_Analog(PAD_SB09, 1);
    HAL_PIN_Set_Analog(PAD_SB10, 1);
    HAL_PIN_Set_Analog(PAD_SB11, 1);
    HAL_PIN_Set_Analog(PAD_SB12, 1);
    return;

}

__HAL_ROM_USED void HAL_PIN_SetFlash2_WithMode(int mode)
{
    SFPIN_SIP2_MODE idx = (SFPIN_SIP2_MODE)(mode & 0xff);
    switch (idx)
    {
    case SFPIN_SIP2_APM_XCA128:
        HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_DQSDM, PIN_PULLDOWN, 1);

        HAL_PIN_Set_Analog(PAD_SB00, 1);
        HAL_PIN_Set_Analog(PAD_SB10, 1);
        break;
    case SFPIN_SIP2_APM_LEG32:
        HAL_PIN_Set(PAD_SB00, MPI2_DM, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_CLKB,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DQS, PIN_PULLDOWN, 1);
        break;
    default:
        return;
    }

    HAL_PIN_Set_Analog(PAD_SA00, 1);
    HAL_PIN_Set_Analog(PAD_SA01, 1);
    HAL_PIN_Set_Analog(PAD_SA02, 1);
    HAL_PIN_Set_Analog(PAD_SA03, 1);
    HAL_PIN_Set_Analog(PAD_SA04, 1);
    HAL_PIN_Set_Analog(PAD_SA05, 1);
    HAL_PIN_Set_Analog(PAD_SA06, 1);
    HAL_PIN_Set_Analog(PAD_SA07, 1);
    HAL_PIN_Set_Analog(PAD_SA08, 1);
    HAL_PIN_Set_Analog(PAD_SA09, 1);
    HAL_PIN_Set_Analog(PAD_SA10, 1);
    HAL_PIN_Set_Analog(PAD_SA11, 1);
    HAL_PIN_Set_Analog(PAD_SA12, 1);
    return;
}

__HAL_ROM_USED void HAL_PIN_SetFlash3(void)
{
    HAL_PIN_Set(PAD_PA10, MPI3_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA06, MPI3_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA09, MPI3_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA07, MPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA08, MPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA11, MPI3_DIO3, PIN_PULLUP, 1);
}

#elif defined(SF32LB52X)
__HAL_ROM_USED void HAL_PIN_Set_Dual_flash2(void)
{
}

__HAL_ROM_USED void HAL_PIN_Set_Dual_flash2_default(void)
{
}

__HAL_ROM_USED void HAL_PIN_Set_Single_flash2(void)
{
    HAL_PIN_Set(PAD_PA16, MPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA12, MPI2_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP, 1);
}

__HAL_ROM_USED void HAL_PIN_Set_Single_flash2_default(void)
{
    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLUP, 1);

}

#endif
#endif

HAL_RAM_RET_CODE_SECT(HAL_PIN_Backup,  HAL_StatusTypeDef HAL_PIN_Backup(PIN_BackupBufTypeDef *buf))
{
    uint32_t i;
    __IO uint32_t *reg;
    uint32_t *wr;

    HAL_ASSERT(buf);

#ifdef SOC_BF0_HCPU
    reg = (__IO uint32_t *)hwp_pinmux1;
#else
    reg = (__IO uint32_t *)hwp_pinmux2;
#endif /* SOC_BF0_HCPU */
    wr = &buf->pad_regs[0];
    for (i = 0; i < PIN_SAVE_PINMUX_INSTANCE_SIZE; i++)
    {
        *wr = *reg;
        wr++;
        reg++;
    }

#if defined(PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE) && (PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE > 0)
#ifdef SOC_BF0_HCPU
    reg = (__IO uint32_t *)hwp_hpsys_cfg + HPSYS_CFG_PINR_OFFSET;
#else
    reg = (__IO uint32_t *)hwp_lpsys_cfg + LPSYS_CFG_PINR_OFFSET;
#endif /* SOC_BF0_HCPU */

    wr = &buf->ext_regs[0];
    for (i = 0; i < PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE; i++)
    {
        *wr = *reg;
        wr++;
        reg++;
    }
#endif /* PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE */

    return HAL_OK;
}

HAL_RAM_RET_CODE_SECT(HAL_PIN_Restore, HAL_StatusTypeDef HAL_PIN_Restore(PIN_BackupBufTypeDef *buf))
{
    uint32_t i;
    __IO uint32_t *reg;
    uint32_t *rd;

    HAL_ASSERT(buf);

#ifdef SOC_BF0_HCPU
    reg = (__IO uint32_t *)hwp_pinmux1;
#else
    reg = (__IO uint32_t *)hwp_pinmux2;
#endif  /* SOC_BF0_HCPU */
    rd = &buf->pad_regs[0];
    for (i = 0; i < PIN_SAVE_PINMUX_INSTANCE_SIZE; i++)
    {
        *reg = *rd;
        rd++;
        reg++;
    }

#if defined(PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE) && (PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE > 0)
#ifdef SOC_BF0_HCPU
    reg = (__IO uint32_t *)hwp_hpsys_cfg + HPSYS_CFG_PINR_OFFSET;
#else
    reg = (__IO uint32_t *)hwp_lpsys_cfg + LPSYS_CFG_PINR_OFFSET;
#endif /* SOC_BF0_HCPU */

    rd = &buf->ext_regs[0];
    for (i = 0; i < PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE; i++)
    {
        *reg = *rd;
        rd++;
        reg++;
    }
#endif /* PIN_SAVE_PINMUX_EXT_INSTANCE_SIZE */

    return HAL_OK;
}

/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
