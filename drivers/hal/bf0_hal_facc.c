/**
  ******************************************************************************
  * @file   bf0_hal_facc.c
  * @author Sifli software development team
  * @brief   FACC HAL module driver.
  *          This file provides firmware functions to accelerate FIR/IIR filter implement (8bit/16bit).
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

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup FACC FACC
  * @brief FACC HAL module driver
  * @{
  */

#if defined(HAL_FACC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#ifdef hwp_facc2
    #define IS_VALID_INSTANCE(inst)  ((inst == hwp_facc1) || (inst == hwp_facc2))
#else
    #define IS_VALID_INSTANCE(inst)  (inst == hwp_facc1)
#endif

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Config(FACC_HandleTypeDef *facc, FACC_ConfigTypeDef *config)
{

    if (HAL_FACC_STATE_READY != facc->State)
    {
        return HAL_ERROR;
    }
    facc->config = *config;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Reset(FACC_HandleTypeDef *facc)
{
    facc->Instance->INT_STA = FACC_INT_STA_INT_STA;
    facc->Instance->VECT_CLR = FACC_VECT_CLR_VECT_CLR;
    return HAL_OK;
}

static void HAL_FACC_SetFifo(uint8_t *data, uint16_t len, volatile uint32_t *fifo_reg, uint8_t reverse, uint8_t is_8bit)
{
    int i;
    uint32_t value = 0;

    if (reverse == 0 || is_8bit)
    {
        for (i = 0; i < len; i++)
        {
            uint32_t temp;
            if (reverse)
                temp = *(data + len - i - 1);
            else
                temp = *(data + i);
            temp <<= ((i & 3) * 8);
            value |= temp;
            if ((i & 3) == 3)
            {
                //HAL_DBG_printf("Add to FIFO: %p=%08X\n", fifo_reg, value);
                *fifo_reg = value;
                value = 0;
            }
        }
    }
    else
    {
        len >>= 1;
        uint16_t *p = (uint16_t *)data;
        for (i = 0; i < len; i++)
        {
            uint32_t temp;

            temp = *(p + len - i - 1);
            temp <<= ((i & 1) * 16);
            value |= temp;
            if ((i & 1) == 1)
            {
                //HAL_DBG_printf("Add to FIFO: %p=%08X\n", fifo_reg, value);
                *fifo_reg = value;
                value = 0;
            }
        }
    }
    if (len & 3)
    {
        //HAL_DBG_printf("Add to FIFO: %p=%08X\n", fifo_reg, value);
        *fifo_reg = value;
    }

}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_SetCoeff(FACC_HandleTypeDef *facc, uint8_t *coff_b, uint16_t len_b, uint8_t *coff_a, uint16_t len_a, uint8_t reverse)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t value = 0;

    if (HAL_FACC_STATE_READY != facc->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    if ((len_b + len_a) > FACC_MAX_FIFO_SIZE)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }
    if (facc->config.mod_sel == 1 &&
            ((len_b > (FACC_MAX_FIFO_SIZE >> 1)) || (len_a > (FACC_MAX_FIFO_SIZE >> 1))))
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    HAL_FACC_Reset(facc);
    HAL_FACC_SetFifo(coff_b, len_b, &(facc->Instance->VECT_B), reverse, facc->config.fp_sel);
    if (len_a)
        HAL_FACC_SetFifo(coff_a, len_a, &(facc->Instance->VECT_A), reverse, facc->config.fp_sel);
    if (facc->config.fp_sel == 0)
    {
        len_b >>= 1;
        len_a >>= 1;
    }

    /* Set co-efficient array length*/
    if (len_a > 0)
        // IIR first a0 is always 1 and not in coeff_a, total length is len_a+1, no need to reduce 1 for 0-based.
        value = len_a;
    value <<= FACC_VECT_PARA_M_Pos;
    if (len_b > 0)
        value += len_b - 1;

    HAL_DBG_printf("Vector param=%08X\n", value);
    facc->Instance->VECT_PARA = value;

__EXIT:
    return status;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Buffer_Enable(FACC_HandleTypeDef *facc, uint8_t *buf_addr)
{
    if (!IS_VALID_INSTANCE(facc->Instance))
    {
        return HAL_ERROR;
    }
    facc->config.buf_sel = 1;
    facc->Instance->BUF_ADDR = (uint32_t)buf_addr;
    HAL_DBG_printf("BUF_ADDR=%08X\n", (uint32_t)buf_addr);
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_IRQHandler(FACC_HandleTypeDef *facc)
{
    if (!IS_VALID_INSTANCE(facc->Instance))
    {
        return HAL_ERROR;
    }

    facc->Instance->INT_STA = FACC_INT_STA_INT_STA;
    if (facc->CpltCallback)
    {
        facc->CpltCallback(facc);
    }

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Init(FACC_HandleTypeDef *facc)
{
    if (HAL_FACC_STATE_RESET != facc->State)
    {
        return HAL_ERROR;
    }
    if (!IS_VALID_INSTANCE(facc->Instance))
    {
        return HAL_ERROR;
    }
    if (facc->Instance == hwp_facc1)
    {
        HAL_RCC_ResetModule(RCC_MOD_FACC1);
    }
    else
    {
#ifdef hwp_facc2
        HAL_RCC_ResetModule(RCC_MOD_FACC2);
#endif
    }
    HAL_FACC_Reset(facc);
    facc->State = HAL_FACC_STATE_READY;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_DeInit(FACC_HandleTypeDef *facc)
{
    if (HAL_FACC_STATE_READY != facc->State)
    {
        return HAL_ERROR;
    }
    if (facc->Instance == hwp_facc1)
    {
        HAL_RCC_DisableModule(RCC_MOD_FACC1);
    }
    else
    {
#ifdef hwp_facc2
        HAL_RCC_DisableModule(RCC_MOD_FACC2);
#endif
    }
    facc->State = HAL_FACC_STATE_RESET;
    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Program(FACC_HandleTypeDef *facc, uint8_t *input, uint8_t *output, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (HAL_FACC_STATE_READY != facc->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    if (len & 3)         /* Only last block length could be non-32bit aligned */
    {
        HAL_DBG_printf("None 32-bt aligned, should be last block\n");
        facc->config.last_sel = 1;
    }
    facc->Instance->FACC_PARA = *(uint32_t *)(&facc->config);
    HAL_DBG_printf("FACC param=%08X\n", facc->Instance->FACC_PARA);
    facc->Instance->SRC_ADDR = (uint32_t)input;
    facc->Instance->DST_ADDR = (uint32_t)output;


    if (facc->config.fp_sel == 0)
        len >>= 1;
    facc->Instance->BLK_LEN = (uint32_t)len;
    HAL_DBG_printf("FACC BLK_LEN=%d\n", len);
    facc->Instance->INT_STA = FACC_INT_STA_INT_STA;         /*Clear any interrupts pending*/

__EXIT:
    return status;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Start(FACC_HandleTypeDef *facc, uint8_t *input, uint8_t *output, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_FACC_Program(facc, input, output, len);

    if (status == HAL_OK)
    {
        facc->Instance->INT_STA = FACC_INT_STA_INT_STA;         /*Clear any interrupts pending*/
        facc->Instance->INT_EN  &= ~FACC_INT_EN_INT_EN;         /*Disable interrupt*/
        facc->Instance->FACC_EN |= FACC_FACC_EN_FACC_EN;        /*Start FACC calculation*/
        while (0 == (facc->Instance->INT_STA & FACC_INT_STA_INT_STA_Msk));
    }
    return status;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FACC_Start_IT(FACC_HandleTypeDef *facc, uint8_t *input, uint8_t *output, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_FACC_Program(facc, input, output, len);

    if (status == HAL_OK)
    {
        facc->Instance->INT_EN = FACC_INT_EN_INT_EN;            /*Enable interrupt*/
        facc->Instance->FACC_EN |= FACC_FACC_EN_FACC_EN;        /*Start FACC calculation*/
    }
    return status;
}

#endif /* HAL_FACC_MODULE_ENABLED */


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
