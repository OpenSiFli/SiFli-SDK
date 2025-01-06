/**
  ******************************************************************************
  * @file   bf0_hal_nn_acc.c
  * @author Sifli software development team
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

#include <bf0_hal.h>
#include <string.h>

#if defined(HAL_NNACC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#ifdef SF32LB55X
    #define NNACC_DISABLE(p_nnacc)  HAL_RCC_DisableModule(RCC_MOD_NNACC)
    #define NNACC_ENABLE(p_nnacc)   HAL_RCC_EnableModule(RCC_MOD_NNACC)
#else
    #define NNACC_DISABLE(p_nnacc) HAL_RCC_DisableModule((hwp_nnacc1 == (p_nnacc)->instance)?RCC_MOD_NNACC1:RCC_MOD_NNACC2)
    #define NNACC_ENABLE(p_nnacc)  HAL_RCC_EnableModule((hwp_nnacc1 == (p_nnacc)->instance)?RCC_MOD_NNACC1:RCC_MOD_NNACC2)
#endif /* SF32LB55X */


#define IS_ACCROSS_1M_BOUNDARY(addr,size) ((((uint32_t)(addr))&0xFFF00000)!=((((uint32_t)(addr))+((uint32_t)(size)))&0xFFF00000))
#define IS_ACCROSS_1M_BOUNDARY_2(addr1,addr2) ((((uint32_t)(addr1))&0xFFF00000)!=(((uint32_t)(addr2))&0xFFF00000))

static HAL_StatusTypeDef check_config(NN_ACC_TypeDef *instance, NNACC_ConfigTypeDef *p_config)
{
    HAL_StatusTypeDef err;
    uint32_t nn_ker_buf_size;

#define KER_BIAS_IN_OUT_ALIGN 4

    UNUSED(instance);

    if (HAL_NNACC_MODE_CONV2D == p_config->mode)
        nn_ker_buf_size  = HAL_ALIGN(p_config->kernel_dim_x * p_config->kernel_dim_y * p_config->out_ch_num * p_config->in_ch_num, KER_BIAS_IN_OUT_ALIGN);
    else
        nn_ker_buf_size  = HAL_ALIGN(p_config->kernel_dim_x * p_config->kernel_dim_y * p_config->out_ch_num, KER_BIAS_IN_OUT_ALIGN);

    const uint32_t nn_bias_buf_size = HAL_ALIGN(p_config->out_ch_num, KER_BIAS_IN_OUT_ALIGN);
    const uint32_t nn_in_buf_size  = HAL_ALIGN(p_config->in_dim_x * p_config->in_dim_y * p_config->in_ch_num, KER_BIAS_IN_OUT_ALIGN);
    const uint32_t nn_out_buf_size = HAL_ALIGN(p_config->out_dim_x * p_config->out_dim_y * p_config->out_ch_num, KER_BIAS_IN_OUT_ALIGN);


    if (IS_ACCROSS_1M_BOUNDARY(p_config->wt, nn_ker_buf_size)
            || IS_ACCROSS_1M_BOUNDARY(p_config->input, nn_in_buf_size)
            || IS_ACCROSS_1M_BOUNDARY(p_config->output, nn_out_buf_size)
            || IS_ACCROSS_1M_BOUNDARY(p_config->bias, nn_bias_buf_size)
            || IS_ACCROSS_1M_BOUNDARY_2(p_config->input, p_config->bias) //bias should in same 1MB as input
       )
    {
        err = HAL_ERROR;
    }
    else
    {
        err = HAL_OK;
    }

    return err;
}

static HAL_StatusTypeDef setup(NN_ACC_TypeDef *instance, NNACC_ConfigTypeDef *config)
{
    uint32_t temp;
    HAL_StatusTypeDef err;

    err = check_config(instance, config);
    HAL_ASSERT(HAL_OK == err);
    if (err != HAL_OK)
        return err;


#ifdef hwp_nnacc1
    if (hwp_nnacc1 == instance)
    {
        HAL_RCC_ResetModule(RCC_MOD_NNACC1);
    }
    else
    {
        HAL_RCC_ResetModule(RCC_MOD_NNACC2);
    }
#else
    HAL_RCC_ResetModule(RCC_MOD_NNACC);
#endif

    if (HAL_NNACC_MODE_CONV2D == config->mode ||
            HAL_NNACC_MODE_CONV2D_UNSIGNED == config->mode)
    {
        if (1 == config->in_ch_num)
        {
            temp = 0;
        }
        else
        {
            temp = MAKE_REG_VAL(1, NN_ACC_CFG1_ORIENT_Msk, NN_ACC_CFG1_ORIENT_Pos);
        }
    }
    else if (HAL_NNACC_MODE_DEPTHWISE_CONV2D == config->mode ||
             HAL_NNACC_MODE_DEPTHWISE_CONV2D_UNSIGNED == config->mode)
    {
        temp = MAKE_REG_VAL(2, NN_ACC_CFG1_ORIENT_Msk, NN_ACC_CFG1_ORIENT_Pos);
#ifdef SF32LB55X
        if (HAL_NNACC_MODE_DEPTHWISE_CONV2D_UNSIGNED == config->mode)
            temp = MAKE_REG_VAL(3, NN_ACC_CFG1_ORIENT_Msk, NN_ACC_CFG1_ORIENT_Pos);
#endif
    }
    temp |= MAKE_REG_VAL(config->kernel_dim_x, NN_ACC_CFG1_KERW_Msk, NN_ACC_CFG1_KERW_Pos)
            | MAKE_REG_VAL(config->kernel_dim_y, NN_ACC_CFG1_KERH_Msk, NN_ACC_CFG1_KERH_Pos)
            | MAKE_REG_VAL(config->out_dim_x, NN_ACC_CFG1_OFW_Msk, NN_ACC_CFG1_OFW_Pos)
            | MAKE_REG_VAL(config->out_dim_y, NN_ACC_CFG1_OFH_Msk, NN_ACC_CFG1_OFH_Pos)
            | MAKE_REG_VAL(config->stride_x, NN_ACC_CFG1_STRIDEX_Msk, NN_ACC_CFG1_STRIDEX_Pos)
            | MAKE_REG_VAL(config->stride_y, NN_ACC_CFG1_STRIDEY_Msk, NN_ACC_CFG1_STRIDEY_Pos);

    instance->CFG1 = temp;

    temp = MAKE_REG_VAL(config->in_dim_x, NN_ACC_CFG2_IFW_Msk, NN_ACC_CFG2_IFW_Pos)
           | MAKE_REG_VAL(config->in_dim_y, NN_ACC_CFG2_IFH_Msk, NN_ACC_CFG2_IFH_Pos)
           | MAKE_REG_VAL(config->out_ch_num, NN_ACC_CFG2_OFCH_NUM_Msk, NN_ACC_CFG2_OFCH_NUM_Pos)
           | MAKE_REG_VAL(config->in_ch_num, NN_ACC_CFG2_IFCH_NUM_Msk, NN_ACC_CFG2_IFCH_NUM_Pos);

    instance->CFG2 = temp;

    temp = MAKE_REG_VAL(config->kernel_dim_x * config->kernel_dim_y, NN_ACC_CFG7_KERNUM_Msk, NN_ACC_CFG7_KERNUM_Pos)
           | MAKE_REG_VAL(config->padding_x, NN_ACC_CFG7_PADX_Msk, NN_ACC_CFG7_PADX_Pos)
           | MAKE_REG_VAL(config->padding_y, NN_ACC_CFG7_PADY_Msk, NN_ACC_CFG7_PADY_Pos)
           | MAKE_REG_VAL(config->out_shift, NN_ACC_CFG7_OUTSHIFT_Msk, NN_ACC_CFG7_OUTSHIFT_Pos)
           | MAKE_REG_VAL(config->bias_shift, NN_ACC_CFG7_BIASSHIFT_Msk, NN_ACC_CFG7_BIASSHIFT_Pos);

    instance->CFG7 = temp;

    instance->CFG3 = (uint32_t)config->wt;
    instance->CFG4 = (uint32_t)config->input;
    instance->CFG5 = (uint32_t)config->output;
    instance->CFG6 = (uint32_t)config->bias;

#ifndef SF32LB55X
    if (HAL_NNACC_MODE_DEPTHWISE_CONV2D_UNSIGNED == config->mode ||
            HAL_NNACC_MODE_CONV2D_UNSIGNED == config->mode)
    {
        instance->CFG8 = NN_ACC_CFG8_UNSIGNED_KER | NN_ACC_CFG8_UNSIGNED_IN | NN_ACC_CFG8_UNSIGNED_OUT;
    }
#endif

    return err;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_NNACC_Init(NNACC_HandleTypeDef *nn_acc)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (nn_acc)
    {
        nn_acc->state = HAL_NNACC_STATE_READY;
        ret = HAL_OK;
    }

    return ret;

}

__HAL_ROM_USED HAL_StatusTypeDef HAL_NNACC_Start_IT(NNACC_HandleTypeDef *nn_acc, NNACC_ConfigTypeDef *config)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (!nn_acc || (nn_acc->state != HAL_NNACC_STATE_READY)) return HAL_ERROR;

    NNACC_ENABLE(nn_acc);

#ifdef PSRAM_CACHE_WB
    SCB_CleanDCache();
#endif

    nn_acc->state = HAL_NNACC_STATE_BUSY;

    ret = setup(nn_acc->instance, config);


    if (HAL_OK == ret)
    {
        nn_acc->instance->IRQ |= NN_ACC_IRQ_IMR;
        nn_acc->instance->CR1  |= NN_ACC_CR1_START;
    }
    else
    {
        NNACC_DISABLE(nn_acc);
    }

    return ret;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_NNACC_Start(NNACC_HandleTypeDef *nn_acc, NNACC_ConfigTypeDef *config)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (!nn_acc || (nn_acc->state != HAL_NNACC_STATE_READY)) return HAL_ERROR;

    NNACC_ENABLE(nn_acc);

#ifdef PSRAM_CACHE_WB
    SCB_CleanDCache();
#endif

    nn_acc->state = HAL_NNACC_STATE_BUSY;

    ret = setup(nn_acc->instance, config);

    if (HAL_OK == ret)
    {
        nn_acc->instance->IRQ &= ~NN_ACC_IRQ_IMR;
        nn_acc->instance->CR1  |= NN_ACC_CR1_START;

        while (0 == (nn_acc->instance->IRQ & NN_ACC_IRQ_IRSR))
        {
            /* wait for complete */
        }

        nn_acc->instance->IRQ |= NN_ACC_IRQ_ICR;
        nn_acc->instance->IRQ &= ~NN_ACC_IRQ_ICR;


        nn_acc->state = HAL_NNACC_STATE_READY;
        ret = HAL_OK;
    }

    NNACC_DISABLE(nn_acc);

    return ret;
}


__HAL_ROM_USED void HAL_NNACC_IRQHandler(NNACC_HandleTypeDef *nn_acc)
{
    nn_acc->instance->IRQ &= ~NN_ACC_IRQ_IMR;
    nn_acc->instance->IRQ |= NN_ACC_IRQ_ICR;
    nn_acc->instance->IRQ &= ~NN_ACC_IRQ_ICR;

    NNACC_DISABLE(nn_acc);

    nn_acc->state = HAL_NNACC_STATE_READY;

    if (nn_acc->CpltCallback)
    {
        void (* cb)(NNACC_HandleTypeDef *);

        cb = nn_acc->CpltCallback;
        cb(nn_acc);
    }

}





#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
