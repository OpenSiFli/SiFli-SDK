/**
  ******************************************************************************
  * @file   drv_nnacc.c
  * @author Sifli software development team
  * @brief Neural Network Accelerator BSP driver
  * @{
  ******************************************************************************
*/
/**
 * @attention
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

#include <board.h>

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_nnacc NN Accelerator
  * @brief Neural Network Accelerator BSP driver
  * @{
  */

#ifdef BSP_USING_NN_ACC
#include <drv_nnacc.h>
#include <drv_common.h>
#include <string.h>

#define  DBG_LEVEL            DBG_ERROR  //DBG_LOG //
#define LOG_TAG              "drv.nnacc"
#include <drv_log.h>


static NNACC_HandleTypeDef nn_acc = {.instance = hwp_nnacc};



void NNACC_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_NNACC_IRQHandler(&nn_acc);
    rt_interrupt_leave();
}


rt_err_t nn_acc_start(NNACC_ConfigTypeDef *config)
{
    HAL_StatusTypeDef ret;

    ret = HAL_NNACC_Start(&nn_acc, config);

    if (HAL_OK == ret)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}


rt_err_t nn_acc_start_IT(NNACC_ConfigTypeDef *config, nn_acc_cbk cbk)
{
    HAL_StatusTypeDef ret;

#ifdef SOC_BF0_HCPU
    NVIC_EnableIRQ(NNACC_IRQn);
#else
    NVIC_EnableIRQ(NNACC2_IRQn);
#endif

    nn_acc.CpltCallback = (void (*)(NNACC_HandleTypeDef *))cbk;
    ret = HAL_NNACC_Start_IT(&nn_acc, config);
    RT_ASSERT(HAL_OK == ret);

    if (HAL_OK == ret)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

NNACC_HandleTypeDef *drv_get_nnacc_handle()
{
    return &nn_acc;
}

int nn_acc_init(void)
{
    HAL_StatusTypeDef ret = RT_ERROR;


    ret = HAL_NNACC_Init(&nn_acc);

    if (HAL_OK == ret)
    {
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}
INIT_BOARD_EXPORT(nn_acc_init);

#endif /* BSP_USING_NN_ACC */

/// @} drv_nnacc
/// @} bsp_driver
/// @} file


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
