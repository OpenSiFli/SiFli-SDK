/**
  ******************************************************************************
  * @file   bf0_hal_psram.c
  * @author Sifli software development team
  * @brief   PSRAM HAL module driver.
  * @{
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

/** @defgroup PSRAM PSRAM
  * @brief PSRAM HAL module driver
  * @{
  */
#if defined(HAL_PSRAM_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
#include "bf0_hal_psram.h"

HAL_StatusTypeDef HAL_PSRAM_Init(PSRAM_HandleTypeDef *hpsram)
{
    uint32_t temp;

    if (hpsram == NULL)
        return HAL_ERROR;

    if (hpsram->State == HAL_PSRAM_STATE_READY) // initialized before.
        return HAL_OK;

    hpsram->Instance = PSRAM;
    hpsram->State = HAL_PSRAM_STATE_READY;
    hpsram->Lock = HAL_UNLOCKED;

    HAL_RCC_HCPU_reset(HPSYS_RCC_RSTR1_PSRAMC, 1);
    HAL_RCC_HCPU_reset(HPSYS_RCC_RSTR1_PSRAMC, 0);

    HAL_RCC_HCPU_enable(HPSYS_RCC_ENR1_PSRAMC, 1);

    //hpsram->Init.delay = ;

    /* setting merge from simulation */
    //set cre
    hpsram->Instance->CRE = PSRAMC_CRE_CRE;

    hpsram->Instance->PSRAM_CFG |= (0x3 << PSRAMC_PSRAM_CFG_PSRAM_DENSITY_Pos) ;

    temp = hpsram->Instance->PSRAM_CFG;
    if (hpsram->dual_psram)
    {
        MODIFY_REG(temp, PSRAMC_PSRAM_CFG_PACKAGE_TYPE_Msk,
                   PSRAMC_PSRAM_CFG_DUAL_PACKAGE_TYPE);
    }
    else
    {
        MODIFY_REG(temp, PSRAMC_PSRAM_CFG_PACKAGE_TYPE_Msk,
                   PSRAMC_PSRAM_CFG_SINGLE_PACKAGE_TYPE);
    }

    if (!hpsram->is_xccela)
    {
        temp &= ~PSRAMC_PSRAM_CFG_XCCELA_PSRAM_EN;
        hpsram->Instance->CTRL_TIME &= ~PSRAMC_CTRL_TIME_W_TCPH;
        hpsram->Instance->CTRL_TIME |= 0x6 << PSRAMC_CTRL_TIME_W_TCPH_Pos;
    }

    /* enable hold to improve performance */
    temp |= PSRAMC_PSRAM_CFG_RD_HOLD_EN;
    MODIFY_REG(temp, PSRAMC_PSRAM_CFG_TCE_MAX_LENGTH_Msk,
               MAKE_REG_VAL(0xB0, PSRAMC_PSRAM_CFG_TCE_MAX_LENGTH_Msk, PSRAMC_PSRAM_CFG_TCE_MAX_LENGTH_Pos));

    hpsram->Instance->PSRAM_CFG = temp;

    //hpsram->Instance->DELAY_FINAL_ADD |= 0x15 << PSRAMC_DELAY_FINAL_ADD_DELAY_FINAL_ADD_DQS_I_Pos;
    MODIFY_REG(hpsram->Instance->DELAY_FINAL_ADD, PSRAMC_DELAY_FINAL_ADD_DELAY_FINAL_ADD_DQS_I_Msk,
               0x17 << PSRAMC_DELAY_FINAL_ADD_DELAY_FINAL_ADD_DQS_I_Pos);
    if (!hpsram->is_xccela)
    {
        hpsram->Instance->DELAY_FINAL_ADD &= ~PSRAMC_DELAY_FINAL_ADD_DELAY_FINAL_ADD_CLK_Msk;
        hpsram->Instance->DELAY_FINAL_ADD |= 0x5 << PSRAMC_DELAY_FINAL_ADD_DELAY_FINAL_ADD_CLK_Pos;
    }

    HAL_PSRAM_MspInit(hpsram);

    //set read fifo depth trigger to 2
    hpsram->Instance->READ_CTRL |= PSRAMC_READ_CTRL_RD_START_MODE;
    hpsram->Instance->READ_CTRL &= ~PSRAMC_READ_CTRL_OPT_LENGTH_Msk;
    hpsram->Instance->READ_CTRL |= 2 << PSRAMC_READ_CTRL_OPT_LENGTH_Pos;
    hpsram->Instance->READ_CTRL &= ~PSRAMC_READ_CTRL_RD_START_NUM_Msk;
    hpsram->Instance->READ_CTRL |= 1 << PSRAMC_READ_CTRL_RD_START_NUM_Pos;

    // from psramc_tst_drv.c psramc_init setting
    if (hpsram->is_xccela)
    {
        uint32_t rdata;
        uint32_t tce_max = 0xf0;    // FPGA: 0x30; //2us * 24M /// ASIC: 0xF0; //2us * 120M //XCLK default freq shall be set to 120M
        hpsram->Instance->CTRL_TIME &= ~PSRAMC_CTRL_TIME_WL;
        hpsram->Instance->PSRAM_CFG &= ~PSRAMC_PSRAM_CFG_TCE_MAX_LENGTH;
        hpsram->Instance->PSRAM_CFG |= tce_max << PSRAMC_PSRAM_CFG_TCE_MAX_LENGTH_Pos;
        if (hpsram->wakeup)
        {
            //restore refresh rate
            hwp_psramc->MR4 = 0x00400040UL;
        }

        //change xccela psram wrap size
        rdata = hpsram->Instance->MR0 ;
        //rdata[1:0] = 'h0;
        //rdata[17:16] = 'h0;
        rdata &= 0xfffcfffc;
        if (hpsram->dual_psram)  //dual psram needs fixed latency to sync
            rdata |= 0x00200020;
        hpsram->Instance->MR0 = rdata;
        //hwp_psramc->MR0 = 0x00280028;//rdata;
        rdata = hpsram->Instance->MR8 ;
        //rdata[2:0]   = 'h3;
        //rdata[18:16] = 'h3;
        rdata &= 0xfff8fff8;
        rdata |= 0x00030003;
        hpsram->Instance->MR8 = rdata;

        //write wl code for xccela psram
        hpsram->Instance->CTRL_TIME |= (7 << PSRAMC_CTRL_TIME_WL_Pos) ;
    }
    else if (hpsram->wakeup)
    {
        /* restore refresh rate */
        hpsram->Instance->MR4 &= ~0x8UL;
    }

    //reset cre
    hpsram->Instance->CRE = 0;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PSRAM_DeInit(PSRAM_HandleTypeDef *hpsram)
{
    hpsram->State = HAL_PSRAM_STATE_RESET;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_PSRAM_Config(PSRAM_HandleTypeDef *hpsram, PSRAM_CONFIG_HAL_T *conf)
{
    if (hpsram == NULL || conf == NULL)
        return HAL_ERROR;

    hpsram->Instance->CTRL_TIME = conf->ctrl_time;
    hpsram->Instance->READ_CTRL = conf->read_ctrl;
    hpsram->Instance->DELAY_FINAL_ADD = conf->delay_final_add;
    hpsram->Instance->DQS_CTRL = conf->dqs_ctrl;
    hpsram->Instance->CLK_CTRL = conf->clk_ctrl;
    hpsram->Instance->POWER_UP = conf->power_up;
    hpsram->Instance->POWER_TIME = conf->power_time;
    hpsram->Instance->REG_TIME = conf->reg_time;
    hpsram->Instance->IMR = conf->imr;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_PSRAM_READ_CTRL(PSRAM_HandleTypeDef *hpsram, uint32_t value)
{
    if (hpsram == NULL)
        return HAL_ERROR;


    return HAL_OK;
}

HAL_StatusTypeDef HAL_PSRAM_SET_DELAY(PSRAM_HandleTypeDef *hpsram, uint32_t value)
{
    if (hpsram == NULL)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PSRAM_CHECK_POWER(PSRAM_HandleTypeDef *hpsram)
{
    if (hpsram == NULL)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PSRAM_SET_CLOCK(PSRAM_HandleTypeDef *hpsram, uint32_t value)
{
    if (hpsram == NULL)
        return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief Initialize the UART MSP.
  * @param huart UART handle.
  * @retval None
  */
__weak void HAL_PSRAM_MspInit(PSRAM_HandleTypeDef *hpsram)
{
    uint32_t state;


    //temp add for delay offset of i_dq_l

    //enable train using default setting
    //*(volatile uint32_t *) 0x40003040 |= 0x2;
    hpsram->Instance->DELAY_TRAIN |= PSRAMC_DELAY_TRAIN_TRAINING_EN;

    //enable auto_cfg
    //*(volatile uint32_t *) 0x40003040 |= 0x80000000;
    hpsram->Instance->DELAY_TRAIN |= PSRAMC_DELAY_TRAIN_AUTO_CFG;

    //wait delay train done
    //dll_state = (*(volatile uint32_t *) 0x40003044 ) & 0x10000;
    do
    {
        state = hpsram->Instance->DLL_STATE & PSRAMC_DLL_STATE_DLL_LOCKED_Msk;
    }
    while (!state);

    //disable train
    //*(volatile uint32_t *) 0x40003040 &= 0xfffffffd;
    hpsram->Instance->DELAY_TRAIN &= ~PSRAMC_DELAY_TRAIN_TRAINING_EN;

    // read phy state
    //state = *(volatile uint32_t *) 0x40003038;
    do
    {
        state = hpsram->Instance->PSRAM_FREE &  PSRAMC_PSRAM_FREE_PSRAM_FREE_Msk;
    }
    while (!state);

    if (!hpsram->wakeup)
    {
        //write to globle reset psram
        //*(volatile uint32_t *) 0x40003014 |= 0x10;
        hpsram->Instance->POWER_UP |= PSRAMC_POWER_UP_HW_POWER_PULSE;
        // read init state
        //state = *(volatile uint32_t *) 0x40003028;
        do
        {
            state = hpsram->Instance->POWER_UP & PSRAMC_POWER_UP_INIT_DONE_STATE_Msk;
        }
        while (!state);
    }

    //set psram drive stength
    hpsram->Instance->MR0 &= 0xFFFFFFFC;
    //hpsram->Instance->MR0 |= 0xFFFFFFF1;
    hpsram->Instance->MR0 |= 0x1;

    //hpsram->Instance->DEBUG_SEL = 0x1; //TODO, actually needed?
    //hpsram->Instance->DELAY_TRAIN &= ~PSRAMC_DELAY_TRAIN_AUTO_CFG ;
    //hpsram->Instance->CLK_CTRL = 0x0404;
    //hpsram->Instance->DQS_CTRL = 0x04040303;

}

#endif

/// @} PSRAM

/// @} BF0_HAL_Driver

/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
