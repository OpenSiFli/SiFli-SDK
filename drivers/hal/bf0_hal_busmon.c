/**
  ******************************************************************************
  * @file   bf0_hal_busmon.c
  * @author Sifli software development team
  * @brief Bus monitor
  * @{
  ******************************************************************************
*/
/*
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


#if defined(HAL_BUSMON_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)

#define HAL_BUSMON_MAXCHN   8

#ifdef SF32LB55X

#define HAL_BUSMON_MAXCHN_H HAL_BUSMON_MAXCHN
#define HAL_BUSMON_MAXCHN_L HAL_BUSMON_MAXCHN
#define HAL_BUSMON_HCPU     BUSMON1_BASE
#define HAL_BUSMON_LCPU     BUSMON2_BASE

const static HAL_BUSMON_FuncTypeDef Hcpu_busmon_sel[][HAL_BUSMON_MAXCHN] =
{
    {HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C},
    {HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S},
    {HAL_BUSMON_DMAC1, HAL_BUSMON_EXTDMA, HAL_BUSMON_EPIC_A, HAL_BUSMON_EPIC_B, HAL_BUSMON_LCDC1, HAL_BUSMON_EZIP, HAL_BUSMON_USBC, HAL_BUSMON_SDMMC2},
    {HAL_BUSMON_SDMMC1, HAL_BUSMON_LP2HP, HAL_BUSMON_PTC1, HAL_BUSMON_LCDC1, HAL_BUSMON_DMAC1, HAL_BUSMON_EXTDMA, HAL_BUSMON_EPIC_A, HAL_BUSMON_EPIC_B},
    {HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
    {HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
    {HAL_BUSMON_RAM0, HAL_BUSMON_RAM1, HAL_BUSMON_RAM2, HAL_BUSMON_RAM3, HAL_BUSMON_RAM4, HAL_BUSMON_RAM5, HAL_BUSMON_HP_APB, HAL_BUSMON_HP_AHB},
    {HAL_BUSMON_QSPI1, HAL_BUSMON_QSPI2, HAL_BUSMON_QSPI3, HAL_BUSMON_HP2LP, HAL_BUSMON_OPSRAM, HAL_BUSMON_QSPI1, HAL_BUSMON_QSPI2, HAL_BUSMON_NA},
};

const static HAL_BUSMON_FuncTypeDef Lcpu_busmon_sel[][HAL_BUSMON_MAXCHN] =
{
    {HAL_BUSMON_LCPU_C, HAL_BUSMON_LCPU_C, HAL_BUSMON_LCPU_C, HAL_BUSMON_LCPU_C, HAL_BUSMON_LCPU_S, HAL_BUSMON_LCPU_S, HAL_BUSMON_LCPU_S, HAL_BUSMON_LCPU_S},
    {HAL_BUSMON_DMAC2, HAL_BUSMON_LCDC2, HAL_BUSMON_HP2LP, HAL_BUSMON_PTC2, HAL_BUSMON_DMAC2, HAL_BUSMON_LCDC2, HAL_BUSMON_HP2LP, HAL_BUSMON_PTC2},
    {HAL_BUSMON_RAM0, HAL_BUSMON_RAM1, HAL_BUSMON_RAM2, HAL_BUSMON_RAM3, HAL_BUSMON_RAM4, HAL_BUSMON_RAM5, HAL_BUSMON_LP_APB, HAL_BUSMON_LP_AHB},
    {HAL_BUSMON_QSPI4, HAL_BUSMON_PHY_DUMP, HAL_BUSMON_LP2HP, HAL_BUSMON_AHBT, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
    {HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
    {HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
    {HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
    {HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA, HAL_BUSMON_NA},
};
#elif defined(SF32LB58X)

#define HAL_BUSMON_MAXCHN_H   8
#define HAL_BUSMON_MAXCHN_A   4
#define HAL_BUSMON_MAXCHN_L   4

#define HAL_BUSMON_HCPU     BUSMON1_BASE
#define HAL_BUSMON_ACPU     BUSMON2_BASE
#define HAL_BUSMON_LCPU     BUSMON3_BASE

const static HAL_BUSMON_FuncTypeDef Hcpu_busmon_sel[HAL_BUSMON_MAXSEL][HAL_BUSMON_MAXCHN] =
{
    {HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C},
    {HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S},
    {HAL_BUSMON_LP2HP,  HAL_BUSMON_JENC,    HAL_BUSMON_JDEC,    HAL_BUSMON_EXTDMA,  HAL_BUSMON_EZIP1,   HAL_BUSMON_EPIC_A,  HAL_BUSMON_NA,      HAL_BUSMON_EPIC_B},
    {HAL_BUSMON_GPU_A,  HAL_BUSMON_GPU_B,   HAL_BUSMON_USBC,    HAL_BUSMON_NNACC1_A, HAL_BUSMON_SDMMC2,  HAL_BUSMON_NNACC1_B, HAL_BUSMON_SDMMC1,  HAL_BUSMON_NNACC1_C},
    {HAL_BUSMON_AES,    HAL_BUSMON_LCDC1,   HAL_BUSMON_PTC1,    HAL_BUSMON_A2HP,    HAL_BUSMON_DMAC1,   HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_RAM0,   HAL_BUSMON_RAM1,    HAL_BUSMON_RAM2,    HAL_BUSMON_RAM3,    HAL_BUSMON_RAM4,    HAL_BUSMON_RAM5,    HAL_BUSMON_HP2A,    HAL_BUSMON_MPI1},
    {HAL_BUSMON_MPI2,   HAL_BUSMON_MPI3,    HAL_BUSMON_MPI4,    HAL_BUSMON_HP_APB,  HAL_BUSMON_HP_AHB,  HAL_BUSMON_HP2LP,   HAL_BUSMON_NA,      HAL_BUSMON_NA},
};

const static HAL_BUSMON_FuncTypeDef Acpu_busmon_sel[HAL_BUSMON_MAXSEL][HAL_BUSMON_MAXCHN] =
{

    {HAL_BUSMON_ACPU_C, HAL_BUSMON_ACPU_C,  HAL_BUSMON_ACPU_C,  HAL_BUSMON_ACPU_C},
    {HAL_BUSMON_ACPU_S, HAL_BUSMON_ACPU_S,  HAL_BUSMON_ACPU_S,  HAL_BUSMON_ACPU_S},
    {HAL_BUSMON_DMAC2,  HAL_BUSMON_HP2A,    HAL_BUSMON_FFT1,    HAL_BUSMON_FACC1},
    {HAL_BUSMON_RAM6,   HAL_BUSMON_RAM7,    HAL_BUSMON_RAM8,    HAL_BUSMON_RAM9},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_A2HP},
    // Rest of them is HAL_BUSMON_NA
};
const static HAL_BUSMON_FuncTypeDef Lcpu_busmon_sel[HAL_BUSMON_MAXSEL][HAL_BUSMON_MAXCHN] =
{
    {HAL_BUSMON_LCPU_C, HAL_BUSMON_LCPU_C,  HAL_BUSMON_LCPU_C,  HAL_BUSMON_LCPU_C},
    {HAL_BUSMON_LCPU_S, HAL_BUSMON_LCPU_S,  HAL_BUSMON_LCPU_S,  HAL_BUSMON_LCPU_S},
    {HAL_BUSMON_DMAC3,  HAL_BUSMON_FFT2,    HAL_BUSMON_NNACC2_B, HAL_BUSMON_LCDC2},
    {HAL_BUSMON_HP2LP,  HAL_BUSMON_PTC2,    HAL_BUSMON_NNACC2_A, HAL_BUSMON_FACC2},
    {HAL_BUSMON_NNACC2_C, HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_MPI5,   HAL_BUSMON_RAM0,    HAL_BUSMON_RAM1,    HAL_BUSMON_RAM2},
    {HAL_BUSMON_RAM3,   HAL_BUSMON_RAM4,    HAL_BUSMON_RAM5,    HAL_BUSMON_TCM},
    {HAL_BUSMON_LP_APB, HAL_BUSMON_LP_AHB,  HAL_BUSMON_PHY_DUMP, HAL_BUSMON_LP2HP},
};

#elif defined(SF32LB56X)
#define HAL_BUSMON_MAXCHN   8
#define HAL_BUSMON_MAXCHN_H HAL_BUSMON_MAXCHN
#define HAL_BUSMON_MAXCHN_L 4

#define HAL_BUSMON_HCPU     BUSMON1_BASE
#define HAL_BUSMON_LCPU     BUSMON2_BASE

const static HAL_BUSMON_FuncTypeDef Hcpu_busmon_sel[HAL_BUSMON_MAXSEL][HAL_BUSMON_MAXCHN] =
{

    {HAL_BUSMON_HCPU_C, HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C,  HAL_BUSMON_HCPU_C},
    {HAL_BUSMON_HCPU_S, HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S,  HAL_BUSMON_HCPU_S},
    {HAL_BUSMON_LP2HP,  HAL_BUSMON_DMAC1,   HAL_BUSMON_EXTDMA,  HAL_BUSMON_NNACC1_C, HAL_BUSMON_EZIP1,   HAL_BUSMON_EPIC_A,  HAL_BUSMON_EPIC_B,  HAL_BUSMON_LCDC1},
    {HAL_BUSMON_FACC1,  HAL_BUSMON_NNACC1_B, HAL_BUSMON_FFT1,    HAL_BUSMON_SDMMC1,  HAL_BUSMON_NNACC1_A, HAL_BUSMON_USBC,    HAL_BUSMON_AES,     HAL_BUSMON_PTC1},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_RAM0,   HAL_BUSMON_RAM1,    HAL_BUSMON_RAM2,    HAL_BUSMON_RAM3,    HAL_BUSMON_MPI1,    HAL_BUSMON_MPI2,    HAL_BUSMON_HP_APB,  HAL_BUSMON_HP_AHB},
    {HAL_BUSMON_HP2LP,  HAL_BUSMON_MPI3,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
};

const static HAL_BUSMON_FuncTypeDef Lcpu_busmon_sel[HAL_BUSMON_MAXSEL][HAL_BUSMON_MAXCHN] =
{
    {HAL_BUSMON_LCPU_C, HAL_BUSMON_LCPU_C,  HAL_BUSMON_LCPU_C,  HAL_BUSMON_LCPU_C},
    {HAL_BUSMON_LCPU_S, HAL_BUSMON_LCPU_S,  HAL_BUSMON_LCPU_S,  HAL_BUSMON_LCPU_S},
    {HAL_BUSMON_DMAC2,  HAL_BUSMON_HP2LP,   HAL_BUSMON_PTC2,    HAL_BUSMON_NA},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_NA},
    {HAL_BUSMON_MPI5,   HAL_BUSMON_RAM0,    HAL_BUSMON_RAM1,    HAL_BUSMON_RAM2},
    {HAL_BUSMON_TCM,    HAL_BUSMON_LP_APB,  HAL_BUSMON_LP_AHB,  HAL_BUSMON_PHY_DUMP},
    {HAL_BUSMON_NA,     HAL_BUSMON_NA,      HAL_BUSMON_NA,      HAL_BUSMON_LP2HP},
};
#else
#error "Not supported yet"
#endif

static int8_t find_sel(const HAL_BUSMON_FuncTypeDef busmon_sel[][HAL_BUSMON_MAXCHN], uint8_t *channel, uint8_t sel_func)
{
    int8_t r;
    int i;

    if (*channel < HAL_BUSMON_MAXCHN)
    {
        for (i = 0; i < HAL_BUSMON_MAXSEL; i++)
            if (busmon_sel[i][*channel] == sel_func)
                break;
        r = (i < HAL_BUSMON_MAXSEL) ? i : -1;
    }
    else
    {
        int j;

        for (i = 0; i < HAL_BUSMON_MAXSEL; i++)
        {
            for (j = 0; j < HAL_BUSMON_MAXCHN; j++)
                if (busmon_sel[i][j] == sel_func)
                    break;
            if (j < HAL_BUSMON_MAXCHN)
            {
                *channel = j;
                r = i;
                break;
            }
        }
    }
    return r;
}

/**
* @brief  Initialize Busmonitor
* @param  hbusmon Handle of busmonitor
* @retval HAL_OK if successful, otherwise error
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_BUSMON_Init(BUSMON_HandleTypeDef *hbusmon)
{
    uint32_t address, cr;
    int8_t  sel;

    HAL_ASSERT(hbusmon);
    HAL_ASSERT(hbusmon->Init.Channel < HAL_BUSMON_MAXCHN || hbusmon->Init.Channel == HAL_BUSMON_CHN_UNASSIGN);

    address = (uint32_t)hbusmon->Instance;
    if (address == HAL_BUSMON_HCPU)
        sel = find_sel(Hcpu_busmon_sel, &hbusmon->Init.Channel, hbusmon->Init.SelFunc);
    else if (address == HAL_BUSMON_LCPU)
        sel = find_sel(Lcpu_busmon_sel, &hbusmon->Init.Channel, hbusmon->Init.SelFunc);
#ifdef HAL_BUSMON_ACPU
    else if (address == HAL_BUSMON_ACPU)
        sel = find_sel(Acpu_busmon_sel, &hbusmon->Init.Channel, hbusmon->Init.SelFunc);
#endif
    else
        HAL_ASSERT(0);
    if (sel >= 0)
    {
        address += hbusmon->Init.Channel * sizeof(BUSMON_ChnTypeDef);
        hbusmon->Chn = (BUSMON_ChnTypeDef *)address;
        hbusmon->Chn->MAX = hbusmon->Init.Max;
        hbusmon->Chn->MIN = hbusmon->Init.Min;
        cr = hbusmon->Init.Flags;
        cr <<= BUSMON_CR1_OP_Pos;
        cr |= sel;
        if (hbusmon->Init.count > 0)
        {
            cr |= ((hbusmon->Init.count) << BUSMON_CR1_CMP_Pos);
            cr |= BUSMON_CR1_CMPEN;
        }
        hbusmon->Chn->CR = cr;
        hbusmon->State = HAL_BUSMON_STATE_READY;
        return HAL_OK;
    }
    else
        return HAL_ERROR;
}

/**
* @brief  Enable/disable Busmonitor
* @param  hbusmon Handle of busmonitor
* @retval HAL_OK if successful, otherwise error
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_BUSMON_Enable(BUSMON_HandleTypeDef *hbusmon, int enable)
{
    if (enable)
    {
        hbusmon->Instance->CCR |= (1 << hbusmon->Init.Channel);
        hbusmon->Instance->CER |= (1 << hbusmon->Init.Channel);
        hbusmon->State = HAL_BUSMON_STATE_RUNNING;
    }
    else
    {
        hbusmon->Instance->CER &= ~(1UL << hbusmon->Init.Channel);
        hbusmon->State = HAL_BUSMON_STATE_READY;
    }
    return HAL_OK;
}


/**
* @brief  Get Busmonitor count
* @param  hbusmon Handle of busmonitor
* @param  count count for bus monitor
* @retval HAL_OK if successful, otherwise error
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_BUSMON_GetCount(BUSMON_HandleTypeDef *hbusmon, int32_t *count)
{
    if (hbusmon->State > HAL_BUSMON_STATE_RESET)
    {
        *count = (hbusmon->Chn->CNT & BUSMON_CNT1_CNT_Msk);
        return (hbusmon->Chn->CNT & BUSMON_CNT1_OF) ? HAL_ERROR : HAL_OK;
    }
    else
        return HAL_ERROR;
}


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
