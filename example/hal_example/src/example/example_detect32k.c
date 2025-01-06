/**
  ******************************************************************************
  * @file   example_detect32k.c
  * @author Sifli software development team
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

#include <stdlib.h>
#include <string.h>
#include "utest.h"

#ifndef LXT_DISABLE

#include "bf0_hal.h"
#include "string.h"

#define MAX_COUNT 100

__STATIC_INLINE uint32_t HAL_RC_CAL_DisableInterrupt(void)
{
    uint32_t mask;

    mask = __get_PRIMASK();
    __set_PRIMASK(1);
    return mask;
}

__STATIC_INLINE void HAL_RC_CAL_EnableInterrupt(uint32_t mask)
{
    __set_PRIMASK(mask);
}

static uint8_t g_stop = 0;
static uint8_t g_xt48_used = 0;

#ifdef SF32LB55X
uint32_t HAL_32k_CAL_update_reference_cycle_on_48M(uint8_t lp_cycle)
{
    uint32_t cur = 0;

    {
        uint32_t rcc_reg, div, div1, div2, try_times, delta;
        uint32_t int_mask;

        int_mask = HAL_RC_CAL_DisableInterrupt();
        hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HXT48_REQ;
        g_xt48_used = 1;
        HAL_RC_CAL_EnableInterrupt(int_mask);

        if (!(hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
        {
            while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
            {
                /* wait until HXT48 ready */
            }
        }
        // switch system clock to HXT48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);

        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);
        {
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_ble_mac->RCCAL_CTRL |= (0x0 << BLE_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_ble_mac->RCCAL_CTRL |= (lp_cycle << BLE_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_ble_mac->RCCAL_CTRL |= (0x1 << BLE_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            while (!(hwp_ble_mac->RCCAL_RESULT & BLE_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_ble_mac->RCCAL_RESULT & BLE_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);

            LOG_I("LPCycles: failed cur=%d delta=%d\n", cur, delta);
        }
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

#if 1
    {
        float   f_freq;
        uint64_t freq = 48000000ULL;
        freq = freq * (uint32_t)lp_cycle / cur;
        f_freq = (float)48000 * lp_cycle/(float)cur;
        LOG_I("32K Cycles: %d, freq=%d  (%.4f)\n", cur, (uint32_t)freq, f_freq);
    }
#endif

    return 48000000ULL * lp_cycle / cur; 
}

#else
uint32_t HAL_32k_CAL_update_reference_cycle_on_48M(uint8_t lp_cycle)
{
    uint32_t cur = 0;

    {
        uint32_t rcc_reg, div, div1, div2, try_times, delta;
        uint32_t int_mask;

        int_mask = HAL_RC_CAL_DisableInterrupt();
        hwp_lpsys_aon->ACR |= LPSYS_AON_ACR_HXT48_REQ;
        g_xt48_used = 1;
        HAL_RC_CAL_EnableInterrupt(int_mask);

        if (!(hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
        {
            while (0 == (hwp_lpsys_aon->ACR & LPSYS_AON_ACR_HXT48_RDY))
            {
                /* wait until HXT48 ready */
            }
        }
        // switch system clock to HXT48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);

        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);

        {
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_bt_mac->RCCAL_CTRL |= (0x0 << BT_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_bt_mac->RCCAL_CTRL |= (lp_cycle << BT_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_bt_mac->RCCAL_CTRL |= (0x1 << BT_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            HAL_Delay(1);
            while (!(hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);
        }

        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

#if 1   // For Debug only.
    {
        float   f_freq;
        uint64_t freq = 48000000ULL;
        freq = freq * (uint32_t)lp_cycle / cur;
        f_freq = (float)48000 * lp_cycle/(float)cur;
        LOG_I("32K Cycles: %d, freq=%d  (%.4f)\n", cur, (uint32_t)freq, f_freq);
    }
#endif
    return 48000000ULL * lp_cycle / cur;
}
#endif

static rt_err_t utest_tc_init(void)
{
    g_stop = 0;
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    LOG_I("Stop detect32k");
    g_stop = 1;
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    int i;
    int freq;
#ifdef SOC_SF32LB55X
    if (HAL_PMU_LXT_ENABLED())
        freq = LXT_FREQ;
    else
#elif defined(SOC_SF32LB52X)
    if (HAL_PMU_LXT_ENABLED())
        freq = RC32K_FREQ;
    else
#endif
        freq = RC10K_FREQ;
    int times = 0;
    while (g_stop == 0)
    {
        HAL_32k_CAL_update_reference_cycle_on_48M(200);
        rt_thread_mdelay(100);
        times++;
        if (times > 100)
            break;
    }
    g_stop = 0;
}

#ifndef HAL_USING_HTOL
    UTEST_TC_EXPORT(testcase, "example_cal32k", utest_tc_init, utest_tc_cleanup, 10);
#endif


#endif /*LXT_DISABLE*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
