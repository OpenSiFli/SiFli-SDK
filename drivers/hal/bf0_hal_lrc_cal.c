/**
  ******************************************************************************
  * @file   bf0_hal_lrc_cal.c
  * @author Sifli software development team
  * @brief Lower power RC clock calibration.
  * @{
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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
#include "string.h"
#include "bf0_hal_rtc.h"

#define LRC_CAL_RETRY_OFFSET 5000
#define LRC_CAL_JUDGE_COUNT 1
#define LRC_CAL_COUNT_AVE_LIMIT 5
#define LRC_CAL_THRESHOLD_UPPER 600
#define LRC_CAL_THRESHOLD_RANGE 200
#define LRC_CAL_MAX_FILTER_WIN 32
#define LRC_CAL_DEFAULT_NOISE_THD0 250
#define LRC_CAL_DEFAULT_NOISE_THD1 200
#define LRC_CAL_DEFAULT_TEMP_THD0 2
#define LRC_CAL_DEFAULT_TEMP_THD1 4
#define LRC_CAL_LPCYCE 200
#define LRC_CAL_LPCYCE_EX 20


__HAL_ROM_USED uint32_t HAL_RC_CAL_get_reference_cycle_on_48M(void)
{
#if 1 //ndef SF32LB52X HAL_LCPU_CONFIG_LPCYCLE_CURR  rom patch used
    uint32_t cycle_cur = HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
#else
    uint32_t cycle_cur;
    uint16_t len = 4;
    HAL_StatusTypeDef ret = HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_LPCYCLE_CURR, (uint8_t *)&cycle_cur, &len);
    HAL_ASSERT(ret == HAL_OK);
#endif

    return cycle_cur;
}

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
uint8_t g_xt48_used = 0;

#ifdef SF32LB55X
//extern void rt_kprintf(const char *fmt, ...);
uint8_t HAL_RC_CAL_GetLPCycle(void)
{
    return LRC_CAL_LPCYCE;
}

__HAL_ROM_USED int HAL_RC_CAL_update_reference_cycle_on_48M_ex(uint8_t lp_cycle, int clear_ave, int ave_window)
{
    static uint32_t count = 0;
    static uint64_t value = 0;
    uint32_t cur = 0;


    if ((hwp_pmuc->CR & PMUC_CR_SEL_LPCLK) != 0)
    {
        return -1;
    }

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
        for (try_times = 0; try_times < 10; try_times++)
        {
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_ble_mac->RCCAL_CTRL |= (0x0 << BLE_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_ble_mac->RCCAL_CTRL &= ~BLE_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_ble_mac->RCCAL_CTRL |= (lp_cycle << BLE_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_ble_mac->RCCAL_CTRL |= (0x1 << BLE_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            while (!(hwp_ble_mac->RCCAL_RESULT & BLE_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_ble_mac->RCCAL_RESULT & BLE_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);
            if (cur > 1000000)
                delta = cur - 1000000;
            else
                delta = 1000000 - cur;
            if (delta < 300000)
                break;
            //rt_kprintf("LPCycles: failed cur=%d delta=%d\n", cur, delta);
        }

        HAL_Set_backup(RTC_BACKUP_LPCYCLE_CUR, cur);
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

    if (clear_ave)
    {
        count = 0;
        value = 0;
    }
    if (count < ave_window)
        count++;
    if (count)
    {
        if (count < ave_window)
        {
            value += HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        else
        {
            value = (uint64_t)HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
            value = value * (ave_window - 1) + HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        HAL_Set_backup(RTC_BACKUP_LPCYCLE_AVE, (uint32_t)(value / count));

#if 0   // For Debug only.
        {
            extern void rt_kprintf(const char *fmt, ...);
            rt_kprintf("LPCycles: %d, ave=%d, count=%d\n", HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR), (uint32_t)(value / count), count);
        }
#endif
    }

    return 0;
}

#else

#ifdef LRC_OLD_ALGO

#define LRC_CAL_RETRY_OFFSET 5000
#define LRC_CAL_JUDGE_COUNT 1
#define LRC_CAL_COUNT_AVE_LIMIT 5
#define LRC_CAL_THRESHOLD_UPPER 600
#define LRC_CAL_THRESHOLD_RANGE 200
__HAL_ROM_USED int HAL_RC_CAL_update_reference_cycle_on_48M_ex(uint8_t lp_cycle, int clear_ave, int ave_window)
{
    // TODO: Implement this for PRO
    static uint32_t count = 0;
    static uint64_t value = 0;
    uint32_t cur = 0;
    uint8_t cal_ok = 1;
    uint32_t cal_ave = 0;
    // Whether need reduce interval, 0: no need; 1: Suggest; 2: Must
    int cal_lvl = 0;
    static uint8_t cal_err = 0;


    if (HAL_RTC_LXT_ENABLED())
    {
        return -1;
    }

#ifdef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_Delay(5);
#endif
    {
        uint32_t rcc_reg, div, div1, div2, try_times, delta = 0;
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
        HAL_Delay(10);
        // switch system clock to HXT48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);

        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);
        for (try_times = 0; try_times < 10; try_times++)
        {

#ifdef SF32LB52X
            MODIFY_REG(hwp_bt_mac->RCCAL_CTRL, BT_MAC_RCCAL_CTRL_CON_NUM_Msk, 1 << BT_MAC_RCCAL_CTRL_CON_NUM_Pos);
            hwp_bt_mac->RCCAL_CTRL |= BT_MAC_RCCAL_CTRL_CON_MODE;
#endif

            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_bt_mac->RCCAL_CTRL |= (0x0 << BT_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_bt_mac->RCCAL_CTRL |= (lp_cycle << BT_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_bt_mac->RCCAL_CTRL |= (0x1 << BT_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            HAL_Delay(1);
            while (!(hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);

            if (count >= LRC_CAL_JUDGE_COUNT)
            {
                cal_ave = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
                delta = cur > cal_ave ? (cur - cal_ave) : (cal_ave - cur);

                if (delta < LRC_CAL_RETRY_OFFSET)
                    break;
            }
            else
            {
                break;
            }
        }
        HAL_DBG_printf("LPCycles: cur=%d delta=%d,try_times=%d\n", cur, delta, (try_times + 1));

        if (try_times == 10)
        {
            cal_ok = 0;
            cal_err++;
            if (cal_err == 4)
            {
                count = 0;
                value = 0;
            }
        }
        else
        {
            uint32_t delta_limit = LRC_CAL_THRESHOLD_UPPER - LRC_CAL_THRESHOLD_RANGE * count / ave_window;
            HAL_DBG_printf("Delta limit: %d\n", delta_limit);
            cal_err = 0;
            HAL_Set_backup(RTC_BACKUP_LPCYCLE_CUR, cur);
            if (delta > delta_limit)
                clear_ave = 1;
        }
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

    if (0 == cal_ok)
    {
#ifdef SF32LB52X
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
        return -2;
    }

    if (clear_ave)
    {
        count = 0;
        value = 0;
        cal_lvl = 2;
    }
    else if (count <= (LRC_CAL_COUNT_AVE_LIMIT - 1))
        cal_lvl = 1;

    if (count < ave_window)
        count++;
    if (count)
    {
        if (count < ave_window)
        {
            value += HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        else
        {
            value = (uint64_t)HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
            value = value * (ave_window - 1) + HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
        }
        HAL_Set_backup(RTC_BACKUP_LPCYCLE_AVE, (uint32_t)((value + (count >> 1)) / count));
#ifdef SF32LB52X
        {
            uint32_t set_value = (uint32_t)((value + (count >> 1)) / count);;
            float lxtfreq;

            lxtfreq = (48000000UL / (float)set_value * lp_cycle);
            HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_AVE, &set_value, 4);
            HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_CURR, &lxtfreq, 4);
        }
#endif
        HAL_DBG_printf("LPCycles: %d, ave=%d, count=%d\n", HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR), HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE), count);
    }
#ifdef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
    return cal_lvl;
}

#else



#ifdef HAL_LRC_CONFIG
    #if !defined(SF32LB56X) && !defined(SF32LB58X) && !defined(SF32LB55X)
        static uint16_t g_filter_noise_thd0 = LRC_CAL_DEFAULT_NOISE_THD0;
        static uint16_t g_filter_noise_thd1 = LRC_CAL_DEFAULT_NOISE_THD1;
        static uint8_t g_lp_cycle = LRC_CAL_LPCYCE;
    #endif
#endif

static uint8_t g_lp_cycle_ex = LRC_CAL_LPCYCE;


uint8_t HAL_RC_CAL_GetLPCycle(void)
{
    uint8_t lp_cycle = LRC_CAL_LPCYCE;
#ifdef HAL_LRC_CONFIG
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
    lp_cycle = HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA) & 0xFF;
    if (lp_cycle == 0)
        lp_cycle = LRC_CAL_LPCYCE;
#else // !(defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X))
#if defined(SOC_BF0_HCPU)
    lp_cycle = g_lp_cycle;
#else // !(SOC_BF0_HCPU)
    lp_cycle = *(uint32_t *)(0x2040fddc);
#endif // SOC_BF0_HCPU
#endif // defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
#endif // HAL_LRC_CONFIG
    return lp_cycle;
}


uint8_t HAL_RC_CAL_GetLPCycle_ex(void)
{
    return g_lp_cycle_ex;
}

int HAL_RC_CAL_SetLPCycle_ex(uint8_t lpcycle)
{
    int ret = -1;
    if (lpcycle)
    {
        g_lp_cycle_ex = lpcycle;
        ret = 0;
    }

    return ret;
}

static uint16_t HAL_RC_get_fitler_thd0(void)
{
    uint16_t thd0 = LRC_CAL_DEFAULT_NOISE_THD0;

#ifdef HAL_LRC_CONFIG
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
    thd0 = (HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA) >> 12) & 0x3FF;
    if (thd0 == 0)
        thd0 = LRC_CAL_DEFAULT_NOISE_THD0;
#else
    thd0 = g_filter_noise_thd0;
#endif
#endif // HAL_LRC_CONFIG

    return thd0;
}

static uint16_t HAL_RC_get_fitler_thd1(void)
{
    uint16_t thd1 = LRC_CAL_DEFAULT_NOISE_THD1;

#ifdef HAL_LRC_CONFIG
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
    thd1 = (HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA) >> 22) & 0x3FF;
    if (thd1 == 0)
        thd1 = LRC_CAL_DEFAULT_NOISE_THD1;
#else
    thd1 = g_filter_noise_thd1;
#endif
#endif // HAL_LRC_CONFIG

    return thd1;

}


#if 0
int HAL_RC_CAL_GetParameter(uint8_t *lp_cycle, uint16_t *thd0, uint16_t *thd1)
{

    if (!(lp_cycle || thd0 || thd1))
        return -1;

    if (lp_cycle)
    {
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
        *lp_cycle = HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA) & 0xFF;
        if (*lp_cycle == 0)
            *lp_cycle = LRC_CAL_LPCYCE;
#else
        *lp_cycle = g_lp_cycle;
#endif
    }

    if (thd0)
    {
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
        *thd0 = (HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA) >> 11) & 0x3FF;
        if (*thd0 == 0)
            *thd0 = LRC_CAL_DEFAULT_NOISE_THD0;
#else
        *thd0 = g_filter_noise_thd0;
#endif
    }

    if (thd1)
    {
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
        *thd1 = (HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA) >> 21) & 0x3FF;
        if (*thd1 == 0)
            *thd1 = LRC_CAL_DEFAULT_NOISE_THD0;
#else
        *thd1 = g_filter_noise_thd1;
#endif
    }

    return 0;
}
#endif

int HAL_RC_CAL_SetParameter(uint8_t *lp_cycle, uint16_t *thd0, uint16_t *thd1)
{
#if HAL_LRC_CONFIG
    if (!(lp_cycle || thd0 || thd1))
        return -1;

#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
    uint32_t algo_param = HAL_Get_backup(RTC_BACKUP_RC_ALGO_PARA);
#endif

    if (lp_cycle)
    {
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
        algo_param &= ~(0xFF);
        algo_param |= (*lp_cycle) & 0xFF;
#else
#if defined(SOC_BF0_HCPU)
        g_lp_cycle = *lp_cycle;
        *(uint32_t *)(0x2040fddc) = *lp_cycle;
#endif

#endif
    }

    if (thd0)
    {
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
        algo_param &= ~(0x3FF << 12);
        algo_param |= (*thd0 & 0x3FF) << 12;
#else
        g_filter_noise_thd0 = *thd0;
#endif
    }

    if (thd1)
    {
#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
        algo_param &= ~(0x3FF << 22);
        algo_param |= (*thd1 & 0x3FF) << 22;
#else
        g_filter_noise_thd1 = *thd1;
#endif
    }

#if defined(SF32LB56X) || defined(SF32LB58X) || defined(SF32LB55X)
    HAL_Set_backup(RTC_BACKUP_RC_ALGO_PARA, algo_param);
#endif

    return 0;
#else // !HAL_LRC_CONFIG
    return -2;
#endif
}





// return 1 only b -a > val
#define RC_COMPARE_LESS(a, b, val) (a < b ? ((b - a) > val ? 1 : 0) : 0)
// return 1 only a -b > val
#define RC_COMPARE_MORE(a, b, val) (a > b ? ((a - b) > val ? 1 : 0) : 0)

static uint8_t HAL_RC_CAL_get_filter_scale(uint8_t lp_cycle)
{
    uint8_t scale = 1;
    switch (lp_cycle)
    {
    case 100:
        scale = 2;
        break;
    case 50:
        scale = 4;
        break;
    case 25:
        scale = 8;
        break;
    case 10:
        scale = 20;
        break;
    }

    return scale;
}

#ifdef HAL_LRC_TESN_TEST
    #if defined(SF32LB56X)
        TSEN_HandleTypeDef g_tsen_handle;
        static uint8_t g_is_tsen_init;
    #endif
#endif

#ifndef SF32LB52X
    static uint8_t g_rc_locked;
#endif

static int8_t HAL_RC_locked(void)
{
    int8_t ret = 0;
#ifndef SF32LB52X
    if (g_rc_locked)
        ret = -1;
    else
        g_rc_locked = 1;
#else // SF32LB52X

    MUTEX_HandleTypeDef hmutex;
#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif // SOC_BF0_HCPU
    hmutex.Instance = HAL_MAILBOX_GetMutex(CORE_ID_LCPU, 0);

    MUTEX_LockCoreIdTypeDef core = HAL_MAILBOX_Lock(&hmutex, 0);

#ifdef SOC_BF0_HCPU
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif // SOC_BF0_HCPU

    //HAL_MAILBOX_UnLock(&handle, mb->channel);
    if (core != MUTEX_UNLOCKED)
        ret = -1;
#endif // !SF32LB52X

    return ret;
}

static void HAL_RC_unlocked(void)
{
#ifndef SF32LB52X
    g_rc_locked = 0;
#else // SF32LB52X
    MUTEX_HandleTypeDef hmutex;
#ifdef SOC_BF0_HCPU
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif // SOC_BF0_HCPU
    hmutex.Instance = HAL_MAILBOX_GetMutex(CORE_ID_LCPU, 0);

    HAL_MAILBOX_UnLock(&hmutex, 0);

#ifdef SOC_BF0_HCPU
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif // SOC_BF0_HCPU

#endif // !SF32LB52X
}


int HAL_RC_CALget_curr_cycle_on_48M(uint8_t lp_cycle, uint32_t *count)
{
    uint32_t cur = 0, cal_ave;
    uint8_t alpha = 0;

    static uint32_t cur_ave = 0, cur_backup = 0;



    if (HAL_LXT_ENABLED())
    {
        return -1;
    }

    {

#if !defined(SF32LB52X)
        if (HAL_RCC_LCPU_GetClockSrc(RCC_CLK_MOD_SYS) != 1)
        {
            return -2;
        }

        if (0 != GET_REG_VAL(hwp_lpsys_aon->SLP_CTRL, LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk, LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos))
            return -4;
#endif

        if (HAL_RC_locked() != 0)
            return -3;

#if !defined(SF32LB52X)
        uint32_t rcc_reg, div, div1, div2;
        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);
#endif

#ifdef SF32LB52X
        MODIFY_REG(hwp_bt_mac->RCCAL_CTRL, BT_MAC_RCCAL_CTRL_CON_NUM_Msk, 1 << BT_MAC_RCCAL_CTRL_CON_NUM_Pos);
        hwp_bt_mac->RCCAL_CTRL |= BT_MAC_RCCAL_CTRL_CON_MODE;
#endif


        hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
        hwp_bt_mac->RCCAL_CTRL |= (0x0 << BT_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
        hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_LENGTH;
        hwp_bt_mac->RCCAL_CTRL |= (lp_cycle << BT_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
        hwp_bt_mac->RCCAL_CTRL |= (0x1 << BT_MAC_RCCAL_CTRL_RCCAL_START_Pos);
        HAL_Delay(1);
        while (!(hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
        cur = (hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);

#if !defined(SF32LB52X)
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
#endif
    }

    //uint8_t scale = HAL_RC_CAL_get_filter_scale(lp_cycle);
    uint8_t scale = 1;
    cur *= scale;

    uint16_t thd0 = HAL_RC_get_fitler_thd0();
    uint16_t thd1 = HAL_RC_get_fitler_thd1();


    if (cur_backup == 0)
        alpha = 0;
    else
    {
        if ((RC_COMPARE_MORE(cur, cur_ave, thd0) && RC_COMPARE_MORE(cur_backup, cur_ave, thd0)) ||
                (RC_COMPARE_LESS(cur, cur_ave, thd1) && RC_COMPARE_LESS(cur_backup, cur_ave, thd1)))
            alpha = 2;
        else
            alpha = 1;
    }

    if (alpha == 0)
        cur_ave = cur;
    else if (alpha == 1)
    {
        cur_ave = (cur_ave * 15 + cur + 8) >> 4;
        //cur_ave = (cur_ave * 31 + cur + 16) >> 5;
        //cur_ave = (cur_ave * 7 + cur + 4) >> 3;
    }
    else
        cur_ave = (cur_ave * 3 + cur * 13 + 8) >> 4;

#if !defined(SF32LB52X)
    HAL_DBG_printf("LPCycles cur=%d , ave %d, cur_backup=%d delta %d\n", cur, cur_ave, cur_backup, (int)(cur_ave - cur));
#endif

    cur_backup = cur;
    cal_ave = cur_ave / scale;

#ifdef SF32LB52X
    {
        uint32_t set_value = cal_ave;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_AVE, &set_value, 4);
    }
#endif

    *count = cal_ave;
    HAL_RC_unlocked();
    return 0;
}


__HAL_ROM_USED int HAL_RC_CAL_update_reference_cycle_on_48M_ex(uint8_t lp_cycle, int clear_ave, int ave_window)
{
    // TODO: Implement this for PRO
    static uint32_t count = 0, cur_backup = 0, cur_ave = 0;
    uint32_t cur = 0;
    uint8_t cal_ok = 1;
    uint32_t cal_ave = 0;
    // Whether need reduce interval, 0: no need; 1: Suggest; 2: Must
    int cal_lvl = 0;
    static uint8_t cal_err = 0;

#ifdef HAL_LRC_TESN_TEST
    static int temp1, temp2;
#endif

    uint8_t temp_cycle = HAL_RC_CAL_GetLPCycle();
    uint32_t alpha;

    int temp=0;

    if (HAL_RTC_LXT_ENABLED())
    {
        return -1;
    }

    if (HAL_RC_locked() != 0)
        return -3;

#ifdef HAL_LRC_TESN_TEST
#if defined(SF32LB56X)
    if (!g_is_tsen_init)
    {
        g_is_tsen_init = 1;
        HAL_TSEN_Init(&g_tsen_handle);
    }

    //tmp = HAL_TSEN_Read(&g_tsen_handle);
#endif
#endif

#ifdef SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    //HAL_Delay(5);
#endif
    {
        uint32_t rcc_reg, div, div1, div2, try_times, delta = 0;
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
        //HAL_Delay(10);
        // switch system clock to HXT48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, 1);

        rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
        div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;
        div1 = (rcc_reg & LPSYS_RCC_CFGR_PDIV1_Msk) >> LPSYS_RCC_CFGR_PDIV1_Pos;
        div2 = (rcc_reg & LPSYS_RCC_CFGR_PDIV2_Msk) >> LPSYS_RCC_CFGR_PDIV2_Pos;
        // Reduce LPAON frequency to avoid voltage unstable
        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(-1, -1, 7);

// debug code for multi time calculate
#if 0
        static uint8_t test_cnt = 0;
        if (test_cnt++ <= 10)
        {
#ifdef SF32LB52X
            MODIFY_REG(hwp_bt_mac->RCCAL_CTRL, BT_MAC_RCCAL_CTRL_CON_NUM_Msk, 1 << BT_MAC_RCCAL_CTRL_CON_NUM_Pos);
            hwp_bt_mac->RCCAL_CTRL |= BT_MAC_RCCAL_CTRL_CON_MODE;
#endif


            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_bt_mac->RCCAL_CTRL |= (0x0 << BT_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_bt_mac->RCCAL_CTRL |= (lp_cycle << BT_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_bt_mac->RCCAL_CTRL |= (0x1 << BT_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            HAL_Delay(1);
            while (!(hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);

            HAL_DBG_printf("LPCycles temp(%d): cur=%d ,cur_scale %d, tmp=%d\n", test_cnt, cur, cur * temp_cycle / lp_cycle, tmp);

            if (test_cnt < 10)
            {
                if (div2 != 7)
                    HAL_RCC_LCPU_SetDiv(div, div1, div2);
                g_xt48_used = 0;
                HAL_RC_unlocked();
                return 0;
            }

            test_cnt = 0;
        }
#endif

        lp_cycle = temp_cycle;

#ifdef HAL_LRC_TESN_TEST
#if defined(SF32LB56X)
        temp = HAL_TSEN_Read(&g_tsen_handle);
#endif
#endif

        for (try_times = 0; try_times < 10; try_times++)
        {

#ifdef SF32LB52X
            MODIFY_REG(hwp_bt_mac->RCCAL_CTRL, BT_MAC_RCCAL_CTRL_CON_NUM_Msk, 1 << BT_MAC_RCCAL_CTRL_CON_NUM_Pos);
            hwp_bt_mac->RCCAL_CTRL |= BT_MAC_RCCAL_CTRL_CON_MODE;
#endif

            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_AUTO;
            hwp_bt_mac->RCCAL_CTRL |= (0x0 << BT_MAC_RCCAL_CTRL_RCCAL_AUTO_Pos);
            hwp_bt_mac->RCCAL_CTRL &= ~BT_MAC_RCCAL_CTRL_RCCAL_LENGTH;
            hwp_bt_mac->RCCAL_CTRL |= (lp_cycle << BT_MAC_RCCAL_CTRL_RCCAL_LENGTH_Pos);
            hwp_bt_mac->RCCAL_CTRL |= (0x1 << BT_MAC_RCCAL_CTRL_RCCAL_START_Pos);
            HAL_Delay(1);
            while (!(hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_DONE_Msk));
            cur = (hwp_bt_mac->RCCAL_RESULT & BT_MAC_RCCAL_RESULT_RCCAL_RESULT_Msk);

            if (count >= LRC_CAL_JUDGE_COUNT)
            {
                cal_ave = HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE);
                delta = cur > cal_ave ? (cur - cal_ave) : (cal_ave - cur);

                if (delta < LRC_CAL_RETRY_OFFSET)
                    break;
            }
            else
            {
                break;
            }
        }
        HAL_DBG_printf("LPCycles: cur=%d delta=%d,tmp=%d\n", cur, delta, temp);

        if (try_times == 10)
        {
            cal_ok = 0;
            cal_err++;
            if (cal_err == 4)
            {
                count = 0;
            }
        }
        else
        {
            HAL_Set_backup(RTC_BACKUP_LPCYCLE_CUR, cur);
        }

        if (div2 != 7)
            HAL_RCC_LCPU_SetDiv(div, div1, div2);
        g_xt48_used = 0;
    }

    if (0 == cal_ok)
    {
#ifdef SF32LB52X
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
        HAL_RC_unlocked();
        return -2;
    }


// According algorithm
    /*
        for n = 1: N
            curr = data_in(n);

            if n<2
                alpha = 1;
            else
                if (((curr - avg_reg)>thd0)&& ((curr_reg - avg_reg)>thd0)) || (((curr - avg_reg)<-thd1)&& ((curr_reg - avg_reg)<-thd1))
                    alpha = 1 - 0.125 - 0.0625;
                else
                    alpha = 0.125;
                end
            end

            avg_reg = (1-alpha)*avg_reg + alpha*curr;
            avg_out(n) =  avg_reg;
            curr_reg = curr;
        end
    */

    uint8_t scale = HAL_RC_CAL_get_filter_scale(lp_cycle);
    cur *= scale;

    uint16_t thd0 = HAL_RC_get_fitler_thd0();
    uint16_t thd1 = HAL_RC_get_fitler_thd1();

    HAL_DBG_printf("LP cycles %d, thd0 %d, thd1 %d, scale %d", lp_cycle, thd0, thd1, scale);

    if (clear_ave)
    {
        count = 0;
        cal_lvl = 2;
    }

    if (count < (uint32_t)ave_window)
        count++;

    if (count < 2)
        alpha = 0;
    else if (count == 2)
        alpha = 1;
    else
    {
#ifdef HAL_LRC_TESN_TEST
        if (((temp >= temp1) && (((temp - temp2) >= LRC_CAL_DEFAULT_TEMP_THD0) || ((temp - temp1) >= LRC_CAL_DEFAULT_TEMP_THD1))) ||
                ((temp <= temp1) && (((temp2 - temp) >= LRC_CAL_DEFAULT_TEMP_THD0) || ((temp1 - temp) >= LRC_CAL_DEFAULT_TEMP_THD1))))
            alpha = 4;
        else
#endif
            if ((RC_COMPARE_MORE(cur, cur_ave, thd0) && RC_COMPARE_MORE(cur_backup, cur_ave, thd0)) ||
                    (RC_COMPARE_LESS(cur, cur_ave, thd1) && RC_COMPARE_LESS(cur_backup, cur_ave, thd1)))
                alpha = 3;
            else
                alpha = 2;
    }

    if (alpha == 0)
        cur_ave = cur;
    else if (alpha == 1)
        cur_ave = (cur_ave + cur + 1) >> 1;
#ifdef HAL_LRC_TESN_TEST
    else if (alpha == 4)
        cur_ave = (cur_ave * 3  + cur * 5 + 4) >> 3;
#endif
    else if (alpha == 3)
        cur_ave = (cur_ave * 3 + cur * 13 + 8) >> 4;
    else
        cur_ave = (cur_ave * 7 + cur + 4) >> 3;


    HAL_DBG_printf("Actual Para: cur=%d, ave=%d, cur_backup %d, alpha %d, tmp(*10)=%d\r\n", cur, cur_ave, cur_backup, alpha, temp);
    cur_backup = cur;
    cal_ave = cur_ave / scale;

#ifdef HAL_LRC_TESN_TEST
    temp2 = temp1;
    temp1 = temp;
#endif

    HAL_Set_backup(RTC_BACKUP_LPCYCLE_AVE, cal_ave);

#ifdef SF32LB52X
    {
        float lxtfreq;
        uint8_t is_lcpu_cal = 0;
        uint16_t len = 1;
        lxtfreq = (48000000UL / (float)cur_ave * lp_cycle);
        HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_BT_RC_CAL_IN_L, &is_lcpu_cal, &len);
        if (!is_lcpu_cal)
        {
            HAL_DBG_printf("set ave");
            HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_AVE, &cal_ave, 4);
        }
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LPCYCLE_CURR, &lxtfreq, 4);
    }
#endif

    //HAL_DBG_printf("Actual Para: cur=%d, ave=%d, cur_backup %d, tmp=%d\r\n", cur, cur_ave, cur_backup, tmp);
    HAL_DBG_printf("LPCycles: %d, ave=%d, count=%d tmp=%d\r\n", HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR), HAL_Get_backup(RTC_BACKUP_LPCYCLE_AVE), count, temp);


#ifdef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif
    HAL_RC_unlocked();
    return cal_lvl;
}

#endif

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

