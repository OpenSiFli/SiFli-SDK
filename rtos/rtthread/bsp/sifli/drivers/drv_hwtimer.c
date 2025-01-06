/**
  ******************************************************************************
  * @file   drv_hwtimer.c
  * @author Sifli software development team
  * @brief Hardware Timer BSP driver
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

#include <stdio.h>
#include <board.h>



/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_hwtimer Hardware Timer
  * @brief Hardware Timer BSP driver
  * @{
  */


#if defined(BSP_USING_TIM) || defined(_SIFLI_DOXYGEN_)
#include "drv_config.h"
#include "bf0_hal_tim.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.hwtimer"
#include <drv_log.h>

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

#if defined(RT_USING_HWTIMER) || defined(_SIFLI_DOXYGEN_)


/* #if defined(BSP_USING_GPTIM1) || defined(BSP_USING_GPTIM2) || defined(BSP_USING_GPTIM3) \
    || defined(BSP_USING_GPTIM4) || defined(BSP_USING_GPTIM5) \
    || defined(BSP_USING_BTIM1) || defined(BSP_USING_BTIM2) \
    || defined(BSP_USING_BTIM3) || defined(BSP_USING_BTIM4) */

#define BSP_USING_GPT_BTIM
//#endif /* BSP_USING_GPT_BTIM */

//#if defined(BSP_USING_LPTIM1) || defined(BSP_USING_LPTIM2) || defined(BSP_USING_LPTIM3)
#define BSP_USING_LPTIM
//#endif


#ifdef BSP_USING_GPT_BTIM

enum
{
#if defined(BSP_USING_GPTIM1) || defined(_SIFLI_DOXYGEN_)
    GPTIM1_INDEX,
#endif
#if defined(BSP_USING_GPTIM2) || defined(_SIFLI_DOXYGEN_)
    GPTIM2_INDEX,
#endif
#if defined(BSP_USING_GPTIM3) || defined(_SIFLI_DOXYGEN_)
    GPTIM3_INDEX,
#endif
#if defined(BSP_USING_GPTIM4) || defined(_SIFLI_DOXYGEN_)
    GPTIM4_INDEX,
#endif
#if defined(BSP_USING_GPTIM5) || defined(_SIFLI_DOXYGEN_)
    GPTIM5_INDEX,
#endif
#if defined(BSP_USING_ATIM1) || defined(_SIFLI_DOXYGEN_)
    ATIM1_INDEX,
#endif
#if defined(BSP_USING_ATIM2) || defined(_SIFLI_DOXYGEN_)
    ATIM2_INDEX,
#endif

#if defined(BSP_USING_BTIM1) || defined(_SIFLI_DOXYGEN_)
    BTIM1_INDEX,
#endif
#if defined(BSP_USING_BTIM2) || defined(_SIFLI_DOXYGEN_)
    BTIM2_INDEX,
#endif
#if defined(BSP_USING_BTIM3) || defined(_SIFLI_DOXYGEN_)
    BTIM3_INDEX,
#endif
#if defined(BSP_USING_BTIM4) || defined(_SIFLI_DOXYGEN_)
    BTIM4_INDEX,
#endif
    BTIM_MAX,
};
#endif /* BSP_USING_GPT_BTIM */

struct bf0_hwtimer
{
    rt_hwtimer_t time_device;           /*!< HW timer os device */
    GPT_HandleTypeDef    tim_handle;    /*!< HW timer low level handle */
    IRQn_Type tim_irqn;                 /*!< interrupt number for timer*/
    uint8_t core;                       /*!< Clock source from which core*/
    char *name;                         /*!< HW timer device name*/
};

#if defined(BSP_USING_LPTIM) || defined(_SIFLI_DOXYGEN_)
#include "bf0_hal_lptim.h"
struct bf0_lptimer
{
    rt_hwtimer_t time_device;           /*!< HW timer os device */
    LPTIM_HandleTypeDef tim_handle;     /*!< HW timer low level handle */
    IRQn_Type tim_irqn;                 /*!< interrupt number for timer*/
    char *name;                         /*!< HW timer device name*/
};
#endif

#ifdef BSP_USING_GPT_BTIM
struct bf0_hwtimer bf0_hwtimer_obj[] =
{
#if defined(BSP_USING_GPTIM1) || defined(_SIFLI_DOXYGEN_)
    GPTIM1_CONFIG,
#endif

#if defined(BSP_USING_GPTIM2) || defined(_SIFLI_DOXYGEN_)
    GPTIM2_CONFIG,
#endif

#if defined(BSP_USING_GPTIM3) || defined(_SIFLI_DOXYGEN_)
    GPTIM3_CONFIG,
#endif

#if defined(BSP_USING_GPTIM4) || defined(_SIFLI_DOXYGEN_)
    GPTIM4_CONFIG,
#endif

#if defined(BSP_USING_GPTIM5) || defined(_SIFLI_DOXYGEN_)
    GPTIM5_CONFIG,
#endif

#if defined(BSP_USING_ATIM1) || defined(_SIFLI_DOXYGEN_)
    ATIM1_CONFIG,
#endif
#if defined(BSP_USING_ATIM2) || defined(_SIFLI_DOXYGEN_)
    ATIM2_CONFIG,
#endif

#if defined(BSP_USING_BTIM1) || defined(_SIFLI_DOXYGEN_)
    BTIM1_CONFIG,
#endif
#if defined(BSP_USING_BTIM2) || defined(_SIFLI_DOXYGEN_)
    BTIM2_CONFIG,
#endif
#if defined(BSP_USING_BTIM3) || defined(_SIFLI_DOXYGEN_)
    BTIM3_CONFIG,
#endif
#if defined(BSP_USING_BTIM4) || defined(_SIFLI_DOXYGEN_)
    BTIM4_CONFIG,
#endif
};
#endif /* BSP_USING_GPT_BTIM */

#if defined(BSP_USING_LPTIM) || defined(_SIFLI_DOXYGEN_)

#ifdef __CC_ARM
    #pragma arm section rwdata="UNINITZI"
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section data="UNINITRW"
#else
#endif

#if /*defined(LXT_DISABLE)&&*/defined(SOC_SF32LB55X)&&defined(BF0_LCPU)
    __ROM_USED struct bf0_lptimer bf0_lptimer_obj[] =
#else
    static struct bf0_lptimer bf0_lptimer_obj[] =
#endif
{
#if defined(BSP_USING_LPTIM1) || defined(_SIFLI_DOXYGEN_)
    LPTIM1_CONFIG,
#endif
#if defined(BSP_USING_LPTIM2) || defined(_SIFLI_DOXYGEN_)
    LPTIM2_CONFIG,
#endif
#if defined(BSP_USING_LPTIM3) || defined(_SIFLI_DOXYGEN_)
    LPTIM3_CONFIG,
#endif
};

#ifdef __CC_ARM
    #pragma arm section rwdata
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section data=""
#else
#endif

enum
{
#if defined(BSP_USING_LPTIM1) || defined(_SIFLI_DOXYGEN_)
    LPTIM1_INDEX,
#endif
#if defined(BSP_USING_LPTIM2) || defined(_SIFLI_DOXYGEN_)
    LPTIM2_INDEX,
#endif
#if defined(BSP_USING_LPTIM3) || defined(_SIFLI_DOXYGEN_)
    LPTIM3_INDEX,
#endif
    LPTIM_MAX
};

#endif  /* BSP_USING_LPTIM */

#ifdef RT_USING_PM
    #ifdef PM_STANDBY_ENABLE
        //static void bf0_hwtimer_recovery(uint8_t *user_data);
    #endif
#endif
/** @defgroup hwtimer_device HW timer device functions registered to OS
 * @ingroup drv_hwtimer
 * @{
 */


/**
  * @brief  Initialize HW timer device.
  * @param[in]  timer: HW timer device.
  * @param[in]  state: 1 open, 0 close.
  */
static void timer_init(struct rt_hwtimer_device *timer, rt_uint32_t state)
{
    uint32_t prescaler_value = 0;
    GPT_HandleTypeDef *tim = RT_NULL;
    struct bf0_hwtimer *tim_device = RT_NULL;

    RT_ASSERT(timer != RT_NULL);
    if (state)
    {
        tim = (GPT_HandleTypeDef *)timer->parent.user_data;
        tim_device = (struct bf0_hwtimer *)timer;

        prescaler_value = HAL_RCC_GetPCLKFreq(
                              tim_device->core,
                              1);
#ifdef SF32LB52X
        if (tim->Instance == hwp_gptim2 || tim->Instance == (GPT_TypeDef *)hwp_btim2)
            prescaler_value = 24000000;
#endif
        prescaler_value = prescaler_value / 1000 - 1;
        tim->Init.Period            = 10000 - 1;
        tim->Init.Prescaler         = prescaler_value;
        tim->core                   = tim_device->core;
        if (timer->info->cntmode == HWTIMER_CNTMODE_UP)
        {
            tim->Init.CounterMode   = GPT_COUNTERMODE_UP;
        }
        else
        {
            tim->Init.CounterMode   = GPT_COUNTERMODE_DOWN;
        }
        tim->Init.RepetitionCounter = 0;

        if (HAL_GPT_Base_Init(tim) != HAL_OK)
        {
            LOG_E("%s init failed", tim_device->name);
            return;
        }
        else
        {
            /* set the TIMx priority */
            HAL_NVIC_SetPriority(tim_device->tim_irqn, 3, 0);

            /* enable the TIMx global Interrupt */
            HAL_NVIC_EnableIRQ(tim_device->tim_irqn);

            /* clear update flag */
            __HAL_GPT_CLEAR_FLAG(tim, GPT_FLAG_UPDATE);
            /* enable update request source */
            __HAL_GPT_URS_ENABLE(tim);

            LOG_D("%s init success", tim_device->name);
        }
    }
}

/**
  * @brief  Start HW timer .
  * @param[in]  timer  HW timer device.
  * @param[in]  t timer count loaded.
  * @param[in]  opmode  1 one short, 2 periodical .
  * @retval RT_EOK if success, -RT_ERROR if failed.
  */
static rt_err_t timer_start(rt_hwtimer_t *timer, rt_uint32_t t, rt_hwtimer_mode_t opmode)
{
    rt_err_t result = RT_EOK;
    GPT_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (GPT_HandleTypeDef *)timer->parent.user_data;

    /* set tim cnt */
#if 0
    LOG_D("Counter set to %d\n", t);
    LOG_D("Prsc set to %d\n", tim->Instance->PSC);
    LOG_D("PCLK is %d\n", HAL_RCC_GetPCLKFreq(CORE_ID_DEFAULT, 1));
    LOG_D("HCLK is %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_DEFAULT));
#endif

    __HAL_GPT_SET_AUTORELOAD(tim, t);

    if (opmode == HWTIMER_MODE_ONESHOT)
    {
        /* set timer to single mode */
        tim->Instance->CR1 |= GPT_OPMODE_SINGLE;
    }
    else
    {
        /* set timer to Repetitive mode */
        tim->Instance->CR1 &= ~GPT_OPMODE_SINGLE;
    }
    /* start timer */
    if (HAL_GPT_Base_Start_IT(tim) != HAL_OK)
    {
        LOG_E("TIM2 start failed");
        result = -RT_ERROR;
    }

    return result;
}

/**
  * @brief  Stop HW timer.
  * @param[in]  timer: HW timer device.
  */
static void timer_stop(rt_hwtimer_t *timer)
{
    GPT_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (GPT_HandleTypeDef *)timer->parent.user_data;

    /* stop timer */
    HAL_GPT_Base_Stop_IT(tim);
}

/**
  * @brief  HW timer configuration.
  * @param[in]  timer: HW timer device.
  * @param[in]  cmd: HW timer configuration command.
  * @param[in]  arg: HW timer configuration command argument.
  */
static rt_err_t timer_ctrl(rt_hwtimer_t *timer, rt_uint32_t cmd, void *arg)
{
    GPT_HandleTypeDef *tim = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(timer != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    tim = (GPT_HandleTypeDef *)timer->parent.user_data;

    switch (cmd)
    {
    case HWTIMER_CTRL_FREQ_SET:
    {
        rt_uint32_t freq;
        rt_uint32_t val;

        /* set timer frequence */
        freq = *((rt_uint32_t *)arg);

        val = HAL_RCC_GetPCLKFreq(
                  tim->core,
                  1);
#ifdef SF32LB52X
        if (tim->Instance == hwp_gptim2 || tim->Instance == (GPT_TypeDef *)hwp_btim2)
            val = 24000000;
#endif
        val /= freq;
        if (val > RT_UINT16_MAX)
            result = -RT_EINVAL;
        else
        {
            __HAL_GPT_SET_PRESCALER(tim, val - 1);
            tim->Instance->EGR |= GPT_EVENTSOURCE_UPDATE; /* Update frequency value */
        }
    }
    break;
    default:
    {
        result = -RT_ENOSYS;
    }
    break;
    }

    return result;
}

/**
  * @brief  Get HW timer counter.
  * @param[in]  timer: HW timer device.
  * @retval timer count
  */
static rt_uint32_t timer_counter_get(rt_hwtimer_t *timer)
{
    GPT_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (GPT_HandleTypeDef *)timer->parent.user_data;

    return tim->Instance->CNT;
}

static const struct rt_hwtimer_info _info = GPT_DEV_INFO_CONFIG;

static const struct rt_hwtimer_ops _ops =
{
    .init = timer_init,
    .start = timer_start,
    .stop = timer_stop,
    .count_get = timer_counter_get,
    .control = timer_ctrl,
};

#if defined(BSP_USING_LPTIM) || defined(_SIFLI_DOXYGEN_)

static const struct rt_hwtimer_info _lp_info = LPTIM_DEV_INFO_CONFIG;
/**
  * @brief  Initialize HW low power timer device.
  * @param[in]  timer: HW low power timer device.
  * @param[in]  state: 1 open, 0 close.
  */
static void lp_timer_init(struct rt_hwtimer_device *timer, rt_uint32_t state)
{

    LPTIM_HandleTypeDef *tim = RT_NULL;
    struct bf0_lptimer *tim_device = RT_NULL;

    RT_ASSERT(timer != RT_NULL);
    if (state)
    {
        tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;
        tim_device = (struct bf0_lptimer *)timer;
        HAL_LPTIM_InitDefault(tim);
        if (HAL_LPTIM_Init(tim) != HAL_OK)
        {
            LOG_E("%s init failed", tim_device->name);
            return;
        }
        else
        {
            /* set the TIMx priority */
            HAL_NVIC_SetPriority(tim_device->tim_irqn, 3, 0);

            /* enable the TIMx global Interrupt */
            HAL_NVIC_EnableIRQ(tim_device->tim_irqn);


            LOG_D("%s init success", tim_device->name);
        }
    }

}

/**
  * @brief  Start HW low power timer .
  * @param[in]  timer   HW low power timer device.
  * @param[in]  t       timer count loaded.
  * @param[in]  opmode  1 one short, 2 periodical .
  * @retval RT_EOK if success, -RT_ERROR if failed.
  */
static rt_err_t lp_timer_start(rt_hwtimer_t *timer, rt_uint32_t t, rt_hwtimer_mode_t opmode)
{

    rt_err_t result = RT_EOK;
    LPTIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;

    if (opmode == HWTIMER_MODE_ONESHOT)
    {
        tim->Mode = HAL_LPTIM_ONESHOT;
    }
    else
    {
        tim->Mode = HAL_LPTIM_PERIOD;
    }


    /* start timer */
    if (HAL_LPTIM_Counter_Start_IT(tim, t) != HAL_OK)
    {
        LOG_E("LPTIM start failed");
        result = -RT_ERROR;
    }

    return result;
}

/**
  * @brief  Stop HW low power timer.
  * @param[in]  timer: HW low power timer device.
  */
static void lp_timer_stop(rt_hwtimer_t *timer)
{
    LPTIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;

    /* stop timer */
    HAL_LPTIM_Counter_Stop_IT(tim);

}


static uint32_t lp_timer_freq_div_calc(rt_uint32_t freq)
{
    uint32_t div[8] = {LPTIM_PRESCALER_DIV1, LPTIM_PRESCALER_DIV2, LPTIM_PRESCALER_DIV4, LPTIM_PRESCALER_DIV8, LPTIM_PRESCALER_DIV16,
                       LPTIM_PRESCALER_DIV32, LPTIM_PRESCALER_DIV64, LPTIM_PRESCALER_DIV128
                      };

    uint32_t clk_src = _lp_info.maxfreq;
    /* ffff means calc failed */
    uint32_t ret = 0xffff;

    if (freq <= clk_src)
    {
        uint8_t rem = clk_src / freq;
        if (clk_src % freq == 0)
        {
            for (uint32_t i = 0; i < 8; i++)
            {
                if (rem == 1 << i)
                {
                    ret = div[i];
                    break;
                }
            }
        }
    }

    return ret;
}

/**
  * @brief  HW timer configuration.
  * @param[in]  timer: HW low power timer device.
  * @param[in]  cmd: HW low power timer configuration command.
  * @param[in]  arg: HW low power timer configuration command argument.
  */
static rt_err_t lp_timer_ctrl(rt_hwtimer_t *timer, rt_uint32_t cmd, void *arg)
{
    LPTIM_HandleTypeDef *tim = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(timer != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;

    switch (cmd)
    {
    case HWTIMER_CTRL_FREQ_SET:
    {

        rt_uint32_t freq;
        /* set timer frequence */
        if ((tim->Init.Clock.Source ==  LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC) &&
                (tim->Init.Clock.IntSource == LPTIM_INTCLOCKSOURCE_LPCLOCK))
        {

            uint32_t div;
            freq = *((rt_uint32_t *)arg);
            if (HAL_LXT_DISABLED())
            {
                *((rt_uint32_t *)arg) = (48000000ULL * LXT_LP_CYCLE) / HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
                div = LPTIM_PRESCALER_DIV1;
            }
            else
            {
                div = lp_timer_freq_div_calc(freq);
            }
            if (freq != 0xFFFF)
            {
                __HAL_LPTIM_CLEAR_PRESCALER(tim, LPTIM_PRESCALER_DIV128);
                __HAL_LPTIM_SET_PRESCALER(tim, div);
            }
            else
            {
                result = -RT_EINVAL;
            }
        }
        else
        {
            /* For RTT, only use internal 32K clock source */
            result = -RT_EINVAL;
        }

    }
    break;
    default:
    {
        result = -RT_ENOSYS;
    }
    break;
    }

    return result;

}

/**
  * @brief  Get HW timer counter.
  * @param[in]  timer: HW timer device.
  * @retval timer count
  */
static rt_uint32_t lp_timer_counter_get(rt_hwtimer_t *timer)
{
    LPTIM_HandleTypeDef *tim = RT_NULL;

    RT_ASSERT(timer != RT_NULL);

    tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;

    if (HAL_LPTIM_PERIOD == tim->Mode)
    {
        /* check overflow interrupt in case ISR is not processed yet when wakeup by LPTIM interrupt,
         * while counter has wrapped to 0 in periodic mode
         */
        HAL_LPTIM_IRQHandler(tim);
    }

    return HAL_LPTIM_ReadCounter(tim);

}


// TODO: For A0 LCPU RC10K only, Revisit for future Chipset
#if /*defined(LXT_DISABLE)&&*/defined(SOC_SF32LB55X)&&defined(BF0_LCPU)
    __ROM_USED const struct rt_hwtimer_ops _lp_ops =
#else
    static const struct rt_hwtimer_ops _lp_ops =
#endif
{
    .init = lp_timer_init,
    .start = lp_timer_start,
    .stop = lp_timer_stop,
    .count_get = lp_timer_counter_get,
    .control = lp_timer_ctrl,
};
#endif  /* BSP_USING_LPTIM */

#if defined(BSP_USING_ATIM1) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 2 interrupt
  */
void ATIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[ATIM1_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_ATIM2) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 2 interrupt
  */
void ATIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[ATIM2_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_GPTIM1) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 2 interrupt
  */
void GPTIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[GPTIM1_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_GPTIM2) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 3 interrupt
  */
void GPTIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[GPTIM2_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_GPTIM3) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 4 interrupt
  */
void GPTIM3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[GPTIM3_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_GPTIM4) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 4 interrupt
  */
void GPTIM4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[GPTIM4_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_GPTIM5) || defined(_SIFLI_DOXYGEN_)
/**
  * @brief  HW timer 4 interrupt
  */
void GPTIM5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[GPTIM5_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif


#if defined(BSP_USING_BTIM1) || defined(_SIFLI_DOXYGEN_)
void BTIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[BTIM1_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_BTIM2) || defined(_SIFLI_DOXYGEN_)
void BTIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[BTIM2_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_BTIM3) || defined(_SIFLI_DOXYGEN_)
void BTIM3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[BTIM3_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_BTIM4) || defined(_SIFLI_DOXYGEN_)
void BTIM4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_hwtimer_obj[BTIM4_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_LPTIM1) || defined(_SIFLI_DOXYGEN_)
void LPTIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_LPTIM_IRQHandler(&bf0_lptimer_obj[LPTIM1_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_LPTIM2) || defined(_SIFLI_DOXYGEN_)
#if LB55X_CHIP_ID >= 3 || !defined(SOC_SF32LB55X)
    void LPTIM2_IRQHandler(void)
#else
    __ROM_USED void LPTIM2_IRQHandler(void)
#endif
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_LPTIM_IRQHandler(&bf0_lptimer_obj[LPTIM2_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_LPTIM3) || defined(_SIFLI_DOXYGEN_)
#if defined(LB55X_CHIP_ID) && (LB55X_CHIP_ID<3)
    __ROM_USED void LPTIM3_IRQHandler(void)
#else
    void LPTIM3_IRQHandler(void)
#endif
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_LPTIM_IRQHandler(&bf0_lptimer_obj[LPTIM3_INDEX].tim_handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif


/**
  * @brief  HW timer periodical timeout call back
  * @param[in]  htim: low leverl HW timer device.
  */
#if LB55X_CHIP_ID >= 3 || !defined(SOC_SF32LB55X)
    void HAL_GPT_PeriodElapsedCallback(GPT_HandleTypeDef *htim)
#else
    __ROM_USED  void HAL_GPT_PeriodElapsedCallback(GPT_HandleTypeDef *htim)
#endif
{
#if defined(BSP_USING_ATIM1)
    if (htim->Instance == (GPT_TypeDef *)ATIM1)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[ATIM1_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_ATIM2)
    if (htim->Instance == (GPT_TypeDef *)ATIM2)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[ATIM2_INDEX].time_device);
    }
#endif


#if defined(BSP_USING_GPTIM1)
    if (htim->Instance == GPTIM1)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[GPTIM1_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_GPTIM2)

    if (htim->Instance == GPTIM2)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[GPTIM2_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_GPTIM3)
    if (htim->Instance == GPTIM3)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[GPTIM3_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_GPTIM4)
    if (htim->Instance == GPTIM4)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[GPTIM4_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_GPTIM5)
    if (htim->Instance == GPTIM5)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[GPTIM5_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_BTIM1)
    if (htim->Instance == (GPT_TypeDef *)BTIM1)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[BTIM1_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_BTIM2)
    if (htim->Instance == (GPT_TypeDef *)BTIM2)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[BTIM2_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_BTIM3)
    if (htim->Instance == (GPT_TypeDef *)BTIM3)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[BTIM3_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_BTIM4)
    if (htim->Instance == (GPT_TypeDef *)BTIM4)
    {
        rt_device_hwtimer_isr(&bf0_hwtimer_obj[BTIM4_INDEX].time_device);
    }
#endif
}
/**
@}
*/

/**
  * @brief  LP HW timer periodical timeout call back
  * @param[in]  hlptim low level HW timer device.
  */
#if defined(BSP_USING_LPTIM) || defined(_SIFLI_DOXYGEN_)
#if LB55X_CHIP_ID >= 3 || !defined(SOC_SF32LB55X)
    void HAL_LPTIM_AutoReloadWriteCallback(LPTIM_HandleTypeDef *hlptim)
#else
    __ROM_USED  void HAL_LPTIM_AutoReloadWriteCallback(LPTIM_HandleTypeDef *hlptim)
#endif
{
#if defined(BSP_USING_LPTIM1) || defined(_SIFLI_DOXYGEN_)
    if (hlptim->Instance == bf0_lptimer_obj[LPTIM1_INDEX].tim_handle.Instance)
    {
        rt_device_hwtimer_isr(&bf0_lptimer_obj[LPTIM1_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_LPTIM2) || defined(_SIFLI_DOXYGEN_)
    if (hlptim->Instance == bf0_lptimer_obj[LPTIM2_INDEX].tim_handle.Instance)
    {
        rt_device_hwtimer_isr(&bf0_lptimer_obj[LPTIM2_INDEX].time_device);
    }
#endif
#if defined(BSP_USING_LPTIM3) || defined(_SIFLI_DOXYGEN_)
    if (hlptim->Instance == bf0_lptimer_obj[LPTIM3_INDEX].tim_handle.Instance)
    {
        rt_device_hwtimer_isr(&bf0_lptimer_obj[LPTIM3_INDEX].time_device);
    }
#endif
}
#endif /*BSP_USING_LPTIM*/

/**
@}
*/


__ROM_USED int bf0_gptimer_init2(struct bf0_hwtimer *hwtimers, uint8_t cnt)
{
    int result = RT_EOK;
    int i;
    for (i = 0; i < cnt; i++)
    {
        hwtimers[i].time_device.info = &_info;
        hwtimers[i].time_device.ops  = &_ops;
        if (rt_device_hwtimer_register(&hwtimers[i].time_device, hwtimers[i].name, &hwtimers[i].tim_handle) == RT_EOK)
        {
            LOG_D("%s register success", hwtimers[i].name);
        }
        else
        {
            LOG_E("%s register failed", hwtimers[i].name);
            result = -RT_ERROR;
        }
    }
    return result;
}

#ifdef BSP_USING_LPTIM
__ROM_USED int bf0_lptimer_init2(struct bf0_lptimer *hwtimers, uint8_t cnt)
{
    int result = RT_EOK;
    int i;

    /* For Lower power timer init */
    for (i = 0; i < cnt ; i++)
    {
        hwtimers[i].time_device.info = &_lp_info;
        hwtimers[i].time_device.ops = &_lp_ops;
        if (rt_device_hwtimer_register(&hwtimers[i].time_device, hwtimers[i].name, &hwtimers[i].tim_handle) == RT_EOK)
            LOG_D("%s register success", hwtimers[i].name);
        else
        {
            LOG_E("%s register failed", hwtimers[i].name);
            result = -RT_ERROR;
        }
    }
    return result;
}
#endif
/**
  * @brief  HW timer driver initialize
  * @retval RT_EOK if success, -RT_ERROR if failed.
  */
__ROM_USED int bf0_hwtimer_init(void)
{
    int result = RT_EOK;

#ifdef BSP_USING_GPT_BTIM
    result = bf0_gptimer_init2(bf0_hwtimer_obj, sizeof(bf0_hwtimer_obj) / sizeof(bf0_hwtimer_obj[0]));
#endif /* BSP_USING_GPT_BTIM */

#if defined(BSP_USING_LPTIM) || defined(_SIFLI_DOXYGEN_)
    result = bf0_lptimer_init2(bf0_lptimer_obj, sizeof(bf0_lptimer_obj) / sizeof(bf0_lptimer_obj[0]));
#ifdef RT_USING_PM
    if (0 != SystemPowerOnModeGet())
    {
        /* set the TIMx priority */
        HAL_NVIC_SetPriority(bf0_lptimer_obj[0].tim_irqn, 3, 0);
        /* enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(bf0_lptimer_obj[0].tim_irqn);
        //TODO: stop timer in case it's not reintialized
    }
#endif // RT_USING_PM
#endif /*BSP_USING_LPTIM*/

    return result;
}

void bf0_lptimer_start(rt_hwtimer_t *timer, uint32_t timeout)
{
    uint32_t counter = timeout * timer->freq / RT_TICK_PER_SECOND;
    LPTIM_HandleTypeDef *tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;
    if (counter > timer->info->maxcnt)
        counter = timer->info->maxcnt;
    timer->cycles = 1;
    timer->reload = 1;
    timer->period_sec = counter / timer->freq;
    timer->overflow = 0;
    HAL_LPTIM_Counter_Start_IT(tim, counter);
}

// TODO: For A0 LCPU RC10K only, Revisit for future Chipset
#if /*defined(LXT_DISABLE)&&*/ defined(SOC_SF32LB55X)&&defined(BF0_LCPU)
#include "string.h"

static rt_err_t lp_timer_ctrl2(rt_hwtimer_t *timer, rt_uint32_t cmd, void *arg)
{
    LPTIM_HandleTypeDef *tim = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(timer != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    tim = (LPTIM_HandleTypeDef *)timer->parent.user_data;

    switch (cmd)
    {
    case HWTIMER_CTRL_FREQ_SET:
    {

        rt_uint32_t freq;
        /* set timer frequence */
        if ((tim->Init.Clock.Source ==  LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC) &&
                (tim->Init.Clock.IntSource == LPTIM_INTCLOCKSOURCE_LPCLOCK))
        {

            uint32_t div;
            freq = *((rt_uint32_t *)arg);
            if (HAL_LXT_DISABLED())
            {
                *((rt_uint32_t *)arg) = (48000000ULL * LXT_LP_CYCLE) / HAL_Get_backup(RTC_BACKUP_LPCYCLE_CUR);
                div = LPTIM_PRESCALER_DIV1;
                //rt_kprintf("freq real=%d\n", *((rt_uint32_t *)arg));
            }
            else
            {
                div = lp_timer_freq_div_calc(freq);
            }
            if (freq != 0xFFFF)
            {
                __HAL_LPTIM_CLEAR_PRESCALER(tim, LPTIM_PRESCALER_DIV128);
                __HAL_LPTIM_SET_PRESCALER(tim, div);
            }
            else
            {
                result = -RT_EINVAL;
            }
        }
        else
        {
            /* For RTT, only use internal 32K clock source */
            result = -RT_EINVAL;
        }

    }
    break;
    default:
    {
        result = -RT_ENOSYS;
    }
    break;
    }

    return result;
}

struct rt_hwtimer_ops _lp_ops2;
int bf0_hwtimer_init2(void)
{
    int r = bf0_hwtimer_init();
    int i;

    memcpy(&_lp_ops2, &_lp_ops, sizeof(_lp_ops));
    _lp_ops2.control = lp_timer_ctrl2;
    for (i = 0; i < sizeof(bf0_lptimer_obj) / sizeof(bf0_lptimer_obj[0]); i++)
        bf0_lptimer_obj[i].time_device.ops = &_lp_ops2;
    return r;
}

INIT_BOARD_EXPORT(bf0_hwtimer_init2);
#else
INIT_BOARD_EXPORT(bf0_hwtimer_init);
#endif

#ifdef RT_USING_PM
#if 0
static void bf0_hwtimer_recovery(uint8_t *user_data)
{
    struct bf0_lptimer *lptimer = (struct bf0_lptimer *) user_data;
    rt_object_init(&lptimer->time_device.parent.parent, RT_Object_Class_Device, lptimer->name);
    /* set the TIMx priority */
    HAL_NVIC_SetPriority(lptimer->tim_irqn, 3, 0);

    /* enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(lptimer->tim_irqn);

}
#endif //PM_STANDBY_ENABLE
#endif // RT_USING_PM

/// @} drv_hwtimer
/// @} bsp_driver


/** @addtogroup bsp_sample BSP driver sample commands.
  * @{
  */

/** @defgroup bsp_sample_hwtimer Hardware Timer sample commands
  * @brief Hardware Timer sample commands
  *
  * This sample commands demonstrate the usage of hardware timer driver.
  * @{
  */

//#define DRV_TEST
#if defined(DRV_TEST)

#include "string.h"
static rt_device_t g_t_hwtimer;

/**
  * @brief  Hardware timer timeout indication.
  * @param[in]  dev: HW timer device.
  * @param[in]  size: Timeout value size.
  * @retval RT_EOK
  */
static rt_err_t hwtm_rx_ind(rt_device_t dev, rt_size_t size)
{
    LOG_I("HW time out\n");
    return RT_EOK;
}

/**
* @brief  HWtimer commands.
* This function provide 'hwtimer' command to shell(FINSH) . It test with timer3.
* The commands supported:
*   - hwtimer set [index] [freq]

      Set frequency for HW timer[index]. Open timer device if not opened.

    - hwtimer write [sec] [usec]

      Start timer of sec.usec.

    - hwtimer read

      Read current timer count, dump the returned count as hex array.

    - hwtimer stop

      Stop hw timer

    - hwtimer get

      Get timer setting

    - hwtimer set [mode]

      Set mode for timer, 1: one shot, 2 : periodical

* @retval RT_EOK
*/

int cmd_hwtimer(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "set") == 0)
        {
            if (g_t_hwtimer == NULL)
            {
                char timer_s[7] = {0};
                int num = atoi(argv[2]);
                if (num < 6)
                {
                    snprintf(timer_s, 7, "gptim%d", num);
                    g_t_hwtimer = rt_device_find(timer_s);
                }
                if (g_t_hwtimer)
                    rt_device_open(g_t_hwtimer, RT_DEVICE_FLAG_RDWR);
            }

            if (g_t_hwtimer)
            {
                uint32_t freq = atoi(argv[3]);
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *) &freq);
                rt_device_set_rx_indicate(g_t_hwtimer, hwtm_rx_ind);
            }
        }
        if (strcmp(argv[1], "set_btim") == 0)
        {
            if (g_t_hwtimer == NULL)
            {
                char timer_s[7] = {0};
                int num = atoi(argv[2]);
                if (num < 5)
                {
                    snprintf(timer_s, 7, "btim%d", num);
                    g_t_hwtimer = rt_device_find(timer_s);
                }
                if (g_t_hwtimer)
                    rt_device_open(g_t_hwtimer, RT_DEVICE_FLAG_RDWR);
            }

            if (g_t_hwtimer)
            {
                uint32_t freq = atoi(argv[3]);
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *) &freq);
                rt_device_set_rx_indicate(g_t_hwtimer, hwtm_rx_ind);
            }
        }
        if (strcmp(argv[1], "LPset") == 0)
        {
            if (g_t_hwtimer == NULL)
            {
                char timer_s[7] = {0};
                int num = atoi(argv[2]);
                snprintf(timer_s, 7, "lptim%d", num);
                g_t_hwtimer = rt_device_find(timer_s);
                if (g_t_hwtimer)
                    rt_device_open(g_t_hwtimer, RT_DEVICE_FLAG_RDWR);
            }

            if (g_t_hwtimer)
            {
                uint32_t freq = atoi(argv[2]);
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *) &freq);
                rt_device_set_rx_indicate(g_t_hwtimer, hwtm_rx_ind);
            }
        }
        if (strcmp(argv[1], "write") == 0)
        {
            if (g_t_hwtimer)
            {
                rt_hwtimerval_t t;
                t.sec = atoi(argv[2]);
                t.usec = atoi(argv[3]);
                rt_device_write(g_t_hwtimer, 0, &t, sizeof(t));
            }
        }
        if (strcmp(argv[1], "read") == 0)
        {
            if (g_t_hwtimer)
            {
                rt_hwtimerval_t t;
                rt_device_read(g_t_hwtimer, 0, &t, sizeof(t));
                HAL_DBG_print_data((char *)&t, 0, sizeof(t));
            }
        }
        if (strcmp(argv[1], "stop") == 0)
        {
            if (g_t_hwtimer)
            {
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_STOP, NULL);
            }
        }
        if (strcmp(argv[1], "get") == 0)
        {
            if (g_t_hwtimer)
            {
                struct rt_hwtimer_info info;
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_INFO_GET, (void *)&info);
                HAL_DBG_print_data((char *)&info, 0, sizeof(info));
            }
        }
        if (strcmp(argv[1], "setmode") == 0)
        {
            if (g_t_hwtimer)
            {
                uint32_t mode = atoi(argv[2]);
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_MODE_SET, (void *)&mode);
            }
        }
        if (strcmp(argv[1], "setfreq") == 0)
        {
            if (g_t_hwtimer)
            {
                uint32_t freq = atoi(argv[2]);
                rt_device_control(g_t_hwtimer, HWTIMER_CTRL_FREQ_SET, (void *)&freq);
            }
        }
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_hwtimer, __cmd_hwtimer, Test hw timers);
#endif
/// @} bsp_sample_hwtimer
/// @} bsp_sample

#endif /* RT_USING_HWTIMER */
#endif /* BSP_USING_TIM */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
