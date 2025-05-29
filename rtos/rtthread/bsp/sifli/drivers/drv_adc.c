/**
  ******************************************************************************
  * @file   drv_adc.c
  * @author Sifli software development team
  * @brief ADC BSP driver
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
#include <string.h>
#include <stdlib.h>

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_adc ADC
  * @brief ADC BSP driver
  * @{
  */

#ifdef BSP_USING_ADC
#include "drv_config.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.adc"
#include <drv_log.h>

// New code default output voltage with rt_device_read, not register value, do not call calculate functin again !!!
// ONLY for debug, rt_device_read return register value, need calculte to voltage manual.
//#define ADC_DEBUG

static ADC_HandleTypeDef adc_config[] =
{
#ifdef BSP_USING_ADC1
    ADC1_CONFIG,
#endif

};
#if defined(hwp_gpadc1)
    ADC_HandleTypeDef   *ADC_Handler_gpadc1;
#endif
#if defined(hwp_gpadc2)
    ADC_HandleTypeDef   *ADC_Handler_gpadc2;
#endif

struct sifli_adc
{
    ADC_HandleTypeDef ADC_Handler;
    struct rt_adc_device sifli_adc_device;
};

//MAX support voltage, for A0, voltage = 1.1v, for RPO, voltage = 3.3v
#ifdef SF32LB55X
    #define ADC_MAX_VOLTAGE_MV     (1100)
#else
    #define ADC_MAX_VOLTAGE_MV     (3300)
#endif

#define ADC_SW_AVRA_CNT         (22)

// Standard voltage from ATE , it should not changed !!!
#define ADC_BIG_RANGE_VOL1           (1000)
#define ADC_BIG_RANGE_VOL2           (2500)
#define ADC_SML_RANGE_VOL1           (300)
#define ADC_SML_RANGE_VOL2           (800)


#define ADC_RATIO_ACCURATE          (1000)

static struct sifli_adc sifli_adc_obj[sizeof(adc_config) / sizeof(adc_config[0])];

#ifdef SF32LB55X
    // default value, they should be over write by calibrate
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 199.0;
    // mv per bit, if accuracy not enough, change to 0.1 mv or 0.01 mv later
    static float adc_vol_ratio = 3930.0; // 4296; //6 * ADC_RATIO_ACCURATE; //600; //6;
    static int adc_range = 0;   /* flag for ATE calibration voltage range,
    *  0 for big range (1.0v/2.5v)
    *  1 for small range () */
#elif defined(SF32LB56X)
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 822.0;
    // 0.001 mv per bit
    static float adc_vol_ratio = 1068.0; //
    static int adc_range = 1;
#else
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 822.0;
    // 0.001 mv per bit,
    static float adc_vol_ratio = 1068.0; //
    static int adc_range = 1;
#endif

static float adc_vbat_factor = 2.01;

// register data for max supported voltage, for A0, voltage = 1.1v, for RPO, voltage = 3.3v
static uint32_t adc_thd_reg;

static struct rt_semaphore         gpadc_lock;
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
struct bf0_hwtimer
{
    rt_hwtimer_t time_device;           /*!< HW timer os device */
    GPT_HandleTypeDef    tim_handle;    /*!< HW timer low level handle */
    IRQn_Type tim_irqn;                 /*!< interrupt number for timer*/
    uint8_t core;                       /*!< Clock source from which core*/
    char *name;                         /*!< HW timer device name*/
};

static const struct rt_hwtimer_info hwtimer_info = {                                           \
    .maxfreq = 1000000,                     \
               .minfreq = 2000,                        \
                          .maxcnt  = 0xFFFF,                      \
                                     .cntmode = HWTIMER_CNTMODE_UP,          \
};
GPT_HandleTypeDef tim;
struct bf0_hwtimer hw_gptimer;
extern void pin_debug_toggle();

void GPTIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&tim);
    /* leave interrupt */
    rt_interrupt_leave();
}

static rt_uint32_t time_calc(rt_hwtimer_t *timer, rt_hwtimerval_t *tv)
{
    float overflow;
    float timeout;
    rt_uint32_t counter;
    int i, index = 0;
    float tv_sec;
    float devi_min = 1;
    float devi;

    /* changed to second */
    overflow = timer->info->maxcnt / (float)timer->freq;
    tv_sec = tv->sec + tv->usec / (float)1000000;

    if (tv_sec < (1 / (float)timer->freq))
    {
        /* little timeout */
        i = 0;
        timeout = 1 / (float)timer->freq;
    }
    else
    {

        for (i = 1; i > 0; i ++)
        {
            timeout = tv_sec / i;

            if (timeout <= overflow)
            {
// Do not need to be such accurate, want to sleep longer each time.
#if 0
                counter = timeout * timer->freq;
                devi = tv_sec - (counter / (float)timer->freq) * i;
                /* Minimum calculation error */
                if (devi > devi_min)
                {
                    i = index;
                    timeout = tv_sec / i;
                    break;
                }
                else if (devi == 0)
                {
                    break;
                }
                else if (devi < devi_min)
                {
                    devi_min = devi;
                    index = i;
                }
#else
                break;
#endif
            }
        }
    }

    timer->cycles = i;
    timer->reload = i;
    timer->period_sec = timeout;
    counter = timeout * timer->freq;
    return counter;
}

static rt_err_t sifli_adc_use_gptime_init()
{

    /*Init gptime*/
#if BSP_GPADC_USE_GPTIME1
    {
        hw_gptimer.tim_handle.Instance     = GPTIM1;
        hw_gptimer.tim_irqn                = GPTIM1_IRQn;
        hw_gptimer.name                    = "gptim1";
        hw_gptimer.core                    = GPTIM1_CORE;
    }
#endif
#if BSP_GPADC_USE_GPTIME2
    {
        hw_gptimer.tim_handle.Instance     = GPTIM2;
        hw_gptimer.tim_irqn                = GPTIM2_IRQn;
        hw_gptimer.name                    = "gptim2";
        hw_gptimer.core                    = GPTIM2_CORE;
    }
#endif

#if BSP_GPADC_USE_GPTIME3
    {
        hw_gptimer.tim_handle.Instance     = GPTIM3;
        hw_gptimer.tim_irqn                = GPTIM3_IRQn;
        hw_gptimer.name                    = "gptim3";
        hw_gptimer.core                    = GPTIM3_CORE;
    }
#endif
#if BSP_GPADC_USE_GPTIME5
    {
        hw_gptimer.tim_handle.Instance     = GPTIM4;
        hw_gptimer.tim_irqn                = GPTIM4_IRQn;
        hw_gptimer.name                    = "gptim4";
        hw_gptimer.core                    = GPTIM4_CORE;
    }
#endif
#if BSP_GPADC_USE_GPTIME5

    {
        hw_gptimer.tim_handle.Instance     = GPTIM5;
        hw_gptimer.tim_irqn                = GPTIM5_IRQn;
        hw_gptimer.name                    = "gptim5";
        hw_gptimer.core                    = GPTIM5_CORE;
    }
#endif

    hw_gptimer.time_device.info = &hwtimer_info;


    if ((1000000 <= hw_gptimer.time_device.info->maxfreq) && (1000000 >= hw_gptimer.time_device.info->minfreq))
    {
        hw_gptimer.time_device.freq = 1000000;
    }
    else
    {
        hw_gptimer.time_device.freq = hw_gptimer.time_device.info->minfreq;
    }
    hw_gptimer.time_device.mode = HWTIMER_MODE_PERIOD;
    hw_gptimer.time_device.cycles = 0;
    hw_gptimer.time_device.overflow = 0;


    uint32_t  prescaler_value = HAL_RCC_GetPCLKFreq(
                                    hw_gptimer.core,
                                    1);

    prescaler_value = prescaler_value / 1000 - 1;
    tim.Init.Period            = 10000 - 1;
    tim.Init.Prescaler         = prescaler_value;
    tim.Instance = hw_gptimer.tim_handle.Instance;
    tim.core                   = hw_gptimer.core;
    tim.Init.CounterMode   = GPT_COUNTERMODE_UP;
    tim.Init.RepetitionCounter = 0;

    if (HAL_GPT_Base_Init(&tim) != HAL_OK)
    {
        LOG_E("%s init failed", hw_gptimer.name);
        return RT_ERROR;
    }
    else
    {
        /* set the TIMx priority */
        HAL_NVIC_SetPriority(hw_gptimer.tim_irqn, 3, 0);

        /* enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(hw_gptimer.tim_irqn);

        /* clear update flag */
        __HAL_GPT_CLEAR_FLAG(&tim, GPT_FLAG_UPDATE);
        /* enable update request source */
        __HAL_GPT_URS_ENABLE(&tim);
        tim.Instance->CR2 &= ~GPT_CR2_MMS;
        tim.Instance->CR2 = (0x2 << GPT_CR2_MMS_Pos);

        LOG_I("%s init success;\n", hw_gptimer.name);
    }

    /*update  prescaler*/
    rt_uint32_t val = 0;
    val = HAL_RCC_GetPCLKFreq(tim.core, 1);
    val /= 1000000;
    __HAL_GPT_SET_PRESCALER(&tim,  val - 1);
    tim.Instance->EGR |= GPT_EVENTSOURCE_UPDATE; /* Update frequency value */
    hw_gptimer.time_device.freq = 1000000;


    rt_hwtimerval_t t_value;
    t_value.sec = 0;
    t_value.usec = BSP_GPADC_GPTIME_PEROID;
    rt_uint32_t t = time_calc(&(hw_gptimer.time_device), &t_value);
    __HAL_GPT_SET_AUTORELOAD(&tim, t);

    /* set timer to Repetitive mode */
    tim.Instance->CR1 &= ~GPT_OPMODE_SINGLE;
    HAL_GPT_Base_Start_IT(&tim);

    return RT_EOK;
}

#if defined(hwp_gpadc1)

void GPADC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    sifli_adc_obj[0].ADC_Handler.Instance->GPADC_IRQ |= GPADC_GPADC_IRQ_GPADC_ICR;

    /*customer can add action after adc upadte*/
    if (sifli_adc_obj[0].sifli_adc_device.parent.rx_indicate)
    {
        sifli_adc_obj[0].sifli_adc_device.parent.rx_indicate(&sifli_adc_obj[0].sifli_adc_device.parent, 4);

    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#endif

/**
* @brief  Get voltage by register value.
* @param[in]  value register value.
* @retval voltage in mv.
*/
int sifli_adc_get_mv(uint32_t value)
{
    int offset, ratio;
    // get offset
    offset = adc_vol_offset;
    // get ratio, adc_vol_ratio calculate by calibration voltage
    if (adc_range == 0) // calibration with big range, app use small rage, need div 3
        ratio = adc_vol_ratio / 3;
    else // calibration and app all use small rage
        ratio = adc_vol_ratio;

    return ((int)value - offset) * ratio / ADC_RATIO_ACCURATE;
}

/**
* @brief  Get voltage by register value.
* @param[in]  value register value.
* @retval voltage based on mv.
*/
float sifli_adc_get_float_mv(float value)
{
    float offset, ratio;
    // get offset
    offset = adc_vol_offset;
    // get ratio, adc_vol_ratio calculate by calibration voltage
    if (adc_range == 0) // calibration with big range, app use small rage, need div 3
        ratio = adc_vol_ratio / 3;
    else // calibration and app all use small rage
        ratio = adc_vol_ratio;

    return (value - offset) * ratio / ADC_RATIO_ACCURATE;
}

/**
* @brief  Get voltage offset and ratio.
* @param[in]  value1 register value 1.
* @param[in]  value2 register value 2.
* @param[in]  vol1  voltage 1 in mv.
* @param[in]  vol2  voltage 2 in mv.
* @param[out]  offset, reg value offset vs 0v value.
* @param[out]  ratio, voltage (mv)per bit, ratio with 100 based or 0.01 mv, 1 base for 1 mv
* @retval offset.
*/
int sifli_adc_calibration(uint32_t value1, uint32_t value2,
                          uint32_t vol1, uint32_t vol2, float *offset, float *ratio)
{
    float gap1, gap2;
    uint32_t reg_max;

    if (offset == NULL || ratio == NULL)
        return 0;

    if (value1 == 0 || value2 == 0)
        return 0;

    gap1 = value1 > value2 ? value1 - value2 : value2 - value1; // register value gap
    gap2 = vol1 > vol2 ? vol1 - vol2 : vol2 - vol1; // voltage gap -- mv

    if (gap1 != 0)
    {
        *ratio = gap2 * ADC_RATIO_ACCURATE / gap1; // gap2 * 10 for 0.1mv, gap2 * 100 for 0.01mv
        adc_vol_ratio = *ratio;
    }
    else //
        return 0;

    *offset = value1 - vol1 * ADC_RATIO_ACCURATE / adc_vol_ratio;
    adc_vol_offset = *offset;

    // get register value for max voltage
    adc_thd_reg = ADC_MAX_VOLTAGE_MV * ADC_RATIO_ACCURATE / adc_vol_ratio + adc_vol_offset;
    reg_max = GPADC_ADC_RDATA0_SLOT0_RDATA >> GPADC_ADC_RDATA0_SLOT0_RDATA_Pos;
    if (adc_thd_reg >= (reg_max - 3))
        adc_thd_reg = reg_max - 3;

    return adc_vol_offset;
}

static void sifli_adc_vbat_fact_calib(uint32_t voltage, uint32_t reg)
{
    float vol_from_reg;

    // get voltage calculate by register data
    vol_from_reg = (reg - adc_vol_offset) * adc_vol_ratio / ADC_RATIO_ACCURATE;
    adc_vbat_factor = (float)voltage / vol_from_reg;
}


static rt_err_t sifli_adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
    ADC_HandleTypeDef *sifli_adc_handler = device->parent.user_data;

    RT_ASSERT(device != RT_NULL);

#ifndef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
    if (enabled)
    {
        HAL_ADC_EnableSlot(sifli_adc_handler, channel, 1);
    }
    else
    {
        HAL_ADC_EnableSlot(sifli_adc_handler, channel, 0);
    }
#endif
    return RT_EOK;
}

static rt_uint32_t sifli_adc_get_channel(rt_uint32_t channel)
{
    rt_uint32_t sifli_channel = 0;

    switch (channel)
    {
    case  0:
        sifli_channel = 0;
        break;
    case  1:
        sifli_channel = 1;
        break;
    case  2:
        sifli_channel = 2;
        break;
    case  3:
        sifli_channel = 3;
        break;
    case  4:
        sifli_channel = 4;
        break;
    case  5:
        sifli_channel = 5;
        break;
    case  6:
        sifli_channel = 6;
        break;
    case  7:
        sifli_channel = 7;
        break;
    }

    return sifli_channel;
}

static rt_err_t sifli_get_adc_value(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    HAL_StatusTypeDef res = 0;
    ADC_ChannelConfTypeDef ADC_ChanConf;
    ADC_HandleTypeDef *sifli_adc_handler = device->parent.user_data;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(value != RT_NULL);

    rt_err_t r = rt_sem_take(&gpadc_lock, rt_tick_from_millisecond(2000));
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
    uint32_t adc_origin = HAL_ADC_GetValue(sifli_adc_handler, channel);
    float fave = (float) adc_origin;

#else
    RT_ASSERT(RT_EOK == r);

    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));

    if (channel <= 7)
    {
        /* set ADC channel */
        ADC_ChanConf.Channel =  sifli_adc_get_channel(channel);
    }
    else
    {
        LOG_E("ADC channel must be between 0 and 7.");
        r = rt_sem_release(&gpadc_lock);
        RT_ASSERT(RT_EOK == r);
        return -RT_ERROR;
    }
    ADC_ChanConf.pchnl_sel = channel;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.nchnl_sel = 0;
    //ADC_ChanConf.acc_en = 0;
    ADC_ChanConf.acc_num = 0;   // remove hardware do multi point average
    HAL_ADC_ConfigChannel(sifli_adc_handler, &ADC_ChanConf);

    /* start ADC */
    HAL_ADC_Start(sifli_adc_handler);

    // do filter and average here
    int i, j;
    uint32_t data[ADC_SW_AVRA_CNT];
    uint32_t total, ave;
    float fave;
    total = 0;
    for (i = 0; i < ADC_SW_AVRA_CNT; i++)
    {
        if (i != 0)
        {
#ifndef  SF32LB55X
            // unmute before read adc
            ADC_SET_UNMUTE(sifli_adc_handler);
            HAL_Delay_us(200);
#else
            // FRC EN before each start
            ADC_FRC_EN(sifli_adc_handler);
            HAL_Delay_us(50);
#endif

            // triger sw start,
            __HAL_ADC_START_CONV(sifli_adc_handler);
        }

        /* Wait for the ADC to convert */
        res = HAL_ADC_PollForConversion(sifli_adc_handler, 100);
        if (res != HAL_OK)
        {
            HAL_ADC_Stop(sifli_adc_handler);
            LOG_E("Polling ADC fail %d\n", res);
            r = rt_sem_release(&gpadc_lock);
            RT_ASSERT(RT_EOK == r);
            return RT_ERROR;
        }

        /* get ADC value */
        data[i] = (rt_uint32_t)HAL_ADC_GetValue(sifli_adc_handler, channel);
        //rt_kprintf("ch[%d]count[%d] adc raw = %d;\n", channel, i,  data[i]);

        if (data[i] >= adc_thd_reg)
        {
            HAL_ADC_Stop(sifli_adc_handler);
            LOG_E("ADC input voltage too large, register value %d\n", data[i]);
            r = rt_sem_release(&gpadc_lock);
            *value = 50000; // output 5v as invalid voltage !
            return RT_ERROR;
        }
//        LOG_I("ADC reg value %d\n", data[i]);
        total += data[i];
        // Add a delay between each read to make voltage stable
#ifdef RT_USING_PM
        rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif

#ifndef  SF32LB55X
        ADC_SET_MUTE(sifli_adc_handler);
#ifdef SF32LB52X
        rt_thread_delay(1);
#else
        rt_thread_delay(10);
#endif
#else   /* SF32LB55X */
        ADC_CLR_FRC_EN(sifli_adc_handler);
        rt_thread_delay(5);
#endif

#ifdef RT_USING_PM
        rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif

    }
    HAL_ADC_Stop(sifli_adc_handler);

    // sort
    for (i = 0; i < ADC_SW_AVRA_CNT - 1; i++)
        for (j = 0; j < ADC_SW_AVRA_CNT - 1 - i; j++)
            if (data[j] > data[j + 1])
            {
                ave = data[j];
                data[j] = data[j + 1];
                data[j + 1] = ave;
            }
    // drop max/min , mid filter
    total -= data[0];
    total -= data[ADC_SW_AVRA_CNT - 1];

    fave = (float)total / (ADC_SW_AVRA_CNT - 2);
    //rt_kprintf("ch[%d]average val =%f;\n", channel, fave);

#endif


#ifndef SF32LB52X   // TODO: remove macro check after 52x ADC calibration work
    float fval = sifli_adc_get_float_mv(fave) * 10; // mv to 0.1mv based
    *value = (rt_uint32_t)fval;
#else
    //*value = (rt_uint32_t)fave;
    float fval = sifli_adc_get_float_mv(fave) * 10; // mv to 0.1mv based
    *value = (rt_uint32_t)fval;
    if (channel == 7)   // for 52x, channel fix used for vbat with 1/2 update(need calibrate)
        *value = (rt_uint32_t)(fval * adc_vbat_factor);
#endif
    //LOG_I("ADC vol %d , reg %f, max %d, min %d\n", *value, fave, data[ADC_SW_AVRA_CNT - 1], data[0]);
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
    rt_kprintf("ch[%d]origin:%d, voltage:%d;\n", channel, adc_origin, *value);
#else
    rt_kprintf("ch[%d]voltage=%d;\n", channel, *value);
#endif
    r = rt_sem_release(&gpadc_lock);
    RT_ASSERT(RT_EOK == r);

    return RT_EOK;
}

static rt_err_t sifli_op_adc_init(struct rt_adc_device *device)
{
    ADC_HandleTypeDef *sifli_adc_handler = device->parent.user_data;
    HAL_StatusTypeDef status;

    status = HAL_ADC_DeInit(sifli_adc_handler);
    if (HAL_OK != status)
    {
        return RT_ERROR;
    }
    status = HAL_ADC_Init(sifli_adc_handler);
    if (HAL_OK == status)
    {
#ifdef SF32LB52X
        uint32_t adc_freq = 240000; // use 240k for 52x to meet ATE setting
        HAL_ADC_SetFreq(sifli_adc_handler, adc_freq);
#endif
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
        {
            ADC_ChannelConfTypeDef ADC_ChanConf;

            /*configure all channels*/
#ifdef SF32LB52X
            uint8_t ch_num = 7;
#else
            uint8_t ch_num = 8;
#endif
            rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
            HAL_ADC_Set_MultiMode(sifli_adc_handler, 1);
            for (uint8_t ch = 0; ch < ch_num; ch++)
            {
                ADC_ChanConf.Channel = ch;
                ADC_ChanConf.pchnl_sel = ch;
                ADC_ChanConf.slot_en = 1;
                ADC_ChanConf.nchnl_sel = 0;
                ADC_ChanConf.acc_num = 0;   /* remove hardware do multi point average*/
                HAL_ADC_ConfigChannel(sifli_adc_handler, &ADC_ChanConf);
            }
            HAL_ADC_Prepare(sifli_adc_handler);

            sifli_adc_use_gptime_init();

            /*set timer trigger src to gptimer 1*/
            HAL_ADC_SetTimer(sifli_adc_handler, HAL_ADC_SRC_GPTIM1);

            HAL_NVIC_EnableIRQ(GPADC_IRQn);
            ADC_ENABLE_TIMER_TRIGER(sifli_adc_handler);

        }
#endif
        return RT_EOK;
    }
    else
    {
        return RT_ERROR;
    }
}

static rt_err_t sifli_adc_control(struct rt_adc_device *device, rt_uint32_t cmd, void *arg)
{
    rt_adc_cmd_read_arg_t *read_arg = (rt_adc_cmd_read_arg_t *)arg;
    HAL_StatusTypeDef res = 0;
    ADC_ChannelConfTypeDef ADC_ChanConf;
    ADC_HandleTypeDef *sifli_adc_handler = device->parent.user_data;
    rt_uint32_t channel;
    uint32_t data;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(RT_ADC_CMD_READ == cmd);
    RT_ASSERT(read_arg);

    rt_err_t r = rt_sem_take(&gpadc_lock, rt_tick_from_millisecond(2000));
    RT_ASSERT(RT_EOK == r);

#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
    data = HAL_ADC_GetValue(sifli_adc_handler, read_arg->channel);
#else
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));

    channel = read_arg->channel;
    if (channel <= 7)
    {
        /* set ADC channel */
        ADC_ChanConf.Channel =  sifli_adc_get_channel(channel);
    }
    else
    {
        LOG_E("ADC channel must be between 0 and 7.");
        r = rt_sem_release(&gpadc_lock);
        RT_ASSERT(RT_EOK == r);
        return -RT_ERROR;
    }
    ADC_ChanConf.pchnl_sel = channel;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.nchnl_sel = 0;
    //ADC_ChanConf.acc_en = 0;
    ADC_ChanConf.acc_num = 0;   // remove hardware do multi point average
    HAL_ADC_ConfigChannel(sifli_adc_handler, &ADC_ChanConf);

    /* start ADC */
    HAL_ADC_Start(sifli_adc_handler);

    /* Wait for the ADC to convert */
    res = HAL_ADC_PollForConversion(sifli_adc_handler, 100);
    if (res != HAL_OK)
    {
        HAL_ADC_Stop(sifli_adc_handler);
        LOG_E("Polling ADC fail %d\n", res);
        r = rt_sem_release(&gpadc_lock);
        RT_ASSERT(RT_EOK == r);
        return RT_ERROR;
    }

    /* get ADC value */
    data = (rt_uint32_t)HAL_ADC_GetValue(sifli_adc_handler, channel);

    if (data >= adc_thd_reg)
    {
        HAL_ADC_Stop(sifli_adc_handler);
        LOG_E("ADC input voltage too large, register value %d\n", data);
        r = rt_sem_release(&gpadc_lock);
        read_arg->value = 50000; // output 5v as invalid voltage !
        return RT_ERROR;
    }

    HAL_ADC_Stop(sifli_adc_handler);
#endif

    read_arg->value = (rt_uint32_t)(sifli_adc_get_float_mv((float)data) * 10);   // based on 0.1 mv for more accurate
    rt_kprintf("adc control origin data %d, Voltage %d\n", data, read_arg->value);

    r = rt_sem_release(&gpadc_lock);
    RT_ASSERT(RT_EOK == r);

    return RT_EOK;
}

static const struct rt_adc_ops sifli_adc_ops =
{
    .enabled = sifli_adc_enabled,
    .convert = sifli_get_adc_value,
    .init = sifli_op_adc_init,
    .control = sifli_adc_control,
};

static int sifli_adc_init(void)
{
    int result = RT_EOK;
    /* save adc name */
    char name_buf[5] = {'b', 'a', 't', '0', 0};
    int i = 0;

    for (i = 0; i < sizeof(adc_config) / sizeof(adc_config[0]); i++)
    {
        /* ADC init */
        name_buf[3] = '0';
        sifli_adc_obj[i].ADC_Handler = adc_config[i];
#if defined(hwp_gpadc1)
        if (sifli_adc_obj[i].ADC_Handler.Instance == hwp_gpadc1)
        {
            name_buf[3] = '1';
            ADC_Handler_gpadc1 = &sifli_adc_obj[i].ADC_Handler;
        }
#endif
#if defined(hwp_gpadc2)
        if (sifli_adc_obj[i].ADC_Handler.Instance == hwp_gpadc2)
        {
            name_buf[3] = '2';
            ADC_Handler_gpadc2 = &sifli_adc_obj[i].ADC_Handler;
        }
#endif
#ifdef BSP_GPADC_USING_DMA
        ADC_HandleTypeDef *hadc = &sifli_adc_obj[i].ADC_Handler;
        hadc->DMA_Handle = (DMA_HandleTypeDef *)malloc(sizeof(DMA_HandleTypeDef));
        if (hadc->DMA_Handle != NULL)
        {
            memset((void *)hadc->DMA_Handle, 0, sizeof(DMA_HandleTypeDef));
            hadc->DMA_Handle->Instance                 = GPADC_DMA_INSTANCE;
            hadc->DMA_Handle->Init.Request             = GPADC_DMA_REQUEST;
            hadc->DMA_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
            hadc->DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
            hadc->DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
            hadc->DMA_Handle->Init.PeriphInc           = DMA_PINC_DISABLE;
            hadc->DMA_Handle->Init.MemInc              = DMA_MINC_ENABLE;
            hadc->DMA_Handle->Init.Mode                = DMA_NORMAL;
            hadc->DMA_Handle->Init.Priority            = DMA_PRIORITY_MEDIUM;
        }
#endif
#ifndef SF32LB55X
#ifdef BSP_AVDD_V18_ENABLE
        sifli_adc_obj[i].ADC_Handler.Init.avdd_v18_en = 1;
#else
        sifli_adc_obj[i].ADC_Handler.Init.avdd_v18_en = 0;
#endif
#endif
        if (HAL_ADC_Init(&sifli_adc_obj[i].ADC_Handler) != HAL_OK)
        {
            LOG_E("%s init failed", name_buf);
            result = -RT_ERROR;
        }
        else
        {
#ifdef SF32LB52X
            uint32_t adc_freq = 240000; // use 240k for 52x to meet ATE setting
            HAL_ADC_SetFreq(&sifli_adc_obj[i].ADC_Handler, adc_freq);
#endif


#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
            {
                ADC_ChannelConfTypeDef ADC_ChanConf;

                /*configure all channels*/

                uint8_t ch_num = 8;

                rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
                HAL_ADC_Set_MultiMode(&sifli_adc_obj[i].ADC_Handler, 1);
                for (uint8_t ch = 0; ch < ch_num; ch++)
                {
                    ADC_ChanConf.Channel = ch;
                    ADC_ChanConf.pchnl_sel = ch;
                    ADC_ChanConf.slot_en = 1;
                    ADC_ChanConf.nchnl_sel = 0;
                    ADC_ChanConf.acc_num = 0;   /* remove hardware do multi point average*/
                    HAL_ADC_ConfigChannel(&sifli_adc_obj[i].ADC_Handler, &ADC_ChanConf);
                }

                HAL_ADC_Prepare(&sifli_adc_obj[i].ADC_Handler);

                /*set timer trigger src to gptimer 1*/
                HAL_ADC_SetTimer(&sifli_adc_obj[i].ADC_Handler, HAL_ADC_SRC_GPTIM1);
                sifli_adc_use_gptime_init();

                HAL_NVIC_EnableIRQ(GPADC_IRQn);
                ADC_ENABLE_TIMER_TRIGER(&sifli_adc_obj[i].ADC_Handler);
            }
#endif
            //adc_vol_offset = HAL_ADC_Get_Offset(&sifli_adc_obj[i].ADC_Handler);
            //LOG_I("ADC OFFSET = %d\n",adc_vol_offset);
            /* register ADC device */
            if (rt_hw_adc_register(&sifli_adc_obj[i].sifli_adc_device, name_buf, &sifli_adc_ops, &sifli_adc_obj[i].ADC_Handler) == RT_EOK)
            {
                //LOG_D("%s init success", name_buf);
            }
            else
            {
                LOG_E("%s register failed", name_buf);
                result = -RT_ERROR;
            }
        }
    }
    rt_sem_init(&gpadc_lock, "gpadc", 1, RT_IPC_FLAG_FIFO);

    // set default adc thd to register max value
    adc_thd_reg = GPADC_ADC_RDATA0_SLOT0_RDATA >> GPADC_ADC_RDATA0_SLOT0_RDATA_Pos;


    FACTORY_CFG_ADC_T cfg;
    int len = sizeof(FACTORY_CFG_ADC_T);
    rt_memset((uint8_t *)&cfg, 0, len);
    if (BSP_CONFIG_get(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, len))  // TODO: configure read ADC parameters method after ATE confirm
    {
        float off, rat;
        uint32_t vol1, vol2;
        if (cfg.vol10 == 0 || cfg.vol25 == 0) // not valid paramter
        {
            //LOG_I("Get GPADC configure invalid : %d, %d\n", cfg.vol10, cfg.vol25);
        }
        else
        {
#ifndef SF32LB55X
            cfg.vol10 &= 0x7fff;
            cfg.vol25 &= 0x7fff;
            vol1 = cfg.low_mv;
            vol2 = cfg.high_mv;
            adc_range = 1;
#else
            if ((cfg.vol10 & (1 << 15)) && (cfg.vol25 & (1 << 15))) // small range, use X1 mode
            {
                cfg.vol10 &= 0x7fff;
                cfg.vol25 &= 0x7fff;
                vol1 = ADC_SML_RANGE_VOL1;
                vol2 = ADC_SML_RANGE_VOL2;
                adc_range = 1;
            }
            else // big range , use X3 mode for A0
            {
                vol1 = ADC_BIG_RANGE_VOL1;
                vol2 = ADC_BIG_RANGE_VOL2;
                adc_range = 0;
            }
#endif
            sifli_adc_calibration(cfg.vol10, cfg.vol25, vol1, vol2, &off, &rat);
#ifdef SF32LB52X
            sifli_adc_vbat_fact_calib(cfg.vbat_mv, cfg.vbat_reg);

            if (SF32LB52X_LETTER_SERIES())
            {
#if defined(hwp_gpadc1)
                if (ADC_Handler_gpadc1)
                {
                    if (cfg.ldovref_flag)
                    {
                        __HAL_ADC_SET_LDO_REF_SEL(ADC_Handler_gpadc1, cfg.ldovref_sel);
                    }

                }
#endif
            }
#endif
            //LOG_I("\nGPADC :vol10: %d mv, %d; vol25: %d mv reg %d; offset %f, ratio %f, max reg %d;\n",
            //      vol1, cfg.vol10, vol2, cfg.vol25,  off, rat, adc_thd_reg);
            //LOG_I("\n vbat_mv: %d mv, %d; ldoref_flag = %d, ldoref_sel = %d;\n",
            //      cfg.vbat_mv, cfg.vbat_reg, cfg.ldovref_flag, cfg.ldovref_sel);

        }
    }
    else
    {
        LOG_I("Get ADC configure fail\n");
    }
//#endif
    return result;
}
INIT_BOARD_EXPORT(sifli_adc_init);

#ifdef RT_USING_PM
static int rt_adc_freq_chg(const struct rt_device *device, uint8_t mode)
{
    uint32_t adc_freq = 240000; // use 240k for 52x to meet ATE setting
    if (sizeof(sifli_adc_obj) > 0)
        HAL_ADC_SetFreq(&sifli_adc_obj[0].ADC_Handler, adc_freq);
    //rt_kprintf("OP changed with adc freq update : data_samp_delay %d, conv_width %d, sample_width %d\n",
    //sifli_adc_obj[0].ADC_Handler.Init.data_samp_delay , sifli_adc_obj[0].ADC_Handler.Init.conv_width, sifli_adc_obj[0].ADC_Handler.Init.sample_width);
    return 0;
}

static const struct rt_device_pm_ops adc_pm_op =
{
    .suspend = NULL,
    .resume = NULL,
    .frequency_change = rt_adc_freq_chg,
};

static int sifli_adc_pm_register(void)
{
    rt_pm_device_register(NULL, &adc_pm_op);
    return 0;
}
INIT_ENV_EXPORT(sifli_adc_pm_register);

#endif  /* RT_USING_PM */


#ifdef RT_USING_FINSH
//#define DRV_GPADC_TEST
#ifdef DRV_GPADC_TEST

#include <string.h>

#ifdef BSP_USING_SPI_FLASH
    #include "drv_flash.h"
#endif

#define TEST_STABLE_LOOP        (3)
static int test_adc10 = 0;
static int test_adc25 = 0;
static int rgap = 0;
static int st_mode = 1; // 0 for start/stop for each read, 1 for start /stop once, read many times
static int mid_value = 0;   // 0 using average register value for voltage, 1 use middle value for voltage

typedef struct
{
    int pin_num;
    int chnl;
} _GPADC_TEST_PIN_CHN_T;

static rt_event_t g_adc_key_ev;
static _GPADC_TEST_PIN_CHN_T pincfg;

static int adc_manual_cal(int reg1000, int reg2500)
{
    float off, rat;
    int res = 0;
    res = sifli_adc_calibration((uint32_t)reg1000, (uint32_t)reg2500, 1000, 2500, &off, &rat);
    if (res != 0)
    {
        rt_kprintf("Calib offset %f, rat %f\n", off, rat);
    }
    else
        rt_kprintf("calib fail\n");

    return res;
}

static int adc_calib_func(int loop)
{
    int i, j;
    uint32_t *data;
    uint32_t total, ave;
    float fave;
    HAL_StatusTypeDef res = 0;
    ADC_ChannelConfTypeDef ADC_ChanConf;
    ADC_HandleTypeDef *sifli_adc_handler = &sifli_adc_obj[i].ADC_Handler;
    int channel = 1;

#ifdef  SF32LB55X
    channel = 1;
#elif defined (SF32LB56X)
    channel = 5;
#elif defined (SF32LB58X)
    channel = 3;
#elif defined (SF32LB52X)
    channel = 7;
#endif

    if (loop > 1024)
    {
        rt_kprintf("too many counter %d larger than 1024\n", loop);
        return -1;
    }
    data = (uint32_t *)malloc(loop * 4);
    if (data == NULL)
    {
        rt_kprintf("Malloc memory fail\n");
        return -1;
    }
    rt_kprintf("Test adc with CHN %d, loop %d, delay %d us \n", channel, loop, rgap);

    //HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLDOWN, 0);

    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));

    /* set ADC channel */
    ADC_ChanConf.Channel =  sifli_adc_get_channel(channel);
    ADC_ChanConf.pchnl_sel = channel;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.nchnl_sel = 0;
    ADC_ChanConf.acc_num = 0; //hw_ave;   // disable hardware average
    HAL_ADC_ConfigChannel(sifli_adc_handler, &ADC_ChanConf);

    // do filter and average here
    total = 0;
    if (st_mode == 0)
    {
        for (i = 0; i < loop; i++)
        {
            /* start ADC */
            HAL_ADC_Start(sifli_adc_handler);
            // triger sw start,
            __HAL_ADC_START_CONV(sifli_adc_handler);

            /* Wait for the ADC to convert */
            res = HAL_ADC_PollForConversion(sifli_adc_handler, 100);
            if (res != HAL_OK)
            {
                HAL_ADC_Stop(sifli_adc_handler);
                rt_kprintf("Polling ADC fail %d\n", res);
                return -1;
            }

            /* get ADC value */
            data[i] = (rt_uint32_t)HAL_ADC_GetValue(sifli_adc_handler, channel);

            HAL_ADC_Stop(sifli_adc_handler);
            //rt_kprintf("%d\n", data[i]);
            //total += data[i];
            if (rgap != 0)
                HAL_Delay_us(rgap);

        }
    }
    else
    {
        /* start ADC */
        HAL_ADC_Start(sifli_adc_handler);
        for (i = 0; i < loop; i++)
        {
            if (i != 0)
            {
#ifndef  SF32LB55X
                // unmute before read adc
                ADC_SET_UNMUTE(sifli_adc_handler);
                HAL_Delay_us(200);
#else
                // FRC EN before each start
                ADC_FRC_EN(sifli_adc_handler);
                HAL_Delay_us(50);
#endif
                // triger sw start,
                __HAL_ADC_START_CONV(sifli_adc_handler);
            }
            /* Wait for the ADC to convert */
            res = HAL_ADC_PollForConversion(sifli_adc_handler, 100);
            if (res != HAL_OK)
            {
                HAL_ADC_Stop(sifli_adc_handler);
                rt_kprintf("Polling ADC fail %d\n", res);
                return -1;
            }

            /* get ADC value */
            data[i] = (rt_uint32_t)HAL_ADC_GetValue(sifli_adc_handler, channel);
#ifndef  SF32LB55X
            ADC_SET_MUTE(sifli_adc_handler);
#else   /* SF32LB55X */
            ADC_CLR_FRC_EN(sifli_adc_handler);
#endif
            //rt_kprintf("%d\n", data[i]);
            //total += data[i];
            if (rgap != 0)
                HAL_Delay_us(rgap);

        }
        HAL_ADC_Stop(sifli_adc_handler);
    }

#if 1
    for (i = 0; i < loop; i++)
    {
        total += data[i];
        rt_kprintf("%d\n", data[i]);
    }
    // sort
    for (i = 0; i < loop - 1; i++)
        for (j = 0; j < loop - 1 - i; j++)
            if (data[j] > data[j + 1])
            {
                ave = data[j];
                data[j] = data[j + 1];
                data[j + 1] = ave;
            }
    // drop max/min , mid filter
    total -= data[0];
    total -= data[loop - 1];
    rt_kprintf("Drop %d and %d, Data after sort:\n", data[0], data[loop - 1]);
    //for (i = 1; i < loop - 1; i++)
    //{
    //rt_kprintf("%d\n", data[i]);
    //}

    ave = data[loop / 2];
    fave = (float)total / (loop - 2);
    float volt = sifli_adc_get_float_mv(fave);
    int v2 = sifli_adc_get_mv(ave);
    if (mid_value == 0)
    {
        //volt = volt * (1000 + 470) / 1000;
        //volt = volt * (1000 + 220) / 220;
        rt_kprintf("AVERAGE %.6f, voltage %.6f mv\n", fave, volt);
    }
    else
    {
        //v2 = v2 * (1000 + 470) / 1000;
        //v2 = v2 * (1000 + 220) / 220;
        rt_kprintf("Mid %d, voltage %d mv\n", ave, v2);
    }
#if 0
    FACTORY_CFG_BATTERY_CALI_T battery_cfg;
    res = rt_flash_config_read(FACTORY_CFG_ID_BATTERY, (uint8_t *)&battery_cfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
    if (res <= 0)
    {
        rt_kprintf("FACTORY_CFG_ID_BATTERY read fail with %d\n", res);
    }
    else
    {
        rt_kprintf("BAT a %d, b %d, magic 0x%x\n", battery_cfg.a, battery_cfg.b, battery_cfg.magic);
    }

    if (mid_value == 0)
    {
        volt = volt * battery_cfg.a / 10000;
        rt_kprintf("Final output vol %.6f mv\n", volt);
    }
    else
    {
        v2 = v2 * battery_cfg.a / 10000;
        rt_kprintf("Final output vol %d mv\n", volt);
    }
#endif
#else
    for (i = 0; i < loop; i++)
    {
        total += data[i];
        rt_kprintf("%d\n", data[i]);
    }

    ave = total / loop;
    fave = (float)total / loop;
    float volt = sifli_adc_get_float_mv(fave);
    volt = volt * (1000 + 220) / 220;
    rt_kprintf("AVERAGE %d, %.6f, voltage %.6f mv\n", ave, fave, volt);
#endif

    return 0;

}

static void key_int_test(void *args)
{
    int value = (int)args;
    rt_pin_irq_enable(value, 0);
    rt_event_send(g_adc_key_ev, 1);

    rt_kprintf("key_int_test irq pin %d\n", value);
}

static void test_adc_key_thread_entry(void *p)
{
    rt_uint32_t evt;
    rt_device_t dev;
    uint32_t value;
    int res;
    _GPADC_TEST_PIN_CHN_T *pin_cfg = (_GPADC_TEST_PIN_CHN_T *)p;

    // set pin to adc, with gpio mode
    rt_kprintf("Initial pin %d\n", pin_cfg->pin_num);
    rt_pin_mode(pin_cfg->pin_num, PIN_MODE_INPUT);
    rt_pin_attach_irq(pin_cfg->pin_num, PIN_IRQ_MODE_RISING, key_int_test, (void *)(pin_cfg->pin_num));
    rt_pin_irq_enable(pin_cfg->pin_num, 1);

    dev = rt_device_find("bat1");
    if (dev == NULL)
    {
        rt_kprintf("Find bat1 device fail\n");
        RT_ASSERT(0);
    }
    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
    rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)(pin_cfg->chnl));

    g_adc_key_ev = rt_event_create("adc_key_evt", RT_IPC_FLAG_FIFO);

    while (1)
    {
        rt_event_recv(g_adc_key_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

        // set to analog pin
        rt_kprintf("trt to read adc\n");
        HAL_PIN_Set_Analog(PAD_PA00 + pin_cfg->pin_num, 1);
        HAL_Delay(300);
        res = rt_device_read(dev, pin_cfg->chnl, &value, 1);
        rt_kprintf("Read ADC channel%d : %d, res = %d \n", pin_cfg->chnl, value, res);
        rt_thread_delay(500);

        // set to input gpio again
        rt_pin_mode(pin_cfg->pin_num, PIN_MODE_INPUT);
        rt_pin_attach_irq(pin_cfg->pin_num, PIN_IRQ_MODE_RISING, key_int_test, (void *)34);
        rt_pin_irq_enable(pin_cfg->pin_num, 1);
    }
}

static int test_adc_key(int pin_num, int chnl)
{
    rt_thread_t task_handle;
    pincfg.chnl = chnl;
    pincfg.pin_num = pin_num;
    task_handle = rt_thread_create("test_adc_key", test_adc_key_thread_entry, &pincfg, 2048, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT * 2);
    rt_thread_startup(task_handle);
    return 0;
}

int cmd_gpadc(int argc, char *argv[])
{
    rt_device_t dev;
    int i;

    if (strcmp(argv[1], "-enable") == 0)
    {
        uint32_t chnl = atoi(argv[3]);
        dev = rt_device_find(argv[2]);
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)chnl);
            //LOG_I("ADC %s channel %d enabled\n", argv[2], chnl);
        }
        else
        {
            LOG_I("Find %s device fail\n", argv[2]);
        }
    }
    else if (strcmp(argv[1], "-disable") == 0)
    {
        uint32_t chnl = atoi(argv[3]);
        dev = rt_device_find(argv[2]);
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            rt_device_control(dev, RT_ADC_CMD_DISABLE, (void *)chnl);
            LOG_I("ADC channel %d disabled\n", chnl);
        }
        else
        {
            LOG_I("Find adc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        uint32_t chnl = atoi(argv[3]);
        uint32_t value, res;
        dev = rt_device_find(argv[2]);
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_read(dev, chnl, &value, 1);
            LOG_I("Read ADC channel %d : %d, res = %d \n", chnl, value, res);
        }
        else
        {
            LOG_I("Find adc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-list") == 0)
    {
        LOG_I("Offset = %f, ratio = %f\n", adc_vol_offset, adc_vol_ratio);
#ifdef SF32LB52X
        LOG_I("vbat factor = %f\n", adc_vbat_factor);
#endif
    }
    else if (strcmp(argv[1], "-list2") == 0)
    {
        HAL_LCPU_CONFIG_ADC_T cfg;
        int len = (int)sizeof(HAL_LCPU_CONFIG_ADC_T);
#ifndef SF32LB52X
        if (BSP_CONFIG_get(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, len))
#else
        if (0)
#endif
        {
            LOG_I("ADC : small VOL %d, big VOL %d, X1 mode %d\n", cfg.vol10 & 0xfff, cfg.vol25 & 0xfff, cfg.vol10 >> 15);
        }
        else
        {
            LOG_I("Get ADC configure fail\n");
        }
    }
#ifdef ADC_DEBUG
    else if (strcmp(argv[1], "-vol") == 0) // only used for read return register value, not voltage !!!
    {
        uint32_t chnl = atoi(argv[3]);
        uint32_t value, res;
        dev = rt_device_find(argv[2]);
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_read(dev, chnl, &value, 1);
            LOG_I("Read ADC channel %d : %d, res = %d \n", chnl, value, res);
            int vol = sifli_adc_get_mv(value);
            LOG_I("Calculate votage %d mv\n", vol);
        }
        else
        {
            LOG_I("Find adc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-rv10") == 0) // for X3 mode, input 1.0v, for X1 mode, input 300mv
    {
        uint32_t chnl = atoi(argv[3]);
        uint32_t value, res;
        dev = rt_device_find(argv[2]);
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            for (i = 0; i < TEST_STABLE_LOOP; i++)
                res = rt_device_read(dev, chnl, &value, 1);
            LOG_I("Read ADC channel %d : %d, res = %d \n", chnl, value, res);
            test_adc10 = value;
        }
        else
        {
            LOG_I("Find adc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-rv25") == 0) // for X3 mode, input 2.5v, for X1 mode, input 800mv
    {
        uint32_t chnl = atoi(argv[3]);
        uint32_t value, res;
        dev = rt_device_find(argv[2]);
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            for (i = 0; i < TEST_STABLE_LOOP; i++)
                res = rt_device_read(dev, chnl, &value, 1);
            LOG_I("Read ADC channel %d : %d, res = %d \n", chnl, value, res);
            test_adc25 = value;
        }
        else
        {
            LOG_I("Find adc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-otpx3") == 0)
    {
#ifdef BSP_USING_SPI_FLASH

        FACTORY_CFG_ADC_T adc_cfg;
        FACTORY_CFG_ADC_T adc_ret;

        if (test_adc25 == 0 || test_adc10 == 0)
        {
            LOG_I("Invalid value v1.0 %d, v2.5 %d\n", test_adc10, test_adc25);
            return 0;
        }
        adc_cfg.vol25 = test_adc25;
        adc_cfg.vol10 = test_adc10;
        int res = rt_flash_config_write(FACTORY_CFG_ID_ADC, (uint8_t *)&adc_cfg, sizeof(adc_cfg));
        if (res <= 0)
        {
            LOG_I("FACTORY_CFG_ID_ADC write fail with %d\n", res);
        }
        else
            LOG_I("CFG ADC 1.0v with %d, 2.5v with %d, res %d\n", adc_cfg.vol10, adc_cfg.vol25, res);

        res = rt_flash_config_read(FACTORY_CFG_ID_ADC, (uint8_t *)&adc_ret, sizeof(adc_ret));
        if (res <= 0)
        {
            LOG_I("FACTORY_CFG_ID_ADC write fail with %d\n", res);
        }
        else
        {
            LOG_I("Get 1.0v reg %d, 2.5v %d, res = %d\n", adc_ret.vol10, adc_ret.vol25, res);
        }
        if (adc_ret.vol10 != adc_cfg.vol10 || adc_ret.vol25 != adc_cfg.vol25)
            LOG_I("Config ADC fail\n");
        else
            LOG_I("Config ADC success\n");

#endif
    }
    else if (strcmp(argv[1], "-otpx1") == 0)
    {
#ifdef BSP_USING_SPI_FLASH

        FACTORY_CFG_ADC_T adc_cfg;
        FACTORY_CFG_ADC_T adc_ret;

        if (test_adc25 == 0 || test_adc10 == 0)
        {
            LOG_I("Invalid value 300mv %d, 800mv %d\n", test_adc10, test_adc25);
            return 0;
        }
        adc_cfg.vol25 = test_adc25 | (1 << 15);
        adc_cfg.vol10 = test_adc10 | (1 << 15);
        int res = rt_flash_config_write(FACTORY_CFG_ID_ADC, (uint8_t *)&adc_cfg, sizeof(adc_cfg));
        if (res <= 0)
        {
            LOG_I("FACTORY_CFG_ID_ADC write fail with %d\n", res);
        }
        else
            LOG_I("CFG ADC 300mv with %d, 800 mv with %d, res %d\n", adc_cfg.vol10, adc_cfg.vol25, res);

        res = rt_flash_config_read(FACTORY_CFG_ID_ADC, (uint8_t *)&adc_ret, sizeof(adc_ret));
        if (res <= 0)
        {
            LOG_I("FACTORY_CFG_ID_ADC write fail with %d\n", res);
        }
        else
        {
            LOG_I("Get 300 mv reg %d, 800 mv %d, res = %d, X1 range %d\n", adc_ret.vol10 & 0X7FFF, adc_ret.vol25 & 0X7FFF, res, adc_ret.vol10 >> 15);
        }
        if (adc_ret.vol10 != adc_cfg.vol10 || adc_ret.vol25 != adc_cfg.vol25)
            LOG_I("Config ADC fail\n");
        else
            LOG_I("Config ADC success\n");

#endif
    }
#endif
    else if (strcmp(argv[1], "-lt") == 0)
    {
        int cnt = atoi(argv[2]);
        int res = adc_calib_func(cnt);
        rt_kprintf("ADC loop test done with result %d\n", res);
    }
    else if (strcmp(argv[1], "-sgap") == 0)
    {
        int delay = atoi(argv[2]);
        rt_kprintf("old gap between 2 read %d us\n", rgap);
        rgap = delay;
        rt_kprintf("new gap between 2 read %d us\n", rgap);
    }
    else if (strcmp(argv[1], "-mode") == 0)
    {
        int mode = atoi(argv[2]);
        st_mode = mode;
        if (st_mode == 0)
            rt_kprintf("Mode %d : Start/Stop for each adc read\n", st_mode);
        else
            rt_kprintf("Mode %d: Start/Stop once, read many time between them\n", st_mode);
    }
    else if (strcmp(argv[1], "-usemid") == 0)
    {
        int mid = atoi(argv[2]);
        rt_kprintf("old mid value flag %d\n", mid_value);
        mid_value = mid;
        rt_kprintf("new  mid value flag %d\n", mid_value);
    }
    else if (strcmp(argv[1], "-lt2") == 0)
    {
        int cnt = atoi(argv[2]);
        int cnt2 = atoi(argv[3]);
        int res, i;
        for (i = 0; i < cnt2; i++)
        {
            res = adc_calib_func(cnt);
            rt_kprintf("ADC loop %d with result %d\n", i, res);
            rt_thread_delay(1000);
        }
    }
    else if (strcmp(argv[1], "-set_cal") == 0)
    {
        int reg10 = atoi(argv[2]);
        int reg25 = atoi(argv[3]);
        adc_manual_cal(reg10, reg25);
    }
    else if (strcmp(argv[1], "-adc_key") == 0)
    {
        int pin_num = atoi(argv[2]);
        int chnl = atoi(argv[3]);
        test_adc_key(pin_num, chnl);
    }
    else
    {
        //LOG_I("Invalid parameter\n");
        //LOG_I("gpadc -enable/-read/-close bat1/bat2 channel\n");
        //LOG_I("example: gpadc -vol  bat1 1\n");
        //LOG_I("example: gpadc -read bat1 1\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_gpadc, __cmd_gpadc, Test gpadc driver);

#endif /* DRV_GPADC_TEST */

#endif /* finsh */

#endif /* BSP_USING_ADC */

/// @} drv_lcpu
/// @} bsp_driver

/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
