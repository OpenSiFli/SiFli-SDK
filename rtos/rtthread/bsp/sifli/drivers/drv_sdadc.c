/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <board.h>

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_sdadc SDADC
  * @brief SDADC BSP driver
  * @{
  */

#if defined(BSP_USING_SDADC)
#include "drv_config.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.sdadc"
#include <drv_log.h>

struct sifli_sdadc
{
    SDADC_HandleTypeDef SDADC_Handler;
    struct rt_adc_device sifli_sdadc_device;
};

static struct sifli_sdadc sifli_sdadc_obj;

static HAL_SDADC_CalibContextTypeDef g_sdadc_calib_ctx;
static struct rt_semaphore g_sdadc_lock;

#ifdef BSP_SDADC_USE_TIMER_TRIGGER

/* Number of SDADC channels reported to the rx_indicate callback on each
 * timer-triggered conversion complete interrupt (CHN0..CHN3). */
#define SDADC_RX_INDICATE_SIZE      (HAL_SDADC_GPIO_CHN3 - HAL_SDADC_GPIO_CHN0 + 1)

static GPT_HandleTypeDef g_sdadc_gptim;

void SDADC_IRQHandler(void)
{
    rt_interrupt_enter();

    /* Clear interrupt */
    sifli_sdadc_obj.SDADC_Handler.Instance->INT_CLR = 1;

    /* Notify waiting thread */
    if (sifli_sdadc_obj.sifli_sdadc_device.parent.rx_indicate)
        sifli_sdadc_obj.sifli_sdadc_device.parent.rx_indicate(
            &sifli_sdadc_obj.sifli_sdadc_device.parent, SDADC_RX_INDICATE_SIZE);

    rt_interrupt_leave();
}

static void sifli_sdadc_use_gptime_init(void)
{
    uint32_t prescaler;

    g_sdadc_gptim.Instance = GPTIM3;

#if defined(SF32LB58X)
    prescaler = HAL_RCC_GetPCLKFreq(GPTIM3_CORE, 1);
#else
    prescaler = HAL_RCC_GetPCLKFreq(0, 1);
#endif
    prescaler = prescaler / 1000 - 1;   /* timer_clock = 1 kHz, 1 tick = 1 ms */

    g_sdadc_gptim.Init.Period            = BSP_SDADC_TIMER_PERIOD_MS - 1;  /* unit: ms, timer_clock = 1 kHz */
    g_sdadc_gptim.Init.Prescaler         = prescaler;
    g_sdadc_gptim.Init.CounterMode       = GPT_COUNTERMODE_UP;
    g_sdadc_gptim.Init.RepetitionCounter = 0;
#if defined(SF32LB58X)
    g_sdadc_gptim.core = GPTIM3_CORE;
#endif

    if (HAL_GPT_Base_Init(&g_sdadc_gptim) != HAL_OK)
    {
        LOG_E("SDADC GPTIM3 init failed");
        return;
    }

    /* Route update event to TRGO (MMS=010) → SDADC hardware trigger input */
    g_sdadc_gptim.Instance->CR2 &= ~GPT_CR2_MMS;
    g_sdadc_gptim.Instance->CR2 |= (0x2 << GPT_CR2_MMS_Pos);

    /* Start counter (GPTIM runs forever, SDADC receives trigger via hardware) */
    g_sdadc_gptim.Instance->CR1 |= GPT_CR1_CEN;

    LOG_I("SDADC GPTIM3: PSC=%lu ARR=%lu period=%lu ms",
          (uint32_t)(g_sdadc_gptim.Instance->PSC),
          (uint32_t)(g_sdadc_gptim.Instance->ARR),
          (uint32_t)BSP_SDADC_TIMER_PERIOD_MS);
}
#endif /* BSP_SDADC_USE_TIMER_TRIGGER */

static rt_err_t sifli_sdadc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
    SDADC_HandleTypeDef *sifli_adc_handler = device->parent.user_data;
    rt_err_t r;

    RT_ASSERT(device != RT_NULL);

    if (channel < HAL_SDADC_GPIO_CHN0 || channel > HAL_SDADC_GPIO_CHN3)
    {
        LOG_E("SDADC channel must be between %d and %d.", HAL_SDADC_GPIO_CHN0, HAL_SDADC_GPIO_CHN3);
        return -RT_ERROR;
    }

    /* Serialize access to the SDADC hardware with sdadc_read_voltage(). */
    r = rt_sem_take(&g_sdadc_lock, rt_tick_from_millisecond(2000));
    if (r != RT_EOK)
        return r;

    HAL_SDADC_EnableSlot(sifli_adc_handler, channel, enabled ? 1 : 0);

    rt_sem_release(&g_sdadc_lock);

    return RT_EOK;
}

static rt_err_t sdadc_read_voltage(SDADC_HandleTypeDef *sifli_adc_handler, rt_uint32_t channel, rt_uint32_t *value)
{
    uint32_t reg_val;
    rt_err_t r;

    r = rt_sem_take(&g_sdadc_lock, rt_tick_from_millisecond(2000));
    if (r != RT_EOK)
        return r;

    /* Reject channels outside the supported GPIO channel range. */
    if (channel < HAL_SDADC_GPIO_CHN0 || channel > HAL_SDADC_GPIO_CHN3)
    {
        LOG_E("SDADC channel must be between %d and %d.", HAL_SDADC_GPIO_CHN0, HAL_SDADC_GPIO_CHN3);
        rt_sem_release(&g_sdadc_lock);
        return -RT_ERROR;
    }

#if defined(BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING)

#ifdef BSP_SDADC_USE_TIMER_TRIGGER
    reg_val = HAL_SDADC_GetValue(sifli_adc_handler, channel);
#else
    /* Software trigger */
    HAL_SDADC_Start(sifli_adc_handler);
    if (HAL_SDADC_PollForConversion(sifli_adc_handler, 100) != HAL_OK)
    {
        HAL_SDADC_Stop(sifli_adc_handler);
        rt_sem_release(&g_sdadc_lock);
        return -RT_ERROR;
    }
    HAL_SDADC_Stop(sifli_adc_handler);
    reg_val = HAL_SDADC_GetValue(sifli_adc_handler, channel);
#endif /* BSP_SDADC_USE_TIMER_TRIGGER */
#else
    /* single-channel path */
    {
        SDADC_ChannelConfTypeDef ADC_ChanConf;

        rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
        ADC_ChanConf.Channel = channel;
        ADC_ChanConf.pchnl_sel = channel;
        ADC_ChanConf.nchnl_sel = 0;
        ADC_ChanConf.shift_num = 2;
        ADC_ChanConf.slot_en = 1;
        HAL_SDADC_ConfigChannel(sifli_adc_handler, &ADC_ChanConf);

#ifdef BSP_SDADC_USE_TIMER_TRIGGER
        reg_val = HAL_SDADC_GetValue(sifli_adc_handler, channel);
#else
        HAL_SDADC_Start(sifli_adc_handler);

        if (HAL_SDADC_PollForConversion(sifli_adc_handler, 100) != HAL_OK)
        {
            HAL_SDADC_Stop(sifli_adc_handler);
            rt_sem_release(&g_sdadc_lock);
            return -RT_ERROR;
        }
        HAL_SDADC_Stop(sifli_adc_handler);
        reg_val = HAL_SDADC_GetValue(sifli_adc_handler, channel);
#endif /* BSP_SDADC_USE_TIMER_TRIGGER */
    }
#endif /* BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING */

    /* Clamp to (threshold_reg - 1). Guard against threshold_reg == 0 (invalid
     * calibration data) which would otherwise underflow to UINT32_MAX. */
    if (g_sdadc_calib_ctx.threshold_reg != 0 && reg_val >= g_sdadc_calib_ctx.threshold_reg)
    {
        reg_val = g_sdadc_calib_ctx.threshold_reg - 1;
    }

    /* convert register value to 0.1mV voltage */
    *value = (rt_uint32_t)(HAL_SDADC_RegToVoltageFloat((float)reg_val, &g_sdadc_calib_ctx) * 10.0f);

    r = rt_sem_release(&g_sdadc_lock);
    RT_ASSERT(r == RT_EOK);

    return RT_EOK;
}

static rt_err_t sifli_get_sdadc_value(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(value != RT_NULL);

    return sdadc_read_voltage((SDADC_HandleTypeDef *)device->parent.user_data, channel, value);
}

static rt_err_t sifli_sdadc_control(struct rt_adc_device *device, rt_uint32_t cmd, void *arg)
{
    rt_adc_cmd_read_arg_t *read_arg = (rt_adc_cmd_read_arg_t *)arg;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(RT_ADC_CMD_READ == cmd);
    RT_ASSERT(read_arg);

    return sdadc_read_voltage((SDADC_HandleTypeDef *)device->parent.user_data,
                              read_arg->channel, &read_arg->value);
}

static void sifli_sdadc_config(SDADC_HandleTypeDef *sifli_adc_handler)
{
    SDADC_GainConfTypeDef gain;
    SDADC_AccurateConfTypeDef accu;

    gain.gain_deno = 4;
    gain.gain_nume = 1;
    HAL_SDADC_ConfigGain(sifli_adc_handler, &gain);

    accu.chop1_num = 0x9c;
    accu.chop2_num = 0xc9;
    accu.chop3_num = 0x1ff;
    accu.chop_ref_num = 0x9c;
    accu.sample_num = 0xe0;
    HAL_SDADC_ConfigAccu(sifli_adc_handler, &accu);

#if defined(BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING)
    {
        SDADC_ChannelConfTypeDef ADC_ChanConf;

        for (uint32_t channel = HAL_SDADC_GPIO_CHN0; channel <= HAL_SDADC_GPIO_CHN3; channel++)
        {
            rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
            ADC_ChanConf.Channel = channel;
            ADC_ChanConf.pchnl_sel = channel;
            ADC_ChanConf.nchnl_sel = 0;
            ADC_ChanConf.shift_num = 2;
            ADC_ChanConf.slot_en = 1;
            HAL_SDADC_ConfigChannel(sifli_adc_handler, &ADC_ChanConf);
        }
    }
#endif /* BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING */
}

static rt_err_t sifli_op_sdadc_init(struct rt_adc_device *device)
{
    SDADC_HandleTypeDef *sifli_adc_handler = device->parent.user_data;

    if (HAL_SDADC_DeInit(sifli_adc_handler) != HAL_OK)
        return -RT_ERROR;

    if (HAL_SDADC_Init(sifli_adc_handler) != HAL_OK)
        return -RT_ERROR;

    sifli_sdadc_config(sifli_adc_handler);

#ifdef BSP_SDADC_USE_TIMER_TRIGGER
    sifli_sdadc_use_gptime_init();

    HAL_NVIC_SetPriority(SDADC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(SDADC_IRQn);

    HAL_SDADC_SetTimer(sifli_adc_handler, HAL_SDADC_SRC_GPTIM3);
    SDADC_ENABLE_TIMER_TRIGER(sifli_adc_handler);
    sifli_adc_handler->Init.src_sel = HAL_SDADC_SRC_TIMER;
#endif

    return RT_EOK;
}

static const struct rt_adc_ops sifli_sdadc_ops =
{
    .enabled = sifli_sdadc_enabled,
    .convert = sifli_get_sdadc_value,
    .init    = sifli_op_sdadc_init,
    .control = sifli_sdadc_control,
};

static int sifli_sdadc_init(void)
{
    int result = RT_EOK;
    char name_buf[6] = {'s', 'd', 'a', 'd', 'c', 0};

    rt_sem_init(&g_sdadc_lock, "sdadc", 1, RT_IPC_FLAG_FIFO);

    /* Load calibration */
    if (HAL_SDADC_CalibLoad(&g_sdadc_calib_ctx) != HAL_OK)
    {
        LOG_W("Get SDADC configure fail, use default calibration");
    }

    sifli_sdadc_obj.SDADC_Handler.Instance = hwp_sdadc;
    sifli_sdadc_obj.SDADC_Handler.DMA_Handle = NULL;
    sifli_sdadc_obj.SDADC_Handler.ErrorCode = 0;
    sifli_sdadc_obj.SDADC_Handler.Lock = HAL_UNLOCKED;
    sifli_sdadc_obj.SDADC_Handler.State = 0;

    sifli_sdadc_obj.SDADC_Handler.Init.adc_se = 1;
    sifli_sdadc_obj.SDADC_Handler.Init.diff_sel = 0;
    sifli_sdadc_obj.SDADC_Handler.Init.en_slot = 0;

#if defined(BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING)
    sifli_sdadc_obj.SDADC_Handler.Init.conti_mode = 0;
    sifli_sdadc_obj.SDADC_Handler.Init.dma_en = 0;
    sifli_sdadc_obj.SDADC_Handler.Init.dsample_mode = 1;
#else
    sifli_sdadc_obj.SDADC_Handler.Init.conti_mode = 0;
    sifli_sdadc_obj.SDADC_Handler.Init.dma_en = 0;
    sifli_sdadc_obj.SDADC_Handler.Init.dsample_mode = 0;
#endif
#ifdef BSP_SDADC_USE_TIMER_TRIGGER
    sifli_sdadc_obj.SDADC_Handler.Init.src_sel = HAL_SDADC_SRC_TIMER;
#else
    sifli_sdadc_obj.SDADC_Handler.Init.src_sel = HAL_SDADC_SRC_SW;
#endif
    sifli_sdadc_obj.SDADC_Handler.Init.vref_sel = HAL_SDADC_VERF_INTERNAL;

    if (HAL_SDADC_Init(&sifli_sdadc_obj.SDADC_Handler) != HAL_OK)
    {
        LOG_E("%s init failed", name_buf);
        result = -RT_ERROR;
    }
    else
    {
        sifli_sdadc_config(&sifli_sdadc_obj.SDADC_Handler);

        /* register SDADC device */
        if (rt_hw_adc_register(&sifli_sdadc_obj.sifli_sdadc_device, name_buf, &sifli_sdadc_ops, &sifli_sdadc_obj.SDADC_Handler) == RT_EOK)
        {
            LOG_D("%s init success", name_buf);
        }
        else
        {
            LOG_E("%s register failed", name_buf);
            result = -RT_ERROR;
        }
    }

    return result;
}
INIT_BOARD_EXPORT(sifli_sdadc_init);

/*********************************************
* This part only for VREF as power supply(3.3)
*********************************************/

/**
* @brief  Get SDMADC voltage by register value (uses HAL calibration context).
* @param[in]  value register value.
* @retval voltage in mv.
*/
int sdadc_get_vol(uint32_t value)
{
    return (int)HAL_SDADC_RegToVoltageFloat((float)value, &g_sdadc_calib_ctx);
}

//#define DRV_SDADC_TEST
#ifdef DRV_SDADC_TEST

#include <string.h>
static uint32_t reg_data[1024];

__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_TOPEN(SDADC_HandleTypeDef *hadc, uint8_t en, uint32_t delay)
{
    if (en)
    {
        // enable SDADCK CLK
        hwp_pmuc->HXT_CR2 |= PMUC_HXT_CR2_SDADC_CLKIN_EN;
        SDADC_Enable(hadc);
        hadc->Instance->CFG0 |= SDADC_CFG0_PPU_LV;

        HAL_Delay(delay);
    }
    else
    {
        SDADC_Disable(hadc);
        hadc->Instance->CFG0 &= (~SDADC_CFG0_PPU_LV);
        // enable SDADCK CLK
        hwp_pmuc->HXT_CR2 &= (~PMUC_HXT_CR2_SDADC_CLKIN_EN);
    }

    return HAL_ERROR;
}

__HAL_ROM_USED uint32_t HAL_SDADC_TREAD(SDADC_HandleTypeDef *hadc)
{
    uint32_t value;
    uint32_t Timeout;
    uint32_t tickstart;

    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLDOWN, 0);
    value = hadc->Instance->CFG3;
    value &= ~(SDADC_CFG3_SEL_PCH_LV);
    //value |=  (0 << SDADC_CFG3_SEL_PCH_LV_Pos);
    hadc->Instance->CFG3 = value;

    hadc->Instance->TRIG |= SDADC_TRIG_ADC_START;

    tickstart = HAL_GetTick();
    Timeout = 100;

    while (HAL_IS_BIT_CLR(hadc->Instance->CFG0, SDADC_CFG0_SDADC_DATA_RDY))
    {
        /* Check if timeout is disabled (set to infinite wait) */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0) || ((HAL_GetTick() - tickstart) > Timeout))
            {
                return 0;
            }
        }
    }
    HAL_PIN_Set(PAD_PB01, GPIO_B1, PIN_PULLUP, 0);

    value |= (7 << SDADC_CFG3_SEL_PCH_LV_Pos);
    hadc->Instance->CFG3 = value;

    value = hadc->Instance->SINGLE_DOUT & 0xffffff;

    return value;
}

uint32_t HAL_SDADC_LOOP(uint32_t channel, uint32_t delay, uint32_t gap, uint32_t gain_nume, uint32_t loop)
{
    SDADC_ChannelConfTypeDef ADC_ChanConf;
    SDADC_GainConfTypeDef gain;
    int i = 0;

    //if(loop > 1024)
    //{
    //    rt_kprintf("Loop count too large, it can not larger than 1024\n");
    //    return 1;
    //}
    gain.gain_deno = 4;
    gain.gain_nume = (uint8_t)(gain_nume & 0x7);
    HAL_SDADC_ConfigGain(&sifli_sdadc_obj.SDADC_Handler, &gain);

    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));

    ADC_ChanConf.Channel = channel;
    ADC_ChanConf.pchnl_sel = channel;
    ADC_ChanConf.nchnl_sel = 0;
    ADC_ChanConf.shift_num = 2;
    ADC_ChanConf.slot_en = 1;
    HAL_SDADC_ConfigChannel(&sifli_sdadc_obj.SDADC_Handler, &ADC_ChanConf);

    /* start SDADC */
    //HAL_SDADC_Start(&sifli_sdadc_obj.SDADC_Handler);

    sifli_sdadc_obj.SDADC_Handler.Instance->RSVD |= (1 << SDADC_RSVD_RESERVE1_Pos);
    sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 |= SDADC_CFG0_PPU_LV;
    sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 |= SDADC_CFG0_PU_LV;

    HAL_Delay(3000);

    //HAL_SDADC_Start(&sifli_sdadc_obj.SDADC_Handler);

    /* Wait for the SDADC to convert */
    HAL_SDADC_PollForConversion(&sifli_sdadc_obj.SDADC_Handler, 100);

    /* get SDADC value */
    reg_data[0] = (rt_uint32_t)HAL_SDADC_GetValue(&sifli_sdadc_obj.SDADC_Handler, channel);

    sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= ~SDADC_CFG0_PU_LV;
    sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= ~SDADC_CFG0_PPU_LV;
    sifli_sdadc_obj.SDADC_Handler.Instance->RSVD &= ~SDADC_RSVD_RESERVE1;
    rt_kprintf("cnt 0, value %d\n", reg_data[0]);

    i = 1;
    while (i < loop)
    {
        HAL_Delay(gap); // wait 950 between twice test

        sifli_sdadc_obj.SDADC_Handler.Instance->RSVD |= (1 << SDADC_RSVD_RESERVE1_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 |= SDADC_CFG0_PPU_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 |= SDADC_CFG0_PU_LV;

        HAL_Delay(delay);   // wait 50ms between power and get data

        //HAL_SDADC_Start(&sifli_sdadc_obj.SDADC_Handler);

        /* Wait for the SDADC to convert */
        HAL_SDADC_PollForConversion(&sifli_sdadc_obj.SDADC_Handler, 100);

        /* get SDADC value */
        reg_data[0] = (rt_uint32_t)HAL_SDADC_GetValue(&sifli_sdadc_obj.SDADC_Handler, channel);

        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= ~SDADC_CFG0_PU_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= ~SDADC_CFG0_PPU_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->RSVD &= ~SDADC_RSVD_RESERVE1;

        rt_kprintf("cnt %d, value %d\n", i, reg_data[0]);
        i++;
    }

    return 0;
}

uint32_t test_sdadc_diff(uint32_t pre_delay, uint32_t delay, uint32_t interv, uint32_t loop)
{
    SDADC_GainConfTypeDef gain;
    int i = 0;
    uint32_t value;

    //HAL_PIN_Set(PAD_PB23, SDADC_CH0, PIN_NOPULL, 0);  //##SDADC_CH0
    hwp_pinmux2->PAD_PB23 = 0X20A;
    HAL_Delay(pre_delay);
    i = 0;
    while (i < loop)
    {
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG3 &= (~SDADC_CFG3_VREF_SEL_LV); // VREF_SEL_LV = 0

        gain.gain_deno = 2;
        gain.gain_nume = 2; //(uint8_t)(gain_nume & 0x7);
        HAL_SDADC_ConfigGain(&sifli_sdadc_obj.SDADC_Handler, &gain);    // gain_nume/deno = 2;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= (~SDADC_CFG0_FCK_SEL_LV); // fck_sel_lv = 0;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG2 |= SDADC_CFG2_SE_DIFF_SEL_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG2 &= (~SDADC_CFG2_DEM_EN_LV);

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG1;
        value &= ~(SDADC_CFG1_CHOP1_NUM_LV | SDADC_CFG1_CHOP2_NUM_LV);
        value |= (0x9c << SDADC_CFG1_CHOP1_NUM_LV_Pos) | (0xc9 << SDADC_CFG1_CHOP2_NUM_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG1 = value;

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG2;
        value &= ~(SDADC_CFG2_CHOP_REF_NUM_LV | SDADC_CFG2_SAMPLE_NUM_LV);
        value |= (0x9c << SDADC_CFG2_CHOP_REF_NUM_LV_Pos) | (0xe0 << SDADC_CFG2_SAMPLE_NUM_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG2 = value;

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG1;
        value &= ~(SDADC_CFG1_FCHOP_SIG_SEL_LV);
        value |= (2 << SDADC_CFG1_FCHOP_SIG_SEL_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG1 = value;

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG2;
        value &= ~(SDADC_CFG2_FCHOP_REF_SEL_LV);
        value |= (2 << SDADC_CFG2_FCHOP_REF_SEL_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG2 = value;

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG3; // &= (~SDADC_CFG3_REFBUF_CHOP_MX_LV);
        value &= ~(SDADC_CFG3_REFBUF_CHOP_MX_LV | SDADC_CFG3_AMP1_BM_LV | SDADC_CFG3_REFBUF_BM_LV | SDADC_CFG3_AMP2_BM_LV);
        value |= (5 << SDADC_CFG3_AMP1_BM_LV_Pos) | (5 << SDADC_CFG3_REFBUF_BM_LV_Pos) | (5 << SDADC_CFG3_AMP2_BM_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG3 = value;

        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= (~SDADC_CFG0_DSAMPLE_MODE);   // dsample_mode = 0

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG3; //
        value &= ~(SDADC_CFG3_SEL_PCH_LV | SDADC_CFG3_SEL_NCH_LV);
        value |= (1 << SDADC_CFG3_SEL_NCH_LV_Pos) | (2 << SDADC_CFG3_SEL_PCH_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG3 = value;

        sifli_sdadc_obj.SDADC_Handler.Instance->RSVD |= (1 << SDADC_RSVD_RESERVE1_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 |= SDADC_CFG0_PPU_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 |= SDADC_CFG0_PU_LV;

        hwp_pmuc->HXT_CR2 |= PMUC_HXT_CR2_SDADC_CLKIN_EN;

        HAL_Delay_us(delay);

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG3; //
        value &= ~(SDADC_CFG3_SEL_PCH_LV | SDADC_CFG3_SEL_NCH_LV);
        value |= (3 << SDADC_CFG3_SEL_NCH_LV_Pos) | (0 << SDADC_CFG3_SEL_PCH_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG3 = value;

        sifli_sdadc_obj.SDADC_Handler.Instance->TRIG |= SDADC_TRIG_ADC_START;

        while ((sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 & SDADC_CFG0_SDADC_DATA_RDY) == 0);

        hwp_pmuc->HXT_CR2 &= ~PMUC_HXT_CR2_SDADC_CLKIN_EN;

        value = sifli_sdadc_obj.SDADC_Handler.Instance->CFG3; //
        value &= ~(SDADC_CFG3_SEL_PCH_LV | SDADC_CFG3_SEL_NCH_LV);
        value |= (1 << SDADC_CFG3_SEL_NCH_LV_Pos) | (2 << SDADC_CFG3_SEL_PCH_LV_Pos);
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG3 = value;

        reg_data[0] = sifli_sdadc_obj.SDADC_Handler.Instance->SINGLE_DOUT & SDADC_SINGLE_DOUT_DATA_Msk;

        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= ~SDADC_CFG0_PU_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->CFG0 &= ~SDADC_CFG0_PPU_LV;
        sifli_sdadc_obj.SDADC_Handler.Instance->RSVD &= ~SDADC_RSVD_RESERVE1;

        rt_kprintf("cnt %d, value %d\n", i, reg_data[0]);

        i++;
        HAL_Delay(interv);
    }

    return 0;
}

int cmd_sdadc(int argc, char *argv[])
{
    rt_device_t dev;
    if (argc < 3)
    {
        LOG_I("Invalid parameter\n");
        LOG_I("sdadc -enable/-read/-close channel\n");
    }
    else if (strcmp(argv[1], "-enable") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        dev = rt_device_find("sdadc");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)chnl);
            LOG_I("SDADC channel %d enabled\n",  chnl);
        }
        else
        {
            LOG_I("Find sdadc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-disable") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        dev = rt_device_find("sdadc");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            rt_device_control(dev, RT_ADC_CMD_DISABLE, (void *)chnl);
            LOG_I("SDADC channel %d disabled\n", chnl);
        }
        else
        {
            LOG_I("Find sdadc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        uint32_t value, res;
        dev = rt_device_find("sdadc");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_read(dev, chnl, &value, 1);
            LOG_I("Read SDADC channel %d : 0x%x, res = %d \n", chnl, value, res);
        }
        else
        {
            LOG_I("Find sdadc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-vol") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        uint32_t value, res;
        dev = rt_device_find("sdadc");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_read(dev, chnl, &value, 1);
            if (res > 0)
            {
                int vol = sdadc_get_vol(value);
                LOG_I("SDADC voltage = %d mv\n", vol);
            }
            else
                LOG_I("Read SDADC channel %d fail, res = %d \n", chnl, res);
        }
        else
        {
            LOG_I("Find sdadc device fail\n");
        }
    }
    else if (strcmp(argv[1], "-a") == 0)
    {
        uint32_t delay = atoi(argv[2]);
        uint32_t cnt = atoi(argv[3]);
        uint32_t delay2 = 100;
        if (argc > 4)
            delay2 = atoi(argv[4]);
        int i = 0;
        if (cnt > 1024)
        {
            rt_kprintf("Too large couter\n");
            return 0;
        }
        rt_kprintf("delay1 %d ms, loop %d times, delay2 %d ms\n", delay, cnt, delay2);
        for (i = 0; i < cnt; i++)
        {
            HAL_SDADC_TOPEN(&sifli_sdadc_obj.SDADC_Handler, 1, delay);
            reg_data[i] = HAL_SDADC_TREAD(&sifli_sdadc_obj.SDADC_Handler);
            HAL_SDADC_TOPEN(&sifli_sdadc_obj.SDADC_Handler, 0, delay);
            HAL_Delay(delay2);
        }
        for (i = 0; i < cnt; i++)
        {
            rt_kprintf("%d \n", reg_data[i]);
        }
    }
    else if (strcmp(argv[1], "-b") == 0)
    {
        uint32_t delay = atoi(argv[2]);
        uint32_t cnt = atoi(argv[3]);
        int i = 0;
        if (cnt > 1024)
        {
            rt_kprintf("Too large couter\n");
            return 0;
        }
        rt_kprintf("delay1 %d ms, loop %d times\n", delay, cnt);
        HAL_SDADC_TOPEN(&sifli_sdadc_obj.SDADC_Handler, 1, delay);
        for (i = 0; i < cnt; i++)
        {
            reg_data[i] = HAL_SDADC_TREAD(&sifli_sdadc_obj.SDADC_Handler);
        }
        HAL_SDADC_TOPEN(&sifli_sdadc_obj.SDADC_Handler, 0, delay);
        for (i = 0; i < cnt; i++)
        {
            rt_kprintf("%d \n", reg_data[i]);
        }
    }
    else if (strcmp(argv[1], "-loop") == 0)
    {
        uint32_t chnl = atoi(argv[2]);  // sdmadc channel
        uint32_t delay = atoi(argv[3]); // delay between power up and read, typcal 50 ms
        uint32_t gap = atoi(argv[4]);   // delay between 2 test, typcal 950 ms
        uint32_t gain_num = atoi(argv[5]);  // gain nume
        uint32_t loop = atoi(argv[6]);  // loop counter, can not larger than 1024

        int res = HAL_SDADC_LOOP(chnl, delay, gap, gain_num, loop);
        if (res == 0)
            rt_kprintf("Test success\n");
        else
            rt_kprintf("Test fail\n");
    }
    else if (strcmp(argv[1], "-diff") == 0)
    {
        uint32_t pre_delay = atoi(argv[2]);  // delay after set pinmux, before start test mith ms
        uint32_t interv = atoi(argv[3]);  // interval between each loop, with ms
        uint32_t delay = atoi(argv[4]); // delay before triger start, with us
        uint32_t loop = atoi(argv[5]);  // loop counter

        int res = test_sdadc_diff(pre_delay, delay, interv, loop);
        if (res == 0)
            rt_kprintf("Test success\n");
        else
            rt_kprintf("Test fail\n");
    }
    else
    {
        LOG_I("Invalid parameter\n");
        LOG_I("sdadc -enable/-read/-close channel\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_sdadc, __cmd_sdadc, Test sdadc driver);

#endif /* DRV_SDADC_TEST */

#endif /* BSP_USING_SDADC */

/// @} drv_lcpu
/// @} bsp_driver

/// @} file

