/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "string.h"
#include "rtthread.h"

/* User code start from here --------------------------------------------------------*/

//#define BSP_GPADC_USING_DMA 1
#if defined(SF32LB52X)
    #define ADC_DEV_CHANNEL     7           /* ADC channel 7  */
#elif defined(SF32LB56X)
    #define ADC_DEV_CHANNEL     1           /* ADC channel 1 */
#elif defined(SF32LB58X)
    #define ADC_DEV_CHANNEL     2           /* ADC channel 2  */
#elif defined(SF32LB57X)
    #define ADC_DEV_CHANNEL     11          /* ADC channel 11, VBAT on 57x */
#else
    #define ADC_DEV_CHANNEL     7
#endif

#define ADC_SW_AVRA_CNT      (22)

static HAL_ADC_CalibContextTypeDef g_adc_calib_ctx;
static uint32_t g_adc_slot;
static int g_adc_calib_ok;
ADC_HandleTypeDef hadc;

/*
    This example demo:
        1. Configure ADC parameters
        2. Polling ADC value
*/
static int utest_adc_calib(void)
{
    HAL_ADC_CalibFactoryInfoTypeDef factory_info;

    if (HAL_ADC_CalibLoad(NULL,
                          &g_adc_calib_ctx,
                          HAL_ADC_CALIB_SOURCE_BSP,
                          HAL_ADC_CALIB_F_INIT) != HAL_OK)
    {
        rt_kprintf("Get ADC configure fail\n");
        return HAL_ERROR;
    }

    if (HAL_ADC_CalibGetFactoryInfo(HAL_ADC_CALIB_SOURCE_BSP, &factory_info) != HAL_OK)
    {
        rt_kprintf("Get ADC configure fail\n");
        return HAL_ERROR;
    }

    rt_kprintf("\nGPADC :vol10: %d mv, %d; vol25: %d mv reg %d; offset %f, ratio %f, max reg %d;\n",
               factory_info.voltage1_mv, factory_info.reg_value1,
               factory_info.voltage2_mv, factory_info.reg_value2,
               g_adc_calib_ctx.offset, g_adc_calib_ctx.ratio, g_adc_calib_ctx.threshold_reg);

#ifdef ADC_VBAT_DEDICATED_CHANNEL_SUPPORT
    rt_kprintf("\n vbat_mv: %d mv, %d; ldoref_flag = %d, ldoref_sel = %d;\n",
               factory_info.vbat_mv, factory_info.vbat_reg,
               factory_info.ldovref_flag, factory_info.ldovref_sel);
#else
    rt_kprintf("\n vbat_mv: %d mv, %d; ldoref_flag = %d, ldoref_sel = %d;\n", 0, 0, 0, 0);
#endif

    return HAL_OK;
}

static void adc_example_init(void)
{
    ADC_ChannelConfTypeDef ADC_ChanConf;
    uint32_t lslot = ADC_DEV_CHANNEL;

    // make sure set CORRECT ADC pin to correct mode
#if defined(SF32LB56X)
    HAL_PIN_Set_Analog(PAD_PB23, 0);
#elif defined(SF32LB58X)
    HAL_PIN_Set_Analog(PAD_PB34, 0);
#endif
    hadc.Instance = hwp_gpadc1;

    g_adc_calib_ok = (utest_adc_calib() == HAL_OK) ? 1 : 0;
    rt_kprintf("ADC Get calibration res %d\n", g_adc_calib_ok);

    // initial adc handle

#ifndef SF32LB55X
    hadc.Init.data_samp_delay = 2;
#ifdef SF32LB52X
    hadc.Init.conv_width = 75;
    hadc.Init.sample_width = 71;
#else
    hadc.Init.conv_width = 24;
    hadc.Init.sample_width = 22;
#endif
#else
    hadc.Init.clk_div = 31;
#endif
    hadc.Init.adc_se = 1;   // single end
    hadc.Init.adc_force_on = 0;
    hadc.Init.atten3 = 0;
    hadc.Init.dma_en = 0;   // no dma
    hadc.Init.en_slot = 0;  // default slot 0
    hadc.Init.op_mode = 0;  // single mode, not continous

    /* 2, open adc clock source  */
    HAL_RCC_EnableModule(RCC_MOD_GPADC);

    HAL_ADC_Init(&hadc);
    if (g_adc_calib_ok)
    {
        (void)HAL_ADC_CalibApply(&hadc, &g_adc_calib_ctx);
    }
    // delay 300ms before start adc start, only once
    HAL_Delay(300);
    // enable slot
    //HAL_ADC_EnableSlot(&hadc, lslot, 1);

#ifndef BSP_GPADC_USING_DMA
    // Channel to select register, pchnl_sel to choose which pin used, here use the same number
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = lslot;
    ADC_ChanConf.pchnl_sel = lslot;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    g_adc_slot = lslot;

#endif
    // never call Deinit function !!!
}

static float adc_example_read_mv(void)
{
    HAL_StatusTypeDef ret = HAL_OK;
    float mv;
    uint32_t data[ADC_SW_AVRA_CNT];
    uint32_t total = 0;
    uint32_t ave;
    float fave;
    int i, j;

    /* start ADC */
    HAL_ADC_Start(&hadc);

    for (i = 0; i < ADC_SW_AVRA_CNT; i++)
    {
        if (i != 0)
        {
#ifndef  SF32LB55X
            // unmute before read adc
            ADC_SET_UNMUTE(&hadc);
            HAL_Delay_us(200);
#else
            // FRC EN before each start
            ADC_FRC_EN(&hadc);
            HAL_Delay_us(50);
#endif
            __HAL_ADC_START_CONV(&hadc);
        }

        /* Wait for the ADC to convert */
        ret = HAL_ADC_PollForConversion(&hadc, 100);
        if (ret != HAL_OK)
        {
            HAL_ADC_Stop(&hadc);
            return -1.0f;
        }

        /* get ADC register value */
        data[i] = (uint32_t)HAL_ADC_GetValue(&hadc, g_adc_slot);
        total += data[i];

#ifndef  SF32LB55X
        ADC_SET_MUTE(&hadc);
#ifdef SF32LB52X
        if (g_adc_slot == 7)
            rt_thread_delay(1);
        else
            rt_thread_delay(10);
#else
        rt_thread_delay(10);
#endif
#else   /* SF32LB55X */
        ADC_CLR_FRC_EN(&hadc);
        rt_thread_delay(5);
#endif
    }

    HAL_ADC_Stop(&hadc);

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

    // Use new unified HAL calibration API to convert register value to voltage
    mv = HAL_ADC_RegToVoltageFloat(fave, &g_adc_calib_ctx);
#ifdef ADC_VBAT_DEDICATED_CHANNEL_SUPPORT
    if (ADC_DEV_CHANNEL == ADC_VBAT_DEDICATED_CHANNEL)
    {
        // VBAT uses 1/2 divider (52x ch7, 57x ch11), apply vbat_factor to recover actual voltage
        mv *= g_adc_calib_ctx.vbat_factor;
    }
#endif
    return mv;
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Start adc demo!\n");
    adc_example_init();
    while (1)
    {
        float mv = adc_example_read_mv();
        if (mv >= 0.0f)
            rt_kprintf("battery voltage %f mv\n", mv);
        else
            rt_kprintf("adc read failed\n");

        rt_thread_mdelay(1000);
    }
    return 0;
}
