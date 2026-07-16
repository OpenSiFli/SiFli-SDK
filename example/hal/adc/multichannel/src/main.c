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
#include <stdlib.h>

//#define BSP_GPADC_USING_DMA 1
#define ADC_DEV_CHANNEL     0           /* ADC channel */

static HAL_ADC_CalibContextTypeDef g_adc_calib_ctx;
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

#ifdef SF32LB52X
    rt_kprintf("\n vbat_mv: %d mv, %d; ldoref_flag = %d, ldoref_sel = %d;\n",
               factory_info.vbat_mv, factory_info.vbat_reg,
               factory_info.ldovref_flag, factory_info.ldovref_sel);
#else
    rt_kprintf("\n vbat_mv: %d mv, %d; ldoref_flag = %d, ldoref_sel = %d;\n", 0, 0, 0, 0);
#endif

    return HAL_OK;
}

static void adc_example(void)
{

    ADC_ChannelConfTypeDef ADC_ChanConf;
    uint32_t dst;
    uint32_t lslot = ADC_DEV_CHANNEL;
    HAL_StatusTypeDef ret = HAL_OK;

    // make sure set CORRECT ADC pin to correct mode
    //HAL_PIN_Set_Analog(PAD_PA32, 0);
    hadc.Instance = hwp_gpadc1;

    int calib;
    calib = utest_adc_calib();
    rt_kprintf("ADC Get calibration res %d\n", calib);

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
#ifdef BSP_GPADC_USING_DMA

    // example for 52x 6 channel 0-5 only
    /* set pinmux of channel 0 to analog input */
    HAL_PIN_Set_Analog(PAD_PA28, 1); /* channel 0 */
    HAL_PIN_Set_Analog(PAD_PA29, 1);
    HAL_PIN_Set_Analog(PAD_PA30, 1);
    HAL_PIN_Set_Analog(PAD_PA31, 1);
    HAL_PIN_Set_Analog(PAD_PA32, 1);
    HAL_PIN_Set_Analog(PAD_PA33, 1); /* channel 5 */

#include "dma_config.h"
    hadc.DMA_Handle = (DMA_HandleTypeDef *)malloc(sizeof(DMA_HandleTypeDef));
    if (hadc.DMA_Handle != NULL)
    {
        memset((void *)hadc.DMA_Handle, 0, sizeof(DMA_HandleTypeDef));
        hadc.DMA_Handle->Instance                 = GPADC_DMA_INSTANCE;
        hadc.DMA_Handle->Init.Request             = GPADC_DMA_REQUEST;
        hadc.DMA_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
        hadc.DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hadc.DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hadc.DMA_Handle->Init.PeriphInc           = DMA_PINC_DISABLE;
        hadc.DMA_Handle->Init.MemInc              = DMA_MINC_ENABLE;
        hadc.DMA_Handle->Init.Mode                = DMA_NORMAL;
        hadc.DMA_Handle->Init.Priority            = DMA_PRIORITY_MEDIUM;
    }
    hadc.Init.en_slot = 1;  // use multi channels, need multi slot
#else
    /* set pinmux of channel 0 to analog input */
    HAL_PIN_Set_Analog(PAD_PA28, 1);
#endif

    /* 2, open adc clock source  */
    HAL_RCC_EnableModule(RCC_MOD_GPADC);

    HAL_ADC_Init(&hadc);
    if (calib == HAL_OK)
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

    /* start ADC */
    HAL_ADC_Start(&hadc);

    /* Wait for the ADC to convert */
    ret = HAL_ADC_PollForConversion(&hadc, 100);

    /* get ADC register value */
    dst = HAL_ADC_GetValue(&hadc, lslot);
    rt_kprintf("ADC reg value %d ", dst);
    if (calib == 0)
    {
        rt_kprintf("voltage %f mv\n", HAL_ADC_RegToVoltageFloat((float)dst, &g_adc_calib_ctx));
    }

    HAL_ADC_Stop(&hadc);
#else
    uint32_t data[8];
    int i, j;

    // configure channel, can support multi channels
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = 0;
    ADC_ChanConf.pchnl_sel = 0;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 1;
    ADC_ChanConf.pchnl_sel = 1;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 2;
    ADC_ChanConf.pchnl_sel = 2;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 3;
    ADC_ChanConf.pchnl_sel = 3;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 4;
    ADC_ChanConf.pchnl_sel = 4;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 5;
    ADC_ChanConf.pchnl_sel = 5;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    /* configure power setting */
    HAL_ADC_DMA_PREPARE(&hadc);

    for (j = 0; j < 0x1; j++)
    {
        /* start ADC */
        HAL_ADC_Start_DMA(&hadc, data, 6);

        /* Wait for the ADC to convert */
        ret = HAL_ADC_DMA_WAIT_DONE(&hadc, 100);

        for (i = 0; i < 6; i++)
        {
            rt_kprintf("ADC reg value[%d] %d ", i, data[i] & 0xfff);
            if (calib == 0)
            {
                rt_kprintf("voltage %f mv\n", HAL_ADC_RegToVoltageFloat((float)(data[i] & 0xfff), &g_adc_calib_ctx));
            }
        }

        rt_kprintf("\nLoop %d done ===\n", j);
        HAL_Delay(1000);
    }

    HAL_ADC_Stop_DMA(&hadc);

#endif
    // TODO, if need get adc more times, need delay 5/10 ms before next start

    // never call Deinit function !!!
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */
    rt_kprintf("Start adc demo!\n");
    adc_example();
    rt_kprintf("adc demo end!\n");
    while (1);
    return 0;
}
