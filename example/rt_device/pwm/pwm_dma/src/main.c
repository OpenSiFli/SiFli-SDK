/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"
#include "stdlib.h"// for mlloc and free
#include <rtdbg.h>

/* pwm example for RT-Thread based platform -----------------------------------------------*/
#define PWM_PERIOD (1 * 1000 * 1000) /*(ns) -> freq = 1,000,000,000/PWM_PERIOD (hz) 1Mhz*/
#define PWM_DEV_GTIM_NAME_52 "pwmt2"//52x  pw3_cc1
#define PWM_DEV_GTIM_NAME_58 "pwmt1"//58x  pw2_cc2

#define PWM_DEV_ATIM_NAME_52 "pwma1"//52x  pwma1_cc1
#define PWM_DEV_ATIM_NAME_58 "pwma2"//58x  pwma2_cc4

void pwm_dma_atim_set_example()
{
    struct rt_pwm_configuration config_atim;
    uint8_t percentage = 90;
    uint32_t period = PWM_PERIOD;

    LOG_I("pwm_set:percentage:%d,period:%d,freq:%dhz", percentage, period, 1000000000 / period);
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA20, ATIM1_CH1, PIN_NOPULL, 1);//52x ATIM1_CH1 corresponds to pwma1_cc1
#elif defined SF32LB58X
    HAL_PIN_Set(PAD_PA51, ATIM2_CH4, PIN_NOPULL, 1);//58X ATIM2_CH4 corresponds to pwma2_cc4
#endif
    if(percentage > 100)
        percentage = 100;
    rt_uint32_t pulse = percentage * period / 100;
#ifdef SF32LB52X
    config_atim.channel = 1;//pwm config
#elif defined SF32LB58X
    config_atim.channel = 4;//58 pwm config
#endif

    config_atim.period = period;
    config_atim.pulse = pulse;
    config_atim.use_percentage = 1;//Enables the percentage calculation of pulse
    config_atim.data_len = 3;//dma_data_len
//Calculating pulse procedure

    // Allocates memory for pulse_dma_data and initializes pulse as a percentage of period before the calculation
    config_atim.pulse_dma_data = (rt_uint32_t *)malloc(config_atim.data_len * sizeof(rt_uint32_t));
    if (!config_atim.pulse_dma_data)
    {
        printf("Memory allocation failed\n");

    }

    //pulse percentage value before calculation
    config_atim.pulse_dma_data[0] = (50 % 100) * PWM_PERIOD / 100;; // 50%
    config_atim.pulse_dma_data[1] = (60 % 100) * PWM_PERIOD / 100;; // 60%
    config_atim.pulse_dma_data[2] = (70 % 100) * PWM_PERIOD / 100;; // 70%
    //

    struct rt_device_pwm *device = RT_NULL;
#ifdef SF32LB52X
    device = (struct rt_device_pwm *)rt_device_find(PWM_DEV_ATIM_NAME_52);
    if (!device)
    {
        LOG_I("find pwma1 err");
        return;
    }
#elif defined SF32LB58X
    device = (struct rt_device_pwm *)rt_device_find(PWM_DEV_ATIM_NAME_58);
    if (!device)
    {
        LOG_I("find pwma2 err");
        return;
    }
#endif

    rt_device_control((struct rt_device *)device, PWM_CMD_SET, (void *)&config_atim); //Set basic data（ARR,CRR） and calculate pulse
#ifdef SF32LB52X
    LOG_I("hwp_atim1_ccr1:%d", hwp_atim1->CCR1);
    LOG_I("hwp_atim1_arr:%d", hwp_atim1->ARR);
#endif

#ifdef SF32LB58X
    LOG_I("hwp_atim2_ccr4:%d", hwp_atim2->CCR4);    //58x atim
    LOG_I("hwp_atim2_arr:%d", hwp_atim2->ARR);
#endif

    //The pulse dma data is overwritten after calculation
    for (size_t i = 0; i < global_array_length; i++)
    {
        config_atim.pulse_dma_data[i] = (rt_uint32_t)global_pulse_values[i];
    }

    // Check the calculated pulse dma data
    printf("DMA data before transfer:\n");
    for (size_t i = 0; i < config_atim.data_len; i++)
    {
        printf("pulse_dma_data[%zu]: %lu\n", i, config_atim.pulse_dma_data[i]);
    }
    //Assign the calculated pulse_dma_data array starting address value to dma_data
    config_atim.dma_data = (rt_uint16_t *)config_atim.pulse_dma_data;

    rt_device_control((struct rt_device *)device, PWM_CMD_ENABLE, (void *)&config_atim); //dma_transfer

    free(config_atim.pulse_dma_data);

}

void pwm_dma_gptim_set_example()
{
    struct rt_pwm_configuration config_gptim;
    uint8_t percentage = 10;
    uint32_t period = PWM_PERIOD;

    LOG_I("pwm_set:percentage:%d,period:%d,freq:%dhz", percentage, period, 1000000000 / period);
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA20, GPTIM2_CH1, PIN_NOPULL, 1);//52x gtime2_ch1 corresponds to pwm3_cc1
#elif defined SF32LB58X
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);//58X gtime1_ch2 corresponds to pwm2_cc2
#endif
    if(percentage > 100)
        percentage = 100;
    rt_uint32_t pulse = percentage * period / 100;
#ifdef SF32LB52X
    config_gptim.channel = 1;//pwm config
#elif defined SF32LB58X
    config_gptim.channel = 2;//58 pwm config
#endif

    config_gptim.period = period;
    config_gptim.pulse = pulse;
    config_gptim.use_percentage = 1;//Enables the percentage calculation of pulse
    config_gptim.data_len = 3;//dma_data_len
//Calculating pulse procedure

    // Allocates memory for pulse_dma_data and initializes pulse as a percentage of period before the calculation
    config_gptim.pulse_dma_data = (rt_uint32_t *)malloc(config_gptim.data_len * sizeof(rt_uint32_t));
    if (!config_gptim.pulse_dma_data)
    {
        printf("Memory allocation failed\n");

    }
    //dma_data
    config_gptim.dma_data = (rt_uint16_t *)malloc(config_gptim.data_len * sizeof(rt_uint16_t));
    if (!config_gptim.dma_data)
    {
        printf("Memory allocation failed\n");

    }
    //pulse percentage value before calculation
    config_gptim.pulse_dma_data[0] = (20 % 100) * PWM_PERIOD / 100;; // 20%
    config_gptim.pulse_dma_data[1] = (30 % 100) * PWM_PERIOD / 100;; // 40%
    config_gptim.pulse_dma_data[2] = (40 % 100) * PWM_PERIOD / 100;; // 60%

    struct rt_device_pwm *device = RT_NULL;
#ifdef SF32LB52X
    device = (struct rt_device_pwm *)rt_device_find(PWM_DEV_GTIM_NAME_52);
    if (!device)
    {
        LOG_I("find pwm3 err");
        return;
    }
#elif defined SF32LB58X
    device = (struct rt_device_pwm *)rt_device_find(PWM_DEV_GTIM_NAME_58);
    if (!device)
    {
        LOG_I("find pwm2 err");
        return;
    }
#endif

    rt_device_control((struct rt_device *)device, PWM_CMD_SET, (void *)&config_gptim);
#ifdef SF32LB52X
    LOG_I("hwp_gptim2_ccr1:%d", hwp_gptim2->CCR1); //52x
    LOG_I("hwp_gptim2_arr:%d", hwp_gptim2->ARR);
#endif

#ifdef SF32LB58X
    LOG_I("hwp_gptim1_ccr2:%d", hwp_gptim1->CCR2); //58x
    LOG_I("hwp_gptim1_arr:%d", hwp_gptim1->ARR);
#endif
    // Copy the calculated pulse value to the dma data
    for (size_t i = 0; i < global_array_length; i++)
    {
        config_gptim.dma_data[i] = (rt_uint16_t)global_pulse_values[i];
    }

    //check
    printf("DMA data before transfer:\n");
    for (size_t i = 0; i < config_gptim.data_len; i++)
    {
        printf("dma_data[%zu]: %u\n", i, config_gptim.dma_data[i]);
    }

    rt_device_control((struct rt_device *)device, PWM_CMD_ENABLE, (void *)&config_gptim); //dma_transfer

    free(config_gptim.dma_data);
    free(config_gptim.pulse_dma_data);

}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Start ATIM pwm demo!\n");
    pwm_dma_atim_set_example();
    rt_kprintf("ATIM pwm demo end!\n");

    rt_thread_mdelay(20);
#ifdef SF32LB52X
    MODIFY_REG(hwp_hpsys_cfg->ATIM1_PINR1, HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Msk, HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Msk);
#endif

    rt_kprintf("Start GPTIM pwm demo!\n");
    pwm_dma_gptim_set_example();
    rt_kprintf("GPTIM pwm demo end!\n");
    while (1)
    {
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }
    return RT_EOK;
}

