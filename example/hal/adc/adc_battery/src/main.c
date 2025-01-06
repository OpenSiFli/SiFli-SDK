#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "string.h"
#include "rtthread.h"

/* User code start from here --------------------------------------------------------*/
#include <stdlib.h>

//#define BSP_GPADC_USING_DMA 1
#define ADC_DEV_CHANNEL     7           /* ADC channe7 */

#define ADC_RATIO_ACCURATE          (1000)

#ifdef SF32LB55X
    #define ADC_MAX_VOLTAGE_MV     (1100)
#else
    #define ADC_MAX_VOLTAGE_MV     (3300)
#endif

#ifdef SF32LB55X
    // default value, they should be over write by calibrate
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 199.0;
    // mv per bit, if accuracy not enough, change to 0.1 mv or 0.01 mv later
    static float adc_vol_ratio = 3930.0; // 4296; //6 * ADC_RATIO_ACCURATE; //600; //6;
#elif defined(SF32LB56X)
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 822.0;
    // 0.001 mv per bit
    static float adc_vol_ratio = 1068.0; //
#else
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 822.0;
    // 0.001 mv per bit,
    static float adc_vol_ratio = 1068.0; //
#endif

static float adc_vbat_factor = 2.01;

// register data for max supported voltage, for A0, voltage = 1.1v, for RPO, voltage = 3.3v
static uint32_t adc_thd_reg;

/*
    This example demo:
        1. Configure ADC parameters
        2. Polling ADC value
*/
//校准
static int example_adc_calibration(uint32_t value1, uint32_t value2,
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

static void example_adc_vbat_fact_calib(uint32_t voltage, uint32_t reg)
{
    float vol_from_reg;

    // get voltage calculate by register data
    vol_from_reg = (reg - adc_vol_offset) * adc_vol_ratio / ADC_RATIO_ACCURATE;
    adc_vbat_factor = (float)voltage / vol_from_reg;
}

static int utest_adc_calib(void)
{
#ifndef SF32LB52X   //
    HAL_LCPU_CONFIG_ADC_T cfg;
    uint16_t len = (uint16_t)sizeof(HAL_LCPU_CONFIG_ADC_T);
    if (HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_ADC_CALIBRATION, (uint8_t *)&cfg, &len) == 0)
#else
    FACTORY_CFG_ADC_T cfg;
    int len = sizeof(FACTORY_CFG_ADC_T);
    if (BSP_CONFIG_get(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, len))
#endif
    {
        float off, rat;
        uint32_t vol1, vol2;
        if (cfg.vol10 == 0 || cfg.vol25 == 0) // not valid paramter
        {
            // no valid parameter
        }
        else
        {
#ifndef SF32LB55X
            cfg.vol10 &= 0x7fff;
            cfg.vol25 &= 0x7fff;
            vol1 = cfg.low_mv;
            vol2 = cfg.high_mv;
#else
            if ((cfg.vol10 & (1 << 15)) && (cfg.vol25 & (1 << 15))) // small range, use X1 mode
            {
                cfg.vol10 &= 0x7fff;
                cfg.vol25 &= 0x7fff;
                vol1 = 300;
                vol2 = 800;
            }
#endif
            example_adc_calibration(cfg.vol10, cfg.vol25, vol1, vol2, &off, &rat);
#ifdef SF32LB52X
            example_adc_vbat_fact_calib(cfg.vbat_mv, cfg.vbat_reg);
#endif
            //LOG_I("GPADC : %d mv reg %d, %d mv reg %d, offset %d, ratio %d, max reg %d\n",
            //    vol1, cfg.vol10, vol2, cfg.vol25, off, rat, adc_thd_reg);
        }
    }
    else
        return 1;

    return 0;
}

static float example_adc_get_float_mv(float value)
{
    float offset, ratio;
    // get offset
    offset = adc_vol_offset;
    ratio = adc_vol_ratio;
    if (value < offset)
        return 0;

    return (value - offset) * ratio / ADC_RATIO_ACCURATE;
}

static void adc_example(void)
{
    ADC_HandleTypeDef hadc;
    ADC_ChannelConfTypeDef ADC_ChanConf;
    uint32_t dst;
    uint32_t lslot = 7;
    HAL_StatusTypeDef ret = HAL_OK;

    // make sure set CORRECT ADC pin to correct mode
    //HAL_PIN_Set_Analog(PAD_PB27, 0);
#ifdef SF32LB55X
    lslot = ADC_DEV_CHANNEL;  // set slot to test
#elif defined(SF32LB52X)
    lslot = ADC_DEV_CHANNEL;
#else
    lslot = ADC_DEV_CHANNEL;
#endif

    int calib = utest_adc_calib();
    rt_kprintf("ADC Get calibration res %d\n", calib);

    // initial adc handle
    hadc.Instance = hwp_gpadc1;
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
        rt_kprintf("voltage %f mv\n", example_adc_get_float_mv((float)dst));
    }

    rt_kprintf("voltage %f mv\n", example_adc_get_float_mv((float)dst));

    HAL_ADC_Stop(&hadc);

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

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

