/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
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

#include "app_common.h"

#define LOG_TAG "adc_app"
#include "log.h"

static rt_device_t adc_dev;
static rt_err_t adc_init(void)
{

    int i;
    uint32_t value, res, chnl;
    HAL_PIN_Set(PAD_PA28, GPIO_A28, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, GPIO_A29, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, GPIO_A33, PIN_NOPULL, 1);
    HAL_PIN_Set_Analog(PAD_PA28, 1);
    HAL_PIN_Set_Analog(PAD_PA29, 1);
    HAL_PIN_Set_Analog(PAD_PA30, 1);
    HAL_PIN_Set_Analog(PAD_PA31, 1);
    HAL_PIN_Set_Analog(PAD_PA32, 1);
    HAL_PIN_Set_Analog(PAD_PA33, 1);

    adc_dev = rt_device_find("bat1");
    if (adc_dev)
    {
        res = rt_device_open(adc_dev, RT_DEVICE_FLAG_RDONLY);
        rt_kprintf("adc open, res = %d \n", res);
        for (i = 0; i <= 7; i++)
        {
            chnl = i;
            res = rt_device_control(adc_dev, RT_ADC_CMD_ENABLE, (void *)chnl);
            res = rt_device_read(adc_dev, chnl, &value, 1);
            rt_kprintf("adc read channel %d : %d, res = %d \n", chnl, value, res);
            res = rt_device_control(adc_dev, RT_ADC_CMD_DISABLE, (void *)chnl);
        }
        //LOG_I("ADC %s channel %d enabled\n", argv[2], chnl);
    }
    else
    {
        LOG_I("Find %s device fail\n", adc_dev);
    }

    return res;
}

int cmd_adc_test(int argc, char *argv[])
{
//   if((argc < 2))
    rt_kprintf("argc:%d,argv[0]:%s,[1]:%s,[2]:%s,[3]:%s,[4]:%s\n ", argc, argv[0], argv[1], argv[2], argv[3], argv[4]);
    return (-RT_EINVAL);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_adc_test, __cmd_adc_test, Test adc_test driver);

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


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

extern int adc_get_data(int loop, int gap);

static void testadc(int argc, char **argv)
{

    int count = atoi(argv[2]);
    int dly = atoi(argv[3]);
    int res = adc_get_data(count, dly);
    rt_kprintf("Get adc value res %d, loop count %d, delay between each test %d us\n", res, count, dly);
}
MSH_CMD_EXPORT(testadc, forward testadc command);


int adc_get_data(int loop, int gap)
{
    // TODO: implement this using adc driver.
    return 0;
}

void adc_set_pinmux()
{
    // TODO: implement this using adc driver.

}
#ifdef BSP_GPADC_SUPPORT_MULTI_CH_SAMPLING
#define PM_DEBUG_PIN_TEST
#ifdef PM_DEBUG_PIN_TEST
#define DEBUG_PIN   5

#ifdef SOC_BF0_HCPU
#define PM_DEBUG_PIN_HIGH()      ((GPIO1_TypeDef *)hwp_gpio1)->DOSR0 |= (1UL << DEBUG_PIN)
#define PM_DEBUG_PIN_LOW()       ((GPIO1_TypeDef *)hwp_gpio1)->DOCR0 |= (1UL << DEBUG_PIN)
#define PM_DEBUG_PIN_TOGGLE()    ((GPIO1_TypeDef *)hwp_gpio1)->DOR0  ^= (1UL << DEBUG_PIN)
#define PM_DEBUG_PIN_ENABLE()    ((GPIO1_TypeDef *)hwp_gpio1)->DOESR0 |= (1UL << DEBUG_PIN)
#define PM_DEBUG_PIN_INIT()                         \
    do                                              \
    {                                               \
        GPIO_InitTypeDef GPIO_InitStruct;           \
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;    \
        GPIO_InitStruct.Pin = DEBUG_PIN;                   \
        GPIO_InitStruct.Pull = GPIO_NOPULL;         \
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct); \
    }                                               \
    while (0)


void pin_debug_init()
{
    HAL_PIN_Set(PAD_PA05, GPIO_A5, PIN_NOPULL, 1);

    PM_DEBUG_PIN_INIT();
    PM_DEBUG_PIN_ENABLE();
}

void pin_debug_toggle()
{
    PM_DEBUG_PIN_TOGGLE();
}


#endif

#endif

extern void BSP_GPIO_Set(int pin, int val, int is_porta);

__weak void BSP_adc_pinmux_config()
{

}

rt_err_t adc_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_err_t ret = RT_EOK;
    //rt_kprintf("%s;\n", __FUNCTION__);
    pin_debug_toggle();
    return ret;
}

static rt_err_t adc_init1(void)
{

    int i;
    uint32_t value, res, chnl;
    BSP_adc_pinmux_config();
    adc_dev = rt_device_find("bat1");
    if (adc_dev)
    {
        rt_device_set_rx_indicate(adc_dev, adc_rx_indicate);
    }
    else
    {
        LOG_I("Find %s device fail\n", adc_dev);
    }

#ifdef PM_DEBUG_PIN_TEST
    pin_debug_init();
    PM_DEBUG_PIN_HIGH();
#endif
    //BSP_GPIO_Set(34, 1, 1);

    return res;
}


#ifdef RT_USING_FINSH
rt_timer_t adc_timer_handle;

void adc_timeout_handler(void *parameter)
{
    for (uint8_t i = 0; i < 7; i++)
    {
        rt_kprintf("ch[%d] =%d;\n", i, rt_adc_read((rt_adc_device_t)adc_dev, i));
    }
}

int cmd_adc_all_ch(int argc, char *argv[])
{
    if (strcmp(argv[1], "-init") == 0)
    {
        adc_init1();
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        adc_timer_handle  = rt_timer_create("adc_timer", adc_timeout_handler,  NULL,
                                            rt_tick_from_millisecond(1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER); // 1000ms
        rt_timer_start(adc_timer_handle);
    }
    return (-RT_EINVAL);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_adc_all_ch, __cmd_adc_all_ch, Test adc_all_ch driver);
#endif
#endif





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
