/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include <string.h>
#include <stdlib.h>
#include "rtthread.h"

/* User code start from here --------------------------------------------------------*/

#if defined(SF32LB58X)
    /* 58x: SDADC on PB_40 ~ PB_43 */
    #define SDADC_CH1_PAD       PAD_PB40
    #define SDADC_CH1_GPIO      GPIO_B40
    #define SDADC_CH2_PAD       PAD_PB41
    #define SDADC_CH2_GPIO      GPIO_B41
    #define SDADC_CH3_PAD       PAD_PB42
    #define SDADC_CH3_GPIO      GPIO_B42
    #define SDADC_CH4_PAD       PAD_PB43
    #define SDADC_CH4_GPIO      GPIO_B43
#elif defined(SF32LB55X)
    /* 55x: SDADC on PB_23 ~ PB_26 */
    #define SDADC_CH1_PAD       PAD_PB23
    #define SDADC_CH1_GPIO      GPIO_B23
    #define SDADC_CH2_PAD       PAD_PB24
    #define SDADC_CH2_GPIO      GPIO_B24
    #define SDADC_CH3_PAD       PAD_PB25
    #define SDADC_CH3_GPIO      GPIO_B25
    #define SDADC_CH4_PAD       PAD_PB26
    #define SDADC_CH4_GPIO      GPIO_B26
#else
    #error "SDADC is only supported on SF32LB55X and SF32LB58X"
#endif

/* Select test channel: CH3 (PB_42 on 58x, PB_25 on 55x) */
#define SDADC_TEST_CHANNEL      HAL_SDADC_GPIO_CHN2   /* CH3: GPIO_CHN2 = 3 */

#define ADC_RATIO_ACCURATE      (1000000)   /* ratio accuracy multiplier (use 1M for SDADC) */
#define ADC_SW_AVRA_CNT     (22)       /* number of samples for averaging */

#define SDADC_REG_AT_0V         (961912)

static uint32_t adc_offset = SDADC_REG_AT_0V;
static uint32_t adc_ratio  = 7068;

static SDADC_HandleTypeDef hadc;
static uint32_t g_sdadc_gain_nume = 1;
static uint32_t g_sdadc_gain_deno = 4;

/*
 * Convert SDADC register value to pin voltage in mV.
 */
static int sifli_sdadc_get_mv(uint32_t reg_value)
{
    return (int)((int64_t)(reg_value - adc_offset) * (int64_t)adc_ratio / ADC_RATIO_ACCURATE);
}

/*
 * Two-point calibration. Provide register values at two known voltages.
 *
 * For 58x, recommended calibration points: 1.0V and 2.5V.
 * For 55x, recommended calibration points: 0.3V and 0.8V.
 *
 * Returns 0 on success, -1 if points are invalid.
 */
static int sifli_sdadc_calibration(uint32_t reg1, uint32_t mv1,
                                   uint32_t reg2, uint32_t mv2)
{
    uint32_t gap_reg, gap_mv;

    gap_reg = (reg1 > reg2) ? (reg1 - reg2) : (reg2 - reg1);
    gap_mv  = (mv1  > mv2)  ? (mv1  - mv2)  : (mv2  - mv1);

    if (gap_reg == 0 || gap_mv == 0)
        return -1;

    adc_ratio = (uint32_t)((uint64_t)gap_mv * ADC_RATIO_ACCURATE / gap_reg);

    /* Compute offset using point1 */
    adc_offset = reg1 - (uint32_t)((uint64_t)mv1 * ADC_RATIO_ACCURATE / adc_ratio);

    return 0;
}

/*
 * Load factory calibration data from BSP config.
 */
static void sdadc_load_factory_calib(void)
{
    HAL_LCPU_CONFIG_SDMADC_T cfg;
    int len = (int)sizeof(HAL_LCPU_CONFIG_SDMADC_T);

    if (BSP_CONFIG_get(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg, len)
            && cfg.value != 0 && cfg.vol_mv != 0)
    {
        /* Factory calibration: cfg.value @ cfg.vol_mv (gain was likely 1) */
        /* Need to compensate for our gain setting */
        uint32_t reg_at_0v = SDADC_REG_AT_0V; /* VREF_PWR_BASE/2 */
        uint32_t reg_at_cal = reg_at_0v +
                              (cfg.value - reg_at_0v) * g_sdadc_gain_nume / g_sdadc_gain_deno;

        sifli_sdadc_calibration(reg_at_0v, 0, reg_at_cal, cfg.vol_mv);

        rt_kprintf("SDADC factory calib: %lu mV -> reg %lu\n", cfg.vol_mv, cfg.value);
    }
    else
    {
        rt_kprintf("SDADC factory calib not found, using defaults\n");
    }
    rt_kprintf("  offset=%lu, ratio=%lu (×%lu)\n",
               adc_offset, adc_ratio, (uint32_t)ADC_RATIO_ACCURATE);
}

/*
 * Get the PAD and GPIO definition for the selected channel.
 */
static void sdadc_get_pin_info(uint32_t channel, uint32_t *pad, uint32_t *gpio)
{
    switch (channel)
    {
    case HAL_SDADC_GPIO_CHN0: /* CH1 */
        *pad = SDADC_CH1_PAD;
        *gpio = SDADC_CH1_GPIO;
        break;
    case HAL_SDADC_GPIO_CHN1: /* CH2 */
        *pad = SDADC_CH2_PAD;
        *gpio = SDADC_CH2_GPIO;
        break;
    case HAL_SDADC_GPIO_CHN2: /* CH3 */
        *pad = SDADC_CH3_PAD;
        *gpio = SDADC_CH3_GPIO;
        break;
    case HAL_SDADC_GPIO_CHN3: /* CH4 */
        *pad = SDADC_CH4_PAD;
        *gpio = SDADC_CH4_GPIO;
        break;
    default:
        *pad = 0;
        *gpio = 0;
        break;
    }
}

static void sdadc_example_init(void)
{
    SDADC_ChannelConfTypeDef ADC_ChanConf;
    SDADC_GainConfTypeDef gain;
    SDADC_AccurateConfTypeDef accu;
    uint32_t pad, gpio;
    uint32_t lslot = SDADC_TEST_CHANNEL;

    /* 1. Load calibration */
    sdadc_load_factory_calib();

    /* 2. Set pinmux for the target SDADC channel */
    sdadc_get_pin_info(lslot, &pad, &gpio);
    if (pad == 0)
    {
        rt_kprintf("Invalid SDADC channel %lu\n", lslot);
        return;
    }
    HAL_PIN_Set(pad, gpio, PIN_NOPULL, 0);
    HAL_PIN_Select(pad, 10, 0);
    rt_kprintf("SDADC channel %lu pinmux configured (gain=%lu/%lu)\n",
               lslot, g_sdadc_gain_nume, g_sdadc_gain_deno);

    /* 3. Initialize SDADC handle */
    memset(&hadc, 0, sizeof(hadc));
    hadc.Instance = hwp_sdadc;
    hadc.Init.adc_se = 1;                       /* single-end mode */
    hadc.Init.src_sel = HAL_SDADC_SRC_SW;       /* software trigger */
    hadc.Init.vref_sel = HAL_SDADC_VERF_INTERNAL; /* internal VREF */
    hadc.Init.dma_en = 0;                       /* no DMA */
    hadc.Init.en_slot = 0;                      /* will be updated by enable/config */
    hadc.Init.conti_mode = 0;                   /* single conversion */
    hadc.Init.diff_sel = 0;                     /* single-end */
    hadc.Init.dsample_mode = 0;
    HAL_SDADC_Init(&hadc);

    rt_kprintf("SDADC Init done\n");

    /* 4. Wait 2 seconds for reference voltage to stabilize */
    rt_kprintf("Waiting 2s for VREF to stabilize...\n");
    rt_thread_mdelay(2000);

    /* 5. Configure gain */
    gain.gain_deno = g_sdadc_gain_deno;
    gain.gain_nume = g_sdadc_gain_nume;
    HAL_SDADC_ConfigGain(&hadc, &gain);

    /* 6. Configure accuracy parameters */
    accu.chop1_num = 0x9c;
    accu.chop2_num = 0xc9;
    accu.chop3_num = 0x1ff;
    accu.chop_ref_num = 0x9c;
    accu.sample_num = 0xe0;
    HAL_SDADC_ConfigAccu(&hadc, &accu);

    /* 7. Enable slot */
    HAL_SDADC_EnableSlot(&hadc, lslot, 1);

    /* 8. Configure channel */
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = lslot;
    ADC_ChanConf.shift_num = 2;
    ADC_ChanConf.pchnl_sel = lslot;
    ADC_ChanConf.slot_en = 1;
    HAL_SDADC_ConfigChannel(&hadc, &ADC_ChanConf);

    rt_kprintf("SDADC channel %lu configured, ready\n", lslot);
}

static void sdadc_example_read(void)
{
    HAL_StatusTypeDef ret;
    uint32_t data[ADC_SW_AVRA_CNT];
    uint32_t total;
    uint32_t lslot = SDADC_TEST_CHANNEL;
    int i, j;
    int voltage_mv;

    /* Start SDADC */
    HAL_SDADC_Start(&hadc);

    for (i = 0; i < ADC_SW_AVRA_CNT; i++)
    {
        if (i != 0)
        {
            /* Re-trigger conversion (first is auto-triggered by Start) */
            hadc.Instance->TRIG |= SDADC_TRIG_ADC_START;
        }

        /* Wait for conversion to complete */
        ret = HAL_SDADC_PollForConversion(&hadc, 100);
        if (ret != HAL_OK)
        {
            rt_kprintf("SDADC poll timeout!\n");
            return;
        }

        /* Get register value */
        data[i] = HAL_SDADC_GetValue(&hadc, lslot);

        /* Short delay between samples for stability */
        rt_thread_mdelay(5);
    }

    /* Bubble sort */
    for (i = 0; i < ADC_SW_AVRA_CNT - 1; i++)
        for (j = 0; j < ADC_SW_AVRA_CNT - 1 - i; j++)
            if (data[j] > data[j + 1])
            {
                uint32_t tmp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = tmp;
            }

    /* Drop min and max, average the middle samples */
    total = 0;
    for (i = 1; i < ADC_SW_AVRA_CNT - 1; i++)
        total += data[i];

    uint32_t avg_reg = total / (ADC_SW_AVRA_CNT - 2);

    /* Convert to voltage */
    voltage_mv = sifli_sdadc_get_mv(avg_reg);

    rt_kprintf("SDADC ch%lu: reg=%lu (min=%lu max=%lu), voltage=%d mV\n",
               lslot, avg_reg, data[0], data[ADC_SW_AVRA_CNT - 1], voltage_mv);
}

/* ---- Shell command for runtime calibration ---- */
static int cmd_sdadc_cal(int argc, char *argv[])
{
    uint32_t reg1, mv1, reg2, mv2;

    if (argc < 4)
    {
        rt_kprintf("Usage: sdadc_cal <reg1> <mV1> <reg2> <mV2>\n");
        rt_kprintf("  58x recommended points: 1.0V and 2.5V\n");
        rt_kprintf("  Example: sdadc_cal 1103405 1000 1320000 2500\n");
        rt_kprintf("  Current: offset=%lu ratio=%lu\n", adc_offset, adc_ratio);
        return 0;
    }

    if (argc < 5)
    {
        /* Show current params */
        rt_kprintf("offset=%lu, ratio=%lu\n", adc_offset, adc_ratio);
        return 0;
    }

    reg1 = atoi(argv[1]);
    mv1  = atoi(argv[2]);
    reg2 = atoi(argv[3]);
    mv2  = atoi(argv[4]);

    if (sifli_sdadc_calibration(reg1, mv1, reg2, mv2) != 0)
    {
        rt_kprintf("Calibration failed: invalid points\n");
        return -1;
    }

    rt_kprintf("Calibration OK:\n");
    rt_kprintf("  Point1: reg=%lu @ %lu mV\n", reg1, mv1);
    rt_kprintf("  Point2: reg=%lu @ %lu mV\n", reg2, mv2);
    rt_kprintf("  offset=%lu, ratio=%lu\n", adc_offset, adc_ratio);

    /* Verify */
    rt_kprintf("  Verify: reg1->%d mV, reg2->%d mV\n",
               sifli_sdadc_get_mv(reg1), sifli_sdadc_get_mv(reg2));

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sdadc_cal, __cmd_sdadc_cal, SDADC calibration);

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n======== SDADC HAL Example ========\n");
    rt_kprintf("Chip: %s\n",
#if defined(SF32LB58X)
               "SF32LB58X"
#elif defined(SF32LB55X)
               "SF32LB55X"
#endif
              );
    rt_kprintf("Test channel: %lu (CH3)\n", (uint32_t)SDADC_TEST_CHANNEL);
    rt_kprintf("Calibrate: sdadc_cal <reg1> <mV1> <reg2> <mV2>\n");

    sdadc_example_init();

    rt_kprintf("\nReading SDADC every 1 second...\n\n");

    while (1)
    {
        sdadc_example_read();
        rt_thread_mdelay(1000);
    }

    return 0;
}
