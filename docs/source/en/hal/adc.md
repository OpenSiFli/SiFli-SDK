# ADC

ADC HAL provides basic APIs for accessing ADC peripheral registers.
Main features include:
 - Support up to 8 ADC channels.
 - 10-bit data (A0), 12-bit data (PRO).
 - DMA support.

## Using ADC HAL Driver
ADC supports 8 channels, with each bit change corresponding to approximately 1 millivolt, requiring individual calibration for each chip.
Test range: 0~ 1.1V on A0, 0 ~ 3.3V on PRO.

Example of using ADC HAL in polling mode:

```c
ADC_HandleTypeDef hadc;
uint32_t channel, value, timeout;
ADC_ChannelConfTypeDef ADC_ChanConf;

/* initial handle */
hadc.Instance = hwp_gpadc1;
#ifndef SF32LB55X
hadc.Init.data_samp_delay = 2;
hadc.Init.conv_width = 24;
hadc.Init.sample_width = 22;
#else
hadc.Init.clk_div = 31;  // set frequency
#endif
hadc.Init.adc_se = 1;   // single channel
hadc.Init.adc_force_on = 0;
hadc.Init.dma_en = 0;   // no dma
hadc.Init.en_slot = 0;  // default slot, update by enable and configure
hadc.Init.op_mode = 0;  // single mode, not continous
/* initial ADC controller */
ret = HAL_ADC_Init(&hadc);


/* enable ADC */
channel = 1;
HAL_ADC_EnableSlot(&hadc, channel, 1);

/* configure ADC */
rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
ADC_ChanConf.Channel = channel;
ADC_ChanConf.pchnl_sel = channel;
ADC_ChanConf.slot_en = 1;
HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

/* start ADC */
HAL_ADC_Start(&hadc);

/* Wait for the ADC to convert */
timeout = 100; // 100 ms
HAL_ADC_PollForConversion(&hadc, tmieout);

/* Get ADC value */
value = (rt_uint32_t)HAL_ADC_GetValue(&hadc, channel);

...
```

## API Reference
[](#hal-adc)
