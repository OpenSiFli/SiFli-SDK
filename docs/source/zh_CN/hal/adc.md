# ADC

ADC HAL 提供用于访问 adc 外设寄存器的基本 API。
主要功能包括：
 - 10 位数据(A0), 12 位数据（PRO)。
 - DMA 支持。

:::{only} SF32LB58X or SF32LB56X or SF32LB52X or SF32LB55X
 - 最多支持 8 个 ADC 通道（通道 0 ~ 7），其中 `SF32LB52x` 的 **通道 7** 固定用于电池电压检测（VBAT 经电阻分压后输入）。
:::

:::{only} SF32LB57X
 - 最多支持 12 个 ADC 通道（通道 0 ~ 11），其中 **通道 11** 固定用于电池电压检测（VBAT 经电阻分压后输入）。
:::

## 使用 ADC HAL 驱动程序
每一比特的变化对应电压大约在 1 毫伏左右，具体需要根据每颗芯片单独校准。 
测试量程，A0上为 0~ 1.1V， PRO上为 0 ~ 3.3V.

在轮询模式下使用 ADC HAL 的示例：

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

## API参考
[](#hal-adc)
