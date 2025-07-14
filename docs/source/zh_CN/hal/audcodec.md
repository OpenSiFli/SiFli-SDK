# AUDCODEC

AUDCODEC(audio codec) 完成音频通路数据的AD/DA转换。支持的采样率包括8K，12K，16K，24K，32K，48K，11.025K，22.05K，44.1K，支持音量的缓慢变化。

## 使用 AUDCODEC HAL 驱动程序：
HAL AUDCODEC sample for TX：

```c
AUDCODEC_HandleTypeDef *haudcodec = &audcodec;

bf0_enable_pll(44100, 1); //RCC ENABLE
int res = HAL_AUDCODEC_Init(haudcodec);

HAL_AUDCODEC_Config_TChanel(haudcodec, 0, haudcodec->Init.dac_cfg);
res = HAL_AUDCODEC_Transmit_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_DAC_CH1], haudcodec->bufSize, HAL_AUDCODEC_DAC_CH1);
HAL_NVIC_EnableIRQ(AUDCODEC_DAC1_DMA_IRQ);
__HAL_AUDCODEC_HP_ENABLE(haudcodec);

HAL_AUDCODEC_Config_DACPath(haudcodec, 1);
HAL_AUDCODEC_Config_Analog_DACPath(haudcodec->Init.dac_cfg);
HAL_AUDCODEC_Config_DACPath(haudcodec, 0);
    
```
HAL AUDCODEC sample for RX：

```c
AUDCODEC_HandleTypeDef *haudcodec = &audcodec;

bf0_enable_pll(44100, 1); //RCC ENABLE
int res = HAL_AUDCODEC_Init(haudcodec);
HAL_AUDCODEC_Config_RChanel(haudcodec, 0, haudcodec->Init.adc_cfg);
res = HAL_AUDCODEC_Receive_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_ADC_CH0], haudcodec->bufSize, HAL_AUDCODEC_ADC_CH0);
HAL_NVIC_EnableIRQ(AUDCODEC_ADC0_DMA_IRQ);
HAL_AUCODEC_Refgen_Init();
HAL_AUDCODEC_Config_Analog_ADCPath(haudcodec->Init.adc_cfg);

/* enable AUDCODEC at last*/
__HAL_AUDCODEC_LP_ENABLE(haudcodec);
    
```


## API参考
[](#hal-audcodec)
