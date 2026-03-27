# AUDCODEC

AUDCODEC (audio codec) completes AD/DA conversion of audio path data. Supported
sampling rates include 8K, 12K, 16K, 24K, 32K, 48K, 11.025K, 22.05K, 44.1K, and
supports gradual volume changes.

## Using AUDCODEC HAL Driver:
HAL AUDCODEC sample for TX:

```c
AUDCODEC_HandleTypeDef *haudcodec = &amp;audcodec;

bf0_enable_pll(44100, 1); // Enable RCC
int res = HAL_AUDCODEC_Init(haudcodec);

HAL_AUDCODEC_Config_TChanel(haudcodec, 0, haudcodec-&gt;Init.dac_cfg);
res = HAL_AUDCODEC_Transmit_DMA(haudcodec, haudcodec-&gt;buf[HAL_AUDCODEC_DAC_CH1], haudcodec-&gt;bufSize, HAL_AUDCODEC_DAC_CH1);
HAL_NVIC_EnableIRQ(AUDCODEC_DAC1_DMA_IRQ);
__HAL_AUDCODEC_HP_ENABLE(haudcodec);

HAL_AUDCODEC_Config_DACPath(haudcodec, 1);
HAL_AUDCODEC_Config_Analog_DACPath(haudcodec-&gt;Init.dac_cfg);
HAL_AUDCODEC_Config_DACPath(haudcodec, 0);
```
HAL AUDCODEC sample for RX:

```c
AUDCODEC_HandleTypeDef *haudcodec = &amp;audcodec;

bf0_enable_pll(44100, 1); // Enable RCC
int res = HAL_AUDCODEC_Init(haudcodec);
HAL_AUDCODEC_Config_RChanel(haudcodec, 0, haudcodec-&gt;Init.adc_cfg);
res = HAL_AUDCODEC_Receive_DMA(haudcodec, haudcodec-&gt;buf[HAL_AUDCODEC_ADC_CH0], haudcodec-&gt;bufSize, HAL_AUDCODEC_ADC_CH0);
HAL_NVIC_EnableIRQ(AUDCODEC_ADC0_DMA_IRQ);
HAL_AUCODEC_Refgen_Init();
HAL_AUDCODEC_Config_Analog_ADCPath(haudcodec-&gt;Init.adc_cfg);

/** Enable AUDCODEC last*/
__HAL_AUDCODEC_LP_ENABLE(haudcodec);
```


## API Reference
[]
