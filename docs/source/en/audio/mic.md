# Mic Gain Debug

## EQ Tool
[Sifli_EQ](https://wiki.sifli.com/tools/index.html) The EQ tool can adjust the
digital microphone gain by modifying the variable `g_adc_volume` in the file
`drv_audprc.c`. The default value of this variable is 0, and reducing it
decreases the microphone gain. Most echo cancellation algorithms require
microphone sampling values to remain between 3k and 5k. The microphone sampling
values can be captured using the audio dump tool. Currently, the platform
software for the microphone is set to 16-bit, with a saturation range of -32768
to +32767. Sometimes, when the microphone gain is lowered, it may saturate at
lower values, affecting echo cancellation performance. At this point, the
g_adc_volume can be set to 0 to adjust the rough volumeThe code is located in
bf0_hal_audcodec.c (56x/58x platform) or bf0_hal_audcodec_m.c (52x/57x
platform)Here, the rough vol is set to 0xc. To address this, reduce the value
corresponding to ADC_CH0_CFG_ROUGH_VOL by changing it from 0xc to 0xa. By
appropriately lowering the rough vol, this digital gain value can be reduced to
improve the issue of excessively low saturation. The step size for the value of
ROUGH_VOL is 6 dB, with the corresponding settings as follows:
```
0   ----- -60dB  
1   ----- -54dB  
    ...  
    ...  
0xa -----   0dB  
0xb -----   6dB  
    ...  
0xf -----  30dB
```
FINE_VOL value is 0 ~ 0xc，corresponding 0~6db, step is 0.5db

```c
//56x/58x code
hacodec->Instance_lp->ADC_CH0_CFG = (1   << AUDCODEC_LP_ADC_CH0_CFG_ENABLE_Pos) |
                                            (0   << AUDCODEC_LP_ADC_CH0_CFG_HPF_BYPASS_Pos) |
                                            (0x7 << AUDCODEC_LP_ADC_CH0_CFG_HPF_COEF_Pos) |
                                            (0   << AUDCODEC_LP_ADC_CH0_CFG_STB_INV_Pos) |
                                            (0   << AUDCODEC_LP_ADC_CH0_CFG_DMA_EN_Pos) |
                                            (0xc << AUDCODEC_LP_ADC_CH0_CFG_ROUGH_VOL_Pos) |
                                            (0   << AUDCODEC_LP_ADC_CH0_CFG_FINE_VOL_Pos) |
                                            (1   << AUDCODEC_LP_ADC_CH0_CFG_DATA_FORMAT_Pos);
```

```c
//52x/57x
        hacodec-&gt;Instance-&gt;ADC_CH0_CFG = (1   &lt;&lt; AUDCODEC_ADC_CH0_CFG_ENABLE_Pos) |
                                         (0   &lt;&lt; AUDCODEC_ADC_CH0_CFG_HPF_BYPASS_Pos) |
                                         (0x7 &lt;&lt; AUDCODEC_ADC_CH0_CFG_HPF_COEF_Pos) |
                                         (0   &lt;&lt; AUDCODEC_ADC_CH0_CFG_STB_INV_Pos) |
                                         (0   &lt;&lt; AUDCODEC_ADC_CH0_CFG_DMA_EN_Pos) |
                                         (0xc &lt;&lt; AUDCODEC_ADC_CH0_CFG_ROUGH_VOL_Pos) |
                                         (0   &lt;&lt; AUDCODEC_ADC_CH0_CFG_FINE_VOL_Pos) |
                                         (1   &lt;&lt; AUDCODEC_ADC_CH0_CFG_DATA_FORMAT_Pos);
```

If saturation persists, decrease the value corresponding to the analog register
ADC1_CFG1_GC. Note that adjusting analog gain is generally not recommended. To
proceed, search for ADC1_CFG1 in the code and slightly decrease its value.
```c
ADC1_CFG1 |= (0x1E &lt;&lt; AUDCODEC_LP_ADC1_CFG1_GC_Pos);
```
