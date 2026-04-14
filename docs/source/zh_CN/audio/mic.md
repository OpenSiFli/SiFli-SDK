# Mic增益调试

## EQ工具 
[Sifli_EQ](https://wiki.sifli.com/tools/index.html)
EQ工具可以调试数字麦增益，修改的变量为drv_audprc.c里的g_adc_volume，这个变量默认值为0，减小可以降低mic的增益，一般的回声消除算法都要求mic采样值在3k~5k之间，mic的采样值可以用audio dump工具抓取. mic的平台软件目前都设置的为16位，饱和值在-32768~+32767， 有时候mic的增益调低了，容易在较小值位置饱和，影响回声消除。
这时可以把g_adc_volume改为0，调节rough volume. 代码在bf0_hal_audcodec.c（56x/58x platform）或bf0_hal_audcodec_m.c (52x/57x platform)，这里rough vol为 0xc， 把 ADC_CH0_CFG_ROUGH_VOL 对应的值减小来解决，可以从0xc改为0xa， 通过适当降低rough vol，降低这个数字增益值改善饱和值过低问题。
ROUGH_VOL的值步长为6db，设置的值对应关系如下：
```
0   ----- -60db  
1   ----- -54db  
    ...  
    ...  
0xa -----   0db  
0xb -----   6db  
    ...  
0xf -----  30db  
```
FINE_VOL的值为0 ~ 0xc，对应0~6db, 步长为0.5db

```c
//56x/58x的代码
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
        hacodec->Instance->ADC_CH0_CFG = (1   << AUDCODEC_ADC_CH0_CFG_ENABLE_Pos) |
                                         (0   << AUDCODEC_ADC_CH0_CFG_HPF_BYPASS_Pos) |
                                         (0x7 << AUDCODEC_ADC_CH0_CFG_HPF_COEF_Pos) |
                                         (0   << AUDCODEC_ADC_CH0_CFG_STB_INV_Pos) |
                                         (0   << AUDCODEC_ADC_CH0_CFG_DMA_EN_Pos) |
                                         (0xc << AUDCODEC_ADC_CH0_CFG_ROUGH_VOL_Pos) |
                                         (0   << AUDCODEC_ADC_CH0_CFG_FINE_VOL_Pos) |
                                         (1   << AUDCODEC_ADC_CH0_CFG_DATA_FORMAT_Pos);
```

如果还是饱和，就修改模拟ADC1_CFG1_GC对应的值减小，这个不建议, 一般不改模拟增益，代码中搜索ADC1_CFG1，把对应的值减小一点
```c
ADC1_CFG1 |= (0x1E << AUDCODEC_LP_ADC1_CFG1_GC_Pos);
```
