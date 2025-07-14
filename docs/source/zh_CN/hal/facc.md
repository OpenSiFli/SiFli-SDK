# FACC

HAL FACC模块提供8bit/16bit的卷积，FIR和IIR滤波的硬件加速。其中FIR和卷积还添加了对CMSIS-DSP的支持。

```{note}
55x不支持本模块， 58x 在HCPU/LCPU各有一个FACC模块, 56x 仅在HCPU里面提供一个FACC模块。
```

详细的API说明参考 [FACC](#hal-facc)

## 使用HAL FACC

```c
// 卷积示例
// facc_out = dsp_conv_conv_coef_b * dsp_conv_conv_input;
uint32_t dsp_conv_conv_param = 0x200;                     // 参数结构参考 #FACC_ConfigTypeDef
param = (FACC_ConfigTypeDef *)&dsp_conv_conv_param;
param->last_sel = 1;                                      // 这个是最后一个输入块
HAL_FACC_Config(&hfacc, param);                                     // 设置FACC
HAL_FACC_SetCoeff(&hfacc, dsp_conv_conv_coef_b, (uint16_t)sizeof(dsp_conv_conv_coef_b), NULL, 0, 0); // 输入 dsp_conv_conv_coef_b
HAL_FACC_Start(&hfacc, dsp_conv_conv_input, facc_out, sizeof(dsp_conv_conv_input)); // 启动卷积计算. 同步完成。


// FIR示例, 使用中断模式，在中断中需要设置信号。
uint32_t fir_fir_param = 0x0;
param = (FACC_ConfigTypeDef *)&fir_fir_param;                       // 参数结构参考 #FACC_ConfigTypeDef
HAL_FACC_Config(&hfacc, param);                                     
HAL_FACC_SetCoeff(&hfacc, fir_fir_coef_b, (uint16_t)sizeof(fir_fir_coef_b), NULL, 0, 0); // 输入滤波器函数系数
NVIC_EnableIRQ(FACC1_IRQn);
HAL_FACC_Start_IT(&hfacc, fir_fir_input, facc_out, sizeof(fir_fir_input)); // 启动FIR计算，采用中断模式，中断完成会产生FACC1中断
...         // 同步完成中断

// IIR示例
uint32_t iir_iir_param = 0x42;
param = (FACC_ConfigTypeDef *)&iir_iir_param;                       // 参数结构参考 #FACC_ConfigTypeDef
HAL_FACC_Config(&hfacc, param);                                     
HAL_FACC_SetCoeff(&hfacc, iir_iir_coef_b, (uint16_t)sizeof(iir_iir_coef_b), iir_iir_coef_a, (uint16_t)sizeof(iir_iir_coef_a), 0); // 输入滤波器函数系数
HAL_FACC_Start(&hfacc, iir_iir_input, facc_out, sizeof(iir_iir_input)); // 启动IIR计算. 同步完成。

```
## API参考
[](#hal-facc)

