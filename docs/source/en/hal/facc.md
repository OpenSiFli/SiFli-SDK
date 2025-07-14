# FACC

HAL FACC module provides hardware acceleration for 8bit/16bit convolution, FIR and IIR filtering. FIR and convolution also add support for CMSIS-DSP.

```{note}
55x does not support this module. 58x has one FACC module each in HCPU/LCPU, 56x only provides one FACC module in HCPU.
```

For detailed API documentation, refer to [FACC](#hal-facc)

## Using HAL FACC

```c
// Convolution example
// facc_out = dsp_conv_conv_coef_b * dsp_conv_conv_input;
uint32_t dsp_conv_conv_param = 0x200;                     // Parameter structure reference #FACC_ConfigTypeDef
param = (FACC_ConfigTypeDef *)&dsp_conv_conv_param;
param->last_sel = 1;                                      // This is the last input block
HAL_FACC_Config(&hfacc, param);                                     // Set FACC
HAL_FACC_SetCoeff(&hfacc, dsp_conv_conv_coef_b, (uint16_t)sizeof(dsp_conv_conv_coef_b), NULL, 0, 0); // Input dsp_conv_conv_coef_b
HAL_FACC_Start(&hfacc, dsp_conv_conv_input, facc_out, sizeof(dsp_conv_conv_input)); // Start convolution calculation. Synchronous completion.


// FIR example, using interrupt mode, signal needs to be set in interrupt.
uint32_t fir_fir_param = 0x0;
param = (FACC_ConfigTypeDef *)&fir_fir_param;                       // Parameter structure reference #FACC_ConfigTypeDef
HAL_FACC_Config(&hfacc, param);                                     
HAL_FACC_SetCoeff(&hfacc, fir_fir_coef_b, (uint16_t)sizeof(fir_fir_coef_b), NULL, 0, 0); // Input filter function coefficients
NVIC_EnableIRQ(FACC1_IRQn);
HAL_FACC_Start_IT(&hfacc, fir_fir_input, facc_out, sizeof(fir_fir_input)); // Start FIR calculation, using interrupt mode, interrupt completion will generate FACC1 interrupt
...         // Synchronous completion interrupt

// IIR example
uint32_t iir_iir_param = 0x42;
param = (FACC_ConfigTypeDef *)&iir_iir_param;                       // Parameter structure reference #FACC_ConfigTypeDef
HAL_FACC_Config(&hfacc, param);                                     
HAL_FACC_SetCoeff(&hfacc, iir_iir_coef_b, (uint16_t)sizeof(iir_iir_coef_b), iir_iir_coef_a, (uint16_t)sizeof(iir_iir_coef_a), 0); // Input filter function coefficients
HAL_FACC_Start(&hfacc, iir_iir_input, facc_out, sizeof(iir_iir_input)); // Start IIR calculation. Synchronous completion.

```
## API Reference
[](#hal-facc)
