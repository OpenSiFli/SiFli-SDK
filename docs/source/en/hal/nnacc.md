
# NNACC

SiFli implements an NNACC (Neural Network Accelerator) to accelerate artificial intelligence on IoT devices. It can be used for speech/vision recognition applications and other AI applications using convolutional neural networks (CNNs).
Supports:<br>
 - Supports CMSIS-NN, 8 times faster than ARM DSP implementation in CMSIS-NN. <br>
 - Can achieve low MIPS requirement keyword recognition.<br>

## Driver Configuration
Firmware can use NNACC with CMSIS-NN enabled. Configuration is done using the menuconfig tool and typically saved in a C header file. By default, configuration is saved as rtconfig.h.
The following shows flags defined in a project header file that will enable NNACC support.

```{note}
`SF32LB55X/56X` HCPU has one NNACC (NNACC1), SF32LB58X HCPU/LCPU each has one NNACC (NNACC1/NNACC2 respectively).
```

Once configuration is done, users need to include the header file in all source code that needs to access the driver.

```c
#define BSP_USING_CMSIS_NN
#define BSP_USING_NN_ACC
```

## Using NNACC
SiFli SDK implements CNN functions in CMSIS-NN. Users can still use CMSIS-NN interfaces in their AI applications. For detailed introduction to Accelerator APIs, please refer to CNN Accelerator.
The following CMSIS-NN functions have acceleration from SiFli SDK:
- arm_convolve_1x1_HWC_q7_fast_nonsquare
- arm_convolve_HWC_q7_basic_nonsquare
- arm_depthwise_separable_conv_HWC_q7_nonsquare

## API Reference
[](../api/hal/nnacc.md)
