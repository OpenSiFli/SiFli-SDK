# NNACC Example

Source code path: example/hal/nnacc

## Supported Platforms
The example can run on the following development boards:
* sf32lb58-lcd_a128r32n1_dsi

## Overview
NNACC (Neural Network Accelerator) is an abbreviation for a dedicated hardware module provided in Sifli chips, specifically designed to accelerate the following tasks:

+ Convolutional computation (Conv2D)
+ Depthwise separable convolution (Depthwise Conv2D)
+ Gaussian filtering
+ Other AI inference related operations

This example demonstrates: verifying whether the hardware accelerator on the chip can correctly execute depthwise separable convolution (Depthwise Convolution) operations.

## Example Usage
```
\\Example core process
main()
├── Define convolution parameters (size, channels, stride, padding, etc.)
├── Allocate input buffers: input, wt, bias
├── Allocate output buffer: output
├── Initialize NN Accelerator hardware (drv_nnacc.c already completed)
├── Fill random data to simulate real input
├── Start hardware inference: nn_acc_start[synchronous start]nn_acc_start_IT[asynchronous start], modified by top macro definition selection
├── Wait for completion and verify results
└── Output test results (pass or fail)
```

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile (board=board type):
```
scons --board=sf32lb58-lcd_a128r32n1_dsi -j8
```
`build_sf32lb58-lcd_a128r32n1_dsi_hcpu\uart_download.bat`, select the port as prompted for download:

```
build_sf32lb58-lcd_a128r32n1_dsi_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

For detailed steps on compilation and download, please refer to the relevant introduction in [](/quickstart/get-started.md).

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example (supports nnacc)
```
**Note**: 
1. In sf32lb58x, the large core defaults to using hwp_nnacc1, and the small core defaults to using hwp_nnacc2. For specific chip configuration, please check the corresponding register.h file
2. sf32lb56x only has one hwp_nnacc1 on the large core, and the small core has no nnacc. For specific chip configuration, please check the corresponding register.h file

```

### menuconfig Configuration
Enable NN_ACC switch
![](./assets/image.png)

### Verification Results
#### Example output result display:
* log output:
```
SFBL
```
Boot log
```
Example log: ```
Buffers initialized successfully.
fill_with_random_data. 2000fc60 20012808 20012894  200182bc, sp=2000e35f
HAL_NNACC_Start.
(msh />Async operation complete.) --asynchronous mode log
Test passed: NNACC result is correct.
msh />
```

## Exception Diagnosis
If the expected log does not appear, you can troubleshoot from the following aspects:
* Whether the hardware supports nnacc
* Whether the corresponding switch is enabled in menuconfig

## Reference Documentation

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |7/2025 |Initial version | 