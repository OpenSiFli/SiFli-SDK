# NNACC Example

Source Path: example/hal/nnacc

## Supported Platforms
This example can run on the following development boards:
* sf32lb58-lcd_a128r32n1_dsi

## Overview
NNACC (Neural Network Accelerator) is a dedicated hardware module in Sifli chips designed to accelerate the following tasks:

+ Convolutional calculations (Conv2D)
+ Depthwise separable convolution
+ Gaussian filtering
+ Other AI inference-related operations

This example verifies whether the hardware accelerator on the chip can correctly perform Depthwise Convolution operations.


## Example Usage
```
// Example core workflow
main()
├── Define convolution parameters (size, channels, stride, padding, etc.)
├── Allocate input buffers: input, wt, bias
├── Allocate output buffer: output
├── Initialize NN Accelerator hardware
├── Fill with random data to simulate real input
├── Start hardware inference: HAL_NNACC_Start()
├── Wait for completion and verify results
└── Output test results (pass or fail)
```

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile (board=board_type):
```
scons --board=sf32lb58-lcd_a128r32n1_dsi -j8
```
Execute `build_sf32lb58-lcd_a128r32n1_dsi_hcpu\uart_download.bat` and select the port as prompted to download:

```
build_sf32lb58-lcd_a128r32n1_dsi_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

For detailed steps on compilation and downloading, please refer to the relevant introduction in [](/quickstart/get-started.md).

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example (with nnacc support)
```
**Note**: 
1. In sf32lb58x, the big core uses hwp_nnacc1 by default, and the small core uses hwp_nnacc2 by default. For specific chip configurations, please refer to the corresponding register.h file
2. sf32lb56x only has one hwp_nnacc1 in the big core, and no nnacc in the small core. For specific chip configurations, please refer to the corresponding register.h file

```

### menuconfig Configuration
This example does not require menuconfig configuration


### Verification Results
#### Example output result display:
* log output:
```
SFBL
```boot log```
```example log: ```
Buffers initialized successfully.
testcase start.
fill_with_random_data. 2000fc60 20012808 20012894  200182bc, sp=2000e35f
HAL_NNACC_Start.
Test passed: NNACC result is correct.
msh />
```


## Troubleshooting
If the expected log does not appear, you can troubleshoot from the following aspects:
* Whether the hardware supports nnacc

## Reference Documentation

## Update History
| Version | Date   | Release Notes |
|:---|:---|:---|
| 0.0.1 | 7/2025 | Initial version |