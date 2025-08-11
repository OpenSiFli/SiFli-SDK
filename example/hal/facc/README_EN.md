# FACC Example

Source Code Path: example/hal/facc

## Supported Platforms
The example can run on the following development boards:
* sf32lb56-lcd_n16r12n1
* sf32lb56-lcd_a128r12n1
* sf32lb58-lcd_a128r32n1_dsi
* sf32lb58-lcd_n16r64n4

## Example Overview
* This example demonstrates how to initialize and use a hardware accelerator module called FACC for performing FIR (Finite Impulse Response) filtering operations, and verifies whether the output results are correct.

## Example Usage

### Hardware Requirements
Before running this example, you need to prepare a supported development board and a USB data cable.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb58-lcd_n16r64n4 -j8
```
Execute the flashing command:
```
build_sf32lb58-lcd_n16r64n4_hcpu\download.bat
```

### Example Output
* After successful flashing, the serial port will print filtering information
* If the log shows "ok" after comparison, it indicates successful filtering
![facc_result](assets/facc1.png)

### Troubleshooting
* Log information for filtering errors
![facc_fail](assets/facc2.png)
* First check if the input parameters are correct. If the input parameters are incorrect, please verify them. If the input parameters are correct, please check if the filter is working properly
* Check if the array used to store filtered data is out of bounds: `uint8_t facc_output [1024];`