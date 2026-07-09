# SDADC_HAL Example
Source code path: example/hal/adc/sdadc

## Overview
* Uses the HAL API to operate the SDADC directly, implementing single-channel voltage sampling.

## Supported Platforms
The example can run on the following development board.
+ sf32lb58 HDK

> **Note**: The SDADC IP is only available on SF32LB55X and SF32LB58X chips. This example was debugged on the 58 series.

## Hardware Connection
Mapping between SDADC channels and pins:

| Chip | CH1 | CH2 | CH3 | CH4 |
|------|-----|-----|-----|-----|
| SF32LB55X | PB_23 | PB_24 | PB_25 | PB_26 |
| SF32LB58X | PB_40 | PB_41 | PB_42 | PB_43 |

> CH0 is only used for BGA packages and is not available on regular GPIO.

Connect the target SDADC pin to an external voltage source (0 ~ 3.3V). **GND must share a common ground with the board.**

## Usage
### Build and Flash
* This example uses the HAL layer API. Make sure the SDADC module is enabled in `rtconfig.h`:

```c
#define BSP_USING_SDADC 1
```

* Switch to the example project directory and run scons to build (using ec-lb587 as an example):

```
scons --board=ec-lb587_hcpu -j8
```

* Run `build_xxx_hcpu\uart_download.bat` and follow the prompts to select the port for download.

#### Example Output:
Using CH3 (58x: PB_42) as an example, the register value and converted voltage are printed in a loop once per second:

```
======== SDADC HAL Example ========
Chip: SF32LB58X
Test channel: 3 (CH3)
Calibrate: sdadc_cal <reg1> <mV1> <reg2> <mV2>
SDADC factory calib not found, using defaults
  offset=961912, ratio=7068 (×1000000)
SDADC channel 3 pinmux configured (gain=1/4)
SDADC Init done
Waiting 2s for VREF to stabilize...
msh >
SDADC channel 3 configured, ready
Reading SDADC every 1 second...
SDADC ch3: reg=1055680 (min=1041502 max=1062947), voltage=662 mV
```

* In the floating state, the reading is about 600~700 mV (the SDADC has a high-impedance input; the floating value is affected by the environment, which is normal).
* After connecting a known voltage source, the reading should be close to the actual voltage.
* The register value `reg` is the 24-bit raw output, and `voltage` is the converted value in mV.

### Calibration
If the chip has not been SDADC-calibrated, you can send the following command to perform a two-point calibration:

```sh
sdadc_cal <reg1> <mV1> <reg2> <mV2>
```

For example, connect 1.2V and 2.6V voltage sources, record the two reg values, and send over the serial port:

```sh
sdadc_cal 1083587 1200 1268771 2600
```

Calibration values are lost after a reboot.


## Troubleshooting
* The log shows `Invalid SDADC channel`
  * `SDADC_TEST_CHANNEL` selects an invalid channel
* The log shows `SDADC poll timeout!`
  * The SDADC conversion did not complete; check the pin configuration and whether the reference voltage is normal.
* Readings are abnormal or deviate significantly from the actual voltage
  * Missing calibration values cause abnormal data; run `sdadc_cal` to calibrate manually.

## Revision History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |07/2026 |Initial version |
