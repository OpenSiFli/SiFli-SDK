
# ADC Usage Guide

## 1. ADC Introduction
Currently, the ADC has 8 available channels with a measurement range of 0 to 1.0 V. The ADC can test individual channels or perform multi-channel cyclic tests. It can stop after testing a set of data or continue to output data indefinitely.

## 2. ADC Configuration
The ADC configuration includes PINMUX configuration and ADC clock-related configuration, which are defined in `adc_config.h`. Users can modify them as needed. 
The PINMUX setup varies slightly for A0 and Z0. For Z0, when the PIN is used as ADC, you can select the ADC function directly during PIN configuration (e.g., `HAL_PIN_Set(PAD_PB_04, ADC_PIN, PIN_PULLUP, 0)`).
For A0, each pin that can be configured as GPADC corresponds to a different FUNCTION setting: GPADC_CH0, GPADC_CH1, and so on. When using these, set the pins according to the corresponding channels, for example, `HAL_PIN_Set(PAD_PB08, GPADC_CH0, PIN_NOPULL, 0)` for PAD_PB8 corresponding to GPADC channel 0. 
Alternatively, you can directly set the PIN to analog function without worrying about the specific analog function of each PIN, such as `HAL_PIN_Set_Analog(PAD_PB08, 0)` / `HAL_PIN_Set_Analog(PAD_PB13, 0)`.

For A0, the available PADs and their corresponding GPADC channels are as follows:
| PAD Number | Channel | Description |
|------------|---------|------------|
| PAD_PB08   | GPADC_CH0 | Connected to LCPU |
| PAD_PB10   | GPADC_CH1 | Connected to LCPU |
| PAD_PB12   | GPADC_CH2 | Connected to LCPU |
| PAD_PB13   | GPADC_CH3 | Connected to LCPU |
| PAD_PB16   | GPADC_CH4 | Connected to LCPU |
| PAD_PB17   | GPADC_CH5 | Connected to LCPU |
| PAD_PB18   | GPADC_CH6 | Connected to LCPU |
| PAD_PB19   | GPADC_CH7 | Connected to LCPU |

## 3. ADC Usage

In our system, the ADC is registered as a battery voltage device, which can be operated via device methods. The default device name is "bat1":

```c
uint32_t chnl = 1;
uint32_t value;

// find device
rt_device_t dev = rt_device_find(argv[2]);
if (dev)
{
    // open device
    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
    // enable adc
    rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)chnl);
    // read adc value
    rt_device_read(dev, chnl, &value, 1);
}
......
```

## 4. Voltage Calculation
The voltage value can be obtained through the ADC using the `HAL_ADC_GetValue` interface, or via the `read` interface if using `rt_device`. 
Each increment of the register value corresponds to a voltage increase of approximately 1 mV (the exact value may vary per chip, requiring factory calibration).
The relationship between ADC value and voltage is linear, and to calibrate for offset and slope accuracy, you can refer to the function `sifli_adc_calibration`.
The method involves determining a line through two known points, then using this line to calculate voltage for any subsequent register value.

Example conversion code for the register value to voltage:

```c
static uint32_t adc_vol_offset = 200;
static uint32_t adc_vol_ratio = 3930;

int sifli_adc_get_mv(uint32_t value)
{
    uint32_t offset, ratio;
    offset = adc_vol_offset;
    ratio = adc_vol_ratio;
    return (value - offset) * ratio / ADC_RATIO_ACCURATE;
}

int sifli_adc_calibration(uint32_t value1, uint32_t value2, uint32_t vol1, uint32_t vol2, uint32_t *offset, uint32_t *ratio)
{
    uint32_t gap1, gap2;
    if (offset == NULL || ratio == NULL)
        return 0;
    gap1 = value1 > value2 ? value1 - value2 : value2 - value1;
    gap2 = vol1 > vol2 ? vol1 - vol2 : vol2 - vol1;
    if (gap1 != 0)
    {
        *ratio = gap2 * ADC_RATIO_ACCURATE / gap1;
        adc_vol_ratio = *ratio;
    }
    *offset = value1 - vol1 * ADC_RATIO_ACCURATE / adc_vol_ratio;
    adc_vol_offset = *offset;
    return adc_vol_offset;
}
```
