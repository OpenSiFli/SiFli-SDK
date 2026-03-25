# ADC User Guide

## 1. Introduction
The ADC currently supports eight available channels with a measurement range of
0 to 1.0 V per unit. It supports both single-channel and multi-channel cyclic
scanning modes. Operation can be configured to stop after capturing a single set
of data or to provide continuous data output.

## 2. ADC Configuration
ADC configuration involves setting up the PINMUX and the ADC clock. Clock
parameters are defined in `adc_config.h` and can be modified as needed. <br/>
PINMUX configuration differs slightly between A0 and Z0 revisions. On Z0, if a
pin is used for ADC, the ADC function can be selected directly when configuring
the pin (e.g., `HAL_PIN_Set(PAD_PB_04, ADC_PIN, PIN_PULLUP, 0)`). <br/> On A0,
each pin configurable as a GPADC corresponds to a specific function:
`GPADC_CH0`, `GPADC_CH1`, etc. <br/> When using A0, pins must be configured
according to their specific mapping. For example, since `PAD_PB08` corresponds
to GPADC Channel 0, it should be set as: `HAL_PIN_Set(PAD_PB08, GPADC_CH0,
PIN_NOPULL, 0)`. <br/> Similarly, since `PAD_PB13` corresponds to GPADC Channel
3, it should be set as: `HAL_PIN_Set(PAD_PB13, GPADC_CH3, PIN_NOPULL, 0)`. <br/>
Alternatively, a pin can be set directly to analog mode without explicitly
specifying the channel mapping, for example: `HAL_PIN_Set_Analog(PAD_PB08, 0)`
or `HAL_PIN_Set_Analog(PAD_PB13, 0)`. <br/> The following table maps the pads
available for GPADC on the A0 revision to their respective functions:<br/>
| Pad Number | Channel Number | Description    |
| ---------- | -------------- | -------------- |
| PAD_PB08   | GPADC_CH0      | Mapped to LCPU |
| PAD_PB10   | GPADC_CH1      | Mapped to LCPU |
| PAD_PB12   | GPADC_CH2      | Mapped to LCPU |
| PAD_PB13   | GPADC_CH3      | Mapped to LCPU |
| PAD_PB16   | GPADC_CH4      | Mapped to LCPU |
| PAD_PB17   | GPADC_CH5      | Mapped to LCPU |
| PAD_PB18   | GPADC_CH6      | Mapped to LCPU |
| PAD_PB19   | GPADC_CH7      | Mapped to LCPU |

## 3. ADC Application

In this system, the ADC is registered by default as a battery voltage device. It
can be operated using standard device drivers (read/write). The default device
name is "bat1":<br/>

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
	rt_device_read(dev, chnl, &amp;value, 1);
}
......
```

## 4. Voltage Calculation
The ADC digital value can be obtained via the HAL_ADC_GetValue interface (or the
read interface when using rt_device). <br/> Each increment in the register value
corresponds to a voltage increase of approximately 1 mV (this varies between
units and requires factory calibration). <br/> There is a linear relationship
between the ADC raw value and the actual voltage. For details on how to
calibrate the precise offset and slope, refer to the sifli_adc_calibration
function. <br/> The calibration principle utilizes a two-point linear
interpolation: a line is defined by two known coordinates, allowing any
subsequent register value (one coordinate) to be converted into its
corresponding voltage (the other coordinate). <br/> Operation process: <br/> Use
ATE equipment to input two precise, known voltages (e.g., 0.3V and 0.8V; avoid
0V or the maximum range). Read the corresponding ADC register values for both.
By storing these two sets of voltage and register values, the application can
determine the line parameters, including the offset (the theoretical register
value at 0V, though accuracy decreases near 0V) and the slope (voltage increment
per register LSB). These values are then saved for future voltage
measurements.<br/>

Reference code for converting register values to voltage:<br/>

```c
// default value, they should be over write by calibrate
// it should be register value offset vs 0 v value.
static uint32_t adc_vol_offset = 200;
// mv per bit, if accuracy not enough, change to 0.1 mv or 0.01 mv later
static uint32_t adc_vol_ratio = 3930; //6;

/**
* @brief  Converts a register value to voltage.
* @param[in]  value Register value.
* @retval Voltage in mV.

int sifli_adc_get_mv(uint32_t value)
{
    uint32_t offset, ratio;
    // get offset
    offset = adc_vol_offset;
    // get ratio
    ratio = adc_vol_ratio;

    return (value - offset) * ratio / ADC_RATIO_ACCURATE;
}

/**
* @brief  Calculate voltage offset and ratio via calibration.
* @param[in]  value1 Register value 1.
* @param[in]  value2 Register value 2.
* @param[in]  vol1   Voltage 1 in mV.
* @param[in]  vol2   Voltage 2 in mV.
* @param[out] offset Register value offset relative to 0V.
* @param[out] ratio  Voltage (mV) per bit; scaled by ADC_RATIO_ACCURATE (e.g., 100 for 0.01 mV resolution).
* @retval Measured offset.
*/
int sifli_adc_calibration(uint32_t value1, uint32_t value2,
                          uint32_t vol1, uint32_t vol2, uint32_t *offset, uint32_t *ratio)
{
    uint32_t gap1, gap2;

    if (offset == NULL || ratio == NULL)
        return 0;

    gap1 = value1 &gt; value2 ? value1 - value2 : value2 - value1; // register value gap
    gap2 = vol1 &gt; vol2 ? vol1 - vol2 : vol2 - vol1; // voltage gap -- mv

    if (gap1 != 0)
    {
        *ratio = gap2 * ADC_RATIO_ACCURATE / gap1; // gap2 * 10 for 0.1mv, gap2 * 100 for 0.01mv
        adc_vol_ratio = *ratio;
    }
    *offset = value1 - vol1 * ADC_RATIO_ACCURATE / adc_vol_ratio;
    adc_vol_offset = *offset;

    return adc_vol_offset;
}
```
