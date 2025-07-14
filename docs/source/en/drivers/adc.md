# ADC Device

## Driver Configuration

Select the ADC device to be used and configure relevant parameters in the menu {menuselection}`On-Chip Peripheral RTOS Drivers --> Enable ADC`.

The following macro definitions indicate that the ADC1 device is enabled:
```c
#define BSP_USING_ADC
#define BSP_USING_ADC1
```

## Device Naming
- `bat<x>`,
where x is the device index, such as `bat1`, `bat2`, corresponding to the peripheral device number.

## PinMux and Slot Definitions
Each ADC slot corresponds to a pin, and this pin can be multiplexed for different functionalities. Before using it as an ADC input, its pinmux function must be set. Below is an example of configuring the pin for ADC SLOT 0. For more details, refer to the hardware pinmux table:
```c
HAL_PIN_Set(PAD_PB_04, ADC_PIN, PIN_INPUT, 0);
```

## ADC to Voltage Conversion
Two different voltage levels are required to obtain their register values, which allows for accurate calculation of offset and ratio (millivolts per bit).

## Example Code

```c
// Find and open ADC device
dev = rt_device_find("bat1");
rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);

// Enable slot
rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)chn);

// Read value
res = rt_device_read(dev, chn, &value, 1);

// Convert value to voltage
res = (value - TC_ADC_OFFSET) * ratio;

...

// Close device. Keep it open if further operations are needed.
rt_device_close(dev);
```

[adc]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/adc/adc
## RT-Thread Reference Documentation

- [ADC Device][adc]
