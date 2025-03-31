# ADC设备

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable ADC`菜单中选择要使用的ADC设备，并配置相关参数

下面的宏开关表示使能了ADC1设备
```c
#define BSP_USING_ADC
#define BSP_USING_ADC1
```

## 设备名称
- `bat<x>`，
其中x为设备编号，如`bat1`、`bat2`，与操作的外设编号对应

## PinMux 和插槽定义
对于每个 adc 插槽，都有一个引脚，但该引脚可以复用到不同的功能。 在将其用作 ADC 之前，我们需要设置它的 pinmux 功能，这里是将 pin 设置为 ADC SLOT 0 的示例，更多详细信息需要检查硬件 pinmux 表：
```c
HAL_PIN_Set(PAD_PB_04, ADC_PIN, PIN_INPUT, 0);
```

## ADC TO VOL
需要2个不同的电压，得到它们的寄存器值，可以得到准确的偏移量和比率（mv per bit）。

## 示例代码


```c
// Find and open adc device
dev = rt_device_find("bat1");
rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);

// enable slot
rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)chn);

// read value
res = rt_device_read(dev, chn, &value, 1);

// value to voltage
res = (value - TC_ADC_OFFSET) * ratio;

...

// Close device, keep it open for more user.
rt_device_close(dev);

```

[adc]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/adc/adc
## RT-Thread参考文档

- [ADC设备][adc]