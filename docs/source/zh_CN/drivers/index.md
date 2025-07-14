# RT-Thread 设备驱动

[device]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/device

[watchdog]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/watchdog/watchdog

[touch]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/touch/touch


RT-Thread的设备模型介绍：[I/O设备模型][device]


## 芯片外设

设备驱动                  |  对应HAL           |
-------------------------|--------------------|
[UART设备](uart.md)      |  [UART](/hal/uart.md)  |
[PIN设备](gpio.md)       | [GPIO](/hal/gpio.md)   | 
[ADC设备](adc.md)        | [ADC](/hal/adc.md)   | 
[HWTIMER设备](timer.md)   | [GPT](/hal/gpt.md)   | 
[I2C设备](i2c.md)        | [I2C](/hal/i2c.md)   | 
[PWM设备](pwm.md)           | [GPT](/hal/gpt.md)   | 
[RTC设备](rtc.md)           | [RTC](/hal/rtc.md)   | 
[SPI设备](spi.md)           | [SPI](/hal/spi.md)   | 
[看门狗设备][watchdog]   | [WDT](/hal/wdt.md)   | 
[I2S Audio设备](audio.md)       |  [I2S](/hal/i2s.md) | 
[USBD设备](usbd.md)       |  [USBC](/hal/pcd.md) | 
[SDIO设备](sdio.md)       |   | 


```{toctree}
:hidden:

uart.md
gpio.md
adc.md
timer.md
i2c.md
pwm.md
rtc.md
spi.md
audio.md
usbd.md
sdio.md
spi_flash.md
```


### 配置
运行`menuconfig`，进入 `On-chip Perepheral RTOS Drivers`菜单配置所要使用的外设，在menuconfig中选择了设备后即表示在系统初始化时会注册相应的设备，这样应用程序中就可以使用`rt_device_find`接口按设备名称获取设备指针，对于没有注册的设备，`rt_device_find`会返回空指针。例如，选择了UART1设备后，应用程序可用使用`rt_device_find("uart1")`获取设备指针。

```{note}
如果是在工程目录下运行`menuconfig`，需要添加`--board=<board_name>`指定所用的板子，参考[](/app_development/create_application.md)的说明，修改后的配置保存在`proj.conf`中
```



## 板级外设

- [显示屏设备](lcd.md)
- [触控设备](touch.md)


```{toctree}
:hidden:

lcd.md
touch.md

```