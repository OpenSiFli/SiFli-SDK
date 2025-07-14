
# RT-Thread Device Drivers

[device]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/device

[watchdog]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/watchdog/watchdog

[touch]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/touch/touch


Introduction to RT-Thread device model: [I/O Device Model][device]


## Chip Peripherals

Device Driver            | Corresponding HAL           |
-------------------------|-----------------------------|
[UART Device](uart.md)   | [UART](../hal/uart.md)            |
[PIN Device](gpio.md)    | [GPIO](../hal/gpio.md)            | 
[ADC Device](adc.md)     | [ADC](../hal/adc.md)             | 
[HWTIMER Device](timer.md)| [GPT](../hal/gpt.md)             | 
[I2C Device](i2c.md)     | [I2C](../hal/i2c.md)             | 
[PWM Device](pwm.md)     | [GPT](../hal/gpt.md)             | 
[RTC Device](rtc.md)     | [RTC](../hal/rtc.md)             | 
[SPI Device](spi.md)     | [SPI](../hal/spi.md)             | 
[Watchdog Device][watchdog] | [WDT](../hal/wdt.md)          | 
[I2S Audio Device](audio.md) | [I2S](../hal/i2s.md)          | 
[USBD Device](usbd.md)   | [USBC](../hal/pcd.md)             | 
[SDIO Device](sdio.md)   |                             |


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


### Configuration
Run `menuconfig`, then enter the `On-chip Peripheral RTOS Drivers` menu to configure the peripherals to be used. After selecting a device in menuconfig, it means that the corresponding device will be registered during system initialization, and the application can use the `rt_device_find` interface to obtain the device pointer by device name. For unregistered devices, `rt_device_find` will return a NULL pointer. For example, after selecting the UART1 device, the application can use `rt_device_find("uart1")` to obtain the device pointer.

```{note}
If running `menuconfig` in the project directory, you need to add `--board=<board_name>` to specify the board being used. Refer to the instructions in [](/app_development/create_application.md). The modified configuration is saved in `proj.conf`.
```


## Board-Level Peripherals

- [Display Device](lcd.md)
- [Touch Device](touch.md)


```{toctree}
:hidden:

lcd.md
touch.md

```
