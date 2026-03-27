# RT-Thread Device Drivers

[device]:
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/device

[watchdog]:
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/watchdog/watchdog

[touch]:
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/touch/touch


For an introduction to the RT-Thread device model, see [I/O Device
Model][device].


## On-chip Peripherals

| Device Drivers                             | Corresponding HAL                                         |
| ------------------------------------------ | --------------------------------------------------------- |
| [UART Device](uart.md)                     | [UART](/hal/uart.md)                                      |
| [PIN Device](gpio.md)                      | [GPIO](/hal/gpio.md)                                      |
| [ADC Device](adc.md)                       | [ADC](/hal/adc.md)                                        |
| [HWTIMER Device](timer.md)                 | [GPT](/hal/gpt.md)                                        |
| [I2C Device](i2c.md)                       | [I2C](/hal/i2c.md)                                        |
| [PWM Device](pwm.md)                       | [GPT](/hal/gpt.md)                                        |
| [RTC Device](rtc.md)                       | [RTC](/hal/rtc.md)                                        |
| [SPI Device](spi.md)                       | [SPI](/hal/spi.md)                                        |
| [Watchdog Device][watchdog]                | [WDT](/hal/wdt.md)                                        |
| [audprc_audcodec](audprc_audcodec.md)      | [audprc](/hal/audprc.md) and [audcodec](/hal/audcodec.md) |
| [I2S Audio Device](i2s.md)                 | [I2S](/hal/i2s.md)                                        |
| [USBD Device](usbd.md)                     | [USBC](/hal/pcd.md)                                       |
| [SDIO Device](sdio.md)                     |                                                           |
| [Graphics Rendering Driver Layer](epic.md) | [EPIC](/hal/epic.md)                                      |

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
audprc_audcodec.md
i2s.md
usbd.md
sdio.md
spi_flash.md
spi_flash_52x.md
epic.md
```


### Configuration
Run `menuconfig` and navigate to the `On-chip Peripheral RTOS Drivers` menu to
configure the required peripherals. Selecting a device in menuconfig ensures it
is registered during system initialization. Once registered, applications can
retrieve the device pointer by name using the `rt_device_find` interface. If a
device is not registered, `rt_device_find` returns a null pointer. For example,
after enabling the UART1 device, the application can obtain its pointer using
`rt_device_find("uart1")`.

```{note}
When running `menuconfig` from the project directory, use the `--board=<board_name>` option to specify the target board, as described in [](/app_development/create_application.md). The modified configuration is saved to `proj.conf`.</board_name>
```



## Board-Level Peripherals

- [Display Devices](lcd.md)
- [Touch Devices](touch.md)

```{toctree}
:hidden:

lcd.md
touch.md
```
