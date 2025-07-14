# Chip Peripheral Drivers

As shown in the software architecture diagram, the development framework provides two sets of chip peripheral driver interfaces: one is the HAL interface, and the other is the RT-Thread device driver interface.
![sdk arch](../../assets/sdk_arch_diagram.png)

The HAL interface does not use operating system services and provides all the functionality supported by the chip peripherals. It requires the user to implement interrupt service routines. The HAL interface is compatible across different chip families. Most HAL interfaces offer both synchronous (polling) and asynchronous versions.

The RT-Thread device driver interface provides a higher level of abstraction, implemented based on the HAL interface, which makes it easier to port between chip platforms from different manufacturers. Users do not need to implement interrupt service routines, and the interface is relatively simpler to use than the HAL interface. Even the synchronous interface uses a thread-suspension mechanism, which avoids occupying CPU resources. Some peripheral drivers integrate buffer management functions. For example, in UART devices, when DMA mode is used, data is first stored in an internal buffer, and calling the `rt_device_read` interface retrieves data from this buffer, relieving the user from managing the buffer. However, due to the generic nature of the abstraction, some hardware features are not implemented in the RT-Thread device driver layer.

The HAL interface, compared to the RT-Thread device driver interface, is closer to the hardware layer, allowing for more flexible use of hardware and fewer intermediary steps, resulting in higher function execution efficiency. However, it is more complex to use. Generally, if the features provided by the RT-Thread device driver meet the application development needs, it is recommended to use the device driver. The HAL interface should only be chosen when the device driver cannot fulfill the requirements or does not offer sufficient performance. For guidance on using device drivers, refer to [device driver guide](../drivers/index.md), and for HAL interface usage, refer to the [HAL guide](../hal/index.md).

```{warning}
Note that for a given peripheral, such as UART, the HAL and device driver interfaces should not be mixed. When using the HAL interface to operate UART, ensure that the `serial device drivers` in the `Device Drivers` section are not enabled.
```
