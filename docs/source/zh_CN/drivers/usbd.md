# USB设备

USB 控制器驱动包括设备驱动和主机驱动，我们只启用设备驱动。 它有两层：硬件访问层（HAL）和RT-Thread的适配层。<br>
HAL 提供用于访问 USB 控制器外围设备的寄存器的基本 API。 有关详细信息，请参阅 USB HAL 的 API 文档。<br>
适配层提供对 RT-Thread 驱动框架的支持。 用户可以使用 RT-Thread POSIX 驱动程序接口对 USB 设备进行编程。 请参阅 RT-Thread 驱动程序的 API 文档。<br> 
主要功能包括： <br>
- Mstorage 存储支持
- VCOM 支持
- 支持复合设备
- 最多 8 个端点

```{note} 
只有SF32LB58X系列支持USB 2.0 high speed mode, SF32LB55X/SF32LB56X仅支持USB full/low speed.
```

## 驱动配置

硬件驱动程序可以用作 mstorage、ecm、hid、vcom。 可以使用menuconfig 工具为每个项目选择函数及其外设，通常保存在C 头文件中。 默认情况下，配置保存为 _rtconfig.h_。 

以下示例显示在一个项目头文件中定义的标志，该项目使用 MSTORAGE 和 VCOM。 为 BSP 选择配置的步骤：
- 在项目下的命令中输入“menuconfig”
- 选择“RTOS --->”
- 选择“On-chip Peripheral Driver--->”	
- 选择“Enable USB Device”启用USB驱动，定义宏BSP_USING_USBD
```c
#define BSP_USING_USBD
```

为操作系统选择配置的步骤（在 menuconfig 主菜单中）：
- 选择“RTOS”—>
- 选择“RT-Thread Components--->”
- 选择“Device Driver--->”
- 选择“Using USB  --->”
- 选择“Enable USB device”             启用 USB 设备模式，定义 RT_USING_USB_DEVICE
- 启用“Enable composite device”
- 启用“Enable to use device as CDC device” 使能将设备用作CDC设备 USB可以用作CDC VCOM设备，定义RT_USB_DEVICE_CDC
- 启用“Enable to use device as Mass Storage device”  启用将设备用作大容量存储设备 USB 可以用作 mstorage 设备，定义 RT_USB_DEVICE_MSTORAGE
- "msc class disk name"                     输入存储设备，通常我们使用flash，所以输入mtd设备名如"flash1"

```c
#define RT_USING_USB_DEVICE
#define RT_USBD_THREAD_STACK_SZ 4096
#define USB_VENDOR_ID 0x0FFE
#define USB_PRODUCT_ID 0x0001
#define RT_USB_DEVICE_COMPOSITE
#define RT_USB_DEVICE_CDC
#define RT_USB_DEVICE_NONE
#define RT_USB_DEVICE_MSTORAGE
#define RT_VCOM_TASK_STK_SIZE 512
#define RT_VCOM_SERNO "32021919830108"
#define RT_VCOM_SER_LEN 14
#define RT_VCOM_TX_TIMEOUT 1000
#define RT_USB_MSTORAGE_DISK_NAME "flash1"
```

配置完成后，用户需要在所有需要访问驱动程序的源代码中包含头文件。

## 使用USB设备
在上述配置中，插入USB接口，PC上可以识别出USB 硬盘和一个USB串口，使用方式和其他USB硬盘/串口一致。

