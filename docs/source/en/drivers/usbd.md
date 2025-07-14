# USB Device

USB controller driver includes device driver and host driver, we only enable the device driver. It has two layers: hardware access layer (HAL) and RT-Thread adaptation layer.<br>
HAL provides basic APIs for accessing USB controller peripheral registers. For details, please refer to the API documentation of USB HAL.<br>
The adaptation layer provides support for RT-Thread driver framework. Users can use RT-Thread POSIX driver interface for USB device programming. Please refer to the API documentation of RT-Thread driver.<br> 
Main features include:<br>
- Mstorage storage support
- VCOM support
- Composite device support
- Up to 8 endpoints

```{note} 
Only SF32LB58X series supports USB 2.0 high speed mode, SF32LB55X/SF32LB56X only supports USB full/low speed.
```

## Driver Configuration

Hardware driver can be used as mstorage, ecm, hid, vcom. Functions and their peripherals can be selected for each project using menuconfig tool, usually saved in C header files. By default, configuration is saved as _rtconfig.h_.

The following example shows flags defined in a project header file that uses MSTORAGE and VCOM. Steps to select configuration for BSP:
- Enter "menuconfig" in the command under the project
- Select "RTOS --->"
- Select "On-chip Peripheral Driver--->"	
- Select "Enable USB Device" to enable USB driver, define macro BSP_USING_USBD
```c
#define BSP_USING_USBD
```

Steps to select configuration for operating system (in menuconfig main menu):
- Select "RTOS"—>
- Select "RT-Thread Components--->"
- Select "Device Driver--->"
- Select "Using USB  --->"
- Select "Enable USB device"             Enable USB device mode, define RT_USING_USB_DEVICE
- Enable "Enable composite device"
- Enable "Enable to use device as CDC device" Enable using device as CDC device USB can be used as CDC VCOM device, define RT_USB_DEVICE_CDC
- Enable "Enable to use device as Mass Storage device"  Enable using device as mass storage device USB can be used as mstorage device, define RT_USB_DEVICE_MSTORAGE
- "msc class disk name"                     Enter storage device, usually we use flash, so enter mtd device name like "flash1"

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

After configuration, users need to include header files in all source code that needs to access the driver.

## Using USB Device
With the above configuration, when plugging in the USB interface, the PC can recognize a USB hard disk and a USB serial port, which can be used in the same way as other USB hard disks/serial ports.