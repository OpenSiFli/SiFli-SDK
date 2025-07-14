# 软件架构

SiFli SDK是官方提供的基于RT-Thread定制开发的一套软件开发框架，使用它可以快速开发运行于思澈科技芯片平台的应用程序。

软件框架如下图，
![](../../assets/sdk_arch_diagram.png)

- HAL为硬件抽象层，提供不依赖于操作系统服务的驱动功能
- RT-Thread设备驱动（Device Driver）基于HAL实现，提供了更高层的封装，用户无需实现中断服务程序，更易于使用，HAL与RT-Thread设备驱动更详细的比较请参考[](../app_development/drivers.md)。
- 中间件（components）包括RT-Thread自带的软件组件（如finsh、ulog）、第三方组件（位于`external`目录下）以及自研组件（位于`middleware`目录下）。应用程序可以使用包括HAL在内的所有服务接口开发应用程序。


SDK的目录结构
```
+---customer                 // 板级支撑包
|   +---boards               // 板子配置文件
|   |
|   +---peripherals          // 板级外设驱动
|
|
+---drivers
|   +---cmsis                // 芯片寄存器头文件、启动文件、链接脚本
|   |   +---Include
|   |   +---sf32lb52x
|   |   |     
|   |   +---sf32lb55x
|   |   | 
|   |   +---sf32lb56x
|   |   |
|   |   \---sf32lb58x
|   |     
|   +---hal                  // HAL实现代码
|   |
|   \---Include              // HAL头文件
| 
+---example                  // 例程
|
+---external                 // 第三方组件
|
+---middleware               // 自研组件
|
+---rtos                     // 操作系统
|   +---freertos             // FreeRTOS
|   |
|   +---os_adaptor           // OS抽象层
|   |
|   |
|   \---rtthread             // RT-Thread
|       \---bsp
|           \---sifli
|               \---drivers  // RT-Thread设备驱动适配代码
|       
|       
|
\---tools                    // 工具

```


