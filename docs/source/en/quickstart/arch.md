
# Software Architecture

SiFli SDK is an officially provided software development framework customized based on RT-Thread. Using it, you can quickly develop applications running on SiFli Technology chip platforms.

The software framework is shown in the following diagram:
![](../../assets/sdk_arch_diagram.png)

- HAL is the Hardware Abstraction Layer, providing driver functionality independent of operating system services
- RT-Thread Device Driver is implemented based on HAL, providing higher-level encapsulation. Users do not need to implement interrupt service routines, making it easier to use. For a more detailed comparison between HAL and RT-Thread device drivers, please refer to [drivers.md](/docs/source/en/app_development/drivers.md).
- Middleware (components) includes RT-Thread's built-in software components (such as finsh, ulog), third-party components (located in the `external` directory), and self-developed components (located in the `middleware` directory). Applications can use all service interfaces including HAL to develop applications.

SDK Directory Structure
```
+---customer                 // Board support package
|   +---boards               // Board configuration files
|   |
|   +---peripherals          // Board-level peripheral drivers
|
|
+---drivers
|   +---cmsis                // Chip register header files, startup files, linker scripts
|   |   +---Include
|   |   +---sf32lb52x
|   |   |     
|   |   +---sf32lb55x
|   |   | 
|   |   +---sf32lb56x
|   |   |
|   |   \---sf32lb58x
|   |     
|   +---hal                  // HAL implementation code
|   |
|   \---Include              // HAL header files
| 
+---example                  // Examples
|
+---external                 // Third-party components
|
+---middleware               // Self-developed components
|
+---rtos                     // Operating system
|   +---freertos             // FreeRTOS
|   |
|   +---os_adaptor           // OS abstraction layer
|   |
|   |
|   \---rtthread             // RT-Thread
|       \---bsp
|           \---sifli
|               \---drivers  // RT-Thread device driver adaptation code
|       
|       
|
\---tools                    // Tools