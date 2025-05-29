# 点灯（RT-Thread）
## 概述
例程展示了通过GPIO实现LED闪烁功能，以便更好的理解GPIO的使用。

## 支持的开发板
例程可以运行在以下开发板.
* eh-lb520
* eh-lb523
* eh-lb525
* eh-lb561
* eh-lb563
* eh-ss6500

**注意：** 一般而言，例程都是运行芯片的HCPU。如果想在LCPU运行例程，可以使用"eh-lb563_lcpu"。

## 例程目录结构
Blink工程包含了1个.c文件(main.c),下面的树状结构展示 了工程目录下的其他文件.
```c
|--Readme.md
|--no-os
|--rtt
    |--src
    |    |--main.c
    |    |--Sconscript
    |--project  
         |--Kconfig
         |--Kconfig.proj
         |--proj.conf
         |--rtconfig.py
         |--SConscript
         |--SConstruct
```
## 例程的使用
### 硬件需求
运行例程的前提，需要拥有一块支持该例程的开发板。
### 管脚配置
**注意:** 下面表格展示了各开发板用于LED控制的管脚配置。
|               | pin assign  |
|---------------|-------------|
|eh-lb523    |    PA25     |
|eh-lb520    |    PA25     |
|eh-lb525    |    PA25     |
|eh-lb561    |    PA41     |
|eh-lb563    |    PA41     |
|eh-ss6500   |    PA41     |

### 编译和烧录
按照以下步骤，可以完成编译和烧录。
```shell
menuconfig --board=eh-lb523
scons --board=eh-lb523 -j8
 .\build_eh-lb523\uart_download.bat
```
## 例程输出结果展示
下面结果展示了例程在开发板运行起来后的log。如果看不到这些log，就说明例程没能按预期运行成功，需要进行原因排查。
```
Start example blink
Turning the LED OFF
Turning the LED ON
```
 ## 故障排除
如果未能出现预期的log，可以从以下方面进行故障排除：
* 硬件连接是否正常
* 管脚配置是否正确  

 ## 例程扩展
 
 如果实现控制更多led，下面以在开发板 eh-lb523 增加第二led为例，展示了如何扩展例程：
 1.  修改kconfig文件 **"\siflisdk\customer\boards\Kconfig.v2"**，增加LED2 选项：BSP_USING_LED2;
 ```kconfig
 menuconfig BSP_USING_LED2
    bool "Use LED2"
    default n
    if BSP_USING_LED2  
        config BSP_LED2_PIN
            int "LED2 pin number"
            default 99
        config BSP_LED2_ACTIVE_HIGH
            bool "Level is high if LED is on"
            default n
    endif 
```
 2.  修改开发板配置文件 **"\siflisdk\customer\boards\eh-lb523\hcpu\board.conf"**，使能LED2并且配置对应的管脚，以及有效电平；
  ```c
CONFIG_BSP_USING_LED2=y
CONFIG_BSP_LED2_PIN=26
CONFIG_BSP_LED2_ACTIVE_HIGH=y
```
上面的配置为**eh-lb523**的默认配置，即LED2默认配置为pin 26；如果想修改为其他pin，可以通过运行 **menuconfig --board eh-lb523**进行修改；

3.修改代码**main.c**, 控制BSP_LED2_PINs实现LED的闪烁；
  ```c
int main(void)
{
    /*recode led state*/

    /* Output a message on console using printf function */
    rt_kprintf("Start example blink\n");

    /*Init pin state */
    rt_pin_mode(BSP_LED1_PIN, PIN_MODE_OUTPUT);
	  rt_pin_mode(BSP_LED2_PIN, PIN_MODE_OUTPUT);
    bool state = RT_FALSE;

    /* Infinite loop, len flash inter 1s */
    while (1)
    {
      rt_kprintf("Turning the LED %s", state == RT_TRUE ? "ON" : "OFF");
#ifndef BSP_LED1_ACTIVE_HIGH
      rt_pin_write(BSP_LED1_PIN, !state);
      rt_pin_write(BSP_LED2_PIN, !state);
#else
      rt_pin_write(BSP_LED1_PIN, state);
		  rt_pin_write(BSP_LED2_PIN, !state);
#endif
      /*toggle the LED statu*/
      state = !state;

      rt_thread_mdelay(1000);
    }
    return 0;
}
```