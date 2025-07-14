# LED Blink (NO-OS)
## Overview
This example demonstrates LED blinking functionality using GPIO to better understand GPIO usage.

## Supported Development Boards
This example can run on the following development boards:
* eh-lb520
* eh-lb523
* eh-lb525
* eh-lb561
* eh-lb563
* eh-ss6500

**Note:** Generally, examples run on the chip's HCPU. If you want to run the example on LCPU, you can use "eh-lb563_lcpu".

## Example Directory Structure
The Blink project contains 1 .c file (main.c). The tree structure below shows other files in the project directory.
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
## Example Usage
### Hardware Requirements
To run this example, you need to have a development board that supports this example.
### Pin Configuration
**Note:** The table below shows the pin configurations for LED control on each development board.
|               | pin assign  |
|---------------|-------------|
|eh-lb523    |    PA25     |
|eh-lb520    |    PA25     |
|eh-lb525    |    PA25     |
|eh-lb561    |    PA41     |
|eh-lb563    |    PA41     |
|eh-ss6500   |    PA41     |

### Compilation and Flashing
Follow these steps to complete compilation and flashing:
```
menuconfig --board=eh-lb523
scons --board=eh-lb523 -j8
 .\build_eh-lb523\uart_download.bat
```
## Example Output Results
The following results show the log after the example runs on the development board. If you cannot see these logs, it means the example did not run successfully as expected and troubleshooting is needed.
```c
Start example blink
Turning the LED OFF
Turning the LED ON
```
 ## Troubleshooting
If the expected logs do not appear, troubleshooting can be performed from the following aspects:
* Whether hardware connections are normal
* Whether pin configuration is correct  

 ## Example Extension
 
 To control more LEDs, the following example shows how to extend the example by adding a second LED on development board eh-lb523:
 1. Modify the kconfig file **"\siflisdk\customer\boards\Kconfig.v2"** to add LED2 option: BSP_USING_LED2;
 ```c
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
 2. Modify the development board configuration file **"\siflisdk\customer\boards\eh-lb523\hcpu\board.conf"** to enable LED2 and configure the corresponding pin and active level;
  ```c
CONFIG_BSP_USING_LED2=y
CONFIG_BSP_LED2_PIN=26
CONFIG_BSP_LED2_ACTIVE_HIGH=y
```
The above configuration is the default configuration for **eh-lb523**, i.e., LED2 is configured to pin 26 by default; if you want to modify it to another pin, you can run **menuconfig --board eh-lb523** to modify it;

3. Modify the code **main.c** to control BSP_LED2_PINs to implement LED blinking;
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