# GPIO Example
Source code path: example/hal/gpio

## Supported Platforms
The example can run on the following development boards.
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4
* sf32lb56-lcd_n16r12n1
## Example Overview
* Configure GPIO output and input interrupt operations to demonstrate GPIO HAL functions
* Toggle GPIO_out level value every second, input GPIO triggers interrupt on rising and falling edges, printing interrupt information via serial port
## GPIO Overview
The HAL GPIO module provides an abstract software interface to operate the hardware GPIO module. 
HPSYS and LPSYS each have a GPIO module, with supported features including:
- Output mode
- Input mode, can detect input level to trigger interrupts, supports high level, low level, rising edge, falling edge, and dual edge detection

The hardware GPIO module of HPSYS is `hwp_gpio1` (or called GPIO_A), and the hardware GPIO module of LPSYS is `hwp_gpio2` (or called GPIO_B). 

```{note}
If you need to set GPIO pins for other functions, or change pull-up/pull-down drive capability, please refer to pinmux settings [PINMUX](#hal-pinmux)
```

For detailed API description, refer to [GPIO](#hal-gpio) 


## Example Usage

### Hardware Connection
* Note: Connect the input GPIO and output GPIO with a jumper wire, so that the GPIO_out level transition can be assigned to GPIO_in to achieve the effect of entering the interrupt and printing information

|Development Board    |OUT Pin |OUT Pin Name|IN Pin |IN Pin Name |
|:---     |:---    |:---      |:---   |:---      |
|sf32lb52-lcd_n16r8 |5       |PA41      |3      |PA42      |
|sf32lb58-lcd_n16r64n4 |5       |PB28      |3      |PB29      |
|sf32lb56-lcd_n16r12n1 |5       |PA20      |3      |PB12   |

* For more detailed pin definitions, please refer to
`[sf32lb52-lcd_n16r8]()`
`[sf32lb58-lcd_n16r64n4]()`

### Compilation and Programming
#### SF525 Project Code Compilation
Switch to the example project directory and run the scons command to compile:

```
scons --board=sf32lb52-lcd_n16r8 -j8
```

Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

#### SF587 Project Code Compilation
Switch to the example project directory and run the scons command to compile:

```
scons --board=sf32lb58-lcd_n16r64n4 -j8
```

Run `build_sf32lb58-lcd_n16r64n4_hcpu\download.bat`, the program downloads automatically via JLink:

```
build_sf32lb58-lcd_n16r64n4_hcpu\download.bat
```


### Example Output Results Display:
* Log output:
```
   Serial:c2,Chip:2,Package:0,Rev:1  Reason:00000000
   \ | /
  - SiFli Corporation
   / | \     build on Dec 18 2024, 2.1.7 build bb6471d6
   2020 - 2022 Copyright by SiFli team
   mount /dev sucess
   [D/USBD] No class register on usb device
   [I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [I/drv.rtc] RTC use LXT RTC_CR=00000000
   [I/drv.rtc] Init RTC, wake = 0
   [I/drv.sdhci] rt_hw_sdmmc_init 2 begin
   [I/drv.sdhci] host version = 2
   [I/drv.sdhci] SDHCI clock 288000000
   [I/drv.sdhci] Maximum Clock Supported by HOST : 288 MHz 
   [I/drv.sdhci] host minclock 400000  host maxclock 288000000  
   [I/drv.sdhci] SDHCI controller on sdmmc2 using DMA
   [I/drv.sdhci] Add host success
   [I/drv.sdhci] rt_hw_sdmmc_init 2 done
   [I/drv.audprc] init 00 ADC_PATH_CFG0 0x924
   [I/drv.audprc] HAL_AUDPRC_Init res 0
   [I/drv] HAL_AUDCODEC_Init res 0
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2054
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2054
   Start gpio demo!
```
When GPIO_out level inverts every second:
```
Interrupt occurred!
Pin_Out 41 has been toggle, value = 1
Pin_In 42, value = 1
```
After 1 second toggle:
```
Interrupt occurred!
Pin_Out 41 has been toggle, value = 0
Pin_In 42, value = 0
```
## Example Description

```{note}
Example description uses sf32lb52-lcd_n16r8 as an example
```

### Macro Definitions
Adapt to different pin configurations on different development boards

```C
#ifdef SF32LB52X
    #define Pin_Out 41
    #define Pin_In 42
    #define GPIO_IRQn GPIO1_IRQn
    #define hwp_gpio hwp_gpio1
    #define RCC_MOD_GPIO RCC_MOD_GPIO1
#elif defined(SF32LB58X)
    #define Pin_Out 28
    #define Pin_In 29
    #define GPIO_IRQn GPIO2_IRQn
    #define hwp_gpio hwp_gpio2
    #define RCC_MOD_GPIO RCC_MOD_GPIO2
#elif defined(SF32LB56X)
    #define Pin_Out 20
    #define Pin_In 12
    #define GPIO_IRQn GPIO1_IRQn
    #define hwp_gpio hwp_gpio1
    #define RCC_MOD_GPIO RCC_MOD_GPIO1
#endif
```

### Output Mode
Configure `GPIO1 pin41` (i.e., GPIO_A41) as output mode

```C
GPIO_InitStruct.Pin = Pin_Out;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);
```

### Input Mode (with interrupt)

#### GPIO Initialization
Configure `GPIO1 pin42` (i.e., GPIO_A42) as input mode with rising and falling edge detection

```C
PIO_InitStruct.Pin = Pin_In;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; //Set interrupt to trigger on raising and falling edge
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);
HAL_NVIC_SetPriority(GPIO_IRQn, 2, 1); // Configure NVIC priority
```
#### Interrupt Service Function
Find the interrupt service function according to the interrupt vector table in the startup file
```C
void GPIO1_IRQHandler(void) // Define the interrupt siervice routine (ISR) according to the interrupt vector table
{
    HAL_GPIO_IRQHandler(hwp_gpio);  
}
```

#### Interrupt Callback Function
```C
// override the weak Callback to add user defined action, it's called by HAL_GPIO_EXTI_IRQHandler 
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Pin_In)
    {
        rt_kprintf("Interrupt occurred!\n");
        rt_kprintf("Pin_Out %d has been toggle, value = %d\n", Pin_Out, HAL_GPIO_ReadPin(hwp_gpio, Pin_Out));
        rt_kprintf("Pin_In %d, value = %d\n", Pin_In, HAL_GPIO_ReadPin(hwp_gpio, Pin_In));
        rt_kprintf(" \n");
    }
}
```
## API Reference
[](#hal-gpio)

## Update Log
|Version  |Date    |Release Notes |
|:---  |:---    |:---    |
|0.0.1 |12/2024 |Initial version |
|      |        |        |