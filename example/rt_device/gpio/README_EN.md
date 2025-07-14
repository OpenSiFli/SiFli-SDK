# GPIO Example
Source code path: example\rt_device\gpio

## Supported Platforms
The example can run on the following development boards.
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4

## Example Overview
* Configure GPIO output, input interrupt operation, demonstrate GPIO HAL functions
* Toggle GPIO_out level value every second, input GPIO triggers interrupt on rising and falling edges, serial port prints interrupt information


## Example Usage

### Hardware Connection
* Note: Connect input GPIO and output GPIO through jumper wire, so that GPIO_out level transition can be assigned to GPIO_in to achieve entering interrupt and printing information

|Development Board    |OUT Pin |OUT Pin Name|IN Pin |IN Pin Name |
|:---     |:---    |:---      |:---   |:---      |
|sf32lb52-lcd_n16r8 |5       |PA41      |3      |PA42      |
|sf32lb58-lcd_n16r64n4 |5       |PB28      |3      |PB29      |

* For more detailed pin definitions please refer to\
`[sf32lb52-lcd_n16r8]()`\
`[sf32lb58-lcd_n16r64n4]()`

### Compilation and Flashing
#### SF525 Project Code Compilation
Switch to the example project directory and run scons command to compile:

> scons --board=sf32lb52-lcd_n16r8 -j8

Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:

> build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

> Uart Download

> please input the serial port num:5

#### SF587 Project Code Compilation
Switch to the example project directory and run scons command to compile:

> scons --board=sf32lb58-lcd_n16r64n4 -j8

Switch to the example `project/build_xx` directory and run `download.bat`, program downloads automatically through JLink:

> build_sf32lb58-lcd_n16r64n4_hcpu\download.bat


### Example Output Results Display:
* Log output:
```
   SFBL
   Serial:c2,Chip:4,Package:4,Rev:7  Reason:00000000
   [I/drv.adc] Get ADC configure fail
   \ | /
  - SiFli Corporation
   / | \     build on Jan 23 2025, 2.1.7 build bef6b3d8
   2020 - 2022 Copyright by SiFli team
   mount /dev sucess
   [D/USBD] No class register on usb device
   [I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [I/drv.rtc] RTC use LXT RTC_CR=00000001
   [I/drv.rtc] Init RTC, wake = 0
   [I/drv.audprc] init 00 ADC_PATH_CFG0 0x606
   [I/drv.audprc] HAL_AUDPRC_Init res 0
   [I/drv.audcodec] HAL_AUDCODEC_Init res 0
   call par CFG1(3313)
   fc 9, xtal 2000, pll 2075
   call par CFG1(3313)
   fc 7, xtal 2000, pll 1657
   Start gpio rtt demo!
   msh />
```
When GPIO_out level toggles every second:
```
Interrupt occurred!
Pin_Out 41 has been toggle, value = 1
Pin_In 42, value = 1
```
Toggle after 1 second:
```
Interrupt occurred!
Pin_Out 41 has been toggle, value = 0
Pin_In 42, value = 0
```
## Example Description



```{note}
Example description uses sf32lb52-lcd_n16r8 as example
```

### Macro Definitions
Adapt to different development board pin differences

```C
#ifdef SF32LB52X
    #define Pin_Out 41
    #define Pin_In 42
    #define hwp_gpio hwp_gpio1
#elif defined(SF32LB58X)
    #define Pin_Out 124
    #define Pin_In 125
    #define hwp_gpio hwp_gpio2
#endif
```

```{note}
When using PB pins, pin number needs to add 96, for detailed explanation please refer to [](/drivers/gpio.md)
```

### Output Mode
Configure `GPIO1 pin41` (i.e., GPIO_A41) as output mode

```C
struct rt_device_pin_mode m;
m.pin = Pin_Out;
m.mode = PIN_MODE_OUTPUT;
rt_device_control(device, 0, &m);
```

### Input Mode (with Interrupt)

#### GPIO Initialization
Configure `GPIO1 pin42` (i.e., GPIO_A42) as input mode with rising and falling edge detection

```C
//set pin to PIN_MODE_INPUT
m.pin = Pin_In;
m.mode = PIN_MODE_INPUT;
rt_device_control(device, 0, &m);

//set irq mode
rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING_FALLING, irq_handler, (void *)(rt_uint32_t)m.pin);
rt_pin_irq_enable(m.pin, 1);
```

#### Interrupt Callback Function
```C
void irq_handler(void *args)
{
    //set your own irq handle
    rt_kprintf("Interrupt occurred!\n");
    
    //read Pin_Out
    struct rt_device_pin_status st;
    st.pin = Pin_Out;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_kprintf("Pin_Out %d has been toggle, value = %d\n", Pin_Out, st.status);

    //read Pin_In
    st.pin = Pin_In;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_kprintf("Pin_In %d, value = %d\n", Pin_In, st.status);
    rt_kprintf(" \n");
}
```
## API Reference
[](#hal-gpio)

## Update History
|Version  |Date    |Release Notes |
|:---  |:---    |:---    |
|0.0.1 |12/2024 |Initial version |
|      |        |        |