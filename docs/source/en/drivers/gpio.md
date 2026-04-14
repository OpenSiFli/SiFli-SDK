
# PIN Device

The PIN device allows configuring GPIO input, output, and interrupt triggers, but does not support pull-up or pull-down functionalities.

## Driver Configuration

To add the PIN device, select `{menuselection}` `On-Chip Peripheral RTOS Drivers --> Enable GPIO` menu. You do not need to fill in the `GPIO BASE number` option.

Corresponding macro switches are as follows:
```c
#define RT_USING_PIN
#define BSP_USING_GPIO
```

## Getting Pin Numbers (Pin ID)
Unlike the HAL layer, where GPIOs need to specify the GPIO group and group number, the driver layer uses a numeric ID to represent a specific pin (including GPIOA, GPIOB, and PBR on supported series). The numbering method is as follows:

Pin         | Pin ID | Notes
------------|--------|------
GPIOA_00    | 0      | `GET_PIN(1, 0)`
GPIOA_01    | 1      | `GET_PIN(1, 1)`
GPIOA_02    | 2      | `GET_PIN(1, 2)`
...         | ...    |
GPIOB_00    | 96     | `GET_PIN(2, 0)`
GPIOB_01    | 97     | `GET_PIN(2, 1)`
GPIOB_02    | 98     | `GET_PIN(2, 2)`
...         | ...    |
PBR0        | 160    | `GET_PIN(0, 0)`
PBR1        | 161    | `GET_PIN(0, 1)`
...         | ...    | Range depends on series, see note below

Example:
 - GPIOB03 pin, Pin ID is 99
 - GPIOA03 pin, Pin ID is 3
 

```{note}
You can also use macros to obtain pin numbers.
```
```c
GET_PIN(port, pin)

#define LED0_PIN       GET_PIN(1,  3)   //GPIOA_03
#define LED1_PIN       GET_PIN(2,  9)   //GPIOB_09  
```

```{note}
For PBR pins in the pin device layer, use `GET_PIN(0, pbr_index)`.

Chip Difference Description:
- SF32LB52 / SF32LB56: support `PBR0~PBR3` (pin id `160~163`)
- SF32LB58: support `PBR0~PBR5` (pin id `160~165`)
- SF32LB55: PBR is not supported
```




## Example 1 – Interrupt Mode
Set PA00 as rising edge interrupt trigger mode and disable the interrupt trigger mode after 3 seconds.
```c
static void pin_irq_callback(void *args)
{
    LOG_I("pin_irq_callback");
}

void pin_irq()
{
	rt_base_t pin_id = GET_PIN(1,0); //Get GPIOA_00 pin id
	//Set pin input mode
    rt_pin_mode(pin_id, PIN_MODE_INPUT_PULLUP);
	//Enable rising edge interrupt mode
    rt_pin_attach_irq(pin_id, PIN_IRQ_MODE_RISING, gpio_int_callback, RT_NULL);
    //Enable interrupt
    rt_pin_irq_enable(pin_id, 1);
    
	rt_thread_mdelay(3000);
	
	//Disable interrupt
    rt_pin_irq_enable(pin_id, 0);
    //Detach irq handler
    rt_pin_detach_irq(pin_id);
}
```

## Example 2 – Input or Output Mode
Set PB02 to input mode, then read the level, followed by outputting a high level.
```c
void pin_read_and_write(void)
{
	int v;
	rt_base_t pin_id = GET_PIN(2,2);  //Get GPIOB_02 pin id
	//Set pin input mode
	rt_pin_mode(pin_id, PIN_MODE_INPUT_PULLUP);
	v = rt_pin_read(pin_id);
    LOG_I("pin_read value=%d",v);
    
	//Set pin output mode
	rt_pin_mode(pin_id, PIN_MODE_OUTPUT);
	//Set pin output high
    rt_pin_write(pin_id, 1);
}
```

[pin]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/pin/pin

## RT-Thread Reference Documentation

- [PIN Device][pin]
