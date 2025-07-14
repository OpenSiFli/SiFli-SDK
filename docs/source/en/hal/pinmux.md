# PINMUX

HAL PINMUX provides abstract software interfaces to operate the hardware PINMUX module, setting pin functions and pull-up/pull-down attributes.
The chip has two PINMUX instances: PINMUX1 (`hwp_pinmux1`) in the HPSYS domain and PINMUX2 (`hwp_pinmux2`) in the LPSYS domain.
PINMUX1's pin list is found in `pin_pad_hcpu`, with available pin functions referenced in `pin_pad_func_hcpu`. Similarly, PINMUX2's pin list is found in `pin_pad_lcpu`, with pin function definitions found in `pin_pad_func_lcpu`.

Starting from 56x series chips (excluding 55x, 58x), any GPIO in the pinmux functionality can serve as an I/O pin for any I2C/UART/PWM in the current system.

For detailed API documentation, refer to [](#hal-pinmux)

## Difference Between GPIO and PINMUX Modules
Physically, GPIO needs to connect to the external world through the pinmux module, as shown in the figure:
![Figure 1: Relationship between pinmux module and GPIO module](../../assets/relation_of_gpio_pinmux.png)

## Using HAL PINMUX

```c
void pin_func_set_example(void)
{
    /* set HCPU PA10 and PA14 for I2C */
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);
    
    /* set LCPU PB12 and PB14 for UART4 */
    HAL_PIN_Set(PAD_PB12, USART4_TXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB14, USART4_RXD, PIN_PULLUP, 0);
}
```

## API Reference
[](#hal-pinmux)