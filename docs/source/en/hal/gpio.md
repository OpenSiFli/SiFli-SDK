# GPIO

HAL GPIO module provides abstract software interface to operate hardware GPIO
module. HPSYS and LPSYS each have one GPIO module with supported features:
- Output mode
- Input mode, can detect input level to trigger interrupt, supports high level,
  low level, rising edge, falling edge and both edge detection

HPSYS hardware GPIO module is `hwp_gpio1` (or called GPIO_A), LPSYS hardware
GPIO module is `hwp_gpio2` (or called GPIO_B).

```{note}
If you need to set GPIO pins for other functions, or change pull-up/pull-down drive capability, please refer to pinmux settings [PINMUX](#hal-pinmux)
```

For detailed API documentation, refer to [GPIO](#hal-gpio).

## Using GPIO HAL

### Output mode
Configure `GPIO1 pin10` (i.e., GPIO_A10) as output mode, output high level
```c
void write_pin(void)
{
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t pin = 10; 

    /* Configure GPIO1 pin 10 as output */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &amp;GPIO_InitStruct);

    /* Drive the pin high */
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
}
```

### Input Mode (No Interrupt)
Configure GPIO1 pin10 (i.e., GPIO_A10) as input mode, read level state
```c
void read_pin(void)
{
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t pin = 10; 
    GPIO_PinState state;

    /* Configure GPIO1 pin 10 as input */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &amp;GPIO_InitStruct);

    /* Read pin state */
    state = HAL_GPIO_ReadPin(gpio, pin);
}
```


### Input Mode (With Interrupt)
Configure `GPIO1 pin10` (i.e., GPIO_A10) as input mode, both edge detection
```c
/* GPIO1 IRQ Handler in the vector table */
void GPIO1_IRQHandler(void)
{
    for (uint32_t i = 0; i &lt;= 41; i++)
    {
        HAL_GPIO_EXTI_IRQHandler(hwp_gpio1, i);
    }
}

/* Override the weak callback to implement user-defined actions; called by HAL_GPIO_EXTI_IRQHandler */
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_PinState state;

    state = HAL_GPIO_ReadPin(hgpio, GPIO_Pin);
}

void detect_pin(void)
{
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t pin = 10; 
    GPIO_PinState state;

    /* Configure GPIO1 pin 10 as input */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &amp;GPIO_InitStruct);

    /* Enable dual-edge detection on GPIO1 pin 10 */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &amp;GPIO_InitStruct);    

    /* Enable GPIO1 interrupt */
    HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPIO1_IRQn);    
}
```
## API Reference
[]
