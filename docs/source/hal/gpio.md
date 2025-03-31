# GPIO

HAL GPIO 模块提供抽象的软件接口操作硬件GPIO模块. 
HPSYS和LPSYS各有一个GPIO模块，支持的特性有:
- 输出模式
- 输入模式， 可检测输入电平触发中断，支持高电平、低电平、上升沿、下降沿和双沿检测

HPSYS的硬件GPIO模块为 `hwp_gpio1` (或称为GPIO_A), LPSYS的硬件GPIO模块为 `hwp_gpio2` (或称为GPIO_B). 

```{note}
如果需要设置GPIO管脚为其他功能，或者更改上下拉驱动能力，请参考pinmux的设置[PINMUX](#hal-pinmux)
```

详细的API说明参考[GPIO](#hal-gpio) .

## 使用GPIO HAL

### 输出模式
配置`GPIO1 pin10`(即GPIO_A10)为输出模式，输出高电平
```c
void write_pin(void)
{
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t pin = 10; 

    /* set GPIO1 pin10 to output mode */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    /* set pin to high */
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
}
```

### 输入模式(无中断)
配置GPIO1 pin10(即GPIO_A10)为输入模式，读取电平状态
```c
void read_pin(void)
{
    GPIO_TypeDef *gpio = hwp_gpio1;
    GPIO_InitTypeDef GPIO_InitStruct;
    uint16_t pin = 10; 
    GPIO_PinState state;

    /* set GPIO1 pin10 to input mode */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    /* read pin state */
    state = HAL_GPIO_ReadPin(gpio, pin);
}
```


### 输入模式(有中断)
配置`GPIO1 pin10`(即GPIO_A10)为输入模式，双沿检测
```c

/* GPIO1 IRQ Handler in vector table */
void GPIO1_IRQHandler(void)
{
    for (uint32_t i = 0; i <= 41; i++)
    {
        HAL_GPIO_EXTI_IRQHandler(hwp_gpio1, i);
    }
}

/* override the weak Callback to add user defined action, it's called by HAL_GPIO_EXTI_IRQHandler */
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

    /* set GPIO1 pin10 to input mode  */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);
    
    /* enable GPIO1 pin10 double edge detection */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);    
    
    /* Enable GPIO1 interrupt */
    HAL_NVIC_SetPriority(GPIO1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPIO1_IRQn);    
}
```
## API参考
[](#hal-gpio)
