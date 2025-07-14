# PINMUX

HAL PINMUX提供抽象的软件接口操作硬件PINMUX模块，设置pin的功能和上下拉属性等。
芯片有两个PINMUX实例，HPSYS域的PINMUX1(`hwp_pinmux1`)和LPSYS域的PINMUX2(`hwp_pinmux2`)。
PINMUX1的pin列表见 `pin_pad_hcpu`, pin可用功能参考 `pin_pad_func_hcpu` ，类似的，PINMUX2的pin列表见 `pin_pad_lcpu` ， pin功能定义
见 `pin_pad_func_lcpu` 。

pinmux的功能从56x的芯片开始(不包括55x,58x)任意一个GPIO都可以作为当前系统任意一个I2C/UART/PWM的IO脚。

详细的API说明参考 [](#hal-pinmux)

## GPIO和pinmux模块的区别
物理上，GPIO需要通过pinmux模块才能和外界连接，如图：
![Figure 1: pinmux模块和GPIO模块的关系](../../assets/relation_of_gpio_pinmux.png)

## 使用HAL PINMUX

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

## API参考
[](#hal-pinmux)

