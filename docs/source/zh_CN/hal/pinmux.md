# PINMUX

HAL PINMUX提供抽象的软件接口操作硬件PINMUX模块，设置pin的功能和上下拉属性等。
芯片有两个PINMUX实例，HPSYS域的PINMUX1(`hwp_pinmux1`)和LPSYS域的PINMUX2(`hwp_pinmux2`)。
所有pin统一定义在枚举类型 `pin_pad` 中，不再区分HCPU和LCPU；pin可用的功能统一定义在枚举类型 `pin_function` 中。
SF32LB57X系列的专用管脚支持功能可查阅总表 `pad_fsel_func_tbls`。`pin_function` 同时包含任意引脚功能和专用管脚功能：枚举值在16~255之间的为可任意映射的功能（matrix function，起始值为 `PIN_MATRIX_FUNC_START = 16`），枚举值小于16的为专用管脚功能。

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

