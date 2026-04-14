# PIN设备

通过pin device 可以设置GPIO输入、输出、中断触发，但不支持上下拉。

## 驱动配置

选中{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable GPIO`菜单即可添加PIN设备。`GPIO BASE number`选项不用填写。

对应的宏开关如下：
```c
#define RT_USING_PIN
#define BSP_USING_GPIO
```

## 获取引脚编号(pin id)
不同于HAL层指定GPIO需要指定所在GPIO组和组内的编号，在驱动层通常使用一个数字编号来表示某一个GPIO管脚（包括GPIOA、GPIOB以及部分系列支持的PBR），以下是驱动层对GPIO/PBR编号的方法：

引脚          | pin id | 说明
--------------|--------|-----
GPIOA_00      | 0      | `GET_PIN(1, 0)`
GPIOA_01      | 1      | `GET_PIN(1, 1)`
GPIOA_02      | 2      | `GET_PIN(1, 2)`
...           | ...    | 
GPIOB_00      | 96     | `GET_PIN(2, 0)`
GPIOB_01      | 97     | `GET_PIN(2, 1)`
GPIOB_02      | 98     | `GET_PIN(2, 2)`
...           | ...    | 
PBR0          | 160    | `GET_PIN(0, 0)`
PBR1          | 161    | `GET_PIN(0, 1)`
...           | ...    | 依系列不同，范围见下方说明

举例说明：
 - GPIOB03脚，pin id为 99
 - GPIOA03脚，pin id为 3
 

```{note}
也通过宏定义来获取引脚编号
```
```c
GET_PIN(port, pin)

#define LED0_PIN       GET_PIN(1,  3)   //GPIOA_03
#define LED1_PIN       GET_PIN(2,  9)   //GPIOB_09  
```

```{note}
`PBR` 在 pin device 中使用 `port=0`，即 `GET_PIN(0, pbr_index)`。

芯片差异说明：
- SF32LB52/SF32LB56：支持 `PBR0~PBR3`（pin id `160~163`）
- SF32LB58：支持 `PBR0~PBR5`（pin id `160~165`）
- SF32LB55：不支持 PBR
```




## 示例1——中断模式
PA00 设置为上升沿中断触发模式，等待3秒后关闭中断触发模式
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
	//Enable rasing edge interrupt mode
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

## 示例2——输入或输出模式
PB02 先设置输入模式然后读电平，再输出高电平
```c
void pin_read_and_write(void)
{
	int v;
	rt_base_t pin_id = GET_PIN(2,2);  //Get GPIOB_02 pin id
	//Set pin input mode
	rt_pin_mode(pin_id, PIN_MODE_INPUT_PULLUP);
	v = rt_pin_read(pin_id);
    LOG_I("pin_read value=%d",v);
    
	//Set pin ouput mode
	rt_pin_mode(pin_id, PIN_MODE_OUTPUT);
	//Set pin output high
    rt_pin_write(pin_id, 1);
}
```

[pin]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/pin/pin

## RT-Thread参考文档

- [PIN设备][pin]