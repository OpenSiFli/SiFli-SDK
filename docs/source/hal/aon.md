# AON

HAL AON提供抽象的软件接口操作硬件AON(Always On)模块，用于控制芯片各个子系统的低功耗模式，芯片分为HPSYS和LPSYS两个子系统(电源域)，
分别对应HPAON({c:macro}`hwp_hpsys_aon`)和LPAON({c:macro}`hwp_lpsys_aon`)，两个电源域的控制方法类似，支持的特性有:
- PIN、RTC、LPTIM、MAILBOX和手动唤醒，LPSYS还支持LPCOMP和BLE唤醒， PIN唤醒可以是电平触发也可以是边沿触发，手动唤醒指由另外一个核操作特定寄存器唤醒指定的核,
  MAILBOX唤醒指可以通过触发mailbox中断来唤醒对应的核，比如LPSYS可以配置 #L2H_MAILBOX 触发给HPSYS的MAILBOX中断，若HPSYS使能了MAILBOX唤醒，就能被该中断自动唤醒
- HPSYS支持4个唤醒PIN，LPSYS支持6个唤醒PIN，他们都与固定的GPIO管脚绑定
- 支持LIGHT/DEEP/STANDBY三种低功耗模式，其中LIGHT和DEEP模式数字模块不会掉电，所有寄存器和SRAM都会保留，STANDBY模式数字模块会掉电，所有寄存器都会丢失，SRAM可以有选择的保留

````{note}
由于PIN的边沿检测存在延迟，如果被其它唤醒源唤醒时刚好有唤醒PIN的电平变化，就有可能在AON中断时看到WSR寄存器中的PIN唤醒标志还是0，过了一会儿才变为1，又因为对应的GPIO边沿检测这时还没准备好，
就会导致WSR寄存器的PIN唤醒状态没有被清掉而一直不睡眠并且会丢失一次边沿检测的GPIO中断，如果没有使用SDK里drv_common.c实现的`SysTick_Handler`作为SysTick的中断服务程序，
建议在自定义的SysTick中断服务程序中添加如下代码，当发现边沿触发的唤醒PIN标志为1时，手动触发一次GPIO中断回调函数。
```c
    /* Trigger GPIO callback manually as GPIO edge detection interrupt may get lost
    and WSR.PIN status is not cleared */
#ifdef SOC_BF0_HCPU
    status = HAL_HPAON_GET_WSR() & HPSYS_AON_WSR_PIN_ALL;
    pin_wsr = status >> HPSYS_AON_WSR_PIN0_Pos;
    HAL_HPAON_CLEAR_WSR(status);
    wake_pin_num = HPSYS_AON_WSR_PIN_NUM;
#else
    status = HAL_LPAON_GET_WSR() & LPSYS_AON_WSR_PIN_ALL;
    pin_wsr = status >> LPSYS_AON_WSR_PIN0_Pos;
    HAL_LPAON_CLEAR_WSR(status);
    wake_pin_num = LPSYS_AON_WSR_PIN_NUM;
#endif

    for (i = 0; (i < wake_pin_num) && pin_wsr; i++)
    {
        if (pin_wsr & 1)
        {

            hal_status = HAL_AON_GetWakePinMode(i, &pin_mode);
            if ((HAL_OK == hal_status) && (pin_mode != AON_PIN_MODE_HIGH)
                    && (pin_mode != AON_PIN_MODE_LOW))
            {
                gpio = HAL_AON_QueryWakeupGpioPin(i, &pin);
                RT_ASSERT(gpio);
                HAL_GPIO_EXTI_Callback(gpio, pin);
            }
        }
        pin_wsr >>= 1;
    }
```
````

## 唤醒PIN与GPIO管脚映射关系
### SF32LB55X
#### HPSYS

Wakeup PIN       | GPIO           | 
-----------------|----------------|
  PIN0           |  GPIO_A77      |
  PIN1           |  GPIO_A78      |  
  PIN2           |  GPIO_A79      |  
  PIN3           |  GPIO_A80      |  


#### LPSYS

Wakeup PIN       | GPIO           | 
-----------------|----------------|
  PIN0           |  GPIO_B43      |
  PIN1           |  GPIO_B44      |  
  PIN2           |  GPIO_B45      |  
  PIN3           |  GPIO_B46      |  
  PIN4           |  GPIO_B47      |  
  PIN5           |  GPIO_B48      |  

详细的API说明参考 [AON](#hal-aes).

## 使用HAL HPAON

### 配置睡眠
```c
void example(void)
{
    /* Enable LPTIM1 as wakeup source */
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_LPTIM1, AON_PIN_MODE_HIGH);
    /* Enable MAILBOX interrupt triggered by LPSYS as wakeup source */
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_LP2HP_IRQ, AON_PIN_MODE_HIGH);
    /* Enable manual wakeup triggered by LPSYS */
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_LP2HP_REQ, AON_PIN_MODE_HIGH);
    /* Enable PIN0 low level wakeup  */
    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN0, AON_PIN_MODE_LOW);

    ...
    
    /* Configure HPSYS enter LIGHT mode */
    HAL_HPAON_EnterLightSleep(0);
}

```

### 查询唤醒PIN和对应的GPIO管脚
```c
void example(void)
{
    int8_t wakeup_pin;
    uint16_t *gpio_pin;
    GPIO_TypeDef *gpio;
    
    /* Query which wakeup pin is mapping to GPIO_A80, 
       if found return value >=0, otherwise, return -1 */
    wakeup_pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 80);

    /* Query which GPIO PIN is mapping to wakeup pin0, 
       if found, return GPIO instance and pin id, otherwise return NULL */
    gpio = HAL_HPAON_QueryWakeupGpioPin(0, &pin);
}

```


## 使用HAL LPAON

### 配置睡眠
```c
void example(void)
{
    /* Enable LPTIM2 as wakeup source */
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_LPTIM2, AON_PIN_MODE_HIGH);
    /* Enable MAILBOX interrupt triggered by HPSYS as wakeup source */
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_HP2LP_IRQ, AON_PIN_MODE_HIGH);
    /* Enable manual wakeup triggered by HPSYS */
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_HP2LP_REQ, AON_PIN_MODE_HIGH);
    /* Enable PIN0 low level wakeup  */
    HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN0, AON_PIN_MODE_LOW);

    ...
    
    /* Configure LPSYS enter LIGHT mode */
    HAL_LPAON_EnterLightSleep(0);
}

```

### 查询唤醒PIN和对应的GPIO管脚
```c
void example(void)
{
    int8_t wakeup_pin;
    uint16_t *gpio_pin;
    GPIO_TypeDef *gpio;
    
    /* Query which wakeup pin is mapping to GPIO_B43, 
       if found return value >=0, otherwise, return -1 */
    wakeup_pin = HAL_LPAON_QueryWakeupPin(hwp_gpio2, 43);

    /* Query which GPIO PIN is mapping to wakeup pin0, 
       if found, return GPIO instance and pin id, otherwise return NULL */
    gpio = HAL_HPAON_QueryWakeupGpioPin(0, &pin);
}

```
## API参考
[](#hal-aon)

