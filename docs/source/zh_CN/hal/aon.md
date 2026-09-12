# AON

HAL AON提供抽象的软件接口操作硬件AON(Always On)模块，用于控制芯片各个子系统的低功耗模式，芯片分为HPSYS和LPSYS两个子系统(电源域)，
分别对应HPAON({c:macro}`hwp_hpsys_aon`)和LPAON({c:macro}`hwp_lpsys_aon`)，两个电源域的控制方法类似，支持的特性有:
- PIN、RTC、LPTIM、MAILBOX和手动唤醒，PIN唤醒可以是电平触发也可以是边沿触发，手动唤醒指由另外一个核操作特定寄存器唤醒指定的核,
  MAILBOX唤醒指可以通过触发mailbox中断来唤醒对应的核，比如LPSYS可以配置 #L2H_MAILBOX 触发给HPSYS的MAILBOX中断，若HPSYS使能了MAILBOX唤醒，就能被该中断自动唤醒
- 各系列支持的唤醒源不同（部分系列的LPSYS还支持LPCOMP、BLE/BT唤醒），唤醒PIN的数量和映射关系也不同，详见[唤醒PIN与GPIO管脚映射关系](#唤醒pin与gpio管脚映射关系)
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

唤醒PIN与GPIO管脚固定绑定，可以通过 `HAL_HPAON_QueryWakeupPin()` / `HAL_HPAON_QueryWakeupGpioPin()`（LPSYS对应
`HAL_LPAON_QueryWakeupPin()` / `HAL_LPAON_QueryWakeupGpioPin()`）在唤醒PIN和GPIO管脚之间互查，接口详见[API参考](#hal-aon)。
各系列支持的唤醒源和唤醒PIN数量不同，下面按系列说明，表中 `PBRn` 表示RTC域的按键管脚，编号方法见[PIN设备](../drivers/gpio.md)。

:::{only} SF32LB55X

### SF32LB55X

HPSYS支持4个唤醒PIN，LPSYS支持6个唤醒PIN，两个电源域各自独立。

- HPSYS唤醒源：`HPAON_WAKEUP_SRC_RTC`、`HPAON_WAKEUP_SRC_LPTIM1`、`HPAON_WAKEUP_SRC_LP2HP_REQ`（手动唤醒）、
  `HPAON_WAKEUP_SRC_LP2HP_IRQ`（MAILBOX唤醒），以及4个唤醒PIN `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN3`
- LPSYS唤醒源：`LPAON_WAKEUP_SRC_RTC`、`LPAON_WAKEUP_SRC_LPTIM2`、`LPAON_WAKEUP_SRC_LPCOMP1`、`LPAON_WAKEUP_SRC_LPCOMP2`、
  `LPAON_WAKEUP_SRC_BLE`、`LPAON_WAKEUP_SRC_HP2LP_REQ`（手动唤醒）、`LPAON_WAKEUP_SRC_HP2LP_IRQ`（MAILBOX唤醒），
  以及6个唤醒PIN `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN5`

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

:::

:::{only} SF32LB52X

### SF32LB52X

HPSYS和LPSYS各自支持21个唤醒PIN，并且映射到相同的GPIO管脚上，所以SDK中 `HAL_LPAON_QueryWakeupPin()` /
`HAL_LPAON_QueryWakeupGpioPin()` 直接复用HPSYS的实现。

- HPSYS唤醒源：`HPAON_WAKEUP_SRC_RTC`、`HPAON_WAKEUP_SRC_LPTIM1`、`HPAON_WAKEUP_SRC_PMUC`、`HPAON_WAKEUP_SRC_GPIO1`、
  `HPAON_WAKEUP_SRC_LP2HP_REQ`（手动唤醒）、`HPAON_WAKEUP_SRC_LP2HP_IRQ`（MAILBOX唤醒），
  以及21个唤醒PIN `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN20`
- LPSYS唤醒源：`LPAON_WAKEUP_SRC_RTC`、`LPAON_WAKEUP_SRC_LPTIM3`、`LPAON_WAKEUP_SRC_GPIO2`、`LPAON_WAKEUP_SRC_BT`、
  `LPAON_WAKEUP_SRC_HP2LP_REQ`（手动唤醒）、`LPAON_WAKEUP_SRC_HP2LP_IRQ`（MAILBOX唤醒），
  以及21个唤醒PIN `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN20`

HPSYS/LPSYS唤醒PIN映射表：

唤醒PIN          | GPIO管脚       |
-----------------|----------------|
  PIN0           |  GPIO_A24      |
  PIN1           |  GPIO_A25      |
  PIN2           |  GPIO_A26      |
  PIN3           |  GPIO_A27      |
  PIN10          |  GPIO_A34      |
  PIN11          |  GPIO_A35      |
  PIN12          |  GPIO_A36      |
  PIN13          |  GPIO_A37      |
  PIN14          |  GPIO_A38      |
  PIN15          |  GPIO_A39      |
  PIN16          |  GPIO_A40      |
  PIN17          |  GPIO_A41      |
  PIN18          |  GPIO_A42      |
  PIN19          |  GPIO_A43      |
  PIN20          |  GPIO_A44      |

```{note}
- `PIN4` ~ `PIN9`（`PA28` ~ `PA33`）与ADC复用，已取消唤醒口功能，不能作为唤醒源使用，所以上表中没有列出，
  唤醒PIN的编号仍然保持连续，即 `PIN10` 对应 `PA34`。
- 52X没有独立的PBR管脚，`PBR0` ~ `PBR3` 与 `PA24` ~ `PA27`（即 `PIN0` ~ `PIN3`）复用同一管脚，AON PIN唤醒由PBR模块检测，
  所以使能 `PIN0` ~ `PIN3` 作为唤醒源时SDK会同时打开对应PBR的输入使能。
- DEEPSLEEP模式下的睡眠不需要额外配置唤醒PIN，唤醒源走 `HPAON_WAKEUP_SRC_GPIO1`，PM框架默认使能。
```

:::

:::{only} SF32LB56X

### SF32LB56X

HPSYS和LPSYS各自支持14个唤醒PIN，并且映射到相同的GPIO管脚上。

- HPSYS唤醒源：`HPAON_WAKEUP_SRC_RTC`、`HPAON_WAKEUP_SRC_LPTIM1`、`HPAON_WAKEUP_SRC_GPIO1`、`HPAON_WAKEUP_SRC_LP2HP_REQ`（手动唤醒）、
  `HPAON_WAKEUP_SRC_LP2HP_IRQ`（MAILBOX唤醒），以及14个唤醒PIN `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN13`
- LPSYS唤醒源：`LPAON_WAKEUP_SRC_RTC`、`LPAON_WAKEUP_SRC_LPTIM2`、`LPAON_WAKEUP_SRC_LPCOMP1`、`LPAON_WAKEUP_SRC_LPCOMP2`、
  `LPAON_WAKEUP_SRC_GPIO2`、`LPAON_WAKEUP_SRC_BT`、`LPAON_WAKEUP_SRC_HP2LP_REQ`（手动唤醒）、`LPAON_WAKEUP_SRC_HP2LP_IRQ`（MAILBOX唤醒），
  以及14个唤醒PIN `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN13`

HPSYS/LPSYS唤醒PIN映射表：

唤醒PIN          | GPIO管脚       |
-----------------|----------------|
  PIN0           |  GPIO_B32      |
  PIN1           |  GPIO_B33      |
  PIN2           |  GPIO_B34      |
  PIN3           |  GPIO_B35      |
  PIN4           |  GPIO_B36      |
  PIN5           |  GPIO_A50      |
  PIN6           |  GPIO_A51      |
  PIN7           |  GPIO_A52      |
  PIN8           |  GPIO_A53      |
  PIN9           |  GPIO_A54      |
  PIN10          |  PBR0          |
  PIN11          |  PBR1          |
  PIN12          |  PBR2          |
  PIN13          |  PBR3          |

:::

:::{only} SF32LB57X

### SF32LB57X

HPSYS支持14个唤醒PIN，LPSYS不支持PIN唤醒（`HAL_LPAON_GetWakeupPinMode()` 固定返回 `HAL_ERROR`）。另外57X的PMUC和HPSYS_AON
共用同一个唤醒PIN寄存器。

- HPSYS唤醒源：`HPAON_WAKEUP_SRC_RTC`、`HPAON_WAKEUP_SRC_LPTIM1`、`HPAON_WAKEUP_SRC_GPIO1`、`HPAON_WAKEUP_SRC_CHG`（PMUC唤醒）、
  `HPAON_WAKEUP_SRC_LP2HP_REQ`（手动唤醒）、`HPAON_WAKEUP_SRC_LP2HP_IRQ`（MAILBOX唤醒），
  以及14个唤醒PIN `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN13`
- LPSYS唤醒源：`LPAON_WAKEUP_SRC_LPTIM3`、`LPAON_WAKEUP_SRC_BT`、`LPAON_WAKEUP_SRC_HP2LP_REQ`（手动唤醒）、
  `LPAON_WAKEUP_SRC_HP2LP_IRQ`（MAILBOX唤醒）

HPSYS唤醒PIN映射表，唤醒PIN分为 `PA33` ~ `PA42` 和 `PA24` ~ `PA27` 两段：

唤醒PIN          | GPIO管脚       |
-----------------|----------------|
  PIN0           |  GPIO_A33      |
  PIN1           |  GPIO_A34      |
  PIN2           |  GPIO_A35      |
  PIN3           |  GPIO_A36      |
  PIN4           |  GPIO_A37      |
  PIN5           |  GPIO_A38      |
  PIN6           |  GPIO_A39      |
  PIN7           |  GPIO_A40      |
  PIN8           |  GPIO_A41      |
  PIN9           |  GPIO_A42      |
  PIN10          |  GPIO_A24      |
  PIN11          |  GPIO_A25      |
  PIN12          |  GPIO_A26      |
  PIN13          |  GPIO_A27      |

```{note}
DEEPSLEEP模式下的睡眠不需要额外配置唤醒PIN，唤醒源走 `HPAON_WAKEUP_SRC_GPIO1`，PM框架默认使能。
```

:::

:::{only} SF32LB58X

### SF32LB58X

HPSYS和LPSYS各自支持18个唤醒PIN，并且映射到相同的GPIO管脚上。

- HPSYS唤醒源：`HPAON_WAKEUP_SRC_RTC`、`HPAON_WAKEUP_SRC_LPTIM1`、`HPAON_WAKEUP_SRC_GPIO1`、`HPAON_WAKEUP_SRC_LP2HP_REQ`（手动唤醒）、
  `HPAON_WAKEUP_SRC_LP2HP_IRQ`（MAILBOX唤醒），以及18个唤醒PIN `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN17`
- LPSYS唤醒源：`LPAON_WAKEUP_SRC_RTC`、`LPAON_WAKEUP_SRC_LPTIM2`、`LPAON_WAKEUP_SRC_LPCOMP1`、`LPAON_WAKEUP_SRC_LPCOMP2`、
  `LPAON_WAKEUP_SRC_GPIO2`、`LPAON_WAKEUP_SRC_BT`、`LPAON_WAKEUP_SRC_HP2LP_REQ`（手动唤醒）、`LPAON_WAKEUP_SRC_HP2LP_IRQ`（MAILBOX唤醒），
  以及18个唤醒PIN `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN17`

HPSYS/LPSYS唤醒PIN映射表：

唤醒PIN          | GPIO管脚       |
-----------------|----------------|
  PIN0           |  GPIO_B54      |
  PIN1           |  GPIO_B55      |
  PIN2           |  GPIO_B56      |
  PIN3           |  GPIO_B57      |
  PIN4           |  GPIO_B58      |
  PIN5           |  GPIO_B59      |
  PIN6           |  GPIO_A64      |
  PIN7           |  GPIO_A65      |
  PIN8           |  GPIO_A66      |
  PIN9           |  GPIO_A67      |
  PIN10          |  GPIO_A68      |
  PIN11          |  GPIO_A69      |
  PIN12          |  PBR0          |
  PIN13          |  PBR1          |
  PIN14          |  PBR2          |
  PIN15          |  PBR3          |
  PIN16          |  PBR4          |
  PIN17          |  PBR5          |

:::

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
    /* Enable PIN0 low level wakeup, PIN0 is bound to a fixed gpio pin,
       refer to the wakeup pin mapping table of the target chip series */
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
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;

    /* Query which wakeup pin is mapping to the gpio pin, take GPIO_A80 of
       SF32LB55X as an example, if found return value >=0, otherwise, return -1 */
    wakeup_pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 80);

    /* Query which GPIO PIN is mapping to wakeup pin0,
       if found, return GPIO instance and pin id, otherwise return NULL */
    gpio = HAL_HPAON_QueryWakeupGpioPin(0, &gpio_pin);
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
    /* Enable PIN0 low level wakeup, PIN0 is bound to a fixed gpio pin,
       refer to the wakeup pin mapping table of the target chip series */
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
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;

    /* Query which wakeup pin is mapping to the gpio pin, take GPIO_B43 of
       SF32LB55X as an example, if found return value >=0, otherwise, return -1 */
    wakeup_pin = HAL_LPAON_QueryWakeupPin(hwp_gpio2, 43);

    /* Query which GPIO PIN is mapping to wakeup pin0,
       if found, return GPIO instance and pin id, otherwise return NULL */
    gpio = HAL_LPAON_QueryWakeupGpioPin(0, &gpio_pin);
}

```
## API参考
[](#hal-aon)
