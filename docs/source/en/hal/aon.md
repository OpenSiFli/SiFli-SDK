# AON

HAL AON provides abstract software interface to operate hardware AON (Always On) module, used to control low power modes of various subsystems in the chip. The chip is divided into HPSYS and LPSYS subsystems (power domains), corresponding to HPAON ({c:macro}`hwp_hpsys_aon`) and LPAON ({c:macro}`hwp_lpsys_aon`) respectively. The control methods for both power domains are similar, and supported features include:
- PIN, RTC, LPTIM, MAILBOX and manual wakeup. PIN wakeup can be level-triggered or edge-triggered. Manual wakeup means another core operates specific registers to wake up the designated core.
  MAILBOX wakeup means waking up the corresponding core by triggering mailbox interrupt, for example, LPSYS can configure #L2H_MAILBOX to trigger MAILBOX interrupt to HPSYS, if HPSYS enables MAILBOX wakeup, it can be automatically awakened by this interrupt.
- Supported wakeup sources are different among chip series (the LPSYS of some series also supports LPCOMP and BLE/BT wakeup), and the number of wakeup PINs and their mapping are also different, refer to [Wakeup PIN to GPIO Pin Mapping](#wakeup-pin-to-gpio-pin-mapping)
- Supports LIGHT/DEEP/STANDBY three low power modes. In LIGHT and DEEP modes, digital modules will not power down, all registers and SRAM will be retained. In STANDBY mode, digital modules will power down, all registers will be lost, SRAM can be selectively retained.

````{note}
Due to delay in PIN edge detection, if woken up by other wakeup sources when there's a wakeup PIN level change, the PIN wakeup flag in WSR register may still be 0 when AON interrupt occurs, and becomes 1 after a while. Since the corresponding GPIO edge detection is not ready yet, the PIN wakeup status in WSR register won't be cleared and will keep not sleeping while missing one GPIO interrupt for edge detection. If not using `SysTick_Handler` implemented in drv_common.c in SDK as SysTick interrupt service routine, it's recommended to add the following code in custom SysTick interrupt service routine. When edge-triggered wakeup PIN flag is found to be 1, manually trigger GPIO interrupt callback function once.
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

## Wakeup PIN to GPIO Pin Mapping

A wakeup PIN is bound to a fixed GPIO pin, `HAL_HPAON_QueryWakeupPin()` / `HAL_HPAON_QueryWakeupGpioPin()`
(`HAL_LPAON_QueryWakeupPin()` / `HAL_LPAON_QueryWakeupGpioPin()` for LPSYS) can be used to query one from the other,
refer to [API Reference](#hal-aon). Supported wakeup sources and the number of wakeup PINs are different among chip
series, they are described below per series. `PBRn` in the tables means the key pin in RTC domain, refer to
[PIN device](../drivers/gpio.md) for its numbering.

:::{only} SF32LB55X

### SF32LB55X

HPSYS supports 4 wakeup PINs and LPSYS supports 6 wakeup PINs, the two power domains are independent.

- HPSYS wakeup sources: `HPAON_WAKEUP_SRC_RTC`, `HPAON_WAKEUP_SRC_LPTIM1`, `HPAON_WAKEUP_SRC_LP2HP_REQ` (manual wakeup),
  `HPAON_WAKEUP_SRC_LP2HP_IRQ` (MAILBOX wakeup), and 4 wakeup PINs `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN3`
- LPSYS wakeup sources: `LPAON_WAKEUP_SRC_RTC`, `LPAON_WAKEUP_SRC_LPTIM2`, `LPAON_WAKEUP_SRC_LPCOMP1`, `LPAON_WAKEUP_SRC_LPCOMP2`,
  `LPAON_WAKEUP_SRC_BLE`, `LPAON_WAKEUP_SRC_HP2LP_REQ` (manual wakeup), `LPAON_WAKEUP_SRC_HP2LP_IRQ` (MAILBOX wakeup),
  and 6 wakeup PINs `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN5`

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

HPSYS and LPSYS each support 21 wakeup PINs, and they are mapped to the same GPIO pins, so `HAL_LPAON_QueryWakeupPin()` /
`HAL_LPAON_QueryWakeupGpioPin()` in SDK directly reuse the implementations of HPSYS.

- HPSYS wakeup sources: `HPAON_WAKEUP_SRC_RTC`, `HPAON_WAKEUP_SRC_LPTIM1`, `HPAON_WAKEUP_SRC_PMUC`, `HPAON_WAKEUP_SRC_GPIO1`,
  `HPAON_WAKEUP_SRC_LP2HP_REQ` (manual wakeup), `HPAON_WAKEUP_SRC_LP2HP_IRQ` (MAILBOX wakeup),
  and 21 wakeup PINs `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN20`
- LPSYS wakeup sources: `LPAON_WAKEUP_SRC_RTC`, `LPAON_WAKEUP_SRC_LPTIM3`, `LPAON_WAKEUP_SRC_GPIO2`, `LPAON_WAKEUP_SRC_BT`,
  `LPAON_WAKEUP_SRC_HP2LP_REQ` (manual wakeup), `LPAON_WAKEUP_SRC_HP2LP_IRQ` (MAILBOX wakeup),
  and 21 wakeup PINs `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN20`

HPSYS/LPSYS wakeup PIN mapping table:

Wakeup PIN       | GPIO pin       |
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
- `PIN4` ~ `PIN9` (`PA28` ~ `PA33`) are shared with ADC and their wakeup function has been removed, they can't be used
  as wakeup sources, so they are not listed in the table above. The numbering of wakeup PINs is still continuous,
  that is, `PIN10` is mapped to `PA34`.
- 52X has no dedicated PBR pin like 56X and 58X, `PBR0` ~ `PBR3` share the pad with `PA24` ~ `PA27` (that is, `PIN0` ~ `PIN3`),
  and AON PIN wakeup detection is done by PBR module, so SDK also enables the input of the corresponding PBR when
  `PIN0` ~ `PIN3` are enabled as wakeup sources.
- Sleep in DEEPSLEEP mode doesn't need extra wakeup PIN configuration, the wakeup source is `HPAON_WAKEUP_SRC_GPIO1`,
  which is enabled by PM framework by default.
```

:::

:::{only} SF32LB56X

### SF32LB56X

HPSYS and LPSYS each support 14 wakeup PINs, and they are mapped to the same GPIO pins.

- HPSYS wakeup sources: `HPAON_WAKEUP_SRC_RTC`, `HPAON_WAKEUP_SRC_LPTIM1`, `HPAON_WAKEUP_SRC_GPIO1`,
  `HPAON_WAKEUP_SRC_LP2HP_REQ` (manual wakeup), `HPAON_WAKEUP_SRC_LP2HP_IRQ` (MAILBOX wakeup),
  and 14 wakeup PINs `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN13`
- LPSYS wakeup sources: `LPAON_WAKEUP_SRC_RTC`, `LPAON_WAKEUP_SRC_LPTIM2`, `LPAON_WAKEUP_SRC_LPCOMP1`, `LPAON_WAKEUP_SRC_LPCOMP2`,
  `LPAON_WAKEUP_SRC_GPIO2`, `LPAON_WAKEUP_SRC_BT`, `LPAON_WAKEUP_SRC_HP2LP_REQ` (manual wakeup),
  `LPAON_WAKEUP_SRC_HP2LP_IRQ` (MAILBOX wakeup), and 14 wakeup PINs `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN13`

HPSYS/LPSYS wakeup PIN mapping table:

Wakeup PIN       | GPIO pin       |
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

HPSYS supports 14 wakeup PINs, LPSYS doesn't support PIN wakeup (`HAL_LPAON_GetWakeupPinMode()` always returns `HAL_ERROR`).
Besides, PMUC and HPSYS_AON of 57X share the same wakeup PIN register.

- HPSYS wakeup sources: `HPAON_WAKEUP_SRC_RTC`, `HPAON_WAKEUP_SRC_LPTIM1`, `HPAON_WAKEUP_SRC_GPIO1`, `HPAON_WAKEUP_SRC_CHG`
  (PMUC wakeup), `HPAON_WAKEUP_SRC_LP2HP_REQ` (manual wakeup), `HPAON_WAKEUP_SRC_LP2HP_IRQ` (MAILBOX wakeup),
  and 14 wakeup PINs `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN13`
- LPSYS wakeup sources: `LPAON_WAKEUP_SRC_LPTIM3`, `LPAON_WAKEUP_SRC_BT`, `LPAON_WAKEUP_SRC_HP2LP_REQ` (manual wakeup),
  `LPAON_WAKEUP_SRC_HP2LP_IRQ` (MAILBOX wakeup)

HPSYS wakeup PIN mapping table, the wakeup PINs are divided into two parts, `PA33` ~ `PA42` and `PA24` ~ `PA27`:

Wakeup PIN       | GPIO pin       |
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
Sleep in DEEPSLEEP mode doesn't need extra wakeup PIN configuration, the wakeup source is `HPAON_WAKEUP_SRC_GPIO1`,
which is enabled by PM framework by default.
```

:::

:::{only} SF32LB58X

### SF32LB58X

HPSYS and LPSYS each support 18 wakeup PINs, and they are mapped to the same GPIO pins.

- HPSYS wakeup sources: `HPAON_WAKEUP_SRC_RTC`, `HPAON_WAKEUP_SRC_LPTIM1`, `HPAON_WAKEUP_SRC_GPIO1`,
  `HPAON_WAKEUP_SRC_LP2HP_REQ` (manual wakeup), `HPAON_WAKEUP_SRC_LP2HP_IRQ` (MAILBOX wakeup),
  and 18 wakeup PINs `HPAON_WAKEUP_SRC_PIN0` ~ `HPAON_WAKEUP_SRC_PIN17`
- LPSYS wakeup sources: `LPAON_WAKEUP_SRC_RTC`, `LPAON_WAKEUP_SRC_LPTIM2`, `LPAON_WAKEUP_SRC_LPCOMP1`, `LPAON_WAKEUP_SRC_LPCOMP2`,
  `LPAON_WAKEUP_SRC_GPIO2`, `LPAON_WAKEUP_SRC_BT`, `LPAON_WAKEUP_SRC_HP2LP_REQ` (manual wakeup),
  `LPAON_WAKEUP_SRC_HP2LP_IRQ` (MAILBOX wakeup), and 18 wakeup PINs `LPAON_WAKEUP_SRC_PIN0` ~ `LPAON_WAKEUP_SRC_PIN17`

HPSYS/LPSYS wakeup PIN mapping table:

Wakeup PIN       | GPIO pin       |
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

## Using HAL HPAON

### Configure Sleep
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

### Query Wakeup PIN and Corresponding GPIO Pin
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

## Using HAL LPAON

### Configure Sleep
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

### Query Wakeup PIN and Corresponding GPIO Pin
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
## API Reference
[](#hal-aon)
