# AON

HAL AON provides abstract software interface to operate hardware AON (Always On) module, used to control low power modes of various subsystems in the chip. The chip is divided into HPSYS and LPSYS subsystems (power domains), corresponding to HPAON ({c:macro}`hwp_hpsys_aon`) and LPAON ({c:macro}`hwp_lpsys_aon`) respectively. The control methods for both power domains are similar, and supported features include:
- PIN, RTC, LPTIM, MAILBOX and manual wakeup. LPSYS also supports LPCOMP and BLE wakeup. PIN wakeup can be level-triggered or edge-triggered. Manual wakeup means another core operates specific registers to wake up the designated core.
  MAILBOX wakeup means waking up the corresponding core by triggering mailbox interrupt, for example, LPSYS can configure #L2H_MAILBOX to trigger MAILBOX interrupt to HPSYS, if HPSYS enables MAILBOX wakeup, it can be automatically awakened by this interrupt.
- HPSYS supports 4 wakeup PINs, LPSYS supports 6 wakeup PINs, all bound to fixed GPIO pins
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

For detailed API documentation, refer to [AON](#hal-aes).

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
    /* Enable PIN0 low level wakeup  */
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
    /* Enable PIN0 low level wakeup  */
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
## API Reference
[](#hal-aon)
