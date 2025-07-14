# PMU

HAL PMU provides abstract software interfaces to operate the hardware PMU (Power Management Unit) module, offering the following functionality:
- Set chip power supply parameters
- Set low-power clock parameters
- Configure chip to enter shutdown mode and set wake-up sources. In shutdown mode, the chip can be awakened by PIN and RTC. The mapping relationship between wake-up PINs and GPIO pins is the same as the wake-up PINs in LPSYS sleep mode, see [](aon.md)
- SF32LB52X supports charging, with related interfaces such as #HAL_PMU_ChgInit

For detailed API documentation, refer to [PMU](#hal-pmu)

```{note}
 - `HAL_PMU_CheckBootMode` needs to be executed at every startup to get the boot mode and clear the corresponding status. If the PMU status is not cleared when waking up from shutdown state, `HAL_PMU_EnterHibernate` cannot be called to shut down again
 - After executing `HAL_PMU_EnterHibernate`, the chip will not shut down immediately. It may delay up to one LP Clock cycle before shutting down. During this time, the CPU can continue executing instructions and responding to interrupts. To avoid accidentally executing operations after interrupt triggers, it is recommended to disable interrupts before calling `HAL_PMU_EnterHibernate`. `HAL_PMU_EnterHibernate` itself will be an infinite loop that never exits
```

## Using HAL PMU

### Enable RTC and PIN wake-up before shutdown
```c
void shutdown(void)
{
    /* Enable PIN0 low level wakeup  */
    HAL_PMU_EnablePinWakeup(0, 1);
    /* Enable RTC wakeup */
    HAL_PMU_EnableRtcWakeup();
    /* Disable Interrupt */
    
    /* Shutdown, can be wakeup by PIN0 and RTC */
    HAL_PMU_EnterHibernate();
}

/* Use HAL_PMU_CheckBootMode in initialization stage to check whether it's cold boot or wakeup from shutdown mode */
void init(void)
{
    PMU_BootModeTypeDef boot_mode;
    uint32_t wakeup_src;
    /* check boot mode and clear PMU shutdown state if wakeup from shutdown mode
     */
    HAL_PMU_CheckBootMode(&boot_mode, &wakeup_src);
}
```

### Configure charging
```c
static PMU_ChgHandleTypeDef pmu_chg_handle;

static void pmu_chg_callback(PMU_ChgHandleTypeDef *handle, uint32_t status)
{
    if (status & (1 << PMU_CHG_IRQ_VBUS_RDY))
    {
        printf("vbus: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_VBUS_RDY));
    }

    if (status & (1 << PMU_CHG_IRQ_EOC))
    {
        printf("eoc: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_EOC));
    }

    if (status & (1 << PMU_CHG_IRQ_VBAT_HIGH))
    {
        printf("vbat high: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_VBAT_HIGH));
    }
}

void config_charger(void)
{
    PMU_ChgCalParamTypeDef cal_param;
    uint32_t current;
    uint32_t volt;
    
    //config calibration data for charger, calibration is saved in EFUSE
    //cal_param.cc_mn = xxx;
    //cal_param.cc_mp = xxx;
    //cal_param.bg = xxx;
    //cal_param.cv_vctrl = xxx;
    //cal_param.rep_vctrl = xxx;
    status = HAL_PMU_ChgInit(&pmu_chg_handle, &cal_param);
    
    // config CC current
    current = 300; //300mA
    current = HAL_PMU_ChgConfigCcCurrent(&pmu_chg_handle, current);
    
    // config target voltage
    volt = 4300;  //4300mV
    volt = HAL_PMU_ChgConfigTargetVolt(&pmu_chg_handle, volt);

    // enable IRQ and register callback
    HAL_PMU_ChgRegisterCallback(&pmu_chg_handle, pmu_chg_callback);
    status = HAL_PMU_ChgConfigIRQ(&pmu_chg_handle, PMU_CHG_IRQ_VBUS_RDY, PMU_CHG_IRQ_TRIG_MODE_DOUBLE_EDGE);
    RT_ASSERT(HAL_OK == status);
    status = HAL_PMU_ChgConfigIRQ(&pmu_chg_handle, PMU_CHG_IRQ_VBAT_HIGH, PMU_CHG_IRQ_TRIG_MODE_POS_EDGE);
    RT_ASSERT(HAL_OK == status);
    status = HAL_PMU_ChgConfigIRQ(&pmu_chg_handle, PMU_CHG_IRQ_EOC, PMU_CHG_IRQ_TRIG_MODE_POS_EDGE);
    RT_ASSERT(HAL_OK == status);

    HAL_NVIC_SetPriority(PMUC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(PMUC_IRQn);    
}
```

## API Reference
[](#hal-pmu)
