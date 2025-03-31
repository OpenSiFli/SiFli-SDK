# PMU

HAL PMU提供抽象的软件接口操作硬件PMU(Power Management Unit)模块，提供的功能有:
- 设置芯片的供电参数
- 设置低功耗时钟参数，
- 配置芯片进入关机模式并设置唤醒源, 在关机模式下芯片可被PIN和RTC唤醒, 唤醒PIN与GPIO管脚的映射关系与LPSYS睡眠模式的唤醒PIN相同，参见[](aon.md)
- SF32LB52X支持充电，相关接口为 #HAL_PMU_ChgInit 等

详细的API说明参考[PMU](#hal-pmu)

```{note}
 - `HAL_PMU_CheckBootMode` 需要在每次启动时执行以获取开机模式并清除相应状态，如果关机状态醒来不清PMU的状态，
  就无法调用 `HAL_PMU_EnterHibernate` 再次关机
 - 执行 `HAL_PMU_EnterHibernate` 后芯片并不会马上关机, 最长会延迟一个LP Clock周期才会关机, 这期间CPU还能继续执行指令和响应中断，
为避免中断触发后误执行一些操作, 建议在调用 `HAL_PMU_EnterHibernate` 前关闭中断, `HAL_PMU_EnterHibernate` 本身会是一个死循环, 永远不会退出
```

## 使用HAL PMU

### 使能RTC和PIN唤醒后关机
```c
void shutdown(void)
{
    /* Enable PIN0 low level wakeup  */
    HAL_PMU_EnablePinWakeup(0, 1);
    /* Enble RTC wakeup */
    HAL_PMU_EnableRtcWakeup();
    /* Disable Interrupt */
    
    /* Shutdown, can be wakeup by PIN0 and RTC */
    HAL_PMU_EnterHibernate();
    
}

/* Use HAL_PMU_CheckBootMode in initializtion stage to check whether it's cold boot or wakeup from shutdown mode */
void init(void)
{
    PMU_BootModeTypeDef boot_mode;
    uint32_t wakeup_src;
    /* check boot mode and clear PMU shutdown state if wakeup from shutdown mode
     */
    HAL_PMU_CheckBootMode(&boot_mode, &wakeup_src);
}
```

### 配置充电
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


## API参考
[](#hal-pmu)


