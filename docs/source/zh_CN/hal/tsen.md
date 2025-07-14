# TSEN

TSEN（Temperature SENsor）用于测量芯片当前的温度。 HCPU 和 LCPU 都可以使用该模块来获取芯片组的当前温度。 当测量数据准备好时，它可以产生中断。

## 使用TSEN
 以下代码将在不中断的情况下测量芯片组的温度。

```c
    int temperature;
    TSEN_HandleTypeDef   TsenHandle;

    /*##-1- Initialize TSEN peripheral #######################################*/
    HAL_TSEN_Init(&TsenHandle);
    temperature = HAL_TSEN_Read(&TsenHandle);
    printf("Sync: Current temperature is %d degree\n", temperature);
    
```

下面的代码将测量芯片组的温度，当测量准备好时产生一个中断。
```c

    void TSEN_IRQHandler(void)
    {
        LOG_I("IRQ Fired");
        HAL_TSEN_IRQHandler(&TsenHandle);
    }

    int temperature;
    TSEN_HandleTypeDef   TsenHandle;

    /*##-1- Initialize TSEN peripheral #######################################*/
    HAL_TSEN_Init(&TsenHandle);
    if (HAL_TSEN_Read_IT(&TsenHandle) == HAL_TSEN_STATE_BUSY)
    {
        int count = 0;
        while (HAL_TSEN_GetState(&TsenHandle) != HAL_TSEN_STATE_READY);
    }
    printf("Async: Current temperature is %d degree\n", TsenHandle.temperature);    
```


## API参考
[](../api/hal/tsen.md)


