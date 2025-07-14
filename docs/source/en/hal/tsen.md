# TSEN

TSEN (Temperature SENsor) is used to measure the current temperature of the chip. Both HCPU and LCPU can use this module to get the current temperature of the chipset. It can generate an interrupt when measurement data is ready.

## Using TSEN
The following code will measure the chipset temperature without interrupts.

```c
    int temperature;
    TSEN_HandleTypeDef   TsenHandle;

    /*##-1- Initialize TSEN peripheral #######################################*/
    HAL_TSEN_Init(&TsenHandle);
    temperature = HAL_TSEN_Read(&TsenHandle);
    printf("Sync: Current temperature is %d degree\n", temperature);
    
```

The following code will measure the chipset temperature and generate an interrupt when the measurement is ready.
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

## API Reference
[](../api/hal/tsen.md)
