# TSEN

TSEN (Temperature SENsor) is used to measure the current temperature of the
chip. Both HCPU and LCPU can use this module to get the current temperature of
the chipset. It can generate an interrupt when measurement data is ready.

## Using TSEN
The following code will measure the chipset temperature without interrupts.

```c
int temperature;
    TSEN_HandleTypeDef   TsenHandle;

    /*##-1- Initialize TSEN peripheral #######################################*/
    HAL_TSEN_Init(&amp;TsenHandle);
    temperature = HAL_TSEN_Read(&amp;TsenHandle);
    printf("Sync: Current temperature is %d Celsius\n", temperature);
```

The following code will measure the chipset temperature and generate an
interrupt when the measurement is ready.
```c
void TSEN_IRQHandler(void)
    {
        LOG_I("IRQ Fired");
        HAL_TSEN_IRQHandler(&amp;TsenHandle);
    }

    int temperature;
    TSEN_HandleTypeDef   TsenHandle;

    /*##-1- Initialize TSEN peripheral #######################################*/
    HAL_TSEN_Init(&amp;TsenHandle);
    if (HAL_TSEN_Read_IT(&amp;TsenHandle) == HAL_TSEN_STATE_BUSY)
    {
        int count = 0;
        while (HAL_TSEN_GetState(&amp;TsenHandle) != HAL_TSEN_STATE_READY);
    }
    printf("Async: Current temperature is %d Celsius\n", TsenHandle.temperature);
```


## API Reference
[]


