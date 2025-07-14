# TSEN Example
Source code path: example/hal/tsen

## Supported Platforms
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4

## Overview
Used to represent the "Temperature Sensor" hardware module. Specifically, TSEN is an integrated temperature sensor designed to measure on-chip temperature, helping to implement temperature monitoring and temperature-related system functions.

## Example Usage
### Compilation and Programming
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted for download:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

For detailed steps on compilation and download, please refer to the relevant introduction in [](/quickstart/get-started.md).

## Methods for Using TSEN
The following are general steps for using TSEN:

### Initialize TSEN:
The initialization function internally enables TSEN and starts temperature measurement:
```c
 HAL_TSEN_Init(TSEN_HandleTypeDef *htsen)
```

### Read Temperature Data:
#### Method 1: Synchronous Reading
```c
temperature = HAL_TSEN_Read(&TsenHandle);
```

#### Method 2: Asynchronous Reading (Interrupt)
```c
HAL_TSEN_Read_IT(&TsenHandle)
```

#### Interrupt Function Content
```c
void TSEN_IRQHandler(void)
{

    printf("IRQ Fired\n");
    HAL_TSEN_IRQHandler(&TsenHandle);
}

__HAL_ROM_USED void HAL_TSEN_IRQHandler(TSEN_HandleTypeDef *htsen)
{
    htsen->Instance->TSEN_IRQ |= TSEN_TSEN_IRQ_TSEN_ICR;
    htsen->temperature = HAL_TSEN_Data(htsen);
    HAL_TSEN_Disable(htsen);
    NVIC_DisableIRQ(TSEN_IRQn);
}
```

#### Interrupt Function Content Jump Changes State Value
In the interrupt method, use `while (HAL_TSEN_GetState(&TsenHandle) != HAL_TSEN_STATE_READY)` to monitor whether new data is generated

![state_change](./assets/state_change.png)

## Code Section
```c
static TSEN_HandleTypeDef   TsenHandle;

void TSEN_IRQHandler(void)
{

    printf("IRQ Fired\n");
    HAL_TSEN_IRQHandler(&TsenHandle);
}



static void testcase(void)
{
    // HAL_StatusTypeDef   status;
    int temperature;

    /*##-1- Initialize TSEN peripheral #######################################*/
    TsenHandle.Instance = hwp_tsen;
    if (HAL_TSEN_Init(&TsenHandle) == HAL_OK)
    {
        temperature = HAL_TSEN_Read(&TsenHandle);                                   /* Read synchronized*/
        printf("Sync: Current temperature is %d degree\n", temperature);

        HAL_NVIC_SetPriority(TSEN_IRQn, 5, 0);                                      /* Set interrupt priority*/
        if (HAL_TSEN_Read_IT(&TsenHandle) == HAL_TSEN_STATE_BUSY)                   /* Read Async, interrupt will be enabled*/
        {
            printf("state1:%d\n",TsenHandle.State);
            while (HAL_TSEN_GetState(&TsenHandle) != HAL_TSEN_STATE_READY);
        }
        printf("state2:%d\n",TsenHandle.State);    
        printf("Async: Current temperature is %d degree\n", TsenHandle.temperature);
    }
}
```

### Example Output Results Display:
```c
Start tsen demo!
Sync: Current temperature is 19 degree
state1:4
IRQ Fired
state2:1
Async: Current temperature is 19 degree
```
![tsen_Serial_print](./assets/tsen_Serial_print.png)

## Exception Diagnosis
If the expected log temperature changes do not appear, troubleshooting can be performed from the following aspects:
* Whether hardware connections are normal
* Whether interrupt handling functions are normal
* Whether state value changes to HAL_TSEN_STATE_READY

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
|0.0.2 |12/2024 |2.0 |