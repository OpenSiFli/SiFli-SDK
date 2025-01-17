#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include <stdlib.h>
#include "bf0_hal_tsen.h"


static TSEN_HandleTypeDef   TsenHandle;

void TSEN_IRQHandler(void)
{

    rt_kprintf("IRQ Fired\n");
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
        rt_kprintf("Sync: Current temperature is %d degree\n", temperature);

        HAL_NVIC_SetPriority(TSEN_IRQn, 5, 0);                                      /* Set interrupt priority*/
        if (HAL_TSEN_Read_IT(&TsenHandle) == HAL_TSEN_STATE_BUSY)                   /* Read Async, interrupt will be enabled*/
        {
            rt_kprintf("state1:%d\n", TsenHandle.State);
            while (HAL_TSEN_GetState(&TsenHandle) != HAL_TSEN_STATE_READY);
        }
        rt_kprintf("state2:%d\n", TsenHandle.State);
        rt_kprintf("Async: Current temperature is %d degree\n", TsenHandle.temperature);
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;
    /* Output a message on console using printf function */
    rt_kprintf("Start tsen demo!\n");

    while (1)
    {
        testcase();
        HAL_Delay(500);
    }

}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

