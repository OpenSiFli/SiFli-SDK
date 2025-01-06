/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
    BSP_IO_Init();
    /* remove this line if LCPU use default 48MHz configured in HAL_PreInit */
    if (!HAL_PMU_LXT_DISABLED())
        HAL_RCC_LCPU_SetDiv(2, 0, 3); //using 24M
}


