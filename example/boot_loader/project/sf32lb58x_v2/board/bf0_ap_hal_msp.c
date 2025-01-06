/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
#ifdef BSP_USING_UART4
    HAL_PIN_Set(PAD_PB37, USART4_TXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB36, USART4_RXD, PIN_PULLUP, 0);
#elif defined(BSP_USING_UART1)
    HAL_PIN_Set(PAD_PA31, USART1_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA32, USART1_RXD, PIN_PULLUP, 1);
#else
#error Select UART1/4 for bootloader
#endif


    HAL_PIN_Set(PAD_SA04, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA05, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA06, MPI1_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_DIO3, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_SA09, MPI1_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_CLK,   PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_SA11, MPI1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA12, MPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA13, MPI1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA14, MPI1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA15, MPI1_DQS0,  PIN_PULLDOWN, 1);

    __HAL_WDT_DISABLE();

    BSP_SetFlash1DIV(1);
}



