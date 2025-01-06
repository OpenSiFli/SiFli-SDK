/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "bf0_hal.h"
#include "drv_io.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
#ifdef BSP_USING_UART4
    HAL_PIN_Set(PAD_PB17, USART4_TXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB16, USART4_RXD, PIN_PULLUP, 0);
#elif defined(BSP_USING_UART1)
    HAL_PIN_Set(PAD_PA17, USART1_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA18, USART1_RXD, PIN_PULLUP, 1);
#else
#error Select UART1/4 for bootloader
#endif


    HAL_PIN_Set(PAD_SA01, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA04, MPI1_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA08, MPI1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA11, MPI1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA05, MPI1_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA12, MPI1_DQSDM, PIN_PULLDOWN, 1);

    __HAL_WDT_DISABLE();

    BSP_SetFlash1DIV(1);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


