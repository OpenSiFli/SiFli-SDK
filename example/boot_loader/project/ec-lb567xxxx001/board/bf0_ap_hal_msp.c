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
    HAL_PIN_Set(PAD_PA30, USART1_RXD, PIN_PULLUP, 1);   // UART1_RXD
    HAL_PIN_Set(PAD_PA34, USART1_TXD, PIN_PULLUP, 1);   // UART1_TXD

    // MPI2 PSRAM
    HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB05, MPI2_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB06, MPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB07, MPI2_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB08, MPI2_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB11, MPI2_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB12, MPI2_CS,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB09, MPI2_DQSDM, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA03, GPIO_A3,  PIN_PULLUP, 1);             // MPI3_SD2_EN

    __HAL_WDT_DISABLE();

    BSP_SetFlash2DIV(1);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


