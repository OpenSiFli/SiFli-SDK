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

#include "bf0_hal.h"
#include "register.h"

enum
{
    ATTR_CODE_IDX,
    ATTR_RAM_IDX,
    ATTR_DEVICE_IDX
};

#define ATTR_CODE       ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0))
#define ATTR_RAM        ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE)
#define ATTR_DEVICE     ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE, ARM_MPU_ATTR_DEVICE_nGnRnE)

void HAL_MspInit(void)
{
#ifdef SOC_BF0_HCPU
    {
        BSP_IO_Init();
    }
#endif
}

static void mpu_clear_region(void)
{
    for (uint32_t i = 0; i < MPU_REGION_NUM; i++)
    {
        ARM_MPU_ClrRegion(i);
    }
}
void mpu_config(void)
{
//#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uint32_t rnr, rbar, rlar;

    ARM_MPU_Disable();
    mpu_clear_region();

    ARM_MPU_SetMemAttr(ATTR_CODE_IDX, ATTR_CODE);
    ARM_MPU_SetMemAttr(ATTR_RAM_IDX, ATTR_RAM);
    ARM_MPU_SetMemAttr(ATTR_DEVICE_IDX, ATTR_DEVICE);

    rnr = 0;

    //  hpsys rom
    rbar = ARM_MPU_RBAR(0x0, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x0000ffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // hpsys RETM/ITCM ram��disable sram cache
    rbar = ARM_MPU_RBAR(0x00010000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x0002ffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    //  flash1, region 1
    rbar = ARM_MPU_RBAR(0x10000000, ARM_MPU_SH_NON, 1, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x1fffffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // hpsys ram, disable sram cache
    rbar = ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x2027ffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // peripheral
    rbar = ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0, 1, 1); //Non-shareable,RW,any privilege,non-executable
    rlar = ARM_MPU_RLAR(0x5fffffff, ATTR_DEVICE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // psram code
    rbar = ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x601fffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // psram data
    rbar = ARM_MPU_RBAR(0x60200000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x63ffffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // lpsys ram
    rbar = ARM_MPU_RBAR(0x203fc000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x204fffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // lpsys ITCM and CBUS
    rbar = ARM_MPU_RBAR(0x20bfc000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x20cbffff, ATTR_RAM_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // ble rom
    rbar = ARM_MPU_RBAR(0x20800000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RW,any privilege,executable
    rlar = ARM_MPU_RLAR(0x208fffff, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // qspi2
    rbar = ARM_MPU_RBAR(0x64000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x67FFFFFF, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);

    // qspi3
    rbar = ARM_MPU_RBAR(0x68000000, ARM_MPU_SH_NON, 0, 1, 0); //Non-shareable,RO,any privilege,executable
    rlar = ARM_MPU_RLAR(0x6fFFFFFF, ATTR_CODE_IDX);
    ARM_MPU_SetRegion(rnr++, rbar, rlar);


    HAL_ASSERT(rnr <= MPU_REGION_NUM);
    ARM_MPU_Enable(MPU_CTRL_HFNMIENA_Msk);
//#endif

}

#if 1
void HAL_PostMspInit(void)
{

}
#endif
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


