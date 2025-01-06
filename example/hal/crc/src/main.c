#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>

/* Common functions for RT-Thread based platform -----------------------------------------------*/

/* User code start from here --------------------------------------------------------*/

/* Input data for test. */
ALIGN(4)
uint8_t g_raw_data[] = "This is a test data used for testing crc algorithm with the crc hardware block.";

/**
  * @brief  Print hexadecimal data.
  * @param  info Customized information.
  * @param  buf Data buffer.
  * @param  len Data length.
  *
  * @retval None
  */
static void dump_hex(const char *info, const uint8_t *buf, size_t len)
{
    size_t i, col;
    col = 0;
    rt_kprintf("%s\n", info);
    for (i = 0; i < len; i++)
    {
        rt_kprintf(" 0x%02x,", buf[i]);
        if (++col == 8)
        {
            rt_kprintf("\n");
            col = 0;
        }
    }
    if (col != 0)
        rt_kprintf("\n");
}

/**
 * @brief Common initialization.
 */
static rt_err_t comm_init(void)
{
    // Do nothing in this project.
    return RT_EOK;
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    uint32_t crc_result = 0;

    /* Common initialization. */
    comm_init();

    /* CRC hardware block initialization. */
    CRC_HandleTypeDef crc_handle = {0};
    crc_handle.Instance = hwp_crc;
    HAL_StatusTypeDef status = HAL_CRC_Init(&crc_handle);
    HAL_ASSERT(HAL_OK == status);

    dump_hex("Input raw data:", g_raw_data, strlen((const char *)g_raw_data));

    /*
     * CRC calculations
     * Supported CRC mode see enum HAL_CRC_Mode in bf0_hal_crc.h .
     */
    for (int mode = CRC_7_MMC; mode < CRC_MODE_NUM; mode ++)
    {
        /* Set crc mode,*/
        HAL_CRC_Setmode(&crc_handle, mode);
        /* Calculate crc. */
        crc_result = HAL_CRC_Calculate(&crc_handle, (uint8_t *)g_raw_data, strlen((const char *)g_raw_data));
        rt_kprintf("CRC Result(mode:%d):\n0x%X\n", mode, crc_result);
    }

    /* Base on CRC_8 mode, customize initial value (0xFF) and polynomial (0x1D). */
    HAL_CRC_Setmode_Customized(&crc_handle, 0xFF, 0x1D, CRC_8);
    /* Calculate crc. */
    crc_result = HAL_CRC_Calculate(&crc_handle, (uint8_t *)g_raw_data, strlen((const char *)g_raw_data));
    rt_kprintf("CRC Result(CRC_8 initial value:0xFF poly:0x1D):\n0x%X \n", crc_result);

    /* Infinite loop */
    while (1)
    {
        /* Do nothing. */
    }

    return 0;
}

