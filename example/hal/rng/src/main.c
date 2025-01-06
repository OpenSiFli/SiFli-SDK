#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"


/* TRBG functions and declarations. */
RNG_HandleTypeDef   RngHandle = {0};

#define RND_BUFFER_SIZE (16)
uint32_t RND_BUFFER[RND_BUFFER_SIZE] = {0};

/**
 * @brief This function generate random numbers.
 * @param[out] buf pointer to uint32_t buffer for storing random numbers.
 * @param[in] size size of buf.
 *
 * @retval none
 */
void random_vect_generate(uint32_t *buf, uint8_t size)
{
    uint32_t value = 0;

    for (int i = 0; i < size; i++)
    {
        /* Generate seed(32-bit). */
        if (HAL_RNG_Generate(&RngHandle, &value,  1) != HAL_OK)
        {
            rt_kprintf("TRNG generate seed failed.\n");
            HAL_ASSERT(0);
        }
        /* Generate random number(32-bit) */
        if (HAL_RNG_Generate(&RngHandle, &value,  0) != HAL_OK)
        {
            rt_kprintf("TRNG generate random number failed.\n");
            HAL_ASSERT(0);
        }
        buf[i] = value;
    }
}

/**
 * @brief Print buf data in hex format.
 *
 * @retval none
 */
void HEX_DUMP(const char *note, uint32_t *buf, uint8_t size)
{
    if (note)
        rt_kprintf("%s\n", note);
    for (int i = 0; i < size; i++)
    {
        rt_kprintf("%08X ", buf[i]);
        if (0 == ((i + 1) % 8))
            rt_kprintf("\n");
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{

    /* TRNG initialization. */
    RngHandle.Instance = TRNG;
    if (HAL_RNG_Init(&RngHandle) != HAL_OK)
    {
        rt_kprintf("TRNG init failed.\n");
        HAL_ASSERT(0);
    }

    /* Infinite loop */
    while (1)
    {
        /* Generate random numbers. */
        random_vect_generate(RND_BUFFER, RND_BUFFER_SIZE);
        /* Print randomw numbers. */
        HEX_DUMP("RND_BUFFER:", RND_BUFFER, RND_BUFFER_SIZE);
        /* Delay 500ms. */
        HAL_Delay(500);
    }
    return 0;
}

