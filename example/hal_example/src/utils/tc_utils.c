
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include "utest.h"
#include "board.h"


/**
 * @brief Return an random value of [0, max]
 * @param max - should be less than UINT32_MAX
 * @return Value from 0 ~ max
 */
uint32_t rand_within(uint32_t max)
{
    return  rand() % (max + 1);
}

/*
    Return value from min ~ max
*/
uint32_t rand_in_range(uint32_t min, uint32_t max)
{
    return  min + (rand() % (max - min + 1));
}


void fill_with_random_data(uint8_t *p_buf, uint32_t size)
{
    uint32_t ptr;
    int cnt = 0;

    if ((NULL == p_buf) || (0 == size)) return;

    uint32_t start = (uint32_t) p_buf;
    uint32_t end   = (uint32_t)(p_buf + size - 1);

    uint32_t start_4aligned = start & 0xFFFFFFFC;
    uint32_t end_4aligned   = end   & 0xFFFFFFFC;

    for (ptr = start; ptr < start_4aligned; ptr += 1)
    {
        *(uint8_t *)ptr = (uint8_t) rand();
    }

    for (ptr = start_4aligned; ptr < end_4aligned; ptr += 4)
    {
        *(uint32_t *)ptr = rand();
        if (((++cnt) % 1024) == 0)  // give wdt a chance to run
            rt_thread_mdelay(10);
    }

    for (ptr = end_4aligned; ptr < end; ptr += 1)
    {
        *(uint8_t *)ptr = (uint8_t) rand();
    }
}


void dwtIpInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk | DWT_CTRL_CPIEVTENA_Msk;
}

void dwtReset(void)
{
    DWT->CYCCNT = 0; /* Clear DWT cycle counter */
}

uint32_t dwtGetCycles(void)
{
    return DWT->CYCCNT;
}



