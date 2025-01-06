/**
 * @file llt_mem.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include "llt_mem.h"

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

#if defined(RT_USING_LONG_LIFETIME_MEMHEAP)

ALIGN(4) static rt_uint8_t llt_mem_pool[LLT_MEMHEAP_SIZE];
struct rt_memheap llt_mem_heap;


int llt_mem_init(void)
{
    rt_memheap_init(LLT_MEM_HEAP, "llt", llt_mem_pool, sizeof(llt_mem_pool));
    return 0;
}
INIT_COMPONENT_EXPORT(llt_mem_init);

#endif /* RT_USING_LONG_LIFETIME_MEMHEAP */

