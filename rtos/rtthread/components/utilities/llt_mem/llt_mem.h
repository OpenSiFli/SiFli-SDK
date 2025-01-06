/**
 * @file llt_mem.h
 *
 */

#ifndef __LLT_MEM_H__
#define __LLT_MEM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>


#if defined(RT_USING_LONG_LIFETIME_MEMHEAP)

extern struct rt_memheap llt_mem_heap;

#define LLT_MEM_HEAP   &llt_mem_heap

#define LLT_MEM_ALLOC(size) rt_memheap_alloc(LLT_MEM_HEAP, size)

#define LLT_MEM_REALLOC(ptr, newsize) rt_memheap_realloc(LLT_MEM_HEAP, ptr, newsize)

#define LLT_MEM_FREE(ptr)  rt_memheap_free(ptr)

#define LLT_MEM_CALLOC(count, size) rt_memheap_calloc(LLT_MEM_HEAP, count, size)

#else
#define LLT_MEM_ALLOC(size) rt_malloc(size)

#define LLT_MEM_REALLOC(ptr, newsize) rt_realloc(ptr, newsize)

#define LLT_MEM_FREE(ptr)  rt_free(ptr)

#define LLT_MEM_CALLOC(count, size) rt_calloc(count, size)

#endif /* RT_USING_LONG_LIFETIME_MEMHEAP */


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*__LLT_MEM_H__*/
