/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2008-7-12      Bernard      the first version
 * 2010-06-09     Bernard      fix the end stub of heap
 *                             fix memory check in rt_realloc function
 * 2010-07-13     Bernard      fix RT_ALIGN issue found by kuronca
 * 2010-10-14     Bernard      fix rt_realloc issue when realloc a NULL pointer.
 * 2017-07-14     armink       fix rt_realloc issue when new size is 0
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *         Simon Goldschmidt
 *
 */

#include <rthw.h>
#include <rtthread.h>
#ifdef _MSC_VER
    #include <intrin.h>
#endif

#ifdef RT_USING_MEMHEAP2

#define RT_MEMHEAP_BACKUP_OPT

#ifdef _MSC_VER
    #define MEMHEAP_RET_ADDR  (rt_uint32_t) _ReturnAddress()
#else
    #define MEMHEAP_RET_ADDR  (rt_uint32_t) __builtin_return_address(0)
#endif

#define RT_MEMHEAP_CONT_USED_BLOCK_LEN_FIELD_SIZE  (sizeof(rt_uint32_t))

/* #define RT_MEM_DEBUG */

#define RT_MEMHEAP_MAGIC        0x1ea01ea0
#define RT_MEMHEAP_MASK         0xfffffffe
#define RT_MEMHEAP_USED         0x01
#define RT_MEMHEAP_FREED        0x00

#define RT_MEMHEAP_IS_USED(i)   ((i)->magic & RT_MEMHEAP_USED)



#define RT_MEMHEAP_MIN_SIZE 12
#define RT_MEMHEAP_MIN_SIZE_ALIGNED     RT_ALIGN(RT_MEMHEAP_MIN_SIZE, RT_ALIGN_SIZE)
#define RT_MEMHEAP_SIZE                 RT_ALIGN(sizeof(struct rt_memheap_item), RT_ALIGN_SIZE)
#define RT_MEMHEAP_DATA_SIZE(heap,item)      ((rt_uint8_t *)((item)->next) - (rt_uint8_t *)(item) - RT_MEMHEAP_SIZE)

typedef struct
{
    void *src;
    rt_ubase_t size;
} rt_memheap_default_backup_hdr_t;



#ifdef RT_USING_MEMTRACE
#ifdef RT_MEM_RECORD_THREAD_NAME
rt_inline void rt_mem_setname(struct rt_memheap_item *mem, const char *name)
{
    int index;
    for (index = 0; index < sizeof(mem->thread); index ++)
    {
        if (name[index] == '\0') break;
        mem->thread[index] = name[index];
    }

    for (; index < sizeof(mem->thread); index ++)
    {
        mem->thread[index] = ' ';
    }
}
#endif /* RT_MEM_RECORD_THREAD_NAME */
#endif

static void plug_holes(struct rt_memheap_item *mem)
{
    struct rt_memheap_item *nmem;
    struct rt_memheap_item *pmem;
    struct rt_memheap *memheap;

    memheap = mem->pool_ptr;

    RT_ASSERT((rt_uint8_t *)mem >= memheap->start);
    RT_ASSERT((rt_uint8_t *)mem < (rt_uint8_t *)memheap->end);
    RT_ASSERT(!RT_MEMHEAP_IS_USED(mem));

    /* plug hole forward */
    nmem = mem->next;
    if (mem != nmem &&
            !RT_MEMHEAP_IS_USED(nmem) &&
            (rt_uint8_t *)nmem != (rt_uint8_t *)memheap->end)
    {
        /* if mem->next is unused and not end of heap_ptr,
         * combine mem and mem->next
         */
        if (memheap->free == nmem)
        {
            memheap->free = mem;
        }
        mem->next = nmem->next;
        nmem->next->prev = mem;

        /* adjust the available number of bytes. */
        memheap->available_size += RT_MEMHEAP_SIZE;
    }

    /* plug hole backward */
    pmem = mem->prev;
    if (pmem != mem && !RT_MEMHEAP_IS_USED(pmem))
    {
        /* if mem->prev is unused, combine mem and mem->prev */
        if (memheap->free == mem)
        {
            memheap->free = pmem;
        }
        pmem->next = mem->next;
        mem->next->prev = pmem;

        /* adjust the available number of bytes. */
        memheap->available_size += RT_MEMHEAP_SIZE;
    }
}

static void update_highest_used(struct rt_memheap *memheap)
{
    /* Find prev used block */
    while (!RT_MEMHEAP_IS_USED(memheap->last_used) && ((rt_uint8_t *)memheap->last_used != memheap->start))
    {
        memheap->last_used = memheap->last_used->prev;
    }
    if (((rt_uint8_t *)memheap->last_used == memheap->start) && !RT_MEMHEAP_IS_USED(memheap->last_used))
    {
        memheap->last_used = RT_NULL;
    }
}


static void init_heap_end(struct rt_memheap *memheap)
{
    memheap->end = (struct rt_memheap_item *)&memheap->start[memheap->mem_size_aligned + RT_MEMHEAP_SIZE];
    memheap->end->magic = RT_MEMHEAP_MAGIC | RT_MEMHEAP_USED;
    memheap->end->pool_ptr = memheap;
    memheap->end->next  = (struct rt_memheap_item *)memheap->end;
    memheap->end->prev  = (struct rt_memheap_item *)memheap->end;
#ifdef RT_USING_MEMTRACE
#ifdef RT_MEM_RECORD_THREAD_NAME
    rt_mem_setname(memheap->end, "INIT");
#endif  /* RT_MEM_RECORD_THREAD_NAME */
#endif
}

rt_uint32_t rt_memheap_used_size(struct rt_memheap *heap)
{
    rt_uint32_t used_size;

    struct rt_memheap_item *next;

    RT_ASSERT(heap);
    if (heap->last_used)
    {
        used_size = (rt_uint8_t *)(heap->last_used->next) - heap->start;
        next = heap->last_used->next;
        if ((rt_uint32_t)next < (rt_uint32_t)heap->end)
        {
            /* next should not be used */
            RT_ASSERT(!RT_MEMHEAP_IS_USED(next));
            /* including header of the first unused block */
            used_size += RT_MEMHEAP_SIZE;
        }
        else
        {
            /* next is heap_end, do nothing */
        }
    }
    else
    {
        /* including header of the first unused block */
        used_size = RT_MEMHEAP_SIZE;
    }

    return used_size;
}


/**
 * @ingroup SystemInit
 *
 * This function will initialize system heap memory.
 *
 * @param begin_addr the beginning address of system heap memory.
 * @param end_addr the end address of system heap memory.
 */
__ROM_USED rt_err_t rt_memheap_init(struct rt_memheap *memheap,
                                    const char        *name,
                                    void              *start_addr,
                                    rt_size_t         size)
{
    struct rt_memheap_item *mem;
    void *end_addr = (void *)((rt_uint32_t)start_addr + size);
    rt_uint32_t begin_align = RT_ALIGN((rt_uint32_t)start_addr, RT_ALIGN_SIZE);
    rt_uint32_t end_align = RT_ALIGN_DOWN((rt_uint32_t)end_addr, RT_ALIGN_SIZE);

    RT_DEBUG_NOT_IN_INTERRUPT;

    RT_ASSERT(memheap != RT_NULL);

    memheap->pool_size = end_align - begin_align;

    /* alignment addr */
    if ((end_align > (2 * RT_MEMHEAP_SIZE)) &&
            ((end_align - 2 * RT_MEMHEAP_SIZE) >= begin_align))
    {
        /* calculate the aligned memory size, tail RT_MEMHEAP_SIZE bytes are used by heap end block */
        memheap->mem_size_aligned = memheap->pool_size - 2 * RT_MEMHEAP_SIZE;
        memheap->available_size = memheap->mem_size_aligned;
        /* tail space is reserved */
        memheap->used_size = RT_MEMHEAP_SIZE;
        memheap->max_used_size = memheap->used_size;
        memheap->actual_used_size = 0;
        memheap->max_actual_used_size = 0;
    }
    else
    {
        rt_kprintf("mem init, error begin address 0x%x, and end address 0x%x\n",
                   (rt_uint32_t)start_addr, (rt_uint32_t)end_addr);

        return -RT_ERROR;;
    }

    /* initialize pool object */
    rt_object_init(&(memheap->parent), RT_Object_Class_MemHeap, name);

    /* point to begin address of heap */
    memheap->start     = (rt_uint8_t *)begin_align;
    memheap->start_addr = start_addr;

    memheap->linked_to_sys_mem = RT_FALSE;

    RT_DEBUG_LOG(RT_DEBUG_MEM, ("mem init, heap begin address 0x%x, size %d\n",
                                (rt_uint32_t)memheap->start, memheap->mem_size_aligned));

    /* initialize the start of the heap */
    mem        = (struct rt_memheap_item *)memheap->start;
    mem->magic = RT_MEMHEAP_MAGIC;
    mem->pool_ptr = memheap;
    mem->next  = (struct rt_memheap_item *)(memheap->start + memheap->mem_size_aligned + RT_MEMHEAP_SIZE);
    mem->prev  = mem;
#ifdef RT_USING_MEMTRACE
#ifdef RT_MEM_RECORD_THREAD_NAME
    rt_mem_setname(mem, "INIT");
#endif  /* RT_MEM_RECORD_THREAD_NAME */
#endif

    /* initialize the end of the heap */
    init_heap_end(memheap);

    rt_sem_init(&memheap->lock, name, 1, RT_IPC_FLAG_FIFO);

    /* initialize the lowest-free pointer to the start of the heap */
    memheap->free = (struct rt_memheap_item *)memheap->start;
    /* initialize the highest-used pointer to NULL as no block is allocated yet */
    memheap->last_used = RT_NULL;

    return RT_EOK;
}

__ROM_USED rt_err_t rt_memheap_detach(struct rt_memheap *heap)
{
    RT_ASSERT(heap);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);
    RT_ASSERT(rt_object_is_systemobject(&heap->parent));

    rt_object_detach(&(heap->lock.parent.parent));
    rt_object_detach(&(heap->parent));

    /* Return a successful completion. */
    return RT_EOK;
}
RTM_EXPORT(rt_memheap_detach);


/**
 * Allocate a block of memory with a minimum of 'size' bytes.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @return pointer to allocated memory or NULL if no free memory was found.
 */
__ROM_USED void *rt_memheap_alloc(struct rt_memheap *heap, rt_size_t size)

{
    struct rt_memheap_item *mem, *mem2;

    RT_ASSERT(heap);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    if (size == 0)
    {
        //RT_ASSERT(0);
        return RT_NULL;
    }

    RT_DEBUG_NOT_IN_INTERRUPT;

    if (size != RT_ALIGN(size, RT_ALIGN_SIZE))
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("malloc size %d, but align to %d\n",
                                    size, RT_ALIGN(size, RT_ALIGN_SIZE)));
    else
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("malloc size %d\n", size));

    /* alignment size */
    size = RT_ALIGN(size, RT_ALIGN_SIZE);

    if (size > heap->mem_size_aligned)
    {
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("no memory\n"));
        //RT_ASSERT(0);
        return RT_NULL;
    }

    /* every data block must be at least RT_MEMHEAP_MIN_SIZE_ALIGNED long */
    if (size < RT_MEMHEAP_MIN_SIZE_ALIGNED)
        size = RT_MEMHEAP_MIN_SIZE_ALIGNED;

    /* take memory semaphore */
    rt_sem_take(&heap->lock, RT_WAITING_FOREVER);

    //rt_kprintf("heap_ptr %x lfree %x - %x\n", lfree, heap_ptr, (rt_uint8_t *)lfree - heap_ptr);
    for (mem = heap->free;
            (rt_uint8_t *)mem < (heap->start + heap->mem_size_aligned - size);
            mem = mem->next)
    {
        RT_ASSERT((mem->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);

        if ((!RT_MEMHEAP_IS_USED(mem)) && (((rt_uint8_t *)(mem->next) - (rt_uint8_t *)mem - RT_MEMHEAP_SIZE) >= size))
        {
            /* mem is not used and at least perfect fit is possible:
             * mem->next - (ptr + RT_MEMHEAP_SIZE) gives us the 'user data size' of mem */

            if ((rt_uint8_t *)(mem->next) - ((rt_uint8_t *)mem + RT_MEMHEAP_SIZE) >=
                    (size + RT_MEMHEAP_SIZE + RT_MEMHEAP_MIN_SIZE_ALIGNED))
            {
                /* (in addition to the above, we test if another struct rt_memheap_item (RT_MEMHEAP_SIZE) containing
                 * at least RT_MEMHEAP_MIN_SIZE_ALIGNED of data also fits in the 'user data space' of 'mem')
                 * -> split large block, create empty remainder,
                 * remainder must be large enough to contain RT_MEMHEAP_MIN_SIZE_ALIGNED data: if
                 * mem->next - (ptr + (2*RT_MEMHEAP_SIZE)) == size,
                 * struct rt_memheap_item would fit in but no data between mem2 and mem2->next
                 * @todo we could leave out RT_MEMHEAP_MIN_SIZE_ALIGNED. We would create an empty
                 *       region that couldn't hold data, but when mem->next gets freed,
                 *       the 2 regions would be combined, resulting in more free memory
                 */

                mem2 = (struct rt_memheap_item *)((rt_uint8_t *)mem + RT_MEMHEAP_SIZE + size);

                /* create mem2 struct */
                mem2->magic = RT_MEMHEAP_MAGIC;
                mem2->pool_ptr = heap;
                mem2->next = mem->next;
                mem2->prev = mem;
#ifdef RT_USING_MEMTRACE
                /* record free block available */
                mem2->tick = rt_system_get_time();
#ifdef RT_MEM_RECORD_THREAD_NAME
                rt_mem_setname(mem2, "    ");
#endif  /* RT_MEM_RECORD_THREAD_NAME */
#endif
#ifdef MEM_ASYN_FREE
                mem2->ref_count_magic = REF_COUNT_MAGIC;
                mem2->ref_count = 0;
#endif
                /* and insert it between mem and mem->next */
                mem->next = mem2;

                if (mem2->next != heap->end)
                {
                    mem2->next->prev = mem2;
                }
                heap->used_size += (size + RT_MEMHEAP_SIZE);
                heap->available_size -= (size + RT_MEMHEAP_SIZE);
                if (heap->max_used_size < heap->used_size)
                    heap->max_used_size = heap->used_size;
            }
            else
            {
                /* (a mem2 struct does no fit into the user data space of mem and mem->next will always
                 * be used at this point: if not we have 2 unused structs in a row, plug_holes should have
                 * take care of this).
                 * -> near fit or excact fit: do not split, no mem2 creation
                 * also can't move mem->next directly behind mem, since mem->next
                 * will always be used at this point!
                 */
                heap->used_size += ((rt_uint8_t *) mem->next) - ((rt_uint8_t *)mem);
                heap->available_size -= RT_MEMHEAP_DATA_SIZE(heap, mem);
                if (heap->max_used_size < heap->used_size)
                    heap->max_used_size = heap->used_size;
            }
            heap->actual_used_size += size;
            if (heap->max_actual_used_size < heap->actual_used_size)
                heap->max_actual_used_size = heap->actual_used_size;

            /* set memory block magic */
            mem->magic |= RT_MEMHEAP_USED;
            mem->pool_ptr = heap;
            mem->size = size;
#ifdef RT_USING_MEMTRACE
            mem->tick = rt_system_get_time();
            mem->ret_addr = MEMHEAP_RET_ADDR;
#endif

#ifdef RT_USING_MEMTRACE
#ifdef RT_MEM_RECORD_THREAD_NAME
            if (rt_thread_self())
                rt_mem_setname(mem, rt_thread_self()->name);
            else
                rt_mem_setname(mem, "NONE");
#endif  /* RT_MEM_RECORD_THREAD_NAME */
#endif

            if (mem == heap->free)
            {
                /* Find next free block after mem and update lowest free pointer */
                while (RT_MEMHEAP_IS_USED(heap->free) && (heap->free != heap->end))
                    heap->free = heap->free->next;

                RT_ASSERT(((heap->free == heap->end) || (!RT_MEMHEAP_IS_USED(heap->free))));
            }

            if ((rt_uint32_t)mem > (rt_uint32_t)heap->last_used)
            {
                heap->last_used = mem;
            }

            rt_sem_release(&heap->lock);
            RT_ASSERT((rt_uint32_t)mem + RT_MEMHEAP_SIZE + size <= (rt_uint32_t)heap->end);
            RT_ASSERT((rt_uint32_t)((rt_uint8_t *)mem + RT_MEMHEAP_SIZE) % RT_ALIGN_SIZE == 0);
            RT_ASSERT((((rt_uint32_t)mem) & (RT_ALIGN_SIZE - 1)) == 0);

            RT_DEBUG_LOG(RT_DEBUG_MEM,
                         ("allocate memory at 0x%x, size: %d\n",
                          (rt_uint32_t)((rt_uint8_t *)mem + RT_MEMHEAP_SIZE),
                          (rt_uint32_t)((rt_uint8_t *)mem->next - (rt_uint8_t *)mem)));

            /* return the memory data except mem struct */
            return (rt_uint8_t *)mem + RT_MEMHEAP_SIZE;
        }
    }

    rt_sem_release(&heap->lock);

    return RT_NULL;
}
RTM_EXPORT(rt_memheap_alloc);

/**
 * This function will change the previously allocated memory block.
 *
 * @param rmem pointer to memory allocated by rt_malloc
 * @param newsize the required new size
 *
 * @return the changed memory block address
 */
__ROM_USED void *rt_memheap_realloc(struct rt_memheap *heap, void *rmem, rt_size_t newsize)
{
    rt_size_t size;
    struct rt_memheap_item *mem, *mem2;
    void *nmem;

    RT_ASSERT(heap);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* align allocated size */
    newsize = RT_ALIGN(newsize, RT_ALIGN_SIZE);
    if (newsize < RT_MEMHEAP_MIN_SIZE_ALIGNED)
        newsize = RT_MEMHEAP_MIN_SIZE_ALIGNED;

    if (newsize > heap->mem_size_aligned)
    {
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("realloc: out of memory\n"));

        return RT_NULL;
    }
    else if (newsize == 0)
    {
        rt_memheap_free(rmem);
        return RT_NULL;
    }

    /* allocate a new memory block */
    if (rmem == RT_NULL)
        return rt_memheap_alloc(heap, newsize);

    rt_sem_take(&heap->lock, RT_WAITING_FOREVER);

    if ((rt_uint8_t *)rmem < (rt_uint8_t *)heap->start ||
            (rt_uint8_t *)rmem >= (rt_uint8_t *)heap->end)
    {
        /* illegal memory */
        rt_sem_release(&heap->lock);

        return rmem;
    }

    mem = (struct rt_memheap_item *)((rt_uint8_t *)rmem - RT_MEMHEAP_SIZE);

    size = (rt_uint8_t *)(mem->next) - (rt_uint8_t *)mem - RT_MEMHEAP_SIZE;
    if (size == newsize)
    {
        /* the size is the same as */
        rt_sem_release(&heap->lock);

        return rmem;
    }

    if (newsize + RT_MEMHEAP_SIZE + RT_MEMHEAP_MIN_SIZE < size)
    {
        /* split memory block */
        heap->used_size -= (size - newsize);
        heap->available_size = heap->available_size + (size - newsize) - RT_MEMHEAP_SIZE;

        if (mem->size > newsize)
        {
            heap->actual_used_size -= (size - newsize);
        }
        else
        {
            heap->actual_used_size += (newsize - size);
        }

        mem2 = (struct rt_memheap_item *)((rt_uint8_t *)mem + RT_MEMHEAP_SIZE + newsize);
        mem2->magic = RT_MEMHEAP_MAGIC;
        mem2->pool_ptr = heap;
        mem2->next = mem->next;
        mem2->prev = mem;
#ifdef RT_USING_MEMTRACE
        /* record free block available */
        mem2->tick = rt_system_get_time();
#ifdef RT_MEM_RECORD_THREAD_NAME
        rt_mem_setname(mem2, "    ");
#endif  /* RT_MEM_RECORD_THREAD_NAME */
#endif
        mem->next = mem2;
        mem->size = newsize;

        if (mem2->next != heap->end)
        {
            mem2->next->prev = mem2;
        }

        /*rt_realloc may cause <lfree> change, if we don't change <lfree> timely, it may cause memory leakage*/
        if (mem2 < heap->free)
        {
            /* the splited struct is now the lowest */
            heap->free = mem2;
        }

        plug_holes(mem2);

        rt_sem_release(&heap->lock);

        return rmem;
    }
    rt_sem_release(&heap->lock);

    /* expand memory */
    nmem = rt_memheap_alloc(heap, newsize);
    if (nmem != RT_NULL) /* check memory */
    {
#ifdef MEM_ASYN_FREE
        ((struct rt_memheap_item *)((rt_uint8_t *)nmem - RT_MEMHEAP_SIZE))->ref_count_magic = mem->ref_count_magic;
        ((struct rt_memheap_item *)((rt_uint8_t *)nmem - RT_MEMHEAP_SIZE))->ref_count = mem->ref_count;
#endif
        rt_memcpy(nmem, rmem, size < newsize ? size : newsize);
        rt_memheap_free(rmem);
    }

    return nmem;
}
RTM_EXPORT(rt_memheap_realloc);

/**
 * This function will contiguously allocate enough space for count objects
 * that are size bytes of memory each and returns a pointer to the allocated
 * memory.
 *
 * The allocated memory is filled with bytes of value zero.
 *
 * @param count number of objects to allocate
 * @param size size of the objects to allocate
 *
 * @return pointer to allocated memory / NULL pointer if there is an error
 */
__ROM_USED void *rt_memheap_calloc(struct rt_memheap *heap, rt_size_t count, rt_size_t size)
{
    void *p;

    /* allocate 'count' objects of size 'size' */
    p = rt_memheap_alloc(heap, count * size);

    /* zero the memory */
    if (p)
        rt_memset(p, 0, count * size);

    return p;
}
RTM_EXPORT(rt_memheap_calloc);

/**
 * This function will release the previously allocated memory block by
 * rt_malloc. The released memory block is taken back to system heap.
 *
 * @param rmem the address of memory which will be released
 */
__ROM_USED void rt_memheap_free(void *rmem)
{
    struct rt_memheap_item *mem;
    struct rt_memheap *heap;

    if (rmem == RT_NULL)
        return;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* Get the corresponding struct rt_memheap_item ... */
    mem = (struct rt_memheap_item *)((rt_uint8_t *)rmem - RT_MEMHEAP_SIZE);
    heap = mem->pool_ptr;

    RT_ASSERT((((rt_uint32_t)rmem) & (RT_ALIGN_SIZE - 1)) == 0);
    RT_ASSERT((rt_uint8_t *)rmem >= (rt_uint8_t *)heap->start &&
              (rt_uint8_t *)rmem < (rt_uint8_t *)heap->end);

    if ((rt_uint8_t *)rmem < (rt_uint8_t *)heap->start ||
            (rt_uint8_t *)rmem >= (rt_uint8_t *)heap->end)
    {
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("illegal memory\n"));

        return;
    }

#ifdef MEM_ASYN_FREE
    if (mem->ref_count)
    {
        RT_ASSERT(REF_COUNT_MAGIC == mem->ref_count_magic);
        extern void app_mem_insert_asyn_node(void *ptr, void (*)(void *));
        app_mem_insert_asyn_node(rmem, rt_memheap_free);
        return;
    }
#endif

    RT_DEBUG_LOG(RT_DEBUG_MEM,
                 ("release memory 0x%x, size: %d\n",
                  (rt_uint32_t)rmem,
                  (rt_uint32_t)((rt_uint8_t *)(mem->next) - (rt_uint8_t *)mem)));

    /* protect the heap from concurrent access */
    rt_sem_take(&heap->lock, RT_WAITING_FOREVER);

    /* ... which has to be in a used state ... */
    if (mem->magic != (RT_MEMHEAP_MAGIC | RT_MEMHEAP_USED))
    {
        rt_kprintf("to free a bad data block:\n");
        rt_kprintf("mem: 0x%08x, magic code: 0x%08x\n", mem, mem->magic);
    }

    /* check magic */
    RT_ASSERT((mem->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);
    RT_ASSERT(mem->magic & RT_MEMHEAP_USED);
    /* check whether this block of memory has been over-written. */
    RT_ASSERT((mem->next->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);

    /* ... and is now unused. */
    mem->magic = RT_MEMHEAP_MAGIC;
#ifdef RT_USING_MEMTRACE
    /* record free time */
    mem->tick = rt_system_get_time();
#ifdef RT_MEM_RECORD_THREAD_NAME
    rt_mem_setname(mem, "    ");
#endif /* RT_MEM_RECORD_THREAD_NAME */
#endif

    if (mem < heap->free)
    {
        /* the newly freed struct is now the lowest */
        heap->free = mem;
    }

    if (mem == heap->last_used)
    {
        update_highest_used(heap);
    }

    heap->used_size -= ((rt_uint8_t *)(mem->next) - (rt_uint8_t *)mem);
    heap->available_size += RT_MEMHEAP_DATA_SIZE(heap, mem);
    heap->actual_used_size -= mem->size;

    /* finally, see if prev or next are free also */
    plug_holes(mem);
    rt_sem_release(&heap->lock);
}
RTM_EXPORT(rt_memheap_free);


static rt_uint32_t rt_memheap_backup_compress_copy(void *dst, const void *src, rt_ubase_t copy_size,
        rt_uint32_t max_size)
{
    rt_uint32_t actual_size;
    rt_memheap_default_backup_hdr_t *hdr;

    hdr = (rt_memheap_default_backup_hdr_t *)dst;
    if (copy_size + sizeof(*hdr) <= max_size)
    {
        hdr->src = (void *)src;
        hdr->size = copy_size;
        rt_memcpy((void *)(hdr + 1), (void *)src, copy_size);
        actual_size = copy_size + sizeof(*hdr);
    }
    else
    {
        actual_size = 0;
    }

    actual_size = RT_ALIGN(actual_size, RT_ALIGN_SIZE);

    if (actual_size > max_size)
    {
        actual_size = 0;
    }

    return actual_size;
}

static rt_uint32_t rt_memheap_backup_decompress_copy(void *dst, const void *src, rt_ubase_t copy_size, rt_ubase_t max_size, rt_compressor_cb_t decompressor_cb)
{
    rt_uint32_t actual_size;
    rt_memheap_default_backup_hdr_t *hdr;
    rt_uint32_t rd_size;

    if (!decompressor_cb)
    {
        actual_size = 0;
        hdr = (rt_memheap_default_backup_hdr_t *)src;
        while (copy_size > 0)
        {
            RT_ASSERT(hdr->size <= max_size);
            rt_memcpy(hdr->src, (void *)(hdr + 1), hdr->size);
            actual_size += hdr->size;

            rd_size = RT_ALIGN(hdr->size + sizeof(*hdr), RT_ALIGN_SIZE);
            /* get next block */
            hdr = (rt_memheap_default_backup_hdr_t *)((rt_uint32_t)hdr + rd_size);
            if (copy_size > rd_size)
            {
                copy_size -= rd_size;
            }
            else
            {
                copy_size = 0;
            }
        }
    }
    else
    {
        actual_size = decompressor_cb(dst, src, copy_size, max_size);
    }

    return actual_size;
}

static rt_uint32_t rt_memheap_backup_contiguous_used_block(struct rt_memheap *heap, rt_uint8_t **buf, struct rt_memheap_item *end_block, struct rt_memheap_item *start_block,
        rt_uint32_t *max_size)
{
    rt_uint32_t copy_size;
    rt_uint8_t *dst;
    rt_uint32_t total_copy_size;
    rt_uint32_t remaining_size;

    RT_ASSERT(heap);

    dst = *buf;
    total_copy_size = 0;
    remaining_size = *max_size;
    /* copy contiguous used block */
    RT_ASSERT(RT_MEMHEAP_IS_USED(start_block));
    RT_ASSERT((rt_uint32_t)end_block > (rt_uint32_t)start_block);
    if (end_block != heap->end)
    {
        RT_ASSERT(!RT_MEMHEAP_IS_USED(end_block));
    }

    copy_size = (rt_uint32_t)end_block - (rt_uint32_t)start_block;
    /* save data */
    copy_size = rt_memheap_backup_compress_copy((void *)dst, (void *)start_block, copy_size, remaining_size);

    copy_size = RT_ALIGN(copy_size, RT_ALIGN_SIZE);

    if ((0 == copy_size) || (copy_size > remaining_size))
    {
        return 0;
    }

    dst += copy_size;
    total_copy_size += copy_size;
    RT_ASSERT(remaining_size >= copy_size);
    remaining_size -= copy_size;

    *max_size = remaining_size;
    *buf = dst;
    return total_copy_size;
}

__ROM_USED rt_err_t rt_memheap_backup(struct rt_memheap *heap, rt_uint8_t *buf, rt_uint32_t max_size,
                                      rt_uint32_t *used_size)
{
    rt_uint32_t copy_size;
#ifdef RT_MEMHEAP_BACKUP_OPT
    rt_uint32_t total_used_size;
    struct rt_memheap_item *block;
    struct rt_memheap_item *start_block;
#endif /* RT_MEMHEAP_BACKUP_OPT */

    if (!heap || !buf || !used_size)
    {
        return RT_ERROR;
    }

#ifndef RT_MEMHEAP_BACKUP_OPT
    copy_size = rt_memheap_used_size(heap);

    *used_size = rt_memheap_backup_compress_copy((void *)buf, (void *)heap->start, copy_size,
                 max_size);
    if (0 == *used_size)
    {
        return RT_EFULL;
    }
#else
    total_used_size = 0;

    block = (struct rt_memheap_item *)heap->start;
    start_block = RT_NULL;
    while (block < heap->end)
    {
        if (RT_MEMHEAP_IS_USED(block))
        {
            if (!start_block)
            {
                start_block = block;
            }
        }
        else
        {
            if (start_block)
            {
                copy_size = rt_memheap_backup_contiguous_used_block(heap, &buf, block, start_block, &max_size);
                if (0 == copy_size)
                {
                    return RT_EFULL;

                }
                total_used_size += copy_size;
                start_block = RT_NULL;

            }

            /* copy uncompressed header of unused block */
            copy_size = rt_memheap_backup_compress_copy((void *)buf, block, RT_MEMHEAP_SIZE, max_size);
            if (0 == copy_size)
            {
                return RT_EFULL;
            }
            buf += copy_size;
            total_used_size += copy_size;
            RT_ASSERT(max_size >= copy_size);
            max_size -= copy_size;
        }
        block = block->next;
    }

    if (start_block)
    {
        copy_size = rt_memheap_backup_contiguous_used_block(heap, &buf, block, start_block, &max_size);
        if (0 == copy_size)
        {
            return RT_EFULL;
        }
        total_used_size += copy_size;
        start_block = RT_NULL;
    }

    *used_size = total_used_size;
#endif

    return RT_EOK;

}


__ROM_USED rt_err_t rt_memheap_restore(void *instance, rt_uint8_t *buf, rt_uint32_t size, rt_compressor_cb_t decompressor_cb)
{
    rt_uint32_t wr_size;
    rt_uint32_t max_size;
    struct rt_memheap *heap;
#ifdef RT_MEMHEAP_BACKUP_OPT
    rt_uint32_t copy_size;
    struct rt_memheap_item *wr;
    rt_uint32_t rd_size;
#endif /* RT_MEMHEAP_BACKUP_OPT */

    if (!instance || !buf)
    {
        return RT_ERROR;
    }

    heap = (struct rt_memheap *)instance;

    max_size = (rt_uint32_t)heap->end - (rt_uint32_t)heap->start;
    wr_size = rt_memheap_backup_decompress_copy((void *)heap->start, (void *)buf, size, max_size, decompressor_cb);
    RT_ASSERT(wr_size > 0);

    RT_ASSERT(((rt_uint32_t)heap->start + wr_size) <= (rt_uint32_t)heap->end);

    /* reinit heap end */
    init_heap_end(heap);

    return RT_EOK;
}

rt_err_t rt_memheap_dump(struct rt_memheap *heap, rt_mem_dump_cb_t dump_cb, void *user_data, rt_uint32_t *dump_size, rt_uint32_t *actual_size)
{
    rt_uint32_t copy_size;
    rt_uint32_t ret_size;
#ifdef RT_MEMHEAP_BACKUP_OPT
    struct rt_memheap_item *next;
    rt_uint32_t total_actual_size;
#endif /* RT_MEMHEAP_BACKUP_OPT */

    if (!heap || !dump_cb)
    {
        return RT_ERROR;
    }

#ifndef RT_MEMHEAP_BACKUP_OPT
    copy_size = rt_memheap_used_size(heap);
    ret_size = dump_cb((void *)heap->start_addr, copy_size, user_data, actual_size);
    if (dump_size)
    {
        *dump_size = ret_size;
    }
    if (ret_size < copy_size)
    {
        return RT_EFULL;
    }
#else
    next = (struct rt_memheap_item *)heap->start;
    total_actual_size = 0;
    if (dump_size)
    {
        *dump_size = 0;
    }
    while (next < heap->end)
    {
        if (RT_MEMHEAP_IS_USED(next))
        {
            /* copy header and data */
            copy_size = (rt_uint32_t)next->next - (rt_uint32_t)next;
        }
        else
        {
            /* only copy header */
            copy_size = RT_MEMHEAP_SIZE;
        }

        ret_size = dump_cb((rt_uint32_t)next, copy_size, user_data, actual_size);
        if (actual_size)
        {
            total_actual_size += *actual_size;
            *actual_size = total_actual_size;
        }
        if (dump_size)
        {
            *dump_size += ret_size;
        }
        if (ret_size < copy_size)
        {
            return RT_EFULL;
        }
        next = next->next;
    }

#endif

    return RT_EOK;
}



#ifndef SOC_BF0_LCPU
rt_size_t rt_heapmem_size(void *rmem)
{
    struct rt_memheap_item *mem;
    struct rt_memheap *heap;

    if (rmem == RT_NULL)
        return 0;

    /* Get the corresponding struct rt_memheap_item ... */
    mem = (struct rt_memheap_item *)((rt_uint8_t *)rmem - RT_MEMHEAP_SIZE);
    heap = mem->pool_ptr;

    RT_ASSERT((((rt_uint32_t)rmem) & (RT_ALIGN_SIZE - 1)) == 0);
    RT_ASSERT((rt_uint8_t *)rmem >= (rt_uint8_t *)heap->start &&
              (rt_uint8_t *)rmem < (rt_uint8_t *)heap->end);

    if ((rt_uint8_t *)rmem < (rt_uint8_t *)heap->start ||
            (rt_uint8_t *)rmem >= (rt_uint8_t *)heap->end)
    {
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("illegal memory\n"));

        return 0;
    }

    RT_ASSERT(mem->magic == (RT_MEMHEAP_MAGIC | RT_MEMHEAP_USED));
    RT_ASSERT((mem->next->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);

    return mem->size;
}
#endif

#ifdef RT_USING_MEMHEAP_AS_HEAP
    #error "RT_USING_MEMHEAP2 defined!!! RT_USING_MEMHEAP_AS_HEAP dose not support!!!"
#endif

#endif /* end of RT_USING_MEMHEAP2 */

