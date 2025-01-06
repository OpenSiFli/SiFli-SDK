/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * File      : memheap.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-04-10     Bernard      first implementation
 * 2012-10-16     Bernard      add the mutex lock for heap object.
 * 2012-12-29     Bernard      memheap can be used as system heap.
 *                             change mutex lock to semaphore lock.
 * 2013-04-10     Bernard      add rt_memheap_realloc function.
 * 2013-05-24     Bernard      fix the rt_memheap_realloc issue.
 * 2013-07-11     Grissiom     fix the memory block splitting issue.
 * 2013-07-15     Grissiom     optimize rt_memheap_realloc
 */

#include <rthw.h>
#include <rtthread.h>

#ifdef RT_USING_MEMHEAP

#define RT_MEMHEAP_BACKUP_OPT

/* dynamic pool magic and mask */
#define RT_MEMHEAP_MAGIC        0x1ea01ea0
#define RT_MEMHEAP_MASK         0xfffffffe
#define RT_MEMHEAP_USED         0x01
#define RT_MEMHEAP_FREED        0x00

#define RT_MEMHEAP_IS_USED(i)   ((i)->magic & RT_MEMHEAP_USED)
#define RT_MEMHEAP_MINIALLOC    12

#define RT_MEMHEAP_SIZE         RT_ALIGN(sizeof(struct rt_memheap_item), RT_ALIGN_SIZE)
#define MEMITEM_SIZE(item)      ((rt_uint32_t)item->next - (rt_uint32_t)item - RT_MEMHEAP_SIZE)
#define RT_MEMHEAP_TAILER(memheap)   ((struct rt_memheap_item *)((rt_uint32_t)((memheap)->start_addr) + (memheap)->pool_size - RT_MEMHEAP_SIZE))

#ifdef _MSC_VER
    #define MEMHEAP_RET_ADDR  (rt_uint32_t) _ReturnAddress()
#else
    #define MEMHEAP_RET_ADDR  (rt_uint32_t) __builtin_return_address(0)
#endif

static void update_last_used(struct rt_memheap *heap)
{
    struct rt_memheap_item *item;

    if (!heap->last_used)
    {
        return;
    }

    item = heap->last_used;
    /* Find prev used block */
    while (!RT_MEMHEAP_IS_USED(item)
            && (item != heap->block_list))
    {
        item = item->prev;
    }
    if ((item == heap->block_list)
            && !RT_MEMHEAP_IS_USED(item))
    {
        heap->last_used = RT_NULL;
    }
    else
    {
        heap->last_used = item;
    }
}

static void init_tailer(struct rt_memheap *memheap,
                        struct rt_memheap_item *tailer)
{
    RT_ASSERT(tailer);

    /* it's a used memory block */
    tailer->magic     = RT_MEMHEAP_MAGIC | RT_MEMHEAP_USED;
    tailer->pool_ptr  = memheap;
    tailer->next      = (struct rt_memheap_item *)memheap->start_addr;
    tailer->prev      = (struct rt_memheap_item *)memheap->start_addr;
    /* not in free list */
    tailer->next_free = tailer->prev_free = RT_NULL;
}

static void insert_free_header(struct rt_memheap *memheap, struct rt_memheap_item *new_header)
{
    struct rt_memheap_item *pos;
    pos = memheap->free_list->next_free;
    while (pos != memheap->free_list)
    {
        if ((rt_uint32_t)pos > (rt_uint32_t)new_header)
        {
            break;
        }
        pos = pos->next_free;
    }
    new_header->next_free = pos;
    new_header->prev_free = pos->prev_free;
    pos->prev_free->next_free = new_header;
    pos->prev_free = new_header;

#ifdef RT_USING_MEMTRACE
    new_header->tick = rt_system_get_time();
#endif /* RT_USING_MEMTRACE */
}

static rt_uint32_t rt_memheap_used_size(struct rt_memheap *heap)
{
    rt_uint32_t used_size;
    if (heap->last_used)
    {
        used_size = (rt_uint32_t)(heap->last_used->next) - (rt_uint32_t)(heap->start_addr);
        if (heap->last_used->next < RT_MEMHEAP_TAILER(heap))
        {
            /* next should not be used */
            RT_ASSERT(!RT_MEMHEAP_IS_USED(heap->last_used->next));
            /* including header of the first unused block */
            used_size += RT_MEMHEAP_SIZE;
        }
        else
        {
            /* next is end_block, do nothing */
        }
    }
    else
    {
        /* including header of the first unused block */
        used_size = RT_MEMHEAP_SIZE;
    }

    return used_size;
}

/*
 * The initialized memory pool will be:
 * +-----------------------------------+--------------------------+
 * | whole freed memory block          | Used Memory Block Tailer |
 * +-----------------------------------+--------------------------+
 *
 * block_list --> whole freed memory block
 *
 * The length of Used Memory Block Tailer is 0,
 * which is prevents block merging across list
 */
__ROM_USED rt_err_t rt_memheap_init(struct rt_memheap *memheap,
                                    const char        *name,
                                    void              *start_addr,
                                    rt_size_t         size)
{
    struct rt_memheap_item *item;

    RT_ASSERT(memheap != RT_NULL);

    /* initialize pool object */
    rt_object_init(&(memheap->parent), RT_Object_Class_MemHeap, name);

    memheap->start_addr     = start_addr;
    memheap->pool_size      = RT_ALIGN_DOWN(size, RT_ALIGN_SIZE);
    memheap->available_size = memheap->pool_size - (2 * RT_MEMHEAP_SIZE);
    memheap->max_used_size  = memheap->pool_size - memheap->available_size;
    memheap->actual_used_size = 0;
    memheap->max_actual_used_size = 0;
    memheap->linked_to_sys_mem = RT_FALSE;
    memheap->last_used = RT_NULL;

    /* initialize the free list header */
    item            = &(memheap->free_header);
    item->magic     = RT_MEMHEAP_MAGIC;
    item->pool_ptr  = memheap;
    item->next      = RT_NULL;
    item->prev      = RT_NULL;
    item->next_free = item;
    item->prev_free = item;

    /* set the free list to free list header */
    memheap->free_list = item;

    /* initialize the first big memory block */
    item            = (struct rt_memheap_item *)start_addr;
    item->magic     = RT_MEMHEAP_MAGIC;
    item->pool_ptr  = memheap;
    item->next      = RT_NULL;
    item->prev      = RT_NULL;
    item->next_free = item;
    item->prev_free = item;

    item->next = (struct rt_memheap_item *)
                 ((rt_uint8_t *)item + memheap->available_size + RT_MEMHEAP_SIZE);
    item->prev = item->next;

    /* block list header */
    memheap->block_list = item;

    /* place the big memory block to free list */
    item->next_free = memheap->free_list->next_free;
    item->prev_free = memheap->free_list;
    memheap->free_list->next_free->prev_free = item;
    memheap->free_list->next_free            = item;

    /* move to the end of memory pool to build a small tailer block,
     * which prevents block merging
     */
    item = item->next;
    init_tailer(memheap, item);

    /* initialize semaphore lock */
    rt_sem_init(&(memheap->lock), name, 1, RT_IPC_FLAG_FIFO);

    RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                 ("memory heap: start addr 0x%08x, size %d, free list header 0x%08x\n",
                  start_addr, size, &(memheap->free_header)));

    return RT_EOK;
}
RTM_EXPORT(rt_memheap_init);

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


#if 0//def RT_USING_MEMTRACE
rt_inline void rt_memheap_setname(struct rt_memheap_item *mem, const char *name)
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
#endif

__ROM_USED void *rt_memheap_alloc(struct rt_memheap *heap, rt_size_t size)
{
    rt_err_t result;
    rt_uint32_t free_size;
    struct rt_memheap_item *header_ptr;

    RT_ASSERT(heap != RT_NULL);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    /* align allocated size */
    size = RT_ALIGN(size, RT_ALIGN_SIZE);
    if (size < RT_MEMHEAP_MINIALLOC)
        size = RT_MEMHEAP_MINIALLOC;

    RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("allocate %d on heap:%8.*s",
                                    size, RT_NAME_MAX, heap->parent.name));

    if (size < heap->available_size)
    {
        /* search on free list */
        free_size = 0;

        /* lock memheap */
        result = rt_sem_take(&(heap->lock), RT_WAITING_FOREVER);
        if (result != RT_EOK)
        {
            rt_set_errno(result);

            return RT_NULL;
        }

        /* get the first free memory block */
        header_ptr = heap->free_list->next_free;
        while (header_ptr != heap->free_list && free_size < size)
        {
            /* get current freed memory block size */
            free_size = MEMITEM_SIZE(header_ptr);
            if (free_size < size)
            {
                /* move to next free memory block */
                header_ptr = header_ptr->next_free;
            }
        }

        /* determine if the memory is available. */
        if (free_size >= size)
        {
            /* a block that satisfies the request has been found. */

            /* determine if the block needs to be split. */
            if (free_size >= (size + RT_MEMHEAP_SIZE + RT_MEMHEAP_MINIALLOC))
            {
                struct rt_memheap_item *new_ptr;

                /* split the block. */
                new_ptr = (struct rt_memheap_item *)
                          (((rt_uint8_t *)header_ptr) + size + RT_MEMHEAP_SIZE);

                RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                             ("split: block[0x%08x] nextm[0x%08x] prevm[0x%08x] to new[0x%08x]\n",
                              header_ptr,
                              header_ptr->next,
                              header_ptr->prev,
                              new_ptr));

                /* mark the new block as a memory block and freed. */
                new_ptr->magic = RT_MEMHEAP_MAGIC;

                /* put the pool pointer into the new block. */
                new_ptr->pool_ptr = heap;

                /* break down the block list */
                new_ptr->prev          = header_ptr;
                new_ptr->next          = header_ptr->next;
                header_ptr->next->prev = new_ptr;
                header_ptr->next       = new_ptr;

                /* remove header ptr from free list */
                header_ptr->next_free->prev_free = header_ptr->prev_free;
                header_ptr->prev_free->next_free = header_ptr->next_free;
                header_ptr->next_free = RT_NULL;
                header_ptr->prev_free = RT_NULL;

                /* insert new_ptr to free list */
#if 0
                new_ptr->next_free = heap->free_list->next_free;
                new_ptr->prev_free = heap->free_list;
                heap->free_list->next_free->prev_free = new_ptr;
                heap->free_list->next_free            = new_ptr;
#else
                insert_free_header(heap, new_ptr);
#endif
                RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("new ptr: next_free 0x%08x, prev_free 0x%08x\n",
                                                new_ptr->next_free,
                                                new_ptr->prev_free));

                /* decrement the available byte count.  */
                heap->available_size = heap->available_size -
                                       size -
                                       RT_MEMHEAP_SIZE;
                if (heap->pool_size - heap->available_size > heap->max_used_size)
                    heap->max_used_size = heap->pool_size - heap->available_size;
            }
            else
            {
                /* decrement the entire free size from the available bytes count. */
                heap->available_size = heap->available_size - free_size;
                if (heap->pool_size - heap->available_size > heap->max_used_size)
                    heap->max_used_size = heap->pool_size - heap->available_size;

                /* remove header_ptr from free list */
                RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                             ("one block: block[0x%08x], next_free 0x%08x, prev_free 0x%08x\n",
                              header_ptr,
                              header_ptr->next_free,
                              header_ptr->prev_free));

                header_ptr->next_free->prev_free = header_ptr->prev_free;
                header_ptr->prev_free->next_free = header_ptr->next_free;
                header_ptr->next_free = RT_NULL;
                header_ptr->prev_free = RT_NULL;
            }

            heap->actual_used_size += size;
            if (heap->max_actual_used_size < heap->actual_used_size)
            {
                heap->max_actual_used_size = heap->actual_used_size;
            }
            /* Mark the allocated block as not available. */
            header_ptr->magic |= RT_MEMHEAP_USED;
            header_ptr->size = size;
#ifdef RT_USING_MEMTRACE
            header_ptr->tick = rt_system_get_time();
            header_ptr->ret_addr = MEMHEAP_RET_ADDR;
            //header_ptr->dbg_magic = RT_MEMHEAP_MAGIC;

            //if (rt_thread_self())
            //    rt_memheap_setname(header_ptr, rt_thread_self()->name);
            //else
            //    rt_memheap_setname(header_ptr, "NONE");

#endif

            if (header_ptr > heap->last_used)
            {
                heap->last_used = header_ptr;
            }

            /* release lock */
            rt_sem_release(&(heap->lock));

            /* Return a memory address to the caller.  */
            RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                         ("alloc mem: memory[0x%08x], heap[0x%08x], size: %d\n",
                          (void *)((rt_uint8_t *)header_ptr + RT_MEMHEAP_SIZE),
                          header_ptr,
                          size));

            return (void *)((rt_uint8_t *)header_ptr + RT_MEMHEAP_SIZE);
        }

        /* release lock */
        rt_sem_release(&(heap->lock));
    }

    RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("allocate memory: failed\n"));

    /* Return the completion status.  */
    return RT_NULL;
}
RTM_EXPORT(rt_memheap_alloc);

__ROM_USED void *rt_memheap_realloc(struct rt_memheap *heap, void *ptr, rt_size_t newsize)
{
    rt_err_t result;
    rt_size_t oldsize;
    struct rt_memheap_item *header_ptr;
    struct rt_memheap_item *new_ptr;

    RT_ASSERT(heap);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    if (newsize == 0)
    {
        rt_memheap_free(ptr);

        return RT_NULL;
    }
    /* align allocated size */
    newsize = RT_ALIGN(newsize, RT_ALIGN_SIZE);
    if (newsize < RT_MEMHEAP_MINIALLOC)
        newsize = RT_MEMHEAP_MINIALLOC;

    if (ptr == RT_NULL)
    {
        ptr = rt_memheap_alloc(heap, newsize);

#ifdef RT_USING_MEMTRACE
        if (ptr)
        {
            header_ptr    = (struct rt_memheap_item *)((rt_uint8_t *)ptr - RT_MEMHEAP_SIZE);
            /* update caller address */
            header_ptr->ret_addr = MEMHEAP_RET_ADDR;
        }
#endif
        return ptr;
    }

    /* get memory block header and get the size of memory block */
    header_ptr = (struct rt_memheap_item *)
                 ((rt_uint8_t *)ptr - RT_MEMHEAP_SIZE);
    oldsize = MEMITEM_SIZE(header_ptr);
    /* re-allocate memory */
    if (newsize > oldsize)
    {
        void *new_ptr;
        volatile struct rt_memheap_item *next_ptr;

        /* lock memheap */
        result = rt_sem_take(&(heap->lock), RT_WAITING_FOREVER);
        if (result != RT_EOK)
        {
            rt_set_errno(result);
            return RT_NULL;
        }

        next_ptr = header_ptr->next;

        /* header_ptr should not be the tail */
        RT_ASSERT(next_ptr > header_ptr);

        /* check whether the following free space is enough to expand */
        if (!RT_MEMHEAP_IS_USED(next_ptr))
        {
            rt_int32_t nextsize;

            nextsize = MEMITEM_SIZE(next_ptr);
            RT_ASSERT(next_ptr > 0);

            /* Here is the ASCII art of the situation that we can make use of
             * the next free node without alloc/memcpy, |*| is the control
             * block:
             *
             *      oldsize           free node
             * |*|-----------|*|----------------------|*|
             *         newsize          >= minialloc
             * |*|----------------|*|-----------------|*|
             */
            if (nextsize + oldsize > newsize + RT_MEMHEAP_MINIALLOC)
            {
                /* decrement the entire free size from the available bytes count. */
                heap->available_size = heap->available_size - (newsize - oldsize);
                if (heap->pool_size - heap->available_size > heap->max_used_size)
                    heap->max_used_size = heap->pool_size - heap->available_size;

                heap->actual_used_size += newsize - header_ptr->size;
                if (heap->max_actual_used_size < heap->actual_used_size)
                {
                    heap->max_actual_used_size = heap->actual_used_size;
                }

                /* remove next_ptr from free list */
                RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                             ("remove block: block[0x%08x], next_free 0x%08x, prev_free 0x%08x",
                              next_ptr,
                              next_ptr->next_free,
                              next_ptr->prev_free));

                next_ptr->next_free->prev_free = next_ptr->prev_free;
                next_ptr->prev_free->next_free = next_ptr->next_free;
                next_ptr->next->prev = next_ptr->prev;
                next_ptr->prev->next = next_ptr->next;

                /* build a new one on the right place */
                next_ptr = (struct rt_memheap_item *)((char *)ptr + newsize);

                RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                             ("new free block: block[0x%08x] nextm[0x%08x] prevm[0x%08x]",
                              next_ptr,
                              next_ptr->next,
                              next_ptr->prev));

                /* mark the new block as a memory block and freed. */
                next_ptr->magic = RT_MEMHEAP_MAGIC;

                /* put the pool pointer into the new block. */
                next_ptr->pool_ptr = heap;

                next_ptr->prev          = header_ptr;
                next_ptr->next          = header_ptr->next;
                header_ptr->next->prev = (struct rt_memheap_item *)next_ptr;
                header_ptr->next       = (struct rt_memheap_item *)next_ptr;
                /* update size */
                header_ptr->size = newsize;

#ifdef RT_USING_MEMTRACE
                header_ptr->ret_addr = MEMHEAP_RET_ADDR;
#endif
                /* insert next_ptr to free list */
#if 0
                next_ptr->next_free = heap->free_list->next_free;
                next_ptr->prev_free = heap->free_list;
                heap->free_list->next_free->prev_free = (struct rt_memheap_item *)next_ptr;
                heap->free_list->next_free            = (struct rt_memheap_item *)next_ptr;
#else
                insert_free_header(heap, (struct rt_memheap_item *)next_ptr);
#endif
                RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("new ptr: next_free 0x%08x, prev_free 0x%08x",
                                                next_ptr->next_free,
                                                next_ptr->prev_free));

                /* release lock */
                rt_sem_release(&(heap->lock));

                return ptr;
            }
        }

        /* release lock */
        rt_sem_release(&(heap->lock));

        /* re-allocate a memory block */
        new_ptr = (void *)rt_memheap_alloc(heap, newsize);
        if (new_ptr != RT_NULL)
        {
            rt_memcpy(new_ptr, ptr, oldsize < newsize ? oldsize : newsize);
            rt_memheap_free(ptr);

#ifdef RT_USING_MEMTRACE
            header_ptr = (struct rt_memheap_item *)((rt_uint8_t *)new_ptr - RT_MEMHEAP_SIZE);
            /* update caller address */
            header_ptr->ret_addr = MEMHEAP_RET_ADDR;
#endif
        }

        return new_ptr;
    }

    /* don't split when there is less than one node space left */
    if (newsize + RT_MEMHEAP_SIZE + RT_MEMHEAP_MINIALLOC >= oldsize)
    {
        /* update used size */
        if (header_ptr->size > newsize)
        {
            heap->actual_used_size -= header_ptr->size - newsize;
        }
        else
        {
            heap->actual_used_size += newsize - header_ptr->size;
        }
        if (heap->max_actual_used_size < heap->actual_used_size)
        {
            heap->max_actual_used_size = heap->actual_used_size;
        }

        /* update size */
        header_ptr->size = newsize;
#ifdef RT_USING_MEMTRACE
        /* update caller address */
        header_ptr->ret_addr = MEMHEAP_RET_ADDR;
#endif

        return ptr;
    }

    /* lock memheap */
    result = rt_sem_take(&(heap->lock), RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        rt_set_errno(result);

        return RT_NULL;
    }

    /* split the block. */
    new_ptr = (struct rt_memheap_item *)
              (((rt_uint8_t *)header_ptr) + newsize + RT_MEMHEAP_SIZE);

    RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                 ("split: block[0x%08x] nextm[0x%08x] prevm[0x%08x] to new[0x%08x]\n",
                  header_ptr,
                  header_ptr->next,
                  header_ptr->prev,
                  new_ptr));

    /* mark the new block as a memory block and freed. */
    new_ptr->magic = RT_MEMHEAP_MAGIC;
    /* put the pool pointer into the new block. */
    new_ptr->pool_ptr = heap;

    /* break down the block list */
    new_ptr->prev          = header_ptr;
    new_ptr->next          = header_ptr->next;
    header_ptr->next->prev = new_ptr;
    header_ptr->next       = new_ptr;

    /* update used size */
    if (header_ptr->size > newsize)
    {
        heap->actual_used_size -= header_ptr->size - newsize;
    }
    else
    {
        heap->actual_used_size += newsize - header_ptr->size;
    }
    if (heap->max_actual_used_size < heap->actual_used_size)
    {
        heap->max_actual_used_size = heap->actual_used_size;
    }
    /* update size */
    header_ptr->size = newsize;
#ifdef RT_USING_MEMTRACE
    /* update caller address */
    header_ptr->ret_addr = MEMHEAP_RET_ADDR;
#endif

    /* determine if the block can be merged with the next neighbor. */
    if (!RT_MEMHEAP_IS_USED(new_ptr->next))
    {
        struct rt_memheap_item *free_ptr;

        /* merge block with next neighbor. */
        free_ptr = new_ptr->next;
        heap->available_size = heap->available_size - MEMITEM_SIZE(free_ptr);

        RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                     ("merge: right node 0x%08x, next_free 0x%08x, prev_free 0x%08x\n",
                      header_ptr, header_ptr->next_free, header_ptr->prev_free));

        free_ptr->next->prev = new_ptr;
        new_ptr->next   = free_ptr->next;

        /* remove free ptr from free list */
        free_ptr->next_free->prev_free = free_ptr->prev_free;
        free_ptr->prev_free->next_free = free_ptr->next_free;
    }

    /* insert the split block to free list */
#if 0
    new_ptr->next_free = heap->free_list->next_free;
    new_ptr->prev_free = heap->free_list;
    heap->free_list->next_free->prev_free = new_ptr;
    heap->free_list->next_free            = new_ptr;
#else
    insert_free_header(heap, new_ptr);
#endif
    RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("new free ptr: next_free 0x%08x, prev_free 0x%08x\n",
                                    new_ptr->next_free,
                                    new_ptr->prev_free));

    /* increment the available byte count.  */
    heap->available_size = heap->available_size + MEMITEM_SIZE(new_ptr);

    /* release lock */
    rt_sem_release(&(heap->lock));

    /* return the old memory block */
    return ptr;
}
RTM_EXPORT(rt_memheap_realloc);

__ROM_USED void rt_memheap_free(void *ptr)
{
    rt_err_t result;
    struct rt_memheap *heap;
    struct rt_memheap_item *header_ptr, *new_ptr;
    rt_uint32_t insert_header;

    /* NULL check */
    if (ptr == RT_NULL) return;

    /* set initial status as OK */
    insert_header = 1;
    new_ptr       = RT_NULL;
    header_ptr    = (struct rt_memheap_item *)
                    ((rt_uint8_t *)ptr - RT_MEMHEAP_SIZE);

    RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("free memory: memory[0x%08x], block[0x%08x]\n",
                                    ptr, header_ptr));

    /* check magic */
    RT_ASSERT((header_ptr->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);
    RT_ASSERT(header_ptr->magic & RT_MEMHEAP_USED);
    /* check whether this block of memory has been over-written. */
    RT_ASSERT((header_ptr->next->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);

    /* get pool ptr */
    heap = header_ptr->pool_ptr;

    RT_ASSERT(heap);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    /* lock memheap */
    result = rt_sem_take(&(heap->lock), RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        rt_set_errno(result);

        return ;
    }

    /* Mark the memory as available. */
    header_ptr->magic &= ~RT_MEMHEAP_USED;
    /* Adjust the available number of bytes. */
    heap->available_size = heap->available_size + MEMITEM_SIZE(header_ptr);

    /* update used size */
    heap->actual_used_size -= header_ptr->size;

    if (heap->last_used == header_ptr)
    {
        update_last_used(heap);
    }

    /* Determine if the block can be merged with the previous neighbor. */
    if (!RT_MEMHEAP_IS_USED(header_ptr->prev))
    {
        RT_DEBUG_LOG(RT_DEBUG_MEMHEAP, ("merge: left node 0x%08x\n",
                                        header_ptr->prev));

        /* adjust the available number of bytes. */
        heap->available_size = heap->available_size + RT_MEMHEAP_SIZE;

        /* yes, merge block with previous neighbor. */
        (header_ptr->prev)->next = header_ptr->next;
        (header_ptr->next)->prev = header_ptr->prev;

        /* move header pointer to previous. */
        header_ptr = header_ptr->prev;
        /* don't insert header to free list */
        insert_header = 0;
    }

    /* determine if the block can be merged with the next neighbor. */
    if (!RT_MEMHEAP_IS_USED(header_ptr->next))
    {
        /* adjust the available number of bytes. */
        heap->available_size = heap->available_size + RT_MEMHEAP_SIZE;

        /* merge block with next neighbor. */
        new_ptr = header_ptr->next;

        RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                     ("merge: right node 0x%08x, next_free 0x%08x, prev_free 0x%08x\n",
                      new_ptr, new_ptr->next_free, new_ptr->prev_free));

        new_ptr->next->prev = header_ptr;
        header_ptr->next    = new_ptr->next;

        /* remove new ptr from free list */
        new_ptr->next_free->prev_free = new_ptr->prev_free;
        new_ptr->prev_free->next_free = new_ptr->next_free;
    }

    if (insert_header)
    {
#if 0
        header_ptr->next_free = heap->free_list->next_free;
        header_ptr->prev_free = heap->free_list;
        heap->free_list->next_free->prev_free = header_ptr;
        heap->free_list->next_free            = header_ptr;
#else
        insert_free_header(heap, header_ptr);
#endif

        RT_DEBUG_LOG(RT_DEBUG_MEMHEAP,
                     ("insert to free list: next_free 0x%08x, prev_free 0x%08x\n",
                      header_ptr->next_free, header_ptr->prev_free));
    }

    /* release lock */
    rt_sem_release(&(heap->lock));
}
RTM_EXPORT(rt_memheap_free);

__ROM_USED void *rt_memheap_calloc(struct rt_memheap *heap, rt_size_t count, rt_size_t size)
{
    void *ptr;
    rt_size_t total_size;
#ifdef RT_USING_MEMTRACE
    struct rt_memheap_item *header_ptr;
#endif

    total_size = count * size;
    ptr = rt_memheap_alloc(heap, total_size);
    if (ptr != NULL)
    {
        /* clean memory */
        rt_memset(ptr, 0, total_size);

#ifdef RT_USING_MEMTRACE
        header_ptr    = (struct rt_memheap_item *)((rt_uint8_t *)ptr - RT_MEMHEAP_SIZE);
        header_ptr->ret_addr = MEMHEAP_RET_ADDR;
#endif
    }

    return ptr;
}
RTM_EXPORT(rt_memheap_calloc);


#if 0
void rt_memheap_used(struct rt_memheap *heap, rt_uint32_t *start_addr, rt_uint32_t *size)
{
    if (start_addr && size)
    {
        *start_addr = (rt_uint32_t)heap->start_addr;
        if (heap->last_used)
        {
            *size = (rt_uint32_t)(heap->last_used->next) - (rt_uint32_t)(heap->start_addr);
            if (heap->last_used->next > heap->last_used)
            {
                /* next should not be used */
                RT_ASSERT(!RT_MEMHEAP_IS_USED(heap->last_used->next))
            }
        }
        else
        {
            *size = 0;
        }
    }
}
#endif


__ROM_USED rt_err_t rt_memheap_backup(struct rt_memheap *heap, rt_uint8_t *buf,
                                      rt_uint32_t max_size, rt_uint32_t *used_size)
{
#ifdef RT_MEMHEAP_BACKUP_OPT
    rt_uint32_t total_used_size;
    rt_uint32_t copy_size;
    struct rt_memheap_item *next;
#endif /* RT_MEMHEAP_BACKUP_OPT */

    if (!heap || !buf || !used_size)
    {
        return RT_ERROR;
    }

#ifndef RT_MEMHEAP_BACKUP_OPT
    *used_size = rt_memheap_used_size(heap);

    if (*used_size > max_size)
    {
        return RT_EFULL;
    }

    rt_memcpy((void *)buf, (void *)heap->start_addr, *used_size);
#else
    total_used_size = 0;

    next = heap->block_list;

    while (next < RT_MEMHEAP_TAILER(heap))
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
        if ((total_used_size + copy_size) > max_size)
        {
            return RT_EFULL;
        }
        rt_memcpy((void *)buf, (void *)next, copy_size);

        total_used_size += copy_size;
        buf += copy_size;
        next = next->next;
    }

    *used_size = total_used_size;

#endif

    return RT_EOK;
}


__ROM_USED rt_err_t rt_memheap_restore(void *instance, rt_uint8_t *buf, rt_uint32_t size, rt_compressor_cb_t decompressor_cb)
{
    struct rt_memheap *heap;
#ifdef RT_MEMHEAP_BACKUP_OPT
    struct rt_memheap_item *rd;
    rt_uint32_t copy_size;
    struct rt_memheap_item *wr;
    rt_uint32_t rd_size;
#endif /* RT_MEMHEAP_BACKUP_OPT */

    if (!buf || !instance)
    {
        return RT_ERROR;
    }

    heap = (struct rt_memheap *)instance;

#ifndef RT_MEMHEAP_BACKUP_OPT
    rt_memcpy((void *)(heap->start_addr), (void *)buf, size);
#else

    wr = (struct rt_memheap_item *)heap->start_addr;
    rd_size = 0;
    while (rd_size < size)
    {
        rd = (struct rt_memheap_item *)buf;
        if (RT_MEMHEAP_IS_USED(rd))
        {
            /* copy header and data */
            copy_size = (rt_uint32_t)rd->next - (rt_uint32_t)wr;
        }
        else
        {
            /* only copy header */
            copy_size = RT_MEMHEAP_SIZE;
        }

        if (((rt_uint8_t *)wr + copy_size) > (rt_uint8_t *)RT_MEMHEAP_TAILER(heap))
        {
            return RT_ERROR;
        }

        rt_memcpy((void *)wr, (void *)buf, copy_size);

        rd_size += copy_size;
        buf += copy_size;
        wr = wr->next;
    }
#endif

    /* reinit heap tailer */
    init_tailer(heap, RT_MEMHEAP_TAILER(heap));

    return RT_EOK;
}

#ifndef SOC_BF0_LCPU
rt_size_t rt_heapmem_size(void *rmem)
{
    struct rt_memheap_item *header_ptr;

    /* NULL check */
    if (rmem == RT_NULL) return 0;

    header_ptr    = (struct rt_memheap_item *)
                    ((rt_uint8_t *)rmem - RT_MEMHEAP_SIZE);

    /* check magic */
    RT_ASSERT((header_ptr->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);
    RT_ASSERT(header_ptr->magic & RT_MEMHEAP_USED);
    /* check whether this block of memory has been over-written. */
    RT_ASSERT((header_ptr->next->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);

    return header_ptr->size;
}
#endif

#ifdef RT_USING_MEMHEAP_AS_HEAP
static struct rt_memheap _heap;
/* Not used actullay, defined for compatibility, such that T32 script can run when mem.c is not used */
RT_USED static rt_uint8_t *heap_ptr;

__ROM_USED void rt_system_heap_init(void *begin_addr, void *end_addr)
{
    /* initialize a default heap in the system */
    rt_memheap_init(&_heap,
                    "heap",
                    begin_addr,
                    (rt_uint32_t)end_addr - (rt_uint32_t)begin_addr);
}

__ROM_USED void *rt_malloc(rt_size_t size)
{
    void *ptr;
#ifdef RT_USING_MEMTRACE
    struct rt_memheap_item *header_ptr;
#endif

    /* try to allocate in system heap */
    ptr = rt_memheap_alloc(&_heap, size);
    if (ptr == RT_NULL)
    {
        struct rt_object *object;
        struct rt_list_node *node;
        struct rt_memheap *heap;
        struct rt_object_information *information;

        /* try to allocate on other memory heap */
        information = rt_object_get_information(RT_Object_Class_MemHeap);
        RT_ASSERT(information != RT_NULL);
        for (node  = information->object_list.next;
                node != &(information->object_list);
                node  = node->next)
        {
            object = rt_list_entry(node, struct rt_object, list);
            heap   = (struct rt_memheap *)object;

            RT_ASSERT(heap);
            RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

            if (!heap->linked_to_sys_mem)
            {
                continue;
            }

            /* not allocate in the default system heap */
            if (heap == &_heap)
                continue;

            ptr = rt_memheap_alloc(heap, size);
            if (ptr != RT_NULL)
                break;
        }
    }
#ifdef RT_USING_MEMTRACE
    if (ptr)
    {
        header_ptr    = (struct rt_memheap_item *)((rt_uint8_t *)ptr - RT_MEMHEAP_SIZE);
        header_ptr->ret_addr = MEMHEAP_RET_ADDR;
    }
#endif

    return ptr;
}
RTM_EXPORT(rt_malloc);

__ROM_USED void rt_free(void *rmem)
{
    rt_memheap_free(rmem);
}
RTM_EXPORT(rt_free);

__ROM_USED void *rt_realloc(void *rmem, rt_size_t newsize)
{
    void *new_ptr;
    struct rt_memheap_item *header_ptr;

    if (rmem == RT_NULL)
        return rt_malloc(newsize);

    if (newsize == 0)
    {
        rt_free(rmem);
        return RT_NULL;
    }

    /* get old memory item */
    header_ptr = (struct rt_memheap_item *)
                 ((rt_uint8_t *)rmem - RT_MEMHEAP_SIZE);

    new_ptr = rt_memheap_realloc(header_ptr->pool_ptr, rmem, newsize);
    if (new_ptr == RT_NULL && newsize != 0)
    {
        /* allocate memory block from other memheap */
        new_ptr = rt_malloc(newsize);
        if (new_ptr != RT_NULL && rmem != RT_NULL)
        {
            rt_size_t oldsize;

            /* get the size of old memory block */
            oldsize = MEMITEM_SIZE(header_ptr);
            if (newsize > oldsize)
                rt_memcpy(new_ptr, rmem, oldsize);
            else
                rt_memcpy(new_ptr, rmem, newsize);

            rt_free(rmem);
        }
    }

    return new_ptr;
}
RTM_EXPORT(rt_realloc);

__ROM_USED void *rt_calloc(rt_size_t count, rt_size_t size)
{
    void *ptr;
    rt_size_t total_size;

    total_size = count * size;
    ptr = rt_malloc(total_size);
    if (ptr != RT_NULL)
    {
        /* clean memory */
        rt_memset(ptr, 0, total_size);
    }

    return ptr;
}
RTM_EXPORT(rt_calloc);


rt_size_t rt_mem_size(void *rmem)
{
    struct rt_memheap_item *header_ptr;

    /* NULL check */
    if (rmem == RT_NULL) return 0;

    header_ptr    = (struct rt_memheap_item *)
                    ((rt_uint8_t *)rmem - RT_MEMHEAP_SIZE);

    /* check magic */
    RT_ASSERT((header_ptr->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);
    RT_ASSERT(header_ptr->magic & RT_MEMHEAP_USED);
    /* check whether this block of memory has been over-written. */
    RT_ASSERT((header_ptr->next->magic & RT_MEMHEAP_MASK) == RT_MEMHEAP_MAGIC);

    return header_ptr->size;
}
RTM_EXPORT(rt_mem_size);


__ROM_USED void rt_memory_info(rt_uint32_t *total,
                               rt_uint32_t *used,
                               rt_uint32_t *max_used)
{
    if (total != RT_NULL)
        *total = _heap.pool_size;
    if (used  != RT_NULL)
        *used = _heap.pool_size - _heap.available_size;
    if (max_used != RT_NULL)
        *max_used = _heap.max_used_size;
}

rt_err_t rt_memheap_add_to_sys(struct rt_memheap *heap)
{
    RT_ASSERT(heap != RT_NULL);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    heap->linked_to_sys_mem = RT_TRUE;

    return RT_EOK;
}

rt_err_t rt_memheap_remove_from_sys(struct rt_memheap *heap)
{
    RT_ASSERT(heap != RT_NULL);
    RT_ASSERT(rt_object_get_type(&heap->parent) == RT_Object_Class_MemHeap);

    heap->linked_to_sys_mem = RT_FALSE;

    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

__ROM_USED void list_mem(void)
{
    rt_kprintf("total memory: %d ", _heap.pool_size);
    rt_kprintf("used memory : %d ", _heap.pool_size - _heap.available_size);
    rt_kprintf("maximum allocated memory: %d\n", _heap.max_used_size);
    rt_kprintf("actual used memory : %d ", _heap.actual_used_size);
    rt_kprintf("maximum actual allocated memory: %d\n", _heap.max_actual_used_size);
}
MSH_CMD_EXPORT(list_mem, list memory usage information)

#endif

rt_uint32_t used_sram_size(void)
{
    return _heap.actual_used_size;
}
#endif

#endif
