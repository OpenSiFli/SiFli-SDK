/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include "board.h"
#include "audio_mem.h"

#if AUDIO_MEMORY_LEAK_CHECK

#define AUDIO_MEMORY_OVERFLOW_CHECK     1

#define AUDIO_MEM_END   0xDD

typedef struct
{
    struct rt_list_node     node;
    const char              *filename;
    int                     line;
    uint32_t                size;
    uint8_t                 malloc_mem[0];
} audio_mem_node;

static struct rt_mutex     g_mutex;
static struct rt_list_node g_root;
static int g_is_init;

void *audio_mem_malloc_do(uint32_t size, const char *file, int line)
{
    audio_mem_node *node = NULL;
#if AUDIO_MEMORY_OVERFLOW_CHECK
    node = (audio_mem_node *)rt_malloc(sizeof(audio_mem_node) + size + 2);
#else
    node = (audio_mem_node *)rt_malloc(sizeof(audio_mem_node) + size);
#endif

    if (!node)
    {
        rt_kprintf("av_malloc_do: %s line %d size=%d\r\n", file, line, size);
        RT_ASSERT(0);
    }
    else
    {
        node->filename = file;
        node->line = line;
        node->size = size;
#if AUDIO_MEMORY_OVERFLOW_CHECK
        node->malloc_mem[size] = AUDIO_MEM_END;
        node->malloc_mem[size + 1] = AUDIO_MEM_END;
#endif
        rt_mutex_take(&g_mutex, RT_WAITING_FOREVER);
        rt_list_insert_before(&g_root, &node->node);
        rt_mutex_release(&g_mutex);
    }
    //rt_kprintf("%s[%d]: node=0x%p m=0x%p s=%d end=[%02x %02x]\r\n", node->filename, node->line, node, node->malloc_mem, size, node->malloc_mem[size], node->malloc_mem[size+1]);

    return (void *)node->malloc_mem;
}
void audio_mem_free_do(void *ptr)
{
    audio_mem_node *node;
    if (ptr)
    {
        node = rt_container_of(ptr, audio_mem_node, malloc_mem);

        rt_mutex_take(&g_mutex, RT_WAITING_FOREVER);
        rt_list_remove(&node->node);
        rt_mutex_release(&g_mutex);
#if AUDIO_MEMORY_OVERFLOW_CHECK
        //rt_kprintf("%s[%d]: node=0x%p m=0x%p s=%d end=[%02x %02x]\r\n", node->filename, node->line, node, node->malloc_mem, node->size, node->malloc_mem[node->size], node->malloc_mem[node->size+1]);
        if (node->malloc_mem[node->size] != AUDIO_MEM_END || node->malloc_mem[node->size + 1] != AUDIO_MEM_END)
        {
            rt_kprintf("audio mem overflow: %s %d\r\n", node->filename, node->line);
            RT_ASSERT(0);
        }
#endif
        rt_free(node);
    }
}
void *audio_mem_calloc_do(uint32_t count, uint32_t size, const char *file, int line)
{
    void  *p = audio_mem_malloc_do(count * size, file, line);
    if (p)
    {
        rt_memset(p, 0, count * size);
    }
    return p;
}
void *audio_mem_realloc_do(void *mem_address, unsigned int newsize, const char *file, int line)
{
    void *ptr = NULL;
    if (!mem_address)
    {
        if (newsize == 0)
        {
            newsize++;
        }
        ptr = audio_mem_malloc_do(newsize, file, line);
    }
    else if (newsize == 0)
    {
        audio_mem_free_do(mem_address);
    }
    else
    {
        ptr = audio_mem_malloc_do(newsize, file, line);
        if (ptr)
        {
            memcpy(ptr, mem_address, newsize);
        }
        else
        {
            audio_mem_free_do(mem_address);
        }
    }
    return ptr;
}
int audio_mem_init(void)
{
    rt_list_init(&g_root);
    rt_mutex_init(&g_mutex, "audio_mem", RT_IPC_FLAG_FIFO);
    g_is_init = 1;
    return 0;
}

INIT_DEVICE_EXPORT(audio_mem_init);

#ifdef RT_USING_FINSH
int audio_mem(void)
{
    rt_list_t *pos;
    audio_mem_node *node;
    if (rt_list_isempty(&g_root))
    {
        rt_kprintf("audio mem no leak\r\n");
        return 0;
    }

    rt_kprintf("audio mem leak:\r\n");
    rt_mutex_take(&g_mutex, RT_WAITING_FOREVER);
    rt_list_for_each(pos, &g_root)
    {
        node = rt_list_entry(pos, audio_mem_node, node);
        rt_kprintf("%s line %d size=%d\r\n", node->filename, node->line, node->size);
    }
    rt_mutex_release(&g_mutex);
    return 0;
}

FINSH_FUNCTION_EXPORT(audio_mem,       audio mem check);
MSH_CMD_EXPORT(audio_mem,     audio mem check);
#endif

#elif !defined(_WIN32)
RT_WEAK void *audio_mem_malloc(uint32_t size)
{
    void *ptr = rt_malloc(size);
    RT_ASSERT(ptr);
    return ptr;
}
RT_WEAK void audio_mem_free(void *ptr)
{
    rt_free(ptr);
}
RT_WEAK void *audio_mem_calloc(uint32_t count, uint32_t size)
{
    return rt_calloc(count, size);
}
RT_WEAK void *audio_mem_realloc_do(void *mem_address, unsigned int newsize)
{
    void *ptr = NULL;
#if 1
    ptr = rt_realloc(mem_address, newsize);
#else
    if (!mem_address)
    {
        //never come here for audio module
        if (newsize == 0)
            newsize++;

        ptr = rt_malloc(newsize);
    }
    else if (newsize == 0)
    {
        //audio not use this function now, newsize always > 0
        rt_free(mem_address);
    }
    else
    {
        ptr = rt_malloc(newsize);
        if (ptr)
        {
            memcpy(ptr, mem_address, newsize); //include rubish data if newsize big than old size
        }
        else
        {
            rt_free(mem_address);
        }
    }
#endif
    return ptr;
}
#endif
