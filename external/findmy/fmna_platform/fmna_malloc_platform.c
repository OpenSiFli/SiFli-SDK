/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#include <stdlib.h>
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "fmna_malloc_platform.h"

#define LOG_TAG "fmna_malloc"
#include "log.h"

#define FMNA_MALLOC_POOL_MAX_POOL 7
#define FMNA_MALLOC_POOL_0 32
#define FMNA_MALLOC_POOL_1 64
#define FMNA_MALLOC_POOL_2 128
#define FMNA_MALLOC_POOL_3 256
#define FMNA_MALLOC_POOL_4 512
#define FMNA_MALLOC_POOL_5 1056
#define FMNA_MALLOC_POOL_6 2048

uint16_t pool_size[] =
{
    FMNA_MALLOC_POOL_0,
    FMNA_MALLOC_POOL_1,
    FMNA_MALLOC_POOL_2,
    FMNA_MALLOC_POOL_3,
    FMNA_MALLOC_POOL_4,
    FMNA_MALLOC_POOL_5,
    FMNA_MALLOC_POOL_6
};

uint32_t m_malloc_pool_count[FMNA_MALLOC_POOL_MAX_POOL] = {0};
uint32_t m_malloc_pool_total_count[FMNA_MALLOC_POOL_MAX_POOL] = {0};
uint32_t m_malloc_pool_max_count[FMNA_MALLOC_POOL_MAX_POOL] = {0};

uint32_t fmna_malloc_size_to_pool(uint32_t size)
{
    if (size > FMNA_MALLOC_POOL_5)
    {
        LOG_I("FMNA large malloc: %d", size);
        return 6;
    }
    else if (size > FMNA_MALLOC_POOL_4)
    {
        return 5;
    }
    else if (size > FMNA_MALLOC_POOL_3)
    {
        return 4;
    }
    else if (size > FMNA_MALLOC_POOL_2)
    {
        return 3;
    }
    else if (size > FMNA_MALLOC_POOL_1)
    {
        return 2;
    }
    else if (size > FMNA_MALLOC_POOL_0)
    {
        return 1;
    }
    return 0;

}
void fmna_free(void *ptr)
{
    uint32_t pool = *(uint32_t *)(ptr - 4);
    m_malloc_pool_count[pool]--;
    rt_free((ptr - 4));
}

void *fmna_malloc(size_t size)
{
    uint16_t pool = fmna_malloc_size_to_pool(size);
    void *pBuf =  rt_malloc(size + 4);
    *(uint32_t *)pBuf = pool;
    m_malloc_pool_count[pool]++;
    if (m_malloc_pool_count[pool] > m_malloc_pool_max_count[pool])
    {
        m_malloc_pool_max_count[pool] = m_malloc_pool_count[pool] ;
    }
    m_malloc_pool_total_count[pool]++;
    return (void *)((pBuf + 4));
}

void *fmna_realloc(void *ptr, size_t size)
{
    void *pBuf = ptr;
    uint32_t new_pool = fmna_malloc_size_to_pool(size);
    uint32_t pool = *(uint32_t *)(ptr - 4);
    if (new_pool > pool)
    {
        m_malloc_pool_count[pool]--;
        m_malloc_pool_count[new_pool]++;
        m_malloc_pool_total_count[new_pool]++;
        if (m_malloc_pool_count[new_pool] > m_malloc_pool_max_count[new_pool])
        {
            m_malloc_pool_max_count[new_pool] = m_malloc_pool_count[new_pool] ;
        }
        pBuf = rt_realloc((pBuf - 4), (size + 4));
        *(uint32_t *)pBuf = new_pool;
        return (void *)((pBuf + 4));
    }
    return pBuf;
}

void fmna_malloc_dump(void)
{
#ifdef USE_FMNA_DEBUG_MALLOC
    for (int i = 0; i < FMNA_MALLOC_POOL_MAX_POOL; ++i)
    {
        LOG_I("FNMA Malloc Pool %d (%d): %x %x %x", i, pool_size[i], m_malloc_pool_count[i], m_malloc_pool_max_count[i], m_malloc_pool_total_count[i]);
    }
#endif
}
