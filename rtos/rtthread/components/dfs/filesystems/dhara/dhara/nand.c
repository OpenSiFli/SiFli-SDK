#include "rtthread.h"
#include <rtdevice.h>
#include "nand.h"
#include <stdbool.h>
#include "log.h"
#include "map.h"


/**NOTE: BSP_USING_BBM (global bad block management) should be enabled if RT_DHARA_BBM_ENABLED is not defined */

//#define DHARA_LRU_CACHE_ENABLED
//#define DHARA_LRU2_CACHE_ENABLED

#define DHARA_META_CACHE_SIZE   (128)

#ifdef DHARA_LRU_CACHE_ENABLED
    #define DHARA_CHECK_CACHE_IDX(idx)    RT_ASSERT(((idx) >= 0) && ((idx) < DHARA_META_CACHE_SIZE))
    #define DHARA_GET_CACHE(idx)          (&(dhara_meta_cache[(idx)]))

#elif defined(DHARA_LRU2_CACHE_ENABLED)
    #define DHARA_CHECK_CACHE_IDX(idx)    RT_ASSERT(((idx) > 0) && ((idx) <= DHARA_META_CACHE_SIZE))
    #define DHARA_GET_CACHE(idx)          (&(dhara_meta_cache[(idx) - 1]))
    /* must be 2^n */
    #define DHARA_META_CACHE_LU_TBL_SIZE  (DHARA_META_CACHE_SIZE / 2)
    #define DHARA_CACHE_HASH(p, offset)  (((p)+(offset)) & (DHARA_META_CACHE_LU_TBL_SIZE - 1))
#endif /* DHARA_LRU_CACHE_ENABLED  */



typedef struct
{
    const struct dhara_nand *n;
    dhara_page_t p;
    int32_t offset;
#ifdef DHARA_LRU_CACHE_ENABLED
    //dhara_meta_cache_list_t node;
    uint16_t prev;
    uint16_t next;
#elif defined(DHARA_LRU2_CACHE_ENABLED)
    int32_t life;
    /** prev node, 0: first node, use [prev-1] to get cache node */
    uint16_t prev;
    /** next node, 0: tail node, use []next-1] to get cache node */
    uint16_t next;
#endif /* DHARA_LRU_CACHE_ENABLED */
    uint8_t data[DHARA_META_SIZE];
} dhara_meta_cache_t;


static uint32_t meta_cache_index;
static dhara_meta_cache_t dhara_meta_cache[DHARA_META_CACHE_SIZE];
static struct rt_mutex dhara_meta_cache_lock;
static uint32_t dhara_meta_access_cnt;
static uint32_t dhara_meta_cache_miss_cnt;
#if defined(DHARA_LRU_CACHE_ENABLED)
    static uint16_t dhara_meta_cache_head;
#elif defined(DHARA_LRU2_CACHE_ENABLED)
    /** lookup table, 0: empty list, use [value-1] to get first cache node */
    static uint16_t dhara_meta_cache_lookup_tbl[DHARA_META_CACHE_LU_TBL_SIZE];
    static uint16_t eldest_cache_idx;
#endif /* DHARA_LRU_CACHE_ENABLED */


static inline void dhara_lock_cache(void)
{
    rt_err_t err;

    err = rt_mutex_take(&dhara_meta_cache_lock, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
}

static inline void dhara_unlock_cache(void)
{
    rt_err_t err;

    err = rt_mutex_release(&dhara_meta_cache_lock);
    RT_ASSERT(RT_EOK == err);
}


// public function definitions
int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b)
{
    struct rt_mtd_nand_device *dev;
    rt_err_t err;
    int is_bad;

    return 0;

    dev = (struct rt_mtd_nand_device *)n->user_data;

    err = rt_mtd_nand_check_block(dev, b);
    if (RT_EOK != err)
    {
        is_bad = 1;
    }
    else
    {
        is_bad = 0;
    }
#ifndef RT_DHARA_BBM_ENABLED
    RT_ASSERT(!is_bad);
#endif /* RT_DHARA_BBM_ENABLED */

    return is_bad;
}

void dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b)
{
    struct rt_mtd_nand_device *dev;

    dev = (struct rt_mtd_nand_device *)n->user_data;

#ifndef RT_DHARA_BBM_ENABLED
    RT_ASSERT(0);
#endif /* RT_DHARA_BBM_ENABLED */

    rt_mtd_nand_mark_badblock(dev, b);
}

int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b, dhara_error_t *err)
{
    struct rt_mtd_nand_device *dev;
    rt_err_t rt_err;

    dev = (struct rt_mtd_nand_device *)n->user_data;

    //rt_kprintf("erase:%d\n", b);

    rt_err = rt_mtd_nand_erase_block(dev, b);

    if (RT_EOK == rt_err)
    {
        // success
        return 0;
    }
    else
    {
        RT_ASSERT(0);
        if (err)
        {
            *err = DHARA_E_BAD_BLOCK;
        }
        return -1;
    }
}

static inline void dhara_del_cache(const struct dhara_nand *n, dhara_page_t p)
{
    dhara_meta_cache_t *cache;
#ifdef DHARA_LRU_CACHE_ENABLED
    RT_ASSERT(0);
#elif defined(DHARA_LRU2_CACHE_ENABLED)
    RT_ASSERT(0);
#else
    uint32_t i;

    cache = &dhara_meta_cache[0];
    for (i = 0; i < DHARA_META_CACHE_SIZE; i++, cache++)
    {
        if ((n == cache->n)
                && (-1 != cache->offset)
                && (cache->p == p))
        {
            /* mark as invalid */
            cache->offset = -2;
        }
    }
#endif
}

int dhara_nand_prog(const struct dhara_nand *n, dhara_page_t p, const uint8_t *data,
                    dhara_error_t *err)
{
    struct rt_mtd_nand_device *dev;
    rt_err_t rt_err;

    dev = (struct rt_mtd_nand_device *)n->user_data;

    //rt_kprintf("prog:%d\n", p);

    //LOG_HEX("prog", 8, (uint8_t *)data, 2048);

    if (15 == (p & 15))
    {
        /* invalidate meta cache */
        dhara_del_cache(n, p);
    }

    rt_err = rt_mtd_nand_write(dev, p, data, 1 << n->log2_page_size, NULL, 0);

    if (RT_EOK == rt_err)
    {
        // success
        return 0;
    }
    else
    {
#ifndef RT_DHARA_BBM_ENABLED
        RT_ASSERT(0);
#endif /* RT_DHARA_BBM_ENABLED  */
        if (err)
        {
            *err = DHARA_E_BAD_BLOCK;
        }
        return -1;
    }

}

int dhara_nand_is_free(const struct dhara_nand *n, dhara_page_t p)
{
    struct rt_mtd_nand_device *dev;
    rt_err_t err;
    rt_uint32_t *buf;
    int is_free;
    uint32_t page_size;

    dev = (struct rt_mtd_nand_device *)n->user_data;

    //rt_kprintf("is_free:%d\n", p);

    page_size = 1 << n->log2_page_size;
    buf = rt_malloc(page_size);
    RT_ASSERT(buf);

    err = rt_mtd_nand_read(dev, p, (rt_uint8_t *)buf, page_size, NULL, 0);

    if (err == RT_EOK)
    {
        is_free = true;
        for (rt_uint32_t i = 0; i < page_size / 4; i++)
        {
            if (buf[i] != RT_UINT32_MAX)
            {
                is_free = false;
                break;
            }
        }

    }
    else
    {
        is_free = 0;
    }

    rt_free(buf);

    return is_free;
}

static inline void dhara_update_cache_life(void)
{
#ifdef DHARA_LRU2_CACHE_ENABLED
    dhara_meta_cache_t *eldest_cache = RT_NULL;
    dhara_meta_cache_t *cache = RT_NULL;
    uint32_t i;

    eldest_cache = &dhara_meta_cache[0];
    eldest_cache_idx = 1;
    if (eldest_cache->life > INT32_MIN)
    {
        eldest_cache->life--;
    }
    cache = &dhara_meta_cache[1];
    for (i = 1; i < DHARA_META_CACHE_SIZE; i++, cache++)
    {
        if (cache->life > INT32_MIN)
        {
            cache->life--;
        }
        if (cache->life < eldest_cache->life)
        {
            eldest_cache = cache;
            eldest_cache_idx = i + 1;
        }
    }
#endif
}

#ifdef DHARA_LRU_CACHE_ENABLED
static inline void dhara_remove_cache(dhara_meta_cache_t *cache)
{
    dhara_meta_cache_t *neigh_cache;
    uint16_t self_idx;

    DHARA_CHECK_CACHE_IDX(cache->prev);
    neigh_cache = DHARA_GET_CACHE(cache->prev);
    self_idx = neigh_cache->next;
    if (dhara_meta_cache_head == self_idx)
    {
        dhara_meta_cache_head = cache->next;
    }
    neigh_cache->next = cache->next;
    DHARA_CHECK_CACHE_IDX(cache->next);
    neigh_cache = DHARA_GET_CACHE(cache->next);
    neigh_cache->prev = cache->prev;

    cache->next = self_idx;
    cache->prev = self_idx;
}

static inline void dhara_enqueue_cache(dhara_meta_cache_t *cache)
{
    dhara_meta_cache_t *head;
    dhara_meta_cache_t *tail;
    uint16_t self_idx;

    RT_ASSERT(cache->prev == cache->next);
    self_idx = cache->prev;

    DHARA_CHECK_CACHE_IDX(dhara_meta_cache_head);
    head = DHARA_GET_CACHE(dhara_meta_cache_head);
    DHARA_CHECK_CACHE_IDX(head->prev);
    tail = DHARA_GET_CACHE(head->prev);

    cache->prev = head->prev;
    cache->next = dhara_meta_cache_head;
    head->prev = self_idx;
    tail->next = self_idx;
    dhara_meta_cache_head = self_idx;

    //rt_kprintf("eq:%d,%d,%d\n", dhara_meta_cache_head, cache->prev, cache->next);
}

static inline dhara_meta_cache_t *dhara_dequeue_cache(void)
{
    dhara_meta_cache_t *head;
    dhara_meta_cache_t *tail;

    DHARA_CHECK_CACHE_IDX(dhara_meta_cache_head);
    head = DHARA_GET_CACHE(dhara_meta_cache_head);
    DHARA_CHECK_CACHE_IDX(head->prev);
    tail = DHARA_GET_CACHE(head->prev);
    //rt_kprintf("dq:%d,%d\n", dhara_meta_cache_head, head->prev);

    dhara_remove_cache(tail);

    //rt_kprintf("dq2:%d,%d,%d\n", dhara_meta_cache_head, tail->prev, tail->next);

    return tail;
}
#endif /* DHARA_LRU_CACHE_ENABLED */

static inline dhara_meta_cache_t *dhara_find_cache(const struct dhara_nand *n, dhara_page_t p, size_t offset)
{
#ifdef DHARA_LRU_CACHE_ENABLED
    dhara_meta_cache_t *cache = RT_NULL;
    uint16_t idx;

    dhara_meta_access_cnt++;

    DHARA_CHECK_CACHE_IDX(dhara_meta_cache_head);
    cache = DHARA_GET_CACHE(dhara_meta_cache_head);
    while (cache->offset >= 0)
    {
        if ((n == cache->n)
                && (cache->offset == offset)
                && (cache->p == p))
        {
            break;
        }

        if (cache->next == dhara_meta_cache_head)
        {
            /* no match */
            cache = RT_NULL;
            break;

        }
        else
        {
            DHARA_CHECK_CACHE_IDX(cache->next);
            cache = DHARA_GET_CACHE(cache->next);
        }
    }

    if (cache && (cache->offset >= 0))
    {
        /* cache hit, move to the head */
        dhara_remove_cache(cache);
        dhara_enqueue_cache(cache);
    }
    else
    {
        /* no match */
        cache = RT_NULL;
        dhara_meta_cache_miss_cnt++;
    }

    return cache;

#elif defined(DHARA_LRU2_CACHE_ENABLED)
    uint32_t i;
    uint32_t hash;
    dhara_meta_cache_t *cache = RT_NULL;
    uint16_t idx;

    dhara_meta_access_cnt++;

    hash = DHARA_CACHE_HASH(p, offset);
    RT_ASSERT(hash < DHARA_META_CACHE_LU_TBL_SIZE);

    idx = dhara_meta_cache_lookup_tbl[hash];
    if (0 == idx)
    {
        goto __EXIT;
    }

    DHARA_CHECK_CACHE_IDX(idx);
    cache = DHARA_GET_CACHE(idx);
    while (cache)
    {
        if ((n == cache->n)
                && (cache->offset == offset)
                && (cache->p == p))
        {
            cache->life = 0;
            break;
        }

        if (cache->next)
        {
            DHARA_CHECK_CACHE_IDX(cache->next);
            cache = DHARA_GET_CACHE(cache->next);
        }
        else
        {
            cache = RT_NULL;
        }
    }


__EXIT:
    if (!cache)
    {
        dhara_meta_cache_miss_cnt++;
    }
    return cache;

#else
    bool cache_hit;
    dhara_meta_cache_t *cache = RT_NULL;
    uint32_t i;

    dhara_meta_access_cnt++;
    cache_hit = false;
    cache = &dhara_meta_cache[0];
    for (i = 0; i < DHARA_META_CACHE_SIZE; i++, cache++)
    {
        if ((n == cache->n)
                && (cache->offset == offset)
                && (cache->p == p))
        {
            //rt_kprintf("read hit:%d,%d,%d\n", p, offset, length);
            cache_hit = true;
            break;
        }
        else if (-1 == cache->offset)
        {
            break;
        }
    }

    if (!cache_hit)
    {
        cache = RT_NULL;
        dhara_meta_cache_miss_cnt++;
    }
    return cache;
#endif
}

static inline void dhara_add_cache(const struct dhara_nand *n, dhara_page_t p,
                                   size_t offset, uint8_t *data)
{
    dhara_meta_cache_t *cache;
#ifdef DHARA_LRU_CACHE_ENABLED

    cache = dhara_dequeue_cache();
    dhara_enqueue_cache(cache);

    //rt_kprintf("replace:%d,%d,%d,%d\n", cache->p, cache->offset, p, offset);

    cache->n = n;
    cache->p = p;
    cache->offset = offset;
    rt_memcpy((void *)&cache->data[0], (void *)data, DHARA_META_SIZE);

#elif defined(DHARA_LRU2_CACHE_ENABLED)
    uint32_t hash;
    uint32_t i;

    dhara_meta_cache_t *neigh_cache;

    DHARA_CHECK_CACHE_IDX(eldest_cache_idx);
    cache = DHARA_GET_CACHE(eldest_cache_idx);

    /* remove node from link list and update header */
    if (cache->prev)
    {
        DHARA_CHECK_CACHE_IDX(cache->prev);
        neigh_cache = DHARA_GET_CACHE(cache->prev);
        neigh_cache->next = cache->next;
    }
    else if (cache->offset >= 0)
    {
        hash = DHARA_CACHE_HASH(cache->p, cache->offset);
        RT_ASSERT(hash < DHARA_META_CACHE_LU_TBL_SIZE);
        dhara_meta_cache_lookup_tbl[hash] = cache->next;
    }

    if (cache->next)
    {
        DHARA_CHECK_CACHE_IDX(cache->next);
        neigh_cache = DHARA_GET_CACHE(cache->next);
        neigh_cache->prev = cache->prev;
    }

    /* add node into the new link list */
    hash = DHARA_CACHE_HASH(p, offset);
    RT_ASSERT(hash < DHARA_META_CACHE_LU_TBL_SIZE);

    if ((p == 1023) && (offset == 1472))
    {
        //rt_kprintf("hash2:%d,%d\n", hash,dhara_meta_cache_lookup_tbl[hash]);
    }

    //rt_kprintf("replace:%d,%d,%d,%d,%d,%p\n", eldest_cache_idx, cache->p, cache->offset, p, offset, cache);
    if ((cache->p == 1023) && (cache->offset == 1472))
    {
        RT_ASSERT(0);
    }
    cache->n = n;
    cache->p = p;
    cache->offset = offset;
    cache->life = 0;
    rt_memcpy((void *)&cache->data[0], (void *)data, DHARA_META_SIZE);

    cache->prev = 0;
    cache->next = dhara_meta_cache_lookup_tbl[hash];
    if (cache->next)
    {
        DHARA_CHECK_CACHE_IDX(cache->next);
        neigh_cache = DHARA_GET_CACHE(cache->next);
        neigh_cache->prev = eldest_cache_idx;
    }
    /* update header */
    dhara_meta_cache_lookup_tbl[hash] = eldest_cache_idx;

#else

    if (meta_cache_index >= DHARA_META_CACHE_SIZE)
    {
        meta_cache_index = 0;
    }
    cache = &dhara_meta_cache[meta_cache_index];

    //rt_kprintf("cache update,%d,%d\n", index, cache->life);
    //rt_kprintf("%d,%d,%d,%d\n", cache->p, cache->offset, p, offset);
    cache->n = n;
    cache->p = p;
    cache->offset = offset;
    rt_memcpy((void *)&cache->data[0], (void *)data, DHARA_META_SIZE);
    meta_cache_index++;

#endif

}


int dhara_nand_read(const struct dhara_nand *n, dhara_page_t p, size_t offset, size_t length,
                    uint8_t *data, dhara_error_t *err)
{
    struct rt_mtd_nand_device *dev;
    rt_err_t rt_err;
    rt_uint8_t *buf;
    int ret;
    static bool cache_init;
    uint32_t i;
    dhara_meta_cache_t *cache;

    rt_enter_critical();

    if (!cache_init)
    {
        rt_err = rt_mutex_init(&dhara_meta_cache_lock, "dhara", RT_IPC_FLAG_FIFO);
        RT_ASSERT(RT_EOK == rt_err);
        for (i = 0; i < DHARA_META_CACHE_SIZE; i++)
        {
            dhara_meta_cache[i].offset = -1;
#ifdef DHARA_LRU2_CACHE_ENABLED
            dhara_meta_cache[i].life = INT32_MIN;
#endif /* DHARA_LRU2_CACHE_ENABLED */
#ifdef DHARA_LRU_CACHE_ENABLED
            /* construct link list in descending order */
            if (0 == i)
            {
                dhara_meta_cache[i].next = DHARA_META_CACHE_SIZE - 1;
            }
            else
            {
                dhara_meta_cache[i].next = i - 1;
            }
            if ((DHARA_META_CACHE_SIZE - 1) == i)
            {
                dhara_meta_cache[i].prev = 0;
            }
            else
            {
                dhara_meta_cache[i].prev = i + 1;
            }
#endif /* DHARA_LRU_CACHE_ENABLED */
        }
#ifdef DHARA_LRU_CACHE_ENABLED
        dhara_meta_cache_head = DHARA_META_CACHE_SIZE - 1;
#endif
        cache_init = true;
    }
    rt_exit_critical();

    if (length == DHARA_META_SIZE)
    {
        //rt_kprintf("meta:%d,%d\n", p, offset);
        dhara_lock_cache();
        dhara_update_cache_life();
        cache = dhara_find_cache(n, p, offset);
        if (cache)
        {
            rt_memcpy((void *)data, (void *)&cache->data[0], DHARA_META_SIZE);
        }
        dhara_unlock_cache();

        if (cache)
        {
            return 0;
        }
    }

    //rt_kprintf("read:%d,%d,%d\n", p, offset, length);

    dev = (struct rt_mtd_nand_device *)n->user_data;

    if (length < dev->page_size)
    {
        //buf = rt_malloc(dev->page_size);
        buf = n->meta_buf;
        RT_ASSERT(buf);
    }
    else
    {
        buf = data;
    }




#if 0
    rt_err = rt_mtd_nand_read(dev, p, (rt_uint8_t *)buf, dev->page_size, NULL, 0);
#else
    buf = data;
    // rt_err = rt_mtd_nand_read(dev, p, (rt_uint8_t *)data, length, NULL, offset);
    rt_err = rt_mtd_nand_read_with_offset(dev, p, offset, (rt_uint8_t *)data, length, NULL, (rt_uint32_t)NULL);
#endif

    if ((RT_EOK == rt_err) && ((offset + length) <= dev->page_size))
    {
        if (buf != data)
        {
            rt_memcpy(data, buf + offset, length);
        }
        ret = 0;
    }
    else
    {
        if (err)
        {
            *err = DHARA_E_ECC;
        }
        ret = -1;
    }

    if ((0 == ret) && (length == DHARA_META_SIZE))
    {
        dhara_lock_cache();
        dhara_add_cache(n, p, offset, data);
        dhara_unlock_cache();
    }

    //LOG_HEX("dhara_read", 8, (uint8_t *)buf, 2048);

    if (buf != data)
    {
        // rt_free(buf);
    }

    //rt_kprintf("read_res:%d\n", ret);

    return ret;
}

/* Read a page from one location and reprogram it in another location.
 * This might be done using the chip's internal buffers, but it must use
 * ECC.
 */
int dhara_nand_copy(const struct dhara_nand *n, dhara_page_t src, dhara_page_t dst,
                    dhara_error_t *err)
{
    struct rt_mtd_nand_device *dev;
    rt_err_t rt_err;
    rt_uint8_t *buf;
    int ret;
    uint32_t page_size;


    dev = (struct rt_mtd_nand_device *)n->user_data;

    page_size = 1 << n->log2_page_size;
    buf = rt_malloc(page_size);
    RT_ASSERT(buf);

    //rt_kprintf("copy:%d,%d\n", dst, src);

    ret = 0;
    rt_err = rt_mtd_nand_read(dev, src, (rt_uint8_t *)buf, page_size, NULL, 0);

    if (RT_EOK == rt_err)
    {
        rt_err = rt_mtd_nand_write(dev, dst, (rt_uint8_t *)buf, page_size, NULL, 0);
#ifdef RT_DHARA_BBM_ENABLED
        RT_ASSERT(RT_EOK == rt_err);
#endif /* RT_DHARA_BBM_ENABLED */

        if (RT_EOK != rt_err)
        {
            if (err)
            {
                *err = DHARA_E_BAD_BLOCK;
            }
            ret = -1;
        }
    }
    else
    {
        if (err)
        {
            *err = DHARA_E_ECC;
        }
        ret = -1;
    }

    rt_free(buf);

    return ret;
}

void dhara_cache_monitor(uint32_t *access_cnt, uint32_t *miss_cnt)
{
    if (access_cnt)
    {
        *access_cnt = dhara_meta_access_cnt;
    }
    if (miss_cnt)
    {
        *miss_cnt = dhara_meta_cache_miss_cnt;
    }
}

void dhara_cache_monitor_reset(void)
{
    dhara_meta_access_cnt = 0;
    dhara_meta_cache_miss_cnt = 0;
}

