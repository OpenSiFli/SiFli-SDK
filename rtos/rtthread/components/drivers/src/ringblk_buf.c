/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-25     armink       the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

/**
 * ring block buffer object initialization
 *
 * @param rbb ring block buffer object
 * @param buf buffer
 * @param buf_size buffer size
 * @param block_set block set
 * @param blk_max_num max block number
 *
 * @note When your application need align access, please make the buffer address is aligned.
 */
__ROM_USED void rt_rbb_init(rt_rbb_t rbb, rt_uint8_t *buf, rt_size_t buf_size, rt_rbb_blk_t block_set, rt_size_t blk_max_num)
{
#ifndef RT_RBB_NOT_USING_BLK_SET
    rt_size_t i;
#endif

    RT_ASSERT(rbb);
    RT_ASSERT(buf);
#ifndef RT_RBB_NOT_USING_BLK_SET
    RT_ASSERT(block_set);
#endif

    rbb->buf = buf;
    rbb->buf_size = buf_size;
#ifndef RT_RBB_NOT_USING_BLK_SET
    rbb->blk_set = block_set;
    rbb->blk_max_num = blk_max_num;
#endif

    rt_list_init(&rbb->blk_list);
#ifndef RT_RBB_NOT_USING_BLK_SET
    /* initialize block status */
    for (i = 0; i < blk_max_num; i++)
    {
        block_set[i].status = RT_RBB_BLK_UNUSED;
    }
#endif
}
RTM_EXPORT(rt_rbb_init);

/**
 * ring block buffer object create
 *
 * @param buf_size buffer size
 * @param blk_max_num max block number
 *
 * @return != NULL: ring block buffer object
 *            NULL: create failed
 */
__ROM_USED rt_rbb_t rt_rbb_create(rt_size_t buf_size, rt_size_t blk_max_num)
{
    rt_rbb_t rbb = NULL;
    rt_uint8_t *buf;
    rt_rbb_blk_t blk_set = NULL;

    rbb = (rt_rbb_t)rt_malloc(sizeof(struct rt_rbb));
    if (!rbb)
    {
        return NULL;
    }

    buf = (rt_uint8_t *)rt_malloc(buf_size);
    if (!buf)
    {
        rt_free(rbb);
        return NULL;
    }

#ifndef RT_RBB_NOT_USING_BLK_SET
    blk_set = (rt_rbb_blk_t)rt_malloc(sizeof(struct rt_rbb_blk) * blk_max_num);
    if (!blk_set)
    {
        rt_free(buf);
        rt_free(rbb);
        return NULL;
    }
#endif
    rt_rbb_init(rbb, buf, buf_size, blk_set, blk_max_num);

    return rbb;
}
RTM_EXPORT(rt_rbb_create);

/**
 * ring block buffer object destroy
 *
 * @param rbb ring block buffer object
 */
__ROM_USED void rt_rbb_destroy(rt_rbb_t rbb)
{
    RT_ASSERT(rbb);

    rt_free(rbb);
    rt_free(rbb->buf);
#ifndef RT_RBB_NOT_USING_BLK_SET
    rt_free(rbb->blk_set);
#endif

}
RTM_EXPORT(rt_rbb_destroy);

#ifndef RT_RBB_NOT_USING_BLK_SET
static rt_rbb_blk_t find_empty_blk_in_set(rt_rbb_t rbb)
{
    rt_size_t i;

    RT_ASSERT(rbb);

    for (i = 0; i < rbb->blk_max_num; i ++)
    {
        if (rbb->blk_set[i].status == RT_RBB_BLK_UNUSED)
        {
            return &rbb->blk_set[i];
        }
    }

    return NULL;
}
#endif

/**
 * Allocate a block by given size. The block will add to blk_list when allocate success.
 *
 * @param rbb ring block buffer object
 * @param blk_size block size
 *
 * @note When your application need align access, please make the blk_szie is aligned.
 *
 * @return != NULL: allocated block
 *            NULL: allocate failed
 */
#ifndef RT_RBB_NOT_USING_BLK_SET
__ROM_USED rt_rbb_blk_t rt_rbb_blk_alloc(rt_rbb_t rbb, rt_size_t blk_size)
{
    rt_base_t level;
    rt_size_t empty1 = 0, empty2 = 0;
    rt_rbb_blk_t head, tail, new = NULL;

    RT_ASSERT(rbb);
    RT_ASSERT(blk_size < (1L << 24));

    level = rt_hw_interrupt_disable();

    new = find_empty_blk_in_set(rbb);

    if (rt_list_len(&rbb->blk_list) < rbb->blk_max_num && new)
    {
        if (!rt_list_isempty(&rbb->blk_list))
        {
            head = rt_list_first_entry(&rbb->blk_list, struct rt_rbb_blk, list);
            tail = rt_list_tail_entry(&rbb->blk_list, struct rt_rbb_blk, list);
            if (head->buf <= tail->buf)
            {
                /**
                 *                      head                     tail
                 * +--------------------------------------+-----------------+------------------+
                 * |      empty2     | block1 |   block2  |      block3     |       empty1     |
                 * +--------------------------------------+-----------------+------------------+
                 *                            rbb->buf
                 */
                empty1 = (rbb->buf + rbb->buf_size) - (tail->buf + tail->size);
                empty2 = head->buf - rbb->buf;

                if (empty1 >= blk_size)
                {
                    rt_list_insert_before(&rbb->blk_list, &new->list);
                    new->status = RT_RBB_BLK_INITED;
                    new->buf = tail->buf + tail->size;
                    new->size = blk_size;
                }
                else if (empty2 >= blk_size)
                {
                    rt_list_insert_before(&rbb->blk_list, &new->list);
                    new->status = RT_RBB_BLK_INITED;
                    new->buf = rbb->buf;
                    new->size = blk_size;
                }
                else
                {
                    /* no space */
                    new = NULL;
                }
            }
            else
            {
                /**
                 *        tail                                              head
                 * +----------------+-------------------------------------+--------+-----------+
                 * |     block3     |                empty1               | block1 |  block2   |
                 * +----------------+-------------------------------------+--------+-----------+
                 *                            rbb->buf
                 */
                empty1 = head->buf - (tail->buf + tail->size);

                if (empty1 >= blk_size)
                {
                    rt_list_insert_before(&rbb->blk_list, &new->list);
                    new->status = RT_RBB_BLK_INITED;
                    new->buf = tail->buf + tail->size;
                    new->size = blk_size;
                }
                else
                {
                    /* no space */
                    new = NULL;
                }
            }
        }
        else
        {
            /* the list is empty */
            rt_list_insert_before(&rbb->blk_list, &new->list);
            new->status = RT_RBB_BLK_INITED;
            new->buf = rbb->buf;
            new->size = blk_size;
        }
    }
    else
    {
        new = NULL;
    }

    rt_hw_interrupt_enable(level);

    return new;
}
#else
__ROM_USED rt_rbb_blk_t rt_rbb_blk_alloc(rt_rbb_t rbb, rt_size_t blk_size)
{
    rt_base_t level;
    rt_size_t empty1 = 0, empty2 = 0;
    rt_rbb_blk_t head, tail, new = NULL;

    RT_ASSERT(rbb);
    RT_ASSERT(blk_size < (1L << 24));

    level = rt_hw_interrupt_disable();

    if (!rt_list_isempty(&rbb->blk_list))
    {
        head = rt_list_first_entry(&rbb->blk_list, struct rt_rbb_blk, list);
        tail = rt_list_tail_entry(&rbb->blk_list, struct rt_rbb_blk, list);
        if (head->buf <= tail->buf)
        {
            /**
             *                      head                     tail
             * +--------------------------------------+-----------------+------------------+
             * |      empty2     | block1 |   block2  |      block3     |       empty1     |
             * +--------------------------------------+-----------------+------------------+
             *                            rbb->buf
             */
            empty1 = (rbb->buf + rbb->buf_size) - (tail->buf + tail->size);
            empty2 = head->buf - rbb->buf - sizeof(*new);

            if (empty1 >= (blk_size + sizeof(*new)))
            {
                new = (rt_rbb_blk_t)(tail->buf + tail->size);
                rt_list_insert_before(&rbb->blk_list, &new->list);
                new->status = RT_RBB_BLK_INITED;
                new->buf = (rt_uint8_t *)new + sizeof(*new);
                new->size = blk_size;
            }
            else if (empty2 >= (blk_size + sizeof(*new)))
            {
                new = (rt_rbb_blk_t)rbb->buf;
                rt_list_insert_before(&rbb->blk_list, &new->list);
                new->status = RT_RBB_BLK_INITED;
                new->buf = (rt_uint8_t *)new + sizeof(*new);
                new->size = blk_size;
            }
            else
            {
                /* no space */
                new = NULL;
            }
        }
        else
        {
            /**
             *        tail                                              head
             * +----------------+-------------------------------------+--------+-----------+
             * |     block3     |                empty1               | block1 |  block2   |
             * +----------------+-------------------------------------+--------+-----------+
             *                            rbb->buf
             */
            empty1 = head->buf - (tail->buf + tail->size) - sizeof(*new);

            if (empty1 >= (blk_size + sizeof(*new)))
            {
                new = (rt_rbb_blk_t)(tail->buf + tail->size);
                rt_list_insert_before(&rbb->blk_list, &new->list);
                new->status = RT_RBB_BLK_INITED;
                new->buf = (rt_uint8_t *)new + sizeof(*new);
                new->size = blk_size;
            }
            else
            {
                /* no space */
                new = NULL;
            }
        }
    }
    else if (rbb->buf_size >= (blk_size + sizeof(*new)))
    {
        new = (rt_rbb_blk_t)rbb->buf;
        rt_list_insert_before(&rbb->blk_list, &new->list);
        new->status = RT_RBB_BLK_INITED;
        new->buf = rbb->buf + sizeof(*new);
        new->size = blk_size;
    }

    rt_hw_interrupt_enable(level);

    return new;
}

#endif
RTM_EXPORT(rt_rbb_blk_alloc);

/**
 * put a block to ring block buffer object
 *
 * @param block the block
 */
__ROM_USED void rt_rbb_blk_put(rt_rbb_blk_t block)
{
    RT_ASSERT(block);
    RT_ASSERT(block->status == RT_RBB_BLK_INITED);

    block->status = RT_RBB_BLK_PUT;
}
RTM_EXPORT(rt_rbb_blk_put);

/**
 * get a block from the ring block buffer object
 *
 * @param rbb ring block buffer object
 *
 * @return != NULL: block
 *            NULL: get failed
 */
__ROM_USED rt_rbb_blk_t rt_rbb_blk_get(rt_rbb_t rbb)
{
    rt_base_t level;
    rt_rbb_blk_t block = NULL;
    rt_list_t *node;

    RT_ASSERT(rbb);

    if (rt_list_isempty(&rbb->blk_list))
        return 0;

    level = rt_hw_interrupt_disable();

    rt_list_for_each(node, &rbb->blk_list)
    {
        block = rt_list_entry(node, struct rt_rbb_blk, list);
        if (block->status == RT_RBB_BLK_PUT)
        {
            block->status = RT_RBB_BLK_GET;
            goto __exit;
        }
    }
    /* not found */
    block = NULL;

__exit:

    rt_hw_interrupt_enable(level);

    return block;
}
RTM_EXPORT(rt_rbb_blk_get);

/**
 * return the block size
 *
 * @param block the block
 *
 * @return block size
 */
__ROM_USED rt_size_t rt_rbb_blk_size(rt_rbb_blk_t block)
{
    RT_ASSERT(block);

    return block->size;
}
RTM_EXPORT(rt_rbb_blk_size);

/**
 * return the block buffer
 *
 * @param block the block
 *
 * @return block buffer
 */
__ROM_USED rt_uint8_t *rt_rbb_blk_buf(rt_rbb_blk_t block)
{
    RT_ASSERT(block);

    return block->buf;
}
RTM_EXPORT(rt_rbb_blk_buf);

/**
 * free the block
 *
 * @param rbb ring block buffer object
 * @param block the block
 */
__ROM_USED void rt_rbb_blk_free(rt_rbb_t rbb, rt_rbb_blk_t block)
{
    rt_base_t level;

    RT_ASSERT(rbb);
    RT_ASSERT(block);
    RT_ASSERT(block->status != RT_RBB_BLK_UNUSED);

    level = rt_hw_interrupt_disable();

    /* remove it on rbb block list */
    rt_list_remove(&block->list);

    block->status = RT_RBB_BLK_UNUSED;

    rt_hw_interrupt_enable(level);
}
RTM_EXPORT(rt_rbb_blk_free);

/**
 * get a continuous block to queue by given size
 *
 *          tail                         head
 * +------------------+---------------+--------+----------+--------+
 * |      block3      |  empty1       | block1 |  block2  |fragment|
 * +------------------+------------------------+----------+--------+
 *                                    |<-- return_size -->|    |
 *                                    |<--- queue_data_len --->|
 *
 *         tail                          head
 * +------------------+---------------+--------+----------+--------+
 * |      block3      |  empty1       | block1 |  block2  |fragment|
 * +------------------+------------------------+----------+--------+
 *                                    |<-- return_size -->|              out of len(b1+b2+b3)    |
 *                                    |<-------------------- queue_data_len -------------------->|
 *
 * @param rbb ring block buffer object
 * @param queue_data_len The max queue data size, and the return size must less then it.
 * @param queue continuous block queue
 *
 * @return the block queue data total size
 */
__ROM_USED rt_size_t rt_rbb_blk_queue_get(rt_rbb_t rbb, rt_size_t queue_data_len, rt_rbb_blk_queue_t blk_queue)
{
    rt_base_t level;
    rt_size_t data_total_size = 0;
    rt_list_t *node;
    rt_rbb_blk_t last_block = NULL, block;

    RT_ASSERT(rbb);
    RT_ASSERT(blk_queue);

    if (rt_list_isempty(&rbb->blk_list))
        return 0;

    level = rt_hw_interrupt_disable();

    rt_list_for_each(node, &rbb->blk_list)
    {
        if (!last_block)
        {
            last_block = rt_list_entry(node, struct rt_rbb_blk, list);
            if (last_block->status == RT_RBB_BLK_PUT)
            {
                /* save the first put status block to queue */
                blk_queue->blocks = last_block;
                blk_queue->blk_num = 0;
            }
            else
            {
                /* the first block must be put status */
                last_block = NULL;
                continue;
            }
        }
        else
        {
            block = rt_list_entry(node, struct rt_rbb_blk, list);
            /*
             * these following conditions will break the loop:
             * 1. the current block is not put status
             * 2. the last block and current block is not continuous
             * 3. the data_total_size will out of range
             */
            if (block->status != RT_RBB_BLK_PUT ||
                    last_block->buf > block->buf ||
                    data_total_size + block->size > queue_data_len)
            {
                break;
            }
            /* backup last block */
            last_block = block;
        }
        /* remove current block */
        rt_list_remove(&last_block->list);
        data_total_size += last_block->size;
        last_block->status = RT_RBB_BLK_GET;
        blk_queue->blk_num++;
    }

    rt_hw_interrupt_enable(level);

    return data_total_size;
}
RTM_EXPORT(rt_rbb_blk_queue_get);

/**
 * get all block length on block queue
 *
 * @param blk_queue the block queue
 *
 * @return total length
 */
__ROM_USED rt_size_t rt_rbb_blk_queue_len(rt_rbb_blk_queue_t blk_queue)
{
    rt_size_t i, data_total_size = 0;

    RT_ASSERT(blk_queue);

    for (i = 0; i < blk_queue->blk_num; i++)
    {
        data_total_size += blk_queue->blocks[i].size;
    }

    return data_total_size;
}
RTM_EXPORT(rt_rbb_blk_queue_len);

/**
 * return the block queue buffer
 *
 * @param blk_queue the block queue
 *
 * @return block queue buffer
 */
__ROM_USED rt_uint8_t *rt_rbb_blk_queue_buf(rt_rbb_blk_queue_t blk_queue)
{
    RT_ASSERT(blk_queue);

    return blk_queue->blocks[0].buf;
}
RTM_EXPORT(rt_rbb_blk_queue_buf);

/**
 * free the block queue
 *
 * @param rbb ring block buffer object
 * @param blk_queue the block queue
 */
__ROM_USED void rt_rbb_blk_queue_free(rt_rbb_t rbb, rt_rbb_blk_queue_t blk_queue)
{
    rt_size_t i;

    RT_ASSERT(rbb);
    RT_ASSERT(blk_queue);

    for (i = 0; i < blk_queue->blk_num; i++)
    {
        rt_rbb_blk_free(rbb, &blk_queue->blocks[i]);
    }
}
RTM_EXPORT(rt_rbb_blk_queue_free);

/**
 * The put status and buffer continuous blocks can be make a block queue.
 * This function will return the length which from next can be make block queue.
 *
 * @param rbb ring block buffer object
 *
 * @return the next can be make block queue's length
 */
__ROM_USED rt_size_t rt_rbb_next_blk_queue_len(rt_rbb_t rbb)
{
    rt_base_t level;
    rt_size_t data_len = 0;
    rt_list_t *node;
    rt_rbb_blk_t last_block = NULL, block;

    RT_ASSERT(rbb);

    if (rt_list_isempty(&rbb->blk_list))
        return 0;

    level = rt_hw_interrupt_disable();

    rt_list_for_each(node, &rbb->blk_list)
    {
        if (!last_block)
        {
            last_block = rt_list_entry(node, struct rt_rbb_blk, list);
            if (last_block->status != RT_RBB_BLK_PUT)
            {
                /* the first block must be put status */
                last_block = NULL;
                continue;
            }
        }
        else
        {
            block = rt_list_entry(node, struct rt_rbb_blk, list);
            /*
             * these following conditions will break the loop:
             * 1. the current block is not put status
             * 2. the last block and current block is not continuous
             */
            if (block->status != RT_RBB_BLK_PUT || last_block->buf > block->buf)
            {
                break;
            }
            /* backup last block */
            last_block = block;
        }
        data_len += last_block->size;
    }

    rt_hw_interrupt_enable(level);

    return data_len;
}
RTM_EXPORT(rt_rbb_next_blk_queue_len);

/**
 * get the ring block buffer object buffer size
 *
 * @param rbb ring block buffer object
 *
 * @return buffer size
 */
__ROM_USED rt_size_t rt_rbb_get_buf_size(rt_rbb_t rbb)
{
    RT_ASSERT(rbb);

    return rbb->buf_size;
}
RTM_EXPORT(rt_rbb_get_buf_size);
