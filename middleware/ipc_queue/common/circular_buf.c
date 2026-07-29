/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef BSP_USING_RTTHREAD
    #include "rtthread.h"
#endif
#include "ipc_hw.h"
#include "circular_buf.h"
#include "string.h"

#ifndef __ROM_USED
    #define __ROM_USED
#endif

#ifdef DBG_CIRCULAR_BUFFER
uint8_t g_cb_backup[512];
void circular_buf_log(struct circular_buf *cb, int idx, const uint8_t *data, int len)
{
    if (cb && cb->backup_ptr)
    {
        uint8_t *g_cb_buf_b = (uint8_t *)&g_cb_backup[0];
        memcpy(g_cb_buf_b + idx, data, len);
    }
}
void circular_buf_set_backup(struct circular_buf *cb)
{
    if (cb)
    {
#ifdef SOC_BF0_HCPU
        if (HCPU2LCPU_MB_CH1_BUF_START_ADDR == (uint32_t)cb)
        {
            cb->backup_ptr = (uint8_t *)HCPU_ADDR_2_LCPU_ADDR((uint32_t)g_cb_backup);
        }
        else
        {
            cb->backup_ptr = 0;
        }
#else
        if (LCPU2HCPU_MB_CH1_BUF_START_ADDR == (uint32_t)cb)
        {
            cb->backup_ptr = (uint8_t *)LCPU_ADDR_2_HCPU_ADDR((uint32_t)g_cb_backup);
        }
        else
        {
            cb->backup_ptr = 0;
        }
#endif
    }
}

void circular_buf_validate(struct circular_buf *cb, int idx, uint8_t *data, int len)
{
    uint8_t i = 0;
    if (cb && cb->magic == DBG_CIRCULAR_MAGIC && cb->backup_ptr)
    {
        if (len && memcmp(cb->backup_ptr + idx, data, len))
        {
            for (i = 0; i < 2; i++)
            {
                memcpy(data, &cb->rd_buffer_ptr[idx], len);
                if (memcmp(cb->backup_ptr + idx, data, len) == 0)
                {
                    break;
                }
            }
            if (i == 2)
            {
                RT_ASSERT(0);
            }
        }
    }
}
#endif

static inline enum circular_buf_state circular_buf_status(struct circular_buf *cb, uint32_t rd_ptr, uint32_t wr_ptr)
{
    uint32_t rd_idx;
    uint32_t wr_idx;
    uint32_t rd_mirror;
    uint32_t wr_mirror;

    rd_idx = CB_GET_PTR_IDX(rd_ptr);
    rd_mirror = CB_GET_PTR_MIRROR(rd_ptr);
    wr_idx = CB_GET_PTR_IDX(wr_ptr);
    wr_mirror = CB_GET_PTR_MIRROR(wr_ptr);

    if (rd_idx == wr_idx)
    {
        if (rd_mirror == wr_mirror)
        {
            return CIRCULAR_BUF_EMPTY;
        }
        else
        {
            return CIRCULAR_BUF_FULL;
        }
    }
    return CIRCULAR_BUF_HALFFULL;
}

__ROM_USED void circular_buf_init(struct circular_buf *cb,
                                  uint8_t           *pool,
                                  int16_t            size)
{
    SF_ASSERT(cb != NULL);
    SF_ASSERT(size > 0);

    /* initialize read and write index */
    cb->read_idx_mirror = 0;
    cb->write_idx_mirror = 0;

    /* set buffer pool for write and size */
    cb->wr_buffer_ptr = pool;
    cb->rd_buffer_ptr = pool;
    cb->buffer_size = size & ~3UL;
}


__ROM_USED void circular_buf_wr_init(struct circular_buf *cb,
                                     uint8_t           *pool,
                                     int16_t            size)
{
    SF_ASSERT(cb != NULL);
    SF_ASSERT(size > 0);

    /* initialize read and write index */
    cb->read_idx_mirror = 0;
    cb->write_idx_mirror = 0;

    /* set buffer pool for write and size */
    cb->wr_buffer_ptr = pool;
    cb->buffer_size = size & ~3UL;
#ifdef  DBG_CIRCULAR_BUFFER
    cb->magic = DBG_CIRCULAR_MAGIC;
#endif
}

__ROM_USED void circular_buf_rd_init(struct circular_buf *cb,
                                     uint8_t           *pool,
                                     int16_t            size)
{
    (void)size;
    /* set buffer pool for read */
    cb->rd_buffer_ptr = pool;
}


/**
 * put a block of data into ring buffer
 */
__ROM_USED size_t circular_buf_put(struct circular_buf *cb,
                                   const uint8_t     *ptr,
                                   uint16_t           length)
{
    uint16_t size;
    uint32_t wr_mirror;
    uint32_t wr_idx;

    SF_ASSERT(cb != NULL);

    /* whether has enough space */
    size = circular_buf_space_len(cb);

    /* no space */
    if (size == 0)
    {
        return 0;
    }

    /* drop some data */
    if (size < length)
    {
        length = size;
    }

    wr_idx = CB_GET_PTR_IDX(cb->write_idx_mirror);
    wr_mirror = CB_GET_PTR_MIRROR(cb->write_idx_mirror);
    if ((cb->buffer_size - wr_idx) > length)
    {
        /* read_index - write_index = empty space */
        memcpy(&cb->wr_buffer_ptr[wr_idx], ptr, length);
        circular_buf_log(cb, wr_idx, ptr, length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        wr_idx += length;
        cb->write_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, wr_mirror);
        return length;
    }

    memcpy(&cb->wr_buffer_ptr[wr_idx],
           &ptr[0],
           cb->buffer_size - wr_idx);
    circular_buf_log(
        cb, wr_idx, &ptr[0],
        cb->buffer_size - wr_idx);
    if ((length - (cb->buffer_size - wr_idx)) > 0)
    {
        memcpy(&cb->wr_buffer_ptr[0],
               &ptr[cb->buffer_size - wr_idx],
               length - (cb->buffer_size - wr_idx));
        circular_buf_log(
            cb, 0, &ptr[cb->buffer_size - wr_idx],
            length - (cb->buffer_size - wr_idx));
    }
    /* we are going into the other side of the mirror */
    wr_mirror = ~wr_mirror;
    wr_idx = length - (cb->buffer_size - wr_idx);
    cb->write_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, wr_mirror);

    return length;
}

/**
 * put a block of data into ring buffer
 *
 * When the buffer is full, it will discard the old data.
 */
__ROM_USED size_t circular_buf_put_force(struct circular_buf *cb,
        const uint8_t       *ptr,
        uint16_t            length)
{
    uint16_t space_length;
    uint32_t wr_mirror;
    uint32_t wr_idx;
    uint32_t rd_mirror;

    SF_ASSERT(cb != NULL);

    space_length = circular_buf_space_len(cb);

    if (length > cb->buffer_size)
    {
        ptr = &ptr[length - cb->buffer_size];
        length = cb->buffer_size;
    }

    wr_idx = CB_GET_PTR_IDX(cb->write_idx_mirror);
    wr_mirror = CB_GET_PTR_MIRROR(cb->write_idx_mirror);
    if ((cb->buffer_size - wr_idx) > length)
    {
        /* read_index - write_index = empty space */
        memcpy(&cb->wr_buffer_ptr[wr_idx], ptr, length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        wr_idx += length;
        cb->write_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, wr_mirror);

        if (length > space_length)
        {
            rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);
            cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, rd_mirror);
        }

        return length;
    }

    memcpy(&cb->wr_buffer_ptr[wr_idx],
           &ptr[0],
           cb->buffer_size - wr_idx);
    memcpy(&cb->wr_buffer_ptr[0],
           &ptr[cb->buffer_size - wr_idx],
           length - (cb->buffer_size - wr_idx));

    /* we are going into the other side of the mirror */
    wr_mirror = ~wr_mirror;
    wr_idx = length - (cb->buffer_size - wr_idx);
    cb->write_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, wr_mirror);

    if (length > space_length)
    {
        rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);
        rd_mirror = ~rd_mirror;
        cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, rd_mirror);
    }

    return length;
}

/**
 *  get data from ring buffer
 */
__ROM_USED size_t circular_buf_get(struct circular_buf *cb,
                                   uint8_t             *ptr,
                                   uint16_t            length)
{
    size_t size;
    uint32_t rd_mirror;
    uint32_t rd_idx;

    SF_ASSERT(cb != NULL);

    /* whether has enough data  */
    size = circular_buf_data_len(cb);

    /* no data */
    if (size == 0)
        return 0;

    /* less data */
    if (size < length)
        length = size;

    rd_idx = CB_GET_PTR_IDX(cb->read_idx_mirror);
    rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);
    if (cb->buffer_size - rd_idx > length)
    {
        /* copy all of data */
        memcpy(ptr, &cb->rd_buffer_ptr[rd_idx], length);
        circular_buf_validate(cb, rd_idx, ptr, length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rd_idx += length;
        cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(rd_idx, rd_mirror);
        return length;
    }

    memcpy(&ptr[0],
           &cb->rd_buffer_ptr[rd_idx],
           cb->buffer_size - rd_idx);
    circular_buf_validate(cb, rd_idx, ptr, cb->buffer_size - rd_idx);
    memcpy(&ptr[cb->buffer_size - rd_idx],
           &cb->rd_buffer_ptr[0],
           length - (cb->buffer_size - rd_idx));
    circular_buf_validate(cb, 0, &ptr[cb->buffer_size - rd_idx], length - (cb->buffer_size - rd_idx));

    /* we are going into the other side of the mirror */
    rd_mirror = ~rd_mirror;
    rd_idx = length - (cb->buffer_size - rd_idx);
    cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(rd_idx, rd_mirror);

    return length;
}


/**
 *  get data from ring buffer and return remaining length
 */
__ROM_USED size_t circular_buf_get_and_update_len(struct circular_buf *cb,
        uint8_t           *ptr,
        uint16_t           length,
        size_t            *remaining_len)
{
    size_t size;
    uint32_t mask;
    uint32_t rd_mirror;
    uint32_t rd_idx;

    SF_ASSERT(cb != NULL);

    /* whether has enough data  */
    size = circular_buf_data_len(cb);

    /* no data */
    if (size == 0)
        return 0;

    /* less data */
    if (size < length)
        length = size;

    rd_idx = CB_GET_PTR_IDX(cb->read_idx_mirror);
    rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);
    if (cb->buffer_size - rd_idx > length)
    {

        /* copy all of data */
        memcpy(ptr, &cb->rd_buffer_ptr[rd_idx], length);
        circular_buf_validate(cb, rd_idx, ptr, length);

        mask = os_interrupt_disable();

        if (remaining_len)
        {
            *remaining_len = circular_buf_data_len(cb) - length;
        }

        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rd_idx += length;
        cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(rd_idx, rd_mirror);

        os_interrupt_enable(mask);

        return length;
    }

    memcpy(&ptr[0],
           &cb->rd_buffer_ptr[rd_idx],
           cb->buffer_size - rd_idx);
    circular_buf_validate(cb, rd_idx, ptr, cb->buffer_size - rd_idx);
    memcpy(&ptr[cb->buffer_size - rd_idx],
           &cb->rd_buffer_ptr[0],
           length - (cb->buffer_size - rd_idx));
    circular_buf_validate(cb, 0, &ptr[cb->buffer_size - rd_idx], length - (cb->buffer_size - rd_idx));

    mask = os_interrupt_disable();

    if (remaining_len)
    {
        *remaining_len = circular_buf_data_len(cb) - length;
    }

    /* we are going into the other side of the mirror */
    rd_mirror = ~rd_mirror;
    rd_idx = length - (cb->buffer_size - rd_idx);
    cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(rd_idx, rd_mirror);

    os_interrupt_enable(mask);

    return length;
}




/**
 * put a character into ring buffer
 */
__ROM_USED size_t circular_buf_putchar(struct circular_buf *cb, const uint8_t ch)
{
    uint32_t wr_mirror;
    uint32_t wr_idx;

    SF_ASSERT(cb != NULL);

    /* whether has enough space */
    if (!circular_buf_space_len(cb))
        return 0;

    wr_idx = CB_GET_PTR_IDX(cb->write_idx_mirror);
    wr_mirror = CB_GET_PTR_MIRROR(cb->write_idx_mirror);
    cb->wr_buffer_ptr[wr_idx] = ch;

    /* flip mirror */
    if ((int16_t)wr_idx == cb->buffer_size - 1)
    {
        wr_mirror = ~wr_mirror;
        wr_idx = 0;
    }
    else
    {
        wr_idx++;
    }
    cb->write_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, wr_mirror);

    return 1;
}

/**
 * put a character into ring buffer
 *
 * When the buffer is full, it will discard one old data.
 */
__ROM_USED size_t circular_buf_putchar_force(struct circular_buf *cb, const uint8_t ch)
{
    enum circular_buf_state old_state;
    uint32_t wr_mirror;
    uint32_t wr_idx;
    uint32_t rd_mirror;

    SF_ASSERT(cb != NULL);

    old_state = circular_buf_status(cb, cb->read_idx_mirror, cb->write_idx_mirror);

    wr_idx = CB_GET_PTR_IDX(cb->write_idx_mirror);
    wr_mirror = CB_GET_PTR_MIRROR(cb->write_idx_mirror);
    cb->wr_buffer_ptr[wr_idx] = ch;

    /* flip mirror */
    if ((int16_t)wr_idx == cb->buffer_size - 1)
    {
        wr_mirror = ~wr_mirror;
        wr_idx = 0;
        if (old_state == CIRCULAR_BUF_FULL)
        {
            rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);
            rd_mirror = ~rd_mirror;
            cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, rd_mirror);
        }
    }
    else
    {
        wr_idx++;
        if (old_state == CIRCULAR_BUF_FULL)
        {
            rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);
            cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, rd_mirror);
        }
    }

    return 1;
}

/**
 * get a character from a circular_buf
 */
__ROM_USED size_t circular_buf_getchar(struct circular_buf *cb, uint8_t *ch)
{
    uint32_t rd_mirror;
    uint32_t rd_idx;

    SF_ASSERT(cb != NULL);

    /* circular_buf is empty */
    if (!circular_buf_data_len(cb))
        return 0;

    rd_idx = CB_GET_PTR_IDX(cb->read_idx_mirror);
    rd_mirror = CB_GET_PTR_MIRROR(cb->read_idx_mirror);

    /* put character */
    *ch = cb->rd_buffer_ptr[rd_idx];

    if ((int16_t)rd_idx == cb->buffer_size - 1)
    {
        rd_mirror = ~rd_mirror;
        rd_idx = 0;
    }
    else
    {
        rd_idx++;
    }
    cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(rd_idx, rd_mirror);

    return 1;
}

/**
 * get the size of data in cb
 */
__ROM_USED size_t circular_buf_data_len(struct circular_buf *cb)
{
    uint32_t rd_idx;
    uint32_t wr_idx;
    uint32_t rd_ptr;
    uint32_t wr_ptr;

    rd_ptr = cb->read_idx_mirror;
    wr_ptr = cb->write_idx_mirror;
    switch (circular_buf_status(cb, rd_ptr, wr_ptr))
    {
    case CIRCULAR_BUF_EMPTY:
        return 0;
    case CIRCULAR_BUF_FULL:
        return cb->buffer_size;
    case CIRCULAR_BUF_HALFFULL:
    default:
        rd_idx = CB_GET_PTR_IDX(rd_ptr);
        wr_idx = CB_GET_PTR_IDX(wr_ptr);
        if (wr_idx > rd_idx)
            return wr_idx - rd_idx;
        else
            return cb->buffer_size - (rd_idx - wr_idx);
    };
}

/**
 * empty the cb
 */
__ROM_USED void circular_buf_reset(struct circular_buf *cb)
{
    SF_ASSERT(cb != NULL);

    cb->read_idx_mirror = 0;
    cb->write_idx_mirror = 0;
}


