/**
  ******************************************************************************
  * @file   circular_buf.c
  * @author Sifli software development team
  * @brief Sibles circular buf library source.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifdef BSP_USING_RTTHREAD
    #include "rtthread.h"
#endif
#include "circular_buf.h"
#include "string.h"
#include "ipc_hw.h"

#ifndef __ROM_USED
    #define __ROM_USED
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
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        wr_idx += length;
        cb->write_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(wr_idx, wr_mirror);
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
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rd_idx += length;
        cb->read_idx_mirror = CB_MAKE_PTR_IDX_MIRROR(rd_idx, rd_mirror);
        return length;
    }

    memcpy(&ptr[0],
           &cb->rd_buffer_ptr[rd_idx],
           cb->buffer_size - rd_idx);
    memcpy(&ptr[cb->buffer_size - rd_idx],
           &cb->rd_buffer_ptr[0],
           length - (cb->buffer_size - rd_idx));

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
    memcpy(&ptr[cb->buffer_size - rd_idx],
           &cb->rd_buffer_ptr[0],
           length - (cb->buffer_size - rd_idx));

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


