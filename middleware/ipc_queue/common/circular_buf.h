/**
  ******************************************************************************
  * @file   circular_buf.h
  * @author Sifli software development team
  * @brief Sifli circular_buf library interface
  * @{
  ******************************************************************************
*/
/*
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

#ifndef CIRCULAR_BUF_H__
#define CIRCULAR_BUF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "sf_type.h"


#define CB_PTR_MIRROR_OFFSET        (0)
#define CB_PTR_MIRROR_MASK          (0xFFFF)
#define CB_PTR_IDX_OFFSET           (16)
#define CB_PTR_IDX_MASK             (0xFFFF)


#define CB_MAKE_PTR_IDX_MIRROR(idx, mirror)   (((uint32_t)(idx) << CB_PTR_IDX_OFFSET) | ((mirror) & CB_PTR_MIRROR_MASK))
#define CB_GET_PTR_IDX(ptr_idx_mirror)        (((ptr_idx_mirror) >> CB_PTR_IDX_OFFSET) & CB_PTR_IDX_MASK)
#define CB_GET_PTR_MIRROR(ptr_idx_mirror)     ((ptr_idx_mirror) & CB_PTR_MIRROR_MASK)



/* circular buffer */
struct circular_buf
{
    uint8_t *rd_buffer_ptr;
    uint8_t *wr_buffer_ptr;
    /* use the msb of the {read,write}_index as mirror bit. You can see this as
     * if the buffer adds a virtual mirror and the pointers point either to the
     * normal or to the mirrored buffer. If the write_index has the same value
     * with the read_index, but in a different mirror, the buffer is full.
     * While if the write_index and the read_index are the same and within the
     * same mirror, the buffer is empty. The ASCII art of the ringbuffer is:
     *
     *          mirror = 0                    mirror = 1
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Full
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     *  read_idx-^                   write_idx-^
     *
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Empty
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * read_idx-^ ^-write_idx
     *
     * The tradeoff is we could only use 32KiB of buffer for 16 bit of index.
     * But it should be enough for most of the cases.
     *
     * Ref: http://en.wikipedia.org/wiki/Circular_buffer#Mirroring */
    uint32_t read_idx_mirror;
    uint32_t write_idx_mirror;
    /* as we use msb of index as mirror bit, the size should be signed and
     * could only be positive. */
    int16_t buffer_size;
};

enum circular_buf_state
{
    CIRCULAR_BUF_EMPTY,
    CIRCULAR_BUF_FULL,
    /* half full is neither full nor empty */
    CIRCULAR_BUF_HALFFULL,
};

/**
 * CircularBuffer for DeviceDriver
 *
 */
void circular_buf_init(struct circular_buf *cb, uint8_t *pool, int16_t size);
void circular_buf_wr_init(struct circular_buf *cb, uint8_t *pool, int16_t size);
void circular_buf_rd_init(struct circular_buf *cb, uint8_t *pool, int16_t size);
void circular_buf_reset(struct circular_buf *cb);
size_t circular_buf_put(struct circular_buf *cb, const uint8_t *ptr, uint16_t length);
size_t circular_buf_put_force(struct circular_buf *cb, const uint8_t *ptr, uint16_t length);
size_t circular_buf_putchar(struct circular_buf *cb, const uint8_t ch);
size_t circular_buf_putchar_force(struct circular_buf *cb, const uint8_t ch);
size_t circular_buf_get(struct circular_buf *cb, uint8_t *ptr, uint16_t length);
size_t circular_buf_get_and_update_len(struct circular_buf *cb,
                                       uint8_t           *ptr,
                                       uint16_t           length,
                                       size_t             *remaining_len);
size_t circular_buf_getchar(struct circular_buf *cb, uint8_t *ch);
size_t circular_buf_data_len(struct circular_buf *cb);


inline uint16_t circular_buf_get_size(struct circular_buf *cb)
{
    SF_ASSERT(cb != NULL);
    return cb->buffer_size;
}

/** return the size of empty space in cb */
#define circular_buf_space_len(cb) ((cb)->buffer_size - circular_buf_data_len(cb))


#ifdef __cplusplus
}
#endif

#endif  /* CIRCULAR_BUF_H__ */
