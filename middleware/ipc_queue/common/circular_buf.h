/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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

#if defined(SOC_SF32LB58X) && defined(LCPU_CONFIG_V2) && !defined(BF0_ACPU)
#define DBG_CIRCULAR_BUFFER
#endif
#define DBG_CIRCULAR_MAGIC 0xDBCBE123
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
#ifdef  DBG_CIRCULAR_BUFFER
    uint32_t magic;
    uint8_t *backup_ptr;
#endif
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

#ifdef  DBG_CIRCULAR_BUFFER
void circular_buf_log(struct circular_buf *cb, int idx, const uint8_t *data, int len);
void circular_buf_set_backup(struct circular_buf *cb);
void circular_buf_validate(struct circular_buf *cb, int idx, uint8_t *data, int len);
#else
#define circular_buf_log(cb,idx,data,len)
#define circular_buf_set_backup(cb)
#define circular_buf_validate(cb,idx,data,len)
#endif


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
