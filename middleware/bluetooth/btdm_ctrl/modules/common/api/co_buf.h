/**
 ****************************************************************************************
 *
 * @file co_buf.h
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */
#ifndef _CO_BUF_H_
#define _CO_BUF_H_

/**
 ****************************************************************************************
 * @defgroup CO_BUF Buffer
 * @ingroup COMMON_API
 * @brief  Time utilities
 *
 * The Common Buffer module provides buffer used for manipulation of data for network
 * protocol that uses encapsulation of header or trailing information.
 * A buffer is a contiguous memory section used to store message information including
 * several protocol layers.
 *
 * Since a unique buffer can be used by multiple layers, a mechanism monitors the
 * buffer life cycle. Finally, if data usage information are kept within the buffer,
 * it speeds up software to retrieve the execution context.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>       // standard definitions
#include <stddef.h>       // standard definitions

#include "arch.h"         // Arch defines
#include "co_list.h"      // List manipulation
#include "co_utils.h"     // Bit Field manipulation

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */
/// size of meta-data variables 32 bytes
#define CO_BUF_META_DATA_SIZE (32 >> 2)

/// Buffer Error status codes
enum co_buf_err
{
    /// No Error
    CO_BUF_ERR_NO_ERROR              = 0x00, ///! 0x00
    /// Invalid parameter(s)
    CO_BUF_ERR_INVALID_PARAM         = 0x01, ///! 0x01
    /// Not enough resources
    CO_BUF_ERR_INSUFFICIENT_RESOURCE = 0x02, ///! 0x02
    /// Resource is busy, operation cannot be performed
    CO_BUF_ERR_RESOURCE_BUSY         = 0x03, ///! 0x03
};


/// Buffer meta-data bit field
enum co_buf_metadata_bf
{
    /// Size of meta-data data frozen. This size has a step of 4 bytes
    CO_BUF_METADATA_FROZEN_SIZE_LSB                  = 0,
    CO_BUF_METADATA_FROZEN_SIZE_MASK                 = 0x0F,
    /// If equals 1, a callback is executed before freeing the buffer.
    CO_BUF_METADATA_FREE_CB_POS                      = 4,
    CO_BUF_METADATA_FREE_CB_BIT                      = 0x10,
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Buffer structure
typedef struct co_buf
{
    /// List header for chaining
    co_list_hdr_t hdr;
    /// Length of the data part
    uint16_t      data_len;
    /// Prefix length available
    uint16_t      head_len;
    /// Suffix length available
    uint16_t      tail_len;
    /// Pool identifier (see enum co_buf_pool_id)
    uint8_t       pool_id;
    /// Acquisition counter
    uint8_t       acq_cnt;
    /// Meta-data variable that can be used for multiple purposes
    /// meta-data is always 32 bits aligned
    uint32_t      metadata[CO_BUF_META_DATA_SIZE];
    /// Pattern used to verify that meta-data didn’t overflow
    uint8_t       pattern;
    /// Meta-data bit field (see enum #co_buf_metadata_bf)
    uint8_t       metadata_bf;
    /// Padding
    uint16_t      padding;

    /// Variable buffer array that contains header, data and tail payload
    /// Length is buf_len = (head_len + data_len + tail_len)
    uint8_t       buf[__ARRAY_EMPTY];
} co_buf_t;

/**
 ****************************************************************************************
 * @brief This function is called when all software modules has release the buffer.
 *
 * @param[in] p_buf   Pointer to buffer that being free
 * @param[in] p_env   Pointer to environment that will be used as callback parameter.
 ****************************************************************************************
 */
typedef void (*co_buf_free_cb)(co_buf_t *p_buf, void *p_env);

/*
 * CONSTANT DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Allocate a buffer and specify initial length of head, data and tail parts.
 *        When doing a buffer allocation, acquisition counter is equals to 1.
 *
 * If total length is lower than @see CO_BUF_SMALL_SIZE and small buffer pool is not empty:
 *      A buffer will be picked from small buffer pool.
 * else if total length is lower than @see CO_BUF_BIG_SIZE and big buffer pool is not empty:
 *      A buffer will be picked from big buffer pool.
 * else:
 *      the buffer will be dynamically allocated.
 *
 * @param[out] pp_buf      Pointer to a variable that will contain the address of the allocated buffer.
 * @param[in]  head_len    Initial Head Length.
 * @param[in]  data_len    Initial Data Length.
 * @param[in]  tail_len    Initial Tail Length.
 *
 * @return CO_BUF_ERR_NO_ERROR if buffer can be allocated.
 *         CO_BUF_ERR_INSUFFICIENT_RESOURCE if no more buffers are available.
 ****************************************************************************************
 */
uint8_t co_buf_alloc(co_buf_t **pp_buf, uint16_t head_len, uint16_t data_len, uint16_t tail_len);

/**
 ****************************************************************************************
 * @brief Prepare a buffer and specify initial length of head, data and tail parts.
 *
 * Buffer pointer provided must have a total length greater or equals to head_len + data_len + tail_len
 * When doing a buffer allocation, acquisition counter is equals to 1.
 * Buffer isn’t free by @see co_buf_release function when acquisition counter is equals to 0.

 *
 * @param[out] p_buf       Pointer to buffer to prepare.
 * @param[in]  head_len    Initial Head Length.
 * @param[in]  data_len    Initial Data Length.
 * @param[in]  tail_len    Initial Tail Length.
 *
 * @return CO_BUF_ERR_NO_ERROR if buffer can be allocated.
 ****************************************************************************************
 */
uint8_t co_buf_prepare(co_buf_t *p_buf, uint16_t head_len, uint16_t data_len, uint16_t tail_len);

/**
 ****************************************************************************************
 * @brief Function used to increment value of acquire counter of a buffer during processing
 *        of buffer content.
 *
 * @param[in] p_buf        Pointer to acquired buffer
 *
 * @return CO_BUF_ERR_NO_ERROR if operation succeed
 ****************************************************************************************
 */
uint8_t co_buf_acquire(co_buf_t *p_buf);

/**
 ****************************************************************************************
 * @brief Function used to release previously acquired buffer. The acquire counter for
 *        this buffer is decremented. If the acquire counter value becomes zero,
 *        the buffer is freed as no more entity is using the buffer anymore.
 *
 *        if acquire counter becomes zero
 *            if a free callback has been configured using @see co_buf_cb_set:
 *                  call the free callback
 *
 *            if buffer comes from a buffer pool:
 *                  buffer is pushed to the corresponding pool, and available for another
 *                  software module
 *
 *            else if buffer comes from dynamic memory:
 *                  corresponding memory is free
 *
 *            else if a buffer has been initialized with @see co_buf_prepare:
 *                  nothing is done
 *
 * @note  A software module shall not use a buffer after releasing it.
 *
 * @param[in] p_buf        Pointer to acquired buffer.
 *
 * @return CO_BUF_ERR_NO_ERROR if buffer has been released.
 *         CO_BUF_ERR_INVALID_PARAM if buffer was free.
 ****************************************************************************************
 */
uint8_t co_buf_release(co_buf_t *p_buf);

/**
 ****************************************************************************************
 * @brief Retrieve buffer data pointer.
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Pointer to first byte of data field ; NULL if an error occurs
 ****************************************************************************************
 */
__INLINE uint8_t *co_buf_data(co_buf_t *p_buf)
{
    uint8_t *p_ret = NULL;

    if (p_buf)
    {
        p_ret = &(p_buf->buf[p_buf->head_len]);
    }

    return (p_ret);
}

/**
 ****************************************************************************************
 * @brief Retrieve buffer data length.
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Buffer data field size. 0 if an error occurs.
 ****************************************************************************************
 */
__INLINE uint16_t co_buf_data_len(const co_buf_t *p_buf)
{
    uint16_t ret = 0;

    if (p_buf)
    {
        ret = p_buf->data_len;
    }

    return (ret);
}

/**
 ****************************************************************************************
 * @brief Retrieve buffer available prefix length.
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Buffer data prefix size available. 0 if an error occurs.
 ****************************************************************************************
 */
__INLINE uint16_t co_buf_head_len(const co_buf_t *p_buf)
{
    uint16_t ret = 0;

    if (p_buf)
    {
        ret = p_buf->head_len;
    }

    return (ret);
}

/**
 ****************************************************************************************
 * @brief  Memory Size of the buffer
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Memory size of the buffer
 ****************************************************************************************
 */
uint16_t co_buf_size(const co_buf_t *p_buf);

/**
 ****************************************************************************************
 * @brief Retrieve buffer head pointer.
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Pointer to first byte of tail field ; NULL if an error occurs
 ****************************************************************************************
 */
__INLINE uint8_t *co_buf_head(co_buf_t *p_buf)
{
    uint8_t *p_ret = NULL;

    if (p_buf)
    {
        p_ret = &(p_buf->buf[0]);
    }

    return (p_ret);
}

/**
 ****************************************************************************************
 * @brief Prefix the data with a given header length, it is mandatory that data reserved
 *        length is less or equals to header length.
 *        Header length is reduced according to number of byte reserved.

 *
 * @param[in] p_buf        Pointer to buffer
 * @param[in] length       Length of prefix data to reserve.
 *
 * @return CO_BUF_ERR_NO_ERROR if needed length has been reserved.
 *         CO_BUF_ERR_INVALID_PARAM if provided length is higher than current length of head part.
 ****************************************************************************************
 */
uint8_t co_buf_head_reserve(co_buf_t *p_buf, uint16_t length);

/**
 ****************************************************************************************
 * @brief Remove a data prefix of a specific length header length, it is mandatory that
 *        data released length is less or equals to data length.
 *        Header length is increased according to number of byte released.
 *
 * @param[in] p_buf        Pointer to buffer
 * @param[in] length       Length of prefix data to release.
 *
 * @return CO_BUF_ERR_NO_ERROR if needed length has been released.
 *         CO_BUF_ERR_INVALID_PARAM if provided length is higher than current length of data part.
 ****************************************************************************************
 */
uint8_t co_buf_head_release(co_buf_t *p_buf, uint16_t length);

/**
 ****************************************************************************************
 * @brief Retrieve buffer tail pointer.
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Pointer to first byte of tail field ; NULL if an error occurs
 ****************************************************************************************
 */
__INLINE uint8_t *co_buf_tail(co_buf_t *p_buf)
{
    uint8_t *p_ret = NULL;

    if (p_buf)
    {
        p_ret = &(p_buf->buf[p_buf->head_len + p_buf->data_len]);
    }

    return (p_ret);
}

/**
 ****************************************************************************************
 * @brief Retrieve buffer available suffix length.
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Buffer data suffix size available. 0 if an error occurs.
 ****************************************************************************************
 */
__INLINE uint16_t co_buf_tail_len(const co_buf_t *p_buf)
{
    uint16_t ret = 0;

    if (p_buf)
    {
        ret = p_buf->tail_len;
    }

    return (ret);
}

/**
 ****************************************************************************************
 * @brief Prefix the data with a given tail length, it is mandatory that data reserved
 *        length is less or equals to header length.
 *        Tail length is reduced according to number of byte reserved.
 *
 * @param[in] p_buf        Pointer to buffer
 * @param[in] length       Length of suffix data to reserve.
 *
 * @return CO_BUF_ERR_NO_ERROR if needed length has been reserved.
 *         CO_BUF_ERR_INVALID_PARAM if provided length is higher than current length of tail part.
 ****************************************************************************************
 */
uint8_t co_buf_tail_reserve(co_buf_t *p_buf, uint16_t length);

/**
 ****************************************************************************************
 * @brief Remove a data suffix of a specific length header length, it is mandatory that
 *        data released length is less or equals to data length.
 *        Tail length is increased according to number of byte released.
 *
 * @param[in] p_buf        Pointer to buffer
 * @param[in] length       Length of suffix data to release.
 *
 * @return CO_BUF_ERR_NO_ERROR if needed length has been released.
 *         CO_BUF_ERR_INVALID_PARAM if provided length is higher than current length of data part.
 ****************************************************************************************
 */
uint8_t co_buf_tail_release(co_buf_t *p_buf, uint16_t length);

/**
 ****************************************************************************************
 * @brief Retrieve pointer to buffer meta-data. This pointer is 32-bit aligned, and
 *        corresponds to buffer meta-data start pointer plus blocked meta-data length
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Pointer to the available meta-data pointer ;NULL if an error occurs
 ****************************************************************************************
 */
__INLINE uint8_t *co_buf_metadata(co_buf_t *p_buf)
{
    uint8_t *p_ret = NULL;

    if ((p_buf) && (GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) < CO_BUF_META_DATA_SIZE))
    {
        p_ret = (uint8_t *) & (p_buf->metadata[GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE)]);
    }

    return (p_ret);
}

/**
 ****************************************************************************************
 * @brief Freeze some meta-data into the buffer, this update the meta-data
 *        pointer given by @see co_buf_metadata function .
 *        Frozen meta-data should not be updated. A software module that has frozen some
 *        meta-data must unblock it before using it or before releasing buffer.
 *
 *        Length provided is aligned to 4 bytes in order to ensure that meta-data pointer
 *        is always 32-bits aligned. Length parameter cannot exceed remaining meta-data
 *        length.
 *
 * @param[in] p_buf        Pointer to buffer
 * @param[in] length       Number of byte in buffer meta-data to freeze.
 *
 * @return CO_BUF_ERR_NO_ERROR if needed length has been frozen.
 *         CO_BUF_ERR_INVALID_PARAM if provided length is higher than remaining meta-data size
 ****************************************************************************************
 */
uint8_t co_buf_metadata_freeze(co_buf_t *p_buf, uint8_t length);

/**
 ****************************************************************************************
 * @brief Unfreeze some meta-data into the buffer, this update the meta-data
 *        pointer given by @see co_buf_metadata function.
 *        Length provided is aligned to 4 bytes in order to ensure that meta-data
 *        pointer is always 32-bits aligned.
 *
 *        Length parameter cannot exceed meta-data frozen length
 *
 * @param[in] p_buf        Pointer to buffer
 * @param[in] length       Number of byte in buffer meta-data to un-freeze.
 *
 * @return CO_BUF_ERR_NO_ERROR if needed length has been frozen.
 *         CO_BUF_ERR_INVALID_PARAM if provided length is higher than frozen meta-data size
 ****************************************************************************************
 */
uint8_t co_buf_metadata_unfreeze(co_buf_t *p_buf, uint8_t length);

/**
 ****************************************************************************************
 * @brief Retrieve number of bytes in meta-data field that can be used by software layer
 *
 * @param[in] p_buf        Pointer to buffer
 *
 * @return Number of byte in buffer meta-data remains 0 if an error occurs
 ****************************************************************************************
 */
__INLINE uint8_t co_buf_metadata_len(const co_buf_t *p_buf)
{
    uint8_t ret = 0;

    if (p_buf)
    {
        ret = (CO_BUF_META_DATA_SIZE - GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE)) << 2;
    }

    return (ret);
}

/**
 ****************************************************************************************
 * @brief Allocate a new buffer, specify initial length of head and tail parts, plus copy
 *        data of input buffer.
 *
 *        @see m_buf_alloc function is used to allocate output buffer.
 *
 * @note meta-data isn't copied
 *
 * @param[in]  p_buf_in        Pointer to input buffer.
 * @param[out] pp_buf_out      Pointer to output buffer.
 * @param[in]  head_len        Head Length of allocated output buffer
 * @param[in]  tail_len        Tail Length of allocated output buffer
 *
 * @return CO_BUF_ERR_NO_ERROR if buffer can be allocated.
 *         CO_BUF_ERR_INSUFFICIENT_RESOURCE if no more buffers are available.
 ****************************************************************************************
 */
uint8_t co_buf_duplicate(const co_buf_t *p_buf_in, co_buf_t **pp_buf_out, uint16_t head_len, uint16_t tail_len);

/**
 ****************************************************************************************
 * @brief Copy content of a buffer to another buffer.
 *
 * @param[in] p_buf_in        Pointer to input buffer.
 * @param[in] p_buf_out       Pointer to output buffer.
 * @param[in] length          Length of data to copy
 * @param[in] copy_meta_size  Indicate size of meta-data to copy. It doesn't copy frozen data.
 *
 * @return CO_BUF_ERR_NO_ERROR if copy has been properly performed.
 *         CO_BUF_ERR_INVALID_PARAM if the output buffer data size is cannot accept input data length.
 ****************************************************************************************
 */
uint8_t co_buf_copy(const co_buf_t *p_buf_in, co_buf_t *p_buf_out, uint16_t length, uint8_t copy_meta_size);

/**
 ****************************************************************************************
 * @brief Reuse a given buffer with keeping data information. This can be done only if
 *        buffer has been released by other software module. meta-data data can be
 *        considered empty if function execution succeeds.
 *
 *        If this function succeeds it must be considered as an old buffer release and
 *        new buffer allocation.
 *        If function execution fails, the buffer is not considered as released
 *
 * @param[in] p_buf        Pointer to the buffer.
 *
 * @return CO_BUF_ERR_NO_ERROR if buffer has been properly released and reused
 *         CO_BUF_ERR_RESOURCE_BUSY if buffer acquisition counter > 1
 ****************************************************************************************
 */
uint8_t co_buf_reuse(co_buf_t *p_buf);

/**
 ****************************************************************************************
 * @brief Reuse a given buffer without keeping data information. This can be done only if
 *        buffer has been released by other software module. meta-data data can be
 *        considered empty if function execution succeeds.
 *
 *        Size of header, data_ and trailing length must not exceed size of the buffer.
 *
 *        If this function succeeds it must be considered as an old buffer release and
 *        new buffer allocation.
 *        If function execution fails, the buffer is not considered as released.
 *
 * @param[in] p_buf        Pointer to the buffer.
 * @param[in] head_len     Initial Head Length.
 * @param[in] data_len     Initial Data Length.
 * @param[in] tail_len     Initial Tail Length.
 *
 * @return CO_BUF_ERR_NO_ERROR if buffer has been properly released and reused
 *         CO_BUF_ERR_RESOURCE_BUSY if buffer acquisition counter > 1
 *         CO_BUF_ERR_INVALID_PARAM if length fields exceed length of initial buffer
 ****************************************************************************************
 */
uint8_t co_buf_reuse_full(co_buf_t *p_buf, uint16_t head_len, uint16_t data_len, uint16_t tail_len);

/**
 ****************************************************************************************
 * @brief TThis function allows a software module to be informed when buffer is free.
 *        It freezes 8 bytes in buffer meta-data.
 *
 *        This function shall be called only after a buffer allocation or reuse.
 *
 * @param[in] p_buf        Pointer to the buffer.
 * @param[in] cb_free      Pointer to the function called when the buffer is free.
 * @param[in] p_env        Pointer to environment that will be used as callback parameter.
 *
 * @return CO_BUF_ERR_NO_ERROR callback has been properly set
 *         CO_BUF_ERR_RESOURCE_BUSY if some buffer meta-data already frozen
 ****************************************************************************************
 */
uint8_t co_buf_cb_free_set(co_buf_t *p_buf, co_buf_free_cb cb_free, void *p_env);

/**
 ****************************************************************************************
 * @brief This function copies content of an input memory data to the data part of a
 *        buffer. The length field shall not exceed buffer data length.
 *
 * @param[in] p_buf       Pointer to the buffer.
 * @param[in] p_in        Pointer to input data.
 * @param[in] length      Length of data to copy
 *
 * @return CO_BUF_ERR_NO_ERROR if copy has been properly performed.
 *         CO_BUF_ERR_INVALID_PARAM if data length fields < length parameter
 ****************************************************************************************
 */
uint8_t co_buf_copy_data_from_mem(co_buf_t *p_buf, const uint8_t *p_in, uint16_t length);

/**
 ****************************************************************************************
 * @brief This function copies data part of a buffer into an output memory block.
 *        The length field shall not exceed buffer data length..
 *
 * @param[in] p_buf       Pointer to the buffer.
 * @param[in] p_out       Pointer to output data.
 * @param[in] length      Length of data to copy
 *
 * @return CO_BUF_ERR_NO_ERROR if copy has been properly performed.
 *         CO_BUF_ERR_INVALID_PARAM if data length fields < length parameter
 ****************************************************************************************
 */
uint8_t co_buf_copy_data_to_mem(const co_buf_t *p_buf, uint8_t *p_out, uint16_t length);

/**
 ****************************************************************************************
 * @brief Append data to a buffer. if buffer isn't big enough, a new buffer is allocated
 *
 * @param[in,out] pp_buf    Pointer to Buffer pointer
 * @param[in]     length    Length of data to append to buffer
 * @param[in]     p_data    Pointer to data
 *
 * @return True if operation succeed, False otherwise
 ****************************************************************************************
 */
bool co_buf_append_data(co_buf_t **pp_buf, uint16_t length, const uint8_t *p_data);

/**
 ****************************************************************************************
 * @brief Initialize Common buffer module.
 *
 * @param[in] is_reset     True if reset is requested, false for an initialization
 * @param[in] p_big_pool   Pointer to the Big pool memory Array
 * @param[in] p_small_pool Pointer to the Small pool memory Array
 ****************************************************************************************
 */
void co_buf_init(bool is_reset, uint32_t *p_big_pool, uint32_t *p_small_pool);

/// @} CO_BUF

#endif // _CO_BUF_H_
