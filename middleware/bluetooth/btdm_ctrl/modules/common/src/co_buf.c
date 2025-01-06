/**
 ****************************************************************************************
 * @file co_buf.c
 *
 * @brief Common Buffer Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_BUF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (CO_BUF_PRESENT)
#include "co_buf.h"       // Common buffer module
#include "co_math.h"      // Common Mathematic module
#include "ke_mem.h"       // For dynamic memory allocation
#include <string.h>       // For memcpy

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// pattern used to detect meta-data memory overflow into a buffer
#define CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN   (0xA5)


/// Buffer pool identifier
enum co_buf_pool_id
{
    /// "Small" Buffer pool
    CO_BUF_POOL_SMALL         = 0xFC,
    /// "Big" Buffer pool
    CO_BUF_POOL_BIG           = 0xFD,
    /// Buffer dynamically  allocated from kernel memory heap
    CO_BUF_POOL_DYNAMIC_ALLOC = 0xFE,
    /// Buffer created by an upper layer software module
    CO_BUF_POOL_UNKNOWN       = 0xFF,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Buffer environment structure
typedef struct co_buf_env_
{
    /// Small Buffer Pool
    co_list_t          small_pool;
    /// Big Buffer Pool
    co_list_t          big_pool;
    /// List of buffers dynamically allocated
    co_list_t          dyn_alloc_buf;
#if (RW_DEBUG)
    /// Number of buffer allocated
    uint32_t           nb_buf_alloc;
#endif // (RW_DEBUG)
} co_buf_env_t;


/// Free callback info
typedef struct co_buf_free_info
{
    /// Pointer to the function called when the buffer is free.
    co_buf_free_cb  cb_free;
    /// Pointer to environment that will be used as callback parameter.
    void           *p_env;
} co_buf_free_info_t;

/// Encapsulation structure used to keep list of dynamically allocated buffers.
typedef struct co_buf_dyn_alloc
{
    /// List header
    co_list_hdr_t hdr;
    /// Buffer content
    co_buf_t      buf;
} co_buf_dyn_alloc_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Buffer environment structure
__STATIC co_buf_env_t co_buf_env;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Copy data from a place to another
 *
 * @param[in] p_out     Pointer to the data buffer in which data must be written
 * @param[in] p_in      Pointer to the data buffer from which data must be read
 * @param[in] length    Length of data to be copied
 ****************************************************************************************
 */
__STATIC void co_buf_mem_cpy(uint32_t *p_out, uint32_t *p_in, uint32_t length)
{
    // Remaining length to copy
    uint32_t rem_len = length;
    // Counter
    uint8_t cnt;

    // Check if provided pointers are word aligned
    if ((((uint32_t)p_out % 4) == 0) && (((uint32_t)p_in % 4) == 0))
    {
        while (rem_len >= 4)
        {
            // Copy 4 bytes
            *p_out = *p_in;

            // Move pointers
            p_out++;
            p_in++;

            // Decrease remaining length
            rem_len -= 4;
        }
    }
    else
    {
        while (rem_len >= 4)
        {
            // Copy 4 bytes
            co_write32p(p_out, co_read32p(p_in));

            // Move pointers
            p_out++;
            p_in++;

            // Decrease remaining length
            rem_len -= 4;
        }
    }

    // Copy remaining bytes
    for (cnt = 0; cnt < rem_len; cnt++)
    {
        // Copy 1 byte
        *((uint8_t *)p_out) = *((uint8_t *)p_in);

        // Move pointer
        p_out = (uint32_t *)(((uint8_t *)p_out) + 1);
        p_in  = (uint32_t *)(((uint8_t *)p_in)  + 1);
    }
}


/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

void co_buf_init(bool is_reset, uint32_t *p_big_pool, uint32_t *p_small_pool)
{
    DBG_MEM_GRANT_CTRL(&(co_buf_env), true);

    if (!is_reset)
    {
        co_list_init(&(co_buf_env.big_pool));
        co_list_init(&(co_buf_env.small_pool));
        co_list_init(&(co_buf_env.dyn_alloc_buf));
    }

#if (RW_DEBUG)
    co_buf_env.nb_buf_alloc = 0;
#endif // (RW_DEBUG)

    // Initialize the list of big buffers
    co_list_pool_init(&(co_buf_env.big_pool),
                      p_big_pool,
                      CO_ALIGN4_HI(sizeof(co_buf_t) + CO_BUF_BIG_SIZE),
                      CO_BUF_BIG_NB);

    // Initialize the list of small buffers
    co_list_pool_init(&(co_buf_env.small_pool),
                      p_small_pool,
                      CO_ALIGN4_HI(sizeof(co_buf_t) + CO_BUF_SMALL_SIZE),
                      CO_BUF_SMALL_NB);

    while (!co_list_is_empty(&(co_buf_env.dyn_alloc_buf)))
    {
        ke_free(co_list_pop_front(&(co_buf_env.dyn_alloc_buf)));
    }

    DBG_MEM_GRANT_CTRL(&(co_buf_env), false);
}

uint8_t co_buf_alloc(co_buf_t **pp_buf, uint16_t head_len, uint16_t data_len, uint16_t tail_len)
{
    DBG_FUNC_ENTER(co_buf_alloc);
    // Returned status
    uint8_t status = CO_BUF_ERR_INSUFFICIENT_RESOURCE;
    // Required buffer length
    uint16_t buf_length = head_len + data_len + tail_len;
    // total size of buffer
    uint16_t buf_size;
    // Pool used
    uint8_t  pool_id;
    co_buf_t *p_buf = NULL;

    if (buf_length <= CO_BUF_SMALL_SIZE)
    {
        DBG_MEM_GRANT_CTRL(&(co_buf_env.small_pool), true);
        // Can use a small buffer
        p_buf    = (co_buf_t *) co_list_pop_front(&(co_buf_env.small_pool));

        pool_id  = CO_BUF_POOL_SMALL;
        buf_size = CO_BUF_SMALL_SIZE;
        DBG_MEM_GRANT_CTRL(&(co_buf_env.small_pool), false);
    }
    else if (buf_length <= CO_BUF_BIG_SIZE)
    {
        DBG_MEM_GRANT_CTRL(&(co_buf_env.big_pool), true);
        // Can use a big buffer
        p_buf    = (co_buf_t *) co_list_pop_front(&(co_buf_env.big_pool));
        pool_id  = CO_BUF_POOL_BIG;
        buf_size = CO_BUF_BIG_SIZE;
        DBG_MEM_GRANT_CTRL(&(co_buf_env.big_pool), false);
    }

    // check if buffer must be dynamically allocated
    if (p_buf == NULL)
    {
        uint16_t alloc_size = (sizeof(co_buf_dyn_alloc_t) + buf_length);
        co_buf_dyn_alloc_t *p_buf_dyn_alloc = (co_buf_dyn_alloc_t *) ke_malloc_user(alloc_size, KE_MEM_KE_MSG);

        if (p_buf_dyn_alloc != NULL)
        {
            // keep in mind that buffer has been dynamically allocated
            co_list_push_back(&(co_buf_env.dyn_alloc_buf), &(p_buf_dyn_alloc->hdr));

            p_buf = &(p_buf_dyn_alloc->buf);
            pool_id  = CO_BUF_POOL_DYNAMIC_ALLOC;
            buf_size = buf_length;
        }
    }

    if (p_buf != NULL)
    {
        DBG_MEM_PERM_SET(p_buf, sizeof(co_buf_t) + buf_size, true, true, true);
        // Set provided length
        p_buf->data_len        = data_len;
        p_buf->head_len        = head_len;
        p_buf->tail_len        = buf_size - data_len - head_len;
        // Initialize acquire counter
        p_buf->acq_cnt         = 1;

        p_buf->pool_id         = pool_id;
        p_buf->pattern         = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN;
        // Initialize meta-data bit field
        p_buf->metadata_bf = 0;
#if (RW_DEBUG)
        co_buf_env.nb_buf_alloc++;
#endif // (RW_DEBUG)

        // Return the allocated buffer
        *pp_buf = p_buf;

        // And a no error status
        status = CO_BUF_ERR_NO_ERROR;
    }

    ASSERT_WARN(status == CO_BUF_ERR_NO_ERROR, status, 0);

    DBG_FUNC_EXIT(co_buf_alloc);
    return (status);
}

uint8_t co_buf_prepare(co_buf_t *p_buf, uint16_t head_len, uint16_t data_len, uint16_t tail_len)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    if (p_buf != NULL)
    {
        // Set provided length
        p_buf->head_len        = head_len;
        p_buf->tail_len        = tail_len;
        p_buf->data_len        = data_len;
        // Initialize acquire counter
        p_buf->acq_cnt         = 1;

        p_buf->pool_id         = CO_BUF_POOL_UNKNOWN;
        p_buf->pattern         = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN;
        // Initialize meta-data bit field
        p_buf->metadata_bf = 0;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_acquire(co_buf_t *p_buf)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    ASSERT_ERR((p_buf != NULL)); //TODO understand and restore
    ASSERT_ERR((p_buf->acq_cnt > 0)); //TODO understand and restore

    if ((p_buf != NULL) && (p_buf->acq_cnt > 0))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Increment acquire counter
        p_buf->acq_cnt += 1;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_release(co_buf_t *p_buf)
{
    DBG_FUNC_ENTER(co_buf_release);
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    ASSERT_ERR((p_buf != NULL)); //TODO understand and restore
    ASSERT_ERR((p_buf->acq_cnt > 0)); //TODO understand and restore

    if ((p_buf != NULL) && (p_buf->acq_cnt > 0))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Decrement acquire counter
        p_buf->acq_cnt -= 1;


        // check if buffer can be completely released
        if (p_buf->acq_cnt == 0)
        {
            // inform buffer user that free operation is performed
            if (GETB(p_buf->metadata_bf, CO_BUF_METADATA_FREE_CB))
            {
                co_buf_free_info_t *p_cb_info = (co_buf_free_info_t *) & (p_buf->metadata[0]);
                p_cb_info->cb_free(p_buf, p_cb_info->p_env);
            }

            // Free the buffer
            switch (p_buf->pool_id)
            {
            case CO_BUF_POOL_SMALL:
            {
                DBG_MEM_GRANT_CTRL(p_buf, true);
                co_list_push_back(&(co_buf_env.small_pool), &(p_buf->hdr));
                DBG_MEM_PERM_SET(p_buf, sizeof(co_buf_t) + CO_BUF_SMALL_SIZE, false, false, false);
                DBG_MEM_GRANT_CTRL(p_buf, false);
            }
            break;
            case CO_BUF_POOL_BIG:
            {
                DBG_MEM_GRANT_CTRL(p_buf, true);
                co_list_push_back(&(co_buf_env.big_pool), &(p_buf->hdr));
                DBG_MEM_PERM_SET(p_buf, sizeof(co_buf_t) + CO_BUF_BIG_SIZE, false, false, false);
                DBG_MEM_GRANT_CTRL(p_buf, false);
            }
            break;
            case CO_BUF_POOL_DYNAMIC_ALLOC:
            {
                // retrieve allocated pointer
                co_buf_dyn_alloc_t *p_buf_dyn_alloc =
                    (co_buf_dyn_alloc_t *)(((uint8_t *)p_buf) - offsetof(co_buf_dyn_alloc_t, buf));

                // extract buffer from dynamically allocated buffer list
                co_list_extract(&(co_buf_env.dyn_alloc_buf), &(p_buf_dyn_alloc->hdr));

                // free memory
                ke_free(p_buf_dyn_alloc);
            }
            break;
            case CO_BUF_POOL_UNKNOWN:       { /* Nothing to do */                                          } break;
            default:
            {
                ASSERT_INFO_FORCE(0, p_buf->pool_id, 0);
            }
            break;
            }

#if (RW_DEBUG)
            co_buf_env.nb_buf_alloc--;
#endif // (RW_DEBUG)
        }

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    DBG_FUNC_EXIT(co_buf_release);
    return (status);
}

uint16_t co_buf_size(const co_buf_t *p_buf)
{
    return sizeof(co_buf_t) + p_buf->head_len + p_buf->data_len + p_buf->tail_len;
}

uint8_t co_buf_head_reserve(co_buf_t *p_buf, uint16_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    ASSERT_ERR((p_buf != NULL) && (p_buf->head_len >= length));

    if ((p_buf != NULL) && (p_buf->head_len >= length))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);


        // Increment data length
        p_buf->data_len += length;

        // decrement head length
        p_buf->head_len -= length;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_head_release(co_buf_t *p_buf, uint16_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    ASSERT_ERR(p_buf != NULL);

    if ((p_buf != NULL) && (p_buf->data_len >= length))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Increment head length
        p_buf->head_len += length;

        // decrement data length
        p_buf->data_len -= length;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);

}

uint8_t co_buf_tail_reserve(co_buf_t *p_buf, uint16_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    ASSERT_ERR((p_buf != NULL) && (p_buf->tail_len >= length));

    if ((p_buf != NULL) && (p_buf->tail_len >= length))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Increment data length
        p_buf->data_len += length;

        // decrement tail length
        p_buf->tail_len -= length;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_tail_release(co_buf_t *p_buf, uint16_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    ASSERT_ERR(p_buf != NULL);

    if ((p_buf != NULL) && (p_buf->data_len >= length))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Increment tail length
        p_buf->tail_len += length;

        // decrement data length
        p_buf->data_len -= length;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_metadata_freeze(co_buf_t *p_buf, uint8_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // update meta-data length in block of 4 bytes
    length = CO_ALIGN4_HI(length) >> 2;

    if ((p_buf != NULL) && ((GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) + length) <= CO_BUF_META_DATA_SIZE))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // decrement frozen size
        SETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE,
             GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) + length);

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_metadata_unfreeze(co_buf_t *p_buf, uint8_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // update meta-data length in block of 4 bytes
    length = CO_ALIGN4_HI(length) >> 2;

    if ((p_buf != NULL) && (GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) >= length))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // decrement frozen size
        SETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE, GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) - length);

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_duplicate(const co_buf_t *p_buf_in, co_buf_t **pp_buf_out, uint16_t head_len, uint16_t tail_len)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // check pointers
    if (p_buf_in != NULL)
    {
        ASSERT_ERR(p_buf_in->pattern == CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // allocate output buffer
        status = co_buf_alloc(pp_buf_out, head_len, p_buf_in->data_len, tail_len);

        if (status == CO_BUF_ERR_NO_ERROR)
        {
            // Copy data
            co_buf_mem_cpy((uint32_t *) & ((*pp_buf_out)->buf[(*pp_buf_out)->head_len]),
                           (uint32_t *) & (p_buf_in->buf[p_buf_in->head_len]),
                           p_buf_in->data_len);
        }
    }

    return (status);
}

uint8_t co_buf_copy(const co_buf_t *p_buf_in, co_buf_t *p_buf_out, uint16_t length, uint8_t copy_meta_size)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // update meta-data length in block of 4 bytes
    copy_meta_size = CO_ALIGN4_HI(copy_meta_size) >> 2;

    // check pointers
    if ((p_buf_in != NULL) && ((p_buf_out != NULL))
            // check if data can be copied in output buffer
            && (p_buf_out->data_len >= length)
            // check if meta-data can be copied
            && ((GETF(p_buf_out->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) + copy_meta_size) <= CO_BUF_META_DATA_SIZE))
    {
        ASSERT_ERR(p_buf_in->pattern == CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);
        ASSERT_ERR(p_buf_out->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Copy data
        co_buf_mem_cpy((uint32_t *) & (p_buf_out->buf[p_buf_out->head_len]),
                       (uint32_t *) & (p_buf_in->buf[p_buf_in->head_len]),
                       length);

        // Copy meta-data
        co_buf_mem_cpy((uint32_t *) & (p_buf_out->metadata[GETF(p_buf_out->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE)]),
                       (uint32_t *) & (p_buf_in->metadata[GETF(p_buf_in->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE)]),
                       copy_meta_size << 2);

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_reuse(co_buf_t *p_buf)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // check that buffer is ready to be free
    if ((p_buf != NULL) && (p_buf->acq_cnt == 1))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        p_buf->acq_cnt -= 1;

        // make like if buffer is free
        if (GETB(p_buf->metadata_bf, CO_BUF_METADATA_FREE_CB))
        {
            co_buf_free_info_t *p_cb_info = (co_buf_free_info_t *) & (p_buf->metadata[0]);
            p_cb_info->cb_free(p_buf, p_cb_info->p_env);
        }

        // initialize buffer to be reused
        p_buf->acq_cnt        = 1;
        p_buf->metadata_bf = 0;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_reuse_full(co_buf_t *p_buf, uint16_t head_len, uint16_t data_len, uint16_t tail_len)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // Compute buffer length and required buffer length
    uint16_t buf_len     = p_buf->head_len + p_buf->data_len + p_buf->tail_len;
    uint16_t exp_buf_len = head_len + data_len + tail_len;

    // check that buffer is ready to be free
    if ((p_buf != NULL) && (p_buf->acq_cnt == 1)
            // check if buffer is big enough
            && (buf_len >= exp_buf_len))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        p_buf->acq_cnt -= 1;

        // make like if buffer is free
        if (GETB(p_buf->metadata_bf, CO_BUF_METADATA_FREE_CB))
        {
            co_buf_free_info_t *p_cb_info = (co_buf_free_info_t *) & (p_buf->metadata[0]);
            p_cb_info->cb_free(p_buf, p_cb_info->p_env);
        }

        // initialize buffer to be reused
        p_buf->acq_cnt         = 1;
        p_buf->metadata_bf = 0;
        p_buf->head_len        = head_len;
        p_buf->data_len        = data_len;
        p_buf->tail_len        = tail_len;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_cb_free_set(co_buf_t *p_buf, co_buf_free_cb cb_free, void *p_env)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // check pointer
    if ((p_buf != NULL)
            // Free callback already set
            && (GETB(p_buf->metadata_bf, CO_BUF_METADATA_FREE_CB)
                // or no meta-data frozen
                || GETF(p_buf->metadata_bf, CO_BUF_METADATA_FROZEN_SIZE) == 0))
    {
        co_buf_free_info_t *p_cb_info = (co_buf_free_info_t *) & (p_buf->metadata[0]);

        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        if (!GETB(p_buf->metadata_bf, CO_BUF_METADATA_FREE_CB))
        {
            // freeze meta-data for callback
            co_buf_metadata_freeze(p_buf, sizeof(co_buf_free_info_t));
        }

        SETB(p_buf->metadata_bf, CO_BUF_METADATA_FREE_CB, true);

        // set callback information
        p_cb_info->cb_free = cb_free;
        p_cb_info->p_env   = p_env;

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_copy_data_from_mem(co_buf_t *p_buf, const uint8_t *p_in, uint16_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // check pointers
    if ((p_buf != NULL) && ((p_in != NULL))
            // check if data can be copied in output buffer
            && (p_buf->data_len >= length))
    {
        ASSERT_ERR(p_buf->pattern = CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Copy data
        co_buf_mem_cpy((uint32_t *) & (p_buf->buf[p_buf->head_len]),
                       (uint32_t *) p_in,
                       length);

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}

uint8_t co_buf_copy_data_to_mem(const co_buf_t *p_buf, uint8_t *p_out, uint16_t length)
{
    uint8_t status = CO_BUF_ERR_INVALID_PARAM;

    // check pointers
    if ((p_buf != NULL) && ((p_out != NULL))
            // check if data enough big
            && (p_buf->data_len >= length))
    {
        ASSERT_ERR(p_buf->pattern == CO_BUF_ENV_MEM_OVERFLOW_DETECTION_PATTERN);

        // Copy data
        co_buf_mem_cpy((uint32_t *) p_out,
                       (uint32_t *) & (p_buf->buf[p_buf->head_len]),
                       length);

        // Update return status
        status = CO_BUF_ERR_NO_ERROR;
    }

    return (status);
}


bool co_buf_append_data(co_buf_t **pp_buf, uint16_t length, const uint8_t *p_data)
{
    bool succeed = false;

    if (*pp_buf == NULL)
    {
        co_buf_alloc(pp_buf, 0, 0, length);
    }
    else if (co_buf_tail_len(*pp_buf) < length)
    {
        co_buf_t *p_old_buf = *pp_buf;
        co_buf_t *p_new_buf = NULL;

        // allocate a bigger buffer to concatenate data
        co_buf_duplicate(p_old_buf, &p_new_buf, 0, length);

        // release old buffer
        co_buf_release(p_old_buf);
        *pp_buf = p_new_buf;
    }


    if (*pp_buf)
    {
        // append fragment
        memcpy(co_buf_tail(*pp_buf), p_data, length);
        co_buf_tail_reserve(*pp_buf, length);

        succeed = true;
    }
    return (succeed);
}

#endif // (CO_BUF_PRESENT)
/// @} CO_BUF

