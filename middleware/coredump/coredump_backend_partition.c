/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "coredump.h"
#include "drv_flash.h"
#include "fal.h"
#if defined(BSP_USING_SDIO)
    #include "mmcsd_core.h"
#endif /* BSP_USING_SDIO */

#define COREDUMP_CACHE_BUF_SIZE (4096)

typedef size_t (*coredump_backend_erase_t)(uint32_t offset, size_t len);
typedef struct
{
    /** rt device, such as SPI flash, SDMMC, etc. */
    rt_device_t device;
    /** fal device */
    const struct fal_flash_dev *fal_dev;
    /** start address of the partition*/
    uint32_t start_addr;
    /** total size of the partition, unit: bytes */
    size_t total_size;
    /** current write position, unit: bytes */
    uint32_t wr_pos;
    coredump_backend_write_t write;
    coredump_backend_erase_t erase;
    coredump_backend_read_t read;
    /** cache buffer for NAND flash */
    uint8_t *cache_buf;
    /** current size of data in cache buffer, unit: bytes */
    uint16_t cache_data_size;
    /** total size of cache buffer, unit: bytes */
    uint16_t cache_buf_size;
    coredump_backend_state_t state;
} coredump_backend_partition_ctx_t;


static coredump_backend_partition_ctx_t fulldump_partition_backend_ctx;

#ifdef COREDUMP_MINIDUMP_ENABLED
    static coredump_backend_partition_ctx_t minidump_partition_backend_ctx;
#endif /* COREDUMP_MINIDUMP_ENABLED */
static coredump_backend_partition_ctx_t *partition_backend_ctx = &fulldump_partition_backend_ctx;
static uint8_t partition_cache_buf[COREDUMP_CACHE_BUF_SIZE];


#ifdef BSP_USING_SPI_FLASH
static size_t coredump_nor_write(uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;
    uint32_t remain_size;
    uint32_t wr_len;
    uint32_t r;

    remain_size = len;
    do
    {
        wr_len = 0;
        if (IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)buf, remain_size))
        {
            wr_len = DMA_1M_LEN - ((uint32_t)buf & DMA_1M_BOUNDARY_MASK);
        }
        else
        {
            wr_len = remain_size;
        }
        r = rt_flash_write(ctx->start_addr + ctx->wr_pos + len - remain_size, buf, wr_len);
        if (r < wr_len)
        {
            return (len - remain_size + r);

        }
        buf += wr_len;
        remain_size -= wr_len;
    }
    while (remain_size);

    return len;
}

static size_t coredump_nor_erase(uint32_t offset, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if ((offset + len) > ctx->total_size)
    {
        return 0;
    }

    if (RT_EOK == rt_flash_erase(ctx->start_addr + offset, len))
    {
        return len;
    }
    else
    {
        return 0;
    }
}

static size_t coredump_nor_read(uint32_t offset, uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if ((offset + len) > ctx->total_size)
    {
        return 0;
    }

    return rt_flash_read(ctx->start_addr + offset, buf, len);
}


static size_t coredump_nand_write(uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    return rt_nand_write(ctx->start_addr + ctx->wr_pos, buf, len);
}

static size_t coredump_nand_erase(uint32_t offset, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if ((offset + len) > ctx->total_size)
    {
        return 0;
    }

    if (RT_EOK == rt_nand_erase(ctx->start_addr + offset, len))
    {
        return len;
    }
    else
    {
        return 0;
    }
}

static size_t coredump_nand_read(uint32_t offset, uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if ((offset + len) > ctx->total_size)
    {
        return 0;
    }

    return rt_nand_read(ctx->start_addr + offset, buf, len);
}
#endif /* BSP_USING_SPI_FLASH */

static size_t coredump_sdmmc_write(uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;
    rt_off_t    pos;
    rt_size_t   size;

    pos = ctx->wr_pos / ctx->cache_buf_size;
    size = len / ctx->cache_buf_size;
    return rt_device_write(ctx->device, pos, buf, size) * ctx->cache_buf_size;
}

static size_t coredump_sdmmc_read(uint32_t offset, uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;
    rt_off_t    pos;
    rt_size_t   size;

    if ((offset + len) > ctx->total_size)
    {
        return 0;
    }

    pos = offset / ctx->cache_buf_size;
    size = len / ctx->cache_buf_size;

    return rt_device_read(ctx->device, pos, buf, size) * ctx->cache_buf_size;
}


static void coredump_get_part_and_dev(const char *part_name, const struct fal_partition **fal_part,
                                      const struct fal_flash_dev **fal_dev)
{
    RT_ASSERT(part_name && fal_part && fal_dev);
    *fal_part = fal_partition_find(part_name);
    if (!*fal_part)
    {
        return;
    }
    *fal_dev = fal_flash_device_find((*fal_part)->flash_name);
}


static coredump_err_code_t coredump_backend_partition_init(coredump_type_t coredump_type)
{
    const char *part_name;
    const struct fal_partition *fal_part;
    const struct fal_flash_dev *fal_dev;
    uint32_t erase_size;
    coredump_backend_partition_ctx_t *ctx;

    if (COREDUMP_TYPE_MINIMUM == coredump_type)
    {
#ifdef COREDUMP_MINIDUMP_ENABLED
        part_name = COREDUMP_MINIDUMP_PART_NAME;
        ctx = &minidump_partition_backend_ctx;
#else
        return COREDUMP_ERR_INVALID_PARAM;
#endif /* COREDUMP_MINIDUMP_ENABLED */
    }
    else
    {
#ifdef COREDUMP_BACKEND_PARTITION
        part_name = COREDUMP_PARTITION_NAME;
        ctx = &fulldump_partition_backend_ctx;
#else
        return COREDUMP_ERR_INVALID_PARAM;
#endif /* COREDUMP_BACKEND_PARTITION */
    }

    if (COREDUMP_BACKEND_STATE_BUSY == ctx->state)
    {
        return COREDUMP_ERR_BUSY;
    }

    memset((void *)ctx, 0, sizeof(*ctx));
    coredump_get_part_and_dev(part_name, &fal_part, &fal_dev);
    if (!fal_part || !fal_dev)
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }

    ctx->device = rt_device_find(fal_part->name);
    ctx->fal_dev = fal_dev;
    if (ctx->device == NULL)
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }
#ifdef BSP_USING_SPI_FLASH
    if (0 == fal_dev->nand_flag)
    {
        ctx->write = coredump_nor_write;
        ctx->erase = coredump_nor_erase;
        ctx->read = coredump_nor_read;
    }
    else if (1 == fal_dev->nand_flag)
    {
        ctx->write = coredump_nand_write;
        ctx->erase = coredump_nand_erase;
        ctx->read = coredump_nand_read;
        ctx->cache_buf = partition_cache_buf;
        ctx->cache_data_size = 0;
        ctx->cache_buf_size = fal_dev->sector_size;
    }
#else
    if (0)
    {

    }
#endif /* BSP_USING_SPI_FLASH */
    else if (2 == fal_dev->nand_flag)
    {
        ctx->write = coredump_sdmmc_write;
        ctx->read = coredump_sdmmc_read;
        ctx->cache_buf = partition_cache_buf;
        ctx->cache_data_size = 0;
        ctx->cache_buf_size = fal_dev->sector_size;
    }
    else
    {
        return COREDUMP_ERR_INVALID_PART_TYPE;
    }

    RT_ASSERT(ctx->cache_buf_size <= COREDUMP_CACHE_BUF_SIZE);

    ctx->start_addr = fal_dev->addr + fal_part->offset;
    /* align to block boundary */
    ctx->total_size = RT_ALIGN_DOWN(fal_part->len, fal_dev->blk_size);
    ctx->state = COREDUMP_BACKEND_STATE_IDLE;

    return COREDUMP_ERR_NO;
}

static coredump_err_code_t coredump_backend_partition_start(void)
{
    uint32_t erase_size;
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if (COREDUMP_BACKEND_STATE_INVALID == ctx->state)
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }
    if (COREDUMP_BACKEND_STATE_BUSY == ctx->state)
    {
        return COREDUMP_ERR_BUSY;
    }

    if (ctx->erase)
    {
        if (ctx->total_size != ctx->erase(0, ctx->total_size))
        {
            return COREDUMP_ERR_ERASE_FAILED;
        }
    }

// TODO: SDIO is not ready yet
#if defined(BSP_USING_SDIO)
    // rt_mmcsd_irq_disable();
#endif /* BSP_USING_SDIO */

    rt_flash_enable_lock(0);

    ctx->wr_pos = 0;
    ctx->cache_data_size = 0;
    ctx->state = COREDUMP_BACKEND_STATE_BUSY;

    return COREDUMP_ERR_NO;
}

static void coredump_backend_partition_end(void)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;
    size_t written;

    if (ctx->cache_buf && (ctx->cache_data_size > 0))
    {
        if (ctx->wr_pos >= ctx->cache_data_size)
        {
            /* jump back to the actual write position */
            ctx->wr_pos -= ctx->cache_data_size;
            written = ctx->write(ctx->cache_buf, ctx->cache_buf_size);
            ctx->wr_pos += written;
        }
        ctx->cache_data_size = 0;
    }
    ctx->state = COREDUMP_BACKEND_STATE_IDLE;
}

static size_t coredump_backend_partition_write(uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;
    size_t remaining_size;
    size_t total_written = 0;
    uint16_t copy_len;
    size_t direct_write_len;
    size_t written;

    /* Calculate remaining space in partition */
    if (ctx->wr_pos >= ctx->total_size)
    {
        return 0;
    }
    remaining_size = ctx->total_size - ctx->wr_pos;
    if (len > remaining_size)
    {
        len = remaining_size;
    }

    if (ctx->cache_buf)
    {
        /* Step 1: Fill existing cache if non-empty */
        if (ctx->cache_data_size > 0)
        {
            /* Calculate space available in cache */
            copy_len = ctx->cache_buf_size - ctx->cache_data_size;
            if (copy_len > len)
            {
                copy_len = len;
            }
            /* Copy data to cache and update wr_pos immediately */
            memcpy(ctx->cache_buf + ctx->cache_data_size, buf, copy_len);
            ctx->cache_data_size += copy_len;
            ctx->wr_pos += copy_len;
            buf += copy_len;
            len -= copy_len;
            total_written += copy_len;

            /* Flush cache if full */
            if (ctx->cache_data_size >= ctx->cache_buf_size)
            {
                /* jump back to the actual position for write */
                ctx->wr_pos -= ctx->cache_buf_size;
                written = ctx->write(ctx->cache_buf, ctx->cache_buf_size);
                ctx->wr_pos += written;
                ctx->cache_data_size = 0;
                if (written != ctx->cache_buf_size)
                {
                    return total_written - ctx->cache_buf_size + written;
                }
            }
        }

        /* Step 2: Process remaining data */
        if (len > 0)
        {
            /* If length is aligned, write directly */
            if ((len % ctx->cache_buf_size) == 0)
            {
                written = ctx->write(buf, len);
                total_written += written;
                ctx->wr_pos += written;
            }
            else
            {
                /* Write aligned portion directly */
                direct_write_len = (len / ctx->cache_buf_size) * ctx->cache_buf_size;
                if (direct_write_len > 0)
                {
                    written = ctx->write(buf, direct_write_len);
                    total_written += written;
                    ctx->wr_pos += written;
                    buf += direct_write_len;
                    if (written != direct_write_len)
                    {
                        return total_written;
                    }
                }
                /* Store unaligned remainder in cache and update wr_pos immediately */
                copy_len = len - direct_write_len;
                memcpy(ctx->cache_buf, buf, copy_len);
                ctx->cache_data_size = copy_len;
                ctx->wr_pos += copy_len;
                total_written += copy_len;
            }
        }
    }
    else
    {
        written = ctx->write(buf, len);
        total_written = written;
        ctx->wr_pos += written;
    }

    return total_written;
}

static size_t coredump_backend_partition_read(uint32_t offset, uint8_t *buf, size_t len)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if (ctx->state != COREDUMP_BACKEND_STATE_IDLE)
    {
        return 0;
    }

    return ctx->read(offset, buf, len);
}

static int32_t coredump_backend_partition_query(coredump_query_id_t id, void *arg)
{
    int32_t r;
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    switch (id)
    {
    case COREDUMP_QUERY_MAX_SIZE:
    {
        if (COREDUMP_BACKEND_STATE_INVALID == ctx->state)
        {
            r = -COREDUMP_ERR_BACKEND_NOT_READY;
        }
        else
        {
            r = ctx->total_size;
        }
        break;
    }
    case COREDUMP_QUERY_REMAIN_SIZE:
    {
        if (ctx->wr_pos >= ctx->total_size)
        {
            r = 0;
        }
        else
        {
            r = ctx->total_size - ctx->wr_pos;
        }
        break;
    }
    case COREDUMP_QUERY_PATH:
    {
        if (ctx->state == COREDUMP_BACKEND_STATE_INVALID)
        {
            r = 0;
        }
        else
        {
            r = (int32_t)ctx->start_addr;
        }
        break;
    }
    case COREDUMP_QUERY_DATA_PRESENT:
    {
        if (ctx->state == COREDUMP_BACKEND_STATE_INVALID)
        {
            r = -COREDUMP_ERR_BACKEND_NOT_READY;
        }
        else
        {
            size_t rd_size;
            uint8_t *buf;
            uint8_t header[4];

            if (ctx->cache_buf)
            {
                RT_ASSERT(0 == ctx->cache_data_size);
                rd_size = ctx->cache_buf_size;
                buf = ctx->cache_buf;
            }
            else
            {
                rd_size = sizeof(header);
                buf = &header[0];
            }
            if (rd_size != ctx->read(0, buf, rd_size))
            {
                r = -COREDUMP_ERR_READ_FAILED;
            }
            else if (buf[0] == '5' && buf[2] == 'X' && buf[3] == '_')
            {
                r = ctx->total_size;
            }
            else
            {
                r = 0;
            }
        }
        break;
    }
    default:
    {
        r = -COREDUMP_ERR_INVALID_PARAM;
        break;
    }
    }

    return r;
}

static coredump_err_code_t coredump_backend_partition_clear(void)
{
    coredump_backend_partition_ctx_t *ctx = partition_backend_ctx;

    if (COREDUMP_BACKEND_STATE_IDLE != ctx->state)
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }

    if (ctx->erase)
    {
        if (ctx->total_size != ctx->erase(0, ctx->total_size))
        {
            return COREDUMP_ERR_ERASE_FAILED;
        }
    }

    return COREDUMP_ERR_NO;
}

coredump_err_code_t coredump_backend_partition_set_mode(coredump_type_t coredump_type)
{
    coredump_err_code_t r;

    r = COREDUMP_ERR_NO;
    if (COREDUMP_TYPE_FULL == coredump_type)
    {
        if (COREDUMP_BACKEND_STATE_IDLE != fulldump_partition_backend_ctx.state)
        {
            r = COREDUMP_ERR_BACKEND_NOT_READY;
        }
        else
        {
            partition_backend_ctx = &fulldump_partition_backend_ctx;
        }
    }
#ifdef COREDUMP_MINIDUMP_ENABLED
    else if (COREDUMP_TYPE_MINIMUM == coredump_type)
    {
        if (COREDUMP_BACKEND_STATE_IDLE != minidump_partition_backend_ctx.state)
        {
            r = COREDUMP_ERR_BACKEND_NOT_READY;
        }
        else
        {
            partition_backend_ctx = &minidump_partition_backend_ctx;
        }
    }
#endif /* COREDUMP_MINIDUMP_ENABLED */
    else
    {
        r = COREDUMP_ERR_INVALID_PARAM;
    }

    return r;
}

const coredump_backend_t coredump_backend_partition =
{
    .init = coredump_backend_partition_init,
    .start = coredump_backend_partition_start,
    .end = coredump_backend_partition_end,
    .write = coredump_backend_partition_write,
    .query = coredump_backend_partition_query,
    .clear = coredump_backend_partition_clear,
    .read = coredump_backend_partition_read,
    .set_mode = coredump_backend_partition_set_mode
};

