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
#include "dfs_posix.h"
#include "drv_flash.h"
#if defined(BSP_USING_SDIO)
    #include "mmcsd_core.h"
#endif /* BSP_USING_SDIO */


#define COREDUMP_PATH_JOIN(dir, file) \
    (strlen(dir) > 0 && (dir[strlen(dir) - 1] == '/' || dir[strlen(dir) - 1] == '\\') ? \
        dir file : dir "/" file)


typedef struct
{
    int fid;
    const char *file_full_path;
    /** total size of the file, unit: bytes */
    size_t total_size;
    /** current write position, unit: bytes */
    uint32_t wr_pos;
    coredump_backend_write_t write;
    coredump_backend_state_t state;
} coredump_backend_file_ctx_t;

static coredump_backend_file_ctx_t file_backend_ctx;

static uint32_t get_free_size(const char *path)
{
    int result;
    struct statfs buffer;
    uint32_t free_size;

    free_size = 0;
    if (!path)
    {
        goto __EXIT;
    }

    result = dfs_statfs(path, &buffer);
    if (result != 0)
    {
        goto __EXIT;
    }


    free_size = buffer.f_bfree * buffer.f_bsize;

__EXIT:
    return free_size;
}

static coredump_err_code_t coredump_backend_file_init(coredump_type_t coredump_type)
{
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;

    if (COREDUMP_BACKEND_STATE_BUSY == ctx->state)
    {
        return COREDUMP_ERR_BUSY;
    }
    if (COREDUMP_TYPE_MINIMUM == coredump_type)
    {
        return COREDUMP_ERR_INVALID_PARAM;
    }

    memset((void *)ctx, 0, sizeof(*ctx));

    if (0 == strlen(COREDUMP_FILE_PATH))
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }
    if ((0 != access(COREDUMP_FILE_PATH, 0)) && (0 != mkdir(COREDUMP_FILE_PATH, 0)))
    {
        rt_kprintf("coredump file path not exist: %s\n", COREDUMP_FILE_PATH);
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }

    ctx->total_size = get_free_size(COREDUMP_FILE_PATH);
    ctx->file_full_path = COREDUMP_PATH_JOIN(COREDUMP_FILE_PATH, COREDUMP_FILE_NAME);
    ctx->state = COREDUMP_BACKEND_STATE_IDLE;

    return COREDUMP_ERR_NO;
}


static coredump_err_code_t coredump_backend_file_start(void)
{
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;

    if (COREDUMP_BACKEND_STATE_INVALID == ctx->state)
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }
    if (COREDUMP_BACKEND_STATE_BUSY == ctx->state)
    {
        return COREDUMP_ERR_BUSY;
    }

    unlink(ctx->file_full_path);
    ctx->total_size = get_free_size(COREDUMP_FILE_PATH);
    if (0 == ctx->total_size)
    {
        return COREDUMP_ERR_NO_SPACE;
    }
    ctx->fid = open(ctx->file_full_path, O_RDWR | O_CREAT | O_TRUNC | O_BINARY, 0);
    if (ctx->fid < 0)
    {
        return COREDUMP_ERR_FILE_CREATE_FAILED;
    }

    //TODO: SDIO is not ready yet
#if defined(BSP_USING_SDIO)
    // rt_mmcsd_irq_disable();
#endif /* BSP_USING_SDIO */

    rt_flash_enable_lock(0);

    ctx->wr_pos = 0;
    ctx->state = COREDUMP_BACKEND_STATE_BUSY;

    return COREDUMP_ERR_NO;
}

static void coredump_backend_file_end(void)
{
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;

    close(ctx->fid);
    ctx->fid = -1;

    ctx->state = COREDUMP_BACKEND_STATE_IDLE;
}

static size_t coredump_backend_file_write(uint8_t *buf, size_t len)
{
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;
    int r;

    r = write(ctx->fid, buf, len);
    if (r < 0)
    {
        return 0;
    }
    ctx->wr_pos += r;

    return (size_t)r;
}

static int32_t coredump_backend_file_query(coredump_query_id_t id, void *arg)
{
    int32_t r;
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;

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
            r = (int32_t)ctx->file_full_path;
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
            int fid = open(ctx->file_full_path, O_RDONLY);
            if (fid >= 0)
            {
                close(fid);
                r = 1;
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

static void coredump_backend_file_sync(void)
{
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;

    fsync(ctx->fid);
}

static coredump_err_code_t coredump_backend_file_clear(void)
{
    coredump_backend_file_ctx_t *ctx = &file_backend_ctx;
    int fid;

    if (COREDUMP_BACKEND_STATE_IDLE != ctx->state)
    {
        return COREDUMP_ERR_BACKEND_NOT_READY;
    }

    fid = open(ctx->file_full_path, O_RDONLY);
    if (fid >= 0)
    {
        close(fid);
        if (0 != unlink(ctx->file_full_path))
        {
            return COREDUMP_ERR_ERASE_FAILED;
        }
    }

    return COREDUMP_ERR_NO;
}

coredump_err_code_t coredump_backend_file_set_mode(coredump_type_t coredump_type)
{
    if (COREDUMP_TYPE_FULL == coredump_type)
    {
        return COREDUMP_ERR_NO;
    }
    else
    {
        return COREDUMP_ERR_INVALID_PARAM;
    }
}

const coredump_backend_t coredump_backend_file =
{
    .init = coredump_backend_file_init,
    .start = coredump_backend_file_start,
    .end = coredump_backend_file_end,
    .write = coredump_backend_file_write,
    .query = coredump_backend_file_query,
    .sync = coredump_backend_file_sync,
    .clear = coredump_backend_file_clear,
    .set_mode = coredump_backend_file_set_mode
};