/**
  ******************************************************************************
  * @file   file_logger.c
  * @author Sifli software development team
  * @brief Metrics Collector source
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

#include <rtthread.h>
#include "board.h"
#include <string.h>
#include "file_logger.h"

#include "dfs_posix.h"
#include "log.h"


#define FL_FILE_HDR_MAGIC  (0x5354454D)   //METS
#define FL_IS_VALID_FILE_HDR(hdr)  ((hdr)->magic == FL_FILE_HDR_MAGIC)
#define FL_FILE_HDR_SIZE  (sizeof(fl_file_hdr_t))
#define FL_FILE_OFFSET(data_pos)   (FL_FILE_HDR_SIZE + (data_pos))

#define FL_PACKET_HDR_MAGIC  (0x4448) //HD
#define FL_PACKET_TAIL_MAGIC (0x4154) //TA

#define FL_PACKET_LEN(data_len)    ((data_len) + sizeof(fl_packet_hdr_t) + sizeof(fl_packet_tail_t))

typedef struct
{
    int fd;
    fl_iter_cb_t cb;
    void *arg;
} fl_iter_arg_t;

typedef struct
{
    uint32_t magic;
    int32_t wr_pos;
} fl_file_hdr_t;

typedef struct
{
    uint16_t magic;
    uint16_t len;
} fl_packet_hdr_t;

typedef struct
{
    uint16_t magic;
} fl_packet_tail_t;

/*
typedef struct
{
    fl_packet_hdr_t hdr;
    uint8_t data[len]
    fl_packet_tail_t tail;
} fl_packet_t;
*/

typedef struct
{
    const char *name;
    int fd;
    uint32_t max_size;
    uint32_t used_size;
    /* true: reach the file end and go the beginning */
    bool turnaround;
    fl_file_hdr_t cursor;
} fl_handle_t;


static fl_err_t fl_write_file(fl_handle_t *handle, void *data, uint32_t data_len)
{
    fl_err_t err = FL_OK;
    uint32_t remaining_len;
    int wr_size;

    remaining_len = handle->max_size - handle->cursor.wr_pos;

    if (remaining_len <= data_len)
    {
        wr_size = write(handle->fd, data, remaining_len);
        if (wr_size != remaining_len)
        {
            err = FL_WRITE_ERR;
            goto __EXIT;
        }
        handle->used_size = handle->max_size;
        handle->cursor.wr_pos = 0;
        wr_size = lseek(handle->fd, FL_FILE_HDR_SIZE, SEEK_SET);
        if (wr_size != FL_FILE_HDR_SIZE)
        {
            err = FL_ERROR;
            goto __EXIT;
        }
        data = (uint8_t *)data + remaining_len;
        data_len -= remaining_len;
    }

    if (data_len > 0)
    {
        wr_size = write(handle->fd, data, data_len);
        if (wr_size != data_len)
        {
            err = FL_WRITE_ERR;
            goto __EXIT;
        }
        handle->cursor.wr_pos += data_len;
        RT_ASSERT(handle->cursor.wr_pos < handle->max_size);
    }

    if (handle->cursor.wr_pos > handle->used_size)
    {
        handle->used_size = handle->cursor.wr_pos;
    }

__EXIT:

    return err;
}

static fl_err_t fl_read_file(fl_handle_t *handle, uint32_t *pos, void *data, uint32_t data_len)
{
    fl_err_t err = FL_OK;
    uint32_t remaining_len;
    uint32_t avail_len;
    int rd_size;

    RT_ASSERT(pos);
//    LOG_I("rd:%d,%d", *pos, data_len);

    if (*pos >= handle->used_size)
    {
        err = FL_READ_ERR;
        goto __EXIT;
    }

    if (handle->used_size == handle->max_size)
    {
        if (*pos >= handle->cursor.wr_pos)
        {
            if (!handle->turnaround)
            {
                RT_ASSERT(handle->max_size > *pos);
                remaining_len = handle->max_size - *pos;
                avail_len = handle->max_size - *pos + handle->cursor.wr_pos;
            }
            else
            {
                err = FL_READ_ERR;
                goto __EXIT;
            }
        }
        else
        {
            remaining_len = handle->cursor.wr_pos - *pos;
            avail_len = remaining_len;
        }
    }
    else if (*pos < handle->cursor.wr_pos)
    {
        remaining_len = handle->cursor.wr_pos - *pos;
        avail_len = remaining_len;
    }
    else
    {
        remaining_len = 0;
        avail_len = 0;
    }

    if (avail_len < data_len)
    {
        err = FL_READ_ERR;
        goto __EXIT;
    }

    rd_size = lseek(handle->fd, FL_FILE_HDR_SIZE + *pos, SEEK_SET);
    if (rd_size != (FL_FILE_HDR_SIZE + *pos))
    {
        err = FL_READ_ERR;
        goto __EXIT;
    }

    if (remaining_len <= data_len)
    {
        rd_size = read(handle->fd, data, remaining_len);
        if (rd_size != remaining_len)
        {
            err = FL_READ_ERR;
            goto __EXIT;
        }
        *pos += remaining_len;
        data = (uint8_t *)data + remaining_len;
        data_len -= remaining_len;
        if (*pos >= handle->max_size)
        {
            /* Reach file end */
            *pos = 0;
            handle->turnaround = true;
            rd_size = lseek(handle->fd, FL_FILE_HDR_SIZE, SEEK_SET);
            if (rd_size != FL_FILE_HDR_SIZE)
            {
                err = FL_ERROR;
                goto __EXIT;
            }
        }
    }

    if (data_len > 0)
    {
        rd_size = read(handle->fd, data, data_len);
        if (rd_size != data_len)
        {
            err = FL_READ_ERR;
            goto __EXIT;
        }
        *pos += data_len;
        RT_ASSERT(*pos < handle->used_size);
    }

__EXIT:

    return err;
}

static fl_err_t fl_find_packet(fl_handle_t *handle, uint32_t *pos, fl_packet_hdr_t *hdr)
{
    fl_err_t err;
    uint32_t tail_pos;
    uint32_t pos_bak;
    fl_packet_tail_t tail;

    RT_ASSERT(hdr && pos);

__TRY_AGAIN:
    do
    {
        /* search for packet header first byte */
        do
        {
            err = fl_read_file(handle, pos, &hdr->magic, 1);
            if (FL_OK != err)
            {
                goto __EXIT;
            }
        }
        while ((hdr->magic & 0XFF) != (FL_PACKET_HDR_MAGIC & 0xFF));

        pos_bak = *pos;
        err = fl_read_file(handle, pos, (uint8_t *)&hdr->magic + 1, sizeof(hdr) - 1);
        if (FL_OK != err)
        {
            goto __EXIT;
        }

        if (hdr->magic == FL_PACKET_HDR_MAGIC)
        {
            break;
        }
        else
        {
            *pos = pos_bak;
        }
    }
    while (1);

    /* check tail magic */
    tail_pos = *pos + hdr->len;
    if (tail_pos >= handle->max_size)
    {
        tail_pos -= handle->max_size;
    }
    err = fl_read_file(handle, &tail_pos, &tail, sizeof(tail));

    if (FL_OK != err)
    {
        goto __EXIT;
    }

    if (tail.magic != FL_PACKET_TAIL_MAGIC)
    {
        goto __TRY_AGAIN;
    }

__EXIT:
    return err;
}


void *file_logger_init(const char *name, uint32_t max_size)
{
    int fd = -1;
    fl_handle_t *handle = NULL;
    bool is_new;
    int rw_size;
    bool bad_file;

    if (max_size <= FL_FILE_HDR_SIZE)
    {
        LOG_W("wrong max_size");
        goto __ERROR;
    }

    if (0 != access(name, 0))
    {
        fd = open(name, O_CREAT | O_RDWR | O_BINARY);
        is_new = true;
    }
    else
    {
        fd = open(name, O_RDWR | O_BINARY);
        is_new = false;
    }

    if (fd < 0)
    {
        LOG_W("open %s fail", name);
        goto __ERROR;
    }

    handle = rt_malloc(sizeof(*handle));
    RT_ASSERT(handle);

    handle->fd = fd;
    handle->name = name;
    handle->max_size = max_size - FL_FILE_HDR_SIZE;
    if (is_new)
    {
        handle->used_size = 0;
        handle->cursor.wr_pos = 0;
        handle->cursor.magic = FL_FILE_HDR_MAGIC;
        rw_size = write(fd, &handle->cursor, FL_FILE_HDR_SIZE);
        if (rw_size != FL_FILE_HDR_SIZE)
        {
            LOG_W("write file hdr error");
            goto __ERROR;
        }
    }
    else
    {
        handle->used_size = lseek(fd, 0, SEEK_END);
        bad_file = true;
        do
        {
            if (handle->used_size < FL_FILE_HDR_SIZE)
            {
                LOG_W("invalid header");
                break;
            }
            handle->used_size -= FL_FILE_HDR_SIZE;
            if (handle->used_size > handle->max_size)
            {
                LOG_W("max size changes, clean data");
                break;
            }
            lseek(fd, 0, SEEK_SET);
            rw_size = read(fd, &handle->cursor, sizeof(handle->cursor));
            if (rw_size < sizeof(handle->cursor))
            {
                LOG_W("invalid header");
                break;
            }

            if (!FL_IS_VALID_FILE_HDR(&handle->cursor))
            {
                LOG_W("header magic error");
                break;
            }
            rw_size = lseek(fd, handle->cursor.wr_pos, SEEK_SET);
            if (rw_size != handle->cursor.wr_pos)
            {
                LOG_W("Fail to move cursor");
                break;
            }
            bad_file = false;
        }
        while (0);

        if (bad_file)
        {
            close(fd);
            fd = open(name, O_CREAT | O_RDWR | O_TRUNC | O_BINARY);
            if (fd < 0)
            {
                goto __ERROR;
            }
            handle->used_size = 0;
            handle->cursor.wr_pos = 0;
            handle->cursor.magic = FL_FILE_HDR_MAGIC;
            rw_size = write(fd, &handle->cursor, FL_FILE_HDR_SIZE);
            if (rw_size != FL_FILE_HDR_SIZE)
            {
                LOG_W("write file hdr error");
                goto __ERROR;
            }
        }
    }

    return (void *)handle;

__ERROR:
    if (fd >= 0)
    {
        close(fd);
    }

    if (handle)
    {
        rt_free(handle);
    }

    return NULL;
}

fl_err_t file_logger_write(void *logger, void *data, uint32_t data_len)
{
    fl_handle_t *handle = (fl_handle_t *)logger;
    int wr_size;
    fl_err_t err = FL_OK;
    uint32_t total_len;
    fl_packet_hdr_t packet_hdr;
    fl_packet_tail_t packet_tail;

    RT_ASSERT(logger && data);

    if (0 == data_len)
    {
        return FL_OK;
    }

    total_len = FL_PACKET_LEN(data_len);
    if (total_len > handle->max_size)
    {
        err = FL_INVALID_DATA_LEN;
        goto __EXIT;
    }

    wr_size = lseek(handle->fd, FL_FILE_HDR_SIZE + handle->cursor.wr_pos, SEEK_SET);
    if (wr_size != (FL_FILE_HDR_SIZE + handle->cursor.wr_pos))
    {
        err = FL_WRITE_ERR;
        goto __EXIT;
    }

    /* write header */
    packet_hdr.magic = FL_PACKET_HDR_MAGIC;
    packet_hdr.len = data_len;
    err = fl_write_file(handle, &packet_hdr, sizeof(packet_hdr));
    if (FL_OK != err)
    {
        goto __EXIT;
    }

    /* write data */
    err = fl_write_file(handle, data, data_len);
    if (FL_OK != err)
    {
        goto __EXIT;
    }

    /* write tail */
    packet_tail.magic = FL_PACKET_TAIL_MAGIC;
    err = fl_write_file(handle, &packet_tail, sizeof(packet_tail));
    if (FL_OK != err)
    {
        goto __EXIT;
    }

__EXIT:

    return err;
}

fl_err_t file_logger_write_noheader(void *logger, void *data, uint32_t data_len)
{
    fl_handle_t *handle = (fl_handle_t *)logger;
    int wr_size;
    fl_err_t err = FL_OK;

    RT_ASSERT(logger && data);

    if (0 == data_len)
    {
        return FL_OK;
    }

    if (data_len > handle->max_size)
    {
        err = FL_INVALID_DATA_LEN;
        goto __EXIT;
    }

    wr_size = lseek(handle->fd, FL_FILE_HDR_SIZE + handle->cursor.wr_pos, SEEK_SET);
    if (wr_size != (FL_FILE_HDR_SIZE + handle->cursor.wr_pos))
    {
        err = FL_WRITE_ERR;
        goto __EXIT;
    }

    /* write data */
    err = fl_write_file(handle, data, data_len);
    if (FL_OK != err)
    {
        goto __EXIT;
    }

__EXIT:

    return err;
}

fl_err_t file_logger_iter(void *logger, fl_iter_cb_t cb, void *arg)
{
    fl_handle_t *handle = (fl_handle_t *)logger;
    int rd_size;
    fl_packet_hdr_t hdr;
    fl_err_t err = FL_OK;
    uint8_t *data = NULL;
    uint32_t rd_pos;

    if (0 == handle->used_size)
    {
        goto __EXIT;
    }

    if (handle->used_size == handle->max_size)
    {
        /* read starting from wr_pos */
        rd_pos = handle->cursor.wr_pos;
    }
    else
    {
        /* read starting from beginning */
        rd_pos = 0;
    }
    handle->turnaround = false;

    rd_size = lseek(handle->fd, FL_FILE_HDR_SIZE, SEEK_SET);
    if (rd_size != FL_FILE_HDR_SIZE)
    {
        goto __EXIT;
    }

    do
    {
        err = fl_find_packet(handle, &rd_pos, &hdr);
        if (FL_OK != err)
        {
            goto __EXIT;
        }

        /* read data */
        data = rt_malloc(hdr.len);
        RT_ASSERT(data);
        err = fl_read_file(handle, &rd_pos, data, hdr.len);
        if (FL_OK != err)
        {
            goto __EXIT;
        }

        cb(data, hdr.len, arg);
        rt_free(data);
        data = NULL;
        /* skip tail */
        rd_pos += sizeof(fl_packet_tail_t);
        if (rd_pos >= handle->max_size)
        {
            rd_pos -= handle->max_size;
        }
    }
    while (1);

__EXIT:
    if (data)
    {
        rt_free(data);
    }

    return err;
}

fl_err_t file_logger_clear(void *logger)
{
    fl_handle_t *handle = (fl_handle_t *)logger;
    int ret;
    int wr_size;

    ret = close(handle->fd);
    if (ret)
    {
        return FL_ERROR;
    }
    handle->fd = -1;

    handle->fd = open(handle->name, O_RDWR | O_TRUNC | O_BINARY);
    handle->cursor.wr_pos = 0;
    handle->cursor.magic = FL_FILE_HDR_MAGIC;
    wr_size = write(handle->fd, &handle->cursor, sizeof(handle->cursor));
    if (wr_size != sizeof(handle->cursor))
    {
        return FL_WRITE_ERR;
    }

    return FL_OK;
}

fl_err_t file_logger_flush(void *logger)
{
    fl_handle_t *handle = (fl_handle_t *)logger;
    int wr_size;
    fl_err_t err = FL_OK;

    /* write file header */
    wr_size = lseek(handle->fd, 0, SEEK_SET);
    if (wr_size != 0)
    {
        err = FL_ERROR;
        goto __EXIT;
    }

    wr_size = write(handle->fd, &handle->cursor, sizeof(handle->cursor));
    if (wr_size != sizeof(handle->cursor))
    {
        err = FL_WRITE_ERR;
        goto __EXIT;
    }

    wr_size = lseek(handle->fd, FL_FILE_HDR_SIZE + handle->cursor.wr_pos, SEEK_SET);
    if (wr_size != (FL_FILE_HDR_SIZE + handle->cursor.wr_pos))
    {
        err = FL_WRITE_ERR;
        goto __EXIT;
    }

    fsync(handle->fd);

__EXIT:

    return err;
}

fl_err_t file_logger_close(void *logger)
{
    fl_handle_t *handle = (fl_handle_t *)logger;
    fl_err_t err;

    err = file_logger_flush(logger);
    if (FL_OK != err)
    {
        goto __EXIT;
    }

    close(handle->fd);

    rt_free(handle);

__EXIT:
    return err;
}


