/*
 * ram I/O
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "avformat.h"
#include "internal.h"
#include "os_support.h"
#include "url.h"

typedef struct RamContext {
    const AVClass *class;
    int64_t img_start;
    int64_t img_size;
    int64_t offset;
} RamContext;

static const AVClass ram_class = {
    .class_name = "ram",
    .item_name  = av_default_item_name,
    .option     = NULL,
    .version    = LIBAVUTIL_VERSION_INT,
};

static int ram_read(URLContext *h, unsigned char *buf, int size)
{
    RamContext *c = h->priv_data;
    int ret;
    if (size < 0)
        return -1;
    if (c->offset + size >= c->img_size)
    {
        size = c->img_size - c->offset;
    }
    if (size == 0)
    {
        return 0;
    }

    memcpy(buf, (uint32_t)(c->img_start + c->offset), size);
    c->offset += size;
    return size;
}

static int ram_open(URLContext *h, const char *filename, int flags)
{
    RamContext *c = h->priv_data;
    uint32_t address;
    uint32_t len;
    int fd;
    if (strncmp("ram://", filename, 6))
    {
        return -1;
    }
    sscanf(filename, FFMPEG_RAM_URL_FMT, &address, &len);
    c->img_start = address;
    c->img_size = len;
    c->offset = 0;
    return 0;
}
static int ram_close(URLContext *h)
{
     return 0;
}
static int64_t ram_seek(URLContext *h, int64_t pos, int whence)
{
    RamContext *c = h->priv_data;
    int64_t ret;

    if (whence == AVSEEK_SIZE) {
        return c->img_size;
    }
    if (whence == SEEK_SET)
    {
        if (pos < 0)
        {
            pos = 0;
        }
        if (pos > c->img_size)
        {
            pos = c->img_size;
        }
        c->offset = pos;
    }
    else if (whence == SEEK_CUR)
    {
        c->offset += pos;
    }
    else if (whence == SEEK_END)
    {
        c->offset = c->img_size + pos;
    }
    if (c->offset > c->img_size)
    {
        c->offset = c->img_size;
    }
    else if (c->offset < 0)
    {
        c->offset = 0;
    }
    return c->offset;
}


URLProtocol ff_ram_protocol = {
    .name                = "ram",
    .url_open            = ram_open,
    .url_read            = ram_read,
    .url_seek            = ram_seek,
    .url_close           = ram_close,
    .priv_data_size      = sizeof(RamContext),
    .priv_data_class     = &ram_class,
    .default_whitelist   = "ram"
};


