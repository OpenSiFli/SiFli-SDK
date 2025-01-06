/*
 * Copyright (c) 2020, Armink, <armink.ztl@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <fal.h>
#include <stdio.h>
#if defined(PAGE_SIZE)
    #undef PAGE_SIZE
#endif
#define PAGE_SIZE     8192


static FILE *fp_flash1, *fp_flash2, *fp_flash3, *fp_flash4;
static uint8_t flash_buf[PAGE_SIZE];

void file_expand(FILE *fp, int size)
{
    long pos;
    fseek(fp, 0, SEEK_END);
    pos = ftell(fp);
    while (pos < size)
    {
        fwrite(flash_buf, 1, PAGE_SIZE, fp);
        pos += PAGE_SIZE;
    }
    if (pos != size)
    {
        pos -= PAGE_SIZE;
        fwrite(flash_buf, 1, size - pos, fp);
    }
    fflush(fp);
}
static int init(long base)
{
    struct _stat buf;
    memset(flash_buf, 0xff, PAGE_SIZE);
    if (base == nor_flash1.addr)
    {
        fp_flash1 = fopen("flash1.bin", "rb+");
        if (fp_flash1 == NULL)
            fp_flash1 = fopen("flash1.bin", "wb+");
        if (fp_flash1)
        {
            _fstat(_fileno(fp_flash1), &buf);
            if (buf.st_size != nor_flash1.len)
            {
                file_expand(fp_flash1, nor_flash1.len);
            }
        }
    }
    else if (base == nor_flash2.addr)
    {
        fp_flash2 = fopen("flash2.bin", "rb+");
        if (fp_flash2 == NULL)
            fp_flash2 = fopen("flash2.bin", "wb+");
        if (fp_flash2)
        {
            _fstat(_fileno(fp_flash2), &buf);
            if (buf.st_size != nor_flash2.len)
            {
                file_expand(fp_flash2, nor_flash2.len);
            }
        }
    }
    else if (base == nor_flash3.addr)
    {
        fp_flash3 = fopen("flash3.bin", "rb+");
        if (fp_flash3 == NULL)
            fp_flash3 = fopen("flash3.bin", "wb+");
        if (fp_flash3)
        {
            _fstat(_fileno(fp_flash3), &buf);
            if (buf.st_size != nor_flash3.len)
            {
                file_expand(fp_flash3, nor_flash3.len);
            }
        }
    }
    else if (base == nor_flash4.addr)
    {
        fp_flash4 = fopen("flash4.bin", "rb+");
        if (fp_flash4 == NULL)
            fp_flash4 = fopen("flash4.bin", "wb+");
        if (fp_flash4)
        {
            _fstat(_fileno(fp_flash4), &buf);
            if (buf.st_size != nor_flash4.len)
            {
                file_expand(fp_flash4, nor_flash4.len);
            }
        }
    }
    else
        RT_ASSERT(0);
    return 1;
}


static int ef_err_port_cnt = 0;
static int on_ic_read_cnt  = 0;
static int on_ic_write_cnt = 0;

FILE *get_fp(long base)
{
    FILE *fp;

    if (base == nor_flash1.addr)
    {
        fp = fp_flash1;
    }
    else if (base == nor_flash2.addr)
    {
        fp = fp_flash2;
    }
    else if (base == nor_flash3.addr)
    {
        fp = fp_flash3;
    }
    else
    {
        fp = NULL;
        RT_ASSERT(0);
    }
    return fp;
}

static int read(long base, long offset, uint8_t *buf, size_t size)
{
    FILE *fp = get_fp(base);

    if (offset % 4 != 0)
        ef_err_port_cnt++;
    fseek(fp, offset, SEEK_SET);
    size = fread(buf, 1, size, fp);
    on_ic_read_cnt++;
    return size;
}


static int write(long base, long offset, const uint8_t *buf, size_t size)
{
    FILE *fp = get_fp(base);

    fseek(fp, offset, SEEK_SET);
    size = fwrite(buf, 1, size, fp);
    on_ic_write_cnt++;
    fflush(fp);
    return size;
}


static int erase(long base, long offset, size_t size)
{
    FILE *fp = get_fp(base);

    size_t i, erase_pages;
    uint32_t PAGEError = 0;

    erase_pages = size / PAGE_SIZE;
    if (size % PAGE_SIZE != 0)
    {
        erase_pages++;
    }
    offset &= ~(PAGE_SIZE - 1);
    fseek(fp, offset, SEEK_SET);
    for (i = 0; i < erase_pages; i++)
        fwrite(flash_buf, 1, PAGE_SIZE, fp);
    return size;
}

static int init1(void)
{
    return init(nor_flash1.addr);
}
static int read1(long offset, uint8_t *buf, size_t size)
{
    return read(nor_flash1.addr, offset, buf, size);
}
static int write1(long offset, const uint8_t *buf, size_t size)
{
    return write(nor_flash1.addr, offset, buf, size);
}
static int erase1(long offset, size_t size)
{
    return erase(nor_flash1.addr, offset, size);
}

static int init2(void)
{
    return init(nor_flash2.addr);
}
static int read2(long offset, uint8_t *buf, size_t size)
{
    return read(nor_flash2.addr, offset, buf, size);
}
static int write2(long offset, const uint8_t *buf, size_t size)
{
    return write(nor_flash2.addr, offset, buf, size);
}
static int erase2(long offset, size_t size)
{
    return erase(nor_flash2.addr, offset, size);
}

static int init3(void)
{
    return init(nor_flash3.addr);
}
static int read3(long offset, uint8_t *buf, size_t size)
{
    return read(nor_flash3.addr, offset, buf, size);
}
static int write3(long offset, const uint8_t *buf, size_t size)
{
    return write(nor_flash3.addr, offset, buf, size);
}
static int erase3(long offset, size_t size)
{
    return erase(nor_flash3.addr, offset, size);
}

static int init4(void)
{
    return init(nor_flash4.addr);
}
static int read4(long offset, uint8_t *buf, size_t size)
{
    return read(nor_flash4.addr, offset, buf, size);
}
static int write4(long offset, const uint8_t *buf, size_t size)
{
    return write(nor_flash4.addr, offset, buf, size);
}
static int erase4(long offset, size_t size)
{
    return erase(nor_flash4.addr, offset, size);
}


const struct fal_flash_dev nor_flash1 =
{
    .name       = "flash1",
    .addr       = FLASH_BASE_ADDR,
    .len        = 4 * 1024 * 1024,
    .blk_size   = 8 * 1024,
    .sector_size = 8 * 1024,
    .ops        = {init1, read1, write1, erase1},
    .write_gran = 32
};

const struct fal_flash_dev nor_flash2 =
{
    .name = "flash2",
    .addr = FLASH2_BASE_ADDR,
    .len = 32 * 1024 * 1024,
    .blk_size = 8 * 1024,
    .sector_size = 8 * 1024,
    .ops = {init2, read2, write2, erase2},
    .write_gran = 32
};

const struct fal_flash_dev nor_flash3 =
{
    .name = "flash3",
    .addr = FLASH3_BASE_ADDR,
    .len = 128 * 1024 * 1024,
    .blk_size = 128 * 1024,
    .sector_size = 2048,
    .ops = {init3, read3, write3, erase3},
    .write_gran = 32
};

const struct fal_flash_dev nor_flash4 =
{
    .name = "flash4",
    .addr = FLASH4_BASE_ADDR,
    .len = 128 * 1024 * 1024,
    .blk_size = 128 * 1024,
    .sector_size = 2048,
    .ops = {init4, read4, write4, erase4},
    .write_gran = 32
};


