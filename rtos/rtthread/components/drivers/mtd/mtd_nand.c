/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-05     Bernard      the first version
 */

/*
 * COPYRIGHT (C) 2012, Shanghai Real Thread
 */

#include <drivers/mtd_nand.h>
#include "mem_map.h"
#include <string.h>

#ifdef RT_USING_MTD_NAND

/**
 * RT-Thread Generic Device Interface
 */

#ifndef RT_NAND_FS_BASE_SEC
    #define RT_NAND_FS_BASE_SEC          (0)
#endif  // RT_NAND_FS_BASE_SEC

#ifndef NAND_SECT_SIZE
    #define NAND_SECT_SIZE                 (2048)
#endif  //NAND_SECT_SIZE



static rt_err_t _mtd_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t _mtd_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t _mtd_close(rt_device_t dev)
{
    return RT_EOK;
}

static void _mtd_erase(rt_device_t dev, rt_off_t pos, rt_size_t size)
{
    struct rt_mtd_nand_device *nand = (struct rt_mtd_nand_device *)dev;

    if (nand)
    {
        // get block, first page in block
        rt_uint32_t blk = pos / (nand->pages_per_block);    // page to block
        if (nand->cache_buf)
        {
            if (blk != nand->blk_cnum)  // block switch, flush old block and read new block
            {
                int i;
                // write to flash first
                if (nand->blk_cnum != 0xffffffff)
                {
                    nand->ops->erase_block(nand, nand->blk_cnum);

                    for (i = 0; i < nand->pages_per_block; i++)
                        nand->ops->write_page(nand, nand->blk_cnum * nand->pages_per_block + i,
                                              nand->cache_buf + i * nand->page_size, nand->page_size, NULL, 0);
                }
                // load new block to cache
                for (i = 0; i < nand->pages_per_block; i++)
                    nand->ops->read_page(nand, blk * nand->pages_per_block + i,
                                         nand->cache_buf + i * nand->page_size, nand->page_size, NULL, 0);
                //rt_kprintf("erase switch block %d : %d, page %d\n",nand->blk_cnum, blk, pos);
                nand->blk_cnum = blk;
            }
            rt_uint32_t pg = pos % nand->pages_per_block;
            //int i;
            //for(i=0; i<nand->page_size; i++)
            memset((nand->cache_buf + pg * nand->page_size), 0xff, nand->page_size);
        }
        else
        {
            // erase block
            nand->ops->erase_block(nand, blk);
        }
    }
}

static rt_size_t _mtd_read(rt_device_t dev,
                           rt_off_t    pos,
                           void       *buffer,
                           rt_size_t   size)
{
    struct rt_mtd_nand_device *nand = RT_MTD_NAND_DEVICE(dev);
    if (nand && nand->ops && nand->ops->read_page)
    {
        // get block, first page in block
        rt_uint32_t blk = pos / (nand->pages_per_block);    // page to block
        if (nand->cache_buf)
        {
            if (blk != nand->blk_cnum)  // block switch, read new block
            {
                int i;
                // write to flash first
                if (nand->blk_cnum != 0xffffffff)
                {
                    nand->ops->erase_block(nand, nand->blk_cnum);

                    for (i = 0; i < nand->pages_per_block; i++)
                        nand->ops->write_page(nand, nand->blk_cnum * nand->pages_per_block + i,
                                              nand->cache_buf + i * nand->page_size, nand->page_size, NULL, 0);
                }
                // load new block to cache
                for (i = 0; i < nand->pages_per_block; i++)
                    nand->ops->read_page(nand, blk * nand->pages_per_block + i,
                                         nand->cache_buf + i * nand->page_size, nand->page_size, NULL, 0);
                //rt_kprintf("read switch block %d : %d, page %d\n",nand->blk_cnum, blk, pos);
                nand->blk_cnum = blk;
            }
            rt_uint32_t pg = pos % nand->pages_per_block;
            memcpy(buffer, (const void *)(nand->cache_buf + pg * nand->page_size), (size_t)(size * nand->page_size));
        }
        else
        {
            // todo , size counter by byte or page?
            // use page size instead sector
            rt_err_t res = nand->ops->read_page(nand, pos, buffer, size * NAND_SECT_SIZE, NULL, 0);
            //rt_kprintf("mtd_read with pos %d, res %d\n",pos, res);
            if (res != RT_EOK)
                return 0;
        }
    }

    return size;
}

static rt_size_t _mtd_write(rt_device_t dev,
                            rt_off_t    pos,
                            const void *buffer,
                            rt_size_t   size)
{
    struct rt_mtd_nand_device *nand = RT_MTD_NAND_DEVICE(dev);
    if (nand && nand->ops && nand->ops->write_page)
    {
        // get block, first page in block
        rt_uint32_t blk = pos / (nand->pages_per_block);    // page to block
        if (nand->cache_buf)
        {
            if (blk != nand->blk_cnum)  // block switch, read new block
            {
                int i;
                //rt_kprintf("_mtd_write %d, %d\n", pos, size);
                // write to flash first
                if (nand->blk_cnum != 0xffffffff)
                {
                    nand->ops->erase_block(nand, nand->blk_cnum);

                    for (i = 0; i < nand->pages_per_block; i++)
                        nand->ops->write_page(nand, nand->blk_cnum * nand->pages_per_block + i,
                                              nand->cache_buf + i * nand->page_size, nand->page_size, NULL, 0);
                }
                // load new block to cache
                for (i = 0; i < nand->pages_per_block; i++)
                    nand->ops->read_page(nand, blk * nand->pages_per_block + i,
                                         nand->cache_buf + i * nand->page_size, nand->page_size, NULL, 0);
                //rt_kprintf("write switch block %d : %d , page %d\n",nand->blk_cnum, blk, pos);
                nand->blk_cnum = blk;
            }
            rt_uint32_t pg = pos % nand->pages_per_block;
            memcpy(nand->cache_buf + pg * nand->page_size, buffer, size * nand->page_size);
        }
        else
        {
            // size counter by page
            _mtd_erase(dev, pos, size);
            rt_err_t res = nand->ops->write_page(nand, pos, buffer, size * nand->page_size, NULL, 0);
            if (res != RT_EOK)
                return 0;
        }
    }

    return size;
}

static rt_err_t _mtd_control(rt_device_t dev, int cmd, void *args)
{
    if (dev == NULL)
    {
        return RT_EINVAL;
    }
    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry = (struct rt_device_blk_geometry *)args;
        if (args == NULL)
        {
            return RT_EINVAL;
        }
        struct rt_mtd_nand_device *ndev = (struct rt_mtd_nand_device *)dev;

        geometry->block_size = ndev->pages_per_block * ndev->page_size;
        geometry->bytes_per_sector = ndev->page_size;
        geometry->sector_count = ndev->block_total * ndev->pages_per_block - RT_NAND_FS_BASE_SEC;
    }
    else if (cmd == RT_DEVICE_CTRL_BLK_ERASE)
    {
        rt_uint32_t *param = (rt_uint32_t *)args;
        struct rt_mtd_nand_device *ndev = (struct rt_mtd_nand_device *)dev;
        //rt_kprintf("mtd erase block %d\n", param[1]);
        return rt_mtd_nand_erase_block(ndev, param[0]);
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops mtd_nand_ops =
{
    _mtd_init,
    _mtd_open,
    _mtd_close,
    _mtd_read,
    _mtd_write,
    _mtd_control
};
#endif

rt_err_t rt_mtd_nand_register_device(const char                *name,
                                     struct rt_mtd_nand_device *device)
{
    rt_device_t dev;

    dev = RT_DEVICE(device);
    RT_ASSERT(dev != RT_NULL);

    /* set device class and generic device interface */
    dev->type        = RT_Device_Class_MTD;
#ifdef RT_USING_DEVICE_OPS
    dev->ops         = &mtd_nand_ops;
#else
    dev->init        = _mtd_init;
    dev->open        = _mtd_open;
    dev->read        = _mtd_read;
    dev->write       = _mtd_write;
    dev->close       = _mtd_close;
    dev->control     = _mtd_control;
#endif

    dev->rx_indicate = RT_NULL;
    dev->tx_complete = RT_NULL;

    /* register to RT-Thread device system */
    return rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}

#if defined(RT_MTD_NAND_DEBUG) && defined(RT_USING_FINSH)
#include <finsh.h>
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')

static void mtd_dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;
    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%06x: ", i);
        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02x ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");
        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}

int mtd_nandid(const char *name)
{
    struct rt_mtd_nand_device *nand;
    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    return rt_mtd_nand_read_id(nand);
}
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nandid, nand_id, read ID - nandid(name));

int mtd_nand_read(const char *name, int block, int page)
{
    rt_err_t result;
    rt_uint8_t *page_ptr;
    rt_uint8_t *oob_ptr;
    struct rt_mtd_nand_device *nand;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    page_ptr = rt_malloc(nand->page_size + nand->oob_size);
    if (page_ptr == RT_NULL)
    {
        rt_kprintf("out of memory!\n");
        return -RT_ENOMEM;
    }

    oob_ptr = page_ptr + nand->page_size;
    rt_memset(page_ptr, 0xff, nand->page_size + nand->oob_size);

    /* calculate the page number */
    page = block * nand->pages_per_block + page;
    result = rt_mtd_nand_read(nand, page, page_ptr, nand->page_size,
                              oob_ptr, nand->oob_size);

    rt_kprintf("read page, rc=%d\n", result);
    mtd_dump_hex(page_ptr, nand->page_size);
    mtd_dump_hex(oob_ptr, nand->oob_size);

    rt_free(page_ptr);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_read, nand_read, read page in nand - nand_read(name, block, page));

int mtd_nand_readoob(const char *name, int block, int page)
{
    struct rt_mtd_nand_device *nand;
    rt_uint8_t *oob_ptr;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    oob_ptr = rt_malloc(nand->oob_size);
    if (oob_ptr == RT_NULL)
    {
        rt_kprintf("out of memory!\n");
        return -RT_ENOMEM;
    }

    /* calculate the page number */
    page = block * nand->pages_per_block + page;
    rt_mtd_nand_read(nand, page, RT_NULL, nand->page_size,
                     oob_ptr, nand->oob_size);
    mtd_dump_hex(oob_ptr, nand->oob_size);

    rt_free(oob_ptr);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_readoob, nand_readoob, read spare data in nand - nand_readoob(name, block, page));

int mtd_nand_write(const char *name, int block, int page)
{
    rt_err_t result;
    rt_uint8_t *page_ptr;
    rt_uint8_t *oob_ptr;
    rt_uint32_t index;
    struct rt_mtd_nand_device *nand;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    page_ptr = rt_malloc(nand->page_size + nand->oob_size);
    if (page_ptr == RT_NULL)
    {
        rt_kprintf("out of memory!\n");
        return -RT_ENOMEM;
    }

    oob_ptr = page_ptr + nand->page_size;
    /* prepare page data */
    for (index = 0; index < nand->page_size; index ++)
    {
        page_ptr[index] = index & 0xff;
    }
    /* prepare oob data */
    for (index = 0; index < nand->oob_size; index ++)
    {
        oob_ptr[index] = index & 0xff;
    }

    /* calculate the page number */
    page = block * nand->pages_per_block + page;
    result = rt_mtd_nand_write(nand, page, page_ptr, nand->page_size,
                               oob_ptr, nand->oob_size);
    if (result != RT_MTD_EOK)
    {
        rt_kprintf("write page failed!, rc=%d\n", result);
    }

    rt_free(page_ptr);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_write, nand_write, write dump data to nand - nand_write(name, block, page));

int mtd_nand_erase(const char *name, int block)
{
    struct rt_mtd_nand_device *nand;
    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    return rt_mtd_nand_erase_block(nand, block);
}
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_erase, nand_erase, nand_erase(name, block));

int mtd_nand_erase_all(const char *name)
{
    rt_uint32_t index = 0;
    struct rt_mtd_nand_device *nand;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    for (index = 0; index < (nand->block_end - nand->block_start); index ++)
    {
        rt_mtd_nand_erase_block(nand, index);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_erase_all, nand_erase_all, erase all of nand device - nand_erase_all(name, block));
#endif

#endif
