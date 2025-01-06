/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-5-30     Bernard      the first version
 */

#include <drivers/mtd_nor.h>

#ifdef RT_USING_MTD_NOR

/**
 * RT-Thread Generic Device Interface
 */


#ifndef FLASH_SECT_SIZE
    #define FLASH_SECT_SIZE                 (4096)
#endif  //FLASH_SECT_SIZE

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

// mtd only used in filesystem, read/write/erase should be sector based.
static rt_err_t _mtd_erase(rt_device_t dev, rt_off_t pos, rt_size_t size)
{
    struct rt_mtd_nor_device *ndev = (struct rt_mtd_nor_device *)dev;
    if (ndev && ndev->ops && ndev->ops->erase_block)
    {
        rt_off_t tpos = pos * ndev->sector_size;
        rt_size_t tsize = size * ndev->sector_size;
        return ndev->ops->erase_block(ndev, tpos, tsize);
    }
    return RT_ERROR;
}

static rt_size_t _mtd_read(rt_device_t dev,
                           rt_off_t    pos,
                           void       *buffer,
                           rt_size_t   size)
{
    rt_size_t res = 0;
    struct rt_mtd_nor_device *ndev = (struct rt_mtd_nor_device *)dev;
    if (ndev && ndev->ops && ndev->ops->read)
    {
        rt_off_t tpos = pos * ndev->sector_size;
        rt_size_t tsize = size * ndev->sector_size;
        res = ndev->ops->read(ndev, tpos, buffer, tsize);
    }

    return res / ndev->sector_size;
}

static rt_size_t _mtd_write(rt_device_t dev,
                            rt_off_t    pos,
                            const void *buffer,
                            rt_size_t   size)
{
    struct rt_mtd_nor_device *ndev = (struct rt_mtd_nor_device *)dev;
    rt_size_t res = 0;

    // for flash, erase sector before write
    _mtd_erase(dev, pos, size);
    if (ndev && ndev->ops && ndev->ops->write)
    {
        rt_off_t tpos = pos * ndev->sector_size;
        rt_size_t tsize = size * ndev->sector_size;
        res = ndev->ops->write(ndev, tpos, buffer, tsize);
    }

    return res / ndev->sector_size;
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
        struct rt_mtd_nor_device *ndev = (struct rt_mtd_nor_device *)dev;

        geometry->block_size = ndev->block_size;
        geometry->bytes_per_sector = ndev->sector_size;
        //geometry->sector_count = (ndev->block_size * ndev->block_end) / geometry->bytes_per_sector;
        geometry->sector_count = (ndev->block_size * (ndev->block_end - ndev->block_start)) /
                                 geometry->bytes_per_sector;
    }
    else if (cmd == RT_DEVICE_CTRL_BLK_ERASE)
    {
        rt_uint32_t *param = (rt_uint32_t *)args;
        return _mtd_erase(dev, param[0], param[1]);
    }
    else if (cmd == RT_DEVICE_CTRL_GET_PHY_ADDR)
    {
        struct rt_mtd_nor_device *ndev = (struct rt_mtd_nor_device *)dev;
        struct rt_device_phy_addr_mapping *addr_mapping = (struct rt_device_phy_addr_mapping *)args;
        if (ndev->ops->control)
        {
            /* convert sector index to byte address */
            addr_mapping->logical_addr = ndev->block_size * ndev->block_start
                                         +  addr_mapping->logical_addr * ndev->sector_size;
            return ndev->ops->control(ndev, cmd, addr_mapping);
        }
        else
        {
            return RT_EINVAL;
        }

    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops mtd_nor_ops =
{
    _mtd_init,
    _mtd_open,
    _mtd_close,
    _mtd_read,
    _mtd_write,
    _mtd_control
};
#endif

rt_err_t rt_mtd_nor_register_device(const char               *name,
                                    struct rt_mtd_nor_device *device)
{
    rt_device_t dev;

    dev = RT_DEVICE(device);
    RT_ASSERT(dev != RT_NULL);

    /* set device class and generic device interface */
    dev->type        = RT_Device_Class_MTD;
#ifdef RT_USING_DEVICE_OPS
    dev->ops         = &mtd_nor_ops;
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
    if (device->sector_size == 0)
        device->sector_size = 4 * 1024;

    /* register to RT-Thread device system */
    return rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}

#endif
