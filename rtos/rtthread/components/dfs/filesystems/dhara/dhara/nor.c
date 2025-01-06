#include "rtthread.h"
#include <rtdevice.h>
#include "nand.h"
#include <stdbool.h>
#include <string.h>
#include "log.h"
#include "map.h"


// public function definitions
int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b)
{
    return 0;
}

void dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b)
{
}

int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b, dhara_error_t *err)
{
    struct rt_mtd_nor_device *dev;
    rt_err_t rt_err;

    dev = (struct rt_mtd_nor_device *)n->user_data;

    //rt_kprintf("erase:%d\n", b);
    rt_err = rt_mtd_nor_erase_block(dev, b * dev->block_size, dev->block_size);
    if (RT_EOK == rt_err)
    {
        // success
        return 0;
    }
    else
    {
        RT_ASSERT(0);
        *err = DHARA_E_BAD_BLOCK;
        return -1;
    }
}

int dhara_nand_prog(const struct dhara_nand *n, dhara_page_t p, const uint8_t *data,
                    dhara_error_t *err)
{
    struct rt_mtd_nor_device *dev;
    rt_size_t wr_size;

    //rt_kprintf("prog:%d\n", p);
    //LOG_HEX("prog", 8, (uint8_t *)data, 2048);

    dev = (struct rt_mtd_nor_device *)n->user_data;
    wr_size = rt_mtd_nor_write(dev, p * dev->sector_size, data, dev->sector_size);
    if (wr_size == dev->sector_size)
    {
        //LOG_HEX("data", 16, (rt_uint8_t *)data, 64);
        // success
        return 0;
    }
    else
    {
        RT_ASSERT(0);
        if (err)
        {
            *err = DHARA_E_BAD_BLOCK;
        }
        return -1;
    }
}

int dhara_nand_is_free(const struct dhara_nand *n, dhara_page_t p)
{
    struct rt_mtd_nor_device *dev;
    rt_err_t err;
    rt_uint32_t *buf;
    int is_free;
    uint32_t page_size_in_word;
    struct rt_device_phy_addr_mapping addr_mapping;

    //rt_kprintf("is_free:%d\n", p);

    dev = (struct rt_mtd_nor_device *)n->user_data;
    page_size_in_word = (1 << n->log2_page_size) >> 2;
    addr_mapping.logical_addr = p;
    if (RT_EOK != rt_device_control((rt_device_t)dev, RT_DEVICE_CTRL_GET_PHY_ADDR, &addr_mapping))
    {
        RT_ASSERT(0);
    }

    buf = (rt_uint32_t *)addr_mapping.physical_addr;
    is_free = true;
    for (rt_uint32_t i = 0; i < page_size_in_word; i++)
    {
        if (buf[i] != RT_UINT32_MAX)
        {
            is_free = false;
            break;
        }
    }

    return is_free;
}

int dhara_nand_read(const struct dhara_nand *n, dhara_page_t p, size_t offset, size_t length,
                    uint8_t *data, dhara_error_t *err)
{
    struct rt_mtd_nor_device *dev;
    rt_err_t rt_err;
    rt_uint8_t *buf;
    int ret;
    struct rt_device_phy_addr_mapping addr_mapping;

    dev = (struct rt_mtd_nor_device *)n->user_data;
    addr_mapping.logical_addr = p;
    if (RT_EOK != rt_device_control((rt_device_t)dev, RT_DEVICE_CTRL_GET_PHY_ADDR, &addr_mapping))
    {
        RT_ASSERT(0);
    }

    buf = (rt_uint8_t *)addr_mapping.physical_addr + offset;
    memcpy(data, buf, length);

    ret = 0;

    //rt_kprintf("read:%d,%d,%d\n", p, offset, length);

    return ret;
}

/* Read a page from one location and reprogram it in another location.
 * This might be done using the chip's internal buffers, but it must use
 * ECC.
 */
int dhara_nand_copy(const struct dhara_nand *n, dhara_page_t src, dhara_page_t dst,
                    dhara_error_t *err)
{
    struct rt_mtd_nor_device *dev;
    rt_size_t wr_size;
    rt_uint8_t *buf;
    int ret;
    struct rt_device_phy_addr_mapping addr_mapping;
    rt_uint32_t page_size;
    
    dev = (struct rt_mtd_nor_device *)n->user_data;

    addr_mapping.logical_addr = src;
    if (RT_EOK != rt_device_control((rt_device_t)dev, RT_DEVICE_CTRL_GET_PHY_ADDR, &addr_mapping))
    {
        RT_ASSERT(0);
    }

    page_size = (1 << n->log2_page_size);
    buf = rt_malloc(page_size);
    RT_ASSERT(buf);

    //rt_kprintf("copy:%d,%d\n", dst, src);
    memcpy(buf, (void *)addr_mapping.physical_addr, page_size);

    //wr_size = rt_device_write((rt_device_t)dev, dst, buf, 1);    
    wr_size = rt_mtd_nor_write(dev, dst * dev->sector_size, buf, dev->sector_size);    
    if (dev->sector_size != wr_size)
    {
        RT_ASSERT(0);
        if (err)
        {
            *err = DHARA_E_ECC;
        }
        ret = -1;
    }
    else
    {
        ret = 0;
    }

    rt_free(buf);

    return ret;
}


