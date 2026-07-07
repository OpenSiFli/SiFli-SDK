/*
 * SPDX-FileCopyrightText: 2024 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file crypt_block_dev.c
 * @brief Transparent AES-encrypted block device — dm-crypt style wrapper over any
 *        RT_Device_Class_Block device.
 *
 * Usage example:
 * @code
 *   #include "crypt_block_dev.h"
 *
 *   static const uint8_t key[16] = { 0x00,0x01,... };
 *   static const uint8_t iv[16]  = { 0x10,0x20,... };
 *
 *   // Wrap "sd0" with AES-CBC-128 encryption, exposing it as "crypt_sd0"
 *   rt_crypt_blk_device_create("crypt_sd0", "sd0", key, 16, iv, AES_MODE_CBC);
 *
 *   // Now "crypt_sd0" can be used like any other block device (mount, etc.)
 *   rt_device_t dev = rt_device_find("crypt_sd0");
 *   rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
 *   rt_device_read(dev, 0, buf, 1);   // reads & decrypts sector 0
 *   rt_device_write(dev, 0, buf, 1);  // encrypts & writes sector 0
 * @endcode
 */

#include "crypt_block_dev.h"
#include <string.h>

#define DBG_TAG               "CRYPT_BLK"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

#ifndef __REV
#define __REV(x)  ((((x) & 0xFF000000U) >> 24) | \
                   (((x) & 0x00FF0000U) >>  8) | \
                   (((x) & 0x0000FF00U) <<  8) | \
                   (((x) & 0x000000FFU) << 24))
#endif

/* ---- Device operations ---- */

static rt_err_t crypt_blk_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t crypt_blk_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct crypt_blk_device *crypt_dev = (struct crypt_blk_device *)dev->user_data;

    if (crypt_dev->lower_dev == RT_NULL)
        return -RT_ERROR;

    return rt_device_open(crypt_dev->lower_dev, oflag);
}

static rt_err_t crypt_blk_close(rt_device_t dev)
{
    struct crypt_blk_device *crypt_dev = (struct crypt_blk_device *)dev->user_data;

    if (crypt_dev->lower_dev == RT_NULL)
        return -RT_ERROR;

    return rt_device_close(crypt_dev->lower_dev);
}

/**
 * @brief Read sectors from underlying device and decrypt them in-place.
 */
static rt_size_t crypt_blk_read(rt_device_t dev,
                                rt_off_t    pos,
                                void       *buffer,
                                rt_size_t   size)
{
    struct crypt_blk_device *crypt_dev = (struct crypt_blk_device *)dev->user_data;
    rt_size_t ret;
    rt_uint64_t byte_offset;

    if (dev == RT_NULL || buffer == RT_NULL || size == 0)
    {
        rt_set_errno(-EINVAL);
        return 0;
    }

    rt_mutex_take(&crypt_dev->lock, RT_WAITING_FOREVER);

    /* Read encrypted data from lower device directly into user buffer */
    ret = rt_device_read(crypt_dev->lower_dev, pos, buffer, size);
    if (ret != size)
    {
        LOG_E("crypt_blk_read: lower read failed at sector %d, "
              "ret=%d/%d", pos, ret, size);
        rt_mutex_release(&crypt_dev->lock);
        rt_set_errno(-EIO);
        return 0;
    }

    /* Decrypt in-place using AES-CTR: update IV counter based on byte offset.
     * Last 4 bytes of IV (iv[3]) are the counter in big-endian, incremented per 16-byte block. */
    byte_offset = (rt_uint64_t)pos * crypt_dev->geometry.bytes_per_sector;
    crypt_dev->aes_cfg.iv[3] = __REV((rt_uint32_t)(byte_offset >> 4));

    AES_IOTypeDef io;
    /* drv_aes support in-place operation */
    io.in_data  = (rt_uint8_t *)buffer;
    io.out_data = (rt_uint8_t *)buffer;
    io.size     = size * crypt_dev->geometry.bytes_per_sector;

    if (drv_aes_dec_sync(&crypt_dev->aes_cfg, &io) != RT_EOK)
    {
        LOG_E("crypt_blk_read: AES decrypt failed at sector %d, count=%d",
              pos, size);
        rt_mutex_release(&crypt_dev->lock);
        rt_set_errno(-EIO);
        return 0;
    }

    rt_mutex_release(&crypt_dev->lock);
    return size;
}

/**
 * @brief Encrypt data in-place then write to the underlying device.
 */
static rt_size_t crypt_blk_write(rt_device_t dev,
                                 rt_off_t    pos,
                                 const void *buffer,
                                 rt_size_t   size)
{
    struct crypt_blk_device *crypt_dev = (struct crypt_blk_device *)dev->user_data;
    rt_size_t ret;
    rt_uint64_t byte_offset;

    if (dev == RT_NULL || buffer == RT_NULL || size == 0)
    {
        rt_set_errno(-EINVAL);
        return 0;
    }

    rt_mutex_take(&crypt_dev->lock, RT_WAITING_FOREVER);

    /* Encrypt in-place using AES-CTR: update IV counter based on byte offset.
     * Last 4 bytes of IV (iv[3]) are the counter in big-endian, incremented per 16-byte block. */
    byte_offset = (rt_uint64_t)pos * crypt_dev->geometry.bytes_per_sector;
    crypt_dev->aes_cfg.iv[3] = __REV((rt_uint32_t)(byte_offset >> 4));

    AES_IOTypeDef io;
    /* drv_aes support in-place operation */
    io.in_data  = (rt_uint8_t *)buffer;
    io.out_data = (rt_uint8_t *)buffer;
    io.size     = size * crypt_dev->geometry.bytes_per_sector;

    if (drv_aes_enc_sync(&crypt_dev->aes_cfg, &io) != RT_EOK)
    {
        LOG_E("crypt_blk_write: AES encrypt failed at sector %d, count=%d",
              pos, size);
        rt_mutex_release(&crypt_dev->lock);
        rt_set_errno(-EIO);
        return 0;
    }

    /* Write encrypted data from user buffer directly to lower device */
    ret = rt_device_write(crypt_dev->lower_dev, pos, buffer, size);
    if (ret != size)
    {
        LOG_E("crypt_blk_write: lower write failed at sector %d, "
              "ret=%d/%d", pos, ret, size);
        rt_set_errno(-EIO);
    }

    /* Restore plaintext in buffer: CTR mode is symmetric (decrypt again = encrypt) */
    if (drv_aes_dec_sync(&crypt_dev->aes_cfg, &io) != RT_EOK)
    {
        LOG_E("crypt_blk_write: AES restore failed at sector %d, count=%d",
              pos, size);
        RT_ASSERT(0);
        rt_set_errno(-EIO);
        ret = 0;
    }

    rt_mutex_release(&crypt_dev->lock);

    if (ret != size)
    {
        goto __ERROR;
    }

    return size;

__ERROR:

    return ret;
}

/**
 * @brief Device control: get geometry, change key/IV at runtime.
 */
static rt_err_t crypt_blk_control(rt_device_t dev, int cmd, void *args)
{
    struct crypt_blk_device *crypt_dev = (struct crypt_blk_device *)dev->user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        if (args)
        {
            rt_memcpy(args, &crypt_dev->geometry,
                      sizeof(struct rt_device_blk_geometry));
        }
        else
        {
            return -RT_EINVAL;
        }
        break;

    case RT_DEVICE_CTRL_CRYPT_SET_KEY:
    {
        AES_KeyTypeDef *new_cfg = (AES_KeyTypeDef *)args;
        if ((new_cfg == RT_NULL)
                || ((new_cfg->key_size != 16) && (new_cfg->key_size != 24) && (new_cfg->key_size != 32)))
        {
            return -RT_EINVAL;
        }

        rt_mutex_take(&crypt_dev->lock, RT_WAITING_FOREVER);
        crypt_dev->aes_cfg.key_size = new_cfg->key_size;
        rt_memcpy(crypt_dev->key_buf, new_cfg->key, new_cfg->key_size);
        crypt_dev->aes_cfg.key = (rt_uint32_t *)crypt_dev->key_buf;
        rt_mutex_release(&crypt_dev->lock);
        break;
    }

    case RT_DEVICE_CTRL_CRYPT_SET_IV:
    {
        if (args == RT_NULL)
            return -RT_EINVAL;

        rt_mutex_take(&crypt_dev->lock, RT_WAITING_FOREVER);
        rt_memcpy(crypt_dev->iv_buf, args, 16);
        crypt_dev->aes_cfg.iv = (rt_uint32_t *)crypt_dev->iv_buf;
        rt_mutex_release(&crypt_dev->lock);
        break;
    }

    default:
        return -RT_EINVAL;
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops crypt_blk_ops =
{
    crypt_blk_init,
    crypt_blk_open,
    crypt_blk_close,
    crypt_blk_read,
    crypt_blk_write,
    crypt_blk_control
};
#endif

/* ---- Public API ---- */

/**
 * @brief Create a transparently encrypted block device.
 *
 * The new device presents identical sector count and geometry as the
 * underlying device. All data is transparently encrypted on write and
 * decrypted on read.
 *
 * @note The underlying device must already be registered.
 */
rt_err_t rt_crypt_blk_device_create(const char *name,
                                    const char *lower_name,
                                    const rt_uint8_t *key,
                                    rt_uint32_t key_size,
                                    const rt_uint8_t *iv)
{
    struct crypt_blk_device *crypt_dev = NULL;
    rt_device_t lower_dev;
    rt_err_t ret;

    RT_ASSERT(name != RT_NULL);
    RT_ASSERT(lower_name != RT_NULL);
    RT_ASSERT(key != RT_NULL);
    RT_ASSERT(key_size == 16 || key_size == 24 || key_size == 32);

    /* Find the underlying block device */
    lower_dev = rt_device_find(lower_name);
    if (lower_dev == RT_NULL)
    {
        LOG_E("crypt_blk: underlying device '%s' not found", lower_name);
        return -RT_ERROR;
    }

    /* Check that it is a block device */
    if (lower_dev->type != RT_Device_Class_Block)
    {
        LOG_E("crypt_blk: device '%s' is not a block device (type=%d)",
              lower_name, lower_dev->type);
        return -RT_ERROR;
    }

    /* Allocate the crypt block device */
    crypt_dev = (struct crypt_blk_device *)rt_calloc(1,
                sizeof(struct crypt_blk_device));
    if (crypt_dev == RT_NULL)
    {
        LOG_E("crypt_blk: failed to allocate device structure");
        return -RT_ENOMEM;
    }

    /* Initialize cipher configuration */
    rt_memcpy(crypt_dev->key_buf, key, key_size);
    crypt_dev->aes_cfg.key      = (rt_uint32_t *)crypt_dev->key_buf;
    crypt_dev->aes_cfg.key_size = key_size;

    if (iv != RT_NULL)
    {
        rt_memcpy(crypt_dev->iv_buf, iv, 16);
        crypt_dev->aes_cfg.iv = (rt_uint32_t *)crypt_dev->iv_buf;
    }
    else
    {
        rt_memset(crypt_dev->iv_buf, 0, 16);
        crypt_dev->aes_cfg.iv = (rt_uint32_t *)crypt_dev->iv_buf;
    }

    crypt_dev->aes_cfg.mode = AES_MODE_CTR;

    /* Query geometry from the lower device */
    ret = rt_device_control(lower_dev, RT_DEVICE_CTRL_BLK_GETGEOME,
                            &crypt_dev->geometry);
    if (ret != RT_EOK)
    {
        LOG_E("crypt_blk: failed to get geometry from '%s'", lower_name);
        goto __ERROR;
    }

    if (crypt_dev->geometry.bytes_per_sector & 15)
    {
        LOG_E("crypt_blk: sector size %d is not 16bytes aligned", crypt_dev->geometry.bytes_per_sector);
        goto __ERROR;
    }

    crypt_dev->lower_dev = lower_dev;

    /* Initialize mutex */
    ret = rt_mutex_init(&crypt_dev->lock, name, RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
    {
        LOG_E("crypt_blk: failed to create mutex");
        goto __ERROR;
    }

    /* Set up RT-Thread device */
    crypt_dev->dev.type      = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    crypt_dev->dev.ops       = &crypt_blk_ops;
#else
    crypt_dev->dev.init      = crypt_blk_init;
    crypt_dev->dev.open      = crypt_blk_open;
    crypt_dev->dev.close     = crypt_blk_close;
    crypt_dev->dev.read      = crypt_blk_read;
    crypt_dev->dev.write     = crypt_blk_write;
    crypt_dev->dev.control   = crypt_blk_control;
#endif
    crypt_dev->dev.user_data = crypt_dev;

    /* Register the device */
    ret = rt_device_register(&crypt_dev->dev, name,
                             RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
    if (ret != RT_EOK)
    {
        LOG_E("crypt_blk: failed to register device '%s'", name);
        rt_mutex_detach(&crypt_dev->lock);
        goto __ERROR;
    }

    LOG_I("crypt_blk: created encrypted device '%s' -> '%s', "
          "sectors=%d, key_size=%d",
          name, lower_name, crypt_dev->geometry.sector_count, key_size);

    return RT_EOK;

__ERROR:
    if (crypt_dev)
    {
        rt_free(crypt_dev);
    }

    return ret;
}

/**
 * @brief Destroy a crypt block device.
 */
void rt_crypt_blk_device_destroy(struct crypt_blk_device *crypt_dev)
{
    if (crypt_dev == RT_NULL)
        return;

    rt_device_unregister(&crypt_dev->dev);
    rt_mutex_detach(&crypt_dev->lock);
    rt_free(crypt_dev);

    LOG_I("crypt_blk: destroyed encrypted block device");
}
