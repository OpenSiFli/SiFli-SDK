/*
 * SPDX-FileCopyrightText: 2024 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CRYPT_BLOCK_DEV_H_
#define __CRYPT_BLOCK_DEV_H_

#include <rtthread.h>
#include "drv_aes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup middleware Middleware
  * @{
  */

/** @defgroup crypt_block_dev Crypt Block Device
  * @brief Transparent AES-encrypted block device (dm-crypt style)
  * @{
  */

/**
 * @brief IOCTL command: set encryption key
 * @arg AES_KeyTypeDef pointer
 */
#define RT_DEVICE_CTRL_CRYPT_SET_KEY      0x2001

/**
 * @brief IOCTL command: set initial vector
 * @arg 16-byte IV buffer pointer
 */
#define RT_DEVICE_CTRL_CRYPT_SET_IV       0x2002

/**
 * @brief Crypt block device structure
 */
struct crypt_blk_device
{
    struct rt_device dev;                       /**< RT-Thread device */
    rt_device_t lower_dev;                      /**< underlying raw block device */
    AES_KeyTypeDef aes_cfg;                     /**< AES cipher configuration */
    rt_uint8_t key_buf[32] ALIGN(4);            /**< key buffer (4-byte aligned) */
    rt_uint8_t iv_buf[16] ALIGN(4);             /**< IV buffer (4-byte aligned) */
    struct rt_device_blk_geometry geometry;     /**< device geometry */
    struct rt_mutex lock;                       /**< mutex for thread safety */
};

/**
 * @brief Create a transparently encrypted block device wrapping an existing one using AES-CTR.
 *
 * All data written is AES-encrypted before passing to the underlying device.
 * All data read is AES-decrypted before returning to the caller.
 *
 * @param name       Name for the new encrypted block device
 * @param lower_name Name of the underlying block device to wrap
 * @param key        AES key (16, 24, or 32 bytes)
 * @param key_size   Key size in bytes: 16 (AES-128), 24 (AES-192), 32 (AES-256)
 * @param iv         16-byte initial vector (used for CTR; may be NULL for ECB)
 * @return RT_EOK on success, error code otherwise
 */
rt_err_t rt_crypt_blk_device_create(const char *name,
                                    const char *lower_name,
                                    const rt_uint8_t *key,
                                    rt_uint32_t key_size,
                                    const rt_uint8_t *iv);

/**
 * @brief Destroy a previously created crypt block device.
 * @param dev Pointer to the crypt_blk_device to destroy
 */
void rt_crypt_blk_device_destroy(struct crypt_blk_device *dev);

/// @} crypt_block_dev
/// @} middleware

#ifdef __cplusplus
}
#endif

#endif /* __CRYPT_BLOCK_DEV_H_ */
