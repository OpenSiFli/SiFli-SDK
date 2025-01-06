/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#include <fal.h>
#include <string.h>

/* ===================== Flash device Configuration ========================= */

#ifndef FAL_FLASH_DEV_TABLE

static const struct fal_flash_dev *const device_table[] =
{
#if defined(BSP_ENABLE_QSPI1)&&(BSP_QSPI1_MODE<2)
    &nor_flash1,
#endif
#if defined(BSP_ENABLE_QSPI2)&&(BSP_QSPI2_MODE<2)
    &nor_flash2,
#endif
#if defined(BSP_ENABLE_QSPI3)&&(BSP_QSPI3_MODE<2)
    &nor_flash3,
#endif
#if defined(BSP_ENABLE_QSPI4)&&(BSP_QSPI4_MODE<2)
    &nor_flash4,
#endif
};
#else

static const struct fal_flash_dev *const device_table[] = FAL_FLASH_DEV_TABLE;

#endif

static const size_t device_table_len = sizeof(device_table) / sizeof(device_table[0]);
static uint8_t init_ok = 0;

/**
 * Initialize all flash device on FAL flash table
 *
 * @return result
 */
int fal_flash_init(void)
{
    size_t i;

    if (init_ok)
    {
        return 0;
    }

    for (i = 0; i < device_table_len; i++)
    {
        assert(device_table[i]->ops.read);
        assert(device_table[i]->ops.write);
        assert(device_table[i]->ops.erase);
        /* init flash device on flash table */
        if (device_table[i]->ops.init)
        {
            device_table[i]->ops.init();
        }
        log_d("Flash device | %*.*s | addr: 0x%08lx | len: 0x%08x | blk_size: 0x%08x |initialized finish.",
                FAL_DEV_NAME_MAX, FAL_DEV_NAME_MAX, device_table[i]->name, device_table[i]->addr, device_table[i]->len,
                device_table[i]->blk_size);
    }

    init_ok = 1;
    return 0;
}

/**
 * find flash device by name
 *
 * @param name flash device name
 *
 * @return != NULL: flash device
 *            NULL: not found
 */
const struct fal_flash_dev *fal_flash_device_find(const char *name)
{
    assert(init_ok);
    assert(name);

    size_t i;

    for (i = 0; i < device_table_len; i++)
    {
        if (!strncmp(name, device_table[i]->name, FAL_DEV_NAME_MAX)) {
            return device_table[i];
        }
    }

    return NULL;
}
