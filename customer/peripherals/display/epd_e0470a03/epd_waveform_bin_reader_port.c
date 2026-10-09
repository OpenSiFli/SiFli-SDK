/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "mem_map.h"
#include "drv_flash.h"

#ifdef EPD_WAVEFORM_USE_BIN

#include <stdint.h>
#include <stddef.h>

/*
 * These three functions are called by libepd_waveform_bin_reader_gcc.a.
 * They are declared as extern in epd_waveform_bin_reader.h:
 *   extern void *waveform_bin_reader_malloc(size_t size);
 *   extern void waveform_bin_reader_free(void *ptr);
 *   extern int waveform_bin_reader_read_data(uint32_t offset, uint8_t *buf, uint32_t size);
 */

void *waveform_bin_reader_malloc(size_t size)
{
    return rt_malloc(size);
}

void waveform_bin_reader_free(void *ptr)
{
    rt_free(ptr);
}

int waveform_bin_reader_read_data(uint32_t offset, uint8_t *buf, uint32_t size)
{
    return rt_flash_read(CUSTOM_EPD_WAVE_TABLE_START_ADDR + offset, buf, (int)size);
}

#endif /* EPD_WAVEFORM_USE_BIN */
