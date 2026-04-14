/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_I2S_AUDIO_H__
#define __DRV_I2S_AUDIO_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_common.h>

#define MUTE_UNDER_MIN_VOLUME  (-256)

#define AUDIO_DATA_SIZE 640 //480

typedef void (*i2s_rx_callback_t)(char *name, uint8_t *data, rt_size_t len);

void rt_device_set_i2s_dma_rx_callback(i2s_rx_callback_t callback);

void bf0_i2s_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size); /*para is same to rt_device_write*/

#endif
