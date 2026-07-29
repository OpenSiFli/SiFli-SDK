/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EPD_WAVEFORM_H__
#define __EPD_WAVEFORM_H__

typedef enum
{
    EPD_DRAW_MODE_INVALID = 0,
    EPD_DRAW_MODE_AUTO = 1,
    EPD_DRAW_MODE_FULL = 2,
    EPD_DRAW_MODE_PARTIAL = 3,
} EpdDrawMode;

void epd_wave_table(void);// Initialize wave table

uint32_t epd_wave_table_get_frames(int temperature, EpdDrawMode mode);

void epd_wave_table_fill_lut(uint32_t *p_argb8888_lut, uint32_t frame_num);
#endif /* __EPD_WAVEFORM_H__ */