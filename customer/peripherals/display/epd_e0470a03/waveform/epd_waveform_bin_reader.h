/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

typedef enum {
    EPD_WAVEFORM_ERR_OK = 0,
    EPD_WAVEFORM_ERR_INVALID_MAGIC_NUMBER = -1,
    EPD_WAVEFORM_ERR_INVALID_VERSION = -2,
    EPD_WAVEFORM_ERR_MALLOC_FAILED = -3,
    EPD_WAVEFORM_ERR_READ_FAILED = -4,
    EPD_WAVEFORM_ERR_INVALID_FIRST_TABLE_OFFSET = -10, //-10 ~ -25 for mode 0~15

    EPD_WAVEFORM_ERR_INVALID_TABLE_OFFSET       = -1000, //
} EPD_WAVEFORM_ERR_TYPEDEF;



/*
E0470A03 Waveform Table Mode:
mode 0: GC16 FULL   // full refresh
mode 1: DU PARTIAL  // black/white refresh, 2 levels
mode 3: INIT        // refresh to an all-white page, used for initialization
mode 5: GL16        // 16 gray levels
mode 6: A2          // animation mode, black/white 2 levels
mode 7: DU4         // DU, 4 gray levels
*/

typedef enum
{
    WAVE_MODE_NONE    = 0,
    WAVE_MODE_PARTIAL = 5,
    WAVE_MODE_FULL    = 0,

    WAVE_MODE_MAX = 16
} WAVE_TABLE_MODE_T;

int waveform_bin_reader_init(uint32_t max_size);
uint32_t waveform_bin_reader_get_frames(int temperature, WAVE_TABLE_MODE_T mode);
void waveform_bin_reader_fill_lut(uint32_t *p_epic_lut, uint32_t frame_num);


/* External interfaces */
extern int waveform_bin_reader_read_data(uint32_t offset, uint8_t *buf, uint32_t size);
extern void *waveform_bin_reader_malloc(size_t size);
extern void waveform_bin_reader_free(void *ptr);