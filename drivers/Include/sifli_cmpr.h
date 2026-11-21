/**
 ******************************************************************************
 * @file   sifli_cmpr.h
 * @author Sifli software development team
 * @brief  SiFli compression module
 *
 ******************************************************************************
 */
/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SIFLI_CMPR_H
#define __SIFLI_CMPR_H
#include "bf0_hal_def.h"

#ifndef HAL_ALIGN
    #define HAL_ALIGN(size, align)           (((size) + (align) - 1) & ~((align) - 1))
#endif /* HAL_ALIGN */

#define MAX_CMPR_TBL_SIZE (3)
//const float rgb565_cmpr_rate_tbl[MAX_CMPR_TBL_SIZE] = {1.33, 1.47, 1.6, 1.73, 1.87, 1.93, 2, 2.13, 2.26, 2.4};
//const float rgb888_cmpr_rate_tbl[MAX_CMPR_TBL_SIZE] = {2, 2.2, 2.4, 2.6, 2.8, 2.9, 3, 3.2, 3.4, 3.6};
//const float argb8888_cmpr_rate_tbl[MAX_CMPR_TBL_SIZE] = {2.67, 2.93, 3.2, 3.47, 3.73, 3.86, 4.0, 4.27, 4.53, 4.8};

/************
 *
 ***********/

#ifdef HAL_EPICTL_ENABLED

    #define ROUNDED_TARGET_SIZE(bytes, num, denom) ((((((bytes)*(num) + (denom -1))/(denom)) +5 )/6)&(~1))

    #define CMPR_CHUNKS(pixels)  (HAL_ALIGN(pixels, 512)>>9)
    #define CMPR_CHUNK_SIZE(pixels) (((pixels) + CMPR_CHUNKS(pixels) - 1)/CMPR_CHUNKS(pixels))

    #define CMPR_1_RGB565_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*2, 3,   4)
    #define CMPR_2_RGB565_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*2, 100, 147)
    #define CMPR_3_RGB565_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*2, 5,   8)

    #define CMPR_1_RGB888_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*3, 1,   2)
    #define CMPR_2_RGB888_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*3, 5,   11)
    #define CMPR_3_RGB888_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*3, 5,   12)

    #define CMPR_1_ARGB8888_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*4, 100, 267)
    #define CMPR_2_ARGB8888_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*4, 100, 293)
    #define CMPR_3_ARGB8888_TGT_SIZE(chunk_pixels) ROUNDED_TARGET_SIZE(chunk_pixels*4, 25, 80)

    #define CMPR_1_RGB565_COMPRESSED_BYTES(pixels)     (CMPR_CHUNKS(pixels) * 6 * CMPR_1_RGB565_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))
    #define CMPR_2_RGB565_COMPRESSED_BYTES(pixels)     (CMPR_CHUNKS(pixels) * 6 * CMPR_2_RGB565_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))
    #define CMPR_3_RGB565_COMPRESSED_BYTES(pixels)     (CMPR_CHUNKS(pixels) * 6 * CMPR_3_RGB565_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))

    #define CMPR_1_RGB888_COMPRESSED_BYTES(pixels)     (CMPR_CHUNKS(pixels) * 6 * CMPR_1_RGB888_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))
    #define CMPR_2_RGB888_COMPRESSED_BYTES(pixels)     (CMPR_CHUNKS(pixels) * 6 * CMPR_2_RGB888_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))
    #define CMPR_3_RGB888_COMPRESSED_BYTES(pixels)     (CMPR_CHUNKS(pixels) * 6 * CMPR_3_RGB888_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))

    #define CMPR_1_ARGB8888_COMPRESSED_BYTES(pixels)    (CMPR_CHUNKS(pixels) * 6 * CMPR_1_ARGB8888_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))
    #define CMPR_2_ARGB8888_COMPRESSED_BYTES(pixels)    (CMPR_CHUNKS(pixels) * 6 * CMPR_2_ARGB8888_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))
    #define CMPR_3_ARGB8888_COMPRESSED_BYTES(pixels)    (CMPR_CHUNKS(pixels) * 6 * CMPR_3_ARGB8888_TGT_SIZE(CMPR_CHUNK_SIZE(pixels)))


    #define CMPR_DATA_TYPE uint8_t


#else /* HAL_EPICTL_ENABLED */



    #define TARGET_SIZE_TO_CMPR_WORDS(size) ((size)*6/4)
    #define ROUNDED_TARGET_SIZE(w, num, denom) (((((w)*(num) + (denom - 1))/(denom))*4/6)&(~1))


    #define CMPR_1_RGB565_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 3, 4)
    #define CMPR_2_RGB565_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 100, 147)
    #define CMPR_3_RGB565_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 5, 8)

    #define CMPR_1_RGB888_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 1, 2)
    #define CMPR_2_RGB888_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 5, 11)
    #define CMPR_3_RGB888_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 5, 12)

    #define CMPR_1_ARGB8888_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 100, 267)
    #define CMPR_2_ARGB8888_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 100, 293)
    #define CMPR_3_ARGB8888_TGT_SIZE(uncmpr_words) ROUNDED_TARGET_SIZE(uncmpr_words, 25, 80)

    #define CMPR_0_TGT_SIZE(uncmpr_words)         ROUNDED_TARGET_SIZE(uncmpr_words, 1, 1)



    #define CMPR_1_RGB565_COMPRESSED_BYTES(pixels)     (6 * CMPR_1_RGB565_TGT_SIZE(HAL_ALIGN((pixels)*2, 4)/4))
    #define CMPR_2_RGB565_COMPRESSED_BYTES(pixels)     (6 * CMPR_2_RGB565_TGT_SIZE(HAL_ALIGN((pixels)*2, 4)/4))
    #define CMPR_3_RGB565_COMPRESSED_BYTES(pixels)     (6 * CMPR_3_RGB565_TGT_SIZE(HAL_ALIGN((pixels)*2, 4)/4))

    #define CMPR_1_RGB888_COMPRESSED_BYTES(pixels)     (6 * CMPR_1_RGB888_TGT_SIZE(HAL_ALIGN((pixels)*3, 4)/4))
    #define CMPR_2_RGB888_COMPRESSED_BYTES(pixels)     (6 * CMPR_2_RGB888_TGT_SIZE(HAL_ALIGN((pixels)*3, 4)/4))
    #define CMPR_3_RGB888_COMPRESSED_BYTES(pixels)     (6 * CMPR_3_RGB888_TGT_SIZE(HAL_ALIGN((pixels)*3, 4)/4))

    #define CMPR_1_ARGB8888_COMPRESSED_BYTES(pixels)   (6 * CMPR_1_ARGB8888_TGT_SIZE(pixels))
    #define CMPR_2_ARGB8888_COMPRESSED_BYTES(pixels)   (6 * CMPR_2_ARGB8888_TGT_SIZE(pixels))
    #define CMPR_3_ARGB8888_COMPRESSED_BYTES(pixels)   (6 * CMPR_3_ARGB8888_TGT_SIZE(pixels))

    #define CMPR_DATA_TYPE uint8_t


#endif /* HAL_EPICTL_ENABLED */


__STATIC_INLINE HAL_StatusTypeDef CMPR_GetConfig(uint32_t pixel_bytes, uint32_t *p_cfg0, uint32_t *p_cfg1)
{
    (void) pixel_bytes;

    if (!p_cfg0 || !p_cfg1)
    {
        return HAL_ERROR;
    }


    *p_cfg0 = 0x40023344; //0x40023345;
    *p_cfg1 = 0x01044aa2;


    return HAL_OK;
}

// Get the compressed pixels bytes with specified compression rate.
__STATIC_INLINE uint32_t CMPR_GetCompressedBytes(uint32_t pixels, uint32_t pixel_bytes, uint32_t cmpr_rate)
{
    // HAL_ASSERT((pixel_bytes >= 2) && (pixel_bytes <= 4));
    // HAL_ASSERT((cmpr_rate > 0) && ((cmpr_rate - 1) < MAX_CMPR_TBL_SIZE));
    uint32_t compressed_bytes = UINT32_MAX;


    if (2 == pixel_bytes)//RGB565
    {
        switch (cmpr_rate)
        {
        case 1:
        default:
            compressed_bytes = CMPR_1_RGB565_COMPRESSED_BYTES(pixels);
            break;
        case 2:
            compressed_bytes = CMPR_2_RGB565_COMPRESSED_BYTES(pixels);
            break;
        case 3:
            compressed_bytes = CMPR_3_RGB565_COMPRESSED_BYTES(pixels);
            break;
        }
    }
    else if (3 == pixel_bytes)//RGB888
    {
        switch (cmpr_rate)
        {
        case 1:
        default:
            compressed_bytes = CMPR_1_RGB888_COMPRESSED_BYTES(pixels);
            break;
        case 2:
            compressed_bytes = CMPR_2_RGB888_COMPRESSED_BYTES(pixels);
            break;
        case 3:
            compressed_bytes = CMPR_3_RGB888_COMPRESSED_BYTES(pixels);
            break;
        }
    }
    else if (4 == pixel_bytes) //ARGB8888
    {
        switch (cmpr_rate)
        {
        case 1:
        default:
            compressed_bytes = CMPR_1_ARGB8888_COMPRESSED_BYTES(pixels);
            break;
        case 2:
            compressed_bytes = CMPR_2_ARGB8888_COMPRESSED_BYTES(pixels);
            break;
        case 3:
            compressed_bytes = CMPR_3_ARGB8888_COMPRESSED_BYTES(pixels);
            break;
        }
    }

    return compressed_bytes;
}
#endif /* __SIFLI_CMPR_H */