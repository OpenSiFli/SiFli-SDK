/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_draw_epic.h"

#if LV_USE_DRAW_EPIC
#include "app_mem.h"
#include "lv_epic_utils.h"
#include "../lv_image_decoder_private.h"
#include "../../stdlib/lv_string.h"
#include <stdint.h>

typedef struct
{
    EPIC_LayerConfigTypeDef image;
    lv_image_decoder_dsc_t decoder;
    uint8_t *repacked;
} epic_image_resource_t;

/* Called on the UI thread after the render list finishes using the pixels. */
void lv_draw_epic_image_release(void *image)
{
    epic_image_resource_t *resource = image;
    if (resource == NULL) return;
    lv_image_decoder_close(&resource->decoder);
    app_free(resource->repacked);
    app_free(resource);
}

/* Keep the decoder/cache reference alive instead of copying its pixels.
 * Only byte strides that EPIC cannot express in pixels need repacking. */
EPIC_LayerConfigTypeDef *lv_draw_epic_image_prepare(const void *src, lv_opa_t opa,
                                                    lv_color_t color,
                                                    EPIC_LayerConfigTypeDef *mask)
{
    (void)color; /* Used for alpha-only images when EPIC_SUPPORT_A8 is enabled. */
    HAL_EPIC_LayerConfigInit(mask);
    epic_image_resource_t *resource = app_malloc(sizeof(*resource));
    if (resource == NULL) return NULL;
    *resource = (epic_image_resource_t){0};
    if (lv_image_decoder_open(&resource->decoder, src, NULL) != LV_RESULT_OK)
    {
        app_free(resource);
        return NULL;
    }

    EPIC_LayerConfigTypeDef *image = NULL;
    const lv_draw_buf_t *decoded = resource->decoder.decoded;
    if (resource->decoder.error_msg != NULL || decoded == NULL ||
        decoded->data == NULL || decoded->data_size == 0 ||
        decoded->header.w == 0 || decoded->header.h == 0)
        goto done;

    uint32_t color_mode;
    bool compressed = false;
    bool separate_alpha = false;
    if (decoded->header.flags & LV_IMAGE_FLAGS_EZIP)
    {
    #ifdef HAL_EZIP_MODULE_ENABLED
        color_mode = EPIC_INPUT_EZIP;
        compressed = true;
    #else
        goto done;
    #endif
    }
    else if (decoded->header.flags & LV_IMAGE_FLAGS_JPEG)
    {
    #ifdef EPIC_SUPPORT_JPEGD
        color_mode = EPIC_INPUT_JPEG;
        compressed = true;
    #else
        goto done;
    #endif
    }
    else
    {
        switch (decoded->header.cf)
        {
        case LV_COLOR_FORMAT_RGB565:
        case LV_COLOR_FORMAT_RGB888:
        case LV_COLOR_FORMAT_ARGB8888:
        case LV_COLOR_FORMAT_XRGB8888:
        case LV_COLOR_FORMAT_ARGB8565:
            break;
    #ifdef EPIC_SUPPORT_MASK
        case LV_COLOR_FORMAT_RGB565A8:
            separate_alpha = true;
            break;
    #endif
    #ifdef EPIC_SUPPORT_A8
        case LV_COLOR_FORMAT_A8:
            break;
    #endif
        default:
            goto done;
        }
        color_mode = lv_img_2_epic_cf(decoded->header.cf);
        if (decoded->header.flags & LV_IMAGE_FLAGS_PREMULTIPLIED)
            goto done;
    }

    size_t row_bytes = 0;
    size_t pixel_bytes = decoded->data_size;
    uint32_t pitch = decoded->header.w;
    bool repack = false;
    if (!compressed)
    {
        uint32_t color_size =
            separate_alpha ? 2 : lv_color_format_get_size(decoded->header.cf);
        row_bytes = (size_t)decoded->header.w * color_size;
        size_t source_size = (size_t)decoded->header.stride * decoded->header.h;
        if (color_size == 0 || decoded->header.stride < row_bytes)
            goto done;
        if (separate_alpha)
        {
            if (decoded->header.stride % 2 != 0 ||
                source_size > SIZE_MAX - source_size / 2)
                goto done;
            source_size += source_size / 2;
        }
        if (source_size > decoded->data_size)
            goto done;
        repack = decoded->header.stride % color_size != 0;
        if (!repack)
            pitch = decoded->header.stride / color_size;
        if (pitch > UINT16_MAX) goto done;
        size_t bytes_per_row =
            row_bytes + (separate_alpha ? decoded->header.w : 0);
        if (decoded->header.h > (SIZE_MAX - sizeof(*image)) / bytes_per_row)
            goto done;
        pixel_bytes = bytes_per_row * decoded->header.h;
    }
    if (pixel_bytes > SIZE_MAX - sizeof(*image) || pixel_bytes > UINT32_MAX)
        goto done;

    if (repack)
    {
        resource->repacked = app_malloc(pixel_bytes);
        if (resource->repacked == NULL) goto done;
    }
    image = &resource->image;
    HAL_EPIC_LayerConfigInit(image);
    image->data = repack ? resource->repacked : decoded->data;
    image->data_size = compressed ? decoded->data_size :
                       (repack ? row_bytes : decoded->header.stride) * decoded->header.h;
    image->color_mode = color_mode;
    image->alpha = opa;
    image->width = decoded->header.w;
    image->height = decoded->header.h;
    image->total_width = pitch;
    #ifdef EPIC_SUPPORT_A8
    if (color_mode == EPIC_INPUT_A8)
    {
        image->color_en = true;
        image->color_r = color.red;
        image->color_g = color.green;
        image->color_b = color.blue;
    }
    #endif
    if (repack)
    {
        /* EPIC expresses pitch in pixels. Repack rows to also handle RGB888
         * decoder strides that are aligned to bytes rather than whole pixels.
         */
        for (uint32_t row = 0; row < image->height; row++)
            lv_memcpy(image->data + row * row_bytes,
                      decoded->data + row * decoded->header.stride, row_bytes);
    }
    if (separate_alpha)
    {
        *mask = *image;
        mask->data = decoded->data + (size_t)decoded->header.stride * image->height;
        mask->data_size = (size_t)(decoded->header.stride / 2) * image->height;
        mask->color_mode = EPIC_INPUT_A8;
        mask->alpha = LV_OPA_COVER;
        mask->ax_mode = ALPHA_BLEND_MASK;
    }
    return image;
done:
    lv_draw_epic_image_release(resource);
    return NULL;
}

#endif /*LV_USE_DRAW_EPIC*/
