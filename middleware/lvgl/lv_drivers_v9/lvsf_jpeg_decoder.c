/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/

#include "../../draw/lv_image_decoder_private.h"
#include "../../../lvgl.h"
#if (1 == LV_USE_DRAW_EPIC) && defined(LV_USE_SIFLI_JPEGD)

#include "../../misc/lv_fs_private.h"
#include <string.h>

/*********************
 *      DEFINES
 *********************/

#define DECODER_NAME    "SFJPGD"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static lv_result_t decoder_info(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc, lv_image_header_t *header);
static lv_result_t decoder_open(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc);
static void decoder_close(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc);
static int is_jpg(const uint8_t *raw_data, size_t len);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lvsf_jpegd_init(void)
{
    lv_image_decoder_t *dec = lv_image_decoder_create();
    lv_image_decoder_set_info_cb(dec, decoder_info);
    lv_image_decoder_set_open_cb(dec, decoder_open);
    lv_image_decoder_set_close_cb(dec, decoder_close);

    dec->name = DECODER_NAME;
}

void lvsf_jpegd_deinit(void)
{
    lv_image_decoder_t *dec = NULL;
    while ((dec = lv_image_decoder_get_next(dec)) != NULL)
    {
        if (dec->info_cb == decoder_info)
        {
            lv_image_decoder_delete(dec);
            break;
        }
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static lv_result_t decoder_info(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc, lv_image_header_t *header)
{
    LV_UNUSED(decoder);

    const void *src = dsc->src;
    lv_image_src_t src_type = dsc->src_type;

    if (src_type == LV_IMAGE_SRC_VARIABLE)
    {
        const lv_image_dsc_t *img_dsc = src;
        uint8_t *raw_data = (uint8_t *)img_dsc->data;
        const uint32_t raw_data_size = img_dsc->data_size;

        if (is_jpg(raw_data, raw_data_size) == true)
        {
            lv_memcpy(header, &img_dsc->header, sizeof(lv_image_header_t));
            header->flags |= LV_IMAGE_FLAGS_JPEG;
            return LV_RESULT_OK;
        }
    }
    else if (src_type == LV_IMAGE_SRC_FILE)
    {
        const char *fn = src;
        const char *ext = lv_fs_get_ext(fn);
        if ((lv_strcmp(ext, "jpg") == 0) || (lv_strcmp(ext, "jpeg") == 0))
        {


            //TODO: read the image header to get width and height
            return LV_RESULT_INVALID;
        }
    }
    return LV_RESULT_INVALID;
}

/**
 * Decode a JPG image and return the decoded data.
 * @param decoder pointer to the decoder
 * @param dsc     pointer to the decoder descriptor
 * @return LV_RESULT_OK: no error; LV_RESULT_INVALID: can't open the image
 */
static lv_result_t decoder_open(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);
    if (dsc->src_type == LV_IMAGE_SRC_VARIABLE)
    {
        const lv_image_dsc_t *img_dsc = dsc->src;
        if (is_jpg(img_dsc->data, img_dsc->data_size) == true)
        {
            //Return the compressed data directly
            LV_ASSERT(!dsc->decoded);
            dsc->decoded = lv_malloc(sizeof(lv_draw_buf_t));
            if (dsc->decoded == NULL)
            {
                return LV_RESULT_INVALID;
            }
            lv_draw_buf_t *decoded = (lv_draw_buf_t *)dsc->decoded;
            decoded->data = (uint8_t *)img_dsc->data;
            decoded->header = img_dsc->header;
            decoded->header.flags |= LV_IMAGE_FLAGS_JPEG;
            decoded->data_size = img_dsc->data_size;
            decoded->unaligned_data = decoded->data;
            decoded->handlers = lv_draw_buf_get_handlers();

            return LV_RESULT_OK;
        }
    }
    else if (dsc->src_type == LV_IMAGE_SRC_FILE)
    {
        const char *fn = dsc->src;
        if ((lv_strcmp(lv_fs_get_ext(fn), "jpg") == 0) || (lv_strcmp(lv_fs_get_ext(fn), "jpeg") == 0))
        {
            //TODO: Implement file reading and decoding logic
            return LV_RESULT_INVALID;
        }
    }

    return LV_RESULT_INVALID;
}

/**
 * Free the allocated resources
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 */
static void decoder_close(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);
    if (dsc->decoded) lv_free((void *)dsc->decoded);
}

static int is_jpg(const uint8_t *raw_data, size_t len)
{
    const uint8_t jpg_signature[] = {0xFF, 0xD8, 0xFF,  0xE0,  0x00,  0x10, 0x4A,  0x46, 0x49, 0x46};
    if (len < sizeof(jpg_signature)) return false;
    return memcmp(jpg_signature, raw_data, sizeof(jpg_signature)) == 0;
}

#endif /*LV_USE_DRAW_EPIC*/
