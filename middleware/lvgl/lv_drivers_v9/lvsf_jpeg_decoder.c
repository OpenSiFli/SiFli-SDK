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

#ifdef SOC_SF32LB58X
#include "jpeg_dec.h"
#include "app_mem.h"
#include "../../draw/lv_draw_buf_private.h"
#include "../../osal/lv_os_private.h"
#if defined(LVSF_JPEG_DECODER_ALLOC_SRAM) || defined(CONFIG_LVSF_JPEG_DECODER_ALLOC_SRAM)
#define JPEG_DECODER_CACHE_POOL IMAGE_CACHE_SRAM
#else
#define JPEG_DECODER_CACHE_POOL IMAGE_CACHE_PSRAM
#endif
#ifdef DRV_EPIC_NEW_API
#include "lv_draw_epic.h"
#endif
#endif

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
#ifndef SOC_SF32LB58X
static int is_jpg(const uint8_t *raw_data, size_t len);
#endif

#ifdef SOC_SF32LB58X
/* Decode with the SF32LB58X JPEG codec, then let EPIC draw the RGB buffer. */
typedef struct {
    lv_draw_buf_t *draw_buf;
    void *allocation;
} jpeg_session_t;

static lv_draw_buf_handlers_t jpeg_buf_handlers;

typedef struct {
    uint8_t *data;
    uint32_t size;
    void *allocation;
    const void *src;
    lv_image_src_t src_type;
} jpeg_source_t;

/* Keep one compressed source across callbacks; info-only queries have no close callback. */
typedef struct {
    jpeg_source_t source;
    lv_mutex_t mutex;
} jpeg_decoder_t;

static int jpeg_decode_memory(const uint8_t *jpeg_data, uint32_t jpeg_size,
                              lv_image_dsc_t *out_dsc, void **psram_allocation);

static void jpeg_source_close(jpeg_source_t *source)
{
    if (source->allocation != NULL) app_cache_free(source->allocation);
    if (source->src_type == LV_IMAGE_SRC_FILE) lv_free((void *)source->src);
    memset(source, 0, sizeof(*source));
}

/* Reuse aligned input between info and open, including arrays stored in Flash. */
static lv_result_t jpeg_source_open(const lv_image_decoder_dsc_t *dsc,
                                    jpeg_source_t *source)
{
    if (source->data != NULL && source->src_type == dsc->src_type &&
        ((dsc->src_type == LV_IMAGE_SRC_VARIABLE && source->src == dsc->src) ||
         (dsc->src_type == LV_IMAGE_SRC_FILE && lv_strcmp(source->src, dsc->src) == 0)))
        return LV_RESULT_OK;

    /* Replace the previous source instead of accumulating compressed images. */
    jpeg_source_close(source);
    lv_fs_file_t file;
    bool file_open = false;
    const uint8_t *array = NULL;
    uint32_t length = 0;

    if (dsc->src_type == LV_IMAGE_SRC_VARIABLE) {
        const lv_image_dsc_t *image = dsc->src;
        if (image == NULL || image->data == NULL || image->data_size < 3 ||
            image->data[0] != 0xff || image->data[1] != 0xd8 || image->data[2] != 0xff)
            return LV_RESULT_INVALID;
        array = image->data;
        length = image->data_size;
    }
    else if (dsc->src_type == LV_IMAGE_SRC_FILE) {
        const char *ext = lv_fs_get_ext(dsc->src);
        if (lv_strcmp(ext, "jpg") != 0 && lv_strcmp(ext, "jpeg") != 0)
            return LV_RESULT_INVALID;
        if (lv_fs_open(&file, dsc->src, LV_FS_MODE_RD) != LV_FS_RES_OK)
            return LV_RESULT_INVALID;
        file_open = true;
        if (lv_fs_seek(&file, 0, LV_FS_SEEK_END) != LV_FS_RES_OK ||
            lv_fs_tell(&file, &length) != LV_FS_RES_OK ||
            lv_fs_seek(&file, 0, LV_FS_SEEK_SET) != LV_FS_RES_OK)
            goto fail;
    }
    else {
        return LV_RESULT_INVALID;
    }

    if (length < 3 || length > UINT32_MAX - 63U) goto fail;
    source->allocation = app_cache_alloc(length + 63, JPEG_DECODER_CACHE_POOL);
    if (source->allocation == NULL) goto fail;
    source->data = (uint8_t *)RT_ALIGN((uintptr_t)source->allocation, 64);
    source->size = length;

    if (file_open) {
        uint32_t count = 0;
        lv_fs_res_t result = lv_fs_read(&file, source->data, length, &count);
        if (result != LV_FS_RES_OK || count != length) goto fail;
        lv_fs_close(&file);
        file_open = false;
    }
    else {
        lv_memcpy(source->data, array, length);
    }
    if (source->data[0] != 0xff || source->data[1] != 0xd8 || source->data[2] != 0xff)
        goto fail;
    /* Own file paths because the caller may release its path after an info query. */
    source->src_type = dsc->src_type;
    source->src = dsc->src_type == LV_IMAGE_SRC_FILE ? lv_strdup(dsc->src) : dsc->src;
    if (source->src == NULL) goto fail;
    return LV_RESULT_OK;

fail:
    if (file_open) lv_fs_close(&file);
    jpeg_source_close(source);
    return LV_RESULT_INVALID;
}

static lv_result_t decoder_info(lv_image_decoder_t *decoder,
                                 lv_image_decoder_dsc_t *dsc,
                                 lv_image_header_t *header)
{
    jpeg_decoder_t *context = decoder->user_data;
    /* Info and open use separate LVGL locks, so protect the shared source here. */
    if (lv_mutex_lock(&context->mutex) != LV_RESULT_OK) return LV_RESULT_INVALID;
    jpeg_source_t *source = &context->source;
    if (jpeg_source_open(dsc, source) != LV_RESULT_OK) {
        lv_mutex_unlock(&context->mutex);
        return LV_RESULT_INVALID;
    }
    uint32_t width = 0, height = 0;
    jpeg_decode_init();
    int ret = jpeg_decode_get_dimension(source->data, source->size, &width, &height);
    jpeg_decode_deinit();
    if (ret != 0 || width == 0 || height == 0 ||
        width > UINT16_MAX / 4 || height > UINT16_MAX) {
        jpeg_source_close(source);
        lv_mutex_unlock(&context->mutex);
        return LV_RESULT_INVALID;
    }
    /* Leave the compressed input cached for decoder_open. */
    lv_mutex_unlock(&context->mutex);

    memset(header, 0, sizeof(*header));
    header->magic = LV_IMAGE_HEADER_MAGIC;
    /* Only 16-bit displays use RGB565; all other depths use ARGB8888. */
#if LV_COLOR_DEPTH == 16
    header->cf = LV_COLOR_FORMAT_RGB565;
    header->stride = width * 2;
#else
    header->cf = LV_COLOR_FORMAT_ARGB8888;
    header->stride = width * 4;
#endif
    header->w = width;
    header->h = height;
    return LV_RESULT_OK;
}

static lv_result_t decoder_open(lv_image_decoder_t *decoder,
                                 lv_image_decoder_dsc_t *dsc)
{
    jpeg_decoder_t *context = decoder->user_data;
    jpeg_session_t *session = lv_malloc_zeroed(sizeof(*session));
    if (session == NULL) return LV_RESULT_INVALID;
    session->draw_buf = lv_malloc_zeroed(sizeof(*session->draw_buf));
    if (session->draw_buf == NULL) {
        lv_free(session);
        return LV_RESULT_INVALID;
    }
    lv_image_dsc_t image;
    if (lv_mutex_lock(&context->mutex) != LV_RESULT_OK) {
        lv_free(session->draw_buf);
        lv_free(session);
        return LV_RESULT_INVALID;
    }
    jpeg_source_t *source = &context->source;
    int ret = -1;
    /* Reuse info input, or load it if LVGL skipped info after a header-cache hit. */
    if (jpeg_source_open(dsc, source) == LV_RESULT_OK)
        ret = jpeg_decode_memory(source->data, source->size, &image, &session->allocation);
    if (ret != 0) jpeg_source_close(source);
    lv_mutex_unlock(&context->mutex);
    if (ret != 0) {
        lv_free(session->draw_buf);
        lv_free(session);
        return LV_RESULT_INVALID;
    }
    if (lv_draw_buf_init(session->draw_buf, image.header.w, image.header.h,
                        image.header.cf, image.header.stride,
                        (void *)image.data, image.data_size) != LV_RESULT_OK) {
        app_cache_free(session->allocation);
        lv_free(session->draw_buf);
        lv_free(session);
        return LV_RESULT_INVALID;
    }
    dsc->header = session->draw_buf->header;
    session->draw_buf->unaligned_data = session->allocation;
    session->draw_buf->handlers = &jpeg_buf_handlers;
    session->draw_buf->header.flags |= LV_IMAGE_FLAGS_ALLOCATED;
    dsc->decoded = session->draw_buf;
    dsc->user_data = session;
    return LV_RESULT_OK;
}

static void decoder_close(lv_image_decoder_t *decoder,
                           lv_image_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);
    jpeg_session_t *session = dsc->user_data;
    if (session != NULL) {
#if defined(DRV_EPIC_NEW_API) && LV_USE_DRAW_EPIC
        /* The render list still references these pixels after decoder close. */
        bool retained = lv_draw_epic_retain_layer_draw_buf(session->draw_buf);
        /* Never free pixels that an asynchronous render list still references. */
        RT_ASSERT(retained);
        LV_UNUSED(retained);
#else
        lv_draw_buf_destroy(session->draw_buf);
#endif
        lv_free(session);
    }
    dsc->user_data = NULL;
    dsc->decoded = NULL;
}
static int jpeg_decode_memory(const uint8_t *jpeg_data, uint32_t jpeg_size,
                              lv_image_dsc_t *out_dsc, void **psram_allocation)
{
    memset(out_dsc, 0, sizeof(*out_dsc));

    /* Query dimensions using the hardware codec API. */
    uint32_t width = 0, height = 0;
    jpeg_decode_init();
    int ret = jpeg_decode_get_dimension((uint8_t *)jpeg_data, jpeg_size, &width, &height);
    if (ret != 0 || width == 0 || height == 0 ||
        width > UINT16_MAX / 4 || height > UINT16_MAX ||
        (uint64_t)width * height * 4 > UINT32_MAX - 63) {
        LV_LOG_WARN("SiFli JPEG: get dim fail, ret=%d, w=%d, h=%d",
                     (int)ret, (int)width, (int)height);
        jpeg_decode_deinit();
        return -1;
    }

    /* The JPEG codec requires a 64-byte aligned output buffer. */
#if LV_COLOR_DEPTH == 16
    const char *fmt = "RGB565";
    uint32_t pixel_size = 2;
    lv_color_format_t cf = LV_COLOR_FORMAT_RGB565;
#else
    const char *fmt = "ARGB8888";
    uint32_t pixel_size = 4;
    lv_color_format_t cf = LV_COLOR_FORMAT_ARGB8888;
#endif
    uint32_t buf_size = width * height * pixel_size;
    void *allocation = NULL;
    uint8_t *decode_buf;
    allocation = app_cache_alloc(buf_size + 63, JPEG_DECODER_CACHE_POOL);
    decode_buf = allocation ? (uint8_t *)RT_ALIGN((uintptr_t)allocation, 64) : NULL;

    if (decode_buf == NULL) {
        LV_LOG_WARN("SiFli JPEG: malloc decode buf fail, size=%d", (int)buf_size);
        jpeg_decode_deinit();
        return -1;
    }

    /* Decode the complete image before submitting RGB pixels to EPIC. */
    ret = jpeg_decode2((uint8_t *)jpeg_data, jpeg_size, fmt, decode_buf, buf_size);
    jpeg_decode_deinit();

    if ((uint32_t)ret != jpeg_size) {
        LV_LOG_WARN("SiFli JPEG: decode fail, ret=%d, expect=%d", (int)ret, jpeg_size);
        app_cache_free(allocation);
        return -1;
    }

    *psram_allocation = allocation;

    /* Return uncompressed pixels, without the SF32LB57X JPEG flag. */
    out_dsc->header.magic = LV_IMAGE_HEADER_MAGIC;
    out_dsc->header.stride = width * pixel_size;
    out_dsc->header.w    = width;
    out_dsc->header.h    = height;
    out_dsc->header.cf   = cf;
    out_dsc->data        = decode_buf;
    out_dsc->data_size   = buf_size;

    LV_LOG_INFO("SiFli JPEG: decoded %dx%d, buf=%d bytes",
                 (int)width, (int)height, (int)buf_size);
    return 0;
}

#endif

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
    if (dec == NULL) return;
#ifdef SOC_SF32LB58X
    jpeg_decoder_t *context = lv_malloc_zeroed(sizeof(*context));
    if (context == NULL) {
        lv_image_decoder_delete(dec);
        return;
    }
    if (lv_mutex_init(&context->mutex) != LV_RESULT_OK) {
        lv_free(context);
        lv_image_decoder_delete(dec);
        return;
    }
    dec->user_data = context;
    jpeg_buf_handlers = *lv_draw_buf_get_handlers();
    jpeg_buf_handlers.buf_free_cb = app_cache_free;
#endif
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
#ifdef SOC_SF32LB58X
            jpeg_decoder_t *context = dec->user_data;
            /* Also release input retained by an info-only query. */
            jpeg_source_close(&context->source);
            lv_mutex_delete(&context->mutex);
            lv_free(context);
#endif
            lv_image_decoder_delete(dec);
            break;
        }
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#ifndef SOC_SF32LB58X
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

#endif /* !SOC_SF32LB58X */

#endif /*LV_USE_DRAW_EPIC*/
