/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lvsf_img_decoder.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "../../draw/lv_image_decoder_private.h"
#include "../../../lvgl.h"
#if LV_USE_DRAW_EPIC

#include "../../misc/lv_fs_private.h"
#include <string.h>
#ifdef RT_USING_DFS
    #include <dfs_posix.h>
#endif

/*********************
 *      DEFINES
 *********************/

#define DECODER_NAME    "EZIP"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static lv_result_t decoder_info(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc, lv_image_header_t *header);
static lv_result_t decoder_open(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc);
static void decoder_close(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/


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

        if (img_dsc->header.flags & LV_IMAGE_FLAGS_EZIP)
        {
            header->cf = LV_COLOR_FORMAT_RAW;
            header->w = img_dsc->header.w;
            header->h = img_dsc->header.h;
            header->stride = img_dsc->header.w * 3;
            header->flags = img_dsc->header.flags;
            return LV_RESULT_OK;
        }
    }
    else if (src_type == LV_IMAGE_SRC_FILE)
    {
        const char *fn = src;

        lv_fs_file_t *f = lv_malloc(sizeof(lv_fs_file_t));

        lv_fs_res_t res;
        res = lv_fs_open(f, fn, LV_FS_MODE_RD);
        if (res != LV_FS_RES_OK)
        {
            LV_LOG_WARN("Cannot open: %s\n", fn);
            lv_free(f);
            return LV_RESULT_INVALID;
        }

        uint32_t size;

        res = lv_fs_read(f, header, sizeof(*header), &size);
        lv_fs_close(f);
        lv_free(f);

        if (size < sizeof(*header))
        {
            LV_LOG_WARN("Read error: %d\n", size);
            return LV_RESULT_INVALID;
        }

        if (LV_IMG_CF_TRUE_COLOR != header->cf
                && LV_IMG_CF_TRUE_COLOR_ALPHA != header->cf
                && LV_IMG_CF_RAW != header->cf
                && LV_IMG_CF_RAW_ALPHA != header->cf
           )
        {
            LV_LOG_WARN("Cf error: %d\n", header->cf);
            return LV_RESULT_INVALID;
        }

        return LV_RESULT_OK;
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
    lv_result_t ret = LV_RESULT_INVALID;

    if (dsc->src_type == LV_IMAGE_SRC_VARIABLE)
    {
        const lv_image_dsc_t *img_dsc = dsc->src;
        if (img_dsc->header.flags & LV_IMAGE_FLAGS_EZIP)
        {
            LV_ASSERT(!dsc->decoded);
            lv_draw_buf_t *p_decoded = lv_malloc_zeroed(sizeof(lv_draw_buf_t));
            LV_ASSERT(p_decoded != NULL);

            memcpy(&p_decoded->header, &dsc->header, sizeof(p_decoded->header));
            p_decoded->data = (uint8_t *)img_dsc->data;
            p_decoded->data_size = img_dsc->data_size;
            p_decoded->unaligned_data = (uint8_t *)img_dsc->data;
            p_decoded->handlers = lv_draw_buf_get_handlers();
            dsc->decoded = p_decoded;
            ret = LV_RESULT_OK;
        }
    }
    else if (dsc->src_type == LV_IMAGE_SRC_FILE)
    {
        const char *fn = dsc->src;

        lv_fs_file_t *f = lv_malloc(sizeof(lv_fs_file_t));
        LV_ASSERT(f != NULL);

        lv_fs_res_t res = lv_fs_open(f, fn, LV_FS_MODE_RD);
        if (LV_FS_RES_OK != res)
        {
            LV_LOG_WARN("Cannot open: %s\n", fn);
            lv_free(f);
            return LV_RESULT_INVALID;
        }

        uint32_t file_size;
        lv_fs_seek(f, 0, LV_FS_SEEK_END);
        lv_fs_tell(f, &file_size);
        lv_fs_seek(f, 0, LV_FS_SEEK_SET);

        if (file_size <= sizeof(lv_image_header_t))
        {
            LV_LOG_WARN("File too small: %s\n", fn);
            lv_fs_close(f);
            lv_free(f);
            return LV_RESULT_INVALID;
        }

        LV_ASSERT(!dsc->decoded);
        lv_draw_buf_t *p_decoded = lv_malloc_zeroed(sizeof(lv_draw_buf_t));
        LV_ASSERT(p_decoded != NULL);

        memcpy(&p_decoded->header, &dsc->header, sizeof(p_decoded->header));
        p_decoded->data_size = file_size - sizeof(lv_image_header_t);

#if defined(RT_USING_MTD_NAND)
        p_decoded->data = lv_malloc(p_decoded->data_size);
        if (p_decoded->data == NULL)
        {
            LV_LOG_WARN("Memory allocation failed for image data\n");
            lv_free(p_decoded);
            lv_fs_close(f);
            lv_free(f);
            return LV_RESULT_INVALID;
        }

        uint32_t br;
        lv_fs_seek(f, sizeof(lv_image_header_t), LV_FS_SEEK_SET);
        res = lv_fs_read(f, (void *)p_decoded->data, p_decoded->data_size, &br);
        if (LV_FS_RES_OK != res || p_decoded->data_size != br)
        {
            LV_LOG_WARN("Read bytes mismatch: %u/%u\n", br, p_decoded->data_size);
            lv_free(p_decoded->data);
            lv_free(p_decoded);
            lv_fs_close(f);
            lv_free(f);
            return LV_RESULT_INVALID;
        }
        dsc->user_data = p_decoded->data; /* Mark as allocated memory */
        ret = LV_RESULT_OK;

#elif defined(RT_USING_MTD_NOR)
        /* Check if the file is on NOR flash (MTD device) */
        /* NOTE: The file descriptor in lv_fs_file_t is offset by 1 */
        int fd = (int)f->file_d - 1;
        rt_device_t dev = RT_NULL;
        struct dfs_fd *dfs_fd = fd_get(fd);
        if (dfs_fd && dfs_fd->fs && dfs_fd->fs->dev_id)
        {
            dev = dfs_fd->fs->dev_id;
        }

        /* Try to get physical address if device is MTD type */
        bool use_direct_mapping = false;
        if (dev && dev->type == RT_Device_Class_MTD)
        {
            rt_uint32_t addr;
            if (ioctl(fd, F_GET_PHY_ADDR, &addr) == 0)
            {
                p_decoded->data = (uint8_t *)(addr + sizeof(lv_image_header_t));
                dsc->user_data = NULL; /* Direct mapping, no need to free */
                use_direct_mapping = true;
                ret = LV_RESULT_OK;
            }
        }

        /* Release the fd reference obtained from fd_get */
        if (dfs_fd)
        {
            fd_put(dfs_fd);
        }

        /* Not MTD device (e.g., SD card) or ioctl failed, must read into memory */
        if (!use_direct_mapping)
        {
            p_decoded->data = lv_malloc(p_decoded->data_size);
            if (p_decoded->data == NULL)
            {
                LV_LOG_WARN("Memory allocation failed for image data\n");
                lv_free(p_decoded);
                lv_fs_close(f);
                lv_free(f);
                return LV_RESULT_INVALID;
            }

            uint32_t br;
            lv_fs_seek(f, sizeof(lv_image_header_t), LV_FS_SEEK_SET);
            res = lv_fs_read(f, (void *)p_decoded->data, p_decoded->data_size, &br);
            if (LV_FS_RES_OK != res || p_decoded->data_size != br)
            {
                LV_LOG_WARN("Read bytes mismatch: %u/%u\n", br, p_decoded->data_size);
                lv_free(p_decoded->data);
                lv_free(p_decoded);
                lv_fs_close(f);
                lv_free(f);
                return LV_RESULT_INVALID;
            }
            dsc->user_data = p_decoded->data; /* Mark as allocated memory */
            ret = LV_RESULT_OK;
        }
#endif

        dsc->decoded = p_decoded;
        lv_fs_close(f);
        lv_free(f);
    }

    return ret;
}

/**
 * Free the allocated resources
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 */
static void decoder_close(lv_image_decoder_t *decoder, lv_image_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);
    if (dsc->user_data)
    {
        lv_free(dsc->user_data);
        dsc->user_data = NULL;
    }

    if (dsc->decoded) lv_free((void *)dsc->decoded);
}

#endif /*LV_USE_DRAW_EPIC*/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_sifli_img_decoder(void)
{
#if LV_USE_DRAW_EPIC
    lv_image_decoder_t *dec = lv_image_decoder_create();
    lv_image_decoder_set_info_cb(dec, decoder_info);
    lv_image_decoder_set_open_cb(dec, decoder_open);
    lv_image_decoder_set_close_cb(dec, decoder_close);

    dec->name = DECODER_NAME;
#endif /* LV_USE_DRAW_EPIC */
}

void lv_sifli_img_decoder_deinit(void)
{
#if LV_USE_DRAW_EPIC
    lv_image_decoder_t *dec = NULL;
    while ((dec = lv_image_decoder_get_next(dec)) != NULL)
    {
        if (dec->info_cb == decoder_info)
        {
            lv_image_decoder_delete(dec);
            break;
        }
    }
#endif /* LV_USE_DRAW_EPIC */
}
