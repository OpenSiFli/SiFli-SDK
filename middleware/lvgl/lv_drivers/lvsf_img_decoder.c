/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "board.h"
//#include "EventRecorder.h"
#include "drv_io.h"
#include "drv_flash.h"
#include "log.h"
#include "bf0_pm.h"
#include "section.h"
#if __has_include("app_mem.h")
    #include "app_mem.h"
#else
    #define app_cache_alloc(size,type) lv_mem_alloc(size)
    #define app_cache_free(p) lv_mem_free(p)
#endif
#ifdef RT_USING_DFS
    #include <dfs_posix.h>
#endif
#include "lv_draw_sw.h"
//#include "lv_draw.h"


#if LV_USE_GPU

static lv_res_t epic_cf_decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header);
static lv_res_t epic_cf_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);
static lv_res_t epic_cf_decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc, lv_coord_t x,
        lv_coord_t y, lv_coord_t len, uint8_t *buf);
static void epic_cf_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);
extern bool EPIC_SUPPORTED_CF(lv_img_cf_t cf);



static lv_res_t epic_cf_decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header)
{
    (void)decoder; /*Unused*/

    lv_img_src_t src_type = lv_img_src_get_type(src);
    if (src_type == LV_IMG_SRC_VARIABLE)
    {
        lv_img_cf_t cf = ((lv_img_dsc_t *)src)->header.cf;
        if (EPIC_SUPPORTED_CF(cf))
        {
            header->w  = ((lv_img_dsc_t *)src)->header.w;
            header->h  = ((lv_img_dsc_t *)src)->header.h;
            header->cf = ((lv_img_dsc_t *)src)->header.cf;
        }
        else
        {
            return LV_RES_INV;
        }
    }
    else
    {
        LV_LOG_WARN("Image get info found unknown src type");
        return LV_RES_INV;
    }
    return LV_RES_OK;
}

/**
 * Open a built in image
 * @param decoder the decoder where this function belongs
 * @param dsc pointer to decoder descriptor. `src`, `style` are already initialized in it.
 * @return LV_RES_OK: the info is successfully stored in `header`; LV_RES_INV: unknown format or other error.
 */
static lv_res_t epic_cf_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    /*Open the file if it's a file*/
    if (dsc->src_type == LV_IMG_SRC_VARIABLE)
    {
        /*The variables should have valid data*/
        if (((lv_img_dsc_t *)dsc->src)->data == NULL)
        {
            return LV_RES_INV;
        }
    }
    else
    {
        return LV_RES_INV;
    }

    lv_img_cf_t cf = dsc->header.cf;
    /*Process true color formats*/
    if (EPIC_SUPPORTED_CF(cf))
    {
        if (dsc->src_type == LV_IMG_SRC_VARIABLE)
        {
            /* In case of compressed formats the image stored in the ROM/RAM.
             * So simply give its pointer*/
            dsc->img_data = ((lv_img_dsc_t *)dsc->src)->data;
            dsc->img_data_size = ((lv_img_dsc_t *)dsc->src)->data_size;

            return LV_RES_OK;
        }
        else
        {
            return LV_RES_INV;
        }
    }
    /*Unknown format. Can't decode it.*/
    else
    {
        LV_LOG_WARN("Image decoder open: unknown color format");
        return LV_RES_INV;
    }
}

/**
 * Decode `len` pixels starting from the given `x`, `y` coordinates and store them in `buf`.
 * Required only if the "open" function can't return with the whole decoded pixel array.
 * @param decoder pointer to the decoder the function associated with
 * @param dsc pointer to decoder descriptor
 * @param x start x coordinate
 * @param y start y coordinate
 * @param len number of pixels to decode
 * @param buf a buffer to store the decoded pixels
 * @return LV_RES_OK: ok; LV_RES_INV: failed
 */
static lv_res_t epic_cf_decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc, lv_coord_t x,
        lv_coord_t y, lv_coord_t len, uint8_t *buf)
{
    return LV_RES_INV;

}

/**
 * Close the pending decoding. Free resources etc.
 * @param decoder pointer to the decoder the function associated with
 * @param dsc pointer to decoder descriptor
 */
static void epic_cf_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    (void)decoder; /*Unused*/

}

#endif

#if defined(RT_USING_DFS)
static lv_res_t file_decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header)
{
    (void)decoder; /*Unused*/

    lv_img_src_t src_type = lv_img_src_get_type(src);
    if (src_type == LV_IMG_SRC_FILE)
    {
        int fd;
        int size;

        fd = open(src, O_RDONLY);

        if (fd >= 0)
        {
            size = read(fd, header, sizeof(*header));
            close(fd);

            if (size < sizeof(*header))
            {
                LV_LOG_WARN("Read error: %d\n", size);
                goto __ERROR;
            }

            if (LV_IMG_CF_TRUE_COLOR != header->cf
                    && LV_IMG_CF_TRUE_COLOR_ALPHA != header->cf
                    && LV_IMG_CF_RAW != header->cf
                    && LV_IMG_CF_RAW_ALPHA != header->cf
               )
            {
                LV_LOG_WARN("Cf error: %d\n", header->cf);
                goto __ERROR;
            }
        }
        else
        {
            LV_LOG_WARN("Cannot open: %s\n", src);
            goto __ERROR;
        }
    }
    else
    {
        LV_LOG_WARN("Src type is wrong: %d\n", src_type);
        goto __ERROR;
    }
    return LV_RES_OK;

__ERROR:
    LV_LOG_WARN("Image get info found unknown src type");
    return LV_RES_INV;
}

/**
 * Open a built in image
 * @param decoder the decoder where this function belongs
 * @param dsc pointer to decoder descriptor. `src`, `style` are already initialized in it.
 * @return LV_RES_OK: the info is successfully stored in `header`; LV_RES_INV: unknown format or other error.
 */
static lv_res_t file_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    /*Open the file if it's a file*/
    if (dsc->src_type != LV_IMG_SRC_FILE)
    {
        return LV_RES_INV;
    }

    lv_img_cf_t cf = dsc->header.cf;
    /*Process true color formats*/
    if (cf != LV_IMG_CF_TRUE_COLOR && cf != LV_IMG_CF_TRUE_COLOR_ALPHA
            && cf != LV_IMG_CF_RAW && cf != LV_IMG_CF_RAW_ALPHA)
    {
        LV_LOG_WARN("Image decoder open: unknown color format");
        return LV_RES_INV;
    }

    struct stat file_stat;
    int res = stat(dsc->src, &file_stat);
    if (0 != res)
    {
        LV_LOG_WARN("Cannot stat: %s\n", dsc->src);
        return LV_RES_INV;
    }

    int fd = open(dsc->src, O_RDONLY);
    if (fd < 0)
    {
        LV_LOG_WARN("Cannot open: %s\n", dsc->src);
        return LV_RES_INV;
    }

    dsc->img_data_size = file_stat.st_size - sizeof(lv_img_header_t);
    lv_res_t ret = LV_RES_INV;

#if defined(RT_USING_MTD_NAND)
    dsc->img_data = app_cache_alloc(dsc->img_data_size, IMAGE_CACHE_PSRAM);
    if (dsc->img_data == NULL)
    {
        LV_LOG_WARN("Memory allocation failed for image data\n");
        close(fd);
        return LV_RES_INV;
    }

    lseek(fd, sizeof(lv_img_header_t), SEEK_SET);
    size_t bytes_read = read(fd, (void *)dsc->img_data, dsc->img_data_size);
    if (bytes_read != dsc->img_data_size)
    {
        LV_LOG_WARN("Read bytes mismatch: %u/%u\n", bytes_read, dsc->img_data_size);
        app_cache_free((void *)dsc->img_data);
        dsc->img_data = NULL;
        close(fd);
        return LV_RES_INV;
    }
    dsc->user_data = (void *)dsc->img_data; /* Mark as allocated memory */
    ret = LV_RES_OK;

#elif defined(RT_USING_MTD_NOR)
    /* Check if the file is on NOR flash (MTD device) */
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
            dsc->img_data = (const uint8_t *)(addr + sizeof(lv_img_header_t));
            dsc->user_data = NULL; /* Direct mapping, no need to free */
            use_direct_mapping = true;
            ret = LV_RES_OK;
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
        dsc->img_data = app_cache_alloc(dsc->img_data_size, IMAGE_CACHE_PSRAM);
        if (dsc->img_data == NULL)
        {
            LV_LOG_WARN("Memory allocation failed for image data\n");
            close(fd);
            return LV_RES_INV;
        }

        lseek(fd, sizeof(lv_img_header_t), SEEK_SET);
        size_t bytes_read = read(fd, (void *)dsc->img_data, dsc->img_data_size);
        if (bytes_read != dsc->img_data_size)
        {
            LV_LOG_WARN("Read bytes mismatch: %u/%u\n", bytes_read, dsc->img_data_size);
            app_cache_free((void *)dsc->img_data);
            dsc->img_data = NULL;
            close(fd);
            return LV_RES_INV;
        }
        dsc->user_data = (void *)dsc->img_data; /* Mark as allocated memory */
        ret = LV_RES_OK;
    }
#endif
    close(fd);
    return ret;
}

/**
 * Decode `len` pixels starting from the given `x`, `y` coordinates and store them in `buf`.
 * Required only if the "open" function can't return with the whole decoded pixel array.
 * @param decoder pointer to the decoder the function associated with
 * @param dsc pointer to decoder descriptor
 * @param x start x coordinate
 * @param y start y coordinate
 * @param len number of pixels to decode
 * @param buf a buffer to store the decoded pixels
 * @return LV_RES_OK: ok; LV_RES_INV: failed
 */
static lv_res_t file_decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc, lv_coord_t x,
                                       lv_coord_t y, lv_coord_t len, uint8_t *buf)
{
    return LV_RES_INV;

}

/**
 * Close the pending decoding. Free resources etc.
 * @param decoder pointer to the decoder the function associated with
 * @param dsc pointer to decoder descriptor
 */
static void file_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    (void)decoder; /*Unused*/

    if (dsc->user_data)
    {
        app_cache_free((void *)dsc->img_data);
        dsc->img_data = NULL;
        dsc->user_data = NULL;
    }

}
#endif /* RT_USING_DFS */


void lv_sifli_img_decoder(void)
{
#if LV_USE_GPU
    {
        lv_img_decoder_t *decoder;

        /*Create a decoder for the built in color format*/
        decoder = lv_img_decoder_create();
        RT_ASSERT(decoder);
        lv_img_decoder_set_info_cb(decoder, epic_cf_decoder_info);
        lv_img_decoder_set_open_cb(decoder, epic_cf_decoder_open);
        lv_img_decoder_set_read_line_cb(decoder, epic_cf_decoder_read_line);
        lv_img_decoder_set_close_cb(decoder, epic_cf_decoder_close);
    }
#endif


#if defined(RT_USING_DFS)
    {
        lv_img_decoder_t *decoder;

        /*Create a decoder for the built in color format*/
        decoder = lv_img_decoder_create();
        RT_ASSERT(decoder);
        lv_img_decoder_set_info_cb(decoder, file_decoder_info);
        lv_img_decoder_set_open_cb(decoder, file_decoder_open);
        lv_img_decoder_set_read_line_cb(decoder, file_decoder_read_line);
        lv_img_decoder_set_close_cb(decoder, file_decoder_close);

    }
#endif /* RT_USING_DFS */

}
