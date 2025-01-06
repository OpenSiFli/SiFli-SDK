#include "rtthread.h"
#if defined(RT_USING_DFS)
#include <dfs_posix.h>
#include "lvgl.h"

static lv_res_t file_decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header)
{
    (void)decoder; /*Unused*/

    lv_img_src_t src_type = lv_img_src_get_type(src);
    if (src_type == LV_IMG_SRC_FILE)
    {
        int fd;
        int size;

        fd = rt_open(src, O_RDONLY);

        if (fd >= 0)
        {
            size = rt_read(fd, header, sizeof(*header));
            rt_close(fd);

            if (size < sizeof(*header))
            {
                goto __ERROR;
            }

            if (LV_IMG_CF_TRUE_COLOR != header->cf
                    && LV_IMG_CF_TRUE_COLOR_ALPHA != header->cf
#if defined(HAL_EZIP_MODULE_ENABLED)
                    && LV_IMG_CF_RAW != header->cf
                    && LV_IMG_CF_RAW_ALPHA != header->cf
#endif /* HAL_EZIP_MODULE_ENABLED */
               )
            {
                goto __ERROR;
            }
        }
        else
        {
            goto __ERROR;
        }
    }
    else
    {
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
    if (cf == LV_IMG_CF_TRUE_COLOR || cf == LV_IMG_CF_TRUE_COLOR_ALPHA
#if defined(HAL_EZIP_MODULE_ENABLED)
            || cf == LV_IMG_CF_RAW || cf == LV_IMG_CF_RAW_ALPHA
#endif  /* HAL_EZIP_MODULE_ENABLED */
       )
    {
        int fd;
        lv_res_t ret = LV_RES_INV;
        off_t offset;

        fd = rt_open(dsc->src, O_RDONLY);
        if (fd >= 0)
        {
            offset = rt_lseek(fd, 0, SEEK_END);
            if (offset > sizeof(lv_img_header_t))
            {
                dsc->img_data_size = offset - sizeof(lv_img_header_t);
                dsc->img_data = malloc(dsc->img_data_size);
                RT_ASSERT(dsc->img_data);
                offset = rt_lseek(fd, sizeof(lv_img_header_t), SEEK_SET);
                if (sizeof(lv_img_header_t) == offset)
                {
                    offset = rt_read(fd, (void *)dsc->img_data, dsc->img_data_size);
                    if (dsc->img_data_size = offset)
                    {
                        ret = LV_RES_OK;
                    }
                    else
                    {
                        free((void *)dsc->img_data);
                        dsc->img_data = NULL;
                    }
                }
            }
            rt_close(fd);
        }
        return ret;
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

    if (dsc->img_data)
    {
        free((void *)dsc->img_data);
    }
}

void lv_win32_imgdec_init(void)
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

