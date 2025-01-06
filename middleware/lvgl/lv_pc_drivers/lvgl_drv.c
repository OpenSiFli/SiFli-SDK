/**
  ******************************************************************************
  * @file   lvgl_drv.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "littlevgl2rtt.h"
#include "monitor.h"
#include "mouse.h"
#include "mousewheel.h"
#include "keyboard.h"   // Init display driver

extern void perf_monitor(struct _lv_disp_drv_t *disp_drv, uint32_t time, uint32_t px);
static lv_indev_t *kb_indev;
static lv_indev_t *mouse_indev;
static lv_indev_t *mousewheel_indev;


#if defined(RT_USING_DFS)
#include <dfs_posix.h>
#ifdef WIN32
    #define open(filename,flag)  rt_open(filename,flag,0x644)
    #define close(handle) rt_close(handle)
    #define read rt_read
    #define write rt_write
    #define lseek rt_lseek
#endif
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

        fd = open(dsc->src, O_RDONLY);
        if (fd >= 0)
        {
            offset = lseek(fd, 0, SEEK_END);
            if (offset > sizeof(lv_img_header_t))
            {
                dsc->img_data_size = offset - sizeof(lv_img_header_t);
                dsc->img_data = malloc(dsc->img_data_size);
                RT_ASSERT(dsc->img_data);
                offset = lseek(fd, sizeof(lv_img_header_t), SEEK_SET);
                if (sizeof(lv_img_header_t) == offset)
                {
                    offset = read(fd, (void *)dsc->img_data, dsc->img_data_size);
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
            close(fd);
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
#endif /* RT_USING_DFS */


void lv_hal_init(const char *name)
{
    monitor_init();
    {
        static lv_disp_drv_t disp_drv;
        static lv_disp_draw_buf_t disp_buf1;
        static lv_color_t buf1_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
        lv_disp_drv_init(&disp_drv);
        lv_disp_draw_buf_init(&disp_buf1, buf1_1, buf1_1, LV_HOR_RES_MAX * LV_VER_RES_MAX);

        disp_drv.draw_buf = &disp_buf1;
        disp_drv.flush_cb = monitor_flush;
        disp_drv.monitor_cb = perf_monitor;
        lv_disp_drv_register(&disp_drv);
    }

    // Init mouse driver
    {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);          /*Basic initialization*/
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = mouse_read;         /*This function will be called periodically (by the library) to get the mouse position and state*/
        mouse_indev = lv_indev_drv_register(&indev_drv);
#ifdef BSP_USING_LVGL_INPUT_AGENT
        {
            extern void lv_indev_agent_init(lv_indev_drv_t *drv);
            lv_indev_agent_init(&indev_drv);
        }
#endif
    }

    // Init mouse wheel driver
    {
        static lv_indev_drv_t wheel_drv;
        lv_indev_drv_init(&wheel_drv);          /*Basic initialization*/
        wheel_drv.type = LV_INDEV_TYPE_ENCODER;
        wheel_drv.read_cb = mousewheel_read;         /*This function will be called periodically (by the library) to get the mouse position and state*/
        mousewheel_indev = lv_indev_drv_register(&wheel_drv);
#ifdef BSP_USING_LVGL_INPUT_AGENT
        {
            extern void lv_indev_agent_init(lv_indev_drv_t *drv);
            lv_indev_agent_init(&wheel_drv);
        }
#endif
    }

    // Init keyboard driver
    {
        static lv_indev_drv_t kb_drv;
        //lv_indev_t *kb_indev;
        lv_indev_drv_init(&kb_drv);
        kb_drv.type = LV_INDEV_TYPE_KEYPAD;
        kb_drv.read_cb = keyboard_read;
        kb_indev = lv_indev_drv_register(&kb_drv);
#ifdef BSP_USING_LVGL_INPUT_AGENT
        lv_indev_agent_init(&kb_drv);
#endif
    }

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


lv_indev_t *keypad_get_indev_handler(void)
{
    return kb_indev;
}

lv_indev_t *wheel_get_indev_handler(void)
{
    return mousewheel_indev;
}

lv_indev_t *touch_get_indev_handler(void)
{
    return mouse_indev;
}

uint8_t fb_set_cmpr_rate(uint8_t rate)
{
    return 0;
}

uint8_t fb_get_cmpr_rate(void)
{
    return 0;
}


void *get_disp_buf(uint32_t size)
{
    lv_disp_t *disp = lv_disp_get_default();
    uint32_t buf_size = disp->driver.buffer->size * LV_COLOR_DEPTH / 8;


    if (buf_size >= size)
    {
        return (void *) disp->driver.buffer->buf_act;
    }

    return NULL;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
