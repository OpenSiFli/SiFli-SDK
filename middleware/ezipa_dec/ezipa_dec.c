/**
  ******************************************************************************
  * @file   ezipa_dec.c
  * @author Sifli software development team
  * @brief EZIP-A Decoder.
 *
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


#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <string.h>
#include "board.h"
#include "drv_epic.h"
#include "ezipa_dec.h"
#include "log.h"
#ifdef RT_USING_DFS
    #include "dfs_posix.h"
#endif /* RT_USING_DFS */


#define EPIC_TIMEOUT_MS  (100)

#define EZ_IS_IN_SRAM_RANGE(addr)    ((((addr) >= HPSYS_RAM0_BASE) && ((addr) < HPSYS_RAM_END)) ? true : false)


#ifdef RT_USING_DFS
#define EZ_SWAP16(val)    ((((uint16_t)(val) >> 8) & 0xFF) | (((uint16_t)(val) << 8) & 0xFF00))
#define EZ_SWAP32(val)    ((((uint32_t)(val) >> 24) & 0xFF) | (((uint32_t)(val) >> 8) & 0xFF00) \
                           | (((uint32_t)(val) << 24) & 0xFF000000) | (((uint32_t)(val) << 8) & 0xFF0000))

#define EZ_PALETTE_TBL_SIZE(palette_num)      ((palette_num) * 4)
#define EZ_FRAME_OFFSET_TBL_SIZE(frame_num)      ((frame_num) * 4)
#define EZ_FRAME_OFFSET_TBL_POS(palette_num)  (sizeof(ezipa_hdr_t) + EZ_PALETTE_TBL_SIZE(palette_num) + sizeof(ezipa_anim_ctrl_t))
#define EZ_PALETTE_TBL_POS(palette_num)       (sizeof(ezipa_hdr_t))
#define EZ_ANIM_CTRL_POS(palette_num)         (sizeof(ezipa_hdr_t) + EZ_PALETTE_TBL_SIZE(palette_num))

/* preceding bytes before the actual ezipa header in the file */
#define EZ_FILE_LEADING_HDR_SIZE   (4)

typedef struct
{
    uint8_t data0[13];
    uint8_t palette_num;
    uint8_t data1[2];
} ezipa_hdr_t;


typedef struct
{
    uint32_t frame_num;
    uint32_t play_num;
} ezipa_anim_ctrl_t;

typedef struct __PACKED
{
    uint32_t seq_num;
    uint32_t width;
    uint32_t height;
    uint32_t x_offset;
    uint32_t y_offset;
    uint16_t delay_num;
    uint16_t delay_den;
    uint8_t dispose_op;
    uint8_t blend_op;
    uint16_t frame_size_h;
    uint16_t frame_size_l;
//    uint8_t reserved[4];//TODO
} ezipa_frame_hdr_t;


#if 0
typedef struct
{
    /* EZ_FILE_LEADING_HDR_SIZE */
    uint8_t fmt[4];
    ezipa_hdr_t header;
    uint32_t palette[palette_num];
    ezipa_anim_ctrl_t anim_ctrl;
    uint32_t frame_offset[frame_num];
    uint8_t frame_data;
} ez_file_fmt_t
#endif

#endif /* RT_USING_DFS */

/** canvas pixel size in bits  */
static const uint8_t ezipa_canvas_pixel_size_tbl[] =
{
    16, 24, 24, 32, 8, 8, 4
};

#define EZIP_CANVAS_PIXEL_SIZE_TBL_LEN   (sizeof(ezipa_canvas_pixel_size_tbl) / sizeof(ezipa_canvas_pixel_size_tbl[0]))


static uint8_t ezipa_get_canvas_pixel_size(uint8_t color_fmt)
{
    RT_ASSERT(color_fmt < EZIP_CANVAS_PIXEL_SIZE_TBL_LEN);

    return ezipa_canvas_pixel_size_tbl[color_fmt];
}

static void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    rt_err_t err;
    ezipa_obj_t *obj;

    RT_ASSERT(epic->user_data);
    obj = (ezipa_obj_t *)epic->user_data;

    err = rt_sem_release(&obj->sem);
    RT_ASSERT(RT_EOK == err);
}

static int32_t ezipa_init_output_buf(ezipa_obj_t *obj)
{
    EPIC_LayerConfigTypeDef *input_layer;
    uint8_t input_layer_num;
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_HandleTypeDef *epic;
    HAL_StatusTypeDef status;
    rt_err_t err;
    EPIC_FillingCfgTypeDef fill_param;

    RT_ASSERT(obj && obj->output_buf);

    if (EZ_IS_IN_SRAM_RANGE((uint32_t)obj->output_buf))
    {
        uint8_t *data;
        data = obj->output_buf + (obj->curr_frame.y_offset * obj->header.width + obj->curr_frame.x_offset) * obj->pixel_size;
        for (uint32_t i = 0; i < obj->curr_frame.height; i++)
        {
            memset(data, 0, obj->pixel_size * obj->curr_frame.width);
            data += obj->header.width * obj->pixel_size;
        }

        if (IS_DCACHED_RAM((uint32_t)obj->output_buf))
        {
            mpu_dcache_clean(obj->output_buf, obj->header.width * obj->header.height * obj->pixel_size);
        }
    }
    else if (obj->alpha_enabled)
    {
        input_layer_num = 1;

        input_layer = rt_malloc(sizeof(*input_layer));
        RT_ASSERT(input_layer);
        HAL_EPIC_LayerConfigInit(input_layer);
        HAL_EPIC_LayerConfigInit(&output_layer);

        /* update output buf */
        input_layer[0].data = (uint8_t *)obj->output_buf;
        input_layer[0].color_mode = obj->epic_color_fmt;
        input_layer[0].height = obj->header.height;
        input_layer[0].x_offset = 0;
        input_layer[0].y_offset = 0;
        input_layer[0].total_width = obj->header.width;
        input_layer[0].width = obj->header.width;
        input_layer[0].color_en = false;
        input_layer[0].alpha = 0;

        output_layer.data = (uint8_t *)obj->output_buf
                            + (obj->curr_frame.y_offset * obj->header.width + obj->curr_frame.x_offset) * obj->pixel_size;
        output_layer.color_mode = obj->epic_color_fmt;
        output_layer.height = obj->curr_frame.height;
        output_layer.x_offset = obj->curr_frame.x_offset;
        output_layer.y_offset = obj->curr_frame.y_offset;
        output_layer.total_width = obj->header.width;
        output_layer.width = obj->curr_frame.width;
        output_layer.color_en = false;
        output_layer.alpha = 0;

        epic = drv_get_epic_handle();
        RT_ASSERT(epic);
        epic->XferCpltCallback = epic_cplt_callback;
        epic->user_data = (void *)obj;

        status = HAL_EPIC_BlendStartEx_IT(epic, input_layer, input_layer_num, &output_layer);
        RT_ASSERT(HAL_OK == status);

        /* wait for complete */
        err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
        RT_ASSERT(RT_EOK == err);

        rt_free(input_layer);
    }
    else
    {
        /* fill output buf with black color */
        fill_param.start = (uint8_t *)obj->output_buf
                           + (obj->curr_frame.y_offset * obj->header.width + obj->curr_frame.x_offset) * obj->pixel_size;
        fill_param.color_mode = obj->epic_color_fmt;
        fill_param.height = obj->curr_frame.height;
        fill_param.total_width = obj->header.width;
        fill_param.width = obj->curr_frame.width;
        fill_param.color_r = 0;
        fill_param.color_g = 0;
        fill_param.color_b = 0;
        fill_param.alpha = EPIC_LAYER_OPAQUE;

        epic = drv_get_epic_handle();
        RT_ASSERT(epic);
        epic->XferCpltCallback = epic_cplt_callback;
        epic->user_data = (void *)obj;

        status = HAL_EPIC_FillStart_IT(epic, &fill_param);
        RT_ASSERT(RT_EOK == status);

        /* wait for complete */
        err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
        RT_ASSERT(RT_EOK == err);
    }

    return 0;
}

static int32_t ezipa_draw_disp_op_none(ezipa_obj_t *obj, ezipa_canvas_t *canvas)
{
    EPIC_LayerConfigTypeDef *input_layer;
    uint8_t input_layer_num;
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_HandleTypeDef *epic;
    HAL_StatusTypeDef status;
    rt_err_t err;
    uint8_t canvas_pixel_size;

    input_layer_num = 2;
    input_layer = rt_malloc(input_layer_num * sizeof(*input_layer));
    RT_ASSERT(input_layer);

    HAL_EPIC_LayerConfigInit(&input_layer[0]);
    HAL_EPIC_LayerConfigInit(&input_layer[1]);
    HAL_EPIC_LayerConfigInit(&output_layer);

    /* update output buf */
    input_layer[1].data = (uint8_t *)obj->ezipa_data;
    input_layer[1].color_mode = EPIC_INPUT_EZIP;
    input_layer[1].height = obj->next_frame.height;
    input_layer[1].x_offset = obj->next_frame.x_offset;
    input_layer[1].y_offset = obj->next_frame.y_offset;
    input_layer[1].total_width = obj->next_frame.width;
    input_layer[1].width = obj->next_frame.width;
    input_layer[1].color_en = false;
    input_layer[1].alpha = EPIC_LAYER_OPAQUE;

    input_layer[0].data = (uint8_t *)obj->output_buf;
    input_layer[0].color_mode = obj->epic_color_fmt;
    input_layer[0].height = obj->header.height;
    input_layer[0].x_offset = 0;
    input_layer[0].y_offset = 0;
    input_layer[0].total_width = obj->header.width;
    input_layer[0].width = obj->header.width;
    input_layer[0].color_en = false;
    input_layer[0].alpha = EPIC_LAYER_OPAQUE;

    output_layer.data = (uint8_t *)obj->output_buf;
    output_layer.color_mode = obj->epic_color_fmt;
    output_layer.height = obj->header.height;
    output_layer.x_offset = 0;
    output_layer.y_offset = 0;
    output_layer.total_width = obj->header.width;
    output_layer.width = obj->header.width;
    output_layer.color_en = false;
    output_layer.alpha = EPIC_LAYER_OPAQUE;

    epic = drv_get_epic_handle();
    RT_ASSERT(epic);
    epic->XferCpltCallback = epic_cplt_callback;
    epic->user_data = (void *)obj;

    status = HAL_EPIC_BlendStartEx_IT(epic, input_layer, input_layer_num, &output_layer);
    RT_ASSERT(HAL_OK == status);

    /* wait for complete */
    err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    /* update canvas buf */
    epic->XferCpltCallback = epic_cplt_callback;
    epic->user_data = (void *)obj;

    input_layer[1].data = (uint8_t *)obj->output_buf;
    input_layer[1].color_mode = obj->epic_color_fmt;
    input_layer[1].height = obj->header.height;
    input_layer[1].x_offset = canvas->x_offset;
    input_layer[1].y_offset = canvas->y_offset;
    input_layer[1].total_width = obj->header.width;
    input_layer[1].width = obj->header.width;
    input_layer[1].color_en = false;
    input_layer[1].alpha = EPIC_LAYER_OPAQUE;

    input_layer[0].data = (uint8_t *)canvas->buf;
    input_layer[0].color_mode = canvas->color_fmt;
    input_layer[0].height = canvas->height;
    input_layer[0].x_offset = 0;
    input_layer[0].y_offset = 0;
    input_layer[0].total_width = canvas->width;
    input_layer[0].width = canvas->width;
    input_layer[0].color_en = false;
    input_layer[0].alpha = EPIC_LAYER_OPAQUE;


    canvas_pixel_size = ezipa_get_canvas_pixel_size(canvas->color_fmt);
    output_layer.data = (uint8_t *)canvas->buf + ((canvas_pixel_size * (canvas->mask_y_offset * canvas->width + canvas->mask_x_offset)) >> 3);
    output_layer.color_mode = canvas->color_fmt;
    output_layer.height = canvas->mask_height;
    output_layer.x_offset = canvas->mask_x_offset;
    output_layer.y_offset = canvas->mask_y_offset;
    output_layer.total_width = canvas->width;
    output_layer.width = canvas->mask_width;
    output_layer.color_en = false;
    output_layer.alpha = EPIC_LAYER_OPAQUE;

    status = HAL_EPIC_BlendStartEx_IT(epic, input_layer, input_layer_num, &output_layer);
    RT_ASSERT(HAL_OK == status);

    /* wait for complete */
    err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    rt_free(input_layer);

    return 0;
}

static int32_t ezipa_draw_disp_op_bg_prev(ezipa_obj_t *obj, ezipa_canvas_t *canvas, bool op_bg)
{
    EPIC_HandleTypeDef *epic;
    HAL_StatusTypeDef status;
    rt_err_t err;
    uint8_t canvas_pixel_size;

    EPIC_BlendingDataType fg;
    EPIC_BlendingDataType bg;
    EPIC_BlendingDataType dst;

    HAL_EPIC_BlendDataInit(&fg);
    HAL_EPIC_BlendDataInit(&bg);
    HAL_EPIC_BlendDataInit(&dst);

    epic = drv_get_epic_handle();
    RT_ASSERT(epic);

    /* backup output buffer if dispose_op is OP_PREVIOUS */
    if (EZIP_EZIPA_DISPOSE_OP_PREVIOUS == obj->next_frame.dispose_op)
    {
        RT_ASSERT(!obj->region_area_bak_buf);
        obj->region_area_bak_buf = EZIPA_LARGE_BUF_MALLOC(obj->next_frame.width * obj->next_frame.height * obj->pixel_size);
        RT_ASSERT(obj->region_area_bak_buf);

        if (IS_DCACHED_RAM((uint32_t)obj->region_area_bak_buf))
        {
            mpu_dcache_clean(obj->region_area_bak_buf, obj->next_frame.width * obj->next_frame.height * obj->pixel_size);
        }

        bg.data = (uint8_t *)obj->output_buf
                  + (obj->next_frame.y_offset * obj->header.width + obj->next_frame.x_offset) * obj->pixel_size;
        bg.color_mode = obj->epic_color_fmt;
        bg.height = obj->next_frame.height;
        bg.x_offset = 0;
        bg.y_offset = 0;
        bg.total_width = obj->header.width;
        bg.width = obj->next_frame.width;
        bg.color_en = false;

        dst.data = obj->region_area_bak_buf;
        dst.color_mode = obj->epic_color_fmt;
        dst.height = obj->next_frame.height;
        dst.x_offset = 0;
        dst.y_offset = 0;
        dst.total_width = obj->next_frame.width;
        dst.width = obj->next_frame.width;
        dst.color_en = false;

        epic->XferCpltCallback = epic_cplt_callback;
        epic->user_data = (void *)obj;

        status = HAL_EPIC_Copy_IT(epic, &bg, &dst);
        RT_ASSERT(HAL_OK == status);

        /* wait for complete */
        err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
        RT_ASSERT(RT_EOK == err);
    }

    /* update output buf */
    fg.data = (uint8_t *)obj->ezipa_data;
    fg.color_mode = EPIC_INPUT_EZIP;
    fg.height = obj->next_frame.height;
    fg.x_offset = obj->next_frame.x_offset;
    fg.y_offset = obj->next_frame.y_offset;
    fg.total_width = obj->next_frame.width;
    fg.width = obj->next_frame.width;
    fg.color_en = false;

    bg.data = (uint8_t *)obj->output_buf;
    bg.color_mode = obj->epic_color_fmt;
    bg.height = obj->header.height;
    bg.x_offset = 0;
    bg.y_offset = 0;
    bg.total_width = obj->header.width;
    bg.width = obj->header.width;
    bg.color_en = false;

    dst.data = (uint8_t *)obj->output_buf;
    dst.color_mode = obj->epic_color_fmt;
    dst.height = obj->header.height;
    dst.x_offset = 0;
    dst.y_offset = 0;
    dst.total_width = obj->header.width;
    dst.width = obj->header.width;
    dst.color_en = false;

    epic->XferCpltCallback = epic_cplt_callback;
    epic->user_data = (void *)obj;

    if (EZIP_EZIPA_BLEND_OP_SOURCE == obj->next_frame.blend_op)
    {
        status = HAL_EPIC_Copy_IT(epic, &fg, &dst);
    }
    else
    {
        status = HAL_EPIC_BlendStart_IT(epic, &fg, &bg, &dst, EPIC_LAYER_OPAQUE);
    }
    RT_ASSERT(HAL_OK == status);

    /* wait for complete */
    err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);


    if (canvas)
    {
        /* update canvas buf */
        fg.data = (uint8_t *)obj->output_buf;
        fg.color_mode = obj->epic_color_fmt;
        fg.height = obj->header.height;
        fg.x_offset = canvas->x_offset;
        fg.y_offset = canvas->y_offset;
        fg.total_width = obj->header.width;
        fg.width = obj->header.width;
        fg.color_en = false;

        bg.data = (uint8_t *)canvas->buf;
        bg.color_mode = canvas->color_fmt;
        bg.height = canvas->height;
        bg.x_offset = 0;
        bg.y_offset = 0;
        bg.total_width = canvas->width;
        bg.width = canvas->width;
        bg.color_en = false;

        canvas_pixel_size = ezipa_get_canvas_pixel_size(canvas->color_fmt);
        dst.data = (uint8_t *)canvas->buf + ((canvas_pixel_size * (canvas->mask_y_offset * canvas->width + canvas->mask_x_offset)) >> 3);
        dst.color_mode = canvas->color_fmt;
        dst.height = canvas->mask_height;
        dst.x_offset = canvas->mask_x_offset;
        dst.y_offset = canvas->mask_y_offset;
        dst.total_width = canvas->width;
        dst.width = canvas->mask_width;
        dst.color_en = false;

        epic->XferCpltCallback = epic_cplt_callback;
        epic->user_data = (void *)obj;

        status = HAL_EPIC_BlendStart_IT(epic, &fg, &bg, &dst, EPIC_LAYER_OPAQUE);
        RT_ASSERT(HAL_OK == status);

        /* wait for complete */
        err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
        RT_ASSERT(RT_EOK == err);
    }


    if (op_bg)
    {
        ezipa_init_output_buf(obj);
    }

    return 0;
}

#ifdef RT_USING_DFS
static bool ezipa_is_file(const uint8_t *data)
{
    bool is_file = false;

    RT_ASSERT(data);

    if ((data[0] >= 0x20) && (data[0] <= 0x7F))
    {
        is_file = true;
    }

    return is_file;
}


static void ezipa_init_file_mode(ezipa_obj_t *obj, const char *filename)
{
    ezipa_hdr_t hdr;
    ezipa_anim_ctrl_t anim_ctrl;
    uint32_t i;
    uint32_t frame_hdr_offset;
    uint32_t frame_num;
    uint16_t palette_num;
    int rd_size;
    uint8_t *palette_table = NULL;
    uint32_t palette_tbl_size;
    uint32_t frame_offset_tbl_size;
    struct stat stat_buf;

    obj->fd = open(filename, O_RDONLY);
    if (obj->fd < 0)
    {
        LOG_D("File open failure: %s\n", filename);
        return;
    }
    obj->filename = filename;

    if (RT_EOK != fstat(obj->fd, &stat_buf))
    {
        RT_ASSERT(0);
    }

    obj->file_size = stat_buf.st_size - 4;

    /* skip the first 4 bytes */
    rd_size = lseek(obj->fd, EZ_FILE_LEADING_HDR_SIZE, SEEK_CUR);
    RT_ASSERT(4 == rd_size);

    rd_size = read(obj->fd, &hdr, sizeof(ezipa_hdr_t));
    RT_ASSERT(rd_size == sizeof(ezipa_hdr_t));

    /* read palette */
    palette_num = hdr.palette_num;
    if (3 == (hdr.data0[4] & 0xF)) /* color type 3 for palette format */
    {
        if (0 == palette_num)
        {
            palette_num = 256;
        }
        palette_tbl_size = EZ_PALETTE_TBL_SIZE(palette_num);
        palette_table = rt_malloc(palette_tbl_size);
        rd_size = read(obj->fd, (void *)palette_table, palette_tbl_size);
        RT_ASSERT(rd_size == palette_tbl_size);
    }
    else
    {
        palette_tbl_size = 0;
        palette_num = 0;
    }

    /* read anim ctrl */
    rd_size = read(obj->fd, (void *)&anim_ctrl, sizeof(ezipa_anim_ctrl_t));
    RT_ASSERT(rd_size == sizeof(ezipa_anim_ctrl_t));
    frame_num = EZ_SWAP32(anim_ctrl.frame_num);
    RT_ASSERT(frame_num > 0);
    obj->frame_num = frame_num;

    frame_offset_tbl_size = EZ_FRAME_OFFSET_TBL_SIZE(frame_num);

    /* fake ezipa file header size */
    obj->ezipa_hdr_size = sizeof(ezipa_hdr_t) + sizeof(ezipa_anim_ctrl_t);
    obj->ezipa_hdr_size += palette_tbl_size + frame_offset_tbl_size;

    /* save actual frame offset table */
    obj->org_frame_offset_tbl = rt_malloc(frame_offset_tbl_size);
    RT_ASSERT(obj->org_frame_offset_tbl);
    rd_size = read(obj->fd, (void *)(obj->org_frame_offset_tbl), frame_offset_tbl_size);
    RT_ASSERT(rd_size == frame_offset_tbl_size);

    /* big endian to little endian */
    for (i = 0; i < frame_num; i++)
    {
        obj->org_frame_offset_tbl[i] = EZ_SWAP32(obj->org_frame_offset_tbl[i]);
    }
    /* get the largest frame size */
    obj->max_frame_size = 0;
    for (i = 0; i < frame_num; i++)
    {
        if ((i + 1) < frame_num)
        {
            if (obj->max_frame_size < (obj->org_frame_offset_tbl[i + 1] - obj->org_frame_offset_tbl[i]))
            {
                obj->max_frame_size = (obj->org_frame_offset_tbl[i + 1] - obj->org_frame_offset_tbl[i]);
            }
        }
        else
        {
            if (obj->max_frame_size < (obj->file_size - obj->org_frame_offset_tbl[i]))
            {
                obj->max_frame_size = (obj->file_size - obj->org_frame_offset_tbl[i]);
            }
        }
    }
    /* frame offset must be 4 bytes aligned as EZIP required. original frame offset table alignment is assured by tool */
    obj->max_frame_size = RT_ALIGN(obj->max_frame_size, 4);
    /* include one frame and next frame hdr */
    obj->max_frame_size += sizeof(ezipa_frame_hdr_t);

    obj->ezipa_data = EZIPA_LARGE_BUF_MALLOC(obj->ezipa_hdr_size + obj->max_frame_size);
    RT_ASSERT(obj->ezipa_data);

    /* copy raw ezipa header */
    /* copy file header */
    memcpy((void *)obj->ezipa_data, &hdr, sizeof(hdr));
    if (palette_num > 0)
    {
        /* copy palette */
        memcpy((void *)(obj->ezipa_data + EZ_PALETTE_TBL_POS(palette_num)), palette_table, EZ_PALETTE_TBL_SIZE(palette_num));
    }
    /* copy anim ctrl */
    memcpy((void *)(obj->ezipa_data + EZ_ANIM_CTRL_POS(palette_num)), &anim_ctrl, sizeof(anim_ctrl));
    obj->fake_frame_offset_tbl = (uint32_t *)(obj->ezipa_data + EZ_FRAME_OFFSET_TBL_POS(palette_num));

    /* prepare frame header for the first frame */
    if (frame_num > 1)
    {
        frame_hdr_offset = obj->org_frame_offset_tbl[1];
    }
    else
    {
        frame_hdr_offset = RT_ALIGN(obj->file_size - obj->org_frame_offset_tbl[0], 4);
    }
    RT_ASSERT(frame_hdr_offset < (obj->max_frame_size + obj->ezipa_hdr_size));
    /* read frame header of the first frame */
    rd_size = read(obj->fd, (void *)(obj->ezipa_data + frame_hdr_offset), sizeof(ezipa_frame_hdr_t));
    RT_ASSERT(rd_size == sizeof(ezipa_frame_hdr_t));
    obj->fake_frame_offset_tbl[0] = EZ_SWAP32(frame_hdr_offset);

    if (IS_DCACHED_RAM((uint32_t)obj->ezipa_data))
    {
        mpu_dcache_clean(obj->ezipa_data, obj->ezipa_hdr_size + obj->max_frame_size);
    }

    if (palette_table)
    {
        rt_free(palette_table);
    }
}

static void ezipa_load_file_data(ezipa_obj_t *obj)
{
    uint8_t *frame_hdr;
    uint32_t prev_frame_offset;
    uint32_t curr_frame_offset;
    uint32_t next_frame_offset;
    uint8_t *frame_data;
    int rd_size;
    uint32_t frame_size;
    uint32_t fake_frame_offset;

    RT_ASSERT(obj->org_frame_offset_tbl);
    RT_ASSERT(obj->fake_frame_offset_tbl)
    RT_ASSERT(obj->next_frame.seq_num < obj->header.frame_num);

    curr_frame_offset = obj->org_frame_offset_tbl[obj->next_frame.seq_num];
    fake_frame_offset = obj->org_frame_offset_tbl[0];
    /* must be 4 bytes aligned */
    RT_ASSERT(0 == (fake_frame_offset & 3));
    /* copy prefetched frame header to the beginning  */
    memcpy(obj->ezipa_data + fake_frame_offset,
           obj->ezipa_data + EZ_SWAP32(obj->fake_frame_offset_tbl[obj->next_frame.seq_num]),
           sizeof(ezipa_frame_hdr_t));

    /* update current fake frame start position */
    obj->fake_frame_offset_tbl[obj->next_frame.seq_num] = EZ_SWAP32(obj->org_frame_offset_tbl[0]);

    frame_data = obj->ezipa_data + fake_frame_offset + sizeof(ezipa_frame_hdr_t);
    if ((obj->next_frame.seq_num + 1) == obj->frame_num) /* last frame */
    {
        frame_size = obj->file_size - curr_frame_offset;
        /* update offset for frame 0 */
        if (obj->frame_num > 1)
        {
            obj->fake_frame_offset_tbl[0] = EZ_SWAP32(fake_frame_offset + RT_ALIGN(frame_size, 4));
        }
        /* seek to the read position as same file handle might be shared by multiple ezipa objects */
        lseek(obj->fd, curr_frame_offset + EZ_FILE_LEADING_HDR_SIZE + sizeof(ezipa_frame_hdr_t), SEEK_SET);
        rd_size = read(obj->fd, (void *)frame_data, frame_size - sizeof(ezipa_frame_hdr_t));
        RT_ASSERT(rd_size == frame_size - sizeof(ezipa_frame_hdr_t));
        /* seek to the begin of frame offset table */
        rd_size = lseek(obj->fd, fake_frame_offset + EZ_FILE_LEADING_HDR_SIZE, SEEK_SET);
        RT_ASSERT(rd_size == (fake_frame_offset + EZ_FILE_LEADING_HDR_SIZE));
        rd_size = read(obj->fd, (void *)(obj->ezipa_data + fake_frame_offset + RT_ALIGN(frame_size, 4)),
                       sizeof(ezipa_frame_hdr_t));
        RT_ASSERT(rd_size == sizeof(ezipa_frame_hdr_t));
    }
    else
    {
        next_frame_offset = obj->org_frame_offset_tbl[obj->next_frame.seq_num + 1];
        frame_size = next_frame_offset - curr_frame_offset;
        /* must be 4 bytes aligned */
        RT_ASSERT(0 == (frame_size & 3));
        obj->fake_frame_offset_tbl[obj->next_frame.seq_num + 1] = EZ_SWAP32(fake_frame_offset + frame_size);
        /* seek to the read position as same file handle might be shared by multiple ezipa objects */
        lseek(obj->fd, curr_frame_offset + EZ_FILE_LEADING_HDR_SIZE + sizeof(ezipa_frame_hdr_t), SEEK_SET);
        /* read frame data and next frame header */
        rd_size = read(obj->fd, (void *)frame_data, frame_size);
        RT_ASSERT(rd_size == frame_size);
    }

    if (IS_DCACHED_RAM((uint32_t)obj->ezipa_data))
    {
        mpu_dcache_clean(obj->ezipa_data, obj->ezipa_hdr_size + obj->max_frame_size);
    }
}
#endif /* RT_USING_DFS */


static int32_t ezipa_render_output_buf(ezipa_obj_t *obj)
{
    EPIC_LayerConfigTypeDef output_layer;
    EPIC_HandleTypeDef *epic;
    HAL_StatusTypeDef status;
    rt_err_t err;
    uint8_t canvas_pixel_size;


    EPIC_BlendingDataType fg;
    EPIC_BlendingDataType bg;
    EPIC_BlendingDataType dst;

    HAL_EPIC_BlendDataInit(&fg);
    HAL_EPIC_BlendDataInit(&bg);
    HAL_EPIC_BlendDataInit(&dst);

    /* update output buf */
    fg.data = (uint8_t *)obj->ezipa_data;
    fg.color_mode = EPIC_INPUT_EZIP;
    fg.height = obj->curr_frame.height;
    fg.x_offset = obj->curr_frame.x_offset;
    fg.y_offset = obj->curr_frame.y_offset;
    fg.total_width = obj->curr_frame.width;
    fg.width = obj->curr_frame.width;
    fg.color_en = false;

    bg.data = (uint8_t *)obj->output_buf;
    bg.color_mode = obj->epic_color_fmt;
    bg.height = obj->header.height;
    bg.x_offset = 0;
    bg.y_offset = 0;
    bg.total_width = obj->header.width;
    bg.width = obj->header.width;
    bg.color_en = false;

    dst.data = (uint8_t *)obj->output_buf;
    dst.color_mode = obj->epic_color_fmt;
    dst.height = obj->header.height;
    dst.x_offset = 0;
    dst.y_offset = 0;
    dst.total_width = obj->header.width;
    dst.width = obj->header.width;
    dst.color_en = false;

    epic = drv_get_epic_handle();
    RT_ASSERT(epic);
    epic->XferCpltCallback = epic_cplt_callback;
    epic->user_data = (void *)obj;

    if (EZIP_EZIPA_BLEND_OP_SOURCE == obj->curr_frame.blend_op)
    {
        status = HAL_EPIC_Copy_IT(epic, &fg, &dst);
    }
    else
    {
        status = HAL_EPIC_BlendStart_IT(epic, &fg, &bg, &dst, EPIC_LAYER_OPAQUE);
    }
    RT_ASSERT(HAL_OK == status);

    /* wait for complete */
    err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    return 0;
}

static void ezipa_restore_output_buf(ezipa_obj_t *obj)
{
    EPIC_HandleTypeDef *epic;
    HAL_StatusTypeDef status;
    rt_err_t err;

    EPIC_BlendingDataType bg;
    EPIC_BlendingDataType dst;

    HAL_EPIC_BlendDataInit(&bg);
    HAL_EPIC_BlendDataInit(&dst);

    epic = drv_get_epic_handle();
    RT_ASSERT(epic);

    RT_ASSERT(obj->region_area_bak_buf);
    bg.data = (uint8_t *)obj->region_area_bak_buf;
    bg.color_mode = obj->epic_color_fmt;
    bg.height = obj->curr_frame.height;
    bg.x_offset = obj->curr_frame.x_offset;
    bg.y_offset = obj->curr_frame.y_offset;
    bg.total_width = obj->curr_frame.width;
    bg.width = obj->curr_frame.width;
    bg.color_en = false;

    dst.data = (uint8_t *)obj->output_buf;
    dst.color_mode = obj->epic_color_fmt;
    dst.height = obj->header.height;
    dst.x_offset = 0;
    dst.y_offset = 0;
    dst.total_width = obj->header.width;
    dst.width = obj->header.width;
    dst.color_en = false;

    epic->XferCpltCallback = epic_cplt_callback;
    epic->user_data = (void *)obj;

    status = HAL_EPIC_Copy_IT(epic, &bg, &dst);
    RT_ASSERT(HAL_OK == status);

    /* wait for complete */
    err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    EZIPA_LARGE_BUF_FREE(obj->region_area_bak_buf);
    obj->region_area_bak_buf = NULL;
}


static int32_t ezipa_update_output_buf(ezipa_obj_t *obj, uint8_t dispose_op)
{
    int32_t error = 0;

    if (EZIP_EZIPA_DISPOSE_OP_BACKGROUND == dispose_op)
    {
        error = ezipa_init_output_buf(obj);
    }
    else if (EZIP_EZIPA_DISPOSE_OP_PREVIOUS == dispose_op)
    {
        ezipa_restore_output_buf(obj);
    }
    else
    {
        /* output buf is left as is */
    }

    return error;
}


static int32_t ezipa_render_next_frame(ezipa_obj_t *obj, ezipa_canvas_t *canvas)
{
    HAL_StatusTypeDef status;
    int32_t error;

    /* update output buffer before rendering the next frame */
    if (0 == obj->next_frame.seq_num)
    {
        ezipa_init_output_buf(obj);
    }
    else
    {
        ezipa_update_output_buf(obj, obj->curr_frame.dispose_op);
    }

#ifdef RT_USING_DFS
    if (obj->fd >= 0)
    {
        ezipa_load_file_data(obj);
    }
#endif /* RT_USING_DFS */

    if ((obj->play_idx < obj->header.play_num)
            && ((obj->next_frame.seq_num + 1) == obj->header.frame_num))
    {
        obj->play_idx++;
    }

    status = HAL_EZIP_ResumeEZIPA(obj->ezip_handle, obj->ezipa_data, obj->next_frame.seq_num, obj->play_idx);
    RT_ASSERT(HAL_OK == status);

    error = ezipa_draw_disp_op_bg_prev(obj, canvas, false);

    memcpy(&obj->curr_frame, &obj->next_frame, sizeof(obj->curr_frame));
    obj->valid_curr_frame = true;

    status = HAL_EZIP_SuspendEZIPA(obj->ezip_handle);
    RT_ASSERT(HAL_OK == status);

    if ((0 == obj->header.play_num) || (obj->play_idx != obj->header.play_num))
    {
        if ((obj->next_frame.seq_num + 1) == obj->header.frame_num)
        {
            status = HAL_EZIP_ResumeEZIPA(obj->ezip_handle, obj->ezipa_data, 0, obj->play_idx);
        }
        else
        {
            status = HAL_EZIP_ResumeEZIPA(obj->ezip_handle, obj->ezipa_data, obj->next_frame.seq_num + 1, obj->play_idx);
        }
        RT_ASSERT(HAL_OK == status);

        status = HAL_EZIP_GetNextFrameInfo(obj->ezip_handle, &obj->next_frame);
        RT_ASSERT(HAL_OK == status);

        RT_ASSERT(obj->next_frame.seq_num < obj->header.frame_num);
        RT_ASSERT(obj->next_frame.seq_num != obj->curr_frame.seq_num);

        status = HAL_EZIP_SuspendEZIPA(obj->ezip_handle);
        RT_ASSERT(HAL_OK == status);
    }

    return error;
}

static int32_t ezipa_render_curr_frame(ezipa_obj_t *obj, ezipa_canvas_t *canvas)
{
    EPIC_BlendingDataType fg;
    EPIC_BlendingDataType bg;
    EPIC_BlendingDataType dst;
    EPIC_HandleTypeDef *epic;
    HAL_StatusTypeDef status;
    rt_err_t err;
    uint8_t canvas_pixel_size;

    HAL_EPIC_BlendDataInit(&fg);
    HAL_EPIC_BlendDataInit(&bg);
    HAL_EPIC_BlendDataInit(&dst);

    epic = drv_get_epic_handle();
    RT_ASSERT(epic);

    /* update canvas buf */
    fg.data = (uint8_t *)obj->output_buf;
    fg.color_mode = obj->epic_color_fmt;
    fg.height = obj->header.height;
    fg.x_offset = canvas->x_offset;
    fg.y_offset = canvas->y_offset;
    fg.total_width = obj->header.width;
    fg.width = obj->header.width;
    fg.color_en = false;

    bg.data = (uint8_t *)canvas->buf;
    bg.color_mode = canvas->color_fmt;
    bg.height = canvas->height;
    bg.x_offset = 0;
    bg.y_offset = 0;
    bg.total_width = canvas->width;
    bg.width = canvas->width;
    bg.color_en = false;

    canvas_pixel_size = ezipa_get_canvas_pixel_size(canvas->color_fmt);
    dst.data = (uint8_t *)canvas->buf + ((canvas_pixel_size * (canvas->mask_y_offset * canvas->width + canvas->mask_x_offset)) >> 3);
    dst.color_mode = canvas->color_fmt;
    dst.height = canvas->mask_height;
    dst.x_offset = canvas->mask_x_offset;
    dst.y_offset = canvas->mask_y_offset;
    dst.total_width = canvas->width;
    dst.width = canvas->mask_width;
    dst.color_en = false;

    epic->XferCpltCallback = epic_cplt_callback;
    epic->user_data = (void *)obj;

    status = HAL_EPIC_BlendStart_IT(epic, &fg, &bg, &dst, EPIC_LAYER_OPAQUE);
    RT_ASSERT(HAL_OK == status);

    /* wait for complete */
    err = rt_sem_take(&obj->sem, EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    return 0;
}

ezipa_obj_t *ezipa_open(const void *data, ezipa_color_fmt_t output_color_fmt)
{
    ezipa_obj_t *obj;
    EZIP_HandleTypeDef *ezip_handle;
    HAL_StatusTypeDef status;
    EPIC_HandleTypeDef *epic_handle;
    rt_err_t err;
    IRQn_Type ezip_irqn;

    obj = NULL;
    if (!data)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }
    if ((EZIPA_RGB565 != output_color_fmt)
            && (EZIPA_RGB888 != output_color_fmt))
    {
        status = HAL_ERROR;
        LOG_D("Unsupported color fmt: %d\n", output_color_fmt);
        goto __EXIT;
    }

    obj = rt_malloc(sizeof(*obj));
    RT_ASSERT(obj);
    memset(obj, 0, sizeof(*obj));
    err = rt_sem_init(&obj->sem, "ezipa_dec", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(RT_EOK == err);

#ifdef EZIPA_USE_EZIP2
    obj->ezip_handle = drv_get_ezip2_handle();
    ezip_irqn = EZIP2_IRQn;
#else
    obj->ezip_handle = drv_get_ezip_handle();
    ezip_irqn = EZIP_IRQn;
#endif
    ezip_handle = obj->ezip_handle;

    //TODO: multithread conflict
    if (HAL_EZIP_STATE_RESET == obj->ezip_handle->State)
    {
        status = HAL_EZIP_Init(obj->ezip_handle);
        if (HAL_OK != status)
        {
            LOG_D("Fail to init ezip\n");
            goto __EXIT;
        }
        HAL_NVIC_SetPriority(ezip_irqn, 3, 0);
        HAL_NVIC_EnableIRQ(ezip_irqn);
    }

    epic_handle = drv_get_epic_handle();
    if (HAL_EPIC_STATE_RESET == epic_handle->State)
    {
        HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EPIC_IRQn);

        epic_handle->Instance = hwp_epic;
        epic_handle->hezip = obj->ezip_handle;
        status = HAL_EPIC_Init(epic_handle);
        if (HAL_OK != status)
        {
            LOG_D("Fail to init epic\n");
            goto __EXIT;
        }
    }

#ifdef RT_USING_DFS
    if (ezipa_is_file(data))
    {
        ezipa_init_file_mode(obj, data);
        if (obj->fd < 0)
        {
            status = HAL_ERROR;
            goto __EXIT;
        }
    }
    else
#endif /* RT_USING_DFS */
    {
        obj->fd = -1;
        obj->ezipa_data = (uint8_t *)data;
        if (IS_DCACHED_RAM((uint32_t)obj->ezipa_data))
        {
            SCB_CleanDCache();
        }
    }

    err = drv_epic_take(EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    status = HAL_EZIP_OpenEZIPA(ezip_handle, (uint8_t *)obj->ezipa_data, &obj->header, &obj->next_frame);
    if (HAL_OK != status)
    {
        err = drv_epic_release();
        RT_ASSERT(RT_EOK == err);

        LOG_D("Fail to open ezip\n");
        goto __EXIT;
    }

#ifdef RT_USING_DFS
    if (obj->fd >= 0)
    {
        RT_ASSERT(obj->header.frame_num == obj->frame_num);
    }
#endif /* RT_USING_DFS */

    status = HAL_EZIP_SuspendEZIPA(obj->ezip_handle);
    RT_ASSERT(HAL_OK == status);

    err = drv_epic_release();
    RT_ASSERT(RT_EOK == err);

    if (0 == obj->next_frame.delay_den)
    {
        obj->next_frame.delay_den = 100;
    }
    obj->frame_delay_ms = obj->next_frame.delay_num * 1000 / obj->next_frame.delay_den;

    if ((EZIP_EZIPA_COLOR_RGB565 != obj->header.color_type)
            && (EZIP_EZIPA_COLOR_RGB888 != obj->header.color_type)
            && (EZIP_EZIPA_COLOR_ARGB8565 != obj->header.color_type)
            && (EZIP_EZIPA_COLOR_ARGB8888 != obj->header.color_type)
            && (EZIP_EZIPA_COLOR_INDEXCOLOR != obj->header.color_type))
    {
        status = HAL_ERROR;
        LOG_D("Unsupported colortype %d\n", obj->header.color_type);
        goto __EXIT;
    }

    if (EZIPA_RGB565 == output_color_fmt)
    {
        obj->pixel_size = 2;
    }
    else
    {
        obj->pixel_size = 3;
    }

    if ((EZIP_EZIPA_COLOR_INDEXCOLOR == obj->header.color_type)
            || (EZIP_EZIPA_COLOR_ARGB8565 == obj->header.color_type)
            || (EZIP_EZIPA_COLOR_ARGB8888 == obj->header.color_type))
    {
        obj->alpha_enabled = true;
        obj->pixel_size++;
        if (EZIPA_RGB565 == output_color_fmt)
        {
            obj->epic_color_fmt = EPIC_COLOR_ARGB8565;
        }
        else
        {
            obj->epic_color_fmt = EPIC_COLOR_ARGB8888;
        }
    }
    else
    {
        obj->alpha_enabled = false;
        if (EZIPA_RGB565 == output_color_fmt)
        {
            obj->epic_color_fmt = EPIC_COLOR_RGB565;
        }
        else
        {
            obj->epic_color_fmt = EPIC_COLOR_RGB888;
        }
    }

    obj->output_buf = EZIPA_LARGE_BUF_MALLOC(obj->header.width * obj->header.height * obj->pixel_size);
    obj->region_area_bak_buf = NULL;
    if (!obj->output_buf)
    {
        status = HAL_ERROR;
        LOG_D("Fail to allocate output buf:%dx%dx%d\n", obj->header.width, obj->header.height, obj->pixel_size);
        goto __EXIT;
    }
    obj->output_color_fmt = output_color_fmt;
    obj->play_idx = 0;
    obj->valid_curr_frame = false;

    if (IS_DCACHED_RAM((uint32_t)obj->output_buf))
    {
        mpu_dcache_clean(obj->output_buf, obj->header.width * obj->header.height * obj->pixel_size);
    }

__EXIT:
    if (HAL_OK != status)
    {
        if (obj)
        {
            ezipa_close(obj);
            obj = NULL;
        }
    }

    return obj;
}

int32_t ezipa_close(ezipa_obj_t *obj)
{
    rt_err_t err;

    if (!obj)
    {
        return -1;
    }

    err = rt_sem_detach(&obj->sem);
    RT_ASSERT(RT_EOK == err);

    err = drv_epic_take(EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    HAL_EZIP_CloseEZIPA(obj->ezip_handle);

    err = drv_epic_release();
    RT_ASSERT(RT_EOK == err);

    if (obj->output_buf)
    {
        EZIPA_LARGE_BUF_FREE(obj->output_buf);
    }
    if (obj->region_area_bak_buf)
    {
        EZIPA_LARGE_BUF_FREE(obj->region_area_bak_buf);
    }
#ifdef RT_USING_DFS
    if (obj->fd >= 0)
    {
        close(obj->fd);
        EZIPA_LARGE_BUF_FREE(obj->ezipa_data);
        rt_free(obj->org_frame_offset_tbl);
    }
#endif /* RT_USING_DFS */
    rt_free(obj);

    return 0;
}

int32_t ezipa_draw(ezipa_obj_t *obj, ezipa_canvas_t *canvas, bool next)
{
    int32_t error;
    rt_err_t err;
    EZIP_HandleTypeDef *old_ezip_handle;
    EPIC_HandleTypeDef *epic_handle;
    uint32_t canvas_size = 0;

    error = 0;
    if (!obj)
    {
        error = -1;
        goto __EXIT;
    }
    if (canvas && canvas->color_fmt >= EZIP_CANVAS_PIXEL_SIZE_TBL_LEN)
    {
        error = -2;
        goto __EXIT;
    }

    if (!next && !obj->valid_curr_frame)
    {
        /* no valid current frame data is present */
        error = -3;
        goto __EXIT;
    }

    RT_ASSERT(obj->ezip_handle);

    err = drv_epic_take(EPIC_TIMEOUT_MS);
    RT_ASSERT(RT_EOK == err);

    epic_handle = drv_get_epic_handle();
    old_ezip_handle = epic_handle->hezip;
    epic_handle->hezip = obj->ezip_handle;

    if (canvas && IS_DCACHED_RAM((uint32_t)canvas->buf))
    {
        canvas_size = canvas->width * canvas->height * ezipa_get_canvas_pixel_size(canvas->color_fmt);
        mpu_dcache_clean(canvas->buf, canvas_size);
    }

    if (next)
    {
        error = ezipa_render_next_frame(obj, canvas);
    }
    else if (canvas)
    {
        RT_ASSERT(obj->valid_curr_frame);
        error = ezipa_render_curr_frame(obj, canvas);
    }

    if (IS_DCACHED_RAM((uint32_t)obj->output_buf))
    {
        mpu_dcache_invalidate(obj->output_buf, obj->header.width * obj->header.height * obj->pixel_size);
    }

    if (canvas_size > 0)
    {
        mpu_dcache_invalidate(canvas->buf, canvas_size);
    }

    epic_handle->hezip = old_ezip_handle;

    err = drv_epic_release();
    RT_ASSERT(RT_EOK == err);

__EXIT:

    return error;
}



