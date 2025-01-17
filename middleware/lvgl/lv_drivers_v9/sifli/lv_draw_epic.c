/**
  ******************************************************************************
  * @file   lv_draw_epic.c
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


/*********************
 *      INCLUDES
 *********************/

#include "lv_draw_epic.h"

#if LV_USE_DRAW_EPIC
#include "lv_epic_utils.h"

#include "lv_display_private.h"

/*********************
 *      DEFINES
 *********************/

#define DRAW_UNIT_ID_EPIC 5


/**********************
 *      TYPEDEFS
 **********************/
typedef struct _lv_draw_epic_unit_t
{
    lv_draw_unit_t base_unit;
    lv_draw_task_t *task_act;
#if LV_USE_OS
    lv_thread_sync_t sync;
    lv_thread_t thread;
    volatile bool inited;
    volatile bool exit_status;
#endif

    lv_draw_unit_t *p_sw_unit;
} lv_draw_epic_unit_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*
 * Dispatch a task to the EPIC unit.
 * Return 1 if task was dispatched, 0 otherwise (task not supported).
 */
static int32_t dispatch(lv_draw_unit_t *draw_unit, lv_layer_t *layer);

/*
 * Evaluate a task and set the score and preferred EPIC unit.
 * Return 1 if task is preferred, 0 otherwise (task is not supported).
 */
static int32_t evaluate(lv_draw_unit_t *draw_unit, lv_draw_task_t *task);

#if LV_USE_OS
    static int32_t wait_for_finish(lv_draw_unit_t *draw_unit);
    static void render_thread_cb(void *ptr);
#endif

static void execute_drawing(lv_draw_epic_unit_t *u);

/**********************
 *  STATIC VARIABLES
 **********************/
//#define DEBUG_LV_DRAW_EPIC_ENABLED_FUNCTION
#ifdef DEBUG_LV_DRAW_EPIC_ENABLED_FUNCTION
    volatile uint32_t g_enable_epic = 0xFFFFFFFF;
#endif
static uint8_t initialized = 0;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_draw_epic_init(void)
{
    if (!initialized)
    {
        lv_draw_epic_unit_t *draw_epic_unit = lv_draw_create_unit(sizeof(lv_draw_epic_unit_t));
        draw_epic_unit->base_unit.dispatch_cb = dispatch;
        draw_epic_unit->base_unit.evaluate_cb = evaluate;
        drv_gpu_open();

#if LV_USE_OS
        draw_epic_unit->base_unit.wait_for_finish_cb = wait_for_finish;
        lv_thread_init(&draw_epic_unit->thread, LV_THREAD_PRIO_HIGHEST, render_thread_cb, 4 * 1024, draw_epic_unit);
#endif

        draw_epic_unit->p_sw_unit = draw_epic_unit->base_unit.next;
        initialized = 1;
    }
}

void lv_draw_epic_deinit(void)
{
    if (initialized)
    {
        drv_gpu_close();
        initialized = 0;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static inline bool cf_supported(lv_color_format_t cf, uint32_t flags)
{
    //Ezip images
    if ((LV_COLOR_FORMAT_RAW == cf || LV_COLOR_FORMAT_RAW_ALPHA == cf) && (0 != (flags & LV_IMAGE_FLAGS_EZIP)))
        return true;

    bool is_cf_unsupported = (LV_COLOR_FORMAT_I1 == cf
                              || LV_COLOR_FORMAT_I2 == cf
                              || LV_COLOR_FORMAT_I4 == cf
                              || LV_COLOR_FORMAT_UNKNOWN == cf
                             );

    return (!is_cf_unsupported);
}

static int32_t evaluate(lv_draw_unit_t *draw_unit, lv_draw_task_t *t)
{
    lv_draw_epic_unit_t *draw_epic_unit = (lv_draw_epic_unit_t *) draw_unit;

#ifdef DEBUG_LV_DRAW_EPIC_ENABLED_FUNCTION
    if (0 == (g_enable_epic & (1 << (t->type))))
    {
        return 0;
    }
#endif

    switch (t->type)
    {
    case LV_DRAW_TASK_TYPE_FILL:
    {
        const lv_draw_fill_dsc_t *draw_dsc = (const lv_draw_fill_dsc_t *) t->draw_dsc;

        if (draw_dsc->radius != 0)
            return 0;

        if ((LV_GRADIENT_MAX_STOPS > 2) && (draw_dsc->grad.dir != (lv_grad_dir_t)LV_GRAD_DIR_NONE))
            return 0;

        if (t->preference_score > 70)
        {
            t->preference_score = 70;
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;
        }
        return 1;
    }
#if 0
    case LV_DRAW_TASK_TYPE_LINE:
    case LV_DRAW_TASK_TYPE_ARC:
        if (t->preference_score > 90)
        {
            t->preference_score = 90;
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;
        }
        return 1;

#endif

    case LV_DRAW_TASK_TYPE_LABEL:
        if (t->preference_score > 95)
        {
            t->preference_score = 95;
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;
        }
        return 1;

    case LV_DRAW_TASK_TYPE_BORDER:
    {
        const lv_draw_border_dsc_t *draw_dsc = (lv_draw_border_dsc_t *) t->draw_dsc;

        if (draw_dsc->side != (lv_border_side_t)LV_BORDER_SIDE_FULL)
            return 0;

        if (draw_dsc->radius != 0)
            return 0;

        if (t->preference_score > 90)
        {
            t->preference_score = 90;
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;
        }
        return 1;
    }


    case LV_DRAW_TASK_TYPE_LAYER:
    {
        const lv_draw_image_dsc_t *draw_dsc = (const lv_draw_image_dsc_t *) t->draw_dsc;
        lv_layer_t *layer_to_draw = (lv_layer_t *)draw_dsc->src;

        bool has_recolor = (draw_dsc->recolor_opa != LV_OPA_TRANSP);
        bool has_transform = (draw_dsc->rotation != 0 || draw_dsc->scale_x != LV_SCALE_NONE ||
                              draw_dsc->scale_y != LV_SCALE_NONE);

        if (has_recolor
                || (!cf_supported(layer_to_draw->color_format, 0))
           )
            return 0;
        if ((LV_COLOR_FORMAT_RGB565A8 == layer_to_draw->color_format) && has_transform)
            return 0;

        if (t->preference_score > 80)
        {
            t->preference_score = 80;
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;
        }
        return 1;
    }

    case LV_DRAW_TASK_TYPE_IMAGE:
    {
        lv_draw_image_dsc_t *draw_dsc = (lv_draw_image_dsc_t *) t->draw_dsc;
        const lv_image_dsc_t *img_dsc = draw_dsc->src;

        bool has_recolor = (draw_dsc->recolor_opa != LV_OPA_TRANSP);
        bool has_transform = (draw_dsc->rotation != 0 || draw_dsc->scale_x != LV_SCALE_NONE ||
                              draw_dsc->scale_y != LV_SCALE_NONE);

        if (has_recolor
                || (!cf_supported(img_dsc->header.cf, img_dsc->header.flags))
           )
            return 0;

        if ((LV_COLOR_FORMAT_RGB565A8 == img_dsc->header.cf) && has_transform)
            return 0;

        if (t->preference_score > 80)
        {
            t->preference_score = 80;
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;
        }
        return 1;
    }
    default:
        return 0;
    }

    return 0;
}


static int32_t dispatch(lv_draw_unit_t *draw_unit, lv_layer_t *layer)
{
    lv_draw_epic_unit_t *draw_epic_unit = (lv_draw_epic_unit_t *) draw_unit;

    /* Try to get an ready to draw. */
    lv_draw_task_t *t = lv_draw_get_next_available_task(layer, NULL, DRAW_UNIT_ID_EPIC);

    if (t == NULL)
        return LV_DRAW_UNIT_IDLE;

#if 0
    /*Use sw draw unit if EPIC is busy*/
    if ((draw_epic_unit->task_act) && (DRAW_UNIT_ID_EPIC == t->preferred_draw_unit_id))
    {
        int32_t ret;
        t->preferred_draw_unit_id = LV_DRAW_UNIT_NONE;//Let sw unit pick this layer
        ret = draw_epic_unit->p_sw_unit->dispatch_cb(draw_epic_unit->p_sw_unit, layer);

        if (ret != 1)
        {
            t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC; //Restore preferred id to EPIC.
        }
        return ret;
    }
#else
    /*Return immediately if it's busy with draw task*/
    if (draw_epic_unit->task_act)
    {
        return 0;
    }
#endif /* 0 */

    if (lv_draw_get_unit_count() > 1)
    {
        /* Let the SW unit to draw this task. */
        if (t->preferred_draw_unit_id != DRAW_UNIT_ID_EPIC)
            return LV_DRAW_UNIT_IDLE;
    }
    else
    {
        /* Fake unsupported tasks as ready. */
        if (t->preferred_draw_unit_id != DRAW_UNIT_ID_EPIC)
        {
            t->state = LV_DRAW_TASK_STATE_READY;

            /* Request a new dispatching as it can get a new task. */
            lv_draw_dispatch_request();

            return 1;
        }
    }

    void *buf = lv_draw_layer_alloc_buf(layer);
    if (buf == NULL)
        return LV_DRAW_UNIT_IDLE;

    t->state = LV_DRAW_TASK_STATE_IN_PROGRESS;
    draw_epic_unit->base_unit.target_layer = layer;
    draw_epic_unit->base_unit.clip_area = &t->clip_area;
    draw_epic_unit->task_act = t;

#if LV_USE_OS
    /* Let the render thread work. */
    if (draw_epic_unit->inited)
        lv_thread_sync_signal(&draw_epic_unit->sync);
#else
    execute_drawing(draw_epic_unit);

    draw_epic_unit->task_act->state = LV_DRAW_TASK_STATE_READY;
    draw_epic_unit->task_act = NULL;

    /* The draw unit is free now. Request a new dispatching as it can get a new task. */
    lv_draw_dispatch_request();
#endif

    return 1;
}

static void execute_drawing(lv_draw_epic_unit_t *u)
{
    lv_draw_task_t *t = u->task_act;
    lv_draw_unit_t *draw_unit = (lv_draw_unit_t *) u;

    int ret = (int)drv_epic_wait_done();
    if (ret > 0)
    {
        LV_LOG_ERROR("sf32lb_epic_waitdone error %d", ret);
        LV_ASSERT(0);
    }

    LV_EPIC_LOG("_epic_execute_drawing %d ", t->type);
    lv_epic_print_area_info("src_area", &t->area);
    lv_epic_print_layer_info(draw_unit);

    switch (t->type)
    {

    case LV_DRAW_TASK_TYPE_FILL: /*0*/
        lv_draw_epic_fill(draw_unit, t->draw_dsc, &t->area);
        break;
    case LV_DRAW_TASK_TYPE_BORDER:/*1*/
        lv_draw_epic_border(draw_unit, t->draw_dsc, &t->area);
        break;


    case LV_DRAW_TASK_TYPE_LABEL:/*4*/
        lv_draw_epic_label(draw_unit, t->draw_dsc, &t->area);
        break;

    case LV_DRAW_TASK_TYPE_IMAGE: /*5*/
        lv_draw_epic_img(draw_unit, t->draw_dsc, &t->area);
        break;
    case LV_DRAW_TASK_TYPE_LAYER: /*6*/
        lv_draw_epic_layer(draw_unit, t->draw_dsc, &t->area);
        break;
#if 0


    case LV_DRAW_TASK_TYPE_LINE:/*7*/
        lv_draw_epic_line(draw_unit, t->draw_dsc);
        break;

#endif
    case LV_DRAW_TASK_TYPE_ARC:/*8*/
        lv_draw_epic_arc(draw_unit, t->draw_dsc, &t->area);
        break;
    default:
        break;
    }

    drv_epic_wait_done();

#if LV_USE_PARALLEL_DRAW_DEBUG
    /*Layers manage it for themselves*/
    if (t->type != LV_DRAW_TASK_TYPE_LAYER)
    {
        lv_area_t draw_area;
        if (!lv_area_intersect(&draw_area, &t->area, u->base_unit.clip_area)) return;

        int32_t idx = 0;
        lv_draw_unit_t *draw_unit_tmp = _draw_info.unit_head;
        while (draw_unit_tmp != (lv_draw_unit_t *)u)
        {
            draw_unit_tmp = draw_unit_tmp->next;
            idx++;
        }
        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = lv_palette_main(idx % LV_PALETTE_LAST);
        rect_dsc.border_color = rect_dsc.bg_color;
        rect_dsc.bg_opa = LV_OPA_10;
        rect_dsc.border_opa = LV_OPA_80;
        rect_dsc.border_width = 1;
        lv_draw_epic_fill((lv_draw_unit_t *)u, &rect_dsc, &draw_area);

        lv_point_t txt_size;
        lv_text_get_size(&txt_size, "W", LV_FONT_DEFAULT, 0, 0, 100, LV_TEXT_FLAG_NONE);

        lv_area_t txt_area;
        txt_area.x1 = draw_area.x1;
        txt_area.y1 = draw_area.y1;
        txt_area.x2 = draw_area.x1 + txt_size.x - 1;
        txt_area.y2 = draw_area.y1 + txt_size.y - 1;

        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = lv_color_white();
        lv_draw_epic_fill((lv_draw_unit_t *)u, &rect_dsc, &txt_area);

        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", idx);
        lv_draw_label_dsc_t label_dsc;
        lv_draw_label_dsc_init(&label_dsc);
        label_dsc.color = lv_color_black();
        label_dsc.text = buf;
        lv_draw_epic_label((lv_draw_unit_t *)u, &label_dsc, &txt_area);
    }
#endif
}

#if LV_USE_OS
static int32_t wait_for_finish(lv_draw_unit_t *draw_unit)
{
    lv_draw_epic_unit_t *draw_epic_unit = (lv_draw_epic_unit_t *) draw_unit;


    drv_epic_wait_done();

    return 1;
}

static void render_thread_cb(void *ptr)
{
    lv_draw_epic_unit_t *u = ptr;

    lv_thread_sync_init(&u->sync);
    u->inited = true;

    while (1)
    {
        /* Wait for sync if there is no task set. */
        while (u->task_act == NULL)
        {
            if (u->exit_status)
                break;

            lv_thread_sync_wait(&u->sync);
        }

        if (u->exit_status)
        {
            LV_LOG_INFO("Ready to exit EPIC draw thread.");
            break;
        }

        execute_drawing(u);

        /* Signal the ready state to dispatcher. */
        u->task_act->state = LV_DRAW_TASK_STATE_READY;

        /* Cleanup. */
        u->task_act = NULL;

        /* The draw unit is free now. Request a new dispatching as it can get a new task. */
        lv_draw_dispatch_request();
    }

    u->inited = false;
    lv_thread_sync_delete(&u->sync);
    LV_LOG_INFO("Exit EPIC draw thread.");
}
#endif

#endif /*LV_USE_DRAW_EPIC*/
