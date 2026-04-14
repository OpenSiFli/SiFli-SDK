/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/*********************
 *      INCLUDES
 *********************/

#include "lv_draw_epic.h"

#if LV_USE_DRAW_EPIC
#include <string.h>

#include "lv_epic_utils.h"

#include "lv_display_private.h"

/*********************
 *      DEFINES
 *********************/

#define DRAW_UNIT_ID_EPIC 5

#define MAX_TRACKED_RENDER_LISTS 6
#define DEFAULT_PIXEL_ALIGN 1U

/**********************
 *      TYPEDEFS
 **********************/
typedef struct _lv_draw_epic_unit_t
{
    lv_draw_unit_t base_unit;
    lv_draw_task_t *task_act;
    lv_draw_unit_t *p_sw_unit;
    bool is_sub_layer_task;
} lv_draw_epic_unit_t;

typedef enum
{
    RL_RESOURCE_LAYER_DRAW_BUF, // Sub-layer draw buffer
    RL_RESOURCE_GLYPH_DATA, // Glyph data
} render_list_resource_type_t;

// Resource linked list node - used for deferred release
typedef struct render_list_resource_node
{
    render_list_resource_type_t type;
    void *resource;
    struct render_list_resource_node *next;
} render_list_resource_node_t;

// Per-render-list release tracking state
typedef struct
{
    drv_epic_render_list_t rl;
    uint8_t released;   // Released flag
    render_list_resource_node_t *retained_resources;    // Resource list attached to this RL
} render_list_release_state_t;


typedef struct
{
    drv_epic_render_list_t rl;
    drv_epic_render_buf buf;
} render_list_active_state_t;

// Frame-level state - tracks main frame render list
typedef struct
{
    render_list_active_state_t active;  // Current active render list + buffer
    lv_display_t *disp;     // Associated display device
    uint32_t pixel_align;   // Pixel alignment requirement
    uint32_t create_seq;    // Creation sequence number (debug)
    uint32_t submit_seq;    // Submit sequence number (debug)
    drv_epic_render_list_t last_created;
    uint8_t pending_submit;     // Pending submit flag (leak detection)
} render_list_frame_state_t;

// Sub-layer level state
typedef struct
{
    render_list_active_state_t active;  // Current active sub-layer RL
    lv_layer_t *owner;  // Layer that owns this RL
} render_list_layer_state_t;

typedef struct
{
    lv_display_t *disp;
    uint32_t pixel_align;
} render_list_display_cache_t;

// Global context - manages render list state and resources
typedef struct
{
    render_list_frame_state_t frame;    // Main frame state
    render_list_layer_state_t layer;    // Sub-layer state
    render_list_release_state_t tracked[MAX_TRACKED_RENDER_LISTS];  // Track up to 6 RLs
    render_list_resource_node_t *deferred_resources;    // Global deferred release list
    render_list_display_cache_t display_cache;      // Display device cache
} render_list_context_t;

/**********************
 *  STATIC VARIABLES
 **********************/
static uint8_t initialized = 0;
static render_list_context_t render_list_ctx;

/**********************
 *      MACROS
 **********************/

// Main frame render done callback (called by EPIC hardware), sends result to LCD and releases RL
void main_frame_render_done_callback(drv_epic_render_list_t rl, EPIC_LayerConfigTypeDef *p_dst,
                                     void *usr_data, uint32_t last);

/**********************
 *   STATIC FUNCTIONS
 **********************/

// Initialize render list manager global context
static void render_list_manager_init(void)
{
    memset(&render_list_ctx, 0, sizeof(render_list_ctx));
    render_list_ctx.frame.pixel_align = DEFAULT_PIXEL_ALIGN;
}

// Set display device pixel alignment and cache it
void render_list_set_display_pixel_align(lv_display_t *disp, uint32_t pixel_align)
{
    if (disp == NULL)
    {
        return;
    }

    render_list_ctx.display_cache.disp = disp;
    render_list_ctx.display_cache.pixel_align = (pixel_align != 0U) ? pixel_align : DEFAULT_PIXEL_ALIGN;
}

// Reset main frame active state (clear RL, buffer, display reference)
static void render_list_reset_frame_active(void)
{
    render_list_ctx.frame.active.rl = NULL;
    memset(&render_list_ctx.frame.active.buf, 0, sizeof(render_list_ctx.frame.active.buf));
    render_list_ctx.frame.disp = NULL;
    render_list_ctx.frame.pixel_align = DEFAULT_PIXEL_ALIGN;
}

// Reset sub-layer active state (clear RL, buffer, owner reference)
static void render_list_reset_layer_active(void)
{
    render_list_ctx.layer.active.rl = NULL;
    memset(&render_list_ctx.layer.active.buf, 0, sizeof(render_list_ctx.layer.active.buf));
    render_list_ctx.layer.owner = NULL;
}

// Find RL in tracking array
static render_list_release_state_t *render_list_find_state_locked(drv_epic_render_list_t rl)
{
    for (int i = 0; i < MAX_TRACKED_RENDER_LISTS; i++)
    {
        if (render_list_ctx.tracked[i].rl == rl)
        {
            return &render_list_ctx.tracked[i];
        }
    }

    return NULL;
}

// Get or allocate a tracking slot
static render_list_release_state_t *render_list_acquire_state_locked(drv_epic_render_list_t rl)
{
    render_list_release_state_t *state = render_list_find_state_locked(rl);
    render_list_release_state_t *free_state = NULL;
    render_list_release_state_t *released_state = NULL;

    if (state != NULL)
    {
        return state;
    }

    for (int i = 0; i < MAX_TRACKED_RENDER_LISTS; i++)
    {
        if (free_state == NULL && render_list_ctx.tracked[i].rl == NULL)
        {
            free_state = &render_list_ctx.tracked[i];
        }

        if (released_state == NULL && render_list_ctx.tracked[i].released != 0U)
        {
            released_state = &render_list_ctx.tracked[i];
        }
    }

    state = (free_state != NULL) ? free_state : released_state;
    RT_ASSERT(state != NULL);
    RT_ASSERT(state->retained_resources == NULL);

    memset(state, 0, sizeof(*state));
    state->rl = rl;
    return state;
}

// Allocate a resource list node
static render_list_resource_node_t *render_list_alloc_resource_node(render_list_resource_type_t type, void *resource)
{
    render_list_resource_node_t *node = (render_list_resource_node_t *)rt_malloc(sizeof(*node));
    if (node == NULL)
    {
        return NULL;
    }

    node->type = type;
    node->resource = resource;
    node->next = NULL;
    return node;
}

// Append resource list to the head of global deferred release list (must be called in critical section)
static void render_list_append_resources_locked(render_list_resource_node_t *resource_list)
{
    if (resource_list == NULL)
    {
        return;
    }

    render_list_resource_node_t *tail = resource_list;
    while (tail->next != NULL)
    {
        tail = tail->next;
    }

    tail->next = render_list_ctx.deferred_resources;
    render_list_ctx.deferred_resources = resource_list;
}

//  Defer resource release
static void render_list_defer_resources_locked(drv_epic_render_list_t rl)
{
    render_list_release_state_t *state = render_list_find_state_locked(rl);
    if (state == NULL || state->retained_resources == NULL)
    {
        return;
    }

    render_list_resource_node_t *resource_list = state->retained_resources;
    state->retained_resources = NULL;   // Detach from RL
    render_list_append_resources_locked(resource_list);     // Append to global deferred list
}

// Get display device pixel alignment from LCD driver (with cache)
static uint32_t render_list_get_display_pixel_align(lv_display_t *disp)
{
    if (disp == NULL)
    {
        return DEFAULT_PIXEL_ALIGN;
    }

    if (render_list_ctx.display_cache.disp == disp && render_list_ctx.display_cache.pixel_align != 0U)
    {
        return render_list_ctx.display_cache.pixel_align;
    }

    uint32_t pixel_align = DEFAULT_PIXEL_ALIGN;
    rt_device_t lcd_device = (rt_device_t)lv_display_get_driver_data(disp);
    if (lcd_device != NULL)
    {
        struct rt_device_graphic_info lcd_info;
        if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &lcd_info) == RT_EOK)
        {
            pixel_align = (lcd_info.draw_align != 0U) ? lcd_info.draw_align : DEFAULT_PIXEL_ALIGN;
        }
        else
        {
            LV_LOG_WARN("EPIC: Failed to get LCD info, using default pixel_align=%u\n", DEFAULT_PIXEL_ALIGN);
        }
    }
    else
    {
        LV_LOG_WARN("EPIC: LCD device not found in display, using default pixel_align=%u\n", DEFAULT_PIXEL_ALIGN);
    }

    render_list_set_display_pixel_align(disp, pixel_align);
    return render_list_ctx.display_cache.pixel_align;
}

// Get current active render list handle (sub-layer takes priority over main frame)
static drv_epic_render_list_t render_list_get_current_handle(void)
{
    if (render_list_ctx.layer.active.rl != NULL)
    {
        return render_list_ctx.layer.active.rl;
    }

    if (render_list_ctx.frame.active.rl != NULL)
    {
        return render_list_ctx.frame.active.rl;
    }

    return NULL;
}

// Get buffer descriptor for current active render list (sub-layer takes priority over main frame)
drv_epic_render_buf *render_list_get_current_buf(void)
{
    if (render_list_ctx.layer.active.rl != NULL)
    {
        return &render_list_ctx.layer.active.buf;
    }

    if (render_list_ctx.frame.active.rl != NULL)
    {
        return &render_list_ctx.frame.active.buf;
    }

    LV_LOG_ERROR("EPIC: No active render list available\n");
    LV_ASSERT(0);
    return NULL;
}

// Attach resource to current active RL, deferred release when RL is released
static bool render_list_retain_resource(void *resource, render_list_resource_type_t type, const char *resource_name)
{
    if (resource == NULL)
    {
        return false;
    }

    drv_epic_render_list_t rl = render_list_get_current_handle();
    if (rl == NULL)
    {
        LV_LOG_ERROR("EPIC: No active render list when retaining %s", resource_name);
        LV_ASSERT(0);
        return false;
    }

    render_list_resource_node_t *node = render_list_alloc_resource_node(type, resource);
    if (node == NULL)
    {
        LV_LOG_ERROR("EPIC: Failed to allocate retain node for %s", resource_name);
        return false;
    }

    rt_enter_critical();
    render_list_release_state_t *state = render_list_acquire_state_locked(rl);
    RT_ASSERT(state != NULL);
    node->next = state->retained_resources;     // Head insertion
    state->retained_resources = node;
    rt_exit_critical();

    return true;
}

// Retain sub-layer draw buffer, associate with current active RL
bool lv_draw_epic_retain_layer_draw_buf(lv_draw_buf_t *draw_buf)
{
    return render_list_retain_resource(draw_buf, RL_RESOURCE_LAYER_DRAW_BUF, "layer draw buffer");
}

// Retain glyph data, associate with current active RL
bool lv_draw_epic_retain_glyph_data(void *glyph_data)
{
    return render_list_retain_resource(glyph_data, RL_RESOURCE_GLYPH_DATA, "glyph data");
}

// Release deferred resources
void lv_draw_epic_release_deferred_resources(void)
{
    render_list_resource_node_t *resource_list = NULL;

    rt_enter_critical();
    resource_list = render_list_ctx.deferred_resources;
    render_list_ctx.deferred_resources = NULL;
    rt_exit_critical();

    while (resource_list != NULL)
    {
        render_list_resource_node_t *next = resource_list->next;
        if (resource_list->resource != NULL)
        {
            if (resource_list->type == RL_RESOURCE_LAYER_DRAW_BUF)
            {
                lv_draw_buf_destroy((lv_draw_buf_t *)resource_list->resource);
            }
            else
            {
                lv_free(resource_list->resource);
            }
        }
        rt_free(resource_list);
        resource_list = next;
    }
}

// Set RL released flag (idempotent, returns false if no change)
static bool render_list_set_released_flag(drv_epic_render_list_t rl, uint8_t released)
{
    if (rl == NULL)
    {
        return false;
    }

    bool changed = false;

    rt_enter_critical();
    render_list_release_state_t *state = render_list_acquire_state_locked(rl);
    RT_ASSERT(state != NULL);
    if (state->released != released)
    {
        state->released = released;
        changed = true;
    }
    rt_exit_critical();

    return changed;
}

// Detach RL from active state (does not release EPIC hardware resources, only clears software references)
static void render_list_detach_by_handle(drv_epic_render_list_t rl)
{
    if (rl == NULL) return;

    if (render_list_ctx.frame.active.rl == rl)
    {
        render_list_reset_frame_active();
    }

    if (render_list_ctx.layer.active.rl == rl)
    {
        render_list_reset_layer_active();
    }
}

// Release render list: mark released, detach from active state, defer release of attached resources
int render_list_release_by_handle(drv_epic_render_list_t rl)
{
    if (rl == NULL)
    {
        LV_LOG_WARN("EPIC: Render list handle is NULL\n");
        return 0;
    }

    if (!render_list_set_released_flag(rl, 1U))
    {
        return 0;
    }

    render_list_detach_by_handle(rl);

    rt_enter_critical();
    render_list_defer_resources_locked(rl);
    rt_exit_critical();

    if ((rl == render_list_ctx.frame.last_created) && render_list_ctx.frame.pending_submit)
    {
        LV_LOG_ERROR("EPIC: RL_TRACK release before submit! handle=%p create_seq=%u submit_seq=%u\n",
                     rl, render_list_ctx.frame.create_seq, render_list_ctx.frame.submit_seq);
        LV_ASSERT(0);
        render_list_ctx.frame.pending_submit = 0;
    }

    return 0;
}

// Sub-layer render done callback (called by EPIC hardware), releases RL when last slice completes
static void sub_layer_render_done_callback(drv_epic_render_list_t rl, EPIC_LayerConfigTypeDef *p_dst,
                                           void *usr_data, uint32_t last)
{
    LV_UNUSED(p_dst);
    LV_UNUSED(usr_data);

    if (!last)
    {
        return;
    }

    RT_ASSERT(rl != NULL);
    render_list_release_by_handle(rl);
}

// Internal submit function: build EPIC message and submit, supports render-to-buffer or direct-to-LCD
static int render_list_submit_internal(drv_epic_render_list_t rl, drv_epic_render_buf *render_buf, uint32_t render_type)
{
    if (rl == NULL || render_buf == NULL)
    {
        LV_LOG_ERROR("EPIC: Invalid render list or buffer\n");
        LV_ASSERT(0);
        return -1;
    }

    EPIC_MsgTypeDef msg;
    msg.render_list = rl;

    if (render_type == EPIC_MSG_RENDER_TO_BUF)
    {
        msg.id = EPIC_MSG_RENDER_TO_BUF;
        msg.content.r2b.dst_area = render_buf->area;
        msg.content.r2b.usr_data = render_buf->data;
        msg.content.r2b.done_cb = sub_layer_render_done_callback;
    }
    else
    {
        msg.id = EPIC_MSG_RENDER_DRAW;

        lv_display_t *disp = render_list_ctx.frame.disp;
        if (disp == NULL)
        {
            LV_LOG_ERROR("EPIC: No active display for main frame render list\n");
            LV_ASSERT(0);
            return -1;
        }
        lv_draw_buf_t *draw_buf = disp->buf_act;

        if (!draw_buf || !draw_buf->data)
        {
            LV_LOG_ERROR("EPIC: No active draw buffer for main frame\n");
            LV_ASSERT(0);
            return -1;
        }

        msg.content.rd.area.x0 = disp->offset_x;
        msg.content.rd.area.y0 = disp->offset_y;
        msg.content.rd.area.x1 = (int16_t)(disp->offset_x + disp->hor_res - 1);
        msg.content.rd.area.y1 = (int16_t)(disp->offset_y + disp->ver_res - 1);

        msg.content.rd.usr_data = disp;
        msg.content.rd.partial_done_cb = main_frame_render_done_callback;
        msg.content.rd.pixel_align = (render_list_ctx.frame.pixel_align != 0U) ?
                                     render_list_ctx.frame.pixel_align :
                                     DEFAULT_PIXEL_ALIGN;
    }

    rt_err_t err = drv_epic_render_msg_commit(&msg);
    if (err != RT_EOK)
    {
        LV_LOG_ERROR("EPIC: Failed to commit render list, error code: %d\n", err);
        return -1;
    }

    return 0;
}

// Check if layer has unfinished draw tasks (skip specified task)
static bool layer_has_pending_tasks(const lv_layer_t *layer, const lv_draw_task_t *skip_task)
{
    if (layer == NULL) return false;

    const lv_draw_task_t *t = layer->draw_task_head;
    while (t)
    {
        if (t != skip_task &&
            (t->state == LV_DRAW_TASK_STATE_QUEUED ||
             t->state == LV_DRAW_TASK_STATE_WAITING ||
             t->state == LV_DRAW_TASK_STATE_IN_PROGRESS))
        {
            return true;
        }
        t = t->next;
    }

    return false;
}

// Flush sub-layer RL: submit sub-layer render list to EPIC when conditions are met
static void render_list_flush_layer(lv_draw_epic_unit_t *u, bool force)
{
    drv_epic_render_list_t rl = render_list_ctx.layer.active.rl;

    if (rl == NULL)
    {
        return;
    }

    if (!force)
    {
        if (u == NULL || !u->is_sub_layer_task)
        {
            return;
        }

        lv_draw_task_t *task = u->task_act;
        lv_layer_t *layer = task ? task->target_layer : NULL;
        if (layer == NULL || layer->parent == NULL)
        {
            return;
        }

        if (render_list_ctx.layer.owner != layer)
        {
            return;
        }

        if (!layer->all_tasks_added)
        {
            return;
        }

        if (layer_has_pending_tasks(layer, task))
        {
            return;
        }
    }

    int ret = render_list_submit_internal(rl, &render_list_ctx.layer.active.buf, EPIC_MSG_RENDER_TO_BUF);
    if (ret != 0)
    {
        LV_LOG_ERROR("EPIC: Failed to submit sub-layer render list, ret=%d\n", ret);
        render_list_release_by_handle(rl);
        return;
    }

    render_list_reset_layer_active();
}

// Finish current draw task: flush sub-layer, mark finished, request next dispatch
static void finish_current_task(lv_draw_epic_unit_t *u)
{
    if (u == NULL || u->task_act == NULL)
    {
        return;
    }

    u->task_act->state = LV_DRAW_TASK_STATE_FINISHED;
    u->is_sub_layer_task = false;
    u->task_act = NULL;

    lv_draw_dispatch_request();
}

// Create main frame render list: release deferred resources, handle old RL, allocate and configure new RL
int render_list_create_main_frame(lv_display_t *disp)
{
    lv_draw_epic_release_deferred_resources();

    if (disp->inv_p == 0) // inv_p is LVGL display invalid area counter. If 0, no area needs refresh
    {
        render_list_reset_frame_active();
        return 0;
    }

    if (render_list_ctx.frame.active.rl != NULL)
    {
        drv_epic_render_list_t old_main_rl = render_list_ctx.frame.active.rl;

        if (render_list_ctx.frame.pending_submit && (old_main_rl == render_list_ctx.frame.last_created))
        {
            LV_LOG_ERROR("EPIC: RL_TRACK previous main list was never submitted! handle=%p create_seq=%u submit_seq=%u\n",
                         old_main_rl,
                         render_list_ctx.frame.create_seq,
                         render_list_ctx.frame.submit_seq);

            // List was never submitted, critical error
            LV_ASSERT(0);
        }
        else
        {
            render_list_detach_by_handle(old_main_rl);
        }
    }

    lv_color_format_t display_cf = disp->color_format;
    if (display_cf == LV_COLOR_FORMAT_UNKNOWN && disp->buf_act)
    {
        display_cf = disp->buf_act->header.cf;
    }

    if (display_cf == LV_COLOR_FORMAT_UNKNOWN)
    {
#if LV_COLOR_DEPTH == 16
        display_cf = LV_COLOR_FORMAT_RGB565;
#elif LV_COLOR_DEPTH == 24
        display_cf = LV_COLOR_FORMAT_RGB888;
#elif LV_COLOR_DEPTH == 32
        display_cf = LV_COLOR_FORMAT_ARGB8888;
#else
        LV_LOG_ERROR("EPIC: Unable to determine color format!\n");
        return -1;
#endif
    }

    render_list_ctx.frame.active.buf.cf = lv_img_2_epic_cf(display_cf);
    render_list_ctx.frame.active.buf.data = (uint8_t *)0xCCCCCCCC;
    render_list_ctx.frame.active.buf.area.x0 = 0;
    render_list_ctx.frame.active.buf.area.y0 = 0;
    render_list_ctx.frame.active.buf.area.x1 = (int16_t)(disp->hor_res - 1);
    render_list_ctx.frame.active.buf.area.y1 = (int16_t)(disp->ver_res - 1);

    EPIC_AreaTypeDef ow_area = {0, 0, (int16_t)(disp->hor_res - 1), (int16_t)(disp->ver_res - 1)};

    render_list_ctx.frame.active.rl = drv_epic_alloc_render_list(&render_list_ctx.frame.active.buf, &ow_area);
    if (render_list_ctx.frame.active.rl == NULL)
    {
        render_list_reset_frame_active();
        LV_LOG_ERROR("EPIC: Failed to allocate main frame render list (timeout or no resource)\n");
        return -1;
    }

    render_list_ctx.frame.disp = disp;
    render_list_ctx.frame.pixel_align = render_list_get_display_pixel_align(disp);
    render_list_set_released_flag(render_list_ctx.frame.active.rl, 0U);

    render_list_ctx.frame.create_seq++;
    render_list_ctx.frame.last_created = render_list_ctx.frame.active.rl;
    render_list_ctx.frame.pending_submit = 1;

    return 0;
}

// Submit main frame render list to EPIC hardware (direct draw to LCD)
int render_list_submit_main_frame(void)
{
    if (render_list_ctx.frame.active.rl == NULL)
    {
        if (render_list_ctx.frame.pending_submit)
        {
            LV_LOG_ERROR("EPIC: RL_TRACK submit skipped but pending main list exists! last_created=%p create_seq=%u submit_seq=%u\n",
                         render_list_ctx.frame.last_created,
                         render_list_ctx.frame.create_seq,
                         render_list_ctx.frame.submit_seq);
            LV_ASSERT(0);
        }
        LV_LOG_WARN("EPIC: Main frame render list is not created, skip submit\n");
        return 0;
    }

    drv_epic_render_list_t submit_rl = render_list_ctx.frame.active.rl;
    int ret = render_list_submit_internal(submit_rl, &render_list_ctx.frame.active.buf, EPIC_MSG_RENDER_DRAW);

    if (ret == 0)
    {
        render_list_ctx.frame.submit_seq++;
        if (submit_rl == render_list_ctx.frame.last_created)
        {
            render_list_ctx.frame.pending_submit = 0;
        }
        return 0;
    }

    LV_LOG_ERROR("EPIC: RL_TRACK submit failed handle=%p ret=%d create_seq=%u submit_seq=%u pending=%u\n",
                 submit_rl, ret,
                 render_list_ctx.frame.create_seq,
                 render_list_ctx.frame.submit_seq,
                 render_list_ctx.frame.pending_submit);
    render_list_release_by_handle(submit_rl);
    return ret;
}

// Get current main frame render list handle
drv_epic_render_list_t render_list_get_current_frame_handle(void)
{
    return render_list_ctx.frame.active.rl;
}

// LVGL Draw Unit evaluate callback: set EPIC preference scores for each task type
static int32_t evaluate(lv_draw_unit_t *draw_unit, lv_draw_task_t *t)
{
    LV_UNUSED(draw_unit);

    t->preferred_draw_unit_id = DRAW_UNIT_ID_EPIC;

    switch (t->type)
    {
    case LV_DRAW_TASK_TYPE_LAYER:
        if (t->preference_score > 80)
            t->preference_score = 80;
        break;

    case LV_DRAW_TASK_TYPE_FILL:
        if (t->preference_score > 70)
            t->preference_score = 70;
        break;

    case LV_DRAW_TASK_TYPE_BORDER:
        if (t->preference_score > 90)
            t->preference_score = 90;
        break;

    case LV_DRAW_TASK_TYPE_IMAGE:
        if (t->preference_score > 80)
            t->preference_score = 80;
        break;

    case LV_DRAW_TASK_TYPE_LINE:
        if (t->preference_score > 90)
            t->preference_score = 90;
        break;

#ifdef EPIC_SUPPORT_A8
    case LV_DRAW_TASK_TYPE_LABEL:
        if (t->preference_score > 95)
            t->preference_score = 95;
        break;
#endif

    case LV_DRAW_TASK_TYPE_ARC:
        if (t->preference_score > 90)
            t->preference_score = 90;
        break;

    default:
        if (t->preference_score > 99)
            t->preference_score = 99;
        break;
    }
    return 1;
}

// Execute draw task: dispatch to corresponding EPIC draw function based on task type
static void execute_drawing(lv_draw_epic_unit_t *u)
{
    lv_draw_task_t *t = u->task_act;

    lv_epic_print_area_info("src_area", &t->area);
    lv_epic_print_layer_info(t);
    if (t->type < 0)
     {
        LV_LOG_WARN("EPIC: Invalid task type %d, treating as regular operation execute \n", t->type);
    }

    switch (t->type)
    {
    case LV_DRAW_TASK_TYPE_FILL:
        lv_draw_epic_fill(t, t->draw_dsc, &t->area);
        break;
    case LV_DRAW_TASK_TYPE_BORDER:
        lv_draw_epic_border(t, t->draw_dsc, &t->area);
        break;
    case LV_DRAW_TASK_TYPE_LABEL:
        lv_draw_epic_label(t, t->draw_dsc, &t->area);
        break;
    case LV_DRAW_TASK_TYPE_LAYER:
    {
        lv_draw_epic_layer(t, t->draw_dsc, &t->area);
        break;
    }
    case LV_DRAW_TASK_TYPE_IMAGE:
        lv_draw_epic_img(t, t->draw_dsc, &t->area);
        break;
    case LV_DRAW_TASK_TYPE_LINE:
        lv_draw_epic_line(t, t->draw_dsc);
        break;
    case LV_DRAW_TASK_TYPE_ARC:
        lv_draw_epic_arc(t, t->draw_dsc, &t->area);
        break;
    default:
        LV_LOG_WARN("EPIC: Unsupported draw task type %d\n", t->type);
        break;
    }

#if LV_USE_PARALLEL_DRAW_DEBUG
    if (t->type != LV_DRAW_TASK_TYPE_LAYER)
    {
        lv_area_t draw_area;
        if (!lv_area_intersect(&draw_area, &t->area, &t->clip_area)) return;

        int32_t idx = t->draw_unit->idx;
        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = lv_palette_main(idx % LV_PALETTE_LAST);
        rect_dsc.border_color = rect_dsc.bg_color;
        rect_dsc.bg_opa = LV_OPA_10;
        rect_dsc.border_opa = LV_OPA_80;
        rect_dsc.border_width = 1;
        lv_draw_epic_fill(t, &rect_dsc, &draw_area);

        lv_point_t txt_size;
        lv_text_get_size(&txt_size, "W", LV_FONT_DEFAULT, 0, 0, 100, LV_TEXT_FLAG_NONE);

        lv_area_t txt_area;
        txt_area.x1 = draw_area.x1;
        txt_area.y1 = draw_area.y1;
        txt_area.x2 = draw_area.x1 + txt_size.x - 1;
        txt_area.y2 = draw_area.y1 + txt_size.y - 1;

        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = lv_color_white();
        lv_draw_epic_fill(t, &rect_dsc, &txt_area);

        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", idx);
        lv_draw_label_dsc_t label_dsc;
        lv_draw_label_dsc_init(&label_dsc);
        label_dsc.color = lv_color_black();
        label_dsc.text = buf;
        lv_draw_epic_label(t, &label_dsc, &txt_area);
    }
#endif
}


// LVGL Draw Unit dispatch callback: get task, manage sub-layer RL, execute drawing
static int32_t dispatch(lv_draw_unit_t *draw_unit, lv_layer_t *layer)
{
    lv_draw_epic_unit_t *draw_epic_unit = (lv_draw_epic_unit_t *) draw_unit;

    lv_draw_epic_release_deferred_resources();

    lv_draw_task_t *t = lv_draw_get_next_available_task(layer, NULL, DRAW_UNIT_ID_EPIC);

    if (t == NULL)
        return LV_DRAW_UNIT_IDLE;

    if (draw_epic_unit->task_act)
    {
        return 0;
    }

    if (lv_draw_get_unit_count() > 1)
    {
        if (t->preferred_draw_unit_id != DRAW_UNIT_ID_EPIC)
            return LV_DRAW_UNIT_IDLE;
    }
    else
    {
        if (t->preferred_draw_unit_id != DRAW_UNIT_ID_EPIC)
        {
            t->state = LV_DRAW_TASK_STATE_FINISHED;

            lv_draw_dispatch_request();

            return 1;
        }
    }

    bool is_sub_layer = (layer && layer->parent != NULL);

    if (is_sub_layer)
    {

        if (render_list_ctx.layer.active.rl == NULL)
        {
            void *buf = lv_draw_layer_alloc_buf(layer);
            if (buf == NULL)
            {
                LV_LOG_ERROR("EPIC: Failed to allocate layer buffer, returning IDLE\n");
                return LV_DRAW_UNIT_IDLE;
            }

            render_list_ctx.layer.active.buf.cf = lv_img_2_epic_cf(layer->color_format);
            render_list_ctx.layer.active.buf.data = buf;
            render_list_ctx.layer.active.buf.area.x0 = 0;
            render_list_ctx.layer.active.buf.area.y0 = 0;
            render_list_ctx.layer.active.buf.area.x1 = lv_area_get_width(&layer->buf_area) - 1;
            render_list_ctx.layer.active.buf.area.y1 = lv_area_get_height(&layer->buf_area) - 1;

            EPIC_AreaTypeDef ow_area = {0, 0,
                                        render_list_ctx.layer.active.buf.area.x1,
                                        render_list_ctx.layer.active.buf.area.y1};

            render_list_ctx.layer.active.rl = drv_epic_alloc_render_list(&render_list_ctx.layer.active.buf, &ow_area);
            if (render_list_ctx.layer.active.rl == NULL)
            {
                LV_LOG_ERROR("EPIC: Failed to allocate layer render list, returning IDLE\n");
                render_list_reset_layer_active();
                return LV_DRAW_UNIT_IDLE;
            }
            render_list_ctx.layer.owner = layer;
            render_list_set_released_flag(render_list_ctx.layer.active.rl, 0U);
        }
    }
    else
    {
        if (render_list_ctx.layer.active.rl != NULL)
        {
            render_list_flush_layer(NULL, true);
        }

        void *buf = lv_draw_layer_alloc_buf(layer);
        if (buf == NULL)
        {
            LV_LOG_ERROR("EPIC: Failed to allocate layer buffer, returning IDLE\n");
            return LV_DRAW_UNIT_IDLE;
        }
    }

    t->state = LV_DRAW_TASK_STATE_IN_PROGRESS;
    draw_epic_unit->task_act = t;
    draw_epic_unit->task_act->draw_unit = draw_unit;
    draw_epic_unit->is_sub_layer_task = is_sub_layer;

    execute_drawing(draw_epic_unit);
    finish_current_task(draw_epic_unit);

    return 1;
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
// EPIC draw unit init: register Draw Unit, open GPU, init manager
void lv_draw_epic_init(void)
{
    if (!initialized)
    {
        lv_draw_epic_unit_t *draw_epic_unit = lv_draw_create_unit(sizeof(lv_draw_epic_unit_t));
        draw_epic_unit->base_unit.dispatch_cb = dispatch;
        draw_epic_unit->base_unit.evaluate_cb = evaluate;

        drv_gpu_open();
        render_list_manager_init();

        draw_epic_unit->p_sw_unit = draw_epic_unit->base_unit.next;
        initialized = 1;
    }
}

// EPIC draw unit deinit: close GPU
void lv_draw_epic_deinit(void)
{
    if (initialized)
    {
        drv_gpu_close();
        initialized = 0;
    }
}

#endif /*LV_USE_DRAW_EPIC*/
