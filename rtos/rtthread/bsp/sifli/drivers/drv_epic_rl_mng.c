/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "drv_epic.h"
#if defined(DRV_EPIC_NEW_API) && defined(BSP_USING_EPIC)
#include <rthw.h>
#include "string.h"
#include "mem_section.h"
#ifdef HAL_EZIP_MODULE_ENABLED
    #include "drv_flash.h"
#endif

#include "drv_epic_private.h"
#ifdef DRV_EPIC_NEW_API_DUAL_CORE_HCPU
    #include "acpu_ctrl.h"
#endif /*DRV_EPIC_NEW_API_DUAL_CORE_HCPU*/

#ifdef SOC_SF32LB55X
    #error "Render-list mode can not works on SF32LB55X"
#endif


extern EPIC_DrvTypeDef drv_epic;
#define LETTER_POOL_FROM_STACK() (rl->letter_pool == &drv_epic_letter_pool[letter_pool_max * rl->letter_pool_index])
#define LETTER_POOL_ALLOW_FROM_HEAP() (drv_epic.letter_buf_pool_max > letter_pool_max)



static rt_err_t render_start(drv_epic_render_list_t list);
static rt_err_t render(drv_epic_render_list_t list);
static rt_err_t render_end(drv_epic_render_list_t list);
static rt_err_t destroy_render_list(drv_epic_render_list_t list);
static void statisttics_start(void);
static void statisttics_end(void);

extern void EPIC_GetRotatedArea(EPIC_AreaTypeDef *output, uint16_t w, uint16_t h, int16_t angle,
                                const EPIC_PointTypeDef *pivot);
/* Keep the render list pool in retention memory so sleep/resume does not
 * randomize the in-flight bookkeeping state.
 */
L1_RET_BSS_SECT_BEGIN(drv_epic_render_list_pool)
L1_RET_BSS_SECT(drv_epic_render_list_pool, ALIGN(RT_ALIGN_SIZE) static priv_render_list_t drv_epic_render_list_pool[render_list_pool_max]);
L1_RET_BSS_SECT_END

L1_NON_RET_BSS_SECT_BEGIN(drv_epic_stack)
L1_NON_RET_BSS_SECT(drv_epic_stack, ALIGN(RT_ALIGN_SIZE) static uint8_t drv_epic_mask_buf_pool[mask_buf_max_bytes]);
L1_NON_RET_BSS_SECT(drv_epic_stack, ALIGN(RT_ALIGN_SIZE) static uint8_t drv_epic_mask_buf2_pool[mask_buf2_max_bytes]);
L1_NON_RET_BSS_SECT(drv_epic_stack, ALIGN(RT_ALIGN_SIZE) static drv_epic_letter_type_t drv_epic_letter_pool[letter_pool_max * render_list_pool_max]);
#if (rotate_buf_bytes > 0)
    L1_NON_RET_BSS_SECT(drv_epic_stack, ALIGN(RT_ALIGN_SIZE) static uint8_t drv_epic_rotate_buf[rotate_buf_bytes]);
#endif
L1_NON_RET_BSS_SECT(drv_epic_stack, ALIGN(RT_ALIGN_SIZE) static uint8_t drv_epic_stack[4096]);
L1_NON_RET_BSS_SECT_END





static void print_rl(priv_render_list_t *rl)
{
    LOG_E("\n\n\nPrint Render list");
    LOG_E(FORMATED_LAYER_INFO(&rl->dst, "DST"));

    uint16_t i = 0;
    rt_list_t *pos;
    rt_list_for_each(pos, &rl->src_list)
    {
        drv_epic_operation *p_operation = rt_list_entry(pos, drv_epic_operation, list);
        char idex_str[4];
        rt_sprintf(&idex_str[0], "%d", i++);
        print_operation(&idex_str[0], p_operation);
    }
    LOG_E("\n\nPrint Render list DONE.\n");

}


static inline void add_flash_addr(uint32_t addr)
{
    if (addr == 0) return;
    void *handle = rt_flash_get_handle_by_addr(addr);
    if (handle == NULL)
    {
        return; //Not a flash address
    }

    for (uint32_t i = 0; i < max_flash_num; i++)
    {
        if (drv_epic.flash_addr[i] == 0)
        {
            //Find empty slot
            drv_epic.flash_addr[i] = addr;
            return;
        }
        else if (rt_flash_get_handle_by_addr(drv_epic.flash_addr[i]) == handle)
        {
            //Already locked
            return;
        }
        else
        {
            //Do nothing, continue
        }
    }

    RT_ASSERT(0); //Can't find empty slot
}

static void render_lock_flash(drv_epic_render_list_t list)
{
    priv_render_list_t *rl = (priv_render_list_t *)list;

    rt_list_t *pos;
    rt_list_for_each(pos, (&rl->src_list))
    {
        drv_epic_operation *o = rt_list_entry(pos, drv_epic_operation, list);

        add_flash_addr((uint32_t)o->mask.data);

        if (o->op == DRV_EPIC_DRAW_IMAGE)
            add_flash_addr((uint32_t)o->desc.blend.layer.data);

        if (o->op == DRV_EPIC_DRAW_LETTERS)
        {
            //Assume that all letters are in the same flash
            if (o->desc.label.p_letters[0].data)
                add_flash_addr((uint32_t)o->desc.label.p_letters[0].data);
        }
    }

    for (uint32_t i = 0; i < max_flash_num; i++)
    {
        if (drv_epic.flash_addr[i])
            rt_flash_lock(drv_epic.flash_addr[i]);
    }

    __DEBUG_RENDER_LOCK__(0);
}


static void render_unlock_flash(drv_epic_render_list_t list)
{
    __DEBUG_RENDER_UNLOCK__;

    for (uint32_t i = 0; i < max_flash_num; i++)
    {
        if (drv_epic.flash_addr[i])
        {
            rt_flash_unlock(drv_epic.flash_addr[i]);
            drv_epic.flash_addr[i] = 0;
        }
    }
}

static rt_err_t render_start(drv_epic_render_list_t list)
{
    statisttics_start();
    render_lock_flash(list);
#ifdef BSP_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif /*BSP_USING_PM*/
#ifdef PSRAM_CACHE_WB
    SCB_CleanDCache();
#endif

    return RT_EOK;
}


static rt_err_t render_end(drv_epic_render_list_t list)
{
    SCB_InvalidateDCache();

#ifdef BSP_USING_PM
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /*BSP_USING_PM*/
    render_unlock_flash(list);
    statisttics_end();
    return RT_EOK;
}


static rt_err_t rl_sem_take(rt_uint32_t ms)
{
    rt_err_t err;
    err = rt_sem_take(&drv_epic.rl_sema, rt_tick_from_millisecond(ms));
    DRV_EPIC_ASSERT(RT_EOK == err);
    return err;
}

static rt_err_t rl_sem_trytake(void)
{
    rt_err_t err;
    err = rt_sem_trytake(&drv_epic.rl_sema);
    return err;
}


static rt_err_t rl_sem_release(void)
{
    rt_err_t err;
    RT_ASSERT(drv_epic.rl_sema.value < render_list_pool_max);
    err = rt_sem_release(&drv_epic.rl_sema);
    return err;
}

static void push_rl_stack(priv_render_list_t *rl)
{
    RT_ASSERT(drv_epic.using_rl_count < render_list_pool_max);
    drv_epic.using_rl_stack[drv_epic.using_rl_count++] = rl;
}

static priv_render_list_t *pop_rl_stack(void)
{
    RT_ASSERT(drv_epic.using_rl_count > 0);
    return drv_epic.using_rl_stack[--drv_epic.using_rl_count];
}

static priv_render_list_t *get_rl_from_stack(void)
{
    RT_ASSERT(drv_epic.using_rl_count > 0);
    return drv_epic.using_rl_stack[drv_epic.using_rl_count - 1];
}

static void pixel_offset(uint32_t offset_bytes, uint32_t line_width, uint32_t color_bytes, int16_t *x_off, int16_t *y_off)
{
    RT_ASSERT(0 == (offset_bytes % color_bytes));
    uint32_t offset_pixels = offset_bytes / color_bytes;

    *y_off = offset_pixels / line_width;
    *x_off = offset_pixels - (line_width * (*y_off));
}

static inline int16_t merge_a2b(
    const drv_epic_render_buf *p_buf_a,
    const EPIC_LayerConfigTypeDef *p_buf_b,
    int16_t *x_off, //Vector of a->b
    int16_t *y_off,
    uint16_t *new_height)
{
    if (p_buf_a->cf != p_buf_b->color_mode) return 0;
    if (p_buf_a->area.x1 - p_buf_a->area.x0 + 1 != p_buf_b->total_width) return 0;

    uint32_t color_bytes = HAL_EPIC_GetColorDepth(p_buf_b->color_mode) >> 3;
    RT_ASSERT(color_bytes > 0);

    uint32_t p_buf_a_bytes = HAL_EPIC_AreaWidth(&p_buf_a->area) * HAL_EPIC_AreaHeight(&p_buf_a->area) * color_bytes;
    uint32_t p_buf_b_bytes = p_buf_b->total_width * p_buf_b->height * color_bytes;

    if ((p_buf_a->data == p_buf_b->data) && (p_buf_a_bytes == p_buf_b_bytes))
    {
        *x_off = 0;
        *y_off = 0;
        return 1;//Same buffer
    }
    else if (p_buf_a->data <= p_buf_b->data)
    {
        if ((p_buf_a->data + p_buf_a_bytes) >= (p_buf_b->data + p_buf_b_bytes))
        {
            pixel_offset(p_buf_b->data - p_buf_a->data, p_buf_b->total_width, color_bytes, x_off, y_off);

            return 2;//A include B
        }
        else if ((p_buf_a->data + p_buf_a_bytes) >= p_buf_b->data)
        {
            pixel_offset(p_buf_b->data - p_buf_a->data, p_buf_b->total_width, color_bytes, x_off, y_off);

            if (*x_off)
            {
                return 0;//Can't merge to 1 buffer
            }
            else
            {
                *new_height = p_buf_b->height + *y_off;
                return 3;//A&B have intersection
            }
        }
        else
        {
            return 0;//No intersection
        }
    }
    else
    {
        if ((p_buf_b->data + p_buf_b_bytes) >= (p_buf_a->data + p_buf_a_bytes))
        {
            pixel_offset(p_buf_a->data - p_buf_b->data, p_buf_b->total_width, color_bytes, x_off, y_off);
            *y_off = - *y_off;
            *x_off = - *x_off;
            return 4;//B include A
        }
        else if ((p_buf_b->data + p_buf_b_bytes) >= p_buf_a->data)
        {
            pixel_offset(p_buf_a->data - p_buf_b->data, p_buf_b->total_width, color_bytes, x_off, y_off);

            if (*x_off)
            {
                return 0;//Can't merge to 1 buffer
            }
            else
            {
                *new_height = HAL_EPIC_AreaHeight(&p_buf_a->area) + *y_off;
                *y_off = - *y_off;
                *x_off = - *x_off;
                return 5;//B&A have intersection
            }
        }
        else
        {
            return 0; //No intersection
        }
    }


}


static rt_err_t lock_render_list(drv_epic_render_list_t list)
{
    priv_render_list_t *rl = (priv_render_list_t *) list;

    rt_enter_critical();
    rl->flag |= rl_flag_rendering;
    rt_exit_critical();
    return RT_EOK;
}

static void statisttics_start(void)
{
    drv_epic.start_ms = rt_tick_get_millisecond();
    drv_epic.start_epic_wait_cnt = drv_epic.epic_handle.WaitCnt;
    drv_epic.start_epic_cnt = drv_epic.epic_handle.PerfCnt;
    drv_epic.last_rd_operation_start_epic_cnt = drv_epic.epic_handle.PerfCnt;
}

static void statisttics_end(void)
{
    EPIC_DrvTypeDef *p_drv_epic = &drv_epic;
    uint32_t cur_ms = rt_tick_get_millisecond();
    uint32_t rd_cost_ms = cur_ms - drv_epic.start_ms;

    p_drv_epic->rd_count++;
    p_drv_epic->rd_total += rd_cost_ms;

    p_drv_epic->rd_min = MIN(p_drv_epic->rd_min, rd_cost_ms);
    p_drv_epic->rd_max = MAX(p_drv_epic->rd_max, rd_cost_ms);
    p_drv_epic->rd_epic_hw_us += GetElapsedUs(drv_epic.start_epic_cnt, drv_epic.epic_handle.PerfCnt);
    p_drv_epic->rd_epic_sync_wait_us += GetElapsedUs(drv_epic.start_epic_wait_cnt, drv_epic.epic_handle.WaitCnt);

    if (cur_ms - p_drv_epic->last_statistics_ms > 2000)
    {
        uint32_t elapsed_ms = cur_ms - p_drv_epic->last_statistics_ms;
        if (p_drv_epic->dbg_flag_print_statistics)
        {
            int32_t sw_ms = p_drv_epic->rd_total - p_drv_epic->rd_usr_cb_total - p_drv_epic->rd_epic_async_wait - p_drv_epic->rd_epic_sync_wait_us / 1000;
            LOG_E("\n\n<<Render statistics(ms)>>");
            LOG_E("Elapsed %d, Rd total(%d)=usr_cb(%d)+wait_epic(async:%d,sync:%d)+sw(%d)",
                  elapsed_ms, p_drv_epic->rd_total,
                  p_drv_epic->rd_usr_cb_total, p_drv_epic->rd_epic_async_wait, p_drv_epic->rd_epic_sync_wait_us / 1000,
                  sw_ms);

            LOG_E("Total epic_hw:%d, epic_hal:%d, %d(Frames),render time min:%d max:%d ",
                  (p_drv_epic->rd_epic_hw_us / 1000), (p_drv_epic->rd_epic_hal_us / 1000),
                  p_drv_epic->rd_count, p_drv_epic->rd_min, p_drv_epic->rd_max);

            LOG_E("---Detail---:");
            for (drv_epic_op_type_t i = DRV_EPIC_DRAW_MIN; i < DRV_EPIC_DRAW_MAX; i++)
            {
                LOG_E("%s, hw:%d  sw=%d(async_wait:%d sync_wait:%d)", operation_name(i),
                      p_drv_epic->rd_operations_detail[i].hw / 1000,
                      p_drv_epic->rd_operations_detail[i].sw / 1000,
                      p_drv_epic->rd_operations_detail[i].async_wait / 1000,
                      p_drv_epic->rd_operations_detail[i].sync_wait / 1000
                     );
            }

            LOG_E("letter_pool_used_max:%d, total:%d", drv_epic.letter_pool_used_max, drv_epic.letter_buf_pool_max);


            LOG_E("---Others---:");
            LOG_E("EPIC HW usage:%d%%,  fps:%d",
                  (p_drv_epic->rd_epic_hw_us / 10) / elapsed_ms,
                  p_drv_epic->rd_count / (elapsed_ms / 1000));

#define ROUND_DIVIDE_FRAMES(v) (((v) + (p_drv_epic->rd_count >> 1)) / p_drv_epic->rd_count)
            LOG_E("Render avg(%d)= epic_hw(%d), epic_hal(%d), sw(%d), usr_cb=%d, async=%d,sync=%d",
                  ROUND_DIVIDE_FRAMES(p_drv_epic->rd_total),
                  ROUND_DIVIDE_FRAMES(p_drv_epic->rd_epic_hw_us / 1000),
                  ROUND_DIVIDE_FRAMES(p_drv_epic->rd_epic_hal_us / 1000),
                  ROUND_DIVIDE_FRAMES(sw_ms),
                  ROUND_DIVIDE_FRAMES(p_drv_epic->rd_usr_cb_total),
                  ROUND_DIVIDE_FRAMES(p_drv_epic->rd_epic_async_wait),
                  ROUND_DIVIDE_FRAMES(p_drv_epic->rd_epic_sync_wait_us / 1000)
                 );
        }

        p_drv_epic->last_statistics_ms = cur_ms;
        p_drv_epic->rd_count = 0;
        p_drv_epic->rd_min = UINT32_MAX;
        p_drv_epic->rd_max = 0;
        p_drv_epic->rd_total = 0;
        p_drv_epic->rd_usr_cb_total = 0;

        p_drv_epic->rd_epic_sync_wait_us = 0;
        p_drv_epic->rd_epic_async_wait = 0;
        p_drv_epic->rd_epic_hw_us = 0;
        p_drv_epic->rd_epic_hal_us = 0;
        memset(p_drv_epic->rd_operations_detail, 0, sizeof(p_drv_epic->rd_operations_detail));
    }
}





rt_err_t destroy_render_list(drv_epic_render_list_t list)
{
    priv_render_list_t *rl = (priv_render_list_t *) list;

    rt_list_t *pos, *next;
    rt_list_for_each_safe(pos, next, (&rl->src_list))
    {
        drv_epic_operation *o = rt_list_entry(pos, drv_epic_operation, list);
        rt_list_remove(&o->list);
        epic_free(o);
    }

    if (!LETTER_POOL_ALLOW_FROM_HEAP() && !LETTER_POOL_FROM_STACK())
    {
        epic_free((void *)rl->letter_pool);
        rl->letter_pool = &drv_epic_letter_pool[letter_pool_max * rl->letter_pool_index];
        rl->letter_pool_size = letter_pool_max;
    }

    rt_enter_critical();
    rl->flag = 0;
    rl->src_list_len = 0;
    rl->src_list_alloc_len = 0;
    rl->letter_pool_free = 0;
    rl->used = 0;
    rl->commit_area.x0 = 0;
    rl->commit_area.x1 = -1;
    rl->commit_area.y0 = 0;
    rl->commit_area.y1 = -1;
    rt_exit_critical();
    rl_sem_release();
    return RT_EOK;
}



static rt_err_t render_list(EPIC_DrvTypeDef *p_drv_epic, priv_render_list_t *rl)
{
    rt_err_t err;
    err = rt_sem_take(&p_drv_epic->render_sema, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);

#ifdef DRV_EPIC_NEW_API_DUAL_CORE_HCPU
    epic_rl_arg_t arg;
    arg.p_drv_epic = p_drv_epic;
    arg.p_render_list = rl;
    arg.p_scaled_area = NULL;
    // Send to ACPU to process render list
    err = (rt_err_t) acpu_run_task(ACPU_TASK_epic_rl, &arg, sizeof(epic_rl_arg_t), NULL);
#else
    err = drv_epic_render_list(p_drv_epic, rl);
#endif  /*DRV_EPIC_NEW_API_DUAL_CORE_HCPU*/

    rt_sem_release(&p_drv_epic->render_sema);
    return err;
}

static rt_err_t render_list_scale(EPIC_DrvTypeDef *p_drv_epic, priv_render_list_t *rl, EPIC_AreaTypeDef *p_scaled_area)
{
    rt_err_t err;
    err = rt_sem_take(&p_drv_epic->render_sema, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);

#ifdef DRV_EPIC_NEW_API_DUAL_CORE_HCPU
    epic_rl_arg_t arg;
    arg.p_drv_epic = p_drv_epic;
    arg.p_render_list = rl;
    arg.p_scaled_area = p_scaled_area;
    // Send to ACPU to process render list with scaling
    err = (rt_err_t) acpu_run_task(ACPU_TASK_epic_rl, &arg, sizeof(epic_rl_arg_t), NULL);
#else
    err = drv_epic_render_list_scale(p_drv_epic, rl, p_scaled_area);
#endif  /*DRV_EPIC_NEW_API_DUAL_CORE_HCPU*/

    rt_sem_release(&p_drv_epic->render_sema);
    return err;
}



static void epic_task(void *param)
{
    EPIC_DrvTypeDef *p_drv_epic = (EPIC_DrvTypeDef *)param;

    EPIC_MsgTypeDef msg;
    rt_err_t err;

    while (1)
    {
        p_drv_epic->task_idle = 1;
        err = rt_mq_recv(p_drv_epic->mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        p_drv_epic->task_idle = 0;

        RT_ASSERT(RT_EOK == err);

        LOG_D("epic_task exec msg%x: [%d].", msg.tick, msg.id);


        switch (msg.id)
        {
        case EPIC_MSG_RENDER_DRAW:
        case EPIC_MSG_RENDER_DRAW2:
        {
            drv_epic_render_draw_cfg *p_RenderDrawctx = &msg.content.rd;
            EPIC_AreaTypeDef *p_invalid_area = &p_RenderDrawctx->area;


            /*Start flush*/
            priv_render_list_t *rl = (priv_render_list_t *)msg.render_list;


            lock_render_list(rl);
            if (drv_epic.dbg_flag_print_rl) print_rl(rl);

            if (rl->flag & rl_flag_rendering)
            {
                EPIC_LayerConfigTypeDef *p_dst = &rl->dst;
                uint32_t color_bytes = HAL_EPIC_GetColorDepth(p_dst->color_mode) >> 3;
                uint32_t max_row;

                if (EPIC_MSG_RENDER_DRAW == msg.id)
                {
                    //Replace p_dst with drv_epic.buf1&2
                    p_dst->width = HAL_EPIC_AreaWidth(p_invalid_area);
                    p_dst->total_width = p_dst->width;
                    uint32_t max_buf = MIN(drv_epic.dbg_render_buf_max, p_drv_epic->buf_bytes);
                    max_row = (uint32_t)(max_buf / color_bytes) / p_dst->total_width;
                    max_row = RT_ALIGN_DOWN(max_row, p_RenderDrawctx->pixel_align);
                    DRV_EPIC_ASSERT(max_row > 0);
                    p_dst->height = max_row;

                    p_dst->data_size = color_bytes * p_dst->total_width * p_dst->height;
                    p_dst->x_offset = p_invalid_area->x0;


                    p_dst->data = (uint8_t *)p_drv_epic->cur_buf;
                }
                else
                {
                    //Clip the dst layer to the invalid area
                    HAL_EPIC_LayerSetDataOffset((EPIC_BlendingDataType *)p_dst, p_invalid_area->x0, p_invalid_area->y0);
                    p_dst->width = HAL_EPIC_AreaWidth(p_invalid_area);
                    p_dst->height = HAL_EPIC_AreaHeight(p_invalid_area);
                    p_dst->x_offset = p_invalid_area->x0;
                    max_row = p_dst->height;
                }

                render_start(rl);

                max_row = (DRV_EPIC_ROT_NONE == drv_epic.rotated) ? max_row : max_row - 2;

                for (int16_t start_row = p_invalid_area->y0; start_row <= p_invalid_area->y1; start_row += max_row)
                {
                    p_dst->y_offset = start_row;
                    uint32_t last;
                    if (start_row + p_dst->height - 1 >= p_invalid_area->y1)
                    {
                        p_dst->height = p_invalid_area->y1 - start_row + 1;
                        last = 1;
                    }
                    else
                    {
                        last = 0;
                    }

                    if (RT_EOK == render_list(&drv_epic, rl))
                    {
                        if (p_RenderDrawctx->partial_done_cb)
                        {
                            uint32_t usr_cb_start_ms = rt_tick_get_millisecond();
                            p_RenderDrawctx->partial_done_cb(rl, p_dst, p_RenderDrawctx->usr_data, last);
                            p_drv_epic->rd_usr_cb_total += rt_tick_get_millisecond() - usr_cb_start_ms;
                        }

                        if (EPIC_MSG_RENDER_DRAW == msg.id)
                        {
                            if (p_drv_epic->cur_buf == (uint8_t *)p_drv_epic->buf1)
                                p_drv_epic->cur_buf = (uint8_t *)p_drv_epic->buf2;
                            else
                                p_drv_epic->cur_buf = (uint8_t *)p_drv_epic->buf1;

                            p_dst->data = p_drv_epic->cur_buf;
                        }
                    }
                    else
                    {
                        LOG_E("Render list failed, the render list has not intersect with the dst layer area typically.");
                        print_rl(rl);

                        if (p_RenderDrawctx->partial_done_cb)
                        {
                            uint32_t usr_cb_start_ms = rt_tick_get_millisecond();
                            p_RenderDrawctx->partial_done_cb(rl, p_dst, p_RenderDrawctx->usr_data, last);
                            p_drv_epic->rd_usr_cb_total += rt_tick_get_millisecond() - usr_cb_start_ms;
                        }
                    }
                }

                render_end(rl);
                destroy_render_list(rl);
            }
        }
        break;

        case EPIC_MSG_RENDER_TO_BUF:
        {
            drv_epic_render_to_buf_cfg *p_Render2Buf = &msg.content.r2b;

            /*Start flush*/
            priv_render_list_t *rl = (priv_render_list_t *)msg.render_list;


            lock_render_list(rl);
            if (drv_epic.dbg_flag_print_rl) print_rl(rl);

            if (rl->flag & rl_flag_rendering)
            {
                render_start(rl);

                if (HAL_EPIC_AreaWidth(&p_Render2Buf->dst_area) != rl->dst.width ||
                        HAL_EPIC_AreaHeight(&p_Render2Buf->dst_area) != rl->dst.height)
                {
                    render_list_scale(&drv_epic, rl, &p_Render2Buf->dst_area);
                }
                else
                {
                    render_list(&drv_epic, rl);
                }

                if (p_Render2Buf->done_cb)
                {
                    uint32_t usr_cb_start_ms = rt_tick_get_millisecond();
                    p_Render2Buf->done_cb(rl, &rl->dst, p_Render2Buf->usr_data, 1);
                    p_drv_epic->rd_usr_cb_total += rt_tick_get_millisecond() - usr_cb_start_ms;
                }

                render_end(rl);
                destroy_render_list(rl);
            }
        }
        break;



        default:
            RT_ASSERT(0);
            break;
        }

    }

}


#if 0

static void update_list_op_offset(priv_render_list_t *rl,
                                  int16_t x_off, int16_t y_off //Vector from old to new
                                 )
{
    for (uint16_t idx = 0; idx < rl->src_list_len; idx++)
    {
        rl->src_list[idx].offset_x += x_off;
        rl->src_list[idx].offset_y += y_off;
    }
}

void test_merge(void)
{
    int16_t merge_ret;
    int16_t x_off, y_off; //Vector from 'p_buf(A)' to 'rl->dst(B)'
    uint16_t new_height;

    drv_epic_render_buf request_buf;
    EPIC_LayerConfigTypeDef dst;



    //Initl

    request_buf.cf = EPIC_COLOR_RGB565;
    request_buf.data = (uint8_t *)0x10000000;
    request_buf.area.x0 = 100;
    request_buf.area.y0 = 100;
    request_buf.area.x1 = 199;
    request_buf.area.y1 = 199;

    HAL_EPIC_LayerConfigInit(&dst);
    dst.data        = request_buf.data;
    dst.color_mode  = request_buf.cf;
    dst.width       = HAL_EPIC_AreaWidth(&request_buf.area);
    dst.height      = HAL_EPIC_AreaHeight(&request_buf.area);
    dst.total_width = dst.width;
    dst.x_offset    = request_buf.area.x0;
    dst.y_offset    = request_buf.area.y0;


    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(1 == merge_ret);


    request_buf.data = (uint8_t *)0x10000000 - 400;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(3 == merge_ret); //A&B
    RT_ASSERT(2 == y_off);
    RT_ASSERT(102 == new_height);


    request_buf.data = (uint8_t *)0x10000000 + 400;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(5 == merge_ret);//B&A
    RT_ASSERT(-2 == y_off);
    RT_ASSERT(102 == new_height);


    request_buf.data = (uint8_t *)0x10000000 - 400 - 4;
    request_buf.area.y0 = 100;
    request_buf.area.y1 = 299;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(2 == merge_ret);//A include B
    RT_ASSERT(2 == x_off);
    RT_ASSERT(2 == y_off);



    request_buf.data = (uint8_t *)0x10000000 + 400 + 4;
    request_buf.area.y0 = 111;
    request_buf.area.y1 = 199;
    merge_ret = merge_a2b(&request_buf, &dst, &x_off, &y_off, &new_height);
    RT_ASSERT(4 == merge_ret);//B include A
    RT_ASSERT(-2 == x_off);
    RT_ASSERT(-2 == y_off);


}
#endif /* 0 */

/**
 * Get the rectangle area that substract an area form another
 * @param aex_p pointer to an area which should not in `aholder_p`
 * @param aholder_p pointer to an area which we considered
 * @return true: `aex_p` is fully inside `aholder_p`, and the area else is a rectangle area.
 */
static bool AreaExcept(EPIC_AreaTypeDef *res_p, const EPIC_AreaTypeDef *aex_p, const EPIC_AreaTypeDef *aholder_p)
{
    if ((aholder_p->x0 == aex_p->x0) && (aholder_p->x1 == aex_p->x1))
    {
        if ((aholder_p->y0 == aex_p->y0) && (aholder_p->y1 > aex_p->y1))
        {
            res_p->x0 = aholder_p->x0;
            res_p->x1 = aholder_p->x1;
            res_p->y0 = aex_p->y1 + 1;
            res_p->y1 = aholder_p->y1;
            return true;
        }
        else if ((aholder_p->y0 < aex_p->y0) && (aholder_p->y1 == aex_p->y1))
        {
            res_p->x0 = aholder_p->x0;
            res_p->x1 = aholder_p->x1;
            res_p->y0 = aholder_p->y0;
            res_p->y1 = aex_p->y0 - 1;
            return true;
        }
    }
    else if ((aholder_p->y0 == aex_p->y0) && (aholder_p->y1 == aex_p->y1))
    {
        if ((aholder_p->x0 == aex_p->x0) && (aholder_p->x1 > aex_p->x1))
        {
            res_p->x0 = aex_p->x1 + 1;
            res_p->x1 = aholder_p->x1;
            res_p->y0 = aholder_p->y0;
            res_p->y1 = aholder_p->y1;
            return true;
        }
        else if ((aholder_p->x0 < aex_p->x0) && (aholder_p->x1 == aex_p->x1))
        {
            res_p->x0 = aholder_p->x0;
            res_p->x1 = aex_p->x0 - 1;
            res_p->y0 = aholder_p->y0;
            res_p->y1 = aholder_p->y1;
            return true;
        }
    }

    return false;
}



drv_epic_render_list_t drv_epic_alloc_render_list(drv_epic_render_buf *p_buf, EPIC_AreaTypeDef *p_ow_area)
{
    priv_render_list_t *rl_ret = NULL;

    rl_sem_take(GPU_BLEND_EXP_MS);
    rt_enter_critical();

    for (uint32_t idx = 0; idx < render_list_pool_max; idx++)
    {
        priv_render_list_t *rl = &drv_epic.render_list_pool[idx];
        if (0 == rl->used)
        {
            RT_ASSERT(0 == rl->flag);
            RT_ASSERT(0 == rl->src_list_len);

            rl->used = 1;
            rl_ret = rl;
            break;
        }
    }
    rt_exit_critical();
    RT_ASSERT(rl_ret);

    if (rl_ret)
    {
        //Make sure all operations were commited.
        RT_ASSERT(rl_ret->src_list_len == rl_ret->src_list_alloc_len);
        HAL_EPIC_LayerConfigInit(&rl_ret->dst);
        rl_ret->dst.data        = p_buf->data;
        rl_ret->dst.color_mode  = p_buf->cf;
        RT_ASSERT(HAL_EPIC_AreaWidth(&p_buf->area) > 0);
        RT_ASSERT(HAL_EPIC_AreaHeight(&p_buf->area) > 0);
        rl_ret->dst.width       = HAL_EPIC_AreaWidth(&p_buf->area);
        rl_ret->dst.height      = HAL_EPIC_AreaHeight(&p_buf->area);
        rl_ret->dst.total_width = rl_ret->dst.width;
        rl_ret->dst.x_offset    = p_buf->area.x0;
        rl_ret->dst.y_offset    = p_buf->area.y0;

        if (p_ow_area) HAL_EPIC_AreaCopy(p_ow_area, &rl_ret->commit_area);
        push_rl_stack(rl_ret);
    }
    else
    {
        LOG_E("Render list pool is full!");
        return NULL;
    }


    return (drv_epic_render_list_t)rl_ret;
}

drv_epic_operation *drv_epic_alloc_op(drv_epic_render_buf *p_buf)
{
    priv_render_list_t *found_list = NULL;
    priv_render_list_t *free_list = NULL;
    int16_t merge_ret;
    int16_t x_off, y_off; //Vector from 'p_buf(A)' to 'rl->dst(B)'
    uint16_t new_height;
    priv_render_list_t *rl;

    //test_merge();

#if 0
    //Try to find same rendering list
    for (uint32_t idx = 0; idx < render_list_pool_max; idx++)
    {
        rl = &drv_epic.render_list_pool[idx];
        if (0 == (rl->flag & rl_flag_rendering))
        {
            if (rl->used)
            {
                merge_ret = merge_a2b(p_buf, &rl->dst, &x_off, &y_off, &new_height);
                if (merge_ret > 0)
                {
                    found_list = rl;
                    break;
                }
            }
            else if (!free_list)
            {
                free_list = rl;
            }
        }
    }
#else
    rl = get_rl_from_stack();
    RT_ASSERT(rl);
    RT_ASSERT(0 == (rl->flag & rl_flag_rendering));

    if (rl->src_list_len > 0)
    {
        found_list = rl;
        merge_ret = merge_a2b(p_buf, &rl->dst, &x_off, &y_off, &new_height);
    }
    else
    {
        free_list = rl;
    }
#endif

    //Update the dst layer of rendering list
    if (found_list)
    {
        rl = found_list;

        switch (merge_ret)
        {
        case 2://A include B
            rl->dst.data        = p_buf->data;
            rl->dst.height      = HAL_EPIC_AreaHeight(&p_buf->area);
            rl->dst.x_offset    = rl->dst.x_offset - x_off;
            rl->dst.y_offset    = rl->dst.y_offset - y_off;
            break;

        case 3://A&B have intersection
            rl->dst.data        = p_buf->data;
            rl->dst.height      = new_height;
            rl->dst.x_offset    = rl->dst.x_offset - x_off;
            rl->dst.y_offset    = rl->dst.y_offset - y_off;
            break;

        case 5://B&A have intersection
            rl->dst.height      = new_height;
            break;

        case 1: //A == B
        case 4: //B include A
            break;

        default:
            RT_ASSERT(0);
            break;
        }
    }
    else if (free_list)
    {
        rl = free_list;

        HAL_EPIC_LayerConfigInit(&rl->dst);
        rl->dst.data        = p_buf->data;
        rl->dst.color_mode  = p_buf->cf;
        rl->dst.width       = HAL_EPIC_AreaWidth(&p_buf->area);
        rl->dst.height      = HAL_EPIC_AreaHeight(&p_buf->area);
        rl->dst.total_width = rl->dst.width;
        rl->dst.x_offset    = p_buf->area.x0;
        rl->dst.y_offset    = p_buf->area.y0;
    }
    else
    {
        return NULL;
    }

    RT_ASSERT(rl->src_list_len == rl->src_list_alloc_len);
    drv_epic_operation *ret_op = epic_calloc(1, sizeof(drv_epic_operation));
    if (!ret_op)
    {
        LOG_E("sys memory is full!");
        print_rl(rl);
        RT_ASSERT(0);
    }

    rt_list_insert_before(&rl->src_list, &ret_op->list);

    if ((rl == found_list) && ((4 == merge_ret) || (5 == merge_ret)))
    {
        ret_op->offset_x = x_off;
        ret_op->offset_y = y_off;
    }
    else
    {
        ret_op->offset_x = 0;
        ret_op->offset_y = 0;
    }

    rl->src_list_alloc_len++;
    RT_ASSERT(rt_list_len(&rl->src_list) == rl->src_list_alloc_len);

    return ret_op;
}


rt_err_t drv_epic_commit_op(drv_epic_operation *op)
{
    priv_render_list_t *rl = get_rl_from_stack();
    bool curr_op_merged = false;
    /*Nodes to be deleted during merging:
    Keep the new op, delete the old prev_op (where op is the immediate successor of prev_op,
    and after deleting prev_op, op naturally connects to the previous node of the original prev_op). Default = op
    (for some branches, the content is written to prev_op and the new op is deleted). */
    drv_epic_operation *merged_op_to_remove = op;

    RT_ASSERT(rl);
    RT_ASSERT(0 == (rl->flag & (rl_flag_rendering)));
    RT_ASSERT(rl->used);
    RT_ASSERT(rl->src_list_len + 1 == rl->src_list_alloc_len);

    HAL_EPIC_AreaMove(&op->clip_area, op->offset_x, op->offset_y);

    if (DRV_EPIC_DRAW_IMAGE == op->op)
        RT_ASSERT((uint32_t)op->desc.blend.layer.data != drv_epic.dbg_src_addr);

    if (0xFFFFFFFF != drv_epic.dbg_flag_dis_merge_operations)
    {
        if (rl->src_list_len > 1)
        {
            RT_ASSERT(op->list.prev != &rl->src_list);
            drv_epic_operation *prev_op = rt_list_entry(op->list.prev, drv_epic_operation, list);
            drv_epic_operation *curr_op = op;

            if ((DRV_EPIC_DRAW_FILL == prev_op->op) && (prev_op->desc.fill.opa >= OPA_MAX) && (NULL == prev_op->mask.data)
                    && (DRV_EPIC_DRAW_IMAGE == op->op) && (EPIC_BLEND_MODE_FIXED_BG == op->desc.blend.use_dest_as_bg)
                    && (0 == op->desc.blend.layer.transform_cfg.angle) && (0 == op->desc.blend.layer.transform_cfg.type)
                    && (EPIC_INPUT_SCALE_NONE == op->desc.blend.layer.transform_cfg.scale_x)
                    && (EPIC_INPUT_SCALE_NONE == op->desc.blend.layer.transform_cfg.scale_y)
               )
            {
                if (drv_epic.dbg_flag_dis_merge_operations & 1) goto __COMMIT_OPERATION;

                EPIC_AreaTypeDef prev_area = prev_op->clip_area;
                //HAL_EPIC_AreaMove(&prev_area, prev_op->offset_x, prev_op->offset_y);
                //HAL_EPIC_AreaMove(&prev_area, -op->offset_x, -op->offset_y);

                /*    if (HAL_EPIC_AreaIsIn(&prev_area, &op->clip_area))
                    {
                        //Merge filling&blending operations to one
                        op->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_FIXED_BG;
                        op->desc.blend.r = prev_op->desc.fill.r;
                        op->desc.blend.g = prev_op->desc.fill.g;
                        op->desc.blend.b = prev_op->desc.fill.b;

                        memcpy(prev_op, op, sizeof(drv_epic_operation));
                        rl->src_list_len--;
                    }
                    else
                */
                if (HAL_EPIC_AreaIsIn(&op->clip_area, &prev_area))
                {
                    //Merge filling&blending operations to one
                    op->desc.blend.use_dest_as_bg = EPIC_BLEND_MODE_FIXED_BG;
                    op->desc.blend.r = prev_op->desc.fill.r;
                    op->desc.blend.g = prev_op->desc.fill.g;
                    op->desc.blend.b = prev_op->desc.fill.b;

                    op->clip_area = prev_area;

                    /* Keep op (which already includes image + bg), and remove the old fill prev_op*/
                    merged_op_to_remove = prev_op;
                    curr_op_merged = true;
                }
            }
            else if ((DRV_EPIC_DRAW_FILL == prev_op->op) && (DRV_EPIC_DRAW_FILL == curr_op->op))
            {
                if (drv_epic.dbg_flag_dis_merge_operations & 2) goto __COMMIT_OPERATION;

                if ((NULL == prev_op->mask.data) && (NULL == curr_op->mask.data))
                {
                    EPIC_AreaTypeDef res_area;

                    if (AreaExcept(&res_area, &prev_op->clip_area, &curr_op->clip_area))
                    {
                        RT_ASSERT(HAL_EPIC_AreaIsValid(&res_area));
                        if (drv_epic.dbg_flag_dis_merge_operations & 4) goto __COMMIT_OPERATION;

                        if (curr_op->desc.fill.opa >= OPA_MAX)
                        {
                            /* curr(op) completely replaces prev_op: retains op and deletes the old prev_op */
                            merged_op_to_remove = prev_op;
                            curr_op_merged = true;
                        }
                        else if (prev_op->desc.fill.opa >= OPA_MAX)
                        {
                            //TODO  Reduce current area, and mix current color to previous color
                        }
                    }
                    else if (AreaExcept(&res_area, &curr_op->clip_area, &prev_op->clip_area))
                    {
                        RT_ASSERT(HAL_EPIC_AreaIsValid(&res_area));
                        if (drv_epic.dbg_flag_dis_merge_operations & 8) goto __COMMIT_OPERATION;

                        if (curr_op->desc.fill.opa >= OPA_MAX)
                        {
                            prev_op->clip_area = res_area; //Reduce previous area
                        }
                        else if (prev_op->desc.fill.opa >= OPA_MAX)
                        {
                            //TODO  Reduce current area, and mix current color to previous color
                        }
                    }
                    else if (HAL_EPIC_AreaIsIn(&prev_op->clip_area, &curr_op->clip_area))
                    {
                        if (drv_epic.dbg_flag_dis_merge_operations & 0x10) goto __COMMIT_OPERATION;

                        if (curr_op->desc.fill.opa >= OPA_MAX)
                        {
                            /* curr(op) is non-transparent and completely covers prev_op: op is already the correct fill,
                               so just delete the old prev_op. */
                            merged_op_to_remove = prev_op;
                            curr_op_merged = true;
                        }
                        else if (HAL_EPIC_AreaIsIn(&curr_op->clip_area, &prev_op->clip_area)) //Same area
                        {
                            //Todo Mix current color to previous color
                        }
                    }

                }
            }

        }
    }

__COMMIT_OPERATION:
    if (curr_op_merged)
    {
        /* Keep the new op and remove the old node "merged_op_to_remove" (usually "prev_op").
        Op is the immediate successor of "prev_op", and after removing "prev_op", op naturally takes its original position. */
        rt_list_remove(&merged_op_to_remove->list);
        rl->src_list_alloc_len--;
        epic_free(merged_op_to_remove);
        RT_ASSERT(rl->src_list_len == rl->src_list_alloc_len);
        return RT_EOK;
    }

    /*Commit operation*/
    rl->src_list_len++;
    return RT_EOK;
}


drv_epic_letter_type_t *drv_epic_op_alloc_letter(drv_epic_operation *op)
{
    priv_render_list_t *rl = get_rl_from_stack();

    RT_ASSERT(rl);
    RT_ASSERT(0 == (rl->flag & (rl_flag_rendering)));
    RT_ASSERT(rl->letter_pool_free < rl->letter_pool_size);

    if (0 == rl->letter_pool_free && LETTER_POOL_ALLOW_FROM_HEAP() && LETTER_POOL_FROM_STACK())
    {
        uint32_t size = sizeof(drv_epic_letter_type_t) * drv_epic.letter_buf_pool_max;
        rl->letter_pool_size = drv_epic.letter_buf_pool_max;
        rl->letter_pool = epic_calloc(1, size);
        RT_ASSERT(rl->letter_pool);

        LOG_D("alloc new letter_pool[0x%p] size[%d]", rl->letter_pool, size);
    }
    drv_epic_letter_type_t *p_free = &rl->letter_pool[rl->letter_pool_free];
    rl->letter_pool_free++;


    if (!op->desc.label.p_letters)
    {
        op->desc.label.letter_num = 1;
        op->desc.label.p_letters = p_free;
    }
    else
    {
        RT_ASSERT(op->desc.label.p_letters + op->desc.label.letter_num == p_free);

        op->desc.label.letter_num++;
    }

    return p_free;
}




rt_err_t drv_epic_render_msg_commit(EPIC_MsgTypeDef *p_msg)
{
    bool send_msg;
    priv_render_list_t *rl = (priv_render_list_t *)p_msg->render_list;
    RT_ASSERT(rl->src_list_len == rl->src_list_alloc_len);
    RT_ASSERT(rl == get_rl_from_stack());

    rt_enter_critical();
    if (rl_flag_commit & rl->flag)
    {
        send_msg = false;
        //RT_ASSERT(drv_epic.mq->entry > 0);
    }
    else
        send_msg = true;
    rl->flag |= rl_flag_commit;
    rt_exit_critical();

    if (EPIC_MSG_RENDER_DRAW == p_msg->id)
    {
        HAL_EPIC_AreaCopy(&rl->commit_area, &p_msg->content.rd.area);
    }

    pop_rl_stack();
    if (send_msg)
    {
        rt_err_t err;
        p_msg->tick = rt_tick_get();
        err = rt_mq_send(drv_epic.mq, p_msg, sizeof(EPIC_MsgTypeDef));
        if (err != RT_EOK)
        {
            RT_ASSERT(0);
        }
        return err;
    }

    return RT_EOK;
}

rt_err_t drv_epic_render_list_init(void)
{
    if (NULL == drv_epic.mq)
    {
        drv_epic.render_list_pool = &drv_epic_render_list_pool[0];
        drv_epic.mask_buf_pool = &drv_epic_mask_buf_pool[0];
        drv_epic.mask_buf2_pool = &drv_epic_mask_buf2_pool[0];
#if (rotate_buf_bytes > 0)
        drv_epic.rotate_buf = &drv_epic_rotate_buf[0];
#endif

        if (drv_epic.render_list_pool)
        {
            for (uint32_t i = 0; i < render_list_pool_max; i++)
            {
                drv_epic.render_list_pool[i].used = 0;
                drv_epic.render_list_pool[i].flag = 0;
                drv_epic.render_list_pool[i].src_list_alloc_len = 0;
                drv_epic.render_list_pool[i].src_list_len = 0;
                drv_epic.render_list_pool[i].letter_pool_free = 0;
                drv_epic.render_list_pool[i].commit_area.x0 = 0;
                drv_epic.render_list_pool[i].commit_area.x1 = -1;
                drv_epic.render_list_pool[i].commit_area.y0 = 0;
                drv_epic.render_list_pool[i].commit_area.y1 = -1;
                drv_epic.render_list_pool[i].letter_pool = &drv_epic_letter_pool[letter_pool_max * i];
                drv_epic.render_list_pool[i].letter_pool_size = letter_pool_max;
                drv_epic.render_list_pool[i].letter_pool_index = i;
                rt_list_init(&drv_epic.render_list_pool[i].src_list);
            }
        }


        rt_err_t err = rt_sem_init(&drv_epic.rl_sema, "epic_rl", render_list_pool_max, RT_IPC_FLAG_FIFO);
        RT_ASSERT(RT_EOK == err);

        err = rt_sem_init(&drv_epic.render_sema, "epic_render", 1, RT_IPC_FLAG_FIFO);
        RT_ASSERT(RT_EOK == err);


        drv_epic.mq = rt_mq_create("drv_epic", sizeof(EPIC_MsgTypeDef), 4, RT_IPC_FLAG_FIFO);
        RT_ASSERT(drv_epic.mq);

        uint16_t priority = RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_LOWWER;
#ifdef SOLUTION
        priority = RT_THREAD_PRIORITY_HIGH - 3;
#endif

        drv_epic.magic_num = EPIC_DRV_MAGIC_NUM;
        rt_err_t ret = rt_thread_init(&drv_epic.task, "epic_task", epic_task, &drv_epic, drv_epic_stack, sizeof(drv_epic_stack),
                                      priority, RT_THREAD_TICK_DEFAULT);

        RT_ASSERT(RT_EOK == ret);
        rt_thread_startup(&drv_epic.task);
    }
    else
    {
        LOG_W("drv_epic_render_list_init() already called.");
    }

    return RT_EOK;
}
rt_err_t drv_epic_setup_render_buffer(uint8_t *buf1, uint8_t *buf2, uint32_t buf_bytes)
{
    drv_epic.buf1 = buf1;
    drv_epic.buf2 = buf2;
    drv_epic.buf_bytes = buf_bytes;
    drv_epic.cur_buf = buf1;


    drv_epic_render_list_init();
    return RT_EOK;
}

rt_thread_t drv_epic_task_get(void)
{
    return drv_epic.task.name[0] ? &drv_epic.task : NULL;
}

rt_err_t drv_epic_render_trav(drv_epic_render_list_t list, drv_epic_render_trav_cb cb, void *usr_data)
{
    if (!list || !cb)
    {
        LOG_E("%s: parameter[list] %p [cb] %p is null.", __func__, list, cb);
        return RT_ERROR;
    }
    priv_render_list_t *rl = (priv_render_list_t *)list;

    rt_list_t *pos;
    rt_list_for_each(pos, (&rl->src_list))
    {
        drv_epic_operation *o = rt_list_entry(pos, drv_epic_operation, list);
        cb(o, usr_data);
    }

    return RT_EOK;
}

rt_err_t drv_epic_set_letter_pool_size(uint32_t pool_size)
{
    if (pool_size > letter_pool_max)
        drv_epic.letter_buf_pool_max = pool_size;
    else
        drv_epic.letter_buf_pool_max = letter_pool_max;

    return RT_EOK;
}

rt_err_t drv_epic_set_rotation(drv_epic_rotate_t rotate)
{
    drv_epic.rotated = rotate;

    return RT_EOK;
}

drv_epic_rotate_t drv_epic_get_rotation(void)
{
    return drv_epic.rotated;
}

static int16_t drv_epic_get_rotate_angle(void)
{
    switch (drv_epic.rotated)
    {
    case DRV_EPIC_ROT_90:
        return 90;
    case DRV_EPIC_ROT_180:
        return 180;
    case DRV_EPIC_ROT_270:
        return 270;
    default:
        return 0;
    }
}

static void epic_copy_cplt_callback(EPIC_HandleTypeDef *epic)
{
    drv_epic_cplt_cbk cbk = epic->user_data;
    if (cbk) cbk(epic);
    rt_sem_release(&drv_epic.render_sema);
}

rt_err_t drv_epic_copy(const uint8_t *src, uint8_t *dst,
                       const EPIC_AreaTypeDef *src_area,
                       const EPIC_AreaTypeDef *dst_area,
                       const EPIC_AreaTypeDef *copy_area,
                       uint32_t src_cf, uint32_t dst_cf,
                       drv_epic_cplt_cbk cbk)
{
    RT_ASSERT((RT_NULL != src_area) && (RT_NULL != dst_area) && (RT_NULL != copy_area));
    RT_ASSERT(HAL_EPIC_AreaIsIn(copy_area, src_area) && HAL_EPIC_AreaIsIn(copy_area, dst_area));

    rt_err_t err;
    EPIC_HandleTypeDef *h_epic = &drv_epic.epic_handle;
    EPIC_LayerConfigTypeDef input_layer;
    EPIC_LayerConfigTypeDef output_layer;
    int16_t angle = drv_epic_get_rotate_angle() * 10;

    EPIC_PointTypeDef dst_pivot;
    dst_pivot.x = dst_area->x0 + (HAL_EPIC_AreaWidth(dst_area) >> 1);
    dst_pivot.y = dst_area->y0 + (HAL_EPIC_AreaHeight(dst_area) >> 1);

    HAL_EPIC_LayerConfigInit(&input_layer);

    input_layer.data = (uint8_t *)src;
    input_layer.color_mode = src_cf;
    input_layer.height = HAL_EPIC_AreaHeight(src_area);
    input_layer.total_width = HAL_EPIC_AreaWidth(src_area);
    input_layer.width = HAL_EPIC_AreaWidth(src_area);
    input_layer.x_offset = src_area->x0;
    input_layer.y_offset = src_area->y0;
    input_layer.data_size = get_layer_size((EPIC_BlendingDataType *)&input_layer);

    input_layer.transform_cfg.angle        = angle;
    input_layer.transform_cfg.pivot_x = dst_pivot.x - src_area->x0;
    input_layer.transform_cfg.pivot_y = dst_pivot.y - src_area->y0;

    EPIC_PointTypeDef rotated_pivot;
    rotated_pivot.x = input_layer.transform_cfg.pivot_x;
    rotated_pivot.y = input_layer.transform_cfg.pivot_y;
    EPIC_AreaTypeDef rotated_area;
    rotated_area = *copy_area;
    int16_t rotated_width = HAL_EPIC_AreaWidth(&rotated_area);
    int16_t rotated_height = HAL_EPIC_AreaHeight(&rotated_area);
    EPIC_GetRotatedArea((EPIC_AreaTypeDef *)&rotated_area, rotated_width, rotated_height, angle, &rotated_pivot);

    switch (drv_epic.rotated)
    {
    case DRV_EPIC_ROT_90:
    {
        rotated_area.x0 += src_area->x0;
        if (src_area->y0 >= dst_pivot.y)
            rotated_area.x1 += src_area->x0 - 2;
        else
            rotated_area.x1 += src_area->x0;
        rotated_area.y0 += src_area->y0;
        rotated_area.y1 += src_area->y0;

        break;
    }
    case DRV_EPIC_ROT_180:
    {
        rotated_area.x0 += src_area->x0;
        rotated_area.x1 += src_area->x0;
        rotated_area.y0 += src_area->y0;
        if (src_area->y0 > dst_pivot.y)
            rotated_area.y1 += src_area->y0 - 2;
        else
            rotated_area.y1 += src_area->y0 - 1;
        break;
    }
    case DRV_EPIC_ROT_270:
    {
        if (src_area->y0 > dst_pivot.y)
            rotated_area.x0 += src_area->x0 + 4;
        else
            rotated_area.x0 += src_area->x0 + 2;
        rotated_area.x1 += src_area->x0;
        rotated_area.y0 += src_area->y0;
        rotated_area.y1 += src_area->y0;
        break;
    }
    default:
    {
        rotated_area.x0 += src_area->x0;
        rotated_area.x1 += src_area->x0;
        rotated_area.y0 += src_area->y0;
        rotated_area.y1 += src_area->y0;
        break;
    }
    }

    clip_intersect_area(&rotated_area, (EPIC_AreaTypeDef *)dst_area);

    HAL_EPIC_LayerConfigInit(&output_layer);
    output_layer.color_mode = dst_cf;
    output_layer.total_width = HAL_EPIC_AreaWidth(dst_area);

    clip_layer_to_area((EPIC_BlendingDataType *)&output_layer, dst, dst_area->x0, dst_area->y0, &rotated_area);


    err = rt_sem_take(&drv_epic.render_sema, rt_tick_from_millisecond(GPU_BLEND_EXP_MS));
    if (RT_EOK != err)
    {
        print_gpu_error_info();
        RT_ASSERT(0);
    }

    h_epic->user_data = (void *)cbk;
    h_epic->XferCpltCallback = epic_copy_cplt_callback;

    HAL_StatusTypeDef ret = HAL_EPIC_BlendStartEx_IT(h_epic, &input_layer, 1, &output_layer);
    return (HAL_OK == ret) ? RT_EOK : RT_ERROR;
}




rt_err_t drv_gpu_take(rt_int32_t ms)
{
    rt_err_t err;

    err = rt_sem_take(&drv_epic.render_sema, rt_tick_from_millisecond(ms));


    if (RT_EOK != err)
    {
        print_gpu_error_info();
    }

    return err;
}


rt_err_t drv_gpu_release(void)
{
    return rt_sem_release(&drv_epic.render_sema);
}



rt_err_t drv_gpu_check_done(rt_int32_t ms)
{
    rt_err_t err;
    err = rt_sem_take(&drv_epic.render_sema, rt_tick_from_millisecond(ms));
    if (err == RT_EOK)
    {
        rt_sem_release(&drv_epic.render_sema);
        return RT_EOK;
    }
    else
    {
        print_gpu_error_info();
        return -RT_ETIMEOUT;
    }
}


bool drv_gpu_is_busy(void)
{
    if (drv_epic.mq) if (drv_epic.mq->entry > 0) return true;
    if (drv_epic.task_idle != 1) return true;
    if (-RT_ETIMEOUT == rt_sem_trytake(&drv_epic.render_sema))
        return true;
    else
    {
        RT_ASSERT(0 == drv_epic.render_sema.value);
        rt_sem_release(&drv_epic.render_sema);
        return false;
    }

}


#endif /*DRV_EPIC_NEW_API*/
