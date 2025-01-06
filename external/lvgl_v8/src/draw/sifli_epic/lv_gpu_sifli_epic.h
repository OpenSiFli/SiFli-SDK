
#ifndef _LV_GPU_SIFLI_EPIC_H
#define _LV_GPU_SIFLI_EPIC_H


typedef struct
{
    lv_draw_sw_ctx_t sw_ctx;

    /***** EPIC ctx ******/
    lv_draw_sw_ctx_t sw_ctx_backup;

    uint32_t reserved;
} epic_draw_ctx_t;



void lv_gpu_epic_ctx_init(lv_disp_drv_t *drv, lv_draw_ctx_t *draw_ctx);
void lv_gpu_epic_ctx_deinit(lv_disp_drv_t *drv, lv_draw_ctx_t *draw_ctx);

#endif /*_LV_GPU_SIFLI_EPIC_H*/