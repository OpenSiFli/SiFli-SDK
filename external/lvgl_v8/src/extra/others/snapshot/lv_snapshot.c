/**
 * @file lv_snapshot.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "app_mem.h"
#include "lv_snapshot.h"
#if LV_USE_SNAPSHOT

#include <stdbool.h>
#include "../../../core/lv_disp.h"
#include "../../../core/lv_refr.h"
#ifdef BSP_USING_EPIC
#include "drv_epic.h"
#endif

#undef snapshot_malloc
#undef snapshot_free

#ifdef SOLUTION
    #define snapshot_malloc app_malloc
    #define snapshot_free   app_free
#else
    #define snapshot_malloc rt_malloc
    #define snapshot_free   rt_free
#endif
/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static uint8_t snapshot_log_en = 0;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/** Get the buffer needed for object snapshot image.
 *
 * @param obj    The object to generate snapshot.
 * @param cf     color format for generated image.
 *
 * @return the buffer size needed in bytes
 */
uint32_t lv_snapshot_buf_size_needed(lv_obj_t * obj, lv_img_cf_t cf)
{
    LV_ASSERT_NULL(obj);
    switch(cf) {
        case LV_IMG_CF_TRUE_COLOR:
        case LV_IMG_CF_TRUE_COLOR_ALPHA:
        case LV_IMG_CF_ALPHA_1BIT:
        case LV_IMG_CF_ALPHA_2BIT:
        case LV_IMG_CF_ALPHA_4BIT:
        case LV_IMG_CF_ALPHA_8BIT:
            break;
        default:
            return 0;
    }

    lv_obj_update_layout(obj);

    /*Width and height determine snapshot image size.*/
    lv_coord_t w = lv_obj_get_width(obj);
    lv_coord_t h = lv_obj_get_height(obj);
    lv_coord_t ext_size = _lv_obj_get_ext_draw_size(obj);
    w += ext_size * 2;
    h += ext_size * 2;

    uint8_t px_size = lv_img_cf_get_px_size(cf);
    return w * h * ((px_size + 7) >> 3);
}

/** Take snapshot for object with its children, save image info to provided buffer.
 *
 * @param obj    The object to generate snapshot.
 * @param cf     color format for generated image.
 * @param dsc    image descriptor to store the image result.
 * @param buf    the buffer to store image data.
 * @param buff_size provided buffer size in bytes.
 *
 * @return LV_RES_OK on success, LV_RES_INV on error.
 */
lv_res_t lv_snapshot_take_to_buf(lv_obj_t * obj, lv_img_cf_t cf, lv_img_dsc_t * dsc, void * buf, uint32_t buff_size)
{
    LV_ASSERT_NULL(obj);
    LV_ASSERT_NULL(dsc);
    LV_ASSERT_NULL(buf);

    switch(cf) {
        case LV_IMG_CF_TRUE_COLOR:
        case LV_IMG_CF_TRUE_COLOR_ALPHA:
        case LV_IMG_CF_ALPHA_1BIT:
        case LV_IMG_CF_ALPHA_2BIT:
        case LV_IMG_CF_ALPHA_4BIT:
        case LV_IMG_CF_ALPHA_8BIT:
            break;
        default:
            return LV_RES_INV;
    }

    if(lv_snapshot_buf_size_needed(obj, cf) > buff_size)
        return LV_RES_INV;

    uint32_t tick = rt_tick_get();

    /*Width and height determine snapshot image size.*/
    lv_coord_t w = lv_obj_get_width(obj);
    lv_coord_t h = lv_obj_get_height(obj);
    lv_coord_t ext_size = _lv_obj_get_ext_draw_size(obj);
    w += ext_size * 2;
    h += ext_size * 2;

    lv_area_t snapshot_area;
    lv_obj_get_coords(obj, &snapshot_area);
    lv_area_increase(&snapshot_area, ext_size, ext_size);

    lv_memset(buf, 0x00, buff_size);
    lv_memset_00(dsc, sizeof(lv_img_dsc_t));

    lv_disp_t * obj_disp = lv_obj_get_disp(obj);
    lv_disp_drv_t driver;
    lv_disp_drv_init(&driver);
    /*In lack of a better idea use the resolution of the object's display*/
    driver.hor_res = lv_disp_get_hor_res(obj_disp);
    driver.ver_res = lv_disp_get_hor_res(obj_disp);
    lv_disp_drv_use_generic_set_px_cb(&driver, cf);

    lv_disp_t fake_disp;
    lv_memset_00(&fake_disp, sizeof(lv_disp_t));
    fake_disp.driver = &driver;

    lv_draw_ctx_t * draw_ctx = lv_mem_alloc(obj_disp->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if(draw_ctx == NULL) return LV_RES_INV;
    obj_disp->driver->draw_ctx_init(fake_disp.driver, draw_ctx);
    fake_disp.driver->draw_ctx = draw_ctx;
    draw_ctx->clip_area = &snapshot_area;
    draw_ctx->buf_area = &snapshot_area;
    draw_ctx->buf = (void *)buf;
    driver.draw_ctx = draw_ctx;

    lv_disp_t * refr_ori = _lv_refr_get_disp_refreshing();
    _lv_refr_set_disp_to_draw_start(&fake_disp);

    lv_obj_redraw(draw_ctx, obj);

    _lv_refr_set_disp_to_draw_end(refr_ori, NULL);
    obj_disp->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);

    dsc->data = buf;
    dsc->header.w = w;
    dsc->header.h = h;
    dsc->header.cf = cf;
    dsc->data_size = lv_img_buf_get_img_size(w, h, cf);
    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);
    return LV_RES_OK;
}

/** Take snapshot for object with its children, alloc the memory needed.
 *
 * @param obj    The object to generate snapshot.
 * @param cf     color format for generated image.
 *
 * @return a pointer to an image descriptor, or NULL if failed.
 */
lv_img_dsc_t * lv_snapshot_take(lv_obj_t * obj, lv_img_cf_t cf)
{
    LV_ASSERT_NULL(obj);
    uint32_t tick = rt_tick_get();
    uint32_t buff_size = lv_snapshot_buf_size_needed(obj, cf);

    void * buf = lv_mem_alloc(buff_size);
    LV_ASSERT_MALLOC(buf);
    if(buf == NULL) {
        return NULL;
    }

    lv_img_dsc_t * dsc = lv_mem_alloc(sizeof(lv_img_dsc_t));
    LV_ASSERT_MALLOC(buf);
    if(dsc == NULL) {
        lv_mem_free(buf);
        return NULL;
    }

    if(lv_snapshot_take_to_buf(obj, cf, dsc, buf, buff_size) == LV_RES_INV) {
        lv_mem_free(buf);
        lv_mem_free(dsc);
        return NULL;
    }
    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return dsc;
}

/** Free the snapshot image returned by @ref lv_snapshot_take
 *
 * It will firstly free the data image takes, then the image descriptor.
 *
 * @param dsc    The image descriptor generated by lv_snapshot_take.
 *
 */
void lv_snapshot_free(lv_img_dsc_t * dsc)
{
    if(!dsc)
        return;

    if(dsc->data)
        lv_mem_free((void *)dsc->data);

    lv_mem_free(dsc);
}


#include "../../../draw/lv_draw.h"

extern void *get_disp_buf(uint32_t size);
static void init_fake_disp(lv_disp_t *ori_disp, 
                            lv_disp_t *fake_disp, 
                            lv_disp_drv_t *fake_drv,
                            lv_area_t *fake_clip_area,
                            lv_img_dsc_t * dsc)
{
    fake_clip_area->x1 = 0;
    fake_clip_area->x2 = dsc->header.w - 1;
    fake_clip_area->y1 = 0;
    fake_clip_area->y2 = dsc->header.h - 1;

    /*Allocate the fake driver on the stack as the entire display doesn't outlive this function*/
    lv_memset_00(fake_disp, sizeof(lv_disp_t));
    fake_disp->driver = fake_drv;

    //lv_disp_drv_init(fake_drv);

    //lv_disp_drv_init(disp->driver);
    fake_disp->driver->hor_res = dsc->header.w;
    fake_disp->driver->ver_res = dsc->header.h;

    lv_draw_ctx_t * draw_ctx = lv_mem_alloc(ori_disp->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if(draw_ctx == NULL)  return;
    ori_disp->driver->draw_ctx_init(fake_drv, draw_ctx);
    fake_drv->draw_ctx = draw_ctx;
    draw_ctx->clip_area = fake_clip_area;
    draw_ctx->buf_area = fake_clip_area;
    draw_ctx->buf = (void *)dsc->data;

    lv_disp_drv_use_generic_set_px_cb(fake_drv, dsc->header.cf);
    if(LV_COLOR_SCREEN_TRANSP && dsc->header.cf != LV_IMG_CF_TRUE_COLOR_ALPHA) {
        fake_drv->screen_transp = 0;
    }
}

static void deinit_fake_disp(lv_disp_t *ori_disp, lv_disp_t * fake_disp)
{
    ori_disp->driver->draw_ctx_deinit(fake_disp->driver, fake_disp->driver->draw_ctx);
    lv_mem_free(fake_disp->driver->draw_ctx);
}

//Dump act framebuffer to img_dsc immediately
lv_res_t lv_refr_dump_buf_to_img_now(lv_img_dsc_t *img_dsc)
{
    lv_disp_t *ori_disp = lv_disp_get_default();
    lv_disp_draw_buf_t *disp_buf = lv_disp_get_draw_buf(ori_disp);

    lv_draw_ctx_t *draw_ctx = ori_disp->driver->draw_ctx;

    /*Create img use framebuffer*/
    lv_img_dsc_t img_screen;
    lv_coord_t hor_res = lv_disp_get_hor_res(ori_disp);
    lv_coord_t ver_res = lv_disp_get_ver_res(ori_disp);
    img_screen.header.always_zero = 0;
    img_screen.header.w = hor_res;
    img_screen.header.h = ver_res;
    img_screen.data_size  = (LV_COLOR_DEPTH * hor_res * ver_res) / 8;
    img_screen.header.cf = LV_IMG_CF_TRUE_COLOR,
    img_screen.data = (uint8_t *)get_disp_buf(img_screen.data_size);
    if(NULL == img_screen.data)    return LV_RES_INV;


    LV_LOG_TRACE("lv_refr_dump_buf_to_img_now \r\n");

    //2. Scale img_screen to img_dsc
    {
        /* Create a dummy display to fool the lv_draw function.
         * It will think it draws to real screen. */
        lv_draw_img_dsc_t img_draw_dsc;

        lv_draw_img_dsc_init(&img_draw_dsc);
        img_draw_dsc.zoom = ((uint32_t) img_dsc->header.w) * LV_IMG_ZOOM_NONE / ((uint32_t)hor_res);
        //img_draw_dsc.recolor_opa = LV_OPA_TRANSP;




        /*Create a dummy display to fool the lv_draw function.
         *It will think it draws to real screen.*/
        lv_disp_t fake_disp;
        lv_disp_drv_t driver;
        lv_area_t clip_area;
        lv_memcpy(&driver, ori_disp->driver, sizeof(lv_disp_drv_t));
        init_fake_disp(ori_disp, &fake_disp, &driver, &clip_area,img_dsc);
        
        if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);
        _lv_refr_set_disp_to_draw_start(&fake_disp);
        
       
        lv_draw_img(driver.draw_ctx, &img_draw_dsc, &clip_area, &img_screen);
        if (driver.draw_ctx->wait_for_finish) driver.draw_ctx->wait_for_finish(driver.draw_ctx);
        _lv_refr_set_disp_to_draw_end(ori_disp, NULL);

        deinit_fake_disp(ori_disp, &fake_disp);
    }

    return LV_RES_OK;

}


#ifdef BSP_USING_PC_SIMULATOR
lv_color_t sim_bf[LV_HOR_RES_MAX * LV_VER_RES_MAX];
void* lv_snapshot_to_sim_buf(uint8_t *buf)
{
    if (16 == LV_COLOR_DEPTH)
        return buf;
    for (uint32_t i = 0; i < LV_VER_RES_MAX; i++)
    {
        for (uint32_t j = 0; j < LV_HOR_RES_MAX; j++)
        {
            uint32_t index = i * LV_HOR_RES_MAX + j;
            uint32_t src_i = index * 4;
            uint32_t dst_i = index * 3;
            sim_bf[index] = lv_color_make(buf[src_i + 2], buf[src_i + 1], buf[src_i + 0]);
        }
    }
    return sim_bf;
}
#endif
extern void* get_disp_buf(uint32_t size);

void lv_snapshot_dsc_init(lv_img_dsc_t *dsc, lv_coord_t w, lv_coord_t h, lv_area_t *clip_area, lv_area_t *buf_area)
{
    clip_area->x1 = 0;
    clip_area->y1 = 0;
    clip_area->x2 = w - 1;
    clip_area->y2 = h - 1;

    buf_area->x1 = 0;
    buf_area->y1 = 0;
    buf_area->x2 = dsc->header.w - 1;
    buf_area->y2 = dsc->header.h - 1;

    if (LV_IMG_CF_TRUE_COLOR == dsc->header.cf)
    {
        lv_color_t *p = (lv_color_t*)dsc->data;
        lv_color_t back = lv_color_make(0, 0, 0);
        for (int32_t j = 0; j < dsc->header.h; j++)
        {
            for (int32_t i = clip_area->x2 + 1; i < buf_area->x2 + 1; i++)
            {
                p[i - buf_area->x1 + j * dsc->header.w] = back;
            }
        }

        for (int32_t i = 0; i < dsc->header.w; i++)
        {
            for (int32_t j = clip_area->y2 + 1; j < buf_area->y2 + 1; j++)
            {
                p[i + (j - buf_area->y1) * dsc->header.w] = back;
            }
        }
    }
    else
    {
        lv_color_t back = lv_color_make(0, 0, 0);
        int32_t pix_size = LV_COLOR_DEPTH >> 3;
        for (int32_t j = 0; j < dsc->header.h; j++)
        {
            for (int32_t i = clip_area->x2 + 1; i < buf_area->x2 + 1; i++)
            {
                int32_t index = (i - buf_area->x1 + j * dsc->header.w) * (pix_size + 1);
                uint8_t *p = (uint8_t*)(dsc->data + index);
                *((lv_color_t*)p) = back;
                *(p + pix_size) = 0;
            }
        }

        for (int32_t i = 0; i < dsc->header.w; i++)
        {
            for (int32_t j = clip_area->y2 + 1; j < buf_area->y2 + 1; j++)
            {
                int32_t index = (i + (j - buf_area->y1) * dsc->header.w) * (pix_size + 1);
                uint8_t *p = (uint8_t*)(dsc->data + index);
                *((lv_color_t*)p) = back;
                *(p + pix_size) = 0;
            }
        }
    }
}


#ifdef DRV_EPIC_NEW_API
bool lv_snapshot_fb_to_dsc(lv_img_dsc_t *dsc)
{
#if defined(LCD_FB_USING_TWO_COMPRESSED) || defined(LCD_FB_USING_ONE_COMPRESSED)
    lv_obj_t *screen = lv_scr_act();
    lv_snapshot_obj_to_dsc_scale(screen,&screen->coords,dsc);
    return true;
    //RT_ASSERT(0);
#endif

    uint32_t tick = rt_tick_get();
    lv_disp_t *refr_ori = _lv_refr_get_disp_refreshing();
    if (NULL == refr_ori)
        refr_ori = lv_disp_get_default();

    lv_coord_t hor = lv_disp_get_hor_res(refr_ori);
    lv_coord_t ver = lv_disp_get_ver_res(refr_ori);

    lv_draw_img_dsc_t img_dsc;
    lv_draw_img_dsc_init(&img_dsc);

    drv_epic_rotate_t rotated = drv_epic_get_rotation();
    if(DRV_EPIC_ROT_NONE != rotated)
    {
        int16_t angle = 0;
        switch(rotated)
        {
            case DRV_EPIC_ROT_90:
                angle = 90;
                break;
            case DRV_EPIC_ROT_180:
                angle = 180;
                break;
            case DRV_EPIC_ROT_270:
                angle = 270;
                break;
            default:
                RT_ASSERT(0); /* Not support */
        }
        
        img_dsc.pivot.x = (hor >> 1) - 1;
        img_dsc.pivot.y = (ver >> 1) - 1;
        img_dsc.angle = (360 - angle) * 10;
    }
    
    lv_disp_drv_t driver;
    lv_disp_drv_init(&driver);
    driver.hor_res = hor;
    driver.ver_res = ver;
    lv_disp_drv_use_generic_set_px_cb(&driver, dsc->header.cf);

    lv_img_dsc_t src_dsc;
    lv_img_cf_t src_cf = refr_ori->driver->draw_buf->cf;
    int32_t draw_size = lv_img_buf_get_img_size(hor, ver, refr_ori->driver->draw_buf->cf);
    void* draw_buf = get_disp_buf(draw_size);
    LV_ASSERT_NULL(draw_buf);
    src_dsc.data = draw_buf;
    src_dsc.data_size = draw_size;
    src_dsc.header.always_zero = 0;
    src_dsc.header.reserved = 0;
    src_dsc.header.w = hor;
    src_dsc.header.h = ver;
    src_dsc.header.cf = src_cf;

    lv_area_t img_area;
    img_area.x1 = 0;
    img_area.y1 = 0;
    img_area.x2 = hor - 1;
    img_area.y2 = ver - 1;

    lv_disp_t fake_disp;
    lv_memset_00(&fake_disp, sizeof(lv_disp_t));
    fake_disp.driver = &driver;

    lv_draw_ctx_t *draw_ctx = lv_mem_alloc(refr_ori->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if (draw_ctx == NULL) return LV_RES_INV;
    refr_ori->driver->draw_ctx_init(fake_disp.driver, draw_ctx);
    fake_disp.driver->draw_ctx = draw_ctx;
    draw_ctx->clip_area = &img_area;
    draw_ctx->buf_area = &img_area;
    draw_ctx->buf = (void*)dsc->data;
    driver.draw_ctx = draw_ctx;

    lv_area_t snapshot_area;
    snapshot_area.x1 = 0;
    snapshot_area.y1 = 0;
    snapshot_area.x2 = dsc->header.w - 1;
    snapshot_area.y2 = dsc->header.h - 1;
    
    _lv_refr_set_disp_to_draw_start(&fake_disp);
    
    lv_draw_img(draw_ctx, &img_dsc, &img_area, &src_dsc);

    _lv_refr_set_disp_to_draw_end(refr_ori, &snapshot_area);
    refr_ori->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);

    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return true;


}

bool lv_snapshot_obj_to_dsc_scale (lv_obj_t *obj, lv_area_t *mask, lv_img_dsc_t *dsc)
{
    uint32_t tick = rt_tick_get();
	lv_obj_update_layout(obj);
	/*Width and height determine snapshot image size.*/

    lv_area_t snapshot_area;
    snapshot_area.x1 = 0;
    snapshot_area.y1 = 0;
    snapshot_area.x2 = dsc->header.w - 1;
    snapshot_area.y2 = dsc->header.h - 1;
    lv_area_move(&snapshot_area, mask->x1, mask->y1);

    lv_disp_t* obj_disp = lv_obj_get_disp(obj);
    lv_disp_drv_t driver;
    lv_disp_drv_init(&driver);
    /*In lack of a better idea use the resolution of the object's display*/
    driver.hor_res = lv_disp_get_hor_res(obj_disp);
    driver.ver_res = lv_disp_get_ver_res(obj_disp);
    lv_disp_drv_use_generic_set_px_cb(&driver, dsc->header.cf);

    lv_disp_t fake_disp;
    lv_memset_00(&fake_disp, sizeof(lv_disp_t));
    fake_disp.driver = &driver;

    lv_draw_ctx_t *draw_ctx = lv_mem_alloc(obj_disp->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if (draw_ctx == NULL) return false;
    obj_disp->driver->draw_ctx_init(fake_disp.driver, draw_ctx);
    fake_disp.driver->draw_ctx = draw_ctx;
    draw_ctx->clip_area = mask;
    draw_ctx->buf_area = mask;
    draw_ctx->buf = (void*)dsc->data;
    driver.draw_ctx = draw_ctx;
    
    lv_disp_t *refr_ori = _lv_refr_get_disp_refreshing();
	_lv_refr_set_disp_to_draw_start(&fake_disp);

    lv_obj_redraw(draw_ctx, obj);

	_lv_refr_set_disp_to_draw_end(refr_ori, &snapshot_area);
    obj_disp->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);

    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return true;

}

bool lv_snapshot_obj_to_dsc (lv_obj_t *obj, lv_area_t *mask, lv_img_dsc_t *dsc)
{
    return lv_snapshot_obj_to_dsc_scale(obj,mask,dsc);
}
#else

bool lv_snapshot_fb_to_dsc(lv_img_dsc_t *dsc)
{
#if defined(LCD_FB_USING_TWO_COMPRESSED) || defined(LCD_FB_USING_ONE_COMPRESSED)
    lv_obj_t* screen = lv_scr_act();
    lv_snapshot_obj_to_dsc_scale(screen, &screen->coords, dsc);
    return true;
    //RT_ASSERT(0);
#endif
    uint32_t tick = rt_tick_get();
    lv_disp_t *refr_ori = _lv_refr_get_disp_refreshing();
    if (NULL == refr_ori)
        refr_ori = lv_disp_get_default();

    lv_coord_t hor = lv_disp_get_hor_res(refr_ori);
    lv_coord_t ver = lv_disp_get_ver_res(refr_ori);

    lv_coord_t target_w = dsc->header.w;
    lv_coord_t target_h = dsc->header.h;
    lv_coord_t zoom_w = target_w * LV_IMG_ZOOM_NONE / hor;
    lv_coord_t zoom_h = target_h * LV_IMG_ZOOM_NONE / ver;
    lv_coord_t target_zoom = LV_MIN(zoom_w, zoom_h);
    target_w = target_zoom * hor / LV_IMG_ZOOM_NONE;
    target_h = target_zoom * ver / LV_IMG_ZOOM_NONE;

    uint32_t target_size = lv_img_buf_get_img_size(target_w, target_h, dsc->header.cf);
    if (target_size > dsc->data_size)
        return false;

    lv_area_t clip_area;
    lv_area_t buf_area;
    lv_snapshot_dsc_init(dsc, target_w, target_h, &clip_area, &buf_area);

    lv_draw_img_dsc_t img_dsc;
    lv_draw_img_dsc_init(&img_dsc);
    img_dsc.zoom = target_zoom;

    lv_disp_drv_t driver;
    lv_disp_drv_init(&driver);
    driver.hor_res = target_w;
    driver.ver_res = target_h;
    lv_disp_drv_use_generic_set_px_cb(&driver, dsc->header.cf);

    lv_img_dsc_t src_dsc;
    lv_img_cf_t src_cf = refr_ori->driver->draw_buf->cf;
	int32_t draw_size = lv_img_buf_get_img_size(hor, ver, refr_ori->driver->draw_buf->cf);
    void* draw_buf = get_disp_buf(draw_size);
    LV_ASSERT_NULL(draw_buf);
#ifdef BSP_USING_PC_SIMULATOR
    src_cf = LV_IMG_CF_TRUE_COLOR;
    draw_buf = lv_snapshot_to_sim_buf(draw_buf);
#endif
    src_dsc.data = draw_buf;
    src_dsc.data_size = draw_size;
    src_dsc.header.always_zero = 0;
    src_dsc.header.reserved = 0;
    src_dsc.header.w = hor;
    src_dsc.header.h = ver;
	src_dsc.header.cf = src_cf;

    lv_area_t img_area;
    img_area.x1 = 0;
    img_area.y1 = 0;
    img_area.x2 = hor - 1;
    img_area.y2 = ver - 1;

    lv_disp_t fake_disp;
    lv_memset_00(&fake_disp, sizeof(lv_disp_t));
    fake_disp.driver = &driver;

    lv_draw_ctx_t *draw_ctx = lv_mem_alloc(refr_ori->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if (draw_ctx == NULL) return LV_RES_INV;
    refr_ori->driver->draw_ctx_init(fake_disp.driver, draw_ctx);
    fake_disp.driver->draw_ctx = draw_ctx;
    draw_ctx->clip_area = &clip_area;
    draw_ctx->buf_area = &buf_area;
    draw_ctx->buf = (void*)dsc->data;
    driver.draw_ctx = draw_ctx;

	_lv_refr_set_disp_to_draw_start(&fake_disp);
    lv_draw_img(draw_ctx, &img_dsc, &img_area, &src_dsc);

	_lv_refr_set_disp_to_draw_end(refr_ori, NULL);
    refr_ori->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);

    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return true;
}
bool lv_snapshot_obj_to_dsc(lv_obj_t *obj, lv_area_t *mask, lv_img_dsc_t *dsc)
{
    uint32_t tick = rt_tick_get();
    lv_area_t buf_area;
    buf_area.x1 = mask->x1;
    buf_area.y1 = mask->y1;
    buf_area.x2 = buf_area.x1 + dsc->header.w - 1;
    buf_area.y2 = buf_area.y1 + dsc->header.h - 1;
    lv_area_t clip_area;
    _lv_area_intersect(&clip_area, &buf_area, mask);

    lv_disp_t *obj_disp = lv_obj_get_disp(obj);
    lv_disp_drv_t driver;
    lv_disp_drv_init(&driver);
    /*In lack of a better idea use the resolution of the object's display*/
    driver.hor_res = lv_disp_get_hor_res(obj_disp);
    driver.ver_res = lv_disp_get_ver_res(obj_disp);
    lv_disp_drv_use_generic_set_px_cb(&driver, dsc->header.cf);

    lv_disp_t fake_disp;
    lv_memset_00(&fake_disp, sizeof(lv_disp_t));
    fake_disp.driver = &driver;

    lv_draw_ctx_t *draw_ctx = lv_mem_alloc(obj_disp->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if (draw_ctx == NULL) return LV_RES_INV;
    obj_disp->driver->draw_ctx_init(fake_disp.driver, draw_ctx);
    fake_disp.driver->draw_ctx = draw_ctx;
    draw_ctx->clip_area = &clip_area;
    draw_ctx->buf_area = &buf_area;
    draw_ctx->buf = (void*)dsc->data;
    driver.draw_ctx = draw_ctx;
    

    lv_disp_t *refr_ori = _lv_refr_get_disp_refreshing();
	_lv_refr_set_disp_to_draw_start(&fake_disp);

    lv_obj_redraw(draw_ctx, obj);

	_lv_refr_set_disp_to_draw_end(refr_ori, NULL);
    obj_disp->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);
    //LV_ASSERT(LV_IMG_CF_TRUE_COLOR_ALPHA !=  dsc->header.cf);

    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return true;
}


bool lv_snapshot_obj_to_dsc_scale(lv_obj_t *obj, lv_area_t *mask, lv_img_dsc_t *dsc)
{
    uint32_t tick = rt_tick_get();
	lv_obj_update_layout(obj);
	/*Width and height determine snapshot image size.*/
	lv_coord_t mask_w = lv_area_get_width(mask);
	lv_coord_t mask_h = lv_area_get_height(mask);
	int32_t size_per_line = lv_img_buf_get_img_size(mask_w, 1, dsc->header.cf);

    lv_coord_t target_w = dsc->header.w;
    lv_coord_t target_h = dsc->header.h;
    lv_coord_t target_zoom = target_w * LV_IMG_ZOOM_NONE / mask_w;
    target_w = target_zoom * mask_w / LV_IMG_ZOOM_NONE;
    target_h = target_zoom * mask_h / LV_IMG_ZOOM_NONE;
    target_h = LV_MIN(dsc->header.h, target_h);
    uint32_t target_size = lv_img_buf_get_img_size(target_w, target_h, dsc->header.cf);
    if (target_size > dsc->data_size)
        return false;

    lv_area_t clip_area;
    lv_area_t buf_area;
    lv_snapshot_dsc_init(dsc, target_w, target_h, &clip_area, &buf_area);
    lv_area_move(&clip_area, mask->x1, mask->y1);
    lv_area_move(&buf_area, mask->x1, mask->y1);

    lv_disp_t *refr_ori = _lv_refr_get_disp_refreshing();
    if (NULL == refr_ori)
        refr_ori = lv_disp_get_default();
    lv_disp_draw_buf_t *draw_buf = lv_disp_get_draw_buf(refr_ori);

    lv_img_dsc_t tmp_dsc;
    tmp_dsc.header.cf = dsc->header.cf;
    tmp_dsc.header.always_zero = 0;
    tmp_dsc.header.reserved = 0;
    tmp_dsc.data = draw_buf->buf_act;
    tmp_dsc.header.w = mask_w;
    tmp_dsc.header.h = ((draw_buf->size / size_per_line) >> 1) << 1;
    if (tmp_dsc.header.h > mask_h) tmp_dsc.header.h = mask_h;
    tmp_dsc.data_size = lv_img_buf_get_img_size(tmp_dsc.header.w, tmp_dsc.header.h, dsc->header.cf);

    lv_draw_img_dsc_t draw_img_dsc;
    lv_draw_img_dsc_init(&draw_img_dsc);
    draw_img_dsc.zoom = target_zoom;
    draw_img_dsc.antialias = false;

    lv_disp_drv_t driver;
    lv_disp_drv_init(&driver);
    driver.hor_res = mask_w;
    driver.ver_res = tmp_dsc.header.h;
    lv_disp_drv_use_generic_set_px_cb(&driver, dsc->header.cf);

    lv_disp_t fake_disp;
    lv_memset_00(&fake_disp, sizeof(lv_disp_t));
    fake_disp.driver = &driver;

    lv_draw_ctx_t *draw_ctx = lv_mem_alloc(refr_ori->driver->draw_ctx_size);
    LV_ASSERT_MALLOC(draw_ctx);
    if (draw_ctx == NULL) return false;
    refr_ori->driver->draw_ctx_init(fake_disp.driver, draw_ctx);
    fake_disp.driver->draw_ctx = draw_ctx;
    draw_ctx->clip_area = &clip_area;
    draw_ctx->buf_area = &buf_area;
    draw_ctx->buf = (void*)dsc->data;
    driver.draw_ctx = draw_ctx;

    lv_coord_t row_offset = 0;
    lv_area_t sub_area;
    sub_area.x1 = mask->x1;
    sub_area.x2 = mask->x2;
    if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);
    /*For situations with scaling, it is necessary to stack lines in the middle, 
     otherwise some parts may not be drawn, resulting in loopholes in the image*/
    for (row_offset = mask->y1; row_offset <= mask->y2; row_offset += (tmp_dsc.header.h - 2)) {
        sub_area.y1 = row_offset;
        sub_area.y2 = row_offset + tmp_dsc.header.h - 1;
        if (sub_area.y2 > mask->y2) sub_area.y2 = mask->y2;

        lv_snapshot_obj_to_dsc(obj, &sub_area, &tmp_dsc);

		_lv_refr_set_disp_to_draw_start(&fake_disp);
        draw_img_dsc.pivot.x = mask->x1 - sub_area.x1;
        draw_img_dsc.pivot.y = mask->y1 - sub_area.y1;
        lv_draw_img(draw_ctx, &draw_img_dsc, &sub_area, &tmp_dsc);
		_lv_refr_set_disp_to_draw_end(refr_ori, NULL);
    }
    refr_ori->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);

    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return true;
}
#endif

void lv_snapshot_img_delete_cb(lv_event_t *e)
{
    lv_obj_t *img = lv_event_get_current_target(e);
    lv_img_dsc_t *dsc = (lv_img_dsc_t *)lv_img_get_src(img);
    if (dsc->data)
        app_cache_free((void *)dsc->data);
    snapshot_free(dsc);
}

lv_img_dsc_t *lv_snapshot_create_dsc(lv_coord_t w, lv_coord_t h, lv_img_cf_t cf, uint16_t cache_type)
{
    uint32_t img_size = lv_img_buf_get_img_size(w, h, cf);
    lv_img_dsc_t *dsc = (lv_img_dsc_t *)snapshot_malloc(sizeof(lv_img_dsc_t));
    uint8_t *data = (uint8_t *)app_cache_alloc(img_size, cache_type);
    if (dsc)
    {
        dsc->header.cf = cf;
        dsc->header.always_zero = 0;
        dsc->header.reserved = 0;
        dsc->header.w = w;
        dsc->header.h = h;
        dsc->data = data;
        dsc->data_size = img_size;
    }
    return dsc;
}

/**
    The main function is to dump the designated area of obj onto the image,
    and the image buffer alloced in PSRAM will be auto free when the image is deleted
 *
 @param obj The obj The obj need to be dump
 @param mask The designated area of obj will be dump
 @param dsc The dump data will be save in it
 @return the image snapshoted of obj.
 */

lv_obj_t *lv_snapshot_obj_to_img(lv_obj_t *parent, lv_obj_t *obj, lv_area_t *area, lv_img_cf_t cf)
{
    uint32_t tick = rt_tick_get();
    lv_coord_t img_w = lv_area_get_width(area);
    lv_coord_t img_h = lv_area_get_height(area);
    lv_img_dsc_t *dsc = lv_snapshot_create_dsc(img_w, img_h, cf, CACHE_PSRAM);
    LV_ASSERT_NULL(dsc);

#if !defined(DRV_EPIC_NEW_API) || defined (BSP_USING_PC_SIMULATOR)
    if (LV_IMG_CF_TRUE_COLOR_ALPHA == cf)
        rt_memset((void*)dsc->data, 0, dsc->data_size);
#endif // BSP_USING_PC_SIMULATOR

    lv_snapshot_obj_to_dsc(obj, area, dsc);

    lv_obj_t *img = lv_img_create(parent);
    lv_img_set_src(img, dsc);
    lv_obj_add_event_cb(img, lv_snapshot_img_delete_cb, LV_EVENT_DELETE, NULL);

    if (snapshot_log_en)
        rt_kprintf("%s: tick %d (ms) \n", __func__, rt_tick_get() - tick);

    return img;
}

static rt_err_t snapshot_log(int argc, char** argv)
{
    snapshot_log_en = (snapshot_log_en + 1) & 0x01;
    rt_kprintf("%s: snapshot_log_en %d \n", __func__, snapshot_log_en);

    return 0;
}

MSH_CMD_EXPORT(snapshot_log, snapshot log);

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif /*LV_USE_SNAPSHOT*/
