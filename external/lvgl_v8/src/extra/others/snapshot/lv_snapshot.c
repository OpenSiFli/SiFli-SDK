/**
 * @file lv_snapshot.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_snapshot.h"
#if LV_USE_SNAPSHOT

#include <stdbool.h>
#include "../../../core/lv_disp.h"
#include "../../../core/lv_refr.h"
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
    _lv_refr_set_disp_refreshing(&fake_disp);

    lv_obj_redraw(draw_ctx, obj);

    _lv_refr_set_disp_refreshing(refr_ori);
    obj_disp->driver->draw_ctx_deinit(fake_disp.driver, draw_ctx);
    lv_mem_free(draw_ctx);

    dsc->data = buf;
    dsc->header.w = w;
    dsc->header.h = h;
    dsc->header.cf = cf;
    dsc->data_size = lv_img_buf_get_img_size(w, h, cf);
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
        _lv_refr_set_disp_refreshing(&fake_disp);
        
       
        lv_draw_img(driver.draw_ctx, &img_draw_dsc, &clip_area, &img_screen);
        if (driver.draw_ctx->wait_for_finish) driver.draw_ctx->wait_for_finish(driver.draw_ctx);
        _lv_refr_set_disp_refreshing(ori_disp);

        deinit_fake_disp(ori_disp, &fake_disp);
    }

    return LV_RES_OK;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif /*LV_USE_SNAPSHOT*/
