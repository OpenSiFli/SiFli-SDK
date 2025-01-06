/**
 * @file lvsf_utils.c
 *
 *
 *
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"

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
char *lv_event_to_name(int event)
{
#define LV_EVENT_TO_NAME_CASE(e) case e: return #e
    switch (event)
    {
        /** Input device events*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_PRESSED);             /**< The object has been pressed*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_PRESSING);            /**< The object is being pressed (called continuously while pressing)*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_PRESS_LOST);          /**< The object is still being pressed but slid cursor/finger off of the object */
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SHORT_CLICKED);       /**< The object was pressed for a short period of time, then released it. Not called if scrolled.*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_LONG_PRESSED);        /**< Object has been pressed for at least `long_press_time`.  Not called if scrolled.*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_LONG_PRESSED_REPEAT); /**< Called after `long_press_time` in every `long_press_repeat_time` ms.  Not called if scrolled.*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_CLICKED);             /**< Called on release if not scrolled (regardless to long press)*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_RELEASED);            /**< Called in every cases when the object has been released*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCROLL_BEGIN);        /**< Scrolling begins*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCROLL_END);          /**< Scrolling ends*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCROLL);              /**< Scrolling*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_GESTURE);             /**< A gesture is detected. Get the gesture with `lv_indev_get_gesture_dir(lv_indev_get_act());` */
        LV_EVENT_TO_NAME_CASE(LV_EVENT_KEY);                 /**< A key is sent to the object. Get the key with `lv_indev_get_key(lv_indev_get_act());`*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_FOCUSED);             /**< The object is focused*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DEFOCUSED);           /**< The object is defocused*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_LEAVE);               /**< The object is defocused but still selected*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_HIT_TEST);            /**< Perform advanced hit-testing*/

        /** Drawing events*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_COVER_CHECK);        /**< Check if the object fully covers an area. The event parameter is `lv_cover_check_info_t *`.*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_REFR_EXT_DRAW_SIZE); /**< Get the required extra draw area around the object (e.g. for shadow). The event parameter is `lv_coord_t *` to store the size.*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_MAIN_BEGIN);    /**< Starting the main drawing phase*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_MAIN);          /**< Perform the main drawing*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_MAIN_END);      /**< Finishing the main drawing phase*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_POST_BEGIN);    /**< Starting the post draw phase (when all children are drawn)*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_POST);          /**< Perform the post draw phase (when all children are drawn)*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_POST_END);      /**< Finishing the post draw phase (when all children are drawn)*/
#ifdef DISABLE_LVGL_V9
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_PART_BEGIN);    /**< Starting to draw a part. The event parameter is `lv_obj_draw_dsc_t *`. */
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DRAW_PART_END);      /**< Finishing to draw a part. The event parameter is `lv_obj_draw_dsc_t *`. */
#endif
        /** Special events*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_VALUE_CHANGED);       /**< The object's value has changed (i.e. slider moved)*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_INSERT);              /**< A text is inserted to the object. The event data is `char *` being inserted.*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_REFRESH);             /**< Notify the object to refresh something on it (for the user)*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_READY);               /**< A process has finished*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_CANCEL);              /**< A process has been cancelled */

        /** Other events*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_DELETE);              /**< Object is being deleted*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_CHILD_CHANGED);       /**< Child was removed, added, or its size, position were changed */
        LV_EVENT_TO_NAME_CASE(LV_EVENT_CHILD_CREATED);       /**< Child was created, always bubbles up to all parents*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_CHILD_DELETED);       /**< Child was deleted, always bubbles up to all parents*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCREEN_UNLOAD_START); /**< A screen unload started, fired immediately when scr_load is called*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCREEN_LOAD_START);   /**< A screen load started, fired when the screen change delay is expired*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCREEN_LOADED);       /**< A screen was loaded*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SCREEN_UNLOADED);     /**< A screen was unloaded*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_SIZE_CHANGED);        /**< Object coordinates/size have changed*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_STYLE_CHANGED);       /**< Object's style has changed*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_LAYOUT_CHANGED);      /**< The children position has changed due to a layout recalculation*/
        LV_EVENT_TO_NAME_CASE(LV_EVENT_GET_SELF_SIZE);       /**< Get the internal size of a widget*/

    default:
        return "UNKNOW";
    }

}

#ifndef _MSC_VER
#include "drv_ext_dma.h"

static rt_sem_t copy_sema;

static void dma_done_cb(void)
{
    rt_err_t err;

    err = rt_sem_release(copy_sema);
    RT_ASSERT(RT_EOK == err);
}

static void dma_err_cb(void)
{
    RT_ASSERT(0);
}


void _lv_copy_vdb(uint8_t *buf_act, uint8_t *buf_ina, uint32_t size)
{
    rt_err_t err;

    if (copy_sema == NULL)
        copy_sema = rt_sem_create("lv_copy", 0, RT_IPC_FLAG_FIFO);

    /*start ext dma*/
    EXT_DMA_Config(1, 1);

    EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, dma_done_cb);
    EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, dma_err_cb);
    err = EXT_DMA_START_ASYNC((uint32_t)buf_ina, (uint32_t)buf_act, (size + 3) >> 2);
    RT_ASSERT(RT_EOK == err);

    err = rt_sem_take(copy_sema, 1000);
    RT_ASSERT(RT_EOK == err);
}
#endif



