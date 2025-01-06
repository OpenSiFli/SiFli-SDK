/**
 * @file    app_clock_main.h
 * @brief  clock app framework for all clock UI
 *
 * \n
 * @details
 * entrance of all clock, and it loads inbuilt clocks and customized clocks
 *
 * \n
 * @version
 * @author  folks
 * @date    2020-2-5
 *
 * @history
 *
 */
#ifndef __APP_CLOCK_MAIN_H__
#define __APP_CLOCK_MAIN_H__
#include <rtthread.h>
#include "lvgl.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"

typedef struct
{
    uint8_t h;
    uint8_t m;
    uint8_t s;
    uint16_t ms;
} app_clock_time_t;


typedef rt_int32_t (*app_clock_init_cb_t)(lv_obj_t *);
typedef rt_int32_t (*app_clock_func_ptr_t)(void);


typedef struct
{
    app_clock_init_cb_t   init;     //!< clock UI init cbk func
    app_clock_func_ptr_t  pause;    //!< clock UI pause cbk func
    app_clock_func_ptr_t resume;    //!< clock UI resume cbk func
    //app_clock_func_ptr_t click;   use sys layer to filte other event
    app_clock_func_ptr_t deinit;    //!< clock UI destroy cbk func
} app_clock_ops_t;



/****                     API for single clcok              ****/

/**
 * get current time  -   API for all clocks UI
 *
 * \n
 *
 * @param pt  pointer to return time var
 * \n
 * @see
 */
void app_clock_main_get_current_time(app_clock_time_t *pt);

/**
 * Reset current time  -   API for all clocks UI
 *
 */
void app_clock_reset_time(void);


/**
 * Get clock state change context, currently is clock face ID.
 *
 */
char *app_clock_change_context(void);


/**
 * regist built-in clock or customized clock
 * \n
 *
 * @param operations
 * @param id
 * @return
 * \n
 * @see
 */
int32_t app_clock_register(const char *id, const app_clock_ops_t *operations);


#if 0
    /**
    * duplicate an image to SRAM to improve drawn performance
    * \n
    *
    * @return
    * @param copy
    * \n
    * @see
    */
    lv_img_dsc_t *app_clock_img_cache_malloc(const void *copy);

    void app_clock_img_cache_free(lv_img_dsc_t *p_img);
#endif

#endif /*__APP_CLOCK_MAIN_H_*/
