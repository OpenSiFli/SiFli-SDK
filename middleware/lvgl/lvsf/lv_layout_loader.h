#ifndef _LV_LAYOUT_LOADER_H_
#define _LV_LAYOUT_LOADER_H_

#include "rtconfig.h"
#ifdef SOLUTION_WATCH
    #include "app_clock_comm.h"
#else
    #include "app_clock_main.h"
#endif

/*********************
 *      INCLUDES
 *********************/

#ifdef __cplusplus
extern "C" {
#endif


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
{
    const char     *id_str;
    const uint8_t  *data;
    const uint32_t  data_size;
} ui_layout_desc_t;


/**********************
 * GLOBAL PROTOTYPES
 **********************/
lv_res_t lv_obj_resume(lv_obj_t *obj);
lv_res_t lv_obj_pause(lv_obj_t *obj);


void *lv_load_layout(lv_obj_t *parent, const ui_layout_desc_t *layout);
void lv_free_layout(void *ui);
lv_obj_t *lv_find_obj_in_layout(void *ui, const char *name);


/**********************
 *      MACROS
 **********************/


#define APP_WATCHFACE_LAYOUT_SECTION_NAME   app_watchface_layout_db

extern const app_clock_ops_t g_app_watchface_layout_ops;

#define APP_BUILTIN_WATCHFACE_LAYOUT_REGISTER(priority, id, name, thumb_img, _data)        \
    APP_BUILTIN_CLOCK_REGISTER(priority, id, name, &g_app_watchface_layout_ops, thumb_img);\
    APP_WATCHFACE_LAYOUT_REGISTER(id, _data)

#define APP_WATCHFACE_LAYOUT_REGISTER(id, _data)      \
    SECTION_ITEM_REGISTER(APP_WATCHFACE_LAYOUT_SECTION_NAME, static const ui_layout_desc_t CONCAT_2(id, _layout_var)) =  \
    {                                    \
        .id_str     = STRINGIFY(id),     \
        .data       = _data,             \
        .data_size  = sizeof(_data),     \
    }


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*_LV_LAYOUT_LOADER_H_*/

