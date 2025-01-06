#ifndef __GUI_APP_INT_H__
#define __GUI_APP_INT_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "lvgl.h"
#include "gui_app_fwk.h"
#include "intent_int.h"



#ifdef DL_APP_SUPPORT
    #include "dlfcn.h"
#endif
#ifdef RT_USING_DFS
    #include <dfs_posix.h>
#endif
#include "lv_ext_resource_manager.h"
#ifdef LV_EXT_RES_STANDALONE
    #include "lv_ext_resource_manager_dl_impl.h"
    #include "mod_installer.h"
#endif /* LV_EXT_RES_STANDALONE */



/*---------------------------------app_schedule.h-------------------------------------------------------------------------*/

#define SUBPAGE_MAX_LEN 16  //page name maximum length
#define GUI_APP_MSG_MAX_LEN sizeof(gui_app_msg_t)

#define GUI_APP_CMD_MAX_LEN INTENT_MAX_LEN
#define GUI_APP_CMD_ARG_MAX 8

typedef uint32_t *screen_t;
typedef uint32_t *task_t;
typedef void (*task_handler_t)(task_t);

/*
   Draw a screen to smaller lv_img without screen sized buffer.

   For examle,
    #define  SCALE_TRANS_ANIM_USE_DIVIDED_FB   4
   means use 1/4 screen sized framebuffer as temporary buffer to zoom screen to specified lv_img,
   after that temporary buffer is useless.
*/
//#define SCALE_TRANS_ANIM_USE_DIVIDED_FB  4

typedef enum
{
    page_st_none = 0,
    page_st_created,
    page_st_started,
    page_st_resumed,
    page_st_paused,
    page_st_stoped,
} page_state_enum;

typedef uint32_t subpage_handle;


typedef struct
{
    //regist_app_desc_t *app_info;
    char id[GUI_APP_ID_MAX_LEN];   //!< app id
    _intent param;                 //!< app run parameters
    void *app_data;                //!< app data
    gui_app_entry_func_ptr_t entry_f; //!< app entry function

    rt_list_t page_list;

    //bool press_back;  //!< resume app by press back button
    //uint8_t *gui_buffer;
    //time_t start_time;
    //time_t pause_time; //for trash collection  OR  move to a suspend list??
    //gui_app_state_type_t state;
    //uint8_t priority;
    //lv_task_id;

    rt_list_t node;
    uint8_t state;
    uint8_t target_state;
    uint8_t flag;
    uint32_t tick_cnt; //!< app running ticks(including subpage's ticks)
} gui_runing_app_t;

typedef struct _subpage_node
{
    gui_runing_app_t *parent;
    char name[SUBPAGE_MAX_LEN];
    screen_t scr;
    page_state_enum state;
    page_state_enum target_state;

    gui_app_trans_anim_group_t a_group;

    gui_page_msg_cb_t  msg_handler;
    void *user_data;
    rt_list_t node;
    uint32_t tick_cnt; //!< running ticks
} subpage_node_t;

typedef enum
{
    // Message used by APP framework
    /*
        Start an app by id
    */
    GUI_APP_MSG_RUN_APP = GUI_APP_MSG_USER_END + 1,
    /*
        Start an app by schedule
    */
    GUI_APP_MSG_RUN_APP_I,
    /*
        Exit an app by id, and all subpages belong this app will be exit.
    */
    GUI_APP_MSG_EXIT_APP,
    /*
        Goback current scheduling/active app's subpage,
            and exit app if goback last subgpage.
    */
    GUI_APP_MSG_GOBACK,
    /*
        Create a subpage on app and active it
    */
    GUI_APP_MSG_OPEN_PAGE,
    /*
        Remove specified subpage on app
    */
    GUI_APP_MSG_RM_PAGE,
    /*
        Goback current subpage util specified subpage active
    */
    GUI_APP_MSG_GOBACK_TO_PAGE,
    /*
        Refresh specified subpage on active app
    */
    GUI_APP_MSG_REFRESH_PAGE,
    /*
        Exit background apps
    */
    GUI_APP_MSG_CLEANUP_BACKGROUND,
    /*
        Exit all runing apps
    */
    GUI_APP_MSG_CLEANUP,
    GUI_APP_MSG_MANUAL_GOBACK_ANIM,
    /*
        Pause current actived subpage and suspend scheduler
        and will NOT execute next GUI_APP_MSG_xxx util
        resume scheduler by 'app_scheduler_resume'
    */
    GUI_APP_MSG_SUSPEND_SCHEDULER,
    /*
        Resume last subpage after 'app_scheduler_resume'
    */
    GUI_APP_MSG_RESUME_SCHEDULER,
    /*
        Abort current transform animation if it is present.
    */
    GUI_APP_MSG_ABORT_TRANS_ANIM,
} gui_app_frmmsg_type_t;


typedef struct
{
    char name[SUBPAGE_MAX_LEN];
    gui_page_msg_cb_t  msg_handler;
    void *user_data;
} page_msg_t;

typedef struct
{
    gui_runing_app_t    *handler;  //!< app handler
    uint8_t             msg_id;
    uint32_t            tick;
    union
    {
        char cmd[GUI_APP_CMD_MAX_LEN];   //!< obsolete: app execution command(include arguments)
        _intent intnt;
        page_msg_t page;                 //!< subpage operate param
    } content;
} gui_app_msg_t;


typedef struct
{
    uint32_t  user_data;
    uint32_t  entry_func;
} app_entity_info;

enum
{
    APP_SCHE_LOG_LEVEL_DEBUG  = 0,
    APP_SCHE_LOG_LEVEL_INFO   = 1,
    APP_SCHE_LOG_LEVEL_ERROR  = 2,
    APP_SCHE_LOG_LEVEL_ASSERT = 3
};

typedef enum
{
    APP_SCHE_IDLE,     //!< Nothing to do
    APP_SCHE_SUSPEND,  //!< Suspend by something, like playing animation etc.
} app_sche_state;

typedef void (*app_load_func)(const char *app_id, app_entity_info *p_app_info);
typedef void (*app_destory_func)(const char *app_id, const app_entity_info *p_app_info);
typedef void (*app_sche_debug_func)(uint8_t level, const char *fmt, ...);
typedef void (*app_sche_hook_func)(app_sche_state state);
typedef void *(*app_sche_malloc_func)(rt_size_t size);
typedef void (*app_sche_free_func)(void *p_mem);
typedef void (*app_sche_subpage_func)(gui_page_msg_cb_t func, gui_app_msg_type_t msg_id, const char *app_id, const char *subpage_id);


/**
 * @brief Regist app schedule cbk functions
 * @param ld_func        - Load app by app_id
 * @param app_dstry_func - Destory app function
 * @param debug_func - [Optinal]Print log, assert, error
 * @param idle_hook  - Call while app_schedule_task idle
 * @param malloc_func -
 * @param free_func -
 * @param subpage_func - [Optional] replace subpage execute
 * @return 0 if sucess
 */
uint32_t app_schedule_init(app_load_func ld_func,
                           app_destory_func app_dstry_func,
                           app_sche_debug_func debug_func,
                           app_sche_hook_func idle_hook,
                           app_sche_malloc_func malloc_func,
                           app_sche_free_func free_func,
                           app_sche_subpage_func subpage_func);
void app_schedule_max_running_apps(uint32_t v);
uint32_t app_schedule_get_max_running_apps(void);
void app_schedule_task(rt_mailbox_t msg_mbx);
gui_runing_app_t *app_schedule_get_active(void);
gui_runing_app_t *app_schedule_get_this(void);
gui_runing_app_t *app_schedule_is_app_running(const char *id);
subpage_node_t *app_schedule_get_this_page(void);
uint32_t app_schedule_get_running_apps(void);
subpage_node_t *app_schedule_get_page_in_app(gui_runing_app_t *app, const char *page_id);
rt_err_t list_schedule_app(void);
void app_schedule_set_idle_hook(app_sche_hook_func idle_hook);
void app_schedule_enable_trans_anim(bool en);
page_state_enum app_schedule_get_page_state(const char *app_id, const char *page_id);
void app_scheduler_resume(void);
bool app_scheduler_is_suspend(void);
/*
 1.Close all other app while open/resume main app.
 2.Open default app while the last app being closed.
    Default id is 'Main'.
*/
uint32_t app_schedule_change_main_app_id(const char *id);
/*
 Always play goback animation while open first subpage of desktop app. Default id is 'Main'.
*/
uint32_t app_schedule_change_desktop_app_id(const char *id);
char *fwk_msg_to_name(gui_app_frmmsg_type_t msg);
app_sche_state app_schedule_state_get(void);
void app_schedule_destory_suspend_apps(uint32_t v);
void app_scheduler_print_perf_tick(uint8_t en);

/*---------------------------------app_schedule.h-------------------------------------------------------------------------*/


#include "app_schedule_port.h"


/**
 * get actived app handler
 * \n
 *
 * @return
 * \n
 * @see
 */
gui_runing_app_t *gui_app_get_actived(void);

/**
 * get actived page handler
 * \n
 *
 * @return
 * \n
 * @see
 */
subpage_node_t *gui_app_page_get_actived(void);


/**
 ***********************  Dynamic load app ************************************************
 */

/**
 *  @brief Open dl-app by it's command name
 *  @param[in] id Identification of application
 *  @param[in] flags flags for dlopen, currently unused.
 *  @retval module handle.
 */
void  *gui_app_dlopen(const char *id, int flags);


/**
 *  @brief Close dl-app
 *  @param[in] handle Module handle
 */
void gui_app_dlclose(void *handle);


/**
 *  @brief Register exist dl-app to system
 *  @param[in] reg_file - reg_file should saved in root dicrectory of dl-app
 *  @retval RT_OK if successful, else other value
 */
rt_err_t gui_dl_app_register(const char *reg_file);



char *app_trans_get_buf_a(void);
char *app_trans_get_buf_b(void);

typedef enum
{
    TRANS_RES_FINISHED, //Trans animation finished
    TRANS_RES_ABORTED,  //Trans animation aborted
} TransResult_T;

typedef void (*app_trans_anim_xcb_t)(TransResult_T res);
typedef void (*gui_anim_exe_cb)(gui_anim_value_t v);
typedef void (*gui_anim_free_run_cb)(TransResult_T res);

void app_trans_animation_init(void);

rt_err_t app_trans_animation_setup(const gui_app_trans_anim_group_t *g_enter,
                                   const gui_app_trans_anim_group_t *g_exit,
                                   const screen_t enter_scr,
                                   const screen_t exit_scr,
                                   app_trans_anim_xcb_t cbk, bool is_back, bool is_manual_anim);

static void app_trans_anim_finish_callback(TransResult_T res);


rt_err_t app_trans_anim_abort(void);
rt_err_t app_trans_animation_reset(void);

typedef void (*app_trans_ex_cb_t)(void);
void app_trans_end_cb_register(app_trans_ex_cb_t callback);


/**
 * @brief Enable/Disbale input event from ui framework.
 * @param enable -
 */
void gui_app_enable_input_device(bool enable);

void app_trans_anim_init_cfg(gui_app_trans_anim_t *cfg, gui_app_trans_anim_type_t type);



gui_anim_obj_t app_trans_animation_obj_create(const screen_t scr, bool is_cur_screen, bool b_scale, int buf_index);
void app_trans_animation_obj_destroy(gui_anim_obj_t obj);
void app_trans_animation_obj_move_foreground(gui_anim_obj_t obj);
void app_trans_anim_set_opa_scale(gui_anim_obj_t var, gui_anim_value_t opa_scale);
void app_trans_anim_set_zoom(gui_anim_obj_t var, gui_anim_value_t zoom);
void app_trans_anim_set_x(gui_anim_obj_t var, gui_anim_value_t x);
void app_trans_anim_set_pivot(gui_anim_obj_t var, const gui_point_t *p);
rt_err_t app_trans_anim_free_run_start(gui_anim_value_t start,
                                       gui_anim_value_t end,
                                       uint32_t duration,
                                       gui_anim_exe_cb anim_process,
                                       gui_anim_free_run_cb done_cb);
rt_err_t app_trans_anim_free_run_clean(void);





#endif  /* __GUI_APP_UTILS_H__ */
