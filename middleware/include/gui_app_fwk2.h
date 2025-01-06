#ifndef GUI_APP_FWK2_H
#define GUI_APP_FWK2_H

#include "intent.h"

/**
 ****************************************************************************************
* @addtogroup gui_app_fwk GUI Application Framework
* @ingroup middleware
* @brief Sifli GUI application framework, provide basic functions for user to start/stop/pause/resume
*  application. It also provide navigation among applications and transition of GUI.
* @{

* @defgroup gui_app_function_group_1  GUI APP Framework exported functions
* @{
*/

/************************ Application framework *******************************************************/
/**
    @brief Page state notification messages

    Messages be send to page's msg handler by firmware while page's state changed
*/
typedef enum
{
    /**
        Page was started.<br/>
        Recieve this msg only once after the page created.
    */
    GUI_APP_MSG_ONSTART = 0,


    /**
        Page was bring to foreground.<br/>
        The page's state can be paused or started before.
    */
    GUI_APP_MSG_ONRESUME,


    /**
        Page was bring to background.<br/>
        The page's state must be resumed when recieve this message.
    */
    GUI_APP_MSG_ONPAUSE,


    /**
        Page was being destory.<br/>
        The page's state must be paused when recieve this message, and will recieve once.
    */
    GUI_APP_MSG_ONSTOP,

    GUI_APP_MSG_USER_END,               //!< last user application message
} gui_app_msg_type_t;


/**
    @brief Initialize application framework.
 */
void gui_app_init(void);

/**
    @brief run an app
    @param[in] cmd application command - format:[app_id] [param0] [param1] [param2] ...
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_run(const char *cmd);

/**
    @brief exit specified app by it's id
    @param[in] id The id of app
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_exit(const char *id);

/**
    @brief exit current app
 */
void gui_app_self_exit(void);

/**
    @brief     check whether the app is active(or running)
    @param[in] id Identification of application
    @retval    FALSE - if app is paused, or not even started, TRUE if app is active.
 */
bool gui_app_is_actived(char *id);

/**
    @brief     check whether the subpage is present in cur app
    @param[in] id Identification of subpage
    @retval    FALSE - if subpage is not started, TRUE if subpage is active, paused, or resumed.
 */
bool gui_app_is_page_present(char *id);

/**
    @brief Remove a subgpage from this app
    @param[in] pg_id Subpage id (should be unique in app )
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
void gui_app_remove_page(const char *pg_id);

/**
    @brief Refresh subgpage(pause then resume)
    @param[in] pg_id Subpage id (should be unique in app )
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
void gui_app_refr_page(const char *pg_id);

/**
    @brief Goback to specified subpage in this app.
    @param[in] pg_id Subpage id (should be unique in app )
    @retval RT_EOK if pg_id is existent, otherwise return error number < 0.
 */
int gui_app_goback_to_page(const char *pg_id);

/**
    @brief Destory current page, and destory app if it's having no page.
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_goback(void);

/**
    @brief Goback current page manual
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_manual_goback_anim(void);

/**
    @brief Close all app
    @retval void
 */
void gui_app_cleanup(void);

/**
    @brief Close all background app
    @retval void
 */
void gui_app_cleanup_bg_apps(void);

/**
    @brief Close all app synchronously, must been call while all gui app task is NOT running.
    @retval void
 */
void gui_app_cleanup_now(void);

/**
    @brief Run app immediately, no animation.
    @param[in] cmd application command - format:[app_id] [param0] [param1] [param2] ...
    @retval void
 */
void gui_app_run_now(const char *cmd);

/**
    @brief Check whether all app has been closed
    @retval true: all app is closed, false: not closed yet
 */
bool gui_app_is_all_closed(void);


/**
 * @brief Get current actived app's intent
 * @return Current active app's intent
 */
intent_t gui_app_get_intent(void);

/**
 * @brief Get subpage's user data
 * @param pg_id - Subpage id (should be unique in app )
 * @return NULL if not found pg_id
 */
void *gui_app_get_page_userdata(const char *pg_id);

/**
 * @brief Set subpage's user data
 * @param pg_id - Subpage id (should be unique in app )
 * @param usr_data - user data
 * @return RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_set_page_userdata(const char *pg_id, void *usr_data);


/**
 * @brief Get current subpage's user data
 * @return NULL if no data
 */
void *gui_app_this_page_userdata(void);


/**
* @brief Get all runing apps numbers
* \n
*
* @return The number of running apps
* \n
* @see
*/
uint32_t gui_app_get_running_apps(void);

/**
* @brief Get current clock parent
* \n
*
* @return clock parent object
* \n
* @see
*/
lv_obj_t *gui_app_get_clock_parent();

/**
* @brief Execute gui app schedule immediately util finish all job.
* \n
*
* @return none
* \n
* @see
*/
void gui_app_exec_now(void);


/**
 * @brief suspend gui_app framework asynchronized,
 *        append 'gui_app_exec_now' after if you want suspend synchronized.
 * @return RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_fwk_suspend(void);

/**
 * @brief resume gui_app framework asynchronized
 * @return RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_fwk_resume(void);


/**
  * @} gui_app_function_group_1
  */

/**
  * @} gui_app_fwk
  */

#endif

