/*********************
 *      INCLUDES
 *********************/
#ifndef _APP_BT_H
#define _APP_BT_H
#include <rtdevice.h>

/**
* @brief make a phone call.
* @param[in] *number phone number string
* @param[in] size    phone number size
* @return function execution result @see bt_err_t
*/


bt_err_t app_bt_search_device_start(void);


/**
* @brief stop searching for bluetooth devices.
* @return function execution result @see bt_err_t
*/
bt_err_t app_bt_search_device_stop(void);

/**
* @brief start connect peer bluetooth device.
*
* @param[in] *addr peer bluetooth device mac addr
* @param[in] size    mac addr size
* @return function execution result @see bt_err_t
* @see BT_EVENT_CONNECT_COMPLETE BT device connect success
*/
bt_err_t app_bt_connect_start(const bt_mac_t *addr);


/**
* @brief disconnect the bluetooth device.
* @return function execution result @see bt_err_t
* @see BT_EVENT_DISCONNECT BT device disconnect success
*/
bt_err_t app_bt_disconnect(void);


/**
* @brief query bluetooth state.
* @param[out] *state the bluetooth state
* @return function execution result @see bt_err_t
*/
bt_err_t app_bt_query_state(bt_state_t *state);


/**
* @brief query bluetooth state nonblock.
* @return function execution result @see bt_err_t
* @note BT_EVENT_STATE
*/
bt_err_t app_bt_nonblock_query_state(void);


/**
* @brief  register bt event callback.
* @param[in] cb  bt event callback @see bt_notify_event_t
* @return function execution result @see bt_err_t
*/
bt_err_t app_bt_register_notify(bt_notify_cb cb);

/**
* @brief  unregister bt event callback.
* @param[in] cb  bt event callback @see bt_notify_event_t
* @return function execution result @see bt_err_t
*/
bt_err_t app_bt_unregister_notify(bt_notify_cb cb);


/**
* @brief  open bt device.
* @return function execution result @see bt_err_t
*/
bt_err_t app_open_bt(void);


/**
* @brief  close bt device.
* @return function execution result @see bt_err_t
*/
bt_err_t app_close_bt(void);



#endif /* _APP_BT_H */


