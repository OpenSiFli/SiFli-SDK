/*********************
 *      INCLUDES
 *********************/
#ifndef _BT_FR508X_CONTROL_H
#define _BT_FR508X_CONTROL_H
bt_err_t fr508x_control(struct rt_bt_device *gps_handle, int cmd, void *args);
void fr508x_set_cmd_table(at_client_t client);
#endif /* _BT_FR508X_CONTROL_H */
