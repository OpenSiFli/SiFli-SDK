#ifndef _BT_DEVICE_FSM_H_
#define _BT_DEVICE_FSM_H_

#include "bt_device.h"

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

int bt_connect_fsm_init(void);
int bt_acl_fsm_init(void);
int bt_media_fsm_init(void);
int bt_call_fsm_init(void);

int bt_connect_fsm_handle(rt_bt_device_t *dev, int event_type, void *args);
int bt_acl_fsm_handle(rt_bt_device_t *dev, int event_type, void *args);
int bt_device_fsm_handle(rt_bt_device_t *dev, int event_type, void *args);
int bt_media_fsm_handle(rt_bt_device_t *dev, int event_type, void *args);
int bt_call_fsm_handle(rt_bt_device_t *dev, uint8_t index, int event_type, void *args);
uint8_t bt_call_event_hdl(rt_bt_device_t *dev, uint32_t event, void *args);
char *bt_call_state_to_name(bt_call_state_t state);

#ifdef __cplusplus
}
#endif



#endif /* _BT_FSM_H_ */

