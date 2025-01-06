#ifndef L2CC_TASK_BT_H
#define L2CC_TASK_BT_H

#if (BLE_L2CC)
#include "l2cc_task.h"
#include "l2cc.h"
#include "l2cc_int.h"

#ifndef U16
    #define U16 uint16_t
#endif


#define GATT_OVER_BREDR_INDEX (BLE_CONNECTION_MAX - 1)

typedef struct
{
    uint8_t conidx;
    uint8_t is_init;
    uint16_t cid;
} l2cc_task_bt_env_t;

uint8_t is_gatt_over_bredr_conidx(uint8_t conidx);
void bt_att_data_send(void const *param_ptr);
#endif
#endif