#ifndef _BT_PAN_MP3_H_
#define _BT_PAN_MP3_H_
#include "bts2_app_inc.h"

typedef struct
{
    BOOL bt_connected;
    bt_notify_device_mac_t bd_addr;
    rt_timer_t pan_connect_timer;
} bt_app_t;

void exist_sniff_mode(void);

#endif