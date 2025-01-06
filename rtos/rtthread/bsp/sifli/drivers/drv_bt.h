/*********************
 *      INCLUDES
 *********************/
#ifndef _DRV_BT_H
#define _DRV_BT_H
#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
int rt_hw_bt_init(const struct rt_bt_ops *ops, rt_uint16_t open_flag);
#endif /* _DRV_BT_H */


