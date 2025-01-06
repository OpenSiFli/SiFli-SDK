/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-14     tyx          the first version
 */

#ifndef __BT_PROT_H__
#define __BT_PROT_H__

#include "bts2_app_pan.h"



#ifdef __cplusplus
extern "C" {
#endif

#ifndef RT_BT_PROT_NAME_LEN
#define RT_BT_PROT_NAME_LEN  (8)
#endif



#define RT_BT_PROT_LWIP  ("lwip")

typedef enum
{
    RT_BT_PROT_EVT_INIT_DONE = 0,
    RT_BT_PROT_EVT_CONNECT,
    RT_BT_PROT_EVT_DISCONNECT,
    RT_BT_PROT_EVT_MAX,
} rt_bt_prot_event_t;


struct rt_bt_prot;

struct rt_bt_prot_ops
{
    rt_err_t (*prot_recv)(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
    struct rt_bt_prot *(*dev_reg_callback)(struct rt_bt_prot *prot, struct rt_bt_pan_instance *bt_instance);
    void (*dev_unreg_callback)(struct rt_bt_prot *prot, struct rt_bt_pan_instance *bt_instance);
};


struct rt_bt_prot
{
    char name[RT_BT_PROT_NAME_LEN];
    const struct rt_bt_prot_ops *ops;
};


typedef void (*rt_bt_prot_event_handler)(struct rt_bt_prot *port, struct rt_bt_pan_instance *bt_instance, int event);

static void rt_bt_prot_event_handle(struct rt_bt_pan_instance *bt_instance, rt_bt_instance_event_t event);
rt_err_t rt_bt_prot_attach_pan_instance(struct rt_bt_pan_instance *panInstance);

rt_err_t rt_bt_prot_detach_pan_instance(struct rt_bt_pan_instance *bt_instance);
rt_err_t rt_bt_prot_regisetr(struct rt_bt_prot *prot);
//rt_err_t rt_bt_prot_event_register(struct rt_bt_prot *prot, rt_bt_prot_event_t event, rt_bt_prot_event_handler handler);
rt_err_t rt_bt_prot_event_unregister(struct rt_bt_prot *prot, rt_bt_prot_event_t event);
rt_err_t rt_bt_prot_transfer_instance(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
rt_err_t rt_bt_instance_transfer_prot(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
extern void rt_lwip_instance_register_event_handler(struct rt_bt_pan_instance *bt_instance, rt_bt_instance_event_t event, rt_bt_instance_event_handler handler);






#ifdef __cplusplus
}
#endif

#endif
