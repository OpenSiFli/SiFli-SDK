/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-14     tyx          the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "pbuf.h"
//#include "bts2_app_demo.h"
//#include "bnep.h"
#include "rtdef.h"
#include "bts2_bt.h"
#include "bts2_app_pan.h"
#include "bt_prot.h"


#define DBG_TAG "BLUETOOTH.prot"
//#define DBG_LVL DBG_INFO
#include <rtdbg.h>


struct rt_bt_prot_event_des
{
    //rt_bt_prot_event_handler handler;
    struct rt_bt_prot *prot;
};

static struct rt_bt_prot *bt_prot;

static struct rt_bt_prot_event_des bt_prot_event_tab[RT_BT_PROT_EVT_MAX][MAX_PAN_INSTANCE_NUM];

extern struct rt_bt_pan_instance  bt_pan_instance[MAX_PAN_INSTANCE_NUM];


rt_err_t rt_bt_prot_attach_pan_instance(struct rt_bt_pan_instance *panInstance)
{
    panInstance->prot = bt_prot;
    panInstance->prot = bt_prot->ops->dev_reg_callback(panInstance->prot, panInstance); /* attach prot */
    return RT_EOK;
}


rt_err_t rt_bt_prot_detach_pan_instance(struct rt_bt_pan_instance *panInstance)
{
    panInstance->prot = bt_prot;
    bt_prot->ops->dev_unreg_callback(panInstance->prot, panInstance); /* deattach prot */
    return RT_EOK;
}



rt_err_t rt_bt_prot_regisetr(struct rt_bt_prot *prot)
{
    int i;
    rt_uint32_t id;
    static rt_uint8_t num;

    /* Parameter checking */
    if ((prot == RT_NULL) ||
            (prot->ops->prot_recv == RT_NULL) ||
            (prot->ops->dev_reg_callback == RT_NULL))
    {
        LOG_E("F:%s L:%d Parameter Wrongful", __FUNCTION__, __LINE__);
        return -RT_EINVAL;
    }

    /* save prot */
    bt_prot = prot;

    // rt_kprintf("lwip:rt_bt_prot_regisetr \n");

    return RT_EOK;
}


/*rt_err_t rt_bt_prot_event_register(struct rt_bt_prot *prot, rt_bt_prot_event_t event, rt_bt_prot_event_handler handler)
{
   int i;

   if ((prot == RT_NULL) || (handler == RT_NULL))
   {
       return -RT_EINVAL;
   }

  if (bt_prot_event_tab[event].handler == RT_NULL)
   {
       bt_prot_event_tab[event].handler = handler;
       bt_prot_event_tab[event].prot = prot;
       return RT_EOK;
   }


   return -RT_ERROR;
}*/

rt_err_t rt_bt_prot_event_unregister(struct rt_bt_prot *prot, rt_bt_prot_event_t event)
{
    int i;

    if (prot == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*if ((bt_prot_event_tab[event].handler != RT_NULL) &&
            (bt_prot_event_tab[event].prot == prot))
    {
        rt_memset(&bt_prot_event_tab[event], 0, sizeof(struct rt_bt_prot_event_des));
        return RT_EOK;
    }*/


    return -RT_ERROR;
}



rt_err_t rt_bt_prot_transfer_instance(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    if (bt_instance->ops->bt_send != RT_NULL)
    {
        bt_instance->ops->bt_send(bt_instance, buff, len);
        return RT_EOK;
    }
    return -RT_ERROR;
}



rt_err_t rt_bt_instance_transfer_prot(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    struct rt_bt_prot *prot = bt_instance->prot;

    if (prot != RT_NULL)
    {
        return prot->ops->prot_recv(bt_instance, buff, len);
    }
    return -RT_ERROR;
}



