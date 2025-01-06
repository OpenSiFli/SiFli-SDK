#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include "bf0_sibles_nvds.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "ulog.h"



enum bt_app_event_t
{
    BT_APP_CONNECT_PAN,
};


typedef struct
{
    U16 connected_profile;
    BOOL bt_connected;
    BTS2S_BD_ADDR bd_addr;
#ifdef BT_FINSH_PAN
    rt_timer_t pan_connect_timer;
#endif
} bt_app_t;

static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;



void bt_app_entry(void *param)
{
    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BT_APP_CONNECT_PAN)
        {
#ifdef BT_FINSH_PAN
            extern void bt_pan_conn_by_addr(BTS2S_BD_ADDR * remote_addr);

            LOG_I("[bt_app]prepare connect pan, addr %x-%x-%x\r\n", g_bt_app_env.bd_addr.nap, g_bt_app_env.bd_addr.uap, g_bt_app_env.bd_addr.lap);

            if (g_bt_app_env.bt_connected)
            {
                bt_pan_conn_by_addr(&g_bt_app_env.bd_addr);
            }
#endif
        }
    }
}



void bt_app_connect_pan_timeout_handle(void *parameter)
{
#ifdef BT_FINSH_PAN
    if ((g_bt_app_mb != NULL) && (g_bt_app_env.bt_connected))
        rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN);
#endif
    return;
}


void bt_app_check_pan_connect_opportunity(void)
{
#ifdef BT_FINSH_PAN
    if ((g_bt_app_env.connected_profile & BT_BASIC_PROFILE) == BT_BASIC_PROFILE)
    {
        if (g_bt_app_env.pan_connect_timer)
        {
            rt_timer_stop(g_bt_app_env.pan_connect_timer);
        }

        if ((g_bt_app_mb != NULL) && (g_bt_app_env.bt_connected))
            rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN);
    }
#endif
    return;
}


int bt_app_hci_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    case DM_EN_ACL_OPENED_IND:
    {
        BTS2S_DM_EN_ACL_OPENED_IND *ind = (BTS2S_DM_EN_ACL_OPENED_IND *)msg;

        LOG_I("[bt_app]link connected COD:%d Incoming:%d res %d\r\n", ind->dev_cls, ind->incoming, ind->st);
        LOG_I("[bt_app]bd addr %x-%x-%x\r\n", ind->bd.nap, ind->bd.uap, ind->bd.lap);

        // Enable first
        if (ind->st != HCI_SUCC)
        {
            LOG_I("[bt_app]acl connect fail!!!!\n");
        }
        else
        {
            memcpy(&g_bt_app_env.bd_addr, &ind->bd, sizeof(BTS2S_BD_ADDR));
            g_bt_app_env.bt_connected = TRUE;
            g_bt_app_env.connected_profile = 0;

#ifdef BT_FINSH_PAN
            if (!g_bt_app_env.pan_connect_timer)
            {
                g_bt_app_env.pan_connect_timer = rt_timer_create("connect_pan", bt_app_connect_pan_timeout_handle, (void *)&g_bt_app_env,
                                                 rt_tick_from_millisecond(5000), RT_TIMER_FLAG_SOFT_TIMER);
            }
            else
            {
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            }
            rt_timer_start(g_bt_app_env.pan_connect_timer);
#endif
        }
        break;
    }
    case DM_ACL_DISC_IND:
    {
        BTS2S_DM_ACL_DISC_IND *ind = (BTS2S_DM_ACL_DISC_IND *)msg;
        LOG_I("[bt_app]link dis-connected %x %d\r\n", ind->hdl, ind->reason);

        g_bt_app_env.bt_connected = FALSE;
        g_bt_app_env.connected_profile = 0;

        g_bt_app_env.bd_addr.lap = CFG_BD_LAP;
        g_bt_app_env.bd_addr.nap = CFG_BD_NAP;
        g_bt_app_env.bd_addr.uap = CFG_BD_UAP;

#ifdef BT_FINSH_PAN
        if (g_bt_app_env.pan_connect_timer)
        {
            rt_timer_stop(g_bt_app_env.pan_connect_timer);
        }
#endif

        break;
    }
    }

    return 0;
}


int bt_app_gap_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    // Using RD LOCAL NAME CFM as app init completed
    case BTS2MU_GAP_RD_LOCAL_NAME_CFM:
    {
        LOG_I("[bt_app]BT CM rd local dev cfm");

        rt_thread_t tid;
        g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
        tid = rt_thread_create("bt_app", bt_app_entry, NULL, 1024, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
        rt_thread_startup(tid);
        break;
    }

    default:
        break;
    }

    return 0;
}


int bt_app_hf_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    case BTS2MU_HF_CONN_IND:
    {
        BTS2S_HF_CONN_IND *ind = (BTS2S_HF_CONN_IND *)msg;
        LOG_I("[bt_app]hf connected\r\n %d", ind->srv_chnl);

        g_bt_app_env.connected_profile |= BT_CM_HFP;

        bt_app_check_pan_connect_opportunity();

        break;
    }
    case BTS2MU_HF_CONN_CFM:
    {
        BTS2S_HF_CONN_CFM *ind = (BTS2S_HF_CONN_CFM *)msg;
        LOG_I("[bt_app]hf conn cfm %d", ind->res);

        if (ind->res == BTS2_SUCC)
        {
            g_bt_app_env.connected_profile |= BT_CM_HFP;

            bt_app_check_pan_connect_opportunity();
        }

        break;
    }
    case BTS2MU_HF_DISC_IND:
    {
        BTS2S_HF_DISC_IND *ind = (BTS2S_HF_DISC_IND *)msg;
        LOG_I("[bt_app]hf dis-connected %d\r\n", ind->res);
        g_bt_app_env.connected_profile &= (~BT_CM_HFP);
        break;
    }
    default:
        break;
    }
    return 0;
}


int bt_app_a2dp_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    case BTS2MU_AV_CONN_IND:
    {
        BTS2S_AV_CONN_IND *ind = (BTS2S_AV_CONN_IND *)msg;
        LOG_I("[bt_app]a2dp connect ind %d\r\n", ind->conn_id);

        g_bt_app_env.connected_profile |= BT_CM_A2DP;

        bt_app_check_pan_connect_opportunity();

        break;
    }
    case BTS2MU_AV_CONN_CFM:
    {

        BTS2S_AV_CONN_CFM *ind = (BTS2S_AV_CONN_CFM *)msg;
        LOG_I("[bt_app]a2dp connect cfm %d res %d\r\n", ind->conn_id, ind->res);

        if (ind->res == AV_ACPT)
        {
            g_bt_app_env.connected_profile |= BT_CM_A2DP;

            bt_app_check_pan_connect_opportunity();
        }

        break;

    }
    case BTS2MU_AV_DISC_IND:
    {
        BTS2S_AV_DISC_IND *ind = (BTS2S_AV_DISC_IND *)msg;
        LOG_I("[bt_app]a2dp dis-connected(%d) %d\r\n", ind->conn_id, ind->res);

        g_bt_app_env.connected_profile &= (~BT_CM_A2DP);
        break;
    }
    default:
        break;
    }
    return 0;

}


int bt_app_avrcp_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    case BTS2MU_AVRCP_CONN_CFM:
    {
        BTS2S_AVRCP_CONN_CFM *ind = (BTS2S_AVRCP_CONN_CFM *)msg;
        if (ind->res == BTS2_SUCC)
        {
            LOG_I("[bt_app]avrcp connect successed \n");

            g_bt_app_env.connected_profile |= BT_CM_AVRCP;

            bt_app_check_pan_connect_opportunity();
        }
        break;
    }
    case BTS2MU_AVRCP_CONN_IND:
    {
        BTS2S_AVRCP_CONN_IND *ind = (BTS2S_AVRCP_CONN_IND *)msg;
        LOG_I("[bt_app]avrcp indicate to connect with remote device\n");

        g_bt_app_env.connected_profile |= BT_CM_AVRCP;

        bt_app_check_pan_connect_opportunity();
        break;
    }
    case BTS2MU_AVRCP_DISC_IND:
    {
        BTS2S_AVRCP_DISC_IND *ind = (BTS2S_AVRCP_DISC_IND *)msg;

        LOG_I("[bt_app]avrcp indicate to disconnect with remote device\n");

        g_bt_app_env.connected_profile &= (~BT_CM_AVRCP);

        break;
    }
    default:
        break;
    }
    return 0;

}


int bt_app_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    if (type == BTS2M_HFP_HF)
    {
        bt_app_hf_event_handler(event_id, msg);
    }
    else if (type == BTS2M_AV)
    {
        bt_app_a2dp_event_handler(event_id, msg);
    }
    else if (type == BTS2M_HCI_CMD)
    {
        bt_app_hci_event_handler(event_id, msg);
    }
    else if (type == BTS2M_AVRCP)
    {
        bt_app_avrcp_event_handler(event_id, msg);
    }
    else if (type == BTS2M_GAP)
    {
        bt_app_gap_event_handler(event_id, msg);
    }

    return 0;

}
BT_EVENT_REGISTER(bt_app_event_hdl, NULL);

