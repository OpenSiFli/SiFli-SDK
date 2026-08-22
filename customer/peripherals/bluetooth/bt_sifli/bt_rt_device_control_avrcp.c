/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
#include "bt_rt_device.h"
#include "bts2_global.h"
#include "bts2_app_inc.h"
#include "bf0_ble_common.h"
#include "bf0_sibles.h"
#include "bt_connection_manager.h"

#define DBG_TAG               "bt_rt_device.control_avrcp"
//#define DBG_LVL               DBG_INFO
#include <log.h>


#ifdef BT_USING_AVRCP
bt_err_t bt_sifli_set_avrcp_volume(rt_bt_device_t *dev, bt_volume_set_t *set)
{
    uint8_t volume, temp_volume;
    temp_volume = set->volume.media_volume;
    bt_err_t ret = BT_EOK;
    uint8_t max_vol = 15;
#ifdef     AUDIO_USING_MANAGER
    max_vol = audio_server_get_max_volume();
#endif // AUDIO_USING_MANAGER
    volume = bt_interface_avrcp_local_vol_2_abs_vol(temp_volume, max_vol);

    if (BT_STATE_CONNECTED != rt_bt_get_connect_state_by_conn_idx(dev, set->conn_idx, BT_PROFILE_AVRCP))
    {
        return BT_ERROR_DISCONNECTED;
    }

    bt_connect_dev_t *link_dev = rt_bt_get_connect_dev_by_addr(dev, &set->mac);
    if (link_dev)
    {
        if (link_dev->link_type == BT_LINK_EARPHONE)
        {
            ret = bt_interface_avrcp_set_absolute_volume_as_tg_role_ext((bt_notify_device_mac_t *)&set->mac, volume);
        }
        else
        {
            ret = bt_interface_avrcp_set_absolute_volume_as_ct_role_ext((bt_notify_device_mac_t *)&set->mac, volume);
        }
    }
    else
        return BT_ERROR_DISCONNECTED;

    if (BT_EOK == ret)
    {
#ifdef AUDIO_USING_MANAGER
        audio_server_set_private_volume(AUDIO_TYPE_BT_MUSIC, temp_volume);
#endif // AUDIO_USING_MANAGER
    }

    return ret;
}
#endif

bt_err_t bt_sifli_control_avrcp(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case BT_CONTROL_OPEN_AVRCP:
    {
        LOG_I("open avrcp ,event %x", bt_sifli_get_bt_event());
        if (bt_sifli_check_bt_event(BT_SET_AVRCP_CLOSE_EVENT))
        {
            LOG_I("during avrcp close porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_AVRCP_OPEN_EVENT);
            bt_interface_open_avrcp();
        }
    }
    break;

    case BT_CONTROL_CLOSE_AVRCP:
    {
        LOG_I("close avrcp ,event %x", bt_sifli_get_bt_event());
        if (bt_sifli_check_bt_event(BT_SET_AVRCP_OPEN_EVENT))
        {
            LOG_I("during avrcp open porcess");
            return BT_ERROR_STATE;
        }
        else
        {
            bt_sifli_set_bt_event(BT_SET_AVRCP_CLOSE_EVENT);
            bt_interface_close_avrcp();
        }
    }
    break;

#ifdef CFG_AVRCP_COVER_ART
    case BT_CONTROL_AVRCP_GET_COVER_ART:
    {
        //todo:Adapting avrcp multi-connection
        ret = bt_interface_avrcp_get_cover_art((bt_notify_device_mac_t *)args);
    }
    break;
#endif

#ifndef BT_CONNECT_SUPPORT_MULTI_LINK
    case BT_CONTROL_PHONE_PLAY_NEXT:
    {
        //todo:Adapting avrcp multi-connection
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_interface_avrcp_next();
    }
    break;

    case BT_CONTROL_PHONE_PLAY:
    {
        //todo:Adapting avrcp multi-connection
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_interface_avrcp_play();
    }
    break;

    case BT_CONTROL_PHONE_PLAY_SUSPEND:
    {
        //todo:Adapting avrcp multi-connection
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_interface_avrcp_pause();
    }
    break;

    case BT_CONTROL_PHONE_PLAY_STOP:
    {
        //todo:Adapting avrcp multi-connection
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_interface_avrcp_stop();
    }
    break;

    case BT_CONTROL_PHONE_PLAY_PREVIOUS:
    {
        //todo:Adapting avrcp multi-connection
        bt_mac_t *mac = (bt_mac_t *)args;
        bt_interface_avrcp_previous();
    }
    break;
#endif
    case BT_CONTROL_AVRCP_PLAY:
    {
        bt_interface_avrcp_play_ext((bt_notify_device_mac_t *)args);
    }
    break;

    case BT_CONTROL_AVRCP_PAUSE:
    {
        bt_interface_avrcp_pause_ext((bt_notify_device_mac_t *)args);
    }
    break;

    case BT_CONTROL_AVRCP_PREVIOUS:
    {
        bt_interface_avrcp_previous_ext((bt_notify_device_mac_t *)args);
    }
    break;

    case BT_CONTROL_AVRCP_NEXT:
    {
        bt_interface_avrcp_next_ext((bt_notify_device_mac_t *)args);
    }
    break;

    case BT_CONTROL_AVRCP_REWIND:
    {
        bt_interface_avrcp_rewind((bt_notify_device_mac_t *)args);
    }
    break;

    default:
        ret = BT_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

