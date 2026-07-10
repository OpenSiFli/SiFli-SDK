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
//#include "utf8_unicode.h"
#include "bt_rt_device.h"
#include "bts2_global.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "bt_rt_device_urc.h"

#define DBG_TAG               "bt_rt_device.urc_avrcp"
//#define DBG_LVL               DBG_INFO
#include <log.h>


void urc_func_bt_avrcp_open_complete_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_AVRCP_OPEN_COMPLETE;
    args.args = RT_NULL;
    if (bt_sifli_check_bt_event(BT_SET_AVRCP_OPEN_EVENT))
    {
        LOG_I("URC BT avrcp open complete ind %x", bt_sifli_get_bt_event());
        bt_sifli_reset_bt_event(BT_SET_AVRCP_OPEN_EVENT);
        rt_bt_event_notify(&args);
    }
    return;
}

void urc_func_bt_avrcp_close_complete_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_AVRCP_CLOSE_COMPLETE;
    args.args = RT_NULL;
    if (bt_sifli_check_bt_event(BT_SET_AVRCP_CLOSE_EVENT))
    {
        LOG_I("URC BT avrcp close complete ind %x", bt_sifli_get_bt_event());
        bt_sifli_reset_bt_event(BT_SET_AVRCP_CLOSE_EVENT);
        rt_bt_event_notify(&args);
    }
    return;
}

void urc_func_bt_avrcp_playback_status_sifli(bt_media_play_status_t *ind)
{
    bt_notify_t args;
    ind->conn_idx = urc_func_bt_get_conn_idx(&ind->mac);
    args.event = BT_EVENT_MUSIC_PLAY_STATUS_CHANGED;
    args.args = ind;

    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp conn_id(%d) playback status ind %d", ind->conn_idx, ind->status);
}

void urc_func_bt_avrcp_volume_change_rigister_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_AVRCP_VOLUME_CHANGE_RIGISTER;
    args.args = RT_NULL;

    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp volume change rigister ind");
}

void urc_func_bt_avrcp_track_change_sifli(uint8_t track_change)
{
    bt_notify_t args;
    args.event = BT_EVENT_MUSIC_TRACK_CHANGED;
    args.args = &track_change;// solution 0x00:previous ;0x01:next

    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp track change ind");
}

void urc_func_bt_avrcp_cover_art_data_sifli(uint8_t is_final_packet, uint8_t *addr, uint16_t total_length, uint8_t *payload, uint16_t payload_len)
{
    bt_notify_t args;
    bt_avrcp_cover_art_data_t data_ind = {0};
    data_ind.data = payload;
    data_ind.len = payload_len;
    data_ind.total_length = total_length;
    data_ind.is_final_packet = is_final_packet;
    rt_memcpy(data_ind.mac_addr.addr, addr, BT_MAX_MAC_LEN);
    args.event = BT_EVENT_AVRCP_COVER_ART_DATA;
    args.args = &data_ind;

    rt_bt_event_notify(&args);
    if (total_length)
        LOG_I("URC BT avrcp cover art data ind %d", total_length);
}

void urc_func_bt_avrcp_update_cover_art_sifli(uint8_t *addr)
{
    bt_notify_t args;
    args.event = BT_EVENT_AVRCP_UPDATE_COVER_ART;
    args.args = addr;

    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp update cover art");
}

void urc_func_bt_avrcp_cover_art_data_end_sifli(uint8_t *addr)
{
    bt_notify_t args;
    args.event = BT_EVENT_AVRCP_COVER_ART_DATA_ERR;
    args.args = addr;

    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp cover art data end");
}

void urc_func_bt_avrcp_song_change_sifli(void)
{
    bt_notify_t args;
    args.event = BT_EVENT_AVRCP_SONG_CHANGED;
    args.args = NULL;

    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp song change ind");
}

void urc_func_bt_avrcp_mp3_detail_sifli(bt_mp3_detail_info_t *detail)
{
    bt_notify_t args;
    args.event = BT_EVENT_MP3_DETAIL_INFO;
    args.args = detail;
    rt_bt_event_notify(&args);

    LOG_I("URC BT mp3 detail ind");
}

static void urc_func_bt_avrcp_song_progress_sifli(bt_media_play_progress_t *progress)
{
    bt_notify_t args;
    args.event = BT_EVENT_SONG_PLAY_PROGRESS;
    progress->conn_idx = urc_func_bt_get_conn_idx(&progress->mac);
    args.args = progress;

    rt_bt_event_notify(&args);
    LOG_I("URC BT mp3 conn_idx(%d) progress %d ms", progress->conn_idx, progress->progress); //unit :ms. example:0x92be->37566ms
}

static void urc_func_bt_avrcp_absolute_volume_sifli(bt_volume_set_t *volume)
{
    bt_notify_t args;
    volume->conn_idx = urc_func_bt_get_conn_idx(&volume->mac);
    args.event = BT_EVENT_VOL_CHANGED;
    args.args = volume;
    rt_bt_event_notify(&args);
    LOG_I("URC BT avrcp ab-volue idx:%d vol:%d", volume->conn_idx, volume->volume.media_volume);
}

int bt_sifli_notify_avrcp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_AVRCP_CLOSE_COMPLETE:
    {
        urc_func_bt_avrcp_close_complete_sifli();
        break;
    }
    case BT_NOTIFY_AVRCP_OPEN_COMPLETE:
    {
        urc_func_bt_avrcp_open_complete_sifli();
        break;
    }
    case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        bt_cm_dev_info_t *bonded_dev = bt_cm_get_bonded_dev_by_addr(profile_info->mac.addr);
        if (bonded_dev->link_type == BT_LINK_PHONE)
        {
            bt_interface_set_avrcp_role_ext(&profile_info->mac, AVRCP_CT);
        }
        else
        {
            bt_interface_set_avrcp_role_ext(&profile_info->mac, AVRCP_TG);
        }
        urc_func_profile_conn_sifli(profile_info, BT_PROFILE_AVRCP);
        break;
    }
    case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_AVRCP, profile_info->res);
        break;
    }
    case BT_NOTIFY_AVRCP_MUSIC_DETAIL_INFO:
    {
        urc_func_bt_avrcp_mp3_detail_sifli((bt_mp3_detail_info_t *) data);
        break;
    }
    case BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER:
    {
        urc_func_bt_avrcp_volume_change_rigister_sifli();
        break;
    }
    case BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME:
    {
        urc_func_bt_avrcp_absolute_volume_sifli((bt_volume_set_t *)data);
        break;
    }
    case BT_NOTIFY_AVRCP_PLAY_STATUS:
    {
        urc_func_bt_avrcp_playback_status_sifli((bt_media_play_status_t *)data);
        break;
    }
    case BT_NOTIFY_SWITCH_SONG:
    {
        urc_func_bt_avrcp_song_change_sifli();
        break;
    }
    case BT_NOTIFY_AVRCP_SONG_PROGREAS_STATUS:
    {
        urc_func_bt_avrcp_song_progress_sifli((bt_media_play_progress_t *)data);
        break;
    }
    case BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS:
    {
        urc_func_bt_avrcp_track_change_sifli(data[0]);
        break;
    }
    case BT_NOTIFY_AVRCP_COVER_ART_DATA_IND:
    {
        bt_notify_avrcp_cover_art_ind_t *avrcp_data_info = (bt_notify_avrcp_cover_art_ind_t *)data;
        urc_func_bt_avrcp_cover_art_data_sifli(avrcp_data_info->is_final_packet, avrcp_data_info->mac.addr, avrcp_data_info->total_len, avrcp_data_info->payload, avrcp_data_info->payload_len);
        break;
    }
    case BT_NOTIFY_AVRCP_UPDATE_COVER_ART:
    {
        //!todo:update cover art
        bt_notify_device_mac_t *mac = (bt_notify_device_mac_t *)data;
        urc_func_bt_avrcp_update_cover_art_sifli(data);
        break;
    }
    case BT_NOTIFY_AVRCP_COVER_ART_DATA_END:
    {
        bt_notify_device_mac_t *mac = (bt_notify_device_mac_t *)data;
        urc_func_bt_avrcp_cover_art_data_end_sifli(data);
        break;
    }
    default:
        return -1;
    }

    return 0;
}

