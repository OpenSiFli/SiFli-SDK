/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bap_broadcast_src_api.h"

/********* classis bt section start *****/
#include "bf0_sibles.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"

#define BT_APP_READY  1

#ifdef BT_DEVICE_NAME
    static const char *local_name = BT_DEVICE_NAME;
#else
    static const char *local_name = "sifli_music_sink";
#endif



typedef struct
{
    bt_notify_device_mac_t  addr;
    uint8_t is_a2dp_connected;
    uint8_t is_abs_enabled;
} bt_app_t;

static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;

static bt_app_t *bt_app_get_env(void)
{
    return &g_bt_app_env;
}



static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    bt_app_t *env = bt_app_get_env();

    if (type == BT_NOTIFY_COMMON)
    {
        switch (event_id)
        {
        case BT_NOTIFY_COMMON_BT_STACK_READY:
        {
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
        }
        break;
        default:
            break;
        }
    }
    else if (type == BT_NOTIFY_A2DP)
    {
        switch (event_id)
        {
        case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
        {
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            if (profile_info->res == BTS2_SUCC)
            {
                env->addr = profile_info->mac;
                env->is_a2dp_connected = 1;
            }
            rt_kprintf("A2DP connected\n");
        }
        break;
        case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *info = (bt_notify_profile_state_info_t *)data;
            env->is_a2dp_connected = 0;
            rt_kprintf("A2DP disconnected %d\n", info->res);
        }
        break;
        }
    }
    else if (type == BT_NOTIFY_AVRCP)
    {
        switch (event_id)
        {
        case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
        {
            rt_kprintf("AVRCP connected\n");
            BTS2S_BD_ADDR bd_addr;
            bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
            bt_addr_convert_to_bts((bd_addr_t *)&profile_info->mac, &bd_addr);
            bt_interface_set_avrcp_role(&bd_addr, AVRCP_CT);
        }
        break;
        case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
        {
            bt_notify_profile_state_info_t *info = (bt_notify_profile_state_info_t *)data;
            env->is_abs_enabled = 0;
            rt_kprintf("AVRCP disconnected %d", info->res);
        }
        break;
        case BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER:
        {
            env->is_abs_enabled = 1;
        }
        break;
        case BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME:
        {
            uint8_t *volume = (uint8_t *)data;
#ifdef AUDIO_USING_MANAGER
            uint8_t local_vol = bt_interface_avrcp_abs_vol_2_local_vol(*volume, audio_server_get_max_volume());
            audio_server_set_private_volume(AUDIO_TYPE_BT_MUSIC, local_vol);
#endif
        }
        break;
        default:
            break;
        }
    }

    return 0;
}


/********* classis bt section end *****/
int main(void)
{
    struct bt_le_ext_adv *adv;
    uint32_t value;
    int err;

    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_bt_app_mb);

    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");


    // Wait for stack/profile ready.
    if (RT_EOK == rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, 8000) && value == BT_APP_READY)
    {
        rt_kprintf("BT/BLE stack and profile ready\n");
        // Update Bluetooth name
        bt_interface_set_local_name(strlen(local_name), (void *)local_name);
    }
    else
        rt_kprintf("BT/BLE stack and profile init failed\n");


    printk("input play play filename\n");
    rt_tick_t tick = rt_tick_get();

    printk("\r\nrun audio_src 0 to stop\r\n");

    //must wait some seconds, otherwise start ble audio src error
    for (int i = 0; i < 5; i++)
    {
        rt_thread_mdelay(1000);
    }

    bap_broadcast_src_start();

    return 0;
}


__ROM_USED void audio_src(int argc, char **argv)
{
    if (argc < 2)
        return;

    if (argv[1][0] == '0')
    {
        bap_broadcast_src_stop();
        printk("\r\ninput audio_src 1 to start\n\n");
    }
    else
    {
        bap_broadcast_src_start();
        printk("\r\ninput audio_src 0 to stop\n\n");
    }
}
MSH_CMD_EXPORT(audio_src, audio_src command)

static void help(void)
{
}


__ROM_USED void music(int argc, char **argv)
{
    bt_app_t *env = bt_app_get_env();

    if (argc < 2)
        help();
    else
    {
        if (strcmp(argv[1], "c") == 0)
        {
#ifdef BSP_BT_CONNECTION_MANAGER
            bt_cm_delete_bonded_devs();
#endif // BSP_BT_CONNECTION_MANAGER
        }
        else if (strcmp(argv[1], "set_vol") == 0)
        {
            uint8_t max_vol = 15;
#ifdef AUDIO_USING_MANAGER
            max_vol = audio_server_get_max_volume();
#endif // AUDIO_USING_MANAGER
            uint8_t local_vol = atoi(argv[2]);
            uint8_t abs_vol = bt_interface_avrcp_local_vol_2_abs_vol(local_vol, max_vol);
            bt_interface_avrcp_set_absolute_volume_as_ct_role(abs_vol);
#ifdef AUDIO_USING_MANAGER
            // If absolute volume register by peer device, then local volume shall also adjust.
            if (env->is_abs_enabled)
                audio_server_set_private_volume(AUDIO_TYPE_BT_MUSIC, local_vol);
#endif // AUDIO_USING_MANAGER
        }
        else if (strcmp(argv[1], "conn") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            bt_interface_conn_ext((void *)&mac, BT_PROFILE_A2DP);
        }
    }
}
MSH_CMD_EXPORT(music, music command)


