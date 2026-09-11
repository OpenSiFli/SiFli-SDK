/*
 * SPDX-FileCopyrightText: 2024-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "usbh_core.h"
#include "usbh_rndis.h"
#include "bf0_sibles.h"
#include "bf0_ble_common.h"
#include "bnep_dev.h"
#include "bt_prot.h"
#ifdef BT_FINSH
    #include "bts2_msg.h"
    #include "gap_api.h"
    #include "sc_api.h"
    #include "bts2_app_interface.h"
    #include "bts2_app_pan.h"
    #include "pan_api.h"
#endif
#include <stdint.h>
#include <string.h>
#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #include "drv_flash.h"
#endif

#define LINK_POLL_INTERVAL_MS 500U
#define RNDIS_LINK_SETTLE_MS 3000U
#define BT_APP_READY 1U
#define PAN_INQUIRY_TIMEOUT_S 15U
#define PAN_INQUIRY_MAX_RESPONSES 10U
#define PAN_CONNECT_TIMEOUT_MS 25000U

#define BRIDGE_ETH_HEADER_LEN 14
#define BRIDGE_ETH_FRAME_MAX_LEN 1600

#ifdef USB_BT_BRIDGE_BT_NAME
    #define USB_BT_BRIDGE_LOCAL_NAME USB_BT_BRIDGE_BT_NAME
#else
    #define USB_BT_BRIDGE_LOCAL_NAME "SiFli_USB_Bridge"
#endif

static rt_mailbox_t s_bt_ready_mb;

/* PAN connection state (used by the pan_net command) */
static volatile uint8_t s_bt_stack_ready;
static volatile uint8_t s_pan_connected;
static volatile uint8_t s_pan_connect_pending;
static rt_timer_t s_pan_connect_timer;
static BTS2S_BD_ADDR s_pan_target;
static volatile uint16_t s_pan_bnep_id = 0xffff; /* current PAN BNEP channel id */

static void pan_connect_timeout(void *parameter)
{
    (void)parameter;
    if (s_pan_connect_pending)
    {
        s_pan_connect_pending = 0U;
        rt_kprintf("[BT] PAN connect timeout: %04X:%02X:%06lX\n",
                   s_pan_target.nap, s_pan_target.uap, s_pan_target.lap);
    }
}

static int usb_bt_bridge_bt_event_handler(uint16_t type,
        uint16_t event_id,
        uint8_t *data,
        uint16_t data_len)
{
    if (type == BT_NOTIFY_COMMON &&
            event_id == BT_NOTIFY_COMMON_BT_STACK_READY)
    {
        s_bt_stack_ready = 1U;
        if (s_bt_ready_mb != RT_NULL)
        {
            rt_mb_send(s_bt_ready_mb, BT_APP_READY);
        }
    }
    else if (type == BT_NOTIFY_PAN)
    {
        if (event_id == BT_NOTIFY_PAN_PROFILE_CONNECTED)
        {
            const bt_notify_profile_state_info_t *profile;

            s_pan_connected = 1U;
            s_pan_connect_pending = 0U;
            profile = (const bt_notify_profile_state_info_t *)data;
            if (data != RT_NULL && data_len >= sizeof(*profile))
            {
                s_pan_bnep_id = profile->profile_channel;
            }
            if (s_pan_connect_timer != RT_NULL)
            {
                rt_timer_stop(s_pan_connect_timer);
            }
            rt_kprintf("[BT] PAN connected, USB<->BT bridge ready\n");
        }
        else if (event_id == BT_NOTIFY_PAN_PROFILE_DISCONNECTED)
        {
            s_pan_connected = 0U;
            s_pan_bnep_id = 0xffff;
            rt_kprintf("[BT] PAN disconnected\n");
        }
    }

    return 0;
}

static void bt_start_thread(void *parameter)
{
    rt_uint32_t event;

    (void)parameter;
    sifli_ble_enable();
    rt_kprintf("[BT] Bluetooth Classic PAN startup requested\n");

    if (rt_mb_recv(s_bt_ready_mb, &event, rt_tick_from_millisecond(10000)) == RT_EOK &&
            event == BT_APP_READY)
    {
        bt_interface_set_local_name(strlen(USB_BT_BRIDGE_LOCAL_NAME),
                                    (void *)USB_BT_BRIDGE_LOCAL_NAME);
        rt_kprintf("[BT] Classic Bluetooth stack ready, PAN NAP enabled\n");
    }
    else
    {
        rt_kprintf("[BT] Bluetooth stack ready timeout\n");
    }
}

static int wait_for_rndis_link(void)
{
    while (1)
    {
        if (usbh_rndis_is_link_up())
        {
            return 1;
        }

        rt_thread_mdelay(LINK_POLL_INTERVAL_MS);
    }
}

/* ----------------------------------------------------------------------
 * USB <-> BT transparent bridge (Plan B)
 *
 * Overrides the weak hooks in usbh_rndis.c (usbh_rndis_on_raw_rx) and
 * bts2_app_pan.c (bt_pan_on_raw_rx). Incoming ethernet frames are
 * forwarded directly between USB RNDIS and BT PAN, bypassing lwIP
 * entirely: no NAT, no DHCP client/server, no IP routing.
 * The USB RX hook is void: the frame is forwarded or dropped. The BT RX hook
 * returns 0 once the frame has been forwarded to USB, non-zero keeps the
 * normal BNEP/lwIP path.
 * -------------------------------------------------------------------- */
// USB → BT
void usbh_rndis_on_raw_rx(uint8_t *buf, uint32_t len)
{
    /* forward USB frame to the connected PAN peer (by BNEP channel id) */
    if (s_pan_bnep_id == 0xffff)
    {
        /* no PAN peer yet, drop frame */
        return;
    }

    if (len < BRIDGE_ETH_HEADER_LEN || len > BRIDGE_ETH_FRAME_MAX_LEN)
    {
        return;
    }

    bt_pan_send_data(s_pan_bnep_id, buf, len);
}
// BT → USB
int bt_pan_on_raw_rx(void *buff, int len)
{
    uint8_t *txbuf;

    if (len <= 0 || len > BRIDGE_ETH_FRAME_MAX_LEN)
    {
        return -1;
    }

    txbuf = usbh_rndis_get_eth_txbuf();
    if (txbuf == RT_NULL)
    {
        return -1;
    }

    /* forward BT PAN frame to USB RNDIS (phone) */
    memcpy(txbuf, buff, len);
    return usbh_rndis_eth_output(len) < 0 ? -1 : 0;
}

static void rndis_monitor_thread(void *parameter)
{
    (void)parameter;
    rt_kprintf("[USB] Waiting for RNDIS link\n");

    while (1)
    {
        /* transparent bridge: no DHCP client / no netif, just wait for the link */
        wait_for_rndis_link();
        rt_thread_mdelay(RNDIS_LINK_SETTLE_MS);
        if (!usbh_rndis_is_link_up())
        {
            /* link dropped again while settling, keep waiting */
            continue;
        }
        rt_kprintf("[USB] RNDIS link up, USB<->BT bridge active\n");

        while (1)
        {
            rt_thread_mdelay(LINK_POLL_INTERVAL_MS);
            if (!usbh_rndis_is_link_up())
            {
                rt_kprintf("[USB] RNDIS link lost, bridge inactive\n");
                break;
            }
        }
    }
}

#if defined(BT_FINSH) && defined(BT_FINSH_PAN)
static void pan_net_help(void)
{
    rt_kprintf("Usage:\n");
    rt_kprintf("  pan_net inquiry start\n");
    rt_kprintf("  pan_net inquiry stop\n");
    rt_kprintf("  pan_net conn <XX:XX:XX:XX:XX:XX|XXXXXXXXXXXX>\n");
}

__ROM_USED void pan_net(int argc, char **argv)
{
    bd_addr_t mac;
    BTS2S_BD_ADDR address;
    int ret;

    if (argc == 3 && strcmp(argv[1], "inquiry") == 0)
    {
        if (!s_bt_stack_ready)
        {
            rt_kprintf("[BT] Bluetooth stack is not ready\n");
            return;
        }

        if (strcmp(argv[2], "start") == 0)
        {
            bt_start_inquiry_ex_t inquiry = {0};

            inquiry.dev_cls_mask = 0U;
            inquiry.max_timeout = PAN_INQUIRY_TIMEOUT_S;
            inquiry.max_rsp = PAN_INQUIRY_MAX_RESPONSES;
            ret = bt_interface_start_inquiry_ex(&inquiry);
            if (ret == 0)
            {
                rt_kprintf("[BT] Inquiry request accepted timeout=%us max_rsp=%u\n",
                           inquiry.max_timeout, inquiry.max_rsp);
            }
            else
            {
                rt_kprintf("[BT] Inquiry request rejected ret=%d\n", ret);
            }
            return;
        }
        if (strcmp(argv[2], "stop") == 0)
        {
            bt_interface_stop_inquiry();
            rt_kprintf("[BT] Inquiry stop requested\n");
            return;
        }
    }
    else if (argc == 3 && strcmp(argv[1], "conn") == 0)
    {
        if (!s_bt_stack_ready)
        {
            rt_kprintf("[BT] Bluetooth stack is not ready\n");
            return;
        }

        if (s_pan_connect_pending)
        {
            rt_kprintf("[BT] PAN connection already pending\n");
            return;
        }
        if (s_pan_connected)
        {
            rt_kprintf("[BT] PAN is already connected\n");
            return;
        }

        if (bt_addr_convert_from_string_to_general(argv[2], &mac) != BD_ADDR_LEN)
        {
            rt_kprintf("[BT] Invalid Bluetooth address: %s\n", argv[2]);
            pan_net_help();
            return;
        }

        bt_addr_convert_to_bts(&mac, &address);
        if (address.nap == 0U && address.uap == 0U && address.lap == 0U)
        {
            rt_kprintf("[BT] Invalid Bluetooth address: %s\n", argv[2]);
            pan_net_help();
            return;
        }

        if (s_pan_connect_timer == RT_NULL)
        {
            s_pan_connect_timer = rt_timer_create("pan_conn",
                                                  pan_connect_timeout,
                                                  RT_NULL,
                                                  rt_tick_from_millisecond(PAN_CONNECT_TIMEOUT_MS),
                                                  RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
            if (s_pan_connect_timer == RT_NULL)
            {
                rt_kprintf("[BT] PAN connect timer create failed\n");
                return;
            }
        }

        s_pan_target = address;
        s_pan_connect_pending = 1U;
        rt_timer_stop(s_pan_connect_timer);
        if (rt_timer_start(s_pan_connect_timer) != RT_EOK)
        {
            s_pan_connect_pending = 0U;
            rt_kprintf("[BT] PAN connect timer start failed\n");
            return;
        }

        bt_interface_stop_inquiry();
        ret = bt_pan_connect_request_ext(&address, PAN_PANU_ROLE);
        if (ret != 0)
        {
            rt_timer_stop(s_pan_connect_timer);
            s_pan_connect_pending = 0U;
            rt_kprintf("[BT] PAN connect request rejected ret=%d\n", ret);
            return;
        }

        rt_kprintf("[BT] PAN connect request accepted: %04X:%02X:%06lX (local NAP, remote PANU)\n",
                   address.nap, address.uap, address.lap);
        return;
    }

    pan_net_help();
}
MSH_CMD_EXPORT(pan_net, scan and connect a Bluetooth PANU device)
#endif

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
#define NAND_MTD_NAME "root"
static int fs_mount_init(void)
{
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("[FS] mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("[FS] mount fs on flash to root fail, mkfs first\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("[FS] make elm fs on flash success, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
            {
                rt_kprintf("[FS] mount fs on flash success\n");
            }
            else
            {
                rt_kprintf("[FS] mount to fs on flash fail\n");
            }
        }
        else
        {
            rt_kprintf("[FS] dfs_mkfs elm flash fail\n");
        }
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(fs_mount_init);
#endif

int main(void)
{
    rt_thread_t bt_thread;
    rt_thread_t thread;
    int ret;

    s_bt_ready_mb = rt_mb_create("bt_ready", 1, RT_IPC_FLAG_FIFO);
    if (s_bt_ready_mb == RT_NULL)
    {
        rt_kprintf("[BT] Ready mailbox create failed\n");
        return -RT_ENOMEM;
    }

    ret = (int)bt_interface_register_bt_event_notify_callback(
              usb_bt_bridge_bt_event_handler);
    if (ret != BT_INTERFACE_STATUS_OK)
    {
        rt_kprintf("[BT] Event callback registration failed ret=%d\n", ret);
        return ret;
    }

    bt_thread = rt_thread_create("bt_start", bt_start_thread, RT_NULL,
                                 2048, 18, 10);
    if (bt_thread == RT_NULL)
    {
        rt_kprintf("[BT] Startup thread create failed\n");
        return -RT_ENOMEM;
    }
    rt_thread_startup(bt_thread);

    ret = usbh_initialize(0, (uintptr_t)USBC_BASE, RT_NULL);
    if (ret < 0)
    {
        rt_kprintf("[USB] Host init failed ret=%d\n", ret);
        return ret;
    }

    thread = rt_thread_create("rndis_mon", rndis_monitor_thread, RT_NULL,
                              2048, 20, 10);
    if (thread == RT_NULL)
    {
        rt_kprintf("[USB] RNDIS monitor create failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(thread);
    return 0;
}
