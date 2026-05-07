/*
 * SPDX-FileCopyrightText: 2024-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
/* Common functions for RT-Thread based platform -----------------------------------------------*/
/**
  * @brief  Initialize board default configuration.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
    //__asm("B .");        /*For debugging purpose*/
    BSP_IO_Init();
}
/* User code start from here --------------------------------------------------------*/
#include "bts2_app_inc.h"
#include "ble_connection_manager.h"
#include "bt_connection_manager.h"

#include <rtdevice.h>
#include <board.h>

#include "bts2_app_demo.h"
#include "bts2_task.h"
#include "gap_api.h"
#include "hci_spec.h"
#include "hci_api.h"
#include "bts2_util.h"
#include "bts2_dbg.h"

#include "ulog.h"


static rt_mailbox_t g_bt_app_mb;


static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    if (type == BT_NOTIFY_COMMON)
    {
        switch (event_id)
        {
        case BT_NOTIFY_COMMON_ACL_CONNECTED:
        {
            bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *) data;

            LOG_I("[bt_app]link connected COD:%d Incoming:%d res %d\r\n", acl_info->dev_cls, acl_info->acl_dir, acl_info->res);
            LOG_I("[bt_app]bd addr %02x:%02x:%02x:%02x:%02x:%02x\r\n", acl_info->mac.addr[0], acl_info->mac.addr[1], acl_info->mac.addr[2], acl_info->mac.addr[3], acl_info->mac.addr[4], acl_info->mac.addr[5]);

            if (acl_info->res != HCI_SUCC)
            {
                LOG_I("[bt_app]acl connect fail!!!!\n");
            }
            else
            {
                LOG_I("[bt_app]acl connect success!!!!\n");
            }
        }
        break;
        case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
        {
            bt_notify_device_base_info_t *device_info = (bt_notify_device_base_info_t *)data;
            LOG_I("[bt_app]link dis-connected %d\r\n", device_info->res);
            LOG_I("[bt_app]bd addr %02x:%02x:%02x:%02x:%02x:%02x\r\n", device_info->mac.addr[0], device_info->mac.addr[1], device_info->mac.addr[2], device_info->mac.addr[3], device_info->mac.addr[4], device_info->mac.addr[5]);
        }
        break;
        default:
            break;
        }
    }
    else if (type == BT_NOTIFY_SPP)
    {
        switch (event_id)
        {
        case BT_NOTIFY_SPP_CONN_IND:
        {
            bt_notify_spp_conn_ind_t *conn_ind = (bt_notify_spp_conn_ind_t *)data;
            LOG_I("spp connect success!!!");
            LOG_I("[bt_app]bd addr %02x:%02x:%02x:%02x:%02x:%02x,chl %d\r\n", conn_ind->mac.addr[0], conn_ind->mac.addr[1], conn_ind->mac.addr[2], conn_ind->mac.addr[3], conn_ind->mac.addr[4], conn_ind->mac.addr[5], conn_ind->srv_chl);
            bt_interface_dump_all_spp_connection_info();
        }
        break;
        case BT_NOTIFY_SPP_DISC_IND:
        {
            bt_notify_spp_disc_ind_t *disc_ind = (bt_notify_spp_disc_ind_t *)data;
            LOG_I("spp connect disconnect!!!");
            LOG_I("[bt_app]bd addr %02x:%02x:%02x:%02x:%02x:%02x,chl %d\r\n", disc_ind->mac.addr[0], disc_ind->mac.addr[1], disc_ind->mac.addr[2], disc_ind->mac.addr[3], disc_ind->mac.addr[4], disc_ind->mac.addr[5], disc_ind->srv_chl);
            bt_interface_dump_all_spp_connection_info();
        }
        break;
        case BT_NOTIFY_SPP_DATA_IND:
        {
            bt_notify_spp_data_ind_t *data_info = (bt_notify_spp_data_ind_t *)data;
            if (data_info->payload_len <= 20)
            {

                LOG_I("[SPP addr:%02x:%02x:xx:xx:%02x:%02x-channel:%d RX]:",
                      data_info->mac.addr[0], data_info->mac.addr[1],
                      data_info->mac.addr[4], data_info->mac.addr[5],
                      data_info->srv_chl);
                for (int i = 0; i < data_info->payload_len; i++)
                {
                    LOG_I("0x%x ", data_info->payload[i]);
                }
            }

            //Customers need to implement the corresponding flow control logic!!!
            //This value needs to be modified in the customer's processing
            //is_flow_ctrl equal to 0 means that flow control is not enabled
            //Is_flow_ctrl is not equal to 0, which means that flow control is enabled,
            //but the customer needs to call bt_interface_spp_srv_data_rsp_ext() himself to reply the received data
            *(data_info->is_flow_ctrl) = 0;
        }
        break;
        case BT_NOTIFY_SPP_DATA_CFM:
        {
            bt_notify_spp_data_cfm_t *data_cfm = (bt_notify_spp_data_cfm_t *)data;
            // LOG_I("BT_NOTIFY_SPP_DATA_CFM");
        }
        break;
        }
    }

    return 0;
}

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "root"
int mnt_init(void)
{
    //TODO: how to get base address
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000), FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)), FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        LOG_I("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        LOG_I("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            LOG_I("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                LOG_I("mount fs on flash success\n");
            else
                LOG_I("mount to fs on flash fail\n");
        }
        else
            LOG_I("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

//!Customers need to implement this function to add custom uuid
void bt_spp_srv_add_uuid_list(void)
{
    U8 uuid_1[] =
    {
        0x30, 0x01
    };

    U8 uuid_2[] =
    {
        0x30, 0x02
    };

    U8 uuid_3[] =
    {
        0x30, 0x03
    };

    spp_add_uuid_list_node(uuid_1, sizeof(uuid_1), "aaaa");
    spp_add_uuid_list_node(uuid_2, sizeof(uuid_2), "bbbb");
    spp_add_uuid_list_node(uuid_3, sizeof(uuid_3), "cccc");
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    //!zhengyu:first,set the profile which can connect
    bt_cm_set_profile_target(0, BT_SLAVE_ROLE, 0);
    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);
    sifli_ble_enable();
    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
    }
    return 0;
}


static void help(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##               SPP command                        ##\n");
    printf("##   disc: disconnect special spp connection        ##\n");
    printf("##   disc_all: disconnect all spp connection        ##\n");
    printf("##   send_data: send test data to peer device       ##\n");
    printf("##   send_file: send file to peer device            ##\n");
    printf("##   dump: dump all spp connection information      ##\n");
    printf("##   search: search remote device spp service       ##\n");
    printf("##   connect: connect remote device spp service     ##\n");
    printf("##   through_put: through put test                  ##\n");
    printf("##                Send vast data (least 1024byte)   ##\n");
    printf("##                                                  ##\n");
    printf("##   for example:input spp dis_all                  ##\n");
    printf("##               disconnect all spp connection      ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

__ROM_USED void spp(int argc, char **argv)
{
    if (argc < 2)
        help();
    else
    {
        if (strcmp(argv[1], "disc") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            uint8_t srv_chl = atoi(argv[3]);
            bt_interface_dis_spp_by_addr_and_chl_ext((bt_notify_device_mac_t *)&mac, srv_chl);
        }
        else if (strcmp(argv[1], "disc_all") == 0)
        {
            bt_interface_dis_spp_all();
        }
        else if (strcmp(argv[1], "send_data") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            uint8_t srv_chl = atoi(argv[3]);
            char *test_str = "This is spp test!!!";
            U8 len = (U8)bstrlen(test_str);
            U8 *data = bmalloc(len);
            bmemcpy(data, test_str, len);

            bt_interface_spp_send_data_ext(data, len, (bt_notify_device_mac_t *)&mac, srv_chl);
        }
        else if (strcmp(argv[1], "send_file") == 0)
        {
            bd_addr_t mac;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            uint8_t srv_chl = atoi(argv[3]);
            char *file_name = argv[4];
            bt_interface_spp_srv_send_file((bt_notify_device_mac_t *)&mac, srv_chl, file_name);
        }
        else if (strcmp(argv[1], "dump") == 0)
        {
            bt_interface_dump_all_spp_connection_info();
        }
        else if (strcmp(argv[1], "search") == 0)
        {
            if (CFG_MAX_BT_ACL_NUM == bt_cm_get_conn_num())
            {
                LOG_I("Already connect a device!!!\n");
            }
            else
            {
                bd_addr_t mac;
                uint8_t uuid_len = 0;
                bt_addr_convert_from_string_to_general(argv[2], &mac);
                uuid_len = atoi(argv[3]);
                uint8_t uuid[16];
                char *uuid_str = argv[4];

                bt_convert_from_string_to_uuid_array(uuid_str, uuid, uuid_len);

                gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
                bt_interface_spp_client_sdp_search_req((bt_notify_device_mac_t *)&mac, uuid, uuid_len);
            }
        }
        else if (strcmp(argv[1], "connect") == 0)
        {
            bd_addr_t mac;
            uint8_t uuid_len = 0;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            uuid_len = atoi(argv[3]);
            uint8_t uuid[16];
            char *uuid_str = argv[4];

            bt_convert_from_string_to_uuid_array(uuid_str, uuid, uuid_len);

            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
            bt_interface_spp_client_conn_req((bt_notify_device_mac_t *)&mac, uuid, uuid_len);
        }
        else if (strcmp(argv[1], "through_put") == 0)
        {
            bd_addr_t mac;
            BTS2S_BD_ADDR bd_addr;
            uint32_t rand_len = 0;
            bt_addr_convert_from_string_to_general(argv[2], &mac);
            uint8_t srv_chl = atoi(argv[3]);
            rand_len = atoi(argv[4]);
            char *test_str = "This is spp through-put test!!!";
            U8 len = (U8)bstrlen(test_str);
            U8 *data = bmalloc(len);
            bmemcpy(data, test_str, len);

            bt_err_t ret = bt_interface_spp_prepare_send_rand_data(rand_len - len, (bt_notify_device_mac_t *)&mac, srv_chl);
            if (!ret)
                bt_interface_spp_send_data_ext(data, len, (bt_notify_device_mac_t *)&mac, srv_chl);
            else
                LOG_I("can't do a test!!!!\n");
        }
    }
}
MSH_CMD_EXPORT(spp, spp command)
