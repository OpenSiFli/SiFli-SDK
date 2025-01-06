/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include "bf0_hal.h"

/* Common functions for RT-Thread based platform -----------------------------------------------*/
#include "drv_io.h"
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
#include "bf0_sibles.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "ulog.h"

#ifdef ZBT
    #include "zephyr/bluetooth/bluetooth.h"
#endif

#define BT_APP_READY  1


typedef struct
{
    U16             connected_profile;
    BTS2S_BD_ADDR   bd_addr;
} bt_app_t;

static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;


/** Mount file system if using NAND, as BT NVDS is save in file*/
#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS) && !defined(ZBT)
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
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif


static int bt_app_a2dp_event_handler(uint16_t event_id, uint8_t *msg)
{
    uint8_t bd_addr[BD_ADDR_LEN];

    switch (event_id)
    {
    case BTS2MU_AV_CONN_IND:
    {
        BTS2S_AV_CONN_IND *ind = (BTS2S_AV_CONN_IND *)msg;
        LOG_I("[bt_app]a2dp connect ind %d\r\n", ind->conn_id);
        g_bt_app_env.connected_profile |= BT_CM_A2DP;
        break;
    }
    case BTS2MU_AV_CONN_CFM:
    {
        BTS2S_AV_CONN_CFM *ind = (BTS2S_AV_CONN_CFM *)msg;
        LOG_I("[bt_app]a2dp connect cfm %d res %d\r\n", ind->conn_id, ind->res);
        if (ind->res == AV_ACPT)
        {
            g_bt_app_env.connected_profile |= BT_CM_A2DP;
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

static int bt_app_avrcp_event_handler(uint16_t event_id, uint8_t *msg)
{
    uint8_t bd_addr[BD_ADDR_LEN];

    switch (event_id)
    {
    case BTS2MU_AVRCP_CONN_CFM:
    {
        BTS2S_AVRCP_CONN_CFM *ind = (BTS2S_AVRCP_CONN_CFM *)msg;
        if (ind->res == BTS2_SUCC)
        {
            LOG_I("[bt_app]avrcp connect successed \n");
            g_bt_app_env.connected_profile |= BT_CM_AVRCP;
        }
        break;
    }
    case BTS2MU_AVRCP_CONN_IND:
    {
        BTS2S_AVRCP_CONN_IND *ind = (BTS2S_AVRCP_CONN_IND *)msg;
        LOG_I("[bt_app]avrcp indicate to connect with remote device\n");
        g_bt_app_env.connected_profile |= BT_CM_AVRCP;
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

static int bt_app_pan_event_handler(uint16_t event_id, uint8_t *msg)
{
    switch (event_id)
    {
    case BTS2MU_PAN_CONN_IND:
    {
        BTS2S_PAN_CONN_IND *ind = (BTS2S_PAN_CONN_IND *)msg;
        if (ind->res == BTS2_SUCC)
        {
            LOG_I("[bt_app]pan connect successed \n");
            g_bt_app_env.connected_profile |= BT_CM_PAN;
        }
        break;
    }
    case BTS2MU_PAN_DISC_IND:
    {
        BTS2S_PAN_DISC_IND *ind = (BTS2S_PAN_DISC_IND *)msg;
        LOG_I("[bt_app]pan disconnect with remote device\n");
        g_bt_app_env.connected_profile &= ~BT_CM_PAN;
        break;
    }
    default:
        break;
    }
    return 0;
}

int bt_app_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    switch (type)
    {
    case BTS2M_AV:
        bt_app_a2dp_event_handler(event_id, msg);
        break;
    case BTS2M_AVRCP:
        bt_app_avrcp_event_handler(event_id, msg);
        break;
    case BTS2M_PAN:
        bt_app_pan_event_handler(event_id, msg);
        break;
    case BTS2M_GAP:
        if (event_id == BTS2MU_GAP_RD_LOCAL_NAME_CFM) /*Last initialization BT command*/
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
        break;
    default:
        LOG_D("Unknown message, type=%d, event_id=%d\n", type, event_id);
        break;
    }
    return 0;
}

BT_EVENT_REGISTER(bt_app_event_hdl, NULL);


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
#ifdef BT_DEVICE_NAME
    static const char *local_name = BT_DEVICE_NAME;
#else
    static const char *local_name = "sifli_music";
#endif

int main(void)
{
    uint32_t value;

    //__asm("B .");
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_bt_app_mb);

    bt_cm_set_profile_target(BT_CM_PAN, BT_SLAVE_ROLE, 1);

    // Start BT/BLE stack/profile.
#ifdef ZBT
    bt_enable(NULL);
#else
    sifli_ble_enable();
#endif

    // Wait for stack/profile ready.
    if (RT_EOK == rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, 5000) && value == BT_APP_READY)
        LOG_I("BT/BLE stack and profile ready");
    else
        LOG_I("BT/BLE stack and profile init failed");

    // Update Bluetooth name
    bt_interface_set_local_name(strlen(local_name), (void *)local_name);

    while (1)
    {
        rt_thread_mdelay(15000);
    }
    return 0;
}

static void help(void)
{
}

__ROM_USED void music(int argc, char **argv)
{
    if (argc < 2)
        help();
    else
    {
        if (strcmp(argv[1], "c") == 0)
        {
            bt_cm_delete_bonded_devs();
        }
    }
}
MSH_CMD_EXPORT(music, music command)



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

