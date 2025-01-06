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




typedef struct
{
    U16 connected_profile;
    BOOL bt_connected;
    BTS2S_BD_ADDR bd_addr;
} bt_app_t;

static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;


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
        break;
    }

    default:
        break;
    }

    return 0;
}

int bt_app_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    if (type == BTS2M_HCI_CMD)
    {
        bt_app_hci_event_handler(event_id, msg);
    }
    else if (type == BTS2M_GAP)
    {
        bt_app_gap_event_handler(event_id, msg);
    }
    return 0;

}
BT_EVENT_REGISTER(bt_app_event_hdl, NULL);


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


BOOL bt_spp_srv_test_enable(void)
{
    return TRUE;
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
    sifli_ble_enable();
    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
    }
    return 0;
}
