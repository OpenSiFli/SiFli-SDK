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
#include "bf0_sibles_advertising.h"
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


SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}

#define DEFAULT_LOCAL_NAME "SIFLI_APP"
/* Enable advertise via advertising service. */
static void ble_app_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf(local_name, 31, "SIFLI_APP-%x-%x-%x-%x-%x-%x", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    //para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed. Due to name is too long to put adv data, put it to rsp data.*/
    para.rsp_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.rsp_data.completed_name->name, local_name, para.rsp_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_advertising_event;

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}
int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    //app_env_t *env = ble_app_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (g_bt_app_mb)
            rt_mb_send(g_bt_app_mb, BLE_POWER_ON_IND);
        break;
    }

    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);

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
    LOG_I("now only 58x is support 3sco!\r\n");
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    bt_interface_acl_accept_role_set(HCI_SLAVE);
    sifli_ble_enable();


    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            ble_app_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return 0;
}

int cmd_diss(int argc, char *argv[])
{

    if (argc > 1)
    {
        if (strcmp(argv[1], "adv_start") == 0)
        {
            sibles_advertising_start(g_app_advertising_context);
        }
        else if (strcmp(argv[1], "adv_stop") == 0)
        {
            sibles_advertising_stop(g_app_advertising_context);
        }

    }

    return 0;
}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(cmd_diss, My device information service.);
#endif