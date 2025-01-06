
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

#define BT_APP_CONNECT_PAN  1
#define PAN_TIMER_MS        3000

typedef struct
{
    BOOL bt_connected;
    BTS2S_BD_ADDR bd_addr;
    rt_timer_t pan_connect_timer;
} bt_app_t;
static bt_app_t g_bt_app_env;
static rt_mailbox_t g_bt_app_mb;

void bt_app_connect_pan_timeout_handle(void *parameter)
{
    if ((g_bt_app_mb != NULL) && (g_bt_app_env.bt_connected))
        rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN);
    return;
}

int bt_app_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    int pan_conn = 0;

    if (type == BTS2M_HCI_CMD && event_id == DM_ACL_DISC_IND)
    {
        BTS2S_DM_ACL_DISC_IND *ind = (BTS2S_DM_ACL_DISC_IND *)msg;
        rt_kprintf("[bt_app]link dis-connected %x %d\r\n", ind->hdl, ind->reason);
        g_bt_app_env.bt_connected = FALSE;
        g_bt_app_env.bd_addr.lap = CFG_BD_LAP;
        g_bt_app_env.bd_addr.nap = CFG_BD_NAP;
        g_bt_app_env.bd_addr.uap = CFG_BD_UAP;
        if (g_bt_app_env.pan_connect_timer)
            rt_timer_stop(g_bt_app_env.pan_connect_timer);
    }
    else if (type == BTS2M_GAP && event_id == BTS2MU_GAP_ENCRYPTION_IND)
    {
        // Reconnect auth success
        BTS2S_GAP_ENCRYPTION_IND *ind = NULL;
        ind = (BTS2S_GAP_ENCRYPTION_IND *)msg;
        pan_conn = 1;
        memcpy(&g_bt_app_env.bd_addr, &ind->bd, sizeof(BTS2S_BD_ADDR));
    }
    else if (type == BTS2M_SC && event_id == BTS2MU_SC_PAIR_IND)
    {
        // Pair for the first time
        BTS2S_SC_PAIR_IND *ind = NULL;
        ind = (BTS2S_SC_PAIR_IND *)msg;
        pan_conn = 1;
        memcpy(&g_bt_app_env.bd_addr, &ind->bd, sizeof(BTS2S_BD_ADDR));
    }

    if (pan_conn)
    {
        rt_kprintf("[bt_app]bd addr %x-%x-%x\n", g_bt_app_env.bd_addr.nap, g_bt_app_env.bd_addr.uap, g_bt_app_env.bd_addr.lap);
        g_bt_app_env.bt_connected = TRUE;
        // Trigger PAN connection after PAN_TIMER_MS period to avoid SDP confliction.
        if (!g_bt_app_env.pan_connect_timer)
            g_bt_app_env.pan_connect_timer = rt_timer_create("connect_pan", bt_app_connect_pan_timeout_handle, (void *)&g_bt_app_env,
                                             rt_tick_from_millisecond(PAN_TIMER_MS), RT_TIMER_FLAG_SOFT_TIMER);
        else
            rt_timer_stop(g_bt_app_env.pan_connect_timer);
        rt_timer_start(g_bt_app_env.pan_connect_timer);
    }
    return 0;
}
BT_EVENT_REGISTER(bt_app_event_hdl, NULL);


static void connect_last(void)
{
    bt_app_connect_pan_timeout_handle(NULL);
}
MSH_CMD_EXPORT(connect_last, Connect PAN to last paired device);


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

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
    bt_cm_set_profile_target(BT_CM_PAN, BT_SLAVE_ROLE, 1);
    sifli_ble_enable();
    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BT_APP_CONNECT_PAN)
        {
            rt_kprintf("[bt_app]prepare connect pan, addr %x-%x-%x\r\n", g_bt_app_env.bd_addr.nap, g_bt_app_env.bd_addr.uap, g_bt_app_env.bd_addr.lap);
            if (g_bt_app_env.bt_connected)
                bt_pan_conn_by_addr(&g_bt_app_env.bd_addr);
        }
    }
    return 0;
}

