/*
 * SPDX-FileCopyrightText: 2024-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */



/** Mount file system if using NAND, as BT NVDS is save in file*/
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
#include "bts2_app_interface.h"
#include "ulog.h"

typedef struct
{
    uint16_t            type;
    uint16_t            event_id;
    uint16_t            data_len;
    uint8_t             *data;
} bt_app_notify_data_t;

static rt_mq_t g_bt_pbap_service_queue;
static struct rt_thread g_bt_pbap_service_thread;
static uint8_t g_bt_pbap_service_thread_stack[2048];
static bt_notify_pbap_phone_repository_t g_pbap_repository = BT_PBAP_REPOSITORY_CURRENT;
static U8 g_pbap_phonebook = PBAP_PB;

static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    rt_err_t err;
    bt_app_notify_data_t *msg;
    msg = rt_malloc(sizeof(bt_app_notify_data_t) + data_len);
    RT_ASSERT(msg);
    msg->type = type;
    msg->event_id = event_id;
    msg->data_len = data_len;
    msg->data = (uint8_t *)((uint8_t *)msg + sizeof(bt_app_notify_data_t));
    if (NULL != data)
    {
        memcpy(msg->data, data, data_len);
    }
    else if (data_len != 0)
    {
        rt_free(msg);
        return -RT_ERROR;
    }
    err = rt_mq_send(g_bt_pbap_service_queue, &msg, sizeof(msg));
    if (err != RT_EOK)
    {
        rt_free(msg);
    }

    return err;
}

static int bt_pbap_service_notify_event_handle(bt_app_notify_data_t *msg)
{
    if (msg == RT_NULL || msg->data == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (msg->type != BT_NOTIFY_PBAP)
    {
        LOG_D("Ignore BT event type=%u event=%u", msg->type, msg->event_id);
        return 0;
    }

    switch (msg->event_id)
    {
    case BT_NOTIFY_PBAP_PROFILE_CONNECTED:
    case BT_NOTIFY_PBAP_PROFILE_DISCONNECTED:
        if (msg->data_len >= sizeof(bt_notify_profile_state_info_t))
        {
            bt_notify_profile_state_info_t *info = (bt_notify_profile_state_info_t *)msg->data;
            LOG_I("PBAP %s result=%u addr=%02x:%02x:%02x:%02x:%02x:%02x",
                  msg->event_id == BT_NOTIFY_PBAP_PROFILE_CONNECTED ? "connected" : "disconnected",
                  info->res, info->mac.addr[0], info->mac.addr[1], info->mac.addr[2],
                  info->mac.addr[3], info->mac.addr[4], info->mac.addr[5]);
        }
        break;

    case BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND:
        if (msg->data_len >= sizeof(bt_notify_pbap_vcard_listing_item_t))
        {
            bt_notify_pbap_vcard_listing_item_t *item = (bt_notify_pbap_vcard_listing_item_t *)msg->data;
            uint8_t name_len = item->name_len;
            if (name_len > BT_NOTIFY_PBAP_MAX_VCARD_CONTACT_NAME_SIZE)
            {
                name_len = BT_NOTIFY_PBAP_MAX_VCARD_CONTACT_NAME_SIZE;
            }
            LOG_I("PBAP list item: %.*s", name_len, item->vcard_name);
        }
        break;

    case BT_NOTIFY_PBAP_VCARD_LIST_CMPL:
    case BT_NOTIFY_PBAP_PULL_VCARD_CMPL:
        if (msg->data_len >= sizeof(uint8_t))
        {
            LOG_I("PBAP %s complete, result=%u",
                  msg->event_id == BT_NOTIFY_PBAP_VCARD_LIST_CMPL ? "vCard list" : "vCard pull",
                  msg->data[0]);
        }
        break;

    case BT_NOTIFY_PBAP_PULL_PB_CMPL:
    case BT_NOTIFY_PBAP_SET_PATH_CFM:
        if (msg->data_len >= sizeof(bt_notify_pbap_vcard_item_cmpl_t))
        {
            bt_notify_pbap_vcard_item_cmpl_t *result = (bt_notify_pbap_vcard_item_cmpl_t *)msg->data;
            LOG_I("PBAP %s complete: phonebook=%u result=%u",
                  msg->event_id == BT_NOTIFY_PBAP_PULL_PB_CMPL ? "phonebook pull" : "set path",
                  result->phone_book, result->res);
        }
        break;

    case BT_NOTIFY_PBAP_VCARD_ITEM_IND:
        if (msg->data_len >= sizeof(bt_notify_pbap_vcard_item_t))
        {
            bt_notify_pbap_vcard_item_t *item = (bt_notify_pbap_vcard_item_t *)msg->data;
            uint8_t name_len = item->vcard_name_len;
            uint8_t number_len = item->vcard_number_len;
            if (name_len >= BT_NOTIFY_PBAP_MAX_VCARD_SIZE)
            {
                name_len = BT_NOTIFY_PBAP_MAX_VCARD_SIZE - 1;
            }
            if (number_len >= BT_NOTIFY_PBAP_MAX_VCARD_SIZE)
            {
                number_len = BT_NOTIFY_PBAP_MAX_VCARD_SIZE - 1;
            }
            LOG_I("PBAP vCard item: name=%.*s number=%.*s", name_len, item->vcard_name,
                  number_len, item->vcard_number);
        }
        break;

    case BT_NOTIFY_PBAP_VCARD_TOTAL_NUM_IND:
        if (msg->data_len >= sizeof(bt_notify_pbap_vcard_num_check_t))
        {
            bt_notify_pbap_vcard_num_check_t *num = (bt_notify_pbap_vcard_num_check_t *)msg->data;
            LOG_I("PBAP phonebook=%u total contacts=%u", num->phone_book, num->total_num);
        }
        break;

    default:
        LOG_W("Unhandled PBAP event=%u len=%u", msg->event_id, msg->data_len);
        break;
    }

    return 0;
}

static void bt_pbap_service_thread_entry(void *param)
{
    bt_app_notify_data_t *msg;
    rt_err_t err;

    while (1)
    {
        err = rt_mq_recv(g_bt_pbap_service_queue, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (err == RT_EOK && msg != RT_NULL)
        {
            bt_pbap_service_notify_event_handle(msg);
            rt_free(msg);
        }
    }
}

static rt_err_t pbap_service_thread_init(void)
{
    rt_err_t err;

    g_bt_pbap_service_queue = rt_mq_create("pbap_svc", sizeof(void *), 30, RT_IPC_FLAG_FIFO);
    if (g_bt_pbap_service_queue == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    err = rt_thread_init(&g_bt_pbap_service_thread, "pbap_svc", bt_pbap_service_thread_entry, RT_NULL,
                         g_bt_pbap_service_thread_stack, sizeof(g_bt_pbap_service_thread_stack),
                         RT_THREAD_PRIORITY_MIDDLE, 4);
    if (err != RT_EOK)
    {
        rt_mq_delete(g_bt_pbap_service_queue);
        g_bt_pbap_service_queue = RT_NULL;
        return err;
    }

    return rt_thread_startup(&g_bt_pbap_service_thread);
}


int main(void)
{
    /* ls files in root. */
    extern void ls(const char *name);
    ls("/");

    if (pbap_service_thread_init() != RT_EOK)
    {
        LOG_E("PBAP service thread init failed");
        return -RT_ERROR;
    }

    bt_interface_register_bt_event_notify_callback(bt_app_interface_event_handle);

    // Start BT/BLE stack/profile.
#ifdef ZBT
    bt_enable(NULL);
#else
    sifli_ble_enable();
#endif

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}

static void help(void)
{
    rt_kprintf("PBAP commands:\n");
    rt_kprintf("  pbap_cmd connect <XX:XX:XX:XX:XX:XX> [auth:0|1]\n");
    rt_kprintf("  pbap_cmd disconnect <XX:XX:XX:XX:XX:XX>\n");
    rt_kprintf("  pbap_cmd set <local|sim1> <pb|ich|och|mch|cch>\n");
    rt_kprintf("  pbap_cmd pull_pb [max_count]\n");
    rt_kprintf("  pbap_cmd list <phone_number>\n");
    rt_kprintf("  pbap_cmd vcard <handle>\n");
    rt_kprintf("  PBAP operations are asynchronous; check PBAP notify logs for results.\n");
}

static int pbap_parse_addr(const char *str, BTS2S_BD_ADDR *bd)
{
    bd_addr_t addr = {0};

    if (str == RT_NULL || bd == RT_NULL ||
            bt_addr_convert_from_string_to_general((char *)str, &addr) != BD_ADDR_LEN)
    {
        return -RT_ERROR;
    }

    bd->nap = (U16)((addr.addr[0] << 8) | addr.addr[1]);
    bd->uap = addr.addr[2];
    bd->lap = (U24)((addr.addr[3] << 16) | (addr.addr[4] << 8) | addr.addr[5]);
    return RT_EOK;
}

static int pbap_parse_repository(const char *str, bt_notify_pbap_phone_repository_t *repo)
{
    if (rt_strcmp(str, "local") == 0)
    {
        *repo = BT_PBAP_REPOSITORY_LOCAL;
    }
    else if (rt_strcmp(str, "sim1") == 0)
    {
        *repo = BT_PBAP_REPOSITORY_SIM1;
    }
    else if (rt_strcmp(str, "current") == 0)
    {
        *repo = BT_PBAP_REPOSITORY_CURRENT;
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

static int pbap_parse_phonebook(const char *str, U8 *phonebook)
{
    if (rt_strcmp(str, "pb") == 0)
    {
        *phonebook = PBAP_PB;
    }
    else if (rt_strcmp(str, "ich") == 0)
    {
        *phonebook = PBAP_ICH;
    }
    else if (rt_strcmp(str, "och") == 0)
    {
        *phonebook = PBAP_OCH;
    }
    else if (rt_strcmp(str, "mch") == 0)
    {
        *phonebook = PBAP_MCH;
    }
    else if (rt_strcmp(str, "cch") == 0)
    {
        *phonebook = PBAP_CCH;
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

__ROM_USED void pbap_cmd(int argc, char **argv)
{
    bt_err_t ret;
    const char *cmd;

    if (argc < 2 || rt_strcmp(argv[1], "help") == 0)
    {
        help();
        return;
    }

    cmd = argv[1];
    if (rt_strcmp(cmd, "connect") == 0)
    {
        bd_addr_t mac;
        bt_addr_convert_from_string_to_general(argv[2], &mac);
        bt_interface_conn_ext((unsigned char *)&mac, BT_PROFILE_PBAP);
    }
    else if (rt_strcmp(cmd, "disconnect") == 0)
    {
        bd_addr_t mac;
        bt_addr_convert_from_string_to_general(argv[2], &mac);
        bt_interface_disc_ext((unsigned char *)&mac, BT_PROFILE_PBAP);
    }
    else if (rt_strcmp(cmd, "set") == 0)
    {
        bt_notify_pbap_phone_repository_t repo;
        U8 phonebook;
        if (argc != 4 || pbap_parse_repository(argv[2], &repo) != RT_EOK ||
                pbap_parse_phonebook(argv[3], &phonebook) != RT_EOK)
        {
            help();
            return;
        }
        ret = bt_interface_pbap_client_set_pb(repo, phonebook);
        if (ret == BT_EOK)
        {
            g_pbap_repository = repo;
            g_pbap_phonebook = phonebook;
        }
    }
    else if (rt_strcmp(cmd, "pull_pb") == 0)
    {
        char *end;
        unsigned long max_count = 0xffff;
        if (argc > 3)
        {
            help();
            return;
        }
        if (argc == 3)
        {
            max_count = strtoul(argv[2], &end, 10);
            if (*argv[2] == '\0' || *end != '\0' || max_count > 0xffff)
            {
                help();
                return;
            }
        }
        ret = bt_interface_pbap_client_pull_pb(g_pbap_repository, g_pbap_phonebook, (U16)max_count);
    }
    else if (rt_strcmp(cmd, "list") == 0)
    {
        size_t len;
        if (argc != 3)
        {
            help();
            return;
        }
        len = rt_strlen(argv[2]);
        if (len == 0 || len > 0xffff)
        {
            rt_kprintf("Invalid phone number.\n");
            return;
        }
        ret = bt_interface_pbap_client_get_name_by_number(argv[2], (U16)len);
    }
    else if (rt_strcmp(cmd, "vcard") == 0)
    {
        if (argc != 3)
        {
            help();
            return;
        }
        ret = bt_interface_pbap_client_pull_vcard(argv[2]);
    }
    else
    {
        help();
        return;
    }

    rt_kprintf("PBAP command '%s' returned %d\n", cmd, ret);
}
MSH_CMD_EXPORT(pbap_cmd, pbap_cmd command)