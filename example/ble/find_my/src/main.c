/*
 * SPDX-FileCopyrightText: 2025-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include <dfs_posix.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"
#include "bf0_ble_tpss.h"

#ifdef PKG_USING_FMNA
    #include "fmna_adv.h"
    #include "fmna_gatt_platform.h"
    #include "fmna_version.h"

    #include "fmna_ble_stack_platform.h"
    #include "fmna_constants.h"
    #include "fmna_state_machine.h"
    #include "fmna_connection.h"
    #include "fmna_gatt.h"
    #include "fmna_crypto.h"
    #include "fmna_sound_platform.h"
    #include "fmna_peer_manager.h"
    #include "fmna_motion_detection.h"
    #include "fmna_nfc.h"
    #include "fmna_malloc_platform.h"
#endif

#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "fmna-main"

#include "log.h"

#ifdef BT_FINSH
    #include "bts2_app_interface.h"
#endif

#define BLE_APP_TIMEOUT_INTERVAL (5000)

#define LINE_COUNT         3
#define HEX_CHARS_PER_LINE 2048
#define BYTES_PER_LINE     1024
#define FILE_PATH          "/token.txt"

uint8_t new_token[1024] = {0};
uint8_t new_uuid[16] = {0};
uint8_t new_sn[16] = {0};

int8_t parse_token_file(void);

typedef struct
{
    uint8_t is_power_on;
    uint8_t conn_idx;
    struct
    {
        bd_addr_t peer_addr;
        uint16_t conn_interval;
        uint8_t peer_addr_type;
    } conn_para;
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

#ifndef NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE
ble_common_update_type_t ble_request_public_address(bd_addr_t *addr)
{
    int ret = bt_mac_addr_generate_via_uid_v2(addr);

    if (ret != 0)
    {
        LOG_W("generate mac addres failed %d", ret);
        return BLE_UPDATE_NO_UPDATE;
    }

    return BLE_UPDATE_ONCE;
}
#endif // NVDS_AUTO_UPDATE_MAC_ADDRESS_ENABLE

uint8_t sibles_advertising_disc_mode_get()
{
    return GAPM_ADV_MODE_BEACON;
}

void set_sn()
{
    uint8_t length = 16;
    uint8_t sn[length];

    memset(sn, 0, length);
    for (int i = 0; i < length; i++)
    {
        sn[i] = i;
    }

    sn[0] = 0x3C;
    sn[1] = 0xAA;
    sn[2] = 0xFE;
    sn[3] = 0x09;
    sn[4] = 0x00;

    fmna_connection_platform_set_serial_number(sn, length);
}

void set_file_sn()
{
    uint8_t length = 16;
    uint8_t sn[length];

    memset(sn, 0, length);
    memcpy(sn, new_sn, length);

    LOG_I("using file sn");
    fmna_connection_platform_set_serial_number(sn, length);
}

int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
#if defined(BT_FINSH) && defined(SF32LB52X_58)
    bt_interface_acl_accept_role_set(0);
    bt_interface_set_linkpolicy(1, 1);
#endif

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            uint8_t tx_power = 0;
#ifdef SF32LB55X
            tx_power = BLE_TX_POWER_VAL;
#else
            tx_power = BT_TX_POWER_VAL;
#endif
            ble_tx_power_service_server_init(NULL, tx_power);

#ifdef PKG_USING_FMNA
            memset(new_sn, 0, 16);

            set_sn();
            int r = parse_token_file();
            if (r == 0)
            {
                set_file_sn();
            }

            device_info_init();
            fmna_version_init();

            // Initialize platform-dependent modules.
            fmna_gatt_services_init();
            fmna_adv_info_init();
            fmna_sound_platform_init();

            LOG_I("Compiled: %s %s", __DATE__, __TIME__);
            LOG_I("Starting Find My Accessory");

            fmna_connection_init();
            fmna_crypto_init();
            fmna_motion_detection_init();

            // fmna_nfc_init();
#ifdef USE_UARP
            fmna_uarp_control_point_init();
#endif
            // HAL_sw_breakpoint();
            fmna_state_machine_init();
            fmna_log_mfi_token_help();
            fmna_malloc_dump();
#endif
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}

static void ble_app_update_conn_param(uint8_t conn_idx, uint16_t inv_max, uint16_t inv_min, uint16_t timeout)
{
    ble_gap_update_conn_param(BLE_GAP_CREATE_UPDATE_CONN_PARA(conn_idx, inv_min, inv_max, 0, timeout));
}

int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (env->mb_handle)
            rt_mb_send(env->mb_handle, BLE_POWER_ON_IND);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_para.conn_interval = ind->con_interval;
        env->conn_para.peer_addr_type = ind->peer_addr_type;
        env->conn_para.peer_addr = ind->peer_addr;
        if (ind->role == 0)
            LOG_E("Peripheral should be slave!!!");

        LOG_I("Peer device(%x-%x-%x-%x-%x-%x) connected", env->conn_para.peer_addr.addr[5],
              env->conn_para.peer_addr.addr[4],
              env->conn_para.peer_addr.addr[3],
              env->conn_para.peer_addr.addr[2],
              env->conn_para.peer_addr.addr[1],
              env->conn_para.peer_addr.addr[0]);
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        env->conn_para.conn_interval = ind->con_interval;
        LOG_I("Updated connection interval :%d", ind->con_interval);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        LOG_I("BLE_GAP_DISCONNECTED_IND(%d)", ind->reason);
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);

uint8_t ble_app_dis_enable()
{
    return 1;
}

#ifdef SF32LB52X_58
uint16_t g_em_offset[HAL_LCPU_CONFIG_EM_BUF_MAX_NUM] =
{
    0x178, 0x178, 0x740, 0x7A0, 0x810, 0x880, 0xA00, 0xBB0, 0xD48,
    0x133C, 0x13A4, 0x19BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC,
    0x21BC, 0x21BC, 0x263C, 0x265C, 0x2734, 0x2784, 0x28D4, 0x28E8, 0x28FC,
    0x29EC, 0x29FC, 0x2BBC, 0x2BD8, 0x3BE8, 0x5804, 0x5804, 0x5804
};

void lcpu_rom_config(void)
{
    hal_lcpu_bluetooth_em_config_t em_offset;
    memcpy((void *)em_offset.em_buf, (void *)g_em_offset, HAL_LCPU_CONFIG_EM_BUF_MAX_NUM * 2);
    em_offset.is_valid = 1;
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_EM_BUF, &em_offset, sizeof(hal_lcpu_bluetooth_em_config_t));

    hal_lcpu_bluetooth_act_configt_t act_cfg;
    act_cfg.ble_max_act = 6;
    act_cfg.ble_max_iso = 0;
    act_cfg.ble_max_ral = 3;
    act_cfg.bt_max_acl = 7;
    act_cfg.bt_max_sco = 0;
    act_cfg.bit_valid = CO_BIT(0) | CO_BIT(1) | CO_BIT(2) | CO_BIT(3) | CO_BIT(4);
    HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACT_CFG, &act_cfg, sizeof(hal_lcpu_bluetooth_act_configt_t));
}
#endif


void fmna_add_key()
{
#ifdef PKG_USING_FMNA
    static uint8_t current_token[195] =
    {
        // replace with your token here
    };
    uint16_t d_len = 195;

    static uint8_t uuid[SOFTWARE_AUTH_UUID_BLEN] = {0x50, 0xba, 0x89, 0xa8, 0x07, 0x44, 0x4f, 0x40, 0xb7, 0x60, 0x17, 0xa5, 0x3a, 0xf8, 0x1e, 0x87};


    uint8_t d_token[1024];
    memset(d_token, 0, 1024);
    memcpy(d_token, current_token, d_len);

    fmna_connection_update_token(d_token, 1024);
    fmna_connection_update_uuid(uuid, SOFTWARE_AUTH_UUID_BLEN);
#endif
}

void fmna_show_key()
{
    static uint8_t t_token[1024];
    static uint8_t t_uuid[16];
    static uint8_t t_sn[16];

    memcpy(t_token, (uint8_t *)fmna_fmna_connection_get_token(), 1024);
    memcpy(t_uuid, (uint8_t *)fmna_fmna_connection_get_uuid(), 16);
    fmna_connection_platform_get_serial_number(t_sn, 16);


    rt_kprintf("token\n");
    for (int i = 0; i < 1024; i++)
    {
        rt_kprintf("%02x", t_token[i]);
    }
    rt_kprintf("\n");

    rt_kprintf("uuid\n");
    for (int i = 0; i < 16; i++)
    {
        rt_kprintf("%02x", t_uuid[i]);
    }
    rt_kprintf("\n");

    rt_kprintf("sn\n");
    for (int i = 0; i < 16; i++)
    {
        rt_kprintf("%02x", t_sn[i]);
    }
    rt_kprintf("\n");
}


/* 辅助函数：将两个十六进制字符转换为一个 uint8 */
static uint8_t hex2byte(const char *hex)
{
    uint8_t byte = 0;
    for (int i = 0; i < 2; i++)
    {
        char c = hex[i];
        byte <<= 4;
        if (c >= '0' && c <= '9') byte |= (c - '0');
        else if (c >= 'a' && c <= 'f') byte |= (c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') byte |= (c - 'A' + 10);
    }
    return byte;
}

int8_t parse_token_file(void)
{
    int ret = -1;


    FILE *fp = NULL;
    // 缓冲区大小：2048字符 + 可能的换行符 + \0
    char *line_buf = (char *)rt_malloc(HEX_CHARS_PER_LINE + 4);
    // 存储解析后的 3 行数据
    uint8_t (*tokens)[BYTES_PER_LINE] = rt_malloc(LINE_COUNT * BYTES_PER_LINE);

    if (!line_buf || !tokens)
    {
        rt_kprintf("Memory alloc failed!\n");
        goto __cleanup;
    }

    fp = fopen(FILE_PATH, "r");
    if (!fp)
    {
        rt_kprintf("Open file failed!\n");
        goto __cleanup;
    }

    for (int row = 0; row < LINE_COUNT; row++)
    {
        /* 1. 读取一行 */
        if (fgets(line_buf, HEX_CHARS_PER_LINE + 4, fp) != NULL)
        {
            /* 2. 解析当前行的前 2048 个字符 */
            for (int i = 0; i < BYTES_PER_LINE; i++)
            {
                tokens[row][i] = hex2byte(&line_buf[i * 2]);
            }
            rt_kprintf("Row %d parsed successfully.\n", row);
        }
        else
        {
            rt_kprintf("Error: Failed to read row %d\n", row);
            break;
        }
    }

    memcpy(new_token, tokens[0], 1024);
    memcpy(new_uuid, tokens[1], 16);
    memcpy(new_sn, tokens[2], 16);

    LOG_HEX("TOKEN", 16, new_token, 16);
    LOG_HEX("uuid", 16, new_uuid, 16);
    LOG_HEX("sn", 16, new_sn, 16);
    ret = 0;

    /* 验证读取结果（打印每行前 2 个字节） */
    // for (int i = 0; i < LINE_COUNT; i++) {
    //     rt_kprintf("Token[%d]: 0x%02x 0x%02x...\n", i, tokens[i][0], tokens[i][1]);
    // }

__cleanup:
    if (fp) fclose(fp);
    if (line_buf) rt_free(line_buf);
    if (tokens) rt_free(tokens); // 实际使用时，根据业务逻辑决定何时释放 tokens
    return ret;
}

void fmna_using_new_key()
{
    LOG_I("fmna using new key");
    uint8_t d_token[1024];
    memset(d_token, 0, 1024);
    memcpy(d_token, new_token, 1024);

    uint8_t d_uuid[16];
    memset(d_uuid, 0, 16);
    memcpy(d_uuid, new_uuid, 16);

    fmna_connection_update_token(d_token, 1024);
    fmna_connection_update_uuid(d_uuid, SOFTWARE_AUTH_UUID_BLEN);
}



int cmd_diss(int argc, char *argv[])
{
    app_env_t *env = ble_app_get_env();

    if (argc > 1)
    {
        if (strcmp(argv[1], "trc") == 0)
        {
            if (strcmp(argv[2], "mode") == 0)
            {
                uint8_t mode = atoi(argv[3]);
                uint32_t mask = atoi(argv[4]);
                sibles_set_trc_cfg(mode, mask);
            }
        }
        else if (0 == strcmp(argv[1], "conn"))
        {
            uint32_t interval;
            app_env_t *env = ble_app_get_env();

            interval = atoi(argv[2]);
            // value = argv * 1.25
            interval = interval * 100 / 125;
            ble_app_update_conn_param(env->conn_idx, interval, interval, 500);
        }
        else if (strcmp(argv[1], "gen_addr") == 0)
        {
            bd_addr_t addr;
            int ret;
            ret = bt_mac_addr_generate_via_uid_v2(&addr);
            LOG_D("ret %d", ret);
        }
        else if (strcmp(argv[1], "add_key") == 0)
        {
            LOG_I("fmna add key");
            fmna_add_key();
        }
        else if (strcmp(argv[1], "get_key") == 0)
        {
            fmna_show_key();
        }
        else if (strcmp(argv[1], "parse_key_file") == 0)
        {
            parse_token_file();
        }
        else if (strcmp(argv[1], "update_new_key") == 0)
        {
            fmna_using_new_key();
        }
        else if (strcmp(argv[1], "fmna_unpair") == 0)
        {
            extern void fmna_connection_fmna_unpair(bool force_disconnect);
            fmna_connection_fmna_unpair(false);
        }
        else if (strcmp(argv[1], "fmna_enable") == 0)
        {
            LOG_I("fmna enable");
            fmna_state_machine_enable();
        }
        else if (strcmp(argv[1], "fmna_disable") == 0)
        {
            LOG_I("fmna disable");
            fmna_state_machine_disable();
        }
    }

    return 0;
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(cmd_diss, My device information service.);
#endif
