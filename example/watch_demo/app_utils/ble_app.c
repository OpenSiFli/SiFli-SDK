#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>
#include <string.h>
#include "bf0_hal_hlp.h"
#include "bf0_sibles.h"
#include "bf0_ble_gap.h"

#include "bf0_ble_fmpt.h"
#include "bf0_ble_pxpr.h"
#include "bf0_sibles_serial_trans_service.h"
#include "bf0_ble_bass.h"
#ifdef BSP_BLE_TIMEC
    #include "bf0_ble_tipc.h"
#endif
#include "bf0_sibles_advertising.h"

#define LOG_TAG "ble_app"
#include "log.h"

#define BLE_APP_CATEID 0x4
static rt_mailbox_t g_ble_app_mb;

#define sifli_ble_test_service_uuid {\
    0x53, 0x49, 0x46, 0x4C, \
    0x49, 0x42, 0x4C, 0x45, \
    0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
}

#define sifli_ble_test_tx_uuid {\
    0x53, 0x49, 0x46, 0x4C, \
    0x49, 0x42, 0x4C, 0x45, \
    0x00, 0x01, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
}

#define sifli_ble_test_rx_uuid {\
    0x53, 0x49, 0x46, 0x4C, \
    0x49, 0x42, 0x4C, 0x45, \
    0x00, 0x02, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
}

enum sifli_ble_test_att_list
{
    SIFLI_BLE_TEST_SVC,
    SIFLI_BLE_TEST_TX_CHAR,
    SIFLI_BLE_TEST_TX_VALUE,
    SIFLI_BLE_TEST_RX_CHAR,
    SIFLI_BLE_TEST_RX_VALUE,
    SIFLI_BLE_TEST_RX_CCCD,
    SIFLI_BLE_TEST_ATT_NB
};

#define SERIAL_UUID_16(x) {((uint8_t)(x&0xff)),((uint8_t)(x>>8))}

struct attm_desc_128 sifli_ble_test_att_db[] =
{
    [SIFLI_BLE_TEST_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE), PERM(RD, ENABLE), 0, 0},
    [SIFLI_BLE_TEST_TX_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    [SIFLI_BLE_TEST_TX_VALUE] = {sifli_ble_test_tx_uuid, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 256},
    [SIFLI_BLE_TEST_RX_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    [SIFLI_BLE_TEST_RX_VALUE] = {sifli_ble_test_rx_uuid, PERM(RD, ENABLE) | PERM(NTF, ENABLE) | PERM(IND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 256},
    [SIFLI_BLE_TEST_RX_CCCD] = {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},
};

static uint8_t g_sifli_ble_test_svc[ATT_UUID_128_LEN] = sifli_ble_test_service_uuid;
static sibles_hdl g_sifli_test_ble_test_hdl;

// sifli ble test
static uint8_t *sifli_ble_test_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    switch (idx)
    {
    case SIFLI_BLE_TEST_TX_VALUE:
    {
        break;
    }
    }
    *len = 1;
    return 0;
}

static uint8_t sifli_ble_test_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    switch (para->idx)
    {
    case SIFLI_BLE_TEST_TX_VALUE:
    {
        break;
    }
    default:
        break;
    }
    return 0;
}

static void sifli_ble_test_init()
{
    sibles_register_svc_128_t svc;

    svc.att_db = (struct attm_desc_128 *)&sifli_ble_test_att_db;
    svc.num_entry = SIFLI_BLE_TEST_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, SEC_CON) | PERM(SVC_UUID_LEN, UUID_128) | PERM(SVC_MI, DISABLE);
    svc.uuid = g_sifli_ble_test_svc;
    g_sifli_test_ble_test_hdl = sibles_register_svc_128(&svc);
    if (g_sifli_test_ble_test_hdl)
    {
        sibles_register_cbk(g_sifli_test_ble_test_hdl, sifli_ble_test_get_cbk, sifli_ble_test_set_cbk);
    }
}


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


static void ble_app_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;
    // Local name
    char local_name[] = "TEST_SIFLI";

    // Manufaturer data
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;

    // Set name to ble service
    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + sizeof(local_name));
    dev_name->len = sizeof(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    // Set adveritsing address as static
    para.own_addr_type = GAPM_STATIC_ADDR;
    // Using connect mode to connect with remote device
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x140;
    para.config.mode_config.conn_config.backgroud_mode_enabled = 0x0;
    para.config.mode_config.conn_config.backgroud_duration = 6000;
    para.config.mode_config.conn_config.backgroud_interval = 0x200;
    para.config.mode_config.conn_config.is_repeated = 1;
    // Max TX Power set 0x7F which let ble stack control it
    para.config.max_tx_pwr = 0x7F;
    // Enable restart after disconnected
    para.config.is_auto_restart = 1;
    // adv data and rsp data use same data
    para.config.is_rsp_data_duplicate = 1;

    para.adv_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name, para.adv_data.completed_name->name_len);

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

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}


void fmp_app_callback(uint8_t conn_idx, uint8_t alert_lvl)
{
    LOG_I("fmp alert %d\r\n", alert_lvl);
}

#ifdef BSP_BLE_PXPR
void pxp_app_callback(uint8_t conn_idx, uint8_t event, uint8_t alert_lvl)
{
    LOG_I("pxp alert type %d, level %d\r\n", event, alert_lvl);
}
#endif // BSP_BLE_PXPR

static uint8_t g_bass_app_bas_lvl = 100;
uint8_t bass_app_callback(uint8_t conn_idx, uint8_t event)
{
    uint8_t ret = 0;
    switch (event)
    {
    case BLE_BASS_GET_BATTERY_LVL:
    {
        ret = g_bass_app_bas_lvl;
        break;
    }
    default:
        break;
    }
    LOG_I("bass callback type %d, ret %d\r\n", event, ret);
    return ret;
}

uint8_t g_diss_conn_idx;
uint16_t g_th_total_cnt;
uint16_t g_th_interval;
uint16_t g_th_packet_size;
void ble_app_entry(void *param)
{
    while (1)
    {
        uint32_t value;
        rt_mb_recv(g_ble_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            ble_app_advertising_start();
            sifli_ble_test_init();
#ifdef BSP_BLE_SERIAL_TRANSMISSION
            ble_serial_tran_init();
#endif // BSP_BLE_SERIAL_TRANSMISSION
            ble_fmpt_init(fmp_app_callback);
#ifdef BSP_BLE_PXPR
            ble_pxpr_init(pxp_app_callback);
#endif
            ble_bass_init(bass_app_callback, g_bass_app_bas_lvl);
            LOG_I("receive BLE power on!\r\n");
        }
        else if (value == 0xFF1F)
        {
            uint16_t total_count = g_th_total_cnt;
            LOG_I("receive throughput test!\r\n");
#ifdef BSP_BLE_SERIAL_TRANSMISSION
            uint8_t *array = rt_malloc(g_th_packet_size);
            int ret;

            while (total_count--)
            {
                ble_serial_tran_data_t rsp_header;
                rt_memset(array, (uint8_t)(total_count & 0xFF), g_th_packet_size);
                rsp_header.handle = g_diss_conn_idx;
                rsp_header.cate_id = BLE_APP_CATEID;
                rsp_header.len = g_th_packet_size;
                rsp_header.data = array;
                ret = ble_serial_tran_send_data(&rsp_header);
                while (ret == 0)
                {
                    ret = ble_serial_tran_send_data(&rsp_header);
                }
                //LOG_E("Send failed");
                rt_thread_mdelay(g_th_interval);
            }
            rt_free(array);
#endif // BSP_BLE_SERIAL_TRANSMISSION
        }
    }
}

int ble_app_init(void)
{
    rt_thread_t tid;
    sifli_ble_enable();
    g_ble_app_mb = rt_mb_create("ble_app", 8, RT_IPC_FLAG_FIFO);
    tid = rt_thread_create("ble_app", ble_app_entry, NULL, 1024, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(ble_app_init);


int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        if (g_ble_app_mb)
            rt_mb_send(g_ble_app_mb, BLE_POWER_ON_IND);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        // ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
        //case BLE_GAP_UPDATE_CONN_PARAM_CNF:
    {
        break;
    }
    case SIBLES_REMOTE_CONNECTED_IND:
    {
        // sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        LOG_I("BLE_GAP_DISCONNECTED_IND");
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);


#define TEST_BF0_SIBLES
#ifdef TEST_BF0_SIBLES

uint8_t g_diss_adv_idx;
//uint8_t g_diss_conn_idx;
uint8_t g_diss_scan_idx;

int cmd_diss(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "stack_adv") == 0)
        {
            if (strcmp(argv[2], "create") == 0)
            {
                ble_gap_adv_parameter_t para;
                para.own_addr_type = atoi(argv[3]);
                para.type = GAPM_ADV_TYPE_LEGACY;
                para.disc_mode = atoi(argv[4]);
                para.prop = atoi(argv[5]);
                para.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
                para.prim_cfg.chnl_map = 0x07;
                para.prim_cfg.phy = GAP_PHY_LE_1MBPS;
                para.prim_cfg.adv_intv_min = 0x64;
                para.prim_cfg.adv_intv_max = 0x64;
                ble_gap_create_advertising(&para);
            }
            else if (strcmp(argv[2], "set_data") == 0)
            {
                int i, len;
                ble_gap_adv_data_t *adv_data = rt_malloc(sizeof(ble_gap_adv_data_t) + GAP_ADV_DATA_LEN);

                len = atoi(argv[3]);
                len = (len <= GAP_ADV_DATA_LEN ? len : GAP_ADV_DATA_LEN);
                for (i = 0; i < len; i++)
                {
                    adv_data->data[i] = (uint8_t)atoh(&(argv[4][i * 3]));
                }

                adv_data->actv_idx = g_diss_adv_idx;
                adv_data->length = len;
                ble_gap_set_adv_data(adv_data);
                ble_gap_set_scan_rsp_data(adv_data);
            }
            else if (strcmp(argv[2], "enable") == 0)
            {
                if (strcmp(argv[3], "0") == 0)
                {
                    ble_gap_adv_stop_t adv_stop;
                    adv_stop.actv_idx = g_diss_adv_idx;
                    ble_gap_stop_advertising(&adv_stop);
                }
                else
                {
                    ble_gap_adv_start_t adv_start;
                    adv_start.actv_idx = g_diss_adv_idx;
                    adv_start.duration = 0;
                    adv_start.max_adv_evt = 0;
                    ble_gap_start_advertising(&adv_start);
                }
            }
            else if (strcmp(argv[2], "delete") == 0)
            {
                ble_gap_adv_delete_t del;
                del.actv_idx = g_diss_adv_idx;
                ble_gap_delete_advertising(&del);
            }
        }
        else if (strcmp(argv[1], "connection") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                int i;
                ble_gap_connection_create_param_t conn_param;
                conn_param.own_addr_type = GAPM_STATIC_ADDR;
                conn_param.conn_to = 500;
                conn_param.type = GAPM_INIT_TYPE_DIRECT_CONN_EST;
                conn_param.conn_param_1m.scan_intv = 0x30;
                conn_param.conn_param_1m.scan_wd = 0x30;
                conn_param.conn_param_1m.conn_intv_max = 0x80;
                conn_param.conn_param_1m.conn_intv_min = 0x60;
                conn_param.conn_param_1m.conn_latency = 0;
                conn_param.conn_param_1m.supervision_to = 500;
                conn_param.conn_param_1m.ce_len_max = 100;
                conn_param.conn_param_1m.ce_len_min = 60;
                conn_param.peer_addr.addr_type = atoi(argv[3]);
                for (i = 0; i < BD_ADDR_LEN; i++)
                {
                    conn_param.peer_addr.addr.addr[i] = (uint8_t)atoh(&(argv[4][i * 3]));
                }
#ifdef CFG_BQB
                conn_param.peer_addr.addr.addr[0] = 0xbd;
                conn_param.peer_addr.addr.addr[1] = 0xb6;
                conn_param.peer_addr.addr.addr[2] = 0xf4;
                conn_param.peer_addr.addr.addr[3] = 0xdc;
                conn_param.peer_addr.addr.addr[4] = 0x1b;
                conn_param.peer_addr.addr.addr[5] = 0x00;
#endif
                ble_gap_create_connection(&conn_param);
            }
            else if (strcmp(argv[2], "cancel") == 0)
            {
                ble_gap_cancel_create_connection();
            }
            else if (strcmp(argv[2], "disc") == 0)
            {
                ble_gap_disconnect_t conn;
                conn.conn_idx = 0;
                conn.reason = CO_ERROR_CON_TERM_BY_LOCAL_HOST;
                ble_gap_disconnect(&conn);
            }

        }
        else if (strcmp(argv[1], "scan") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                ble_gap_scan_start_t scan_param;
                scan_param.own_addr_type = GAPM_STATIC_ADDR;
                scan_param.type = GAPM_SCAN_TYPE_GEN_DISC;
                scan_param.dup_filt_pol = 1;
                scan_param.scan_param_1m.scan_intv = 0x40;
                scan_param.scan_param_1m.scan_wd = 0x20;
                scan_param.duration = 0;
                scan_param.period = 0;
                ble_gap_scan_start(&scan_param);
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                ble_gap_scan_stop();
            }
        }
        else if (strcmp(argv[1], "search_svc") == 0)
        {
            int i, len;
            uint8_t uuid[128] = {0};

            len = atoi(argv[2]);
            for (i = 0; i < len; i++)
            {
                uuid[i] = (uint8_t)atoh(&(argv[3][i * 2]));
            }
            //sibles_attm_convert_to128(uuid, uuid, len);
            sibles_search_service(0, len, uuid);
        }
        else if (strcmp(argv[1], "update_bas_lvl") == 0)
        {
            g_bass_app_bas_lvl = atoi(argv[2]);
            ble_bass_notify_battery_lvl(g_diss_conn_idx, g_bass_app_bas_lvl);
        }
#ifdef BSP_BLE_TIMEC
        else if (strcmp(argv[1], "tipc") == 0)
        {
            if (strcmp(argv[2], "enabled") == 0)
            {
                LOG_I("conn(%d)\r\n", g_diss_conn_idx);
                ble_tipc_enable(g_diss_conn_idx);
            }
            else if (strcmp(argv[2], "read") == 0)
            {
                uint8_t read_type = atoi(argv[3]); // 0. curr time. 1. local time info
                if (read_type == 0)
                {
                    ble_tipc_read_current_time(g_diss_conn_idx);
                }
                else if (read_type == 1)
                {
                    ble_tipc_read_local_time_info(g_diss_conn_idx);
                }
            }
        }
#endif
        else if (strcmp(argv[1], "up_conn") == 0)
        {
            ble_gap_update_conn_param_t conn_para;
            conn_para.conn_idx = g_diss_conn_idx;
            conn_para.intv_max = (uint16_t)atoi(argv[2]);
            conn_para.intv_min = (uint16_t)atoi(argv[3]);
            // value = argv * 1.25
            conn_para.ce_len_max = 0x100;
            conn_para.ce_len_min = 0x1;
            conn_para.latency = 0;
            conn_para.time_out = 500;
            ble_gap_update_conn_param(&conn_para);
        }
        else if (strcmp(argv[1], "lepsm_cfg") == 0)
        {
            ble_gap_lepsm_register();
        }
        else if (strcmp(argv[1], "bass_ind") == 0)
        {
            ble_bass_notify_battery_lvl(g_diss_conn_idx, (uint8_t)atoi(argv[2]));
        }
        else if (strcmp(argv[1], "th_test") == 0)
        {
            if (g_ble_app_mb)
            {
                g_th_total_cnt = (uint16_t)atoi(argv[2]);
                g_th_interval = (uint16_t)atoi(argv[3]);
                g_th_packet_size = (uint16_t)atoi(argv[4]);
                rt_mb_send(g_ble_app_mb, 0xFF1F);
            }
        }
        else if (strcmp(argv[1], "up_phy") == 0)
        {
            ble_gap_update_phy_t phy;
            phy.conn_idx = g_diss_conn_idx;
            phy.rx_phy = GAP_PHY_LE_2MBPS;
            phy.tx_phy = GAP_PHY_LE_2MBPS;
            ble_gap_update_phy(&phy);
        }
        else if (strcmp(argv[1], "up_len") == 0)
        {
            ble_gap_update_data_len_t data_len;
            data_len.conn_idx = g_diss_conn_idx;
            data_len.tx_octets = (uint16_t)atoi(argv[2]);
            data_len.tx_time = (uint16_t)atoi(argv[3]);
            ble_gap_update_data_len(&data_len);
        }
        else if (strcmp(argv[1], "exch_mtu") == 0)
        {
            sibles_exchange_mtu(g_diss_conn_idx);
        }
        else if (strcmp(argv[1], "up_chann") == 0)
        {
            ble_gap_update_channel_map_t map;
            memset(map.channel_map, 0xFF, GAP_LE_CHNL_MAP_LEN);
            uint8_t index, pos;
            uint8_t bad_channel[] = {10, 21, 33, 37, 38, 39};
            uint32_t i;
            for (i = 0; i < sizeof(bad_channel); i++)
            {
                index = bad_channel[i] / 8;
                pos = bad_channel[i] % 8;
                map.channel_map[index] &= ~(1 << pos);
            }
            ble_gap_update_channel_map(&map);
        }
#ifndef BSP_USING_PC_SIMULATOR
        else if (strcmp(argv[1], "bt_lp") == 0)
        {
            uint8_t is_off = (uint8_t)atoi(argv[2]);

            if (is_off)
                hwp_hpsys_aon->ISSR |= HPSYS_AON_ISSR_HP2LP_REQ;
            else
                hwp_hpsys_aon->ISSR &= ~HPSYS_AON_ISSR_HP2LP_REQ;
        }
#endif
    }

    return 0;
}



int ble_app_cmd_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    switch (event_id)
    {
    case BLE_GAP_ADV_CREATED_IND:
    {
        ble_gap_adv_created_ind_t *ind = (ble_gap_adv_created_ind_t *)data;
        g_diss_adv_idx = ind->actv_idx;
        LOG_I("ADV start %d!\r\n", g_diss_adv_idx);
        break;
    }
    case BLE_GAP_CREATE_CONNECTION_CNF:
    {
        ble_gap_create_connection_cnf_t *ind = (ble_gap_create_connection_cnf_t *)data;
        LOG_I("CONN init %d\r\n", ind->status);
        break;
    }
    case BLE_GAP_SCAN_START_CNF:
    {
        ble_gap_start_scan_cnf_t *ind = (ble_gap_start_scan_cnf_t *)data;
        LOG_I("scan init %d\r\n", ind->status);
        break;
    }
    case SIBLES_REMOTE_CONNECTED_IND:
    {
        sibles_remote_connected_ind_t *ind = (sibles_remote_connected_ind_t *)data;
        g_diss_conn_idx = ind->conn_idx;
        //ble_tipc_enable(g_diss_conn_idx);
        LOG_I("Connected %d\r\n", g_diss_conn_idx);
        break;
    }
#ifdef BSP_BLE_TIMEC
    case BLE_TIPC_CURRENT_TIME_NOTIFY:
    case BLE_TIPC_READ_CURRENT_TIME_RSP:
    {
        ble_tipc_curr_time_t *cur_time = (ble_tipc_curr_time_t *)data;
#ifdef BSP_USING_PC_SIMULATOR
        LOG_I("Simulator do not change date/time via TIPC");
#else
        set_date(cur_time->date_time.year + 2000, cur_time->date_time.month, cur_time->date_time.day);
        set_time(cur_time->date_time.hour, cur_time->date_time.min, cur_time->date_time.sec);
#endif
        {
            extern void app_clock_reset_time(void);
            app_clock_reset_time();
        }
        LOG_I("event(%d), year(%4d), mon(%2d), d(%2d), h(%2d), m(%2d), sec(%2d), dayofweek(%2d), fraction(%3d/256), adjust_reason(%d)\r\n",
              event_id, cur_time->date_time.year, cur_time->date_time.month, cur_time->date_time.day, cur_time->date_time.hour,
              cur_time->date_time.min, cur_time->date_time.sec, cur_time->day_of_week, cur_time->fraction_256, cur_time->adjust_reason);
        break;
    }
    case BLE_TIPC_READ_LOCAL_INFO_RSP:
    {
        ble_tip_local_time_info_t *loc_info = (ble_tip_local_time_info_t *)data;
        LOG_I("time zone(%d), dst offset(%d)\r\n", loc_info->time_zone, loc_info->dst_offset);
        break;
    }
#endif // BSP_BLE_TIMEC
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_cmd_handler, NULL);

FINSH_FUNCTION_EXPORT_ALIAS(cmd_diss, __cmd_diss, My device information service.);
#endif

