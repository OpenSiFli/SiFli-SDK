/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#include "fmna_gatt_platform.h"

#include "fmna_constants.h"
#include "fmna_util.h"
#include "fmna_adv.h"

OS_SEM_DECLAR(g_fmna_adv_sem);
SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);                               /**< Advertising module instance. */

sibles_advertising_para_t fmna_adv_para;
uint8_t fmna_adv_type;

typedef enum
{
    FMNA_ADV_TYPE_PAIRING,
    FMNA_ADV_TYPE_NEARBY,
    FMNA_ADV_TYPE_SEPARATED,
} FMNA_ADV_TYPE_t;

// Universally unique service identifier.
// static ble_uuid_t m_prox_adv_uuids[] = {
//     {FINDMY_UUID_SERVICE, BLE_UUID_TYPE_BLE}
// };

void reverse_copy(uint8_t *dest, uint8_t *src, int size)
{
    for (int i = 0; i < size; i++)
    {
        dest[i] = src[size - 1 - i];
    }
}

void fmna_adv_info_init()
{
    memset((uint8_t *)&fmna_adv_para, 0, sizeof(sibles_advertising_para_t));
}

void fmna_adv_platform_get_default_bt_addr(uint8_t default_bt_addr[FMNA_BLE_MAC_ADDR_BLEN])
{
    // // Read hardcoded address from factory register
    // memcpy(default_bt_addr, (uint8_t *)NRF_FICR->DEVICEADDR, FMNA_BLE_MAC_ADDR_BLEN);

    // // Nordic SD sd_ble_gap_addr_set expects ble_gap_addr_t as LSB.
    // reverse_array(default_bt_addr, 0, FMNA_BLE_MAC_ADDR_BLEN-1);

    bd_addr_t addr;
    ble_get_public_address(&addr);
    //uint8_t new_bt_mac[FMNA_BLE_MAC_ADDR_BLEN];

    reverse_copy(default_bt_addr, addr.addr, FMNA_BLE_MAC_ADDR_BLEN);
}

struct sibles_random_addr
{
    uint8_t addr_type;
    uint8_t addr[6];
};

void fmna_adv_platform_set_random_static_bt_addr(uint8_t new_bt_mac[FMNA_BLE_MAC_ADDR_BLEN])
{
    uint8_t new_addr[FMNA_BLE_MAC_ADDR_BLEN];

    //memcpy(new_bt_mac, current_pubkey, FMNA_BLE_MAC_ADDR_BLEN);
    reverse_copy(new_addr, new_bt_mac, FMNA_BLE_MAC_ADDR_BLEN);

    // Set address type bits of public key bd_addr for random static, 0b11.
    if ((new_addr[5] & (uint8_t)FMNA_ADV_ADDR_TYPE_MASK) != (uint8_t)FMNA_ADV_ADDR_TYPE_MASK)
    {
        LOG_E("NOT RANDOM ADDRESS!!!!!!!!!!!!!!!!!");
    }

    struct sibles_random_addr *r_addr = rt_malloc(sizeof(struct sibles_random_addr));
    BT_OOM_ASSERT(r_addr);
    if (r_addr)
    {
        LOG_I("SET RANDOM");
        r_addr->addr_type = 1;

        memcpy(r_addr->addr, new_addr, BD_ADDR_LEN);
        sibles_set_random_addr(0, (uint8_t *)r_addr);
        rt_free(r_addr);
    }
}

static void fmna_adv_start_adv(ble_adv_mode_t adv_mode)
{
    LOG_I("fmna_adv_start_adv %d, mode %d", adv_mode, fmna_adv_type);

    if (adv_mode == BLE_ADV_MODE_FAST)
    {
        if (fmna_adv_type == FMNA_ADV_TYPE_PAIRING)
        {
            // fmna_adv_para.config.mode_config.conn_config.duration = fmna_pairing_adv_fast_duration;
            fmna_adv_para.config.mode_config.conn_config.duration = fmna_pairing_adv_slow_duration;
            fmna_adv_para.config.mode_config.conn_config.interval = fmna_pairing_adv_fast_intv;
        }
        else if (fmna_adv_type == FMNA_ADV_TYPE_NEARBY)
        {
            fmna_adv_para.config.mode_config.conn_config.duration = fmna_nearby_adv_fast_duration;
            fmna_adv_para.config.mode_config.conn_config.interval = fmna_nearby_adv_fast_intv;
        }
        else if (fmna_adv_type == FMNA_ADV_TYPE_SEPARATED)
        {
            fmna_adv_para.config.mode_config.conn_config.duration = fmna_separated_adv_fast_duration;
            fmna_adv_para.config.mode_config.conn_config.interval = fmna_separated_adv_fast_intv;
        }
    }
    else if(adv_mode == BLE_ADV_MODE_SLOW)
    {
        if (fmna_adv_type == FMNA_ADV_TYPE_PAIRING)
        {
            fmna_adv_para.config.mode_config.conn_config.duration = fmna_pairing_adv_slow_duration;
            fmna_adv_para.config.mode_config.conn_config.interval = fmna_pairing_adv_slow_intv;
        }
        else if (fmna_adv_type == FMNA_ADV_TYPE_NEARBY)
        {
            fmna_adv_para.config.mode_config.conn_config.duration = fmna_nearby_adv_duration;
            fmna_adv_para.config.mode_config.conn_config.interval = fmna_nearby_adv_intv;
        }
        else if (fmna_adv_type == FMNA_ADV_TYPE_SEPARATED)
        {
            fmna_adv_para.config.mode_config.conn_config.duration = fmna_separated_adv_slow_duration;
            fmna_adv_para.config.mode_config.conn_config.interval = fmna_separated_adv_slow_intv;
        }
    }

    uint8_t ret = sibles_advertising_init(g_app_advertising_context, &fmna_adv_para);
    LOG_I("fmna_adv_platform_adv init %d", ret);

    sibles_advertising_start(g_app_advertising_context);

    if (fmna_adv_type == FMNA_ADV_TYPE_PAIRING)
    {
        rt_free(fmna_adv_para.adv_data.srv_data);
    }
    else
    {
        rt_free(fmna_adv_para.adv_data.manufacturer_data);
    }
}

void fmna_adv_platform_start_fast_adv(void)
{
    fmna_adv_start_adv(BLE_ADV_MODE_FAST);
}

void fmna_adv_platform_start_slow_adv(void)
{
    fmna_adv_start_adv(BLE_ADV_MODE_SLOW);
}

void fmna_adv_platform_stop_adv(void)
{
    LOG_I("fmna_adv_platform_stop_adv");
    sibles_advertising_stop(g_app_advertising_context);
    sibles_advertising_delete(g_app_advertising_context);

    if (g_fmna_adv_sem)
    {
        os_sem_delete(g_fmna_adv_sem);
        g_fmna_adv_sem = NULL;
    }
#ifdef OS_ADAPTOR_V2
    g_fmna_adv_sem = os_sem_create("fmna_adv_sem", 0);
#else
    os_sem_create(g_fmna_adv_sem, 0);
#endif

    if (g_fmna_adv_sem)
    {
        // make sure next start after delete
        LOG_I("adv sem taken");
        os_sem_take(g_fmna_adv_sem, 1000);
    }
}

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("ADV start resutl %d, mode %d", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d", evt->reason, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_DELETED:
    {
        sibles_adv_evt_deleted_t *evt = (sibles_adv_evt_deleted_t *)data;
        LOG_I("ADV delete %d", evt->status);
        if (g_fmna_adv_sem)
        {
            os_sem_release(g_fmna_adv_sem);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}


#define DEFAULT_LOCAL_NAME "SIFLI_APP"
void fmna_adv_platform_init_pairing(uint8_t *pairing_adv_service_data, size_t pairing_adv_service_data_size)
{
    LOG_I("ADV Pairing");

    memset((uint8_t *)&fmna_adv_para, 0, sizeof(sibles_advertising_para_t));
    fmna_adv_type = FMNA_ADV_TYPE_PAIRING;

    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = SIG_SIFLI_COMPANY_ID;
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);

    uint8_t name_len = sizeof(DEVICE_NAME_PAIR);
    memcpy(local_name, DEVICE_NAME_PAIR, name_len);
    //memcpy(dev_name->name, (uint8_t *)DEVICE_NAME, name_len);

    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);

    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    fmna_adv_para.own_addr_type = GAPM_STATIC_ADDR;
    fmna_adv_para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    fmna_adv_para.config.mode_config.conn_config.duration = 0x0;
    fmna_adv_para.config.mode_config.conn_config.interval = 0x30;
    fmna_adv_para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    fmna_adv_para.config.is_auto_restart = 0;
    /* Scan rsp data is same as advertising data. */
    //fmna_adv_para.config.is_rsp_data_duplicate = 1;

    uint16_t uuid = FINDMY_UUID_SERVICE;

    fmna_adv_para.adv_data.srv_data = rt_malloc(sizeof(fmna_pairing_payload_t) + sizeof(sibles_adv_type_srv_data_t));
    LOG_I("ADV SRV LEN %d, %d", sizeof(fmna_pairing_payload_t), sizeof(sibles_adv_type_srv_data_t));

    memcpy(fmna_adv_para.adv_data.srv_data->uuid.uuid.uuid_16, &uuid, ATT_UUID_16_LEN);
    fmna_adv_para.adv_data.srv_data->uuid.uuid_len = ATT_UUID_16_LEN;

    fmna_adv_para.adv_data.srv_data->data_len = sizeof(fmna_pairing_payload_t);
    memcpy(fmna_adv_para.adv_data.srv_data->additional_data, &pairing_adv_service_data, pairing_adv_service_data_size);

    fmna_adv_para.evt_handler = ble_app_advertising_event;
}

void fmna_adv_platform_init_nearby(uint8_t *nearby_adv_manuf_data, size_t nearby_adv_manuf_data_size)
{
    memset((uint8_t *)&fmna_adv_para, 0, sizeof(sibles_advertising_para_t));
    fmna_adv_type = FMNA_ADV_TYPE_NEARBY;

    //sibles_advertising_para_t fmna_adv_para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x12, 0x02, 0, 0};
    uint16_t manu_company_id = SIG_APPLE_COMPANY_ID;
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

    fmna_adv_para.own_addr_type = GAPM_STATIC_ADDR;
    fmna_adv_para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    fmna_adv_para.config.mode_config.conn_config.duration = 0x0;
    fmna_adv_para.config.mode_config.conn_config.interval = 0x30;
    fmna_adv_para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    fmna_adv_para.config.is_auto_restart = 0;
    /* Scan rsp data is same as advertising data. */
    //fmna_adv_para.config.is_rsp_data_duplicate = 1;

    // /* Prepare manufacturer filed .*/
    fmna_adv_para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + nearby_adv_manuf_data_size);
    fmna_adv_para.adv_data.manufacturer_data->company_id = manu_company_id;
    fmna_adv_para.adv_data.manufacturer_data->data_len = nearby_adv_manuf_data_size;
    rt_memcpy(fmna_adv_para.adv_data.manufacturer_data->additional_data, nearby_adv_manuf_data, nearby_adv_manuf_data_size);

    fmna_adv_para.evt_handler = ble_app_advertising_event;
}


void fmna_adv_platform_init_separated(uint8_t *separated_adv_manuf_data, size_t separated_adv_manuf_data_size)
{
    memset((uint8_t *)&fmna_adv_para, 0, sizeof(sibles_advertising_para_t));
    fmna_adv_type = FMNA_ADV_TYPE_SEPARATED;

    //sibles_advertising_para_t fmna_adv_para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[27] = {0};
    uint16_t manu_company_id = SIG_APPLE_COMPANY_ID;
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

    fmna_adv_para.own_addr_type = GAPM_STATIC_ADDR;
    fmna_adv_para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    fmna_adv_para.config.mode_config.conn_config.duration = 0x0;
    fmna_adv_para.config.mode_config.conn_config.interval = 0x30;
    fmna_adv_para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    fmna_adv_para.config.is_auto_restart = 0;
    /* Scan rsp data is same as advertising data. */
    //fmna_adv_para.config.is_rsp_data_duplicate = 1;

    /* Prepare manufacturer filed .*/
    fmna_adv_para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + separated_adv_manuf_data_size);
    fmna_adv_para.adv_data.manufacturer_data->company_id = manu_company_id;
    fmna_adv_para.adv_data.manufacturer_data->data_len = separated_adv_manuf_data_size;
    rt_memcpy(fmna_adv_para.adv_data.manufacturer_data->additional_data, separated_adv_manuf_data, separated_adv_manuf_data_size);

    fmna_adv_para.evt_handler = ble_app_advertising_event;
}