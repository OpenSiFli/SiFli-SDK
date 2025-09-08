/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#include "fmna_connection_platform.h"

#include "fmna_constants.h"
#include "fmna_connection.h"
#include "fmna_gatt_platform.h"
#include "fmna_state_machine.h"

#include "ble_connection_manager.h"
#include "bf0_sibles_nvds.h"

typedef struct
{
    uint8_t fmna_uuid[SOFTWARE_AUTH_UUID_BLEN];
    uint8_t fmna_token[SOFTWARE_AUTH_TOKEN_BLEN];
    uint8_t fmna_pair_state;

    fmna_primary_key_t primary_key;
    fmna_secondary_key_t secondary_key;

    bd_addr_t fmna_remote_public_addr;
} fmna_pair_info_t;

static fmna_pair_info_t g_fmna_pair_info;
bd_addr_t g_last_pair_dev;
uint8_t g_fmna_pSN[16];

static void fmna_connection_token_storage_init(void);

ret_code_t fmna_connection_platform_disconnect(uint16_t conn_handle)
{
    connection_manager_disconnect(conn_handle);
    return 0;
}

void fmna_connection_platform_gap_params_init(void)
{
    LOG_I("fmna_connection_platform_gap_params_init");
    // ret_code_t              ret_code;
    // ble_gap_conn_sec_mode_t sec_mode;

    // // Setup the GAP connection parameters struct with the relevant values.
    // ble_gap_conn_params_t   gap_conn_params = {
    //     .min_conn_interval = DEFAULT_MIN_CONNECTION_INTERVAL,
    //     .max_conn_interval = MAX_CONNECTION_INTERVAL,
    //     .slave_latency     = SLAVE_LATENCY,
    //     .conn_sup_timeout  = SUPERVISION_TIMEOUT,
    // };

    // BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&sec_mode);

    // ret_code = sd_ble_gap_device_name_set(&sec_mode,
    //                                       (const uint8_t *)DEVICE_NAME,
    //                                       strlen(DEVICE_NAME));
    // APP_ERROR_CHECK(ret_code);

    // ret_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    // APP_ERROR_CHECK(ret_code);

    fmna_connection_token_storage_init();

}

/// Function for handling a Connection Parameters error.
/// @param nrf_error Error code containing information about what went wrong.
static void conn_params_error_handler(uint32_t nrf_error)
{
    LOG_I("conn_params_error_handler %d", nrf_error);
    // APP_ERROR_HANDLER(nrf_error);
}

void fmna_connection_platform_conn_params_init(void)
{
    LOG_I("fmna_connection_platform_conn_params_init");
    // ret_code_t             ret_code;
    // ble_conn_params_init_t cp_init = {0};

    // cp_init.p_conn_params                  = NULL;
    // cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    // cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    // cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    // cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    // cp_init.disconnect_on_fail             = true;
    // cp_init.evt_handler                    = NULL;  // Ignore events.
    // cp_init.error_handler                  = conn_params_error_handler;

    // ret_code = ble_conn_params_init(&cp_init);
    //APP_ERROR_CHECK(ret_code);
}

/// Handle encryption request for Find My Network accessories.
///
/// @details    This function bypasses Peer Manager handling. It is heavily based on
///             sec_info_request_process function from security_dispatcher.c.
// static void fmna_sec_info_request_process(ble_gap_evt_t const * p_gap_evt) {
//     ret_code_t                 err_code;
//     ble_gap_enc_info_t         new_enc_info = {0};

//     NRF_LOG_INFO("fmna_sec_info_request_process");
//     NRF_LOG_INFO("Using FMNA LTK");

//     // Copy over our own LTK
//     memcpy(new_enc_info.ltk, fmna_connection_get_active_ltk(), BLE_GAP_SEC_KEY_LEN);
//     new_enc_info.ltk_len = BLE_GAP_SEC_KEY_LEN;

//     err_code = sd_ble_gap_sec_info_reply(p_gap_evt->conn_handle, &new_enc_info, NULL, NULL);
//     if (err_code == NRF_ERROR_INVALID_STATE) {
//         // Do nothing. If disconnecting, it will be caught later by the handling of the DISCONNECTED
//         // event. If there is no SEC_INFO_REQ pending, there is either a logic error, or the user
//         // is also calling sd_ble_gap_sec_info_reply(), but there is no way for the present code to
//         // detect which one is the case.
//         NRF_LOG_WARNING("sd_ble_gap_sec_info_reply() returned NRF_EROR_INVALID_STATE, which is an"\
//                         "error unless the link is disconnecting.");
//     } else if (err_code != NRF_SUCCESS) {
//         NRF_LOG_ERROR("Could not complete encryption procedure. sd_ble_gap_sec_info_reply() "\
//                       "returned %s. conn_handle: %d.",
//                       nrf_strerror_get(err_code),
//                       p_gap_evt->conn_handle);
//     }
// }

int ble_conn_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    switch (event_id)
    {
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        fmna_connection_connected_handler(ind->conn_idx, ind->con_interval);


        // LOG_I("Peer device(%x-%x-%x-%x-%x-%x) connected", env->conn_para.peer_addr.addr[5],
        //       env->conn_para.peer_addr.addr[4],
        //       env->conn_para.peer_addr.addr[3],
        //       env->conn_para.peer_addr.addr[2],
        //       env->conn_para.peer_addr.addr[1],
        //       env->conn_para.peer_addr.addr[0]);
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        fmna_connection_conn_param_update_handler(ind->conn_idx, ind->con_interval);
        LOG_I("Updated connection interval :%d", ind->con_interval);
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        // env->conn_para.mtu = ind->mtu;
        LOG_I("Exchanged MTU size: %d", ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        // TODO REMOVE
        fmna_connection_disconnected_handler(ind->conn_idx, ind->reason);
        LOG_I("BLE_GAP_DISCONNECTED_IND(%d)", ind->reason);
        break;
    }
    case BLE_GAP_BOND_IND:
    {
        ble_gap_bond_ind_t *ind = (ble_gap_bond_ind_t *)data;
        switch (ind->info)
        {
            case GAPC_PAIRING_SUCCEED:
            {
                LOG_I("FMNA pair success %d", ind->conn_idx);

                // mark as encrypted in the connection record
                fmna_connection_update_connection_info(ind->conn_idx,
                                                       FMNA_MULTI_STATUS_ENCRYPTED,
                                                       true);
    
                // BT pairing completed successfully/ link was encrypted. Send BONDED event to state machine.
                fmna_evt_handler(FMNA_SM_EVENT_BONDED, NULL);
                break;
            }
            case GAPC_IRK_EXCH:
            {
                LOG_I("update last pair dev");
                memcpy(g_last_pair_dev.addr, ind->data.irk.addr.addr.addr, BD_ADDR_LEN);
                break;
            }
        }
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        if (fmna_connection_is_fmna_paired())
        {
            LOG_I("FMNA BLE_GAP_EVT_CONN_SEC_UPDATE");

            LOG_I("Conn secured: conn_handle: 0x%x, level: %d.",
                  ind->conn_idx,
                  ind->auth);

            // mark as encrypted in the connection record
            fmna_connection_update_connection_info(ind->conn_idx,
                                                   FMNA_MULTI_STATUS_ENCRYPTED,
                                                   true);

            // BT pairing completed successfully/ link was encrypted. Send BONDED event to state machine.
            fmna_evt_handler(FMNA_SM_EVENT_BONDED, NULL);
        }
        break;
    }
    // case BLE_GAP_BOND_IND:
    // {
    //     ble_gap_bond_ind_t *evt = (ble_gap_bond_ind_t *)data;
    //     // LOG_I("FMNA BOND %d", evt->info);
    //     if (evt->info == GAPC_PAIRING_SUCCEED)
    //     {
    //         LOG_I("FMNA pair success %d", evt->conn_idx);

    //         // mark as encrypted in the connection record
    //         fmna_connection_update_connection_info(evt->conn_idx,
    //                                                FMNA_MULTI_STATUS_ENCRYPTED,
    //                                                true);

    //         // BT pairing completed successfully/ link was encrypted. Send BONDED event to state machine.
    //         fmna_evt_handler(FMNA_SM_EVENT_BONDED, NULL);
    //     }
    //     break;
    // }
    // TODO: ask ltk and ask pair
    /*
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
            if (fmna_connection_is_fmna_paired()) {
                NRF_LOG_INFO("FMNA BLE_GAP_EVT_SEC_INFO_REQUEST");

                fmna_sec_info_request_process(p_gap_evt);
            }
            break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST: {
            NRF_LOG_INFO("FMNA BLE_GAP_EVT_SEC_PARAMS_REQUEST");

            if (fmna_connection_is_fmna_paired()) {
                // Reject the incoming security request if we are already FMNA paired.
                NRF_LOG_ERROR("FMNA Already paired. Reject request.");

                ret_code_t ret_code = sd_ble_gap_sec_params_reply(p_gap_evt->conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                if (NRF_SUCCESS != ret_code) {
                    NRF_LOG_ERROR("sd_ble_gap_sec_params_reply err 0x%x", ret_code);
                }
            }

        } break;
    */
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_conn_event_handler, NULL);


void fmna_connection_platform_log_token_help(void *auth_token, uint16_t token_size, void *auth_uuid, uint16_t uuid_size)
{
    LOG_I("MFi token: 0x%x, %d UUID 0x%x, %d", auth_token, token_size, auth_uuid, uuid_size);
    LOG_I("UUID: nrfjprog -f nrf52 --memrd 0x%x --w 8 --n %d", auth_uuid, uuid_size);
    LOG_I("Token: nrfjprog -f nrf52 --memrd 0x%x --w 8 --n %d", auth_token, token_size);
    //while (NRF_LOG_PROCESS()){}
}

#define MFI_TOKEN_MAX_LOG_CHUNK 64
void fmna_connection_platform_log_token(void *auth_token, uint16_t token_size, uint8_t isCrash)
{
    uint16_t token_remaining = token_size;
    void *p_temp = auth_token;
    uint16_t to_print;

    LOG_I("MFi Token:");
    while (token_remaining)
    {
        if (token_remaining > MFI_TOKEN_MAX_LOG_CHUNK)
        {
            to_print = MFI_TOKEN_MAX_LOG_CHUNK;
        }
        else
        {
            to_print = token_remaining;
        }
        //NRF_LOG_HEXDUMP_INFO(p_temp, to_print);
        token_remaining -= to_print;
        p_temp += to_print;
        // if (isCrash) {
        //     while (NRF_LOG_PROCESS()){}
        // }
    }
}

char num_to_char(uint8_t nibble)
{
    if (nibble < 10)
    {
        return (('0' + nibble));
    }

    return (('a' + nibble - 10));
}

void fmna_connection_platform_set_serial_number(uint8_t *pSN, uint8_t length)
{
    if (length > 16)
    {
        length = 16;
    }

    memcpy(g_fmna_pSN, pSN, length);
}

void fmna_connection_platform_get_serial_number(uint8_t *pSN, uint8_t length)
{
    int i = 0;

    memcpy(pSN, g_fmna_pSN, length);

    LOG_I("Serial Number:");
    LOG_HEX("s", 16, pSN, length);
}

// TODO remove / replace this with POR storage
// Using fstorage as temp storage for Token
// Please note this implementation is not 100% safe
// There is a window after erase and before write completes
// that there is no Token stored. A reset in this window could
// cause the device to become unusable.
#define SOFTWARE_AUTH_UUID_ADDR                0x79000
#define CODE_PAGE_SIZE                         4096
static void *m_p_update_token_data;
static uint16_t m_update_token_size;
//static void fmna_connection_fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
bool m_token_is_erasing = false;
bool m_new_token_stored = false;
// NRF_FSTORAGE_DEF(nrf_fstorage_t m_token_fs) =
// {
//     .evt_handler = fmna_connection_fstorage_evt_handler,
//     .start_addr  = SOFTWARE_AUTH_UUID_ADDR,
//     .end_addr    = SOFTWARE_AUTH_UUID_ADDR + CODE_PAGE_SIZE
// };

static void fmna_connection_token_storage_init(void)
{
    // Using fstorage as temp storage for Token
    // TODO remove / replace this with POR storage
    // ret_code_t ret_code = nrf_fstorage_init(&m_token_fs, &nrf_fstorage_sd, NULL);
    // APP_ERROR_CHECK(ret_code);

    LOG_I("fmna_connection_token_storage_init");
}

// static void fmna_connection_fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
// {
//     NRF_LOG_INFO("fmna_connection_fstorage_evt_handler %d %d", p_evt->id, p_evt->result);
//     switch (p_evt->id)
//     {
//         case NRF_FSTORAGE_EVT_ERASE_RESULT:
//             if (m_token_is_erasing) {
//                 m_token_is_erasing = false;
//                 if (p_evt->result != NRF_SUCCESS) {
//                     NRF_LOG_ERROR("Error erasing MFi Token");
//                     fmna_state_machine_dispatch_event(FMNA_SM_EVENT_FMNA_PAIRING_MFITOKEN);
//                 }
//                 else {
//                     ret_code_t ret_code = nrf_fstorage_write(&m_token_fs, SOFTWARE_AUTH_UUID_ADDR, m_p_update_token_data, m_update_token_size, NULL);
//                     APP_ERROR_CHECK(ret_code);
//                 }
//                 break;
//             }
//         case NRF_FSTORAGE_EVT_WRITE_RESULT:
//             if (p_evt->result != NRF_SUCCESS) {
//                 NRF_LOG_ERROR("Error writing updated MFi Token");
//             }
//             else {
//                 NRF_LOG_ERROR("MFi Token updated");
//                 m_new_token_stored = true;
//             }
//             fmna_state_machine_dispatch_event(FMNA_SM_EVENT_FMNA_PAIRING_MFITOKEN);
//         default:
//             break;
//     }
// }

void fmna_connection_storage_init()
{
    if (sizeof(g_fmna_pair_info) > SIFLI_NVDS_KEY_LEN_FIND_MY)
        RT_ASSERT(0);
    uint16_t len = sizeof(g_fmna_pair_info);

    uint8_t ret = sifli_nvds_read(SIFLI_NVDS_TYPE_FIND_MY, &len, (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("read fmna key failed!");
        memset((uint8_t *)&g_fmna_pair_info, 0, sizeof(g_fmna_pair_info));
        return;
    }

    fmna_connection_set_is_fmna_paired_power_on(g_fmna_pair_info.fmna_pair_state);
}

void fmna_connection_get_primary_key(fmna_primary_key_t *fmna_current_primary_key)
{
    fmna_current_primary_key->index = g_fmna_pair_info.primary_key.index;
    memcpy(fmna_current_primary_key->public_key, g_fmna_pair_info.primary_key.public_key, FMNA_PUBKEY_BLEN);
    memcpy(fmna_current_primary_key->ltk, g_fmna_pair_info.primary_key.ltk, GAP_SEC_KEY_LEN);
}

void fmna_connection_get_secondary_key(fmna_secondary_key_t *fmna_current_secondary_key)
{
    fmna_current_secondary_key->index = g_fmna_pair_info.secondary_key.index;
    memcpy(fmna_current_secondary_key->public_key, g_fmna_pair_info.secondary_key.public_key, FMNA_PUBKEY_BLEN);
}

void fmna_connection_update_primary_key(fmna_primary_key_t fmna_current_primary_key)
{
    g_fmna_pair_info.primary_key = fmna_current_primary_key;

    uint8_t ret = sifli_nvds_write(SIFLI_NVDS_TYPE_FIND_MY, sizeof(g_fmna_pair_info), (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("write fmna primary key failed!");
        return;
    }
}

void fmna_connection_update_secondary_key(fmna_secondary_key_t fmna_current_secondary_key)
{
    g_fmna_pair_info.secondary_key = fmna_current_secondary_key;

    uint8_t ret = sifli_nvds_write(SIFLI_NVDS_TYPE_FIND_MY, sizeof(g_fmna_pair_info), (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("write fmna secondary key failed!");
        return;
    }
}

uint8_t* fmna_product_value_get()
{
    static uint8_t product_data[PRODUCT_DATA_BLEN] = PRODUCT_DATA_VAL;
    return product_data;
}

uint8_t* fmna_fmna_connection_get_uuid()
{
    return g_fmna_pair_info.fmna_uuid;
}

uint8_t* fmna_fmna_connection_get_token()
{
    return g_fmna_pair_info.fmna_token;
}

bd_addr_t fmna_get_paired_addr()
{
    return g_fmna_pair_info.fmna_remote_public_addr;
}

void fmna_connection_update_pair_state(bool is_paired)
{
    g_fmna_pair_info.fmna_pair_state = is_paired;

    if (is_paired)
    {
        LOG_I("update fmna device");
        g_fmna_pair_info.fmna_remote_public_addr = g_last_pair_dev;
    }

    uint8_t ret = sifli_nvds_write(SIFLI_NVDS_TYPE_FIND_MY, sizeof(g_fmna_pair_info), (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("write fmna pair state failed!");
        return;
    }
}

void fmna_connection_storage_write_key()
{
    memcpy(g_fmna_pair_info.fmna_uuid, m_p_update_token_data, SOFTWARE_AUTH_UUID_BLEN);

    memset(g_fmna_pair_info.fmna_token, 0, SOFTWARE_AUTH_TOKEN_BLEN);
    memcpy(g_fmna_pair_info.fmna_token, m_p_update_token_data + SOFTWARE_AUTH_UUID_BLEN, m_update_token_size - SOFTWARE_AUTH_UUID_BLEN);

    uint8_t ret = sifli_nvds_write(SIFLI_NVDS_TYPE_FIND_MY, sizeof(g_fmna_pair_info), (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("write fmna key failed!");
        fmna_state_machine_dispatch_event(FMNA_SM_EVENT_FMNA_PAIRING_MFITOKEN);
        return;
    }

    m_new_token_stored = true;
    fmna_state_machine_dispatch_event(FMNA_SM_EVENT_FMNA_PAIRING_MFITOKEN);
}

void fmna_connection_update_uuid(uint8_t *data, uint16_t len)
{
    memcpy(g_fmna_pair_info.fmna_uuid, data, SOFTWARE_AUTH_UUID_BLEN);
    uint8_t ret = sifli_nvds_write(SIFLI_NVDS_TYPE_FIND_MY, sizeof(g_fmna_pair_info), (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("write fmna uuid failed!");
    }
}

void fmna_connection_update_token(uint8_t *data, uint16_t len)
{
    memcpy(g_fmna_pair_info.fmna_token, data, SOFTWARE_AUTH_TOKEN_BLEN);
    uint8_t ret = sifli_nvds_write(SIFLI_NVDS_TYPE_FIND_MY, sizeof(g_fmna_pair_info), (uint8_t *)&g_fmna_pair_info);
    if (ret != NVDS_OK)
    {
        LOG_E("write fmna token failed!");
    }
}


void fmna_connection_update_mfi_token_storage(void *p_data, uint16_t data_size)
{
    LOG_I("fmna_connection_update_mfi_token_storage %d", data_size);
    ret_code_t ret_code;

    //erase the MFi Token / UUID
    m_p_update_token_data = p_data;
    m_update_token_size = data_size;

    m_token_is_erasing = true;
    m_new_token_stored = false;
    fmna_connection_storage_write_key();
    // ret_code = nrf_fstorage_erase(&m_token_fs, SOFTWARE_AUTH_UUID_ADDR, 1, NULL);
    APP_ERROR_CHECK(ret_code);
}

bool fmna_connection_mfi_token_stored(void)
{
    return m_new_token_stored;
}
