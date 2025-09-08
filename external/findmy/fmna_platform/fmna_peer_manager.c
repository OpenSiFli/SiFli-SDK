/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */


#include "fmna_peer_manager.h"
#include "fmna_state_machine.h"
#include "fmna_connection.h"

#include "ble_connection_manager.h"

uint32_t fmna_pm_peer_count(void)
{
    // return pm_peer_count();
    conn_manager_get_bonded_dev_t bonded_device;
    return connection_manager_get_bonded_devices((uint8_t *)&bonded_device);
}

void fmna_pm_delete_bonds(void)
{
    ret_code_t ret_code;
    LOG_I("Erase bonds!");
    ret_code = connection_manager_delete_all_bond();

    FMNA_ERROR_CHECK(ret_code);
}

// /// Function for handling Peer Manager events.
// /// @param p_evt Peer Manager event.
// static void pm_evt_handler(pm_evt_t const * p_evt) {
//     pm_handler_on_pm_evt(p_evt);
//     pm_handler_disconnect_on_sec_failure(p_evt);
//     pm_handler_flash_clean(p_evt);

//     ret_code_t ret_code = NRF_SUCCESS;

//     switch (p_evt->evt_id) {

//         case PM_EVT_CONNECTED:
//             NRF_LOG_INFO("PM_EVT_CONNECTED");

//             /* Put the necessary conditions in the if statement below. FMN accessory should be in FMN paired state at this point */
//             if (fmna_connection_is_fmna_paired())
//             {
//                 //TODO: for multiple services call pm_conn_exclude only when the
//                 // connected context indicates FMNA advertising, so we use
//                 // FMNA LTK for encryption.
//                 ret_code = pm_conn_exclude(p_evt->conn_handle, p_evt->params.connected.p_context);
//                 if (ret_code != NRF_SUCCESS)
//                 {
//                     NRF_LOG_ERROR("pm_conn_exclude returned error code: %d", ret_code);
//                 }
//             }
//             break;

//         case PM_EVT_PEERS_DELETE_SUCCEEDED: {
//             NRF_LOG_DEBUG("Del prev bonded peers.");
//         } break;

//         case PM_EVT_BONDED_PEER_CONNECTED: {
//             NRF_LOG_DEBUG("PM Conn to a prev bonded device.");
//         } break;

//         case PM_EVT_CONN_SEC_FAILED:
//             NRF_LOG_ERROR("PM Conn security failed: conn_handle: 0x%x", p_evt->conn_handle);
//             break;

//         case PM_EVT_CONN_SEC_SUCCEEDED: {
//             NRF_LOG_INFO("PM Conn secured: conn_handle: 0x%x, procedure: 0x%x.", p_evt->conn_handle, p_evt->params.conn_sec_succeeded.procedure);

//             // mark as encrypted in the connection record
//             fmna_connection_update_connection_info(p_evt->conn_handle, FMNA_MULTI_STATUS_ENCRYPTED, true);

//             // BT pairing completed successfully/ link was encrypted. Send BONDED event to state machine.
//             fmna_evt_handler(FMNA_SM_EVENT_BONDED, NULL);
//         } break;

//         case PM_EVT_CONN_SEC_PARAMS_REQ: {
//             NRF_LOG_INFO("PM Conn sec param req: conn_handle: 0x%x.", p_evt->conn_handle);

//             if (p_evt->params.conn_sec_params_req.p_peer_params == NULL) {
//                 // This is not a valid security procedure request, ignore it.
//                 break;
//             }

//             if (fmna_connection_is_fmna_paired()) {
//                 // Reject the incoming security request if we are already FMNA paired.
//                 NRF_LOG_ERROR("Already paired. Reject request.");
//                 pm_conn_sec_params_reply(p_evt->conn_handle, NULL, p_evt->params.conn_sec_params_req.p_context);
//             }

//         } break;

//         default:
//             break;
//     }
// }

// void peer_manager_init(void) {
//     ret_code_t ret_code;
//     ble_gap_sec_params_t sec_param = {0};

//     ret_code = pm_init();
//     FMNA_ERROR_CHECK(ret_code);

//     // Security parameters to be used for all security procedures.
//     sec_param.bond           = SEC_PARAM_BOND;
//     sec_param.mitm           = SEC_PARAM_MITM;
//     sec_param.lesc           = SEC_PARAM_LESC;
//     sec_param.keypress       = SEC_PARAM_KEYPRESS;
//     sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
//     sec_param.oob            = SEC_PARAM_OOB;
//     sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
//     sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
//     sec_param.kdist_own.enc  = 1;
//     sec_param.kdist_own.id   = 1;
//     sec_param.kdist_peer.enc = 1;
//     sec_param.kdist_peer.id  = 1;

//     ret_code = pm_sec_params_set(&sec_param);
//     FMNA_ERROR_CHECK(ret_code);

//     ret_code = pm_register(pm_evt_handler);
//     FMNA_ERROR_CHECK(ret_code);
// }

