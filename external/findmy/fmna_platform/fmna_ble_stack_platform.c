/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

/*
#include "fmna_constants.h"
#include "fmna_platform_includes.h"
#include "fmna_ble_stack_platform.h"
#include "fmna_connection.h"

static volatile bool m_sd_enabled = false;

/// Function for checking whether a bluetooth stack event is an advertising timeout.
/// @param p_ble_evt Bluetooth stack event.
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt) {
   return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}

/// Function for handling BLE events.
/// @param p_ble_evt  Bluetooth stack event.
/// @param p_context Unused.
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
   uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
   uint16_t role        = ble_conn_state_role(conn_handle);

   // based on the role this device plays in the connection, dispatch to the right handler
   if (role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt)) {
       fmna_ble_peripheral_evt(p_ble_evt);
   }
}

void fmna_ble_stack_platform_init(void) {
   ret_code_t ret_code = nrf_sdh_enable_request();
   APP_ERROR_CHECK(ret_code);

   // configure the BLE stack using the default settings and
   // fetch the start address of the application RAM
   uint32_t ram_start = 0;
   ret_code = nrf_sdh_ble_default_cfg_set(FMNA_BLE_CONN_CFG_TAG, &ram_start);
   APP_ERROR_CHECK(ret_code);

   // Enable BLE stack.
   ret_code = nrf_sdh_ble_enable(&ram_start);
   APP_ERROR_CHECK(ret_code);

   // Register a handler for BLE events.
   NRF_SDH_BLE_OBSERVER(m_ble_observer, FMNA_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

   // Wait for the Nordic Softdevice to be enabled
   while (!m_sd_enabled);

   // SoftDevice is enabled. Stack is up.
}

/// Event observer for Nordic Softdevice Handler events.
static void fmna_ble_stack_platform_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context) {
   switch (state) {
       case NRF_SDH_EVT_STATE_ENABLE_PREPARE:
           NRF_LOG_DEBUG("NRF_SDH_EVT_STATE_ENABLE_PREPARE");
           break;

       case NRF_SDH_EVT_STATE_ENABLED:
           NRF_LOG_DEBUG("NRF_SDH_EVT_STATE_ENABLED");
           m_sd_enabled = true;
           break;

       case NRF_SDH_EVT_STATE_DISABLE_PREPARE:
           NRF_LOG_DEBUG("NRF_SDH_EVT_STATE_DISABLE_PREPARE");
           break;

       case NRF_SDH_EVT_STATE_DISABLED:
           NRF_LOG_DEBUG("NRF_SDH_EVT_STATE_DISABLED");
           m_sd_enabled = false;
           break;

       default:
           break;
   }
}

NRF_SDH_STATE_OBSERVER(m_app_sd_sdh_state_obs, 0) = {
   .handler = fmna_ble_stack_platform_sdh_state_observer,
};
*/
