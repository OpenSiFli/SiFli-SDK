/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_connection_platform_h
#define fmna_connection_platform_h

#include "fmna_platform_includes.h"
#include "fmna_constants_platform.h"

#define FIRST_CONN_PARAMS_UPDATE_DELAY      MSEC_TO_TIMER_TICKS(5000)        /**< Time from initiating event (connect or start of notification) to the first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       MSEC_TO_TIMER_TICKS(30000)       /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                            /**< Number of attempts before giving up the connection parameter negotiation. */

/* Connection Interval Parameters Requirements:
 * 1. conn_sup_timeout * 4 > (1 + slave_latency) * max_conn_interval
 * 2. The Supervision_Timeout in milliseconds shall be larger than
 *    (1 + Conn_Latency) * Conn_Interval_Max * 2, where Conn_Interval_Max is given in milliseconds.
 */
#define DEFAULT_MIN_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(15, UNIT_1_25_MS)     /**< Determines minimum connection interval in milliseconds. */
#define NON_OWNER_MIN_CONNECTION_INTERVAL   (uint16_t) MSEC_TO_UNITS(200, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL             (uint16_t) MSEC_TO_UNITS(2000, UNIT_1_25_MS)   /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                       0                                              /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT                 (uint16_t) MSEC_TO_UNITS(5000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

/// Disconnect the link.
/// @details     Disconnects the link with BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION disconnect reason.
/// @param[in]   conn_handle     Connection handle for connection to disconnect.
ret_code_t fmna_connection_platform_disconnect(uint16_t conn_handle);

/// This function sets up all the necessary GAP (Generic Access Profile) parameters of the
/// device, including the device name, appearance, and the preferred connection parameters.
void fmna_connection_platform_gap_params_init(void);

/// Function for initializing the Connection Parameters module.
void fmna_connection_platform_conn_params_init(void);

void fmna_connection_platform_log_token_help(void *auth_token, uint16_t token_size, void *auth_uuid, uint16_t uuid_size);
void fmna_connection_platform_log_token(void *auth_token, uint16_t token_size, uint8_t isCrash);

void fmna_connection_platform_get_serial_number(uint8_t *pSN, uint8_t length);

void fmna_connection_update_mfi_token_storage(void *p_data, uint16_t data_size);
bool fmna_connection_mfi_token_stored(void);

bd_addr_t fmna_get_paired_addr();
void fmna_connection_update_uuid(uint8_t *data, uint16_t len);
void fmna_connection_update_token(uint8_t *data, uint16_t len);

uint8_t* fmna_fmna_connection_get_token();
void fmna_connection_platform_set_serial_number(uint8_t *pSN, uint8_t length);
uint8_t* fmna_fmna_connection_get_uuid();

#endif /* fmna_connection_platform_h */
