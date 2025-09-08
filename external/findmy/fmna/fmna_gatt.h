/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_gatt_h
#define fmna_gatt_h

#include "fmna_constants.h"
#include <stdint.h>
#include <stdbool.h>

extern uint16_t m_gatt_mtu;

#define FRAGMENTED_FLAG_INDEX       0
#define FRAGMENTED_FLAG_LENGTH      1
#define FRAGMENTATION_BIT           0

#define FRAGMENTATION_HEADER_LENGTH         1

#define FMNA_AIS_UUID_SERVICE 0x0102;

#define ACC_CAPABILITY_PLAY_SOUND_BIT_POS           0
#define ACC_CAPABILITY_UT_MOTION_DETECT_BIT_POS     1
#define ACC_CAPABILITY_SRNM_LOOKUP_NFC_BIT_POS      2
#define ACC_CAPABILITY_SRNM_LOOKUP_BLE_BIT_POS      3
#define ACC_CAPABILITY_FW_UPDATE_SERVICE_BIT_POS    4

// Fragmentation flags available
typedef enum
{
    FRAGMENTED_FLAG_START_OR_CONTINUE = 0x0,
    FRAGMENTED_FLAG_FINAL,
} fragmented_flag_t;


// UUIDs of the characteristics in AIS
typedef enum
{
    FMNA_AIS_UUID_PRODUCT_DATA = 0x0001,
    FMNA_AIS_UUID_MANU_NAME,
    FMNA_AIS_UUID_MODEL_NAME,
    FMNA_AIS_UUID_RESERVED,
    FMNA_AIS_UUID_ACC_CATEGORY,
    FMNA_AIS_UUID_ACC_CAPABILITIES,
    FMNA_AIS_UUID_FW_VERS,
    FMNA_AIS_UUID_FINDMY_VERS,
    FMNA_AIS_UUID_BATT_TYPE,
    FMNA_AIS_UUID_BATT_LVL,
} FMNA_AIS_Char_UUID_t;

// PAIRING control point Commands
typedef enum
{
    FMNA_SERVICE_OPCODE_NONE                                              = 0x0000,

    // Pairing
    FMNA_SERVICE_OPCODE_PAIRING_CONTROL_POINT_BASE                        = 0x0100,
    FMNA_SERVICE_OPCODE_INITIATE_PAIRING                                  = 0x0100,
    FMNA_SERVICE_OPCODE_SEND_PAIRING_DATA                                 = 0x0101,
    FMNA_SERVICE_OPCODE_FINALIZE_PAIRING                                  = 0x0102,
    FMNA_SERVICE_OPCODE_SEND_PAIRING_STATUS                               = 0x0103,
    FMNA_SERVICE_OPCODE_PAIRING_COMPLETE                                  = 0x0104,

    // Config
    FMNA_SERVICE_OPCODE_CONFIG_CONTROL_POINT_BASE                         = 0x0200,
    FMNA_SERVICE_OPCODE_SOUND_START                                       = 0x0200,
    FMNA_SERVICE_OPCODE_SOUND_STOP                                        = 0x0201,
    FMNA_SERVICE_OPCODE_PERSISTENT_CONNECTION_STATUS                      = 0x0202,
    FMNA_SERVICE_OPCODE_SET_NEARBY_TIMEOUT                                = 0x0203,
    FMNA_SERVICE_OPCODE_UNPAIR                                            = 0x0204,
    FMNA_SERVICE_OPCODE_CONFIGURE_SEPARATED_STATE                         = 0x0205,
    FMNA_SERVICE_OPCODE_LATCH_SEPARATED_KEY                               = 0x0206,
    FMNA_SERVICE_OPCODE_SET_MAX_CONNECTIONS                               = 0x0207,
    FMNA_SERVICE_OPCODE_SET_UTC                                           = 0x0208,
    FMNA_SERVICE_OPCODE_GET_MULTI_STATUS                                  = 0x0209,
    FMNA_SERVICE_OPCODE_KEYROLL_INDICATION                                = 0x020A,
    FMNA_SERVICE_OPCODE_COMMAND_RESPONSE                                  = 0x020B,
    FMNA_SERVICE_OPCODE_GET_MULTI_STATUS_RESPONSE                         = 0x020C,
    FMNA_SERVICE_OPCODE_SOUND_COMPLETED                                   = 0x020D,
    FMNA_SERVICE_OPCODE_LATCH_SEPARATED_KEY_RESPONSE                      = 0x020E,

    // Non-owner
    FMNA_SERVICE_OPCODE_NON_OWNER_CONTROL_POINT_BASE                      = 0x0300,
    FMNA_SERVICE_NON_OWNER_OPCODE_SOUND_START                             = 0x0300,
    FMNA_SERVICE_NON_OWNER_OPCODE_SOUND_STOP                              = 0x0301,
    FMNA_SERVICE_NON_OWNER_OPCODE_COMMAND_RESPONSE                        = 0x0302,
    FMNA_SERVICE_NON_OWNER_OPCODE_SOUND_COMPLETED                         = 0x0303,
    FMNA_SERVICE_NON_OWNER_OPCODE_START_AGGRESSIVE_ADV                    = 0x0304,

    // Paired Owner information
    FMNA_SERVICE_OPCODE_PAIRED_OWNER_CONTROL_POINT_BASE                   = 0x0400,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_GET_CURRENT_PRIMARY_KEY              = 0x0400,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_GET_ICLOUD_IDENTIFIER                = 0x0401,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_GET_CURRENT_PRIMARY_KEY_RESPONSE     = 0x0402,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_GET_ICLOUD_IDENTIFIER_RESPONSE       = 0x0403,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_GET_SERIAL_NUMBER                    = 0x0404,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_GET_SERIAL_NUMBER_RESPONSE           = 0x0405,
    FMNA_SERVICE_PAIRED_OWNER_OPCODE_COMMAND_RESPONSE                     = 0x0406,

#if DEBUG
    // Debug
    FMNA_SERVICE_OPCODE_DEBUG_CONTROL_POINT_BASE                          = 0x0500,
    FMNA_SERVICE_DEBUG_OPCODE_SET_KEY_ROTATION_TIMEOUT                    = 0x0500,
    FMNA_SERVICE_DEBUG_OPCODE_RETRIEVE_LOGS                               = 0x0501,
    FMNA_SERVICE_DEBUG_OPCODE_LOG_RESPONSE                                = 0x0502,
    FMNA_SERVICE_DEBUG_OPCODE_COMMAND_RESPONSE                            = 0x0503,
    FMNA_SERVICE_DEBUG_OPCODE_RESET                                       = 0x0504,
    FMNA_SERVICE_DEBUG_UT_MOTION_TIMERS_CONFIG                            = 0x0505,
#endif

    // Reserved - can change if needed
    FMNA_SERVICE_OPCODE_PACKET_EXTENSION                                  = 0x01FF,

    // Internal opcode for sending UARP
    FMNA_SERVICE_OPCODE_INTERNAL_UARP_BASE                                = 0x0600,
    FMNA_SERVICE_OPCODE_INTERNAL_UARP                                     = 0x06FF,
} FMNA_Service_Opcode_t;

#define FMNA_SERVICE_OPCODE_BASE_MASK 0xFF00

// Response Status to Command Reponse possibilities
typedef enum
{
    RESPONSE_STATUS_SUCCESS                 = 0x0000,
    RESPONSE_STATUS_INVALID_STATE           = 0x0001,
    RESPONSE_STATUS_INVALID_CONFIGURATION   = 0x0002,
    RESPONSE_STATUS_INVALID_LENGTH          = 0x0003,
    RESPONSE_STATUS_INVALID_PARAM           = 0x0004,
    RESPONSE_STATUS_NO_COMMAND_RESPONSE     = 0xFFFE,
    RESPONSE_STATUS_INVALID_COMMAND         = 0xFFFF,
} FMNA_Response_Status_t;

typedef struct
{
    FMNA_Service_Opcode_t    opcode;
    uint16_t                 length;
} fmna_service_length_check_manager_t;

#define FMNA_SERVICE_LENGTH_CHECK_MANAGERS_SIZE(managers)    (sizeof(managers)/sizeof(managers[0]))

// Struct for containing the data for a packet longer than the MTU
typedef struct
{
    FMNA_Service_Opcode_t   opcode;
    uint8_t                *data;
    uint16_t                length;
} fmna_service_extended_packet_t;

extern fmna_service_extended_packet_t fmna_service_current_extended_packet_tx;

#if HARDCODED_PAIRING_ENABLED
    extern uint32_t current_key_index;
    extern uint8_t keys_to_rotate[NUM_OF_KEYS][FMNA_PUBKEY_BLEN];
#endif

typedef struct
{
    FMNA_Service_Opcode_t opcode;
} __attribute__((packed)) generic_control_point_packet_t;

typedef struct
{
    FMNA_Service_Opcode_t opcode;
    FMNA_Response_Status_t     status;
} __attribute__((packed)) control_point_command_response_data_t;

#define MAX_CONTROL_POINT_RSP 10
extern uint8_t m_command_response_index;

/// Generic function to send indications (via GATT) to a specific central.
///
/// @param[in]  conn_handle     Handle of connection to send indication to.
/// @param[in]  opcode                If this is the first fragment of the indication to be sent, the opcode should correspond to the command to be sent.
///                          See FMNA_Service_Opcode_t for more details.
/// @param[in]  data                    Data buffer for data associated w/ the specific opcode.
/// @param[in]  length                Length in bytes of associated data.
///
/// @note Function will append the opcode, if needed. Do NOT include the opcode in the data buffer.
void fmna_gatt_send_indication(uint16_t conn_handle, FMNA_Service_Opcode_t opcode, void *data, uint16_t length);
// Function for the platform file
void fmna_gatt_send_indication_internal(uint16_t conn_handle, FMNA_Service_Opcode_t opcode, void *data, uint16_t length);

void fmna_gatt_send_command_response(FMNA_Service_Opcode_t command_response_opcode, uint16_t conn_handle, FMNA_Service_Opcode_t command_opcode, FMNA_Response_Status_t status);

void fmna_gatt_dispatch_send_packet_extension_indication(void);

FMNA_Response_Status_t fmna_gatt_verify_control_point_opcode_and_length(FMNA_Service_Opcode_t opcode, uint16_t length, fmna_service_length_check_manager_t *managers, uint8_t num_managers);

void fmna_gatt_init(void);
void fmna_gatt_services_init(void);

void fmna_gatt_reset_queues(void);
void fmna_gatt_dispatch_send_next_packet(void);

/// Get conn handle of the most recently connected device, useful for pairing.
uint16_t fmna_gatt_get_most_recent_conn_handle(void);

fmna_ret_code_t fmna_gatt_config_char_write_handler(uint16_t conn_handle, uint16_t uuid, uint16_t length, uint8_t const *data);
fmna_ret_code_t fmna_gatt_pairing_char_authorized_write_handler(uint16_t conn_handle, uint16_t uuid, uint16_t length, uint8_t const *data);
fmna_ret_code_t fmna_gatt_nonown_char_write_handler(uint16_t conn_handle, uint16_t uuid, uint16_t length, uint8_t const *data);
fmna_ret_code_t fmna_gatt_paired_owner_char_write_handler(uint16_t conn_handle, uint16_t uuid, uint16_t length, uint8_t const *data);
fmna_ret_code_t fmna_gatt_debug_char_write_handler(uint16_t conn_handle, uint16_t uuid, uint16_t length, uint8_t const *data);
fmna_ret_code_t fmna_gatt_uarp_char_write_handler(uint16_t conn_handle, uint16_t uuid, uint16_t length, uint8_t const *data);
#endif /* fmna_gatt_h */
