/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_gatt_platform_h
#define fmna_gatt_platform_h

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#include "fmna_crypto.h"
#include "fmna_gatt.h"

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

#if HARDCODED_PAIRING_ENABLED
    extern uint32_t current_key_index;
    extern uint8_t keys_to_rotate[NUM_OF_KEYS][FMNA_PUBKEY_BLEN];
#endif

typedef struct
{
    FMNA_Service_Opcode_t initiate_pairing_opcode;
    fmna_initiate_pairing_data_t initiate_pairing_data;
} __attribute__((packed)) initiate_pairing_packet_t;

typedef struct
{
    FMNA_Service_Opcode_t finalize_pairing_opcode;
    fmna_finalize_pairing_data_t finalize_pairing_data;
} __attribute__((packed)) finalize_pairing_packet_t;

typedef struct
{
    FMNA_Service_Opcode_t pairing_complete_opcode;
} __attribute__((packed)) pairing_complete_packet_t;

void device_info_init();

void fmna_gatt_platform_send_authorized_write_reply(bool accept);
uint8_t fmna_gatt_platform_send_indication(uint16_t conn_idx, FMNA_Service_Opcode_t *op_code, void *data, uint16_t length);
uint8_t fmna_gatt_platform_send_indication_busy(uint16_t conn_idx, FMNA_Service_Opcode_t opcode, void *data, uint16_t length);
void fmna_gatt_platform_services_init();
uint16_t fmna_gatt_platform_get_most_recent_conn_handle();
uint8_t fmna_gatt_platform_get_next_command_response_index(void);
void fmna_gatt_platform_init();
void fmna_gatt_platform_reset_indication_queue();
void fmna_gatt_platform_send_next_indication();

#endif /* fmna_gatt_platform_h */
