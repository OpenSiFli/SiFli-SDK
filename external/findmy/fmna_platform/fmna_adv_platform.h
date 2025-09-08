/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_adv_platform_h
#define fmna_adv_platform_h

#include "fmna_constants.h"
#include "fmna_platform_includes.h"

#define FMNA_ADV_TX_POWER_DBM       4    /** 4dBm -  maximum tx power. */
#define SIG_APPLE_COMPANY_ID 0x004C


typedef enum
{
    BLE_ADV_MODE_FAST = 0,
    BLE_ADV_MODE_SLOW,
} ble_adv_mode_t;

extern uint16_t fmna_separated_adv_fast_intv;
extern uint32_t fmna_separated_adv_fast_duration;
extern uint16_t fmna_separated_adv_slow_intv;
extern uint32_t fmna_separated_adv_slow_duration;
extern uint16_t fmna_nearby_adv_fast_intv;
extern uint32_t fmna_nearby_adv_fast_duration;
extern uint16_t fmna_nearby_adv_intv;
extern uint32_t fmna_nearby_adv_duration;
extern uint16_t fmna_pairing_adv_fast_intv;
extern uint32_t fmna_pairing_adv_fast_duration;
extern uint16_t fmna_pairing_adv_slow_intv;
extern uint32_t fmna_pairing_adv_slow_duration;

void fmna_adv_platform_get_default_bt_addr(uint8_t default_bt_addr[FMNA_BLE_MAC_ADDR_BLEN]);

/// Sets Random Static BT MAC address.
/// @param[in]   new_bt_mac      6-byte MAC address to set, in MSB first, e.g.
///                           new_bt_mac[0] is MSB of MAC address.
void fmna_adv_platform_set_random_static_bt_addr(uint8_t new_bt_mac[FMNA_BLE_MAC_ADDR_BLEN]);

void fmna_adv_platform_start_fast_adv(void);

void fmna_adv_platform_start_slow_adv(void);

/// Stop BLE advertising.
void fmna_adv_platform_stop_adv(void);

/// Setup Pairing advertisement.
void fmna_adv_platform_init_pairing(uint8_t *pairing_adv_service_data, size_t pairing_adv_service_data_size);

/// Setup Separated advertising,
void fmna_adv_platform_init_nearby(uint8_t *nearby_adv_manuf_data, size_t nearby_adv_manuf_data_size);

/// Setup Separated advertising.
/// @param[in]   separated_adv_manuf_data    Separated ADV manufacturer data.
void fmna_adv_platform_init_separated(uint8_t *separated_adv_manuf_data, size_t separated_adv_manuf_data_size);

void fmna_adv_info_init();

#endif /* fmna_adv_platform_h */
