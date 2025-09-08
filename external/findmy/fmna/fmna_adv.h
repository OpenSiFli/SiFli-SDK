/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_adv_h
#define fmna_adv_h

#include "fmna_constants.h"
#include "fmna_adv_platform.h"

typedef struct
{
    uint8_t type;
    uint8_t length;
    struct fmna_separated_payload_s
    {
        uint8_t status;
        uint8_t pubkey2[FMNA_SEPARATED_ADV_PAYLOAD_PUBKEY_BLEN];
        uint8_t extra;
        uint8_t hint;
    } __attribute__((packed)) fmna_separated_payload;
} __attribute__((packed)) fmna_separated_adv_packet_t;  // BLE manufacturer data

typedef struct
{
    uint8_t type;
    uint8_t length;
    struct fmna_nearby_payload_s
    {
        uint8_t status;
        uint8_t opt;
    } __attribute__((packed)) fmna_nearby_payload;
} __attribute__((packed)) fmna_nearby_adv_packet_t;  // BLE manufacturer data

#define RESERVED_BLEN 4

typedef struct fmna_pairing_payload_s
{
    uint8_t product_data[PRODUCT_DATA_BLEN];
    uint8_t acc_category[ACC_CATEGORY_MAX_LEN];
    uint8_t reserved[RESERVED_BLEN];
    uint8_t battery_state;
} __attribute__((packed)) fmna_pairing_payload_t;

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

#define FMNA_ADV_OPT_ADDR_TYPE_SHIFT                             6
#define FMNA_ADV_ADDR_TYPE_MASK                                  (0x03 << FMNA_ADV_OPT_ADDR_TYPE_SHIFT)

#define fmna_adv_start_fast_adv         fmna_adv_platform_start_fast_adv
#define fmna_adv_start_slow_adv         fmna_adv_platform_start_slow_adv
#define fmna_adv_stop_adv               fmna_adv_platform_stop_adv
#define fmna_adv_get_unpaired_bt_addr   fmna_adv_platform_get_default_bt_addr

/// Reset BT address to be default out-of-box address.
///
/// @details    Read the default address and set the appropriate BLE GAP 0b11 bits for Random Static address setting.
void fmna_adv_reset_bd_addr(void);

/// Setup Pairing advertisement.
void fmna_adv_init_pairing(void);

/// Setup Separated advertising with public key and hint.
///
/// @param[in]  separated_pubkey    Current primary separated key, e.g. latched key.
/// @param[in]  hint                               Byte 5 of current primary key.
void fmna_adv_init_separated(uint8_t separated_pubkey[FMNA_PUBKEY_BLEN], uint8_t hint);

/// Setup Separated advertising with public key.
///
/// @param[in]  pubkey      Current primary key.
void fmna_adv_init_nearby(uint8_t pubkey[FMNA_PUBKEY_BLEN]);

#if HARDCODED_PAIRING_ENABLED
    /// HARDCODED PAIRING FUNCTION: Separates hardcoded keys.
    /// @param[in]   keys        Keys to separate and rotate through.
    void organize_pub_keys(uint8_t *keys);
#endif

#endif /* fmna_adv_h */
