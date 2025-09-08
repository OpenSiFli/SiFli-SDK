/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_constants_h
#define fmna_constants_h

#include "fmna_constants_platform.h"

#ifdef USE_CRYPTO
    #define HARDCODED_PAIRING_ENABLED 0
#else // USE_CRYPTO
    #define HARDCODED_PAIRING_ENABLED 1
#endif // USE_CRYPTO

#define FMNA_COMPANY_IDENTIFIER                        0x004C                                      /**< Company identifier for Apple. as per www.bluetooth.org. */
//#define DEVICE_NAME                                   "Find My Accessory"
#define DEVICE_NAME_PAIR                                "iGS800 - Find My"      // dev under pair mode, name should end with " - Find My"
#define DEVICE_NAME                                     "iGS800"
#define BLE_GAP_ADDR_LEN                              6

#define SYS_POINTER_BSIZE                             (sizeof(void *))

#define FMNA_PUBKEY_BLEN                              28                                          /**< Length (in bytes) of the public key. */
#define FMNA_SEPARATED_ADV_PAYLOAD_PUBKEY_BLEN        (FMNA_PUBKEY_BLEN - BLE_GAP_ADDR_LEN)       /**< Length (in bytes) of the Separated payload public key (bytes 6-27). */
#define FMNA_SEPARATED_ADV_PUBKEY_HINT_INDEX          5
#define PRIMARY_KEYS_PER_SECONDARY_KEY                96
#define SEPARATED_STATE_CONFIG_NEGATIVE_BOUNDARY      4
#define FMNA_BLE_MAC_ADDR_BLEN                        6                                           /** 6 bytes per BT spec. */
#define NEARBY_TIMEOUT_MAX_SECONDS                    3600

#define NUM_OF_KEYS                                   8
#define PAIRING_MAX_LEN                               1394
#define CONFIG_MAX_LEN                                64
#define NONOWN_MAX_LEN                                7
#define PAIRED_OWNER_MAX_LEN                          144
#define GATT_HEADER_LEN                               3                                           /**< GATT header length. */
#define GATT_PAYLOAD_LEN(mtu)                         ((mtu) - GATT_HEADER_LEN)                   /**< Length of the ATT payload for a given ATT MTU. */

//MARK: Apple Information Service lengths
#define PROD_DATA_MAX_LEN                             8
#define MANU_NAME_MAX_LEN                             64
#define MODEL_NAME_MAX_LEN                            64
#define RESERVED_MAX_LEN                              3
#define ACC_CATEGORY_MAX_LEN                          8
#define ACC_CAP_MAX_LEN                               4
#define FW_VERS_MAX_LEN                               4
#define FINDMY_VERS_MAX_LEN                           4
#define BATT_TYPE_MAX_LEN                             1
#define BATT_LVL_MAX_LEN                              1

// Operand Lengths as per spec - R2 update from 8/5/20


#define INITIATE_PAIRING_DATA_LENGTH                  (SESSION_NONCE_BLEN + E1_BLEN)

#define OPCODE_OP_LENGTH                              4
#define STATUS_LENGTH                                 4

//MARK: Crypto Key Lengths
#define SESSION_NONCE_BLEN                            32
#define SERVER_SHARED_SECRET_BLEN                     32
#define SOFTWARE_AUTH_TOKEN_BLEN                      1024
#define SOFTWARE_AUTH_UUID_BLEN                       16
#define SERIAL_NUMBER_RAW_BLEN                        16
#define SK_BLEN                                       32
#define P_BLEN                                        57
#define C1_BLEN                                       32
#define E1_BLEN                                       113
#define C3_BLEN                                       60
#define E2_BLEN                                       1326
#define E3_BLEN                                       1040
#define E4_BLEN                                       1286
#define C2_BLEN                                       89
#define SEEDS_BLEN                                    32
#define ICLOUD_IDENTIFIER_BLEN                        60
#define S2_BLEN                                       100
#define H1_BLEN                                       32
#define SERIAL_NUMBER_PAYLOAD_HMAC_BLEN               32
#define SERIAL_NUMBER_PAYLOAD_OP_BLEN                 4
#define ENCRYPTED_SERIAL_NUMBER_PAYLOAD_BLEN          141

#define APPLE_SERVER_ENCRYPTION_KEY_BLEN              65
#define APPLE_SERVER_SIG_VERIFICATION_KEY_BLEN        65

#define PRODUCT_DATA_BLEN                             8
#define FINDMY_UUID_SERVICE                           0xFD44
#define FINDMY_CHAR_BASE_UUID                         {0x7A, 0x42, 0x04, 0x03, 0x73, 0x2F, 0xD4, 0xBE, 0xEF, 0x49, 0x3B, 0x94, 0x00, 0x00, 0x86, 0x4F}
#define FINDMY_UUID_PAIRING_CHAR                      0x0001
#define FINDMY_UUID_CONFIG_CHAR                       0x0002
#define FINDMY_UUID_NONOWN_CHAR                       0x0003
#define FINDMY_UUID_PAIRED_OWNER_CHAR                 0x0004
#define FINDMY_UUID_DEBUG_CHAR                        0x0005

#define UARP_UUID_SERVICE                             0xFD43
#define UARP_CHAR_BASE_UUID                           {0xDE, 0xB0, 0x01, 0x7F, 0x4A, 0x6A, 0xF1, 0xA4, 0x25, 0x42, 0x9B, 0x6D, 0x00, 0x00, 0x11, 0x94}
#define UARP_UUID_DATA_CHAR                           0x0001


#define AIS_SERVICE_BASE_UUID                         {0x8B, 0x47, 0x38, 0xDC, 0xB9, 0x11, 0xA9, 0xA1, 0xB1, 0x43, 0x51, 0x3C, 0x02, 0x01, 0x29, 0x87}
#define AIS_CHAR_BASE_UUID                            {0x0B, 0xBB, 0x6F, 0x41, 0x3A, 0x00, 0xB4, 0xA7, 0x57, 0x4D, 0x52, 0x63, 0x00, 0x00, 0xA5, 0X6A}

#define PRODUCT_DATA_VAL                              {0x9F, 0x9E, 0xB5, 0x40, 0x7E, 0x06, 0x66, 0xC9}
//#define ACCESSORY_CATEGORY                            129
#define ACCESSORY_CATEGORY                            163
#endif /* fmna_constants_h */
