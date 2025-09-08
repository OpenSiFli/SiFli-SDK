/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"

#include "fmna_gatt_platform.h"
#include "fmna_constants.h"
#include "fmna_crypto.h"
#include "fmna_state_machine.h"
#include "fmna_adv.h"
#include "fmna_util_platform.h"

#include "fmna_pairing_control_point.h"
#include "fmna_connection.h"

#include "fmna_queue.h"
#include "fmna_util.h"
#include "fmna_version.h"


#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "ffmna_gatt_plat"
#include "log.h"

enum ble_fmna_accessory_information_att_list
{
    ACCESSORY_INFORMATION_SVC,
    ACCESSORY_PRODUCT_DATA_CHAR,
    ACCESSORY_PRODUCT_DATA_CHAR_VALUE,
    ACCESSORY_MANUFACTURER_NAME_CHAR,
    ACCESSORY_MANUFACTURER_NAME_CHAR_VALUE,
    ACCESSORY_MODEL_NAME_CHAR,
    ACCESSORY_MODEL_NAME_CHAR_VALUE,
    ACCESSORY_CATEGORY_CHAR,
    ACCESSORY_CATEGORY_CHAR_VALUE,
    ACCESSORY_CAPABILITIES_CHAR,
    ACCESSORY_CAPABILITIES_CHAR_VALUE,
    ACCESSORY_FIRMWARE_VERSION_CHAR,
    ACCESSORY_FIRMWARE_VERSION_CHAR_VALUE,
    ACCESSORY_FIND_MY_VERSION_CHAR,
    ACCESSORY_FIND_MY_VERSION_CHAR_VALUE,
    ACCESSORY_BATTERY_TYPE_CHAR,
    ACCESSORY_BATTERY_TYPE_CHAR_VALUE,
    ACCESSORY_BATTERY_LEVEL_CHAR,
    ACCESSORY_BATTERY_LEVEL_CHAR_VALUE,
    ACCESSORY_ATT_NB
};

enum ble_fmna_find_may_network_att_list
{
    FIND_MY_NETWORK_SVC,
    FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR,
    FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE,
    FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_CCCD,

    FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR,
    FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_VALUE,
    FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_CCCD,

    FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR,
    FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_VALUE,
    FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_CCCD,

    FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR,
    FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_VALUE,
    FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_CCCD,

    FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR,
    FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_VALUE,
    FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_CCCD,

    FIND_MY_NETWORK_ATT_NB
};

#define fmna_accessory_information_svc_uuid { \
    0x8b, 0x47, 0x38, 0xdc, \
    0xb9, 0x11, 0xa9, 0xa1, \
    0xb1, 0x43, 0x51, 0x3c, \
    0x02, 0x01, 0x29, 0x87 \
};

#define accessory_product_data_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x01, 0x00, 0xa5, 0x6a \
}

#define accessory_manufacture_name_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x02, 0x00, 0xa5, 0x6a \
}

#define accessory_model_name_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x03, 0x00, 0xa5, 0x6a \
}

#define accessory_category_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x05, 0x00, 0xa5, 0x6a \
}

#define accessory_capabilities_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x06, 0x00, 0xa5, 0x6a \
}

#define accessory_firmware_version_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x07, 0x00, 0xa5, 0x6a \
}

#define accessory_find_my_version_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x08, 0x00, 0xa5, 0x6a \
}

#define accessory_battery_type_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x09, 0x00, 0xa5, 0x6a \
}

#define accessory_battery_level_chara_uuid { \
    0x0b, 0xbb, 0x6f, 0x41, \
    0x3a, 0x00, 0xb4, 0xa7, \
    0x57, 0x4d, 0x52, 0x63, \
    0x0A, 0x00, 0xa5, 0x6a \
}



#define fmna_finy_my_netwrok_svc_uuid { \
    0xFB, 0x34, 0x9B, 0x5F, \
    0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, \
    0x44, 0xFD, 0x00, 0x00 \
}

#define fmna_find_my_pairing_control_point_chara_uuid { \
    0x7A, 0x42, 0x04, 0x03, \
    0x73, 0x2F, 0xD4, 0xBE, \
    0xEF, 0x49, 0x3B, 0x94, \
    0x01, 0x00, 0x86, 0x4F \
}

#define fmna_find_my_configuration_control_point_chara_uuid { \
    0x7A, 0x42, 0x04, 0x03, \
    0x73, 0x2F, 0xD4, 0xBE, \
    0xEF, 0x49, 0x3B, 0x94, \
    0x02, 0x00, 0x86, 0x4F \
}

#define fmna_find_my_non_owner_control_point_chara_uuid { \
    0x7A, 0x42, 0x04, 0x03, \
    0x73, 0x2F, 0xD4, 0xBE, \
    0xEF, 0x49, 0x3B, 0x94, \
    0x03, 0x00, 0x86, 0x4F \
}

#define fmna_find_my_paired_owner_information_control_point_chara_uuid { \
    0x7A, 0x42, 0x04, 0x03, \
    0x73, 0x2F, 0xD4, 0xBE, \
    0xEF, 0x49, 0x3B, 0x94, \
    0x04, 0x00, 0x86, 0x4F \
}


#define fmna_find_my_debug_control_point_chara_uuid { \
    0x7A, 0x42, 0x04, 0x03, \
    0x73, 0x2F, 0xD4, 0xBE, \
    0xEF, 0x49, 0x3B, 0x94, \
    0x05, 0x00, 0x86, 0x4F \
}




#define SERIAL_UUID_16(x) {((uint8_t)(x&0xff)),((uint8_t)(x>>8))}

static uint8_t g_fmna_accessory_information_svc[ATT_UUID_128_LEN] = fmna_accessory_information_svc_uuid;
static uint8_t g_fmna_find_my_network_svc[ATT_UUID_128_LEN] = fmna_finy_my_netwrok_svc_uuid;

// fmna_primary_key_t m_fmna_current_primary_key = {0};
// fmna_primary_key_t m_fmna_current_separated_primary_key = {0};
// fmna_secondary_key_t m_fmna_current_secondary_key = {0};


const struct attm_desc_128 accessory_information_svc_att_db[ACCESSORY_ATT_NB] =
{
    // TX POWER Service Declaration
    [ACCESSORY_INFORMATION_SVC]                  =   {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE),  PERM(RD, ENABLE), 0, 0},

    [ACCESSORY_PRODUCT_DATA_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_PRODUCT_DATA_CHAR_VALUE]          =   {accessory_product_data_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_MANUFACTURER_NAME_CHAR]           =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_MANUFACTURER_NAME_CHAR_VALUE]     =   {accessory_manufacture_name_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_MODEL_NAME_CHAR]                  =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_MODEL_NAME_CHAR_VALUE]            =   {accessory_model_name_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_CATEGORY_CHAR]                    =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_CATEGORY_CHAR_VALUE]              =   {accessory_category_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_CAPABILITIES_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_CAPABILITIES_CHAR_VALUE]          =   {accessory_capabilities_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_FIRMWARE_VERSION_CHAR]            =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_FIRMWARE_VERSION_CHAR_VALUE]      =   {accessory_firmware_version_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_FIND_MY_VERSION_CHAR]             =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_FIND_MY_VERSION_CHAR_VALUE]       =   {accessory_find_my_version_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_BATTERY_TYPE_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_BATTERY_TYPE_CHAR_VALUE]          =   {accessory_battery_type_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},

    [ACCESSORY_BATTERY_LEVEL_CHAR]               =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [ACCESSORY_BATTERY_LEVEL_CHAR_VALUE]         =   {accessory_battery_level_chara_uuid,    PERM(RD, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},
};

const struct attm_desc_128 find_my_network_svc_att_db[ACCESSORY_ATT_NB] =
{
    [FIND_MY_NETWORK_SVC]                  =   {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE),  PERM(RD, ENABLE), 0, 0},

    [FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE]          =   {fmna_find_my_pairing_control_point_chara_uuid,    PERM(WRITE_REQ, ENABLE) | PERM(IND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},
    [FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_CCCD]           =   {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},

    [FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR]          =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_VALUE]    =   {fmna_find_my_configuration_control_point_chara_uuid,    PERM(WRITE_REQ, ENABLE) | PERM(IND, ENABLE) | PERM(WP, SEC_CON), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},
    [FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_CCCD]     =   {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WP, SEC_CON), PERM(RI, ENABLE), 2},

    [FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_VALUE]          =   {fmna_find_my_non_owner_control_point_chara_uuid,    PERM(WRITE_REQ, ENABLE) | PERM(IND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},
    [FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_CCCD]           =   {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},

    [FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_VALUE]          =   {fmna_find_my_paired_owner_information_control_point_chara_uuid,    PERM(WRITE_REQ, ENABLE) | PERM(IND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},
    [FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_CCCD]           =   {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},

    [FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR]                =   {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    [FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_VALUE]          =   {fmna_find_my_debug_control_point_chara_uuid,    PERM(WRITE_REQ, ENABLE) | PERM(IND, ENABLE), PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE), 1024},
    [FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_CCCD]           =   {SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 2},
};

typedef struct
{
    uint16_t len;
    uint8_t data[PAIRING_MAX_LEN];
} pairing_rx_buffer_t;

typedef struct
{
    uint8_t conn_idx;
    sibles_hdl accessory_information_handle;
    sibles_hdl find_my_network_handle;
    uint8_t cccd_enable;
    uint16_t mtu;



    uint8_t product_data[PROD_DATA_MAX_LEN];
    uint8_t product_data_len;

    uint8_t manufacturer_name[MANU_NAME_MAX_LEN];
    uint8_t manufacturer_name_len;

    uint8_t model_name[MODEL_NAME_MAX_LEN];
    uint8_t model_name_len;

    uint8_t accessory_category[ACC_CATEGORY_MAX_LEN];
    uint8_t accessory_category_len;

    uint8_t accessory_capability[ACC_CAP_MAX_LEN];
    uint8_t firmware_version[FW_VERS_MAX_LEN];
    uint8_t find_my_network_version[FINDMY_VERS_MAX_LEN];
    uint8_t battery_type[BATT_TYPE_MAX_LEN];
    uint8_t battery_state[BATT_LVL_MAX_LEN];




    uint8_t is_paired;
    pairing_rx_buffer_t pairing_rx_buffer;
} ble_fmna_service_env_t;

static ble_fmna_service_env_t g_fmna_service_env_t;

static ble_fmna_service_env_t *ble_fmna_service_get_env(void)
{
    return &g_fmna_service_env_t;
}

// very basic indication queue
#define FMNA_INDICATION_QUEUE_SIZE 15
typedef struct
{
    void *data;
    uint16_t length;
    uint16_t conn_handle;
    FMNA_Service_Opcode_t opcode;
} fmna_indication_queue_t;

//m_indication_queue_busy is used to flag there is an active indication
// on going and new ones should be queued
static uint8_t m_indication_queue_busy = 0;
// m_indication_conn_handle holds the connection handle for the current indication
// If the there is a disconnect for this connection handle before the indication succeeds,
// then we should abandon and move on to the next queued indication
static uint16_t m_indication_conn_handle = BLE_CONN_HANDLE_INVALID;
fmna_queue_t m_indication_queue;


// #if HARDCODED_PAIRING_ENABLED
// uint8_t keys_to_rotate[NUM_OF_KEYS][FMNA_PUBKEY_BLEN] = {0};
// uint32_t current_key_index = 0;
// #endif
//static volatile FMNA_SM_State_t m_fmna_state;

#define FMNA_OPCODE_LENGTH                  2
#define START_PACKET_DATA_LENGTH            (GATT_MAX_MTU_SIZE - FRAGMENTATION_HEADER_LENGTH - FMNA_OPCODE_LENGTH)
#define CONTINUE_PACKET_DATA_LENGTH         (GATT_MAX_MTU_SIZE - FRAGMENTATION_HEADER_LENGTH)

typedef struct
{
    uint8_t header;
    union
    {
        struct
        {
            FMNA_Service_Opcode_t opcode;
            uint8_t               data[START_PACKET_DATA_LENGTH];
        } __attribute__((packed)) single_packet_data;
        uint8_t continuation_packet_data[CONTINUE_PACKET_DATA_LENGTH];
    } __attribute__((packed)) data;
} __attribute__((packed)) tx_buffer_t;

tx_buffer_t gatt_tx_buffer;

void device_info_init()
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    // env->product_data;

    uint8_t pd[8] = PRODUCT_DATA_VAL;
    memcpy(env->product_data, pd, PROD_DATA_MAX_LEN);

    /*
    uint8_t manu[8] = MANA;
    env->manufacturer_name_len = 8;
    memcpy(env->manufacturer_name, manu, 8);


    env->model_name_len = 5;
    uint8_t mn[5] = MODEL_NAMLE;
    memcpy(env->model_name, mn, 5);
    */

    env->manufacturer_name_len = sizeof(FMNA_MANUFACTURER_NAME);
    memcpy(env->manufacturer_name, (uint8_t *)FMNA_MANUFACTURER_NAME, env->manufacturer_name_len);

    env->model_name_len = sizeof(FMNA_MODEL_NAME);
    memcpy(env->model_name, (uint8_t *)FMNA_MODEL_NAME, env->model_name_len);


    env->accessory_category_len = 8;
    env->accessory_category[0] = ACCESSORY_CATEGORY;

    uint32_t sound_play = 1;
    uint32_t motion_detector_UT = 1;
    uint32_t serial_bumber_NFC = 0;
    uint32_t serial_bumber_BLE = 1;
    uint32_t fimware_update_serive = 0;

    uint32_t cap = (fimware_update_serive << 5) | (serial_bumber_BLE << 4) | (serial_bumber_NFC << 3) | (serial_bumber_NFC << 2) | sound_play;
    memcpy(env->accessory_capability, &cap, 4);

    uint8_t revision_version = FW_VERSION_REVISION_NUMBER;
    uint8_t minor_version = FW_VERSION_MINOR_NUMBER;
    uint16_t major_version = FW_VERSION_MAJOR_NUMBER;

    env->firmware_version[0] = revision_version;
    env->firmware_version[1] = minor_version;
    memcpy(env->firmware_version + 2, &major_version, 2);

    uint16_t find_my_network_version = 1;
    env->find_my_network_version[0] = 0;
    env->find_my_network_version[1] = 0;
    memcpy(env->find_my_network_version + 2, &find_my_network_version, 2);

    env->battery_type[0] = 2;

    env->battery_state[0] = 0;
}

uint8_t *ble_fmna_accessory_information_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    *len = 0;
    switch (idx)
    {
    case ACCESSORY_PRODUCT_DATA_CHAR_VALUE:
    {
        // Prepare data to remote device
        ret_val = (uint8_t *)&env->product_data;
        *len = PROD_DATA_MAX_LEN;
        break;
    }
    case ACCESSORY_MANUFACTURER_NAME_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->manufacturer_name;
        *len = env->manufacturer_name_len;
        break;
    }
    case ACCESSORY_MODEL_NAME_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->model_name;
        *len = env->model_name_len;
        break;
    }
    case ACCESSORY_CATEGORY_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->accessory_category;
        *len = env->accessory_category_len;
        break;
    }
    case ACCESSORY_CAPABILITIES_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->accessory_capability;
        *len = ACC_CAP_MAX_LEN;
        break;
    }
    case ACCESSORY_FIRMWARE_VERSION_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->firmware_version;
        *len = FW_VERS_MAX_LEN;
        break;
    }
    case ACCESSORY_FIND_MY_VERSION_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->find_my_network_version;
        *len = FW_VERS_MAX_LEN;
        break;
    }
    case ACCESSORY_BATTERY_TYPE_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->battery_type;
        *len = BATT_TYPE_MAX_LEN;
        break;
    }
    case ACCESSORY_BATTERY_LEVEL_CHAR_VALUE:
    {
        ret_val = (uint8_t *)&env->battery_state;
        *len = BATT_LVL_MAX_LEN;
        break;
    }
    default:
        break;
    }
    return ret_val;
}

void fmna_pairing_control_point_unpair_plat(void)
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    //add other cleanup
    memset(&env->pairing_rx_buffer, 0, sizeof(pairing_rx_buffer_t));
    return;
}



void fmna_gatt_platform_send_authorized_write_reply(bool accept)
{
    if (fmna_connection_get_num_connections() == 0)
    {
        return;
    }

    LOG_I("fmna_gatt_platform_send_authorized_write_reply, do nothing !");
}

uint8_t fmna_gatt_platform_get_next_command_response_index(void)
{
    uint8_t index;
    // CRITICAL_REGION_ENTER();
    index = m_command_response_index;
    m_command_response_index++;
    if (m_command_response_index <= MAX_CONTROL_POINT_RSP)
    {
        m_command_response_index = 0;
    }
    // CRITICAL_REGION_EXIT();
    return index;
}


uint8_t fmna_gatt_platform_send_indication(uint16_t conn_idx, FMNA_Service_Opcode_t *op_code, void *data, uint16_t length)
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    LOG_I("fmna_gatt_platform_send_indication %d", length);
    uint8_t indication_idx = FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE;

    switch (*op_code & FMNA_SERVICE_OPCODE_BASE_MASK)
    {
    case FMNA_SERVICE_OPCODE_PAIRING_CONTROL_POINT_BASE:
        indication_idx = FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE;
        break;

    case FMNA_SERVICE_OPCODE_CONFIG_CONTROL_POINT_BASE:
        indication_idx = FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_VALUE;
        break;

    case FMNA_SERVICE_OPCODE_NON_OWNER_CONTROL_POINT_BASE:
        indication_idx = FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_VALUE;
        break;

    case FMNA_SERVICE_OPCODE_PAIRED_OWNER_CONTROL_POINT_BASE:
        indication_idx = FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_VALUE;
        break;

#if DEBUG
    case FMNA_SERVICE_OPCODE_DEBUG_CONTROL_POINT_BASE:
        indication_idx = FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_VALUE;
        break;
#endif // DEBUG

    default:
        LOG_E("Unknown opcode: 0x%x", *op_code);
        break;
    }
    m_indication_conn_handle = conn_idx;

    sibles_value_t value;
    value.hdl = env->find_my_network_handle;
    value.idx = indication_idx;
    value.len = length;
    value.value = data;

    int ret = sibles_write_value_with_rsp(conn_idx, &value);
    if (ret != length)
    {
        LOG_E("sibles_write_value_with_rsp FAIL");
    }
    return FMNA_SUCCESS;
}

uint8_t ble_fmna_find_my_network_gatts_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    env->conn_idx = conn_idx;
    switch (para->idx)
    {
    case FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE:
    {
        fmna_gatt_pairing_char_authorized_write_handler(conn_idx, FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE, para->len, para->value);
        break;
    }
    case FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_VALUE:
    {
        fmna_gatt_config_char_write_handler(conn_idx, FIND_MY_NETWORK_CONFIGURATION_CONTROL_POINT_CHAR_VALUE, para->len, para->value);
        break;
    }
    case FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_VALUE:
    {
        fmna_gatt_nonown_char_write_handler(conn_idx, FIND_MY_NETWORK_NON_OWNER_CONTROL_POINT_CHAR_VALUE, para->len, para->value);
        break;
    }
    case FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_VALUE:
    {
        fmna_gatt_paired_owner_char_write_handler(conn_idx, FIND_MY_NETWORK_PAIRED_OWNER_INFORMATION_CONTROL_POINT_CHAR_VALUE, para->len, para->value);
        break;
    }
    case FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_VALUE:
    {
        fmna_gatt_debug_char_write_handler(conn_idx, FIND_MY_NETWORK_DEBUG_CONTROL_POINT_CHAR_VALUE, para->len, para->value);
        break;
    }
    default:
        break;
    }
    return 0;
}

void ble_fmna_accessory_information_service_server_init()
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();

    sibles_register_svc_128_t svc;
    svc.att_db = (struct attm_desc_128 *)&accessory_information_svc_att_db;
    svc.num_entry = ACCESSORY_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_MI, ENABLE) |  PERM(SVC_UUID_LEN, UUID_128);
    svc.uuid = g_fmna_accessory_information_svc;
    env->accessory_information_handle = sibles_register_svc_128(&svc);
    if (env->accessory_information_handle)
        sibles_register_cbk(env->accessory_information_handle, ble_fmna_accessory_information_gatts_get_cbk, NULL);
}

void ble_fmna_find_my_network_service_server_init()
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();

    memset(&env->pairing_rx_buffer, 0, sizeof(pairing_rx_buffer_t));
    sibles_register_svc_128_t svc;
    svc.att_db = (struct attm_desc_128 *)&find_my_network_svc_att_db;
    svc.num_entry = FIND_MY_NETWORK_ATT_NB;
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_MI, ENABLE) |  PERM(SVC_UUID_LEN, UUID_128);
    svc.uuid = g_fmna_find_my_network_svc;
    env->find_my_network_handle = sibles_register_svc_128(&svc);
    if (env->find_my_network_handle)
        sibles_register_cbk(env->find_my_network_handle, NULL, ble_fmna_find_my_network_gatts_set_cbk);
}

int ble_fmna_svc_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    switch (event_id)
    {
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->mtu = ind->mtu;
        m_gatt_mtu = MIN(GATT_MAX_MTU_SIZE, ind->mtu) - GATT_HEADER_LEN;
        LOG_I("Exchanged MTU size: %d, fmna mtu %d", ind->mtu, m_gatt_mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        if (ind->conn_idx == m_indication_conn_handle)
        {
            // TODO REMOVE
            fmna_gatt_dispatch_send_next_packet();
        }
        break;
    }
    case SIBLES_WRITE_VALUE_RSP:
    {
        m_indication_conn_handle = BLE_CONN_HANDLE_INVALID;
        sibles_write_value_rsp_t *rsp = (sibles_write_value_rsp_t *)data;
        LOG_I("SIBLES_WRITE_VALUE_RSP");
        if (memcmp_val(&fmna_service_current_extended_packet_tx, 0, sizeof(fmna_service_current_extended_packet_tx)))
        {
            LOG_I("Indicatiion has finished being sent.");
            fmna_gatt_dispatch_send_next_packet();
        }
        else
        {
            fmna_gatt_dispatch_send_packet_extension_indication();
        }
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_fmna_svc_handler, NULL);

void fmna_gatt_platform_init()
{
    LOG_I("fmna_gatt_platform_init");
}

void fmna_gatt_platform_services_init()
{
    LOG_I("fmna_gatt_platform_services_init");
    ble_fmna_accessory_information_service_server_init();
    ble_fmna_find_my_network_service_server_init();

    fmna_queue_init(&m_indication_queue, sizeof(fmna_indication_queue_t), FMNA_INDICATION_QUEUE_SIZE);
}

uint16_t fmna_gatt_platform_get_most_recent_conn_handle()
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();
    return env->conn_idx;
}

uint8_t fmna_gatt_platform_send_indication_busy(uint16_t conn_idx, FMNA_Service_Opcode_t opcode, void *data, uint16_t length)
{
    uint8_t queue_msg = 1;
    if (m_indication_queue_busy == 0)
    {
        queue_msg = 0;
        m_indication_queue_busy = 1;
    }
    else
    {
        fmna_indication_queue_t q_data =
        {
            .data = data,
            .length = length,
            .conn_handle = conn_idx,
            .opcode = opcode,
        };
        fmna_queue_push(&m_indication_queue, &q_data);
        FMNA_LOG_INFO("TX indication queued");
    }

    return queue_msg;
}


void fmna_gatt_platform_reset_indication_queue()
{
    LOG_I("fmna_gatt_platform_reset_indication_queue");
    m_indication_queue_busy = 0;
    m_indication_conn_handle = BLE_CONN_HANDLE_INVALID;
    fmna_queue_reset(&m_indication_queue);
}

void fmna_gatt_platform_send_next_indication()
{
    LOG_I("fmna_gatt_platform_send_next_indication");

    if (fmna_queue_count(&m_indication_queue) == 0)
    {
        // free up the busy flag
        m_indication_queue_busy = 0;
        m_indication_conn_handle = BLE_CONN_HANDLE_INVALID;
    }
    else
    {
        fmna_indication_queue_t entry;
        fmna_queue_pop(&m_indication_queue, &entry);
        fmna_gatt_send_indication_internal(entry.conn_handle, entry.opcode, entry.data, entry.length);
    }
}


int cmd_fmna(int argc, char *argv[])
{
    ble_fmna_service_env_t *env = ble_fmna_service_get_env();

    if (argc > 1)
    {
        if (strcmp(argv[1], "trc") == 0)
        {
            if (strcmp(argv[2], "mode") == 0)
            {
                uint8_t mode = atoi(argv[3]);
                uint32_t mask = atoi(argv[4]);
                sibles_set_trc_cfg(mode, mask);
            }
        }
        else if (0 == strcmp(argv[1], "ind"))
        {
            sibles_value_t value;
            uint8_t g_d[243] = {0};
            value.hdl = env->find_my_network_handle;
            value.idx = FIND_MY_NETWORK_PAIRING_CONTROL_POINT_CHAR_VALUE;
            value.len = 243;
            value.value = (uint8_t *)&g_d;
            sibles_write_value_with_rsp(0, &value);
        }
    }

    return 0;
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(cmd_fmna, My device information service.);
#endif