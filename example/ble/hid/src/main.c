/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"
#define LOG_TAG "ble_app"
#include "log.h"
#include "button.h"

/* Choose one HID type. */
#define HID_KEYBOARD
// #define HID_CONSUMER

#define HIDS_TEST
/* 24 * 1.25 = 30ms */
#define BLE_APP_HIGH_PERFORMANCE_INTERVAL (24)
#define BLE_APP_TIMEOUT_INTERVAL (5000)

typedef struct
{
    uint8_t key_code;      // Physical keycode
    uint8_t hid_code;      // HID keycode
} key_mapping_t;

static key_mapping_t key_mapping_table[1] =
{
    {0x01, 0x04}        // HID keycode for 'a'
};

typedef struct
{
    uint8_t is_power_on;
    uint8_t conn_idx;
    uint8_t is_bg_adv_on;
    struct
    {
        bd_addr_t peer_addr;
        uint16_t mtu;
        uint16_t conn_interval;
        uint8_t peer_addr_type;
    } conn_para;
    struct
    {
        sibles_hdl srv_handle;
        uint8_t is_config_on;
    } data;
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;
static app_env_t *ble_app_get_env(void);

/**********************Start of HID service ****************************************************/

#define BASE_USB_HID_SPEC_VERSION   0x0101
/* Buttons configuration */

/* Note: The configuration below is the same as BOOT mode configuration
 * This simplifies the code as the BOOT mode is the same as REPORT mode.
 * Changing this configuration would require separate implementation of
 * BOOT mode report generation.
 */


enum
{
    HIDS_REMOTE_WAKE = 1,
    HIDS_NORMALLY_CONNECTABLE = 2,
};

struct hids_info
{
    uint16_t version; /* version number of base USB HID Specification */
    uint8_t code; /* country HID Device hardware is localized for. */
    uint8_t flags;
} __packed;

struct hids_report
{
    uint8_t id; /* report id */
    uint8_t type; /* report type */
} __packed;

static struct hids_info info =
{
    .version = BASE_USB_HID_SPEC_VERSION,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE,
};

enum
{
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_report input =
{
    .id = 0x0,
    .type = HIDS_INPUT,
};

static uint8_t ctrl_point;


#ifdef HID_KEYBOARD

/* Number of bytes in key report
 *
 * 1B - control keys
 * 1B - reserved
 * rest - non-control keys
 */



#define KEY_CTRL_CODE_MIN 224 /* Control key codes - required 8 of them */
#define KEY_CTRL_CODE_MAX 231 /* Control key codes - required 8 of them */
#define KEY_CODE_MIN      0   /* Normal key codes */
#define KEY_CODE_MAX      101 /* Normal key codes */
#define KEY_PRESS_MAX     6   /* Maximum number of non-control keys pressed simultaneously*/

#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)


static struct keyboard_state
{
    uint8_t ctrl_keys_state; /* Current keys state */
    uint8_t reserved;
    uint8_t keys_state[KEY_PRESS_MAX];
} hid_keyboard_state;

static const uint8_t report_map[] =
{
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xA1, 0x01,       /* Collection (Application) */

    /* Keys */
    0x05, 0x07,       /* Usage Page (Key Codes) */
    0x19, 0xe0,       /* Usage Minimum (224) */
    0x29, 0xe7,       /* Usage Maximum (231) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x25, 0x01,       /* Logical Maximum (1) */
    0x75, 0x01,       /* Report Size (1) */
    0x95, 0x08,       /* Report Count (8) */
    0x81, 0x02,       /* Input (Data, Variable, Absolute) */

    0x95, 0x01,       /* Report Count (1) */
    0x75, 0x08,       /* Report Size (8) */
    0x81, 0x01,       /* Input (Constant) reserved byte(1) */

    0x95, 0x06,       /* Report Count (6) */
    0x75, 0x08,       /* Report Size (8) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x25, 0x65,       /* Logical Maximum (101) */
    0x05, 0x07,       /* Usage Page (Key codes) */
    0x19, 0x00,       /* Usage Minimum (0) */
    0x29, 0x65,       /* Usage Maximum (101) */
    0x81, 0x00,       /* Input (Data, Array) Key array(6 bytes) */

    /* LED */
    0x95, 0x05,       /* Report Count (5) */
    0x75, 0x01,       /* Report Size (1) */
    0x05, 0x08,       /* Usage Page (Page# for LEDs) */
    0x19, 0x01,       /* Usage Minimum (1) */
    0x29, 0x05,       /* Usage Maximum (5) */
    0x91, 0x02,       /* Output (Data, Variable, Absolute), */
    /* Led report */
    0x95, 0x01,       /* Report Count (1) */
    0x75, 0x03,       /* Report Size (3) */
    0x91, 0x01,       /* Output (Data, Variable, Absolute), */
    /* Led report padding */

    0xC0              /* End Collection (Application) */
};
#elif defined (HID_CONSUMER)

enum
{
    HIDS_CTRL_PLAY,
    HIDS_CTRL_CONFG,
    HIDS_CTRL_SCAN_NEX,
    HIDS_CTRL_SCAN_PREV,
    HIDS_CTRL_VOL_DOWN,
    HIDS_CTRL_VOL_UP,
    HIDS_CTRL_FWD,
    HIDS_CTRL_BACK,
};


#define KEY_CODE_MIN      HIDS_CTRL_PLAY   /* Normal key codes */
#define KEY_CODE_MAX      HIDS_CTRL_BACK /* Normal key codes */


/* Report as bit..*/
static struct consume_key_state
{
    uint8_t key_state;
} hid_consume_state;


static const uint8_t report_map[] =
{
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x15, 0x00,       // Logical minimum (0)
    0x25, 0x01,       // Logical maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x01,       // Report Count (1)

    0x09, 0xCD,       // Usage (Play/Pause)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB5,       // Usage (Scan Next Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB6,       // Usage (Scan Previous Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

    0x09, 0xEA,       // Usage (Volume Down)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xE9,       // Usage (Volume Up)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x25, 0x02, // Usage (AC Forward)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x24, 0x02, // Usage (AC Back)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0xC0              // End Collection

};
#else
#error "Invalid config"
#endif


/// HID Service Attributes Indexes
enum
{
    HIDS_IDX_SVC,

    HIDS_IDX_INFO_CHAR,
    HIDS_IDX_INFO_VAL,

    HIDS_IDX_REPORT_MAP,
    HIDS_IDX_REPORT_MAP_VAL,

    HIDS_IDX_REPORT,
    HIDS_IDX_REPORT_VAL,
    HIDS_IDX_REPORT_REF,
    HIDS_IDX_REPORT_NTF_CFG,

    HIDS_IDX_CTRL,
    HIDS_IDX_CTRL_VAL,
    HDIS_ATT_NB
};


struct attm_desc hids_att_db[] =
{
    // HID service
    [HIDS_IDX_SVC]              =   {ATT_DECL_PRIMARY_SERVICE,  PERM(RD, ENABLE), 0, 0},

    // HID Info
    [HIDS_IDX_INFO_CHAR]        =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_INFO_VAL]         =   {ATT_CHAR_HID_INFO,         PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(info)},

    // HID Report map
    [HIDS_IDX_REPORT_MAP]       =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_REPORT_MAP_VAL]   =   {ATT_CHAR_REPORT_MAP,       PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(report_map)},

    // HID Report
    [HIDS_IDX_REPORT]           =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_REPORT_VAL]       =   {
        ATT_CHAR_REPORT,
        PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE) | PERM(NTF, ENABLE) |
        PERM(WP, UNAUTH), PERM(UUID_LEN, UUID_16) | PERM(RI, ENABLE),
        8
    },
    [HIDS_IDX_REPORT_NTF_CFG]   =   {ATT_DESC_CLIENT_CHAR_CFG,  PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WP, UNAUTH), PERM(RI, ENABLE), 2},
    [HIDS_IDX_REPORT_REF]       =   {ATT_DESC_REPORT_REF,  PERM(RD, ENABLE), PERM(RI, ENABLE), 2},

    // HID Control
    [HIDS_IDX_CTRL]             =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    [HIDS_IDX_CTRL_VAL]         =   {ATT_CHAR_HID_CTNL_PT,      PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 1},

};

uint8_t *ble_hids_gatts_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    uint8_t *ret_val = NULL;
    *len = 0;

    LOG_I("HIDS get: idx=%d\n", idx);
    switch (idx)
    {
    // HID Info
    case HIDS_IDX_INFO_VAL:
        ret_val = (uint8_t *)&info;
        *len = sizeof(info);
        break;
    case HIDS_IDX_REPORT_MAP_VAL:
        ret_val = (uint8_t *)report_map;
        *len = sizeof(report_map);
        break;
    case HIDS_IDX_REPORT_VAL:
        break;
    case HIDS_IDX_CTRL_VAL:
    {
        ret_val = (uint8_t *)&ctrl_point;
        *len = sizeof(ctrl_point);
        LOG_I("HIDS %s", ctrl_point ? "Exit Suspend" : "Suspend");
        break;
    }
    case HIDS_IDX_REPORT_REF:
    {
        ret_val = (uint8_t *)&input;
        *len = sizeof(input);
    }
    default:
        break;
    }
    return ret_val;
}


uint8_t ble_hids_gatts_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    app_env_t *env = ble_app_get_env();

    LOG_I("HIDS get: idx=%d\n", para->idx);
    switch (para->idx)
    {
    case HIDS_IDX_REPORT_NTF_CFG:
        env->data.is_config_on = *(para->value);
        LOG_I("CCCD %d", env->data.is_config_on);
        break;            ;
    case HIDS_IDX_CTRL_VAL:
        RT_ASSERT(para->len <= sizeof(ctrl_point));
        memcpy(&ctrl_point, para->value, para->len);
        LOG_I("Updated app value to:%x", ctrl_point);
        break;
    default:
        break;
    }
    return 0;
}

static void ble_app_service_init(void)
{
    app_env_t *env = ble_app_get_env();
    sibles_register_svc_t svc;

    svc.att_db = (struct attm_desc *)&hids_att_db;
    svc.num_entry = HDIS_ATT_NB;
    /* Service security level to control all characteristic. */
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_UUID_LEN, UUID_16);
    svc.uuid = ATT_SVC_HID;
    /* Reigster GATT service and related callback for next response. */
    env->data.srv_handle = sibles_register_svc(&svc);
    if (env->data.srv_handle)
    {
        LOG_I("Register HID service handle\n");
        sibles_register_cbk(env->data.srv_handle, ble_hids_gatts_get_cbk, ble_hids_gatts_set_cbk);
    }
    else
    {
        LOG_E("Connection failed\n");
    }
}

/*********************** End of HIDS***********************************************/

/********************** Start of HID Application *********************************/

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

#ifdef HID_KEYBOARD

/** @brief Change key code to ctrl code mask
 *
 *  Function changes the key code to the mask in the control code
 *  field inside the raport.
 *  Returns 0 if key code is not a control key.
 *
 *  @param key Key code
 *
 *  @return Mask of the control key or 0.
 */
static uint8_t button_ctrl_code(uint8_t key)
{
    if (KEY_CTRL_CODE_MIN <= key && key <= KEY_CTRL_CODE_MAX)
    {
        return (uint8_t)(1U << (key - KEY_CTRL_CODE_MIN));
    }
    return 0;
}

static int hid_kbd_state_key_set(uint8_t key)
{
    uint8_t ctrl_mask = button_ctrl_code(key);

    if (ctrl_mask)
    {
        hid_keyboard_state.ctrl_keys_state |= ctrl_mask;
        return 0;
    }
    for (size_t i = 0; i < KEY_CTRL_CODE_MAX; ++i)
    {
        if (hid_keyboard_state.keys_state[i] == 0)
        {
            hid_keyboard_state.keys_state[i] = key;
            return 0;
        }
    }
    /* All slots busy */
    return -EBUSY;
}

static int hid_kbd_state_key_clear(uint8_t key)
{
    uint8_t ctrl_mask = button_ctrl_code(key);

    if (ctrl_mask)
    {
        hid_keyboard_state.ctrl_keys_state &= ~ctrl_mask;
        return 0;
    }
    for (size_t i = 0; i < KEY_CTRL_CODE_MAX; ++i)
    {
        if (hid_keyboard_state.keys_state[i] == key)
        {
            hid_keyboard_state.keys_state[i] = 0;
            return 0;
        }
    }
    /* Key not found */
    return -EINVAL;
}

#elif defined(HID_CONSUMER)

static int hid_consume_state_key_set_bit(uint8_t key)
{
    if ((hid_consume_state.key_state & (1  << key)) == 0)
    {
        hid_consume_state.key_state |= 1  << key;
        return 0;
    }
    return -EBUSY;
}

static int hid_consume_state_key_clear_bit(uint8_t key)
{
    if ((hid_consume_state.key_state & (1  << key)) != 0)
    {
        hid_consume_state.key_state &= ~(1  << key);
        return 0;
    }
    /* All slots busy */
    return -EBUSY;
}
#endif

void key_report_send(uint8_t *key_val, uint16_t key_val_len)
{
    app_env_t *env = ble_app_get_env();
    if (env->data.is_config_on)
    {
        sibles_value_t value;
        value.hdl = env->data.srv_handle;
        value.idx = HIDS_IDX_REPORT_VAL;
        value.len = key_val_len;
        value.value = key_val;
        sibles_write_value(env->conn_idx, &value);
        LOG_I("key_report_send: sibles_write_value called");
    }
}


/********************** End of HID Application *********************************/


SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_bg_context);

static uint8_t ble_app_background_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        LOG_I("Broadcast ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("Broadcast ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}


/* Enable advertise via advertising service. */
static void ble_app_bg_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[] = "SIFLI_BG_INFO";
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;

    para.own_addr_type = GAPM_GEN_NON_RSLV_ADDR;
    para.config.adv_mode = SIBLES_ADV_BROADCAST_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.broadcast_config.duration = 0x0;
    para.config.mode_config.broadcast_config.interval = 0x140;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed .*/
    para.adv_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name, para.adv_data.completed_name->name_len);

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_background_advertising_event;

    ret = sibles_advertising_init(g_app_advertising_bg_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_bg_context);
    }

    rt_free(para.adv_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}




SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

static uint8_t ble_app_advertising_event(uint8_t event, void *context, void *data)
{
    app_env_t *env = ble_app_get_env();

    switch (event)
    {
    case SIBLES_ADV_EVT_ADV_STARTED:
    {
        sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
        if (!env->is_bg_adv_on)
        {
            env->is_bg_adv_on = 1;
            ble_app_bg_advertising_start();
        }
        LOG_I("ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
        break;
    }
    case SIBLES_ADV_EVT_ADV_STOPPED:
    {
        sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
        LOG_I("ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
        break;
    }
    default:
        break;
    }
    return 0;
}


#define DEFAULT_LOCAL_NAME "SIFLI_APP"
/* Enable advertise via advertising service. */
static void ble_app_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    uint8_t ret;

    char local_name[31] = {0};
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf(local_name, 31, "SIFLI_APP-%x-%x-%x-%x-%x-%x", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        memcpy(local_name, DEFAULT_LOCAL_NAME, sizeof(DEFAULT_LOCAL_NAME));

    ble_gap_dev_name_t *dev_name = malloc(sizeof(ble_gap_dev_name_t) + strlen(local_name));
    dev_name->len = strlen(local_name);
    memcpy(dev_name->name, local_name, dev_name->len);
    ble_gap_set_dev_name(dev_name);
    free(dev_name);

    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
    /* Keep advertising till disable it or connected. */
    para.config.mode_config.conn_config.duration = 0x0;
    para.config.mode_config.conn_config.interval = 0x30;
    para.config.max_tx_pwr = 0x7F;
    /* Advertising will re-start after disconnected. */
    para.config.is_auto_restart = 1;
    /* Scan rsp data is same as advertising data. */
    //para.config.is_rsp_data_duplicate = 1;

    /* Prepare name filed. Due to name is too long to put adv data, put it to rsp data.*/
    para.rsp_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.rsp_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.rsp_data.completed_name->name, local_name, para.rsp_data.completed_name->name_len);

    /* Prepare service data filed .*/
    {
        uint16_t uuid_hids = ATT_SVC_HID;
        para.adv_data.completed_uuid = rt_malloc(sizeof(sibles_adv_type_srv_uuid_t) + 1 + sizeof(ATT_UUID_16_LEN));
        para.adv_data.completed_uuid->count = 1;
        para.adv_data.completed_uuid->uuid_list[0].uuid_len = 2;
        memcpy(para.adv_data.completed_uuid->uuid_list[0].uuid.uuid_16, &uuid_hids, para.adv_data.completed_uuid->uuid_list[0].uuid_len);
    }

    /* Prepare appearance filed .*/
    para.adv_data.appearance = rt_malloc(2);
#ifdef HID_KEYBOARD
    // Keyboard (HID subtype)
    uint16_t appearance = 961;
#elif defined(HID_CONSUMER)
    // Mouse (HID subtype)
    uint16_t appearance = 962;
#endif
    memcpy(para.adv_data.appearance, &appearance, 2);

    // set ad type flag
    para.adv_data.disc_mode = GAPM_ADV_MODE_GEN_DISC;

    /* Prepare manufacturer filed .*/
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));

    para.evt_handler = ble_app_advertising_event;

    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }
    rt_free(para.rsp_data.appearance);
    rt_free(para.rsp_data.completed_name);
    rt_free(para.adv_data.manufacturer_data);
}

#ifdef HIDS_TEST

    #ifdef HID_KEYBOARD
        #define HID_KEY_SET(key) hid_kbd_state_key_set(key)
        #define HID_KEY_CLEAR(key) hid_kbd_state_key_clear(key)
        #define HID_KEY_SEND() key_report_send((uint8_t *)&hid_keyboard_state, sizeof(hid_keyboard_state))
    #elif defined(HID_CONSUMER)
        #define HID_KEY_SET(key) hid_consume_state_key_set_bit(key)
        #define HID_KEY_CLEAR(key) hid_consume_state_key_clear_bit(key)
        #define HID_KEY_SEND() key_report_send((uint8_t *)&hid_consume_state, sizeof(hid_consume_state))
    #endif
#endif
#ifdef HID_KEYBOARD
static void key_button_handler(int32_t pin, button_action_t action)
{
    uint8_t hid_code = key_mapping_table[0].hid_code;
    switch (action)
    {
    case BUTTON_PRESSED:
        HID_KEY_SET(hid_code);
        HID_KEY_SEND();
        break;
    case BUTTON_RELEASED:
        HID_KEY_CLEAR(hid_code);
        HID_KEY_SEND();
        break;
    default:
        break;
    }
}

static void key_button_init(void)
{
    button_cfg_t key_cfg =
    {
        .pin = BSP_KEY1_PIN,
        .mode = PIN_MODE_INPUT_PULLUP,
        .active_state = BUTTON_ACTIVE_HIGH,
        .button_handler = key_button_handler,
    };
    int key_id = button_init(&key_cfg);
    if (key_id >= 0)button_enable(key_id);


}
#endif
int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
#ifdef HID_KEYBOARD
    key_button_init(); // Initialize key button
#endif
    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
            ble_app_service_init();
            /* First enable connectable adv then enable non-connectable. */
            ble_app_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}


static void ble_app_update_conn_param(uint8_t conn_idx, uint16_t inv_max, uint16_t inv_min, uint16_t timeout)
{
    ble_gap_update_conn_param_t conn_para;
    conn_para.conn_idx = conn_idx;
    conn_para.intv_max = inv_max;
    conn_para.intv_min = inv_min;
    /* value = argv * 1.25 */
    conn_para.ce_len_max = 0x100;
    conn_para.ce_len_min = 0x1;
    conn_para.latency = 0;
    conn_para.time_out = timeout;
    ble_gap_update_conn_param(&conn_para);
}

int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (env->mb_handle)
            rt_mb_send(env->mb_handle, BLE_POWER_ON_IND);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->conn_para.conn_interval = ind->con_interval;
        env->conn_para.peer_addr_type = ind->peer_addr_type;
        env->conn_para.peer_addr = ind->peer_addr;
        if (ind->role == 0)
            LOG_E("Peripheral should be slave!!!");

        LOG_I("Peer device(%x-%x-%x-%x-%x-%x) connected", env->conn_para.peer_addr.addr[5],
              env->conn_para.peer_addr.addr[4],
              env->conn_para.peer_addr.addr[3],
              env->conn_para.peer_addr.addr[2],
              env->conn_para.peer_addr.addr[1],
              env->conn_para.peer_addr.addr[0]);
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        env->conn_para.conn_interval = ind->con_interval;
        LOG_I("Updated connection interval :%d", ind->con_interval);
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->conn_para.mtu = ind->mtu;
        LOG_I("Exchanged MTU size: %d", ind->mtu);
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        LOG_I("BLE_GAP_DISCONNECTED_IND(%d)", ind->reason);
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);


#ifdef HIDS_TEST

static rt_err_t test_hids(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("usage: test_hids key [p|r] \n");
    }
    else
    {
        uint8_t key = (uint8_t)atoi(&argv[1][0]);
        if (argv[2][0] == 'p')
            HID_KEY_SET(key);
        else
            HID_KEY_CLEAR(key);
        HID_KEY_SEND();
    }
    return 0;
}
FINSH_FUNCTION_EXPORT(test_hids, Test HIDS);
MSH_CMD_EXPORT(test_hids, Test HIDS);
#endif



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

