/**
  ******************************************************************************
  * @file   ble_connection_manager.h
  * @author Sifli software development team
  * @brief Header file - Sibles connection manager.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#ifndef _BLE_CONNECTION_MANAGER_H_
#define _BLE_CONNECTION_MANAGER_H_
#include "bf0_ble_gap.h"

#define MAX_CONNECTION_LINK_NUM 8
#define MAX_PAIR_DEV 3

#define NOTIFY_ENABLE_VALUE 1
#define INDICATION_ENABLE_VALUE 2

typedef struct
{
    uint8_t conn_idx;
    uint8_t bond_index;
    uint8_t bond_state;
    uint8_t auth_state;
    uint8_t connection_state;
    uint8_t enc_state;
    uint8_t svc_changed_ccc;
    uint8_t first_bond;
    uint8_t role;

    // Address type of the device 0=public/1=private random
    uint8_t peer_addr_type;
    uint16_t connection_interval;
    uint16_t connection_latency;
    uint16_t supervision_timeout;
    bd_addr_t peer_addr;
    bool sec_con_enabled;
    uint16_t mtu;
    uint8_t update_state;
} conn_manager_t;

typedef struct
{
    // set priority to 0 as available
    // new dev will be set priority 1
    // other paired dev's priority will +1
    // set priority to MAX_PAIR_DEV as oldest
    // oldest dev will be replaced by new pari dev
    uint8_t priority[MAX_PAIR_DEV];
    // use to fill a fake device in resolve list when rpa enable
    uint8_t fixed[MAX_PAIR_DEV];
    ble_gap_addr_t peer_addr[MAX_PAIR_DEV];
    ble_gap_ltk_t ltk[MAX_PAIR_DEV];
    ble_gap_sec_key_t local_irk[MAX_PAIR_DEV];
    ble_gap_sec_key_t peer_irk[MAX_PAIR_DEV];
    uint8_t auth[MAX_PAIR_DEV];
    bool ltk_present[MAX_PAIR_DEV];
} conn_manager_pair_dev_t;
typedef enum
{
    REQUST_LTK_ILK_H6,
    REQUST_LTK_ILK_H7,
    REQUST_ILK_LK_H6,
    OTHER
} CB_REQUEST_TYPE;
typedef struct
{
    uint8_t   ilk[GAP_KEY_LEN];
} bt_ilk_t;
typedef struct
{
    uint8_t   lk[GAP_KEY_LEN];
} bt_lk_t;
typedef struct
{
    bt_ilk_t ilk[MAX_PAIR_DEV];
    bt_lk_t  lk[MAX_PAIR_DEV];
} conn_manager_pair_dev_map_info_t;

typedef struct
{
    uint8_t priority[MAX_PAIR_DEV];
    uint8_t fixed[MAX_PAIR_DEV];
    ble_gap_addr_t peer_addr[MAX_PAIR_DEV];
} conn_manager_get_bonded_dev_t;

typedef struct
{
    ble_gap_addr_t peer_addr;
    ble_gap_ltk_t ltk;
    ble_gap_sec_key_t local_irk;
    ble_gap_sec_key_t peer_irk;
} conn_manager_bonding_dev_t;

typedef struct
{
    uint8_t device_count;
    uint8_t conn_idx[MAX_CONNECTION_LINK_NUM];
} conn_manager_get_connected_dev_t;

struct cm_connection_cfm
{
    /// Local CSRK value
    ble_gap_sec_key_t lcsrk;
    /// Local signature counter value
    uint32_t lsign_counter;

    /// Remote CSRK value
    ble_gap_sec_key_t rcsrk;
    /// Remote signature counter value
    uint32_t rsign_counter;

    /// Authentication (@see gap_auth)
    uint8_t auth;
    /// Service Changed Indication enabled
    uint8_t svc_changed_ind_enable;
    /// LTK exchanged during pairing.
    bool ltk_present;
};

typedef struct
{
    bool accept;
    uint8_t oob;
    uint8_t sec_req;
    uint8_t iocap;
    uint8_t auth;
} conn_manager_bond_cnf_information;

typedef struct
{
    uint8_t remote_index;
    uint16_t remote_handle;
    uint16_t cccd_hdl;
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;
} conn_manager_svc_change;

typedef struct
{
    bd_addr_t local_addr;
    // in some situation, index can not pass to another func
    uint8_t manager_index;

    bool bond_role_master;

    uint8_t security_level;
    // when booting under dual core, we need read flash in ble_stack_init,
    // which is early than power on ind. if not the adv may start early than update rslv list.
    uint8_t update_after_read;

    // default auth when bond none device connected
    // bonded device will use last connected auth
    uint8_t connected_auth;

    uint8_t bond_ack; // @see enum conn_manager_bond_ack_t
    uint8_t remote_auth;
    uint8_t oob_data[GAP_KEY_LEN];
    uint8_t set_app_bd_addr;
    uint8_t is_rslv_update_succ; // 0 is not update; 1 is succ; 0xFF is failed.

    uint8_t is_init;
    uint8_t is_pub_key_gen;

    // global key distribution for initiator key/responder key
    uint8_t key_dist;
    // display value
    uint32_t pin_code;

    // default bond confirm
    conn_manager_bond_cnf_information bond_cnf_info;
#ifdef BLE_SVC_CHG_ENABLE
    conn_manager_svc_change svc_change;
#endif //BLE_SVC_CHG_ENABLE
    ble_gap_oob_t sec_oob_data;
    rt_timer_t update_timer;
} connection_manager_env_t;


typedef enum
{
    BOND_ACCEPT,
    BOND_PENDING,
    BOND_REJECT,
} conn_manager_bond_ack_t;

typedef struct
{
    uint8_t conn_idx;
    uint8_t request;
    uint32_t confirm_data;
} connection_manager_bond_ack_infor_t;

typedef struct
{
    uint16_t local_mtu;
    uint16_t remote_mtu;
} connection_manager_gatt_over_bredr_mtu_ind_t;

typedef struct
{
    uint32_t handle;
} connection_manager_gatt_over_bredr_reg_ind_t;

///BD address type
enum
{
    ///Public BD address
    CM_ADDR_PUBLIC                   = 0x00,
    ///Random BD Address
    CM_ADDR_RAND,
    /// Controller generates Resolvable Private Address based on the
    /// local IRK from resolving list. If resolving list contains no matching
    /// entry, use public address.
    CM_ADDR_RPA_OR_PUBLIC,
    /// Controller generates Resolvable Private Address based on the
    /// local IRK from resolving list. If resolving list contains no matching
    /// entry, use random address.
    CM_ADDR_RPA_OR_RAND,
    /// mask used to determine Address type in the air
    CM_ADDR_MASK                     = 0x01,
    /// mask used to determine if an address is an RPA
    CM_ADDR_RPA_MASK                 = 0x02,
    /// Random device address (controller unable to resolve)
    CM_ADDR_RAND_UNRESOLVED          = 0xFE,
    /// No address provided (anonymous advertisement)
    CM_ADDR_NONE                     = 0xFF,
};

enum connection_manager_state
{
    BOND_STATE_NONE = 0x10,
    BOND_STATE_BONDING = 0x11,
    BOND_STATE_BONDED = 0x12,

    CREATE_BOND = 0x13,
    CANCEL_BOND = 0x14,
    REMOVE_BOND = 0x15,
    LINK_KEY_REQUEST = 0x16,

    ENC_STATE_NONE = 0x17,
    ENC_STATE_ON = 0x18,

    CONNECTION_STATE_DISCONNECTED = 0x20,
    CONNECTION_STATE_CONNECTING = 0x21,
    CONNECTION_STATE_DISCONNECTING = 0x22,
    CONNECTION_STATE_CONNECTED = 0x23,
    CONNECTION_STATE_RESOLVING = 0x24,


    BOND_STATE_CHANGE_EVENT = 0x30,
    CONNECTION_STATE_CHANGE_EVENT = 0x31,
    ENCRYPT_IND_EVENT = 0x32,

    CM_CONNECTED_IND = 0x40,
    CM_DISCONNECTED_IND = 0x41,
    CM_BOND_IND = 0x42,
    CM_BOND_REQ_IND = 0x43,
    CM_PAIRING_SUCCEED = 0x44,
    CM_PAIRING_FAILED = 0x45,
    CM_READ_REMOTE_FEATURE_IND = 0x46,
    CM_FLASH_WRITE = 0x47,
    CM_FLASH_READ = 0x48,
    CM_UPDATE_CONN_IND = 0x49,
    CM_BOND_AUTH_INFOR_CONFIRM = 0x4A,
    CM_UPDATING_PARA_IND = 0x4B,
};

enum connection_manager_event_t
{
    CONNECTION_MANAGER_CONNCTED_IND = BLE_CONNECTION_MANAGER_TYPE,
    CONNECTION_MANAGER_DISCONNECTED_IND,
    CONNECTION_MANAGER_BOND_IND,
    CONNECTION_MANAGER_BOND_REQ_IND,
    CONNECTION_MANAGER_PAIRING_SUCCEED,
    CONNECTION_MANAGER_PAIRING_FAILED,
    CONNECTION_MANAGER_READ_REMOTE_FEATURE_IND,
    CONNECTION_MANAGER_FLASH_WRITE,
    CONNECTION_MANAGER_FLASH_READ,
    CONNECTION_MANAGER_ENCRYPT_IND_EVENT,
    CONNECTION_MANAGER_UPDATE_CONNECTION_IND,
    CONNECTION_MANAGER_BOND_AUTH_INFOR,
    CONNECTION_MANAGER_GATT_OVER_BREDR_REG_IND,
    CONNECTION_MANAGER_GATT_OVER_BREDR_MTU_IND,
    CONNECTION_MANAGER_UPDATING_CONNECTION_PARAMETER_IND,
};

enum connection_manager_status
{
    CM_STATUS_OK = 0,
    CM_ADDR_ERROR = 1,
    CM_SECURITY_LEVEL_ERROR = 3,
    CM_CONNECITON_STATE_MAX_DEVICES = 4,
    CM_PARAMETER_ERROR = 5,
    CM_FLASH_PEDING = 6,
    CM_FLASH_FAIL = 7,
    CM_BOND_CFM_COMMAND_ERROR = 8,
    CM_BOND_ACK_ERROR = 9,
    CM_PARAMETER_UPDATING = 10,
    CM_PARAMETER_SAME = 11,

    CM_CONN_INDEX_EMPTY = 0xF0,
    CM_CONN_INDEX_ERROR = 0xF1,
    CM_CONN_OUT_OF_MEMORY = 0xF2,
};

enum connection_manager_update_conneciton
{
    CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE = 1,
    CONNECTION_MANAGER_INTERVAL_BALANCED = 2,
    CONNECTION_MANAGER_INTERVAL_LOW_POWER = 3,
    CONNECTION_MANAGER_INTERVAL_CUSTOMIZE = 4,

    CONNECTION_MANAGER_INTERVAL_MIN = 0x0006,
    CONNECTION_MANAGER_INTERVAL_MAX = 0x0C80,

    CONNECTION_MANAGER_LATENCY_MIN = 0x00,
    CONNECTION_MANAGER_LATENCY_MAX = 0x01F3,

    CONNECTION_MANAGER_TIME_OUT_MIN = 0x0a,
    CONNECTION_MANAGER_TIME_OUT_MAX = 0x0C80,
};

enum connection_manager_update_conneciton_state
{
    UPDATE_PARAMETER_NONE = 0,
    UPDATE_PARAMETER_UPDATING = 1,
};

enum connection_manager_static_random_mode
{
    STATIC_RANDOM_ADDR = 1,
    STATIC_RANDOM_ADDR_TEMP = 2,
};

enum connection_manager_update_type
{
    UPDATE_TYPE_LL,
    UPDATE_TYPE_L2CAP,
};

typedef struct
{
    uint16_t connection_interval_min;
    uint16_t connection_interval_max;
    uint16_t connection_latency;
    uint16_t supervision_timeout;
} cm_customize_parameter;

typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Reason of disconnection(@see enum co_error)
    uint8_t reason;
} connection_manager_disconnected_ind_t;

typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Connection interval, time = N * 1.25ms
    uint16_t con_interval;
    /// Connection latency
    uint16_t con_latency;
    /// Link supervision timeout, time = N * 10 ms
    uint16_t sup_to;
    /// Peer address type
    uint8_t peer_addr_type;
    /// Peer BT address
    bd_addr_t peer_addr;
    /// Role of device in connection (0 = Master / 1 = Slave)
    uint8_t role;
    /// Need one and only one user to fill this filed to provide link configurations.
    ble_gap_connect_configure_t config_info;
} connection_manager_connect_ind_t;

typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// 8-byte array for LE features
    uint8_t features[GAP_LE_FEATS_LEN];
} connection_manager_remote_features_ind_t;

typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Connection interval value
    uint16_t            con_interval;
    /// Connection latency value
    uint16_t            con_latency;
    /// Supervision timeout
    uint16_t            sup_to;
} connection_manager_update_conn_param_ind_t;

enum update_conneciton_parameter
{
    CONNECTION_MANAGER_UPDATE_CONNECITON_OK = 0x00,
    CONNECTION_MANAGER_UPDATE_CONNECITON_INTERVAL_ERROR = 0x01,

    HIGH_PREFORMENCE_INTERVAL_MIN = 12,
    HIGH_PERFORMANCE_INTERVAL_MAX = 12,
    HIGH_PERFORMANCE_LATENCY = 0,

    BANLANCED_INTERVAL_MIN = 24,
    BANLANCED_INTERVAL_MAX = 40,
    BANLANCED_LANTENCY = 0,

    LOW_POWER_INTERVAL_MIN = 80,
    LOW_POWER_INTERVAL_MAX = 100,
    LOW_POWER_LATENCY = 2,

    // iOS support max supervision timeout value is 6s
    DEFAULT_TIMEOUT = 600,
};

enum connection_manager_security_level
{
    LE_SECURITY_LEVEL_NO_MITM_NO_BOND = 0,
    LE_SECURITY_LEVEL_NO_MITM_BOND = 1,
    LE_SECURITY_LEVEL_MITM_NO_BOND = 2,
    LE_SECURITY_LEVEL_MITM_BOND = 3,
    LE_SECURITY_LEVEL_SEC_CON_NO_BOND = 4,
    LE_SECURITY_LEVEL_SEC_CON_BOND = 5,
    LE_SECURITY_LEVEL_SEC_CON_MITM_BOND = 10,
};

typedef struct
{

    uint16_t interval;
    uint16_t slave_latency;
    uint16_t supervision_timeout;
} cm_conneciont_parameter_value_t;


typedef struct
{
    /// Connection index
    uint8_t conn_idx;
    /// Authentication  level (@see gap_auth)
    uint8_t auth;
} connection_manager_encrypt_ind_t;

typedef struct
{
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t latency;
    uint8_t  interval_level;
} connection_para_updata_ind_t;

extern connection_para_updata_ind_t ble_connect_para_ind;

typedef struct
{
    uint8_t conn_idx;
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t slave_latency;
    uint16_t timeout;
} connection_para_updating_ind_t;

/**
 * @brief update conneciton parameters, default update by l2cap
 * @param[conn_idx] connection index.
 * @param[interval_level] interval level to be set, @see connection_manager_update_conneciton.
 * @param[data] conneciton parameters vlaue, @see cm_conneciont_parameter_value_t.
 * @return set status @see connection_manager_state
 */
uint8_t connection_manager_update_parameter(uint8_t conn_idx, uint8_t interval_level, uint8_t *data);

/**
 * @brief update conneciton parameters.
 * @param[conn_idx] connection index.
 * @param[interval_level] interval level to be set, @see connection_manager_update_conneciton.
 * @param[data] conneciton parameters vlaue, @see cm_conneciont_parameter_value_t.
 * @param[type] update by LL or l2cap, @see connection_manager_update_type.
 * @return set status @see connection_manager_state
 */
uint8_t connection_manager_update_parameter_with_type(uint8_t conn_idx, uint8_t interval_level,
        uint8_t *data, enum connection_manager_update_type type);

/**
 * @brief get current conneciton parameters on specified conneciton index.
 * @param[conn_idx] connection index.
 * @param[data] conneciton parameters vlaue, @see cm_conneciont_parameter_value_t.
 * @return set status @see connection_manager_state
 */
uint8_t connection_manager_get_connetion_parameter(uint8_t conn_idx, uint8_t *data);

/**
 * @brief set link security level.
 * @param[conn_idx] connection index.
 * @param[sec_level] conneciton parameters vlaue, @see connection_manager_security_level.
 * @return set status @see connection_manager_state
 */
uint8_t connection_manager_set_link_security(uint8_t conn_index, uint8_t sec_level);

/**
 * @brief get bond state on specified conneciton index.
 * @param[conn_idx] connection index.
 * @return bond state @see connection_manager_state
 */
uint8_t connection_manager_get_bond_state(uint8_t conn_idx);

/**
 * @brief get connection state on specified conneciton index
 * @param[conn_idx] connection index.
 * @return bond state @see connection_manager_state
 */
uint8_t connection_manager_get_connection_state(uint8_t conn_idx);

/**
 * @brief get all connected device's index
 * @param[data] use struct conn_manager_get_connected_dev_t
 * device_count will return connected device count
 * conn_idx[device_count] will return index of these devices
 */
void connection_manager_get_all_connected_index(uint8_t *data);

/**
 * @brief get address value and type on specified conneciton index
 * @param[conn_idx] connection index.
 * @return bond state @see connection_manager_state
 */
uint8_t connection_manager_get_addr_by_conn_idx(uint8_t conn_idx, ble_gap_addr_t *data);

/**
 * @brief set bond req confirm accecpt
 * @param[accept] set true if accept pair request, set false to refuse.
 * @return always return OK
 */
uint8_t connection_manager_set_bond_cnf_accept(bool accept);

/**
 * @brief set bond req confirm io capabilities
 * @param[iocap] @see gap_io_cap, if not set, default value will be used
 * @return CM_STATUS_OK if set success, @see connection_manager_status
 */
uint8_t connection_manager_set_bond_cnf_iocap(uint8_t iocap);

/**
 * @brief set bond req confirm sec
 * @param[oob] @see gap_sec_req, if not set, default value will be used
 * @return CM_STATUS_OK if set success, @see connection_manager_status
 */
uint8_t connection_manager_set_bond_cnf_sec(uint8_t sec_req);

/**
 * @brief set bond req confirm oob
 * @param[oob] @see gap_oob, if not set, default value will be used
 * @return CM_STATUS_OK if set success, @see connection_manager_status
 */
uint8_t connection_manager_set_bond_cnf_oob(uint8_t oob);

/**
 * @brief set bond req confirm auth
 * @param[auth] @see gap_auth, if not set, default value will be used
 * @return CM_STATUS_OK if set success, @see connection_manager_status
 */
uint8_t connection_manager_set_bond_cnf_auth(uint8_t auth);

/**
 * @brief get all bonded device list
 * @param[data] @see conn_manager_get_bonded_dev_t, will assign the data pointer
 * @return bonded device count
 */
uint8_t connection_manager_get_bonded_devices(uint8_t *data);

/**
 * @brief delete bonded device
 * @param[peer_addr] device that will delete bond, include addr and type
 * @return CM_STATUS_OK if delete success, CM_ADDR_ERROR if device not found,
 * CM_FLASH_PEDING to see later callback event CM_FLASH_WRITE,
 * CM_FLASH_FAIL if set fail, see error code
 */
uint8_t connection_manager_delete_bond(ble_gap_addr_t peer_addr);

/**
 * @brief delete all bonded device
 * @return CM_STATUS_OK if delete success,
 * CM_FLASH_PEDING to see later callback event CM_FLASH_WRITE,
 * CM_FLASH_FAIL if set fail, see error code
 */
uint8_t connection_manager_delete_all_bond();

/**
 * @brief set auth that bond none device connected to use.
 * @param[peer_addr] @see gap_auth, if not set, default value will be used
 * @return CM_STATUS_OK if set success, @see connection_manager_status
 */
uint8_t connection_manager_set_connected_auth(uint8_t auth);

/**
 * @brief get remote feature
 * @param[conn_idx] conn_idx
 * @return CM_STATUS_OK if get cmd send success, @see connection_manager_status
 */
uint8_t connection_manager_get_remote_feature(uint8_t conn_idx);

/**
 * @brief set data for out of band pair
 * @param[oob_data] oob data
 * @return CM_STATUS_OK if set data success
 */
uint8_t connection_manager_set_oob_data(uint8_t *oob_data, uint8_t length);

/**
 * @brief set security data for out of band pair
 * @param[ble_gap_oob_t] oob data include confirm value and random key
 * @return CM_STATUS_OK if set data success, @see ble_gap_oob_t
 */
uint8_t connection_manager_set_sec_oob_data(ble_gap_oob_t *oob_data);

/**
 * @brief disconnect target ble link
 * @param[conn_index] connection index
 */
void connection_manager_disconnect(uint8_t conn_index);

uint8_t read_bond_infor_from_flash_start_up();

void clear_resolving_list();
void update_resolving_list();
void add_white_list();

void start_enc(uint8_t conn_idx);


/**
 * @brief set current security level, shall be set before connect
 * @param[level] 1, 2, 3, 4 see function to see different level
 * @return 0
 */
uint8_t connection_manager_set_security_level(int level);

/**
 * @brief create bond on given connection index's link
 * @param[conn_idx] connection index
 */
void connection_manager_create_bond(uint8_t conn_idx);
void connection_manager_create_bond_bqb(uint8_t conn_idx, uint8_t command);

/**
 * @brief set bond response procedure
 * @param[state] @see conn_manager_bond_ack_t
 * set BOND_ACCEPT will reply accept to remote immediately
 * set BOND_REJECT will reply reject to remote immediately
 * set BOND_PENDING will send a message CONNECTION_MANAGER_BOND_AUTH_INFOR,
 * and user shall call connection_manager_bond_ack_reply to reply remote.
 * @return CM_STATUS_OK if set success
 */
uint8_t connection_manager_set_bond_ack(uint8_t state);

/**
 * @brief use to reply remote if set bond ack BOND_PENDING by connection_manager_set_bond_ack
 * @param[conn_idx] connection index
 * @param[command] command need reply to remote,
 * should be the same in message CONNECTION_MANAGER_BOND_AUTH_INFOR's request, @see connection_manager_bond_ack_infor_t
 * @param[accept] set true if accpet, set false if reject
 * @return CM_STATUS_OK if send to stack success
 */
uint8_t connection_manager_bond_ack_reply(uint8_t conn_idx, uint8_t command, bool accept);

/**
 * @brief get current encryption state
 * @param[conn_idx] conn_idx
 * @return ENC_STATE_NONE if given conn_idx's link is not encrypted.
 * ENC_STATE_ON if given conn_idx's link is encrypted.
 */
uint8_t connection_manager_get_enc_state(uint8_t conn_idx);

bool connection_manager_check_conn_idx(uint8_t conn_idx);
bool connection_manager_check_normal_conn_idx(uint8_t conn_idx);

/**
 * @brief use public address, will take effect after next power on.
 */
void connection_manager_use_public_addr();

/**
 * @brief set static random address, will take effect after next power on.
 */
void connection_manager_set_random_addr();

/**
 * @brief set temp random address, will take effect immediately, recommend to use connection_manager_set_random_addr
 */
void connection_manager_set_temp_random();

/**
 * @brief stop set temp random address
 */
void connection_manager_stop_temp_random(uint8_t conn_idx);
void connection_manager_h6_result_cb(uint8_t *aes_res, uint32_t metainfo);
void connection_manager_h7_result_cb(uint8_t *aes_res, uint32_t metainfo);
void connection_manager_convert_ilk_to_lk(int i);
void connection_manager_convert_ltk_to_ilk(int i);

/**
 * @brief return 1 if success, 0 not find target uuid
 */
uint8_t connection_manager_gatt_over_bredr_service_register(uint16_t uuid);
uint8_t connection_manager_gatt_over_bredr_service_register_128(uint8_t *uuid);

/**
 * @brief Customisation of preset connection parameter values
 * @param[interval_min] connection interval min
 * @param[interval_max] connection interval max
 * @param[latency] slave latency
 * @param[timeout] supervision timeout, unit is 10ms
 */
void connection_parameter_get_high_performance(uint16_t *interval_min, uint16_t *interval_max, uint16_t *latency, uint16_t *timeout);
void connection_parameter_get_balance(uint16_t *interval_min, uint16_t *interval_max, uint16_t *latency, uint16_t *timeout);
void connection_parameter_get_low_power(uint16_t *interval_min, uint16_t *interval_max, uint16_t *latency, uint16_t *timeout);

//extern uint8_t ble_gap_aes_h6(uint8_t *w, uint8_t *key_id, uint32_t cb_request);
//extern uint8_t ble_gap_aes_h7(uint8_t *salt, uint8_t *w, uint32_t metainfo);
#endif
