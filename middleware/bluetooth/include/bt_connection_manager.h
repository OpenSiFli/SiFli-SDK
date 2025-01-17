/**
  ******************************************************************************
  * @file   bt_connection_manager.h
  * @author Sifli software development team
  * @brief Header file - bt connection manager.
 *
  ******************************************************************************
*/
/**
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

#ifndef _BT_CONNECTION_MANAGER_H_
#define _BT_CONNECTION_MANAGER_H_

#ifdef BSP_BT_CONNECTION_MANAGER
#include "config.h"
#include "bts2_bt.h"
#include "bts2_app_demo.h"
#include "drivers/bt_device.h"

#ifdef CFG_MAX_BT_ACL_NUM
    #define BT_CM_MAX_CONN CFG_MAX_BT_ACL_NUM
#else
    #define BT_CM_MAX_CONN 1
#endif
#ifdef CFG_MAX_BT_BOND_NUM
    #define BT_CM_MAX_BOND CFG_MAX_BT_BOND_NUM
#else
    #define BT_CM_MAX_BOND 2
#endif

#define BT_CM_INVALID_CONN_INDEX (0xFF)
#define BT_CM_DEFAULT_MASTER_BIT (BT_CM_A2DP|BT_CM_AVRCP)


#ifdef CFG_OPEN_AVSNK
    #define BT_CM_DEFAULT_SLAVE_BIT  (BT_CM_HFP | BT_CM_A2DP)
#else
    #define BT_CM_DEFAULT_SLAVE_BIT  (BT_CM_HFP)
#endif

#ifdef CFG_HID
    #define BT_CM_DEFAULT_SLAVE_BIT_EXT  (BT_CM_DEFAULT_SLAVE_BIT | BT_CM_HID)
#else
    #define BT_CM_DEFAULT_SLAVE_BIT_EXT  BT_CM_DEFAULT_SLAVE_BIT
#endif


#define BT_CM_HFP       (0x01 << 0)
#define BT_CM_A2DP      (0x01 << 1)
#define BT_CM_AVRCP     (0x01 << 2)
#define BT_CM_HID       (0x01 << 3)
#define BT_CM_PAN       (0x01 << 4)

#define BT_BASIC_PROFILE (BT_CM_HFP | BT_CM_A2DP | BT_CM_AVRCP)


// Master device means audio provider, not BT's role
typedef enum
{
    BT_CM_MASTER,
    BT_CM_SLAVE
} bt_cm_conn_role_t;

typedef enum
{
    BT_CM_ERR_NO_ERR = 0,
    BT_CM_ERR_INVALID_PARA = 1,
    BT_CM_ERR_UNSUPPORTED = 2,
    BT_CM_ERR_GENERAL_ERR = 3,
    BT_CM_ERR_CONN_EXISTED = 4,
    BT_CM_ERR_RESOURCE_NOT_ENOUGH = 5,
} bt_cm_err_t;

typedef enum
{
    BT_CM_NO_CLOSE,
    BT_CM_ON_CLOSE_PROCESS,
    BT_CM_CLOSE_COMPLETE
} bt_cm_close_status_t;

typedef enum
{
    BT_CM_STATE_IDLE,
    BT_CM_STATE_CONNECTING,
    BT_CM_STATE_RECONNECTING,
    BT_CM_STATE_CONNECTED,
} bt_cm_fsm_t;

typedef enum
{
    BT_CM_SUB_STATE_IDLE,
    BT_CM_SUB_PROFILING_CONNECTING,
} bt_cm_sub_fsm_t;


typedef struct
{
    BTS2S_BD_ADDR bd_addr;
    uint32_t dev_cls;
    bt_cm_conn_role_t role;
    uint8_t is_reconn;
} bt_cm_conn_info_t;

typedef struct
{
    bt_cm_conn_info_t info;
    uint32_t conned_profiles;
    rt_timer_t tim_hdl;
    uint16_t conn_hdl;
    // Curretly connect as master or slave;
    uint8_t incoming;
    uint8_t state;
    uint8_t sub_state;
    uint8_t sniff_changing;
    //uint8_t rmt_smc;//1:remote device indicate encryption change;
} bt_cm_conned_dev_t;

typedef struct
{
    bt_cm_conn_info_t info[BT_CM_MAX_BOND];
    uint8_t dev_state[BT_CM_MAX_BOND];
    uint8_t g_bt_cm_last_bond_idx;
} bt_cm_bonded_dev_t;

// Added reconnect flag and addr
typedef struct
{
    bt_cm_conned_dev_t conn_device[BT_CM_MAX_CONN];
    bt_cm_conn_role_t cur_role;
    bt_cm_close_status_t close_process;
} bt_cm_env_t;


void init_bt_cm();
void set_last_connect_a2dp_device(BTS2S_BD_ADDR *bd);
void connect_bt_a2dp();
void set_app_data(bts2_app_stru *bts2_app_data);
bt_cm_err_t bt_cm_connect_req(BTS2S_BD_ADDR *bd_addr, bt_cm_conn_role_t role);
int bt_cm_close_bt(void);
int bt_cm_open_bt(void);
void bt_cm_set_profile_target(uint32_t setProfile, bt_cm_conn_role_t role, uint8_t addFlag);
void bt_cm_delete_bonded_devs(void);
void bt_cm_delete_bonded_devs_and_linkkey(uint8_t *addr);
void bt_cm_reconnect_last_device(void);
uint8_t bt_cm_last_device_is_valid(void);
void bt_cm_disconnect_req(void);
void bt_cm_last_device_bd_addr(bt_mac_t *bd_addr_c);
void bt_cm_change_page_activity(uint8_t is_high);
bt_cm_conned_dev_t *bt_cm_get_free_conn(bt_cm_env_t *env);
bt_cm_conned_dev_t *bt_cm_find_conn_by_addr(bt_cm_env_t *env, BTS2S_BD_ADDR *bd_addr);
bt_cm_env_t *bt_cm_get_env();
bt_cm_bonded_dev_t *bt_cm_get_bonded_dev(void);
bt_cm_conn_info_t *bt_cm_find_bonded_dev_by_addr(uint8_t *addr);
void bt_cm_change_inquiryscan_activity(uint8_t is_high);
uint8_t bt_cm_find_conn_index_by_addr(uint8_t *addr);
uint8_t bt_cm_find_addr_by_conn_index(uint8_t idx, BTS2S_BD_ADDR *addr);
void bt_cm_add_bonded_dev(bt_cm_conn_info_t *dev, uint8_t force);
#endif // BSP_BT_CONNECTION_MANAGER
#endif // _BT_CONNECTION_MANAGER_H_
