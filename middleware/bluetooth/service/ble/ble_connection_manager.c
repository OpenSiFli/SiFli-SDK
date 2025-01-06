/**
  ******************************************************************************
  * @file   ble_connection_manager.c
  * @author Sifli software development team
  * @brief Sibles connection manager.
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


#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles.h"
#include "att.h"
#include "bf0_ble_gap.h"
#include "bluetooth_int.h"

#include "ble_connection_manager.h"
#include "bf0_ble_gap_internal.h"
#include "bf0_sibles_nvds.h"
#include "stdlib.h"

#ifdef BT_USING_GATT
    #include "bts2_msg.h"
    #include "bt_gatt_api.h"
#endif


//#if defined(SOC_SF32LB58X) && defined(BT_FINSH)
#ifdef BT_FINSH
    extern void sc_ble_bt_link_key_ind(uint8_t *p_bd, uint8_t key_type, uint8_t key[GAP_KEY_LEN]);
#endif

//#define DBG_LVL               DBG_INFO
#define DBG_TAG "ble_cm"
#include "log.h"


#ifdef BSP_BLE_CONNECTION_MANAGER


#define ALLOCATE_CONNECTION_FAIL 0xFF
#define ADVERTISE_MODE_LOW_LANTECY 0  // 100ms
#define ADVERTISE_MODE_LOW_POWER 1  // 1s
#define ADVERTISE_MODE_BALANCED 2  // 250ms

connection_para_updata_ind_t ble_connect_para_ind = {LOW_POWER_INTERVAL_MIN, LOW_POWER_INTERVAL_MAX, LOW_POWER_LATENCY, CONNECTION_MANAGER_INTERVAL_LOW_POWER};
const uint8_t generic_attribute_uuid[ATT_UUID_16_LEN] =
{
    0x01, 0x18,
};

const uint8_t svc_changed_uuid[ATT_UUID_16_LEN] =
{
    0x05, 0x2a,
};

void connection_manager_connection_state_change(uint8_t manager_index, uint8_t new_states, uint16_t event);

// get bonded dev from nv
conn_manager_pair_dev_t g_bond_info;
conn_manager_pair_dev_map_info_t g_bond_map_info;
// current bonding device info
conn_manager_bonding_dev_t g_bonding_info;
// connected device info
conn_manager_t g_conn_manager[MAX_CONNECTION_LINK_NUM];
// local info
static connection_manager_env_t g_cm_env;

rt_timer_t g_time_handle;

static void cm_timeout_handler(void *parameter);

static connection_manager_env_t *cm_get_env()
{
    return &g_cm_env;
}

int connection_manager_event_process(uint8_t command, uint16_t len, void *data);
static uint8_t get_manager_index_by_connection_index(uint8_t conn_idx);


int cm_env_init(void)
{
    connection_manager_env_t *env = cm_get_env();
    env->update_after_read = true;
    env->bond_role_master = false;
    env->bond_ack = BOND_ACCEPT;
    env->set_app_bd_addr = 0;
    env->is_rslv_update_succ = 0;
#ifdef BLE_CTKD_ENABLE
    env->key_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY | GAP_KDIST_LINKKEY;
#else
    env->key_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY;
#endif
    return 0;
}

INIT_ENV_EXPORT(cm_env_init);

#ifndef BLE_CM_BOND_DISABLE
uint8_t read_bond_infor_from_flash_start_up()
{
    uint8_t ret;
    uint8_t *data;
    connection_manager_env_t *env = cm_get_env();
    env->update_after_read = false;
    data = bt_mem_alloc(sizeof(conn_manager_pair_dev_t));
    if (data)
    {
        rt_memset(data, 0, sizeof(conn_manager_pair_dev_t));

        uint16_t len = SIFLI_NVDS_KEY_LEN_CM;
        ret = sifli_nvds_read(SIFLI_NVDS_TYPE_CM, &len, data);

        if (ret == NVDS_OK)
        {
            rt_memcpy(&g_bond_info, data, sizeof(conn_manager_pair_dev_t));
        }
        bt_mem_free(data);
        LOG_I("read_bond_infor_from_flash: %d", ret);
    }
    else
    {

        LOG_E("read bond infor from flash start out of memory");
        ret = NVDS_NO_SPACE_AVAILABLE;
    }


    return ret;
}


static uint8_t read_bond_infor_from_flash()
{
    uint8_t ret;
    uint8_t *data;
    connection_manager_env_t *env = cm_get_env();
    env->update_after_read = true;

    data = bt_mem_alloc(sizeof(conn_manager_pair_dev_t));
    if (data)
    {
        rt_memset(data, 0, sizeof(conn_manager_pair_dev_t));


        uint16_t len = SIFLI_NVDS_KEY_LEN_CM;
        ret = sifli_nvds_read(SIFLI_NVDS_TYPE_CM, &len, data);


        if (ret == NVDS_OK)
        {
            rt_memcpy(&g_bond_info, data, sizeof(conn_manager_pair_dev_t));
        }
        bt_mem_free(data);
        LOG_I("read_bond_infor_from_flash: %d", ret);
    }
    else
    {

        LOG_E("read bond infor from flash out of memory");
        ret = NVDS_NO_SPACE_AVAILABLE;
    }


    return ret;
}

uint8_t set_bond_infor_to_flash()
{
    uint8_t ret;
    uint8_t *data;

    data = bt_mem_alloc(sizeof(conn_manager_pair_dev_t));
    if (data)
    {
        rt_memcpy(data, &g_bond_info, sizeof(conn_manager_pair_dev_t));
        ret = sifli_nvds_write(SIFLI_NVDS_TYPE_CM, sizeof(conn_manager_pair_dev_t), data);
        bt_mem_free(data);
        LOG_I("set_bond_infor_to_flash: %d", ret);
    }
    else
    {
        LOG_E("set_bond_infor no memory");
        ret = NVDS_NO_SPACE_AVAILABLE;
    }
    return ret;
}
#endif // BLE_CM_BOND_DISABLE

uint8_t read_local_bd_addr()
{
    uint8_t ret;
    uint8_t *data;
    connection_manager_env_t *env = cm_get_env();
    sifli_nvds_read_tag_t read_tag;
    read_tag.tag = NVDS_STACK_TAG_BD_ADDRESS;
    read_tag.length = NVDS_STACK_LEN_BD_ADDRESS;


    data = bt_mem_alloc(BD_ADDR_LEN);
    if (data)
    {

        ret = sifli_nvds_read_tag_value(&read_tag, data);
        if (ret == NVDS_OK)
        {
            rt_memcpy(&env->local_addr, data, BD_ADDR_LEN);
        }
        bt_mem_free(data);
    }

    else
    {
        LOG_E("read local bd addr no memory");
        ret = NVDS_NO_SPACE_AVAILABLE;
    }
    return ret;
}

static void set_nvds_app_addr(bd_addr_t addr)
{

    sifli_nvds_write_tag_t *update_tag = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_APP_LEN_BD_ADDRESS);
    BT_OOM_ASSERT(update_tag);
    if (update_tag)
    {
        update_tag->is_flush = 1;
        update_tag->type = BLE_UPDATE_ALWAYS;
        update_tag->value.tag = NVDS_APP_TAG_BD_ADDRESS;
        update_tag->value.len = NVDS_APP_LEN_BD_ADDRESS;
        memcpy((void *)update_tag->value.value, (void *)addr.addr, NVDS_APP_LEN_BD_ADDRESS);


        sifli_nvds_write_tag_value(update_tag);
        bt_mem_free(update_tag);
    }

}

void update_resolving_list()
{
    ble_gap_resolving_list_t *rl_list;
    int list_size = 0;
    uint8_t found_fix = 1;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        if (g_bond_info.priority[i] != 0)
        {
            list_size++;
        }

        if (g_bond_info.fixed[i] == 1)
        {
            found_fix = 1;
        }
    }
    // LOG_I("update_resolving_list, size: %d", list_size);

    if (found_fix == 0)
    {
        uint8_t expect_priority;
        if (list_size == MAX_PAIR_DEV)
        {
            expect_priority = MAX_PAIR_DEV;
        }
        else
        {
            expect_priority = 0;
            list_size++;
        }
        LOG_I("insert fix device %d", expect_priority);
        for (int i = 0; i < MAX_PAIR_DEV; i++)
        {
            if (g_bond_info.priority[i] == expect_priority)
            {
                g_bond_info.fixed[i] = 1;
                g_bond_info.priority[i] = list_size;
                ble_get_local_irk(&g_bond_info.local_irk[i]);
                g_bond_info.peer_irk[i].key[0] = 1;
                g_bond_info.peer_irk[i].key[1] = 2;
                g_bond_info.peer_addr[i].addr.addr[0] = 0x03;
                g_bond_info.peer_addr[i].addr.addr[1] = 0x00;
                g_bond_info.peer_addr[i].addr.addr[2] = 0x00;
                g_bond_info.peer_addr[i].addr.addr[3] = 0x00;
                g_bond_info.peer_addr[i].addr.addr[4] = 0x00;
                g_bond_info.peer_addr[i].addr.addr[5] = 0x00;
                g_bond_info.peer_addr[i].addr_type = 0;
                break;
            }
        }
    }



    rl_list = bt_mem_alloc(sizeof(ble_gap_ral_dev_info_t) * list_size + sizeof(uint8_t));
    BT_OOM_ASSERT(rl_list);
    if (rl_list)
    {
        rl_list->size = list_size;


        int rl_list_index = 0;
        for (int i = 0; i < MAX_PAIR_DEV; i++)
        {

            // LOG_I("update_resolving_list, priority: %d, addr:%x", g_bond_info.priority[i], g_bond_info.peer_addr[i].addr.addr[0]);
            if (g_bond_info.priority[i] != 0 && rl_list_index < list_size)
            {
                rl_list->ral[rl_list_index].addr = g_bond_info.peer_addr[i];
                // network privacy mode for 0
                // device privacy mode for 1
                rl_list->ral[rl_list_index].priv_mode = 1;
                rt_memcpy(rl_list->ral[rl_list_index].local_irk, g_bond_info.local_irk[i].key, GAP_KEY_LEN);
                rt_memcpy(rl_list->ral[rl_list_index].peer_irk, g_bond_info.peer_irk[i].key, GAP_KEY_LEN);
                rl_list_index++;
            }
        }
        ble_gap_set_resolving_list(rl_list);
        bt_mem_free(rl_list);
    }

}

void clear_resolving_list()
{
    ble_gap_resolving_list_t *rl_list;

    rl_list = bt_mem_alloc(sizeof(ble_gap_ral_dev_info_t) + sizeof(uint8_t));
    BT_OOM_ASSERT(rl_list);
    if (rl_list)
    {
        rl_list->size = 1;
        rt_memset(rl_list->ral, 0, sizeof(ble_gap_ral_dev_info_t));


        ble_gap_set_resolving_list(rl_list);
        bt_mem_free(rl_list);
    }
}

void add_white_list()
{
    int list_size = 0;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        if (g_bond_info.priority[i] != 0)
        {
            list_size++;
        }
    }

    ble_gap_white_list_t *wl;

    wl = bt_mem_alloc(sizeof(ble_gap_addr_t) * list_size + sizeof(uint8_t));
    BT_OOM_ASSERT(wl);
    if (wl)
    {
        wl->size = list_size;


        int wl_index = 0;
        for (int i = 0; i < MAX_PAIR_DEV; i++)
        {
            if (g_bond_info.priority[i] != 0 && wl_index < list_size)
                // TODO:
                wl->addr[wl_index] = g_bond_info.peer_addr[i];
            wl_index++;
        }

        ble_gap_set_white_list(wl);
        bt_mem_free(wl);
    }

}

#ifndef BLE_CM_BOND_DISABLE
void flash_init()
{
    uint8_t ret;
    LOG_E("CLEAR BOND INFOR !!!");
    rt_memset(&g_bond_info, 0, sizeof(conn_manager_pair_dev_t));
    ret = set_bond_infor_to_flash();
}

void check_bond_infor()
{
    int cmp;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        if (g_bond_info.priority[i] > MAX_PAIR_DEV)
        {
            LOG_E("Unexpected priority: %d, clear bond information", g_bond_info.priority[i]);
            flash_init();
            return;
        }

        for (int j = i; j < MAX_PAIR_DEV; j++)
        {
            if (i == j)
            {
                continue;
            }

            cmp = rt_memcmp(&g_bond_info.peer_addr[i], &g_bond_info.peer_addr[j], sizeof(ble_gap_addr_t));
            if (cmp == 0)
            {
                if (g_bond_info.priority[i] > g_bond_info.priority[j])
                {
                    LOG_E("Found repeat device %d, %d, del older", g_bond_info.priority[i], g_bond_info.priority[j]);
                    g_bond_info.priority[i] = 0;
                    rt_memset(&g_bond_info.peer_addr[i], 0, sizeof(ble_gap_addr_t));
                    set_bond_infor_to_flash();
                }

                if (g_bond_info.priority[i] < g_bond_info.priority[j])
                {
                    LOG_E("Found repeat device %d, %d, del oloder", g_bond_info.priority[i], g_bond_info.priority[j]);
                    g_bond_info.priority[j] = 0;
                    rt_memset(&g_bond_info.peer_addr[j], 0, sizeof(ble_gap_addr_t));
                    set_bond_infor_to_flash();
                }

                if (g_bond_info.priority[i] == 0)
                {
                    continue;
                }


                if (g_bond_info.priority[i] == g_bond_info.priority[j])
                {
                    LOG_E("Found repeat device %d, %d, both del", g_bond_info.priority[i], g_bond_info.priority[j]);
                    g_bond_info.priority[i] = 0;
                    rt_memset(&g_bond_info.peer_addr[i], 0, sizeof(ble_gap_addr_t));
                    g_bond_info.priority[j] = 0;
                    rt_memset(&g_bond_info.peer_addr[j], 0, sizeof(ble_gap_addr_t));
                    set_bond_infor_to_flash();
                }
            }
        }
    }
}

static void init_bond_cnf_information()
{
    connection_manager_env_t *env = cm_get_env();
    env->bond_cnf_info.accept = true;
    env->bond_cnf_info.iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    env->bond_cnf_info.oob = GAP_OOB_AUTH_DATA_NOT_PRESENT;
    env->bond_cnf_info.sec_req = GAP_NO_SEC;
    env->bond_cnf_info.auth = GAP_AUTH_REQ_SEC_CON_BOND;
}

uint8_t connection_manager_set_security_level(int level)
{
    LOG_I("set sec level %d", level);
    connection_manager_env_t *env = cm_get_env();
    switch (level)
    {
    case 1:
        env->bond_cnf_info.iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
        env->bond_cnf_info.sec_req = GAP_NO_SEC;
        env->bond_cnf_info.auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        env->security_level = 1;
        break;
    case 2:
        env->bond_cnf_info.iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
        env->bond_cnf_info.auth = GAP_AUTH_REQ_SEC_CON_BOND;
        env->bond_cnf_info.sec_req = GAP_SEC1_NOAUTH_PAIR_ENC;
        env->security_level = 2;
        break;
    case 3:
        env->bond_cnf_info.iocap = GAP_IO_CAP_DISPLAY_ONLY;
        env->bond_cnf_info.auth = GAP_AUTH_MITM | GAP_AUTH_BOND | GAP_AUTH_SEC_CON;
        env->bond_cnf_info.sec_req = GAP_SEC1_AUTH_PAIR_ENC;
        env->security_level = 3;
        break;
    case 4:
        env->bond_cnf_info.iocap = GAP_IO_CAP_DISPLAY_YES_NO;
        env->bond_cnf_info.auth = GAP_AUTH_MITM | GAP_AUTH_BOND | GAP_AUTH_SEC_CON;
        env->bond_cnf_info.sec_req = GAP_SEC1_AUTH_PAIR_ENC;
        env->security_level = 4;
        break;
    }
    return 0;
}

static void init_connected_auth()
{
    connection_manager_env_t *env = cm_get_env();
    env->connected_auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
}
#endif //BLE_CM_BOND_DISABLE

static void connection_manager_init()
{
    uint8_t ret;
    connection_manager_env_t *env = cm_get_env();

    env->is_init = 1;

    for (uint8_t i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        g_conn_manager[i].conn_idx = CM_CONN_INDEX_EMPTY;
        g_conn_manager[i].bond_state = BOND_STATE_NONE;
        g_conn_manager[i].connection_state = CONNECTION_STATE_DISCONNECTED;
        g_conn_manager[i].auth_state = GAP_AUTH_REQ_NO_MITM_NO_BOND;
    }

#ifndef BLE_CM_BOND_DISABLE
    // on dual core, read flash will be called in ble_stack.c, which is earlier than power on.
    // single core
    if (env->update_after_read)
    {
        ret = read_bond_infor_from_flash();
        if (ret == NVDS_OK)
        {
            check_bond_infor();
            update_resolving_list();
        }
    }
    else
    {
        // dual core
        check_bond_infor();
        update_resolving_list();
    }
#endif // BLE_CM_BOND_DISABLE

    read_local_bd_addr();
#ifndef BLE_CM_BOND_DISABLE
    init_connected_auth();
    init_bond_cnf_information();
#endif //BLE_CM_BOND_DISABLE
    env->update_timer = NULL;
}

#ifdef BLE_CM_RESOLVE_ADDRESS
static uint8_t connection_manager_get_bonded_device_num(void)
{
    uint8_t device_count = 0;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        if (g_bond_info.priority[i] != 0)
        {
            device_count++;
        }
    }
    return device_count;
}
#endif

#ifdef BLE_CM_RESOLVE_ADDRESS
static void connection_manager_update_unresolving_conn(void)
{
    uint32_t i;
    connection_manager_env_t *env = cm_get_env();
    // Find first resolve addres
    for (i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].connection_state == CONNECTION_STATE_RESOLVING)
        {
            ble_gap_connection_response_t rsp = {0};
            rsp.auth = env->connected_auth;
            rsp.conn_idx =  g_conn_manager[i].conn_idx;
            ble_gap_connect_response(&rsp);

            ble_gap_connect_ind_t data;
            data.conn_idx = g_conn_manager[i].conn_idx;
            data.con_interval = g_conn_manager[i].connection_interval;
            data.con_latency = g_conn_manager[i].connection_latency;
            data.role = g_conn_manager[i].role;
            data.sup_to = g_conn_manager[i].supervision_timeout;
            data.peer_addr = g_conn_manager[i].peer_addr;
            data.peer_addr_type = g_conn_manager[i].peer_addr_type;

            connection_manager_event_process(CM_CONNECTED_IND, sizeof(connection_manager_connect_ind_t), &data);
            connection_manager_connection_state_change(i, CONNECTION_STATE_CONNECTED, BLE_GAP_RESOLVE_ADDRESS_CNF);

            break;
        }
    }

}

static void connection_manager_update_conn_with_RPA(ble_gap_solved_addr_ind_t *ind)
{
    do
    {
        uint32_t i, j;
        // Find address to resolve
        for (i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
        {
            if (g_conn_manager[i].connection_state == CONNECTION_STATE_RESOLVING)
            {
                if (memcmp(&g_conn_manager[i].peer_addr, &ind->addr, sizeof(bd_addr_t)) == 0)
                {
                    break;
                }
            }
        }

        if (i >= MAX_CONNECTION_LINK_NUM)
            break;

        // Find identiy address.
        for (j = 0; j < MAX_PAIR_DEV; j++)
        {
            if (g_bond_info.priority[i] != 0 &&
                    memcmp(&g_bond_info.peer_irk[j], &ind->irk, sizeof(ble_gap_sec_key_t)) == 0)
            {

                g_conn_manager[i].peer_addr = g_bond_info.peer_addr[j].addr;
                g_conn_manager[i].peer_addr_type = g_bond_info.peer_addr[j].addr_type;
                g_conn_manager[i].bond_index = j;
                g_conn_manager[i].bond_state = BOND_STATE_BONDED;

                ble_gap_connection_response_t rsp = {0};
                rsp.conn_idx =  g_conn_manager[i].conn_idx;
                rsp.auth = g_bond_info.auth[j];
                rsp.ltk_present = g_bond_info.ltk_present[j];
                ble_gap_connect_response(&rsp);

                ble_gap_connect_ind_t data;
                data.conn_idx = g_conn_manager[i].conn_idx;
                data.con_interval = g_conn_manager[i].connection_interval;
                data.con_latency = g_conn_manager[i].connection_latency;
                data.role = g_conn_manager[i].role;
                data.sup_to = g_conn_manager[i].supervision_timeout;
                data.peer_addr = g_conn_manager[i].peer_addr;
                data.peer_addr_type = g_conn_manager[i].peer_addr_type;

                connection_manager_event_process(CM_CONNECTED_IND, sizeof(connection_manager_connect_ind_t), &data);
                connection_manager_connection_state_change(i, CONNECTION_STATE_CONNECTED, BLE_GAP_SOLVED_ADDRESS_IND);
                break;
            }
        }

        if (j >= MAX_PAIR_DEV)
        {
            connection_manager_env_t *env = cm_get_env();
            LOG_E("Unexpected resolving!!!");
            ble_gap_connection_response_t rsp = {0};
            rsp.auth = env->connected_auth;
            rsp.conn_idx =  g_conn_manager[i].conn_idx;
            ble_gap_connect_response(&rsp);
            connection_manager_connection_state_change(i, CONNECTION_STATE_CONNECTED, BLE_GAP_SOLVED_ADDRESS_IND);
        }
    }
    while (0);
}
#endif // BLE_CM_RESOLVE_ADDRESS

static int8_t allocate_connection(ble_gap_connect_ind_t *ind, uint8_t new_state)
{
    uint8_t conn_idx = ind->conn_idx;

    for (uint8_t i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].connection_state == CONNECTION_STATE_DISCONNECTED)
        {
            g_conn_manager[i].conn_idx = conn_idx;
            return i;
        }
    }
    return ALLOCATE_CONNECTION_FAIL;
}

void connection_manager_connection_state_change(uint8_t manager_index, uint8_t new_states, uint16_t event)
{
    uint8_t old_state;
    old_state = g_conn_manager[manager_index].connection_state;
    g_conn_manager[manager_index].connection_state = new_states;

    LOG_I("connection_manager_connection_state_change 0x%x to 0x%x", old_state, new_states);

    if (new_states == CONNECTION_STATE_DISCONNECTED)
    {
        g_conn_manager[manager_index].conn_idx = CM_CONN_INDEX_EMPTY;
    }
}

void max_links_connection_state_change(ble_gap_connect_ind_t *ind)
{
    connection_manager_disconnected_ind_t dis_ind;
    dis_ind.conn_idx = ind->conn_idx;
    dis_ind.reason = CO_ERROR_REMOTE_USER_TERM_CON;

    connection_manager_event_process(CM_DISCONNECTED_IND, sizeof(connection_manager_disconnected_ind_t), &dis_ind);
}

#if 0
void app_sec_send_security_req(uint8_t conidx)
{
    // Send security request
    struct gapc_security_cmd *cmd = sifli_msg_alloc(GAPC_SECURITY_CMD,
                                    TASK_BUILD_ID(TASK_ID_GAPC, conidx), sifli_get_stack_id(),
                                    sizeof(struct gapc_security_cmd));

    cmd->operation = GAPC_SECURITY_REQ;
    cmd->auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;

    // Send the message
    sifli_msg_send(cmd);
}
#endif

void connection_manager_disconnect(uint8_t conn_index)
{
    ble_gap_disconnect_t conn;
    conn.conn_idx = conn_index;
    conn.reason = CO_ERROR_CON_TERM_BY_LOCAL_HOST;

    ble_gap_disconnect(&conn);
}


#ifndef BLE_CM_BOND_DISABLE
uint8_t connection_manager_get_bonded_devices(uint8_t *data)
{
    uint8_t device_count = 0;
    conn_manager_get_bonded_dev_t bonded_device;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        if (g_bond_info.priority[i] != 0)
        {
            bonded_device.priority[device_count] = g_bond_info.priority[i];
            bonded_device.peer_addr[device_count] = g_bond_info.peer_addr[i];
            bonded_device.fixed[device_count] = g_bond_info.fixed[i];
            device_count++;
        }
    }

    if (device_count == 0)
    {
        return 0;
    }

    rt_memcpy(data, &bonded_device, sizeof(conn_manager_get_bonded_dev_t));
    return device_count;
}

#if 0
static uint8_t connection_manager_delete_bond_without_addr_type(ble_gap_addr_t peer_addr)
{
    uint8_t old_priority;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        int ret = rt_memcmp(g_bond_info.peer_addr[i].addr.addr, peer_addr.addr.addr, BD_ADDR_LEN);
        if (ret == 0)
        {
            LOG_I("Del bond addr on peer");
            old_priority = g_bond_info.priority[i];
            g_bond_info.priority[i] = 0;

            rt_memset(&g_bond_info.peer_addr[i], 0, sizeof(ble_gap_addr_t));
            rt_memset(&g_bond_info.ltk[i], 0, sizeof(ble_gap_ltk_t));
            rt_memset(&g_bond_info.local_irk[i], 0, sizeof(ble_gap_sec_key_t));
            rt_memset(&g_bond_info.peer_irk[i], 0, sizeof(ble_gap_sec_key_t));

            for (int j = 0; j < MAX_PAIR_DEV; j++)
            {
                if (g_bond_info.priority[j] == 0 || g_bond_info.priority[j] < old_priority)
                {
                    continue;
                }

                if (g_bond_info.priority[j] > old_priority)
                {
                    g_bond_info.priority[j]--;
                }
            }

            update_resolving_list();
            uint8_t set_result = set_bond_infor_to_flash();
            if (set_result == NVDS_OK)
            {
                return CM_STATUS_OK;
            }
            else if (set_result == NVDS_PENDING)
            {
                return CM_FLASH_PEDING;
            }
            else
            {
                LOG_E("Del bond, set to flash fail %d", set_result);
                return CM_FLASH_FAIL;
            }
        }
    }

    return CM_ADDR_ERROR;
}
#endif

uint8_t connection_manager_delete_bond(ble_gap_addr_t peer_addr)
{
    uint8_t old_priority;
    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        int ret = rt_memcmp(g_bond_info.peer_addr[i].addr.addr, peer_addr.addr.addr, BD_ADDR_LEN);
        if (ret == 0 && g_bond_info.peer_addr[i].addr_type == peer_addr.addr_type)
        {
            LOG_I("Del bond on peer");
            old_priority = g_bond_info.priority[i];
            g_bond_info.priority[i] = 0;

            rt_memset(&g_bond_info.peer_addr[i], 0, sizeof(ble_gap_addr_t));
            rt_memset(&g_bond_info.ltk[i], 0, sizeof(ble_gap_ltk_t));
            rt_memset(&g_bond_info.local_irk[i], 0, sizeof(ble_gap_sec_key_t));
            rt_memset(&g_bond_info.peer_irk[i], 0, sizeof(ble_gap_sec_key_t));

            for (int j = 0; j < MAX_PAIR_DEV; j++)
            {
                if (g_bond_info.priority[j] == 0 || g_bond_info.priority[j] < old_priority)
                {
                    continue;
                }

                if (g_bond_info.priority[j] > old_priority)
                {
                    g_bond_info.priority[j]--;
                }
            }

            update_resolving_list();
            uint8_t set_result = set_bond_infor_to_flash();
            if (set_result == NVDS_OK)
            {
                return CM_STATUS_OK;
            }
            else if (set_result == NVDS_PENDING)
            {
                return CM_FLASH_PEDING;
            }
            else
            {
                LOG_E("Del bond, set to flash fail %d", set_result);
                return CM_FLASH_FAIL;
            }
        }
    }

    return CM_ADDR_ERROR;
}

uint8_t connection_manager_delete_all_bond()
{
    LOG_I("Del all bonded device");

    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        g_bond_info.priority[i] = 0;
        rt_memset(&g_bond_info.peer_addr[i], 0, sizeof(ble_gap_addr_t));
        rt_memset(&g_bond_info.ltk[i], 0, sizeof(ble_gap_ltk_t));
        rt_memset(&g_bond_info.local_irk[i], 0, sizeof(ble_gap_sec_key_t));
        rt_memset(&g_bond_info.peer_irk[i], 0, sizeof(ble_gap_sec_key_t));
    }

    update_resolving_list();
    uint8_t set_result = set_bond_infor_to_flash();
    if (set_result == NVDS_OK)
    {
        return CM_STATUS_OK;
    }
    else if (set_result == NVDS_PENDING)
    {
        return CM_FLASH_PEDING;
    }
    else
    {
        LOG_E("Del bond, set to flash fail %d", set_result);
        return CM_FLASH_FAIL;
    }
}
#endif //BLE_CM_BOND_DISABLE

#ifdef BLE_SVC_CHG_ENABLE
uint8_t ble_svc_change_enable(uint8_t conn_idx)
{
    LOG_I("ble_svc_change_enable");
    sibles_search_service(conn_idx, ATT_UUID_16_LEN, (uint8_t *)generic_attribute_uuid);
    // Only treat enable conn as search target
    return 0;
}
#endif //BLE_SVC_CHG_ENABLE

#ifndef BLE_CM_BOND_DISABLE
static void update_pair_infor(ble_gap_bond_ind_t *ind, uint8_t manager_index)
{
    int i;

    uint8_t update = 0;
    do
    {
        for (i = 0; i < MAX_PAIR_DEV; i++)
        {
            int ret = rt_memcmp(g_bond_info.peer_addr[i].addr.addr, g_bonding_info.peer_addr.addr.addr, BD_ADDR_LEN);
            if (ret == 0)
            {
                update = 1;
                LOG_I("update_pair_infor flash use original");
                // update priority
                int old_priority = g_bond_info.priority[i];
                for (int j = 0; j < MAX_PAIR_DEV; j++)
                {
                    if (g_bond_info.priority[j] != 0 && g_bond_info.priority[j] < old_priority)
                    {
                        g_bond_info.priority[j]++;
                    }
                }

                g_bond_info.fixed[i] = 0;
                g_bond_info.local_irk[i] = g_bonding_info.local_irk;
                g_bond_info.ltk[i] = g_bonding_info.ltk;
                g_bond_info.peer_addr[i] = g_bonding_info.peer_addr;
                g_bond_info.peer_irk[i] = g_bonding_info.peer_irk;
                g_bond_info.priority[i] = 1;
                g_bond_info.auth[i] = ind->data.auth.info;
                g_bond_info.ltk_present[i] = ind->data.auth.ltk_present;
                g_conn_manager[manager_index].bond_index = i;
                break;
            }
        }

        if (update)
        {
            break;
        }

        for (i = 0; i < MAX_PAIR_DEV; i++)
        {
            if (g_bond_info.priority[i] == 0)
            {
                update = 1;
                LOG_I("update_pair_infor flash use empty");
                for (int j = 0; j < MAX_PAIR_DEV; j++)
                {
                    if (g_bond_info.priority[j] != 0 && g_bond_info.priority[j] < MAX_PAIR_DEV)
                    {
                        g_bond_info.priority[j]++;
                    }
                }

                g_bond_info.fixed[i] = 0;
                g_bond_info.local_irk[i] = g_bonding_info.local_irk;
                g_bond_info.ltk[i] = g_bonding_info.ltk;
                g_bond_info.peer_addr[i] = g_bonding_info.peer_addr;
                g_bond_info.peer_irk[i] = g_bonding_info.peer_irk;
                g_bond_info.priority[i] = 1;
                g_bond_info.auth[i] = ind->data.auth.info;
                g_bond_info.ltk_present[i] = ind->data.auth.ltk_present;
                g_conn_manager[manager_index].bond_index = i;
                break;
            }
        }

        if (update)
        {
            break;
        }

        for (i = 0; i < MAX_PAIR_DEV; i++)
        {
            if (g_bond_info.priority[i] >= MAX_PAIR_DEV)
            {
                LOG_I("update_pair_infor flash replace");
                // update priority
                for (int j = 0; j < MAX_PAIR_DEV; j++)
                {
                    if (g_bond_info.priority[j] != 0 && g_bond_info.priority[j] < MAX_PAIR_DEV)
                    {
                        g_bond_info.priority[j]++;
                    }
                }

                // replace the oldest bonded device
                g_bond_info.fixed[i] = 0;
                g_bond_info.local_irk[i] = g_bonding_info.local_irk;
                g_bond_info.ltk[i] = g_bonding_info.ltk;
                g_bond_info.peer_addr[i] = g_bonding_info.peer_addr;
                g_bond_info.peer_irk[i] = g_bonding_info.peer_irk;
                g_bond_info.priority[i] = 1;
                g_bond_info.auth[i] = ind->data.auth.info;
                g_bond_info.ltk_present[i] = ind->data.auth.ltk_present;
                g_conn_manager[manager_index].bond_index = i;
                break;
            }
        }
    }
    while (0);

    if (i < MAX_PAIR_DEV)
    {
#ifndef SOC_SF32LB55X
#ifdef BLE_CTKD_ENABLE
        connection_manager_convert_ltk_to_ilk(i);
#endif
#endif
    }
    // sifli_nvds_write(SIFLI_NVDS_TYPE_CM, &g_bond_info, sizeof(g_bond_info));
    set_bond_infor_to_flash();
}

static void process_bond_event(ble_gap_bond_ind_t *ind, uint16_t command)
{
    uint8_t connection_index = ind->conn_idx;
    uint8_t bond_state = BOND_STATE_NONE;

    uint8_t manager_index;
    manager_index = get_manager_index_by_connection_index(connection_index);

    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return;
    }
    LOG_I("BLE_GAP_BOND_IND %d", ind->info);

    switch (ind->info)
    {
    case GAPC_PAIRING_SUCCEED:
    {
        LOG_I("GAPC_PAIRING_SUCCEED");

        update_pair_infor(ind, manager_index);
        update_resolving_list();

        bond_state = BOND_STATE_BONDED;
        g_conn_manager[manager_index].bond_state = bond_state;
        g_conn_manager[manager_index].auth_state = ind->data.auth.info;
        connection_manager_event_process(CM_PAIRING_SUCCEED, sizeof(ble_gap_bond_ind_t), ind);
        break;
    }
    case (GAPC_REPEATED_ATTEMPT):
    {
        LOG_I("GAPC_REPEATED_ATTEMPT");
        connection_manager_disconnect(connection_index);
        break;
    }
    case (GAPC_LTK_EXCH) :
    {
        LOG_I("GAPC_LTK_EXCH");
        uint8_t manager_index;
        manager_index = get_manager_index_by_connection_index(ind->conn_idx);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            break;
        }

        // when first bond, encrypt on event comes from here.
        g_conn_manager[manager_index].enc_state = ENC_STATE_ON;
        g_conn_manager[manager_index].first_bond = 1;

        if (g_conn_manager[manager_index].sec_con_enabled)
        {
            LOG_I("store LTK when sec connection enabled");
            // Store LTK in NVDS
            g_bonding_info.ltk = ind->data.ltk;
        }
#ifdef BLE_SVC_CHG_ENABLE
        ble_svc_change_enable(ind->conn_idx);
#endif //BLE_SVC_CHG_ENABLE
        break;
    }
    case GAPC_PAIRING_FAILED:
    {
        LOG_I("GAPC_PAIRING_FAILED %d", ind->data.reason);
        //app_sec_send_security_req(connection_index);

#if 0
        connection_manager_delete_bond_without_addr_type(g_bonding_info.peer_addr);
        bond_state = BOND_STATE_NONE;
        g_conn_manager[manager_index].bond_state = bond_state;
#else
        LOG_I("keep original state when pair fail");
#endif
        connection_manager_event_process(CM_PAIRING_FAILED, sizeof(ble_gap_bond_ind_t), ind);
        break;
    }
    case GAPC_IRK_EXCH:
    {
        LOG_I("GAPC_IRK_EXCH");
        uint8_t manager_index;

        manager_index = get_manager_index_by_connection_index(ind->conn_idx);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            break;
        }

        g_conn_manager[manager_index].peer_addr = ind->data.irk.addr.addr;
        g_conn_manager[manager_index].peer_addr_type = ind->data.irk.addr.addr_type;

        ble_gap_sec_key_t loc_irk;
        ble_get_local_irk(&loc_irk);

        g_bonding_info.local_irk = loc_irk;
        g_bonding_info.peer_irk = ind->data.irk.irk;
        g_bonding_info.peer_addr.addr = g_conn_manager[manager_index].peer_addr;
        g_bonding_info.peer_addr.addr_type = g_conn_manager[manager_index].peer_addr_type;
        break;
    }
    }
}
#endif //BLE_CM_BOND_DISABLE

__INLINE uint32_t cm_rand_word(void)
{
    return (uint32_t)rand();
}


#ifndef BLE_CM_BOND_DISABLE
void init_bonding_info()
{
    rt_memset(&g_bonding_info, 0, sizeof(conn_manager_bonding_dev_t));
}

uint8_t connection_manager_set_bond_cnf_accept(bool accept)
{
    connection_manager_env_t *env = cm_get_env();
    env->bond_cnf_info.accept = accept;
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_bond_cnf_iocap(uint8_t iocap)
{
    connection_manager_env_t *env = cm_get_env();
    switch (iocap)
    {
    case GAP_IO_CAP_DISPLAY_ONLY:
    case GAP_IO_CAP_DISPLAY_YES_NO:
    case GAP_IO_CAP_KB_ONLY:
    case GAP_IO_CAP_NO_INPUT_NO_OUTPUT:
    case GAP_IO_CAP_KB_DISPLAY:
    case GAP_IO_CAP_LAST:
        env->bond_cnf_info.iocap = iocap;
        break;
    default:
        return CM_PARAMETER_ERROR;
    }
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_bond_cnf_sec(uint8_t sec_req)
{
    connection_manager_env_t *env = cm_get_env();
    switch (sec_req)
    {
    case GAP_NO_SEC:
    case GAP_SEC1_NOAUTH_PAIR_ENC:
    case GAP_SEC1_AUTH_PAIR_ENC:
    case GAP_SEC2_NOAUTH_DATA_SGN:
    case GAP_SEC2_AUTH_DATA_SGN:
    case GAP_SEC1_SEC_CON_PAIR_ENC:
        env->bond_cnf_info.sec_req = sec_req;
        break;
    default:
        return CM_PARAMETER_ERROR;
    }
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_bond_cnf_oob(uint8_t oob)
{
    connection_manager_env_t *env = cm_get_env();
    switch (oob)
    {
    case GAP_OOB_AUTH_DATA_NOT_PRESENT:
    case GAP_OOB_AUTH_DATA_PRESENT:
    case GAP_OOB_AUTH_DATA_LAST:
        env->bond_cnf_info.oob = oob;
        break;
    default:
        return CM_PARAMETER_ERROR;
    }
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_bond_cnf_auth(uint8_t auth)
{
    connection_manager_env_t *env = cm_get_env();
    switch (auth)
    {
    case GAP_AUTH_REQ_NO_MITM_NO_BOND:
    case GAP_AUTH_REQ_NO_MITM_BOND:
    case GAP_AUTH_REQ_MITM_NO_BOND:
    case GAP_AUTH_REQ_MITM_BOND:
    case GAP_AUTH_REQ_SEC_CON_NO_BOND:
    case GAP_AUTH_REQ_SEC_CON_BOND:
    case GAP_AUTH_REQ_SEC_CON_MITM_BOND:
    case GAP_AUTH_REQ_LAST:
        env->bond_cnf_info.auth = auth;
        break;
    default:
        return CM_PARAMETER_ERROR;
    }
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_connected_auth(uint8_t auth)
{
    connection_manager_env_t *env = cm_get_env();
    switch (auth)
    {
    case GAP_AUTH_REQ_NO_MITM_NO_BOND:
    case GAP_AUTH_REQ_NO_MITM_BOND:
    case GAP_AUTH_REQ_MITM_NO_BOND:
    case GAP_AUTH_REQ_MITM_BOND:
    case GAP_AUTH_REQ_SEC_CON_NO_BOND:
    case GAP_AUTH_REQ_SEC_CON_BOND:
    case GAP_AUTH_REQ_SEC_CON_MITM_BOND:
    case GAP_AUTH_REQ_LAST:
        env->connected_auth = auth;
        break;
    default:
        return CM_PARAMETER_ERROR;
    }
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_oob_data(uint8_t *oob_data, uint8_t length)
{
    connection_manager_env_t *env = cm_get_env();
    if (!oob_data || length > GAP_KEY_LEN)
        return CM_PARAMETER_ERROR;

    memset(env->oob_data, 0, GAP_KEY_LEN);
    memcpy(env->oob_data, oob_data, length);
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_sec_oob_data(ble_gap_oob_t *oob_data)
{
    connection_manager_env_t *env = cm_get_env();
    if (!oob_data)
        return CM_PARAMETER_ERROR;

    memcpy((void *)&env->sec_oob_data, (void *)oob_data, sizeof(ble_gap_oob_t));
    return CM_STATUS_OK;
}

uint8_t connection_manager_set_bond_ack(uint8_t state)
{
    connection_manager_env_t *env = cm_get_env();
    switch (state)
    {
    case BOND_ACCEPT:
    case BOND_PENDING:
    case BOND_REJECT:
        env->bond_ack = state;
        break;
    default:
        LOG_E("error bond ack %d", state);
        return CM_BOND_ACK_ERROR;
    }
    return CM_STATUS_OK;
}

uint8_t connection_manager_bond_ack_reply(uint8_t conn_idx, uint8_t command, bool accept)
{

    ble_gap_bond_confirm_t *cfm = bt_mem_alloc(sizeof(ble_gap_bond_confirm_t));
    connection_manager_env_t *env = cm_get_env();
    uint8_t manager_index = get_manager_index_by_connection_index(conn_idx);
    if (cfm == NULL)
        return CM_CONN_OUT_OF_MEMORY;

    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return CM_CONN_INDEX_ERROR;
    }
    switch (command)
    {
    case GAPC_PAIRING_REQ:
        cfm->accept = accept;
        cfm->conn_idx = conn_idx;
        cfm->request = GAPC_PAIRING_RSP;

        cfm->cfm_data.pairing_feat.auth = env->bond_cnf_info.auth;
        cfm->cfm_data.pairing_feat.iocap = env->bond_cnf_info.iocap;
        cfm->cfm_data.pairing_feat.key_size  = 16;
        cfm->cfm_data.pairing_feat.oob = env->bond_cnf_info.oob;
        cfm->cfm_data.pairing_feat.sec_req = env->bond_cnf_info.sec_req;
        cfm->cfm_data.pairing_feat.ikey_dist = env->key_dist;
        cfm->cfm_data.pairing_feat.rkey_dist = env->key_dist;
        if ((cfm->cfm_data.pairing_feat.auth & GAP_AUTH_SEC_CON) && (env->remote_auth & GAP_AUTH_SEC_CON))
        {
            LOG_I("sec on");
            g_conn_manager[manager_index].sec_con_enabled = true;
        }
        else
        {
            g_conn_manager[manager_index].sec_con_enabled = false;
        }

        break;
    case GAPC_NC_EXCH:
        cfm->accept = accept;
        cfm->conn_idx = conn_idx;
        cfm->request = GAPC_NC_EXCH;
        break;
    case GAPC_TK_EXCH:
        cfm->accept = accept;
        cfm->conn_idx = conn_idx;
        cfm->request = GAPC_TK_EXCH;

        uint32_t pin_code = env->pin_code;
        cfm->cfm_data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
        cfm->cfm_data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
        cfm->cfm_data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
        cfm->cfm_data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
        break;
    default:
        LOG_E("command error: %d", command);
        bt_mem_free(cfm);
        return CM_BOND_CFM_COMMAND_ERROR;
    }

    ble_gap_bond_confirm(cfm);
    bt_mem_free(cfm);
    return CM_STATUS_OK;
}
#endif //BLE_CM_BOND_DISABLE

uint8_t connection_manager_get_remote_feature(uint8_t conn_idx)
{
    uint8_t manager_index;
    manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return CM_CONN_INDEX_ERROR;
    }

    bt_gap_get_remote_feature_t feature;
    feature.conn_idx = conn_idx;
    ble_gap_get_remote_feature(&feature);
    return CM_STATUS_OK;
}

#ifndef BLE_CM_BOND_DISABLE
static void process_bond_req_ind(ble_gap_bond_req_ind_t *ind)
{
    connection_manager_env_t *env = cm_get_env();
    uint8_t connection_index = ind->conn_idx;
    uint8_t manager_index = get_manager_index_by_connection_index(connection_index);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        LOG_W("process_bond_req_ind %d index error", ind->request);
        return;
    }

    ble_gap_bond_confirm_t *cfm = bt_mem_alloc(sizeof(ble_gap_bond_confirm_t));
    BT_OOM_ASSERT(cfm);
    LOG_I("BLE_GAP_BOND_REQ_IND %d", ind->request);
    switch (ind->request)
    {
    case (GAPC_PAIRING_REQ):
    {
        LOG_I("GAPC_PAIRING_REQ");
        read_local_bd_addr();
        init_bonding_info();
        g_bonding_info.peer_addr.addr = g_conn_manager[manager_index].peer_addr;
        g_bonding_info.peer_addr.addr_type = g_conn_manager[manager_index].peer_addr_type;

        g_conn_manager[manager_index].bond_state = BOND_STATE_BONDING;

        //cfm->accept = env->bond_cnf_info.accept;
        LOG_I("BOND_PENDING %d", env->bond_ack);

        if (env->bond_ack == BOND_PENDING)
        {
            connection_manager_bond_ack_infor_t pair_req_infor;
            pair_req_infor.request = GAPC_PAIRING_REQ;
            pair_req_infor.confirm_data = 0;
            pair_req_infor.conn_idx = connection_index;
            env->remote_auth = ind->data.auth_req;
            connection_manager_event_process(CM_BOND_AUTH_INFOR_CONFIRM,
                                             sizeof(connection_manager_bond_ack_infor_t),
                                             &pair_req_infor);
            bt_mem_free(cfm);
            return;
        }
        else if (env->bond_ack == BOND_ACCEPT)
        {
            cfm->accept = true;
        }
        else
        {
            cfm->accept = false;
        }

        cfm->request = GAPC_PAIRING_RSP;
        cfm->conn_idx = connection_index;

        cfm->cfm_data.pairing_feat.auth      = env->bond_cnf_info.auth;
        cfm->cfm_data.pairing_feat.iocap     = env->bond_cnf_info.iocap;
        LOG_I("local auth: %d", cfm->cfm_data.pairing_feat.auth);

        cfm->cfm_data.pairing_feat.key_size  = 16;
        cfm->cfm_data.pairing_feat.oob       = env->bond_cnf_info.oob;
        cfm->cfm_data.pairing_feat.sec_req   = env->bond_cnf_info.sec_req;

        cfm->cfm_data.pairing_feat.ikey_dist = env->key_dist;
        cfm->cfm_data.pairing_feat.rkey_dist = env->key_dist;

        if ((cfm->cfm_data.pairing_feat.auth & GAP_AUTH_SEC_CON) && (ind->data.auth_req & GAP_AUTH_SEC_CON))
        {
            LOG_I("sec on");
            g_conn_manager[manager_index].sec_con_enabled = true;
        }
        else
        {
            g_conn_manager[manager_index].sec_con_enabled = false;
        }

    }
    break;

    case (GAPC_LTK_EXCH):
    {
        LOG_I("GAPC_LTK_EXCH");
        // Counter
        uint8_t counter;

        cfm->conn_idx = connection_index;
        cfm->accept  = true;
        cfm->request = GAPC_LTK_EXCH;

        // Generate all the values
        cfm->cfm_data.ltk.ediv = (uint16_t)cm_rand_word();

        for (counter = 0; counter < GAP_RAND_NB_LEN; counter++)
        {
            cfm->cfm_data.ltk.ltk.key[counter]    = (uint8_t)cm_rand_word();
            cfm->cfm_data.ltk.randnb.nb[counter] = (uint8_t)cm_rand_word();
        }

        for (counter = GAP_RAND_NB_LEN; counter < GAP_KEY_LEN; counter++)
        {
            cfm->cfm_data.ltk.ltk.key[counter]    = (uint8_t)cm_rand_word();
        }

        // store ltk
        if (!env->bond_role_master)
        {
            g_bond_info.ltk[g_conn_manager[manager_index].bond_index] = cfm->cfm_data.ltk;
            g_bonding_info.ltk = cfm->cfm_data.ltk;
        }
        else
        {
            env->bond_role_master = false;
        }
    }
    break;

    case (GAPC_NC_EXCH):
    {
        LOG_I("GAPC_NC_EXCH");
        LOG_I("GAPC_NC_EXCH %d %d %d %d", ind->data.nc_data.value[0], ind->data.nc_data.value[1],
              ind->data.nc_data.value[2], ind->data.nc_data.value[3]);

        LOG_I("GAPC_NC_EXCH number: %d", *(uint32_t *)&ind->data.nc_data.value);

        if (env->bond_ack == BOND_PENDING)
        {
            connection_manager_bond_ack_infor_t nc_exchange;
            nc_exchange.request = GAPC_NC_EXCH;
            nc_exchange.conn_idx = connection_index;
            nc_exchange.confirm_data = *(uint32_t *)&ind->data.nc_data.value;
            connection_manager_event_process(CM_BOND_AUTH_INFOR_CONFIRM,
                                             sizeof(connection_manager_bond_ack_infor_t),
                                             &nc_exchange);
            bt_mem_free(cfm);
            return;
        }
        else if (env->bond_ack == BOND_ACCEPT)
        {
            cfm->accept = true;
        }
        else
        {
            cfm->accept = false;
        }
        cfm->conn_idx = connection_index;
        cfm->request = GAPC_NC_EXCH;
        break;
    }


    case (GAPC_IRK_EXCH):
    {
        LOG_I("GAPC_IRK_EXCH");

        cfm->conn_idx = connection_index;
        cfm->accept  = true;
        cfm->request = GAPC_IRK_EXCH;

        // Load IRK
        ble_gap_sec_key_t loc_irk;
        ble_get_local_irk(&loc_irk);
        rt_memcpy(cfm->cfm_data.irk.irk.key, loc_irk.key, GAP_KEY_LEN);

        // load device address
        rt_memcpy(cfm->cfm_data.irk.addr.addr.addr, env->local_addr.addr, BD_ADDR_LEN);
        cfm->cfm_data.irk.addr.addr_type = (cfm->cfm_data.irk.addr.addr.addr[5] & 0xC0) ? CM_ADDR_RAND : CM_ADDR_PUBLIC;
    }
    break;


    case (GAPC_TK_EXCH):
    {
        LOG_I("GAPC_TK_EXCH %d", ind->data.tk_type);

        cfm->conn_idx = connection_index;
        cfm->accept  = true;
        cfm->request = GAPC_TK_EXCH;

        // Set the TK value
        rt_memset(cfm->cfm_data.tk.key, 0, 0x10);

        switch (ind->data.tk_type)
        {
        case GAP_TK_DISPLAY:
        {
            // Display the PIN Code

            uint32_t pin_code = (100000 + (cm_rand_word() % 900000));
            LOG_I("GAPC_TK_EXCH, PIN: %d", pin_code);

            cfm->cfm_data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
            cfm->cfm_data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
            cfm->cfm_data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
            cfm->cfm_data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);

            if (env->bond_ack == BOND_PENDING)
            {
                connection_manager_bond_ack_infor_t tk_exchange;
                tk_exchange.request = GAPC_TK_EXCH;
                tk_exchange.confirm_data = pin_code;
                tk_exchange.conn_idx = connection_index;
                env->pin_code = pin_code;
                connection_manager_event_process(CM_BOND_AUTH_INFOR_CONFIRM,
                                                 sizeof(connection_manager_bond_ack_infor_t),
                                                 &tk_exchange);
                bt_mem_free(cfm);
                return;
            }
            else if (env->bond_ack == BOND_ACCEPT)
            {
                cfm->accept = true;
            }
            else
            {
                cfm->accept = false;
            }

        }
        break;
        case GAP_TK_KEY_ENTRY:
        {
            // TODO: Get key entry from upper layer
        }
        break;
        case GAP_TK_OOB:
        {
            memcpy(cfm->cfm_data.tk.key, env->oob_data, GAP_KEY_LEN);
        }
        break;
        }

    }
    break;
    case GAPC_OOB_EXCH:
    {
        cfm->conn_idx = connection_index;
        cfm->accept  = true;
        cfm->request = GAPC_OOB_EXCH;
        memcpy(cfm->cfm_data.oob.rand, env->sec_oob_data.rand, GAP_KEY_LEN);
        memcpy(cfm->cfm_data.oob.conf, env->sec_oob_data.conf, GAP_KEY_LEN);
    }
    break;
    case GAPC_CSRK_EXCH:
    {
        cfm->conn_idx = connection_index;
        cfm->accept = true;
        cfm->request = GAPC_CSRK_EXCH;
        memset(cfm->cfm_data.csrk.key, 0, GAP_KEY_LEN);
        break;
    }
    default:
    {
        LOG_I("ASSERT_ERR");
    }
    break;
    }

    ble_gap_bond_confirm(cfm);
    bt_mem_free(cfm);

}

void start_enc(uint8_t conn_idx)
{
    struct gapc_encrypt_cmd *cmd =
        sifli_msg_alloc(GAPC_ENCRYPT_CMD,
                        TASK_BUILD_ID(TASK_ID_GAPC, conn_idx),
                        sifli_get_stack_id(),
                        sizeof
                        (struct gapc_encrypt_cmd));

    uint8_t manager_index;
    manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return;
    }
    int ret = 0;

    for (int i = 0; i < MAX_PAIR_DEV; i++)
    {
        ret = rt_memcmp(g_conn_manager[manager_index].peer_addr.addr, g_bond_info.peer_addr[i].addr.addr, BD_ADDR_LEN);
        if (ret == 0)
        {
            cmd->ltk = g_bond_info.ltk[i];
            LOG_I("start_enc, ediv: %d, ltk: %x %x %x %x", g_bond_info.ltk[i].ediv, g_bond_info.ltk[i].ltk.key[0], g_bond_info.ltk[i].ltk.key[1],
                  g_bond_info.ltk[i].ltk.key[2], g_bond_info.ltk[i].ltk.key[3]);
            LOG_I("start_enc, ediv: %d, ltk: %x %x %x %x", cmd->ltk.ediv, cmd->ltk.ltk.key[0], cmd->ltk.ltk.key[1],
                  cmd->ltk.ltk.key[2], cmd->ltk.ltk.key[3]);
            break;
        }
    }

    cmd->operation = GAPC_ENCRYPT;

    sifli_msg_send((void const *)cmd);
}
#endif //BLE_CM_BOND_DISABLE

void print_addr(bd_addr_t addr)
{
    LOG_I("addr %x:%x:%x:%x:%x:%x", addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);
}

#ifndef BLE_CM_BOND_DISABLE
void connection_manager_create_bond(uint8_t conn_idx)
{
    LOG_I("create bond");
    connection_manager_env_t *env = cm_get_env();
    uint8_t manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return;
    }
    g_bonding_info.peer_addr.addr = g_conn_manager[manager_index].peer_addr;
    g_bonding_info.peer_addr.addr_type = g_conn_manager[manager_index].peer_addr_type;
    LOG_I("addr type %d", g_bonding_info.peer_addr.addr_type);
    print_addr(g_bonding_info.peer_addr.addr);

    ble_gap_bond_t bndt;

    bndt.conn_idx = conn_idx;
    bndt.pair_info.iocap = env->bond_cnf_info.iocap;
    bndt.pair_info.auth = env->bond_cnf_info.auth;
    bndt.pair_info.oob = env->bond_cnf_info.oob;
    bndt.pair_info.key_size = 16;
    bndt.pair_info.ikey_dist = env->key_dist;
    bndt.pair_info.rkey_dist = env->key_dist;
    bndt.pair_info.sec_req = env->bond_cnf_info.sec_req;

    g_conn_manager[manager_index].sec_con_enabled = true;
    env->bond_role_master = true;

    ble_gap_bond(&bndt);
}

void connection_manager_create_bond_bqb(uint8_t conn_idx, uint8_t command)
{
    LOG_I("create bond");
    connection_manager_env_t *env = cm_get_env();
    uint8_t manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return;
    }
    g_bonding_info.peer_addr.addr = g_conn_manager[manager_index].peer_addr;
    g_bonding_info.peer_addr.addr_type = g_conn_manager[manager_index].peer_addr_type;
    LOG_I("addr type %d", g_bonding_info.peer_addr.addr_type);
    print_addr(g_bonding_info.peer_addr.addr);

    ble_gap_bond_t bndt;

    bndt.conn_idx = conn_idx;
    bndt.pair_info.iocap = env->bond_cnf_info.iocap;
    bndt.pair_info.auth = env->bond_cnf_info.auth;
    bndt.pair_info.oob = env->bond_cnf_info.oob;
    if (command == 1)
    {
        bndt.pair_info.key_size = 7;
    }
    else
    {
        bndt.pair_info.key_size = 16;
    }

    if (command == 2)
    {
        bndt.pair_info.ikey_dist = GAP_KDIST_IDKEY | GAP_KDIST_ENCKEY | GAP_KDIST_SIGNKEY;
        bndt.pair_info.rkey_dist = GAP_KDIST_IDKEY | GAP_KDIST_ENCKEY | GAP_KDIST_SIGNKEY;
    }
    else if (command == 3)
    {
        bndt.pair_info.ikey_dist = GAP_KDIST_NONE;
        bndt.pair_info.rkey_dist = GAP_KDIST_NONE;
    }
    else
    {
        bndt.pair_info.ikey_dist = GAP_KDIST_IDKEY | GAP_KDIST_ENCKEY;
        bndt.pair_info.rkey_dist = GAP_KDIST_IDKEY | GAP_KDIST_ENCKEY;
    }

    bndt.pair_info.sec_req = env->bond_cnf_info.sec_req;

    g_conn_manager[manager_index].sec_con_enabled = true;
    env->bond_role_master = true;

    ble_gap_bond(&bndt);
}
#endif //BLE_CM_BOND_DISABLE

static void cm_timeout_handler(void *parameter)
{
    connection_manager_env_t *env = cm_get_env();
    ble_gap_update_data_len_t len;
    len.conn_idx = g_conn_manager[env->manager_index].conn_idx;
    if (g_conn_manager[env->manager_index].mtu > 251)
    {
        len.tx_octets = 251;
    }
    else
    {
        len.tx_octets = g_conn_manager[env->manager_index].mtu;
    }

    len.tx_time = 2120;

    //LOG_I("cm_timeout_handler");
    ble_gap_update_data_len(&len);
}

void update_data_length(uint8_t conn_idx, uint16_t mtu)
{
    LOG_I("update_data_length");
    connection_manager_env_t *env = cm_get_env();
    uint8_t manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return;
    }

    g_conn_manager[manager_index].mtu = mtu;
    env->manager_index = manager_index;

    if (!g_time_handle)
    {
        g_time_handle = rt_timer_create("ble_cm", cm_timeout_handler, NULL,
                                        rt_tick_from_millisecond(500), RT_TIMER_FLAG_SOFT_TIMER);
    }
    else
    {
        rt_timer_stop(g_time_handle);
    }
    rt_timer_start(g_time_handle);

    //ble_gap_update_data_len(&len);
}

#ifdef BLE_SVC_CHG_ENABLE
static void send_svc_changed_indication(uint8_t conn_idx)
{
    uint8_t manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return;
    }
    if (g_conn_manager[manager_index].first_bond == 1)
    {
        return;
    }

    if ((g_conn_manager[manager_index].svc_changed_ccc == INDICATION_ENABLE_VALUE) &&
            (g_conn_manager[manager_index].enc_state == ENC_STATE_ON))
    {
        sibles_send_svc_changed_ind(conn_idx, 1, 0xFFFF);
    }
}

static int ble_svc_change_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    connection_manager_env_t *env = cm_get_env();

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_write_remote_value_t value;
        uint16_t enable = 2;
        value.handle = env->svc_change.cccd_hdl;
        value.write_type = SIBLES_WRITE;
        value.len = 2;
        value.value = (uint8_t *)&enable;
        sibles_write_remote_value(env->svc_change.remote_handle, env->svc_change.remote_index, &value);
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        // Notify upper layer
        uint16_t start_handle, end_handle;

        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        memcpy(&start_handle, ind->value, 2);
        memcpy(&end_handle, ind->value + 2, 2);

        sibles_send_remote_svc_change_ind(ind->conn_idx, start_handle, end_handle);
        break;
    }
    default:
        break;
    }
    return 0;
}
#endif //BLE_SVC_CHG_ENABLE

int ble_connection_manager_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    connection_manager_env_t *env = cm_get_env();
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        connection_manager_init();
#ifndef SOC_SF32LB55X
        if (env->is_pub_key_gen)
            bt_stack_nvds_update();
#endif //SOC_SF32LB55X
        break;
    }

    case SIBLES_REMOTE_CONNECTED_IND:
    {
        LOG_I("SIBLES_REMOTE_CONNECTED_IND");
        break;
    }

    case BLE_GAP_CREATE_CONNECTION_CNF:
    {
        break;
    }
    case GAPC_CONNECTION_CFM:
    {
        LOG_I("GAPC_CONNECTION_CFM");
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        LOG_I("BLE_GAP_CONNECTED_IND");

        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        uint8_t connection_index = ind->conn_idx;
        uint8_t manager_index = allocate_connection(ind, CONNECTION_STATE_CONNECTED);

        if (manager_index == ALLOCATE_CONNECTION_FAIL)
        {
            LOG_E("max dev!");
            max_links_connection_state_change(ind);
            connection_manager_disconnect(connection_index);
            break;
        }

        //ble_gap_set_resolving_list

        g_conn_manager[manager_index].conn_idx = connection_index;
        g_conn_manager[manager_index].peer_addr = ind->peer_addr;
        g_conn_manager[manager_index].peer_addr_type = ind->peer_addr_type;
        g_conn_manager[manager_index].bond_state = BOND_STATE_NONE;
        g_conn_manager[manager_index].enc_state = ENC_STATE_NONE;
        g_conn_manager[manager_index].svc_changed_ccc = 0;
        g_conn_manager[manager_index].first_bond = 0;
        g_conn_manager[manager_index].role = ind->role;

        g_conn_manager[manager_index].connection_interval = ind->con_interval;
        g_conn_manager[manager_index].connection_latency = ind->con_latency;
        g_conn_manager[manager_index].supervision_timeout = ind->sup_to;
        g_conn_manager[manager_index].update_state = UPDATE_PARAMETER_NONE;

        ind->config_info.auth = env->connected_auth;
        for (int i = 0; i < MAX_PAIR_DEV; i++)
        {
            int ret = rt_memcmp(ind->peer_addr.addr, g_bond_info.peer_addr[i].addr.addr, BD_ADDR_LEN);
            if (ret == 0)
            {
                LOG_I("Boned device connected");
                g_conn_manager[manager_index].bond_index = i;
                g_conn_manager[manager_index].bond_state = BOND_STATE_BONDED;
                ind->config_info.auth = g_bond_info.auth[manager_index];
                ind->config_info.ltk_present = g_bond_info.ltk_present[i];
                break;
            }
        }

#ifdef BLE_CM_RESOLVE_ADDRESS
        if (env->is_rslv_update_succ == 0xFF &&
                connection_manager_get_bonded_device_num() != 0 &&
                g_conn_manager[manager_index].peer_addr_type == 1 &&
                g_conn_manager[manager_index].bond_state == BOND_STATE_NONE)
        {
            // Since RSLV update failed, so try to rslv this address.
            uint32_t irk_num = connection_manager_get_bonded_device_num();
            ble_gap_resolve_address_t *req = bt_mem_alloc(sizeof(ble_gap_resolve_address_t) + sizeof(ble_gap_sec_key_t) * irk_num);
            if (req)
            {
                uint32_t key_idx = 0;
                for (int i = 0; i < MAX_PAIR_DEV && key_idx < irk_num; i++)
                {
                    if (g_bond_info.priority[i] != 0)
                    {
                        memcpy((void *)&req->irk[key_idx++], (void *)&g_bond_info.peer_irk[i], sizeof(ble_gap_sec_key_t));
                    }
                }
                req->nb_key = irk_num;
                memcpy((void *)&req->addr, (void *)&g_conn_manager[manager_index].peer_addr, sizeof(bd_addr_t));

                ind->config_info.not_respond = 1;

                ble_gap_resolve_address(req);
                bt_mem_free(req);
            }
            else
                LOG_E("Resolve address failed[No mem]!!");
        }
#endif // BLE_CM_RESOLVE_ADDRESS

#if 0
        if (g_conn_manager[manager_index].bond_state == BOND_STATE_BONDED)
        {
            // Ask for the peer device to either start encryption
            app_sec_send_security_req(connection_index);
        }
#endif
        if (ind->config_info.not_respond)
        {
            connection_manager_connection_state_change(manager_index, CONNECTION_STATE_RESOLVING, event_id);
        }
        else
        {
            connection_manager_event_process(CM_CONNECTED_IND, sizeof(connection_manager_connect_ind_t), data);
            connection_manager_connection_state_change(manager_index, CONNECTION_STATE_CONNECTED, event_id);
        }
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        // read_bond_infor_from_flash();

        if (g_time_handle)
        {
            rt_timer_stop(g_time_handle);
        }

        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        LOG_I("BLE_GAP_DISCONNECTED_IND, %d", ind->reason);
        uint8_t connection_index = ind->conn_idx;
        uint8_t manager_index = get_manager_index_by_connection_index(connection_index);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            break;
        }

        g_conn_manager[manager_index].update_state = UPDATE_PARAMETER_NONE;
        if (env->update_timer)
        {
            rt_timer_stop(env->update_timer);
        }
        connection_manager_event_process(CM_DISCONNECTED_IND, sizeof(connection_manager_disconnected_ind_t), data);
        connection_manager_connection_state_change(manager_index, CONNECTION_STATE_DISCONNECTED, event_id);
#ifdef BLE_SVC_CHG_ENABLE
        if (env->svc_change.remote_index == ind->conn_idx)
        {
            sibles_unregister_remote_svc(env->svc_change.remote_index, env->svc_change.svc_start_handle, env->svc_change.svc_end_handle, ble_svc_change_event_handler);
        }
#endif //BLE_SVC_CHG_ENABLE
        break;
    }
#ifndef BLE_CM_BOND_DISABLE
    case BLE_GAP_BOND_IND:
    {
        LOG_I("BLE_GAP_BOND_IND");
        ble_gap_bond_ind_t *ind = (ble_gap_bond_ind_t *)data;
        process_bond_event(ind, event_id);
        break;
    }
#endif //BLE_CM_BOND_DISABLE
    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        uint8_t connection_index = ind->conn_idx;
        uint8_t manager_index = get_manager_index_by_connection_index(connection_index);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            break;
        }
        g_conn_manager[manager_index].connection_interval = ind->con_interval;
        g_conn_manager[manager_index].connection_latency = ind->con_latency;
        g_conn_manager[manager_index].supervision_timeout = ind->sup_to;
        g_conn_manager[manager_index].update_state = UPDATE_PARAMETER_NONE;
        if (env->update_timer)
        {
            rt_timer_stop(env->update_timer);
        }
        LOG_I("BLE_GAP_UPDATE_CONN_PARAM_IND %d, %d", ind->con_interval, ind->con_latency);
        connection_manager_event_process(CM_UPDATE_CONN_IND, sizeof(ble_gap_update_conn_param_ind_t), ind);
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_CNF:
    {
        ble_gap_update_conn_param_cnf_t *cnf = (ble_gap_update_conn_param_cnf_t *)data;
        if (cnf->status != 0)
        {
            uint8_t manager_index = get_manager_index_by_connection_index(cnf->conn_idx);
            if (manager_index != CM_CONN_INDEX_ERROR)
            {
                g_conn_manager[manager_index].update_state = UPDATE_PARAMETER_NONE;
            }

            if (env->update_timer)
            {
                rt_timer_stop(env->update_timer);
            }
        }
        break;
    }
    case BLE_GAP_SET_RESOLVING_LIST_CNF:
    {
        ble_gap_set_resolve_list_cnf_t *cnf = (ble_gap_set_resolve_list_cnf_t *)data;
        env->is_rslv_update_succ = cnf->status == HL_ERR_NO_ERROR ? 1 : 0xFF;
        break;

    }
#ifdef BLE_SVC_CHG_ENABLE
    case SIBLES_SVC_CHANGED_CFG:
    {
        sibles_svc_changed_cfg_t *rsp = (sibles_svc_changed_cfg_t *)data;
        LOG_I("SIBLES_SVC_CHANGED_CFG %d", rsp->ind_cfg);
        uint8_t manager_index = get_manager_index_by_connection_index(rsp->conn_idx);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            break;
        }
        g_conn_manager[manager_index].svc_changed_ccc = rsp->ind_cfg;
        send_svc_changed_indication(rsp->conn_idx);
        break;
    }
#endif //BLE_SVC_CHG_ENABLE
#ifndef BLE_CM_BOND_DISABLE
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        LOG_I("BLE_GAP_ENCRYPT_IND: %d", ind->auth);

        uint8_t connection_index = ind->conn_idx;
        uint8_t manager_index = get_manager_index_by_connection_index(connection_index);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            break;
        }
        g_conn_manager[manager_index].enc_state = ENC_STATE_ON;

#ifdef BLE_SVC_CHG_ENABLE
        send_svc_changed_indication(ind->conn_idx);
#endif //BLE_SVC_CHG_ENABLE
        connection_manager_event_process(ENCRYPT_IND_EVENT, sizeof(ble_gap_encrypt_ind_t), ind);
#ifdef BLE_SVC_CHG_ENABLE
        ble_svc_change_enable(ind->conn_idx);
#endif
        break;
    }
    case BLE_GAP_ENCRYPT_REQ_IND:
    {
        ble_gap_encrypt_req_ind_t *ind = (ble_gap_encrypt_req_ind_t *)data;
        uint8_t connection_index = ind->conn_idx;
        uint8_t manager_index = get_manager_index_by_connection_index(connection_index);
        if (manager_index == CM_CONN_INDEX_ERROR)
        {
            struct gapc_encrypt_cfm *cfm_error = sifli_msg_alloc(GAPC_ENCRYPT_CFM,
                                                 TASK_BUILD_ID(TASK_ID_GAPC, ind->conn_idx), sifli_get_stack_id(),
                                                 sizeof(struct gapc_encrypt_cfm));
            cfm_error->found = false;
            sifli_msg_send(cfm_error);
            break;
        }

        LOG_I("BLE_GAP_ENCRYPT_REQ_IND index: %d, manager index: %d, bond index: %d", connection_index,
              manager_index, g_conn_manager[manager_index].bond_index);


        // LTK value
        ble_gap_ltk_t ltk = g_bond_info.ltk[g_conn_manager[manager_index].bond_index];

        struct gapc_encrypt_cfm *cfm = sifli_msg_alloc(GAPC_ENCRYPT_CFM,
                                       TASK_BUILD_ID(TASK_ID_GAPC, ind->conn_idx), sifli_get_stack_id(),
                                       sizeof(struct gapc_encrypt_cfm));

        cfm->found = false;

        if (g_conn_manager[manager_index].bond_state == BOND_STATE_BONDED)
        {
            // Retrieve the required informations from NVDS

            if (true)
            {
                // Check if the provided EDIV and Rand Nb values match with the stored values
                if ((ind->ediv == ltk.ediv) &&
                        !rt_memcmp(&ind->rand_nb.nb[0], &ltk.randnb.nb[0], sizeof(ble_gap_rand_nb_t)))
                {
                    cfm->found = true;
                    cfm->key_size = 16;
                    rt_memcpy(&cfm->ltk, &ltk.ltk, sizeof(ble_gap_sec_key_t));
                }
            }
        }
        // Send the message
        sifli_msg_send(cfm);
        break;
    }
#endif //BLE_CM_BOND_DISABLE
    case SIBLES_MTU_EXCHANGE_IND:
    {
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        LOG_I("SIBLES_MTU_EXCHANGE_IND %d, %d", ind->conn_idx, ind->mtu);
        update_data_length(ind->conn_idx, ind->mtu);
        break;
    }
#ifdef BLE_SVC_CHG_ENABLE
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;

        if (rsp->result != HL_ERR_NO_ERROR)
        {
            break;
        }
        if (memcmp(rsp->search_uuid, generic_attribute_uuid, rsp->search_svc_len) == 0)
        {
            LOG_I("generic attribute");

            sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
            uint8_t find = 0;
            for (int i = 0; i < rsp->svc->char_count; i++)
            {
                if (memcmp(chara->uuid, svc_changed_uuid, chara->uuid_len) == 0)
                {
                    LOG_I("noti_uuid received, att handle(%x), des handle(%x)", chara->attr_hdl, chara->desc[0].attr_hdl);
                    env->svc_change.cccd_hdl = chara->desc[0].attr_hdl;
                    find = 1;
                    break;
                }

            }

            if (find == 0)
            {
                break;
            }
            env->svc_change.svc_start_handle = rsp->svc->hdl_start;
            env->svc_change.svc_end_handle = rsp->svc->hdl_end;

            env->svc_change.remote_index = rsp->conn_idx;
            env->svc_change.remote_handle = sibles_register_remote_svc(rsp->conn_idx, rsp->svc->hdl_start, rsp->svc->hdl_end, ble_svc_change_event_handler);
            LOG_I("svc change register result %d", env->svc_change.remote_handle);
        }
        break;
    }
#endif // BLE_SVC_CHG_ENABLE
#ifndef BLE_CM_BOND_DISABLE
    case BLE_GAP_BOND_REQ_IND:
    {
        ble_gap_bond_req_ind_t *ind = (ble_gap_bond_req_ind_t *)data;
        process_bond_req_ind(ind);
        break;
    }
#endif //BLE_CM_BOND_DISABLE
    case BLE_GAP_REMOTE_FEATURE_IND:
    {
        connection_manager_event_process(CM_READ_REMOTE_FEATURE_IND, len, data);
        break;
    }
#ifndef BSP_BLE_NVDS_SYNC
    case BLE_NVDS_ASYNC_READ_CNF:
    {
        LOG_I("BLE_NVDS_ASYNC_READ_CNF");
        sifli_nvds_get_value_cnf_t *cnf = (sifli_nvds_get_value_cnf_t *)data;
        if (cnf->type == SIFLI_NVDS_TYPE_CM)
        {
            if (cnf->len == 0)
            {
                LOG_D("first read");
            }
            else if (cnf->len != sizeof(conn_manager_pair_dev_t))
            {
                LOG_E("len check error! expect %d, read len %d", sizeof(conn_manager_pair_dev_t), cnf->len);
            }
            rt_memcpy(&g_bond_info, cnf->value, sizeof(conn_manager_pair_dev_t));
            for (int i = 0; i < MAX_PAIR_DEV; i++)
            {
                LOG_I("BLE_NVDS_ASYNC_READ_CNF 0x%x", g_bond_info.peer_addr[i].addr.addr[0]);
            }
            if (env->update_after_read)
            {
                check_bond_infor();
                update_resolving_list();
            }
        }
        break;
    }
    case BLE_NVDS_ASYNC_WRITE_CNF:
    {
        LOG_I("BLE_NVDS_ASYNC_WRITE_CNF");
        sifli_nvds_set_value_cnf_t *cnf = (sifli_nvds_set_value_cnf_t *)data;
        if (cnf->type == SIFLI_NVDS_TYPE_CM)
        {
            connection_manager_event_process(CM_FLASH_WRITE, len, data);
        }
        break;
    }
    case BLE_NVDS_ASYNC_READ_TAG_CNF:
    {
        sifli_nvds_read_tag_cnf_t *cnf = (sifli_nvds_read_tag_cnf_t *)data;
        if (cnf->tag == NVDS_STACK_TAG_BD_ADDRESS
                && cnf->status == NVDS_OK
                && cnf->length == BD_ADDR_LEN)
        {
            rt_memcpy(env->local_addr.addr, cnf->buffer, BD_ADDR_LEN);
        }
        LOG_I("BLE_NVDS_ASYNC_READ_TAG_CNF: %d", cnf->tag);
        break;
    }
    case BLE_NVDS_ASYNC_WRITE_TAG_CNF:
    {
        LOG_I("BLE_NVDS_ASYNC_WRITE_TAG_CNF");
        break;
    }
    case BLE_NVDS_AYSNC_FLUSH_TAG_CNF:
    {
        LOG_I("BLE_NVDS_AYSNC_FLUSH_TAG_CNF");
        break;
    }
#endif //BSP_BLE_NVDS_SYNC
    case BLE_GAP_LOCAL_BD_ADDR_IND:
    {
        ble_gap_dev_bdaddr_ind_t *ind = (ble_gap_dev_bdaddr_ind_t *)data;

        if (env->set_app_bd_addr == STATIC_RANDOM_ADDR && ind->addr.addr_type == 1)
        {
            LOG_I("updata nvds app addr");
            env->set_app_bd_addr = 0;
            set_nvds_app_addr(ind->addr.addr);
        }
        else if (env->set_app_bd_addr == STATIC_RANDOM_ADDR_TEMP && ind->addr.addr_type == 1)
        {
            struct sibles_random_addr *r_addr = bt_mem_alloc(sizeof(struct sibles_random_addr));
            BT_OOM_ASSERT(r_addr);
            if (r_addr)
            {
                env->set_app_bd_addr = 0;
                r_addr->addr_type = ind->addr.addr_type;
                memcpy(r_addr->addr, ind->addr.addr.addr, BD_ADDR_LEN);
                sibles_set_random_addr(ind->actv_idx, (uint8_t *)r_addr);
                bt_mem_free(r_addr);
            }
        }
        break;
    }
#ifdef BLE_CM_RESOLVE_ADDRESS
    case BLE_GAP_SOLVED_ADDRESS_IND:
    {

        LOG_I("Resolve addre ind");
        ble_gap_solved_addr_ind_t *ind = (ble_gap_solved_addr_ind_t *)data;
        connection_manager_update_conn_with_RPA(ind);
        break;
    }
    case BLE_GAP_RESOLVE_ADDRESS_CNF:
    {
        LOG_I("Resolve addre ret");
        ble_gap_resolve_address_cnf_t *cnf = (ble_gap_resolve_address_cnf_t *)data;
        if (cnf->status == GAP_ERR_NOT_FOUND)
            connection_manager_update_unresolving_conn();
        break;
    }
#endif // BLE_CM_RESOLVE_ADDRESS
#ifndef SOC_SF32LB55X
    case BLE_GAP_PUBLIC_KEY_GEN_IND:
    {
        env->is_pub_key_gen = 1;
        if (env->is_init)
        {
            // Force update nvds
            bt_stack_nvds_update();
        }
        break;
    }
#endif // !SOC_SF32LB55X
    case BT_DBG_ASSERT_NOTIFY_IND:
    {
        memset(env, 0, sizeof(connection_manager_env_t));
        break;
    }
    }
    return 0;
}

BLE_EVENT_REGISTER(ble_connection_manager_handler, NULL);

static void ble_cm_update_timeout(void *context)
{

    for (int i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].update_state == UPDATE_PARAMETER_UPDATING && g_conn_manager[i].connection_state == CONNECTION_STATE_CONNECTED)
        {
            LOG_E("connidx %d update fail with timeout", g_conn_manager[i].conn_idx);
            g_conn_manager[i].update_state = UPDATE_PARAMETER_NONE;
        }
    }
}

__WEAK void connection_parameter_get_high_performance(uint16_t *interval_min, uint16_t *interval_max, uint16_t *latency, uint16_t *timeout)
{
    *interval_min = HIGH_PREFORMENCE_INTERVAL_MIN;
    *interval_max = HIGH_PERFORMANCE_INTERVAL_MAX;
    *latency = HIGH_PERFORMANCE_LATENCY;
    *timeout = DEFAULT_TIMEOUT;
}

__WEAK void connection_parameter_get_balance(uint16_t *interval_min, uint16_t *interval_max, uint16_t *latency, uint16_t *timeout)
{
    *interval_min = BANLANCED_INTERVAL_MIN;
    *interval_max = BANLANCED_INTERVAL_MAX;
    *latency = BANLANCED_LANTENCY;
    *timeout = DEFAULT_TIMEOUT;
}

__WEAK void connection_parameter_get_low_power(uint16_t *interval_min, uint16_t *interval_max, uint16_t *latency, uint16_t *timeout)
{
    *interval_min = LOW_POWER_INTERVAL_MIN;
    *interval_max = LOW_POWER_INTERVAL_MAX;
    *latency = LOW_POWER_LATENCY;
    *timeout = DEFAULT_TIMEOUT;
}

static uint8_t cm_update_parameter(uint8_t conn_idx, uint8_t interval_level, uint8_t *data, enum connection_manager_update_type type)
{
    connection_manager_env_t *env = cm_get_env();
    uint8_t manager_index;
    uint16_t interval_min, interval_max, latency, time_out;

    manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return CM_CONN_INDEX_ERROR;
    }

    if (g_conn_manager[manager_index].update_state == UPDATE_PARAMETER_UPDATING)
    {
        LOG_I("skip update due to updating");
        return CM_PARAMETER_UPDATING;
    }

    latency = g_conn_manager[manager_index].connection_latency;
    time_out = g_conn_manager[manager_index].supervision_timeout;

    ble_connect_para_ind.interval_level = interval_level;
    switch (interval_level)
    {
    case CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE:
    {
        connection_parameter_get_high_performance(&interval_min, &interval_max, &latency, &time_out);

#ifdef BLE_CONNECTION_PRIORITY_SYNC
        ble_gap_update_phy_t phy;
        phy.conn_idx = conn_idx;
        phy.rx_phy = GAP_PHY_LE_2MBPS;
        phy.tx_phy = GAP_PHY_LE_2MBPS;
        ble_gap_update_phy(&phy);
#endif
        break;
    }
    case CONNECTION_MANAGER_INTERVAL_BALANCED:
    {
        connection_parameter_get_balance(&interval_min, &interval_max, &latency, &time_out);
        break;
    }
    case CONNECTION_MANAGER_INTERVAL_LOW_POWER:
    {
        connection_parameter_get_low_power(&interval_min, &interval_max, &latency, &time_out);

#ifdef BLE_CONNECTION_PRIORITY_SYNC
        ble_gap_update_phy_t phy;
        phy.conn_idx = conn_idx;
        phy.rx_phy = GAP_PHY_LE_CODED;
        phy.tx_phy = GAP_PHY_LE_CODED;
        ble_gap_update_phy(&phy);
#endif
        break;
    }
    case CONNECTION_MANAGER_INTERVAL_CUSTOMIZE:
    {
        if (data == NULL)
        {
            return CM_PARAMETER_ERROR;
        }
        cm_customize_parameter *cm_data = (cm_customize_parameter *) data;
        interval_min = cm_data->connection_interval_min;
        interval_max = cm_data->connection_interval_max;
        if (interval_min > interval_max)
        {
            return CM_PARAMETER_ERROR;
        }

        if (interval_min < CONNECTION_MANAGER_INTERVAL_MIN || interval_max > CONNECTION_MANAGER_INTERVAL_MAX)
        {
            return CM_PARAMETER_ERROR;
        }

        time_out = cm_data->supervision_timeout;
        if (time_out < 10 || time_out > 3200)
        {
            return CM_PARAMETER_ERROR;
        }

        latency = cm_data->connection_latency;
        if (latency < CONNECTION_MANAGER_LATENCY_MIN ||
                latency > CONNECTION_MANAGER_LATENCY_MAX ||
                latency > (time_out * 10 / (interval_max * 2) - 1))
        {
            return CM_PARAMETER_ERROR;
        }
        break;
    }
    default:
    {
        return CM_PARAMETER_ERROR;
    }
    }

    if (interval_max >= g_conn_manager[manager_index].connection_interval &&
            interval_min <= g_conn_manager[manager_index].connection_interval)
    {
        if (latency == g_conn_manager[manager_index].connection_latency)
        {
            LOG_I("same connection parameter");
            return CM_PARAMETER_SAME;
        }
    }

    if (!env->update_timer)
    {
        env->update_timer = rt_timer_create("ble_cm_update", ble_cm_update_timeout, NULL,
                                            60000, RT_TIMER_FLAG_SOFT_TIMER);
    }
    else
    {
        rt_timer_stop(env->update_timer);
    }
    rt_timer_start(env->update_timer);

    g_conn_manager[manager_index].update_state = UPDATE_PARAMETER_UPDATING;
    ble_gap_update_conn_param_t conn_para;
    conn_para.conn_idx = conn_idx;
    conn_para.intv_max = interval_max;
    conn_para.intv_min = interval_min;
    /* value = argv * 1.25 */
    conn_para.ce_len_max = 0x100;
    conn_para.ce_len_min = 0x1;
    conn_para.latency = latency;
    conn_para.time_out = time_out;
    ble_connect_para_ind.interval_min = interval_min;
    ble_connect_para_ind.interval_max = interval_max;
    ble_connect_para_ind.latency = latency;

    connection_para_updating_ind_t ind;
    ind.conn_idx = conn_idx;
    ind.interval_min = interval_min;
    ind.interval_max = interval_max;
    ind.slave_latency = latency;
    ind.timeout = time_out;

    connection_manager_event_process(CM_UPDATING_PARA_IND, sizeof(connection_para_updating_ind_t), (uint8_t *)&ind);

    if (type == UPDATE_TYPE_L2CAP)
    {
        ble_gap_update_conn_param_on_l2cap(&conn_para);
    }
    else if (type == UPDATE_TYPE_LL)
    {
        ble_gap_update_conn_param(&conn_para);
    }
    return CM_STATUS_OK;
}

// default use l2cap update
uint8_t connection_manager_update_parameter(uint8_t conn_idx, uint8_t interval_level, uint8_t *data)
{
    return cm_update_parameter(conn_idx, interval_level, data, UPDATE_TYPE_L2CAP);
}

uint8_t connection_manager_update_parameter_with_type(uint8_t conn_idx, uint8_t interval_level, uint8_t *data, enum connection_manager_update_type type)
{
    return cm_update_parameter(conn_idx, interval_level, data, type);
}

uint8_t connection_manager_get_connetion_parameter(uint8_t conn_idx, uint8_t *data)
{
    uint8_t manager_index;
    cm_conneciont_parameter_value_t value;

    manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return CM_CONN_INDEX_ERROR;
    }
    value.interval = g_conn_manager[manager_index].connection_interval;
    value.slave_latency = g_conn_manager[manager_index].connection_latency;
    value.supervision_timeout = g_conn_manager[manager_index].supervision_timeout;

    rt_memcpy(data, &value, sizeof(cm_conneciont_parameter_value_t));
    return CM_STATUS_OK;
}

// slave ask to start bond
uint8_t connection_manager_set_link_security(uint8_t conn_index, uint8_t sec_level)
{
    uint8_t manager_index = get_manager_index_by_connection_index(conn_index);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return CM_CONN_INDEX_ERROR;
    }

    ble_gap_sec_req_t sec_req;
    sec_req.conn_idx = conn_index;

    switch (sec_level)
    {
    case LE_SECURITY_LEVEL_NO_MITM_NO_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        break;
    }
    case LE_SECURITY_LEVEL_NO_MITM_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_NO_MITM_BOND;
        break;
    }
    case LE_SECURITY_LEVEL_MITM_NO_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_MITM_NO_BOND;
    }
    case LE_SECURITY_LEVEL_MITM_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_MITM_BOND;
        break;
    }
    case LE_SECURITY_LEVEL_SEC_CON_NO_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_SEC_CON_NO_BOND;
        break;
    }
    case LE_SECURITY_LEVEL_SEC_CON_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_SEC_CON_BOND;
        break;
    }
    case LE_SECURITY_LEVEL_SEC_CON_MITM_BOND:
    {
        sec_req.auth = GAP_AUTH_REQ_SEC_CON_MITM_BOND;
        break;
    }
    default:
        return CM_SECURITY_LEVEL_ERROR;
    }

    ble_gap_security_request(&sec_req);
    return CM_STATUS_OK;
}

#ifndef BLE_CM_BOND_DISABLE
uint8_t connection_manager_get_bond_state(uint8_t conn_idx)
{
    for (uint8_t i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].conn_idx == conn_idx)
        {
            return g_conn_manager[i].bond_state;
        }
    }
    return CM_CONN_INDEX_ERROR;
}

uint8_t connection_manager_get_enc_state(uint8_t conn_idx)
{
    for (uint8_t i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].conn_idx == conn_idx)
        {
            return g_conn_manager[i].enc_state;
        }
    }
    return CM_CONN_INDEX_ERROR;
}
#endif //BLE_CM_BOND_DISABLE

void connection_manager_get_all_connected_index(uint8_t *data)
{
    uint8_t dev_count = 0;
    conn_manager_get_connected_dev_t dev;
    for (uint8_t i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].connection_state == CONNECTION_STATE_CONNECTED)
        {
            dev.conn_idx[dev_count] = g_conn_manager[i].conn_idx;
            dev_count++;
        }
    }
    dev.device_count = dev_count;
    rt_memcpy(data, &dev, sizeof(conn_manager_get_connected_dev_t));
}

uint8_t connection_manager_get_connection_state(uint8_t conn_idx)
{
    for (uint8_t i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].conn_idx == conn_idx)
        {
            return g_conn_manager[i].connection_state;
        }
    }
    return CM_CONN_INDEX_ERROR;
}

uint8_t connection_manager_get_addr_by_conn_idx(uint8_t conn_idx, ble_gap_addr_t *data)
{
    uint8_t manager_index;
    ble_gap_addr_t addr;

    manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return CM_CONN_INDEX_ERROR;
    }

    addr.addr = g_conn_manager[manager_index].peer_addr;
    addr.addr_type = g_conn_manager[manager_index].peer_addr_type;

    rt_memcpy(data, &addr, sizeof(ble_gap_addr_t));
    return CM_STATUS_OK;
}

#ifndef BLE_CM_BOND_DISABLE
void connection_manager_bond_state_change(uint8_t manager_index, uint8_t new_state)
{
    uint8_t old_state;

    old_state = g_conn_manager[manager_index].bond_state;
    g_conn_manager[manager_index].bond_state = new_state;
}
#endif //BLE_CM_BOND_DISABLE

static uint8_t get_manager_index_by_connection_index(uint8_t conn_idx)
{
    for (int i = 0; i < MAX_CONNECTION_LINK_NUM; i++)
    {
        if (g_conn_manager[i].conn_idx == conn_idx)
        {
            return i;
        }
    }
    LOG_E("unexpected conn_idx %d", conn_idx);
    return CM_CONN_INDEX_ERROR;
}


int connection_manager_event_process(uint8_t command, uint16_t len, void *data)
{
    LOG_D("connection_manager_event_process 0x%x", command);

    switch (command)
    {
#ifndef BLE_CM_BOND_DISABLE
    case ENCRYPT_IND_EVENT:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        connection_manager_encrypt_ind_t cm_ind;
        cm_ind.conn_idx = ind->conn_idx;
        cm_ind.auth = ind->auth;
        ble_event_publish(CONNECTION_MANAGER_ENCRYPT_IND_EVENT, &cm_ind, len);
        break;
    }
#endif //BLE_CM_BOND_DISABLE
    case CM_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        connection_manager_connect_ind_t cm_ind;
        rt_memcpy(&cm_ind, ind, sizeof(connection_manager_connect_ind_t));
        ble_event_publish(CONNECTION_MANAGER_CONNCTED_IND, &cm_ind, len);
        break;
    }
    case CM_DISCONNECTED_IND:
    {
        ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
        connection_manager_disconnected_ind_t cm_ind;
        cm_ind.conn_idx = ind->conn_idx;
        cm_ind.reason = ind->reason;
        ble_event_publish(CONNECTION_MANAGER_DISCONNECTED_IND, &cm_ind, len);
        break;
    }
    case CM_READ_REMOTE_FEATURE_IND:
    {
        ble_gap_remote_features_ind_t *ind = (ble_gap_remote_features_ind_t *)data;
        connection_manager_remote_features_ind_t cm_ind;
        cm_ind.conn_idx = ind->conn_idx;
        rt_memcpy(cm_ind.features, ind->features, GAP_LE_FEATS_LEN);
        ble_event_publish(CONNECTION_MANAGER_READ_REMOTE_FEATURE_IND, &cm_ind, len);
        break;
    }
    case CM_FLASH_WRITE:
    {
        sifli_nvds_set_value_cnf_t *cnf = (sifli_nvds_set_value_cnf_t *)data;
        uint8_t status = cnf->status;
        ble_event_publish(CONNECTION_MANAGER_FLASH_WRITE, &status, sizeof(uint8_t));
        break;
    }
    case CM_UPDATE_CONN_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        connection_manager_update_conn_param_ind_t cm_ind;
        cm_ind.conn_idx = ind->conn_idx;
        cm_ind.con_interval = ind->con_interval;
        cm_ind.con_latency = ind->con_latency;
        cm_ind.sup_to = ind->sup_to;
        ble_event_publish(CONNECTION_MANAGER_UPDATE_CONNECTION_IND, &cm_ind, sizeof(connection_manager_update_conn_param_ind_t));
        break;
    }
#ifndef BLE_CM_BOND_DISABLE
    case CM_BOND_AUTH_INFOR_CONFIRM:
    {
        connection_manager_bond_ack_infor_t *ind = (connection_manager_bond_ack_infor_t *)data;
        ble_event_publish(CONNECTION_MANAGER_BOND_AUTH_INFOR, ind, sizeof(connection_manager_bond_ack_infor_t));
        break;
    }
    case CM_PAIRING_SUCCEED:
    {
        // ble_gap_bond_ind_t
        ble_event_publish(CONNECTION_MANAGER_PAIRING_SUCCEED, data, len);
        break;
    }
    case CM_PAIRING_FAILED:
    {
        // ble_gap_bond_ind_t
        ble_event_publish(CONNECTION_MANAGER_PAIRING_FAILED, data, len);
        break;
    }
#endif //BLE_CM_BOND_DISABLE
    case CM_UPDATING_PARA_IND:
    {
        // connection_para_updating_ind_t
        ble_event_publish(CONNECTION_MANAGER_UPDATING_CONNECTION_PARAMETER_IND, data, len);
        break;
    }
    default:
        return 0;
    }
    return 0;
}

bool connection_manager_check_conn_idx(uint8_t conn_idx)
{
    uint8_t manager_index = get_manager_index_by_connection_index(conn_idx);
    if (manager_index == CM_CONN_INDEX_ERROR)
    {
        return false;
    }
    return true;
}

bool connection_manager_check_normal_conn_idx(uint8_t conn_idx)
{
    if (conn_idx >= MAX_CONNECTION_LINK_NUM)
    {
        return false;
    }
    return true;
}

void connection_manager_use_public_addr()
{
    bd_addr_t addr;
    memset(addr.addr, 0, BD_ADDR_LEN);
    set_nvds_app_addr(addr);
}

void connection_manager_set_random_addr()
{
    connection_manager_env_t *env = cm_get_env();
    env->set_app_bd_addr = STATIC_RANDOM_ADDR;
    ble_gap_generate_rand_addr(GAP_STATIC_ADDR);
}

void connection_manager_set_temp_random()
{
    connection_manager_env_t *env = cm_get_env();
    env->set_app_bd_addr = STATIC_RANDOM_ADDR_TEMP;
    ble_gap_generate_rand_addr(GAP_STATIC_ADDR);
}

void connection_manager_stop_temp_random(uint8_t conn_idx)
{
    struct sibles_random_addr *r_addr = bt_mem_alloc(sizeof(struct sibles_random_addr));
    BT_OOM_ASSERT(r_addr);
    if (r_addr)
    {
        r_addr->addr_type = 0;
        memset(r_addr, 0, BD_ADDR_LEN);
        sibles_set_random_addr(conn_idx, (uint8_t *)r_addr);
        bt_mem_free(r_addr);
    }
}

void connection_manager_gatt_over_bredr_mtu_changed_ind(uint16_t remote_mtu, uint16_t local_mtu)
{
    LOG_I("connection_manager_gatt_over_bredr_mtu_changed_ind %d, %d", remote_mtu, local_mtu);
    connection_manager_gatt_over_bredr_mtu_ind_t ind;
    ind.remote_mtu = remote_mtu;
    ind.local_mtu = local_mtu;
    ble_event_publish(CONNECTION_MANAGER_GATT_OVER_BREDR_MTU_IND, &ind, sizeof(connection_manager_gatt_over_bredr_mtu_ind_t));
}

void connection_manager_gatt_over_bredr_register_ind(uint32_t handle)
{
    LOG_I("connection_manager_gatt_over_bredr_register_ind %d", handle);
    connection_manager_gatt_over_bredr_reg_ind_t ind;
    ind.handle = handle;
    ble_event_publish(CONNECTION_MANAGER_GATT_OVER_BREDR_REG_IND, &ind, sizeof(connection_manager_gatt_over_bredr_reg_ind_t));
}

uint8_t connection_manager_gatt_over_bredr_service_register(uint16_t uuid)
{
    uint8_t ret = 0;
    sibles_local_svc_t svc[8];
    sibles_get_all_gatt_handle(svc);
    for (int i = 0; i < 8; i++)
    {
        if (svc[i].state == 1)
        {
            if (svc[i].uuid_len == ATT_UUID_16_LEN)
            {
                if (memcmp(svc[i].uuid, &uuid, ATT_UUID_16_LEN) == 0)
                {
                    LOG_I("REG %d, %d!", svc[i].start_handle, svc[i].end_handle);
#ifdef BT_USING_GATT
                    bt_gatt_create_sdp_record(svc[i].uuid_len, svc[i].uuid, svc[i].start_handle, svc[i].end_handle);
                    ret = 1;
#endif
                    break;
                }
            }
        }
    }
    return ret;
}

uint8_t connection_manager_gatt_over_bredr_service_register_128(uint8_t *uuid)
{
    uint8_t ret = 0;
    sibles_local_svc_t svc[8];
    sibles_get_all_gatt_handle(svc);
    for (int i = 0; i < 8; i++)
    {
        if (svc[i].state == 1)
        {
            if (svc[i].uuid_len == ATT_UUID_128_LEN)
            {
                if (memcmp(svc[i].uuid, uuid, ATT_UUID_128_LEN) == 0)
                {
                    LOG_I("REG %d, %d!", svc[i].start_handle, svc[i].end_handle);
#ifdef BT_USING_GATT
                    bt_gatt_create_sdp_record(svc[i].uuid_len, svc[i].uuid, svc[i].start_handle, svc[i].end_handle);
                    ret = 1;
#endif
                    break;
                }
            }
        }
    }
    return ret;
}

uint8_t connection_manager_gatt_over_bredr_unregister(uint32_t handle)
{
#ifdef BT_USING_GATT
    bt_gatt_create_sdp_unreg(handle);
    return 0;
#else
    return 0;
#endif
}

#ifdef FINSH_USING_MSH
static void cm_cmd(uint8_t argc, char **argv)
{
    uint8_t j;
    connection_manager_env_t *env = cm_get_env();

    if (argc > 1)
    {
        if (strcmp(argv[1], "delbond") == 0)
        {
#ifndef BLE_CM_BOND_DISABLE
            connection_manager_delete_all_bond();
#endif // BLE_CM_BOND_DISABLE
        }
        else if (strcmp(argv[1], "showbond") == 0)
        {
#ifndef BLE_CM_BOND_DISABLE
            for (int i = 0; i < MAX_PAIR_DEV; i++)
            {
                LOG_I("%d, addr %x:%x:%x:%x:%x:%x, key: %x,%x,%x,%x,%x,%x,%x,%x", g_bond_info.priority[i],
                      g_bond_info.peer_addr[i].addr.addr[5], g_bond_info.peer_addr[i].addr.addr[4], g_bond_info.peer_addr[i].addr.addr[3],
                      g_bond_info.peer_addr[i].addr.addr[2], g_bond_info.peer_addr[i].addr.addr[1], g_bond_info.peer_addr[i].addr.addr[0],
                      g_bond_info.ltk[i].ltk.key[15], g_bond_info.ltk[i].ltk.key[14], g_bond_info.ltk[i].ltk.key[13],
                      g_bond_info.ltk[i].ltk.key[12], g_bond_info.ltk[i].ltk.key[11], g_bond_info.ltk[i].ltk.key[10],
                      g_bond_info.ltk[i].ltk.key[9], g_bond_info.ltk[i].ltk.key[8]);
            }
#endif //BLE_CM_BOND_DISABLE
        }
        else if (strcmp(argv[1], "secreq") == 0)
        {
#ifndef BLE_CM_BOND_DISABLE
            for (j = 0; j < MAX_CONNECTION_LINK_NUM; j++)
            {
                if (g_conn_manager[j].connection_state == CONNECTION_STATE_CONNECTED)
                {
                    break;
                }
            }
            LOG_I("j %x conn_idx %x\n", j, g_conn_manager[j].conn_idx);
            connection_manager_set_link_security(g_conn_manager[j].conn_idx, LE_SECURITY_LEVEL_SEC_CON_BOND);
#endif //BLE_CM_BOND_DISABLE
        }
#ifndef BSP_USING_PC_SIMULATOR
        else if (strcmp(argv[1], "dfu_reg") == 0)
        {
            uint8_t des_state = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
            LOG_I("check_patch_install HCPU target %d", des_state);
        }
        else if (strcmp(argv[1], "set_dfu_reg") == 0)
        {
            int val = atoi(argv[2]);
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, val);
        }
        else if (strcmp(argv[1], "reboot") == 0)
        {
            drv_reboot();
        }
#ifdef BLE_SVC_CHG_ENABLE
        else if (strcmp(argv[1], "svc_change") == 0)
        {
            ble_svc_change_enable(0);
        }
#endif //BLE_SVC_CHG_ENABLE
        else if (strcmp(argv[1], "svc_dis") == 0)
        {
#ifdef BLE_SVC_CHG_ENABLE
            sibles_unregister_remote_svc(env->svc_change.remote_index, env->svc_change.svc_start_handle, env->svc_change.svc_end_handle, ble_svc_change_event_handler);
#endif // BLE_SVC_CHG_ENABLE
        }
#endif
    }
}
MSH_CMD_EXPORT(cm_cmd, cm command);
#endif

#ifndef BLE_CM_BOND_DISABLE
#define KEY_ID_LEN 4
void connection_manager_h6_result_cb(uint8_t *aes_res, uint32_t metainfo)
{
    int i;
    uint32_t  requestType;
    i = (metainfo & 0x000000ff);
    LOG_I("h6 cb: i%x \n", i);
    requestType = metainfo >> 8;
    LOG_I("h6 cb: requestType%x metainfo%x  \n", requestType, metainfo);
    if (REQUST_LTK_ILK_H6 == requestType)
    {
        memcpy(g_bond_map_info.ilk[i].ilk, aes_res, GAP_KEY_LEN);
        connection_manager_convert_ilk_to_lk(i);
    }
    else if (REQUST_ILK_LK_H6 == requestType)
    {
        memcpy(g_bond_map_info.lk[i].lk, aes_res, GAP_KEY_LEN);
        LOG_D("lk: %x,%x,%x,%x,%x,%x,%x,%x %x,%x,%x,%x,%x,%x,%x,%x \n",
              aes_res[0], aes_res[1], aes_res[2], aes_res[3], aes_res[4], aes_res[5],
              aes_res[6], aes_res[7], aes_res[8], aes_res[9], aes_res[10], aes_res[11],
              aes_res[12], aes_res[13], aes_res[14], aes_res[15]);
//#if defined(SOC_SF32LB58X) && defined(BT_FINSH)
#ifdef BT_FINSH
        uint8_t        key_type = 0;
        sc_ble_bt_link_key_ind(g_bond_info.peer_addr[i].addr.addr, key_type, g_bond_map_info.lk[i].lk);
#endif
    }
}
void connection_manager_h7_result_cb(uint8_t *aes_res, uint32_t metainfo)
{
    int i;
    uint32_t  requestType;
    i = (metainfo & 0x000000ff);
    requestType = metainfo >> 8;
    if (REQUST_LTK_ILK_H7 == requestType)
    {
        memcpy(g_bond_map_info.ilk[i].ilk, aes_res, GAP_KEY_LEN);
        connection_manager_convert_ilk_to_lk(i);
    }
    else
    {
        LOG_E(" aes_cmac h7 requestType error \n");
    }
}
void connection_manager_convert_ilk_to_lk(int i)
{
    uint32_t cb_request;
    uint8_t keyid_labr[KEY_ID_LEN] = {0x72, 0x62, 0x65, 0x6c};
    uint8_t  result;
    cb_request = ((REQUST_ILK_LK_H6 << 8) | i);
    LOG_D("ilk to lk: cb_request%x \n", cb_request);
    result = ble_gap_aes_h6(g_bond_map_info.ilk[i].ilk, keyid_labr, cb_request);
}
void connection_manager_convert_ltk_to_ilk(int i)
{
    uint8_t w_ltk[GAP_KEY_LEN];
    uint8_t keyid_tmp1[KEY_ID_LEN] = {0x31, 0x70, 0x6d, 0x74};
    uint32_t cb_request;
    uint8_t salt[GAP_KEY_LEN];
    uint8_t  result;
    memset(salt, 0x00, GAP_KEY_LEN);
    memcpy(salt, keyid_tmp1, KEY_ID_LEN);
    memcpy(w_ltk, g_bond_info.ltk[i].ltk.key, GAP_KEY_LEN);
    LOG_D("ltk: %x,%x,%x,%x,%x,%x,%x,%x %x,%x,%x,%x,%x,%x,%x,%x \n",
          w_ltk[0], w_ltk[1], w_ltk[2], w_ltk[3], w_ltk[4], w_ltk[5],
          w_ltk[6], w_ltk[7], w_ltk[8], w_ltk[9], w_ltk[10], w_ltk[11],
          w_ltk[12], w_ltk[13], w_ltk[14], w_ltk[15]);
    if (g_bond_info.auth[i] & 0x20) //h7
    {
        cb_request = ((REQUST_LTK_ILK_H7 << 8) | i);
        result = ble_gap_aes_h7(salt, w_ltk, cb_request);
    }
    else//h6
    {
        cb_request = ((REQUST_LTK_ILK_H6 << 8) | i);
        LOG_D("ltk to ilk: cb_request%x \n", cb_request);
        result = ble_gap_aes_h6(w_ltk, keyid_tmp1, cb_request);
    }
}
#endif //BLE_CM_BOND_DISABLE

#else
void connection_manager_gatt_over_bredr_mtu_changed_ind(uint16_t remote_mtu, uint16_t local_mtu)
{
    LOG_I("connection_manager_gatt_over_bredr_mtu_changed_ind %d, %d", remote_mtu, local_mtu);
    connection_manager_gatt_over_bredr_mtu_ind_t ind;
    ind.remote_mtu = remote_mtu;
    ind.local_mtu = local_mtu;
    ble_event_publish(CONNECTION_MANAGER_GATT_OVER_BREDR_MTU_IND, &ind, sizeof(connection_manager_gatt_over_bredr_mtu_ind_t));
}
#endif // BSP_BLE_CONNECTION_MANAGER


