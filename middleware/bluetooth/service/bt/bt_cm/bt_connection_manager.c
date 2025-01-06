/**
  ******************************************************************************
  * @file   bt_connection_manager.c
  * @author Sifli software development team
  * @brief Sifli bt connection manager.
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
#include <stdio.h>

#ifdef BSP_BT_CONNECTION_MANAGER
#include "config.h"
#include "bts2_app_inc.h"
#include "bf0_sibles_nvds.h"
#include "bt_connection_manager.h"

#ifdef RT_USING_ULOG
    #include "ulog.h"
#else
    #undef DBG_WARNING
    #undef DBG_ERROR
    #include "rtdbg.h"
#endif


#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif



#define BT_CM_MAX_TIMEOUT (3000)
#define BT_CM_SNIFF_INV (798)
#define BT_CM_SNIFF_ATTEMPT (1)
#define BT_CM_SNIFF_TIMEOUT (1)
// TODO: Some time iPhone insist using 240ms, so change to 62.5ms first
#define BT_CM_SNIFF_CHG_TH (100)
#define BT_CM_SNIFF_ENTER_TIME (5) // seconds
#define PAGE_SCAN_INTVL_LOW                     0x800
#define PAGE_SCAN_WINDOW_LOW                    0x12
#define PAGE_SCAN_INTVL_HIGH                    0x200
#define PAGE_SCAN_WINDOW_HIGH                   0x24

#define INQUIRY_SCAN_INTVL_LOW                  0x800
#define INQUIRY_SCAN_WINDOW_LOW                 0x12
#define INQUIRY_SCAN_INTVL_HIGH                 0x200
#define INQUIRY_SCAN_WINDOW_HIGH                0x24



OS_TIMER_DECLAR(g_btcm_timer);

typedef struct
{
    uint8_t mode;
    float interval;
} bt_cm_gap_mode_t;


// master and slave profile target
static uint32_t g_bt_cm_mp_tar = BT_CM_DEFAULT_MASTER_BIT;
static uint32_t g_bt_cm_sp_tar = BT_CM_DEFAULT_SLAVE_BIT;
static bt_cm_env_t g_bt_cm_env;
static bt_cm_bonded_dev_t g_bt_bonded_dev;
static bt_cm_gap_mode_t g_bt_gap_mode;

static uint16_t bt_cm_get_profile_role(bt_cm_conn_role_t role, uint32_t profile_bit);
static uint32_t bt_cm_conn_get_next_profile(bt_cm_conned_dev_t *conn);
static uint32_t bt_cm_get_profile_target(bt_cm_conn_role_t role);
static void bt_fsm_hook_fun(const uint8_t *string, uint8_t state, uint8_t evt);


bt_cm_env_t *bt_cm_get_env()
{
    return &g_bt_cm_env;
}

bt_cm_bonded_dev_t *bt_cm_get_bonded_dev(void)
{
    return &g_bt_bonded_dev;
}

void bt_cm_add_bonded_dev(bt_cm_conn_info_t *dev, uint8_t force)
{
    uint32_t i;
    // Ensure current index is same device
    if ((g_bt_bonded_dev.dev_state[g_bt_bonded_dev.g_bt_cm_last_bond_idx] != 0)
            && (bd_eq(&g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].bd_addr, &dev->bd_addr) == TRUE)
            && (g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].is_reconn == dev->is_reconn))
        return;

    for (i = 0; i < BT_CM_MAX_BOND; i ++)
    {
        if (g_bt_bonded_dev.dev_state[i] == 0)
            break;
#ifdef BT_CONNECT_SUPPORT_MULTI_LINK
        else if (bd_eq(&g_bt_bonded_dev.info[i].bd_addr, &dev->bd_addr) == TRUE)
        {
            break;
        }
#else
        if (g_bt_bonded_dev.info[i].role == dev->role)
        {
            if (force)
            {
                g_bt_bonded_dev.dev_state[i] = 0;
                break;
            }

            if (BT_CM_SLAVE == g_bt_bonded_dev.info[i].role)
            {
                return;
            }

            g_bt_bonded_dev.dev_state[i] = 0;
        }
#endif
    }


#if 0
    for (i = 0; i < BT_CM_MAX_BOND; i ++)
        if ((g_bt_bonded_dev.dev_state[i] == 0) || ((g_bt_bonded_dev.dev_state[i] != 0)
                && bd_eq(&g_bt_bonded_dev.info[i].bd_addr, &dev->bd_addr) == TRUE))
            break;
#endif


    if (i == BT_CM_MAX_BOND && force)
    {
        i = (g_bt_bonded_dev.g_bt_cm_last_bond_idx + 1) % BT_CM_MAX_BOND;
    }

    if (i < BT_CM_MAX_BOND)
    {
        g_bt_bonded_dev.dev_state[i] = 1;
        memcpy(&g_bt_bonded_dev.info[i], dev, sizeof(bt_cm_conn_info_t));
        g_bt_bonded_dev.g_bt_cm_last_bond_idx = i;

        sifli_nvds_write(SIFLI_NVDS_TYPE_BT_CM, sizeof(bt_cm_bonded_dev_t), (uint8_t *)&g_bt_bonded_dev);
    }
}


bt_cm_conn_info_t *bt_cm_find_bonded_dev_by_addr(uint8_t *addr)
{
    uint32_t i;
    BTS2S_BD_ADDR     bd_addr;
    bt_addr_convert_to_bts((bd_addr_t *)addr, &bd_addr);
    for (i = 0; i < BT_CM_MAX_BOND; i ++)
    {
        if (bd_eq(&g_bt_bonded_dev.info[i].bd_addr, &bd_addr) == TRUE)
        {
            return &g_bt_bonded_dev.info[i];
        }
    }

    return NULL;
}

void bt_cm_delete_bonded_devs(void)
{
    memset(&g_bt_bonded_dev, 0, sizeof(bt_cm_bonded_dev_t));
    sifli_nvds_write(SIFLI_NVDS_TYPE_BT_CM, sizeof(bt_cm_bonded_dev_t), (uint8_t *)&g_bt_bonded_dev);
}

void bt_cm_delete_bonded_devs_and_linkkey(uint8_t *addr)
{
    uint32_t i;
    BTS2S_BD_ADDR     bd_addr;
    bt_addr_convert_to_bts((bd_addr_t *)addr, &bd_addr);
    for (i = 0; i < BT_CM_MAX_BOND; i ++)
    {
        if (bd_eq(&g_bt_bonded_dev.info[i].bd_addr, &bd_addr) == TRUE)
        {
            memset(&g_bt_bonded_dev.info[i], 0, sizeof(bt_cm_conn_info_t));
            sifli_nvds_write(SIFLI_NVDS_TYPE_BT_CM, sizeof(bt_cm_bonded_dev_t), (uint8_t *)&g_bt_bonded_dev);
            sc_unpair_req(bts2_task_get_app_task_id(), &bd_addr);
        }
    }
}

bt_cm_conned_dev_t *bt_cm_get_free_conn(bt_cm_env_t *env)
{
    uint32_t i;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (env->conn_device[i].state == BT_CM_STATE_IDLE)
        {
            return &env->conn_device[i];
        }
    }

    return NULL;
}

uint8_t bt_cm_find_conn_index_by_addr(uint8_t *addr)
{
#if defined(BT_CONNECT_SUPPORT_MULTI_LINK) && (BT_CM_MAX_CONN > 1)
    uint32_t i;
    BTS2S_BD_ADDR     bd_addr;
    bt_addr_convert_to_bts((bd_addr_t *)addr, &bd_addr);
    for (i = 0; i < BT_CM_MAX_BOND; i++)
    {
        if (bd_eq(&g_bt_bonded_dev.info[i].bd_addr, &bd_addr) == TRUE)
        {
            return i;
        }
    }
    return  BT_CM_INVALID_CONN_INDEX;
#else
    return 0;
#endif
}

uint8_t bt_cm_find_addr_by_conn_index(uint8_t idx, BTS2S_BD_ADDR *addr)
{
#if defined(BT_CONNECT_SUPPORT_MULTI_LINK) && (BT_CM_MAX_CONN > 1)
    if (idx >= BT_CM_MAX_BOND)
    {
        return -1;
    }

    memcpy(addr, &g_bt_bonded_dev.info[idx].bd_addr, sizeof(BTS2S_BD_ADDR));
#else
    if (idx >= BT_CM_MAX_CONN)
    {
        return -1;
    }
    memcpy(addr, &g_bt_cm_env.conn_device[idx].info.bd_addr, sizeof(BTS2S_BD_ADDR));
#endif
    return 0;
}


static void bt_cm_conn_destory(bt_cm_env_t *env, bt_cm_conned_dev_t *conn)
{
    uint32_t i;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (&env->conn_device[i] == conn)
        {
            memset(conn, 0, sizeof(bt_cm_conned_dev_t));
            return;
        }
    }

    // If destory wrongly pointer
    RT_ASSERT(0);
}


bt_cm_conned_dev_t *bt_cm_find_conn_by_addr(bt_cm_env_t *env, BTS2S_BD_ADDR *bd_addr)
{
    uint32_t i;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (env->conn_device[i].state != BT_CM_STATE_IDLE
                && (bd_eq(bd_addr, &env->conn_device[i].info.bd_addr) == TRUE))
        {
            return &env->conn_device[i];
        }
    }

    return NULL;
}

static bt_cm_conned_dev_t *bt_cm_find_conn_by_hdl(bt_cm_env_t *env, uint16_t hdl)
{
    uint32_t i;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (env->conn_device[i].state != BT_CM_STATE_IDLE
                && (env->conn_device[i].conn_hdl == hdl))
        {
            return &env->conn_device[i];
        }
    }

    return NULL;
}


static uint8_t bt_cm_get_conn_num(bt_cm_env_t *env)
{
    uint32_t i;
    uint8_t n = 0;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (env->conn_device[i].state != BT_CM_STATE_IDLE)
            n++;
    }

    return n;
}

static uint8_t read_bt_infor_from_flash()
{
    uint8_t ret;

    if (sizeof(g_bt_bonded_dev) > SIFLI_NVDS_KEY_LEN_BT_CM)
        RT_ASSERT(0);

    uint16_t len = sizeof(g_bt_bonded_dev);
    ret = sifli_nvds_read(SIFLI_NVDS_TYPE_BT_CM, &len, (uint8_t *)&g_bt_bonded_dev);

    if (ret != NVDS_OK)
    {
        LOG_E("read bt bonded failed!");
    }

    /*
        LOG_D("read_bt_bond_infor_from_flash: %d\n", ret);

        LOG_D("read_bt_bond_infor_from_flash: %04X:%02X:%06lX\n",
                env->conn_device.bd_addr.nap,
                env->conn_device.bd_addr.uap,
                env->conn_device.bd_addr.lap);
    */
    return ret;
}

static void bt_cm_a2dp_thread(void *parameter)
{
    rt_thread_mdelay(2000);
    connect_bt_a2dp();
}

static void bt_start_a2dp_thread()
{
    rt_thread_t tid;
    tid = rt_thread_create("bt_cm_a2dp", bt_cm_a2dp_thread, NULL, 1024, RT_THREAD_PRIORITY_LOW, 10);
    rt_thread_startup(tid);
}


extern uint8_t bts2s_avsnk_openFlag;
static bt_cm_err_t bt_cm_profile_connect(uint32_t profile_bit, bt_cm_conned_dev_t *conn)
{
    bt_cm_err_t err = BT_CM_ERR_NO_ERR;
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
    LOG_I("profile %d connect", profile_bit);
#ifdef CFG_HFP_HF
    if (profile_bit == BT_CM_HFP)
    {
        // Call HFP
        // TODO: support AG
        if (conn->info.role == BT_CM_MASTER)
            err = BT_CM_ERR_UNSUPPORTED;
        else
            hfp_hf_conn_req(&conn->info.bd_addr, HF_CONN);
    }
    else
#endif
#ifdef CFG_AV
        if (profile_bit == BT_CM_A2DP)
        {
            // Call A2DP
            if (bts2s_avsnk_openFlag == 0)
            {
                err = BT_CM_ERR_RESOURCE_NOT_ENOUGH;
            }
            else
            {
                uint16_t role = bt_cm_get_profile_role(conn->info.role, BT_CM_A2DP);
                uint16_t rmt_role;
                if (role == AV_AUDIO_SRC)
                    rmt_role = AV_AUDIO_SNK;
                else if (role == AV_AUDIO_SNK)
                    rmt_role = AV_AUDIO_SRC;
                else
                    RT_ASSERT(0);
                av_conn_req(bts2_task_get_app_task_id(), conn->info.bd_addr, rmt_role, role);
            }
        }
        else
#endif
#ifdef CFG_AVRCP
            if (profile_bit == BT_CM_AVRCP)
            {
                // Cal AVRCP
                uint16_t role = bt_cm_get_profile_role(conn->info.role, BT_CM_AVRCP);
                uint16_t rmt_role;
                if (role == AVRCP_CT)
                    rmt_role = AVRCP_TG;
                else if (role == AVRCP_TG)
                    rmt_role = AVRCP_CT;
                else
                    RT_ASSERT(0);
                avrcp_conn_req(bts2_task_get_app_task_id(), conn->info.bd_addr, rmt_role, role);
            }
// #ifdef CFG_HID
//     else if (profile_bit == BT_CM_HID)
//     {
//         hid_conn_req(bts2_task_get_app_task_id(), conn->info.bd_addr, HID_Host, HID_Device);
//     }
// #endif
            else
#endif
#ifdef CFG_PAN
                if (profile_bit == BT_CM_PAN)
                {
#ifdef BT_FINSH_PAN
                    extern void bt_pan_conn_by_addr(BTS2S_BD_ADDR * remote_addr);
                    bt_pan_conn_by_addr(&(conn->info.bd_addr));
#endif
                }
                else
#endif

                {
                    // No need to handle
                    err = BT_CM_ERR_UNSUPPORTED;
                }
#endif
    return err;
}

static void bt_cm_conn_timeout(void *parameter)
{
    // 0 is delete, 1 is restart
    uint32_t time_state = 0;
    // Only handle parameter is not NULL
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
    if (parameter != NULL)
    {
        bt_cm_conned_dev_t *conn = (bt_cm_conned_dev_t *)parameter;
        if (conn->state == BT_CM_STATE_CONNECTED
                && conn->sub_state == BT_CM_SUB_PROFILING_CONNECTING)
        {
            uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
            if (profile_bit != 0)
            {
                bt_cm_err_t err = bt_cm_profile_connect(profile_bit, conn);
                if (err == BT_CM_ERR_NO_ERR)
                    time_state = 1;
                else
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
            }
            else
                conn->sub_state = BT_CM_SUB_STATE_IDLE;
        }

        if (conn->tim_hdl)
        {
            if (time_state == 0)
            {
                rt_timer_delete(conn->tim_hdl);
                conn->tim_hdl = NULL;
            }
            else
                rt_timer_start(conn->tim_hdl);
        }

    }
    else
        RT_ASSERT(0);
#endif
}


static uint8_t bt_cm_conn_check_profile_completed(uint32_t profile_bit, bt_cm_conn_role_t role)
{
    uint32_t target = bt_cm_get_profile_target(role);
    uint32_t left = target ^ (profile_bit & target);
    return left != 0 ? 0 : 1;
}

uint32_t bt_cm_filter_profile(uint32_t profile)
{
#ifndef CFG_AVRCP
    profile &= ~BT_CM_AVRCP;
#endif
#if !defined(CFG_AV_SRC)&&!defined(CFG_AV_SNK)
    profile &= ~BT_CM_A2DP;
#endif
#if !defined(CFG_HFP_HF)&&!defined(CFG_HFP_AG)
    profile &= ~BT_CM_HFP;
#endif
#ifndef CFG_HID
    profile &= ~BT_CM_HID;
#endif
#ifndef CFG_PAN
    profile &= ~BT_CM_PAN;
#endif

    return profile;
}

static uint32_t bt_cm_conn_get_next_profile(bt_cm_conned_dev_t *conn)
{
    uint32_t profile_bit = 0;
    if (conn)
    {
        uint32_t target = bt_cm_get_profile_target(conn->info.role);
        target = bt_cm_filter_profile(target);
        uint32_t left = target ^ (conn->conned_profiles & target);
        uint32_t i;
        for (i = 0; i < 32; i++)
        {
            if (left & (1 << i))
                break;
        }
        if (i < 32)
            profile_bit = 1 << i;
    }

    return profile_bit;
}


static uint32_t bt_cm_get_profile_target(bt_cm_conn_role_t role)
{
    if (role == BT_MASTER_ROLE)
        return g_bt_cm_mp_tar;
    else if (role == BT_SLAVE_ROLE)
        return g_bt_cm_sp_tar;
    else return 0;

}

static uint16_t bt_cm_get_profile_role(bt_cm_conn_role_t role, uint32_t profile_bit)
{
    uint16_t profile_role = 0;
    if (role == BT_MASTER_ROLE)
    {
        if (profile_bit == BT_CM_HFP)
            profile_role = 0; //TODO: No role to select currently
#ifdef CFG_AV_SRC
        else if (profile_bit == BT_CM_A2DP)
            profile_role = AV_AUDIO_SRC;
#endif

#ifdef CFG_AVRCP
        else if (profile_bit == BT_CM_AVRCP)
            profile_role = AVRCP_TG;
#endif
    }
    else
    {
        if (profile_bit == BT_CM_HFP)
            profile_role = 0; //TODO: No role to select currently
#ifdef CFG_AV_SNK
        else if (profile_bit == BT_CM_A2DP)
            profile_role = AV_AUDIO_SNK;
#endif
#ifdef CFG_AVRCP
        else if (profile_bit == BT_CM_AVRCP)
            profile_role = AVRCP_TG;
#endif
    }
    return profile_role;
}

void bt_cm_set_profile_target(uint32_t setProfile, bt_cm_conn_role_t role, uint8_t addFlag)
{
    if ((BT_MASTER_ROLE == role) && (1 == addFlag))
    {
        g_bt_cm_mp_tar |= setProfile;
    }
    else if ((BT_MASTER_ROLE == role) && (0 == addFlag))
    {
        g_bt_cm_mp_tar = setProfile;
    }
    else if ((BT_SLAVE_ROLE == role) && (1 == addFlag))
    {
        g_bt_cm_sp_tar |= setProfile;
    }
    else if ((BT_SLAVE_ROLE == role) && (0 == addFlag))
    {
        g_bt_cm_sp_tar = setProfile;
    }
}



uint8_t bt_cm_get_reconnect_flag_by_role(bt_cm_conn_role_t role)
{
    uint8_t reconn_flag = 0;

#ifdef BT_AUTO_CONNECT_LAST_DEVICE
    if (role == BT_CM_SLAVE)
        reconn_flag = 1;
#endif

    return reconn_flag;
}



void init_bt_cm()
{
    bt_cm_env_t *env = bt_cm_get_env();
    memset(env, 0, sizeof(bt_cm_env_t));

#ifdef CFG_OPEN_SCAN
    env->close_process = BT_CM_NO_CLOSE;
#else
    env->close_process = BT_CM_CLOSE_COMPLETE;
#endif

    read_bt_infor_from_flash();

}

void bt_cm_set_reconnect_conn_device(bt_cm_conn_info_t *info)
{
    uint8_t ret = 0xFF;
    if (info)
        ret = sifli_nvds_write(SIFLI_NVDS_TYPE_BT_CM, sizeof(bt_cm_conn_info_t), (uint8_t *)info);
    LOG_I("set_last_connect_conn_device: %d\n", ret);
}

int bt_cm_close_bt(void)
{
    bt_cm_env_t *env = bt_cm_get_env();
    uint8_t  i;
    uint8_t  conn_num = 0;


    gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);

    env->close_process = BT_CM_ON_CLOSE_PROCESS;

    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (env->conn_device[i].state >= BT_CM_STATE_CONNECTED)
        {
            conn_num++;
            gap_close_req(&env->conn_device[i].info.bd_addr);
        }
    }

    LOG_I("bt_cm_close_bt, close_process%d conn_num %d ", env->close_process, conn_num);

    if (0 == conn_num)
    {
        gap_close_req(NULL);
        env->close_process = BT_CM_CLOSE_COMPLETE;
        for (i = 0; i < BT_CM_MAX_CONN; i++)
        {
            env->conn_device[i].state =  BT_CM_STATE_IDLE;
        }
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_CLOSE_COMPLETE, NULL, 0);
    }

    return 0;
}

int bt_cm_open_bt(void)
{
    bt_cm_env_t *env = bt_cm_get_env();

    env->close_process = BT_CM_NO_CLOSE;
    gap_open_req();
    gap_wr_scan_enb_req(bts2_task_get_app_task_id(), TRUE, TRUE);
    return 0;
}
int bt_cm_open_bt_scan(uint8_t scan)
{
    BOOL inquiry_scan = true, page_scan = true;

    bt_cm_env_t *env = bt_cm_get_env();

    env->close_process = BT_CM_NO_CLOSE;
    gap_open_req();
    if (scan == 0)
    {
        inquiry_scan    =   true;
        page_scan       =   true;
        LOG_I("Both inquiry_scan and page_scan open!\n");
    }
    else if (scan == 1)
    {
        inquiry_scan    =   true;
        page_scan       =   false;
        LOG_I("Only inquiry_scan  open!\n");
    }
    else if (scan == 2)
    {
        inquiry_scan    =   false;
        page_scan       =   true;
        LOG_I("Only page_scan  open!\n");
    }

    gap_wr_scan_enb_req(bts2_task_get_app_task_id(), inquiry_scan, page_scan);
    return 0;
}

void bt_cm_close_bt_complete_check(bt_cm_env_t *env, bt_cm_conned_dev_t *conn)
{
    uint8_t ret = 0;
    if (conn != NULL)
    {
        if (conn->tim_hdl)
        {
            rt_timer_stop(conn->tim_hdl);
            rt_timer_delete(conn->tim_hdl);
            conn->tim_hdl = NULL;
        }
        bt_cm_conn_destory(env, conn);
    }

    // double check:Avoid scan and page
    gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
    if (0 == bt_cm_get_conn_num(env))
    {

        if (BT_CM_NO_CLOSE != env->close_process)
        {
            bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_CLOSE_COMPLETE, NULL, 0);
        }

        env->close_process = BT_CM_CLOSE_COMPLETE;
    }


    LOG_I("bt_cm_close_bt_complete_check, close_process%d", env->close_process);
}


void bt_cm_close_boundary_condition(bt_cm_env_t *env, bt_cm_conned_dev_t *conn, BTS2S_DM_EN_ACL_OPENED_IND *ind)
{
    LOG_I("bt_cm_close_protect, close_process%d st %d conn %d", env->close_process, ind->st, conn);

    if (ind->st == HCI_SUCC)
    {
        if (conn == NULL)
        {
            gap_disconnect_req(&ind->bd);
        }
        else
        {
            bt_cm_conn_destory(env, conn);
        }
    }

    gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
}

void bt_cm_reconnect_last_device(void)
{
    LOG_D("[%s] dev_state:%d is_reconn:%d\n", __func__, g_bt_bonded_dev.dev_state[g_bt_bonded_dev.g_bt_cm_last_bond_idx],
          g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].is_reconn);
    if (g_bt_bonded_dev.dev_state[g_bt_bonded_dev.g_bt_cm_last_bond_idx] != 0
            && g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].is_reconn)
    {
        bt_cm_err_t ret = bt_cm_connect_req(&g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].bd_addr, g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].role);
        LOG_D("BT reconnected status %d\n", ret);
    }
    return;
}


uint8_t bt_cm_last_device_is_valid(void)
{
    BTS2S_BD_ADDR bd_addr = {0};
    if (g_bt_bonded_dev.dev_state[g_bt_bonded_dev.g_bt_cm_last_bond_idx] == 0
            && bd_eq(&g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].bd_addr, &bd_addr))
    {
        return 0;
    }

    return 1;
}

void bt_cm_disconnect_req(void)
{
    bt_cm_env_t *env = bt_cm_get_env();
    uint32_t i;
    for (i = 0; i < BT_CM_MAX_CONN; i++)
    {
        if (env->conn_device[i].state >= BT_CM_STATE_CONNECTED)
        {
            gap_disconnect_req(&env->conn_device[i].info.bd_addr);
        }
    }
}

void bt_cm_last_device_bd_addr(bt_mac_t *bd_addr_c)
{
    RT_ASSERT(bd_addr_c);
    bt_addr_convert_to_general(&g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].bd_addr, (bd_addr_t *)bd_addr_c);
}

void bt_cm_change_page_activity(uint8_t is_high)
{
    uint16_t interval = is_high ? PAGE_SCAN_INTVL_HIGH : PAGE_SCAN_INTVL_LOW;
    uint16_t window = is_high ? PAGE_SCAN_WINDOW_HIGH : PAGE_SCAN_WINDOW_LOW;
    hcia_wr_pagescan_activity(interval, window, NULL);
}

void bt_cm_change_inquiryscan_activity(uint8_t is_high)
{
    uint16_t interval = is_high ? INQUIRY_SCAN_INTVL_HIGH : INQUIRY_SCAN_INTVL_LOW;
    uint16_t window = is_high ? INQUIRY_SCAN_WINDOW_HIGH : INQUIRY_SCAN_WINDOW_LOW;
    hcia_wr_inquiryscan_activity(interval, PAGE_SCAN_WINDOW, NULL);
}

#ifdef CFG_HFP_HF
int bt_cm_hf_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();
    switch (event_id)
    {
    case BTS2MU_HF_CONN_IND:
    {
        BTS2S_HF_CONN_IND *ind = (BTS2S_HF_CONN_IND *)msg;
        LOG_I("hf connected\r\n %d", ind->srv_chnl);

#if 0
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
        // If conn not existed, it should be disconnected in GAP connected handle
        if (conn)
        {
            conn->conned_profiles |= BT_CM_HFP;
            if (conn->incoming)
            {
                if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                {
                    if (conn->tim_hdl)
                    {
                        rt_timer_stop(conn->tim_hdl);
                        rt_timer_delete(conn->tim_hdl);
                        conn->tim_hdl = NULL;
                    }
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }
            else
            {
                uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                if (profile_bit != 0)
                    bt_cm_profile_connect(profile_bit, conn);
                else
                {
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }

        }
#endif
        break;
    }
    case BTS2MU_HF_CONN_CFM:
    {
        BTS2S_HF_CONN_CFM *ind = (BTS2S_HF_CONN_CFM *)msg;
        LOG_I("hf conn cfm %d", ind->res);

#ifdef BT_AUTO_CONNECT_LAST_DEVICE
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
        // If conn not existed, it should be disconnected in GAP connected handle
        if (conn)
        {
            conn->conned_profiles |= BT_CM_HFP;
            if (conn->incoming)
            {
                if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                {
                    if (conn->tim_hdl)
                    {
                        rt_timer_stop(conn->tim_hdl);
                        rt_timer_delete(conn->tim_hdl);
                        conn->tim_hdl = NULL;
                    }
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }
            else
            {
                uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                if (ind->res != 4 && profile_bit != 0)
                    bt_cm_profile_connect(profile_bit, conn);
                else
                {
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }

        }
#endif
        break;
    }
    case BTS2MU_HF_DISC_IND:
    {
        BTS2S_HF_DISC_IND *ind = (BTS2S_HF_DISC_IND *)msg;
        LOG_I("hf dis-connected %d\r\n", ind->res);
        break;
    }
    default:
        break;
    }
    return 0;
}
#else
#define bt_cm_hf_event_handler(event_id,msg) 0
#endif

#ifdef CFG_AV
int bt_cm_a2dp_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();

    switch (event_id)
    {
    case BTS2MU_AV_CONN_IND:
    {
        BTS2S_AV_CONN_IND *ind = (BTS2S_AV_CONN_IND *)msg;
        LOG_I("a2dp connect ind %d\r\n", ind->conn_id);
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
        // If conn not existed, it should be disconnected in GAP connected handle
        if (conn)
        {
            conn->conned_profiles |= BT_CM_A2DP;
            if (conn->incoming)
            {
                if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                {
                    if (conn->tim_hdl)
                    {
                        rt_timer_stop(conn->tim_hdl);
                        rt_timer_delete(conn->tim_hdl);
                        conn->tim_hdl = NULL;
                    }
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }
            else
            {
                uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                if (profile_bit != 0)
                    bt_cm_profile_connect(profile_bit, conn);
                else
                {
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }

        }
#endif
        break;
    }
    case BTS2MU_AV_CONN_CFM:
    {

        BTS2S_AV_CONN_CFM *ind = (BTS2S_AV_CONN_CFM *)msg;
        LOG_I("a2dp connect cfm %d res %d\r\n", ind->conn_id, ind->res);
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
        // If conn not existed, it should be disconnected in GAP connected handle
        if (conn)
        {
            conn->conned_profiles |= BT_CM_A2DP;
            if (conn->incoming)
            {
                if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                {
                    if (conn->tim_hdl)
                    {
                        rt_timer_stop(conn->tim_hdl);
                        rt_timer_delete(conn->tim_hdl);
                        conn->tim_hdl = NULL;
                    }
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }
            else
            {
                uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                if (profile_bit != 0)
                    bt_cm_profile_connect(profile_bit, conn);
                else
                {
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }

        }
#endif
        break;

    }
    case BTS2MU_AV_DISC_IND:
    {
        BTS2S_AV_DISC_IND *ind = (BTS2S_AV_DISC_IND *)msg;
        LOG_I("a2dp dis-connected(%d) %d\r\n", ind->conn_id, ind->res);
        break;
    }
    default:
        break;
    }
    return 0;

}
#else
#define bt_cm_a2dp_event_handler(event_id,msg) 0
#endif

#ifdef CFG_HID
int bt_cm_hid_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();
    switch (event_id)
    {
    case BTS2MU_HID_CONN_IND:
    {
        BTS2S_HID_CONN_IND *ind = (BTS2S_HID_CONN_IND *)msg;
        LOG_I("hid conn ind\n");
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
        // If conn not existed, it should be disconnected in GAP connected handle
        if (conn && ind->local_psm == BT_PSM_HID_INTR)
        {
            conn->conned_profiles |= BT_CM_HID;
            if (conn->incoming)
            {
                if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                {
                    if (conn->tim_hdl)
                    {
                        rt_timer_stop(conn->tim_hdl);
                        rt_timer_delete(conn->tim_hdl);
                        conn->tim_hdl = NULL;
                    }
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }
            else
            {
                uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                if (profile_bit != 0)
                    bt_cm_profile_connect(profile_bit, conn);
                else
                {
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }

        }
#endif
        break;
    }
    case BTS2MU_HID_CONN_CFM:
    {
        BTS2S_HID_CONN_CFM *ind = (BTS2S_HID_CONN_CFM *)msg;
        LOG_I("hid conn cfm %d", ind->res);

#ifdef BT_AUTO_CONNECT_LAST_DEVICE
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
        // If conn not existed, it should be disconnected in GAP connected handle
        if (conn && ind->local_psm == BT_PSM_HID_INTR)
        {
            conn->conned_profiles |= BT_CM_HID;
            if (conn->incoming)
            {
                if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                {
                    if (conn->tim_hdl)
                    {
                        rt_timer_stop(conn->tim_hdl);
                        rt_timer_delete(conn->tim_hdl);
                        conn->tim_hdl = NULL;
                    }
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }
            else
            {
                uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                if (profile_bit != 0)
                    bt_cm_profile_connect(profile_bit, conn);
                else
                {
                    conn->sub_state = BT_CM_SUB_STATE_IDLE;
                }
            }

        }
#endif
        break;
    }
    case BTS2MU_HID_DISC_IND:
    {
        BTS2S_HID_DISC_IND *ind = (BTS2S_HID_DISC_IND *)msg;
        LOG_I("hid dis-connected by remote\n");
        break;
    }
    case BTS2MU_HID_DISC_CFM:
    {
        BTS2S_HID_DISC_CFM *ind = (BTS2S_HID_DISC_CFM *)msg;
        LOG_I("hid dis-connected by local\n");
        break;
    }
    default:
        break;
    }
    return 0;
}
#else
#define bt_cm_hid_event_handler(event_id,msg) 0
#endif

#ifdef CFG_PAN
int bt_cm_pan_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();

    switch (event_id)
    {
    case BTS2MU_PAN_CONN_IND:
    {
        BTS2S_PAN_CONN_IND *ind = (BTS2S_PAN_CONN_IND *)msg;

        LOG_I("pan connect ind\n");

        if (ind->res == BTS2_SUCC)
        {
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
            bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd_addr);
            // If conn not existed, it should be disconnected in GAP connected handle
            if (conn)
            {
                conn->conned_profiles |= BT_CM_PAN;
                if (conn->incoming)
                {
                    if (bt_cm_conn_check_profile_completed(conn->conned_profiles, conn->info.role) == 1)
                    {
                        if (conn->tim_hdl)
                        {
                            rt_timer_stop(conn->tim_hdl);
                            rt_timer_delete(conn->tim_hdl);
                            conn->tim_hdl = NULL;
                        }
                        conn->sub_state = BT_CM_SUB_STATE_IDLE;
                    }
                }
                else
                {
                    uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
                    if (profile_bit != 0)
                        bt_cm_profile_connect(profile_bit, conn);
                    else
                    {
                        conn->sub_state = BT_CM_SUB_STATE_IDLE;
                    }
                }

            }
#endif
        }
        break;
    }
    case BTS2MU_PAN_DISC_IND:
    {
        BTS2S_PAN_DISC_IND *ind = (BTS2S_PAN_DISC_IND *)msg;
        LOG_I("pan dis-connected %d\r\n", ind->res);
        break;
    }
    default:
        break;
    }
    return 0;
}
#endif

int bt_cm_gap_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();

    switch (event_id)
    {
    // Using RD LOCAL NAME CFM as app init completed
    case BTS2MU_GAP_RD_LOCAL_NAME_CFM:
    {
        LOG_I("BT CM rd local dev cfm");
        bt_system_mask_clear(BT_RESET_MASK_BT);
        bt_fsm_hook_set(bt_fsm_hook_fun);

#if defined(BT_AUTO_CONNECT_LAST_DEVICE)
        if (g_bt_bonded_dev.dev_state[g_bt_bonded_dev.g_bt_cm_last_bond_idx] != 0
                && g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].is_reconn)
        {
            if (BT_CM_NO_CLOSE == env->close_process)
                bt_cm_connect_req(&g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].bd_addr, g_bt_bonded_dev.info[g_bt_bonded_dev.g_bt_cm_last_bond_idx].role);
        }
        else
#endif // BT_AUTO_CONNECT_LAST_DEVICE
            if ((BT_CM_NO_CLOSE == env->close_process) && (bt_cm_get_conn_num(env) < BT_CM_MAX_CONN))
            {
                gap_wr_scan_enb_req(bts2_task_get_app_task_id(), TRUE, TRUE);
            }

        break;
    }
    case BTS2MU_GAP_DISCOV_RES_IND:
    {
        BTS2S_GAP_DISCOV_RES_IND *ind = (BTS2S_GAP_DISCOV_RES_IND *)msg;
        BTS2S_DEV_NAME *name = calloc(1, sizeof(BTS2S_DEV_NAME));

        strncpy((char *)name, (char *)ind->dev_disp_name, MAX_FRIENDLY_NAME_LEN);

        LOG_I("address: %04X:%02X:%06lX--class: %06lX--name: %s\n",
              ind->bd.nap,
              ind->bd.uap,
              ind->bd.lap,
              ind->dev_cls,
              name);

        bt_mem_free(name);
        break;
    }
    case BTS2MU_GAP_MODE_CHANGED_IND:
    {
        BTS2MU_GAP_MODE_CHANGED_IND_t *ind = (BTS2MU_GAP_MODE_CHANGED_IND_t *)msg;
        uint8_t mod_str[3][7] = {"Active", "Hold", "Sniff"};

        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);

        if (conn)
        {
            if (conn->sniff_changing && ind->st != HCI_SUCC)
            {
                LOG_E("Adjust sniff failed");
                conn->sniff_changing = 0;
            }

            if (ind->mode == ACT_MODE && conn->sniff_changing)
            {
                conn->sniff_changing = 0;
                hcia_sniff_mode(&ind->bd, BT_CM_SNIFF_INV, BT_CM_SNIFF_INV, BT_CM_SNIFF_ATTEMPT, BT_CM_SNIFF_TIMEOUT, NULL);
            }
            if (ind->mode == SNIFF_MODE &&
                    ind->st == HCI_SUCC)
            {
                if (ind->interval < BT_CM_SNIFF_CHG_TH)
                {
                    conn->sniff_changing = 1;
                    hcia_exit_sniff_mode(&ind->bd, NULL);
                }
            }
        }

        g_bt_gap_mode.mode = ind->mode;
        g_bt_gap_mode.interval = 0.0;

        if (ind->mode > PARK_MODE)
            LOG_W("abnormal mode %d", ind->mode);
        else
        {
            g_bt_gap_mode.interval = (float)ind->interval * 5 / 8;
            LOG_D("%s mode st: %d, inv: %.2f", mod_str[ind->mode], ind->st, g_bt_gap_mode.interval);
        }
        break;
    }
    case BTS2MU_GAP_ENCRYPTION_IND:
    {
        BTS2S_GAP_ENCRYPTION_IND *ind = (BTS2S_GAP_ENCRYPTION_IND *)msg;

        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);

        LOG_I("BTS2MU_GAP_ENCRYPTION_IND");

        //conn->rmt_smc = 1;

        if ((NULL != conn) && (NULL != conn->tim_hdl)) //incoming call
        {
            // rt_timer_start(conn->tim_hdl);
            uint8_t addr[6];
            bt_addr_convert(&ind->bd, addr);
            bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_ENCRYPTION, addr, 6);
            INFO_TRACE("encryption ind\n");
        }



        break;
    }
    default:
        break;
    }

    return 0;
}


int bt_cm_hci_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();
    switch (event_id)
    {
    case DM_EN_ACL_OPENED_IND:
    {
        BTS2S_DM_EN_ACL_OPENED_IND *ind = (BTS2S_DM_EN_ACL_OPENED_IND *)msg;

        LOG_I("link connected COD:%d Incoming:%d res %d\r\n", ind->dev_cls, ind->incoming, ind->st);
        LOG_I("bd addr %x-%x-%x\r\n", ind->bd.nap, ind->bd.uap, ind->bd.lap);


        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, &ind->bd);
#ifdef BT_CONNECT_SUPPORT_MULTI_LINK
        if (conn == NULL)
        {
            conn = bt_cm_get_free_conn(env);
            if (conn != NULL)
            {
                memcpy(&conn->info.bd_addr, &ind->bd, sizeof(BTS2S_BD_ADDR));
                // Re-use phdl to get conn hdl
                conn->conn_hdl = ind->phdl;
                conn->info.dev_cls = ind->dev_cls;
                conn->info.role = BT_CM_SLAVE;

                conn->info.is_reconn = bt_cm_get_reconnect_flag_by_role(conn->info.role);
                conn->incoming = ind->incoming;
                conn->state = BT_CM_STATE_CONNECTED;
                conn->sub_state = BT_CM_SUB_PROFILING_CONNECTING;
                bt_cm_conn_info_t *bonded_dev = bt_cm_find_bonded_dev_by_addr(bd_addr_c.addr);
                if (bonded_dev)
                {
                    conn->info.role = bonded_dev->role;
                }
            }
        }
        RT_ASSERT(conn);
        bt_cm_add_bonded_dev(&conn->info, 1);
#endif

        bt_notify_device_acl_conn_info_t acl_info;
        bt_addr_convert(&ind->bd, acl_info.mac.addr);
        acl_info.res = ind->st;
        acl_info.acl_dir = ind->incoming;
        acl_info.dev_cls = ind->dev_cls;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_ACL_CONNECTED, &acl_info, sizeof(bt_notify_device_acl_conn_info_t));
        // Enable first
        if (ind->st != HCI_SUCC)
        {
            if (BT_CM_NO_CLOSE == env->close_process)
            {
                gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 1, 1);
            }
        }

        if (BT_CM_NO_CLOSE != env->close_process)
        {
            bt_cm_close_boundary_condition(env, conn, ind);
            break;
        }

        if (conn == NULL)
        {
            // Should Act as slave
            if (ind->incoming == 0)
                LOG_E("Connection is not created by connection manager");

            if (ind->st == HCI_SUCC)
            {
                conn = bt_cm_get_free_conn(env);

                // Disconnect directly
                if (conn == NULL)
                {
                    LOG_I("no free conn\r\n");
                    gap_disconnect_req(&ind->bd);
                    break;
                }

                memcpy(&conn->info.bd_addr, &ind->bd, sizeof(BTS2S_BD_ADDR));

                // Re-use phdl to get conn hdl
                conn->conn_hdl = ind->phdl;
                conn->info.dev_cls = ind->dev_cls;
                conn->info.role = BT_CM_SLAVE;

                conn->info.is_reconn = bt_cm_get_reconnect_flag_by_role(conn->info.role);
                conn->incoming = ind->incoming;
                conn->state = BT_CM_STATE_CONNECTED;
                conn->sub_state = BT_CM_SUB_PROFILING_CONNECTING;

#ifdef RT_USING_BT
                bt_cm_conn_info_t *bonded_dev = bt_cm_find_bonded_dev_by_addr(acl_info.mac.addr);
                if (bonded_dev)
                {
                    conn->info.role = bonded_dev->role;
                }
#endif

                hcia_wr_lp_settings_keep_sniff_interval(&ind->bd, HCI_LINK_POLICY_NO_CHANGE, BT_CM_SNIFF_ENTER_TIME, BT_CM_SNIFF_INV, BT_CM_SNIFF_INV, BT_CM_SNIFF_ATTEMPT, BT_CM_SNIFF_TIMEOUT, NULL);

                bt_cm_add_bonded_dev(&conn->info, 1);
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
                if (ind->incoming)
                {
                    conn->tim_hdl = rt_timer_create("btcm_con", bt_cm_conn_timeout, conn,
                                                    rt_tick_from_millisecond(BT_CM_MAX_TIMEOUT), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);

                    rt_timer_start(conn->tim_hdl);   //start the timer,when gap receive BTS2MU_GAP_RMT_SMC_IND
                }
#endif
            }
        }
        else
        {
            if (ind->st == HCI_SUCC)
            {

                // Re-use phdl to get conn hdl
                conn->conn_hdl = ind->phdl;
                conn->info.dev_cls = ind->dev_cls;
                conn->incoming = ind->incoming;
                conn->info.is_reconn = bt_cm_get_reconnect_flag_by_role(conn->info.role);
                conn->state = BT_CM_STATE_CONNECTED;
                conn->sub_state = BT_CM_SUB_PROFILING_CONNECTING;
                hcia_wr_lp_settings_keep_sniff_interval(&ind->bd, HCI_LINK_POLICY_NO_CHANGE, BT_CM_SNIFF_ENTER_TIME, BT_CM_SNIFF_INV, BT_CM_SNIFF_INV, BT_CM_SNIFF_ATTEMPT, BT_CM_SNIFF_TIMEOUT, NULL);
                bt_cm_add_bonded_dev(&conn->info, 1);
                // Since profile cause link establishment, currently must a profile is connecting
            }
            else if (ind->st == HCI_ERR_PAGE_TIMEOUT)
            {
                // Try re-connect
                // If not reconnect ,should destory
                bt_cm_conn_destory(env, conn);
            }
            else
            {
                bt_cm_conn_destory(env, conn);
            }
        }

        if ((bt_cm_get_conn_num(env) + 1) > BT_CM_MAX_CONN)
        {
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
        }

        LOG_I("ACL_OPENED  conn->state-%x\r\n", conn->state);
        break;
    }
    case DM_ACL_DISC_IND:
    {
        BTS2S_DM_ACL_DISC_IND *ind = (BTS2S_DM_ACL_DISC_IND *)msg;
        LOG_I("link dis-connected %x %d process:%d\r\n", ind->hdl, ind->reason, env->close_process);

        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_hdl(env, ind->hdl);
        //conn->rmt_smc = 0;

        bt_notify_device_base_info_t device_info;
        bt_addr_convert(&ind->cur_bd, device_info.mac.addr);
        device_info.res = ind->reason;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_ACL_DISCONNECTED, &device_info, sizeof(bt_notify_device_base_info_t));

#ifdef CFG_HID
        hid_reset_req();
#endif

#ifdef CFG_AV
        a2dp_reset_req();
#endif

#ifdef CFG_AVRCP
        avrcp_reset_req();
#endif

        //close bt branch
        if (BT_CM_ON_CLOSE_PROCESS == env->close_process)
        {
            bt_cm_close_bt_complete_check(env, conn);
            return 0;
        }

        // Enable inquiry and page scan
// #ifndef RT_USING_UTEST
        if (BT_CM_NO_CLOSE == env->close_process)
        {
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 1, 1);
        }
// #endif

        if (conn != NULL)
        {
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
            BTS2S_BD_ADDR recon_addr = conn->info.bd_addr;
#endif
            bt_cm_conn_role_t role = conn->info.role;
            uint8_t is_reconn = conn->info.is_reconn;

            if (conn->tim_hdl)
            {
                rt_timer_stop(conn->tim_hdl);
                rt_timer_delete(conn->tim_hdl);
                conn->tim_hdl = NULL;
            }
            bt_cm_conn_destory(env, conn);
            if (ind->reason == HCI_ERR_CONN_TIMEOUT
                    && is_reconn)
            {
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
                // Try re-connect
                bt_cm_connect_req(&recon_addr, role);
#endif
            }
        }
        break;
    }
    case DM_SM_LINK_KEY_IND:
    {
        BTS2S_DM_SM_LINK_KEY_IND *ind = (BTS2S_DM_SM_LINK_KEY_IND *)msg;
        LOG_D("Link key(%x-%x-%x) type: %d received: ", ind->bd.nap, ind->bd.uap, ind->bd.lap, ind->key_type);
        LOG_HEX("Key:", 16, ind->key, LINK_KEY_SIZE);
        break;
    }
    default:
        break;
    }
    return 0;

}


int bt_cm_sc_event_handler(uint16_t event_id, uint8_t *msg)
{
    bt_cm_env_t *env = bt_cm_get_env();
    switch (event_id)
    {
    case BTS2MU_SC_RD_PAIRED_DEV_KEY_CFM:
    {
        BTS2S_SC_RD_PAIRED_DEV_KEY_CFM *ind = (BTS2S_SC_RD_PAIRED_DEV_KEY_CFM *)msg;
        LOG_I("Get link key (%x-%x-%x), is_valid(%d):", ind->bd.nap, ind->bd.uap, ind->bd.lap, ind->res == 0);
        if (ind->res == 0)
        {
            LOG_HEX("Key :", 16, ind->link_key, LINK_KEY_SIZE);
        }
        break;
    }
    default:
        break;
    }

    return 0;
}

int bt_cm_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    if (type == BTS2M_HFP_HF)
    {
        bt_cm_hf_event_handler(event_id, msg);
    }
    else if (type == BTS2M_AV)
    {
        bt_cm_a2dp_event_handler(event_id, msg);
    }
    else if (type == BTS2M_HCI_CMD)
    {
        bt_cm_hci_event_handler(event_id, msg);
    }
    else if (type == BTS2M_GAP)
    {
        bt_cm_gap_event_handler(event_id, msg);
    }
#ifdef CFG_HID
    else if (type == BTS2M_HID)
    {
        bt_cm_hid_event_handler(event_id, msg);
    }
#endif
#ifdef CFG_PAN
    else if (type == BTS2M_PAN)
    {
        bt_cm_pan_event_handler(event_id, msg);
    }
#endif
    else if (type == BTS2M_SC)
    {
        bt_cm_sc_event_handler(event_id, msg);
    }

    return 0;

}
BT_EVENT_REGISTER(bt_cm_event_hdl, NULL);


bt_cm_err_t bt_cm_set_target_profiles_by_role(uint32_t profile_bits, bt_cm_conn_role_t role)
{
    bt_cm_err_t ret = BT_CM_ERR_NO_ERR;
    if (role == BT_CM_MASTER)
        g_bt_cm_mp_tar = profile_bits;
    else if (role == BT_CM_SLAVE)
        g_bt_cm_sp_tar = profile_bits;
    else
        ret = BT_CM_ERR_INVALID_PARA;

    return ret;
}

// As master
bt_cm_err_t bt_cm_connect_req(BTS2S_BD_ADDR *bd_addr, bt_cm_conn_role_t role)
{

    bt_cm_env_t *env = bt_cm_get_env();
    bt_cm_err_t err = BT_CM_ERR_GENERAL_ERR;
#ifdef BT_AUTO_CONNECT_LAST_DEVICE
    do
    {
        bt_cm_conned_dev_t *conn = bt_cm_find_conn_by_addr(env, bd_addr);
        if (conn)
        {
            err = BT_CM_ERR_CONN_EXISTED;
            break;
        }

        conn = bt_cm_get_free_conn(env);
        if (conn == NULL)
        {
            err = BT_CM_ERR_RESOURCE_NOT_ENOUGH;
            break;
        }

        memcpy(&conn->info.bd_addr, bd_addr, sizeof(BTS2S_BD_ADDR));
        conn->info.role = role;
        conn->state = role == BT_CM_MASTER ? BT_CM_STATE_CONNECTING : BT_CM_STATE_RECONNECTING;
        conn->sub_state = BT_CM_SUB_STATE_IDLE;

        uint32_t profile_bit = bt_cm_conn_get_next_profile(conn);
        if (profile_bit)
        {
            bt_cm_err_t ret = bt_cm_profile_connect(profile_bit, conn);
            LOG_I("Reconnect ret %d", ret);
            if (ret != BT_CM_ERR_NO_ERR)
            {
                err = BT_CM_ERR_INVALID_PARA;
                bt_cm_conn_destory(env, conn);
                break;
            }

            // Avoid scan and page
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), 0, 0);
        }
        err = BT_CM_ERR_NO_ERR;

    }
    while (0);
#endif
    return err;

}

static void bt_fsm_hook_fun(const uint8_t *string, uint8_t state, uint8_t evt)
{
    // Not print l2cap recv data to avoid too much data in A2DP streaming scenario.
    if (strcmp((const char *)string, "L2C_CH") == 0 && state == 2 && (evt == 7 || evt == 9))
        return;

    if (strcmp((const char *)string, "L2C_CID") == 0 && state == 5 && evt == 3)
        return;

    if (strcmp((const char *)string, "RFC_MUX") == 0 && state == 6 && evt == 25)
        return;

    if (strcmp((const char *)string, "RFC_DLC") == 0 && state == 4 && (evt == 14 || evt == 28))
        return;

    if (strcmp((const char *)string, "no_module") == 0)
        return;

    LOG_D("fsm: %s st:%d evt:%d", string, state, evt);
}


void bt_cm(uint8_t argc, char **argv)
{

    if (argc > 1)
    {
        if (strcmp(argv[1], "discon") == 0)
        {
            bt_cm_env_t *env = bt_cm_get_env();
            uint32_t i;
            for (i = 0; i < BT_CM_MAX_CONN; i++)
            {
                if (env->conn_device[i].state >= BT_CM_STATE_CONNECTED)
                    gap_disconnect_req(&env->conn_device[i].info.bd_addr);
            }
        }
        else if (strcmp(argv[1], "search") == 0)
        {
            BTS2S_CPL_FILTER inq_filter;

            inq_filter.filter = BTS2_INQ_FILTER_CLEAR;
            inq_filter.dev_mask_cls = BT_DEVCLS_MISC;
            gap_discov_req(bts2_task_get_app_task_id(), atoi(argv[2]), atoi(argv[3]), &inq_filter, TRUE);
        }
        else if (strcmp(argv[1], "conn") == 0)
        {
            BTS2S_BD_ADDR bd_addr;
            bd_addr.nap = strtol(argv[2], NULL, 0);
            bd_addr.uap = strtol(argv[3], NULL, 0);
            bd_addr.lap = strtol(argv[4], NULL, 0);
            LOG_I("conn addr: %04X:%02X:%06lX",
                  bd_addr.nap,
                  bd_addr.uap,
                  bd_addr.lap);
            bt_cm_connect_req(&bd_addr, atoi(argv[5]));
        }
        else if (strcmp(argv[1], "delete") == 0)
        {
            bt_cm_delete_bonded_devs();
        }
        else if (strcmp(argv[1], "close") == 0)
        {
            bt_cm_close_bt();
        }
        else if (strcmp(argv[1], "open") == 0)
        {

            if (argc == 2)
                bt_cm_open_bt();
            else if (argc == 3)
            {
                if (strcmp(argv[2], "inquiry") == 0)
                {
                    bt_cm_open_bt_scan(1);
                }
                else if (strcmp(argv[2], "page") == 0)
                {
                    bt_cm_open_bt_scan(2);
                }


            }
        }
        else if (strcmp(argv[1], "rd_addr") == 0)
        {
            gap_rd_local_bd_req(bts2_task_get_app_task_id());
        }
        else if (strcmp(argv[1], "scan") == 0)
        {
            uint8_t is_on = atoi(argv[2]);
            gap_wr_scan_enb_req(bts2_task_get_app_task_id(), is_on, is_on);
        }
        else if (strcmp(argv[1], "ch_addr") == 0)
        {
            uint8_t addr_type = atoi(argv[2]);
            uint8_t addr_method = atoi(argv[3]);
            BTS2S_BD_ADDR bd_addr;
            bd_addr_t bd_addr_c;
            bd_addr.nap = strtol(argv[4], NULL, 0);
            bd_addr.uap = strtol(argv[5], NULL, 0);
            bd_addr.lap = strtol(argv[6], NULL, 0);
            LOG_I("conn addr: %04X:%02X:%06lX",
                  bd_addr.nap,
                  bd_addr.uap,
                  bd_addr.lap);

            bt_addr_convert_to_general(&bd_addr, &bd_addr_c);
            sibles_change_bd_addr(addr_type, addr_method, &bd_addr_c);
        }
        else if (strcmp(argv[1], "en_btlog") == 0)
        {
            uint8_t type = atoi(argv[2]);
            if (type == 0) // FSM
            {
                uint8_t is_on = atoi(argv[3]);
                if (is_on)
                    bt_fsm_hook_set(bt_fsm_hook_fun);
                else
                    bt_fsm_hook_set(NULL);
            }
        }
        else if (strcmp(argv[1], "dut") == 0)
        {
            gap_enb_dut_mode_req(bts2_task_get_app_task_id());
        }
        else if (strcmp(argv[1], "sleep") == 0)
        {
#ifdef SOC_SF32LB52X
            uint8_t is_enable = atoi(argv[2]);
            if (is_enable == 0)
                bt_sleep_control(0);
            else
                bt_sleep_control(1);
#else
            LOG_E("Not Support!!!");
#endif
        }
        else if (strcmp(argv[1], "uart_dut") == 0)
        {
            gap_enb_dut_mode_req(bts2_task_get_app_task_id());
            rt_thread_mdelay(500);
#ifndef BSP_USING_PC_SIMULATOR
#ifdef BT_RF_TEST
            {
                extern void uart_ipc_path_change(void);
                uart_ipc_path_change();
            }
#endif //BT_RF_TEST
#endif
        }
        else if (strcmp(argv[1], "get_gap_mode") == 0)
        {
            uint8_t mod_str[3][7] = {"Active", "Hold", "Sniff"};
            if (g_bt_gap_mode.mode > PARK_MODE)
                LOG_I("bt_gap_mode:abnormal,inv:%.2f", g_bt_gap_mode.interval);
            else
                LOG_I("bt_gap_mode:%s,inv:%.2f", mod_str[g_bt_gap_mode.mode], g_bt_gap_mode.interval);
        }
        else if (strcmp(argv[1], "exit_sniff") == 0)
        {
            bt_cm_env_t *env = bt_cm_get_env();
            uint32_t i;
            for (i = 0; i < BT_CM_MAX_CONN; i++)
            {
                if (env->conn_device[i].state >= BT_CM_STATE_CONNECTED)
                    hcia_exit_sniff_mode(&env->conn_device[i].info.bd_addr, NULL);
            }
        }
        else if (strcmp(argv[1], "get_link_key") == 0)
        {
            uint32_t i;
            for (i = 0; i < BT_CM_MAX_BOND; i++)
            {
                if (g_bt_bonded_dev.dev_state[i] != 0)
                {
                    sc_rd_paired_dev_link_key_req(bts2_task_get_app_task_id(), &g_bt_bonded_dev.info[i].bd_addr);
                }
            }
        }
#ifdef BSP_BQB_TEST
        else if (strcmp(argv[1], "av_conn") == 0)
        {
            BTS2S_BD_ADDR bd_addr;
            bd_addr.nap = strtol(argv[2], NULL, 0);
            bd_addr.uap = strtol(argv[3], NULL, 0);
            bd_addr.lap = strtol(argv[4], NULL, 0);
            LOG_I("conn addr: %04X:%02X:%06lX",
                  bd_addr.nap,
                  bd_addr.uap,
                  bd_addr.lap);
            bt_av_conn(&bd_addr, AV_SRC);
        }
        else if (strcmp(argv[1], "av_disconn") == 0)
        {
            bt_av_disconnect(0);
        }
        else if (strcmp(argv[1], "avrcp_conn") == 0)
        {
            BTS2S_BD_ADDR bd_addr;
            bd_addr.nap = strtol(argv[2], NULL, 0);
            bd_addr.uap = strtol(argv[3], NULL, 0);
            bd_addr.lap = strtol(argv[4], NULL, 0);
            LOG_I("conn addr: %04X:%02X:%06lX",
                  bd_addr.nap,
                  bd_addr.uap,
                  bd_addr.lap);
            if (atoi(argv[5]))
                avrcp_conn_req(bts2_task_get_app_task_id(), bd_addr, AVRCP_TG, AVRCP_CT);
            else
                avrcp_conn_req(bts2_task_get_app_task_id(), bd_addr, AVRCP_CT, AVRCP_TG);
        }
        else if (strcmp(argv[1], "release_a2dp") == 0)
        {
            bt_av_release_stream(0);
        }
#endif
    }
}




#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(bt_cm, BT connection manager command);
#endif // RT_USING_FINSH


#endif // BSP_BT_CONNECTION_MANAGER
