/**
  ******************************************************************************
  * @file   bf0_ble_common.c
  * @author Sifli software development team
  * @brief Header file - Bluetooth common interface implement.
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
#include <stdlib.h>
#include "bf0_ble_common.h"
#include "bf0_ble_gap_internal.h"
#include "bf0_sibles_util.h"
#include "bf0_sibles_internal.h"
#include "bf0_ble_err.h"
#include "bluetooth_int.h"
//#include  "register.h"
#ifndef BSP_USING_PC_SIMULATOR
    #include "ble_rf_cal.h"
#endif

#include "bf0_sibles_nvds.h"

#define LOG_TAG "btcomm"
#include "log.h"

#ifdef BSP_USING_DATA_SVC
    #include "data_service.h"
#endif

#define EFUSE_OFFSET_UID        0
#define BT_TAG_D 0
#define BT_TAG_I 1
#define BT_TAG_W 2
#define BT_TAG_E 3
#define BT_TAG_V 4

#ifdef ZBT
#define sifli_get_stack_id() TASK_ID_AHI
void connection_manager_gatt_over_bredr_mtu_changed_ind(uint16_t remote_mtu, uint16_t local_mtu)
{
}
int32_t ble_event_process(uint16_t const msgid, void const *param,
                          uint16_t const dest_id, uint16_t const src_id)
{
    return 0;
}

uint8_t sibles_change_bd_addr(sibles_change_bd_addr_type_t type, sibles_change_bd_addr_method_t method, bd_addr_t *addr)
{
    return 0;
}

#endif
/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

const static bd_addr_t default_addr =
{
    .addr = {0x08, 0x05, 0x02, 0x03, 0x05, 0xAB},
};

uint8_t ble_get_local_irk(ble_gap_sec_key_t *local_irk)
{
    ble_gap_sec_key_t temp_irk;
    sifli_nvds_read_tag_t read_tag;
    int8_t ret;
    if (!local_irk)
        return GAP_ERR_INVALID_PARAM;

    read_tag.tag = NVDS_APP_TAG_LOC_IRK;
    read_tag.length = NVDS_APP_LEN_LOC_IRK;
    ret = sifli_nvds_read_tag_value(&read_tag, temp_irk.key);
    // First support sync operation.
    // TODO: async operation.
    if (ret == 0)
        memcpy(local_irk->key, temp_irk.key, NVDS_APP_LEN_LOC_IRK);
    else
        return GAP_ERR_INVALID_PARAM;
    return HL_ERR_NO_ERROR;

}


uint8_t ble_get_public_address(bd_addr_t *addr)
{
    bd_addr_t temp_addr;
    sifli_nvds_read_tag_t read_tag;
    int8_t ret;
    if (!addr)
        return GAP_ERR_INVALID_PARAM;

    read_tag.tag = NVDS_STACK_TAG_BD_ADDRESS;
    read_tag.length = NVDS_STACK_LEN_BD_ADDRESS;
    ret = sifli_nvds_read_tag_value(&read_tag, temp_addr.addr);
    if (ret == 0)
        memcpy(addr->addr, temp_addr.addr, NVDS_STACK_LEN_BD_ADDRESS);
    else
        return GAP_ERR_INVALID_PARAM;
    return HL_ERR_NO_ERROR;



}

#ifdef BT_FINSH
uint8_t bt_addr_convert_to_general(BTS2S_BD_ADDR *src_addr, bd_addr_t *dest_addr)
{
    if (src_addr == NULL || dest_addr == NULL)
        return PRF_ERR_INVALID_PARAM;

    dest_addr->addr[0] = src_addr->lap & 0xFF;
    dest_addr->addr[1] = (src_addr->lap >> 8) & 0xFF;
    dest_addr->addr[2] = (src_addr->lap >> 16) & 0xFF;
    dest_addr->addr[3] = src_addr->uap & 0xFF;
    dest_addr->addr[4] = src_addr->nap & 0xFF;
    dest_addr->addr[5] = (src_addr->nap >> 8) & 0xFF;

    return HL_ERR_NO_ERROR;
}


uint8_t bt_addr_convert_to_bts(bd_addr_t  *src_addr, BTS2S_BD_ADDR *dest_addr)
{
    if (src_addr == NULL || dest_addr == NULL)
        return PRF_ERR_INVALID_PARAM;

    dest_addr->nap = (src_addr->addr[5] << 8) | src_addr->addr[4];
    dest_addr->uap = src_addr->addr[3];
    dest_addr->lap = (src_addr->addr[2] << 16) | (src_addr->addr[1] << 8) | src_addr->addr[0];

    return HL_ERR_NO_ERROR;
}

#endif



static int8_t stohex(char ch)
{

    int8_t r = 0;
    if (ch >= '0' && ch <= '9')
        r = ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        r = ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        r = ch - 'A' + 10;
    else
        r = -1;

    return r;
}


// Only handle str foramt as : xxbxxbxxbxxbxxbxx or xxxxxxxxxxxx. x is a hex string(0-9,a-f,A-F),
// b could only string except 0
int bt_addr_convert_from_string_to_general(char *hexstr, bd_addr_t *addr)
{
    int i = 0;
    int j = 0;

    while (*(hexstr + j) && i < BD_ADDR_LEN)
    {

        uint8_t high_part = stohex(*(hexstr + j++));
        uint8_t low_part = stohex(*(hexstr + j++));
        if (low_part < 0 || high_part < 0)
            break;

        addr->addr[i] = (high_part << 4) + low_part;
        i++;

        int8_t temp = stohex(*(hexstr + j));
        if (temp < 0)
        {
            temp = stohex(*(hexstr + j + 1));
            if (temp < 0)
                break;
            j++;
        }
    }
    return i;
}



__WEAK ble_common_update_type_t ble_request_public_address(bd_addr_t *addr)
{
    memcpy(addr, (const void *)&default_addr, sizeof(bd_addr_t));
    return BLE_UPDATE_NO_UPDATE;
}

extern int rand(void);

__WEAK ble_common_update_type_t ble_request_local_irk(ble_gap_sec_key_t *local_irk)
{
    ble_gap_sec_key_t temp_key;
    for (int i = 0; i < GAP_KEY_LEN; i++)
    {
        temp_key.key[i] = (uint8_t)(rand() & 0xFF);
    }

    memcpy(local_irk, &temp_key, sizeof(ble_gap_sec_key_t));
    return BLE_UPDATE_ONCE;
}

static void ble_generate_public_address(void)
{
    bd_addr_t pub_addr;
    ble_common_update_type_t type = ble_request_public_address(&pub_addr);
    /* Ret != 0 indicate no need to update address. */
    if (type == BLE_UPDATE_NO_UPDATE)
        return;

    // NVDS has default address
    uint8_t ret = ble_nvds_update_address(&pub_addr, type, 0);
    if (ret != NVDS_OK)
        LOG_W("Update addr failed %d", ret);
}


static void ble_generate_loc_irk(void)
{
    ble_gap_sec_key_t loc_irk;
    ble_common_update_type_t type = ble_request_local_irk(&loc_irk);

    if (type == BLE_UPDATE_NO_UPDATE)
        return;

    sifli_nvds_write_tag_t *write_req = bt_mem_alloc(sizeof(sifli_nvds_write_tag_t) + NVDS_APP_LEN_LOC_IRK);
    if (!write_req)
    {
        LOG_E("Public address generate failed!");
        return;
    }

    write_req->value.tag = NVDS_APP_TAG_LOC_IRK;
    write_req->value.len = NVDS_APP_LEN_LOC_IRK;
    memcpy(write_req->value.value, loc_irk.key, NVDS_APP_LEN_LOC_IRK);
    write_req->type = type;
    write_req->is_flush = 0;

    sifli_nvds_write_tag_value(write_req);
    bt_mem_free(write_req);
}

void ble_nvds_config_prepare()
{
    ble_generate_public_address();
    ble_generate_loc_irk();
    sifli_nvds_flush();
}

#if defined(BSP_BLE_SIBLES) && !defined(SF32LB55X)
#define NVDS_PATTERN 0x4E564453

#define SIFLI_NVDS_UNINIT 0
#define SIFLI_NVDS_READY 1
#define SIFLI_NVDS_UPDATING 2

#if defined(SOC_SF32LB58X)
    #ifdef SF32LB58X_3SCO
        #define NVDS_BUFF_START NVDS_BUF_START_ADDR
    #else
        #define NVDS_BUFF_START 0x204FFD00
    #endif
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB56X)
    #define NVDS_BUFF_START 0x2041FD00
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB52X)
    #define NVDS_BUFF_START 0x2040FE00
    #define NVDS_BUFF_SIZE 512
#else
    #define NVDS_BUFF_START 0
    #define NVDS_BUFF_SIZE 0
#endif


typedef struct
{
    uint32_t pattern;
    uint16_t used_mem;
    uint16_t writting;
} sifli_nvds_mem_init_t;


void bt_stack_nvds_init(void)
{
#ifdef SOC_SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif
    uint8_t *ptr = bt_mem_alloc(NVDS_BUFF_SIZE);
    uint16_t len = NVDS_BUFF_SIZE;
    uint8_t ret = sifli_nvds_read(SIFLI_NVDS_TYPE_STACK, &len, ptr);

    if (ret != NVDS_OK)
    {
        len = NVDS_BUFF_SIZE;
        ret = sifli_nvds_get_default_vaule(ptr, &len);
    }

    if (ret == NVDS_OK)
    {
        sifli_nvds_mem_init_t *ptr1 = (sifli_nvds_mem_init_t *)NVDS_BUFF_START;
        ptr1->pattern = NVDS_PATTERN;
        ptr1->used_mem = len;
        ptr1->writting = 0;
        memcpy((void *)(ptr1 + 1), (void *)ptr, len);
    }
    else
    {
        LOG_E("Stack nvds init failed!");
    }

    bt_mem_free(ptr);

#ifdef SOC_SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif

}


void bt_stack_nvds_update(void)
{
#ifdef SOC_SF32LB52X
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif

    sifli_nvds_mem_init_t *ptr1 = (sifli_nvds_mem_init_t *)NVDS_BUFF_START;
    if (ptr1->pattern == NVDS_PATTERN && ptr1->writting == 0)
    {
        sifli_nvds_write(SIFLI_NVDS_TYPE_STACK, ptr1->used_mem, (uint8_t *)(ptr1 + 1));
    }
#ifdef SOC_SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif

}
#endif // defined(BSP_BLE_SIBLES) && !defined(SF32LB55X)

int bt_mac_addr_generate_via_uid(bd_addr_t *addr)
{
#ifdef SOC_BF0_HCPU
    uint8_t uid[16];
    int32_t ret = HAL_EFUSE_Read(EFUSE_OFFSET_UID, uid, 16);
    bd_addr_t addr_0 = {0};
    uint32_t i;

    if (!addr)
        return -1;

    if (ret != 16)
    {
        return -2;
    }

    for (i = 0; i < 16; i++)
        if (uid[i] != 0)
            break;

    LOG_HEX("uid", 8, uid, 16);

    if (i >= 16)
    {
        // uid is 0
        return -3;
    }


    uint32_t lower_part = uid[8] | (uid[9] << 8) | ((uid[10] & 0x07) << 16);
    uint32_t lot_part = ((uid[10] & 0xF8) >> 3) | ((uid[11] & 0x7F) << 5) | ((uid[12] << 12) | (uid[13] << 20) | ((uid[14] & 0x3) << 28));
    uint32_t higher_part = 0;
    uint32_t base = 36;
    LOG_D("lot %x", lot_part);
    for (i = 0; i < 5; i++)
    {
        if (i == 0)
            higher_part += (lot_part & 0x3F) % 36;
        else
        {
            higher_part += (lot_part & 0x3F) % 36 * base;
            base *= 36;
        }
        lot_part = lot_part >> 6;
    }
    LOG_D("high %x, lower %d", higher_part, lower_part);
    uint64_t mac = (uint64_t)(lower_part & 0xFFFFF) | (((uint64_t)higher_part & 0xFFFFFFF) << 20);
    //uint64_t rand;
    LOG_D("mac %llx", mac);
    //memcpy(&rand, uid, sizeof(rand));
    //LOG_I("rand %llx", rand);
    //mac ^= rand;
    memcpy(addr, &mac, sizeof(bd_addr_t));
    LOG_D("mac %llx", mac);
    return 0;
#else
    return -4;
#endif

}


int bt_mac_addr_generate_via_uid_v2(bd_addr_t *addr)
{
    int result = -4;

#ifdef SOC_BF0_HCPU
    uint8_t uid[16] = {0};
    uint8_t pattern;
    int32_t ret = HAL_EFUSE_Read(EFUSE_OFFSET_UID, uid, 16);
    uint8_t use_v2 = 0;
    uint32_t i;

    if (!addr)
        return -1;

    if (ret != 16)
    {
        return -2;
    }

    LOG_HEX("uid", 8, uid, 16);

    for (i = 0; i < 16; i++)
        if (uid[i] != 0)
            break;


    if (i >= 16)
    {
        // uid is 0
        return -3;
    }


    pattern = uid[7];


    LOG_D("pattern %x", pattern);

    if (pattern == 0xA5)
    {
        uint8_t chk_sum = uid[6];
        uint8_t chk_sum_cal = uid[0] + uid[1] + uid[2] + uid[3] + uid[4] + uid[5];
        LOG_D("chk_sum %d, cal %d", chk_sum, chk_sum_cal);

        if (chk_sum == chk_sum_cal)
            use_v2 = 1;
    }

    if (use_v2)
    {
        memcpy((void *)&addr->addr[0], (void *)&uid[0], 6);
        LOG_HEX("addr", 8, addr->addr, 6);
        result = 0;
    }
    else
        result = -5;
#endif

    return result;
}


int bt_mac_addr_generate_rand_addr_via_uid(bd_addr_t *addr)
{
#ifdef SOC_BF0_HCPU
    uint8_t uid[16], rand_o[16];
    uint8_t rand_e[16] = {0};
    uint8_t addr_t[6];
    RNG_HandleTypeDef   RngHandle;
    uint32_t rand;
    int32_t ret = HAL_EFUSE_Read(EFUSE_OFFSET_UID, uid, 16);
    uint32_t i;

    if (!addr)
        return -1;

    if (ret != 16)
    {
        return -2;
    }

    for (i = 0; i < 16; i++)
        if (uid[i] != 0)
            break;

    //LOG_HEX("uid", 8, (rt_uint8_t *)uid, 16);

    if (i >= 16)
    {
        // uid is 0
        return -3;
    }

    RngHandle.Instance = hwp_trng;

    if (HAL_RNG_Init(&RngHandle) != HAL_OK)
    {
        LOG_E("rng init failed");
        return -4;
    }

    if (HAL_RNG_Generate(&RngHandle, &rand,  1) != HAL_OK)
    {
        LOG_E("rng generate seed failed");
        return -5;
    }

    if (HAL_RNG_Generate(&RngHandle, &rand,  0) != HAL_OK)
    {
        LOG_E("rng generate failed");
        return -6;
    }

    if (HAL_RNG_Generate(&RngHandle, (uint32_t *)&uid[0],  0) != HAL_OK)
    {
        LOG_E("rng generate failed");
        return -6;
    }


    if (HAL_RNG_Generate(&RngHandle, (uint32_t *)&uid[4],  0) != HAL_OK)
    {
        LOG_E("rng generate failed");
        return -6;
    }

    //LOG_HEX("rand", 16, (rt_uint8_t *)&rand, 4);

    rand = rand & 0xFFFFFF;
    memcpy((void *)addr_t, (void *)&rand, 3);
    memcpy((void *)rand_e, (void *)&rand, 3);

    HAL_AES_init((uint32_t *)uid, 16, NULL, AES_MODE_ECB);
    HAL_AES_run(AES_ENC, (uint8_t *)rand_e, (uint8_t *)rand_o, 16);
    memcpy((void *)&addr_t[3], (void *)rand_o, 3);

    //LOG_HEX("ah", 16, (rt_uint8_t *)&rand_o, 16);


    memcpy((void *)addr, (void *)addr_t, 6);

    //LOG_HEX("addr", 16, (rt_uint8_t *)addr, 6);
    return 0;

#else
    return -7;
#endif

}

#if defined(SOC_BF0_HCPU) && defined(BSP_USING_DATA_SVC) && !defined(DATA_SVC_MBOX_THREAD_DISABLED)

#include "data_service.h"
static int bt_common_data_srv_callback(data_callback_arg_t *arg)
{
    OS_ASSERT(arg);
    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        if (rsp->result == 0)
        {
            data_msg_t msg;
            data_service_init_msg(&msg, MSG_SERVICE_START_REQ, 0);
            rt_err_t ret = datac_send_msg(rsp->handle, &msg);
            LOG_I("BT enable LCPU %d.", ret);
        }
        else
        {
            LOG_W("subscribe failed(%d), rsp->result");
        }
#if defined(BLUETOOTH) && defined(SOC_BF0_HCPU) && !defined(SOC_SF32LB55X)
        extern rt_err_t sifli_sem_release_ex(void);
        sifli_sem_release_ex();
#endif
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        break;
    }
    case MSG_SERVICE_START_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)arg->data;
        LOG_I("BT enabled result %d", rsp->result);
        break;
    }
    default:
        break;
    }
    return 0;
}


static void lcpu_bt_enable(void)
{
    datac_handle_t bt_lcpu_handle = datac_open();
    OS_ASSERT(DATA_CLIENT_INVALID_HANDLE != bt_lcpu_handle);
    datac_subscribe(bt_lcpu_handle, "BT_EN", bt_common_data_srv_callback, (uint32_t)NULL);
}
#endif


static void ble_core_sleep_lock()
{

}


static void ble_core_sleep_unlock()
{

}

extern uint8_t lcpu_power_on(void);

uint8_t ble_power_on(void)
{
    ble_core_sleep_lock();

#ifdef SOC_BF0_HCPU
    sifli_nvds_init();
    ble_nvds_config_prepare();
#if defined(BSP_BLE_SIBLES) && !defined(SF32LB55X)
    bt_stack_nvds_init();
#endif

#if defined(BSP_USING_DATA_SVC) && !defined(DATA_SVC_MBOX_THREAD_DISABLED)
    lcpu_bt_enable();
#else
    lcpu_power_on();
#endif // BSP_USING_DATA_SVC

#endif
    ble_core_sleep_unlock();
    return 0;
}

uint8_t ble_power_off(uint8_t pwroff_type)
{
    if (pwroff_type == BLE_OFF)
    {
        // Just power off BLE
    }
    else
    {
        // Power off BCPU
    }
    return 0;
}

#ifndef SOC_SF32LB55X

static uint8_t bt_reset_mask;

void bt_system_mask_clear(uint8_t mask)
{
    if (mask == BT_RESET_MASK_BT || mask == BT_RESET_MASK_BLE)
    {
        LOG_I("Clear mask %d", mask);
        bt_reset_mask &= ~mask;
    }
    else
        LOG_E("Using wrongly type(%d) to mask clear!", mask);
}

uint8_t bt_system_reset(void)
{
    if (bt_reset_mask != 0)
    {
        LOG_E("reset on-going");
        return 1;
    }
    bt_ns_test_mode_ctrl_cmd_t dut_ctrl;
#ifdef BT_FINSH
    bt_reset_mask |= BT_RESET_MASK_BT;
    extern void bts2_task_stop(void);
    bts2_task_stop();
#endif
    dut_ctrl.op = BT_TEST_OP_EXIT_TEST;
    bt_reset_mask |= BT_RESET_MASK_BLE;
    bt_enter_no_signal_dut_mode(&dut_ctrl);
    return 0;
}

#endif // !SOC_SF32LB55X

#ifndef BSP_USING_PC_SIMULATOR
__WEAK char *bt_lib_get_ver(void)
{
    return "bt.0.0.0";
}
#endif

#ifdef SOC_SF32LB52X

void bt_sleep_control(uint8_t is_enable)
{
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    HAL_Delay(5);

    if (is_enable)
        hwp_lpsys_aon->RESERVE0 = 0;
    else
        hwp_lpsys_aon->RESERVE0 = 1;

    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
}

#define FACTORY_CFG_ID_TMXCAPINEFUSE  23
extern uint8_t rt_flash_config_read(uint8_t id, uint8_t *data, uint8_t size);
int8_t bt_rf_is_golden_unit()
{
    int res;
    uint32_t value = 0;
    res = rt_flash_config_read(FACTORY_CFG_ID_TMXCAPINEFUSE, (uint8_t *)&value, sizeof(value));

    //LOG_I("bt_rf_is_golden_unit\n");

    if ((res > 0) && (value == 0xa5a55a5a))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

#endif // SOC_SF32LB55X



uint8_t ble_enter_dut_mode(ble_dut_mode_t *dut_ctrl)
{

    ble_dut_mode_t *ctrl = sifli_msg_alloc(GAPM_LE_TEST_MODE_CTRL_CMD,
                                           TASK_ID_GAPM, sifli_get_stack_id(),
                                           sizeof(ble_dut_mode_t));

    memcpy(ctrl, dut_ctrl, sizeof(ble_dut_mode_t));

    sifli_msg_send((void const *)ctrl);
    //sifli_msg_free((void *)ctrl);
    return HL_ERR_NO_ERROR;

}


#ifndef SOC_SF32LB55X
static uint8_t g_non_signaling_test = 0;

uint8_t app_bt_get_non_signaling_test_status(void)
{
    return g_non_signaling_test;
}

uint8_t bt_enter_no_signal_dut_mode(bt_ns_test_mode_ctrl_cmd_t *dut_ctrl)
{
    if (dut_ctrl->op == BT_TEST_OP_ENTER_TEST)
    {
        g_non_signaling_test = 1;
    }
    else if (dut_ctrl->op == BT_TEST_OP_EXIT_TEST)
    {
        g_non_signaling_test = 0;
    }
    bt_ns_test_mode_ctrl_cmd_t *ctrl = sifli_msg_alloc(COMM_BT_TEST_MODE_CTRL_CMD,
                                       TASK_ID_COMMON, sifli_get_stack_id(),
                                       sizeof(bt_ns_test_mode_ctrl_cmd_t));

    memcpy(ctrl, dut_ctrl, sizeof(bt_ns_test_mode_ctrl_cmd_t));

    sifli_msg_send((void const *)ctrl);
    //sifli_msg_free((void *)ctrl);
    return HL_ERR_NO_ERROR;
}
#endif // SOC_SF32LB55X




int ble_event_start(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    return 0;
}
#ifndef BSP_USING_PC_SIMULATOR
    SIFLI_BLE_REGISTER(ble_event_start, NULL, "0");
#else
    SIFLI_BLE_REGISTER(ble_event_start, NULL, ".ble_sifli_reg$0");
#endif

int ble_event_end(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    return 0;
}
#ifndef BSP_USING_PC_SIMULATOR
    SIFLI_BLE_REGISTER(ble_event_end, NULL, "3.end");
#else
    SIFLI_BLE_REGISTER(ble_event_end, NULL, ".ble_sifli_reg$3.end");

#endif



void ble_event_publish(uint16_t event_id, void *data, uint16_t len)
{
#ifndef BSP_USING_PC_SIMULATOR
    const ble_register_callback_t *fn_ptr;
    for (fn_ptr = &__ble_callback_ble_event_start; fn_ptr < &__ble_callback_ble_event_end; fn_ptr++)
    {
        fn_ptr->function(event_id, data, len, fn_ptr->context_p);
    }
#else
    const ble_register_callback_t *fn_ptr = &__ble_callback_ble_event_start;
    while (fn_ptr < &__ble_callback_ble_event_end)
    {
        if (fn_ptr->function)
        {
            fn_ptr->function(event_id, data, len, fn_ptr->context_p);
            fn_ptr++;
        }
        else
            fn_ptr = (const ble_register_callback_t *)((uint8_t *)fn_ptr + 4);
    }

#endif
}



int bt_event_start(uint16_t type, uint16_t event_id, uint8_t *data, uint32_t context)
{
    return 0;
}
#ifndef BSP_USING_PC_SIMULATOR
    SIFLI_BT_REGISTER(bt_event_start, NULL, "0");
#else
    SIFLI_BT_REGISTER(bt_event_start, NULL, ".bt_sifli_reg$0");
#endif

int bt_event_end(uint16_t type, uint16_t event_id, uint8_t *data, uint32_t context)
{
    return 0;
}
#ifndef BSP_USING_PC_SIMULATOR
    SIFLI_BT_REGISTER(bt_event_end, NULL, "3.end");
#else
    SIFLI_BT_REGISTER(bt_event_end, NULL, ".bt_sifli_reg$3.end");

#endif



void bt_log_output(uint8_t tag, char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

#if defined(RT_USING_ULOG)
    rt_uint32_t lvl = LOG_LVL_DBG;

    switch (tag)
    {
    case BT_TAG_D:
    {
        lvl = LOG_LVL_DBG;
        break;
    }
    case BT_TAG_I:
    {
        lvl = LOG_LVL_INFO;
        break;
    }
    case BT_TAG_W:
    {
        lvl = LOG_LVL_WARNING;
        break;
    }
    case BT_TAG_E:
    {
        lvl = LOG_LVL_ERROR;
        break;
    }
    case BT_TAG_V:
#if 0
    {
        lvl = LOG_LVL_DBG;
        break;
    }
#else
    va_end(args);
    return;
#endif
    }


    ulog_voutput(lvl, "bt_common", RT_TRUE, fmt, args);
#endif
    va_end(args);
}

void bt_event_publish(uint16_t type, uint16_t event_id, void *data)
{
#ifndef BSP_USING_PC_SIMULATOR
    const bt_register_callback_t *fn_ptr;
    for (fn_ptr = &__bt_callback_bt_event_start; fn_ptr < &__bt_callback_bt_event_end; fn_ptr++)
    {
        fn_ptr->function(type, event_id, data, fn_ptr->context_p);
    }
#else
    const bt_register_callback_t *fn_ptr = &__bt_callback_bt_event_start;
    while (fn_ptr < &__bt_callback_bt_event_end)
    {
        if (fn_ptr->function)
        {
            fn_ptr->function(type, event_id, data, fn_ptr->context_p);
            fn_ptr++;
        }
        else
            fn_ptr = (bt_register_callback_t *)((uint8_t *)fn_ptr + 4);
    }

#endif
}

#ifndef SOC_SF32LB55X

#ifdef BT_RF_TEST
typedef struct
{
    uint8_t  rf_mode;// 0: null  1: enter dut  2: exit dut
    uint8_t  rf_stat;// 0: end 1:bt tx test  2:bt rx test  3:ble tx test 4:ble rx test
    ble_dut_mode_t  ble_test;
    bt_ns_test_mode_ctrl_cmd_t  bt_test;
} btdm_rftest_t;
static btdm_rftest_t  g_btdm_rftest;

static void bt_rftest(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "enter") == 0)
        {
            g_btdm_rftest.bt_test.op = BT_TEST_OP_ENTER_TEST;
            bt_enter_no_signal_dut_mode(&(g_btdm_rftest.bt_test));
            g_btdm_rftest.rf_mode = 1;
            g_btdm_rftest.rf_stat = 0;
        }
        else if (strcmp(argv[1], "exit") == 0)
        {
            g_btdm_rftest.bt_test.op = BT_TEST_OP_EXIT_TEST;
            bt_enter_no_signal_dut_mode(&(g_btdm_rftest.bt_test));
            g_btdm_rftest.rf_mode = 2;
        }
        else if (strcmp(argv[1], "bttx") == 0)
        {
            if ((g_btdm_rftest.rf_mode == 1) && (g_btdm_rftest.rf_stat == 0))
            {
                uint8_t channel = 8;
                uint8_t pkt_payload = 0;
                uint8_t pkt_type = 5;
                uint8_t pwr_lvl = 10;
                uint16_t pkt_len = 339;

                g_btdm_rftest.bt_test.op = BT_TEST_OP_TX_TEST;
                g_btdm_rftest.bt_test.para.tx_para.channel = channel;
                g_btdm_rftest.bt_test.para.tx_para.pkt_len = pkt_len;
                g_btdm_rftest.bt_test.para.tx_para.pkt_payload = pkt_payload;
                g_btdm_rftest.bt_test.para.tx_para.pkt_type = pkt_type;
                g_btdm_rftest.bt_test.para.tx_para.pwr_lvl = pwr_lvl;
                bt_enter_no_signal_dut_mode(&(g_btdm_rftest.bt_test));
                g_btdm_rftest.rf_stat = 1;
            }
            else if (g_btdm_rftest.rf_mode != 1)
            {
                LOG_I("please enter test mode first!");
            }
            else
            {
                LOG_I("please stop test first!");
            }
        }
#ifndef BSP_USING_PC_SIMULATOR
        else if (strcmp(argv[1], "bt_newrx") == 0)
        {
            if ((g_btdm_rftest.rf_mode == 1) && (g_btdm_rftest.rf_stat == 0))
            {
                uint8_t channel = 8;
                uint8_t pkt_payload = 0;
                uint8_t pkt_type = 5;
                uint16_t pkt_len = 339;
                int8_t rett;
                uint32_t delay;
                bt_ns_test_new_rx_para_t  rx_para_new;
                bt_ns_test_new_rx_rslt_t  rx_rslt;

                rx_para_new.channel = channel;
                rx_para_new.pkt_len = pkt_len;
                rx_para_new.pkt_payload = pkt_payload;
                rx_para_new.pkt_type = pkt_type;

                {
                    int8_t bt_ns_rx_start(bt_ns_test_new_rx_para_t *rxpara, bt_ns_test_new_rx_rslt_t *rst, uint32_t delay);

                    delay = 2000;//ms   after delay time, stop rx,and return result
                    rett = bt_ns_rx_start(&rx_para_new, &rx_rslt, delay);

                    LOG_I("bt ns rx: %d, %d, %d, %d, %d\n", rx_rslt.err_bit_num, rx_rslt.total_bit_num,
                          rx_rslt.err_pkt_num, rx_rslt.total_pkt_num, rx_rslt.rssi);

                }

            }
            else if (g_btdm_rftest.rf_mode != 1)
            {
                LOG_I("please enter test mode first!");
            }
            else
            {
                LOG_I("please stop test first!");
            }
        }
        else if (strcmp(argv[1], "ble_newrx") == 0)
        {
            if ((g_btdm_rftest.rf_mode == 1) && (g_btdm_rftest.rf_stat == 0))
            {
                int8_t rett;
                uint32_t delay;
                ble_ns_test_rx_t  rx_para_new;
                ble_ns_test_rx_rslt_t  rx_rslt;

                rx_para_new.channel = 0;
                rx_para_new.phy = 1;//1M PHY
                rx_para_new.modulation_idx = 0;

                {
                    int8_t ble_ns_rx_start(ble_ns_test_rx_t *rxpara, ble_ns_test_rx_rslt_t *rst, uint32_t delay);

                    delay = 2000;//ms   after delay time, stop rx,and return result
                    rett = ble_ns_rx_start(&rx_para_new, &rx_rslt, delay);

                    LOG_I("ble ns rx: %d, %d\n", rx_rslt.total_pkt_num, rx_rslt.rssi);

                }

            }
            else if (g_btdm_rftest.rf_mode != 1)
            {
                LOG_I("please enter test mode first!");
            }
            else
            {
                LOG_I("please stop test first!");
            }
        }
#endif
        else if (strcmp(argv[1], "btrx") == 0)
        {
            if ((g_btdm_rftest.rf_mode == 1) && (g_btdm_rftest.rf_stat == 0))
            {
                uint8_t channel = 0;
                uint8_t pkt_type = 0;

                g_btdm_rftest.bt_test.op = BT_TEST_OP_RX_TEST;
                g_btdm_rftest.bt_test.para.rx_para.channel = channel;
                g_btdm_rftest.bt_test.para.rx_para.pkt_type = pkt_type;
                bt_enter_no_signal_dut_mode(&(g_btdm_rftest.bt_test));
                g_btdm_rftest.rf_stat = 2;
            }
            else if (g_btdm_rftest.rf_mode != 1)
            {
                LOG_I("please enter test mode first!");
            }
            else
            {
                LOG_I("please stop test first!");
            }
        }
        else if (strcmp(argv[1], "btstop") == 0)
        {
            if (g_btdm_rftest.rf_mode == 1)
            {
                if ((g_btdm_rftest.rf_stat == 1) || (g_btdm_rftest.rf_stat == 2))
                {
                    g_btdm_rftest.bt_test.op = BT_TEST_OP_STOP_TEST;
                    bt_enter_no_signal_dut_mode(&(g_btdm_rftest.bt_test));
                    g_btdm_rftest.rf_stat = 0;
                }
                else
                {
                    LOG_I("not start bt test!");
                }
            }
            else
            {
                LOG_I("please enter test mode first!");
            }
        }
        else if (strcmp(argv[1], "bletx") == 0)
        {
            if ((g_btdm_rftest.rf_mode == 1) && (g_btdm_rftest.rf_stat == 0))
            {
                int8_t  tx_pwr = 10;
                {
                    extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
                    blebredr_rf_power_set(0, tx_pwr);
                }
                g_btdm_rftest.ble_test.operation = GAPM_LE_TEST_TX_START;
                g_btdm_rftest.ble_test.channel = 0;
                g_btdm_rftest.ble_test.tx_data_length = 37;
                g_btdm_rftest.ble_test.tx_pkt_payload = 0;
                g_btdm_rftest.ble_test.phy = 1;
                ble_enter_dut_mode(&(g_btdm_rftest.ble_test));
                g_btdm_rftest.rf_stat = 3;
            }
            else if (g_btdm_rftest.rf_mode != 1)
            {
                LOG_I("please enter test mode first!");
            }
            else
            {
                LOG_I("please stop test first!");
            }
        }
        else if (strcmp(argv[1], "blerx") == 0)
        {
            if ((g_btdm_rftest.rf_mode == 1) && (g_btdm_rftest.rf_stat == 0))
            {
                g_btdm_rftest.ble_test.operation = GAPM_LE_TEST_RX_START;
                g_btdm_rftest.ble_test.channel = 0;
                g_btdm_rftest.ble_test.modulation_idx = 0;
                g_btdm_rftest.ble_test.phy = 1;
                ble_enter_dut_mode(&(g_btdm_rftest.ble_test));
                g_btdm_rftest.rf_stat = 4;
            }
            else if (g_btdm_rftest.rf_mode != 1)
            {
                LOG_I("please enter test mode first!");
            }
            else
            {
                LOG_I("please stop test first!");
            }
        }
        else if (strcmp(argv[1], "blestop") == 0)
        {
            if (g_btdm_rftest.rf_mode == 1)
            {
                if ((g_btdm_rftest.rf_stat == 3) || (g_btdm_rftest.rf_stat == 4))
                {
                    g_btdm_rftest.ble_test.operation = GAPM_LE_TEST_STOP;
                    ble_enter_dut_mode(&(g_btdm_rftest.ble_test));
                    g_btdm_rftest.rf_stat = 0;
                }
                else
                {
                    LOG_I("not start ble test!");
                }
            }
            else
            {
                LOG_I("please enter test mode first!");
            }
        }
    }
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(bt_rftest, BT / BLE no signal test);
#endif // RT_USING_FINSH

#endif //BT_RF_TEST
#endif // !SOC_SF32LB55X

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
