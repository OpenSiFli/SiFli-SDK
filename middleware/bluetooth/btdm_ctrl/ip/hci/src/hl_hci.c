/**
  ******************************************************************************
  * @file   hl_hci.c
  * @author Sifli software development team
  * @brief Host HCI interface.
  * @{
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
#include "rwip_config.h"       // SW configuration

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_endian.h"       // common endianess definition
#include "co_list.h"         // list definition

#include "hci.h"             // hci definition

#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_mem.h"          // kernel memory definition
#include "ke_timer.h"        // kernel timer definition
#include "rwip.h"            // rw bt core interrupt
#include "dbg.h"             // debug

#if (H4TL_SUPPORT)
    #include "h4tl.h"            // H4TL definitions
    #include "hci_int.h"
#endif // H4TL_SUPPORT

#if BLE_EMB_PRESENT
    #include "ble_util_buf.h"
    #include "em_map.h"
    #include "reg_access.h"
    #if (BLE_ISOOHCI)
        #include "isoohci.h"
    #endif //(BLE_ISOOHCI)
#endif //BLE_EMB_PRESENT

#if (HOST_PRESENT)
    //#if (!EMB_PRESENT)
    #include "gap.h"
    #include "gapc.h"
    //#endif // (!EMB_PRESENT)
    #include "hl_hci.h"
    #include "rwble_hl.h"

    #if CFG_BT_HOST
        //#include "bts2_global_int.h"
        #include "hci_drv.h"
        #include "bts2_mem.h"
    #endif
#endif // (HOST_PRESENT)

#define LE_EVT_CODE(le_code) HCI_##le_code##_EVT_SUBCODE
#define EVT_CODE(code) HCI_##code##_EVT_CODE



#define BT_ACL_MIN (0x80)
#define BT_ACL_MAX (BT_ACL_MIN + CFG_CON_ACL - 1)

#define BLE_ACL_MIN (0)
#define BLE_ACL_MAX (BLE_ACL_MIN + CFG_ACT - 1)

#define BT_SCO_MIN (0x1)
#define BT_SCO_MAX (BT_SCO_MIN + CFG_CON_SCO - 1)


#if (HOST_PRESENT)

#define MAX_COMMON_CMD 6
static uint16_t bt_host_cmd[MAX_COMMON_CMD];

static int get_common_bt(uint16_t opcode)
{
    int i;

    for (i = 0; i < MAX_COMMON_CMD; i++)
    {
        if (bt_host_cmd[i] == opcode)
        {
            bt_host_cmd[i] = 0;             // Next back to default to BLE host
            break;
        }
    }
    if (i == MAX_COMMON_CMD)
        i = -1;
    return i;
}

static ke_task_id_t hl_hci_get_dest_task(uint8_t dest)
{
    ke_task_id_t task = TASK_NONE;
    switch (dest)
    {
    case BLE_MNG:
    case MNG:
    case DBG:
    {
        task = TASK_GAPM;
    }
    break;
    case BLE_CTRL:
    case CTRL:
        task = TASK_GAPC;
        break;
    case COMMON:
        task = TASK_COMMON;
        break;
    default:
        break;
    }
    return task;
}

uint16_t em_addr_get(uint8_t *p_em_buf)
{
    return 0;
}

uint8_t hl_hci_get_host_tl_dest_from_conhdl(uint16_t conhdl)
{
    uint8_t dest = 0;
    uint16_t sco_hdl = (conhdl >> 8);
    // rt_kprintf("conhdl = %d,BT_ACL_MIN = %d,BT_ACL_MAX = %d\n",conhdl,BT_ACL_MIN,BT_ACL_MAX);
    // rt_kprintf("BLE_ACL_MIN = %d,BLE_ACL_MAX = %d,BT_SCO_MIN = %d,BT_SCO_MAX = %d\n",BLE_ACL_MIN,BLE_ACL_MAX,BT_SCO_MIN,BT_SCO_MAX);
    if (conhdl >= BT_ACL_MIN && conhdl <= BT_ACL_MAX)
        dest = HOST_TL_PK;
    else if (conhdl >= BLE_ACL_MIN && conhdl <= BLE_ACL_MAX)
        dest = HOST_TL_UPK;
    else if (sco_hdl >= BT_SCO_MIN && sco_hdl <= BT_SCO_MAX)
        dest = HOST_TL_PK;
    return dest;
}

uint8_t hl_hci_cmd_evt_get_host_tl_dest(uint16_t opcode)
{
    uint8_t dest = HOST_TL_PK;
    uint16_t ogf = HCI_OP2OGF(opcode);

    if (ogf == 0x08 || get_common_bt(opcode) < 0)
        dest = HOST_TL_UPK;

    //rt_kprintf("Opcode %x to %s\n", opcode, (dest == HOST_TL_UPK) ? "BLE" : "BT");
    return dest;
}

void hl_hci_send_cmd_evt_to_host(uint8_t dest, uint8_t evt_type, uint16_t opcode, uint16_t evt_len, void *param)
{
    ke_task_id_t dest_task = hl_hci_get_dest_task(dest);
    uint8_t conn_idx = 0;
    uint8_t *evt_param = NULL;
    if (dest_task == TASK_NONE)
    {
        if (evt_len == HL_HCI_NOT_TL)
            ke_free(param);
        return;
    }

    RW_ASSERT(param);
    RW_ASSERT(evt_len != 0);

    switch (evt_type)
    {
    case HL_HCI_CMD_CMP_EVT:
    case HL_HCI_CMD_STAT_EVT:
    {
        ke_msg_id_t id;
        if (evt_type == HL_HCI_CMD_CMP_EVT)
            id = HCI_CMD_CMP_EVENT;
        else if (evt_type == HL_HCI_CMD_STAT_EVT)
            id = HCI_CMD_STAT_EVENT;
        else
            RW_ASSERT(0);

        if (dest_task == TASK_GAPC)
        {
            if (evt_type == HL_HCI_CMD_CMP_EVT && evt_len >= 4)
            {
                // Conn hdl should be aligned after unpack
                uint16_t conn_hdl = co_btohs(co_read16p((uint16_t *)param + 1));
                conn_idx = gapc_get_conidx(conn_hdl);
            }
            else if (evt_type == HL_HCI_CMD_STAT_EVT)
            {
                if (evt_len == 2)
                {
                    // Just pass through the conn_idx
                    conn_idx = *((uint8_t *)param + 1);
                    evt_len -= 1;
                }
            }
            if (conn_idx == 0xFF) // Means GAPC could not handle it. just discard
            {
                if (evt_len == HL_HCI_NOT_TL)
                    ke_free(param);
                return;
            }
            dest_task = KE_BUILD_ID(dest_task, conn_idx);
        }
        if (evt_len != HL_HCI_NOT_TL)
        {
            evt_param = (uint8_t *)ke_msg_alloc(id, dest_task, opcode, evt_len);
            RW_ASSERT(evt_param);
            memcpy(evt_param, param, evt_len);
        }
        else
        {
            struct ke_msg *p_msg = ke_param2msg(param);
            p_msg->dest_id = dest_task;
            evt_param = (uint8_t *)param;
        }
    }
    break;
    default:
        break;
    }
    if (evt_param)
        ke_msg_send((void const *)evt_param);
    else if (evt_len == HL_HCI_NOT_TL)
        ke_free(param);
}

#if (BT_HOST_PRESENT)
void hl_hci_pk_send_to_host(uint8_t type, uint8_t *p_buf, uint16_t len, hci_done_callback cbk)
{
    BTS2S_RECV_DATA data;

    data.buflen = len;
    data.buf = bt_mem_alloc(len);
    memcpy((void *)data.buf, p_buf, len);
    if (type == HCI_ACL_MSG_TYPE)
    {
        extern void hcit_hdl_acl_data(BTS2S_RECV_DATA * src);
        hcit_hdl_acl_data(&data);
    }
#ifdef CFG_SCO
    else if (type == HCI_SYNC_MSG_TYPE)
    {
        extern void hcit_hdl_sco_data(BTS2S_RECV_DATA * src);
        hcit_hdl_sco_data(&data);
    }
#endif // CFG_SCO
    else
    {
        extern void hcit_hdl_ev(BTS2S_RECV_DATA * src);
        hcit_hdl_ev(&data);
    }
    bt_mem_free(data.buf);
}
#endif

void hl_hci_send_evt_to_host(uint8_t evt_type, uint16_t opcode, uint16_t conn_hdl, uint16_t evt_len, void *param)
{
    uint8_t *evt_param = NULL;
    ke_task_id_t dest_task = TASK_NONE;
    uint16_t id;

    if (evt_type == HL_HCI_LE_EVT)
    {
        id = HCI_LE_EVENT;
        switch (opcode)
        {
        case LE_EVT_CODE(LE_ENH_CON_CMP):
        {
            if (evt_len != HL_HCI_NOT_TL)
            {
                extern int hci_le_enh_con_cmp_evt_handler(uint16_t opcode, struct hci_le_enh_con_cmp_evt const * event);
                hci_le_enh_con_cmp_evt_handler(opcode, param);
                break;
            }
        }
        case LE_EVT_CODE(LE_CON_CMP):
        case LE_EVT_CODE(LE_ADV_REPORT):
#if (SECURE_CONNECTIONS==1)
        case LE_EVT_CODE(LE_RD_LOC_P256_PUB_KEY_CMP):
        case LE_EVT_CODE(LE_GEN_DHKEY_CMP):
#endif // (SECURE_CONNECTIONS==1)
        case LE_EVT_CODE(LE_DIR_ADV_REP):
        case LE_EVT_CODE(LE_EXT_ADV_REPORT):
        case LE_EVT_CODE(LE_PER_ADV_SYNC_EST):
        case LE_EVT_CODE(LE_PER_ADV_REPORT):
        case LE_EVT_CODE(LE_PER_ADV_SYNC_LOST):
        case LE_EVT_CODE(LE_SCAN_TIMEOUT):
        case LE_EVT_CODE(LE_ADV_SET_TERMINATED):
        case LE_EVT_CODE(LE_SCAN_REQ_RCVD):
        {
            dest_task = TASK_GAPM;
        }
        break;
        case LE_EVT_CODE(LE_CON_UPDATE_CMP):
        case LE_EVT_CODE(LE_RD_REM_FEATS_CMP):
        case LE_EVT_CODE(LE_LTK_REQUEST):
        case LE_EVT_CODE(LE_REM_CON_PARAM_REQ):
        case LE_EVT_CODE(LE_DATA_LEN_CHG):
        case LE_EVT_CODE(LE_PHY_UPD_CMP):
        case LE_EVT_CODE(LE_CH_SEL_ALGO):
        {
#if (HCI_BLE_CON_SUPPORT)
            uint8_t idx = gapc_get_conidx(conn_hdl);
            dest_task = KE_BUILD_ID(TASK_GAPC, idx);
#else
            dest_task = TASK_GAPC;
#endif
        }
        break;
        default:
            break;
        }
    }
    else if (evt_type == HL_HCI_EVT)
    {
        id = HCI_EVENT;
        uint8_t idx = gapc_get_conidx(conn_hdl);
        uint8_t conn_valid = 0;
        if ((conn_hdl != GAP_INVALID_CONHDL) &&
                (idx != GAP_INVALID_CONIDX))
            conn_valid = 1;
        switch (opcode)
        {
        case EVT_CODE(NB_CMP_PKTS):
        {
            dest_task = conn_valid ? KE_BUILD_ID(TASK_L2CC, idx) : TASK_NONE;
        }
        break;
        case EVT_CODE(AUTH_PAYL_TO_EXP):
        case EVT_CODE(ENC_KEY_REFRESH_CMP):
        case EVT_CODE(ENC_CHG):
        case EVT_CODE(RD_REM_VER_INFO_CMP):
        case EVT_CODE(DISC_CMP):
        {
            dest_task = conn_valid ? KE_BUILD_ID(TASK_GAPC, idx) : TASK_NONE;
        }
        break;
        case EVT_CODE(HW_ERR):
        {
            dest_task = TASK_GAPM;
        }
        default:
            break;
        }
    }
    else if (evt_type == HL_HCI_DBG_EVT)
    {
        id = HCI_DBG_EVT;
        dest_task = TASK_GAPM;
    }

    if (dest_task != TASK_NONE)
    {
        if (evt_len != HL_HCI_NOT_TL)
        {
            evt_param = (uint8_t *)ke_msg_alloc(id, dest_task, opcode, evt_len);
            RW_ASSERT(evt_param);
            memcpy(evt_param, param, evt_len);
            ke_msg_send(evt_param);
        }
        else
        {
            struct ke_msg *p_msg = ke_param2msg(param);
            p_msg->dest_id = dest_task;
            ke_msg_send(param);
        }
    }
    else if (evt_len == HL_HCI_NOT_TL)
        ke_free(param);

}


void hl_hci_send_data_to_host(uint8_t evt_type, void *param, uint8_t is_tl)
{
    struct hci_acl_data *data = (struct hci_acl_data *)param;
    uint16_t conhdl = GETF(data->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL);
    uint8_t idx = gapc_get_conidx(conhdl);
    if ((conhdl != GAP_INVALID_CONHDL) &&
            (idx != GAP_INVALID_CONIDX))
    {
        if (is_tl)
        {
            struct hci_acl_data *data_rx = KE_MSG_ALLOC(HCI_ACL_DATA, KE_BUILD_ID(TASK_L2CC, idx), 0, hci_acl_data);
            RW_ASSERT(data_rx);
            data_rx->conhdl_pb_bc_flag = data->conhdl_pb_bc_flag;
            data_rx->length = data->length;
            data_rx->buf_ptr = (uint32_t)data;
            // Move data buffer to header that received module could free it.
            memcpy(data, (void *)data->buf_ptr, data->length);
            ke_msg_send(data_rx);
        }
        else
        {
            struct ke_msg *p_msg = ke_param2msg(param);
            p_msg->dest_id = KE_BUILD_ID(TASK_L2CC, idx);
            ke_msg_send(param);
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, conhdl, idx);
    }
}


#ifndef BSP_USING_PC_SIMULATOR
__WEAK void bts2_main(void)
{
    //Do nothing
}
#else
void bts2_main_weak(void)
{
    //Do nothing
}

#pragma comment(linker, "/alternatename:_bts2_main=_bts2_main_weak")
#endif



#include "bt_ver.h"
__USED char *bt_lib_get_ver(void)
{
    return BT_LIB_VER;
}



void hl_initialize(bool is_reboot)
{
    rwble_hl_init();


#if CFG_BT_HOST
    extern void bts2_main(void);
    //if (!is_reboot)
    bts2_main();
#endif

}

int hl_hci_mark_host_bt(uint16_t opcode)
{
    int i, r = -1;

    for (i = 0; i < MAX_COMMON_CMD; i++)
    {
        if (bt_host_cmd[i] == opcode)
        {
            r = i;
            break;
        }
        if (r < 0 && bt_host_cmd[i] == 0)
            r = i;
    }
    if (r >= 0)
    {
        bt_host_cmd[r] = opcode;
        r = 1;
    }
    else
        r = 0;
    return r;
}


uint8_t hl_hci_cmd_parse(uint16_t opcode, uint8_t *out_buff, uint8_t *in_buff, uint16_t out_size, uint16_t in_size)
{
    uint8_t ret = 0;
    const hci_cmd_desc_t *p_cmd_desc = hci_msg_cmd_desc_get(opcode);
    uint8_t *p_out = out_buff + 3;
    uint8_t status = hci_msg_cmd_pkupk(p_cmd_desc, p_out, in_buff, &out_size, in_size);
    if (status == 0)
    {
        co_write16p(out_buff, co_htobs(opcode));
        out_buff += 2;

        //pack command parameter length
        *out_buff++ = out_size;
        ret = 1;
    }
    return ret;
}

uint8_t hl_hci_evt_parse(uint8_t code, uint8_t length, uint8_t *p_payload, uint8_t *p_evt, uint16_t max_p_len)
{
    uint8_t status = CO_UTIL_PACK_OK;
    uint16_t opcode = 0;
    uint8_t evt_type = HL_HCI_UNDEF;
    const hci_cmd_desc_t *p_cmd_desc = NULL;
    const hci_evt_desc_t *p_evt_desc = NULL;
    uint8_t hl_tl_dest = HOST_NONE;
    bool release = true;
    // retrieve pointer to allocated buffer

    ASSERT_ERR(p_payload != NULL);


    switch (code)
    {
    case HCI_CMD_CMP_EVT_CODE:
    {
        ASSERT_WARN((length >= HCI_CCEVT_HDR_PARLEN), length, 0);
        if (length < HCI_CCEVT_HDR_PARLEN) break;

        // Retrieve opcode from parameters, expected at position 1 in payload (after nb cmp pkts)
        opcode = co_btohs(co_read16p(p_payload + 1));
        // Look for the command descriptor
        p_cmd_desc = hci_msg_cmd_desc_get(opcode);
        evt_type = HL_HCI_CMD_CMP_EVT;
        ASSERT_WARN((p_cmd_desc != NULL), opcode, 0);
    }
    break;
    case HCI_CMD_STATUS_EVT_CODE:
    {
        ASSERT_WARN((length == HCI_CSEVT_PARLEN), length, 0);
        if (length != HCI_CSEVT_PARLEN) break;
        // Retrieve opcode from parameters, expected at position 2 in payload (after status and nb cmp pkts)
        opcode = co_btohs(co_read16p(p_payload + 2));
        // Look for the command descriptor
        p_cmd_desc = hci_msg_cmd_desc_get(opcode);
        evt_type = HL_HCI_CMD_STAT_EVT;
        ASSERT_WARN((p_cmd_desc != NULL), opcode, 0);
    }
    break;
    default:
    {
        p_evt_desc = hci_msg_evt_desc_get(code);
        evt_type = HL_HCI_EVT;
        ASSERT_WARN((p_evt_desc != NULL), code, evt_type);
    }
    break;
    }

    if (p_cmd_desc != NULL)
    {
        hl_tl_dest = hl_hci_cmd_evt_get_host_tl_dest(opcode);

        {
            // At least, should have status code
            uint16_t evt_len = 1;
            uint8_t dest = hci_msg_cmd_ll_dest_get(p_cmd_desc);
            if (evt_type == HL_HCI_CMD_CMP_EVT)
            {
                p_evt[0] = *p_payload; // nb of comp
                co_write16(p_evt + 2, opcode); // opcode
                length    -= HCI_CCEVT_HDR_PARLEN;
                p_payload += HCI_CCEVT_HDR_PARLEN;
                // Check if there are parameters to unpack
                if (length > 0)
                {
                    uint16_t unpk_length = max_p_len;
                    // Unpack
                    status = hci_msg_cmd_cmp_pkupk(p_cmd_desc, (uint8_t *)&p_evt[4], p_payload, &unpk_length, length);
                    evt_len = unpk_length;
                }
            }
            else // HL_HCI_CMD_STAT_EVT
            {
                p_evt[0] = *p_payload; // status code only
                p_evt[1] = *p_payload + 1; // nb of comp
                co_write16(p_evt + 2, opcode); // opcode

            }
        }
    }
    else if (p_evt_desc != NULL)
    {
        hl_tl_dest = hci_msg_evt_get_hl_tl_dest(p_evt_desc);
        uint16_t conhdl = GAP_INVALID_CONHDL;
        if (hl_tl_dest == HOST_UNDEF)
        {
            {
                // Retrieve host destination according to connection handle present in event
                conhdl = hci_msg_evt_look_for_conhdl(p_evt_desc, p_payload);
                hl_tl_dest = hl_hci_get_host_tl_dest_from_conhdl(conhdl);
            }
        }

        if (hl_tl_dest == HOST_TL_PK)
        {
            uint16_t evt_len = 0;
            // Check if there are parameters to unpack
            if (length > 0)
            {
                uint16_t unpk_length = max_p_len;
                status =  hci_msg_evt_pkupk(p_evt_desc, p_evt, p_payload, &unpk_length, length);
                ASSERT_INFO((status == CO_UTIL_PACK_OK), status, code);
                evt_len = unpk_length;
            }
        }
        else
            status = CO_UTIL_PACK_WRONG_FORMAT;
    }

    return (status);
}


#endif
