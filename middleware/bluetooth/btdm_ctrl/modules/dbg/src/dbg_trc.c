/**
 ****************************************************************************************
 *
 * @file dbg_tcr.c
 *
 * @brief Tracer module source file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup TRACER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"  // platform definitions
#include "rwip.h"

#if (TRACER_PRESENT)
#include <stdio.h>
#include <string.h>
#include "ke_event.h"       // kernel event definition
#include "co_utils.h"
#include "arch.h"            // architectural definition

#include "dbg_trc_int.h"     // tracer internal definitions
#include "dbg_trc.h"        // tracer definitions

#if (TRC_PROG || (TRC_L2CAP && BLE_EMB_PRESENT))
    #include "reg_em_et.h"           // EM Exchange Tables
#endif /*(TRC_PROG || (TRC_L2CAP && BLE_EMB_PRESENT))*/

#if (TRC_ADV || TRC_CS_BLE)
    #include "reg_em_ble_cs.h"       // BLE EM Control Structure
#endif /*(TRC_ADV || TRC_CS_BLE)*/

#if (TRC_ADV || TRC_RX_DESC)
    #include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors
#endif /*(TRC_ADV || TRC_RX_DESC)*/

#if (TRC_ADV)
    #include "reg_blecore.h"         // BLE core registers
    #include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
    #include "reg_em_ble_ral.h"      // BLE RAL structures
#endif /*(TRC_ADV)*/

#if (TRC_CS_BT)
    #include "reg_em_bt_cs.h"        // BT EM Control Structure
#endif /*(TRC_CS_BT)*/


#if (TRC_ACL || TRC_LLCP)
    #include "lld.h"
    #include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#endif // (TRC_ACL || TRC_LLCP)
/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATION DEFINITION
 ****************************************************************************************
 */
/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// Tracer environment
struct dbg_trc_env_tag dbg_trc_env;

/*
 * LOCAL FUNCTION DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Set the compiled configuration word
 *
 * This function checks the compiled trace groups and sets the compiled configuration word
 *
 ****************************************************************************************
 */
__STATIC void dbg_trc_set_compiled_cw()
{
    dbg_trc_env.compiled_cw = 0;
    //set the enabled traces in the supported configuration word
#if TRC_KE_MSG
    SETB(dbg_trc_env.compiled_cw, TRC_KE_MSG, true);
#endif /*(TRC_KE_MSG)*/
#if TRC_KE_TMR
    SETB(dbg_trc_env.compiled_cw, TRC_KE_TMR, true);
#endif /*(TRC_KE_TMR)*/
#if TRC_KE_EVT
    SETB(dbg_trc_env.compiled_cw, TRC_KE_EVT, true);
#endif /*(TRC_KE_EVT)*/
#if TRC_KE_STATE
    SETB(dbg_trc_env.compiled_cw, TRC_KE_STATE, true);
#endif /*(TRC_KE_EVT)*/
#if TRC_MEM
    SETB(dbg_trc_env.compiled_cw, TRC_MEM, true);
#endif /*(TRC_MEM)*/
#if TRC_SLEEP
    SETB(dbg_trc_env.compiled_cw, TRC_SLEEP, true);
#endif /*(TRC_SLEEP)*/
#if TRC_SW_ASS
    SETB(dbg_trc_env.compiled_cw, TRC_SW_ASS, true);
#endif /*(TRC_SW_ASS)*/
#if TRC_PROG
    SETB(dbg_trc_env.compiled_cw, TRC_PROG, true);
#endif /*(TRC_PROG)*/
#if TRC_CS_BLE
    SETB(dbg_trc_env.compiled_cw, TRC_CS_BLE, true);
#endif /*(TRC_CS_BLE)*/
#if TRC_CS_BT
    SETB(dbg_trc_env.compiled_cw, TRC_CS_BT, true);
#endif /*(TRC_CS_BT)*/
#if TRC_RX_DESC
    SETB(dbg_trc_env.compiled_cw, TRC_RX_DESC, true);
#endif /*(TRC_RX_DESC)*/
#if TRC_LLCP
    SETB(dbg_trc_env.compiled_cw, TRC_LLCP, true);
#endif /*(TRC_RX_DESC)*/
#if TRC_LMP
    SETB(dbg_trc_env.compiled_cw, TRC_LMP, true);
#endif /*(TRC_LMP)*/
#if TRC_L2CAP
    SETB(dbg_trc_env.compiled_cw, TRC_L2CAP, true);
#endif /*(TRC_L2CAP)*/
#if TRC_ARB
    SETB(dbg_trc_env.compiled_cw, TRC_ARB, true);
#endif /*(TRC_ARB)*/
#if TRC_LLC_STATE_TRANS
    SETB(dbg_trc_env.compiled_cw, TRC_LLC_STATE_TRANS, true);
#endif /*(TRC_LLC_STATE_TRANS)*/
#if TRC_LC_STATE_TRANS
    SETB(dbg_trc_env.compiled_cw, TRC_LC_STATE_TRANS, true);
#endif /*(TRC_LC_STATE_TRANS)*/
#if TRC_HCI
    SETB(dbg_trc_env.compiled_cw, TRC_HCI, true);
#endif /*(TRC_HCI)*/
#if TRC_CUSTOM
    SETB(dbg_trc_env.compiled_cw, TRC_CUSTOM, true);
#endif /*(TRC_CUSTOM)*/
}

/**
 ****************************************************************************************
 * @brief Kernel event callback of KE_EVENT_TRC
 *
 ****************************************************************************************
 */
__STATIC void dbg_trc_ke_evt_cbk(void)
{
    //Clear the event field
    ke_event_clear(KE_EVENT_TRC);

    //trigger the transmission of traces
    dbg_trc_tx_trigger();
}
/*
 * EXPORTED FUNCTION DEFINITION
 ****************************************************************************************
 */
#if (TRC_KE_MSG)
void dbg_trc_req_ke_msg_send(struct ke_msg const *msg)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_KE_MSG)) >> TRC_GRP_P(TRC_KE_MSG))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct ke_msg)      - \
                      sizeof(struct co_list_hdr) + \
                      msg->param_len;

        trace = TRC_MEM_ALLOC(payload_len);;
        if (trace != NULL)
        {
            //update trace
            *trace++ = KE_MSG_SEND;
            memcpy(trace, &msg->id, payload_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
void dbg_trc_req_ke_msg_handled(ke_msg_id_t const id, ke_task_id_t const dest_id, ke_task_id_t const src_id, uint8_t const msg_status)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_KE_MSG)) >> TRC_GRP_P(TRC_KE_MSG))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(id)             +
                      sizeof(dest_id)        +
                      sizeof(src_id)         +
                      sizeof(msg_status);

        trace = TRC_MEM_ALLOC(payload_len);;
        if (trace != NULL)
        {
            //update trace
            *trace++ = KE_MSG_HANDLED;
            co_write16p(trace, id);
            trace += sizeof(id);
            co_write16p(trace, dest_id);
            trace += sizeof(dest_id);
            co_write16p(trace, src_id);
            trace += sizeof(src_id);
            *trace = msg_status;

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_KE_MSG)*/


#if (TRC_KE_TMR)
void dbg_trc_req_ke_tmr(uint8_t const tmr_trc_type, uint32_t const target_time, ke_task_id_t const task_id, ke_msg_id_t const msg_id)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_KE_TMR)) >> TRC_GRP_P(TRC_KE_TMR))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(target_time)     + \
                      sizeof(task_id)         + \
                      sizeof(msg_id);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = tmr_trc_type;
            co_write32p(trace, target_time);
            trace += sizeof(target_time);
            co_write16p(trace, task_id);
            trace += sizeof(task_id);
            co_write16p(trace, msg_id);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_KE_TMR)*/

#if (TRC_KE_EVT)
void dbg_trc_req_ke_evt(uint8_t const evt_trc_type, uint8_t const event_type)
{
    if ((event_type != KE_EVENT_TRC) && (event_type != KE_EVENT_DJOB_ISR) && (event_type != KE_EVENT_DJOB_LP))
    {
        if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_KE_EVT)) >> TRC_GRP_P(TRC_KE_EVT))
        {
            //write trace in memory
            uint8_t *trace;
            uint16_t payload_len;
            payload_len = sizeof(event_type);

            trace = TRC_MEM_ALLOC(payload_len);
            if (trace != NULL)
            {
                //update trace
                *trace++ = evt_trc_type;
                *trace   = event_type;

                // trigger the event
                ke_event_set(KE_EVENT_TRC);
            }
        }
    }
}
#endif /*(TRC_KE_EVT)*/

#if (TRC_KE_STATE)
void dbg_trc_req_ke_state(uint8_t const evt_trc_type, uint16_t const task_id, uint8_t const task_state)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_KE_STATE)) >> TRC_GRP_P(TRC_KE_STATE))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(task_id) + sizeof(task_state);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = evt_trc_type;
            co_write16p(trace, task_id);
            trace += 2;
            *trace   = task_state;

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif // TRC_KE_STATE

#if (TRC_MEM)
void dbg_trc_req_mem(uint8_t const mem_trc_type, uint8_t const heap_id, void const *blk_address, uint32_t const size)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_MEM)) >> TRC_GRP_P(TRC_MEM))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(heap_id)    + \
                      sizeof(blk_address)     + \
                      sizeof(size);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = mem_trc_type;
            *trace++ = heap_id;
            co_write32p(trace, (uint32_t) blk_address);
            trace += sizeof(blk_address);
            co_write32p(trace, size);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_MEM)*/

#if (TRC_SLEEP)
void dbg_trc_req_deep_sleep(uint8_t const sleep_trc_type)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_SLEEP)) >> TRC_GRP_P(TRC_SLEEP))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = 0;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace  = sleep_trc_type;

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_SLEEP)*/

#if (TRC_SW_ASS)
void dbg_trc_req_sw_ass(uint8_t const ass_type, char const *filename, uint16_t const code_line, uint32_t const param0, uint32_t const param1)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_SW_ASS)) >> TRC_GRP_P(TRC_SW_ASS))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        const uint8_t filename_len = 16;

        payload_len = filename_len       + \
                      sizeof(code_line)       + \
                      sizeof(param0)          + \
                      sizeof(param1);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ass_type;
            //TODO [AAL] change separator character in Linux
            char *file = strrchr(filename, '\\') + 1;
            strcpy((char *) trace, file);
            trace   += filename_len;
            co_write16p(trace, code_line);
            trace   += sizeof(code_line);
            co_write32p(trace, param0);
            trace   += sizeof(param0);
            co_write32p(trace, param1);

            GLOBAL_INT_DISABLE();
            //trigger the transmission of traces
            dbg_trc_tx_trigger();
            GLOBAL_INT_RESTORE();

            // Let time for the message transfer
            for (int i = 0; i < 0x0005FFFF; i++) {};
        }
    }
}
#endif /*(TRC_SW_ASS)*/

#if (TRC_PROG)
void dbg_trc_req_et_prog(uint8_t const elt_idx)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_PROG)) >> TRC_GRP_P(TRC_PROG))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(elt_idx)   + \
                      sizeof(int); // int represents REG_EM_ET_SIZE

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ET_PROG;
            *trace++ = elt_idx;
            co_write16p(trace, em_le_extab_get(elt_idx));
            trace += 2;
            co_write16p(trace, em_csptr_get(elt_idx));

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
void dbg_trc_req_evt_cnt(uint16_t const conhdl, uint16_t const value)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_PROG)) >> TRC_GRP_P(TRC_PROG))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(conhdl)   + \
                      sizeof(value);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = CONN_EVT_CNT;
            co_write16p(trace, conhdl);
            trace += sizeof(conhdl);
            co_write16p(trace, value);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
void dbg_trc_req_frm_cmp(uint32_t const timestamp, uint8_t const frm_id, uint32_t const frm_cbk)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_PROG)) >> TRC_GRP_P(TRC_PROG))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(timestamp)          + \
                      sizeof(frm_id)                  + \
                      sizeof(frm_cbk);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = FRM_CMP_BLE;
            co_write32p(trace, timestamp);
            trace   += sizeof(timestamp);
            *trace++ = frm_id;
            co_write32p(trace, frm_cbk);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_PROG)*/

#if (TRC_CS_BLE)
void dbg_trc_req_cs_ble(uint16_t const cs_addr)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_CS_BLE)) >> TRC_GRP_P(TRC_CS_BLE))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = REG_EM_BLE_CS_SIZE;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = CS_BLE;
            em_rd(trace, cs_addr, REG_EM_BLE_CS_SIZE);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_CS_BLE)*/

#if (TRC_CS_BT)
void dbg_trc_req_cs_bt(uint16_t const cs_addr)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_CS_BT)) >> TRC_GRP_P(TRC_CS_BT))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = REG_EM_BT_CS_SIZE;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = CS_BT;
            em_rd(trace, cs_addr, REG_EM_BT_CS_SIZE);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_CS_BT)*/

#if (TRC_RX_DESC)
void dbg_trc_req_rx_desc(uint8_t const drv_type, uint8_t const act_id, uint16_t const rx_desc_add)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_RX_DESC)) >> TRC_GRP_P(TRC_RX_DESC))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(drv_type)   + \
                      sizeof(act_id) + \
                      REG_EM_BLE_RX_DESC_SIZE;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = RX_DESC;
            *trace++ = drv_type;
            *trace++ = act_id;
            em_rd(trace, rx_desc_add, REG_EM_BLE_RX_DESC_SIZE);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_RX_DESC)*/

#if (TRC_LLCP)
void dbg_trc_req_llcp(uint8_t const llcp_trc_type, uint16_t const conhdl, uint8_t const data_len, uint8_t const *pdu_data)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_LLCP)) >> TRC_GRP_P(TRC_LLCP))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(conhdl)   + \
                      sizeof(data_len)      + \
                      data_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = llcp_trc_type;
            co_write16p(trace, conhdl);
            trace += sizeof(conhdl);
            *trace++ = data_len;
            memcpy(trace, pdu_data, data_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_LLCP)*/

#if (TRC_LMP)
void dbg_trc_req_lmp(uint8_t const lmp_trc_type, uint16_t const conhdl, uint8_t const data_len, uint8_t const *pdu_data)
{
    if (GETB(dbg_trc_env.curr_cw, TRC_LMP))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(conhdl)   + \
                      sizeof(data_len)      + \
                      2                     + \
                      data_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = lmp_trc_type;
            co_write16p(trace, conhdl);
            trace += sizeof(conhdl);
            *trace++ = data_len;
            *trace++ = pdu_data[0] & 0x1; // TrID
            *trace++ = pdu_data[0] >> 1; // Opcode
            memcpy(trace, &pdu_data[1], data_len - 1);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_lmp_ack(uint8_t const lmp_trc_type, uint16_t const conhdl, uint8_t const opcode, uint8_t const ext_code)
{

    if (GETB(dbg_trc_env.curr_cw, TRC_LMP))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(conhdl) + sizeof(opcode) + sizeof(ext_code);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = lmp_trc_type;
            co_write16p(trace, conhdl);
            trace += sizeof(conhdl);
            *trace++ = opcode;
            *trace = ((opcode == LMP_ESC4_OPCODE) ? ext_code : 0);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

#endif /*(TRC_LMP)*/

#if (TRC_L2CAP)
void dbg_trc_req_l2cap(uint8_t const l2cap_trc_type, uint16_t const conhdl, uint16_t const data_len, uint32_t const buf_ptr)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_L2CAP)) >> TRC_GRP_P(TRC_L2CAP))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(conhdl) + sizeof(data_len) + data_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = l2cap_trc_type;
            co_write16p(trace, conhdl);
            trace += sizeof(conhdl);
            co_write16p(trace, data_len);
            trace += sizeof(data_len);

#if (BLE_EMB_PRESENT)
            em_rd(trace, buf_ptr, data_len);
#else // (BLE_HOST_PRESENT)
            memcpy((void *) trace, (void *)buf_ptr, data_len);
#endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_l2cap_ack(uint16_t const conhdl, uint16_t const num_ack_pkts)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_L2CAP)) >> TRC_GRP_P(TRC_L2CAP))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(conhdl) + sizeof(num_ack_pkts);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = L2CAP_ACK;
            co_write16p(trace, conhdl);
            trace += sizeof(conhdl);
            co_write16p(trace, num_ack_pkts);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif/*(TRC_L2CAP) */

#if (TRC_ARB)
void dbg_trc_req_sch_arb_insert(uint32_t const target_time, uint8_t const status, const struct sch_arb_elt_tag *sch_arb_elt)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ARB)) >> TRC_GRP_P(TRC_ARB))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(target_time)    + \
                      sizeof(void *)               + \
                      sizeof(status)              + \
                      sizeof(struct sch_arb_elt_tag);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = SCH_ARB_REQ;
            co_write32p(trace, target_time);
            trace += sizeof(target_time);
            co_write32p(trace, (uint32_t) sch_arb_elt);
            trace += 4;
            *trace++ = status;
            memcpy(trace, sch_arb_elt, sizeof(struct sch_arb_elt_tag));

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
void dbg_trc_req_sch_arb(uint8_t const sch_arb_trc_type, const struct sch_arb_elt_tag *sch_arb_elt)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ARB)) >> TRC_GRP_P(TRC_ARB))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(void *);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = sch_arb_trc_type;
            co_write32p(trace, (uint32_t) sch_arb_elt);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
void dbg_trc_req_sch_arb_shift(const struct sch_arb_elt_tag *sch_arb_elt)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ARB)) >> TRC_GRP_P(TRC_ARB))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(void *)             + \
                      sizeof(sch_arb_elt->current_prio)    + \
                      sizeof(sch_arb_elt->duration_min)    + \
                      sizeof(sch_arb_elt->time.hs);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = SCH_ARB_SHIFT;
            co_write32p(trace, (uint32_t) sch_arb_elt);
            trace   += 4;
            *trace++ = sch_arb_elt->current_prio;
            co_write32p(trace, sch_arb_elt->duration_min);
            trace   += sizeof(sch_arb_elt->duration_min);
            co_write32p(trace, sch_arb_elt->time.hs);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_ARB)*/

#if (TRC_LC_STATE_TRANS)
void dbg_trc_req_lc_state_trans(uint16_t const conhdl, uint8_t const proc_type, uint8_t const prev_state, uint8_t const curr_state)
{
}
#endif /*(TRC_LLC_STATE_TRANS)*/

#if (TRC_LLC_STATE_TRANS)
void dbg_trc_req_llc_state_trans(uint16_t const conhdl, uint8_t const proc_type, uint8_t const prev_state, uint8_t const curr_state)
{
    if (prev_state != curr_state)
    {
        if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_LLC_STATE_TRANS)) >> TRC_GRP_P(TRC_LLC_STATE_TRANS))
        {
            //write trace in memory
            uint8_t *trace;
            uint16_t payload_len;
            payload_len = sizeof(conhdl)               + \
                          sizeof(proc_type)                   + \
                          sizeof(prev_state)                  + \
                          sizeof(curr_state);

            trace = TRC_MEM_ALLOC(payload_len);
            if (trace != NULL)
            {
                //update trace
                *trace++ = LLC_ST;
                co_write16p(trace, conhdl);
                trace   += sizeof(conhdl);
                *trace++ = proc_type;
                *trace++ = prev_state;
                *trace   = curr_state;

                // trigger the event
                ke_event_set(KE_EVENT_TRC);
            }
        }
    }
}
#endif /*(TRC_LLC_STATE_TRANS)*/

#if (TRC_HCI)
void dbg_trc_req_hci_cmd(uint16_t const opcode, uint8_t const par_len, uint32_t const *params)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_HCI)) >> TRC_GRP_P(TRC_HCI))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        /* HCI type */
        payload_len = 1 + \
                      sizeof(opcode)              + \
                      sizeof(par_len)             + \
                      par_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = H4TL_PACKET_HOST;
            *trace++ = HCI_CMD_MSG_TYPE;
            co_write16p(trace, opcode);
            trace   += sizeof(opcode);
            *trace++ = par_len;
            memcpy(trace, params, par_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_hci_evt(uint8_t const hci_evt_type, uint8_t const opcode, uint8_t const par_len, uint32_t const *params)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_HCI)) >> TRC_GRP_P(TRC_HCI))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        /* HCI type */
        payload_len = 1 + \
                      sizeof(opcode)              + \
                      sizeof(par_len)             + \
                      par_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = hci_evt_type;
            *trace++ = HCI_EVT_MSG_TYPE;
            *trace++ = opcode;
            *trace++ = par_len;
            memcpy((void *)trace, (void *)params, par_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}


void dbg_trc_req_hci_acl_data(uint8_t in, uint16_t const handle, uint16_t const par_len, uint32_t const *params)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_HCI_ACL_TRUNCATE)) >> TRC_GRP_P(TRC_HCI_ACL_TRUNCATE))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        uint16_t origin_len = par_len;
        uint8_t trace_code = in == 1 ? H4TL_PACKET_HOST : H4TL_PACKET_CTRL;

        // Not show this data
        if (par_len >= 200)
            return;

        /* HCI type */
        if (origin_len > 60)
            origin_len = 60;

        payload_len = 1 + \
                      sizeof(handle)              + \
                      sizeof(par_len)             + \
                      origin_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = trace_code;
            *trace++ = HCI_ACL_MSG_TYPE;
            co_write16p(trace, handle);
            trace   += sizeof(handle);
            co_write16p(trace, par_len);
            trace   += sizeof(par_len);
            memcpy(trace, params, origin_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
    else if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_HCI)) >> TRC_GRP_P(TRC_HCI))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        uint8_t trace_code = in == 1 ? H4TL_PACKET_HOST : H4TL_PACKET_CTRL;
        /* HCI type */
        payload_len = 1 + \
                      sizeof(handle)              + \
                      sizeof(par_len)             + \
                      par_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = trace_code;
            *trace++ = HCI_ACL_MSG_TYPE;
            co_write16p(trace, handle);
            trace   += sizeof(handle);
            co_write16p(trace, par_len);
            trace   += sizeof(par_len);
            memcpy(trace, params, par_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_HCI)*/

#if (TRC_ADV)
void dbg_trc_req_adv_rx_pdu(const uint8_t index_pkt, const uint8_t data_len, void const *data)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(uint8_t)           + \
                      sizeof(uint8_t)            + \
                      sizeof(data_len)           + \
                      data_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ADV_ADV_RX;

            *trace++ = em_ble_rxphadv_rxtype_getf(index_pkt);
            *trace++ = em_ble_rxchass_used_ch_idx_getf(index_pkt);
            *trace++ = data_len;
            memcpy(trace, data, data_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_adv_tx_pdu(const uint8_t adv_txdesc_idx, const uint8_t cs_idx)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_adv_tx_pdu);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ADV_ADV_TX;

            //fill trc_adv_tx_pdu
            struct trc_adv_tx_pdu adv_pdu;
            adv_pdu.packet_type = em_ble_txphadv_txtype_getf(adv_txdesc_idx);
            adv_pdu.advertising_channel_map = em_ble_chmap2_advchmap_getf(cs_idx);
            co_write16p(&adv_pdu.adv_addr.addr[0], em_ble_lebdaddr_get(cs_idx, 0));
            co_write16p(&adv_pdu.adv_addr.addr[2], em_ble_lebdaddr_get(cs_idx, 1));
            co_write16p(&adv_pdu.adv_addr.addr[4], em_ble_lebdaddr_get(cs_idx, 2));

            memcpy(trace, &adv_pdu, sizeof(struct trc_adv_tx_pdu));

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_init_rx_pdu(const uint8_t index_pkt, uint16_t rxdataptr)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_init_rx_pdu);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ADV_INIT_RX;

            //fill trc_init_rx_pdu
            struct trc_init_rx_pdu adv_pdu;
            adv_pdu.packet_type = em_ble_rxphadv_rxtype_getf(index_pkt);
            adv_pdu.used_channel = em_ble_rxchass_used_ch_idx_getf(index_pkt);
            em_rd((void *)&adv_pdu.adv_addr.addr[0], rxdataptr, BD_ADDR_LEN);

            memcpy(trace, &adv_pdu, sizeof(struct trc_init_rx_pdu));

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_init_tx_pdu(const uint8_t index_pkt, struct bd_addr const addr)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_adv_tx_pdu);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ADV_INIT_TX;

            //fill pdu
            struct trc_init_tx_pdu adv_pdu;
            adv_pdu.packet_type = em_ble_txphadv_txtype_getf(index_pkt);
            adv_pdu.advertising_channel_map = em_ble_chmap2_advchmap_getf(index_pkt);
            adv_pdu.peer_id_addr = addr;

            memcpy(trace, &adv_pdu, sizeof(struct trc_init_tx_pdu));

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_scan_rx_pdu(const uint8_t index_pkt, struct lld_adv_rep_ind const *ind)
{

    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_scan_rx_pdu) + ind->data_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ADV_SCAN_RX;

            //fill trc_scan_rx_pdu
            struct trc_scan_rx_pdu scan_pdu;
            scan_pdu.packet_type = em_ble_rxphadv_rxtype_getf(index_pkt);
            scan_pdu.used_channel = em_ble_rxchass_used_ch_idx_getf(index_pkt);
            scan_pdu.scanner_addr = ind->peer_id_addr;
            scan_pdu.scanner_addr_type = ind->addr_type;
            scan_pdu.data_len = ind->data_len;
            memcpy(trace, &scan_pdu, sizeof(struct trc_scan_rx_pdu));
            trace += sizeof(struct trc_scan_rx_pdu);

            em_rd(trace, ind->em_buf + ind->data_offset, scan_pdu.data_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_scan_tx_pdu(const uint8_t txdesc_idx, const uint8_t cs_idx)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_adv_tx_pdu);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ADV_SCAN_RX;

            //fill trc_adv_tx_pdu
            struct trc_adv_tx_pdu adv_pdu;
            adv_pdu.packet_type = em_ble_txphadv_txtype_getf(txdesc_idx);
            adv_pdu.advertising_channel_map = em_ble_chmap2_advchmap_getf(cs_idx);
            co_write16p(&adv_pdu.adv_addr.addr[0], em_ble_lebdaddr_get(cs_idx, 0));
            co_write16p(&adv_pdu.adv_addr.addr[2], em_ble_lebdaddr_get(cs_idx, 1));
            co_write16p(&adv_pdu.adv_addr.addr[4], em_ble_lebdaddr_get(cs_idx, 2));

            memcpy(trace, &adv_pdu, sizeof(struct trc_adv_tx_pdu));

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_ADV)*/

#if (TRC_ACL)
void dbg_trc_req_acl_rx_pdu(struct lld_acl_rx_ind const *msg)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ACL)) >> TRC_GRP_P(TRC_ACL))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_acl_rx_pdu) +  msg->data_len;

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = ACL_RX;

            //fill trc_acl_rx_pdu
            struct trc_acl_rx_pdu acl_pdu;
            acl_pdu.data_len = msg->data_len;

            memcpy(trace, &acl_pdu, sizeof(struct trc_acl_rx_pdu));
            trace += sizeof(struct trc_acl_rx_pdu);
            em_rd(trace, msg->em_buf, acl_pdu.data_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}

void dbg_trc_req_acl_tx_pdu(const uint8_t trc_acl_tx_type, const uint8_t txdesc_idx)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_ADV)) >> TRC_GRP_P(TRC_ADV))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(struct trc_acl_tx_pdu) + em_ble_txphce_txlen_getf(txdesc_idx);

        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = trc_acl_tx_type;

            //fill trc_acl_tx_pdu
            struct trc_acl_tx_pdu acl_pdu;
            acl_pdu.data_len = em_ble_txphce_txlen_getf(txdesc_idx);
            acl_pdu.txdataptr = em_ble_txdataptr_get(txdesc_idx);
            memcpy(trace, &acl_pdu, sizeof(struct trc_acl_tx_pdu));
            trace += sizeof(struct trc_acl_tx_pdu);
            uint16_t buf_offset = em_ble_txdataptr_getf(txdesc_idx);
            em_rd(trace, buf_offset, acl_pdu.data_len);

            // trigger the event
            ke_event_set(KE_EVENT_TRC);
        }
    }
}
#endif /*(TRC_ACL)*/

#if (TRC_CUSTOM)
void dbg_trc_req_custom(const uint8_t trc_id, const uint8_t data_type, void const *data)
{
    if ((dbg_trc_env.curr_cw & TRC_GRP_B(TRC_CUSTOM)) >> TRC_GRP_P(TRC_CUSTOM))
    {
        //write trace in memory
        uint8_t *trace;
        uint16_t payload_len;
        payload_len = sizeof(trc_id) + sizeof(data_type);
        switch (data_type)
        {
        case UINT8_T:
        {
            payload_len++;
        }
        break;
        case UINT16_T:
        {
            payload_len += 2;
        }
        break;
        case UINT32_T:
        case VOID:
        {
            payload_len += 4;
        }
        break;
        default:
        {
            ASSERT_ERR(0);
        }
        }
        trace = TRC_MEM_ALLOC(payload_len);
        if (trace != NULL)
        {
            //update trace
            *trace++ = CUSTOM;
            *trace++ = (uint8_t) trc_id;
            *trace++ = (uint8_t) data_type;
            switch (data_type)
            {
            case UINT8_T:
            {
                *trace++ = *(uint8_t *)data;
            }
            break;
            case UINT16_T:
            {
                co_write16p(trace, *(uint16_t *)data);
            }
            break;
            case UINT32_T:
            {
                co_write32p(trace, *(uint32_t *)data);
            }
            break;
            case VOID:
            {
                co_write32p(trace, (uint32_t) data);
            }
            break;
            default:
            {
                ASSERT_ERR(0);
            }
            break;
            }
            // trigger the event
//            ke_event_set(KE_EVENT_TRC);
            dbg_trc_tx_trigger();
        }
    }
}
#endif /*(TRC_CUSTOM)*/

void dbg_trc_init(bool is_reset)
{
    uint8_t length = PARAM_LEN_TRACER_CONFIG;
    uint8_t tracer_cfg[PARAM_LEN_TRACER_CONFIG];

    if (!is_reset)
    {
        //Register trace event
        ke_event_callback_set(KE_EVENT_TRC, &dbg_trc_ke_evt_cbk);

        //Set the compiled configuration word
        dbg_trc_set_compiled_cw();

        //initialize the trace memory
        dbg_trc_mem_init();
    }

    ASSERT_ERR(length == 4);
    //Read the configuration word from NVDS
    if (rwip_param.get(PARAM_ID_TRACER_CONFIG, &length, tracer_cfg) == PARAM_OK)
    {
        //Set the configuration word with the value stored in NVDS
        dbg_trc_env.curr_cw = co_read32p(tracer_cfg);
    }
    else
    {
        //Initialize the configuration word to 0
        dbg_trc_env.curr_cw = 0;
    }

    //initialize transport layer
    dbg_trc_tl_init();
}
#endif /*(TRACER_PRESENT)*/
/// @} TRACER
