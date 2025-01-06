/**
 ****************************************************************************************
 *
 * @file dbg_trc.h
 *
 * @brief This file contains definitions related to the Tracer module.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef DBG_TRC_H_
#define DBG_TRC_H_

/**
****************************************************************************************
* @addtogroup TRACER
* @ingroup TRACER
* @brief Tracer module sends traces through UART
* @{
****************************************************************************************
*/

#include "rwip_config.h"

void print_data(char *data, unsigned int offset, int len);

#if (TRACER_PRESENT)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>

#if (TRC_KE_MSG || TRC_KE_TMR)
    #include "ke_msg.h"
#endif /*(TRC_KE_MSG || TRC_KE_TMR)*/

#if (TRC_ARB)
    #include "sch_arb.h"
#endif /*(TRC_ARB) */

#if (TRC_ADV)
    #include "lld.h"
#endif /*(TRC_ADV)*/

/*
 * DEFINES
 ****************************************************************************************
 */
#define TRC_GRP_B(__t) (__t##_BIT)
#define TRC_GRP_P(__t) (__t##_POS)


#define TRC_OPCODE_LEN      (1)
#define TRC_PDULEN_LEN      (2)
#define TRC_MSG_HDR_LEN     (TRC_OPCODE_LEN + TRC_PDULEN_LEN)

#define TRC_CFG_WORD_LEN    (4)

/*
 * ENUMERATION DEFINITION
 ****************************************************************************************
 */
/// Possible types of trace (parameter, size, conversion_to_string)
/*@TRACE*/
enum trace_types
{
#if TRC_KE_MSG
    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_msg_send
     *  TRC_VARS: msg_id_names = get_elements(TASK_API_ID, "'@%s_msg_id'%(#.lower().rsplit('_', 1)[1])")
     *  TRC_UNION: ("Parameters", "$Message_id", msg_id_names, "#.lower()")
     *  @NO_PAD
     *
     *  ('Message_id' , uint16_t, "@['%s_msg_id'%(@TASK_API_ID.name(int($Message_id >> 8)).rsplit('_',1)[1].lower())].name(int($Message_id))", msg_id_names),
     *  ('Destination_task_id', uint16_t, "@KE_TASK_TYPE.name(int($Destination_task_id & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Source_task_id', uint16_t, "@KE_TASK_TYPE.name(int($Source_task_id & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Parameter_length', uint16_t),
     *  ('Parameters', "union", CArray(uint8_t, '$Parameter_length'))
     */
    KE_MSG_SEND         = 0x00,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_msg_handled
     *  TRC_VARS: msg_id_names = get_elements(TASK_API_ID, "'@%s_msg_id'%(#.lower().rsplit('_', 1)[1])")
     *  @NO_PAD
     *
     *  ('Message_id' , uint16_t, "@['%s_msg_id'%(@TASK_API_ID.name(int($Message_id >> 8)).rsplit('_',1)[1].lower())].name(int($Message_id))", msg_id_names),
     *  ('Destination_task_id', uint16_t, "@KE_TASK_TYPE.name(int($Destination_task_id & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Source_task_id', uint16_t, "@KE_TASK_TYPE.name(int($Source_task_id & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Message_status', uint8_t, "@ke_msg_status_tag.name(int($Message_status))", '@ke_msg_status_tag')
     */
    KE_MSG_HANDLED      = 0x01,
#endif /*TRC_KE_MSG*/

#if TRC_KE_TMR
    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_tmr_set
     *  TRC_VARS: msg_id_names = get_elements(TASK_API_ID, "'@%s_msg_id'%(#.lower().rsplit('_', 1)[1])")
     *  @NO_PAD
     *
     *  ('Target_time' , uint32_t),
     *  ('Task_ID', uint16_t, "@KE_TASK_TYPE.name(int($Task_ID & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Message_id', uint16_t, "@['%s_msg_id'%(@TASK_API_ID.name(int($Message_id >> 8)).rsplit('_',1)[1].lower())].name(int($Message_id))", msg_id_names)
     */
    KE_TMR_SET          = 0x02,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_tmr_clr
     *  TRC_VARS: msg_id_names = get_elements(TASK_API_ID, "'@%s_msg_id'%(#.lower().rsplit('_', 1)[1])")
     *  @NO_PAD
     *
     *
     *  ('Target_time' , uint32_t),
     *  ('Task_ID', uint16_t, "@KE_TASK_TYPE.name(int($Task_ID & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Message_id', uint16_t, "@['%s_msg_id'%(@TASK_API_ID.name(int($Message_id >> 8)).rsplit('_',1)[1].lower())].name(int($Message_id))", msg_id_names)
     */
    KE_TMR_CLR          = 0x03,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_tmr_exp
     *  TRC_VARS: msg_id_names = get_elements(TASK_API_ID, "'@%s_msg_id'%(#.lower().rsplit('_', 1)[1])")
     *  @NO_PAD
     *
     *  ('Target_time' , uint32_t),
     *  ('Task_ID', uint16_t, "@KE_TASK_TYPE.name(int($Task_ID & 0x00FF))", '@KE_TASK_TYPE'),
     *  ('Message_id', uint16_t, "@['%s_msg_id'%(@TASK_API_ID.name(int($Message_id >> 8)).rsplit('_',1)[1].lower())].name(int($Message_id))", msg_id_names)
     */
    KE_TMR_EXP          = 0x04,
#endif /*TRC_KE_TMR*/

#if TRC_KE_EVT
    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_evt_set
     *  @NO_PAD
     *
     *  ('Event_type', uint8_t, "@KE_EVENT_TYPE.name(int($Event_type))", '@KE_EVENT_TYPE'),
     */
    KE_EVT_SET          = 0x05,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_evt_handled
     *  @NO_PAD
     *
     *  ('Event_type', uint8_t, "@KE_EVENT_TYPE.name(int($Event_type))", '@KE_EVENT_TYPE')
     */
    KE_EVT_HANDLED      = 0x06,

#endif /* TRC_KE_EVT */

#if TRC_MEM
    /*  @TRACE_DEF
     *  TRC_NAME: trc_mem_alloc
     *  @NO_PAD
     *
     *  ('Heap_ID', uint8_t, "@KE_MEM_HEAP.name(int($Heap_ID))", '@KE_MEM_HEAP'),
     *  ('Block_address', uint32_t),
     *  ('Size', uint32_t),
     */
    MEM_ALLOC           = 0x07,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_mem_free
     *  @NO_PAD
     *
     *  ('Heap_ID', uint8_t, "@KE_MEM_HEAP.name(int($Heap_ID))", '@KE_MEM_HEAP'),
     *  ('Block_address', uint32_t),
     *  ('Size', uint32_t),
     */
    MEM_FREE            = 0x08,

#endif /*TRC_MEM*/

#if TRC_SLEEP
    /*  @TRACE_DEF
     *  TRC_NAME: trc_sleep_enter
     */
    SLEEP_ENTER         = 0x09,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_sleep_wakeup
     */
    SLEEP_WAKEUP        = 0x0A,
#endif /*TRC_SLEEP*/

#if TRC_SW_ASS
    //TODO[AAL] add support for strings in rwdt_types
    ASSERT_WARNING      = 0x0B,
    ASSERT_ERROR        = 0x0C,
#endif /*TRC_SW_ASS*/

#if TRC_PROG
    /*  @TRACE_DEF
     *  TRC_NAME: trc_et_prog
     *  @NO_PAD
     *
     * ('Entry_index', uint8_t),
     * ('Entry_value', ET),
     */
    ET_PROG             = 0x0D,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_conn_evt_cnt
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Event_counter', uint16_t),
     */
    CONN_EVT_CNT        = 0x0E,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_frm_cmp_bt
     *  @NO_PAD
     *
     * ('Timestamp_programmed_frame', uint32_t),
     * ('Frame_identifier', uint8_t),
     * ('Callback_for_interrupt_handling', uint32_t)
     */
    FRM_CMP_BT          = 0x0F,
    /*  @TRACE_DEF
     *  TRC_NAME: trc_frm_cmp_ble
     *  @NO_PAD
     *
     * ('Timestamp_programmed_frame', uint32_t),
     * ('Frame_identifier', uint8_t),
     * ('Callback_for_interrupt_handling', uint32_t)
     */
    FRM_CMP_BLE         = 0x10,
#endif /*TRC_PROG*/

#if TRC_CS_BLE
    /*  @TRACE_DEF
     *  TRC_NAME: trc_cs_ble
     *  @NO_PAD
     *
     *  ('Control_structure', CS),
     */
    CS_BLE              = 0x11,
#endif /*TRC_CS_BLE*/

#if TRC_CS_BT
    CS_BT               = 0x12,
#endif /*TRC_CS_BT*/

#if TRC_RX_DESC
    /*  @TRACE_DEF
     *  TRC_NAME: trc_rx_desc
     *  @NO_PAD
     *
     * ('Driver_type', uint8_t, "@lld_driver_types.name(int($Driver_type))", '@lld_driver_types'),
     * ('Connection_index', uint8_t),
     * ('Rx_descriptor', RX_DESC)
     */
    RX_DESC             = 0x13,
#endif /*TRC_RX_DESC*/

#if TRC_LLCP
    /*  @TRACE_DEF
     *  TRC_NAME: trc_llcp_tx
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint8_t),
     * ('LLCP_pdu', llcp_pdu)
     */
    LLCP_TX             = 0x14,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_llcp_rx
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint8_t),
     * ('LLCP_pdu', llcp_pdu)
     */
    LLCP_RX             = 0x15,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_llcp_ack
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint8_t),
     * ('LLCP_pdu', llcp_pdu)
     */
    LLCP_ACK            = 0x16,
#endif /*TRC_LLCP*/

#if TRC_LMP
    /*  @TRACE_DEF
     *  TRC_NAME: trc_lmp_tx
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint8_t),
     * ('TrID', uint8_t),
     * ('Opcode', uint8_t),
     * ('Data', CArray(uint8_t, '$Data_Length')),
     */
    LMP_TX              = 0x17,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_lmp_rx
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint8_t),
     * ('TrID', uint8_t),
     * ('Opcode', uint8_t),
     * ('Data', CArray(uint8_t, '$Data_Length')),
     */
    LMP_RX              = 0x18,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_lmp_ack
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint8_t),
     * ('TrID', uint8_t),
     * ('Opcode', uint8_t),
     * ('Data', CArray(uint8_t, '$Data_Length')),
     */
    LMP_ACK             = 0x19,
#endif /*TRC_LMP*/

#if TRC_L2CAP
    /*  @TRACE_DEF
     *  TRC_NAME: trc_l2cap_tx
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint16_t),
     * ('l2cap_sig_pdu', l2cap_sig_pdu)
     */
    L2CAP_TX            = 0x1A,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_l2cap_rx
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Data_Length', uint16_t),
     * ('l2cap_sig_pdu', l2cap_sig_pdu),
     */
    L2CAP_RX            = 0x1B,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_l2cap_ack
     *  @NO_PAD
     *
     * ('Connection_handle', uint16_t),
     * ('Number_acknowledged_packets', uint16_t),
     */
    L2CAP_ACK           = 0x1C,
#endif /*TRC_L2CAP*/

#if TRC_ARB
    /*  @TRACE_DEF
     *  TRC_NAME: trc_sch_arb_req
     *  @NO_PAD
     *
     * ('Target_time', uint32_t),
     * ('elt_address', uint32_t),
     * ('Status', uint8_t),
     * ('sch_arb_elt', sch_arb_elt_tag)
     */
    SCH_ARB_REQ              = 0x1D,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_sch_arb_canc
     *  @NO_PAD
     *
     * ('elt_address', uint32_t),
     */
    SCH_ARB_CANC             = 0x1E,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_sch_arb_rem
     *  @NO_PAD
     *
     * ('elt_address', uint32_t),
     */
    SCH_ARB_REM              = 0x1F,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_sch_arb_start
     *  @NO_PAD
     *
     * ('elt_address', uint32_t),
     */
    SCH_ARB_START            = 0x20,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_sch_arb_shift
     *  @NO_PAD
     *
     * ('elt_address', uint32_t),
     * ('Current_prio', uint8_t),
     * ('Duration_min', uint32_t),
     * ('Timestamp', uint32_t),
     */
    SCH_ARB_SHIFT            = 0x21,
#endif /*TRC_ARB*/

#if TRC_LC_STATE_TRANS
    LC_ST               = 0x22,
#endif /*TRC_LC_STATE_TRANS*/

#if TRC_LLC_STATE_TRANS
    /*  @TRACE_DEF
     *  TRC_NAME: trc_llc_st
     *  TRC_VARS: llc_proc_states = get_elements(llc_proc_id, "'@llc_%s_state'%(#.split('_', 2)[2].lower())")
     *  @NO_PAD
     *
     *  ('Connection_handle', uint16_t),
     *  ('Procedure_ID', uint8_t,  '@llc_proc_id.name(int($Procedure_ID))', '@llc_proc_id'),
     *  ('Previous_state', uint8_t, "@['_llc_%s_state'%(@llc_proc_id.name(int($Procedure_ID)).split('_', 2)[2].lower())].name(int($Previous_state))", llc_proc_states),
     *  ('Current_state', uint8_t, "@['llc_%s_state'%(@llc_proc_id.name(int($Procedure_ID)).split('_', 2)[2].lower())].name(int($Current_state))", llc_proc_states),
     */
    LLC_ST              = 0x23,
#endif /*TRC_LLC_STATE_TRANS*/

#if TRC_HCI
    /*  @TRACE_DEF
     *  TRC_NAME: trc_hci_cmd
     *  TRC_UNION: ("Parameters", "$Hci_opcode", hci_opcode, "#.rsplit('_OPCODE', 1)[0].lower()")
     *  @NO_PAD
     *
     * ('Hci_opcode', uint16_t, '@hci_opcode.name(int($Hci_opcode))', '@hci_opcode'),
     * ('Param_length', uint16_t),
     * ('Parameters', "union", CArray(uint8_t, '$Param_length'))
     */
    HCI_CMD             = 0x24,


    /*  @TRACE_DEF
     *  TRC_NAME: trc_hci_cmd_stat_evt
     *  TRC_UNION: ("Parameters", "$Hci_opcode", hci_opcode, "'%s_cmp_evt'%(#.rsplit('_OPCODE', 1)[0].lower())")
     *  @NO_PAD
     *
     * ('Hci_opcode', uint16_t, '@hci_opcode.name(int($Hci_opcode))', '@hci_opcode'),
     * ('Param_length', uint16_t),
     * ('Parameters', "union", CArray(uint8_t, '$Param_length'))
     */
    HCI_CMD_STAT_EVT     = 0x25,


    /*  @TRACE_DEF
     *  TRC_NAME: trc_hci_cmd_cmp_evt
     *  TRC_UNION: ("Parameters", "$Hci_opcode", hci_opcode, "'%s_cmp_evt'%(#.rsplit('_OPCODE', 1)[0].lower())")
     *  @NO_PAD
     *
     * ('Hci_opcode', uint16_t, '@hci_opcode.name(int($Hci_opcode))', '@hci_opcode'),
     * ('Param_length', uint16_t),
     * ('Parameters', "union", CArray(uint8_t, '$Param_length'))
     */
    HCI_CMD_CMP_EVT     = 0x26,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_hci_evt
     *  TRC_UNION: ("Parameters", "$Hci_evt_code", hci_evt_code, "#.rsplit('_CODE', 1)[0].lower()")
     *  @NO_PAD
     *
     * ('Hci_evt_code', uint16_t, '@hci_evt_code.name(int($Hci_evt_code))', '@hci_evt_code'),
     * ('Param_length', uint16_t),
     * ('Parameters', "union", CArray(uint8_t, '$Param_length'))
     */
    HCI_EVT             = 0x27,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_hci_le_evt
     *  TRC_UNION: ("Parameters", "$Hci_le_subcode", hci_le_evt_subcode, "#.rsplit('_SUBCODE', 1)[0].lower()")
     *  @NO_PAD
     *
     * ('Hci_le_subcode', uint16_t, '@hci_le_evt_subcode.name(int($Hci_le_subcode))', '@hci_le_evt_subcode'),
     * ('Param_length', uint16_t),
     * ('Parameters', "union", CArray(uint8_t, '$Param_length'))
     */
    HCI_LE_EVT          = 0x28,
#endif /*TRC_HCI*/

#if TRC_ADV
    /*  @TRACE_DEF
     *  TRC_NAME: trc_adv_init_rx
     *
     * ('adv_pdu', trc_init_rx_pdu)
     */
    ADV_INIT_RX             = 0x29,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_adv_init_tx
     *
     * ('adv_pdu', trc_adv_tx_pdu)
     */
    ADV_INIT_TX             = 0x30,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_adv_scan_rx
     *
     * ('adv_pdu', trc_scan_rx_pdu)
     */
    ADV_SCAN_RX             = 0x31,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_adv_scan_tx
     *
     * ('adv_pdu', trc_adv_tx_pdu)
     */
    ADV_SCAN_TX             = 0x32,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_adv_adv_rx
     *  TRC_UNION: ("payload", "$packet_type", ble_adv_type, [
     *                  ('BLE_CONNECT_IND', 'connect_req'),
     *                  ('BLE_SCAN_REQ', 'scan_req'),
     *  ])
     *  @NO_PAD
     *
     * ('packet_type', uint8_t, '@ble_adv_type.name(int($packet_type))', '@ble_adv_type'),
     * ('used_channel', uint8_t),
     * ('payload_length', uint8_t),
     * ('payload', "union", CArray(uint8_t, '$payload_length'))
     */
    ADV_ADV_RX             = 0x33,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_adv_adv_tx
     *
     * ('adv_pdu', trc_adv_tx_pdu)
     */
    ADV_ADV_TX             = 0x34,
#endif /*TRC_ADV*/

#if TRC_ACL || TRC_HCI
    /*  @TRACE_DEF
     *  TRC_NAME: trc_acl_rx
     *
     * ('adv_pdu', trc_acl_rx_pdu)
     */
    ACL_RX                  = 0x35,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_acl_prog_tx
     *
     * ('adv_pdu', trc_acl_tx_pdu)
     */
    ACL_PROG_TX                  = 0x36,

    /*  @TRACE_DEF
     *  TRC_NAME: trc_acl_ack_tx
     *
     * ('adv_pdu', trc_acl_tx_pdu)
     */
    ACL_ACK_TX               = 0x37,
#endif /*TRC_ACL*/

#if TRC_CUSTOM
    /*  @TRACE_DEF
     *  TRC_NAME: trc_custom
     *  TRC_UNION: ("Data", "$Trace_type", custom_trace_types, "#.lower()")
     *
     *  ('Custom_trace_ID', uint8_t),
     *  ('Trace_type', uint8_t, "@custom_trace_types.name(int($Trace_type))", '@custom_trace_types'),
     *  ('Data', "union"),
     */
    CUSTOM              = 0x38,
#endif /*TRC_CUSTOM*/
#if TRC_KE_STATE
    /*  @TRACE_DEF
     *  TRC_NAME: trc_ke_state_set
     *  @NO_PAD
     *
     *  ('task_id', uint16_t),
     *  ('task_state', uint8_t),
     */
    KE_STATE_SET        = 0x39,

#endif /* TRC_KE_STATE */
    PRINTF              = 0x60,
    H4TL_PACKET_HOST    = 0x61,
    H4TL_PACKET_CTRL    = 0x62,
    TRC_SHELL           = 0x63,
    TRC_ID_STRING       = 0x64,
    TRC_ID_INTEGER      = 0x65,
    TRC_ID_INTEGER_1    = 0x66,
    TRC_ID_INTEGER_2    = 0x67,
    TRC_ID_CRASH_DUMP   = 0x68,
};

/// Configuration word bit field
///
///     7       6       5       4       3       2       1       0
/// +-------+-------+-------+-------+-------+-------+-------+-------+
/// |CS_BLE | PROG  |SW_ASS | SLEEP |  MEM  |KE_EVT |KE_TMR |KE_MSG |
/// +-------+-------+-------+-------+-------+-------+-------+-------+
///
///    15      14      13      12      11       10      9       8
/// +-------+-------+-------+-------+-------+-------+-------+-------+
/// | LC_ST |LLC_ST |  ARB  | L2CAP |  LMP  | LLCP  |RX_DESC| CS_BT |
/// +-------+-------+-------+-------+-------+-------+-------+-------+
///
///    23      22      21      20      19       18      17      16
/// +-------+-------+-------+-------+-------+-------+-------+-------+
/// |       |       |       |KE_STAT|CUSTOM |  ACL  |  ADV  |  HCI  |
/// +-------+-------+-------+-------+-------+-------+-------+-------+
///
///    31      30      29      28       27      26      25      24
/// +-------+-------+-------+-------+-------+-------+-------+-------+
/// |       |       |       |       |       |       |       |       |
/// +-------+-------+-------+-------+-------+-------+-------+-------+
///
enum dbg_trc_cfg_fields
{
    /// Kernel message
    TRC_KE_MSG_POS              = 0,
    TRC_KE_MSG_BIT              = 0x00000001,

    /// Kernel timer
    TRC_KE_TMR_POS              = 1,
    TRC_KE_TMR_BIT              = 0x00000002,

    /// Kernel event
    TRC_KE_EVT_POS              = 2,
    TRC_KE_EVT_BIT              = 0x00000004,

    /// Memory allocation and deallocation
    TRC_MEM_POS                 = 3,
    TRC_MEM_BIT                 = 0x00000008,

    /// Sleep mode
    TRC_SLEEP_POS               = 4,
    TRC_SLEEP_BIT               = 0x00000010,

    /// Software Assert
    TRC_SW_ASS_POS              = 5,
    TRC_SW_ASS_BIT              = 0x00000020,

    /// Programming of the exchange table, updating of the event counter and handling of "End of Event" interrupt
    TRC_PROG_POS                = 6,
    TRC_PROG_BIT                = 0x00000040,

    /// BLE Control structure
    TRC_CS_BLE_POS              = 7,
    TRC_CS_BLE_BIT              = 0x00000080,

    /// BT Control structure
    TRC_CS_BT_POS               = 8,
    TRC_CS_BT_BIT               = 0x00000100,

    /// Processing of RX descriptors at each LLD driver
    TRC_RX_DESC_POS             = 9,
    TRC_RX_DESC_BIT             = 0x00000200,

    /// LLCP transmission, reception and acknowledgment
    TRC_LLCP_POS                = 10,
    TRC_LLCP_BIT                = 0x00000400,

    /// LMP transmission, reception and acknowledgment
    TRC_LMP_POS                 = 11,
    TRC_LMP_BIT                 = 0x00000800,

    /// L2CAP transmission, reception and acknowledgment
    TRC_L2CAP_POS               = 12,
    TRC_L2CAP_BIT               = 0x00001000,

    /// Scheduling request, cancellation, shift and remove
    TRC_ARB_POS                 = 13,
    TRC_ARB_BIT                 = 0x00002000,

    /// LLC state transition
    TRC_LLC_STATE_TRANS_POS     = 14,
    TRC_LLC_STATE_TRANS_BIT     = 0x00004000,

    /// LC state transition
    TRC_LC_STATE_TRANS_POS      = 15,
    TRC_LC_STATE_TRANS_BIT      = 0x00008000,

    /// HCI messages (in Full embedded mode)
    TRC_HCI_POS                 = 16,
    TRC_HCI_BIT                 = 0x00010000,

    /// Advertising pdu
    TRC_ADV_POS                 = 17,
    TRC_ADV_BIT                 = 0x00020000,

    /// ACL pdu
    TRC_ACL_POS                 = 18,
    TRC_ACL_BIT                 = 0x00040000,

    /// Custom trace
    TRC_CUSTOM_POS              = 19,
    TRC_CUSTOM_BIT              = 0x00080000,

    /// KE_STATE trace
    TRC_KE_STATE_POS            = 20,
    TRC_KE_STATE_BIT            = 0x00100000,

    /// ACL truncate trace
    TRC_HCI_ACL_TRUNCATE_POS    = 21,
    TRC_HCI_ACL_TRUNCATE_BIT    = 0x00200000,

};

/// Tracer packet types
enum dbg_trc_pkt_types
{
    /// Trace message
    TRACE       = 0x01,
    /// Configuration message
    TRACER_CFG  = 0x02,
    /// Acknowledgment message
    TRACER_ACK  = 0x03,
    TRACER_SHELL = 0x4
};

#if (TRC_RX_DESC)
/// LLD driver types
/*@TRACE*/
enum lld_driver_types
{
    LLD_SCAN    = 0x01,
    LLD_INIT    = 0x02,
    LLD_ADV     = 0x03,
    LLD_CON     = 0x04,
    LLD_CIS     = 0x05,
};
#endif /*(TRC_RX_DESC)*/

#if (TRC_CUSTOM)
/// Custom trace types
/*@TRACE*/
enum custom_trace_types
{
    UINT8_T     =   0x01,
    UINT16_T    =   0x02,
    UINT32_T    =   0x03,
    VOID        =   0x04,
};
#endif /*(TRC_CUSTOM)*/
#endif /*(TRACER_PRESENT)*/

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATION
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Request a new trace
 *
 * This function checks the current configuration of the tracer
 *
 ****************************************************************************************
 */
#if (TRACER_PRESENT && TRC_KE_MSG)
/**
 ****************************************************************************************
 * @brief Trace request API for kernel message pushed in the send queue
 *
 * This function is called when a kernel message is pushed in the send queue.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "kernel message" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   msg     Kernel message
 ****************************************************************************************
 */
void dbg_trc_req_ke_msg_send(struct ke_msg const *msg);

/// Macro used to send trace request API for kernel message send
#define TRC_REQ_KE_MSG_SEND(msg) \
        dbg_trc_req_ke_msg_send(msg);

/**
 ****************************************************************************************
 * @brief Trace request API for kernel message handled
 *
 * This function is called when a task handles a message
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "kernel message" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   id          Kernel message id
 * @param[in]   dest_id     Destination kernel identifier
 * @param[in]   src_id      Source kernel identifier
 * @param[in]   msg_status  Status returned by the kernel message handler
 ****************************************************************************************
 */
void dbg_trc_req_ke_msg_handled(ke_msg_id_t const id, ke_task_id_t const dest_id, ke_task_id_t const src_id, uint8_t const msg_status);

/// Macro used to send trace request API for kernel message handled
#define TRC_REQ_KE_MSG_HANDLED(id, dest_id, src_id, msg_status) \
        dbg_trc_req_ke_msg_handled(id, dest_id, src_id, msg_status);
#else /*TRACER_PRESENT && TRC_KE_MSG*/
#define TRC_REQ_KE_MSG_SEND(msg)
#define TRC_REQ_KE_MSG_HANDLED(id, dest_id, src_id, msg_status)
#endif /*TRACER_PRESENT && TRC_KE_MSG*/

#if (TRACER_PRESENT && TRC_KE_TMR)
/**
 ****************************************************************************************
 * @brief Trace request API for kernel timer
 *
 * This function is called when a kernel timer is set, cleared or expired.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "kernel timer" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   tmr_trc_type        Trace type for kernel timer (timer set, cleared and expired of @see enum trace_types)
 * @param[in]   target_time         Timer time value
 * @param[in]   task_id             Timer task identifier
 * @param[in]   msg_id              Timer message identifier
 ****************************************************************************************
 */
void dbg_trc_req_ke_tmr(uint8_t const tmr_trc_type, uint32_t const target_time, ke_task_id_t const task_id, ke_msg_id_t const msg_id);

/// Macro used to send trace request API for kernel timer set
#define TRC_REQ_KE_TMR_SET(target_time, task_id, msg_id) \
        dbg_trc_req_ke_tmr(KE_TMR_SET, target_time, task_id, msg_id);

/// Macro used to send trace request API for kernel timer cleared
#define TRC_REQ_KE_TMR_CLR(target_time, task_id, msg_id) \
        dbg_trc_req_ke_tmr(KE_TMR_CLR,target_time, task_id, msg_id);

/// Macro used to send trace request API for kernel timer expired
#define TRC_REQ_KE_TMR_EXP(target_time, task_id, msg_id) \
        dbg_trc_req_ke_tmr(KE_TMR_EXP,target_time, task_id, msg_id);
#else /*(TRACER_PRESENT && TRC_KE_TMR)*/
#define TRC_REQ_KE_TMR_SET(target_time, task_id, msg_id)
#define TRC_REQ_KE_TMR_CLR(target_time, task_id, msg_id)
#define TRC_REQ_KE_TMR_EXP(target_time, task_id, msg_id)
#endif /*(TRACER_PRESENT && TRC_KE_TMR)*/

#if (TRACER_PRESENT && TRC_KE_EVT)
/**
 ****************************************************************************************
 * @brief Trace request API for kernel event
 *
 * This function is called when a kernel event is set or handled.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "kernel event" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   evt_req_type        Trace type for kernel event (event set and handled of @see enum trace_types)
 * @param[in]   event_type          Type of event
 ****************************************************************************************
 */
void dbg_trc_req_ke_evt(uint8_t const evt_trc_type, uint8_t const event_type);

/// Macro used to send trace request API for kernel event set
#define TRC_REQ_KE_EVT_SET(evt_type)                    \
        dbg_trc_req_ke_evt(KE_EVT_SET, evt_type);

/// Macro used to send trace request API for kernel event handled
#define TRC_REQ_KE_EVT_HANDLED(evt_type)                \
        dbg_trc_req_ke_evt(KE_EVT_HANDLED, evt_type);
#else /*(TRACER_PRESENT && TRC_KE_EVT)*/
#define TRC_REQ_KE_EVT_SET(evt_type)
#define TRC_REQ_KE_EVT_HANDLED(evt_type)
#endif /*(TRACER_PRESENT && TRC_KE_EVT)*/


#if (TRACER_PRESENT && TRC_KE_STATE)
/**
 ****************************************************************************************
 * @brief Trace request API for kernel task state
 *
 * This function is called when a kernel task state is set.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "kernel state" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   evt_req_type        Trace type for kernel event (event set and handled of @see enum trace_types)
 * @param[in]   task_id             Task identifier
 * @param[in]   task_state          Task State
 ****************************************************************************************
 */
void dbg_trc_req_ke_state(uint8_t const evt_trc_type, uint16_t const task_id, uint8_t const task_state);

/// Macro used to send trace request API for kernel task state set
#define TRC_REQ_KE_STATE_SET(task_id, task_state)                \
        dbg_trc_req_ke_state(KE_STATE_SET, task_id, task_state);
#else /*(TRACER_PRESENT && TRC_KE_EVT)*/
#define TRC_REQ_KE_STATE_SET(task_id, task_state)
#endif /*(TRACER_PRESENT && TRC_KE_STATE)*/

#if (TRACER_PRESENT && TRC_MEM)
/**
 ****************************************************************************************
 * @brief Trace request API for memory management
 *
 * This function is called when the memory is allocated or freed.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "memory" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   mem_trc_type    Trace type for memory management (memory alloc and free of @see enum trace_types)
 * @param[in]   heap_id         Heap identifier
 * @param[in]   blk_address     Address of the used memory block
 * @param[in]   size            Size of the memory area that has been allocated or freed.
 ****************************************************************************************
 */
void dbg_trc_req_mem(uint8_t const mem_trc_type, uint8_t const heap_id, void const *blk_address, uint32_t const size);

/// Macro used to send trace request API for memory allocation
#define TRC_REQ_MEM_ALLOC(heap_id, blk_address, size)   \
        dbg_trc_req_mem(MEM_ALLOC, heap_id, blk_address, size);

/// Macro used to send trace request API for memory deallocation
#define TRC_REQ_MEM_FREE(heap_id, blk_address, size)    \
        dbg_trc_req_mem(MEM_FREE, heap_id, blk_address, size);

#else /*(TRACER_PRESENT && TRC_MEM)*/
#define TRC_REQ_MEM_ALLOC(heap_id, blk_address, size)
#define TRC_REQ_MEM_FREE(heap_id, blk_address, size)
#endif/*(TRACER_PRESENT && TRC_MEM)*/

#if (TRACER_PRESENT && TRC_SLEEP)
/**
 ****************************************************************************************
 * @brief Trace request API for sleep mode
 *
 * This function is called when the IP enter in deep sleep and when it wakes up.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "sleep" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   sleep_trc_type  Trace type for sleep mode (sleep enter and wakeup of @see enum trace_types)
 ****************************************************************************************
 */
void dbg_trc_req_deep_sleep(uint8_t const sleep_trc_type);

/// Macro used to send trace request API when the IP enters in deep sleep
#define TRC_REQ_SLEEP()                 \
        dbg_trc_req_deep_sleep(SLEEP_ENTER);

/// Macro used to send trace request API when the IP wakes up from deep sleep
#define TRC_REQ_WAKEUP()                \
        dbg_trc_req_deep_sleep(SLEEP_WAKEUP);

#else /*(TRACER_PRESENT && TRC_SLEEP)*/
#define TRC_REQ_SLEEP()
#define TRC_REQ_WAKEUP()
#endif /*(TRACER_PRESENT && TRC_SLEEP)*/

#if (TRACER_PRESENT && TRC_SW_ASS)
/**
 ****************************************************************************************
 * @brief Trace request API for software asserts
 *
 * This function is called when an assertion statement is evaluated as true.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "sw assertions" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   ass_trc_type    Trace type for assertions (assert warning and assert error of @see enum trace_types)
 * @param[in]   filename        Name of the file that triggered the assertion
 * @param[in]   code_line       Line of code where the assertion was triggered
 * @param[in]   param0          Parameter value 0
 * @param[in]   param1          Parameter value 1
 ****************************************************************************************
 */
void dbg_trc_req_sw_ass(uint8_t const ass_trc_type,  char const *filename, uint16_t const code_line, uint32_t const param0, uint32_t const param1);

/// Macro used to send trace request API when an assert warning is evaluated as true
#define TRC_REQ_SW_ASS_WARN(filename, code_line, param0, param1)                     \
        dbg_trc_req_sw_ass(ASSERT_WARNING, filename, code_line, param0, param1);

/// Macro used to send trace request API when an assert warning is evaluated as true
#define TRC_REQ_SW_ASS_ERR(filename, code_line, param0, param1)                     \
        dbg_trc_req_sw_ass(ASSERT_ERROR, filename, code_line, param0, param1);

#else /*(TRACER_PRESENT && TRC_SW_ASS)*/
#define dbg_trc_req_sw_ass(ass_trc_type, filename, code_line, param0, param1)
#define TRC_REQ_SW_ASS_WARN(filename, code_line, param0, param1)
#define TRC_REQ_SW_ASS_ERR(filename, code_line, param0, param1)
#endif /*(TRACER_PRESENT && TRC_SW_ASS)*/

void dbg_dump_trace(void);


#if (TRACER_PRESENT && TRC_PROG)
/**
 ****************************************************************************************
 * @brief Trace request API for exchange table programming
 *
 * This function is called when the exchange table is programmed.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "programming" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   elt_idx     Index of the programmed entry in the ET
 ****************************************************************************************
 */
void dbg_trc_req_et_prog(uint8_t const elt_idx);

#define TRC_REQ_ET_PROG(elt_idx) \
        dbg_trc_req_et_prog(elt_idx);

/**
 ****************************************************************************************
 * @brief Trace request API for connection event counter programming
 *
 * This function is called when the connection event counter is set.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "programming" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   conhdl       Connection handle
 * @param[in]   value        Value of connection event counter
 ****************************************************************************************
 */
void dbg_trc_req_evt_cnt(uint16_t const conhdl, uint16_t const value);
#define TRC_REQ_EVT_CNT(conhdl, value) \
        dbg_trc_req_evt_cnt(conhdl, value);
/**
 ****************************************************************************************
 * @brief Trace request API for frame completion
 *
 * This function is called when a frame is completed.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "programming" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   timestamp   Timestamp of the programmed frame (in BLE slots, based on local clock)
 * @param[in]   frm_id      Frame identifier
 * @param[in]   frm_cbk     Callback for handling interrupts related to the frame
 ****************************************************************************************
 */
void dbg_trc_req_frm_cmp(uint32_t const timestamp, uint8_t const frm_id, uint32_t const frm_cbk);

#define TRC_REQ_FRM_CMP(timestamp, frm_id, frm_cbk) \
        dbg_trc_req_frm_cmp(timestamp, frm_id, frm_cbk);

#else /*(TRACER_PRESENT && TRC_PROG)*/
#define TRC_REQ_ET_PROG(elt_idx)
#define TRC_REQ_EVT_CNT(conhdl, value)
#define TRC_REQ_FRM_CMP(timestamp, frm_id, frm_cbk)
#endif /*(TRACER_PRESENT && TRC_PROG)*/

#if (TRACER_PRESENT && TRC_CS_BLE)
/**
 ****************************************************************************************
 * @brief Trace request API for control structure programming in BLE
 *
 * This function is called when a control structure is modified.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "BLE control structure" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   cs_addr     Address of the control structure in the exchange memory
 ****************************************************************************************
 */
void dbg_trc_req_cs_ble(uint16_t const cs_addr);

#define TRC_REQ_CS_BLE(cs_addr) \
        dbg_trc_req_cs_ble(cs_addr);

#else /*(TRACER_PRESENT && TRC_CS_BLE)*/
#define TRC_REQ_CS_BLE(cs_addr)
#endif /*(TRACER_PRESENT && TRC_CS_BLE)*/

#if (TRACER_PRESENT && TRC_CS_BT)
/**
 ****************************************************************************************
 * @brief Trace request API for control structure programming in BT
 *
 * This function is called when a control structure is modified.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "BT control structure" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   cs_addr     Address of the control structure in the exchange memory
 ****************************************************************************************
 */
void dbg_trc_req_cs_bt(uint16_t const cs_addr);

#define TRC_REQ_CS_BT(cs_addr) \
        dbg_trc_req_cs_bt(cs_addr);
#else /*(TRACER_PRESENT && TRC_CS_BT)*/
#define TRC_REQ_CS_BT(cs_addr)
#endif /*(TRACER_PRESENT && TRC_CS_BT)*/

#if (TRACER_PRESENT && TRC_RX_DESC)
/**
 ****************************************************************************************
 * @brief Trace request API for processing of rx descriptors
 *
 * This function is called when a packet is received and a driver processes a rx descriptor.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "Rx descriptor" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   drv_type    LLD driver (@see enum driver_type)
 * @param[in]   act_id      Activity identifier
 * @param[in]   rx_desc_add Address of the rx descriptor in the exchange memory
 ****************************************************************************************
 */
void dbg_trc_req_rx_desc(uint8_t const drv_type, uint8_t const act_id, uint16_t const rx_desc_add);

#define TRC_REQ_RX_DESC(drv_type, act_id, rx_desc_add) \
        dbg_trc_req_rx_desc(drv_type, act_id, rx_desc_add);

#else /*(TRACER_PRESENT && TRC_RX_DESC)*/
#define TRC_REQ_RX_DESC(drv_type, act_id, rx_desc_add)
#endif /*(TRACER_PRESENT && TRC_RX_DESC) */

#if (TRACER_PRESENT && TRC_LLCP)
/**
 ****************************************************************************************
 * @brief Trace request API for llcp packets transmission, reception and acknowledgment
 *
 * This function is called when a llcp packet is transmitted, received or acknowledged.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "LLCP" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   llcp_trc_type   Trace type for llcp (llcp tx, llcp rx and llcp ack of @see enum trace_types)
 * @param[in]   conhdl        Connection handle
 * @param[in]   data_len        Data length (in bytes)
 * @param[in]   pdu_data        PDU Data
 ****************************************************************************************
 */
void dbg_trc_req_llcp(uint8_t const llcp_trc_type, uint16_t const conhdl, uint8_t const data_len, uint8_t const *pdu_data);

/// Macro used to send trace request API when an llcp packet is transmitted
#define TRC_REQ_LLCP_TX(conhdl, data_len, pdu_data)               \
        dbg_trc_req_llcp(LLCP_TX, conhdl, data_len, pdu_data);

/// Macro used to send trace request API when an llcp packet is received
#define TRC_REQ_LLCP_RX(conhdl, data_len, pdu_data)               \
        dbg_trc_req_llcp(LLCP_RX, conhdl, data_len, pdu_data);

/// Macro used to send trace request API when an llcp packet is acknowledged
#define TRC_REQ_LLCP_ACK(conhdl, data_len, pdu_data)              \
        dbg_trc_req_llcp(LLCP_ACK, conhdl, data_len, pdu_data);
#else /*(TRACER_PRESENT && TRC_LLCP)*/
#define TRC_REQ_LLCP_TX(conhdl, data_len, pdu_data)
#define TRC_REQ_LLCP_RX(conhdl, data_len, pdu_data)
#define TRC_REQ_LLCP_ACK(conhdl, data_len, pdu_data)
#endif /*(TRACER_PRESENT && TRC_LLCP) */

#if (TRACER_PRESENT && TRC_LMP)
/**
 ****************************************************************************************
 * @brief Trace request API for lmp packets transmission, reception and acknowledgment
 *
 * This function is called when a lmp packet is transmitted, received or acknowledged.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "LMP" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   lmp_trc_type    Trace type for lmp (lmp tx, lmp rx and lmp ack of @see enum trace_types)
 * @param[in]   conhdl          Connection handle
 * @param[in]   data_len        Data length (in bytes)
 * @param[in]   pdu_data        PDU Data
 ****************************************************************************************
 */
void dbg_trc_req_lmp(uint8_t const lmp_trc_type, uint16_t const conhdl, uint8_t const data_len, uint8_t const *pdu_data);
/// same as lmp for lmp_ack
void dbg_trc_req_lmp_ack(uint8_t const lmp_trc_type, uint16_t const conhdl, uint8_t const opcode, uint8_t const ext_code);

/// Macro used to send trace request API when an lmp packet is transmitted
#define TRC_REQ_LMP_TX(conhdl, data_len, pdu_data)               \
        dbg_trc_req_lmp(LMP_TX, conhdl, data_len, pdu_data);

/// Macro used to send trace request API when an lmp packet is received
#define TRC_REQ_LMP_RX(conhdl, data_len, pdu_data)               \
        dbg_trc_req_lmp(LMP_RX, conhdl, data_len, pdu_data);

/// Macro used to send trace request API when an lmp packet is acknowledged
#define TRC_REQ_LMP_ACK(conhdl, opcode, ext_code)              \
        dbg_trc_req_lmp_ack(LMP_ACK, conhdl, opcode, ext_code);

#else /*(TRACER_PRESENT && TRC_LMP)*/
#define TRC_REQ_LMP_TX(conhdl, data_len, pdu_data)
#define TRC_REQ_LMP_RX(conhdl, data_len, pdu_data)
#define TRC_REQ_LMP_ACK(conhdl, opcode, ext_code)
#endif /*(TRACER_PRESENT && TRC_LMP)*/

#if (TRACER_PRESENT && TRC_L2CAP)
/**
 ****************************************************************************************
 * @brief Trace request API for l2cap packets transmission and reception
 *
 * This function is called when a l2cap packet is transmitted or received.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "L2CAP" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   l2cap_trc_type  Trace type for l2cap (l2cap tx and l2cap rx of @see enum trace_types)
 * @param[in]   conhdl        Connection handle
 * @param[in]   data_len        Data length (in bytes)
 * @param[in]   buf_ptr         Memory pointer address
 ****************************************************************************************
 */
void dbg_trc_req_l2cap(uint8_t const l2cap_trc_type, uint16_t const conhdl, uint16_t const data_len, uint32_t const buf_ptr);

/// Macro used to send trace request API when an l2cap packet is transmitted
#define TRC_REQ_L2CAP_TX(conhdl, tx_len, buf_ptr)               \
        dbg_trc_req_l2cap(L2CAP_TX, conhdl, tx_len, buf_ptr);

/// Macro used to send trace request API when an l2cap packet is received
#define TRC_REQ_L2CAP_RX(conhdl, tx_len, buf_ptr)               \
        dbg_trc_req_l2cap(L2CAP_RX, conhdl, tx_len, buf_ptr);

/**
 ****************************************************************************************
 * @brief Trace request API for l2cap packet acknowledgment
 *
 * This function is called when l2cap packets are acknowledged.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "L2CAP" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   conhdl        Connection handle
 * @param[in]   num_ack_pkts    Number of acknowledged l2cap packets
 ****************************************************************************************
 */
void dbg_trc_req_l2cap_ack(uint16_t const conhdl, uint16_t const num_ack_pkts);

#define TRC_REQ_L2CAP_ACK(conhdl, num_ack_pkts)               \
        dbg_trc_req_l2cap_ack(conhdl, num_ack_pkts);

#else /*(TRACER_PRESENT && TRC_L2CAP)*/
#define TRC_REQ_L2CAP_TX(conhdl, tx_len, buf_ptr)
#define TRC_REQ_L2CAP_RX(conhdl, tx_len, buf_ptr)
#define TRC_REQ_L2CAP_ACK(conhdl, num_ack_pkts)
#endif/*(TRACER_PRESENT && TRC_L2CAP) */

#if (TRACER_PRESENT && TRC_ARB)
/**
 ****************************************************************************************
 * @brief Trace request API for cancellation, removal and starting of events
 *
 * This function is called when an event starts, is removed or is cancelled.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "EA" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   sch_arb_trc_type     Trace type for Scheduling Arbiter (cancel, remove and start of @see enum trace_types)
 * @param[in]   sch_arb_elt          Pointer to the Scheduling Arbiter element
 ****************************************************************************************
 */
void dbg_trc_req_sch_arb(uint8_t const sch_arb_trc_type, const struct sch_arb_elt_tag *sch_arb_elt);

/// Macro used to send trace request API when an event is cancelled
#define TRC_REQ_SCH_ARB_CANC(sch_arb_elt)                 \
        dbg_trc_req_sch_arb(SCH_ARB_CANC, sch_arb_elt);

/// Macro used to send trace request API when an event is removed
#define TRC_REQ_SCH_ARB_REM(sch_arb_elt)                  \
        dbg_trc_req_sch_arb(SCH_ARB_REM, sch_arb_elt);

/// Macro used to send trace request API when an event starts
#define TRC_REQ_SCH_ARB_START(sch_arb_elt)                \
        dbg_trc_req_sch_arb(SCH_ARB_START, sch_arb_elt);

/**
 ****************************************************************************************
 * @brief Trace request API for shifting of events
 *
 * This function is called when an event is shifted.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "EA" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   sch_arb_elt          Pointer to the Scheduling Arbiter element
 ****************************************************************************************
 */
void dbg_trc_req_sch_arb_shift(const struct sch_arb_elt_tag *sch_arb_elt);

#define TRC_REQ_SCH_ARB_SHIFT(sch_arb_elt)                \
        dbg_trc_req_sch_arb_shift(sch_arb_elt);
/**
 ****************************************************************************************
 * @brief Trace request API for pushing of event in the waiting queue
 *
 * This function is called when an event is pushed in the waiting queue.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "EA" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   sch_arb_elt          Pointer to the Scheduling Arbiter element
 ****************************************************************************************
 */
void dbg_trc_req_sch_arb_insert(uint32_t const target_time, uint8_t const status, const struct sch_arb_elt_tag *sch_arb_elt);

#define TRC_REQ_SCH_ARB_INSERT(target_time, status, sch_arb_elt)        \
        dbg_trc_req_sch_arb_insert(target_time, status, sch_arb_elt);

#else /*(TRACER_PRESENT && TRC_ARB)*/
#define TRC_REQ_SCH_ARB_CANC(sch_arb_elt)
#define TRC_REQ_SCH_ARB_REM(sch_arb_elt)
#define TRC_REQ_SCH_ARB_START(sch_arb_elt)
#define TRC_REQ_SCH_ARB_SHIFT(sch_arb_elt)
#define TRC_REQ_SCH_ARB_INSERT(target_time, status, sch_arb_elt)
#endif /*(TRACER_PRESENT && TRC_ARB)*/

#if (TRACER_PRESENT && TRC_LC_STATE_TRANS)
/**
 ****************************************************************************************
 * @brief Trace request API for lc procedure state transition
 *
 * This function is called when an lc procedure changes its state.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "LC state transition" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   conhdl        Connection handle
 * @param[in]   proc_type       LC procedure
 * @param[in]   prev_state      Previous procedure state
 * @param[in]   curr_state      Current procedure state
 ****************************************************************************************
 */
void dbg_trc_req_lc_state_trans(uint16_t const conhdl, uint8_t const proc_type, uint8_t const prev_state, uint8_t const curr_state);

#define TRC_REQ_LC_STATE_TRANS(conhdl, proc_type, prev_state, curr_state)       \
        dbg_trc_req_lc_state_trans(conhdl, proc_type, prev_state, curr_state);

#else /*(TRACER_PRESENT && TRC_LC_STATE_TRANS)*/
#define TRC_REQ_LC_STATE_TRANS(conhdl, proc_type, prev_state, curr_state)
#endif /*(TRACER_PRESENT && TRC_LC_STATE_TRANS)*/

#if (TRACER_PRESENT && TRC_LLC_STATE_TRANS)
/**
 ****************************************************************************************
 * @brief Trace request API for llc procedure state transition
 *
 * This function is called when an llcp procedure changes its state.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "LLC state transition" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   conhdl          Connection handle
 * @param[in]   proc_type       LLC procedure
 * @param[in]   prev_state      Previous procedure state
 * @param[in]   curr_state      Current procedure state
 ****************************************************************************************
 */
void dbg_trc_req_llc_state_trans(uint16_t const conhdl, uint8_t const proc_type, uint8_t const prev_state, uint8_t const curr_state);

#define TRC_REQ_LLC_STATE_TRANS(conhdl, proc_type, prev_state, curr_state)      \
        dbg_trc_req_llc_state_trans(conhdl, proc_type, prev_state, curr_state);

#else /*(TRACER_PRESENT && TRC_LLC_STATE_TRANS)*/
#define TRC_REQ_LLC_STATE_TRANS(conhdl, proc_type, prev_state, curr_state)
#endif /*(TRACER_PRESENT && TRC_LLC_STATE_TRANS)*/

#if (TRACER_PRESENT && TRC_HCI)
/**
 ****************************************************************************************
 * @brief Trace request API for HCI messages (in FE mode)
 *
 * This function is called when a HCI command is received.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "HCI" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   opcode          HCI opcode
 * @param[in]   par_len         HCI parameter length
 * @param[in]   params          unpacked HCI parameters
 ****************************************************************************************
 */
void dbg_trc_req_hci_cmd(uint16_t const opcode, uint8_t const par_len, uint32_t const *params);

/**
 ****************************************************************************************
 * @brief Trace request API for HCI messages (in FE mode)
 *
 * This function is called when a HCI event is sent.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "HCI" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   hci_evt_type    Type of hci event (@see enum trace_types)
 * @param[in]   opcode          HCI opcode
 * @param[in]   par_len         HCI parameter length
 * @param[in]   params          unpacked HCI parameters
 ****************************************************************************************
 */
void dbg_trc_req_hci_evt(uint8_t const hci_evt_type, uint8_t const opcode, uint8_t const par_len, uint32_t const *params);
void dbg_trc_req_hci_acl_data(uint8_t in, uint16_t const handle, uint16_t const par_len, uint32_t const *params);

/// Macro used to send trace request API when a HCI command is received
#define TRC_REQ_HCI_CMD(opcode, par_len, params)    \
    dbg_trc_req_hci_cmd(opcode, (uint8_t)par_len, (uint32_t const *)params);

/// Macro used to send trace request API when a HCI command status event is sent
#define TRC_REQ_HCI_CMD_STAT_EVT(opcode, par_len, params)    \
    dbg_trc_req_hci_evt((uint8_t)H4TL_PACKET_CTRL, (uint8_t)opcode, (uint8_t)par_len, (uint32_t const *)params);

/// Macro used to send trace request API when a HCI command complete event is sent
#define TRC_REQ_HCI_CMD_CMP_EVT(opcode, par_len, params)    \
    dbg_trc_req_hci_evt((uint8_t)H4TL_PACKET_CTRL, (uint8_t)opcode, (uint8_t)par_len, (uint32_t const *)params);

/// Macro used to send trace request API when a HCI event is sent
#define TRC_REQ_HCI_EVT(opcode, par_len, params)    \
    dbg_trc_req_hci_evt((uint8_t)H4TL_PACKET_CTRL, (uint8_t)opcode, (uint8_t)par_len, (uint32_t const *)params);

/// Macro used to send trace request API when a HCI LE event is sent
#define TRC_REQ_HCI_LE_EVT(opcode, par_len, params)    \
    dbg_trc_req_hci_evt((uint8_t)H4TL_PACKET_CTRL, (uint8_t) opcode, (uint8_t)par_len, (uint32_t const *)params);


#define TRC_REQ_HCI_ACL_DATA(in, handle, par_len, params) \
    dbg_trc_req_hci_acl_data((uint8_t)in, handle, par_len, (uint32_t const *)params);

#else /*(TRACER_PRESENT && TRC_HCI)*/
#define TRC_REQ_HCI_CMD(opcode, par_len, params)
#define TRC_REQ_HCI_CMD_STAT_EVT(opcode, par_len, params)
#define TRC_REQ_HCI_CMD_CMP_EVT(opcode, par_len, params)
#define TRC_REQ_HCI_EVT(opcode, par_len, params)
#define TRC_REQ_HCI_LE_EVT(opcode, par_len, params)
#define TRC_REQ_HCI_ACL_DATA(in, handle, par_len, params)
#endif /*(TRACER_PRESENT && TRC_HCI)*/

#if (TRACER_PRESENT && TRC_ADV)
/*@TRACE*/
enum trc_addr_type
{
    PUBLIC_DEVICE_ADDRESS = 0x00,
    RANDOM_DEVICE_ADDRESS = 0x01,
    PUBLIC_IDENTITY_ADDRESS = 0x02,
    RANDOM_IDENTITY_ADDRESS = 0x03,
    RESERVED = 0x04
};

//struct trc_scan_req
//{
//    /// Scanner address type
//    //@trc_ref addr_type
//    uint8_t scan_addr_type;
//
//    /// Scanner address
//    struct bd_addr scan_addr;
//
//    /// Scan request
//    struct scan_req data;
//};
//
//struct trc_connect_req
//{
//    /// Initiator address type
//    //@trc_ref addr_type
//    uint8_t init_addr_type;
//
//    /// Initiator address
//    struct bd_addr init_addr;
//
//    /// Connect request
//    struct connect_req data;
//};
//
//  @trc_ref ble_adv_type
// */
//union trc_adv_rx_payload
//{
//    //@trc_union packet_type == BLE_CONNECT_IND
//    struct trc_connect_req conn_req;
//
//    //@trc_union packet_type == BLE_SCAN_REQ
//    struct trc_scan_req scan_req;
//};
//
//struct trc_adv_rx_pdu_hdr
//{
//    ///Packet type
//    //@trc_ref ble_adv_type
//    uint8_t packet_type;
//
//    ///Channel used
//    uint8_t used_channel;
//};
//
//struct trc_adv_rx_pdu
//{
//    ///Header
//    struct trc_adv_rx_pdu_hdr header;
//
//    ///Payload
//    union trc_adv_rx_payload payload;
//};

/*@TRACE*/
struct trc_scan_rx_pdu
{
    ///Packet type
    //@trc_ref ble_adv_type
    uint8_t packet_type;

    ///Channel used
    uint8_t used_channel;

    /// Scanner device address
    struct bd_addr scanner_addr;

    /// Scanner address type
    //@trc_ref addr_type
    uint8_t scanner_addr_type;

    /// Data length in bytes
    uint8_t data_len;

    /// Data
    uint8_t data[__ARRAY_EMPTY];
};


/*@TRACE*/
struct trc_init_rx_pdu
{
    ///Packet type
    //@trc_ref ble_adv_type
    uint8_t packet_type;

    ///Channel used
    uint8_t used_channel;

    /// Advertiser device address
    struct bd_addr adv_addr;

    /// Peer device address type
    //@trc_ref addr_type
    uint8_t adv_addr_type;
};


/*@TRACE*/
struct trc_adv_tx_pdu
{
    ///Packet type
    //@trc_ref ble_adv_type
    uint8_t packet_type;

    ///Advertising channel map
    uint8_t advertising_channel_map;

    /// BD Address of the advertiser
    struct bd_addr adv_addr;
};

/*@TRACE*/
struct trc_init_tx_pdu
{
    ///Packet type
    //@trc_ref ble_adv_type
    uint8_t packet_type;

    ///Advertising channel map
    uint8_t advertising_channel_map;

    /// Peer device address
    struct bd_addr peer_id_addr;
};

void dbg_trc_req_init_rx_pdu(const uint8_t index_pkt, uint16_t rxdataptr);
void dbg_trc_req_init_tx_pdu(const uint8_t index_pkt, struct bd_addr const addr);
void dbg_trc_req_adv_rx_pdu(const uint8_t index_pkt, const uint8_t data_len, void const *data);
void dbg_trc_req_adv_tx_pdu(const uint8_t adv_txdesc_idx, const uint8_t cs_idx);
void dbg_trc_req_scan_rx_pdu(const uint8_t index_pkt, struct lld_adv_rep_ind const *ind);
void dbg_trc_req_scan_tx_pdu(const uint8_t txdesc_idx, const uint8_t cs_idx);

#define TRC_REQ_INIT_RX_PDU(index_pkt, addr)      \
    dbg_trc_req_init_rx_pdu(index_pkt, addr);
#define TRC_REQ_INIT_TX_PDU(index_pkt, addr)         \
    dbg_trc_req_init_tx_pdu(index_pkt, addr);

#define TRC_REQ_ADV_RX_PDU(index_pkt, data_len, data)       \
    dbg_trc_req_adv_rx_pdu(index_pkt, data_len, data);
#define TRC_REQ_ADV_TX_PDU(adv_txdesc_idx, cs_idx)  \
    dbg_trc_req_adv_tx_pdu(adv_txdesc_idx, cs_idx);

#define TRC_REQ_SCAN_RX_PDU(pkt_type, ind)  \
    dbg_trc_req_scan_rx_pdu(pkt_type, ind);
#define TRC_REQ_SCAN_TX_PDU(txdesc_idx, cs_idx)   \
    dbg_trc_req_scan_tx_pdu(txdesc_idx, cs_idx);

#else
#define TRC_REQ_INIT_RX_PDU(index_pkt, addr)
#define TRC_REQ_INIT_TX_PDU(txdesc_idx, cs_idx)
#define TRC_REQ_ADV_RX_PDU(index_pkt, addr, addr_type)
#define TRC_REQ_ADV_TX_PDU(adv_txdesc_idx, cs_idx)
#define TRC_REQ_SCAN_RX_PDU(pkt_type, ind)
#define TRC_REQ_SCAN_TX_PDU(txdesc_idx, cs_idx)
#endif /*(TRACER_PRESENT && TRC_ADV)*/

#if (TRACER_PRESENT && TRC_ACL)
/*@TRACE*/
struct trc_acl_tx_pdu
{
    uint16_t txdataptr;

    ///Data length
    uint16_t data_len;

    ///Data
    uint8_t data[__ARRAY_EMPTY];
};

/*@TRACE*/
struct trc_acl_rx_pdu
{
    ///Data length
    uint16_t data_len;

    ///Data
    uint8_t data[__ARRAY_EMPTY];
};
struct lld_acl_rx_ind;

void dbg_trc_req_acl_rx_pdu(struct lld_acl_rx_ind const *msg);
void dbg_trc_req_acl_tx_pdu(const uint8_t trc_acl_tx_type, const uint8_t txdesc_idx);

#define TRC_REQ_ACL_RX_PDU(msg)     \
        dbg_trc_req_acl_rx_pdu(msg);

#define TRC_REQ_ACL_ACK_TX_PDU(txdesc_idx)     \
        dbg_trc_req_acl_tx_pdu(ACL_ACK_TX, txdesc_idx);

#define TRC_REQ_ACL_PROG_TX_PDU(txdesc_idx)     \
        dbg_trc_req_acl_tx_pdu(ACL_PROG_TX, txdesc_idx);

#else
#define TRC_REQ_ACL_RX_PDU(msg)
#define TRC_REQ_ACL_PROG_TX_PDU(txdesc_idx)
#define TRC_REQ_ACL_ACK_TX_PDU(txdesc_idx)
#endif /*(TRACER_PRESENT && TRC_ACL)*/

#if (TRACER_PRESENT && TRC_CUSTOM)
/**
 ****************************************************************************************
 * @brief Trace request API for custom traces
 *
 * This function is called to send a custom trace.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "custom" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   trc_id      Trace request identifier (progressive number that identifies the caller)
 * @param[in]   data_type   Type of data (@see enum custom_trace_types)
 * @param[in]   data        Pointer to the data to send
 ****************************************************************************************
 */
void dbg_trc_req_custom(const uint8_t trc_id, const uint8_t data_type, void const *data);
#define TRC_REQ_CUSTOM(trc_id, data_type, data)    \
        dbg_trc_req_custom(trc_id, data_type, data);
#else /*(TRACER_PRESENT && TRC_CUSTOM)*/
#define TRC_REQ_CUSTOM(trc_id, data_type, data)
#endif /*(TRACER_PRESENT && TRC_CUSTOM)*/


#if (TRACER_PRESENT)
/**
 ****************************************************************************************
 * @brief Trace request API for kernel message pushed in the send queue
 *
 * This function is called when a kernel message is pushed in the send queue.
 *
 * It checks the current tracer configuration and writes the trace in memory if the trace
 * group "kernel message" (@see enum tracer_cfg_fields) is enabled.
 *
 * @param[in]   msg     Kernel message
 ****************************************************************************************
 */
void dbg_trc_printf(const char *fmt, ...);

/// Macro used to send trace request API for kernel message handled
//#define TRC_PRINTF(...) dbg_trc_printf(__VA_ARGS__)
// Yier, need enable when debug tool optimized
#define TRC_PRINTF(...)


void dbg_trc_string(const char *str);

/// Macro used to send trace request API for kernel message handled
#define TRC_STRING(str) dbg_trc_string(str)


void dbg_trc_binary(uint8_t trace_code, uint8_t *data, uint32_t data_size);

#define TRC_BINARY(trace_code, data, data_size) dbg_trc_binary(trace_code, data, data_size)


void dbg_trc_send_shell_response(const char *fmt, ...);

/// Macro used to send trace request API for kernel message handled
#define TRC_SEND_SHELL_RESPONSE(...) dbg_trc_send_shell_response(__VA_ARGS__)

void dbg_trc_integer(uint32_t data[], uint32_t num);

#define TRC_INTEGER(data, num) dbg_trc_integer(data, num)

void dbg_trc_integer_1(uint32_t data);

#define TRC_INTEGER_1(data) dbg_trc_integer_1(data)

void dbg_trc_integer_2(uint32_t data1, uint32_t data2);

#define TRC_INTEGER_2(data1, data2) dbg_trc_integer_2(data1, data2)


/*@TRACE*/
typedef struct trc_crash_dump
{
    uint32_t mem_addr;
    uint32_t mem_size;
    uint16_t blk_num;
    uint16_t blk_idx;
    uint16_t blk_len;
    uint16_t padding;
} trc_crash_dump_t;




#else /*TRACER_PRESENT && TRC_KE_MSG*/
#define TRC_PRINTF(...)
#define TRC_BINARY(trace_code, data, data_size)
#define TRC_STRING(str)
#endif /*TRACER_PRESENT && TRC_KE_MSG*/




#if (TRACER_PRESENT)
    /**
    ****************************************************************************************
    * @brief Configuration received
    *
    * This function checks if the received configuration can be adopted and acknowledges it
    *
    ****************************************************************************************
    */
    void dbg_trc_cfg_received(void);
    void dbg_trc_cfg_received_via_hci(uint32_t cfg);


    /**
    ****************************************************************************************
    * @brief Get the reception buffer for configuration word
    *
    * @return  Buffer for payload reception of tracer configuration packets
    ****************************************************************************************
    */
    uint8_t *dbg_trc_pay_buff_get(void);

    int dbg_hci_build(int type, uint8_t *param);
    void dbg_hci_trc_write(int type, uint8_t *trace, uint16_t payload_len);

#endif /*(TRACER_PRESENT)*/
///@} TRACER
#endif // DBG_TRC_H_
