/**
 ****************************************************************************************
 *
 * @file lc_task.c
 *
 * @brief LC task source file
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include "ke_mem.h"         // kernel memory

#include <string.h>
#include "co_bt.h"          // BT standard definitions
#include "co_endian.h"
#include "co_utils.h"
#include "co_math.h"
#include "co_version.h"
#include "reg_access.h"

#include "lc.h"             // link controller definitions
#include "lc_int.h"         // link controller internal definitions
#include "lc_sco.h"         // link controller SCO definitions
#include "lc_sniff.h"       // link controller sniff definitions
#include "lc_lmppdu.h"
#include "lc_util.h"
#if (MAX_NB_SYNC > 0)
    #include "lm_sco.h"             // lm sco definitions
#endif //(MAX_NB_SYNC > 0)

#include "ld.h"             // link driver
#include "lm.h"                    // lm definitions
#include "lb.h"                    // lb definitions
#include "bt_util_key.h"

#include "ke_timer.h"
#include "ecc_p256.h"
#include "ecc_p192.h"

#include "rwip.h"           // stack main module

#include "sch_plan.h"       // SCH_PLAN API

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT

#include "dbg.h"
#include "_reg_em_et.h"
#include "btdm_patch.h"
#include "sha.h"

#if EAVESDROPPING_SUPPORT
    #include "ed.h"
#endif // EAVESDROPPING_SUPPORT
/*
 * DEFINES
 ****************************************************************************************
 */

/// Simple pairing method
#define SP_JUST_WORKS                    0x00
#define SP_NUM_COMP                      0x01
#define SP_PASSKEY                       0x02
#define SP_OOB                           0x03

/// Macro for building an LMP descriptor
#define LMP(name_uc, name_lc, estab, req, fmt) \
    [LMP_##name_uc##_OPCODE] = {(lc_lmp_msg_hdl_func_t) lmp_##name_lc##_handler, fmt, LMP_##name_uc##_LEN | (estab << LMP_ESTAB_POS) | (req << LMP_REQ_POS)}

/// Macro for building an LMP descriptor
#define LMP_EXT(name_uc, name_lc, estab, req, fmt) \
    [LMP_##name_uc##_EXTOPCODE] = {(lc_lmp_msg_hdl_func_t) lmp_##name_lc##_handler, fmt, LMP_##name_uc##_LEN | (estab << LMP_ESTAB_POS) | (req << LMP_REQ_POS)}


/*
 * ENUMERATIONS DEFINITION
 ****************************************************************************************
 */

/**
 * LMP information bit field
 *
 *    7        6     5     4  ..  0
 * +-------+-------+----+--------------+
 * | ESTAB |  REQ  |    |    LENGTH    |
 * +-------+-------+----+--------------+
 */
enum lmp_info_fields
{
    /// LMP parameters length (in bytes)
    LMP_LEN_LSB       = 0,
    LMP_LEN_MASK      = 0x1F,

    /// LMP request, requires a response
    LMP_REQ_POS       = 6,
    LMP_REQ_BIT       = 0x40,

    /// LMP allowed at connection establishment
    LMP_ESTAB_POS     = 7,
    LMP_ESTAB_BIT     = 0x80,
};


/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Format of a HCI command handler function
typedef int (*lc_hci_cmd_hdl_func_t)(void const *param, ke_task_id_t const dest_id, uint16_t opcode);


/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Element of a HCI command handler table.
struct lc_hci_cmd_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    lc_hci_cmd_hdl_func_t func;
};


bt_sco_data_handle_t bt_sco_data_handle_fun = NULL;
void bt_sco_data_handle_register(bt_sco_data_handle_t fun)
{
    bt_sco_data_handle_fun = fun;
}
__STATIC void bt_sco_mem_free(void *p_param)
{
    struct ke_msg *p_msg = ke_param2msg(p_param);

    // Free the kernel message space
    ke_msg_free(p_msg);
    return;
}
static struct hci_sync_con_cmp_evt g_lc_sco_para;
void lc_save_sco_para(struct hci_sync_con_cmp_evt *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LC_TASK_PATCH_TYPE, LC_SAVE_SCO_PARA_FUN_BIT, evt);
    //rt_sem_take(&g_sco_para_lock,RT_WAITING_FOREVER);
    memcpy(&g_lc_sco_para, evt, sizeof(g_lc_sco_para));
    //rt_sem_release(&g_sco_para_lock);
}
__STATIC uint16_t lc_get_sco_conhdl()
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LC_TASK_PATCH_TYPE, LC_GET_SCO_CONHDL_FUN_BIT, uint16_t);
    uint16_t conhdl;

    //rt_sem_take(&g_sco_para_lock,RT_WAITING_FOREVER);
    conhdl = g_lc_sco_para.conhdl;
    //rt_sem_release(&g_sco_para_lock);

    return conhdl;
}
struct hci_sync_con_cmp_evt lc_get_sco_para()
{
    return g_lc_sco_para;
}
void lc_save_sco_chg_para(struct hci_sync_con_chg_evt *event)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LC_TASK_PATCH_TYPE, LC_SAVE_SCO_CHG_PARA_FUN_BIT, event);

    g_lc_sco_para.conhdl = event->sync_conhdl;
    g_lc_sco_para.tx_int = event->tx_int;
    g_lc_sco_para.ret_win = event->ret_win;
    g_lc_sco_para.tx_pkt_len = event->tx_pkt_len;
    g_lc_sco_para.rx_pkt_len = event->rx_pkt_len;
}



uint8_t lc_bt_sco_data_recv(struct lc_sco_data_tag *p_sco_para)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LC_TASK_PATCH_TYPE, LC_BT_SCO_DATA_RECV_FUN_BIT, uint8_t, p_sco_para);
    uint16_t conhdl;
    uint8_t *p_buf;
    uint8_t sync_link_id;
    uint16_t acl_handle;
    struct hci_sync_data *data_tx;
    uint8_t ret = 0;

    conhdl = lc_get_sco_conhdl();

    sync_link_id = BT_SYNC_CONHDL_LID(conhdl);

    if (sync_link_id < MAX_NB_SYNC)
    {
        // Try allocating a buffer from BT pool
        p_buf = (uint8_t *)(uint32_t) bt_util_buf_sync_tx_alloc(sync_link_id, p_sco_para->length);

        if (p_buf != NULL)
        {
            memcpy((void *)((uint32_t)p_buf + REG_EM_ET_BASE_ADDR), (void *)(p_sco_para->data_ptr), p_sco_para->length);

            acl_handle = conhdl & ~(BT_SYNC_CONHDL_MSK);

            // Sends the Kernel message (with space for unpacked parameters)
            data_tx = KE_MSG_ALLOC(HCI_SYNC_DATA, KE_BUILD_ID(TASK_LC, acl_handle - BT_ACL_CONHDL_MIN), 0, hci_sync_data);
            data_tx->buf_ptr = (uint16_t)(p_buf);
            data_tx->conhdl_psf = conhdl;
            data_tx->length = p_sco_para->length;
            ke_msg_send(data_tx);
        }
        else
        {
            ret = 1;
        }
    }
    else
    {
        ret = 2;
    }

    return ret;
}
//extern uint16_t u16_voice_numer;
#if 0
__STATIC void lc_bt_sco_data_send(struct hci_sync_data *data_rx)
{
    int ret;
    struct lc_sco_data_tag sco_para;

    if (bt_sco_data_handle_fun)  // host callback function
    {
        sco_para.length = data_rx->length;
        sco_para.data_ptr = (uint8_t *)(REG_EM_ET_BASE_ADDR + (uint32_t)data_rx->buf_ptr);
        //sco_para.free_ptr = (void *)data_rx;
        ret = bt_sco_data_handle_fun(&sco_para);
        bt_sco_mem_free(data_rx);

        // Free the RX buffer associated with the message
        bt_util_buf_sync_rx_free(BT_SYNC_CONHDL_LID(GETF(data_rx->conhdl_psf, HCI_SYNC_HDR_HDL)), data_rx->buf_ptr);
        //count packets for flow control
        //hci_fc_sync_packet_sent();
    }
    else
    {
        hci_send_2_host(data_rx);
    }
    //rt_kprintf("sco_data:u16_voice_numer=%d\n",u16_voice_numer);
}
#else
__STATIC void lc_bt_sco_data_send(struct hci_sync_data *data_rx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LC_TASK_PATCH_TYPE, LC_BT_SCO_DATA_SEND_FUN_BIT, data_rx);
    int ret;
    bt_sco_callback_para_t sco_callback_para;
    struct lc_sco_data_tag *sco_para;

    if (bt_sco_data_handle_fun)  // host callback function
    {

        sco_callback_para.handle_type = AUDIO_PATH_SCO_TX_DATA;
        sco_para = &sco_callback_para.un_para.sco_data;
        sco_para->length = data_rx->length;
        sco_para->packet_status = GETF(data_rx->conhdl_psf, HCI_SYNC_HDR_PSF);
        sco_para->data_ptr = (uint8_t *)(REG_EM_ET_BASE_ADDR + (uint32_t)data_rx->buf_ptr);
        //sco_para.free_ptr = (void *)data_rx;
        ret = bt_sco_data_handle_fun(&sco_callback_para);
        //bt_sco_mem_free(data_rx);

        // Free the RX buffer associated with the message
        bt_util_buf_sync_rx_free(BT_SYNC_CONHDL_LID(GETF(data_rx->conhdl_psf, HCI_SYNC_HDR_HDL)), data_rx->buf_ptr);
        bt_sco_mem_free(data_rx);
        //count packets for flow control
        //hci_fc_sync_packet_sent();
    }
    else
    {
        hci_send_2_host(data_rx);
    }
    //rt_kprintf("sco_data:u16_voice_numer=%d\n",u16_voice_numer);
}
__STATIC void lc_bt_send_data_done(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LC_TASK_PATCH_TYPE, LC_BT_SEND_DATA_DONE_FUN_BIT);
    int ret;
    bt_sco_callback_para_t sco_callback_para;

    //rt_kprintf("lc_bt_send_data_done\n");

    if (bt_sco_data_handle_fun)  // host callback function
    {
        sco_callback_para.handle_type = AUDIO_PATH_SCO_RX_DATA;
        ret = bt_sco_data_handle_fun(&sco_callback_para);

    }
}

#endif



/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is used to get a piece of the local P192 public key for encapsulated PDU.
 *
 * @param[out] EncapDataOutPtr       Pointer to encapsulated data
 * @param[in]  EncapPduCtr           Counter of the encapsulated PDU
 ****************************************************************************************
 */
__STATIC void lc_get_encap_pdu_data_192(struct byte16 *EncapDataOutPtr, uint8_t *p_pub_key_x, uint8_t *p_pub_key_y,
                                        uint8_t EncapPduCtr)
{
    switch (EncapPduCtr)
    {
    case 0:
    {
        memcpy(EncapDataOutPtr, p_pub_key_x, sizeof(struct byte16));
    }
    break;
    case 1:
    {
        memcpy(EncapDataOutPtr, &(p_pub_key_x[16]), sizeof(struct byte16) / 2);
        memcpy(&(EncapDataOutPtr->A[8]), p_pub_key_y, sizeof(struct byte16) / 2);
    }
    break;
    case 2:
    {
        memcpy(EncapDataOutPtr, &(p_pub_key_y[8]), sizeof(struct byte16));
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, EncapPduCtr, 0);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief This function is used to get a piece of the local P256 public key for encapsulated PDU.
 *
 * @param[out] EncapDataOutPtr       Pointer to encapsulated data
 * @param[in]  EncapPduCtr           Counter of the encapsulated PDU
 ****************************************************************************************
 */
__STATIC void lc_get_encap_pdu_data_256(struct byte16 *EncapDataOutPtr, uint8_t *p_pub_key_x, uint8_t *p_pub_key_y,
                                        uint8_t EncapPduCtr)
{
    switch (EncapPduCtr)
    {
    case 0:
    case 1:
    {
        memcpy(EncapDataOutPtr, &(p_pub_key_x[EncapPduCtr * sizeof(struct byte16)]), sizeof(struct byte16));
    }
    break;

    case 2:
    case 3:
    {
        memcpy(EncapDataOutPtr, &(p_pub_key_y[(EncapPduCtr - 2) * sizeof(struct byte16)]), sizeof(struct byte16));
    }
    break;

    default:
    {
        ASSERT_INFO_FORCE(0, EncapPduCtr, 0);
    }
    break;
    }
}

/// Send public key in encapsulated PSU
void lc_send_encap_pdu_pub_key(uint8_t idx, struct lc_env_tag *p_env, uint8_t tr_id, uint16_t task_id)
{
    struct byte16 EncapDataOut;
    uint8_t *p_local_pub_key_x  =  p_env->sp.p_data->local_pub_key.x;
    uint8_t *p_local_pub_key_y  =  p_env->sp.p_data->local_pub_key.y;

    //  send LMP_EncapsulatedPayload
    if (p_env->sp.sec_con)
    {
        lc_get_encap_pdu_data_256(&EncapDataOut, p_local_pub_key_x, p_local_pub_key_y, p_env->sp.EncapPduCtr);
    }
    else
    {
        lc_get_encap_pdu_data_192(&EncapDataOut, p_local_pub_key_x, p_local_pub_key_y, p_env->sp.EncapPduCtr);
    }

    lc_send_pdu_encaps_payl(idx, &EncapDataOut, tr_id);
    lc_start_lmp_to(task_id);
}

/**
 ****************************************************************************************
 * @brief This function is used to extract the P192 remote public key received as encapsulated PDU
 *
 * @param[in] EncapDataInPtr          The input data to be saved
 * @param[in] p_pub_key_x             Pointer to remote key X-Coordinate
 * @param[in] p_pub_key_y             Pointer to remote key Y-Coordinate
 * @param[in] EncapPduCtr             The number of Encapsulated pay load data
 ****************************************************************************************
 */
__STATIC void lc_extract_rem_pub_key_192_from_encap_pdu(struct byte16 *EncapDataInPtr,
        uint8_t *p_pub_key_x, uint8_t *p_pub_key_y,
        uint8_t EncapPduCtr)
{
    switch (EncapPduCtr)
    {
    case 0:
    {
        // Get first 16 bytes of Pub Key X
        memcpy(p_pub_key_x, EncapDataInPtr, sizeof(struct byte16));
    }
    break;

    case 1:
    {
        // Get last 8 bytes of Pub Key X
        memcpy((p_pub_key_x + 16), EncapDataInPtr, sizeof(struct byte16) / 2);
        // Get first 8 bytes of Pub Key Y
        memcpy(p_pub_key_y, &(EncapDataInPtr->A[8]), sizeof(struct byte16) / 2);
    }
    break;

    case 2:
    {
        // Get last 16 bytes of Pub Key Y
        memcpy((p_pub_key_y + 8), EncapDataInPtr, sizeof(struct byte16));
    }
    break;

    default:
        ASSERT_ERR_FORCE(0);
        break;
    }
}

/**
 ****************************************************************************************
 * @brief This function is used to extract the P256 remote public key received as encapsulated PDU
 *
 * @param[in] EncapDataInPtr             The input data to be saved
 * @param[in] p_pub_key_x                Pointer to remote key X-Coordinate
 * @param[in] p_pub_key_y                Pointer to remote key Y-Coordinate
 * @param[in] EncapPduCtr                The number of Encapsulated pay load data
 ****************************************************************************************
 */
__STATIC void lc_extract_rem_pub_key_256_from_encap_pdu(struct byte16 *EncapDataInPtr,
        uint8_t *p_pub_key_x, uint8_t *p_pub_key_y,
        uint8_t EncapPduCtr)
{
    switch (EncapPduCtr)
    {
    case 0:  // Get first 16 bytes of Pub Key X
    case 1:  // Get last 16 bytes of Pub Key X
    {
        memcpy(&(p_pub_key_x[EncapPduCtr * sizeof(struct byte16)]), EncapDataInPtr, sizeof(struct byte16));
    }
    break;

    case 2: // Get first 16 bytes of Pub Key Y
    case 3: // Get last 16 bytes of Pub Key Y
    {
        memcpy(&(p_pub_key_y[(EncapPduCtr - 2) * sizeof(struct byte16)]), EncapDataInPtr, sizeof(struct byte16));
    }
    break;

    default:
        ASSERT_ERR_FORCE(0);
        break;
    }
}

/**
 ****************************************************************************************
 * @brief This function is used to set a particular 32 bit number in a 16 byte number
 *
 * @param[out] A        Output array
 * @param[in]  Number   32-bit number to be copied
 ****************************************************************************************
 */
__STATIC void lm_sp_32bits_to_array(struct byte16 *A, uint32_t Number)
{
    memset(A, 0, sizeof(struct byte16));
    A->A[0] = Number & 0xff;
    Number >>= 8;
    A->A[1] = Number & 0xff;
    Number >>= 8;
    A->A[2] = Number & 0xff;
    Number >>= 8;
    A->A[3] = Number & 0xff;
    Number >>= 8;
}

/**
 ****************************************************************************************
 * @brief This function is used to get the authentication algorithm based on
 *        the device's IO capability
 *
 * @param[in] LocalCap           Local IO capability pointer structure
 * @param[in] RemoteCap          Remote IO capability pointer structure
 *
 * @return Simple pairing method
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_auth_method(struct io_capability *LocalCap, struct io_capability *RemoteCap)
{
    const uint8_t AUTH1_MAPPING_TABLE[4][4] =
    {
        {SP_JUST_WORKS, SP_JUST_WORKS, SP_PASSKEY, SP_JUST_WORKS},
        {SP_JUST_WORKS, SP_NUM_COMP, SP_PASSKEY, SP_JUST_WORKS},
        {SP_PASSKEY, SP_PASSKEY, SP_PASSKEY, SP_JUST_WORKS},
        {SP_JUST_WORKS, SP_JUST_WORKS, SP_JUST_WORKS, SP_JUST_WORKS}
    };

    /* If any of the device has the OOB go for OOB algorithm                            */
    if (LocalCap->oob_data_present || RemoteCap->oob_data_present)
    {
        return SP_OOB;
    }
    /* If both the device have authentication requirement set to 0, go for JUST WORKS   */
    if (!(LocalCap->aut_req & 0x01) && !(RemoteCap->aut_req & 0x01))
    {
        return SP_JUST_WORKS;
    }
    /* Otherwise return the value from the table                                        */
    return (AUTH1_MAPPING_TABLE[LocalCap->io_cap][RemoteCap->io_cap]);
}

__STATIC uint8_t LM_CheckDisconnectParam(uint8_t Reason)
{
    uint8_t ValidReason[] = {0x05, 0x13, 0x14, 0x15, 0x1A, 0x29};
    uint8_t Counter = 0;
    while (Counter < sizeof(ValidReason))
    {
        if (Reason == ValidReason[Counter])
        {
            return 1;
        }
        Counter++;
    }
    return 0;
}

/// Send HCI CC event returning a connection handle
__STATIC void lc_cmd_cmp_conhdl_send(uint16_t opcode, uint8_t status, uint16_t conhdl)
{
    struct hci_basic_conhdl_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, conhdl, opcode, hci_basic_conhdl_cmd_cmp_evt);
    event->status = status;
    event->conhdl = co_htobs(conhdl);
    hci_send_2_host(event);
}

/// Send HCI CC event returning a BD address
__STATIC void lc_cmd_cmp_bd_addr_send(uint16_t opcode, uint8_t status, struct bd_addr const *bd_addr)
{
    struct hci_basic_bd_addr_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_bd_addr_cmd_cmp_evt);
    event->status = status;
    memcpy(&event->bd_addr, bd_addr, BD_ADDR_LEN);
    hci_send_2_host(event);
}

/**
 ****************************************************************************************
 * @brief This function is used to store a segment of a remote name.
 *
 * @param[in]  link_id             Link ID
 * @param[in]  NameSeg             Name segment
 * @param[in]  NameOffset          Offset of the name
 * @param[in]  NameLen             Length of the name
 *
 ****************************************************************************************
 */
__STATIC void LC_StoreRemoteNameSeg(uint8_t link_id, struct name_vect *NameSeg, uint8_t NameOffset, uint8_t NameLen)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];
    uint8_t Index ;

    // Store remote name segment
    for (Index = 0 ;
            (Index < NAME_VECT_SIZE) &&
            ((Index + NameOffset) < BD_NAME_SIZE) &&
            ((Index + NameOffset) < NameLen);
            Index++)
    {
        lc_env_ptr->info.name_rem.name[Index + NameOffset] = NameSeg->vect[Index];
    }

    // Write end of string if there are enough place in buffer
    if ((Index + NameOffset) < BD_NAME_SIZE)
    {
        lc_env_ptr->info.name_rem.name[Index + NameOffset] = '\0';
    }

    // Update remote name length
    lc_env_ptr->info.name_rem.namelen = NameLen;
}

__STATIC void lc_discon_event_complete_send(ke_task_id_t src_id, uint8_t status, uint16_t conhdl, uint8_t reason)
{
    // allocate the complete event message
    struct hci_disc_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_DISC_CMP_EVT_CODE, hci_disc_cmp_evt);
    // update the status
    event->status = status;
    // update the reason
    event->reason = reason;
    // update the link handle
    event->conhdl = co_htobs(conhdl);
    // send the message
    hci_send_2_host(event);
}

/// Callback executed when Elliptic Curve algorithm completes
__STATIC void lc_dhkey_computation_completed(ke_task_id_t pid, const uint8_t *p_dh_key, uint8_t key_len)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    ke_state_t state = ke_state_get(pid);
    switch (state)
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Result discarded - in disconnection states
        break;
    default:
    {
        lc_env_ptr->sp.dh_key_compute_state = LC_DH_KEY_COMP_SUCCESS;

        // Copy the resulting key
        if (lc_env_ptr->sp.p_data != NULL)
        {
            memcpy(&lc_env_ptr->sp.p_data->dh_key.key, p_dh_key, key_len);
        }

        if (state == LC_WAIT_DHKEY_COMPUTING)
        {
            /*
             * SP AUTH STG2 : START
             */
            lc_try_start_auth_stage2(pid);
        }
    }
    break;
    }
}

__STATIC void lc_dhkey_256_computation_completed(uint32_t pid, const ecc_p256_result_t *p_res)
{
    lc_dhkey_computation_completed((ke_task_id_t)pid, p_res->key_res_x, PUB_KEY_256_LEN / 2);
}

__STATIC void lc_dhkey_192_computation_completed(uint32_t pid, uint8_t status, const ecc_p192_result_t *p_res)
{
    lc_dhkey_computation_completed((ke_task_id_t)pid, p_res->key_res_x, PUB_KEY_192_LEN / 2);
}
/**
 ****************************************************************************************
 * @brief This function starts the Diffie Hellman Key computation
 *
 * @param[in] pid         Link identifier
 ****************************************************************************************
 */
__STATIC void lc_dhkey_computation_start(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t key_status;
    lc_sp_data_t *p_data = lc_env_ptr->sp.p_data;

    if (lc_env_ptr->sp.sec_con)
    {
        key_status = ecc_p256_gen_dh_key(p_data->priv_key.key,
                                         p_data->remote_pub_key.x,
                                         p_data->remote_pub_key.y,
                                         pid, lc_dhkey_256_computation_completed);
    }
    else
    {
        key_status = ecc_p192_gen_dh_key(p_data->priv_key.key,
                                         p_data->remote_pub_key.x,
                                         p_data->remote_pub_key.y,
                                         pid, lc_dhkey_192_computation_completed);
    }

    lc_env_ptr->sp.dh_key_compute_state = (key_status == CO_ERROR_NO_ERROR)
                                          ? LC_DH_KEY_COMP_PENDING : LC_DH_KEY_COMP_FAILED;
}


/**
 ****************************************************************************************
 * @brief Handle sending of the received number of completed packet events.
 *
 * @param[in] conhdl        connection handle.
 * @param[in] nb_of_pkt     number of completed packets
 ****************************************************************************************
 */
__STATIC void lc_common_nb_of_pkt_comp_evt_send(uint16_t conhdl, uint8_t nb_of_pkt)
{
    // allocates the message to send
    struct hci_nb_cmp_pkts_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_NB_CMP_PKTS_EVT_CODE, hci_nb_cmp_pkts_evt);

    // gets the connection handle used
    event->con[0].hdl         = co_htobs(conhdl);
    // gets the number of packet sent
    event->con[0].nb_comp_pkt = co_htobs(nb_of_pkt);
    // processed handle by handle
    event->nb_of_hdl          = 1;

    // send the message
    hci_send_2_host(event);
}

#if (MAX_NB_SYNC > 0)
/**
 ****************************************************************************************
 * @brief Check the synchronous connection parameters
 * @param[in] params     Parameters to be checked
 ****************************************************************************************
 */
__STATIC uint8_t lc_setup_sync_param_check(uint8_t idx, struct lc_sco_host_params_tag *params)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t CommonPacketType;

    struct features *RemoteFeaturesPtr = &lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0];

    uint16_t vx_set = 0x0040;
    switch (params->tx_cod_fmt[0])
    {
    case CODING_FORMAT_ULAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_ALAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_TRANSP:
        vx_set += AIR_COD_TRANS;
        break;
    case CODING_FORMAT_MSBC:
        vx_set += AIR_COD_TRANS;
        break;
    default:
        vx_set += AIR_COD_CVSD;
        break;
    }

    do
    {
        // Check if the link is encrypted with AES-CCM (used when secure connections is supported)
        if (lc_env_ptr->enc.EncEnable && lc_env_ptr->sp.sec_con)
        {
            // SCO cannot be established if AES-CCM encryption is enabled
            if ((params->packet_type & (SYNC_PACKET_TYPE_HV1_FLAG | SYNC_PACKET_TYPE_HV2_FLAG | SYNC_PACKET_TYPE_HV3_FLAG)) &&
                    ((params->packet_type & (SYNC_PACKET_TYPE_EV3_FLAG | SYNC_PACKET_TYPE_EV4_FLAG | SYNC_PACKET_TYPE_EV5_FLAG |
                                             SYNC_PACKET_TYPE_EV3_2_FLAG | SYNC_PACKET_TYPE_EV3_3_FLAG |
                                             SYNC_PACKET_TYPE_EV5_2_FLAG | SYNC_PACKET_TYPE_EV5_3_FLAG)) == 0))
            {
                status = CO_ERROR_CONN_REJ_SECURITY_REASONS;
                break;
            }
        }

        // Check if SCO only
        if ((((params->packet_type & SYNC_PACKET_TYPE_HV1_FLAG) == SYNC_PACKET_TYPE_HV1_FLAG) ||
                ((params->packet_type & SYNC_PACKET_TYPE_HV2_FLAG) == SYNC_PACKET_TYPE_HV2_FLAG) ||
                ((params->packet_type & SYNC_PACKET_TYPE_HV3_FLAG) == SYNC_PACKET_TYPE_HV3_FLAG)) &&
                ((params->packet_type & (SYNC_PACKET_TYPE_EV3_FLAG |
                                         SYNC_PACKET_TYPE_EV4_FLAG |
                                         SYNC_PACKET_TYPE_EV5_FLAG |
                                         SYNC_PACKET_TYPE_EV3_2_FLAG |
                                         SYNC_PACKET_TYPE_EV3_3_FLAG |
                                         SYNC_PACKET_TYPE_EV5_2_FLAG |
                                         SYNC_PACKET_TYPE_EV5_3_FLAG)) == 0))
        {
            // HV2 not supported
            if (params->packet_type == SYNC_PACKET_TYPE_HV2_FLAG)
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        /* Check if at least a packet type specified                                    */
        if (params->packet_type == 0x0000)
            break;

        /* Check the retransmission effort if the SCO connection is requested           */
        if ((((params->packet_type & SYNC_PACKET_TYPE_HV1_FLAG) == SYNC_PACKET_TYPE_HV1_FLAG) ||
                ((params->packet_type & SYNC_PACKET_TYPE_HV2_FLAG) == SYNC_PACKET_TYPE_HV2_FLAG) ||
                ((params->packet_type & SYNC_PACKET_TYPE_HV3_FLAG) == SYNC_PACKET_TYPE_HV3_FLAG)) &&
                ((params->packet_type & (SYNC_PACKET_TYPE_EV3_FLAG |
                                         SYNC_PACKET_TYPE_EV4_FLAG |
                                         SYNC_PACKET_TYPE_EV5_FLAG |
                                         SYNC_PACKET_TYPE_EV3_2_FLAG |
                                         SYNC_PACKET_TYPE_EV3_3_FLAG |
                                         SYNC_PACKET_TYPE_EV5_2_FLAG |
                                         SYNC_PACKET_TYPE_EV5_3_FLAG)) == 0))
        {
            if ((params->retx_eff == SYNC_RE_TX_POWER) || (params->retx_eff == SYNC_RE_TX_QUALITY))
                break;
        }
        if ((((RemoteFeaturesPtr->feats[1] & B1_SCO_MSK) == 0) &&
                ((RemoteFeaturesPtr->feats[3] & B3_ESCO_EV3_MSK) == 0)) ||
                (((RemoteFeaturesPtr->feats[1] & B1_SCO_MSK) == 0) &&
                 ((params->packet_type & (SYNC_PACKET_TYPE_EV3_FLAG |
                                          SYNC_PACKET_TYPE_EV4_FLAG |
                                          SYNC_PACKET_TYPE_EV5_FLAG |
                                          SYNC_PACKET_TYPE_EV3_2_FLAG |
                                          SYNC_PACKET_TYPE_EV3_3_FLAG |
                                          SYNC_PACKET_TYPE_EV5_2_FLAG |
                                          SYNC_PACKET_TYPE_EV5_3_FLAG)) == 0)) ||
                (((RemoteFeaturesPtr->feats[3] & B3_ESCO_EV3_MSK) == 0) &&
                 ((params->packet_type & (SYNC_PACKET_TYPE_HV1_FLAG |
                                          SYNC_PACKET_TYPE_HV2_FLAG |
                                          SYNC_PACKET_TYPE_HV3_FLAG)) == 0)))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        /* Get Common packet type (SCO & eSCO)                                          */
        CommonPacketType = lm_get_common_pkt_types(SYNC_PACKET_TYPE_HV1_FLAG |
                           SYNC_PACKET_TYPE_HV2_FLAG |
                           SYNC_PACKET_TYPE_HV3_FLAG |
                           SYNC_PACKET_TYPE_EV3_FLAG |
                           SYNC_PACKET_TYPE_EV4_FLAG |
                           SYNC_PACKET_TYPE_EV5_FLAG |
                           SYNC_PACKET_TYPE_EV3_2_FLAG |
                           SYNC_PACKET_TYPE_EV3_3_FLAG |
                           SYNC_PACKET_TYPE_EV5_2_FLAG |
                           SYNC_PACKET_TYPE_EV5_3_FLAG,
                           RemoteFeaturesPtr);

        /* Check if at least one common packet type can be established                  */
        if ((CommonPacketType & params->packet_type) == 0x0000)
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        /* Keep only compatible packet types                                           */
        params->packet_type &= CommonPacketType;

        /* Check Retransmission Effort                                                  */
        if ((params->retx_eff > SYNC_RE_TX_QUALITY) && (params->retx_eff != SYNC_RE_TX_DONT_CARE))
            break;

        /* Check if Latency requested is not too small                                  */
        if (params->max_lat < SYNC_MIN_LATENCY)
            break;

        /* Check voice setting input coding                                            */
        if ((vx_set & INPUT_COD_MSK) > INPUT_COD_ALAW)
            break;

        status = CO_ERROR_SCO_AIR_MODE_REJECTED;

        /* Check Air mode requested                                                     */
        if ((vx_set & AIR_COD_MSK) == AIR_COD_CVSD)
        {
            if ((RemoteFeaturesPtr->feats[2] & B2_CVSD_MSK) == 0)
                break;
        }
        else if ((vx_set & AIR_COD_MSK) == AIR_COD_MULAW)
        {
            if ((RemoteFeaturesPtr->feats[1] & B1_MULAW_MSK) == 0)
                break;
        }
        else if ((vx_set & AIR_COD_MSK) == AIR_COD_ALAW)
        {
            if ((RemoteFeaturesPtr->feats[1] & B1_ALAW_MSK) == 0)
                break;
        }
        else if ((vx_set & AIR_COD_MSK) == AIR_COD_TRANS)
        {
            if ((RemoteFeaturesPtr->feats[2] & B2_TRANSPARENT_SCO_MSK) == 0)
                break;
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    return status;
}
#endif // (MAX_NB_SYNC > 0)


/**
 ****************************************************************************************
 * @brief This function checks if the maximum number of slot that can be used for the connection
 *
 * @param[in] LinkId      Link identifier
 *
 * @return maximum number of slot that can be used for the connection
 ****************************************************************************************
 */
__STATIC uint8_t lc_check_max_slot(uint8_t LinkId)
{
    struct lc_env_tag *lc_env_ptr = lc_env[LinkId];
    uint8_t max_slot = 0x01;
    uint8_t min_sync_intv = 0xFF;
#if (MAX_NB_SYNC > 0)
    min_sync_intv = lm_get_min_sync_intv();
#endif //(MAX_NB_SYNC > 0)

    if (min_sync_intv < 12)
    {
        max_slot = 0x1;
    }
    else if ((min_sync_intv < 18) && (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_3_SLOT_BIT_POS)))
    {
        max_slot = 0x03;
    }
    else if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_5_SLOT_BIT_POS))
    {
        max_slot = 0x05;
    }

#if (EAVESDROPPING_SUPPORT)
    max_slot = co_min(max_slot, lm_get_host_max_slot());
#endif // EAVESDROPPING_SUPPORT

    return max_slot;
}

#if (EAVESDROPPING_SUPPORT)
__STATIC void lc_ed_min_max_pwr_ind(uint8_t link_id, uint8_t pwr_level)
{
    // Allocate indication message
    struct ed_min_max_pwr_ind *ind = KE_MSG_ALLOC(ED_MIN_MAX_PWR_IND, TASK_ED, TASK_NONE, ed_min_max_pwr_ind);

    // Fill data
    ind->link_id = link_id;
    ind->level   = pwr_level;

    ke_msg_send(ind);
}
#endif // EAVESDROPPING_SUPPORT

/**
 ****************************************************************************************
 * @brief This function returns the number rx slots in the remote submap
 *
 * @param[in] idx      Link identifier
 * @param[in] t_sam_sm      Submap interval
 *
 * @return number of rx slots in the submap, limited to specified interval size
 ****************************************************************************************
 */
__STATIC uint8_t lc_sam_rem_submap_rx_slots_get(uint8_t idx, uint8_t t_sam_sm)
{
    uint8_t byte_idx;
    uint8_t bit_pos;

    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    uint8_t rx_slots = 0;

    for (int i = 0; i < t_sam_sm; i++)
    {
        byte_idx = i >> 2; // 4 sam slots per byte
        bit_pos = (i & 0x3) << 1; // 2 bits fields

        if (lc_env_ptr->sam_info.rem_submap.map[byte_idx] & (SAM_SLOT_RX_AVAILABLE << bit_pos))
            rx_slots++;
    }

    return rx_slots;
}

/**
 ****************************************************************************************
 * @brief This function returns the number tx slots in the remote submap
 *
 * @param[in] idx      Link identifier
 * @param[in] t_sam_sm      Submap interval
 *
 * @return number of tx slots in the submap, limited to specified interval size
 ****************************************************************************************
 */
__STATIC uint8_t lc_sam_rem_submap_tx_slots_get(uint8_t idx, uint8_t t_sam_sm)
{
    uint8_t byte_idx;
    uint8_t bit_pos;

    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    uint8_t tx_slots = 0;

    for (int i = 0; i < t_sam_sm; i++)
    {
        byte_idx = i >> 2; // 4 sam slots per byte
        bit_pos = (i & 0x3) << 1; // 2 bits fields

        if (lc_env_ptr->sam_info.rem_submap.map[byte_idx] & (SAM_SLOT_TX_AVAILABLE << bit_pos))
            tx_slots++;
    }

    return tx_slots;
}

/**
 ****************************************************************************************
 * @brief This function builds a full SAM slot map from defined SAM pattern and submap.
 *
 * @param[in] lc_env_ptr      Link env ptr
 * @param[in] sam_index      active SAM index
 * @param[out] sam_map        SAM Map
 *
 ****************************************************************************************
 */
__STATIC void lc_build_sam_map(struct lc_env_tag *lc_env_ptr, uint8_t sam_index, uint8_t *sam_map)
{
    struct lc_sam_pattern *sam_pattern = &lc_env_ptr->sam_info.rem_pattern[sam_index];
    uint8_t *rem_submaps = &sam_pattern->submaps.map[0];
    uint8_t *rem_submap0 = &lc_env_ptr->sam_info.rem_submap.map[0];

    uint8_t rd_byte_idx; // submap read byte index
    uint8_t rd_bit_pos; // submap read bit pos
    int i_sm; // submap index
    int i_sl; // slot index

    uint8_t wr_byte_idx; // slot write byte index
    uint8_t wr_bit_pos; // slot write bit pos
    int i_wr; // write index

    uint8_t submap_type;

    memset(sam_map, 0, RW_PEER_SAM_MAP_MAX_LEN); // 0 = all unavailable

    // Construct the SAM Map from the peer's SAM Pattern & Submap0.
    for (i_sm = 0, i_wr = 0; i_sm < sam_pattern->n_sam_sm; i_sm++)
    {
        rd_byte_idx = i_sm >> 2; // 4 sam submaps per byte
        rd_bit_pos = (i_sm & 0x3) << 1; // 2 bit fields

        submap_type = (rem_submaps[rd_byte_idx] >> rd_bit_pos) & 0x3;

        switch (submap_type)
        {
        case SAM_SLOTS_SUBMAPPED:
        {
            for (i_sl = 0; i_sl < sam_pattern->t_sam_sm; i_sl++, i_wr++)
            {
                rd_byte_idx = i_sl >> 2; // 4 sam submaps per byte
                rd_bit_pos = (i_sl & 0x3) << 1; // 2 bit fields

                wr_byte_idx = i_wr >> 2; // 4 slots per byte
                wr_bit_pos = (i_wr & 0x3) << 1; // 2 bit fields

                sam_map[wr_byte_idx] |= ((rem_submap0[rd_byte_idx] >> rd_bit_pos) & 0x3) << wr_bit_pos;
            }
        }
        break;
        case SAM_SLOTS_AVAILABLE:
        {
            for (i_sl = 0; i_sl < sam_pattern->t_sam_sm; i_sl++, i_wr++)
            {
                wr_byte_idx = i_wr >> 2; // 4 slots per byte
                wr_bit_pos = (i_wr & 0x3) << 1; // 2 bit fields

                sam_map[wr_byte_idx] |= (SAM_SLOT_TX_RX_AVAILABLE << wr_bit_pos);
            }
        }
        break;
        case SAM_SLOTS_UNAVAILABLE:
        {
            // Increment over unavailable slots, leave set to 0
            i_wr += sam_pattern->t_sam_sm;
        }
        break;
        default: /* No impact */
            break;
        }
    }
}

/**
****************************************************************************************
* @brief This function restores SAM enabled on the specified link
*
* @param[in] idx      Link identifier
****************************************************************************************
*/
__STATIC void lc_sam_restore(uint8_t idx)
{
    struct lc_sam_tag *sam_info = &lc_env[idx]->sam_info;

    if ((sam_info->loc_idx != SAM_DISABLED) || (sam_info->rem_idx != SAM_DISABLED))
    {
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        /*
        * Send status change event only. SAM is inhibited at driver level for role switch, and deferred decision
        * on disabling SAM at driver level on succesful role switch lets SAM continue otherwise.
        */
        struct hci_sam_status_change_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SAM_STATUS_CHANGE_EVT_CODE, hci_sam_status_change_evt);

        evt->conhdl = conhdl;
        evt->rem_idx = sam_info->rem_idx;
        evt->loc_idx = sam_info->loc_idx;
        evt->rem_tx_av = sam_info->rem_tx_av;
        evt->rem_rx_av = sam_info->rem_rx_av;
        evt->loc_tx_av = sam_info->loc_tx_av;
        evt->loc_rx_av = sam_info->loc_rx_av;

        hci_send_2_host(evt);
    }
}

/**
****************************************************************************************
* @brief This function completes the ACL connection establishment
*
* @param[in] idx      Link identifier
****************************************************************************************
*/
__STATIC void lc_setup_cmp(uint8_t idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->link.ConnectedState)
    {
        if (!lc_env_ptr->link.SetupComplete)
        {
            if (!lc_env_ptr->link.SetupCompTx)
            {
                // send LMP_SetupComplete
                lc_send_pdu_setup_cmp(idx, lc_env_ptr->link.Role);
                lc_env_ptr->link.SetupCompTx = true;
            }

            if (lc_env_ptr->link.SetupCompRx)
            {
                lc_con_cmp(KE_BUILD_ID(TASK_LC, idx));
            }
        }
    }
}

/// Check if remote public key is valid, stop pairing if not
__STATIC bool lc_sp_check_remote_pub_key_valid(ke_task_id_t pid, struct lc_env_tag *lc_env_ptr)
{
    bool is_remote_public_key_valid = false;
    lc_sp_data_t *p_data = lc_env_ptr->sp.p_data;

    if (lc_env_ptr->sp.sec_con)
    {
        if ((memcmp(p_data->local_pub_key.x, p_data->remote_pub_key.y, PUB_KEY_256_LEN / 2) != 0)
                || lm_debug_key_compare_256(p_data->remote_pub_key.x, p_data->remote_pub_key.y))
        {
            is_remote_public_key_valid = true;
        }
    }
    else
    {
        if ((memcmp(p_data->local_pub_key.x, p_data->remote_pub_key.x, PUB_KEY_192_LEN / 2) != 0)
                || lm_debug_key_compare_192(p_data->remote_pub_key.x, p_data->remote_pub_key.y))
        {
            is_remote_public_key_valid = true;
        }
    }

    if (!is_remote_public_key_valid)
    {
        // send LMP_NotAccepted(LMP_ENCAPS_PAYL_OPCODE)
        lc_send_pdu_not_acc(KE_IDX_GET(pid), LMP_ENCAPS_PAYL_OPCODE, CO_ERROR_INVALID_LMP_PARAM,
                            lc_env_ptr->link.RxTrIdServer);

        lc_sp_fail(pid);
    }

    return is_remote_public_key_valid;
}

/// Do the public key exchange
__STATIC void lc_sp_do_public_key_exchange(ke_task_id_t pid, struct lc_env_tag *lc_env_ptr)
{
    int idx = KE_IDX_GET(pid);
    // Send public key as initiator of the pairing
    if (lc_env_ptr->sp.SPInitiator)
    {
        if (lc_env_ptr->sp.sec_con)
        {
            lc_send_pdu_encaps_head(idx, lc_env_ptr->link.RxTrIdServer, LMP_ENCAPS_P256_MAJ_TYPE, LMP_ENCAPS_P256_MIN_TYPE, LMP_ENCAPS_P256_PAYL_LEN);
        }
        else
        {
            lc_send_pdu_encaps_head(idx, lc_env_ptr->link.RxTrIdServer, LMP_ENCAPS_P192_MAJ_TYPE, LMP_ENCAPS_P192_MIN_TYPE, LMP_ENCAPS_P192_PAYL_LEN);
        }

        // start the lmp timer
        lc_start_lmp_to(pid);

        // set to new state
        ke_state_set(pid, LC_PUB_KEY_HEADER_INIT_LOC);
    }
    // Send public key as responder of the pairing
    else // responder
    {
        // check public key validity
        if (lc_sp_check_remote_pub_key_valid(pid, lc_env_ptr))
        {
            // LMP_Accepted (LMP_ENCAPS_PAYL_OPCODE)
            lc_send_pdu_acc(idx, LMP_ENCAPS_PAYL_OPCODE, lc_env_ptr->link.RxTrIdServer);

            /*
             * START COMPUTING DIFFIE HELLMAN KEY (only if received public key is valid)
             */
            lc_dhkey_computation_start(pid);


            if (lc_env_ptr->sp.sec_con)
            {
                lc_send_pdu_encaps_head(idx, lc_env_ptr->link.RxTrIdServer, LMP_ENCAPS_P256_MAJ_TYPE, LMP_ENCAPS_P256_MIN_TYPE, LMP_ENCAPS_P256_PAYL_LEN);
            }
            else
            {
                lc_send_pdu_encaps_head(idx, lc_env_ptr->link.RxTrIdServer, LMP_ENCAPS_P192_MAJ_TYPE, LMP_ENCAPS_P192_MIN_TYPE, LMP_ENCAPS_P192_PAYL_LEN);
            }

            // start timer
            lc_start_lmp_to(pid);

            // state change
            ke_state_set(pid, LC_PUB_KEY_HEADER_RSP_CFM);
        }
    }
}


/// P256 public key generated
__STATIC void lc_sp_gen_p256_pub_key_completed(uint32_t pid, const ecc_p256_result_t *p_res)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_sp_data_t *p_data = lc_env_ptr->sp.p_data;
    if (p_data != NULL)
    {
        // copy generated P256 public key
        memcpy(p_data->local_pub_key.x, p_res->key_res_x, PUB_KEY_256_LEN / 2);
        memcpy(p_data->local_pub_key.y, p_res->key_res_y, PUB_KEY_256_LEN / 2);

        // Send public key
        lc_sp_do_public_key_exchange((uint32_t)pid, lc_env_ptr);
    }
}

/// P192 public key generated
__STATIC void lc_sp_gen_p192_pub_key_completed(uint32_t pid, uint8_t status, const ecc_p192_result_t *p_res)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_sp_data_t *p_data = lc_env_ptr->sp.p_data;
    if (p_data != NULL)
    {
        // copy generated P192 public key
        memcpy(p_data->local_pub_key.x, p_res->key_res_x, PUB_KEY_192_LEN / 2);
        memcpy(p_data->local_pub_key.y, p_res->key_res_y, PUB_KEY_192_LEN / 2);

        // Send public key
        lc_sp_do_public_key_exchange((ke_task_id_t)pid, lc_env_ptr);
    }
}

/// Prepare execution of public key exchange
__STATIC void lc_sp_prepare_public_key_exchange(ke_task_id_t pid, struct lc_env_tag *lc_env_ptr)
{
    bool generate_priv_pub_key_pair = true;
    lc_sp_data_t *p_data = lc_env_ptr->sp.p_data;

    // Ensure that OOB random is initialized to Zero - required if no local OOB data provided to peer
    memset(&p_data->local_oob_rand, 0, sizeof(struct randomizer));

    // OOB data exchanged
    if (lc_env_ptr->sp.IOCap_rem.oob_data_present)
    {
        // Retrieve private/public key pair, and random value associated with OOB data
        lm_oob_data_t *p_oob_data = lm_sp_get_local_oob_data();
        if (p_oob_data != NULL)
        {

            // P256 - Secure Connection
            if (lc_env_ptr->sp.sec_con)
            {
                // Copy OOB data information for Secure connection (P-256)
                memcpy(&(p_data->local_oob_rand), &(p_oob_data->rand_256), sizeof(struct randomizer));
                memcpy(p_data->priv_key.key, p_oob_data->priv_key_256.key, PRIV_KEY_256_LEN);
                memcpy(p_data->local_pub_key.x, p_oob_data->pub_key_256.x, PUB_KEY_256_LEN / 2);
                memcpy(p_data->local_pub_key.y, p_oob_data->pub_key_256.y, PUB_KEY_256_LEN / 2);
                // generate private/public key pair only if no P-256 OOB data generated
                generate_priv_pub_key_pair = !p_oob_data->is_256_generated;
            }
            // P192
            else
            {
                // Copy OOB data information for P-192
                memcpy(&(p_data->local_oob_rand), &(p_oob_data->rand_192), sizeof(struct randomizer));
                memcpy(p_data->priv_key.key, p_oob_data->priv_key_192.key, PRIV_KEY_192_LEN);
                memcpy(p_data->local_pub_key.x, p_oob_data->pub_key_192.x, PUB_KEY_192_LEN / 2);
                memcpy(p_data->local_pub_key.y, p_oob_data->pub_key_192.y, PUB_KEY_192_LEN / 2);
                generate_priv_pub_key_pair = false;
            }

            // OOB data no more valid for other pairing
            lm_sp_release_local_oob_data();
        }
    }

    if (generate_priv_pub_key_pair)
    {
        uint8_t key_gen_status;
        // Mark that private/public key pair is waiting to be generated
        ke_state_set(pid, LC_WAIT_PRIV_PUB_KEY_GEN);

        if (lc_env_ptr->sp.sec_con)
        {
            // Generate p256 key pair
            key_gen_status = ecc_p256_gen_key_pair(lm_sp_debug_mode_get(), p_data->priv_key.key, pid, lc_sp_gen_p256_pub_key_completed);
        }
        else
        {
            // Generate p192 key pair
            key_gen_status = ecc_p192_gen_key_pair(lm_sp_debug_mode_get(), p_data->priv_key.key, pid, lc_sp_gen_p192_pub_key_completed);
        }

        ASSERT_INFO(key_gen_status == CO_ERROR_NO_ERROR, pid, key_gen_status);
        if (key_gen_status != CO_ERROR_NO_ERROR)
        {
            // Continue with invalid key - pairing will fail
            lc_sp_do_public_key_exchange(pid, lc_env_ptr);
        }
    }
    else
    {
        // send public key
        lc_sp_do_public_key_exchange(pid, lc_env_ptr);
    }
}
/*
 * HCI LINK CONTROL COMMANDS HANDLERS
 ****************************************************************************************
 */

HCI_CMD_HANDLER_C(disconnect, struct hci_disconnect_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    default:
    {
        // Check command parameters
        if (!LM_CheckDisconnectParam(param->reason))
        {
            lc_cmd_stat_send(opcode, CO_ERROR_INVALID_HCI_PARAM);
            break;
        }

        // If ACL connection
        if (param->conhdl == (BT_ACL_CONHDL_MIN + idx))
        {
            lc_env_ptr->link.Reason = param->reason;
            lc_env_ptr->req.LocDetachReq = true;
            ke_msg_send_basic(LC_OP_DETACH_IND, dest_id, dest_id);

            lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);
        }
        // If SCO connection
        else
        {
            uint8_t status = CO_ERROR_NO_ERROR;

            if (ke_state_get(dest_id) == LC_CONNECTED)
            {
#if (MAX_NB_SYNC > 0)
                status = lc_sco_host_request_disc(dest_id, param->conhdl, param->reason);
#else // (MAX_NB_SYNC > 0)
                status = CO_ERROR_UNKNOWN_CONNECTION_ID;
#endif // (MAX_NB_SYNC > 0)

                lc_cmd_stat_send(opcode, status);
            }
            else
            {
                return (KE_MSG_SAVED);
            }
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}


/// Handle the command HCI cancel create connection
HCI_CMD_HANDLER_C(create_con_cancel, struct hci_basic_bd_addr_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check state
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        if (!lc_env_ptr->link.ConnectionCompleteSent)
        {
            //Disconnect link
            status = ld_acl_stop(KE_IDX_GET(dest_id));

            if (status != CO_ERROR_NO_ERROR)
            {
                ASSERT_ERR_FORCE(0); // should not reach here
            }
        }
        else
        {
            // Reject the command as the connection already exists
            status = CO_ERROR_CON_ALREADY_EXISTS;
        }
    }
    break;
    }

    // Send CC event
    lc_cmd_cmp_bd_addr_send(opcode, status, &param->bd_addr);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(setup_sync_con, struct hci_setup_sync_con_cmd)
{
#if (MAX_NB_SYNC > 0)
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t air_coding, hci_coding, pcm_samp_size, pcm_data_fmt, pcm_msb_pos;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Send the command complete event
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    case LC_FREE:
        // Send the command complete event
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    {
        struct lc_sco_host_params_tag *host_params = &lc_env_ptr->sco;

        // Decode voice setting field (air coding)
        switch (param->vx_set & AIR_COD_MSK)
        {
        case AIR_COD_MULAW:
            air_coding = CODING_FORMAT_ULAW;
            break;
        case AIR_COD_ALAW:
            air_coding = CODING_FORMAT_ALAW;
            break;
        case AIR_COD_TRANS:
            air_coding = CODING_FORMAT_TRANSP;
            break;
        case AIR_COD_CVSD:
        default:
            air_coding = CODING_FORMAT_CVSD;
            break;
        }
        // Decode voice setting field (input coding)
        switch (param->vx_set & INPUT_COD_MSK)
        {
        case INPUT_COD_MULAW:
            hci_coding = CODING_FORMAT_ULAW;
            break;
        case INPUT_COD_ALAW:
            hci_coding = CODING_FORMAT_ALAW;
            break;
        case INPUT_COD_LIN:
        default:
            hci_coding = CODING_FORMAT_LINPCM;
            break;
        }
        // Decode voice setting field (input sample size)
        switch (param->vx_set & INPUT_SAMPSIZE_MSK)
        {
        case INPUT_SAMP_16BIT:
            pcm_samp_size = PCM_SAMPLE_SIZE_16BITS;
            break;
        case INPUT_SAMP_8BIT:
        default:
            pcm_samp_size = PCM_SAMPLE_SIZE_8BITS;
            break;
        }
        // Decode voice setting field (input data format)
        switch (param->vx_set & INPUT_DATAFORM_MSK)
        {
        case INPUT_DATA_1COMP:
            pcm_data_fmt = PCM_FORMAT_1SCOMP;
            break;
        case INPUT_DATA_SMAG:
            pcm_data_fmt = PCM_FORMAT_SIGNMAG;
            break;
        case INPUT_DATA_UNSIGNED:
            pcm_data_fmt = PCM_FORMAT_UNSIGNED;
            break;
        case INPUT_DATA_2COMP:
        default:
            pcm_data_fmt = PCM_FORMAT_2SCOMP;
            break;
        }
        // Decode voice setting field (msb position)
        pcm_msb_pos = ((param->vx_set & LIN_PCM_BIT_POS_MSK) >> LIN_PCM_BIT_POS_OFF);

        // Transmit/Receive Bandwidth (in B/sec)
        host_params->tx_bw             = param->tx_bw;
        host_params->rx_bw             = param->rx_bw;

        // Transmit/Receive Coding Format
        memset(&host_params->tx_cod_fmt[0], 0, sizeof(host_params->tx_cod_fmt));
        host_params->tx_cod_fmt[0]    = air_coding;
        memset(&host_params->rx_cod_fmt[0], 0, sizeof(host_params->rx_cod_fmt));
        host_params->rx_cod_fmt[0]    = air_coding;

        // Transmit/Receive Codec Frame Size (in B)
        host_params->tx_cod_fr_sz     = 0;
        host_params->rx_cod_fr_sz     = 0;

        // Input/Output Bandwidth (in B/sec)
        host_params->in_bw            = param->tx_bw;
        host_params->out_bw           = param->rx_bw;

        // Input/Output Coding Format
        memset(&host_params->in_cod_fmt[0], 0, sizeof(host_params->in_cod_fmt));
        host_params->in_cod_fmt[0]    = hci_coding;
        memset(&host_params->out_cod_fmt[0], 0, sizeof(host_params->out_cod_fmt));
        host_params->out_cod_fmt[0]   = hci_coding;

        // Input/Output Coded Data Size (in bits)
        host_params->in_cod_data_sz   = pcm_samp_size;
        host_params->out_cod_data_sz  = pcm_samp_size;

        // Input/Output PCM Data Format
        host_params->in_data_fmt      = pcm_data_fmt;
        host_params->out_data_fmt     = pcm_data_fmt;

        // Input/Output PCM Sample Payload MSB Position (in bits)
        host_params->in_msb_pos       = pcm_msb_pos;
        host_params->out_msb_pos      = pcm_msb_pos;

        // Input/Output Data Path (VoHCI / PCM / Other ...)
        host_params->in_data_path     = AUDIO_DATA_PATH_HCI;
        host_params->out_data_path    = AUDIO_DATA_PATH_HCI;

        // Input/Output Transport Unit Size (in bits)
        host_params->in_tr_unit_sz    = 0;
        host_params->out_tr_unit_sz   = 0;

        // Max Latency (in ms)
        host_params->max_lat           = param->max_lat;
        // Packet Type (HV1 / HV2 / HV3 / EV3 / EV4 / EV5 / 2EV3 / 3EV3 / 2EV5 / 3EV5)
        host_params->packet_type       = param->pkt_type ^ (SYNC_PACKET_TYPE_NO_EV3_2_FLAG |
                                         SYNC_PACKET_TYPE_NO_EV3_3_FLAG |
                                         SYNC_PACKET_TYPE_NO_EV5_2_FLAG |
                                         SYNC_PACKET_TYPE_NO_EV5_3_FLAG);
        // Retransmission Effort (No, opt power, opt quality, don't care)
        host_params->retx_eff          = param->retx_eff;

        // Check parameters
        status = lc_setup_sync_param_check(idx, host_params);

        // Send command status event
        lc_cmd_stat_send(opcode, status);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Trigger setup synchronous connection procedure
            struct lc_op_loc_sync_con_req *req = KE_MSG_ALLOC(LC_OP_LOC_SYNC_CON_REQ, dest_id, dest_id, lc_op_loc_sync_con_req);
            req->conhdl = param->conhdl;
            req->old_style = true;
            req->host_initiated = true;
            ke_msg_send(req);
        }
    }
    break;
    }

#else // (MAX_NB_SYNC > 0)
    lc_cmd_stat_send(opcode, CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED);
#endif // (MAX_NB_SYNC > 0)

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(enh_setup_sync_con, struct hci_enh_setup_sync_con_cmd)
{
#if (MAX_NB_SYNC > 0)
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Send the command complete event
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    case LC_FREE:
        // Send the command complete event
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    {
        struct lc_sco_host_params_tag *host_params = &lc_env_ptr->sco;

        host_params->tx_bw           = param->tx_bw           ;  // Transmit Bandwidth (in B/sec)
        host_params->rx_bw           = param->rx_bw           ;  // Receive Bandwidth (in B/sec)
        host_params->tx_cod_fr_sz    = param->tx_cod_fr_sz    ;  // Transmit Codec Frame Size (in B)
        host_params->rx_cod_fr_sz    = param->rx_cod_fr_sz    ;  // Receive Codec Frame Size (in B)
        host_params->in_bw           = param->in_bw           ;  // Input Bandwidth (in B/sec)
        host_params->out_bw          = param->out_bw          ;  // Output Bandwidth (in B/sec)
        host_params->in_cod_data_sz  = param->in_cod_data_sz  ;  // Input Coded Data Size (in bits)
        host_params->out_cod_data_sz = param->out_cod_data_sz ;  // Output Coded Data Size (in bits)
        host_params->in_data_fmt     = param->in_data_fmt     ;  // Input PCM Data Format
        host_params->out_data_fmt    = param->out_data_fmt    ;  // Output PCM Data Format
        host_params->in_msb_pos      = param->in_msb_pos      ;  // Input PCM Sample Payload MSB Position (in bits)
        host_params->out_msb_pos     = param->out_msb_pos     ;  // Output PCM Sample Payload MSB Position (in bits)
        host_params->in_data_path    = param->in_data_path    ;  // Input Data Path (VoHCI / PCM / Other ...)
        host_params->out_data_path   = param->out_data_path   ;  // Output Data Path
        host_params->in_tr_unit_sz   = param->in_tr_unit_sz   ;  // Input Transport Unit Size (in bits)
        host_params->out_tr_unit_sz  = param->out_tr_unit_sz  ;  // Output Transport Unit Size (in bits)
        host_params->max_lat         = param->max_lat         ;  // Max Latency (in ms)
        host_params->retx_eff        = param->retx_eff        ;  // Retransmission Effort (No, opt power, opt quality, don't care)

        memcpy(&host_params->tx_cod_fmt, &param->tx_cod_fmt, sizeof(host_params->tx_cod_fmt));          // Transmit Coding Format
        memcpy(&host_params->rx_cod_fmt, &param->rx_cod_fmt, sizeof(host_params->rx_cod_fmt));          // Receive Coding Format
        memcpy(&host_params->in_cod_fmt, &param->in_cod_fmt, sizeof(host_params->in_cod_fmt));          // Input Coding Format
        memcpy(&host_params->out_cod_fmt, &param->out_cod_fmt, sizeof(host_params->out_cod_fmt));       // Output Coding Format

        // Packet Type (HV1 / HV2 / HV3 / EV3 / EV4 / EV5 / 2EV3 / 3EV3 / 2EV5 / 3EV5)
        host_params->packet_type       = param->packet_type ^ (SYNC_PACKET_TYPE_NO_EV3_2_FLAG |
                                         SYNC_PACKET_TYPE_NO_EV3_3_FLAG |
                                         SYNC_PACKET_TYPE_NO_EV5_2_FLAG |
                                         SYNC_PACKET_TYPE_NO_EV5_3_FLAG);

        // Check parameters
        status = lc_setup_sync_param_check(idx, host_params);

        // Send command status event
        lc_cmd_stat_send(opcode, status);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Trigger setup synchronous connection procedure
            struct lc_op_loc_sync_con_req *req = KE_MSG_ALLOC(LC_OP_LOC_SYNC_CON_REQ, dest_id, dest_id, lc_op_loc_sync_con_req);
            req->conhdl = param->conhdl;
            req->old_style = false;
            req->host_initiated = true;
            ke_msg_send(req);
        }
    }
    break;
    }

#else // (MAX_NB_SYNC > 0)
    lc_cmd_stat_send(opcode, CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED);
#endif // (MAX_NB_SYNC > 0)

    return (KE_MSG_CONSUMED);
}

/*
 * HCI LINK POLICY COMMANDS HANDLERS
 ****************************************************************************************
 */

/*
 * HCI CONTROL & BASEBAND COMMANDS HANDLERS
 ****************************************************************************************
 */

HCI_CMD_HANDLER_C(rd_auth_payl_to, struct hci_rd_auth_payl_to_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // allocate the complete event message
    struct hci_rd_auth_payl_to_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, hci_rd_auth_payl_to_cmd_cmp_evt);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
    {
        event->status = CO_ERROR_COMMAND_DISALLOWED;
    }
    break;
    default:
    {
        event->status = CO_ERROR_NO_ERROR;
        // The authenticated payload timeout is expressed in units of 10 ms
        event->auth_payl_to = lc_env_ptr->link.auth_payl_to;
    }
    break;
    }

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

#if RW_DEBUG
HCI_CMD_HANDLER_C(dbg_bt_send_lmp, struct hci_dbg_bt_send_lmp_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    int idx = KE_IDX_GET(dest_id);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    {
        // Nothing to do
    } break;
    case LC_WAIT_DBG_SEND_LMP_CFM:
    {
        return (KE_MSG_SAVED);
    }
    default:
    {
        // Transmit LMP
        struct bt_em_lmp_buf_elt *buf_elt = bt_util_buf_lmp_tx_alloc();
        em_wr(&param->buf.data, buf_elt->buf_ptr, param->buf.length);
        buf_elt->length = param->buf.length;
        ld_acl_lmp_tx(idx, buf_elt);
        ke_state_set(dest_id, LC_WAIT_DBG_SEND_LMP_CFM);
        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    // send command complete now if not waiting on the LMP response status
    if (CO_ERROR_NO_ERROR != status)
    {
        lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);
    }

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(dbg_bt_discard_lmp_en, struct hci_dbg_bt_discard_lmp_en_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    {
        // Nothing to do
    } break;
    default:
    {
        lc_env_ptr->link.LMPPacketDiscard = param->enable;
        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);
    return (KE_MSG_CONSUMED);
}
#endif //RW_DEBUG

HCI_CMD_HANDLER_C(wr_auth_payl_to, struct hci_wr_auth_payl_to_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // allocate the complete event message
    struct hci_wr_auth_payl_to_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, hci_wr_auth_payl_to_cmd_cmp_evt);

    event->status = CO_ERROR_COMMAND_DISALLOWED;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
        break;

    default:
    {
        /*
         * If the connection is in sniff mode, the Authenticated_Payload_Timeout shall be equal to or greater
         *  than Tsniff.
         * If the connection is in sniff subrating mode, the Authenticated_Payload_Timeout shall be equal to or
         *  greater than (max subrate)xTsniff.
         *
         * When the Connection_Handle identifies a BR/EDR synchronous connection, this command shall be rejected with
         *  the error code Command Disallowed (0x0C).
         *
         * The authenticated payload timeout is expressed in units of 10 ms (multiply by 16 to get units of 625 us)
         */
        if (param->conhdl == (BT_ACL_CONHDL_MIN + idx))
        {
            if (param->auth_payl_to < AUTH_PAYL_TO_MIN)
            {
                event->status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
            if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
            {
                uint16_t interval;
                uint8_t max_subrate;

                lc_sniff_interval_get(idx, &interval, &max_subrate);

                max_subrate = co_max(max_subrate, 1);

                if (param->auth_payl_to < co_slot_to_duration(max_subrate * interval))
                {
                    event->status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
            }

            event->status = CO_ERROR_NO_ERROR;
            lc_env_ptr->link.auth_payl_to = param->auth_payl_to;
            lc_env_ptr->link.auth_payl_to_margin = co_slot_to_duration(4 * POLL_INTERVAL_DFT);

            // If the timers are already running, reset them as per spec
            if (lc_env_ptr->enc.EncEnable && lc_env_ptr->sp.sec_con)
            {
                ke_timer_set(LC_AUTH_PAYL_NEARLY_TO, dest_id, 10 * (lc_env_ptr->link.auth_payl_to - lc_env_ptr->link.auth_payl_to_margin));
                ke_timer_set(LC_AUTH_PAYL_REAL_TO, dest_id, 10 * lc_env_ptr->link.auth_payl_to);
            }
        }
    }
    break;
    }
    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/*
 * HCI INFORMATIONAL PARAMETERS COMMANDS HANDLERS
 ****************************************************************************************
 */

/*
 * HCI STATUS PARAMETERS COMMANDS HANDLERS
 ****************************************************************************************
 */

HCI_CMD_HANDLER_C(rd_rssi, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);

    struct hci_rd_rssi_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_rssi_cmd_cmp_evt);
    event->rssi = ld_acl_rssi_delta_get(idx);
    event->status = CO_ERROR_NO_ERROR;
    event->conhdl = param->conhdl;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/*
 * HCI TESTING COMMANDS HANDLERS
 ****************************************************************************************
 */
/// Handle the command HCI write secure connections test mode
HCI_CMD_HANDLER_C(wr_sec_con_test_mode, struct hci_wr_sec_con_test_mode_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // allocate the complete event message
    struct hci_wr_sec_con_test_mode_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_WR_SEC_CON_TEST_MODE_CMD_OPCODE, hci_wr_sec_con_test_mode_cmd_cmp_evt);

    event->status = CO_ERROR_COMMAND_DISALLOWED;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
        break;

    default:
    {
        if (param->conhdl == (BT_ACL_CONHDL_MIN + idx))
        {
            if ((param->dm1_acl_u_mode > 1) || (param->esco_loopback_mode > 1))
            {
                event->status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Disable/enable use of DM1 packets for ACL-U traffic
            ld_acl_dm1_packet_type_dis(idx, param->dm1_acl_u_mode);

            // Store eSCO loopback mode
            lc_env_ptr->link.esco_loopback_mode = param->esco_loopback_mode;

#if (MAX_NB_SYNC > 0)
            // Enable/disable eSCO loopback mode (if eSCO link present)
            ld_esco_loopback_mode_en(idx, param->esco_loopback_mode);
#endif // (MAX_NB_SYNC > 0)

            event->status = CO_ERROR_NO_ERROR;
        }
    }
    break;
    }
    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/*
 * HCI VENDOR SPECIFIC COMMANDS HANDLERS
 ****************************************************************************************
 */
#if (EAVESDROPPING_SUPPORT)
/// Handle the command HCI VS set segmented page scan
HCI_CMD_HANDLER_C(vs_set_cust_acl_sched, struct hci_vs_set_cust_acl_sched_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    uint8_t link_id = KE_IDX_GET(dest_id);

    if (param->enable)
    {
        status = ld_acl_set_cust_evt_dur(link_id, param->evt_dur);
    }
    else
    {
        status = ld_acl_clr_cust_evt_dur(link_id);
    }

    // Send the command complete event
    lc_cmd_cmp_bd_addr_send(opcode, status, &param->bd_addr);

    return (KE_MSG_CONSUMED);
}
#endif //EAVESDROPPING_SUPPORT

HCI_CMD_HANDLER_C(qos_setup, struct hci_qos_setup_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    {
        uint8_t status = CO_ERROR_NO_ERROR;
        lc_env_ptr->link.TokenRate = param->tok_rate;
        lc_env_ptr->link.ServiceType = param->serv_type;
        lc_env_ptr->link.PeakBandwidth = param->pk_bw;
        lc_env_ptr->link.Latency = param->lat;
        lc_env_ptr->link.DelayVariation = param->del_var;

        status = LM_GetQoSParam(lc_env_ptr->link.Role, &lc_env_ptr->link.ServiceType,
                                &lc_env_ptr->link.TokenRate, &lc_env_ptr->link.PeakBandwidth, &lc_env_ptr->link.Latency,
                                &lc_env_ptr->link.DelayVariation, &lc_env_ptr->link.PollInterval, lc_env_ptr->link.CurPacketType,
                                QOS_HL_REQ);

        lc_cmd_stat_send(opcode, status);

        if (status == CO_ERROR_NO_ERROR)
        {
            ke_msg_send_basic(LC_OP_QOS_IND, dest_id, dest_id);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(auth_req, struct hci_basic_conhdl_cmd)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        break;
    default:
    {
        if (lm_get_sp_en())
        {
            lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
        }

        ke_msg_send_basic(LC_OP_AUTH_IND, dest_id, dest_id);

        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    // Send command status event
    lc_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(set_con_enc, struct hci_set_con_enc_cmd)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        break;
    default:
    {
        if (lc_env_ptr->sp.sec_con)
        {
            if (!param->enc_en)
            {
                status = CO_ERROR_ENC_MODE_NOT_ACCEPT;
                break;
            }

#if (MAX_NB_SYNC > 0)
            // AES encryption cannot be set if a SCO link is present
            if (lm_look_for_sync(idx, SCO_TYPE))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
#endif // (MAX_NB_SYNC > 0)
        }

        if (param->enc_en == lc_env_ptr->enc.EncEnable)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        status = CO_ERROR_NO_ERROR;

        lc_env_ptr->req.LocEncReq = true;

        // Check if encryption is activated or de-activated
        if (lc_env_ptr->enc.EncEnable == ENCRYPTION_OFF)
        {
            lc_env_ptr->enc.NewEncMode = ENC_PP_ENABLED;

            if (lc_env_ptr->enc.KeyStatus == KEY_ABSENT)
            {
                ke_msg_send_basic(LC_OP_AUTH_IND, dest_id, dest_id);
                break;
            }
        }
        else
        {
            lc_env_ptr->enc.NewEncMode = ENC_DISABLED;
        }

        ke_msg_send_basic(LC_OP_ENC_IND, dest_id, dest_id);
    }
    break;
    }

    // Send command status event
    lc_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_link_supv_to, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
    struct hci_rd_link_supv_to_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, conhdl, opcode, hci_rd_link_supv_to_cmd_cmp_evt);
    evt->status   = CO_ERROR_NO_ERROR;
    evt->conhdl   = conhdl;
    evt->lsto_val = ld_acl_lsto_get(idx);
    hci_send_2_host(evt);
    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(wr_link_supv_to, struct hci_wr_link_supv_to_cmd)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        break;
    default:
    {
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        if ((lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE) && (param->lsto_val))
        {
            uint16_t interval;
            uint8_t max_subrate;

            lc_sniff_interval_get(idx, &interval, &max_subrate);

            if (param->lsto_val <= interval)
            {
                lc_cmd_cmp_conhdl_send(opcode, CO_ERROR_INVALID_HCI_PARAM, param->conhdl);
                break;
            }

            if ((max_subrate != 0) && (param->lsto_val <= ((uint32_t)(interval * max_subrate))))
            {
                ld_acl_ssr_set(idx, 0, 0, 0);
                ke_msg_send_basic(LC_OP_SSRNEGO_IND, dest_id, dest_id);
            }
        }

        lc_env_ptr->link.LinkTimeout = param->lsto_val;
        lc_send_pdu_lsto(idx, lc_env_ptr->link.LinkTimeout, lc_env_ptr->link.Role);
        ld_acl_lsto_set(idx, lc_env_ptr->link.LinkTimeout);
        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    // Send command complete event
    lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(flow_spec, struct hci_flow_spec_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        // check direction of the flow, service type
        if ((param->flow_dir > FLOW_DIR_IN) || (param->serv_type > QOS_GUARANTEED))
        {
            lc_cmd_stat_send(opcode, CO_ERROR_INVALID_HCI_PARAM);
            break;
        }

        // get parameters from command
        lc_env_ptr->link.FlowDirection   = param->flow_dir;
        lc_env_ptr->link.ServiceType     = param->serv_type;
        lc_env_ptr->link.TokenRate       = param->tk_rate;
        lc_env_ptr->link.TokenBucketSize = param->tk_buf_sz;
        lc_env_ptr->link.PeakBandwidth   = param->pk_bw;
        lc_env_ptr->link.AccessLatency   = param->acc_lat;
        lc_env_ptr->link.Flags           = param->flags;

        if ((lc_env_ptr->link.Role == SLAVE_ROLE) ^ (lc_env_ptr->link.FlowDirection == FLOW_DIR_IN))
        {
            uint8_t status = LM_GetQoSParam(lc_env_ptr->link.Role,
                                            &lc_env_ptr->link.ServiceType, &lc_env_ptr->link.TokenRate,
                                            &lc_env_ptr->link.PeakBandwidth, &lc_env_ptr->link.AccessLatency,
                                            &lc_env_ptr->link.DelayVariation, &lc_env_ptr->link.PollInterval,
                                            lc_env_ptr->link.CurPacketType, QOS_HL_REQ);

            // Send command status event
            lc_cmd_stat_send(HCI_FLOW_SPEC_CMD_OPCODE, status);

            if (status == CO_ERROR_NO_ERROR)
            {
                // send LMP_QualityOfServiceReq(idx , PollInterval, nb_bcst , role)
                lc_send_pdu_qos_req(idx, lb_util_get_nb_broadcast() + 1, lc_env_ptr->link.PollInterval, lc_env_ptr->link.Role);
                lc_start_lmp_to(dest_id);

                // Indicate there is an ongoing flow spec nego
                ke_state_set(dest_id, LC_WAIT_FLOW_SPEC_CFM);
            }
        }
        else
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_flow_spec_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_FLOW_SPEC_CMP_EVT_CODE, hci_flow_spec_cmp_evt);

            // Send command status event
            lc_cmd_stat_send(HCI_FLOW_SPEC_CMD_OPCODE, CO_ERROR_NO_ERROR);

            evt->status = CO_ERROR_NO_ERROR;
            evt->conhdl = conhdl;
            evt->flags = 0;
            evt->flow_dir = lc_env_ptr->link.FlowDirection;
            evt->pk_bw = lc_env_ptr->link.PeakBandwidth;
            evt->serv_type = lc_env_ptr->link.ServiceType;
            evt->tk_buf_sz = lc_env_ptr->link.TokenBucketSize;
            evt->tk_rate = lc_env_ptr->link.TokenRate;
            evt->acc_lat = lc_env_ptr->link.AccessLatency;

            hci_send_2_host(evt);
        }
    }
    break;

    case LC_FREE:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;

    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(flush, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct hci_flush_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_flush_cmd_cmp_evt);
    evt->conhdl = param->conhdl;
    evt->status = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        hci_send_2_host(evt);
        break;

    default:
    {
        uint8_t nb_of_pkt_flushed = 0;

        hci_send_2_host(evt);

        lc_env_ptr->link.FlushContinue = true;

        // Both automatically-flushable and non-automatically-flushable packets shall be discarded
        ld_acl_data_flush(idx, &nb_of_pkt_flushed, true);

        // if the number of packets flushed is not NULL send a number of packets
        if (nb_of_pkt_flushed > 0)
        {
            lc_common_nb_of_pkt_comp_evt_send(param->conhdl, nb_of_pkt_flushed);
        }

        {
            // sends the flush occurred event
            struct hci_flush_occurred_evt *evt = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_FLUSH_OCCURRED_EVT_CODE, hci_flush_occurred_evt);
            evt->conhdl = param->conhdl;
            hci_send_2_host(evt);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(enh_flush, struct hci_enh_flush_cmd)
{
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;

    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];
        uint8_t nb_of_pkt_flushed = 0;

        // The only packet type accepted is automatically flushable (0x00)
        if (param->pkt_type)
        {
            lc_cmd_stat_send(opcode, CO_ERROR_INVALID_HCI_PARAM);
            break;
        }

        lc_env_ptr->link.FlushContinue = true;

        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

        ld_acl_data_flush(idx, &nb_of_pkt_flushed, false);

        // if the number of packet flushed is not NULL send a number of packets
        if (nb_of_pkt_flushed > 0)
        {
            lc_common_nb_of_pkt_comp_evt_send(param->conhdl, nb_of_pkt_flushed);
        }

        {
            // sends the enhanced flush complete event
            struct hci_basic_conhdl_evt *evt = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_ENH_FLUSH_CMP_EVT_CODE, hci_basic_conhdl_evt);
            evt->conhdl = param->conhdl;
            hci_send_2_host(evt);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_auto_flush_to, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct hci_rd_auto_flush_to_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_auto_flush_to_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->conhdl = param->conhdl;
    evt->flush_to = ld_acl_flush_timeout_get(idx);
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(wr_auto_flush_to, struct hci_wr_auto_flush_to_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    uint8_t status;

    if (param->flush_to > AUTO_FLUSH_TIMEOUT_MAX)
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        status = CO_ERROR_NO_ERROR;
        ld_acl_flush_timeout_set(idx, param->flush_to);
    }

    // Send the command complete event
    lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(chg_con_pkt_type, struct hci_chg_con_pkt_type_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    {
        if (BT_ACL_CONHDL_MIN + idx == param->conhdl)
        {
            // Mask packet types not supported by remote device
            lc_env_ptr->link.AclPacketType = LM_UpdateAclPacketType(param->pkt_type, lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[0]) |
                                             LM_UpdateAclEdrPacketType(param->pkt_type,
                                                     lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[3],
                                                     lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[4],
                                                     lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[5]);

            if (lc_env_ptr->link.AclPacketType == param->pkt_type)
            {
                // Force enabling DM1, as per standard
                lc_env_ptr->link.AclPacketType |= PACKET_TYPE_DM1_FLAG;

                lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);
                lc_env_ptr->req.LocCPTReq  = true;

                ke_msg_send_basic(LC_OP_PT_IND, dest_id, dest_id);
            }
            else
            {
                // send status - unsupported
                lc_cmd_stat_send(opcode, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE);
            }
        }
        else if (ke_state_get(dest_id) == LC_CONNECTED)
        {
            uint8_t status = CO_ERROR_NO_ERROR;

#if (MAX_NB_SYNC > 0)
            struct lc_sco_host_params_tag host_params;

            // Copy parameters
            host_params.max_lat = 0;
            host_params.packet_type = 0;
            // Transform old packet type to new packet type
            if (param->pkt_type & PACKET_TYPE_HV1_FLAG)
                host_params.packet_type |= SYNC_PACKET_TYPE_HV1_FLAG;
            if (param->pkt_type & PACKET_TYPE_HV2_FLAG)
                host_params.packet_type |= SYNC_PACKET_TYPE_HV2_FLAG;
            if (param->pkt_type & PACKET_TYPE_HV3_FLAG)
                host_params.packet_type |= SYNC_PACKET_TYPE_HV3_FLAG;
            host_params.retx_eff = 0;
            host_params.rx_bw = 0;
            host_params.tx_bw = 0;

            // Handle the request
            status = lc_sco_host_request(dest_id, true, true, param->conhdl, &host_params);
#else // (MAX_NB_SYNC > 0)
            status = CO_ERROR_UNKNOWN_CONNECTION_ID;
#endif // (MAX_NB_SYNC > 0)

            // send status
            lc_cmd_stat_send(opcode, status);
        }
        else
        {
            return (KE_MSG_SAVED);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_clk_off, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    {
        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            ke_msg_send_basic(LC_OP_CLKOFF_IND, dest_id, dest_id);
        }
        else
        {
            // Report the procedure completion
            struct hci_rd_clk_off_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_RD_CLK_OFF_CMP_EVT_CODE, hci_rd_clk_off_cmp_evt);
            event->status      = CO_ERROR_NO_ERROR;
            event->conhdl      = param->conhdl;
            event->clk_off_val = (ld_acl_clock_offset_get(idx).hs >> 2) & 0x7FFF;
            hci_send_2_host(event);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_tx_pwr_lvl, struct hci_rd_tx_pwr_lvl_cmd)
{
    // Allocate the command complete event message
    struct hci_rd_tx_pwr_lvl_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_tx_pwr_lvl_cmd_cmp_evt);
    event->conhdl     = param->conhdl;
    event->tx_pow_lvl = param->type;

#if (BT_PWR_CTRL)
    switch (param->type)
    {
    case MAX_TX_POWER:
        event->tx_pow_lvl = LD_TXPWR_DBM_GET(rwip_rf.txpwr_max, MOD_GFSK, rwip_rf.txpwr_max_mod);
        event->status = CO_ERROR_NO_ERROR;
        break;
    case CURRENT_TX_POWER:
        event->tx_pow_lvl = ld_acl_current_tx_power_get(KE_IDX_GET(dest_id), MOD_GFSK);
        event->status = CO_ERROR_NO_ERROR;
        break;
    default:
        event->status = CO_ERROR_INVALID_HCI_PARAM;
        break;
    }
#else // !(BT_PWR_CTRL)
    event->status = CO_ERROR_UNSUPPORTED;
#endif // !(BT_PWR_CTRL)

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_rem_ver_info, struct hci_rd_rem_ver_info_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
        // Send the command status event
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    default:
    {
        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

        if (lc_env_ptr->info.RecvRemVerRec)
        {
            // Send remote version information
            struct hci_rd_rem_ver_info_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_RD_REM_VER_INFO_CMP_EVT_CODE, hci_rd_rem_ver_info_cmp_evt);
            evt->status  = CO_ERROR_NO_ERROR;
            evt->conhdl  = param->conhdl;
            evt->compid  = lc_env_ptr->info.RemVers.compid;
            evt->subvers = lc_env_ptr->info.RemVers.subvers;
            evt->vers    = lc_env_ptr->info.RemVers.vers;
            hci_send_2_host(evt);
        }
        else
        {
            // Post the remote version operation
            lc_env_ptr->req.LocVersReq = true;
            ke_msg_send_basic(LC_OP_VERS_IND, dest_id, dest_id);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_rem_supp_feats, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    {
        // Send the command status event
        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);
        {
            // allocate the complete event message
            struct hci_rd_rem_supp_feats_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_RD_REM_SUPP_FEATS_CMP_EVT_CODE, hci_rd_rem_supp_feats_cmp_evt);
            // update the status
            event->status = CO_ERROR_NO_ERROR;
            event->conhdl = param->conhdl;
            memcpy(&event->rem_feats.feats[0], &lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[0], FEATS_LEN);

            // send the message
            hci_send_2_host(event);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_link_pol_stg, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct hci_rd_link_pol_stg_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode,
            hci_rd_link_pol_stg_cmd_cmp_evt);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        evt->status = CO_ERROR_NO_ERROR;
        evt->conhdl = param->conhdl;
        evt->lnk_policy = lc_env_ptr->link.LinkPolicySettings;
    }
    break;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}
HCI_CMD_HANDLER_C(wr_link_pol_stg, struct hci_wr_link_pol_stg_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        if (param->lnk_policy & 0xF0)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        if (param->lnk_policy & (POLICY_HOLD | POLICY_PARK))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        lc_env_ptr->link.LinkPolicySettings = param->lnk_policy;
        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    // Send the command complete event
    lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(sniff_mode, struct hci_sniff_mode_cmd)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        break;
    default :
    {
        status = CO_ERROR_INVALID_HCI_PARAM;

        /* First check the sniff parameters                                             */
        if ((param->max_int > SNIFF_INTERVAL_MAX) || ((param->max_int & 0x01) == 0x01) ||
                (param->min_int < SNIFF_INTERVAL_MIN) || ((param->min_int & 0x01) == 0x01) ||
                (param->attempt == 0)                 || (param->timeout > SNIFF_TIMEOUT_MAX)  ||
                (param->attempt > (param->max_int / 2)) || (param->min_int > param->max_int) ||
                ((uint32_t)(2 * (param->timeout + param->attempt)) >= param->max_int))
            break;

        status = CO_ERROR_COMMAND_DISALLOWED;

        if (!(lc_env_ptr->link.LinkPolicySettings & POLICY_SNIFF))
            break;

        if (lc_env_ptr->link.CurrentMode != LM_ACTIVE_MODE)
            break;

        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;

        if (!LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_SNIFF_BIT_POS))
            break;

        {
            // Trigger local sniff procedure
            struct lc_op_loc_sniff_req *req = KE_MSG_ALLOC(LC_OP_LOC_SNIFF_REQ, dest_id, dest_id, lc_op_loc_sniff_req);
            req->max_int = param->max_int;
            req->min_int = param->min_int;
            req->attempt = param->attempt;
            req->timeout = param->timeout;
            ke_msg_send(req);
        }

        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    // Send command status event
    lc_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(exit_sniff_mode, struct hci_basic_conhdl_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_UNSNIFF_ACC:
        break;
    case LC_FREE:
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
        {
            status = CO_ERROR_NO_ERROR;
            ke_msg_send_basic(LC_OP_LOC_UNSNIFF_REQ, dest_id, dest_id);
        }
    }
    break;
    }

    // Send command status event
    lc_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(sniff_sub, struct hci_sniff_sub_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        if (!LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_SNIFF_SUBRAT_BIT_POS))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        lc_sniff_ssr_host_request(dest_id, param->max_lat, param->min_rem_to, param->min_loc_to);

        status = CO_ERROR_NO_ERROR;
    }
    break;
    }

    // Send the command complete event
    lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(role_discovery, struct hci_basic_conhdl_cmd)
{
    struct hci_role_discovery_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode,
            hci_role_discovery_cmd_cmp_evt);
    evt->conhdl = param->conhdl;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        evt->status = CO_ERROR_NO_ERROR;
        evt->role = lc_env_ptr->link.Role;
    }
    break;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_fail_contact_cnt, struct hci_basic_conhdl_cmd)
{
    struct hci_rd_fail_contact_cnt_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode,
            hci_rd_fail_contact_cnt_cmd_cmp_evt);
    evt->conhdl = param->conhdl;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        evt->status = CO_ERROR_NO_ERROR;
        evt->fail_cnt = lc_env_ptr->link.FailedContact;
    }
    break;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rst_fail_contact_cnt, struct hci_basic_conhdl_cmd)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        lc_env_ptr->link.FailedContact = 0;
    }
    break;
    }

    lc_cmd_cmp_conhdl_send(opcode, status, param->conhdl);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_link_qual, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);

    struct hci_rd_link_qual_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode,
            hci_rd_link_qual_cmd_cmp_evt);
    event->quality = 0xFF - co_abs(ld_acl_rssi_delta_get(idx));
    event->conhdl = param->conhdl;
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_afh_ch_map, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    struct hci_rd_afh_ch_map_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode,
            hci_rd_afh_ch_map_cmd_cmp_evt);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
    {
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
    }
    break;
    default:
    {
        evt->status  = CO_ERROR_NO_ERROR;
        evt->conhdl = param->conhdl;
        evt->afh_mode = lc_env_ptr->afh.en ? AFH_ENABLED : AFH_DISABLED;
        memcpy(&evt->afh_map.map[0], &lc_env_ptr->afh.ch_map.map[0], BT_CH_MAP_LEN);
    }
    break;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_lmp_hdl, struct hci_basic_conhdl_cmd)
{
    // structure type for the complete command event
    struct hci_rd_lmp_hdl_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode,
            hci_rd_lmp_hdl_cmd_cmp_evt);

#if (MAX_NB_SYNC > 0)
    evt->lmp_hdl = lm_get_synchdl(BT_SYNC_CONHDL_LID(param->conhdl));
    if (evt->lmp_hdl)
    {
        evt->status = CO_ERROR_NO_ERROR;
    }
    else
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
#else //(MAX_NB_SYNC > 0)
    evt->status = CO_ERROR_UNSUPPORTED;
#endif //(MAX_NB_SYNC > 0)

    evt->rsvd = 0;
    evt->conhdl = param->conhdl;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}
HCI_CMD_HANDLER_C(rd_rem_ext_feats, struct hci_rd_rem_ext_feats_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
    case LC_CONNECTED:
    {
        if (param->pg_nb >= FEATURE_PAGE_MAX)
        {
            lc_cmd_stat_send(opcode, CO_ERROR_INVALID_HCI_PARAM);
        }
        else if (!LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_EXT_FEATS_BIT_POS))
        {
            lc_cmd_stat_send(opcode, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE);
        }
        else if ((lc_env_ptr->info.RemFeatureRec & (1 << param->pg_nb)))
        {
            // If remote features have been received, return the cached value
            lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);
            uint16_t conhdl = BT_ACL_CONHDL_MIN + idx;
            struct hci_rd_rem_ext_feats_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_EXT_FEATS_CMP_EVT_CODE, hci_rd_rem_ext_feats_cmp_evt);
            memcpy(&evt->ext_feats.feats[0], &lc_env_ptr->info.RemoteFeatures[param->pg_nb].feats[0], FEATS_LEN);
            evt->status    = CO_ERROR_NO_ERROR;
            evt->pg_nb     = param->pg_nb;
            evt->pg_nb_max = FEATURE_PAGE_MAX - 1;
            evt->conhdl    = conhdl;
            hci_send_2_host(evt);
        }
        else if (ke_state_get(dest_id) == LC_CONNECTED)
        {
            lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);
            lc_env_ptr->req.LocRemoteExtendedReq = true;

            lc_ext_feat(dest_id, idx, param->pg_nb);
        }
        else
        {
            return (KE_MSG_SAVED);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(rd_enc_key_size, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
    {
        // allocate the command complete event message
        struct hci_rd_enc_key_size_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_enc_key_size_cmd_cmp_evt);
        evt->conhdl = param->conhdl;
        evt->key_sz = 0;
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
        // send the message
        hci_send_2_host(evt);
    }
    break;
    default:
    {
        // allocate the command complete event message
        struct hci_rd_enc_key_size_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_enc_key_size_cmd_cmp_evt);
        evt->conhdl = param->conhdl;

        if (lc_env_ptr->link.CurrentMode == LM_ACTIVE_MODE)
        {
            if (lc_env_ptr->enc.EncMode == ENC_DISABLED)
            {
                evt->status = CO_ERROR_INSUFFICIENT_SECURITY;
                evt->key_sz = 0;
            }
            else
            {
                // update the status
                evt->status = CO_ERROR_NO_ERROR;
                evt->key_sz = lc_env_ptr->enc.EncSize;
            }
        }
        else
        {
            // update the status
            evt->status = CO_ERROR_COMMAND_DISALLOWED;
            evt->key_sz = 0;
        }

        // send the message
        hci_send_2_host(evt);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

#if (BT_PWR_CTRL)
HCI_CMD_HANDLER_C(rd_enh_tx_pwr_lvl, struct hci_rd_enh_tx_pwr_lvl_cmd)
{
    int idx = KE_IDX_GET(dest_id);

    // Allocate the command complete event message
    struct hci_rd_enh_tx_pwr_lvl_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_enh_tx_pwr_lvl_cmd_cmp_evt);
    // gets connection handle
    evt->conhdl = param->conhdl;

    switch (param->type)
    {
    case MAX_TX_POWER:
        evt->pw_gfsk  = LD_TXPWR_DBM_GET(rwip_rf.txpwr_max, MOD_GFSK, rwip_rf.txpwr_max_mod);
        evt->pw_dqpsk = LD_TXPWR_DBM_GET(rwip_rf.txpwr_max, MOD_DQPSK, rwip_rf.txpwr_max_mod);
        evt->pw_8dpsk = LD_TXPWR_DBM_GET(rwip_rf.txpwr_max, MOD_8DPSK, rwip_rf.txpwr_max_mod);
        evt->status   = CO_ERROR_NO_ERROR;
        break;
    case CURRENT_TX_POWER:
        evt->pw_gfsk  = ld_acl_current_tx_power_get(idx, MOD_GFSK);
        evt->pw_dqpsk = ld_acl_current_tx_power_get(idx, MOD_DQPSK);
        evt->pw_8dpsk = ld_acl_current_tx_power_get(idx, MOD_8DPSK);
        evt->status   = CO_ERROR_NO_ERROR;
        break;
    default:
        evt->status   = CO_ERROR_INVALID_HCI_PARAM;
        evt->pw_gfsk  = 0;
        evt->pw_dqpsk = 0;
        evt->pw_8dpsk = 0;
        break;
    }

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}
#endif // (BT_PWR_CTRL)

HCI_CMD_HANDLER_C(refresh_enc_key, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if ((lc_env_ptr->enc.EncMode != ENC_DISABLED) && (!lc_env_ptr->epr.on))
        {
            if (!LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_PAUSE_ENCRYPT_BIT_POS))
            {
                lc_cmd_stat_send(opcode, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE);
                break;
            }

            lc_env_ptr->epr.on = true;
            lc_env_ptr->req.LocEncKeyRefresh = true;
            lc_env_ptr->link.Initiator = true;
            lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);
            if (lc_env_ptr->epr.rsw)
            {
                lc_locepr_rsw(dest_id);
            }
            else
            {
                lc_locepr_lkref(dest_id);
            }
        }
        else
        {
            lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        lc_cmd_stat_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

HCI_CMD_HANDLER_C(chg_con_lk, struct hci_basic_conhdl_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (lc_env_ptr->epr.on)
        {
            lc_cmd_stat_send(opcode, CO_ERROR_LMP_COLLISION);
        }
        else
        {
            uint8_t status = CO_ERROR_NO_ERROR;
            if (lc_env_ptr->enc.KeyStatus == KEY_PRESENT)
            {
                if (lm_get_sp_en())
                {
                    lc_env_ptr->enc.KeyType = BT_CHANGED_COMB_KEY;
                }
                lc_env_ptr->req.LocKeyExchangeReq = true;
                if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
                {
                    lc_env_ptr->epr.cclk = true;
                    lc_env_ptr->epr.on = true;
                }
            }
            else
            {
                status = CO_ERROR_PIN_MISSING;
            }
            lc_cmd_stat_send(opcode, status);
            if (status == CO_ERROR_NO_ERROR)
            {
                lc_env_ptr->link.Initiator = true;
                lc_env_ptr->enc.key_from_host = false;
                lc_start_key_exch(dest_id);
            }
        }
    }
    break;
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
        // Send the command status event
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI accept connection request.
HCI_CMD_HANDLER_C(accept_con_req, struct hci_accept_con_req_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    {
        struct lc_sco_host_params_tag host_params =
        {
            SYNC_BANDWIDTH_DONT_CARE,                // Transmit Bandwidth (in B/sec)
            SYNC_BANDWIDTH_DONT_CARE,                // Receive Bandwidth (in B/sec)
            {CODING_FORMAT_CVSD, 0, 0, 0, 0},        // Transmit Coding Format
            {CODING_FORMAT_CVSD, 0, 0, 0, 0},        // Receive Coding Format
            0,         // Transmit Codec Frame Size (in B)
            0,         // Receive Codec Frame Size (in B)
            SYNC_BANDWIDTH_DONT_CARE,                // Input Bandwidth (in B/sec)
            SYNC_BANDWIDTH_DONT_CARE,               // Output Bandwidth (in B/sec)
            {CODING_FORMAT_LINPCM, 0, 0, 0, 0},        // Input Coding Format
            {CODING_FORMAT_LINPCM, 0, 0, 0, 0},       // Output Coding Format
            PCM_SAMPLE_SIZE_8BITS,       // Input Coded Data Size (in bits)
            PCM_SAMPLE_SIZE_8BITS,      // Output Coded Data Size (in bits)
            PCM_FORMAT_2SCOMP,          // Input PCM Data Format
            PCM_FORMAT_2SCOMP,         // Output PCM Data Format
            0,           // Input PCM Sample Payload MSB Position (in bits)
            0,          // Output PCM Sample Payload MSB Position (in bits)
            AUDIO_DATA_PATH_HCI,         // Input Data Path (VoHCI / PCM / Other ...)
            AUDIO_DATA_PATH_HCI,        // Output Data Path
            0,        // Input Transport Unit Size (in bits)
            0,       // Output Transport Unit Size (in bits)
            SYNC_DONT_CARE_LATENCY,              // Max Latency (in ms)
            (SYNC_PACKET_TYPE_HV1_FLAG | SYNC_PACKET_TYPE_HV2_FLAG |
             SYNC_PACKET_TYPE_HV3_FLAG | SYNC_PACKET_TYPE_NO_EV3_2_FLAG |
             SYNC_PACKET_TYPE_NO_EV3_3_FLAG | SYNC_PACKET_TYPE_NO_EV5_2_FLAG |
             SYNC_PACKET_TYPE_NO_EV5_3_FLAG),          // Packet Type (HV1 / HV2 / HV3 / EV3 / EV4 / EV5 / 2EV3 / 3EV3 / 2EV5 / 3EV5)
            SYNC_NO_RE_TX,             // Retransmission Effort (No, opt power, opt quality, don't care)
        };

        // Handle the response
        lc_sco_host_accept(dest_id, opcode, &host_params);
    }
    break;
#endif // (MAX_NB_SYNC > 0)
    case LC_WAIT_ACL_ACC:
    {
        ke_timer_clear(LC_CON_ACCEPT_TO, dest_id);
        ke_state_set(dest_id, LC_CONNECTED);

        // Reply to Host that command will be processed
        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

        //Check if remote features (page 0 or 1) have not been received
        if ((lc_env_ptr->info.RemFeatureRec & 0x3) != 0x3)
        {
            //Request the remote and/or extended peer features if it is not done
            ke_msg_send_basic(LC_OP_FEAT_IND, dest_id, dest_id);
        }

        do
        {
            // Check if a role switch shall be started
            if (param->role == ACCEPT_SWITCH_TO_MASTER)
            {
                // Register the Role Switch at LM
                if (lm_role_switch_start(idx, &lc_env_ptr->link.LtAddr))
                {
                    lc_env_ptr->req.LocSwitchReq = true;
                    lc_env_ptr->link.Initiator = true;
                    lc_local_switch(dest_id);
                    break;
                }
            }

            // LMP_Accepted(LMP_HOST_CON_REQ_OPCODE)
            lc_env_ptr->link.ConnectedState = true;
            lc_send_pdu_acc(idx, LMP_HOST_CON_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);
        }
        while (0);

        //Check if authentication should be started
        if (lm_get_auth_en() == AUTH_ENABLED)
        {
            ke_msg_send_basic(LC_OP_AUTH_IND, dest_id, dest_id);
        }
        ke_msg_send_basic(LC_OP_SET_CMP_IND, dest_id, dest_id);
    }
    break;
    default:
        // Reply to Host that command can not be processed
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI reject connection request.
HCI_CMD_HANDLER_C(reject_con_req, struct hci_reject_con_req_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // BT standard specification only accepts 0x0D-0x0F as reason (HCI:7.1.9)
    switch (param->reason)
    {
    case CO_ERROR_CONN_REJ_LIMITED_RESOURCES:
    case CO_ERROR_CONN_REJ_SECURITY_REASONS:
    case CO_ERROR_CONN_REJ_UNACCEPTABLE_BDADDR:
        break;
    default:
    {
        // Send command status event
        lc_cmd_stat_send(opcode, CO_ERROR_INVALID_HCI_PARAM);

        return (KE_MSG_CONSUMED);
    }
    break;
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    {
        // Send command status event
        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

        // Handle the response
        lc_sco_host_reject(dest_id, true, param->reason);
    }
    break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_ACL_ACC:
    {
        // Reply to Host that command will be processed
        lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

        ke_timer_clear(LC_CON_ACCEPT_TO, dest_id);

        // send LMP_NotAccepted(LMP_HOST_CON_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_HOST_CON_REQ_OPCODE, param->reason, lc_env_ptr->link.RxTrIdServer);

        lc_detach(dest_id, param->reason);
    }
    break;

    default:
        // Reply to Host that command can not be processed
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI accept synchronous connection request.
HCI_CMD_HANDLER_C(accept_sync_con_req, struct hci_accept_sync_con_req_cmd)
{
    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    {
        uint8_t air_coding, hci_coding, pcm_samp_size, pcm_data_fmt, pcm_msb_pos;
        struct lc_sco_host_params_tag host_params;

        // Decode voice setting field (air coding)
        switch (param->vx_set & AIR_COD_MSK)
        {
        case AIR_COD_MULAW:
            air_coding = CODING_FORMAT_ULAW;
            break;
        case AIR_COD_ALAW:
            air_coding = CODING_FORMAT_ALAW;
            break;
        case AIR_COD_TRANS:
            air_coding = CODING_FORMAT_TRANSP;
            break;
        case AIR_COD_CVSD:
        default:
            air_coding = CODING_FORMAT_CVSD;
            break;
        }
        // Decode voice setting field (input coding)
        switch (param->vx_set & INPUT_COD_MSK)
        {
        case INPUT_COD_MULAW:
            hci_coding = CODING_FORMAT_ULAW;
            break;
        case INPUT_COD_ALAW:
            hci_coding = CODING_FORMAT_ALAW;
            break;
        case INPUT_COD_LIN:
        default:
            hci_coding = CODING_FORMAT_LINPCM;
            break;
        }
        // Decode voice setting field (input sample size)
        switch (param->vx_set & INPUT_SAMPSIZE_MSK)
        {
        case INPUT_SAMP_16BIT:
            pcm_samp_size = PCM_SAMPLE_SIZE_16BITS;
            break;
        case INPUT_SAMP_8BIT:
        default:
            pcm_samp_size = PCM_SAMPLE_SIZE_8BITS;
            break;
        }
        // Decode voice setting field (input data format)
        switch (param->vx_set & INPUT_DATAFORM_MSK)
        {
        case INPUT_DATA_1COMP:
            pcm_data_fmt = PCM_FORMAT_1SCOMP;
            break;
        case INPUT_DATA_SMAG:
            pcm_data_fmt = PCM_FORMAT_SIGNMAG;
            break;
        case INPUT_DATA_UNSIGNED:
            pcm_data_fmt = PCM_FORMAT_UNSIGNED;
            break;
        case INPUT_DATA_2COMP:
        default:
            pcm_data_fmt = PCM_FORMAT_2SCOMP;
            break;
        }
        // Decode voice setting field (msb position)
        pcm_msb_pos = ((param->vx_set & LIN_PCM_BIT_POS_MSK) >> LIN_PCM_BIT_POS_OFF);

        // Transmit/Receive Bandwidth (in B/sec)
        host_params.tx_bw             = param->tx_bw;
        host_params.rx_bw             = param->rx_bw;

        // Transmit/Receive Coding Format
        memset(&host_params.tx_cod_fmt[0], 0, sizeof(host_params.tx_cod_fmt));
        host_params.tx_cod_fmt[0]    = air_coding;
        memset(&host_params.rx_cod_fmt[0], 0, sizeof(host_params.rx_cod_fmt));
        host_params.rx_cod_fmt[0]    = air_coding;

        // Transmit/Receive Codec Frame Size (in B)
        host_params.tx_cod_fr_sz     = 0;
        host_params.rx_cod_fr_sz     = 0;

        // Input/Output Bandwidth (in B/sec)
        host_params.in_bw            = param->tx_bw;
        host_params.out_bw           = param->rx_bw;

        // Input/Output Coding Format
        memset(&host_params.in_cod_fmt[0], 0, sizeof(host_params.in_cod_fmt));
        host_params.in_cod_fmt[0]    = hci_coding;
        memset(&host_params.out_cod_fmt[0], 0, sizeof(host_params.out_cod_fmt));
        host_params.out_cod_fmt[0]   = hci_coding;

        // Input/Output Coded Data Size (in bits)
        host_params.in_cod_data_sz   = pcm_samp_size;
        host_params.out_cod_data_sz  = pcm_samp_size;

        // Input/Output PCM Data Format
        host_params.in_data_fmt      = pcm_data_fmt;
        host_params.out_data_fmt     = pcm_data_fmt;

        // Input/Output PCM Sample Payload MSB Position (in bits)
        host_params.in_msb_pos       = pcm_msb_pos;
        host_params.out_msb_pos      = pcm_msb_pos;

        // Input/Output Data Path (VoHCI / PCM / Other ...)
        host_params.in_data_path     = AUDIO_DATA_PATH_HCI;
        host_params.out_data_path    = AUDIO_DATA_PATH_HCI;

        // Input/Output Transport Unit Size (in bits)
        host_params.in_tr_unit_sz    = 0;
        host_params.out_tr_unit_sz   = 0;

        // Max Latency (in ms)
        host_params.max_lat           = param->max_lat;
        // Packet Type (HV1 / HV2 / HV3 / EV3 / EV4 / EV5 / 2EV3 / 3EV3 / 2EV5 / 3EV5)
        host_params.packet_type       = param->pkt_type ^ (SYNC_PACKET_TYPE_NO_EV3_2_FLAG |
                                        SYNC_PACKET_TYPE_NO_EV3_3_FLAG |
                                        SYNC_PACKET_TYPE_NO_EV5_2_FLAG |
                                        SYNC_PACKET_TYPE_NO_EV5_3_FLAG);
        // Retransmission Effort (No, opt power, opt quality, don't care)
        host_params.retx_eff          = param->retx_eff;

        // Handle the response
        lc_sco_host_accept(dest_id, opcode, &host_params);

        // Save the host parameters
        lc_env[KE_IDX_GET(dest_id)]->sco = host_params;
    }
    break;
#endif // (MAX_NB_SYNC > 0)
    default:
        // Reply to Host that command can not be processed
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI enhanced accept synchronous connection request.
HCI_CMD_HANDLER_C(enh_accept_sync_con, struct hci_enh_accept_sync_con_cmd)
{
    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    {
        struct lc_sco_host_params_tag host_params;

        host_params.tx_bw           = param->tx_bw           ;  // Transmit Bandwidth (in B/sec)
        host_params.rx_bw           = param->rx_bw           ;  // Receive Bandwidth (in B/sec)
        host_params.tx_cod_fr_sz    = param->tx_cod_fr_sz    ;  // Transmit Codec Frame Size (in B)
        host_params.rx_cod_fr_sz    = param->rx_cod_fr_sz    ;  // Receive Codec Frame Size (in B)
        host_params.in_bw           = param->in_bw           ;  // Input Bandwidth (in B/sec)
        host_params.out_bw          = param->out_bw          ;  // Output Bandwidth (in B/sec)
        host_params.in_cod_data_sz  = param->in_cod_data_sz  ;  // Input Coded Data Size (in bits)
        host_params.out_cod_data_sz = param->out_cod_data_sz ;  // Output Coded Data Size (in bits)
        host_params.in_data_fmt     = param->in_data_fmt     ;  // Input PCM Data Format
        host_params.out_data_fmt    = param->out_data_fmt    ;  // Output PCM Data Format
        host_params.in_msb_pos      = param->in_msb_pos      ;  // Input PCM Sample Payload MSB Position (in bits)
        host_params.out_msb_pos     = param->out_msb_pos     ;  // Output PCM Sample Payload MSB Position (in bits)
        host_params.in_data_path    = param->in_data_path    ;  // Input Data Path (VoHCI / PCM / Other ...)
        host_params.out_data_path   = param->out_data_path   ;  // Output Data Path
        host_params.in_tr_unit_sz   = param->in_tr_unit_sz   ;  // Input Transport Unit Size (in bits)
        host_params.out_tr_unit_sz  = param->out_tr_unit_sz  ;  // Output Transport Unit Size (in bits)
        host_params.max_lat         = param->max_lat         ;  // Max Latency (in ms)
        host_params.retx_eff        = param->retx_eff        ;  // Retransmission Effort (No, opt power, opt quality, don't care)

        memcpy(&host_params.tx_cod_fmt, &param->tx_cod_fmt, sizeof(host_params.tx_cod_fmt));          // Transmit Coding Format
        memcpy(&host_params.rx_cod_fmt, &param->rx_cod_fmt, sizeof(host_params.rx_cod_fmt));          // Receive Coding Format
        memcpy(&host_params.in_cod_fmt, &param->in_cod_fmt, sizeof(host_params.in_cod_fmt));          // Input Coding Format
        memcpy(&host_params.out_cod_fmt, &param->out_cod_fmt, sizeof(host_params.out_cod_fmt));       // Output Coding Format

        // Packet Type (HV1 / HV2 / HV3 / EV3 / EV4 / EV5 / 2EV3 / 3EV3 / 2EV5 / 3EV5)
        host_params.packet_type       = param->packet_type ^ (SYNC_PACKET_TYPE_NO_EV3_2_FLAG |
                                        SYNC_PACKET_TYPE_NO_EV3_3_FLAG |
                                        SYNC_PACKET_TYPE_NO_EV5_2_FLAG |
                                        SYNC_PACKET_TYPE_NO_EV5_3_FLAG);

        // Handle the response
        lc_sco_host_accept(dest_id, opcode, &host_params);

        // Save the host parameters
        lc_env[KE_IDX_GET(dest_id)]->sco = host_params;
    }
    break;
#endif // (MAX_NB_SYNC > 0)
    default:
        // Reply to Host that command can not be processed
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI reject synchronous connection request.
HCI_CMD_HANDLER_C(reject_sync_con_req, struct hci_reject_sync_con_req_cmd)
{
    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    {
        // BT standard specification only accepts 0x0D-0x0F as reason (HCI:7.1.9)
        switch (param->reason)
        {
        case CO_ERROR_CONN_REJ_LIMITED_RESOURCES:
        case CO_ERROR_CONN_REJ_SECURITY_REASONS:
        case CO_ERROR_CONN_REJ_UNACCEPTABLE_BDADDR:
        {
            // Send command status event
            lc_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

            // Handle the response
            lc_sco_host_reject(dest_id, false, param->reason);
        }
        break;
        default:
        {
            // Send command status event
            lc_cmd_stat_send(opcode, CO_ERROR_INVALID_HCI_PARAM);
        }
        break;
        }
    }
    break;
#endif // (MAX_NB_SYNC > 0)

    default:
        // Reply to Host that command can not be processed
        lc_cmd_stat_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI long term key request reply request.
HCI_CMD_HANDLER_C(lk_req_reply, struct hci_lk_req_reply_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_LINK_KEY:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_LK_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        // timer clear
        ke_timer_clear(LC_QUERY_HOST_EXP_IND, dest_id);

        // stored received values
        memcpy(&lc_env_ptr->enc.LTKey.ltk[0], &param->key.ltk[0], KEY_LEN);

        // Key set to PRESENT
        lc_env_ptr->enc.KeyStatus = KEY_PRESENT;
        lc_env_ptr->enc.key_from_host = true;

        if (lc_env_ptr->req.PeerAuthReq)
        {

            if (lc_env_ptr->sp.sec_con)
            {
                lc_resp_sec_auth(dest_id);
            }
            else
            {
                // Compute SRES and ACO
                E1(lc_env_ptr->enc.LTKey,
                   lc_env_ptr->info.LocalBdAddr,
                   lc_env_ptr->enc.RandomRx,
                   &lc_env_ptr->enc.Sres,
                   &lc_env_ptr->enc.Aco);

                // send LMP_Sres, reply to previous LMP_AuRand
                lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
                lc_auth_cmp(dest_id, CO_ERROR_NO_ERROR);
            }
        }

        if (lc_env_ptr->req.LocAuthReq)
        {
            lc_loc_auth(dest_id, idx);
        }
    }
    break;
    default:
    {
        lc_cmd_cmp_bd_addr_send(HCI_LK_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI long term key negative reply request.
HCI_CMD_HANDLER_C(lk_req_neg_reply, struct hci_basic_bd_addr_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_LINK_KEY:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_LK_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        // reset the timer
        ke_timer_clear(LC_QUERY_HOST_EXP_IND, dest_id);

        // Key set to ABSENT
        lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
        lc_env_ptr->enc.PinStatus = PIN_ABSENT;
        lc_env_ptr->enc.key_from_host = false;

        if (lc_env_ptr->req.PeerAuthReq)
        {
            lc_env_ptr->req.PeerAuthReq = false;
            lc_env_ptr->req.LocAuthReq = false;
            //  send Not accepted - PIN missing
            lc_send_pdu_not_acc(idx, LMP_AURAND_OPCODE, CO_ERROR_PIN_MISSING, lc_env_ptr->link.RxTrIdServer);

            // back to previous state
            ke_state_set(dest_id, LC_CONNECTED);
        }
        else
        {
            lc_pair(dest_id, idx);
        }
    }
    break;
    default:
    {
        lc_cmd_cmp_bd_addr_send(HCI_LK_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI PIN code request reply request.
HCI_CMD_HANDLER_C(pin_code_req_reply, struct hci_pin_code_req_reply_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PIN_CODE:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_PIN_CODE_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        lc_env_ptr->enc.PinLength = param->pin_len;
        memcpy(&lc_env_ptr->enc.PinCode.pin[0], &param->pin.pin[0], param->pin_len);

        // PIN Status
        lc_env_ptr->enc.PinStatus = PIN_PRESENT;

        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // initiate pairing
        lc_pairing_cont(dest_id);
    }
    break;
    default:
    {
        lc_cmd_cmp_bd_addr_send(HCI_PIN_CODE_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI PIN code request negative reply request.
HCI_CMD_HANDLER_C(pin_code_req_neg_reply, struct hci_basic_bd_addr_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t reason = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PIN_CODE:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_PIN_CODE_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        reason = CO_ERROR_PAIRING_NOT_ALLOWED;
        if (!lc_env_ptr->link.Initiator)
        {
            // send LMP_NotAcceptedExt(LMP_INRAND_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_INRAND_OPCODE, reason, lc_env_ptr->link.RxTrIdServer);
        }
        if (lc_env_ptr->req.LocAuthReq && lc_conn_seq_done(dest_id))
        {
            uint16_t conhdl = BT_ACL_CONHDL_MIN + idx;
            struct hci_auth_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_CMP_EVT_CODE, hci_auth_cmp_evt);
            evt->status = reason;
            evt->conhdl = conhdl;
            hci_send_2_host(evt);
            lc_env_ptr->req.LocAuthReq = false;
            lc_env_ptr->link.Initiator  = false;
        }

        if (lc_env_ptr->link.SetupComplete)
        {
            // back to previous state
            ke_state_set(dest_id, LC_CONNECTED);
        }
        else
        {
            lc_detach(dest_id, reason);
        }
    }
    break;
    default:
    {
        lc_cmd_cmp_bd_addr_send(HCI_PIN_CODE_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI role switch request.
HCI_CMD_HANDLER_C(switch_role, struct hci_switch_role_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_WAIT_SWITCH_CFM:
    case LC_WAIT_SWITCH_CMP:
        break;
    default:
    {
        if (param->role != lc_env_ptr->link.Role)
        {
            if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_SWITCH_BIT_POS))
            {
                if ((lc_env_ptr->link.CurrentMode != LM_SNIFF_MODE) && (lc_env_ptr->link.LinkPolicySettings & POLICY_SWITCH))
                {
#if (MAX_NB_SYNC > 0)
                    if (lm_look_for_sync(idx, ACL_TYPE))
                        break;
#endif // (MAX_NB_SYNC > 0)

#if BCAST_ENC_SUPPORT
                    // disallowed if broadcast encryption enabled
                    if (lc_env_ptr->enc.KeyFlag == TEMPORARY_KEY)
                        break;
#endif // BCAST_ENC_SUPPORT

                    // Trigger role switch procedure
                    status = CO_ERROR_NO_ERROR;
                    ke_msg_send_basic(LC_OP_LOC_SWITCH_REQ, dest_id, dest_id);
                }
            }
            else
            {
                status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            }
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
    }
    break;
    }

    // Send command status event
    lc_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read clock
HCI_CMD_HANDLER_C(rd_clk, struct hci_rd_clk_cmd)
{
    struct hci_rd_clk_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_rd_clk_cmd_cmp_evt);
    evt->conhdl = param->conhdl;


    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];
        uint32_t clock;

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            clock = ld_read_clock();
        }
        else
        {
            clock = CLK_ADD_2(ld_read_clock(), ld_acl_clock_offset_get(idx).hs); // Get Master Clock
        }

        evt->status = CO_ERROR_NO_ERROR;
        evt->clk = clock;
        evt->clk_acc = CLOCK_ACCURACY_UNKNOWN;
    }
    break;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI remote name request cancel
HCI_CMD_HANDLER_C(rem_name_req, struct hci_rem_name_req_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        lc_env_ptr->req.LocNameReq = true;
        lc_rd_rem_name(dest_id);
    }
    break;
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    case LC_FREE:
    {
        // The disconnection is happening, or happened during the LM->LC command transfer, report completion of the procedure
        struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        memcpy(&evt->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(evt);
    }
    break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI remote name request cancel
HCI_CMD_HANDLER_C(rem_name_req_cancel, struct hci_basic_bd_addr_cmd)
{
    // If remote name request reach LC, device has already been paged, it is considered too late to cancel
    lc_cmd_cmp_bd_addr_send(opcode, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI IO capability request reply request.
HCI_CMD_HANDLER_C(io_cap_req_reply, struct hci_io_cap_req_reply_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_HL_IO_CAP_RSP:
    case LC_WAIT_HL_IO_CAP_INIT:
    {
        lc_env_ptr->sp.IOCap_loc.io_cap = param->io_capa;
        if (lc_env_ptr->sp.sec_con)
        {
            lc_env_ptr->sp.IOCap_loc.oob_data_present = (param->oob_data_pres == REM_OOB_DATA_P256) || (param->oob_data_pres == REM_OOB_DATA_P192_P256);
        }
        else
        {
            lc_env_ptr->sp.IOCap_loc.oob_data_present = (param->oob_data_pres == REM_OOB_DATA_P192) || (param->oob_data_pres == REM_OOB_DATA_P192_P256);
        }
        lc_env_ptr->sp.IOCap_loc.aut_req  = param->auth_req;

        if (ke_state_get(dest_id) == LC_WAIT_HL_IO_CAP_RSP)
        {
            // send LMP_IOCapabilityRes
            lc_send_pdu_io_cap_res(idx);

            ke_state_set(dest_id, LC_WAIT_PUB_KEY_HEADER_RSP_PEER);
        }
        else
        {
            struct lmp_io_cap_req pdu;

            pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->sp.SpTId);
            pdu.ext_opcode = LMP_IO_CAP_REQ_EXTOPCODE;
            pdu.auth_req   = lc_env_ptr->sp.IOCap_loc.aut_req ;
            pdu.io_cap     = lc_env_ptr->sp.IOCap_loc.io_cap;
            pdu.oob_auth_data = lc_env_ptr->sp.IOCap_loc.oob_data_present;
            lc_send_lmp(idx, &pdu);

            ke_state_set(dest_id, LC_WAIT_IO_CAP_INIT_CFM);
        }

        // restart the timer
        lc_start_lmp_to(dest_id);

        status = CO_ERROR_NO_ERROR;
    }
    break;

    default:
        status = CO_ERROR_COMMAND_DISALLOWED;
        break;
    }

    // Send HCI CC event
    lc_cmd_cmp_bd_addr_send(HCI_IO_CAP_REQ_REPLY_CMD_OPCODE, status, &param->bd_addr);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI IO capability request negative reply request.
HCI_CMD_HANDLER_C(io_cap_req_neg_reply, struct hci_io_cap_req_neg_reply_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_HL_IO_CAP_RSP:
    {
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        // send LMP_NotAccepted(LMP_IOCAP_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, param->reason, lc_env_ptr->sp.SpTId);
    } // No break
    case LC_WAIT_HL_IO_CAP_INIT:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_IO_CAP_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        lc_sp_fail(dest_id);
    }
    break;
    default:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_IO_CAP_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI user confirmation request reply request.
HCI_CMD_HANDLER_C(user_cfm_req_reply, struct hci_basic_bd_addr_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_NUM_COMP_USER_CONF_INIT:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_CFM_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        memset(&lc_env_ptr->sp.RemCommitment.A[0], 0x00, sizeof(struct byte16));
        memset(&lc_env_ptr->sp.LocCommitment.A[0], 0x00, sizeof(struct byte16));
        /*
         * SP AUTH STG1 : END
         */
        /*
         * SP AUTH STG2 : START
         */
        lc_try_start_auth_stage2(dest_id);
    }
    break;
    case LC_WAIT_NUM_COMP_USER_CONF_RSP:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_CFM_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        lc_env_ptr->sp.SPPhase1Failed = 0;
        memset(&lc_env_ptr->sp.RemCommitment.A[0], 0x00, sizeof(struct byte16));
        memset(&lc_env_ptr->sp.LocCommitment.A[0], 0x00, sizeof(struct byte16));
        /*
         * SP AUTH STG1 : END
         */
        /*
         * SP AUTH STG2 : START
         */
        lc_try_start_auth_stage2(dest_id);
    }
    break;
    default:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_CFM_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI user confirmation request negative reply request.
HCI_CMD_HANDLER_C(user_cfm_req_neg_reply, struct hci_basic_bd_addr_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_NUM_COMP_USER_CONF_RSP:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_CFM_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        lc_env_ptr->sp.SPPhase1Failed = 1;

        memset(&lc_env_ptr->sp.RemCommitment.A[0], 0x00, sizeof(struct byte16));
        memset(&lc_env_ptr->sp.LocCommitment.A[0], 0x00, sizeof(struct byte16));

        lc_start_lmp_to(dest_id);
        ke_state_set(dest_id, LC_WAIT_DHKEY_RSP_FAIL);
    }
    break;
    case LC_WAIT_NUM_COMP_USER_CONF_INIT:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_CFM_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        lc_send_pdu_num_comp_fail(idx, lc_env_ptr->sp.SpTId);

        lc_sp_fail(dest_id);
    }
    break;
    default:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_CFM_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI pass key request reply request.
HCI_CMD_HANDLER_C(user_passkey_req_reply, struct hci_user_passkey_req_reply_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PASSKEY_HL_RPLY:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_PASSKEY_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        lc_env_ptr->sp.Passkey = param->num_val;
        lc_start_passkey_loop(dest_id);
    }
    break;
    default:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_PASSKEY_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);

        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI pass key request negative reply request.
HCI_CMD_HANDLER_C(user_passkey_req_neg_reply, struct hci_basic_bd_addr_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PASSKEY_HL_RPLY:
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_PASSKEY_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        lc_env_ptr->sp.SPPhase1Failed = 1;
        lc_start_passkey_loop(dest_id);
    }
    break;
    default:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_USER_PASSKEY_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);

        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI remote OOB data request reply request.
HCI_CMD_HANDLER_C(rem_oob_data_req_reply, struct hci_rem_oob_data_req_reply_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Check if Host support secure connections
    if (lm_get_sec_con_host_supp())
    {
        /* Reject the command "If both the Host and Controller support Secure Connections the Host shall respond with
        the Remote OOB Extended Data Request Reply command." */
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_DATA_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);
        return (KE_MSG_CONSUMED);
    }

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_HL_OOB_DATA:
    {
        ASSERT_ERR(lc_env_ptr->sp.sec_con == false);

        memcpy(&lc_env_ptr->sp.RemRandN.A[0], &param->oob_r.R[0], 16);

        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_DATA_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        if (lc_env_ptr->sp.p_data != NULL)
        {
            uint8_t Z = 0;
            const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

            sha_256_f1(false, p_remote_pub_key_x, p_remote_pub_key_x,
                       lc_env_ptr->sp.RemRandN.A, &Z, lc_env_ptr->sp.LocCommitment.A);
        }

        memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &param->oob_c.C[0], 16);
        if (memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.RemCommitment, KEY_LEN))
        {
            lc_env_ptr->sp.SPPhase1Failed = 1;
        }
        memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &lc_env_ptr->sp.RemRandN.A[0], KEY_LEN);
        lc_skip_hl_oob_req(dest_id);
    }
    break;
    default:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_DATA_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);

        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI remote OOB data request negative reply request.
HCI_CMD_HANDLER_C(rem_oob_data_req_neg_reply, struct hci_basic_bd_addr_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_HL_OOB_DATA:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_DATA_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        memset(&lc_env_ptr->sp.RemCommitment.A[0], 0x00, sizeof(struct byte16));
        if (lc_env_ptr->sp.SPInitiator)
        {
            // send LMP_OOBFailed(idx,sptid)
            {
                struct lmp_oob_fail pdu;

                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->sp.SpTId);
                pdu.ext_opcode = LMP_OOB_FAIL_EXTOPCODE;

                lc_send_lmp(idx, &pdu);
            }
            lc_sp_fail(dest_id);
        }
        else
        {
            lc_resp_oob_wait_nonce(dest_id);
        }
        break;
    default:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_DATA_REQ_NEG_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);

        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI send key press notification request.
HCI_CMD_HANDLER_C(send_keypress_notif, struct hci_send_keypress_notif_cmd)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (param->notif_type > SP_PASSKEY_COMPLETED)
    {
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_SEND_KEYPRESS_NOTIF_CMD_OPCODE, CO_ERROR_INVALID_HCI_PARAM, &param->bd_addr);

        return (KE_MSG_CONSUMED);
    }

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PASSKEY_HL_RPLY:
    {
        // send LMP_KeyPressNotification(idx, param->notif_type,sptid)
        struct lmp_keypress_notif pdu;
        pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->sp.SpTId);
        pdu.ext_opcode = LMP_KEYPRESS_NOTIF_EXTOPCODE;
        pdu.type    = param->notif_type;
        lc_send_lmp(idx, &pdu);

        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_SEND_KEYPRESS_NOTIF_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);
    }
    break;
    default:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_SEND_KEYPRESS_NOTIF_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);

        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI remote OOB extended data request reply request.
HCI_CMD_HANDLER_C(rem_oob_ext_data_req_reply, struct hci_rem_oob_ext_data_req_reply_cmd)
{
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_HL_OOB_DATA:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];
        struct hash const *oob_c;
        struct randomizer const *oob_r;
        uint8_t Z = 0x00;
        const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

        // Check if secure connections is active on the link
        if (lc_env_ptr->sp.sec_con)
        {
            oob_c = &param->oob_c_256;
            oob_r = &param->oob_r_256;
        }
        else
        {
            oob_c = &param->oob_c_192;
            oob_r = &param->oob_r_192;
        }

        memcpy(&lc_env_ptr->sp.RemRandN.A[0], &oob_r->R[0], 16);

        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_EXT_DATA_REQ_REPLY_CMD_OPCODE, CO_ERROR_NO_ERROR, &param->bd_addr);

        sha_256_f1(lc_env_ptr->sp.sec_con, p_remote_pub_key_x, p_remote_pub_key_x,
                   &lc_env_ptr->sp.RemRandN.A[0],
                   &Z,
                   &lc_env_ptr->sp.LocCommitment.A[0]);

        memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &oob_c->C[0], 16);
        if (memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.RemCommitment, KEY_LEN))
        {
            lc_env_ptr->sp.SPPhase1Failed = 1;
        }
        memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &lc_env_ptr->sp.RemRandN.A[0], KEY_LEN);
        lc_skip_hl_oob_req(dest_id);
    }
    break;
    default:
        // Send HCI CC event
        lc_cmd_cmp_bd_addr_send(HCI_REM_OOB_EXT_DATA_REQ_REPLY_CMD_OPCODE, CO_ERROR_COMMAND_DISALLOWED, &param->bd_addr);

        break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 * HCI COMMAND HANDLING
 ****************************************************************************************
 */

/// The message handlers for HCI commands
HCI_CMD_HANDLER_TAB(lc)
{
    {HCI_DISCONNECT_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_disconnect_cmd_lc_handler                 },
    {HCI_CREATE_CON_CANCEL_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_create_con_cancel_cmd_lc_handler         },
    {HCI_RD_RSSI_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_rssi_cmd_lc_handler                    },
    {HCI_SETUP_SYNC_CON_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_setup_sync_con_cmd_lc_handler             },
    {HCI_ENH_SETUP_SYNC_CON_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_enh_setup_sync_con_cmd_lc_handler         },
    {HCI_QOS_SETUP_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_qos_setup_cmd_lc_handler                  },
    {HCI_AUTH_REQ_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_auth_req_cmd_lc_handler                   },
    {HCI_SET_CON_ENC_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_set_con_enc_cmd_lc_handler                },
    {HCI_RD_LINK_SUPV_TO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_link_supv_to_cmd_lc_handler            },
    {HCI_WR_LINK_SUPV_TO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_wr_link_supv_to_cmd_lc_handler            },
    {HCI_FLOW_SPEC_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_flow_spec_cmd_lc_handler                  },
    {HCI_FLUSH_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_flush_cmd_lc_handler                      },
    {HCI_ENH_FLUSH_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_enh_flush_cmd_lc_handler                  },
    {HCI_RD_AUTO_FLUSH_TO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_auto_flush_to_cmd_lc_handler           },
    {HCI_WR_AUTO_FLUSH_TO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_wr_auto_flush_to_cmd_lc_handler           },
    {HCI_CHG_CON_PKT_TYPE_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_chg_con_pkt_type_cmd_lc_handler           },
    {HCI_RD_CLK_OFF_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_clk_off_cmd_lc_handler                 },
    {HCI_RD_TX_PWR_LVL_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_tx_pwr_lvl_cmd_lc_handler              },
    {HCI_RD_REM_VER_INFO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_rem_ver_info_cmd_lc_handler            },
    {HCI_RD_REM_SUPP_FEATS_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_rem_supp_feats_cmd_lc_handler          },
    {HCI_RD_LINK_POL_STG_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_link_pol_stg_cmd_lc_handler            },
    {HCI_WR_LINK_POL_STG_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_wr_link_pol_stg_cmd_lc_handler            },
    {HCI_SNIFF_MODE_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_sniff_mode_cmd_lc_handler                 },
    {HCI_EXIT_SNIFF_MODE_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_exit_sniff_mode_cmd_lc_handler            },
    {HCI_SNIFF_SUB_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_sniff_sub_cmd_lc_handler                  },
    {HCI_ROLE_DISCOVERY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_role_discovery_cmd_lc_handler             },
    {HCI_RD_FAIL_CONTACT_CNT_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_fail_contact_cnt_cmd_lc_handler        },
    {HCI_RST_FAIL_CONTACT_CNT_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rst_fail_contact_cnt_cmd_lc_handler       },
    {HCI_RD_LINK_QUAL_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_link_qual_cmd_lc_handler               },
    {HCI_RD_AFH_CH_MAP_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_afh_ch_map_cmd_lc_handler              },
    {HCI_RD_LMP_HDL_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_lmp_hdl_cmd_lc_handler                 },
    {HCI_RD_REM_EXT_FEATS_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_rem_ext_feats_cmd_lc_handler           },
    {HCI_RD_ENC_KEY_SIZE_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_enc_key_size_cmd_lc_handler            },
#if (BT_PWR_CTRL)
    {HCI_RD_ENH_TX_PWR_LVL_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_enh_tx_pwr_lvl_cmd_lc_handler          },
#endif //(BT_PWR_CTRL)
    {HCI_REFRESH_ENC_KEY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_refresh_enc_key_cmd_lc_handler            },
    {HCI_CHG_CON_LK_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_chg_con_lk_cmd_lc_handler                 },
    {HCI_ACCEPT_CON_REQ_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_accept_con_req_cmd_lc_handler             },
    {HCI_REJECT_CON_REQ_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_reject_con_req_cmd_lc_handler             },
    {HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_accept_sync_con_req_cmd_lc_handler        },
    {HCI_ENH_ACCEPT_SYNC_CON_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_enh_accept_sync_con_cmd_lc_handler        },
    {HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_reject_sync_con_req_cmd_lc_handler        },
    {HCI_LK_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_lk_req_reply_cmd_lc_handler               },
    {HCI_LK_REQ_NEG_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_lk_req_neg_reply_cmd_lc_handler           },
    {HCI_PIN_CODE_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_pin_code_req_reply_cmd_lc_handler         },
    {HCI_PIN_CODE_REQ_NEG_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_pin_code_req_neg_reply_cmd_lc_handler     },
    {HCI_SWITCH_ROLE_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_switch_role_cmd_lc_handler                },
    {HCI_RD_CLK_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_clk_cmd_lc_handler                     },
    {HCI_REM_NAME_REQ_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rem_name_req_cmd_lc_handler               },
    {HCI_REM_NAME_REQ_CANCEL_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rem_name_req_cancel_cmd_lc_handler        },
    {HCI_IO_CAP_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_io_cap_req_reply_cmd_lc_handler           },
    {HCI_IO_CAP_REQ_NEG_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_io_cap_req_neg_reply_cmd_lc_handler       },
    {HCI_USER_CFM_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_user_cfm_req_reply_cmd_lc_handler         },
    {HCI_USER_CFM_REQ_NEG_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_user_cfm_req_neg_reply_cmd_lc_handler     },
    {HCI_USER_PASSKEY_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_user_passkey_req_reply_cmd_lc_handler     },
    {HCI_USER_PASSKEY_REQ_NEG_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_user_passkey_req_neg_reply_cmd_lc_handler },
    {HCI_REM_OOB_DATA_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rem_oob_data_req_reply_cmd_lc_handler     },
    {HCI_REM_OOB_DATA_REQ_NEG_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rem_oob_data_req_neg_reply_cmd_lc_handler },
    {HCI_SEND_KEYPRESS_NOTIF_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_send_keypress_notif_cmd_lc_handler        },
    {HCI_REM_OOB_EXT_DATA_REQ_REPLY_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rem_oob_ext_data_req_reply_cmd_lc_handler },
    {HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_rd_auth_payl_to_cmd_lc_handler            },
    {HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_wr_auth_payl_to_cmd_lc_handler            },
    {HCI_WR_SEC_CON_TEST_MODE_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_wr_sec_con_test_mode_cmd_lc_handler       },
#if (EAVESDROPPING_SUPPORT)
    {HCI_VS_SET_CUST_ACL_SCHED_CMD_OPCODE, (lc_hci_cmd_hdl_func_t)hci_vs_set_cust_acl_sched_cmd_lc_handler       },
#endif //EAVESDROPPING_SUPPORT
#if RW_DEBUG
    {HCI_DBG_BT_SEND_LMP_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_dbg_bt_send_lmp_cmd_lc_handler            },
    {HCI_DBG_BT_DISCARD_LMP_EN_CMD_OPCODE, (lc_hci_cmd_hdl_func_t) hci_dbg_bt_discard_lmp_en_cmd_lc_handler     },
#endif //RW_DEBUG
};

/**
 ****************************************************************************************
 * @brief Handles any HCI command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(hci_command_lc, void)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < ARRAY_LEN(lc_hci_command_handler_tab); i++)
    {
        // Check if opcode matches
        if (lc_hci_command_handler_tab[i].opcode == src_id)
        {
            // Check if there is a handler function
            if (lc_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = lc_hci_command_handler_tab[i].func(param, dest_id, src_id);
            }
            break;
        }
    }

    return return_status;
}


/**
 ****************************************************************************************
 * @brief Handles HCI ACL data packet
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(hci_acl_data_lc, struct hci_acl_data)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Nothing to do
    }
    break;
    default:
    {
        if (param->length > 0)
        {
            struct bt_em_acl_buf_elt *tx_elt = bt_util_buf_acl_tx_elt_get((uint16_t) param->buf_ptr);

            // Fill data length and flag fields
            tx_elt->data_len_flags = 0;
            SETF(tx_elt->data_len_flags, BT_EM_ACL_DATA_LEN, param->length);
            SETF(tx_elt->data_len_flags, BT_EM_ACL_BF, GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG));

            // If an L2CAP start fragment is pending, force packet boundary flag to "start"
            if (lc_env_ptr->link.l2cap_start)
            {
                SETF(tx_elt->data_len_flags, BT_EM_ACL_PBF, PBF_1ST_NF_HL_FRAG);
                lc_env_ptr->link.l2cap_start = false;
            }
            else
            {
                SETF(tx_elt->data_len_flags, BT_EM_ACL_PBF, GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG));
            }

            //Check if a flush is pending
            if (lc_env_ptr->link.FlushContinue)
            {
                if (GETF(tx_elt->data_len_flags, BT_EM_ACL_PBF) == PBF_CONT_HL_FRAG)
                {
                    break;
                }
                else
                {
                    lc_env_ptr->link.FlushContinue = false;
                }
            }

            status = ld_acl_data_tx(idx, tx_elt);
        }
        else if (GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG) == PBF_1ST_NF_HL_FRAG)
        {
            // Indicate L2CAP start fragment is pending, and ignore HCI packet
            lc_env_ptr->link.l2cap_start = true;
        }
    }
    break;
    }

    // if an error occurs, drop the buffer
    if (status != CO_ERROR_NO_ERROR)
    {
        if (param->length > 0)
        {
            // Free buffer
            bt_util_buf_acl_tx_free(param->buf_ptr);
        }

        // inform host that buffer is available
        lc_common_nb_of_pkt_comp_evt_send(BT_ACL_CONHDL_MIN + idx, 1);
    }

    return (KE_MSG_CONSUMED);
}

#if VOICE_OVER_HCI
/**
 ****************************************************************************************
 * @brief Handles HCI Synchronous data TX packet
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(hci_sync_data, struct hci_sync_data)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Free the buffer
        bt_util_buf_sync_tx_free(BT_SYNC_CONHDL_LID(GETF(param->conhdl_psf, HCI_SYNC_HDR_HDL)), param->buf_ptr);
    }
    break;
    default:
    {
        uint8_t sync_link_id = BT_SYNC_CONHDL_LID(GETF(param->conhdl_psf, HCI_SYNC_HDR_HDL));
        struct bt_em_sync_buf_elt *buf_elt = bt_util_buf_sync_tx_elt_get(sync_link_id, param->buf_ptr);
        status = ld_sco_data_tx(sync_link_id, buf_elt);

        if (status != CO_ERROR_NO_ERROR)
        {
            // Free the buffer
            bt_util_buf_sync_tx_free(sync_link_id, param->buf_ptr);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}
#endif // VOICE_OVER_HCI



/*
 * LMP PACKETS HANDLERS
 ****************************************************************************************
 */

/// Handles the LMP packet LMP_name_req
LMP_MSG_HANDLER(name_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        struct lmp_name_res pdu;
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        pdu.opcode = LMP_OPCODE(LMP_NAME_RES_OPCODE, lc_env_ptr->link.RxTrIdServer);
        pdu.offset    = param->offset;

        // Get the local name segment from LM
        LM_GetLocalNameSeg(&pdu.name_frag, pdu.offset, &pdu.length);

        lc_send_lmp(idx, &pdu);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_name_res
LMP_MSG_HANDLER(name_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_REM_NAME:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // Store segment
        LC_StoreRemoteNameSeg(idx, (struct name_vect *)&param->name_frag, param->offset, param->length);

        if (!((param->offset + NAME_VECT_SIZE) >= param->length))
        {
            // send LMP_Name Req
            struct lmp_name_req pdu;

            // fill the parameters
            pdu.opcode = LMP_OPCODE(LMP_NAME_REQ_OPCODE,  GETB(param->opcode, LMP_TR_ID));
            pdu.offset  = (param->offset + NAME_VECT_SIZE);

            // send the pdu
            lc_send_lmp(idx, &pdu);

            // start the timer
            lc_start_lmp_to(dest_id);
        }
        else
        {
            lc_rem_name_cont(dest_id);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_accepted
LMP_MSG_HANDLER(accepted)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr != NULL)
    {
        // reset the values: LinkId, Opcode, OpcodeExt, Mode
        lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
        if ((param->orig_opcode == LMP_SCO_LINK_REQ_OPCODE))
        {
            lc_sco_peer_accept(dest_id);
        }
        else
        {
            ASSERT_INFO_FORCE(0, 0, param->opcode);
        }
        break;

    case LC_SCO_DISC_ONGOING:
        if ((param->orig_opcode == LMP_RMV_SCO_LINK_REQ_OPCODE))
        {
            lc_sco_peer_accept_disc(dest_id);
        }
        else
        {
            ASSERT_INFO_FORCE(0, 0, param->opcode);
        }
        break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_DHKEY_CHECK_RSP_PEER_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_DHKEY_CHK_OPCODE) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            lc_start_lmp_to(dest_id);
            lc_calc_link_key(dest_id);
        }
        break;

    case LC_WAIT_DHKEY_CHECK_INIT_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_DHKEY_CHK_OPCODE) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_DHKEY_CHECK_INIT_PEER);
        }
        break;

    case LC_WAIT_OOB_RANDN_RSP_PEER_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_SP_NB_OPCODE) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            // reuse local commitment
            memcpy(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.p_data->local_oob_rand, sizeof(struct byte16));
            /*
             * SP AUTH STG1 : END
             */
            /*
             * SP AUTH STG2 : START
             */
            lc_try_start_auth_stage2(dest_id);
        }
        break;

    case LC_WAIT_OOB_RANDN_INIT_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_SP_NB_OPCODE) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            lc_start_lmp_to(dest_id);

            ke_state_set(dest_id, LC_WAIT_OOB_RANDN_INIT_PEER);
        }
        break;

    case LC_WAIT_PASSKEY_RANDN_RSP_PEER_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_SP_NB_OPCODE) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            lc_env_ptr->sp.EncapPduCtr += 1;
            if (lc_env_ptr->sp.EncapPduCtr < SSP_PASSKEY_NB_BITS)
            {
                ke_state_set(dest_id, LC_WAIT_PASSKEY_COMM_RSP_PEER);
            }
            else
            {
                lm_sp_32bits_to_array(&lc_env_ptr->sp.RemCommitment, lc_env_ptr->sp.Passkey);
                lm_sp_32bits_to_array(&lc_env_ptr->sp.LocCommitment, lc_env_ptr->sp.Passkey);
                /*
                 * SP AUTH STG1 : END
                 */
                /*
                 * SP AUTH STG2 : START
                 */
                lc_try_start_auth_stage2(dest_id);
            }
        }
        break;

    case LC_WAIT_PASSKEY_RANDN_INIT_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_SP_NB_OPCODE) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_PASSKEY_RANDN_INIT_PEER);
        }
        break;

    case LC_WAIT_NUM_COMP_RANDN_RSP_PEER_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_SP_NB_OPCODE) && (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
            const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

            /*
             * SP AUTH STG1 : VERIFICATION CHECK
             */
            sha_256_g(lc_env_ptr->sp.sec_con, p_remote_pub_key_x, p_local_pub_key_x,
                      &lc_env_ptr->sp.RemRandN.A[0],
                      &lc_env_ptr->sp.LocRandN.A[0],
                      &lc_env_ptr->sp.Passkey);

            {
                struct hci_user_cfm_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_USER_CFM_REQ_EVT_CODE, hci_user_cfm_req_evt);
                evt->passkey = lc_env_ptr->sp.Passkey;
                memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                hci_send_2_host(evt);

                lc_start_lmp_to(dest_id);

                ke_state_set(dest_id, LC_WAIT_NUM_COMP_USER_CONF_RSP);
            }
        }
        break;

    case LC_WAIT_NUM_COMP_COMM_INIT_RANDN_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_SP_NB_OPCODE) && (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            //Restart the LMP TO to wait the next message from the peer
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_NUM_COMP_COMM_INIT_RANDN_PEER);
        }
        break;

    case LC_PUB_KEY_PAYLOAD_RSP_PEER_CFM:
        if (param->orig_opcode == LMP_ENCAPS_PAYL_OPCODE)
        {
            uint8_t nb_payl = (lc_env_ptr->sp.sec_con) ? LMP_ENCAPS_P256_PAYL_NB : LMP_ENCAPS_P192_PAYL_NB;

            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            lc_env_ptr->sp.EncapPduCtr += 1;
            if (lc_env_ptr->sp.EncapPduCtr >= nb_payl)
            {
                /*
                 * STOP THE PUBLICK KEY EXCHANGE
                 */

                /*
                 * SP AUTH STG1 : START
                 */

                //Set default key
                lc_env_ptr->enc.KeyType = (lc_env_ptr->sp.sec_con) ? BT_AUTH_COMB_KEY_256 : BT_AUTH_COMB_KEY_192;

                // Check the common authentication method between local and peer
                switch (lm_get_auth_method(&lc_env_ptr->sp.IOCap_loc, &lc_env_ptr->sp.IOCap_rem))
                {
                case SP_JUST_WORKS:
                    // just works method
                    lc_env_ptr->enc.KeyType = (lc_env_ptr->sp.sec_con) ? BT_UNAUTH_COMB_KEY_256 : BT_UNAUTH_COMB_KEY_192;
                case SP_NUM_COMP:
                    // numeric comparison method
                    ke_state_set(dest_id, LC_WAIT_AUTH_STG1_NC_RANDN);
                    ke_msg_send_basic(LC_AUTH_STG1_RANDN, dest_id, dest_id);
                    break;
                case SP_PASSKEY:
                    // pass key method
                    ke_state_set(dest_id, LC_WAIT_AUTH_STG1_PK_RANDN);
                    ke_msg_send_basic(LC_AUTH_STG1_RANDN, dest_id, dest_id);
                    break;
                case SP_OOB:
                    // OOB method
                    lc_start_oob(dest_id);
                    break;
                default:
                    break;
                }
            }
            else
            {
                lc_send_encap_pdu_pub_key(idx, lc_env_ptr, lc_env_ptr->link.RxTrIdServer, dest_id);
            }
        }
        break;

    case LC_PUB_KEY_HEADER_RSP_CFM:
        if (param->orig_opcode == LMP_ENCAPS_HDR_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            lc_env_ptr->sp.EncapPduCtr = 0;
            lc_send_encap_pdu_pub_key(idx, lc_env_ptr, lc_env_ptr->link.RxTrIdServer, dest_id);
            ke_state_set(dest_id, LC_PUB_KEY_PAYLOAD_RSP_PEER_CFM);
        }
        break;

    case LC_PUB_KEY_PAYLOAD_INIT_LOC:
        if (param->orig_opcode == LMP_ENCAPS_PAYL_OPCODE)
        {
            uint8_t nb_payl = (lc_env_ptr->sp.sec_con) ? LMP_ENCAPS_P256_PAYL_NB : LMP_ENCAPS_P192_PAYL_NB;

            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            lc_env_ptr->sp.EncapPduCtr += 1;

            // this is encapsulation pdu counter: 3 payloads
            if (lc_env_ptr->sp.EncapPduCtr >= nb_payl)
            {
                // start timer
                lc_start_lmp_to(dest_id);

                // wait for public key header from the peer
                ke_state_set(dest_id, LC_WAIT_PUB_KEY_HEADER_INIT_PEER);
            }
            else
            {
                lc_send_encap_pdu_pub_key(idx, lc_env_ptr, lc_env_ptr->link.RxTrIdServer, dest_id);
                //Keep the same state
            }
        }
        break;

    case LC_PUB_KEY_HEADER_INIT_LOC:
        if (param->orig_opcode == LMP_ENCAPS_HDR_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            lc_env_ptr->sp.EncapPduCtr = 0;

            //  send LMP_EncapsulatedPayload
            lc_send_encap_pdu_pub_key(idx, lc_env_ptr, lc_env_ptr->link.Role, dest_id);

            // set state to wait for public key payload confirmation
            ke_state_set(dest_id, LC_PUB_KEY_PAYLOAD_INIT_LOC);
        }
        break;

#if BCAST_ENC_SUPPORT
    case LC_WAIT_MST_SEMI_ACC:
        if (param->orig_opcode == LMP_USE_SEMI_PERM_KEY_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_env_ptr->enc.KeyFlag = lc_env_ptr->enc.NewKeyFlag;
            lc_semi_key_cmp(dest_id, CO_ERROR_NO_ERROR);
        }
        break;
#endif // BCAST_ENC_SUPPORT

    case LC_WAIT_SWITCH_CFM:
        if (param->orig_opcode == LMP_SWITCH_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            uint8_t status = CO_ERROR_NO_ERROR;
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            status = ld_acl_rsw_req(idx, lc_env_ptr->link.LtAddr, lc_env_ptr->link.SlotOffset, lc_env_ptr->link.SwitchInstant, lm_page_scan_rep_mode_get());
            if (status == CO_ERROR_NO_ERROR)
            {
                // if SAM enabled, disable prior to switch instant
                lc_sam_disable(idx);

                ke_state_set(dest_id, LC_WAIT_SWITCH_CMP);
            }
            else
            {

                lc_switch_cmp(dest_id, status);
            }
        }
        break;

    case LC_WAIT_QOS_CFM:
        if (param->orig_opcode == LMP_QOS_REQ_OPCODE)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            lc_mst_qos_done(dest_id);
        }
        break;
    case LC_WAIT_FLOW_SPEC_CFM:
        // Receive response to LMP_QOS_req sent for flow specification procedure
        if (param->orig_opcode == LMP_QOS_REQ_OPCODE)
        {
            uint16_t conhdl = BT_ACL_CONHDL_MIN + idx;
            struct hci_flow_spec_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_FLOW_SPEC_CMP_EVT_CODE, hci_flow_spec_cmp_evt);

            ld_acl_t_poll_set(idx, lc_env_ptr->link.PollInterval);

            evt->status    = CO_ERROR_NO_ERROR;
            evt->conhdl    = conhdl;
            evt->flags     = lc_env_ptr->link.Flags;
            evt->flow_dir  = lc_env_ptr->link.FlowDirection;
            evt->pk_bw     = lc_env_ptr->link.PeakBandwidth;
            evt->serv_type = lc_env_ptr->link.ServiceType;
            evt->tk_buf_sz = lc_env_ptr->link.TokenBucketSize;
            evt->tk_rate   = lc_env_ptr->link.TokenRate ;
            evt->acc_lat   = lc_env_ptr->link.AccessLatency;

            hci_send_2_host(evt);

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
    case LC_WAIT_UNSNIFF_ACC:
        if (param->orig_opcode == LMP_UNSNIFF_REQ_OPCODE)
        {
            lc_sniff_unsniff_peer_accept(dest_id);
        }

        break;
    case LC_WAIT_SNIFF_REQ:
        if (param->orig_opcode == LMP_SNIFF_REQ_OPCODE)
        {
            lc_sniff_peer_accept(dest_id);
        }
        break;
    case LC_WAIT_MAX_SLOT_CFM:
        if (param->orig_opcode == LMP_MAX_SLOT_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_chg_pkt_type_cont(dest_id, CO_ERROR_NO_ERROR);
        }
        break;
    case LC_WAIT_ENC_START_CFM:
        if (param->orig_opcode == LMP_START_ENC_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            if ((lc_env_ptr->link.Initiator ^ (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->link.Role)))
            {
#if BCAST_ENC_SUPPORT
                struct lb_enc_start_ind *ind = KE_MSG_ALLOC(LB_ENC_START_IND, TASK_LB, dest_id, lb_enc_start_ind);
                ind->key_flag = lc_env_ptr->enc.KeyFlag;
                ke_msg_send(ind);
#endif // BCAST_ENC_SUPPORT

                // Start encryption in TX
                ld_acl_tx_enc(idx, LD_ENC_E0 + lc_env_ptr->sp.sec_con);

                lc_env_ptr->enc.EncEnable = ENCRYPTION_ON;
                lc_env_ptr->enc.EncMode   = lc_env_ptr->enc.NewEncMode;

                if (lc_env_ptr->epr.on)
                {
                    lc_epr_cmp(dest_id);
                }
                else
                {
                    lc_enc_cmp(dest_id, CO_ERROR_NO_ERROR);
                }

                if (lc_env_ptr->sp.sec_con)
                {
                    // Start authenticated payload timers
                    ke_timer_set(LC_AUTH_PAYL_NEARLY_TO, dest_id, 10 * (lc_env_ptr->link.auth_payl_to - lc_env_ptr->link.auth_payl_to_margin));
                    ke_timer_set(LC_AUTH_PAYL_REAL_TO, dest_id, 10 * lc_env_ptr->link.auth_payl_to);
                }
            }
            else
            {
                // send LMP_NotAccepted(LMP_START_ENC_REQ_OPCODE)
                lc_send_pdu_not_acc(idx, LMP_START_ENC_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);
            }
        }
        break;
    case LC_WAIT_ENC_STOP_CFM_MST:
        if (param->orig_opcode == LMP_STOP_ENC_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            if ((lc_env_ptr->link.Initiator ^ (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->link.Role)))
            {
#if BCAST_ENC_SUPPORT
                ke_msg_send_basic(LB_ENC_STOP_IND, TASK_LB, dest_id);
#endif // BCAST_ENC_SUPPORT

                ld_acl_tx_enc(idx, LD_ENC_DISABLE);

                lc_env_ptr->enc.EncEnable = ENCRYPTION_OFF;
                lc_env_ptr->enc.EncMode = lc_env_ptr->enc.NewEncMode;

                if (lc_env_ptr->sp.sec_con)
                {
                    // Stop authenticated payload timers
                    ke_timer_clear(LC_AUTH_PAYL_NEARLY_TO, dest_id);
                    ke_timer_clear(LC_AUTH_PAYL_REAL_TO, dest_id);
                }

                if (lc_env_ptr->epr.on)
                {
                    //Check if a role switch is requested
                    if (lc_env_ptr->epr.rsw)
                    {
                        lc_local_switch(dest_id);
                    }
                    else
                    {
                        if (lc_env_ptr->link.Initiator)
                        {
                            lc_start_enc(dest_id);
                        }
                        else
                        {
                            // wait the request from the peer
                            lc_start_lmp_to(dest_id);
                            ke_state_set(dest_id, LC_WAIT_EPR_PEER_REQ_MST);
                        }
                    }
                }
                else
                {
                    lc_enc_cmp(dest_id, CO_ERROR_NO_ERROR);
                }
            }
            else
            {
                // send LMP_NotAcceptedExt(LMP_STOP_ENC_REQ_OPCODE)
                lc_send_pdu_not_acc(idx, LMP_STOP_ENC_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);
            }
        }
        break;
    case LC_WAIT_ENC_SIZE_CFM:
        if (param->orig_opcode == LMP_ENC_KEY_SIZE_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_start_enc(dest_id);
        }
        break;

    case LC_WAIT_ENC_SLV_SIZE:
        if (param->orig_opcode == LMP_ENC_KEY_SIZE_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_WAIT_ENC_START_REQ_SLV);
        }
        break;

    case LC_WAIT_ENC_MODE_CFM:
        if (param->orig_opcode == LMP_ENC_MODE_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);

            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                lc_start_lmp_to(dest_id);

                if (lc_env_ptr->enc.NewEncMode == ENC_DISABLED)
                {
                    ke_state_set(dest_id, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
                }
                else
                {
                    ke_state_set(dest_id, LC_WAIT_ENC_SLV_SIZE);
                }
            }
            else
            {
                if (lc_env_ptr->enc.NewEncMode != ENC_DISABLED)
                {
                    lc_start_enc_key_size(dest_id);
                }
                else
                {
                    lc_stop_enc(dest_id);
                }
            }
        }
        break;

    case LC_WAIT_REM_HL_CON:
        if (param->orig_opcode == LMP_HOST_CON_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_env_ptr->link.ConnectedState = true;
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
    case LC_WAIT_PAIR_CFM_INIT:
    case LC_WAIT_PAIR_CFM_RSP:
        if (param->orig_opcode == LMP_INRAND_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            if (lc_env_ptr->link.Initiator)
            {
                lc_start_key_exch(dest_id);
            }
            else
            {
                lc_start_lmp_to(dest_id);
                ke_state_set(dest_id, LC_WAIT_KEY_EXCH_RSP);
            }
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
#if RW_DEBUG
    case LC_WAIT_DBG_SEND_LMP_CFM:
        ke_state_set(dest_id, LC_CONNECTED);
        lc_cmd_cmp_conhdl_send(HCI_DBG_BT_SEND_LMP_CMD_OPCODE, CO_ERROR_NO_ERROR, BT_ACL_CONHDL_MIN + idx);
        break;
#endif // RW_DEBUG
    case LC_CONNECTED:
        if ((lc_env_ptr->link.Role == SLAVE_ROLE) && (param->orig_opcode == LMP_UNSNIFF_REQ_OPCODE))
        {
            // Local unsniff request rejected due to transaction collision, do nothing
            break; // no default handling
        }
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), param->orig_opcode);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_not_accepted
LMP_MSG_HANDLER(not_accepted)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    if (lc_env_ptr != NULL)
    {
        lc_env_ptr->link.Reason = param->reason;

        // reset the values: LinkId, Opcode, OpcodeExt, Mode
        lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
        if ((param->orig_opcode == LMP_SCO_LINK_REQ_OPCODE))
        {
            lc_sco_peer_reject(dest_id, param->reason);
        }
        else
        {
            ASSERT_WARN(0, 0, param->opcode);
        }
        break;

    case LC_SCO_DISC_ONGOING:
        if ((param->orig_opcode == LMP_RMV_SCO_LINK_REQ_OPCODE))
        {
            lc_sco_peer_reject_disc(dest_id, param->reason);
        }
        else
        {
            ASSERT_WARN(0, 0, param->opcode);
        }
        break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_DHKEY_CHECK_RSP_PEER_CFM:
    case LC_WAIT_DHKEY_CHECK_INIT_CFM:
    case LC_WAIT_OOB_RANDN_RSP_PEER_CFM:
    case LC_WAIT_OOB_RANDN_INIT_CFM:
    case LC_WAIT_PASSKEY_RANDN_RSP_PEER_CFM:
    case LC_WAIT_PASSKEY_RANDN_INIT_CFM:
    case LC_WAIT_PASSKEY_COMM_INIT_PEER:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (((param->orig_opcode == LMP_SP_NB_OPCODE)
                || (param->orig_opcode == LMP_SP_CFM_OPCODE)
                || (param->orig_opcode == LMP_DHKEY_CHK_OPCODE)) &&
                (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_sp_fail(dest_id);
        }
        break;
    case LC_WAIT_NUM_COMP_COMM_INIT_RANDN_CFM:
    case LC_PUB_KEY_PAYLOAD_INIT_LOC:
    case LC_PUB_KEY_PAYLOAD_RSP_PEER_CFM:
    case LC_PUB_KEY_HEADER_INIT_LOC:
    case LC_PUB_KEY_HEADER_RSP_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_opcode == LMP_ENCAPS_HDR_OPCODE)
                || (param->orig_opcode == LMP_ENCAPS_PAYL_OPCODE)
                || (param->orig_opcode == LMP_SP_NB_OPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_sp_fail(dest_id);
        }
        break;
    case LC_WAIT_NUM_COMP_RANDN_RSP_PEER_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        lc_sp_fail(dest_id);
        break;

#if BCAST_ENC_SUPPORT
    case LC_WAIT_MST_SEMI_ACC:
        if (param->orig_opcode == LMP_USE_SEMI_PERM_KEY_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_semi_key_cmp(dest_id, param->reason);
        }
        break;
#endif // BCAST_ENC_SUPPORT

    case LC_WAIT_SWITCH_CFM:
        if (param->orig_opcode == LMP_SWITCH_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            if (lc_env_ptr->epr.rsw)
            {
                lc_epr_rsw_cmp(dest_id, param->reason);
            }
            else
            {
                lc_switch_cmp(dest_id, param->reason);
            }
        }
        break;
    case LC_WAIT_QOS_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (param->orig_opcode == LMP_QOS_REQ_OPCODE)
        {
            uint8_t status = CO_ERROR_NO_ERROR;
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            do
            {
                if ((param->reason == CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE)
                        || (param->reason == CO_ERROR_INVALID_LMP_PARAM))
                {
                    status = LM_GetQoSParam(lc_env_ptr->link.Role, &lc_env_ptr->link.ServiceType,
                                            &lc_env_ptr->link.TokenRate, &lc_env_ptr->link.PeakBandwidth, &lc_env_ptr->link.Latency,
                                            &lc_env_ptr->link.DelayVariation, &lc_env_ptr->link.PollInterval, lc_env_ptr->link.CurPacketType,
                                            QOS_PEER_REJECT);
                    if (status == CO_ERROR_NO_ERROR)
                    {
                        lc_qos_setup(dest_id);
                        break;
                    }
                }
                // allocate the status event message
                uint16_t conhdl = BT_ACL_CONHDL_MIN + idx;
                struct hci_qos_setup_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_QOS_SETUP_CMP_EVT_CODE, hci_qos_setup_cmp_evt);

                // update the status
                event->status    = param->reason;
                event->conhdl    = conhdl;
                event->flags     = 0;
                event->tok_rate  = lc_env_ptr->link.TokenRate;
                event->serv_type = lc_env_ptr->link.ServiceType;
                event->pk_bw     = lc_env_ptr->link.PeakBandwidth;
                event->lat       = lc_env_ptr->link.Latency;
                event->del_var   = lc_env_ptr->link.DelayVariation;
                // send the message
                hci_send_2_host(event);
                ke_state_set(dest_id, LC_CONNECTED);
            }
            while (0);
        }
        break;
    case LC_WAIT_FLOW_SPEC_CFM:
        // Receive response to LMP_QOS_req sent for flow specification procedure
        if (param->orig_opcode == LMP_QOS_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

            if ((param->reason == CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE) ||
                    (param->reason == CO_ERROR_INVALID_LMP_PARAM))
            {
                status = LM_GetQoSParam(lc_env_ptr->link.Role,
                                        &lc_env_ptr->link.ServiceType, &lc_env_ptr->link.TokenRate,
                                        &lc_env_ptr->link.PeakBandwidth, &lc_env_ptr->link.Latency,
                                        &lc_env_ptr->link.DelayVariation, &lc_env_ptr->link.PollInterval,
                                        lc_env_ptr->link.CurPacketType, QOS_PEER_REJECT);

                if (status == CO_ERROR_NO_ERROR)
                {
                    // send LMP_QualityOfServiceReq(idx , PollInterval, nb_bcst , role)
                    lc_send_pdu_qos_req(idx, lb_util_get_nb_broadcast() + 1, lc_env_ptr->link.PollInterval, lc_env_ptr->link.Role);
                    lc_start_lmp_to(dest_id);
                    break;
                }
            }

            uint16_t conhdl = BT_ACL_CONHDL_MIN + idx;
            struct hci_flow_spec_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_FLOW_SPEC_CMP_EVT_CODE, hci_flow_spec_cmp_evt);
            // fill the parameters
            evt->status    = param->reason;
            evt->conhdl    = conhdl;
            evt->flags     = 0;
            evt->flow_dir  = lc_env_ptr->link.FlowDirection;
            evt->pk_bw     = lc_env_ptr->link.PeakBandwidth;
            evt->serv_type = lc_env_ptr->link.ServiceType;
            evt->tk_buf_sz = lc_env_ptr->link.TokenBucketSize;
            evt->tk_rate   = lc_env_ptr->link.TokenRate ;
            evt->acc_lat   = lc_env_ptr->link.AccessLatency;
            // send the event
            hci_send_2_host(evt);

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;

    case LC_WAIT_SNIFF_REQ:
        if (param->orig_opcode == LMP_SNIFF_REQ_OPCODE)
        {
            lc_sniff_peer_reject(dest_id, param->reason);
        }
        break;
    case LC_WAIT_MAX_SLOT_CFM:
        if (param->orig_opcode == LMP_MAX_SLOT_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            uint8_t max_slot;
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            max_slot = LM_MaxSlot(lc_env_ptr->link.CurPacketType);
            if (max_slot == 0x05)
            {
                LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x05);
            }
            else if (max_slot == 0x03)
            {
                LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x03);
            }
            lc_chg_pkt_type_retry(dest_id);
        }
        break;
    case LC_WAIT_ENC_START_CFM:
        if (param->orig_opcode == LMP_START_ENC_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ld_acl_rx_enc(idx, LD_ENC_DISABLE);

            if (lc_env_ptr->req.LocEncReq)
            {
                lc_send_enc_chg_evt(idx, CO_ERROR_LMP_RSP_TIMEOUT);
            }
            lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        }
        break;
    case LC_WAIT_ENC_STOP_CFM_MST:
        if (param->orig_opcode == LMP_STOP_ENC_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            ld_acl_rx_enc(idx, LD_ENC_DISABLE);

            lc_env_ptr->enc.EncEnable = ENCRYPTION_OFF;

            if (lc_env_ptr->sp.sec_con)
            {
                // Stop authenticated payload timers
                ke_timer_clear(LC_AUTH_PAYL_NEARLY_TO, dest_id);
                ke_timer_clear(LC_AUTH_PAYL_REAL_TO, dest_id);
            }

            if (lc_env_ptr->req.LocEncReq)
            {
                lc_send_enc_chg_evt(idx, param->reason);
            }
            lc_detach(dest_id, param->reason);
        }
        break;
    case LC_WAIT_ENC_SLV_SIZE:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (param->orig_opcode == LMP_ENC_KEY_SIZE_REQ_OPCODE)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_enc_cmp(dest_id, param->reason);
        }
        break;
    case LC_WAIT_ENC_SIZE_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (param->orig_opcode == LMP_ENC_KEY_SIZE_REQ_OPCODE)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_enc_cmp(dest_id, param->reason);
        }
        break;
    case LC_WAIT_ENC_MODE_CFM:
        if (param->orig_opcode == LMP_ENC_MODE_REQ_OPCODE)
        {
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            if (param->reason != CO_ERROR_LMP_COLLISION)
            {
                if (lc_env_ptr->req.LocEncReq)
                {
                    lc_send_enc_chg_evt(idx, param->reason);
                }
            }
            lc_env_ptr->req.LocEncReq = false;
            lc_env_ptr->link.Initiator = true;
            if (lc_env_ptr->req.RestartEncReq)
            {
                ld_acl_flow_on(idx);
                lc_restart_enc_cont(dest_id, param->reason);
            }
            else
            {
                lc_detach(dest_id, param->reason);
            }
        }
        break;
    case LC_WAIT_REM_HL_CON:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        if (param->orig_opcode == LMP_HOST_CON_REQ_OPCODE)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_detach(dest_id, param->reason);
        }
        break;
    case LC_WAIT_AUTH_SRES:
        if (param->orig_opcode == LMP_AURAND_OPCODE)
        {
            uint8_t reason;
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
            switch (param->reason)
            {
            case CO_ERROR_LMP_COLLISION:
                //Retry cause
                if (!lc_env_ptr->link.Role)
                {
                    LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);
                    lc_send_pdu_au_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.Role);
                }
                break;
            default:
                reason = (param->reason == CO_ERROR_NO_ERROR) ?
                         CO_ERROR_AUTH_FAILURE : param->reason;
                lc_auth_cmp(dest_id, reason);
                break;
            }
        }
        break;

    case LC_WAIT_KEY_EXCH_RSP:
        if (param->orig_opcode == LMP_INRAND_OPCODE)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            if (param->reason == CO_ERROR_LMP_COLLISION)
            {
                lc_env_ptr->req.LocAuthReq = false;
                lc_env_ptr->link.Initiator = false;
                lc_start_lmp_to(dest_id);
                break;
            }
        }
    case LC_WAIT_PAIR_CFM_INIT:
    case LC_WAIT_PAIR_CFM_RSP:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (param->orig_opcode == LMP_INRAND_OPCODE)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            if (param->reason == CO_ERROR_LMP_COLLISION)
            {
                lc_env_ptr->req.LocAuthReq = false;
                lc_start_lmp_to(dest_id);
            }
            else
            {
                lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
                lc_auth_cmp(dest_id, param->reason);
            }
        }
    case LC_WAIT_SRES_INIT:
    case LC_WAIT_AU_RAND_SEC_AUTH_INIT:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        lc_mutual_auth_end(dest_id, param->reason);
        break;
    case LC_WAIT_UNSNIFF_ACC:
    {
        if (param->orig_opcode == LMP_UNSNIFF_REQ_OPCODE)
        {
            lc_sniff_unsniff_peer_reject(dest_id, param->reason);
        }
    }
    break;

    case LC_WAIT_TIM_ACC_RSP:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        ke_state_set(dest_id, LC_CONNECTED);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
#if RW_DEBUG
    case LC_WAIT_DBG_SEND_LMP_CFM:
        ke_state_set(dest_id, LC_CONNECTED);
        lc_cmd_cmp_conhdl_send(HCI_DBG_BT_SEND_LMP_CMD_OPCODE, param->reason, BT_ACL_CONHDL_MIN + idx);
        break;
#endif // RW_DEBUG
    case LC_CONNECTED:
        if ((lc_env_ptr->link.Role == SLAVE_ROLE) && (param->orig_opcode == LMP_UNSNIFF_REQ_OPCODE))
        {
            // Local unsniff request rejected due to transaction collision, do nothing
            //ASSERT_WARN((param->reason == CO_ERROR_LMP_COLLISION), ke_state_get(dest_id), param->reason);
            break; // no default handling
        }
    // no break
    default:
        ASSERT_WARN((param->reason == CO_ERROR_LMP_COLLISION), ke_state_get(dest_id), param->orig_opcode);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_clk_off_req
LMP_MSG_HANDLER(clk_off_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t clk_off;
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        // save the parameters in the environment
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        clk_off = (ld_acl_clock_offset_get(idx).hs >> 2) & 0x7FFF;
        clk_off = 32768 - clk_off;

        {
            // send LMP_ClkOffsetRes(idx,Role)
            struct lmp_clk_off_res pdu;
            pdu.clk_offset = clk_off;
            pdu.opcode = LMP_OPCODE(LMP_CLK_OFF_RES_OPCODE, lc_env_ptr->link.RxTrIdServer);
            lc_send_lmp(idx, &pdu);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_clk_off_res
LMP_MSG_HANDLER(clk_off_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // save the parameters in the environment
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_CLK_OFF:
    {
        // Report procedure completion
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_rd_clk_off_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_CLK_OFF_CMP_EVT_CODE, hci_rd_clk_off_cmp_evt);
        event->status      = CO_ERROR_NO_ERROR;
        event->conhdl      = conhdl;
        event->clk_off_val = param->clk_offset;
        hci_send_2_host(event);

        // timer clear
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        ke_state_set(dest_id, LC_CONNECTED);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_detach
LMP_MSG_HANDLER(detach)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    lc_env_ptr->link.Reason = param->reason;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_CFM:
    case LC_FREE:
        break;
    default:
    {
        uint16_t timer_duration_slot = ld_acl_t_poll_get(idx);

        lc_env_ptr->req.PeerDetachReq = true;
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // send authentication complete event if local authentication request
        if (lc_env_ptr->req.LocAuthReq)
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_auth_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_CMP_EVT_CODE, hci_auth_cmp_evt);
            evt->status = param->reason;
            evt->conhdl = conhdl;
            hci_send_2_host(evt);
            lc_env_ptr->req.LocAuthReq = false;
            lc_env_ptr->link.Initiator  = false;
        }

        ld_acl_flow_off(idx);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            timer_duration_slot *= 6;
        }
        else
        {
            timer_duration_slot *= 3;
        }
        ke_timer_set(LC_LMP_RSP_TO, dest_id, 10 * co_slot_to_duration(timer_duration_slot));
        ke_state_set(dest_id, LC_WAIT_DETACH_TO);

    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_inrand
LMP_MSG_HANDLER(inrand)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    memcpy(&lc_env_ptr->enc.RandomRx.ltk[0], &param->random.ltk[0], KEY_LEN);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_env_ptr->link.Initiator = false;
        lc_pair(dest_id, idx);
        break;
    case LC_WAIT_PAIR_CFM_INIT:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->link.Role)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            if (LM_GetPINType() == FIXED_PIN)
            {
                // send LMP_NotAccepted(LMP_INRAND_OPCODE)
                lc_send_pdu_not_acc(idx, LMP_INRAND_OPCODE, CO_ERROR_PAIRING_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);

                lc_auth_cmp(dest_id, CO_ERROR_PAIRING_NOT_ALLOWED);
            }
            else// VARIABLE_PIN
            {
                // LMP_Accepted(LMP_INRAND_OPCODE)
                lc_send_pdu_acc(idx, LMP_INRAND_OPCODE, lc_env_ptr->link.RxTrIdServer);

                //Generate link key
                E22(lc_env_ptr->enc.RandomRx, lc_env_ptr->info.LocalBdAddr, lc_env_ptr->enc.PinCode,
                    lc_env_ptr->enc.PinLength, &lc_env_ptr->enc.LTKey);

                // start key exchange
                lc_start_key_exch(dest_id);
            }
        }
        else
        {
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // send LMP_NotAccepted(LMP_INRAND_OPCODE)
                lc_send_pdu_not_acc(idx, LMP_INRAND_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
            }
            else
            {
                lc_env_ptr->link.Initiator = false;
                lc_pairing_cont(dest_id);
            }
        }
        break;
    case LC_WAIT_PAIR_CFM_RSP:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        // send LMP_NotAccepted(LMP_INRAND_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_INRAND_OPCODE, CO_ERROR_PAIRING_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);

        lc_auth_cmp(dest_id, CO_ERROR_PAIRING_NOT_ALLOWED);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_combkey
LMP_MSG_HANDLER(combkey)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct ltk temp_key1, temp_key2;

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_KEY_EXCH:
        memcpy(&temp_key2.ltk[0], &param->random.ltk[0], KEY_LEN);
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->link.Role)
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            //Compute LK_RANDa
            E21(lc_env_ptr->enc.RandomTx, lc_env_ptr->info.LocalBdAddr, &temp_key1);
            //Extract LK_RANDb
            XorKey(temp_key2, lc_env_ptr->enc.LTKey, &temp_key2);
            //Compute LK_Kb
            E21(temp_key2, lc_env_ptr->info.BdAddr, &temp_key2);
            //Create combination key
            XorKey(temp_key1, temp_key2, &lc_env_ptr->enc.LTKey);

            lc_key_exch_end(dest_id, CO_ERROR_NO_ERROR);
        }
        else
        {
            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                lc_key_exch_end(dest_id, CO_ERROR_LMP_COLLISION);
            }
            else
            {
                lc_send_pdu_not_acc(idx, LMP_COMBKEY_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
            }
        }
        break;

    case LC_WAIT_KEY_EXCH_RSP:
        lc_comb_key_svr(dest_id, (struct ltk *) &param->random);
        break;

    case LC_CONNECTED:
        if (!lc_env_ptr->epr.on)
        {
            if (lm_get_sp_en())
            {
                lc_env_ptr->enc.KeyType = BT_CHANGED_COMB_KEY;
            }
            lc_env_ptr->link.Initiator = false;
            lc_env_ptr->enc.key_from_host = false;

            lc_comb_key_svr(dest_id, (struct ltk *) &param->random);
        }
        else
        {
            // send LMP_NotAccepted(LMP_COMBKEY_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_COMBKEY_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_unitkey
LMP_MSG_HANDLER(unitkey)
{
    int idx = KE_IDX_GET(dest_id);

    // This LMP is deprecated since BT5.1 (see Vol 2 Part C Chapter 4.2.2.4)
    lc_send_pdu_not_acc(idx, LMP_UNITKEY_OPCODE, CO_ERROR_PAIRING_WITH_UNIT_KEY_NOT_SUP,  GETB(param->opcode, LMP_TR_ID));

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_aurand
LMP_MSG_HANDLER(aurand)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct aco temp_aco;

    // Save the values from the au_rand message
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    memcpy(&lc_env_ptr->enc.RandomRx.ltk[0], &param->random.ltk[0], KEY_LEN);

    // Save LMP_au_rand to avoid SRES computation conflict
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_env_ptr->req.PeerAuthReq = true;
        lc_resp_auth(dest_id);
        break;
    case LC_WAIT_AU_RAND_SEC_AUTH_INIT:
    {
        // Compute both signed responses with ACO
        lc_sec_auth_compute_sres(dest_id);

        // If slave, transmit the signed response
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            // Send signed response
            lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
        }

        // Wait signed response from the peer
        lc_start_lmp_to(dest_id);
        ke_state_set(dest_id, LC_WAIT_SRES_SEC_AUTH_INIT);
    }
    break;
    case LC_WAIT_AUTH_SRES:
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            // Master ignore the request from  slave
            // send LMP_NotAccepted(LMP_AURAND_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_AURAND_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
        }
        else
        {
            // E1 procedure
            E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.LocalBdAddr,
               lc_env_ptr->enc.RandomRx, &lc_env_ptr->enc.Sres, &lc_env_ptr->enc.Aco);

            //  send LMP_Sres
            lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
            lc_loc_auth(dest_id, idx);
        }
        break;
    case LC_WAIT_AU_RAND_PEER_INIT:
    {
        // E1 procedure
        E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.LocalBdAddr, lc_env_ptr->enc.RandomRx, &lc_env_ptr->enc.Sres, &temp_aco);

#if BCAST_ENC_SUPPORT
        if (!lc_env_ptr->req.MasterKeyReq)
#endif // BCAST_ENC_SUPPORT
        {
            memcpy(&lc_env_ptr->enc.Aco.a[0], &temp_aco.a[0], 12);
        }
        //  send LMP_Sres
        lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);

        lc_mutual_auth_end(dest_id, lc_env_ptr->link.Reason);
    }
    break;
    case LC_WAIT_AU_RAND_RSP:
    {
        if (!lc_env_ptr->sp.sec_con)
        {
            // E1 procedure
            E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.LocalBdAddr, lc_env_ptr->enc.RandomRx, &lc_env_ptr->enc.Sres, &temp_aco);

#if BCAST_ENC_SUPPORT
            if (!lc_env_ptr->req.MasterKeyReq)
#endif // BCAST_ENC_SUPPORT
            {
                memcpy(&lc_env_ptr->enc.Aco.a[0], &temp_aco.a[0], 12);
            }
            //  send LMP_Sres
            lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
        }

        // Make random vector
        LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);
        // send LMP_AuRand
        lc_send_pdu_au_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.RxTrIdServer);

        // In secure authentication, compute both signed responses
        if (lc_env_ptr->sp.sec_con)
        {
            // Compute both signed responses with ACO
            lc_sec_auth_compute_sres(dest_id);

            // If slave in secure authentication, send the local signed response
            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
            }
        }

        // start timer for LMP_sres
        lc_start_lmp_to(dest_id);
        ke_state_set(dest_id, LC_WAIT_SRES_RSP);
    }
    break;
    case LC_WAIT_PAIR_CFM_INIT:
    case LC_WAIT_PAIR_CFM_RSP:
        //  send Not accepted
        lc_send_pdu_not_acc(idx, LMP_AURAND_OPCODE, CO_ERROR_PIN_MISSING, lc_env_ptr->link.RxTrIdServer);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        // If authentication not already initiated by the peer, queue message for later processing
        if (!lc_env_ptr->req.PeerAuthReq)
        {
            return (KE_MSG_SAVED);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sres
LMP_MSG_HANDLER(sres)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct aco temp_aco;
    uint8_t reason = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SRES_SEC_AUTH_INIT:
    {
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // Check received sres
        if (!memcmp(&lc_env_ptr->enc.Sres_expected.nb[0], &param->Sres.nb[0], SRES_LEN))
        {
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // Send signed response
                lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
            }

            if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
            {
                lc_env_ptr->epr.on = true;
                lc_env_ptr->epr.cclk = true;
            }

            reason = CO_ERROR_NO_ERROR;
        }
        else
        {
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // Send signed response anyway, for LMP procedure completeness
                lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
            }

            lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
            lc_env_ptr->enc.PinStatus = PIN_ABSENT;
            reason = CO_ERROR_AUTH_FAILURE;
        }

        lc_mutual_auth_end(dest_id, reason);
    }
    break;
    case LC_WAIT_AUTH_SRES:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        // E1 procedure
        E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.BdAddr, lc_env_ptr->enc.RandomTx, &lc_env_ptr->enc.Sres, &temp_aco);

        //Check received sres
        if (!memcmp(&lc_env_ptr->enc.Sres.nb[0], &param->Sres.nb[0], 4))
        {
            memcpy(&lc_env_ptr->enc.Aco.a[0], &temp_aco.a[0], 12);
            reason = CO_ERROR_NO_ERROR;
        }
        else
        {
            lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
            lc_env_ptr->enc.PinStatus = PIN_ABSENT;
            reason = CO_ERROR_AUTH_FAILURE;
        }
        lc_auth_cmp(dest_id, reason);
        break;
    case LC_WAIT_SRES_RSP:
    {
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        if (!lc_env_ptr->sp.sec_con)
        {
            // E1 procedure
            E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.BdAddr, lc_env_ptr->enc.RandomTx, &lc_env_ptr->enc.Sres_expected, &temp_aco);
        }

        // Check received sres
        if (!memcmp(&lc_env_ptr->enc.Sres_expected.nb[0], &param->Sres.nb[0], 4))
        {
            // If master in secure authentication, send the local signed response
            if (lc_env_ptr->sp.sec_con && (lc_env_ptr->link.Role == MASTER_ROLE))
            {
                lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
            }

            reason = CO_ERROR_NO_ERROR;

#if BCAST_ENC_SUPPORT
            if (!lc_env_ptr->req.MasterKeyReq)
#endif // BCAST_ENC_SUPPORT
            {
                if (!lc_env_ptr->sp.sec_con)
                {
                    memcpy(&lc_env_ptr->enc.Aco.a[0], &temp_aco.a[0], 12);
                }

                if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
                {
                    lc_env_ptr->epr.on = true;
                    lc_env_ptr->epr.cclk = true;
                }
            }
        }
        else
        {
            lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
            lc_env_ptr->enc.PinStatus = PIN_ABSENT;

            reason = CO_ERROR_AUTH_FAILURE;
        }

        // mutual authentication
        lc_mutual_auth_end(dest_id, reason);
    }
    break;
    case LC_WAIT_SRES_INIT:
    {
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // E1 procedure
        E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.BdAddr, lc_env_ptr->enc.RandomTx, &lc_env_ptr->enc.Sres, &temp_aco);

        //Check received sres
        if (!memcmp(&lc_env_ptr->enc.Sres.nb[0], &param->Sres.nb[0], 4))
        {
#if BCAST_ENC_SUPPORT
            if (!lc_env_ptr->req.MasterKeyReq)
#endif // BCAST_ENC_SUPPORT
            {
                memcpy(&lc_env_ptr->enc.Aco.a[0], &temp_aco.a[0], 12);
                lc_env_ptr->link.Reason = CO_ERROR_NO_ERROR;
            }
        }
        else
        {
            lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
            lc_env_ptr->enc.PinStatus = PIN_ABSENT;

            lc_env_ptr->link.Reason = CO_ERROR_AUTH_FAILURE;
        }
        // start timer
        lc_start_lmp_to(dest_id);
        // wait for au_rand
        ke_state_set(dest_id, LC_WAIT_AU_RAND_PEER_INIT);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_temprand
LMP_MSG_HANDLER(temprand)
{
#if BCAST_ENC_SUPPORT
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            struct ltk temp_key;
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

            memcpy(&temp_key.ltk[0], &param->random.ltk[0], KEY_LEN);

            lc_env_ptr->req.MasterKeyReq = true;

            lc_env_ptr->enc.NewKeyFlag = TEMPORARY_KEY;

            memcpy(&lc_env_ptr->enc.SemiPermanentKey.ltk[0], &lc_env_ptr->enc.LTKey.ltk[0], KEY_LEN);

            memcpy(&lc_env_ptr->enc.PinCode.pin[0], &lc_env_ptr->enc.SemiPermanentKey.ltk[0], KEY_LEN);

            // E22 procedure - Generate link key
            E22(temp_key, lc_env_ptr->info.BdAddr, lc_env_ptr->enc.PinCode, KEY_LEN,
                &lc_env_ptr->enc.Overlay);

            lc_mst_key(dest_id);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
#endif // BCAST_ENC_SUPPORT

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_tempkey
LMP_MSG_HANDLER(tempkey)
{
#if BCAST_ENC_SUPPORT
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            struct ltk temp_key;
            lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

            memcpy(&temp_key.ltk[0], &param->key.ltk[0], KEY_LEN);
            //Generate link key
            XorKey(lc_env_ptr->enc.Overlay, temp_key, &lc_env_ptr->enc.LTKey);
        }
        break;
    }
#endif // BCAST_ENC_SUPPORT

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_enc_mode_req
LMP_MSG_HANDLER(enc_mode_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t reason = CO_ERROR_NO_ERROR;

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_ENC_MODE_CFM:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            // send LMP_NotAccepted(LMP_ENC_MODE_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_MODE_REQ_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdClient);
        }
        else
        {
            lc_env_ptr->link.Initiator = false;
            lc_env_ptr->enc.NewEncMode = param->enc_mode;
            // LMP_Accepted(LMP_ENC_MODE_REQ_OPCODE)
            lc_send_pdu_acc(idx, LMP_ENC_MODE_REQ_OPCODE, lc_env_ptr->link.RxTrIdClient);
            // start lmp timer
            lc_start_lmp_to(dest_id);
            if (lc_env_ptr->enc.NewEncMode == ENC_DISABLED)
            {
                ke_state_set(dest_id, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
            }
            else
            {
                ke_state_set(dest_id, LC_WAIT_ENC_SLV_SIZE);
            }
        }
        break;
    case LC_WAIT_ENC_SLV_RESTART:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->enc_mode == ENC_PP_ENABLED) || (param->enc_mode == ENC_PP_BC_ENABLED))
        {
            lc_env_ptr->enc.NewEncMode = ENC_PP_ENABLED;
            lc_rem_enc(dest_id);
        }
        else
        {
            // send LMP_NotAccepted(LMP_ENC_MODE_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_MODE_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

            lc_enc_cmp(dest_id, CO_ERROR_INVALID_LMP_PARAM);
        }
        break;
    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT:
    case LC_WAIT_RESTART_ENC:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        if (param->enc_mode == ENC_DISABLED)
        {
            lc_env_ptr->enc.NewEncMode = ENC_DISABLED;
            lc_rem_enc(dest_id);
        }
        else
        {
            // send LMP_NotAccepted(LMP_ENC_MODE_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_MODE_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

            lc_enc_cmp(dest_id, CO_ERROR_INVALID_LMP_PARAM);
        }
        break;
    case LC_CONNECTED:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        do
        {
            if (lc_env_ptr->sp.sec_con && (param->enc_mode == ENC_DISABLED))
            {
                reason = CO_ERROR_ENC_MODE_NOT_ACCEPT;
                break;
            }

            if (param->enc_mode > ENC_PP_BC_ENABLED)
            {
                reason = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
                break;
            }

            if (((param->enc_mode == ENC_DISABLED) && (lc_env_ptr->enc.EncEnable == ENCRYPTION_OFF))
                    || (!(param->enc_mode == ENC_DISABLED) && (lc_env_ptr->enc.EncEnable == ENCRYPTION_ON)))
            {
                reason = CO_ERROR_LMP_PDU_NOT_ALLOWED;
                break;
            }
            else
            {
                lc_env_ptr->req.PeerEncReq = true;
                lc_env_ptr->enc.NewEncMode = param->enc_mode;
                lc_rem_enc(dest_id);
            }
        }
        while (0);

        if (reason != CO_ERROR_NO_ERROR)
        {
            // send LMP_NotAccepted(LMP_ENC_MODE_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_MODE_REQ_OPCODE, reason, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        if (lc_env_ptr->req.LocEncReq)
        {
            lc_end_chk_colli(dest_id, param->enc_mode);
        }
        else
        {
            //ASSERT_WARN(0, ke_state_get(dest_id), 0);
            return (KE_MSG_SAVED);
        }
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_enc_key_size_req
LMP_MSG_HANDLER(enc_key_size_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_ENC_SIZE_CFM:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        if (
#if BCAST_ENC_SUPPORT
            (lc_env_ptr->enc.KeyFlag == SEMI_PERMANENT_KEY) &&
#endif // BCAST_ENC_SUPPORT
#if (BT_53)
#if (RW_DEBUG)
            (param->key_size <= lc_env_ptr->enc.max_enc_key_size) &&
#else //(!RW_DEBUG)
            (param->key_size <= ENC_KEY_SIZE_MAX) &&
#endif //(!RW_DEBUG)
            (param->key_size >= lc_env_ptr->enc.min_enc_key_size))
#else // !(BT_53)
            (param->key_size <= ENC_KEY_SIZE_MAX) &&
            (param->key_size >= ENC_KEY_SIZE_MIN))
#endif // !(BT_53)
        {
            lc_env_ptr->enc.EncSize = param->key_size;
            // LMP_Accepted (LMP_enc_key_size_req)
            lc_send_pdu_acc(idx, LMP_ENC_KEY_SIZE_REQ_OPCODE, lc_env_ptr->link.RxTrIdClient);

            lc_start_enc(dest_id);
        }
        else
        {
            // send LMP_NotAccepted(LMP_ENC_KEY_SIZE_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_KEY_SIZE_REQ_OPCODE, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE, lc_env_ptr->link.RxTrIdClient);

            lc_enc_cmp(dest_id, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
        }
        break;
    case LC_WAIT_ENC_SLV_SIZE:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
#if (BT_53)
        if (param->key_size >= lc_env_ptr->enc.min_enc_key_size)
#else // !(BT_53)
        if (param->key_size >= ENC_KEY_SIZE_MIN)
#endif // !(BT_53)
        {
#if (BT_53) && (RW_DEBUG)
            if (param->key_size <= lc_env_ptr->enc.max_enc_key_size)
#else // (BT_53) && (RW_DEBUG)
            if (param->key_size <= ENC_KEY_SIZE_MAX)
#endif // (BT_53) && (RW_DEBUG)
            {
                lc_env_ptr->enc.EncSize = param->key_size;
                // LMP_Accepted(LMP_ENC_KEY_SIZE_OPCODE)
                lc_send_pdu_acc(idx, LMP_ENC_KEY_SIZE_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);
                // set state
                ke_state_set(dest_id, LC_WAIT_ENC_START_REQ_SLV);
            }
            else
            {
#if (BT_53) && (RW_DEBUG)
                lc_env_ptr->enc.EncSize = lc_env_ptr->enc.max_enc_key_size;
#else // (BT_53) && (RW_DEBUG)
                lc_env_ptr->enc.EncSize = ENC_KEY_SIZE_MAX;
#endif // (BT_53) && (RW_DEBUG)
                // send LMP_EncryptionKeySizeReq
                lc_send_pdu_enc_key_sz_req(idx, lc_env_ptr->enc.EncSize, lc_env_ptr->link.RxTrIdServer);
            }

            lc_start_lmp_to(dest_id);
        }
        else
        {
            // send LMP_NotAccepted(LMP_ENC_KEY_SIZE_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_KEY_SIZE_REQ_OPCODE, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE, lc_env_ptr->link.RxTrIdServer);

            lc_enc_cmp(dest_id, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_start_enc_req
LMP_MSG_HANDLER(start_enc_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    // save values from message
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    memcpy(&lc_env_ptr->enc.RandomRx.ltk[0], &param->random.ltk[0], KEY_LEN);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_EPR_PEER_REQ_SLV:
    case LC_WAIT_ENC_START_REQ_SLV:
        if ((lc_env_ptr->link.Initiator ^ (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->link.Role)))
        {
            if (lc_env_ptr->epr.on)
            {
                ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            }

            if (!lc_env_ptr->sp.sec_con)
            {
                struct ltk temp_ltk;
#if BCAST_ENC_SUPPORT
                if (lc_env_ptr->enc.KeyFlag == SEMI_PERMANENT_KEY)
                {
                    E3(lc_env_ptr->enc.LTKey, lc_env_ptr->enc.Aco, lc_env_ptr->enc.RandomRx, &temp_ltk);
                    ld_acl_bcst_rx_dec(idx, ENCRYPTION_OFF);
                }
                else
                {
                    struct aco aco_tmp;
                    LM_MakeCof(lc_env_ptr->info.BdAddr, &aco_tmp);
                    E3(lc_env_ptr->enc.LTKey, aco_tmp, lc_env_ptr->enc.RandomRx, &temp_ltk);
                    ld_acl_bcst_rx_dec(idx, ENCRYPTION_ON);
                }
#else // !BCAST_ENC_SUPPORT
                E3(lc_env_ptr->enc.LTKey, lc_env_ptr->enc.Aco, lc_env_ptr->enc.RandomRx, &temp_ltk);
#endif // !BCAST_ENC_SUPPORT

                KPrimC(temp_ltk, lc_env_ptr->enc.EncSize, &lc_env_ptr->enc.EncKey);

#if (EAVESDROPPING_SUPPORT)
                // Save the IV
                memcpy(&lc_env_ptr->enc.EncIV, &lc_env_ptr->enc.Aco, IV_LEN);
#endif // EAVESDROPPING_SUPPORT

                ld_acl_enc_key_load(idx, &lc_env_ptr->enc.EncKey, NULL);
                ld_acl_rx_enc(idx, LD_ENC_E0);
                ld_acl_tx_enc(idx, LD_ENC_E0);
            }
            else
            {
                struct initialization_vector *iv;
                uint8_t *master_bd = (lc_env_ptr->link.Role) ? &lc_env_ptr->info.BdAddr.addr[0]      : &lc_env_ptr->info.LocalBdAddr.addr[0];
                uint8_t *slave_bd  = (lc_env_ptr->link.Role) ? &lc_env_ptr->info.LocalBdAddr.addr[0] : &lc_env_ptr->info.BdAddr.addr[0];

                // Compute encryption key
                sha_256_h3(&lc_env_ptr->enc.LTKey.ltk[0], master_bd, slave_bd, &lc_env_ptr->enc.Aco.a[0], &lc_env_ptr->enc.EncKey.ltk[0]);

                // Revert encryption key and reduce its size if necessary
                {
                    ASSERT_ERR(lc_env_ptr->enc.EncSize <= KEY_LEN);

                    uint8_t enc_key[KEY_LEN];
                    uint8_t zero_byte_nb = KEY_LEN - lc_env_ptr->enc.EncSize;

                    for (int i = zero_byte_nb ; i < KEY_LEN ; i++)
                    {
                        enc_key[(KEY_LEN - 1) - i] = lc_env_ptr->enc.EncKey.ltk[i];
                    }

                    // Perform encryption key size reduction if necessary (LSBs of the key are replaced with 0x00)
                    if (zero_byte_nb > 0)
                    {
                        memset(&enc_key[lc_env_ptr->enc.EncSize], 0, zero_byte_nb);
                    }

                    memcpy(&lc_env_ptr->enc.EncKey.ltk[0], enc_key, KEY_LEN);
                }

                if (lc_env_ptr->epr.on)
                {
                    // If resume encryption, the initialization vector is taken from LMP_pause_enc_aes_req
                    iv = (struct initialization_vector *) &lc_env_ptr->sp.LocRandN;
                }
                else
                {
                    // If start encryption, the initialization vector is taken from ACO (authentication)
                    iv = (struct initialization_vector *) &lc_env_ptr->enc.Aco;
                }

#if (EAVESDROPPING_SUPPORT)
                // Save the IV
                memcpy(&lc_env_ptr->enc.EncIV, iv, IV_LEN);
#endif // EAVESDROPPING_SUPPORT

                // Load encryption key
                ld_acl_enc_key_load(idx, &lc_env_ptr->enc.EncKey, iv);

                // Start encryption in both RX and TX
                ld_acl_rx_enc(idx, LD_ENC_AES);
                ld_acl_tx_enc(idx, LD_ENC_AES);

                // Start authenticated payload timers
                ke_timer_set(LC_AUTH_PAYL_NEARLY_TO, dest_id, 10 * (lc_env_ptr->link.auth_payl_to - lc_env_ptr->link.auth_payl_to_margin));
                ke_timer_set(LC_AUTH_PAYL_REAL_TO, dest_id, 10 * lc_env_ptr->link.auth_payl_to);
            }

            // LMP_Accepted(LMP_START_ENC_REQ_OPCODE)
            lc_send_pdu_acc(idx, LMP_START_ENC_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);

            lc_env_ptr->enc.EncEnable = ENCRYPTION_ON;
            lc_env_ptr->enc.EncMode = lc_env_ptr->enc.NewEncMode;

            if (lc_env_ptr->epr.on)
            {
                lc_epr_cmp(dest_id);
            }
            else
            {
                lc_enc_cmp(dest_id, CO_ERROR_NO_ERROR);
            }
        }
        else
        {
            // send LMP_NotAccepted(LMP_START_ENC_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_START_ENC_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        // send LMP_NotAccepted(LMP_START_ENC_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_START_ENC_REQ_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_stop_enc_req
LMP_MSG_HANDLER(stop_enc_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT:
    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        if ((lc_env_ptr->link.Initiator ^ (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->link.Role)))
        {
            if (lc_env_ptr->enc.EncEnable != ENCRYPTION_OFF)
            {
                ld_acl_rx_enc(idx, LD_ENC_DISABLE);
                ld_acl_tx_enc(idx, LD_ENC_DISABLE);
#if BCAST_ENC_SUPPORT
                ld_acl_bcst_rx_dec(idx, LD_ENC_DISABLE);
#endif // BCAST_ENC_SUPPORT

                // LMP_Accepted(LMP_STOP_ENC_REQ_OPCODE)
                lc_send_pdu_acc(idx, LMP_STOP_ENC_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);

                lc_env_ptr->enc.EncEnable = ENCRYPTION_OFF;
                lc_env_ptr->enc.EncMode = lc_env_ptr->enc.NewEncMode;

                if (lc_env_ptr->sp.sec_con)
                {
                    // Stop authenticated payload timers
                    ke_timer_clear(LC_AUTH_PAYL_NEARLY_TO, dest_id);
                    ke_timer_clear(LC_AUTH_PAYL_REAL_TO, dest_id);
                }

                if (lc_env_ptr->epr.on)
                {
                    if (lc_env_ptr->epr.rsw)
                    {
                        lc_local_switch(dest_id);
                    }
                    else
                    {
                        if (lc_env_ptr->link.Initiator)
                        {
                            lc_send_pdu_resu_enc_req(idx, lc_env_ptr->link.Role);
                            //wait the start encryption from the master
                        }
                        ke_state_set(dest_id, LC_WAIT_EPR_PEER_REQ_SLV);
                    }
                }
                else
                {
                    lc_enc_cmp(dest_id, CO_ERROR_NO_ERROR);
                }
            }
            else
            {
                // send LMP_NotAccepted(LMP_START_ENC_REQ_OPCODE)
                lc_send_pdu_not_acc(idx, LMP_START_ENC_REQ_OPCODE, CO_ERROR_INSUFFICIENT_SECURITY, lc_env_ptr->link.RxTrIdServer);
            }
        }
        else
        {
            // send LMP_NotAccepted(LMP_START_ENC_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_START_ENC_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        // send LMP_NotAccepted(LMP_START_ENC_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_START_ENC_REQ_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_switch_req
LMP_MSG_HANDLER(switch_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t reason = CO_ERROR_NO_ERROR;
    bool epr_ongoing = false;

    lc_env_ptr->link.SwitchInstant = param->switch_inst;
    lc_env_ptr->link.RxTrIdServer  = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_REM_HL_CON:
    {
        do
        {
            if (lc_env_ptr->link.AllowRoleSwitch == ROLE_SWITCH_ALLOWED)
            {
                if (lm_role_switch_start(idx, &lc_env_ptr->link.LtAddr))
                {
                    lc_env_ptr->link.Initiator = false;
                    lc_env_ptr->req.PeerSwitchReq = true;
                    lc_rem_switch(dest_id);
                    break;
                }
                else
                {
                    reason = CO_ERROR_CON_LIMIT_EXCEED;
                }
            }
            else
            {
                reason = CO_ERROR_ROLE_CHANGE_NOT_ALLOWED;
            }

            ///send LMP_NotAccepted
            lc_send_pdu_not_acc(idx, LMP_SWITCH_REQ_OPCODE, reason, GETB(param->opcode, LMP_TR_ID));
        }
        while (0);
    }
    break;
    case LC_WAIT_EPR_PEER_REQ_MST:
    case LC_WAIT_EPR_PEER_REQ_SLV:
        epr_ongoing = true;
    //Normal role switch management
    case LC_CONNECTED:
        do
        {
#if (MAX_NB_SYNC > 0)
            bool activ_sco = lm_look_for_sync(idx, ACL_TYPE);
#endif // (MAX_NB_SYNC > 0)
            if ((lc_env_ptr->link.LinkPolicySettings & POLICY_SWITCH)
                    && (lc_env_ptr->link.CurrentMode != LM_SNIFF_MODE)
#if (MAX_NB_SYNC > 0)
                    && (!activ_sco)
#endif // (MAX_NB_SYNC > 0)
#if BCAST_ENC_SUPPORT
                    && (lc_env_ptr->enc.KeyFlag == SEMI_PERMANENT_KEY)
#endif // BCAST_ENC_SUPPORT
               )
            {
                if (lm_role_switch_start(idx, &lc_env_ptr->link.LtAddr))
                {
                    lc_env_ptr->link.Initiator = false;
                    lc_env_ptr->req.PeerSwitchReq = true;
                    if (epr_ongoing)
                    {
                        lc_env_ptr->epr.rsw = true;
                    }
                    lc_rem_switch(dest_id);
                    break;
                }
                else
                {
                    reason = CO_ERROR_CON_LIMIT_EXCEED;
                }
            }
            else
            {
                reason = CO_ERROR_LMP_PDU_NOT_ALLOWED;
            }

            // send LMP_NotAccepted(LMP_SWITCH_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_SWITCH_REQ_OPCODE, reason, lc_env_ptr->link.RxTrIdServer);

        }
        while (0);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    case LC_WAIT_SRES_RSP:
    case LC_WAIT_AU_RAND_SEC_AUTH_INIT:
    case LC_WAIT_SRES_SEC_AUTH_INIT:
        reason = CO_ERROR_ROLE_CHANGE_NOT_ALLOWED;
        // send LMP_NotAccepted(LMP_SWITCH_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SWITCH_REQ_OPCODE, reason, lc_env_ptr->link.RxTrIdServer);
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sniff_req
LMP_MSG_HANDLER(sniff_req)
{
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SNIFF_REQ:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            // reset the values: LinkId, Opcode, OpcodeExt, Mode
            lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);

            lc_env_ptr->link.RxTrIdServer = lc_env_ptr->link.Role;
        }

        lc_sniff_peer_request(dest_id, param->flags, param->d_sniff, param->t_sniff, param->sniff_attempt, param->sniff_to);
    }
    break;

    case LC_CONNECTED:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

#if (EAVESDROPPING_SUPPORT)
        // Sniff request
        lc_send_sniff_request_evt(lc_env_ptr, param->t_sniff, param->d_sniff, param->sniff_attempt, param->sniff_to);
#endif // EAVESDROPPING_SUPPORT

        if ((lc_env_ptr->link.CurrentMode == LM_ACTIVE_MODE) && (lc_env_ptr->link.LinkPolicySettings & POLICY_SNIFF))
        {
            lc_sniff_peer_request(dest_id, param->flags, param->d_sniff, param->t_sniff, param->sniff_attempt, param->sniff_to);
        }
        else
        {
            // send LMP_NotAccepted(LMP_SNIFF_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_SNIFF_REQ_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, (GETB(param->opcode, LMP_TR_ID)));
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_unsniff_req
LMP_MSG_HANDLER(unsniff_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    case LC_WAIT_UNSNIFF_ACC:
    case LC_WAIT_SSR_INSTANT:
    {
        lc_sniff_unsniff_peer_request(dest_id);
    }
    break;

    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_incr_pwr_req
LMP_MSG_HANDLER(incr_pwr_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        bool max_pwr = rwip_rf.txpwr_inc(idx);
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (max_pwr)
        {
            //  send LMP_MaxPower
            {
                struct lmp_max_pwr pdu;

                pdu.opcode = LMP_OPCODE(LMP_MAX_PWR_OPCODE, lc_env_ptr->link.RxTrIdServer);
                // send the message
                lc_send_lmp(idx, &pdu);
            }
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_decr_pwr_req
LMP_MSG_HANDLER(decr_pwr_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        bool min_pwr = rwip_rf.txpwr_dec(idx);
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if (min_pwr)
        {
            //  send LMP_MinPower
            {
                struct lmp_min_pwr pdu;

                // fill the parameters
                pdu.opcode = LMP_OPCODE(LMP_MIN_PWR_OPCODE, lc_env_ptr->link.RxTrIdServer);
                // send the pdu
                lc_send_lmp(idx, &pdu);
            }
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_max_pwr
LMP_MSG_HANDLER(max_pwr)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        lc_env_ptr->link.MaxPowerRcv = true;

#if (EAVESDROPPING_SUPPORT)
        lc_ed_min_max_pwr_ind(idx, ED_MAX_POWER);
#endif // EAVESDROPPING_SUPPORT
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_min_pwr
LMP_MSG_HANDLER(min_pwr)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        lc_env_ptr->link.MinPowerRcv = true;

#if (EAVESDROPPING_SUPPORT)
        lc_ed_min_max_pwr_ind(idx, ED_MIN_POWER);
#endif // EAVESDROPPING_SUPPORT
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_pwr_ctrl_req
LMP_MSG_HANDLER(pwr_ctrl_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        uint8_t response;

        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        if (lc_env_ptr->link.epc_supported)
        {
            switch (param->pwr_adj)
            {
            case PWR_ADJ_REQ_DEC_1_STEP:
            {
                // No per-modulation power control supported, so simply decrease the TX power
                bool minmax = rwip_rf.txpwr_dec(idx);

                // Check if min level is reached
                if (minmax)
                {
                    response = (PWR_ADJ_RES_MIN << PWR_ADJ_RES_GFSK_POS      //1MB
                                | PWR_ADJ_RES_MIN << PWR_ADJ_RES_DQPSK_POS   //2MB
                                | PWR_ADJ_RES_MIN << PWR_ADJ_RES_8DPSK_POS); //3MB
                }
                else
                {
                    response = (PWR_ADJ_RES_CHG_1_STEP << PWR_ADJ_RES_GFSK_POS      //1MB
                                | PWR_ADJ_RES_CHG_1_STEP << PWR_ADJ_RES_DQPSK_POS   //2MB
                                | PWR_ADJ_RES_CHG_1_STEP << PWR_ADJ_RES_8DPSK_POS); //3MB
                }
            }
            break;

            case PWR_ADJ_REQ_INC_1_STEP:
            {
                // No per-modulation power control supported, so simply increase the TX power
                bool minmax = rwip_rf.txpwr_inc(idx);

                // Check if maximum level is reached
                if (minmax)
                {
                    response = (PWR_ADJ_RES_MAX << PWR_ADJ_RES_GFSK_POS      //1MB
                                | PWR_ADJ_RES_MAX << PWR_ADJ_RES_DQPSK_POS   //2MB
                                | PWR_ADJ_RES_MAX << PWR_ADJ_RES_8DPSK_POS); //3MB
                }
                else
                {
                    response = (PWR_ADJ_RES_CHG_1_STEP << PWR_ADJ_RES_GFSK_POS      //1MB
                                | PWR_ADJ_RES_CHG_1_STEP << PWR_ADJ_RES_DQPSK_POS   //2MB
                                | PWR_ADJ_RES_CHG_1_STEP << PWR_ADJ_RES_8DPSK_POS); //3MB
                }
            }
            break;

            case PWR_ADJ_REQ_INC_MAX:
            {
                // No per-modulation power control supported, so simply set TX max power
                rwip_rf.txpwr_max_set(idx);
                response = (PWR_ADJ_RES_MAX << PWR_ADJ_RES_GFSK_POS      //1MB
                            | PWR_ADJ_RES_MAX << PWR_ADJ_RES_DQPSK_POS   //2MB
                            | PWR_ADJ_RES_MAX << PWR_ADJ_RES_8DPSK_POS); //3MB
            }
            break;

            default:
            {
                response = (PWR_ADJ_RES_NOT_SUPP << PWR_ADJ_RES_GFSK_POS      //1MB
                            | PWR_ADJ_RES_NOT_SUPP << PWR_ADJ_RES_DQPSK_POS   //2MB
                            | PWR_ADJ_RES_NOT_SUPP << PWR_ADJ_RES_8DPSK_POS); //3MB
            }
            break;
            }

            // send LMP_PowerCntlRes(idx,PwAdjParamA,rx_tr_id)
            {
                struct lmp_pwr_ctrl_res pdu;
                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.RxTrIdServer);
                pdu.ext_opcode = LMP_PWR_CTRL_RES_EXTOPCODE;
                pdu.pwr_adj = response;
                lc_send_lmp(idx, &pdu);
            }
        }
        else
        {
            // send LMP_NotAcceptedExt(LMP_PW_CNTL_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_PWR_CTRL_REQ_EXTOPCODE, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE, lc_env_ptr->link.RxTrIdServer);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_pwr_ctrl_res
LMP_MSG_HANDLER(pwr_ctrl_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PWR_CTRL_RES:
    {
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        if ((((param->pwr_adj & PWR_ADJ_RES_GFSK_MASK)  >> PWR_ADJ_RES_GFSK_POS) == PWR_ADJ_RES_MAX)    //BR max power
                && ((((param->pwr_adj & PWR_ADJ_RES_DQPSK_MASK) >> PWR_ADJ_RES_DQPSK_POS) == PWR_ADJ_RES_MAX) || //2MB max power
                    (((param->pwr_adj & PWR_ADJ_RES_DQPSK_MASK) >> PWR_ADJ_RES_DQPSK_POS) == PWR_ADJ_RES_NOT_SUPP)) //2MB not supported
                && ((((param->pwr_adj & PWR_ADJ_RES_8DPSK_MASK) >> PWR_ADJ_RES_8DPSK_POS) == PWR_ADJ_RES_MAX) || //3MB max power
                    (((param->pwr_adj & PWR_ADJ_RES_8DPSK_MASK) >> PWR_ADJ_RES_8DPSK_POS) == PWR_ADJ_RES_NOT_SUPP))) //3MB not supported
        {
            lc_env_ptr->link.MaxPowerRcv = true;

#if (EAVESDROPPING_SUPPORT)
            lc_ed_min_max_pwr_ind(idx, ED_MAX_POWER);
#endif // EAVESDROPPING_SUPPORT
        }
        else if ((((param->pwr_adj & PWR_ADJ_RES_GFSK_MASK)  >> PWR_ADJ_RES_GFSK_POS) == PWR_ADJ_RES_MIN)    //BR min power
                 && ((((param->pwr_adj & PWR_ADJ_RES_DQPSK_MASK) >> PWR_ADJ_RES_DQPSK_POS) == PWR_ADJ_RES_MIN) || //2MB min power
                     (((param->pwr_adj & PWR_ADJ_RES_DQPSK_MASK) >> PWR_ADJ_RES_DQPSK_POS) == PWR_ADJ_RES_NOT_SUPP)) //2MB not supported
                 && ((((param->pwr_adj & PWR_ADJ_RES_8DPSK_MASK) >> PWR_ADJ_RES_8DPSK_POS) == PWR_ADJ_RES_MIN) || //3MB min power
                     (((param->pwr_adj & PWR_ADJ_RES_8DPSK_MASK) >> PWR_ADJ_RES_8DPSK_POS) == PWR_ADJ_RES_NOT_SUPP))) //3MB not supported
        {
            lc_env_ptr->link.MinPowerRcv = true;

#if (EAVESDROPPING_SUPPORT)
            lc_ed_min_max_pwr_ind(idx, ED_MIN_POWER);
#endif // EAVESDROPPING_SUPPORT
        }

        ke_state_set(dest_id, LC_CONNECTED);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        // Assert if there is no on-going local disconnection process
        // Otherwise, we might be in the process of disconnecting SCO or unsniffing
        // In any case, we do not care about the response since we are disconnecting
        ASSERT_WARN(lc_env_ptr->req.LocDetachReq, ke_state_get(dest_id), param->pwr_adj);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_auto_rate
LMP_MSG_HANDLER(auto_rate)
{
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_pref_rate
LMP_MSG_HANDLER(pref_rate)
{
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];
        uint8_t maxslot;
        uint16_t pkt_type;

        lc_env_ptr->link.RxPreferredRate = lc_util_convert_pref_rate_to_packet_type(param->rate);

        // Check maximum number of slots that can be used
        maxslot = lc_check_max_slot(idx);

        // Check what packet types can be used
        pkt_type = lc_util_convert_maxslot_to_packet_type(maxslot);
        pkt_type = LM_ComputePacketType(pkt_type, lc_env_ptr->link.RxPreferredRate, true);
        pkt_type = LM_ComputePacketType(pkt_type, lc_env_ptr->link.CurPacketType, true);

        if (pkt_type != lc_env_ptr->link.CurPacketType)
        {
            ld_acl_allowed_tx_packet_types_set(idx, pkt_type);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ver_req
LMP_MSG_HANDLER(ver_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        lc_env_ptr->info.RemVers.compid  = param->co_id;
        lc_env_ptr->info.RemVers.subvers = param->subver;
        lc_env_ptr->info.RemVers.vers    = param->ver;
        // send LMP_VersionRes (LMP_VERSION,LMP_MANUFACTURER_NAME,LMP_SUB_VERSION, param->TransactionId)
        {
            struct lmp_ver_res pdu;

            // fill the parameters
            pdu.opcode = LMP_OPCODE(LMP_VER_RES_OPCODE, (GETB(param->opcode, LMP_TR_ID)));
            pdu.subver  = co_htobs(CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD));
            pdu.ver     = RWBT_SW_VERSION_MAJOR;
            pdu.co_id   = RW_COMP_ID;

            // send the lmp pdu
            lc_send_lmp(idx, &pdu);
        }
        lc_env_ptr->info.RecvRemVerRec = true;
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ver_res
LMP_MSG_HANDLER(ver_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    switch (ke_state_get(dest_id))
    {
#if BCAST_ENC_SUPPORT
    case LC_MST_WAIT_VERS:
        if (param->ver >= BT_LMP_V1_2)
        {
            lc_env_ptr->enc.KeySizeMask = 0;
        }
        else
        {
            lc_env_ptr->enc.KeySizeMask = 0xFFFF;
        }
        lc_mst_send_mst_key(dest_id);
        break;
#endif // BCAST_ENC_SUPPORT
    case LC_WAIT_REM_VERS:
        lc_env_ptr->info.RemVers.compid = param->co_id;
        lc_env_ptr->info.RemVers.subvers = param->subver;
        lc_env_ptr->info.RemVers.vers = param->ver;
        if (lc_env_ptr->req.LocVersReq)
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_rd_rem_ver_info_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_VER_INFO_CMP_EVT_CODE, hci_rd_rem_ver_info_cmp_evt);
            evt->status  = CO_ERROR_NO_ERROR;
            evt->conhdl  = conhdl;
            evt->compid  = param->co_id;
            evt->subvers = param->subver;
            evt->vers    = param->ver;
            // send the event
            hci_send_2_host(evt);

            lc_env_ptr->req.LocVersReq = false;
        }
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        lc_env_ptr->info.RecvRemVerRec = true;
        ke_state_set(dest_id, LC_CONNECTED);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_feats_req
LMP_MSG_HANDLER(feats_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        memcpy(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[0], &param->feats.feats[0], FEATS_LEN);

        lc_send_pdu_feats_res(idx, GETB(param->opcode, LMP_TR_ID));

        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_RSSI_BIT_POS))
        {
            lc_env_ptr->link.epc_supported = (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_EPC_BIT_POS)) ? true : false;
        }
        lc_env_ptr->info.RemFeatureRec |= 0x1;
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_feats_res
LMP_MSG_HANDLER(feats_res)
{
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_REM_FEATS:
    {
        int idx = KE_IDX_GET(dest_id);
        struct lc_env_tag *lc_env_ptr = lc_env[idx];

        // save remote features information
        memcpy(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[0], &param->feats.feats[0], FEATS_LEN);

        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);

        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_RSSI_BIT_POS))
        {
            lc_env_ptr->link.epc_supported = (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_EPC_BIT_POS)) ? true : false;
        }
        lc_env_ptr->info.RemFeatureRec |= 0x1;

        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_EXT_FEATS_BIT_POS))
        {
            lc_ext_feat(dest_id, idx, FEATURE_PAGE_1);
        }
        else
        {
            ke_state_set(dest_id, LC_CONNECTED);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_qos
LMP_MSG_HANDLER(qos)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            lc_env_ptr->link.PollInterval = param->poll_intv;

            LM_GetQoSParam(lc_env_ptr->link.Role,
                           &lc_env_ptr->link.ServiceType,
                           &lc_env_ptr->link.TokenRate,
                           &lc_env_ptr->link.PeakBandwidth,
                           &lc_env_ptr->link.Latency,
                           &lc_env_ptr->link.DelayVariation,
                           &lc_env_ptr->link.PollInterval,
                           lc_env_ptr->link.CurPacketType,
                           QOS_PEER_FORCE);

            ld_acl_t_poll_set(idx, lc_env_ptr->link.PollInterval);
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_qos_setup_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_QOS_SETUP_CMP_EVT_CODE, hci_qos_setup_cmp_evt);

            // update the status
            event->status    = CO_ERROR_NO_ERROR;
            event->conhdl    = conhdl;
            event->flags     = 0x00;
            event->tok_rate  = lc_env_ptr->link.TokenRate;
            event->serv_type = lc_env_ptr->link.ServiceType;
            event->pk_bw     = lc_env_ptr->link.PeakBandwidth;
            event->lat       = lc_env_ptr->link.Latency;
            event->del_var   = lc_env_ptr->link.DelayVariation;

            // send the message
            hci_send_2_host(event);
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_qos_req
LMP_MSG_HANDLER(qos_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        lc_env_ptr->link.PollInterval = param->poll_intv;

        uint8_t status = LM_GetQoSParam(lc_env_ptr->link.Role, &lc_env_ptr->link.ServiceType,
                                        &lc_env_ptr->link.TokenRate, &lc_env_ptr->link.PeakBandwidth, &lc_env_ptr->link.Latency,
                                        &lc_env_ptr->link.DelayVariation, &lc_env_ptr->link.PollInterval, lc_env_ptr->link.CurPacketType,
                                        QOS_PEER_REQ);
        if (status == CO_ERROR_NO_ERROR)
        {
            ld_acl_t_poll_set(idx, lc_env_ptr->link.PollInterval);
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            // allocate the status event message
            struct hci_qos_setup_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_QOS_SETUP_CMP_EVT_CODE, hci_qos_setup_cmp_evt);

            // update the status
            event->status    = status;
            event->conhdl    = conhdl;
            event->flags     = 0x00;
            event->tok_rate  = lc_env_ptr->link.TokenRate;
            event->serv_type = lc_env_ptr->link.ServiceType;
            event->pk_bw     = lc_env_ptr->link.PeakBandwidth;
            event->lat       = lc_env_ptr->link.Latency;
            event->del_var   = lc_env_ptr->link.DelayVariation;
            // send the message
            hci_send_2_host(event);

            // LMP_Accepted (LMP_QOS_REQ_OPCODE)
            lc_send_pdu_acc(idx, LMP_QOS_REQ_OPCODE, GETB(param->opcode, LMP_TR_ID));
        }
        else
        {
            // send LMP_NotAccepted(LMP_QOS_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_QOS_REQ_OPCODE, status, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sco_link_req
LMP_MSG_HANDLER(sco_link_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer  = GETB(param->opcode, LMP_TR_ID);

    lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        // SCO cannot be established if AES-CCM encryption is enabled
        if (lc_env_ptr->enc.EncEnable && lc_env_ptr->sp.sec_con)
        {
            lc_send_pdu_not_acc(idx, LMP_SCO_LINK_REQ_OPCODE, CO_ERROR_CONN_REJ_SECURITY_REASONS, GETB(param->opcode, LMP_TR_ID));
            return (KE_MSG_CONSUMED);
        }
    } // No break
    case LC_SCO_NEGO_ONGOING:
    {
#if (MAX_NB_SYNC > 0)
        struct lc_sco_air_params_tag peer_params;
        peer_params.tr_id          = GETB(param->opcode, LMP_TR_ID);
        peer_params.esco_hdl       = param->sco_hdl;
        peer_params.esco_lt_addr   = 0;
        peer_params.flags          = param->flags;
        peer_params.d_esco         = param->d_sco;
        peer_params.t_esco         = param->t_sco;
        peer_params.w_esco         = 0;
        peer_params.m2s_pkt_type   = param->sco_pkt;
        peer_params.s2m_pkt_type   = param->sco_pkt;
        peer_params.m2s_pkt_len    = 0;
        peer_params.s2m_pkt_len    = 0;
        peer_params.air_mode       = param->air_mode;
        peer_params.nego_state     = 0;

        lc_sco_peer_request(dest_id, SCO_TYPE, &peer_params);
#else // (MAX_NB_SYNC > 0)
        // send LMP_NotAccepted(LMP_SCO_LINK_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SCO_LINK_REQ_OPCODE, CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED, GETB(param->opcode, LMP_TR_ID));
#endif // (MAX_NB_SYNC > 0)
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_rmv_sco_link_req
LMP_MSG_HANDLER(rmv_sco_link_req)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
#if (MAX_NB_SYNC > 0)
        lc_sco_peer_request_disc(dest_id, SCO_TYPE, param->sco_hdl, param->reason);
#else // (MAX_NB_SYNC > 0)
        // send LMP_NotAccepted(LMP_RMV_SCO_LINK_REQ_OPCODE)
        lc_send_pdu_not_acc(KE_IDX_GET(dest_id), LMP_RMV_SCO_LINK_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, GETB(param->opcode, LMP_TR_ID));
#endif // (MAX_NB_SYNC > 0)
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_max_slot
LMP_MSG_HANDLER(max_slot)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        if ((param->max_slots == 0x01)
                || (param->max_slots == 0x03)
                || (param->max_slots == 0x05))
        {
            lc_env_ptr->link.MaxSlotReceived = param->max_slots;

            lc_max_slot_mgt(dest_id, param->max_slots);
        }
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_max_slot_req
LMP_MSG_HANDLER(max_slot_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        if ((param->max_slots == 0x01) ||
                (((param->max_slots == 0x03) || (param->max_slots == 0x05))
#if (MAX_NB_SYNC > 0)
                 && (lm_get_nb_sync_link() == 0)
#endif //(MAX_NB_SYNC > 0)
#if (EAVESDROPPING_SUPPORT)
                 && (param->max_slots <= lm_get_host_max_slot())
#endif // EAVESDROPPING_SUPPORT
                ))
        {
            // LMP_Accepted(LMP_MAX_SLOT_REQ_OPCODE)
            lc_send_pdu_acc(idx, LMP_MAX_SLOT_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);

            // Update the maximum number of slots the peer device is allowed to use
            ld_acl_rx_max_slot_set(idx, param->max_slots);
            break;
        }
        else
        {
            // send LMP_NotAccepted(LMP_MAX_SLOT_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_MAX_SLOT_REQ_OPCODE, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE, lc_env_ptr->link.RxTrIdServer);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_timing_accu_req
LMP_MSG_HANDLER(timing_accu_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        // send LMP_TimingAccuracyRes
        struct lmp_timing_accu_res pdu;

        // update the parameters
        pdu.opcode = LMP_OPCODE(LMP_TIMING_ACCU_RES_OPCODE, lc_env_ptr->link.RxTrIdServer);
        pdu.jitter  = 0;
        pdu.drift   = rwip_max_drift_get();
        // send the lmp pdu
        lc_send_lmp(idx, &pdu);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_timing_accu_res
LMP_MSG_HANDLER(timing_accu_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_TIM_ACC_RSP:
    {
        // simple check on the parameters
        if ((param->drift > BT_MAX_DRIFT_SLEEP) || (param->jitter > BT_MAX_JITTER))
            lc_send_pdu_not_acc(idx, LMP_TIMING_ACCU_RES_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

        // Set drift and jitter
        ld_acl_timing_accuracy_set(idx, co_min(param->drift, BT_MAX_DRIFT_SLEEP), co_min(param->jitter, BT_MAX_JITTER));
        ke_state_set(dest_id, LC_CONNECTED);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
    }
    break;
#if RW_DEBUG
    case LC_WAIT_DBG_SEND_LMP_CFM:
        ke_state_set(dest_id, LC_CONNECTED);
        lc_cmd_cmp_conhdl_send(HCI_DBG_BT_SEND_LMP_CMD_OPCODE, CO_ERROR_NO_ERROR, BT_ACL_CONHDL_MIN + idx);
        break;
#endif // RW_DEBUG
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);

}

/// Handles the LMP packet LMP_setup_cmp
LMP_MSG_HANDLER(setup_cmp)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    case LC_CONNECTED:
    {
        lc_env_ptr->link.SetupCompRx = true;

        // Complete ACL connection establishment
        lc_setup_cmp(idx);
    }
    break;
    default:
    {
        lc_env_ptr->link.SetupCompRx = true;

        // Post a message to indicate to complete the ACL conneciton establishment
        ke_msg_send_basic(LC_OP_SET_CMP_IND, dest_id, dest_id);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_use_semi_perm_key
LMP_MSG_HANDLER(use_semi_perm_key)
{
#if BCAST_ENC_SUPPORT
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        memcpy(&lc_env_ptr->enc.LTKey.ltk[0], &lc_env_ptr->enc.SemiPermanentKey.ltk[0], KEY_LEN);
        lc_env_ptr->enc.NewKeyFlag = SEMI_PERMANENT_KEY;
        lc_env_ptr->req.MasterKeyReq = true;
        lc_mst_key(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        return (KE_MSG_SAVED);
    }
#endif // BCAST_ENC_SUPPORT
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_host_con_req
LMP_MSG_HANDLER(host_con_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (!lc_env_ptr->link.HostConnected && (lc_env_ptr->link.Role == SLAVE_ROLE))
        {
            // allocate the status event message
            struct hci_con_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, idx, HCI_CON_REQ_EVT_CODE, hci_con_req_evt);
            // gets the device address from the registers
            evt->lk_type = ACL_TYPE;
            memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
            memcpy(&evt->classofdev.A[0], &lc_env_ptr->link.Class.A[0], DEV_CLASS_LEN);
            // send the message
            hci_send_2_host(evt);

            lc_env_ptr->link.HostConnected = true;
            ke_timer_set(LC_CON_ACCEPT_TO, dest_id, 10 * co_slot_to_duration(hci_con_accept_to_get()));
            ke_state_set(dest_id, LC_WAIT_ACL_ACC);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_slot_off
LMP_MSG_HANDLER(slot_off)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        lc_env_ptr->link.SlotOffset = param->slot_off;

        ld_acl_rsw_slot_offset_set(idx, param->slot_off);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_page_mode_req
LMP_MSG_HANDLER(page_mode_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        // send LMP_NotAccepted(LMP_PAGE_MODE_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_PAGE_MODE_REQ_OPCODE, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE, lc_env_ptr->link.RxTrIdServer);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_page_scan_mode_req
LMP_MSG_HANDLER(page_scan_mode_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        // send LMP_NotAccepted(LMP_PAGE_SCAN_MODE_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_PAGE_SCAN_MODE_REQ_OPCODE, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE, lc_env_ptr->link.RxTrIdServer);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_supv_to
LMP_MSG_HANDLER(supv_to)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            if ((lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE) && (param->supv_to != 0))
            {
                uint16_t interval;
                uint8_t max_subrate;

                lc_sniff_interval_get(idx, &interval, &max_subrate);

                if (param->supv_to < (interval * max_subrate))
                {
                    ld_acl_ssr_set(idx, 0, 0, 0);
                }
            }

            ld_acl_lsto_set(idx, param->supv_to);

            {
                uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
                // link supervision timeout change event
                struct hci_link_supv_to_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_LINK_SUPV_TO_CHG_EVT_CODE, hci_link_supv_to_chg_evt);
                // fill up the parameters
                evt->conhdl   = conhdl;
                evt->lsto_val = param->supv_to;
                // send the event
                hci_send_2_host(evt);
            }
        }
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_test_activate
LMP_MSG_HANDLER(test_activate)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    // Check current test mode state : enabled/activated
    if (lm_dut_mode_en_get() && (!lc_env_ptr->tst_mode.activated))
    {
        //LMP_Accepted
        lc_send_pdu_acc(idx, LMP_TEST_ACTIVATE_OPCODE, GETB(param->opcode, LMP_TR_ID));

        // Set the activation state
        lc_env_ptr->tst_mode.activated = true;
    }
    else
    {
        // send LMP_NotAccepted(LMP_TEST_ACTIV_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_TEST_ACTIVATE_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_test_ctrl
LMP_MSG_HANDLER(test_ctrl)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer =  GETB(param->opcode, LMP_TR_ID);

    if (lc_env_ptr->tst_mode.activated)
    {
        uint8_t status = CO_ERROR_NO_ERROR;

        /* Each received parameters need to be XORed with 0x55 as specified in the      */
        /* Bluetooth specification, part I.1.                                           */
        uint8_t  TestScenario = param->scenario    ^ 0x55;
        uint8_t  HoppingMode  = param->hop         ^ 0x55;
        uint8_t  TxFreq       = param->tx_freq     ^ 0x55;
        uint8_t  RxFreq       = param->rx_freq     ^ 0x55;
        uint8_t  PowerControl = param->pwr_ctrl    ^ 0x55;
        uint8_t  PollPeriod   = param->poll_period ^ 0x55;
        uint8_t  PacketType   = param->pkt_type    ^ 0x55;
        uint16_t DataLength   = param->data_len    ^ 0x5555;

        switch (TestScenario)
        {
        case PAUSE_MODE :
        case EXITTEST_MODE :
        case TXTEST0_MODE :
        case TXTEST1_MODE :
        case TXTEST10_MODE :
        case PRAND_MODE :
        case ACLLOOP_MODE :
        case ACLNOWHIT_MODE :
        case TXTEST1100_MODE :
            break;
        case SCOLOOP_MODE :
        case SCONOWHIT_MODE :
#if (MAX_NB_SYNC == 0)
            status = CO_ERROR_UNSUPPORTED;
#endif /* (MAX_NB_SYNC == 0) */
            break;

        default:
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        switch (HoppingMode)
        {
        case HOPSINGLE :
        case HOPUSA :
            break;

        default:
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        switch (PowerControl)
        {
        case FIXTXPOW :
        case ADAPTTXPOW :
            break;

        default:
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        switch ((PacketType & LMP_TEST_CTRL_PKT_TYPE_LINK_MSK) >> LMP_TEST_CTRL_PKT_TYPE_LINK_POS)
        {
        case TEST_ACLSCO:
        case TEST_ESCO:
        case TEST_EDRACL:
        case TEST_EDRESCO:
            break;

        default:
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        if (status == CO_ERROR_NO_ERROR)
        {
            uint8_t NbFlushed;

            // Stop ACL flow
            ld_acl_flow_off(idx);

            // Flush all pending ACL packets
            ld_acl_data_flush(idx, &NbFlushed, true);

            lc_env_ptr->tst_mode.TestScenario = TestScenario;
            lc_env_ptr->tst_mode.HoppingMode  = HoppingMode ;
            lc_env_ptr->tst_mode.TxFreq       = TxFreq      ;
            lc_env_ptr->tst_mode.RxFreq       = RxFreq      ;
            lc_env_ptr->tst_mode.PowerControl = PowerControl;
            lc_env_ptr->tst_mode.PollPeriod   = PollPeriod  ;
            lc_env_ptr->tst_mode.PacketType   = PacketType  ;
            lc_env_ptr->tst_mode.DataLength   = DataLength  ;

            // LMP_Accepted(idx,LMP_TEST_CNTL_OPCODE,rx_tr_id)
            lc_send_pdu_acc(idx, LMP_TEST_CTRL_OPCODE, lc_env_ptr->link.RxTrIdServer);
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_TEST_TX_CFM);
        }
        else
        {
            // send LMP_NotAccepted
            lc_send_pdu_not_acc(idx, LMP_TEST_CTRL_OPCODE, status, lc_env_ptr->link.RxTrIdServer);
        }
    }
    else
    {
        // send LMP_NotAccepted
        lc_send_pdu_not_acc(idx, LMP_TEST_CTRL_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_enc_key_size_mask_req
LMP_MSG_HANDLER(enc_key_size_mask_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

#if BCAST_ENC_SUPPORT
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            // send LMP_NotAccepted(LMP_ENC_KSZ_MSK_REQ_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENC_KEY_SIZE_MASK_REQ_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        }
        else
        {
            // send LMP_EncryptioKeySizeMaskRes
            struct lmp_enc_key_size_mask_res pdu;
            pdu.opcode = LMP_OPCODE(LMP_ENC_KEY_SIZE_MASK_RES_OPCODE, lc_env_ptr->link.RxTrIdServer);
            pdu.mask   = ENC_KEY_SIZE_MASK;
            lc_send_lmp(idx, &pdu);
        }
#else // !BCAST_ENC_SUPPORT
        // send LMP_NotAccepted(LMP_ENC_KSZ_MSK_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_ENC_KEY_SIZE_MASK_REQ_OPCODE, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE, lc_env_ptr->link.RxTrIdServer);
#endif // !BCAST_ENC_SUPPORT
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_enc_key_size_mask_res
LMP_MSG_HANDLER(enc_key_size_mask_res)
{
#if BCAST_ENC_SUPPORT
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_MST_WAIT_ENC_SIZE_MSK:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        lc_env_ptr->enc.KeySizeMask = param->mask;
        lc_mst_send_mst_key(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
#endif // BCAST_ENC_SUPPORT
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_set_afh
LMP_MSG_HANDLER(set_afh)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    DBG_SWDIAG(AFH, LMP_RX, 1);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            if ((param->mode == AFH_DISABLED) || (param->mode == AFH_ENABLED))
            {
                memcpy(&lc_env_ptr->afh.ch_map.map[0], &param->map.map[0], BT_CH_MAP_LEN);
                ld_acl_afh_set(idx, param->instant, param->mode, &param->map);
                lc_env_ptr->afh.en = param->mode;
                break;
            }
            else
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        else
        {
            status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
        }

        // send LMP_NotAccepted(LMP_SET_AFH_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SET_AFH_OPCODE, status, lc_env_ptr->link.RxTrIdServer);
        break;
    }

    DBG_SWDIAG(AFH, LMP_RX, 0);

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_encaps_hdr
LMP_MSG_HANDLER(encaps_hdr)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PUB_KEY_HEADER_RSP_PEER:
        do
        {
            if (lc_env_ptr->sp.SpTId == lc_env_ptr->link.RxTrIdServer)
            {
                // Check parameters
                if ((lc_env_ptr->sp.sec_con && (param->maj_type == LMP_ENCAPS_P256_MAJ_TYPE) && (param->min_type == LMP_ENCAPS_P256_MIN_TYPE) && (param->payl_len == LMP_ENCAPS_P256_PAYL_LEN))
                        || (!lc_env_ptr->sp.sec_con && (param->maj_type == LMP_ENCAPS_P192_MAJ_TYPE) && (param->min_type == LMP_ENCAPS_P192_MIN_TYPE) && (param->payl_len == LMP_ENCAPS_P192_PAYL_LEN)))
                {
                    lc_env_ptr->sp.EncapPduCtr = 0;

                    //LMP_Accepted (LMP_ENCAPS_HDR_OPCODE)
                    lc_send_pdu_acc(idx, LMP_ENCAPS_HDR_OPCODE, lc_env_ptr->link.RxTrIdServer);

                    lc_start_lmp_to(dest_id);

                    ke_state_set(dest_id, LC_WAIT_PUB_KEY_PAYLOAD_RSP_PEER);
                    break;
                }
                else
                {
                    // incorrect header, authentication failure
                    status = CO_ERROR_AUTH_FAILURE;
                }
            }
            else
            {
                // Transaction ID not equal to SPTID
                status = CO_ERROR_INVALID_LMP_PARAM;
            }

            // send LMP_NotAccepted(LMP_ENCAPS_HDR_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENCAPS_HDR_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

            // simple pairing fail procedure
            lc_sp_fail(dest_id);
        }
        while (0);

        break;
    case LC_WAIT_PUB_KEY_HEADER_INIT_PEER:
        do
        {
            if (lc_env_ptr->sp.SpTId == lc_env_ptr->link.RxTrIdServer)
            {
                // Check parameters
                if ((lc_env_ptr->sp.sec_con && (param->maj_type == LMP_ENCAPS_P256_MAJ_TYPE) && (param->min_type == LMP_ENCAPS_P256_MIN_TYPE) && (param->payl_len == LMP_ENCAPS_P256_PAYL_LEN))
                        || (!lc_env_ptr->sp.sec_con && (param->maj_type == LMP_ENCAPS_P192_MAJ_TYPE) && (param->min_type == LMP_ENCAPS_P192_MIN_TYPE) && (param->payl_len == LMP_ENCAPS_P192_PAYL_LEN)))
                {
                    // set the pdu counter to 0
                    lc_env_ptr->sp.EncapPduCtr = 0 ;

                    // send accepted lmp message for encapsulated header
                    lc_send_pdu_acc(idx, LMP_ENCAPS_HDR_OPCODE, lc_env_ptr->link.RxTrIdServer);

                    // start timer
                    lc_start_lmp_to(dest_id);

                    // change state to wait for public key payload
                    ke_state_set(dest_id, LC_WAIT_PUB_KEY_PAYLOAD_INIT_PEER);
                    break;
                }
            }
            // send LMP_NotAccepted(LMP_ENCAPS_HDR_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_ENCAPS_HDR_OPCODE, CO_ERROR_INVALID_LMP_PARAM, GETB(param->opcode, LMP_TR_ID));

            // authentication failure
            lc_sp_fail(dest_id);
        }
        while (0);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_encaps_payl
LMP_MSG_HANDLER(encaps_payl)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // save the parameters from the message
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    memcpy(&lc_env_ptr->sp.LocRandN.A[0], &param->data.A[0], KEY_LEN);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PUB_KEY_PAYLOAD_RSP_PEER:
    case LC_WAIT_PUB_KEY_PAYLOAD_INIT_PEER:
        if (lc_env_ptr->sp.SpTId == lc_env_ptr->link.RxTrIdServer)
        {
            uint8_t nb_payl = (lc_env_ptr->sp.sec_con) ? LMP_ENCAPS_P256_PAYL_NB : LMP_ENCAPS_P192_PAYL_NB;

            uint8_t *p_remote_pub_key_x =  lc_env_ptr->sp.p_data->remote_pub_key.x;
            uint8_t *p_remote_pub_key_y =  lc_env_ptr->sp.p_data->remote_pub_key.y;

            if (lc_env_ptr->sp.sec_con)
            {
                lc_extract_rem_pub_key_256_from_encap_pdu(&lc_env_ptr->sp.LocRandN, p_remote_pub_key_x, p_remote_pub_key_y,
                        lc_env_ptr->sp.EncapPduCtr);
            }
            else
            {
                lc_extract_rem_pub_key_192_from_encap_pdu(&lc_env_ptr->sp.LocRandN, p_remote_pub_key_x, p_remote_pub_key_y,
                        lc_env_ptr->sp.EncapPduCtr);
            }

            lc_env_ptr->sp.EncapPduCtr += 1;

            if (lc_env_ptr->sp.EncapPduCtr < nb_payl)
            {
                // LMP_Accepted (LMP_ENCAPS_PAYL_OPCODE)
                lc_send_pdu_acc(idx, LMP_ENCAPS_PAYL_OPCODE, lc_env_ptr->link.RxTrIdServer);
                lc_start_lmp_to(dest_id);
                break;
            }


            if (lc_env_ptr->sp.SPInitiator)
            {
                // check public key validity
                if (!lc_sp_check_remote_pub_key_valid(dest_id, lc_env_ptr))
                {
                    break;
                }

                // LMP_Accepted (LMP_ENCAPS_PAYL_OPCODE)
                lc_send_pdu_acc(idx, LMP_ENCAPS_PAYL_OPCODE, lc_env_ptr->link.RxTrIdServer);

                /*
                 * START COMPUTING DIFFIE HELLMAN KEY (only if received public key is valid)
                 */
                lc_dhkey_computation_start(dest_id);

                /*
                 * STOP THE PUBLICK KEY EXCHANGE
                 */
                /*
                 * SP AUTH STG1 : START
                 */

                //Set default key
                lc_env_ptr->enc.KeyType = (lc_env_ptr->sp.sec_con) ? BT_AUTH_COMB_KEY_256 : BT_AUTH_COMB_KEY_192;

                // Check the common authentication method between local and peer
                switch (lm_get_auth_method(&lc_env_ptr->sp.IOCap_loc, &lc_env_ptr->sp.IOCap_rem))
                {
                case SP_JUST_WORKS:
                    // just works method
                    lc_env_ptr->enc.KeyType = (lc_env_ptr->sp.sec_con) ? BT_UNAUTH_COMB_KEY_256 : BT_UNAUTH_COMB_KEY_192;
                case SP_NUM_COMP:
                    // numeric comparison method
                    ke_state_set(dest_id, LC_WAIT_AUTH_STG1_NC_RANDN);
                    ke_msg_send_basic(LC_AUTH_STG1_RANDN, dest_id, dest_id);
                    break;
                case SP_PASSKEY:
                    // pass key method
                    ke_state_set(dest_id, LC_WAIT_AUTH_STG1_PK_RANDN);
                    ke_msg_send_basic(LC_AUTH_STG1_RANDN, dest_id, dest_id);
                    break;
                case SP_OOB:
                    // OOB method
                    lc_start_oob(dest_id);
                    break;
                default:
                    break;
                }
            }
            else
            {
                // prepare for local public key transmission
                lc_sp_prepare_public_key_exchange(dest_id, lc_env_ptr);
            }
        }
        break;

    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sp_cfm
LMP_MSG_HANDLER(sp_cfm)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PASSKEY_COMM_RSP_PEER:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &param->commitment_val.A[0], KEY_LEN);

            lc_passkey_comm(dest_id);
        }
        else
        {
            // send LMP_NotAccepted(LMP_SP_CFM_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_SP_CFM_OPCODE, CO_ERROR_AUTH_FAILURE, lc_env_ptr->link.RxTrIdServer);
        }
        break;

    case LC_WAIT_PASSKEY_COMM_INIT_PEER:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &param->commitment_val.A[0], KEY_LEN);

            lc_send_pdu_sp_nb(idx, &lc_env_ptr->sp.LocRandN, lc_env_ptr->sp.SpTId);

            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_PASSKEY_RANDN_INIT_CFM);
        }
        break;
    case LC_WAIT_NUM_COMP_COMM_INIT_CONF:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            memcpy(&lc_env_ptr->sp.RemCommitment.A[0], &param->commitment_val.A[0], KEY_LEN);

            lc_start_lmp_to(dest_id);

            lc_send_pdu_sp_nb(idx, &lc_env_ptr->sp.LocRandN, lc_env_ptr->sp.SpTId);

            ke_state_set(dest_id, LC_WAIT_NUM_COMP_COMM_INIT_RANDN_CFM);
        }
        break;
    case LC_WAIT_PASSKEY_HL_RPLY:
    case LC_WAIT_DHKEY_COMPUTING:
    case LC_WAIT_AUTH_STG1_NC_RANDN:
    case LC_WAIT_AUTH_STG1_PK_RANDN:
        return (KE_MSG_SAVED);
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sp_nb
LMP_MSG_HANDLER(sp_nb)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    memcpy(&lc_env_ptr->sp.RemRandN.A[0], &param->nonce.A[0], KEY_LEN);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_OOB_RANDN_RSP_PEER:
        lc_resp_oob_nonce(dest_id);
        break;

    case LC_WAIT_OOB_RANDN_INIT_PEER:
        do
        {
            if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
            {
                if (!lc_env_ptr->sp.SPPhase1Failed)
                {
                    // reuse local commitment
                    memcpy(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.p_data->local_oob_rand, sizeof(struct byte16));
                    lc_send_pdu_acc(idx, LMP_SP_NB_OPCODE, lc_env_ptr->sp.SpTId);
                    /*
                     * SP AUTH STG1 : END
                     */
                    /*
                     * SP AUTH STG2 : START
                     */
                    lc_try_start_auth_stage2(dest_id);
                    break;
                }
                else
                {
                    status = CO_ERROR_AUTH_FAILURE;
                }
            }
            else
            {
                status = CO_ERROR_AUTH_FAILURE;
            }
            // send LMP_NotAccepted(LMP_SP_NB_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

            lc_sp_fail(dest_id);
        }
        while (0);
        break;

    case LC_WAIT_PASSKEY_RANDN_RSP_PEER:
        do
        {
            if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
            {
                uint8_t passkey_bit = ((lc_env_ptr->sp.Passkey & (1 << lc_env_ptr->sp.EncapPduCtr)) ? 0x81 : 0x80);

                const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
                const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

                sha_256_f1(lc_env_ptr->sp.sec_con, p_remote_pub_key_x, p_local_pub_key_x,
                           &lc_env_ptr->sp.RemRandN.A[0],
                           &passkey_bit,
                           &lc_env_ptr->sp.LocCommitment.A[0]);

                if (!memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.RemCommitment, KEY_LEN))
                {
                    // send LMP_Accepted
                    lc_send_pdu_acc(idx, LMP_SP_NB_OPCODE, lc_env_ptr->link.RxTrIdServer);

                    // send LMP_Simplepairingnumber(localnonce, sptid)
                    lc_send_pdu_sp_nb(idx, &lc_env_ptr->sp.LocRandN, lc_env_ptr->link.RxTrIdServer);

                    // start lmp timer
                    lc_start_lmp_to(dest_id);

                    // change state
                    ke_state_set(dest_id, LC_WAIT_PASSKEY_RANDN_RSP_PEER_CFM);
                    break;
                }
                else
                {
                    status = CO_ERROR_AUTH_FAILURE;
                }
            }
            else
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
            }

            // send LMP_NotAccepted
            lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

            // sp authentication fail
            lc_sp_fail(dest_id);
        }
        while (0);
        break;

    case LC_WAIT_PASSKEY_RANDN_INIT_PEER:
        do
        {
            if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
            {
                uint8_t passkey_bit = ((lc_env_ptr->sp.Passkey & (1 << lc_env_ptr->sp.EncapPduCtr)) ? 0x81 : 0x80);
                const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
                const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

                sha_256_f1(lc_env_ptr->sp.sec_con, p_remote_pub_key_x, p_local_pub_key_x,
                           &lc_env_ptr->sp.RemRandN.A[0],
                           &passkey_bit,
                           &lc_env_ptr->sp.LocCommitment.A[0]);

                if (!memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.RemCommitment, KEY_LEN))
                {
                    // send LMP_Accepted
                    lc_send_pdu_acc(idx, LMP_SP_NB_OPCODE, lc_env_ptr->sp.SpTId);

                    lc_env_ptr->sp.EncapPduCtr += 1;

                    if (lc_env_ptr->sp.EncapPduCtr  < SSP_PASSKEY_NB_BITS)
                    {
                        lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);
                        lc_init_passkey_loop(dest_id);
                    }
                    else
                    {
                        lm_sp_32bits_to_array(&lc_env_ptr->sp.RemCommitment, lc_env_ptr->sp.Passkey);
                        lm_sp_32bits_to_array(&lc_env_ptr->sp.LocCommitment, lc_env_ptr->sp.Passkey);
                        /*
                         * SP AUTH STG1 : END
                         */
                        /*
                         * SP AUTH STG2 : START
                         */
                        lc_try_start_auth_stage2(dest_id);
                    }
                    break;
                }
                else
                {
                    status = CO_ERROR_AUTH_FAILURE;
                }
            }
            else
            {
                status = CO_ERROR_HARDWARE_FAILURE;
            }

            // send LMP_NotAccepted
            lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

            lc_sp_fail(dest_id);
        }
        while (0);
        break;

    case LC_WAIT_NUM_COMP_RANDN_RSP_PEER:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            // send LMP_Accepted(idx LMP_SP_NB_OPCODE,sptid)
            lc_send_pdu_acc(idx, LMP_SP_NB_OPCODE, lc_env_ptr->link.RxTrIdServer);

            lc_start_lmp_to(dest_id);

            lc_send_pdu_sp_nb(idx, &lc_env_ptr->sp.LocRandN, lc_env_ptr->link.RxTrIdServer);

            ke_state_set(dest_id, LC_WAIT_NUM_COMP_RANDN_RSP_PEER_CFM);
        }
        else
        {
            // send LMP_NotAccepted(LMP_SP_NB_OPCO
            lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

            lc_sp_fail(dest_id);
        }
        break;

    case LC_WAIT_NUM_COMP_COMM_INIT_RANDN_PEER:
        do
        {
            if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
            {
                uint8_t Z = 0x00;
                const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
                const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

                sha_256_f1(lc_env_ptr->sp.sec_con, p_remote_pub_key_x, p_local_pub_key_x,
                           &lc_env_ptr->sp.RemRandN.A[0],
                           &Z,
                           &lc_env_ptr->sp.LocCommitment.A[0]);


                if (!memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.RemCommitment, KEY_LEN))
                {
                    // send LMP_Accepted(idx LMP_SP_NB_OPCODE,sptid)
                    lc_send_pdu_acc(idx, LMP_SP_NB_OPCODE, lc_env_ptr->sp.SpTId);

                    /*
                     * SP AUTH STG1 : COMMITMENT CHECK OK
                     */
                    /*
                     * SP AUTH STG1 : VERIFICATION CHECK
                     */
                    sha_256_g(lc_env_ptr->sp.sec_con, p_local_pub_key_x, p_remote_pub_key_x,
                              &lc_env_ptr->sp.LocRandN.A[0],
                              &lc_env_ptr->sp.RemRandN.A[0],
                              &lc_env_ptr->sp.Passkey);

                    /*
                     * SP AUTH STG1 : VERIFICATION CHECK OK
                     */
                    struct hci_user_cfm_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_USER_CFM_REQ_EVT_CODE, hci_user_cfm_req_evt);

                    // fill up parameters
                    evt->passkey = lc_env_ptr->sp.Passkey;
                    memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);

                    // send the event
                    hci_send_2_host(evt);

                    // start lmp timer
                    lc_start_lmp_to(dest_id);

                    ke_state_set(dest_id, LC_WAIT_NUM_COMP_USER_CONF_INIT);
                    break;
                }
                else
                {
                    /*
                     * SP AUTH STG1 : COMMITMENT CHECK NOT OK
                     */
                    status = CO_ERROR_AUTH_FAILURE;
                }
            }
            else
            {
                status = CO_ERROR_HARDWARE_FAILURE;
            }

            // send LMP_NotAccepted(LMP_SP_NB_OPCODE)
            lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

            // authentication failure
            lc_sp_fail(dest_id);
        }
        while (0);
        break;
    case LC_WAIT_HL_OOB_DATA:
    case LC_WAIT_DHKEY_COMPUTING:
        return (KE_MSG_SAVED);
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        // send LMP_NotAccepted(LMP_SP_NB_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);

        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_dhkey_chk
LMP_MSG_HANDLER(dhkey_chk)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    memcpy(&lc_env_ptr->sp.DHKeyCheck.ltk[0], &param->cfm_val.ltk[0], KEY_LEN);
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_DHKEY_RSP:
        // clear the timer
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            /* If the received public key is invalid, the Responder should send an LMP_not_accepted
             * PDU with reason "Authentication Failure".(LMP 4.2.7.4.1) */
            if (lc_env_ptr->sp.dh_key_compute_state == LC_DH_KEY_COMP_SUCCESS)
            {
                lc_resp_calc_f3(dest_id);
                break;
            }
        }
    //else
    case LC_WAIT_DHKEY_RSP_FAIL:
        // clear the timer
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        lc_send_pdu_not_acc(idx, LMP_DHKEY_CHK_OPCODE, CO_ERROR_AUTH_FAILURE, lc_env_ptr->sp.SpTId);

        lc_sp_fail(dest_id);
        break;

    case LC_WAIT_DHKEY_CHECK_INIT_PEER:
        do
        {
            if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
            {
                /* If the received public key is invalid, the Initiator should send an LMP_not_accepted
                 * PDU with reason "Authentication Failure".(LMP 4.2.7.4.1) */
                if (lc_env_ptr->sp.dh_key_compute_state != LC_DH_KEY_COMP_FAILED)
                {
                    uint8_t *p_dh_key = lc_env_ptr->sp.p_data->dh_key.key;
                    sha_256_f3(lc_env_ptr->sp.sec_con, p_dh_key,
                               &lc_env_ptr->sp.RemRandN.A[0],
                               &lc_env_ptr->sp.LocRandN.A[0],
                               &lc_env_ptr->sp.LocCommitment.A[0],
                               (uint8_t *) &lc_env_ptr->sp.IOCap_rem,
                               &lc_env_ptr->info.BdAddr.addr[0],
                               &lc_env_ptr->info.LocalBdAddr.addr[0],
                               &lc_env_ptr->sp.LocCommitment.A[0]);

                    if (!memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.DHKeyCheck, KEY_LEN))
                    {
                        lc_send_pdu_acc(idx, LMP_DHKEY_CHK_OPCODE, lc_env_ptr->sp.SpTId);
                        /*
                         * SP AUTH STG2: KEY DERIVATION FONCTION F2
                         */
                        lc_calc_link_key(dest_id);
                        break;
                    }
                }

                status = CO_ERROR_AUTH_FAILURE;
            }
            else
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
            }
            lc_send_pdu_not_acc(idx, LMP_DHKEY_CHK_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

            lc_sp_fail(dest_id);
        }
        while (0);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    case LC_WAIT_NUM_COMP_USER_CONF_RSP:
    case LC_WAIT_DHKEY_COMPUTING:
        return (KE_MSG_SAVED);
    default:
        lc_send_pdu_not_acc(idx, LMP_DHKEY_CHK_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        break;
    }
    return (KE_MSG_CONSUMED);
}


/*
 * EXTENDED LMP PACKETS HANDLERS
 ****************************************************************************************
 */

/// Handles the LMP packet LMP_accepted_ext
LMP_MSG_HANDLER(accepted_ext)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr != NULL)
    {
        // reset the values: LinkId, Opcode, OpcodeExt, Mode
        lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_ESCO_LINK_REQ_EXTOPCODE))
        {
            lc_sco_peer_accept(dest_id);
        }
        else
        {
            ASSERT_WARN(0, 0, param->ext_opcode);
        }
        break;

    case LC_SCO_DISC_ONGOING:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_RMV_ESCO_LINK_REQ_EXTOPCODE))
        {
            lc_sco_peer_accept_disc(dest_id);
        }
        else
        {
            ASSERT_WARN(0, 0, param->ext_opcode);
        }
        break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_PKT_TBL_TYP_ACC_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_PKT_TYPE_TBL_REQ_EXTOPCODE))
        {
            uint16_t pkt_type = 0x0000;

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            ld_acl_edr_set(idx, lc_env_ptr->link.ptt_tmp);
            lc_env_ptr->link.CurPacketTypeTable = lc_env_ptr->link.ptt_tmp;
            pkt_type = LM_ComputePacketType(lc_env_ptr->link.RxPreferredRate, lc_env_ptr->link.CurPacketType, true);

            //Update max packet size and preferred rate
            ld_acl_allowed_tx_packet_types_set(idx, pkt_type);
            ld_acl_flow_on(idx);
            lc_chg_pkt_type_retry(dest_id);
        }
        break;
#if PCA_SUPPORT
    case LC_WAIT_CLK_ADJ_REQ_ACC_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_CLK_ADJ_REQ_EXTOPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
#endif // PCA_SUPPORT
#if RW_BT_MWS_COEX
    case LC_WAIT_SAM_SET_TYPE0_CFM:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SAM_SET_TYPE0_EXTOPCODE))
        {
            // SAM Submap Type0 accepted by peer device.
            lc_env_ptr->sam_info.loc_submap0_av = true;

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);

            if (lc_env_ptr->sam_info.config_mode)
            {
                // Request further SAM configuration
                ke_msg_send_basic(LC_OP_SAM_IND, dest_id, dest_id);
            }
        }
        break;
    case LC_WAIT_SAM_DEFINE_MAP_CFM:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SAM_DEFINE_MAP_EXTOPCODE))
        {
            // Slot Map Definition accepted - Already configured locally, no further action required

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);

            if (lc_env_ptr->sam_info.config_mode)
            {
                // Request further SAM configuration
                ke_msg_send_basic(LC_OP_SAM_IND, dest_id, dest_id);
            }
        }
        break;
    case LC_WAIT_SAM_SWITCH_CFM:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SAM_SWITCH_EXTOPCODE))
        {
            uint8_t sam_idx = lc_env_ptr->sam_info.loc_idx_wait_cfm;
            uint8_t loc_tx_av = 0xFF;
            uint8_t loc_rx_av = 0xFF;

            if (sam_idx != SAM_DISABLED)
            {
                // Determine Local SAM TX, RX availability
                loc_tx_av = (((uint16_t)lc_env_ptr->sam_info.n_tx_slots << 8) - (lc_env_ptr->sam_info.n_tx_slots)) / lc_env_ptr->sam_info.t_sam;
                loc_rx_av = (((uint16_t)lc_env_ptr->sam_info.n_rx_slots << 8) - (lc_env_ptr->sam_info.n_rx_slots)) / lc_env_ptr->sam_info.t_sam;
            }

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);

            lc_env_ptr->sam_info.loc_tx_av = loc_tx_av;
            lc_env_ptr->sam_info.loc_rx_av = loc_rx_av;

            lc_env_ptr->sam_info.loc_t_sam_av = (sam_idx != SAM_DISABLED) ? lc_env_ptr->sam_info.t_sam : 0;

            // Update the local index, as now confirmed accepted
            lc_env_ptr->sam_info.loc_idx = sam_idx;

            // The new SAM slot map may be activated at SAM_Instant
            ld_acl_local_sam_index_set(idx, sam_idx, lc_env_ptr->sam_info.instant);
        }
        break;
#endif //RW_BT_MWS_COEX
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
#if RW_DEBUG
    case LC_WAIT_DBG_SEND_LMP_CFM:
        ke_state_set(dest_id, LC_CONNECTED);
        lc_cmd_cmp_conhdl_send(HCI_DBG_BT_SEND_LMP_CMD_OPCODE, CO_ERROR_NO_ERROR, BT_ACL_CONHDL_MIN + idx);
        break;
#endif // RW_DEBUG
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), param->orig_ext_opcode);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_not_accepted_ext
LMP_MSG_HANDLER(not_accepted_ext)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr != NULL)
    {
        // reset the values: LinkId, Opcode, OpcodeExt, Mode
        lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_ESCO_LINK_REQ_EXTOPCODE))
        {
            lc_sco_peer_reject(dest_id, param->reason);
        }
        else
        {
            ASSERT_WARN(0, 0, param->ext_opcode);
        }
        break;

    case LC_SCO_DISC_ONGOING:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_RMV_ESCO_LINK_REQ_EXTOPCODE))
        {
            lc_sco_peer_reject_disc(dest_id, param->reason);
        }
        else
        {
            ASSERT_WARN(0, 0, param->ext_opcode);
        }
        break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_IO_CAP_INIT_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_IO_CAP_REQ_EXTOPCODE))
        {
            // handle collision
            if ((lc_env_ptr->sp.SpTId != (GETB(param->opcode, LMP_TR_ID))) || (param->reason != CO_ERROR_LMP_COLLISION))
            {
                lc_sp_fail(dest_id);
            }
        }
        break;

    case LC_WAIT_SNIFF_SUB_RSP:
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SSR_REQ_EXTOPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            lc_sniff_ssr_peer_reject(dest_id, param->reason);
        }
        break;

    case LC_WAIT_PKT_TBL_TYP_ACC_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_PKT_TYPE_TBL_REQ_EXTOPCODE))
        {
            uint16_t pkt_type = 0x0000;

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            pkt_type = LM_ComputePacketType(lc_env_ptr->link.RxPreferredRate, lc_env_ptr->link.CurPacketType, true);

            //Update max packet size and preferred rate
            ld_acl_allowed_tx_packet_types_set(idx, pkt_type);
            ld_acl_flow_on(idx);
            lc_chg_pkt_type_retry(dest_id);
        }
        break;

    case LC_WAIT_REM_EXT_FEATS:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_FEATS_REQ_EXT_EXTOPCODE))
        {
            // structure type for the complete command event
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_rd_rem_ext_feats_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_EXT_FEATS_CMP_EVT_CODE, hci_rd_rem_ext_feats_cmp_evt);
            evt->status    = param->reason;
            evt->conhdl    = conhdl;
            memset(&evt->ext_feats.feats[0], 0, FEATS_LEN);
            evt->pg_nb     = 0;
            evt->pg_nb_max = 0;
            hci_send_2_host(evt);

            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;

    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_PAUSE_ENC_REQ_EXTOPCODE))
        {
            ASSERT_WARN((param->reason == CO_ERROR_LMP_COLLISION), ke_state_get(dest_id), param->reason);
        }
        break;

    case LC_WAIT_ENC_START_REQ_SLV:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_RESUME_ENC_REQ_EXTOPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            lc_env_ptr->link.Initiator = true;
            // enable flow control
            ld_acl_flow_on(idx);
            // send detach
            lc_detach(dest_id, CO_ERROR_ENC_MODE_NOT_ACCEPT);
        }
        break;
#if PCA_SUPPORT
    case LC_WAIT_CLK_ADJ_REQ_ACC_CFM:
        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_CLK_ADJ_REQ_EXTOPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);

#if RW_BT_MWS_COEX
            ke_timer_set(LB_PCA_MONITOR_INTV_TO, TASK_LB, BT_PCA_UPDATE_PERIOD);
#endif //RW_BT_MWS_COEX
        }
        break;
#endif // PCA_SUPPORT
#if RW_BT_MWS_COEX
    case LC_WAIT_SAM_SET_TYPE0_CFM:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SAM_SET_TYPE0_EXTOPCODE))
        {
            // SAM Submap Type0 rejected by peer device.
            lc_env_ptr->sam_info.loc_submap0_av = false;

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
    case LC_WAIT_SAM_DEFINE_MAP_CFM:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SAM_DEFINE_MAP_EXTOPCODE))
        {
            // SAM Pattern rejected by peer device.
            lc_env_ptr->sam_info.loc_pattern_av[lc_env_ptr->sam_info.loc_idx_wait_cfm] = false;

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
    case LC_WAIT_SAM_SWITCH_CFM:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) &&
                (param->orig_ext_opcode == LMP_SAM_SWITCH_EXTOPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
#endif //RW_BT_MWS_COEX
    case LC_WAIT_SNIFF_SUB_RSP_TX_CFM:
    case LC_WAIT_SSR_INSTANT:
    case LC_CONNECTED:
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_SSR_REQ_EXTOPCODE))
        {
            ASSERT_WARN((param->reason == CO_ERROR_LMP_COLLISION), ke_state_get(dest_id), param->reason);
        }
        break;
    case LC_WAIT_PWR_CTRL_RES:
    {
        if ((param->orig_esc_opcode == LMP_ESC4_OPCODE) && (param->orig_ext_opcode == LMP_PWR_CTRL_REQ_EXTOPCODE))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            ke_state_set(dest_id, LC_CONNECTED);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
#if RW_DEBUG
    case LC_WAIT_DBG_SEND_LMP_CFM:
        ke_state_set(dest_id, LC_CONNECTED);
        lc_cmd_cmp_conhdl_send(HCI_DBG_BT_SEND_LMP_CMD_OPCODE, param->reason, BT_ACL_CONHDL_MIN + idx);
        break;
#endif // RW_DEBUG
    default:
        ASSERT_WARN((param->reason == CO_ERROR_LMP_COLLISION), ke_state_get(dest_id), param->orig_ext_opcode);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_feats_req_ext
LMP_MSG_HANDLER(feats_req_ext)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        if (param->page <= FEATURE_PAGE_MAX)
        {
            // send LMP_FeaturesResExt
            struct lmp_feats_res_ext pdu;

            lm_read_features(param->page, &pdu.max_page, &pdu.ext_feats);
            pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.RxTrIdServer);
            pdu.ext_opcode = LMP_FEATS_RES_EXT_EXTOPCODE;
            pdu.page     = param->page;
            lc_send_lmp(idx, &pdu);

            // Store Remote extended feature mask
            memcpy(&lc_env_ptr->info.RemoteFeatures[param->page].feats[0], &param->ext_feats.feats[0], FEATS_LEN);
            lc_env_ptr->info.RemFeatureRec |= (1 << param->page);

            /*
             * If both devices support both the Secure Connections (Controller Support) and Secure Connections (Host Support)
             * features, the secure connections is active
             */
            lc_env_ptr->sp.sec_con = (lm_get_sec_con_host_supp() && lm_get_sp_en()
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_SSP_BIT_POS)
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_1], FEAT_SSP_HOST_BIT_POS)
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SEC_CON_CTRL_BIT_POS)
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_1], FEAT_SEC_CON_HOST_BIT_POS));
        }
        else
        {
            // Reject the request
            lc_send_pdu_not_acc_ext4(idx, LMP_FEATS_REQ_EXT_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_feats_res_ext
LMP_MSG_HANDLER(feats_res_ext)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_REM_EXT_FEATS:
    {
        // Check if feature page exists
        if (param->page > FEATURE_PAGE_MAX)
            break;
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // Store Remote extended feature mask
        memcpy(&lc_env_ptr->info.RemoteFeatures[param->page].feats[0], &param->ext_feats.feats[0], FEATS_LEN);
        lc_env_ptr->info.RemFeatureRec |= (1 << param->page);

        if (lc_env_ptr->link.HostConnected)
        {
            if (lc_env_ptr->req.LocRemoteExtendedReq)
            {
                // structure type for the complete command event
                uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
                struct hci_rd_rem_ext_feats_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_EXT_FEATS_CMP_EVT_CODE, hci_rd_rem_ext_feats_cmp_evt);
                memcpy(&evt->ext_feats.feats[0], &param->ext_feats.feats[0], FEATS_LEN);
                evt->status    = CO_ERROR_NO_ERROR;
                evt->pg_nb     = param->page;
                evt->pg_nb_max = param->max_page;
                evt->conhdl    = conhdl;
                hci_send_2_host(evt);

                lc_env_ptr->req.LocRemoteExtendedReq = false;
            }
        }
        else
        {
            if (lc_env_ptr->req.LocNameReq && (param->page == FEATURE_PAGE_1))
            {
                // structure type for the complete command event
                struct hci_rem_host_supp_feats_notif_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_HOST_SUPP_FEATS_NOTIF_EVT_CODE, hci_rem_host_supp_feats_notif_evt);
                memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                memcpy(&evt->ext_feats.feats[0], &param->ext_feats.feats[0], FEATS_LEN);
                hci_send_2_host(evt);
            }
        }

        if ((param->page == FEATURE_PAGE_1) && (!lc_conn_seq_done(dest_id)))
        {
            lc_ext_feat(dest_id, idx, FEATURE_PAGE_2);
        }
        else
        {
            ke_state_set(dest_id, LC_CONNECTED);

            /*
             * If both devices support both the Secure Connections (Controller Support) and Secure Connections (Host Support)
             * features, the secure connections is active
             */
            lc_env_ptr->sp.sec_con = (lm_get_sec_con_host_supp() && lm_get_sp_en()
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_SSP_BIT_POS)
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_1], FEAT_SSP_HOST_BIT_POS)
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SEC_CON_CTRL_BIT_POS)
                                      && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_1], FEAT_SEC_CON_HOST_BIT_POS));
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

#if PCA_SUPPORT
/// Handles the LMP packet LMP_clk_adj
LMP_MSG_HANDLER(clk_adj)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    uint8_t  clk_adj_id = param->clk_adj_id;
    uint32_t clk_adj_instant = param->clk_adj_instant;
    int16_t clk_adj_us = param->clk_adj_us;
    uint8_t  clk_adj_slots = param->clk_adj_slots;
    uint8_t  clk_adj_mode = param->clk_adj_mode;
    uint32_t clk_adj_clk = param->clk_adj_clk;

    /* A slave shall discard and not acknowledge an LMP_clk_adj PDU with a value of clk_adj_id
     that is the same as that in the most recently received LMP_clk_adj PDU, if any, and any
      LMP_clk_adj PDU with any aparameters outside the valid range */
    if ((!lc_env_ptr->pca.activated) || (lc_env_ptr->pca.clk_adj_id != clk_adj_id))
    {
        if (!((clk_adj_instant & 1) || (clk_adj_mode > 1) || (clk_adj_us <= -SLOT_SIZE) || (clk_adj_us >= SLOT_SIZE)))
        {
            lc_env_ptr->pca.activated = true;
            lc_env_ptr->pca.clk_adj_id = clk_adj_id;

            if (clk_adj_mode == 1)
            {
                clk_adj_instant = (clk_adj_clk << 1) + 0x10;
            }

            ld_acl_clk_adj_set(idx, clk_adj_slots, clk_adj_us, clk_adj_instant);

#if RW_BT_MWS_COEX
            ld_pca_reporting_enable(false);
#endif // RW_BT_MWS_COEX

            // acknowledge receipt of the clk_adj directive
            lc_send_pdu_clk_adj_ack(idx, clk_adj_id, GETB(param->opcode, LMP_TR_ID));
        }
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_clk_adj_ack
LMP_MSG_HANDLER(clk_adj_ack)
{
    lb_clk_adj_ack(param->clk_adj_id, KE_IDX_GET(dest_id));

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_clk_adj_req
LMP_MSG_HANDLER(clk_adj_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        uint8_t status = CO_ERROR_INVALID_LMP_PARAM;

        // The offset between the old and new slot boundaries. Valid range is -624 to +624.
        if ((-SLOT_SIZE < param->clk_adj_us) && (param->clk_adj_us < SLOT_SIZE))
        {
            /* Slave request for coarse adjustment of piconet clock, and Master coarse adjustment of piconet clock, are considered two distinct transactions,
               and the latter should therefore always use MASTER_ROLE for its transaction ID */
            status = lb_master_clk_adj_req_handler(param->clk_adj_us, param->clk_adj_slots, param->clk_adj_period, MASTER_ROLE /*GETB(param->opcode, LMP_TR_ID)*/);
        }

        lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

        if (CO_ERROR_NO_ERROR == status)
        {
            lc_send_pdu_acc_ext4(idx, LMP_CLK_ADJ_REQ_EXTOPCODE, lc_env_ptr->link.RxTrIdServer);
        }
        else
        {
            lc_send_pdu_not_acc_ext4(idx, LMP_CLK_ADJ_REQ_EXTOPCODE, status, lc_env_ptr->link.RxTrIdServer);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
#endif // PCA_SUPPORT

/// Handles the LMP packet LMP_pkt_type_tbl_req
LMP_MSG_HANDLER(pkt_type_tbl_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.ptt_tmp      = param->pkt_type_tbl;
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    lc_env_ptr->link.RxPktTypeID  = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        if (lc_env_ptr->link.SetupComplete)
        {
            lc_env_ptr->link.saved_state = LC_CONNECTED;
            lc_ptt(dest_id);
        }
        else
        {
            // send LMP_NotAcceptedExt
            lc_send_pdu_not_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_WAIT_PKT_TBL_TYP_ACC_CFM:
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            // Save the master request to respond later
            return (KE_MSG_SAVED);
        }
        else
        {
            // send LMP_NotAcceptedExt(LMP_PKT_TYPE_TBL_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_WAIT_PUB_KEY_HEADER_RSP_PEER:
        // Workaround interop deadlock
        // Check if the request asks for changing the packet type table
        if (lc_env_ptr->link.ptt_tmp != lc_env_ptr->link.CurPacketTypeTable)
        {
            lc_env_ptr->link.saved_state = LC_WAIT_PUB_KEY_HEADER_RSP_PEER;
            lc_ptt(dest_id);
        }
        else
        {
            // LMP_AcceptedExt(LMP_PKT_TYPE_TBL_REQ_EXTOPCODE)
            lc_send_pdu_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        // Save the request to respond later
        return (KE_MSG_SAVED);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_esco_link_req
LMP_MSG_HANDLER(esco_link_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdClient  = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    case LC_SCO_NEGO_ONGOING:
    {
#if (MAX_NB_SYNC > 0)
        struct lc_sco_air_params_tag peer_params;
        peer_params.tr_id          = GETB(param->opcode, LMP_TR_ID);
        peer_params.esco_hdl       = param->esco_hdl;
        peer_params.esco_lt_addr   = param->esco_lt_addr;
        peer_params.flags          = param->flags;
        peer_params.d_esco         = param->d_esco;
        peer_params.t_esco         = param->t_esco;
        peer_params.w_esco         = param->w_esco;
        peer_params.m2s_pkt_type   = param->m2s_pkt_type;
        peer_params.s2m_pkt_type   = param->s2m_pkt_type;
        peer_params.m2s_pkt_len    = param->m2s_pkt_len;
        peer_params.s2m_pkt_len    = param->s2m_pkt_len;
        peer_params.air_mode       = param->air_mode;
        peer_params.nego_state     = param->nego_state;

        lc_sco_peer_request(dest_id, ESCO_TYPE, &peer_params);
#else // (MAX_NB_SYNC > 0)
        // send LMP_NotAcceptedExt(LMP_ESCO_LINK_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_ESCO_LINK_REQ_EXTOPCODE, CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED, GETB(param->opcode, LMP_TR_ID));
#endif // (MAX_NB_SYNC > 0)
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_rmv_esco_link_req
LMP_MSG_HANDLER(rmv_esco_link_req)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
#if (MAX_NB_SYNC > 0)
        lc_sco_peer_request_disc(dest_id, ESCO_TYPE, param->esco_hdl, param->reason);
#else // (MAX_NB_SYNC > 0)
        // send LMP_NotAcceptedExt(LMP_RMV_ESCO_LINK_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(KE_IDX_GET(dest_id), LMP_RMV_ESCO_LINK_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, GETB(param->opcode, LMP_TR_ID));
#endif // (MAX_NB_SYNC > 0)
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ch_class_req
LMP_MSG_HANDLER(ch_class_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            switch (param->rep_mode)
            {
            case AFH_REPORTING_ENABLED:
            {
                uint32_t reporting_interval = param->max_intv - 2;

                // Check if the interval parameters are correct
                if ((param->min_intv > param->max_intv) || (param->min_intv < AFH_REPORT_INTERVAL_MIN) || (param->max_intv > AFH_REPORT_INTERVAL_MAX))
                {
                    // send LMP_NotAcceptedExt
                    lc_send_pdu_not_acc_ext4(idx, LMP_CH_CLASS_REQ_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, GETB(param->opcode, LMP_TR_ID));

                    break;
                }

                // Check if the reporting configuration has changed
                if ((lc_env_ptr->afh.reporting_interval == reporting_interval) && lc_env_ptr->afh.reporting_en)
                {
                    //if channel classification already started and the param are the same do nothing.
                    break;
                }

                // Start reporting timer
                ke_timer_set(LC_AFH_REPORT_TO, dest_id, 10 * co_slot_to_duration(reporting_interval));

                // Save parameters
                lc_env_ptr->afh.reporting_interval = reporting_interval;
                lc_env_ptr->afh.reporting_en = true;
            }
            break;
            case AFH_REPORTING_DISABLED:
            {
                // Clear reporting timer
                ke_timer_clear(LC_AFH_REPORT_TO, dest_id);

                lc_env_ptr->afh.reporting_en = false;
            }
            break;
            default:
            {
                // send LMP_NotAcceptedExt
                lc_send_pdu_not_acc_ext4(idx, LMP_CH_CLASS_REQ_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, GETB(param->opcode, LMP_TR_ID));
            }
            break;
            }
        }
        else
        {
            // send LMP_NotAcceptedExt
            lc_send_pdu_not_acc_ext4(idx, LMP_CH_CLASS_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, GETB(param->opcode, LMP_TR_ID));
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ch_class
LMP_MSG_HANDLER(ch_class)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    DBG_SWDIAG(AFH, CLASS_RX, 1);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        // Store channel classification from slave
        memcpy(&lc_env_ptr->afh.ch_class.map[0], &param->ch_class.map[0], BT_CH_MAP_LEN);

        // Indicate to LM
        memcpy(&lc_env_ptr->afh.peer_ch_class.map[0], &param->ch_class.map[0], BT_CH_MAP_LEN);
    }
    break;
    }

    DBG_SWDIAG(AFH, CLASS_RX, 0);

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ssr_req
LMP_MSG_HANDLER(ssr_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer         = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SNIFF_SUB_RSP:
    {
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            lc_sniff_ssr_peer_request(dest_id, param->max_subrate, param->min_to, param->instant);
        }
        else
        {
            // send LMP_NotAcceptedExt(LMP_SNIFF_SUBR_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_SSR_REQ_EXTOPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
        }
    }
    break;
    case LC_WAIT_SSR_INSTANT:
    {
        // send LMP_NotAcceptedExt(LMP_SNIFF_SUBR_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_SSR_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
    }
    break;
    case LC_CONNECTED:
    {
        if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
        {
            lc_sniff_ssr_peer_request(dest_id, param->max_subrate, param->min_to, param->instant);
        }
        else
        {
            // send LMP_NotAcceptedExt(LMP_SNIFF_SUBR_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_SSR_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ssr_res
LMP_MSG_HANDLER(ssr_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SNIFF_SUB_RSP:
    {
        lc_env_ptr->link.RxTrIdClient = GETB(param->opcode, LMP_TR_ID);

        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        lc_sniff_ssr_peer_response(dest_id, param->max_subrate, param->min_to, param->instant);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_pause_enc_aes_req
LMP_MSG_HANDLER(pause_enc_aes_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Store random number
    memcpy(&lc_env_ptr->sp.LocRandN, &param->rand.ltk[0], KEY_LEN);

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_EPR_RSP:
    {
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        // epr initiator
        lc_initiator_epr(dest_id);
    }
    break;

    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT:
    case LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT:
    {
        // send LMP_NotAcceptedExt(LMP_PAUSE_ENC_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_PAUSE_ENC_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
    }
    break;

    case LC_WAIT_EPR_ENC_PAUSE_REQ_MST_RSP:
    {
        // Send LMP_PauseEncryptionReq
        lc_stop_enc(dest_id);
    }
    break;

    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP:
    {
        lc_send_pdu_paus_enc_req(idx, lc_env_ptr->link.RxTrIdServer);
        lc_start_lmp_to(dest_id);
    }
    break;

    case LC_CONNECTED:
    {
        if (lc_env_ptr->enc.EncEnable == ENCRYPTION_OFF)
            lc_send_pdu_not_acc(idx, LMP_PAUSE_ENC_AES_REQ_OPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        else
            lc_epr_resp(dest_id);
    }
    break;

    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_pause_enc_req
LMP_MSG_HANDLER(pause_enc_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_EPR_RSP:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        // epr initiator
        lc_initiator_epr(dest_id);
        break;

    case LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT:
        if ((lc_env_ptr->link.Initiator ^ (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->link.Role)))
        {
            // clear the timer
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            // stop the encryption
            lc_stop_enc(dest_id);
        }
        else
        {
            // send LMP_NotAcceptedExt(LMP_PAUSE_ENC_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_PAUSE_ENC_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        }
        break;

    case LC_WAIT_EPR_ENC_PAUSE_REQ_MST_RSP:
        // Send LMP_PauseEncryptionReq
        lc_stop_enc(dest_id);
        break;

    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT:
    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP:
        lc_send_pdu_paus_enc_req(idx, lc_env_ptr->link.RxTrIdServer);
        lc_start_lmp_to(dest_id);
        break;

    case LC_CONNECTED:
        if (lc_env_ptr->enc.EncEnable == ENCRYPTION_OFF)
            lc_send_pdu_not_acc_ext4(idx, LMP_PAUSE_ENC_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        else
            lc_epr_resp(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_resume_enc_req
LMP_MSG_HANDLER(resume_enc_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_EPR_PEER_REQ_MST:
    case LC_WAIT_EPR_ENC_RESUME_REQ_MST_RSP:
        if ((lc_env_ptr->link.Initiator ^ (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->link.Role)))
        {
            ke_timer_clear(LC_LMP_RSP_TO, dest_id);
            lc_start_enc(dest_id);
        }
        else
        {
            // send LMP_NotAcceptedExt(LMP_RESUME_ENC_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_RESUME_ENC_REQ_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        // send LMP_NotAcceptedExt(LMP_RESUME_ENC_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_RESUME_ENC_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->link.RxTrIdServer);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_io_cap_req
LMP_MSG_HANDLER(io_cap_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_IO_CAP_INIT_CFM:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

            // simple pairing failure
            lc_sp_fail(dest_id);
        }
        else
        {
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
                lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->sp.SpTId);
            }
            else
            {
                // save the values from the message
                lc_env_ptr->link.RxTrIdServer     = GETB(param->opcode, LMP_TR_ID);
                lc_env_ptr->sp.IOCap_rem.aut_req  = param->auth_req;
                lc_env_ptr->sp.IOCap_rem.io_cap   = param->io_cap;
                lc_env_ptr->sp.IOCap_rem.oob_data_present = param->oob_auth_data;

                lc_env_ptr->sp.SPInitiator = false;
                lc_env_ptr->sp.SpTId = lc_env_ptr->link.RxTrIdServer;
                {
                    // allocate the status event message
                    struct hci_io_cap_rsp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_IO_CAP_RSP_EVT_CODE, hci_io_cap_rsp_evt);

                    // gets connection handle
                    evt->auth_req      = lc_env_ptr->sp.IOCap_rem.aut_req;
                    evt->io_capa       = lc_env_ptr->sp.IOCap_rem.io_cap;
                    evt->oob_data_pres = lc_env_ptr->sp.IOCap_rem.oob_data_present ;
                    memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                    // send the event
                    hci_send_2_host(evt);
                }
                //  send LMP_IOCapabilityRes
                lc_send_pdu_io_cap_res(idx);

                lc_start_lmp_to(dest_id);
                ke_state_set(dest_id, LC_WAIT_PUB_KEY_HEADER_RSP_PEER);
            }
        }
        break;

    case LC_WAIT_HL_IO_CAP_INIT:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->sp.SpTId)
        {
            // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

            // simple pairing failed
            lc_sp_fail(dest_id);
        }
        else
        {
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
                lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
            }
            else
            {
                // save the values from the message
                lc_env_ptr->link.RxTrIdServer     = GETB(param->opcode, LMP_TR_ID);
                lc_env_ptr->sp.IOCap_rem.aut_req  = param->auth_req;
                lc_env_ptr->sp.IOCap_rem.io_cap   = param->io_cap;
                lc_env_ptr->sp.IOCap_rem.oob_data_present = param->oob_auth_data;

                lc_env_ptr->sp.SPInitiator = false;
                lc_env_ptr->sp.SpTId = lc_env_ptr->link.RxTrIdServer;
                {
                    // allocate the status event message
                    struct hci_io_cap_rsp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_IO_CAP_RSP_EVT_CODE, hci_io_cap_rsp_evt);

                    // gets connection handle
                    evt->auth_req      = lc_env_ptr->sp.IOCap_rem.aut_req;
                    evt->io_capa       = lc_env_ptr->sp.IOCap_rem.io_cap;
                    evt->oob_data_pres = lc_env_ptr->sp.IOCap_rem.oob_data_present ;
                    memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                    // send the event
                    hci_send_2_host(evt);
                }
                // start timer
                lc_start_lmp_to(dest_id);

                // set state to IO RSP wait
                ke_state_set(dest_id, LC_WAIT_HL_IO_CAP_RSP);
            }
        }
        break;

    case LC_CONNECTED:
        if (lc_env_ptr->link.RxTrIdServer == lc_env_ptr->link.Role)
        {
            // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
            lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->sp.SpTId);
        }
        else
        {
            if (lc_env_ptr->sp.p_data != NULL)
            {
                // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
                lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_LMP_PDU_NOT_ALLOWED, lc_env_ptr->sp.SpTId);
            }
            else
            {
                lc_env_ptr->sp.SpTId = lc_env_ptr->link.RxTrIdServer;
                if (lm_get_sp_en())
                {
                    // save the values from the message
                    lc_env_ptr->link.RxTrIdServer     = GETB(param->opcode, LMP_TR_ID);
                    lc_env_ptr->sp.IOCap_rem.aut_req  = param->auth_req;
                    lc_env_ptr->sp.IOCap_rem.io_cap   = param->io_cap;
                    lc_env_ptr->sp.IOCap_rem.oob_data_present = param->oob_auth_data;

                    lc_resp_pair(dest_id, idx);
                }
                else
                {
                    // send LMP_NotAcceptedExt(LMP_IOCAP_REQ_EXTOPCODE)
                    lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_SP_NOT_SUPPORTED_HOST, lc_env_ptr->sp.SpTId);
                }
            }
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        // IOCAP has been already exchanged and local initiator already accepted
        if (lc_env_ptr->sp.p_data != NULL)
        {

            // This is an unexpected behavior from peer
            // Reject PDU with Transaction collision since other error code is considered as disconnection reason for peer.
            lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_REQ_EXTOPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->sp.SpTId);
        }
        // otherwise handle it as soon as possible
        else
        {
            return (KE_MSG_SAVED);
        }
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_io_cap_res
LMP_MSG_HANDLER(io_cap_res)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_IO_CAP_INIT_CFM:
        if (lc_env_ptr->link.RxTrIdServer != lc_env_ptr->sp.SpTId)
        {
            lc_send_pdu_not_acc_ext4(idx, LMP_IO_CAP_RES_EXTOPCODE, CO_ERROR_INVALID_LMP_PARAM, lc_env_ptr->link.RxTrIdServer);

            // authentication failure
            lc_sp_fail(dest_id);
        }
        else
        {
            // allocate the status event message
            struct hci_io_cap_rsp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_IO_CAP_RSP_EVT_CODE, hci_io_cap_rsp_evt);
            // save the values from the message

            lc_env_ptr->sp.IOCap_rem.aut_req  = param->auth_req;
            lc_env_ptr->sp.IOCap_rem.io_cap   = param->io_cap;
            lc_env_ptr->sp.IOCap_rem.oob_data_present = param->oob_auth_data;

            // fill the parameters
            evt->auth_req      = lc_env_ptr->sp.IOCap_rem.aut_req;
            evt->io_capa       = lc_env_ptr->sp.IOCap_rem.io_cap;
            evt->oob_data_pres = lc_env_ptr->sp.IOCap_rem.oob_data_present;
            memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);

            // send the event
            hci_send_2_host(evt);

            //********************************************
            //END OF IO CAPABILITY EXCHANGE
            //********************************************

            //********************************************
            //START OF THE PUBLIC KEY EXCHANGE
            //********************************************
            lc_sp_prepare_public_key_exchange(dest_id, lc_env_ptr);
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_num_comparison_fail
LMP_MSG_HANDLER(num_comparison_fail)
{
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_NUM_COMP_USER_CONF_RSP:
    case LC_WAIT_DHKEY_RSP:
    case LC_WAIT_DHKEY_COMPUTING:
    case LC_WAIT_DHKEY_RSP_FAIL:
        // clear the timer
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        // simple pairing failure
        lc_sp_fail(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_passkey_fail
LMP_MSG_HANDLER(passkey_fail)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PASSKEY_COMM_RSP_PEER:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // simple pairing failure
        lc_sp_fail(dest_id);
        break;
    case LC_WAIT_AUTH_STG1_PK_RANDN:
    case LC_WAIT_DHKEY_COMPUTING:
        return (KE_MSG_SAVED);
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_oob_fail
LMP_MSG_HANDLER(oob_fail)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_OOB_RANDN_RSP_PEER:
    case LC_WAIT_HL_OOB_DATA:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);

        // simple pairing failure
        lc_sp_fail(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), 0);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_keypress_notif
LMP_MSG_HANDLER(keypress_notif)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.RxTrIdServer = GETB(param->opcode, LMP_TR_ID);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PASSKEY_HL_RPLY:
    case LC_WAIT_PASSKEY_COMM_RSP_PEER:
    case LC_WAIT_PASSKEY_COMM_INIT_PEER:
    {
        // allocate the status event message
        struct hci_keypress_notif_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_KEYPRESS_NOTIF_EVT_CODE, hci_keypress_notif_evt);

        // fill the parameters
        evt->type = param->type;
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);

        // send the event
        hci_send_2_host(evt);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_WARN(0, ke_state_get(dest_id), idx);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ping_req
LMP_MSG_HANDLER(ping_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        // Send LMP_ping_res
        struct lmp_ping_res pdu;
        pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, !lc_env_ptr->link.Role);
        pdu.ext_opcode = LMP_PING_RES_EXTOPCODE;
        lc_send_lmp(idx, &pdu);
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_ping_res
LMP_MSG_HANDLER(ping_res)
{
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_PING_RES:
    {
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        ke_state_set(dest_id, LC_CONNECTED);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        ASSERT_WARN(0, ke_state_get(dest_id), KE_IDX_GET(dest_id));
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sam_set_type0
LMP_MSG_HANDLER(sam_set_type0)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        uint8_t status = CO_ERROR_NO_ERROR;

        if ((param->update_mode == SAM_UPDATE_INVALIDATE_MAPS) && (lc_env_ptr->sam_info.rem_idx != SAM_DISABLED)
                && (lc_env_ptr->sam_info.rem_pattern[lc_env_ptr->sam_info.rem_idx].n_ex_sm > 0))
        {
            // Update Mode shall not be set to 0 if the active SAM slot map contains type 0 submap (LMP 4.1.15.1)
            status = CO_ERROR_INVALID_LMP_PARAM;

            lc_send_pdu_not_acc_ext4(idx, LMP_SAM_SET_TYPE0_EXTOPCODE, status, !lc_env_ptr->link.Role);
        }
        else
        {
            // Save the new submap for potential immediate use
            memcpy(&lc_env_ptr->sam_info.rem_submap.map[0], &param->submap.map[0], SAM_TYPE0_SUBMAP_LEN);

            lc_env_ptr->sam_info.rem_submap0_av = true;

            if (SAM_UPDATE_INVALIDATE_MAPS == param->update_mode)
            {
                for (int i = 0; i < SAM_INDEX_MAX; i++)
                {
                    // Disable SAM patterns which use old Submap0
                    if ((lc_env_ptr->sam_info.rem_pattern_av[i]) && (lc_env_ptr->sam_info.rem_pattern[i].n_ex_sm > 0))
                    {
                        memset(lc_env_ptr->sam_info.rem_pattern, 0, sizeof(struct lc_sam_pattern));
                        lc_env_ptr->sam_info.rem_pattern_av[i] = false;
                    }
                }
            }
            else if (SAM_DISABLED != lc_env_ptr->sam_info.rem_idx) // SAM_UPDATE_IMMEDIATE, SAM_UPDATE_AT_SUBINTERVAL
            {
                struct lc_sam_pattern *sam_pattern = &lc_env_ptr->sam_info.rem_pattern[lc_env_ptr->sam_info.rem_idx];

                uint8_t sam_map[RW_PEER_SAM_MAP_MAX_LEN];

                uint8_t t_update = (SAM_UPDATE_IMMEDIATE == param->update_mode) ? 0 : sam_pattern->t_sam_sm;

                // The active SAM Pattern uses this Submap  - requires updating
                if (sam_pattern->n_ex_sm > 0)
                {
                    uint16_t t_sam = sam_pattern->n_sam_sm * sam_pattern->t_sam_sm;

                    uint16_t n_tx_slots = sam_pattern->n_tx_slots + sam_pattern->n_ex_sm * lc_sam_rem_submap_tx_slots_get(idx, sam_pattern->t_sam_sm);
                    uint16_t n_rx_slots = sam_pattern->n_rx_slots + sam_pattern->n_ex_sm * lc_sam_rem_submap_rx_slots_get(idx, sam_pattern->t_sam_sm);

                    lc_env_ptr->sam_info.rem_tx_av = ((n_tx_slots << 8) - n_tx_slots) / t_sam;
                    lc_env_ptr->sam_info.rem_rx_av = ((n_rx_slots << 8) - n_rx_slots) / t_sam;

                    // Re-build a complete SAM map
                    lc_build_sam_map(lc_env_ptr, lc_env_ptr->sam_info.rem_idx, &sam_map[0]);

                    // Instruct LD to apply the SAM pattern
                    ld_acl_remote_sam_map_refresh(idx, &sam_map[0], t_update);
                }
            }

            lc_send_pdu_acc_ext4(idx, LMP_SAM_SET_TYPE0_EXTOPCODE, !lc_env_ptr->link.Role);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sam_define_map
LMP_MSG_HANDLER(sam_define_map)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        uint8_t status = CO_ERROR_NO_ERROR;

        if ((param->index > SAM_INDEX_MAX) || (param->index == lc_env_ptr->sam_info.rem_idx))
        {
            // The sam index shall not be 0xFF, out of range, or that of the currently selected map
            status = CO_ERROR_INVALID_LMP_PARAM;
        }
        else if (0 == param->n_sam_sm)
        {
            // If Nsam-sm is 0 the map shall be deleted.
            memset(lc_env_ptr->sam_info.rem_pattern, 0, sizeof(struct lc_sam_pattern));
            lc_env_ptr->sam_info.rem_pattern_av[param->index] = false;
        }
        else
        {
            struct lc_sam_pattern *sam_pattern = &lc_env_ptr->sam_info.rem_pattern[param->index];

            uint16_t n_tx_slots = 0;
            uint16_t n_rx_slots = 0;
            uint8_t n_ex_sm = 0;

            uint8_t byte_idx;
            uint8_t bit_pos;

            for (int i = 0; i < param->n_sam_sm; i++)
            {
                byte_idx = i >> 2; // 4 submaps per byte
                bit_pos = (i & 0x3) << 1; // 2 bit fields

                switch ((param->submaps.map[byte_idx] >> bit_pos) & 0x3)
                {
                case SAM_SLOTS_SUBMAPPED:
                    n_ex_sm++;
                    break;
                case SAM_SLOTS_AVAILABLE:
                    n_rx_slots += param->t_sam_sm;
                    n_tx_slots += param->t_sam_sm;
                    break;
                case SAM_SLOTS_UNAVAILABLE:
                    break;
                default:
                    status = CO_ERROR_INVALID_LMP_PARAM;
                    break;
                }
            }

            if (n_ex_sm && !lc_env_ptr->sam_info.rem_submap0_av)
            {
                // The slot map may only contain a type0 submap if this is available
                status = CO_ERROR_TYPE0_SUBMAP_NOT_DEFINED;
            }
            else if (param->n_sam_sm * param->t_sam_sm > RW_MAX_PEER_SAM_MAP_SLOTS)
            {
                // HW restriction - cannot exceed RW_MAX_PEER_SAM_MAP_SLOTS slots size for peer pattern size
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
            }

            if (status == CO_ERROR_NO_ERROR)
            {
                // Store the new Map. If the sam index has been previously defined, then it is relplaced.
                memcpy(&sam_pattern->submaps.map[0], &param->submaps.map[0], SAM_SUBMAPS_LEN);

                sam_pattern->t_sam_sm = param->t_sam_sm;
                sam_pattern->n_sam_sm = param->n_sam_sm;

                sam_pattern->n_tx_slots = n_tx_slots;
                sam_pattern->n_rx_slots = n_rx_slots;
                sam_pattern->n_ex_sm = n_ex_sm;
            }

            lc_env_ptr->sam_info.rem_pattern_av[param->index] = true;
        }

        if (status == CO_ERROR_NO_ERROR)
        {
            lc_send_pdu_acc_ext4(idx, LMP_SAM_DEFINE_MAP_EXTOPCODE, !lc_env_ptr->link.Role);
        }
        else
        {
            lc_send_pdu_not_acc_ext4(idx, LMP_SAM_DEFINE_MAP_EXTOPCODE, status, !lc_env_ptr->link.Role);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/// Handles the LMP packet LMP_sam_switch
LMP_MSG_HANDLER(sam_switch)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        uint8_t status = CO_ERROR_NO_ERROR;

        uint16_t t_sam = 0;

        uint16_t rem_tx_av = 0xFF;
        uint16_t rem_rx_av = 0xFF;

        if (((param->index < SAM_INDEX_MAX) || (SAM_DISABLED == param->index)) && ((param->instant & 1) == 0))
        {
            if (param->index < SAM_INDEX_MAX)
            {
                struct lc_sam_pattern *sam_pattern = &lc_env_ptr->sam_info.rem_pattern[param->index];

                t_sam = sam_pattern->t_sam_sm * sam_pattern->n_sam_sm;

                ASSERT_ERR(t_sam <= RW_MAX_PEER_SAM_MAP_SLOTS);

                // SAM pattern must be defined
                if (!lc_env_ptr->sam_info.rem_pattern_av[param->index])
                {
                    status = CO_ERROR_INVALID_LMP_PARAM;
                }
                // d_sam shall be less than t_sam interval, and even value. Only bit1 of timing control valid.
                else if ((param->d_sam < t_sam) && !(param->d_sam & 0x1) && !(param->flags & ~INIT2_FLAG))
                {
                    // Determine remote SAM TX/RX availability
                    uint16_t n_slots = sam_pattern->n_tx_slots + sam_pattern->n_ex_sm * lc_sam_rem_submap_tx_slots_get(idx, sam_pattern->t_sam_sm);
                    rem_tx_av = ((n_slots << 8) - n_slots) / t_sam;
                    n_slots = sam_pattern->n_rx_slots + sam_pattern->n_ex_sm * lc_sam_rem_submap_rx_slots_get(idx, sam_pattern->t_sam_sm);
                    rem_rx_av = ((n_slots << 8) - n_slots) / t_sam;

                    lc_env_ptr->sam_info.rem_t_sam_av = t_sam;
                }
                else
                {
                    status = CO_ERROR_INVALID_LMP_PARAM;
                }
            }
            else
            {
                lc_env_ptr->sam_info.rem_t_sam_av = 0;
            }
        }
        else
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
        }

        if (CO_ERROR_NO_ERROR == status)
        {
            lc_env_ptr->sam_info.rem_idx = param->index;
            lc_env_ptr->sam_info.rem_tx_av = rem_tx_av;
            lc_env_ptr->sam_info.rem_rx_av = rem_rx_av;

            if (SAM_DISABLED != param->index)
            {
                uint8_t sam_map[RW_PEER_SAM_MAP_MAX_LEN];

                // Build a complete SAM map
                lc_build_sam_map(lc_env_ptr, param->index, &sam_map[0]);

                // Instruct LD to apply the SAM pattern
                ld_acl_remote_sam_map_set(idx, &sam_map[0], t_sam, param->instant, param->d_sam, param->flags);
            }
            else
            {
                // Instruct LD to disable remote SAM patterns
                ld_acl_remote_sam_map_set(idx, NULL, 0, param->instant, 0, 0);
            }
        }

        if (status == CO_ERROR_NO_ERROR)
        {
            lc_send_pdu_acc_ext4(idx, LMP_SAM_SWITCH_EXTOPCODE, !lc_env_ptr->link.Role);
        }
        else
        {
            lc_send_pdu_not_acc_ext4(idx, LMP_SAM_SWITCH_EXTOPCODE, status, !lc_env_ptr->link.Role);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 * (EXTENDED) LMP PACKETS HANDLING
 ****************************************************************************************
 */

/// LMP packets descriptors
const struct lmp_desc_tag lmp_desc_tab[] =
{
    LMP(NAME_REQ, name_req, 1, 1, "BB"),
    LMP(NAME_RES, name_res, 1, 0, "BBB14B"),
    LMP(ACCEPTED, accepted, 1, 0, "BB"),
    LMP(NOT_ACCEPTED, not_accepted, 1, 0, "BBB"),
    LMP(CLK_OFF_REQ, clk_off_req, 1, 1, "B"),
    LMP(CLK_OFF_RES, clk_off_res, 1, 0, "BH"),
    LMP(DETACH, detach, 1, 0, "BB"),
    LMP(INRAND, inrand, 1, 1, "B16B"),
    LMP(COMBKEY, combkey, 1, 1, "B16B"),
    LMP(UNITKEY, unitkey, 1, 1, "B16B"),
    LMP(AURAND, aurand, 1, 1, "B16B"),
    LMP(SRES, sres, 1, 0, "B4B"),
    LMP(TEMPRAND, temprand, 1, 0, "B16B"),
    LMP(TEMPKEY, tempkey, 1, 0, "B16B"),
    LMP(ENC_MODE_REQ, enc_mode_req, 1, 1, "BB"),
    LMP(ENC_KEY_SIZE_REQ, enc_key_size_req, 1, 1, "BB"),
    LMP(START_ENC_REQ, start_enc_req, 1, 1, "B16B"),
    LMP(STOP_ENC_REQ, stop_enc_req, 1, 1, "B"),
    LMP(SWITCH_REQ, switch_req, 1, 1, "BL"),
    LMP(SNIFF_REQ, sniff_req, 0, 1, "BBHHHH"),
    LMP(UNSNIFF_REQ, unsniff_req, 0, 1, "B"),
    LMP(INCR_PWR_REQ, incr_pwr_req, 1, 1, "BB"),
    LMP(DECR_PWR_REQ, decr_pwr_req, 1, 1, "BB"),
    LMP(MAX_PWR, max_pwr, 1, 0, "B"),
    LMP(MIN_PWR, min_pwr, 1, 0, "B"),
    LMP(AUTO_RATE, auto_rate, 0, 0, "B"),
    LMP(PREF_RATE, pref_rate, 0, 0, "BB"),
    LMP(VER_REQ, ver_req, 1, 1, "BBHH"),
    LMP(VER_RES, ver_res, 1, 0, "BBHH"),
    LMP(FEATS_REQ, feats_req, 1, 1, "B8B"),
    LMP(FEATS_RES, feats_res, 1, 0, "B8B"),
    LMP(QOS, qos, 0, 0, "BHB"),
    LMP(QOS_REQ, qos_req, 0, 1, "BHB"),
    LMP(SCO_LINK_REQ, sco_link_req, 0, 1, "BBBBBBB"),
    LMP(RMV_SCO_LINK_REQ, rmv_sco_link_req, 0, 1, "BBB"),
    LMP(MAX_SLOT, max_slot, 0, 0, "BB"),
    LMP(MAX_SLOT_REQ, max_slot_req, 0, 1, "BB"),
    LMP(TIMING_ACCU_REQ, timing_accu_req, 1, 1, "B"),
    LMP(TIMING_ACCU_RES, timing_accu_res, 1, 0, "BBB"),
    LMP(SETUP_CMP, setup_cmp, 1, 0, "B"),
    LMP(USE_SEMI_PERM_KEY, use_semi_perm_key, 1, 0, "B"),
    LMP(HOST_CON_REQ, host_con_req, 1, 1, "B"),
    LMP(SLOT_OFF, slot_off, 1, 0, "BH6B"),
    LMP(PAGE_MODE_REQ, page_mode_req, 1, 1, "BBB"),
    LMP(PAGE_SCAN_MODE_REQ, page_scan_mode_req, 1, 1, "BBB"),
    LMP(SUPV_TO, supv_to, 1, 0, "BH"),
    LMP(TEST_ACTIVATE, test_activate, 0, 1, "B"),
    LMP(TEST_CTRL, test_ctrl, 0, 1, "BBBBBBBBH"),
    LMP(ENC_KEY_SIZE_MASK_REQ, enc_key_size_mask_req, 1, 1, "B"),
    LMP(ENC_KEY_SIZE_MASK_RES, enc_key_size_mask_res, 1, 0, "BH"),
    LMP(SET_AFH, set_afh, 1, 1, "BLB10B"),
    LMP(ENCAPS_HDR, encaps_hdr, 1, 1, "BBBB"),
    LMP(ENCAPS_PAYL, encaps_payl, 1, 1, "B16B"),
    LMP(SP_CFM, sp_cfm, 1, 1, "B16B"),
    LMP(SP_NB, sp_nb, 1, 1, "B16B"),
    LMP(DHKEY_CHK, dhkey_chk, 1, 1, "B16B"),
    LMP(PAUSE_ENC_AES_REQ, pause_enc_aes_req, 1, 1, "B16B"),
};


/// LMP packets descriptors
const struct lmp_desc_tag lmp_ext_desc_tab[] =
{
    LMP_EXT(ACCEPTED_EXT, accepted_ext, 1, 0, "BBBB"),
    LMP_EXT(NOT_ACCEPTED_EXT, not_accepted_ext, 1, 0, "BBBBB"),
    LMP_EXT(FEATS_REQ_EXT, feats_req_ext, 1, 1, "BBBB8B"),
    LMP_EXT(FEATS_RES_EXT, feats_res_ext, 1, 0, "BBBB8B"),
#if PCA_SUPPORT
    LMP_EXT(CLK_ADJ, clk_adj, 0, 0, "BBBLHBBL"),
    LMP_EXT(CLK_ADJ_ACK, clk_adj_ack, 0, 0, "BBB"),
    LMP_EXT(CLK_ADJ_REQ, clk_adj_req, 0, 1, "BBHBB"),
#endif // PCA_SUPPORT
    LMP_EXT(PKT_TYPE_TBL_REQ, pkt_type_tbl_req, 1, 1, "BBB"),
    LMP_EXT(ESCO_LINK_REQ, esco_link_req, 0, 1, "BBBBBBBBBBHHBB"),
    LMP_EXT(RMV_ESCO_LINK_REQ, rmv_esco_link_req, 0, 1, "BBBB"),
    LMP_EXT(CH_CLASS_REQ, ch_class_req, 1, 1, "BBBHH"),
    LMP_EXT(CH_CLASS, ch_class, 1, 0, "BB10B"),
    LMP_EXT(SSR_REQ, ssr_req, 0, 1, "BBBHL"),
    LMP_EXT(SSR_RES, ssr_res, 0, 0, "BBBHL"),
    LMP_EXT(PAUSE_ENC_REQ, pause_enc_req, 1, 1, "BB"),
    LMP_EXT(RESUME_ENC_REQ, resume_enc_req, 1, 1, "BB"),
    LMP_EXT(IO_CAP_REQ, io_cap_req, 1, 1, "BBBBB"),
    LMP_EXT(IO_CAP_RES, io_cap_res, 1, 1, "BBBBB"),
    LMP_EXT(NUM_COMPARISON_FAIL, num_comparison_fail, 1, 0, "BB"),
    LMP_EXT(PASSKEY_FAIL, passkey_fail, 1, 0, "BB"),
    LMP_EXT(OOB_FAIL, oob_fail, 1, 0, "BB"),
    LMP_EXT(KEYPRESS_NOTIF, keypress_notif, 1, 0, "BBB"),
    LMP_EXT(PWR_CTRL_REQ, pwr_ctrl_req, 1, 1, "BBB"),
    LMP_EXT(PWR_CTRL_RES, pwr_ctrl_res, 1, 0, "BBB"),
    LMP_EXT(PING_REQ, ping_req, 0, 1, "BB"),
    LMP_EXT(PING_RES, ping_res, 0, 0, "BB"),
    LMP_EXT(SAM_SET_TYPE0, sam_set_type0, 0, 1, "BBB14B"),
    LMP_EXT(SAM_DEFINE_MAP, sam_define_map, 0, 1, "BBBBB12B"),
    LMP_EXT(SAM_SWITCH, sam_switch, 0, 1, "BBBBBL"),
};

typedef struct lmp_desc_tag *lmp_desc_ptr;

typedef lmp_desc_ptr(*lmp_get_pdu_handler_desc)(uint8_t opcode);

static lmp_desc_ptr _lmp_get_pdu_handler_desc(uint8_t opcode)
{
    struct lmp_desc_tag *handler = NULL;
    return handler;
}
lmp_get_pdu_handler_desc lmp_ext_get_pdu_handler_pre = _lmp_get_pdu_handler_desc;
lmp_get_pdu_handler_desc lmp_get_pdu_handler_pre     = _lmp_get_pdu_handler_desc;


/**
 ****************************************************************************************
 * @brief Handles LMP RX
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_lmp_rx, struct lc_lmp_rx)
{
    uint8_t return_status = KE_MSG_CONSUMED;
    uint8_t opcode;
    const struct lmp_desc_tag *lmp_desc;

    // Get LMP OpCode
    opcode = GETF(*param->pdu, LMP_OPCODE);

    if (opcode == LMP_ESC4_OPCODE)
    {
        // Get LMP extended OpCode
        uint8_t ext_opcode = *(param->pdu + 1);

        // Point to LMP descriptor
        if ((lmp_desc = lmp_ext_get_pdu_handler_pre(ext_opcode)) == NULL)
            lmp_desc = &lmp_ext_desc_tab[ext_opcode];
    }
    else
    {
        // Point to LMP descriptor
        if ((lmp_desc = lmp_get_pdu_handler_pre(opcode)) == NULL)
            lmp_desc = &lmp_desc_tab[opcode];
    }


    // Check if there is a handler function
    if (lmp_desc->handler != NULL)
    {
        // Call handler
        return_status = lmp_desc->handler(param->pdu, dest_id);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    return return_status;
}

/**
 ****************************************************************************************
 * @brief Handles LMP RX indication
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_lmp_rx_ind, struct lc_lmp_rx_ind)
{
    int idx = KE_IDX_GET(dest_id);
    uint8_t status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    bool reject = true;
    uint8_t tr_id, opcode, ext_opcode = 0;

    do
    {
        struct lc_env_tag *lc_env_ptr = lc_env[idx];
        const struct lmp_desc_tag *lmp_desc = NULL;

        // Builds a new message containing unpacked PDU
        struct lc_lmp_rx *lmp_rx;

        //Trace the received LMP packet
        TRC_REQ_LMP_RX(BT_ACL_CONHDL_MIN + idx, GETF(param->lmp_len_flags, BT_RX_LMP_LEN), (uint8_t *) &param->pdu);

        // Get LMP OpCode
        opcode = GETF(*param->pdu, LMP_OPCODE);
        tr_id = GETB(*param->pdu, LMP_TR_ID);

        // Retrieve LMP descriptor
        if (opcode == LMP_ESC4_OPCODE)
        {
            // Get LMP extended OpCode
            ext_opcode = *(param->pdu + 1);

            if ((ext_opcode < (sizeof(lmp_ext_desc_tab) / sizeof(struct lmp_desc_tag))) && (lmp_ext_desc_tab[ext_opcode].handler != NULL))
            {
                // Point to LMP descriptor
                lmp_desc = &lmp_ext_desc_tab[ext_opcode];
            }
        }
        else if ((opcode < (sizeof(lmp_desc_tab) / sizeof(struct lmp_desc_tag))) && (lmp_desc_tab[opcode].handler != NULL))
        {
            // Point to LMP descriptor
            lmp_desc = &lmp_desc_tab[opcode];
        }

        // ACL-B shall ignore PDUs other than LMP_CLK_ADJ, ACL-C shall ignore LMP_CLK_ADJ => ignore ACL-B XOR LMP_CLK_ADJ
        if ((GETB(param->lmp_len_flags, BT_RX_LMP_BF) != 0) ^ (LMP_CLK_ADJ_EXTOPCODE == ext_opcode))
        {
            reject = false;
            break;
        }

        // Check if LMP desc found
        if (lmp_desc == NULL)
        {
            // The LMP test spec expects the DUT to transmit PDU LMP_not_accepted containing "Reason = 0x1A" upon reception of PDU LMP_park_req
            status = ((opcode == LMP_HOLD_REQ_OPCODE) || (opcode == LMP_PARK_REQ_OPCODE)) ? CO_ERROR_UNSUPPORTED_REMOTE_FEATURE : CO_ERROR_UNKNOWN_LMP_PDU;
            break;
        }

        ASSERT_ERR(lmp_desc->fmt != NULL);

#if RW_DEBUG
        if (lc_env_ptr->link.LMPPacketDiscard)
        {
            reject = false;
            break;
        }
#endif //RW_DEBUG

        // If connection establishment and LMP not allowed during connection establishment
        if (!lc_env_ptr->link.ConnectionCompleteSent && !GETB(lmp_desc->info, LMP_ESTAB))
        {
            // Reject only a request
            reject = GETB(lmp_desc->info, LMP_REQ);
            break;
        }

        // Check if AES encryption with secure connection, and packet is point-to-point
        if (lc_env_ptr->enc.EncEnable && lc_env_ptr->sp.sec_con && !GETB(param->lmp_len_flags, BT_RX_LMP_BF))
        {
            // Reload timers for authentication payload timeout
            ke_timer_set(LC_AUTH_PAYL_NEARLY_TO, dest_id, 10 * (lc_env_ptr->link.auth_payl_to - lc_env_ptr->link.auth_payl_to_margin));
            ke_timer_set(LC_AUTH_PAYL_REAL_TO, dest_id, 10 * lc_env_ptr->link.auth_payl_to);
        }

        // Check expected parameters length, trailing bytes ignored
        if (GETF(param->lmp_len_flags, BT_RX_LMP_LEN) < GETF(lmp_desc->info, LMP_LEN))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // Check transaction collision
        {
            bool collision = false;

            switch (opcode)
            {
            case LMP_SWITCH_REQ_OPCODE:
            case LMP_SNIFF_REQ_OPCODE:
            case LMP_UNSNIFF_REQ_OPCODE:
            case LMP_SCO_LINK_REQ_OPCODE:
            case LMP_RMV_SCO_LINK_REQ_OPCODE:
                collision = (opcode != lc_env_ptr->local_trans_details.Opcode);
                break;
            case LMP_ESC4_OPCODE:
                switch (ext_opcode)
                {
                case LMP_ESCO_LINK_REQ_EXTOPCODE:
                case LMP_RMV_ESCO_LINK_REQ_EXTOPCODE:
                case LMP_SSR_REQ_EXTOPCODE:
                case LMP_CLK_ADJ_REQ_EXTOPCODE:
                    collision = (ext_opcode != lc_env_ptr->local_trans_details.OpcodeExt);
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }

            if (lc_env_ptr->local_trans_details.InUse && (tr_id == SLAVE_ROLE) && collision)
            {
                status = CO_ERROR_DIFF_TRANSACTION_COLLISION;
                break;
            }
        }

        // Allocate Kernel message for unpacked PDU
        lmp_rx = (struct lc_lmp_rx *) ke_msg_alloc(LC_LMP_RX, dest_id, dest_id, sizeof(union lmp_pdu));

        // Unpack PDU
        {
            // Unpack the parameters
            uint16_t length = sizeof(union lmp_pdu);
            status = co_util_unpack((uint8_t *) lmp_rx->pdu, (uint8_t *) param->pdu, &length, GETF(lmp_desc->info, LMP_LEN), lmp_desc->fmt);

            ASSERT_INFO(status != CO_UTIL_PACK_WRONG_FORMAT, opcode, opcode);

            // Check unpack status
            if (status == CO_UTIL_PACK_OK)
            {
                // Re-posts the message for processing
                ke_msg_send(lmp_rx);
                reject = false;
            }
            else
            {
                ke_msg_free((struct ke_msg *) lmp_rx);
                status = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

    }
    while (0);

    // If the LMP needs to be rejected
    if (reject)
    {
        if (opcode == LMP_ESC4_OPCODE)
        {
            lc_send_pdu_not_acc_ext4(idx, ext_opcode, status, tr_id);
        }
        else
        {
            lc_send_pdu_not_acc(idx, opcode, status, tr_id);
        }
    }

    return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles LMP TX confirmation
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_lmp_tx_cfm, struct lc_lmp_tx_cfm)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t opcode = GETF(em_rd8p(param->em_buf), LMP_OPCODE);

    // Free LMP TX buffer
    bt_util_buf_lmp_tx_free(param->em_buf);

    //Trace the acknowledge LMP packet
    TRC_REQ_LMP_ACK(BT_ACL_CONHDL_MIN + idx, opcode, em_rd8p(param->em_buf + 1));

    if (opcode == LMP_SET_AFH_OPCODE)
    {
        // Confirm to LD
        ld_acl_afh_confirm(idx);
        return (KE_MSG_CONSUMED);
    }
    else if (opcode == LMP_ESC4_OPCODE)
    {
        // The extended opcode is the second byte in the payload body
        uint8_t ext_opcode = em_rd16p(param->em_buf) >> 8;
        if (ext_opcode == LMP_CH_CLASS_EXTOPCODE)
        {
            ASSERT_ERR(lc_env_ptr->afh.lmp_ch_class_pending == true);
            lc_env_ptr->afh.lmp_ch_class_pending = false;
        }
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    case LC_SCO_DISC_ONGOING:
    {
        lc_sco_baseband_ack(dest_id, opcode);
    }
    break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_SNIFF_SUB_RSP_TX_CFM:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        if (opcode == LMP_ESC4_OPCODE)
        {
            lc_sniff_ssr_baseband_ack(dest_id);
        }
        break;
    case LC_WAIT_UNSNIFF_ACC_TX_CFM:
        if (opcode == LMP_ACCEPTED_OPCODE)
        {
            lc_sniff_unsniff_peer_accept(dest_id);
        }
        break;
    case LC_WAIT_SNIFF_ACC_TX_CFM:
        if (opcode == LMP_ACCEPTED_OPCODE)
        {
            lc_sniff_baseband_ack(dest_id, opcode);
        }
        break;
    case LC_WAIT_TEST_TX_CFM:
        if (opcode == LMP_ACCEPTED_OPCODE)
        {
            struct ld_acl_test_mode_params params;

            ke_timer_clear(LC_LMP_RSP_TO, dest_id);

            params.test_scenario = lc_env_ptr->tst_mode.TestScenario;
            params.hopping_mode  = lc_env_ptr->tst_mode.HoppingMode ;
            params.tx_freq       = lc_env_ptr->tst_mode.TxFreq      ;
            params.rx_freq       = lc_env_ptr->tst_mode.RxFreq      ;
            params.power_control = lc_env_ptr->tst_mode.PowerControl;
            params.packet_type   = lc_env_ptr->tst_mode.PacketType  ;
            params.data_length   = lc_env_ptr->tst_mode.DataLength  ;

            // Apply new test parameters
            ld_acl_test_mode_set(idx, &params);

            // Restart ACL data flow
            ld_acl_flow_on(idx);

            ke_state_set(dest_id, LC_CONNECTED);

            if (lc_env_ptr->tst_mode.TestScenario == EXITTEST_MODE)
            {
                // De-activate test mode
                lc_env_ptr->tst_mode.activated = false;
            }
        }
        break;
    case LC_WAIT_DETACH_REQ_TX_CFM:
        if (opcode == LMP_DETACH_OPCODE)
        {
            uint16_t poll_int = ld_acl_t_poll_get(idx);
            poll_int *= 3;
            ke_timer_set(LC_LMP_RSP_TO, dest_id, 10 * co_slot_to_duration(poll_int));
            ke_state_set(dest_id, LC_WAIT_DISC_TO);
        }
        break;
    case LC_WAIT_DETACH_TO:
        break;
    case LC_WAIT_PWR_CTRL_TX_CFM:
        if ((opcode == LMP_INCR_PWR_REQ_OPCODE) || opcode == LMP_DECR_PWR_REQ_OPCODE)
        {
            ke_state_set(dest_id, LC_CONNECTED);
        }
        break;
    case LC_WAIT_PTT_ACC_TX_CFM:
        lc_ptt_cmp(dest_id);
        break;
    default:
        //Do nothing stay in the same state, just free the message
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles ACL RX indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_acl_rx_ind, struct lc_acl_rx_ind)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Free the buffer
        bt_util_buf_acl_rx_free(param->em_buf);
    }
    break;
    default:
        if (lm_get_loopback_mode() == REMOTE_LOOPBACK)
        {
            // Allocate a Tx buffer
            uint16_t buf_ptr = bt_util_buf_acl_tx_alloc();
            struct bt_em_acl_buf_elt *temp_tx_buf_elt = bt_util_buf_acl_tx_elt_get(buf_ptr);
            if (temp_tx_buf_elt != NULL)
            {
                uint8_t *tx_buf_ptr = (uint8_t *)(EM_BASE_ADDR + buf_ptr);
                uint8_t *rx_buf_ptr = (uint8_t *)(EM_BASE_ADDR + param->em_buf);
                uint16_t data_len = GETF(param->data_len_flags, BT_EM_ACL_DATA_LEN);

                //Copy Rx data in the Tx buffer
                temp_tx_buf_elt->data_len_flags = param->data_len_flags;
                memcpy(tx_buf_ptr, rx_buf_ptr, data_len);
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }

            // Transmit the data
            if (ld_acl_data_tx(idx, temp_tx_buf_elt) != CO_ERROR_NO_ERROR)
            {
                // Free the buffer
                bt_util_buf_acl_tx_free(buf_ptr);
            }

            // Free the buffer
            bt_util_buf_acl_rx_free(param->em_buf);
        }
        else
        {
            if (lc_env_ptr->link.ConnectionCompleteSent)
            {
                // allocate the message
                struct hci_acl_data *rx_data = KE_MSG_ALLOC(HCI_ACL_DATA, idx, 0, hci_acl_data);
                // set connection handle and flags
                rx_data->conhdl_pb_bc_flag = 0;
                SETF(rx_data->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL,     BT_ACL_CONHDL_MIN + idx);
                SETF(rx_data->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG, GETF(param->data_len_flags, BT_EM_ACL_BF));
                SETF(rx_data->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG, GETF(param->data_len_flags, BT_EM_ACL_PBF));
                // fill data length
                rx_data->length  = GETF(param->data_len_flags, BT_EM_ACL_DATA_LEN);
                // fill the data buffer pointer
                rx_data->buf_ptr = param->em_buf;
                // send the message
                hci_send_2_host(rx_data);

                // Check if AES encryption with secure connection, and packet is point-to-point
                if (lc_env_ptr->enc.EncEnable && lc_env_ptr->sp.sec_con && (GETF(param->data_len_flags, BT_EM_ACL_BF) == BCF_P2P))
                {
                    // Reload timers for authentication payload timeout
                    ke_timer_set(LC_AUTH_PAYL_NEARLY_TO, dest_id, 10 * (lc_env_ptr->link.auth_payl_to - lc_env_ptr->link.auth_payl_to_margin));
                    ke_timer_set(LC_AUTH_PAYL_REAL_TO, dest_id, 10 * lc_env_ptr->link.auth_payl_to);
                }
            }
            else
            {
                // Free the buffer
                bt_util_buf_acl_rx_free(param->em_buf);
            }
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles ACL RX MIC-error indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_acl_rx_mic_err_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        // Increment MIC failures counter
        lc_env_ptr->link.micerr_cnt++;

        /*
         * BT standard specification Vol 2 Part H.9.5 - REPEATED MIC FAILURES :
         *  Any time the MIC check fails and the CRC passes on a given packet, it is
         *  considered an authentication failure. No more than three authentication failures
         *  shall be permitted during the lifetime of an encryption key with a given IV. The
         *  third authentication failure shall initiate an encryption key refresh (see [Vol 2]
         *  Part C, Section 4.2.5.8). If a fourth authentication failure occurs prior to the
         *  encryption key refresh procedure completing, the link shall be disconnected
         *  with reason code Connection Rejected Due to Security Reasons (0x0E).
         *  Note: The MIC is not checked when the CRC is invalid.
         */

        // If counter reaches 3
        if (lc_env_ptr->link.micerr_cnt == 3)
        {
            // Initiate LMP refresh encryption key
            ke_msg_send_basic(LC_OP_EPR_IND, dest_id, dest_id);
        }

        // Else if counter reaches 4
        if (lc_env_ptr->link.micerr_cnt == 4)
        {
            // Initiate detach
            lc_detach(dest_id, CO_ERROR_CONN_REJ_SECURITY_REASONS);
        }
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles ACL TX confirmation
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_acl_tx_cfm, struct lc_acl_tx_cfm)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        if (lm_get_loopback_mode() != REMOTE_LOOPBACK)
        {
            lc_common_nb_of_pkt_comp_evt_send(BT_ACL_CONHDL_MIN + idx, 1);

            if (param->flushed)
            {
                // sends the flush occurred event
                uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
                struct hci_flush_occurred_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_FLUSH_OCCURRED_EVT_CODE, hci_flush_occurred_evt);
                evt->conhdl = conhdl;
                hci_send_2_host(evt);

                // Increment the Failed Contact Counter by 1
                lc_env_ptr->link.FailedContact++;
            }
            else
            {
                // Reset the Failed Contact Counter
                lc_env_ptr->link.FailedContact = 0;
            }
        }
        else
        {
            // Reset the Failed Contact Counter
            lc_env_ptr->link.FailedContact = 0;
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the disconnection indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_acl_disc_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = lc_env_ptr->link.Reason;

    ke_timer_clear(LC_LMP_RSP_TO, dest_id);
    ke_timer_clear(LC_CON_ACCEPT_TO, dest_id);

    if (lc_env_ptr->sp.sec_con)
    {
        // Stop authenticated payload timers
        ke_timer_clear(LC_AUTH_PAYL_NEARLY_TO, dest_id);
        ke_timer_clear(LC_AUTH_PAYL_REAL_TO, dest_id);
    }

    if (ke_state_get(dest_id) != LC_WAIT_DISC_CFM)
    {
        status = CO_ERROR_CON_TIMEOUT;
    }
    else if (lc_env_ptr->link.HostConnected && lc_env_ptr->link.ConnectionCompleteSent)
    {
        status = (lc_env_ptr->req.LocDetachReq) ? CO_ERROR_CON_TERM_BY_LOCAL_HOST : status;
    }

#if (MAX_NB_SYNC > 0)
    // Remove any SCO link on this ACL
    lc_sco_release(dest_id, status);
#endif // (MAX_NB_SYNC > 0)

    // Check if a simple pairing procedure is ongoing, in case the end of procedure needs to be reported
    if (lc_env_ptr->sp.p_data != NULL)
    {
        // End of simple pairing
        lc_sp_end(idx, status);
    }

    if (lc_env_ptr->link.HostConnected)
    {
        if (lc_env_ptr->link.ConnectionCompleteSent)
        {
            // Check if an authentication procedure is ongoing, in case the end of procedure needs to be reported
            if (lc_env_ptr->req.LocAuthReq)
            {
                uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
                struct hci_auth_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_CMP_EVT_CODE, hci_auth_cmp_evt);
                evt->status = status;
                evt->conhdl = conhdl;
                hci_send_2_host(evt);
            }

            //Send the disconnection confirmation
            lc_discon_event_complete_send(dest_id, CO_ERROR_NO_ERROR, BT_ACL_CONHDL_MIN + idx, status);
        }
        else
        {
            lc_con_cmp_evt_send(dest_id, status);
        }
    }
    else if (lc_env_ptr->req.LocNameReq || lc_env_ptr->info.SendRemoteNameCfm)
    {
        // Send remote name request complete event
        struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
        uint8_t status = CO_ERROR_NO_ERROR;

        if (lc_env_ptr->info.SendRemoteNameCfm)
        {
            memcpy(&evt->name.name[0], &lc_env_ptr->info.name_rem.name[0], lc_env_ptr->info.name_rem.namelen);
            memset(&evt->name.name[lc_env_ptr->info.name_rem.namelen], 0, BD_NAME_SIZE - lc_env_ptr->info.name_rem.namelen);
        }
        else if (lc_env_ptr->req.LocNameReq)
        {
            status = CO_ERROR_CON_TIMEOUT;
        }

        evt->status = status;
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(evt);
    }

#if EAVESDROPPING_SUPPORT
    // ACL connection changed
    {
        // Allocate indication message
        struct ed_acl_con_chg_ind *ind = KE_MSG_ALLOC(ED_ACL_CON_CHG_IND, TASK_ED, TASK_NONE, ed_acl_con_chg_ind);

        // Fill data
        memcpy(&ind->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        ind->link_id  = idx;
        ind->status = ED_STATUS_NOT_CONNECTED;
        ind->role = MASTER_ROLE;

        // Send message
        ke_msg_send(ind);
    }
#endif // EAVESDROPPING_SUPPORT

    // Clear sniff parameters
    lc_sniff_clear(idx);

    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        ke_timer_clear(LC_AFH_REPORT_TO, dest_id);
    }

    // Inform LM that the link has been disconnected
    lm_acl_disc(idx);

    ASSERT_ERR(lc_env_ptr->sp.p_data == NULL);

    // set the current lc task in free state
    ke_state_set(dest_id, LC_FREE);
    // Free the memory allocated to the environment of this particular link ID
    ke_free(lc_env[idx]);
    lc_env[idx] = NULL;

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the Role Switch End indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_rsw_end_ind, struct lc_rsw_end_ind)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SWITCH_CMP:
    {
        if (param->status == CO_ERROR_NO_ERROR)
        {
            // Set the new role
            lc_env_ptr->link.Role = !lc_env_ptr->link.Role;

            lc_env_ptr->afh.en = false;
            lc_env_ptr->afh.reporting_en = false;

            lc_env_ptr->sam_info.rem_idx = SAM_DISABLED;
            lc_env_ptr->sam_info.loc_idx = SAM_DISABLED;
            lc_env_ptr->sam_info.rem_tx_av = 0xFF;
            lc_env_ptr->sam_info.rem_rx_av = 0xFF;
            lc_env_ptr->sam_info.loc_tx_av = 0xFF;
            lc_env_ptr->sam_info.loc_rx_av = 0xFF;

            //switch the tr id
            lc_env_ptr->link.RxTrIdServer ^= 1;
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                ke_timer_clear(LC_AFH_REPORT_TO, dest_id);

                lc_env_ptr->link.LinkTimeout = LSTO_DFT;
                lc_env_ptr->link.PollInterval = POLL_INTERVAL_DFT;
                ld_acl_t_poll_set(idx, lc_env_ptr->link.PollInterval);
            }

            // update max slots & packet types
            ke_msg_send_basic(LC_SYNC_IND, dest_id, dest_id);
        }
        else
        {
            lc_sam_restore(idx);
        }

        if (lc_env_ptr->epr.rsw)
        {
            lc_epr_rsw_cmp(dest_id, param->status);
        }
        else
        {
            lc_switch_cmp(dest_id, param->status);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_INFO_FORCE(0, idx, ke_state_get(dest_id));
        break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the slot offset indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_rsw_slot_offset_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SWITCH_CMP:
    {
        // Get slot offset
        uint16_t slot_offset = ld_acl_rsw_slot_offset_get(idx);

        // send LMP_SlotOffset
        lc_send_pdu_slot_off(idx, slot_offset, lc_env_ptr->link.RxTrIdServer);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_INFO_FORCE(0, idx, ke_state_get(dest_id));
        break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the power increase indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_pwr_incr_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (!lc_env_ptr->link.MaxPowerRcv)
        {
            if (lc_env_ptr->link.MinPowerRcv)
            {
                lc_env_ptr->link.MinPowerRcv = false;

#if (EAVESDROPPING_SUPPORT)
                lc_ed_min_max_pwr_ind(idx, ED_INTERM_POWER);
#endif // EAVESDROPPING_SUPPORT
            }

            if (!lc_env_ptr->link.epc_supported)
            {
                // send LMP_incr_power_req
                struct lmp_incr_pwr_req pdu;
                pdu.opcode = LMP_OPCODE(LMP_INCR_PWR_REQ_OPCODE, lc_env_ptr->link.Role);
                pdu.reserved = 0;
                lc_send_lmp(idx, &pdu);

                ke_state_set(dest_id, LC_WAIT_PWR_CTRL_TX_CFM);
            }
            else
            {
                struct lmp_pwr_ctrl_req pdu;
                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
                pdu.ext_opcode = LMP_PWR_CTRL_REQ_EXTOPCODE;
                pdu.pwr_adj  = PWR_ADJ_REQ_INC_1_STEP;
                lc_send_lmp(idx, &pdu);

                lc_start_lmp_to(dest_id);
                ke_state_set(dest_id, LC_WAIT_PWR_CTRL_RES);
            }
        }
    }
    break;
    case LC_WAIT_PWR_CTRL_RES:
    case LC_WAIT_PWR_CTRL_TX_CFM:
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the power decrease indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_pwr_decr_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (!lc_env_ptr->link.MinPowerRcv)
        {
            if (lc_env_ptr->link.MaxPowerRcv)
            {
                lc_env_ptr->link.MaxPowerRcv = false;

#if (EAVESDROPPING_SUPPORT)
                lc_ed_min_max_pwr_ind(idx, ED_INTERM_POWER);
#endif // EAVESDROPPING_SUPPORT
            }

            if (!lc_env_ptr->link.epc_supported)
            {
                // send LMP_decr_power_req
                struct lmp_decr_pwr_req pdu;
                pdu.opcode = LMP_OPCODE(LMP_DECR_PWR_REQ_OPCODE, lc_env_ptr->link.Role);
                pdu.reserved = 0;
                lc_send_lmp(idx, &pdu);

                ke_state_set(dest_id, LC_WAIT_PWR_CTRL_TX_CFM);
            }
            else
            {
                struct lmp_pwr_ctrl_req pdu;
                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
                pdu.ext_opcode = LMP_PWR_CTRL_REQ_EXTOPCODE;
                pdu.pwr_adj  = PWR_ADJ_REQ_DEC_1_STEP;
                lc_send_lmp(idx, &pdu);

                lc_start_lmp_to(dest_id);
                ke_state_set(dest_id, LC_WAIT_PWR_CTRL_RES);
            }
        }
    }
    break;
    case LC_WAIT_PWR_CTRL_RES:
    case LC_WAIT_PWR_CTRL_TX_CFM:
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the max power indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_pwr_max_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (!lc_env_ptr->link.MaxPowerRcv)
        {
            if (lc_env_ptr->link.MinPowerRcv)
            {
                lc_env_ptr->link.MinPowerRcv = false;

#if (EAVESDROPPING_SUPPORT)
                lc_ed_min_max_pwr_ind(idx, ED_INTERM_POWER);
#endif // EAVESDROPPING_SUPPORT
            }

            if (!lc_env_ptr->link.epc_supported)
            {
                // send LMP_incr_power_req
                struct lmp_incr_pwr_req pdu;
                pdu.opcode = LMP_OPCODE(LMP_INCR_PWR_REQ_OPCODE, lc_env_ptr->link.Role);
                pdu.reserved = 0;
                lc_send_lmp(idx, &pdu);

                ke_state_set(dest_id, LC_WAIT_PWR_CTRL_TX_CFM);
            }
            else
            {
                struct lmp_pwr_ctrl_req pdu;
                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
                pdu.ext_opcode = LMP_PWR_CTRL_REQ_EXTOPCODE;
                pdu.pwr_adj  = PWR_ADJ_REQ_INC_MAX;
                lc_send_lmp(idx, &pdu);

                lc_start_lmp_to(dest_id);
                ke_state_set(dest_id, LC_WAIT_PWR_CTRL_RES);
            }
        }
    }
    break;
    case LC_WAIT_PWR_CTRL_RES:
    case LC_WAIT_PWR_CTRL_TX_CFM:
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER(lc_ssr_inst_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SSR_INSTANT:
    {
        lc_sniff_ssr_instant(dest_id);
    }
    break;
    default:
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the sniff offset update indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_sniff_offset_upd_ind, struct lc_sniff_offset_upd_ind)
{
    int idx = KE_IDX_GET(dest_id);

    ASSERT_ERR(lc_env[idx]->link.Role == SLAVE_ROLE);

    lc_sniff_offset_update(idx, param->sniff_offset);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the SCO offset update indication message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_sco_offset_upd_ind, struct lc_sco_offset_upd_ind)
{
#if (MAX_NB_SYNC > 0)
    ASSERT_ERR(lc_env[KE_IDX_GET(dest_id)]->link.Role == SLAVE_ROLE);

    lc_sco_offset_update(param->sco_link_id, param->sco_offset);
#endif // (MAX_NB_SYNC > 0)

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the simple pairing authentication stage 1 random number computing.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_auth_stg1_randn, void)
{
    struct lc_env_tag *lc_env_ptr = lc_env[KE_IDX_GET(dest_id)];

    lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_AUTH_STG1_NC_RANDN:
        if (lc_env_ptr->sp.SPInitiator)
        {
            ke_state_set(dest_id, LC_WAIT_NUM_COMP_COMM_INIT_CONF);
        }
        else
        {
            uint8_t Z = 0x00;
            const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
            const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

            sha_256_f1(lc_env_ptr->sp.sec_con, p_local_pub_key_x, p_remote_pub_key_x,
                       &lc_env_ptr->sp.LocRandN.A[0],
                       &Z,
                       &lc_env_ptr->sp.LocCommitment.A[0]);

            lc_send_pdu_sp_cfm(KE_IDX_GET(dest_id), &lc_env_ptr->sp.LocCommitment, lc_env_ptr->sp.SpTId);

            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_NUM_COMP_RANDN_RSP_PEER);
        }
        break;
    case LC_WAIT_AUTH_STG1_PK_RANDN:
        lc_start_passkey(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_INFO_FORCE(0, ke_state_get(dest_id), 0);
        break;
    }

    return (KE_MSG_CONSUMED);
}

#if BCAST_ENC_SUPPORT
/**
 ****************************************************************************************
 * @brief Handles the master key request message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_mst_key_req, struct lc_mst_key_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_env_ptr->enc.NewKeyFlag = param->new_key_flag;

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        do
        {
            uint8_t status = CO_ERROR_NO_ERROR;
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                if (lc_env_ptr->enc.KeyStatus == KEY_PRESENT)
                {
                    if (lc_env_ptr->link.CurrentMode != LM_PARK_MODE)
                    {
                        lc_env_ptr->req.MasterKeyReq = true;

                        lc_mst_key(dest_id);
                        break;
                    }
                }
                else
                {
                    status = CO_ERROR_PIN_MISSING;
                }
            }
            else
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
            }

            struct lb_mst_key_cfm *msg = KE_MSG_ALLOC(LB_MST_KEY_CFM, TASK_LB, dest_id, lb_mst_key_cfm);
            msg->status     = status;
            msg->enc_mode   = lc_env_ptr->enc.EncMode;
            msg->key_sz_msk = lc_env_ptr->enc.KeySizeMask;
            ke_msg_send(msg);

        }
        while (0);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }
    return (KE_MSG_CONSUMED);
}
#endif // BCAST_ENC_SUPPORT

#if VOICE_OVER_HCI
/**
 ****************************************************************************************
 * @brief Handles Synchronous RX indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_sync_rx_ind, struct lc_sync_rx_ind)
{
    int idx = KE_IDX_GET(dest_id);
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Free the buffer
        bt_util_buf_sync_rx_free(param->sync_link_id, param->em_buf);
    }
    break;
    default:
    {
        // Push synchronous data packet to HCI
        struct hci_sync_data *data_rx = KE_MSG_ALLOC(HCI_SYNC_DATA, idx, dest_id, hci_sync_data);
        data_rx->buf_ptr = param->em_buf;
        data_rx->length  = param->data_len;
        data_rx->conhdl_psf = BT_SYNC_CONHDL(idx, param->sync_link_id);
        SETF(data_rx->conhdl_psf, HCI_SYNC_HDR_PSF, param->packet_status_flag);

        /*
         * If no data has been received, the buffer content is replaced by '0' values, as per BT standard (HCI:5.4.3)
         * " No data received. All data from the baseband received during the (e)SCO interval(s) corresponding to
         * the HCI Synchronous Data Packet have been marked as "lost data" by the baseband. The Payload data octets
         * shall be set to 0."
         */
        if (param->packet_status_flag == NO_RX_DATA_FLAG)
        {
            em_set(0x00, param->em_buf, param->data_len);
        }

        lc_bt_sco_data_send(data_rx);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Synchronous TX confirmation
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_sync_tx_cfm, struct lc_sync_tx_cfm)
{
    int idx = KE_IDX_GET(dest_id);
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Nothing to do
    }
    break;
    default:
    {
        if (lm_sync_flow_ctrl_en_get())
        {
            // Report TX CFM to Host
            lc_common_nb_of_pkt_comp_evt_send(BT_SYNC_CONHDL(idx, param->sync_link_id), 1);
        }
        lc_bt_send_data_done();
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}
#endif // VOICE_OVER_HCI

/**
 ****************************************************************************************
 * @brief Handles the features request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_feat_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if ((lc_env_ptr->info.RemFeatureRec & 0x1) == 0)
        {
            lc_feat(dest_id, idx);
        }
        else if (((lc_env_ptr->info.RemFeatureRec & 0x2) == 0) && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_EXT_FEATS_BIT_POS))
        {
            lc_ext_feat(dest_id, idx, FEATURE_PAGE_1);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the version request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_vers_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_version(dest_id, KE_IDX_GET(dest_id));
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the sniff subrating local negotiation in case of active mode.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_ssrnego_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        lc_sniff_ssr_nego(dest_id);
    }
    break;
    case LC_WAIT_SNIFF_SUB_RSP:
    case LC_WAIT_SNIFF_SUB_RSP_TX_CFM:
    case LC_WAIT_SSR_INSTANT:
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER(lc_op_loc_unsniff_req, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_SSR_INSTANT:
    case LC_CONNECTED:
    {
        // Check if the link is still in sniff mode
        if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
        {
            // Unsniff link
            lc_sniff_unsniff(dest_id);
            break;
        }
    } // No break
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Report the procedure completion
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
        evt->conhdl = conhdl;
        evt->interv = 0;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        hci_send_2_host(evt);
    }
    break;
    default :
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER(lc_op_loc_sniff_req, struct lc_op_loc_sniff_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        // The link mode might have changed between the transmission and reception of this message
        if (lc_env_ptr->link.CurrentMode == LM_ACTIVE_MODE)
        {
            lc_sniff_host_request(dest_id, param->max_int, param->min_int, param->attempt, param->timeout);
            break;
        }
    }
    // No break
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Report the procedure completion
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
        evt->conhdl = conhdl;
        evt->interv = 0;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        hci_send_2_host(evt);
    }
    break;
    default :
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER(lc_op_loc_switch_req, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

        if ((lc_env_ptr->link.CurrentMode != LM_SNIFF_MODE) && (lc_env_ptr->link.LinkPolicySettings & POLICY_SWITCH))
        {
            bool switch_allowed = true;

#if (MAX_NB_SYNC > 0)
            // disallowed if active sco
            if (lm_look_for_sync(idx, ACL_TYPE))
                switch_allowed = false;
#endif // (MAX_NB_SYNC > 0)

#if BCAST_ENC_SUPPORT
            // disallowed if broadcast encrpytion enabled
            if (lc_env_ptr->enc.KeyFlag == TEMPORARY_KEY)
                switch_allowed = false;
#endif // BCAST_ENC_SUPPORT

            if (switch_allowed)
            {
                // Register the Role Switch at LM
                if (lm_role_switch_start(idx, &lc_env_ptr->link.LtAddr))
                {
                    lc_env_ptr->link.Initiator = true;
                    lc_env_ptr->req.LocSwitchReq = true;
                    if (lc_env_ptr->enc.EncMode == ENC_DISABLED)
                    {
                        lc_local_switch(dest_id);
                        break;
                    }
                    else if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_PAUSE_ENCRYPT_BIT_POS))
                    {
                        lc_env_ptr->epr.on = true;
                        lc_env_ptr->epr.rsw = true;
                        //EPR start switch role
                        lc_locepr_rsw(dest_id);
                        break;
                    }

                    // Unregister the Role Switch at LM
                    lm_role_switch_finished(idx, false);
                }
                else
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                }
            }
        }

        // Report the procedure completion
        struct hci_role_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_ROLE_CHG_EVT_CODE, hci_role_chg_evt);
        evt->status = status;
        evt->bd_addr = lc_env_ptr->info.BdAddr;
        evt->new_role = lc_env_ptr->link.Role;
        hci_send_2_host(evt);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Report the procedure completion
        struct hci_role_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_ROLE_CHG_EVT_CODE, hci_role_chg_evt);
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        evt->bd_addr = lc_env_ptr->info.BdAddr;
        evt->new_role = lc_env_ptr->link.Role;
        hci_send_2_host(evt);
    }
    break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

#if MAX_NB_SYNC
KE_MSG_HANDLER(lc_op_loc_sync_con_req, struct lc_op_loc_sync_con_req)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        // Handle the request
        uint8_t status = lc_sco_host_request(dest_id, param->old_style, param->host_initiated, param->conhdl, &lc_env_ptr->sco);

        if ((status != CO_ERROR_NO_ERROR) && (param->host_initiated))
        {
            // Report the procedure completion
            struct hci_sync_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_SYNC_CON_CMP_EVT_CODE, hci_sync_con_cmp_evt);
            evt->status = status;
            evt->conhdl = param->conhdl;
            evt->bd_addr = lc_env_ptr->info.BdAddr;
            hci_send_2_host(evt);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        if (param->host_initiated)
        {
            // Report the procedure completion
            struct hci_sync_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, param->conhdl, HCI_SYNC_CON_CMP_EVT_CODE, hci_sync_con_cmp_evt);
            evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
            evt->conhdl = param->conhdl;
            evt->bd_addr = lc_env_ptr->info.BdAddr;
            hci_send_2_host(evt);
        }
    }
    break;
    default :
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
#endif // MAX_NB_SYNC

/**
 ****************************************************************************************
 * @brief Handles the QOS operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_qos_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        lc_qos_setup(dest_id);
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the Authentication request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_auth_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_env[KE_IDX_GET(dest_id)]->req.LocAuthReq = true;
        lc_env[KE_IDX_GET(dest_id)]->link.Initiator = true;
        lc_loc_auth(dest_id, KE_IDX_GET(dest_id));
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the Encryption request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_enc_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_send_enc_mode(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the clock offset request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_clkoff_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            // send LMP_ClkOffsetReq(idx,Role)
            struct lmp_clk_off_req pdu;
            pdu.opcode = LMP_OPCODE(LMP_CLK_OFF_REQ_OPCODE, MASTER_ROLE);
            lc_send_lmp(idx, &pdu);

            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_CLK_OFF);
        }
        else
        {
            // Report procedure completion
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_rd_clk_off_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_CLK_OFF_CMP_EVT_CODE, hci_rd_clk_off_cmp_evt);
            event->status      = CO_ERROR_NO_ERROR;
            event->conhdl      = conhdl;
            event->clk_off_val = (ld_acl_clock_offset_get(idx).hs >> 1) & 0x7FFF;
            hci_send_2_host(event);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
    {
        // Report procedure completion
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_rd_clk_off_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_CLK_OFF_CMP_EVT_CODE, hci_rd_clk_off_cmp_evt);
        event->status      = CO_ERROR_UNKNOWN_CONNECTION_ID;
        event->conhdl      = conhdl;
        event->clk_off_val = 0;
        hci_send_2_host(event);
    }
    break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the Encryption Pause Resume request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_epr_ind, void)
{
    struct lc_env_tag *lc_env_ptr = lc_env[KE_IDX_GET(dest_id)];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_PAUSE_ENCRYPT_BIT_POS))
        {
            lc_env_ptr->epr.on = true;
            if (lc_env_ptr->epr.rsw)
            {
                lc_locepr_rsw(dest_id);
            }
            else
            {
                lc_locepr_lkref(dest_id);
            }
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the clock offset request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_timacc_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (LM_GetFeature(&lc_env[KE_IDX_GET(dest_id)]->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_TIMING_ACC_BIT_POS))
        {
            lc_send_pdu_tim_acc(KE_IDX_GET(dest_id), lc_env[KE_IDX_GET(dest_id)]->link.Role);
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_TIM_ACC_RSP);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the packet type request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_pt_ind, void)
{
    int idx = KE_IDX_GET(dest_id);

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_packet_type(dest_id, idx);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

#if RW_BT_MWS_COEX
/**
****************************************************************************************
* @brief Handles the SAM configuration request operation.
*
* @param[in] msgid Id of the message received.
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance.
* @param[in] src_id ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lc_op_sam_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SAM_BIT_POS))
        {
            struct lm_sam_config_req *msg = KE_MSG_ALLOC(LM_SAM_CONFIG_REQ, TASK_LM, KE_BUILD_ID(TASK_LC, idx), lm_sam_config_req);
            msg->link_id = idx;
            msg->submap_av = lc_env_ptr->sam_info.loc_submap0_av;
            memcpy(&msg->pattern_av[0], &lc_env_ptr->sam_info.loc_pattern_av[0], sizeof(bool)*SAM_INDEX_MAX);
            ke_msg_send(msg);

            // configuration sequence of SAM maps is activated
            lc_env_ptr->sam_info.config_mode = true;
        }
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
****************************************************************************************
* @brief Handles the SAM configuration request operation.
*
* @param[in] msgid Id of the message received.
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance.
* @param[in] src_id ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lc_op_sam_cfm, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // configuration sequence of SAM maps is complete
    lc_env_ptr->sam_info.config_mode = false;

    return (KE_MSG_CONSUMED);
}
#endif //RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Handles the host connection request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_hl_cnx_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_hl_connect(dest_id, KE_IDX_GET(dest_id));
        ke_msg_send_basic(LC_OP_HL_CNX_CONT_IND, dest_id, dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the host connection request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_hl_cnx_cont_ind, void)
{
    struct lc_env_tag *lc_env_ptr = lc_env[KE_IDX_GET(dest_id)];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        //Check if AFH is possible
        if ((lc_env_ptr->link.Role == MASTER_ROLE) && LM_GetFeature(&lc_env[KE_IDX_GET(dest_id)]->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_AFH_CAPABLE_S_BIT_POS))
        {
            lc_afh_start(dest_id);
        }
        //Check if authentication before connection should be done
        if (lm_get_auth_en() == AUTH_ENABLED)
        {
            ke_msg_send_basic(LC_OP_AUTH_IND, dest_id, dest_id);
        }
        ke_msg_send_basic(LC_OP_SET_CMP_IND, dest_id, dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        if (lc_env_ptr->link.SetupComplete)
        {
            break;
        }
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the setup completed operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_set_cmp_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        // Completes the ACL conenction establishment
        lc_setup_cmp(KE_IDX_GET(dest_id));
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the remote name request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_name_req_ind, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
        lc_rd_rem_name(dest_id);
        break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the link disconnection request operation.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_op_detach_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    case LC_CONNECTED:
    {
        lc_env_ptr->link.Initiator = true;

        // Check if the link is in sniff mode
        if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
        {
            // Unsniff link
            lc_sniff_unsniff(dest_id);

            return (KE_MSG_SAVED);
        }

#if (MAX_NB_SYNC > 0)
        // Check if SCO is present, if yes detach all SCO connections before ACL
        if (lc_sco_detach(dest_id, lc_env_ptr->link.Reason) == CO_ERROR_NO_ERROR)
        {
            return (KE_MSG_SAVED);
        }
#endif //(MAX_NB_SYNC > 0)

        // The detach process can be initiated
        lc_detach(dest_id, lc_env_ptr->link.Reason);
        break;
    }
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the synchronous indication request.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_sync_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    case LC_WAIT_SWITCH_CMP:
    {
        return (KE_MSG_SAVED);
    }
    break;
    default:
    {
        uint16_t pkt_type  = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                             + PACKET_TYPE_NO_2_DH3_FLAG + PACKET_TYPE_NO_3_DH3_FLAG
                             + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
        uint8_t max_slot = 0x01;
        uint8_t min_sync_intv = 0xFF;
#if (MAX_NB_SYNC > 0)
        min_sync_intv = lm_get_min_sync_intv();
#endif //(MAX_NB_SYNC > 0)

#if (EAVESDROPPING_SUPPORT)
        uint8_t host_max_slot = lm_get_host_max_slot();
        uint8_t max_slot_received = (lc_env_ptr->link.MaxSlotReceived >= 3) ? lc_env_ptr->link.MaxSlotReceived : 1;

        if ((min_sync_intv < 12) || (host_max_slot == 1))
        {
            max_slot = 0x1;
        }
        else if (((min_sync_intv < 18) || (host_max_slot == 3)) && (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_3_SLOT_BIT_POS)))
        {
            max_slot = 0x03;

            if (max_slot_received >= 3)
            {
                pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                           + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                           + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
            }
        }
        else if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_5_SLOT_BIT_POS))
        {
            max_slot = 0x05;

            if (max_slot_received == 5)
            {
                pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                           + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                           + PACKET_TYPE_DM5_FLAG + PACKET_TYPE_DH5_FLAG;
            }
            else if (max_slot_received == 3)
            {
                pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                           + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                           + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
            }
        }

        if (max_slot != ld_acl_rx_max_slot_get(idx))
        {
            lc_send_pdu_max_slot(idx, max_slot, lc_env_ptr->link.Role);

            // Update the maximum number of slots the peer device is allowed to use
            ld_acl_rx_max_slot_set(idx, max_slot);
        }

        pkt_type = LM_ComputePacketType(lc_env_ptr->link.AclPacketType, pkt_type, false);
        pkt_type = LM_ComputePacketType(lc_env_ptr->link.RxPreferredRate, pkt_type, true);
        lc_env_ptr->link.CurPacketType = pkt_type;
        lc_env_ptr->link.TxMaxSlotCur = co_min(max_slot, max_slot_received);
        ld_acl_allowed_tx_packet_types_set(idx, pkt_type);

#else
        if (min_sync_intv < 12)
        {
            max_slot = 0x1;
        }
        else if ((min_sync_intv < 18) && (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_3_SLOT_BIT_POS)))
        {
            max_slot = 0x03;

            pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                       + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                       + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
        }
        else if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_5_SLOT_BIT_POS))
        {
            max_slot = 0x05;

            pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                       + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                       + PACKET_TYPE_DM5_FLAG + PACKET_TYPE_DH5_FLAG;
        }

        if (max_slot != ld_acl_rx_max_slot_get(idx))
        {
            lc_send_pdu_max_slot(idx, max_slot, lc_env_ptr->link.Role);

            // Update the maximum number of slots the peer device is allowed to use
            ld_acl_rx_max_slot_set(idx, max_slot);
        }

        pkt_type = LM_ComputePacketType(lc_env_ptr->link.CurPacketType, pkt_type, false);
        pkt_type = LM_ComputePacketType(lc_env_ptr->link.RxPreferredRate, pkt_type, true);
        ld_acl_allowed_tx_packet_types_set(idx, pkt_type);
#endif // EAVESDROPPING_SUPPORT
    }
    break;
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the LMP response time out.
 * The handler is in charge to change the state of the current logical link controller
 * task and to notify the cancellation of the link to the higher layers.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_lmp_rsp_to_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // check if pointer is valid since presence of connection is checked later
    if (lc_env_ptr != NULL)
    {
        // reset the values: LinkId, Opcode, OpcodeExt, Mode
        lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
    }

    switch (ke_state_get(dest_id))
    {
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
    case LC_SCO_DISC_ONGOING:
        lc_sco_timeout(dest_id);
        break;
#endif // (MAX_NB_SYNC > 0)

    case LC_WAIT_AU_RAND_RSP:
    case LC_WAIT_AU_RAND_PEER_INIT:
    case LC_WAIT_SRES_INIT:
    case LC_WAIT_AU_RAND_SEC_AUTH_INIT:
        lc_mutual_auth_end(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
    case LC_WAIT_EPR_RSP:
        lc_env_ptr->epr.on = true;
        lc_env_ptr->req.LocEncKeyRefresh = true;
        lc_enc_key_refresh(dest_id);
        break;

    case LC_WAIT_HL_OOB_DATA:
        memset(&lc_env_ptr->sp.RemCommitment.A[0], 0x00, sizeof(struct byte16));
        if (lc_env_ptr->sp.SPInitiator)
        {
            // send LMP_OOBFailed(idx,sptid)
            struct lmp_oob_fail pdu;

            pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->sp.SpTId);
            pdu.ext_opcode = LMP_OOB_FAIL_EXTOPCODE;

            lc_send_lmp(idx, &pdu);

            lc_sp_fail(dest_id);
        }
        else
        {
            lc_resp_oob_wait_nonce(dest_id);
        }
        break;
    case LC_WAIT_PASSKEY_HL_RPLY:
        lc_env_ptr->sp.SPPhase1Failed = 1;
        lc_start_passkey_loop(dest_id);
        break;
    case LC_WAIT_DHKEY_CHECK_RSP_PEER_CFM:
    case LC_WAIT_DHKEY_RSP_FAIL:
    case LC_WAIT_DHKEY_RSP:
    case LC_WAIT_DHKEY_CHECK_INIT_PEER:
    case LC_WAIT_DHKEY_CHECK_INIT_CFM:
    case LC_WAIT_OOB_RANDN_RSP_PEER_CFM:
    case LC_WAIT_OOB_RANDN_RSP_PEER:
    case LC_WAIT_OOB_RANDN_INIT_PEER:
    case LC_WAIT_PASSKEY_RANDN_RSP_PEER_CFM:
    case LC_WAIT_PASSKEY_RANDN_RSP_PEER:
    case LC_WAIT_PASSKEY_COMM_RSP_PEER:
    case LC_WAIT_PASSKEY_RANDN_INIT_PEER:
    case LC_WAIT_PASSKEY_RANDN_INIT_CFM:
    case LC_WAIT_PASSKEY_COMM_INIT_PEER:
    case LC_WAIT_NUM_COMP_USER_CONF_RSP:
    case LC_WAIT_NUM_COMP_RANDN_RSP_PEER:
    case LC_WAIT_NUM_COMP_RANDN_RSP_PEER_CFM:
    case LC_WAIT_NUM_COMP_COMM_INIT_RANDN_CFM:
    case LC_WAIT_NUM_COMP_COMM_INIT_RANDN_PEER:
    case LC_WAIT_NUM_COMP_COMM_INIT_CONF:
    case LC_PUB_KEY_PAYLOAD_RSP_PEER_CFM:
    case LC_PUB_KEY_HEADER_RSP_CFM:
    case LC_WAIT_PUB_KEY_PAYLOAD_RSP_PEER:
    case LC_WAIT_PUB_KEY_HEADER_RSP_PEER:
    case LC_WAIT_PUB_KEY_PAYLOAD_INIT_PEER:
    case LC_WAIT_PUB_KEY_HEADER_INIT_PEER:
    case LC_PUB_KEY_PAYLOAD_INIT_LOC:
    case LC_PUB_KEY_HEADER_INIT_LOC:
    case LC_WAIT_HL_IO_CAP_RSP:
    case LC_WAIT_IO_CAP_INIT_CFM:
    case LC_WAIT_PRIV_PUB_KEY_GEN:
    {
        // End of simple pairing
        lc_sp_end(idx, CO_ERROR_LMP_RSP_TIMEOUT);

        lc_env_ptr->link.Reason = CO_ERROR_LMP_RSP_TIMEOUT;

        lc_mutual_auth_end(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
    }
    break;
    case LC_WAIT_NUM_COMP_USER_CONF_INIT:
        // send LMP_NumericComparaisonFailed(idx,sptid)
        lc_send_pdu_num_comp_fail(idx, lc_env_ptr->sp.SpTId);
    case LC_WAIT_HL_IO_CAP_INIT:
        lc_sp_fail(dest_id);
        break;
    case LC_WAIT_EPR_ENC_RESUME_REQ_MST_RSP:
        lc_env_ptr->epr.rsw = false;
    case LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT:
        lc_env_ptr->epr.on = false;
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_enc_key_refresh_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_ENC_KEY_REFRESH_CMP_EVT_CODE, hci_enc_key_refresh_cmp_evt);
            evt->status = CO_ERROR_LMP_RSP_TIMEOUT;
            evt->conhdl = conhdl;
            hci_send_2_host(evt);
        }
        lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
    case LC_WAIT_REM_EXT_FEATS:
        lc_env_ptr->link.Reason = CO_ERROR_LMP_RSP_TIMEOUT;
        if (!lc_env_ptr->link.HostConnected &&
                lc_env_ptr->req.LocNameReq)
        {
            // Send remote name request complete event
            struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
            evt->status = CO_ERROR_LMP_RSP_TIMEOUT;
            memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
            hci_send_2_host(evt);
        }

        // detach with lmp response timeout error
        lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
#if BCAST_ENC_SUPPORT
    case LC_WAIT_MST_SEMI_ACC:
        lc_semi_key_cmp(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
#endif // BCAST_ENC_SUPPORT
    case LC_WAIT_SWITCH_CFM:
        if (lc_env_ptr->epr.rsw)
        {
            lc_epr_rsw_cmp(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        }
        else
        {
            lc_switch_cmp(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        }
        break;
    case LC_WAIT_QOS_CFM:
    {
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_qos_setup_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_QOS_SETUP_CMP_EVT_CODE, hci_qos_setup_cmp_evt);
        // update the status
        event->status    = CO_ERROR_LMP_RSP_TIMEOUT;
        event->conhdl    = conhdl;
        event->flags     = 0;
        event->tok_rate  = lc_env_ptr->link.TokenRate;
        event->serv_type = lc_env_ptr->link.ServiceType;
        event->pk_bw     = lc_env_ptr->link.PeakBandwidth;
        event->lat       = lc_env_ptr->link.Latency;
        event->del_var   = lc_env_ptr->link.DelayVariation;
        // send the message
        hci_send_2_host(event);
    }

    lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
    break;
    case LC_WAIT_FLOW_SPEC_CFM:
    {
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        // Report the procedure completion
        struct hci_flow_spec_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_FLOW_SPEC_CMP_EVT_CODE, hci_flow_spec_cmp_evt);
        evt->status   = CO_ERROR_LMP_RSP_TIMEOUT;
        evt->conhdl   = conhdl;
        evt->flags    = 0;
        evt->flow_dir = lc_env_ptr->link.FlowDirection;
        evt->pk_bw    = lc_env_ptr->link.PeakBandwidth;
        evt->serv_type = lc_env_ptr->link.ServiceType;
        evt->tk_buf_sz = lc_env_ptr->link.TokenBucketSize;
        evt->tk_rate  = lc_env_ptr->link.TokenRate;
        evt->acc_lat  = lc_env_ptr->link.AccessLatency;
        hci_send_2_host(evt);

        ke_state_set(dest_id, LC_CONNECTED);
    }
    break;
    case LC_WAIT_CLK_OFF:
    {
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        // Report procedure completion
        struct hci_rd_clk_off_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_CLK_OFF_CMP_EVT_CODE, hci_rd_clk_off_cmp_evt);
        event->status      = CO_ERROR_LMP_RSP_TIMEOUT;
        event->conhdl      = conhdl;
        event->clk_off_val = 0x0000;
        hci_send_2_host(event);

        ke_state_set(dest_id, LC_CONNECTED);
    }
    break;

    case LC_WAIT_UNSNIFF_ACC_TX_CFM:
    case LC_WAIT_UNSNIFF_ACC:
    {
        lc_sniff_unsniff_peer_reject(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
    }
    break;
    case LC_WAIT_SNIFF_REQ:
        lc_sniff_timeout(dest_id);
        break;
    case LC_WAIT_REM_VERS:
    {
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_rd_rem_ver_info_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_VER_INFO_CMP_EVT_CODE, hci_rd_rem_ver_info_cmp_evt);

        evt->status  = CO_ERROR_LMP_RSP_TIMEOUT;
        evt->conhdl  = conhdl;
        evt->compid  = 0;
        evt->subvers = 0;
        evt->vers    = 0;

        hci_send_2_host(evt);
    }
    lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
    break;
    case LC_WAIT_DETACH_TO:
    {
        lc_release(dest_id);
    }
    break;
    case LC_WAIT_ENC_STOP_CFM_MST:
        lc_env_ptr->enc.EncEnable = ENCRYPTION_OFF;
    case LC_WAIT_ENC_START_CFM:
        ke_timer_clear(LC_LMP_RSP_TO, dest_id);
        ld_acl_rx_enc(idx, LD_ENC_DISABLE);

        if (lc_env_ptr->req.LocEncReq)
        {
            lc_send_enc_chg_evt(idx, CO_ERROR_LMP_RSP_TIMEOUT);
        }
        lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;

    case LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT:
        if (lc_env_ptr->req.LocEncReq)
        {
            lc_send_enc_chg_evt(idx, CO_ERROR_LMP_RSP_TIMEOUT);
            lc_env_ptr->req.LocEncReq = false;
        }
        lc_env_ptr->link.Initiator = true;
        ld_acl_flow_on(idx);
        lc_detach(dest_id, CO_ERROR_ENC_MODE_NOT_ACCEPT);
        break;

    case LC_WAIT_ENC_SLV_RESTART:
    case LC_WAIT_ENC_SLV_SIZE:
    case LC_WAIT_ENC_SIZE_CFM:
        lc_enc_cmp(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;

    case LC_WAIT_DETACH_REQ_TX_CFM:
        lc_env_ptr->link.Reason = CO_ERROR_CON_TIMEOUT;
    case LC_WAIT_DISC_TO:
        lc_release(dest_id);
        break;

    case LC_WAIT_DISC_CFM:
        break;

    case LC_WAIT_REM_NAME:
    {
        // Send remote name request complete event
        struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
        evt->status = CO_ERROR_LMP_RSP_TIMEOUT;
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(evt);
    }
    lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
    break;

    case LC_WAIT_REM_FEATS:
        lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;

#if BCAST_ENC_SUPPORT
    case LC_MST_WAIT_ENC_SIZE_MSK:
        lc_semi_key_cmp(CO_ERROR_LMP_RSP_TIMEOUT, dest_id);
        lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
#endif // BCAST_ENC_SUPPORT
    case LC_WAIT_PAIR_CFM_INIT:
    case LC_WAIT_PAIR_CFM_RSP:
        lc_env_ptr->enc.KeyStatus = KEY_ABSENT;
        lc_env_ptr->link.Reason = CO_ERROR_LMP_RSP_TIMEOUT;
    case LC_WAIT_AUTH_SRES:
        lc_auth_cmp(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
    case LC_WAIT_SNIFF_SUB_RSP:
    {
        lc_sniff_ssr_peer_reject(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
    }
    break;
    case LC_FREE:
        //Free msg in case of LSTO
        break;
    case LC_WAIT_EPR_PEER_REQ_MST:
        //No request from the peer restart the encryption
        lc_start_enc(dest_id);
        break;
    case LC_WAIT_EPR_PEER_REQ_SLV:
    case LC_WAIT_REM_HL_CON:
    case LC_WAIT_PKT_TBL_TYP_ACC_CFM:
    case LC_WAIT_MAX_SLOT_CFM:
    case LC_WAIT_ENC_MODE_CFM:
    case LC_WAIT_PWR_CTRL_RES:
    case LC_WAIT_TIM_ACC_RSP:
    case LC_WAIT_PIN_CODE:
#if BCAST_ENC_SUPPORT
    case LC_MST_WAIT_VERS:
#endif // BCAST_ENC_SUPPORT
    default:
        lc_detach(dest_id, CO_ERROR_LMP_RSP_TIMEOUT);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the connection accept timeout request.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_con_accept_to, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (ke_state_get(dest_id))
    {
    case LC_WAIT_ACL_ACC:
        // send LMP_NotAccepted(LMP_HOST_CON_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_HOST_CON_REQ_OPCODE, CO_ERROR_CONN_ACCEPT_TIMEOUT_EXCEED, lc_env_ptr->link.RxTrIdServer);
        lc_detach(dest_id, CO_ERROR_CONN_ACCEPT_TIMEOUT_EXCEED);
        break;
#if (MAX_NB_SYNC > 0)
    case LC_SCO_NEGO_ONGOING:
        lc_sco_host_reject(dest_id, false, CO_ERROR_CONN_ACCEPT_TIMEOUT_EXCEED);
        break;
#endif // (MAX_NB_SYNC > 0)
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        ASSERT_INFO_FORCE(0, ke_state_get(dest_id), 0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the AFH report timeout message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_afh_report_to, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    DBG_SWDIAG(AFH, REPORT_TO, 1);

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
    {
        // Do not generate another lmp_ch_class PDU if the previous one is still pending
        if (!lc_env_ptr->afh.lmp_ch_class_pending)
        {
            struct bt_ch_class ch_class;

            // Get Host channel classification
            struct bt_ch_map *host_ch_class = lm_afh_host_ch_class_get();

#if (EAVESDROPPING_SUPPORT)
            // Get ED channel classification
            struct bt_ch_map *ed_ch_class = lm_afh_ed_ch_class_get();
#endif // EAVESDROPPING_SUPPORT

            // Get local channel classification based on assessment
            rwip_ch_class_bt_get(&ch_class);

            for (int i = 0 ; i < BT_CH_MAP_LEN ; i++)
            {
                // At each loop, 4 channel pairs are classified
                for (int j = 0 ; j < 4 ; j++)
                {
                    /*
                     * Decision made for pairs of frequencies:
                     *  - BB => B
                     *  - BU => B
                     *  - BG => B
                     *  - UU => U
                     *  - UG => G
                     *  - GG => G
                     * Each bit of the host channel classification corresponds to:
                     *      0: bad channel
                     *      1: unknown channel (treated as good)
                     */
                    uint8_t class = AFH_CH_CLASS_UNKNOWN;

                    do
                    {
                        // Host indicates the 1st channel as bad
                        if (((host_ch_class->map[i] >> (2 * j)) & 0x1) == 0x00)
                        {
                            class = AFH_CH_CLASS_BAD;
                            break;
                        }

                        // Host indicates the 2nd channel as bad
                        if (((host_ch_class->map[i] >> (2 * j + 1)) & 0x1) == 0x00)
                        {
                            class = AFH_CH_CLASS_BAD;
                            break;
                        }

#if (EAVESDROPPING_SUPPORT)
                        // ED channel classification is calculated in pairs
                        if (((ed_ch_class->map[i] >> (2 * j)) & 0x3) == AFH_CH_CLASS_BAD)
                        {
                            class = AFH_CH_CLASS_BAD;
                            break;
                        }
#endif // EAVESDROPPING_SUPPORT
                    }
                    while (0);

                    if (class == AFH_CH_CLASS_BAD)
                    {
                        // Clear previous state
                        ch_class.map[i] &= ~(3 << (2 * j));
                        // frequency pair is considered as bad
                        ch_class.map[i] |= (AFH_CH_CLASS_BAD << (2 * j));
                    }

                }
            }

            // Check if report has changed
            if (memcmp(&lc_env_ptr->afh.ch_class.map[0], &ch_class.map[0], BT_CH_CLASS_MAP_LEN))
            {
                // send LMP_ChannelClassification
                struct lmp_ch_class pdu;
                memcpy(&pdu.ch_class.map[0], &ch_class.map[0], BT_CH_CLASS_MAP_LEN);
                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
                pdu.ext_opcode = LMP_CH_CLASS_EXTOPCODE;
                lc_send_lmp(idx, &pdu);
                lc_env_ptr->afh.lmp_ch_class_pending = true;

                // Save channel classification report
                memcpy(&lc_env_ptr->afh.ch_class.map[0], &ch_class.map[0], BT_CH_CLASS_MAP_LEN);
            }
        }

        // Restart reporting timer
        ke_timer_set(LC_AFH_REPORT_TO, dest_id, 10 * co_slot_to_duration(lc_env_ptr->afh.reporting_interval));
    }
    break;
    }

    DBG_SWDIAG(AFH, REPORT_TO, 0);

    return (KE_MSG_CONSUMED);
}

#if RW_BT_MWS_COEX
/**
****************************************************************************************
* @brief Handles the SAM Submap update indication
*
* @param[in] msgid Id of the message received.
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance.
* @param[in] src_id ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lc_sam_submap_update_ind, struct lc_sam_submap_update_ind)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SAM_BIT_POS))
        {
            lc_send_pdu_sam_set_type0(idx, param->update_mode, &param->submap[0], lc_env_ptr->link.Role);
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_SAM_SET_TYPE0_CFM);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}


/**
****************************************************************************************
* @brief Handles the SAM Map update indication
*
* @param[in] msgid Id of the message received.
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance.
* @param[in] src_id ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lc_sam_map_update_ind, struct lc_sam_map_update_ind)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SAM_BIT_POS))
        {
            lc_send_pdu_sam_define_map(idx, param->pattern_index, param->t_sam_sm, param->n_sam_sm, &param->submaps.map[0], lc_env_ptr->link.Role);
            lc_env_ptr->sam_info.loc_pattern_av[param->pattern_index] = (param->n_sam_sm != 0);
            lc_env_ptr->sam_info.loc_idx_wait_cfm = param->pattern_index;

            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_SAM_DEFINE_MAP_CFM);
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

/**
****************************************************************************************
* @brief Handles the MWS Pattern update indication
*
* @param[in] msgid Id of the message received.
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance.
* @param[in] src_id ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lc_mws_pattern_ind, struct lc_mws_pattern_ind)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SAM_BIT_POS))
        {
            // Do not invoke a SAM Switch on an pattern which is not fully defined
            if (((param->pattern_index == SAM_DISABLED) || (lc_env_ptr->sam_info.loc_pattern_av[param->pattern_index] && ((0 == param->n_ex_sm) || lc_env_ptr->sam_info.loc_submap0_av)))
                    && (param->pattern_index != lc_env_ptr->sam_info.loc_idx))
            {
                uint32_t sam_instant_clk;
                uint8_t d_sam = 0;
                uint8_t flags = 0;

                // Determine sam instant, d_sam, timing control flags for this link
                if (SLAVE_ROLE == lc_env_ptr->link.Role)
                {
                    rwip_time_t clk_offset = ld_acl_clock_offset_get(idx);
                    sam_instant_clk = CLK_ADD_3(param->time.hs, clk_offset.hs, (param->time.hus + (HALF_SLOT_SIZE - clk_offset.hus) / HALF_SLOT_SIZE));
                }
                else
                {
                    sam_instant_clk = param->time.hs;
                }

                // Instant must be on an even slot boundary - mask lower bits
                sam_instant_clk &= ~(0x3);

                // The sam_instant_clk is the start of the next mws frame
                lc_env_ptr->sam_info.loc_idx_wait_cfm = param->pattern_index;
                lc_env_ptr->sam_info.instant = sam_instant_clk >> 1;

                if (param->pattern_index != SAM_DISABLED)
                {
                    d_sam = CO_MOD(((sam_instant_clk & (BT_CLOCK_MSB - 1)) >> 1), param->t_sam);
                    flags = (sam_instant_clk & BT_CLOCK_MSB) ? INIT2_FLAG : 0;

                    lc_env_ptr->sam_info.t_sam = param->t_sam;
                    lc_env_ptr->sam_info.t_sam_sm = param->t_sam_sm;
                    lc_env_ptr->sam_info.n_tx_slots = param->n_tx_slots + param->n_ex_sm * lm_sam_submap_tx_slots_get(param->t_sam_sm);
                    lc_env_ptr->sam_info.n_rx_slots = param->n_rx_slots + param->n_ex_sm * lm_sam_submap_rx_slots_get(param->t_sam_sm);
                }

                // Send SAM Switch to the peer
                lc_send_pdu_sam_switch(idx, param->pattern_index, flags, d_sam, sam_instant_clk >> 1, lc_env_ptr->link.Role);
                lc_start_lmp_to(dest_id);
                ke_state_set(dest_id, LC_WAIT_SAM_SWITCH_CFM);
            }
        }
    }
    break;
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case of LSTO or Drop this response as we are disconnecting
        break;
    default:
        return (KE_MSG_SAVED);
    }

    return (KE_MSG_CONSUMED);
}

#endif // RW_BT_MWS_COEX

/**
****************************************************************************************
* @brief Handles the SAM change indication
*
* @param[in] msgid Id of the message received.
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance.
* @param[in] src_id ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lc_sam_change_ind, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t conhdl = BT_ACL_CONHDL_MIN + idx;

    // SAM Switch on instant  - notify Host of a successful SAM switch sequence
    struct hci_sam_status_change_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SAM_STATUS_CHANGE_EVT_CODE, hci_sam_status_change_evt);

    evt->conhdl = conhdl;

    evt->loc_idx = lc_env_ptr->sam_info.loc_idx;
    evt->loc_tx_av = lc_env_ptr->sam_info.loc_tx_av;
    evt->loc_rx_av = lc_env_ptr->sam_info.loc_rx_av;

    evt->rem_idx = lc_env_ptr->sam_info.rem_idx;
    evt->rem_tx_av = lc_env_ptr->sam_info.rem_tx_av;
    evt->rem_rx_av = lc_env_ptr->sam_info.rem_rx_av;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the authentication timeout nearly expired
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_auth_payl_nearly_to, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_CONNECTED:
    {
        // Send LMP_ping_req
        struct lmp_ping_req pdu;
        pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
        pdu.ext_opcode = LMP_PING_REQ_EXTOPCODE;
        lc_send_lmp(idx, &pdu);

        // Start the LMP response timeout
        lc_start_lmp_to(dest_id);

        // Wait for the response
        ke_state_set(dest_id, LC_WAIT_PING_RES);
    }
    break;
    default:
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the authentication timeout really expired
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_auth_payl_real_to, void)
{
    int idx = KE_IDX_GET(dest_id);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        //Free msg in case disconnecting
        break;
    default:
    {
        // Report event to the Host
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_auth_payl_to_exp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_PAYL_TO_EXP_EVT_CODE, hci_auth_payl_to_exp_evt);
        event->conhdl = conhdl;
        hci_send_2_host(event);

        // Restart timers
        ke_timer_set(LC_AUTH_PAYL_NEARLY_TO, dest_id, 10 * (lc_env_ptr->link.auth_payl_to - lc_env_ptr->link.auth_payl_to_margin));
        ke_timer_set(LC_AUTH_PAYL_REAL_TO, dest_id, 10 * lc_env_ptr->link.auth_payl_to);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

#if PCA_SUPPORT

#if RW_BT_MWS_COEX
/**
 ****************************************************************************************
 * @brief Handles a local PCA request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_local_pca_req, struct lc_local_pca_req)
{
    if (ke_state_get(dest_id) == LC_CONNECTED)
    {
        int idx = KE_IDX_GET(dest_id);

        // Request clock adjust to peer if supported
        if (LM_GetFeature(&lc_env[idx]->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_COARSE_CLK_ADJ_BIT_POS))
        {
            lc_send_pdu_clk_adj_req(idx, param->clk_adj_us, param->clk_adj_slots, param->clk_adj_period, SLAVE_ROLE);
            lc_start_lmp_to(dest_id);
            ke_state_set(dest_id, LC_WAIT_CLK_ADJ_REQ_ACC_CFM);
        }
    }
    else
    {
        // Discard the message and wait for fresh PCA sample on next frame
        ld_pca_reporting_enable(true);
    }

    return (KE_MSG_CONSUMED);
}
#endif // RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Handles a PCA synchronisation scan recovery indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_pca_sscan_clk_ind, struct lc_pca_sscan_clk_ind)
{
    int idx = KE_IDX_GET(dest_id);
    struct bd_addr *p_bd_addr = &lc_env[idx]->info.BdAddr;

    /* A slave that receives a synchronisation train packet with the BD_ADDR of its master shall change the value
       slave_offset to make CLK correspond to the contents of the packet */
    if (!memcmp(&p_bd_addr->addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
    {
        /* TODO: If the new clock is in the range CLKold - LSTO to CLKold inclusive, then this shall be treated
           as a negative change. In this case, the slave shall not transmit any packet and shall ignore all received
           directed packets until the value of CLK, after being changed, is strictly greater than the value of CLK
           the last time it transmitted or received a packet. BB 8.6.10.2 */
        ld_acl_clk_set(idx, param->clock_offset, param->bit_offset);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles a Sync Scan request for PCA recovery
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_pca_sscan_start_req, void)
{
    uint8_t idx = KE_IDX_GET(dest_id);

    // Check if peer device may be using Corase Clock Adjust and Sync Train
    if (LM_GetFeature(&lc_env[idx]->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_COARSE_CLK_ADJ_BIT_POS)
            &&    LM_GetFeature(&lc_env[idx]->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_SYNC_TRAIN_BIT_POS))
    {
        // Request to start synchronization scan for this link
        lb_sscan_start_req(idx, &lc_env[idx]->info.BdAddr);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles a Sync Scan stop request for PCA recovery
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lc_pca_sscan_stop_req, void)
{
    uint8_t idx = KE_IDX_GET(dest_id);

    // Request to stop synchronization scan for this link
    lb_sscan_stop_req(idx);

    return (KE_MSG_CONSUMED);
}

#endif // PCA_SUPPORT

#if (EAVESDROPPING_SUPPORT)
KE_MSG_HANDLER(lc_sco_delayed_host_req, void)
{
    lc_sco_send_delayed_host_req(dest_id);
    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER(lc_mic_enc_key_refresh, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        // Initiate LMP refresh encryption key
        ke_msg_send_basic(LC_OP_EPR_IND, dest_id, dest_id);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER(lc_mic_disc, void)
{
    switch (ke_state_get(dest_id))
    {
    case LC_FREE:
    case LC_WAIT_DETACH_REQ_TX_CFM:
    case LC_WAIT_DISC_TO:
    case LC_WAIT_DISC_CFM:
    case LC_WAIT_DETACH_TO:
        // Message discarded in disconnection states
        break;
    default:
    {
        // Initiate detach
        lc_detach(dest_id, CO_ERROR_CONN_REJ_SECURITY_REASONS);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
}
#endif // EAVESDROPPING_SUPPORT

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(lc)
{
    // Note: all messages must be sorted in ID ascending order
    //************** Msg LC->LC****************
    {LC_LMP_RX, (ke_msg_func_t) lc_lmp_rx_handler  },
    {LC_SYNC_IND, (ke_msg_func_t)lc_sync_ind_handler},
    {LC_LMP_RSP_TO, (ke_msg_func_t)lc_lmp_rsp_to_ind_handler},
    {LC_CON_ACCEPT_TO, (ke_msg_func_t)lc_con_accept_to_handler},
    {LC_AFH_REPORT_TO, (ke_msg_func_t)lc_afh_report_to_handler},
    {LC_AUTH_PAYL_NEARLY_TO, (ke_msg_func_t)lc_auth_payl_nearly_to_handler},
    {LC_AUTH_PAYL_REAL_TO, (ke_msg_func_t)lc_auth_payl_real_to_handler},

    {LC_OP_LOC_SWITCH_REQ, (ke_msg_func_t)lc_op_loc_switch_req_handler},
    {LC_OP_LOC_SNIFF_REQ, (ke_msg_func_t)lc_op_loc_sniff_req_handler},
    {LC_OP_LOC_UNSNIFF_REQ, (ke_msg_func_t)lc_op_loc_unsniff_req_handler},

#if (MAX_NB_SYNC)
    {LC_OP_LOC_SYNC_CON_REQ, (ke_msg_func_t)lc_op_loc_sync_con_req_handler},
#endif // (MAX_NB_SYNC)

#if (RW_BT_MWS_COEX)
    {LC_SAM_SUBMAP_UPDATE_IND, (ke_msg_func_t)lc_sam_submap_update_ind_handler},
    {LC_SAM_MAP_UPDATE_IND, (ke_msg_func_t)lc_sam_map_update_ind_handler },
#endif // (RW_BT_MWS_COEX)

    //************** Msg LB->LC****************
#if BCAST_ENC_SUPPORT
    {LC_MST_KEY_REQ, (ke_msg_func_t)lc_mst_key_req_handler},
#endif // BCAST_ENC_SUPPORT
#if (PCA_SUPPORT)
    {LC_PCA_SSCAN_CLK_IND, (ke_msg_func_t)lc_pca_sscan_clk_ind_handler},
#endif // (PCA_SUPPORT)

    ///Procedures to be done before the hl_connect_req
    {LC_OP_FEAT_IND, (ke_msg_func_t)lc_op_feat_ind_handler},
    {LC_OP_VERS_IND, (ke_msg_func_t)lc_op_vers_ind_handler},
    {LC_OP_CLKOFF_IND, (ke_msg_func_t)lc_op_clkoff_ind_handler},
    {LC_OP_TIMACC_IND, (ke_msg_func_t)lc_op_timacc_ind_handler},
    {LC_OP_PT_IND, (ke_msg_func_t)lc_op_pt_ind_handler},
#if (RW_BT_MWS_COEX)
    {LC_OP_SAM_IND, (ke_msg_func_t)lc_op_sam_ind_handler},
    {LC_OP_SAM_CFM, (ke_msg_func_t)lc_op_sam_cfm_handler},
#endif // (RW_BT_MWS_COEX)
    {LC_OP_HL_CNX_IND, (ke_msg_func_t)lc_op_hl_cnx_ind_handler},
    {LC_OP_HL_CNX_CONT_IND, (ke_msg_func_t)lc_op_hl_cnx_cont_ind_handler},
    ///Procedures to be done before the setup completed
    {LC_OP_AUTH_IND, (ke_msg_func_t)lc_op_auth_ind_handler},
    {LC_OP_ENC_IND, (ke_msg_func_t)lc_op_enc_ind_handler},
    {LC_OP_EPR_IND, (ke_msg_func_t)lc_op_epr_ind_handler},
    ///Procedure setup completed
    {LC_OP_SET_CMP_IND, (ke_msg_func_t)lc_op_set_cmp_ind_handler},
    {LC_OP_NAME_REQ_IND, (ke_msg_func_t)lc_op_name_req_ind_handler},
    {LC_OP_SSRNEGO_IND, (ke_msg_func_t)lc_op_ssrnego_ind_handler},
    {LC_OP_QOS_IND, (ke_msg_func_t)lc_op_qos_ind_handler},
    {LC_OP_DETACH_IND, (ke_msg_func_t)lc_op_detach_ind_handler},

    //SP new handler
    {LC_AUTH_STG1_RANDN, (ke_msg_func_t)lc_auth_stg1_randn_handler},

    //************** Msg LD->LC****************
    {LC_LMP_RX_IND, (ke_msg_func_t) lc_lmp_rx_ind_handler  },
    {LC_LMP_TX_CFM, (ke_msg_func_t) lc_lmp_tx_cfm_handler  },
    {LC_ACL_RX_IND, (ke_msg_func_t) lc_acl_rx_ind_handler  },
    {LC_ACL_RX_MIC_ERR_IND, (ke_msg_func_t) lc_acl_rx_mic_err_ind_handler},
    {LC_ACL_TX_CFM, (ke_msg_func_t) lc_acl_tx_cfm_handler  },
    {LC_ACL_DISC_IND, (ke_msg_func_t) lc_acl_disc_ind_handler},
    {LC_RSW_END_IND, (ke_msg_func_t) lc_rsw_end_ind_handler},
    {LC_RSW_SLOT_OFFSET_IND, (ke_msg_func_t) lc_rsw_slot_offset_ind_handler},
    {LC_PWR_INCR_IND, (ke_msg_func_t) lc_pwr_incr_ind_handler},
    {LC_PWR_DECR_IND, (ke_msg_func_t) lc_pwr_decr_ind_handler},
    {LC_PWR_MAX_IND, (ke_msg_func_t) lc_pwr_max_ind_handler},
    {LC_SSR_INST_IND, (ke_msg_func_t) lc_ssr_inst_ind_handler},
    {LC_SNIFF_OFFSET_UPD_IND, (ke_msg_func_t) lc_sniff_offset_upd_ind_handler},
    {LC_SCO_OFFSET_UPD_IND, (ke_msg_func_t) lc_sco_offset_upd_ind_handler},

#if (VOICE_OVER_HCI)
    {LC_SYNC_RX_IND, (ke_msg_func_t) lc_sync_rx_ind_handler  },
    {LC_SYNC_TX_CFM, (ke_msg_func_t) lc_sync_tx_cfm_handler  },
#endif // (VOICE_OVER_HCI)

#if (PCA_SUPPORT)
#if (RW_BT_MWS_COEX)
    {LC_LOCAL_PCA_REQ, (ke_msg_func_t) lc_local_pca_req_handler },
#endif // (RW_BT_MWS_COEX)
    {LC_PCA_SSCAN_START_REQ, (ke_msg_func_t)lc_pca_sscan_start_req_handler},
    {LC_PCA_SSCAN_STOP_REQ, (ke_msg_func_t)lc_pca_sscan_stop_req_handler},
#endif // (PCA_SUPPORT)

#if (RW_BT_MWS_COEX)
    {LC_MWS_PATTERN_IND, (ke_msg_func_t)lc_mws_pattern_ind_handler },
#endif // (RW_BT_MWS_COEX)

    {LC_SAM_CHANGE_IND, (ke_msg_func_t)lc_sam_change_ind_handler },

#if (EAVESDROPPING_SUPPORT)
    {LC_SCO_DELAY_HOST_REQ, (ke_msg_func_t)lc_sco_delayed_host_req_handler},
    {LC_MIC_ENC_KEY_REFRESH, (ke_msg_func_t)lc_mic_enc_key_refresh_handler},
    {LC_MIC_DISC, (ke_msg_func_t)lc_mic_disc_handler},
#endif // EAVESDROPPING_SUPPORT

    {HCI_COMMAND, (ke_msg_func_t) hci_command_lc_handler    },
    {HCI_ACL_DATA, (ke_msg_func_t) hci_acl_data_lc_handler   },

#if (VOICE_OVER_HCI)
    {HCI_SYNC_DATA, (ke_msg_func_t) hci_sync_data_handler  },
#endif // (VOICE_OVER_HCI)
};

/// Specifies the message handlers that are common to all states.
//KE_MSG_STATE(lc)

/// Defines the place holder for the states of all the task instances.
ke_state_t lc_state[LC_IDX_MAX];

/// LLC task descriptor
const struct ke_task_desc TASK_DESC_LC = {lc_msg_handler_tab, lc_state, LC_IDX_MAX, ARRAY_LEN(lc_msg_handler_tab)};

/// @} LCTASK
