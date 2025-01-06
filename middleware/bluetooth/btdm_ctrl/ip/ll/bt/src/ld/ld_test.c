/**
****************************************************************************************
*
* @file ld_test_mode.c
*
* @brief LD test mode source code
*
* Copyright (C) RivieraWaves 2009-2021
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLTESTMODE
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration
#if (BT_HCI_TEST_MODE)

#include "ke_mem.h"
#include "ke_msg.h"              // kernel messages
#include "rwip.h"

#include "ld.h"                  // link driver API
#include "ld_int.h"              // link layer driver internal
#include "ld_util.h"             // link driver utilities

#include "lm.h"

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer

#include "reg_btcore.h"          // BT core registers
#include "reg_em_bt_cs.h"        // BT EM Control Structure
#include "reg_em_bt_txdesc.h"    // BT EM TX descriptors
#include "reg_btcore_esco.h"     // eSCO registers
//#include "reg_em_bt_rxdesc.h"    // BT EM RX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Link ID used for test mode
#define BT_TEST_LINK_ID             0

/// Duration of test mode event in slots (2 slots)
#define BT_TEST_DUR                 2

/// Test mode maximum size
#define BT_TEST_SIZE_MAX            DH5_3_PACKET_SIZE

/// Test mode event states
enum BT_TEST_EVT_STATE
{
    TEST_EVT_WAIT,
    TEST_EVT_ACTIVE,
    TEST_EVT_END,
};

/// Test mode type (Rx/Tx)
enum BT_TEST_TYPE
{
    TEST_RX,
    TEST_TX,
};

/// Test mode packet types
enum BT_TEST_PKT_TYPE
{
    DTM_DM1,
    DTM_DH1,
    DTM_DM3,
    DTM_DH3,
    DTM_DM5,
    DTM_DH5,
    DTM_2DH1,
    DTM_3DH1,
    DTM_2DH3,
    DTM_3DH3,
    DTM_2DH5,
    DTM_3DH5,
    DTM_HV1,
    DTM_HV2,
    DTM_HV3,
    DTM_EV3,
    DTM_EV4,
    DTM_EV5,
    DTM_2EV3,
    DTM_3EV3,
    DTM_2EV5,
    DTM_3EV5,
};

/// Test logical transport
enum TEST_TRANSPORT
{
    ACL_TRANSPORT,
    SCO_TRANSPORT,
    ESCO_TRANSPORT,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD test mode environment structure
struct ld_test_env_tag
{
    /// Pointer to inquiry event
    struct sch_arb_elt_tag evt;

    /// Buffer used for sending the test data
    uint16_t em_buf;

    /// Length of test data
    uint16_t data_len;

    /// Type (0: RX | 1: TX)
    uint8_t type;

    /// RF channel index: 0 -> 39 for even channels and 40 -> 79 for odd channels
    uint8_t chan_idx;

    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t payload;

    /// current state of the test mode
    uint8_t state;

    /// Logical transport (ACL/SCO/eSCO)
    uint8_t transport;

    /// Tx type
    uint8_t tx_type;

    /// Indicates whether EDR is enabled or not
    uint8_t edr;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD test mode environment variable
__STATIC struct ld_test_env_tag *ld_test_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_test_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup test environment variable
 ****************************************************************************************
 */
__STATIC void ld_test_cleanup(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_CLEANUP_BIT);

    if (ld_test_env != NULL)
    {
        // Re-enable force AGC mechanism
        rwip_rf.force_agc_enable(true);
        // Clear the Regulatory Body and RF Testing Register
        bt_rftestcntl_set(0);
        // Enable the whitening
        bt_rwbtcntl_whitdsb_setf(0);

        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ACL_INDEX(BT_TEST_LINK_ID))), REG_EM_BT_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(ld_test_env);
        ld_test_env = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Fulfills the payload for the transmit test mode.
 *
 * This function fulfills the payload for the transmit test mode.
 *
 * @param[in] pattern_type         type of the pattern.
 * @param[in] payload_len          length of the payload.
 *
 ****************************************************************************************
 */
__INLINE void ld_gen_pattern(uint8_t pattern_type, uint16_t payload_len, uint8_t *payload)
{
    uint8_t pattern = 0;

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_GEN_PATTERN_BIT, pattern_type, payload_len, payload);

    // get the pattern
    switch (pattern_type)
    {
    case PAYL_11110000:
        pattern = 0xF0;
        break;
    case PAYL_10101010:
        pattern = 0xAA;
        break;
    case PAYL_ALL_1:
        pattern = 0xFF;
        break;
    case PAYL_ALL_0:
        pattern = 0x00;
        break;
    case PAYL_00001111:
        pattern = 0x0F;
        break;
    case PAYL_01010101:
        pattern = 0x55;
        break;
    default:
        ASSERT_ERR(pattern_type <= PAYL_01010101);
        break;
    }
    // fulfill the payload
    memset(payload, pattern, payload_len);
}


/**
 ****************************************************************************************
 * @brief Select the transport type based on the selected packet type
 ****************************************************************************************
 */
__STATIC void ld_test_transport_select(uint8_t pkt_type, uint16_t data_len)
{
    bool edr = false;
    uint8_t transport_type = ACL_TRANSPORT;
    uint8_t tx_type = 0;
    uint16_t max_data_len = 0;
    uint16_t tx_data_len = data_len;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_TRANSPORT_SELECT_BIT, pkt_type, data_len);

    switch (pkt_type)
    {
    case DTM_DM1:
    {
        tx_type = DM1_TYPE;
        max_data_len = DM1_PACKET_SIZE;
    }
    break;
    case DTM_DH1:
    {
        tx_type = DH1_TYPE;
        max_data_len = DH1_PACKET_SIZE;
    }
    break;
    case DTM_DM3:
    {
        tx_type = DM3_TYPE;
        max_data_len = DM3_PACKET_SIZE;
    }
    break;
    case DTM_DH3:
    {
        tx_type = DH3_TYPE;
        max_data_len = DH3_PACKET_SIZE;
    }
    break;
    case DTM_DM5:
    {
        tx_type = DM5_TYPE;
        max_data_len = DM5_PACKET_SIZE;
    }
    break;
    case DTM_DH5:
    {
        tx_type = DH5_TYPE;
        max_data_len = DH5_PACKET_SIZE;
    }
    break;
    case DTM_2DH1:
    {
        tx_type = DH1_2_TYPE;
        edr = true;
        max_data_len = DH1_2_PACKET_SIZE;
    }
    break;
    case DTM_3DH1:
    {
        tx_type = DH1_3_TYPE;
        edr = true;
        max_data_len = DH1_3_PACKET_SIZE;
    }
    break;
    case DTM_2DH3:
    {
        tx_type = DH3_2_TYPE;
        edr = true;
        max_data_len = DH3_2_PACKET_SIZE;
    }
    break;
    case DTM_3DH3:
    {
        tx_type = DH3_3_TYPE;
        edr = true;
        max_data_len = DH3_3_PACKET_SIZE;
    }
    break;
    case DTM_2DH5:
    {
        tx_type = DH5_2_TYPE;
        edr = true;
        max_data_len = DH5_2_PACKET_SIZE;
    }
    break;
    case DTM_3DH5:
    {
        tx_type = DH5_3_TYPE;
        edr = true;
        max_data_len = DH5_3_PACKET_SIZE;
    }
    break;
    case DTM_HV1:
    {
        tx_type = HV1_TYPE;
        transport_type = SCO_TRANSPORT;
        max_data_len = HV1_PACKET_SIZE;
    }
    break;
    case DTM_HV2:
    {
        tx_type = HV2_TYPE;
        transport_type = SCO_TRANSPORT;
        max_data_len = HV2_PACKET_SIZE;
    }
    break;
    case DTM_HV3:
    {
        tx_type = HV3_TYPE;
        transport_type = SCO_TRANSPORT;
        max_data_len = HV3_PACKET_SIZE;
    }
    break;
    case DTM_EV3:
    {
        tx_type = EV3_TYPE;
        transport_type = ESCO_TRANSPORT;
        max_data_len = EV3_PACKET_SIZE;
    }
    break;
    case DTM_EV4:
    {
        tx_type = EV4_TYPE;
        transport_type = ESCO_TRANSPORT;
        max_data_len = EV4_PACKET_SIZE;
    }
    break;
    case DTM_EV5:
    {
        tx_type = EV5_TYPE;
        transport_type = ESCO_TRANSPORT;
        max_data_len = EV5_PACKET_SIZE;
    }
    break;
    case DTM_2EV3:
    {
        tx_type = EV3_2_TYPE;
        transport_type = ESCO_TRANSPORT;
        edr = true;
        max_data_len = EV3_2_PACKET_SIZE;
    }
    break;
    case DTM_3EV3:
    {
        tx_type = EV3_3_TYPE;
        transport_type = ESCO_TRANSPORT;
        edr = true;
        max_data_len = EV3_3_PACKET_SIZE;
    }
    break;
    case DTM_2EV5:
    {
        tx_type = EV5_2_TYPE;
        transport_type = ESCO_TRANSPORT;
        edr = true;
        max_data_len = EV5_2_PACKET_SIZE;
    }
    break;
    case DTM_3EV5:
    {
        tx_type = EV5_3_TYPE;
        transport_type = ESCO_TRANSPORT;
        edr = true;
        max_data_len = EV5_3_PACKET_SIZE;
    }
    break;
    default:
    {
        ASSERT_INFO(0, pkt_type, data_len);
    }
    break;
    }

    tx_data_len = co_min(data_len, max_data_len);

    {
        // Point to parameters
        struct ld_test_env_tag *test_par = ld_test_env;

        test_par->tx_type = tx_type;
        test_par->data_len = tx_data_len;
        test_par->transport = transport_type;
        test_par->edr = edr;
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_test_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_EVT_START_CBK_BIT, evt);

    ASSERT_ERR(&(ld_test_env->evt) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_test_env_tag *test_par = (struct ld_test_env_tag *) evt;
        struct sch_prog_params prog_par;
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(BT_TEST_LINK_ID);
        uint8_t frm_type = (test_par->transport == ACL_TRANSPORT) ? SCH_BT_FRAME_TYPE_NORMAL : SCH_BT_FRAME_TYPE_ESCO;

        // Push the programming to SCH PROG
        prog_par.frm_cbk        = &ld_test_frm_cbk;
        prog_par.time.hs        = evt->time.hs;
        prog_par.time.hus       = 0;
        prog_par.cs_idx         = cs_idx;
        prog_par.dummy          = cs_idx;
        prog_par.bandwidth      = evt->duration_min;
        prog_par.prio_1         = evt->current_prio;
        prog_par.prio_2         = 0;
        prog_par.prio_3         = 0;
        prog_par.pti_prio       = RW_BT_PTI_PRIO_AUTO;
        prog_par.add.bt.frm_type = frm_type;
        prog_par.add.bt.vxchan   = 0;
        prog_par.mode           = SCH_PROG_BT;
        sch_prog_push(&prog_par);

        // Move state
        test_par->state = TEST_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_test_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_EVT_CANCELED_CBK_BIT, evt);

    ASSERT_ERR(&(ld_test_env->evt) == evt);

    if (evt != NULL)
    {
        ASSERT_ERR(((struct ld_test_env_tag *) evt)->state == TEST_EVT_WAIT);

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));

        // Reschedule ASAP
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            ASSERT_ERR_FORCE(0);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle Rx interrupt
 ****************************************************************************************
 */
__STATIC void ld_test_rx_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_RX_ISR_BIT, timestamp);

    if (ld_test_env != NULL)
    {
        // Check if a packet has been received
        while (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(BT_TEST_LINK_ID)))
        {
            // Free RX descriptor
            ld_rxdesc_free();
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_test_frm_isr(uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_FRM_ISR_BIT, timestamp, abort);

    if (ld_test_env != NULL)
    {
        // Point to parameters
        struct ld_test_env_tag *test_par = ld_test_env;
        struct sch_arb_elt_tag *evt = &(ld_test_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Check test mode end
        if (test_par->state == TEST_EVT_END)
        {
            // Report test mode end to LLM
            struct lm_test_end_ind *ind = KE_MSG_ALLOC(LM_TEST_END_IND, TASK_LM, TASK_NONE, lm_test_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->nb_pkt = (test_par->type == TEST_RX) ? bt_rftestrxstat_rxpktcnt_getf() : bt_rftesttxstat_txpktcnt_getf();
            ke_msg_send(ind);

            if (test_par->type == TEST_TX)
            {
                // Release TX buffer
                bt_util_buf_acl_tx_free(test_par->em_buf);
            }

            // Free event memory
            ld_test_cleanup();
        }
        else
        {

            // update event priority
            evt->current_prio = abort
                                ? RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX))
                                : rwip_priority[RWIP_PRIO_ACL_DFT_IDX].value;

            // Reschedule ASAP
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, 0, 0, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));

            // Try to reschedule
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                test_par->state = TEST_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_test_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_FRM_CBK_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    {
        ld_test_frm_isr(timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_test_frm_isr(timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        ld_test_rx_isr(timestamp);
    }
    break;
    default:
    {
        ASSERT_INFO(0, dummy, irq_type);
    }
    break;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

uint8_t ld_test_start(struct ld_test_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_START_BIT, uint8_t, params);

    // Check if test mode is inactive
    if (ld_test_env == NULL)
    {
        // Allocate event
        ld_test_env = LD_ALLOC_EVT(ld_test_env_tag);

        if (ld_test_env != NULL)
        {
            // Point to parameters
            struct ld_test_env_tag *test_par = ld_test_env;
            struct sch_arb_elt_tag *evt = &(ld_test_env->evt);

            uint32_t clock = ld_read_clock();
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(BT_TEST_LINK_ID);

            LD_INIT_EVT(evt, ld_test_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_test_evt_canceled_cbk;
            evt->cb_start         = &ld_test_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio     = rwip_priority[RWIP_PRIO_ACL_DFT_IDX].value;
            evt->duration_min     = 2 * BT_TEST_DUR * HALF_SLOT_SIZE;
            evt->time.hus         = 0;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_PHASE_0, 0, RWIP_PRIO_INC(RWIP_PRIO_ACL_DFT_IDX));

            // Initialize event parameters (test mode part)
            test_par->type = params->type;

            // Convert channel (0 -> 79) into frequency table index: 0 -> 39 for even channels and 40 -> 79 for odd channels
            test_par->chan_idx = (params->channel & 0x1) ? (params->channel >> 1) + 40 : params->channel >> 1;

            // Configure testing registers
            bt_rftestcntl_pack(/*infiniterx*/ 0, /*rxpktcnten*/ 1, /*percountmode*/ 0, /*sserrren*/ 0, /*herrren*/ 0, /*infinitetx*/ 0, /*prbstype*/ 0, /*txpldsrc*/ 0, /*txpktcnten*/ 1);
            bt_rftestfreq_pack(/*loopbackmode*/ 0, /*directloopbacken*/ 0, /*testmodeen*/ 1, /*rxfreq*/ test_par->chan_idx, /*txfreq*/ test_par->chan_idx);

            // Prepare CS
            //em_bt_txrxcntl_nullflt_setf(cs_idx, 1);
            //em_bt_loopcntl_att_nb_setf(cs_idx, 1);
            //em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ACL_INDEX(TEST_LINK_ID, 0)));
            //em_bt_rxmaxbuf_bch2_rxmaxbuf_setf(cs_idx, ACL_DATA_BUF_SIZE);

            // Set CS format
            {
                uint8_t frame_format = (params->type == TEST_RX) ? EM_BT_CS_FMT_DTM_RX : EM_BT_CS_FMT_DTM_TX;
                em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(CONNECT, TXBSY), RWIP_COEX_GET(CONNECT, RXBSY), RWIP_COEX_GET(CONNECT, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, frame_format);
            }

            // Set BD address and BCH in the CS
            {
                /// BD Address for test mode
                struct bd_addr bd_addr = {{0x0F, 0xF0, 0x0C, 0xC0, 0x03, 0x30}};
                /// BCH for test mode
                uint8_t bch[LD_BCH_SIZE];

                // Compute the BCH for test mode BD address
                ld_util_bch_create(&bd_addr.addr[0], &bch[0]);

                em_bt_bdaddr_setf(cs_idx, 0, (bd_addr.addr[1] << 8) | bd_addr.addr[0]);
                em_bt_bdaddr_setf(cs_idx, 1, (bd_addr.addr[3] << 8) | bd_addr.addr[2]);
                em_bt_bdaddr_setf(cs_idx, 2, (bd_addr.addr[5] << 8) | bd_addr.addr[4]);
                em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
                em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
                em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, bch[4] & EM_BT_BCH2_MASK);
            }

            em_bt_txrxcntl_pack(cs_idx, 0,
#if (EAVESDROPPING_SUPPORT)
                                0,
#endif // EAVESDROPPING_SUPPORT
                                0, 0, 0, 0, 0, 0, 0, 0, 0);
            em_bt_acltxstat_pack(cs_idx, /*txtog*/ 0, /*txflush*/ 0, /*fcrc*/ 0, /*fnak*/ 0, /*rswitch*/ 0, /*waitack*/ 0, /*lastnull*/ 0, /*lasttxack*/ 0, /*lasttxseqn*/ 0);
            em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);
            em_bt_clkoff0_setf(cs_idx, 0);
            em_bt_clkoff1_setf(cs_idx, 0);
            em_bt_loopcntl_pack(cs_idx, 1, 1);

            // Pre-initialization of conditionally set fields
            // TODO: restore
            //em_bt_crcinit1_set(cs_idx, 0);
            //em_bt_rxdfantpattcntl_set(cs_idx, 0);

            // Select logical transport and frame type
            ld_test_transport_select(params->pkt_type, params->data_len);

            if (test_par->transport == SCO_TRANSPORT)
            {
                // Configure SCO Logical Transport
                bt_e_scoltcntl_pack(/*elt_idx*/ BT_TEST_LINK_ID, /*retxnb*/ 1, /*escoedrrx*/ 0, /*escoedrtx*/ 0, /*syntype*/ 0, /*synltaddr*/ 0);

                // Initialize and configure reception and transmission
                bt_e_scotrcntl_pack(/*elt_idx*/ BT_TEST_LINK_ID, /*txseqn*/ 0, /*txlen*/ test_par->data_len, /*txtype*/ test_par->tx_type, /*rxlen*/ test_par->data_len, /*rxtype*/ test_par->tx_type);
            }
            else if (test_par->transport == ESCO_TRANSPORT)
            {
                // Configure eSCO Logical Transport
                bt_e_scoltcntl_pack(/*elt_idx*/ BT_TEST_LINK_ID, /*retxnb*/ 1, /*escoedrrx*/ test_par->edr, /*escoedrtx*/ test_par->edr, /*syntype*/ 1, /*synltaddr*/ 0);

                // Initialize and configure reception and transmission
                bt_e_scotrcntl_pack(/*elt_idx*/ BT_TEST_LINK_ID, /*txseqn*/ 0, /*txlen*/ test_par->data_len, /*txtype*/ test_par->tx_type, /*rxlen*/ test_par->data_len, /*rxtype*/ test_par->tx_type);
            }
            else // ACL_TRANSPORT
            {
                // Configure HW to new data rate
                em_bt_linkcntl_pack(cs_idx,
                                    /*aknena*/ 0,
                                    /*afhena*/ 0,
                                    /*laap*/ 0,
                                    /*whdsb*/ 1,
                                    /*acledr*/ test_par->edr,
                                    /*aclltaddr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                                    /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                    /*linklbl*/ cs_idx);
            }

            // Check the type of the test (Rx/Tx)
            if (params->type == TEST_RX)
            {
                //disable the whitening
                bt_rwbtcntl_whitdsb_setf(1);

                // Set the RF test control register
                // Rx packet count enabled, and reported in CS-RXCCMPKTCNT and RFTESTRXSTAT-RXPKTCNT on RF abort command
                bt_rftestcntl_rxpktcnten_setf(1);
                // Clear counter dedicated for the test mode
                em_bt_rxccmpldcnt0_set(cs_idx, 0);
                em_bt_rxccmpldcnt1_set(cs_idx, 0);
                em_bt_rxccmpldcnt2_set(cs_idx, 0);

                // set Wide-open mode, Size of the Rx window in slots
                em_bt_wincntl_pack(cs_idx, 1, ((75 + 1) >> 1));

                // extend length to TEST_SIZE_MAX
                //em_bt_rxmaxbuf_set(cs_idx, TEST_SIZE_MAX);
                //em_bt_rxmaxtime_set(cs_idx, 0);

                // Disable unused control
                em_bt_txrxcntl_set(cs_idx, 0);

                // Disable frequency hopping
                em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 0, /*freq*/ test_par->chan_idx, /*txpwr*/ 0);

                // Disable force AGC mechanism
                rwip_rf.force_agc_enable(false);
            }
            else if (params->type == TEST_TX)
            {
                test_par->data_len = params->data_len;
                test_par->payload = params->payload;

                // System ram buffer to be filled
                uint8_t *data = ke_malloc_system(BT_TEST_SIZE_MAX, KE_MEM_KE_MSG);
                // Exchange memory buffer to be filled
                uint16_t buf_ptr = bt_util_buf_acl_tx_alloc();
                uint8_t txdesc_idx = EM_BT_TXDESC_ACL_INDEX(BT_TEST_LINK_ID, 0);

                if (buf_ptr != 0)
                {
                    // Save test buffer pointer
                    test_par->em_buf = buf_ptr;

                    // Prepare Tx descriptor
                    em_bt_txheader_txtype_setf(txdesc_idx, ((params->type == TEST_TX) ? POLL_TYPE : ID_NUL_TYPE));
                    em_bt_txheader_txltaddr_setf(txdesc_idx, 0);
                    em_bt_txpheader_txlength_setf(txdesc_idx, 0);
                    em_bt_txaclbufptr_setf(txdesc_idx, test_par->em_buf);
                    em_bt_txlmbufptr_setf(txdesc_idx, 0);
                    em_bt_txptr_pack(txdesc_idx, 1, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ACL_INDEX(BT_TEST_LINK_ID, 0)));

                    // Check new ACL data to send
                    if (test_par->data_len != 0)
                    {
                        // Fill descriptor
                        em_bt_txheader_txtype_setf(txdesc_idx, test_par->tx_type);
                        em_bt_txpheader_pack(txdesc_idx, 0, test_par->data_len, 1, LLID_START);

                        // Release descriptor
                        em_bt_txptr_txdone_setf(txdesc_idx, 0);
                    }

                    //disable the whitening
                    bt_rwbtcntl_whitdsb_setf(1);

                    // Set the RF test control register
                    // Tx packet count enabled, and reported in CS-TXCCMPKTCNT and RFTESTRXSTAT-TXPKTCNT on RF abort command
                    bt_rftestcntl_txpktcnten_setf(1);
                    // Clear counter dedicated for the test mode
                    em_bt_txccmpldcnt0_set(cs_idx, 0);
                    em_bt_txccmpldcnt1_set(cs_idx, 0);
                    em_bt_txccmpldcnt2_set(cs_idx, 0);

                    //check the type of test
                    switch (params->payload)
                    {
                    case PAYL_PSEUDO_RAND_9:
                    case PAYL_PSEUDO_RAND_15:
                        // sets the type of the PRBS
                        bt_rftestcntl_prbstype_setf(params->payload & 0x1);
                        // sets the source to PRBS generator
                        bt_rftestcntl_txpldsrc_setf(1);
                        break;
                    case PAYL_11110000:
                    case PAYL_10101010:
                    case PAYL_ALL_1:
                    case PAYL_ALL_0:
                    case PAYL_00001111:
                    case PAYL_01010101:
                        ld_gen_pattern(params->payload, params->data_len, data);
                        em_wr((void *)&data, test_par->em_buf, params->data_len);
                        // sets the source to CS
                        bt_rftestcntl_txpldsrc_setf(0);
                        break;
                    default:
                        ASSERT_ERR(params->payload <= PAYL_01010101);
                        break;
                    }
                    ke_free(data);
                }
                else
                {
                    ke_free(data);
                    // clean-up allocated memory
                    ld_test_cleanup();

                    return (CO_ERROR_MEMORY_CAPA_EXCEED);
                }

                // Convert dBm into HW TX power settings depending on the radio
                {
                    uint8_t tx_pwr;
                    uint8_t rf_mod;

                    switch (params->tx_pwr_lvl)
                    {
                    case MIN_TX_PWR_LVL:
                        // Minimum Tx power
                        tx_pwr =  rwip_rf.txpwr_min;
                        rf_mod =  rwip_rf.txpwr_min_mod;
                        break;
                    case MAX_TX_PWR_LVL:
                        // Maximum Tx power
                        tx_pwr =  rwip_rf.txpwr_max;
                        rf_mod =  rwip_rf.txpwr_max_mod;
                        break;
                    default:
                        // Selected Tx power level
                        rf_mod = rwip_rf.txpwr_mdlt_get(params->tx_pwr_lvl - BT_DFT_TX_PATH_RF_COMPENSATION / 10, test_par->edr);
                        tx_pwr = rwip_rf.txpwr_cs_get(params->tx_pwr_lvl - BT_DFT_TX_PATH_RF_COMPENSATION / 10, TXPWR_CS_NEAREST, rf_mod);
                        break;
                    }

                    em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 0, /*freq*/ test_par->chan_idx, /*txpwr*/ tx_pwr);
                    rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rf_mod, tx_pwr);
                }

                // Set the Tx descriptor pointer in the CS
                em_bt_txdescptr_set(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, txdesc_idx));
            }
            else
            {
                ASSERT_ERR(0);
            }

            // Disable unused control
            em_bt_chmap0_set(cs_idx, 0);
            em_bt_chmap1_set(cs_idx, 0);
            em_bt_chmap2_set(cs_idx, 0);
            //em_bt_minevtime_set(cs_idx, 0);
            //em_bt_maxevtime_set(cs_idx, 0);

            // Initialize packet counter
            em_bt_rxccmpldcnt0_set(cs_idx, 0);
            em_bt_rxccmpldcnt1_set(cs_idx, 0);
            em_bt_rxccmpldcnt2_set(cs_idx, 0);

            // Schedule event ASAP
            evt->time.hs = clock;

            GLOBAL_INT_DISABLE();

            //if (1)
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                test_par->state = TEST_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }

            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR(0);
        }
    }

    return (status);
}

uint8_t ld_test_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_STOP_BIT, uint8_t);

    GLOBAL_INT_DISABLE();

    if (ld_test_env != NULL)
    {
        // Point to parameters
        struct ld_test_env_tag *test_par = ld_test_env;
        struct sch_arb_elt_tag *evt = &(ld_test_env->evt);

        switch (test_par->state)
        {
        case TEST_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            if (test_par->type == TEST_TX)
            {
                // Release TX buffer
                bt_util_buf_acl_tx_free(test_par->em_buf);
            }

            // Report test mode end to LLM
            struct lm_test_end_ind *ind = KE_MSG_ALLOC(LM_TEST_END_IND, TASK_LM, TASK_NONE, lm_test_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->nb_pkt = (test_par->type == TEST_RX) ? bt_rftestrxstat_rxpktcnt_getf() : bt_rftesttxstat_txpktcnt_getf();
            ke_msg_send(ind);

            // Free event memory
            ld_test_cleanup();
        }
        break;

        case TEST_EVT_ACTIVE:
        {
            // Abort the event
            bt_rwbtcntl_set(bt_rwbtcntl_get() | BT_RFTEST_ABORT_BIT);

            // Move state
            test_par->state = TEST_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}


void ld_test_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_INIT_BIT);

    ld_test_env = NULL;
}

void ld_test_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_TEST_PATCH_TYPE, LD_TEST_RESET_BIT);

    // clean-up allocated memory
    ld_test_cleanup();
}

#endif // (BT_HCI_TEST_MODE)
///@} LDTESTMODE
