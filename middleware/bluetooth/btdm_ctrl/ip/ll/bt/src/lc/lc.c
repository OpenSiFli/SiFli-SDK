/**
 ****************************************************************************************
 *
 * @file lc.c
 *
 * @brief Definition of the functions used by the link controller
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "co_bt.h"          // BT standard definitions
#include "co_math.h"
#include "co_endian.h"
#include "co_utils.h"

#include "ke_mem.h"         // kernel memory
#include "ke_timer.h"         // kernel memory

#include "lc.h"          // link controller definitions
#include "lc_int.h"      // link controller internal definitions
#include "lc_util.h"     // link controller utilities definitions
#include "lc_sniff.h"
#include "lc_sco.h"
#include "lm.h"
#include "lb.h"
#include "hci.h"
#include "lc_lmppdu.h"           // LMP PDU
#include "lm_sco.h"
#include "bt_util_key.h"
#include "ecc_p256.h"
#include "ecc_p192.h"

#include "ld.h"          // link driver definitions

#include "dbg.h"

#include "sha.h"

#if (EAVESDROPPING_SUPPORT)
    #include "ed.h"
    #include "../lm/lm_int.h"
#endif // EAVESDROPPING_SUPPORT
/*
 * DEFINES
 ****************************************************************************************
 */

/// Minimum AFH instant delay
#define AFH_T_HS_MIN            (96 + 4)


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LC environment variable
struct lc_env_tag *lc_env[MAX_NB_ACTIVE_ACL];

/// LC task
//static const struct ke_task_desc TASK_DESC_LC = {NULL, &lc_default_handler, lc_state, 0, LC_IDX_MAX};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC void lc_locepr_lkexc(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_PAUSE_ENCRYPT_BIT_POS))
    {
        ld_acl_flow_off(idx);

        lc_start_lmp_to(pid);
        if (lc_env_ptr->link.Initiator)
        {
            if (lc_env_ptr->sp.sec_con)
            {
                // Send LMP_PauseEncryptionAesReq
                lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);
                lc_send_pdu_pause_enc_aes_req(idx, lc_env_ptr->link.Role, &lc_env_ptr->sp.LocRandN);
            }
            else
            {
                // Send LMP_PauseEncryptionReq
                lc_send_pdu_paus_enc_req(idx, lc_env_ptr->link.Role);
            }

            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
            }
            else
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT);
            }
        }
        else
        {
            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP);
            }
            else
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_PAUSE_REQ_MST_RSP);
            }
        }
    }
    else
    {
        lc_env_ptr->epr.on = false;
        lc_env_ptr->epr.cclk = false;
        //lc_env_ptr->req.LocKeyExchangeReq = false;
        // change connection link key event
        lc_epr_change_lk(pid, CO_ERROR_NO_ERROR);
    }
}

__STATIC void lc_ptt_cont(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Stop the flow until the Tx confirmation of the LMP_AcceptedExt PDU
    ld_acl_flow_off(idx);

    // LMP_AcceptedExt (LMP_ESC4 LMP_PKT_TYPE_TBL_REQ_EXTOPCODE rxpkttypeID)
    lc_send_pdu_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, lc_env_ptr->link.RxPktTypeID);

    ke_state_set(pid, LC_WAIT_PTT_ACC_TX_CFM);
}

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void lc_util_set_loc_trans_coll(uint8_t LinkId, uint8_t Opcode, uint8_t OpcodeExt, uint8_t Mode)
{
    ASSERT_INFO(lc_env[LinkId] != NULL, LinkId, Opcode);
    lc_env[LinkId]->local_trans_details.Opcode = Opcode;
    lc_env[LinkId]->local_trans_details.OpcodeExt = OpcodeExt;
    lc_env[LinkId]->local_trans_details.InUse = Mode;
}

/// Send HCI CS event
void lc_cmd_stat_send(uint16_t opcode, uint8_t status)
{
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    evt->status = status;
    hci_send_2_host(evt);
}

void lc_sam_disable(uint8_t link_id)
{
    struct lc_sam_tag *sam_info = &lc_env[link_id]->sam_info;

    if ((sam_info->loc_idx != SAM_DISABLED) || (sam_info->rem_idx != SAM_DISABLED))
    {
        /*
        * Send status change event only. SAM is inhibited at driver level for role switch, and simpler
        * to defer explicily disabling SAM at driver level until after succesful role switch.
        */
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + link_id);
        struct hci_sam_status_change_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SAM_STATUS_CHANGE_EVT_CODE, hci_sam_status_change_evt);

        evt->conhdl  = conhdl;
        evt->rem_idx = SAM_DISABLED;
        evt->loc_idx = SAM_DISABLED;
        evt->rem_tx_av = 0xFF;
        evt->rem_rx_av = 0xFF;
        evt->loc_tx_av = 0xFF;
        evt->loc_rx_av = 0xFF;

        hci_send_2_host(evt);
    }
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void lc_init(void)
{
    memset(&lc_env, 0, sizeof(lc_env));

    // Create LC task
    ke_task_create(TASK_LC, &TASK_DESC_LC);

    // Initialize LC states
    for (uint8_t link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
    {
        lc_state[link_id] = LC_FREE;
    }

    // Initialize Sniff
    lc_sniff_init(false);

#if (MAX_NB_SYNC > 0)
    // Initialize SCO
    lc_sco_init();
    lm_init_sync();
#endif //(MAX_NB_SYNC > 0)
}

void lc_reset(void)
{
    // Clean LC environment
    for (uint8_t link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
    {
        if (lc_env[link_id] != NULL)
        {
            // Check Simple Pairing data
            if (lc_env[link_id]->sp.p_data != NULL)
            {
                // Free the memory allocated
                ke_free(lc_env[link_id]->sp.p_data);
            }

            // Free the memory allocated
            ke_free(lc_env[link_id]);

            // Clear pointer
            lc_env[link_id] = NULL;
        }

        // Clear state
        lc_state[link_id] = LC_FREE;
    }

    // Reset Sniff
    lc_sniff_init(true);

#if (MAX_NB_SYNC > 0)
    // Reset SCO
    lc_sco_reset();
    lm_reset_sync();
#endif //(MAX_NB_SYNC > 0)
}

void lc_start(uint8_t link_id, struct lc_init_parameters *con_params)
{
    ASSERT_ERR(link_id < MAX_NB_ACTIVE_ACL);
    ASSERT_INFO(lc_state[link_id] == LC_FREE, link_id, lc_state[link_id]);

    // Allocate an environment
    lc_env[link_id] = ke_malloc_system(sizeof(struct lc_env_tag), KE_MEM_ENV);

    ASSERT_ERR(lc_env[link_id] != NULL);

    if (lc_env[link_id] != NULL)
    {
        ke_task_id_t tid = KE_BUILD_ID(TASK_LC, link_id);

        // Initialize environment
        memset(lc_env[link_id], 0, sizeof(struct lc_env_tag));

        struct lc_env_tag *lc_env_ptr = lc_env[link_id];
        // Initialize default values
        // link structure init
        lc_env_ptr->link.CurrentMode        = LM_ACTIVE_MODE;
        lc_env_ptr->link.LinkPolicySettings = con_params->link_pol_stg;
        lc_env_ptr->link.Initiator          = true;
        lc_env_ptr->link.AclPacketType      = PACKET_TYPE_DM1_FLAG | PACKET_TYPE_DH5_FLAG |
                                              PACKET_TYPE_DH1_FLAG | PACKET_TYPE_DM3_FLAG |
                                              PACKET_TYPE_DH3_FLAG | PACKET_TYPE_DM5_FLAG;
        lc_env_ptr->link.TxMaxSlotCur       = 0x01;
        lc_env_ptr->link.CurPacketTypeTable = PACKET_TABLE_1MBPS;
        lc_env_ptr->link.ServiceType        = QOS_BEST_EFFORT;
        lc_env_ptr->link.FailedContact = 0;
        lc_env_ptr->link.auth_payl_to       = AUTH_PAYL_TO_DFT;
        lc_env_ptr->link.auth_payl_to_margin = co_slot_to_duration(4 * POLL_INTERVAL_DFT);

        lc_env_ptr->sam_info.rem_idx = SAM_DISABLED;
        lc_env_ptr->sam_info.loc_idx = SAM_DISABLED;
        lc_env_ptr->sam_info.rem_tx_av = 0xFF;
        lc_env_ptr->sam_info.rem_rx_av = 0xFF;
        lc_env_ptr->sam_info.loc_tx_av = 0xFF;
        lc_env_ptr->sam_info.loc_rx_av = 0xFF;

#if (BT_53)
        lc_env_ptr->enc.min_enc_key_size = con_params->min_enc_key_size;
#if (RW_DEBUG)
        lc_env_ptr->enc.max_enc_key_size = con_params->max_enc_key_size;
#endif // (RW_DEBUG)
#endif // (BT_53)

        // Copy local BD Address
        ld_bd_addr_get(&lc_env_ptr->info.LocalBdAddr);

        // Initialize channel map
        memset(&lc_env_ptr->afh.ch_map.map[0], 0xFF, BT_CH_MAP_LEN);
        lc_env_ptr->afh.ch_map.map[BT_CH_MAP_LEN - 1] &= ~(0x80);

#if PCA_SUPPORT
        // For all devices that may use coarse clock adjustment recovery mode, the RF channel indices used for
        //  the synchronization train shall be marked as unused in the AFH_channel_map for logical links LM 4.1.14.1
        lc_env_ptr->afh.ch_map.map[SYNC_TRAIN_CHANNEL_0 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_0 & 0x7));
        lc_env_ptr->afh.ch_map.map[SYNC_TRAIN_CHANNEL_1 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_1 & 0x7));
        lc_env_ptr->afh.ch_map.map[SYNC_TRAIN_CHANNEL_2 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_2 & 0x7));
#endif // PCA_SUPPORT

        // Save peer's BD Address
        memcpy(&lc_env_ptr->info.BdAddr.addr[0], &con_params->bd_addr.addr[0], BD_ADDR_LEN);

        switch (con_params->reason)
        {
        case LC_MASTER_CON:
        {
            lc_env_ptr->link.Role = MASTER_ROLE;
            lc_env_ptr->link.AclPacketType = con_params->acl_pkt_type;
            lc_env_ptr->link.AllowRoleSwitch = con_params->role_switch_en;
            lc_env_ptr->link.LinkTimeout = LSTO_DFT;
            lc_env_ptr->link.PollInterval = POLL_INTERVAL_DFT;

            lc_start_lmp_to(tid);
            lc_env_ptr->link.HostConnected  = true;

            //Request the remote and extended peer features
            ke_msg_send_basic(LC_OP_FEAT_IND, tid, tid);
            //Request the Version of the peer
            ke_msg_send_basic(LC_OP_VERS_IND, tid, tid);
            //Request the host connection request
            ke_msg_send_basic(LC_OP_HL_CNX_IND, tid, tid);
        }
        break;
        case LC_SLAVE_CON:
        {
            lc_env_ptr->link.Role = SLAVE_ROLE;
            memcpy(&lc_env_ptr->link.Class.A[0], &con_params->class_of_dev.A[0], DEV_CLASS_LEN);
            lc_env_ptr->link.LinkTimeout = LSTO_DFT;
            lc_env_ptr->link.PollInterval = POLL_INTERVAL_DFT;
        }
        break;
        case LC_REM_NAME:
        {
            lc_env_ptr->link.Role = MASTER_ROLE;
            lc_env_ptr->link.AclPacketType = con_params->acl_pkt_type;
            lc_env_ptr->link.AllowRoleSwitch = con_params->role_switch_en;
            lc_env_ptr->link.LinkTimeout = LSTO_DFT;
            lc_env_ptr->link.PollInterval = POLL_INTERVAL_DFT;

            lc_env_ptr->req.LocNameReq = true;

            //Request the remote and extended peer features
            ke_msg_send_basic(LC_OP_FEAT_IND, tid, tid);
            //Request the remote name request
            ke_msg_send_basic(LC_OP_NAME_REQ_IND, tid, tid);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, con_params->reason, link_id);
        }
        break;
        }

        // Start ACL Baseband link
        ld_acl_start(link_id, lc_env_ptr->link.Role, con_params->slave_timing_info_ptr);

#if EAVESDROPPING_SUPPORT
        // ACL connection changed
        {
            // Allocate indication message
            struct ed_acl_con_chg_ind *ind = KE_MSG_ALLOC(ED_ACL_CON_CHG_IND, TASK_ED, TASK_NONE, ed_acl_con_chg_ind);

            // Fill data
            memcpy(&ind->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
            ind->link_id  = link_id;
            ind->status = ED_STATUS_CONNECTED;
            ind->role = lc_env_ptr->link.Role;

            // Send message
            ke_msg_send(ind);
        }
#endif // EAVESDROPPING_SUPPORT

        // Initialize LC state
        ke_state_set(tid, LC_CONNECTED);
    }
}

void lc_stop(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_env_ptr->link.Reason = reason;
    // Release the link, send HCI disconnection complete
    lc_release(pid);
}

void lc_ch_map_update(uint8_t link_id, struct bt_ch_map *ch_map)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    // Compute the channel map (peer classification)
    for (int i = 0 ; i < AFH_NB_CHANNEL_MAX ; i++)
    {
        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;

        if ((((lc_env_ptr->afh.peer_ch_class.map[byte_idx]) >> (bit_pos & 0x06)) & 0x3) == AFH_CH_CLASS_BAD)
        {
            ch_map->map[byte_idx] &= ~(1 << bit_pos);
        }
    }

    // If the channel map does not contain enough enabled channel, activate channels in order to have
    // a minimum of AFH_NB_CHANNEL_MIN channels activated.
    lm_fill_ch_map(ch_map);

    // Check if the map if different from the preceding one
    if (memcmp(&lc_env_ptr->afh.ch_map.map[0], &ch_map->map[0], BT_CH_MAP_LEN))
    {
        // Compute the Hopping Scheme Switch Instant (HSSI)
        uint32_t instant;

        // Check if link is in sniff or active mode
        if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
        {
            uint16_t interval = 0;
            uint8_t max_subrate = 0;

            lc_sniff_interval_get(link_id, &interval, &max_subrate);

            // Use 6 sniff intervals after sniff subrate
            instant = (6 + max_subrate) * interval;
        }
        else
        {
            // Use 6 polling intervals
            instant = 6 * ld_acl_t_poll_get(link_id);
        }

        // Switch instant must be at least 96 slot in the future
        instant = co_max(instant, AFH_T_HS_MIN);

        // Instant is an absolute time => add master clock
        instant = CLK_ADD_3(2 * instant, ld_read_clock(), ld_acl_clock_offset_get(link_id).hs) >> 1;

        // Ensure instant is an even master clock value
        instant = CO_ALIGN2_HI(instant);

        // Try to prepare a new map update to LD
        if (ld_acl_afh_prepare(link_id, instant, AFH_ENABLED, ch_map) == CO_ERROR_NO_ERROR)
        {
            DBG_SWDIAG(AFH, LMP_TX, 1);

            // Store new map
            memcpy(&lc_env_ptr->afh.ch_map.map[0], &ch_map->map[0], BT_CH_MAP_LEN);

            // Send LMP_SetAFH
            lc_send_pdu_set_afh(link_id, instant, AFH_ENABLED, lc_env_ptr->link.Role);
            lc_env_ptr->afh.en = true;

            DBG_SWDIAG(AFH, LMP_TX, 0);
        }
    }
}

uint16_t lc_sam_intv_get(uint8_t link_id)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    // If both peer & local SAM maps active, return the larger t_sam (should be equal or multiples to coex)
    uint16_t t_sam = co_max(lc_env_ptr->sam_info.loc_t_sam_av, lc_env_ptr->sam_info.rem_t_sam_av);

    return t_sam;
}

uint16_t lc_sam_loc_offset_get(uint8_t link_id)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    uint16_t t_sam = lc_env_ptr->sam_info.loc_t_sam_av;
    uint32_t sam_instant_clk;
    uint16_t d_sam;

#if RW_BT_MWS_COEX
    // SAM instant is MWS Frame Sync timing relative to the BT piconet clock.
    rwip_time_t sam_ts = ld_pca_ext_frame_ts_get();
    if (SLAVE_ROLE == lc_env_ptr->link.Role)
    {
        rwip_time_t clk_offset = ld_acl_clock_offset_get(link_id);
        sam_instant_clk = CLK_ADD_3(sam_ts.hs, clk_offset.hs, (sam_ts.hus + (HALF_SLOT_SIZE - clk_offset.hus) / HALF_SLOT_SIZE));
    }
    else
    {
        sam_instant_clk = sam_ts.hs;
    }
#else // !RW_BT_MWS_COEX
    sam_instant_clk = ld_acl_sam_ts_get(link_id);
#endif // RW_BT_MWS_COEX

    // The sam_instant_clk is the start of the next mws frame - use to calculate offset
    d_sam = CO_MOD(((sam_instant_clk & (BT_CLOCK_MSB - 1)) >> 1), t_sam);

    return d_sam;
}

uint16_t lc_sam_rem_offset_get(uint8_t link_id)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    uint16_t t_sam = lc_env_ptr->sam_info.rem_t_sam_av;
    uint32_t sam_instant_clk = ld_acl_sam_ts_get(link_id);
    uint16_t d_sam;

    // The sam_instant_clk is the start of the next mws frame - use to calculate offset
    d_sam = CO_MOD(((sam_instant_clk & (BT_CLOCK_MSB - 1)) >> 1), t_sam);

    return d_sam;
}

uint16_t lc_sam_slot_av_get(uint8_t link_id, uint16_t offset_min, uint16_t offset_max)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];
    uint8_t *sam_pattern = NULL;
    uint8_t *submap0 = NULL;
    uint16_t t_sam = 0;
    uint16_t d_sam = 0;
    uint8_t t_sam_sm = 0;

    uint16_t slot_idx;
    uint8_t val;

    bool loc_offset_ok = (lc_env_ptr->sam_info.loc_idx == SAM_DISABLED);
    bool rem_offset_ok = (lc_env_ptr->sam_info.rem_idx == SAM_DISABLED);
    bool *p_offset_ok = &loc_offset_ok;

    uint16_t offset = offset_min;

    do
    {
        // if local map is enabled, check for slot available in the active local map
        if (!loc_offset_ok)
        {
            struct lc_sam_tag *sam_info = &lc_env_ptr->sam_info;

            sam_pattern = lm_sam_pattern_get(sam_info->loc_idx);
            submap0 = lm_sam_submap0_get();
            t_sam = sam_info->loc_t_sam_av;
            t_sam_sm = sam_info->t_sam_sm;
            d_sam = lc_sam_loc_offset_get(link_id);
            p_offset_ok = &loc_offset_ok;
        }
        // if remote map is enabled, check for slot available in the active remote map
        else if (!rem_offset_ok)
        {
            struct lc_sam_tag *sam_info = &lc_env_ptr->sam_info;
            struct lc_sam_pattern *sam_pt_info = &lc_env_ptr->sam_info.rem_pattern[lc_env_ptr->sam_info.rem_idx];

            sam_pattern = &sam_pt_info->submaps.map[0];
            submap0 = &sam_info->rem_submap.map[0];
            t_sam = sam_info->rem_t_sam_av;
            t_sam_sm = sam_pt_info->t_sam_sm;
            d_sam = lc_sam_rem_offset_get(link_id);
            p_offset_ok = &rem_offset_ok;
        }

        while (offset <= offset_max)
        {
            // check corresponding slot in sam pattern, and submap where applicable
            slot_idx = CO_MOD(t_sam + offset - d_sam, t_sam);
            val = sam_pattern[(slot_idx / t_sam_sm) >> 2] >> (((slot_idx / t_sam_sm) & 0x3) << 1);

            if (SAM_SLOTS_AVAILABLE == val)
            {
                *p_offset_ok = true;
                break;
            }
            else if (SAM_SLOTS_SUBMAPPED == val)
            {
                // submapped, so check individual slot availability
                val = submap0[CO_MOD(slot_idx, t_sam_sm) >> 2] >> ((CO_MOD(slot_idx, t_sam_sm) & 0x3) << 1);

                if ((SAM_SLOT_TX_RX_AVAILABLE == val) || (SAM_SLOT_TX_AVAILABLE == val))
                {
                    *p_offset_ok = true;
                    break;
                }
            }

            // if offset not suitable, progress to check next transmit slot
            if (!*p_offset_ok)
            {
                offset += 2;
            }
        }

    }
    while ((!loc_offset_ok || !rem_offset_ok) && (offset <= offset_max));

    // suitable offset not found, default to offset_min.
    if (offset > offset_max)
    {
        offset = offset_min;
    }

    return offset;
}

void lc_auth_cmp(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if ((reason == CO_ERROR_NO_ERROR) || ((reason != CO_ERROR_NO_ERROR) && (lc_env_ptr->link.SetupComplete)))
    {
        if (reason == CO_ERROR_NO_ERROR)
        {
            lc_env_ptr->enc.LinkKeyValid = true;
        }

        if (lc_env_ptr->req.LocAuthReq && lc_conn_seq_done(pid))
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_auth_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_CMP_EVT_CODE, hci_auth_cmp_evt);
            evt->status = reason;
            evt->conhdl = conhdl;
            hci_send_2_host(evt);
        }
        // reset local authentication request
        lc_env_ptr->req.LocAuthReq = false;
        lc_env_ptr->link.Initiator = false;
        lc_env_ptr->req.PeerAuthReq = false;

        switch (reason)
        {
        case CO_ERROR_NO_ERROR:
        {
            if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
            {
                lc_env_ptr->epr.on = true;
                lc_env_ptr->req.LocEncKeyRefresh = true;
                lc_env_ptr->link.Initiator = true;
            }

            if (lc_env_ptr->req.LocEncReq == true)
            {
                ke_msg_send_basic(LC_OP_ENC_IND, pid, pid);
            }
            else if (lc_env_ptr->req.LocEncKeyRefresh == true)
            {
                ke_msg_send_basic(LC_OP_EPR_IND, pid, pid);
            }
            ke_state_set(pid, LC_CONNECTED);
        }
        break;

        case CO_ERROR_LMP_RSP_TIMEOUT:
        {
            lc_detach(pid, reason);
        }
        break;

        default:
        {
            if (lc_env_ptr->req.LocEncReq)
            {
                lc_send_enc_chg_evt(idx, reason);
                lc_env_ptr->req.LocEncReq = false;
            }
            ke_state_set(pid, LC_CONNECTED);
        }
        break;
        }
    }
    else
    {
        lc_detach(pid, reason);
    }
}

void lc_calc_link_key(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    bool debug_key = false;
    uint8_t *p_remote_pub_key_x = lc_env_ptr->sp.p_data->remote_pub_key.x;
    uint8_t *p_remote_pub_key_y = lc_env_ptr->sp.p_data->remote_pub_key.y;
    uint8_t *p_dh_key = lc_env_ptr->sp.p_data->dh_key.key;

    lc_env_ptr->enc.KeyStatus = KEY_PRESENT;

    if (lc_env_ptr->sp.sec_con)
    {
        debug_key = lm_debug_key_compare_256(p_remote_pub_key_x, p_remote_pub_key_y);
    }
    else
    {
        debug_key = lm_debug_key_compare_192(p_remote_pub_key_x, p_remote_pub_key_y);
    }

    if (lm_sp_debug_mode_get() || debug_key)
    {
        lc_env_ptr->enc.KeyType = BT_DEBUG_COMB_KEY;
    }

    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        sha_256_f2(lc_env_ptr->sp.sec_con, p_dh_key,
                   &lc_env_ptr->sp.RemRandN.A[0],
                   &lc_env_ptr->sp.LocRandN.A[0],
                   (uint8_t *) "kltb",
                   &lc_env_ptr->info.BdAddr.addr[0],
                   &lc_env_ptr->info.LocalBdAddr.addr[0],
                   &lc_env_ptr->enc.LTKey.ltk[0]);
    }
    else
    {
        sha_256_f2(lc_env_ptr->sp.sec_con, p_dh_key,
                   &lc_env_ptr->sp.LocRandN.A[0],
                   &lc_env_ptr->sp.RemRandN.A[0],
                   (uint8_t *) "kltb",
                   &lc_env_ptr->info.LocalBdAddr.addr[0],
                   &lc_env_ptr->info.BdAddr.addr[0],
                   &lc_env_ptr->enc.LTKey.ltk[0]);
    }

    // End of simple pairing
    lc_sp_end(idx, CO_ERROR_NO_ERROR);

    /*
     * SP AUTH STG2: END
     */
    if (lc_env_ptr->sp.SPInitiator)
    {
        lc_init_start_mutual_auth(pid);
    }
    else
    {
        ke_state_set(pid, LC_WAIT_AU_RAND_RSP);
    }
}

void lc_chg_pkt_type_cont(ke_task_id_t pid, uint8_t status)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    if (lc_env_ptr->link.CurPacketType)
    {
        uint8_t max_slot = LM_MaxSlot(lc_env_ptr->link.CurPacketType);

        if (max_slot != lc_env_ptr->link.TxMaxSlotCur)
        {
            // allocate the status event message
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_max_slot_chg_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MAX_SLOT_CHG_EVT_CODE, hci_max_slot_chg_evt);

            // gets the connection handle
            event->conhdl = conhdl;
            // gets max slot
            event->max_slot = max_slot;
            // send the event
            hci_send_2_host(event);
        }
        lc_env_ptr->link.TxMaxSlotCur = max_slot;

        ld_acl_allowed_tx_packet_types_set(idx, lc_env_ptr->link.CurPacketType);

    }
    lc_chg_pkt_type_cmp(pid, status);
}

void lc_chg_pkt_type_cmp(ke_task_id_t pid, uint8_t status)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->req.LocCPTReq)
    {
        // allocate the status event message
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_con_pkt_type_chg_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_CON_PKT_TYPE_CHG_EVT_CODE, hci_con_pkt_type_chg_evt);
        // gets the status
        event->status = status;
        // gets connection handle
        event->sync_conhdl = conhdl;
        // gets the packet type
        event->pkt_type = lc_env_ptr->link.AclPacketType;
        // send the event
        hci_send_2_host(event);

        lc_env_ptr->req.LocCPTReq = false;
    }
    ke_state_set(pid, LC_CONNECTED);
}

void lc_chg_pkt_type_retry(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

#if (MAX_NB_SYNC > 0)
    if (lm_get_nb_sync_link() > 0)
    {
        uint8_t min_sync_intv = lm_get_min_sync_intv();

        if (min_sync_intv < 12)
        {
            LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x03);
        }
        if (min_sync_intv < 18)
        {
            LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x05);
        }
    }
#endif //(MAX_NB_SYNC > 0)

    if (!LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_3_SLOT_BIT_POS))
    {
        LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x03);
    }
    if (!LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_5_SLOT_BIT_POS))
    {
        LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x05);
    }

    while (lc_env_ptr->link.CurPacketType)
    {
        uint8_t max_slot = LM_MaxSlot(lc_env_ptr->link.CurPacketType);

#if EAVESDROPPING_SUPPORT
        if (max_slot < 5)
        {
            LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x05);
        }
        if (max_slot < 3)
        {
            LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, 0x03);
        }
#endif // EAVESDROPPING_SUPPORT

        if (max_slot > lc_env_ptr->link.TxMaxSlotCur)
        {
            if (lc_env_ptr->link.MaxSlotReceived)
            {
                if (max_slot > lc_env_ptr->link.MaxSlotReceived)
                {
                    LM_SuppressAclPacket(&lc_env_ptr->link.CurPacketType, max_slot);
                }
                else
                {
                    lc_chg_pkt_type_cont(pid, CO_ERROR_NO_ERROR);
                    return;
                }
            }
            else
            {
                if (max_slot == 0x01)
                {
                    lc_chg_pkt_type_cont(pid, CO_ERROR_NO_ERROR);
                    return;
                }
                else
                {
                    // send LMP_MaxSlotReq
                    lc_send_pdu_max_slot_req(idx, max_slot, lc_env_ptr->link.Role);
                    lc_start_lmp_to(pid);
                    ke_state_set(pid, LC_WAIT_MAX_SLOT_CFM);
                    return;
                }
            }
        }
        else
        {
            lc_chg_pkt_type_cont(pid, CO_ERROR_NO_ERROR);
            return;
        }
    }

    if (lc_env_ptr->link.SetupComplete)
    {
        lc_chg_pkt_type_cont(pid, CO_ERROR_UNSUPPORTED);
    }
    else
    {
        lc_detach(pid, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
    }
    return;
}

void lc_afh_start(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct bt_ch_map ch_map;
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Start with all channels marked as disabled
    memset(&lc_env_ptr->afh.ch_map.map[0], 0x00, BT_CH_MAP_LEN);

    // Compute new channel map
    lm_ch_map_compute(&ch_map);

    // If the channel map does not contain enough enabled channel, activate channels in order to have
    // a minimum of AFH_NB_CHANNEL_MIN channels activated.
    lm_fill_ch_map(&ch_map);

    // Set the AFH map
    lc_ch_map_update(idx, &ch_map);

    // Start channel classification reporting
    if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_AFH_CLASS_S_BIT_POS))
    {
        struct lmp_ch_class_req pdu;
        pdu.opcode     = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
        pdu.ext_opcode = LMP_CH_CLASS_REQ_EXTOPCODE;
        pdu.max_intv  = BT_AFH_CH_CLASS_INT_MAX;
        pdu.min_intv  = BT_AFH_CH_CLASS_INT_MIN;
        pdu.rep_mode  = AFH_REPORTING_ENABLED;
        lc_send_lmp(idx, &pdu);

        lc_env_ptr->afh.reporting_en = true;
    }
}

void lc_comb_key_svr(ke_task_id_t pid, struct ltk *key)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct ltk temp_key1;
    struct ltk temp_key2;

    // Copy remote combination key from LMP params
    memcpy(&temp_key2.ltk[0], &key->ltk[0], KEY_LEN);

    //Create LK_RANDb
    LM_MakeRandVec(&temp_key1);

    //Compute Cb
    XorKey(temp_key1, lc_env_ptr->enc.LTKey, &temp_key1);

    // send LMP_CombKey(idx,temp_key1,rx_tr_id)
    lc_send_pdu_comb_key(idx, &temp_key1, lc_env_ptr->link.RxTrIdServer);

    //Restore LK_RANDb
    XorKey(temp_key1, lc_env_ptr->enc.LTKey, &temp_key1);

    //Create LK_Kb
    E21(temp_key1, lc_env_ptr->info.LocalBdAddr, &temp_key1);

    //Compute LK_RANDa
    XorKey(temp_key2, lc_env_ptr->enc.LTKey, &temp_key2);

    //Create LK_Ka
    E21(temp_key2, lc_env_ptr->info.BdAddr, &temp_key2);

    //XOR keys
    XorKey(temp_key2, temp_key1, &lc_env_ptr->enc.LTKey);

    // start timer
    lc_start_lmp_to(pid);

    // wait for au_rand lmp message
    ke_state_set(pid, LC_WAIT_AU_RAND_RSP);
}

void lc_con_cmp(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (!lc_env_ptr->link.ConnectionCompleteSent)
    {
        lc_con_cmp_evt_send(pid, CO_ERROR_NO_ERROR);
        lc_env_ptr->link.ConnectionCompleteSent = true;
    }

    lc_env_ptr->link.SetupComplete = true;

    // Initialize the packet type
    lc_env_ptr->link.AclPacketType = LM_UpdateAclPacketType(lc_env_ptr->link.AclPacketType, lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[0]) |
                                     LM_UpdateAclEdrPacketType(lc_env_ptr->link.AclPacketType,
                                             lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[3],
                                             lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[4],
                                             lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0].feats[5]);

    // Turn on ACL flow
    ld_acl_flow_on(idx);

    ke_msg_send_basic(LC_SYNC_IND, pid, pid);

    ke_state_set(pid, LC_CONNECTED);

    ke_msg_send_basic(LC_OP_PT_IND, pid, pid);

    //Request the timing accuracy of the peer
    ke_msg_send_basic(LC_OP_TIMACC_IND, pid, pid);

#if RW_BT_MWS_COEX
    // Request SAM configuration to peer
    ke_msg_send_basic(LC_OP_SAM_IND, pid, pid);
#endif //RW_BT_MWS_COEX

    if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_QUALITY_BIT_POS))
    {
        // send LMP_AutoRate
        lc_send_pdu_auto_rate(idx, lc_env_ptr->link.Role);
    }
}

void lc_con_cmp_evt_send(ke_task_id_t pid, uint8_t status)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    // allocate the status event message
    uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
    struct hci_con_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_CON_CMP_EVT_CODE, hci_con_cmp_evt);
    event->status = status;
    event->link_type = ACL_TYPE;
    memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
    event->conhdl = conhdl;
    event->enc_en = lc_env_ptr->enc.EncMode;
    hci_send_2_host(event);

    // Register the new connection handle to HCI
    hci_bt_acl_conhdl_register(idx);
}

void lc_try_start_auth_stage2(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->sp.dh_key_compute_state != LC_DH_KEY_COMP_PENDING)
    {
        // DH Key calculation is over, start Auth Stage 2
        if (lc_env_ptr->sp.SPInitiator)
        {
            lc_init_calc_f3(pid);
        }
        else
        {
            lc_start_lmp_to(pid);
            ke_state_set(pid, LC_WAIT_DHKEY_RSP);
        }
    }
    else
    {
        // Wait for DH Key calculation before starting to Auth Stage 2
        ke_state_set(pid, LC_WAIT_DHKEY_COMPUTING);
    }
}

void lc_locepr_rsw(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    ld_acl_flow_off(idx);

    if (lc_env_ptr->sp.sec_con)
    {
        // Send LMP_PauseEncryptionAesReq
        lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);
        lc_send_pdu_pause_enc_aes_req(idx, lc_env_ptr->link.Role, &lc_env_ptr->sp.LocRandN);
    }
    else
    {
        // Send LMP_PauseEncryptionReq
        lc_send_pdu_paus_enc_req(idx, lc_env_ptr->link.Role);
    }

    lc_start_lmp_to(pid);
    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
    }
    else
    {
        ke_state_set(pid, LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT);
    }
}

void lc_locepr_lkref(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.Initiator = true;

    ld_acl_flow_off(idx);

    lc_start_lmp_to(pid);
    if (lc_env_ptr->sp.sec_con)
    {
        // Send LMP_PauseEncryptionAesReq
        lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);
        lc_send_pdu_pause_enc_aes_req(idx, lc_env_ptr->link.Role, &lc_env_ptr->sp.LocRandN);
    }
    else
    {
        // Send LMP_PauseEncryptionReq
        lc_send_pdu_paus_enc_req(idx, lc_env_ptr->link.Role);
    }

    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
    }
    else
    {
        ke_state_set(pid, LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT);
    }
}
void lc_enc_key_refresh(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_PAUSE_ENCRYPT_BIT_POS))
    {
        if (lc_env_ptr->epr.rsw)
        {
            lc_cmd_stat_send(HCI_SWITCH_ROLE_CMD_OPCODE, CO_ERROR_NO_ERROR);
        }

        ld_acl_flow_off(idx);

        lc_start_lmp_to(pid);
        if (lc_env_ptr->link.Initiator)
        {
            if (lc_env_ptr->sp.sec_con)
            {
                // Send LMP_PauseEncryptionAesReq
                lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);
                lc_send_pdu_pause_enc_aes_req(idx, lc_env_ptr->link.Role, &lc_env_ptr->sp.LocRandN);
            }
            else
            {
                // Send LMP_PauseEncryptionReq
                lc_send_pdu_paus_enc_req(idx, lc_env_ptr->link.Role);
            }
            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
            }
            else
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_PAUSE_REQ_MST_INIT);
            }
        }
        else
        {
            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP);
            }
            else
            {
                ke_state_set(pid, LC_WAIT_EPR_ENC_PAUSE_REQ_MST_RSP);
            }
        }
    }
    else
    {
        lc_env_ptr->epr.on = false;
        lc_env_ptr->req.LocEncKeyRefresh = false;

        if (lc_env_ptr->epr.cclk)
        {
            lc_env_ptr->epr.cclk = false;
        }

        if (lc_env_ptr->epr.rsw)
        {
            lc_env_ptr->epr.rsw = false;
            lc_env_ptr->enc.NewEncMode = ENC_DISABLED;
            lc_env_ptr->req.LocEncReq = true;
        }
        else
        {
            lc_env_ptr->req.PeerEncKeyRefresh = false;
        }

        if (lc_env_ptr->req.LocEncReq == true)
        {
            ke_msg_send_basic(LC_OP_ENC_IND, pid, pid);
        }
        else
        {
            ke_state_set(pid, LC_CONNECTED);
        }
    }
}

void lc_end_chk_colli(ke_task_id_t pid, uint8_t enc_mode)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        // send LMP_NotAccepted(LMP_ENC_MODE_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_ENC_MODE_REQ_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
    }
    else
    {
        if (lc_env_ptr->req.LocEncReq)
        {
            // send encryption change event
            lc_send_enc_chg_evt(idx, CO_ERROR_LMP_COLLISION);
            lc_env_ptr->req.LocEncReq = false;
        }
        lc_env_ptr->req.PeerEncReq = true;
        lc_env_ptr->enc.NewEncMode = enc_mode;

        lc_rem_enc(pid);
    }
}

void lc_enc_cmp(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    ke_timer_clear(LC_LMP_RSP_TO, pid);

    ld_acl_flow_on(idx);

    do
    {
        if (lc_env_ptr->req.LocEncReq)
        {
            lc_send_enc_chg_evt(idx, reason);
            lc_env_ptr->req.LocEncReq = false;
        }
        else
        {
            if (lc_env_ptr->link.SetupComplete)
            {
                if (lc_env_ptr->req.RestartEncReq && (reason == CO_ERROR_NO_ERROR))
                {
                    lc_restart_enc_cont(pid, reason);
                    break;
                }
                else
                {
                    if (lc_env_ptr->enc.PreventEncEvt)
                    {
                        // reset the master key responder flag only when encryption has
                        // been restarted : if Encryption OFF, don't reset the flag
                        // by-pass sending of encryption change event to upper layer
                        if (lc_env_ptr->enc.EncEnable == ENCRYPTION_ON)
                        {
                            lc_env_ptr->enc.PreventEncEvt = false;
                        }
                    }
                    // always send enc chg event if success, but just locally if fail
                    else if ((lc_env_ptr->link.Role == lc_env_ptr->link.RxTrIdServer) || (reason == CO_ERROR_NO_ERROR))
                    {
                        // send encryption change event
                        lc_send_enc_chg_evt(idx, reason);
                    }
                }
            }
        }
        lc_env_ptr->link.Initiator = false;
        if (reason == CO_ERROR_LMP_RSP_TIMEOUT)
        {
            lc_detach(pid, reason);
        }
        else
        {
            if (lc_env_ptr->enc.EncMode == ENC_DISABLED)
            {
                lc_env_ptr->epr.cclk = false;
                lc_env_ptr->epr.on = false;
            }
            ke_state_set(pid, LC_CONNECTED);
        }
    }
    while (0);
}

void lc_epr_cmp(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    ke_timer_clear(LC_LMP_RSP_TO, pid);

    ld_acl_flow_on(idx);

    uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
    struct hci_enc_key_refresh_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_ENC_KEY_REFRESH_CMP_EVT_CODE, hci_enc_key_refresh_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->conhdl = conhdl;
    hci_send_2_host(evt);

    // Clear Mic failures counter
    lc_env_ptr->link.micerr_cnt = 0;

    if (lc_env_ptr->req.LocKeyExchangeReq)
    {
        lc_env_ptr->epr.cclk = false;
        lc_epr_change_lk(pid, CO_ERROR_NO_ERROR);
    }
    else
    {
        if (lc_env_ptr->epr.rsw)
        {
            lc_rsw_done(pid, lc_env_ptr->epr.rswerror);
        }
        else
        {
            ke_state_set(pid, LC_CONNECTED);
        }
    }

    lc_env_ptr->epr.on = false;
    lc_env_ptr->epr.rsw = false;
}

void lc_epr_change_lk(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->req.LocKeyExchangeReq)
    {
        // change connection link key event
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_chg_con_lk_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_CHG_CON_LK_CMP_EVT_CODE, hci_chg_con_lk_cmp_evt);
        // fill the parameters
        evt->status = reason;
        evt->conhdl = conhdl;
        // send the event
        hci_send_2_host(evt);

        lc_env_ptr->req.LocKeyExchangeReq = false;
    }
    lc_mutual_auth_end2(pid, reason);
}

void lc_rsw_done(ke_task_id_t pid, uint8_t status)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Unregister the Role Switch at LM
    lm_role_switch_finished(idx, (status == CO_ERROR_NO_ERROR));

    if (lc_env_ptr->req.LocSwitchReq || (lc_env_ptr->req.PeerSwitchReq && (status == CO_ERROR_NO_ERROR)))
    {
        // Send the complete event message
        struct hci_role_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_ROLE_CHG_EVT_CODE, hci_role_chg_evt);
        evt->status = status;
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        evt->new_role = lc_env_ptr->link.Role;
        hci_send_2_host(evt);

        lc_env_ptr->req.LocSwitchReq = false;
        lc_env_ptr->req.PeerSwitchReq = false;

#if EAVESDROPPING_SUPPORT
        // ACL connection changed (RSW)
        if (status == CO_ERROR_NO_ERROR)
        {
            // Allocate indication message
            struct ed_acl_con_chg_ind *ind = KE_MSG_ALLOC(ED_ACL_CON_CHG_IND, TASK_ED, TASK_NONE, ed_acl_con_chg_ind);

            // Fill data
            memcpy(&ind->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
            ind->link_id  = idx;
            ind->status = ED_STATUS_CONNECTED;
            ind->role = lc_env_ptr->link.Role;

            // Send message
            ke_msg_send(ind);
        }
#endif // EAVESDROPPING_SUPPORT
    }

    if ((status != CO_ERROR_NO_ERROR) && (lc_env_ptr->link.Role == SLAVE_ROLE))
    {
        lc_restore_afh_reporting(pid);
    }

    if (lc_conn_seq_done(pid))
    {
        switch (status)
        {
        case CO_ERROR_NO_ERROR:
        {
            // Start AFH if local device is master and slave supports it
            if ((lc_env_ptr->link.Role == MASTER_ROLE) && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_AFH_CAPABLE_S_BIT_POS))
            {
                lc_afh_start(pid);
            }

            ke_state_set(pid, LC_CONNECTED);
        }
        break;
        case CO_ERROR_LMP_RSP_TIMEOUT:
            lc_detach(pid, CO_ERROR_LMP_RSP_TIMEOUT);
            break;
        default:
            ke_state_set(pid, LC_CONNECTED);
            break;
        }
    }
    else
    {
        if (lc_env_ptr->link.Role ^ (status == CO_ERROR_NO_ERROR))
        {
            // LMP_Accepted(LMP_HOST_CON_REQ_OPCODE)
            lc_env_ptr->link.ConnectedState = true;
            lc_send_pdu_acc(idx, LMP_HOST_CON_REQ_OPCODE, MASTER_ROLE);
            ke_state_set(pid, LC_CONNECTED);

            // Start AFH if local device is master and slave supports it
            if ((lc_env_ptr->link.Role == MASTER_ROLE) && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_AFH_CAPABLE_S_BIT_POS))
            {
                lc_afh_start(pid);
            }
        }
        else
        {
            lc_start_lmp_to(pid);
            ke_state_set(pid, LC_WAIT_REM_HL_CON);
        }
    }
}

void lc_epr_rsw_cmp(ke_task_id_t pid, uint8_t status)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->epr.rswerror = status;

    // start timer
    lc_start_lmp_to(pid);

    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        if (!lc_env_ptr->req.LocSwitchReq)
        {
            //if we are master and we are not the r/s initiator
            //wait the resume request from the slave initiator
            ke_state_set(pid, LC_WAIT_EPR_ENC_RESUME_REQ_MST_RSP);
        }
        else
        {
            //if master and initiator restart the encryption
            ke_timer_clear(LC_LMP_RSP_TO, pid);
            lc_start_enc(pid);
        }
    }
    else
    {
        if (lc_env_ptr->link.Initiator)
        {
            // if slave and r/s initiator send Resume Encryption Request
            lc_send_pdu_resu_enc_req(idx, lc_env_ptr->link.Role);
        }
        //wait the start encryption from the master
        ke_state_set(pid, LC_WAIT_EPR_PEER_REQ_SLV);
    }
}

void lc_init_calc_f3(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    /* If the Initiator has determined that the received public key is invalid (see [Vol 2] Part H, Section 7.6),
     * the PDU should include a value that is different from the computed confirmation value (for example, substituting
     * a randomly generated number). Otherwise, the PDU shall include the computed confirmation value. */
    if (lc_env_ptr->sp.dh_key_compute_state != LC_DH_KEY_COMP_SUCCESS)
    {
        for (int8_t i = (KEY_LEN - 1); i > 0; i--)
        {
            lc_env_ptr->sp.DHKeyCheck.ltk[i] = co_rand_byte();
        }
    }
    else
    {
        uint8_t *p_dh_key = lc_env_ptr->sp.p_data->dh_key.key;
        sha_256_f3(lc_env_ptr->sp.sec_con, p_dh_key,
                   &lc_env_ptr->sp.LocRandN.A[0],
                   &lc_env_ptr->sp.RemRandN.A[0],
                   &lc_env_ptr->sp.RemCommitment.A[0],
                   (uint8_t *) &lc_env_ptr->sp.IOCap_loc,
                   &lc_env_ptr->info.LocalBdAddr.addr[0],
                   &lc_env_ptr->info.BdAddr.addr[0],
                   &lc_env_ptr->sp.DHKeyCheck.ltk[0]);
    }

    // send LMP_DHKeyCheck(idx,DHKeyCheck,sptid)
    lc_send_pdu_dhkey_chk(idx, &lc_env_ptr->sp.DHKeyCheck, lc_env_ptr->sp.SpTId);

    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_DHKEY_CHECK_INIT_CFM);
}

void lc_init_start_mutual_auth(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);

    // send LMP_AuRand
    lc_send_pdu_au_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.Role);

    lc_start_lmp_to(pid);

    if (lc_env_ptr->sp.sec_con)
    {
        ke_state_set(pid, LC_WAIT_AU_RAND_SEC_AUTH_INIT);
    }
    else
    {
        ke_state_set(pid, LC_WAIT_SRES_INIT);
    }
}

void lc_init_passkey_loop(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t passkey_bit = ((lc_env_ptr->sp.Passkey & (1 << lc_env_ptr->sp.EncapPduCtr)) ? 0x81 : 0x80);

    const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
    const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

    sha_256_f1(lc_env_ptr->sp.sec_con, p_local_pub_key_x, p_remote_pub_key_x,
               &lc_env_ptr->sp.LocRandN.A[0],
               &passkey_bit,
               &lc_env_ptr->sp.LocCommitment.A[0]);


    // send LMP_SimplePairingConfirm
    lc_send_pdu_sp_cfm(idx, &lc_env_ptr->sp.LocCommitment, lc_env_ptr->sp.SpTId);

    lc_start_lmp_to(pid);
    ke_state_set(pid, LC_WAIT_PASSKEY_COMM_INIT_PEER);
}

void lc_initiator_epr(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if ((!lc_env_ptr->epr.cclk) && (!lc_env_ptr->epr.on))
    {
        lc_env_ptr->req.PeerEncKeyRefresh = true;
        lc_env_ptr->epr.on = true;
        lc_env_ptr->link.Initiator = false;
        lc_enc_key_refresh(pid);
    }
    else
    {
        //send LMP_NotAcceptedExt (idx,escape4,LMP_PAUSE_ENC_REQ_EXTOPCODE,LMP_COLLISION,servertid)
        lc_send_pdu_not_acc_ext4(idx, LMP_PAUSE_ENC_REQ_EXTOPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
    }
}

void lc_epr_resp(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t reason = CO_ERROR_NO_ERROR;
    do
    {
        if (!lc_env_ptr->epr.on)
        {
            lc_env_ptr->epr.on = true;
            lc_env_ptr->link.Initiator = false;
            ld_acl_flow_off(idx);

            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                lc_start_lmp_to(pid);
                // Send LMP_PauseEncryptionReq
                lc_send_pdu_paus_enc_req(idx, !lc_env_ptr->link.Role);
                ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_RSP);
            }
            else
            {
                lc_stop_enc(pid);
            }
            break;
        }
        else
        {
            reason = CO_ERROR_LMP_COLLISION;
        }
        //send LMP_NotAcceptedExt (idx,escape4,LMP_PAUSE_ENC_REQ_EXTOPCODE,LMP_COLLISION,servertid)
        lc_send_pdu_not_acc_ext4(idx, LMP_PAUSE_ENC_REQ_EXTOPCODE, reason, lc_env_ptr->link.RxTrIdServer);
    }
    while (0);
}

void lc_key_exch_end(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (reason == CO_ERROR_NO_ERROR)
    {
        lc_init_start_mutual_auth(pid);
    }
    else
    {
        if (lc_env_ptr->req.LocKeyExchangeReq)
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_chg_con_lk_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_CHG_CON_LK_CMP_EVT_CODE, hci_chg_con_lk_cmp_evt);
            // fill the parameters
            evt->status = reason;
            evt->conhdl = conhdl;

            // send the event
            hci_send_2_host(evt);

            lc_env_ptr->req.LocKeyExchangeReq = false;
        }
        if ((reason != CO_ERROR_LMP_COLLISION) && (!lc_env_ptr->link.SetupComplete))
        {
            lc_detach(pid, reason);
        }
        else
        {
            lc_mutual_auth_end(pid, reason);
        }
    }
}

void lc_legacy_pair(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->enc.PinStatus == PIN_PRESENT)
    {
        lc_pairing_cont(pid);
    }
    else
    {
        // allocate the status event message
        struct hci_pin_code_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_PIN_CODE_REQ_EVT_CODE, hci_pin_code_req_evt);
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(evt);

        // start lmp timer
        lc_start_lmp_to(pid);

        // wait for pin code
        ke_state_set(pid, LC_WAIT_PIN_CODE);
    }
}

void lc_local_switch(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    ld_acl_flow_off(idx);

    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        // Get slot offset
        uint16_t slot_offset = ld_acl_rsw_slot_offset_get(idx);

        // send LMP_SlotOffset
        lc_send_pdu_slot_off(idx, slot_offset, lc_env_ptr->link.Role);

        lc_stop_afh_reporting(pid);
    }
    else
    {
        lc_util_set_loc_trans_coll(idx, LMP_SWITCH_REQ_OPCODE, LC_UTIL_NOT_USED, LC_UTIL_INUSED);
    }

    /*
     * Switch instant should be 20 times the POLL interval since we have 3 LMP messages
     * to exchange (SwitchReq, SlotOffset, Accepted) and we must give a chance for
     * retransmission
     */
    lc_env_ptr->link.SwitchInstant = 10 * ld_acl_t_poll_get(idx);
    /*
     * Switch instant must be at least 300 slots in the future (187ms) in case we have
     * a multi-slots data flow in both directions. The peer device must have time to stop
     * its current transmission flow !
     */
    if (lc_env_ptr->link.SwitchInstant < 300)
    {
        lc_env_ptr->link.SwitchInstant = 300;
    }
    /* SwitchInstant is an absolute time => add master clock                             */
    lc_env_ptr->link.SwitchInstant = CLK_ADD_3(2 * lc_env_ptr->link.SwitchInstant, ld_read_clock(), ld_acl_clock_offset_get(idx).hs) >> 1;
    // Ensure SwitchInstant is an even slot                                                          */
    lc_env_ptr->link.SwitchInstant = CO_ALIGN2_HI(lc_env_ptr->link.SwitchInstant);

    {
        // send LMP_SwitchReq
        struct lmp_switch_req pdu;
        // fill the parameters
        pdu.opcode     =  LMP_OPCODE(LMP_SWITCH_REQ_OPCODE, lc_env_ptr->link.Role);
        pdu.switch_inst = lc_env_ptr->link.SwitchInstant;
        // send lmp pdu
        lc_send_lmp(idx, &pdu);
    }
    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_SWITCH_CFM);
}

void lc_max_slot_mgt(ke_task_id_t pid, uint8_t max_slot)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t pkt_type = 0;
    uint8_t loop = 1;

#if (EAVESDROPPING_SUPPORT)
    max_slot = co_min(max_slot, lm_get_host_max_slot());
#endif // EAVESDROPPING_SUPPORT

    do
    {
        switch (max_slot)
        {
        case 0x01:
            pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                       + PACKET_TYPE_NO_2_DH3_FLAG + PACKET_TYPE_NO_3_DH3_FLAG
                       + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
            loop = 0;
            break;
        case 0x03:
            pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                       + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                       + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
            loop = 0;
            break;
        case 0x05:
            pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                       + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                       + PACKET_TYPE_DM5_FLAG + PACKET_TYPE_DH5_FLAG;
            loop = 0;
            break;
        default:
            ASSERT_ERR_FORCE(0); // should not reach here
            break;
        }
    }
    while (loop);

    pkt_type = LM_ComputePacketType(lc_env_ptr->link.AclPacketType, pkt_type, false);

    if (!pkt_type)
    {
        pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG;
    }

    lc_env_ptr->link.CurPacketType = pkt_type;

#if (MAX_NB_SYNC > 0)
    if ((max_slot != 0x01) && (lm_get_nb_sync_link() > 0))
        return;
#endif //(MAX_NB_SYNC > 0)

    if (max_slot != lc_env_ptr->link.TxMaxSlotCur)
    {
        // allocate the status event message
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_max_slot_chg_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MAX_SLOT_CHG_EVT_CODE, hci_max_slot_chg_evt);
        // gets the status
        event->conhdl = conhdl;
        // gets connection handle
        event->max_slot = max_slot;
        // send the event
        hci_send_2_host(event);
    }
    lc_env_ptr->link.TxMaxSlotCur = LM_MaxSlot(pkt_type);

    pkt_type = LM_ComputePacketType(lc_env_ptr->link.RxPreferredRate, pkt_type, true);

    ld_acl_allowed_tx_packet_types_set(idx, pkt_type);
}

void lc_mutual_auth_end2(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    bool wait_restart = false;

#if BCAST_ENC_SUPPORT
    if (lc_env_ptr->req.MasterKeyReq)
    {
        lc_env_ptr->enc.KeyFlag = lc_env_ptr->enc.NewKeyFlag;
        wait_restart = true;

        // no need to send the encryption change event
        // this event is optional for master link key command
        if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
        {
            // prevent the sending of encryption change event
            lc_env_ptr->enc.PreventEncEvt = true;
        }

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            // send master key confirm
            struct lb_mst_key_cfm *msg = KE_MSG_ALLOC(LB_MST_KEY_CFM, TASK_LB, pid, lb_mst_key_cfm);
            lc_env_ptr->req.MasterKeyReq = false;

            msg->status = reason;
            msg->enc_mode = lc_env_ptr->enc.EncMode;
            msg->key_sz_msk = lc_env_ptr->enc.KeySizeMask;
            ke_msg_send(msg);
        }
        else // ROLE is SLAVE
        {
            if (lc_env_ptr->enc.EncMode == ENC_DISABLED)
            {
                uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
                struct hci_master_lk_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MASTER_LK_CMP_EVT_CODE, hci_master_lk_cmp_evt);
                lc_env_ptr->req.MasterKeyReq = false;

                evt->status = reason;
                evt->conhdl = conhdl;
                evt->key_flag = lc_env_ptr->enc.KeyFlag;
                hci_send_2_host(evt);
            }
        }
    }
#endif // BCAST_ENC_SUPPORT

    if ((reason != CO_ERROR_NO_ERROR) && (reason != CO_ERROR_LMP_COLLISION) && (!lc_env_ptr->link.SetupComplete))
    {
        lc_detach(pid, reason);
    }
    else
    {
        if (reason == CO_ERROR_NO_ERROR)
        {
            lc_env_ptr->enc.LinkKeyValid = true;
            if ((lc_env_ptr->enc.EncMode == ENC_DISABLED) || (lc_env_ptr->link.Initiator))
            {
                if (lc_env_ptr->req.LocEncReq == true)
                {
                    ke_msg_send_basic(LC_OP_ENC_IND, pid, pid);
                }
                else
                {
                    // Clear the Initiator of procedure status
                    lc_env_ptr->link.Initiator = false;
                    ke_state_set(pid, LC_CONNECTED);
                }
            }
            else
            {
                if (wait_restart == true)
                {
                    // set state to prepare for encryption restart
                    ke_state_set(pid, LC_WAIT_RESTART_ENC);
                }
                else
                {
                    lc_start_lmp_to(pid);
                    ke_state_set(pid, LC_WAIT_EPR_RSP);
                }

            }
        }
        else
        {
            if (lc_env_ptr->req.LocEncReq)
            {
                lc_send_enc_chg_evt(idx, reason);
                lc_env_ptr->req.LocEncReq = false;
            }

            if (reason == CO_ERROR_LMP_RSP_TIMEOUT)
            {
                lc_detach(pid, reason);
            }
            else
            {
                ke_state_set(pid, LC_CONNECTED);
            }
        }
    }
}

void lc_mutual_auth_end(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if ((reason == CO_ERROR_NO_ERROR) && !lc_env_ptr->enc.key_from_host && !lc_env_ptr->req.PeerAuthReq
#if BCAST_ENC_SUPPORT
            && !lc_env_ptr->req.MasterKeyReq
#endif // BCAST_ENC_SUPPORT
       )
    {
        // send link key notification event
        struct hci_lk_notif_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_LK_NOTIF_EVT_CODE, hci_lk_notif_evt);
        evt->key_type = lc_env_ptr->enc.KeyType;
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        memcpy(&evt->key.ltk[0], &lc_env_ptr->enc.LTKey.ltk[0], KEY_LEN);
        hci_send_2_host(evt);
    }
    lc_env_ptr->enc.key_from_host = false;
    lc_env_ptr->enc.LinkKeyValid = true;

    if (lc_env_ptr->req.LocAuthReq && lc_conn_seq_done(pid)
#if BCAST_ENC_SUPPORT
            && !lc_env_ptr->req.MasterKeyReq
#endif // BCAST_ENC_SUPPORT
       )
    {
        uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
        struct hci_auth_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_CMP_EVT_CODE, hci_auth_cmp_evt);
        evt->status = reason;
        evt->conhdl = conhdl;
        hci_send_2_host(evt);
    }

    // reset local authentication request
    lc_env_ptr->req.LocAuthReq = false;
    lc_env_ptr->req.PeerAuthReq = false;

    //Check if EPR is requested
    if (lc_env_ptr->epr.on)
    {
        // Perform encryption pause resume
        lc_locepr_lkexc(pid);
    }
    else
    {
        // mutual authentication II
        lc_epr_change_lk(pid, reason);
    }
}

#if BCAST_ENC_SUPPORT
void lc_mst_key(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        // no need to send the encryption change event
        // this event is optional for master link key command
        if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
        {
            // prevent the sending of encryption change event
            lc_env_ptr->enc.PreventEncEvt = true;
        }

        if (lc_env_ptr->enc.NewKeyFlag == SEMI_PERMANENT_KEY)
        {
            memcpy(&lc_env_ptr->enc.LTKey.ltk[0], &lc_env_ptr->enc.SemiPermanentKey.ltk[0], KEY_LEN);

            // send LMP_UseSemiPermanentKey(idx,Role)
            {
                struct lmp_use_semi_perm_key pdu;
                pdu.opcode = LMP_OPCODE(LMP_USE_SEMI_PERM_KEY_OPCODE, lc_env_ptr->link.Role);
                // send LMP pdu
                lc_send_lmp(idx, &pdu);
            }

            lc_start_lmp_to(pid);

            ke_state_set(pid, LC_WAIT_MST_SEMI_ACC);
        }
        else
        {
            if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_BCAST_ENCRYPT_BIT_POS))
            {
                // send LMP_EncryptionKeySizeMaskReq(idx,Role)
                struct lmp_enc_key_size_mask_req pdu;
                // fill the parameters
                pdu.opcode = LMP_OPCODE(LMP_ENC_KEY_SIZE_MASK_REQ_OPCODE, lc_env_ptr->link.Role);
                // send the lmp pdu
                lc_send_lmp(idx, &pdu);

                lc_start_lmp_to(pid);
                ke_state_set(pid, LC_MST_WAIT_ENC_SIZE_MSK);
            }
            else
            {
                //  send LMP_VersionReq(idx,LMP_VERSION,LMP_MANUFACTURER_NAME,
                // LMP_SUB_VERSION,Role)
                lc_send_pdu_vers_req(idx, lc_env_ptr->link.Role);

                // start the timer
                lc_start_lmp_to(pid);

                // set the state
                ke_state_set(pid, LC_MST_WAIT_VERS);
            }
        }
    }
    else // if SLAVE
    {
        if (lc_env_ptr->enc.NewKeyFlag == SEMI_PERMANENT_KEY)
        {
            // LMP_Accepted(idx,LMP_USE_SEMIPERM_KEY_OPCODE,rx_tr_id)
            lc_send_pdu_acc(idx, LMP_USE_SEMI_PERM_KEY_OPCODE, lc_env_ptr->link.RxTrIdServer);
            lc_env_ptr->enc.KeyFlag = lc_env_ptr->enc.NewKeyFlag;
            if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
            {
                lc_env_ptr->req.RestartEncReq = true;
            }
            lc_semi_key_cmp(pid, CO_ERROR_NO_ERROR);
        }
        else
        {
            lc_start_lmp_to(pid);
            ke_state_set(pid, LC_WAIT_AU_RAND_RSP);
        }
    }
}

void lc_mst_send_mst_key(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct ltk temp_key;
    // send LMP_TempRand
    struct lmp_temprand pdu1;
    // send LMP_TempKey
    struct lmp_tempkey pdu2;

    memcpy(&lc_env_ptr->enc.SemiPermanentKey.ltk[0], &lc_env_ptr->enc.LTKey.ltk[0], KEY_LEN);

    LM_GetMasterKeyRand(&temp_key);

    pdu1.opcode = LMP_OPCODE(LMP_TEMPRAND_OPCODE, lc_env_ptr->link.Role);
    memcpy(&pdu1.random.ltk[0], &temp_key.ltk[0], KEY_LEN);
    lc_send_lmp(idx, &pdu1);

    memcpy(&lc_env_ptr->enc.PinCode.pin[0], &lc_env_ptr->enc.SemiPermanentKey.ltk[0], KEY_LEN);

    E22(temp_key, lc_env_ptr->info.BdAddr, lc_env_ptr->enc.PinCode, KEY_LEN, &lc_env_ptr->enc.Overlay);

    LM_GetMasterKey(&lc_env_ptr->enc.LTKey);

    XorKey(lc_env_ptr->enc.Overlay, lc_env_ptr->enc.LTKey, &temp_key);

    pdu2.opcode = LMP_OPCODE(LMP_TEMPKEY_OPCODE, lc_env_ptr->link.Role);
    memcpy(&pdu2.key.ltk[0], &temp_key.ltk[0], KEY_LEN);
    lc_send_lmp(idx, &pdu2);

    lc_init_start_mutual_auth(pid);
}
#endif // BCAST_ENC_SUPPORT

void lc_mst_qos_done(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    ld_acl_t_poll_set(idx, lc_env_ptr->link.PollInterval);

    // allocate the status event message
    uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
    struct hci_qos_setup_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_QOS_SETUP_CMP_EVT_CODE, hci_qos_setup_cmp_evt);
    // update the status
    event->status = CO_ERROR_NO_ERROR;
    event->conhdl = conhdl;
    event->flags = 0;
    event->tok_rate = lc_env_ptr->link.TokenRate;
    event->serv_type = lc_env_ptr->link.ServiceType;
    event->pk_bw = lc_env_ptr->link.PeakBandwidth;
    event->lat = lc_env_ptr->link.Latency;
    event->del_var = lc_env_ptr->link.DelayVariation;
    // send the message
    hci_send_2_host(event);
    ke_state_set(pid, LC_CONNECTED);
}

void lc_pairing_cont(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Standard pairing: set key type to combination key
    lc_env_ptr->enc.KeyType = BT_COMB_KEY;

    if (lc_env_ptr->link.Initiator)
    {
        LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);

        // E22 procedure
        E22(lc_env_ptr->enc.RandomTx, lc_env_ptr->info.BdAddr,
            lc_env_ptr->enc.PinCode, lc_env_ptr->enc.PinLength,
            &lc_env_ptr->enc.LTKey);

        lc_env_ptr->enc.KeyStatus = KEY_PRESENT;

        // send LMP_InRand
        lc_send_pdu_in_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.Role);

        ke_state_set(pid, LC_WAIT_PAIR_CFM_INIT);
    }
    else
    {
        if (LM_GetPINType() == FIXED_PIN)
        {
            LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);

            // E22 procedure
            E22(lc_env_ptr->enc.RandomTx, lc_env_ptr->info.BdAddr,
                lc_env_ptr->enc.PinCode, lc_env_ptr->enc.PinLength,
                &lc_env_ptr->enc.LTKey);
            lc_env_ptr->enc.KeyStatus = KEY_PRESENT;

            // send LMP_InRand
            lc_send_pdu_in_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.RxTrIdServer);

            ke_state_set(pid, LC_WAIT_PAIR_CFM_RSP);
        }
        else
        {
            // LMP_Accepted(LMP_INRAND_OPCODE)
            lc_send_pdu_acc(idx, LMP_INRAND_OPCODE, lc_env_ptr->link.RxTrIdServer);

            // use randomrx, value received from lmp_in_rand
            E22(lc_env_ptr->enc.RandomRx, lc_env_ptr->info.LocalBdAddr,
                lc_env_ptr->enc.PinCode, lc_env_ptr->enc.PinLength,
                &lc_env_ptr->enc.LTKey);

            lc_env_ptr->enc.KeyStatus = KEY_PRESENT;

            ke_state_set(pid, LC_WAIT_KEY_EXCH_RSP);
        }
    }

    lc_start_lmp_to(pid);
}

void lc_passkey_comm(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->sp.SPPhase1Failed)
    {
        // send LMP_NotAccepted(LMP_SP_CFM_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SP_CFM_OPCODE, CO_ERROR_AUTH_FAILURE, lc_env_ptr->link.RxTrIdServer);
    }
    else
    {
        uint8_t passkey_bit = ((lc_env_ptr->sp.Passkey & (1 << lc_env_ptr->sp.EncapPduCtr)) ? 0x81 : 0x80);

        lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);

        const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
        const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

        sha_256_f1(lc_env_ptr->sp.sec_con, p_local_pub_key_x, p_remote_pub_key_x,
                   &lc_env_ptr->sp.LocRandN.A[0],
                   &passkey_bit,
                   &lc_env_ptr->sp.LocCommitment.A[0]);

        // send LMP_SimplePairingConfirm
        lc_send_pdu_sp_cfm(idx, &lc_env_ptr->sp.LocCommitment, lc_env_ptr->sp.SpTId);

        lc_start_lmp_to(pid);
        ke_state_set(pid, LC_WAIT_PASSKEY_RANDN_RSP_PEER);
    }
}

void lc_ptt(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    do
    {
        if (lc_env_ptr->sp.SPInitiator)
        {
            if (lc_env_ptr->link.ptt_tmp == lc_env_ptr->link.CurPacketTypeTable)
            {
                // LMP_AcceptedExt(LMP_PKT_TYPE_TBL_REQ_EXTOPCODE)
                lc_send_pdu_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, lc_env_ptr->link.RxTrIdServer);
            }
            else
            {
                // send LMP_NotAcceptedExt
                lc_send_pdu_not_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, CO_ERROR_CONTROLLER_BUSY, lc_env_ptr->link.RxTrIdServer);
            }
            break;
        }

        if (lc_env_ptr->link.ptt_tmp == PACKET_TABLE_1MBPS)
        {
            if (lc_env_ptr->link.ptt_tmp == lc_env_ptr->link.CurPacketTypeTable)
            {
                if (lc_env_ptr->link.CurPacketType == lc_env_ptr->link.AclPacketType)
                {
                    // LMP_AcceptedExt(LMP_PKT_TYPE_TBL_REQ_EXTOPCODE)
                    lc_send_pdu_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, lc_env_ptr->link.RxPktTypeID);
                    break;
                }
            }
            else
            {
                lc_env_ptr->link.CurPacketTypeTable = lc_env_ptr->link.ptt_tmp;
            }
            lc_ptt_cont(pid);
            break;
        }
        else if (lc_env_ptr->link.ptt_tmp == PACKET_TABLE_2_3MBPS)
        {
            if (LM_CheckEdrFeatureRequest(lc_env_ptr->link.AclPacketType))
            {
                if (lc_env_ptr->link.ptt_tmp == lc_env_ptr->link.CurPacketTypeTable)
                {
                    if (lc_env_ptr->link.CurPacketType == lc_env_ptr->link.AclPacketType)
                    {
                        // LMP_AcceptedExt(LMP_PKT_TYPE_TBL_REQ_EXTOPCODE)
                        lc_send_pdu_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, lc_env_ptr->link.RxPktTypeID);
                        break;
                    }
                    else
                    {
                        lc_ptt_cont(pid);
                        break;
                    }
                }
                else
                {
                    lc_env_ptr->link.CurPacketTypeTable = lc_env_ptr->link.ptt_tmp;
                    lc_ptt_cont(pid);
                    break;
                }
            }
            else
            {
                status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
            }
        }
        else
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
        }

        // send LMP_NotAcceptedExt(LMP_PKT_TYPE_TBL_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_PKT_TYPE_TBL_REQ_EXTOPCODE, status, lc_env_ptr->link.RxPktTypeID);
    }
    while (0);
}

void lc_ptt_cmp(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t pkt_type;

    ld_acl_edr_set(idx, lc_env_ptr->link.CurPacketTypeTable);

    pkt_type = LM_ComputePacketType(lc_env_ptr->link.RxPreferredRate, lc_env_ptr->link.CurPacketType, true);

    ld_acl_allowed_tx_packet_types_set(idx, pkt_type);
    lc_env_ptr->link.PacketTypeTable2Mb = true;

    // Restart the flow
    ld_acl_flow_on(idx);

    ke_state_set(pid, lc_env_ptr->link.saved_state);
}

void lc_qos_setup(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    uint8_t nb_bcst = lb_util_get_nb_broadcast();

    do
    {
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            if (!lc_env_ptr->link.QOSNotified)
            {
                // send LMP_QualityOfService(idx , PollInterval, nb_bcst , role)
                struct lmp_qos pdu;
                pdu.opcode = LMP_OPCODE(LMP_QOS_OPCODE, lc_env_ptr->link.Role);
                pdu.poll_intv = lc_env_ptr->link.PollInterval;
                pdu.nbc = nb_bcst;
                lc_send_lmp(idx, &pdu);

                lc_env_ptr->link.QOSNotified = true;

                lc_mst_qos_done(pid);
                break;
            }
        }
        // send LMP_QualityOfServiceReq(idx , PollInterval, nb_bcst , role)
        lc_send_pdu_qos_req(idx, nb_bcst, lc_env_ptr->link.PollInterval, lc_env_ptr->link.Role);

        lc_start_lmp_to(pid);

        ke_state_set(pid, LC_WAIT_QOS_CFM);
    }
    while (0);
}

void lc_rd_rem_name(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->req.LocNameReq = false;
    {
        struct lmp_name_req pdu;

        // fill the parameters
        pdu.opcode = LMP_OPCODE(LMP_NAME_REQ_OPCODE, lc_env_ptr->link.Role);
        pdu.offset = 0;

        // send the lmp pdu
        lc_send_lmp(idx, &pdu);
    }

    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_REM_NAME);
}

void lc_rem_switch(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint32_t MasterClock;
    uint8_t status = CO_ERROR_INVALID_LMP_PARAM;

    ld_acl_flow_off(idx);

    // Check SwitchInstant validity
    if (lc_env_ptr->link.SwitchInstant <= (BT_CLOCK_MSB - 1))
    {
        MasterClock = CLK_ADD_2(ld_read_clock(), ld_acl_clock_offset_get(idx).hs);

        // If SwitchInstant is in the past
        if (CLK_LOWER_EQ(2 * lc_env_ptr->link.SwitchInstant, MasterClock))
        {
            status = CO_ERROR_INSTANT_PASSED;
        }
        else
        {
            status = CO_ERROR_NO_ERROR;
        }
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            // Get slot offset
            uint16_t slot_offset = ld_acl_rsw_slot_offset_get(idx);

            lc_stop_afh_reporting(pid);

            // send LMP_SlotOffset
            lc_send_pdu_slot_off(idx, slot_offset, lc_env_ptr->link.RxTrIdServer);
        }
        // LMP_Accepted (LMP_SwitcReq)
        lc_send_pdu_acc(idx, LMP_SWITCH_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);

        // Trigger role switch
        status = ld_acl_rsw_req(idx, lc_env_ptr->link.LtAddr, lc_env_ptr->link.SlotOffset, lc_env_ptr->link.SwitchInstant, lm_page_scan_rep_mode_get());

        if (status == CO_ERROR_NO_ERROR)
        {
            // if SAM enabled, disable prior to switch instant
            lc_sam_disable(idx);

            ke_state_set(pid, LC_WAIT_SWITCH_CMP);
        }
        else
        {
            lc_switch_cmp(pid, status);
        }
    }
    else
    {
        // send LMP_NotAccepted(LMP_SWITCH_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SWITCH_REQ_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

        lc_switch_cmp(pid, status);
    }
}

void lc_rem_enc(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    lc_env_ptr->req.PeerEncReq = false;
    ld_acl_flow_off(idx);

    // LMP_Accepted(LMP_ENC_MODE_REQ_OPCODE)
    lc_send_pdu_acc(idx, LMP_ENC_MODE_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);

    lc_env_ptr->link.Initiator = false;
    lc_env_ptr->req.LocEncReq = false;

    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        lc_start_lmp_to(pid);
        if (lc_env_ptr->enc.NewEncMode == ENC_DISABLED)
        {
            ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
        }
        else
        {
            ke_state_set(pid, LC_WAIT_ENC_SLV_SIZE);
        }
    }
    else
    {
        if (lc_env_ptr->enc.NewEncMode == ENC_DISABLED)
        {
            lc_stop_enc(pid);
        }
        else
        {
            lc_start_enc_key_size(pid);
        }
    }
}

void lc_rem_name_cont(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->link.HostConnected == true)
    {
        // Send remote name request complete event
        struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
        evt->status = CO_ERROR_NO_ERROR;
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        memcpy(&evt->name.name[0], &lc_env_ptr->info.name_rem.name[0], lc_env_ptr->info.name_rem.namelen);
        memset(&evt->name.name[lc_env_ptr->info.name_rem.namelen], 0, BD_NAME_SIZE - lc_env_ptr->info.name_rem.namelen);
        hci_send_2_host(evt);

        ke_state_set(pid, LC_CONNECTED);
    }
    else
    {
        lc_env_ptr->info.SendRemoteNameCfm = true;
        lc_detach(pid, CO_ERROR_REMOTE_USER_TERM_CON);
    }
}

void lc_sec_auth_compute_sres(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // In secure authentication, compute both signed responses
    uint8_t temp[16];
    uint8_t *master_bd = (lc_env_ptr->link.Role) ? &lc_env_ptr->info.BdAddr.addr[0]      : &lc_env_ptr->info.LocalBdAddr.addr[0];
    uint8_t *slave_bd  = (lc_env_ptr->link.Role) ? &lc_env_ptr->info.LocalBdAddr.addr[0] : &lc_env_ptr->info.BdAddr.addr[0];
    uint8_t *master_rand = (lc_env_ptr->link.Role) ? &lc_env_ptr->enc.RandomRx.ltk[0] : &lc_env_ptr->enc.RandomTx.ltk[0];
    uint8_t *slave_rand  = (lc_env_ptr->link.Role) ? &lc_env_ptr->enc.RandomTx.ltk[0] : &lc_env_ptr->enc.RandomRx.ltk[0];

    uint8_t *loc_sres = (lc_env_ptr->link.Role) ? temp + 8          : temp + 8 + SRES_LEN;
    uint8_t *rem_sres = (lc_env_ptr->link.Role) ? temp + 8 + SRES_LEN : temp + 8;
    uint8_t *aco      = temp;

    // Compute authentication key
    sha_256_h4(&lc_env_ptr->enc.LTKey.ltk[0],
               master_bd,
               slave_bd,
               &lc_env_ptr->enc.auth_key.ltk[0]);

    // Compute signed responses SRES and ACO
    sha_256_h5(&lc_env_ptr->enc.auth_key.ltk[0],
               master_rand,
               slave_rand,
               temp);

    // Signed response TX
    memcpy(&lc_env_ptr->enc.Sres.nb[0], loc_sres, SRES_LEN);
    // Signed response RX
    memcpy(&lc_env_ptr->enc.Sres_expected.nb[0], rem_sres, SRES_LEN);
    // ACO
    memcpy(&lc_env_ptr->enc.Aco.a[0], aco, 8);

}

void lc_resp_sec_auth(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // Make random vector
    LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);
    // send LMP_AuRand
    lc_send_pdu_au_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.RxTrIdServer);

    // Compute both signed responses with ACO
    lc_sec_auth_compute_sres(pid);

    // If slave in secure authentication, send the local signed response
    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);
    }

    // start timer for LMP_sres
    lc_start_lmp_to(pid);
    ke_state_set(pid, LC_WAIT_SRES_RSP);
}

void lc_resp_auth(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->enc.KeyStatus == KEY_ABSENT)
    {
        if (lm_look_for_stored_link_key(&lc_env_ptr->info.BdAddr, &lc_env_ptr->enc.LTKey))
        {
            lc_env_ptr->enc.KeyStatus = KEY_PRESENT;
        }
    }

    if (lc_env_ptr->enc.KeyStatus == KEY_PRESENT)
    {
        if (lc_env_ptr->sp.sec_con)
        {
            // Responder of secure authentication
            lc_resp_sec_auth(pid);
        }
        else
        {
            // E1 procedure, compute Sres and Aco
            E1(lc_env_ptr->enc.LTKey, lc_env_ptr->info.LocalBdAddr,
               lc_env_ptr->enc.RandomRx, &lc_env_ptr->enc.Sres,
               &lc_env_ptr->enc.Aco);

            // send LMP_Sres
            lc_send_pdu_sres(idx, &lc_env_ptr->enc.Sres, lc_env_ptr->link.RxTrIdServer);

            lc_env_ptr->req.PeerAuthReq = false;
        }
    }
    else if (lc_env_ptr->enc.KeyStatus == KEY_ABSENT)
    {
        lc_env_ptr->enc.KeyStatus = KEY_PENDING;
        // allocate the complete event message
        struct hci_lk_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_LK_REQ_EVT_CODE, hci_lk_req_evt);
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        // send the message
        hci_send_2_host(evt);

        // start 30s timer
        ke_timer_set(LC_QUERY_HOST_EXP_IND, pid, LMP_RSP_TO * 1000);
        // set to new state
        ke_state_set(pid, LC_WAIT_LINK_KEY);
    }
}

void lc_resp_oob_wait_nonce(ke_task_id_t pid)
{
    lc_start_lmp_to(pid);
    ke_state_set(pid, LC_WAIT_OOB_RANDN_RSP_PEER);
}

void lc_resp_oob_nonce(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status;

    do
    {
        if (lc_env_ptr->sp.SpTId == lc_env_ptr->link.RxTrIdServer)
        {
            if (!lc_env_ptr->sp.SPPhase1Failed)
            {
                // LMP_Accepted(LMP_SP_NB_OPCODE,sptid)
                lc_send_pdu_acc(idx, LMP_SP_NB_OPCODE, lc_env_ptr->sp.SpTId);

                lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);

                lc_start_lmp_to(pid);

                // send LMP_SimplePairingNumber(idx,LocRandN,sptid)
                lc_send_pdu_sp_nb(idx, &lc_env_ptr->sp.LocRandN, lc_env_ptr->sp.SpTId);

                ke_state_set(pid, LC_WAIT_OOB_RANDN_RSP_PEER_CFM);
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

        // send LMP_NotAccepted(LMP_SP_NB_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SP_NB_OPCODE, status, lc_env_ptr->link.RxTrIdServer);

        lc_sp_fail(pid);


    }
    while (0);
}

void lc_resp_calc_f3(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t *p_dh_key = lc_env_ptr->sp.p_data->dh_key.key;

    sha_256_f3(lc_env_ptr->sp.sec_con, p_dh_key,
               &lc_env_ptr->sp.RemRandN.A[0],
               &lc_env_ptr->sp.LocRandN.A[0],
               &lc_env_ptr->sp.LocCommitment.A[0],
               (uint8_t *) &lc_env_ptr->sp.IOCap_rem,
               &lc_env_ptr->info.BdAddr.addr[0],
               &lc_env_ptr->info.LocalBdAddr.addr[0],
               &lc_env_ptr->sp.LocCommitment.A[0]);

    do
    {
        if (!memcmp(&lc_env_ptr->sp.LocCommitment, &lc_env_ptr->sp.DHKeyCheck, KEY_LEN))
        {
            // send LMP_Accepted(idx LMP_DHKeyCheck,sptid)
            lc_send_pdu_acc(idx, LMP_DHKEY_CHK_OPCODE, lc_env_ptr->sp.SpTId);

            sha_256_f3(lc_env_ptr->sp.sec_con, p_dh_key,
                       &lc_env_ptr->sp.LocRandN.A[0],
                       &lc_env_ptr->sp.RemRandN.A[0],
                       &lc_env_ptr->sp.RemCommitment.A[0],
                       (uint8_t *) &lc_env_ptr->sp.IOCap_loc,
                       &lc_env_ptr->info.LocalBdAddr.addr[0],
                       &lc_env_ptr->info.BdAddr.addr[0],
                       &lc_env_ptr->sp.DHKeyCheck.ltk[0]);

            lc_send_pdu_dhkey_chk(idx, &lc_env_ptr->sp.DHKeyCheck, lc_env_ptr->sp.SpTId);

            lc_start_lmp_to(pid);

            ke_state_set(pid, LC_WAIT_DHKEY_CHECK_RSP_PEER_CFM);

            break;
        }

        // send LMP_NotAccepted(LMP_DHKEY_CHK_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_DHKEY_CHK_OPCODE, CO_ERROR_AUTH_FAILURE, lc_env_ptr->sp.SpTId);

        lc_sp_fail(pid);
    }
    while (0);
}

void lc_release(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    uint8_t status = CO_ERROR_UNDEFINED;

    //Disconnect link
    status = ld_acl_stop(idx);

    if (status == CO_ERROR_NO_ERROR)
    {
        ke_state_set(pid, LC_WAIT_DISC_CFM);
    }
    else
    {
        ASSERT_ERR_FORCE(0); // should not reach here
    }
}

void lc_restart_enc(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->req.RestartEncReq = true;
    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        lc_start_lmp_to(pid);
        ke_state_set(pid, LC_WAIT_EPR_ENC_STOP_REQ_SLV_INIT);
    }
    else
    {
        lc_env_ptr->enc.NewEncMode = ENC_DISABLED;
        lc_send_enc_mode(pid);
    }

}

void lc_restart_enc_cont(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if ((reason != CO_ERROR_NO_ERROR) || (lc_env_ptr->enc.EncMode != ENC_DISABLED))
    {
        lc_env_ptr->req.RestartEncReq = false;

#if BCAST_ENC_SUPPORT
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            struct lb_enc_restart_cfm *cfm = KE_MSG_ALLOC(LB_ENC_RESTART_CFM, TASK_LB, pid, lb_enc_restart_cfm);
            cfm->status = reason;
            ke_msg_send(cfm);
        }
        else
        {
            if (lc_env_ptr->req.MasterKeyReq)
            {
                uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
                struct hci_master_lk_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MASTER_LK_CMP_EVT_CODE, hci_master_lk_cmp_evt);
                lc_env_ptr->req.MasterKeyReq = false;

                evt->status = reason;
                evt->conhdl = conhdl;
                evt->key_flag = lc_env_ptr->enc.KeyFlag;
                hci_send_2_host(evt);
            }
        }
#endif // BCAST_ENC_SUPPORT

        ke_state_set(pid, LC_CONNECTED);
    }
    else
    {
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_env_ptr->enc.NewEncMode = ENC_PP_ENABLED;
            lc_send_enc_mode(pid);
        }
        else
        {
            lc_start_lmp_to(pid);
            ke_state_set(pid, LC_WAIT_ENC_SLV_RESTART);
        }
    }
}

void lc_restore_to(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t tmp_to = ld_acl_lsto_get(idx);

    //if link timeout need to be modified
    if (tmp_to != lc_env_ptr->link.LinkTimeout)
    {
        ld_acl_lsto_set(idx, tmp_to);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            //  send LMP_SupervisionTimeout
            lc_send_pdu_lsto(idx, tmp_to, lc_env_ptr->link.Role);
        }
    }
}

void lc_start_lmp_to(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint32_t lmp_to_val = LMP_RSP_TO * 100;

    if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
    {
        uint16_t interval;

        lc_sniff_interval_get(idx, &interval, NULL);

        lmp_to_val += 2 * co_slot_to_duration(interval);
    }

    ke_timer_set(LC_LMP_RSP_TO, pid, 10 * lmp_to_val);
}

void lc_start_enc(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t role;
    struct ltk temp_key;

    if (!lc_env_ptr->sp.sec_con)
    {
#if BCAST_ENC_SUPPORT
        if (lc_env_ptr->enc.KeyFlag == TEMPORARY_KEY)
        {
            struct aco aco_tmp;
            LM_GetMasterEncRand(&lc_env_ptr->enc.RandomTx);
            LM_MakeCof(lc_env_ptr->info.LocalBdAddr, &aco_tmp);
            E3(lc_env_ptr->enc.LTKey, aco_tmp, lc_env_ptr->enc.RandomTx, &temp_key);
        }
        else
#endif // BCAST_ENC_SUPPORT
        {
            LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);
            E3(lc_env_ptr->enc.LTKey, lc_env_ptr->enc.Aco, lc_env_ptr->enc.RandomTx,
               &temp_key);
        }

#if (EAVESDROPPING_SUPPORT)
        // Save the IV
        memcpy(&lc_env_ptr->enc.EncIV, &lc_env_ptr->enc.Aco, IV_LEN);
#endif // EAVESDROPPING_SUPPORT

        KPrimC(temp_key, lc_env_ptr->enc.EncSize, &lc_env_ptr->enc.EncKey);
        ld_acl_enc_key_load(idx, &lc_env_ptr->enc.EncKey, NULL);
        ld_acl_rx_enc(idx, LD_ENC_E0);
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

        // Start encryption in RX
        ld_acl_rx_enc(idx, LD_ENC_AES);
    }

    role = (lc_env_ptr->link.Initiator == true) ? lc_env_ptr->link.Role : lc_env_ptr->link.RxTrIdServer;

    lc_start_lmp_to(pid);
    {
        // send LMP_StartEncryptionReq
        struct lmp_start_enc_req pdu;

        pdu.opcode = LMP_OPCODE(LMP_START_ENC_REQ_OPCODE, role);
        memcpy(&pdu.random.ltk[0], &lc_env_ptr->enc.RandomTx.ltk[0], KEY_LEN);
        lc_send_lmp(idx, &pdu);
    }
    ke_state_set(pid, LC_WAIT_ENC_START_CFM);
}

void lc_start_enc_key_size(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t role;
    lc_env_ptr->req.PeerEncReq = false;

#if BCAST_ENC_SUPPORT
    if (lc_env_ptr->enc.KeyFlag == TEMPORARY_KEY)
    {
        lc_env_ptr->enc.EncSize = LM_GetMasterEncKeySize();
    }
    else
#endif // BCAST_ENC_SUPPORT
    {
#if (BT_53) && (RW_DEBUG)
        lc_env_ptr->enc.EncSize = lc_env_ptr->enc.max_enc_key_size;
#else // (BT_53) && (RW_DEBUG)
        lc_env_ptr->enc.EncSize = ENC_KEY_SIZE_MAX;
#endif // (BT_53) && (RW_DEBUG)
    }

    role = (lc_env_ptr->link.Initiator == true) ? lc_env_ptr->link.Role : lc_env_ptr->link.RxTrIdServer;

    // send LMP_encryptionKeySizeReq
    lc_send_pdu_enc_key_sz_req(idx, lc_env_ptr->enc.EncSize, role);

    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_ENC_SIZE_CFM);
}

void lc_start_key_exch(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    struct ltk temp_key;

    lc_start_lmp_to(pid);

    //Create a random vector
    LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);

    //Xor random with current link key
    XorKey(lc_env_ptr->enc.RandomTx, lc_env_ptr->enc.LTKey, &temp_key);

    // send LMP_CombKey(idx,temp_key1,rx_tr_id)
    lc_send_pdu_comb_key(idx, &temp_key, lc_env_ptr->link.Role);

    ke_state_set(pid, LC_WAIT_KEY_EXCH);
}

void lc_start_passkey(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    const uint8_t AUTH1_MAPPING_TABLE[4][4] =
    {
        {0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00},
        {0x01, 0x01, 0x01, 0x00},
        {0x00, 0x00, 0x00, 0x00}
    };

    if ((AUTH1_MAPPING_TABLE[lc_env_ptr->sp.IOCap_loc.io_cap][lc_env_ptr->sp.IOCap_rem.io_cap]) == 1)
    {
        struct hci_user_passkey_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_USER_PASSKEY_REQ_EVT_CODE, hci_user_passkey_req_evt);
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0],
               BD_ADDR_LEN);
        hci_send_2_host(evt);

        lc_start_lmp_to(pid);
        ke_state_set(pid, LC_WAIT_PASSKEY_HL_RPLY);
    }
    else
    {
        struct hci_user_passkey_notif_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_USER_PASSKEY_NOTIF_EVT_CODE, hci_user_passkey_notif_evt);
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0],
               BD_ADDR_LEN);
        lc_env_ptr->sp.Passkey = (uint32_t)CO_MOD(co_rand_word(), 1000000);
        evt->passkey = lc_env_ptr->sp.Passkey;
        hci_send_2_host(evt);

        lc_start_lmp_to(pid);
        lc_start_passkey_loop(pid);
    }
}

void lc_start_passkey_loop(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->sp.EncapPduCtr = 0;
    if (lc_env_ptr->sp.SPInitiator)
    {
        if (lc_env_ptr->sp.SPPhase1Failed)
        {
            // send LMP_passkey_fail(idx, sptid)
            {
                struct lmp_passkey_fail pdu;

                pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->sp.SpTId);
                pdu.ext_opcode = LMP_PASSKEY_FAIL_EXTOPCODE;
                lc_send_lmp(idx, &pdu);
            }
            lc_sp_fail(pid);
        }
        else
        {
            lc_init_passkey_loop(pid);
        }
    }
    else
    {
        lc_start_lmp_to(pid);

        ke_state_set(pid, LC_WAIT_PASSKEY_COMM_RSP_PEER);
    }
}

void lc_start_oob(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->sp.IOCap_loc.oob_data_present)
    {
        struct hci_rem_oob_data_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_OOB_DATA_REQ_EVT_CODE, hci_rem_oob_data_req_evt);
        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(evt);

        lc_start_lmp_to(pid);
        ke_state_set(pid, LC_WAIT_HL_OOB_DATA);
    }
    else
    {
        memset(&lc_env_ptr->sp.RemCommitment.A[0], 0x00, sizeof(struct byte16));
        lc_skip_hl_oob_req(pid);
    }
}

void lc_resp_num_comp(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);

    uint8_t Z = 0x00;
    const uint8_t *p_local_pub_key_x = &(lc_env_ptr->sp.p_data->local_pub_key.x[0]);
    const uint8_t *p_remote_pub_key_x = &(lc_env_ptr->sp.p_data->remote_pub_key.x[0]);

    sha_256_f1(lc_env_ptr->sp.sec_con, p_local_pub_key_x, p_remote_pub_key_x,
               &lc_env_ptr->sp.LocRandN.A[0],
               &Z,
               &lc_env_ptr->sp.LocCommitment.A[0]);

    // send LMP_SimplePairingConfirm
    lc_send_pdu_sp_cfm(idx, &lc_env_ptr->sp.LocCommitment, lc_env_ptr->sp.SpTId);

    lc_start_lmp_to(pid);
    ke_state_set(pid, LC_WAIT_NUM_COMP_RANDN_RSP_PEER);
}

void lc_stop_enc(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t role;

    lc_start_lmp_to(pid);

    ld_acl_rx_enc(idx, LD_ENC_DISABLE);
    lc_env_ptr->req.PeerEncReq = false;
    role = (lc_env_ptr->link.Initiator == true) ? lc_env_ptr->link.Role : lc_env_ptr->link.RxTrIdServer;
    {
        // send LMP_StopEncryptionReq
        struct lmp_stop_enc_req pdu;
        // fill event parameters
        pdu.opcode = LMP_OPCODE(LMP_STOP_ENC_REQ_OPCODE, role);
        // send lmp pdu
        lc_send_lmp(idx, &pdu);
    }

    ke_state_set(pid, LC_WAIT_ENC_STOP_CFM_MST);
}

void lc_stop_afh_reporting(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->afh.temp_en = lc_env_ptr->afh.en;
    lc_env_ptr->afh.en = false;
    ke_timer_clear(LC_AFH_REPORT_TO, pid);
}

void lc_restore_afh_reporting(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->afh.en = lc_env_ptr->afh.temp_en;

    if (lc_env_ptr->afh.reporting_en && lc_env_ptr->afh.en)
    {
        // Restart reporting timer
        ke_timer_set(LC_AFH_REPORT_TO, pid, 10 * co_slot_to_duration(lc_env_ptr->afh.reporting_interval));
    }
}

void lc_skip_hl_oob_req(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr->sp.SPInitiator)
    {
        lm_generate_rand_16bytes(&lc_env_ptr->sp.LocRandN);

        lc_start_lmp_to(pid);
        lc_send_pdu_sp_nb(idx, &lc_env_ptr->sp.LocRandN, lc_env_ptr->sp.SpTId);

        ke_state_set(pid, LC_WAIT_OOB_RANDN_INIT_CFM);
    }
    else
    {
        lc_resp_oob_wait_nonce(pid);
    }
}

void lc_send_enc_mode(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.Initiator = true;
    lc_env_ptr->req.LocEncReq = false;
    ld_acl_flow_off(idx);

    {
        // send LMP_EncryptionModeReq
        struct lmp_enc_mode_req pdu;
        pdu.opcode  = LMP_OPCODE(LMP_ENC_MODE_REQ_OPCODE, lc_env_ptr->link.Role);
        pdu.enc_mode = lc_env_ptr->enc.NewEncMode;
        lc_send_lmp(idx, &pdu);
    }
    lc_start_lmp_to(pid);
    ke_state_set(pid, LC_WAIT_ENC_MODE_CFM);
}

void lc_semi_key_cmp(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // no need to send the encryption change event
    // this event is optional for master link key command
    if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
    {
        // prevent the sending of encryption change event
        lc_env_ptr->enc.PreventEncEvt = true;
    }

#if BCAST_ENC_SUPPORT
    if (lc_env_ptr->link.Role == SLAVE_ROLE)
    {
        if (lc_env_ptr->enc.EncMode == ENC_DISABLED)
        {
            uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);
            struct hci_master_lk_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MASTER_LK_CMP_EVT_CODE, hci_master_lk_cmp_evt);
            lc_env_ptr->req.MasterKeyReq = false;

            evt->status = reason;
            evt->conhdl = conhdl;
            evt->key_flag = lc_env_ptr->enc.KeyFlag;
            hci_send_2_host(evt);
        }
    }
    else
    {
        struct lb_mst_key_cfm *msg = KE_MSG_ALLOC(LB_MST_KEY_CFM, TASK_LB, pid, lb_mst_key_cfm);
        lc_env_ptr->req.MasterKeyReq = false;

        msg->status = reason;
        msg->enc_mode = lc_env_ptr->enc.EncMode;
        msg->key_sz_msk = lc_env_ptr->enc.KeySizeMask;
        ke_msg_send(msg);
    }
#endif // BCAST_ENC_SUPPORT

    if (reason != CO_ERROR_NO_ERROR)
    {
        lc_detach(pid, reason);
    }
    else
    {
        if (lc_env_ptr->enc.EncMode != ENC_DISABLED)
        {
            lc_restart_enc(pid);
        }
        else
        {
            ke_state_set(pid, LC_CONNECTED);
        }
    }
}

void lc_detach(ke_task_id_t pid, uint8_t reason)
{
    uint32_t timer_duration = 0;

    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.Reason = reason;

    ld_acl_flow_off(idx);

    {
        // LMP_Detach
        struct lmp_detach pdu;
        pdu.opcode = LMP_OPCODE(LMP_DETACH_OPCODE, lc_env_ptr->link.Role);
        pdu.reason  = lc_env_ptr->link.Reason;
        lc_send_lmp(idx, &pdu);
    }

    ke_timer_clear(LC_LMP_RSP_TO, pid);

    timer_duration = 6 * ld_acl_t_poll_get(idx);

    if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
    {
        uint16_t interval;

        lc_sniff_interval_get(idx, &interval, NULL);

        timer_duration += (interval * 2);
    }
    timer_duration = co_slot_to_duration(timer_duration);
    ke_timer_set(LC_LMP_RSP_TO, pid, 10 * timer_duration);
    ke_state_set(pid, LC_WAIT_DETACH_REQ_TX_CFM);

    if (lc_env_ptr->sp.sec_con)
    {
        // Stop authenticated payload timers
        ke_timer_clear(LC_AUTH_PAYL_NEARLY_TO, pid);
        ke_timer_clear(LC_AUTH_PAYL_REAL_TO, pid);
    }
}

void lc_switch_cmp(ke_task_id_t pid, uint8_t status)
{
    int idx = KE_IDX_GET(pid);

    ld_acl_flow_on(idx);

    lc_rsw_done(pid, status);
}

void lc_sp_fail(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.Reason = CO_ERROR_AUTH_FAILURE;

    if (lc_env_ptr->enc.LinkKeyValid)
    {
        // set key status
        lc_env_ptr->enc.KeyStatus = KEY_PRESENT;
    }

    if (lc_env_ptr->sp.sec_con)
    {
        // Abort a potential ongoing DH key generation
        ecc_p256_abort_key_gen(pid);
    }
    else
    {
        ecc_p192_abort_key_gen(pid);
    }

    // End of simple pairing
    lc_sp_end(idx, CO_ERROR_AUTH_FAILURE);

    // mutual authentication procedure, status failure
    lc_mutual_auth_end(pid, CO_ERROR_AUTH_FAILURE);
}

void lc_resp_pair(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // set simple pairing variables to init values
    lc_env_ptr->sp.dh_key_compute_state   = LC_DH_KEY_COMP_PENDING;
    lc_env_ptr->sp.SPPhase1Failed    = 0;
    lc_env_ptr->sp.SPInitiator       = false;

    lc_env_ptr->sp.p_data = ke_malloc_user(sizeof(lc_sp_data_t), KE_MEM_KE_MSG);

    if (lc_env_ptr->sp.p_data == NULL)
    {
        // sp failure, invalid index
        lc_sp_fail(pid);
    }
    else
    {
        // allocate the response event message
        struct hci_io_cap_rsp_evt *evt1 = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_IO_CAP_RSP_EVT_CODE, hci_io_cap_rsp_evt);

        // fill the event
        evt1->auth_req      = lc_env_ptr->sp.IOCap_rem.aut_req;
        evt1->io_capa       = lc_env_ptr->sp.IOCap_rem.io_cap;
        evt1->oob_data_pres = lc_env_ptr->sp.IOCap_rem.oob_data_present;
        memcpy(&evt1->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);

        // send 1st event
        hci_send_2_host(evt1);

        {
            // allocate the request event message
            struct hci_io_cap_req_evt *evt2 = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_IO_CAP_REQ_EVT_CODE, hci_io_cap_req_evt);

            memcpy(&evt2->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
            // send 2nd event
            hci_send_2_host(evt2);
        }

        lc_start_lmp_to(pid);
        // set new state
        ke_state_set(pid, LC_WAIT_HL_IO_CAP_RSP);
    }
}

void lc_feat(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // send LMP_FeatureReq(idx, local features,role)
    struct lmp_feats_req pdu;
    lm_read_features(FEATURE_PAGE_0, NULL, &pdu.feats);
    pdu.opcode = LMP_OPCODE(LMP_FEATS_REQ_OPCODE, lc_env_ptr->link.Role);
    lc_send_lmp(idx, &pdu);

    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_REM_FEATS);
}

void lc_hl_connect(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    // send LMP_HostConnectionReq(idx,role)
    struct lmp_host_con_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_HOST_CON_REQ_OPCODE, lc_env_ptr->link.Role);
    lc_send_lmp(idx, &pdu);

    ke_state_set(pid, LC_WAIT_REM_HL_CON);
}


void lc_version(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    // send LMP_VersionReq(idx , LMP_VERSION,LMP_MANUFACTURER_NAME,LMP_SUB_VERSION, role)
    lc_send_pdu_vers_req(idx, lc_env_ptr->link.Role);
    lc_start_lmp_to(pid);
    ke_state_set(pid, LC_WAIT_REM_VERS);
}

void lc_ext_feat(ke_task_id_t pid, int idx, uint8_t page_nb)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    // send LMP_FeaturesReqExt
    lc_send_pdu_feats_ext_req(idx, page_nb, lc_env_ptr->link.Role);

    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_REM_EXT_FEATS);
}

void lc_pair(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    do
    {
        if (lm_get_sp_en())
        {
            if (LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_SSP_BIT_POS)
                    && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_1], FEAT_SSP_HOST_BIT_POS))
            {
                // Secure Simple Pairing Procedure
                lc_env_ptr->sp.p_data = ke_malloc_user(sizeof(lc_sp_data_t), KE_MEM_KE_MSG);

                if (lc_env_ptr->sp.p_data == NULL)
                {
                    lc_sp_fail(pid);
                }
                else
                {
                    // Simple Pairing Flags
                    lc_env_ptr->sp.dh_key_compute_state = LC_DH_KEY_COMP_PENDING;
                    lc_env_ptr->sp.SPPhase1Failed = 0;
                    lc_env_ptr->sp.SPInitiator = true;
                    lc_env_ptr->sp.SpTId = lc_env_ptr->link.Role;

                    {
                        // allocate the status event message
                        struct hci_io_cap_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_IO_CAP_REQ_EVT_CODE, hci_io_cap_req_evt);
                        memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                        hci_send_2_host(evt);
                    }

                    // start timer for IO capability
                    lc_start_lmp_to(pid);

                    ke_state_set(pid, LC_WAIT_HL_IO_CAP_INIT);
                }
                break;
            }
        }

        // If not simple pairing, do legacy pairing
        lc_legacy_pair(pid);

    }
    while (0);
}

void lc_loc_auth(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    switch (lc_env_ptr->enc.KeyStatus)
    {
    case KEY_ABSENT:
        if (!lm_get_sp_en())
        {
            if (lm_look_for_stored_link_key(&lc_env_ptr->info.BdAddr, &lc_env_ptr->enc.LTKey))
            {
                // key status set to PRESENT
                lc_env_ptr->enc.KeyStatus = KEY_PRESENT;

                // create random vector
                LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);

                // send LMP au_rand
                lc_send_pdu_au_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.Role);

                lc_start_lmp_to(pid);

                if (lc_env_ptr->sp.sec_con)
                {
                    ke_state_set(pid, LC_WAIT_AU_RAND_SEC_AUTH_INIT);
                }
                else
                {
                    ke_state_set(pid, LC_WAIT_AUTH_SRES);
                }
            }
            else
            {
                // key status set to PENDING
                lc_env_ptr->enc.KeyStatus = KEY_PENDING;
                {
                    // allocate event message
                    struct hci_lk_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_LK_REQ_EVT_CODE, hci_lk_req_evt);
                    memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                    hci_send_2_host(evt);
                }
                // start 30s timer
                ke_timer_set(LC_QUERY_HOST_EXP_IND, pid, LMP_RSP_TO * 1000);
                // set to new state
                ke_state_set(pid, LC_WAIT_LINK_KEY);
            }
        }
        else
        {
            lc_env_ptr->enc.KeyStatus = KEY_PENDING;
            {
                // allocate event message
                struct hci_lk_req_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_LK_REQ_EVT_CODE, hci_lk_req_evt);
                memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
                hci_send_2_host(evt);
            }
            // start 30s timer
            ke_timer_set(LC_QUERY_HOST_EXP_IND, pid, LMP_RSP_TO * 1000);
            // set to new state
            ke_state_set(pid, LC_WAIT_LINK_KEY);
        }
        break;
    case KEY_PRESENT:
        LM_MakeRandVec(&lc_env_ptr->enc.RandomTx);
        // send LMP au rand message to peer
        lc_send_pdu_au_rand(idx, &lc_env_ptr->enc.RandomTx, lc_env_ptr->link.Role);

        // start lmp timer
        lc_start_lmp_to(pid);

        if (lc_env_ptr->sp.sec_con)
        {
            ke_state_set(pid, LC_WAIT_AU_RAND_SEC_AUTH_INIT);
        }
        else
        {
            ke_state_set(pid, LC_WAIT_AUTH_SRES);
        }

        break;
    case KEY_PENDING:
    default:
        ASSERT_ERR_FORCE(0); // should not reach here
        break;
    }
}

void lc_packet_type(ke_task_id_t pid, int idx)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    lc_env_ptr->link.CurPacketType = lc_env_ptr->link.AclPacketType;

    if (LM_CheckEdrFeatureRequest(lc_env_ptr->link.CurPacketType) && LM_GetFeature(&lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0], FEAT_EDR_2MB_BIT_POS))
    {
        if (lc_env_ptr->link.PacketTypeTable2Mb)
        {
            lc_chg_pkt_type_retry(pid);
        }
        else
        {
            lc_env_ptr->link.PacketTypeTable2Mb = true;

            lc_env_ptr->link.ptt_tmp = PACKET_TABLE_2_3MBPS;

            ld_acl_flow_off(idx);

            lc_send_pdu_ptt_req(idx, lc_env_ptr->link.ptt_tmp, lc_env_ptr->link.Role);
            lc_start_lmp_to(pid);
            ke_state_set(pid, LC_WAIT_PKT_TBL_TYP_ACC_CFM);
        }
    }
    else
    {
        if (!lc_env_ptr->link.PacketTypeTable2Mb)
        {
            lc_chg_pkt_type_retry(pid);
        }
        else
        {
            lc_env_ptr->link.PacketTypeTable2Mb = false;

            lc_env_ptr->link.ptt_tmp = PACKET_TABLE_1MBPS;

            ld_acl_flow_off(idx);

            lc_send_pdu_ptt_req(idx, lc_env_ptr->link.ptt_tmp, lc_env_ptr->link.Role);
            lc_start_lmp_to(pid);
            ke_state_set(pid, LC_WAIT_PKT_TBL_TYP_ACC_CFM);
        }
    }
}

bool lc_conn_seq_done(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    if (lc_env_ptr->link.SetupCompRx == true && lc_env_ptr->link.SetupCompTx == true)
    {
        return true;
    }
    return false;
}

void lc_send_enc_chg_evt(uint8_t idx, uint8_t status)
{
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint16_t conhdl = (BT_ACL_CONHDL_MIN + idx);

#if (BT_53)
    /* If an event has more than one version and the event is generated, the Controller shall use the latest version
     * that is enabled ("unmasked") in the relevant event mask - vol4e 4.8 versioned events */
    if (hci_evtcode_mask_check(HCI_ENC_CHG_V2_EVT_CODE))
    {
        struct hci_enc_chg_v2_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_ENC_CHG_V2_EVT_CODE, hci_enc_chg_v2_evt);

        evt->status = status;
        evt->conhdl = conhdl;
        evt->enc_stat = (lc_env_ptr->enc.EncEnable) ? (ENC_BRDER_E0_LE_AESCCM + lc_env_ptr->sp.sec_con) : ENC_OFF;
        evt->enc_key_size = lc_env_ptr->enc.EncSize;
        hci_send_2_host(evt);
    }
    else
#endif // (BT_53)
    {
        struct hci_enc_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_ENC_CHG_EVT_CODE, hci_enc_chg_evt);

        evt->status = status;
        evt->conhdl = conhdl;
        evt->enc_stat = (lc_env_ptr->enc.EncEnable) ? (ENC_BRDER_E0_LE_AESCCM + lc_env_ptr->sp.sec_con) : ENC_OFF;
        hci_send_2_host(evt);
    }
}

void lc_sp_end(uint8_t link_id, uint8_t reason)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    // Report simple pairing completion
    struct hci_sp_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_SP_CMP_EVT_CODE, hci_sp_cmp_evt);
    evt->status = reason;
    memcpy(&evt->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
    hci_send_2_host(evt);

    ASSERT_ERR(lc_env_ptr->sp.p_data != NULL);

    // free pairing data
    if (lc_env_ptr->sp.p_data != NULL)
    {
        ke_free(lc_env_ptr->sp.p_data);
        lc_env_ptr->sp.p_data = NULL;
    }
}

#if PCA_SUPPORT
uint8_t lc_prepare_all_slaves_for_clk_adj(void)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Parse all links
    for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
    {
        // Check if a master ACL connection is present
        if ((lc_env[idx] != NULL) && (lc_env[idx]->link.Role == MASTER_ROLE))
        {
            if (!LM_GetFeature(&lc_env[idx]->info.RemoteFeatures[FEATURE_PAGE_2], FEAT_COARSE_CLK_ADJ_BIT_POS))
            {
                /* Coarse clock adjustment can only be used when all slaves support it. BB 4.1.14.1 */
                status = CO_ERROR_CCA_REJ_USE_CLOCK_DRAG;
                break;
            }

            if (LC_CONNECTED != ke_state_get(KE_BUILD_ID(TASK_LC, idx)))
            {
                /* Coarse clock adjustmet should not be used if transactions outstanding. */
                status = CO_ERROR_DIFF_TRANSACTION_COLLISION;
            }
        }
    }

    if (CO_ERROR_NO_ERROR == status)
    {
        for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
        {
            // For each master ACL connection present
            if ((lc_env[idx] != NULL) && (lc_env[idx]->link.Role == MASTER_ROLE))
            {
                /* Prevent other procedures from initiating until after this procedure has completed */
                ke_state_set(KE_BUILD_ID(TASK_LC, idx), LC_WAIT_CLK_ADJ_CFM);
                lc_util_set_loc_trans_coll(idx, LMP_ESC4_OPCODE, LMP_CLK_ADJ_EXTOPCODE, LC_UTIL_INUSED);
            }
        }
    }

    return status;
}

void lc_notify_all_slaves_clk_adj_complete(void)
{
    for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
    {
        // For each master ACL connection present
        if ((lc_env[idx] != NULL) && (lc_env[idx]->link.Role == MASTER_ROLE))
        {
            ke_task_id_t tid = KE_BUILD_ID(TASK_LC, idx);

            if (LC_WAIT_CLK_ADJ_CFM == ke_state_get(tid))
            {
                /* Resume connected state */
                ke_state_set(KE_BUILD_ID(TASK_LC, idx), LC_CONNECTED);
                lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);
            }
        }
    }
}
#endif // PCA_SUPPORT

bool lc_check_all_slaves_support_sec_con(void)
{
    bool master_con_present = false;
    bool sec_con_not_supp = false;

    // Parse all links
    for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
    {
        // Check if a master ACL connection is present
        if ((lc_env[idx] != NULL) && (lc_env[idx]->link.Role == MASTER_ROLE))
        {
            // Indicate that at least one master connection is present
            master_con_present = true;

            // Check if slave supports secure connection
            if (!lc_env[idx]->sp.sec_con)
            {
                sec_con_not_supp = true;
                break;
            }
        }
    }

    return (master_con_present && !sec_con_not_supp);
}

#if (EAVESDROPPING_SUPPORT)
void lc_get_enc_params(uint8_t link_id, uint8_t *key, uint8_t *iv)
{
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];
    memcpy(key, &lc_env_ptr->enc.EncKey.ltk[0], sizeof(lc_env_ptr->enc.EncKey.ltk));
    memcpy(iv,  &lc_env_ptr->enc.EncIV.vect[0], IV_LEN);
}

/**
 ****************************************************************************************
 * @brief Indicate max slot update to all active links.
 ****************************************************************************************
 */
void lc_max_slot_update(void)
{
    // Send the request to all other active link(s)
    for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
    {
        // Check if connection is active
        if (ke_state_get(KE_BUILD_ID(TASK_LC, idx)) != LC_FREE)
        {
            ke_msg_send_basic(LC_SYNC_IND, KE_BUILD_ID(TASK_LC, idx), KE_BUILD_ID(TASK_LC, idx));
        }
    }
}
#endif // EAVESDROPPING_SUPPORT

#if (EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief This function send sniff request event
 *
 * @param[in] lc_env                lc_env           lc_env_tag pointer
 ****************************************************************************************
 */
void lc_send_sniff_request_evt(struct lc_env_tag *lc_env_ptr,
                               uint16_t sniff_interval, uint16_t sniff_delay,
                               uint16_t sniff_attempt, uint16_t sniff_timeout)
{
    if (lm_env.hci.notify_sniff_events)
    {
        // send the custom HCI-VS Sniff Request event
        struct hci_vs_sniff_request_evt *event =
            KE_MSG_ALLOC(HCI_DBG_EVT, 0, 0, hci_vs_sniff_request_evt);

        event->subcode = HCI_VS_SNIFF_REQUEST_EVT_SUBCODE;
        memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        event->role = lc_env_ptr->link.Role;
        event->sniff_interval = sniff_interval;
        event->sniff_delay = sniff_delay;
        event->sniff_attempt = sniff_attempt;
        event->sniff_timeout = sniff_timeout;
        hci_send_2_host(event);
    }
}
#endif // EAVESDROPPING_SUPPORT

/// @} LC
