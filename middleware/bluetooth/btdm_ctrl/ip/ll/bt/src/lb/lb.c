/**
 ****************************************************************************************
 *
 * @file lb.c
 *
 * @brief Definition of the functions used by the link broadcaster
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LB
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

#include "lb.h"          // link broadcaster definitions
#include "lb_int.h"      // link broadcaster internal definitions

#include "bt_util_key.h"
#include "reg_access.h"
#include "lm.h"
#include "lc.h"
#include "ld.h"          // link driver definitions
#include "hci.h"         // HCI definitions
#include "ke_timer.h"

#include "rwip.h"           // stack main module

/*
 * DEFINES
 ****************************************************************************************
 */

// Clock adjust default interval (slots)
#define LB_CLK_ADJ_DEFAULT_INTERVAL     (6 * POLL_INTERVAL_DFT)

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LB environment variable
struct lb_env_tag lb_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function to get the maximum encryption key size from encryption
 *        key size mask.
 *
 * @param[in] EncKeySizeMask    Encryption key mask
 *
 * @return Encryption key size
 ****************************************************************************************
 */
uint8_t LM_ExtractMaxEncKeySize(uint16_t EncKeySizeMask)
{
    int EncKeySize;

    // Keep only common EncKeySize
    EncKeySizeMask &= ENC_KEY_SIZE_MASK;

    // Check encryption size from 16 down to 1
    for (EncKeySize =  16 ; EncKeySize > 0 ; EncKeySize--)
    {
        // If MSB is 1 => exit loop
        if (EncKeySizeMask & 0x8000)
        {
            break;
        }
        EncKeySizeMask <<= 1;
    }
    return (uint8_t)EncKeySize;
}

#if BCAST_ENC_SUPPORT
/**
 ****************************************************************************************
 * @brief This function create a master key.
 ****************************************************************************************
 */
__STATIC void LM_CreateMasterKey(void)
{
    struct ltk Random1;
    struct ltk Random2;
    struct bd_addr BdAddr; /* not used */

    memset((uint8_t *)&BdAddr, 0, sizeof(BdAddr));

    LM_MakeRandVec(&Random1);
    LM_MakeRandVec(&Random2);

    E22(Random1, BdAddr, *((struct pin_code *)&Random2), 16, &lb_env.LM_MasterKey);
}

/**
 ****************************************************************************************
 * @brief This function create a master key random.
 ****************************************************************************************
 */
__STATIC void LM_CreateMasterKeyRand(void)
{
    LM_MakeRandVec(&lb_env.LM_MasterKeyRand);
}
#endif // BCAST_ENC_SUPPORT

#if PCA_SUPPORT
/**
******************************************************************************************
* @brief Function called to pack and broadcast an LMP PDU.
 ****************************************************************************************
 */
__STATIC void lb_send_lmp(void *param, uint8_t length, uint32_t clock, uint8_t num_bcst)
{
    struct bt_em_lmp_buf_elt *buf_elt = bt_util_buf_lmp_tx_alloc();

    if (buf_elt != NULL)
    {
        uint8_t status;

        em_wr(param, buf_elt->buf_ptr, length);
        buf_elt->length = length;

        status = ld_bcst_lmp_tx(buf_elt, &lb_env.ch_map, clock, num_bcst);

        // Activate timer if needed
        lm_afh_activate_timer();

        if (status != CO_ERROR_NO_ERROR)
        {
            // Free the buffer
            bt_util_buf_lmp_tx_free(buf_elt->buf_ptr);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/**
****************************************************************************************
* @brief This function updates the remote clock adjust period of slave devices
*
* @param[in] clk_adj_period    Period (slots)
*
*****************************************************************************************
*/
__STATIC void lb_clk_adj_period_update(uint8_t clk_adj_period)
{
    // Set a new clk_adj_period if not previously set, or if a multiple of a previous setting.
    if (!lb_env.pca.frame_period || ((lb_env.pca.frame_period < clk_adj_period) && (0 == CO_MOD(clk_adj_period, lb_env.pca.frame_period))))
    {
        lb_env.pca.frame_period = clk_adj_period;
    }
}

/**
****************************************************************************************
* @brief This function is used to actiavte  the coarse clock adjustment procedure
*
* @param[in] clk_adj_us        Adjustment us
* @param[in] clk_adj_slots     Adjustment slots
* @param[in] clk_adj_period    Period (slots)
* @param[in] tr_id             Transaction ID
****************************************************************************************
*/
__STATIC void lb_clk_adj_activate(int16_t clk_adj_us, uint8_t clk_adj_slots, uint8_t clk_adj_period, uint8_t tr_id)
{
    uint32_t current_clock = ld_read_clock() >> 1; // Read the clock in slots

    lb_env.pca.tr_id = tr_id;

    lb_env.pca.clk_adj_instant = (current_clock + LB_CLK_ADJ_DEFAULT_INTERVAL) & 0xFFFFFFFE;
    lb_env.pca.clk_adj_id++;

    // clk_adj_period may be used to select alternative clk_adj_slots - not used currently
    lb_clk_adj_period_update(clk_adj_period);

    lb_env.pca.clk_adj_us = clk_adj_us;
    lb_env.pca.clk_adj_slots = clk_adj_slots;

    lb_env.pca.clk_adj_mode = CLK_ADJ_BEFORE_INSTANT;
    lb_env.pca.clk_adj_clk = (current_clock + LB_CLK_ADJ_CLK_TX_INSTANT_OFFSET) >> 1;

    // Schedule the first broadcast clk_adj message
    lb_send_pdu_clk_adj(lb_env.pca.clk_adj_instant, lb_env.pca.clk_adj_id, lb_env.pca.clk_adj_us, lb_env.pca.clk_adj_slots,
                        lb_env.pca.clk_adj_mode, lb_env.pca.clk_adj_clk, lb_env.pca.tr_id);

    // Schedule the coarse clock adjustment to LD PCA manager
    ld_pca_coarse_clock_adjust(clk_adj_us, clk_adj_slots, lb_env.pca.clk_adj_instant);

    // Start broadcast clk_adj retransmit timer
    ke_timer_set(LB_PCA_TX_INTV_TO, TASK_LB, 10 /* 10ms */);

    lb_env.pca.instant_pending = true;
}
#endif // PCA_SUPPORT

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void lb_init(void)
{
    memset(&lb_env, 0, sizeof(lb_env));

    lb_env.num_bcst_ret = LB_DEFAULT_NBC;

#if CSB_SUPPORT
    lb_env.sync_train_intv = SYNC_TRAIN_INTV_DEFAULT;
    lb_env.sync_train_to = SYNC_TRAIN_TO_DEFAULT;
    lb_env.service_data = SYNC_TRAIN_SVC_DATA_DEFAULT;
    lb_env.csb_max_pkt_size = DM1_PACKET_SIZE;
    lb_env.sync_scan_to = SYNC_SCAN_TO_DEFAULT;
    lb_env.sync_scan_win = SYNC_SCAN_WIN_DEFAULT;
    lb_env.sync_scan_int = SYNC_SCAN_INTV_DEFAULT;
#endif //CSB_SUPPORT

    // Initialize broadcast channel map
    memset(&lb_env.ch_map.map[0], 0xFF, BT_CH_MAP_LEN);
    lb_env.ch_map.map[BT_CH_MAP_LEN - 1] &= ~(0x80);

#if PCA_SUPPORT
    // For all devices that may use coarse clock adjustment recovery mode, the RF channel indices used for
    //  the synchronization train shall be marked as unused in the AFH_channel_map for logical links LM 4.1.14.1
    lb_env.ch_map.map[SYNC_TRAIN_CHANNEL_0 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_0 & 0x7));
    lb_env.ch_map.map[SYNC_TRAIN_CHANNEL_1 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_1 & 0x7));
    lb_env.ch_map.map[SYNC_TRAIN_CHANNEL_2 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_2 & 0x7));
#endif // PCA_SUPPORT

    // Create LB task
    ke_task_create(TASK_LB, &TASK_DESC_LB);

    ke_state_set(TASK_LB, LB_IDLE);

#if BCAST_ENC_SUPPORT
    // Disable broadcast encryption
    lb_mst_stop_act_bcst_enc();
#endif // BCAST_ENC_SUPPORT
}

void lb_reset(void)
{
    memset(&lb_env, 0, sizeof(lb_env));

    // Initialize broadcast channel map
    memset(&lb_env.ch_map.map[0], 0xFF, BT_CH_MAP_LEN);
    lb_env.ch_map.map[BT_CH_MAP_LEN - 1] &= ~(0x80);

#if PCA_SUPPORT
    // For all devices that may use coarse clock adjustment recovery mode, the RF channel indices used for
    //  the synchronization train shall be marked as unused in the AFH_channel_map for logical links LM 4.1.14.1
    lb_env.ch_map.map[SYNC_TRAIN_CHANNEL_0 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_0 & 0x7));
    lb_env.ch_map.map[SYNC_TRAIN_CHANNEL_1 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_1 & 0x7));
    lb_env.ch_map.map[SYNC_TRAIN_CHANNEL_2 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_2 & 0x7));
#endif // PCA_SUPPORT

    lb_env.num_bcst_ret = LB_DEFAULT_NBC;

#if CSB_SUPPORT
    lb_env.sync_train_intv = SYNC_TRAIN_INTV_DEFAULT;
    lb_env.sync_train_to = SYNC_TRAIN_TO_DEFAULT;
    lb_env.service_data = SYNC_TRAIN_SVC_DATA_DEFAULT;
    lb_env.csb_max_pkt_size = DM1_PACKET_SIZE;
    lb_env.sync_scan_to = SYNC_SCAN_TO_DEFAULT;
    lb_env.sync_scan_win = SYNC_SCAN_WIN_DEFAULT;
    lb_env.sync_scan_int = SYNC_SCAN_INTV_DEFAULT;
#endif //CSB_SUPPORT

    ke_state_set(TASK_LB, LB_IDLE);

#if BCAST_ENC_SUPPORT
    // Disable broadcast encryption
    lb_mst_stop_act_bcst_enc();
#endif // BCAST_ENC_SUPPORT

    // invalidate csb not lpo_allowed flag
    rwip_prevent_sleep_clear(RW_CSB_NOT_LPO_ALLOWED);
}

uint8_t lb_util_get_nb_broadcast(void)
{
    return lb_env.num_bcst_ret;
}

void lb_util_set_nb_broadcast(uint8_t num_bcst_ret)
{
    lb_env.num_bcst_ret = num_bcst_ret;
}

#if CSB_SUPPORT
uint8_t lb_util_get_res_lt_addr(void)
{
    return lb_env.res_lt_addr;
}

bool lb_util_get_csb_mode(void)
{
    return GETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN);
}
#endif //CSB_SUPPORT

void lb_ch_map_update(struct bt_ch_map *new_ch_map)
{
    // Check if the map if different from the preceding one
    if (memcmp(&lb_env.ch_map.map[0], &new_ch_map->map[0], BT_CH_MAP_LEN))
    {
        // Store new map
        memcpy(&lb_env.ch_map.map[0], &new_ch_map->map[0], BT_CH_MAP_LEN);

        ld_bcst_afh_update(new_ch_map);

#if CSB_SUPPORT
        if (GETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN))
        {
            struct hci_con_slv_bcst_ch_map_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_SLV_BCST_CH_MAP_CHG_EVT_CODE, hci_con_slv_bcst_ch_map_chg_evt);

            ld_csb_tx_afh_update(new_ch_map);

            evt->ch_map = *new_ch_map;
            // Send the message
            hci_send_2_host(evt);
        }
#endif //CSB_SUPPORT
    }
}

#if PCA_SUPPORT
/**
 ****************************************************************************************
 * @brief Generic handler for a PCA request on a Master device
 *
 * @param[in] clk_adj_us. Request intraslot adjustment
 * @param[in] clk_adj_slots. Request interslot adjustment
 * @param[in] clk_adj_period. Period associated with interslot adjustment (slots)
 * @param[in] tr_id. Transaction ID of the request.
 * @return status as a result of processing the request.
 ****************************************************************************************
 */
uint8_t lb_master_clk_adj_req_handler(int16_t clk_adj_us, uint8_t clk_adj_slots, uint8_t clk_adj_period, uint8_t tr_id)
{
    uint8_t status = CO_ERROR_UNSPECIFIED_ERROR;

    if (!lb_env.pca.instant_pending)
    {
        if ((0 == clk_adj_us) && (0 == clk_adj_slots))
        {
            /* the slave may indicate its preferred clk_adj_period to the master without requesting a clock
            adjustment. In this case it shall set clk_adj_slots and clk_adj_us to zero. Upon receiving, the master
            shall resond with LMP_accepted but not initate any clock adjustment. */
            lb_clk_adj_period_update(clk_adj_period);
            status = CO_ERROR_NO_ERROR;
        }
#if RW_BT_MWS_COEX
        else if ((SLAVE_ROLE == tr_id) && (lm_local_ext_fr_configured()))
        {
            /* if a slave device requests a coarse clock adjustment, and the master is aligning to its own MWS frame,
             * disallow the request, as changing alignment would misalign the piconet with the local MWS frame. */
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
#endif //RW_BT_MWS_COEX
        else if ((co_abs(clk_adj_us) < BT_PCA_MIN_CC_ADJ_US) && (0 == clk_adj_slots))
        {
            status = CO_ERROR_CCA_REJ_USE_CLOCK_DRAG;
        }
        else if ((0 == lm_get_nb_acl(SLAVE_FLAG)) && !rwip_pca_clock_dragging_only())
        {
            status = lc_prepare_all_slaves_for_clk_adj();

            if (status == CO_ERROR_NO_ERROR)
            {
                /* initiate coarse clock adjustment */
                lb_clk_adj_activate(clk_adj_us, clk_adj_slots, clk_adj_period, tr_id);
                /* management of LB_clk_adj_ack responses */
                lb_env.pca.pending_clk_adj_acks = lm_get_nb_acl(MASTER_FLAG);
                lb_env.pca.clk_adj_ack_flags = 0;
            }
        }
        else
        {
            /* In certain modes (e.g. scatternet, LE), must use clock dragging as master piconet to perform the alignmenet. */
            status = CO_ERROR_CCA_REJ_USE_CLOCK_DRAG;
        }

        if ((CO_ERROR_CCA_REJ_USE_CLOCK_DRAG == status) && (0 != clk_adj_us))
        {
            /* rejected coarse clock adjustment request, so initiate clock dragging on clk_adj_us instead */
            ld_pca_initiate_clock_dragging(clk_adj_us);
        }
    }

    return status;
}

void lb_send_pdu_clk_adj(uint32_t clk_adj_instant, uint8_t clk_adj_id, int16_t clk_adj_us, uint8_t clk_adj_slots, uint8_t clk_adj_mode, uint32_t clk_adj_clk,  uint8_t tr_id)
{
    uint8_t pdu[LMP_CLK_ADJ_LEN];

    pdu[0] = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu[1] = LMP_CLK_ADJ_EXTOPCODE;
    pdu[2] = clk_adj_id;
    co_write32p(&pdu[3], clk_adj_instant);
    co_write16p(&pdu[7], clk_adj_us);
    pdu[9] = clk_adj_slots;
    pdu[10] = clk_adj_mode;
    co_write32p(&pdu[11], clk_adj_clk);

    // clk_adj_clk is specified in double slots => convert to timestamp clock in half slots
    lb_send_lmp(&pdu, LMP_CLK_ADJ_LEN, clk_adj_clk << 2, 1);
}

void lb_clk_adj_ack(uint8_t clk_adj_id, uint8_t link_id)
{
    if (clk_adj_id == lb_env.pca.clk_adj_id)
    {
        if ((lb_env.pca.pending_clk_adj_acks) && (0 == (lb_env.pca.clk_adj_ack_flags & (1 << link_id))))
        {
            lb_env.pca.clk_adj_ack_flags |= (1 << link_id);
            lb_env.pca.pending_clk_adj_acks--;
        }

        // Check if broadcast LMP_clk_adj should be cancelled
        if (!lb_env.pca.pending_clk_adj_acks)
        {
            ld_bcst_lmp_cancel();

            // If after instant, also need to stop the synchronization train
            if (lb_env.pca.clk_adj_mode == CLK_ADJ_AFTER_INSTANT)
            {
                ld_strain_stop();
            }
        }
    }
}

void lb_sscan_start_req(uint8_t link_id, struct bd_addr *bd_addr)
{
    if (lb_env.sscan_req == LB_SSCAN_IDLE)
    {
        /*
         * LM to decide on what conditions sync scan should start, and with what parameters for window, interval & timeout. TSync_Scan_Window
         * should be set large enough to enable reception of at least one synchronization train packet. A slave that could have lost its
         * connection to the master due to coarse clock adjustment should prioritize synchronization scan [BB 8.6.10.2].
         * The synchronization scan may be interrupted by higher priority traffic. In particular, the reserved synchronous slots should have
         * higher priority than the  synchronization scan [BB 8.11.1].
         */

        // Start synchronization scan with default values
        if (CO_ERROR_NO_ERROR == ld_sscan_start(bd_addr, SYNC_SCAN_TO_CCA_RM_DEFAULT, SYNC_SCAN_INTV_DEFAULT, SYNC_SCAN_WIN_DEFAULT))
        {
            lb_env.pca.sscan_link_id = link_id;
            lb_env.sscan_req = LB_SSCAN_PCA_SYNC;
        }
    }
}

void lb_sscan_stop_req(uint8_t link_id)
{
    // Confirm sync scan is enabled for this link
    if ((lb_env.sscan_req == LB_SSCAN_PCA_SYNC) && (lb_env.pca.sscan_link_id == link_id))
    {
        // Stop synchronization scan
        ld_sscan_stop();

        // Clear the request
        lb_env.sscan_req = LB_SSCAN_IDLE;
    }
}
#endif // PCA_SUPPORT

#if BCAST_ENC_SUPPORT
void lb_mst_key_cmp(uint8_t status, uint16_t conhdl, uint8_t new_key_flag)
{
    lb_env.mst_key_req = false;

    // allocate the complete event message
    struct hci_master_lk_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MASTER_LK_CMP_EVT_CODE, hci_master_lk_cmp_evt);


    // update the status
    evt->status = status;
    evt->conhdl = conhdl;
    evt->key_flag = new_key_flag;

    hci_send_2_host(evt);
}

void lb_mst_key(void)
{
    uint8_t link_id;

    if (lb_env.new_key_flag != SEMI_PERMANENT_KEY)
    {
        LM_CreateMasterKey();
        LM_CreateMasterKeyRand();
    }

    lb_env.pid_lc_cnt = 0;

    // Send the request to all active link(s)
    for (link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
    {
        lb_env.mstkey[link_id].in_use = false;

        if (lm_is_acl_con(link_id))
        {
            struct lc_mst_key_req *msg = KE_MSG_ALLOC(LC_MST_KEY_REQ, KE_BUILD_ID(TASK_LC, link_id), TASK_LB, lc_mst_key_req);
            msg->new_key_flag = lb_env.new_key_flag;
            ke_msg_send(msg);

            lb_env.mstkey[link_id].in_use = true;
            lb_env.pid_lc_cnt++;
        }
    }

    if (lb_env.pid_lc_cnt)
    {
        ke_state_set(TASK_LB, LB_WAIT_MASTER_KEY_CFM);
    }
    else
    {
        lb_env.conhdl = 0;
        lb_mst_key_cmp(CO_ERROR_COMMAND_DISALLOWED, lb_env.conhdl, lb_env.new_key_flag);
    }
}

void lb_mst_key_restart_enc(uint16_t conhdl)
{
    bool wait_enc_restart = false;
    uint16_t mst_key = 0xFFFF;
    uint8_t enc_key_size;

    for (uint8_t link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
    {
        if (lb_env.mstkey[link_id].in_use == true)
        {
            mst_key = mst_key & lb_env.mstkey[link_id].enc_key_size_msk;
            if (lb_env.mstkey[link_id].enc_mode != ENC_DISABLED)
            {
                lb_env.pid_lc_cnt += 1;
                wait_enc_restart = true;
            }
        }
    }

    enc_key_size = LM_ExtractMaxEncKeySize(mst_key);

    if ((enc_key_size <= ENC_KEY_SIZE_MAX) && (enc_key_size >= ENC_KEY_SIZE_MIN))
    {
        lb_env.LM_MasterEncKeySize = enc_key_size;
        LM_MakeRandVec(&lb_env.LM_MasterEncRand);
    }
    else
    {
        lb_env.pid_lc_cnt = 0;
        wait_enc_restart = false;
    }

    if (wait_enc_restart)
    {
        ke_state_set(TASK_LB, LB_WAIT_RESTART_ENC_CFM);
    }
    else
    {
        lb_mst_key_cmp(CO_ERROR_NO_ERROR, conhdl, lb_env.new_key_flag);
    }
}

void lb_mst_start_act_bcst_enc(void)
{
    struct ltk MasterKey;
    struct ltk EncRand;
    struct ltk Kc;
    struct ltk EncKey;
    struct bd_addr TempBdAddr;
    struct aco Aco;

    memcpy(&MasterKey.ltk[0], &lb_env.LM_MasterKey.ltk[0], KEY_LEN);

    memcpy(&EncRand.ltk[0], &lb_env.LM_MasterEncRand.ltk[0], KEY_LEN);

    ld_bd_addr_get(&TempBdAddr);

    // COF = BD_ADDR U BD_ADDR
    for (int Index = 0 ; Index < 6 ; Index++)
    {
        Aco.a[Index] = TempBdAddr.addr[Index];
        Aco.a[Index + 6] = TempBdAddr.addr[Index];
    }

    E3(MasterKey, Aco, EncRand, &Kc);

    KPrimC(Kc, lb_env.LM_MasterEncKeySize, &EncKey);

    ld_bcst_enc_key_load(&EncKey);
    ld_bcst_tx_enc(ENCRYPTION_ON);
}

void lb_mst_stop_act_bcst_enc(void)
{
    ld_bcst_tx_enc(ENCRYPTION_OFF);
}



void LM_GetMasterKey(struct ltk *MasterKey)
{
    memcpy(&MasterKey->ltk[0], &lb_env.LM_MasterKey.ltk[0], KEY_LEN);
}

void LM_GetMasterKeyRand(struct ltk *MasterRand)
{
    memcpy(&MasterRand->ltk[0], &lb_env.LM_MasterKeyRand.ltk[0], KEY_LEN);
}

void LM_GetMasterEncRand(struct ltk *MasterRand)
{
    memcpy(&MasterRand->ltk[0], &lb_env.LM_MasterEncRand.ltk[0], KEY_LEN);
}

uint8_t LM_GetMasterEncKeySize(void)
{
    return (lb_env.LM_MasterEncKeySize);
}
#endif // BCAST_ENC_SUPPORT

/// @} LB
