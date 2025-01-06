/**
****************************************************************************************
*
* @file ld.c
*
* @brief LD source code
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "co_math.h"

#include "arch.h"
#include "rwip.h"

#include "ld.h"             // link driver
#include "ld_int.h"         // link driver internal
#include "ld_util.h"        // link driver utilities
#include "bt_util_buf.h"      // BT EM buffer management

#include "dbg.h"

#include "sch_prog.h"           // Scheduling Programmer
#include "sch_slice.h"          // Scheduling Slicer

#include "em_map.h"

#include "reg_btcore.h"         // BT core registers
#include "reg_btcore_esco.h"    // BT core ESCO registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "reg_em_et.h"          // EM Exchange Table

#if (EAVESDROPPING_SUPPORT)
    #include "ed.h"
    #include "ke_msg.h"
#endif // EAVESDROPPING_SUPPORT

#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */


/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

/// Basic Rate packet types table (classified by packet size)
const uint8_t ld_acl_br_types[] =
{
    [DM1_IDX]  = DM1_TYPE,
    [DH1_IDX]  = DH1_TYPE,
    [DM3_IDX]  = DM3_TYPE,
    [DH3_IDX]  = DH3_TYPE,
    [DM5_IDX]  = DM5_TYPE,
    [DH5_IDX]  = DH5_TYPE,
};

/// Basic Rate packet sizes table (classified by packet size)
const uint16_t ld_acl_br_sizes[] =
{
    [DM1_IDX]  = DM1_PACKET_SIZE,
    [DH1_IDX]  = DH1_PACKET_SIZE,
    [DM3_IDX]  = DM3_PACKET_SIZE,
    [DH3_IDX]  = DH3_PACKET_SIZE,
    [DM5_IDX]  = DM5_PACKET_SIZE,
    [DH5_IDX]  = DH5_PACKET_SIZE,
};

/// Enhanced Data Rate packet types table (classified by packet size)
const uint8_t ld_acl_edr_types[] =
{
    [DM1_IDX]   = DM1_TYPE,
    [DH1_2_IDX] = DH1_2_TYPE,
    [DH1_3_IDX] = DH1_3_TYPE,
    [DH3_2_IDX] = DH3_2_TYPE,
    [DH3_3_IDX] = DH3_3_TYPE,
    [DH5_2_IDX] = DH5_2_TYPE,
    [DH5_3_IDX] = DH5_3_TYPE,
};

/// Enhanced Data Rate packet sizes table (classified by packet size)
const uint16_t ld_acl_edr_sizes[] =
{
    [DM1_IDX]   = DM1_PACKET_SIZE,
    [DH1_2_IDX] = DH1_2_PACKET_SIZE,
    [DH1_3_IDX] = DH1_3_PACKET_SIZE,
    [DH3_2_IDX] = DH3_2_PACKET_SIZE,
    [DH3_3_IDX] = DH3_3_PACKET_SIZE,
    [DH5_2_IDX] = DH5_2_PACKET_SIZE,
    [DH5_3_IDX] = DH5_3_PACKET_SIZE,
};

#if MAX_NB_SYNC
/// Default PCM settings
const uint8_t ld_pcm_settings_dft[8] =
{
    // PCMGENCNTL: 0x0000
    0x00, 0x00,
    // PCMPHYSCNTL0: 0x00013025
    0x25, 0x30, 0x01, 0x00,
    // PCMPHYSCNTL1: 0x8000
    0x00, 0x80
};
#endif //MAX_NB_SYNC

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support
///  Permutation structures for rfMode79 (used in frequency hop calculations)
const uint8_t  hop_permutation[14] =
{
    BIT0 | BIT1, BIT2 | BIT3, BIT1 | BIT2, BIT3 | BIT4, BIT0 | BIT4,
    BIT1 | BIT3, BIT0 | BIT2, BIT3 | BIT4, BIT1 | BIT4, BIT0 | BIT3,
    BIT2 | BIT4, BIT1 | BIT3, BIT0 | BIT3, BIT1 | BIT2
};
#endif // RW_BT_MWS_COEX

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD environment variable
struct ld_env_tag ld_env;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */


__STATIC void ld_core_init(void)
{
    struct bt_em_acl_buf_elt *acl_buf_elt;

    uint8_t length;
    uint8_t diag_cfg[PARAM_LEN_DIAG_BT_HW];

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_CORE_INIT_BIT);

#if RW_DEBUG
    // Debug only, initialize EM with 'all 1s'.
    em_set(0xFF, EM_BT_OFFSET, (EM_BT_END - EM_BT_OFFSET));
#endif //RW_DEBUG

    length = PARAM_LEN_DIAG_BT_HW;

    // Read diagport configuration from NVDS
    if (rwip_param.get(PARAM_ID_DIAG_BT_HW, &length, diag_cfg) == PARAM_OK)
    {
        bt_diagcntl_pack(1, diag_cfg[3], 1, diag_cfg[2], 1, diag_cfg[1], 1, diag_cfg[0]);
    }

    // Initialize common RX descriptors list
    for (int i = 0 ; i < EM_NUM_BT_RXDESC ; i++)
    {
        // Point each descriptor to next RX descriptor (in circular loop)
        em_bt_rxptr_pack(i, 0, REG_EM_ADDR_GET(BT_RXDESC, CO_MOD((i + 1), EM_NUM_BT_RXDESC)));

        // Allocate a ACL buffer
        acl_buf_elt = bt_util_buf_acl_rx_alloc();
        ASSERT_ERR(acl_buf_elt != NULL);
        em_bt_rxaclbufptr_setf(i, acl_buf_elt->buf_ptr);

        // Allocate a fixed LMP buffer (for the entire system life)
        em_bt_rxlmbufptr_setf(i, EM_BT_LMPRXBUF_OFFSET + (i * EM_BT_LMPRXBUF_SIZE));
    }

    // Initial permission/status of CS is unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + EM_BT_CS_OFFSET), EM_BT_CS_NB * REG_EM_BT_CS_SIZE, false, false, false);

    DBG_MEM_GRANT_CTRL((const void *)REG_EM_BT_CS_BASE_ADDR, true);
    // Initialize Control structures
    for (int i = 0 ; i < EM_BT_CS_INDEX_MAX ; i++)
    {
        // Set link label
        em_bt_linkcntl_linklbl_setf(i, i);
    }
    DBG_MEM_GRANT_CTRL((const void *)REG_EM_BT_CS_BASE_ADDR, false);

    // Point HW to 1st RX descriptor
    bt_currentrxdescptr_setf(REG_EM_ADDR_GET(BT_RXDESC, 0));

    // Set normal synchronization window size
    bt_rwbtcntl_nwinsize_setf(NORMAL_WIN_SIZE / 2);

    // Enable some interrupts needed by the LD

    bt_intcntl0_set(BT_SKIPFRMINTMSK_BIT | BT_ENDFRMINTMSK_BIT | BT_RXINTMSK_BIT | BT_ERRORINTMSK_BIT
#if RW_BT_MWS_COEX
                    | BT_FRSYNCINTMSK_BIT
#endif // RW_BT_MWS_COEX
#if EAVESDROPPING_SUPPORT
                    | BT_CLKCAPINTMSK_BIT | BT_CLKSETINTMSK_BIT | BT_EDMICERRINTMSK_BIT
#endif // EAVESDROPPING_SUPPORT
                   );
    bt_intack0_clear(0xFFFFFFFF);

    // Get Tx and Rx path delay
    ld_env.tx_path_delay = bt_radiotxrxtim_txpathdly_getf();
    ld_env.rx_path_delay = bt_radiotxrxtim_rxpathdly_getf();

    // Calculate the expected sync position
    ld_env.exp_sync_pos = NORMAL_SYNC_POS + ld_env.rx_path_delay;

    // Turn on BT Core
    bt_rwbtcntl_rwbten_setf(1);
}

__STATIC void ld_core_reset(void)
{
    struct bt_em_acl_buf_elt *acl_buf_elt;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_CORE_RESET_BIT);

    ASSERT_INFO((bt_version_get() == BT_VERSION_RESET), BT_VERSION_RESET, bt_version_get());

    // Turn off BT Core
    bt_rwbtcntl_rwbten_setf(0);

    // Reset the BT state machines
    bt_rwbtcntl_master_soft_rst_setf(1);
    while (bt_rwbtcntl_master_soft_rst_getf());

#if (MAX_NB_SYNC)
    // Disable Audio resources
    bt_pcmgencntl_pcmen_setf(0);
    for (int i = 0; i <= MAX_NB_SYNC ; i++)
    {
        bt_e_scochancntl_e_scochanen_setf(i, 0);
    }
#endif // (MAX_NB_SYNC)

    // Initialize common RX descriptors list
    for (int i = 0 ; i < EM_NUM_BT_RXDESC ; i++)
    {
        // Point each descriptor to next RX descriptor (in circular loop)
        em_bt_rxptr_pack(i, 0, REG_EM_ADDR_GET(BT_RXDESC, CO_MOD((i + 1), EM_NUM_BT_RXDESC)));

        // Allocate a ACL buffer
        acl_buf_elt = bt_util_buf_acl_rx_alloc();
        ASSERT_ERR(acl_buf_elt != NULL);
        em_bt_rxaclbufptr_setf(i, acl_buf_elt->buf_ptr);

        // Allocate a fixed LMP buffer (for the entire system life)
        em_bt_rxlmbufptr_setf(i, EM_BT_LMPRXBUF_OFFSET + (i * EM_BT_LMPRXBUF_SIZE));
    }

    // Point HW to 1st RX descriptor
    bt_currentrxdescptr_setf(REG_EM_ADDR_GET(BT_RXDESC, 0));

    // Enable some interrupts needed by the LD
    bt_intcntl0_set(BT_SKIPFRMINTMSK_BIT | BT_ENDFRMINTMSK_BIT | BT_RXINTMSK_BIT | BT_ERRORINTMSK_BIT
#if RW_BT_MWS_COEX
                    | BT_FRSYNCINTMSK_BIT
#endif // RW_BT_MWS_COEX
#if EAVESDROPPING_SUPPORT
                    | BT_CLKCAPINTMSK_BIT | BT_CLKSETINTMSK_BIT | BT_EDMICERRINTMSK_BIT
#endif // RW_BT_MWS_COEX
                   );
    bt_intack0_clear(0xFFFFFFFF);

    // Restore default control register
    bt_rwbtcntl_pack(
        0, /*mastersoftrst   */
        0, /*mastertgsoftrst */
        0, /*regsoftrst      */
        0, /*radiocntlsoftrst*/
        0, /*swintreq        */
        0, /*scanabort       */
        0, /*rftestabort     */
        0, /*pageinqabort    */
        0, /*sniffabort      */
        0, /*lmpflowdsb      */
        0, /*cryptdsb        */
        0, /*crcdsb          */
        0, /*whitdsb         */
        0, /*hopdsb          */
        0, /*flowdsb         */
        0, /*arqndsb         */
        0, /*seqndsb         */
        1, /*cxtxbsyena      */
        1, /*cxrxbsyena      */
        1, /*cxdnabort       */
        0, /*rwbten          */
        NORMAL_WIN_SIZE / 2 /*nwinsize        */);

    // Get Tx and Rx path delay
    ld_env.tx_path_delay = bt_radiotxrxtim_txpathdly_getf();
    ld_env.rx_path_delay = bt_radiotxrxtim_rxpathdly_getf();

    // Calculate the expected sync position
    ld_env.exp_sync_pos = NORMAL_SYNC_POS + ld_env.rx_path_delay;
}


/*
 * MODULE FUNCTIONS DEFINITION
 *****************************************************************************************
 */

#if (RW_BT_MWS_COEX)
void ld_mwscoex_xi_scan_mask_build(uint32_t *mwscoex_xi_scan_mask, uint32_t uap_lap)
{
    uint16_t result;
    uint8_t num_freqs = 32;
    uint8_t X;

    uint8_t mws_rxchan_h = 0;
    uint8_t mws_rxchan_l = 0;
    uint8_t mws_txchan_h = 0;
    uint8_t mws_txchan_l = 0;

    bool mwstxfreqmsk_on;
    bool mwsrxfreqmsk_on;

    /* Initially set all scan mask to 0 - indicating all as non-occupied */
    *mwscoex_xi_scan_mask = 0;

    /*
     * Fetch MWS configuration [f=2402+k MHz, k=0,...,78]
     */
    mwstxfreqmsk_on = (LD_MWS_CAN_STOP_BT_TXRX == bt_coexifcntl0_mwstxfrqmsk_getf());
    mwsrxfreqmsk_on = (LD_MWS_CAN_STOP_BT_TXRX == bt_coexifcntl0_mwsrxfrqmsk_getf());

    if (mwstxfreqmsk_on)
    {
        mws_txchan_h = bt_mwstxtable0_mwstxfreqh_getf() - HOP_CHANNEL_BASE_MHZ;
        mws_txchan_l = bt_mwstxtable0_mwstxfreql_getf() - HOP_CHANNEL_BASE_MHZ;
    }
    if (mwsrxfreqmsk_on)
    {
        mws_rxchan_h = bt_mwsrxtable0_mwsrxfreqh_getf() - HOP_CHANNEL_BASE_MHZ;
        mws_rxchan_l = bt_mwsrxtable0_mwsrxfreql_getf() - HOP_CHANNEL_BASE_MHZ;
    }

    if (mwstxfreqmsk_on || mwsrxfreqmsk_on)
    {

        /* Initial values as f(Addr) for A, B, C, D, E and F.  E,F combined to EF.
         *         A = Addr[27:23]           => UAP[3:0]lap[23]
         *         B = Addr[22:19]           => LAP[22:19]
         *         C = Addr[8,6,4,2,0]       => LAP[8,6,4,2,0]
         *         D = Addr[18:10]           => LAP[18:10]
         *         E = Addr[13,11,9,7,5,3,1] => LAP[*]
         *         F = 0
         */
        uint8_t Ai = (uint8_t)(uap_lap >> 23) & 0x1F;
        uint8_t Bi = (uint8_t)(uap_lap >> 19) & 0x0F;
        uint8_t Ci = (uint8_t)(((uap_lap & BIT8) >> 4) | ((uap_lap & BIT6) >> 3) |
                               ((uap_lap & BIT4) >> 2) | ((uap_lap & BIT2) >> 1) |
                               (uap_lap & BIT0));
        uint16_t Di = (uint16_t)((uap_lap >> 10) & 0x01FF);
        uint8_t EFi = (uint8_t)(((uap_lap & BIT13) >> 7) | ((uap_lap & BIT11) >> 6) |
                                ((uap_lap & BIT9) >> 5) | ((uap_lap & BIT7) >> 4)  |
                                ((uap_lap & BIT5) >> 3)  | ((uap_lap & BIT3) >> 2)  |
                                ((uap_lap & BIT1) >> 1));


        /* Page_Scan/Inquiry_Scan:  Receive  frequency for X */
        for (X = 0; X < num_freqs; X++)
        {
            /* Stage 1*** ADD */
            result = (X + Ai) & 0x1F;

            /* Stage 2*** XOR   B not modified by clock in Connection */
            result ^= Bi;

            /*
             * Stage 3*** Permutate
             *   for each Permutation Operation do
             *     if (Permutation Necessary) then
             *        if (Bits to Permutate are different) then
             *           Swap Bits
             *        endif
             *     endif
             *   endfor
             */
            {
                uint16_t permControl = Di | Ci << 9;

                const uint8_t    *permEntry;
                uint16_t permute;
                uint16_t  mask;

                permEntry = hop_permutation + 13;

                mask = BIT13;
                while (mask != 0)
                {
                    permute = *permEntry--;
                    if (permControl & mask)
                    {
                        if (!((permute & result) == 0 || (permute & result) == permute))
                            result ^= permute;
                    }
                    mask >>= 1;
                }
            }

            /*
             * Stage 4*** Add and Modulo rfMode79
             * Use subtraction due to limited summation (i.e. max result 317)
             * Result = Result + E + F + ((Y=0)<<5)
             */
            {
                result += (uint16_t) EFi;
                while (result > HOP_NB_CHANNEL)
                {
                    result -= HOP_NB_CHANNEL;
                }
            }

            /*
             * Stage 5*** Frequency selection [Algorithm based]
             * if (Result in Table TopHalf ( <=rfMode/2) then
             *   Result = Result*2                  * Table TopHalf    0,2,4,6,  ,rfMode-1 *
             * else
             *   Result = (Result-rfMode/2-1)*2 + 1 * Table BottomHalf 1,3,5,7,  ,rfMode-2 *
             * endif
             */
            result <<= 1;
            if (result > HOP_NB_CHANNEL)
            {
                result -= HOP_NB_CHANNEL;
            }

            /*
             * Determine if the channel for this X-input is occupied by applicable active MWS tx/rx
             */
            if (((mwstxfreqmsk_on) && (result >= mws_txchan_l) && (result <= mws_txchan_h))
                    || ((mwsrxfreqmsk_on) && (result >= mws_rxchan_l) && (result <= mws_rxchan_h)))
            {
                *mwscoex_xi_scan_mask |= (1 << X); /* set bit N of X input as MWS occupied channel */
            }
        }
    }
}
#endif // (RW_BT_MWS_COEX)

void ld_rxdesc_free(void)
{
    uint8_t rxdesc_idx = ld_env.curr_rxdesc_index;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_RXDESC_FREE_BIT);

    // Check if the TX descriptor needs an ACL buffer
    if (em_bt_rxaclbufptr_getf(rxdesc_idx) == 0)
    {
        // Allocate new ACL RX buffer
        struct bt_em_acl_buf_elt *acl_buf_elt = bt_util_buf_acl_rx_alloc();

        // If new buffer found, link to the descriptor
        if (acl_buf_elt)
        {
            em_bt_rxaclbufptr_setf(rxdesc_idx, acl_buf_elt->buf_ptr);
        }
        else
        {
            // Indicate this RX descriptor has no buffer for data reception
            ld_env.rxdesc_data_ptr_bf |= CO_BIT(rxdesc_idx);
        }
    }

    // Free descriptor
    em_bt_rxptr_rxdone_setf(rxdesc_idx, 0);

    // Move RX descriptor index
    ld_env.curr_rxdesc_index = CO_MOD(ld_env.curr_rxdesc_index + 1, EM_NUM_BT_RXDESC);
}

bool ld_rxdesc_check(uint8_t label)
{
    uint8_t rxdesc_idx = ld_env.curr_rxdesc_index;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PATCH_TYPE, LD_RXDESC_CHECK_BIT, bool, label);

    // Check if the RX descriptor is consumed by the frame
    return (em_bt_rxptr_rxdone_getf(rxdesc_idx) && (em_bt_rxstat_rxlinklbl_getf(rxdesc_idx) == label));
}

void ld_active_mode_set(uint8_t type, uint8_t link_id, uint8_t sco_link_id, uint8_t mode)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_ACTIVE_MODE_SET_BIT, type, link_id, sco_link_id, mode);
    if (mode)
    {
        // Set the corresponding bit
        if (type == LD_ACT_TYPE_ACL)
        {
            LD_ACT_ACL_SET(ld_env.active_links, link_id);
        }
        else
        {
            LD_ACT_SCO_SET(ld_env.active_links, sco_link_id, link_id);
        }

        // Indicate BT requires active mode
        rwip_prevent_sleep_set(RW_BT_ACTIVE_MODE);
    }
    else
    {
        // Clear the corresponding bit
        if (type == LD_ACT_TYPE_ACL)
        {
            LD_ACT_ACL_CLEAR(ld_env.active_links, link_id);
        }
        else
        {
            LD_ACT_SCO_CLEAR(ld_env.active_links, sco_link_id);
        }

        if (ld_env.active_links == 0)
        {
            // Indicate BT does not require active mode
            rwip_prevent_sleep_clear(RW_BT_ACTIVE_MODE);
        }
    }
}

bool ld_active_link_check(uint8_t link_id)
{
    bool active = true;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PATCH_TYPE, LD_ACTIVE_LINK_CHECK_BIT, bool, link_id);

    // Check if ACL link is active
    if ((ld_env.active_links & (1 << link_id)) == 0)
    {
        int i = 0;

        // The link is not active, then check if there is a SCO on that link
        for (i = 0 ; i < MAX_NB_SYNC ; i++)
        {
            if (ld_env.active_links >> (8 * (i + 1)) == (uint32_t)(0x80 + link_id))
                break;
        }

        // No SCO on that link
        if (i == MAX_NB_SYNC)
        {
            active = false;
        }
    }

    return active;
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_init(void)
{
    uint8_t length;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_INIT_BIT);

    memset(&ld_env, 0, sizeof(ld_env));

    // Initialize BT CORE + EM
    ld_core_init();

    // Point SW to 1st RX descriptor
    ld_env.curr_rxdesc_index = 0;

    // Initialize local BD_ADDR
    length = BD_ADDR_LEN;
    if (rwip_param.get(PARAM_ID_BD_ADDRESS, &length, &ld_env.local_bd_addr.addr[0]) != PARAM_OK)
    {
        memcpy(&ld_env.local_bd_addr.addr[0], &co_default_bdaddr, BD_ADDR_LEN);
    }

#if MAX_NB_SYNC
    // Initialize PCM configuration
    length = PARAM_LEN_PCM_SETTINGS;
    if (rwip_param.get(PARAM_ID_PCM_SETTINGS, &length, &ld_env.pcm_settings[0]) != PARAM_OK)
    {
        memcpy(ld_env.pcm_settings, ld_pcm_settings_dft, 8);
    }
#endif //MAX_NB_SYNC

    // Compute the BCH for local BD Address
    ld_util_bch_create(&ld_env.local_bd_addr.addr[0], &ld_env.local_bch[0]);

    // Initialize Inquiry manager
    ld_inq_init();
    // Initialize Inquiry Scan manager
    ld_iscan_init();
    // Initialize Page manager
    ld_page_init();
    // Initialize Page Scan manager
    ld_pscan_init();
    // Initialize ACL manager
    ld_acl_init();
    // Initialize boadcast manager
    ld_bcst_acl_init();
#if CSB_SUPPORT || PCA_SUPPORT
    // Initialize sync train manager
    ld_strain_init();
    // Initialize sync scan manager
    ld_sscan_init();
#endif // CSB_SUPPORT || PCA_SUPPORT
#if CSB_SUPPORT
    // Initialize connectionless slave broadcast TX manager
    ld_csb_tx_init();
    // Initialize connectionless slave broadcast RX manager
    ld_csb_rx_init();
#endif // CSB_SUPPORT
#if PCA_SUPPORT
    // Initialize piconet clock adjust manager
    ld_pca_init();
#endif // PCA_SUPPORT
#if (BT_HCI_TEST_MODE)
    // Initialize Test manager
    ld_test_init();
#endif //(BT_HCI_TEST_MODE)
#if (EAVESDROPPING_SUPPORT)
    ld_ed_init();
#endif // EAVESDROPPING_SUPPORT
}

void ld_reset(void)
{
    uint8_t length;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_RESET_BIT);

    memset(&ld_env, 0, sizeof(ld_env));

    // Reset BT CORE + EM
    ld_core_reset();

    // Initialize local BD_ADDR
    length = BD_ADDR_LEN;
    if (rwip_param.get(PARAM_ID_BD_ADDRESS, &length, &ld_env.local_bd_addr.addr[0]) != PARAM_OK)
    {
        memcpy(&ld_env.local_bd_addr.addr[0], &co_default_bdaddr, BD_ADDR_LEN);
    }

#if MAX_NB_SYNC
    // Initialize PCM configuration
    length = PARAM_LEN_PCM_SETTINGS;
    if (rwip_param.get(PARAM_ID_PCM_SETTINGS, &length, &ld_env.pcm_settings[0]) != PARAM_OK)
    {
        memcpy(ld_env.pcm_settings, ld_pcm_settings_dft, 8);
    }
#endif //MAX_NB_SYNC

    // Compute the BCH for local BD Address
    ld_util_bch_create(&ld_env.local_bd_addr.addr[0], &ld_env.local_bch[0]);

    // Reset Inquiry manager
    ld_inq_reset();
    // Reset Inquiry Scan manager
    ld_iscan_reset();
    // Reset Page manager
    ld_page_reset();
    // Reset Page Scan manager
    ld_pscan_reset();
    // Reset ACL manager
    ld_acl_reset();
    // Reset broadcast ACL manager
    ld_bcst_acl_reset();
#if CSB_SUPPORT || PCA_SUPPORT
    // Reset sync train manager
    ld_strain_reset();
    // Reset sync scan manager
    ld_sscan_reset();
#endif // CSB_SUPPORT || PCA_SUPPORT
#if CSB_SUPPORT
    // Reset connectionless slave broadcast TX manager
    ld_csb_tx_reset();
    // Reset connectionless slave broadcast RX manager
    ld_csb_rx_reset();
#endif // CSB_SUPPORT
#if PCA_SUPPORT
    // Resets piconet clock adjust manager
    ld_pca_reset();
#endif // PCA_SUPPORT
#if (BT_HCI_TEST_MODE)
    // Reset Test manager
    ld_test_reset();
#endif //(BT_HCI_TEST_MODE)

#if (EAVESDROPPING_SUPPORT)
    ld_ed_reset();
#endif // EAVESDROPPING_SUPPORT
}

uint32_t ld_read_clock(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_PATCH_TYPE, LD_READ_CLOCK_BIT, uint32_t);
    return rwip_time_get().hs;
}

void ld_bd_addr_get(struct bd_addr *bd_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_BD_ADDR_GET_BIT, bd_addr);
    // Copy BD address to output buffer
    memcpy(&bd_addr->addr[0], &ld_env.local_bd_addr.addr[0], BD_ADDR_LEN);
}

void ld_class_of_dev_get(struct devclass *class_of_dev)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_CLASS_OF_DEV_GET_BIT, class_of_dev);
    // Copy Class Of Device to output buffer
    memcpy(&class_of_dev->A[0], &ld_env.class_of_dev.A[0], DEV_CLASS_LEN);
}

void ld_class_of_dev_set(struct devclass *class_of_dev)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PATCH_TYPE, LD_CLASS_OF_DEV_SET_BIT, class_of_dev);
    // Copy Class Of Device from input buffer
    memcpy(&ld_env.class_of_dev.A[0], &class_of_dev->A[0], DEV_CLASS_LEN);
}

bool ld_rxdesc_buf_ready(uint16_t em_ptr)
{
    bool assign = false;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PATCH_TYPE, LD_RXDESC_BUF_READY_BIT, bool, em_ptr);

    if (ld_env.rxdesc_data_ptr_bf != 0)
    {
        uint8_t rxdesc_idx = co_ctz(ld_env.rxdesc_data_ptr_bf);
        ASSERT_ERR(em_ptr != 0);

        // Link to the descriptor
        em_bt_rxaclbufptr_setf(rxdesc_idx, em_ptr);
        // Indicate the RX descriptor has a buffer for data reception
        ld_env.rxdesc_data_ptr_bf &= ~CO_BIT(rxdesc_idx);

        assign = true;
    }

    return (assign);
}

#if (RW_BT_MWS_COEX)

uint8_t ld_mws_channel_params_set(uint8_t mws_channel_enable, uint16_t mws_rx_center_frequency, uint16_t mws_tx_center_frequency,
                                  uint16_t mws_rx_channel_bandwidth, uint16_t mws_tx_channel_bandwidth, uint8_t mws_channel_type)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint16_t mws_rx_hbw_mhz, mws_tx_hbw_mhz;
    uint16_t freql, freqh;

    // calculate half-bandwidths in MHz
    mws_rx_hbw_mhz = mws_rx_channel_bandwidth / 2000;
    if (CO_MOD(mws_rx_channel_bandwidth, 2000))
        mws_rx_hbw_mhz++;

    mws_tx_hbw_mhz = mws_tx_channel_bandwidth / 2000;
    if (CO_MOD(mws_tx_channel_bandwidth, 2000))
        mws_tx_hbw_mhz++;

    if (MWS_CHANNEL_ENABLED == mws_channel_enable)
    {
        // MWS tx/rx frequency can stop TXRX, or set user defined configuration by dbg_wr_mem_cmd, ref RW-BT-MWS_COEX_AN.
        bt_coexifcntl0_mwstxfrqmsk_setf(LD_MWS_CAN_STOP_BT_TXRX);
        bt_coexifcntl0_mwsrxfrqmsk_setf(LD_MWS_CAN_STOP_BT_TXRX);
    }
    else
    {
        // reset to No Impact
        bt_coexifcntl0_mwstxfrqmsk_setf(LD_MWS_NO_IMPACT);
        bt_coexifcntl0_mwsrxfrqmsk_setf(LD_MWS_NO_IMPACT);
    }

    // calculate & set upper and lower rx frequencies
    freql = (mws_rx_center_frequency > mws_rx_hbw_mhz) ? (mws_rx_center_frequency - mws_rx_hbw_mhz) : 0;
    freqh = ((uint32_t)mws_rx_center_frequency + mws_rx_hbw_mhz < 0xffff) ? (mws_rx_center_frequency + mws_rx_hbw_mhz) : 0xffff;

    bt_mwsrxtable0_pack(freqh, freql);

    // calculate & set upper and lower tx frequencies
    freql = (mws_tx_center_frequency > mws_tx_hbw_mhz) ? (mws_tx_center_frequency - mws_tx_hbw_mhz) : 0;
    freqh = ((uint32_t)mws_tx_center_frequency + mws_tx_hbw_mhz < 0xffff) ? (mws_tx_center_frequency + mws_tx_hbw_mhz) : 0xffff;

    bt_mwstxtable0_pack(freqh, freql);

    if (MWS_TDD_CHANNEL_TYPE == mws_channel_type)
    {
        // MWS tx/rx can stop TXRX, or set user defined configuration by dbg_wr_mem_cmd, ref RW-BT-MWS_COEX_AN.
        bt_coexifcntl0_mwstxmsk_setf(LD_MWS_CAN_STOP_BT_TXRX);
        bt_coexifcntl0_mwsrxmsk_setf(LD_MWS_CAN_STOP_BT_TXRX);
    }
    else
    {
        // reset to No Impact
        bt_coexifcntl0_mwstxmsk_setf(LD_MWS_NO_IMPACT);
        bt_coexifcntl0_mwsrxmsk_setf(LD_MWS_NO_IMPACT);
    }

    return status;
}

uint8_t ld_ext_frame_info_set(uint16_t ext_fr_duration, int16_t ext_fr_sync_assert_offset, uint16_t ext_fr_scan_duration)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    /* if Ext_Frame_Sync_Assert_Offset is negative then
     MSWFRYSNCASSERTDLY = |Ext_Frame_Sync_Assert_Offset|
     else
     MSWFRYSNCASSERTDLY = External_Frame_Duration - |Ext_Frame_Sync_Assert_Offset| */

    if (ext_fr_sync_assert_offset <= 0)
    {
        bt_mwsfrsyncoffset_mwsfrsyncassertdly_setf(0 - ext_fr_sync_assert_offset);
    }
    else
    {
        bt_mwsfrsyncoffset_mwsfrsyncassertdly_setf(ext_fr_duration - ext_fr_sync_assert_offset);
    }

    ld_env.mws_ext_fr_duration = ext_fr_duration;
    ld_env.mws_ext_fr_offset = ext_fr_sync_assert_offset;
    ld_env.mws_ext_fr_scan_dur = ext_fr_scan_duration;

    return status;
}

uint8_t ld_mws_signaling_offsets_set(struct ld_mws_signaling_wr_offsets *p_wroff, struct ld_mws_signaling_rd_offsets *p_rdoff)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    uint16_t ext_fr_duration = ld_env.mws_ext_fr_duration;

    /* Set Rx Assert Offset */
    if (p_wroff->rx_assert_offset <= 0)
    {
        bt_mwsrxoffset_mwsrxassertdly_setf(0 - p_wroff->rx_assert_offset);
    }
    else
    {
        bt_mwsrxoffset_mwsrxassertdly_setf(ext_fr_duration - p_wroff->rx_assert_offset);
    }

    /* Set Rx Deassert Offset */
    if (p_wroff->rx_deassert_offset <= 0)
    {
        bt_mwsrxoffset_mwsrxdeassertdly_setf(0 - p_wroff->rx_deassert_offset);
    }
    else
    {
        bt_mwsrxoffset_mwsrxdeassertdly_setf(ext_fr_duration - p_wroff->rx_deassert_offset);
    }

    /* Set Tx Assert Offset */
    if (p_wroff->tx_assert_offset <= 0)
    {
        bt_mwstxoffset_mwstxassertdly_setf(0 - p_wroff->tx_assert_offset);
    }
    else
    {
        bt_mwstxoffset_mwstxassertdly_setf(ext_fr_duration - p_wroff->tx_assert_offset);
    }

    /* Set Tx Deassert Offset */
    if (p_wroff->tx_deassert_offset <= 0)
    {
        bt_mwstxoffset_mwstxdeassertdly_setf(0 - p_wroff->tx_deassert_offset);
    }
    else
    {
        bt_mwstxoffset_mwstxdeassertdly_setf(ext_fr_duration - p_wroff->tx_deassert_offset);
    }

    /* Get Rx On Assert Offset */
    if (LD_MWS_HI_PRIO_IND == bt_coexifcntl0_wlcrxpriomode_getf())
    {
        p_rdoff->rx_prio_assert_offset = bt_radiopwrupdn_rxpwrupct_getf(); /* RADIOPWRUPDN-RXPWRUPCT */
    }
    else
    {
        p_rdoff->rx_prio_assert_offset = 0; /* Return 0x0 */
    }

    /* Get Rx Prio Deassert Offset */
    p_rdoff->rx_prio_deassert_offset = 0; /* Always 0x0 */

    /* Get Tx On Assert Offset */
    if (LD_MWS_HI_PRIO_IND == bt_coexifcntl0_wlctxpriomode_getf())
    {
        p_rdoff->tx_on_assert_offset = bt_radiopwrupdn_txpwrupct_getf(); /* RADIOPWRUPDN-TXPWRUPCT */
    }
    else
    {
        p_rdoff->tx_on_assert_offset = 0; /* Return 0x0 */
    }

    /* Get Tx On Deassert Offset */
    p_rdoff->tx_on_deassert_offset = bt_radiopwrupdn_txpwrdnct_getf(); /* Always RADIOPWRUPDN-TXPWRDNCT */

    /* Get Tx/Rx Assert/Deassert Jitter */
    p_rdoff->rx_prio_assert_jitter = 0; /* Always 0x0 */
    p_rdoff->rx_prio_deassert_jitter = 0; /* Always 0x0 */
    p_rdoff->tx_on_assert_jitter = 0; /* Always 0x0 */
    p_rdoff->tx_on_deassert_jitter = 0; /* Always 0x0 */

    return status;
}

uint8_t ld_mws_transport_layer_set(uint8_t transport_layer, uint32_t to_mws_baud_rate, uint32_t from_mws_baud_rate)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    /* transport layer is one of MWS_SIGNALING_ONLY, MWS_WCI_1 or MWS_WCI_2 */
    if (MWS_SIGNALING_ONLY == transport_layer)
    {
        bt_coexifcntl0_mwswci_en_setf(0);
    }
    else
    {
#if 1 // Not validated - The current MWSGEN does not support WCI.
        status = CO_ERROR_UNSUPPORTED;
#else
        uint16_t intdiv_upscale;

        bt_coexifcntl0_mwswci_en_setf(1);
        bt_mwswcicntl1_wcisel_setf((MWS_WCI_2 == transport_layer) ? 1 : 0);

        /* WCI serial Tx/rx interface baud rates are defined by the following formulas -
         to_mws_baud_rate =  clk_sel / (MWSSWCI_TXINTDIV + (MWSSWICI_TXFRACTDIV)/8)
         from_mws_baud_rate = clk_sel / (MWSSWCO+RXOMTDOV + (MWSWCI_RXFRACTDIV)/8) */

        uint32_t ref_clk = 8000000; // Assuming IP is running at 8MHz.

        intdiv_upscale = ((ref_clk << 4) / to_mws_baud_rate) + 1;
        bt_mwswcicntl0_mwswci_txintdiv_setf(intdiv_upscale >> 4);
        bt_mwswcicntl0_mwswci_txfractdiv_setf((intdiv_upscale >> 1) & 0x7);

        intdiv_upscale = ((ref_clk << 4) / from_mws_baud_rate) + 1;
        bt_mwswcicntl0_mwswci_rxintdiv_setf(intdiv_upscale >> 4);
        bt_mwswcicntl0_mwswci_rxfractdiv_setf((intdiv_upscale >> 1) & 0x7);
#endif
    }

    return status;
}

uint8_t ld_mws_scan_freq_table_set(uint8_t num_freqs, const struct mws_scan_freq *scan_freqs)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    /* If greater than 8, then use only the 8 first values within the commands */
    if (num_freqs > 8)
    {
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
        num_freqs = 8;
    }

    /* as reg_btcore header definitions don't naturally provide for iteration through mwsftable[n], am pack writing
       scan frequency configurations in sequence from a fall-through switch - */
    switch (num_freqs)
    {
    case 8:
        bt_mwssftable8_pack(scan_freqs[7].high, scan_freqs[7].low);
    case 7:
        bt_mwssftable7_pack(scan_freqs[6].high, scan_freqs[6].low);
    case 6:
        bt_mwssftable6_pack(scan_freqs[5].high, scan_freqs[5].low);
    case 5:
        bt_mwssftable5_pack(scan_freqs[4].high, scan_freqs[4].low);
    case 4:
        bt_mwssftable4_pack(scan_freqs[3].high, scan_freqs[3].low);
    case 3:
        bt_mwssftable3_pack(scan_freqs[2].high, scan_freqs[2].low);
    case 2:
        bt_mwssftable2_pack(scan_freqs[1].high, scan_freqs[1].low);
    case 1:
        bt_mwssftable1_pack(scan_freqs[0].high, scan_freqs[0].low);
        break;
    default:
        break;
    }

    return status;
}

uint8_t ld_mws_pattern_config_set(uint8_t pattern_index, uint8_t num_intervals, const struct mws_pattern_intv *p_intv)
{
    uint8_t status = CO_ERROR_NO_ERROR;

#define LD_MWS_PATTERN_MAX 6

    uint8_t mwspat_type[LD_MWS_PATTERN_MAX] = {0, 0, 0, 0, 0, 0};
    uint16_t mwspat_duration[LD_MWS_PATTERN_MAX] = {0, 0, 0, 0, 0, 0};
    int i;

    /* If greater than 6, then use only the 6 first values within the commands */
    if (num_intervals > LD_MWS_PATTERN_MAX)
    {
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
        num_intervals = LD_MWS_PATTERN_MAX;
    }

    // DIRECT MAPPING
    // HCI MWS_PATTERN_IntervalType definition -> HW definition of MWSPAT bits:
    // 0: Neither transmission nor reception is allowed
    // 1: Transmission is allowed
    // 2: Reception is allowed
    // 3: Both transmission and reception are allowed
    // 4: Interval for MWS frame as defined by Set_External_Frame_Configuration
    for (i = 0; i < num_intervals; i++)
    {
        mwspat_type[i] = p_intv[i].type;
        mwspat_duration[i] = p_intv[i].duration;
    }

    // Configure the timing and behavior of the specified  MWS Pattern
    if (0 == pattern_index)
    {
        bt_mwsptable0_pack(mwspat_type[5], mwspat_type[4], mwspat_type[3], mwspat_type[2], mwspat_type[1], mwspat_type[0]);
        bt_mwsptiming00_pack(mwspat_duration[1], mwspat_duration[0]);
        bt_mwsptiming01_pack(mwspat_duration[3], mwspat_duration[2]);
        bt_mwsptiming02_mwsptim0_4_to_5_setf(mwspat_duration[4]);
    }
    else if (1 == pattern_index)
    {
        bt_mwsptable1_pack(mwspat_type[5], mwspat_type[4], mwspat_type[3], mwspat_type[2], mwspat_type[1], mwspat_type[0]);
        bt_mwsptiming10_pack(mwspat_duration[1], mwspat_duration[0]);
        bt_mwsptiming11_pack(mwspat_duration[3], mwspat_duration[2]);
        bt_mwsptiming12_mwsptim1_4_to_5_setf(mwspat_duration[4]);
    }
    else if (2 == pattern_index)
    {
        bt_mwsptable2_pack(mwspat_type[5], mwspat_type[4], mwspat_type[3], mwspat_type[2], mwspat_type[1], mwspat_type[0]);
        bt_mwsptiming20_pack(mwspat_duration[1], mwspat_duration[0]);
        bt_mwsptiming21_pack(mwspat_duration[3], mwspat_duration[2]);
        bt_mwsptiming22_mwsptim2_4_to_5_setf(mwspat_duration[4]);
    }

    return status;
}

ld_mwsifstat_t ld_mws_pattern_index_get(void)
{
    ld_mwsifstat_t mwsifstat;
    uint8_t inact_dur = 0;
    uint8_t start_ind = 0;

    mwsifstat.pattern_index = SAM_DISABLED;
    mwsifstat.slot_offset = 0;

    if (bt_coexifcntl0_mwscoex_en_getf())
    {
        // Sample the active MWS interface for pattern index & inactivity duration
        bt_mwsifstat_mws_if_samp_setf(1);
        while (bt_mwsifstat_mws_if_samp_getf());

        // get mws pattern index
        mwsifstat.pattern_index = bt_mwsifstat_mws_pattern_val_getf();
        // get mws pattern start indication
        start_ind = bt_mwsifstat_mws_pattern_start_ind_val_getf();
        // get inactivity duration
        inact_dur = bt_mwsifstat_mws_inactivity_duration_val_getf();

        if (!start_ind)
        {
            mwsifstat.pattern_index = SAM_INDEX_CONTINUE;
        }
    }

    if (MWS_INACT_DUR_INFINITE == inact_dur)
    {
        // Inactive for indeterminate duration - SAM disabled.
        mwsifstat.pattern_index = SAM_DISABLED;
    }
    else
    {
        // Activation pending - convert inactivity duration (5ms units) to slots.
        mwsifstat.slot_offset = (inact_dur << 3);
    }

    return mwsifstat;
}

uint8_t ld_mws_inactivity_duration_get(void)
{
    uint8_t inact_dur = MWS_INACT_DUR_INFINITE;

    if (bt_coexifcntl0_mwscoex_en_getf())
    {
        // Sample the active MWS interface for pattern index & inactivity duration
        bt_mwsifstat_mws_if_samp_setf(1);
        while (bt_mwsifstat_mws_if_samp_getf());

        // get inactivity duration
        inact_dur = bt_mwsifstat_mws_inactivity_duration_val_getf();
    }

    return inact_dur;
}

void ld_local_sam_submap_write(const uint8_t *sam_ptr, uint8_t sam_length)
{
    DBG_SWDIAG(SAM, LSUBMAP_CFG, 1);

    // DIRECT MAPPING
    // LMP SAM_Submap definition -> HW definition of SAM Map dibit:
    // 0: The slot is not available for either tx or rx
    // 1: The slot is available for tx but not rx
    // 2: The slot is available for rx but not tx
    // 3: The slot is available for tx and rx
    em_wr((void *)sam_ptr, EM_BT_LOCAL_SAM_SUBMAP_OFFSET, SAM_TYPE0_SUBMAP_LEN);

    // Configure Local SAM. Initially disabled as new submap invalidates old patterns.
    bt_lsamcntl0_pack(/*localsamen*/ 0, /*localsamlength*/ sam_length, /*localsamptr*/ EM_BT_LOCAL_SAM_SUBMAP_OFFSET >> 2);
    bt_lsamcntl1_local_sam_offset_setf(0); // aligned with Frame sync

    DBG_SWDIAG(SAM, LSUBMAP_CFG, 0);
}

#endif // (RW_BT_MWS_COEX)

#if (EAVESDROPPING_SUPPORT)

void ld_get_clock(uint32_t *clkn, uint16_t *fine_cnt)
{
    // Get captured clock + fine counter;
    *clkn = bt_binslotclk_binclkncnt_getf();
    *fine_cnt = (HALF_SLOT_TIME_MAX - bt_binfinecnt_getf());

    // Convert to slots, us
    if ((*clkn) & 0x1)
    {
        (*clkn) &= ~0x1;
        (*fine_cnt) += HALF_SLOT_SIZE;
    }

    *clkn = (*clkn) >> 1;
    *fine_cnt = (*fine_cnt) >> 1;
}

void ld_clock_capture_int(void)
{
    // Activate capture bit
    bt_binslotclk_get_clk_en_setf(1);
}

void ld_clock_capture_imm(void)
{
    // Immediate action, trigger external signal
    bt_binslotclk_binsampclk_setf(1);

    // Wait for bit to be cleared
    while (bt_binslotclk_binsampclk_getf());
}

void ld_set_clock(uint32_t clkn, uint16_t fine_cnt)
{
    // Convert to half-slots, half-us
    fine_cnt = fine_cnt << 1;
    clkn = clkn << 1;

    if (fine_cnt >= HALF_SLOT_SIZE)
    {
        fine_cnt -= HALF_SLOT_SIZE;
        clkn = CLK_ADD_2(clkn, 1);
    }

    // Set fine counter first
    bt_binfinecnt_setf(HALF_SLOT_TIME_MAX - fine_cnt);
    // Set the slot count and clock set bit at the same time
    bt_binslotclk_pack(0, 0, 1, clkn);
}

/**
****************************************************************************************
 * @brief Clock capture interrupt event.
 ****************************************************************************************
 */
void ld_clock_captured_isr(void)
{
    uint32_t clkn = 0;
    uint16_t fine_cnt = 0;

    // Send the MSG to ED TASK
    struct ed_clk_capt_ind *msg = KE_MSG_ALLOC(ED_CLK_CAPT_IND, TASK_ED, TASK_NONE, ed_clk_capt_ind);

    ld_get_clock(&clkn, &fine_cnt);

    msg->clkn = clkn;
    msg->fine_cnt = fine_cnt;

    ke_msg_send(msg);
}

/**
****************************************************************************************
 * @brief Clock adjusted interrupt event.
 ****************************************************************************************
 */
void ld_clock_adjusted_isr(void)
{
    // Send the MSG to ED TASK
    ke_msg_send_basic(ED_CLK_ADJ_IND, TASK_ED, TASK_NONE);
}

#endif // EAVESDROPPING_SUPPORT

///@} LD
