/**
****************************************************************************************
*
* @file lld.c
*
* @brief LLD source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // stack configuration

#include <string.h>
#include "co_math.h"

#include "arch.h"
#include "rwip.h"

#include "lld.h"                  // link layer driver
#include "lld_int.h"              // link layer driver internal
#if (BLE_CIS || BLE_BIS)
    #include "lld_int_iso.h"          // LLD Internal API for ISO
#endif // (BLE_CIS || BLE_BIS)
#include "ble_util_buf.h"         // BLE EM buffer management

#include "sch_alarm.h"            // Scheduling Alarm
#include "sch_arb.h"              // Scheduling Arbiter
#include "sch_prog.h"             // Scheduling Programmer
#include "sch_slice.h"            // Scheduling Slicer

#include "ke_mem.h"               // memory allocation
#include "ke_task.h"              // kernel task management

#include "em_map.h"

#include "reg_blecore.h"          // BLE core registers
#include "reg_em_ble_cs.h"        // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"   // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"   // BLE EM RX descriptors
#include "reg_em_ble_rx_cte_desc.h" //  BLE EM RX CTE descriptors
#include "reg_em_ble_wpal.h"      // BLE EM White list private
#include "reg_em_ble_ral.h"       // BLE EM Resolving list
#include "reg_em_et.h"            // EM Exchange Table
#include "btdm_patch.h"
#include "reg_ipcore.h"
/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD resolvable private addresses renewal event parameters structure
struct lld_rpa_renew_env_tag
{
    /// Event for scheduling
    struct sch_arb_elt_tag   evt;
    /// Alarm for the addresses renewal operation
    struct sch_alarm_tag     alarm;
};

///Access address generation structure
struct access_addr_gen
{
    /// random
    uint8_t intrand;
    /// index 1
    uint8_t ct1_idx;
    /// index 2
    uint8_t ct2_idx;
};

/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

/// Table indicating the expected sync position depending on the PHY used
uint16_t lld_exp_sync_pos_tab[CO_RATE_MAX];

#if (AUDIO_SYNC_SUPPORT)
    /// Table indicating the transmit delay depending on the PHY used
    uint16_t lld_tx_path_delay[CO_RATE_MAX];
#endif //(AUDIO_SYNC_SUPPORT)

///Constant nibble to use as top 4 MSBs in AA gen, to have at least 2 transitions
const uint8_t LLM_AA_CT1[3]  = {0x02, 0x04, 0x06};

///Constant nibble to use in AA gen with two repetition each to satisfy various requirements
const uint8_t LLM_AA_CT2[2]  = {0x0C, 0x03};


/// Table for converting AUX PHY to rate (Warning: the coded PHY is converted to 125K by default)
__STATIC const uint8_t lld_aux_phy_to_rate[] =
{
    [AUX_PHY_1MBPS]  = CO_RATE_1MBPS,
    [AUX_PHY_2MBPS]  = CO_RATE_2MBPS,
    [AUX_PHY_CODED]  = CO_RATE_125KBPS,
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// Event for scheduling the resolvable private addresses renewal
__STATIC struct lld_rpa_renew_env_tag *lld_rpa_renew_env;

/// LLD environment variable
struct lld_env_tag lld_env;

/// Access address generation structure
struct access_addr_gen aa_gen;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */
#define LLD_LOW_BUFFER_LIMIT 1024

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Search if a device is in resolving address list
 *
 * @param[in] bd_addr      BD Address
 * @param[in] bd_addr_type BD Address type (0: public | 1: random)
 *
 * @return position of the device in the list (Max value if not found)
 ****************************************************************************************
 */
uint8_t lld_ral_search(const struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t position;
    struct bd_addr tmp_addr;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_RAL_SEARCH_BIT, uint8_t, bd_addr, bd_addr_type);

    for (position = 0 ; position < BLE_RESOL_ADDR_LIST_MAX ; position++)
    {
        if (em_ble_ral_info_entry_valid_getf(position))
        {
            // Do a burst read in the exchange memory
            em_rd(&tmp_addr, REG_EM_BLE_RAL_ADDR_GET(position) + EM_BLE_RAL_PEER_ID_INDEX * 2, sizeof(struct bd_addr));

            // Compare the 2 BD addresses
            if (((bd_addr_type & ADDR_RAND) == em_ble_ral_info_peer_id_type_getf(position)) && (co_bdaddr_compare(&tmp_addr, bd_addr)))
            {
                break;
            }
        }
    }

    return position;
}

/**
 ****************************************************************************************
 * @brief Handle RPA renewal instant alarm
 ****************************************************************************************
 */
__STATIC void lld_rpa_renew_instant_cbk(struct sch_alarm_tag *elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RPA_RENEW_INSTANT_CBK_BIT, elt);

    if (lld_rpa_renew_env != NULL)
    {
        struct sch_arb_elt_tag *evt = &(lld_rpa_renew_env->evt);
        uint8_t position;

        for (position = 0 ; position < BLE_RESOL_ADDR_LIST_MAX ; position++)
        {
            // check if resolving address list entry is valid
            if (em_ble_ral_info_entry_valid_getf(position))
            {
                // retrieve if local and peer irk are valid
                bool l_irk_valid = em_ble_ral_info_local_irk_valid_getf(position);
                bool p_irk_valid = em_ble_ral_info_peer_irk_valid_getf(position);

                // set the renew bit to 1 if corresponding IRK is valid
                em_ble_ral_info_local_rpa_renew_setf(position, l_irk_valid);
                em_ble_ral_info_peer_rpa_renew_setf(position, p_irk_valid);
            }
        }

        // Remove event
        sch_arb_remove(evt, true);

        // Free the memory
        ke_free(lld_rpa_renew_env);
        lld_rpa_renew_env = NULL;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle RPA renewal event start notification
 ****************************************************************************************
 */
__STATIC void lld_rpa_renew_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RPA_RENEW_EVT_START_CBK_BIT, evt);

    if (lld_rpa_renew_env != NULL)
    {
        // Point to parameters
        struct lld_rpa_renew_env_tag *rpa_par = (struct lld_rpa_renew_env_tag *) evt;

        uint32_t clock = lld_read_clock();

        // Abort scan or advertising if needed
        ble_rwblecntl_advert_abort_setf(1);
        ble_rwblecntl_scan_abort_setf(1);

        // Program addresses renewal at the beginning of the reserved time window, via an alarm
        DBG_MEM_INIT(&(rpa_par->alarm), sizeof(struct sch_alarm_tag));
        rpa_par->alarm.time.hs = CLK_ADD_2(clock, rwip_prog_delay);
        rpa_par->alarm.time.hus = 0;
        rpa_par->alarm.cb_alarm = &lld_rpa_renew_instant_cbk;

        sch_alarm_set(&rpa_par->alarm);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle RPA renewal event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_rpa_renew_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RPA_RENEW_EVT_CANCELED_CBK_BIT, evt);

    if (lld_rpa_renew_env != NULL)
    {
        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_RPA_RENEW_IDX));

        // Reschedule
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            ASSERT_ERR_FORCE(0);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Initialize or reset BLE core

 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
__STATIC void lld_core_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_CORE_INIT_BIT, is_reset);

    uint8_t diag_cfg[PARAM_LEN_DIAG_BLE_HW];
    uint8_t length = PARAM_LEN_DIAG_BLE_HW;

    if (is_reset)
    {
        // Check consistency between HW register compiled version and the BLE core version read on the platform
        //ASSERT_INFO((ble_version_get() == BLE_VERSION_RESET), ble_version_get(), BLE_VERSION_RESET);

        // Turn off BLE Core
        ble_rwblecntl_rwble_en_setf(0);

        // Reset the BLE state machines
        ble_rwblecntl_master_soft_rst_setf(1);
        while (ble_rwblecntl_master_soft_rst_getf());

        // Disable all the BLE interrupts
        ble_intcntl0_set(0);

        // And acknowledge any possible pending ones
        ble_intack0_clear(0xFFFFFFFF);

        // Clear white list
        for (uint8_t position = 0; position < BLE_WHITELIST_MAX ; position++)
        {
            // Initialize entry
            em_ble_list_info_pack(position, 0, 0, 0, 0);
        }

        // Clear resolving list
        lld_res_list_clear();

        // Disable PRBS generator
        ble_rftestcntl_txpldsrc_setf(0);

        // Clear diagport configuration
        // For debug
        //ble_diagcntl_set(0);
    }
    // No break

    // Initializes the diagnostic port
    if (rwip_param.get(PARAM_ID_DIAG_BLE_HW, &length, diag_cfg) == PARAM_OK)
    {
        ble_diagcntl_pack(1, diag_cfg[3], 1, diag_cfg[2], 1, diag_cfg[1], 1, diag_cfg[0]);
    }

    // Set default half window size, MD_DSB control, anonymous advertisements reporting disable
    ble_rwblecntl_set(BLE_MD_DSB_BIT | BLE_ANONYMOUS_ADV_FILT_EN_BIT | BLE_ADVERTFILT_EN_BIT
                      | (((BLE_NORMAL_WIN_SIZE + 1) >> 1) << BLE_RXWINSZDEF_LSB));

    // Enable default BLE interrupts
    ble_intcntl0_set(BLE_RXINTMSK_BIT      | BLE_TXINTMSK_BIT
                     | BLE_ENDEVTINTMSK_BIT  | BLE_SKIPEVTINTMSK_BIT
                     | BLE_ERRORINTMSK_BIT
#if (BLE_ISO_PRESENT)
                     | BLE_ISORXINTMSK_BIT   | BLE_ISOTXINTMSK_BIT
#endif // (BLE_ISO_PRESENT)
#if (EAVESDROPPING_SUPPORT)
                     | BLE_ADVTRIGINTMSK_BIT
#endif // EAVESDROPPING_SUPPORT
                    );

    // Initialize receiver and transmitter AUXPTR threshold for automatic HW control
    ble_advtim_pack(/*txauxptrthr*/LLD_TX_AUX_PTR_THR, /*rxauxptrthr*/LLD_RX_AUX_PTR_THR, /*advint*/BLE_ADV_INT_US);

    // Set white list addresses and count
    ble_wpalcntl_pack(BLE_WHITELIST_MAX, REG_EM_ADDR_GET(BLE_WPAL, 0));
    // Set current WL pointer
    ble_wpalcurrentptr_set(REG_EM_ADDR_GET(BLE_WPAL, 0));

    // Set resolving list addresses and count
    ble_ralcntl_pack(BLE_RESOL_ADDR_LIST_MAX, REG_EM_ADDR_GET(BLE_RAL, 0));
    // Set current RAL pointer
    ble_ralcurrentptr_set(REG_EM_ADDR_GET(BLE_RAL, 0));

    // RAL and List Search engines timeout delay - valid range 0 to 63 (units in  us)
    ble_search_timeout_set(BLE_SEARCH_TIMEOUT_RST);

    // Initialize common RX descriptors list
    for (int i = 0 ; i < EM_NUM_BLE_RX_DESC ; i++)
    {
        uint16_t acl_buf_ptr;

        // Point each descriptor to next RX descriptor (in circular loop)
        em_ble_rxcntl_pack(i, 0, (REG_EM_ADDR_GET(BLE_RX_DESC, CO_MOD((i + 1), EM_NUM_BLE_RX_DESC))));

        // Allocate a ACL buffer
        acl_buf_ptr = ble_util_buf_rx_alloc();
        ASSERT_ERR(acl_buf_ptr != 0);

        // Point the descriptor to the ACL buffer
        em_ble_rxdataptr_setf(i, acl_buf_ptr);
    }

    // Set the first RX descriptor pointer into the HW and the ET base ptr to 0
    ble_currentrxdescptr_setf(REG_EM_ADDR_GET(BLE_RX_DESC, 0));
    ip_etptr_setf(EM_ET_OFFSET >> 2);

    // Initial permission/status of CS is unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + EM_BLE_CS_OFFSET), EM_BLE_CS_NB * REG_EM_BLE_CS_SIZE, false, false, false);

    DBG_MEM_GRANT_CTRL((const void *)REG_EM_BLE_CS_BASE_ADDR, true);
    // Initialize Control structures
    for (int i = 0 ; i < EM_BLE_CS_INDEX_MAX ; i++)
    {
        // Set link label
        em_ble_linkcntl_linklbl_setf(i, i);
    }
    DBG_MEM_GRANT_CTRL((const void *)REG_EM_BLE_CS_BASE_ADDR, false);

    // Incorporate rxpathdly into  expected sync position
    lld_exp_sync_pos_tab[CO_RATE_1MBPS]   = BLE_PREAMBLE_ACCESS_ADDR_DUR_1MBPS   + ble_radiotxrxtim0_rxpathdly0_getf();
    lld_exp_sync_pos_tab[CO_RATE_2MBPS]   = BLE_PREAMBLE_ACCESS_ADDR_DUR_2MBPS   + ble_radiotxrxtim1_rxpathdly1_getf();
    lld_exp_sync_pos_tab[CO_RATE_125KBPS] = BLE_PREAMBLE_ACCESS_ADDR_DUR_125KBPS + ble_radiotxrxtim2_rxpathdly2_getf();
    lld_exp_sync_pos_tab[CO_RATE_500KBPS] = BLE_PREAMBLE_ACCESS_ADDR_DUR_500KBPS + ble_radiotxrxtim2_rxpathdly2_getf();

#if (AUDIO_SYNC_SUPPORT)
    lld_tx_path_delay[CO_RATE_1MBPS] = ble_radiotxrxtim0_txpathdly0_getf();
    lld_tx_path_delay[CO_RATE_2MBPS] = ble_radiotxrxtim1_txpathdly1_getf();
    lld_tx_path_delay[CO_RATE_125KBPS] = ble_radiotxrxtim2_txpathdly2_getf();
    lld_tx_path_delay[CO_RATE_500KBPS] = ble_radiotxrxtim2_txpathdly2_getf();
#endif //(AUDIO_SYNC_SUPPORT)

    // Initialize random seed for RPA generator
    ble_ral_local_rnd_pack(1, co_rand_word() & BLE_LRND_VAL_MASK);
    ble_ral_peer_rnd_pack(1, co_rand_word() & BLE_PRND_VAL_MASK);

#if (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    // Initialize common RX CTE descriptors list
    for (int i = 0 ; i < EM_BLE_RX_CTE_DESC_NB ; i++)
    {
        // Point each descriptor to next RX descriptor (in circular loop)
        em_ble_rxctecntl_pack(i, 0, (REG_EM_ADDR_GET(BLE_RX_CTE_DESC, CO_MOD((i + 1), EM_BLE_RX_CTE_DESC_NB))));
    }

    // Set first RX CTE descriptor pointer
    ble_dfcurrentptr_setf(REG_EM_ADDR_GET(BLE_RX_CTE_DESC, 0));
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

#if (EAVESDROPPING_SUPPORT)
    // Disable the advertising trigger
    ble_advtrigcntl_advtrigen_setf(0);
    ble_intack0_advtrigintack_clearf(1);
#endif // EAVESDROPPING_SUPPORT

    // Un-mask the ISO GPIO 0
    ble_isogpiocntl_isogpiomsk_setf(0x1);

    // Enable the BLE core
    ble_rwblecntl_rwble_en_setf(1);
}

/*
 * MODULE FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_rxdesc_free(void)
{
    uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RXDESC_FREE_BIT);

    // Check if the RX descriptor needs a buffer
    if (em_ble_rxdataptr_getf(rxdesc_idx) == 0)
    {
        uint16_t em_ptr = ble_util_buf_rx_alloc();

        // if no buffer are available
        if (em_ptr == 0)
        {
            // Indicate this RX descriptor has no buffer for data reception
            lld_env.rxdesc_data_ptr_bf |= CO_BIT(rxdesc_idx);
        }
        else
        {
            // link to the descriptor
            em_ble_rxdataptr_setf(rxdesc_idx, em_ptr);
        }
    }

    // Free descriptor
    em_ble_rxcntl_rxdone_setf(rxdesc_idx, 0);

    // Move RX descriptor index
    CO_VAL_INC(lld_env.curr_rxdesc_index, EM_NUM_BLE_RX_DESC);

    // Yier: Avoid MAC already change the descriptor
    if (em_ble_rxcntl_rxdone_getf(lld_env.curr_rxdesc_index) == 0)
    {
        // Yier: Flow ctrl to avoid ke buffer is not enough.
        if (ke_is_lower_than_target(KE_MEM_KE_MSG, LLD_LOW_BUFFER_LIMIT))
        {
            em_ble_rxcntl_rxdone_setf(lld_env.curr_rxdesc_index, 1);
            lld_env.in_low_buffer = 1;
        }
    }
}

bool lld_rxdesc_check(uint8_t label)
{
    uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;
    bool done = em_ble_rxcntl_rxdone_getf(rxdesc_idx);
    uint8_t rx_lbl = em_ble_rxfcntsync_rxlinklbl_getf(rxdesc_idx);

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_RXDESC_CHECK_BIT, bool, label);
    // Yier: System buffer is not enough, should not handle rx message.
    if (lld_env.in_low_buffer)
        return 0;

    // Check if the RX descriptor is consumed by the frame
    return (done && (rx_lbl == label));
}

// Yier, func to recovery descriptor.
void lld_recovery_rxdesc(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RECOVERY_RXDESC_BIT);
    if (lld_env.in_low_buffer)
    {
        if (!ke_is_lower_than_target(KE_MEM_KE_MSG, LLD_LOW_BUFFER_LIMIT))
        {
            int index = lld_env.curr_rxdesc_index;
            int index_next = CO_VAL_INC(index, EM_NUM_BLE_RX_DESC);
            if (em_ble_rxcntl_rxdone_getf(index_next) == 1)
                ASSERT_ERR_FORCE(0);
            em_ble_rxcntl_rxdone_setf(index, 0);
            lld_env.in_low_buffer = 0;
        }
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_INIT_BIT, is_reset);

    uint8_t length;
    if (is_reset)
    {
        // Free RPA renew environment
        if (lld_rpa_renew_env != NULL)
        {
            ke_free(lld_rpa_renew_env);
        }
    }
    // No break


    // Initialize environment
    memset(&lld_env, 0, sizeof(lld_env));

    // Initialize channel map table and number of used channels
    for (uint8_t i = 0; i < BLE_DATA_CHANNEL_NB; i++)
    {
        // All channels are enabled by default
        lld_env.sec_adv_ch_list[i] = i;
    }
    lld_env.nb_used_ch = BLE_DATA_CHANNEL_NB;

    lld_rpa_renew_env = NULL;

    // Initialize the LE Coded PHY 500 Kbps selection (false: 125 Kbps, true: 500 Kbps), 125 Kbps by default
    length = PARAM_LEN_LE_CODED_PHY_500;
    rwip_param.get(PARAM_ID_LE_CODED_PHY_500, &length, (uint8_t *) &lld_env.le_coded_phy_500);

    //the random byte used for incrementing is generated only once
    aa_gen.intrand = co_rand_byte();
    aa_gen.ct1_idx = 0;
    aa_gen.ct2_idx = 0;

    // Initialize default slave MD bit
    length = PARAM_LEN_DFT_SLAVE_MD;
    if (rwip_param.get(PARAM_ID_DFT_SLAVE_MD, &length, &lld_env.dft_slave_md) != PARAM_OK)
    {
        // Default MD set to 1 in order to receive master's ACK in the same event
        lld_env.dft_slave_md = 1;
    }

    // Initialize TX/RX path compensation
    lld_env.tx_path_comp = BLE_DFT_TX_PATH_RF_COMPENSATION;
    lld_env.rx_path_comp = BLE_DFT_RX_PATH_RF_COMPENSATION;

    // Initialize BLE CORE + EM
    lld_core_init(is_reset);

#if(BLE_BROADCASTER)
    // Initialize LE advertising driver
    lld_adv_init(is_reset);
    // Initialize LE periodic advertising driver
    lld_per_adv_init(is_reset);
#endif // (BLE_BROADCASTER)

#if(BLE_OBSERVER)
    // Initialize LE scanning driver
    lld_scan_init(is_reset);
    // Initialize LE periodic advertising Rx driver
    lld_sync_init(is_reset);
#endif // (BLE_OBSERVER)

#if(BLE_CENTRAL)
    // Initialize LE initiating driver
    lld_init_init(is_reset);
#endif // (BLE_CENTRAL)

    // Initialize LE Test mode driver
    lld_test_init(is_reset);

#if (BLE_CENTRAL|| BLE_PERIPHERAL)
    // Initialize LE Connection driver
    lld_con_init(is_reset);
#endif //(BLE_CENTRAL|| BLE_PERIPHERAL)

#if (BLE_CIS || BLE_BIS)
    // Initialize ISO driver
    lld_iso_init(is_reset);
#endif //(BLE_CIS || BLE_BIS)

#if (BLE_ISO_PRESENT)
    lld_isoal_init(is_reset);
#endif // (BLE_ISO_PRESENT)

#if (BLE_CH_SCAN_SUPPORT)
    lld_ch_scan_init(is_reset);
#endif // (BLE_CH_SCAN_SUPPORT)
}

uint32_t lld_read_clock(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_READ_CLOCK_BIT, uint32_t);

    return rwip_time_get().hs;
}

bool lld_rxdesc_buf_ready(uint16_t em_ptr)
{
    bool assign = false;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_RXDESC_BUF_READY_BIT, bool, em_ptr);
    // *** HW Issue: ***
    // RX Data pointer on RX Descriptor is updated (written by HW) when RX descriptor is updated
    // This can create a race condition where RX Data Pointer is cleared by HW creating a RX Data memory leak.
    //
    // if(lld_env.rxdesc_data_ptr_bf != 0)
    // {
    //     uint8_t rxdesc_idx = co_ctz(lld_env.rxdesc_data_ptr_bf);
    //     ASSERT_ERR(em_ptr != 0);
    //
    //     // Link to the descriptor
    //     em_ble_rxdataptr_setf(rxdesc_idx, em_ptr);
    //     // Indicate the RX descriptor has a buffer for data reception
    //     lld_env.rxdesc_data_ptr_bf &= ~CO_BIT(rxdesc_idx);
    //
    //     assign = true;
    // }

    return (assign);
}

void lld_rpa_renew(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RPA_RENEW_BIT);

    if (lld_rpa_renew_env == NULL)
    {
        // Allocate an scheduling arbitration and alarm event structure
        lld_rpa_renew_env =  LLD_ALLOC_ENV(lld_rpa_renew_env_tag);

        if (lld_rpa_renew_env != NULL)
        {
            struct sch_arb_elt_tag *evt = &(lld_rpa_renew_env->evt);

            // Initialize event parameters (common part)
            evt->cb_cancel          = &lld_rpa_renew_evt_canceled_cbk;
            evt->cb_start           = &lld_rpa_renew_evt_start_cbk;
            evt->cb_stop            = NULL;
            evt->current_prio       = rwip_priority[RWIP_PRIO_RPA_RENEW_IDX].value;
            evt->duration_min       = 8 * HALF_SLOT_SIZE;
            evt->stop_latency       = 0;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 10, 1);

            /* Schedule the address renewal safely by reserving a free time window, in order to avoid potential conflicts
             * between HW and SW access on the same fields */
            evt->time.hs          = lld_read_clock();
            evt->time.hus = 0;

            GLOBAL_INT_DISABLE();

            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                ASSERT_ERR_FORCE(0);
            }

            GLOBAL_INT_RESTORE();
        }
    }
    else
    {
        ASSERT_WARN(0, 0, 0);
    }
}

void lld_white_list_add(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_WHITE_LIST_ADD_BIT, position, bd_addr, bd_addr_type);

    if (bd_addr_type != ANONYMOUS_ADV_ADDR_TYPE)
    {
        // Write the EM list entry
        em_ble_list_info_entry_valid_setf(position, 1);
        em_ble_list_info_idtype_setf(position, bd_addr_type);
        em_ble_list_info_in_wl_setf(position, 1);
        em_wr(bd_addr, REG_EM_BLE_WPAL_ADDR_GET(position) + (EM_BLE_LIST_BDADDR_INDEX * 2), sizeof(struct bd_addr));

        // Check if the device is in the resolving list
        position = lld_ral_search(bd_addr, bd_addr_type);

        if (position < BLE_RESOL_ADDR_LIST_MAX)
        {
            // Indicate this device as white listed in resolving list
            em_ble_ral_info_in_whlist_setf(position, 1);
        }
    }
    else
    {
        // Devices sending anonymous advertisements are reported
        ble_rwblecntl_anonymous_adv_filt_en_setf(0);
    }
}

void lld_white_list_rem(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_WHITE_LIST_REM_BIT, position, bd_addr, bd_addr_type);

    if (bd_addr_type != ANONYMOUS_ADV_ADDR_TYPE)
    {
        // Indicate the device is no more in white list to HW
        em_ble_list_info_in_wl_setf(position, 0);

        // If device not in periodic advertiser list, remove the entry
        if (0 == em_ble_list_info_in_peradvl_getf(position))
        {
            em_ble_list_info_entry_valid_setf(position, 0);
        }

        // Check if the device is in the resolving list
        position = lld_ral_search(bd_addr, bd_addr_type);

        if (position < BLE_RESOL_ADDR_LIST_MAX)
        {
            // Indicate this device as not white listed in resolving list
            em_ble_ral_info_in_whlist_setf(position, 0);
        }
    }
    else
    {
        // Devices sending anonymous advertisements are not reported
        ble_rwblecntl_anonymous_adv_filt_en_setf(1);
    }
}

void lld_per_adv_list_add(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type, uint8_t adv_sid)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_PER_ADV_LIST_ADD_BIT, position, bd_addr, bd_addr_type, adv_sid);

    // Check if the device is in the resolving list
    uint8_t position_ral = lld_ral_search(bd_addr, bd_addr_type);

    // Write the EM list entry
    em_ble_list_info_entry_valid_setf(position, 1);
    em_ble_list_info_idtype_setf(position, bd_addr_type);
    em_ble_list_info_in_peradvl_setf(position, 1);
    em_wr(bd_addr, REG_EM_BLE_WPAL_ADDR_GET(position) + (EM_BLE_LIST_BDADDR_INDEX * 2), sizeof(struct bd_addr));

    // Add the ADV_SID to the entry
    lld_env.adv_sids[position] |= (1 << adv_sid);
    em_ble_list_sid_set(position, lld_env.adv_sids[position]);

    if (position_ral < BLE_RESOL_ADDR_LIST_MAX)
    {
        // Indicate this device as listed in periodic advertising list, in resolving list
        em_ble_ral_info_in_peradv_list_setf(position_ral, 1);

        // Indicate the followed advertising SIDs
        em_ble_ral_peer_sid_setf(position_ral, lld_env.adv_sids[position]);
    }
}

void lld_per_adv_list_rem(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type, uint16_t adv_sids)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_PER_ADV_LIST_REM_BIT, position, bd_addr, bd_addr_type, adv_sids);

    // Check if the device is in the resolving list
    uint8_t position_ral = lld_ral_search(bd_addr, bd_addr_type);

    // Remove the ADV_SIDs from the entry
    lld_env.adv_sids[position] &= ~(adv_sids);
    em_ble_list_sid_set(position, lld_env.adv_sids[position]);

    if (position_ral < BLE_RESOL_ADDR_LIST_MAX)
    {
        // Update the followed advertising SIDs
        em_ble_ral_peer_sid_setf(position_ral, lld_env.adv_sids[position]);
    }

    // If no more ADV_SID to follow
    if (lld_env.adv_sids[position] == 0x0000)
    {
        // Indicate the device is no more in periodic advertiser list to HW
        em_ble_list_info_in_peradvl_setf(position, 0);

        // If device not in white list, remove the entry
        if (0 == em_ble_list_info_in_wl_getf(position))
        {
            em_ble_list_info_entry_valid_setf(position, 0);
        }

        if (position_ral < BLE_RESOL_ADDR_LIST_MAX)
        {
            // Indicate this device as not listed in periodic advertising list, in resolving list
            em_ble_ral_info_in_peradv_list_setf(position_ral, 0);
        }
    }
}

void lld_res_list_clear(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RES_LIST_CLEAR_BIT);
    // Invalid all entries
    for (uint8_t i = 0 ; i < BLE_RESOL_ADDR_LIST_MAX ; i++)
    {
        em_ble_ral_info_entry_valid_setf(i, 0);
    }
}

void lld_res_list_add(uint8_t position, struct bd_addr const *bd_addr, struct irk const *peer_irk, struct irk const *local_irk, uint8_t bd_addr_type, bool in_wl)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RES_LIST_ADD_BIT, 6, position, bd_addr, peer_irk, local_irk, bd_addr_type, in_wl);

    bool l_irk_valid = (memcmp(local_irk->key, co_null_key, KEY_LEN) != 0);
    bool p_irk_valid = (memcmp(peer_irk->key,  co_null_key, KEY_LEN) != 0);

    // Do a burst write in the exchange memory
    em_wr(bd_addr, REG_EM_BLE_RAL_ADDR_GET(position) + EM_BLE_RAL_PEER_ID_INDEX * 2, sizeof(struct bd_addr));

    // Check if local irk should be copied
    if (l_irk_valid)
    {
        // Do a burst write in the exchange memory
        em_wr(local_irk, REG_EM_BLE_RAL_ADDR_GET(position) + EM_BLE_RAL_LOCAL_IRK_INDEX * 2, KEY_LEN);
    }

    // Check if peer irk should be copied
    if (p_irk_valid)
    {
        // Do a burst write in the exchange memory
        em_wr(peer_irk, REG_EM_BLE_RAL_ADDR_GET(position) + EM_BLE_RAL_PEER_IRK_INDEX * 2, KEY_LEN);
    }

    // Write the RAL entry info
    em_ble_ral_info_pack(position, /*entryvalid*/ 1, /*connected (deprecated usage)*/ 0,
                         /*inwhlist*/ in_wl, /*inperadvlist*/ false,
                         /*pef*/ false, /*localrpavalid*/ false, /*localrparenew*/l_irk_valid,
                         /*localirkvalid*/ l_irk_valid, /*peerrpavalid*/ false, /*peerrparenew*/p_irk_valid,
                         /*peerirkvalid*/ p_irk_valid, /* peeridtype*/ bd_addr_type);
}

void lld_res_list_rem(uint8_t position)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RES_LIST_REM_BIT, position);
    // Invalidate the entry
    em_ble_ral_info_entry_valid_setf(position, 0);
}

void lld_res_list_priv_mode_update(uint8_t position, uint8_t privacy_mode)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_RES_LIST_PRIV_MODE_UPDATE_BIT, position, privacy_mode);

    // Configure privacy error filtering (0: normal, 1:privacy_error not reported)
    uint8_t pef = (PRIV_TYPE_DEVICE == privacy_mode) ? 1 : 0;
    em_ble_ral_info_pef_setf(position, pef);
}

uint8_t lld_res_list_peer_rpa_get(uint8_t position, struct bd_addr *peer_res_addr)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_RES_LIST_PEER_RPA_GET_BIT, uint8_t, position, peer_res_addr);

    // Check if the entry is valid
    if (em_ble_ral_info_peer_rpa_valid_getf(position))
    {
        // Read the current peer resolvable address
        em_rd(&peer_res_addr->addr[0], REG_EM_BLE_RAL_ADDR_GET(position) + EM_BLE_RAL_PEER_RPA_INDEX * 2, BD_ADDR_LEN);

        status = CO_ERROR_NO_ERROR;
    }

    return (status);
}

uint8_t lld_res_list_local_rpa_get(uint8_t position, struct bd_addr *local_res_addr)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_RES_LIST_LOCAL_RPA_GET_BIT, uint8_t, position, local_res_addr);

    // Check if the entry is valid
    if (em_ble_ral_info_local_rpa_valid_getf(position))
    {
        // Read the current local resolvable address
        em_rd(&local_res_addr->addr[0], REG_EM_BLE_RAL_ADDR_GET(position) + EM_BLE_RAL_LOCAL_RPA_INDEX * 2, BD_ADDR_LEN);

        status = CO_ERROR_NO_ERROR;
    }

    return (status);
}


#if (RW_DEBUG)
uint16_t lld_reg_rd(uint16_t reg_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_REG_RD_BIT, uint16_t, reg_addr);

    return REG_BLE_RD((REG_BLECORE_BASE_ADDR + reg_addr));
}

void lld_reg_wr(uint16_t reg_addr, uint16_t reg_value)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_REG_WR_BIT, reg_addr, reg_value);

    REG_BLE_WR((REG_BLECORE_BASE_ADDR + reg_addr), reg_value);
}
#endif // (RW_DEBUG)

void lld_aa_gen(uint8_t *acc_addr, uint8_t act_id)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_AA_GEN_BIT, acc_addr, act_id);

    //the pseudo random byte is generated every time
    uint8_t prand = co_rand_byte();

    //Create the AA - LSB to MSB
    acc_addr[0] = (LLM_AA_CT2[aa_gen.ct2_idx] << 4) | (aa_gen.intrand & 0x0F) | (1 << (prand & 3));  // ensures a 3rd bit set to 1
    acc_addr[1] = (LLM_AA_CT2[aa_gen.ct2_idx] << 4) | (aa_gen.intrand >> 4);
    acc_addr[2] = (LLM_AA_CT2[aa_gen.ct2_idx] << 4) | (act_id & 0x0F);  // ensures at least 1 bit different from & between Advertising AAs
    acc_addr[3] = (LLM_AA_CT1[aa_gen.ct1_idx] << 4) | (prand >> 4) | (1 << (prand & 3));  // ensures no more than six consecutive 0s

    aa_gen.ct1_idx = CO_MOD((aa_gen.ct1_idx + 1), 3);
    aa_gen.ct2_idx = CO_MOD((aa_gen.ct2_idx + 1), 2);

    //Increase random
    aa_gen.intrand ++;
}

bool lld_calc_aux_rx(struct lld_calc_aux_rx_out *aux_rx_out, uint8_t index_pkt, uint32_t aux_data)
{
    uint16_t aux_offset = GETF(aux_data, BLE_AUX_OFFSET);
    bool offload = false;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_CALC_AUX_RX_BIT, bool, aux_rx_out, index_pkt, aux_data);

    if (aux_offset)
    {
        uint32_t sync_win_size_us;
        int32_t bit_offset;

        uint8_t ll_channel = GETF(aux_data, BLE_AUX_LL_CHANNEL);
        uint8_t ca = GETB(aux_data, BLE_AUX_CA);
        uint8_t offset_units = GETB(aux_data, BLE_AUX_OFFSET_UNIT);
        uint16_t aux_offset = GETF(aux_data, BLE_AUX_OFFSET);
        uint8_t aux_phy = GETF(aux_data,  BLE_AUX_PHY);

        uint32_t aux_offset_us = AUX_OFFSET_USECS(aux_offset, offset_units); // calculate offset (us)
        uint32_t drift = AUX_DRIFT_PPM(rwip_current_drift_get(), ca); // calculate drift (ppm)

        // Read clock value where sync has been found
        uint32_t base_cnt_rxsync = (em_ble_rxclknsync1_clknrxsync1_getf(index_pkt) << 16)
                                   | em_ble_rxclknsync0_clknrxsync0_getf(index_pkt);
        // Read bit position where sync has been found
        uint16_t fine_cnt_rxsync = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(index_pkt);

        // The auxiliary packet shall not start any earlier than the Aux Offset and shall start no
        // later than the Aux Offset plus one Offset Unit. This allows the LL to round the Aux Offset to the Offset Unit.
        aux_offset_us += AUX_OFFSET_UNIT_USECS(offset_units) / 2; // add one half Offset Unit.

        // Calculate window size
        sync_win_size_us = BLE_NORMAL_WIN_SIZE + 2 * (AUX_OFFSET_UNIT_USECS(offset_units) + ((uint32_t)(AUX_OFFSET_USECS(aux_offset, offset_units) * drift) / 1000000));

        // Calculate total bit offset
        bit_offset = fine_cnt_rxsync + (aux_offset_us * 2) - sync_win_size_us - 2 * lld_exp_sync_pos_tab[em_ble_rxchass_rate_getf(index_pkt)];

        // RXRATE  = 00b: LE 1M; 01b LE 2M; 010b LE Coded;
        aux_rx_out->rate = lld_aux_phy_to_rate[aux_phy];
        // HOPCNTL = Channel Index to LL_Channel
        aux_rx_out->ch_idx = ll_channel;

        // Set the offload timestamp based on aux_offset
        aux_rx_out->time.hs  = CLK_ADD_2(base_cnt_rxsync, bit_offset / HALF_SLOT_SIZE);
        aux_rx_out->time.hus = CO_MOD(bit_offset, HALF_SLOT_SIZE);
        aux_rx_out->sync_win_size_us = sync_win_size_us;

        ASSERT_ERR(aux_rx_out->time.hus < HALF_SLOT_SIZE);

        // For Coded PHY, the baseband needs 8 symbols duration at 125Kbps to capture the sync, the sync window is extended accordingly.
        if (aux_phy == AUX_PHY_CODED)
        {
            aux_rx_out->sync_win_size_us += (BLE_NORMAL_WIN_SIZE) * 7;
        }

        offload = true;
    }

    return offload;
}

#if (BLE_PERIPHERAL || BLE_OBSERVER)
uint32_t lld_rx_timing_compute(uint32_t last_sync_ts, uint32_t *p_target_ts, int32_t *p_bit_off, uint16_t master_sca, uint8_t rate, uint32_t add_rx_win)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_RX_TIMING_COMPUTE_BIT, uint32_t, 6, last_sync_ts, p_target_ts, p_bit_off, master_sca, rate, add_rx_win);

    uint32_t offset, half_win_size;

    // Local drift and peer drift in PPM
    uint16_t local_drift = rwip_current_drift_get();
    uint16_t peer_drift = master_sca;
    // Duration between last synchronization and next expected anchor in half slots
    uint32_t clock_diff_hs = CLK_SUB(*p_target_ts, last_sync_ts);
    // Deduce max drift to take into account for next event
    uint32_t max_drift = clock_diff_hs * (local_drift + peer_drift) / 1600; // half-slots * ppm * 625 half-us / 1000000;
    // Window widening in half-us
    uint32_t win_widening = max_drift + 2 * BLE_MAX_JITTER;
    // Computed RX window size
    uint32_t rx_win_size = 2 * win_widening + add_rx_win;

    // Ensure window is larger than minimum value
    rx_win_size = co_max(rx_win_size, 2 * BLE_NORMAL_WIN_SIZE);

    // Half window size
    half_win_size = rx_win_size >> 1;
    offset = (half_win_size) / HALF_SLOT_SIZE;

    *p_target_ts = CLK_SUB(*p_target_ts, offset);
    // retrieve bit offset according to remaining half us
    *p_bit_off -= (half_win_size - (offset * HALF_SLOT_SIZE));

    // ensure that bit offset is not negative
    if (*p_bit_off < 0)
    {
        *p_target_ts = CLK_SUB(*p_target_ts, 1);
        *p_bit_off += HALF_SLOT_SIZE;
    }

    // For Coded PHY, the baseband needs 8 symbols duration at 125Kbps to capture the sync, the sync window is extended accordingly.
    if (co_rate_to_phy[rate] == PHY_CODED_VALUE)
    {
        rx_win_size += (2 * BLE_NORMAL_WIN_SIZE) * 7;
    }

    return rx_win_size;
}
#endif // (BLE_PERIPHERAL || BLE_OBSERVER)

#if (BLE_BROADCASTER)
void lld_sec_adv_ch_map_set(const struct le_ch_map *p_ch_map)
{
    uint8_t idx = 0;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PATCH_TYPE, LLD_SEC_ADV_CH_MAP_SET_FUNC_BIT, p_ch_map);

    GLOBAL_INT_DISABLE();

    // Construct the channel map lookup table
    for (uint8_t i = 0 ; i < BLE_DATA_CHANNEL_NB ; i++)
    {
        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;

        if (p_ch_map->map[byte_idx] & (1 << bit_pos))
        {
            lld_env.sec_adv_ch_list[idx] = i;
            idx++;
        }
    }

    // Store number of good channels
    lld_env.nb_used_ch = idx;

    GLOBAL_INT_RESTORE();
}

uint8_t lld_ch_idx_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_PATCH_TYPE, LLD_CH_IDX_GET_BIT, uint8_t);

    // Select randomly an index for the channel map lookup table
    uint8_t idx = CO_MOD(co_rand_byte(), lld_env.nb_used_ch);

    return lld_env.sec_adv_ch_list[idx];
}
#endif // (BLE_BROADCASTER)

void lld_path_comp_set(int16_t tx_path_comp, int16_t rx_path_comp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_PATCH_2_TYPE, LLD_PATH_COMP_SET_2_FUN_BIT, tx_path_comp, rx_path_comp);
    // Set the TX/RX path compensation
    lld_env.tx_path_comp = tx_path_comp;
    lld_env.rx_path_comp = rx_path_comp;
}

int16_t lld_tx_path_comp_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_PATCH_2_TYPE, LLD_TX_PATH_COMP_GET_2_FUN_BIT, int16_t);
    // Get the RX path compensation
    return lld_env.tx_path_comp;
}

int16_t lld_rx_path_comp_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_PATCH_2_TYPE, LLD_RX_PATH_COMP_GET_2_FUN_BIT, int16_t);
    // Get the TX path compensation
    return lld_env.rx_path_comp;
}

int8_t lld_tx_power_dbm_get(uint8_t idx, uint8_t type)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PATCH_2_TYPE, LLD_TX_POWER_DBM_GET_2_FUN_BIT, int8_t, idx, type);
    // Get the radiative TX power
    return (rwip_rf.txpwr_dbm_get(idx, MOD_GFSK, type) + lld_env.tx_path_comp / 10);
}

///@} LLD
