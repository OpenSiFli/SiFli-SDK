/**
****************************************************************************************
*
* @file lld_int.h
*
* @brief LLD Internal API
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

#ifndef LLD_INT_H_
#define LLD_INT_H_

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

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean

#include "lld.h"


/*
 * MACROS
 ****************************************************************************************
 */


/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// Allocate an environment with an associated data structure
#define LLD_ALLOC_ENV(data_struct)           (struct data_struct *) ke_malloc_system(sizeof(struct data_struct), KE_MEM_ENV);

/// Initialize memory of an environment and an associated data structure
#define LLD_INIT_ENV(env, data_struct)       memset(env , 0, sizeof(struct data_struct));

#if 0 //(RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
    /// Coexistence disabled
    #define BLECOEX_DISABLED        0
    /// Coexistence enabled
    #define BLECOEX_ENABLED         1

    /// Coexistence Definitions
    #define BLEMPRIO_CONREQ     0
    #define BLEMPRIO_LLCP        1
    #define BLEMPRIO_DATA       2
    #define BLEMPRIO_INITSC     3
    #define BLEMPRIO_ACTSC      4
    #define BLEMPRIO_CONADV     5
    #define BLEMPRIO_NCONADV    6
    #define BLEMPRIO_PASSC      7
#endif // (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)

/// Undefined BT clock value
#define LLD_CLOCK_UNDEF       0xFFFFFFFF

/// Max fine counter value
#define LLD_FINECNT_MAX             (HALF_SLOT_SIZE - 1)

/// Adv packet error masking
#define LLD_ADV_ERR_MASK        (EM_BLE_SYNC_ERR_BIT|EM_BLE_LEN_ERR_BIT|EM_BLE_CRC_ERR_BIT|EM_BLE_PRIV_ERR_BIT|EM_BLE_TYPE_ERR_BIT)

/// ADV_INT (time between two ADV_XXX packets) in us
#define BLE_ADV_INT_US               (729)

/// Minimum scan window size when advertising using legacy PDUs in us
#define BLE_ADV_MIN_SCAN_WIN_SIZE_US (40)

/// Calculate AUX OFFSET in usecs from AuxPtr AUX Offset and Offset Units fields
#define AUX_OFFSET_USECS(aux_offset, offset_units) ((aux_offset) * ((offset_units)?300:30))

/// AUX OFFSET Offset Units in usecs
#define AUX_OFFSET_UNIT_USECS(offset_units) ((offset_units)?300:30)

/// Calculate drift in ppm from local drift and AuxPtr CA field
#define AUX_DRIFT_PPM(local_drift, ca) ((local_drift) + ((ca)?50:500))

/// Margin for event time reservation (in half-us)
#define BLE_RESERVATION_TIME_MARGIN_HUS    (2 * IP_PREFETCHABORT_TIME_US)

/// Get the normal rate window size (in us) for the specified rate (@see enum phy_rate)
#define BLE_RATE_NORMAL_WIN_SIZE(rate)     ((co_rate_to_phy[rate] == PHY_CODED_VALUE)?(BLE_PHY_CODED_NORMAL_WIN_SIZE):(BLE_NORMAL_WIN_SIZE))


/// AUX_PTR Thresholds, below which HW manages AuxPtr automatically in same event (granularity 16us, 0 disabled)
#define LLD_TX_AUX_PTR_THR   (0xFF) // 255 x 16 = 4080us (max value)
#define LLD_RX_AUX_PTR_THR   (150) // 150 x 16 = 2520us (optimized for SW offload)

/// Maximum supported CTE IQ samples - CTE RX descriptors dimensioned REG_EM_BLE_RX_CTE_DESC_SIZE to accommodate maximum
// An IQ sample is taken each usec during the reference period (8us) and one each sample slot thereafter.
// This results in 8 reference IQ samples, 1 to 37 IQ samples with 2 us slots, and 2 to 74 IQ samples with 1 us slots, meaning 9 to 82 samples in total.
#define LLD_MAX_CTE_IQ_SAMPLES   (82)

/// Undefined  RSSI value
#define LLD_RSSI_UNDEF                  (0xFF)

/// Undefined Path Loss zone
#define LLD_PATH_LOSS_UNDEF             (0xFF)

/// Hoping mode
enum lld_hop_mode
{
    /// Frequency Hopping Scheme #1
    LLD_HOP_MODE_CHAN_SEL_1         = 0,
    /// Frequency Hopping Scheme #2
    LLD_HOP_MODE_CHAN_SEL_2         = 1,
#if (BLE_BIS || BLE_CIS)
    /// Use Frequency Hopping Sequence defined by CS-HOP_SEQ_PTR
    LLD_HOP_MODE_SEQ_PTR            = 2,
#endif // (BLE_BIS || BLE_CIS)
#if (BLE_ADV_CH_IDX)
    /// Primary Advertising Channel randomized order mode
    LLD_HOP_MODE_RAND_PRIM_ADV_CH   = 3,
#endif // (BLE_ADV_CH_IDX)
};

#if (BLE_ISO_PRESENT)
/// Isochronous link type
enum lld_iso_mode
{
    /// 0x0: Isochronous Mode 0
    LLD_ISO_MODE_0     = 0,
#if (BLE_CIS)
    /// 0x1: Connected Isochronous  (CIG / CIS)
    LLD_ISO_MODE_CI    = 1,
#endif // (BLE_CIS)
#if (BLE_BIS)
    /// 0x2: Broadcast Isochronous (BIG / BIS)
    LLD_ISO_MODE_BI    = 2,
#endif // (BLE_BIS)
};
#endif // (BLE_ISO_PRESENT)

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_RX_DESC_NB <= 8)
    typedef uint8_t lld_rx_data_ptr_bf_t;
#elif (BLE_RX_DESC_NB <= 16)
    typedef uint16_t lld_rx_data_ptr_bf_t;
#else
    typedef uint32_t lld_rx_data_ptr_bf_t;
#endif // (BLE_RX_DESC_NB)

/// LLD environment structure
struct lld_env_tag
{
    /// ADV_SIDs bit field for PAL entries
    uint16_t                  adv_sids[BLE_WHITELIST_MAX_CONFIG];

    /// RF Rx path compensation value (Range: -128.0 dB <= N <= 128.0 dB, Units: 0.1 dB)
    int16_t rx_path_comp;
    /// RF Tx path compensation value (Range: -128.0 dB <= N <= 128.0 dB, Units: 0.1 dB)
    int16_t tx_path_comp;

    /// Current RX descriptor index [0:N-1]
    uint8_t                   curr_rxdesc_index;

    /// LE Coded PHY 500 Kbps selection (false: 125 Kbps, true: 500 Kbps)
    bool                      le_coded_phy_500;

    /**
     * Each bit of this field correspond to presence of data buffer onto a specific RX descriptor
     *  - if bit n = 1: RX descriptor index n has no buffer
     */
    lld_rx_data_ptr_bf_t      rxdesc_data_ptr_bf;

    /// secondary advertising channels list
    uint8_t                   sec_adv_ch_list[BLE_DATA_CHANNEL_NB];

    /// Number of used channels in the secondary advertising channel map
    uint8_t                   nb_used_ch;

    /// Default MD bit used by slave when sending a data packet on a BLE connection (set to 1 helps receiving the ACK in the same event)
    uint8_t                   dft_slave_md;
    /// System in low buffer state.
    uint8_t                   in_low_buffer;
};

/// LLD Scheduling information for next AUX channel
struct lld_calc_aux_rx_out
{
    /// Timestamp (in half-slots) and delay (in half-us) from slot boundary for AUX offload channel receive
    rwip_time_t time;

    /// Sync window size for the AUX offload channel (us)
    uint32_t sync_win_size_us;

    /// Channel index for AUX offload channel
    uint8_t ch_idx;

    /// PHY rate for AUX offload channel (@see enum lld_rate)
    uint8_t rate;
};

/*
 * VARIABLE DECLARATION
 *****************************************************************************************
 */

/// Table indicating the expected sync position depending on the rate used
extern uint16_t lld_exp_sync_pos_tab[];

#if (AUDIO_SYNC_SUPPORT)
    /// Table indicating the transmit delay depending on the PHY used
    extern uint16_t lld_tx_path_delay[];
#endif //(AUDIO_SYNC_SUPPORT)


/// LLD environment variable
extern struct lld_env_tag lld_env;

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Free current RX descriptor
 ****************************************************************************************
 */
void lld_rxdesc_free(void);

/**
 ****************************************************************************************
 * @brief Check if the current RX descriptor is consumed by a given activity
 *
 * @param[in]   label   representing the activity (as indicated in the CS)
 *
 * @return True if current RX descriptor is consumed by the activity identified by label
 ****************************************************************************************
 */
bool lld_rxdesc_check(uint8_t label);

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
uint8_t lld_ral_search(const struct bd_addr *bd_addr, uint8_t bd_addr_type);

#if (BLE_BROADCASTER)
    /**
    ****************************************************************************************
    * @brief Initialize advertising driver
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lld_adv_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Initialize periodic advertising driver
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lld_per_adv_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Get sync info from a periodic advertiser
    *
    * @param[in]  act_id        Activity ID
    * @param[out] sync_ind_ts   Timestamp of the AUX_SYNC_IND packet
    * @param[out] pa_evt_cnt    Corresponding paEventCounter value
    * @param[out] p_ch_map      Corresponding channel map
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_per_adv_sync_info_get(uint8_t act_id, uint32_t *sync_ind_ts, uint16_t *pa_evt_cnt, struct le_ch_map *p_ch_map);

    /**
    ****************************************************************************************
    * @brief Get initialization info from a periodic advertiser
    *
    * @param[in]  act_id        Activity ID
    * @param[out] aa            Periodic advertising Access Address
    * @param[out] crcinit       Periodic advertising CRC Initialization value
    * @param[out] p_ch_map      Periodic advertising channel Map
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_per_adv_init_info_get(uint8_t act_id, struct access_addr *aa, struct crc_init *crcinit, struct le_ch_map *p_ch_map);

    /**
    ****************************************************************************************
    * @brief Select an auxiliary channel index
    *
    * @return ch_idx            Selected channel index
    ****************************************************************************************
    */
    uint8_t lld_ch_idx_get(void);
#endif // (BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Generating the Access Address for a new Link.
 *
 * The function will generate the following 4 bytes:
 * |Byte 3           | Byte 2           |  Byte 1            | Byte 0            |
 * |CT1 | PRAND_4MSB | CT2 | PRAND_4LSB | CT2 | INCRAND_4MSB | CT2 | INCRAND_4LSB|
 *
 * The AA shall be a random 32-bit value, generated by the device in the Initiating State.
 * It shall have no more than six consecutive zeros or ones.
 * It shall not be the advertising channel packets' Access Address.
 * It shall not be a sequence that differs from the advertising channel packets' Access Address by only one bit.
 * It shall not have all four octets equal.
 * It shall have no more than 24 transitions.
 * It shall have a minimum of two transitions in the most significant six bits
 *
 * For each new generated AA,
 *  - PRAND is a pseudo random 1 byte number.
 *  - INCRAND is an initially generated random number, increased by 1 for each AA
 *  - CT2 is toggled between the two values
 *  - CT1 is again toggled between the 3 possible values
 *
 * Considering the formats of CT1 and CT2, the rules for AA generation are respected:
 *  - since CT2 is put between bytes, and has two equal bits on each half,
 *    it ensures no more than 6 consecutive bits inside the address. CT1 also.
 *
 *  - CT2 and CT1 ensure that the newly generated address is different and at least 1 bit
 *    different from the advertising access address
 *
 *  - since CT1 != CT2 , the four bytes of the AA can never be equal
 *
 *  - Since in a 32 bit number there are 31 transitions, and the neighboring equal bits
 *    in CT1 and CT2 represent 7 transitions, there can never be more than 31-7=24
 *    transitions in the AA
 *
 *  - The format of CT1 ensures at least two transitions in the most significant 6 bits.
 *
 * BT5.0 Requirements:
 *
 * 1/ On an implementation that also supports the LE Coded PHY the Access Address shall also
 * meet the following requirements:
 * It shall have at least three ones in the least significant 8 bits.
 * It shall have no more than eleven transitions in the least significant 16 bits.
 * - CT2 format selected to satisfy these requirements.
 *
 * 2/ It shall not be the Access Address for any enabled periodic advertising.
 * - The act_id is now passed as an argument to AA generation to ensure unique values.
 *
 * @param acc_addr Pointer to the 4 byte array space for the AA.
 *
 *
 *****************************************************************************************
 */
void lld_aa_gen(uint8_t *acc_addr, uint8_t act_id);

/**
 ****************************************************************************************
 * @brief Calculate an AUX RX activity
 *
 * @param[in]  ux_rx_out   Scheduling information for next AUX channel
 * @param[out] index_pkt   current RX descriptor index
 * @param[out] aux_data    received Aux data
 *
 * @return offload         true: offload. false: no offload
 ****************************************************************************************
 */
bool lld_calc_aux_rx(struct lld_calc_aux_rx_out *aux_rx_out, uint8_t index_pkt, uint32_t aux_data);

#if (BLE_PERIPHERAL || BLE_OBSERVER)
    /**
    ****************************************************************************************
    * @brief Compute timings for a future reception
    *
    * @param[in]      last_sync_ts      Time of the last synchronization to master (in half-slots)
    * @param[in/out]  p_target_ts       Input: current scheduling target, output: new scheduling target (in half-slots)
    * @param[in/out]  p_bit_off         Input: current bit offset, output: new bit offset (in half-us)
    * @param[in]      master_sca        Master's sleep clock accuracy (in ppm)
    * @param[in]      rate              PHY rate for reception (@see enum phy_rate)
    * @param[in]      add_rx_win        Additional RX window (in half-us)
    *
    * @return Synchronization window duration, in half-us
    ****************************************************************************************
    */
    uint32_t lld_rx_timing_compute(uint32_t last_sync_ts, uint32_t *p_target_ts, int32_t *p_bit_off, uint16_t master_sca, uint8_t rate, uint32_t add_rx_win);
#endif // (BLE_PERIPHERAL || BLE_OBSERVER)

#if (BLE_OBSERVER)
    /**
    ****************************************************************************************
    * @brief Initialize scanning driver
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */

    void lld_scan_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Initialize periodic advertising Rx driver
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lld_sync_init(bool is_reset);

    #if (BLE_BIS)
        /**
        ****************************************************************************************
        * @brief Informs periodic sync driver about new synchronization on receiver BIS or other set
        *
        * Periodic sync timings are adjusted as per the new synchronization
        *
        * @param[in]  act_id          Periodic sync activity identifier
        * @param[in]  last_sync_ts    Last synchronization timestamp (in half slots)
        * @param[in]  sync_drift_acc  Drift accumulated between expected sync time and effective sync time (in half us)
        *****************************************************************************************
        */
        void lld_sync_sync_time_update(uint8_t act_id, uint32_t last_sync_ts, int32_t sync_drift_acc);
    #endif // (BLE_BIS)

#endif // (BLE_OBSERVER)

#if (BLE_CENTRAL)
    /**
    ****************************************************************************************
    * @brief Initialize initiating driver
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lld_init_init(bool is_reset);
#endif // (BLE_CENTRAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /**
    ****************************************************************************************
    * @brief Initialize connection driver
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lld_con_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Start link
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  params        Pointer to Connection parameters structure
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_con_start(uint8_t link_id, struct lld_con_params *params);
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Initialize test driver
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void lld_test_init(bool is_reset);


#if (BLE_ISO_PRESENT)
// ****************************************************************************************
// *****                              ISOAL Specific                                  *****
// ****************************************************************************************

/**
 ****************************************************************************************
 * @brief Initialize ISOAL Driver
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void lld_isoal_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Start ISOAL Driver for a specific direction
 *
 * @param[in]  act_id           Stream Activity identifier
 * @param[in]  direction        Data Path direction (@see enum iso_rx_tx_select)
 * @param[in]  conhdl           Connection handle of a CIS or BIS (Range 0x0000-0x0EFF)
 * @param[in]  sdu_interval     SDU interval (in us)
 * @param[in]  trans_latency    Transport latency (in us)
 * @param[in]  sync_ref_offset  Offset to apply to synchronization reference (in us) (only for output direction)
 * @param[in]  max_sdu          Maximum SDU size (12 bits meaningful) (in bytes)
 * @param[in]  iso_interval     ISO interval (in us)
 * @param[in]  bn               Burst number: number of new PDUs per event
 * @param[in]  max_pdu          Maximum PDU size (in bytes)
 * @param[in]  framing          Frame mode (@see enum iso_frame)
 * @param[in]  mic_present      Indicate if MIC present in ISO packet
 * @param[in]  am0              Indicate Audio Mode 0
 *
 * @return     status        Error status
 ****************************************************************************************
 */
uint8_t lld_isoal_start(uint8_t act_id, uint8_t direction, uint16_t conhdl,
                        uint32_t sdu_interval, uint32_t trans_latency, int32_t sync_ref_offset, uint16_t max_sdu, uint32_t iso_interval,
                        uint8_t bn, uint8_t max_pdu, uint8_t framing, bool mic_present, bool am0);

/**
 ****************************************************************************************
 * @brief Stop ISOAL Driver for all direction
 *
 * @param[in]  act_id      Stream Activity identifier
 * @param[in]  reason      Reason status code for stopping ISOAL
 ****************************************************************************************
 */
void lld_isoal_stop(uint8_t act_id, uint8_t reason);

/**
 ****************************************************************************************
 * @brief Request to fill a PDU buffer for transmission
 *
 * @param[in]  act_id      Stream Activity identifier
 * @param[in]  iso_buf_idx ISO buffer index
 * @param[in]  ref_anchor  Reference anchor point, time of first possible transmission (in us, based on Bluetooth timestamp)
 * @param[in]  rejected    True if buffer transmission never programmed, False otherwise
 ****************************************************************************************
 */
void lld_isoal_tx_get(uint8_t act_id, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected);

/**
 ****************************************************************************************
 * @brief Indicate reception of an ISO packet
 *
 * @param[in]  act_id      Stream Activity identifier
 * @param[in]  iso_buf_idx ISO buffer index
 * @param[in]  ref_anchor  Reference anchor point, time of first possible transmission (in us, based on Bluetooth timestamp)
 * @param[in]  rejected    True if buffer reception never programmed, False else
 ****************************************************************************************
 */
void lld_isoal_rx_done(uint8_t act_id, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected);

#endif // (BLE_ISO_PRESENT)

#if BLE_PWR_CTRL
    /**
    ****************************************************************************************
    * @brief Get the power level for the Tx Rate
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  tx_rate       Tx rate @see enum lld_rate
    * @param[out] pwr_lvl       Power level
    *
    * @return     status        Error status
    ****************************************************************************************
    */
    uint8_t lld_con_tx_pwr_lvl_get(uint8_t link_id, uint8_t tx_rate, uint8_t *pwr_lvl);

    /**
    ****************************************************************************************
    * @brief Update the RSSI for the Rx Rate
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  rx_rate       Rx rate @see enum lld_rate
    * @param[in]  rssi          RSSI in dBm
    ****************************************************************************************
    */
    void lld_con_rssi_update(uint8_t link_id, uint8_t rx_rate, int8_t rssi);

#endif // BLE_PWR_CTRL

#if (BLE_CH_SCAN_SUPPORT)
    /**
    ****************************************************************************************
    * @brief Initialize channel scan
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lld_ch_scan_init(bool is_reset);
#endif // (BLE_CH_SCAN_SUPPORT)

/// @} LLD
#endif // LLD_INT_H_
