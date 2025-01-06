/**
****************************************************************************************
*
* @file ld.h
*
* @brief LD API
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef LD_H_
#define LD_H_

/**
 ****************************************************************************************
 * @defgroup LD Link Driver
 * @ingroup ROOT
 * @brief LD module handles the real-time job of Bluetooth operations.
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
#include <co_bt.h>          // BT standard definitions
#include "bt_util_buf.h"      // BT buffers management
#include "rwip.h"                 // For rwip_time_t

/*
 * DEFINES
 *****************************************************************************************
 */

// LD ACL timeout before initiating a Synchronisation Scan, ref. TP/XCB/BV-12-C (1 sec)
#define LD_ACL_TO_SYNC_SCAN     0x0C80

// LMP broadcast can LMP_CLK_UNUSED if a specific clk instant is not required for transmission.
#define LD_BCST_LMP_CLK_UNUSED      (0xFFFFFFFF)

// Radiative Transmit Power from CS value (in dBm)
#define LD_TXPWR_DBM_GET(txpwr_idx, mod, type) (rwip_rf.txpwr_dbm_get(txpwr_idx, mod, type) + BT_DFT_TX_PATH_RF_COMPENSATION/10)

// Radiative RSSI from CS value (in dBm)
#define LD_RSSI_CONVERT(rxrssi) (rwip_rf.rssi_convert(rxrssi) - BT_DFT_RX_PATH_RF_COMPENSATION/10)

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/// Encryption mode
enum LD_ENCRYPTION_MODE
{
    LD_ENC_DISABLE, // 00: Don't decrypt Rx/Tx payload / Plain data reception.
    LD_ENC_E0,      // 01: Decrypt Rx/Tx payload using E0 mode.
    LD_ENC_AES,     // 10: Decrypt Rx/Tx payload using AES-CCM mode
};

// Bitmask for use with ld_acl_active_hop_types_get()
#define LD_AFH_HOP_USED 0x01
#define LD_STD_HOP_USED 0x02

/// MWS Tx/Rx/FRQ MSK mode
enum LD_MWS_FRQ_MSK
{
    LD_MWS_NO_IMPACT,           // 00: MWS Frequency has no impact (default mode)
    LD_MWS_CAN_STOP_BT_TX,      // 01: MWS Frequency can stop BT Tx, no impact on BT Rx
    LD_MWS_CAN_STOP_BT_RX,      // 10: MWS Frequency can stop BT Rx, no impact on BT Tx
    LD_MWS_CAN_STOP_BT_TXRX,    // 11: MWS Frequency can stop both BT Tx and BT Rx
};

/// MWS Tx/Rx PRIO mode
enum LD_MWS_PRIO_MODE
{
    LD_MWS_EXCL_PWRUP_DELAY,   // 00: MWS Indication Excluding PowerUp Delay
    LD_MWS_INCL_PWRUP_DELAY,   // 01: MWS Indication Including PowerUp Delay
    LD_MWS_HI_PRIO_IND,        // 10: MWS High Priority Indicator
};


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

struct ld_slave_hop_types
{
    uint8_t std_hop;
    uint8_t afh_hop;
};

/// Inquiry parameters structure
struct ld_inquiry_params
{
    /// LAP to be used for the access code construction (GIAC or DIAC)
    struct lap lap;
    /// Minimum duration between consecutive inquiries in number of 1.28 seconds (0 for non-periodic inquiry)
    uint16_t per_min;
    /// Maximum duration between consecutive inquiries in number of 1.28 seconds (0 for non-periodic inquiry)
    uint16_t per_max;
    /// Length of the inquiry in number of 1.28 seconds (2048 slots)
    uint16_t inq_len;
    /// Max number of responses before halting the inquiry
    uint8_t nb_rsp_max;
    /// Inquiry TX power level (in dBm)
    int8_t tx_pwr_lvl;
    /// Enable/disable EIR response
    bool eir_en;
    /// Ninquiry
    uint16_t n_inq;
    /// Knudge enable
    bool knudge_en;
};

/// Inquiry Scan parameters structure
struct ld_inquiry_scan_params
{
    /// LAP to be used for the access code construction (GIAC or DIAC)
    struct lap lap;
    /// Amount of time between consecutive inquiry scans in slots (625 us)
    uint16_t iscan_intv;
    /// Amount of time for the duration of the inquiry scan in slots (625 us)
    uint16_t iscan_win;
    /// Inquiry Scan type
    uint8_t iscan_type;
    /// Page scan repetition mode
    uint8_t page_scan_rep_mode;
};

/// Page parameters structure
struct ld_page_params
{
    /// BD Address of the paged device
    struct bd_addr bd_addr;
    /// Page Timeout (in slots)
    uint32_t page_to;
    /// Clock Offset (in slots)
    uint16_t clk_off;
    /// Npage
    uint16_t n_page;
    /// Local Page Scan Repetition Mode (R0/R1/R2)
    uint8_t page_scan_rep_mode;
    /// Link identifier
    uint8_t link_id;
    /// LT address
    uint8_t lt_addr;
    /// Truncated page
    bool truncated;
    /// Knudge enable
    bool knudge_en;
};

/// Page Scan parameters structure
struct ld_page_scan_params
{
    /// Amount of time between consecutive page scans in slots (625 us)
    uint16_t pscan_intv;
    /// Amount of time for the duration of the page scan in slots (625 us)
    uint16_t pscan_win;
    /// Page scan type
    uint8_t pscan_type;
    /// Link identifier
    uint8_t link_id;
};

/// Test mode parameters structure (see BT standard specification LMP:4.7.3)
struct ld_acl_test_mode_params
{
    /// Data length
    uint16_t data_length;
    /// Test scenario
    uint8_t  test_scenario;
    /// Hopping mode
    uint8_t  hopping_mode;
    /// Transmit frequency
    uint8_t  tx_freq;
    /// Reception frequency
    uint8_t  rx_freq;
    /// Power control test
    uint8_t  power_control;
    /// Packet type
    uint8_t  packet_type;
};

/// Slave timing information structure
/*@TRACE*/
struct ld_slave_timing_info
{
    /// Value of CLKN when the last sync was detected  (in BT half-slots)
    uint32_t last_sync_ts;
    /// Clock offset (in BT half-slots)
    uint32_t last_sync_clk_off;
    /// Current bit offset (in half-us)
    int16_t last_sync_bit_off;
};

#if MAX_NB_SYNC
/// Audio configuration parameters
struct ld_sco_audio_settings
{
    uint8_t in_coding;
    uint8_t air_coding;
    uint8_t data_path;
    uint8_t in_data_format;
    uint8_t in_sample_size;
    uint8_t msb_position;
};

/// SCO parameters structure
struct ld_sco_params
{
    /// Audio settings
    struct ld_sco_audio_settings audio;
    /// RX packet length (in bytes)
    uint16_t rx_pkt_len;
    /// TX packet length (in bytes)
    uint16_t tx_pkt_len;
    /// eSCO LT address
    uint8_t lt_addr;
    /// Link type (0: SCO | 2: ESCO)
    uint8_t sync_type;
    /// RX packet type (as defined in BT specification LMP:5.2) (HV2 not supported)
    uint8_t rx_pkt_type;
    /// TX packet type (as defined in BT specification LMP:5.2) (HV2 not supported)
    uint8_t tx_pkt_type;
    /// Synchronous link offset (in slots)
    uint8_t d_esco;
    /// Synchronous link interval (in slots)
    uint8_t t_esco;
    /// Retransmission window size (in slots) (eSCO only)
    uint8_t w_esco;
    /// Initialization method (0: init 1 / 1: init 2), from timing control flags
    uint8_t init;
};



#if (EAVESDROPPING_SUPPORT)

/// Eavesdropping AFH settings
struct ld_ed_afh_tag
{
    /// AFH enabled (DISABLED = 0x00, ENABLED = 0x01)
    uint8_t            enabled;
    /// AFH time (BT master clk value at the time of switch seq, bits 27:1, shall be even)
    uint32_t           time;
    /// AFH channel map (10 bytes, 79 usefull bits for every channel)
    struct bt_ch_map   ch_map;
};

/// Eavesdropping encryption settings
struct ld_ed_encryption_tag
{
    /// Encryption type (ENC_TYPE_NONE = 0x00, ENC_TY0E_E0 = 0x01, ENC_TYPE_AES = 0x02)
    uint8_t                       type;
    /// Encryption key (Long term key, 16 bytes)
    struct ltk                     key;
    /// Encryption key size (1..16)
    uint8_t                        key_size;
    /// Encryption initialization vector (8 bytes)
    struct initialization_vector   iv;
};

/// Eavesdropping clock offset settings
struct ld_ed_clock_tag
{
    /// Bluetooth clock offset value in slots
    uint32_t            clk_offset;
    /// Bluetooth fine timer counter offset value in us
    uint16_t            bit_offset;
};

/// ED SCO parameters structure
struct ld_ed_sco_params
{
    /// Audio settings
    struct ld_sco_audio_settings audio;
    /// RX packet length (in bytes)
    uint16_t rx_pkt_len;
    /// TX packet length (in bytes)
    uint16_t tx_pkt_len;
    /// eSCO LT address
    uint8_t lt_addr;
    /// Link type (0: SCO | 2: ESCO)
    uint8_t sync_type;
    /// RX packet type (as defined in BT specification LMP:5.2) (HV2 not supported)
    uint8_t rx_pkt_type;
    /// TX packet type (as defined in BT specification LMP:5.2) (HV2 not supported)
    uint8_t tx_pkt_type;
    /// Synchronous link offset (in slots)
    uint8_t d_esco;
    /// Synchronous link interval (in slots)
    uint8_t t_esco;
    /// Retransmission window size (in slots) (eSCO only)
    uint8_t w_esco;
    /// Initialization method (0: init 1 / 1: init 2), from timing control flags
    uint8_t init;
};

/// LD ED ACL params
struct ld_ed_acl_params
{
    /// Peer BD Address
    struct bd_addr               bd_addr;
    /// Role
    uint8_t                      role;
    /// LT Address
    uint8_t                      lt_addr;

    /// Connection type
    uint8_t                      con_type;

    /// Eavesdropping AFH settings
    struct ld_ed_afh_tag         afh;
    /// Eavesdropping encryption settings
    struct ld_ed_encryption_tag  encryption;
    /// Eavesdropping clock offset settings
    struct ld_ed_clock_tag       clock;
    /// Max slot
    uint8_t                      max_slot;
    /// Eavesdropping packet type table
    uint8_t                      ptt;
    /// Min max power level
    uint8_t                      level;

    struct ld_ed_sco_params      sco;
};

/// LD Segmented scan parameters
struct ld_seg_scan_params
{
    /// Time offset defining the first scan window position, Clock % Interval = Offset (in 312.5us half-slots)
    uint16_t time_offset;

    /// Periodicity of scan segments (in 312.5us half-slots)
    uint16_t periodicity;

    /// Distance between scan segments (in 312.5us half-slots)
    uint16_t distance;

    /// Duration of each scan segment (in half-us)
    uint16_t duration;

    /// Number of segments in a scan window (maximum 16 segments)
    uint8_t segments;

    /// Table of phase offsets for each scan segment (shall contain one offset per segment, maximum 16)
    uint8_t phase_offsets[16];

    /// BD address of a master, if an active slave connection is present, the scan timings are based on this piconet
    struct bd_addr bd_addr;

    /// Half-microsecond offset
    int16_t hus_offset;
};

#endif // EAVESDROPPING_SUPPORT

#endif //MAX_NB_SYNC

#if CSB_SUPPORT
/// LD CSB enable information structure
struct ld_csb_en_params
{
    /// lt_addr reserved for CSB
    uint8_t lt_addr;
    /// packet types supported
    uint16_t packet_types;
    /// max slot packet types supported
    uint8_t max_slots;
    /// CSB interval (in half-slots)
    uint16_t csb_interval;
    /// CSB offset (in half-slots)
    uint16_t csb_offset;
    /// CSB supervision timeout
    uint16_t csb_supv_to;
    /// AFH_Channel_Map
    struct bt_ch_map *afh_ch_map;
};

/// CSB RX parameters structure
struct ld_csb_rx_params
{
    /// BD_ADDR
    struct bd_addr bd_addr;
    /// LT_ADDR
    uint8_t lt_addr;
    /// Interval (in slots)
    uint16_t interval;
    /// Clock_Offset (28 bits) - (CLKNslave – CLK) modulo 2^28
    uint32_t clock_offset;
    /// Next_Connectionless_Slave_Broadcast_Clock (28 bits)
    uint32_t next_csb_clock;
    /// CSB_supervisionTO (in slots)
    uint16_t csb_supv_to;
    /// Remote_Timing_Accuracy (in ppm)
    uint8_t remote_timing_accuracy;
    /// Skip
    uint8_t skip;
    /// AFH_Channel_Map
    struct bt_ch_map afh_ch_map;
    /// Enhanced data rate
    bool edr;
};
#endif // CSB_SUPPORT

#if (RW_BT_MWS_COEX)
/// MWS signalling offsets (configs read)
struct ld_mws_signaling_rd_offsets
{
    ///Rx_Priority_Assert_Offset
    int16_t rx_prio_assert_offset;
    ///Rx_Priority_Assert_Jitter
    uint16_t rx_prio_assert_jitter;
    ///Rx_Priority_Deassert_Offset
    int16_t rx_prio_deassert_offset;
    ///Rx_Priority_Deassert_Jitter
    uint16_t rx_prio_deassert_jitter;
    ///Tx_On_Assert_Offset
    int16_t tx_on_assert_offset;
    ///Tx_On_Assert_Jitter
    uint16_t tx_on_assert_jitter;
    ///Tx_On_Deassert_Offset
    int16_t tx_on_deassert_offset;
    ///Tx_On_Deassert_Jitter
    uint16_t tx_on_deassert_jitter;
};

/// MWS signalling offsets (configs written)
struct ld_mws_signaling_wr_offsets
{
    ///Rx_Assert_Offset
    int16_t rx_assert_offset;
    ///Rx_Assert_Jitter
    uint16_t rx_assert_jitter;
    ///Rx_Deassert_Offset
    int16_t rx_deassert_offset;
    ///Rx_Deassert_Jitter
    uint16_t rx_deassert_jitter;
    ///Tx_Assert_Offset
    int16_t tx_assert_offset;
    ///Tx_Assert_Jitter
    uint16_t tx_assert_jitter;
    ///Tx_Deassert_Offset
    int16_t tx_deassert_offset;
    ///Tx_Deassert_Jitter
    uint16_t tx_deassert_jitter;
    ///Pattern_Assert_Offset
    int16_t pattern_assert_offset;
    ///Pattern_Assert_Jitter
    uint16_t pattern_assert_jitter;
    ///Inactivity_Duration_Assert_Offset
    int16_t inactivity_duration_assert_offset;
    ///Inactivity_Duration_Assert_Jitter
    uint16_t inactivity_duration_assert_jitter;
    ///Scan_Frequency_Assert_Offset
    int16_t scan_frequency_assert_offset;
    ///Scan_Frequency_Assert_Jitter
    uint16_t scan_frequency_assert_jitter;
    ///Priority_Assert_Offset_Request
    uint16_t priority_assert_offset_request;
};

#endif // RW_BT_MWS_COEX

#if (BT_HCI_TEST_MODE)
/// Test mode parameters structure
struct ld_test_params
{
    /// Type (0: RX | 1: TX)
    uint8_t type;

    /// RF channel, N = (F - 2402) / 2
    uint8_t channel;

    /// Length of test data
    uint16_t data_len;

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

    /**
     * Packet_type
     */
    uint8_t pkt_type;

    /// Transmit power level in dBm (0x7E: minimum | 0x7F: maximum | range: -127 to +20)
    int8_t  tx_pwr_lvl;
};
#endif //(BT_HCI_TEST_MODE)

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the LD module
 ****************************************************************************************
 */
void ld_init(void);

/**
 ****************************************************************************************
 * @brief Reset the LD module
 ****************************************************************************************
 */
void ld_reset(void);

/**
 * ***************************************************************************************
 * @brief Read BT clock
 *
 * @return Clock value (in half-slots of 312.5us)
 ****************************************************************************************
 */
uint32_t ld_read_clock(void);

/**
 ****************************************************************************************
 * @brief Get the local BD address
 *
 * @param[out]  bd_addr   Pointer to BD Address
 ****************************************************************************************
 */

void ld_bd_addr_get(struct bd_addr *bd_addr);

/**
 ****************************************************************************************
 * @brief Get the local Class Of Device
 *
 * @param[out]  class_of_dev   Pointer to Class Of Device
 ****************************************************************************************
 */

void ld_class_of_dev_get(struct devclass *class_of_dev);

/**
 ****************************************************************************************
 * @brief Set the local Class Of Device
 *
 * @param[in]  class_of_dev   Pointer to Class Of Device
 ****************************************************************************************
 */

void ld_class_of_dev_set(struct devclass *class_of_dev);

/**
 ****************************************************************************************
 * @brief Inform that a RX buffer has been freed and is ready to be used by an RX Descriptor
 *
 * @note This function must be executed into an interrupt lock context.
 *
 * @param[in] em_ptr     Address of the buffer released
 *
 * @return True is buffer has been assigned to a RX descriptor, False otherwise
 ****************************************************************************************
 */
bool ld_rxdesc_buf_ready(uint16_t em_ptr);

/**
 ****************************************************************************************
 * @brief Get active hop types
 *
 * @return  Bitflag of active hop types
 ****************************************************************************************
 */
uint8_t ld_acl_active_hop_types_get(void);

/**
 ****************************************************************************************
 * @brief Start Inquiry
 *
 * @param[in]  params        Pointer to Inquiry parameters structure
 *
 * @return status            0: success | 1-255: error (BT error code)
 ****************************************************************************************
 */
uint8_t ld_inq_start(struct ld_inquiry_params *params);

/**
 ****************************************************************************************
 * @brief Stop Inquiry
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_inq_stop(void);

/**
 ****************************************************************************************
 * @brief Start Inquiry Scan
 *
 * @param[in]  params        Pointer to Inquiry Scan parameters structure
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_iscan_start(struct ld_inquiry_scan_params *params);

/**
 ****************************************************************************************
 * @brief Stop Inquiry Scan
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_iscan_stop(void);

/**
 ****************************************************************************************
 * @brief Stores the Extended Inquiry response given by upper layers
 *
 * @param[in] fec_req     Host can require to send data using a DMx packet
 * @param[in] eir_ptr     EIR structure pointer
 *
 * @return status
 ****************************************************************************************
 */
uint8_t ld_iscan_eir_set(uint8_t fec_req, const struct eir *eir_ptr);

/**
 ****************************************************************************************
 * @brief Gives the Extended Inquiry response given by upper layers.
 *
 * @param[out] fec_req         Host can require to send data using a DMx packet
 * @param[out] eir_ptr         EIR structure pointer
 *
 * @return status
 ****************************************************************************************
 */
uint8_t ld_iscan_eir_get(uint8_t *fec_req, struct eir *eir_ptr);

/**
 ****************************************************************************************
 * @brief Start Page
 *
 * @param[in]  params        Pointer to Page parameters structure
 *
 * @return status            0: success | 1-255: error (BT error code)
 ****************************************************************************************
 */
uint8_t ld_page_start(struct ld_page_params *params);

/**
 ****************************************************************************************
 * @brief Stop Page
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_page_stop(void);

/**
 ****************************************************************************************
 * @brief Start Page Scan
 *
 * @param[in]  params        Pointer to Page Scan parameters structure
 *
 * @return status            0: success | 1-255: error (BT error code)
 ****************************************************************************************
 */
uint8_t ld_pscan_start(struct ld_page_scan_params *params);

/**
 ****************************************************************************************
 * @brief Stop Page Scan
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_pscan_stop(void);

/**
 ****************************************************************************************
 * @brief Start ACL link
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  role          0: Master | 1: Slave
 * @param[in]  slave_timing_info_ptr  Pointer to slave timing information
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_start(uint8_t link_id, uint8_t role, struct ld_slave_timing_info *slave_timing_info_ptr);

/**
 ****************************************************************************************
 * @brief Stop ACL link
 *
 * @param[in]  link_id       Link identifier
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_stop(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Disable ACL flow
 *
 * @param[in]  link_id       Link identifier
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_flow_off(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Enable ACL flow
 *
 * @param[in]  link_id       Link identifier
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_flow_on(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Send an ACL packet
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  buf_elt       Buffer element of the data to send
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_data_tx(uint8_t link_id, struct bt_em_acl_buf_elt *buf_elt);

/**
 ****************************************************************************************
 * @brief Flush ACL data
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  nb_flushed    Number of packets flushed
 * @param[in]  all           True: flush all packets | False: Non flushable packets are kept
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_data_flush(uint8_t link_id, uint8_t *nb_flushed, bool all);

/**
 ****************************************************************************************
 * @brief Send an LMP packet
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  buf_elt       Buffer element of the LMP to send
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_lmp_tx(uint8_t link_id, struct bt_em_lmp_buf_elt *buf_elt);

/**
 ****************************************************************************************
 * @brief Request for a master/slave switch operation.
 *
 * @param[in]  link_id            Link identifier
 * @param[in]  lt_addr            LT to use on the new piconet (for slave->master switch only)
 * @param[in]  slot_offset        Slot offset (for master->slave switch only)
 * @param[in]  instant            Switch instant
 * @param[in]  page_scan_rep_mode Local page scan repetition mode to be indicated in FHS packet (slave->master switch only)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_rsw_req(uint8_t link_id, uint8_t lt_addr, uint16_t slot_offset,  uint32_t instant, uint8_t page_scan_rep_mode);

/**
 ****************************************************************************************
 * @brief Returns Flush Timeout value
 *
 * @param[in]  link_id       Link identifier
 *
 * @return Flush Timeout value
 ****************************************************************************************
 */
uint16_t ld_acl_flush_timeout_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set Flush Timeout value
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  flush_to      FlushTimeout value to be set
 ****************************************************************************************
 */
void ld_acl_flush_timeout_set(uint8_t link_id, uint16_t flush_to);

/**
 ****************************************************************************************
 * @brief Get Tpoll
 *
 * @param[in]  link_id       Link identifier
 *
 * @return Tpoll (in slots)
 ****************************************************************************************
 */
uint16_t ld_acl_t_poll_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set Tpoll
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  t_poll        Tpoll (in slots)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
void ld_acl_t_poll_set(uint8_t link_id, uint16_t t_poll);

/**
 ****************************************************************************************
 * @brief Enter sniff transition mode (master only)
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  offset        Sniff offset (in 625 us slots) (valid only when moving from active to sniff)
 * @param[in]  intv          Sniff interval (in 625 us slots) (valid only when moving from active to sniff)
 * @param[in]  att           Sniff attempts (in 625 us slots) (valid only when moving from active to sniff)
 * @param[in]  to            Sniff timeout (in 625 us slots) (valid only when moving from active to sniff)
 * @param[in]  init          Initialization method (0: init 1 / 1: init 2), from timing control flags (valid only when moving from active to sniff)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_sniff_trans(uint8_t link_id, uint16_t offset, uint16_t intv, uint16_t att, uint16_t to, uint8_t init);

/**
 ****************************************************************************************
 * @brief Enter sniff low power mode
 *
 * Note: Sniff parameters must be provided when moving from normal to sniff transition mode.
 * When moving from sniff to sniff transition mode, the sniff parameters are ignored,
 * the sniff transition mode continues using current sniff scheduling information.
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  offset        Sniff offset (in 625 us slots)
 * @param[in]  intv          Sniff interval (in 625 us slots)
 * @param[in]  att           Sniff attempts (in 625 us slots)
 * @param[in]  to            Sniff timeout (in 625 us slots)
 * @param[in]  init          Initialization method (0: init 1 / 1: init 2), from timing control flags
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_sniff(uint8_t link_id, uint16_t offset, uint16_t intv, uint16_t att, uint16_t to, uint8_t init);

/**
 ****************************************************************************************
 * @brief Exit sniff low power mode
 *
 * @param[in]  link_id       Link identifier
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_unsniff(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set Sniff Sub-Rating parameters
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  sub_rate      Sniff Sub-rate
 * @param[in]  to            Minimum Sniff Time Out (to come back to sniff sub-rating)
 * @param[in]  instant       Sniff sub-rating instant
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_ssr_set(uint8_t link_id, uint8_t sub_rate, uint16_t to, uint32_t instant);

/**
 ****************************************************************************************
 * @brief Set encryption mode in TX direction
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  mode          00: disabled | 01: E0 | 10: AES
 ****************************************************************************************
 */
void ld_acl_tx_enc(uint8_t link_id, uint8_t mode);

/**
 ****************************************************************************************
 * @brief Set encryption mode in RX direction
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  mode          00: disabled | 01: E0 | 10: AES
 ****************************************************************************************
 */
void ld_acl_rx_enc(uint8_t link_id, uint8_t mode);

#if BCAST_ENC_SUPPORT
    /**
    ****************************************************************************************
    * @brief Enable/disable decryption of RX broadcast payload
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  mode          0: disabled | 1: enabled
    ****************************************************************************************
    */
    void ld_acl_bcst_rx_dec(uint8_t link_id, uint8_t mode);
#endif //BCAST_ENC_SUPPORT

/**
 ****************************************************************************************
 * @brief Load encryption Key
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  key           Encryption key
 * @param[in]  iv            Initialization Vector (for AES encryption only)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
void ld_acl_enc_key_load(uint8_t link_id, struct ltk *key, struct initialization_vector *iv);

/**
 ****************************************************************************************
 * @brief Get current clock offset between piconet clock and local clock
 *
 * A is piconet clock in half-slots.
 * B is local clock in half-slots
 * The resulting clock offset is (A-B) modulo 2^28
 *
 * @param[in]  link_id        Link Identifier
 *
 * @return piocnet offset (@see rwip_time_t)
 ****************************************************************************************
 */
rwip_time_t ld_acl_clock_offset_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Get local slot offset for role switch procedure
 *
 * @param[in]  link_id       Link identifier
 *
 * @return Current slot offset (in us from 0 to 1249 us)
 ****************************************************************************************
 */
uint16_t ld_acl_rsw_slot_offset_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set peer's slot offset for role switch proceudre
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  slot_offset   Current slot offset (in us from 0 to 1249 us)
 ****************************************************************************************
 */
void ld_acl_rsw_slot_offset_set(uint8_t link_id, uint16_t slot_offset);

/**
 ****************************************************************************************
 * @brief Get link supervision timeout
 *
 * @param[in]  link_id       Link identifier
 *
 * @return Link supervision timeout (in slots)
 ****************************************************************************************
 */
uint16_t ld_acl_lsto_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set link supervision timeout
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  lsto          Link supervision timeout (in slots)
 ****************************************************************************************
 */
void ld_acl_lsto_set(uint8_t link_id, uint16_t lsto);

/**
 ****************************************************************************************
 * @brief Set peer's Drift and Jitter
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  drift         Low power clock drift (in ppm)
 * @param[in]  jitter        Low power clock jitter (in us)
 ****************************************************************************************
 */
void ld_acl_timing_accuracy_set(uint8_t link_id, uint8_t drift, uint8_t jitter);

/**
 ****************************************************************************************
 * @brief Enable/disable Enhanced Data Rate (2/3 Mbps)
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  en            True: enable / False: disable
 ****************************************************************************************
 */
void ld_acl_edr_set(uint8_t link_id, bool en);

/**
 ****************************************************************************************
 * @brief Set allowed TX packet types
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  packet_types  Allowed packet types (bit field as defined in BT standard HCI:7.1.5)
 ****************************************************************************************
 */
void ld_acl_allowed_tx_packet_types_set(uint8_t link_id, uint16_t packet_types);

/**
 ****************************************************************************************
 * @brief Disable/enable DM1 packet type for ACL-U traffic
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  disable       Disable DM1 packet type
 ****************************************************************************************
 */
void ld_acl_dm1_packet_type_dis(uint8_t link_id, bool disable);

/**
 ****************************************************************************************
 * @brief Enable/disable eSCO loopback mode
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  enable        Enable eSCO loopback mode
 ****************************************************************************************
 */
void ld_esco_loopback_mode_en(uint8_t link_id, bool enable);

/**
 ****************************************************************************************
 * @brief Gets current TX power
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  modulation    Modulation (1:GFSK | 2:DQPSK | 3:8DPSK)
 *
 * @return Current TX power (in dBm from -100 to 20 dBm)
 ****************************************************************************************
 */
uint8_t ld_acl_current_tx_power_get(uint8_t link_id, uint8_t mod);

#if PCA_SUPPORT
    /**
    ****************************************************************************************
    * @brief Update clock - relative adjustment (slave link)
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  clk_off       Clock offset adjustment (in BT slots)
    * @param[in]  bit_off       Bit offset adjustment (in us)
    * @param[in]  clk_adj_instant Instant (in BT slots)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_acl_clk_adj_set(uint8_t link_id, uint32_t clk_adj_slots, int16_t clk_adj_us, uint32_t clk_adj_instant);

    /**
    ****************************************************************************************
    * @brief Updates clock (slave link)
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  clk_off       Clock offset value (in BT slots)
    * @param[in]  bit_off       Bit offset value (in us)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_acl_clk_set(uint8_t link_id, uint32_t clk_off, int16_t bit_off);

#endif // PCA_SUPPORT

#if (EAVESDROPPING_SUPPORT || PCA_SUPPORT)

    /**
    ****************************************************************************************
    * @brief Read role
    *
    * @param[in]  link_id       Link identifier
    *
    * @return role              role  - MASTER_ROLE, SLAVE_ROLE, or UNKNOWN
    ****************************************************************************************
    */
    uint8_t ld_acl_role_get(uint8_t link_id);

#endif // (0 || PCA_SUPPORT)

/**
 ****************************************************************************************
 * @brief Enable/Disable AFH (slave link)
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  hssi          Hopping Scheme Switch instant
 * @param[in]  en            True: enable / False: disable
 * @param[in]  ch_map        Pointer to channel map
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_afh_set(uint8_t link_id, uint32_t hssi, bool en, const struct bt_ch_map *ch_map);

/**
 ****************************************************************************************
 * @brief Prepare an AFH update (master link)
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  hssi          Hopping Scheme Switch instant
 * @param[in]  en            True: enable / False: disable
 * @param[in]  ch_map        Pointer to channel map
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_afh_prepare(uint8_t link_id, uint32_t hssi, bool en, const struct bt_ch_map *ch_map);

/**
 ****************************************************************************************
 * @brief Confirm a pending AFH update (master link)
 *
 * @param[in]  link_id       Link identifier
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_acl_afh_confirm(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Get maximum number of slots in RX
 *
 * @param[in]  link_id       Link identifier
 * @return     max_slot      Maximum number of slots peer device is allowed to use
 ****************************************************************************************
 */
uint8_t ld_acl_rx_max_slot_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set maximum number of slots in RX
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  max_slot      Maximum number of slots peer device is allowed to use
 ****************************************************************************************
 */
void ld_acl_rx_max_slot_set(uint8_t link_id, uint8_t max_slot);

/**
 ****************************************************************************************
 * @brief Enter/exit the device from test mode
 *
 * @param[in]  link_id     Link identifier
 * @param[in]  params      Pointer to test mode parameters
 ****************************************************************************************
 */
void ld_acl_test_mode_set(uint8_t link_id, struct ld_acl_test_mode_params *params);

/**
 ****************************************************************************************
 * @brief Get the delta between received RSSI and RF golden range
 *
 * @param[in]  link_id       Link identifier
 * @return     rssi delta    in dBm => 0: value in range | >0: above high threshold | <0: below low threshold
 ****************************************************************************************
 */
int8_t ld_acl_rssi_delta_get(uint8_t link_id);

#if MAX_NB_SYNC
    /**
    ****************************************************************************************
    * @brief Start SCO
    *
    * @param[in]  link_id          Link identifier
    * @param[in]  sco_link_id      SCO link identifier
    * @param[in]  params           Pointer to SCO parameters structure
    * @param[in]  remote_loopback  Indicates whether Remote Loopback Mode is enabled (true) or not (false)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_sco_start(uint8_t link_id, uint8_t sco_link_id, struct ld_sco_params *params, bool remote_loopback);

    /**
    ****************************************************************************************
    * @brief Update SCO
    *
    * @param[in]  sco_link_id   SCO link identifier
    * @param[in]  params        Pointer to SCO parameters structure
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_sco_update(uint8_t sco_link_id, struct ld_sco_params *params);

    /**
    ****************************************************************************************
    * @brief Stop SCO
    *
    * @param[in]  sco_link_id   SCO link identifier
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_sco_stop(uint8_t sco_link_id);

    #if VOICE_OVER_HCI
        /**
        ****************************************************************************************
        * @brief Handle BT audio interrupt
        *
        * @param[in]  sco_link_id   SCO link identifier
        ****************************************************************************************
        */
        void ld_sco_audio_isr(uint8_t sco_link_id);

        /**
        ****************************************************************************************
        * @brief Send a synchronous data packet
        *
        * @param[in]  sco_link_id   SCO link identifier
        * @param[in]  buf_elt       Buffer element of the data to send
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t ld_sco_data_tx(uint8_t sco_link_id, struct bt_em_sync_buf_elt *buf_elt);
    #endif // VOICE_OVER_HCI

    #if EAVESDROPPING_SUPPORT
        /**
        ****************************************************************************************
        * @brief set LRD for eSCO
        *
        * @param[in]  link_id          Link identifier
        * @param[in]  en               True: LRD enabled, False: Disabled
        * @param[in]  NULL filtering   True: enabled, False: Disabled
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */

        uint8_t ld_sco_lrd_set(uint8_t link_id, bool en, bool null_flt);
        /**
        ****************************************************************************************
        * @brief set ED LRD for eSCO
        *
        * @param[in]  en               True: LRD enabled, False: Disabled
        * @param[in]  NULL filtering   True: enabled, False: Disabled
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t ld_ed_sco_lrd_set(bool en, bool null_flt);

    #endif // EAVESDROPPING_SUPPORT

#endif // MAX_NB_SYNC

/**
 ****************************************************************************************
 * @brief Send a broadcast ACL packet
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  buf_elt       Buffer element of the data to send
 * @param[in]  afh_ch_map    AFH channel map
 * @param[in]  nbc           Number of broadcast retransmissions
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_bcst_acl_data_tx(uint8_t link_id, struct bt_em_acl_buf_elt *buf_elt, struct bt_ch_map *afh_ch_map, uint8_t nbc);

/**
 ****************************************************************************************
 * @brief Send a broadcast LMP packet
 *
 * @param[in]  buf_elt       Buffer element of the LMP to send
 * @param[in]  afh_ch_map    AFH channel map
 * @param[in]  clock         Instant clock for tranmit (in half-slots) or LD_BCST_LMP_CLK_UNUSED
 * @param[in]  nbc           Number of broadcast retransmissions
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_bcst_lmp_tx(struct bt_em_lmp_buf_elt *buf_elt, struct bt_ch_map *afh_ch_map,  uint32_t clock, uint8_t nbc);

/**
****************************************************************************************
* @brief Cancel pending broadcast LMP packet
*
****************************************************************************************
*/
void ld_bcst_lmp_cancel(void);

/**
 ****************************************************************************************
 * @brief Update AFH map for active broadcast
 *
 * @param[in]  afh_ch_map    Pointer to new channel map
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_bcst_afh_update(struct bt_ch_map *afh_ch_map);

#if BCAST_ENC_SUPPORT
    /**
    ****************************************************************************************
    * @brief Set encryption mode in TX direction
    *
    * @param[in]  mode          00: disabled | 01: E0 | 10: AES
    ****************************************************************************************
    */
    void ld_bcst_tx_enc(uint8_t mode);

    /**
    ****************************************************************************************
    * @brief Load encryption Key
    *
    * @param[in]  key           Encryption key
    ****************************************************************************************
    */
    void ld_bcst_enc_key_load(struct ltk *key);
#endif // BCAST_ENC_SUPPORT

#if PCA_SUPPORT

    /**
    ****************************************************************************************
    * @brief configuration of local PCA target offset/period
    * @param[in]  downlink_us_offset       frame_sync to downlink offset in us
    * @param[in]  uplink_us_offset         frame_sync to uplink offset in us
    * @param[in]  period_duration          external frame slot period in us
    * @param[in]  jitter                   MWS_Frame_Sync_Assert_Jitter in us
    * @param[in]  nb_acl                   number of master/slave links active
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    void ld_pca_local_config(uint16_t downlink_us_offset, uint16_t uplink_us_offset, uint16_t period_duration, uint16_t jitter, uint8_t nb_acl);

    /**
    ****************************************************************************************
    * @brief enabe/disable clock adjust reporting/requests
    ****************************************************************************************
    */
    void ld_pca_reporting_enable(bool enable);

    /**
    ****************************************************************************************
    * @brief Reconfigure target offset for MWS uplink/downlink based on new role
    *
    * @param[in]  link_id       Link identifier
    ****************************************************************************************
    */
    void ld_pca_update_target_offset(uint8_t link_id);

    /**
    ****************************************************************************************
    * @brief Handle PCA corase clock adjustment directive
    *
    * @param[in]  clk_adj_us         clock adjust in us
    * @param[in]  clk_adj_slots      clock adjust slots
    * @param[in]  clk_adj_instant    clock adjust instant (in 625us BT slots)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_pca_coarse_clock_adjust(int16_t clk_adj_us, uint8_t clk_adj_slots, uint32_t clk_adj_instant);

    /**
    ****************************************************************************************
    * @brief Handle PCA clock dragging adjustment directive
    *
    * @param[in]  clk_adj_us         clock adjust in us
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_pca_initiate_clock_dragging(int16_t clk_adj_us);

    /**
    ****************************************************************************************
    * @brief Handle PCA MWS frame sync interrupt
    ****************************************************************************************
    */
    void ld_pca_mws_frame_sync(void);

    /**
    ****************************************************************************************
    * @brief Returns timestamp of the start of the next expected  MWS frame
    ****************************************************************************************
    */
    rwip_time_t ld_pca_ext_frame_ts_get(void);

    /**
    ****************************************************************************************
    * @brief Handle PCA MWS momentary offset 'greater than correction step' interrupt
    ****************************************************************************************
    */
    void ld_pca_mws_moment_offset_gt(void);

    /**
    ****************************************************************************************
    * @brief Handle PCA MWS momentary offset 'less than correction step' interrupt
    ****************************************************************************************
    */
    void ld_pca_mws_moment_offset_lt(void);

#endif // PCA_SUPPORT

#if CSB_SUPPORT || PCA_SUPPORT


    /**
    ****************************************************************************************
    * @brief Start synchronization scan
    *
    * @param[in]  bd_addr         BD Address of the master device
    * @param[in]  sscan_to        Synchronization scan timeout (in slots)
    * @param[in]  sscan_intv      Synchronization scan interval (in slots)
    * @param[in]  sscan_win       Synchronization scan window (in slots)
    * @param[in]  coarse_clk_adj  Started for coarse clock adjustment
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_sscan_start(struct bd_addr *bd_addr, uint32_t sscan_to, uint16_t sscan_intv, uint16_t sscan_win);

    /**
    ****************************************************************************************
    * @brief Stop synchronization scan
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_sscan_stop(void);

    /**
    ****************************************************************************************
    * @brief Start synchronization train
    *
    * @param[in]  strain_to       Synchronization train timeout (in slots)
    * @param[in]  strain_intv     Synchronization train interval (in slots)
    * @param[in]  coarse_clk_adj  Started for coarse clock adjustment
    * @param[in]  ch_map          AFH channel map
    * @param[in]  csb_offset      Connectionless slave broadcast offset (in half-slots)
    * @param[in]  csb_intv        Connectionless slave broadcast interval (in half-slots)
    * @param[in]  csb_lt_addr     Connectionless slave broadcast LT address
    * @param[in]  svc_data        Defined by the service using the Connectionless Slave Broadcast feature
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_strain_start(uint32_t strain_to, uint32_t strain_intv, bool coarse_clk_adj, struct bt_ch_map *ch_map, uint16_t csb_offset, uint16_t csb_intv, uint8_t csb_lt_addr, uint8_t svc_data);

    /**
    ****************************************************************************************
    * @brief Stop synchronization train
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_strain_stop(void);

#endif // CSB_SUPPORT || PCA_SUPPORT

#if CSB_SUPPORT

    /**
    ****************************************************************************************
    * @brief Enables connectionless slave broadcast.
    * param[in] lt_addr;
    * param[in] packet types supported
    * param[in] min CSB interval requested
    * param[in] max CSB interval requested
    * param[in] CSB supervision timeout
    *
    ****************************************************************************************
    */
    uint8_t ld_csb_tx_en(struct ld_csb_en_params *params);

    /**
    ****************************************************************************************
    * @brief Disables connectionless slave broadcast.
    ****************************************************************************************
    */
    void ld_csb_tx_dis(void);

    /**
    ****************************************************************************************
    * @brief Sets transmit data for connectionless slave broadcast.
    * @param[in]  buf_elt      Data buffer, length of data buffer
    ****************************************************************************************
    */
    void ld_csb_tx_set_data(struct bt_em_acl_buf_elt *buf_elt);

    /**
    ****************************************************************************************
    * @brief Clears transmit data for connectionless slave broadcast.
    ****************************************************************************************
    */
    void ld_csb_tx_clr_data(void);

    /**
    ****************************************************************************************
    * @brief Update AFH map for CSB TX
    *
    * @param[in]  afh_ch_map    Pointer to new channel map
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_csb_tx_afh_update(struct bt_ch_map *afh_ch_map);

    /**
    ****************************************************************************************
    * @brief Start CSB RX
    *
    * @param[in]  params        Pointer to CSB RX parameters structure
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_csb_rx_start(struct ld_csb_rx_params *params);

    /**
    ****************************************************************************************
    * @brief Update AFH map for CSB RX
    *
    * @param[in]  afh_ch_map    Pointer to new channel map
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_csb_rx_afh_update(struct bt_ch_map *afh_ch_map);

    /**
    ****************************************************************************************
    * @brief Stop CSB RX
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_csb_rx_stop(void);
#endif // CSB_SUPPORT

#if (RW_BT_MWS_COEX)

/**
 ****************************************************************************************
 * @brief Set MWS Channel Parameters
 *
 * @param[in]  mws_channel_enable           MWS Channel Enable/Disable
 * @param[in]  mws_rx_center_frequency      MWS RX Center Frequency
 * @param[in]  mws_tx_center_frequency      MWS TX Center Frequency
 * @param[in]  mws_rx_channel_bandwidth     MWS RX Channel Bandwidth
 * @param[in]  mws_tx_channel_bandwidth     MWS TX Channel Bandwidth
 * @param[in]  mws_channel_type             MWS Channel Type
 *
 * @return status            0: success | 1-255: error
 *  ****************************************************************************************
 */
uint8_t ld_mws_channel_params_set(uint8_t mws_channel_enable, uint16_t mws_rx_center_frequency, uint16_t mws_tx_center_frequency,
                                  uint16_t mws_rx_channel_bandwidth, uint16_t mws_tx_channel_bandwidth, uint8_t mws_channel_type);

/**
 ****************************************************************************************
 * @brief Set MWS Frame Dimensioning & Sync Offset
 *
 * @param[in]  ext_fr_duration              External Frame Duration
 * @param[in]  ext_fr_sync_assert_offset    External Frame Sync Assert Offset
 * @param[in]  ext_fr_scan_duration         Scannable Duration
 *
 * @return status            0: success | 1-255: error
 *  ****************************************************************************************
 */
uint8_t ld_ext_frame_info_set(uint16_t ext_fr_duration, int16_t ext_fr_sync_assert_offset, uint16_t ext_fr_scan_duration);

/**
 ****************************************************************************************
 * @brief Set MWS Signaling
 *
 * @param[in]  wr_sigoff              External Frame Configurations (Written)
 * @param[in]  rd_sigoff              External Frame Configurations (Read)
 *
 * @return status            0: success | 1-255: error
 *  ****************************************************************************************
 */
uint8_t ld_mws_signaling_offsets_set(struct ld_mws_signaling_wr_offsets *p_wroff, struct ld_mws_signaling_rd_offsets *p_rdoff);

/**
 ****************************************************************************************
 * @brief Set MWS Transport Layer
 *
 * @param[in]  transport_layer              MWS Transport Layer
 * @param[in]  to_mws_baud_rate             Baud rate in the Bluetooth to MWS direction.
 * @param[in]  from_mws_baud_rate           Baud rate in the MWS to Bluetooth direction.
 *
 * @return status            0: success | 1-255: error
 *  ****************************************************************************************
 */
uint8_t ld_mws_transport_layer_set(uint8_t transport_layer, uint32_t to_mws_baud_rate, uint32_t from_mws_baud_rate);

/**
 ****************************************************************************************
 * @brief Set MWS Scan Frequency Tables
 *
 * @param[in]  num_freqs                    Number of frequencies
 * @param[in]  p_scan_freqs                 Pointer to struture of low & high.
 *
 * @return status            0: success | 1-255: error
 *  ****************************************************************************************
 */
uint8_t ld_mws_scan_freq_table_set(uint8_t num_freqs, const struct mws_scan_freq *scan_freqs);

/**
 ****************************************************************************************
 * @brief Set MWS Pattern Configuration
 *
 * @param[in]  pattern_index                MWS Pattern Index.
 * @param[in]  num_intervals                Number of Intervals.
 * @param[in]  p_intv                       Pointer to list of MWS Durations & Types.
 *
 * @return status            0: success | 1-255: error
 *  ****************************************************************************************
 */
uint8_t ld_mws_pattern_config_set(uint8_t pattern_index, uint8_t num_intervals, const struct mws_pattern_intv *p_intv);

/**
 ****************************************************************************************
 * @brief Build Xi mask for Inquiry Scan based on active MWS channels from IAC.
 *
 * @param[in]  iac_lap                      Inquiry access code LAP.
 *  ****************************************************************************************
 */
void ld_iscan_mwscoex_xi_mask_build(struct lap *iac_lap);

/**
 ****************************************************************************************
 * @brief Build Xi mask for Page Scan based on active MWS channels from local BD address.
 *****************************************************************************************
 */
void ld_pscan_mwscoex_xi_mask_build(void);

/**
 ****************************************************************************************
 * @brief Write local SAM Submap
 *
 * @param[in]  sam_ptr       Pointer to SAM Submap
 * @param[in]  sam_length    length of SAM Submap in slots
 ****************************************************************************************
 */
void ld_local_sam_submap_write(const uint8_t *sam_ptr, uint8_t sam_length);

/**
 ****************************************************************************************
 * @brief Set local SAM index
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  sam_idx       active local SAM index
 * @param[in]  instant       instant at which the new SAM index becomes active (CLK[27:1])
 ****************************************************************************************
 */
void ld_acl_local_sam_index_set(uint8_t link_id, uint8_t sam_idx, uint32_t instant);

#endif // (RW_BT_MWS_COEX)

/**
 ****************************************************************************************
 * @brief Set remote SAM map
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  sam_ptr       pointer to the SAM map
 * @param[in]  t_sam         map interval (slots)
 * @param[in]  instant       instant at which the new SAM index becomes active (CLK[27:1])
 * @param[in]  d_sam         map offset (slots)
 * @param[in]  flags         timing control
 ****************************************************************************************
 */
void ld_acl_remote_sam_map_set(uint8_t link_id, uint8_t *sam_ptr, uint16_t t_sam, uint32_t instant, uint8_t d_sam, bool flags);

/**
****************************************************************************************
* @brief Refresh remote SAM map
*
* @param[in]  link_id       Link identifier
* @param[in]  sam_ptr       pointer to the SAM map
* @param[in]  t_sam_sm     update immediate (0) or at next t_sam_sm interval (slots).
****************************************************************************************
*/
void ld_acl_remote_sam_map_refresh(uint8_t link_id, uint8_t *sam_ptr, uint8_t t_sam_sm);

/**
****************************************************************************************
* @brief Get timestamp for start of the next SAM frame.
*
* @param[in]  link_id       Link identifier
****************************************************************************************
*/
uint32_t ld_acl_sam_ts_get(uint8_t link_id);

#if (BT_HCI_TEST_MODE)
    /**
    ****************************************************************************************
    * @brief Start test mode (RX or TX)
    *
    * @param[in]  params        Pointer to Scanning parameters structure
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_test_start(struct ld_test_params *params);

    /**
    ****************************************************************************************
    * @brief Stop test mode
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t ld_test_stop(void);
#endif //(BT_HCI_TEST_MODE)

#if (EAVESDROPPING_SUPPORT)

    /**
    ****************************************************************************************
    * @brief Handle EDMICERR interrupt
    ****************************************************************************************
    */
    void ld_acl_micerr_isr(void);
    void ld_ed_micerr_isr(void);

    /// Clock Capture/Set (units are BT slots and us)
    void ld_set_clock(uint32_t clkn, uint16_t fine_cnt);
    void ld_get_clock(uint32_t *clkn, uint16_t *fine_cnt);

    /// Clock capture types
    void ld_clock_capture_int();
    void ld_clock_capture_imm();

    /// Interrupts
    void ld_clock_captured_isr(void);
    void ld_clock_adjusted_isr(void);

    #if VOICE_OVER_HCI
        bool ld_ed_sco_audio_isr(void);
    #endif // VOICE_OVER_HCI

    /// ACL connection
    uint8_t ld_acl_lt_addr_get(uint8_t link_id);
    void ld_acl_timing_info_get(uint8_t link_id, uint32_t *clk_off, uint16_t *bit_off);
    enum co_error ld_acl_get_aes_ccm_payload_counter(uint8_t con_type, uint8_t link_id, uint8_t *dst_ptr);
    enum co_error ld_acl_update_aes_ccm_payload_cntr(uint8_t con_type, uint8_t link_id, const uint8_t *src_ptr);
    enum co_error ld_acl_set_cust_evt_dur(uint8_t link_id, uint8_t evt_dur);
    enum co_error ld_acl_clr_cust_evt_dur(uint8_t link_id);
    enum co_error ld_acl_set_lrd(uint8_t link_id, bool lrd_en, bool control_tx_prog, bool ignore_seqn_errors, uint16_t l2cap_cid, bool null_flt);
    enum co_error ld_acl_allow_tx_prog(uint8_t link_id);
    enum co_error ld_acl_corr_payl_en(uint8_t link_id, bool en);
    enum co_error ld_acl_rx_stats_reporting(uint8_t link_id, bool en, uint8_t con_type, uint8_t intv);
    enum co_error ld_acl_handle_mic_invalid(uint8_t link_id, bool gen_mic_invalid_event, bool gen_aes_ccm_counter_event, bool increase_counter, bool tolerate_invalid_mic, uint8_t n_refr, uint8_t n_disc);
    enum co_error ld_acl_sample_esco_tx_seqn(uint8_t link_id);

    /// Eavesdropping
    enum co_error ld_ed_start_acl_eavesdropping(struct ld_ed_acl_params *params);
    enum co_error ld_ed_start_esco_eavesdropping(struct ld_ed_acl_params *params);
    enum co_error ld_ed_stop_eavesdropping(uint8_t type);
    enum co_error ld_ed_update_afh_channel_map(uint8_t ed_idx, uint32_t hssi, bool en, const struct bt_ch_map *ch_map);
    enum co_error ld_ed_update_encryption(uint8_t type, uint8_t key_size, const struct ltk *key, const struct initialization_vector *iv);
    enum co_error ld_ed_update_edr(uint8_t ptt);
    enum co_error ld_ed_update_max_slot(uint8_t max_slot);
    enum co_error ld_ed_update_esco_parameters(struct ld_ed_sco_params *params);
    enum co_error ld_ed_update_min_max_power(uint8_t level);
    enum co_error ld_ed_get_aes_ccm_payload_counter(uint8_t con_type, uint8_t *dst_ptr);
    enum co_error ld_ed_update_aes_ccm_payload_cntr(uint8_t con_type, const uint8_t *src_ptr);
    enum co_error ld_ed_set_lrd(bool lrd_en, bool ignore_seqn_errors, uint16_t l2cap_cid, bool null_flt);
    enum co_error ld_ed_set_tx_seqn(uint8_t tx_seqn);
    enum co_error ld_ed_acl_corr_payl_en(bool en);
    enum co_error ld_ed_rx_stats_reporting(bool en, uint8_t con_type, uint8_t intv);
    enum co_error ld_ed_handle_mic_invalid(bool gen_mic_invalid_event, bool gen_aes_ccm_counter_event, bool increase_counter);
    enum co_error ld_ed_sync_esco_tx_seqn(uint8_t tx_seqn, uint32_t clkn);

    /// Segmented scan
    uint8_t ld_iscan_seg_scan_en(struct ld_seg_scan_params *params);
    uint8_t ld_iscan_seg_scan_dis(void);
    uint8_t ld_pscan_seg_scan_en(struct ld_seg_scan_params *params);
    uint8_t ld_pscan_seg_scan_dis(void);

    uint8_t ld_page_set_cust_params(bool enable, uint16_t min_page_timeout, uint8_t train_nudging_offset);
#endif // EAVESDROPPING_SUPPORT

/// @} LD
#endif // LD_H_
