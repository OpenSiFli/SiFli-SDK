/**
****************************************************************************************
*
* @file lld.h
*
* @brief LLD API
*
* Copyright (C) RivieraWaves 2009-2016
*
*
****************************************************************************************
*/

#ifndef LLD_H_
#define LLD_H_

/**
 ****************************************************************************************
 * @defgroup LLD Link Driver
 * @ingroup ROOT
 * @brief LLD module handles the real-time job of Bluetooth operations.
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
#include <co_bt.h>          // BLE standard definitions
#include "ble_util_buf.h"   // BLE buffers management
#include "rwip_task.h"      // Task definitions
#if (BLE_ISO_PRESENT)
    #include "data_path.h"      // Used for Isochronous channels
#endif // (BLE_ISO_PRESENT)



/*
 * DEFINES
 *****************************************************************************************
 */

/// Unknown Tx Power Flags
#define LLD_PWR_FLGS_UNKNOWN          (0x80)


/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */


/*
 * MESSAGES IDENTIFIERS
 ****************************************************************************************
 */

/// Message API of the LLM task
/*@TRACE*/
enum lld_msg_id
{
    LLD_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_LLD),

    /*
     * ************** Msg LD->LLM****************
     */
    LLD_ADV_REP_IND,
    LLD_SCAN_REQ_IND,
    LLD_SYNC_START_REQ,
    LLD_PER_ADV_REP_IND,
    LLD_PER_ADV_RX_END_IND,
    LLD_SCAN_END_IND,
    LLD_ADV_END_IND,
    LLD_PER_ADV_END_IND,
    LLD_INIT_END_IND,
    LLD_TEST_END_IND,
#if BLE_CONLESS_CTE_RX
    LLD_CONLESS_CTE_RX_IND,
#endif // BLE_CONLESS_CTE_RX
    LLD_CH_SCAN_END_IND,

    /*
     * ************** Msg LLD->LLC****************
     */
    LLD_LLCP_RX_IND,
    LLD_LLCP_TX_CFM,
    LLD_ACL_RX_IND,
    LLD_ACL_TX_CFM,
    LLD_CON_PARAM_UPD_CFM,
    LLD_CH_MAP_UPD_CFM,
    LLD_PHY_UPD_CFM,
    LLD_DISC_IND,
    LLD_CON_OFFSET_UPD_IND,
#if BLE_PWR_CTRL
    LLD_CON_PWR_CTRL_IND,
    LLD_CON_PATH_LOSS_CHANGE_IND,
    LLD_CON_PWR_CHANGE_IND,
    LLD_CIS_PWR_CHANGE_IND,
#endif // BLE_PWR_CTRL
#if BLE_CON_CTE_REQ
    LLD_CON_CTE_RX_IND,
#endif // BLE_CON_CTE_REQ
#if (RW_DEBUG)
    LLD_CON_EVT_TRACE_IND,
#endif //(RW_DEBUG)
#if (BLE_CIS)
    LLD_CIS_STOP_IND,
    LLD_CIS_ESTAB_IND,
#endif // (BLE_CIS)
#if (BLE_BIS)
    LLD_BIG_STOP_IND,
    LLD_BIG_RX_IND,
    LLD_BIG_TX_IND,
    LLD_BIG_SYNC_ESTAB_IND,
    LLD_BIG_SYNC_OFFSET_UPD_IND,
#endif // (BLE_BIS)
};

/*
 * MESSAGES STRUCTURES
 ****************************************************************************************
 */

/// Advertising report indication structure
/*@TRACE*/
struct lld_adv_rep_ind
{
    /// Peer device address match
    bool peer_add_match;

    /// local add Match
    bool local_add_match;

    /// Address of the RAL Structure for which RPA resolution is successful
    uint16_t rxralptr;

    /// Peer device address
    struct bd_addr peer_id_addr;

    /// Target device address
    struct bd_addr target_id_addr;

    /// Advertising Data Info field (Data ID + Set ID)
    uint16_t adi;

    /// Indicate if Advertising Data Info field is present
    bool adi_present;

    /// Activity identifier
    uint8_t act_id;

    /**
     * Bit
     * 0        Connectable advertising
     * 1        Scannable advertising
     * 2        Directed advertising
     * 3        Scan response
     * 4        Legacy advertising PDUs used
     */
    uint8_t evt_type;

    /**
     * Data status
     * 00b Complete
     * 01b Incomplete, more data to come
     * 10b Incomplete, data truncated, no more to come
     * 11b Reserved for future use
     */
    uint8_t data_status;

    /**
     * Peer address type
     * 0x00 Public Device Address
     * 0x01 Random Device Address
     * 0xFF None - Anonymous
     */
    uint8_t addr_type;

    /// RxAdd (0: Public | 1: Random)
    uint8_t rx_rxadd;

    /// Primary PHY (@see enum lld_rate)
    uint8_t rate1;

    /// Secondary PHY (@see enum lld_rate)
    uint8_t rate2;

    // Tx Power
    uint8_t tx_power;

    /// Received signal strength in dBm
    int8_t rssi;

    /// Periodic advertising interval
    uint16_t interval;

    /// Data length in bytes
    uint8_t data_len;

    /// Chain ID (differentiate interleaved chains - chain 1, chain 2)
    uint8_t chain;

    /// EM buffer to read & free. Valid if data_len != 0.
    uint16_t em_buf;

    /// Data offset
    uint8_t data_offset;
};

// SyncInfo RX request for initiating a Periodic Scan
/*@TRACE*/
struct lld_sync_start_req
{
    /// Activity identifier
    uint8_t act_id;

    // SyncInfo
    struct sync_info syncinfo;

    /// The clock value of the reference time (in half-slot)
    uint32_t base_cnt;

    /// The fine timer value the reference time (in half-us)
    uint16_t fine_cnt;

    /// The bit rate of periodic advertising (@see enum lld_rate)
    uint8_t rate;

    /// Advertising SID
    uint8_t adv_sid;

    /// Address of the RAL Structure for which RPA resolution is successful
    uint16_t rxralptr;

    /// Advertising address type (0: public / 1: random)
    uint8_t adv_addr_type;

    /// Advertiser address
    struct bd_addr adv_addr;

    /// max Skip after receive
    uint16_t skip;

    /// Sync timeout (Time=N*10ms)
    uint16_t sync_to;
};

/// Periodic advertising report indication structure
/*@TRACE*/
struct lld_per_adv_rep_ind
{
    /// Activity identifier
    uint8_t    act_id;

    /// Advertiser rate (@see enum lld_rate)
    uint8_t    rate;

    /// Periodic advertising interval (Time=N*1.25ms)
    uint16_t   interval;

    /// Activity offset in half-slots (312.5 us)
    uint16_t   act_offset;

    /// value of the sync event counter when the packet is received
    uint16_t   ref_evt_cnt;

    /// Advertiser clock accuracy
    uint8_t    adv_ca;

    /// Tx Power
    uint8_t    tx_power;

    /// RSSI
    int8_t     rssi;

    /**
     * Data status
     * 0: Complete
     * 1: Incomplete, more data to come
     * 2: Incomplete, data truncated, no more to come
     */
    uint8_t    data_status;

    /// Data length in bytes
    uint8_t    data_len;

    /// Indicates data is filtered. Event reported to LLM for other purposes.
    bool       filt_data;

    /// EM buffer to read & free. Valid if data_len != 0.
    uint16_t   em_buf;

    /// Data offset
    uint8_t    data_offset;

    /// CTE Type (0x00: A0A, 0x01: AoD 1 us, 0x02: AoD 2us, 0xFF: No CTE, All other values: RFU)
    uint8_t    cte_type;

    /// Used to know if ACAD data present in the received ADV packet - 0 means not present
    uint8_t    acad_len;

    /// offset of ACAD data content in the EM Buffer
    uint8_t    acad_offset;
};

/// Periodic advertising RX end indication structure
/*@TRACE*/
struct lld_per_adv_rx_end_ind
{
    /// Activity identifier
    uint8_t    act_id;

    /// Status of the advertising sync
    uint8_t status;
};

/// Scan request indication structure
/*@TRACE*/
struct lld_scan_req_ind
{
    /// Activity identifier
    uint8_t act_id;

    /**
     * Scanner address type
     * 0x00 Public Device Address
     * 0x01 Random Device Address
     * 0x02 Public Identity Address (Corresponds to Resolved Private Address)
     * 0x03 Random (static) Identity Address (Corresponds to Resolved Private Address)
     * 0x04-0xFF Reserved for future use
     */
    uint8_t addr_type;

    /// Scanner address
    struct bd_addr addr;
};

/// Advertising end indication structure
/*@TRACE*/
struct lld_adv_end_ind
{
    /// Activity identifier
    uint8_t act_id;

    /// Status (BLE error code)
    uint8_t status;

    /// Local resolvable private address used for advertising
    struct bd_addr local_rpa;

    /// Peer resolvable private address of initiator or peer address indicated in CONNECT_REQ packet
    struct bd_addr peer_rpa;

    /// Initiator identity address
    struct bd_addr peer_id_addr;

    /// Interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t            interval;

    /// Latency
    uint16_t            latency;

    /// Timeout (in units of 10 ms, i.e. 16 slots)
    uint16_t            timeout;

    /// Channel mapping
    struct le_ch_map    ch_map;

    /**
     * Master Sleep clock accuracy (only for slave)
     * 0 251 ppm to 500 ppm
     * 1 151 ppm to 250 ppm
     * 2 101 ppm to 150 ppm
     * 3 76 ppm to 100 ppm
     * 4 51 ppm to 75 ppm
     * 5 31 ppm to 50 ppm
     * 6 21 ppm to 30 ppm
     * 7 0 ppm to 20 ppm
     */
    uint8_t master_sca;

    /// PHY rate to be used for the connection (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t rate;

    /// Connection status (True: device connected | False: device not connected)
    bool connected;

    /// Indicates whether channel selection algorithm #2 will be used or not
    bool ch_sel_2;

    /**
     * Peer address type
     * 0x00 Public Device Address
     * 0x01 Random Device Address
     * 0x02 Public Identity Address (Corresponds to peer's Resolvable Private Address)
     * 0x03 Random (static) Identity Address (Corresponds to peer's Resolvable Private Address)
     * 0x04 - 0xFF Reserved for future use
     */
    uint8_t peer_addr_type;

    /// Number of completed extended advertising events
    uint8_t nb_ext_adv_evts;
};

/// Periodic advertising end indication structure
struct lld_per_adv_end_ind
{
    /// Activity identifier
    uint8_t act_id;

    /// Status (BLE error code)
    uint8_t status;
};

/// Scanning end indication structure
/*@TRACE*/
struct lld_scan_end_ind
{
    /// Status (BLE error code)
    uint8_t status;

    /// Activity identifier used for advertising
    uint8_t act_id;
};

/// Test mode end indication structure
/*@TRACE*/
struct lld_test_end_ind
{
    /// Status (BLE error code)
    uint8_t status;

    /// Number of packets received
    uint16_t nb_pkt_recv;
};

/// Initiating end indication structure
/*@TRACE*/
struct lld_init_end_ind
{
    /// Activity identifier
    uint8_t act_id;

    /// Connection status (True: device connected | False: device not connected)
    bool connected;

    /// Local resolvable private address used for initiating
    struct bd_addr local_rpa;

    /// Peer resolvable private address of advertiser or peer address indicated in ADV packet
    struct bd_addr peer_rpa;

    /// Advertiser identity address
    struct bd_addr peer_id_addr;

    /// Connection interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t interval;

    /// Connection latency
    uint16_t latency;

    /// Connection supervision timeout in 10ms (N * 10ms)
    uint16_t timeout;

    /// Connection offset in half-slots (312.5 us)
    uint16_t con_offset;

    /**
     * Peer address type
     * 0x00 Public Device Address
     * 0x01 Random Device Address
     * 0x02 Public Identity Address (Corresponds to peer's Resolvable Private Address)
     * 0x03 Random (static) Identity Address (Corresponds to peer's Resolvable Private Address)
     * 0x04 - 0xFF Reserved for future use
     */
    uint8_t peer_addr_type;

    /// Channel map for use on the connection
    struct le_ch_map ch_map;

    /// Index of initiating PHY used
    uint8_t phy_idx;

    /// PHY rate to be used for the connection (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t rate;

    /// Master sleep clock accuracy (@see enum SCA)
    uint8_t master_sca;

    /// Indicates whether channel selection algorithm #2 will be used or not
    bool ch_sel_2;
};

/// LLD disconnection indication structure
/*@TRACE*/
struct lld_disc_ind
{
    /// Driver disconnection reason
    uint8_t reason;
};

/// LLD LLCP reception indication structure
/*@TRACE*/
struct lld_llcp_rx_ind
{
    /// Event counter when the packet has been received
    uint16_t event_cnt;
    /// PDU length
    uint8_t length;
    /// EM buffer containing PDU
    uint16_t em_buf;
};

/// LLD ACL reception indication structure
/*@TRACE*/
struct lld_acl_rx_ind
{
    /// EM buffer containing PDU
    uint16_t em_buf;
    /// Data length + Data Flags (PBF + BF)
    uint16_t data_len;
    /// LLID
    uint8_t  llid;
};

/// LLD connection offset update indication structure
/*@TRACE*/
struct lld_con_offset_upd_ind
{
    /// Connection offset in half-slots (312.5 us)
    uint16_t con_offset;
};

#if BLE_PWR_CTRL
/// LLD connection power control indication structure
/*@TRACE*/
struct lld_con_pwr_ctrl_ind
{
    /// Delta in power control required (dBm)
    int8_t  delta;
    /// The bit rate on which delta required (@see enum lld_rate)
    uint8_t rx_rate;
};

/// LLD connection path loss change indication structure
/*@TRACE*/
struct lld_con_path_loss_change_ind
{
    /// Current path loss
    uint8_t path_loss;
    /// Path loss zone
    uint8_t zone;
};

/// LLD connection power change indication structure
/*@TRACE*/
struct lld_con_pwr_change_ind
{
    /// Transmit power (dBm)
    int8_t  pwr;
    /// The bit rate to which the transmit power refers (@see enum lld_rate)
    uint8_t tx_rate;
    /// The disused transmit rate (@see enum lld_rate)
    uint8_t d_rate;
};

/// CIS power change indication structure
/*@TRACE*/
struct lld_cis_pwr_change_ind
{
    /// The bit rate to which the transmit power refers (@see enum lld_rate)
    int8_t  tx_rate;
    /// Whether power management is enabled (true) or disabled (false).
    bool en;
};
#endif // BLE_PWR_CTRL

#if BLE_CON_CTE_REQ
/// LLD connection CTE RX indication structure
/*@TRACE*/
struct lld_con_cte_rx_ind
{
    /// RX CTE descriptor index
    uint8_t em_rx_cte_desc_idx;
    /// CTE time (in 8us unit)
    uint8_t cte_time;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t cte_type;
    /// Number of IQ samples
    uint8_t nb_iq_samp;
    /// PHY (@see enum le_phy_value)
    uint8_t  phy;
    /// Channel index
    uint8_t  channel_idx;
    /// RSSI (in 0.1 dBm)
    int16_t rssi;
    /// RSSI antenna ID
    uint8_t rssi_antenna_id;
    /// connEventCounter
    uint16_t con_evt_cnt;
};
#endif // BLE_CON_CTE_REQ

#if (RW_DEBUG)
/// LLD connection Event trace indication structure
/*@TRACE*/
struct lld_con_evt_trace_ind
{
    /// Number of Events
    uint8_t nb_events;
    /// data
    uint16_t data[HCI_CON_MAX_TRACE_NB_EVENT];
};
#endif //(RW_DEBUG)

#if BLE_CONLESS_CTE_RX
/// LLD connection CTE RX indication structure
/*@TRACE*/
struct lld_conless_cte_rx_ind
{
    /// Activity identifier
    uint8_t act_id;
    /// RX CTE descriptor index
    uint8_t em_rx_cte_desc_idx;
    /// Data channel index
    uint8_t  channel_idx;
    /// RSSI (in 0.1 dBm)
    int16_t rssi;
    /// RSSI antenna ID
    uint8_t rssi_antenna_id;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t cte_type;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t slot_dur;
    /// paEventCounter
    uint16_t pa_evt_cnt;
    /// Sample_count
    uint8_t sample_cnt;
};
#endif // BLE_CONLESS_CTE_RX


#if (BLE_CIS)
/// LLC information about stopped CIS
/*@TRACE*/
struct lld_cis_stop_ind
{
    /// Activity identifier
    uint8_t act_id;
    /// Reason
    uint8_t reason;
};

/// LLC information about established CIS
/*@TRACE*/
struct lld_cis_estab_ind
{
    /// Activity identifier
    uint8_t act_id;
    /// Status (BLE error code)
    uint8_t status;
};
#endif // (BLE_CIS)

#if (BLE_BIS)
/// Indicate BIS has stopped
/*@TRACE*/
struct lld_big_stop_ind
{
    /// Group Handle
    uint8_t grp_hdl;
    /// Reason
    uint8_t reason;
};

/// Indicate reception of BIS control PDU
/*@TRACE*/
struct lld_big_rx_ind
{
    /// Group Handle
    uint8_t  grp_hdl;
    /// Length
    uint8_t  length;
    /// Address of buffer in the exchange memory
    uint16_t em_buf;
    /// Event counter
    uint16_t evt_cnt;
};

/// Indicate transmission of BIS control PDU
/*@TRACE*/
struct lld_big_tx_ind
{
    /// Group handle
    uint8_t grp_hdl;
};

/// Indicate synchronization on BIG
/*@TRACE*/
struct lld_big_sync_estab_ind
{
    /// Group Handle
    uint8_t grp_hdl;
    /// Status (BLE error code)
    uint8_t status;
    /// Activity offset (in half-slots)
    uint8_t act_offset;
};

/// Indicate new offset of a synchronized BIS
/*@TRACE*/
struct lld_big_sync_offset_upd_ind
{
    /// Group Handle
    uint8_t grp_hdl;
    /// Activity offset in half-slots (312.5 us)
    uint16_t act_offset;
};

#endif // (BLE_BIS)

/// Channel Scanning end indication structure
/*@TRACE*/
struct lld_ch_scan_end_ind
{
    /// Status (BLE error code)
    uint8_t status;
    /// Activity index
    uint8_t act_id;
};
/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

#if (EAVESDROPPING_SUPPORT)
/// Custom transmitter test parameters structure
struct lld_cust_tx_test_params
{
    /// Enable/Disable
    uint8_t enable;
    /// Number of transmitted packets
    uint16_t tx_packets;
    /// Transmit power (in dBm)
    int8_t tx_power;
};
#endif //(EAVESDROPPING_SUPPORT)

#if (EAVESDROPPING_SUPPORT)
#if (BLE_BROADCASTER || BLE_PERIPHERAL)
/// Custom advertising parameters structure
struct lld_cust_adv_params
{
    /// Distance for customized advertising
    uint16_t periodicity;
    /// Customization enabled
    bool en;
    /// Delay randomization enabled or fixed to 0
    bool rand_en;
    /// Indicates whether ADV_EXT_IND packets are grouped during custom advertising
    bool group_ext_adv_ind;
    /// Valid only if bit 0 in cust_param value is set to 1
    uint16_t time_offset;
    /// Custom us offset
    int16_t us_offset;
    /// BD address of a master, if an active slave connection is present, the advertising timings are based on this piconet
    struct bd_addr bd_addr;
};

/// Manual advertising parameters structure
struct lld_manual_adv_params
{
    /// Manual advertising enabled
    bool en;
    /// Time offset
    uint16_t time_offset;
    /// Maximum random offset
    uint16_t max_rand_offset;
};
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)

#if (BLE_OBSERVER)
/// Segmented scanning parameters structure
struct lld_scan_seg_params
{
    ///Indicates the PHY(s) on which the scans shall be segmented (@see enum le_phy_mask)
    uint8_t         scan_phys;

    /// BD address of a BR/EDR master, if an active slave connection is present, the scan timings are based on this piconet
    struct bd_addr bd_addr;

    /// Time offset (in slots)
    uint16_t        time_offset_1m; // 1M PHY
    uint16_t        time_offset_c; // coded PHY

    /// Periodicity (in slots)
    uint16_t        periodicity_1m; // 1M PHY
    uint16_t        periodicity_c; // coded PHY

    /// Duration of each segment (in usecs)
    uint16_t        duration_1m; // 1M PHY
    uint16_t        duration_c; // coded PHY

    /// segmnets per scan (in number of segments)
    uint16_t        segments_1m; // 1M PHY
    uint16_t        segments_c; // coded PHY

    /// Microsecond offset (in usecs)
    int16_t         us_offset_1m; // 1M PHY
    int16_t         us_offset_c; // coded PHY
};
#endif //(BLE_OBSERVER)
#endif //(EAVESDROPPING_SUPPORT)

/// Advertising parameters structure
struct lld_adv_params
{
    /// BD Address of the local device
    struct bd_addr own_addr;

    /// BD Address of the peer device (only for directed advertising)
    struct bd_addr peer_addr;

    /// Primary advertising interval
    uint32_t prim_adv_intv;

    /// Initial advertising data buffer
    uint16_t init_adv_data_buf;

    /// Initial scan response data buffer
    uint16_t init_scan_rsp_data_buf;

    /// Initial advertising data length
    uint16_t init_adv_data_len;

    /// Initial scan response data length
    uint16_t init_scan_rsp_data_len;

    /// Advertising event properties
    uint16_t adv_evt_properties;

    /// Duration (N * 10 ms), 0x0000 No advertising duration. Advertising to continue until the Host disables it.
    uint16_t duration;

    /// Maximum number of extended advertising events
    uint8_t max_ext_adv_evt;

    /// Primary advertising channel map
    uint8_t prim_adv_ch_map;

    /**
     * Local address type
     * 0x00 Public Device Address (default)
     * 0x01 Random Device Address
     * 0x02 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use public address.
     * 0x03 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use random address from LE_Set_Random_Address.
     */
    uint8_t own_addr_type;

    /**
     * Peer address type (only for directed advertising)
     * 0x00 Public Device Address (default) or Public Identity Address
     * 0x01 Random Device Address or Random (static) Identity Address
     * 0x02 - 0xFF Reserved for future use
     */
    uint8_t peer_addr_type;

    /**
     * Advertising filter policy:
     * 0x00 Process scan and connection requests from all devices (i.e., the White List is not in use) (default).
     * 0x01 Process connection requests from all devices and only scan requests from devices that are in the White List.
     * 0x02 Process scan requests from all devices and only connection requests from devices that are in the White List.
     * 0x03 Process scan and connection requests only from devices in the White List.
     * 0x04 - 0xFF Reserved for future use.
     */
    uint8_t adv_filter_policy;

    /// Advertising Tx power
    int8_t adv_tx_pwr;

    /// Primary advertising PHY
    uint8_t prim_adv_phy;

    /// Secondary advertising max skip
    uint8_t sec_adv_max_skip;

    /// Secondary advertising PHY
    uint8_t sec_adv_phy;

    /// Advertising SID
    uint8_t adv_sid;

    /// Scan request notification enable
    uint8_t scan_req_notif_en;

    /// Address resolution enabled
    bool addr_resolution_en;

    /// Index of the RAL entry corresponding to peer
    uint8_t ral_idx;

    /// Activity for the potential resulting connection (only for connectable advertising, 0xFF if not used)
    uint8_t con_act_id;

#if (BT_53)
    /// Reasons for refreshing RPAs
    uint8_t chg_reasons;

    /// Boolean indicating if the set needs to update RPAs the next time it is enabled (when the driver is restarted)
    uint8_t adv_rpa_upd;
#endif // (BT_53)

#if (EAVESDROPPING_SUPPORT)
#if (BLE_BROADCASTER || BLE_PERIPHERAL)
    struct lld_cust_adv_params cust_adv_params;
    struct lld_manual_adv_params manual_adv_params;
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)
#endif //(EAVESDROPPING_SUPPORT)
};

#if BLE_CONLESS_CTE_TX
/// Periodic advertising CTE parameters structure
struct lld_per_adv_cte_params
{
    /// CTE length (in 8us unit)
    uint8_t     cte_len;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     cte_type;
    /// CTE count (number of CTEs to transmit in each periodic advertising interval, range 0x01 to 0x10)
    uint8_t     cte_count;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[BLE_MAX_SW_PAT_LEN];
};
#endif // BLE_CONLESS_CTE_TX

/// Periodic advertising parameters structure
struct lld_per_adv_params
{
    /// Periodic advertising interval expressed in units of 1.25 ms
    uint16_t per_adv_intv;

    /// Periodic advertising properties
    uint16_t per_adv_properties;

    /// Initial periodic advertising data buffer
    uint16_t init_adv_data_buf;

    /// Initial periodic advertising data length
    uint16_t init_adv_data_len;

    /// Periodic advertising offset in half-slots (312.5 us)
    uint16_t per_adv_offset;

    /**
     * Local address type
     * 0x00 Public Device Address (default)
     * 0x01 Random Device Address
     * 0x02 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use public address.
     * 0x03 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use random address from LE_Set_Random_Address.
     */
    uint8_t own_addr_type;

    /// Periodic advertising Tx power
    int8_t adv_tx_pwr;

    /// Secondary advertising PHY
    uint8_t sec_adv_phy;

#if BLE_CONLESS_CTE_TX
    struct lld_per_adv_cte_params cte_params;
#endif // BLE_CONLESS_CTE_TX

#if (BLE_BIS)
    /// BIG activity attached to periodic advertiser - 0xFF means no BIG attached
    uint8_t big_act_id;
#endif // (BLE_BIS)

#if (BT_53)
    /// Indicates whether periodic advertising packets include the ADI field
    bool include_adi;

    /// Advertising SID
    uint8_t adv_sid;
#endif // (BT_53)
};

/// Scanning parameters structure
struct lld_scan_params
{
    /// BD Address of the local device
    struct bd_addr own_addr;

    /// Extended scanning
    bool ext_scan;

    /// Scanning PHYs (bit0: 1M PHY, bit2: coded PHY)
    uint8_t scan_phys;

    /// Scan interval in slots (625 us)
    uint16_t intv_1m; // 1M PHY
    uint16_t intv_c; // coded PHY

    /// Scan window in slots (625 us)
    uint16_t win_1m; // 1M PHY
    uint16_t win_c;  // coded PHY

    /**
     * Scanning type
     * 0x00 Passive Scanning. No SCAN_REQ packets shall be sent (default)
     * 0x01 Active scanning. SCAN_REQ packets may be sent.
     * 0x02 - 0xFF Reserved for future use
     */
    uint8_t type_1m; // 1M PHY
    uint8_t type_c; // coded PHY

    /**
     * Local address type
     * 0x00 Public Device Address (default)
     * 0x01 Random Device Address
     * 0x02 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use public address.
     * 0x03 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use random address from LE_Set_Random_Address.
     */
    uint8_t own_addr_type;

    /**
     * Scan filter policy:
     * 0x00 Accept all
     *   - advertisement packets except directed advertising packets not addressed to this device (default)
     * 0x01 Accept only
     *   - advertisement packets from devices where the advertiser's address is in the White list.
     *   - Directed advertising packets which are not addressed for this device shall be ignored.
     * 0x02 Accept all
     *   - undirected advertisement packets, and
     *   - directed advertising packets where the initiator address is a resolvable private address, and
     *   - directed advertising packets addressed to this device.
     * 0x03 Accept all
     *   - advertisement packets from devices where the advertiser's address is in the White list, and
     *   - directed advertising packets where the initiator address is a resolvable private address, and
     *   - directed advertising packets addressed to this device.
     * 0x04 - 0xFF Reserved for future use.
     */
    uint8_t filter_policy;

    /// Address resolution enabled
    bool addr_resolution_en;

    /// Scan duration (N * 10 ms)  | 0x0000: Scan continuously until explicitly disable
    uint16_t duration;
};

#if BLE_CONLESS_CTE_RX
/// Sync CTE parameters structure
struct lld_sync_cte_params
{
    /// Sampling enable
    uint8_t     sampl_en;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t     slot_dur;
    /// Max sampled CTEs
    uint8_t     max_sampl_cte;
    /// Length of switching pattern
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[BLE_MAX_SW_PAT_LEN];
};
#endif // BLE_CONLESS_CTE_RX

/// Sync parameters structure
struct lld_sync_params
{
    // SyncInfo
    const struct sync_info *p_syncinfo;

    /// The clock value of the reference time (in half-slot)
    uint32_t base_cnt;

    /// The fine timer value the reference time (in half-us)
    uint16_t fine_cnt;

    /// The bit rate of periodic advertising (@see enum lld_rate)
    uint8_t rate;

    /// max Skip after receive
    uint16_t skip;

    /// Sync timeout (Time=N*10ms)
    uint16_t sync_to;

    /// Additional drift to consider on given time reference (in half-us)
    uint16_t add_drift;

    /// Specifies whether to only synchronize to periodic advertising with certain types of CTE (@see enum sync_cte_type)
    uint8_t sync_cte_type;

#if (BT_53)
    /// Advertising SID
    uint8_t adv_sid;

    /// ADI filtering enable
    bool adi_filt_en;
#endif // (BT_53)

    /// Reporting enable
    bool rep_en;

    /// Advertiser address
    struct bd_addr adv_addr;
};

/// Initiating parameters PHY sub-structure
struct lld_init_phy_params
{
    /// Scan interval in slots (625 us)
    uint16_t intv;

    /// Scan window in slots (625 us)
    uint16_t win;

    /// Connection interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t con_intv;

    /// Connection offset in half-slots (312.5 us)
    uint16_t con_offset;

    /// Connection latency
    uint16_t con_latency;

    /// Connection supervision timeout in 10ms (N * 10ms)
    uint16_t superv_to;
};

/// Initiating parameters structure
struct lld_init_params
{
    /// BD Address of the local device
    struct bd_addr own_addr;

    /// BD Address of the peer device (only if White List is not used)
    struct bd_addr peer_addr;

    /// Channel map to use for the connection
    struct le_ch_map ch_map;

    /// Extended initiating
    bool ext_init;

    /// Scanning PHYs (bit0: 1M PHY, bit1: 2M PHY, bit2: coded PHY)
    uint8_t init_phys;

    /// Activity identifier
    uint8_t act_id;

    /**
     * Local address type
     * 0x00 Public Device Address (default)
     * 0x01 Random Device Address
     * 0x02 To generate RPA from resolving list. If no matching entry, use public address.
     * 0x03 To generate RPA from resolving list. If no matching entry, use random address.
     */
    uint8_t own_addr_type;

    /**
     * Peer address type (only if White List is not used)
     * 0x00 Public Device Address
     * 0x01 Random Device Address
     * 0x02 - 0xFF Reserved for future use
     */
    uint8_t peer_addr_type;

    /**
     * Initiator filter policy:
     * 0x00 White list is not used to determine which advertiser to connect to. Peer_Address_Type and Peer_Address
     *  shall be used.
     * 0x01 White list is used to determine which advertiser to connect to. Peer_Address_Type and Peer_Address shall
     *  be ignored.
     * 0x02 - 0xFF Reserved for future use.
     */
    uint8_t filter_policy;

    /**
     * Parameters for Initiating PHY(s)
     * The parameters are ordered in the same order as the set bits in the init_phys parameter, starting from bit 0.
     * The number of elements is determined by the number of bits set in the Initiating_PHYs parameter.
     */
    struct lld_init_phy_params phy[MAX_INIT_PHYS];

    /// Address resolution enabled
    bool addr_resolution_en;
};

/// Connection parameters structure
struct lld_con_params
{
    /// Access address
    struct access_addr  aa;

    /// CRC init
    struct crc_init     crcinit;

    /// Timestamp of the first anchor point (in half-slots)
    uint32_t            first_anchor_hs;

    /// Timestamp of the first anchor point (in half-us)
    uint16_t            first_anchor_hus;

    /// Window offset (in units of 1,25 ms, i.e. 2 slots)
    uint16_t            winoffset;

    /// Interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t            interval;

    /// Latency
    uint16_t            latency;

    /// Timeout (in units of 10 ms, i.e. 16 slots)
    uint16_t            timeout;

    /// Channel mapping
    struct le_ch_map    ch_map;

    /// Window size (in units of 1,25 ms, i.e. 2 slots)
    uint8_t             winsize;

    /// Hopping increment (random value in the range of 5 to 16)
    uint8_t             hop_inc;

    /**
     * Master Sleep clock accuracy (only for slave)
     * 0 251 ppm to 500 ppm
     * 1 151 ppm to 250 ppm
     * 2 101 ppm to 150 ppm
     * 3 76 ppm to 100 ppm
     * 4 51 ppm to 75 ppm
     * 5 31 ppm to 50 ppm
     * 6 21 ppm to 30 ppm
     * 7 0 ppm to 20 ppm
     */
    uint8_t master_sca;

    /// Role (0: Master | 1: Slave)
    uint8_t role;

    /// Connection transmit/receive rate (@see enum lld_rate)
    uint8_t rate;

    /// Indicates whether channel selection algorithm #2 will be used or not
    bool ch_sel_2;
};

#if (BLE_ISO_MODE_0)
/// Audio parameters structure
struct lld_con_am0_params
{
    /// Configure transmitter size in bytes
    uint8_t     tx_size;
    /// Configure receiver size in bytes
    uint8_t     rx_size;
};
#endif //(BLE_ISO_MODE_0)

/// Test mode parameters structure
struct lld_test_params
{
    /// Type (0: RX | 1: TX)
    uint8_t type;

    /// RF channel, N = (F - 2402) / 2
    uint8_t channel;

    /// Length of test data
    uint8_t data_len;

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
     * Tx/Rx PHY
     * For Tx PHY:
     * 0x00 Reserved for future use
     * 0x01 LE 1M PHY
     * 0x02 LE 2M PHY
     * 0x03 LE Coded PHY with S=8 data coding
     * 0x04 LE Coded PHY with S=2 data coding
     * 0x05-0xFF Reserved for future use
     * For Rx PHY:
     * 0x00 Reserved for future use
     * 0x01 LE 1M PHY
     * 0x02 LE 2M PHY
     * 0x03 LE Coded PHY
     * 0x04-0xFF Reserved for future use
     */
    uint8_t phy;

    /**
     * CTE length
     * 0x00 No Constant Tone Extension
     * 0x02 - 0x14 Length of the Constant Tone Extension in 8 us units
     * All other values Reserved for future use
     */
    uint8_t cte_len;

    /**
     * CTE type
     * 0x00 AoA Constant Tone Extension
     * 0x01 AoD Constant Tone Extension with 1 us slots
     * 0x02 AoD Constant Tone Extension with 2 us slots
     * All other values Reserved for future use
     */
    uint8_t cte_type;

    /**
     * Slot durations
     * 0x01 Switching and sampling slots are 1 us each
     * 0x02 Switching and sampling slots are 2 us each
     * All other values Reserved for future use
     */
    uint8_t slot_dur;

    /**
     * Length of switching pattern
     * 0x02 - 0x4B The number of Antenna IDs in the pattern
     * All other values Reserved for future use
     */
    uint8_t switching_pattern_len;

    /// Antenna IDs
    uint8_t antenna_id[BLE_MAX_SW_PAT_LEN];

    /// Transmit power level in dBm (0x7E: minimum | 0x7F: maximum | range: -127 to +20)
    int8_t  tx_pwr_lvl;

#if (EAVESDROPPING_SUPPORT)
    struct lld_cust_tx_test_params cust_tx_test_params;
#endif //(EAVESDROPPING_SUPPORT)
};

#if (BLE_CIS)
/// CIS parameters structure
struct lld_cis_params
{
    /// CIS Channel Access address
    uint32_t access_addr;
    /// TX SDU interval (in us)
    uint32_t tx_sdu_interval;
    /// RX SDU interval (in us)
    uint32_t rx_sdu_interval;
    /// TX transport latency (in us)
    uint32_t tx_trans_latency;
    /// RX transport latency (in us)
    uint32_t rx_trans_latency;
    /// Sub-event Interval (in units of 1us)
    /// The range shall be 400us to Channel Interval
    uint32_t sub_interval;
    /// Connection offset in us
    uint32_t con_offset;
    /// Offset time (spacing) of CIS in the CIG
    uint32_t cis_spacing;
    /// CIG sync delay (in us)
    uint32_t cig_sync_delay;
    /// TX maximum SDU size (12 bits meaningful) (in bytes)
    uint16_t tx_max_sdu;
    /// RX maximum SDU size (12 bits meaningful) (in bytes)
    uint16_t rx_max_sdu;
    /// Interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t iso_interval;
    /// Duration of a sub-event to packet size and PHY selected in us
    uint16_t sub_evt_dur;
    /// Connection reference event counter
    uint16_t con_evt_cnt;
    /// Connection activity identifier
    uint8_t  con_link_id;
    /// Activity ID
    uint8_t  act_id;
    /// Group Local Handle
    uint8_t  grp_hdl;
    /// Role (0: Master | 1: Slave)
    uint8_t  role;
    /// TX Flush timeout (Range 0x01-0x1F)
    uint8_t  tx_ft;
    /// RX Flush timeout (Range 0x01-0x1F)
    uint8_t  rx_ft;
    /// Transmit PHY (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t  tx_rate;
    /// Receive PHY (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t  rx_rate;
    /// Number of SubEvents (Range 0x01-0x1F)
    uint8_t  nse;
    /// SDU Frame mode (@see enum iso_frame)
    uint8_t  framing;
    /// Packing (0 - Interleaved, 1 - Sequential)
    uint8_t  packing;
    /// TX maximum PDU size (Range 0x00-0xFB) (in bytes)
    uint8_t  tx_max_pdu;
    /// RX maximum PDU size (Range 0x00-0xFB) (in bytes)
    uint8_t  rx_max_pdu;
    /// TX burst number: number of new payloads per event (0: no new packet -  Range 0x01-0x1F)
    uint8_t  tx_bn;
    /// RX burst number: number of new payloads per event (0: no new packet -  Range 0x01-0x1F)
    uint8_t  rx_bn;
};
#endif // (BLE_CIS)

#if (BLE_BIS)
/// BIS driver parameters structure
struct lld_bis_params
{
    /// BIS Activity Identifier
    uint8_t act_id;
    /// Use to generate access address for the BIS and for position of the BIS in the BIG
    uint8_t idx;
};

/// BIG driver parameters structure
struct lld_big_params
{
    /// Offset of update subevent (in us)
    uint32_t                       ctrl_sub_evt_offset;
    /// ISO interval in microseconds
    uint32_t                       iso_interval_us;
    /// SDU transport latency in microseconds
    uint32_t                       trans_latency;
    /// Pre-Transmission duration in microseconds
    uint32_t                       pretrans_dur;
    /// Duration of a sub-event according to packet size and PHY selected in us
    uint16_t                       sub_evt_dur;
    /// Control sub-event duration
    uint16_t                       ctrl_sub_evt_dur;
    /// Activity offset (in half-slots)
    uint16_t                       act_offset;

#if (BLE_OBSERVER)
    /// BIG supervision timeout (10 ms step)
    uint16_t                       sync_timeout;
    /// Reference counter of periodic sync
    uint16_t                       ref_event_cnt;
    /// Activity identifier of periodic sync driver
    uint8_t                        per_sync_id;
#endif //(BLE_OBSERVER)

    /// Group Handle
    uint8_t                        grp_hdl;
    /// Role (0: Broadcaster | 1: Broadcast receiver)
    uint8_t                        role;
    /// Maximum PDU size (Range 0x00-0xFB) (in bytes)
    uint8_t                        max_pdu;
    /// PHY (0: 1Mbps | 1: 2 Mbps | 2: 125 Kbps | 3: 500 Kbps)
    uint8_t                        rate;
    /// Group packing method (@see enum iso_packing)
    uint8_t                        packing;
    /// Number of pre-transmitted events
    uint8_t                        nb_pt_evt;
    /// Encryption group session key
    struct ltk                     gsk;

    /// List of BIS handles (one more for 0)
    struct lld_bis_params          bis_params[BLE_ISO_STREAM_MAX + 1];
};

/// BIG start time
struct lld_big_start_time
{
    uint32_t big_start_hs;
    uint16_t big_start_us;
};
#endif // (BLE_BIS)

/// Channel Scanning parameters structure
struct lld_ch_scan_params
{
    /// Current Channel map used for the channel scan
    struct le_ch_map ch_map;
    /// Window duration of one channel in us
    uint32_t win_duration;
    /// Channel Scan event in us
    uint32_t scan_duration;
    /// Channel Scan interval in Time = N*1.25ms
    uint16_t intv;
    /// Channel selection on which channel scan is performed  (see enum: ch_scan_ch_sel)
    uint8_t ch_sel;
};

/*
 * EXPORTED VARIABLES DECALRATION
 ****************************************************************************************
 */

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the LLD module
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void lld_init(bool is_reset);

#if (BLE_BIS | BLE_CIS)
    /**
    ****************************************************************************************
    * @brief Handle BLE "HOP ACK" interrupt
    ****************************************************************************************
    */
    void lld_iso_hop_isr(void);
#endif // (BLE_BIS | BLE_CIS)

/**
 * ***************************************************************************************
 * @brief Read BLE clock
 *
 * @return Clock value (in half-slots, i.e. units of 312.5 us)
 ****************************************************************************************
 */
uint32_t lld_read_clock(void);

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
bool lld_rxdesc_buf_ready(uint16_t em_ptr);

/**
 * ***************************************************************************************
 * @brief Renew the resolvable private addresses
 ****************************************************************************************
 */
void lld_rpa_renew(void);

/**
 ****************************************************************************************
 * @brief Add a device to white list
 *
 * @param[in] position     Position of the device in the list
 * @param[in] bd_addr      BD Address
 * @param[in] bd_addr_type BD Address type (0: public | 1: random | 0xFF: anonymous adv)
 ****************************************************************************************
 */
void lld_white_list_add(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type);

/**
 ****************************************************************************************
 * @brief Remove a device from white list
 *
 * @param[in] position     Position of the device in the list
 * @param[in] bd_addr      BD Address
 * @param[in] bd_addr_type BD Address type (0: public | 1: random | 0xFF: anonymous adv)
 ****************************************************************************************
 */
void lld_white_list_rem(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type);

/**
 ****************************************************************************************
 * @brief Add a device to periodic advertiser list
 *
 * @param[in] position     Position of the device in the list
 * @param[in] bd_addr      BD Address
 * @param[in] bd_addr_type BD Address type (0: public | 1: random)
 * @param[in] adv_sid      Advertising Set ID
 ****************************************************************************************
 */
void lld_per_adv_list_add(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type, uint8_t adv_sid);

/**
 ****************************************************************************************
 * @brief Remove a device from periodic advertiser list
 *
 * @param[in] position     Position of the device in the list
 * @param[in] bd_addr      BD Address
 * @param[in] bd_addr_type BD Address type (0: public | 1: random)
 * @param[in] adv_sids     Bit field of advertising Set IDs to remove, 1 bit per ADV_SID
 ****************************************************************************************
 */
void lld_per_adv_list_rem(uint8_t position, struct bd_addr *bd_addr, uint8_t bd_addr_type, uint16_t adv_sids);

/**
 ****************************************************************************************
 * @brief Clear resolving list
 ****************************************************************************************
 */
void lld_res_list_clear(void);

/**
 ****************************************************************************************
 * @brief Add a device to resolving list
 *
 * @param[in] position     Position in RAL
 * @param[in] bd_addr      Peer's BD Address
 * @param[in] peer_irk     Peer identification resolving key
 * @param[in] localr_irk   Local identification resolving key
 * @param[in] bd_addr_type Peer's BD Address type (0: public | 1: random)
 * @param[in] in_wl        True: device is in white list | false: not in white list
 ****************************************************************************************
 */
void lld_res_list_add(uint8_t position, struct bd_addr const *bd_addr, struct irk const *peer_irk, struct irk const *local_irk, uint8_t bd_addr_type, bool in_wl);

/**
 ****************************************************************************************
 * @brief Remove a device from resolving list
 *
 * @param[in] position     Position in RAL
 *
 * @return    status       0: success | 1-255: error
 ****************************************************************************************
 */
void lld_res_list_rem(uint8_t position);

/**
 ****************************************************************************************
 * @brief Update status of a device privacy: network privacy or device privacy
 *
 * @param[in] position     Position in RAL
 * @param[in]  privacy_mode        0: network privacy / 1: device privacy
 ****************************************************************************************
 */
void lld_res_list_priv_mode_update(uint8_t position, uint8_t privacy_mode);

/**
 ****************************************************************************************
 * @brief Get the current peer resolvable address used for a peer device
 *
 * @param[in] position     Position in RAL
 * @param[out] peer_res_addr       Peer resolvable address
 * @return     status              0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_res_list_peer_rpa_get(uint8_t position, struct bd_addr *peer_res_addr);

/**
 ****************************************************************************************
 * @brief Get the current local resolvable address used for a peer device
 *
 * @param[in] position     Position in RAL
 * @param[out] local_res_addr      Local resolvable address
 * @return     status              0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_res_list_local_rpa_get(uint8_t position, struct bd_addr *local_res_addr);


#if (BLE_BROADCASTER)
    /**
    ****************************************************************************************
    * @brief Start Advertising
    *
    * @param[in]  act_id        Activity identifier
    * @param[in]  params        Pointer to extended advertising parameters structure
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_adv_start(uint8_t act_id, struct lld_adv_params *params);

    /**
    ****************************************************************************************
    * @brief Stop Advertising
    *
    * @param[in]  act_id        Activity identifier
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_adv_stop(uint8_t act_id);

    /**
    ****************************************************************************************
    * @brief Update Advertising data
    *
    * Note: the Host can only update the advertising data less than 252 bytes (see HCI:7.8.54)
    *
    * @param[in]  act_id     Activity identifier
    * @param[in]  len        Length of the data, in range [0:252]
    * @param[in]  buffer     EM buffer containing the data
    ****************************************************************************************
    */
    void lld_adv_adv_data_update(uint8_t act_id, uint8_t len, uint16_t buffer);

    /**
    ****************************************************************************************
    * @brief Update advertising duration
    *
    * @param[in]  act_id        Activity identifier
    * @param[in]  duration      Advertising duration (N * 10 ms) | 0x0000: infinite
    ****************************************************************************************
    */
    void lld_adv_duration_update(uint8_t act_id, uint16_t duration);

    /**
    ****************************************************************************************
    * @brief Update advertising random address
    *
    * @param[in]  act_id        Activity identifier
    * @param[in]  addr          Random address
    ****************************************************************************************
    */
    void lld_adv_rand_addr_update(uint8_t act_id, struct bd_addr addr);

    /**
    ****************************************************************************************
    * @brief Reset the event count, update the duration and the maximum number of extended advertising events
    *
    * @param[in]  act_id           Activity identifier
    * @param[in]  duration         Advertising duration (N * 10 ms) | 0x0000: infinite
    * @param[in]  max_ext_adv_evt  Maximum number of extended advertising events
    ****************************************************************************************
    */
    void lld_adv_restart(uint8_t act_id, uint16_t duration, uint8_t max_ext_adv_evt);

    /**
    ****************************************************************************************
    * @brief Update Scan response data
    *
    * Note: the Host may can only update the advertising data less than 252 bytes (see HCI:7.8.55)
    *
    * @param[in]  act_id        Activity identifier
    * @param[in]  len        Length of the data, in range [0:252]
    * @param[in]  buffer     EM buffer containing the data
    ****************************************************************************************
    */
    void lld_adv_scan_rsp_data_update(uint8_t act_id, uint8_t len, uint16_t buffer);

    /**
    ****************************************************************************************
    * @brief Read the TX power used for advertising
    *
    * @param[in]  act_id        Activity identifier
    *
    * @return Used TX power in dBm, signed integer, range between -70dBm and 20dBm.
    ****************************************************************************************
    */
    int8_t lld_adv_tx_pwr_get(uint8_t act_id);

    #if (BT_53)
        /**
        ****************************************************************************************
        * @brief Update the reasons for refreshing RPAs
        *
        * @param[in]  act_id               Extended advertising activity identifier
        * @param[in]  chg_reasons          Reasons for refreshing RPAs
        ****************************************************************************************
        */
        void lld_adv_chg_reasons_update(uint8_t act_id, uint8_t chg_reasons);
    #endif // (BT_53)

    #if (EAVESDROPPING_SUPPORT)
        /**
        ****************************************************************************************
        * @brief Advertising trigger interrupt handler
        ****************************************************************************************
        */
        void lld_adv_trig_isr(void);
    #endif // EAVESDROPPING_SUPPORT

    /**
    ****************************************************************************************
    * @brief Start Periodic advertising
    *
    * @param[in]  act_id        Activity identifier
    * @param[in]  params        Pointer to extended advertising parameters structure
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_per_adv_start(uint8_t act_id, struct lld_per_adv_params *params);

    /**
    ****************************************************************************************
    * @brief Stop Periodic advertising
    *
    * @param[in]  act_id        Activity identifier
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_per_adv_stop(uint8_t act_id);

    /**
    ****************************************************************************************
    * @brief Update Periodic advertising data
    *
    * Note: the Host may can only update the advertising data less than 252 bytes (see HCI:7.8.54)
    *
    * @param[in]  act_id     Activity identifier
    * @param[in]  len        Length of the data, in range [0:252]
    * @param[in]  buffer     EM buffer containing the data
    ****************************************************************************************
    */
    void lld_per_adv_data_update(uint8_t act_id, uint8_t len, uint16_t buffer);

    #if (BT_53)
        /**
        ****************************************************************************************
        * @brief Update the periodic advertising DID
        *
        * @param[in]  act_id        Activity identifier
        ****************************************************************************************
        */
        void lld_per_adv_did_update(uint8_t act_id);
    #endif // (BT_53)

    /**
    ****************************************************************************************
    * @brief Initiate periodic advertising channel map update
    *
    * @param[in]  act_id        Activity ID
    * @param[in]  map           New channel map to be applied
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_per_adv_ch_map_update(uint8_t act_id, struct le_ch_map *p_ch_map);

    #if (BLE_PAST)
        /**
        ****************************************************************************************
        * @brief Get info from a periodic advertiser
        *
        * @param[in]  act_id        Activity ID
        * @param[out] phy           Periodic advertising PHY (@see enum lld_rate)
        * @param[out] intv          Periodic advertising interval in units of 1.25 ms
        * @param[out] aa            Periodic advertising Access Address
        * @param[out] crcinit       Periodic advertising CRC Initialization value
        * @param[out] sync_ind_ts   Timestamp of the AUX_SYNC_IND packet (in 312.5 us half-slots)
        * @param[out] pa_evt_cnt    Corresponding paEventCounter value (in half-us)
        * @param[out] map           Corresponding channel map
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t lld_per_adv_info_get(uint8_t act_id, uint8_t *phy, uint16_t *intv, struct access_addr *aa, struct crc_init *crcinit, uint32_t *sync_ind_ts, uint16_t *pa_evt_cnt, struct le_ch_map *p_ch_map);
    #endif // (BLE_PAST)

    #if (BLE_BIS)
        /**
        ****************************************************************************************
        * @brief Update Periodic advertising with specified BIG identifier.
        *
        * This function is used to start or stop advertising BIG Info in ACAD data.
        *
        * @param[in]  act_id     Activity identifier
        * @param[in]  big_id     BIG identifier, if invalid identifier, stop advertising BIG Info
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t lld_per_adv_big_update(uint8_t act_id, uint8_t big_id);
    #endif // (BLE_BIS)

    /**
    ****************************************************************************************
    * @brief Set the secondary advertising channel map
    *
    * @param[in]  p_ch_map           New channel map for secondary advertising
    ****************************************************************************************
    */
    void lld_sec_adv_ch_map_set(const struct le_ch_map *p_ch_map);

    /**
    ****************************************************************************************
    * @brief Update extended advertising sync info
    *
    * @param[in]  act_id               Extended advertising activity identifier
    * @param[in]  per_adv_id           Periodic advertising activity identifier
    * @param[in]  per_adv_interval     Periodic advertising interval
    ****************************************************************************************
    */
    void lld_adv_sync_info_update(uint8_t act_id, uint8_t per_adv_id, uint16_t per_adv_interval);

    #if BLE_CONLESS_CTE_TX
        /**
        ****************************************************************************************
        * @brief Start Periodic advertising CTE Tx
        *
        * @param[in]  act_id        Activity identifier
        * @param[in]  params        Pointer to periodic advertising CTE parameters structure
        *
        * @return status            0: success | 1-255: error (BLE error code)
        ****************************************************************************************
        */
        uint8_t lld_per_adv_cte_start(uint8_t act_id, struct lld_per_adv_cte_params *params);

        /**
        ****************************************************************************************
        * @brief Stop Periodic advertising CTE Tx
        *
        * @param[in]  act_id        Activity identifier
        *
        * @return status            0: success | 1-255: error (BLE error code)
        ****************************************************************************************
        */
        uint8_t lld_per_adv_cte_stop(uint8_t act_id);
    #endif // BLE_CONLESS_CTE_TX
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
    /**
    ****************************************************************************************
    * @brief Start Scanning
    *
    * @param[in]  act_id        Activitiy ID
    * @param[in]  params        Pointer to Scanning parameters structure
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_scan_start(uint8_t act_id, struct lld_scan_params *params);

    /**
    ****************************************************************************************
    * @brief Stop Scanning
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_scan_stop(void);

    #if (EAVESDROPPING_SUPPORT)
        /**
        ****************************************************************************************
        * @brief Configure/Start Segmented Scanning
        *
        * @param[in]  params        Pointer to Segmented Scanning parameters structure
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t lld_scan_seg_start(struct lld_scan_seg_params *params);

        /**
        ****************************************************************************************
        * @brief Stop Segmented Scanning
        *
        * @return status            0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t lld_scan_seg_stop(void);
    #endif // (EAVESDROPPING_SUPPORT)

    /**
    ****************************************************************************************
    * @brief Update scanning parameters
    *
    * @param[in] duration  Scan duration (N * 10 ms)  | 0x0000: Scan continuously until explicitly disable
    * @param[in] bd_addr   new random address | NULL: no update
    ****************************************************************************************
    */
    uint8_t lld_scan_params_update(uint16_t duration, struct bd_addr *bd_addr);

    /**
    ****************************************************************************************
    * @brief Start Periodic Advertising RX (Create Sync)
    *
    * @param[in] act_id            Activity ID
    * @param[in] options           Sync options (@see enum per_sync_opt)
    * @param[in] adv_sid           Advertising SID
    * @param[in] adv_addr_type     Advertising address type
    * @param[in] adv_addr          Advertiser address
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_scan_create_sync(uint8_t act_id, uint8_t options, uint8_t adv_sid, uint8_t adv_addr_type, struct bd_addr *adv_addr);

    /**
    ****************************************************************************************
    * @brief Stop Create Sync for Periodic Scan Rx (Cancel Sync)
    *
    * @param[in] act_id            Activity ID
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_scan_create_sync_cancel(uint8_t act_id);

    /**
    ****************************************************************************************
    * @brief Start Periodic Advertising Rx
    *
    * @param[in] act_id            Activity ID
    * @param[in] params            Pointer to Scanning parameters structure
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_sync_start(uint8_t act_id, struct lld_sync_params *params);

    /**
    ****************************************************************************************
    * @brief Update channel map on a periodic advertiser
    *
    * @param[in] act_id     Activity ID
    * @param[in] map        Pointer to the new channel map to be applied
    * @param[in] instant    Instant when the new channel map has to be applied
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_sync_ch_map_update(uint8_t act_id, struct le_ch_map *p_ch_map, uint16_t instant);

    /**
    ****************************************************************************************
    * @brief ADI Filtering enable
    *
    * @param[in] act_id            Activity ID
    * @param[in] en                Enable (@see enum per_adv_rec_en)
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    bool lld_sync_rep_filt_en(uint8_t act_id, uint8_t en);

    /**
    ****************************************************************************************
    * @brief Stop Periodic Advertising RX (Terminate Sync)
    *
    * @param[in] act_id            Activity ID
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_sync_stop(uint8_t act_id);

    #if(BLE_HOST_PRESENT)
        /**
        ****************************************************************************************
        * @brief Update the scan random address to use
        ****************************************************************************************
        */
        void lld_scan_rand_addr_update(const struct bd_addr *p_addr);
    #endif //(BLE_HOST_PRESENT)

    #if BLE_CONLESS_CTE_RX
        /**
        ****************************************************************************************
        * @brief Start Periodic advertising IQ sampling
        *
        * @param[in]  act_id        Activity identifier
        * @param[in]  params        Pointer to periodic advertising CTE parameters structure
        *
        * @return status            0: success | 1-255: error (BLE error code)
        ****************************************************************************************
        */
        uint8_t lld_sync_cte_start(uint8_t act_id, struct lld_sync_cte_params *params);

        /**
        ****************************************************************************************
        * @brief Stop Periodic advertising IQ sampling
        *
        * @param[in]  act_id        Activity identifier
        *
        * @return status            0: success | 1-255: error (BLE error code)
        ****************************************************************************************
        */
        uint8_t lld_sync_cte_stop(uint8_t act_id);
    #endif // BLE_CONLESS_CTE_RX

#endif // (BLE_OBSERVER)



#if (BLE_CENTRAL)
    /**
    ****************************************************************************************
    * @brief Start Initiating
    *
    * @param[in]  params        Pointer to Initiating parameters structure
    *
    * @return status            0: success | 1-255: error (BLE error code)
    ****************************************************************************************
    */
    uint8_t lld_init_start(struct lld_init_params *params);

    /**
    ****************************************************************************************
    * @brief Stop Initiating
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_init_stop(void);

    #if(BLE_HOST_PRESENT)
        /**
        ****************************************************************************************
        * @brief Update the initiating Random address to use
        ****************************************************************************************
        */
        void lld_init_rand_addr_update(const struct bd_addr *p_addr);
    #endif //(BLE_HOST_PRESENT)

#endif // (BLE_CENTRAL)


#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Stop link
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  immediate     Immediate execution of the event stop
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_stop(uint8_t link_id, bool immediate);

/**
 ****************************************************************************************
 * @brief Send an LLCP packet
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  buf_elt       Buffer element of the LLCP to send
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_llcp_tx(uint8_t link_id, struct ble_em_llcp_buf_elt *buf_elt);

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
uint8_t lld_con_data_tx(uint8_t link_id, struct ble_em_acl_buf_elt *buf_elt);

/**
 ****************************************************************************************
 * @brief Set ACL data flow
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  enable        True: data flow ON / False: data flow OFF
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_data_flow_set(uint8_t link_id, bool enable);

/**
 ****************************************************************************************
 * @brief Update link parameters
 *
 * @param[in]  link_id    Link identifier
 * @param[in]  win_size   Window size (in units of 1,25 ms, i.e. 2 slots)
 * @param[in]  win_off    Window offset (in units of 1,25 ms, i.e. 2 slots)
 * @param[in]  interval   Interval (in units of 1,25 ms, i.e. 2 slots)
 * @param[in]  latency    Slave latency
 * @param[in]  timeout    Timeout (in units of 10 ms, i.e. 16 slots)
 * @param[in]  instant    Instant when the new parameters have to be applied (when connection event counter equals the instant)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_param_update(uint8_t link_id, uint8_t win_size, uint16_t win_off, uint16_t interval, uint16_t latency,
                             uint16_t timeout, uint16_t instant);

#if (BT_53)
/**
 ****************************************************************************************
 * @brief Update link subrate
 *
 * @param[in]  link_id    Link identifier
 * @param[in]  subrate    Subrate Factor (unit of connection event)
 * @param[in]  base_evt   Subrate Base event (unit of connection event)
 * @param[in]  latency    Peripheral Latency in subrated events (unit of connection event)
 * @param[in]  cont_num   Continuation number (unit of connection event)
 * @param[in]  timeout    Supervision Timeout (in units of 10 ms, i.e. 16 slots)
 * @param[in]  t_mode     Indicates if for Transition mode (true = transition mode)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_subrate(uint8_t link_id, uint16_t subrate, uint16_t base_evt, uint16_t latency,
                        uint16_t cont_num, uint16_t timeout, bool t_mode);
#endif // (BT_53)

/**
 ****************************************************************************************
 * @brief Update channel map
 *
 * @param[in] link_id    Link identifier
 * @param[in] p_ch_map   Pointer to the new channel map to be applied
 * @param[in] instant    Instant when the new channel map has to be applied (when connection event counter equals the instant)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_ch_map_update(uint8_t link_id, struct le_ch_map *p_ch_map, uint16_t instant);

/**
 ****************************************************************************************
 * @brief Update data length
 *
 * @param[in]  link_id          Link identifier
 * @param[in]  eff_tx_time      Maximum effective transmit time (in us)
 * @param[in]  eff_tx_octets    Maximum effective transmit size (in bytes)
 * @param[in]  eff_rx_time      Maximum effective receive time (in us)
 * @param[in]  eff_rx_octets    Maximum effective receive size (in bytes)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_data_len_update(uint8_t link_id, uint16_t eff_tx_time, uint16_t eff_tx_octets, uint16_t eff_rx_time, uint16_t eff_rx_octets);

/**
 ****************************************************************************************
 * @brief Update PHYs
 *
 * @param[in] link_id   Link identifier
 * @param[in] tx_rate   Transmit rate to be set  (@see enum lld_rate)
 * @param[in] rx_rate   Receive rate to be set   (@see enum lld_rate)
 * @param[in] instant   Instant where the new rates have to be applied (when connection event counter equals the instant)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_phys_update(uint8_t link_id, uint8_t tx_rate, uint8_t rx_rate, uint16_t instant);

/**
 ****************************************************************************************
 * @brief Update TX length as per the new connection interval
 *
 * @param[in] link_id   Link identifier
 * @param[in] interval   Interval (in units of 1,25 ms, i.e. 2 slots)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_tx_len_update_for_intv(uint8_t link_id, uint16_t interval);

/**
 ****************************************************************************************
 * @brief Update TX length as per the new TX rate
 *
 * @param[in] link_id   Link identifier
 * @param[in] tx_rate   Transmit rate to be set  (@see enum lld_rate)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_tx_len_update_for_rate(uint8_t link_id, uint8_t tx_rate);

/**
 ****************************************************************************************
 * @brief Gets current event counter
 *
 * @param[in]       link_id      Link identifier
 *
 * @return Current event counter value
 ****************************************************************************************
 */
uint16_t lld_con_event_counter_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set encryption mode in TX direction
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  enable        false: disabled | true: enabled
 ****************************************************************************************
 */
void lld_con_tx_enc(uint8_t link_id, bool enable);

/**
 ****************************************************************************************
 * @brief Set encryption mode in RX direction
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  enable        false: disabled | true: enabled
 ****************************************************************************************
 */
void lld_con_rx_enc(uint8_t link_id, bool enable);

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
void lld_con_enc_key_load(uint8_t link_id, struct ltk *key, struct initialization_vector *iv);

/**
 ****************************************************************************************
 * @brief Get the current TX power
 *
 * @param[in]  link_id       Link identifier
 *
 * @return Current radiative TX power in dBm
 ****************************************************************************************
 */
int8_t lld_con_current_tx_power_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Get the received RSSI
 *
 * @param[in]  link_id       Link identifier
 *
 * @return     rssi          RSSI in dBm
 ****************************************************************************************
 */
int8_t lld_con_rssi_get(uint8_t link_id);

#if BLE_PWR_CTRL

/**
 ****************************************************************************************
 * @brief Get the current TX power
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  phy           PHY (@see enum le_phy_pwr)
 *
 * @return Current radiative TX power in dBm
 ****************************************************************************************
 */
int8_t lld_con_tx_power_get(uint8_t link_id, uint8_t phy);

/**
****************************************************************************************
* @brief Adjust the current TX power
*
* @param[in]  link_id       Link identifier
* @param[in]  phy           PHY (@see enum le_phy_pwr)
* @param[in]  delta         Delta in the transmit power (in dBm)
*
* @return Radiative TX power after adjustment in dBm
****************************************************************************************
*/
int8_t lld_con_tx_power_adj(uint8_t link_id, uint8_t phy, int8_t delta);

/**
 ****************************************************************************************
 * @brief Get the APR (acceptable power reduction) on a specific rate
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  rx_rate       Rx rate @see enum lld_rate
 *
 * @return     APR           Acceptable power reduction (dB).
 ****************************************************************************************
 */
uint8_t lld_con_apr_get(uint8_t link_id, uint8_t rx_rate);

/**
 ****************************************************************************************
 * @brief Get the active Rx Rate
 *
 * @param[in]  link_id       Link identifier
 *
 * @return     rx_rate       Rx rate @see enum lld_rate
 ****************************************************************************************
 */
uint8_t lld_con_rx_rate_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Get the active Tx Rate
 *
 * @param[in]  link_id       Link identifier
 * @param[out] tx_rate       Tx rate @see enum lld_rate
 *
 * @return     status        Error status
 ****************************************************************************************
 */
uint8_t lld_con_tx_rate_get(uint8_t link_id, uint8_t *tx_rate);

/**
 ****************************************************************************************
 * @brief Set the remote transmit power information for path loss monitoring
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  rx_rate       Rx rate @see enum phy_rate
 * @param[in]  tx_pwr        Transmit power in dBm
 * @param[in]  flags         Transmit power flags, or LLD_PWR_FLGS_UNKNOWN @see enum pwr_ctrl_flags
 *
 ****************************************************************************************
 */
void lld_con_remote_tx_pwr_set(uint8_t link_id, uint8_t rx_rate, int8_t tx_pwr, uint8_t flags);

/**
 ****************************************************************************************
 * @brief Get the remote transmit power information in use by the connection driver
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  rx_rate       Rx rate @see enum lld_rate
 * @param[out]  flags        Transmit power flags, or LLD_PWR_FLGS_UNKNOWN @see enum pwr_ctrl_flags
 *
 * @return  tx_pwr           Transmit power in dBm, or BLE_PWR_UNKNOWN
 ****************************************************************************************
 */
int8_t lld_con_remote_tx_pwr_get(uint8_t link_id, uint8_t rx_rate, uint8_t *flags);

/**
 ****************************************************************************************
 * @brief Path loss monitor configuration
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  hi_thr        High threshold (dBm)
 * @param[in]  hi_hyst       High hysteresis (dBm)
 * @param[in]  lo_thr        Low threshold (dBm)
 * @param[in]  lo_hyst       Low hysteresis (dBm)
 * @param[in]  min_evt       Minimum duration (connection events)
 *
 ****************************************************************************************
 */
void lld_con_path_loss_monitor_config(uint8_t link_id, uint8_t hi_thr, uint8_t hi_hyst,
                                      uint8_t lo_thr, uint8_t lo_hyst, uint16_t min_evt);

/**
 ****************************************************************************************
 * @brief Path loss monitor configuration
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  en            Enable (true) /Disable (false).
 *
 * @return     active        True if activated, false if remote transmit power unknown.
 ****************************************************************************************
 */
bool lld_con_path_loss_monitor_en(uint8_t link_id, bool en);

#endif // BLE_PWR_CTRL

/**
 ****************************************************************************************
 * @brief Get the connection offset
 *
 * @param[in]  link_id       Link identifier
 *
 * @return     con_offset    Connection offset in slots (625 us)
 ****************************************************************************************
 */
uint16_t lld_con_offset_get(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief Set peer SCA
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  peer_sca      Peer SCA (@see enum SCA)
 ****************************************************************************************
 */
void lld_con_peer_sca_set(uint8_t link_id, uint8_t peer_sca);

/**
 ****************************************************************************************
 * @brief Get connection event time
 *
 * @param[in]  link_id               Link identifier
 * @param[out] con_evt_cnt           Connection event counter
 * @param[out] con_evt_time_hs       Time of the connection event, in 312.5 us half-slots
 * @param[out] con_evt_time_hus      Time of the connection event, in half-us
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_time_get(uint8_t link_id, uint16_t *con_evt_cnt, uint32_t *con_evt_time_hs, uint16_t *con_evt_time_hus);

#if (AUDIO_SYNC_SUPPORT)
    /**
    ****************************************************************************************
    * @brief Get connection event air time for a specific event on master side
    *
    * @param[in]  link_id               Link identifier
    * @param[in] con_evt_cnt            Connection event count
    * @param[out] con_evt_time_hs       Time of the connection event, in 312.5 us half-slots
    * @param[out] con_evt_time_hus      Time of the connection event, in half-us
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_con_event_tx_time_get(uint8_t link_id, uint16_t con_evt_cnt, uint32_t *con_evt_time_hs, uint16_t *con_evt_time_hus);
#endif //(AUDIO_SYNC_SUPPORT)

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_PERIPHERAL)
    /**
    ****************************************************************************************
    * @brief Set the preferred slave latency
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  latency       Preferred slave latency (in number of connection events)
    ****************************************************************************************
    */
    void lld_con_pref_slave_latency_set(uint8_t link_id, uint16_t latency);

    /**
    ****************************************************************************************
    * @brief Set the preferred slave event duration
    *
    * @param[in]  link_id       Link identifier
    * @param[in]  Duration      Preferred slave event duration (N * 0.625 ms)
    ****************************************************************************************
    */
    void lld_con_pref_slave_evt_dur_set(uint8_t link_id, uint16_t duration, bool single_tx);
#endif // (BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Set the priority elevation for this link
 *
 * @param[in]  link_id       Link identifier
 * @param[in]  elevation     Arbitration priority elevation (in units of increment)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_con_prio_elevation_set(uint8_t link_id, uint8_t elevation);

#if (BLE_ISO_MODE_0)

    /**
    ****************************************************************************************
    * @brief Inform that Isochronous mode is set-up and mic less encryption is required
    *
    * @param[in]  link_id        Link identifier
    *
    * @return Executions status
    ****************************************************************************************
    */
    uint8_t lld_con_am0_use_mic_less(uint8_t link_id);

    /**
    ****************************************************************************************
    * @brief Start audio
    *
    * @param[in]  link_id        Link identifier
    * @param[in]  iso_channel    Isochronous channel
    * @param[in]  params         Configuration parameters
    *
    * @return Executions status
    ****************************************************************************************
    */
    uint8_t lld_con_am0_start(uint8_t link_id, uint8_t iso_channel, struct lld_con_am0_params *params);

    /**
    ****************************************************************************************
    * @brief Stop audio
    *
    * @param[in]  link_id        Link identifier
    *
    * @return Excecutions status
    ****************************************************************************************
    */
    uint8_t lld_con_am0_stop(uint8_t link_id);

#endif //(BLE_ISO_MODE_0)

#if BLE_CON_CTE_REQ
    /**
    ****************************************************************************************
    * @brief Enable reception of CTE on connection
    *
    * @param[in] link_id                 Link identifier
    * @param[in] slot_dur                Slot durations (1: 1us | 2: 2us)
    * @param[in] switching_pattern_len   Length of switching pattern (number of antenna IDs in the pattern)
    *****************************************************************************************
    */
    void lld_con_cte_rx_en(uint8_t link_id, uint8_t slot_dur, uint8_t switching_pattern_len);

    /**
    ****************************************************************************************
    * @brief Disable reception of CTE on connection
    *
    * @param[in] link_id       Link identifier
    *****************************************************************************************
    */
    void lld_con_cte_rx_dis(uint8_t link_id);
#endif // BLE_CON_CTE_REQ

#if BLE_CON_CTE_RSP
    /**
    ****************************************************************************************
    * @brief Configure antenna switching for connection CTE Tx
    *
    * @param[in] link_id                 Link identifier
    * @param[in] switching_pattern_len   Length of switching pattern (number of antenna IDs in the pattern)
    * @param[in] cte_types               CTE types (bit0: AOA | bit1: AOD-1us | bit2: AOD-2us)
    *****************************************************************************************
    */
    void lld_con_cte_tx_ant_switch_config(uint8_t link_id, uint8_t switching_pattern_len, uint8_t cte_types);

    /**
    ****************************************************************************************
    * @brief Set connection CTE Tx parameters
    *
    * @param[in] link_id                 Link identifier
    * @param[in] cte_len                 CTE length in 8 us units (0: No CTE | Range: 0x02 - 0x14)
    * @param[in] cte_type                CTE type (0x00: AOA | 0x01: AOD-1us | 0x02: AOD-2us)
    *****************************************************************************************
    */
    void lld_con_cte_tx_param_set(uint8_t link_id, uint8_t cte_len, uint8_t cte_type);
#endif // BLE_CON_CTE_RSP

/**
 ****************************************************************************************
 * @brief Start test mode (RX or TX)
 *
 * @param[in]  params        Pointer to Scanning parameters structure
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_test_start(struct lld_test_params *params);

/**
 ****************************************************************************************
 * @brief Stop test mode
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_test_stop(void);

#if (RW_DEBUG)
    /**
    ****************************************************************************************
    * @brief Read BLE core register
    *
    * @param[in]   reg_addr     Register address
    *
    * @return reg_value         Register value
    ****************************************************************************************
    */
    uint16_t lld_reg_rd(uint16_t reg_addr);

    /**
    ****************************************************************************************
    * @brief Write BLE core register
    *
    * @param[in]   reg_addr     Register address
    * @param[in]   reg_value    Register value
    ****************************************************************************************
    */
    void lld_reg_wr(uint16_t reg_addr, uint16_t reg_value);
#endif // (RW_DEBUG)

#if (BLE_PAST)
    /**
    ****************************************************************************************
    * @brief Get sync info from a periodic sync activity
    *
    * @param[in]  act_id              Activity ID
    * @param[out] phy                 Periodic advertising PHY (@see enum lld_rate)
    * @param[out] intv                Periodic advertising interval in units of 1.25 ms
    * @param[out] aa                  Periodic advertising Access Address
    * @param[out] crcinit             Periodic advertising CRC Initialization value
    * @param[out] sync_ind_ts         Timestamp of the AUX_SYNC_IND packet (in 312.5 us half-slots)
    * @param[out] sync_ind_bit_off    Bit offset of the AUX_SYNC_IND packet (in half-us)
    * @param[out] pa_evt_cnt          Corresponding paEventCounter value
    * @param[out] p_ch_map            Corresponding channel map
    * @param[out] sca                 Periodic advertising Sleep Clock Accuracy
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    uint8_t lld_sync_info_get(uint8_t act_id, uint8_t *phy, uint16_t *intv, struct access_addr *aa, struct crc_init *crcinit, uint32_t *sync_ind_ts, uint16_t *sync_ind_bit_off, uint16_t *pa_evt_cnt, struct le_ch_map *p_ch_map, uint8_t *sca);
#endif // (BLE_PAST)

#if (BLE_CIS)
/**
 ****************************************************************************************
 * @brief Start Connected Isochronous Stream activity
 *
 * @param[in]  act_id           Activity identifier
 * @param[in]  p_cis_params     Pointer to CIS parameters structure
 * @param[in]  p_cis_start_hs   Pointer to CIS expected start instant, half slot part
 * @param[in]  p_cis_start_hus  Pointer to CIS expected start instant, half microseconds part
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_cis_start(uint8_t act_id, struct lld_cis_params *p_cis_params,
                      uint32_t *p_cis_start_hs, uint16_t *p_cis_start_hus);

/**
 ****************************************************************************************
 * @brief Stop Connected Isochronous Stream activity
 *
 * @param[in]  act_id   Activity identifier
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_cis_stop(uint8_t act_id);

/**
 ****************************************************************************************
 * @brief Enable synchronization of the data path to local timings
 *
 * @param[in] act_id         Activity identifier
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] cb_local_sync  Callback for local synchronization
 ****************************************************************************************
 */
uint8_t lld_cis_local_sync_en(uint8_t act_id, uint8_t direction, data_path_local_sync_cb cb_local_sync);

/**
 ****************************************************************************************
 * @brief Disable synchronization of the data path to local timings
 *
 * @param[in] act_id         Activity identifier
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] cb_local_sync  Callback for local synchronization
 ****************************************************************************************
 */
uint8_t lld_cis_local_sync_dis(uint8_t act_id, uint8_t direction, data_path_local_sync_cb cb_local_sync);


#if (BLE_PERIPHERAL)
    /**
    ****************************************************************************************
    * @brief Enable synchronization of the data path to master's BT timings
    *
    * @param[in] act_id         Activity identifier
    * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
    * @param[in] cb_peer_sync   Callback for peer synchronization
    ****************************************************************************************
    */
    uint8_t lld_cis_peer_sync_en(uint8_t act_id, uint8_t direction, data_path_peer_sync_cb cb_peer_sync);

    /**
    ****************************************************************************************
    * @brief Disable synchronization of the data path to master's BT timings
    *
    * @param[in] act_id         Activity identifier
    * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
    * @param[in] cb_peer_sync   Callback for peer synchronization
    ****************************************************************************************
    */
    uint8_t lld_cis_peer_sync_dis(uint8_t act_id, uint8_t direction, data_path_peer_sync_cb cb_peer_sync);
#endif // (BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Generating the Access Address for a CIS Channel
 *
 * TODO [FBE] add more information about access address generation
 *
 * @param[in]  act_id     Reference activity identifier
 * @param[out] p_acc_addr Pointer to the 4 byte container for the AA.
 *
 *****************************************************************************************
 */
void lld_cis_aa_gen(uint8_t act_id, uint32_t *p_acc_addr);

/**
 ****************************************************************************************
 * @brief Read CIS statistics
 *
 * @param[in]  act_id                   Reference activity identifier
 * @param[out] tx_unacked_packets       Pointer to number of Tx unacked packets
 * @param[out] tx_flushed_packets       Pointer to number of Tx flushed packets
 * @param[out] tx_last_subevent_packets Pointer to number of Tx last subevent packets
 * @param[out] retransmitted_packets    Pointer to number of retransmitted packets
 * @param[out] crc_error_packets        Pointer to number of CRC error packets
 * @param[out] rx_unreceived_packets    Pointer to number of Rx unreceived packets
 * @param[out] duplicate_packets        Pointer to number of duplicate packets
 *
 * @return status of the executed procedure (@see enum co_error)
 ****************************************************************************************
 */
uint8_t lld_cis_stats_get(uint8_t act_id, uint32_t *tx_unacked_packets, uint32_t *tx_flushed_packets, uint32_t *tx_last_subevent_packets,
                          uint32_t *retransmitted_packets, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets,
                          uint32_t *duplicate_packets);

/**
 ****************************************************************************************
 * @brief Get CIS payload counter
 *
 * @param[in]  act_id    Reference activity identifier
 * @param[in]  direction Data direction (@see enum iso_rx_tx_select)
 * @param[out] pld_cnt   Pointer to the payload counter
 *
 * @return status of the executed procedure (@see enum co_error)
 ****************************************************************************************
 */
uint8_t lld_cis_pld_cnt_get(uint8_t act_id, uint8_t direction, uint32_t *pld_cnt);

/**
 ****************************************************************************************
 * @brief Get the received RSSI
 *
 * @param[in]  act_id       Activity identifier
 *
 * @return     rssi          RSSI in dBm
 ****************************************************************************************
 */
int8_t lld_cis_rssi_get(uint8_t act_id);
#endif //  (BLE_CIS)

#if (BLE_BIS)
/**
****************************************************************************************
* @brief Start Broadcast Isochronous Group.
*
* @param[in] p_big_params    Pointer to BIG parameters
* @param[in] p_big_info      Pointer to BIG info
* @param[in] p_big_info_data Pointer to BIG info packed data (for broadcaster only)
*
* @return CO_ERROR_NO_ERROR if BIG has been started properly, else error code indicating
* the reason why the BIG has not been started.
*****************************************************************************************
*/
uint8_t lld_big_start(struct lld_big_params *p_big_params, struct big_info *p_big_info, const uint8_t *p_big_info_data,
                      struct lld_big_start_time *p_big_start_time);

/**
****************************************************************************************
* @brief Stop a previously started BIG.
*
* @param[in] grp_hdl        Group handle of BIG to be stopped.
* @param[in] reason         reason of driver termination
*****************************************************************************************
*/
uint8_t lld_big_stop(uint8_t grp_hdl, uint8_t reason);

/**
****************************************************************************************
* @brief Enable synchronization of the data path to local timings
*
* @param[in] act_id         Activity identifier
* @param[in] cb_local_sync  Callback for local synchronization
****************************************************************************************
*/
uint8_t lld_bis_local_sync_en(uint8_t act_id, data_path_local_sync_cb cb_local_sync);

/**
****************************************************************************************
* @brief Disable synchronization of the data path to local timings
*
* @param[in] act_id         Activity identifier
* @param[in] cb_local_sync  Callback for local synchronization
****************************************************************************************
*/
uint8_t lld_bis_local_sync_dis(uint8_t act_id, data_path_local_sync_cb cb_local_sync);

#if (BLE_OBSERVER)
    /**
    ****************************************************************************************
    * @brief Enable synchronization of the data path to transmitter's BT timings
    *
    * @param[in] act_id         Activity identifier
    * @param[in] cb_peer_sync   Callback for peer synchronization
    ****************************************************************************************
    */
    uint8_t lld_bis_peer_sync_en(uint8_t act_id, data_path_peer_sync_cb cb_peer_sync);

    /**
    ****************************************************************************************
    * @brief Disable synchronization of the data path to transmitter's BT timings
    *
    * @param[in] act_id         Activity identifier
    * @param[in] cb_peer_sync   Callback for peer synchronization
    ****************************************************************************************
    */
    uint8_t lld_bis_peer_sync_dis(uint8_t act_id, data_path_peer_sync_cb cb_peer_sync);
#endif // (BLE_OBSERVER)

/**
****************************************************************************************
* @brief Retrive BIG event counter
*
* @param[in] grp_hdl       BIG handle for which event counter should be retrieved
*****************************************************************************************
*/
uint16_t lld_big_event_counter_get(uint8_t grp_hdl);

/**
****************************************************************************************
* @brief Request to update channel map for a given BIG
*
* @param[in] grp_hdl       BIG handle for which channel map has to be updated
* @param[in] p_map         Pointer to the new channel map to be used
* @param[in] instant       Event counter from which the new channel map must be applied
*****************************************************************************************
*/
uint8_t lld_big_ch_map_update(uint8_t grp_hdl, struct le_ch_map *p_ch_map, uint16_t instant);

#if (BLE_BROADCASTER)
    /**
    ****************************************************************************************
    * @brief Request to transmit a control PDU (channel map update and terminate indication).
    * The TX ISO buffer for the channel 0 of the indicated BIG must have been filled.
    *
    * @param[in] grp_hdl       BIG handle on which control PDU has to be sent.
    * @param[in] em_ptr        Pointer in exchange memory of the BIS Control PDU
    * @param[in] length        Size of the PDU to transmit
    * @param[in] nb_tx         Number of time the control packet should be transmitted
    *****************************************************************************************
    */
    uint8_t lld_big_cntl_tx(uint8_t grp_hdl, uint16_t em_ptr, uint8_t length, uint8_t nb_tx);
#endif //(BLE_BROADCASTER)

/**
****************************************************************************************
* @brief Read BIS statistics
*
* @param[in]  act_id                   Reference activity identifier
* @param[out] crc_error_packets        Pointer to number of CRC error packets
* @param[out] rx_unreceived_packets    Pointer to number of Rx unreceived packets
*
* @return status of the executed procedure (@see enum co_error)
****************************************************************************************
*/
uint8_t lld_bis_stats_get(uint8_t act_id, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets);

/**
****************************************************************************************
* @brief Get BIS payload counter
*
* @param[in]  act_id    Reference activity identifier
* @param[out] pld_cnt   Pointer to the payload counter
*
* @return status of the executed procedure (@see enum co_error)
****************************************************************************************
*/
uint8_t lld_bis_pld_cnt_get(uint8_t act_id, uint32_t *pld_cnt);
#endif // (BLE_BIS)

#if (BLE_ISO_PRESENT)

    /**
    ****************************************************************************************
    * @brief Remove/Stop the current active isochronous data path
    *
    * @param[in]  act_id   Activity identifier
    * @param[in]  direction Data Path direction (@see enum iso_rx_tx_select)
    *
    * @return CO_ERROR_NO_ERROR if succeed, error code else
    ****************************************************************************************
    */
    uint8_t lld_isoal_datapath_remove(uint8_t act_id, uint8_t direction);

    /**
    ****************************************************************************************
    * @brief Set new isochronous data path (Old one must be removed first)
    *
    * @param[in]  act_id         Activity identifier
    * @param[in]  direction      Data Path direction (@see enum iso_rx_tx_select)
    * @param[in]  p_dp           Pointer to the Isochronous Datapath configuration
    *
    * @return CO_ERROR_NO_ERROR if succeed, error code else
    ****************************************************************************************
    */
    uint8_t lld_isoal_datapath_set(uint8_t act_id, uint8_t direction, const struct data_path_itf *p_dp);

#endif //  (BLE_ISO_PRESENT)

/**
 ****************************************************************************************
 * @brief Start Channel Scanning
 *
 * @param[in]  act_id
 * @param[in]  params        Pointer to Channel Scanning parameters structure
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t lld_ch_scan_start(uint8_t act_id, struct lld_ch_scan_params *params);

/**
 ****************************************************************************************
 * @brief Stop Channel Scanning
 ****************************************************************************************
 */
void lld_ch_scan_stop(void);

/**
 ****************************************************************************************
 * @brief Update the channel map used by the channel scan
 *
 * @param[in] ch_class      Pointer to the new channel classification to be used
 ****************************************************************************************
 */
void lld_ch_scan_ch_map_update(struct le_ch_class *ch_class);

#if (RW_DEBUG)
    /**
    ****************************************************************************************
    * @brief Start Connection Event trace
    *
    * @param[in]  con_hdl       Connection handle
    * @param[in]  nb_events     Number of events to trace [1..255]
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    void lld_con_evt_trace(uint16_t con_hdl, uint8_t nb_events);
#endif // (RW_DEBUG)

/**
 ****************************************************************************************
 * @brief Set RF Path Compensation
 *
 * @param[in]  tx_path_comp     TX path compensation value (in 0.1 dB)
 * @param[in]  rx_path_comp     RX path compensation value (in 0.1 dB)
 ****************************************************************************************
 */
void lld_path_comp_set(int16_t tx_path_comp, int16_t rx_path_comp);

/**
 ****************************************************************************************
 * @brief Get RF Tx Path Compensation
 *
 * @param[out]  tx_path_comp     TX path compensation value (in 0.1 dB)
 ****************************************************************************************
 */
int16_t lld_tx_path_comp_get(void);

/**
 ****************************************************************************************
 * @brief Get RF Rx Path Compensation
 *
 * @param[out]  rx_path_comp     RX path compensation value (in 0.1 dB)
 ****************************************************************************************
 */
int16_t lld_rx_path_comp_get(void);

/**
 ****************************************************************************************
 * @brief Get Radiative Transmit Power
 *
 * @param[in]  idx     CS-TXPWR value (in RF specific units)
 *
 * @return     txpwr   The radiative transmit power (in dBm)
 ****************************************************************************************
 */
int8_t lld_tx_power_dbm_get(uint8_t idx, uint8_t type);


/// @} LLD
#endif // LLD_H_
