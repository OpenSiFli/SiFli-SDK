/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file cfgrwnx.h
 *
 * @brief Definition of message index and associated structures for messages of type
 * @ref FHOST_MSG_CFGRWNX.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */
#ifndef _CFGRWNX_H_
#define _CFGRWNX_H_

#include "fhost_api.h"
#include "rwnx_config.h"
//#include "netif_port.h"

/**
 ****************************************************************************************
 * @defgroup FHOST_CFGRWNX FHOST_CFGRWNX
 * @ingroup FHOST_CNTRL
 * Interface description between fhost control task and supplicant application.
 *
 * @{
 ****************************************************************************************
 */
/// UDP port for CFGRWNX connection
#define CFGRWNX_PORT 1630

/// Marker to indicate end of splited message
#define CFGRWNX_LAST_BUF 0x1a51b0ff

/// CFGRWNX index message
enum cfgrwnx_msg_index {
    /// Sent by supplicant to retrieve HW capability (param: none)
    CFGRWNX_HW_FEATURE_CMD = 1,
    /// Response to CFGRWNX_HW_FEATURE_CMD (param: @ref cfgrwnx_hw_feature)
    CFGRWNX_HW_FEATURE_RESP,
    /// Sent by supplicant to retrieve FW capability (param: none)
    CFGRWNX_GET_CAPA_CMD,
    /// Response to CFGRWNX_GET_CAPA_CMD (param: none)
    CFGRWNX_GET_CAPA_RESP,
    /// Sent by Supplicant to install/remove Encryption key (param: @ref cfgrwnx_set_key)
    CFGRWNX_SET_KEY_CMD,
    /// Response to CFGRWNX_SET_KEY_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_SET_KEY_RESP,
    /// Sent by Supplicant to start a SCAN (param: @ref cfgrwnx_scan)
    CFGRWNX_SCAN_CMD,
    /// Response to CFGRWNX_SCAN_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_SCAN_RESP,
    /// Event sent when scan is done (param: @ref cfgrwnx_scan_completed)
    CFGRWNX_SCAN_DONE_EVENT,
    /// Event sent when a new AP is found (param: @ref cfgrwnx_scan_result)
    CFGRWNX_SCAN_RESULT_EVENT,                                    // 10
    /// Sent by supplicant to initiate a connection (param: @ref cfgrwnx_connect)
    CFGRWNX_CONNECT_CMD,
    /// Response to CFGRWNX_CONNECT_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_CONNECT_RESP,
    /// Event sent when the connection is finished (param: @ref cfgrwnx_connect_event)
    CFGRWNX_CONNECT_EVENT,
    /// Sent by supplicant to end a connection (param: @ref cfgrwnx_disconnect)
    CFGRWNX_DISCONNECT_CMD,
    /// Response to CFGRWNX_DISCONNECT_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_DISCONNECT_RESP,
    /// Event sent if the connection is lost (param: @ref cfgrwnx_disconnect_event)
    CFGRWNX_DISCONNECT_EVENT,
    /// Sent by supplicant to open/close a control port (param: @ref cfgrwnx_ctrl_port)
    CFGRWNX_CTRL_PORT_CMD,
    /// Response to CFGRWNX_CTRL_PORT_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_CTRL_PORT_RESP,
    /// Event sent if a Michael MIC failure is detected (param: @ref cfgrwnx_mic_failure_event)
    CFGRWNX_MIC_FAILURE_EVENT,
    /// Sent by Application to retrieve system statistics (param: none)
    CFGRWNX_SYS_STATS_CMD,                                       //20
    /// Response to CFGRWNX_SYS_STATS_CMD (param: @ref cfgrwnx_sys_stats_resp)
    CFGRWNX_SYS_STATS_RESP,
    /// Sent by smartconfig to obtain scan results (param: none)
    CFGRWNX_SCAN_RESULTS_CMD,
    /// Response to CFGRWNX_SCAN_RESULTS_CMD (param: @ref cfgrwnx_scan_results_resp)
    CFGRWNX_SCAN_RESULTS_RESP,
    /// Sent by Application to retrieve FW/PHY supported features (param: none)
    CFGRWNX_LIST_FEATURES_CMD,
    /// Response to CFGRWNX_LIST_FEATURES_CMD (param: @ref cfgrwnx_list_features_resp)
    CFGRWNX_LIST_FEATURES_RESP,
    /// Sent to change the type of an vif at MAC level. MAC VIF is deleted (if it exists) and
    /// re-created with the new type (unless type is VIF_UNKNOWN) (param: @ref cfgrwnx_set_vif_type)
    CFGRWNX_SET_VIF_TYPE_CMD,
    /// Response to CFGRWNX_SET_VIF_TYPE_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_SET_VIF_TYPE_RESP,
    /// Sent by Application to configure a monitor interface (param: @ref cfgrwnx_monitor_cfg)
    CFGRWNX_MONITOR_CFG_CMD,
    /// Response to CFGRWNX_MONITOR_CFG_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_MONITOR_CFG_RESP,
    /// Event sent by the RX task when management frame is forwarded by the wifi task (param: @ref cfgrwnx_rx_mgmt_event)
    CFGRWNX_RX_MGMT_EVENT,                                     //30
    /// Event to defer TX status processing (param: @ref cfgrwnx_tx_status_event)
    CFGRWNX_TX_STATUS_EVENT,
    /// Event sent by wifi task to request external authentication (i.e. Supplicant will do the
    /// authentication procedure for the wifi task, used for SAE) (param: @ref cfgrwnx_external_auth_event)
    CFGRWNX_EXTERNAL_AUTH_EVENT,
    /// Sent by Supplicant to pass external authentication status (param: @ref cfgrwnx_external_auth_status)
    CFGRWNX_EXTERNAL_AUTH_STATUS_RESP,
#ifdef CFG_SOFTAP
    /// Sent by Supplicant to start an AP (param: @ref cfgrwnx_start_ap)
    CFGRWNX_START_AP_CMD,
    /// Response to CFGRWNX_START_AP_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_START_AP_RESP,
    /// Sent by Supplicant to stop an AP (param: @ref cfgrwnx_stop_ap)
    CFGRWNX_STOP_AP_CMD,
    /// Response to CFGRWNX_STOP_AP_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_STOP_AP_RESP,
#endif
    /// Sent by Supplicant to configure EDCA parameter for one AC (param: @ref cfgrwnx_set_edca)
    CFGRWNX_SET_EDCA_CMD,
    /// Response to CFGRWNX_SET_EDCA_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_SET_EDCA_RESP,
#ifdef CFG_SOFTAP
    /// Sent by Supplicant to update the beacon (param: @ref cfgrwnx_bcn_update)
    CFGRWNX_BCN_UPDATE_CMD,
    /// Response to CFGRWNX_BCN_UPDATE (param: @ref cfgrwnx_resp)
    CFGRWNX_BCN_UPDATE_RESP,
#endif
    /// Send to supplicant to register a new Station (param: @ref cfgrwnx_sta_add)
    CFGRWNX_STA_ADD_CMD,
    /// Response to CFGRWNX_STA_ADD_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_STA_ADD_RESP,
    /// Send to supplicant to un-register a Station (param: @ref cfgrwnx_sta_remove)
    CFGRWNX_STA_REMOVE_CMD,
    /// Response to CFGRWNX_STA_REMOVE_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_STA_REMOVE_RESP,
    /// Send by supplicant to retrieve Key current sequence number (param: @ref cfgrwnx_key_seqnum)
    CFGRWNX_KEY_SEQNUM_CMD,
    /// Response to CFGRWNX_KEY_SEQNUM_CMD (param: @ref cfgrwnx_key_seqnum_resp)
    CFGRWNX_KEY_SEQNUM_RESP,
    /// Enable Power Save (param: @ref cfgrwnx_set_ps_mode)
    CFGRWNX_SET_PS_MODE_CMD,
    /// Response to CFGRWNX_SET_PS_MODE_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_SET_PS_MODE_RESP,

    CFGRWNX_CH_SWITCH_EVENT,
    /// Request to remain on specific channel (param: @ref cfgrwnx_remain_on_channel)
    CFGRWNX_REMAIN_ON_CHANNEL_CMD,
    /// Response to CFGRWNX_REMAIN_ON_CHANNEL_CMD. Only indicates if request was valid or
    /// not. The actual status will be in CFGRWNX_REMAIN_ON_CHANNEL_EVENT (param: @ref
    /// cfgrwnx_resp)
    CFGRWNX_REMAIN_ON_CHANNEL_RESP,
    /// Event sent after receiving CFGRWNX_REMAIN_ON_CHANNEL_CMD, to indicate that the
    /// procedure is completed (param: @ref cfgrwnx_remain_on_channel_event)
    CFGRWNX_REMAIN_ON_CHANNEL_EVENT,
    /// Request to cancel remain on channel (param: @ref cfgrwnx_cancel_remain_on_channel)
    CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_CMD,
    /// Response to CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_CMD. Only indicates if request was
    /// valid or not. The actual status will be in CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_EVENT
    /// (param: @ref cfgrwnx_resp)
    CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_RESP,
    /// Event sent after receiving CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_CMD, to indicate that
    /// the procedure is completed (param: @ref cfgrwnx_remain_on_channel_event)
    CFGRWNX_REMAIN_ON_CHANNEL_EXP_EVENT,

    CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_EVENT,
    /// Request by Application to setup NOA protocol (param: @ref cfgrwnx_p2p_noa_cmd)
    CFGRWNX_P2P_NOA_CMD,
    /// Response to CFGRWNX_P2P_NOA_CMD (param: @ref cfgrwnx_resp)
    CFGRWNX_P2P_NOA_RESP,
    /// Notified by supplicant to indicate P2P Provision Discovery Request received (param: @ref cfgrwnx_p2p_prov_disc_req_note)
    CFGRWNX_P2P_PROV_DISC_REQ_NOTE,

    #ifdef CFG_SOFTAP
    /// Sent by Supplicant to update the beacon (param: @ref cfgrwnx_bcn_update)
    CFGRWNX_AP_UPDATE_BASICRATE_CMD,
    /// Response to CFGRWNX_BCN_UPDATE (param: @ref cfgrwnx_resp)
    CFGRWNX_AP_UPDATE_BASICRATE_RESP,
    #endif
};

/// CFGRWNX status
enum cfgrwnx_status {
    /// Success status
    CFGRWNX_SUCCESS = 0,
    /// Generic error status
    CFGRWNX_ERROR,
    /// Error invalid VIF index parameter
    CFGRWNX_INVALID_VIF,
    /// Error invalid STA index parameter
    CFGRWNX_INVALID_STA,
    /// Error invalid parameter
    CFGRWNX_INVALID_PARAM,
};

/// CFGRWNX message header
struct cfgrwnx_msg_hdr {
    /// For CFGRWNX commands, queue handle to use to push the response
    rtos_queue resp_queue;
    /// Length, in bytes, of the message (including this header)
    uint16_t len;
    /// ID of the message.
    uint16_t id;
};

/// CFGRWNX generic message structure
struct cfgrwnx_msg {
    /// header
    struct cfgrwnx_msg_hdr hdr;
};

/// CFGRWNX generic response structure
struct cfgrwnx_resp {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Status
    uint32_t status;
};

/// structure for CFGRWNX_HW_FEATURE_RESP
struct cfgrwnx_hw_feature {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// ME configuration
    struct me_config_req me_config;
    /// Channel configuration
    struct me_chan_config_req *chan;
};

/// structure for CFGRWNX_SET_KEY_CMD
struct cfgrwnx_set_key {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    int fhost_vif_idx;
    /// MAC addr (may be NULL for group key)
    const struct mac_addr *addr;
    /// Cipher
    int cipher_suite;
    /// Key index
    int key_idx;
    /// Key
    const uint8_t *key;
    /// Key length
    int key_len;
    /// Initial Sequence number
    const uint8_t *seq;
    /// Sequence number length
    int seq_len;
};

#ifdef CFG_SOFTAP
/// structure for CFGRWNX_AP_TX_MGMT
struct cfgrwnx_ap_tx_mgmt {
    /// header
    struct cfgrwnx_msg_hdr hdr;

    uint8_t *msg;

    uint16_t len;

    /// Index of the VIF for which the AP is started
    uint8_t vif_idx;

    int sock;
};
/// structure for CFGRWNX_AP_STA_ADD
struct cfgrwnx_ap_sta_add {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    // Idx, at wifi task level, of the AP
    uint32_t sta_idx;
    /// MAC address of the station to be added
    struct mac_addr mac_addr;
    /// Supported legacy rates
    struct mac_rateset rate_set;
    /// HT Capabilities
    struct mac_htcapability ht_cap;
    /// VHT Capabilities
    struct mac_vhtcapability vht_cap;
    /// HE capabilities
    struct mac_hecapability he_cap;
    /// Flags giving additional information about the station
    uint32_t flags;
    /// Association ID of the station
    uint16_t aid;
    /// Bit field indicating which queues have U-APSD enabled
    uint8_t uapsd_queues;
    /// Maximum size, in frames, of a APSD service period
    uint8_t max_sp_len;
    /// Operation mode information (valid if bit @ref STA_OPMOD_NOTIF is
    /// set in the flags)
    uint8_t opmode;
    /// Index of the VIF the station is attached to
    uint8_t vif_idx;
    /// Whether the station is a TDLS station
    bool tdls_sta;
    /// Whether the TDLS station is the TDLS link initiator
    bool tdls_initiator;
    /// Whether the TDLS Channel Switch is allowed
    bool tdls_chsw_allowed;

    int sock;
};
/// structure for CFGRWNX_AP_STA_REMOVE
struct cfgrwnx_ap_sta_del {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    // Idx, at wifi task level, of the AP
    uint32_t sta_idx;
    /// Whether the station is a TDLS station
    bool tdls_sta;
};

#endif /* CFG_SOFTAP */

/// SSID representation used by CFGRWNX_SCAN_CMD and CFGRWNX_CONNECT_CMD
struct cfgrwnx_scan_ssid {
    /// SSID string
    const uint8_t *ssid;
    /// Length of the SSID string
    size_t len;
};

/// structure for CFGRWNX_SCAN_CMD
struct cfgrwnx_scan {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    int fhost_vif_idx;
    /// SSID to scan
    struct cfgrwnx_scan_ssid *ssids;
    /// Number of SSID in ssids
    int ssid_cnt;
    /// Extra IE to add in the proce request
    const uint8_t *extra_ies;
    /// Size of the extra IEs
    int extra_ies_len;
    /// Array of frequencies to scan or %NULL for all frequencies.
    /// The frequency is set in MHz. The array is zero-terminated.
    int *freqs;
    /// Do not use CCK mode
    bool no_cck;
    /// BSSID to scan, can be NULL for wildcard BSSID
    const uint8_t *bssid;
    /// Socket to use to send CFGRWNX events
    int sock;
};

/// structure for CFGRWNX_SCAN_DONE_EVENT
struct cfgrwnx_scan_completed {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Status
    uint32_t status;
    /// Nb result available with CFGRWNX_SCAN_RESULTS_CMD
    uint32_t result_cnt;
};

/// structure for CFGRWNX_SCAN_RESULT_EVENT
struct cfgrwnx_scan_result {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Frequency
    uint16_t freq;
    /// RSSI of the received frame (dBm).
    int16_t rssi;
    /// Length of the frame (beacon or probe resposne)
    uint16_t length;
    /// Frame payload. Must be freed (using rtos_free) when event has been processed
    uint8_t *payload;
};

/// structure for CFGRWNX_CONNECT_CMD
struct cfgrwnx_connect {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// BSSID tO connect to
    const uint8_t *bssid;
    /// SSID to connect to
    struct cfgrwnx_scan_ssid ssid;
    /// Channel of the AP
    struct mac_chan_def chan;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Authentication Type
    uint16_t auth_alg;
    /// Connection flags
    uint32_t flags;
    /// Control port Ethertype
    uint16_t ctrl_port_ethertype;
    /// UAPSD queues (bit0: VO, bit1: VI, bit2: BK, bit3: BE). Set to 0xFFFF to use default config
    uint16_t uapsd;
    /// Length, in bytes, of the extra IE
    uint32_t ie_len;
    /// Extra IE to add to association request
    const uint8_t *ie;
    /// Socket to use to send CFGRWNX events
    int sock;
};

/// structure for CFGRWNX_CONNECT_EVENT
struct cfgrwnx_connect_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// BSSID
    struct mac_addr bssid;
    /// Sta idx
    int sta_idx;
    /// Status code of the connection procedure
    uint16_t status_code;
    /// Frequency of the operational channel in MHz
    uint16_t freq;
    /// Length of the AssocReq IEs (in bytes)
    uint16_t assoc_req_ie_len;
    /// Length of the AssocRsp IEs (in bytes)
    uint16_t assoc_resp_ie_len;
    /// AssocReq IEs (assoc_req_ie_len) followed by AssocRsp IEs (assoc_resp_ie_len)
    /// Must be freed (using rtos_free) when event has been processed
    uint8_t *req_resp_ies;
};

/// structure for CFGRWNX_DISCONNECT_CMD
struct cfgrwnx_disconnect {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Reason of the disconnection
    uint16_t reason_code;
    /// Vif idx
    uint16_t fhost_vif_idx;
};

/// structure for CFGRWNX_DISCONNECT_EVENT
struct cfgrwnx_disconnect_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Reason of the disconnection
    uint16_t reason_code;
    /// Vif idx
    uint16_t fhost_vif_idx;
};

/// structure for CFGRWNX_CTRL_PORT_CMD
struct cfgrwnx_ctrl_port {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// VIF index
    uint16_t fhost_vif_idx;
    /// Address of the STA managed by the port (Needed only for AP interface)
    struct mac_addr addr;
    /// Port status
    uint16_t authorized;
};

/// structure for CFGRWNX_MIC_FAILURE_EVENT
struct cfgrwnx_mic_failure_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Address of the sending STA
    struct mac_addr addr;
    /// Boolean indicating if the packet was a group or unicast one (true if group)
    bool ga;
    /// VIF index
    uint16_t fhost_vif_idx;
};

#ifdef CFG_SOFTAP
/// structure for CFGRWNX_AP_RX_MGMT
struct cfgrwnx_ap_rx_mgmt {
    /// header
    struct cfgrwnx_msg_hdr hdr;

    /// Vif idx
    uint16_t fhost_vif_idx;

    uint16_t frame_len;

    /// Frame payload.
    uint8_t *payload;

};

/// structure for CFGRWNX_AP_TX_STATUS
struct cfgrwnx_ap_tx_status {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    int32_t ack;

    uint16_t type;
    uint16_t stype;
    uint8_t dst[MAC_ADDR_LEN];
    uint16_t data_len;
    uint8_t *data;
};
#endif /* CFG_SOFTAP */

/// Structure for @ref CFGRWNX_SYS_STATS_RESP
struct cfgrwnx_sys_stats_resp {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Statistics
    struct dbg_get_sys_stat_cfm stats;
};

/// Structure for CFGRWNX_SCAN_RESULTS_CMD
struct cfgrwnx_scan_results {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// index of the scan result element
    uint16_t idx;
};

/// Structure for CFGRWNX_SCAN_RESULTS_RESP
struct cfgrwnx_scan_results_resp {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// scan results
    struct mac_scan_result scan_result;
    /// Status
    uint32_t status;
};

/// Structure for @ref CFGRWNX_LIST_FEATURES_RESP
struct cfgrwnx_list_features_resp {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// structure containing FW/PHY features
    struct mm_version_cfm version;
};

/// Structure for @ref CFGRWNX_SET_VIF_TYPE_CMD
struct cfgrwnx_set_vif_type {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Index of the FHOST vif
    int fhost_vif_idx;
    /// Type to set on the interface
    enum mac_vif_type type;
    /// Is a P2P vif (only read if type is VIF_STA or VIF_AP)
    bool p2p;
};

/// Structure for CFGRWNX_MONITOR_CFG_CMD
struct cfgrwnx_monitor_cfg {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Interface configuration
    struct fhost_vif_monitor_cfg cfg;
};

/// Structure for CFGRWNX_RX_MGMT_EVENT
struct cfgrwnx_rx_mgmt_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Frequency
    uint16_t freq;
    /// RSSI of the received frame (dBm).
    int16_t rssi;
    /// Length of the frame
    uint16_t length;
    /// Frame payload.
    uint8_t *payload;
};

/// Structure for CFGRWNX_TX_STATUS_EVENT
struct cfgrwnx_tx_status_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Frame data
    const uint8_t *data;
    /// Frame length
    uint32_t data_len;
    /// TX status
    bool acknowledged;
};

/// Structure for CFGRWNX_EXTERNAL_AUTH_EVENT
struct cfgrwnx_external_auth_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// BSSID
    struct mac_addr bssid;
    /// SSID
    struct mac_ssid ssid;
    /// AKM
    uint32_t akm;
};

/// Structure for CFGRWNX_EXTERNAL_AUTH_STATUS_RESP
struct cfgrwnx_external_auth_status {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Authentication status
    uint16_t status;
};

/// Structure for CFGRWNX_START_AP_CMD
struct cfgrwnx_start_ap {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Basic rates
    struct mac_rateset basic_rates;
    /// Operating Channel for the AP
    struct mac_chan_op chan;
    /// Buffer containing the Beacon
    uint8_t *bcn;
    /// Size, in bytes, of the Beacon buffer
    int bcn_len;
    /// Offset within the beacon of the TIM element (in bytes)
    int tim_oft;
    /// Size, in bytes, of the TIM element
    int tim_len;
    /// Beacon interval in TU
    int bcn_int;
    /// AP flags (@see mac_connection_flags)
    int flags;
    /// Port number for ethernet authentication frame
    uint16_t ctrl_ethertype;
    /// CFGRWNX event socket (socket to upload CFGRWNX event)
    int sock;
};

/// Structure for CFGRWNX_STOP_AP_CMD
struct cfgrwnx_stop_ap {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
};

/// Structure for CFGRWNX_CH_SWITCH_EVENT
struct cfgrwnx_ch_switch_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    int freq;
    int ht_enabled;
    int ch_offset;
    uint16_t ch_width; //enum chan_width
    int cf1;
    int cf2;

};

/// Structure for CFGRWNX_SET_EDCA_CMD
struct cfgrwnx_set_edca {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Access Category index
    uint8_t aci;
    /// Arbitration InterFrame Space Number
    uint8_t aifsn;
    /// Contention Window minimum
    uint16_t cwmin;
    /// Contention Window maximum
    uint16_t cwmax;
    /// TXOP (in unit of 32us)
    uint16_t txop;
};

/// Structure for CFGRWNX_BCN_UPDATE_CMD
struct cfgrwnx_bcn_update {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Buffer containing the Beacon
    uint8_t *bcn;
    /// Size, in bytes, of the Beacon buffer
    int bcn_len;
    /// Offset within the beacon of the TIM element (in bytes)
    int tim_oft;
    /// Size, in bytes, of the TIM element
    int tim_len;
    /// CSA
    uint8_t csa_oft[BCN_MAX_CSA_CPT];
};

/// Structure for CFGRWNX_STA_ADD_CMD
struct cfgrwnx_sta_add {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Association ID of the station
    uint16_t aid;
    /// MAC address of the station to be added
    const struct mac_addr *addr;
    /// Supported legacy rates
    struct mac_rateset rate_set;
    /// HT Capabilities
    struct mac_htcapability ht_cap;
    /// VHT Capabilities
    struct mac_vhtcapability vht_cap;
    /// HE capabilities
    struct mac_hecapability he_cap;
    /// STA flags (@ref mac_sta_flags)
    uint32_t flags;
    /// Bit field indicating which queues have U-APSD enabled
    uint8_t uapsd_queues;
    /// Maximum size, in frames, of a APSD service period
    uint8_t max_sp_len;
    /// Operation mode information (valid if bit @ref STA_OPMOD_NOTIF is
    /// set in the flags)
    uint8_t opmode;
};

/// Structure for CFGRWNX_STA_REMOVE_CMD
struct cfgrwnx_sta_remove {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// MAC address of the station
    const struct mac_addr *addr;
};

/// structure for CFGRWNX_KEY_SEQNUM_CMD
struct cfgrwnx_key_seqnum {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// MAC address of the station (null for group key)
    const struct mac_addr *addr;
    /// Key index
    uint16_t key_idx;
};

/// structure for CFGRWNX_KEY_SEQNUM_RESP
struct cfgrwnx_key_seqnum_resp {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Status
    uint32_t status;
    /// Seq num
    uint64_t seqnum;
};

/// structure for CFGRWNX_SET_PS_MODE_CMD
struct cfgrwnx_set_ps_mode {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// PS mode status
    bool enabled;
};

/// structure for CFGRWNX_REMAIN_ON_CHANNEL_CMD
struct cfgrwnx_roc_cmd {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// freq
    unsigned int freq;
    /// duration
    unsigned int duration;
    /// Index of the FHOST vif
    int fhost_vif_idx;
    /// event socket
    int sock;
};

/// structure for CFGRWNX_CANCEL_REMAIN_ON_CHANNEL_CMD
struct cfgrwnx_cancel_roc_cmd {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Index of the FHOST
    int fhost_vif_idx;
};

/// structcure for CFGRWNX_ROC_EVENT
struct cfgrwnx_roc_event {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// ieee80211_channel center_freq
    uint32_t center_freq;
    /// duration ms
    unsigned int duration;
};

/// Structure for CFGRWNX_P2P_NOA_CMD
struct cfgrwnx_p2p_noa_cmd
{
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Count
    uint8_t count;
    /// Duration (in us)
    uint32_t duration_us;
    /// Interval (in us)
    uint32_t interval_us;
    /// Indicate if NoA can be paused for traffic reason
    bool dyn_noa;
};

/// Structure for CFGRWNX_P2P_PROV_DISC_REQ_NOTE
struct cfgrwnx_p2p_prov_disc_req_note {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Reserved
    uint8_t reserved0[2];
    /// mac_peer
    struct mac_addr mac_peer;
};

/// Structure for CFGRWNX_AP_UPDATE_BASICRATE
struct cfgrwnx_ap_update_basicrate {
    /// header
    struct cfgrwnx_msg_hdr hdr;
    /// Vif idx
    uint16_t fhost_vif_idx;
    /// Basic rates
    struct mac_rateset basic_rates;
};

/**
 * @}
 */
#endif /* _CFGRWNX_H_ */
