/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file fhost_api.h
 *
 * @brief Public API for fully hosted module.
 *
 * Copyright (C) RivieraWaves 2019-2019
 *
 ****************************************************************************************
 */

#ifndef _FHOST_API_H_
#define _FHOST_API_H_

/**
 ****************************************************************************************
 * @defgroup FHOST_AL FHOST_AL
 * @ingroup FHOST
 *
 * The fully hosted module requires to be ported for the RTOS and Network stack used on
 * the final devices.
 * - RTOS: See @ref FHOST_RTOS_AL for details
 * - Network Stack: See @ref FHOST_NET_AL for details
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup FHOST_API FHOST_API
 * @ingroup FHOST
 * @brief Fully Hosted public API.
 *
 * This module list the public API provided by the fully hosted module for its usage in
 * external application/integration layers.
 *
 * It also defines the few functions that need to be implemented by the integration layer.
 *
 * @{
 ****************************************************************************************
 */
#include "mac.h"
#include "rwnx_defs.h"

/**
 * FHOST configuration item description
 */
struct fhost_config_item
{
    /**
     * Id of the configuration item (@ref fhost_config_id)
     */
    uint16_t id;
    /**
     * Size in bytes of data buffer
     */
    uint16_t len;
    /**
     * Configuration data
     */
    void *data;
};

/**
 * IDs for FHOST configuration items
 */
enum fhost_config_id
{
    /**
     * Inidicates last configuration item
     */
    FHOST_CFG_END = 0,
    /**
     * - use: Configure MAC address
     * - len: 6
     * - data: MAC address
     */
    FHOST_CFG_MAC_ADDR,
    /**
     * - use: Configure usage of Power Save mode
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_PS,
    /**
     * - use: Configure usage of Short Guard interval for 20Mhz and 40Mhz channels.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_SGI,
    /**
     * - use: Configure usage of Short Guard interval for 80Mhz channels.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_SGI80,
    /**
     * - use: Configure the maximum number of Spatial Stream used.
     * Default value is the max supported by the modem and invalid values are ignored.
     * - len: 4
     * - data: 32bits val
     */
    FHOST_CFG_NSS,
    /**
     * - use: Configure tx lifetime.
     * - len: 4
     * - data: 32bits val, in ms
     */
    FHOST_CFG_TX_LFT,
    /**
     * - use: Configure UAPSD timeout (for station).
     * - len: 4
     * - data: 32bits val, in ms
     */
    FHOST_CFG_UAPSD_TIMEOUT,
    /**
     * - use: Configure Low Power clock accuracy.
     * - len: 4
     * - data: 32bits val, in ppm
     */
    FHOST_CFG_LP_CLK_ACCURACY,
    /**
     * - use: Configure usage of HT.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_HT,
    /**
     * - use: Configure usage of 40Mhz in HT.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_40MHZ,
    /**
     * - use: Configure usage of VHT.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_VHT,
    /**
     * - use: Configure maximum MCS for VHT. (Ignored if SW or HW doesn't support VHT)
     * - len: 4
     * - data: 32bits val. 0=MCS0-7, 1=MCS0-8, 2=MCS0-9
     */
    FHOST_CFG_VHT_MCS,
    /**
     * - use: Configure advertisment of LDPC RX support in HT/VHT.
     * (Ignored if modem doesn't support it)
     * - len: 4
     * - data: 32bits boolean.
     */
    FHOST_CFG_LDPC,
    /**
     * - use: Configure advertisment of STBC RX support in VHT.
     * (Ignored if modem doesn't support it)
     * - len: 4
     * - data: 32bits boolean.
     */
    FHOST_CFG_STBC,
    /**
     * - use: Configure advertisment of BEAMFORMEE support in VHT.
     * (Ignored if modem doesn't support it)
     * - len: 4
     * - data: 32bits boolean.
     */
    FHOST_CFG_BFMEE,
    /**
     * - use: Configure advertisment of BEAMFORMER support in VHT.
     * (Ignored if modem doesn't support it)
     * - len: 4
     * - data: 32bits boolean.
     */
    FHOST_CFG_BFMER,
    /**
     * - use: Configure advertisment of Multi User RX support in VHT.
     * (Ignored if modem doesn't support it)
     * - len: 4
     * - data: 32bits boolean.
     */
    FHOST_CFG_MURX,
    /**
     * - use: Configure advertisment of Multi User TX support in VHT.
     * (Ignored if modem doesn't support it)
     * - len: 4
     * - data: 32bits boolean.
     */
    FHOST_CFG_MUTX,
    /**
     * - use: Configure karst radio. (Ignored if other radio is used)
     * - len: sizeof(struct phy_karst_cfg_tag)
     * - data: struct phy_karst_cfg_tag.
     */
    FHOST_CFG_PHY_KARST,
    /**
     * - use: Configure trident radio. (Ignored if other radio is used)
     * - len: sizeof(struct phy_trd_cfg_tag)
     * - data: struct phy_trd_cfg_tag.
     */
    FHOST_CFG_PHY_TRD,
    /**
     * - use: Configure usage of Antenna Diversity.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_ANT_DIV,
    /**
     * - use: Configure number of seconds since Epoch at boot.
     * - len: 4
     * - data: 32bits unsigned value
     */
    FHOST_CFG_EPOCH_SEC,
    /**
     * - use: Configure number of microseconds since Epoch at boot.
     * - len: 4
     * - data: 32bits unsigned value
     */
    FHOST_CFG_EPOCH_USEC,
    /**
     * - use: Configure usage of HE.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_HE,
    /**
     * - use: Configure maximum MCS for HE. (Ignored if SW or HW doesn't support HE)
     * - len: 4
     * - data: 32bits val. 0=MCS0-7, 1=MCS0-9, 2=MCS0-11
     */
    FHOST_CFG_HE_MCS,
    /**
     * - use: Configure usage of 80Mhz in VHT/HE.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_80MHZ,
    /**
     * - use: Configure usage of Dynamic Power-Save mode.
     * - len: 4
     * - data: 32bits boolean
     */
    FHOST_CFG_DPSM,
    /**
     * - use: Configure UAPSD queues (for station).
     * - len: 4
     * - data: 32bits bitfield (Bit0 = VO, Bit1 = VI, Bit2 = BK, Bit3 = BE)
     */
    FHOST_CFG_UAPSD_QUEUES,
    FHOST_CFG_MAX,
};

/**
 * Fully hosted module status
 */
struct fhost_status
{
    /**
     * Maximum number of interface supported
     */
    int vif_max_cnt;
    /**
     * Number of active interface
     */
    int vif_active_cnt;
    /**
     * Index of the first active interface. (Valid only if vif_active_cnt is not 0)
     */
    int vif_first_active;
    /**
     * Number of channel supported on 2.4GHz band
     */
    int chan_2g4_cnt;
    /**
     * Number of channel supported on 5GHz band
     */
    int chan_5g_cnt;
    /**
     * List of channels supported on 2.4GHz band
     */
    const struct mac_chan_def *chan_2g4;
    /**
     * List of channels supported on 5GHz band
     */
    const struct mac_chan_def *chan_5g;
};

/**
 * Fully hosted interface status
 */
struct fhost_vif_status
{
    /**
     * ID on the interface
     */
    int index;
    /**
     * Type of the interface (@ref mac_vif_type)
     */
    int type;
    /**
     * MAC address of the interface
     */
    const uint8_t *mac_addr;
    /**
     * Current operating channel.
     * Reset to 0 if there is no operating channel associated the interface
     * (e.g. non connected STA interface)
     */
    struct mac_chan_op chan;
    union {
        /**
         * Fields specific to a STA interface
         */
        struct {
            /**
             * BSSID of the AP. (Set to 0 if interface is not connected)
             */
            struct mac_addr bssid;
            /**
             * RSSI (in dBm) of the last received beacon. (valid only if connected)
             */
            int8_t rssi;
        } sta;
    };
};

/**
 * Fully hosted frame information
 */
struct fhost_frame_info
{
    /**
     * Length (in bytes) of the frame.
     */
    uint16_t length;
    /**
     * Primary channel frequency (in MHz) on which the frame has been received.
     */
    uint16_t freq;
    /**
     * Received signal strength (in dBm)
     */
    int8_t rssi;
    /**
     * Frame payload. Can be NULL if monitor mode is started with @p uf parameter set to
     * true. In this case all other fields are still valid.
     */
    uint8_t *payload;
    /**
     * PHY information about the received frame
     */
    struct rx_info *prinfo;
};

/**
 * Function prototype for RX callbacks
 */
typedef void (*cb_fhost_rx)(struct fhost_frame_info *info, void *arg);

/**
 * Configuration of VIF_MONITOR interface
 */
struct fhost_vif_monitor_cfg
{
    /**
     * Channel to monitor.
     * @ref fhost_configure_vif will fail if channel is not valid. It will also limit
     * the tx power.
     */
    struct mac_chan_op chan;
    /**
     * Report 'Unsupported Frames'.
     * If true, callback may be called with a payload parameter set to NULL
     */
    bool uf;
    /**
     * Frame received callback.
     * @attention The memory passed as parameter is only valid until it returns.
     */
    cb_fhost_rx cb;
    /**
     * Parameter for the monitor callback
     */
    void *cb_arg;
    /**
     * RX filter setting
     */
    uint32_t rx_filter;
};

/**
 * Configuration for VIF_STA interface
 */
struct fhost_vif_sta_cfg
{
    /**
     * SSID to connect to (mandatory)
     */
    struct mac_ssid ssid;
    /**
     * AP password/PSK passed as a string (i.e. MUST be terminated by a null byte)
     */
    char key[64 + 1];
    /**
     * AP BSSID. Optional, clear it to 0 if not set.
     */
    struct mac_addr bssid;
    /**
     * AP frequency. Optional, you can specify up to two frequencies on which AP
     * will be scanned. This is to speed up connection and should be set 0 if no used.
     */
    uint16_t freq[2];
    /**
     * Bit field of Authentication and Key Management (@ref mac_akm_suite) accepted
     * for the connection.
     * If set to 0, it default to MAC_AKM_NONE if @p key is the empty string and
     * MAC_AKM_PSK otherwise.
     */
    uint32_t akm;
    /**
     * Connection timeout in ms. If connection doesn't complete before timeout
     * @ref fhost_configure_vif will return an error.
     * If set to -1 timeout is disabled, and if set to 0 @ref fhost_configure_vif
     * will return without waiting for the connection to complete.
     */
    int timeout_ms;
};

/**
 * Configuration for p2p interface
 */
struct fhost_vif_p2p_cfg
{
    /**
     * SSID to connect to (mandatory)
     */
    struct mac_ssid ssid;
    /**
     * AP operating channel (tx_power and flags fields are ignored).
     */
    struct mac_chan_op chan;
    /**
     * AP password/PSK passed as a string (i.e. MUST be terminated by a null byte)
     */
    char key[64 + 1];
    /**
     * Enable HE
     */
    uint8_t enable_he;
    /**
     * Enable ACS
     */
    uint8_t enable_acs;
    /**
     * Vendor IE
     */
    const char *vendor_ie;
};

/**
 * Configuration for VIF_AP interface
 */
struct fhost_vif_ap_cfg
{
    /**
     * SSID to connect to (mandatory)
     */
    struct mac_ssid ssid;
    /**
     * AP operating channel (tx_power and flags fields are ignored).
     */
    struct mac_chan_op chan;
    /**
     * Beacon interval in TU
     */
    int bcn_interval;
    /**
     * DTIM period (number of beacon between 2 DTIM beacon)
     */
    int dtim_period;
    /**
     * Bit field of Authentication and Key Management (@ref mac_akm_suite)
     * Not all combination are possible
     */
    uint32_t akm;
    /**
     * AP password/PSK passed as a string (i.e. MUST be terminated by a null byte)
     */
    char key[64 + 1];
    /**
     * Bit field of Cipher suites (@ref mac_cipher_suite)
     * Can be left to 0 to use default one (depending of AKM selected)
     */
    uint32_t unicast_cipher;
    /**
     * Bit field of Cipher suites (@ref mac_cipher_suite)
     * Can be left to 0 to use default one (depending of AKM selected)
     */
    uint32_t group_cipher;
    /**
     * Management Frame Protection (0: disable, 1: allowed, 2: required)
     * (Ignored if not supported by selected AKM)
     */
    int mfp;
    /**
     * Hidden ssid
     */
    uint8_t hidden_ssid;
    /**
     * Value for maximum station inactivity, seconds
     */
    int max_inactivity;
    /**
     * Enable HE
     */
    uint8_t enable_he;
    /**
     * Enable ACS
     */
    uint8_t enable_acs;
    /**
     * STA number: max(NX_REMOTE_STA_MAX)
     */
    unsigned char sta_num;
};

/**
 * Fully hosted interface configuration
 */
struct fhost_vif_cfg
{
    /**
     * Identify which part of the union has been updated
     */
    enum mac_vif_type cfg_type;
    union {
        /**
         * Configuration if @p cfg_type == VIF_STA
         */
        struct fhost_vif_sta_cfg sta;
        /**
         * Configuration if @p cfg_type == VIF_MONITOR
         */
        struct fhost_vif_monitor_cfg monitor;
        /**
         * Configuration if @p cfg_type == VIF_AP
         */
        struct fhost_vif_ap_cfg ap;
    };
};

/**
 * Enum for IP address configuration mode
 */
enum fhost_ip_addr_mode
{
    IP_ADDR_NONE,
    IP_ADDR_STATIC_IPV4,
    IP_ADDR_DHCP_CLIENT,
};

/**
 * Fully Hosted IP address configuration (only IPv4 for now)
 */
struct fhost_vif_ip_addr_cfg
{
    /**
     * Select how to configure ip address when calling @ref fhost_set_vif_ip
     * Indicate how ip was configured when updated by @ref fhost_get_vif_ip
     */
    enum fhost_ip_addr_mode mode;
    /**
     * Whether interface must be the default output interface
     * (Unspecified when calling @ref fhost_get_vif_ip)
     */
    bool default_output;
    union {
        /**
         * IPv4 config.
         * Must be set when calling @ref fhost_set_vif_ip with @p mode==IP_ADDR_STATIC_IPV4
         * Always set when updated by @ref fhost_get_vif_ip (whatever the value of @p mode)
         */
        struct {
            /**
             * IPv4 address
             */
            uint32_t addr;
            /**
             * IPv4 address mask
             */
            uint32_t mask;
            /**
             * IPv4 addres of the gateway
             */
            uint32_t gw;
            /**
             * DNS server to use. (Ignored if set to 0)
             */
            uint32_t dns;
        } ipv4;
        /**
         * DHCP config.
         * Must be set when calling @ref fhost_set_vif_ip with @p addr_mode==IP_ADDR_DHCP_CLIENT
         */
        struct {
            /**
             * Timeout, in ms, to obtained an IP address
             */
            uint32_t to_ms;
        } dhcp;
    };
};

typedef struct {
    uint32_t ipc_irq_prio;
} fhost_user_cfg_t;

/**
 * Function prototype for frame transimission callbacks
 * - frame_id is the one returned by @ref fhost_send_80211_frame
 * - acknowledged indicates if the frame has been acknowledged by peer (or sucessfully sent
 *   for multicast frame)
 * - arg is the private argument passed when frame has been pushed for transmission
 */
typedef void (*cb_fhost_tx)(uint32_t frame_id, bool acknowledged, void *arg);

struct fhost_cntrl_link;

/*****************************************************************************************
 * Provided Functions
 ****************************************************************************************/
/**
 ****************************************************************************************
 * @brief Initialization of the fully hosted module.
 *
 * This function allocates all the resources needed by the different FHOST sub-modules.
 * This function will use the rtos_al API to create tasks, semaphores, etc. As such it
 * must be called after @ref rtos_init (or equivalent).
 *
 * @param[in] user_cfg  User specified configure, NULL for default
 ****************************************************************************************
 */
void fhost_init(struct rwnx_hw *rwnx_hw);
void fhost_update(struct rwnx_hw *rwnx_hw);

void fhost_deinit(struct rwnx_hw *rwnx_hw);

/**
 ****************************************************************************************
 * @brief Retrieve fully hosted module status.
 *
 * @param[out] status Structure to retrieve status
 ****************************************************************************************
 */
void fhost_get_status(struct fhost_status *status);

/**
 ****************************************************************************************
 * @brief Retrieve one interface status.
 *
 * @param[in]  fvif_idx  Index of the interface
 * @param[out] status    Structure to retrieve status
 *
 * @return 0 on success and != 0 if error occurred
 ****************************************************************************************
 */
int fhost_get_vif_status(int fvif_idx, struct fhost_vif_status *status);

/**
 ****************************************************************************************
 * @brief Open a CFGRWNX link with the fhost firmware.
 *
 * A CFGRWNX link is used for communication with the 'CONTROL' task of the fhost firmware
 * which centralizes all interaction with the 'WIFI' task.
 * When no longer required a link must be close using @ref fhost_cntrl_cfgrwnx_link_close.
 *
 * @return pointer to fhost_cntrl_link structure on success, NULL otherwise
 ****************************************************************************************
 */
struct fhost_cntrl_link *fhost_cntrl_cfgrwnx_link_open(void);

/**
 ****************************************************************************************
 * @brief Close a CFGRWNX link.
 *
 * Close a link opened with @ref fhost_cntrl_cfgrwnx_link_open.
 *
 * @param[in] link  Pointer to previously opened link
 ****************************************************************************************
 */
void fhost_cntrl_cfgrwnx_link_close(struct fhost_cntrl_link *link);

/**
 ****************************************************************************************
 * @brief Change interface type
 *
 * When the interface type is changed, the current interface will first be cleared (i.e.
 * close its connection, release its IP address ...)
 * If interface is already of the requested type the function returns immediately.
 * If you want to clear an interface without changing its type, you should first set its
 * type to VIF_UNKNOWN and then reset it.
 *
 * @param[in] link      Pointer to a cfgrwnx link
 * @param[in] fvif_idx  Index of the interface
 * @param[in] type      Type to set on the interface
 * @param[in] p2p       Whether interface will be used for a P2P (aka Wifi Direct)
 *                      connection (Used only if @p type is VIF_STA of VIF_AP)
 *
 * @return 0 on success and <0 if any error occurred.
 ****************************************************************************************
 */
int fhost_set_vif_type(struct fhost_cntrl_link *link, int fvif_idx, enum mac_vif_type type,
                       bool p2p);

/**
 ****************************************************************************************
 * @brief Configure an interface IP address
 *
 * If this is not supported by upper layer then this function can be used to configure
 * ip address of an interface. So far only IPv4 is supported.
 * If IP_ADDR_DHCP_CLIENT mode is used, upon success the @p cfg parameter is updated
 * with the actual ip address obtained in cfg.ipv4.
 * If IP_ADDR_STATIC_IPV4 mode is used and DNS is not set, upon success the @p cfg
 * parameter is updated with the current DNS in cfg.ipv4.dns
 *
 * @note This function can only be used if the @ref FHOST_NET_AL implements the IP
 * related functions
 *
 * @param[in]     fvif_idx  Index of the interface
 * @param[in,out] cfg       Configuration of the interface
 *
 * @return 0 on success and <0 if any error occurred.
 ****************************************************************************************
 */
int fhost_set_vif_ip(int fvif_idx, struct fhost_vif_ip_addr_cfg *cfg);

/**
 ****************************************************************************************
 * @brief Retrieve IP address configuration of an interface
 *
 * If this is not supported by upper layer then this function can be used to retrieve
 * ip address of an interface. So far only IPv4 is supported.
 *
 * @note Some fields of the @p cfg parameter have a slightly different meaning than when
 * used by @ref fhost_set_vif_ip (see documentaion for @ref fhost_vif_ip_addr_cfg).
 *
 * @note This function can only be used if the @ref FHOST_NET_AL implements the IP
 * related functions
 *
 * @param[in]  fvif_idx  Index of the interface
 * @param[out] cfg       Configuration of the interface
 *
 * @return 0 on success and <0 if any error occurred.
 ****************************************************************************************
 */
int fhost_get_vif_ip(int fvif_idx, struct fhost_vif_ip_addr_cfg *cfg);

/**
 ****************************************************************************************
 * @brief Scan Access Points on a given interface
 *
 * This function is blocking until the scan is finished. Once the scan is complete you
 * can use @ref fhost_get_scan_results to retrieve the results.
 *
 * @param[in] link      Pointer to a cfgrwnx link
 * @param[in] fvif_idx  Index of the interface
 *
 * @return < 0 if an error occurred, otherwise it returns the number of Access point
 * found.
 ****************************************************************************************
 */
int fhost_scan(struct fhost_cntrl_link *link, int fvif_idx, int *freq_list);
int fhost_scan_frame_handler(struct scanu_result_ind *res);

/**
 ****************************************************************************************
 * @brief Retrieve scan results
 *
 * This function retrieves up to @p max_nb_result scan results starting at index
 * @p result_idx.
 * To retrieve all scan results, first call this function with result_idx = 0;
 * Then as long as the value returned is equal to @p max_nb_result you can retrieve the
 * other results by updating @p result_idx value (incremented by the number of results
 * already read).
 * All results have been read when the value returned is lower than @p max_nb_result.
 *
 * @param[in]  link           Pointer to a cfgrwnx link
 * @param[in]  result_idx     Index of the first result to read
 * @param[in]  max_nb_result  Size, in number of element, of @p results
 * @param[out] results        Buffer to retrieve the scan results.
 *
 * @return The number of results written in @p results.
 ****************************************************************************************
 */
int fhost_get_scan_results(struct fhost_cntrl_link *link, int result_idx,
                           int max_nb_result, struct mac_scan_result *results);

/**
 ****************************************************************************************
 * @brief Transmit a 802.11 frame
 *
 * This function is used to send a pre-formatted 802.11 frame (most likely a Management
 * frame). The frame is sent as a singleton using  basic rates.
 * The memory passed as parameter is first copied before programming the transmission.
 * It can then be re-used immediately once the function returns.
 *
 * @attention If callback function is provided it will be executed in WIFI task context
 * and as such it should be fairly quick and cannot call any function of this API.
 *
 * @param[in] fvif_idx    Index of the interface
 * @param[in] frame       Pointer to the data to send
 * @param[in] length      Size, in bytes, of the provided data
 * @param[in] cfm_cb      Callback when transmission has been completed (may be NULL)
 * @param[in] cfm_cb_arg  Private argument for the callback.
 *
 * @return A unique frame id (the one passed in the confirmation callback) and 0 if
 * an error happened
 ****************************************************************************************
 */
uint32_t fhost_send_80211_frame(int fvif_idx, const uint8_t *frame, uint32_t length,
                                cb_fhost_tx cfm_cb, void *cfm_cb_arg);

/*****************************************************************************************
 * Required Functions
 ****************************************************************************************/
/**
 ****************************************************************************************
 * @brief Get the next configuration item.
 *
 * This function is used by the fully hosted firmware to retrieve its configuration as
 * defined by @ref fhost_config_id.
 * To retrieve its configuration, the firmware first calls this function with a @p item
 * parameter with all its fields set to 0. On all subsequent calls, the @p item parameter
 * is the one returned by the previous call. The firmware stops calling this function as
 * soon as the item @ref FHOST_CFG_END is returned
 *
 * Currently the only mandatory configuration items are @ref FHOST_CFG_MAC_ADDR and
 * FHOST_CFG_PHY_XXX (if applicable to your platform). For any other tiems that is not
 * provided by this function the fullt hosted firmware will select a default value
 * adapted to the MACHW used.
 *
 * @note The firmware may need to retrieve its configuration several times during the
 * execution so this must be supported.
 *
 * @param[in,out] item  Pointer to the previous configuration item updated
 *                      with the next one.
 ****************************************************************************************
 */
void fhost_config_get_next_item(struct fhost_config_item *item);

/// @}

#endif // _FHOST_API_H_
