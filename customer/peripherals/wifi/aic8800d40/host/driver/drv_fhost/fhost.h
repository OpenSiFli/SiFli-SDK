/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file fhost.h
 *
 * @brief Definitions of the fully hosted module.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

#ifndef _FHOST_H_
#define _FHOST_H_

/**
 ****************************************************************************************
 * @defgroup FHOST FHOST
 * @ingroup MACSW
 * @brief Fully Hosted module implementation.
 * This module creates different RTOS tasks that will be used to handle the operations
 * of the fully-hosted FW
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "mac.h"
#include "co_list.h"
#include "fhost_api.h"
#include "rwnx_config.h"
#include "aic_plat_net.h"
#include "aic_plat_types.h"
#include "rtos_port.h"
#include <string.h>
#include "wifi_al.h"

/*
 * DEFINITIONS
 ****************************************************************************************
 */
#define FMAC_REMOTE_STA_MAX     aic_wifi_fmac_remote_sta_max_get()
#define BROADCAST_STA_IDX_MIN   FMAC_REMOTE_STA_MAX
#define VIF_TO_BCMC_IDX(idx)    (BROADCAST_STA_IDX_MIN + (idx))
#define STA_MAX                 (NX_REMOTE_STA_MAX + NX_VIRT_DEV_MAX)
#define FMAC_STA_MAX            (FMAC_REMOTE_STA_MAX + NX_VIRT_DEV_MAX)

#define TRACE_FHOST(fmt, ...)   aic_dbg(fmt, ## __VA_ARGS__)

typedef struct netif            net_if_t;

/// Type of messages sent using @ref fhost_msg structure
enum fhost_msg_type {
    /**
     * Message received from IPC layer (for test/development only)
     */
    FHOST_MSG_IPC,
    /**
     * Messages received from WIFI stack
     */
    FHOST_MSG_KE_WIFI,
    /**
     * Configuration/control messages received from supplicant application
     */
    FHOST_MSG_CFGRWNX,
    /**
     * Message received from console layer (for test/development only)
     */
    FHOST_MSG_CONSOLE,
    /**
     * Messages received from tx cfm callback (for 802.11 frame)
     */
    FHOST_MSG_TXCFM_CB,
    /**
     * Messages received from wifi api
     */
    FHOST_MSG_WIFI_API,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// STA Info Table
struct sta_info_tag
{
    /// linked list header
    struct co_list_hdr list_hdr;

    /// Interface the station belongs to
    uint8_t inst_nbr;
    /// Station index
    uint8_t staid;
    /// Power save state of the associated STA
    //uint8_t ps_state;
    /// Flag indicating if the STA entry is valid or not
    bool valid;
    /// MAC address of the STA
    struct mac_addr mac_addr;

    uint32_t last_active_time_us;
};
/// Information related to the BSS a VIF is linked to
struct me_bss_info
{
    /// Network BSSID.
    struct mac_addr bssid;
};
/// VIF Info Table
struct vif_info_tag
{
    /// linked list header
    //struct co_list_hdr list_hdr;
    /// Bitfield indicating if this VIF currently allows sleep or not
    uint32_t prevent_sleep;
    /// EDCA parameters of the different TX queues for this VIF
    uint32_t txq_params[AC_MAX];

    #if (NX_MULTI_ROLE || NX_CHNL_CTXT || (NX_P2P_GO && NX_POWERSAVE))
    /// TBTT timer structure
    struct mm_timer_tag tbtt_timer;
    #endif //(NX_MULTI_ROLE || NX_CHNL_CTXT || (NX_P2P_GO && NX_POWERSAVE))

    #if (NX_MULTI_ROLE || NX_TDLS)
    /// BSSID this VIF belongs to
    struct mac_addr bssid;
    #endif //(NX_MULTI_ROLE)

    struct mac_chan_op chan;

    /// MAC address of the VIF
    struct mac_addr mac_addr;

    /// Type of the interface (@ref VIF_STA, @ref VIF_IBSS, @ref VIF_MESH_POINT or @ref VIF_AP)
    uint8_t type;
    /// Index of the interface
    uint8_t index;
    /// Flag indicating if the VIF is active or not
    bool active;

    /// TX power configured for the interface (dBm)
    int8_t tx_power;

    /// TX power configured for the interface (dBm) by user space
    /// (Taken into account only if lower than regulatory one)
    int8_t user_tx_power;

    union
    {
        /// STA specific parameter structure
        struct
        {
            #if NX_POWERSAVE
            /// Listen interval
            uint16_t listen_interval;
            /// Flag indicating if we are expecting BC/MC traffic or not
            bool dont_wait_bcmc;
            /// Number of error seen during transmission of last NULL frame indicating PS change
            uint8_t ps_retry;
            #endif
            /// Index of the station being the peer AP
            uint8_t ap_id;
            #if NX_UAPSD
            /// Time of last UAPSD transmitted/received frame
            uint32_t uapsd_last_rxtx;
            /// Bitfield indicating which queues are U-APSD enabled
            uint8_t uapsd_queues;
            /// UAPSD highest TID, used in the QoS-NULL trigger frames
            uint8_t uapsd_tid;
            #endif
            #if NX_CONNECTION_MONITOR
            /// Time of last keep-alive frame sent to AP
            uint32_t mon_last_tx;
            /// CRC of last received beacon
            uint32_t mon_last_crc;
            /// Number of beacon losses since last beacon reception
            uint8_t beacon_loss_cnt;
            #endif

            #if (NX_P2P)
            /// Last computed TSF Offset
            int32_t last_tsf_offset;
            /// Addition duration to be added to the CTWindow, due to the TBTT_DELAY + drift value computed in mm_tbtt_compute
            uint32_t ctw_add_dur;
            /// Status indicated if Service Period has been paused due to GO absence
            bool sp_paused;
            #endif //(NX_P2P)

            #if (NX_P2P_GO)
            // Indicate if AP Beacon has been received at least one time
            bool bcn_rcved;
            #endif //(NX_P2P_GO)

            // Current RSSI
            int8_t rssi;
            // RSSI threshold (0=threshold not set)
            int8_t rssi_thold;
            // RSSI hysteresis
            uint8_t rssi_hyst;
            // Current status of RSSI (0=RSSI is high, 1=RSSI is low)
            bool rssi_status;

            /// Current CSA counter
            uint8_t csa_count;
            /// Indicate if channel switch (due to CSA) just happened
            bool csa_occured;

            #if (NX_TDLS)
            /// TDLS station index which requested the channel switch
            uint8_t tdls_chsw_sta_idx;
            #endif
            uint8_t vif_name[33];
        } sta;
        /// AP specific parameter structure
        struct
        {
            uint32_t dummy;
            /// Flag indicating how many connected stations are currently in PS
            uint8_t ps_sta_cnt;
            /// Control port ethertype
            uint16_t ctrl_port_ethertype;
            /// Current CSA counter
            uint8_t csa_count;

            rtos_semaphore csa_semaphore;
        } ap;
    } u;    ///< Union of AP/STA specific parameter structures

    /// List of stations linked to this VIF
    struct co_list sta_list;

    /// Information about the BSS linked to this VIF
    struct me_bss_info bss_info;

    #if NX_MAC_HE
    /// TXOP RTS threshold
    uint16_t txop_dur_rts_thres;
    #endif
    #if (NX_P2P)
    /// Indicate if this interface is configured for P2P operations
    bool p2p;
    /// Index of the linked P2P Information structure
    uint8_t p2p_index;
    /// Contain current number of registered P2P links for the VIF
    uint8_t p2p_link_nb;
    #endif //(NX_P2P)

    #if (RW_UMESH_EN)
    /// Mesh ID - Index of the used mesh_vif_info_tag structure when type is VIF_MESH_POINT
    uint8_t mvif_idx;
    #endif //(RW_UMESH_EN)
};

/// Structure containing the information about the virtual interfaces
struct fhost_vif_tag
{
    /// RTOS network interface structure
    net_if_t *net_if;
    /// MAC address of the VIF
    struct mac_addr mac_addr;
    /// Socket for scan events
    int scan_sock;
    /// Socket for connect/disconnect events
    int conn_sock;
    /// Socket for AP
    int ap_sock;
    /// Socket for p2p
    int p2p_sock;
    /// Pointer to the MAC VIF structure
    struct vif_info_tag *mac_vif;
    /// Index of the STA being the AP peer of the device - TODO rework
    uint8_t ap_id;
    /// Parameter to indicate if admission control is mandatory for any access category - TODO rework
    uint8_t acm;
    /// UAPSD queue config for STA interface (bitfield, same format as QoS info)
    uint8_t uapsd_queues;
    /// connect router band
    uint8_t band;
    uint8_t chan_index;
};

/// Structure used for the inter-task communication
struct fhost_env_tag
{
    /// Table of RTOS network interface structures
    struct fhost_vif_tag vif[NX_VIRT_DEV_MAX];
    /// Table linking the MAC VIFs to the FHOST VIFs
    struct fhost_vif_tag *mac2fhost_vif[NX_VIRT_DEV_MAX];
};

/// Generate fhost msg ID from a type and an index
#define FHOST_MSG_ID(type, idx) ((type << 12) | (idx & 0xfff))
/// Extract msg Type from msg ID
#define FHOST_MSG_TYPE(id) ((id >> 12) & 0xf)
/// Extract msg Index from msg ID
#define FHOST_MSG_INDEX(id) (id & 0xfff)

/// Generic Message format
struct fhost_msg {
    /// ID of the message. Id is a combination of a type and an index.
    /// To be set using @ref FHOST_MSG_ID macro
    uint16_t id;
    /// Length, in bytes, of the message
    uint16_t len;
    /// Pointer to the message
    void *data;
};

#define TR_32(a) (uint16_t)((uint32_t)(a) >> 16), (uint16_t)((uint32_t)(a))

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// FullHost module environment variable
extern struct fhost_env_tag fhost_env;

/// include here to avoid conflict
#include "netif_port.h"

/*
 * FUNCTIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Get MAC VIF owned by a FHOST VIF.
 *
 * @param[in] fhost_vif_idx Index of the FHOST VIF
 *
 * @return A pointer to the corresponding MAC VIF
 ****************************************************************************************
 */
__STATIC_INLINE struct vif_info_tag *fhost_to_mac_vif(uint8_t fhost_vif_idx)
{
    struct vif_info_tag *mac_vif = fhost_env.vif[fhost_vif_idx].mac_vif;

    // Sanity check - Currently we consider that when this function is called there shall
    // be a MAC VIF attached to the FHOST VIF. If in the future this has to change then
    // this assertion will be removed
    AIC_ASSERT_ERR(mac_vif != NULL);

    return mac_vif;
}

int fhost_mac_vif_valid(uint8_t fhost_vif_idx);

/**
 ****************************************************************************************
 * @brief Get FHOST VIF owner of a MAC VIF.
 *
 * @param[in] mac_vif_idx Index of the MAC VIF
 *
 * @return A pointer to the corresponding FHOST VIF
 ****************************************************************************************
 */
struct fhost_vif_tag *fhost_from_mac_vif(uint8_t mac_vif_idx);


/**
 ****************************************************************************************
 * @brief Get Network interface associated to a FHOST VIF.
 *
 * @param[in] fhost_vif_idx Index of the FHOST VIF
 *
 * @return A pointer to the corresponding network interface
 ****************************************************************************************
 */
net_if_t *fhost_to_net_if(uint8_t fhost_vif_idx);

/**
 ****************************************************************************************
 * @brief Get FHOST VIF name.
 *
 * Copy name of a FHOST VIF inside provided buffer including a terminating a null byte.
 * If the buffer is not big enough then interface name is truncated and the null byte
 * is not written in the buffer.
 *
 * @param[in] fhost_vif_idx  Index of the FHOST VIF
 * @param[in] name           Buffer to retrieve interface name
 * @param[in] len            Size, in bytes, of the @p name buffer
 *
 * @return < 0 if error occurred, otherwise the number of characters (excluding the
 * terminating null byte) needed to write the interface name. If return value is greater
 * or equal to @p len, it means that the interface name has been truncated.
 ****************************************************************************************
 */
int fhost_vif_name(int fhost_vif_idx, char *name, int len);

struct fhost_vif_tag * fhost_vif_get_first_free_itf(void);

/**
 ****************************************************************************************
 * @brief Get FHOST VIF index from its name.
 *
 * @param[in] name Interface name
 *
 * @return index of the fhost vif and < 0 if there is no interface with the provided name
 ****************************************************************************************
 */
int fhost_vif_idx_from_name(const char *name);

int fhost_vif_idx_from_vif_tag(struct fhost_vif_tag *tag);

/**
 ****************************************************************************************
 * @brief Configure default queues enabled for U-APSD.
 *
 * This configuration is used when an interface configured as STA connects to an AP that
 * supports U-APSD. This can be called at any time, but the configuration will only be
 * applied for the next connections.
 *
 * @param[in] fhost_vif_idx  Index of the FHOST VIF. (Use -1 to configure all interfaces)
 * @param[in] uapsd_queues   AC bitfield as expected in Qos Info field
 *                           (i.e. Bit0=VO, Bit1=VI, Bit2=BK, bit3=BE)
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_vif_set_uapsd_queues(int fhost_vif_idx, uint8_t uapsd_queues);

/**
 ****************************************************************************************
 * @brief Initialization of the application.
 *
 * Called during the initialization procedure (i.e. when RTOS scheduler is not yet active).
 * Implementation of this function will depends of the final application and in most
 * cases it will create one of several application tasks and their required communication
 * interface (queue, semaphore, ...)
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_application_init(void);

/**
 ****************************************************************************************
 * @brief Create a socket and connect it to loopback address on the specified port
 *
 * @param[in] port UDP port to connect to
 * @return socket file descriptor and <0 if error occurred
 ****************************************************************************************
 */
int fhost_open_loopback_udp_sock(int port);

/**
 ****************************************************************************************
 * @brief Print a pre-formatted string buffer.
 *
 * Implementation of this function will depend of target/application
 *
 * @param[in] handle Task handle of the RTOS task sending the message
 *                   (can be null to indicate current task)
 * @param[in] buf    Formatted string buffer
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_print_buf(rtos_task_handle handle, const char *buf);

/**
 ****************************************************************************************
 * @brief Print a message.
 *
 * Implementation of this function will depend of target/application
 *
 * @param[in] handle Task handle of the RTOS task sending the message
 *                   (can be null to indicate current task)
 * @param[in] fmt    Format string
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_print(rtos_task_handle handle, const char *fmt, ...);


/**
 ****************************************************************************************
 * @brief Compute checksum on a given IP packet
 *
 * Compute IP packet checksum using HSU and fallback to software computation if HSU is
 * unavailable
 *
 * @param[in] dataptr Buffer containing the IP packet
 * @param[in] len     Buffer length (in bytes)
 * @return IP packet checksum
 ****************************************************************************************
 */
uint16_t fhost_ip_chksum(const void *dataptr, int len);


/**
 ****************************************************************************************
 * @brief Connect a STA interface to an AP
 *
 * This is blocking until connection is successful.
 *
 * @param[in] fhost_vif_idx  Fhost VIF index
 * @param[in] cfg            Interface configuration
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_sta_cfg(int fhost_vif_idx, struct fhost_vif_sta_cfg *cfg);

/**
 ****************************************************************************************
 * @brief Start an Access Point
 *
 * This is blocking until AP is operational.
 *
 * @param[in] fhost_vif_idx  Fhost VIF index
 * @param[in] cfg            AP configuration
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_ap_cfg(int fhost_vif_idx, struct fhost_vif_ap_cfg *cfg);

int fhost_p2p_cfg(int fhost_vif_idx);
int fhost_p2p_go_cfg(int fhost_vif_idx, struct fhost_vif_p2p_cfg *cfg);

extern struct vif_info_tag vif_info_tab[NX_VIRT_DEV_MAX];
extern struct sta_info_tag sta_info_tab[STA_MAX + NX_VIRT_DEV_MAX];
extern int wlan_connected;

extern void fhost_data_save(void);
extern void fhost_data_restore(void);
extern void fhost_sta_recover_connection(void);
extern void fhost_sta_ipc_rxbuf_recover(void);
extern uint8_t vif_mgmt_get_staid(const struct vif_info_tag *vif, const struct mac_addr *sta_addr);
struct sta_info_tag * vif_mgmt_get_sta_by_addr(const struct mac_addr *sta_addr);
struct sta_info_tag * vif_mgmt_get_sta_by_staid(uint8_t sta_id);
extern int fhost_ipc_cntrl_init(uint32_t ipc_irq_prio);
extern int ipc_host_cntrl_start(void);
extern void *get_vif_mgmt_sta_list(void);
extern uint8_t vif_mgmt_sta_cnt(void);

extern int ipc_host_fw_init(void);

extern struct co_list free_sta_list;

typedef enum wifi_mac_status    {
    WIFI_MAC_STATUS_DISCONNECTED,
    WIFI_MAC_STATUS_CONNECTED,
}wifi_mac_status_e;
typedef void (*fhost_mac_status_get_func_t)(wifi_mac_status_e st);
void fhost_get_mac_status_register(fhost_mac_status_get_func_t func);

extern fhost_mac_status_get_func_t fhost_mac_status_get_callback;

uint8_t user_limit_sta_num_get(void);
void user_limit_sta_num_set(uint8_t num);


extern fhost_mac_status_get_func_t fhost_reconnect_dhcp_callback;
void fhost_reconnect_dhcp_register(fhost_mac_status_get_func_t func);

int fhost_scan_for_ssid_pwd(struct fhost_cntrl_link *link, int fvif_idx, uint8_t *p_ssid, uint8_t *p_password);

#endif // _FHOST_H_