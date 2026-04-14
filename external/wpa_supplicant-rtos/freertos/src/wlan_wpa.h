/*
 *  Copyright 2008-2024 NXP
 *  Copyright 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*!\file wlan.h
 *\brief This file provides Wi-Fi APIs for the application.
 */

#ifndef __WLAN_WIFI_H__
#define __WLAN_WIFI_H__

#include <stdint.h>
#include <wifi_events.h>
//#include "linux_compat.h"
#define WLAN_DRV_VERSION "v1.3.r48.p25"

#if CONFIG_WPA2_ENTP
    #include <wm_mbedtls_helper_api.h>
#endif

#define ARG_UNUSED(x) (void)(x)
/* Configuration */

#if !CONFIG_WLAN_KNOWN_NETWORKS
    #define CONFIG_WLAN_KNOWN_NETWORKS 5U
#endif

//#include <wmlog.h>
#define wlcm_e(...) wmlog_e("wlcm", ##__VA_ARGS__)
#define wlcm_w(...) wmlog_w("wlcm", ##__VA_ARGS__)

#if CONFIG_WLCMGR_DEBUG
    #define wlcm_d(...) wmlog("wlcm", ##__VA_ARGS__)
#else
    #define wlcm_d(...) rt_kprintf
#endif /* ! CONFIG_WLCMGR_DEBUG */

#if (defined(configSUPPORT_STATIC_ALLOCATION) && (configSUPPORT_STATIC_ALLOCATION > 0U)) && \
    !((defined(configSUPPORT_DYNAMIC_ALLOCATION) && (configSUPPORT_DYNAMIC_ALLOCATION == 1U)))

    #if CONFIG_WPA_SUPP
        #error "Static memory allocation is not supported for wpa supplicant "
    #endif

#endif

/** Action GET */
#define ACTION_GET (0U)
/** Action SET */
#define ACTION_SET (1)

/** Maximum SSID length */
#ifndef IEEEtypes_SSID_SIZE
    #define IEEEtypes_SSID_SIZE 32U
#endif /* IEEEtypes_SSID_SIZE */

/** MAC Address length */
#ifndef IEEEtypes_ADDRESS_SIZE
    #define IEEEtypes_ADDRESS_SIZE 6
#endif /* IEEEtypes_ADDRESS_SIZE */

#if CONFIG_HOST_SLEEP
    #if CONFIG_POWER_MANAGER
        extern osa_msg_handle_t mon_thread_event_queue;
    #endif
#endif

#define WLAN_REASON_CODE_PREV_AUTH_NOT_VALID 2U

typedef enum
{
    BSS_INFRASTRUCTURE = 1,
    BSS_INDEPENDENT,
    BSS_ANY
} IEEEtypes_Bss_t;

/* The possible types of basic service sets */

/** The number of times that the Wi-Fi connection manager look for a
 *  network before giving up. */
#if CONFIG_MAX_RESCAN_LIMIT
    #define WLAN_RESCAN_LIMIT CONFIG_MAX_RESCAN_LIMIT
#else
    #if CONFIG_WPA_SUPP
        #define WLAN_RESCAN_LIMIT 30U
    #else
        #define WLAN_RESCAN_LIMIT 5U
    #endif /* CONFIG_WPA_SUPP */
#endif /* CONFIG_MAX_RESCAN_LIMIT */

#define WLAN_11D_SCAN_LIMIT 3U
/** The number of times that the Wi-Fi connection manager attempts a
 * reconnection with the network before giving up. */
#ifndef WLAN_RECONNECT_LIMIT
    #define WLAN_RECONNECT_LIMIT 5U
#endif
/** The minimum length for network names, see \ref wlan_network. This should
 *  be between 1 and \ref WLAN_NETWORK_NAME_MAX_LENGTH */
#define WLAN_NETWORK_NAME_MIN_LENGTH 1U
/** The space reserved for storing network names, \ref wlan_network */
#define WLAN_NETWORK_NAME_MAX_LENGTH 32U
/** The space reserved for storing PSK (password) phrases. */
/* Min WPA2 passphrase can be upto 8 ASCII chars */
#define WLAN_PSK_MIN_LENGTH 8U
/** Max WPA2 passphrase can be upto 63 ASCII chars or 64 hexadecimal digits*/
#define WLAN_PSK_MAX_LENGTH 65U
/** Min WPA3 password can be upto 8 ASCII chars */
#define WLAN_PASSWORD_MIN_LENGTH 8U
/** Max WPA3 password can be upto 255 ASCII chars */
#define WLAN_PASSWORD_MAX_LENGTH 255U
/** Max WPA2 Enterprise identity can be upto 256 characters */
#define IDENTITY_MAX_LENGTH 64U
/** Max WPA2 Enterprise password can be upto 256 unicode characters */
#define PASSWORD_MAX_LENGTH 128U
/** Max identities for EAP server users */
#define MAX_USERS 8U
/** Encryption key for EAP-FAST PAC-Opaque values. This key is a secret, random value. It is configured as a
 * 16-octet value in hex format. */
#define PAC_OPAQUE_ENCR_KEY_MAX_LENGTH 33U
/** A-ID indicates the identity of the authority that issues PACs. The A-ID should be unique across all issuing servers.
 * A-ID to be 16 octets in length */
#define A_ID_MAX_LENGTH 33U
/** MAX CA Cert hash len */
#define HASH_MAX_LENGTH 40U
/** MAX domain len */
#define DOMAIN_MATCH_MAX_LENGTH 64U

#if CONFIG_WLAN_KNOWN_NETWORKS
    /** The size of the list of known networks maintained by the Wi-Fi connection manager */
    #define WLAN_MAX_KNOWN_NETWORKS CONFIG_WLAN_KNOWN_NETWORKS
#else
    #error "CONFIG_WLAN_KNOWN_NETWORKS is not defined"
#endif /* CONFIG_WLAN_KNOWN_NETWORKS */
/** Length of a pairwise master key (PMK).  It's always 256 bits (32 Bytes) */
#define WLAN_PMK_LENGTH 32

#if CONFIG_WMM_UAPSD
    #define WMM_UAPSD_QOS_INFO     0x0F
    #define WMM_UAPSD_SLEEP_PERIOD 20
#endif

#if CONFIG_UAP_STA_MAC_ADDR_FILTER

    /* Maximum number of STA filter list can be upto 16 */
    #define WLAN_MAX_STA_FILTER_NUM 16

    /* The length of Wi-Fi MAC address */
    #define WLAN_MAC_ADDR_LENGTH 6
#endif

/** Error codes */
/** The operation was successful. */
#define WLAN_ERROR_NONE 0
/** The operation failed due to an error with one or more parameters. */
#define WLAN_ERROR_PARAM 1
/** The operation could not be performed because there is not enough memory. */
#define WLAN_ERROR_NOMEM 2
/** The operation could not be performed in the current system state. */
#define WLAN_ERROR_STATE 3
/** The operation failed due to an internal error. */
#define WLAN_ERROR_ACTION 4
/** The operation to change power state could not be performed*/
#define WLAN_ERROR_PS_ACTION 5
/** The requested feature is not supported*/
#define WLAN_ERROR_NOT_SUPPORTED 6

/*
 * HOST_WAKEUP_GPIO_PIN / CARD_WAKEUP_GPIO_PIN
 *
 *   Default GPIO pin number. This is chip
 *   specific, and a compile time setting depending on the system
 *   board level build!
 */
#if defined(SD8997) || defined(SD9098) || defined(SD9064) || defined(RW610)
    #define HOST_WAKEUP_GPIO_PIN 12
    #define CARD_WAKEUP_GPIO_PIN 13
#elif defined(SD9177)
    #define HOST_WAKEUP_GPIO_PIN 17
    #define CARD_WAKEUP_GPIO_PIN 16
#elif defined(WIFI_88W8987_BOARD_MURATA_1ZM_M2)
    #define HOST_WAKEUP_GPIO_PIN 13
    #define CARD_WAKEUP_GPIO_PIN 16
#elif defined(WIFI_IW416_BOARD_MURATA_1XK_M2)
    #define HOST_WAKEUP_GPIO_PIN 2
    #define CARD_WAKEUP_GPIO_PIN 16
#else
    #define HOST_WAKEUP_GPIO_PIN 1
    #define CARD_WAKEUP_GPIO_PIN 16
#endif

#define WLAN_MGMT_DIASSOC MBIT(10)
#define WLAN_MGMT_AUTH    MBIT(11)
#define WLAN_MGMT_DEAUTH  MBIT(12)
/** BITMAP for Action frame */
#define WLAN_MGMT_ACTION MBIT(13)

#if CONFIG_WMM_UAPSD
    #define WMM_UAPSD_QOS_INFO     0x0F
    #define WMM_UAPSD_SLEEP_PERIOD 20
#endif

#define WLAN_KEY_MGMT_IEEE8021X             MBIT(0)
#define WLAN_KEY_MGMT_PSK                   MBIT(1)
#define WLAN_KEY_MGMT_NONE                  MBIT(2)
#define WLAN_KEY_MGMT_IEEE8021X_NO_WPA      MBIT(3)
#define WLAN_KEY_MGMT_WPA_NONE              MBIT(4)
#define WLAN_KEY_MGMT_FT_IEEE8021X          MBIT(5)
#define WLAN_KEY_MGMT_FT_PSK                MBIT(6)
#define WLAN_KEY_MGMT_IEEE8021X_SHA256      MBIT(7)
#define WLAN_KEY_MGMT_PSK_SHA256            MBIT(8)
#define WLAN_KEY_MGMT_WPS                   MBIT(9)
#define WLAN_KEY_MGMT_SAE                   MBIT(10)
#define WLAN_KEY_MGMT_FT_SAE                MBIT(11)
#define WLAN_KEY_MGMT_WAPI_PSK              MBIT(12)
#define WLAN_KEY_MGMT_WAPI_CERT             MBIT(13)
#define WLAN_KEY_MGMT_CCKM                  MBIT(14)
#define WLAN_KEY_MGMT_OSEN                  MBIT(15)
#define WLAN_KEY_MGMT_IEEE8021X_SUITE_B     MBIT(16)
#define WLAN_KEY_MGMT_IEEE8021X_SUITE_B_192 MBIT(17)
#define WLAN_KEY_MGMT_FILS_SHA256           MBIT(18)
#define WLAN_KEY_MGMT_FILS_SHA384           MBIT(19)
#define WLAN_KEY_MGMT_FT_FILS_SHA256        MBIT(20)
#define WLAN_KEY_MGMT_FT_FILS_SHA384        MBIT(21)
#define WLAN_KEY_MGMT_OWE                   MBIT(22)
#define WLAN_KEY_MGMT_DPP                   MBIT(23)
#define WLAN_KEY_MGMT_FT_IEEE8021X_SHA384   MBIT(24)
#define WLAN_KEY_MGMT_PASN                  MBIT(25)
#define WLAN_KEY_MGMT_SAE_EXT_KEY           MBIT(26)

#define WLAN_KEY_MGMT_FT                                                                                            \
    (WLAN_KEY_MGMT_FT_PSK | WLAN_KEY_MGMT_FT_IEEE8021X | WLAN_KEY_MGMT_FT_IEEE8021X_SHA384 | WLAN_KEY_MGMT_FT_SAE | \
     WLAN_KEY_MGMT_FT_FILS_SHA256 | WLAN_KEY_MGMT_FT_FILS_SHA384)

#if CONFIG_WPA_SUPP

    #define WLAN_CIPHER_NONE         MBIT(0)
    #define WLAN_CIPHER_WEP40        MBIT(1)
    #define WLAN_CIPHER_WEP104       MBIT(2)
    #define WLAN_CIPHER_TKIP         MBIT(3)
    #define WLAN_CIPHER_CCMP         MBIT(4)
    #define WLAN_CIPHER_AES_128_CMAC MBIT(5)
    #define WLAN_CIPHER_GCMP         MBIT(6)
    #define WLAN_CIPHER_SMS4         MBIT(7)
    #define WLAN_CIPHER_GCMP_256     MBIT(8)
    #define WLAN_CIPHER_CCMP_256     MBIT(9)
    #define WLAN_CIPHER_BIP_GMAC_128 MBIT(11)
    #define WLAN_CIPHER_BIP_GMAC_256 MBIT(12)
    #define WLAN_CIPHER_BIP_CMAC_256 MBIT(13)
    #define WLAN_CIPHER_GTK_NOT_USED MBIT(14)

#endif
#ifndef MOD_ERROR_START
#define MOD_ERROR_START(x) ((x) << 12 | 0)
#endif
#ifndef MOD_WLAN
#define MOD_WLAN 24
#endif
/** Enum for Wi-Fi errors */
enum wm_wlan_errno
{
    WM_E_WLAN_ERRNO_BASE = MOD_ERROR_START(MOD_WLAN),
    /** The firmware download operation failed. */
    WLAN_ERROR_FW_DNLD_FAILED,
    /** The firmware ready register not set. */
    WLAN_ERROR_FW_NOT_READY,
    /** The Wi-Fi card not found. */
    WLAN_ERROR_CARD_NOT_DETECTED,
    /** The Wi-Fi Firmware not found. */
    WLAN_ERROR_FW_NOT_DETECTED,
    /** BSSID not found in scan list */
    WLAN_BSSID_NOT_FOUND_IN_SCAN_LIST,
};

/* Events and states */

/** Wi-Fi connection manager event reason */
enum wlan_event_reason
{
    /** The Wi-Fi connection manager has successfully connected to a network and
     *  is now in the \ref WLAN_CONNECTED state. */
    WLAN_REASON_SUCCESS,
    /** The Wi-Fi connection manager has successfully authenticated to a network and
     *  is now in the \ref WLAN_ASSOCIATED state. */
    WLAN_REASON_AUTH_SUCCESS,
    /** The Wi-Fi connection manager failed to connect before actual
     * connection attempt with AP due to incorrect Wi-Fi network profile.
     * or the Wi-Fi connection manager failed to reconnect to previously connected
     * network and it is now in the \ref WLAN_DISCONNECTED state.*/
    WLAN_REASON_CONNECT_FAILED,
    /** The Wi-Fi connection manager could not find the network that it was
     *  connecting to and it is now in the \ref WLAN_DISCONNECTED state. */
    WLAN_REASON_NETWORK_NOT_FOUND,
    /** The Wi-Fi connection manager could not find the network in background scan during roam attempt that it was
     *  connecting to and it is now in the \ref WLAN_CONNECTED state with previous AP. */
    WLAN_REASON_BGSCAN_NETWORK_NOT_FOUND,
    /** The Wi-Fi connection manager failed to authenticate with the network
     *  and is now in the \ref WLAN_DISCONNECTED state. */
    WLAN_REASON_NETWORK_AUTH_FAILED,
    /** DHCP lease has been renewed.*/
    WLAN_REASON_ADDRESS_SUCCESS,
    /** The Wi-Fi connection manager failed to obtain an IP address
     *  or TCP stack configuration has failed or the IP address
     *  configuration was lost due to a DHCP error.  The system is
     *  now in the \ref WLAN_DISCONNECTED state. */
    WLAN_REASON_ADDRESS_FAILED,
    /** The Wi-Fi connection manager has lost the link to the current network. */
    WLAN_REASON_LINK_LOST,
    /** The Wi-Fi connection manager has received the channel switch
     * announcement from the current network. */
    WLAN_REASON_CHAN_SWITCH,
    /** The Wi-Fi connection manager has disconnected from the WPS network
     *  (or has canceled a connection attempt) by request and is now in the
     *  WLAN_DISCONNECTED state. */
    WLAN_REASON_WPS_DISCONNECT,
    /** The Wi-Fi connection manager has disconnected from the current network
     *  (or has canceled a connection attempt) by request and is now in the
     *  WLAN_DISCONNECTED state. */
    WLAN_REASON_USER_DISCONNECT,
    /** The Wi-Fi connection manager is initialized and is ready for use.
     *  That is, it's now possible to scan or to connect to a network. */
    WLAN_REASON_INITIALIZED,
    /** The Wi-Fi connection manager has failed to initialize and is therefore
     *  not running. It is not possible to scan or to connect to a network.  The
     *  Wi-Fi connection manager should be stopped and started again via
     *  wlan_stop() and wlan_start() respectively. */
    WLAN_REASON_INITIALIZATION_FAILED,
#if (CONFIG_WIFI_IND_DNLD)
    /** The Wi-Fi connection manager has entered in hang mode. */
    WLAN_REASON_FW_HANG,
    /** The Wi-Fi connection manager has reset fw successfully. */
    WLAN_REASON_FW_RESET,
#endif
    /** The Wi-Fi connection manager has entered power save mode. */
    WLAN_REASON_PS_ENTER,
    /** The Wi-Fi connection manager has exited from power save mode. */
    WLAN_REASON_PS_EXIT,
    /** The Wi-Fi connection manager has started UAP */
    WLAN_REASON_UAP_SUCCESS,
    /** A Wi-Fi client has joined UAP's BSS network */
    WLAN_REASON_UAP_CLIENT_ASSOC,
    /** A Wi-Fi client has auhtenticated and connected to UAP's BSS network */
    WLAN_REASON_UAP_CLIENT_CONN,
    /** A Wi-Fi client has left UAP's BSS network */
    WLAN_REASON_UAP_CLIENT_DISSOC,
    /** The Wi-Fi connection manager has failed to start UAP */
    WLAN_REASON_UAP_START_FAILED,
    /** The Wi-Fi connection manager has failed to stop UAP */
    WLAN_REASON_UAP_STOP_FAILED,
    /** The Wi-Fi connection manager has stopped UAP */
    WLAN_REASON_UAP_STOPPED,
    /** The Wi-Fi connection manager has received subscribed RSSI low event on station interface as per configured
       threshold and frequency. If CONFIG_11K, CONFIG_11V, CONFIG_11R or CONFIG_ROAMING enabled then RSSI low event is
       processed internally.*/
    WLAN_REASON_RSSI_LOW,
#if CONFIG_SUBSCRIBE_EVENT_SUPPORT
    /** The Wi-Fi connection manager has received subscribed RSSI high event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_RSSI_HIGH,
    /** The Wi-Fi connection manager has received subscribed SNR low event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_SNR_LOW,
    /** The Wi-Fi connection manager has received subscribed SNR high event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_SNR_HIGH,
    /** The Wi-Fi connection manager has received subscribed maximum fail event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_MAX_FAIL,
    /** The Wi-Fi connection manager has received subscribed beacon missed fail event on station interface as per
       configured threshold and frequency. */
    WLAN_REASON_BEACON_MISSED,
    /** The Wi-Fi connection manager has received subscribed data RSSI low event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_DATA_RSSI_LOW,
    /** The Wi-Fi connection manager has received subscribed data RSSI high event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_DATA_RSSI_HIGH,
    /** The Wi-Fi connection manager has received subscribed data SNR low event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_DATA_SNR_LOW,
    /** The Wi-Fi connection manager has received subscribed data SNR high event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_DATA_SNR_HIGH,
    /** The Wi-Fi connection manager has received subscribed link quality event on station interface as per configured
    link_snr threshold and frequency, link_rate threshold and frequency, link_tx_latency threshold and frequency*/
    WLAN_REASON_LINK_QUALITY,
    /** The Wi-Fi connection manager has received subscribed pre beacon lost event on station interface as per configured
       threshold and frequency. */
    WLAN_REASON_PRE_BEACON_LOST,
#endif
#if CONFIG_NCP
    /** Scan is done */
    WLAN_REASON_SCAN_DONE,
    /** WPS session is done */
    WLAN_REASON_WPS_SESSION_DONE,
#endif
};

/** Wakeup event bitmap */
enum wlan_wakeup_event_t
{
    /** Wakeup on broadcast  */
    WAKE_ON_ALL_BROADCAST = 1,
    /** Wakeup on unicast  */
    WAKE_ON_UNICAST = 1 << 1,
    /** Wakeup on MAC event  */
    WAKE_ON_MAC_EVENT = 1 << 2,
    /** Wakeup on multicast  */
    WAKE_ON_MULTICAST = 1 << 3,
    /** Wakeup on ARP broadcast  */
    WAKE_ON_ARP_BROADCAST = 1 << 4,
    /** Wakeup on receiving a management frame  */
    WAKE_ON_MGMT_FRAME = 1 << 6,
};

/** Wi-Fi station/UAP/Wi-Fi direct connection/status state */
enum wlan_connection_state
{
    /** The Wi-Fi connection manager is not connected and no connection attempt
     *  is in progress. It is possible to connect to a network or scan. */
    WLAN_DISCONNECTED,
    /** The Wi-Fi connection manager is not connected but it is currently
     *  attempting to connect to a network.  It is not possible to scan at this
     *  time.  It is possible to connect to a different network. */
    WLAN_CONNECTING,
    /** The Wi-Fi connection manager is not connected but associated. */
    WLAN_ASSOCIATED,
    /** The Wi-Fi connection manager is not connected but authenticated. */
    WLAN_AUTHENTICATED,
    /** The Wi-Fi connection manager is connected. It is possible to scan and
     *  connect to another network at this time. Information about the current
     *  network configuration is available. */
    WLAN_CONNECTED,
    /** The Wi-Fi connection manager has started UAP */
    WLAN_UAP_STARTED,
    /** The Wi-Fi connection manager has stopped UAP */
    WLAN_UAP_STOPPED,
    /** The Wi-Fi connection manager is not connected and network scan
     * is in progress. */
    WLAN_SCANNING,
    /** The Wi-Fi connection manager is not connected and network association
     * is in progress. */
    WLAN_ASSOCIATING,
};

/* Data Structures */

/** Station power save mode */
typedef enum wlan_ps_mode
{
    /** Active mode */
    WLAN_ACTIVE_W = 0,
    /** IEEE power save mode */
    WLAN_IEEE,
    /** Deep sleep power save mode */
    WLAN_DEEP_SLEEP,
    /** IEEE and deep sleep power save mode */
    WLAN_IEEE_DEEP_SLEEP,
#if CONFIG_WNM_PS
    /** WNM power save mode */
    WLAN_WNM,
    /** WNM and Deep sleep power save mode */
    WLAN_WNM_DEEP_SLEEP,
#endif
} wlan_ps_mode;

enum wlan_ps_state
{
    PS_STATE_AWAKE = 0,
    PS_STATE_PRE_SLEEP,
    PS_STATE_SLEEP_CFM,
    PS_STATE_SLEEP
};
#define CONFIG_TEST 1
typedef enum _ENH_PS_MODES
{
    GET_PS        = 0,
    SLEEP_CONFIRM = 5,
    EXT_PS_PARAM  = 6,
#if (CONFIG_WNM_PS)
    DIS_WNM_PS = 0xfc,
    EN_WNM_PS  = 0xfd,
#endif
    DIS_AUTO_PS = 0xfe,
    EN_AUTO_PS  = 0xff,
} ENH_PS_MODES;

typedef enum _Host_Sleep_Action
{
    HS_CONFIGURE = 0x0001,
    HS_ACTIVATE  = 0x0002,
} Host_Sleep_Action;

#if (CONFIG_WNM_PS)
typedef  struct
{
    uint8_t action;
    uint8_t result;
}  wnm_sleep_result_t;
#endif

#if CONFIG_CSI
enum wlan_csi_opt
{
    CSI_FILTER_OPT_ADD = 0,
    CSI_FILTER_OPT_DELETE,
    CSI_FILTER_OPT_CLEAR,
    CSI_FILTER_OPT_DUMP,
};
#endif

enum wlan_monitor_opt
{
    MONITOR_FILTER_OPT_ADD_MAC = 0,
    MONITOR_FILTER_OPT_DELETE_MAC,
    MONITOR_FILTER_OPT_CLEAR_MAC,
    MONITOR_FILTER_OPT_DUMP,
};

#if (CONFIG_11MC) || (CONFIG_11AZ)
#define FTM_ACTION_START 1
#define FTM_ACTION_STOP  2

#define PROTO_DOT11AZ_NTB 1
#define PROTO_DOT11AZ_TB  2
#define PROTO_DOT11MC     0

/* DOT11MC CFG */
/* Burst duration
 0 - 1: Reserved
 2: 250 micro seconds
 3: 500 micro seconds
 4: 1 ms
 5: 2 ms
 6: 4 ms
 7: 8 ms
 8: 16 ms
 9: 32 ms
 10: 64 ms
 11: 128 ms
 12-14 reserved*/
#define BURST_DURATION 11
/* Burst period in units of 100 milli seconds */
#define BURST_PERIOD 10
/* FTM frames per burst */
#define FTM_PER_BURST 5
/* Indicates minimum time between consecutive FTM (fine timing measurement) frames. It is specified in in units of 100 micro
 * seconds. */
#define MIN_DELTA 60
/* ASAP */
#define IS_ASAP 1
/* Bandwidth
 9  - HT20
 10 - VHT20
 11 - HT40
 12 - VHT40
 13 - VHT80 */
#define BW 13 /* RW610 only allows 20M bandwidth */
/*Indicates how many burst instances are requested for the FTM session */
#define BURST_EXP 3

/* LCI */
#define LCI_REQUEST                1
#define LCI_LATITIUDE              -33.8570095
#define LCI_LONGITUDE              151.2152005
#define LCI_LATITUDE_UNCERTAINITY  18
#define LCI_LONGITUDE_UNCERTAINITY 18
#define LCI_ALTITUDE               11.2
#define LCI_ALTITUDE_UNCERTAINITY  15
#define Z_INFO                     0

/* CIVIC */
#define CIVIC_REQUEST       1
#define CIVIC_LOCATION      1
#define CIVIC_LOCATION_TYPE 1
#define CIVIC_COUNTRY_CODE  0 /* US */
#define CIVIC_ADDRESS_TYPE  22
#define CIVIC_ADDRESS       "#123"

/* DOT11AZ CFG */
#define FORMAT_BW 2 /* RW610 only allows 20M bandwidth */
/*Maximum number of space-time streams to be used in DL/UL NDP frames in the session upto 80MHz*/
#define MAX_I2R_STS_UPTO80 0 /* RW610 only allows to send 1 N_STS*/
#define MAX_R2I_STS_UPTO80 0
/* Measurement freq in Hz to calculate measurement interval*/
#define AZ_MEASUREMENT_FREQ       4 /* in 0.1 Hz increments */
#define AZ_NUMBER_OF_MEASUREMENTS 6
#define I2R_LMR_FEEDBACK          0 /* allow RSTA to request I2R reporting */

#define FOR_RANGING 0

/** Structure of FTM_SESSION_CFG_NTB_RANGING / FTM_SESSION_CFG_TB_RANGING TLV data */
typedef struct _ranging_11az_cfg
{
    /** Indicates the channel BW for session*/
    /*0: HE20, 1: HE40, 2: HE80, 3: HE80+80, 4: HE160, 5:HE160_SRF*/
    uint8_t format_bw;
    /** indicates for bandwidths less than or equal to 80 MHz the maximum number of space-time streams to be used in
     * DL/UL NDP frames in the session*/
    uint8_t max_i2r_sts_upto80;
    /**indicates for bandwidths less than or equal to 80 MHz the maximum number of space-time streams to be used in
     * DL/UL NDP frames in the session*/
    uint8_t max_r2i_sts_upto80;
    /**Specify measurement freq in Hz to calculate measurement interval*/
    uint8_t az_measurement_freq;
    /**Indicates the number of measurements to be done for session*/
    uint8_t az_number_of_measurements;
    /** Initator lmr feedback */
    uint8_t i2r_lmr_feedback;
    /**Include location civic request (Expect location civic from responder)*/
    uint8_t civic_req;
    /**Include LCI request (Expect LCI info from responder)*/
    uint8_t lci_req;
} ranging_11az_cfg_t;

typedef struct _location_cfg_info
{
    /** known latitude uncertainty */
    uint8_t lat_unc;
    /** known longitude uncertainty */
    uint8_t long_unc;
    /** Known altitude uncertainty */
    uint8_t alt_unc;
    /**Include LCI request (expect LCI infomation from responder) */
    uint8_t lci_req;
    /** known longitude */
    double longitude;
    /** known latitude */
    double latitude;
    /** known altitude */
    double altitude;
} location_cfg_info_t;

typedef struct _location_civic_rep
{
    /** Civic location type */
    uint8_t civic_location_type;
    /**Civic address type*/
    uint8_t civic_address_type;
    /**Civic address length*/
    uint8_t civic_address_length;
    /**Include LCI request (Expect LCI info from responder)*/
    uint8_t civic_req;
    /**Country code*/
    t_u16 country_code;
} location_civic_rep_t;

/** Structure of FTM_SESSION_CFG TLV data */
typedef struct _ftm_11mc_nego_cfg
{
    /** Indicates how many burst instances are requested for the FTM session*/
    uint8_t burst_exponent;
    /** Indicates the duration of a burst instance*/
    uint8_t burst_duration;
    /**Minimum time between consecutive FTM frames*/
    uint8_t min_delta_FTM;
    /**ASAP/non-ASAP casel*/
    uint8_t is_ASAP;
    /**Number of FTMs per burst*/
    uint8_t per_burst_FTM;
    /**FTM channel spacing: HT20/HT40/VHT80/... */
    uint8_t channel_spacing;
    /**Indicates the interval between two consecutive burst instances*/
    t_u16 burst_period;
} ftm_11mc_nego_cfg_t;
#endif

/** Scan result */
struct wlan_scan_result
{
    /** The network SSID, represented as a NULL-terminated C string of 0 to 32
     *  characters. If the network has a hidden SSID, this can be the empty
     *  string.
     */
    char ssid[33];
    /** SSID length */
    unsigned int ssid_len;
    /** The network BSSID, represented as a 6-byte array. */
    char bssid[6];
    /** The network channel. */
    unsigned int channel;
    /** The Wi-Fi network type. */
    enum wlan_bss_type type;
    /** The Wi-Fi network mode. */
    enum wlan_bss_role role;

    /* network features */
    /** The network supports 802.11N.  This is set to 0 if the network does not
     *  support 802.11N or if the system does not have 802.11N support enabled. */
    unsigned dot11n : 1;
#if CONFIG_11AC
    /** The network supports 802.11AC.  This is set to 0 if the network does not
     *  support 802.11AC or if the system does not have 802.11AC support enabled. */
    unsigned dot11ac : 1;
#endif
#if CONFIG_11AX
    /** The network supports 802.11AX.  This is set to 0 if the network does not
     *  support 802.11AX or if the system does not have 802.11AX support enabled. */
    unsigned dot11ax : 1;
#endif

    /** The network supports WMM.  This is set to 0 if the network does not
     *  support WMM or if the system does not have WMM support enabled. */
    unsigned wmm : 1;
#if (CONFIG_WPA_SUPP_WPS) || (CONFIG_WPS2)
    /** The network supports WPS.  This is set to 0 if the network does not
     *  support WPS or if the system does not have WPS support enabled. */
    unsigned wps : 1;
    /** WPS Type PBC/PIN */
    unsigned int wps_session;
#endif
    /** The network uses WEP security. */
    unsigned wep : 1;
    /** The network uses WPA security. */
    unsigned wpa : 1;
    /** The network uses WPA2 security */
    unsigned wpa2 : 1;
    /** The network uses WPA2 SHA256 security */
    unsigned wpa2_sha256 : 1;
#if CONFIG_DRIVER_OWE
    /** The network uses OWE security */
    unsigned owe : 1;
#endif
    /** The network uses WPA3 SAE security */
    unsigned wpa3_sae : 1;
    /** The network uses WPA2 Enterprise security */
    unsigned wpa2_entp : 1;
    /** The network uses WPA2 Enterprise SHA256 security */
    unsigned wpa2_entp_sha256 : 1;
    /** The network uses WPA3 Enterprise SHA256 security */
    unsigned wpa3_1x_sha256 : 1;
    /** The network uses WPA3 Enterprise SHA384 security */
    unsigned wpa3_1x_sha384 : 1;
#if CONFIG_11R
    /** The network uses FT 802.1x security (For internal use only)*/
    unsigned ft_1x : 1;
    /** The network uses FT 892.1x SHA384 security */
    unsigned ft_1x_sha384 : 1;
    /** The network uses FT PSK security (For internal use only)*/
    unsigned ft_psk : 1;
    /** The network uses FT SAE security (For internal use only)*/
    unsigned ft_sae : 1;
#endif
    /** The signal strength of the beacon */
    unsigned char rssi;
    /** The network SSID, represented as a NULL-terminated C string of 0 to 32
     *  characters. If the network has a hidden SSID, this should be the empty
     *  string.
     */
    char trans_ssid[33];
    /** SSID length */
    unsigned int trans_ssid_len;
    /** The network BSSID, represented as a 6-byte array. */
    char trans_bssid[6];

    /** Beacon Period */
    uint16_t beacon_period;

    /** DTIM Period */
    uint8_t dtim_period;

    /** MFPC bit of AP*/
    uint8_t ap_mfpc;
    /** MFPR bit of AP*/
    uint8_t ap_mfpr;
    /** PWE bit of AP*/
    uint8_t ap_pwe;

#if CONFIG_11K
    /** Neigbort report support (For internal use only)*/
    bool neighbor_report_supported;
#endif
#if CONFIG_11V
    /** bss transition support (For internal use only)*/
    bool bss_transition_supported;
#endif
};

typedef enum
{
    Band_2_4_GHz = 0,
    Band_5_GHz   = 1,
    Band_4_GHz   = 2,

} ChanBand_e;

#define NUM_CHAN_BAND_ENUMS 3

typedef enum
{
    ChanWidth_20_MHz = 0,
    ChanWidth_10_MHz = 1,
    ChanWidth_40_MHz = 2,
    ChanWidth_80_MHz = 3,
} ChanWidth_e;

typedef enum
{
    SECONDARY_CHAN_NONE  = 0,
    SECONDARY_CHAN_ABOVE = 1,
    SECONDARY_CHAN_BELOW = 3,
    // reserved 2, 4~255
} Chan2Offset_e;

typedef enum
{
    MANUAL_MODE = 0,
    ACS_MODE    = 1,
} ScanMode_e;

typedef  struct
{
    ChanBand_e chanBand : 2;
    ChanWidth_e chanWidth : 2;
    Chan2Offset_e chan2Offset : 2;
    ScanMode_e scanMode : 2;
}  BandConfig_t;

typedef  struct
{
    BandConfig_t bandConfig;
    uint8_t chanNum;

}  ChanBandInfo_t;


#if CONFIG_5GHz_SUPPORT
#define DFS_REC_HDR_LEN (8)
#define DFS_REC_HDR_NUM (10)
#define BIN_COUNTER_LEN (7)

typedef  struct _Event_Radar_Detected_Info
{
    u32 detect_count;
    uint8_t reg_domain;    /*1=fcc, 2=etsi, 3=mic*/
    uint8_t main_det_type; /*0=none, 1=pw(chirp), 2=pri(radar)*/
    uint16_t pw_chirp_type;
    uint8_t pw_chirp_idx;
    uint8_t pw_value;
    uint8_t pri_radar_type;
    uint8_t pri_binCnt;
    uint8_t binCounter[BIN_COUNTER_LEN];
    uint8_t numDfsRecords;
    uint8_t dfsRecordHdrs[DFS_REC_HDR_NUM][DFS_REC_HDR_LEN];
    u32 reallyPassed;
}  Event_Radar_Detected_Info;
#endif

/** Network security types */
enum wlan_security_type
{
    /** The network does not use security. */
    WLAN_SECURITY_NONE,
    /** The network uses WEP security with open key. */
    WLAN_SECURITY_WEP_OPEN,
    /** The network uses WEP security with shared key. */
    WLAN_SECURITY_WEP_SHARED,
    /** The network uses WPA security with PSK. */
    WLAN_SECURITY_WPA,
    /** The network uses WPA2 security with PSK. */
    WLAN_SECURITY_WPA2,
    /** The network uses WPA/WPA2 mixed security with PSK */
    WLAN_SECURITY_WPA_WPA2_MIXED,
#if CONFIG_11R
    /** The network uses WPA2 security with PSK FT. */
    WLAN_SECURITY_WPA2_FT,
#endif
    /** The network uses WPA3 security with SAE. */
    WLAN_SECURITY_WPA3_SAE,
#if CONFIG_WPA_SUPP
#if CONFIG_11R
    /** The network uses WPA3 security with SAE FT. */
    WLAN_SECURITY_WPA3_FT_SAE,
#endif
#endif
    /** The network uses WPA3 security with SAE EXT KEY. */
    WLAN_SECURITY_WPA3_SAE_EXT_KEY,
    /** The network uses WPA2/WPA3 SAE mixed security with PSK. */
    WLAN_SECURITY_WPA2_WPA3_SAE_MIXED,
#if CONFIG_DRIVER_OWE
    /** The network uses OWE only security without Transition mode support. */
    WLAN_SECURITY_OWE_ONLY,
#endif
#if (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE) || (CONFIG_WPA2_ENTP)
    /** The network uses WPA2 Enterprise EAP-TLS security
     * The identity field in \ref wlan_network structure is used */
    WLAN_SECURITY_EAP_TLS,
#endif
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
    /** The network uses WPA2 Enterprise EAP-TLS SHA256 security.
     * The identity field in \ref wlan_network structure is used */
    WLAN_SECURITY_EAP_TLS_SHA256,
#if CONFIG_11R
    /** The network uses WPA2 Enterprise EAP-TLS FT security.
     * The identity field in \ref wlan_network structure is used */
    WLAN_SECURITY_EAP_TLS_FT,
    /** The network uses WPA2 Enterprise EAP-TLS FT SHA384 security
     * The identity field in \ref wlan_network structure is used */
    WLAN_SECURITY_EAP_TLS_FT_SHA384,
#endif
#endif
#if CONFIG_EAP_TTLS
    /** The network uses WPA2 Enterprise EAP-TTLS security.
     * The identity field in \ref wlan_network structure is used */
    WLAN_SECURITY_EAP_TTLS,
#endif
#if CONFIG_EAP_MSCHAPV2
    /** The network uses WPA2 Enterprise EAP-TTLS-MSCHAPV2 security.
     * The anonymous identity, identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_TTLS_MSCHAPV2,
#endif
#endif
#if (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE) || (CONFIG_PEAP_MSCHAPV2) || (CONFIG_WPA2_ENTP)
    /** The network uses WPA2 Enterprise EAP-PEAP-MSCHAPV2 security.
     * The anonymous identity, identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_PEAP_MSCHAPV2,
#endif
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_TLS
    /** The network uses WPA2 Enterprise EAP-PEAP-TLS security.
     * The anonymous identity, identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_PEAP_TLS,
#endif
#if CONFIG_EAP_GTC
    /** The network uses WPA2 Enterprise EAP-PEAP-GTC security.
     * The anonymous identity, identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_PEAP_GTC,
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
    /** The network uses WPA2 Enterprise EAP-FAST-MSCHAPV2 security.
     * The anonymous identity, identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_FAST_MSCHAPV2,
#endif
#if CONFIG_EAP_GTC
    /** The network uses WPA2 Enterprise EAP-FAST-GTC security
     * The anonymous identity, identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_FAST_GTC,
#endif
#endif
#if CONFIG_EAP_SIM
    /** The network uses WPA2 Enterprise EAP-SIM security
     * The identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_SIM,
#endif
#if CONFIG_EAP_AKA
    /** The network uses WPA2 Enterprise EAP-AKA security
     * The identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_AKA,
#endif
#if CONFIG_EAP_AKA_PRIME
    /** The network uses WPA2 Enterprise EAP-AKA-PRIME security
     * The identity and password fields in
     * \ref wlan_network structure are used */
    WLAN_SECURITY_EAP_AKA_PRIME,
#endif
#endif
#if CONFIG_WPA_SUPP_DPP
    /** The network uses DPP security with NAK(Net Access Key) */
    WLAN_SECURITY_DPP,
#endif
    /** The network can use any security method. This is often used when
     * the user only knows the name and passphrase but not the security
     * type.  */
    WLAN_SECURITY_WILDCARD,
};

#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
/** EAP TLS Cipher types*/
enum eap_tls_cipher_type
{
    EAP_TLS_NONE,
    /** EAP TLS with ECDH & ECDSA with p384 */
    EAP_TLS_ECC_P384,
    /** EAP TLS with ECDH & RSA with > 3K */
    EAP_TLS_RSA_3K,
};
#endif
#endif

/** Wi-Fi cipher structure */
struct wlan_cipher
{
    /** 1 bit value can be set for none */
    uint16_t none : 1;
    /** 1 bit value can be set for wep40 */
    uint16_t wep40 : 1;
    /** 1 bit value can be set for wep104 */
    uint16_t wep104 : 1;
    /** 1 bit value can be set for tkip */
    uint16_t tkip : 1;
    /** 1 bit value can be set for ccmp */
    uint16_t ccmp : 1;
    /**  1 bit value can be set for aes 128 cmac */
    uint16_t aes_128_cmac : 1;
    /** 1 bit value can be set for gcmp */
    uint16_t gcmp : 1;
    /** 1 bit value can be set for sms4 */
    uint16_t sms4 : 1;
    /** 1 bit value can be set for gcmp 256 */
    uint16_t gcmp_256 : 1;
    /** 1 bit value can be set for ccmp 256 */
    uint16_t ccmp_256 : 1;
    /** 1 bit is reserved */
    uint16_t rsvd : 1;
    /** 1 bit value can be set for bip gmac 128 */
    uint16_t bip_gmac_128 : 1;
    /** 1 bit value can be set for bip gmac 256 */
    uint16_t bip_gmac_256 : 1;
    /** 1 bit value can be set for bip cmac 256 */
    uint16_t bip_cmac_256 : 1;
    /** 1 bit value can be set for gtk not used */
    uint16_t gtk_not_used : 1;
    /** 4 bits are reserved */
    uint16_t rsvd2 : 2;
};

static inline int is_valid_security(int security)
{
    /*Currently only these modes are supported */
    if ((security == WLAN_SECURITY_NONE) || (security == WLAN_SECURITY_WEP_OPEN) || (security == WLAN_SECURITY_WPA) ||
            (security == WLAN_SECURITY_WPA2) ||
#if CONFIG_11R
            (security == WLAN_SECURITY_WPA2_FT) ||
#endif
            (security == WLAN_SECURITY_WPA_WPA2_MIXED) ||
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
            (security == WLAN_SECURITY_EAP_TLS) || (security == WLAN_SECURITY_EAP_TLS_SHA256) ||
#if CONFIG_11R
            (security == WLAN_SECURITY_EAP_TLS_FT) || (security == WLAN_SECURITY_EAP_TLS_FT_SHA384) ||
#endif
#endif
#if CONFIG_EAP_TTLS
            (security == WLAN_SECURITY_EAP_TTLS) ||
#if CONFIG_EAP_MSCHAPV2
            (security == WLAN_SECURITY_EAP_TTLS_MSCHAPV2) ||
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
            (security == WLAN_SECURITY_EAP_PEAP_MSCHAPV2) ||
#endif
#if CONFIG_EAP_TLS
            (security == WLAN_SECURITY_EAP_PEAP_TLS) ||
#endif
#if CONFIG_EAP_GTC
            (security == WLAN_SECURITY_EAP_PEAP_GTC) ||
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
            (security == WLAN_SECURITY_EAP_FAST_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
            (security == WLAN_SECURITY_EAP_FAST_GTC) ||
#endif
#endif
#if CONFIG_EAP_SIM
            (security == WLAN_SECURITY_EAP_SIM) ||
#endif
#if CONFIG_EAP_AKA
            (security == WLAN_SECURITY_EAP_AKA) ||
#endif
#if CONFIG_EAP_AKA_PRIME
            (security == WLAN_SECURITY_EAP_AKA_PRIME) ||
#endif
#else
#if CONFIG_WPA2_ENTP
            (security == WLAN_SECURITY_EAP_TLS) ||
#endif
#if CONFIG_PEAP_MSCHAPV2
            (security == WLAN_SECURITY_EAP_PEAP_MSCHAPV2) ||
#endif
#endif /* CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE */
#if CONFIG_DRIVER_OWE
            (security == WLAN_SECURITY_OWE_ONLY) ||
#endif
            (security == WLAN_SECURITY_WPA3_SAE) || (security == WLAN_SECURITY_WPA2_WPA3_SAE_MIXED) ||
#if CONFIG_WPA_SUPP
#if CONFIG_11R
            (security == WLAN_SECURITY_WPA3_FT_SAE) ||
#endif
#endif
            (security == WLAN_SECURITY_WPA3_SAE_EXT_KEY) || (security == WLAN_SECURITY_WILDCARD))
    {
        return 1;
    }
    return 0;
}

#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
static inline int is_ep_valid_security(int security)
{
    /*Currently only these modes are supported */
    if (
#if CONFIG_EAP_TLS
        (security == WLAN_SECURITY_EAP_TLS) || (security == WLAN_SECURITY_EAP_TLS_SHA256) ||
#if CONFIG_11R
        (security == WLAN_SECURITY_EAP_TLS_FT) || (security == WLAN_SECURITY_EAP_TLS_FT_SHA384) ||
#endif
#endif
#if CONFIG_EAP_TTLS
        (security == WLAN_SECURITY_EAP_TTLS) ||
#if CONFIG_EAP_MSCHAPV2
        (security == WLAN_SECURITY_EAP_TTLS_MSCHAPV2) ||
#endif
#endif
#if CONFIG_EAP_PEAP
#if CONFIG_EAP_MSCHAPV2
        (security == WLAN_SECURITY_EAP_PEAP_MSCHAPV2) ||
#endif
#if CONFIG_EAP_TLS
        (security == WLAN_SECURITY_EAP_PEAP_TLS) ||
#endif
#if CONFIG_EAP_GTC
        (security == WLAN_SECURITY_EAP_PEAP_GTC) ||
#endif
#endif
#if CONFIG_EAP_FAST
#if CONFIG_EAP_MSCHAPV2
        (security == WLAN_SECURITY_EAP_FAST_MSCHAPV2) ||
#endif
#if CONFIG_EAP_GTC
        (security == WLAN_SECURITY_EAP_FAST_GTC) ||
#endif
#endif
#if CONFIG_EAP_SIM
        (security == WLAN_SECURITY_EAP_SIM) ||
#endif
#if CONFIG_EAP_AKA
        (security == WLAN_SECURITY_EAP_AKA) ||
#endif
#if CONFIG_EAP_AKA_PRIME
        (security == WLAN_SECURITY_EAP_AKA_PRIME) ||
#endif
        false)
    {
        return 1;
    }
    return 0;
}
#endif

/** Network security configuration */
struct wlan_network_security
{
    /** Type of network security to use specified by enum
     * wlan_security_type. */
    enum wlan_security_type type;// 网络安全类型，例如无安全、WEP、WPA、WPA2、WPA3等
    /** Key management type */
    int key_mgmt;// 密钥管理类型，例如 PSK、SAE、EAP 等。
    /** Type of network security Group Cipher suite used internally*/
    struct wlan_cipher mcstCipher;
    /** Type of network security Pairwise Cipher suite used internally*/
    struct wlan_cipher ucstCipher;
#if CONFIG_WPA_SUPP
    /** Proactive key caching */
    unsigned pkc : 1;
    /** Type of network security Group Cipher suite */
    int group_cipher;
    /** Type of network security Pairwise Cipher suite */
    int pairwise_cipher;
    /** Type of network security Pairwise Cipher suite */
    int group_mgmt_cipher;
#endif
    /** Is PMF (protected management frame) required */
    bool is_pmf_required;
    /** Pre-shared key (network password).  For WEP networks this is a hex byte
     * sequence of length psk_len, for WPA and WPA2 networks this is an ASCII
     * pass-phrase of length psk_len.  This field is ignored for networks with no
     * security. */
    char psk[WLAN_PSK_MAX_LENGTH];// 预共享密钥（网络密码），用于 WPA/WPA2 网络。
    /** Length of the WEP key or WPA/WPA2 pass phrase, \ref WLAN_PSK_MIN_LENGTH to \ref
     * WLAN_PSK_MAX_LENGTH.  Ignored for networks with no security. */
    uint8_t psk_len;// 预共享密钥的长度。
    /** WPA3 SAE password, for WPA3 SAE networks this is an ASCII
     * password of length password_len.  This field is ignored for networks with no
     * security. */
    char password[WLAN_PASSWORD_MAX_LENGTH + 1];// WPA3 SAE 密码。
    /** Length of the WPA3 SAE Password, \ref WLAN_PASSWORD_MIN_LENGTH to \ref
     * WLAN_PASSWORD_MAX_LENGTH.  Ignored for networks with no security. */
    size_t password_len;// WPA3 SAE 密码的长度。
    /** SAE Groups */
    char *sae_groups;
    /** PWE derivation */
    uint8_t pwe_derivation;
    /** transition disable */
    uint8_t transition_disable;
#if CONFIG_DRIVER_OWE
    /** OWE Groups */
    char *owe_groups;
#endif
    /** PMK (pairwise master key). When pmk_valid is set, this is the PMK calculated
     * from the PSK for WPA/PSK networks. If pmk_valid is not set, this field
     * is not valid. When adding networks with \ref wlan_add_network, users
     * can initialize PMK and set pmk_valid in lieu of setting the psk. After
     * successfully connecting to a WPA/PSK network, users can call \ref
     * wlan_get_current_network to inspect pmk_valid and pmk. Thus, the pmk
     * value can be populated in subsequent calls to \ref wlan_add_network.
     * This saves the CPU time required to otherwise calculate the PMK.
     */
    char pmk[WLAN_PMK_LENGTH];// 配对主密钥 (PMK)。

    /** Flag reporting whether PMK is valid or not. */
    bool pmk_valid;
    /** Management frame protection capable (MFPC) */
    int8_t mfpc;
    /** Management frame protection required (MFPR) */
    int8_t mfpr;
#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
    /** WPA3 Enterprise mode */
    unsigned wpa3_sb : 1;
    /** WPA3 Enterprise Suite B 192 mode */
    unsigned wpa3_sb_192 : 1;
    /** PEAP version */
    unsigned eap_ver : 1;
#if CONFIG_EAP_PEAP
    /** PEAP label */
    unsigned peap_label : 1;
    /** crypto_binding option can be used to control \ref WLAN_SECURITY_EAP_PEAP_MSCHAPV2, \ref
     * WLAN_SECURITY_EAP_PEAP_TLS and \ref WLAN_SECURITY_EAP_PEAP_GTC version 0 cryptobinding behavior: 0 = do not use
     * cryptobinding (default) 1 = use cryptobinding if server supports it 2 = require cryptobinding */
    uint8_t eap_crypto_binding;
#endif
#if (CONFIG_EAP_SIM) || (CONFIG_EAP_AKA) || (CONFIG_EAP_AKA_PRIME)
    /** eap_result_ind=1 can be used to enable \ref WLAN_SECURITY_EAP_SIM, \ref WLAN_SECURITY_EAP_AKA and \ref
     * WLAN_SECURITY_EAP_AKA_PRIME to use protected result indication.*/
    unsigned eap_result_ind : 1;
#endif
#if CONFIG_EAP_TLS
    /** Cipher for EAP TLS */
    unsigned char tls_cipher;
#endif
    /** Identity string for EAP */
    char identity[IDENTITY_MAX_LENGTH];
    /** Anonymous identity string for EAP */
    char anonymous_identity[IDENTITY_MAX_LENGTH];
    /** Password string for EAP. This field can include
     * either the plaintext password (using ASCII or
     * hex string) */
    char eap_password[PASSWORD_MAX_LENGTH];
    /** CA cert blob in PEM/DER format */
    unsigned char *ca_cert_data;
    /** CA cert blob len */
    size_t ca_cert_len;
    /** Client cert blob in PEM/DER format */
    unsigned char *client_cert_data;
    /** Client cert blob len */
    size_t client_cert_len;
    /** Client key blob */
    unsigned char *client_key_data;
    /** Client key blob len */
    size_t client_key_len;
    /** Client key password */
    char client_key_passwd[PASSWORD_MAX_LENGTH];
    /** CA cert HASH */
    char ca_cert_hash[HASH_MAX_LENGTH];
    /** Domain */
    char domain_match[DOMAIN_MATCH_MAX_LENGTH];
    /** Domain Suffix */
    char domain_suffix_match[DOMAIN_MATCH_MAX_LENGTH]; /*suffix max length same as full domain name length*/
    /** CA cert2 blob in PEM/DER format */
    unsigned char *ca_cert2_data;
    /** CA cert2 blob len */
    size_t ca_cert2_len;
    /** Client cert2 blob in PEM/DER format */
    unsigned char *client_cert2_data;
    /** Client cert2 blob len */
    size_t client_cert2_len;
    /** Client key2 blob */
    unsigned char *client_key2_data;
    /** Client key2 blob len */
    size_t client_key2_len;
    /** Client key2 password */
    char client_key2_passwd[PASSWORD_MAX_LENGTH];
#if CONFIG_HOSTAPD
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
    /** DH params blob */
    unsigned char *dh_data;
    /** DH params blob len */
    size_t dh_len;
    /** Server cert blob in PEM/DER format */
    unsigned char *server_cert_data;
    /** Server cert blob len */
    size_t server_cert_len;
    /** Server key blob */
    unsigned char *server_key_data;
    /** Server key blob len */
    size_t server_key_len;
    /** Server key password */
    char server_key_passwd[PASSWORD_MAX_LENGTH];
    /** Number of EAP users */
    size_t nusers;
    /** User Identities */
    char identities[MAX_USERS][IDENTITY_MAX_LENGTH];
    /** User Passwords */
    char passwords[MAX_USERS][PASSWORD_MAX_LENGTH];
#if CONFIG_EAP_FAST
    /** Encryption key for EAP-FAST PAC-Opaque values */
    char pac_opaque_encr_key[PAC_OPAQUE_ENCR_KEY_MAX_LENGTH];
    /** EAP-FAST authority identity (A-ID) */
    char a_id[A_ID_MAX_LENGTH];
    /** EAP-FAST provisioning modes:
     * 0 = provisioning disabled
     * 1 = only anonymous provisioning allowed
     * 2 = only authenticated provisioning allowed
     * 3 = both provisioning modes allowed (default)
     */
    uint8_t fast_prov;
#endif
#endif
#endif
#elif (CONFIG_WPA2_ENTP)
    /** TLS client cert configuration */
    wm_mbedtls_cert_t tls_cert;
    /** mbedtls_ssl_config handle */
    mbedtls_ssl_config *wlan_ctx;
    /** mbedtls_ssl_context handle */
    mbedtls_ssl_context *wlan_ssl;
#endif
#if CONFIG_WPA_SUPP_DPP
    unsigned char *dpp_connector;
    unsigned char *dpp_c_sign_key;
    unsigned char *dpp_net_access_key;
#endif
};

/** Configuration for Wi-Fi scan */
#ifndef MAX_CHANNEL_LIST
#define MAX_CHANNEL_LIST 26
#endif
/** This structure is used to configure Wi-Fi scan parameters */
struct wifi_scan_params_t
{
    /** BSSID (basic servivce set ID) */
    uint8_t *bssid;
    /** SSID (service set ID) */
    char *ssid;
    /** Channel list */
    int channel[MAX_CHANNEL_LIST];
    /** BSS (basic service set) type.
    1: Infrastructure BSS,
    2: Indenpent BSS.
    */
    IEEEtypes_Bss_t bss_type;
    /** Time for scan duration */
    int scan_duration;
    /** split scan delay */
    int split_scan_delay;
};

#if CONFIG_WIFI_GET_LOG
    /** Wi-Fi firmware stat from \ref wifi_pkt_stats_t
    */
    typedef wifi_pkt_stats_t wlan_pkt_stats_t;
#endif

// /** Configuration for Wi-Fi scan channel list from
//  * \ref wifi_scan_channel_list_t
//  */
// typedef wifi_scan_channel_list_t wlan_scan_channel_list_t;
// /** Configuration for Wi-Fi scan parameters v2 from
//  * \ref wifi_scan_params_v2_t
//  */
// typedef wifi_scan_params_v2_t wlan_scan_params_v2_t;


// /** Configuration for Wi-Fi calibration data from
//  * \ref wifi_cal_data_t
//  */
// typedef wifi_cal_data_t wlan_cal_data_t;

// #if CONFIG_AUTO_RECONNECT
//     /** Configuration for auto reconnect configuration from
//     * \ref wifi_auto_reconnect_config_t
//     */
//     typedef wifi_auto_reconnect_config_t wlan_auto_reconnect_config_t;
// #endif

// /** Configuration for memory efficient filters in Wi-Fi firmware from
//  * \ref wifi_flt_cfg_t
//  */
// typedef wifi_flt_cfg_t wlan_flt_cfg_t;

// /** Configuration for wowlan pattern parameters from
//  * \ref wifi_wowlan_ptn_cfg_t
//  */
// typedef wifi_wowlan_ptn_cfg_t wlan_wowlan_ptn_cfg_t;
// /** Configuration for TCP keep alive parameters from
//  * \ref wifi_tcp_keep_alive_t
//  */
// typedef wifi_tcp_keep_alive_t wlan_tcp_keep_alive_t;

// #if CONFIG_CLOUD_KEEP_ALIVE
//     /** Configuration for cloud keep alive parameters from
//     * \ref wifi_cloud_keep_alive_t
//     */
//     typedef wifi_cloud_keep_alive_t wlan_cloud_keep_alive_t;
// #endif

// /** Configuration for TX rate and get data rate from
//  * \ref wifi_ds_rate
//  */
// typedef wifi_ds_rate wlan_ds_rate;
// /** Configuration for ED MAC Control parameters from
//  * \ref wifi_ed_mac_ctrl_t
//  */
// typedef wifi_ed_mac_ctrl_t wlan_ed_mac_ctrl_t;
// /** Configuration for band from
//  * \ref wifi_bandcfg_t
//  */
// typedef wifi_bandcfg_t wlan_bandcfg_t;
// /** Configuration for CW mode parameters from
//  * \ref wifi_cw_mode_ctrl_t
//  */
// typedef wifi_cw_mode_ctrl_t wlan_cw_mode_ctrl_t;
// /** Configuration for channel list from
//  * \ref wifi_chanlist_t
//  */
// typedef wifi_chanlist_t wlan_chanlist_t;
// /** Configuration for TX power Limit from
//  * \ref wifi_txpwrlimit_t
//  */
// typedef wifi_txpwrlimit_t wlan_txpwrlimit_t;
// #ifdef SD8801
//     /** Statistic of External Coex from
//     * \ref wifi_ext_coex_config_t
//     */
//     typedef wifi_ext_coex_stats_t wlan_ext_coex_stats_t;
//     /** Configuration for external Coex from
//     * \ref wifi_ext_coex_config_t
//     */
//     typedef wifi_ext_coex_config_t wlan_ext_coex_config_t;
// #endif

#if CONFIG_11AX
    /** Configuration for RU TX power limit from
    * \ref wifi_rutxpwrlimit_t
    */
    typedef wifi_rutxpwrlimit_t wlan_rutxpwrlimit_t;
    /** Configuration for 802.11ax capabilities
    * \ref wifi_11ax_config_t
    */
    typedef wifi_11ax_config_t wlan_11ax_config_t;
    #if CONFIG_11AX_TWT
        /** Configuration for TWT setup
        * \ref wifi_twt_setup_config_t
        */
        typedef wifi_twt_setup_config_t wlan_twt_setup_config_t;
        /** Configuration for TWT teardown
        * \ref wifi_twt_teardown_config_t
        */
        typedef wifi_twt_teardown_config_t wlan_twt_teardown_config_t;
        /** Configuration for Broadcast TWT setup
        * \ref wifi_btwt_config_t
        */
        typedef wifi_btwt_config_t wlan_btwt_config_t;
        /** Configuration for TWT report
        * \ref wifi_twt_report_t
        */
        typedef wifi_twt_report_t wlan_twt_report_t;
    #endif /* CONFIG_11AX_TWT */
    #if CONFIG_MMSF
        #define WLAN_AMPDU_DENSITY 0x30
        #define WLAN_AMPDU_MMSF    0x6
    #endif
#endif
#if CONFIG_WIFI_CLOCKSYNC
    /** Configuration for clock sync GPIO TSF latch
    * \ref wifi_clock_sync_gpio_tsf_t
    */
    typedef wifi_clock_sync_gpio_tsf_t wlan_clock_sync_gpio_tsf_t;
    /** Configuration for TSF info
    * \ref wifi_tsf_info_t
    */
    typedef wifi_tsf_info_t wlan_tsf_info_t;
#endif

#if CONFIG_MULTI_CHAN
    /** Configuration for multi-channel switch
    * \ref wifi_drcs_cfg_t
    */
    typedef wifi_drcs_cfg_t wlan_drcs_cfg_t;
#endif

//typedef wifi_mgmt_frame_t wlan_mgmt_frame_t;


#if CONFIG_CSI
    /** Configuration for CSI config params from
    * \ref wifi_csi_config_params_t
    */
    typedef wifi_csi_config_params_t wlan_csi_config_params_t;
#endif

#if CONFIG_NET_MONITOR
    /** Configuration for net monitor from
    * \ref wifi_net_monitor_t
    */
    typedef wifi_net_monitor_t wlan_net_monitor_t;
#endif

#if (CONFIG_WIFI_IND_RESET) && (CONFIG_WIFI_IND_DNLD)
    /** Configuration for GPIO independent reset
    * \ref wifi_indrst_cfg_t
    */
    typedef wifi_indrst_cfg_t wlan_indrst_cfg_t;
#endif

#if CONFIG_11AX
    /** Configuration for TX rate setting from
    * \ref txrate_setting
    */
    typedef txrate_setting wlan_txrate_setting;
#endif

/** Configuration for RSSI information
 * \ref wifi_rssi_info_t
 */
//typedef wifi_rssi_info_t wlan_rssi_info_t;

#if CONFIG_EXTERNAL_COEX_PTA
#define MIN_SAMP_TIMING              20
#define MAX_SAMP_TIMING              200
#define COEX_PTA_FEATURE_ENABLE      1
#define COEX_PTA_FEATURE_DISABLE     0
#define POL_GRANT_PIN_HIGH           0
#define POL_GRANT_PIN_LOW            1
#define STATE_INPUT_DISABLE          0
#define STATE_PTA_PIN                1
#define STATE_PRIORITY_PIN           2
#define SAMPLE_TIMING_VALUE          100
#define EXT_COEX_PTA_INTERFACE       5
#define EXT_COEX_WCI2_INTERFACE      6
#define EXT_COEX_WCI2_GPIO_INTERFACE 7

typedef struct _external_coex_pta_cfg
{
    /** Enable: 0x01, Disable: 0x00 */
    uint8_t enabled;
    /** Enable extended Wi-Fi and Bluetooth LE arbitration: 0x01, disable : 0x00 */
    uint8_t ext_WifiBtArb;
    /** Active high: 0x00, Active low: 0x01 */
    uint8_t polGrantPin;
    /**  Enable PriPtaInt: 0x01, Disable PriPtaInt: 0x00 */
    uint8_t enable_PriPtaInt;
    /** State input disable: 0x00, State info is from state pin: 0x01, State info is sampled on priority pin: 0x02 */
    uint8_t enable_StatusFromPta;
    /** Timing to sample Priority bit */
    t_u16 setPriSampTiming;
    /** Timing to sample TX/RX info */
    t_u16 setStateInfoSampTiming;
    /** Enable external traffic TX/RX Priority: 0x01, Disable external traffic TX/RX Priority: 0x00 */
    uint8_t extRadioTrafficPrio;
    /** Enable wci-2 interface: 0x01, Disable wci-2 interface: 0x00 */
    uint8_t extCoexHwIntWci2;
} ext_coex_pta_cfg;
#endif

/**
* Check whether the scan duration is valid.
*
* \param[in] scan duration time
*
* \return 0 if the time is valid, else return -1.
*/
int verify_scan_duration_value(int scan_duration);

/**
* Check whether the scan channel is valid.
*
* \param[in] channel
*
* \return 0 if the channel is valid, else return -1.
*/
int verify_scan_channel_value(int channel);

/**
* Check whether the scan delay time is valid.
*
* \param[in] delay, the scan delay time.
*
* \return 0 if the time is valid, else return -1.
*/
int verify_split_scan_delay(int delay);

/**
* Set scan parameters.
*
* \param[in] wifi_scan_params Wi-Fi scan parameter structure pointer.
*
* \return WM_SUCCESS.
*/
int set_scan_params(struct wifi_scan_params_t *wifi_scan_params);

/**
* Get scan parameters.
*
* \param[out] wifi_scan_params Wi-Fi scan parameter structure pointer.
*
* \return WM_SUCCESS.
*/
int get_scan_params(struct wifi_scan_params_t *wifi_scan_params);

/**
* Get current RSSI value.
*
* \param[out] rssi, RSSI pointer.
*
* \return WM_SUCCESS.
*/
int wlan_get_current_rssi(short *rssi);

/**
* Get current noise floor.
*
* \return noise floor value .
*/
int wlan_get_current_nf(void);

/** Address types to be used by the element wlan_ip_config.addr_type below
 */
enum address_types
{
    /** static IP address */
    ADDR_TYPE_STATIC = 0,
    /** Dynamic  IP address*/
    ADDR_TYPE_DHCP = 1,
    /** Link level address */
    ADDR_TYPE_LLA = 2,
};

/** This data structure represents an IPv4 address */
struct ipv4_config
{
    /** Set to \ref ADDR_TYPE_DHCP to use DHCP to obtain the IP address or
     *  \ref ADDR_TYPE_STATIC to use a static IP. In case of static IP
     *  address ip, gw, netmask and dns members should be specified. When
     *  using DHCP, the ip, gw, netmask and dns are overwritten by the
     *  values obtained from the DHCP server. They should be zeroed out if
     *  not used. */
    enum address_types addr_type;
    /** The system's IP address in network order. */
    unsigned address;
    /** The system's default gateway in network order. */
    unsigned gw;
    /** The system's subnet mask in network order. */
    unsigned netmask;
    /** The system's primary dns server in network order. */
    unsigned dns1;
    /** The system's secondary dns server in network order. */
    unsigned dns2;
};

#if CONFIG_IPV6
/** This data structure represents an IPv6 address */
struct ipv6_config
{
    /** The system's IPv6 address in network order. */
    unsigned address[4];
    /** The address type: linklocal, site-local or global. */
    unsigned char addr_type;
    /** The state of IPv6 address (Tentative, Preferred, etc). */
    unsigned char addr_state;
};
#endif

/** Network IP configuration.
 *
 *  This data structure represents the network IP configuration
 *  for IPv4 as well as IPv6 addresses
 */
struct wlan_ip_config
{
#if CONFIG_IPV6
    /** The network IPv6 address configuration that should be
     * associated with this interface. */
    struct ipv6_config ipv6[CONFIG_MAX_IPV6_ADDRESSES];
    /** The network IPv6 valid addresses count */
    size_t ipv6_count;
#endif
    /** The network IPv4 address configuration that should be
     * associated with this interface. */
    struct ipv4_config ipv4;
};

/** Wi-Fi network profile
 *
 *  This data structure represents a Wi-Fi network profile. It consists of an
 *  arbitrary name, Wi-Fi configuration, and IP address configuration.
 *
 *  Every network profile is associated with one of the two interfaces. The
 *  network profile can be used for the station interface (i.e. to connect to an
 *  Access Point) by setting the role field to \ref
 *  WLAN_BSS_ROLE_STA. The network profile can be used for the UAP
 *  interface (i.e. to start a network of our own.) by setting the mode field to
 *  \ref WLAN_BSS_ROLE_UAP.
 *
 *  If the mode field is \ref WLAN_BSS_ROLE_STA, either of the SSID or
 *  BSSID fields are used to identify the network, while the other members like
 *  channel and security settings characterize the network.
 *
 *  If the mode field is \ref WLAN_BSS_ROLE_UAP, the SSID, channel and security
 *  fields are used to define the network to be started.
 *
 *  In both the above cases, the address field is used to determine the type of
 *  address assignment to be used for this interface.
 */
struct wlan_network
{
#if CONFIG_WPA_SUPP
    /** Identifier for network profile */
    int id;
    /** WPS netwrok flag. */
    int wps_network;
#endif
    /** The name of this network profile. Each network profile that is
     *  added to the Wi-Fi connection manager should have a unique name. */
    char name[WLAN_NETWORK_NAME_MAX_LENGTH + 1];
    /** The network SSID, represented as a C string of up to 32 characters
     *  in length.
     *  If this profile is used in the UAP mode, this field is
     *  used as the SSID of the network.
     *  If this profile is used in the station mode, this field is
     *  used to identify the network. Set the first byte of the SSID to NULL
     *  (a 0-length string) to use only the BSSID to find the network.
     */
    char ssid[IEEEtypes_SSID_SIZE + 1];
    /** The network BSSID, represented as a 6-byte array.
     *  If this profile is used in the UAP mode, this field is
     *  ignored.
     *  If this profile is used in the station mode, this field is
     *  used to identify the network. Set all 6 bytes to 0 to use any BSSID,
     *  in which case only the SSID is used to find the network.
     */
    char bssid[IEEEtypes_ADDRESS_SIZE];
    /** The channel for this network.
     *
     *  If this profile is used in UAP mode, this field
     *  specifies the channel to start the UAP interface on. Set this
     *  to 0 for auto channel selection.
     *
     *  If this profile is used in the station mode, this constrains the
     *  channel on which the network to connect should be present. Set this
     *  to 0 to allow the network to be found on any channel. */
    unsigned int channel;
    /** The secondary channel offset **/
    uint8_t sec_channel_offset;
    /** The ACS (auto channel selection) band if set channel to 0. */
    uint16_t acs_band;
    /** RSSI (received signal strength indicator) value. */
    int rssi;
#if CONFIG_SCAN_WITH_RSSIFILTER
    /** RSSI threshold */
    short rssi_threshold;
#endif
#if CONFIG_WPA_SUPP
    /** HT capabilities */
    unsigned short ht_capab;
#if CONFIG_11AC
    /** VHT capabilities */
    unsigned int vht_capab;
    /** VHT bandwidth */
    unsigned char vht_oper_chwidth;
#endif
#if CONFIG_11AX
    /** HE bandwidth */
    unsigned char he_oper_chwidth;
#endif
#endif
    /** BSS type */
    enum wlan_bss_type type;
    /** The network Wi-Fi mode enum wlan_bss_role. Set this
     *  to specify what type of Wi-Fi network mode to use.
     *  This can either be \ref WLAN_BSS_ROLE_STA for use in
     *  the station mode, or it can be \ref WLAN_BSS_ROLE_UAP
     *  for use in the UAP mode. */
    /*
    网络的角色。
        可能的值包括：
        WLAN_BSS_ROLE_STA: 作为站点（STA）。
        WLAN_BSS_ROLE_UAP: 作为接入点（UAP）*/
    enum wlan_bss_role role;
    /** The network security configuration specified by struct
     * wlan_network_security for the network. */
    struct wlan_network_security security;//包含安全类型、密钥管理类型、密码等信息。
    /** The network IP address configuration specified by struct
     * wlan_ip_config that should be associated with this interface. */
    struct wlan_ip_config ip;
#if CONFIG_WPA2_ENTP
    /** WPA2 Enterprise identity, the max can be upto 256 characters */
    char identity[IDENTITY_MAX_LENGTH];
#if CONFIG_PEAP_MSCHAPV2
    char anonymous_identity[IDENTITY_MAX_LENGTH];
    /** password string */
    char password[PASSWORD_MAX_LENGTH];
#endif
#endif

    /* Private Fields */

    /**
     * If set to 1, the ssid field contains the specific SSID for this
     * network.
     * the Wi-Fi connection manager can only connect to networks whose SSID
     * matches.  If set to 0, the ssid field contents are not used when
     * deciding whether to connect to a network, the BSSID field is used
     * instead and any network whose BSSID matches is accepted.
     *
     * This field can be set to 1 if the network is added with the SSID
     * specified (not an empty string), otherwise it is set to 0.
     */
    unsigned ssid_specific : 1;
#if CONFIG_DRIVER_OWE
    /**
     * If set to 1, the ssid field contains the transitional SSID for this
     * network.
     */
    unsigned trans_ssid_specific : 1;
#endif
    /** If set to 1, the bssid field contains the specific BSSID for this
     *  network. The Wi-Fi connection manager can not connect to any other
     *  network with the same SSID unless the BSSID matches.  If set to 0, the
     *  Wi-Fi connection manager can connect to any network whose SSID matches.
     *
     *  This field set to 1 if the network is added with the BSSID
     *  specified (not set to all zeroes), otherwise it is set to 0. */
    unsigned bssid_specific : 1;
    /**
     * If set to 1, the channel field contains the specific channel for this
     * network. The Wi-Fi connection manager can not look for this network on
     * any other channel. If set to 0, the Wi-Fi connection manager can look
     * for this network on any available channel.
     *
     * This field set to 1 if the network is added with the channel
     * specified (not set to 0), otherwise it is set to 0. */
    unsigned channel_specific : 1;
    /**
     * If set to 0, any security that matches is used. This field is
     * internally set when the security type parameter above is set to
     * WLAN_SECURITY_WILDCARD.
     */
    unsigned security_specific : 1;
#if CONFIG_WPS2
    /** This indicates this network is used as an internal network for
     * WPS */
    unsigned wps_specific : 1;
#endif

    /** The network supports 802.11N. (For internal use only) */
    unsigned dot11n : 1;
#if CONFIG_11AC
    /** The network supports 802.11AC. (For internal use only) */
    unsigned dot11ac : 1;
#endif
#if CONFIG_11AX
    /** The network supports 802.11AX. (For internal use only) */
    unsigned dot11ax : 1;
#endif

#if CONFIG_11R
    /** Mobility Domain ID */
    uint16_t mdid;
    /** The network uses FT 802.1x security (For internal use only)*/
    unsigned ft_1x : 1;
    /** The network uses FT PSK security (For internal use only)*/
    unsigned ft_psk : 1;
    /** The network uses FT SAE security (For internal use only)*/
    unsigned ft_sae : 1;
#endif
#if CONFIG_DRIVER_OWE
    /** OWE Transition mode */
    unsigned int owe_trans_mode;
    /** The network transitional SSID, represented as a C string of up to 32 characters
     *  in length.
     *
     * This field is used internally.
     */
    char trans_ssid[IEEEtypes_SSID_SIZE + 1];
    /** Transitional SSID length
     *
     * This field is used internally.
     */
    unsigned int trans_ssid_len;
#endif
    /** Beacon period of associated BSS */
    uint16_t beacon_period;
    /** DTIM period of associated BSS */
    uint8_t dtim_period;
#if CONFIG_WIFI_CAPA
    /** Wi-Fi capabilities of UAP network 802.11n, 802.11ac or/and 802.11ax */
    uint8_t wlan_capa;
#endif
#if CONFIG_11V
    /** BTM mode */
    uint8_t btm_mode;
    /** BSS transition support (For internal use only) */
    bool bss_transition_supported;
#endif
#if CONFIG_11K
    /** Neighbor report support (For internal use only) */
    bool neighbor_report_supported;
#endif
};

/** This structure is for IEEE PS (power save) configuration */
struct wlan_ieeeps_config
{
    /** PS null interval in seconds */
    u32 ps_null_interval;
    /** Multiple DTIM interval */
    u32 multiple_dtim_interval;
    /** Listen interval */
    u32 listen_interval;
    /** Adhoc awake period */
    u32 adhoc_awake_period;
    /** Beacon miss timeout in milliseconds */
    u32 bcn_miss_timeout;
    /** Delay to PS in milliseconds */
    s32 delay_to_ps;
    /** Power save mode,
    0: Active mode,
    1: IEEE power save mode,
    2: Deep sleep power save mode,
    3: IEEE and deep sleep power save mode,
    4: WNM power save mode,
    5: WNM and deep sleep power save mode. */
    u32 ps_mode;
};

#if CONFIG_WIFI_TX_PER_TRACK
/** TX per tracking structure
 * Driver sets TX per tracking statistic to FW.
 * FW can check TX packet error rate periodically and
 * report per to host if per is high.
 */
struct wlan_tx_pert_info
{
    /** Enable/Disable TX per tracking */
    uint8_t tx_pert_check;
    /** Check period (unit sec) */
    uint8_t tx_pert_check_peroid;
    /** (Fail TX packet)/(Total TX packet) ratio (unit 10%)
     * default: 5
     */
    uint8_t tx_pert_check_ratio;
    /** A watermark of check number (default 5) */
    t_u16 tx_pert_check_num;
};
#endif
#if defined(RW610)
typedef enum
{
    CLI_DISABLE_WIFI,
    CLI_ENABLE_WIFI,
    CLI_RESET_WIFI,
} cli_reset_option;
#endif

enum wlan_mon_task_event
{
    HOST_SLEEP_HANDSHAKE = 1,
    HOST_SLEEP_EXIT,
    WIFI_RECOVERY_REQ,
};

#if CONFIG_HOST_SLEEP
enum wlan_hostsleep_state
{
    HOST_SLEEP_DISABLE,
    HOST_SLEEP_ONESHOT,
    HOST_SLEEP_PERIODIC,
};

#define WLAN_HOSTSLEEP_SUCCESS    1
#define WLAN_HOSTSLEEP_IN_PROCESS 2
#define WLAN_HOSTSLEEP_FAIL       3
#endif

#if CONFIG_TX_RX_HISTOGRAM
struct wlan_txrx_histogram_info
{
    /**  Enable or disable  */
    uint8_t enable;
    /** Choose to get TX, RX or both */
    t_u16 action;
};

#define FLAG_TX_HISTOGRAM       0x01
#define FLAG_RX_HISTOGRAM       0x02
#define DISABLE_TX_RX_HISTOGRAM 0x00
#define ENABLE_TX_RX_HISTOGRAM  0x01
#define GET_TX_RX_HISTOGRAM     0x02

/** Sum of TX packets for HT (802.11n high throughput) rate. */
typedef struct _tx_pkt_ht_rate_info
{
    /** Sum of TX packets for HT rate. Array index represents MSC0~MCS15,
    the following array indexs have the same effect. */
    u32 htmcs_txcnt[16];
    /** Sum of TX short GI (guard interval) packets for HT rate. */
    u32 htsgi_txcnt[16];
    /** Sum of TX STBC (space time block code) packets for HT rate. */
    u32 htstbcrate_txcnt[16];
} tx_pkt_ht_rate_info;

/** Sum of TX packets for VHT (802.11ac very high throughput) rate. */
typedef struct _tx_pkt_vht_rate_info
{
    /** Sum of TX packets for VHT rate. Array index represents MSC0~MCS9,
    the following array indexs have the same effect. */
    u32 vhtmcs_txcnt[10];
    /** Sum of TX short GI packets for HT mode. */
    u32 vhtsgi_txcnt[10];
    /** Sum of TX STBC (space time block code) packets for VHT mode. */
    u32 vhtstbcrate_txcnt[10];
} tx_pkt_vht_rate_info;

/** Sum of TX packets for HE (802.11ax high efficiency) rate. */
typedef struct _tx_pkt_he_rate_info
{
    /** Sum of TX packets for HE rate. Array index represents MSC0~MCS11,
    the following array indexs have the same effect. */
    u32 hemcs_txcnt[12];
    /** Sum of TX STBC (space time block code) packets for HE rate. */
    u32 hestbcrate_txcnt[12];
} tx_pkt_he_rate_info;

/** Sum of TX packets. */
typedef struct _tx_pkt_rate_info
{
    /** Sum of TX NSS (N*N MIMO spatial stream) packets.
    nss_txcnt[0] is for NSS 1,
    nss_txcnt[1] is for NSS 2.
    */
    u32 nss_txcnt[2];
    /** Sum of TX packets for 3 bandwidths.
    bandwidth_txcnt[0] is for 20MHz,
    bandwidth_txcnt[1] is for 40MHz,
    bandwidth_txcnt[2] is for 80MHz.
    */
    u32 bandwidth_txcnt[3];
    /** Sum of RX packets for 4 preamble format types.
    preamble_txcnt[0] is for preamble format 0,
    preamble_txcnt[1] is for preamble format 1,
    preamble_txcnt[2] is for preamble format 2,
    preamble_txcnt[3] is for preamble format 3,
    */
    u32 preamble_txcnt[4];
    /** Sum of TX LDPC (low density parity check) packets. */
    u32 ldpc_txcnt;
    /** Sum of TX RTS (require to send) packets */
    u32 rts_txcnt;
    /** RSSI of ACK packet */
    s32 ack_RSSI;
} tx_pkt_rate_info;

/** Sum of RX packets for HT (802.11n high throughput) rate. */
typedef struct _rx_pkt_ht_rate_info
{
    /** Sum of RX packets for HT rate. Array index represents MSC0~MCS15,
    the following array indexs have the same effect.
    */
    u32 htmcs_rxcnt[16];
    /** Sum of TX short GI (guard interval) packets for HT rate. */
    u32 htsgi_rxcnt[16];
    /** Sum of TX STBC (space time block code) packets for HT rate. */
    u32 htstbcrate_rxcnt[16];
} rx_pkt_ht_rate_info;

/** Sum of RX packets for VHT (802.11ac very high throughput) rate. */
typedef struct _rx_pkt_vht_rate_info
{
    /** Sum of RX packets for VHT rate. Array index represents MSC0~MCS9,
    the following array indexs have the same effect. */
    u32 vhtmcs_rxcnt[10];
    /** Sum of RX short GI (guard interval) packets for VHT rate. */
    u32 vhtsgi_rxcnt[10];
    /** Sum of RX STBC (space time block code) packets for VHT rate. */
    u32 vhtstbcrate_rxcnt[10];
} rx_pkt_vht_rate_info;

/** Sum of RX packets for HE (802.11ax high efficiency) rate. */
typedef struct _rx_pkt_he_rate_info
{
    /** Sum of TX packets for HE rate. Array index represents MSC0~MCS11,
    the following array indexs have the same effect. */
    u32 hemcs_rxcnt[12];
    /** Sum of TX STBC (space time block code) packets for HE rate. */
    u32 hestbcrate_rxcnt[12];
} rx_pkt_he_rate_info;

/** Sum of RX packets. */
typedef struct _rx_pkt_rate_info
{
    /** Sum of RX NSS (N*N MIMO spatial stream) packets.
    nss_txcnt[0] is for NSS 1,
    nss_txcnt[1] is for NSS 2.
    */
    u32 nss_rxcnt[2];
    /** Sum of received packets for all STBC rates. */
    u32 nsts_rxcnt;
    /** Sum of received packets for 3 bandwith types.
    bandwidth_rxcnt[0] is for 20MHz,
    bandwidth_rxcnt[1] is for 40MHz,
    bandwidth_rxcnt[2] is for 80MHz.
    */
    u32 bandwidth_rxcnt[3];
    /** Sum of received packets for 4 preamble format types.
    preamble_txcnt[0] is for preamble format 0,
    preamble_txcnt[1] is for preamble format 1,
    preamble_txcnt[2] is for preamble format 2,
    preamble_txcnt[3] is for preamble format 3,
    preamble_txcnt[4] and preamble_txcnt[5] are as reserved.
    */
    u32 preamble_rxcnt[6];
    /** Sum of packets for TX LDPC packets. */
    u32 ldpc_txbfcnt[2];
    /** Average RSSI */
    s32 rssi_value[2];
    /** RSSI value of path A */
    s32 rssi_chain0[4];
    /** RSSI value of path B */
    s32 rssi_chain1[4];
} rx_pkt_rate_info;
#endif

#if CONFIG_TX_AMPDU_PROT_MODE
#define TX_AMPDU_RTS_CTS            0
#define TX_AMPDU_CTS_2_SELF         1
#define TX_AMPDU_DISABLE_PROTECTION 2
#define TX_AMPDU_DYNAMIC_RTS_CTS    3

/** Set protection mode for the transmit AMPDU packet */
typedef struct _tx_ampdu_prot_mode_para
{
    /**
    mode,
    0: set RTS/CTS mode,
    1: set CTS to self mode,
    2: disable protection mode,
    3: set dynamic RTS/CTS mode.
    */
    int mode;
} tx_ampdu_prot_mode_para;
#endif

//typedef wifi_uap_client_disassoc_t wlan_uap_client_disassoc_t;

#if CONFIG_INACTIVITY_TIMEOUT_EXT
    typedef wifi_inactivity_to_t wlan_inactivity_to_t;
#endif

/* Wi-Fi connection manager API */
/** Initialize Wi-Fi driver and create the Wi-Fi driver thread.
 *
 * \param[in]  fw_start_addr Start address of the Wi-Fi firmware.
 * \param[in]  size Size of the Wi-Fi firmware.
 *
 * \return WM_SUCCESS if the Wi-Fi connection manager service has
 *         initialized successfully.
 * \return Negative value if initialization failed.
 */
int wlan_init(const uint8_t *fw_start_addr, const size_t size);

/** Start the Wi-Fi connection manager service.
 *
 * This function starts the Wi-Fi connection manager.
 *
 * \note The status of the Wi-Fi connection manager is notified asynchronously
 * through the callback, \a cb, with a WLAN_REASON_INITIALIZED event
 * (if initialization succeeded) or WLAN_REASON_INITIALIZATION_FAILED
 * (if initialization failed).
 * If the Wi-Fi connection manager fails to initialize, the caller should
 * stop Wi-Fi connection manager via wlan_stop() and try wlan_start() again.
 *
 * \param[in] cb A pointer to a callback function that handles Wi-Fi events. All
 * further WLCMGR events can be notified in this callback. Refer to enum
 * \ref wlan_event_reason for the various events for which this callback is called.
 *
 * \return WM_SUCCESS if the Wi-Fi connection manager service has started
 *         successfully.
 * \return -WM_E_INVAL if the \a cb pointer is NULL.
 * \return -WM_FAIL if an internal error occurred.
 * \return WLAN_ERROR_STATE if the Wi-Fi connection manager is already running.
 */
int wlan_start(int (*cb)(enum wlan_event_reason reason, void *data));

/** Stop the Wi-Fi connection manager service.
 *
 *  This function stops the Wi-Fi connection manager, causing station interface
 *  to disconnect from the currently connected network and stop the
 * UAP interface.
 *
 *  \return WM_SUCCESS if the Wi-Fi connection manager service has been
 *          stopped successfully.
 *  \return WLAN_ERROR_STATE if the Wi-Fi connection manager was not
 *          running.
 */
int wlan_stop(void);

/** Deinitialize Wi-Fi driver, send shutdown command to Wi-Fi firmware
 *  and delete the Wi-Fi driver thread.
 *  \param action Additional action to be taken with deinit
 *          WLAN_ACTIVE: no action to be taken
 */
void wlan_deinit(int action);

#if CONFIG_WPS2
    /** Generate valid pin code for WPS session.
    *
    *  This function generate pin for WPS pin session.
    *
    * \param[out]  pin A pointer to WPS pin to be generated.
    */
    void wlan_wps_generate_pin(uint32_t *pin);

    /** Start WPS pin session.
    *
    *  This function starts WPS pin session.
    *
    * \param[in]  pin Pin for WPS session.
    *
    * \return WM_SUCCESS if the pin entered is valid.
    * \return -WM_FAIL if invalid pin entered.
    */
    int wlan_start_wps_pin(uint32_t pin);

    /** Start WPS PBC session.
    *
    *  This function starts WPS PBC session.
    *
    * \return  WM_SUCCESS if successful
    */
    int wlan_start_wps_pbc(void);
    /**
    * Set None/WPS/802.1x session.
    *
    *\param[in] session       0 -- PROV_NON_SESSION_ATTEMPT, 1 -- PROV_WPS_SESSION_ATTEMPT, 2 -- PROV_ENTP_SESSION_ATTEMPT.
    */
    void wlan_set_prov_session(int session);

    /**
    * Get connect session type.
    *
    * \return 0 -- PROV_NON_SESSION_ATTEMPT, 1 -- PROV_WPS_SESSION_ATTEMPT, 2 -- PROV_ENTP_SESSION_ATTEMPT.
    */
    int wlan_get_prov_session(void);
#endif

/** Stop and remove all Wi-Fi network profiles.
 *
 *  \return WM_SUCCESS if successful otherwise return -WM_E_INVAL.
 */
int wlan_remove_all_network_profiles(void);

#if defined(RW610)
    /** Reset driver.
    *  \param[in] ResetOption Option including enable, disable or reset Wi-Fi driver
    *  can be chosen.
    */
    void wlan_reset(cli_reset_option ResetOption);
    /** Stop and remove all Wi-Fi network (access point).
    *
    *  \return WM_SUCCESS if successful.
    */
    int wlan_remove_all_networks(void);

    /**
    * This API destroy all tasks.
    */
    void wlan_destroy_all_tasks(void);
    /** Retrieve the status information of if Wi-Fi started.
    *
    *  \return TRUE if Wi-Fi network is started.
    *  \return FALSE if not started.
    */
    int wlan_is_started(void);

#endif // RW610

#if CONFIG_NCP
    /** UAP provisioning deinit callback function */
    void wlan_register_uap_prov_deinit_cb(int (*cb)(void));
    /** UAP provisioning cleanup callback function */
    void wlan_register_uap_prov_cleanup_cb(void (*cb)(void));
    /** Stop all Wi-Fi network.
    *
    *  \return WM_SUCCESS if successful.
    */
    int wlan_stop_all_networks(void);
#endif

#if CONFIG_RX_ABORT_CFG
struct wlan_rx_abort_cfg
{
    /** Enable/Disable RX abort configuration */
    uint8_t enable;
    /** RX weak RSSI threshold */
    int rssi_threshold;
};
#endif

#if CONFIG_RX_ABORT_CFG_EXT
struct wlan_rx_abort_cfg_ext
{
    /** Enable/Disable dyn RX abort on weak packet RSSI */
    int enable;
    /** Specify RSSI margin */
    int rssi_margin;
    /** Specify ceil RSSI threshold */
    int ceil_rssi_threshold;
    /** Specify floor RSSI threshold */
    int floor_rssi_threshold;
    /** Current dynamic RSSI threshold */
    int current_dynamic_rssi_threshold;
    /** RSSI config: default or user configured */
    int rssi_default_config;
    /** EDMAC status */
    int edmac_enable;
};
#endif

#if CONFIG_CCK_DESENSE_CFG
#define CCK_DESENSE_MODE_DISABLED 0
#define CCK_DESENSE_MODE_DYNAMIC  1
#define CCK_DESENSE_MODE_DYN_ENH  2

struct wlan_cck_desense_cfg
{
    /** CCK (complementary code keying) desense mode: 0:disable 1:normal 2:dynamic */
    t_u16 mode;
    /** Specify RSSI margin */
    int margin;
    /** Specify ceil RSSI threshold */
    int ceil_thresh;
    /** CCK (complementary code keying) desense "on" interval count */
    int num_on_intervals;
    /** CCK desense "off" interval count */
    int num_off_intervals;
};
#endif
#if CONFIG_RX_ABORT_CFG
    /**
    * Set/Get RX abort configure to/from firmware.
    *
    * \param[in,out] cfg A pointer to information buffer
    * \param[in] action Command action: get or set
    *
    * \return WM_SUCCESS if successful otherwise return -WM_FAIL.
    */
    int wlan_set_get_rx_abort_cfg(struct wlan_rx_abort_cfg *cfg, t_u16 action);
#endif

#if CONFIG_RX_ABORT_CFG_EXT
    /**
    * Set dynamic RX abort config to firmware.
    *
    * \param[in] cfg A pointer to information buffer
    *
    * \return WM_SUCCESS if successful otherwise return -WM_FAIL.
    */
    int wlan_set_rx_abort_cfg_ext(const struct wlan_rx_abort_cfg_ext *cfg);

    /**
    * Get dynamic RX abort config from firmware.
    *
    * \param[in,out] cfg A pointer to information buffer
    *
    * \return WM_SUCCESS if successful otherwise return -WM_FAIL.
    */
    int wlan_get_rx_abort_cfg_ext(struct wlan_rx_abort_cfg_ext *cfg);
#endif

#if CONFIG_CCK_DESENSE_CFG
    /**
    * Set/Get CCK (complementary code keying) desense configure to/from firmware.
    *
    * \param[in,out] cfg A pointer to information buffer
    * \param[in] action get or set.
    *
    * \return WM_SUCCESS if successful otherwise return -WM_FAIL.
    */
    int wlan_set_get_cck_desense_cfg(struct wlan_cck_desense_cfg *cfg, t_u16 action);
#endif
typedef struct 
{
    /** Channel scan parameter : Radio type */
    uint8_t radio_type;    // 无线电频段（ 0  2.4GHz 或 1   5GHz）
    /** Channel numder */
    uint8_t chan_number;   // 信道号
    /** Scan type Active = 1, Passive = 2 */
    uint8_t scan_type;   // 扫描类型（主动或被动）
    /** Scan time */
    uint16_t scan_time;       // 每个信道的扫描时间（单位：毫秒）
} wifi_scan_channel_list_t;
#ifndef MLAN_MAC_ADDR_LENGTH
#define MLAN_MAC_ADDR_LENGTH (6U)
#endif
#ifndef MAX_NUM_SSID
#define MAX_NUM_SSID 10
#endif
#ifndef MLAN_MAX_SSID_LENGTH
#define MLAN_MAX_SSID_LENGTH (32U)
#endif
/** V2 scan parameters */
typedef struct _wifi_scan_params_v2_t
{
#if CONFIG_WPA_SUPP
    /** Scan Only */
    u8 scan_only;
    /** BSSID present */
    u8 is_bssid;
    /** SSID present */
    u8 is_ssid;
#endif
    /** BSSID to scan */
    u8 bssid[MLAN_MAC_ADDR_LENGTH];
    /** SSID to scan */
#if CONFIG_COMBO_SCAN
    char ssid[MAX_NUM_SSID][MLAN_MAX_SSID_LENGTH + 1];
#else
    char ssid[MLAN_MAX_SSID_LENGTH + 1];
#endif
    /** Number of channels */
    u8 num_channels;
    /** Channel list with channel information */
    wifi_scan_channel_list_t chan_list[MAX_CHANNEL_LIST];

    /** Number of probes */
    u8 num_probes;
#if CONFIG_SCAN_WITH_RSSIFILTER
    /** Threshold of rssi */
    s16 rssi_threshold;
#endif
#if CONFIG_SCAN_CHANNEL_GAP
    /** scan channel gap */
    u16 scan_chan_gap;
#endif
    /** Callback to be called when scan is completed */
    int (*cb)(unsigned int count);
} wifi_scan_params_v2_t;
typedef wifi_scan_params_v2_t wlan_scan_params_v2_t;

#endif /* __WLAN_WIFI_H__ */
