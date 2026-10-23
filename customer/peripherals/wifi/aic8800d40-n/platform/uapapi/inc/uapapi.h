/** @file  uapapi.h
 *
 *  @brief Header file for API for UAP configuration
 */
/************************************************************************
Change log:
    28/10/11: Initial creation
************************************************************************/

#ifndef _UAPAPI_H
#define _UAPAPI_H

#include        <string.h>      /* memset */

/** Character, 1 byte */
typedef char t_s8;
/** Unsigned character, 1 byte */
typedef unsigned char t_u8;

/** Short integer */
typedef signed short t_s16;
/** Unsigned short integer */
typedef unsigned short t_u16;

/** Long integer */
typedef signed long t_s32;
/** Unsigned long integer */
typedef unsigned long t_u32;

/** Long long integer */
typedef signed long long t_s64;
/** Unsigned long integer */
typedef unsigned long long t_u64;

/** BSS START */
#define UAP_BSS_START               0
/** BSS STOP */
#define UAP_BSS_STOP                1
/** BSS RESET */
#define UAP_BSS_RESET               2

/** Action field value : get */
#define UAP_ACTION_GET  0
/** Action field value : set */
#define UAP_ACTION_SET  1

/** Mac filter mode disabled*/
#define UAP_MAC_FILTER_MODE_DISABLED    0
/** Mac filter mode white list*/
#define UAP_MAC_FILTER_MODE_WHITE_LIST  1
/** Mac filter mode black list*/
#define UAP_MAC_FILTER_MODE_BLACK_LIST  2

/**
* Invalid & Valid values of uap_config_param fields
*/
#define UAP_INVALID_MODE            0
#define UAP_INVALID_PS_MGMT_FLAG    0xFF
#define UAP_INVALID_TX_POWER        0xFF

#define UAP_INVALID_BYTE            0xFF
#define UAP_INVALID_HWORD           0xFFFF
#define UAP_INVALID_WORD            0xFFFFFFFF

#define UAP_BAND_B          (0x01 << 0)
#define UAP_BAND_G          (0x01 << 1)
#define UAP_BAND_A          (0x01 << 2)
#define UAP_BAND_GN         (0x01 << 3)
#define UAP_BAND_AN         (0x01 << 4)
#define UAP_BAND_AX         (0x01 << 5)
#define UAP_BAND_AC         (0x01 << 6)


/** RF: 2.4G */
#define UAP_RF_24_G                           0x01
/** RF: 5.0G */
#define UAP_RF_50_G                           0x02

/** UAP Channel Bandwidth */
#define UAP_CHAN_WIDTH_INVALID  0x00
#define UAP_CHAN_WIDTH_20MHZ      0x01
#define UAP_CHAN_WIDTH_10MHZ      0x02
#define UAP_CHAN_WIDTH_40MHZ      0x03

/** UAP Secondary Channel */
/** no secondary channel */
#define UAP_NO_SEC_CHAN       0x00
/** secondary channel is below */
#define UAP_SEC_CHAN_ABOVE    0x01
/** secondary channel is above */
#define UAP_SEC_CHAN_BELOW    0x02

/* Bitmap for protocol to use */
/** No security */
#define UAP_PROTOCOL_NO_SECURITY            1
/** Static WEP */
#define UAP_PROTOCOL_STATIC_WEP             2
/** WPA */
#define UAP_PROTOCOL_WPA                    8
/** WPA2 */
#define UAP_PROTOCOL_WPA2                   32  // 0x20
/** WPA3 */
#define UAP_PROTOCOL_WPA3                   64  // 0x40
/** WP2 Mixed */
#define UAP_PROTOCOL_WPA2_WPA_MIXED         40  // 0x28
/** WP3 Mixed */
#define UAP_PROTOCOL_WPA3_WPA2_MIXED        96  // 0x60

/* Bitmap for unicast/bcast cipher type */
/** None */
#define UAP_CIPHER_NONE                 0
/** WEP 40 */
#define UAP_CIPHER_WEP_40               1
/** WEP 104 */
#define UAP_CIPHER_WEP_104              2
/** TKIP */
#define UAP_CIPHER_TKIP                 4
/** AES CCMP */
#define UAP_CIPHER_AES_CCMP             8
/** AES CCMP + TKIP cipher */
#define UAP_AES_CCMP_TKIP               12

#define UAP_AUTH_MODE_OPEN                  (0)
#define UAP_AUTH_MODE_SHARED                (1)

#define UAP_PKT_FWD_CTL_BY_FW               (0x01 << 0)
#define UAP_PKT_FWD_INTRA_BSS_BCAST         (0x01 << 1)
#define UAP_PKT_FWD_INTRA_BSS_UCAST         (0x01 << 2)
#define UAP_PKT_FWD_INTER_BSS_UCAST         (0x01 << 3)

/** sleep_param */
typedef struct _uap_ps_sleep_param
{
    /** control bitmap */
    t_u32 ctrl_bitmap;
    /** minimum sleep period (micro second) */
    t_u32 min_sleep;
    /** maximum sleep period (micro second) */
    t_u32 max_sleep;
} uap_ps_sleep_param;

/** inactivity sleep_param */
typedef struct _uap_inact_sleep_param
{
    /** inactivity timeout (micro second) */
    t_u32 inactivity_to;
    /** miniumu awake period (micro second) */
    t_u32 min_awake;
    /** maximum awake period (micro second) */
    t_u32 max_awake;
} uap_inact_sleep_param;

/** flag for ps mode */
#define PS_FLAG_PS_MODE                 1
/** flag for sleep param */
#define PS_FLAG_SLEEP_PARAM             2
/** flag for inactivity sleep param */
#define PS_FLAG_INACT_SLEEP_PARAM       4

/** Disable power mode */
#define PS_MODE_DISABLE                      0
/** Enable periodic dtim ps */
#define PS_MODE_PERIODIC_DTIM                1
/** Enable inactivity ps */
#define PS_MODE_INACTIVITY                   2

/** sleep parameter */
#define SLEEP_PARAMETER                     1
/** inactivity sleep parameter */
#define INACTIVITY_SLEEP_PARAMETER          2

/** sleep parameter : lower limit in micro-sec */
#define PS_SLEEP_PARAM_MIN                  5000
/** sleep parameter : upper limit in micro-sec */
#define PS_SLEEP_PARAM_MAX                  32000
/** power save awake period minimum value in micro-sec */
#define PS_AWAKE_PERIOD_MIN                 2000

/** ps_mgmt */
typedef struct _uap_ps_mgmt
{
    /** flags for valid field */
    t_u16 flags;
    /** power mode */
    t_u16 ps_mode;
    /** sleep param */
    uap_ps_sleep_param sleep_param;
    /** inactivity sleep param */
    uap_inact_sleep_param inact_param;
} uap_ps_mgmt;

/*********************************************************************/

/* UAP API Command ID */
#define BSS_START 1
#define BSS_STOP   0

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

/** Default device name */
#define DEFAULT_DEV_NAME    "uap0"

/** Success */
#define UAP_SUCCESS     0
/** Failure */
#define UAP_FAILURE     -1
/** Command is successful */
#define CMD_SUCCESS     0
/** Command fails */
#define CMD_FAILURE     -1

/** TRUE */
#define UAP_TRUE                    (1)
/** FALSE */
#define UAP_FALSE                   (0)

/** BSS start error : Invalid parameters */
#define BSS_FAILURE_START_INVAL     2

/** Size of command buffer */
#define MRVDRV_SIZE_OF_CMD_BUFFER       (2 * 1024)

/** Maximum number of clients supported by AP */
#define MAX_NUM_CLIENTS         16
/** Maximum number of MAC addresses for one-shot filter modifications */
#define MAX_MAC_ONESHOT_FILTER  16
/** Maximum SSID length */
#define MAX_SSID_LENGTH         32
/** Maximum SSID length */
#define MIN_SSID_LENGTH         1
/** Maximum data rates */
#define MAX_DATA_RATES          14
/** MSB bit is set if its a basic rate */
#define BASIC_RATE_SET_BIT      0x80

/** Maximum TX Power Limit */
#define MAX_TX_POWER    30
/** Minimum TX Power Limit */
#define MIN_TX_POWER    0

/** Maximum channel number in bg mode */
#define MAX_CHANNELS_BG 14
/** Maximum channels */
#define MAX_CHANNELS    14
#define DEFAULT_MAX_VALID_CHANNEL_BG 11

/** Max number of stations */
#define MAX_STA_COUNT     10

/** Maximum DTIM period */
#define MAX_DTIM_PERIOD 100

/** Maximum BEACON period */
#define MAX_BEACON_PERIOD 4000

/** Minimum BEACON period */
#define MIN_BEACON_PERIOD 50

/** Maximum IE buffer length */
#define MAX_IE_BUFFER_LEN 256

/** Maximum custom IE count */
#define MAX_CUSTOM_IE_COUNT 4

/** Maximum number of rates allowed at a time */
#define MAX_RATES               12

/** Default wait period in seconds */
#define DEFAULT_WAIT_TIME       3

/** Maximum valid value of Deauth reason code */
#define MAX_DEAUTH_REASON_CODE     0xFFFF

#ifndef IFNAMSIZ
    /** Interface name length */
    #define IFNAMSIZ        16
#endif

#ifndef ETH_ALEN
    /** MAC address length */
    #define ETH_ALEN    6
#endif

/** Action field value : get */
#define ACTION_GET  0
/** Action field value : set */
#define ACTION_SET  1

/**
* Invalid & Valid values of uap_config_param fields
*/
#define INVALID_MODE            0
#define INVALID_PS_MGMT_FLAG    0xFF
#define INVALID_TX_POWER        0xFF

#define INVALID_BYTE 0xFF
#define INVALID_HWORD 0xFFFF
#define INVALID_WORD 0xFFFFFFFF

/** BSS Band Configuration: [7-6] Channel Selection Mode; 00 manual, 01 ACS */
#define BAND_CONFIG_CHAN_SEL_MASK        0xC0
/** Band config ACS mode */
#define BAND_CONFIG_ACS_MODE    0x40
/** Band config manual */
#define BAND_CONFIG_MANUAL      0x00

/** BSS Band Configuration: [5-4] 00 - no secondary channel,
                        01 - secondary channel is above
                            03 - secondary channel is below */
#define BAND_CONFIG_SEC_CHAN_MASK        0x30
/** secondary channel is below */
#define SECOND_CHANNEL_BELOW    0x30
/** secondary channel is above */
#define SECOND_CHANNEL_ABOVE    0x10
/** max primary channel support secondary channel above */
#define MAX_CHANNEL_ABOVE       7
/** min primary channel support secondary channel below */
#define MIN_CHANNEL_BELOW       5

/** BSS Band Configuration: [3-2] Channel Width; 00 20 MHz */
#define BAND_CONFIG_WIDTH_MASK        0x0C
#define BAND_CONFIG_WIDTH_20MHZ      0x00
#define BAND_CONFIG_WIDTH_10MHZ      0x04
#define BAND_CONFIG_WIDTH_40MHZ      0x08

/** BSS Band Configuration: [1-0] Band Info; 00 2.4 GHz */
/** Band Config mask */
#define BAND_CONFIG_BAND_INFO_MASK        0x03
/** Band Config 2.4GHz */
#define BAND_CONFIG_2_4GHZ      0x00
/** Band config 5GHz */
#define BAND_CONFIG_5GHZ        0x01


#define BAND_B          (0x01)
#define BAND_G          (0x01 << 1)
#define BAND_A          (0x01 << 2)
#define BAND_GN        (0x01 << 3)
#define BAND_AN        (0x01 << 4)


/** RF: 2.4G */
#define UAP_RF_24_G                           0x01
/** RF: 5.0G */
#define UAP_RF_50_G                           0x02

/** UAP Channel Bandwidth */
#define UAP_CHAN_WIDTH_INVALID  0x00
#define UAP_CHAN_WIDTH_20MHZ      0x01
#define UAP_CHAN_WIDTH_10MHZ      0x02
#define UAP_CHAN_WIDTH_40MHZ      0x03

/** UAP Secondary Channel
        00 - no secondary channel */
/** secondary channel is below */
#define UAP_SEC_CHAN_ABOVE    0x01
/** secondary channel is above */
#define UAP_SEC_CHAN_BELOW    0x02

#define MAX_TOTAL_CHANNELS 14
#define MAX_CHANNELS_BG_FCC_DOMAIN 11
#define MAX_CHANNELS_A_FCC_DOMAIN 24

/* Bitmap for protocol to use */
/** No security */
#define PROTOCOL_NO_SECURITY        1
/** Static WEP */
#define PROTOCOL_STATIC_WEP         2
/** WPA */
#define PROTOCOL_WPA                8
/** WPA2 */
#define PROTOCOL_WPA2               32      // 0x20
/** WP2 Mixed */
#define PROTOCOL_WPA2_MIXED         40  // 0x28

/* Bitmap for unicast/bcast cipher type */
/** None */
#define CIPHER_NONE                 0
/** WEP 40 */
#define CIPHER_WEP_40               1
/** WEP 104 */
#define CIPHER_WEP_104              2
/** TKIP */
#define CIPHER_TKIP                 4
/** AES CCMP */
#define CIPHER_AES_CCMP             8
/** Valid cipher bitmap */
#define CIPHER_BITMAP               0x0c
/** Valid protocol bitmap */
#define PROTOCOL_BITMAP             0x28
/** AES CCMP + TKIP cipher */
#define AES_CCMP_TKIP               12

/** Key_mgmt_psk */
#define KEY_MGMT_NONE   0x04
/** Key_mgmt_none */
#define KEY_MGMT_PSK    0x02

/** MLAN 802.11 MAC Address */
typedef t_u8 uap_802_11_mac_addr[ETH_ALEN];
/** mac_filter data structure */
typedef struct _uap_mac_filter
{
    /** Mac filter mode */
    t_u16 filter_mode;
    /** Mac adress count */
    t_u16 mac_count;
    /** Mac address list */
    uap_802_11_mac_addr mac_list[MAX_MAC_ONESHOT_FILTER];
} uap_mac_filter;

/** wpa parameter */
typedef struct _uap_wpa_param
{
    /** Pairwise cipher WPA */
    t_u8 pairwise_cipher_wpa;
    /** Pairwise cipher WPA2 */
    t_u8 pairwise_cipher_wpa2;
    /** Group cipher */
    t_u8 group_cipher;
    /** RSN replay protection */
    t_u8 rsn_protection;
    /** Passphrase length */
    t_u32 length;
    /** Passphrase */
    t_u8 passphrase[64];
    /** Group key rekey time */
    t_u32 gk_rekey_time;
} uap_wpa_param;

/** wep key */
typedef struct _uap_wep_key
{
    /** Key index 0-3 */
    t_u8 key_index;
    /** Is default */
    t_u8 is_default;
    /** Length */
    t_u16 length;
    /** Key data */
    t_u8 key[26];
} uap_wep_key;

/** wep param */
typedef struct _uap_wep_param
{
    /** Key 0 */
    uap_wep_key key0;
    /** Key 1 */
    uap_wep_key key1;
    /** Key 2 */
    uap_wep_key key2;
    /** Key 3 */
    uap_wep_key key3;
} uap_wep_param;

/** station info */
typedef struct _sta_info
{
    /** STA MAC address */
    t_u8 mac_address[ETH_ALEN];
    /** Power mfg status */
    t_u8 power_mfg_status;
    /** RSSI */
    t_s8 rssi;
} sta_info;

/** sta_list structure */
typedef struct _sta_list
{
    /** station count */
    t_u16 sta_count;
    /** station list */
    sta_info info[MAX_NUM_CLIENTS];
} sta_list;

/* UAP API Structure */
typedef struct _uap_config_param
{
    t_u16 action; /* 0: get, 1: set */
    t_u32 ssid_len;
    t_u8 ssid[32];
    t_u8 bcast_ssid_ctrl;
    t_u8 mode;
    t_u8 channel;
    t_u8 acs_mode;
    t_u8 rf_band;
    t_u8 channel_bandwidth;
    t_u8 sec_chan;
    t_u16 max_sta_count;
    t_u32 sta_ageout_timer;
    t_u32 ps_sta_ageout_timer;
    t_u16 auth_mode;
    t_u16 protocol;
    t_u16 ht_cap_info;
    t_u8 ampdu_param;
    t_u8 tx_power;
    t_u8 enable_2040coex;
    uap_mac_filter filter;
    uap_wpa_param wpa_cfg;
    uap_wep_param wep_cfg;
    t_u8 state_802_11d;
    t_u8 country_code[4];
    uap_ps_mgmt power_mode;
} uap_config_param;

/** UAP API: Scan Channels Cfg structure */
typedef struct _uap_config_scan_chan
{
    /** Action */
    t_u16 action; /* 0: get, 1: set */
    /** Total no of bands */
    t_u16 band;
    /** Total no of channels */
    t_u16 total_chan;
    /** Channel no .array */
    t_u8 chan_num[MAX_CHANNELS];
} uap_config_scan_chan;

//ret
/** Failure */
#define UAP_FAILURE_INVALID_SSID_LEN                  (-1)
#define UAP_FAILURE_INVALID_CHAN                      (-2)
#define UAP_FAILURE_INVALID_BW40                      (-3)
#define UAP_FAILURE_INVALID_FREQ                      (-4)
#define UAP_FAILURE_GET_SETTING                       (-5)
#define UAP_FAILURE_INVALID_KEY_MGMT                  (-6)
#define UAP_FAILURE_INVALID_PARAM                     (-7)
#define UAP_FAILURE_SET_SSID                          (-8)
#define UAP_FAILURE_SET_IGNORE_BCAST_SSID             (-9)
#define UAP_FAILURE_INVALID_HW_MODE                   (-10)
#define UAP_FAILURE_SET_HT                            (-11)
#define UAP_FAILURE_SET_HE                            (-12)
#define UAP_FAILURE_SET_FREQ                          (-13)
#define UAP_FAILURE_INVALID_RF_BAND                   (-14)
#define UAP_FAILURE_SET_HT40                          (-15)
#define UAP_FAILURE_SET_KEY_MGMT                      (-16)
#define UAP_FAILURE_INVALID_PROTO                     (-17)
#define UAP_FAILURE_SET_AP_MODE                       (-18)
#define UAP_FAILURE_INVALID_OP                        (-19)
#define UAP_FAILURE_SET_PSK                           (-20)
#define UAP_FAILURE_INVALID_PSK_LEN                   (-21)
#define UAP_FAILURE_SET_PROTO                         (-22)
#define UAP_FAILURE_NOT_INITIALIZED                   (-23)
#define UAP_FAILURE_LIST_STA                          (-24)
#define UAP_FAILURE_DEAUTH                            (-25)
#define UAP_FAILURE_INVALID_MAC_FILTER_MODE           (-26)
#define UAP_FAILURE_MAC_FILTER_SET                    (-27)
#define UAP_FAILURE_MAC_FILTER_GET                    (-28)
#define UAP_FAILURE_REG_CB                            (-29)
#define UAP_FAILURE_INVALID_BW                        (-30)
#define UAP_FAILURE_GET_MAX_NUM_STA                   (-31)
#define UAP_FAILURE_SET_MAX_NUM_STA                   (-32)
#define UAP_FAILURE_SET_AP_MAX_INACTIVITY             (-33)
#define UAP_FAILURE_SET_AUTH_ALG                      (-34)
#define UAP_FAILURE_INVALID_AUTH_MODE                 (-35)
#define UAP_FAILURE_SET_WEP_KEY                       (-36)
#define UAP_FAILURE_SET_WEP_TX_KEYIDX                 (-37)
#define UAP_FAILURE_INVALID_WEP_KEY                   (-38)
#define UAP_FAILURE_GET_COUNTRY                       (-39)
#define UAP_FAILURE_SET_COUNTRY                       (-40)
#define UAP_FAILURE_SET_ACS                           (-41)
#define UAP_FAILURE_GET_FREQ_LIST                     (-42)
#define UAP_FAILURE_SET_FREQ_LIST                     (-43)
#define UAP_FAILURE_INVALID_FREQ_LIST                 (-44)
#define UAP_FAILURE_GET_AP_ISOLATE                    (-45)
#define UAP_FAILURE_SET_AP_ISOLATE                    (-46)
#define UAP_FAILURE_INVALID_AP_ISOLATE                (-47)
#define UAP_FAILURE_INVALID_GK_REKEY_TIME             (-48)
#define UAP_FAILURE_SET_GK_REKEY_TIME                 (-49)
#define UAP_FAILURE_SET_IEEE80211D                    (-50)
#define UAP_FAILURE_INVALID_80211D                    (-51)
#define UAP_FAILURE_SET_2040COEX                      (-52)
#define UAP_FAILURE_SET_PAIRWISE_CIPHER               (-53)
#define UAP_FAILURE_SET_GROUP_CIPHER                  (-54)

/** Maximum number of clients supported by AP */
#define MAX_NUM_CLIENTS         16
/** Maximum number of MAC addresses for one-shot filter modifications */
#define MAX_MAC_ONESHOT_FILTER  16
/** Maximum SSID length */
#define UAP_MAX_SSID_LENGTH         32
/** Maximum SSID length */
#define MIN_SSID_LENGTH         1
/** Maximum channels */
#define UAP_MAX_CHANNELS    14
/** Max number of stations */
#define UAP_MAX_STA_COUNT     16


/* UAP API: Pkt Fwd Ctrl Structure */
typedef struct _uap_pkt_fwd_ctl
{

    /* 0: get, 1: set */
    t_u16 action;
    /* bit 0 -- Packet forwarding handled by Host (0) or Firmware (1)
     *  bit 1 -- Intra-BSS broadcast packets are allowed (0) or denied (1)
     *  bit 2 -- Intra-BSS unicast packets are allowed (0) or denied (1)
     *  bit 3 -- Inter-BSS unicast packets are allowed (0) or denied (1) */
    t_u16 pkt_fwd_ctl;
} uap_pkt_fwd_ctl;

/** Function Prototype Declaration */
t_s32 UAP_Params_Config(uap_config_param *uap_param);
t_s32 UAP_BSS_Config(t_u32 start_stop);
t_s32 UAP_STA_List(sta_list *list);
t_u8 GetWiFiInitStat(void);
t_u8 UAP_STATUS(void);
t_s32 UAP_Scan_Channels_Config(uap_config_scan_chan *param);
int GetMACAddr(t_u8 *macbuf);
int UAP_GetStaNum(void);
void UAP_IntraBSSUnicastDeny(int enable);
#endif /* _UAPAPI_H */
