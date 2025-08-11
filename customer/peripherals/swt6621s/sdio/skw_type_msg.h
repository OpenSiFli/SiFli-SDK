#ifndef __SKW_TYPE_MSG_H__
#define __SKW_TYPE_MSG_H__
#include "rtthread.h"
#include "mlan_ieee.h"

#define SKW_MAX_PEER_SUPPORT                32
#define SKW_NR_TID                      8
/* SKW_NUM_DEFAULT_KEY + SKW_NUM_DEFAULT_MGMT_KEY */
#define SKW_NUM_MAX_KEY                 6
#ifndef IEEE80211_CCMP_PN_LEN
    #define IEEE80211_CCMP_PN_LEN       6
    #define SKW_PN_LEN                      6
#endif
#ifndef IEEE80211_NUM_TIDS
    #define IEEE80211_NUM_TIDS 16
#endif
#define SKW_ETHER_FRAME            0
#define SKW_80211_FRAME            1

#define IPPROTO_IP      0
#define IPPROTO_ICMP    1
#define IPPROTO_TCP     6
#define IPPROTO_UDP     17

#define SKW_WIFI_DUMP     1

#define WLAN_MAX_KEY_LEN 32

struct skw_key
{
    //struct rcu_head rcu;
    u32 key_len;
    u8 key_data[32];
    u8 rx_pn[IEEE80211_NUM_TIDS][SKW_PN_LEN];
};
enum  SKW_STATES
{
    SKW_STATE_NONE,
    SKW_STATE_AUTHING,
    SKW_STATE_AUTHED,
    SKW_STATE_ASSOCING,
    SKW_STATE_ASSOCED,
    SKW_STATE_COMPLETED,
};
enum nl80211_iftype
{
    NL80211_IFTYPE_UNSPECIFIED = 0, // 未指定类型
    NL80211_IFTYPE_ADHOC,           // IBSS（Ad-Hoc，自组网）
    NL80211_IFTYPE_STATION,         // STA（客户端模式）
    NL80211_IFTYPE_AP,              // AP（接入点模式）
    NL80211_IFTYPE_AP_VLAN,         // AP VLAN
    NL80211_IFTYPE_WDS,             // WDS（无线分布式系统）
    NL80211_IFTYPE_MONITOR,         // 监控模式
    NL80211_IFTYPE_MESH_POINT,      // Mesh 点
    NL80211_IFTYPE_P2P_CLIENT,      // P2P 客户端
    NL80211_IFTYPE_P2P_GO,          // P2P Group Owner
    NL80211_IFTYPE_P2P_DEVICE,      // P2P 设备
    NL80211_IFTYPE_OCB,             // OCB（车联网）
    NL80211_IFTYPE_NAN,             // NAN（邻近感知网络）
    NL80211_IFTYPE_MAX
};

struct skw_sm
{
    u8 *addr;
    u8 inst;
    u8 iface_iftype;
    u16 flags; /* reference SKW_SM_FLAG_ */
    enum SKW_STATES state;
};

struct skw_txba_ctrl
{
    u16 bitmap;
    u16 blacklist;
    u8 tx_try[SKW_NR_TID];
};
struct skw_rate
{
    u8 flags;
    u8 mcs_idx;
    u16 legacy_rate;
    u8 nss;
    u8 bw;
    u8 gi;
    u8 he_ru;
    u8 he_dcm;
} ;
struct skw_stats_info
{
    s16 rssi;
    u64 pkts;
    u64 bytes;
    u64 drops;
    u64 cal_time;
    u64 cal_bytes;
    u8  tx_psr;
    u32 tx_failed;
    u16 filter_cnt[35];
    u16 filter_drop_offload_cnt[35];
    struct skw_rate rate;
};
struct skw_key_conf
{
    u8 skw_cipher;
    u8 installed_bitmap;
    u8 flags; /* reference to SKW_KEY_FLAG_ */
    u8 wep_idx;
    struct rt_mutex lock;
    struct skw_key *key[SKW_NUM_MAX_KEY];
};
#ifndef atomic_t
    #define atomic_t int
#endif

struct skw_tid_rx
{
    u16 win_start;
    u16 win_size;
    u32 stored_num;
    int ref_cnt;
    //struct rcu_head rcu_head;
    struct skw_reorder_rx *reorder;
    struct sk_buff_head *reorder_buf;
};
typedef struct
{
    int no;
} spinlock_t;
typedef struct
{
    int no;
} timer_list;
struct skw_rx_todo
{
    spinlock_t lock;
    //struct list_head list;
    u16 seq;
    u16 reason;
    bool actived;
};

struct skw_rx_timer
{
    u16 sn;
    u16 resv;
    int ref_cnt;
};
struct skw_peer;
struct skw_reorder_rx
{
    u32 tid: 4;
    u32 inst: 2;
    u32 peer_idx: 5;
    u32 resv: 21;

    atomic_t ref_cnt;
    struct skw_core *skw;
    struct skw_peer *peer;
    //struct timer_list timer;
    struct skw_rx_timer expired;

    struct skw_rx_todo todo;

    spinlock_t lock;
    struct skw_tid_rx *tid_rx;
};
struct skw_peer
{
    u8 idx;
    u8 flags; /* reference SKW_PEER_FLAG_ */
    u8 addr[ETH_ALEN];
    u16 channel;
    u16 rx_tid_map;
    be32 ip_addr;

    int rx_filter;
    struct skw_sm sm;
    struct skw_iface *iface;
    struct skw_key_conf ptk_conf, gtk_conf;

    struct skw_txba_ctrl txba;
    struct skw_reorder_rx reorder[SKW_NR_TID];
    struct skw_stats_info tx, rx;
};

struct skw_ctx_entry
{
    u8 idx;
    u8 padding;
    u8 addr[ETH_ALEN];
    //struct rcu_head rcu;
    struct skw_peer *peer;
};
enum SKW_BAND
{
    SKW_BAND_2GHZ = 0,   // 2.4GHz 频段
    SKW_BAND_5GHZ = 1,   // 5GHz 频段
    SKW_BAND_6GHZ = 2,   // 6GHz 频段（如支持Wi-Fi 6E）
    SKW_BAND_MAX
};
struct skw_peer_ctx
{
    int idx;
    struct rt_mutex lock;
    struct skw_peer *peer;
    struct skw_ctx_entry *entry;
};

struct skw_join_param
{
    uint8_t chan_num;               // 信道号（如1、6、36等）
    uint8_t center_chn1;            // 第一个中心信道（用于HT/VHT/HE等宽带信道）
    uint8_t center_chn2;            // 第二个中心信道（用于80+80MHz等）
    uint8_t bandwidth;              // 带宽（如20/40/80/160MHz，驱动自定义枚举/宏）
    uint8_t band;     //5              // 频段（2.4G/5G/6G等，参考enum SKW_BAND）
    uint16_t beacon_interval;       // Beacon 间隔（单位：TU，通常为100）
    IEEEtypes_CapInfo_t capability; // 能力信息字段（802.11标准定义，关联帧中的capability字段）
    uint8_t bssid_index;            // MBSSID 扩展用，主BSSID的索引
    uint8_t max_bssid_indicator;    // MBSSID 扩展用，最大BSSID数量指示
    uint8_t bssid[6];               // 目标AP的BSSID（MAC地址）
    uint16_t roaming: 1;            // 是否为漫游连接（1位），1表示漫游
    uint16_t reserved: 15;          // 保留位
    uint16_t bss_ie_offset;         // BSS IE数据的偏移（相对于本结构体起始地址）
    uint32_t bss_ie_len;            // BSS IE数据的长度
    u8 bss_ie[];               // 变长数组，存放BSS的IE（信息元素）数据
} __attribute__((packed)) ;

struct skw_msg    //8个字节
{
    /* for a global message, inst_id should be 0 */
    u8 inst_id: 4; ////对应消息发送的req的ID
    /* reference SKW_MSG_TYPE */
    u8 type: 4; //00//event与ACK的标识
    u8 id;//02 对应返回的消息类型  在skw_msg.h中定义  SKW_CMD_ID
    u16 seq;//00 02  //对应的消息ID
    u16 total_len;//00 08
    u8 resv[2];//00 00
    u16 data[0];//cmd tx没有   只有rx  cmd 的ack 才有data
};

struct tcp_header
{
    uint16_t source_port;       // 源端口（16位）
    uint16_t dest_port;         // 目的端口（16位）
    uint32_t seq_num;           // 序列号（32位）
    uint32_t ack_num;           // 确认号（32位）
    uint8_t  data_offset_res;   // 数据偏移（4位） + 保留位（4位）
    uint8_t  flags;             // 标志位（6位）
    uint16_t window_size;       // 窗口大小（16位）
    uint16_t checksum;          // 校验和（16位）
    uint16_t urgent_ptr;        // 紧急指针（16位）
    // 可选字段（Options，可变长度）
    // 填充（Padding，保证总长度为4字节的倍数）
};
enum nl80211_auth_type
{
    NL80211_AUTHTYPE_OPEN_SYSTEM = 0,   // 开放系统认证（Open System）
    NL80211_AUTHTYPE_SHARED_KEY = 1,    // 共享密钥认证（WEP Shared Key）
    NL80211_AUTHTYPE_FT = 2,            // 快速过渡（FT，802.11r）
    NL80211_AUTHTYPE_NETWORK_EAP = 3,   // 网络EAP认证（WPA/WPA2企业）
    NL80211_AUTHTYPE_SAE = 4,           // SAE认证（WPA3）
    NL80211_AUTHTYPE_FILS_SK = 5,       // FILS SK认证
    NL80211_AUTHTYPE_FILS_SK_PFS = 6,   // FILS SK PFS认证
    NL80211_AUTHTYPE_FILS_PK = 7,       // FILS PK认证
    NL80211_AUTHTYPE_MAX,               // 最大值（边界检查用）
    NL80211_AUTHTYPE_AUTOMATIC = 32768, // 自动选择认证类型（由驱动/上层决定）
};
struct cfg80211_auth_request
{
    struct cfg80211_bss *bss;         // 目标AP的BSS信息
    const u8 *ie;                     // 认证帧中的IE数据
    size_t ie_len;                    // IE数据长度
    const u8 *key;                    // WEP密钥（如有）
    u8 key_len;                       // 密钥长度
    u8 key_idx;                       // 密钥索引
    enum nl80211_auth_type auth_type; // 认证类型（OPEN、SHARED、SAE等）
    bool privacy;                     // 是否启用加密
#if 1//LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
    const u8 *auth_data;              // SAE等扩展认证数据
    size_t auth_data_len;             // 扩展认证数据长度
#else
    const u8 *sae_data;               // 旧版本SAE数据
    size_t sae_data_len;
#endif
};
struct skw_auth_param
{
    u16 auth_algorithm;
    u16 key_data_offset;
    u16 key_data_len;
    u16 auth_data_offset;//wpa3 会使用到
    u16 auth_data_len;
    u16 auth_ie_offset;
    u16 auth_ie_len;
    u8  data[];//
}  ;
enum SKW_CHAN_BW_INFO
{
    SKW_CHAN_WIDTH_20,
    SKW_CHAN_WIDTH_40,
    SKW_CHAN_WIDTH_80,
    SKW_CHAN_WIDTH_80P80,
    SKW_CHAN_WIDTH_160,

    SKW_CHAN_WIDTH_MAX,
};
struct skw_iface
{
    u8 id;
    u8 lmac_id;
    u8 addr[ETH_ALEN];
    struct skw_core *skw;
    struct netdev *ndev;//struct netdev
    uint16_t default_multicast;
    uint16_t peer_map;
    uint16_t actived_ctx;
    uint16_t net_flags; /* reference SKW_IFACE_FLAG_ */
    void (*iface_event_handler)(struct skw_core *skw, struct skw_iface *iface, struct skw_msg *msg_hdr, void *data, size_t data_len);
};
enum
{
    IEEE80211_HT_PARAM_CHA_SEC_NONE,
    IEEE80211_HT_PARAM_CHA_SEC_ABOVE,
    IEEE80211_HT_PARAM_CHA_SEC_BELOW = 3,

};
struct skw_join_resp
{
    u8 peer_idx;
    u8 lmac_id;
    u8 inst;
    u8 multicast_idx;
};
// struct skw_mgmt_hdr {
//  u8 chan;
//  u8 band;
//  s16 signal;
//  u16 mgmt_len;
//  u16 resv;
//  struct wlan_802_11_header mgmt[0];
// } ;
struct ieee80211_ht_cap
{
    le16 cap_info;           // HT能力信息字段（bit位定义见标准）
    u8 ampdu_params_info;      // A-MPDU参数
    u8 supp_mcs_set[16];       // 支持的MCS集合
    le16 extended_ht_cap_info; // 扩展HT能力
    u32 tx_BF_cap_info;        // 波束赋形能力
    u8 antenna_selection_info; // 天线选择能力
} __attribute__((packed)) ; //len= 26
struct ieee80211_vht_mcs_info
{
    le16 rx_mcs_map;     // 支持的Rx MCS映射
    le16 rx_highest;     // 支持的最高Rx数据速率
    le16 tx_mcs_map;     // 支持的Tx MCS映射
    le16 tx_highest;     // 支持的最高Tx数据速率
} __attribute__((packed)) ;
struct ieee80211_vht_cap
{
    u32 vht_cap_info;           // VHT能力信息字段（bit位定义，见802.11ac标准）
    struct ieee80211_vht_mcs_info supp_mcs; // 支持的MCS速率集合
} __attribute__((packed)) ; // len = 12
struct skw_assoc_req_param
{
    struct ieee80211_ht_cap ht_capa;
    struct ieee80211_vht_cap  vht_capa;
    u8 bssid[6];
    u8 pre_bssid[6];
    u16 req_ie_offset;
    u16 req_ie_len;
    u8  req_ie[];
} __attribute__((packed)) ;
struct ethhdr
{
    unsigned char   h_dest[6];   // 目的MAC地址（6字节）
    unsigned char   h_source[6]; // 源MAC地址（6字节）
    u16          h_proto;            // 上层协议类型（2字节，网络字节序）
};//len = 14
struct skw_tx_desc_conf
{
    u16 l4_hdr_offset: 10;
    u16 csum: 1;

    /* ip_prot: 0: UDP, 1: TCP */
    u16 ip_prot: 1;
    u16 rsv: 4;
} __attribute__((packed));//len = 2

// 802.3帧头部
struct skw_tx_desc_hdr
{
    /* pading bytes for gap */
    u16 padding_gap: 2;
    u16 inst: 2;
    u16 tid: 4;
    u16 peer_lut: 5;

    /* frame_type:
     * 0: ethernet frame
     * 1: 80211 frame
     */
    u16 frame_type: 1;
    u16 encry_dis: 1;

    /* rate: 0: auto, 1: sw config */
    u16 rate: 1;

    u16 msdu_len: 12;
    u16 lmac_id: 2;
    u16 rsv: 2;
    u16 eth_type;

    /* pading for address align */
    u8 gap[0];
} __attribute__((packed)) ;//len = 6
struct skw_rx_desc
{
    /* word 13 */
    u16 eosp: 1;
    u16 more_data: 1;
    u16 pm: 1;
    u16 retry_frame: 1; //重传帧
    u16 is_eof: 1; //mpdu_eof_flag
    u16 ba_session: 1;
    u16 resv1: 1;
    u16 resv: 1;
    u16 cipher: 4;
    u16 snap_type: 1;
    u16 vlan: 1;
    u16 eapol: 1;
    u16 rcv_in_ps_mode: 1;
    u16 msdu_len;//2 //

    /* word 14 */
    u8 csum_valid: 1; //是否校验和有效
    u8 is_ampdu: 1;
    u8 snap_match: 1; //0 没有加 type头  需要判断数据类型后加上type
    u8 is_amsdu: 1;
    u8 is_qos_data: 1;
    u8 amsdu_first_idx: 1; //有多个msdu
    u8 amsdu_last_idx: 1; //
    u8 mpdu_sniff: 1; //1
    u16 csum;//2  //数据的校验和
    u8 msdu_filter;//1 //特殊帧  方便调试使用
    /* word 15 */
    u16 sn: 12; /* seq number  data 序列号*/ //有乱序的情况  需要根据该参数进行排序
    u16 frag_num: 4; //分片的数量  有个别的路由器会分片发送数据包
    u16 inst_id: 2; //mpdu_ra_index  网络设备
    u16 inst_id_valid: 1;
    u16 more_frag: 1; //与frag_num 配合使用
    u16 peer_idx: 5;
    u16 peer_idx_valid: 1;
    u16 is_mc_addr: 1; //bc_mc_flag  是否为多播地址
    u16 first_msdu_in_buff: 1;
    u16 tid: 4; //优先级
    /* word 16 & word17*/
    u8 pn[6];   //u16 msdu_len; //与网络安全相关 值是递增   如果当前是比之前小，说明当前网络不安全
    u8 msdu_offset;//1
    u8 amsdu_idx: 6; //有多个msdu时候 的当前msdu的索引
    u16 need_forward: 1; //da ra  不相同的包 需要转发
    u16 mac_drop_frag: 1; //

    //u16 rsv; //保留位
    //1
};
struct skw_key_params
{
    u8 mac_addr[ETH_ALEN];
    u8 key_type;
    u8 cipher_type;
    u8 pn[6];
    u8 key_id;
    u8 key_len;
    u8 key[WLAN_MAX_KEY_LEN];
} __attribute__((packed));
struct skw_setip_param
{
    u8 ip_type;
    uint32_t ipv4;
    //u8 ipv6[16];
} __attribute__((packed));
enum SKW_IP_VERSION
{
    SKW_IP_IPV4 = 0,
    SKW_IP_IPV6,
};
#define ETH_P_802_3_MIN 0x0600
enum SKW_MSG_TYPE
{
    SKW_MSG_CMD,
    SKW_MSG_CMD_ACK,
    SKW_MSG_EVENT,
    SKW_MSG_EVENT_LOCAL
};
enum SKW_KEY_TYPE
{
    SKW_KEY_TYPE_PTK = 0,
    SKW_KEY_TYPE_GTK = 1,
    SKW_KEY_TYPE_IGTK = 2,
    SKW_KEY_TYPE_BIGTK = 3,
};
struct skw_ba_action
{
    u8 action;
    u8 lmac_id;
    u8 peer_idx;
    u8 tid;
    u8 status_code;
    u8 resv[3];
    u16 ssn;
    u16 win_size;
} __attribute__((packed));

enum SKW_CIPHER_TYPE
{
    SKW_CIPHER_TYPE_INVALID = 0,
    SKW_CIPHER_TYPE_WEP40 = 1,
    SKW_CIPHER_TYPE_WEP104 = 2,
    SKW_CIPHER_TYPE_TKIP = 3,
    SKW_CIPHER_TYPE_SMS4 = 4,
    SKW_CIPHER_TYPE_CCMP = 8,
    SKW_CIPHER_TYPE_CCMP_256 = 9,
    SKW_CIPHER_TYPE_GCMP = 10,
    SKW_CIPHER_TYPE_GCMP_256 = 11,
    SKW_CIPHER_TYPE_AES_CMAC = 12, /* BIP_CMAC_128 */
    SKW_CIPHER_TYPE_BIP_CMAC_256 = 13,
    SKW_CIPHER_TYPE_BIP_GMAC_128 = 14,
    SKW_CIPHER_TYPE_BIP_GMAC_256 = 15,
};
enum SKW_CMD_ID
{
    SKW_CMD_DOWNLOAD_INI = 0,
    SKW_CMD_GET_INFO = 1,
    SKW_CMD_SYN_VERSION = 2,
    SKW_CMD_OPEN_DEV = 3,
    SKW_CMD_CLOSE_DEV = 4,
    SKW_CMD_START_SCAN = 5,
    SKW_CMD_STOP_SCAN = 6,
    SKW_CMD_START_SCHED_SCAN = 7,
    SKW_CMD_STOP_SCHED_SCAN = 8,
    SKW_CMD_JOIN = 9,
    SKW_CMD_AUTH = 10,
    SKW_CMD_ASSOC = 11,
    SKW_CMD_ADD_KEY = 12,
    SKW_CMD_DEL_KEY = 13,
    SKW_CMD_TX_MGMT = 14,
    SKW_CMD_TX_DATA_FRAME = 15,
    SKW_CMD_SET_IP = 16,
    SKW_CMD_DISCONNECT = 17,
    SKW_CMD_RPM_REQ = 18,
    SKW_CMD_START_AP = 19,
    SKW_CMD_STOP_AP = 20,
    SKW_CMD_ADD_STA = 21,
    SKW_CMD_DEL_STA = 22,
    SKW_CMD_GET_STA = 23,
    SKW_CMD_RANDOM_MAC = 24,
    SKW_CMD_GET_LLSTAT = 25,
    SKW_CMD_SET_MC_ADDR = 26,
    SKW_CMD_RESUME = 27,
    SKW_CMD_SUSPEND = 28,
    SKW_CMD_REMAIN_ON_CHANNEL = 29,
    SKW_CMD_BA_ACTION = 30,
    SKW_CMD_TDLS_MGMT = 31,
    SKW_CMD_TDLS_OPER = 32,
    SKW_CMD_TDLS_CHANNEL_SWITCH = 33,
    SKW_CMD_SET_CQM_RSSI = 34,
    SKW_CMD_NPI_MSG = 35,
    SKW_CMD_IBSS_JOIN = 36,
    SKW_CMD_SET_IBSS_ATTR = 37,
    SKW_CMD_RSSI_MONITOR = 38,
    SKW_CMD_SET_IE = 39,
    SKW_CMD_SET_MIB = 40,
    SKW_CMD_REGISTER_FRAME = 41,
    SKW_CMD_ADD_TX_TS = 42,
    SKW_CMD_DEL_TX_TS = 43,
    SKW_CMD_REQ_CHAN_SWITCH = 44,
    SKW_CMD_CHANGE_BEACON = 45,
    SKW_CMD_DPD_ILC_GEAR_PARAM = 46,
    SKW_CMD_DPD_ILC_MARTIX_PARAM = 47,
    SKW_CMD_DPD_ILC_COEFF_PARAM = 48,
    SKW_CMD_WIFI_RECOVER = 49,
    SKW_CMD_PHY_BB_CFG = 50,
    SKW_CMD_SET_REGD = 51,
    SKW_CMD_SET_EFUSE = 52,
    SKW_CMD_SET_PROBEREQ_FILTER = 53,
    SKW_CMD_CFG_ANT = 54,
    SKW_CMD_RTT = 55,
    SKW_CMD_GSCAN = 56,
    SKW_CMD_DFS = 57,
    SKW_CMD_SET_SPD_ACTION = 58,
    SKW_CMD_SET_DPD_RESULT = 59,
    SKW_CMD_SET_MONITOR_PARAM = 60,
    SKW_CMD_GET_DEBUG_INFO = 61,

    SKW_CMD_NUM,
};

enum SKW_EVENT_ID
{
    SKW_EVENT_NORMAL_SCAN_CMPL = 0,
    SKW_EVENT_SCHED_SCAN_CMPL = 1,
    SKW_EVENT_DISCONNECT = 2,
    SKW_EVENT_ASOCC = 3,
    SKW_EVNET_RX_MGMT = 4,
    SKW_EVENT_DEAUTH = 5,
    SKW_EVENT_DISASOC = 6,
    SKW_EVENT_JOIN_CMPL = 7,
    SKW_EVENT_ACS_REPORT = 8,
    SKW_EVENT_DEL_STA = 9,

    SKW_EVENT_RRM_REPORT = 10,
    SKW_EVENT_SCAN_REPORT  = 11,
    SKW_EVENT_MGMT_TX_STATUS = 12,
    SKW_EVENT_BA_ACTION = 13,
    SKW_EVENT_CANCEL_ROC = 14,
    SKW_EVENT_TDLS = 15,
    SKW_EVENT_CREDIT_UPDATE = 16,////告诉上层需要传输的限制pack个数  数据大小是完整的802.3的大小 一般是1536
    SKW_EVENT_MIC_FAILURE = 17,
    SKW_EVENT_THERMAL_WARN = 18,
    SKW_EVENT_RSSI_MONITOR = 19,

    SKW_EVENT_CQM = 20,
    SKW_EVENT_RX_UNPROTECT_FRAME = 21,
    SKW_EVENT_CHAN_SWITCH = 22,
    SKW_EVENT_CHN_SCH_DONE = 23,
    SKW_EVENT_DOWNLOAD_FW = 24,
    SKW_EVENT_TX_FRAME = 25,
    SKW_EVENT_NPI_MP_MODE = 26,
    SKW_EVENT_DPD_ILC_COEFF_REPORT = 27,
    SKW_EVENT_DPD_ILC_GEAR_CMPL = 28,
    SKW_EVENT_FW_RECOVERY = 29,

    SKW_EVENT_TDLS_CHAN_SWITCH_RESULT = 30,
    SKW_EVENT_THM_FW_STATE = 31,
    SKW_EVENT_ENTER_ROC = 32,
    SKW_EVENT_RADAR_PULSE = 33,
    SKW_EVENT_RTT = 34,
    SKW_EVENT_GSCAN = 35,
    SKW_EVENT_GSCAN_FRAME = 36,
    SKW_EVENT_DPD_RESULT = 37,

    SKW_EVENT_MAX
};
#define ETH_P_IP      0x0800  // IPv4 协议
#define ETH_P_ARP     0x0806  // ARP 地址解析协议
#define ETH_P_IPV6    0x86DD  // IPv6 协议
#define ETH_P_802_1Q  0x8100  // VLAN 标记帧
#define ETH_P_PAE     0x888E  // EAP over LAN (802.1X)
#define ETH_P_TDLS    0x890D  // TDLS
#define ETH_P_802_2   0x0004  // 802.2
#define ETH_P_802_3   0x0001  // 802.3
#define ETH_P_WAI     0x88B4  // WAPI
struct skw_iface *skw_get_skw_iface(void);
struct skw_core *skw_get_skw_core(void);
struct skw_peer *skw_peer_alloc(void);
void skw_join_resp_handler(struct skw_core *skw, struct skw_iface *iface, struct skw_join_resp *resp);
struct skw_peer_ctx *skw_get_ctx(struct skw_core *skw, u8 lmac_id, u8 idx);
void skw_peer_init(struct skw_peer *peer, const u8 *addr, int idx);
void skw_peer_free(struct skw_peer *peer);
int skw_peer_ctx_bind(struct skw_iface *iface, struct skw_peer_ctx *ctx, struct skw_peer *peer);
int skw_wifi_stop_scan(void);
int skw_disconnected(void);
int wifi_setup_mac_address(uint8_t *mac);
void skw_wifi_set_ipaddr(char *ip);
int skw_sdio_io_init(void);
void wifi_info_init(void);
#endif