/* SPDX-License-Identifier: GPL-2.0 */

/******************************************************************************
 *
 * Copyright (C) 2020 SeekWave Technology Co.,Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 ******************************************************************************/

#ifndef __SKW_IFACE_H__
#define __SKW_IFACE_H__

#include <mlan_ieee.h>
//#include <ieee802_1x_kay.h>
//#include "skw_sdio_core.h"
//#include "skw_sdio.h"
#include "skw_type_msg.h"
/* SKW_LMAC_FLAG_* */

#define SKW_LMAC_FLAG_INIT                  BIT(0)
#define SKW_LMAC_FLAG_ACTIVED               BIT(1)
#define SKW_LMAC_FLAG_RXCB                  BIT(2)
#define SKW_LMAC_FLAG_TXCB                  BIT(3)





#define SKW_MAX_DEFRAG_ENTRY            4

#define SKW_SET(d, v)                ((d) |= (v))
/* enable 80211W */
#define SKW_NUM_DEFAULT_KEY             4
#define SKW_NUM_DEFAULT_MGMT_KEY        2



#define SKW_INVALID_ID                  0xff
#define SKW_PEER_ALIGN                  32

#define SKW_PEER_FLAG_TAINT             BIT(0)
#define SKW_PEER_FLAG_BAD_ID            BIT(1)
#define SKW_PEER_FLAG_BAD_ADDR          BIT(2)
#define SKW_PEER_FLAG_ACTIVE            BIT(3)
#define SKW_PEER_FLAG_DEAUTHED          BIT(4)

#define SKW_IFACE_FLAG_LEGACY_P2P_DEV   BIT(0)
#define SKW_IFACE_FLAG_DEAUTH           BIT(1)

#define SKW_IFACE_STA_ROAM_FLAG_CQM_LOW     BIT(0)
/**
 * enum SKW_STATES - STA state
 *
 * @SKW_STATE_NONE: STA exists without special state
 * @SKW_STATE_AUTHING: STA is trying to authentiacate with a BSS
 * @SKW_STATE_AUTHED: STA is authenticated
 * @SKW_STATE_ASSOCING: STA is trying to assoc with a BSS
 * @SKW_STATE_ASSOCED: STA is associated
 * @SKW_STATE_COMPLETED, STA connection is compeleted
 */


enum SKW_WMM_AC
{
    SKW_WMM_AC_VO = 0,
    SKW_WMM_AC_VI,
    SKW_WMM_AC_BE,
    SKW_WMM_AC_BK,
    SKW_WMM_AC_MAX,
};

#define SKW_ACK_TXQ                      SKW_WMM_AC_MAX

#define SKW_FRAG_STATUS_ACTIVE           BIT(0)
#define SKW_FRAG_STATUS_CHK_PN           BIT(1)

enum skw_wireless_mode
{
    SKW_WIRELESS_11B = 1,
    SKW_WIRELESS_11G,
    SKW_WIRELESS_11A,
    SKW_WIRELESS_11N,
    SKW_WIRELESS_11AC,
    SKW_WIRELESS_11AX,
    SKW_WIRELESS_11G_ONLY,
    SKW_WIRELESS_11N_ONLY,
};

// enum interface_mode {
//  SKW_NONE_MODE = 0,
//  SKW_STA_MODE = 1,
//  SKW_AP_MODE = 2,
//  SKW_GC_MODE = 3,
//  SKW_GO_MODE = 4,
//  SKW_P2P_DEV_MODE = 5,
//  SKW_IBSS_MODE = 6,
//  SKW_MONITOR_MODE = 7,

//  MAX_MODE_TYPE,
// };

// enum SKW_CHAN_BW_INFO {
//  SKW_CHAN_WIDTH_20,
//  SKW_CHAN_WIDTH_40,
//  SKW_CHAN_WIDTH_80,
//  SKW_CHAN_WIDTH_80P80,
//  SKW_CHAN_WIDTH_160,

//  SKW_CHAN_WIDTH_MAX,
// };

enum skw_rate_info_flags
{
    SKW_RATE_INFO_FLAGS_LEGACY,
    SKW_RATE_INFO_FLAGS_HT,
    SKW_RATE_INFO_FLAGS_VHT,
    SKW_RATE_INFO_FLAGS_HE,
};

#define SKW_OPEN_FLAG_OFFCHAN_TX         BIT(0)
// struct skw_open_dev_param {
//  u16 mode;
//  u16 flags; /* reference SKW_OPEN_FLAG_ */
//  u8 mac_addr[6];
// } ;

struct skw_frag_entry
{
    u8 id;
    u8 status; /* reference SKW_FRAG_STATUS */
    u16 pending_len;
    u8 tid;
    u8 frag_num;
    u16 sn;
    unsigned long start;
    //struct sk_buff_head skb_list;

    /* PN of the last fragment if CCMP was used */
    u8 last_pn[IEEE80211_CCMP_PN_LEN];
};



#define SKW_KEY_FLAG_WEP_SHARE        BIT(0)
#define SKW_KEY_FLAG_WEP_UNICAST      BIT(1)
#define SKW_KEY_FLAG_WEP_MULTICAST    BIT(2)







#define SKW_SM_FLAG_SAE_RX_CONFIRM     BIT(0)



enum skw_msdu_filter
{
    SKW_MSDU_FILTER_SUCCESS,
    SKW_MSDU_FILTER_SNAP_MISMATCH,
    SKW_MSDU_FILTER_ARP,
    SKW_MSDU_FILTER_VLAN,
    SKW_MSDU_FILTER_WAPI,
    SKW_MSDU_FILTER_EAP = 5,
    SKW_MSDU_FILTER_PPPOE,
    SKW_MSDU_FILTER_TDLS,
    SKW_MSDU_FILTER_DHCP = 11,
    SKW_MSDU_FILTER_DHCPV6 = 12,
};

#define SKW_RX_FILTER_NONE      0
#define SKW_RX_FILTER_SET       (BIT(SKW_MSDU_FILTER_EAP) | BIT(SKW_MSDU_FILTER_WAPI))

#define SKW_RX_FILTER_EXCL      (BIT(SKW_MSDU_FILTER_EAP) |  \
                 BIT(SKW_MSDU_FILTER_WAPI) | \
                 BIT(SKW_MSDU_FILTER_ARP) |  \
                 BIT(SKW_MSDU_FILTER_DHCP) | \
                 BIT(SKW_MSDU_FILTER_DHCPV6))

#define SKW_RX_FILTER_DBG       (BIT(SKW_MSDU_FILTER_EAP) |  \
                 BIT(SKW_MSDU_FILTER_WAPI) | \
                 BIT(SKW_MSDU_FILTER_ARP) |  \
                 BIT(SKW_MSDU_FILTER_DHCP) | \
                 BIT(SKW_MSDU_FILTER_DHCPV6))

enum SKW_RX_MPDU_DESC_PPDUMODE
{
    SKW_PPDUMODE_11B_SHORT = 0,
    SKW_PPDUMODE_11B_LONG,
    SKW_PPDUMODE_11G,
    SKW_PPDUMODE_HT_MIXED,
    SKW_PPDUMODE_VHT_SU,
    SKW_PPDUMODE_VHT_MU,
    SKW_PPDUMODE_HE_SU,
    SKW_PPDUMODE_HE_TB,
    SKW_PPDUMODE_HE_ER_SU,
    SKW_PPDUMODE_HE_MU,
};

#define NL80211_MAX_NR_CIPHER_SUITES 5
#define NL80211_MAX_NR_AKM_SUITES 2

struct cfg80211_crypto_settings
{
    u32 wpa_versions;           // WPA协议版本（如WPA/WPA2/WPA3），位掩码
    u32 cipher_group;           // 组播/广播加密算法
    int n_ciphers_pairwise;     // 单播加密算法数量
    u32 ciphers_pairwise[NL80211_MAX_NR_CIPHER_SUITES]; // 单播加密算法数组
    int n_akm_suites;           // AKM（密钥管理）套件数量
    u32 akm_suites[NL80211_MAX_NR_AKM_SUITES]; // AKM套件数组
    bool control_port;          // 是否使用控制端口（如EAPOL数据是否由驱动处理）
    bool control_port_no_encrypt; // 控制端口数据是否不加密
    u32 control_port_ethertype; // 控制端口以太网类型（如EAPOL）
    bool control_port_over_nl80211; // 控制端口数据是否通过nl80211传递
    u8 *psk;                    // 预共享密钥（PSK），指针
    const u8 *sae_pwd;          // SAE密码（WPA3/SAE专用）
    int sae_pwd_len;            // SAE密码长度
    u8 *wep_keys[4];            // WEP密钥指针数组
    u8 wep_tx_key;              // WEP默认发送密钥索引
    u8 n_wep_keys;              // WEP密钥数量
};
enum nl80211_band
{
    NL80211_BAND_2GHZ = 0,   // 2.4GHz 频段
    NL80211_BAND_5GHZ = 1,   // 5GHz 频段
    NL80211_BAND_6GHZ = 2,   // 6GHz 频段（Wi-Fi 6E）
    NL80211_BAND_S1GHZ = 3,  // sub-1GHz 频段（如 900MHz）
    NL80211_BAND_MAX
};
struct ieee80211_channel
{
    enum nl80211_band band;    // 频段（2.4G/5G/6G）
    u16 center_freq;           // 中心频率（MHz）
    u16 hw_value;              // 硬件信道号（如1、6、36等）
    u32 flags;                 // 信道属性标志（如禁用、DFS等）
    int max_antenna_gain;      // 最大天线增益（dBi）
    int max_power;             // 最大发射功率（dBm）
    int max_reg_power;         // 法规限制的最大发射功率（dBm）
    bool beacon_found;         // 是否发现信标帧
    u32 orig_flags;            // 原始flags（用于恢复）
    int orig_mag;              // 原始最大天线增益
    int orig_mpwr;             // 原始最大发射功率
};

struct skw_bss_cfg
{
    u8 ssid[32];
    u8 ssid_len;
    u8 ctx_idx;
    u8 bssid[ETH_ALEN];

    enum nl80211_auth_type auth_type;
    enum SKW_CHAN_BW_INFO  width;
    enum SKW_CHAN_BW_INFO  ht_cap_chwidth;
    struct cfg80211_crypto_settings crypto;

    struct ieee80211_channel *channel;
    struct ieee80211_ht_cap *ht_cap;
    struct ieee80211_vht_cap *vht_cap;
};

struct skw_survey_data
{
    u32 time;
    u32 time_busy;
    u32 time_ext_busy;
    u8 chan;
    u8 band;
    s8 noise;
    u8 resv;
} ;
struct list_head
{
    struct list_head *next;
    struct list_head *prev;
};
struct skw_survey_info
{
    struct list_head list;
    struct skw_survey_data data;
};

struct skw_ac_param
{
    u8 aifsn: 4;
    u8 acm: 1;
    u8 aci: 2;
    u8 recv: 1;
    u8 ecw;
    u16 txop_limit;
} ;

struct skw_wmm
{
    u8 id;
    u8 len;
    u8 oui[3];
    u8 type;
    u8 sub_type;
    u8 version;
    u8 qos;
    u8 resv;
    struct skw_ac_param ac[SKW_WMM_AC_MAX];
} ;

struct skw_list
{
    int count;
    spinlock_t lock;
    struct list_head list;
};



struct skw_iftype_ext_cap
{
    u8 iftype;
    u8 ext_cap[10];
    u8 ext_cap_len;
};

struct skw_ctx_pending
{
    unsigned long start;
    u8 *cmd;
    int cmd_len;
    int retry;
};

struct skw_sta_core
{
    struct rt_mutex lock;
    //struct timer_list timer;
    struct skw_ctx_pending pending;

    struct skw_sm sm;
    struct skw_bss_cfg bss;

    unsigned long auth_start;

    u8 *assoc_req_ie;
    u32 assoc_req_ie_len;

    struct cfg80211_bss *cbss;
};

struct skw_wmm_info
{
    u8 acm;
    bool qos_enabled;
    s8 factor[SKW_WMM_AC_MAX];
    struct skw_ac_param ac[SKW_WMM_AC_MAX];
};

#define SKW_AID_DWORD BITS_TO_LONGS(64)

struct skw_monitor_dbg_iface
{
    u8 addr[ETH_ALEN];
    struct net_device *ndev;
    u32 frame_cnt;
};

bool skw_acl_allowed(struct skw_iface *iface, u8 *addr);

static inline const char *skw_state_name(enum SKW_STATES state)
{
    static const char *const st_name[] = {"NONE", "AUTHING", "AUTHED",
                                          "ASSOCING", "ASSOCED", "COMPLETED"
                                         };

    if (state >= ARRAY_SIZE(st_name))
        return "unknown";

    return st_name[state];
}

static inline void skw_list_init(struct skw_list *list)
{
    //spin_lock_init(&list->lock);
    //INIT_LIST_HEAD(&list->list);
    list->count = 0;
}

static inline void skw_list_add(struct skw_list *list, struct list_head *entry)
{
    //spin_lock_bh(&list->lock);
    //list_add_tail(entry, &list->list);
    list->count++;
    //spin_unlock_bh(&list->lock);
}

static inline void skw_list_del(struct skw_list *list, struct list_head *entry)
{
    //spin_lock_bh(&list->lock);
    //list_del(entry);
    list->count--;
    //spin_unlock_bh(&list->lock);
}
#include "rtthread.h"
//#define __ALIGN_MASK_SKW(x, mask)    (((x) + (mask)) & ~(mask))
#define ALIGN_SKW(x, a)        __ALIGN_MASK_SKW(x, (typeof(x))(a) - 1)

static inline void *skw_ctx_entry(const struct skw_peer *peer)
{
    return (char *)peer + sizeof(struct skw_peer);//+ ALIGN_SKW(sizeof(struct skw_peer),SKW_PEER_ALIGN);
}
#if 0
static inline bool is_skw_ap_mode(struct skw_iface *iface)
{
    return iface->wdev.iftype == NL80211_IFTYPE_AP ||
           iface->wdev.iftype == NL80211_IFTYPE_P2P_GO;
}

static inline bool is_skw_sta_mode(struct skw_iface *iface)
{
    return iface->wdev.iftype == NL80211_IFTYPE_STATION ||
           iface->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT;
}
#endif
static inline void skw_peer_ctx_lock(struct skw_peer_ctx *ctx)
{
    //if (WARN_ON(!ctx))
    //  return;

    //mutex_lock(&ctx->lock);
}

static inline void skw_peer_ctx_unlock(struct skw_peer_ctx *ctx)
{
    // if (WARN_ON(!ctx))
    //  return;

    // mutex_unlock(&ctx->lock);
}
#if 0
static inline void skw_sta_lock(struct skw_sta_core *core)
{
    mutex_lock(&core->lock);
}

static inline void skw_sta_unlock(struct skw_sta_core *core)
{
    mutex_unlock(&core->lock);
}
#endif
static inline void skw_sta_assert_lock(struct skw_sta_core *core)
{
    //lockdep_assert_held(&core->lock);
}
#if 0
static inline void skw_wdev_lock(struct wireless_dev *wdev)
//__acquires(wdev)
{
    //mutex_lock(&wdev->mtx);
    //__acquire(wdev->mtx);
}

static inline void skw_wdev_unlock(struct wireless_dev *wdev)
//__releases(wdev)
{
    //__release(wdev->mtx);
    //mutex_unlock(&wdev->mtx);
}
#endif

static inline void skw_wdev_assert_lock(struct skw_iface *iface)
{
    //lockdep_assert_held(&iface->wdev.mtx);
}
#include "wlan_dev.h"
#if 0
struct skw_iface *skw_add_iface(struct rt_wlan_device *wiphy, const char *name,
                                enum nl80211_iftype iftype, u8 *mac,
                                u8 id, bool need_ndev);
int skw_del_iface(struct rt_wlan_device *wiphy, struct skw_iface *iface);


int skw_iface_setup(struct rt_wlan_device *wiphy, struct net_device *dev,
                    struct skw_iface *iface, const u8 *addr,
                    enum nl80211_iftype iftype, int id);

int skw_iface_teardown(struct rt_wlan_device *wiphy, struct skw_iface *iface);

int skw_cmd_open_dev(struct rt_wlan_device *wiphy, int inst, const u8 *mac_addr,
                     enum nl80211_iftype type, u16 flags);
void skw_purge_survey_data(struct skw_iface *iface);
void skw_ap_check_sta_throughput(void *data);
void skw_set_sta_timer(struct skw_sta_core *core, unsigned long timeout);
void skw_iface_set_wmm_capa(struct skw_iface *iface, const u8 *ies,
                            size_t ies_len);

struct skw_peer *skw_peer_alloc(void);
void skw_peer_init(struct skw_peer *peer, const u8 *addr, int idx);
struct skw_peer_ctx *skw_peer_ctx(struct skw_iface *iface, const u8 *mac);
void skw_peer_ctx_transmit(struct skw_peer_ctx *ctx, bool enable);
void __skw_peer_ctx_transmit(struct skw_peer_ctx *ctx, bool enable);
int skw_peer_ctx_bind(struct skw_iface *iface, struct skw_peer_ctx *ctx,
                      struct skw_peer *peer);
int __skw_peer_ctx_bind(struct skw_iface *iface, struct skw_peer_ctx *ctx,
                        struct skw_peer *peer);
void skw_peer_free(struct skw_peer *peer);
void skw_purge_key_conf(struct skw_key_conf *conf);
#endif
#endif
