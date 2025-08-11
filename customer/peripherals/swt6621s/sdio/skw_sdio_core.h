#ifndef __SKW_SDIO_CORE_H__
#define __SKW_SDIO_CORE_H__
#include "rtthread.h"

#include "skw_sdio.h"
#include "skw_boot.h"
#include "skw_type_msg.h"
#include "mlan_ieee.h"

#define SKW_MAX_MSG_ID        256
#define SKW_MAX_LMAC_SUPPORT                2
#define ETH_ALEN    6

#define SKW_SCAN_FLAG_RND_MAC         BIT(0)
#define SKW_SCAN_FLAG_ACS             BIT(1)
#define SKW_SCAN_FLAG_PASSIVE         BIT(7)

#ifndef s16
    #define s16 int16_t
#endif
#define SKW_HW_FLAG_EXTRA_HDR            BIT(0)
#define SKW_HW_FLAG_SDIO_V2              BIT(1)
#define SKW_NR_IFACE                        4


struct skw_version_info
{
    u8 cmd[SKW_MAX_MSG_ID];
    u8 event[SKW_MAX_MSG_ID];
};
enum SKW_CMD_DISCONNECT_TYPE_E
{
    SKW_DISCONNECT_ONLY = 0,
    SKW_DISCONNECT_SEND_DISASSOC = 1,
    SKW_DISCONNECT_SEND_DEAUTH = 2,
};
struct skw_hw_extra
{
    u32 len: 16;
    u32 pad: 7;
    u32 eof: 1;
    u32 channel: 8;
};//len = 4

struct skw_fixed_offset
{
    s16 hdr_offset;
    s16 msdu_offset;
};
//#include "skw_iface.h"
struct skw_lmac
{
    u8 id;
    u8 flags; /* reference SKW_LMAC_FLAG_ */
    s8 lport; /* logic port */
    s8 dport; /* data port */

    int iface_bitmap;
    struct skw_peer_ctx peer_ctx[SKW_MAX_PEER_SUPPORT];
    atomic_t fw_credit;

};
struct skw_hw_info
{
    u8 bus;
    u8 dma;
    u8 nr_lmac;
    u8 cmd_port;

    u16 align;
    s16 pkt_limit;
    u32 flags;
    atomic_t credit; /* total credit of all LMAC */

    struct skw_hw_extra extra;
    struct skw_fixed_offset rx_desc;
    struct skw_lmac lmac[SKW_MAX_LMAC_SUPPORT];

};
struct skw_scan_param
{
    u16 flags;  /* reference SKW_SCAN_FLAG_ */
    u8 rand_mac[6];
    u32 nr_chan;
    u32 chan_offset;
    u32 n_ssid;
    u32 ssid_offset;
    u32 ie_len;
    u32 ie_offset;
    u8 ie[];
};
struct skw_disconnect_param
{
    u8 type;
    u8 local_state_change;
    u16 reason_code;
    u16 ie_offset;
    u16 ie_len;
    u8 ie[];
} __packed;
struct skw_cmd_data
{
    struct skw_hw_extra extra;
    struct skw_msg msg_hdr;//len = 8
    uint8_t *data_t;
};
#if 1
struct mac_address_wifi
{
    rt_uint8_t addr[6];
};
#endif
struct skw_open_dev_param
{
    u16 mode;
    u16 flags; /* reference SKW_OPEN_FLAG_ */
    u8 mac_addr[6];
};
enum interface_mode
{
    SKW_NONE_MODE = 0,
    SKW_STA_MODE = 1,
    SKW_AP_MODE = 2,
    SKW_GC_MODE = 3,
    SKW_GO_MODE = 4,
    SKW_P2P_DEV_MODE = 5,
    SKW_IBSS_MODE = 6,
    SKW_MONITOR_MODE = 7,

    MAX_MODE_TYPE,
};
//   typedef struct {int no; } spinlock_t;
struct skw_vif
{
    u16 bitmap;
    u16 opened_dev;
    spinlock_t lock;
    struct skw_iface *iface[SKW_NR_IFACE];
};
/** Scan channel list */


struct skw_core
{
    uint8_t id_join_resp;
    struct skw_join_resp skw_join_resp[10];
    struct skw_hw_info hw;
    struct skw_vif vif;
    u16 nr_scan_results;
    struct cfg80211_scan_request *scan_req;
    void (*skw_event_handler)(struct skw_core *skw, struct skw_iface *iface, struct skw_msg *msg_hdr, void *data, size_t data_len);
    struct
    {
        struct rt_mutex lock;
        //            wait_queue_head_t wq;
        //            struct wakeup_source *ws;

        unsigned long start_time;
        void (*callback)(struct skw_core *skw);

        unsigned long flags; /* reference SKW_CMD_FLAG_ */

        char name[32];
        void *data;
        void *arg;

        u16 data_len;
        u16 arg_size;

        u16 seq;
        u16 status;

        int id;
    } cmd;
    struct mac_address_wifi address[SKW_NR_IFACE];

};
struct skw_chip_info
{
    u16 enc_capa;

    u32 chip_model;
    u32 chip_version;
    u32 fw_version;
    u32 fw_capa;

    u8 max_sta_allowed;
    u8 max_mc_addr_allowed;

    /* HT */
    u16 ht_capa;
    u16 ht_ext_capa;
    u16 ht_ampdu_param;
    u32 ht_tx_mcs_maps;
    u32 ht_rx_mcs_maps;

    /* VHT */
    u32 vht_capa;
    u16 vht_tx_mcs_maps;
    u16 vht_rx_mcs_maps;

    /* HE */
    u8 max_scan_ssids;
    u8 he_capa[6];
    u8 he_phy_capa[11];
    u16 he_tx_mcs_maps;
    u16 he_rx_mcs_maps;
    u8 mac[ETH_ALEN];

    u8 abg_rate_num;
    u8 abg_rate[15];

    u32 fw_bw_capa; /* reference SKW_BW_CAP_* */

    u32 priv_filter_arp: 1;
    u32 priv_ignore_cred: 1;
    u32 priv_pn_reuse: 1;
    u32 priv_p2p_common_port: 1;
    u32 priv_dfs_master_enabled: 1;
    u32 priv_2g_only: 1;
    u32 priv_resv: 18;
    u32 nr_hw_mac: 8;

    u8 fw_build_time[32];
    u8 fw_plat_ver[16];
    u8 fw_wifi_ver[16];
    u8 fw_bt_ver[16];

    u32 fw_timestamp;
    u32 fw_chip_type;
    u16 calib_module_id;
    u16 resv;
    u32 fw_ext_capa[12];
} ;

static inline struct skw_iface *to_skw_iface(struct skw_core *skw, int id)
{
    if (!skw || id & 0xfffffffc)
        return NULL;

    return skw->vif.iface[id];
}


static inline struct skw_iface *skw_get_iface(struct skw_core *skw, int id)
{
    if (id < 0 || id >= SKW_NR_IFACE)
        return NULL;

    return skw->vif.iface[id];
}

#endif
