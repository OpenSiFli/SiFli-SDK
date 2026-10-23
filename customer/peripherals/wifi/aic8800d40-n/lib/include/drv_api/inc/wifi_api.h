/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WIFI_API_H_
#define _WIFI_API_H_

#include "co_list.h"
#include "userconfig.h"
#include "wifi_def.h"
#include "wifi_driver_event.h"
#include "aic_cfg.h"

#define MACADDR_LEN 6
struct macaddr
{
    uint16_t array[MACADDR_LEN / 2];
};

struct aic_sta_info
{
    struct co_list_hdr list_hdr;
    uint8_t inst_nbr;
    uint8_t staid;
    bool valid;
    struct macaddr mac_addr;
    uint32_t last_active_time_us;
};

#define MAX_CHANNELS    14 // 2.4G
typedef struct _aic_config_scan_chan
{
    uint16_t action;
    uint16_t band;
    uint16_t total_chan;
    uint8_t  chan_num[MAX_CHANNELS];
} aic_config_scan_chan;

#define MAX_AP_COUNT 64

/* SSID 最大 32 字节 (802.11) + 1 字节 '\0' 结尾符 */
#define WIFI_MAX_SSID_LEN 32

/* 扫描 AP 认证/加密方式 (@ref wifi_ap_info_t.auth_mode) */
typedef enum
{
    AIC_WIFI_AUTH_NONE = 0,     /* 开放系统 (无加密) */
    AIC_WIFI_AUTH_WEP,          /* 共享密钥 (WEP) */
    AIC_WIFI_AUTH_WPA_PSK,      /* WPA-PSK */
    AIC_WIFI_AUTH_WPA2_PSK,     /* WPA2-PSK */
    AIC_WIFI_AUTH_WPA3_SAE,     /* WPA3-SAE */
    AIC_WIFI_AUTH_WPA2_WPA3,    /* WPA2/WPA3 混合 (PSK+SAE) */
    AIC_WIFI_AUTH_OTHER,        /* 802.1X 企业级/其他 */
} aic_wifi_auth_mode_e;

typedef struct wifi_ap_info
{
    unsigned char   ssid[WIFI_MAX_SSID_LEN + 1];    /* SSID, 恒以 '\0' 结尾; 隐藏 AP 填占位符 */
    unsigned char   ssid_len;                       /* SSID 实际字节数, 隐藏 AP = 0 */
    unsigned char   bssid[6];
    unsigned int    channel;
    unsigned int    rssi;
    unsigned char   auth_mode;                      /* 认证/加密方式 @ref aic_wifi_auth_mode_e */
} wifi_ap_info_t;

typedef struct _wifi_ap_list
{
    unsigned short  ap_count;
    wifi_ap_info_t ap_info[MAX_AP_COUNT];
} wifi_ap_list_t;

/* STA 已连接 AP 信息 (aic_wifi_get_sta_connected_ap_info 返回此结构)
 * 本结构为 platform 侧可见的副本; 内部依赖 host 的类型均以本地相同布局重定义,
 * 避免 platform 编译时引用 host 头文件。 */
struct aic_connected_ap_info_t
{
    /// SSID
    uint8_t ssid[32];

    /// BSSID (6 字节, 等价 host struct mac_addr)
    uint8_t bssid[6];

    /// Frequency of the operational channel in Hz
    uint16_t freq;

    uint8_t width;

    uint8_t band;

    // 0:bg, 1:n/ac, 2:ax
    uint8_t format;

    // unit: bps
    uint32_t max_rate;
    /// Supported AKM (bit-field of @ref mac_akm_suite)
    uint32_t akm;
    /// Group cipher (bit-field of @ref mac_cipher_suite)
    uint16_t group_cipher;
    /// Pairwise cipher (bit-field of @ref mac_cipher_suite)
    uint16_t pairwise_cipher;
};

struct aic_mm_set_acs_txop_req
{
    uint32_t hwconfig_id;
    uint16_t txop_bk;
    uint16_t txop_be;
    uint16_t txop_vi;
    uint16_t txop_vo;
};

struct aic_mm_set_channel_access_req
{
    uint32_t hwconfig_id;
    uint32_t edca[4];
    uint8_t  vif_idx;
    uint8_t  retry_cnt;
    uint8_t  rts_en;
    uint8_t  long_nav_en;
    uint8_t  cfe_en;
    uint8_t  rc_retry_cnt[3];
    int8_t  ccademod_th;
};

struct aic_mm_set_cca_threshold_req
{
    uint32_t hwconfig_id;
    uint8_t  auto_cca_en;
    int8_t  cca20p_rise_th;
    int8_t  cca20s_rise_th;
    int8_t  cca20p_fall_th;
    int8_t  cca20s_fall_th;
};

typedef enum aic_wifi_mac_status
{
    WIFI_STA_STATUS_DISCONNECTED,
    WIFI_STA_STATUS_CONNECTED,
} aic_wifi_mac_status_e;

typedef void (*aic_wifi_status_get_func)(aic_wifi_mac_status_e st);

void aic_get_chip_info(uint8_t *info);
uint8_t *aic_get_fw_ver(void);

/* 单库模式: 恒声明, 运行时按 chipid 判断是否支持 (仅 DC/DW 有效) */
uint32_t aic_get_dpd_result(void);

void aic_wifi_sw_version(char *ver);
void aic_set_adap_test(int value);
int aic_get_adap_test(void);
int aic_wifi_init(int mode, void *param);
void aic_wifi_deinit(int mode);
int aic_wifi_mode_get(void);
void aic_wifi_mode_set(int mode);
int aic_wifi_phy_freq_to_channel(uint8_t band, uint16_t freq);
uint16_t aic_wifi_phy_channel_to_freq(uint8_t band, int channel);
int aic_wifi_scan(void);
int aic_wifi_super_scan(uint8_t *ssid);
void aic_wifi_getscan(wifi_ap_list_t *ap_list);
int aic_wifi_fixrate(int argc, int argv[]);

int aic_wifi_get_temp(int32_t *temp);
/* D80X2 专属: 获取双天线 RSSI (ant0/ant1), 非 X2 芯片返回错误 */
int aic_wifi_get_two_ant_rssi(int8_t *ant0, int8_t *ant1);
int aic_wifi_send_txpwr_lvl_req(aic_txpwr_lvl_conf_v2_t *txpwrlvl);
int aic_wifi_send_txpwr_lvl_v3_req(aic_txpwr_lvl_conf_v3_t *txpwrlvl);
int aic_wifi_set_acs_txop_param(struct aic_mm_set_acs_txop_req *req);
int aic_wifi_set_channel_access_param(struct aic_mm_set_channel_access_req *req);
int aic_wifi_set_cca_threshold_param(struct aic_mm_set_cca_threshold_req *req);

int aic_wifi_set_agg(unsigned char en, unsigned char id);
int aic_channel_set(aic_config_scan_chan *config_scan_chan);
uint32_t aic_wifi_rx_data_rate_get(void);
uint32_t aic_wifi_tx_data_rate_get(void);

/*
 * FW 日志 sev_filter 阈值常量 (FW 内部 LMAC DBG 约定, 与 Linux 驱动 debugfs
 * fw_dbg 节点 "DBG:XXX" token 表一致; 注意与 aic_log.h 的 host 打印级别无关)
 */
#define AIC_FW_SEV_NONE  0   /* 关闭 FW 日志 */
#define AIC_FW_SEV_CRT   1
#define AIC_FW_SEV_ERR   2
#define AIC_FW_SEV_WRN   3
#define AIC_FW_SEV_INF   4
#define AIC_FW_SEV_VRB   5

/*
 * FW 日志 mod_filter 模块位常量 (FW 内部 LMAC 模块, 与 Linux "MOD:XXX"
 * token 表一致; 注意与 aic_log.h 的 host 打印模块 AIC_MOD_* 无关)
 */
#define AIC_FW_MOD_ALL   0xFFFFFFFFu
#define AIC_FW_MOD_KE    (1u << 0)   /* Kernel 内核 */
#define AIC_FW_MOD_DBG   (1u << 1)   /* DBG 框架 */
#define AIC_FW_MOD_IPC   (1u << 2)   /* IPC 通信 */
#define AIC_FW_MOD_DMA   (1u << 3)   /* DMA */
#define AIC_FW_MOD_MM    (1u << 4)   /* MAC 管理 */
#define AIC_FW_MOD_TX    (1u << 5)   /* 发送路径 */
#define AIC_FW_MOD_RX    (1u << 6)   /* 接收路径 */
#define AIC_FW_MOD_PHY   (1u << 7)   /* PHY/射频 */

/* 开机默认打开的 FW 日志模块组合 (DBG/DMA/MM/TX/RX/PHY, 不含 KE/IPC) */
#define AIC_FW_MOD_DEFAULT  (AIC_FW_MOD_DBG | AIC_FW_MOD_DMA | AIC_FW_MOD_MM | \
                             AIC_FW_MOD_TX  | AIC_FW_MOD_RX  | AIC_FW_MOD_PHY)

/*
 * FW 日志过滤器配置 (对应 Linux 驱动 debugfs fw_dbg 节点, 下发 DBG_SET_xxx_FILTER_REQ)
 * 前置条件: 先调 aic_wifi_set_lp_level(0, 1) 打开 FW 日志总闸门
 *
 * mod_filter 模块位图 (对应 Linux "MOD:XXX" token):
 *   bit0=KE bit1=DBG bit2=IPC bit3=DMA bit4=MM bit5=TX bit6=RX bit7=PHY
 *   0xFFFFFFFF = ALL
 * sev_filter 严重级别阈值 (对应 Linux "DBG:XXX" token, 只打 >= 阈值的日志):
 *   取值见 AIC_FW_SEV_xxx 宏
 *
 * 日志接收: fhost_rx() 处理总线 Host-Type 0x13 (CFG_PRINT) 包, 串口输出 "[FW] xxx"
 */
int aic_wifi_fw_log_set_mod_filter(uint32_t mod_filter);
int aic_wifi_fw_log_set_sev_filter(uint32_t sev_filter);

/*
 * FW 日志回传通道总开关 (对齐 Linux priv_cmd "fwlog_switch <0|1>"):
 * enable=1 打开 FW 日志重定向到 host (总线 Host-Type 0x13 CFG_PRINT 通道);
 * 这是日志能否回传的前提, 过滤器 (mod/sev) 只控制已产生日志的取舍
 */
int aic_wifi_fwlog_switch(uint32_t enable);

/*
 * FW 日志一站式开启: 回传通道总开关 + lp_level 总闸门 + mod/sev 过滤器
 * mod_filter/sev_filter 传 0 表示跳过该项 (保持 FW 当前值)
 * 典型用法: aic_wifi_fw_log_enable(0xFFFFFFFF, AIC_FW_SEV_INF)  // 全模块 + >=INF
 */
int aic_wifi_fw_log_enable(uint32_t mod_filter, uint32_t sev_filter);

#ifdef CONFIG_STATION
    void aic_wifi_scan_limit_set(uint8_t limit);
    uint8_t aic_wifi_scan_limit_get(void);
    void aci_wifi_set_sta_connect_bssid(uint8_t *addr);
    void aic_wifi_set_deepsleep_param(unsigned int listen_interval, unsigned int dont_wait_bcmc);
    int aic_wifi_set_lp_level(uint8_t lp_level, uint8_t disable_filter);
    void aic_wifi_sta_status_cb_set(aic_wifi_status_get_func func);
    int aic_wifi_start_sta(uint8_t *ssid, uint8_t *pw, int timeout_ms);
    int aic_wifi_stop_sta(void);
    int aic_wifi_get_sta_status(void);
    int8_t aic_wifi_ap_rssi_get(void);
    struct aic_connected_ap_info_t *aic_wifi_get_sta_connected_ap_info(void);
#endif

#ifdef CONFIG_SOFTAP
    int aic_wifi_start_ap(struct aic_ap_cfg *cfg);
    int aic_wifi_stop_ap(void);
    int aic_wifi_get_ap_status(void);
    int aic_wifi_get_ap_vif_idx(void);
    int aic_wifi_set_ps_lvl(int lvl);
    int aic_wifi_clear_blacklist(int fhost_vif_idx);
    int aic_wifi_clear_whitelist(int fhost_vif_idx);
    int aic_wifi_add_blacklist(int fhost_vif_idx, struct macaddr *macaddr);
    int aic_wifi_add_whitelist(int fhost_vif_idx, struct macaddr *macaddr);
    int aic_wifi_get_blacklist(int fhost_vif_idx, uint8_t *list, int *list_len);
    int aic_wifi_get_whitelist(int fhost_vif_idx, uint8_t *list, int *list_len);

    int aic_ap_macaddr_acl(int fhost_vif_idx, uint8_t acl);
    void aic_wifi_get_stalist2str(char *str);
    uint8_t aic_ap_get_associated_sta_cnt(void);
    void *aic_ap_get_associated_sta_list(void);
    int8_t aic_ap_get_associated_sta_rssi(uint8_t *addr);
    int8_t aic_ap_get_acs_start_channel(void);
    int aic_ap_wps_pbc(int fhost_vif_idx);
    int aic_ap_wps_pin(int fhost_vif_idx, char *pin);
    int aic_ap_wps_disable(int fhost_vif_idx);
    int aic_ap_wps_cacel(int fhost_vif_idx);
    int aic_ap_wps_config_methods(char *methods);
    int aic_ap_isolate_get(void);
    void aic_ap_isolate_set(int value);
#endif

#ifdef CONFIG_P2P_SUPPORT
    int aic_wifi_start_p2p(struct aic_p2p_cfg *cfg);
    int aic_wifi_stop_p2p(void);
    int aic_wifi_get_p2p_status(void);
#endif

#endif /* _WIFI_API_H_ */
