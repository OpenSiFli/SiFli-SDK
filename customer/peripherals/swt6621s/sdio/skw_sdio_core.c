/********************************/

#include "skw_msg.h"
#include "dfs_file.h"
#include "skw_cfg80211.h"

struct skw_scan_chan_info
{
    u8 chan_num;
    u8 band;
    u8 scan_flags;
};
struct skw_ctx_entry *skw_get_ctx_entry(struct skw_core *skw, const u8 *addr);
int wifi_setup_mac_address(uint8_t *mac);
struct skw_core skw_wifi = {0};
struct skw_iface iface = {0};
struct skw_core *skw_get_skw_core(void)
{
    return &skw_wifi;
}
struct skw_iface *skw_get_skw_iface(void)
{
    return &iface;
}
int skw_add_vif(void)
{
    struct skw_core *skw = skw_get_skw_core();
    struct skw_iface *iface = skw_get_skw_iface();

    iface->net_flags = NL80211_IFTYPE_STATION; // 默认是STA模式
    if (iface->id == 0xff)
        return 0;

    skw->vif.iface[iface->id] = iface;

    return 0;
}
#include "bf0_hal_aon.h"
uint32_t time_data_t = 0;
uint32_t open_time = 0, pack_num = 0;
int wifi_send_cmd(int portno, scatterlist *sg, int sg_num, int total);
static int skw_send_data(void *send_data, int send_len)
{
    struct sv6160_platform_data *wifi_data = skw_sdio_get_pdata();
    struct sdio_port *ports = skw_get_sdio_ports(wifi_data->data_port);
    struct skw_core *skw = skw_get_skw_core();
    struct skw_hw_extra *tx_hdl = (struct skw_hw_extra *)send_data; //{0};

    int total_len = send_len + sizeof(tx_hdl);
    tx_hdl->len = send_len;
    tx_hdl->pad = 0;
    tx_hdl->eof = 0;                        // 标志
    tx_hdl->channel = wifi_data->data_port; // 7

    uint8_t *data = rt_malloc(SKW_CMD_MAX_LEN);
    if (time_data_t == 0)
        open_time = HAL_GTIMER_READ();
    time_data_t++;
    uint8_t bloks = total_len / wifi_data->align_value;
    uint16_t bloks_tail = total_len % wifi_data->align_value;

    if (bloks_tail)
    {
        // rt_memset((data + total_len), 0x00, wifi_data->align_value -
        // bloks_tail);
        bloks++;
    }
    pack_num += bloks;
    // rt_memset(data, 0, total_len);
    // rt_memcpy(data, (void *)&tx_hdl, sizeof(tx_hdl));
    rt_memcpy(data, send_data, total_len);

    // skw_sdio_info("%s %d total_len=%d,bloks_tail=%d
    // bloks=%d\n",__func__,__LINE__,total_len,bloks_tail,bloks);
    int ret =
        wifi_send_cmd(wifi_data->data_port, (scatterlist *)data,
                      wifi_data->align_value * bloks, wifi_data->align_value);
    rt_free(data);
    return ret;
}
static int skw_set_cmd(struct skw_core *skw, int dev_id, int cmd, void *data,
                       int data_len, void *arg, int arg_size, char *name)
{
    struct sv6160_platform_data *wifi_data = skw_sdio_get_pdata();
    struct sdio_port *ports = skw_get_sdio_ports(wifi_data->cmd_port);
    ports->rx_count = 0;
    struct skw_msg msg_hdr;
    int total_len, msg_len, headr_len = 0;
    struct skw_hw_extra extra;
    struct skw_packet2_header header;
    uint8_t *cmd_send_data = rt_malloc(SKW_CMD_MAX_LEN);
    struct skw_cmd_data cmd_t = {0};

    total_len = msg_len = data_len + sizeof(cmd_t.msg_hdr);

    total_len += sizeof(cmd_t.extra);
    headr_len = sizeof(cmd_t.extra) + sizeof(cmd_t.msg_hdr);

    if (total_len > SKW_CMD_MAX_LEN)
    {
        skw_sdio_info("total_len: %d\n", total_len);
        return -E2BIG;
    }

    // msg_hdr 封装
    skw->cmd.seq++;

    cmd_t.extra.len = total_len;
    cmd_t.extra.pad = 0;
    cmd_t.extra.eof = skw->hw.extra.eof;
    cmd_t.extra.channel = wifi_data->cmd_port;

    cmd_t.msg_hdr.inst_id = dev_id;    // open的设备ID
    cmd_t.msg_hdr.type = SKW_MSG_CMD;  // 固定
    cmd_t.msg_hdr.id = cmd;            // 具体的cmd ID
    cmd_t.msg_hdr.seq = skw->cmd.seq;  // 消息验证ID  不能重复
    cmd_t.msg_hdr.total_len = msg_len; // msg+data 长度

    rt_memcpy(cmd_send_data, (void *)&cmd_t, headr_len);
    if (data_len)
    {
        data_len = data_len % 2 ? data_len + 1 : data_len;
        rt_memcpy(cmd_send_data + headr_len, (void *)data, data_len);
    }

    uint8_t bloks = total_len / wifi_data->align_value;
    uint16_t bloks_tail = total_len % wifi_data->align_value;
    if (bloks_tail)
    {
        rt_memset((cmd_send_data + total_len), 0x00,
                  wifi_data->align_value - bloks_tail);
        bloks++;
    }
    // if(arg_size)
    {
        rt_memcpy(&skw->cmd.name, name, strlen(name));

        ports->rx_data = arg;          // 需要接收的数据
        ports->rx_data_len = arg_size; // 需要发送的长度

    }

    wifi_send_cmd(wifi_data->cmd_port, (scatterlist *)cmd_send_data,
                  wifi_data->align_value * bloks,
                  wifi_data->align_value * bloks);

    rt_free(cmd_send_data);
    int ret = rt_completion_wait(&ports->rx_done, 3000); // 等待回复
    if (ret != RT_EOK)
    {
        skw_sdio_err("check send cmd time out ret=%d\n", ret);
        // RT_ASSERT(0);
        // return -ETIME;
    }
    if (arg_size)
    {
        if (ports->rx_count)
        {
            skw_sdio_info("%s %d sdio_port->rx_count=%d\n", __func__, __LINE__,
                          ports->rx_count);
        }
    }

    return 0;
}
int wifi_get_version(void)
{
    struct skw_version_info *wifi_version =
        rt_malloc(sizeof(struct skw_version_info));
    rt_err_t ret = RT_EOK;
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_SYN_VERSION, RT_NULL, 0,
                    wifi_version, sizeof(struct skw_version_info),
                    "version") != RT_EOK)
    {
        ret = RT_ERROR;
    }
    rt_free(wifi_version);
    return ret;
}

int wifi_chip_info(void)
{
    u64 ts = skw_local_clock();
    struct skw_chip_info *chip_info = rt_malloc(sizeof(struct skw_chip_info));
    rt_err_t ret = RT_EOK;
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_GET_INFO, &ts, sizeof(ts),
                    chip_info, sizeof(struct skw_chip_info),
                    "CHIP INFO") != RT_EOK)
    {
        ret = RT_ERROR;
    }
    skw_sdio_info("%s %d %x %x %x %x %x %x\n", __func__, __LINE__,
                  chip_info->mac[0], chip_info->mac[1], chip_info->mac[2],
                  chip_info->mac[3], chip_info->mac[4], chip_info->mac[5]);
    rt_free(chip_info);
    // chip_info->mac  拿到mac地址
    wifi_setup_mac_address(RT_NULL);
    return ret;
}
int wifi_setup_mac_address(uint8_t *mac)
{
    struct skw_core *skw = skw_get_skw_core();
    uint8_t addr[ETH_ALEN] = {0};
    if (mac) // 检查地址是否有效
    {
        skw_ether_copy(addr, mac);
    }
    else
    {
        addr[0] = 0x9c;
        addr[1] = 0x19;
        addr[2] = 0x17;
        addr[3] = 0xb8;
        addr[4] = 0xc3;
        addr[5] = 0xd6;
    }
    for (int i = 0; i < SKW_NR_IFACE; i++)
    {
        skw_ether_copy(skw->address[i].addr, addr);

        if (i != 0)
        {
            skw->address[i].addr[0] |= BIT(1);
            skw->address[i].addr[3] ^= BIT(i);
        }

        skw_sdio_info("addr[%d]: 0x%xM\n", i, skw->address[i].addr);
    }
    int wlan_set_sta_mac_addr(uint8_t *mac);
    wlan_set_sta_mac_addr((uint8_t *)&addr);
    return 0;
}
void wifi_mac_address(void)
{
    wifi_setup_mac_address(NULL);
}
MSH_CMD_EXPORT(wifi_mac_address, wifi_mac_address);
int skw_calib_download(void)
{
    struct dfs_fd fd;
    struct sv6160_platform_data *wifi_data = skw_sdio_get_pdata();
    char *read_bin_buff = (char *)rt_malloc(wifi_data->align_value);
    uint32_t read_len = 0;
    rt_err_t ret = RT_EOK;
    char path_file[BOOT_FILE_NAME_LEN] = {0};
    rt_sprintf(path_file, "%s/%s\0", CALIBRATION_IRAM_PATH,
               CALIBRATION_FILE_NAME);
    skw_sdio_info(" open file path %s \n", path_file);
    if (dfs_file_open(&fd, path_file, O_RDWR | O_CREAT) == 0)
    {
        while (1)
        {
            rt_memset(read_bin_buff, 0x0, wifi_data->align_value);
            read_len =
                dfs_file_read(&fd, read_bin_buff, wifi_data->align_value);
            if (!read_len)
            {
                skw_sdio_info("%s %d boot over \n", __func__, __LINE__);
                dfs_file_close(&fd);
                break;
            }
            skw_sdio_info("read_len=%d read_bin_buff=%p\n", read_len,
                          read_bin_buff);

            if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_PHY_BB_CFG,
                            (void *)read_bin_buff, read_len, RT_NULL, 0,
                            "CFG") != RT_EOK)
            {
                ret = RT_ERROR;
            }
        }
    }
    else
        skw_sdio_info(" open %s fialed\n", path_file);
    rt_free(read_bin_buff);

    return 0;
}

int skw_cmd_open_dev(uint32_t type, uint16_t flags)
{
    int mode, ret;
    struct skw_core *skw = skw_get_skw_core();
    struct skw_open_dev_param open_param;

    switch (type)
    {
    case NL80211_IFTYPE_ADHOC:
        mode = SKW_IBSS_MODE;
        break;
    case NL80211_IFTYPE_STATION:
        mode = SKW_STA_MODE;
        break;
    case NL80211_IFTYPE_AP:
        mode = SKW_AP_MODE;
        break;
    case NL80211_IFTYPE_P2P_CLIENT:
        mode = SKW_GC_MODE;
        break;
    case NL80211_IFTYPE_P2P_GO:
        mode = SKW_GO_MODE;
        break;
    case NL80211_IFTYPE_P2P_DEVICE:
        mode = SKW_P2P_DEV_MODE;
        break;
    case NL80211_IFTYPE_MONITOR:
        mode = SKW_MONITOR_MODE;
        break;
    default:
        skw_sdio_info("iftype: %d not support\n", type);
        return -EINVAL;
    }

    skw_ether_copy(open_param.mac_addr, &skw->address[0].addr[0]);
    open_param.mode = mode;
    open_param.flags = flags;

#ifdef CONFIG_SKW6316_OFFCHAN_TX
    open_param.flags |= SKW_OPEN_FLAG_OFFCHAN_TX;
#endif
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_OPEN_DEV, &open_param,
                    sizeof(open_param), RT_NULL, 0, "CFG") != RT_EOK)
    {
        ret = RT_ERROR;
    }
    return ret;
}
#define ASC_SCAN_ENABLE 1 // BIT(1)
#define RAND_MAC_ENABLE BIT(0)
uint8_t test_scan = 0;
int wifi_start_scan(struct cfg80211_scan_request *req)
{
#if 1
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    char *buff = NULL;
    struct skw_scan_param *param = NULL;
    struct skw_core *skw = skw_get_skw_core();
    int size, nssids_size, offset;
    int i;
    u16 scan_chn_num = 0;
    req->n_channels--;
    skw_sdio_info("%s %d n_channels=%d,n_ssids=%d req=%p\n", __func__, __LINE__,
                  req->n_channels, req->n_ssids, req);
    struct skw_scan_chan_info *chan;

    size = sizeof(struct skw_scan_param) + req->n_channels * sizeof(*chan) +
           req->n_ssids * sizeof(struct cfg80211_ssid) + req->ie_len;

    buff = rt_malloc(size);
    rt_memset(buff, 0, size);
    skw_sdio_info("%s %d size=%d buff=%p\n", __func__, __LINE__, size, buff);
    if (!buff)
    {
        skw_sdio_info("alloc scan buffer failed\n");
        RT_ASSERT(0);
    }
    offset = 0;

    param = (struct skw_scan_param *)buff;
    param->flags = ASC_SCAN_ENABLE;
    offset += sizeof(struct skw_scan_param);
    param->chan_offset = offset;
    chan = (struct skw_scan_chan_info *)(buff + offset);

    for (i = 0; i < req->n_channels; i++)
    {

        chan->chan_num = req->channels[i].chan_number; // 信道号
        chan->band = req->channels[i].radio_type;      // 频段
        // chan->scan_flags = 0;
        chan->scan_flags = SKW_SCAN_FLAG_PASSIVE; // 被动扫描
        skw_sdio_info("%s %d i=%d chan->band=%d,chan->chan_num=%d\n", __func__,
                      __LINE__, i, chan->band, chan->chan_num);

        scan_chn_num++;
        chan = (struct skw_scan_chan_info *)((u8 *)chan + sizeof(*chan));
    }
    param->nr_chan = scan_chn_num;
    offset += scan_chn_num * sizeof(*chan);

    param->n_ssid = req->n_ssids;
    if (req->n_ssids)
    {
        nssids_size = req->n_ssids * sizeof(struct cfg80211_ssid);
        memcpy(buff + offset, req->ssids, nssids_size);
        param->ssid_offset = offset;
        offset += nssids_size;
    }

    if (req->ie_len)
    {
        memcpy(buff + offset, req->ie, req->ie_len);
        param->ie_offset = offset;
        param->ie_len = req->ie_len;
    }

    skw->scan_req = req;
    skw->nr_scan_results = 0;

    rt_err_t ret = RT_EOK;
    // RT_ASSERT(0);
    //if(test_scan == 1) RT_ASSERT(0);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_START_SCAN, buff, size,
                    RT_NULL, 0, "scan") != RT_EOK)
    {
        ret = RT_ERROR;
    }
    test_scan = 1;
#endif
    return 0;
}

int skw_wifi_stop_scan(void)
{
    rt_err_t ret = RT_EOK;
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_STOP_SCAN, RT_NULL, 0,
                    RT_NULL, 0, "scan") != RT_EOK)
    {
        ret = RT_ERROR;
    }
    return ret;
}
MSH_CMD_EXPORT(skw_wifi_stop_scan, skw_wifi_stop_scan);
int skw_disconnected(void)
{
    rt_err_t ret = RT_EOK;
    struct skw_disconnect_param params = {0};
    params.type = SKW_DISCONNECT_ONLY;
    params.reason_code = 0;
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_DISCONNECT, &params,
                    sizeof(params), RT_NULL, 0, "disconnect") != RT_EOK)
    {
        ret = RT_ERROR;
        skw_sdio_info("%s %d disconnect error \n", __func__, __LINE__);
    }
    return ret;
}
MSH_CMD_EXPORT(skw_disconnected, skw_disconnected);

#include "skw_type_msg.h"
#include "skw_iface.h"

int skw_cmd_join(struct skw_join_param *params)
{
    struct skw_core *skw = skw_get_skw_core();
    struct skw_iface *iface = skw_get_skw_iface();
    iface->skw = skw;
    struct skw_join_resp resp = {};
    int ret = 0, size = 0;
    size = sizeof(struct skw_join_param) + params->bss_ie_len;
    skw_sdio_info("%s %d size=%d\n", __func__, __LINE__, size);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_JOIN, params, size, &resp,
                    sizeof(struct skw_join_resp), "connect") != RT_EOK)
    {
        ret = RT_ERROR;
        return ret;
    }
    skw_sdio_info("%s %d inst=%d,lmac_id=%d,multicast_idx=%d,peer_idx=%d\n",
                  __func__, __LINE__, resp.inst, resp.lmac_id, resp.multicast_idx,
                  resp.peer_idx);
    struct skw_peer *peer = skw_peer_alloc();
    skw_peer_init(peer, params->bssid, resp.peer_idx);
    struct skw_peer_ctx *ctx = skw_get_ctx(skw, resp.lmac_id, resp.peer_idx);
    ret = skw_peer_ctx_bind(iface, ctx, peer);

    skw->id_join_resp = resp.peer_idx;
    skw->skw_join_resp[skw->id_join_resp] = resp;

    if (ret < 0)
    {
        skw_sdio_info("%s %d skw_peer_ctx_bind failed ret=%d\n", __func__,
                      __LINE__, ret);
        return ret;
    }
    if (ret)
    {
        skw_sdio_info("%s %d skw_peer_ctx_bind failed ret=%d\n", __func__,
                      __LINE__, ret);
        return -1;
    }
    void skw_join_resp_handler(struct skw_core * skw, struct skw_iface * iface,
                               struct skw_join_resp * resp);
    skw_join_resp_handler(skw, iface, &resp);

#if 0
    ret = skw_peer_ctx_bind(iface, ctx, peer);
    if (ret)
    {
        skw_cmd_unjoin(wiphy, ndev, bss->bssid, SKW_LEAVE, false);
        SKW_KFREE(peer);
        return -EFAULT;
    }
#endif
    return ret;
}
int skw_cmd_auth(struct cfg80211_auth_request *req)
{
    int ret = 0;
    u16 auth_alg;
    int size, offset;
    struct skw_auth_param *params = NULL;
    struct skw_iface *iface = skw_get_skw_iface();
#if 1 // LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
    const u8 *auth_data = req->auth_data;
    size_t auth_data_len = req->auth_data_len;
#else
    const u8 *auth_data = req->sae_data;
    size_t auth_data_len = req->sae_data_len;
#endif
    skw_sdio_info("%s %d req->auth_type=%d\n", __func__, __LINE__, req->auth_type);
    switch (req->auth_type)
    {
    case NL80211_AUTHTYPE_OPEN_SYSTEM:
        auth_alg = WLAN_AUTH_OPEN;
        break;
    case NL80211_AUTHTYPE_SHARED_KEY:
        auth_alg = WLAN_AUTH_SHARED_KEY;
        break;
    case NL80211_AUTHTYPE_FT:
        auth_alg = WLAN_AUTH_FT;
        break;
    case NL80211_AUTHTYPE_NETWORK_EAP:
        auth_alg = WLAN_AUTH_LEAP;
        break;
    case NL80211_AUTHTYPE_SAE:
        auth_alg = WLAN_AUTH_SAE;
        break;
#if 1 // LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
    case NL80211_AUTHTYPE_FILS_SK:
        auth_alg = WLAN_AUTH_FILS_SK;
        break;
    case NL80211_AUTHTYPE_FILS_SK_PFS:
        auth_alg = WLAN_AUTH_FILS_SK_PFS;
        break;
    case NL80211_AUTHTYPE_FILS_PK:
        auth_alg = WLAN_AUTH_FILS_PK;
        break;
#endif
    case NL80211_AUTHTYPE_AUTOMATIC:
        /*
         * Fixme: try open wep first, then set share key after using
         * open wep failed.
         */
        auth_alg = WLAN_AUTH_OPEN;
        break;
    default:
        return -1;
    }

    size = sizeof(struct skw_auth_param) + req->ie_len + auth_data_len;

    params = rt_malloc(size);
    rt_memset(params, 0, size);
    skw_sdio_info(
        "%s %d size=%d params=%p auth_alg=%d auth_data_len=%d,req->ie_len=%d\n",
        __func__, __LINE__, size, params, auth_alg, auth_data_len, req->ie_len);
    offset = sizeof(struct skw_auth_param);
    // auth_alg = WLAN_AUTH_OPEN;
    // auth_alg = WLAN_AUTH_SHARED_KEY;
    params->auth_algorithm = req->auth_type;

    if (auth_data_len)
    {
        params->auth_data_offset = offset;
        params->auth_data_len = auth_data_len;

        memcpy((u8 *)params + offset, auth_data, auth_data_len);

        offset += auth_data_len;
    }

    if (req->ie && req->ie_len)
    {
        params->auth_ie_offset = offset;
        params->auth_ie_len = req->ie_len;
        memcpy((u8 *)params + offset, req->ie, req->ie_len);

        offset += req->ie_len;
    }
    // rt_memset(params, 0, sizeof(struct skw_auth_param));
    //  memcpy(iface->sta.core.pending.cmd, params, size);
    //  iface->sta.core.pending.cmd_len = size;
    skw_sdio_info("%s %d size=%d params=%p auth_alg=%d \n", __func__, __LINE__,
                  size, params, auth_alg);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_AUTH, params, size, RT_NULL,
                    0, "SKW_CMD_AUTH") != RT_EOK)
    {
        ret = RT_ERROR;
        return ret;
    }
    rt_free(params);

    return ret;
}

int skw_cmd_assoc(struct skw_assoc_req_param *params)
{
    struct skw_core *skw = skw_get_skw_core();
    struct skw_iface *iface = skw_get_skw_iface();
    iface->skw = skw;
    int ret = 0, size = 0;
    size = sizeof(struct skw_assoc_req_param) + params->req_ie_len;
    skw_sdio_info("%s %d size=%d\n", __func__, __LINE__, size);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_ASSOC, params, size, RT_NULL,
                    0, "assoc") != RT_EOK)
    {
        ret = RT_ERROR;
        return ret;
    }

    return ret;
}
// #include "lwip/udp.h"
#include "lwip/prot/udp.h"
#include "lwip/prot/ip4.h"
static bool skw_udp_filter(struct udp_hdr *udp)
{
    bool ret = false;
    // struct udphdr *udp = udp_hdr(skb);

#define DHCP_SERVER_PORT 0x43
#define DHCP_CLIENT_PORT 0x44
#define DHCPV6_SERVER_PORT 0x222
#define DHCPV6_CLIENT_PORT 0x223

    uint16_t udp_dest = ntohs(udp->dest);
    switch (udp_dest)
    {
    case DHCP_CLIENT_PORT:
    case DHCP_SERVER_PORT:
#if 0
        if (ndev->priv_flags & IFF_BRIDGE_PORT)
        {
            /* set BOOTP flag to broadcast */
            *((u8 *)udp + 18) = 0x80;
            udp->check = 0;
        }
#endif
    case DHCPV6_CLIENT_PORT:
    case DHCPV6_SERVER_PORT:
        ret = true;
        skw_sdio_info("DHCP, port: %d\n", udp_dest);
        break;

    default:
        ret = false;
        break;
    }

    return ret;
}
// 网络设备发送数据包接口
// 包含发向wifi的数据按照数据帧的格式进行封装
int skw_send_ethdata(void *skb, int buffer_len)
{
    int ret = 0;
    u8 tid = 0;
    u8 fixed_rate = 1;
    u8 peer_index = SKW_INVALID_ID;
    u8 ac_idx = 0, padding = 0;
    u8 prot = 0, tcp_pkt = 0, do_csum = 0;
    bool is_prot_filter = false;
    bool is_udp_filter = false;
    bool is_802_3_frame = false;
    bool pure_tcp_ack = false;
    const u8 tid_map[] = {6, 4, 0, 1};
    int l4_hdr_offset = 0, reset_l4_offset = 0;
    int msdu_len, align_len;
    int nhead, ntail, nroom, txq_len;
    bool is_completed = true;

    // struct netdev_queue *txq;
    struct skw_peer_ctx *ctx;
    struct skw_iface *iface = skw_get_skw_iface();
    struct ethhdr *eth = (struct ethhdr *)skb;
    struct skw_ctx_entry *entry = NULL;
    struct sk_buff_head *qlist;
    struct skw_tx_desc_hdr desc_hdr;
    struct skw_tx_desc_conf desc_conf;
    struct skw_core *skw = skw_get_skw_core();
    struct ip_hdr *ip_hdr = (struct ip_hdr *)(skb + sizeof(struct ethhdr));
    struct udp_hdr *udph = RT_NULL;
    uint8_t *data = RT_NULL;

#ifdef CONFIG_SKW6316_SKB_RECYCLE
    struct sk_buff *skb_recycle;
#endif
    s16 pkt_limit;
    /* Mini frame size that HW support */
    if (buffer_len <= 16)
    {
        skw_sdio_info("skb too small: %d\n", buffer_len);
        goto free;
    }

    msdu_len = buffer_len;

    uint16_t eth_type = ntohs(eth->h_proto);
    // skw_sdio_info("%s %d skb->len=0x%x
    // eth_type=%x\n",__func__,__LINE__,eth->h_proto,eth_type);
    switch (eth_type) // 帧类型区分
    {
    case ETH_P_IP: // IP帧
        prot = ip_hdr->_proto;
        reset_l4_offset = ETH_HLEN + sizeof(struct ip_hdr);
        // skw_sdio_info("%s %d
        // prot=%d,reset_l4_offset=%d\n",__func__,__LINE__,prot,reset_l4_offset);
        if (prot == IPPROTO_TCP)
        {
#if 0
            // 2. 计算TCP头部长度
            struct tcp_hdr *tcph =
                int ip_hdr_len = (iph->_v_hl & 0x0F) * 4;
            int tcp_hdr_len = ((tcph->offset_flags >> 12) & 0xF) * 4;
            int total_len = ntohs(iph->_len);
            int tcp_payload_len = total_len - ip_hdr_len - tcp_hdr_len;

            // 3. 检查TCP标志位和负载
            if ((tcph->flags == TCP_ACK) && (tcp_payload_len == 0))
            {
                // 这是一个纯TCP ACK包
                pure_tcp_ack = true;
            }
#endif
        }

        break;

    case ETH_P_ARP: // ARP帧

        is_prot_filter = true;

#if 0
        /*
        其主要目的是在特定场景下对 ARP 包进行特殊处理，尤其是无线中继（Repeater）和桥接（Bridge）场景。
        */
        if (unlikely(test_bit(SKW_FLAG_REPEATER, &skw->flags)) &&
                ndev->priv_flags & IFF_BRIDGE_PORT)
        {
            if (iface->wdev.iftype == NL80211_IFTYPE_STATION)
            {
                struct sk_buff *arp_skb;
                struct skw_ctx_entry *e;
                struct skw_arphdr *arp = skw_arp_hdr(skb);

                rcu_read_lock();
                e = skw_get_ctx_entry(iface->skw, arp->ar_sha);
                if (e)
                    e->peer->ip_addr = arp->ar_sip;

                rcu_read_unlock();

                arp_skb = arp_create(ntohs(arp->ar_op),
                                     ETH_P_ARP, arp->ar_tip,
                                     iface->ndev,
                                     arp->ar_sip, eth->h_dest,
                                     iface->addr, arp->ar_tha);

                kfree_skb(skb);

                skb = arp_skb;
                if (!skb)
                    return NETDEV_TX_OK;

                eth = skw_eth_hdr(skb);
            }
        }
#endif
        // fixed_rate = 1;
        // fixed_tid = 4;
        break;

    case ETH_P_PAE: // 握手帧
    // case htons(SKW_ETH_P_WAPI):
    case ETH_P_TDLS:
        is_prot_filter = true;
        break;

    default:
        // skw_sdio_info("%s %d
        // eth->h_proto=0x%x\n",__func__,__LINE__,eth->h_proto);
        if (eth_type < ETH_P_802_3_MIN)
            is_802_3_frame = true;

        break;
    }
    /* enable checksum for TCP & UDP frame, except framgment frame */
    switch (prot)
    {
    case IPPROTO_UDP: // 17
        udph = (struct udp_hdr *)((uint8_t *)skb + reset_l4_offset);
        is_udp_filter = skw_udp_filter(udph);
        if (udph->chksum == 0)
            do_csum = 0;

        break;

    case IPPROTO_TCP: // 6
        tcp_pkt = 1;

        break;

    default:
        break;
    }
#if 0 // 获取优先级
    ac_idx = skw_downgrade_ac(iface, g_skw_up_to_ac[skb->priority]);
    tid = tid_map[ac_idx];
#endif

    switch (iface->net_flags)
    {
    case NL80211_IFTYPE_AP:
    case NL80211_IFTYPE_P2P_GO:
#if 0
        if (is_unicast_ether_addr(eth->h_dest))
        {
            entry = skw_get_ctx_entry(skw, eth->h_dest);
            peer_index = entry ? entry->idx : SKW_INVALID_ID;

            if (entry && entry->peer->sm.state != SKW_STATE_COMPLETED)
                is_completed = false;
        }
        else
        {
            fixed_rate = 1;
            peer_index = iface->default_multicast;
        }

        break;
#endif
    case NL80211_IFTYPE_STATION:
    case NL80211_IFTYPE_P2P_CLIENT:
    case NL80211_IFTYPE_ADHOC:
        entry = skw_get_ctx_entry(skw, eth->h_dest);
        if (!entry)
        {
            ctx = skw_get_ctx(skw, iface->lmac_id,
                              skw->skw_join_resp[skw->id_join_resp].peer_idx);
            if (ctx)
                entry = ctx->entry;
        }
        // skw_sdio_info("%s %d
        // entry=%p,entry->idx=%d\n",__func__,__LINE__,entry,entry->idx);
        peer_index = 0; // entry ? entry->idx : SKW_INVALID_ID;

        if (peer_index != SKW_INVALID_ID &&
                is_multicast_ether_addr(eth->h_dest))
        {
            // 判断一个以太网MAC地址是否为多播地址
            fixed_rate = 1;
            peer_index = 0; // iface->default_multicast;
            // skw_sdio_info("%s %d is_multicast_ether_addr
            // peer_index=%d,iface->default_multicast=%d\n",__func__,__LINE__,
            //                                              peer_index,iface->default_multicast);
        }
#if 0 // 是否从处于连接状态
        if (iface->sta.core.sm.state != SKW_STATE_COMPLETED)
            is_completed = false;
#endif
        break;

    default:
        peer_index = SKW_INVALID_ID;
        break;
    }
#if 0
    /*
    skw_setup_txba 用于驱动侧为指定对端和流（TID）发起 802.11 Block Ack（BA）发送端会话的建立，
    是无线高吞吐量数据聚合的关键流程
    */
    if (entry && is_completed)
        skw_setup_txba(skw, iface, entry->peer, tid);
#endif
    if (peer_index == SKW_INVALID_ID)
    {
        goto free;
    }


#if 1

    desc_conf.csum = do_csum;
    desc_conf.ip_prot = tcp_pkt;
    desc_conf.l4_hdr_offset = l4_hdr_offset; // 不需要校验和
    // 2个字节
    desc_hdr.padding_gap = padding;
    desc_hdr.inst = iface->id & 0x3;
    desc_hdr.lmac_id = iface->lmac_id;

    desc_hdr.tid = tid;
    desc_hdr.peer_lut = peer_index;
    desc_hdr.frame_type = SKW_ETHER_FRAME;
    desc_hdr.eth_type = eth->h_proto;

    desc_hdr.encry_dis = 0;
    desc_hdr.msdu_len = msdu_len;
    desc_hdr.rate = 0; // fixed_rate;
    // 6个字节
    int send_size = buffer_len + sizeof(desc_hdr) + sizeof(desc_conf);
#endif

    if (is_prot_filter || is_udp_filter ||
            is_802_3_frame) // 特殊帧采用CMD的方式进行发送
    {

        data = rt_malloc(send_size + 4);
        if (!data)
        {
            skw_sdio_info("skb data alloc failed\n");
            goto free;
        }

        rt_memset(data, 0, send_size);

        rt_memcpy(data, &desc_hdr, sizeof(desc_hdr));
        rt_memcpy(data + sizeof(desc_hdr), &desc_conf, sizeof(desc_conf));
        rt_memcpy(data + sizeof(desc_conf) + sizeof(desc_hdr), skb, buffer_len);
        // 如果是协议过滤或者udp过滤或者802.3帧 就走cmd 发送接口
        if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_TX_DATA_FRAME, data,
                        send_size, RT_NULL, 0,
                        "SKW_CMD_TX_DATA_FRAME") != RT_EOK)
        {
            ret = RT_ERROR;
        }
        if (data)
            rt_free(data);
    }
    else
    {
        // 纯数据帧采用数据流的方式进行发送

        rt_memcpy(skb - 8, &desc_hdr, sizeof(desc_hdr));
        rt_memcpy(skb - 2, &desc_conf, sizeof(desc_conf));
        struct tcp_header *tcps = skb + 34;
        skw_send_data((void *)(skb - 12), send_size);
    }

    if (pure_tcp_ack)
        ac_idx = SKW_ACK_TXQ;

// 需要做数据流控
//  trace_skw_tx_xmit(eth->h_dest, peer_index, prot, fixed_rate,
//         do_csum, ac_idx, skb_queue_len(qlist));//数据发送接口
// skw_send_data(data, send_size);

free:
    return ret;
}
void skw_wifi_setup_txba(void)
{
    int ret = 0;
    struct skw_ba_action tx_ba = {0};
    tx_ba.tid = 0; // 优先级
    tx_ba.win_size = 200;
    tx_ba.lmac_id = 0;
    tx_ba.peer_idx = 0;
    tx_ba.action = SKW_ADD_TX_BA;
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_BA_ACTION, &tx_ba,
                    sizeof(tx_ba), RT_NULL, 0, "SKW_CMD_BA_ACTION") != RT_EOK)
    {
        ret = -RT_ERROR;
        goto free;
    }
free:
    return;
}
void skw_wifi_event_ba_action(void *data)
{
    // 处理BA事件
    struct skw_ba_action *ba = (struct skw_ba_action *)data;
    skw_sdio_info(
        "%s %d ba->action=%x ba->tid=%x ba->status_code=%x,ba->win_size=%x\n",
        __func__, __LINE__, ba->action, ba->tid, ba->status_code, ba->win_size);
}
#include "defs.h"
void skw_wifi_set_key(int cipher, u8 key_idx, int key_type, const u8 *key,
                      int key_len, const u8 *addr)
{
    int ret = 0;
    struct skw_key_params params = {0};
    u8 wapi_tx_pn[] = {0x36, 0x5c, 0x36, 0x5c, 0x36, 0x5c};
    if (addr)
        memcpy(params.mac_addr, addr, ETH_ALEN);
    else
        memset(params.mac_addr, 0xff, ETH_ALEN);

    memcpy(params.key, key, key_len);
    params.key_type = key_type ? SKW_KEY_TYPE_PTK : SKW_KEY_TYPE_GTK;
    params.key_len = key_len;
    params.key_id = key_idx;
    // params.cipher_type = cipher;
    switch (cipher)
    {
    case WPA_ALG_CCMP:
        params.cipher_type = SKW_CIPHER_TYPE_CCMP;
        break;
    case WPA_ALG_CCMP_256:
        params.cipher_type = SKW_CIPHER_TYPE_CCMP_256;
        break;
    case WPA_ALG_WEP:
        params.cipher_type = SKW_CIPHER_TYPE_WEP40;
        break;
    case WPA_ALG_TKIP:
        params.cipher_type = SKW_CIPHER_TYPE_TKIP;
        break;
    case WPA_ALG_BIP_CMAC_128:
        params.cipher_type = SKW_CIPHER_TYPE_BIP_GMAC_128;
        break;
    case WPA_ALG_GCMP:
        params.cipher_type = SKW_CIPHER_TYPE_GCMP;
        break;
    case WPA_ALG_SMS4:
        params.cipher_type = SKW_CIPHER_TYPE_SMS4;
        break;
    case WPA_ALG_GCMP_256:
        params.cipher_type = SKW_CIPHER_TYPE_GCMP_256;
        break;
    case WPA_ALG_BIP_GMAC_256:
        params.cipher_type = SKW_CIPHER_TYPE_BIP_GMAC_256;
        break;
    case WPA_ALG_BIP_CMAC_256:
        params.cipher_type = SKW_CIPHER_TYPE_BIP_CMAC_256;
        break;
    }

    params.pn[0] = 1;

    switch (params.cipher_type)
    {
    case SKW_CIPHER_TYPE_SMS4:
        memcpy(params.pn, wapi_tx_pn, SKW_PN_LEN);

        if (0) //(is_skw_ap_mode(iface))
            params.pn[0] += 1;

        break;

    case SKW_CIPHER_TYPE_TKIP:
        if (0) //(is_skw_ap_mode(iface))
            memcpy(&params.key[0], key, 32);
        else
        {
            memcpy(&params.key[0], key, 16);
            memcpy(&params.key[16], key + 24, 8);
            memcpy(&params.key[24], key + 16, 8);
        }

        break;

    default:
        break;
    }
    skw_sdio_info("%s %d key_type=%d cipher=%d key_idx=%d,key_len=%d\n", __func__,
                  __LINE__, key_type, cipher, key_idx, key_len);
    skw_sdio_info("%s %d addr=%x:%x:%x:%x:%x:%x\n", __func__, __LINE__, addr[0],
                  addr[1], addr[2], addr[3], addr[4], addr[5]);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_ADD_KEY, &params,
                    sizeof(params), RT_NULL, 0, "SKW_CMD_ADD_KEY") != RT_EOK)
    {
        ret = RT_ERROR;
        // goto free;
    }
    // RT_ASSERT(0);
}

int skw_wifi_del_key(u8 key_idx, int key_type, int cipher, const u8 *addr)
{
    struct skw_key_params params;
    int ret = 0;
    memset(&params, 0x0, sizeof(params));

    if (addr)
        skw_ether_copy(params.mac_addr, addr);
    else
        memset(params.mac_addr, 0xff, ETH_ALEN);

    params.key_type = key_type;
    params.cipher_type = cipher;
    params.key_id = key_idx;
    skw_sdio_info("%s %d key_type=%d cipher=%d key_idx=%d\n", __func__, __LINE__,
                  key_type, cipher, key_idx);
    skw_sdio_info("%s %d addr=%x:%x:%x:%x:%x:%x\n", __func__, __LINE__, addr[0],
                  addr[1], addr[2], addr[3], addr[4], addr[5]);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_DEL_KEY, &params,
                    sizeof(params), RT_NULL, 0, "SKW_CMD_DEL_KEY") != RT_EOK)
    {
        ret = RT_ERROR;
        // goto free;
    }
    return 0;
}
uint32_t ip_to_uint32(const char *ip_str)
{
    uint8_t octet[4] = {0};
    int ret;

    ret = sscanf(ip_str, "%hhu.%hhu.%hhu.%hhu",
                 &octet[0], &octet[1], &octet[2], &octet[3]);

    if (ret != 4)
    {
        return 0;
    }

    for (int i = 0; i < 4; i++)
    {
        if (octet[i] > 255)
        {
            return 0;
        }
    }
    return (uint32_t)octet[3] << 24 |
           (uint32_t)octet[2] << 16 |
           (uint32_t)octet[1] << 8  |
           (uint32_t)octet[0];
}
void skw_wifi_set_ipaddr(char *ip)
{
    int ret = 0;
    skw_sdio_info("%s %d ipaddr=0x%s\n", __func__, __LINE__, ip);
    uint32_t ipaddr = ip_to_uint32(ip);
    if (0 == ipaddr)
    {
        skw_sdio_info("%s %d ipaddr error !\n", __func__, __LINE__);
        return;
    }
    struct skw_setip_param setip_param = {0};
    struct skw_core *skw = skw_get_skw_core();
    setip_param.ip_type = SKW_IP_IPV4;
    setip_param.ipv4 = ipaddr;
    skw_sdio_info("%s %d ipaddr=0x%x\n", __func__, __LINE__, ipaddr);
    if (skw_set_cmd(skw_get_skw_core(), 0, SKW_CMD_SET_IP, &setip_param,
                    sizeof(struct skw_setip_param), RT_NULL, 0,
                    "SKW_CMD_SET_IP") != RT_EOK)
    {
        ret = RT_ERROR;
    }
    skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
}
void wifi_info_init(void)
{
    wifi_get_version();
    wifi_chip_info();
    wifi_setup_mac_address(RT_NULL);
    skw_calib_download();
    skw_cmd_open_dev(NL80211_IFTYPE_STATION, 0);
    // wifi_start_scan(0);
}
struct dfs_fd wifi_log_fd;
uint8_t wifi_dfs_log = 0;
void wifi_log_en(int argc, char **argv)
{
    int en = atoi(argv[1]);
    skw_sdio_info("%s %d en=%d\n", __func__, __LINE__, en);
    if (en)
    {
        dfs_file_close(&wifi_log_fd);
        wifi_dfs_log = 0;
    }
    else
    {
        if (dfs_file_open(&wifi_log_fd, "/wifi/wifi_log.txt",
                          O_RDWR | O_CREAT | O_TRUNC) == 0)
        {
            skw_sdio_info("open wifi log file success\n");
            wifi_dfs_log = 1;
        }
    }
    skw_sdio_writeb(SDIOHAL_CPLOG_TO_AP_SWITCH,
                    en); // 开启固件log输出   0是使能  1是禁用
    skw_sdio_writeb(SKW_AP2CP_IRQ_REG, BIT(5));
}
MSH_CMD_EXPORT(skw_wifi_setup_txba, skw_wifi_setup_txba);
MSH_CMD_EXPORT(wifi_log_en, wifi_log_en);
MSH_CMD_EXPORT(wifi_info_init, wifi_info_init);
MSH_CMD_EXPORT(wifi_chip_info, wifi_chip_info);

/*
 * callback function, invoked by bsp
 */
int skw_rx_cb(int port, scatterlist *sglist, int nents, void *priv)
{
    struct sv6160_platform_data *wifi_data = skw_sdio_get_pdata();
    struct sdio_port *ports = skw_get_sdio_ports(wifi_data->cmd_port);

    int ret;
    bool rx_sdma;
    void *sg_addr;
    int idx, total_len;
    scatterlist *sg;
    struct skw_msg msg;
    struct skw_iface *iface;
    struct skw_core *skw = (struct skw_core *)priv;

    u8 usb_data_port = 0;
    int len = ports->sg_len;
    sg_addr = ports->sg_rx;
    uint8_t *data = rt_malloc(len);
    rt_memcpy(&msg, sg_addr, sizeof(struct skw_msg));
    rt_memcpy(data, sg_addr + sizeof(struct skw_msg), len);
    if (port)
    {

        switch (msg.type)
        {
        case SKW_MSG_CMD_ACK:

            // skw_cmd_ack_handler(skw, data, len);

            break;

        case SKW_MSG_EVENT:
            iface = to_skw_iface(skw, msg.inst_id);
            if (iface)
                iface->iface_event_handler(iface->skw, iface, &msg, data, len);
            else
                skw->skw_event_handler(skw, RT_NULL, &msg, data, len);

            break;

        default:
            skw_sdio_info("invalid: type: %d, id: %d, seq: %d\n", msg.type, msg.id,
                          msg.seq);
            break;
        }
    }
    rt_free(data);

    return 0;
}
