/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * wifi_peer instrument firmware: the AP the DUT joins.
 *
 * Only the SoftAP side is implemented. A station role would need a supply of
 * networks to join, which the lab does not control; an AP is something this
 * image can be, so the DUT has a known SSID, a known key and a known BSSID to
 * assert against.
 *
 * What the peer can and cannot see:
 *
 *  - The AP interface's own MAC is available from the driver
 *    (`ops->wlan_get_mac`); it is what the DUT reports as `MAC Addr:` in
 *    `wifi status`, so the two sides can agree on an identity.
 *  - The list of associated stations is not reliable on this platform: the
 *    wlan framework fills it from RT_WLAN_DEV_EVT_AP_ASSOCIATED indications,
 *    and the AIC8800 driver does not forward association events to the wlan
 *    device layer, so rt_wlan_ap_get_sta_num() stays 0. `ap.status` reports
 *    `sta_num` as information only; a suite must not assert on it.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>

#include <wlan_dev.h>
#include <wlan_mgnt.h>
#include <lwip/netif.h>

#include "peer.h"

#define PEER_AP_DEFAULT_CHANNEL     6
#define PEER_AP_MIN_CHANNEL         1
#define PEER_AP_MAX_CHANNEL         13
#define PEER_AP_MAC_LEN             18
/* rt_wlan_start_ap_adv() waits RT_WLAN_START_AP_WAIT_MS (10 s) for the
 * framework's AP_START event. On this driver the SoftAP path first switches
 * chip mode and reloads firmware, so on a slow boot the event can arrive after
 * that wait has expired -- the AP does come up, only the caller has given up.
 * Keep watching before reporting a failure. */
#define PEER_AP_LATE_WAIT_MS        15000
#define PEER_AP_LATE_POLL_MS        200

/* The default address this image gives its SoftAP.
 *
 * It matters: every AIC8800 module in the lab reports the same burned-in MAC
 * (88:00:33:77:69:22 on the two boards this suite was built on), so a peer AP
 * that keeps the default is a BSSID equal to the DUT's own address. A station
 * cannot associate with a BSSID that is its own address, and the DUT's scan
 * does not report it either -- the DUT simply never sees the SSID. Starting
 * the AP from a locally administered address keeps the two boards distinct on
 * the air. `ap.start mac=...` overrides it when a bench has several peers. */
#define PEER_AP_DEFAULT_MAC         "02:00:5e:00:00:01"

/* Name of the lwIP interface that belongs to the AP vif.
 *
 * wlan_lwip.c hands out "w0", "w1", ... to wlan interfaces in registration
 * order (wlan_lwip.c:401-403), and this driver registers STA before AP, so the
 * AP is "w1" -- `ifconfig` on the peer prints w0 as the idle STA and w1 as the
 * AP. It is needed below because lwIP keeps its own copy of the address. */
#define PEER_AP_NETIF_NAME          "w1"

/* Set once the framework has bound (and therefore initialized) the AP device.
 * Every rt_wlan_dev_* call -- get_mac included -- goes through
 * _rt_wlan_dev_control(), which takes wlan->lock, and that mutex is only
 * created by rt_wlan_dev_init() from inside rt_wlan_set_mode(). Touching the
 * device before that asserts in rt_mutex_take(). */
static rt_bool_t s_ap_bound = RT_FALSE;

/* The parameters of the last successful start.
 *
 * Kept so "this AP is already serving what was asked for" can be decided on
 * the whole configuration and not just on the SSID: an AP that keeps its SSID
 * and channel but changes its key or hidden flag is a different network, and
 * answering `already=1` for it would leave the old key in force while telling
 * the host the new one took. */
typedef struct
{
    rt_bool_t valid;
    char ssid[RT_WLAN_SSID_MAX_LENGTH + 1];
    char key[RT_WLAN_PASSWORD_MAX_LENGTH + 1];
    int channel;
    int hidden;
} peer_ap_config_t;

static peer_ap_config_t s_last;

void peer_ap_forget(void)
{
    rt_memset(&s_last, 0, sizeof(s_last));
}


void peer_ap_init(void)
{
    /* Bind the AP interface at boot, before the instrument is reachable, so
     * that the device lock exists by the time any command reads the MAC.
     *
     * This also keeps the chip out of STA+AP concurrency: the driver's own
     * auto-init binds wlan0 as a station, and starting the SoftAP from that
     * state goes through wifi_switch_mode() into WIFI_MODE_APSTA. A concurrent
     * AP with no station link was what the DUT could not hear. Setting the
     * mode here makes aic8800_wlan_softap()'s own switch a no-op. */
    if (rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP) == RT_EOK)
    {
        s_ap_bound = RT_TRUE;
    }
    else
    {
        peer_log("wifi_peer: AP interface %s is not available",
                 RT_WLAN_DEVICE_AP_NAME);
    }
}

/**
 * Read the AP interface's current MAC.
 *
 * Returns -RT_ENOSYS when the platform exposes no get_mac op, so the caller
 * can report `mac=` as empty rather than inventing one.
 */
static rt_err_t peer_ap_mac_raw(rt_uint8_t mac[6])
{
    rt_device_t device;
    struct rt_wlan_device *wlan;

    if (!s_ap_bound)
    {
        return -RT_ENOSYS;
    }

    device = rt_device_find(RT_WLAN_DEVICE_AP_NAME);
    if (device == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    wlan = (struct rt_wlan_device *)device;
    if (wlan->ops == RT_NULL || wlan->ops->wlan_get_mac == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    return wlan->ops->wlan_get_mac(wlan, mac);
}

static rt_err_t peer_ap_mac(char *out, rt_size_t out_size)
{
    rt_uint8_t mac[6];

    if (out_size < PEER_AP_MAC_LEN)
    {
        return -RT_EINVAL;
    }

    if (peer_ap_mac_raw(mac) != RT_EOK)
    {
        return -RT_EIO;
    }

    rt_snprintf(out, out_size, "%02x:%02x:%02x:%02x:%02x:%02x",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return RT_EOK;
}

/** Parse `aa:bb:cc:dd:ee:ff` (':' or '-' between octets). */
static rt_err_t peer_ap_parse_mac(const char *text, rt_uint8_t mac[6])
{
    int i;

    if (text == RT_NULL || text[0] == '\0')
    {
        return -RT_EINVAL;
    }

    for (i = 0; i < 6; i++)
    {
        char *end = RT_NULL;
        long value = strtol(text, &end, 16);

        if (end == text || value < 0 || value > 0xff)
        {
            return -RT_EINVAL;
        }
        mac[i] = (rt_uint8_t)value;
        text = end;

        if (i < 5)
        {
            if (*text != ':' && *text != '-')
            {
                return -RT_EINVAL;
            }
            text++;
        }
    }

    return (*text == '\0') ? RT_EOK : -RT_EINVAL;
}

/**
 * Copy @p mac into the lwIP interface of the AP.
 *
 * lwIP stamps the source address of every frame it sends from netif->hwaddr,
 * and that copy was taken when the interface was created -- before set_mac()
 * ran. Without this step the AP is two-faced on the air: beacons, probe
 * responses, auth/assoc replies and EAPOL are built by the firmware and carry
 * the new address, while DHCP/ARP/ICMP leave with the burned-in one. A station
 * whose own address happens to be that burned-in address (every module in the
 * lab reports the same one) then drops the answer as a frame from itself, so
 * the DUT would associate and never get a lease.
 */
static void peer_ap_sync_netif_mac(const rt_uint8_t mac[6])
{
    struct netif *nif = netif_find(PEER_AP_NETIF_NAME);

    if (nif == RT_NULL)
    {
        peer_log("wifi_peer: no lwip interface %s, AP data frames keep the old address",
                 PEER_AP_NETIF_NAME);
        return;
    }

    rt_memcpy(nif->hwaddr, mac, 6);
}

/**
 * Give the AP interface @p mac.
 *
 * The framework reaches the driver's set_mac op (aic_set_mac_address), which
 * updates the address new virtual interfaces are created with -- so this has
 * to run before the SoftAP is started, not after.
 */
static rt_err_t peer_ap_apply_mac(const rt_uint8_t mac[6])
{
    rt_device_t device;
    rt_uint8_t current[6];
    rt_err_t rc;

    device = rt_device_find(RT_WLAN_DEVICE_AP_NAME);
    if (device == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    if (peer_ap_mac_raw(current) == RT_EOK && rt_memcmp(current, mac, 6) == 0)
    {
        /* Already the wanted address; the lwIP copy may still be stale if the
         * driver was set up by an earlier run, so refresh it as well. */
        peer_ap_sync_netif_mac(mac);
        return RT_EOK;
    }

    rc = rt_wlan_dev_set_mac((struct rt_wlan_device *)device, (rt_uint8_t *)mac);
    if (rc == RT_EOK)
    {
        peer_ap_sync_netif_mac(mac);
    }

    return rc;
}

static void peer_ap_mac_or_empty(char *out, rt_size_t out_size)
{
    if (peer_ap_mac(out, out_size) != RT_EOK)
    {
        out[0] = '\0';
    }
}

/**
 * Wait until the framework reports the AP active, or timeout_ms elapses.
 *
 * Used only after rt_wlan_start_ap_adv() has already returned: see
 * PEER_AP_LATE_WAIT_MS for why that answer alone is not the whole story.
 */
static rt_bool_t peer_ap_wait_active(rt_uint32_t timeout_ms)
{
    rt_uint32_t waited = 0;

    while (waited < timeout_ms)
    {
        if (rt_wlan_ap_is_active())
        {
            return RT_TRUE;
        }
        rt_thread_mdelay(PEER_AP_LATE_POLL_MS);
        waited += PEER_AP_LATE_POLL_MS;
    }

    return rt_wlan_ap_is_active();
}

/* A value the host can read back from a `key=value` token as it was written. */
static rt_bool_t peer_ap_single_token(const char *text)
{
    const char *cursor;

    for (cursor = text; *cursor != '\0'; cursor++)
    {
        if (*cursor == ' ' || *cursor == '\t' || *cursor == '=')
            return RT_FALSE;
    }

    return RT_TRUE;
}

/**
 * Render the `ssid=` fields of a reply or an event.
 *
 * `ssid_hex=` is always there and is what the host parses; `ssid=` is added
 * only when the name is a single token, so a transcript stays readable without
 * making the parser depend on whitespace behaving.
 */
static void peer_ap_ssid_fields(char *out, rt_size_t out_size, const char *ssid)
{
    char hex[RT_WLAN_SSID_MAX_LENGTH * 2 + 1];

    out[0] = '\0';

    if (peer_hex_encode(ssid, hex, sizeof(hex)) < 0)
        return;

    if (ssid[0] != '\0' && peer_ap_single_token(ssid))
    {
        rt_snprintf(out, out_size, "ssid=%s ssid_hex=%s", ssid, hex);
    }
    else
    {
        rt_snprintf(out, out_size, "ssid_hex=%s", hex);
    }
}

int peer_ap_start(const peer_args_t *args, peer_reply_t *reply)
{
    char ssid[RT_WLAN_SSID_MAX_LENGTH + 1];
    char key[RT_WLAN_PASSWORD_MAX_LENGTH + 1];
    char fields[128];
    const char *mac_text = peer_arg_get(args, "mac");
    struct rt_wlan_info info;
    struct rt_wlan_info running;
    char mac[PEER_AP_MAC_LEN];
    char wanted[PEER_AP_MAC_LEN];
    rt_uint8_t mac_bytes[6];
    rt_size_t ssid_len;
    int channel;
    int hidden;
    int actual_channel;
    rt_bool_t keyed;
    rt_err_t err;

    if (peer_arg_get(args, "ssid") == RT_NULL &&
            peer_arg_get(args, "ssid_hex") == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "ssid");
        return -1;
    }

    if (peer_arg_str(args, "ssid_hex", "ssid", ssid, sizeof(ssid)) != 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "ssid_encoding");
        return -1;
    }

    ssid_len = rt_strlen(ssid);
    if (ssid_len == 0 || ssid_len > RT_WLAN_SSID_MAX_LENGTH)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "ssid");
        return -1;
    }

    if (peer_arg_str(args, "key_hex", "key", key, sizeof(key)) != 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "key_encoding");
        return -1;
    }
    keyed = (key[0] != '\0');

    if (peer_arg_int(args, "ch", PEER_AP_DEFAULT_CHANNEL, &channel) < 0 ||
            channel < PEER_AP_MIN_CHANNEL || channel > PEER_AP_MAX_CHANNEL)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "ch");
        return -1;
    }

    if (peer_arg_int(args, "hidden", 0, &hidden) < 0 || (hidden != 0 && hidden != 1))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "hidden");
        return -1;
    }

    if (peer_ap_parse_mac(mac_text != RT_NULL ? mac_text : PEER_AP_DEFAULT_MAC,
                          mac_bytes) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "mac");
        return -1;
    }
    rt_snprintf(wanted, sizeof(wanted), "%02x:%02x:%02x:%02x:%02x:%02x",
                mac_bytes[0], mac_bytes[1], mac_bytes[2],
                mac_bytes[3], mac_bytes[4], mac_bytes[5]);

    /* Bind the AP interface first. Without it every rt_wlan_ap_* call answers
     * -RT_EIO, and every rt_wlan_dev_* call would take a mutex that does not
     * exist yet. This is the step the example under test misses when its own
     * `#if` excludes the mode switch. */
    if (rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_RADIO, "set_mode");
        return -1;
    }
    s_ap_bound = RT_TRUE;

    peer_ap_mac_or_empty(mac, sizeof(mac));

    /* Serving exactly this network already: report it instead of tearing a
     * working AP down and back up, which would drop the DUT mid-suite.
     *
     * "Exactly this network" is the whole requested configuration, not just
     * the name: an AP that keeps its SSID and channel but is asked for a
     * different key or hidden flag must be restarted, or the host is told the
     * new credentials are in force while the old ones still are. A different
     * address counts as a different network too: the DUT identifies the peer
     * by its BSSID. */
    if (rt_wlan_ap_is_active() && s_last.valid &&
            rt_strcmp(s_last.ssid, ssid) == 0 &&
            rt_strcmp(s_last.key, key) == 0 &&
            s_last.channel == channel &&
            s_last.hidden == hidden &&
            rt_wlan_ap_get_info(&running) == RT_EOK &&
            running.ssid.len == ssid_len &&
            rt_memcmp(running.ssid.val, ssid, ssid_len) == 0 &&
            running.channel == channel &&
            rt_strcmp(mac, wanted) == 0)
    {
        peer_ap_ssid_fields(fields, sizeof(fields), ssid);
        peer_reply_ok(reply, "active=1 already=1 %s ch=%d mac=%s count=0",
                      fields, channel, mac);
        return 0;
    }

    if (rt_wlan_ap_is_active())
    {
        if (rt_wlan_ap_stop() != RT_EOK)
        {
            peer_reply_err(reply, PEER_ERR_BUSY, "ap_stop");
            return -1;
        }
    }

    /* Before the AP interface is brought up: the driver's set_mac updates the
     * address that new virtual interfaces inherit. See PEER_AP_DEFAULT_MAC. */
    if (peer_ap_apply_mac(mac_bytes) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_RADIO, "set_mac");
        return -1;
    }

    rt_memset(&info, 0, sizeof(info));
    info.band = RT_802_11_BAND_2_4GHZ;
    info.channel = (rt_int16_t)channel;
    info.hidden = (rt_uint8_t)hidden;
    info.security = keyed ? SECURITY_WPA2_AES_PSK : SECURITY_OPEN;
    info.ssid.len = (rt_uint8_t)ssid_len;
    rt_memcpy(info.ssid.val, ssid, ssid_len);

    /* adv rather than the plain variant: rt_wlan_start_ap() pins channel 6,
     * and a lab with two benches next to each other needs to move. */
    err = rt_wlan_start_ap_adv(&info, keyed ? key : RT_NULL);
    if (err != RT_EOK && !peer_ap_wait_active(PEER_AP_LATE_WAIT_MS))
    {
        peer_reply_err(reply, PEER_ERR_RADIO, "start_ap err=%d", (int)err);
        return -1;
    }

    peer_ap_mac_or_empty(mac, sizeof(mac));

    /* Report the channel the framework actually runs on, not the one that was
     * asked for: a host that pins a channel has to be able to see a peer that
     * ended up somewhere else. */
    actual_channel = channel;
    if (rt_wlan_ap_get_info(&running) == RT_EOK && running.channel > 0)
        actual_channel = running.channel;

    s_last.valid = RT_TRUE;
    rt_snprintf(s_last.ssid, sizeof(s_last.ssid), "%s", ssid);
    rt_snprintf(s_last.key, sizeof(s_last.key), "%s", key);
    s_last.channel = actual_channel;
    s_last.hidden = hidden;

    /* The echo service belongs to "this image is an AP", not to the one start
     * that happened at boot; make sure a start that came back from a failure
     * still leaves it running. */
    peer_echo_ensure();

    /* The event records the transition this op caused. It is emitted before
     * the terminal line, and the reply's count tells the host to consume it.
     * `late=1` says the AP came up after the framework's wait expired, which is
     * slow but correct; a suite does not care, an engineer reading the transcript
     * does. */
    peer_ap_ssid_fields(fields, sizeof(fields), ssid);
    peer_emit_evt("ap_start", "%s ch=%d mac=%s", fields, actual_channel, mac);

    peer_reply_ok(reply,
                  "active=1 already=0 late=%d %s ch=%d mac=%s count=1",
                  err == RT_EOK ? 0 : 1, fields, actual_channel, mac);
    return 0;
}

int peer_ap_stop(peer_reply_t *reply)
{
    char mac[PEER_AP_MAC_LEN];

    if (!rt_wlan_ap_is_active())
    {
        /* Either the AP was never started or it is already down; both are the
         * requested state, and rt_wlan_ap_stop() would answer -RT_EIO for the
         * first case. */
        peer_ap_forget();
        peer_reply_ok(reply, "active=0 already=1 count=0");
        return 0;
    }

    if (rt_wlan_ap_stop() != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_RADIO, "ap_stop");
        return -1;
    }

    peer_ap_forget();
    peer_ap_mac_or_empty(mac, sizeof(mac));
    peer_emit_evt("ap_stop", "mac=%s", mac);

    peer_reply_ok(reply, "active=0 already=0 count=1");
    return 0;
}

int peer_ap_status(peer_reply_t *reply)
{
    struct rt_wlan_info info;
    char mac[PEER_AP_MAC_LEN];
    char ssid[RT_WLAN_SSID_MAX_LENGTH + 1];
    char fields[128];
    rt_bool_t active = rt_wlan_ap_is_active();
    int channel = -1;
    int sta_num = 0;

    peer_ap_mac_or_empty(mac, sizeof(mac));
    ssid[0] = '\0';

    if (active && rt_wlan_ap_get_info(&info) == RT_EOK)
    {
        rt_size_t len = info.ssid.len;

        if (len > RT_WLAN_SSID_MAX_LENGTH)
        {
            len = RT_WLAN_SSID_MAX_LENGTH;
        }
        rt_memcpy(ssid, info.ssid.val, len);
        ssid[len] = '\0';
        channel = info.channel;
    }

    if (active)
    {
        /* Information only: the platform does not forward association events,
         * so this stays 0 even with a station attached. See the file header. */
        sta_num = rt_wlan_ap_get_sta_num();
    }

    peer_ap_ssid_fields(fields, sizeof(fields), ssid);
    peer_reply_ok(reply,
                  "active=%d %s ch=%d mac=%s sta_num=%d mode=%d count=0",
                  active ? 1 : 0, fields, channel, mac, sta_num,
                  (int)rt_wlan_get_mode(RT_WLAN_DEVICE_AP_NAME));
    return 0;
}

int peer_ap_start_simple(const char *ssid, const char *key, const char *mac)
{
    peer_args_t args;
    peer_reply_t reply;

    rt_memset(&args, 0, sizeof(args));
    rt_memset(&reply, 0, sizeof(reply));

    args.items[args.count].key = "ssid";
    args.items[args.count].value = ssid;
    args.count++;
    args.items[args.count].key = "key";
    args.items[args.count].value = (key != RT_NULL) ? key : "";
    args.count++;
    if (mac != RT_NULL && mac[0] != '\0')
    {
        args.items[args.count].key = "mac";
        args.items[args.count].value = mac;
        args.count++;
    }

    /* Reuses the op, so the AP a human sees at boot and the AP a suite raises
     * through `ap.start` are brought up by exactly the same code. */
    if (peer_ap_start(&args, &reply) != 0)
    {
        peer_log("wifi_peer: softap \"%s\" failed: %s %s", ssid,
                 (reply.code != RT_NULL) ? reply.code : "error", reply.text);
        return -1;
    }

    return 0;
}

int peer_ap_autostart(void)
{
    if (peer_ap_start_simple(PEER_AP_SSID, PEER_AP_PASSWORD,
                             PEER_AP_DEFAULT_MAC) != 0)
    {
        peer_log("wifi_peer: softap \"%s\" did not come up at boot; "
                 "a suite can still raise it with ap.start", PEER_AP_SSID);
        return -1;
    }

    peer_log("wifi_peer: softap \"%s\" is up, echo port %d",
             PEER_AP_SSID, PEER_AP_ECHO_PORT);
    return 0;
}
