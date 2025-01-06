

#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
#include "str_convert.h"
#include "bt_5376a2_urc.h"

#define DBG_TAG               "bt_5376.urc"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>


static void urc_func_inq(at_client_t client, const char *data, rt_size_t size)
{
    int reuslt = 0;
    const char *resp_line_buf = RT_NULL;
    const char *pTemp = RT_NULL;
    bt_serached_device_info_t bt_device_info = {0};
    LOG_I("URC inq data:%s", data);
    resp_line_buf = rt_strstr(data, "+INQ:");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    resp_line_buf += 5;
    reuslt = sscanf(resp_line_buf, "%02X%02X%02X%02X%02X%02X", (uint32_t *)&bt_device_info.mac_addr.addr[0], \
                    (uint32_t *)&bt_device_info.mac_addr.addr[1], \
                    (uint32_t *)&bt_device_info.mac_addr.addr[2], \
                    (uint32_t *)&bt_device_info.mac_addr.addr[3], \
                    (uint32_t *)&bt_device_info.mac_addr.addr[4], \
                    (uint32_t *)&bt_device_info.mac_addr.addr[5]);
    if (BT_MAX_MAC_LEN != reuslt)
    {
        return;
    }
    resp_line_buf += BT_MAX_MAC_LEN * 2;
    pTemp = resp_line_buf;
    resp_line_buf = rt_strstr(pTemp, ",");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    resp_line_buf += 1;
    pTemp = resp_line_buf;
    resp_line_buf = rt_strstr(pTemp, ",");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    reuslt = sscanf(pTemp, "%*[^=]=%d[^,]", &bt_device_info.rssi); // rssi
    if (reuslt <= 0)
    {
        return;
    }
    resp_line_buf += 1;
    pTemp = resp_line_buf;
    resp_line_buf = rt_strstr(pTemp, "\r\n");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    reuslt = sscanf(pTemp, "%d", &bt_device_info.name_size); // name_size
    if (reuslt <= 0)
    {
        return;
    }

    if (bt_device_info.name_size >= client->recv_bufsz)
    {
        LOG_E("inq name_size:%d out of buf size:%d ", bt_device_info.name_size, client->recv_bufsz);
        return;
    }
    rt_memset(client->recv_line_buf, 0x00, client->recv_bufsz);
    client->recv_line_len = 0;
    client->recv_line_len = at_client_obj_recv(client, client->recv_line_buf, bt_device_info.name_size, 300);
    if (0 == client->recv_line_len || (client->recv_line_len != bt_device_info.name_size))
    {
        LOG_E("inq rev name data error rev_size:%d name_size:%d ", client->recv_line_len, bt_device_info.name_size);
        return;
    }
#if 0
    resp_line_buf = rt_strstr(client->recv_line_buf, "\r\n");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
#endif
    bt_device_info.bt_name = client->recv_line_buf;
    //bt_device_info.name_size = resp_line_buf - client->recv_line_buf;
    bt_notify_t args;
    args.event = BT_EVENT_INQ;
    args.args = &bt_device_info;
    rt_bt_event_notify(&args);
    LOG_I("URC inq data : rssi:%d  name_size:%d name:%s ", bt_device_info.rssi, bt_device_info.name_size, bt_device_info.bt_name);
    return;
}

static void urc_func_conn(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    args.event = BT_EVENT_CONNECT_COMPLETE;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    LOG_I("URC conn : %.*s", size, data);
    return;
}


static void urc_func_disc(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    bt_disconnect_reason_t reason = BT_DISC_OTHER;
    args.event = BT_EVENT_DISCONNECT;
    args.args = RT_NULL;
    LOG_I("URC disc data : %.*s", size, data);
    if (strstr(data, "+DISC:LL"))
    {
        reason = BT_DISC_CONNECT_TIMOUT;
    }
    else if (strstr(data, "+DISC:UT"))
    {
        reason = BT_DISC_USER_BREAK;
    }
    else if (strstr(data, "+DISC:LT"))
    {
        reason = BT_DISC_LOCAL_BREAK;
    }
    else if (strstr(data, "+DISC:OT"))
    {
        reason = BT_DISC_OTHER;
    }
    args.args = &reason;
    rt_bt_event_notify(&args);
    return;
}

static void urc_func_acc(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    bt_access_mode_t access_mode = BT_ACCESS_MODE_INACCESSIABLE;
    args.event = BT_EVENT_ACCESS_MODE_CHANGE;
    args.args = RT_NULL;
    LOG_I("URC acc data : %.*s", size, data);
    if (rt_strstr(data, "+ACC:II"))
    {
        access_mode = BT_ACCESS_MODE_GENERAL;
    }
    else if (rt_strstr(data, "+ACC:IJ"))
    {
        access_mode = BT_ACCESS_MODE_CONNECT_ABLE;
    }
    else if (rt_strstr(data, "+ACC:IK"))
    {
        access_mode = BT_ACCESS_MODE_INACCESSIABLE;
    }
    args.args = &access_mode;
    rt_bt_event_notify(&args);
    return;
}

static void urc_func_ac(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = rt_strstr(data, "AP");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    args.event = BT_EVENT_CALL_lINK_ESTABLISHED;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    LOG_I("URC ap data : %.d buf:%s", size, data);
    return;
}

static void urc_func_ad(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = rt_strstr(data, "AD");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    args.event = BT_EVENT_CALL_LINK_DOWN;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    LOG_I("URC ad data : %d buf:%s", size, data);
    return;
}

static void urc_func_ring(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    args.event = BT_EVENT_CALL_RING;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    LOG_I("URC ring data : %.*s", size, data);
    rt_size_t len = rt_strlen(data);
    len += 1;
    if (size <= len)
    {
        urc_func_ac(client, data, size);
        urc_func_ad(client, data, size);
        return;
    }
    urc_func_ac(client, data + len, size - len);
    urc_func_ad(client, data + len, size - len);

    return;
}

static void urc_func_cil(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    phone_number_t number;
    args.event = BT_EVENT_CALL_NUMBER;
    if (size <= 10)
    {
        LOG_E("URC cli data error len:%d buf:%s", size, data);
        return;
    }
    number.number = data + 5;
    number.size = size - 7;
    args.args = &number;
    rt_bt_event_notify(&args);

    LOG_I("URC cli len:%d buf:%s", number.size, number.number);
    rt_size_t len = rt_strlen(data);
    len += 1;
    if (size <= len)
    {
        urc_func_ac(client, data, size);
        urc_func_ad(client, data, size);
        return;
    }
    urc_func_ac(client, data + len, size - len);
    urc_func_ad(client, data + len, size - len);
    return;
}

static void urc_func_mt(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    uint8_t status = 0;
    args.event = BT_EVENT_MUSIC_PLAY_STATUS_CHANGED;
    args.args = &status;
    rt_bt_event_notify(&args);
    return;
}

static void urc_func_ms(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    uint8_t status = 1;
    args.event = BT_EVENT_MUSIC_PLAY_STATUS_CHANGED;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    return;
}

static void urc_func_mc(at_client_t client, const char *data, rt_size_t size)
{
    LOG_I("URC mc data : %.*s", size, data);
    return;
}


static void urc_func_mn(at_client_t client, const char *data, rt_size_t size)
{
    LOG_I("URC mn data : %.*s", size, data);
    return;
}

static void urc_func_vol(at_client_t client, const char *data, rt_size_t size)
{
    int reuslt = 0;
    uint32_t volume = 0;
    reuslt = sscanf(data, "+VOL:%02X", (uint32_t *)&volume);
    if (1 != reuslt)
    {
        return;
    }
    bt_notify_t args;
    bt_volume_t vol;
    vol.media_volume = (volume * 100) / 63;
    vol.call_volume = vol.media_volume;
    args.event = BT_EVENT_VOL_CHANGED;
    args.args = &vol;
    rt_bt_event_notify(&args);
    LOG_I("URC vol data : %s volume:%d", data, volume);
    return;
}

static void urc_func_prog(at_client_t client, const char *data, rt_size_t size)
{
    rt_err_t reuslt = BT_EOK;
    rt_size_t offset = 0;
    rt_size_t len = 0;
    uint32_t prog = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = rt_strstr(data, "+PROG:");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    prog = strtol(resp_line_buf + 6, RT_NULL, 16);
    bt_notify_t args;
    args.event = BT_EVENT_SONG_PLAY_PROGRESS;
    args.args = &prog;
    rt_bt_event_notify(&args);
    // LOG_I("URC prog:%08X", prog);
    return;
}
static void urc_func_pea(at_client_t client, const char *data, rt_size_t size)
{
    RT_ASSERT(data && size);
    LOG_I("URC pea data : %.*s", size, data);
    return;
}
static void urc_func_pec(at_client_t client, const char *data, rt_size_t size)
{
    RT_ASSERT(data && size);
    LOG_I("URC pec data : %.*s", size, data);
    return;
}

static void urc_func_pp(at_client_t client, const char *data, rt_size_t size)
{
    RT_ASSERT(data && size);
    LOG_I("URC pp data : %.*s", size, data);
    return;
}


static void urc_func_state(at_client_t client, const char *data, rt_size_t size)
{
    rt_err_t reuslt = BT_EOK;
    rt_size_t offset = 0;
    rt_size_t len = 0;
    uint32_t state = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = rt_strstr(data, "+STATE:");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    state = strtol(resp_line_buf + 7, RT_NULL, 10);
    state += BT_STATE_PAIR;
    bt_notify_t args;
    args.event = BT_EVENT_STATE;
    args.args = &state;
    rt_bt_event_notify(&args);
    LOG_I("urc_func_state:%d", state);
    return;
}

static void urc_func_power_off(at_client_t client, const char *data, rt_size_t size)
{
    bt_notify_t args;
    args.event = BT_EVENT_POWER_OFF;
    args.args = RT_NULL;
    rt_bt_event_notify(&args);
    return;
}

static const struct at_urc urc_table[] =
{
    {"+INQ",        "\r\n",           urc_func_inq},
    {"+CONN",       "\r\n",           urc_func_conn},
    {"+DISC",       "\r\n",           urc_func_disc},
    {"+ACC",        "\r\n",           urc_func_acc},
    {"+RING",       "\r\n",           urc_func_ring},
    {"+CLI",        "\r\n",           urc_func_cil},
    {"+AP",         "\r\n",           urc_func_ac},
    {"+AD",         "\r\n",           urc_func_ad},
    {"+PROG",        "\r\n",          urc_func_prog},
    {"+MS",         "\r\n",           urc_func_ms},
    {"+MT",         "\r\n",           urc_func_mt},
    //{"+MC",         "\r\n",           urc_func_mc},
    //{"+MN",         "\r\n",           urc_func_mn},
    {"+VOL",        "\r\n",           urc_func_vol},
    {"+PROG",       "\r\n",           urc_func_prog},
    {"+STATE",       "\r\n",           urc_func_state},
    {"func_pwroff",  "\r\n",           urc_func_power_off}
    // {"+PEC",        "\r\n",           urc_func_pec},
    // {"+PEA",        "\r\n",           urc_func_pea},
    // {"+PP",         "\r\n",           urc_func_pp},
};

void bt_5376_set_urc(at_client_t client)
{
    at_obj_set_urc_table(client, urc_table, sizeof(urc_table) / sizeof(urc_table[0]));
    return;
}



