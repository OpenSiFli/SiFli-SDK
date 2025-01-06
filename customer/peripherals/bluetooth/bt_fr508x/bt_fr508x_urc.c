

#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
#include "str_convert_to_utf8.h"
#include "bt_fr508x_urc.h"

#define DBG_TAG               "fr508x.urc"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

static inline uint16_t biglittle_swap16(uint16_t n)
{
    return (((n & 0xff00) >> 8) | ((n & 0x00ff) << 8));
}

static inline uint32_t biglittle_swap32(uint32_t n)
{
    return ((((uint32_t)(n) & 0xff000000) >> 24) | \
            (((uint32_t)(n) & 0x00ff0000) >> 8) | \
            (((uint32_t)(n) & 0x0000ff00) << 8) | \
            (((uint32_t)(n) & 0x000000ff) << 24));
}



static int check_cpu_endian()
{
    union
    {
        unsigned long int i;
        unsigned char s[4];
    } c;
    c.i = 0x12345678;
    return (0x12 == c.s[0]);
}


uint32_t t_htonl(uint32_t h)
{
    return check_cpu_endian() ? h : biglittle_swap32(h);
}


uint32_t t_ntohl(uint32_t n)
{
    return check_cpu_endian() ? n : biglittle_swap32(n);
}



uint16_t t_htons(uint16_t h)
{
    return check_cpu_endian() ? h : biglittle_swap16(h);
}


uint16_t t_ntohs(uint16_t n)
{
    return check_cpu_endian() ? n : biglittle_swap16(n);
}

static uint32_t mp3_info_strconvert(uint8_t encode, uint8_t *dst, uint16_t max_size, \
                                    uint8_t *src, uint16_t src_size)
{
    //LOG_I("mp3_info_strconvert encode:%d",encode);
    if (0 == encode)
    {
        return str_convert_to_utf8(STR_ENCODE_GBK, dst, max_size, src, src_size);
    }
    else if (1 == encode)
    {
        return str_convert_to_utf8(STR_ENCODE_UTF16_LE, dst, max_size, src + 2, src_size - 2);
    }
    else if (2 == encode)
    {
        return str_convert_to_utf8(STR_ENCODE_UTF16_BE, dst, max_size, src + 2, src_size - 2);
    }
    else if (3 == encode)
    {
        return str_convert_to_utf8(STR_ENCODE_UTF8, dst, max_size, src, src_size);
    }
    return 0;
}

rt_err_t mp3_info_decode(const char *buf, rt_size_t len, bt_mp3_detail_info_t *info)
{
    uint8_t flag = 0x00;
    bt_mp3_header_t mp3_header = {0};
    int tag_header_len = sizeof(bt_mp3_tag_header_t);
    int mp3_header_len = sizeof(bt_mp3_header_t);
    if (len < mp3_header_len)
    {
        LOG_E("mp3_buf_len[%d] error mp3_header_len:%d", len, mp3_header_len);
        return BT_ERROR_PARSING;
    }
    rt_memcpy(&mp3_header, buf, mp3_header_len);
    mp3_header.duration = t_ntohl(mp3_header.duration);
    mp3_header.song_total_size = t_ntohl(mp3_header.song_total_size);
    mp3_header.tag_data_size = t_ntohs(mp3_header.tag_data_size);
    buf += mp3_header_len;
    len -= mp3_header_len;
    if ((mp3_header.tag_data_size < 10) || (mp3_header.tag_data_size - 10 > len))
    {
        LOG_E("parsing tag_data_size[%d] error len:%d", mp3_header.tag_data_size, len);
        return BT_ERROR_PARSING;
    }
    info->duration = mp3_header.duration;
    info->song_total_size = mp3_header.song_total_size;
    do
    {
        bt_mp3_tag_header_t tag_header = {0};
        bt_mp3_tag_vaule_t tag_vaule = {0};
        if (len < tag_header_len)
        {
            // LOG_I("parsing mp3 tag end len:%d tag_header_len:%d", len, tag_header_len);
            break;
        }
        rt_memcpy(&tag_header, buf, tag_header_len);
        tag_header.size = t_ntohl(tag_header.size);
        buf += tag_header_len;
        len -= tag_header_len;
        if (tag_header.size > len)
        {
            // LOG_E("parsing mp3 tag error len:%d tag_size:%d", len, tag_header.size);
            break;
        }

        rt_memcpy(&tag_vaule, buf, 1);
        tag_vaule.data = buf + 1;
        buf += tag_header.size;
        len -= tag_header.size;
#if 0
        LOG_I("parsing mp3 tag vaule frame_id:%c%c%c%c encode:%02X BOM:%02X%02X", tag_header.frame_id[0], tag_header.frame_id[1], \
              tag_header.frame_id[2], tag_header.frame_id[3], tag_vaule.encode, tag_vaule.bom[0], tag_vaule.bom[1]);
#endif
        if (!rt_strncmp(tag_header.frame_id, "TIT2", rt_strlen("TIT2")))
        {
            info->song_name.size = mp3_info_strconvert(tag_vaule.encode, info->song_name.song_name, BT_MAX_SONG_NAME_LEN, \
                                   (uint8_t *)tag_vaule.data, tag_header.size - 1);

            //LOG_I("mp3 song size:%d name:%s",info->song_name.size, info->song_name.song_name);
            flag |= 0x01;
        }
        else if (!rt_strncmp(tag_header.frame_id, "TPE1", rt_strlen("TPE1")))
        {
            info->singer_name.size = mp3_info_strconvert(tag_vaule.encode, info->singer_name.singer_name, BT_MAX_SINGER_NAME_LEN, \
                                     (uint8_t *)tag_vaule.data, tag_header.size - 1);
            //LOG_I("mp3 singer size:%d name:%s",info->singer_name.size,info->singer_name.singer_name);
            flag |= 0x02;
        }
        else if (!rt_strncmp(tag_header.frame_id, "TALB", rt_strlen("TALB")))
        {
            info->album_info.size = mp3_info_strconvert(tag_vaule.encode, info->album_info.album_name, BT_MAX_ALBUM_INFO_LEN, \
                                    (uint8_t *)tag_vaule.data, tag_header.size - 1);
            //LOG_I("mp3 album_info size:%d name:%s",info->album_info.size,info->album_info.album_name);
            flag |= 0x04;
        }

        if (0x07 == flag)
        {
            break;
        }
    }
    while (1);
    LOG_I("parsing mp3 success duration:%d song_total_size:%d", info->duration, info->song_total_size);
    return BT_EOK;
}


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
    LOG_I("URC mt data : %.*s", size, data);
    return;
}

static void urc_func_ms(at_client_t client, const char *data, rt_size_t size)
{
    LOG_I("URC ms data : %.*s", size, data);
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

static void urc_func_mp3(at_client_t client, const char *data, rt_size_t size)
{
    rt_err_t reuslt = BT_EOK;
    rt_size_t offset = 0;
    rt_size_t len = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = rt_strstr(data, "+MP3:");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    offset = resp_line_buf - data;
    len = size - offset;
    bt_mp3_detail_info_t info = {0};
    reuslt = mp3_info_decode(resp_line_buf, len, &info);
    if (BT_EOK != reuslt)
    {
        return;
    }
    bt_notify_t args;
    args.event = BT_EVENT_MP3_DETAIL_INFO;
    args.args = &info;
    rt_bt_event_notify(&args);
    LOG_I("URC mp3 parsing pass");
    return;
}

#ifdef BT_USING_LOCAL_MEDIA_EX
bt_music_list_pageinfo_t g_music_pageinfo = {0};


static bt_err_t music_list_info_decode(const char *buf, rt_size_t size)
{
    bt_err_t ret = BT_EOK;
    const char *line_buf = RT_NULL;
    const char *pTemp = RT_NULL;
    uint32_t count = 0;
    uint32_t offset = 0;
    uint32_t temp_size = 0;
    bt_music_list_info_t list = {0};
    list.info = (bt_music_info_t *)rt_malloc(sizeof(bt_music_info_t) * g_music_pageinfo.num);
    if (RT_NULL == list.info)
    {
        LOG_E("music_list_info_decode malloc fail:%d", g_music_pageinfo.num);
        return RT_ENOMEM;
    }
    rt_memset(list.info, 0x00, sizeof(bt_music_info_t) * g_music_pageinfo.num);
    pTemp = buf;
    do
    {
        line_buf = rt_strstr(pTemp, ",");    /* music id */
        if (RT_NULL == line_buf)
        {
            LOG_E("music_list_info_decode music id fail");
            ret = BT_ERROR_PARSING;
            goto exit;
        }
        list.info[count].music_id = atoi(pTemp);
        pTemp = line_buf + 1;

        line_buf = rt_strstr(pTemp, ",");    /* music song len */
        if (RT_NULL == line_buf)
        {
            LOG_E("music_list_info_decode music song len fail");
            ret = BT_ERROR_PARSING;
            goto exit;
        }
        temp_size = strtol(pTemp, RT_NULL, 16);
        pTemp = line_buf + 1;       /* music song name */
        offset = pTemp - buf;
        if ((size - offset) < temp_size)
        {
            LOG_E("music_list_info_decode music song name fail:size:%d file_size:%d offset:%d", size, temp_size, offset);
            ret = BT_ERROR_PARSING;
            goto exit;
        }
#if 1
        //list.info[count].song_name.size = temp_size;
        //rt_memcpy(list.info[count].song_name.song_name, pTemp, list.info[count].song_name.size);
        list.info[count].song_name.size = mp3_info_strconvert(0, list.info[count].song_name.song_name, \
                                          BT_MAX_SONG_NAME_LEN, (uint8_t *)pTemp, temp_size);
#endif
        pTemp += temp_size;
        LOG_I("music_len:%d music_name:%s ", list.info[count].song_name.size, list.info[count].song_name.song_name);

        pTemp += 1;               /* music singer name len */
        line_buf = rt_strstr(pTemp, ",");
        if (RT_NULL == line_buf)
        {
            LOG_E("music_list_info_decode music singer len fail");
            ret = BT_ERROR_PARSING;
            goto exit;
        }
        temp_size = strtol(pTemp, RT_NULL, 16);

        pTemp = line_buf + 1;               /* music singer name encode 0:GBK 1:utf16 3:utf8 */
        line_buf = rt_strstr(pTemp, ",");
        if (RT_NULL == line_buf)
        {
            LOG_E("music_list_info_decode music singer encode fail");
            ret = BT_ERROR_PARSING;
            goto exit;
        }
        uint16_t encode = strtol(pTemp, RT_NULL, 16);
        uint16_t encode_size = line_buf - pTemp;

        pTemp = line_buf + 1;               /* music singer name */
        offset = pTemp - buf;
        if ((size - offset) < temp_size)
        {
            LOG_E("music_list_info_decode music singer name fail:size:%d singer_size:%d offset:%d", size, temp_size, offset);
            ret = BT_ERROR_PARSING;
            goto exit;
        }
        temp_size -= encode_size;
        list.info[count].singer_name.size = mp3_info_strconvert(encode, list.info[count].singer_name.singer_name, \
                                            BT_MAX_SINGER_NAME_LEN, (uint8_t *) pTemp, temp_size);
        LOG_I("singer_size:%d encode:%d,name:%s", temp_size, encode, list.info[count].singer_name.singer_name);
        pTemp += temp_size;  /* music info end */
        line_buf = rt_strstr(pTemp, "\r\n");
        if (RT_NULL == line_buf)
        {
            LOG_E("music_list_info_decode end fail");
            ret = BT_ERROR_PARSING;
            goto exit;
        }
        count++;
        pTemp = line_buf + 2;
        offset = pTemp - buf;
        if (offset == size)
        {
            ret = BT_EOK;
            break;
        }

    }
    while (1);
exit:
    if (BT_EOK == ret)
    {
        bt_notify_t args;
        list.num = count;
        args.event = BT_EVENT_LOCAL_MUSIC_LIST_INFO;
        args.args = &list;
        rt_bt_event_notify(&args);
    }
    rt_free(list.info);
    return ret;
}

static bt_err_t music_list_info_read(at_client_t client, char *buf, uint32_t read_size)
{
    rt_size_t real_size = 0;
    real_size = at_client_obj_recv(client, buf, read_size, 200);
    if (read_size != real_size)
    {
        LOG_E("music_list_info_read need_size:%d real_size:%d", read_size, real_size);
        return BT_ERROR_PARSING;
    }

    return BT_EOK;
}
static void urc_func_music_info(at_client_t client, const char *data, rt_size_t size)
{
    rt_err_t reuslt = BT_EOK;
    uint32_t len = 0;
    uint32_t read_size = 0;
    len = strtol(data + 4, RT_NULL, 16);
    char *buf = (char *)rt_malloc(len - 9);
    if (RT_NULL == buf)
    {
        LOG_E("music_list_info_read malloc fail");
        return;
    }
    read_size = size - 10;
    rt_memcpy(buf, data + 10, read_size);
    if (BT_EOK !=  music_list_info_read(client, buf + read_size, len - size))
    {
        rt_free(buf);
        return;
    }
    music_list_info_decode(buf, len - 10);
    rt_free(buf);
    return;
}
#endif


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
    {"+MP3",        "\r\n",           urc_func_mp3},
    {"+PROG",        "\r\n",          urc_func_prog},
    //{"+MS",         "\r\n",           urc_func_ms},
    //{"+MT",         "\r\n",           urc_func_mt},
    //{"+MC",         "\r\n",           urc_func_mc},
    //{"+MN",         "\r\n",           urc_func_mn},
    {"+VOL",        "\r\n",           urc_func_vol},
    {"+PROG",       "\r\n",           urc_func_prog},
#ifdef BT_USING_LOCAL_MEDIA_EX
    {"+MD",       "\r\n",           urc_func_music_info},
#endif
    {"+STATE",       "\r\n",           urc_func_state},

    // {"+PEC",        "\r\n",           urc_func_pec},
    // {"+PEA",        "\r\n",           urc_func_pea},
    // {"+PP",         "\r\n",           urc_func_pp},
};

void fr508x_set_urc(at_client_t client)
{
    at_obj_set_urc_table(client, urc_table, sizeof(urc_table) / sizeof(urc_table[0]));
    return;
}



