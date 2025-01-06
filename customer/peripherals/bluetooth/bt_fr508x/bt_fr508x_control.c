

#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
#include "bt_fr508x_control.h"
#include "bt_fr508x.h"

#define DBG_TAG               "fr508x.control"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>


extern rt_err_t mp3_info_decode(const char *buf, rt_size_t len, bt_mp3_detail_info_t *info);
extern uint16_t t_ntohs(uint16_t n);



static int fr508x_at_ac_response(at_response_t resp, void *args)
{
    rt_size_t offset = 0;
    rt_size_t len = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+MP3:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    return BT_EOK;
}

static int fr508x_at_ab_response(at_response_t resp, void *args)
{
    rt_size_t offset = 0;
    rt_size_t len = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+MP3:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    return BT_EOK;
}

static int fr508x_at_ae_response(at_response_t resp, void *args)
{
    rt_size_t offset = 0;
    rt_size_t len = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+MP3:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    return BT_EOK;
}

#if 0
static int fr508x_at_ch_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    char mac[12], name[128];
    int signal_strength, name_len;
    reuslt = at_resp_parse_line_args(resp, 1, "+INQ:%s%d%d%s", mac, signal_strength, name_len, name);
    if (reuslt)
    {
    }
    return reuslt;
}
#endif
static int fr508x_at_cl_response(at_response_t resp, void *args)
{
    int reuslt = 0;
    int state = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+STATE:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }

    bt_state_t *pState = (bt_state_t *)args;
    reuslt = sscanf(resp_line_buf, "+STATE:%d", &state);
    if (reuslt != 1)
    {
        return BT_ERROR_PARSING;
    }
    *pState = state + BT_STATE_PAIR;
    LOG_I("get bt state:%d", *pState);
    return BT_EOK;
}

static int fr508x_at_cs_response(at_response_t resp, void *args)
{
    int reuslt = 0;
    uint32_t hf_volume = 0;
    uint32_t ad_volume = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, " ");
    if (RT_NULL == resp_line_buf)
    {
        LOG_E("fr508x_at_cs err no space");
        return BT_ERROR_PARSING;
    }
    reuslt = sscanf(resp_line_buf - 2, "%02X %02X", (uint32_t *)&hf_volume, \
                    (uint32_t *)&ad_volume);

    if (2 != reuslt)
    {
        LOG_E("fr508x_at_cs err reuslt:%d", reuslt);
        return BT_ERROR_PARSING;
    }

    if (hf_volume > 0x3F || ad_volume > 0x3F)
    {
        LOG_E("fr508x_at_cs err hf:%02X ad:%02X", hf_volume, ad_volume);
        return BT_ERROR_PARSING;
    }
    bt_volume_t *volume = (bt_volume_t *)args;
    volume->media_volume = (ad_volume * 100) / 63;
    volume->call_volume = (hf_volume * 100) / 63;
    LOG_I("fr508x_at_cs hf:%02X ad:%02X", hf_volume, ad_volume);
    return BT_EOK;
}

static int fr508x_at_cm_response(at_response_t resp, void *args)
{
    int reuslt = 0;
    int state = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+COD:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    bt_peer_deviceinfo_t *info = (bt_peer_deviceinfo_t *)args;
    reuslt = sscanf(resp_line_buf + 5, "%02X%02X%02X%02X%02X%02X %d", (uint32_t *)&info->mac.addr[0], \
                    (uint32_t *)&info->mac.addr[1], \
                    (uint32_t *)&info->mac.addr[2], \
                    (uint32_t *)&info->mac.addr[3], \
                    (uint32_t *)&info->mac.addr[4], \
                    (uint32_t *)&info->mac.addr[5], \
                    (int *)&info->type);
    if ((BT_MAX_MAC_LEN + 1) != reuslt)
    {
        return BT_ERROR_PARSING;
    }

    LOG_I("cm mac:%02X:%02X:%02X:%02X:%02X:%02X type:%d", info->mac.addr[0], \
          info->mac.addr[1], \
          info->mac.addr[2], \
          info->mac.addr[3], \
          info->mac.addr[4], \
          info->mac.addr[5], \
          info->type);
    return BT_EOK;
}

static int fr508x_at_fa_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    rt_size_t offset = 0;
    rt_size_t len = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+MP3:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }

    offset = resp_line_buf - resp->buf;
    len = resp->buf_len - offset;
    bt_mp3_info_t *mp3_info = (bt_mp3_info_t *)args;
    bt_mp3_detail_info_t *info = (bt_mp3_detail_info_t *)mp3_info->args;
    reuslt = mp3_info_decode(resp_line_buf, len, info);
    return reuslt;
}

static int fr508x_at_fb_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    rt_size_t offset = 0;
    rt_size_t len = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+NAME:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    offset = resp_line_buf - resp->buf;
    len = resp->buf_len - offset;
    if (len < sizeof(bt_mp3_brief_header_t))
    {
        LOG_E("parsing mp3 brief header error len:%d tag_size:%d", len, sizeof(bt_mp3_brief_header_t));
        return BT_ERROR_PARSING;
    }
    bt_mp3_brief_header_t mp3_brief_header = {0};
    rt_memcpy(&mp3_brief_header, resp_line_buf, sizeof(bt_mp3_brief_header_t));
    resp_line_buf += sizeof(bt_mp3_brief_header_t);
    len -= sizeof(bt_mp3_brief_header_t);
    mp3_brief_header.file_name_size = t_ntohs(mp3_brief_header.file_name_size);
    if (len < mp3_brief_header.file_name_size)
    {
        LOG_E("parsing mp3 brief file size error len:%d file_size:%d", len, mp3_brief_header.file_name_size);
        return BT_ERROR_PARSING;
    }
    bt_mp3_info_t *mp3_info = (bt_mp3_info_t *)args;
    bt_mp3_brief_info_t *info = (bt_mp3_brief_info_t *)mp3_info->args;
    info->song_name.size = mp3_brief_header.file_name_size;
    rt_memcpy(info->song_name.song_name, resp_line_buf, mp3_brief_header.file_name_size);
    LOG_I("parsing mp3 brief success file name:%s", info->song_name.song_name);
    return reuslt;
}

static int fr508x_at_fc_response(at_response_t resp, void *args)
{
    int reuslt = 0;
    int num = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+MNUM:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    num = strtol(resp_line_buf + 6, RT_NULL, 16);
    uint32_t *pNum = (uint32_t *)args;
    *pNum = num;
    LOG_I("parsing mp3 total num:%d", num);
    return BT_EOK;
}

static int fr508x_at_pt_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    uint8_t state;
    reuslt = at_resp_parse_line_args(resp, 1, "+PEC:%d", state);
    return reuslt;
}
static int fr508x_at_pp_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    return reuslt;
}

static int fr508x_at_fd_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_strstr(resp->buf, resp->buf_len, "+MAC:");
    if (RT_NULL == resp_line_buf)
    {
        return RT_ERROR;
    }
    bt_mac_t *addr = (bt_mac_t *)args;
    reuslt = sscanf(resp_line_buf + 5, "%02X%02X%02X%02X%02X%02X", (uint32_t *)&addr->addr[0], \
                    (uint32_t *)&addr->addr[1], \
                    (uint32_t *)&addr->addr[2], \
                    (uint32_t *)&addr->addr[3], \
                    (uint32_t *)&addr->addr[4], \
                    (uint32_t *)&addr->addr[5]);
    if (BT_MAX_MAC_LEN != reuslt)
    {
        return BT_ERROR_PARSING;
    }
    LOG_I("mac:%02X:%02X:%02X:%02X:%02X:%02X", addr->addr[0], \
          addr->addr[1], \
          addr->addr[2], \
          addr->addr[3], \
          addr->addr[4], \
          addr->addr[5]);
    return BT_EOK;
}

static int fr508x_at_fe_response(at_response_t resp, void *args)
{
    int reuslt = BT_EOK;
    return reuslt;
}


static void fr508x_wake_up(void)
{
#if (-1 != MCU_WAKEUP_BT_PIN)
    rt_pin_mode(MCU_WAKEUP_BT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_WAKEUP_BT_PIN, PIN_HIGH);
    rt_pin_write(MCU_WAKEUP_BT_PIN, PIN_LOW);
    rt_thread_delay(rt_tick_from_millisecond(100));
    rt_pin_write(MCU_WAKEUP_BT_PIN, PIN_HIGH);
    rt_thread_delay(rt_tick_from_millisecond(20));
#endif
    return;
}

static void fr508x_enable_level_shift(void)
{
#if (-1 != BT_LEVEL_SHIFT_PIN)
    rt_pin_mode(BT_LEVEL_SHIFT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BT_LEVEL_SHIFT_PIN, PIN_HIGH);
    rt_thread_delay(rt_tick_from_millisecond(5));
#endif
    return;
}

static void fr508x_disable_level_shift(void)
{
#if (-1 != BT_LEVEL_SHIFT_PIN)
    rt_pin_mode(BT_LEVEL_SHIFT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BT_LEVEL_SHIFT_PIN, PIN_LOW);
#endif
    return;
}

static void fr508x_send_ex(char *buf, rt_size_t len)
{
    at_client_t client = at_client_get(BT_UART_NAME);
    fr508x_wake_up();
    at_client_obj_send(client, buf, len);
    return;
}


static int fr508x_cmd_control(int cmd, rt_size_t line_num, const char *param, rt_size_t size, void *args)
{
    int result = BT_EOK;
    at_client_t client = at_client_get(BT_UART_NAME);
    for (uint8_t i = 0; i < 2; i++)
    {
        fr508x_wake_up();
        result = at_client_cmd_control(client, cmd, line_num, param, size, args);
        if (BT_EOK == result)
        {
            break;
        }
    }
    if (-RT_ETIMEOUT == result)
    {
        result = BT_ERROR_DEVICE_EXCEPTION;
    }
    return result;
}

static void fr508x_client_set_baud(uint32_t baud_rate)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    at_client_t at_client = at_client_get(BT_UART_NAME);
    RT_ASSERT(at_client != RT_NULL);
    config.baud_rate = baud_rate;
    rt_device_control(at_client->device, RT_DEVICE_CTRL_CONFIG, &config);
    return;
}

static bt_err_t fr508x_open(void)
{
    bt_err_t ret = BT_EOK;
    bt_state_t state = BT_STATE_POWER_OFF;
    fr508x_client_set_baud(BT_UART_BAUD);
    fr508x_enable_level_shift();
    fr508x_wake_up();
    ret = fr508x_cmd_control(BT_CONTROL_QUERY_STATE, 0, RT_NULL, 0, &state);
    if (BT_EOK != ret)
    {
#ifdef BT_SHARING_LOG_UART
#ifdef SOC_SF32LB58X
        fr508x_client_set_baud(3000000);
#else
        fr508x_client_set_baud(1000000);
#endif
#endif
    }
    return ret;
}

static bt_err_t fr508x_close(void)
{
    bt_err_t ret = BT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    LOG_I("fr508x_close:begin");
    ret = fr508x_cmd_control(BT_CONTROL_CLOSE_DEVICE, 0, RT_NULL, 0, RT_NULL);
    LOG_I("fr508x_close:end:%d", ret);
    if (BT_EOK == ret)
    {
        //fr508x_pin_deinit();
        fr508x_disable_level_shift();
#ifdef BT_SHARING_LOG_UART
#ifdef SOC_SF32LB58X
        fr508x_client_set_baud(3000000);
#else
        fr508x_client_set_baud(1000000);
#endif
#endif
    }
    return ret;
}

#ifdef BT_USING_DTMF
static bt_err_t fr508x_dtmf_dial(int cmd, void *args)
{
    bt_dtmf_key_t *key = (bt_dtmf_key_t *)args;
    char key_str[2] = {0};
    switch (*key)
    {
    case BT_DTMF_KEY_0:
        key_str[0] = '0';
        break;

    case BT_DTMF_KEY_1:
        key_str[0] = '1';
        break;

    case BT_DTMF_KEY_2:
        key_str[0] = '2';
        break;

    case BT_DTMF_KEY_3:
        key_str[0] = '3';
        break;

    case BT_DTMF_KEY_4:
        key_str[0] = '4';
        break;

    case BT_DTMF_KEY_5:
        key_str[0] = '5';
        break;

    case BT_DTMF_KEY_6:
        key_str[0] = '6';
        break;

    case BT_DTMF_KEY_7:
        key_str[0] = '7';
        break;

    case BT_DTMF_KEY_8:
        key_str[0] = '8';
        break;

    case BT_DTMF_KEY_9:
        key_str[0] = '9';
        break;

    case BT_DTMF_KEY_STAR:
        key_str[0] = '*';
        break;

    case BT_DTMF_KEY_HASH:
        key_str[0] = '#';
        break;

    default:
        return BT_ERROR_INPARAM;
    }
    return fr508x_cmd_control(cmd, 0, key_str, 1, RT_NULL);
}
#endif

#ifdef BT_USING_MIC_MUTE
static bt_err_t fr508x_set_mute(int cmd, void *args)
{
    bt_mic_mute_t *state = (bt_mic_mute_t *)args;
    char state_str[2] = {0};
    switch (*state)
    {
    case BT_MIC_MUTE_ENABLE:
        state_str[0] = '0';
        break;

    case BT_MIC_MUTE_DISABLE:
        state_str[0] = '1';
        break;

    default:
        return BT_ERROR_INPARAM;
    }
    return fr508x_cmd_control(cmd, 0, state_str, 1, RT_NULL);
}

static int fr508x_get_mute_response(at_response_t resp, void *args)
{
    int reuslt = 0;
    int state = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_resp_get_line_by_kw(resp, "+MIC:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    bt_mic_mute_t *pState = (bt_mic_mute_t *)args;
    reuslt = sscanf(resp_line_buf, "+MIC:%d", &state);
    if (reuslt != 1)
    {
        return BT_ERROR_PARSING;
    }
    *pState = state;
    LOG_I("get mute state:%d", *pState);
    return BT_EOK;
}
#endif

#ifdef BT_USING_DEVICE_TYPE
static bt_err_t fr508x_set_device_type(int cmd, void *args)
{
    bt_device_type_t *state = (bt_device_type_t *)args;
    char state_str[2] = {0};
    switch (*state)
    {
    case BT_DEVICE_TYPE_PHONE:
        state_str[0] = '0';
        break;

    case BT_DEVICE_TYPE_EARPHONE:
        state_str[0] = '1';
        break;

    default:
        return BT_ERROR_INPARAM;
    }
    return fr508x_cmd_control(cmd, 0, state_str, 1, RT_NULL);

}
#endif

#ifdef BT_USING_LOCAL_MEDIA_EX
extern bt_music_list_pageinfo_t g_music_pageinfo;
static bt_err_t fr508x_del_local_music(int cmd, void *args)
{
    uint32_t *id = (uint32_t *)args;
    char id_str[16] = {0};
    rt_sprintf(id_str, "%x", *id);
    return fr508x_cmd_control(cmd, 0, id_str, rt_strlen(id_str), RT_NULL);
}

static bt_err_t fr508x_set_local_play_mode(int cmd, void *args)
{
    bt_music_play_mode_t *mode = (bt_music_play_mode_t *)args;
    char mode_str[8] = {0};
    rt_sprintf(mode_str, "%d", *mode);
    return fr508x_cmd_control(cmd, 0, mode_str, rt_strlen(mode_str), RT_NULL);
}

static bt_err_t fr508x_play_local_assigned_music(int cmd, void *args)
{
    uint32_t *id = (uint32_t *)args;
    char id_str[16] = {0};
    rt_sprintf(id_str, "%x", *id);
    return fr508x_cmd_control(cmd, 0, id_str, rt_strlen(id_str), RT_NULL);
}

static bt_err_t fr508x_play_local_get_music_list(int cmd, void *args)
{
    bt_music_list_pageinfo_t *page_info = (bt_music_list_pageinfo_t *)args;
    char tmp_str[16] = {0};
    rt_memcpy(&g_music_pageinfo, page_info, sizeof(bt_music_list_pageinfo_t));
    rt_sprintf(tmp_str, "%04d_%04X", page_info->offset, page_info->num);
    return fr508x_cmd_control(cmd, 0, tmp_str, rt_strlen(tmp_str), RT_NULL);
}

static int fr508x_at_me_response(at_response_t resp, void *args)
{
    int reuslt = 0;
    int music_id = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = at_resp_get_line_by_kw(resp, "+ME:");
    if (RT_NULL == resp_line_buf)
    {
        return BT_ERROR_PARSING;
    }
    music_id = strtol(resp_line_buf + 4, RT_NULL, 16);
    uint32_t *pNum = (uint32_t *)args;
    *pNum = music_id;
    LOG_I("parsing music_id:%d", music_id);
    return BT_EOK;
}
#endif

static bt_err_t fr508x_query_state_nonblock()
{
    bt_err_t ret = BT_EOK;
    char buf[16] = {"AT#CL\r\n"};
    fr508x_send_ex(buf, rt_strlen(buf));
    return ret;
}

static void fr508x_resume(struct rt_bt_device *bt_handle)
{
    HAL_PIN_Set(PAD_PA49, DBG_DO8, PIN_NOPULL, 1);      //set to uart mode
    HAL_PIN_Set(PAD_PA51, USART1_RXD, PIN_PULLUP, 1);
    return;
}
static void fr508x_suspend(struct rt_bt_device *bt_handle)
{
    HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_NOPULL, 1);     // set to GPIO MODE
    HAL_PIN_Set(PAD_PA51, GPIO_A51, PIN_NOPULL, 1);
    return;
}


bt_err_t fr508x_control(struct rt_bt_device *bt_handle, int cmd, void *args)
{
    bt_err_t ret = BT_EOK;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RESUME:
    {
        /*TODO*/
        fr508x_enable_level_shift();
        fr508x_resume(bt_handle);
    }
    break;

    case RT_DEVICE_CTRL_SUSPEND:
    {
        /*TODO*/
        fr508x_suspend(bt_handle);
    }
    break;

    case BT_CONTROL_DEVICE_INIT:
    {
        /*TODO*/
    }
    break;

    case BT_CONTROL_DEVICE_DEINIT:
    {
        /*TODO*/
    }
    break;

    case RT_DEVICE_CTRL_CONFIG:
    {
        at_client_t at_client = at_client_get(BT_UART_NAME);
        rt_device_control(at_client->device, RT_DEVICE_CTRL_CONFIG, args);
    }
    break;


    case BT_CONTROL_MAKE_CALL:
    {
        phone_number_t *pNumber = (phone_number_t *)args;
        ret = fr508x_cmd_control(cmd, 0, pNumber->number, pNumber->size, args);
    }
    break;


    case BT_CONTROL_CONNECT_DEVICE:
    {
        bt_mac_t *mac = (bt_mac_t *)args;
        char buf[BT_MAX_MAC_LEN * 2 + 1] = {0};
        rt_snprintf(buf, BT_MAX_MAC_LEN * 2 + 1, "%02X%02X%02X%02X%02X%02X", mac->addr[0], \
                    mac->addr[1], \
                    mac->addr[2], \
                    mac->addr[3], \
                    mac->addr[4], \
                    mac->addr[5]);
        ret = fr508x_cmd_control(cmd, 0, buf, BT_MAX_MAC_LEN * 2, args);
    }
    break;


    case BT_CONTROL_QUERY_STATE:
    {
        ret = fr508x_cmd_control(cmd, 0, RT_NULL, 0, args);
        if (BT_EOK == ret)
        {
            bt_state_t *pState = (bt_state_t *)args;
            bt_handle->status.last_state = bt_handle->status.cur_state;
            bt_handle->status.cur_state = *pState;
        }
    }
    break;

    case BT_CONTROL_QUERY_STATE_NONBLOCK:
    {
        ret = fr508x_query_state_nonblock();
    }
    break;


    case BT_CONTROL_GET_LOCAL_SONG_DETAILS:
    {
        char buf[3] = {0};
        bt_mp3_info_t *mp3_info = (bt_mp3_info_t *)args;
        rt_snprintf(buf, sizeof(buf), "%02d", mp3_info->dir);
        ret = fr508x_cmd_control(cmd, 0, buf, 2, args);
    }
    break;


    case BT_CONTROL_GET_LOCAL_SONG_BRIEF:
    {
        char buf[3] = {0};
        bt_mp3_info_t *mp3_info = (bt_mp3_info_t *)args;
        rt_snprintf(buf, sizeof(buf), "%02d", mp3_info->dir);
        ret = fr508x_cmd_control(cmd, 0, buf, 2, args);
    }
    break;


    case BT_CONTROL_SET_VOLUME:
    {
        char temp[16] = {0};
        uint8_t vol = 0;
        bt_volume_set_t *config = (bt_volume_set_t *)args;
        if (BT_VOLUME_MEDIA == config->mode)
        {
            vol = (config->volume.media_volume * 63) / 100;
            rt_snprintf(temp, sizeof(temp), "AD_%02X", vol);
        }
        else
        {
            vol = (config->volume.call_volume * 63) / 100;
            rt_snprintf(temp, sizeof(temp), "HF_%02X", vol);
        }
        ret = fr508x_cmd_control(cmd, 0, temp, rt_strlen(temp), args);
    }
    break;


    case BT_CONTROL_CLOSE_DEVICE:
    {
        ret = fr508x_close();
    }
    break;

    case BT_CONTROL_OPEN_DEVICE:
    {
        ret = fr508x_open();
    }
    break;

#ifdef BT_USING_DTMF
    case BT_CONTROL_DTMF_DIAL:
    {
        ret = fr508x_dtmf_dial(cmd, args);
    }
    break;
#endif

#ifdef BT_USING_MIC_MUTE
    case BT_CONTROL_SET_MIC_MUTE:
    {
        ret = fr508x_set_mute(cmd, args);
    }
    break;

    case BT_CONTROL_GET_MIC_MUTE:
    {
        ret = fr508x_cmd_control(cmd, 0, "2", 1, args);
    }
    break;
#endif


#ifdef BT_USING_LOCAL_MEDIA_EX
    case BT_CONTROL_LOCAL_DEL_MUSIC:
    {
        ret = fr508x_del_local_music(cmd, args);
    }
    break;

    case BT_CONTROL_LOCAL_SET_PLAY_MODE:
    {
        ret = fr508x_set_local_play_mode(cmd, args);
    }
    break;

    case BT_CONTROL_LOCAL_PLAY_ASSIGNED_MUSIC:
    {
        ret = fr508x_play_local_assigned_music(cmd, args);
    }
    break;

    case BT_CONTROL_LOCAL_GET_MUSIC_LIST:
    {
        ret = fr508x_play_local_get_music_list(cmd, args);
    }
    break;
#endif

#ifdef BT_USING_DEVICE_TYPE
    case BT_CONTROL_SET_DEVICE_TYPE:
    {
        ret = fr508x_set_device_type(cmd, args);
    }
    break;
#endif


    default:
        ret = fr508x_cmd_control(cmd, 0, RT_NULL, 0, args);
        break;
    }
    return ret;
}

static const at_client_cmd_t fr508x_at_cmd_table[] =
{
    {BT_CONTROL_PHONE_CONNECT, "AT#CA", "OK", 300, RT_NULL},
    {BT_CONTROL_PHONE_HANDUP, "AT#CB", "OK", 300, RT_NULL},
    {BT_CONTROL_DIAL_BACK, "AT#CC", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_MAKE_CALL, "AT#CN", RT_NULL, 200, RT_NULL},
#if 0
    {"AT#CE",       NULL},
    {"AT#CD",       NULL},
    {"AT#CG",       NULL},
    {"AT#CF",       NULL},
#endif
    {BT_CONTROL_SEARCH_EQUIPMENT, "AT#CH", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_CANCEL_SEARCH, "AT#CI", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_CONNECT_DEVICE, "AT#CJ", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_DISCONNECT, "AT#CK", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_QUERY_STATE, "AT#CL", "+STATE", 200, fr508x_at_cl_response},
    {BT_CONTROL_SWITCH_OFF, "AT#CX", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_SWITCH_ON, "AT#CY", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_SWITCH_TO_SOURCE, "AT#CO", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_SWITCH_TO_SINK, "AT#CQ", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_SET_VOLUME, "AT#CR", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_GET_VOLUME, "AT#CS", "OK", 200, fr508x_at_cs_response},
    {BT_CONTROL_GET_PEER_DEVICEINFO, "AT#CM", "+COD", 200, fr508x_at_cm_response},
    {BT_CONTROL_LOCAL_PLAY_NEXT, "AT#AB", "+MP3", 1000, RT_NULL},
    {BT_CONTROL_LOCAL_PLAY_SUSPEND, "AT#AD", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_LOCAL_PLAY_PREVIOUS, "AT#AE", "+MP3", 1000, RT_NULL},
    {BT_CONTROL_EARPHONE_PLAY, "AT#AW", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_LOCAL_PLAY, "AT#AC", "+MP3", 1000, RT_NULL},
    {BT_CONTROL_EARPHONE_PLAY_SUSPEND, "AT#AX", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_EARPHONE_PLAY_NEXT, "AT#AV", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_EARPHONE_PLAY_PREVIOUS, "AT#AY", RT_NULL, 200, RT_NULL},
#if 0
    {"AT#AM",       RT_NULL, 200, RT_NULL},
    {"AT#AN",       RT_NULL, 200, RT_NULL},
    {"AT#AS",       RT_NULL, 200, RT_NULL},
    {"AT#AT",       RT_NULL, 200, RT_NULL},
    {"AT#AL",       RT_NULL, 200, RT_NULL},
    {"AT#BR",       RT_NULL, 200, RT_NULL},
    {"AT#BS",       RT_NULL, 200, RT_NULL},
#endif
    {BT_CONTROL_GET_LOCAL_SONG_DETAILS, "AT#FA", "+MP3", 1000, fr508x_at_fa_response},
    {BT_CONTROL_GET_LOCAL_SONG_BRIEF, "AT#FB", "+NAME", 1000, fr508x_at_fb_response},
    {BT_CONTROL_GET_LOCAL_SONG_TOTAL_NUM, "AT#FC", "+MNUM", 1000, fr508x_at_fc_response},
#if 0
    {"AT#PC",       RT_NULL, 200, RT_NULL},
    {"AT#PT",       fr508x_at_pt_response},
    {"AT#PP",       fr508x_at_pp_response},
#endif
    {BT_CONTROL_GET_BT_MAC, "AT#FD", "+MAC", 200, fr508x_at_fd_response},
#if 0
    //{"AT#FE",       fr508x_at_fe_response},
    //{"AT#FF",       RT_NULL,200,RT_NULL},
#endif
    {BT_CONTROL_CLOSE_DEVICE, "AT#FG", "OK", 200, RT_NULL},
#ifdef BT_USING_DTMF
    {BT_CONTROL_DTMF_DIAL, "AT#CT", RT_NULL, 200, RT_NULL},
#endif
#ifdef BT_USING_MIC_MUTE
    {BT_CONTROL_SET_MIC_MUTE, "AT#CP", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_GET_MIC_MUTE, "AT#CP", "+MIC", 200, fr508x_get_mute_response},
#endif

#ifdef BT_USING_LOCAL_MEDIA_EX
    {BT_CONTROL_LOCAL_DEL_MUSIC, "AT#MA", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_LOCAL_SET_PLAY_MODE, "AT#MB", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_LOCAL_PLAY_ASSIGNED_MUSIC, "AT#MC", "+MP3", 1000, RT_NULL},
    {BT_CONTROL_LOCAL_GET_MUSIC_LIST, "AT#MD", "+MD", 1000, RT_NULL},
    {BT_CONTROL_LOCAL_GET_MUSIC_ID, "AT#ME", RT_NULL, 200, fr508x_at_me_response},
#endif

#ifdef BT_USING_SIRI
    {BT_CONTROL_SIRI_ON, "AT#BR", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_SIRI_OFF, "AT#BS", RT_NULL, 200, RT_NULL},
#endif

#ifdef BT_USING_USB
    {BT_CONTROL_ENTER_USB_MODE, "AT#CU1", RT_NULL, 200, RT_NULL},
    {BT_CONTROL_EXIT_USB_MODE, "AT#CU0", RT_NULL, 200, RT_NULL},
#endif

#ifdef BT_USING_DEVICE_TYPE
    {BT_CONTROL_SET_DEVICE_TYPE, "AT#FJ", RT_NULL, 200, RT_NULL},
#endif

};

void fr508x_set_cmd_table(at_client_t client)
{
    at_obj_set_cmd_table(client, fr508x_at_cmd_table, sizeof(fr508x_at_cmd_table) / sizeof(fr508x_at_cmd_table[0]));
    return;
}




