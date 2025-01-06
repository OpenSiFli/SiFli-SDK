#include "sm_api.h"

#if defined(RT_USING_SAL)
#include <netdb.h>
#include <sys/socket.h>
#if RT_USING_POSIX
#define gethostbyname(name) lwip_gethostbyname(name)
#define gethostbyname_r(name, ret, buf, buflen, result, h_errnop) \
                    lwip_gethostbyname_r(name, ret, buf, buflen, result, h_errnop)
#define freeaddrinfo(addrinfo) lwip_freeaddrinfo(addrinfo)
#define getaddrinfo(nodname, servname, hints, res) \
        lwip_getaddrinfo(nodname, servname, hints, res)
#endif
#else
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#endif /* RT_USING_SAL */

typedef unsigned char BYTE;
#define INVALID_SOCKET  (-1)
#define SOCKET int

#define POST_HEADER_BUFSZ   1024
#define HEAD_LEN            4
#define MEDIA_KEY_VIDEO     0x00
#define MEDIA_KEY_AUDIO     0xFF

static int g_exit = 0;
static uint32_t g_seq_num = 0;
static struct webclient_session *g_session_ctrl = NULL;
static media_queue_t *g_queue_ptr;
static bool is_move = false;
static bool need_send_i_frame;

#if ORDER_VIDEO_SIZE
    static uint32_t g_width = 240, g_height = 320, g_bitrate = 900000;
#endif

typedef struct
{
    float x;
    float y;
} tp_point_t;

static tp_point_t first_point;

static int build_key_event(char *buf, char *key);
static int sm_network_post(const char *url, void **result, const char *body, const char *head);
static void download_thread_entry(void *parameter);
int sm_net_send(SOCKET sock, const char *pbyBuf, const int nSize);

void set_need_iframe()
{
    need_send_i_frame = true;
}
void set_video_order_config(uint32_t width, uint32_t height, uint32_t bitsrate)
{
#if ORDER_VIDEO_SIZE
    g_width = width;
    g_height = height;
    g_bitrate = bitsrate;
#endif
}
int sm_download_open(media_queue_t *queue_ptr)
{
    g_queue_ptr = queue_ptr;
    g_exit = 0;
    g_seq_num = 0;
    LOG_I("-----\n\n");
    LOG_I("%s", APPLYING_DEVICE_URL);
    LOG_I("-----\n\n");
#ifdef BSP_USING_PC_SIMULATOR
    WSADATA wsaData;
    int nResult = -1;
    nResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (nResult != 0)
    {
        rt_kprintf("WSAStartup failed: %d\n", nResult);
        return 1;
    }
#endif

    rt_thread_t tid = rt_thread_create("medianet",
                                       download_thread_entry,
                                       (void *)sm_get_device_info(),
                                       16 * 1024,
                                       RT_THREAD_PRIORITY_MIDDLE,
                                       RT_THREAD_TICK_DEFAULT);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return 0;
    }
    return -1;
}
int sm_download_close(void)
{
    g_exit = 1;
#ifdef BSP_USING_PC_SIMULATOR
    WSACleanup();
#endif
    rt_thread_mdelay(50);

    while (g_exit != 2)
    {
        rt_thread_mdelay(20);
        SM_LOG_I("wait net exit");
    }
    return 0;
}

static int sm_network_post(const char *url, void **result, const char *body, const char *head)
{
    char *response  = NULL;
    char *header    = NULL;
    size_t resp_len = 0;
    int index       = 0;
    webclient_request_header_add(&header, "Content-Length: %d\r\n", strlen(body));
    webclient_request_header_add(&header, "Content-Type:application/json\r\n");
    if (webclient_request(url, header, body, strlen(body), (void **)&response, &resp_len) < 0)
    {
        LOG_I("post failed");
        web_free(header);
        return -1;
    }

    LOG_I("Newwork post\n");
    LOG_I("URL:%s\n", url);
    LOG_I("header:%s\n", header);
    LOG_I("body:%s\n", body);

    LOG_I("post resp len %d", resp_len);

    if (result)
    {
        *result = web_calloc(1, resp_len + 1);
        memset(*result, 0, resp_len + 1);
        memcpy(*result, response, resp_len);
    }
    if (header)
    {
        web_free(header);
    }

    if (response)
    {
        web_free(response);
    }
    return 0;
}

static void save_packet(char *pBuf, int nBufSize, SOCKET sock)
{
    unsigned char code;
    uint16_t ret = 0;
    media_packet_t *real_p;
    code = pBuf[0];
    real_p = (media_packet_t *)(pBuf - sizeof(media_packet_t));
#ifdef __DROP_VIDEO__
    if (code == MEDIA_KEY_VIDEO)
    {
        sm_packet_free(real_p);
        return;
    }
#endif

#ifdef __DROP_AUDIO__

    if (code == MEDIA_KEY_AUDIO)
    {
        sm_packet_free(real_p);
        return;
    }
#endif

    if ((code == MEDIA_KEY_VIDEO) || (code == MEDIA_KEY_AUDIO))
    {
        rt_uint32_t evt;
        real_p->data_offset = 0;
        real_p->seq_num = g_seq_num++;
        real_p->data_len = nBufSize;
        real_p->data_type = (code == MEDIA_KEY_VIDEO) ? 'v' : 'a';
        media_queue_add_tail(g_queue_ptr, real_p);
        //LOG_I("add tail t=%c",  real_p->data_type);
    }
    else
    {
        sm_packet_free(real_p);
    }
}


static inline uint32_t swap_u32(uint32_t val)
{
    return ((val >> 24) & 0x000000ff) |
           ((val >> 8) & 0x0000ff00) |
           ((val << 8) & 0x00ff0000) |
           ((val << 24) & 0xff000000);
}

static void send_get_i_frame_to_cmd(int sock)
{
    if (sock <= INVALID_SOCKET)
        return;

    const char *text = "{\"cmd\":4,\"param\":0}";

    int len = strlen(text);

    char *pcsBuf = (char *)calloc(len + 4 + 1, sizeof(char));
    memcpy(&pcsBuf[0], &len, 4);
    memcpy(&pcsBuf[4], text, len);
    int res = sm_net_send(sock, (const char *)pcsBuf, (len + 4));
    free(pcsBuf);
    LOG_I("send get i frame cmd");
}
static void download_thread_entry(void *parameter)
{
    struct webclient_session *g_session_av = NULL;
    char lenbuf[HEAD_LEN + 1];
    char *pBuf = NULL;
    media_packet_t *real_p = NULL;
    int size;
    int nBufSize;
    int len = 0;

    sm_device_info_t *devinfo = (sm_device_info_t *)parameter;
    char *uri;
    char *body = devinfo->szFirstMsg;
    uri = malloc(strlen(devinfo->szDeviceVideoHost) + 10);
    RT_ASSERT(uri);
    strcpy(uri, "http://");
    strcat(uri, devinfo->szDeviceVideoHost/*43.248.97.226:20023*/);

    int ret = 0;
    int bytes_read;

    len = strlen(body);
    LOG_I("download_thread_entry:");
    LOG_I("d host=[%s]", devinfo->szDeviceVideoHost);
    LOG_I("d body=[%s]", body);
    LOG_I("d body len =[%d]", len);

    g_session_av = webclient_session_create(POST_HEADER_BUFSZ);
    if (g_session_av == RT_NULL)
    {
        LOG_I("webclient error");
        goto __exit;
    }

    /* build header for upload */
    if (len > 0)
    {
        char *buf = (char *)rt_calloc(len + sizeof(len) + 1, 1);
        RT_ASSERT(buf);
        memcpy(&buf[0], &len, 4);
        memcpy(&buf[4], body, len);
        webclient_send_usermethod(g_session_av, uri, buf, len + 4);
        rt_free(buf);
    }
    else
    {
        webclient_send_usermethod(g_session_av, uri, NULL, 0);
        LOG_I("post %s", uri);
    }

    while (1)
    {
        size = 0;
        memset(lenbuf, 0, HEAD_LEN + 1);

        if (g_exit)
        {
            break;
        }

        do
        {
            if (g_session_av->socket == INVALID_SOCKET)
            {
                SM_LOG_I("socket err1");
                goto __exit;
            }

            bytes_read = recv(g_session_av->socket, lenbuf + size, HEAD_LEN - size, 0);
            if (g_exit)
            {
                goto __exit;
            }
            if (bytes_read > 0)
            {
                size += bytes_read;
            }
            else
            {
                SM_LOG_I("recv err1 %d\n", bytes_read);
                if (need_send_i_frame)
                {
                    send_get_i_frame_to_cmd(g_session_av->socket);
                    need_send_i_frame = false;
                }
                rt_thread_mdelay(100);
            }
        }
        while (size < HEAD_LEN);

        //Obtain the data of nBufSize length
        if (size == HEAD_LEN)
        {
            //receive from content
            size = 0;
            nBufSize = 0;
            memcpy(&nBufSize, lenbuf, 4);
            //nBufSize = swap_u32(nBufSize);
            LOG_D("recv len=%d\n", nBufSize);

            if (nBufSize < 0 || nBufSize > MAX_SIZE_OF_ONE_FRAME)
            {
                SM_LOG_E("data error %d", __LINE__, nBufSize);
                goto __exit;
            }

            real_p = (media_packet_t *)sm_packet_malloc(sizeof(media_packet_t) + nBufSize + 1);
            if (NULL == real_p)
            {
                SM_LOG_E("malloc fail");
                goto __exit;
            }

            pBuf = (char *)&real_p[1];
            memset(pBuf, 0, nBufSize + 1);
            do
            {
                if (g_session_av->socket == INVALID_SOCKET)
                {
                    SM_LOG_E("socket err2", __LINE__);
                    sm_packet_free(real_p);
                    real_p = NULL;
                    goto __exit;
                }

                bytes_read = recv(g_session_av->socket, pBuf + size, nBufSize - size, 0);
                if (g_exit)
                {
                    goto __exit;
                }

                if (bytes_read > 0)
                {
                    size += bytes_read;
                }
                else
                {
                    sm_packet_free(real_p);
                    real_p = NULL;
                    SM_LOG_E("recv err2 %d", bytes_read);
                    goto __exit;
                }
            }
            while (size < nBufSize);

            if (size == nBufSize)
            {
                //if(lenbuf[3] == 3)
                {
                    save_packet(pBuf, nBufSize, g_session_av->socket);
                    real_p = NULL; //free by queue
                }
            }
            else
            {
                sm_packet_free(real_p);
                real_p = NULL;
                SM_LOG_E("recv err3");
                goto __exit;
            }
        }
        else
        {
            SM_LOG_E("recv err4");
            goto __exit;
        }
        if (need_send_i_frame)
        {
            send_get_i_frame_to_cmd(g_session_av->socket);
            need_send_i_frame = false;
        }
    }

__exit:
    if (g_session_av)
    {
        webclient_close(g_session_av);
        g_session_av = NULL;
    }
    if (uri)
    {
        free(uri);
    }
    if (real_p)
    {
        sm_packet_free(real_p);
        real_p = NULL;
    }
    while (!g_exit)
    {
        rt_thread_delay(200);
    }
    g_exit = 2;
    SM_LOG_I("net exit");
    return;
}


static int sm_ctrl_process(char *uri, char *body, struct webclient_session *session)
{
    int ret = 0;
    int bytes_read;
    char URL[48] = { 0 };
    int rc = WEBCLIENT_OK;

    char lenbuf[5];
    char *pBuf = NULL;
    int size;
    int nBufSize;
    struct timeval timeout;

    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    memset(URL, 0, 48);
    sprintf(URL, "http://%s", uri);

    /* build header for upload */
    //webclient_header_fields_add(session, "Content-Length: %d\r\n", strlen(szFirstMsg));
    //webclient_header_fields_add(session, "Content-Type: application/octet-stream\r\n");
    int len = strlen(body);
    char *pcsBuf = (char *)calloc(len + 4 + 1, sizeof(char));
    memset(pcsBuf, 0, len + 4 + 1);
    memcpy(&pcsBuf[0], &len, 4);
    memcpy(&pcsBuf[4], body, len);

    /* send POST request by default header */
    rc = webclient_send_usermethod(session, URL, pcsBuf, strlen(body) + 4);

    setsockopt(session->socket, SOL_SOCKET, SO_RCVTIMEO, (void *) &timeout,
               sizeof(timeout));

    free(pcsBuf);
    if (rc != WEBCLIENT_OK)
    {
        return rc;
    }

    while (1)
    {
        size = 0;
        memset(lenbuf, 0, 5);
        //get the data len.
        do
        {

            if (session->socket == INVALID_SOCKET)
            {
                goto __exit;
            }

            bytes_read = recv(session->socket, lenbuf + size, 4 - size, 0);

            if (bytes_read > 0)
            {
                size += bytes_read;
            }
            else if (bytes_read == 0)
            {
                goto __exit;
            }
            else
            {
                goto __exit;
            }
        }
        while (size < 4);

        //Obtain the data of nBufSize length
        if (size == 4)
        {
            //receive from content
            size = 0;
            nBufSize = 0;
            memcpy(&nBufSize, lenbuf, 4);

            pBuf = rt_malloc(nBufSize + 1);
            RT_ASSERT(pBuf);
            memset(pBuf, 0, nBufSize + 1);
            do
            {
                if (session->socket == INVALID_SOCKET)
                {
                    rt_free(pBuf);
                    goto __exit;
                }
#ifdef BSP_USING_PC_SIMULATOR
                bytes_read = recv(session->socket, pBuf + size, nBufSize - size, 0);
#else
                bytes_read = lwip_recv(session->socket, pBuf + size, nBufSize - size, 0);
#endif


                if (bytes_read > 0)
                {
                    size += bytes_read;
                }
                else if (bytes_read == 0)
                {
                    rt_free(pBuf);
                    goto __exit;
                }
                else
                {
                    rt_free(pBuf);
                    goto __exit;
                }
            }
            while (size < nBufSize);

            if (size == nBufSize)
            {
                rt_free(pBuf);
                pBuf = NULL;
                goto __exit;
            }
            else
            {
                rt_free(pBuf);
                goto __exit;
            }
        }
        else
        {
            goto __exit;
        }
    }

__exit:
    return WEBCLIENT_OK;
}

int sm_net_send(SOCKET sock, const char *pbyBuf, const int nSize)
{
    int nSendLen = -1;

    if (sock < 0)
    {
        return 0;
    }
    nSendLen = 0;
    while (nSendLen < nSize)
    {
        int nTemp = send(sock, (const char *)&pbyBuf[nSendLen], nSize - nSendLen, 0);
        if (nTemp <= 0)
        {
            LOG_I("ctrl err:%d,%d\n", sock, errno);
            break;
        }
        nSendLen += nTemp;
    }
    return nSendLen;
}



static void host2ip(char *host, char *ip)
{
    struct in_addr **addr_list;
    char *pos;
    struct hostent *he;

    pos = strstr(host, ":");
    memset(pos, 0, 1);
    he = gethostbyname(host);
    if (he == NULL)
    {
        LOG_I("hostname %s", host);
        return;
    }

    addr_list = (struct in_addr **)he->h_addr_list;

    strcpy(ip, inet_ntoa(*(struct in_addr *)he->h_addr_list[0]));

    memset(pos, ':', 1);
    strcat(ip, pos);
    LOG_I("gethost %s to %s\n", host, ip);
}

static int applying(char *response)
{
    char szIPAddr[30] = {0};
    sm_device_info_t *devinfo = sm_get_device_info();
    cJSON *json, *OrderID, *DeviceTcpHost, *AntiControlToken, *DeviceSigner, *insidePara;;
    cJSON *YunDeviceType, *OrderAC, *pData, *AnboxStreamUrl, *OtherParam, *pSize;

    cJSON *root = cJSON_Parse(response);
    if (root != NULL)
    {

        json = cJSON_GetObjectItem(root, "code");
        if (json != NULL && json->type == cJSON_Number)
        {
            LOG_I("order respone=%d\n", json->valueint);
            if (json->valueint == 0)
            {
                return -1;
            }
        }

        json = cJSON_GetObjectItem(root, "Data");

        OrderID = cJSON_GetObjectItem(json, "OrderId");
        if (OrderID != NULL && OrderID->type == cJSON_Number)
        {
            devinfo->nOrderID = OrderID->valueint;
            LOG_I("OrderID=%d", devinfo->nOrderID);
        }

        YunDeviceType = cJSON_GetObjectItem(json, "YunDeviceType");
        if (YunDeviceType != NULL && YunDeviceType->type == cJSON_Number)
        {
            devinfo->nYunDeviceType = YunDeviceType->valueint;
            LOG_I("DeviceType=%d\n", devinfo->nYunDeviceType);
        }

        memset(devinfo->szDeviceTcpHost, 0, sizeof(devinfo->szDeviceTcpHost));
        DeviceTcpHost = cJSON_GetObjectItem(json, "DeviceTcpHost");
        if (DeviceTcpHost != NULL && DeviceTcpHost->type == cJSON_String)
        {
            strcpy(devinfo->szDeviceTcpHost, DeviceTcpHost->valuestring);

            if (g_exit)
                return -1;

            host2ip(devinfo->szDeviceTcpHost, szIPAddr);
            memset(devinfo->szDeviceTcpHost, 0, sizeof(devinfo->szDeviceTcpHost));
            strcpy(devinfo->szDeviceTcpHost, szIPAddr);

            LOG_I("IP: %s", szIPAddr);
            LOG_I("Host: %s", devinfo->szDeviceTcpHost);
        }

        memset(devinfo->szAntiControlToken, 0, sizeof(devinfo->szAntiControlToken));
        AntiControlToken = cJSON_GetObjectItem(json, "AntiControlToken");
        if (AntiControlToken != NULL && AntiControlToken->type == cJSON_String)
        {
            strcpy(devinfo->szAntiControlToken, AntiControlToken->valuestring);

            LOG_I("token: %s", devinfo->szAntiControlToken);
        }

        memset(devinfo->szDeviceSigner, 0, sizeof(devinfo->szDeviceSigner));
        DeviceSigner = cJSON_GetObjectItem(json, "DeviceSigner");
        if (DeviceSigner != NULL && DeviceSigner->type == cJSON_String)
        {
            strcpy(devinfo->szDeviceSigner, DeviceSigner->valuestring);
            LOG_I("Signer:%s\n", devinfo->szDeviceSigner);
        }

        YunDeviceType = cJSON_GetObjectItem(json, "YunDeviceType");
        if (YunDeviceType != NULL && YunDeviceType->type == cJSON_Number)
        {
            devinfo->nYunDeviceType = YunDeviceType->valueint;
            LOG_I("DeviceType:%d", devinfo->nYunDeviceType);
        }

        OrderAC = cJSON_GetObjectItem(json, "OrderAC");
        if (OrderAC != NULL && OrderAC->type == cJSON_String)
        {
            pData = cJSON_Parse(OrderAC->valuestring);
            if (pData != NULL)
            {
                memset(devinfo->szDeviceVideoHost, 0, sizeof(devinfo->szDeviceVideoHost));
                AnboxStreamUrl = cJSON_GetObjectItem(pData, "AnboxStreamUrl");
                if (AnboxStreamUrl != NULL && AnboxStreamUrl->type == cJSON_String)
                {
                    strcpy(devinfo->szDeviceVideoHost, AnboxStreamUrl->valuestring);
                    if (g_exit)
                        return -1;

                    host2ip(devinfo->szDeviceVideoHost, szIPAddr);
                    memset(devinfo->szDeviceVideoHost, 0, sizeof(devinfo->szDeviceVideoHost));
                    strcpy(devinfo->szDeviceVideoHost, szIPAddr);

                    LOG_I("VideoHost:%s\n", devinfo->szDeviceVideoHost);
                }

                memset(devinfo->szFirstMsg, 0, sizeof(devinfo->szFirstMsg));

                OtherParam = cJSON_GetObjectItem(pData, "OtherParam");
                if (OtherParam != NULL && OtherParam->type == cJSON_String)
                {
                    memcpy(devinfo->szFirstMsg, OtherParam->valuestring, strlen(OtherParam->valuestring));

                    pSize = cJSON_Parse(OtherParam->valuestring);
                    if (pSize != NULL)
                    {
#if ORDER_VIDEO_SIZE
                        cJSON_ReplaceItemInObjectCaseSensitive(pSize, "videowidth", cJSON_CreateNumber(g_width));
                        cJSON_ReplaceItemInObjectCaseSensitive(pSize, "videoheight", cJSON_CreateNumber(g_height));
                        cJSON_ReplaceItemInObjectCaseSensitive(pSize, "bitrate", cJSON_CreateNumber(g_bitrate));
                        insidePara = cJSON_GetObjectItem(pSize, "inside_param");

                        if (insidePara != NULL && insidePara->type == cJSON_Object)
                        {
                            cJSON_ReplaceItemInObjectCaseSensitive(insidePara, "virtual_width", cJSON_CreateNumber(g_width));
                            cJSON_ReplaceItemInObjectCaseSensitive(insidePara, "virtual_height", cJSON_CreateNumber(g_height));
                            cJSON_ReplaceItemInObjectCaseSensitive(insidePara, "width", cJSON_CreateNumber(g_width));
                            cJSON_ReplaceItemInObjectCaseSensitive(insidePara, "height", cJSON_CreateNumber(g_height));
                            cJSON_ReplaceItemInObjectCaseSensitive(insidePara, "bitrate", cJSON_CreateNumber(g_bitrate));
                        }

                        char *json_str = cJSON_Print(pSize);
#endif

                        json = cJSON_GetObjectItem(pSize, "videowidth");
                        if (json != NULL && json->type == cJSON_Number)
                        {
                            devinfo->videowidth = json->valueint;
                            LOG_I("videowidth: %d", devinfo->videowidth);
                        }
                        json = cJSON_GetObjectItem(pSize, "videoheight");
                        if (json != NULL && json->type == cJSON_Number)
                        {
                            devinfo->videoheight = json->valueint;
                            LOG_I("videoheight:%d\n", devinfo->videoheight);
                        }
#if ORDER_VIDEO_SIZE
                        if (json_str)
                        {
                            rt_memcpy(devinfo->szFirstMsg, json_str, strlen(json_str));
                            cJSON_free(json_str);
                        }
#endif
                        cJSON_Delete(pSize);
                    }
                }

                cJSON_Delete(pData);
            }
        }

        cJSON_Delete(root);
    }

    return 0;
}

int sm_net_order(int order_id, char *app_id, char *token)
{
    int res;
    char *body = NULL;
    body = malloc(DATA_BUFFER_SIZE);
    RT_ASSERT(body);

    sprintf(body, "%s", "{");
    sprintf(body, "%s%s\"%s\",", body, "\"AppId\":", app_id);
    sprintf(body, "%s%s\"%s\",", body, "\"DeviceToken\":", token);
    sprintf(body, "%s%s%d,", body, "\"AppVersionCode\":", DES_SDK_VERSION_CODE);
    sprintf(body, "%s%s%d", body, "\"SdkType\":", 1);
    sprintf(body, "%s%s", body, "}");
    LOG_I("order");
    char *response = NULL;
    res = sm_network_post("http://app.ddyun.com/HWYOrder/SdkStart",
                          (void **)&response,
                          body,
                          "content-type:application/json");

    if (0 == res)
    {
        res = applying(response);
    }
    free(body);

    if (response)
    {
        web_free(response);
    }
    LOG_I("order res=%d", res);
    return res;
}


int sm_net_apply_for_device(char *name, char **token)
{
    int res;
    const char *fmt = "{\n    \"appPackageName\": \"%s\",\n    \"userClientIp\": \"%s\",\n    \"effectiveMinutes\": %d\n}";
    char *body = NULL;
    body = calloc(DATA_BUFFER_SIZE, 1);
    RT_ASSERT(body);
    sprintf(body, fmt, name, "192.168.1.1", 60);

    char *response = NULL;
    LOG_I("applaying");

    res = sm_network_post(APPLYING_DEVICE_URL,
                          (void **)&response,
                          body,
                          "content-type:application/json");

    LOG_I("applaying res=%d", res);
    if (0 == res)
    {
        cJSON *json;
        LOG_I("apply res=%s", response);
        cJSON *root = cJSON_Parse(response);
        if (root != NULL)
        {
            json = cJSON_GetObjectItem(root, "code");
            if (json != NULL && json->type == cJSON_Number && json->valueint == 200)
            {
                json = cJSON_GetObjectItem(root, "Data");

                json  = cJSON_GetObjectItem(json, "accessToken");
                if (json  != NULL && json ->type == cJSON_String)
                {
                    char *tokener = NULL;
                    LOG_I("apply_token=%s", json->valuestring);
                    if (*token == RT_NULL)
                    {
                        tokener = malloc(strlen(json->valuestring) + 1);
                        RT_ASSERT(tokener);
                        *token = tokener;
                    }
                    else
                    {
                        tokener = *token;
                    }

                    if (tokener)
                    {
                        strcpy((char *)tokener, json->valuestring);
                    }
                }
            }
            cJSON_Delete(root);
        }
    }

    free(body);

    if (response)
    {
        web_free(response);
    }
    return res;
}

int sm_net_reset_device(char *name, char *token, int operate_type)
{
    int res;
    char *fmt = "{\n    \"operateType\": %d,\n    \"appPackageName\": \"%s\",\n    \"accessToken\": \"%s\"\n}";
    char *body = NULL;
    body = calloc(DATA_BUFFER_SIZE, 1);
    RT_ASSERT(body);

    sprintf(body, fmt, operate_type, name, token);

    char *response = NULL;
    res = sm_network_post(RESET_DEVICE_URL, (void **)&response, body, "content-type:application/json");
    free(body);
    if (response)
    {
        web_free(response);
    }
    return res;
}

static int fill_ctrl_info(char *p)
{
    cJSON *root, *user, *data;
    char *buf;
    long ltime;
    sm_device_info_t *devinfo = sm_get_device_info();

#ifdef BSP_USING_PC_SIMULATOR
    __time32_t timep;

    _time32(&timep);
#else
    time_t timep;
    time(&timep);

#endif

    ltime = timep;

    root = cJSON_CreateObject();
    if (root != NULL)
    {
        user = cJSON_CreateObject();
        cJSON_AddStringToObject(user, "Channel", DES_SDK_CHANNEL);
        cJSON_AddNumberToObject(user, "OrderId", devinfo->nOrderID);
        cJSON_AddStringToObject(user, "UCID", devinfo->app_id);
        cJSON_AddNumberToObject(user, "ddyVerCode", DES_SDK_VERSION_CODE);
        cJSON_AddStringToObject(user, "imeType", "1");
        buf = cJSON_PrintUnformatted(user);

        data = cJSON_CreateObject();
        cJSON_AddStringToObject(data, "command", "userInfo");
        cJSON_AddStringToObject(data, "data", buf);
        cJSON_AddNumberToObject(data, "time", ltime);

        cJSON_AddItemToObject(root, "data", data);
        cJSON_AddNumberToObject(root, "type", 2);
        cJSON_AddNumberToObject(root, "from", 3);
        cJSON_AddNumberToObject(root, "time", ltime);
        cJSON_AddStringToObject(root, "token", devinfo->szAntiControlToken);

        buf = cJSON_PrintUnformatted(root);
        strcpy(p, buf);
        cJSON_free(buf);
        cJSON_Delete(root);

        return 1;
    }

    return 0;
}

static float valueadjust(float v)
{
    float new_v = v;
    if (v < 0.0f)
    {
        new_v = (float)0.0f;
    }

    if (v > 1.0f)
    {
        new_v = (float)1.0f;
    }
    return new_v;
}

int sm_ctrl_create(bool is_new)
{
    int ret = 0;
    sm_device_info_t *devinfo = sm_get_device_info();
    if (is_new && g_session_ctrl)
    {
        webclient_close(g_session_ctrl);
        g_session_ctrl = NULL;
    }

    if (g_session_ctrl == NULL)
    {
        int len;
        char *p = malloc(DATA_BUFFER_SIZE);

        if (p)
        {
            memset(p, 0, DATA_BUFFER_SIZE);
            fill_ctrl_info(p);
            g_session_ctrl = webclient_session_create(POST_HEADER_BUFSZ);
            if (g_session_ctrl)
            {
                sm_ctrl_process(devinfo->szDeviceTcpHost, p, g_session_ctrl);
            }
            else
            {
                SM_LOG_I("ctrl fail");
                ret = -1;
            }
            free(p);
        }
    }

    return ret;
}


int sm_ctrl_destry(void)
{
    if (g_session_ctrl)
    {
        webclient_close(g_session_ctrl);
        g_session_ctrl = NULL;
    }
    return 0;

}

static int build_tp_event(char *buf, int status, int index, int count, tp_point_t *tp)
{
    size_t i;
    int nTLen, nDataLen;
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = 1;
    sm_device_info_t *devinfo = sm_get_device_info();
    nTLen = strlen(devinfo->szAntiControlToken);
    memcpy(buf + 5, &nTLen, sizeof(nTLen));
    memcpy(buf + 9, devinfo->szAntiControlToken, nTLen);

    nDataLen = 12 + count * 12;
    memcpy(buf + 9 + nTLen, &nDataLen, sizeof(nDataLen));
    memcpy(buf + 13 + nTLen, &status, sizeof(status));
    memcpy(buf + 17 + nTLen, &index, sizeof(index));
    memcpy(buf + 21 + nTLen, &count, sizeof(count));

    for (i = 0; i < count; i++)
    {
        memcpy(buf + 25 + nTLen + 12 * i, &i, sizeof(i));
        memcpy(buf + 29 + nTLen + 12 * i, &tp[i].x, sizeof(tp[i].x));
        memcpy(buf + 33 + nTLen + 12 * i, &tp[i].y, sizeof(tp[i].y));
    }

    return 25 + nTLen + count * 12;
}

static int ctrl_send_tp_msg(struct webclient_session *session, int tp_event, float x, float y)
{
    int len;
    char *buf = NULL;
    tp_point_t point;
    point.x = x;
    point.y = y;
    buf = calloc(DATA_BUFFER_SIZE, 1);
    bool is_rotate = true;
    if (is_rotate)
    {
        point.x = valueadjust(y);
        point.y = valueadjust(1.0f - x);
    }

    if (buf)
    {
        len = build_tp_event(buf, tp_event, 0, 1, &point);
        char *pcsBuf = (char *)calloc(len + 4 + 1, sizeof(char));
        RT_ASSERT(pcsBuf);
        memcpy(&pcsBuf[0], &len, 4);
        memcpy(&pcsBuf[4], buf, len);

        if (session)
        {
            if (sm_net_send(session->socket, (const char *)pcsBuf, (len + 4)) != (len + 4))
            {
                sm_ctrl_create(1);
                sm_net_send(g_session_ctrl->socket, (const char *)pcsBuf, (len + 4));
            }
        }
        free(pcsBuf);
        free(buf);
    }
    return 0;
}


int sm_ctrl_send_tp(sm_event_t ss_st, float x, float y)
{
    if (ss_st == SM_TOUCH_PRESS)
    {
        first_point.x = x;
        first_point.y = y;
        is_move = false;
        LOG_D("tp pressed(%f, %f)", x, y);
    }
    else if (ss_st == SM_TOUCH_PRESSING)
    {
        //LOG_I("tp pressing(%f, %f)", x, y);
    }
    else if (ss_st == SM_TOUCH_RELEASE)
    {
        if ((SM_TOUCH_MOVE_MIN <= ABS(first_point.x - x))
                || (SM_TOUCH_MOVE_MIN <= ABS(first_point.y - y)))
        {
            is_move = true;
        }

        LOG_D("tp release(%f, %f) move=%d", x, y, is_move);

        if (is_move)
        {
            sm_decode_clean();
            set_need_iframe();
            //LOG_I("send moving up");
            ctrl_send_tp_msg(g_session_ctrl, SM_TOUCH_PRESS, first_point.x, first_point.y);
            ctrl_send_tp_msg(g_session_ctrl, SM_TOUCH_PRESSING, first_point.x, first_point.y);
            ctrl_send_tp_msg(g_session_ctrl, SM_TOUCH_PRESSING, x, y);
            ctrl_send_tp_msg(g_session_ctrl, SM_TOUCH_RELEASE, x, y);
        }
        else
        {
            //LOG_I("send click");
            ctrl_send_tp_msg(g_session_ctrl, SM_TOUCH_PRESS, first_point.x, first_point.y);
            ctrl_send_tp_msg(g_session_ctrl, SM_TOUCH_RELEASE, first_point.x, first_point.y);
        }
    }

    if (ss_st == SM_VOLUP || ss_st == SM_VOLDOWN)
    {
        int len;
        char *buf = calloc(DATA_BUFFER_SIZE, 1);
        RT_ASSERT(buf);
        if (buf)
        {
            if (ss_st == SM_VOLUP)
                build_key_event(buf, "KEYCODE_VOLUME_UP");//KEYCODE_BACK KEYCODE_HOME KEYCODE_APP_SWITCH KEYCODE_VOLUME_UP KEYCODE_VOLUME_DOWN
            else
                build_key_event(buf, "KEYCODE_VOLUME_DOWN");

            len = strlen(buf);
            if (len != 0)
            {
                char *pcsBuf = (char *)calloc(len + 4 + 1, sizeof(char));
                memcpy(&pcsBuf[0], &len, 4);
                memcpy(&pcsBuf[4], buf, len);

                if (g_session_ctrl)
                {
                    sm_net_send(g_session_ctrl->socket, (const char *)pcsBuf, (len + 4));
                }
                free(pcsBuf);
            }
            free(buf);
        }

    }
    else if (ss_st == SM_BACK)
    {
        int len;
        char *szBuffer = malloc(DATA_BUFFER_SIZE);
        if (szBuffer)
        {
            memset(szBuffer, 0, DATA_BUFFER_SIZE);
            build_key_event(szBuffer, "KEYCODE_BACK");

            len = strlen(szBuffer);
            if (len != 0)
            {
                char *pcsBuf = (char *)calloc(len + 4 + 1, sizeof(char));
                memcpy(&pcsBuf[0], &len, 4);
                memcpy(&pcsBuf[4], szBuffer, len);

                if (g_session_ctrl)
                {
                    sm_net_send(g_session_ctrl->socket, (const char *)pcsBuf, (len + 4));
                }
                free(pcsBuf);
            }
            free(szBuffer);
        }

    }

    return 0;
}



char *json_valu2string(cJSON *root, const char *key)
{
    cJSON *item;

    if (root == NULL || key == NULL)
    {
        return NULL;
    }

    item = cJSON_GetObjectItem(root, key);
    if (item == NULL)
    {
        return NULL;
    }

    switch (item->type)
    {
    case cJSON_False:
        return strdup("false");
    case cJSON_True:
        return strdup("true");
    case cJSON_NULL:
        return strdup("null");
    case cJSON_Number:
    {
        char buf[32];
        _snprintf(buf, sizeof(buf), "%ld", item->valueint);
        return strdup(buf);
    }
    case cJSON_String:
    {
        if (strstr(item->valuestring, "\",") != NULL)
        {
            return "";
        }
        return strdup(cJSON_PrintUnformatted(item));
    }
    case cJSON_Array:
    {
        return strdup(cJSON_PrintUnformatted(item));
    }
    case cJSON_Object:
    {
        return strdup(cJSON_PrintUnformatted(item));
    }
    default:
        return NULL;
    }
}

static int build_key_event(char *buf, char *key)
{
    char szText[50] = "input keyevent ";
    cJSON *root, *data;
    char *json;
    long ltime;
    sm_device_info_t *devinfo = sm_get_device_info();
#ifdef BSP_USING_PC_SIMULATOR
    __time32_t timep;
    _time32(&timep);
#else
    time_t timep;
    time(&timep);

#endif
    ltime = timep;

    root = cJSON_CreateObject();
    if (root != NULL)
    {
        data = cJSON_CreateObject();
        cJSON_AddStringToObject(data, "command", "log");

        strcat(szText, key);
        cJSON_AddStringToObject(data, "data", szText);
        cJSON_AddNumberToObject(data, "time", ltime);

        cJSON_AddItemToObject(root, "data", data);
        cJSON_AddNumberToObject(root, "type", 2);
        cJSON_AddNumberToObject(root, "from", 3);
        cJSON_AddNumberToObject(root, "time", ltime);
        cJSON_AddStringToObject(root, "token", devinfo->szAntiControlToken);

        json = cJSON_PrintUnformatted(root);
        strcpy(buf, json);
        cJSON_free(json);
        cJSON_Delete(root);

        return 1;
    }

    return 0;
}

static int build_heart(char *buf, bool is_init)
{
    cJSON *root, *data;
    char *json;
    long ltime;
    sm_device_info_t *devinfo = sm_get_device_info();
#ifdef BSP_USING_PC_SIMULATOR
    __time32_t timep;
    _time32(&timep);
#else
    time_t timep;
    time(&timep);
#endif
    ltime = timep;

    root = cJSON_CreateObject();
    if (root != NULL)
    {
        data = cJSON_CreateObject();
        cJSON_AddStringToObject(data, "command", "keepalive");
        cJSON_AddNumberToObject(data, "init", is_init ? 1 : 0);

        cJSON_AddItemToObject(root, "data", data);
        cJSON_AddNumberToObject(root, "type", 6);
        cJSON_AddNumberToObject(root, "from", 3);
        cJSON_AddNumberToObject(root, "time", ltime);
        cJSON_AddStringToObject(root, "token", devinfo->szAntiControlToken);

        json = cJSON_PrintUnformatted(root);
        strcpy(buf, json);
        cJSON_free(json);
        cJSON_Delete(root);

        return strlen(buf);
    }

    return 0;
}

int sm_send_heart(bool is_init)
{
    int len;
    char *buf = NULL;
    buf = malloc(DATA_BUFFER_SIZE);

    if (NULL == g_session_ctrl)
    {
        return -1;
    }
    if (buf)
    {
        memset(buf, 0, DATA_BUFFER_SIZE);
        len = build_heart(buf, is_init);

        char *heart_buf = (char *)calloc(len + 4 + 1, sizeof(char));
        RT_ASSERT(heart_buf);
        memcpy(&heart_buf[0], &len, 4);
        memcpy(&heart_buf[4], buf, len);
        if (g_session_ctrl)
        {
            if (sm_net_send(g_session_ctrl->socket, (const char *)heart_buf, (len + 4)) != (len + 4))
            {
                sm_ctrl_create(1);
                sm_net_send(g_session_ctrl->socket, (const char *)heart_buf, (len + 4));
            }
        }
        free(heart_buf);
        free(buf);
    }

    return 0;
}

#if 0

void bezierInterpolationOnePoint(float p1x, float p1y, float p2x, float p2y, float t, float *px, float *py)
{
    *px = (1 - t) * p1x + t * p2x;
    *py = (1 - t) * p1y + t * p2y;
}

void bezierInterpolationMultiPoint(float p0[2], float p1[2], float p2[2], int numPoints, float points[numPoints][2])
{
    for (int i = 0; i < numPoints; i++)
    {
        float t = i / (float)numPoints;
        float b0 = (1 - t) * (1 - t);
        float b1 = 2 * t * (1 - t);
        float b2 = t * t;
        points[i][0] = b0 * p0[0] + b1 * p1[0] + b2 * p2[0];
        points[i][1] = b0 * p0[1] + b1 * p1[1] + b2 * p2[1];
    }
}

#define MAX_NUMPOINTS 20
int bezierInterpolationOnePoint_test()
{
    float p0[2] = {(float)0.5, (float)0.3};
    float p1[2] = {(float)0.5, (float)0.4};
    float p2[2] = {(float)0.5, (float)0.5};

    int numPoints = MAX_NUMPOINTS;
    float points[MAX_NUMPOINTS][2];

    bezierInterpolationMultiPoint(p0, p1, p2, numPoints, points);

    ctrl_send_tp(WIFI_CTL_TOUCH_PRESS, p0[0], p0[1]);

    for (int i = 0; i < numPoints; i++)
    {
        ctrl_send_tp(WIFI_CTL_TOUCH_PRESSING, points[i][0], points[i][1]);
    }
    rt_kprintf("\n");

    ctrl_send_tp(WIFI_CTL_TOUCH_RELEASE, p2[0], p2[1]);

    return 0;
}

#endif

int sm_check_video(char *buf)
{
    cJSON *root, *data;
    char *json;
    long ltime;
    sm_device_info_t *devinfo = sm_get_device_info();
#ifdef BSP_USING_PC_SIMULATOR
    __time32_t timep;
    _time32(&timep);
#else
    time_t timep;
    time(&timep);

#endif
    ltime = timep;

    root = cJSON_CreateObject();
    if (root != NULL)
    {
        data = cJSON_CreateObject();
        cJSON_AddNumberToObject(data, "type", 1);

        cJSON_AddItemToObject(root, "data", data);
        cJSON_AddNumberToObject(root, "type", 1);
        cJSON_AddNumberToObject(root, "from", 3);
        cJSON_AddNumberToObject(root, "time", ltime);
        cJSON_AddStringToObject(root, "token", devinfo->szAntiControlToken);

        json = cJSON_PrintUnformatted(root);
        strcpy(buf, json);
        cJSON_free(json);
        cJSON_Delete(root);

        return 1;
    }

    return 0;
}

/*Overwrite LWIP memory allocation functions*/

void *mem_calloc(mem_size_t count, mem_size_t size)
{
    void *p = app_anim_mem_alloc(size * count, 1);
    if (p)
        memset(p, 0, size * count);
    return p;
}

void *mem_malloc(mem_size_t size)
{
    return app_anim_mem_alloc(size, 1);
}


void  mem_free(void *mem)
{
    app_anim_mem_free(mem);
}


