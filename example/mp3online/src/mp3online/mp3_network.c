
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/api.h"
#include "lwip/dns.h"
#include <webclient.h>
#include "mp3_mem.h"
#include "mp3_network.h"
#include "bt_pan_mp3.h"
static rt_mq_t g_mp3_network_mq = NULL;
static rt_thread_t g_mp3_network_thread = NULL;

static bool g_mp3_network_connected = false;
bool mp3_network_is_connected(void)
{
    return g_mp3_network_connected;
}

void mp3_network_set_connected(bool connected)
{
    g_mp3_network_connected = connected;
}

int mp3_network_get(const char *url, mp3_nw_rsp_data_callback callback)
{
    int ret = 0;
    if (g_mp3_network_mq)
    {
        char *msg_url = strdup(url);
        mp3_nw_msg_t msg = {MP3_NW_CMD_GET, msg_url, NULL, 0, callback};
        ret = rt_mq_send(g_mp3_network_mq, &msg, sizeof(msg));
    }
    return ret;
}
// Obtain some data. Through the message queue mechanism, the download tasks are asynchronously sent to the network processing thread to achieve non-blocking network request processing.
int mp3_network_get_part(const char *url, size_t size, mp3_nw_rsp_data_callback callback)
{
    int ret = 0;
    if (g_mp3_network_mq)
    {
        char *msg_url = strdup(url);
        mp3_nw_msg_t msg = {MP3_NW_CMD_GET_PART, msg_url, NULL, size, callback};
        ret = rt_mq_send(g_mp3_network_mq, &msg, sizeof(msg));
    }
    return ret;
}

int mp3_network_get_part_continue(size_t size, mp3_nw_rsp_data_callback callback)
{
    int ret = 0;
    if (g_mp3_network_mq)
    {
        mp3_nw_msg_t msg = {MP3_NW_CMD_GET_PART_CONTINUE, NULL, NULL, size, callback};
        ret = rt_mq_send(g_mp3_network_mq, &msg, sizeof(msg));
    }
    return ret;
}

int mp3_network_get_part_cancel(void)
{
    int ret = 0;
    if (g_mp3_network_mq)
    {
        mp3_nw_msg_t msg = {MP3_NW_CMD_GET_PART_CANCEL, NULL, NULL, 0, NULL};
        ret = rt_mq_send(g_mp3_network_mq, &msg, sizeof(msg));
    }
    return ret;
}

static void svr_found_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    if (ipaddr != NULL)
    {
        rt_kprintf("DNS lookup succeeded, IP: %s\n", ipaddr_ntoa(ipaddr));
    }
}

int check_internet_access()
{
    int r = 0;
    const char *hostname = MP3_HOST_NAME;
    ip_addr_t addr = {0};

    {
        err_t err = dns_gethostbyname(hostname, &addr, svr_found_callback, NULL);
        if (err != ERR_OK && err != ERR_INPROGRESS)
        {
            rt_kprintf("Coud not find %s, please check PAN connection\n", hostname);
        }
        else
            r = 1;
    }

    return r;
}

void mp3_network_thread_entry(void *params)
{
    struct webclient_session* session = RT_NULL;
    static struct webclient_session* get_part_session = RT_NULL;
    static size_t g_content_length = 0;
    static size_t g_totol_bytes_read = 0;
    int ret = 0;
    int bytes_read, resp_status;

    while (check_internet_access() == 0)
    {
        rt_kprintf("no internet, wait...\n");
        rt_thread_mdelay(2000);
        mp3_network_set_connected(false);//Set the network to disconnected state
    }

    mp3_network_set_connected(true);

    exist_sniff_mode();//退出蓝牙低功耗模式

    while (1)
    {
        mp3_nw_msg_t msg;
        rt_err_t err = rt_mq_recv(g_mp3_network_mq, &msg, sizeof(mp3_nw_msg_t), RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == err);

        switch (msg.cmd)
        {
            case MP3_NW_CMD_GET:
            case MP3_NW_CMD_GET_PART:
            case MP3_NW_CMD_GET_PART_CONTINUE:
            {
                int trunc_size = 0;
                /* get response, check content length */
                if ((msg.cmd == MP3_NW_CMD_GET) || (msg.cmd == MP3_NW_CMD_GET_PART))
                {
                    /* create webclient session and set header response size */
                    session = webclient_session_create(POST_HEADER_BUFSZ);
                    RT_ASSERT(session);

                    if (msg.cmd == MP3_NW_CMD_GET_PART)
                    {
                        get_part_session = session;
                    }

                    rt_kprintf("%s %d: GET msg.url=%s\n", __func__, __LINE__, msg.url);
                    if ((resp_status = webclient_get(session, msg.url)) != 200)
                    {
                        rt_kprintf("webclient GET request failed, response(%d) error.\n", resp_status);
                        webclient_close(session);
                        free((void *)msg.url);  //from strdup

                        if (msg.callback)
                        {
                            msg.callback(NULL, 0);
                        }
                        if (msg.cmd == MP3_NW_CMD_GET_PART)
                        {
                            get_part_session = RT_NULL;
                        }

                        if (resp_status < 0)
                        {
                            /* try reconnect bt-pan? */
                            //extern void bt_app_connect_pan_timeout_handle(void *parameter);
                            //bt_app_connect_pan_timeout_handle(NULL);
                        }
                        break;
                    }
                    free((void *)msg.url);  //from strdup

                    g_content_length = webclient_content_length_get(session);
                    if (g_content_length == 0)
                    {
                        rt_kprintf("webclient GET response data is null.\n");
                        webclient_close(session);
                        if (msg.callback)
                        {
                            msg.callback(NULL, 0);
                        }
                        if (msg.cmd == MP3_NW_CMD_GET_PART)
                        {
                            get_part_session = RT_NULL;
                        }
                        break;
                    }
                }
                /* read content data */
                if (msg.callback)
                {
                    if (msg.cmd == MP3_NW_CMD_GET_PART)
                    {
                        trunc_size = msg.data_len;
                        g_totol_bytes_read = 0;
                    }
                    else if (msg.cmd == MP3_NW_CMD_GET_PART_CONTINUE)
                    {
                        trunc_size = msg.data_len;
                        session = get_part_session;
                    }
                    else if (msg.cmd == MP3_NW_CMD_GET)
                    {
                        trunc_size = g_content_length;
                    }
                    if (session == RT_NULL)
                    {
                        /* something wrong, maybe last read failed */
                        msg.callback(NULL, 0);
                        break;
                    }
                    char *content = NULL;
                    content = mp3_mem_malloc(trunc_size + 1);
                    RT_ASSERT(content);
                    memset(content, 0, trunc_size + 1);
                    bytes_read = webclient_read(session, content, trunc_size);
                    g_totol_bytes_read += bytes_read;
                    //RT_ASSERT(bytes_read == trunc_size);
                    if (msg.cmd == MP3_NW_CMD_GET_PART)
                    {
                        /* use total len instead of read size */
                        msg.callback(content, g_content_length);
                    }
                    else
                    {
                        msg.callback(content, bytes_read);
                    }
                    mp3_mem_free(content);
                }
                /* close session when read finish */
                if ((bytes_read < trunc_size)   //error
                    || (g_totol_bytes_read >= g_content_length))    //end
                {
                    if (session)
                    {
                        webclient_close(session);
                        session = RT_NULL;
                        get_part_session = RT_NULL;
                        g_content_length = 0;
                    }
                }
                break;
            }
            case MP3_NW_CMD_GET_PART_CANCEL:
            {
                if (get_part_session)
                {
                    webclient_close(get_part_session);
                    get_part_session = RT_NULL;
                    g_content_length = 0;
                }
                break;
            }
            default:
                break;
        }
    }
    return;
}

int mp3_network_thread_init(void)
{
    g_mp3_network_mq = rt_mq_create("mp3_nw_mq", sizeof(mp3_nw_msg_t), 5, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_mp3_network_mq);
    g_mp3_network_thread = rt_thread_create("mp3_net", mp3_network_thread_entry, NULL, 4096, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(g_mp3_network_thread);
    rt_err_t err = rt_thread_startup(g_mp3_network_thread);
    RT_ASSERT(RT_EOK == err);
}
INIT_PRE_APP_EXPORT(mp3_network_thread_init);
