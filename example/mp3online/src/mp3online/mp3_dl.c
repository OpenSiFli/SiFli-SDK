
#include <rtthread.h>
#include <string.h>
#include "lwip/api.h"
#include "lwip/dns.h"
#include <webclient.h>
#include <cJSON.h>

#include "mp3_ringbuffer.h"
#include "mp3_network.h"
#include "local_music.h"
#include "mp3_dl.h"
#ifndef MIN
#define MIN(a,b) ((a)>(b)?(b):(a))
#endif


typedef void(*mp3_user_cb)(int ret);

typedef enum
{
    MP3_DL_CMD_READ_MORE,
    MP3_DL_CMD_STOP,
} mp3_dl_cmd_t;

typedef struct
{
    mp3_dl_cmd_t cmd;
    union
    {
        int read_pos;
    } data;
} mp3_dl_msg_t;

mp3_dl_state_t g_mp3_dl_state = MP3_DL_STATE_IDLE;

static int g_mp3_dl_content_len = 0;
static int g_mp3_dl_content_pos = 0;

static rt_timer_t g_mp3_dl_start_timer = NULL;
static mp3_user_cb g_user_cb = NULL;

/*  */
static int mp3_dl_get_part_callback(uint8_t *data, size_t len)
{
    rt_kprintf("%s %d: len=%d\n", __func__, __LINE__, len);
    if ((data == NULL) || (len == 0) || (len > 50*1000*1000))
    {
        if (g_user_cb)
        {
            g_user_cb(-1);
            g_user_cb = NULL;
        }
        if (g_mp3_dl_start_timer)
        {
            rt_timer_stop(g_mp3_dl_start_timer);
        }
        rt_kprintf("%s %d: input invalid\n", __func__, __LINE__);
        return 0;
    }

    mp3_ring_buffer_put(data, mp3_ring_buffer_space_len());//Fill data into the ring buffer

    g_mp3_dl_state = MP3_DL_STATE_DLING;// Downloading status
    g_mp3_dl_content_len = len;// Total content length
    g_mp3_dl_content_pos += MP3_RING_BUFFER_SIZE; // Downloaded Location
    play_ringbuff(g_mp3_dl_content_len);// Start playback
    if (g_user_cb)// Notify the user that the playback has started successfully
    {
        g_user_cb(0);
        g_user_cb = NULL;
    }
    if (g_mp3_dl_start_timer)
    {
        rt_timer_stop(g_mp3_dl_start_timer);
    }
    return 0;
}

static int mp3_dl_get_part_continue_callback(uint8_t *data, size_t len)
{
    rt_kprintf("%s %d: len=%d\n", __func__, __LINE__, len);
    if ((data == NULL) || (len == 0))
    {
        return 0;
    }

    mp3_ring_buffer_put(data, len);

    g_mp3_dl_state = MP3_DL_STATE_DLING;
    g_mp3_dl_content_pos += len;
    rt_kprintf("%s %d: g_mp3_dl_content_pos=%d\n", __func__, __LINE__, g_mp3_dl_content_pos);
    if (g_mp3_dl_content_pos >= g_mp3_dl_content_len)
    {
        /* download done */
        rt_kprintf("%s %d: dl done\n", __func__, __LINE__);
        g_mp3_dl_state = MP3_DL_STATE_IDLE;
    }
    return 0;
}

void mp3_dl_read_more(int read_pos)//Notify the user that playback has started successfully.Request to download more audio data during playback
{
    rt_kprintf("%s %d: read_pos=%d\n", __func__, __LINE__, read_pos);
    if (g_mp3_dl_state == MP3_DL_STATE_IDLE)
    {
        rt_kprintf("%s %d: no more data\n", __func__, __LINE__);
        return;
    }

    int ring_space = mp3_ring_buffer_space_len();// Calculate the available space of the circular buffer
    int last = g_mp3_dl_content_len - g_mp3_dl_content_pos;
    rt_kprintf("%s %d: last=%d\n", __func__, __LINE__, last);
    int dl_len = MIN(ring_space, last);
    rt_kprintf("%s %d: dl_len=%d\n", __func__, __LINE__, dl_len);
    mp3_network_get_part_continue(dl_len, mp3_dl_get_part_continue_callback);//Initiate a network request, continue to download dl_len bytes of data, and set the callback function mp3_dl_get_part_continue_callback to handle the data after the download is completed.

}

int mp3_dl_thread_init(const char *mp3_url)//The initialization preparation function before downloading the MP3 file
{
    int ret = 0;
    rt_kprintf("%s %d: g_mp3_dl_state=%d\n", __func__, __LINE__, g_mp3_dl_state);
    if (g_mp3_dl_state == MP3_DL_STATE_IDLE)//Check whether the current download status is MP3_DL_STATE_IDLE (idle state)
    {
        mp3_ring_buffer_reset();// Reset the MP3 circular buffer
        g_mp3_dl_content_len = 0; // Total content length
        g_mp3_dl_content_pos = 0;//Current location of the content
        //Start downloading the initial part of the MP3 file  MP3 file URL  Length of data to be downloaded (available space in the circular buffer)  Callback function after download is completed
        ret = mp3_network_get_part(mp3_url, mp3_ring_buffer_space_len(), mp3_dl_get_part_callback);
        if (ret < 0)
        {
            rt_kprintf("%s %d: ERR ret=%d\n", __func__, __LINE__, ret);
            return ret;
        }
        g_mp3_dl_state = MP3_DL_STATE_INIT;//Download has been successfully initiated. The status will be set to MP3_DL_STATE_INIT (initialization state).
    }
    else
    {
        rt_kprintf("%s %d: state err=%d\n", __func__, __LINE__, g_mp3_dl_state);
    }
    return ret;
}

void mp3_stream_resume(void)
{
    if (g_mp3_dl_state == MP3_DL_STATE_DLING)
    {
        play_resume();
    }
    else
    {
        rt_kprintf("%s %d: state err=%d\n", __func__, __LINE__, g_mp3_dl_state);
    }
}

void mp3_stream_pause(void)
{
    if (g_mp3_dl_state == MP3_DL_STATE_DLING)
    {
        play_pause();
    }
    else
    {
        rt_kprintf("%s %d: state err=%d\n", __func__, __LINE__, g_mp3_dl_state);
    }
}


void mp3_stream_start_timer_cb(void *parameter)
{
    rt_kprintf("%s %d: wait dl timeout\n", __func__, __LINE__);
    if (g_user_cb)//Check if there is a user callback function named g_user_cb
    {
        g_user_cb(-1);//If it exists, call the user callback function and pass -1 to indicate failure.
        g_user_cb = NULL;
    }
    rt_timer_stop(g_mp3_dl_start_timer);
}

void mp3_stream_start(const char *mp3_url, void *user_cb)
{
    rt_kprintf("mp3_stream_start: url=%s\n", mp3_url);
    rt_kprintf("g_mp3_dl_state=%d\n", g_mp3_dl_state);
    int ret = mp3_dl_thread_init(mp3_url);// Initialize the MP3 download thread and pass in the URL of the MP3 file
    if (ret < 0)
    {
        /* can not init mp3 dl, try reboot to recover network */
        rt_kprintf("mp3_dl_thread_init failed--\n");
        // drv_reboot();// Failed to initialize MP3 download. Try restarting the device to restore the network connection.
    }
    g_user_cb = user_cb;// Save the pointer to the user callback function
    if (g_mp3_dl_state == MP3_DL_STATE_INIT)
    {
        if (!g_mp3_dl_start_timer)
        {// Create a software timer named "mp3start" with a timeout duration of 30 seconds, single-trigger mode, for handling the timeout situation when starting the MP3 download.
            g_mp3_dl_start_timer = rt_timer_create("mp3start", mp3_stream_start_timer_cb, NULL,
                                                rt_tick_from_millisecond(30000), RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_ONE_SHOT);
        }
        else
        {
            rt_timer_stop(g_mp3_dl_start_timer);
        }
        rt_timer_start(g_mp3_dl_start_timer);
    }
    else
    {
       rt_kprintf("%s %d: state err=%d\n", __func__, __LINE__, g_mp3_dl_state);
    }
}

void mp3_stream_stop(void)
{
    rt_kprintf("%s %d: stopping stream, current state=%d\n", __func__, __LINE__, g_mp3_dl_state);
    if (g_mp3_dl_state != MP3_DL_STATE_IDLE)
    {
        mp3_network_get_part_cancel();
        play_stop();
        // Force reset state
        g_mp3_dl_state = MP3_DL_STATE_IDLE;
        g_mp3_dl_content_len = 0;
        g_mp3_dl_content_pos = 0;
        if (g_mp3_dl_start_timer) {
            rt_timer_stop(g_mp3_dl_start_timer);
        }
        rt_kprintf("%s %d: stream stopped, state set to IDLE\n", __func__, __LINE__);
    }
    else 
    {
        rt_kprintf("%s %d: stream already idle\n", __func__, __LINE__);
    }
}

uint32_t mp3_stream_get_total_seconds(void)
{
    return play_get_total_seconds();
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

