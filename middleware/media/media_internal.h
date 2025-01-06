#ifndef MEDIA_INTERNAL_H
#define MEDIA_INTERNAL_H
enum
{
    TRACEID_VIDEO_DECODE_TOTAL = 0x01000000,
    TRACEID_VIDEO_DECODE       = 0x01100000,

    TRACEID_AUDIO_DECODE_TOTAL = 0x02000000,
    TRACEID_AUDIO_DECODE       = 0x02100000,
    TRACEID_AUDIO_CONVERT      = 0x02200000,
    TRACEID_AUDIO_WRITE        = 0x02300000,

    TRACEID_VIDEO_GET          = 0x03000000,
    TRACEID_AUDIO_EMPTY        = 0x04000000,

    TRACEID_AV_PACKET          = 0x05000000,
    TRACEID_VIDEO_PACKET       = 0x05100000,
    TRACEID_AUDIO_PACKET       = 0x05200000,
};

#ifdef PKG_USING_SYSTEMVIEW
    #define TRACE_MARK_START(id)  SEGGER_SYSVIEW_OnUserStart(id)
    #define TRACE_MARK_STOP(id)   SEGGER_SYSVIEW_OnUserStop(id)
#else
    #define TRACE_MARK_START(id)
    #define TRACE_MARK_STOP(id)
#endif /* PKG_USING_SYSTEMVIEW */


#ifdef BSP_USING_PC_SIMULATOR
    #define __IO
    #define audio_dec_task_prio RT_THREAD_PRIORITY_MIDDLE
    #define video_dec_task_prio RT_THREAD_PRIORITY_MIDDLE

    #define audio_video_dec_task_prio RT_THREAD_PRIORITY_MIDDLE
    #define av_read_pkt_task_prio     RT_THREAD_PRIORITY_MIDDLE

    #define network_read_task_prio    RT_THREAD_PRIORITY_MIDDLE
    #define network_decode_task_prio  RT_THREAD_PRIORITY_MIDDLE
#else
    #define __IO    volatile
    #define audio_dec_task_prio (RT_THREAD_PRIORITY_MIDDLE + RT_THREAD_PRIORITY_HIGHER)
    #define video_dec_task_prio (RT_THREAD_PRIORITY_MIDDLE + RT_THREAD_PRIORITY_LOWWER)

    #define audio_video_dec_task_prio RT_THREAD_PRIORITY_MIDDLE
    #define av_read_pkt_task_prio     (RT_THREAD_PRIORITY_MIDDLE + RT_THREAD_PRIORITY_LOWWER + RT_THREAD_PRIORITY_LOWWER)

    #define network_read_task_prio    (RT_THREAD_PRIORITY_MIDDLE + RT_THREAD_PRIORITY_HIGHER + RT_THREAD_PRIORITY_HIGHER)
    #define network_decode_task_prio  (RT_THREAD_PRIORITY_MIDDLE + RT_THREAD_PRIORITY_LOWWER)
#endif /* BSP_USING_PC_SIMULATOR */


#define NETWORK_BUFFER_CAPACITY     64   //cache AVPacket from network download
#define READ_BUFFER_CAPACITY        3    //Undecoded AVPacket
#define AUDIO_CACHE_SIZE            32000

#define FFMPEG_HANDLE_MAGIC     0x55555555

typedef struct
{
    uint32_t is_audio: 1;
    uint32_t padding_size: 2;
    uint32_t data_len: 29;
} sifli_ezip_packet_t;

typedef struct
{
    rt_slist_t      snode;
    uint8_t         *buffer;
    uint32_t        buffer_size;
    uint32_t        data_len;
} ezip_video_packet_t;

#define AUDIO_PACKEET_MEMORY_POOL   (READ_BUFFER_CAPACITY + 6)

typedef struct
{
    rt_slist_t  snode;
    uint8_t     *buf;
    uint32_t    buf_size;
    uint32_t    data_len;
} ezip_audio_packet_t;

#define MP3_MAIN_BUFFER_SIZE    8000
typedef struct
{
    rt_slist_t              empty_audio_slist;
    rt_slist_t              readed_audio_slist;
    ezip_audio_packet_t     cache[AUDIO_PACKEET_MEMORY_POOL];
    uint8_t                 main_buf[MP3_MAIN_BUFFER_SIZE];
    uint32_t                main_left;
    uint8_t                 *main_ptr;
    void                    *decode_handle;
    uint8_t                 *decode_out;
    uint8_t                 *tws_out;
} ezip_audio_cache_t;

typedef struct
{
    rt_slist_t              empty_video_slist;
    rt_slist_t              decoded_video_slist;
    rt_slist_t              display_video_slist;
    ezip_video_packet_t     cache[VIDEO_BUFFER_CAPACITY];
} ezip_video_cache_t;

typedef struct
{
    char        header[8];          // 'siflezip'
    uint32_t    duration_seconds;
    int         fps;
    uint32_t    width;
    uint32_t    height;
    char        audio_codec[4];
    uint32_t    samplerate;
    uint32_t    ch;
    char        audio_fmt[4];
} ezip_media_t;

typedef struct ffmpeg_decoder_tag
{
    uint32_t                magic;
    uint32_t                user_data;
    uint32_t                last_video_get_tick;
    // File information
    const char             *filename;
    uint32_t                offset;
    __IO int                is_ok;

    // Media information
    AVFormatContext        *fmt_ctx;
    AVPacket                pkt;

    // Video information
    int                     video_stream_idx;
    AVCodecContext         *video_dec_ctx;
    media_cache_t           video_cache;
    ezip_video_cache_t      ezip_video_cache;
    ezip_audio_cache_t      ezip_audio_cache;
    float                   period_float;
    uint32_t                total_time_in_seconds;
    uint32_t                frame_index;
    uint32_t                last_seconds;
    uint32_t                seek_to_second;
    uint32_t                period;
    uint32_t                width;
    uint32_t                height;

    // Audio information
    int                     audio_stream_idx;
    AVCodecContext         *audio_dec_ctx;
    AVFrame                *audio_frame;
    uint32_t                audio_samplerate;
    uint32_t                audio_channel;
    uint32_t                audio_data_size;
    uint16_t               *audio_data;
    audio_client_t          audio_handle;
    uint32_t                audio_data_period;

    // Read thread information.
    os_message_queue_t      av_pkt_queue;
    rt_thread_t             av_pkt_read_thread;

    os_thread_t             video_decode_thread;
    os_thread_t             audio_decode_thread;

    os_message_queue_t      av_pkt_queue_audio;
    os_event_t              evt_init;
    os_event_t              evt_video;
    os_event_t              evt_audio;
    os_event_t              evt_pause;
    os_event_t              evt_video_decode;

    //avio for network streaming
    media_queue_t          *network_queue; //for network streaming
    AVIOContext            *avio_ctx;
    uint8_t                *avio_ctx_buffer;
    uint32_t                avio_ctx_buffer_size;

    //only invalid if is_nand == 1
    uint8_t                 *src_in_nand_address;
    uint32_t                src_in_nand_len;

    ffmpeg_config_t         cfg;
    int                     ezip_fd;
    ezip_media_t            ezip_header;
    uint8_t                 is_sifli_ezip_memdia;
    uint8_t                 is_nand;
    uint8_t                 is_network_file;
    uint8_t                 is_paused;    //paused by user
    uint8_t                 is_suspended; //suspended by other audio application
    uint8_t                 seeking_state; //1--start, 2--time ok, need I frame, 0--end
    uint8_t                 is_closing;   //aysnc closing
    sifli_gpu_fmt_t         gpu_pic_fmt;
} ffmpeg_decoder_t;

extern void ffmeg_mem_init();
extern void ffmpeg_memleak_check();
int ezip_video_cache_init(ffmpeg_handle thiz);
void ezip_video_cache_deinit(ffmpeg_handle thiz);
int ezip_video_decode(ffmpeg_handle thiz, uint32_t size, uint32_t paddings);
void ezip_audio_cache_init(ffmpeg_handle thiz);
void ezip_audio_cache_deinit(ffmpeg_handle thiz);
ezip_audio_packet_t *ezip_audio_read_packet(ffmpeg_handle thiz, uint32_t size, uint32_t paddings);
void ezip_audio_decode(ffmpeg_handle thiz, audio_server_callback_func callback);
int ezip_flash_read(ffmpeg_handle thiz, void *buf, int len);

#endif
