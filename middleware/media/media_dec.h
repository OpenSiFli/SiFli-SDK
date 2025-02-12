#ifndef MEDIA_DEC_H
#define MEDIA_DEC_H

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "os_adaptor.h"
#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/avassert.h"
#include "libavutil/time.h"
#include "libavutil/timestamp.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavcodec/avfft.h"
#include "libswresample/swresample.h"
#include "libavcodec/avcodec.h"
#include "libavutil/imgutils.h"
#include "yuv2rgb/yuv2rgb.h"
#include <libswscale/swscale.h>
#ifdef PKG_USING_SYSTEMVIEW
    #include "SEGGER_SYSVIEW.h"
#endif
#include "audio_server.h"
#include "media_queue.h"

#ifndef FFMPEG_NAND_URL_FMT
    #define FFMPEG_NAND_URL_FMT "nand://addr=0x%x&len=0x%x"
#endif

#define VIDEO_BUFFER_CAPACITY       3

// Video

#define IMG_DESC_FMT_RGB565     0
#define IMG_DESC_FMT_RGB888     1
#define IMG_DESC_FMT_YUV420P    2
#define IMG_DESC_FMT_ARGB8888   3
#define IMG_DESC_FMT_EZIP       4

#if defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X)
    #define IMG_DESC_USING_FFMPEG_YUV420P_BUFFER 1
#else
    #define IMG_DESC_USING_FFMPEG_YUV420P_BUFFER 0
#endif

#if IMG_DESC_USING_FFMPEG_YUV420P_BUFFER
    #define IMG_DESC_FMT            IMG_DESC_FMT_YUV420P
    #define IMG_PIXEL_SIZE          2
    #define IMG_LV_FMT              LV_IMG_CF_YUV420_PLANAR2
#else
    #if LV_COLOR_DEPTH == 24 && defined (BSP_USING_PC_SIMULATOR)
        #define IMG_DESC_FMT        IMG_DESC_FMT_RGB888
        #define IMG_PIXEL_SIZE      (LV_COLOR_SIZE>>3)
        #define IMG_LV_FMT          LV_IMG_CF_TRUE_COLOR
    #elif defined(SOC_SF32LB58X) //HW jpeg not support RGB888 on 58x
        #define IMG_DESC_FMT        IMG_DESC_FMT_ARGB8888
        #define IMG_PIXEL_SIZE      4
        #ifndef DISABLE_LVGL_V8
            #define IMG_LV_FMT      LV_IMG_CF_RGBA8888
        #else
            #define IMG_LV_FMT      LV_IMG_CF_ARGB8888
        #endif
    #else
        #define IMG_DESC_FMT        IMG_DESC_FMT_RGB565
        #define IMG_PIXEL_SIZE      2
        #ifdef  BSP_USING_PC_SIMULATOR
            #define IMG_LV_FMT          LV_IMG_CF_TRUE_COLOR
        #else
            #define IMG_LV_FMT          LV_IMG_CF_RGB565
        #endif
    #endif
#endif

typedef enum
{
    e_sifli_fmt_yuv420p,  // data in ffmpeg frame data[0] is yuv420p
    e_sifli_fmt_ezip,     // ...  is ezip,
    e_sifli_fmt_mjpeg,    // ...  is mjpeg
    e_sifli_fmt_argb8888, // ...  is argb8888
} sifli_gpu_fmt_t;


typedef struct ms_frame_data_t_tag
{
    rt_slist_t      snode;
    AVFrame        *frame;
} ms_frame_data_t;

typedef struct
{
    rt_slist_t              empty_frame_slist;
    rt_slist_t              decoded_frame_slist;
    rt_slist_t              display_frame_slist;
    ms_frame_data_t         video_cache[VIDEO_BUFFER_CAPACITY];
} media_cache_t;

typedef enum
{
    e_src_localfile,
    e_network_frames_stream, //audio/video not demuxed, should demux in decoder
} ffmpeg_src_e;

typedef enum
{
    e_ffmpeg_play_to_end,
    e_ffmpeg_suspended,
    e_ffmpeg_resumed,
    e_ffmpeg_progress, //val is seconds
    e_ffmpeg_play_to_error, //read frame error
    e_ffmpeg_play_to_loop,  //loop again
} ffmpeg_cmd_e;

typedef struct
{
    ffmpeg_src_e    src;
    int             fmt_ctx_flag; //ffmpeg AVFMT_FLAG_*  map, in avformat.h. notw used now, should be zero
    uint8_t         fmt;          //dest image format. shoul be IMG_DESC_FMT_* in this file
    uint8_t         is_loop;
    uint8_t         audio_enable;
    uint8_t         video_enable;
    const char     *file_path;   /*if src is e_src_localbuffer, should be type + address + len
                                  example:
                                  nand://addr=0x63000abcd&len=0x12bce
                                 */
    void *(*mem_malloc)(size_t size);
    void (*mem_free)(void *rmem);
    int (*notify)(uint32_t user_data, ffmpeg_cmd_e cmd, uint32_t val);

    /*only for e_network_frames_stream*/
    uint8_t        *avio_buffer;
    uint32_t       avio_buffer_size;
    media_free     pack_free; //media_packet_t free
} ffmpeg_config_t;

typedef struct ffmpeg_decoder_tag *ffmpeg_handle;
typedef struct
{
    uint32_t total_time_in_seconds;
    uint32_t period;
    sifli_gpu_fmt_t gpu_pic_fmt;
} video_info_t;

/*------------API for special app -----------*/
int media_audio_get(AVFrame *frame, uint16_t *audio_data);
int media_decode_video(ffmpeg_handle thiz,
                       int *got_frame,
                       AVPacket *p_AVPacket);

int media_audio_len(AVFrame *frame);
int media_cache_init(media_cache_t *cache,      int cache_num);
void media_cache_deinit(media_cache_t *cache, int cache_num);
int media_video_get(media_cache_t *cache,     int fmt, uint8_t *data, uint8_t is_ezip);
int media_video_convert(uint8_t *buf, AVFrame *frame, int fmt);
bool media_video_need_decode(media_cache_t *cache);
bool ezip_video_need_decode(ffmpeg_handle thiz);
/*------------API for local file and network file stream -----------*/
/*
 0 -- success
 -1-- no memory
 -2-- busy, only one ffmpeg decoder instance allowed
*/
int ffmpeg_open(ffmpeg_handle *return_hanlde, ffmpeg_config_t *cfg, uint32_t user_data);

/*
    for e_network_frames_stream, should stop network downloadint first to avoid memory leak
*/
void ffmpeg_close(ffmpeg_handle hanlde);
void ffmpeg_pause(ffmpeg_handle hanlde);
void ffmpeg_resume(ffmpeg_handle hanlde);
void ffmpeg_seek(ffmpeg_handle hanlde, uint32_t second);
void ffmpeg_audio_mute(ffmpeg_handle hanlde, bool is_mute); //1 mute, 0 unmute

void ffmpeg_eizp_release(uint8_t *ezip);

uint8_t *ffmpeg_get_first_ezip(const char *filename, uint32_t *w, uint32_t *h, uint32_t *psize);
uint8_t *ffmpeg_get_first_ezip_in_nand(const char *nand_address, uint32_t nand_size,
                                       uint32_t *w, uint32_t *h, uint32_t *psize);
/*
0 - success
*/
int ffmpeg_get_video_info(ffmpeg_handle hanlde, uint32_t *video_width, uint32_t *video_height, video_info_t *info);
/*
data :  output result set to data or data[0], [1] , data[2]
        data should has memory space 12 bytes at least for yuv or ezip
0 - success, copy frame to data
*/
int ffmpeg_next_video_frame(ffmpeg_handle hanlde, uint8_t *data);


bool ffmpeg_is_video_available(ffmpeg_handle hanlde);

//only use for e_network_packet_stream, p is malloced by user, and free by pack_free() in ffmpeg_config_t
void ffmpeg_send_frame_to_decoder(ffmpeg_handle thiz, media_packet_t *p);

#endif

