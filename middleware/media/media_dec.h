/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
#ifndef FFMPEG_RAM_URL_FMT
    #define FFMPEG_RAM_URL_FMT "ram://addr=0x%x&len=0x%x"
#endif

#ifdef __cplusplus
extern "C" {
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
#define IMG_DESC_FMT                IMG_DESC_FMT_YUV420P
#define IMG_PIXEL_SIZE              2
#define IMG_LV_FMT                  LV_IMG_CF_YUV420_PLANAR2
#else
#if defined(BSP_USING_PC_SIMULATOR)
#if (LV_COLOR_DEPTH == 24)
#define IMG_DESC_FMT        IMG_DESC_FMT_RGB888
#else
#define IMG_DESC_FMT        IMG_DESC_FMT_RGB565
#endif
#define IMG_PIXEL_SIZE          (LV_COLOR_SIZE>>3)
#define IMG_LV_FMT              LV_IMG_CF_TRUE_COLOR
#else
#if (LV_COLOR_DEPTH == 24) && defined(SOC_SF32LB58X) //HW jpeg not support RGB888 on 58x
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
} ffmpeg_src_e;

typedef enum
{
    e_ffmpeg_play_to_end,
    e_ffmpeg_suspended,
    e_ffmpeg_resumed,
    e_ffmpeg_progress, //val is seconds
    e_ffmpeg_play_to_error, //read frame error
    e_ffmpeg_play_to_loop,  //loop again
    e_ffmpeg_play_frames,  //val is current frames
} ffmpeg_cmd_e;

typedef struct
{
    ffmpeg_src_e    src;          //must be e_src_localfile now
    int             fmt_ctx_flag; //ffmpeg AVFMT_FLAG_*  map, in avformat.h. notw used now, should be zero
    uint8_t         fmt;          //dest image format. shoul be IMG_DESC_FMT_* in this file
    uint8_t         is_wait_for_resume; //only decode 2 frame , then auto pause, app should call ffmpeg_resume() to continue
    uint8_t         is_loop;      //audio loop again if set to 1
    uint8_t         audio_enable; //enable audio in media file if set to 1
    uint8_t         video_enable; //enable video in meida file if set to 1
    const char     *file_path;   /*if src is e_src_localbuffer, should be type + address + len
                                  example:
                                  nand://addr=0x63000abcd&len=0x12bce
                                  ram://addr=0x20000abcd&len=0x12bce
                                  /video/test.mp4
                                  http://xxxx.com/test.mp4
                                  https://xxxx.com/test.mp4
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
    /** media totoal time seconds */
    uint32_t total_time_in_seconds;
    /** media period in millseconds */
    uint32_t period;
    /** picture fmt */
    sifli_gpu_fmt_t gpu_pic_fmt;
} video_info_t;

/*------------API for special app -----------*/
int media_audio_get(AVFrame *frame, uint16_t *audio_data, uint32_t audio_data_size);
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

/**
 * @brief open a media URL to play
 *
 * @param return_hanlde return the handle
 * @param cfg configure for media info @see ffmpeg_config_t
 * @param user_data The user data that will be passed to notify() in ffmpeg_config_t
 *
 * @return 0 if successful, negative errno code if failure.
 */
int ffmpeg_open(ffmpeg_handle *return_hanlde, ffmpeg_config_t *cfg, uint32_t user_data);

/**
 * @brief stop ffmpeg playing
 *
 * do not call this in callback functioin notify()
 *
 * @param hanlde the handle got by ffmpeg_open()
 *
 */
void ffmpeg_close(ffmpeg_handle hanlde);

/**
 * @brief pause ffmpeg playing
 *
 * do not call this in callback functioin notify()
 *
 * @param hanlde the handle got by ffmpeg_open()
 *
 */
void ffmpeg_pause(ffmpeg_handle hanlde);

/**
 * @brief resume ffmpeg playing
 *
 * do not call this in callback functioin notify()
 *
 * @param hanlde the handle got by ffmpeg_open()
 *
 */
void ffmpeg_resume(ffmpeg_handle hanlde);

/**
 * @brief seek ffmpeg playing
 *
 * do not call this in callback functioin notify()
 *
 * @param hanlde the handle got by ffmpeg_open()
 * @param second seek to seconds from beginning
 *
 */
void ffmpeg_seek(ffmpeg_handle hanlde, uint32_t second);

/**
 * @brief mute/unmute audio
 *
 * @param hanlde the handle got by ffmpeg_open()
 * @param is_mute mute or unmute audio. 1--mute, 0-- unmute
 *
 */
void ffmpeg_audio_mute(ffmpeg_handle hanlde, bool is_mute);

/**
 * @brief release memory alloced by ffmpeg_eizp_release() or ffmpeg_get_first_ezip_in_nand()
 *
 * @param ezip alloced memory address by ffmpeg_eizp_release() or ffmpeg_get_first_ezip_in_nand()
 */
void ffmpeg_eizp_release(uint8_t *ezip);


/**
 * @brief get first ezip in local disk media file
 *
 * @param filename the file name in local disk
 * @param[out] w return picture width
 * @param[out] h return picture height
 * @param[out] psize return picture size in bytes
 * @return non-NULL poiner in memory for first picure, should release by ffmpeg_eizp_release(), return NULL if error
 */
uint8_t *ffmpeg_get_first_ezip(const char *filename, uint32_t *w, uint32_t *h, uint32_t *psize);

/**
 * @brief get first ezip nand
 *
 * @param nand_address media address in nand
 * @param nand_size media size in nand
 * @param[out] w return picture width
 * @param[out] h return picture height
 * @param[out] psize return picture size in bytes
 * @return non-NULL poiner in memory for first picure, should release by ffmpeg_eizp_release(), return NULL if error
 */
uint8_t *ffmpeg_get_first_ezip_in_nand(const char *nand_address, uint32_t nand_size,
                                       uint32_t *w, uint32_t *h, uint32_t *psize);

/**
 * @brief get video infomation
 *
 * @param hanlde the handle got by ffmpeg_open()
 * @param[out] video_width return picture width
 * @param[out] video_heightreturn picture height
 * @param[out] info picture format
 * @return 0 if successful, negative errno code if failure.
 */
int ffmpeg_get_video_info(ffmpeg_handle hanlde, uint32_t *video_width, uint32_t *video_height, video_info_t *info);


/*
0 - success
*/
int ffmpeg_get_ezip_info(const char *filename, uint32_t *w, uint32_t *h, uint32_t *max_size);

/**
 * @brief get new frame in ffmepg cache, should call ffmpeg_is_video_available() first, call this if new video frame available
 *
 * @param hanlde the handle got by ffmpeg_open()
 * @param data get the new frame data, diffrent format has diffrent means in *data, @see sifli_gpu_fmt_t
 *             format is return by ffmpeg_get_video_info()
 *             1. if format is e_sifli_fmt_ezip
 *                data[0] -- data addr
 *                data[1] -- data size
 *                data[2] -- IMG_DESC_FMT_EZIP
 *             2. if format e_sifli_fmt_mjpeg
 *                data[0] -- data addr
 *                data[1] -- unused
 *                data[2] -- unused
 *             3. if format e_sifli_fmt_yuv420p, app can use yuv directly for 52x/56x/57x chip
 *                data[0] -- y address
 *                data[1] -- u address
 *                data[2] -- v address
 * @return true if new frame decoded in ffmpeg cache, false no new frame decoded
 */
int ffmpeg_next_video_frame(ffmpeg_handle hanlde, uint8_t *data);

/**
 * @brief check video cache if a new frame is decoded to display
 *
 * @param hanlde the handle got by ffmpeg_open()
 *
 * @return true if new frame decoded in ffmpeg cache, false no new frame decoded
 */
bool ffmpeg_is_video_available(ffmpeg_handle hanlde);

/**
 * @brief check if there is a ffmpeg handle is playing
 *
 * not thread safe, maybe return a handle, the the handle was close by its user
 *
 * @return ffmpeg handle if exist, NULL if not exist
 */
ffmpeg_handle ffmpeg_player_status_get(void);

/**
 * Set whether to wait for the video buffer to be completely filled before reading video frames.
 * @param thiz FFmpeg handle
 * @param is_wait Flag indicating whether to wait for buffer full
 *                - 0: Do not wait, process existing frames immediately
 *                - 1: Wait until the buffer is completely filled before processing
*/
void ffmpeg_set_is_wait_video_full(ffmpeg_handle thiz, uint8_t is_wait);

#ifdef __cplusplus
}
#endif
#endif

