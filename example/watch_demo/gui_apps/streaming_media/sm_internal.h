#ifndef _SM_INTERNAL_H_
#define _SM_INTERNAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <rtthread.h>

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
#include <webclient.h>
#include "lwip/mem.h"
#include "app_mem.h"


#define DBG_TAG           "douyin"
//#define DBG_LVL           LOG_LVL_INFO
#include "log.h"


#include "os_adaptor.h"
#include "cJSON.h"
#include "audio_server.h"
#include "media_queue.h"
#include "media_dec.h"
#include "sm_config.h"

#define SM_TOUCH_MOVE_MIN   (0.1f)
#define ABS(x)              ((x) > 0 ? (x) : -(x))

#define CLOUD_HOST_LEN      30
#define CLOUD_TOKEN_LEN     30
#define CLOUD_SIGNER_LEN    200
#define CLOUD_APPID_LEN     20
#define CLOUD_USERTOKEN_LEN 80

#define DATA_BUFFER_SIZE 1024
#define DES_SDK_VERSION_CODE 1
#define DES_SDK_CHANNEL "scsdk"


#define MAX_SIZE_OF_ONE_FRAME (256*1024)

#define CLOUD_DOWNLOAD_RETRY_CNT    5
#define CLOUD_MEDIA_GET_RETRY_CNT   10

typedef struct
{
    int nOrderID;
    int nYunDeviceType;
    int videoheight;
    int videowidth;
    char szDeviceTcpHost[CLOUD_HOST_LEN];
    char szAntiControlToken[CLOUD_TOKEN_LEN];
    char szDeviceSigner[CLOUD_SIGNER_LEN];
    char szDeviceVideoHost[CLOUD_HOST_LEN];
    char szFirstMsg[DATA_BUFFER_SIZE];
    char app_id[CLOUD_APPID_LEN];
    char szUserToken[CLOUD_USERTOKEN_LEN];
} sm_device_info_t;

#undef __IO
#ifdef BSP_USING_PC_SIMULATOR
    #define __IO
#else
    #define     __IO    volatile
#endif

#define SM_LOG_I    LOG_I
#define SM_LOG_E    LOG_E


typedef struct
{
    void *(*mem_malloc)(size_t size);
    void (*mem_free)(void *rmem);
    int (*notify)(int event, void *paramer);
} streaming_callback_t;

extern void *ffmpeg_alloc(size_t nbytes);
extern void ffmpeg_free(void *p);
extern void *ffmpeg_realloc(void *p, size_t new_size);

int sm_download_open(media_queue_t *queue_ptr);
int sm_download_close(void);
sm_device_info_t *sm_get_device_info(void);
int sm_net_apply_for_device(char *name, char **token);

void sm_packet_free(void *p);
void *sm_packet_malloc(size_t size);

int sm_ctrl_create(bool is_new);
int sm_ctrl_destry(void);
int sm_decode_start(media_queue_t     *queue_ptr, int fmt, streaming_callback_t *callbacks);
void sm_decode_clean(void);
int sm_decode_stop(void);
int sm_net_order(int order_id, char *app_id, char *token);
int sm_send_heart(bool is_init);

#endif
