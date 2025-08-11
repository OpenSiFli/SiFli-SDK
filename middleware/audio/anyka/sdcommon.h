/**
 * @file sdcommon.h
 * @brief Define the common types for audio libs
 *
 * Copyright (C) 2020 Anyka (Guangzhou) Microelectronics Technology Co., Ltd.
 * @update date 2020-06-11
 * @version 1.0
 */

#ifndef __SOUND_COMMON_H__
#define __SOUND_COMMON_H__

#include "anyka_types.h"
#include "medialib_global.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    MEDIALIB_CALLBACK_FUN_MALLOC                Malloc;
    MEDIALIB_CALLBACK_FUN_FREE                  Free;
    MEDIALIB_CALLBACK_FUN_PRINTF                printf;
    MEDIALIB_CALLBACK_FUN_FLUSH_DCACHE_RANGE    flushDCache;
} T_AUDIO_CB_FUNS;

typedef enum
{
    SD_ERR_NEED_MORE_DATA       =  0, // no load
    SD_ERR_FAIL                 = -1, // generic error
    SD_ERR_INVALID_HANDLE       = -2,
    SD_ERR_INVALID_PARAM        = -3,
    SD_ERR_MEMORY_ALLOC         = -4,
    SD_ERR_OUTBUF_TOO_SMALL     = -5,
    SD_ERR_STREAM_TERMINATED    = -9,  // nothing left to process
    SD_ERR_PENDING              = -10, // pending data to output, e.g. processing is on-going
} T_SD_ERROR_CODE;


#define  AK32Q10(x)    ((T_S32)((x)*(1<<10)))
#define  AK32Q15(x)    ((T_S32)((x)*(1<<15)))
#define  AK16Q10(x)    ((T_S16)((x)*(1<<10)))
#define  AK16Q15(x)    ((T_S16)((x)*(1<<15)))
#define  AKU16Q10(x)   ((T_U16)((x)*(1<<10)))
#define  AKU16Q15(x)   ((T_U16)((x)*(1<<15)))

// Parameters about signal level or amplitude in sd APIs are in the form of AK32Q15(x.xx), in which
//   x.xx is the ratio reletive to full scale (FS).
// When user needs to set this type of param to a value in unit of dBFS, they can use the macro
//   SD_dBFS_TO_LEVEL to convert the dBFS value to the form accepted by sd API.
#ifndef NO_MATH_H
extern float powf(float, float);
extern float log10f(float);
// dB is float, level is Q15
#define  SD_dBFS_TO_LEVEL(dB)       AK32Q15(powf(10.0f, (float)(dB)/20.0f)) // 10^(dB/20)
#define  SD_LEVEL_TO_dBFS(level)    (20.0f * log10f(1.0f * (float)(level)/(1<<15))) // 20 * log10(level)
#endif

// debug zones
#define SD_ZONE_ID_ERROR     0x0001
#define SD_ZONE_ID_WARNING   0x0002
#define SD_ZONE_ID_INFO      0x0004 // 必要的、用户可读的信息
#define SD_ZONE_ID_FUNCTION  0x0008 // 函数进入退出
#define SD_ZONE_ID_PARAM     0x0010 // 列出详细参数
#define SD_ZONE_ID_TRACE     0x0020 // 跟踪程序运行的流程
#define SD_ZONE_ID_OPEN      0x0040 // Open 时的信息
#define SD_ZONE_ID_CLOSE     0x0080 // Close 时的信息
#define SD_ZONE_ID_EVENT     0x0100 // 用户发起或接收到的、非频繁发生的事件
#define SD_ZONE_ID_VERBOSE   0x8000

#define SD_DEFAULT_DEBUG_ZONES  (SD_ZONE_ID_ERROR | SD_ZONE_ID_WARNING | SD_ZONE_ID_INFO | SD_ZONE_ID_OPEN | SD_ZONE_ID_CLOSE | SD_ZONE_ID_EVENT)

/* ------ platform implemented method ------ */

typedef T_pVOID(*SDLIB_CALLBACK_FUN_MUTEX_CREATE)(T_BOOL initialState);
typedef T_VOID(*SDLIB_CALLBACK_FUN_MUTEX_DELETE)(T_pVOID mutex);
typedef T_BOOL(*SDLIB_CALLBACK_FUN_MUTEX_GET)(T_pVOID mutex);
typedef T_BOOL(*SDLIB_CALLBACK_FUN_MUTEX_RELEASE)(T_pVOID mutex);
typedef T_S32(*SDLIB_CALLBACK_FUN_DUMP_DATA)(T_HANDLE buf, T_S32 size);

typedef struct sdlib_platform_dependent_list
{
    // call back functions. compatible with T_AUDIO_CB_FUNS
    MEDIALIB_CALLBACK_FUN_MALLOC                Malloc;
    MEDIALIB_CALLBACK_FUN_FREE                  Free;
    MEDIALIB_CALLBACK_FUN_PRINTF                printf;
    MEDIALIB_CALLBACK_FUN_FLUSH_DCACHE_RANGE    flushDCache;

    SDLIB_CALLBACK_FUN_MUTEX_CREATE             mutexCreate;
    SDLIB_CALLBACK_FUN_MUTEX_DELETE             mutexDelete;
    SDLIB_CALLBACK_FUN_MUTEX_GET                mutexGet;
    SDLIB_CALLBACK_FUN_MUTEX_RELEASE            mutexRelease;

    SDLIB_CALLBACK_FUN_DUMP_DATA                dumpData;

    // platform implemented routines
    T_S32(*fftdrv_Dma_Init)(void);
    T_S32(*fftdrv_Dma_Deinit)(void);
    T_S32(*fftdrv_Dma_Start)(T_S16 *inbuf, T_S16 *outbuf);
    T_S32(*fftdrv_Dma_Stop)(void);
} T_SDLIB_PLATFORM_DEPENDENT_LIST;

T_SDLIB_PLATFORM_DEPENDENT_LIST *_SD_GetPlatformDependentList(T_VOID);

/* ------ sub module interface ------ */

// fft
struct sd_entry_fft
{
    T_pVOID(*init)(T_S32 size);
    void (*destroy)(void *handle);
    void (*fft)(void *handle, T_S16 *in, T_S16 *out);      // forward fft
    void (*ifft)(void *handle, T_S16 *in, T_S16 *out);     // inverse fft
};

#ifdef __cplusplus
}
#endif

#endif// __SOUND_COMMON_H__
