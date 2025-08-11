/**
 * @file medialib_global.h
 * @brief Define the global public types for media lib, video lib and audio lib
 *
 * Copyright (C) 2020 Anyka (Guangzhou) Microelectronics Technology Co., Ltd.
 * @update date 2020-06-11
 * @version 4.3
 */

#ifndef _MEDIA_LIB_GLOBAL_H_
#define _MEDIA_LIB_GLOBAL_H_

#include "anyka_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    T_U16   ResourceID;
    T_U8    *Buff;
    T_U32   Resource_len;
} T_AUDIO_LOADRESOURCE_CB_PARA;

typedef T_VOID(*MEDIALIB_CALLBACK_FUN_PRINTF)(T_pCSTR format, ...);

#if 0
typedef T_S32   SEM_ID;
typedef SEM_ID(*MEDIALIB_CALLBACK_FUN_SEM_CREATE)(T_U32 nSemType, T_U32 nMaxLockCount);
typedef T_BOOL(*MEDIALIB_CALLBACK_FUN_SEM_TAKE)(SEM_ID semID, T_S32 nTimeOut);
typedef T_BOOL(*MEDIALIB_CALLBACK_FUN_SEM_GIVE)(SEM_ID semID);
typedef T_BOOL(*MEDIALIB_CALLBACK_FUN_SEM_FLUSH)(SEM_ID semID);
typedef T_BOOL(*MEDIALIB_CALLBACK_FUN_SEM_DELETE)(SEM_ID semID);
#endif

// file operation
typedef T_S32(*MEDIALIB_CALLBACK_FUN_OPEN)(T_pVOID lpData);
typedef T_S32(*MEDIALIB_CALLBACK_FUN_READ)(T_S32 hFile, T_pVOID buf, T_S32 size);
typedef T_S32(*MEDIALIB_CALLBACK_FUN_WRITE)(T_S32 hFile, T_pVOID buf, T_S32 size);
typedef T_S32(*MEDIALIB_CALLBACK_FUN_SEEK)(T_S32 hFile, T_S32 offset, T_S32 whence);
typedef T_S32(*MEDIALIB_CALLBACK_FUN_TELL)(T_S32 hFile);
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_CLOSE)(T_S32 hFile);
typedef T_S32(*MEDIALIB_CALLBACK_FUN_FILE_HANDLE_EXIST)(T_S32 hFile);
typedef T_U32(*MEDIALIB_CALLBACK_FUN_FILE_GET_LENGTH)(T_S32 hFile);
typedef T_BOOL(*MEDIALIB_CALLBACK_FUN_FILESYS_ISBUSY)(void);

// memory operation
typedef T_pVOID(*MEDIALIB_CALLBACK_FUN_MALLOC)(T_U32 size);
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_FREE)(T_pVOID mem);
typedef T_pVOID(*MEDIALIB_CALLBACK_FUN_DMA_MALLOC)(T_U32 size);
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_DMA_FREE)(T_pVOID mem);
typedef T_pVOID(*MEDIALIB_CALLBACK_FUN_DMA_MEMCPY)(T_pVOID dest, T_pCVOID src, T_U32 size);
typedef T_pVOID(*MEDIALIB_CALLBACK_FUN_VADDR_TO_PADDR)(T_pVOID mem);
typedef T_U32(*MEDIALIB_CALLBACK_FUN_MAP_ADDR)(T_U32 phyAddr, T_U32 size);
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_UNMAP_ADDR)(T_U32 addr, T_U32 size);

// register operation
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_REG_BITS_WRITE)(T_U32 phyAddr, T_U32 val, T_U32 mask);

// Dcache operation
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_FLUSH_DCACHE)(void);   // clean and invalidate D-cache
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_FLUSH_DCACHE_RANGE)(T_U32 start, T_U32 size);    // start should be 32byte aligned
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_INVALID_DCACHE)(T_VOID);    // invalidate D-cache
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_INVALID_DCACHE_RANGE)(T_U32 start, T_U32 size);

// hardware mutex
typedef T_pVOID(*MEDIALIB_CALLBACK_FUN_HW_LOCK)(T_S32 hw_id);
typedef T_S32(*MEDIALIB_CALLBACK_FUN_HW_UNLOCK)(T_pVOID hLock);

// others
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_LOADRESOURCE)(T_AUDIO_LOADRESOURCE_CB_PARA *pPara);    //载入资源
typedef T_VOID(*MEDIALIB_CALLBACK_FUN_RELEASERESOURCE)(T_U8 *Buff);      //释放载入的资源

typedef T_BOOL(*MEDIALIB_CALLBACK_FUN_RTC_DELAY)(T_U32 ulTicks);


typedef struct
{
    T_S32   real_time;

    union
    {
        struct
        {
            T_U32 nCurBitIndex;  //不需要用
            T_U32 nFrameIndex;   //第几帧
        } m_ape;
        struct
        {
            T_U32 Indx;          //seek位置开始，当前page中有的packet数
            T_U32 offset;        //seek后，当前page剩余需要解码的数据大小
            T_BOOL flag;         //seek标志位，seek时置1
            T_U32  last_granu;   //解码完上一个page时解出的总sample数
            T_U32  now_granu;    //解码完当前page时接出的总sample数
            T_BOOL is_eos;       //码流结束标志位
            T_U32  re_data;      //seek后，当前page含有不完整packet的大小
            T_U32 pack_no;       //seek的位置在当前page中的第几个packet
            T_U32 list[255];     //记录一个ogg page中每个packet的偏移位置
        } m_speex;
        struct
        {
            T_U32 Indx;          //seek位置开始，当前page中有的packet数
            T_U32 offset;        //seek后，当前page剩余需要解码的数据大小
            T_BOOL flag;         //seek标志位，seek时置1
            T_U32  last_granu;   //解码完上一个page时解出的总sample数
            T_U32  now_granu;    //解码完当前page时接出的总sample数
            T_BOOL is_eos;       //码流结束标志位
            T_U32  re_data;      //seek后，当前page含有不完整packet的大小
            T_U32 pack_no;       //seek的位置在当前page中的第几个packet
            T_U32 list[255];     //记录一个ogg page中每个packet的偏移位置
        } m_speexwb;
        struct
        {
            T_U8    secUse;     //已经读取的section数目
            T_U8    secLen;     //一个page中包含的section数
            T_U8    tmpSec;     //已经解码的section数目
            T_BOOL  is_eos;     //是不是最后一个page
            T_BOOL  is_bos;     //是不是第一个page
            T_U8    endpack;    //当前page中最后一个packet的位置
            //解码出的sample数是一个64位的数，目前只取低32位
            T_U32   gos;        //解码完当前page后解码出总的sample数，低32位
            T_U32   high_gos;   //解码完当前page后解码出总的sample数，高32位，(暂时不用，留给以后需要)
            T_U8    list[255];  //记录一个page中每个section的大小，一个page中最多含有255个section
        } m_vorbis;
        struct
        {
            T_U32 Indx;         //seek位置开始，当前page中有的packet数
            T_U32 offset;       //seek后，当前page剩余需要解码的数据大小
            T_BOOL flag;        //seek标志位，seek时置1
            T_U32  last_granu;  //解码完上一个page时解出的总sample数
            T_U32  now_granu;   //解码完当前page时接出的总sample数
            T_BOOL is_eos;      //码流结束标志位
            T_U32  re_data;     //seek后，当前page含有不完整packet的大小
            T_U32 pack_no;      //seek的位置在当前page中的第几个packet
            T_U32 list[255];    //记录一个ogg page中每个packet的偏移位置
        } m_opus;
    } m_Private;
} T_AUDIO_SEEK_INFO;

#ifdef __cplusplus
}
#endif

#endif//_MEDIA_LIB_GLOBAL_H_
