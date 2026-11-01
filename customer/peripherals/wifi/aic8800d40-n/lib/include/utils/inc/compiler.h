/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC_COMPILER_H_
#define _AIC_COMPILER_H_

#ifndef __INLINE

    #define __INLINE     //inline
#endif /* _COMPILER_H_ */

#ifndef __STATIC_INLINE
    #define __STATIC_INLINE   static inline
#endif /* __STATIC_INLINE */


#if defined(__CC_ARM)
    #pragma anon_unions
#endif

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #define DECLARE_ALIGNED_64() _Pragma("data_alignment=64")
    #define DECLARE_ALIGNED_32() _Pragma("data_alignment=32")
#else
    #define __ALIGN__(n)
#endif


#ifndef dbg_vsnprintf
    #define dbg_vsnprintf vsnprintf
#endif
#ifndef dbg_vsprintf
    #define dbg_vsprintf  vsprintf
#endif

/* 注: aicwf_dbg_snprintf 为平台 porting 接口 (plat_port.h 声明, plat_port.c 实现),
 * 驱动侧直接调用该平台函数, 不在此定义同名宏, 避免与 stdio/平台函数声明冲突 */
#endif /* _AIC_COMPILER_H_ */
