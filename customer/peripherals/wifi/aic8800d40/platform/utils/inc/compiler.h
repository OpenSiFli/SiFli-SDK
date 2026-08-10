/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC_COMPILER_H_
#define _AIC_COMPILER_H_

#ifndef __INLINE
#if defined PLATFORM_GX_ECOS || defined PLATFORM_SUNPLUS_ECOS
#define __INLINE     static __inline
#else
#define __INLINE     __inline
#endif
#endif /* _COMPILER_H_ */

#ifndef __STATIC_INLINE
#define __STATIC_INLINE   static __inline
#endif /* __STATIC_INLINE */

#if defined(__CC_ARM)
#pragma anon_unions
#endif

#include <stdio.h>

#define dbg_sprintf   sprintf
#define dbg_snprintf  snprintf
#define dbg_vsnprintf vsnprintf

#endif /* _AIC_COMPILER_H_ */
