/** @file
* @brief Define the global public types for anyka
*
* Copyright (C) 2024 GuangZhou Anyka Microelectronics Co., Ltd.
* @author
* @date 2024-01-27
* @version 3.0
*/

#ifndef __ANYKA_TYPES_H__
#define __ANYKA_TYPES_H__

#include "ak_types.h"
#include <stdint.h> // for intN_t, uintN_t
#include <stddef.h> // for size_t

/** @defgroup GLOBALTYPES global types
*    @ingroup GLOBAL
*/
/*@{*/

/* preliminary type definition for global area */
typedef    uint8_t              T_U8;        /* unsigned 8 bit integer */
typedef    uint16_t             T_U16;       /* unsigned 16 bit integer */
typedef    uint32_t             T_U32;       /* unsigned 32 bit integer */
typedef    uint64_t             T_U64;
typedef    int8_t               T_S8;        /* signed 8 bit integer */
typedef    int16_t              T_S16;       /* signed 16 bit integer */
typedef    int32_t              T_S32;       /* signed 32 bit integer */
typedef    int64_t              T_S64;
typedef    void                 T_VOID;      /* void */

#define    T_U8_MAX             ((T_U8)0xff)                // maximum T_U8 value
#define    T_U16_MAX            ((T_U16)0xffff)             // maximum T_U16 value
#define    T_U32_MAX            ((T_U32)0xffffffff)         // maximum T_U32 value
#define    T_S8_MIN             ((T_S8)(-127-1))            // minimum T_S8 value
#define    T_S8_MAX             ((T_S8)127)                 // maximum T_S8 value
#define    T_S16_MIN            ((T_S16)(-32767L-1L))       // minimum T_S16 value
#define    T_S16_MAX            ((T_S16)(32767L))           // maximum T_S16 value
#define    T_S32_MIN            ((T_S32)(-2147483647L-1L))  // minimum T_S32 value
#define    T_S32_MAX            ((T_S32)(2147483647L))      // maximum T_S32 value

/* basal type definition for global area */
typedef char                    T_CHR;      /* char */
typedef T_U8                    T_BOOL;     /* BOOL type */

typedef T_VOID                 *T_pVOID;    /* pointer of void data */
typedef const T_VOID           *T_pCVOID;   /* pointer of const void data */

typedef T_CHR                  *T_pSTR;     /* pointer of string */
typedef const T_CHR            *T_pCSTR;    /* pointer of const string */

typedef T_U16                   T_WCHR;     /* unicode char */
typedef T_U16                  *T_pWSTR;    /* pointer of unicode string */
typedef const T_U16            *T_pCWSTR;   /* pointer of const unicode string */

typedef T_U8                   *T_pDATA;    /* pointer of data */
typedef const T_U8             *T_pCDATA;   /* pointer of const data */

typedef T_U32                   T_COLOR;    /* color value */

typedef uintptr_t               T_HANDLE;   /* a handle. NOTE: it is an integer, not a ptr */

typedef T_S32                   T_POS;      /* position type */
typedef T_S32                   T_LEN;      /* length type */

#define        AK_NULL             ((T_pVOID)(0))

#define        AK_EMPTY

/* ------ Macro definition for reading/writing data from/to register ------ */
#define REG_READ_U8(_reg_)              (*((volatile T_U8 *)(_reg_)))
#define REG_READ_U16(_reg_)             (*((volatile T_U16 *)(_reg_)))
#define REG_READ_U32(_reg_)             (*((volatile T_U32 *)(_reg_)))
#define REG_WRITE_U8(_reg_, _value_)    (*((volatile T_U8 *)(_reg_)) = (_value_))
#define REG_WRITE_U16(_reg_, _value_)   (*((volatile T_U16 *)(_reg_)) = (_value_))
#define REG_WRITE_U32(_reg_, _value_)   (*((volatile T_U32 *)(_reg_)) = (_value_))

/*@}*/
#ifdef _WIN32
    #pragma warning (disable:4068)
#endif

#endif
