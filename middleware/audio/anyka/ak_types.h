/** @file
* @brief Define the global public types for anyka
*
* Copyright (C) 2006 Anyka (GuangZhou) Software Technology Co., Ltd.
* @author
* @date 2006-01-16
* @version 1.0
*/

#ifndef __AK_TYPES_H__
#define __AK_TYPES_H__

#include <stdint.h> // for intN_t, uintN_t
#include <stddef.h> // for size_t

/** @defgroup GLOBALTYPES global types
*    @ingroup GLOBAL
*/
/*@{*/

/* preliminary type definition for global area */
typedef    uint8_t                  AK_U8;          /* unsigned 8 bit integer */
typedef    char                     AK_CHAR;        /* char */
typedef    uint16_t                 AK_U16;         /* unsigned 16 bit integer */
typedef    uint32_t                 AK_U32;         /* unsigned 32 bit integer */
typedef    double                   AK_DOUBLE;
#ifdef _MSC_VER
    typedef    unsigned __int64         AK_U64;         /* unsigned 64 bit integer */
#else
    typedef    uint64_t                 AK_U64;         /* unsigned 64 bit integer */
#endif
typedef    int8_t                   AK_S8;          /* signed 8 bit integer */
typedef    int16_t                  AK_S16;         /* signed 16 bit integer */
typedef    int32_t                  AK_S32;         /* signed 32 bit integer */
#ifdef _MSC_VER
    typedef    __int64                  AK_S64;         /* signed 64 bit integer */
#else
    typedef    int64_t                  AK_S64;         /* signed 64 bit integer */
#endif
typedef    void                     AK_VOID;        /* void */
typedef    volatile uint32_t        AK_UINT32;
typedef     uintptr_t               AK_HANDLE;

// maximum AK_U8 value
#define    AK_U8_MAX             ((AK_U8)0xff)
// maximum AK_U16 value
#define    AK_U16_MAX            ((AK_U16)0xffff)
// maximum AK_U32 value
#define    AK_U32_MAX            ((AK_U32)0xffffffff)
// maximum AK_U64 value
#define    AK_U64_MAX            ((AK_U64)0xffffffffffffffff)
// minimum AK_S8 value
#define    AK_S8_MIN             ((AK_S8)(-127-1))
// maximum AK_S8 value
#define    AK_S8_MAX             ((AK_S8)127)
// minimum AK_S16 value
#define    AK_S16_MIN            ((AK_S16)(-32767L-1L))
// maximum AK_S16 value
#define    AK_S16_MAX            ((AK_S16)(32767L))
// minimum AK_S32 value
#define    AK_S32_MIN            ((AK_S32)(-2147483647L-1L))
// maximum AK_S32 value
#define    AK_S32_MAX            ((AK_S32)(2147483647L))
// minimum AK_S64 value
#define    AK_S64_MIN            ((AK_S64)(-9223372036854775807LL-1LL))
// maximum AK_S64 value
#define    AK_S64_MAX            ((AK_S64)(9223372036854775807LL))


#define AK_VOID                  void

typedef    uint8_t              AK_BOOL;
#define AK_FALSE                (0)
#define AK_TRUE                 (1)


#define AK_EMPTY

/*@}*/

#endif
