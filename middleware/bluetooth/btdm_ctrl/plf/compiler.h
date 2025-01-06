/**
 ****************************************************************************************
 *
 * @file gnuarm/compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

#include "rtconfig.h"

#ifdef RISCV
    #include "nmsis_compiler.h"
#elif defined(BSP_USING_PC_SIMULATOR)
    #define __USED
#else
    #include "cmsis_compiler.h"
#endif

//#include "rtdef.h"

/// define the static keyword for this compiler
#define __STATIC static
#define __STATIC_NO_INLINE static __NOINLINE

/// define the force inlining attribute for this compiler
#undef __INLINE
#if defined (_MSC_VER)
    #define __INLINE static __inline
#else
    #define __INLINE static inline
#endif

#define __ARRAY_EMPTY

/// define the BLE IRQ handler attribute for this compiler
#define __BLEIRQ

#define __BTIRQ



/* Offset of member MEMBER in a struct of type TYPE. */
//#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)

#endif // _COMPILER_H_
