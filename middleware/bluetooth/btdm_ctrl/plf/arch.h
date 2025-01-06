/**
 ****************************************************************************************
 *
 * @file arch.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * architecture dependent.  The implementation of those is implemented in the
 * appropriate architecture directory.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _ARCH_H_
#define _ARCH_H_

/**
 ****************************************************************************************
 * @defgroup REFIP
 * @brief Reference IP Platform
 *
 * This module contains reference platform components - REFIP.
 *
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup DRIVERS
 * @ingroup REFIP
 * @brief Reference IP Platform Drivers
 *
 * This module contains the necessary drivers to run the platform with the
 * RW BT SW protocol stack.
 *
 * This has the declaration of the platform architecture API.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"       // SW configuration
#include "rw_porting.h"
#include <stdint.h>        // standard integer definition
//#include "rthw.h"
#include "rtconfig.h"      // Uart configure is depended on project
#include "compiler.h"      // inline functions


/*
 * CPU WORD SIZE
 ****************************************************************************************
 */
/// ARM is a 32-bit CPU
#define CPU_WORD_SIZE   4

/*
 * CPU Endianness
 ****************************************************************************************
 */
/// ARM is little endian
#define CPU_LE          1

/*
 * DEBUG configuration
 ****************************************************************************************
 */
#if defined(CFG_DBG)
    #define PLF_DEBUG          1
#else //CFG_DBG
    #define PLF_DEBUG          0
#endif //CFG_DBG


/*
 * NVDS
 ****************************************************************************************
 */

/// NVDS
#ifdef CFG_NVDS
    #define PLF_NVDS             1
#else // CFG_NVDS
    #define PLF_NVDS             0
#endif // CFG_NVDS


/*
 * UART
 ****************************************************************************************
 */

/// UART
#define PLF_UART             0


/// UART 2
#ifdef SOC_BF_Z0
    #undef USING_IPC_QUEUE
    #define USING_IPC_QUEUE RT_USING_HWMAILBOX
#endif
#ifdef USING_IPC_QUEUE
    #ifdef AHI_TL_SUPPORT
        #define PLF_UART2_OVERMBOX   1
    #else // AHI_TL_SUPPORT
        #define PLF_UART2            1
    #endif // AHI_TL_SUPPORT
#else // RT_USING_HWMAILBOX
    #define PLF_UART2            1
    #define PLF_UART2_OVERMBOX   1
#endif // RT_USING_HWMAILBOX

/*
 * DEFINES
 ****************************************************************************************
 */

/// Possible errors detected by FW
#define    RESET_NO_ERROR         0x00000000
#define    RESET_MEM_ALLOC_FAIL   0xF2F2F2F2

/// Reset platform and stay in ROM
#define    RESET_TO_ROM           0xA5A5A5A5
/// Reset platform and reload FW
#define    RESET_AND_LOAD_FW      0xC3C3C3C3

/// Exchange memory size limit
#if (BT_DUAL_MODE)
    #define    EM_SIZE_LIMIT          0x8000
#else
    #define    EM_SIZE_LIMIT          0x8000
#endif

/*
 * EXPORTED FUNCTION DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Compute size of SW stack used.
 *
 * This function is compute the maximum size stack used by SW.
 *
 * @return Size of stack used (in bytes)
 ****************************************************************************************
 */
uint16_t get_stack_usage(void);

/**
 ****************************************************************************************
 * @brief Re-boot FW.
 *
 * This function is used to re-boot the FW when error has been detected, it is the end of
 * the current FW execution.
 * After waiting transfers on UART to be finished, and storing the information that
 * FW has re-booted by itself in a non-loaded area, the FW restart by branching at FW
 * entry point.
 *
 * Note: when calling this function, the code after it will not be executed.
 *
 * @param[in] error      Error detected by FW
 ****************************************************************************************
 */
void platform_reset(uint32_t error);

void platform_assert(void);


#if PLF_DEBUG
    /**
    ****************************************************************************************
    * @brief Print the assertion error reason and loop forever.
    *
    * @param condition C string containing the condition.
    * @param file C string containing file where the assertion is located.
    * @param line Line number in the file where the assertion is located.
    ****************************************************************************************
    */
    void rw_assert_err(const char *condition, const char *file, int line);

    /**
    ****************************************************************************************
    * @brief Print the assertion error reason and loop forever.
    * The parameter value that is causing the assertion will also be disclosed.
    *
    * @param param0 parameter value 0.
    * @param param1 parameter value 1.
    * @param file C string containing file where the assertion is located.
    * @param line Line number in the file where the assertion is located.
    ****************************************************************************************
    */
    void rw_assert_param(int param0, int param1, const char *file, int line);

    /**
    ****************************************************************************************
    * @brief Print the assertion warning reason.
    *
    * @param param0 parameter value 0.
    * @param param1 parameter value 1.
    * @param file C string containing file where the assertion is located.
    * @param line Line number in the file where the assertion is located.
    ****************************************************************************************
    */
    void rw_assert_warn(int param0, int param1, const char *file, int line);


    /**
    ****************************************************************************************
    * @brief Dump data value into FW.
    *
    * @param data start pointer of the data.
    * @param length data size to dump
    ****************************************************************************************
    */
    void dump_data(uint8_t *data, uint16_t length);
#endif //PLF_DEBUG

#ifdef __GNUC__
    #define __MODULE__ __FILE__
#endif

/*
 * ASSERTION CHECK
 ****************************************************************************************
 */
#if PLF_DEBUG
void dump_hci(uint8_t type, uint8_t direction, uint8_t *p_data, uint16_t length, uint8_t *p_hdr_data, uint16_t hdr_length);

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)                              \
    do {                                              \
        if (!(cond)) {                                \
            rw_assert_err("ASS", __FUNCTION__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            rw_assert_param((int)param0, (int)param1, __FUNCTION__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            rw_assert_warn((int)param0, (int)param1, __FUNCTION__, __LINE__); \
        }                                             \
    } while(0)

#define ASSERT_ERR_KE(cond)                              \
    do {                                              \
        if (!(cond)) {                                \
            rw_assert_err("ASS", __FUNCTION__, __LINE__);  \
        }                                             \
    } while(0)

#define ASSERT_INFO_KE(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            rw_assert_param((int)param0, (int)param1, __FUNCTION__, __LINE__);  \
        }                                             \
    } while(0)

#define DUMP_DATA(data, length) \
    dump_data((uint8_t*)data, length)

/// DUMP HCI packet
#define DUMP_HCI(type, direction, data, length)\
    dump_hci(type, direction, (uint8_t*)data, length, NULL, 0)

/// DUMP HCI packet
#define DUMP_HCI_2(type, direction, hdr_data, hdr_length, data, length)\
    dump_hci(type, direction, (uint8_t*)data, length, (uint8_t*)hdr_data, hdr_length)

/// DUMP HCI packet in unpacked format
#define DUMP_UPK_HCI(evttype, direction, code, data, length)\
    dump_upk_hci(evttype, direction, code, (uint8_t*)data, length)

/// DUMP String using printf format
#define DUMP_STR(level, format, ...) \
    dump_str(level, "%s:%u: " format, __FILE__, __LINE__, ## __VA_ARGS__)

/// Print a string using printf format when argument list is available
#define PRINT_STR(level, format, arg_list) \
    print_str(level, (uint8_t**)&format)

#else
/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1) RT_ASSERT(cond)

#define ASSERT_ERR_KE(cond)        RT_ASSERT(cond)

#define ASSERT_INFO_KE(cond, param0, param1)        RT_ASSERT(cond)


/// DUMP data array present in the SW.
#define DUMP_DATA(data, length)


/// DUMP HCI packet
#define DUMP_HCI(type, direction, data, length)
/// DUMP HCI packet
#define DUMP_HCI_2(type, direction, hdr_data, hdr_length, data, length)
/// DUMP HCI packet in unpacked format
#define DUMP_UPK_HCI(evttype, direction, code, data, length)
/// DUMP String using printf format
#define DUMP_STR(level, format, ...)
/// Print a string using printf format when argument list is available
#define PRINT_STR(level, format, arg_list)

#endif //PLF_DEBUG

#if defined(CFG_SLIM)
    #define ASSERT_ERR_FORCE(cond) platform_assert()
    #define ASSERT_INFO_FORCE(cond, param0, param1) platform_assert()
#elif defined(PLF_DEBUG)
    #define ASSERT_ERR_FORCE(cond) ASSERT_ERR(cond)
    #define ASSERT_INFO_FORCE(cond, param0, param1) ASSERT_INFO(cond, param0, param1)
#else
    #define ASSERT_ERR_FORCE(cond)
    #define ASSERT_INFO_FORCE(cond, param0, param1)
#endif

/// Trace data allocation
#define DBG_DATA_ALLOC(...)

/// Trace data free
#define DBG_DATA_FREE(data)

/// Trace Function Enter
#define DBG_FUNC_ENTER(func)

/// Trace Function Exit
#define DBG_FUNC_EXIT(func)

/// Control memory access
#define DBG_MEM_GRANT_CTRL(mem_ptr, enable)

/// Set permission onto a specific memory block
#define DBG_MEM_PERM_SET(mem_ptr, size, write_en, read_en, init_clr)

/// Mark memory initialized
#define DBG_MEM_INIT(mem_ptr, size)


/// Object allocated in shared memory - check linker script
//#define __SHARED __attribute__ ((section("shram")))

// required to define GLOBAL_INT_** macros as inline assembly. This file is included after
// definition of ASSERT macros as they are used inside ll.h
//#include "ll.h"     // ll definitions

/// @} DRIVERS
#endif // _ARCH_H_
