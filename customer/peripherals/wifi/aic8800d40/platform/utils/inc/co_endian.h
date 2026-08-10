/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file co_endian.h
 *
 * @brief Common endianness conversion functions
 *
 * Copyright (C) RivieraWaves 2011-2019
 *
 ****************************************************************************************
 */

#ifndef _CO_ENDIAN_H_
#define _CO_ENDIAN_H_

/**
 ****************************************************************************************
 * @defgroup CO_ENDIAN CO_ENDIAN
 * @ingroup COMMON
 * @brief  Endianness conversion functions.
 *
 * This set of functions converts values between the local system
 * and a external one. It is inspired from the <tt>htonl</tt>-like functions
 * from the standard C library.
 *
 * Exemple:
 * @code
 *  struct eth_header   *header = get_header();  // get pointer on Eth II packet header
 *  uint16_t            eth_id;                  // will contain the type of the packet
 *  eth_id = co_ntohs(header->eth_id);           // retrieve the type with correct endianness
 * @endcode
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "aic_plat_types.h"
#include "compiler.h"

#define CPU_LE 1
/**
 ****************************************************************************************
 * @brief Swap bytes of an array of bytes
 * .
 * The swap is done in every case. Should not be called directly.
 *
 * @param[in] p_val_out The output value.
 * @param[in] p_val_in  The input value.
 *
 * @param[in] len       number of bytes to swap
 ****************************************************************************************
 */
__STATIC_INLINE void co_bswap(uint8_t* p_val_out, const uint8_t* p_val_in, uint16_t len)
{
    while (len > 0)
    {
        len--;
        *p_val_out = p_val_in[len];
        p_val_out++;
    }
}
/// @} CO_ENDIAN

/**
 ****************************************************************************************
 * @brief Swap bytes of a 32 bits value.
 * The swap is done in every case. Should not be called directly.
 * @param[in] val32 The 32 bits value to swap.
 * @return The 32 bit swapped value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_bswap32(uint32_t val32)
{
    // GCC 4 define __builtin_bswap32(val32);
    return (val32<<24) | ((val32<<8)&0xFF0000) | ((val32>>8)&0xFF00) | ((val32>>24)&0xFF);
    //return __REV(val32);
}

/**
 ****************************************************************************************
 * @brief Swap bytes of a 24 bits value.
 * The swap is done in every case. Should not be called directly.
 * @param[in] val24 The 24 bits value to swap.
 * @return The 24 bit swapped value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_bswap24(uint32_t val24)
{
    return ((val24<<16)&0xFF0000) | ((val24)&0xFF00) | ((val24>>16)&0xFF);
}

/**
 ****************************************************************************************
 * @brief Swap bytes of a 16 bits value.
 * The swap is done in every case. Should not be called directly.
 * @param[in] val16 The 16 bit value to swap.
 * @return The 16 bit swapped value.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_bswap16(uint16_t val16)
{
    return ((val16<<8)&0xFF00) | ((val16>>8)&0xFF);
    //return (uint16_t)__REVSH((int16_t)val16);
}
/// @} CO_ENDIAN


/** @defgroup CO_ENDIAN_NET Endianness (Network)
 *  @ingroup CO_ENDIAN
 *  @brief Endianness conversion functions for Network data
 *
 *  Converts values between the local system and big-endian network data
 *  (e.g. IP, Ethernet, but NOT WLAN).
 *
 *  The \b host term in the descriptions of these functions refers
 *  to the local system, i.e. \b application or \b embedded system.
 *  Therefore, these functions will behave differently depending on which
 *  side they are used. The reason of this terminology is to keep the
 *  same name than the standard C function.
 *
 *  Behavior will depends on the endianness of the host:
 *  - little endian: swap bytes;
 *  - big endian: identity function.
 *  @addtogroup CO_ENDIAN_NET
 *  @{ */

/**
 ****************************************************************************************
 * @brief Convert host to network long word.
 *
 * @param[in] hostlong Long word value to convert.
 * @return The converted long word.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_htonl(uint32_t hostlong)
{
    #if !CPU_LE
        return hostlong;
    #else
        return co_bswap32(hostlong);
    #endif
}

/**
 ****************************************************************************************
 * @brief Convert host to network long 24-bit value.
 *
 * @param[in] val24    24-bit value to convert.
 *
 * @return The converted 24-but value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_hton24(uint32_t host24)
{
    #if (!CPU_LE)
        return host24;
    #else
        return co_bswap24(host24);
    #endif // CPU_LE
}

/**
 ****************************************************************************************
 * @brief Convert host to network short word.
 *
 * @param[in] hostshort Short word value to convert.
 * @return The converted short word.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_htons(uint16_t hostshort)
{
    #if !CPU_LE
        return hostshort;
    #else
        return co_bswap16(hostshort);
    #endif
}

/**
 ****************************************************************************************
 * @brief Convert network to host long word.
 *
 * @param[in] netlong Long word value to convert.
 * @return The converted long word.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_ntohl(uint32_t netlong)
{
    return co_htonl(netlong);
}

/**
 ****************************************************************************************
 * @brief Convert network to host 24-bit value.
 *
 * @param[in] val24 24-bit to convert.
 *
 * @return The converted 24-bit value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_ntoh24(uint32_t val24)
{
    return co_hton24(val24);
}

/**
 ****************************************************************************************
 * @brief Convert network to host short word.
 *
 * @param[in] netshort Short word value to convert.
 * @return The converted short word.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_ntohs(uint16_t netshort)
{
    return co_htons(netshort);
}
/// @} CO_ENDIAN_NET

/** @defgroup CO_ENDIAN_WLAN Endianness (WLAN)
 *  @ingroup CO_ENDIAN
 *  @brief Endianness conversion functions for WLAN data
 *
 *  Converts values between the local system and little-endian WLAN data.
 *
 *  The \b host term in the descriptions of these functions refers
 *  to the local system (check \ref CO_ENDIAN_NET "this comment").
 *
 *  Behavior will depends on the endianness of the host:
 *  - little endian: identity function;
 *  - big endian: swap bytes.
 *
 *  @addtogroup CO_ENDIAN_WLAN
 *  @{ */

/**
 ****************************************************************************************
 * @brief Convert host to wlan long word.
 *
 * @param[in] hostlong Long word value to convert.
 * @return The converted long word.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_htowl(uint32_t hostlong)
{
    #if CPU_LE
        return hostlong;
    #else
        return co_bswap32(hostlong);
    #endif
}

/**
 ****************************************************************************************
 * @brief Convert host to wlan short word.
 *
 * @param[in] hostshort Short word value to convert.
 * @return The converted short word.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_htows(uint16_t hostshort)
{
    #if CPU_LE
        return hostshort;
    #else
        return co_bswap16(hostshort);
    #endif
}


/**
 ****************************************************************************************
 * @brief Convert wlan to host long word.
 *
 * @param[in] wlanlong Long word value to convert.
 * @return The converted long word.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_wtohl(uint32_t wlanlong)
{
    return co_htowl(wlanlong);
}


/**
 ****************************************************************************************
 * @brief Convert wlan to host short word.
 *
 * @param[in] wlanshort Short word value to convert.
 * @return The converted short word.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_wtohs(uint16_t wlanshort)
{
    return co_htows(wlanshort);
}
/// @} CO_ENDIAN_WLAN

#endif // _CO_ENDIAN_H_
