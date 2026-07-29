/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _CO_UTILS_H_
#define _CO_UTILS_H_

/**
 ****************************************************************************************
 * @defgroup CO_UTILS CO_UTILS
 * @ingroup COMMON
 * @brief  Common utilities
 *
 * This module contains the common utilities functions and macros
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "compiler.h"
#include <limits.h>
#include "co_math.h"

/*
 * MACROS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Converts a STA index in an Association Index.
 * @param[in] sta_idx The station index.
 * @return The association index
 ****************************************************************************************
 */
#define CO_STAIDX_TO_AID(sta_idx) ((sta_idx) + 1)

/**
 ****************************************************************************************
 * @brief Get the index of an element in an array.
 * @param[in] __element_ptr Pointer to the element
 * @param[in] __array_ptr Pointer to the array
 * @return The index of the element
 ****************************************************************************************
 */
#define CO_GET_INDEX(__element_ptr, __array_ptr) ((__element_ptr) - (__array_ptr))

/**
 ****************************************************************************************
 * @brief Get the number of element in an array.
 * @param[in] a  Pointer to the array
 * @return The number of the element
 ****************************************************************************************
 */
#define CO_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/// Length of a char in bytes
#define CHAR_LEN    (CHAR_BIT/8)

/// Pack a structure field
#define __PACKED16 __attribute__ ((__packed__))
#ifndef __PACKED
    #define __PACKED __attribute__ ((__packed__))
#endif

/**
 ****************************************************************************************
 * @brief Converts a CPU pointer into a HW address
 * This macro is used to convert a SW pointer into the corresponding HW address. With CPUs
 * having native byte support, the value returned will be the same as the pointer passed.
 * With TL4, the value returned is the pointer multiplied by 2.
 * @param[in] ptr Pointer to be converted
 * @return The corresponding HW address
 ****************************************************************************************
 */
#define CPU2HW(ptr) (((uint32_t)(ptr)) * CHAR_LEN)

/**
 ****************************************************************************************
 * @brief Converts a HW address into a CPU pointer
 * This macro is doing the reverse operation as @ref CPU2HW.
 * @param[in] ptr Address to be converted
 * @return The corresponding CPU pointer
 ****************************************************************************************
 */
#define HW2CPU(ptr) ((void *)(((uint32_t)(ptr)) / CHAR_LEN))

/**
 ****************************************************************************************
 * @brief Return the size of a variable or type in bytes
 * @param[in] a Variable for which the size is computed
 * @return The size of the variable in bytes
 ****************************************************************************************
 */
#define sizeof_b(a) (sizeof(a) * CHAR_LEN)

/**
 ****************************************************************************************
 * @brief Return the offset (in bytes) of a structure element
 * @param[in] a Structure type
 * @param[in] b Field name
 * @return The of the field in bytes
 ****************************************************************************************
 */
#define offsetof_b(a, b)  (offsetof(a, b) * CHAR_LEN)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Read an aligned 16 bits word.
 * @param[in] ptr16 The address of the first byte of the 16 bits word.
 * @return The 16 bits value.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_read16(void const *ptr16)
{
    return *((uint16_t *)ptr16);
}

/**
 ****************************************************************************************
 * @brief Write an aligned 16 bits word.
 * @param[in] ptr16 The address of the first byte of the 16 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__STATIC_INLINE void co_write16(void const *ptr16, uint32_t value)
{
    *(uint16_t *)ptr16 = value;
}

/**
 ****************************************************************************************
 * @brief Read an aligned 32 bit word.
 * @param[in] ptr32 The address of the first byte of the 32 bit word.
 * @return The 32 bit value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_read32(void const *ptr32)
{
    return *((uint32_t *)ptr32);
}

/**
 ****************************************************************************************
 * @brief Write an aligned 32 bits word.
 * @param[in] ptr32 The address of the first byte of the 32 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__STATIC_INLINE void co_write32(void const *ptr32, uint32_t value)
{
    *(uint32_t *)ptr32 = value;
}

/**
 ****************************************************************************************
 * @brief Copy a 32-bit aligned buffer into another one
 * The length in bytes is converted to the corresponding number of 32-bit words. If the
 * byte length is not a multiple of 4, then additional bytes will be copied at the end
 * of the buffer. It is the responsibility of the caller to ensure that these extra-byte
 * copy won't corrupt the memory at the end of the destination buffer.
 *
 * @param[in] dst Pointer to the destination buffer
 * @param[in] src Pointer to the source buffer
 * @param[in] len Length to be copied (in bytes)
 ****************************************************************************************
 */
__STATIC_INLINE void co_copy32(uint32_t *dst, uint32_t *src, uint32_t len)
{
    len = CO_ALIGN4_HI(len) / 4;
    while (len--)
    {
        *dst++ = *src++;
    }
}

/**
 ****************************************************************************************
 * @brief Read a 8 bits word.
 * @param[in] addr The address of the first byte of the 8 bits word.
 * @return The read value
 ****************************************************************************************
 */
__STATIC_INLINE uint8_t co_read8p(uint32_t addr)
{
#ifdef CFG_RWTL
    int shift = (addr & 0x1) * 8;
    uint16_t *ptr = (uint16_t *)(addr / 2);
    return ((uint8_t)((*ptr >> shift) & 0xFF));
#else
    return (*(uint8_t *)addr);
#endif
}

/**
 ****************************************************************************************
 * @brief Write a 8 bits word.
 * @param[in] addr The address of the first byte of the 8 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__STATIC_INLINE void co_write8p(uint32_t addr, uint8_t value)
{
#ifdef CFG_RWTL
    int shift = (addr & 0x1) * 8;
    uint16_t *ptr = (uint16_t *)(addr / 2);
    *ptr = (*ptr & ~(0xFF << shift)) | ((value & 0xFF) << shift);
#else
    *(uint8_t *)addr = value;
#endif
}


/**
 ****************************************************************************************
 * @brief Read a packed 16 bits word.
 * @param[in] addr The address of the first byte of the 16 bits word.
 * @return The 16 bits value.
 ****************************************************************************************
 */
__STATIC_INLINE uint16_t co_read16p(uint32_t addr)
{
#ifdef CFG_RWTL
    return ((((uint16_t)co_read8p(addr + 1)) << 8) | co_read8p(addr));
#else
    struct co_read16_struct
    {
        uint16_t val __PACKED16;
    } *ptr = (struct co_read16_struct *)addr;
    return ptr->val;
#endif
}

/**
 ****************************************************************************************
 * @brief Write a packed 16 bits word.
 * @param[in] addr The address of the first byte of the 16 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__STATIC_INLINE void co_write16p(uint32_t addr, uint32_t value)
{
#ifdef CFG_RWTL
    co_write8p(addr, value & 0xFF);
    co_write8p(addr + 1, ((value >> 8) & 0xFF));
#else
    struct co_read16_struct
    {
        uint16_t val __PACKED16;
    } *ptr = (struct co_read16_struct *) addr;

    ptr->val = value;
#endif
}

/**
 ****************************************************************************************
 * @brief Read a packed 24 bits word.
 * @param[in] addr The address of the first byte of the 24 bits word.
 * @return The 24 bits value, on a 32-bit variable.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_read24p(uint32_t addr)
{
    return ((((uint32_t)co_read16p(addr + 1)) << 8) | co_read8p(addr));
}

/**
 ****************************************************************************************
 * @brief Write a packed 24 bits word.
 * @param[in] addr The address of the first byte of the 24 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__STATIC_INLINE void co_write24p(uint32_t addr, uint32_t value)
{
    co_write8p(addr, value & 0xFF);
    co_write16p(addr + 1, ((value >> 8) & 0xFFFF));
}

/**
 ****************************************************************************************
 * @brief Read a packed 32 bits word.
 * @param[in] addr The address of the first byte of the 32 bits word.
 * @return The 32 bits value.
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t co_read32p(uint32_t addr)
{
#ifdef CFG_RWTL
    return ((((uint32_t)co_read16p(addr + 2)) << 16) | co_read16p(addr));
#else
    struct co_read32_struct
    {
        uint32_t val __PACKED;
    } *ptr = (struct co_read32_struct *) addr;
    return ptr->val;
#endif
}

/**
 ****************************************************************************************
 * @brief Write a packed 32 bits word.
 * @param[in] addr The address of the first byte of the 32 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__STATIC_INLINE void co_write32p(uint32_t addr, uint32_t value)
{
#ifdef CFG_RWTL
    co_write16p(addr, value & 0xFFFF);
    co_write16p(addr + 2, ((value >> 16) & 0xFFFF));
#else
    struct co_read32_struct
    {
        uint32_t val __PACKED;
    } *ptr = (struct co_read32_struct *) addr;
    ptr->val = value;
#endif
}

/**
 ****************************************************************************************
 * @brief Read a packed 64 bits word.
 * @param[in] addr The address of the first byte of the 64 bits word.
 * @return The 64 bits value.
 ****************************************************************************************
 */
__STATIC_INLINE uint64_t co_read64p(uint32_t addr)
{
#ifdef CFG_RWTL
    return ((((uint64_t)co_read32p(addr + 4)) << 32) | co_read32p(addr));
#else
    struct co_read64_struct
    {
        uint64_t val __PACKED;
    } *ptr = (struct co_read64_struct *) addr;
    return ptr->val;
#endif
}

/**
 ****************************************************************************************
 * @brief Compare a packed byte buffer with a CPU byte array
 * @param[in] pkd The address of the packed buffer.
 * @param[in] ptr Pointer to the CPU byte array
 * @param[in] len Length to be compared
 *
 * @return true if the buffers are equal, false otherwise
 ****************************************************************************************
 */
__STATIC_INLINE bool co_cmp8p(uint32_t pkd, uint8_t const *ptr, uint32_t len)
{
    while (len--)
    {
        if (co_read8p(pkd++) != (*ptr++ & 0xFF))
            return false;
    }
    return true;
}

/**
 ****************************************************************************************
 * @brief Copy and pack a byte array to another one
 * @param[in] dst The address of the first byte of the packed buffer in which the data has
 *                to be copied.
 * @param[in] src Pointer to the source buffer
 * @param[in] len Length to be copied
 ****************************************************************************************
 */
__STATIC_INLINE void co_pack8p(uint32_t dst, uint8_t const *src, uint32_t len)
{
    while (len--)
    {
        co_write8p(dst++, *src++);
    }
}

/**
 ****************************************************************************************
 * @brief Copy and unpack a byte array to another one
 * @param[in] dst Pointer to the first byte of the unpacked buffer in which the data has
 *                to be copied.
 * @param[in] src Address of the packed source buffer
 * @param[in] len Length to be copied
 ****************************************************************************************
 */
__STATIC_INLINE void co_unpack8p(uint8_t *dst, uint32_t src, uint32_t len)
{
    while (len--)
    {
        *dst++ = co_read8p(src++);
    }
}

/**
 ****************************************************************************************
 * @brief Copy a packed byte array to another packed byte array
 * @param[in] dst The address of the first byte of the packed buffer in which the data has
 *                to be copied.
 * @param[in] src Address of the source buffer
 * @param[in] len Length to be copied
 ****************************************************************************************
 */
__STATIC_INLINE void co_copy8p(uint32_t dst, uint32_t src, uint32_t len)
{
    while (len--)
    {
        co_write8p(dst++, co_read8p(src++));
    }
}

/**
 ****************************************************************************************
 * @brief This function returns the value of bit field inside an array of bits,
 * represented as an array of bytes.
 * @param[in] array Array of bits
 * @param[in] lsb Position of the LSB of the field inside the array of bits
 * @param[in] width Width of the field
 * @return true if the specified bit is set, false otherwise
 ****************************************************************************************
 */
__STATIC_INLINE uint8_t co_val_get(uint8_t const array[], int lsb, int width)
{
    int msb = lsb + width - 1;
    int l_byte_idx = lsb / 8;
    int m_byte_idx = msb / 8;
    uint8_t val;

    if (m_byte_idx == l_byte_idx)
    {
        uint8_t mask = CO_BIT(width) - 1;
        int shift = lsb % 8;
        val = (array[l_byte_idx] >> shift) & mask;
    }
    else
    {
        uint8_t l_bits_cnt = m_byte_idx * 8 - lsb;
        uint8_t l_mask = CO_BIT(l_bits_cnt) - 1;
        uint8_t m_mask = CO_BIT(width - l_bits_cnt) - 1;
        int l_shift = lsb % 8;
        val = (array[l_byte_idx] >> l_shift) & l_mask;
        val |= (array[m_byte_idx] & m_mask) << l_bits_cnt;
    }
    return (val);
}

/**
 ****************************************************************************************
 * @brief This function sets a value of a bit field inside an array of bits,
 * represented as an array of bytes.
 * @param[in] array Array of bits
 * @param[in] lsb Position of the LSB of the field inside the array of bits
 * @param[in] width Width of the field
 * @param[in] val Value to be set
 ****************************************************************************************
 */
__STATIC_INLINE void co_val_set(uint8_t array[], int lsb, int width, uint8_t val)
{
    int msb = lsb + width - 1;
    int l_byte_idx = lsb / 8;
    int m_byte_idx = msb / 8;

    if (m_byte_idx == l_byte_idx)
    {
        uint8_t mask = CO_BIT(width) - 1;
        int shift = lsb % 8;
        array[l_byte_idx] &= ~(mask << shift);
        array[l_byte_idx] |= (val & mask) << shift;
    }
    else
    {
        uint8_t l_bits_cnt = m_byte_idx * 8 - lsb;
        uint8_t l_mask = CO_BIT(l_bits_cnt) - 1;
        uint8_t m_mask = CO_BIT(width - l_bits_cnt) - 1;
        int l_shift = lsb % 8;
        array[l_byte_idx] &= ~(l_mask << l_shift);
        array[m_byte_idx] &= ~m_mask;
        array[l_byte_idx] |= (val & l_mask) << l_shift;
        array[m_byte_idx] |= (val >> l_bits_cnt) & m_mask;
    }
}

/**
 ****************************************************************************************
 * @brief This function returns the status of a specific bit position inside an array of
 *        bits, represented as an array of bytes.
 * @param[in] array Array of bits to be checked
 * @param[in] pos Bit position to be checked
 * @return true if the specified bit is set, false otherwise
 ****************************************************************************************
 */
__STATIC_INLINE bool co_bit_is_set(uint8_t const array[], int pos)
{
    return ((array[pos / 8] & CO_BIT(pos % 8)) != 0);
}

/**
 ****************************************************************************************
 * @brief This function sets a specific bit position inside an array of bits, represented
 * as an array of bytes.
 * @param[in] array Array of bits
 * @param[in] pos Bit position to be set
 ****************************************************************************************
 */
__STATIC_INLINE void co_bit_set(uint8_t array[], uint8_t pos)
{
    array[pos / 8] |= CO_BIT(pos % 8);
}

/**
 ****************************************************************************************
 * @brief This function clears a specific bit position inside an array of bits,
 * represented as an array of bytes.
 * @param[in] array Array of bits
 * @param[in] pos Bit position to be cleared
 ****************************************************************************************
 */
__STATIC_INLINE void co_bit_clr(uint8_t array[], uint8_t pos)
{
    array[pos / 8] &= ~CO_BIT(pos % 8);
}


/// @} end of group COUTILS

#endif // _CO_UTILS_H_
