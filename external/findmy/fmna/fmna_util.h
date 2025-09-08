/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_util_h
#define fmna_util_h

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/// Macro for converting seconds to milliseconds.
/// @param SECONDS      Number of seconds to convert.
#define SEC_TO_MSEC(SECONDS) (SECONDS * 1000)

/// Macro for converting minutes to milliseconds.
/// @param MINUTES      Number of minutes to convert.
#define MIN_TO_MSEC(MINUTES) (MINUTES * SEC_TO_MSEC(60))

/// Macro for converting HOURS to milliseconds.
/// @param HOURS      Number of hours to convert.
#define HOURS_TO_MSEC(HOURS) (HOURS * MIN_TO_MSEC(60))

/// Macro for converting days to milliseconds.
/// @param DAYS      Number of days to convert.
#define DAYS_TO_MSEC(DAYS)   (DAYS * HOURS_TO_MSEC(24))

/**@brief Set a bit in the uint32 word.
 *
 * @param[in] W  Word whose bit is being set.
 * @param[in] B  Bit number in the word to be set.
 */
// #define SET_BIT(W, B)  ((W) |= (uint32_t)(1U << (B)))


/**@brief Clears a bit in the uint32 word.
 *
 * @param[in] W   Word whose bit is to be cleared.
 * @param[in] B   Bit number in the word to be cleared.
 */
#define CLR_BIT(W, B) ((W) &= (~(uint32_t)(1U << (B))))

/**
 * @brief Define Bit-field mask
 *
 * Macro that defined the mask with selected number of bits set, starting from
 * provided bit number.
 *
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 */
#define BF_MASK(bcnt, boff) ( ((1U << (bcnt)) - 1U) << (boff) )

/**
 * @brief Get bit-field
 *
 * Macro that extracts selected bit-field from provided value
 *
 * @param[in] val  Value from which selected bit-field would be extracted
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 *
 * @return Value of the selected bits
 */
#define BF_GET(val, bcnt, boff) ( ( (val) & BF_MASK((bcnt), (boff)) ) >> (boff) )

/**
 * @brief Create bit-field value
 *
 * Value is masked and shifted to match given bit-field
 *
 * @param[in] val  Value to set on bit-field
 * @param[in] bcnt Number of bits for bit-field
 * @param[in] boff Offset of bit-field
 *
 * @return Value positioned of given bit-field.
 */
#define BF_VAL(val, bcnt, boff) ( (((uint32_t)(val)) << (boff)) & BF_MASK(bcnt, boff) )

/// Utility function to reverse an array given start and end indices.
static inline void reverse_array(uint8_t *array, uint8_t start_idx, uint8_t end_idx)
{
    while (start_idx < end_idx)
    {
        uint8_t temp = array[start_idx];
        array[start_idx] = array[end_idx];
        array[end_idx] = temp;
        start_idx += 1;
        end_idx -= 1;
    }
}

/// Utility function to check if every element in an array is a specified value.
/// @param array Array of values to check.
/// @param val Value to compare array elements to.
/// @param len Length of the array.
static inline bool memcmp_val(void *array, unsigned char val, size_t len)
{
    unsigned char *p_array = array;

    // Compare each element in array to val.
    for (; len > 0; len--, p_array++)
    {
        if (*p_array != val)
        {
            return false;
        }
    }
    return true;
}

#endif /* fmna_util_h */
