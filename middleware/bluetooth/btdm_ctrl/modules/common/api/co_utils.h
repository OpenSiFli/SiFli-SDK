/**
 ****************************************************************************************
 *
 * @file co_utils.h
 *
 * @brief Common utilities definitions
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _CO_UTILS_H_
#define _CO_UTILS_H_

/**
 ****************************************************************************************
 * @defgroup CO_UTILS Utilities
 * @ingroup COMMON
 * @brief  Common utilities
 *
 * This module contains the common utilities functions and macros.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>       // standard definitions
#include <stddef.h>       // standard definitions
#include "co_bt.h"        // common bt definitions
#include "rwip_config.h"  // SW configuration
#include "rwip.h"         // SW configuration
#include "compiler.h"     // for inline functions


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// Common constants - bit field definitions
#define RW_BIT0  0x0001
#define RW_BIT1  0x0002
#define RW_BIT2  0x0004
#define RW_BIT3  0x0008
#define RW_BIT4  0x0010
#define RW_BIT5  0x0020
#define RW_BIT6  0x0040
#define RW_BIT7  0x0080
#define RW_BIT8  0x0100
#define RW_BIT9  0x0200
#define RW_BIT10 0x0400
#define RW_BIT11 0x0800
#define RW_BIT12 0x1000
#define RW_BIT13 0x2000
#define RW_BIT14 0x4000
#define RW_BIT15 0x8000

/// Number of '1' bits in a byte
#define NB_ONE_BITS(byte)   (one_bits[byte & 0x0F] + one_bits[byte >> 4])

/// Get the number of elements within an array, give also number of rows in a 2-D array
#define ARRAY_LEN(array)   (sizeof((array))/sizeof((array)[0]))

/// Get the number of columns within a 2-D array
#define ARRAY_NB_COLUMNS(array)  (sizeof((array[0]))/sizeof((array)[0][0]))


/// Macro for LMP message handler function declaration or definition
#define LMP_MSG_HANDLER(msg_name)   __STATIC int lmp_##msg_name##_handler(struct lmp_##msg_name const *param,  \
                                                                                ke_task_id_t const dest_id)
/// Macro for LMP message handler function declaration or definition
#define LLCP_MSG_HANDLER(msg_name)   __STATIC int llcp_##msg_name##_handler(struct llcp_##msg_name const *param,  \
                                                                                ke_task_id_t const dest_id)

/// Macro for HCI message handler function declaration or definition (for multi-instantiated tasks)
#define HCI_CMD_HANDLER_C(cmd_name, param_struct)   __STATIC int hci_##cmd_name##_cmd_lc_handler(param_struct const *param,  \
                                                                                ke_task_id_t const dest_id,  \
                                                                                uint16_t opcode)

/// Macro for HCI message handler function declaration or definition (with parameters)
#define HCI_CMD_HANDLER(cmd_name, param_struct)   __STATIC int hci_##cmd_name##_cmd_handler(param_struct const *param,  \
                                                                                uint16_t opcode)

/// Macro for HCI message handler function declaration or definition (with parameters)
#define HCI_CMD_HANDLER_TAB(task)   __STATIC const struct task##_hci_cmd_handler task##_hci_command_handler_tab[] =


/// MACRO to build a subversion field from the Minor and Release fields
#define CO_SUBVERSION_BUILD(minor, release)     (((minor) << 8) | (release))


/// Macro to get a structure from one of its structure field
#define CONTAINER_OF(ptr, type, member)    ((type *)( (char *)ptr - offsetof(type,member) ))

/// count number of bit into a long field
#define CO_BIT_CNT(val) (co_bit_cnt((uint8_t*) &(val), sizeof(val)))

/// Increment value and make sure it's never greater or equals max (else wrap to 0)
#define CO_VAL_INC(_val, _max)      \
    (_val) = (_val) + 1;            \
    if((_val) >= (_max)) (_val) = 0


/// Add value and make sure it's never greater or equals max (else wrap)
/// _add must be less that _max
#define CO_VAL_ADD(_val, _add, _max)      \
    (_val) = (_val) + (_add);             \
    if((_val) >= (_max)) (_val) -= (_max)

/// sub value and make sure it's never greater or equals max (else wrap)
/// _sub must be less that _max
#define CO_VAL_SUB(_val, _sub, _max)      \
    if((_val) < (_sub)) (_val) += _max;   \
    (_val) = (_val) - (_sub)

/// Convert slot in half slots
#define S_TO_HS(slots)          ((slots) << 1)
/// Convert half slot to slots
#define HS_TO_S(half_slots)     ((half_slots) >> 1)
/// Convert microseconds in half microseconds
#define US_TO_HUS(us)           ((us) << 1)
/// Convert microseconds to half slots
#define US_TO_HS(us)            ((((us) << 1) + HALF_SLOT_SIZE - 1) / HALF_SLOT_SIZE)
/// Convert half microseconds in microseconds
#define HUS_TO_US(hus)          ((hus) >> 1)
/// Convert half slots in half microseconds
#define HS_TO_HUS(half_slots)   (half_slots * HALF_SLOT_SIZE)
/// Convert frame in half slots
#define FRAME_TO_HS(frame)      (frame << 2)
/// Convert frame in microseconds
#define FRAME_TO_US(frame)      (((frame) * SLOT_SIZE) << 1)

/// Force a value to be within valid range
#define CO_VAL_FORCE_RANGE(val, range_min, range_max) \
    if((val) < (range_min)) (val) = (range_min);      \
    if((val) > (range_max)) (val) = (range_max)

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Status returned by generic packer-unpacker
enum CO_UTIL_PACK_STATUS
{
    CO_UTIL_PACK_OK,
    CO_UTIL_PACK_IN_BUF_OVFLW,
    CO_UTIL_PACK_OUT_BUF_OVFLW,
    CO_UTIL_PACK_WRONG_FORMAT,
    CO_UTIL_PACK_ERROR,
};


/// Rate information
/*@TRACE*/
enum phy_rate
{
    /// 1 Mbits/s Rate
    CO_RATE_1MBPS   = 0,
    /// 2 Mbits/s Rate
    CO_RATE_2MBPS   = 1,
    /// 125 Kbits/s Rate
    CO_RATE_125KBPS = 2,
    /// 500 Kbits/s Rate
    CO_RATE_500KBPS = 3,
    /// Undefined rate (used for reporting when no packet is received)
    CO_RATE_UNDEF   = 4,

    CO_RATE_MAX     = 4,
};


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * CONSTANT DECLARATIONS
 ****************************************************************************************
 */

/// Number of '1' bits in values from 0 to 15, used to fasten bit counting
extern const unsigned char one_bits[16];

/// Conversion table Sleep Clock Accuracy to PPM
extern const uint16_t co_sca2ppm[];

/// NULL BD address
extern const struct bd_addr co_null_bdaddr;

/// Default BD address
extern const struct bd_addr co_default_bdaddr;

/// NULL Key
extern const uint8_t co_null_key[KEY_LEN];

/// Table for converting rate to PHY
extern const uint8_t co_rate_to_phy[];

/// Table for converting PHY to rate (Warning: the coded PHY is converted to 125K by default)
extern const uint8_t co_phy_to_rate[];

/// Convert PHY mask (with one single bit set) to a value
extern const uint8_t co_phy_mask_to_value[];

/// Convert PHY a value to the corresponding mask bit
extern const uint8_t co_phy_value_to_mask[];

/// Convert Rate value to the corresponding PHY mask bit
extern const uint8_t co_rate_to_phy_mask[];

/// Convert PHY mask bit to the corresponding Rate value
extern const uint8_t co_phy_mask_to_rate[];

#if BLE_PWR_CTRL

    /// Convert PHY rate value of power control to the corresponding PHY mask bit
    extern const uint8_t co_phypwr_value_to_mask[];

    /// Convert PHY mask bit of power control to the corresponding PHY rate value
    extern const uint8_t co_phypwr_mask_to_value[];

    /// Convert PHY rate value of power control to Rate value
    extern const uint8_t co_phypwr_to_rate[];

    /// Convert Rate value to PHY rate value of power control
    extern const uint8_t co_rate_to_phypwr[];

    /// Convert Rate value to PHY mask value of power control
    extern const uint8_t co_rate_to_phypwr_mask[];

#endif // BLE_PWR_CTRL

/// Convert Rate value to byte duration in us
extern const uint8_t co_rate_to_byte_dur_us[];


/*
 * OPERATIONS ON BT CLOCK
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Clocks addition with 2 operands
 *
 * @param[in]   clock_a   1st operand value (in BT half-slots)
 * @param[in]   clock_b   2nd operand value (in BT half-slots)
 * @return      result    operation result (in BT half-slots)
 ****************************************************************************************
 */
#define CLK_ADD_2(clock_a, clock_b)     ((uint32_t)(((clock_a) + (clock_b)) & RWIP_MAX_CLOCK_TIME))

/**
 ****************************************************************************************
 * @brief Clocks addition with 3 operands
 *
 * @param[in]   clock_a   1st operand value (in BT half-slots)
 * @param[in]   clock_b   2nd operand value (in BT half-slots)
 * @param[in]   clock_c   3rd operand value (in BT half-slots)
 * @return      result    operation result (in BT half-slots)
 ****************************************************************************************
 */
#define CLK_ADD_3(clock_a, clock_b, clock_c)     ((uint32_t)(((clock_a) + (clock_b) + (clock_c)) & RWIP_MAX_CLOCK_TIME))

/**
 ****************************************************************************************
 * @brief Clocks subtraction
 *
 * @param[in]   clock_a   1st operand value (in BT half-slots)
 * @param[in]   clock_b   2nd operand value (in BT half-slots)
 * @return      result    operation result (in BT half-slots)
 ****************************************************************************************
 */
#define CLK_SUB(clock_a, clock_b)     ((uint32_t)(((clock_a) - (clock_b)) & RWIP_MAX_CLOCK_TIME))

/**
 ****************************************************************************************
 * @brief Bluetooth timestamp Clocks subtraction
 *
 * @param[in]   clock_a   1st operand value (in microseconds)
 * @param[in]   clock_b   2nd operand value (in microseconds)
 * @return      result    operation result (in microseconds)
 ****************************************************************************************
 */
#define CLK_BTS_SUB(clock_a, clock_b)     (((int32_t) ((clock_a) - (clock_b))))

/**
 ****************************************************************************************
 * @brief Check if clock_a is lower than or equal to clock_b
 *
 * @param[in]   clock_a   Clock A value (in BT half-slots)
 * @param[in]   clock_b   Clock B value (in BT half-slots)
 * @return      result    True: clock_a lower than or equal to clock_b | False: else
 ****************************************************************************************
 */
#define CLK_BTS_LOWER_EQ(clock_a, clock_b) (((uint32_t)CLK_BTS_SUB(clock_b, clock_a)) < (RWIP_MAX_BTS_TIME >> 1))

/**
 ****************************************************************************************
 * @brief Clocks time difference
 *
 * @param[in]   clock_a   1st operand value (in BT half-slots)
 * @param[in]   clock_b   2nd operand value (in BT half-slots)
 * @return      result    return the time difference from clock A to clock B
 *                           - result < 0  => clock_b is in the past
 *                           - result == 0 => clock_a is equal to clock_b
 *                           - result > 0  => clock_b is in the future
 ****************************************************************************************
 */
#define CLK_DIFF(clock_a, clock_b)     ( (CLK_SUB((clock_b), (clock_a)) > ((RWIP_MAX_CLOCK_TIME+1) >> 1)) ?                      \
                          ((int32_t)((-CLK_SUB((clock_a), (clock_b))))) : ((int32_t)((CLK_SUB((clock_b), (clock_a))))) )



/// macro to extract a field from a value containing several fields
/// @param[in] __r bit field value
/// @param[in] __f field name
/// @return the value of the register masked and shifted
#define GETF(__r, __f)                                                           \
    (( (__r) & (__f##_MASK) ) >> (__f##_LSB))

/// macro to set a field value into a value  containing several fields.
/// @param[in] __r bit field value
/// @param[in] __f field name
/// @param[in] __v value to put in field
#define SETF(__r, __f, __v)                                                      \
    do {                                                                         \
        ASSERT_INFO( ( ( ( (__v) << (__f##_LSB) ) & ( ~(__f##_MASK) ) ) ) == 0 ,(__f##_MASK), (__v)); \
        __r = (((__r) & ~(__f##_MASK)) | (__v) << (__f##_LSB));                  \
    } while (0)



/// macro to extract a bit field from a value containing several fields
/// @param[in] __r bit field value
/// @param[in] __b bit field name
/// @return the value of the register masked and shifted
#define GETB(__r, __b)                                                           \
    (( (__r) & (__b##_BIT) ) >> (__b##_POS))

/// macro to set a bit field value into a value containing several fields.
/// @param[in] __r bit field value
/// @param[in] __b bit field name
/// @param[in] __v value to put in field
#define SETB(__r, __b, __v)                                                      \
    do {                                                                         \
        ASSERT_ERR( ( ( ( (__v ? 1 : 0) << (__b##_POS) ) & ( ~(__b##_BIT) ) ) ) == 0 ); \
        __r = (((__r) & ~(__b##_BIT)) | (__v ? 1 : 0) << (__b##_POS));                  \
    } while (0)

/// macro to toggle a bit into a value containing several bits.
/// @param[in] __r bit field value
/// @param[in] __b bit field name
#define TOGB(__r, __b)                                                           \
    do {                                                                         \
        __r = ((__r) ^ (__b##_BIT));                                             \
    } while (0)

/**
 ****************************************************************************************
 * @brief Create a mask composed of a given number of trailing 1s
 * Example CO_MASK(5) = b11111;
 *
 * @param[in]   nb_bits     Number of bits to 1 in the mask
 * @return      result
 ****************************************************************************************
 */
#define CO_MASK(nb_bits)              ((1 << (nb_bits)) - 1)

/**
 ****************************************************************************************
 * @brief Check if clock_a is equal to clock_b
 *
 * @param[in]   clock_a   Clock A value (in BT half-slots)
 * @param[in]   clock_b   Clock B value (in BT half-slots)
 * @return      result    True: clock_a lower than or equal to clock_b | False: else
 ****************************************************************************************
 */
#define CLK_EQ(clock_a, clock_b)      (clock_b == clock_a)

/**
 ****************************************************************************************
 * @brief Check if clock_a is lower than or equal to clock_b
 *
 * @param[in]   clock_a   Clock A value (in BT half-slots)
 * @param[in]   clock_b   Clock B value (in BT half-slots)
 * @return      result    True: clock_a lower than or equal to clock_b | False: else
 ****************************************************************************************
 */
#define CLK_LOWER_EQ(clock_a, clock_b)      (CLK_SUB(clock_b, clock_a) < (RWIP_MAX_CLOCK_TIME >> 1))

/**
 ****************************************************************************************
 * @brief Check if clock A is lower than or equal to clock B (with half-us precision)
 *
 * @param[in]   int_a     Integer part of clock A (in BT half-slots)
 * @param[in]   fract_a   Fractional part of clock A (in half-us) (range: 0 to 624)
 * @param[in]   int_b     Integer part of clock B (in BT half-slots)
 * @param[in]   fract_b   Fractional part of clock B (in half-us) (range: 0 to 624)
 * @return      result    True: clock A lower than or equal to clock B | False: else
 ****************************************************************************************
 */
#define CLK_LOWER_EQ_HUS(int_a, fract_a, int_b, fract_b)      (  CLK_GREATER_THAN(int_b, int_a)  \
                                                                 || (   CLK_EQ(int_a, int_b)     \
                                                                     && (fract_a <= fract_b) ) ) \

/**
 ****************************************************************************************
 * @brief Check if clock_a is greater than clock_b
 *
 * @param[in]   clock_a   Clock A value (in BT half-slots)
 * @param[in]   clock_b   Clock B value (in BT half-slots)
 * @return      result    True: clock_a is greater than clock_b | False: else
 ****************************************************************************************
 */
#define CLK_GREATER_THAN(clock_a, clock_b)    !(CLK_LOWER_EQ(clock_a, clock_b))

/**
 ****************************************************************************************
 * @brief Check if clock A is greater than clock B (with half-us precision)
 *
 * @param[in]   int_a     Integer part of clock A (in BT half-slots)
 * @param[in]   fract_a   Fractional part of clock A (in half-us) (range: 0 to 624)
 * @param[in]   int_b     Integer part of clock B (in BT half-slots)
 * @param[in]   fract_b   Fractional part of clock B (in half-us) (range: 0 to 624)
 * @return      result    True: clock A greater than clock B | False: else
 ****************************************************************************************
 */
#define CLK_GREATER_THAN_HUS(int_a, fract_a, int_b, fract_b)      (  CLK_GREATER_THAN(int_a, int_b)  \
                                                                     || (   CLK_EQ(int_a, int_b)     \
                                                                         && (fract_a > fract_b) ) )  \


/// Convert decode length to encode length
#define CO_BASE64_DECODE_TO_ENCODE_LENGTH(length) (4 * (((length) + 2) / 3))
/// Convert encode length to decode length
#define CO_BASE64_ENCODE_TO_DECODE_LENGTH(length) (((length) / 4) * 3)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Read an aligned 32 bit word.
 * @param[in] ptr32 The address of the first byte of the 32 bit word.
 * @return The 32 bit value.
 ****************************************************************************************
 */
__INLINE uint32_t co_read32(void const *ptr32)
{
    return *((uint32_t *)ptr32);
}

/**
 ****************************************************************************************
 * @brief Read an aligned 16 bits word.
 * @param[in] ptr16 The address of the first byte of the 16 bits word.
 * @return The 16 bits value.
 ****************************************************************************************
 */
__INLINE uint16_t co_read16(void const *ptr16)
{
    return *((uint16_t *)ptr16);
}

/**
 ****************************************************************************************
 * @brief Write an aligned 32 bits word.
 * @param[in] ptr32 The address of the first byte of the 32 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__INLINE void co_write32(void const *ptr32, uint32_t value)
{
    *(uint32_t *)ptr32 = value;
}

/**
 ****************************************************************************************
 * @brief Write an aligned 16 bits word.
 * @param[in] ptr16 The address of the first byte of the 16 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__INLINE void co_write16(void const *ptr16, uint32_t value)
{
    *(uint16_t *)ptr16 = value;
}

/**
 ****************************************************************************************
 * @brief Write a 8 bits word.
 * @param[in] ptr8 The address of the first byte of the 8 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__INLINE void co_write8(void const *ptr8, uint32_t value)
{
    *(uint8_t *)ptr8 = value;
}

/**
 ****************************************************************************************
 * @brief Read a packed 16 bits word.
 * @param[in] ptr16 The address of the first byte of the 16 bits word.
 * @return The 16 bits value.
 ****************************************************************************************
 */
__INLINE uint16_t co_read16p(void const *ptr16)
{
    uint16_t value = ((uint8_t const volatile *)ptr16)[0] | ((uint8_t const volatile *)ptr16)[1] << 8;
    return value;
}

/**
 ****************************************************************************************
 * @brief Read a packed 24 bits word.
 * @param[in] ptr24 The address of the first byte of the 24 bits word.
 * @return The 24 bits value.
 ****************************************************************************************
 */
__INLINE uint32_t co_read24p(void const *ptr24)
{
    uint16_t addr_l, addr_h;
    addr_l = co_read16p(ptr24);
    addr_h = *((uint8_t const volatile *)ptr24 + 2) & 0x00FF;
    return ((uint32_t)addr_l | (uint32_t)addr_h << 16);
}

/**
 ****************************************************************************************
 * @brief Write a packed 24 bits word.
 * @param[in] ptr24 The address of the first byte of the 24 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__INLINE void co_write24p(void const *ptr24, uint32_t value)
{
    uint8_t volatile *ptr = (uint8_t *)ptr24;

    *ptr++ = (uint8_t)(value & 0xff);
    *ptr++ = (uint8_t)((value & 0xff00) >> 8);
    *ptr++ = (uint8_t)((value & 0xff0000) >> 16);
}

/**
 ****************************************************************************************
 * @brief Read a packed 32 bits word.
 * @param[in] ptr32 The address of the first byte of the 32 bits word.
 * @return The 32 bits value.
 ****************************************************************************************
 */
__INLINE uint32_t co_read32p(void const *ptr32)
{
    uint16_t addr_l, addr_h;
    addr_l = co_read16p(ptr32);
    addr_h = co_read16p((uint8_t *)ptr32 + 2);
    return ((uint32_t)addr_l | (uint32_t)addr_h << 16);
}
/**
 ****************************************************************************************
 * @brief Write a packed 32 bits word.
 * @param[in] ptr32 The address of the first byte of the 32 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__INLINE void co_write32p(void const *ptr32, uint32_t value)
{
    uint8_t volatile *ptr = (uint8_t *)ptr32;

    *ptr++ = (uint8_t)(value & 0xff);
    *ptr++ = (uint8_t)((value & 0xff00) >> 8);
    *ptr++ = (uint8_t)((value & 0xff0000) >> 16);
    *ptr = (uint8_t)((value & 0xff000000) >> 24);
}

/**
 ****************************************************************************************
 * @brief Write a packed 16 bits word.
 * @param[in] ptr16 The address of the first byte of the 16 bits word.
 * @param[in] value The value to write.
 ****************************************************************************************
 */
__INLINE void co_write16p(void const *ptr16, uint16_t value)
{
    uint8_t volatile *ptr = (uint8_t *)ptr16;

    *ptr++ = value & 0xff;
    *ptr = (value & 0xff00) >> 8;
}

/**
 ****************************************************************************************
 * Count number of bit set to 1 in a value with variable length
 *
 * @param[in] p_val Pointer to value
 * @param[in] size  Number of Bytes
 * @return Number of bit counted
 ****************************************************************************************
 */
__INLINE uint8_t co_bit_cnt(const uint8_t *p_val, uint8_t size)
{
    uint8_t nb_bit = 0;
    while (size-- > 0)
    {
        nb_bit += NB_ONE_BITS(*p_val);
        p_val++;
    }
    return (nb_bit);
}

#if (RW_DEBUG || DISPLAY_SUPPORT)

    /**
    ****************************************************************************************
    * @brief Convert bytes to hexadecimal string
    *
    * @param[out] dest      Pointer to the destination string (must be 2x longer than input table)
    * @param[in]  src       Pointer to the bytes table
    * @param[in]  nb_bytes  Number of bytes to display in the string
    ****************************************************************************************
    */
    void co_bytes_to_string(char *dest, uint8_t *src, uint8_t nb_bytes);
#endif //(RW_DEBUG || DISPLAY_SUPPORT)

/**
 ****************************************************************************************
 * @brief Compares two Bluetooth device addresses
 *
 * This function checks if the two bd address are equal.
 *
 * @param[in] bd_address1        Pointer on the first bd address to be compared.
 * @param[in] bd_address2        Pointer on the second bd address to be compared.
 *
 * @return result of the comparison (true: equal | false: different).
 ****************************************************************************************
 */
bool co_bdaddr_compare(struct bd_addr const *bd_address1, struct bd_addr const *bd_address2);

/**
******************************************************************************
* @brief Convert an duration in baseband slot to a duration in number of ticks.
* @param[in]  slot_cnt  Duration in number of baseband slot
* @return  Duration (in number of ticks).
******************************************************************************
*/
uint32_t co_slot_to_duration(uint32_t slot_cnt);

#if (BT_EMB_PRESENT)

    /**
    ******************************************************************************
    * @brief Count the number of good channels in a map
    * @param[in]  map  Channel Map (bit fields for the 79 BT RF channels)
    * @return  Number of good channels
    ******************************************************************************
    */
    uint8_t co_nb_good_channels(const struct bt_ch_map *map);

#endif //BT_EMB_PRESENT

/**
 ****************************************************************************************
 * @brief Pack parameters from a C structure to a packed buffer
 *
 * This function packs parameters according to a specific format. It takes care of the
 * endianess, padding, required by the compiler.
 *
 * By default output format is LSB but it can be changed with first character of format string
 *     - <  : LSB output format
 *     - >  : MSB output format
 *
 * Format strings are the mechanism used to specify the expected layout when packing and unpacking data. They are built
 * up from Format Characters, which specify the type of data being packed/unpacked.
 *     - B:             byte - 8bits value
 *     - H:             word - 16bits value
 *     - L:             long - 32-bits value
 *     - D:             24 bits value
 *     - XXB:           table of several bytes, where XX is the byte number, in decimal
 *     - XXG:           Number of several bytes, where XX is the byte number, in decimal
 *                      subject to be swapped according to endianess
 *     - nB :           table size over 1 byte, followed by the table of bytes
 *     - NB :           table size over 2 bytes, followed by the table of bytes
 *     - S[1|2|4][...]: Array of structures with S elements, S is an 8-bit value read in the bytes stream.
 *                      [1|2|4] is following structure memory alignment; 1: 8-bit, 2: 16-bit, 4: 32-bit
 *                      [...] is element structure information
 *                      Example: S2BHB structure alignment is 16-bit since it contains a 16-bit variable
 *                               If the byte stream is 0x02:0xAA:0xBB:0xBB:0xCC:0xDD:0xEE:0xEE:0xFF,
 *                               the corresponding array is [{0xAA, 0xBBBB, 0xCC}, {0xDD, 0xEEEE, 0xFF}].
 *
 *     - s[1|2|4][...]: Array of structures with n elements, s is an 8-bit value read in the bytes stream, with n bits set to 1.
 *                      [1|2|4] is following structure memory alignment; 1: 8-bit, 2: 16-bit, 4: 32-bit
 *                      [...] is element structure information
 *                      Example: s2BHB structure alignment is 16-bit since it contains a 16-bit variable
 *                               If the byte stream is 0x88:0xAA:0xBB:0xBB:0xCC:0xDD:0xEE:0xEE:0xFF,
 *                               the corresponding array is [{0xAA, 0xBBBB, 0xCC}, {0xDD, 0xEEEE, 0xFF}].
 *
 * Example:   "BBLH12BLnB" => 1 byte | 1 byte | 1 long | 1 short | 12-bytes table | 1 long | table size over 1 byte | n-bytes table
 *
 * Note: the function works in the same buffer
 *
 * @param[out]     out         Output Data Buffer
 * @param[in]      in          Input Data Buffer
 * @param[out]     out_len     Output size of packed data (in bytes)
 * @param[in]      in_len      Input buffer size (in bytes)
 * @param[in]      format      Parameters format
 *
 * @return  Status of the packing operation
 *****************************************************************************************
 */
uint8_t co_util_pack(uint8_t *out, const uint8_t *in, uint16_t *out_len, uint16_t in_len, const char *format);

/**
 ****************************************************************************************
 * @brief Unpack parameters from an unpacked buffer to a C structure
 *
 * This function unpacks parameters according to a specific format. It takes care of the
 * endianess, padding, required by the compiler.
 *
 * By default input format is LSB but it can be changed with first character of format string
 *     - <  : LSB input format
 *     - >  : MSB input format
 *
 * Format strings are the mechanism used to specify the expected layout when packing and unpacking data. They are built
 * up from Format Characters, which specify the type of data being packed/unpacked.
 *     - B:             byte - 8bits value
 *     - H:             word - 16bits value
 *     - L:             long - 32-bits value
 *     - D:             24 bits value
 *     - XXB:           table of several bytes, where XX is the byte number, in decimal
 *     - XXG:           Number of several bytes, where XX is the byte number, in decimal
 *                      subject to be swapped according to endianess
 *     - nB :           table size over 1 byte, followed by the table of bytes
 *     - NB :           table size over 2 bytes, followed by the table of bytes
 *     - S[1|2|4][...]: Array of structures with S elements, S is an 8-bit value read in the bytes stream.
 *                      [1|2|4] is following structure memory alignment; 1: 8-bit, 2: 16-bit, 4: 32-bit
 *                      [...] is element structure information
 *                      Example: S2BHB structure alignment is 16-bit since it contains a 16-bit variable
 *                               If the byte stream is 0x02:0xAA:0xBB:0xBB:0xCC:0xDD:0xEE:0xEE:0xFF,
 *                               the corresponding array is [{0xAA, 0xBBBB, 0xCC}, {0xDD, 0xEEEE, 0xFF}].
 *
 *     - s[1|2|4][...]: Array of structures with n elements, s is an 8-bit value read in the bytes stream, with n bits set to 1.
 *                      [1|2|4] is following structure memory alignment; 1: 8-bit, 2: 16-bit, 4: 32-bit
 *                      [...] is element structure information
 *                      Example: s2BHB structure alignment is 16-bit since it contains a 16-bit variable
 *                               If the byte stream is 0x88:0xAA:0xBB:0xBB:0xCC:0xDD:0xEE:0xEE:0xFF,
 *                               the corresponding array is [{0xAA, 0xBBBB, 0xCC}, {0xDD, 0xEEEE, 0xFF}].
 *
 * Example:   "BBLH12BLnB" => 1 byte | 1 byte | 1 long | 1 short | 12-bytes table | 1 long | table size over 1 byte | n-bytes table
 *
 * Note: the output buffer provided must be large enough to contain the unpacked data.
 * Note2: if a NULL output buffer is provided, the function does not copy the unpacked parameters. It still parses the
 *  format string and input buffer to return the number of unpacked bytes. Can be used to compute the expected unpacked
 *  buffer size.
 *
 * @param[out]    out         Unpacked parameters buffer
 * @param[in]     in          Packed parameters buffer
 * @param[inout]  out_len     Input: buffer size / Output: size of unpacked data (in bytes)
 * @param[in]     in_len      Size of the packed data (in bytes)
 * @param[in]     format      Parameters format
 *
 * @return  Status of the unpacking operation
 *****************************************************************************************
 */
uint8_t co_util_unpack(uint8_t *out, const uint8_t *in, uint16_t *out_len, uint16_t in_len, const char *format);

/**
 ****************************************************************************************
 * @brief Encode an array of byte into a Base64 string (UTF-8 format without null termination character)
 *
 * @param[in]  decoded_length  Length of decoded data
 * @param[in]  p_decoded_data  Pointer to decoded data array
 * @param[out] p_encoded_data  Pointer to array where encoded data is pushed
 *
 * @return -1 if an error occurs, otherwise length of encoded data
 *****************************************************************************************
 */
int16_t co_base64_encode(uint16_t decoded_length, const uint8_t *p_decoded_data, char *p_encoded_data);

/**
 ****************************************************************************************
 * @brief Decode a Base64 string (UTF-8 Format) to an array of byte
 *
 * @param[in]  encoded_length  Length of encoded data (shall be a multiple of 4 characters)
 * @param[in]  p_encoded_data  Pointer to Base64 encoded string
 * @param[out] p_decoded_data  Pointer to array where decoded data is pushed
 *
 * @return -1 if an error occurs, otherwise length of decoded data
 *****************************************************************************************
 */
int16_t co_base64_decode(uint16_t encoded_length, const char *p_encoded_data, uint8_t *p_decoded_data);

/**
 ****************************************************************************************
 * @brief Compute size of a UTF-8 string.
 *
 * Equivalent to strlen, it doesn't load extra memory data.
 * Can be used to prevent some "Memory Protection" issue. (see PLF_MEM_PROTECTION)
 *
 * @param[in]  p_str Pointer to input string
 *
 * @return Length of the  UTF-8 string.
 *****************************************************************************************
 */
__INLINE uint16_t co_strlen(const char *p_str)
{
    const char *p_str_end = p_str;
    while (*p_str_end != '\0') p_str_end++;
    return ((uint16_t)(p_str_end - p_str));
}

/// @} CO_UTILS

#endif // _CO_UTILS_H_
