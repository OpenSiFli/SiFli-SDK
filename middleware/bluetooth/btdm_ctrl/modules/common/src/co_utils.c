/**
****************************************************************************************
*
* @file co_utils.c
*
* @brief Common Utility functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup  CO_UTILS
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"      // SW configuration

#include <string.h>           // for mem* functions
#include "arch.h"
#include "co_bt.h"            // common bt definitions
#include "co_utils.h"         // common utility definitions
#include "co_endian.h"        // common utility for endianness
#include "co_math.h"



/*
 * DEFINES
 ****************************************************************************************
 */

///Default BT address (if none defined in NVDS)
#define BT_DEFAULT_BDADDR  {{0x01, 0x23, 0x45, 0x67, 0x89, 0xAB}}


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// Number of '1' bits in values from 0 to 15, used to fasten bit counting
const unsigned char one_bits[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

/// SCA to PPM
const uint16_t co_sca2ppm[] =
{
    [SCA_500PPM] = 500,
    [SCA_250PPM] = 250,
    [SCA_150PPM] = 150,
    [SCA_100PPM] = 100,
    [SCA_75PPM] = 75,
    [SCA_50PPM] = 50,
    [SCA_30PPM] = 30,
    [SCA_20PPM] = 20
};

/// NULL BD address
const struct bd_addr co_null_bdaddr = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

/// Default BD address
const struct bd_addr co_default_bdaddr = BT_DEFAULT_BDADDR;

/// NULL Key
const uint8_t co_null_key[KEY_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/// Table for converting rate to PHY
const uint8_t co_rate_to_phy[] =
{
    [CO_RATE_1MBPS  ]  = PHY_1MBPS_VALUE,
    [CO_RATE_2MBPS  ]  = PHY_2MBPS_VALUE,
    [CO_RATE_125KBPS]  = PHY_CODED_VALUE,
    [CO_RATE_500KBPS]  = PHY_CODED_VALUE,
    [CO_RATE_UNDEF]    = PHY_UNDEF_VALUE,
};

/// Table for converting PHY to rate (Warning: the coded PHY is converted to 125Kbps by default)
const uint8_t co_phy_to_rate[] =
{
    [PHY_1MBPS_VALUE]  = CO_RATE_1MBPS,
    [PHY_2MBPS_VALUE]  = CO_RATE_2MBPS,
    [PHY_CODED_VALUE]  = CO_RATE_125KBPS,
};

/// Convert PHY mask (with one single bit set) to a value
const uint8_t co_phy_mask_to_value[] =
{
    [PHY_1MBPS_BIT] = PHY_1MBPS_VALUE,
    [PHY_2MBPS_BIT] = PHY_2MBPS_VALUE,
    [PHY_CODED_BIT] = PHY_CODED_VALUE,
};

/// Convert PHY a value to the corresponding mask bit
const uint8_t co_phy_value_to_mask[] =
{
    [PHY_1MBPS_VALUE] = PHY_1MBPS_BIT,
    [PHY_2MBPS_VALUE] = PHY_2MBPS_BIT,
    [PHY_CODED_VALUE] = PHY_CODED_BIT,
};

/// Convert Rate value to the corresponding PHY mask bit
const uint8_t co_rate_to_phy_mask[] =
{
    [CO_RATE_1MBPS  ]  = PHY_1MBPS_BIT,
    [CO_RATE_2MBPS  ]  = PHY_2MBPS_BIT,
    [CO_RATE_125KBPS]  = PHY_CODED_BIT,
    [CO_RATE_500KBPS]  = PHY_CODED_BIT,
};

/// Convert PHY mask bit to the corresponding Rate value
const uint8_t co_phy_mask_to_rate[] =
{
    [PHY_1MBPS_BIT  ]  = CO_RATE_1MBPS,
    [PHY_2MBPS_BIT  ]  = CO_RATE_2MBPS,
    [PHY_CODED_BIT  ]  = CO_RATE_125KBPS,
};

#if BLE_PWR_CTRL

/// Convert PHY rate value of power control to the corresponding PHY mask bit
const uint8_t co_phypwr_value_to_mask[] =
{
    [PHY_PWR_1MBPS_VALUE]     = PHY_PWR_1MBPS_BIT,
    [PHY_PWR_2MBPS_VALUE]     = PHY_PWR_2MBPS_BIT,
    [PHY_PWR_S8_CODED_VALUE]  = PHY_PWR_S8_CODED_BIT,
    [PHY_PWR_S2_CODED_VALUE]  = PHY_PWR_S2_CODED_BIT,
};

/// Convert PHY mask bit of power control to the corresponding PHY rate value
const uint8_t co_phypwr_mask_to_value[] =
{
    [PHY_PWR_1MBPS_BIT] = PHY_PWR_1MBPS_VALUE,
    [PHY_PWR_2MBPS_BIT] = PHY_PWR_2MBPS_VALUE,
    [PHY_PWR_S8_CODED_BIT] = PHY_PWR_S8_CODED_VALUE,
    [PHY_PWR_S2_CODED_BIT] = PHY_PWR_S2_CODED_VALUE,
};

/// Convert PHY rate value of power control to Rate value
const uint8_t co_phypwr_to_rate[] =
{
    [PHY_PWR_1MBPS_VALUE]     = CO_RATE_1MBPS,
    [PHY_PWR_2MBPS_VALUE]     = CO_RATE_2MBPS,
    [PHY_PWR_S8_CODED_VALUE]  = CO_RATE_125KBPS,
    [PHY_PWR_S2_CODED_VALUE]  = CO_RATE_500KBPS,
};

/// Convert Rate value to PHY rate value of power control
const uint8_t co_rate_to_phypwr[] =
{
    [CO_RATE_1MBPS]     = PHY_PWR_1MBPS_VALUE,
    [CO_RATE_2MBPS]     = PHY_PWR_2MBPS_VALUE,
    [CO_RATE_125KBPS]   = PHY_PWR_S8_CODED_VALUE,
    [CO_RATE_500KBPS]   = PHY_PWR_S2_CODED_VALUE,
};

/// Convert Rate value to PHY mask value of power control
const uint8_t co_rate_to_phypwr_mask[] =
{
    [CO_RATE_1MBPS]     = PHY_PWR_1MBPS_BIT,
    [CO_RATE_2MBPS]     = PHY_PWR_2MBPS_BIT,
    [CO_RATE_125KBPS]   = PHY_PWR_S8_CODED_BIT,
    [CO_RATE_500KBPS]   = PHY_PWR_S2_CODED_BIT,
};

#endif // BLE_PWR_CTRL

const uint8_t co_rate_to_byte_dur_us[] =
{
    [CO_RATE_1MBPS  ]  = 8,
    [CO_RATE_2MBPS  ]  = 4,
    [CO_RATE_125KBPS]  = 16,
    [CO_RATE_500KBPS]  = 64,
    [CO_RATE_UNDEF]    = 0,
};


/// Base64 Encoding table
/// char code:    A  ->   Z      a  ->   z      0  ->   9      +      /      =
/// asci_val:   0x41 -> 0x5A - 0x61 -> 0x7A - 0x30 -> 0x39 - 0x2B - 0x2F - 0x3D
/// base64_val:   0  ->  25     26  ->  51     52  ->  61     62     63     N/A
__STATIC const char co_base64_encoding_table[] =
{
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
    'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
    'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'
};
/// Base64 Decoding table
/// ascii:  0x2B -> 0x7A (offset to handle)
__STATIC const int8_t co_base64_decoding_table[] =
{
    // +   ,   -   .   /   0   1   2   3   4   5   6   7   8   9   :   ;   <   =   >
    62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1,  0, -1,
    // ?   @   A   B   C   D   E   F   G   H   I   J   K   L   M   N   O   P   Q   R
    -1, -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17,
    // S   T   U   V   W   X   Y   Z   [  ' '  ]   ^   -   '   a   b   c   d   e   f
    18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1, -1, 26, 27, 28, 29, 30, 31,
    // g   h   i   j   k   l   m   n   o   p   q   r   s   t   u   v   w   x   y   z
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/// Extract length of an array from a format string
__STATIC uint16_t co_util_read_array_size(char **fmt_cursor)
{
    // Read size
    uint16_t size = 0;

    // Sanity check
    ASSERT_ERR(fmt_cursor);

    // Convert unit
    size = (*(*fmt_cursor)++) - '0';

    while (((*(*fmt_cursor)) >= '0') && ((*(*fmt_cursor)) <= '9'))
    {
        // Convert tens
        size = 10 * size + ((*(*fmt_cursor)++) - '0');
    }

    // Return the read size
    return (size);
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (RW_DEBUG || DISPLAY_SUPPORT)
void co_bytes_to_string(char *dest, uint8_t *src, uint8_t nb_bytes)
{
    for (int i = 0 ; i < nb_bytes ; i++)
    {
        char digit;
        uint8_t byte = *(src + nb_bytes - 1 - i);

        // MSbs
        digit = (byte & 0xF0) >> 4;
        digit += (digit < 10) ? 48 : 55;
        *(dest + 2 * i) = (char)digit;

        // LSbs
        digit = (byte & 0x0F) >> 0;
        digit += (digit < 10) ? 48 : 55;
        *(dest + 2 * i + 1) = (char)digit;
    }
}
#endif //(RW_DEBUG || DISPLAY_SUPPORT)

bool co_bdaddr_compare(struct bd_addr const *bd_address1, struct bd_addr const *bd_address2)
{

    for (uint8_t idx = 0; idx < BD_ADDR_LEN; idx++)
    {
        /// checks if the addresses are similar
        if (bd_address1->addr[idx] != bd_address2->addr[idx])
        {
            return (false);
        }
    }
    return (true);
}

uint32_t co_slot_to_duration(uint32_t slot_cnt)
{
    return co_max(slot_cnt >> 4, 1);
}

#if (BT_EMB_PRESENT)
uint8_t co_nb_good_channels(const struct bt_ch_map *map)
{
    uint8_t nb_good_channels = 0;

    // Count number of good channels
    for (int i = (BT_CH_MAP_LEN - 1) ; i >= 0 ; i--)
    {
        uint8_t byte = map->map[i];
        nb_good_channels += NB_ONE_BITS(byte);
    }

    return nb_good_channels;
}
#endif //BT_EMB_PRESENT

uint8_t co_util_pack(uint8_t *out, const uint8_t *in, uint16_t *out_len, uint16_t in_len, const char *format)
{
    DBG_FUNC_ENTER(co_util_pack);
    uint8_t status = CO_UTIL_PACK_OK;
    const uint8_t *p_in = in;
    uint8_t *p_out = out;
    uint8_t *p_out_end = out + *out_len;
    const uint8_t *p_in_end = in + in_len;
    char *cursor = (char *) format;
    bool b_copy = (in != NULL) && (out != NULL);
    const char *p_struct_format = NULL; // Start of structure definition
    uint8_t struct_nb_elmt = 0; // number of elements in structure
    uint8_t struct_mem_align = 0; // Unpacked structure memory alignment (1 -2- 4)

    ASSERT_ERR(format != NULL);

    bool b_lsb = true;
    bool big_number = false;
    uint16_t nb = 1;

    // Check if forced to little endian
    if (*cursor == '<')
    {
        b_lsb = true;
        cursor++;
    }
    else if (*cursor == '>')
    {
        b_lsb = false;
        cursor++;
    }

    do
    {
        char data_type;

        // Format string termination
        if (*cursor == '\0')
        {
            // nothing more to unpack - exit main loop
            if (p_struct_format == NULL) break;

            // continue with next structure element
            struct_nb_elmt -= 1;
            cursor = (char *) p_struct_format;
            // force input memory alignment
            p_in = (uint8_t *) CO_ALIGN_HI(p_in, struct_mem_align);
        }

        // check if structure array unpack is done
        if ((p_struct_format != NULL) && (struct_nb_elmt == 0))
        {
            break;
        }


        // Check if the new field is an array (starting with a number)
        if ((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = co_util_read_array_size(&cursor);
        }

        data_type = *cursor++;

        // Parse the format string
        switch (data_type)
        {
        case ('G'): // Big Number
        {
            big_number = true;
        }
        // No break
        case ('n'): // table size over 1 byte
        case ('B'): // Byte
        {
            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + nb) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // Copy bytes
                if (!big_number || (b_lsb == CPU_LE))
                {
                    memmove(p_out, p_in, nb);
                }
                // Swap bytes
                else
                {
                    co_bswap_copy(p_out, p_in, nb);
                }
            }

            // Move pointers
            p_out += nb;
            p_in += nb;

            // Re-initialize table size for next loop
            nb = 1;
            if (data_type == 'n')
            {
                // The current byte represents the size of the following table
                nb = *(p_in - 1);
            }

            big_number = false;
        }
        break;

        case ('N'): // table size over 2 byte
        case ('H'): // Short Word
        {
            // Align data buffer to a 16-bits address
            p_in = (uint8_t *)CO_ALIGN2_HI((uint32_t)p_in);

            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + 2 * nb) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    memmove(p_out, p_in, 2 * nb);

                    // Move pointers
                    p_in += 2 * nb;
                    p_out += 2 * nb;
                }
                else
                {
                    // Loop for all numbers
                    for (int i = 0 ; i < nb ; i++)
                    {
                        co_write16p(p_out, co_bswap16(*((uint16_t *)p_in)));

                        // Move pointers
                        p_in += 2;
                        p_out += 2;
                    }
                }
            }
            else
            {
                // Move pointers
                p_out += 2 * nb;
                p_in += 2 * nb;
            }

            // Re-initialize table size for next loop
            nb = 1;
            if (data_type == 'N')
            {
                // The current word represents the size of the following table
                nb = *(uint16_t *)(p_in - 2);
            }
        }
        break;

        case ('D'): // 24 bits integer
        {
            // Align data buffer to a 32-bits address
            uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_in);

            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if (((uint8_t *)(long_word + 1)) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    co_write24p(p_out, *long_word);
                }
                else
                {
                    co_write24p(p_out, co_bswap24(*long_word));
                }
            }

            // Move pointers
            p_in = (uint8_t *)(long_word + 1);
            p_out += 3;
        }
        break;

        case ('L'): // Long Word
        {
            // Align data buffer to a 32-bits address
            p_in = (uint8_t *)CO_ALIGN4_HI((uint32_t)p_in);

            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + 4 * nb) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    memmove(p_out, p_in, 4 * nb);

                    // Move pointers
                    p_in += 4 * nb;
                    p_out += 4 * nb;
                }
                else
                {
                    // Loop for all numbers
                    for (int i = 0 ; i < nb ; i++)
                    {
                        co_write32p(p_out, co_bswap32(*((uint32_t *)p_in)));

                        // Move pointers
                        p_in += 4;
                        p_out += 4;
                    }
                }
            }
            else
            {
                // Move pointers
                p_out += 4 * nb;
                p_in += 4 * nb;
            }

            // Re-initialize table size for next loop
            nb = 1;
        }
        break;

        case ('S'): // Array of structure start
        case ('s'): // Array of structure start (bit field)
        {
            if ((p_in + 1) > p_in_end)
            {
                status = CO_UTIL_PACK_IN_BUF_OVFLW;
                break;
            }

            // retrieve number of element in the structure
            struct_nb_elmt = (data_type == 'S') ? *p_in : co_bit_cnt(p_in, 1);
            if (b_copy)
            {
                *p_out = *p_in;
            }

            // move to next information
            p_in  += 1;
            p_out += 1;

            // Memory alignment
            switch (*cursor)
            {
            case ('1'):
            {
                struct_mem_align = 1;
            }
            break;
            case ('2'):
            {
                struct_mem_align = 2;
            }
            break;
            case ('4'):
            {
                struct_mem_align = 4;
            }
            break;
            default:
            {
                status = CO_UTIL_PACK_WRONG_FORMAT;
            }
            break;
            }


            if (status != CO_UTIL_PACK_OK) break;
            // force input memory alignment
            p_in = (uint8_t *) CO_ALIGN_HI(p_in, struct_mem_align);
            cursor++;
            p_struct_format = cursor; // Keep pointer to structure format
        }
        break;
        case 'T':  // BD address for UAP/NAP/LAP format
        {
            // Align data buffer to a 32-bits address
            uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_in);

            if (b_copy)
            {
                if ((p_in + 8) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }
                // Check if enough space in out buffer to write
                if ((p_out + 6) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }

                // LAP
                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    co_write24p(p_out, *long_word);
                }
                else
                {
                    co_write24p(p_out, co_bswap24(*long_word));
                }


                p_in = (uint8_t *)(long_word + 1);
                p_out += 3;

                // UAP
                *p_out++ = *p_in++;

                uint16_t *short_word = (uint16_t *)CO_ALIGN2_HI((uint32_t)p_in);
                // NAP
                if (b_lsb == CPU_LE)
                {
                    co_write16(p_out, *short_word);
                }
                else
                {
                    co_write16(p_out, co_bswap16(*short_word));
                }

                p_in = (uint8_t *)(short_word + 1);
                p_out += 2;
            }

        }
        break;
        case 'z': // For BTS2 stack, unsed bd-addr for pack in in_buffer
        {
            // Align data buffer to a 32-bits address
            p_in = (uint8_t *)CO_ALIGN4_HI((uint32_t)((uint8_t *)p_in + 8));
        }
        break;
        default:
        {
            // data format error
            status = CO_UTIL_PACK_WRONG_FORMAT;
        }
        break;
        }
    }
    while (status == CO_UTIL_PACK_OK);

    if (status == CO_UTIL_PACK_OK)
    {
        *out_len = (uint16_t)(p_out - out);
    }

    DBG_FUNC_EXIT(co_util_pack);
    return (status);
}

uint8_t co_util_unpack(uint8_t *out, const uint8_t *in, uint16_t *out_len, uint16_t in_len, const char *format)
{
    DBG_FUNC_ENTER(co_util_unpack);
    uint8_t status = CO_UTIL_PACK_OK;
    const uint8_t *p_in = in;
    uint8_t *p_out = out;
    const uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_end = out + *out_len;
    char *cursor = (char *) format;
    bool b_copy = ((out != NULL) && (in != NULL));
    const char *p_struct_format = NULL; // Start of structure definition
    uint8_t struct_nb_elmt = 0; // number of elements in structure
    uint8_t struct_mem_align = 0; // Unpacked structure memory alignment (1 -2- 4)

    bool b_lsb = true;
    bool big_number = false;
    uint16_t nb = 1;

    // Check if forced to little endian
    if (*cursor == '<')
    {
        b_lsb = true;
        cursor++;
    }
    else if (*cursor == '>')
    {
        b_lsb = false;
        cursor++;
    }

    ASSERT_ERR(format != NULL);


    do
    {
        char data_type;

        // Format string termination
        if (*cursor == '\0')
        {
            // nothing more to unpack - exit main loop
            if (p_struct_format == NULL) break;

            // continue with next structure element
            struct_nb_elmt -= 1;
            cursor = (char *) p_struct_format;
            // force output memory alignment
            p_out = (uint8_t *) CO_ALIGN_HI(p_out, struct_mem_align);
        }

        // check if structure array unpack is done
        if ((p_struct_format != NULL) && (struct_nb_elmt == 0))
        {
            break;
        }

        // Check if the new field is a fixed size array (starting with a number)
        if ((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = co_util_read_array_size(&cursor);
        }

        data_type = *cursor++;

        // Parse the format string
        switch (data_type)
        {
        case ('G'): // Big Number
        {
            big_number = true;
        }
        // No break
        case ('n'): // table size over 1 byte
        case ('B'): // Byte
        {
            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + nb) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // Check if enough space in out buffer to write
                if ((p_out + nb) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }

                // Copy bytes
                if (!big_number || (b_lsb == CPU_LE))
                {
                    memmove(p_out, p_in, nb);
                }
                // Swap bytes
                else
                {
                    co_bswap_copy(p_out, p_in, nb);
                }
            }

            // Move pointers
            p_out += nb;
            p_in += nb;

            // Re-initialize table size for next loop
            nb = 1;
            if (data_type == 'n')
            {
                // The current byte represents the size of the following table
                nb = *(p_in - 1);
            }

            big_number = false;
        }
        break;

        case ('N'): // table size over 2 byte
        case ('H'): // Short Word
        {
            // Align data buffer to a 16-bits address
            p_out = (uint8_t *)CO_ALIGN2_HI((uint32_t)p_out);

            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + 2 * nb) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // Check if enough space in output buffer to write
                if ((p_out + 2 * nb) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }

                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    memmove(p_out, p_in, 2 * nb);

                    // Move pointers
                    p_in += 2 * nb;
                    p_out += 2 * nb;
                }
                else
                {
                    // Loop for all numbers
                    for (int i = 0 ; i < nb ; i++)
                    {
                        *((uint16_t *)p_out) = co_bswap16(co_read16p(p_in));

                        // Move pointers
                        p_in += 2;
                        p_out += 2;
                    }
                }
            }
            else
            {
                // Move pointers
                p_out += 2 * nb;
                p_in += 2 * nb;
            }

            // Re-initialize table size for next loop
            nb = 1;
            if (data_type == 'N')
            {
                nb = *(uint16_t *)(p_out - 2);
            }
        }
        break;

        case ('D'): // 24 bits integer
        {
            // Align data buffer to a 32-bits address
            uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_out);

            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + 3) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // Check if enough space in out buffer to write
                if (((uint8_t *)(long_word + 1)) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }

                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    *long_word = co_read24p(p_in);
                }
                else
                {
                    *long_word = co_bswap24(co_read24p(p_in));
                }
            }

            // Move pointers
            p_out = (uint8_t *)(long_word + 1);
            p_in += 3;
        }
        break;

        case ('L'): // Long Word
        {
            // Align data buffer to a 32-bits address
            p_out = (uint8_t *)CO_ALIGN4_HI((uint32_t)p_out);

            if (b_copy)
            {
                // Check if enough space in input buffer to read
                if ((p_in + 4 * nb) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }

                // Check if enough space in output buffer to write
                if ((p_out + 4 * nb) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }

                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    memmove(p_out, p_in, 4 * nb);

                    // Move pointers
                    p_in += 4 * nb;
                    p_out += 4 * nb;
                }
                else
                {
                    // Loop for all numbers
                    for (int i = 0 ; i < nb ; i++)
                    {
                        *((uint32_t *)p_out) = co_bswap32(co_read32p(p_in));

                        // Move pointers
                        p_in += 4;
                        p_out += 4;
                    }
                }
            }
            else
            {
                // Move pointers
                p_out += 4 * nb;
                p_in += 4 * nb;
            }

            // Re-initialize table size for next loop
            nb = 1;
        }
        break;

        case ('S'): // Array of structure start
        case ('s'): // Array of structure start (bit field)
        {
            // retrieve number of element in the structure
            struct_nb_elmt = (data_type == 'S') ? *p_in : co_bit_cnt(p_in, 1);
            if (b_copy)
            {
                if ((p_in + 1) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }
                // Check if enough space in out buffer to write
                if ((p_out + 1) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }
                *p_out = *p_in;
            }

            // move to next information
            p_in  += 1;
            p_out += 1;

            // Memory alignment
            switch (*cursor)
            {
            case ('1'):
            {
                struct_mem_align = 1;
            }
            break;
            case ('2'):
            {
                struct_mem_align = 2;
            }
            break;
            case ('4'):
            {
                struct_mem_align = 4;
            }
            break;
            default:
            {
                status = CO_UTIL_PACK_WRONG_FORMAT;
            }
            break;
            }

            if (status != CO_UTIL_PACK_OK) break;

            // force output memory alignment
            p_out = (uint8_t *) CO_ALIGN_HI(p_out, struct_mem_align);
            cursor++;
            p_struct_format = cursor; // Keep pointer to structure format
        }
        break;
        case ('t'): // status
        {
            *p_out++ = *p_in++;
            // if only status, do not handle align
            if (*cursor != '\0')
            {
                uint32_t *status_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)(p_out));

                if ((uint8_t *)status_word > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }
                // next must be 4byte-align
                p_out = (uint8_t *)status_word;
            }
        }
        break;
        case ('T'): // BD address for UAP/NAP/LAP format
        {
            // Align data buffer to a 32-bits address
            uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_out);

            if (b_copy)
            {
                if ((p_in + 6) > p_in_end)
                {
                    status = CO_UTIL_PACK_IN_BUF_OVFLW;
                    break;
                }
                // Check if enough space in out buffer to write
                if ((p_out + 8) > p_out_end)
                {
                    status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                    break;
                }

                // LAP
                // If same endianess for CPU and data field
                if (b_lsb == CPU_LE)
                {
                    *long_word = co_read24p(p_in);
                }
                else
                {
                    *long_word = co_bswap24(co_read24p(p_in));
                }


                p_out = (uint8_t *)(long_word + 1);
                p_in += 3;

                // UAP
                *p_out++ = *p_in++;

                uint16_t *short_word = (uint16_t *)CO_ALIGN2_HI((uint32_t)p_out);
                // NAP
                if (b_lsb == CPU_LE)
                {
                    *short_word = co_read16p(p_in);
                }
                else
                {
                    *short_word = co_bswap16(co_read16p(p_in));
                }

                p_out = (uint8_t *)(short_word + 1);
                p_in += 2;
            }

        }
        break;
        default:
        {
            // data format error
            status = CO_UTIL_PACK_WRONG_FORMAT;
        }
        break;
        }
    }
    while (status == CO_UTIL_PACK_OK);

    // Check a potential mismatch between the theoretical (measured) input length and the given input length
    if (p_in > p_in_end)
    {
        status = CO_UTIL_PACK_IN_BUF_OVFLW;
    }

    // Return the total size needed for unpacked parameters
    *out_len = (uint16_t)(p_out - out);

    DBG_FUNC_EXIT(co_util_unpack);
    return (status);
}

int16_t co_base64_encode(uint16_t decoded_length, const uint8_t *p_decoded_data, char *p_encoded_data)
{
    uint8_t loop;
    const uint8_t *p_decoded_end = p_decoded_data + decoded_length;
    char *p_encoded_end = p_encoded_data + CO_BASE64_DECODE_TO_ENCODE_LENGTH(decoded_length);
    int16_t encoded_length = (uint16_t)(p_encoded_end - p_encoded_data);

    while (p_decoded_data < p_decoded_end)
    {
        // 24-bits of decoded data
        uint32_t data_24bit = 0;
        for (loop = 0; loop < 3 ; loop++)
        {
            data_24bit <<= 8;
            data_24bit |= (p_decoded_data < p_decoded_end) ? (uint8_t) * (p_decoded_data++) : 0;
        }

        // encode data in base64 format
        *(p_encoded_data++) = co_base64_encoding_table[(data_24bit >> 18) & 0x3F];
        *(p_encoded_data++) = co_base64_encoding_table[(data_24bit >> 12) & 0x3F];
        *(p_encoded_data++) = co_base64_encoding_table[(data_24bit >>  6) & 0x3F];
        *(p_encoded_data++) = co_base64_encoding_table[(data_24bit >>  0) & 0x3F];
    }

    // Suffix end of encoded data with '=' (1 or 2) according to decoded data length
    {
        uint8_t convert_mod3_to_nb_suffix[] = {0, 2, 1};
        for (loop = 0; loop < convert_mod3_to_nb_suffix[CO_MOD(decoded_length, 3)]; loop++)
        {
            *(--p_encoded_data) = '=';
        }
    }

    return (encoded_length);
}

int16_t co_base64_decode(uint16_t encoded_length, const char *p_encoded_data, uint8_t *p_decoded_data)
{
    int16_t decoded_length = -1;

    if (encoded_length % 4 == 0)
    {
        const char *p_encoded_end = p_encoded_data + encoded_length;
        uint8_t *p_decoded_end;

        decoded_length = CO_BASE64_ENCODE_TO_DECODE_LENGTH(encoded_length);
        if (p_encoded_data[encoded_length - 1] == '=') decoded_length--;
        if (p_encoded_data[encoded_length - 2] == '=') decoded_length--;

        p_decoded_end = p_decoded_data + decoded_length;

        while (p_encoded_data < p_encoded_end)
        {
            int32_t data_24bit = 0;
            uint8_t loop;

            // retrieve decoded 24-bit data
            for (loop = 0 ; loop < 4 ; loop++)
            {
                uint8_t decode_idx = (((uint8_t) * (p_encoded_data++)) - ((uint8_t)'+'));
                int32_t data_6bit = (decode_idx < sizeof(co_base64_decoding_table))
                                    ? co_base64_decoding_table[decode_idx] : -1;
                data_24bit = ((data_24bit << 6) | data_6bit);
            }

            // Sanity check - if valid character used
            if (data_24bit < 0)
            {
                decoded_length = -1;
                break;
            }

            // copy data in output buffer
            for (loop = 0 ; (loop < 3) && (p_decoded_data < p_decoded_end) ; loop++)
            {
                *(p_decoded_data++) = ((data_24bit >> (8 * (2 - loop))) & 0xFF);
            }
        }
    }

    return (decoded_length);
}


/// @} CO_UTILS
