/**
 ****************************************************************************************
 *
 * @file ecc_p256.c
 *
 * @brief ECC function definitions for P256
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "rwip.h"

#include "ecc_p256.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "arch.h"

#include "ke_mem.h"
#include "co_bt_defines.h"
#include "co_math.h"
#include "co_utils.h"
#include "co_list.h"
#include "ke_event.h"

#include "dbg_swdiag.h"          // Software diag

#if 0
    #include "led.h"
#else
    #define led_set(a)
    #define led_reset(a)
#endif

// ECC Accelerator in a simulation environement
#ifdef CFG_ECC_SIM_ACCEL
    #include "reg_ecc_accel.h"
#endif // CFG_ECC_SIM_ACCEL

/*
 * DEFINES
 ****************************************************************************************
 */

#define __INLINE__ __INLINE


/*********************************************************************************
 *  The length of P256 numbers stored.
 *  P256 numbers are stored in uint_32 arrays, with one more element than required.
 *  The extra element is used to store overflow's and thus simplify addition, subtraction
 *
 *********************************************************************************/
#if (ECC_MULT_ALGO_TYPE == 16)
    #define MAX_OCTETS256  34
    #define MAX_DIGITS_256  68 // ( (9+9) * 4 )
    #define ELEMENTS_BIG_HEX256  (MAX_OCTETS256/2)  // 17
    #define HIGHEST_INDEX_BIG_HEX256  ELEMENTS_BIG_HEX256 - 1 // 16
#elif (ECC_MULT_ALGO_TYPE == 32)
    #define MAX_OCTETS256  36
    #define MAX_DIGITS_256  72 // ( (9+9) * 4 )
    #define ELEMENTS_BIG_HEX256  (MAX_OCTETS256/4)  // 9
    #define HIGHEST_INDEX_BIG_HEX256  (MAX_OCTETS256/4 )- 1 // 8
#else
    #error "ECC Multiplication Algorithm configuration Not Supported"
#endif // (ECC_MULT_ALGO_TYPE == 16)

#define ECC_4BIT_TABLE_GENERATION_PT1 0x03
#define ECC_4BIT_TABLE_GENERATION_PT2 0x04
#define ECC_DHKEY_GENERATION_USING_TABLE 0x05

#define ECC_PUBLICKEY_GENERATION 0x01
#define ECC_DHKEY_GENERATION     0x02

/*
 * MACROS
 ****************************************************************************************
 */

/**********************************************************************
 * Typedef and macro for 64 bit number handling
 **********************************************************************/

#define _LL(x) x##ll

/************************************************************************************
 * Simple Macros to determine
 *                 a/ if a 'big hex number' is even.
 *                 b/ get the most significant uint32 element
 ************************************************************************************/
#define ISBIGHEXEVEN256(x) (!((x.num[HIGHEST_INDEX_BIG_HEX256]) & 0x01))

#if (ECC_MULT_ALGO_TYPE == 16)
    #define GETMOSTSIGNIGICANTU16_256(tmpHexA)  tmpHexA->num[ELEMENTS_BIG_HEX256-tmpHexA->len]
#elif (ECC_MULT_ALGO_TYPE == 32)
    #define GETMOSTSIGNIGICANTU32_256(tmpHexA)  tmpHexA->num[ELEMENTS_BIG_HEX256-tmpHexA->len]
#endif // (ECC_MULT_ALGO_TYPE == 16)


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef unsigned long long int u64;

/*
 * Signed types
 */

typedef signed char s_int8;
typedef signed short s_int16;
typedef signed int s_int32;

#if (ECC_MULT_ALGO_TYPE == 16)

typedef struct bigHex256
{
    uint16_t num[MAX_OCTETS256 / 2];
    uint32_t  len;
    uint32_t  sign;
} bigHex256;

typedef struct veryBigHex256
{
    uint16_t num[MAX_OCTETS256 / 1];
    uint32_t  len;
    uint32_t  sign;
} veryBigHex256;


#elif (ECC_MULT_ALGO_TYPE == 32)
typedef struct bigHex256
{
    uint32_t num[MAX_OCTETS256 / 4];
    uint32_t  len;
    uint32_t  sign;
} bigHex256;

typedef struct veryBigHex256
{
    uint32_t num[MAX_OCTETS256 / 2];
    uint32_t  len;
    uint32_t  sign;
} veryBigHex256;

#endif // (ECC_MULT_ALGO_TYPE == 16)

typedef struct ECC_Point256
{
    bigHex256 x;
    bigHex256 y;
} ECC_Point256;

typedef struct ECC_Jacobian_Point256
{
    bigHex256 x;
    bigHex256 y;
    bigHex256 z;
} ECC_Jacobian_Point256;



/// ECC multiplication element
struct ecc_elt_tag
{
    /// List element for chaining in the ECC environment
    struct co_list_hdr     hdr;

    uint32_t                Point_Mul_Word256;

    // ECC points Q and R in Jacobian format.
    ECC_Jacobian_Point256  Jacobian_PointQ256;
    ECC_Jacobian_Point256  Jacobian_PointR256;

    // The Private/Secret Key used during calculations
    bigHex256              Pk256;

    /// Callback function to execute once algorithm completes
    ecc_p256_result_cb          cb_result;
    /// Metadata information to return with result in order to retrieve execution context
    uint32_t               metainfo;


#if (ECC_MULT_ALGO_TYPE == 16)
    uint16_t               current_val;
    uint16_t               bit_cursor;
#elif (ECC_MULT_ALGO_TYPE == 32)
    uint32_t               current_val;
    uint32_t               bit_cursor;
#endif // (ECC_MULT_ALGO_TYPE == 16)

    uint8_t                key_type;
    ECC_Jacobian_Point256 *win_4_table;
#ifdef CFG_ECC_SIM_ACCEL
    // result of ECC computation
    ecc_p256_result_t           result;
#endif // CFG_ECC_SIM_ACCEL
};


/// ECC environment structure
struct ecc_env_tag
{
    /// List of ongoing multiplications
    struct co_list ongoing_mul;

#ifdef CFG_ECC_SIM_ACCEL
    /// use accelerator
    bool           use_simu_accelerator;
#endif // CFG_ECC_SIM_ACCEL
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*****************************************************************************
 *  CONSTs REQUIRED FOR ECC P256
 ****************************************************************************/

// Base Points for the ECC P256 Curve. - LSB first
__STATIC const uint8_t BasePoint_x_256[32]      = {0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4, 0xA0, 0x33, 0xEB, 0x2D, 0x81, 0x7D, 0x03, 0x77, 0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8, 0x47, 0x42, 0x2C, 0xE1, 0xF2, 0xD1, 0x17, 0x6B};
__STATIC const uint8_t BasePoint_y_256[32]      = {0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB, 0xCE, 0x5E, 0x31, 0x6B, 0x57, 0x33, 0xCE, 0x2B, 0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E, 0x9B, 0x7F, 0x1A, 0xFE, 0xE2, 0x42, 0xE3, 0x4F};
// Maximum Secret Key Value - LSB first
__STATIC const uint8_t maxSecretKey_256[32]     = {0xA8, 0x92, 0x31, 0x7E, 0x61, 0xe5, 0xdc, 0x79, 0x42, 0xcf, 0x8b, 0xd3, 0x56, 0x7d, 0x73, 0xde, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x7f};
// Debug Public Keys - used when SSP debug mode is enabled - LSB first
__STATIC const uint8_t DebugE256PublicKey_x[32] = {0xE6, 0x9D, 0x35, 0x0E, 0x48, 0x01, 0x03, 0xCC, 0xDB, 0xFD, 0xF4, 0xAC, 0x11, 0x91, 0xF4, 0xEF, 0xB9, 0xA5, 0xF9, 0xE9, 0xA7, 0x83, 0x2C, 0x5E, 0x2C, 0xBE, 0x97, 0xF2, 0xD2, 0x03, 0xB0, 0x20};
__STATIC const uint8_t DebugE256PublicKey_y[32] = {0x8B, 0xD2, 0x89, 0x15, 0xD0, 0x8E, 0x1C, 0x74, 0x24, 0x30, 0xED, 0x8F, 0xC2, 0x45, 0x63, 0x76, 0x5C, 0x15, 0x52, 0x5A, 0xBF, 0x9A, 0x32, 0x63, 0x6D, 0xEB, 0x2A, 0x65, 0x49, 0x9C, 0x80, 0xDC};
// Debug Private Key - used when SSP debug mode is enabled - LSB first
__STATIC const uint8_t DebugE256SecretKey[32] =   {0xBD, 0x1A, 0x3C, 0xCD, 0xA6, 0xB8, 0x99, 0x58, 0x99, 0xB7, 0x40, 0xEB, 0x7B, 0x60, 0xFF, 0x4A, 0x50, 0x3F, 0x10, 0xD2, 0xE3, 0xB3, 0xC9, 0x74, 0x38, 0x5F, 0xC5, 0xA3, 0xD4, 0xF6, 0x49, 0x3F};


#if (ECC_MULT_ALGO_TYPE == 16)
const bigHex256 coef_B =     {{ 0x0000, 0x5ac6, 0x35d8, 0xaa3a, 0x93e7, 0xb3eb, 0xbd55, 0x7698, 0x86bc, 0x651d, 0x06b0, 0xcc53, 0xb0f6, 0x3bce, 0x3c3e, 0x27d2, 0x604b},
    16, 0
};
#else
const bigHex256 coef_B =     {{ 0x0000, 0x5ac635d8, 0xaa3a93e7, 0xb3ebbd55, 0x769886bc, 0x651d06b0, 0xcc53b0f6, 0x3bce3c3e, 0x27d2604b},
    8, 0
};
#endif



#if (ECC_MULT_ALGO_TYPE == 16)
// Value P which is used in mod operations to reflect points back into the Galois Field of the curve.
const bigHex256 bigHexP256 = {{ 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
    16, 0
};


const veryBigHex256 veryBigHexP256 =
{
    {
        0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
    },
    16, 0
};

// Point of infinity in the Jacobian system
const  ECC_Jacobian_Point256 ecc_Jacobian_InfinityPoint256 = { {{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, 0x00, 0x00},
    {{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001 }, 0x01, 0x00},
    {{0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }, 0x00, 0x00}
};


#elif (ECC_MULT_ALGO_TYPE == 32)
// Value P which is used in mod operations to reflect points back into the Galois Field of the curve.
const bigHex256 bigHexP256 = {{ 0x00000000, 0xFFFFFFFF, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
    8, 0
};
const veryBigHex256 veryBigHexP256 =
{
    {
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0xFFFFFFFF, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
    },
    8, 0
};

// Point of infinity in the Jacobian system
const  ECC_Jacobian_Point256 ecc_Jacobian_InfinityPoint256 = { {{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }, 0x00, 0x00},
    {{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000001 }, 0x01, 0x00},
    {{0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }, 0x00, 0x00}
};
#endif // (ECC_MULT_ALGO_TYPE == 16)


#if (ECC_MULT_ALGO_TYPE == 16)
const  ECC_Jacobian_Point256 ECC_4Win_Look_up_table[15] =
{
//0x0001
    {
        /* num */
        /* x   */ {{ 0x0000, 0x6b17, 0xd1f2, 0xe12c, 0x4247, 0xf8bc, 0xe6e5, 0x63a4, 0x40f2, 0x7703, 0x7d81, 0x2deb, 0x33a0, 0xf4a1, 0x3945, 0xd898, 0xc296 }, 16, 0x00},
        /* y   */ {{ 0x0000, 0x4fe3, 0x42e2, 0xfe1a, 0x7f9b, 0x8ee7, 0xeb4a, 0x7c0f, 0x9e16, 0x2bce, 0x3357, 0x6b31, 0x5ece, 0xcbb6, 0x4068, 0x37bf, 0x51f5 }, 16, 0x00},
        /* Z   */ {{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001 },  1, 0x00}
    },
//0x0002
    {
        /* num */
        /* x   */ {{ 0x0000, 0x8f22, 0x8205, 0x98a6, 0x0fe3, 0xc582, 0xa4e4, 0xe495, 0xf299, 0x7b48, 0xa1de, 0x9650, 0x59a9, 0x18c3, 0xc50b, 0xb251, 0xa29d }, 16, 0x00},
        /* y   */ {{ 0x0000, 0x468a, 0xb3e5, 0x1fcb, 0x4722, 0x9871, 0xe7e9, 0x5bf2, 0x8286, 0xcd44, 0x8ed3, 0xeb0c, 0x78ec, 0xd34e, 0xac12, 0x1d28, 0x5bc2 }, 16, 0x00},
        /* z   */ {{ 0x0000, 0xc297, 0x492c, 0x20cc, 0xe496, 0x6ae2, 0xcd2b, 0x58b7, 0x5e48, 0xffbd, 0x5489, 0x95ef, 0x960d, 0x41e2, 0x74dc, 0x1e5b, 0x3ed6 }, 16, 0x00}
    },
//0x0003
    {
        /* num */
        /* x   */ {{ 0x0000, 0x0bad, 0x26fb, 0xd93b, 0x1ab7, 0xe844, 0x8e98, 0xe098, 0xc19f, 0x90e3, 0x156a, 0x94ae, 0x76e3, 0x56e8, 0xbcaa, 0x5d21, 0x0197 }, 16, 0x00},
        /* y   */ {{ 0x0000, 0x4a1d, 0x6bbe, 0xdf46, 0xa06f, 0x7d76, 0x1828, 0x7a69, 0xfd83, 0x346c, 0x8ed1, 0xef77, 0x4189, 0xb0e3, 0x6138, 0x55bd, 0x8d83 }, 16, 0x00},
        /* Z   */ {{ 0x0000, 0x3093, 0x9029, 0xdb21, 0x4543, 0x2131, 0x3957, 0xaec9, 0xc854, 0xa4b2, 0x3170, 0xb0ce, 0xe4c4, 0x6f2e, 0xc46c, 0xdf0d, 0x9342 }, 16, 0x00}
    },
// 0x0004
    {
        /* num */
        /* x   */ {{ 0x0000, 0xe1aa, 0x8f71, 0x41bf, 0xe1ed, 0x0ab4, 0x8559, 0x9b57, 0x79eb, 0xec8b, 0x9353, 0x3c5b, 0xde62, 0x423d, 0xf00d, 0x2b1c, 0xb5f2 }, 16, 0x00},
        /* y   */ {{ 0x0000, 0xd2f1, 0x22ef, 0x8038, 0x5047, 0x8290, 0xab2a, 0x6277, 0x71ec, 0x4ecc, 0xef63, 0x34a3, 0xd343, 0xc0a9, 0xf7e6, 0x5610, 0x3895 }, 16, 0x00},
        /* z   */ {{ 0x0000, 0x9182, 0xb500, 0x3bc1, 0xda72, 0xe3be, 0x14e0, 0xfb0c, 0x2593, 0x3305, 0x22bd, 0x8332, 0x6ee2, 0xe245, 0x7d8e, 0x6de1, 0x783e }, 16, 0x00},
    },
//0x0005
    {
        /* num */
        /* x   */ {{ 0x0000, 0x6a14, 0x0c55, 0x70ae, 0x60a6, 0x1eab, 0x131d, 0xf0a7, 0xb995, 0x5ca5, 0x2eee, 0x3a03, 0xb007, 0xa6fc, 0x97e6, 0x0ddc, 0xd000}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x3266, 0x5be0, 0xdea1, 0x11a9, 0xf1de, 0x8948, 0xb015, 0x14c3, 0x6e69, 0x3bab, 0xf68c, 0x4db9, 0xed63, 0x07a3, 0xe9ec, 0x3df5}, 16, 0x00},
        /* z   */ {{ 0x0000, 0xc53b, 0xb6c4, 0x0c02, 0x8674, 0xca4d, 0xbb66, 0x2166, 0x24f8, 0x9394, 0x09db, 0x2e75, 0xd7fa, 0x62d4, 0xb5e6, 0xa887, 0x0ac3}, 16, 0x00}
    },
//0x0006
    {
        /* num */
        /* x   */ {{ 0x0000, 0xcca1, 0x5363, 0x5063, 0x27c1, 0xcb6a, 0x22b4, 0x5277, 0xe102, 0x2794, 0x82b5, 0xedc0, 0xaff9, 0xa43c, 0x31e5, 0xee4c, 0x7f21}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x5f70, 0x6301, 0x91f1, 0xe4f4, 0xa011, 0xb402, 0x2042, 0x3775, 0xf79c, 0x950c, 0x421d, 0xf6a0, 0x03e8, 0x67c6, 0x4d00, 0xc6a6}, 16, 0x00},
        /* z   */ {{ 0x0000, 0xd1c5, 0x92c6, 0xf4fc, 0x29af, 0x17c2, 0xbd51, 0xf6c8, 0xa5ea, 0x9916, 0x9413, 0xcc8f, 0xbce7, 0x8d14, 0x6d21, 0x3159, 0x2204}, 16, 0x00}
    },
//0x0007
    {
        /* num */
        /* x   */ {{ 0x0000, 0xf8af, 0x31df, 0x7a18, 0x9edd, 0xb7a3, 0xdec5, 0x8223, 0x22ab, 0x6881, 0x1a4a, 0xb165, 0xdd62, 0x16e9, 0x3bed, 0xa632, 0x7b8f}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x4f09, 0xa05f, 0xac18, 0x0f41, 0x62aa, 0x8320, 0xe134, 0xa671, 0x366e, 0x374f, 0xff2a, 0x6698, 0x68ee, 0x233e, 0x77bf, 0x98d2}, 16, 0x00},
        /* z   */ {{ 0x0000, 0xc13e, 0xd81b, 0xdf7b, 0xdf9f, 0x50f3, 0xb993, 0x8aed, 0x5397, 0x8b06, 0xec6c, 0x01b7, 0xc151, 0x11c4, 0x2b81, 0x3f40, 0xf476}, 16, 0x00}
    },
//0x0008
    {
        /* num */
        /* x   */ {{ 0x0000, 0x3d01, 0xd0ab, 0xc595, 0x3e53, 0xe5d1, 0x38e3, 0xd692, 0xd41a, 0x616f, 0x5757, 0x559b, 0xc2d9, 0x2888, 0xc2bc, 0xc4e2, 0xafba}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x193f, 0x2d8a, 0x1f2e, 0xb204, 0x8247, 0xf138, 0x028c, 0xf01a, 0x45ce, 0x2623, 0x6dd5, 0xa56f, 0x7074, 0x3576, 0x8cdf, 0xbfbf}, 16, 0x00},
        /* z   */ {{ 0x0000, 0x4a03, 0xbfe8, 0x5bdb, 0x2daa, 0x2679, 0xf907, 0x08a9, 0x2d0d, 0xd6ca, 0x8939, 0x7a68, 0xfb2d, 0xf14a, 0x32d8, 0x08ed, 0x7f2f}, 16, 0x00}
    },
//0x0009
    {
        /* num */
        /* x   */ {{ 0x0000, 0x339d, 0x3514, 0x3ab7, 0x894b, 0x6d62, 0x618d, 0x19a5, 0xf8d6, 0xbc4c, 0xb6e5, 0x2a11, 0xea88, 0xc6bb, 0xa46d, 0x0ca3, 0x52d1}, 16, 0x00},
        /* y   */ {{ 0x0000, 0xdc93, 0xa482, 0xde79, 0xb694, 0x5961, 0xa59f, 0xde26, 0x70e0, 0x4efa, 0xcd9f, 0xa462, 0xea51, 0x4b7d, 0xc4a3, 0xc108, 0xc7bb}, 16, 0x00},
        /* z   */ {{ 0x0000, 0xce26, 0x909e, 0x040d, 0x04a8, 0x1f89, 0x0e57, 0xc1da, 0x3605, 0x6fcc, 0x9c5b, 0x6e87, 0x0e7d, 0xdbef, 0xc849, 0xc416, 0x7fe7}, 16, 0x00}
    },
//0x000A
    {
        /* num */
        /* x   */ {{ 0x0000, 0xfd00, 0xadb9, 0xb973, 0x01c9, 0xa9cc, 0x2bcc, 0x6007, 0xda4f, 0x30ec, 0x4dbf, 0x3125, 0x9407, 0xba73, 0x2617, 0x928f, 0x48bb}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x880d, 0xf759, 0x2b7b, 0x4d98, 0xb051, 0xaed9, 0x3df1, 0xdddc, 0x56c9, 0x3d8f, 0x1714, 0x8250, 0x867c, 0x1c89, 0xc3bf, 0xd08b}, 16, 0x00},
        /* z   */ {{ 0x0000, 0xfcae, 0x15e2, 0xdc42, 0xb476, 0xc8c6, 0x8143, 0xcfcc, 0x9446, 0xa73f, 0xea01, 0x4537, 0x0be0, 0x9e1d, 0x98cb, 0xe781, 0xfd4b}, 16, 0x00}
    },
//0x000B
    {
        /* num */
        /* x   */ {{ 0x0000, 0xd201, 0x5a57, 0xf4f6, 0x398f, 0xb1e1, 0xd191, 0x4a78, 0x41e7, 0x463c, 0x9536, 0xc59c, 0xe39d, 0x41f5, 0x5168, 0xe5be, 0x91e1}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x06b9, 0xac4a, 0x0b8a, 0x2038, 0xae86, 0x45b7, 0x4673, 0x2630, 0x83f5, 0x8c9f, 0xdc27, 0x15a4, 0x11cd, 0x96d4, 0x6ae3, 0xedb0}, 16, 0x00},
        /* z   */ {{ 0x0000, 0x2a8b, 0x67f7, 0xe052, 0x6c09, 0x0cea, 0x0b00, 0x2ebf, 0x997d, 0x85d2, 0xd9d6, 0x7600, 0x7cca, 0x2645, 0x36cc, 0x109c, 0xee08}, 16, 0x00}
    },
//0x000C
    {
        /* num */
        /* x   */ {{ 0x0000, 0xffcb, 0x0a9a, 0x7dfe, 0x6606, 0xeecf, 0x47ad, 0x4c3c, 0x4a4e, 0x39a6, 0x21a3, 0xc310, 0xb661, 0x115a, 0x0335, 0x6f55, 0xf312}, 16, 0x00},
        /* y   */ {{ 0x0000, 0xe572, 0xe4df, 0x8328, 0x6cb4, 0x0dde, 0xa98c, 0x8958, 0xc6b3, 0xaa07, 0x51fa, 0xb237, 0x93d8, 0x7f73, 0x946e, 0xc5cc, 0xf9f8}, 16, 0x00},
        /* z   */ {{ 0x0000, 0xef20, 0xaf4e, 0x7295, 0xb11e, 0x8f6d, 0x3d02, 0x240a, 0x3dac, 0x0349, 0xcce0, 0xf30b, 0x0638, 0x136b, 0x84c0, 0x966e, 0x7415}, 16, 0x00},
    },
//0x000D
    {
        /* num */
        /* x   */ {{ 0x0000, 0xc94d, 0x5b90, 0xb318, 0xd9ea, 0x9dd2, 0x7189, 0x1ae7, 0x1799, 0xbf5c, 0xe7a4, 0xd629, 0x696e, 0xa97d, 0x5e61, 0x7021, 0xcf29}, 16, 0x00},
        /* y   */ {{ 0x0000, 0x56f3, 0xc6fe, 0xcafa, 0x2a29, 0x0159, 0x021d, 0x82bd, 0x1799, 0xff69, 0x74d1, 0x5e49, 0x646b, 0x5dea, 0x5f0b, 0x32e8, 0x8933}, 16, 0x00},
        /* z   */ {{ 0x0000, 0x1c13, 0x5301, 0x5f85, 0x8c20, 0x4d43, 0x52f5, 0xc186, 0xb6ca, 0x582f, 0x44e2, 0xc250, 0x446a, 0x4973, 0x3d20, 0xaadc, 0xe543}, 16, 0x00},
    },
//0x000E
    {
        /* num */
        /* x   */ {{ 0x0000, 0xbe6e, 0x27b3, 0x2cdf, 0x8dba, 0xfc10, 0x56e5, 0x9f0a, 0x28b2, 0x1b69, 0xf4ba, 0x15cd, 0x8668, 0x702a, 0xa678, 0xa297, 0x7270 }, 16, 0x00},
        /* y   */ {{ 0x0000, 0x7f5b, 0x3426, 0x7b6a, 0x8b10, 0xfbf9, 0x7ffd, 0xc6fd, 0xe4b3, 0x4e95, 0x9e1d, 0xa067, 0xb6e1, 0x869c, 0x4fbe, 0x272a, 0xb43a }, 16, 0x00},
        /* z   */ {{ 0x0000, 0xe389, 0x2e89, 0xb6d2, 0xb79d, 0xb0e0, 0x674d, 0x1287, 0x752e, 0x66d4, 0x2cc6, 0xabe0, 0xdb83, 0x5de7, 0x4ac4, 0x9c6d, 0x041d }, 16, 0x00}
    },
//0x000F
    {
        /* num */
        /* x   */ {{ 0x0000, 0x2436, 0x6156, 0xcb65, 0x9023, 0xf21b, 0xe5dd, 0xa2b9, 0xfe84, 0xb402, 0x833f, 0x46a7, 0xb876, 0xb948, 0x8cde, 0x4727, 0xa595 }, 16, 0x00},
        /* y   */ {{ 0x0000, 0xf0fa, 0x6956, 0xd8c7, 0xaf3e, 0xe73b, 0xc2d4, 0xec91, 0x1c8f, 0x3744, 0x2d40, 0x7daa, 0xd19c, 0xa479, 0x10e1, 0xa5c5, 0xe688 }, 16, 0x00},
        /* z   */ {{ 0x0000, 0x5884, 0xf036, 0xbd33, 0xa657, 0x6f1f, 0x822e, 0x6fb9, 0xcaa5, 0xfdd2, 0xa20e, 0xc14b, 0xde4d, 0xe2d3, 0xef31, 0xaf45, 0x54fb }, 16, 0x00}
    }

};

#elif (ECC_MULT_ALGO_TYPE == 32)
const  ECC_Jacobian_Point256 ECC_4Win_Look_up_table[15] =
{
//0x0001
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x6b17d1f2, 0xe12c4247, 0xf8bce6e5, 0x63a440f2, 0x77037d81, 0x2deb33a0, 0xf4a13945, 0xd898c296 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x4fe342e2, 0xfe1a7f9b, 0x8ee7eb4a, 0x7c0f9e16, 0x2bce3357, 0x6b315ece, 0xcbb64068, 0x37bf51f5 }, 0x8, 0x00},
        /* Z   */ {{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000001 }, 0x1, 0x00}
    },
//0x0002
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x8f228205, 0x98a60fe3, 0xc582a4e4, 0xe495f299, 0x7b48a1de, 0x965059a9, 0x18c3c50b, 0xb251a29d  }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x468ab3e5, 0x1fcb4722, 0x9871e7e9, 0x5bf28286, 0xcd448ed3, 0xeb0c78ec, 0xd34eac12, 0x1d285bc2  }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xc297492c, 0x20cce496, 0x6ae2cd2b, 0x58b75e48, 0xffbd5489, 0x95ef960d, 0x41e274dc, 0x1e5b3ed6  }, 0x8, 0x00}
    },
//0x0003
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x0bad26fb, 0xd93b1ab7, 0xe8448e98, 0xe098c19f, 0x90e3156a, 0x94ae76e3, 0x56e8bcaa, 0x5d210197}, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x4a1d6bbe, 0xdf46a06f, 0x7d761828, 0x7a69fd83, 0x346c8ed1, 0xef774189, 0xb0e36138, 0x55bd8d83}, 0x8, 0x00},
        /* Z   */ {{ 0x00000000, 0x30939029, 0xdb214543, 0x21313957, 0xaec9c854, 0xa4b23170, 0xb0cee4c4, 0x6f2ec46c, 0xdf0d9342}, 0x8, 0x00}
    },
// 0x0004
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xe1aa8f71, 0x41bfe1ed, 0x0ab48559, 0x9b5779eb, 0xec8b9353, 0x3c5bde62, 0x423df00d, 0x2b1cb5f2}, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0xd2f122ef, 0x80385047, 0x8290ab2a, 0x627771ec, 0x4eccef63, 0x34a3d343, 0xc0a9f7e6, 0x56103895}, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0x9182b500, 0x3bc1da72, 0xe3be14e0, 0xfb0c2593, 0x330522bd, 0x83326ee2, 0xe2457d8e, 0x6de1783e}, 0x8, 0x00},
    },
//0x0005
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x6a140c55, 0x70ae60a6, 0x1eab131d, 0xf0a7b995, 0x5ca52eee, 0x3a03b007, 0xa6fc97e6, 0x0ddcd000}, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x32665be0, 0xdea111a9, 0xf1de8948, 0xb01514c3, 0x6e693bab, 0xf68c4db9, 0xed6307a3, 0xe9ec3df5}, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xc53bb6c4, 0x0c028674, 0xca4dbb66, 0x216624f8, 0x939409db, 0x2e75d7fa, 0x62d4b5e6, 0xa8870ac3}, 0x8, 0x00}
    },
//0x0006
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xcca15363, 0x506327c1, 0xcb6a22b4, 0x5277e102, 0x279482b5, 0xedc0aff9, 0xa43c31e5, 0xee4c7f21 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x5f706301, 0x91f1e4f4, 0xa011b402, 0x20423775, 0xf79c950c, 0x421df6a0, 0x03e867c6, 0x4d00c6a6 }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xd1c592c6, 0xf4fc29af, 0x17c2bd51, 0xf6c8a5ea, 0x99169413, 0xcc8fbce7, 0x8d146d21, 0x31592204 }, 0x8, 0x00}
    },
//0x0007
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xf8af31df, 0x7a189edd, 0xb7a3dec5, 0x822322ab, 0x68811a4a, 0xb165dd62, 0x16e93bed, 0xa6327b8f }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x4f09a05f, 0xac180f41, 0x62aa8320, 0xe134a671, 0x366e374f, 0xff2a6698, 0x68ee233e, 0x77bf98d2 }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xc13ed81b, 0xdf7bdf9f, 0x50f3b993, 0x8aed5397, 0x8b06ec6c, 0x01b7c151, 0x11c42b81, 0x3f40f476 }, 0x8, 0x00}
    },
//0x0008
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x3d01d0ab, 0xc5953e53, 0xe5d138e3, 0xd692d41a, 0x616f5757, 0x559bc2d9, 0x2888c2bc, 0xc4e2afba }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x193f2d8a, 0x1f2eb204, 0x8247f138, 0x028cf01a, 0x45ce2623, 0x6dd5a56f, 0x70743576, 0x8cdfbfbf }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0x4a03bfe8, 0x5bdb2daa, 0x2679f907, 0x08a92d0d, 0xd6ca8939, 0x7a68fb2d, 0xf14a32d8, 0x08ed7f2f }, 0x8, 0x00}
    },
//0x0009
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x339d3514, 0x3ab7894b, 0x6d62618d, 0x19a5f8d6, 0xbc4cb6e5, 0x2a11ea88, 0xc6bba46d, 0x0ca352d1 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0xdc93a482, 0xde79b694, 0x5961a59f, 0xde2670e0, 0x4efacd9f, 0xa462ea51, 0x4b7dc4a3, 0xc108c7bb }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xce26909e, 0x040d04a8, 0x1f890e57, 0xc1da3605, 0x6fcc9c5b, 0x6e870e7d, 0xdbefc849, 0xc4167fe7 }, 0x8, 0x00}
    },
//0x000A
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xfd00adb9, 0xb97301c9, 0xa9cc2bcc, 0x6007da4f, 0x30ec4dbf, 0x31259407, 0xba732617, 0x928f48bb }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x880df759, 0x2b7b4d98, 0xb051aed9, 0x3df1dddc, 0x56c93d8f, 0x17148250, 0x867c1c89, 0xc3bfd08b }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xfcae15e2, 0xdc42b476, 0xc8c68143, 0xcfcc9446, 0xa73fea01, 0x45370be0, 0x9e1d98cb, 0xe781fd4b }, 0x8, 0x00}
    },
//0x000B
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xd2015a57, 0xf4f6398f, 0xb1e1d191, 0x4a7841e7, 0x463c9536, 0xc59ce39d, 0x41f55168, 0xe5be91e1 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x06b9ac4a, 0x0b8a2038, 0xae8645b7, 0x46732630, 0x83f58c9f, 0xdc2715a4, 0x11cd96d4, 0x6ae3edb0 }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0x2a8b67f7, 0xe0526c09, 0x0cea0b00, 0x2ebf997d, 0x85d2d9d6, 0x76007cca, 0x264536cc, 0x109cee08 }, 0x8, 0x00}
    },
//0x000C
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xffcb0a9a, 0x7dfe6606, 0xeecf47ad, 0x4c3c4a4e, 0x39a621a3, 0xc310b661, 0x115a0335, 0x6f55f312 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0xe572e4df, 0x83286cb4, 0x0ddea98c, 0x8958c6b3, 0xaa0751fa, 0xb23793d8, 0x7f73946e, 0xc5ccf9f8 }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xef20af4e, 0x7295b11e, 0x8f6d3d02, 0x240a3dac, 0x0349cce0, 0xf30b0638, 0x136b84c0, 0x966e7415 }, 0x8, 0x00},
    },
//0x000D
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xc94d5b90, 0xb318d9ea, 0x9dd27189, 0x1ae71799, 0xbf5ce7a4, 0xd629696e, 0xa97d5e61, 0x7021cf29 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x56f3c6fe, 0xcafa2a29, 0x0159021d, 0x82bd1799, 0xff6974d1, 0x5e49646b, 0x5dea5f0b, 0x32e88933 }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0x1c135301, 0x5f858c20, 0x4d4352f5, 0xc186b6ca, 0x582f44e2, 0xc250446a, 0x49733d20, 0xaadce543 }, 0x8, 0x00},
    },
//0x000E
    {
        /* num */
        /* x   */ {{ 0x00000000, 0xbe6e27b3, 0x2cdf8dba, 0xfc1056e5, 0x9f0a28b2, 0x1b69f4ba, 0x15cd8668, 0x702aa678, 0xa2977270 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0x7f5b3426, 0x7b6a8b10, 0xfbf97ffd, 0xc6fde4b3, 0x4e959e1d, 0xa067b6e1, 0x869c4fbe, 0x272ab43a }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0xe3892e89, 0xb6d2b79d, 0xb0e0674d, 0x1287752e, 0x66d42cc6, 0xabe0db83, 0x5de74ac4, 0x9c6d041d }, 0x8, 0x00}
    },
//0x000F
    {
        /* num */
        /* x   */ {{ 0x00000000, 0x24366156, 0xcb659023, 0xf21be5dd, 0xa2b9fe84, 0xb402833f, 0x46a7b876, 0xb9488cde, 0x4727a595 }, 0x8, 0x00},
        /* y   */ {{ 0x00000000, 0xf0fa6956, 0xd8c7af3e, 0xe73bc2d4, 0xec911c8f, 0x37442d40, 0x7daad19c, 0xa47910e1, 0xa5c5e688 }, 0x8, 0x00},
        /* z   */ {{ 0x00000000, 0x5884f036, 0xbd33a657, 0x6f1f822e, 0x6fb9caa5, 0xfdd2a20e, 0xc14bde4d, 0xe2d3ef31, 0xaf4554fb }, 0x8, 0x00}
    }

};
#endif

// local environment variable
struct ecc_env_tag ecc_env;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/****************************************************************************
 *      Prototypes for local functions
 *      These perform the bulk of the maths for ECC
 ****************************************************************************/
void AddBigHex256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult);
void SubtractBigHex256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult);
void bigHexInversion256(bigHex256 *bigHexA, bigHex256 *pResult);
void Add2SelfBigHex256(bigHex256 *bigHexA, const bigHex256 *bigHexB);
void AddP256(bigHex256 *bigHexA);
void SubtractBigHexMod256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult);
void SubtractFromSelfBigHex256(bigHex256 *bigHexA, const bigHex256 *bigHexB);
void SubtractFromSelfBigHexSign256(bigHex256 *bigHexA, const bigHex256 *bigHexB);
void MultiplyBigHexModP256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult);
#if (ECC_MULT_ALGO_TYPE == 16)
    void MultiplyByU16ModP256(const uint16_t N, bigHex256 *result);
#elif (ECC_MULT_ALGO_TYPE == 32)
    void MultiplyByU32ModP256(const uint32_t N, bigHex256 *result);
#endif // (ECC_MULT_ALGO_TYPE == 16)
void specialModP256(bigHex256 *tmpHexA);
void AddPdiv2_256(bigHex256 *bigHexA);

__STATIC int isValidSecretKey_256(uint8_t *secretKey);

/****************************************************************************
 *      Prototypes for local inline functions
 *      There are small functions which are frequently called
 ****************************************************************************/
__INLINE__ void initBigNumber256(bigHex256 *BigHexResult);
__INLINE__ void setBigNumberLength256(bigHex256 *BigHexResult);
__INLINE__ void setVeryBigNumberLength256(veryBigHex256 *BigHexResult);
__INLINE__ void copyBigHex256(const bigHex256 *source, bigHex256 *destination);
__INLINE__ void shiftLeftOneArrayElement256(bigHex256 *input);
__INLINE__ void divideByTwo256(bigHex256 *A);

__INLINE__ int isGreaterThan256(const bigHex256 *bigHexA, const bigHex256 *bigHexB);
__INLINE__ int isGreaterThanOrEqual256(const bigHex256 *bigHexA, const bigHex256 *bigHexB);
__INLINE__ int isVeryBigHexGreaterThanOrEqual256(const veryBigHex256 *bigHexA, const veryBigHex256 *bigHexB);

__INLINE__ void GF_Setup_Jacobian_Infinity_Point256(ECC_Jacobian_Point256 *infinity);
__INLINE__ void GF_Affine_To_Jacobian_Point_Copy256(const ECC_Point256 *source, ECC_Jacobian_Point256 *destination);
__INLINE__ void GF_Jacobian_Point_Copy256(const ECC_Jacobian_Point256 *source, ECC_Jacobian_Point256 *destination);





// Galois Field Operands
int GF_Jacobian_Point_Double256(const ECC_Jacobian_Point256 *pPointP, ECC_Jacobian_Point256 *pResultPoint);

int isValidSecretKey_256(uint8_t *secretKey)
{
    // First check that the secret key is not all zeros.
    //
    int i;

    for (i = 31; i >= 0; i--)
    {
        if (secretKey[i] != 0)
            break;
    }

    if ((i == -1) && (secretKey[0] == 0))
        return 0;

    for (i = 31; i >= 0; i--)
    {
        if (secretKey[i] > maxSecretKey_256[i])
            return 0;
        else if (secretKey[i] < maxSecretKey_256[i])
            return 1;
    }
    return 1;
}

/******************************************************************************
 * Section of functions for BASIC big number handling
 *   These do not consider the Finite Field or ECC
 *
 *  Mod P is not performed
 *  Sign is not considered -- all inputs and results  are assumed to be positive
 *
 *  AddBigHex            - This adds two 15*u16 positive numbers.
 *  AddVeryBigHex        - This adds two 30*u16 positive numbers.
 *
 *  SubtractBigHex       - This subtracts two 15*u16 positive numbers
 *                         To ensure a +ive result we assume A >= B
 *  SubtractVeryBigHex   - This subtracts two 30*u16 positive numbers
 *                         To ensure a +ive result we assume A >= B
 *  MultiplyBigHex       - This multiplies two 15*u16 positive numbers to produce
 *                         a 30*u16 positive result
 *
 ******************************************************************************/

//
//
// Inputs - Format of the inputs that they begin with the LEAST significant bytes in the 0 entry of the array.
//
void AddBigHex256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult)
{
    uint32_t carry = 0;
    s_int32 i;
#if (ECC_MULT_ALGO_TYPE == 16)
    const uint16_t *numA = bigHexA->num;
    const uint16_t *numB = bigHexB->num;
    uint16_t *result = BigHexResult->num;
#elif (ECC_MULT_ALGO_TYPE == 32)
    const uint32_t *numA = bigHexA->num;
    const uint32_t *numB = bigHexB->num;
    uint32_t *result = BigHexResult->num;
#endif // (ECC_MULT_ALGO_TYPE == 16)

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        u64 tmp;

#if (ECC_MULT_ALGO_TYPE == 16)
        tmp = (uint32_t)((uint32_t)(numA[i]) + (uint32_t)(numB[i]) + carry);
        if (tmp & 0x10000)
            carry = 0x01;
        else
            carry = 0x00;

        result[i] = (tmp & 0xFFFF);
#elif (ECC_MULT_ALGO_TYPE == 32)
        tmp = (u64)((u64)(numA[i]) + (u64)(numB[i]) + carry);
        if (tmp & _LL(0x100000000))
            carry = 0x01;
        else
            carry = 0x00;

        result[i] = (tmp & 0xFFFFFFFF);
#endif // (ECC_MULT_ALGO_TYPE == 16)
    }
    setBigNumberLength256(BigHexResult);
    BigHexResult->sign = 0;
}

void AddBigHexModP256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult)
{
    AddBigHex256(bigHexA, bigHexB, BigHexResult);

    if (BigHexResult->sign == 0x00)   // Positive Number
    {
        if (isGreaterThanOrEqual256(BigHexResult, &bigHexP256))
        {
            //SubtractFromSelfBigHexSign(BigHexResult,&bigHexP);
            SubtractFromSelfBigHex256(BigHexResult, &bigHexP256);
        }
    }
    else
    {
        AddP256(BigHexResult);
    }
}
// A = A + B
void Add2SelfBigHex256(bigHex256 *bigHexA, const bigHex256 *bigHexB)
{
#if (ECC_MULT_ALGO_TYPE == 16)
    uint32_t tmp;
    uint16_t carry = 0;
    s_int32 i;
    uint16_t *numA = bigHexA->num;
    const uint16_t *numB = bigHexB->num;

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        tmp = (uint32_t)((uint32_t)(numA[i]) + (uint32_t)(numB[i]) + carry);
        if (tmp & 0x10000)
            carry = 0x01;
        else
            carry = 0x00;

        numA[i] = (tmp & 0xFFFF);
    }
#elif (ECC_MULT_ALGO_TYPE == 32)
    uint32_t carry = 0;
    s_int32 i;
    uint32_t *numA = bigHexA->num;
    const uint32_t *numB = bigHexB->num;

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        u64 tmp = (u64)((u64)(numA[i]) + (u64)(numB[i]) + carry);
        if (tmp & _LL(0x100000000))
            carry = 0x01;
        else
            carry = 0x00;

        numA[i] = (tmp & 0xFFFFFFFF);
    }
#endif // (ECC_MULT_ALGO_TYPE == 16)
    setBigNumberLength256(bigHexA);
}

void SubtractBigHex256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult)
{
#if (ECC_MULT_ALGO_TYPE == 16)
    // Due to the nature of the GF we have to assume both inputs are +ive.
    uint16_t borrow = 0;
    s_int32 i;
    const uint16_t *pBigNum = bigHexA->num;
    const uint16_t *pSmallNum = bigHexB->num;
    uint16_t *pResult = BigHexResult->num;

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        if (((uint32_t)((uint32_t)(pSmallNum[i]) + (uint32_t)borrow)) > (pBigNum[i]))
        {
            pResult[i] = (((uint32_t)(pBigNum[i] + 0x10000)) - (pSmallNum[i] + borrow)) & 0xFFFF;
            borrow = 0x01;
        }
        else
        {
            pResult[i] = ((pBigNum[i]) - (pSmallNum[i] + borrow)) & 0xFFFF;
            borrow = 0x00;
        }
    }
#elif (ECC_MULT_ALGO_TYPE == 32)
    // Due to the nature of the GF we have to assume both inputs are +ive.
    uint32_t borrow = 0;
    s_int32 i;
    const uint32_t *pBigNum = bigHexA->num;
    const uint32_t *pSmallNum = bigHexB->num;
    uint32_t *pResult = BigHexResult->num;

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        if (((u64)((u64)(pSmallNum[i]) + (u64)borrow)) > (pBigNum[i]))
        {
            pResult[i] = (((u64)(pBigNum[i] + _LL(0x100000000))) - (pSmallNum[i] + borrow)) & 0xFFFFFFFF;
            borrow = 0x01;
        }
        else
        {
            pResult[i] = ((pBigNum[i]) - (pSmallNum[i] + borrow)) & 0xFFFFFFFF;
            borrow = 0x00;
        }
    }
#endif // (ECC_MULT_ALGO_TYPE == 16)
    setBigNumberLength256(BigHexResult);
}

// A = A - B
void SubtractFromSelfBigHex256(bigHex256 *bigHexA, const bigHex256 *bigHexB)
{
#if (ECC_MULT_ALGO_TYPE == 16)
    uint16_t borrow = 0;
    s_int32 i;
    uint16_t *pBigNum = bigHexA->num;
    const uint16_t *pSmallNum = bigHexB->num;

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        if (((uint32_t)((uint32_t)(pSmallNum[i]) + (uint32_t)borrow)) > (pBigNum[i]))
        {
            pBigNum[i]  = (((uint32_t)(pBigNum[i] + 0x10000)) - (pSmallNum[i] + borrow)) & 0xFFFF;
            borrow = 0x01;
        }
        else
        {
            pBigNum[i] = ((pBigNum[i]) - (pSmallNum[i] + borrow)) & 0xFFFF;
            borrow = 0x00;
        }
    }
#elif (ECC_MULT_ALGO_TYPE == 32)
    // Due to the nature of the GF we have to assume both inputs are +ive.
    uint32_t borrow = 0;
    s_int32 i;
    uint32_t *pBigNum = bigHexA->num;
    const uint32_t *pSmallNum = bigHexB->num;

    for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
    {
        if (((u64)((u64)(pSmallNum[i]) + (u64)borrow)) > (pBigNum[i]))
        {
            pBigNum[i]  = (((u64)(pBigNum[i] + _LL(0x100000000))) - (pSmallNum[i] + borrow)) & 0xFFFFFFFF;
            borrow = 0x01;
        }
        else
        {
            pBigNum[i] = ((pBigNum[i]) - (pSmallNum[i] + borrow)) & 0xFFFFFFFF;
            borrow = 0x00;
        }
    }
#endif // (ECC_MULT_ALGO_TYPE == 16)
    setBigNumberLength256(bigHexA);
}

/**********************************************************************************
 * This section extends of the basic functions provided previously to support
 * sign. We concentrate on the Add and Subtract functions for both 15*u16 and 30*u16
 * numbers.
 *
 * AddBigHexSign         - Adds two 15*u16 numbers considering the sign.
 * SubtractBigHexSign    - Subtracts two 15*u16 numbers considering the sign
 * AddVeryBigHexSign
 * SubtractVeryBigHexSign
 * MultiplyBigHexSign    - Multiplies two 15*u16 numbers considering the sign.
 **********************************************************************************/

// AddP is only invoked when the sign of A is -ive
// A is always less than P
//
void AddP256(bigHex256 *bigHexA)
{
    bigHex256 BigHexResult;
    SubtractBigHex256(&bigHexP256, bigHexA, &BigHexResult);
    copyBigHex256(&BigHexResult, bigHexA);
    bigHexA->sign = 0;
}

void AddPdiv2_256(bigHex256 *bigHexA)
{
#if (ECC_MULT_ALGO_TYPE == 16)
    uint16_t *numA = bigHexA->num;
    const uint16_t *numB = bigHexP256.num;

    if (bigHexA->sign == 0)
    {
        uint32_t tmp;
        uint16_t carry;
        s_int32 i;

        carry = 0;

        for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
        {
            tmp = (uint32_t)((uint32_t)(numA[i]) + (uint32_t)(numB[i]) + carry);
            if (tmp & 0x10000)
                carry = 0x01;
            else
                carry = 0x00;

            numA[i] = (tmp & 0xFFFF);
        }
    }
    else // if (bigHexA->sign == 1)
    {
        if (isGreaterThan256(bigHexA, &bigHexP256))
        {
            // Due to the nature of the GF we have to assume both inputs are +ive.
            uint16_t borrow = 0;
            s_int32 i;

            for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
            {
                if (((uint32_t)((uint32_t)(numB[i]) + (uint32_t)borrow)) > (numA[i]))
                {
                    numA[i]  = (((uint32_t)(numA[i] + 0x10000)) - (numB[i] + borrow)) & 0xFFFF;
                    borrow = 0x01;
                }
                else
                {
                    numA[i] = ((numA[i]) - (numB[i] + borrow)) & 0xFFFF;
                    borrow = 0x00;
                }
            }
        }
        else
        {
            bigHex256 BigHexResult;

            SubtractBigHex256(&bigHexP256, bigHexA, &BigHexResult);
            copyBigHex256(&BigHexResult, bigHexA);
            bigHexA->sign = 0;
        }
    }
    // divide by 2
    // divideByTwo(bigHexA);
    {
        uint16_t rem = 0;
        uint32_t val;
        uint32_t i;

        for (i = 0; i < (ELEMENTS_BIG_HEX256); i++)
        {
            val = (uint32_t)((uint32_t)(numA[i]) + ((uint32_t)rem * 0x10000));

            numA[i] = (uint16_t)(val >> 1);
            rem = (uint32_t)(val - ((val >> 1) << 1));
        }

        // We need to re-calculate the length.
        setBigNumberLength256(bigHexA);
    }
#elif (ECC_MULT_ALGO_TYPE == 32)
    uint32_t *numA = bigHexA->num;
    const uint32_t *numB = bigHexP256.num;

    if (bigHexA->sign == 0)
    {
        uint32_t carry;
        s_int32 i;

        carry = 0;

        for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
        {
            u64 tmp = (u64)((u64)(numA[i]) + (u64)(numB[i]) + carry);
            if (tmp & _LL(0x100000000))
                carry = 0x01;
            else
                carry = 0x00;

            numA[i] = (tmp & 0xFFFFFFFF);
        }
    }
    else // if (bigHexA->sign == 1)
    {
        if (isGreaterThan256(bigHexA, &bigHexP256))
        {

            // Due to the nature of the GF we have to assume both inputs are +ive.
            uint32_t borrow = 0;
            s_int32 i;

            for (i = (ELEMENTS_BIG_HEX256 - 1); i >= 0; i--)
            {
                if (((u64)((u64)(numB[i]) + (u64)borrow)) > (numA[i]))
                {
                    numA[i]  = (((u64)(numA[i] + _LL(0x100000000))) - (numB[i] + borrow)) & 0xFFFFFFFF;
                    borrow = 0x01;
                }
                else
                {
                    numA[i] = ((numA[i]) - (numB[i] + borrow)) & 0xFFFFFFFF;
                    borrow = 0x00;
                }
            }
        }
        else
        {
            bigHex256 BigHexResult;

            SubtractBigHex256(&bigHexP256, bigHexA, &BigHexResult);
            copyBigHex256(&BigHexResult, bigHexA);
            bigHexA->sign = 0;
        }
    }
    // divide by 2
    // divideByTwo(bigHexA);
    {
        uint32_t rem = 0;
        uint32_t i;

        for (i = 0; i < (ELEMENTS_BIG_HEX256); i++)
        {
            u64 u64val = (u64)((u64)(numA[i]) + ((u64)rem * _LL(0x100000000)));

            numA[i] = (uint32_t)(u64val >> 1);
            rem = (u64)(u64val - ((u64val >> 1) << 1));
        }

        // We need to re-calculate the length.
        setBigNumberLength256(bigHexA);
    }
#endif // (ECC_MULT_ALGO_TYPE == 16)
}

void SubtractFromSelfBigHexSign256(bigHex256 *bigHexA, const bigHex256 *bigHexB)
{
    // This function uses use the basis AddBigHex and SubtractBigHex to implement a full
    // Addition which considers sign and mod.
    //
    if (bigHexA->sign == 0)
    {
        if (bigHexB->sign == 0)
        {
            if (isGreaterThanOrEqual256(bigHexA, bigHexB))
            {
                SubtractFromSelfBigHex256(bigHexA, bigHexB);
            }
            else
            {
                bigHex256 BigHexResult;

                SubtractBigHex256(bigHexB, bigHexA, &BigHexResult);
                copyBigHex256(&BigHexResult, bigHexA);
                bigHexA->sign = 1;
            }
        }
        else
        {
            // 3/  if A is + and B is -   C =   A+B  Mod
            Add2SelfBigHex256(bigHexA, bigHexB);
        }
    }
    else  // if (bigHexA->sign == 1)
    {
        if (bigHexB->sign == 0)
        {
            Add2SelfBigHex256(bigHexA, bigHexB);
        }
        else
        {
            if (isGreaterThanOrEqual256(bigHexB, bigHexA))
            {
                bigHex256 BigHexResult;

                SubtractBigHex256(bigHexB, bigHexA, &BigHexResult);
                copyBigHex256(&BigHexResult, bigHexA);
                bigHexA->sign = 0;
            }
            else
            {
                SubtractFromSelfBigHex256(bigHexA, bigHexB);
            }
        }
    }
}
/*****************************************************************************
 * Following functions further extend on the basic functions to include Mod P.
 *
 * AddBigHexMod  -- This function takes A and B which can be signed and output
 *                  a result C which is less than P.
 *                  It call AddBigHexSign and then perform Mod P on the output.
 *                  If the output is -ive it recursively add P until we get a +ive
 *                  number which is less than P.
 *                  If the output is +ive it recursively subtracts P until we get
 *                  a +number which is less than P.
 *
 * SubtractBigHexMod  -- This function takes A and B which can be signed and output
 *                  a result C which is less than P.
 *                  It calls SubtractBigHexSign and then performs Mod P on the output.
 *                  If the output is -ive it recursively add P until we get a +ive
 *                  number which is less than P.
 *                  If the output is +ive it recursively subtracts P until we get
 *                  a +number which is less than P.
 *
 * MultiplyBigHexMod
 ****************************************************************************/
void SubtractBigHexMod256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult)
{
    if (bigHexA->sign == 0)
    {
        if (bigHexB->sign == 0)
        {
            if (isGreaterThanOrEqual256(bigHexA, bigHexB))
            {
                SubtractBigHex256(bigHexA, bigHexB, BigHexResult);
                BigHexResult->sign = 0;
            }
            else
            {
                SubtractBigHex256(bigHexB, bigHexA, BigHexResult);
                BigHexResult->sign = 1;
            }
        }
        else
        {
            // 3/  if A is + and B is -   C =   A+B  Mod
            AddBigHex256(bigHexA, bigHexB, BigHexResult);
            BigHexResult->sign = 0;
        }
    }
    else  // if (bigHexA->sign == 1)
    {
        if (bigHexB->sign == 0)
        {
            AddBigHex256(bigHexA, bigHexB, BigHexResult);
            BigHexResult->sign = 1;
        }
        else
        {
            if (isGreaterThanOrEqual256(bigHexB, bigHexA))
            {
                SubtractBigHex256(bigHexB, bigHexA, BigHexResult);
                BigHexResult->sign = 0;
            }
            else
            {
                SubtractBigHex256(bigHexA, bigHexB, BigHexResult);
                BigHexResult->sign = 1;
            }
        }
    }

    if (BigHexResult->sign == 0x00)   // Positive Number
    {
        if (isGreaterThanOrEqual256(BigHexResult, &bigHexP256))
        {
            //SubtractFromSelfBigHexSign(BigHexResult,&bigHexP);
            SubtractFromSelfBigHex256(BigHexResult, &bigHexP256);
        }
    }
    else
    {
        AddP256(BigHexResult);
    }
}

// From curren t usage we can safely assume both inputs are positive.

void SubtractBigHexUint32_256(const bigHex256 *bigHexA, const uint32_t numB, bigHex256 *BigHexResult)
{
    initBigNumber256(BigHexResult);

    if (bigHexA->num[HIGHEST_INDEX_BIG_HEX256] >= numB)
    {
        copyBigHex256(bigHexA, BigHexResult);
        BigHexResult->num[ELEMENTS_BIG_HEX256 - 1] = bigHexA->num[ELEMENTS_BIG_HEX256 - 1] - numB;
    }
    else
    {
        bigHex256 N;

        initBigNumber256(&N);

        N.len = 1;
        N.num[HIGHEST_INDEX_BIG_HEX256] = numB;
        SubtractBigHexMod256(bigHexA, &N, BigHexResult);
    }
}


void MultiplyBigHexModP256(const bigHex256 *bigHexA, const bigHex256 *bigHexB, bigHex256 *BigHexResult)
{
    veryBigHex256 tmpResult;
    {
        s_int32 k;
#if (ECC_MULT_ALGO_TYPE == 16)
        const uint16_t *numA = bigHexA->num;
        const uint16_t *numB = bigHexB->num;
        uint16_t *result = tmpResult.num;
#elif (ECC_MULT_ALGO_TYPE == 32)
        const uint32_t *numA = bigHexA->num;
        const uint32_t *numB = bigHexB->num;
        uint32_t *result = tmpResult.num;
#endif // (ECC_MULT_ALGO_TYPE == 16)
        memset(tmpResult.num, 0, sizeof(tmpResult.num));
        tmpResult.len = 0;
        tmpResult.sign = 0;
        //
        // Below Optimisations seem to degrade performance on Windows by a factor of 2.
        //
        for (k = (ELEMENTS_BIG_HEX256 - 1); k >= 0; k--)
        {
            uint32_t mcarry = 0;
            s_int32 j = 0;

            // The inner loop multiplies each byte of HexB by a single byte of
            // HexA
            for (j = (ELEMENTS_BIG_HEX256 - 1); j >= 0; j--)
            {
#if (ECC_MULT_ALGO_TYPE == 16)
                uint32_t val;

                val = (((uint32_t)(numA[k])) * ((uint32_t)(numB[j]))) + result[j + k + 1] + mcarry;
                result[j + k + 1] = (val & 0xFFFF);
                mcarry = (uint16_t)(val >> 16);
#elif (ECC_MULT_ALGO_TYPE == 32)
                u64 val;

                val = (((u64)(numA[k])) * ((u64)(numB[j]))) + result[j + k + 1] + mcarry;
                result[j + k + 1] = (val & 0xFFFFFFFF);
                mcarry = (uint32_t)(val >> 32);
#endif // (ECC_MULT_ALGO_TYPE == 16)
            }
        }
        setVeryBigNumberLength256(&tmpResult);
        tmpResult.sign = (bigHexA->sign != bigHexB->sign);
    }

    {
        uint32_t i = 0;
        bigHex256 tmpResult2;

        while ((tmpResult.num[i] == 0x00) && (i < (ELEMENTS_BIG_HEX256)))
            i++;

        // Take the next 13 elements
        //
        tmpResult2.sign = tmpResult.sign;

        if (isVeryBigHexGreaterThanOrEqual256(&tmpResult, &veryBigHexP256))
        {
            while (tmpResult.num[i] == 0x00)
                i++;
        }
        memcpy(tmpResult2.num, tmpResult.num + i, sizeof(tmpResult2.num));
        setBigNumberLength256(&tmpResult2);

#if (ECC_MULT_ALGO_TYPE == 16)
        while ((i + ELEMENTS_BIG_HEX256) < (MAX_OCTETS256))
#elif (ECC_MULT_ALGO_TYPE == 32)
        while ((i + ELEMENTS_BIG_HEX256) < (MAX_OCTETS256 / 2))
#endif // (ECC_MULT_ALGO_TYPE == 16)
        {
            specialModP256(&tmpResult2);
            shiftLeftOneArrayElement256(&tmpResult2);

            // Add the next byte from A in left_over;
            tmpResult2.num[HIGHEST_INDEX_BIG_HEX256] = tmpResult.num[i + ELEMENTS_BIG_HEX256];
            i++;
            setBigNumberLength256(&tmpResult2);
        }

        specialModP256(&tmpResult2);
        copyBigHex256(&tmpResult2, BigHexResult);
    }
}


void MultiplyBigHexByUint32_256(const bigHex256 *bigHexA, const uint32_t numB, bigHex256 *BigHexResult)
{
#if (ECC_MULT_ALGO_TYPE == 16)
    s_int32 k;
    const uint16_t *numA = bigHexA->num;
    uint16_t *result = BigHexResult->num;
    uint16_t mcarry = 0;

    //
    // Below Optimisations seem to degrade performance on Windows by a factor of 2.
    //
    for (k = HIGHEST_INDEX_BIG_HEX256; k >= 0; k--)
    {
        uint32_t val;

        val = (((uint32_t)(numA[k])) * ((uint32_t)numB)) + mcarry;
        result[k] = (val & 0xFFFF);
        mcarry = (uint16_t)(val >> 16);

    }
#elif (ECC_MULT_ALGO_TYPE == 32)
    s_int32 k;
    const uint32_t *numA = bigHexA->num;
    uint32_t *result = BigHexResult->num;
    uint32_t mcarry = 0;

    //
    // Below Optimisations seem to degrade performance on Windows by a factor of 2.
    //
    for (k = HIGHEST_INDEX_BIG_HEX256; k >= 0; k--)
    {
        u64 val;

        val = (((u64)(numA[k])) * ((u64)numB)) + mcarry;
        result[k] = (val & 0xFFFFFFFF);
        mcarry = (uint32_t)(val >> 32);
    }
#endif // (ECC_MULT_ALGO_TYPE == 16)
    setBigNumberLength256(BigHexResult);
    BigHexResult->sign = bigHexA->sign;
    specialModP256(BigHexResult);
}


__INLINE__ void shiftLeftOneArrayElement256(bigHex256 *input)
{
    memmove(input->num, (input->num + 1), (8 * sizeof(uint32_t)));
}

//
// This function adds N * Mod P to a number.
//
// The left_over = tmpHexA % P

void specialModP256(bigHex256 *tmpHexA)
{
    bigHex256 NModP;

    initBigNumber256(&NModP);
    if (((tmpHexA->sign == 0) && isGreaterThanOrEqual256(tmpHexA, &bigHexP256)) ||
            (tmpHexA->sign == 1))
    {
        // If I multiply the most significant uint16_t of A by the Mod P and then subtract from A
        // this is equivalent to an iteractive subtraction - but much faster.
        if (tmpHexA->len > bigHexP256.len)
        {
            // approach
#if (ECC_MULT_ALGO_TYPE == 16)
            uint16_t SignificantU16 = GETMOSTSIGNIGICANTU16_256(tmpHexA);
            MultiplyByU16ModP256(SignificantU16, &NModP);
#elif (ECC_MULT_ALGO_TYPE == 32)
            uint32_t SignificantU32 = GETMOSTSIGNIGICANTU32_256(tmpHexA);
            MultiplyByU32ModP256(SignificantU32, &NModP);
#endif // (ECC_MULT_ALGO_TYPE == 16)
            //tmpHexA->num[0] = 0x00;

            if (tmpHexA->sign == 0)
            {
                SubtractFromSelfBigHexSign256(tmpHexA, &NModP);
            }
            else
            {
                // Change sign of A to +ive
                // Perform Subtraction
                // Change sign of A back to -ive
                tmpHexA->sign = 0;
                SubtractFromSelfBigHexSign256(tmpHexA, &NModP);
                tmpHexA->sign = 1;
            }
        }


        if (((tmpHexA->sign == 0) && isGreaterThanOrEqual256(tmpHexA, &bigHexP256)) ||
                (tmpHexA->sign == 1))
        {
            // It is implicit that the sign is negative.
            // so we can remove the check.
            //
            if (tmpHexA->sign == 1)
            {
                AddP256(tmpHexA);
            }
            else
            {
                // Can this be replaced by Adding the invert of P
                SubtractFromSelfBigHex256(tmpHexA, &bigHexP256);
            }
        }
    }
}
#if (ECC_MULT_ALGO_TYPE == 32)
void MultiplyByU32ModP256(const uint32_t N, bigHex256 *result)
{
    uint32_t tempVal = (0xFFFFFFFF - N) + 1;

    result->num[0] = N - 1;
    result->num[1] = (uint32_t)tempVal;
    result->num[2] = N ;
    result->num[3] = 0x00 ;
    result->num[4] = 0x00 ;
    result->num[5] = N - 1;
    result->num[6] = 0xFFFFFFFF;
    result->num[7] = 0xFFFFFFFF;
    result->num[8] = (uint32_t)tempVal;

    result->sign = 0x00;
    if (result->num[0])
        result->len = 0x9;
    else
        result->len = 0x8;
}
#endif

#if (ECC_MULT_ALGO_TYPE == 16)
void MultiplyByU16ModP256(const uint16_t N, bigHex256 *result)
{

    result->num[0] = N - 1;
    result->num[1] = 0xFFFF;
    result->num[2] = 0xFFFF - (N - 1);
    result->num[3] = 0;
    result->num[4] = N;
    result->num[5] = 0;
    result->num[6] = 0;
    result->num[7] = 0;
    result->num[8] = 0;
    result->num[9] = 0;
    result->num[10] = N - 1;
    result->num[11] = 0xFFFF;
    result->num[12] = 0xFFFF;
    result->num[13] = 0xFFFF;
    result->num[14] = 0xFFFF;
    result->num[15] = 0xFFFF;
    result->num[16] = 0xFFFF - (N - 1);

    result->sign = 0x00;
    if (result->num[0])
        result->len = 17;
    else
        result->len = 16;
}
#endif // (ECC_MULT_ALGO_TYPE == 16)

__INLINE__ void copyBigHex256(const bigHex256 *source, bigHex256 *destination)
{
    memcpy(destination->num, source->num, sizeof(destination->num)); //for P256 max_octets = 36

    destination->len = source->len;
    destination->sign = source->sign;
}

__INLINE__ void initBigNumber256(bigHex256 *BigHexResult)
{
    memset(BigHexResult->num, 0, sizeof(BigHexResult->num));

    BigHexResult->len = 0;
    BigHexResult->sign = 0;
}

__INLINE__ void setBigNumberLength256(bigHex256 *BigHexResult)
{
    int i;

    for (i = 0; i < (ELEMENTS_BIG_HEX256); i++)
    {
        if (BigHexResult->num[i] != 0x00)
            break;
    }
    BigHexResult->len = (ELEMENTS_BIG_HEX256) - i;
}

__INLINE__ void setVeryBigNumberLength256(veryBigHex256 *BigHexResult)
{
    int i;

#if (ECC_MULT_ALGO_TYPE == 16)
    for (i = 0; i < (MAX_OCTETS256); i++)
#elif (ECC_MULT_ALGO_TYPE == 32)
    for (i = 0; i < (MAX_OCTETS256 / 2); i++)
#endif // (ECC_MULT_ALGO_TYPE == 16)
    {
        if (BigHexResult->num[i] != 0x00)
            break;
    }
#if (ECC_MULT_ALGO_TYPE == 16)
    BigHexResult->len = (MAX_OCTETS256) - i;
#elif (ECC_MULT_ALGO_TYPE == 32)
    BigHexResult->len = (MAX_OCTETS256 / 2) - i;
#endif // (ECC_MULT_ALGO_TYPE == 16)
}

//
// if A > B return TRUE
//

__INLINE__ int isGreaterThan256(const bigHex256 *bigHexA, const bigHex256 *bigHexB)
{

    uint32_t i;
    uint32_t A_len = bigHexA->len;
    uint32_t B_len = bigHexB->len;

    if (A_len > B_len)
        return 1;
    else if (A_len < B_len)
        return 0;

    // Now we have two equal sized arrays which we need to compare.
    // .
    for (i = ((ELEMENTS_BIG_HEX256) - A_len); i < (ELEMENTS_BIG_HEX256); i++)
    {
        if (bigHexB->num[i] > bigHexA->num[i])
        {
            return 0;
        }
        else if (bigHexB->num[i] < bigHexA->num[i])
        {
            return 1;
        }
    }
    return 0;
}


__INLINE__ int isGreaterThanOrEqual256(const bigHex256 *bigHexA, const bigHex256 *bigHexB)
{
    uint32_t i;
    uint32_t A_len = bigHexA->len;
    uint32_t B_len = bigHexB->len;

    if (A_len > B_len)
        return 1;
    else if (A_len < B_len)
        return 0;

    // Now we have two equal sized arrays which we need to compare.
    // .

    for (i = ((ELEMENTS_BIG_HEX256) - A_len); i < (ELEMENTS_BIG_HEX256); i++)
    {
        if (bigHexB->num[i] > bigHexA->num[i])
        {
            return 0;
        }
        else if (bigHexB->num[i] < bigHexA->num[i])
        {
            return 1;
        }
    }
    return 1;
}


__INLINE__ int isVeryBigHexGreaterThanOrEqual256(const veryBigHex256 *bigHexA, const veryBigHex256 *bigHexB)
{
    int i;
    uint32_t A_len = bigHexA->len;
    uint32_t B_len = bigHexB->len;

    if (A_len > B_len)
        return 1;
    else if (A_len < B_len)
        return 0;

    // Now we have two equal sized arrays which we need to compare.
    //
#if (ECC_MULT_ALGO_TYPE == 16)
    for (i = ((MAX_OCTETS256) - A_len); i < (MAX_OCTETS256); i++)
#elif (ECC_MULT_ALGO_TYPE == 32)
    for (i = ((MAX_OCTETS256 / 2) - A_len); i < (MAX_OCTETS256 / 2); i++)
#endif // (ECC_MULT_ALGO_TYPE == 16)
    {

        if (bigHexB->num[i] != bigHexA->num[i])
        {
            return (bigHexB->num[i] < bigHexA->num[i]);
        }

    }
    return 1;
}


/*__INLINE__*/ int notEqual256(const bigHex256 *bigHexA, const bigHex256 *bigHexB)
{
    uint32_t i;

    for (i = 0; i < (ELEMENTS_BIG_HEX256); i++)
    {
        if (bigHexA->num[i] != bigHexB->num[i])
        {
            return 1;
        }
    }
    return 0;
}

__INLINE__ int Is_Infinite256(const ECC_Jacobian_Point256 *pPointP)
{
    return ((!notEqual256(&pPointP->x, &ecc_Jacobian_InfinityPoint256.x)) && (!notEqual256(&pPointP->y, &ecc_Jacobian_InfinityPoint256.y)) && (!notEqual256(&pPointP->z, &ecc_Jacobian_InfinityPoint256.z)));
}

/*********** JACOBIAN ******/
int GF_Jacobian_Point_Addition256(const ECC_Jacobian_Point256 *pPointP, const ECC_Jacobian_Point256 *pPointQ, ECC_Jacobian_Point256 *pResultPoint)
{

// This should be 12 Multiplications and 2 Squares
// currently implemented as 12 Muls and 2 Squares and one cube
//  Let (X1, Y1, Z1) and (X2, Y2, Z2) be two points (both unequal to the 'point at infinity') represented in 'Standard Projective Coordinates'. Then the sum (X3, Y3, Z3) can be calculated by
//
//  U1 = Y2*Z1
//  U2 = Y1*Z2
//  V1 = X2*Z1
//  V2 = X1*Z2
//  if (V1 == V2)
//     if (U1 != U2)
//         return POINT_AT_INFINITY
//      else
//         return POINT_DOUBLE(X1, Y1, Z1)
// U = U1 - U2
// V = V1 - V2
// W = Z1*Z2

// A = U^2*W - V^3 - 2*V^2*V2
// X3 = V*A
// Y3 = U*(V^2*V2 - A) - V^3*U2
// Z3 = V^3*W
//  return (X3, Y3, Z3)
    bigHex256 *addition_memory = ke_malloc_system(sizeof(struct bigHex256) * 12, KE_MEM_NON_RETENTION);
    uint32_t offset = 0;
    bigHex256 *jac_U1 = addition_memory + offset++;
    bigHex256 *jac_U2 = addition_memory + offset++;
    bigHex256 *jac_V1 = addition_memory + offset++;
    bigHex256 *jac_V2 = addition_memory + offset++;
    bigHex256 *jac_U = addition_memory + offset++;
    bigHex256 *jac_V = addition_memory + offset++;
    bigHex256 *jac_A = addition_memory + offset++;
    bigHex256 *jac_W = addition_memory + offset++;
    bigHex256 *V_sqr = addition_memory + offset++;
    bigHex256 *V_cube = addition_memory + offset++;
    bigHex256 *U_sqr = addition_memory + offset++;
    bigHex256 *V_sqr_mult_by_V2 = addition_memory + offset;


    const bigHex256 *pJac_PointX1 = &pPointP->x;
    const bigHex256 *pJac_PointY1 = &pPointP->y;
    const bigHex256 *pJac_PointZ1 = &pPointP->z;

    const bigHex256 *pJac_PointX2 = &pPointQ->x;
    const bigHex256 *pJac_PointY2 = &pPointQ->y;
    const bigHex256 *pJac_PointZ2 = &pPointQ->z;

    initBigNumber256(&pResultPoint->x);
    initBigNumber256(&pResultPoint->y);
    initBigNumber256(&pResultPoint->z);

    initBigNumber256(jac_U1);
    initBigNumber256(jac_U2);
    initBigNumber256(jac_V1);
    initBigNumber256(jac_V2);

    initBigNumber256(jac_U);
    initBigNumber256(jac_V);
    initBigNumber256(jac_A);
    initBigNumber256(jac_W);

    initBigNumber256(V_sqr);
    initBigNumber256(V_cube);
    initBigNumber256(U_sqr);
    initBigNumber256(V_sqr_mult_by_V2);


    // First Handle points of infinity
    /*
    P1 and P2 are both infinite: P3=infinite.
    P1 is infinite: P3=P2.
    P2 is infinite: P3=P1.
    P1 and P2 have the same x coordinate, but different y coordinates or both y coordinates equal 0: P3=infinite.

    */
    if (Is_Infinite256(pPointP))
    {
        if (Is_Infinite256(pPointQ))
        {
            // Result = Infinity
            //pResultPoint = Infinity
            GF_Jacobian_Point_Copy256(&ecc_Jacobian_InfinityPoint256, pResultPoint);
            ke_free(addition_memory);
            return 0;
        }
        else
        {
            // Result = pPointQ
            // pResultPoint = pPointQ
            GF_Jacobian_Point_Copy256(pPointQ, pResultPoint);
            ke_free(addition_memory);
            return 0;
        }
    }
    else if (Is_Infinite256(pPointQ))
    {
        // Result = pPointP
        // pResultPoint = pPointP;
        GF_Jacobian_Point_Copy256(pPointP, pResultPoint);
        ke_free(addition_memory);
        return 0;
    }

    if (!notEqual256(&pPointQ->x, &pPointP->x)) // Same X coordinate
    {
        if (notEqual256(&pPointQ->y, &pPointP->y)) // Different Y coordinates
        {
            // Result = Infinity
            GF_Jacobian_Point_Copy256(&ecc_Jacobian_InfinityPoint256, pResultPoint);
            ke_free(addition_memory);
            return 0;

        }
        else if ((pPointQ->y.len <= 1) && (pPointQ->y.num[HIGHEST_INDEX_BIG_HEX256] == 0)) // Y co-ords = 0
        {
            // Result = Infinity
            GF_Jacobian_Point_Copy256(&ecc_Jacobian_InfinityPoint256, pResultPoint);
            ke_free(addition_memory);
            return 0;
        }
    }

//  U1 = Y2*Z1
//  U2 = Y1*Z2
//  V1 = X2*Z1
//  V2 = X1*Z2

    MultiplyBigHexModP256(pJac_PointY2, pJac_PointZ1, jac_U1);
    MultiplyBigHexModP256(pJac_PointY1, pJac_PointZ2, jac_U2);
    MultiplyBigHexModP256(pJac_PointX2, pJac_PointZ1, jac_V1);
    MultiplyBigHexModP256(pJac_PointX1, pJac_PointZ2, jac_V2);

//  if (V1 == V2)
//     if (U1 != U2)
//         return POINT_AT_INFINITY
//      else
//         return POINT_DOUBLE(X1, Y1, Z1)

    if (!notEqual256(jac_V1, jac_V2))
    {
        if (notEqual256(jac_U1, jac_U2))
        {
            // Result = Infinity
            GF_Jacobian_Point_Copy256(&ecc_Jacobian_InfinityPoint256, pResultPoint);
            ke_free(addition_memory);
            return 0;
        }
        else
        {
            GF_Jacobian_Point_Double256(pPointP, pResultPoint);
            ke_free(addition_memory);
            return 0; //POINT_DOUBLE(&jac_PointX1,&jac_PointY1,&jac_PointZ1);
        }
    }

// U = U1 - U2
// V = V1 - V2
// W = Z1*Z2   ( 5 muls to here )
// A = U^2*W - V^3 - 2*V^2*V2

    SubtractBigHexMod256((jac_U1), (jac_U2), jac_U);
    SubtractBigHexMod256((jac_V1), (jac_V2), jac_V);

    MultiplyBigHexModP256(pJac_PointZ1, pJac_PointZ2, jac_W);

    {
        // Determine A
        // A = U^2*W - V^3 - 2*V^2*V2
        // V_sqr = V^2
        // V_cube = V^3
        // U_sqr = U^2
        // int1 = V_sqr * V2
        // int2 = 2 * int1  --  (2*V^2*V2)
        // int3 = U_sqr * W
        // int4 = int3 - V_cube
        // A = int4 - int2

        // X3 = D^2 - int3

        bigHex256 int1;
        bigHex256 int3;
        bigHex256 int4;
        bigHex256 Double_V_sqr_mult_by_V2;

        initBigNumber256(&int1);
        initBigNumber256(&int3);
        initBigNumber256(&int4);
        initBigNumber256(&Double_V_sqr_mult_by_V2);

        MultiplyBigHexModP256(jac_V, jac_V, V_sqr);
        MultiplyBigHexModP256(V_sqr, jac_V, V_cube);
        MultiplyBigHexModP256(jac_U, jac_U, U_sqr);

        MultiplyBigHexModP256(V_sqr, jac_V2, V_sqr_mult_by_V2); // V^2 * V2

        MultiplyBigHexByUint32_256(V_sqr_mult_by_V2, 0x02, &Double_V_sqr_mult_by_V2); // 2 * V^2 * V2
        MultiplyBigHexModP256(U_sqr, jac_W, &int3);  // U^2 * W
        SubtractBigHexMod256(&int3, V_cube, &int4); // (U^2 * W) - V^3
        SubtractBigHexMod256(&int4, &Double_V_sqr_mult_by_V2, jac_A); // A = U^2*W - V^3 - 2*V^2*V2
    }

    // Determine X3
    {
        // X3 = V*A

        MultiplyBigHexModP256(jac_V, jac_A, &(pResultPoint->x));
    }
    // Determine Y3
    {
        // Y3 = U*(V^2*V2 - A) - V^3*U2
        // int1 = (V^2*V2 - A)
        // int2 = U*(V^2*V2 - A)
        // int3 =  V^3*U2
        bigHex256 int1;
        bigHex256 int2;
        bigHex256 int3;

        initBigNumber256(&int1);
        initBigNumber256(&int2);
        initBigNumber256(&int3);

        SubtractBigHexMod256(V_sqr_mult_by_V2, jac_A, &int1); // int1 =( V^2*V2 - A)
        // Below line was incorrect

        MultiplyBigHexModP256(jac_U, &int1, &int2); // int2 = U * ( V^2*V2 - A)
        //
        MultiplyBigHexModP256(V_cube, jac_U2, &int3); // int3 = V^3 * U2
        SubtractBigHexMod256(&int2, &int3, &(pResultPoint->y));
    }


    // Determine Z3
    {
        // Z3 = V^3*W
        MultiplyBigHexModP256(V_cube, jac_W, &(pResultPoint->z));
    }

    ke_free(addition_memory);
    return 1;
}

int GF_Jacobian_Point_Double256(const ECC_Jacobian_Point256 *pPointP, ECC_Jacobian_Point256 *pResultPoint)
{
//if (Y == 0)
// return POINT_AT_INFINITY
//W = a*Z^2 + 3*X^2
// if a = -3, then W can also be calculated as W = 3*(X + Z)*(X - Z), saving 2 field squarings
// THUS
//W = 3*(X+Z)*(X-Z)
//S = Y*Z
//B = X*Y*S
//H = W^2 - 8*B
//X' = 2*H*S
//Y' = W*(4*B - H) - 8*Y^2*S^2
//Z' = 8*S^3
//return (X', Y', Z')

    const bigHex256 *pJac_PointX1 = &pPointP->x;
    const bigHex256 *pJac_PointY1 = &pPointP->y;
    const bigHex256 *pJac_PointZ1 = &pPointP->z;
    bigHex256 *addition_memory = ke_malloc_system(sizeof(struct bigHex256) * 11, KE_MEM_NON_RETENTION);
    uint32_t offset = 0;
    bigHex256 *W = addition_memory + offset++;
    bigHex256 *S = addition_memory + offset++;
    bigHex256 *B = addition_memory + offset++;
    bigHex256 *H = addition_memory + offset++;
    bigHex256 *S_sqr = addition_memory + offset++;

    if (((pJac_PointY1->len <= 1) && (pJac_PointY1->num[HIGHEST_INDEX_BIG_HEX256] == 0x00)) ||
            Is_Infinite256(pPointP))
    {
        // Point at Infinity
        // pResultPoint = Infinity
        GF_Jacobian_Point_Copy256(&ecc_Jacobian_InfinityPoint256, pResultPoint);
        ke_free(addition_memory);
        return 0;
    }

    // Determine W
    {
        bigHex256 X_plus_Z;
        bigHex256 X_minus_Z;
        bigHex256 int1;

        // Need to convert below to AddBigHexModP to consider sign and mod..
        AddBigHexModP256(pJac_PointX1, pJac_PointZ1, &X_plus_Z);
        SubtractBigHexMod256(pJac_PointX1, pJac_PointZ1, &X_minus_Z);
        MultiplyBigHexModP256(&X_plus_Z, &X_minus_Z, &int1);
        MultiplyBigHexByUint32_256(&(int1), 0x03, W);
    }
    // Determine S
    {
        MultiplyBigHexModP256(pJac_PointY1, pJac_PointZ1, S);
    }
    // Determine B
    {
        bigHex256 int1;

        //B = X*Y*S
        MultiplyBigHexModP256(pJac_PointY1, S, &int1);
        MultiplyBigHexModP256(pJac_PointX1, &int1, B);
    }

    // Determine H
    {
        //H = W^2 - 8*B
        bigHex256 W_sqr;
        bigHex256 Eight_B;

        MultiplyBigHexModP256(W, W, &W_sqr);
        MultiplyBigHexByUint32_256(B, 8, &Eight_B);

        SubtractBigHexMod256(&W_sqr, &Eight_B, H);
    }
    // Determine X
    //X = 2*H*S
    {
        bigHex256 int1;

        MultiplyBigHexModP256(H, S, &int1);
        MultiplyBigHexByUint32_256((const bigHex256 *)(&int1), 0x02, &pResultPoint->x);

    }
    // Determine Y
    // Y' = W*(4*B - H) - 8*Y^2*S^2

    {
        bigHex256 *Y_sqr = addition_memory + offset++;
        bigHex256 *int1 = addition_memory + offset++;
        bigHex256 *int2 = addition_memory + offset++; // 8*Y^2*S^2
        bigHex256 *int3 = addition_memory + offset++;
        bigHex256 *int4 = addition_memory + offset++;
        bigHex256 *int5 = addition_memory + offset;

        // 8*Y^2*S^2
        MultiplyBigHexModP256(pJac_PointY1, pJac_PointY1, Y_sqr);
        MultiplyBigHexModP256(S, S, S_sqr);
        MultiplyBigHexModP256(Y_sqr, S_sqr, int1);
        MultiplyBigHexByUint32_256((int1), 0x08, int2);

        // 4 * B
        MultiplyBigHexByUint32_256(B, 0x04, int3);
        // (4*B - H)
        SubtractBigHexMod256(int3, H, int4);
        // W * (4*B - H)
        MultiplyBigHexModP256(W, int4, int5);

        SubtractBigHexMod256((const bigHex256 *)int5, (const bigHex256 *)int2, &pResultPoint->y);

    }

    // Determine Z
    {
        //Z1 = 8*S^3
        bigHex256 int1;

        MultiplyBigHexModP256(S_sqr, S, &int1);
        MultiplyBigHexByUint32_256(&int1, 0x08, &pResultPoint->z);
    }

    ke_free(addition_memory);
    return 0;
}

void GF_Point_Jacobian_To_Affine256(ECC_Jacobian_Point256 *pJacPoint, bigHex256 *pX_co_ord_affine, bigHex256 *pY_co_ord_affine)
{
    bigHex256 inverted_Z_jacobian;

    // x_affine = X_jacobian/Z_jacobian
    // y_affine = Y_jacobian/Z_jacobian

    DBG_SWDIAG(ECDH, INV, 1);
    // First Determine 1/Z_jacobian
    bigHexInversion256(&pJacPoint->z, &inverted_Z_jacobian);
    DBG_SWDIAG(ECDH, INV, 0);

    MultiplyBigHexModP256(&pJacPoint->x, &inverted_Z_jacobian, pX_co_ord_affine);
    MultiplyBigHexModP256(&pJacPoint->y, &inverted_Z_jacobian, pY_co_ord_affine);

}


__INLINE__ void GF_Point_Copy256(const ECC_Point256 *source, ECC_Point256 *destination)
{
    copyBigHex256(&source->x, &destination->x);
    copyBigHex256(&source->y, &destination->y);
}


__INLINE__ void GF_Jacobian_Point_Copy256(const ECC_Jacobian_Point256 *source, ECC_Jacobian_Point256 *destination)
{
    copyBigHex256(&source->x, &destination->x);
    copyBigHex256(&source->y, &destination->y);
    copyBigHex256(&source->z, &destination->z);
}

__INLINE__ void GF_Affine_To_Jacobian_Point_Copy256(const ECC_Point256 *source, ECC_Jacobian_Point256 *destination)
{
    bigHex256 BigHex_1;

    BigHex_1.len = 0x01;
    BigHex_1.sign = 0; // Positive Number
    memset(BigHex_1.num, 0, sizeof(BigHex_1.num));
    BigHex_1.num[HIGHEST_INDEX_BIG_HEX256] = 0x01;
    copyBigHex256(&source->x, &destination->x);
    copyBigHex256(&source->y, &destination->y);
    copyBigHex256(&BigHex_1, &destination->z);
}

__INLINE__ void GF_Setup_Jacobian_Infinity_Point256(ECC_Jacobian_Point256 *infinity)
{
    bigHex256 BigHex_1;

    BigHex_1.len = 0x01;
    BigHex_1.sign = 0; // Positive Number
    memset(BigHex_1.num, 0, sizeof(BigHex_1.num));
    BigHex_1.num[HIGHEST_INDEX_BIG_HEX256] = 0x00;
    copyBigHex256(&BigHex_1, &infinity->x);
    BigHex_1.num[HIGHEST_INDEX_BIG_HEX256] = 0x01;
    copyBigHex256(&BigHex_1, &infinity->y);
    BigHex_1.num[HIGHEST_INDEX_BIG_HEX256] = 0x00;
    copyBigHex256(&BigHex_1, &infinity->z);

}

/**************************************************************
 *  Function :- bigHexInversion
 *
 *  Parameters :-  bigHexA - a pointer to bigHex - the input
 *                 pResult - a pointer to bigHex - the output
 *
 *  Description
 *  This function performs the inversion of a bigHex number. The output multiplied
 *  by the input will result in '1', as the output is Moded by P.
 *
 *            ( bigHexA * pResult ) % P = 1.
 *
 * Ref " Software Implementation of the NIST Elliptical "
 *  Input : Prime P (bigHexP), a (bigHexA) E [1, p-1]
 *  Output : a^-1 mod P
 ************************************************************************/

void bigHexInversion256(bigHex256 *bigHexA, bigHex256 *pResult)
{
    bigHex256 u;
    bigHex256 v;
    bigHex256 A;
    bigHex256 C;

    // Change the sign to positive
    bigHexA->sign = 0;

    // Step 1
    // u <-- a, v <-- p, A <-- 1, C <-- 0.
    //
    initBigNumber256(&A);
    initBigNumber256(&C);

    copyBigHex256(bigHexA, &u);
    copyBigHex256(&bigHexP256, &v);

    A.num[HIGHEST_INDEX_BIG_HEX256] = 1;
    C.num[HIGHEST_INDEX_BIG_HEX256] = 0;
    A.len = 1;
    C.len = 1;

    //
    // Step 2.
    //   While u != 0 do
    //   2.1  While u is even do :
    //           u <-- u/2. If A is even then A <-- A/2; else A <-- (A+P)/2
    //   2.2  While v is even do :
    //           v <-- v/2. If C is even then C <-- C/2; else C <-- (C+P)/2
    //   2.3  If u >= v then : u <-- u - v, A <-- A - C; else v <-- v - u, C <-- C - A
    //
    while (u.len != 0)
    {
        while (ISBIGHEXEVEN256(u))
        {
            divideByTwo256(&u);
            if (ISBIGHEXEVEN256(A))
            {
                // A = A/2
                divideByTwo256(&A);
            }
            else
            {
                AddPdiv2_256(&A);
            }
        }
        while (ISBIGHEXEVEN256(v))
        {
            divideByTwo256(&v);
            if (ISBIGHEXEVEN256(C))
            {
                divideByTwo256(&C);
            }
            else
            {
                AddPdiv2_256(&C);
            }
        }
        if (isGreaterThanOrEqual256(&u, &v))
        {
            SubtractFromSelfBigHex256(&u, &v);
            SubtractFromSelfBigHexSign256(&A, &C);
        }
        else
        {
            SubtractFromSelfBigHex256(&v, &u);
            SubtractFromSelfBigHexSign256(&C, &A);
        }
    }
    // Step 3 :- Now perform Mod P
    // pResult = C % P
    {
        // If the Mod P is greater than bigHexA then return with
        // C unmodified.

        if (C.sign == 0)
        {
            if (isGreaterThan256(&bigHexP256, &C))
            {
                copyBigHex256(&C, pResult);
                return;
            }
            else // Positive Number thus subtract P iteratively.
            {
                specialModP256(&C);
            }
        }
        else // Negative Number
        {
            specialModP256(&C);
        }
        copyBigHex256(&C, pResult);
    }
}

/*******************************************************************
 * Funcion :- divideByTwo
 *
 * Parameters :- A - a bigHex pointer - which is the divided by two
 *
 *******************************************************************/

void divideByTwo256(bigHex256 *A)
{
#if (ECC_MULT_ALGO_TYPE == 16)
    uint16_t rem = 0;
    uint32_t val;
    uint32_t i;

    for (i = 0; i < (ELEMENTS_BIG_HEX256); i++)
    {
        val = (uint32_t)((uint32_t)(A->num[i]) + ((uint32_t)(rem << 16)));

        A->num[i] = (uint16_t)(val >> 1);
        rem = (uint32_t)(val - ((val >> 1) << 1));
    }
#elif (ECC_MULT_ALGO_TYPE == 32)
    uint32_t rem = 0;
    uint32_t i;

    for (i = 0; i < (ELEMENTS_BIG_HEX256); i++)
    {
        u64 u64val = (u64)((u64)(A->num[i]) + ((u64)rem * _LL(0x100000000)));

        A->num[i] = (uint32_t)(u64val >> 1);
        rem = (u64)(u64val - ((u64val >> 1) << 1));
    }
#endif // (ECC_MULT_ALGO_TYPE == 16)
    // We need to re-calculate the length.
    setBigNumberLength256(A);
}

//    Q = kP
//
/********************************************************
 * Function :- ECC_Point_Multiplication
 *
 * Parameters :- pk       - a pointer to a bigHex which the point should be multiplied by.
 *               pPointP  - pointer to the point which is to be multiplied by pk
 *
 * Description
 * This function performs ECC point multiplication. It uses the Scalar Multiplication
 * algorithm. Scalar multiplication works is way through the bits of a BigHex starting with the LSB.
 * For each bit (irrespective of Value) a point Doubling is performed, if the bit has a value = 1 then
 * a point addition is also performed.
 *
 * Scalar Multiplication: LSB first
 *   - Require k=(km-1,km-2,...,k0)2, km=1
 *   - Compute Q=kP
 * - Q=0, R=P
 * - For i=0 to m-1
 *   - If ki=1 then
 *      - Q=Q+R
 *   - End if
 *   - R=2R
 * - End for
 * - Return Q
 ******************************************************************************************/
__STATIC void ecc_point_multiplication_win_256(struct ecc_elt_tag *ecc_elt)
{
    ECC_Jacobian_Point256 tmpResultPoint;
    ECC_Jacobian_Point256 tmpResultPoint2;
    ECC_Jacobian_Point256 *ecc_lookup_table;
    ECC_Jacobian_Point256 *jPointQ256 = &(ecc_elt->Jacobian_PointQ256);

#if (ECC_MULT_ALGO_TYPE == 16)
    uint16_t i = ecc_elt->bit_cursor;
#else
    uint32_t i = ecc_elt->bit_cursor;
#endif
    // Give i -- need to work out the index val.
    // i = 0 -> indexVal = bit192:bit128;bit64:bit0
    // i = 1 -> indexVal = bit193:bit129;bit65:bit1

    if ((ecc_elt->key_type == ECC_PUBLICKEY_GENERATION) || (ecc_elt->key_type == ECC_DHKEY_GENERATION))
    {
        uint8_t IndexVal;

        if (ecc_elt->key_type == ECC_PUBLICKEY_GENERATION)
            ecc_lookup_table = (ECC_Jacobian_Point256 *)&ECC_4Win_Look_up_table[0];
        else
            ecc_lookup_table = &ecc_elt->win_4_table[0];

        IndexVal = ((ecc_elt->Pk256.num[((255 - i) / ECC_MULT_ALGO_TYPE) + 1] >> (i % ECC_MULT_ALGO_TYPE)) & 0x01) +
                   (((ecc_elt->Pk256.num[((191 - i) / ECC_MULT_ALGO_TYPE) + 1] >> (i % ECC_MULT_ALGO_TYPE)) & 0x01) * 0x02) +
                   (((ecc_elt->Pk256.num[((127 - i) / ECC_MULT_ALGO_TYPE) + 1] >> (i % ECC_MULT_ALGO_TYPE)) & 0x01) * 0x04) +
                   (((ecc_elt->Pk256.num[((63 - i) / ECC_MULT_ALGO_TYPE) + 1]  >> (i % ECC_MULT_ALGO_TYPE)) & 0x01) * 0x08);

        if (IndexVal != 0)
        {
            GF_Jacobian_Point_Copy256(&ecc_lookup_table[IndexVal - 1], &tmpResultPoint);
            GF_Jacobian_Point_Addition256(&tmpResultPoint, jPointQ256, &tmpResultPoint2);
            GF_Jacobian_Point_Copy256(&tmpResultPoint2, jPointQ256);
        }

        if (ecc_elt->bit_cursor != 0)
        {
            // Point Doubling
            // R = 2R
            GF_Jacobian_Point_Double256(jPointQ256, &tmpResultPoint2);
            // Copy Result point to PointR
            GF_Jacobian_Point_Copy256(&tmpResultPoint2, jPointQ256);
            ecc_elt->bit_cursor--;
        }
        else
        {
            ecc_elt->Point_Mul_Word256 = 0;
        }
    }
    else if (ecc_elt->key_type == ECC_4BIT_TABLE_GENERATION_PT1)
    {
        ///
        /// Perform Double of the point R - copy result to R.
        GF_Jacobian_Point_Double256(&ecc_elt->Jacobian_PointR256, &tmpResultPoint);
        GF_Jacobian_Point_Copy256(&tmpResultPoint, &ecc_elt->Jacobian_PointR256);
        ecc_elt->bit_cursor++;

        if (ecc_elt->bit_cursor == 64)
        {
            GF_Jacobian_Point_Copy256(&ecc_elt->Jacobian_PointR256, &ecc_elt->win_4_table[1]);
        }
        else if (ecc_elt->bit_cursor == 128)
        {

            GF_Jacobian_Point_Copy256(&ecc_elt->Jacobian_PointR256, &ecc_elt->win_4_table[3]);
        }
        else if (ecc_elt->bit_cursor == 192)
        {

            GF_Jacobian_Point_Copy256(&ecc_elt->Jacobian_PointR256, &ecc_elt->win_4_table[7]);

            // Now move onto second part of generating the table -
            ecc_elt->key_type = ECC_4BIT_TABLE_GENERATION_PT2;
            ecc_elt->bit_cursor = 3;
        }

    }
    else if (ecc_elt->key_type == ECC_4BIT_TABLE_GENERATION_PT2)
    {

        if (ecc_elt->bit_cursor == 3) // Construct - table entry corresponding to BitVal = 0011 (table index 2)
        {
            GF_Jacobian_Point_Addition256(&ecc_elt->win_4_table[0], &ecc_elt->win_4_table[1], &ecc_elt->win_4_table[2]);
            // Next entry is BitVal == 0101 - Table Index 4
            ecc_elt->bit_cursor = 5;
        }

        else if (ecc_elt->bit_cursor == 5)
        {
            GF_Jacobian_Point_Addition256(&ecc_elt->win_4_table[0], &ecc_elt->win_4_table[3], &ecc_elt->win_4_table[4]);
            // Next entry is BitVal == 0110 - Table Index 5
            ecc_elt->bit_cursor = 6;
        }
        else if (ecc_elt->bit_cursor == 6)
        {
            // Was 1 + 3 = 5     0010 + 0100 =0110

            GF_Jacobian_Point_Addition256(&ecc_elt->win_4_table[1], &ecc_elt->win_4_table[3], &ecc_elt->win_4_table[5]);
            // Next entry is BitVal == 0111 - Table Index 6
            ecc_elt->bit_cursor = 7;
        }
        else if (ecc_elt->bit_cursor == 7)
        {
            GF_Jacobian_Point_Addition256(&ecc_elt->win_4_table[0], &ecc_elt->win_4_table[5], &ecc_elt->win_4_table[6]);
            // Next entry is BitVal == 1001 - Table Index 8
            ecc_elt->bit_cursor = 9;
        }
        else if ((ecc_elt->bit_cursor >= 9) && (ecc_elt->bit_cursor <= 15)) // between 1001b and 1111b
        {
            GF_Jacobian_Point_Addition256(&ecc_elt->win_4_table[7], &ecc_elt->win_4_table[ecc_elt->bit_cursor - 9], &ecc_elt->win_4_table[ecc_elt->bit_cursor - 1]);
            if (ecc_elt->bit_cursor == 15) // TABLE Generation complete
            {
                // Move on to the usage of the Table to generate the DH_KEY
                ecc_elt->key_type = ECC_DHKEY_GENERATION;
                ecc_elt->bit_cursor = 63;
            }
            else
            {
                ecc_elt->bit_cursor++;
            }
        }
    }
}

__STATIC bool ecc_is_valid_point(bigHex256 *X_coord, bigHex256 *Y_coord)
{
    // Check to see if X and Y point exist on the ECDH curve.
    // This is only required for Public Keys recieved from the peer.
    // Y^2 = X^3 - 3X  + b

    bigHex256 Y_square;
    bigHex256 X_square;
    bigHex256 X_cube;
    bigHex256 double_X;
    bigHex256 tripple_X;
    bigHex256 interim_result1;
    bigHex256 interim_result2;

    initBigNumber256(&Y_square);
    initBigNumber256(&X_square);
    initBigNumber256(&X_cube);

    initBigNumber256(&double_X);
    initBigNumber256(&tripple_X);
    initBigNumber256(&interim_result1);
    initBigNumber256(&interim_result2);

    // First Determine Y^2
    MultiplyBigHexModP256(Y_coord, Y_coord, &Y_square);

    // Now Determine X^3
    MultiplyBigHexModP256(X_coord, X_coord, &X_square);
    MultiplyBigHexModP256(X_coord, &X_square, &X_cube);

    // Determine 3x
    AddBigHexModP256(X_coord, X_coord, &double_X);
    AddBigHexModP256(&double_X, X_coord, &tripple_X);

    // X^3 -3x
    SubtractBigHexMod256(&X_cube, &tripple_X, &interim_result1);

    // interim_result + coeficient_b
    AddBigHexModP256(&interim_result1, &coef_B, &interim_result2);

    // Now check if Y^2 = interim_result2 (x^3 -3x + b)
    return (memcmp(&Y_square.num[0], &interim_result2.num[0], MAX_OCTETS256) == 0);
}

/**
 ****************************************************************************************
 * @brief ECC ecc_multiplication event handler
 ****************************************************************************************
 */
__STATIC void ecc_multiplication_event_handler(void)
{
    DBG_SWDIAG(ECDH, COMPUTE, 1);
    // Take the next multiplication
    struct ecc_elt_tag *ecc_elt = (struct ecc_elt_tag *) co_list_pop_front(&ecc_env.ongoing_mul);

    led_set(4);

    // Check if element is present
    ASSERT_ERR(ecc_elt != NULL);
    if (ecc_elt != NULL)
    {
        // Check that current multiplication is not finished
        if (ecc_elt->Point_Mul_Word256 > 0)
        {
            DBG_SWDIAG(ECDH, MULT, 1);

            // Execute 1 multiplication step (with 4 bits at a time)
            ecc_point_multiplication_win_256(ecc_elt);

            // Re-insert the multiplication at the end of the list
            co_list_push_back(&ecc_env.ongoing_mul, &ecc_elt->hdr);
            DBG_SWDIAG(ECDH, MULT, 0);
        }
        else
        {
            DBG_SWDIAG(ECDH, END, 1);
            ecc_p256_result_t result;
#ifdef CFG_ECC_SIM_ACCEL
            if (ecc_env.use_simu_accelerator)
            {
                result = ecc_elt->result;
            }
            else
#endif // CFG_ECC_SIM_ACCEL
            {
                // Multiplication completed, report to the client
                int32_t i, j;
                ECC_Point256 pointQ256;

                initBigNumber256(&pointQ256.x);
                initBigNumber256(&pointQ256.y);

                GF_Point_Jacobian_To_Affine256(&(ecc_elt->Jacobian_PointQ256), &pointQ256.x, &pointQ256.y);

                // Free the memory previously allocated for Jacobian points Q and R and the private key
                if (ecc_elt->win_4_table != NULL)
                {
                    ke_free(ecc_elt->win_4_table);
                }

                // Copy result keys X coordinate # LSB first
                for (i = 31, j = 1; i >= 0;) // Keys Res is MSB - make it in LSB
                {
#if (ECC_MULT_ALGO_TYPE == 16)
                    result.key_res_x[i] = ((pointQ256.x.num[j] & 0xFF00) >> 8);
                    result.key_res_x[i - 1] = (pointQ256.x.num[j] & 0x00FF);
                    i -= 2;
                    j++;
#elif (ECC_MULT_ALGO_TYPE == 32)
                    result.key_res_x[i]   = ((pointQ256.x.num[j] & 0xFF000000) >> 24);
                    result.key_res_x[i - 1] = ((pointQ256.x.num[j] & 0x00FF0000) >> 16);
                    result.key_res_x[i - 2] = ((pointQ256.x.num[j] & 0x0000FF00) >> 8);
                    result.key_res_x[i - 3] = (pointQ256.x.num[j] & 0x000000FF);
                    i -= 4;
                    j++;
#endif // (ECC_MULT_ALGO_TYPE == 16)
                }

                // Copy result keys Y coordinate
                for (i = 31, j = 1; i >= 0;) // Keys Res is MSB - make it in LSB
                {
#if (ECC_MULT_ALGO_TYPE == 16)
                    result.key_res_y[i]  = ((pointQ256.y.num[j] & 0xFF00) >> 8);
                    result.key_res_y[i - 1] = (pointQ256.y.num[j] & 0x00FF);

                    i -= 2;
                    j++;
#elif (ECC_MULT_ALGO_TYPE == 32)
                    result.key_res_y[i]   = ((pointQ256.y.num[j] & 0xFF000000) >> 24);
                    result.key_res_y[i - 1] = ((pointQ256.y.num[j] & 0x00FF0000) >> 16);
                    result.key_res_y[i - 2] = ((pointQ256.y.num[j] & 0x0000FF00) >> 8);
                    result.key_res_y[i - 3] = (pointQ256.y.num[j] & 0x000000FF);
                    i -= 4;
                    j++;
#endif // (ECC_MULT_ALGO_TYPE == 16)
                }
            }
            // Inform client about computation result
            ecc_elt->cb_result(ecc_elt->metainfo, &result);
            ke_free(ecc_elt);

            DBG_SWDIAG(ECDH, END, 0);
        }
    }

    // Stop background computation if all multiplication performed
    if (co_list_is_empty(&ecc_env.ongoing_mul))
    {
        ke_event_clear(KE_EVENT_ECC_P256_MULTIPLICATION);
        DBG_SWDIAG(ECDH, BUSY, 0);
        led_reset(4);
    }
    DBG_SWDIAG(ECDH, COMPUTE, 0);
}


/**
 ****************************************************************************************
 * @brief Generate a new DHKey using ECC P256 algorithm
 *
 * @param[in] key_type          Type of key to generate (1: public key | 2: DH key)
 * @param[in] secret_key        Private key                  - MSB First
 * @param[in] public_key_x      Peer public key x coordinate - LSB First
 * @param[in] public_key_y      Peer public key y coordinate - LSB First
 * @param[in] public_key_valid  True: public key is valid, False: public key uncertain
 * @param[in] metainfo          Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 * @param[in] cb_result         Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
__STATIC uint8_t ecc_gen_key256(uint8_t key_type, const uint8_t *secret_key,
                                const uint8_t *public_key_x, const uint8_t *public_key_y, bool public_key_valid,
                                uint32_t metainfo, ecc_p256_result_cb cb_result)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    if (cb_result != NULL)
    {
        uint32_t big_num_offset = 1;
        int32_t i, j;
        bigHex256 PrivateKey256;
        ECC_Point256 PublicKey256;

        // Now Copy the Public Key and Secret Key coordinates to ECC point format.
        PrivateKey256.num[0] = 0;
        PublicKey256.x.num[0] = 0;
        PublicKey256.y.num[0] = 0;
        PrivateKey256.sign = 0;

        DBG_SWDIAG(ECDH, BUSY, 1);

        for (i = 31, j = big_num_offset; i >= 0;) // Public and Private Keys Are LSB - make it in MSB
        {
#if (ECC_MULT_ALGO_TYPE == 16)
            PrivateKey256.num[j] = (uint32_t)
                                   ((((*(secret_key + i)) <<  8) & 0xFF00) +
                                    (((*(secret_key + (i - 1)))      & 0x00FF)));

            PublicKey256.x.num[j] = (uint32_t)
                                    ((((*(public_key_x + i)) <<  8) & 0xFF00) +
                                     (((*(public_key_x + (i - 1)))      & 0x00FF)));

            PublicKey256.y.num[j] = (uint32_t)
                                    ((((*(public_key_y + i)) <<  8) & 0xFF00) +
                                     (((*(public_key_y + (i - 1)))      & 0x00FF)));

            i -= 2;
            j++;
#elif (ECC_MULT_ALGO_TYPE == 32)
            PrivateKey256.num[j] = (uint32_t)
                                   ((((*(secret_key + i)) << 24) & 0xFF000000) +
                                    (((*(secret_key + (i - 1))) << 16) & 0x00FF0000) +
                                    (((*(secret_key + (i - 2))) <<  8) & 0x0000FF00) +
                                    ((*(secret_key + (i - 3)))        & 0x000000FF));

            PublicKey256.x.num[j] = (uint32_t)
                                    ((((*(public_key_x + i)) << 24) & 0xFF000000) +
                                     (((*(public_key_x + (i - 1))) << 16) & 0x00FF0000) +
                                     (((*(public_key_x + (i - 2))) <<  8) & 0x0000FF00) +
                                     ((*(public_key_x + (i - 3)))        & 0x000000FF));

            PublicKey256.y.num[j] = (uint32_t)
                                    ((((*(public_key_y + i)) << 24) & 0xFF000000) +
                                     (((*(public_key_y + (i - 1))) << 16) & 0x00FF0000) +
                                     (((*(public_key_y + (i - 2))) <<  8) & 0x0000FF00) +
                                     ((*(public_key_y + (i - 3)))        & 0x000000FF));
            i -= 4;
            j++;
#endif // (ECC_MULT_ALGO_TYPE == 16)
        }

        setBigNumberLength256(&PrivateKey256);
        setBigNumberLength256(&PublicKey256.x);
        setBigNumberLength256(&PublicKey256.y);
        PublicKey256.x.sign = 0;
        PublicKey256.y.sign = 0;


        if (public_key_valid || ecc_is_valid_point(&PublicKey256.x, &PublicKey256.y))
        {
            // Allocate Memory for Jacobian Points P, Q, R
            struct ecc_elt_tag *ecc_elt = (struct ecc_elt_tag *) ke_malloc_system(sizeof(struct ecc_elt_tag), KE_MEM_NON_RETENTION);
            ecc_elt->win_4_table = NULL;

#ifdef CFG_ECC_SIM_ACCEL
            if (ecc_env.use_simu_accelerator)
            {
                ecc_elt->Point_Mul_Word256 = 0; // do not perform any multiplication
                ecc_priv_ptr_setf((uint32_t)secret_key);
                ecc_pub_x_ptr_setf((uint32_t)public_key_x);
                ecc_pub_y_ptr_setf((uint32_t)public_key_y);
                DBG_MEM_INIT(&(ecc_elt->result), sizeof(ecc_p256_result_t));
                ecc_result_setf((uint32_t) & (ecc_elt->result));
                ecc_start_setf(1);
            }
            else
#endif // CFG_ECC_SIM_ACCEL
            {
                ECC_Jacobian_Point256 PointP_Jacobian;
                ECC_Jacobian_Point256 *pPointP_Jacobian = &PointP_Jacobian;

                ecc_elt->key_type    = key_type;

                // Need to map from Affine Point to Jacobian Point
                GF_Affine_To_Jacobian_Point_Copy256(&PublicKey256, pPointP_Jacobian);

                GF_Jacobian_Point_Copy256(pPointP_Jacobian, &(ecc_elt->Jacobian_PointR256));

                copyBigHex256(&PrivateKey256, &(ecc_elt->Pk256));

                GF_Setup_Jacobian_Infinity_Point256(&(ecc_elt->Jacobian_PointQ256));

                // Initialize cursor
                ecc_elt->bit_cursor  = 63; // 64 interations of the 4 Bit Window
                ecc_elt->current_val = 0;
                ecc_elt->Point_Mul_Word256 = ELEMENTS_BIG_HEX256;

                if (ecc_elt->key_type == ECC_DHKEY_GENERATION)
                {
                    // First determine the table
                    /// Allocate memory for the ECC Key Generation Table
                    ecc_elt->win_4_table  = (ECC_Jacobian_Point256 *) ke_malloc_system((sizeof(ECC_Jacobian_Point256) * 15), KE_MEM_NON_RETENTION);
                    /// First step is to generate the 16 entry table to be used for Key Calculation
                    /// -- First part of this is to determine index table values 0x0000,0x0001,0x0010,0x0100,0x1000

                    GF_Jacobian_Point_Copy256(&ecc_elt->Jacobian_PointR256, &ecc_elt->win_4_table[0]);

                    ecc_elt->key_type = ECC_4BIT_TABLE_GENERATION_PT1;
                    ecc_elt->bit_cursor = 0;
                }
            }

            // Store requester information
            ecc_elt->metainfo  = metainfo;
            ecc_elt->cb_result = cb_result;

            // Insert the multiplication at the end of the list
            co_list_push_back(&ecc_env.ongoing_mul, &ecc_elt->hdr);
            // Start the event
            ke_event_set(KE_EVENT_ECC_P256_MULTIPLICATION);

            status = CO_ERROR_NO_ERROR;
        }
    }

    return status;
}

/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void ecc_p256_init(bool is_reset)
{
    if (!is_reset)
    {
        // Register event to handle multiplication steps
        ke_event_callback_set(KE_EVENT_ECC_P256_MULTIPLICATION, &ecc_multiplication_event_handler);
    }
    else
    {
        // Clear the kernel event if pending
        ke_event_clear(KE_EVENT_ECC_P256_MULTIPLICATION);

        // Empty multiplications list
        while (!co_list_is_empty(&ecc_env.ongoing_mul))
        {
            // Free allocated memory
            struct ecc_elt_tag *elt = (struct ecc_elt_tag *) co_list_pop_front(&ecc_env.ongoing_mul);

            // Free the memory previously allocated for Jacobian points Q and R and the private key
            if (elt->win_4_table != NULL)
            {
                ke_free(elt->win_4_table);
            }

            ke_free(elt);
        }
        led_reset(4);
    }
    // No break

    // Initialize multiplications list
    co_list_init(&ecc_env.ongoing_mul);
#ifdef CFG_ECC_SIM_ACCEL
    ecc_env.use_simu_accelerator = true;
#endif // CFG_ECC_SIM_ACCEL
}

void ecc_p256_abort_key_gen(uint16_t metainfo)
{
    struct ecc_elt_tag *prev = NULL;
    struct ecc_elt_tag *elt = (struct ecc_elt_tag *) co_list_pick(&ecc_env.ongoing_mul);

    // Empty multiplications list
    while (elt != NULL)
    {
        // Check if the element correspond to the abort request
        if (elt->metainfo == metainfo)
        {
            // Extract element from the list
            co_list_extract_after(&ecc_env.ongoing_mul, &prev->hdr, &elt->hdr);

            // if DH_Key Generation - ensure any memory assigned for Table is freed
            if (elt->win_4_table != NULL)
            {
                ke_free(elt->win_4_table);
            }

            // Free allocated memory
            ke_free(elt);

            // Check if list is empty
            if (co_list_is_empty(&ecc_env.ongoing_mul))
            {
                // Clear the event
                ke_event_clear(KE_EVENT_ECC_P256_MULTIPLICATION);
            }

            break;
        }

        // Jump to next
        prev = elt;
        elt = (struct ecc_elt_tag *) co_list_next(&elt->hdr);
    }
}

uint8_t ecc_p256_gen_new_public_key(uint8_t *secret_key, uint32_t metainfo, ecc_p256_result_cb cb_result)
{
    return ecc_gen_key256(ECC_PUBLICKEY_GENERATION, secret_key, BasePoint_x_256, BasePoint_y_256, true, metainfo, cb_result);
}

uint8_t ecc_p256_gen_dh_key(const uint8_t *secret_key, const uint8_t *public_key_x, const uint8_t *public_key_y,
                            uint32_t metainfo, ecc_p256_result_cb cb_result)
{
    return ecc_gen_key256(ECC_DHKEY_GENERATION, secret_key, public_key_x, public_key_y, false, metainfo, cb_result);
}


void ecc_p256_gen_new_secret_key(uint8_t *secret_key256)
{
    // Need to get a 32 Byte random number - which we will use as the secret key.
    // We then ECC multiply this by the Base Points, to get a new Public Key.
    //
    do
    {
        co_write32p(&secret_key256[0],  co_rand_word());
        co_write32p(&secret_key256[4],  co_rand_word());
        co_write32p(&secret_key256[8],  co_rand_word());
        co_write32p(&secret_key256[12], co_rand_word());
        co_write32p(&secret_key256[16], co_rand_word());
        co_write32p(&secret_key256[20], co_rand_word());
        co_write32p(&secret_key256[24], co_rand_word());
        co_write32p(&secret_key256[28], co_rand_word());
    }
    while (!isValidSecretKey_256(secret_key256));
}

uint8_t ecc_p256_gen_key_pair(bool debug_key, uint8_t *secret_key, uint32_t metainfo, ecc_p256_result_cb cb_result)
{
    if (debug_key)
    {
        ecc_p256_result_t result;

        memcpy(secret_key,       DebugE256SecretKey,   ECC_256_KEY_SIZE);
        memcpy(result.key_res_x, DebugE256PublicKey_x, ECC_256_KEY_SIZE);
        memcpy(result.key_res_y, DebugE256PublicKey_y, ECC_256_KEY_SIZE);

        // Call immediately result function
        cb_result(metainfo, &result);
    }
    else
    {
        ecc_p256_gen_new_secret_key(secret_key);
        ecc_p256_gen_new_public_key(secret_key, metainfo, cb_result);
    }

    return CO_ERROR_NO_ERROR;
}


const uint8_t *ecc_p256_get_debug_priv_key(void)
{
    return &(DebugE256SecretKey[0]);
}

const uint8_t *ecc_p256_get_debug_pub_key_x(void)
{
    return &(DebugE256PublicKey_x[0]);
}

const uint8_t *ecc_p256_get_debug_pub_key_y(void)
{
    return &(DebugE256PublicKey_y[0]);
}
