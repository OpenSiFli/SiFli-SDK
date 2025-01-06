/**
****************************************************************************************
*
* @file bt_util_key.c
*
* @brief link manager key management
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup BTUTILKEY
* @{
****************************************************************************************
*/


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "co_math.h"
#include "co_endian.h"           // endian definition
#include "bt_util_key.h"              // link manager keys

/*
 * DEFINITIONS & MACROS
 ****************************************************************************************
 */

#define LM_KEY_SIZE          16
#define LM_SRES_SIZE         4
#define LM_ACO_SIZE          12

#define LM_NB_ROUND          8
#define LM_EXP_LOG_TAB_LEN   256
#define LM_MAX_KEY_SIZE      16

#define ROL(x, n)            ((uint8_t)((uint32_t)(x) << (n) | (uint32_t)((x) & 0xFF) >> (8 - (n))))
#define EXP(x)               LM_ExpTab[(x) & 0xFF]
#define LOG(x)               LM_LogTab[(x) & 0xFF]


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Table of (45^i % 257) % 256. see Bluetooth Standard (Baseband)
__STATIC uint8_t const LM_ExpTab[LM_EXP_LOG_TAB_LEN] =
{
    0x01, 0x2D, 0xE2, 0x93, 0xBE, 0x45, 0x15, 0xAE,
    0x78, 0x03, 0x87, 0xA4, 0xB8, 0x38, 0xCF, 0x3F,
    0x08, 0x67, 0x09, 0x94, 0xEB, 0x26, 0xA8, 0x6B,
    0xBD, 0x18, 0x34, 0x1B, 0xBB, 0xBF, 0x72, 0xF7,
    0x40, 0x35, 0x48, 0x9C, 0x51, 0x2F, 0x3B, 0x55,
    0xE3, 0xC0, 0x9F, 0xD8, 0xD3, 0xF3, 0x8D, 0xB1,
    0xFF, 0xA7, 0x3E, 0xDC, 0x86, 0x77, 0xD7, 0xA6,
    0x11, 0xFB, 0xF4, 0xBA, 0x92, 0x91, 0x64, 0x83,
    0xF1, 0x33, 0xEF, 0xDA, 0x2C, 0xB5, 0xB2, 0x2B,
    0x88, 0xD1, 0x99, 0xCB, 0x8C, 0x84, 0x1D, 0x14,
    0x81, 0x97, 0x71, 0xCA, 0x5F, 0xA3, 0x8B, 0x57,
    0x3C, 0x82, 0xC4, 0x52, 0x5C, 0x1C, 0xE8, 0xA0,
    0x04, 0xB4, 0x85, 0x4A, 0xF6, 0x13, 0x54, 0xB6,
    0xDF, 0x0C, 0x1A, 0x8E, 0xDE, 0xE0, 0x39, 0xFC,
    0x20, 0x9B, 0x24, 0x4E, 0xA9, 0x98, 0x9E, 0xAB,
    0xF2, 0x60, 0xD0, 0x6C, 0xEA, 0xFA, 0xC7, 0xD9,
    0x00, 0xD4, 0x1F, 0x6E, 0x43, 0xBC, 0xEC, 0x53,
    0x89, 0xFE, 0x7A, 0x5D, 0x49, 0xC9, 0x32, 0xC2,
    0xF9, 0x9A, 0xF8, 0x6D, 0x16, 0xDB, 0x59, 0x96,
    0x44, 0xE9, 0xCD, 0xE6, 0x46, 0x42, 0x8F, 0x0A,
    0xC1, 0xCC, 0xB9, 0x65, 0xB0, 0xD2, 0xC6, 0xAC,
    0x1E, 0x41, 0x62, 0x29, 0x2E, 0x0E, 0x74, 0x50,
    0x02, 0x5A, 0xC3, 0x25, 0x7B, 0x8A, 0x2A, 0x5B,
    0xF0, 0x06, 0x0D, 0x47, 0x6F, 0x70, 0x9D, 0x7E,
    0x10, 0xCE, 0x12, 0x27, 0xD5, 0x4C, 0x4F, 0xD6,
    0x79, 0x30, 0x68, 0x36, 0x75, 0x7D, 0xE4, 0xED,
    0x80, 0x6A, 0x90, 0x37, 0xA2, 0x5E, 0x76, 0xAA,
    0xC5, 0x7F, 0x3D, 0xAF, 0xA5, 0xE5, 0x19, 0x61,
    0xFD, 0x4D, 0x7C, 0xB7, 0x0B, 0xEE, 0xAD, 0x4B,
    0x22, 0xF5, 0xE7, 0x73, 0x23, 0x21, 0xC8, 0x05,
    0xE1, 0x66, 0xDD, 0xB3, 0x58, 0x69, 0x63, 0x56,
    0x0F, 0xA1, 0x31, 0x95, 0x17, 0x07, 0x3A, 0x28
};

/// Table of inverse (45^i % 257) % 256.
__STATIC uint8_t const LM_LogTab[LM_EXP_LOG_TAB_LEN] =
{
    0x80, 0x00, 0xB0, 0x09, 0x60, 0xEF, 0xB9, 0xFD,
    0x10, 0x12, 0x9F, 0xE4, 0x69, 0xBA, 0xAD, 0xF8,
    0xC0, 0x38, 0xC2, 0x65, 0x4F, 0x06, 0x94, 0xFC,
    0x19, 0xDE, 0x6A, 0x1B, 0x5D, 0x4E, 0xA8, 0x82,
    0x70, 0xED, 0xE8, 0xEC, 0x72, 0xB3, 0x15, 0xC3,
    0xFF, 0xAB, 0xB6, 0x47, 0x44, 0x01, 0xAC, 0x25,
    0xC9, 0xFA, 0x8E, 0x41, 0x1A, 0x21, 0xCB, 0xD3,
    0x0D, 0x6E, 0xFE, 0x26, 0x58, 0xDA, 0x32, 0x0F,
    0x20, 0xA9, 0x9D, 0x84, 0x98, 0x05, 0x9C, 0xBB,
    0x22, 0x8C, 0x63, 0xE7, 0xC5, 0xE1, 0x73, 0xC6,
    0xAF, 0x24, 0x5B, 0x87, 0x66, 0x27, 0xF7, 0x57,
    0xF4, 0x96, 0xB1, 0xB7, 0x5C, 0x8B, 0xD5, 0x54,
    0x79, 0xDF, 0xAA, 0xF6, 0x3E, 0xA3, 0xF1, 0x11,
    0xCA, 0xF5, 0xD1, 0x17, 0x7B, 0x93, 0x83, 0xBC,
    0xBD, 0x52, 0x1E, 0xEB, 0xAE, 0xCC, 0xD6, 0x35,
    0x08, 0xC8, 0x8A, 0xB4, 0xE2, 0xCD, 0xBF, 0xD9,
    0xD0, 0x50, 0x59, 0x3F, 0x4D, 0x62, 0x34, 0x0A,
    0x48, 0x88, 0xB5, 0x56, 0x4C, 0x2E, 0x6B, 0x9E,
    0xD2, 0x3D, 0x3C, 0x03, 0x13, 0xFB, 0x97, 0x51,
    0x75, 0x4A, 0x91, 0x71, 0x23, 0xBE, 0x76, 0x2A,
    0x5F, 0xF9, 0xD4, 0x55, 0x0B, 0xDC, 0x37, 0x31,
    0x16, 0x74, 0xD7, 0x77, 0xA7, 0xE6, 0x07, 0xDB,
    0xA4, 0x2F, 0x46, 0xF3, 0x61, 0x45, 0x67, 0xE3,
    0x0C, 0xA2, 0x3B, 0x1C, 0x85, 0x18, 0x04, 0x1D,
    0x29, 0xA0, 0x8F, 0xB2, 0x5A, 0xD8, 0xA6, 0x7E,
    0xEE, 0x8D, 0x53, 0x4B, 0xA1, 0x9A, 0xC1, 0x0E,
    0x7A, 0x49, 0xA5, 0x2C, 0x81, 0xC4, 0xC7, 0x36,
    0x2B, 0x7F, 0x43, 0x95, 0x33, 0xF2, 0x6C, 0x68,
    0x6D, 0xF0, 0x02, 0x28, 0xCE, 0xDD, 0x9B, 0xEA,
    0x5E, 0x99, 0x7C, 0x14, 0x86, 0xCF, 0xE5, 0x42,
    0xB8, 0x40, 0x78, 0x2D, 0x3A, 0xE9, 0x64, 0x1F,
    0x92, 0x90, 0x7D, 0x39, 0x6F, 0xE0, 0x89, 0x30
};

/// Permutation Vector
__STATIC uint8_t const LM_Permutation[LM_KEY_SIZE] =
{
    8, 11, 12, 15, 2, 1, 6, 5, 10, 9, 14, 13, 0, 7, 4, 3
};

/// g1 Polynomial Degree
__STATIC uint8_t const LM_G1Degree[LM_MAX_KEY_SIZE] =
{
    8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128
};

/// g1 Polynomial table
__STATIC uint8_t const LM_G1Poly[LM_MAX_KEY_SIZE][LM_KEY_SIZE] =
{
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1D},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x3F},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xDB},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xAF},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x39},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x91},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x09},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x15},
    {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3B},
    {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD},
    {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x9D},
    {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x4F},
    {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

/// g2 Polynomial Degree
__STATIC uint8_t const LM_G2Degree[LM_MAX_KEY_SIZE] =
{
    119, 112, 104, 96, 88, 77, 71, 63, 49, 42, 35, 28, 21, 14, 7, 0
};

/// g2 Polynomial Table
__STATIC uint8_t const LM_G2Poly[LM_MAX_KEY_SIZE][LM_KEY_SIZE] =
{
    {0x00, 0xE2, 0x75, 0xA0, 0xAB, 0xD2, 0x18, 0xD4, 0xCF, 0x92, 0x8B, 0x9B, 0xBF, 0x6C, 0xB0, 0x8F},
    {0x00, 0x01, 0xE3, 0xF6, 0x3D, 0x76, 0x59, 0xB3, 0x7F, 0x18, 0xC2, 0x58, 0xCF, 0xF6, 0xEF, 0xEF},
    {0x00, 0x00, 0x01, 0xBE, 0xF6, 0x6C, 0x6C, 0x3A, 0xB1, 0x03, 0x0A, 0x5A, 0x19, 0x19, 0x80, 0x8B},
    {0x00, 0x00, 0x00, 0x01, 0x6A, 0xB8, 0x99, 0x69, 0xDE, 0x17, 0x46, 0x7F, 0xD3, 0x73, 0x6A, 0xD9},
    {0x00, 0x00, 0x00, 0x00, 0x01, 0x63, 0x06, 0x32, 0x91, 0xDA, 0x50, 0xEC, 0x55, 0x71, 0x52, 0x47},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x93, 0x52, 0xAA, 0x6C, 0xC0, 0x54, 0x46, 0x83, 0x11},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB3, 0xF7, 0xFF, 0xFC, 0xE2, 0x79, 0xF3, 0xA0, 0x73},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0xAB, 0x81, 0x5B, 0xC7, 0xEC, 0x80, 0x25},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xC9, 0x80, 0x11, 0xD8, 0xB0, 0x4D},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x8E, 0x24, 0xF9, 0xA4, 0xBB},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xA7, 0x60, 0x24, 0xD7},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x9C, 0x26, 0xB9},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xD9, 0xE3},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x77},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x89},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
};

/*
 * INTERNAL FUNCTION PROTOTYPES
 ****************************************************************************************
 */
__STATIC void LM_E1(uint8_t *Key, uint8_t *BdAddr, uint8_t *Random, uint8_t *Sres, uint8_t *Aco);
__STATIC void LM_E21(uint8_t *Random, uint8_t *BdAddr, uint8_t *Result);
__STATIC void LM_E22(uint8_t *Random, uint8_t *BdAddr, uint8_t *Pin, uint8_t PinLen, uint8_t *Result);
__STATIC void LM_E3(uint8_t *Key, uint8_t *Cof, uint8_t *Random, uint8_t *Result);
__STATIC void LM_KPrimC(uint8_t *Kc, uint32_t length, uint8_t *Result);
__STATIC void LM_Xor(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut);
__STATIC void LM_Ar(uint8_t *Random, uint8_t *Key, uint8_t *Result);
__STATIC void LM_ExpandAddr(uint8_t *BdAddr, uint8_t *Result);
__STATIC uint32_t LM_ExpandPIN(uint8_t *Pin, uint8_t PinLen, uint8_t *BdAddr, uint8_t *Result);
__STATIC void LM_ExpandCof(uint8_t *BdAddr, uint8_t *Result);
__STATIC void LM_Offset(uint8_t *Key, uint8_t *OffsetK);
__STATIC void LM_ArPrim(uint8_t *Vector, uint8_t *Key, uint8_t *Result);
__STATIC void LM_ArRound(uint8_t *Vector, uint8_t *Key, uint8_t *KeyNumber);
__STATIC void LM_CreateBiasVector(uint8_t *Vector, uint32_t Number);
__STATIC void LM_ExpLog(uint8_t *Vector);
__STATIC void LM_InitKeySchedule(uint8_t *Key, uint8_t *KeySchedule);
__STATIC void LM_NextKeySchedule(uint8_t *Key, uint8_t *KeySchedule, uint8_t *KeyNumber);
__STATIC void LM_Add(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut);
__STATIC void LM_XorAdd(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut);
__STATIC void LM_AddXor(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut);
__STATIC void LM_Permute(uint8_t *Vector);
__STATIC void LM_Modulo(uint8_t *Dividend, uint8_t *Divisor, uint32_t DivisorDegree, uint8_t *Rest);
__STATIC void LM_Multiply(uint8_t *Polynom1, uint8_t *Polynom2, uint32_t Poly2Degree, uint8_t *MultPoly);
__STATIC void LM_ReverseVector(uint8_t *Vector);

/*
 * INTERNAL FUNCTION DEFINITION
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Processes E1 algorithm for Authentication.
 *
 * @param[in]  Key       Key Vector[16]
 * @param[in]  BdAddr    BdAddr[6]
 * @param[in]  Random    Random Vector[16]
 * @param[out] Result    Result Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_E1(uint8_t *Key, uint8_t *BdAddr, uint8_t *Random, uint8_t *Sres, uint8_t *Aco)
{
    uint8_t  TmpRes1[LM_KEY_SIZE];               /* Temporary Vector                         */
    uint8_t  TmpRes2[LM_KEY_SIZE];               /* Temporary Vector                         */
    uint8_t  OffsetK[LM_KEY_SIZE];               /* Vector for Offset Key                    */
    uint8_t  Index;

    /* Run Ar Algorithm                                                                 */
    LM_Ar(Random, Key, TmpRes1);

    /* XOR of Random Vector and of Ar Result Vector                                     */
    LM_Xor(Random, TmpRes1, TmpRes1);

    /* Expand BdAddr (from 6 bytes to 16 bytes)                                         */
    LM_ExpandAddr(BdAddr, TmpRes2);

    /* Add expanded BdAddr and xored Vector                                             */
    LM_Add(TmpRes1, TmpRes2, TmpRes2);

    /* Determine Offset Key Vector                                                      */
    LM_Offset(Key, OffsetK);

    /* Run A'r Algorithm for ADDed Vector and Offset Key                                */
    LM_ArPrim(TmpRes2, OffsetK, TmpRes1);

    /* Copy the first 4 bytes of TmpRes1 to Sres                                        */
    for (Index = 0; Index < LM_SRES_SIZE ; Index++)
    {
        Sres[Index] = TmpRes1[Index];
    }

    /* Copy the last 12 bytes of TmpRes1 to Aco                                         */
    for (Index = 0; Index < LM_ACO_SIZE ; Index++)
    {
        Aco[Index] = TmpRes1[Index + LM_SRES_SIZE];
    }
}

/*
 ****************************************************************************************
 * @brief Processes E21 algorithm for Authentication.
 *
 * @param[in]  Random    Random Vector[16]
 * @param[in]  BdAddr    BdAddr[6]
 * @param[out] Result    Result Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_E21(uint8_t *Random, uint8_t *BdAddr, uint8_t *Result)
{
    uint8_t  TmpRand[LM_KEY_SIZE];            /* Temporary Vector                            */
    uint8_t  TmpAddr[LM_KEY_SIZE];            /* Temporary Vector                            */
    uint32_t Index;

    /* First Copy Random Input Vector to temporary Vector                               */
    for (Index = 0 ; Index < (LM_KEY_SIZE - 1) ; Index++)
    {
        TmpRand[Index] = Random[Index];
    }
    TmpRand[Index] = Random[Index] ^ 6;       /* XOR last byte of Random Vector with 6  */

    /* Expand BdAddr (from 6 bytes to 16 bytes)                                         */
    LM_ExpandAddr(BdAddr, TmpAddr);

    /* Run A'r Algorithm for BdAddr Vector and for random Vector                        */
    LM_ArPrim(TmpAddr, TmpRand, Result);
}

/*
 ****************************************************************************************
 * @brief Processes E21 algorithm for Authentication.
 *
 * @param[in]  Random    Random Vector[16]
 * @param[in]  BdAddr    BdAddr[6]
 * @param[in]  Pin       PIN Vector[1..16]
 * @param[in]  PinLen    PIN length (1..16)
 * @param[out] Result    Result Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_E22(uint8_t *Random, uint8_t *BdAddr, uint8_t *Pin, uint8_t PinLen, uint8_t *Result)
{
    uint8_t  PinPrim[LM_KEY_SIZE];            /* Temporary Vector                            */
    uint8_t  TmpRand[LM_KEY_SIZE];            /* Temporary Vector                            */
    uint32_t Index;
    uint32_t lenPrim;

    /* Expand PIN with BdAddr                                                           */
    lenPrim = LM_ExpandPIN(Pin, PinLen, BdAddr, PinPrim);

    /* Copy Random Input Vector to temporary Vector                                     */
    for (Index = 0 ; Index < (LM_KEY_SIZE - 1) ; Index++)
    {
        TmpRand[Index] = Random[Index];
    }

    /* XOR last byte of Random vec with lenPri                                          */
    TmpRand[Index] = Random[Index] ^ (uint8_t)lenPrim;

    /* Run A'r Algorithm for Random Vector and for BdAddr Vector                        */
    LM_ArPrim(TmpRand, PinPrim, Result);
}


/*
 ****************************************************************************************
 * @brief Processes E3 algorithm for Authentication.
 *
 * @param[in]  Key       Key Vector[16]
 * @param[in]  Cof       COF[12]
 * @param[in]  Random    Random Vector[16]
 * @param[out] Result    Result Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_E3(uint8_t *Key, uint8_t *Cof, uint8_t *Random, uint8_t *Result)
{
    uint8_t  TmpRes1[LM_KEY_SIZE];            /* Tempo Vector                                */
    uint8_t  TmpRes2[LM_KEY_SIZE];            /* Tempo Vector                                */
    uint8_t  OffsetK[LM_KEY_SIZE];            /* Vector for Offset Key                       */

    /* Run Ar Algorithm                                                                 */
    LM_Ar(Random, Key, TmpRes1);

    /* XOR Random Vector and Ar Result Vector                                           */
    LM_Xor(Random, TmpRes1, TmpRes1);

    /* Expand Cof (from 12 bytes to 16 bytes]                                           */
    LM_ExpandCof(Cof, TmpRes2);

    /* Add Expanded BdAddr and xored Vector                                             */
    LM_Add(TmpRes1, TmpRes2, TmpRes2);

    /* Determine Offset Key Vector                                                      */
    LM_Offset(Key, OffsetK);

    /* Run A'r Algorithm on added Vector and Offset Key                                 */
    LM_ArPrim(TmpRes2, OffsetK, Result);

}

/*
 ****************************************************************************************
 * @brief Processes Ar algorithm for Authentication.
 *
 * @param[in]  Random    Random Vector[16]
 * @param[in]  Key       Key Vector[16]
 * @param[out] Result    Result Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Ar(uint8_t *Random, uint8_t *Key, uint8_t *Result)
{
    uint8_t  KeySchedule[LM_KEY_SIZE + 1];
    uint8_t  KeyScheduleLast[LM_KEY_SIZE];
    int Index;
    uint8_t  KeyNumber = 0;

    /* First copy Random Input Vector in Result Vector                                  */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Result[Index] = Random[Index];
    }

    /* Init Key Schedule process                                                        */
    LM_InitKeySchedule(Key, KeySchedule);

    /* Ar Algorithm consists of 8 iterations of round                                   */
    for (Index = 0 ; Index < LM_NB_ROUND ; Index++)
    {
        LM_ArRound(Result, KeySchedule, &KeyNumber);                /* one round        */
    }

    /* Get last Key Schedule                                                            */
    LM_NextKeySchedule(KeySchedule, KeyScheduleLast, &KeyNumber);

    /* Xor Add Add Xor 4 times                                                          */
    LM_XorAdd(Result, KeyScheduleLast, Result);
}


/*
 ****************************************************************************************
 * @brief Processes A'r algorithm for Authentication.
 *
 * @param[in]  Random    Random Vector[16]
 * @param[in]  Key       Key Vector[16]
 * @param[out] Result    Result Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_ArPrim(uint8_t *Random, uint8_t *Key, uint8_t *Result)
{
    uint8_t  KeySchedule[LM_KEY_SIZE + 1];
    uint8_t  saveVector[LM_KEY_SIZE + 1];
    uint8_t  KeyScheduleLast[LM_KEY_SIZE];
    uint32_t Index;
    uint8_t  KeyNumber = 0;

    /* Copy Random Input Vector to Result Vector and store it for last loop management  */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Result[Index] = Random[Index];
        saveVector[Index] = Random[Index];
    }

    /* Init Key Schedule process                                                        */
    LM_InitKeySchedule(Key, KeySchedule);

    /* A'r Algorithm consists of 8 iterations of round                                  */
    for (Index = 0 ; Index < LM_NB_ROUND ; Index++)
    {
        /* Xor Add Add Xor 4 times before iteration 3                                   */
        if (Index == 2)
        {
            LM_XorAdd(Result, saveVector, Result);
        }

        LM_ArRound(Result, KeySchedule, &KeyNumber);
    }

    /* Get Last Key Schedule                                                            */
    LM_NextKeySchedule(KeySchedule, KeyScheduleLast, &KeyNumber);

    /* Xor Add Add Xor 4 times                                                          */
    LM_XorAdd(Result, KeyScheduleLast, Result);
}

/*
 ****************************************************************************************
 * @brief Processes one round of the Ar algorithm for Authentication.
 *
 * @param[in/out] Vector          Vector[16]
 * @param[in/out] Key             Key Vector[16]
 * @param[in]     KeyNumber       Key Number
 *
 ****************************************************************************************
 */
__STATIC void LM_ArRound(uint8_t *Vector, uint8_t *Key, uint8_t *KeyNumber)
{
    uint8_t KeySchedule[LM_KEY_SIZE];
    uint32_t Index;
    uint32_t loop;
    uint8_t Tmp1;
    uint8_t Tmp2;

    /* Get Next Key Schedule                                                            */
    LM_NextKeySchedule(Key, KeySchedule, KeyNumber);

    /* Xor Add Add Xor 4 times                                                          */
    LM_XorAdd(Vector, KeySchedule, Vector);

    /* Exp Log Log Exp 4 times                                                          */
    LM_ExpLog(Vector);

    /* Get Next Key Schedule                                                            */
    LM_NextKeySchedule(Key, KeySchedule, KeyNumber);

    /* Add Xor Xor Add 4 times                                                          */
    LM_AddXor(Vector, KeySchedule, Vector);

    /* A Round consists of 3 PHT and permutations                                       */
    for (loop = 0 ; loop < 3 ; loop++)
    {
        /* PHT computation                                                              */
        for (Index = 0 ; Index < LM_KEY_SIZE ;)
        {
            Tmp1 = Vector[Index];
            Tmp2 = Vector[Index + 1];
            Vector[Index++] = 2 * Tmp1 + Tmp2;
            Vector[Index++] = Tmp1 + Tmp2;
        }
        /* Permute Vector                                                               */
        LM_Permute(Vector);
    }

    /* Last PHT computation                                                             */
    for (Index = 0 ; Index < LM_KEY_SIZE ;)
    {
        Tmp1 = Vector[Index];
        Tmp2 = Vector[Index + 1];
        Vector[Index++] = 2 * Tmp1 + Tmp2;
        Vector[Index++] = Tmp1 + Tmp2;
    }
}

/*
 ****************************************************************************************
 * @brief Expands a 6 bytes Vector to a 16 bytes Vector.
 *
 * @param[in]  BdAddr    Address Vector[6]
 * @param[out] Result    Address Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_ExpandAddr(uint8_t *BdAddr, uint8_t *Result)
{
    uint32_t IndexSrc;
    uint32_t IndexDst;

    /* Fill Result Vector with copies of Address Vector                                 */
    for (IndexSrc = 0, IndexDst = 0 ; IndexDst < LM_KEY_SIZE ; IndexDst++)
    {
        Result[IndexDst] = BdAddr[IndexSrc++];
        /* Perform modulo 6 on address Vector                                           */
        if (IndexSrc == 6)
        {
            IndexSrc = 0;
        }
    }
}

/*
 ****************************************************************************************
 * @brief Expand PIN for E22 computing.
 *
 * @param[in]  Pin       PIN Vector[1..16]
 * @param[in]  PinLen    PIN length (1..16)
 * @param[in]  BdAddr    BdAddr[6]
 * @param[out] Result    Result Vector[16]
 *
 * @return LengthPrim
 *
 ****************************************************************************************
 */
__STATIC uint32_t LM_ExpandPIN(uint8_t *Pin, uint8_t PinLen, uint8_t *BdAddr, uint8_t *Result)
{
    uint32_t Index;
    uint32_t Index2;
    uint32_t lenPrim;

    /* copy PIN Vector to Result Vector                                                 */
    for (Index = 0 ; Index < PinLen ; Index++)
    {
        Result[Index] = Pin[Index];
    }

    /* concatenate DB_ADDR at the end of the Result Vector                              */
    Index2 = 0;
    while ((Index < LM_KEY_SIZE) && (Index2 < 6))
    {
        Result[Index++] = BdAddr[Index2++];
    }

    lenPrim = Index;                        /* Store lenPrim                            */

    /* concatenate Result Vector to itself                                              */
    Index2 = 0;
    while (Index < LM_KEY_SIZE)
    {
        Result[Index++] = Result[Index2++];

        if (Index2 == lenPrim)
        {
            Index2 = 0;
        }
    }
    return lenPrim;
}

/*
 ****************************************************************************************
 * @brief Expands a 12 bytes Vector to a 16 bytes Vector.
 *
 * @param[in]  Cof       Address Vector[12]
 * @param[out] Result    Address Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_ExpandCof(uint8_t *BdAddr, uint8_t *Result)
{
    uint32_t IndexSrc;
    uint32_t IndexDst;

    /* Fill Result Vector with copies of Address Vector                                 */
    for (IndexSrc = 0, IndexDst = 0 ; IndexDst < LM_KEY_SIZE ; IndexDst++)
    {
        Result[IndexDst] = BdAddr[IndexSrc++];
        /* Perform modulo 12 on address Vector                                          */
        if (IndexSrc == 12)
        {
            IndexSrc = 0;
        }
    }
}


/*
 ****************************************************************************************
 * @brief Applies Exp Log Log Exp 4 times to a Vector.
 *
 * @param[in/out] Vector    Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_ExpLog(uint8_t *Vector)
{
    uint32_t Index;

    /* Loop 4 times on 4 bytes                                                          */
    for (Index = 0 ; Index < LM_KEY_SIZE ;)
    {
        Vector[Index] = EXP(Vector[Index]);
        Index++;
        Vector[Index] = LOG(Vector[Index]);
        Index++;
        Vector[Index] = LOG(Vector[Index]);
        Index++;
        Vector[Index] = EXP(Vector[Index]);
        Index++;
    }
}

/*
 ****************************************************************************************
 * @brief Determines an Offset on a Key Vector.
 *
 * @param[in]  Key       Key Vector[16]
 * @param[out] OffsetK   Offset Key Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Offset(uint8_t *Key, uint8_t *OffsetK)
{
    OffsetK[0] = Key[0] + 233;
    OffsetK[1] = Key[1] ^ 229;
    OffsetK[2] = Key[2] + 223;
    OffsetK[3] = Key[3] ^ 193;
    OffsetK[4] = Key[4] + 179;
    OffsetK[5] = Key[5] ^ 167;
    OffsetK[6] = Key[6] + 149;
    OffsetK[7] = Key[7] ^ 131;
    OffsetK[8] = Key[8] ^ 233;
    OffsetK[9] = Key[9] + 229;
    OffsetK[10] = Key[10] ^ 223;
    OffsetK[11] = Key[11] + 193;
    OffsetK[12] = Key[12] ^ 179;
    OffsetK[13] = Key[13] + 167;
    OffsetK[14] = Key[14] ^ 149;
    OffsetK[15] = Key[15] + 131;
}

/*
 ****************************************************************************************
 * @brief Initialize Key Scheduling
 *
 * @param[in]  Key            Key Vector[16]
 * @param[out] KeySchedule    Key Vector[17]
 *
 ****************************************************************************************
 */
__STATIC void LM_InitKeySchedule(uint8_t *Key, uint8_t *KeySchedule)
{
    uint32_t Index;
    uint32_t sum = 0;

    /* Copy Key Vector to KeySchedule Vector and XOR each byte                          */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        KeySchedule[Index] = Key[Index];
        sum ^= Key[Index];
    }
    KeySchedule[Index] = (uint8_t)sum;           /* Store XOR Result to the 17 th byte       */
}

/*
 ****************************************************************************************
 * @brief Get a new Key Schedule.
 *
 * @param[in]  Key             Key Vector[17]
 * @param[out] KeySchedule     Key Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_NextKeySchedule(uint8_t *Key, uint8_t *KeySchedule, uint8_t *KeyNumber)
{
    uint32_t srcIndex;
    uint32_t dstIndex;
    uint8_t  biasVector[LM_KEY_SIZE];

    srcIndex = *KeyNumber;

    /* Select octets from Key                                                           */
    for (dstIndex = 0 ; dstIndex < LM_KEY_SIZE ; dstIndex++)
    {
        if (srcIndex > LM_KEY_SIZE)
        {
            srcIndex = 0;
        }
        /* Extract byte for Key Schedule                                                */
        KeySchedule[dstIndex] = Key[srcIndex++];
    }

    /* For Key Schedule 2 to 17, a Bias Vector is needed                                */
    if (*KeyNumber != 0)
    {
        /* Create a Bias Vector                                                         */
        LM_CreateBiasVector(biasVector, *KeyNumber + 1);

        /* Add bias Vector and KeySchedule Vector                                       */
        LM_Add(KeySchedule, biasVector, KeySchedule);
    }

    /* Update Key for next schedule                                                     */
    for (srcIndex = 0 ; srcIndex < (LM_KEY_SIZE + 1) ; srcIndex++)
    {
        Key[srcIndex] = ROL(Key[srcIndex], 3);
    }

    (*KeyNumber)++;                         /* One more key                             */

}


/*
 ****************************************************************************************
 * @brief Create a bias Vector : 45^(45^(17*p+i+1) % 257) % 256.
 *
 * @param[out] Vector      Key Vector[16]
 * @param[in] Number       Vector Number
 *
 ****************************************************************************************
 */
__STATIC void LM_CreateBiasVector(uint8_t *Vector, uint32_t Number)
{
    uint32_t Index;
    uint32_t exp;

    /* For every byte of the bias Vector                                                */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        /* Determines Index for LM_ExtTab access                                        */
        exp = (17 * Number + Index + 1) & 0xFF;

        /* Access in LM_ExtTab (double indirection)                                     */
        Vector[Index] = EXP(EXP(exp));
    }
}


/*
 ****************************************************************************************
 * @brief Add 2 Vectors.
 *
 * @param[in]  VectorIn1   Vector1 [16]
 * @param[in]  VectorIn2   Vector2 [16]
 * @param[out] VectorOut   Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Add(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut)
{
    uint32_t Index;

    /* For each bytes                                                                   */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        VectorOut[Index] = (uint8_t)(VectorIn1[Index] + VectorIn2[Index]);
    }
}


/*
 ****************************************************************************************
 * @brief Xor 2 Vectors.
 *
 * @param[in]  VectorIn1   Vector1 [16]
 * @param[in]  VectorIn2   Vector2 [16]
 * @param[out] VectorOut   Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Xor(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut)
{
    uint32_t Index;

    /* For each bytes                                                                   */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        VectorOut[Index] = VectorIn1[Index] ^ VectorIn2[Index];
    }
}


/*
 ****************************************************************************************
 * @brief Execute Xor Add Add Xor * 4 on 2 Vectors.
 *
 * @param[in]  VectorIn1   Vector1 [16]
 * @param[in]  VectorIn2   Vector2 [16]
 * @param[out] VectorOut   Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_XorAdd(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut)
{
    uint32_t Index;

    /* Loop 4 times on 4 bytes                                                          */
    for (Index = 0 ; Index < LM_KEY_SIZE ;)
    {
        VectorOut[Index] = VectorIn1[Index] ^ VectorIn2[Index];
        Index++;
        VectorOut[Index] = (uint8_t)(VectorIn1[Index] + VectorIn2[Index]);
        Index++;
        VectorOut[Index] = (uint8_t)(VectorIn1[Index] + VectorIn2[Index]);
        Index++;
        VectorOut[Index] = VectorIn1[Index] ^ VectorIn2[Index];
        Index++;
    }
}

/*
 ****************************************************************************************
 * @brief Execute Add Xor Xor Add * 4 on 2 Vectors.
 *
 * @param[in]  VectorIn1   Vector1 [16]
 * @param[in]  VectorIn2   Vector2 [16]
 * @param[out] VectorOut   Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_AddXor(uint8_t *VectorIn1, uint8_t *VectorIn2, uint8_t *VectorOut)
{
    uint32_t Index;

    /* Loop 4 times on 4 bytes                                                          */
    for (Index = 0 ; Index < LM_KEY_SIZE ;)
    {
        VectorOut[Index] = (uint8_t)(VectorIn1[Index] + VectorIn2[Index]);
        Index++;
        VectorOut[Index] = VectorIn1[Index] ^ VectorIn2[Index];
        Index++;
        VectorOut[Index] = VectorIn1[Index] ^ VectorIn2[Index];
        Index++;
        VectorOut[Index] = (uint8_t)(VectorIn1[Index] + VectorIn2[Index]);
        Index++;
    }
}

/*
 ****************************************************************************************
 * @brief Execute a permutation.
 *
 * @param[in/out] Vector    Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Permute(uint8_t *Vector)
{
    uint8_t  TmpVector[LM_KEY_SIZE];
    uint32_t Index;

    /* Copy Input Vector to Temporary Vector                                            */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        TmpVector[Index] = Vector[Index];
    }

    /* Permute Vector                                                                   */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Vector[Index] = TmpVector[LM_Permutation[Index]];
    }
}

/*
 ****************************************************************************************
 * @brief Processes K'c Key for encryption.
 *
 * @param[in]  Kc          Kc Key Vector
 * @param[in]  length      length of the encryption Key (1..16)
 * @param[out] Result      K'c Vector (Result)
 *
 ****************************************************************************************
 */
__STATIC void LM_KPrimC(uint8_t *Kc, uint32_t length, uint8_t *Result)
{
    uint8_t  Index;

    /* Kc modulo g1                                                                     */
    if (length < 16)
    {
        /* Reverse Kc                                                                   */
        LM_ReverseVector(Kc);

        /* result = Kc modulo g1                                                        */
        LM_Modulo(Kc, (uint8_t *)LM_G1Poly[length - 1], (uint32_t)LM_G1Degree[length - 1], Result);

        /* g2 * (Kc modulo g1)                                                          */
        LM_Multiply(Result,
                    (uint8_t *)LM_G2Poly[length - 1],
                    (uint32_t)LM_G2Degree[length - 1],
                    Result);

        /* Reverse Result                                                               */
        LM_ReverseVector(Result);

    }
    else
    {
        /* If Length = 16 , K'c = Kc                                                    */
        for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
        {
            Result[Index] = Kc[Index];
        }
    }
}

/*
 ****************************************************************************************
 * @brief Reverses the vector
 *
 * @param[in/out] Vector       Key Vector[16]
 *
 ****************************************************************************************
 */
__STATIC void LM_ReverseVector(uint8_t *Vector)
{
    int Index;
    uint8_t  Tmp[LM_KEY_SIZE];              /* Tempo Vector                                  */

    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Tmp[Index] = Vector[Index];
    }
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Vector[Index] = Tmp[LM_KEY_SIZE - 1 - Index];
    }
}

/*
 ****************************************************************************************
 * @brief Determines Rest = Dividend modulo Divisor.
 *
 * @param[in]  Dividend        Dividend Poly [16]
 * @param[in]  Divisor         Divisor Poly [16]
 * @param[in]  DivisorDegree   Degree of divisor
 * @param[out] Rest            Rest Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Modulo(uint8_t *Dividend, uint8_t *DivisorTab, uint32_t DivisorDegree, uint8_t *Rest)
{
    uint32_t Clock;
    uint32_t MaskBit;
    uint32_t MaskOff;
    uint8_t  Index;
    uint8_t  Divisor[LM_KEY_SIZE];

    /* Copy Dividend in Rest Vector                                                     */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Rest[Index] = Dividend[Index];
    }

    /* Copy Divisor in Tmp Vector                                                       */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Divisor[Index] = DivisorTab[Index];
    }

    /* align divisor left */
    Clock = 0;
    while ((Divisor[0] & 0x80) == 0)
    {
        Clock++;
        /* the Divisor is shifted by 1 bit Left (a 0 is shifted in LSB)                 */
        Divisor[0]  = (Divisor[0]  << 1) | ((Divisor[1]  & 0x80) >> 7);
        Divisor[1]  = (Divisor[1]  << 1) | ((Divisor[2]  & 0x80) >> 7);
        Divisor[2]  = (Divisor[2]  << 1) | ((Divisor[3]  & 0x80) >> 7);
        Divisor[3]  = (Divisor[3]  << 1) | ((Divisor[4]  & 0x80) >> 7);
        Divisor[4]  = (Divisor[4]  << 1) | ((Divisor[5]  & 0x80) >> 7);
        Divisor[5]  = (Divisor[5]  << 1) | ((Divisor[6]  & 0x80) >> 7);
        Divisor[6]  = (Divisor[6]  << 1) | ((Divisor[7]  & 0x80) >> 7);
        Divisor[7]  = (Divisor[7]  << 1) | ((Divisor[8]  & 0x80) >> 7);
        Divisor[8]  = (Divisor[8]  << 1) | ((Divisor[9]  & 0x80) >> 7);
        Divisor[9]  = (Divisor[9]  << 1) | ((Divisor[10] & 0x80) >> 7);
        Divisor[10] = (Divisor[10] << 1) | ((Divisor[11] & 0x80) >> 7);
        Divisor[11] = (Divisor[11] << 1) | ((Divisor[12] & 0x80) >> 7);
        Divisor[12] = (Divisor[12] << 1) | ((Divisor[13] & 0x80) >> 7);
        Divisor[13] = (Divisor[13] << 1) | ((Divisor[14] & 0x80) >> 7);
        Divisor[14] = (Divisor[14] << 1) | ((Divisor[15] & 0x80) >> 7);
        Divisor[15] = (Divisor[15] << 1);
    }
    Clock++;

    MaskBit = 0x80;
    MaskOff = 0;

    /* Execute 128 times the LFSR process                                               */
    while (Clock-- != 0)
    {
        /* Check if MSB of Dividend = 1                                                 */
        if ((Rest[MaskOff] & MaskBit) != 0)
        {
            /* Rest = Rest XOR Divisor                                                  */
            Rest[0]  ^= Divisor[0];  /* msb */
            Rest[1]  ^= Divisor[1];
            Rest[2]  ^= Divisor[2];
            Rest[3]  ^= Divisor[3];
            Rest[4]  ^= Divisor[4];
            Rest[5]  ^= Divisor[5];
            Rest[6]  ^= Divisor[6];
            Rest[7]  ^= Divisor[7];
            Rest[8]  ^= Divisor[8];
            Rest[9]  ^= Divisor[9];
            Rest[10] ^= Divisor[10];
            Rest[11] ^= Divisor[11];
            Rest[12] ^= Divisor[12];
            Rest[13] ^= Divisor[13];
            Rest[14] ^= Divisor[14];
            Rest[15] ^= Divisor[15];
        }

        /* the Divisor is shifted by 1 bit Left                                         */
        Divisor[15] = (Divisor[15] >> 1) | ((Divisor[14] & 0x01) << 7);
        Divisor[14] = (Divisor[14] >> 1) | ((Divisor[13] & 0x01) << 7);
        Divisor[13] = (Divisor[13] >> 1) | ((Divisor[12] & 0x01) << 7);
        Divisor[12] = (Divisor[12] >> 1) | ((Divisor[11] & 0x01) << 7);
        Divisor[11] = (Divisor[11] >> 1) | ((Divisor[10] & 0x01) << 7);
        Divisor[10] = (Divisor[10] >> 1) | ((Divisor[9]  & 0x01) << 7);
        Divisor[9]  = (Divisor[9]  >> 1) | ((Divisor[8]  & 0x01) << 7);
        Divisor[8]  = (Divisor[8]  >> 1) | ((Divisor[7]  & 0x01) << 7);
        Divisor[7]  = (Divisor[7]  >> 1) | ((Divisor[6]  & 0x01) << 7);
        Divisor[6]  = (Divisor[6]  >> 1) | ((Divisor[5]  & 0x01) << 7);
        Divisor[5]  = (Divisor[5]  >> 1) | ((Divisor[4]  & 0x01) << 7);
        Divisor[4]  = (Divisor[4]  >> 1) | ((Divisor[3]  & 0x01) << 7);
        Divisor[3]  = (Divisor[3]  >> 1) | ((Divisor[2]  & 0x01) << 7);
        Divisor[2]  = (Divisor[2]  >> 1) | ((Divisor[1]  & 0x01) << 7);
        Divisor[1]  = (Divisor[1]  >> 1) | ((Divisor[0]  & 0x01) << 7);
        Divisor[0]  = (Divisor[0]  >> 1);

        if (MaskBit == 0x01)
        {
            MaskBit = 0x80;
            MaskOff++;
        }
        else
        {
            MaskBit >>= 1;
        }
    }
}

/*
 ****************************************************************************************
 * @brief Determines Rest = Polynom1 multiply Polynom2.
 *
 * @param[in]  Polynom1     Poly [16]
 * @param[in]  Polynom2     Poly [16]
 * @param[in]  Poly2Degree  Degree of poly2
 * @param[out] MultPoly     Rest Vector [16]
 *
 ****************************************************************************************
 */
__STATIC void LM_Multiply(uint8_t *Polynom1, uint8_t *Polynom2, uint32_t Poly2Degree, uint8_t *MultPoly)
{
    uint8_t  Index;
    uint8_t  Poly1[LM_KEY_SIZE];
    uint8_t  Poly2[LM_KEY_SIZE];

    /* Reset Result Vector, copy Polynom 1 and 2 to temporary vector                    */
    for (Index = 0 ; Index < LM_KEY_SIZE ; Index++)
    {
        Poly1[Index] = Polynom1[Index];
        Poly2[Index] = Polynom2[Index];
        MultPoly[Index] = 0;
    }

    Poly2Degree++;                          /* Add 1 to take MSB in account             */

    /* Execute Poly2Degree times the process                                            */
    while (Poly2Degree-- != 0)
    {
        /* Check LSB of polynomial 2                                                    */
        if ((Poly2[15] & 0x01) != 0)
        {
            MultPoly[0] ^= Poly1[0];
            MultPoly[1] ^= Poly1[1];
            MultPoly[2] ^= Poly1[2];
            MultPoly[3] ^= Poly1[3];
            MultPoly[4] ^= Poly1[4];
            MultPoly[5] ^= Poly1[5];
            MultPoly[6] ^= Poly1[6];
            MultPoly[7] ^= Poly1[7];
            MultPoly[8] ^= Poly1[8];
            MultPoly[9] ^= Poly1[9];
            MultPoly[10] ^= Poly1[10];
            MultPoly[11] ^= Poly1[11];
            MultPoly[12] ^= Poly1[12];
            MultPoly[13] ^= Poly1[13];
            MultPoly[14] ^= Poly1[14];
            MultPoly[15] ^= Poly1[15];
        }

        /* the polynomial 1 is shifted by 1 bit Left (input 0 in LSB)                   */
        Poly1[0]  = (Poly1[0]  << 1) | ((Poly1[1]  & 0x80) >> 7);
        Poly1[1]  = (Poly1[1]  << 1) | ((Poly1[2]  & 0x80) >> 7);
        Poly1[2]  = (Poly1[2]  << 1) | ((Poly1[3]  & 0x80) >> 7);
        Poly1[3]  = (Poly1[3]  << 1) | ((Poly1[4]  & 0x80) >> 7);
        Poly1[4]  = (Poly1[4]  << 1) | ((Poly1[5]  & 0x80) >> 7);
        Poly1[5]  = (Poly1[5]  << 1) | ((Poly1[6]  & 0x80) >> 7);
        Poly1[6]  = (Poly1[6]  << 1) | ((Poly1[7]  & 0x80) >> 7);
        Poly1[7]  = (Poly1[7]  << 1) | ((Poly1[8]  & 0x80) >> 7);
        Poly1[8]  = (Poly1[8]  << 1) | ((Poly1[9]  & 0x80) >> 7);
        Poly1[9]  = (Poly1[9]  << 1) | ((Poly1[10] & 0x80) >> 7);
        Poly1[10] = (Poly1[10] << 1) | ((Poly1[11] & 0x80) >> 7);
        Poly1[11] = (Poly1[11] << 1) | ((Poly1[12] & 0x80) >> 7);
        Poly1[12] = (Poly1[12] << 1) | ((Poly1[13] & 0x80) >> 7);
        Poly1[13] = (Poly1[13] << 1) | ((Poly1[14] & 0x80) >> 7);
        Poly1[14] = (Poly1[14] << 1) | ((Poly1[15] & 0x80) >> 7);
        Poly1[15] = (Poly1[15] << 1);

        /* the polynomial 2 is shifted by 1 bit Right (input 0 in MSB)                  */
        Poly2[15] = (Poly2[15] >> 1) | ((Poly2[14] & 0x01) << 7);
        Poly2[14] = (Poly2[14] >> 1) | ((Poly2[13] & 0x01) << 7);
        Poly2[13] = (Poly2[13] >> 1) | ((Poly2[12] & 0x01) << 7);
        Poly2[12] = (Poly2[12] >> 1) | ((Poly2[11] & 0x01) << 7);
        Poly2[11] = (Poly2[11] >> 1) | ((Poly2[10] & 0x01) << 7);
        Poly2[10] = (Poly2[10] >> 1) | ((Poly2[9]  & 0x01) << 7);
        Poly2[9]  = (Poly2[9]  >> 1) | ((Poly2[8]  & 0x01) << 7);
        Poly2[8]  = (Poly2[8]  >> 1) | ((Poly2[7]  & 0x01) << 7);
        Poly2[7]  = (Poly2[7]  >> 1) | ((Poly2[6]  & 0x01) << 7);
        Poly2[6]  = (Poly2[6]  >> 1) | ((Poly2[5]  & 0x01) << 7);
        Poly2[5]  = (Poly2[5]  >> 1) | ((Poly2[4]  & 0x01) << 7);
        Poly2[4]  = (Poly2[4]  >> 1) | ((Poly2[3]  & 0x01) << 7);
        Poly2[3]  = (Poly2[3]  >> 1) | ((Poly2[2]  & 0x01) << 7);
        Poly2[2]  = (Poly2[2]  >> 1) | ((Poly2[1]  & 0x01) << 7);
        Poly2[1]  = (Poly2[1]  >> 1) | ((Poly2[0]  & 0x01) << 7);
        Poly2[0]  = (Poly2[0]  >> 1);
    }
}

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */
void E1(struct ltk Key, struct bd_addr BdAddr, struct ltk Random, struct sres_nb *Sres, struct aco *Aco)
{
    LM_E1((uint8_t *)&Key, (uint8_t *)&BdAddr, (uint8_t *)&Random, (uint8_t *)Sres, (uint8_t *)Aco);
}

void E21(struct ltk Random, struct bd_addr BdAddr, struct ltk *Key)
{
    LM_E21((uint8_t *)&Random, (uint8_t *)&BdAddr, (uint8_t *)Key);
}

void E22(struct ltk Random, struct bd_addr BdAddr, struct pin_code Pin, uint8_t PinLen, struct ltk *Key)
{
    LM_E22((uint8_t *)&Random, (uint8_t *)&BdAddr, (uint8_t *)&Pin, PinLen, (uint8_t *)Key);
}

void E3(struct ltk Key, struct aco Cof, struct ltk Random, struct ltk *Kc)
{
    LM_E3((uint8_t *)&Key, (uint8_t *)&Cof, (uint8_t *)&Random, (uint8_t *)Kc);
}

void KPrimC(struct ltk Kc, uint8_t length, struct ltk *KPrimc)
{
    LM_KPrimC((uint8_t *)&Kc, length, (uint8_t *)KPrimc);
}

void XorKey(struct ltk VectorIn1, struct ltk VectorIn2, struct ltk *VectorOut)
{
    LM_Xor((uint8_t *)&VectorIn1, (uint8_t *)&VectorIn2, (uint8_t *)VectorOut);
}

void LM_MakeRandVec(struct ltk *Vector)
{
    uint16_t Index;

    for (Index = 0 ; Index < sizeof(struct ltk) ; Index++)
    {
        Vector->ltk[Index] = (uint8_t)rand();
    }
}

///@} BTUTILKEY
