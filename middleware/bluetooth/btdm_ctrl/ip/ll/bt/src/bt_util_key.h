/**
****************************************************************************************
*
* @file bt_util_key.h
*
* @brief link manager key management
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef BT_UTIL_KEY_H_
#define BT_UTIL_KEY_H_
/**
****************************************************************************************
* @defgroup LMKEY Link Keys
* @ingroup LM
* @brief LM module for link key management.
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "co_bt.h"          // BT standard definitions

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Processes E1 algorithm for Authentication
 *
 * @param[in]  Key       Key Vector[16]
 * @param[in]  BdAddr    BdAddr[6]
 * @param[in]  Random    Random Vector[16]
 * @param[out] Sres      SRES Vector[4]
 * @param[out] Aco       ACIO Vector[12]
 *
 ****************************************************************************************
 */
void E1(struct ltk Key, struct bd_addr BdAddr, struct ltk Random, struct sres_nb *Sres, struct aco *Aco);

/**
 ****************************************************************************************
 * @brief Processes E21 algorithm for Authentication
 *
 * @param[in]  Random      Random Vector[16]
 * @param[in]  BdAddr      BdAddr[6]
 * @param[out] Key         Result Vector[16]
 *
 ****************************************************************************************
 */
void E21(struct ltk Random, struct bd_addr BdAddr, struct ltk *Key);

/**
 ****************************************************************************************
 * @brief Processes E22 algorithm for Authentication
 *
 * @param[in]  Random    Random Vector[16]
 * @param[in]  BdAddr    BdAddr[6]
 * @param[in]  Pin       PIN Vector[1..16]
 * @param[in]  PinLen    PIN length (1..16)
 * @param[out] KeyOut    Result Vector[16]
 *
 ****************************************************************************************
 */
void E22(struct ltk Random, struct bd_addr BdAddr, struct pin_code Pin, uint8_t PinLen, struct ltk *KeyOut);
/**
 ****************************************************************************************
 * @brief Processes E3 algorithm for Authentication
 *
 * @param[in]  Key       Key Vector[16]
 * @param[in]  Cof       COF[12]
 * @param[in]  Random    Random Vector[16]
 * @param[out] KeyOut    Result Vector[16]
 *
 ****************************************************************************************
 */
void E3(struct ltk Key, struct aco Cof, struct ltk Random, struct ltk *KeyOut);

/**
 ****************************************************************************************
 * @brief Encryption : Processes K'c Key for encryption
 *
 * @param[in]  Random      Random Vector[16]
 * @param[in]  BdAddr      BdAddr[6]
 * @param[out] KPrimC      K'c Vector (Result)
 *
 ****************************************************************************************
 */
void KPrimC(struct ltk Kc, uint8_t length, struct ltk *KPrimC);

/**
 ****************************************************************************************
 * @brief Xor of 2 Vectors
 *
 * @param[in]  VectorIn1      Input Vector1 [16]
 * @param[in]  VectorIn2      Input Vector2 [16]
 * @param[out] VectorOut      Result Vector[16]
 *
 ****************************************************************************************
 */
void XorKey(struct ltk VectorIn1, struct ltk VectorIn2, struct ltk *VectorOut);

/**
 ****************************************************************************************
 * @brief This function is used to get a random vector.
 *
 * @param[out] Vector      Random Vector
 *
 ****************************************************************************************
 */
void LM_MakeRandVec(struct ltk *Vector);

///@} LMKEY

#endif // BT_UTIL_KEY_H_
