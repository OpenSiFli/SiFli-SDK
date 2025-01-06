/**
  ******************************************************************************
  * @file   bf0_hal_aes.h
  * @author Sifli software development team
  * @brief AES encrypt/decrypt enginer hardware interface.
  * @{
  ******************************************************************************
*/
/*
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __BF0_HAL_AES_H
#define __BF0_HAL_AES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

#if defined(HAL_AES_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup AES AES Encrypt/decrypt engine
  * @brief AES encrypt/decrypt enginer hardware interface.
  * @{
  */

/**
  * @addtogroup  AES_exported_constants
  * @{
*/
#define AES_DEC 0
#define AES_ENC 1

#define AES_MODE_ECB 0      /*!< AES mode ECB */
#define AES_MODE_CTR 1      /*!< AES mode CTR */
#define AES_MODE_CBC 2      /*!< AES mode CBC */

#define AES_KEY_LEN_128 0
#define AES_KEY_LEN_192 1
#define AES_KEY_LEN_256 2
/**
  * @} AES_exported_constants
*/


/**
  * @defgroup AES_exported_functions AES Exported functions
  * @ingroup AES
  * @{
  *
 */
/**
* @brief  Initialize AES hardware block
* @param  key aes key, must be 32bit aligned
* @param  key_size aes key size in bytes
* @param  iv initial vector, must be 32bit aligned
* @param  mode aes mode
* @retval 0 if successful, otherwise -1
*/
int HAL_AES_init(uint32_t *key, int key_size, uint32_t *iv, uint32_t mode);
int HAL_AES_init_ns(uint32_t *key, int key_size, uint32_t *iv, uint32_t mode);

/**
* @brief  run AES hardware enc/dec
* @param  enc 1:encoding, 0:decoding
* @param  in_data input data, input data could not in ITCM or Retention memory
* @param  out_data output data, output data could not in ITCM or Retention memory
* @param  size length of input/output data in bytes, must be multiple of 16 bytes.
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_AES_run(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size);
HAL_StatusTypeDef HAL_AES_run_ns(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size);

/**
* @brief  run AES hardware enc/dec, generate interrupt when done.
* @param  enc 1:encoding, 0:decoding
* @param  in_data input data, input data could not in ITCM or Retention memory
* @param  out_data output data, output data could not in ITCM or Retention memory
* @param  size length of input/output data in bytes, must be multiple of 16 bytes.
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_AES_run_IT(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size);
HAL_StatusTypeDef HAL_AES_run_IT_ns(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size);

/**
* @brief  Copy 'src' to 'dst' like DMA, with no encoding either decoding.
* @param  src source data, source data could not in ITCM or Retention memory
* @param  dst destination data, destination data could not in ITCM or Retention memory
* @param  size length of src/dst data in bytes, must be multiple of 16 bytes.
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_AES_copy_IT_ns(uint8_t *src, uint8_t *dst, int size);



/** Test whether AES hardware block is busy.*/
#define HAL_AES_busy() (hwp_aes_acc->STATUS & AES_ACC_STATUS_BUSY)

/**
* @brief  Reset AES hardware block
* @retval 0 if successful, otherwise -1
*/
int HAL_AES_reset(void);
int HAL_AES_reset_ns(void);

/**
* @brief  AES IRQ handler
*/
void HAL_AES_IRQHandler(void);
void HAL_AES_IRQHandler_ns(void);

/**
  *@} AES_exported_functions
*/
#endif

/**
  *@} AES
  */

/** @defgroup HASH Hash engine
  * @brief Hash engine hardware interface.
  * @{
  */

/**
  * @addtogroup  HASH_exported_constants
  * @{
*/

/**
  * @defgroup HASH_ALGO_XXX Hash algorithm
  * @brief  Hash algorithm definitions
  * @{
  */
enum
{
    HASH_ALGO_SHA1      = 0,    /*!< HASH algorithm SHA1 */
    HASH_ALGO_SHA224    = 1,    /*!< HASH algorithm SHA224 */
    HASH_ALGO_SHA256    = 2,    /*!< HASH algorithm SHA256 */
    HASH_ALGO_SM3       = 3,    /*!< HASH algorithm SM3 */
    HASH_ALGO_MAX       = 3     /*!< MAX HASH algorithm */
} ;
/**
  * @} HASH_ALGO_XXX
*/


#define HASH_SHA1_SIZE      20     /*!< SHA1 size */
#define HASH_SHA224_SIZE    32     /*!< HASH algorithm SHA224, only 28 bytes in valid in final result. 32 bytes need for internal usage */
#define HASH_SHA256_SIZE    32     /*!< HASH algorithm SHA256 */
#define HASH_SM3_SIZE       32     /*!< HASH algorithm SM3 */



/**
  * @} HASH_exported_constants
*/


#if defined(HAL_HASH_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
/**
  * @defgroup HASH_exported_functions HASH Exported functions
  * @ingroup AES
  * @{
  *
 */
/**
* @brief  Initialize AES Hash hardware block
* @param  iv initial vector, must be 32bit aligned, if NULL, use algorithm default initial vector.
          Initial vector is previous finished hash result, reload this to resume hash calculation.
          Previous finished hash block length, must be 64 bytes aligned.
* @param  algo hash algorithm, see @ref HASH_ALGO_XXX
* @param  length previous finished length, must be 64bytes aligned.
* @retval 0 if successful, otherwise -1
*/
int HAL_HASH_init(uint32_t *iv, uint8_t algo, uint32_t length);
int HAL_HASH_init_ns(uint32_t *iv, uint8_t algo, uint32_t length);

/**
* @brief  run Hash hardware
* @param  in_data input data, input data could not in ITCM or Retention memory
* @param  size length of input data in bytes. if not last block, size must be 64 bytes aligned.
* @param  final Last block
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_HASH_run(uint8_t *in_data, int size, int final);
HAL_StatusTypeDef HAL_HASH_run_ns(uint8_t *in_data, int size, int final);

/**
* @brief  run Hash hardware, return immediately, generate AES HASH interrupt when done.
* @param  in_data input data, input data could not in ITCM or Retention memory
* @param  size length of input data in bytes. if not last block, size must be 64 bytes aligned
* @param  final Last block
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_HASH_run_IT(uint8_t *in_data, int size, int final);
HAL_StatusTypeDef HAL_HASH_run_IT_ns(uint8_t *in_data, int size, int final);

/**
* @brief  Get hash result.
* @param  out_data output data, Size of out_data depends on hash algorithm
* @note  SHA224 result is 28 bytes, but internal need 32 byte, please allocate 32 bytes for usage.
*/
void HAL_HASH_result(uint8_t *out_data);
void HAL_HASH_result_ns(uint8_t *out_data);

/** Test whether AES hardware block is busy.*/
#define HAL_HASH_busy HAL_AES_busy

/**
* @brief  Reset AES HASH hardware block
* @retval 0 if successful, otherwise -1
*/
int HAL_HASH_reset(void);
int HAL_HASH_reset_ns(void);

/**
* @brief  AES HASH IRQ handler
*/
void HAL_HASH_IRQHandler(void);
void HAL_HASH_IRQHandler_ns(void);
/**
  *@} HASH_exported_functions
*/

#define HASH_IRQHandler AES_IRQHandler
#else
#define HAL_HASH_IRQHandler()
#define HAL_HASH_IRQHandler_ns()
#endif


typedef void (*pAESCallback)(void);
HAL_StatusTypeDef HAL_AES_Regist_IT_cb(pAESCallback cb);
HAL_StatusTypeDef HAL_AES_Regist_IT_cb_ns(pAESCallback cb);
#define HAL_HASH_Regist_IT_cb HAL_AES_Regist_IT_cb
#define HAL_HASH_Regist_IT_cb_ns HAL_AES_Regist_IT_cb_ns

/**
  *@} Hash
  */

/**
  *@} BF0_HAL_Driver
  */

#ifdef __cplusplus
}
#endif

#endif

/// @}  file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
