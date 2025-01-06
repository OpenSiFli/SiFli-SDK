/**
 ****************************************************************************************
 *
 * @file ecc_p256.h
 *
 * @brief  ECC functions for P256
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef ECC_P256_H_
#define ECC_P256_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define ECC_256_KEY_SIZE         (32)


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Elliptic Curve computation result structure
typedef struct ecc_p256_result
{
    uint8_t key_res_x[ECC_256_KEY_SIZE];
    uint8_t key_res_y[ECC_256_KEY_SIZE];
} ecc_p256_result_t;

/**
 * Callback executed when Elliptic Curve algorithm completes
 *
 * @param[in] metainfo      Metadata information provided by API user (see \glos{METAINFO})
 * @param[in] p_res         Pointer to Computed result
 */
typedef void (*ecc_p256_result_cb)(uint32_t metainfo, const ecc_p256_result_t *p_res);


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Elliptic Curve algorithm
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void ecc_p256_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Generate a Secret Key compliant with ECC P256 algorithm
 *
 * If key is forced, just check its validity
 *
 * @param[out] secret_key Private key - MSB First
 ****************************************************************************************
 */
void ecc_p256_gen_new_secret_key(uint8_t *secret_key);

/**
 ****************************************************************************************
 * @brief Generate a new Public key pair using ECC P256 algorithm
 *
 * @param[in] secret_key Private key - MSB First
 * @param[in] metainfo   Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 * @param[in] cb_result  Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
uint8_t ecc_p256_gen_new_public_key(uint8_t *secret_key256, uint32_t metainfo, ecc_p256_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Generate a new key pair
 *
 * @param[out] debug_key  True if a debug key must be create, False to generate a random one.
 * @param[out] secret_key Private key - MSB First
 * @param[in]  metainfo   Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 * @param[in]  cb_result  Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
uint8_t ecc_p256_gen_key_pair(bool debug_key, uint8_t *secret_key, uint32_t metainfo, ecc_p256_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Generate a new DHKey using ECC P256 algorithm
 *
 * @param[in] secret_key        Private key                  - MSB First
 * @param[in] public_key_x      Peer public key x coordinate - LSB First
 * @param[in] public_key_y      Peer public key y coordinate - LSB First
 * @param[in] metainfo          Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 * @param[in] cb_result         Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
uint8_t ecc_p256_gen_dh_key(const uint8_t *secret_key, const uint8_t *public_key_x, const uint8_t *public_key_y,
                            uint32_t metainfo, ecc_p256_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Abort a current DHKey generation procedure
 *
 * @param[in] metainfo  Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void ecc_p256_abort_key_gen(uint16_t metainfo);

/**
 ****************************************************************************************
 * @brief Get Pointer to debug private key
 * @return Pointer to debug key
 ****************************************************************************************
 */
const uint8_t *ecc_p256_get_debug_priv_key(void);

/**
 ****************************************************************************************
 * @brief Get Pointer to debug public key key X Coordinate
 * @return Pointer to debug key
 ****************************************************************************************
 */
const uint8_t *ecc_p256_get_debug_pub_key_x(void);

/**
 ****************************************************************************************
 * @brief Get Pointer to debug public key key Y Coordinate
 * @return Pointer to debug key
 ****************************************************************************************
 */
const uint8_t *ecc_p256_get_debug_pub_key_y(void);

#endif /* ECC_P256_H_ */
