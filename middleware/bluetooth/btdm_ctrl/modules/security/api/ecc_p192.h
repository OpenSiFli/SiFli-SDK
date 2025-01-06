/**
 ****************************************************************************************
 *
 * @file ecc_p192.h
 *
 * @brief  ECC functions for P192
 *
 * Copyright (C) RivieraWaves 2009-2022
 *
 ****************************************************************************************
 */

#ifndef ECC_P192_H_
#define ECC_P192_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if(BT_EMB_PRESENT)
#include <stdint.h>
#include <stdbool.h>
#include "ke_task.h"
#include "co_bt_defines.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Elliptic Curve computation result structure
typedef struct ecc_p192_result
{
    uint8_t key_res_x[PUB_KEY_192_LEN / 2];
    uint8_t key_res_y[PUB_KEY_192_LEN / 2];
} ecc_p192_result_t;

/**
 * Callback executed when Elliptic Curve algorithm completes
 *
 * @param[in] metainfo      Metadata information provided by API user (see \glos{METAINFO})
 * @param[in] status        Computation status result
 * @param[in] p_res         Pointer to Computed result
 */
typedef void (*ecc_p192_result_cb)(uint32_t metainfo, uint8_t status, const ecc_p192_result_t *p_res);


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
void ecc_p192_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Generate a Secret Key compliant with ECC P192 algorithm
 *
 * If key is forced, just check its validity
 *
 * @param[out] secret_key Private key - MSB First
 ****************************************************************************************
 */
void ecc_p192_gen_new_secret_key(uint8_t *secret_key);

/**
 ****************************************************************************************
 * @brief Generate a new Public key pair using ECC P192 algorithm
 *
 * @param[in] secret_key Private key - MSB First
 * @param[in] metainfo   Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 * @param[in] cb_result  Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
uint8_t ecc_p192_gen_new_public_key(const uint8_t *secret_key, uint32_t metainfo, ecc_p192_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Generate a new DHKey using ECC P192 algorithm
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
uint8_t ecc_p192_gen_dh_key(const uint8_t *secret_key, const uint8_t *public_key_x, const uint8_t *public_key_y,
                            uint32_t metainfo, ecc_p192_result_cb cb_result);

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
uint8_t ecc_p192_gen_key_pair(bool debug_key, uint8_t *secret_key, uint32_t metainfo, ecc_p192_result_cb cb_result);

/**
 ****************************************************************************************
 * @brief Abort a current DHKey generation procedure
 *
 * @param[in] metainfo  Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 ****************************************************************************************
 */
void ecc_p192_abort_key_gen(uint16_t metainfo);


/**
 ****************************************************************************************
 * @brief Get Pointer to debug public key key X Coordinate
 * @return Pointer to debug key
 ****************************************************************************************
 */
const uint8_t *ecc_p192_get_debug_pub_key_x(void);

/**
 ****************************************************************************************
 * @brief Get Pointer to debug public key key Y Coordinate
 * @return Pointer to debug key
 ****************************************************************************************
 */
const uint8_t *ecc_p192_get_debug_pub_key_y(void);
#endif // (BT_EMB_PRESENT)

#endif /* ECC_P192_H_ */
