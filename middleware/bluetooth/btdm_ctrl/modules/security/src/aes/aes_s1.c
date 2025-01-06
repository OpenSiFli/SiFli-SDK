/**
 ****************************************************************************************
 * *
 * @file aes_s1.c
 *
 * @brief Definition file for AES S1 crypto module functions
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 *
 * Algorithm:
 *
 *   s1(M) = AES-CMACZERO (M)
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AES
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "aes_int.h"  // AES internals

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */


/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void aes_s1(const uint8_t *message, uint8_t message_len, aes_func_result_cb res_cb, uint32_t metainfo)
{
    struct aes_cmac_env *aes_cmac;

    // Allocate AES CMAC environent memory
    aes_cmac = (struct aes_cmac_env *) aes_alloc(sizeof(struct aes_cmac_env), (aes_func_continue_cb) aes_cmac_continue,
               res_cb, metainfo);

    // start execution AES CMAC
    aes_cmac_start(aes_cmac, aes_cmac_zero, message, message_len);
}

/// @} AES_CMAC
