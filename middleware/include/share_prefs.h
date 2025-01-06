/**
  ******************************************************************************
  * @file   share_prefs.h
  * @author Sifli software development team
  * @brief Sifli Shared preference API
  * @{
  ******************************************************************************
*/
/*
 * @attention
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

#ifndef SHARE_PREFS_H
#define SHARE_PREFS_H

#include <rtthread.h>
#include <stdint.h>
#include <string.h>


/**
 ****************************************************************************************
* @addtogroup shard_pref Shared preference
* @ingroup middleware
* @brief Sifli Shared preference API
* @{
****************************************************************************************
*/

#define SHARE_PREFS_MAX_NAME_LEN 32

/// Preference data type
typedef struct
{
    char prfs_name[SHARE_PREFS_MAX_NAME_LEN]; /*!< Preference name*/
    uint32_t mode;                            /*!< Preference mode*/
} share_prefs_t;


/// Preference mode enumeration.
typedef enum
{
    SHAREPREFS_MODE_PRIVATE = 0x1,              /*!< Private */
    SHAREPREFS_MODE_WORLD_READABLE = 0x2,       /*!< Readable */
    SHAREPREFS_MODE_WORLD_WRITEABLE = 0x4,      /*!< Writable */
} share_prefs_mode;



///share_prefs api

/**
    @brief Open shared preference database
    @param[in] prfs_name Preference name
    @param[in] mode      Mode to open
    @retval Handle of shared preference database
*/
share_prefs_t *share_prefs_open(const char *prfs_name, uint32_t mode);
/**
    @brief Close shared preference database
    @param[in] prfs Handle of shared preference database
    @retval RT_EOK if successful, otherwise return error number <0
*/
rt_err_t share_prefs_close(share_prefs_t *prfs);
/**
    @brief Clear shared preference database
    @param[in] prfs Handle of shared preference database
    @retval RT_EOK if successful, otherwise return error number <0
*/
rt_err_t share_prefs_clear(share_prefs_t *prfs);


/**
    @brief Remove an entry for a preference database
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @retval RT_EOK if successful, otherwise return error number <0
*/
rt_err_t share_prefs_remove(share_prefs_t *prfs, const char *key);


//int
/**
    @brief Get a integer type of a preference
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @param[in] default_v Default value if preference not found    .
    @retval value of interger
*/
int32_t share_prefs_get_int(share_prefs_t *prfs, const char *key, int32_t default_v);

/**
    @brief Set a integer type of a preference
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @param[in] value Value to be set
    @retval RT_EOK if successful, otherwise return error number <0
*/rt_err_t share_prefs_set_int(share_prefs_t *prfs, const char *key, int32_t value);

//string
/**
    @brief Get a string type of a preference
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @param[in,out] buf Data buffer of preference value
    @param[in] buf_len Max buffer length
    @retval Length of buf if successful, otherwise return error number <0
*/
int32_t share_prefs_get_string(share_prefs_t *prfs, const char *key, char *buf, int32_t buf_len);
/*
    set string
*/
/**
    @brief Set a string type of a preference
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @param[in] buf Data buffer of preference value
    @retval RT_EOK if successful, otherwise return error number <0
*/
rt_err_t share_prefs_set_string(share_prefs_t *prfs, const char *key, const char *buf);


//block
/**
    @brief Get a general block type of a preference
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @param[in,out] buf Data buffer of preference value
    @param[in] buf_len Max buffer length
    @retval Length of buf if successful, otherwise return error number <0
*/
int32_t share_prefs_get_block(share_prefs_t *prfs, const char *key, void *buf, int32_t buf_len);


/**
    @brief Set a general block type of a preference
    @param[in] prfs Handle of shared preference database
    @param[in] key Key name of preference entry
    @param[in] buf Data buffer of preference value
    @param[in] buf_len  Buffer length
    @retval RT_EOK if successful, otherwise return error number <0
*/
rt_err_t share_prefs_set_block(share_prefs_t *prfs, const char *key, const void *buf, int32_t buf_len);

/// @} shard_pref
/// @} file

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
