/**
  ******************************************************************************
  * @file   file_logger.h
  * @author Sifli software development team
  * @brief Metrics Collector source.
  *
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

#ifndef _FILE_LOGGER_H_
#define _FILE_LOGGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "rtthread.h"


/**
 ****************************************************************************************
* @addtogroup file_logger File Logger
* @ingroup middleware
* @brief File Logger
*
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif

typedef enum fl_err_tag
{
    FL_OK,
    FL_ERROR,
    FL_NOT_INIT,
    FL_INVALID_DATA_LEN,
    FL_WRITE_ERR,
    FL_READ_ERR,
} fl_err_t;


typedef bool (*fl_iter_cb_t)(void *data, uint32_t data_len, void *arg);


/** Create a file logger
 *
 * If the file already exists, it would be opened for write, otherwise a new file is created
 *
 *
 * @param[in] name           file full path
 * @param[in] max_len        file max size
 *
 * @return file logger handle
 */
void *file_logger_init(const char *name, uint32_t max_len);

/** Write data in file logger
 *
 *
 * @param[in] logger         file logger handle created by file_logger_init
 * @param[in] data           point to data to be written
 * @param[in] data_len       data length in byte
 *
 * @return result
 */
fl_err_t file_logger_write(void *logger, void *data, uint32_t data_len);


/** Write data in file logger without header before each write action
 *
 *
 * @param[in] logger         file logger handle created by file_logger_init
 * @param[in] data           point to data to be written
 * @param[in] data_len       data length in byte
 *
 * @return result
 */
fl_err_t file_logger_write_noheader(void *logger, void *data, uint32_t data_len);

/** Iterate all data in file logger in write sequence
 *
 *
 * @param[in] logger         file logger handle created by file_logger_init
 * @param[in] cb             callback for each data
 * @param[in] arg            argument which would be passed to callback
 *
 * @return result
 */
fl_err_t file_logger_iter(void *logger, fl_iter_cb_t cb, void *arg);

/** Clear all data in file logger
 *
 *
 * @param[in] logger         file logger handle created by file_logger_init
 *
 * @return result
 */
fl_err_t file_logger_clear(void *logger);


/** Flush all data to storage, ensure all data is written in storage instead of remaining in cache
 *
 *
 * @param[in] logger         file logger handle created by file_logger_init
 *
 * @return result
 */
fl_err_t file_logger_flush(void *logger);

/** close the file logger
 *
 * Pending data would be flushed
 *
 *
 * @param[in] logger         file logger handle created by file_logger_init
 *
 * @return result
 */
fl_err_t file_logger_close(void *logger);


#ifdef __cplusplus
}
#endif

/// @}  file_logger


#endif /* _FILE_LOGGER_H_ */
