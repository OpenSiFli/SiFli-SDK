/**
  ******************************************************************************
  * @file   mc_file_backend.c
  * @author Sifli software development team
  * @brief Metrics Collector source
 *
  ******************************************************************************
*/
/**
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

#include <rtthread.h>
#include "board.h"
#include <string.h>
#include "metrics_collector.h"
#include "file_logger.h"

#include "dfs_posix.h"
#include "log.h"

void *mc_backend_init(const char *name, uint32_t max_size)
{
    return file_logger_init(name, max_size);
}

mc_err_t mc_backend_write(void *db, void *data, uint32_t data_len)
{
    return (mc_err_t)file_logger_write(db, data, data_len);
}

mc_err_t mc_backend_iter(void *db, mc_backend_iter_cb_t cb, void *arg)
{
    return (mc_err_t)file_logger_iter(db, (fl_iter_cb_t)cb, arg);
}

mc_err_t mc_backend_clear(void *db)
{
    return (mc_err_t)file_logger_clear(db);
}

mc_err_t mc_backend_flush(void *db)
{

    return (mc_err_t)file_logger_flush(db);
}

mc_err_t mc_backend_close(void *db)
{
    return (mc_err_t)file_logger_close(db);
}

