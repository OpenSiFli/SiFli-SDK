/**
  ******************************************************************************
  * @file   mc_tsdb_backend.c
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

#include "flashdb.h"

#ifdef FDB_USING_FILE_MODE
    #include "dfs_posix.h"
#endif /* FDB_USING_FILE_MODE */

typedef struct
{
    fdb_tsdb_t db;
    mc_backend_iter_cb_t cb;
    void *arg;

} mc_backend_iter_arg_t;

static fdb_time_t get_time(void)
{
    time_t t = 0;

    time(&t);
    return t;
}

static bool tsl_cb(fdb_tsl_t tsl, void *arg)
{
    struct fdb_blob blob;
    void *data;
    size_t read_len;
    mc_backend_iter_arg_t *iter_arg = (mc_backend_iter_arg_t *)arg;
    bool res;

    if (FDB_TSL_WRITE != tsl->status)
    {
        return false;
    }

    if (0 == tsl->log_len)
    {
        return false;
    }

    data = rt_malloc(tsl->log_len);
    RT_ASSERT(data);

    read_len = fdb_blob_read((fdb_db_t)iter_arg->db, fdb_tsl_to_blob(tsl, fdb_blob_make(&blob, data, tsl->log_len)));
    RT_ASSERT(read_len == tsl->log_len);
    res = iter_arg->cb(data, tsl->log_len, iter_arg->arg);

    rt_free(data);

    return res;
}

void *mc_backend_init(const char *name, uint32_t max_size)
{
    fdb_err_t result;
    struct fdb_tsdb *tsdb;

    tsdb = rt_malloc(sizeof(*tsdb));

    if (!tsdb)
    {
        return NULL;
    }

    memset(tsdb, 0, sizeof(*tsdb));

#ifdef FDB_USING_FILE_MODE
#include "fal.h"
    int sec_size = PKG_FLASHDB_ERASE_GRAN;
    bool file_mode = true;
    rt_kprintf("mc db init: sector_size %d size %d\n", sec_size, max_size);
    fdb_tsdb_control(tsdb, FDB_TSDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
    fdb_tsdb_control(tsdb, FDB_TSDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
    fdb_tsdb_control(tsdb, FDB_TSDB_CTRL_SET_FILE_MODE, (void *)&file_mode);
    if (0 != access(name, 0) && 0 != mkdir(name, 0))
    {
        rt_kprintf("create db %s fail\n", name);
        return -1;
    }

#endif /* FDB_USING_FILE_MODE */

    result = fdb_tsdb_init(tsdb, name, name, get_time, MC_MAX_DATA_LEN, NULL);
    RT_ASSERT(FDB_NO_ERR == result);

    return tsdb;
}



mc_err_t mc_backend_write(void *db, void *data, uint32_t data_len)
{
    struct fdb_blob blob;
    fdb_err_t result;

    RT_ASSERT(db && data);

    if (0 == data_len)
    {
        return MC_OK;
    }

    result = fdb_tsl_append((struct fdb_tsdb *)db, fdb_blob_make(&blob, (void *)data, data_len));

    rt_kprintf("result:%d\n", result);

    RT_ASSERT(FDB_NO_ERR == result);

    return MC_OK;
}

mc_err_t mc_backend_iter(void *db, mc_backend_iter_cb_t cb, void *arg)
{
    mc_backend_iter_arg_t iter_arg;

    iter_arg.db = (fdb_tsdb_t)db;
    iter_arg.cb = cb;
    iter_arg.arg = arg;
    fdb_tsl_iter((fdb_tsdb_t)db, tsl_cb, (void *)&iter_arg);

    return MC_OK;
}

mc_err_t mc_backend_clear(void *db)
{
    fdb_tsl_clean((fdb_tsdb_t)db);

    return MC_OK;
}



