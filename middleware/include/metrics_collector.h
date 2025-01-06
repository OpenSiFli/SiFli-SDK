/**
  ******************************************************************************
  * @file   metrics_collector.h
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

#ifndef _METRICS_COLLECTOR_H_
#define _METRICS_COLLECTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "rtthread.h"
#if defined(MC_SERVICE_ENABLED) || defined(MC_CLIENT_ENABLED)
    #include "data_service.h"
#endif /* MC_SERVICE_ENABLED || MC_CLIENT_ENABLED */


/**
 ****************************************************************************************
* @addtogroup metrics_collector Metrics Collector
* @ingroup middleware
* @brief Metrics Collector Manager, use flashdb partition "metrics" as storage.
*
* It's initialized at INIT_COMPONENT stage and should be called at INIT_ENV, INIT_PRE_APP or INIT_APP stage.
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif

#ifndef MC_MAX_DATA_LEN
#define MC_MAX_DATA_LEN (256)
#endif /* MC_MAX_DATA_LEN */


#if defined(MC_SERVICE_ENABLED) || defined(MC_CLIENT_ENABLED)
enum
{
    MSG_SERVICE_METRICS_SAVE_REQ             = (MSG_SERVICE_CUSTOM_ID_BEGIN),
    MSG_SERVICE_METRICS_SAVE_RSP              = MSG_SERVICE_METRICS_SAVE_REQ | RSP_MSG_TYPE,
};

typedef struct mc_metrics_save_req_tag
{
    uint8_t data[0];
} mc_metrics_save_req_t;

#endif /* MC_SERVICE_ENABLED || MC_CLIENT_ENABLED */



typedef enum mc_period_tag
{
    MC_PERIOD_EVERY_DAY,    /**< every day */
    MC_PERIOD_EVERY_HOUR,   /**< every hour */
    MC_PERIOD_EVERY_MINUTE, /**< every minute */
    MC_PERIOD_EVERY_SECOND, /**< every second */
    MC_PERIOD_MAX
} mc_period_t;

typedef enum mc_err_tag
{
    MC_OK,
    MC_ERROR,
    MC_NOT_INIT,
    MC_INVALID_DATA_LEN,
    MC_WRITE_ERR,
    MC_READ_ERR,
    MC_DB_INIT_ERR,
} mc_err_t;

/** collector callback type
 *
 * @param[in] user_data user data
 */
typedef void (*mc_collector_callback_t)(void *user_data);


typedef struct mc_collector_tag
{
    rt_list_t node;                    /**< internal use, no need to fill by caller */
    mc_collector_callback_t callback;  /**< collector callback, woudld called according to period */
    mc_period_t period;                /**< period */
    void *user_data;                   /**< user data, would be passed as argument of callback */
} mc_collector_t;


/** raw metrics read callback type
 *
 * @param[in] data        point to read data, no need to free by user
 * @param[in] data_len    data length in byte
 * @param[in] time        time in second
 * @retval                true: interrupt read, false: continue read
 */
typedef bool (*mc_raw_metrics_read_callback_t)(void *data, uint32_t data_len, uint32_t time);

/** metrics read callback type
 *
 * @param[in] id          id
 * @param[in] core        core id, @ref #CORE_ID_HCPU
 * @param[in] data_len    data length in byte
 * @param[in] time        time in second
 * @param[in] data        point to read data, no need to free by user
 * @retval                true: interrupt read, false: continue read
 */
typedef bool (*mc_metrics_read_callback_t)(uint16_t id, uint8_t core, uint16_t data_len, uint32_t time, void *data);

typedef bool (*mc_backend_iter_cb_t)(void *data, uint32_t data_len, void *arg);

typedef struct
{
    const char *name;
    void *db;
    uint32_t max_size;
    rt_list_t node;
} mc_db_t;


/** Register metrics collector
 *
 * The memory of param collector will still be used after the invocation, until #mc_deregister_collector is called.
 * So the memory should be available within this time.
 *
 * @param[in] collector   collector
 *
 * @return result
 */
mc_err_t mc_register_collector(mc_collector_t *collector);

/** Deregister metrics collector
 *
 * After the invocation, the memory of param collector will not be used any more.
 *
 * @param[in] collector   collector
 *
 * @return result
 */
mc_err_t mc_deregister_collector(mc_collector_t *collector);

/** Allocate memory space for one metrics
 *
 * data_len cannot exceed MC_MAX_DATA_LEN
 *
 *
 * @param[in] id           metrics id, it's user defined
 * @param[in] data_len     metrics data length in byte
 *
 * @return pointer to metrics data buffer, buffer size is same as data_len
 */
void *mc_alloc_metrics(uint16_t id, uint16_t data_len);

/** Save the metrics to storage (default db)
 *
 *
 * @param[in] metrics      the metrics allocated by #mc_alloc_metrics
 * @param[in] freed        whether the metrics memory can be freed after the invocation. true: free, false: not free
 *
 * @return result
 */
mc_err_t mc_save_metrics(void *metrics, bool freed);

/** Save the metrics to specified db
 *
 * @param[in] db           metrics db handle initialzed by mc_init_db
 * @param[in] metrics      the metrics allocated by #mc_alloc_metrics
 * @param[in] freed        whether the metrics memory can be freed after the invocation. true: free, false: not free
 *
 * @return result
 */
mc_err_t mc_save_metrics_ex(mc_db_t *db, void *metrics, bool freed);

/** Free the metrics memory allocated by #mc_alloc_metrics
 *
 *
 * @param[in] metrics      the metrics allocated by #mc_alloc_metrics
 *
 * @return result
 */
mc_err_t mc_free_metrics(void *metrics);

/** Read raw metrics saved in storage
 *
 * The callback would be called for each metrics in the database, raw data is provided.
 *
 *
 * @param[in] cb           read callback
 *
 * @return result
 */
mc_err_t mc_read_raw_metrics(mc_raw_metrics_read_callback_t cb);

/** Read metrics saved in storage
 *
 * The callback would be called for each metrics in the database, parsed data is provided.
 *
 * @param[in] cb           read callback
 *
 * @return result
 */
mc_err_t mc_read_metrics(mc_metrics_read_callback_t cb);

/** Clear metrics saved in storage
 *
 *
 *
 * @return result
 */
mc_err_t mc_clear_metrics(void);

/** Initialize db
 *
 * @param[in,out] db       metrics db handle
 * @param[in] name         db name
 * @param[in] max_size     db size
 *
 * @return result
 */
mc_err_t mc_init_db(mc_db_t *db, const char *name, uint32_t max_size);

#if !defined(MC_CLIENT_ENABLED) && defined(MC_AUTO_INIT_DISABLED)
/** Init metrics collector manually
 *
 * It can be called if MC_AUTO_INIT_DISABLED is defined
 *
 * @return result
 */
mc_err_t mc_init(void);
#endif /* !MC_CLIENT_ENABLED && MC_AUTO_INIT_DISABLED */

/** flush db
 *
 * Ensure all data is written in storage instead of remaining in cache
 *
 * @param[in] db       metrics db handle
 *
 * @return result
 */
mc_err_t mc_flush_ex(mc_db_t *db);


/** flush default db
 *
 * Ensure all data is written into storage instead of remaining in cache
 *
 *
 * @return result
 */
mc_err_t mc_flush(void);

/** close default db
 *
 * Same as mc_flush, plus default db is closed afterwards and no more data can be written further
 *
 *
 * @return result
 */
mc_err_t mc_close(void);

/** Get default db path with file backend
 *
 *
 *
 * @return path
 */
const char *mc_get_path(void);

void mc_backend_direct_write(mc_db_t *db, uint16_t id, void *data, uint16_t size);



/*******************************************************************
 *
 * Following API will be implemented by backend
 *
 *********************************************************************/

void *mc_backend_init(const char *name, uint32_t max_len);

mc_err_t mc_backend_write(void *db, void *data, uint32_t data_len);

mc_err_t mc_backend_iter(void *db, mc_backend_iter_cb_t cb, void *arg);

mc_err_t mc_backend_clear(void *db);

mc_err_t mc_backend_flush(void *db);

mc_err_t mc_backend_close(void *db);

mc_db_t *mc_get_metrics_db(void);



#ifdef __cplusplus
}
#endif

/// @}  metrics_collector


#endif /* _METRICS_COLLECTOR_H_ */
