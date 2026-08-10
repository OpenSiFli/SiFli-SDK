/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (C) 2018-2020 AICSemi Ltd.
 *
 * All Rights Reserved
 */

#ifndef RTOS_AL_H_
#define RTOS_AL_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
#include "co_list.h"
#include "aic_plat_mem.h"
#include "aic_plat_sock.h"
#include "aic_plat_types.h"
#include "aic_plat_task.h"



enum aic_task_id
{
    CONTROL_TASK          = 1,
    SUPPLICANT_TASK       = 2,
    SDIO_DATRX_TASK       = 3,
    SDIO_OOBIRQ_TASK      = 4,
    SDIO_PWRCTL_TASK      = 5,
    FHOST_TX_TASK         = 6,
    FHOST_RX_TASK         = 7,
    CLI_CMD_TASK          = 8,
    RWNX_TIMER_TASK       = 9,
    RWNX_APM_STALOSS_TASK = 10,
    WIFI_API_TASK         = 11,
};

/**
 * Time origin
 */
enum time_origin_t
{
    /** Since boot time */
    SINCE_BOOT,
    /** Since Epoch : 1970-01-01 00:00:00 +0000 (UTC) */
    SINCE_EPOCH,
};

#define AIC_RTOS_WAIT_FOREVEVR      -1
#define AIC_RTOS_CREATE_FAILED      5
#define AIC_RTOS_QUEUE_EMPTY        4
#define AIC_RTOS_QUEUE_ERROR        3
#define AIC_RTOS_WAIT_ERROR         1
#define AIC_OS_TIMEOUT              -RT_ETIMEOUT
#define AIC_RTOS_SUCCESS            0
#define AIC_RTOS_ERR                -2
#ifndef RT_THREAD_STAT_YIELD
    #define RT_THREAD_STAT_YIELD  0x00
#endif

#ifndef RT_VERSION_CHECK
#define RT_VERSION_CHECK(major, minor, revise)          ((major * 10000) + \
                                                         (minor * 100) + revise)
#endif
//thread stat
#define AIC_RTOS_READY              RT_THREAD_READY
#define AIC_RTOS_MUTEX_SUSP         RT_THREAD_STAT_SIGNAL_PENDING
#define AIC_RTOS_COMPLETED          RT_THREAD_STAT_YIELD
#define AIC_RTOS_TERMINATED         RT_THREAD_CLOSE
#define AIC_OS_SUSPEND              RT_THREAD_SUSPEND

//rtos timer stat
#define AIC_RTOS_TIMER_ACT          1
#define AIC_RTOS_TIMER_DACT         2
/* DEFINITIONS
****************************************************************************************
*/
/// RTOS tick type
typedef uint32_t rtos_tick_type;

/// RTOS task handle
typedef rt_thread_t rtos_task_handle;

/// RTOS priority
typedef int rtos_prio;

/// RTOS task function
typedef void (*rtos_task_fct)(void *);

/// RTOS queue
typedef rt_mq_t rtos_queue;

/// RTOS semaphore
typedef rt_sem_t  rtos_semaphore;

/// RTOS mutex
typedef rt_mutex_t  rtos_mutex;

/// RTOS event group
typedef int rtos_event_group;

/// RTOS scheduler state
typedef int rtos_sched_state;

/// RTOS timer
typedef rt_timer_t rtos_timer;

/// RTOS timer status
typedef int rtos_timer_status;

/// RTOS timer function
typedef void (*rtos_timer_fct)(void *);

//RTOS TICK
typedef rt_tick_t rtos_tick_t;

//RTOS errt
typedef rt_err_t rtos_err;
#if !SYSYINT_H
    typedef unsigned int    gid_t;
    typedef unsigned int    time_t;
#endif
/*
 * MACROS
 ****************************************************************************************
 */
/// Macro defining a null RTOS task handle
#define RTOS_TASK_NULL             NULL

unsigned long rtos_now(bool isr);
void rtos_msleep(uint32_t time_in_ms);
void rtos_udelay(unsigned int us);

void *rtos_malloc(uint32_t size);
void *rtos_calloc(uint32_t nb_elt, uint32_t size);
void rtos_free(void *ptr);
void rtos_memcpy(void *pdest, const void *psrc, uint32_t size);
void rtos_memset(void *pdest, uint8_t byte, uint32_t size);

void rtos_critical_enter(void);
void rtos_critical_exit(void);

int rtos_task_create(rtos_task_fct func, const char *const name, int task_id, const uint16_t stack_depth,
                     void *const params, rtos_prio prio, rtos_task_handle *const task_handle);
int rtos_task_create_only(rtos_task_fct func, const char *const name, int task_id,
                          const uint16_t stack_depth, void *const params, rtos_prio prio, rtos_task_handle *task_handle);
void rtos_task_delete(rtos_task_handle task_handle);
void rtos_task_suspend(int duration);
void rtos_task_resume(rtos_task_handle task_handle);

int rtos_task_init_notification(rtos_task_handle task);
uint32_t rtos_task_wait_notification(int timeout);
void rtos_task_notify(rtos_task_handle task_handle, uint32_t value, bool isr);
void rtos_task_notify_setbits(rtos_task_handle task_handle, uint32_t value, bool isr);

uint32_t rtos_task_get_priority(rtos_task_handle task_handle);
void rtos_task_set_priority(rtos_task_handle task_handle, uint32_t priority);

int rtos_queue_create(int elt_size, int nb_elt, rtos_queue *queue, const char *const name);
void rtos_queue_delete(rtos_queue queue);
bool rtos_queue_is_empty(rtos_queue queue);
bool rtos_queue_is_full(rtos_queue queue);
int rtos_queue_cnt(rtos_queue queue);
int rtos_queue_write(rtos_queue queue, void *msg, int timeout, bool isr);
int rtos_queue_read(rtos_queue queue, void *msg, int timeout, bool isr);

int rtos_semaphore_create(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count);
int rtos_semaphore_create_only(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count);
int rtos_semaphore_get_count(rtos_semaphore semaphore);
void rtos_semaphore_delete(rtos_semaphore semaphore);
int rtos_semaphore_wait(rtos_semaphore semaphore, int timeout);
int rtos_semaphore_signal(rtos_semaphore semaphore, bool isr);

int rtos_timer_create(const char *const name,
                      rtos_timer *timer,
                      const uint32_t ms,
                      const uint8_t periodic,
                      void *const args,
                      rtos_timer_fct func);
int rtos_timer_start(rtos_timer timer, const uint32_t ms);
int rtos_timer_get_status(rtos_timer timer, rtos_timer_status *timer_status);
int rtos_timer_stop(rtos_timer timer);
int rtos_timer_delete(rtos_timer timer);

int rtos_mutex_create(rtos_mutex *mutex, const char *const name);
void rtos_mutex_delete(rtos_mutex mutex);
int rtos_mutex_lock(rtos_mutex mutex, int timeout);
int rtos_mutex_unlock(rtos_mutex mutex);

#define OS_RES_NAME_MAX     16

typedef enum
{
    //timer first
    RES_TYPE_TIME,
    //task sec
    RES_TYPE_TASK,
    RES_TYPE_SEM,
    RES_TYPE_Q,
    RES_TYPE_MUTEX,
    RES_TYPE_MEM,
    RES_TYPE_MEM_NET_TX,
    RES_TYPE_MEM_NET_RX,
    RES_TYPE_SOCKET,
    RES_TYPE_MAX
} RES_TYPE;


typedef struct
{
    void *ref;
    uint32_t para0;
    uint32_t para1;
    uint32_t type;
    char name[OS_RES_NAME_MAX + 1];
} OsResSt;


typedef struct
{
    struct co_list_hdr hdr;
    OsResSt res;
} OsResList;


bool rtos_task_isvalid(rtos_task_handle task_handle);

int aic_time_get(enum time_origin_t origin, uint32_t *sec, uint32_t *usec);

void rtos_mem_cur_cnt(uint32_t *mtot, uint32_t *ftot, uint32_t *rtot);

uint32_t rtos_wait_task_suspend(rtos_task_handle ref);

uint32_t rtos_get_task_status(rtos_task_handle ref);

void rtos_mem_list_info(void);

void rtos_net_mempush(void *pbuf, uint32_t size, void *pmsg, uint16_t type);

void rtos_net_mempop(void *pmsg, uint16_t type);

void rtos_res_list_info_show(void);

void rtos_remove_all(void);

void rtos_socket_push(int sock);
void rtos_socket_pop(int sock);

extern void platform_net_buf_rx_free(void *ref);

void rtos_wait_all_task_suspend(void);

void rtos_net_mem_free(void *pmsg, uint16_t type);

OsResSt  *rtos_res_find_res(void *ref, uint16_t type);

rtos_task_handle rtos_get_current_task(void);

char *rtos_get_task_name(void *ref);

void rtos_mem_free_task(void *taskref);

uint32_t rtos_wait_task_suspend_only(rtos_task_handle ref);

//if don't use one key may need add del action
#define RTOS_RES_NULL(x) {if(x)x=NULL;}

//micro for some env val
#define RTOS_PVAL_INIT(num,val) void **rtos_pval_##num = (void**)val
extern void **rtos_pval_1;
extern void **rtos_pval_2;

typedef void (*osal_dbg_printf_t)(const char *fmt, ...);

typedef struct
{
    osal_dbg_printf_t osal_dbg_printf;
} osal_porting_ops_t;

extern const osal_porting_ops_t osal_porting_ops;

#endif