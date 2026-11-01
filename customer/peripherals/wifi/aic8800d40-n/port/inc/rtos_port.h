/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTOS port header: ops-dispatch macros for host/platform code.
 */

#ifndef RTOS_AL_H_
#define RTOS_AL_H_

#include "aic_porting_types.h"
#include "co_list.h"
#include <errno.h>

/* ============================================================
 * Shared type definitions
 * ============================================================ */
#ifndef _AIC_OSAL_TYPES_DEFINED_
#define _AIC_OSAL_TYPES_DEFINED_

/// RTOS tick type
typedef uint32_t rtos_tick_type;
/// RTOS task handle
typedef void *rtos_task_handle;
/// RTOS priority
typedef int rtos_prio;
/// RTOS task function
typedef void (*rtos_task_fct)(void *);
/// RTOS queue
typedef void *rtos_queue;
/// RTOS semaphore
typedef void *rtos_semaphore;
/// RTOS mutex
typedef void *rtos_mutex;
/// RTOS event group
typedef void *rtos_event_group;
/// RTOS scheduler state
typedef int rtos_sched_state;
/// RTOS timer
typedef void *rtos_timer;
/// RTOS timer status
typedef int rtos_timer_status;
/// RTOS timer function
typedef void (*rtos_timer_fct)(void *arg);

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
    RWNX_APM_TASK         = 10,
};

enum time_origin_t
{
    SINCE_BOOT,
    SINCE_EPOCH,
};

#define RTOS_AL_INFO_DUMP   0
#define RTOS_INFO_SHORT     1
#define RTOS_DEL_NO_WAIT    1

#if RTOS_INFO_SHORT
    #define OS_RES_NAME_MAX     7
#else
    #define OS_RES_NAME_MAX     15
#endif

typedef enum
{
    RES_TYPE_TIME,
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
    uintptr_t para0;
    uintptr_t para1;
    uint32_t type;
    char name[OS_RES_NAME_MAX + 1];
} OsResSt;

#endif /* _AIC_OSAL_TYPES_DEFINED_ */

/* ============================================================
 * Common macros
 * ============================================================ */
#define AIC_RTOS_WAIT_FOREVEVR      -1
#define AIC_RTOS_CREATE_FAILED      121 /* EREMOTEIO */
#ifndef EREMOTEIO
    #define EREMOTEIO                   121
#endif

/*
 * Must match the platform OSA/semaphore "block forever" convention
 * (FreeRTOS portMAX_DELAY = 0xFFFFFFFF). 0 means "return immediately" on
 * several ports and used to busy-spin callers that passed -1 for "wait".
 */
#define AIC_OS_SUSPEND              (0xFFFFFFFFu)
#define AIC_OS_NO_WAIT              0
#define AIC_OS_TIMEOUT              0

#define AIC_RTOS_SUCCESS            0
#define AIC_RTOS_READY              1
#define AIC_RTOS_WAIT_ERROR         -1
#define AIC_RTOS_MUTEX_SUSP         2
#define AIC_RTOS_QUEUE_EMPTY        1
#define AIC_RTOS_QUEUE_ERROR        1
/* Note: AIC_RTOS_COMPLETED / AIC_RTOS_TERMINATED are early abstract state
 * constants; the RTOS ports now compare the native state directly
 * (FreeRTOS eTaskState / RT-Thread RT_THREAD_*), and they are no longer
 * referenced in the baseline; kept only for compatibility with platform
 * implementations still using them, do not use in new code. */
#define AIC_RTOS_COMPLETED          3
#define AIC_RTOS_TERMINATED         4

#define AIC_RTOS_TIMER_STATUS_ACTIVE    0
#define AIC_RTOS_TIMER_STATUS_INACTIVE  1

#define RTOS_TASK_NULL             NULL

/* OsResList */
typedef struct
{
    struct co_list_hdr hdr;
    OsResSt res;
} OsResList;

#define RTOS_RES_NULL(x) {if(x)x=NULL;}
#define RTOS_PVAL_INIT(num,val) void **rtos_pval_##num = (void**)val
extern void **rtos_pval_1;
extern void **rtos_pval_2;

/* ============================================================
 * RTOS abstraction layer — platform must implement ALL functions
 * No ops tables, no indirect calls, no registration.
 * Missing implementations → linker error (not silent NULL deref).
 * ============================================================ */

/* Memory — memcpy/memset remain as direct <string.h> macros */
#include <string.h>
#define aicwf_rtos_memcpy(dst, src, sz)           memcpy((dst), (src), (sz))
#define aicwf_rtos_memset(dst, b, sz)             memset((dst), (b), (sz))

/* ---- RTOS memory ---- */
void *aicwf_rtos_malloc(uint32_t size);
void *aicwf_rtos_calloc(uint32_t nb_elt, uint32_t size);
void  aicwf_rtos_free(void *ptr);

/* ---- RTOS time ---- */
unsigned long aicwf_rtos_now(bool isr);
void aicwf_rtos_msleep(uint32_t ms);
void aicwf_rtos_udelay(unsigned int us);
int  aicwf_time_get(enum time_origin_t origin, uint32_t *sec, uint32_t *usec);
void aicwf_random_init(void);
uint8_t aicwf_random_byte(void);

/* ---- RTOS task ---- */
int  aicwf_rtos_task_create(rtos_task_fct func, const char *name, int task_id,
                            uint16_t stack_depth, void *params, rtos_prio prio,
                            rtos_task_handle *task_handle);
int  aicwf_rtos_task_create_only(rtos_task_fct func, const char *name, int task_id,
                                 uint16_t stack_depth, void *params, rtos_prio prio,
                                 rtos_task_handle *task_handle);
void aicwf_rtos_task_delete(rtos_task_handle task_handle);
void aicwf_rtos_task_suspend(int duration);
void aicwf_rtos_task_resume(rtos_task_handle task_handle);
int  aicwf_rtos_task_init_notification(rtos_task_handle task);
uint32_t aicwf_rtos_task_wait_notification(int timeout);
void aicwf_rtos_task_notify(rtos_task_handle task_handle, uint32_t value, bool isr);
void aicwf_rtos_task_notify_setbits(rtos_task_handle task_handle, uint32_t value, bool isr);
uint32_t aicwf_rtos_task_get_priority(rtos_task_handle task_handle);
void aicwf_rtos_task_set_priority(rtos_task_handle task_handle, uint32_t priority);
bool aicwf_rtos_task_isvalid(rtos_task_handle task_handle);
rtos_task_handle aicwf_rtos_get_current_task(void);
char *aicwf_rtos_get_task_name(void *ref);
void aicwf_rtos_mem_free_task(void *taskref);

/* ---- RTOS queue ---- */
int  aicwf_rtos_queue_create(int elt_size, int nb_elt, rtos_queue *queue, const char *name);
void aicwf_rtos_queue_delete(rtos_queue queue);
bool aicwf_rtos_queue_is_empty(rtos_queue queue);
bool aicwf_rtos_queue_is_full(rtos_queue queue);
int  aicwf_rtos_queue_cnt(rtos_queue queue);
int  aicwf_rtos_queue_write(rtos_queue queue, uint32_t size, void *msg, int timeout, bool isr);
int  aicwf_rtos_queue_write_intimer(rtos_queue queue, uint32_t size, void *msg, int timeout, bool isr);
int  aicwf_rtos_queue_read(rtos_queue queue, uint32_t size, void *msg, int timeout, bool isr);

/* ---- RTOS semaphore ---- */
int  aicwf_rtos_semaphore_create(rtos_semaphore *semaphore, const char *name, int max_count, int init_count);
int  aicwf_rtos_semaphore_create_only(rtos_semaphore *semaphore, const char *name, int max_count, int init_count);
int  aicwf_rtos_semaphore_get_count(rtos_semaphore semaphore);
int  aicwf_rtos_semaphore_get_count_fromisr(rtos_semaphore semaphore);
void aicwf_rtos_semaphore_delete(rtos_semaphore semaphore);
int  aicwf_rtos_semaphore_wait(rtos_semaphore semaphore, int timeout);
int  aicwf_rtos_semaphore_signal(rtos_semaphore semaphore, bool isr);

/* ---- RTOS timer ---- */
int  aicwf_rtos_timer_create(rtos_timer *timer, char *name);
int  aicwf_rtos_timer_start(rtos_timer timer, uint32_t initialTime, uint32_t rescheduleTime,
                            rtos_timer_fct func, void *args);
int  aicwf_rtos_timer_get_status(rtos_timer timer, rtos_timer_status *timer_status);
int  aicwf_rtos_timer_stop(rtos_timer timer);
int  aicwf_rtos_timer_delete(rtos_timer timer);

/* ---- RTOS mutex ---- */
int  aicwf_rtos_mutex_create(rtos_mutex *mutex, const char *name);
int  aicwf_rtos_mutex_create_only(rtos_mutex *mutex, const char *name);
void aicwf_rtos_mutex_delete(rtos_mutex mutex);
int  aicwf_rtos_mutex_lock(rtos_mutex mutex, int timeout);
int  aicwf_rtos_mutex_unlock(rtos_mutex mutex);

/* ---- RTOS critical section (ISR-safe) ----
 * Short critical section protecting queues shared between USB URB complete(ISR)
 * and tasks.
 * The returned level is the CPU interrupt mask before entry; pass it back as-is
 * on exit.
 * The implementation must be safe in both task and ISR context
 * (FreeRTOS: taskENTER_CRITICAL_FROM_ISR). */
uint32_t aicwf_rtos_critical_enter(void);
void     aicwf_rtos_critical_exit(uint32_t level);

/* ---- RTOS event group ---- */
int  aicwf_rtos_event_group_create(rtos_event_group *event_group);
void aicwf_rtos_event_group_delete(rtos_event_group event_group);
uint32_t aicwf_rtos_event_group_get_bits(rtos_event_group event_group, bool isr);
uint32_t aicwf_rtos_event_group_wait_bits(rtos_event_group event_group, uint32_t val,
        bool clear_on_exit, bool wait_all_bits, int timeout);
uint32_t aicwf_rtos_event_group_set_bits(rtos_event_group event_group, uint32_t val, bool isr);

/* ---- Resource tracking (optional, platform may provide empty stubs) ---- */
void aicwf_rtos_mem_cur_cnt(uint32_t *mtot, uint32_t *ftot, uint32_t *rtot);
/**
 * @brief Get the task status
 *
 * **Return value semantics: must be the native status value of this RTOS**; the
 * callers (aicwf_rtos_wait_task_suspend* etc.) compare it with the status
 * constants of this RTOS -- FreeRTOS: eRunning/eReady/eBlocked/eSuspended/
 * eDeleted (task.h); RT-Thread: RT_THREAD_* (see rtos_rtt.c).
 * Do NOT map it to the AIC_RTOS_* abstract constants: the values do not
 * correspond and would be misjudged (eBlocked=2 on FreeRTOS was once taken as
 * "runnable", which hung ap stop, see fd13c86).
 */
uint32_t aicwf_rtos_get_task_status(rtos_task_handle ref);
uint32_t aicwf_rtos_wait_task_suspend(rtos_task_handle ref);
uint32_t aicwf_rtos_wait_task_suspend_only(rtos_task_handle ref);
void aicwf_rtos_wait_all_task_suspend(void);
void aicwf_rtos_res_list_info_show(void);
void aicwf_rtos_remove_all(void);
void aicwf_rtos_net_mempush(void *pbuf, uint32_t size, void *pmsg, uint16_t type);
void aicwf_rtos_net_mempop(void *pmsg, uint16_t type);
void aicwf_rtos_net_mem_free(void *pmsg, uint16_t type);
void aicwf_rtos_mem_list_info(void);
void aicwf_rtos_socket_push(int sock, uint32_t port);
void aicwf_rtos_socket_pop(int sock);
uint32_t aicwf_rtos_socket_find(int sock);
OsResSt *aicwf_rtos_res_find_res(void *ref, uint16_t type);

#endif /* RTOS_AL_H_ */
