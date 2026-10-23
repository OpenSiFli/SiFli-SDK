/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RT-Thread RTOS port implementation for AIC8800 WiFi driver.
 * Based on rtos_freertos.c — provides equivalent API surface.
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "rtos_port.h"
#include "aic_log.h"
#include "aic_assert.h"
#include "netif_port.h"

#include <rtthread.h>
#include <rthw.h>      /* [AIC8800-PORT][SiFli] rt_hw_interrupt_disable/enable */
#include <rtdevice.h>

/* Forward declarations for static functions used before definition */
rtos_task_handle aicwf_rtos_get_current_task(void);

#ifdef CONFIG_RTOS_RESLIST

// ASR plat need mem statistics
#define AIC_RTOS_MEM_MAX        ((1024+256)*1024)

static uint32_t rtos_mem_cnt = 0;
static uint32_t rtos_free_cnt = 0;
static uint32_t rtos_remain_cnt = 0;

static rtos_mutex rtos_mutexRef = NULL;
struct co_list res_list[RES_TYPE_MAX] = {NULL, NULL};

static uint32_t rtos_res_time_cnt = 0;

static __INLINE void aicwf_OsMemCal(uint8_t inc, uint32_t len)
{
    if (inc)
    {
        rtos_remain_cnt += len;
        rtos_mem_cnt += len;
    }
    else
    {
        rtos_remain_cnt -= len;
        rtos_free_cnt += len;
    }
}

static __INLINE void aicwf_OsResInit(OsResSt *st, char *name, void *ref)
{
    memset(st, 0, sizeof(OsResSt));
    if (name != NULL)
        memcpy(st->name, name, strlen(name) > OS_RES_NAME_MAX ? OS_RES_NAME_MAX : strlen(name));
#if !RTOS_INFO_SHORT
    else
    {
        sprintf(st->name, "%#x", ref);
    }
#endif
    st->ref = ref;
}

extern int aic_close(int s);
extern int aic_shutdown(int s, int how);
extern void rtos_socket_close_all(void);
void aicwf_rtos_del_by_type(OsResSt *st)
{
#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("[rtos]del: type[%d]:%-16s, %#8x, %#8x, taskhdl=%#8x\n",
                 st->type, st->name, st->ref, st->para0, st->para1);
#endif

    switch (st->type)
    {
    case RES_TYPE_MUTEX:
        rt_mutex_delete((rtos_mutex)st->ref);
        break;
    case RES_TYPE_SEM:
        rt_sem_delete((rtos_semaphore)st->ref);
        break;
    case RES_TYPE_Q:
        rt_mq_delete((rtos_queue)st->ref);
        break;
    case RES_TYPE_TIME:
        if (st->ref)
            rt_timer_delete((rtos_timer)st->ref);
        break;
    case RES_TYPE_TASK:
        /* Delete RT-Thread thread object (only when not in CLOSE state) */
        if (st->ref)
        {
            if (RT_THREAD_CLOSE != ((rt_thread_t)st->ref)->stat)
                rt_thread_delete((rt_thread_t)st->ref);
        }
        if (st->para0)
            free((void *)st->para0);
        break;
    case RES_TYPE_MEM:

        if (rtos_remain_cnt < st->para0)
        {
            DBG_OSAL_WRN("[rtos][warning] rtos mem cnt:%d %d %d\n", rtos_remain_cnt, rtos_free_cnt, rtos_mem_cnt);
            //AIC_ASSERT_ERR(0);
        }

        aicwf_OsMemCal(0, st->para0);
        free(st->ref);
        break;

    case RES_TYPE_SOCKET:
        aic_shutdown((int)(st->para0), 2);
        aic_close((int)(st->para0));
        break;

    default:
        break;
    }
}

/**
 * @brief Acquire the RTOS resource mutex
 *
 * Protects concurrent access to the resource list; creates the mutex semaphore on first call.
 *
 * @return RT_EOK=acquired successfully
 */
uint32_t aicwf_rtos_res_mutex_lock(void)
{
    if (rtos_mutexRef == NULL)
    {
        rtos_mutexRef = rt_mutex_create("rtos_res_mutex", RT_IPC_FLAG_FIFO);
        AIC_ASSERT_ERR(rtos_mutexRef != NULL);
    }

    return rt_mutex_take(rtos_mutexRef, RT_WAITING_FOREVER);
}

/**
 * @brief Release the RTOS resource mutex
 */
void aicwf_rtos_res_mutex_unlock(void)
{
    if (rtos_mutexRef)
        rt_mutex_release(rtos_mutexRef);
}

/**
 * @brief Push a resource record onto the resource tracking list
 * @param res Resource descriptor pointer
 * @return Newly allocated resource list node pointer, NULL=failure
 */
OsResList *aicwf_rtos_res_list_push(OsResSt *res)
{
    uint32_t hs = sizeof(OsResList);
    uint32_t nl;
    OsResList *p;

    p = malloc(hs);

    if (p == NULL)
    {
        DBG_OSAL_ERR("[rtos] %s: fail\n", __func__);
        return 0;
    }

    nl = strlen(res->name);
    nl = (nl > OS_RES_NAME_MAX) ? OS_RES_NAME_MAX : nl;
    aicwf_rtos_res_mutex_lock();
    p->res.para0 = res->para0;
    p->res.para1 = res->para1;
    p->res.ref = res->ref;
    p->res.type = res->type;
    memcpy(p->res.name, res->name, nl);
    p->res.name[nl] = 0;

    aicwf_OsMemCal(1, hs);
    if ((res->type == RES_TYPE_MEM) || (res->type == RES_TYPE_MEM_NET_RX))
    {
        aicwf_OsMemCal(1, res->para0);
    }
    if (res->type == RES_TYPE_TIME)
    {
        p->res.para0 = rtos_res_time_cnt++;
    }

    co_list_push_back(&res_list[res->type], &(p->hdr));

    aicwf_rtos_res_mutex_unlock();

#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("[rtos]push:type[%d]:%-16s, %#8x, %#8x, taskhdl=%#8x\n", res->type, res->name, res->ref, p, aicwf_rtos_get_current_task());
#endif
    return p;
}

/**
 * @brief Release a single resource by reference pointer
 * @param ref  Resource reference pointer
 * @param type Resource type (low 8 bits) and parameter (high 8 bits)
 */
void aicwf_rtos_res_list_release(void *ref, uint16_t type)
{
    uint8_t rtype = type & 0xff;
    uint8_t rpara = (type >> 8) & 0xff;
    OsResList *pRes;
    struct co_list *plist;

    aicwf_rtos_res_mutex_lock();
    plist = &res_list[rtype];
    pRes = (OsResList *)co_list_pick(plist);

    while (pRes)
    {
        if (pRes->res.ref)
        {
            if (pRes->res.ref == ref)
            {
                break;
            }
        }
        else if (pRes->res.para0 == rpara)
        {
            break;
        }
        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }

    if (pRes != NULL)
    {
        if (pRes->res.type == RES_TYPE_TIME)
        {
            if (rtos_res_time_cnt)
                rtos_res_time_cnt--;
        }
        aicwf_rtos_del_by_type(&(pRes->res));
        co_list_extract(plist, &(pRes->hdr));
        free(pRes);
        aicwf_OsMemCal(0, sizeof(OsResList));
    }
    aicwf_rtos_res_mutex_unlock();

    if (pRes == NULL)
    {
        DBG_OSAL_INF("[rtos]rtos res no this ref?:%#8x type:%d, taskhdl=%#8x\r\n", ref, type, aicwf_rtos_get_current_task());
    }
}

/**
 * @brief Release all tracked RTOS resources
 *
 * Iterates all resource type lists, deleting each entry and freeing memory.
 * Usually called during WiFi de-initialization.
 */
void aicwf_rtos_res_release_all(void)
{
    OsResList *pRes, *pres1;
    struct co_list *plist;
    int i = 0;

    aicwf_rtos_res_mutex_lock();

    for (i = 0; i < RES_TYPE_MAX; i++)
    {
        plist = &res_list[i];
        pRes = (OsResList *)co_list_pick(plist);

        while (pRes)
        {
            DBG_OSAL_INF("[rtos]del:type[%d]:%-16s, %#8x, taskhdl=%#8x\n",
                         pRes->res.type, pRes->res.name, pRes->res.ref, aicwf_rtos_get_current_task());
            aicwf_rtos_del_by_type(&pRes->res);
            pres1 = pRes;
            pRes = (OsResList *)co_list_next(&(pRes->hdr));
            free((void *)pres1);
            aicwf_OsMemCal(0, sizeof(OsResList));
        }

        co_list_init(plist);
    }

    aicwf_rtos_res_mutex_unlock();

    /* [backport EC] On early release after an init failure, rtos_pval may not
     * yet be assigned by the platform; check for NULL to avoid a hardfault */
    if (rtos_pval_1)
        *rtos_pval_1 = NULL;
    if (rtos_pval_2)
        *rtos_pval_2 = NULL;
}

/**
 * @brief Remove a node from the resource list (does not delete the resource)
 *
 * Unlike _release: only unlinks from the list, no low-level delete call.
 * Used for resources managed manually, such as sockets.
 *
 * @param ref  Resource reference pointer
 * @param type Resource type and parameter
 */
void aicwf_rtos_res_list_rellist(void *ref, uint16_t type)
{
    uint8_t rtype = type & 0x00ff;
    uint8_t rpara = (type >> 8) & 0xff;
    OsResList *pRes;
    struct co_list *plist;

    aicwf_rtos_res_mutex_lock();
    plist = &res_list[rtype];
    pRes = (OsResList *)co_list_pick(plist);

    while (pRes)
    {
        if (pRes->res.ref)
        {
            if (pRes->res.ref == ref)
            {
                break;
            }
        }
        else if (pRes->res.para0 == rpara)
        {
            break;
        }
        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }


    if (pRes != NULL)
    {
        if (rtype == RES_TYPE_SOCKET)
            DBG_OSAL_INF("[rtos]del list:type[%d]:%-16s, %x, taskhdl=%#8x\n",
                         pRes->res.type, pRes->res.name, pRes->res.ref, aicwf_rtos_get_current_task());
        co_list_extract(plist, &(pRes->hdr));
        aicwf_OsMemCal(0, sizeof(OsResList));
        free(pRes);
    }

    aicwf_rtos_res_mutex_unlock();
}


/**
 * @brief Find the specified resource in the resource list
 * @param ref  Resource reference pointer
 * @param type Resource type and parameter
 * @return Pointer to the found resource structure, NULL=not found
 */
OsResSt   *aicwf_rtos_res_find_res(void *ref, uint16_t type)
{

    uint8_t rtype = type & 0xff;
    uint8_t rpara = (type >> 8) & 0xff;
    OsResList *pRes;
    struct co_list *plist;

    aicwf_rtos_res_mutex_lock();
    plist = &res_list[rtype];
    pRes = (OsResList *)co_list_pick(plist);

    while (pRes)
    {
        if (pRes->res.ref)
        {
            if (pRes->res.ref == ref)
            {
                break;
            }
        }
        else if (pRes->res.para0 == rpara)
        {
            break;
        }
        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }

    aicwf_rtos_res_mutex_unlock();

    if (pRes)
        return &(pRes->res);
    else
    {
        DBG_OSAL_INF("[[rtos]]rtos find none:%#8x type:%d, taskhdl=%#8x\r", ref, type, aicwf_rtos_get_current_task());
        return NULL;
    }

}

static OsResSt   *__aicwf_rtos_res_find_res_by_para(uint16_t type, uintptr_t para)
{

    uint8_t rtype = type & 0xff;
    uint8_t rpara = (type >> 8) & 0xff;
    OsResList *pRes;
    struct co_list *plist;

    aicwf_rtos_res_mutex_lock();
    plist = &res_list[rtype];
    pRes = (OsResList *)co_list_pick(plist);

    while (pRes)
    {
        if (rpara == 0)
        {
            if (pRes->res.para0 == para)
            {
                break;
            }
        }
        else
        {
            if (pRes->res.para1 == para)
                break;
        }
        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }

    aicwf_rtos_res_mutex_unlock();

    if (pRes)
        return &(pRes->res);
    else
    {
        DBG_OSAL_INF("[[rtos]]rtos find none:%#8x type:%d, taskhdl=%#8x\r\n", para, type, aicwf_rtos_get_current_task());
        return NULL;
    }

}
#endif /* CONFIG_RTOS_RESLIST */

/**
 * @brief Dump resource list information (for debugging)
 *
 * Prints information of all resource types, including name, reference, parameters.
 */
void aicwf_rtos_res_list_info_show(void)
{
#ifdef CONFIG_RTOS_RESLIST
    OsResList *pRes;
    struct co_list *plist;
    int i = 0;

    aicwf_rtos_res_mutex_lock();

    for (i = 0; i < RES_TYPE_MAX; i++)
    {
        plist = &res_list[i];
        pRes = (OsResList *)co_list_pick(plist);

        while (pRes)
        {
            DBG_OSAL_VRB("[rtos]type[%d] name:%-16s, ref:%#8x para0:%#8x, para1:%#8x\n",
                         pRes->res.type, pRes->res.name, pRes->res.ref, pRes->res.para0, pRes->res.para1);
            pRes = (OsResList *)co_list_next(&(pRes->hdr));
        }

    }

    aicwf_rtos_res_mutex_unlock();

    DBG_OSAL_INF("[rtos][show] rtos mem cnt:%d %d %d %d\n",
                 co_list_cnt(&res_list[RES_TYPE_MEM]), rtos_remain_cnt, rtos_free_cnt, rtos_mem_cnt);
#endif
}


__STATIC_INLINE uint32_t rtos_ms_to_tick(unsigned long ms)
{
    return rt_tick_from_millisecond(ms);
}

/**
 * @brief Convert tick count to milliseconds
 * @param tick RT-Thread tick count
 * @return Corresponding milliseconds
 */
unsigned long aicwf_rtos_tick_to_ms(uint32_t tick)
{
    return (uint32_t)tick * (1000 / RT_TICK_PER_SECOND);
}

__STATIC_INLINE  rt_tick_t rtos_timeout_2_tickcount(int timeout_ms)
{
    if (timeout_ms < 0)
    {
        return RT_WAITING_FOREVER;
    }
    else
    {
        return rt_tick_from_millisecond(timeout_ms);
    }
}

/**
 * @brief Get current system time (milliseconds)
 * @param isr Whether called from interrupt context (tick read is ISR-safe in RT-Thread)
 * @return Milliseconds since system start
 */
unsigned long aicwf_rtos_now(bool isr)
{
    unsigned long tick = 0;
    tick = rt_tick_get();
    return aicwf_rtos_tick_to_ms(tick);
}

/**
 * @brief Millisecond-level delay
 * @param time_in_ms Delay in milliseconds
 */
void aicwf_rtos_msleep(uint32_t time_in_ms)
{
    if (time_in_ms == 0)
        time_in_ms = 1;

    rt_thread_mdelay(time_in_ms);
}

/**
 * @brief Microsecond-level delay
 * @param us Delay in microseconds
 * [AIC8800-PORT][SiFli] Baseline uses extern udelay(); SiFli/RT-Thread has no
 * such symbol, so use rt_hw_us_delay(rt_uint32_t) from rthw.h (board SysTick/timer).
 */
void aicwf_rtos_udelay(unsigned int us)
{
    rt_hw_us_delay((rt_uint32_t)us);
}

/**
 * @brief RTOS heap allocation (with resource tracking)
 *
 * Allocates memory and registers it in the resource tracking list.
 * The memory cap is checked when CONFIG_RTOS_RESLIST is enabled.
 *
 * @param size Allocation size (bytes)
 * @return Memory pointer, NULL=failure
 */
void *aicwf_rtos_malloc(uint32_t size)
{
#ifdef CONFIG_RTOS_RESLIST
    void *pdata = NULL;
    OsResSt st;

    if (rtos_remain_cnt > AIC_RTOS_MEM_MAX)
    {
        DBG_OSAL_INF("[rtos]rtos malloc none space\r\n");
        return NULL;
    }
    if (size == 0)
    {
        DBG_OSAL_INF("[rtos]rtos malloc size 0\r\n");
        return NULL;
    }

    pdata = malloc(size);
    if (pdata)
    {
        void *tref = NULL;
        char *name = NULL;
        tref = aicwf_rtos_get_current_task();
#if !RTOS_INFO_SHORT
        name = aicwf_rtos_get_task_name(tref);
#endif
        aicwf_OsResInit(&st, name, pdata);
        st.para0 = size;
        st.para1 = (uint32_t)tref;
        st.ref = pdata;
        st.type = RES_TYPE_MEM;
        aicwf_rtos_res_list_push(&st);
    }
    else
    {
        DBG_OSAL_ERR("[rtos]malloc fail\r\n");
    }

#else /* CONFIG_RTOS_RESLIST */

    void *pdata = malloc(size);
    if (!pdata)
    {
        DBG_OSAL_ERR("[rtos]malloc fail\r\n");
    }

#endif /* CONFIG_RTOS_RESLIST */

    return pdata;
}

/**
 * @brief RTOS heap allocation with zeroing (calloc semantics)
 * @param nb_elt Number of elements
 * @param size   Size of each element
 * @return Zeroed memory pointer, NULL=failure
 */
void *aicwf_rtos_calloc(uint32_t nb_elt, uint32_t size)
{
    if (nb_elt == 0 || size == 0)
    {
        DBG_OSAL_INF("[rtos]rtos calloc size 0\r\n");
        return NULL;
    }

    void *pdata = aicwf_rtos_malloc(nb_elt * size);
    if (pdata)
    {
        memset(pdata, 0, nb_elt * size);
    }

    return pdata;
}

/**
 * @brief RTOS heap free
 *
 * Frees the memory and removes it from the resource tracking list.
 *
 * @param ptr Memory pointer to free
 */
void aicwf_rtos_free(void *ptr)
{
#ifdef CONFIG_RTOS_RESLIST
    if (ptr != NULL)
    {
        aicwf_rtos_res_list_release(ptr, RES_TYPE_MEM);
    }

#else

    free(ptr);

#endif
}



/**
 * @brief Get current memory usage statistics
 * @param mtot Output: total allocated
 * @param ftot Output: total freed
 * @param rtot Output: currently remaining
 */
void aicwf_rtos_mem_cur_cnt(uint32_t *mtot, uint32_t *ftot, uint32_t *rtot)
{
#ifdef CONFIG_RTOS_RESLIST
    *mtot = rtos_mem_cnt;
    *ftot = rtos_free_cnt;
    *rtot = rtos_remain_cnt;
#endif
}

/**
 * @brief Remove all RTOS resources and reset memory statistics
 */
void aicwf_rtos_remove_all(void)
{
#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_release_all();

    if (rtos_remain_cnt != 0)
    {
        DBG_OSAL_WRN("[rtos][warning] rtos mem cnt:%d %d %d\n", rtos_remain_cnt, rtos_free_cnt, rtos_mem_cnt);
    }

    rtos_remain_cnt = 0;
    rtos_free_cnt = 0;
    rtos_mem_cnt = 0;

#ifdef CONFIG_SOCK_RTOS_AL
    rtos_socket_close_all();
#endif
#endif
}

/**
 * @brief Free all memory associated with the specified task
 * @param taskref Task handle
 */
void aicwf_rtos_mem_free_task(void *taskref)
{
#ifdef CONFIG_RTOS_RESLIST
    uintptr_t para = (uintptr_t)taskref;
    OsResSt *resp = NULL;

    do
    {
        resp = __aicwf_rtos_res_find_res_by_para(RES_TYPE_MEM | 0x0100, para);
        if (resp)
            aicwf_rtos_free(resp->ref);
    }
    while (resp);
#endif
}

/**
 * @brief Track a network buffer enqueue
 * @param pbuf  pbuf pointer
 * @param size  Buffer size
 * @param pmsg  Message reference
 * @param type  Resource type
 */
void aicwf_rtos_net_mempush(void *pbuf, uint32_t size, void *pmsg, uint16_t type)
{
#ifdef CONFIG_RTOS_RESLIST
    OsResSt st;
    if (pbuf)
    {
        aicwf_OsResInit(&st, NULL, pmsg);
        st.para0 = size;
        st.ref = pmsg;
        st.para1 = (uint32_t)pbuf;
        st.type = type;
        aicwf_rtos_res_list_push(&st);

#if RTOS_AL_INFO_DUMP
        DBG_OSAL_INF("[rtos]type[%d] name:%-16s, ref:%#8x, para0:%#8x, para1:%#8x\n",
                     st.type, st.name, st.ref, st.para0, st.para1);
#endif
    }
#endif
}

/**
 * @brief Track a network buffer dequeue
 * @param pmsg Message reference
 * @param type Resource type
 */
void aicwf_rtos_net_mempop(void *pmsg, uint16_t type)
{
#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_list_rellist(pmsg, type);
#endif
}

/**
 * @brief Free network buffer resources
 * @param pmsg Message reference
 * @param type Resource type
 */
void aicwf_rtos_net_mem_free(void *pmsg, uint16_t type)
{
#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_list_release(pmsg, type);
#endif
}

/**
 * @brief Register a socket in the resource tracking list
 * @param sock Socket file descriptor
 * @param port Bound port number
 */
void aicwf_rtos_socket_push(int sock, uint32_t port)
{
#ifdef CONFIG_RTOS_RESLIST
    OsResSt st;
    char name[10];

    if (sock != -1)
    {
        sprintf(name, "sock%d", sock);
        aicwf_OsResInit(&st, name, NULL);
        st.para0 = sock;
        st.para1 = port;
        st.type = RES_TYPE_SOCKET;
        aicwf_rtos_res_list_push(&st);
        DBG_OSAL_INF("[rtos]type[%d] name:%-16s, para0:%#8x\n", st.type, st.name, st.para0);
    }
#endif
}

/**
 * @brief Remove a socket from the resource tracking list
 * @param sock Socket file descriptor
 */
void aicwf_rtos_socket_pop(int sock)
{
#ifdef CONFIG_RTOS_RESLIST
    uint16_t rsock = (uint16_t)sock << 8;
    rsock &= 0xff00;

    aicwf_rtos_res_list_rellist(NULL, rsock | RES_TYPE_SOCKET);
#endif
}

/**
 * @brief Find the port bound to a socket in the resource list
 * @param sock Socket file descriptor
 * @return Bound port number, 0=not found
 */
uint32_t aicwf_rtos_socket_find(int sock)
{
    uint32_t port = 0;

#ifdef CONFIG_RTOS_RESLIST
    OsResList *pRes;
    struct co_list *plist = &res_list[RES_TYPE_SOCKET];

    aicwf_rtos_res_mutex_lock();
    pRes = (OsResList *)co_list_pick(plist);

    while (pRes)
    {
        if (pRes->res.para0 == sock)
        {
            port = pRes->res.para1;
            break;
        }
        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }
    aicwf_rtos_res_mutex_unlock();
#endif

    return port;
}

/**
 * @brief Get task state
 * @param ref Task handle
 * @return Task state code (rt_thread_t->stat)
 */
uint32_t aicwf_rtos_get_task_status(rtos_task_handle ref)
{
    uint32_t st = ((rt_thread_t)ref)->stat;

    DBG_OSAL_INF("[rtos]%s: handle=%p st=%d\n", __func__, ref, st);

    return st;
}

/**
 * @brief Wait for a task to enter the suspended state
 *
 * Polls the task state until suspended/completed; forces suspend if not self-suspended.
 *
 * @param ref Task handle
 * @return 0
 */
uint32_t aicwf_rtos_wait_task_suspend(rtos_task_handle ref)
{
    uint32_t st;

    while (1)
    {
        st = aicwf_rtos_get_task_status(ref);
#if !RTOS_DEL_NO_WAIT
        if ((st == RT_THREAD_RUNNING) || (st == RT_THREAD_READY))
            aicwf_rtos_msleep(1);
        else
#endif
            break;
    };

    if ((st != RT_THREAD_CLOSE))
        rt_thread_suspend((rt_thread_t)ref);

    return 0;
}

/**
 * @brief Wait only for task suspension (no forced suspend)
 * @param ref Task handle
 * @return 0
 */
uint32_t aicwf_rtos_wait_task_suspend_only(rtos_task_handle ref)
{
    uint32_t st;

    while (1)
    {
        st = aicwf_rtos_get_task_status(ref);

        if (((st == RT_THREAD_READY) || (st == RT_THREAD_RUNNING)))
            aicwf_rtos_msleep(1);
        else
            break;
    };

    return 0;
}

/**
 * @brief Wait for all tracked tasks to suspend
 *
 * Iterates the RES_TYPE_TASK list, waiting for each task to suspend.
 */
void aicwf_rtos_wait_all_task_suspend(void)
{
#ifdef CONFIG_RTOS_RESLIST
    OsResList *pRes;
    struct co_list *plist;

    aicwf_rtos_res_mutex_lock();
    plist = &res_list[RES_TYPE_TASK];
    pRes = (OsResList *)co_list_pick(plist);
    aicwf_rtos_res_mutex_unlock();

    while (pRes)
    {
        if (pRes->res.ref)
        {
            aicwf_rtos_wait_task_suspend(pRes->res.ref);
        }

        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }
#endif
}

/**
 * @brief Get the currently running task handle
 * @return Current task handle
 */
rtos_task_handle aicwf_rtos_get_current_task(void)
{
    rtos_task_handle taskref;
    taskref = rt_thread_self();

    return (rtos_task_handle)taskref;
}

/**
 * @brief Get task name
 * @param ref Task handle
 * @return Task name string
 */
char *aicwf_rtos_get_task_name(void *ref)
{
    rt_thread_t th = (rt_thread_t)ref;
#if (defined(RTTHREAD_VERSION) && (RTTHREAD_VERSION >= 50002))  /* [AIC8800-PORT][SiFli] RT-Thread 3.1.3 has no RT_VERSION_CHECK macro (preprocessor parse error); RTTHREAD_VERSION encoding=major*10000+minor*100+patch, 5.0.2=50002; SiFli=30103 so the #else (th->name) branch is taken */
    return th->parent.name;
#else
    return th->name;
#endif
}

/**
 * @brief Create an RTOS task (with resource tracking)
 *
 * Creates a task and registers it in the resource tracking list.
 *
 * @param func          Task entry function
 * @param name          Task name
 * @param task_id       Task ID (unused)
 * @param stack_depth   Stack size (bytes)
 * @param params        Task parameters
 * @param prio          Priority
 * @param task_handle   Output task handle
 * @return 0=success, -3=creation failed
 */
int aicwf_rtos_task_create(rtos_task_fct func,
                           const char *const name,
                           int task_id,
                           const uint16_t stack_depth,
                           void *const params,
                           rtos_prio prio,
                           rtos_task_handle *task_handle)
{
    rt_err_t res;
    OsResSt st;
    rt_thread_t handle;
    void *stack_base = NULL;

    handle = rt_thread_create(name, func, params, stack_depth, prio, 20);
    if (handle == RT_NULL)
    {
        return -3;
    }

    res = rt_thread_startup(handle);
    if (res != RT_EOK)
    {
        return -3;
    }

    if (task_handle)
        *task_handle = (rtos_task_handle)handle;

#ifdef CONFIG_RTOS_RESLIST
    aicwf_OsResInit(&st, (char *)name, *task_handle);
    st.para0 = (uintptr_t)stack_base;
    st.para1 = stack_depth;
    st.type = RES_TYPE_TASK;

    DBG_OSAL_INF("[rtos]%s '%s', handle=%#8x, stack_base=%#8x, stack_size=%#8x\n",
                 __func__, name, *task_handle, stack_base, stack_depth);

    aicwf_rtos_res_list_push(&st);
#endif

    return 0;
}

/**
 * @brief Create an RTOS task (without resource tracking)
 *
 * Similar to task_create but not registered in the resource list.
 * The caller must ensure task_handle is not NULL.
 *
 * @return 0=success, -3=creation failed
 */
int aicwf_rtos_task_create_only(rtos_task_fct func,
                                const char *const name,
                                int task_id,
                                const uint16_t stack_depth,
                                void *const params,
                                rtos_prio prio,
                                rtos_task_handle *task_handle)
{
    rt_thread_t handle;

    AIC_ASSERT_ERR(task_handle != NULL);
    handle = rt_thread_create(name, func, params, stack_depth, prio, 20);
    if (handle == RT_NULL)
    {
        return -3;
    }

    *task_handle = (rtos_task_handle)handle;

    rt_thread_startup(handle);

    return 0;
}

/**
 * @brief Delete an RTOS task
 *
 * Deletes the specified task. RT-Thread does not allow a thread to delete
 * itself; if NULL or the current task handle is passed, only the resource
 * list entry is removed.
 *
 * @param task_handle Task handle (NULL=delete current task)
 */
void aicwf_rtos_task_delete(rtos_task_handle task_handle)
{
    rtos_task_handle hdl = task_handle;
    if ((hdl == NULL) || (hdl == aicwf_rtos_get_current_task()))
    {
        hdl = aicwf_rtos_get_current_task();
        {
            DBG_OSAL_ERR("aicwf_rtos_task_delete self\n");
            // RT-Thread: a thread cannot delete itself, only remove from resource list
            // TODO: [AIC8800-PORT] If self-delete is really needed, ask another thread to do it
#ifdef CONFIG_RTOS_RESLIST
            aicwf_rtos_res_list_rellist(task_handle, RES_TYPE_TASK);
#endif
            return;
        }
    }

#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_list_release(task_handle, RES_TYPE_TASK);
#else
    rt_thread_delete((rt_thread_t)task_handle);
#endif
}

/**
 * @brief Check whether a task is valid
 * @param task_handle Task handle
 * @return true=valid, false=invalid
 */
bool aicwf_rtos_task_isvalid(rtos_task_handle task_handle)
{
    if (task_handle == NULL)
        return false;

#ifdef CONFIG_RTOS_RESLIST
    OsResSt *res = aicwf_rtos_res_find_res(task_handle, RES_TYPE_TASK);
    return (res != NULL);
#else
    return true;
#endif
}

/**
 * @brief Suspend the task for a delay
 * @param duration Delay in milliseconds (<=0 means no delay)
 */
void aicwf_rtos_task_suspend(int duration)
{
    if (duration <= 0)
        return;

    rt_thread_mdelay(duration);
}

/**
 * @brief Resume a suspended task
 * @param task_handle Task handle
 */
void aicwf_rtos_task_resume(rtos_task_handle task_handle)
{
    rt_thread_resume((rt_thread_t)task_handle);
}

/**
 * @brief Get task priority
 * @param task_handle Task handle
 * @return Priority value
 */
uint32_t aicwf_rtos_task_get_priority(rtos_task_handle task_handle)
{
    uint8_t priority = 0;
    priority = ((rt_thread_t)task_handle)->current_priority;

    return priority;
}

/**
 * @brief Set task priority
 * @param task_handle Task handle
 * @param priority    Priority value
 */
void aicwf_rtos_task_set_priority(rtos_task_handle task_handle, uint32_t priority)
{
    rt_thread_control((rt_thread_t)task_handle, RT_THREAD_CTRL_CHANGE_PRIORITY, &priority);
}

/**
 * @brief Create a message queue
 *
 * Creates a message queue with the given element size and count; recorded in the
 * resource list when resource tracking is enabled.
 *
 * @param elt_size  Bytes per element
 * @param nb_elt    Maximum number of elements in the queue
 * @param queue     Output queue handle
 * @param name      Queue name (for debugging and resource tracking)
 * @return 0=success, -1=creation failed
 */
int aicwf_rtos_queue_create(int elt_size, int nb_elt, rtos_queue *queue, const char *const name)
{
    OsResSt st;

    AIC_ASSERT_ERR(queue != NULL);

    *queue = rt_mq_create(name, elt_size, nb_elt, RT_IPC_FLAG_FIFO);

    if (*queue == NULL)
        return -1;

#ifdef CONFIG_RTOS_RESLIST
    aicwf_OsResInit(&st, (char *)name, *queue);
    st.type = RES_TYPE_Q;
    st.para0 = nb_elt;
    st.para1 = elt_size;

#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("[rtos]%s '%s', queue=%#x\n", __func__, name, *queue);
#endif

    aicwf_rtos_res_list_push(&st);
#endif

    return 0;
}

/**
 * @brief Delete a message queue
 *
 * Releases the queue resources; removed from the list when resource tracking
 * is enabled.
 *
 * @param queue Queue handle
 */
void aicwf_rtos_queue_delete(rtos_queue queue)
{
#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_list_release(queue, RES_TYPE_Q);
#else
    rt_mq_delete((rt_mq_t)queue);
#endif
}

/**
 * @brief Check whether the queue is empty
 * @param queue Queue handle
 * @return true=empty, false=not empty
 */
bool aicwf_rtos_queue_is_empty(rtos_queue queue)
{
    rt_mq_t q = (rt_mq_t)queue;
    return (q->entry == 0);
}

/**
 * @brief Check whether the queue is full
 * @param queue Queue handle
 * @return true=full, false=not full
 */
bool aicwf_rtos_queue_is_full(rtos_queue queue)
{
    rt_mq_t q = (rt_mq_t)queue;
    return (q->entry >= q->max_msgs);
}

/**
 * @brief Get the number of pending messages in the queue
 * @param queue Queue handle
 * @return Number of messages currently in the queue (returns -1 if unsupported by RT-Thread)
 */
int aicwf_rtos_queue_cnt(rtos_queue queue)
{
    rt_mq_t q = (rt_mq_t)queue;
    return (int)q->entry;
}

/**
 * @brief Write a message to the queue
 *
 * Appends the message to the queue tail, with timeout wait (task context).
 * RT-Thread rt_mq_send does not support ISR; interrupt context returns failure.
 *
 * @param queue   Queue handle
 * @param size    Message size
 * @param msg     Pointer to the message data to send
 * @param timeout Timeout in milliseconds (only valid in task context)
 * @param isr     true=called from ISR context
 * @return 1=failure, 0=success
 */
int aicwf_rtos_queue_write(rtos_queue queue, uint32_t size, void *msg, int timeout, bool isr)
{
    rt_err_t res;

    if (queue == NULL)
    {
        return 1;
    }

    if (isr)
    {
        /* ISR context does not support waiting, use non-blocking send */
        res = rt_mq_send((rt_mq_t)queue, msg, size);
    }
    else
    {
        /* [AIC8800-PORT][SiFli] RT-Thread 3.1.3 has no rt_mq_send_wait (4.x+ only); use rt_mq_send
         * (non-blocking, -RT_EFULL when full). TODO[HW]: add retry+mdelay loop if blocking timeout is required. */
        (void)timeout;
        res = rt_mq_send((rt_mq_t)queue, msg, size);
    }

    return (res != RT_EOK);
}

/**
 * @brief Write a message to the queue from a timer callback
 *
 * Timer callbacks run in task context, same logic as aicwf_rtos_queue_write.
 *
 * @param queue   Queue handle
 * @param size    Message size
 * @param msg     Pointer to the message data to send
 * @param timeout Timeout in milliseconds
 * @param isr     true=called from ISR context
 * @return 1=failure, 0=success
 */
int aicwf_rtos_queue_write_intimer(rtos_queue queue, uint32_t size, void *msg, int timeout, bool isr)
{
    /* Timer callbacks run in task context, timeout wait is supported */
    rt_err_t res;
    (void)isr;
    (void)timeout;   /* [AIC8800-PORT][SiFli] RT-Thread 3.1.3 has no rt_mq_send_wait; see below */

    if (queue == NULL)
    {
        return 1;
    }

    /* [AIC8800-PORT][SiFli] rt_mq_send_wait(4.x+) -> rt_mq_send(non-blocking). TODO[HW]: add retry if blocking semantics are needed */
    res = rt_mq_send((rt_mq_t)queue, msg, size);

    return (res != RT_EOK);
}

/**
 * @brief Read a message from the queue
 *
 * Pops one message from the head of the queue, with timeout wait (task context).
 *
 * @param queue   Queue handle
 * @param size    Message size
 * @param msg     Output buffer for the received message
 * @param timeout Timeout in milliseconds (only valid in task context)
 * @param isr     true=called from ISR context
 * @return 1=failure (timeout/empty), 0=success
 */
int aicwf_rtos_queue_read(rtos_queue queue, uint32_t size, void *msg, int timeout, bool isr)
{
    if (queue == NULL)
    {
        return 1;
    }

    if (isr)
    {
        /* ISR context does not support waiting, use non-blocking read */
        rt_err_t res = rt_mq_recv((rt_mq_t)queue, msg, size, 0);
        return (res != RT_EOK);
    }
    else
    {
#if (defined(RTTHREAD_VERSION) && (RTTHREAD_VERSION >= 50001))  /* [AIC8800-PORT][SiFli] Same as above, 5.0.1=50001; SiFli RT-Thread 3.1.3 takes #else (rt_mq_recv returns rt_err_t) */
        /* RT-Thread 5.0.1+ : rt_mq_recv returns rt_ssize_t (>0=success) */
        rt_ssize_t ret_len = rt_mq_recv((rt_mq_t)queue, msg, size, rtos_timeout_2_tickcount(timeout));
        return (ret_len <= 0);
#else
        rt_err_t res = rt_mq_recv((rt_mq_t)queue, msg, size, rtos_timeout_2_tickcount(timeout));
        return (res != RT_EOK);
#endif
    }
}


/**
 * @brief Create a semaphore
 *
 * Created with rt_sem_create; the initial RT-Thread semaphore value is the value
 * expected by the driver.
 *
 * @param semaphore  Output semaphore handle
 * @param name       Semaphore name (for debugging and resource tracking)
 * @param max_count  Maximum count value
 * @param init_count Initial count value
 * @return 0=success, -1=creation failed
 */
int aicwf_rtos_semaphore_create(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count)
{
    OsResSt st;
    int res = -1;

    if (semaphore == NULL)
    {
        DBG_OSAL_ERR("[rtos]semaphore_create: NULL output pointer\n");
        return -1;
    }

    // RT-Thread rt_sem_create sets the initial count directly
    *semaphore = rt_sem_create(name, init_count, RT_IPC_FLAG_FIFO);
    if (*semaphore != RT_NULL)
    {
        res = 0;
    }

#ifdef CONFIG_RTOS_RESLIST
    if (res == 0)
    {
        aicwf_OsResInit(&st, (char *)name, *semaphore);
        st.type = RES_TYPE_SEM;

#if RTOS_AL_INFO_DUMP
        DBG_OSAL_INF("[rtos]%s '%s', sema=%#x\n", __func__, name, *semaphore);
#endif

        aicwf_rtos_res_list_push(&st);
    }
#endif

    return res;
}

/**
 * @brief Create a semaphore (without resource tracking)
 *
 * Same logic as aicwf_rtos_semaphore_create, but not registered in the resource list.
 *
 * @param semaphore  Output semaphore handle
 * @param name       Semaphore name
 * @param max_count  Maximum count value
 * @param init_count Initial count value
 * @return 0=success, -1=creation failed
 */
int aicwf_rtos_semaphore_create_only(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count)
{
    int res = -1;

    *semaphore = rt_sem_create(name, init_count, RT_IPC_FLAG_PRIO);

    if (*semaphore != RT_NULL)
    {
        res = 0;
    }

    return res;
}

/**
 * @brief Delete a semaphore
 *
 * Releases the semaphore resources; removed from the list when resource tracking
 * is enabled.
 *
 * @param semaphore Semaphore handle
 */
void aicwf_rtos_semaphore_delete(rtos_semaphore semaphore)
{
    if (semaphore == NULL)
        return;

#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_list_release(semaphore, RES_TYPE_SEM);
#else
    rt_sem_delete((rt_sem_t)semaphore);
#endif
}

/**
 * @brief Get the current semaphore count
 * @param semaphore Semaphore handle
 * @return Current available token count (reads rt_sem_t->value directly)
 */
int aicwf_rtos_semaphore_get_count(rtos_semaphore semaphore)
{
    if (semaphore == NULL)
        return 0;

    return (int)((rt_sem_t)semaphore)->value;
}

/**
 * @brief Get the current semaphore count (interrupt-safe version)
 * @note  In RTT, reading rt_sem_t->value is a pure read and naturally ISR safe,
 *        identical to get_count; provided only to unify the port interface
 * @param semaphore Semaphore handle
 * @return Current count value
 */
int aicwf_rtos_semaphore_get_count_fromisr(rtos_semaphore semaphore)
{
    if (semaphore == NULL)
        return 0;

    return (int)((rt_sem_t)semaphore)->value;
}

/**
 * @brief Wait for a semaphore
 *
 * Tries to acquire the semaphore, waiting up to the given timeout.
 *
 * @param semaphore Semaphore handle
 * @param timeout   Timeout in milliseconds
 * @return 1=timeout failure, 0=acquired successfully
 */
int aicwf_rtos_semaphore_wait(rtos_semaphore semaphore, int timeout)
{
    rt_err_t res = RT_EOK;

    if (semaphore == NULL)
    {
        DBG_OSAL_INF("[rtos]wait semaphore null\n");
        rt_thread_mdelay(1);   /* avoid tight spin burning CPU on NULL semaphore */
        return 1;
    }

    res = rt_sem_take((rt_sem_t)semaphore, rtos_timeout_2_tickcount(timeout));

    return (res == -RT_ETIMEOUT);
}

/**
 * @brief Release a semaphore
 *
 * Increments the semaphore count. RT-Thread rt_sem_release is ISR-safe.
 *
 * @param semaphore Semaphore handle
 * @param isr       true=called from ISR context (rt_sem_release itself is ISR-safe in RT-Thread)
 * @return 1=failure, 0=success
 */
int aicwf_rtos_semaphore_signal(rtos_semaphore semaphore, bool isr)
{
    rt_err_t res;

    if (semaphore == NULL)
    {
        DBG_OSAL_INF("[rtos]signal semaphore null\n");
        return 1;
    }

    // rt_sem_release can be called from both ISR and non-ISR context
    res = rt_sem_release((rt_sem_t)semaphore);
    (void)isr;

    return (res != RT_EOK);
}

/**
 * @brief Create a software timer
 *
 * Initializes the timer resource; registered in the resource list when resource
 * tracking is enabled. The actual RT-Thread timer object is created lazily on start.
 *
 * @param timer Output timer handle
 * @param name  Timer name (for debugging)
 * @return 0=success
 */
int aicwf_rtos_timer_create(rtos_timer *timer, char *name)
{
    int ret = 0;

    *timer = NULL;

#ifdef CONFIG_RTOS_RESLIST
    if (ret == 0)
    {
        OsResSt st;
        aicwf_OsResInit(&st, (char *)name, *timer);
        rtos_res_time_cnt++;
        st.type = RES_TYPE_TIME;
        *timer = (rtos_timer)(aicwf_rtos_res_list_push(&st));
        DBG_OSAL_INF("create timer %s, timer=%#x\n", name, *timer);
    }
#endif

    return 0;
}

/**
 * @brief Start a software timer
 *
 * Creates the actual timer object lazily if not yet created, then starts it.
 * RT-Thread emulates it with a one-shot timer.
 *
 * @param timer           Timer handle
 * @param initialTime     Initial delay (unused, period is used directly as delay)
 * @param rescheduleTime  Timer period (milliseconds)
 * @param func            Timer callback function
 * @param args            Callback argument
 * @return 0=success, -1=failure
 */
int aicwf_rtos_timer_start(rtos_timer timer, uint32_t initialTime, uint32_t rescheduleTime, rtos_timer_fct func, void *const args)
{
    rt_err_t res = RT_EOK;
    rt_timer_t xTimer = NULL;

#ifdef CONFIG_RTOS_RESLIST
    OsResList *p = (OsResList *)timer;
    if (p->res.ref == RT_NULL)
    {
        DBG_OSAL_INF("aicwf_rtos_timer_start %s\n", p->res.name);
        xTimer = rt_timer_create(p->res.name, (void (*)(void *))func, args,
                                 rtos_timeout_2_tickcount(rescheduleTime), RT_TIMER_FLAG_ONE_SHOT);
        if (xTimer == RT_NULL)
        {
            DBG_OSAL_ERR("%s: create timer %s fail\n", __func__, p->res.name);
            return -1;
        }
        p->res.ref = xTimer;
    }
    else
    {
        xTimer = (rt_timer_t)p->res.ref;
    }
#else
    xTimer = (rt_timer_t)timer;
#endif
    res = rt_timer_start(xTimer);
    if (res != RT_EOK)
    {
        DBG_OSAL_ERR("%s: start timer %s fail\n", __func__, res == RT_EOK ? "ok" : "fail");
        return -1;
    }

    return (res != RT_EOK);
}

/**
 * @brief Get timer state
 *
 * @param timer        Timer handle
 * @param timer_status Output timer state
 * @return Always returns AIC_RTOS_TIMER_STATUS_ACTIVE
 */
int aicwf_rtos_timer_get_status(rtos_timer timer, rtos_timer_status *timer_status)
{
    (void)timer;
    (void)timer_status;
    return AIC_RTOS_TIMER_STATUS_ACTIVE;
}

/**
 * @brief Stop a software timer
 *
 * @param timer Timer handle
 * @return 0=success, -1=failure or invalid handle
 */
int aicwf_rtos_timer_stop(rtos_timer timer)
{
    if (timer == NULL)
    {
        DBG_OSAL_INF("%s: NULL timer\n", __func__);
        return -1;
    }
    rt_timer_t xTimer = NULL;
#ifdef CONFIG_RTOS_RESLIST
    OsResList *p = (OsResList *)timer;
    if (p->res.ref != NULL)
    {
        xTimer = (rt_timer_t)p->res.ref;
    }
    else
    {
        DBG_OSAL_ERR("%s: invalid timer %s\n", __func__, p->res.name);
        return -1;
    }
#else
    xTimer = (rt_timer_t)timer;
#endif

    rt_err_t res = RT_EOK;
    res = rt_timer_stop(xTimer);

    if (res != RT_EOK)
    {
        DBG_OSAL_ERR("%s: stop timer fail\n", __func__);
        return -1;
    }

    return (res != RT_EOK);
}

/**
 * @brief Stop a software timer in ISR
 *
 * @param timer Timer handle
 * @return 0=success, -1=failure or invalid handle
 */
int aicwf_rtos_timer_stop_isr(rtos_timer timer)
{
    rt_err_t res = RT_EOK;

    if (timer == NULL)
    {
        DBG_OSAL_INF("%s: NULL timer\n", __func__);
        return -1;
    }

    rt_timer_t xTimer = NULL;
#ifdef CONFIG_RTOS_RESLIST
    OsResList *p = (OsResList *)timer;
    if (p->res.ref != NULL)
    {
        xTimer = (rt_timer_t)p->res.ref;
    }
#endif
    (void)timer;

    res = rt_timer_stop(xTimer);
    return (res != RT_EOK);
}

/**
 * @brief Delete a software timer
 *
 * Stops and releases the timer resources; removed from the list when resource
 * tracking is enabled.
 *
 * @param timer Timer handle
 * @return 0=success, -1=invalid handle
 */
int aicwf_rtos_timer_delete(rtos_timer timer)
{
    if (timer == NULL)
    {
        DBG_OSAL_INF("%s: NULL timer\n", __func__);
        return -1;
    }
    rt_timer_t xTimer = NULL;

#ifdef CONFIG_RTOS_RESLIST
    OsResList *p = (OsResList *)timer;
    if (p->res.ref != NULL)
    {
        xTimer = (rt_timer_t)p->res.ref;
        rt_timer_delete(xTimer);
        aicwf_rtos_res_list_release(xTimer, RES_TYPE_TIME);
    }
    else
    {
        uint16_t type = p->res.para0 << 8 | RES_TYPE_TIME;
        aicwf_rtos_res_list_release(xTimer, type);
    }
#else
    // timer may already be an rt_timer_t handle
    xTimer = (rt_timer_t)timer;
    rt_timer_delete(xTimer);
#endif

    return 0;
}

/**
 * @brief Create a mutex
 *
 * Creates a mutex; recorded in the resource list when resource tracking is enabled.
 *
 * @param mutex Output mutex handle
 * @param name  Mutex name (for debugging and resource tracking)
 * @return 0=success, -1=creation failed
 */
int aicwf_rtos_mutex_create(rtos_mutex *mutex, const char *const name)
{
    OsResSt st;

    int ret = -1;

    *mutex = rt_mutex_create(name, RT_IPC_FLAG_PRIO);
    if (*mutex != 0)
    {
        ret = 0;
    }

#ifdef CONFIG_RTOS_RESLIST
    if (ret == 0)
    {
        aicwf_OsResInit(&st, (char *)name, *mutex);
        st.type = RES_TYPE_MUTEX;

#if RTOS_AL_INFO_DUMP
        DBG_OSAL_INF("[rtos]%s '%s', mutex=%#x\n", __func__, name, *mutex);
#endif

        aicwf_rtos_res_list_push(&st);
    }
#endif

    return ret;
}

/**
 * @brief Create a mutex (without resource tracking)
 *
 * Same logic as aicwf_rtos_mutex_create, but not registered in the resource list.
 *
 * @param mutex Output mutex handle
 * @param name  Mutex name
 * @return 0=success, -1=creation failed
 */
int aicwf_rtos_mutex_create_only(rtos_mutex *mutex, const char *const name)
{

    int ret = -1;

    *mutex = rt_mutex_create(name, RT_IPC_FLAG_PRIO);
    if (*mutex != 0)
    {
        ret = 0;
    }

    return ret;
}

/**
 * @brief Delete a mutex
 *
 * Releases the mutex resources; removed from the list when resource tracking
 * is enabled.
 *
 * @param mutex Mutex handle
 */
void aicwf_rtos_mutex_delete(rtos_mutex mutex)
{
#ifdef CONFIG_RTOS_RESLIST
    aicwf_rtos_res_list_release(mutex, RES_TYPE_MUTEX);
#else
    rt_mutex_delete((rt_mutex_t)mutex);
#endif
}

/**
 * @brief Lock a mutex
 *
 * @param mutex   Mutex handle
 * @param timeout Timeout in milliseconds
 * @return 0=success, -1=NULL handle, 1=timeout failure
 */
int aicwf_rtos_mutex_lock(rtos_mutex mutex, int timeout)
{
    if (mutex == NULL)
    {
        DBG_OSAL_INF("[rtos]lock mutex null\n");
        return -1;
    }
    rt_err_t res = RT_EOK;

    res = rt_mutex_take((rt_mutex_t)mutex, rtos_timeout_2_tickcount(timeout));

    return (res != RT_EOK);
}

/**
 * @brief Unlock a mutex
 *
 * @param mutex Mutex handle
 * @return 0=success, -1=NULL handle, 1=failure
 */
int aicwf_rtos_mutex_unlock(rtos_mutex mutex)
{
    if (mutex == NULL)
    {
        DBG_OSAL_INF("[rtos]unlock mutex null\n");
        return -1;
    }

    rt_err_t res = RT_EOK;

    res = rt_mutex_release((rt_mutex_t)mutex);

    return (res != RT_EOK);
}

/**
 * @brief Create an event group
 *
 * @param event_group Output event group handle
 * @return 0=success, -1=creation failed
 */
int aicwf_rtos_event_group_create(rtos_event_group *event_group)
{
    int res = -1;

    *event_group = rt_event_create("ev_grp", RT_IPC_FLAG_FIFO);
    if (*event_group != 0)
    {
        res = 0;
    }

    return res;
}

/**
 * @brief Delete an event group
 * @param event_group Event group handle
 */
void aicwf_rtos_event_group_delete(rtos_event_group event_group)
{
    if (event_group)
        rt_event_delete((rt_event_t)event_group);
}

/**
 * @brief Get the current event group bit flags
 *
 * @param event_group Event group handle
 * @param isr         true=called from ISR context (rt_event_recv does not support ISR, but timeout=0 only snapshots the current value)
 * @return Current event bit flag value
 */
uint32_t aicwf_rtos_event_group_get_bits(rtos_event_group event_group, bool isr)
{
    uint32_t recved = 0;
    (void)isr;

    if (event_group)
    {
        rt_event_recv((rt_event_t)event_group, 0xFFFFFFFF,
                      RT_EVENT_FLAG_OR, 0, &recved);
    }
    return recved;
}

/**
 * @brief Wait for specified event group bit flags
 *
 * @param event_group    Event group handle
 * @param val            Event bit mask to wait for
 * @param clear_on_exit  true=clear matching bits automatically on exit
 * @param wait_all_bits  true=wait for all bits set, false=any bit set is enough
 * @param timeout        Timeout in milliseconds
 * @return Event bit flag value on return
 */
uint32_t aicwf_rtos_event_group_wait_bits(rtos_event_group event_group, const uint32_t val,
        const bool clear_on_exit, const bool wait_all_bits, int timeout)
{
    if (!event_group)
        return 0;

    rt_uint8_t opt = 0;
    uint32_t recved = 0;
    rt_err_t res;

    if (wait_all_bits)
        opt |= RT_EVENT_FLAG_AND;
    else
        opt |= RT_EVENT_FLAG_OR;

    if (clear_on_exit)
        opt |= RT_EVENT_FLAG_CLEAR;

    res = rt_event_recv((rt_event_t)event_group, val, opt,
                        rtos_timeout_2_tickcount(timeout), &recved);

    if (res == RT_EOK)
        return recved;
    return 0;
}

/**
 * @brief Set event group bit flags
 *
 * @param event_group Event group handle
 * @param val         Bit mask to set
 * @param isr         true=called from ISR context
 * @return Event bit flag value after setting
 */
uint32_t aicwf_rtos_event_group_set_bits(rtos_event_group event_group, const uint32_t val, bool isr)
{
    if (event_group)
    {
        rt_event_send((rt_event_t)event_group, val);
        return aicwf_rtos_event_group_get_bits(event_group, isr);
    }
    return 0;
}


/**
 * @brief Get current time
 *
 * @param origin Time origin (unused, always uses tick-based time)
 * @param sec    Output seconds
 * @param usec   Output microseconds
 * @return 0=success
 */
int aicwf_time_get(enum time_origin_t origin, uint32_t *sec, uint32_t *usec)
{
    rt_tick_t tick;
    tick = rt_tick_get();

    *sec = tick / RT_TICK_PER_SECOND;
    *usec = (tick % RT_TICK_PER_SECOND) * (1000000 / RT_TICK_PER_SECOND);

    return 0;
}

/**
 * @brief Initialize the random number generator
 *
 * Uses the current timestamp as the random seed (requires CONFIG_MBEDTLS_V3).
 */
void aicwf_random_init(void)
{
#if defined(CONFIG_MBEDTLS_V3)
    srand((unsigned int)aicwf_rtos_now(0));
#endif
}

/**
 * @brief Generate a random byte
 *
 * @return Random value in 0~255 (always returns 0 when CONFIG_MBEDTLS_V3 is disabled)
 */
uint8_t aicwf_random_byte(void)
{
#if defined(CONFIG_MBEDTLS_V3)
    int min = 0, max = 255;
    return rand() % (max - min + 1) + min;
#else
    return 0;
#endif
}

/* ============================================================
 * Missing RTOS functions - stub implementations
 * ============================================================ */

/**
 * @brief Initialize the task notification mechanism (stub)
 *
 * @param task Task handle
 * @return 0 (always success, currently an empty implementation)
 */
int aicwf_rtos_task_init_notification(rtos_task_handle task)
{
    (void)task;
    return 0;
}

/**
 * @brief Wait for a task notification (stub)
 *
 * @param timeout Timeout in milliseconds
 * @return 0 (currently an empty implementation)
 */
uint32_t aicwf_rtos_task_wait_notification(int timeout)
{
    (void)timeout;
    return 0;
}

/**
 * @brief Send a task notification (stub)
 *
 * @param task_handle Target task handle
 * @param value       Notification value
 * @param isr         true=called from ISR context
 */
void aicwf_rtos_task_notify(rtos_task_handle task_handle, uint32_t value, bool isr)
{
    (void)task_handle;
    (void)value;
    (void)isr;
}

/**
 * @brief Set task notification bits (stub)
 *
 * @param task_handle Target task handle
 * @param value       Notification bit mask
 * @param isr         true=called from ISR context
 */
void aicwf_rtos_task_notify_setbits(rtos_task_handle task_handle, uint32_t value, bool isr)
{
    (void)task_handle;
    (void)value;
    (void)isr;
}

/**
 * @brief Enter an ISR-safe critical section
 *
 * Short critical section protection for queues shared by USB URB complete (ISR) and tasks.
 * Returns the interrupt mask before entry; pass it to aicwf_rtos_critical_exit to restore.
 * RT-Thread: rt_hw_interrupt_disable disables global interrupts, safe in task and ISR context.
 *
 * @return Interrupt mask before entering the critical section (rt_base_t)
 */
uint32_t aicwf_rtos_critical_enter(void)
{
    return (uint32_t)rt_hw_interrupt_disable();
}

/**
 * @brief Exit an ISR-safe critical section
 *
 * @param level Interrupt mask returned by aicwf_rtos_critical_enter
 */
void aicwf_rtos_critical_exit(uint32_t level)
{
    rt_hw_interrupt_enable((rt_base_t)level);
}

/**
 * @brief Print memory resource list information (stub)
 *
 * Currently an empty implementation, reserved for debugging resource usage.
 */
void aicwf_rtos_mem_list_info(void)
{
    /* No-op stub */
}
