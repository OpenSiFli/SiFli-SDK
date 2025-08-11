/** @file eloop_freertos.c
 *
 *  @brief  This file provides Event loop based on freertos APIs.
 *
 */

/*
 * Copyright (c) 2002-2009, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"
#include <assert.h>
#include "fsl_os_abstraction.h"
#include "rtthread.h"

#include "utils/common.h"
#include "trace.h"
#include "dl_list.h"
#include "eloop.h"
#include "block_alloc.h"
#include "supp_main.h"

#define UNUSED __attribute__((__unused__))

#define MAX_ELOOP_TIMEOUTS 100

extern OSA_EVENT_HANDLE_DEFINE(supplicant_event_Handle);

#ifdef __ZEPHYR__
#define portMAX_DELAY (uint32_t)0xffffffffUL
#endif

struct eloop_sock
{
    int sock;
    void *eloop_data;
    void *user_data;
    eloop_sock_handler handler;
};

struct eloop_timeout
{
    struct os_time time;
    void *eloop_data;
    void *user_data;
    eloop_timeout_handler handler;
    struct eloop_timeout *next;
};

struct eloop_signal
{
    int sig;
    void *user_data;
    eloop_signal_handler handler;
    int signaled;
};

struct eloop_sock_table
{
    int count;
    struct eloop_sock *table;
    int changed;
};

struct eloop_data
{
    int max_sock;

    struct eloop_sock_table readers;
    struct eloop_sock_table writers;
    struct eloop_sock_table exceptions;

    struct eloop_timeout *timeout;

    int signal_count;
    struct eloop_signal *signals;
    int signaled;
    int pending_terminate;

    int terminate;
    int reader_table_changed;
};

static struct eloop_data eloop;

static u8 eloop_init_done = 0;

static struct pool_info *eloop_pool;
static struct eloop_timeout *eloop_timeout_pool_buf;

void eloop_alloc_deinit(void);

int eloop_alloc_init()
{
    eloop_timeout_pool_buf = os_zalloc(MAX_ELOOP_TIMEOUTS * sizeof(struct eloop_timeout));
    if (!eloop_timeout_pool_buf)
    {
        goto err;
    }
    eloop_pool = ba_pool_create(sizeof(struct eloop_timeout), eloop_timeout_pool_buf,
                                MAX_ELOOP_TIMEOUTS * sizeof(struct eloop_timeout));
    if (!eloop_pool)
    {
        goto err;
    }

    return 0;
err:
    eloop_alloc_deinit();
    return -1;
}

void eloop_alloc_deinit(void)
{
    if (eloop_pool)
    {
        ba_pool_delete(eloop_pool);
        eloop_pool = NULL;
    }
    if (eloop_timeout_pool_buf)
    {
        os_free(eloop_timeout_pool_buf);
        eloop_timeout_pool_buf = NULL;
    }
}

static struct eloop_timeout *eloop_alloc()
{
    struct eloop_timeout *ptr = (struct eloop_timeout *)ba_block_allocate(eloop_pool);

    if (ptr != NULL)
    {
        os_memset(ptr, 0x00, sizeof(struct eloop_timeout));
    }

    return ptr;
}

static void eloop_free(struct eloop_timeout *et)
{
    ba_block_release(eloop_pool, (void *)et);
}

int eloop_init(void)
{
    if (eloop_init_done)
        return 0;

    memset(&eloop, 0, sizeof(eloop));

    if (eloop_alloc_init() < 0)
    {
        wpa_printf(MSG_ERROR, "%s: eloop alloc init failed: %s", __func__, strerror(errno));
        return -1;
    }

    eloop_init_done = 1;

    return 0;
}

static int eloop_sock_table_add_sock(
    struct eloop_sock_table *table, int sock, eloop_sock_handler handler, void *eloop_data, void *user_data)
{
    struct eloop_sock *tmp;

    if (table == NULL)
        return -EINVAL;

    tmp = (struct eloop_sock *)os_realloc(table->table, (table->count + 1) * sizeof(struct eloop_sock));

    if (tmp == NULL)
        return -ENOMEM;

    tmp[table->count].sock       = sock;
    tmp[table->count].eloop_data = eloop_data;
    tmp[table->count].user_data  = user_data;
    tmp[table->count].handler    = handler;
    table->count++;
    table->table = tmp;
    if (sock > eloop.max_sock)
        eloop.max_sock = sock;
    table->changed = 1;

    return 0;
}

static void eloop_sock_table_remove_sock(struct eloop_sock_table *table, int sock)
{
    int i;

    if (table == NULL || table->table == NULL || table->count == 0)
        return;

    for (i = 0; i < table->count; i++)
    {
        if (table->table[i].sock == sock)
            break;
    }
    if (i == table->count)
        return;
    if (i != table->count - 1)
    {
        os_memmove(&table->table[i], &table->table[i + 1], (table->count - i - 1) * sizeof(struct eloop_sock));
    }
    table->count--;
    table->changed = 1;
}

#if 0
static void eloop_sock_table_set_fds(struct eloop_sock_table *table, fd_set *fds)
{
    int i;

    FD_ZERO(fds);

    if (table->table == NULL)
        return;

    for (i = 0; i < table->count; i++)
        FD_SET(table->table[i].sock, fds);
}

static void eloop_sock_table_dispatch(struct eloop_sock_table *table, fd_set *fds)
{
    int i;

    if (table == NULL || table->table == NULL)
        return;

    table->changed = 0;
    for (i = 0; i < table->count; i++)
    {
        if (FD_ISSET(table->table[i].sock, fds))
        {
            table->table[i].handler(table->table[i].sock, table->table[i].eloop_data, table->table[i].user_data);
            if (table->changed)
                break;
        }
    }
}
#endif

int eloop_sock_requeue(void)
{
    return 0;
}

static void eloop_sock_table_destroy(struct eloop_sock_table *table)
{
    if (table)
    {
        size_t i;

        for (i = 0; i < table->count && table->table; i++)
        {
            wpa_printf(MSG_INFO,
                       "ELOOP: remaining socket: "
                       "sock=%d eloop_data=%p user_data=%p "
                       "handler=%p",
                       table->table[i].sock, table->table[i].eloop_data, table->table[i].user_data,
                       table->table[i].handler);
        }
        os_free(table->table);
    }
}

int eloop_register_read_sock(int sock, eloop_sock_handler handler, void *eloop_data, void *user_data)
{
    return eloop_register_sock(sock, EVENT_TYPE_READ, handler, eloop_data, user_data);
}

void eloop_unregister_read_sock(int sock)
{
    eloop_unregister_sock(sock, EVENT_TYPE_READ);
}

static struct eloop_sock_table *eloop_get_sock_table(eloop_event_type type)
{
    switch (type)
    {
        case EVENT_TYPE_READ:
            return &eloop.readers;
        case EVENT_TYPE_WRITE:
            return &eloop.writers;
        case EVENT_TYPE_EXCEPTION:
            return &eloop.exceptions;
    }

    return NULL;
}

int eloop_register_sock(int sock, eloop_event_type type, eloop_sock_handler handler, void *eloop_data, void *user_data)
{
    struct eloop_sock_table *table;

    table = eloop_get_sock_table(type);
    return eloop_sock_table_add_sock(table, sock, handler, eloop_data, user_data);
}

void eloop_unregister_sock(int sock, eloop_event_type type)
{
    struct eloop_sock_table *table;

    table = eloop_get_sock_table(type);
    eloop_sock_table_remove_sock(table, sock);
}

int eloop_register_timeout(
    unsigned int secs, unsigned int usecs, eloop_timeout_handler handler, void *eloop_data, void *user_data)
{
    struct eloop_timeout *timeout, *tmp, *prev = NULL;

    timeout = eloop_alloc();
    if (timeout == NULL)
        return -1;
    os_get_time(&timeout->time);
    timeout->time.sec += secs;
    timeout->time.usec += usecs;
    while (timeout->time.usec >= 1000000)
    {
        timeout->time.sec++;
        timeout->time.usec -= 1000000;
    }
    timeout->eloop_data = eloop_data;
    timeout->user_data  = user_data;
    timeout->handler    = handler;
    timeout->next       = NULL;

    if (eloop.timeout == NULL)
    {
        eloop.timeout = timeout;
        goto done;
    }

    prev = NULL;
    tmp  = eloop.timeout;
    while (tmp != NULL)
    {
        if (os_time_before(&timeout->time, &tmp->time))
            break;
        prev = tmp;
        tmp  = tmp->next;
    }

    if (prev == NULL)
    {
        timeout->next = eloop.timeout;
        eloop.timeout = timeout;
    }
    else
    {
        timeout->next = prev->next;
        prev->next    = timeout;
    }

done:
    if ((secs == 0) || (prev == NULL))
    {
        send_wpa_supplicant_dummy_event();
    }
    return 0;
}

static void eloop_remove_timeout(struct eloop_timeout *timeout)
{
    eloop_free(timeout);
}

int eloop_cancel_timeout(eloop_timeout_handler handler, void *eloop_data, void *user_data)
{
    struct eloop_timeout *timeout, *prev, *next;
    int removed = 0;

    prev    = NULL;
    timeout = eloop.timeout;
    while (timeout != NULL)
    {
        next = timeout->next;

        if (timeout->handler == handler && (timeout->eloop_data == eloop_data || eloop_data == ELOOP_ALL_CTX) &&
            (timeout->user_data == user_data || user_data == ELOOP_ALL_CTX))
        {
            if (prev == NULL)
                eloop.timeout = next;
            else
                prev->next = next;
            eloop_remove_timeout(timeout);
            removed++;
        }
        else
            prev = timeout;

        timeout = next;
    }

    return removed;
}

int eloop_cancel_timeout_one(eloop_timeout_handler handler,
                             void *eloop_data,
                             void *user_data,
                             struct os_reltime *remaining)
{
    struct eloop_timeout *timeout, *prev, *next;
    int removed = 0;
    struct os_reltime now;

    os_get_reltime(&now);
    remaining->sec = remaining->usec = 0;

    prev    = NULL;
    timeout = eloop.timeout;
    while (timeout != NULL)
    {
        next = timeout->next;

        if (timeout->handler == handler && (timeout->eloop_data == eloop_data) && (timeout->user_data == user_data))
        {
            if (prev == NULL)
                eloop.timeout = next;
            else
                prev->next = next;
            removed = 1;
            if (os_reltime_before(&now, (struct os_reltime *)&timeout->time))
                os_reltime_sub((struct os_reltime *)&timeout->time, &now, remaining);
            eloop_remove_timeout(timeout);
            break;
        }
        else
            prev = timeout;

        timeout = next;
    }
    return removed;
}

int eloop_is_timeout_registered(eloop_timeout_handler handler, void *eloop_data, void *user_data)
{
    struct eloop_timeout *timeout;

    timeout = eloop.timeout;
    while (timeout != NULL)
    {
        if (timeout->handler == handler && (timeout->eloop_data == eloop_data) && (timeout->user_data == user_data))
        {
            return 1;
        }

        timeout = timeout->next;
    }

    return 0;
}

int eloop_deplete_timeout(
    unsigned int req_secs, unsigned int req_usecs, eloop_timeout_handler handler, void *eloop_data, void *user_data)
{
    struct os_reltime now, requested, remaining;
    struct eloop_timeout *tmp;

    tmp = eloop.timeout;
    while (tmp != NULL)
    {
        if (tmp->handler == handler && tmp->eloop_data == eloop_data && tmp->user_data == user_data)
        {
            requested.sec  = req_secs;
            requested.usec = req_usecs;
            os_get_reltime(&now);
            os_reltime_sub((struct os_reltime *)&tmp->time, &now, &remaining);
            if (os_reltime_before(&requested, &remaining))
            {
                eloop_cancel_timeout(handler, eloop_data, user_data);
                eloop_register_timeout(requested.sec, requested.usec, handler, eloop_data, user_data);
                return 1;
            }
            return 0;
        }
        tmp = tmp->next;
    }

    return -1;
}

int eloop_replenish_timeout(
    unsigned int req_secs, unsigned int req_usecs, eloop_timeout_handler handler, void *eloop_data, void *user_data)
{
    struct os_reltime now, requested, remaining;
    struct eloop_timeout *tmp;

    tmp = eloop.timeout;
    while (tmp != NULL)
    {
        if (tmp->handler == handler && tmp->eloop_data == eloop_data && tmp->user_data == user_data)
        {
            requested.sec  = req_secs;
            requested.usec = req_usecs;
            os_get_reltime(&now);
            os_reltime_sub((struct os_reltime *)&tmp->time, &now, &remaining);
            if (os_reltime_before(&remaining, &requested))
            {
                eloop_cancel_timeout(handler, eloop_data, user_data);
                eloop_register_timeout(requested.sec, requested.usec, handler, eloop_data, user_data);
                return 1;
            }
            return 0;
        }
        tmp = tmp->next;
    }

    return -1;
}

#if !(defined(CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || defined(CONFIG_FREERTOS))
static void eloop_handle_alarm(int sig)
{
    fprintf(stderr,
            "eloop: could not process SIGINT or SIGTERM in two "
            "seconds. Looks like there\n"
            "is a bug that ends up in a busy loop that "
            "prevents clean shutdown.\n"
            "Killing program forcefully.\n"
            "sig is %d.\n",
            sig);
    exit(1);
}
#endif

#if 0
static void eloop_handle_signal(int sig)
{
    int i;

#if !(defined(CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || defined(CONFIG_FREERTOS))
    if ((sig == SIGINT || sig == SIGTERM) && !eloop.pending_terminate)
    {
        /* Use SIGALRM to break out from potential busy loops that
         * would not allow the program to be killed. */
        eloop.pending_terminate = 1;
        signal(SIGALRM, eloop_handle_alarm);
        alarm(2);
    }
#endif
    eloop.signaled++;
    for (i = 0; i < eloop.signal_count; i++)
    {
        if (eloop.signals[i].sig == sig)
        {
            eloop.signals[i].signaled++;
            break;
        }
    }
}

static void eloop_process_pending_signals(void)
{
    int i;

    if (eloop.signaled == 0)
        return;
    eloop.signaled = 0;

    if (eloop.pending_terminate)
    {
#if !(defined(CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || defined(CONFIG_FREERTOS))
        alarm(0);
#endif
        eloop.pending_terminate = 0;
    }

    for (i = 0; i < eloop.signal_count; i++)
    {
        if (eloop.signals[i].signaled)
        {
            eloop.signals[i].signaled = 0;
            eloop.signals[i].handler(eloop.signals[i].sig, eloop.signals[i].user_data);
        }
    }
}
#endif

int eloop_register_signal(int sig, eloop_signal_handler handler, void *user_data)
{
    struct eloop_signal *tmp;

    tmp = (struct eloop_signal *)os_realloc(eloop.signals, (eloop.signal_count + 1) * sizeof(struct eloop_signal));
    if (tmp == NULL)
        return -1;

    tmp[eloop.signal_count].sig       = sig;
    tmp[eloop.signal_count].user_data = user_data;
    tmp[eloop.signal_count].handler   = handler;
    tmp[eloop.signal_count].signaled  = 0;
    eloop.signal_count++;
    eloop.signals = tmp;
#if !(defined(CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || defined(CONFIG_FREERTOS))
    signal(sig, eloop_handle_signal);
#endif

    return 0;
}

int eloop_register_signal_terminate(eloop_signal_handler handler, void *user_data)
{
#if !(defined(CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || defined(CONFIG_FREERTOS))

    int ret = eloop_register_signal(SIGINT, handler, user_data);
    if (ret == 0)
        ret = eloop_register_signal(SIGTERM, handler, user_data);
    return ret;
#else
    return 0;
#endif
}

int eloop_register_signal_reconfig(eloop_signal_handler handler, void *user_data)
{
#if !(defined(CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || defined(CONFIG_FREERTOS))
    return eloop_register_signal(SIGHUP, handler, user_data);
#else
    return 0;
#endif
}

#ifdef __ZEPHYR__
extern struct k_event suppMainTaskEvent;
#endif
void eloop_run(void)
{
    //fd_set *rfds, *wfds, *efds;
    struct timeval _tv;
    struct os_time tv, now;
    uint32_t timeout_ms = 0U;
#ifdef __ZEPHYR__
    uint32_t taskNotification = 0U;
#else
    osa_event_flags_t taskNotification;
#endif
#if 0
    rfds = os_malloc(sizeof(*rfds));
    wfds = os_malloc(sizeof(*wfds));
    efds = os_malloc(sizeof(*efds));
    if (rfds == NULL || wfds == NULL || efds == NULL)
    {
        PRINTF("eloop_run - os_malloc failed\r\n");
        goto out;
    }
#endif

    while (!eloop.terminate &&
           (eloop.timeout || eloop.readers.count > 0 || eloop.writers.count > 0 || eloop.exceptions.count > 0))
    {
        timeout_ms = 0U;
        taskNotification = 0U;
        if (eloop.timeout)
        {
            os_get_time(&now);
            if (os_time_before(&now, &eloop.timeout->time))
                os_time_sub(&eloop.timeout->time, &now, &tv);
            else
                tv.sec = tv.usec = 0;
            _tv.tv_sec  = tv.sec;
            _tv.tv_usec = tv.usec;

            timeout_ms = _tv.tv_sec * 1000 + _tv.tv_usec / 1000;
        }

#if 0
        eloop_sock_table_set_fds(&eloop.readers, rfds);
        eloop_sock_table_set_fds(&eloop.writers, wfds);
        eloop_sock_table_set_fds(&eloop.exceptions, efds);
        res = select(eloop.max_sock + 1, rfds, wfds, efds, eloop.timeout ? &_tv : NULL);
        if (res < 0 && errno != EINTR && errno != 0)
        {
            perror("select");
            goto out;
        }
        eloop_process_pending_signals();
#endif

#ifdef __ZEPHYR__
        taskNotification = k_event_wait(&suppMainTaskEvent, (1U << DUMMY) | (1U << EVENT), false,
            eloop.timeout ? K_MSEC(timeout_ms): K_FOREVER);
        k_event_clear(&suppMainTaskEvent, 0xFF);
#else
        /* Wait till we receive supplicant event */
        (void)OSA_EventWait((osa_event_handle_t)supplicant_event_Handle, (1U << DUMMY) | (1U << EVENT), false, eloop.timeout ? timeout_ms : osaWaitForever_c,
                            &taskNotification);
#endif

        /* check if some registered timeouts have occurred */
        if (eloop.timeout)
        {
            struct eloop_timeout *tmp;

            os_get_time(&now);
            if (!os_time_before(&now, &eloop.timeout->time))
            {
                tmp           = eloop.timeout;
                eloop.timeout = eloop.timeout->next;
                tmp->handler(tmp->eloop_data, tmp->user_data);
                eloop_remove_timeout(tmp);
            }
        }

        if ((taskNotification == 0) || (taskNotification == (1U << DUMMY)))
        {
            continue;
        }

        if (taskNotification & (1U << EVENT))
        {
            process_wpa_supplicant_event();
        }

#if 0
        eloop_sock_table_dispatch(&eloop.readers, rfds);
        eloop_sock_table_dispatch(&eloop.writers, wfds);
        eloop_sock_table_dispatch(&eloop.exceptions, efds);
#endif
    }

#if 0
out:
    os_free(rfds);
    os_free(wfds);
    os_free(efds);
#endif
}

void eloop_terminate(void)
{
    eloop.terminate = 1;
}

void eloop_destroy(void)
{
    struct eloop_timeout *timeout, *prev;

    timeout = eloop.timeout;
    while (timeout != NULL)
    {
        prev    = timeout;
        timeout = timeout->next;
        eloop_remove_timeout(prev);
    }
    eloop_sock_table_destroy(&eloop.readers);
    eloop_sock_table_destroy(&eloop.writers);
    eloop_sock_table_destroy(&eloop.exceptions);
    os_free(eloop.signals);

    eloop_alloc_deinit();

    eloop_init_done = 0;
}

int eloop_terminated(void)
{
    return eloop.terminate;
}

void eloop_wait_for_read_sock(int sock)
{
    fd_set rfds;

    if (sock < 0)
        return;

    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    select(sock + 1, &rfds, NULL, NULL, NULL);
}
