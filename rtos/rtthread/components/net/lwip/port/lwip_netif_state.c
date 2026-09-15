/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <lwip/opt.h>
#include <lwip/sys.h>
#include <lwip/timeouts.h>
#include <lwip/init.h>
#include <lwip/tcpip.h>
#include "lwip_netif_state.h"

#if defined(RT_USING_LWIP_VER_NUM) && (RT_USING_LWIP_VER_NUM >= 0x20102)

struct lwip_netif_state
{
    rt_uint32_t netif_up_count;
#if LWIP_TIMERS
    rt_bool_t timeout_inited;
    rt_bool_t timeout_init_pending;
#endif
};
static struct lwip_netif_state g_lwip_netif_state;
void lwip_netif_state_on_timeout_init(void)
{
#if LWIP_TIMERS
    g_lwip_netif_state.timeout_inited = RT_TRUE;
    // rt_kprintf("[lwip_netif] timeout init\n");
#endif
}
void lwip_netif_state_on_timeout_uninit(void)
{
#if LWIP_TIMERS
    g_lwip_netif_state.timeout_inited = RT_FALSE;
    // rt_kprintf("[lwip_netif] timeout uninit\n");
#endif
}
int lwip_netif_state_timeout_is_init(void)
{
#if LWIP_TIMERS
    return g_lwip_netif_state.timeout_inited ? 1 : 0;
#else
    return 0;
#endif
}
#if LWIP_TIMERS
static void lwip_netif_state_timeout_init_callback(void *arg)
{
    SYS_ARCH_DECL_PROTECT(lev);
    LWIP_UNUSED_ARG(arg);
    rt_kprintf("[lwip_netif] tcpip thread: init timeouts\n");
    sys_timeouts_init();
    SYS_ARCH_PROTECT(lev);
    g_lwip_netif_state.timeout_init_pending = RT_FALSE;
    SYS_ARCH_UNPROTECT(lev);
}
#endif
void lwip_netif_state_on_link_up(void)
{
    SYS_ARCH_DECL_PROTECT(lev);
    SYS_ARCH_PROTECT(lev);
    if (g_lwip_netif_state.netif_up_count == 0)
    {
#if LWIP_TIMERS
        if (!g_lwip_netif_state.timeout_inited &&
                !g_lwip_netif_state.timeout_init_pending)
        {
            err_t err;
            rt_kprintf("[lwip_netif] link up: queue timeout init\n");
            if (tcpip_sys_mbox_valid())
            {
                g_lwip_netif_state.timeout_init_pending = RT_TRUE;
                err = tcpip_callback(lwip_netif_state_timeout_init_callback, NULL);
                if (err != ERR_OK)
                {
                    g_lwip_netif_state.timeout_init_pending = RT_FALSE;
                    rt_kprintf("[lwip_netif] queue timeout init failed: %d\n", err);
                }
            }
        }
#endif
    }
    g_lwip_netif_state.netif_up_count++;
    SYS_ARCH_UNPROTECT(lev);
}
void lwip_netif_state_on_link_down(void)
{
    SYS_ARCH_DECL_PROTECT(lev);
    SYS_ARCH_PROTECT(lev);
    if (g_lwip_netif_state.netif_up_count > 0)
    {
        g_lwip_netif_state.netif_up_count--;
    }
    if (g_lwip_netif_state.netif_up_count == 0)
    {
#if LWIP_TIMERS
        if (g_lwip_netif_state.timeout_inited &&
                !g_lwip_netif_state.timeout_init_pending)
        {
            err_t err;
            rt_kprintf("[lwip_netif] link down: queue timeout uninit\n");
            sys_timeouts_uninit();
        }
#endif
    }
    SYS_ARCH_UNPROTECT(lev);
}

#endif /* RT_USING_LWIP_VER_NUM >= 0x20102 */