/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RT_BT_APP_H__
#define __RT_BT_APP_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "bt_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Shell command table
 * ------------------------------------------------------------------------- */

/**
 * @brief   A shell subcommand exposed by a service.
 *
 * Triggered from the console via "bt <service> <name> [args...]". The core parses
 * argc/argv, checks @ref need_stack, and then calls @ref handler.
 */
typedef struct
{
    const char *name;                       /**< Subcommand keyword, for example "make_call" */
    bt_err_t (*handler)(int argc, char **argv);  /**< Executes the action; argv[0] is the subcommand itself */
    const char *usage;                      /**< One-line help text */
    rt_bool_t   need_stack;                 /**< RT_TRUE: the BT protocol stack must already be ready */
} rt_bt_cmd_entry_t;

/* ---------------------------------------------------------------------------
 * Service descriptor
 * ------------------------------------------------------------------------- */

/**
 * @brief   A Bluetooth service plugin.
 *
 * Each profile provides one instance of this structure and registers it with the
 * RT_BT_SERVICE_REGISTER() macro. The core handles event routing and command
 * dispatch.
 */
typedef struct rt_bt_service
{
    const char *name;                       /**< Service name, for example "hfp" or "spp" */
    rt_uint8_t  event_group;                /**< High byte of the event, used for routing (BT_HF_TYPE_ID, etc.) */

    /**
     * @brief   Event handler callback (called in the core's service thread context).
     *
     * @param event  Full 16-bit event code
     * @param args   Deep-copied arguments (lifetime managed by the core)
     */
    void (*on_event)(rt_uint16_t event, void *args);

    /**
     * @brief   Deep-copy hook (called in the BT notification context).
     *
     * Some event arguments contain pointers (such as strings or data blocks), and
     * those pointers are only valid during the notification callback. @ref clone is
     * responsible for flattening them into @p buf so that the args received by the
     * service thread are fully self-contained.
     *
     * @param event  Full 16-bit event code
     * @param args   Original arguments (pointers may reference temporary buffers)
     * @param buf    Destination buffer (size @p cap bytes)
     * @param cap    Capacity of @p buf
     * @return       Number of bytes actually written to @p buf; if 0 is returned,
     *               the core uses the default shallow copy
     *
     * @note    If this hook is not provided (RT_NULL), the core truncates the
     *          arguments to BT_APP_EVT_ARG_MAX bytes and performs a fixed-size
     *          shallow copy.
     */
    rt_size_t (*clone)(rt_uint16_t event, void *args, void *buf, rt_size_t cap);

    const rt_bt_cmd_entry_t *cmds;          /**< Shell subcommand table */
    rt_size_t                cmd_num;       /**< Length of the @ref cmds array */

    struct rt_bt_service    *next;          /**< Internal linked-list node (managed by the core) */
} rt_bt_service_t;

/**
 * @brief   Service self-registration macro.
 *
 * Call this at the end of a service implementation file in global scope. The core
 * automatically adds the service to the registry during system initialization.
 *
 * @param svc  Name of a static/global variable of type rt_bt_service_t
 *
 * @code
 * static rt_bt_service_t my_service = { ... };
 * RT_BT_SERVICE_REGISTER(my_service);
 * @endcode
 */
#define RT_BT_SERVICE_REGISTER(svc)                                         \
    static int _rt_bt_svc_##svc##_init(void)                                \
    {                                                                        \
        return (int)rt_bt_app_register_service(&svc);                       \
    }                                                                        \
    INIT_APP_EXPORT(_rt_bt_svc_##svc##_init)

/* ---------------------------------------------------------------------------
 * Control interface (used by services, ultimately routed to bt_device)
 * ------------------------------------------------------------------------- */

/**
 * @brief   Send a control command to bt_device.
 *
 * This is a wrapper around rt_device_control(bt_dev, cmd, args), provided for
 * services.
 *
 * @param cmd   BT_CONTROL_* command code (defined in bt_device.h)
 * @param args  Command arguments (type depends on cmd)
 * @return      BT_EOK on success, or a negative error code on failure
 */
bt_err_t rt_bt_app_control(int cmd, void *args);

/**
 * @brief   Query whether the BT protocol stack is ready.
 * @return  RT_TRUE: the stack is ready; RT_FALSE: the stack is not ready or has not
 *          been initialized
 */
rt_bool_t rt_bt_app_is_stack_ready(void);

/* ---------------------------------------------------------------------------
 * Service registry (managed by the core; services register automatically via
 * RT_BT_SERVICE_REGISTER)
 * ------------------------------------------------------------------------- */

/**
 * @brief   Manually register a service (normally not called directly; use the
 *          RT_BT_SERVICE_REGISTER macro instead).
 * @param svc  Pointer to the service descriptor (must be statically allocated;
 *             the core does not copy it)
 * @return     RT_EOK on success, or a negative errno on failure
 */
rt_err_t rt_bt_app_register_service(rt_bt_service_t *svc);

/* ---------------------------------------------------------------------------
 * Core lifecycle (called by main.c)
 * ------------------------------------------------------------------------- */

/**
 * @brief   Initialize the core: create the event thread/queue, find and open
 *          "bt_device", and register the event callback.
 *
 * Must be called before sifli_ble_enable(). Registered services are automatically
 * included (they self-register through INIT_APP_EXPORT before main runs).
 *
 * @return  RT_EOK on success, or a negative errno on failure.
 */
rt_err_t rt_bt_app_core_init(void);

/**
 * @brief   Iterate over the registered services (for shell commands).
 * @return  Head of the internal service linked list, or RT_NULL if none exist.
 */
rt_bt_service_t *rt_bt_app_service_list(void);

/**
 * @brief   Find a registered service by name.
 * @param   name  Service name, for example "hfp".
 * @return  The matching service, or RT_NULL if not found.
 */
rt_bt_service_t *rt_bt_app_service_find(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* __RT_BT_APP_H__ */
