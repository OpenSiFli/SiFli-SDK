/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "rt_bt_app.h"

#define DBG_TAG "rt_bt_app.cmd"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/** @brief  Print the registered service list. */
static void bt_cmd_list_services(void)
{
    rt_bt_service_t *s;

    rt_kprintf("registered BT services:\n");
    for (s = rt_bt_app_service_list(); s != RT_NULL; s = s->next)
    {
        rt_kprintf("  %-8s (event group 0x%x)\n", s->name, s->event_group);
    }
    rt_kprintf("use \"bt <service>\" to list its sub-commands\n");
}

/** @brief  Print the subcommand table for a specific service. */
static void bt_cmd_list_subcmds(const rt_bt_service_t *svc)
{
    rt_size_t i;

    rt_kprintf("usage: bt %s <sub-command> [args...]\n", svc->name);
    for (i = 0; i < svc->cmd_num; i++)
    {
        rt_kprintf("  %-20s %s\n",
                   svc->cmds[i].name,
                   svc->cmds[i].usage ? svc->cmds[i].usage : "");
    }
}

/**
 * @brief   Entry point of the "bt" shell command.
 *
 * Usage:
 *   bt                      list all registered services
 *   bt <service>            list the subcommands of that service
 *   bt <service> <cmd> ...   invoke a subcommand
 */
static void bt(int argc, char **argv)
{
    rt_bt_service_t      *svc;
    const rt_bt_cmd_entry_t *cmd;
    rt_size_t             i;
    bt_err_t              ret;

    /* No arguments: list all services. */
    if (argc < 2)
    {
        bt_cmd_list_services();
        return;
    }

    /* Find the service. */
    svc = rt_bt_app_service_find(argv[1]);
    if (svc == RT_NULL)
    {
        rt_kprintf("bt: service \"%s\" not found\n", argv[1]);
        rt_kprintf("use \"bt\" to list available services\n");
        return;
    }

    /* Service name only: list that service's subcommands. */
    if (argc < 3)
    {
        bt_cmd_list_subcmds(svc);
        return;
    }

    /* Find the subcommand. */
    cmd = RT_NULL;
    for (i = 0; i < svc->cmd_num; i++)
    {
        if (rt_strcmp(svc->cmds[i].name, argv[2]) == 0)
        {
            cmd = &svc->cmds[i];
            break;
        }
    }
    if (cmd == RT_NULL)
    {
        rt_kprintf("bt %s: sub-command \"%s\" not found\n", argv[1], argv[2]);
        bt_cmd_list_subcmds(svc);
        return;
    }

    /* Check whether the stack is ready. */
    if (cmd->need_stack && !rt_bt_app_is_stack_ready())
    {
        rt_kprintf("bt %s %s: BT stack not ready, \"%s\" ignored\n",
                   argv[1], cmd->name, cmd->name);
        return;
    }

    /* Pass argv starting from the subcommand keyword, so the handler sees its own
     * name as argv[0] (argc - 2 / argv + 2). */
    ret = cmd->handler(argc - 2, argv + 2);
    rt_kprintf("bt %s %s: issued, ret=0x%x\n", svc->name, cmd->name, ret);
}
MSH_CMD_EXPORT(bt, RT - Thread device - framework BT control command);
