/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2010-11-17      yi.qiu   first version
 */

#include <rtthread.h>
#include <rtm.h>
#include <string.h>

#include "dlmodule.h"

#define MODULE_ROOT_DIR     "/modules"

void *dlopen(const char *filename, int flags)
{
    struct rt_dlmodule *module;
    char *fullpath;
    const char *def_path = MODULE_ROOT_DIR;

    /* check parameters */
    RT_ASSERT(filename != RT_NULL);

    if (0) //filename[0] != '/') /* it's a relative path, prefix with MODULE_ROOT_DIR */
    {
        fullpath = dlm_malloc(strlen(def_path) + strlen(filename) + 2);

        /* join path and file name */
        rt_snprintf(fullpath, strlen(def_path) + strlen(filename) + 2,
                    "%s/%s", def_path, filename);
    }
    else
    {
        fullpath = (char *)filename; /* absolute path, use it directly */
    }

    rt_enter_critical();

    /* find in module list */
    module = dlmodule_find(fullpath);

    if (module != RT_NULL)
    {
        rt_exit_critical();
        module->nref++;
    }
    else
    {
        rt_exit_critical();
        module = dlmodule_load(fullpath);
    }

    if (fullpath != filename)
    {
        dlm_free(fullpath);
    }

    return (void *)module;
}
RTM_EXPORT(dlopen);

#ifdef RT_USING_XIP_MODULE

void *dlrun(const char *module_name, const char *install_path)
{
    struct rt_dlmodule *module;

    /* check parameters */
    RT_ASSERT(module_name != RT_NULL);
    RT_ASSERT(install_path);

    rt_enter_critical();

    /* find in module list */
    module = dlmodule_find(module_name);

    rt_kprintf("dlrun: module_name %s module %p install_path %s\n", module_name, module, install_path);

    if (module != RT_NULL)
    {
        rt_exit_critical();
        module->nref++;
    }
    else
    {
        rt_exit_critical();
        module = dlmodule_run(module_name, install_path);
    }

    rt_kprintf("dlrun: module %p install_path %s\n", module, install_path);

    return (void *)module;
}
RTM_EXPORT(dlrun);

#endif /* RT_USING_XIP_MODULE */

