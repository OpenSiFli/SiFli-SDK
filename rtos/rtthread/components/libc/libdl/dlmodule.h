/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/08/11     Bernard      the first version
 */

#ifndef RT_DL_MODULE_H__
#define RT_DL_MODULE_H__

#include <rtthread.h>

#define RT_DLMODULE_STAT_INIT       0x00
#define RT_DLMODULE_STAT_RUNNING    0x01
#define RT_DLMODULE_STAT_CLOSING    0x02
#define RT_DLMODULE_STAT_CLOSED     0x03

struct rt_dlmodule;
typedef void *rt_addr_t;

typedef void (*rt_dlmodule_init_func_t)(struct rt_dlmodule *module);
typedef void (*rt_dlmodule_cleanup_func_t)(struct rt_dlmodule *module);
typedef int (*rt_dlmodule_entry_func_t)(int argc, char **argv);

struct rt_dlmodule
{
    struct rt_object parent;
    rt_list_t object_list;  /* objects inside this module */

    rt_uint8_t stat;        /* status of module */

    /* main thread of this module */
    rt_uint16_t priority;
    rt_uint32_t stack_size;
    struct rt_thread *main_thread;
    /* the return code */
    int ret_code;

    /* VMA base address for the first LOAD segment */
    rt_uint32_t vstart_addr;

    /* module entry, RT_NULL for dynamic library */
    rt_dlmodule_entry_func_t  entry_addr;
    char *cmd_line;         /* command line */

    rt_addr_t   mem_space;  /* memory space */
    rt_uint32_t mem_size;   /* sizeof memory space */

    /* init and clean function */
    rt_dlmodule_init_func_t     init_func;
    rt_dlmodule_cleanup_func_t  cleanup_func;

    rt_uint16_t nref;       /* reference count */

    rt_uint16_t nsym;       /* number of symbols in the module */
    struct rt_module_symtab *symtab;    /* module symbol table */
    rt_addr_t exec_mem_space;  /* module excecution space, could be same as or different from mem_space */
    int       fid;             /* file handle if execution space is in file system */
    rt_addr_t user_data;
    rt_uint32_t user_data_size;
    const char *install_path;
    rt_uint32_t wr_offset;
    rt_uint32_t relocated_value;

    void *res_mng;
    void *res_module;
};

typedef rt_uint32_t (*dlmodule_symbol_resolver)(const char *sym_str);


enum
{
    DL_OPEN,
    DL_INSTALL
};

#ifdef SOLUTION_WATCH
    #include "app_mem.h"
    #define dlm_malloc(x) app_cache_alloc(x, IMAGE_CACHE_PSRAM)
    #define dlm_free      app_cache_free
    #define dlm_cache_invalid app_mem_invalid_icache
    #define dlm_cache_clean app_mem_flush_cache
    //#define dlm_malloc app_malloc
    //#define dlm_free   app_free
#else
    #define dlm_malloc rt_malloc
    #define dlm_free   rt_free
    #define dlm_cache_invalid(addr, size)
    #define dlm_cache_clean(addr, size)
#endif

struct rt_dlmodule *dlmodule_create(void);
rt_err_t dlmodule_destroy(struct rt_dlmodule *module);

struct rt_dlmodule *dlmodule_self(void);

struct rt_dlmodule *dlmodule_load(const char *pgname);
struct rt_dlmodule *dlmodule_exec(const char *pgname, const char *cmd, int cmd_size);
void dlmodule_exit(int ret_code);

#ifdef RT_USING_XIP_MODULE
    rt_err_t dlmodule_install(const char *module_name, rt_uint8_t *module_ptr, const char *install_path);
    rt_err_t dlmodule_uninstall(const char *module_name, const char *install_path);
    struct rt_dlmodule *dlmodule_run(const char *module_name, const char *install_path);
#endif /* RT_USING_XIP_MODULE */

void *dlmodule_get_user_data(const char *module_name, rt_uint32_t size);
void dlmodule_free_user_data(const char *module_name);


struct rt_dlmodule *dlmodule_find(const char *name);

rt_uint32_t dlmodule_symbol_find(const char *sym_str);


rt_err_t dlmodule_register_ex_symbol_resolver(dlmodule_symbol_resolver resolver);
rt_err_t dlmodule_unregister_ex_symbol_resolver(void);

void *dlmodule_get_res_mng(struct rt_dlmodule *module);

void dlmodule_set_res_mng(struct rt_dlmodule *module, void *res_mng);


#endif


