/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author      Notes
 * 2018/08/29     Bernard     first version
 */

#include <rthw.h>

#include "dlfcn.h"
#include "dlmodule.h"
#include "dlelf.h"

#include <dfs_posix.h>

#define DBG_TAG    "DLMD"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>          // must after of DEBUG_ENABLE or some other options

static struct rt_module_symtab *_rt_module_symtab_begin = RT_NULL;
static struct rt_module_symtab *_rt_module_symtab_end   = RT_NULL;
/* extended symbol resolver */
static dlmodule_symbol_resolver dlmodule_ex_sym_resolver;

#if defined(__IAR_SYSTEMS_ICC__) /* for IAR compiler */
    #pragma section="RTMSymTab"
#endif

#ifdef RT_USING_XIP_MODULE
const char *make_full_module_path(struct rt_dlmodule *module, const char *install_path);



static const char *_dlmodule_get_module_file_path(const char *install_path, const char *module_name)
{
    rt_uint32_t full_path_size;
    rt_uint32_t path_size;
    rt_uint32_t name_size;
    char *file_full_path;
    rt_err_t err;
    int wsize;

    path_size = rt_strlen(install_path);
    name_size = rt_strlen(module_name);
    /* plus 1) backslash seperator
     *      2) suffix .m,
     *      3) null terminator
     * eg.  /app/sport.m
     */
    full_path_size = path_size + name_size + 4;

    file_full_path = dlm_malloc(full_path_size);
    if (!file_full_path)
    {
        goto __EXIT;
    }
    file_full_path[0] = 0;

    strncat(file_full_path, install_path, path_size);
    strncat(file_full_path, "/", 1);
    strncat(file_full_path, module_name, name_size);
    strncat(file_full_path, ".m", 2);

__EXIT:

    return file_full_path;
}
#endif /* RT_USING_XIP_MODULE */

/* set the name of module */
static void _dlmodule_set_name(struct rt_dlmodule *module, const char *path)
{
    int size;
    struct rt_object *object;
    const char *first, *end, *ptr;

    object = &(module->parent);
    ptr   = first = (char *)path;
    end   = path + rt_strlen(path);

    while (*ptr != '\0')
    {
        if (*ptr == '/')
            first = ptr + 1;
        if (*ptr == '.')
            end = ptr - 1;

        ptr ++;
    }

    size = end - first + 1;
    if (size > RT_NAME_MAX) size = RT_NAME_MAX;
    rt_memset(object->name, 0x00, sizeof(object->name));
    rt_strncpy(object->name, first, size);
    object->name[RT_NAME_MAX - 1] = '\0';
}

#define RT_MODULE_ARG_MAX    8
static int _rt_module_split_arg(char *cmd, rt_size_t length, char *argv[])
{
    int argc = 0;
    char *ptr = cmd;

    while ((ptr - cmd) < length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && (ptr - cmd) < length)
            *ptr++ = '\0';
        /* check whether it's the end of line */
        if ((ptr - cmd) >= length) break;

        /* handle string with quote */
        if (*ptr == '"')
        {
            argv[argc++] = ++ptr;

            /* skip this string */
            while (*ptr != '"' && (ptr - cmd) < length)
                if (*ptr ++ == '\\')  ptr ++;
            if ((ptr - cmd) >= length) break;

            /* skip '"' */
            *ptr ++ = '\0';
        }
        else
        {
            argv[argc++] = ptr;
            while ((*ptr != ' ' && *ptr != '\t') && (ptr - cmd) < length)
                ptr ++;
        }

        if (argc >= RT_MODULE_ARG_MAX) break;
    }

    return argc;
}

/* invoked by main thread for exit */
static void _dlmodule_exit(void)
{
    struct rt_dlmodule *module;

    module = dlmodule_self();
    if (!module) return; /* not a module thread */

    rt_enter_critical();
    if (module->stat == RT_DLMODULE_STAT_RUNNING)
    {
        struct rt_object    *object = RT_NULL;
        struct rt_list_node *node = RT_NULL;

        /* set stat to closing */
        module->stat = RT_DLMODULE_STAT_CLOSING;

        /* suspend all threads in this module */
        for (node = module->object_list.next; node != &(module->object_list); node = node->next)
        {
            object = rt_list_entry(node, struct rt_object, list);

            if ((object->type & ~RT_Object_Class_Static) == RT_Object_Class_Thread)
            {
                rt_thread_t thread = (rt_thread_t)object;

                /* stop timer and suspend thread*/
                if ((thread->stat & RT_THREAD_STAT_MASK) != RT_THREAD_CLOSE ||
                        (thread->stat & RT_THREAD_STAT_MASK) != RT_THREAD_INIT)
                {
                    rt_timer_stop(&(thread->thread_timer));
                    rt_thread_suspend(thread);
                }
            }
        }
    }
    rt_exit_critical();

    return;
}

static void _dlmodule_thread_entry(void *parameter)
{
    int argc = 0;
    char *argv[RT_MODULE_ARG_MAX];

    struct rt_dlmodule *module = (struct rt_dlmodule *)parameter;

    if (module == RT_NULL || module->cmd_line == RT_NULL)
        /* malloc for module_cmd_line failed. */
        return;

    if (module->cmd_line)
    {
        rt_memset(argv, 0x00, sizeof(argv));
        argc = _rt_module_split_arg((char *)module->cmd_line, rt_strlen(module->cmd_line), argv);
        if (argc == 0) goto __exit;
    }

    /* set status of module */
    module->stat = RT_DLMODULE_STAT_RUNNING;

    LOG_D("run main entry: 0x%p with %s",
          module->entry_addr,
          module->cmd_line);

    if (module->entry_addr)
        module->entry_addr(argc, argv);

__exit:
    _dlmodule_exit();

    return ;
}

struct rt_dlmodule *dlmodule_create(void)
{
    struct rt_dlmodule *module = RT_NULL;

    module = (struct rt_dlmodule *) rt_object_allocate(RT_Object_Class_Module, "module");
    if (module)
    {
        module->stat = RT_DLMODULE_STAT_INIT;

        /* set initial priority and stack size */
        module->priority = RT_THREAD_PRIORITY_MAX - 1;
        module->stack_size = 2048;

        rt_list_init(&(module->object_list));
    }

    return module;
}

#ifdef RT_USING_XIP_MODULE
/*
 *   layout:
 *          struct rt_dlmodule
 *          rt_module_symtab[0]
 *          rt_module_symtab[0].name_len (4byte)
 *          rt_module_symtab[0].name
 *          rt_module_symtab[1]
 *          rt_module_symtab[1].name_len (4byte)
 *          rt_module_symtab[1].name
 *          rt_module_symtab[2]
 *          rt_module_symtab[2].name_len (4byte)
 *          rt_module_symtab[2].name
 *          rt_module_symtab[3]
 *          rt_module_symtab[3].name_len (4byte)
 *          rt_module_symtab[3].name
 *
 * name_len doesn't include null terminator, name string doesn't have null terminator either
 */
rt_err_t dlmodule_serialize(struct rt_dlmodule *module, const char *install_path)
{
    const char *file_full_path;
    rt_err_t err;
    int fid = -1;
    int wsize;
    rt_uint32_t name_size;
    rt_uint32_t i;

    err = RT_ERROR;

    file_full_path = _dlmodule_get_module_file_path(install_path, module->parent.name);
    if (!file_full_path)
    {
        goto __EXIT;
    }

    fid = open(file_full_path, O_CREAT | O_WRONLY | O_TRUNC);
    if (fid < 0)
    {
        LOG_W("fail to create file: %s", file_full_path);
        goto __EXIT;
    }
    wsize = write(fid, module, sizeof(*module));
    if (wsize < sizeof(*module))
    {
        goto __EXIT;
    }

    /* save symbol table */
    for (i = 0; i < module->nsym; i++)
    {
        wsize = write(fid, module->symtab + i, sizeof(struct rt_module_symtab));
        if (wsize < sizeof(struct rt_module_symtab))
        {
            goto __EXIT;
        }
        name_size = strlen(module->symtab[i].name);
        /* write name_len */
        wsize = write(fid, &name_size, sizeof(name_size));
        if (wsize < sizeof(name_size))
        {
            goto __EXIT;
        }
        /* write name */
        wsize = write(fid, module->symtab[i].name, name_size);
        if (wsize < name_size)
        {
            goto __EXIT;
        }
    }

    err = RT_EOK;

__EXIT:

    if (file_full_path)
    {
        dlm_free((void *)file_full_path);
    }

    if (fid >= 0)
    {
        close(fid);
    }

    return err;
}

rt_err_t dlmodule_deserialize(struct rt_dlmodule *module, const char *install_path)
{
    const char *file_full_path;
    rt_err_t err;
    int fid = -1;
    int rsize;
    rt_uint32_t name_size;
    rt_uint32_t i;
    struct rt_dlmodule temp;
    rt_uint32_t symtab_size;

    err = RT_ERROR;

    module->symtab = RT_NULL;

    file_full_path = _dlmodule_get_module_file_path(install_path, module->parent.name);
    if (!file_full_path)
    {
        goto __EXIT;
    }

    rt_kprintf("dlmodule_deserialize: file_full_path %s\n", file_full_path);

    fid = open(file_full_path, O_RDONLY);
    if (fid < 0)
    {
        goto __EXIT;
    }
    rsize = read(fid, &temp, sizeof(temp));
    if (rsize < sizeof(temp))
    {
        goto __EXIT;
    }

    if (RT_Object_Class_Module != temp.parent.type)
    {
        /* invalid type */
        goto __EXIT;
    }

    module->stat = temp.stat;
    module->entry_addr = temp.entry_addr;
    module->mem_space = 0;
    module->mem_size = temp.mem_size;
    module->init_func = temp.init_func;
    module->cleanup_func = temp.cleanup_func;
    module->nref = temp.nref;
    module->nsym = temp.nsym;
    module->exec_mem_space = temp.exec_mem_space;
    module->user_data = RT_NULL;
    module->user_data_size = 0;

    symtab_size = module->nsym * sizeof(struct rt_module_symtab);
    module->symtab = dlm_malloc(symtab_size);
    /* read symbol table */
    for (i = 0; i < module->nsym; i++)
    {
        rsize = read(fid, module->symtab + i, sizeof(struct rt_module_symtab));
        if (rsize < sizeof(struct rt_module_symtab))
        {
            goto __EXIT;
        }
        /* read name_len */
        rsize = read(fid, &name_size, sizeof(name_size));
        if (rsize < sizeof(name_size))
        {
            goto __EXIT;
        }
        module->symtab[i].name = dlm_malloc(name_size + 1);
        if (!module->symtab[i].name)
        {
            goto __EXIT;
        }
        /* read name */
        rsize = read(fid, (void *)module->symtab[i].name, name_size);
        if (rsize < name_size)
        {
            goto __EXIT;
        }
        /* add null terminator */
        *((char *)module->symtab[i].name + name_size) = 0;
        rt_kprintf("dlmodule_deserialize: name %s, addr 0x%x\n", module->symtab[i].name, module->symtab[i].addr);
    }

    err = RT_EOK;

__EXIT:

    if (file_full_path)
    {
        dlm_free((void *)file_full_path);
    }

    if (fid >= 0)
    {
        close(fid);
    }

    if ((RT_EOK != err) && module->symtab)
    {
        dlm_free(module->symtab);
    }

    return err;

}
#endif /* RT_USING_XIP_MODULE */

void dlmodule_destroy_subthread(struct rt_dlmodule *module, rt_thread_t thread)
{
    RT_ASSERT(thread->module_id == module);

    /* lock scheduler to prevent scheduling in cleanup function. */
    rt_enter_critical();

    /* remove thread from thread_list (ready or defunct thread list) */
    rt_list_remove(&(thread->tlist));

    if ((thread->stat & RT_THREAD_STAT_MASK) != RT_THREAD_CLOSE &&
            (thread->thread_timer.parent.type == (RT_Object_Class_Static | RT_Object_Class_Timer)))
    {
        /* release thread timer */
        rt_timer_detach(&(thread->thread_timer));
    }

    /* change stat */
    thread->stat = RT_THREAD_CLOSE;

    /* invoke thread cleanup */
    if (thread->cleanup != RT_NULL)
        thread->cleanup(thread);

    rt_exit_critical();

#ifdef RT_USING_SIGNALS
    rt_thread_free_sig(thread);
#endif

    if (thread->type & RT_Object_Class_Static)
    {
        /* detach object */
        rt_object_detach((rt_object_t)thread);
    }
#ifdef RT_USING_HEAP
    else
    {
        /* release thread's stack */
        RT_KERNEL_FREE(thread->stack_addr);
        /* delete thread object */
        rt_object_delete((rt_object_t)thread);
    }
#endif
}

rt_err_t dlmodule_destroy(struct rt_dlmodule *module)
{
    int i;


    RT_DEBUG_NOT_IN_INTERRUPT;

    /* check parameter */
    if (module == RT_NULL)
        return -RT_ERROR;

    /* can not destroy a running module */
    if (module->stat == RT_DLMODULE_STAT_RUNNING)
        return -RT_EBUSY;

    /* do module cleanup */
    if (module->cleanup_func)
    {
        /* no need to enter critical as module is executed and operated in the same thread  */
        module->cleanup_func(module);
    }

    // list_object(&(module->object_list));

    /* cleanup for all kernel objects inside module*/
    {
        struct rt_object *object = RT_NULL;
        struct rt_list_node *node = RT_NULL;

        /* detach/delete all threads in this module */
        for (node = module->object_list.next; node != &(module->object_list);)
        {
            int object_type;

            object = rt_list_entry(node, struct rt_object, list);
            object_type = object->type & ~RT_Object_Class_Static;

            /* to next node */
            node = node->next;

            if (object->type & RT_Object_Class_Static)
            {
                switch (object_type)
                {
                case RT_Object_Class_Thread:
                    dlmodule_destroy_subthread(module, (rt_thread_t)object);
                    break;
#ifdef RT_USING_SEMAPHORE
                case RT_Object_Class_Semaphore:
                    rt_sem_detach((rt_sem_t)object);
                    break;
#endif
#ifdef RT_USING_MUTEX
                case RT_Object_Class_Mutex:
                    rt_mutex_detach((rt_mutex_t)object);
                    break;
#endif
#ifdef RT_USING_EVENT
                case RT_Object_Class_Event:
                    rt_event_detach((rt_event_t)object);
                    break;
#endif
#ifdef RT_USING_MAILBOX
                case RT_Object_Class_MailBox:
                    rt_mb_detach((rt_mailbox_t)object);
                    break;
#endif
#ifdef RT_USING_MESSAGEQUEUE
                case RT_Object_Class_MessageQueue:
                    rt_mq_detach((rt_mq_t)object);
                    break;
#endif
#ifdef RT_USING_MEMHEAP
                case RT_Object_Class_MemHeap:
                    rt_memheap_detach((struct rt_memheap *)object);
                    break;
#endif
#ifdef RT_USING_MEMPOOL
                case RT_Object_Class_MemPool:
                    rt_mp_detach((struct rt_mempool *)object);
                    break;
#endif
                case RT_Object_Class_Timer:
                    rt_timer_detach((rt_timer_t)object);
                    break;
                default:
                    LOG_E("Unsupported oject type in module.");
                    break;
                }
            }
            else
            {
                switch (object_type)
                {
                case RT_Object_Class_Thread:
                    dlmodule_destroy_subthread(module, (rt_thread_t)object);
                    break;
#ifdef RT_USING_SEMAPHORE
                case RT_Object_Class_Semaphore:
                    rt_sem_delete((rt_sem_t)object);
                    break;
#endif
#ifdef RT_USING_MUTEX
                case RT_Object_Class_Mutex:
                    rt_mutex_delete((rt_mutex_t)object);
                    break;
#endif
#ifdef RT_USING_EVENT
                case RT_Object_Class_Event:
                    rt_event_delete((rt_event_t)object);
                    break;
#endif
#ifdef RT_USING_MAILBOX
                case RT_Object_Class_MailBox:
                    rt_mb_delete((rt_mailbox_t)object);
                    break;
#endif
#ifdef RT_USING_MESSAGEQUEUE
                case RT_Object_Class_MessageQueue:
                    rt_mq_delete((rt_mq_t)object);
                    break;
#endif
#ifdef RT_USING_MEMHEAP
                    /* no delete operation */
#endif
#ifdef RT_USING_MEMPOOL
                case RT_Object_Class_MemPool:
                    rt_mp_delete((struct rt_mempool *)object);
                    break;
#endif
                case RT_Object_Class_Timer:
                    rt_timer_delete((rt_timer_t)object);
                    break;
                default:
                    LOG_E("Unsupported oject type in module.");
                    break;
                }
            }
        }
    }

    if (module->cmd_line) dlm_free(module->cmd_line);
    /* release module symbol table */
    for (i = 0; i < module->nsym; i ++)
    {
        if (module->symtab) dlm_free((void *)module->symtab[i].name);
    }
    if (module->symtab != RT_NULL)
    {
        dlm_free(module->symtab);
    }

    /* destory module */
    if (module->mem_space)
    {
        dlm_free(module->mem_space);
    }

    if (module->user_data)
    {
        dlm_free(module->user_data);
    }
    if (module->install_path)
    {
        dlm_free((void *)module->install_path);
    }
    /* delete module object */
    rt_object_delete((rt_object_t)module);

    return RT_EOK;
}

struct rt_dlmodule *dlmodule_self(void)
{
    rt_thread_t tid;
    struct rt_dlmodule *ret = RT_NULL;

    tid = rt_thread_self();
    if (tid)
    {
        ret = (struct rt_dlmodule *) tid->module_id;
    }

    return ret;
}

/*
 * Compatible with old API
 */
struct rt_dlmodule *rt_module_self(void)
{
    return dlmodule_self();
}

struct rt_dlmodule *dlmodule_load(const char *filename)
{
    int fd, length = 0;
    rt_err_t ret = RT_EOK;
    rt_uint8_t *module_ptr = RT_NULL;
    struct rt_dlmodule *module = RT_NULL;

    fd = open(filename, O_RDONLY, 0);
    if (fd >= 0)
    {
        length = lseek(fd, 0, SEEK_END);
        lseek(fd, 0, SEEK_SET);

        if (length == 0) goto __exit;

        module_ptr = (uint8_t *) dlm_malloc(length);
        if (!module_ptr) goto __exit;

        if (read(fd, module_ptr, length) != length)
            goto __exit;

        /* close file and release fd */
        close(fd);
        fd = -1;
    }
    else
    {
        goto __exit;
    }

    /* check ELF header */
    if (rt_memcmp(elf_module->e_ident, RTMMAG, SELFMAG) != 0 &&
            rt_memcmp(elf_module->e_ident, ELFMAG, SELFMAG) != 0)
    {
        rt_kprintf("Module: magic error %s %s [%02x %02x %02x %02x %02x %02x %02x %02x]\n", elf_module->e_ident, filename,
                   module_ptr[0], module_ptr[1], module_ptr[2], module_ptr[3], module_ptr[4], module_ptr[5], module_ptr[6], module_ptr[7]);
        goto __exit;
    }

    /* check ELF class */
    if (elf_module->e_ident[EI_CLASS] != ELFCLASS32)
    {
        rt_kprintf("Module: ELF class error\n");
        goto __exit;
    }

    module = dlmodule_create();
    if (!module) goto __exit;

    /* set the name of module */
    _dlmodule_set_name(module, filename);

    LOG_D("rt_module_load: %.*s", RT_NAME_MAX, module->parent.name);

    if (elf_module->e_type == ET_REL)
    {
        ret = dlmodule_load_relocated_object(module, module_ptr, DL_OPEN);
    }
    else if (elf_module->e_type == ET_DYN)
    {
        ret = dlmodule_load_shared_object(module, module_ptr, DL_OPEN);
    }
    else
    {
        rt_kprintf("Module: unsupported elf type\n");
        goto __exit;
    }

    /* check return value */
    if (ret != RT_EOK) goto __exit;

    /* release module data */
    dlm_free(module_ptr);

    /* increase module reference count */
    module->nref ++;

    /* set module initialization and cleanup function */
    module->init_func = dlsym(module, "module_init");
    module->cleanup_func = dlsym(module, "module_cleanup");
    module->stat = RT_DLMODULE_STAT_INIT;

    /* do module initialization */
    if (module->init_func)
    {
        module->init_func(module);
    }

    return module;

__exit:
    if (fd >= 0) close(fd);
    if (module_ptr) dlm_free(module_ptr);
    if (module) dlmodule_destroy(module);

    return RT_NULL;
}

#ifdef RT_USING_XIP_MODULE
const char *make_full_module_path(struct rt_dlmodule *module, const char *install_path)
{
    char *full_path;
    int full_path_len;
    int install_path_len;

    install_path_len = strlen(install_path);
    full_path_len = strlen(module->parent.name);
    full_path_len += install_path_len;
    full_path_len += 2; /* backslash and null terminator */

    full_path = dlm_malloc(full_path_len);
    if (!full_path)
    {
        return full_path;
    }
    full_path[0] = 0;
    strcat(full_path, install_path);
    full_path[install_path_len] = '/';
    full_path[install_path_len + 1] = 0;
    strcat(full_path, module->parent.name);

    return full_path;
}

rt_err_t dlmodule_install(const char *module_name, rt_uint8_t *module_ptr, const char *install_path)
{
    int length = 0;
    rt_err_t ret = RT_ERROR;
    struct rt_dlmodule *module = RT_NULL;

    if (!module_name || !module_ptr || !install_path)
    {
        rt_kprintf("Bad argument\n");
        goto __exit;
    }

    /* check ELF header */
    if (rt_memcmp(elf_module->e_ident, RTMMAG, SELFMAG) != 0 &&
            rt_memcmp(elf_module->e_ident, ELFMAG, SELFMAG) != 0)
    {
        rt_kprintf("Module: magic error\n");
        goto __exit;
    }

    /* check ELF class */
    if (elf_module->e_ident[EI_CLASS] != ELFCLASS32)
    {
        rt_kprintf("Module: ELF class error\n");
        goto __exit;
    }

    module = dlmodule_create();
    if (!module) goto __exit;

    /* set the name of module */
    _dlmodule_set_name(module, module_name);

    module->install_path = make_full_module_path(module, install_path);
    if (!module->install_path)
    {
        LOG_W("install path make fails");
        goto __exit;
    }

    LOG_D("rt_module_load: %.*s", RT_NAME_MAX, module->parent.name);

    if (elf_module->e_type == ET_DYN)
    {
        ret = dlmodule_load_shared_object(module, module_ptr, DL_INSTALL);
    }
    else
    {
        LOG_W("Module: unsupported elf type\n");
        goto __exit;
    }

    /* check return value */
    if (ret != RT_EOK) goto __exit;

    /* set module initialization and cleanup function */
    module->init_func = dlsym(module, "module_init");
    module->cleanup_func = dlsym(module, "module_cleanup");
    module->stat = RT_DLMODULE_STAT_INIT;

    ret = dlmodule_serialize(module, install_path);

    module->cleanup_func = RT_NULL;

__exit:
    if (module) dlmodule_destroy(module);

    return ret;
}

rt_err_t dlmodule_uninstall(const char *module_name, const char *install_path)
{
    rt_err_t ret = RT_ERROR;
    struct rt_dlmodule *module = RT_NULL;
    const char *mod_file_path = RT_NULL;

    if (!module_name || !install_path)
    {
        rt_kprintf("Bad argument\n");
        goto __exit;
    }

    module = dlmodule_create();
    if (!module) goto __exit;

    /* set the name of module */
    _dlmodule_set_name(module, module_name);

    module->install_path = make_full_module_path(module, install_path);
    if (!module->install_path)
    {
        LOG_W("install path make fails");
        goto __exit;
    }
    if (unlink(module->install_path) < 0)
    {
        LOG_W("fail to delete exec file %s", module->install_path);
    }

    mod_file_path = _dlmodule_get_module_file_path(install_path, module->parent.name);

    if (!mod_file_path)
    {
        LOG_W("mod_file_path make fails");
        goto __exit;
    }

    if (unlink(mod_file_path) < 0)
    {
        LOG_W("fail to delete mod file %s", mod_file_path);
    }

    ret = RT_EOK;

__exit:
    if (mod_file_path)
    {
        dlm_free((void *)mod_file_path);
    }
    if (module) dlmodule_destroy(module);

    return ret;
}

struct rt_dlmodule *dlmodule_run(const char *module_name, const char *install_path)
{

    struct rt_dlmodule *module = RT_NULL;
    rt_err_t err;

    module = dlmodule_create();
    if (!module)
    {
        rt_kprintf("dlmodule_run: dlmodule_create fail\n", module);
        goto __ERROR;
    }

    _dlmodule_set_name(module, module_name);

    err = dlmodule_deserialize(module, install_path);

    if (RT_EOK != err)
    {
        rt_kprintf("dlmodule_run: module err %d\n", module, err);
        goto __ERROR;
    }

    /* increase module reference count */
    module->nref++;

    /* do module initialization */
    if (module->init_func)
    {
        module->init_func(module);
    }

    return module;

__ERROR:
    if (module)
    {
        dlmodule_destroy(module);
    }

    return RT_NULL;
}
#endif /* RT_USING_XIP_MODULE */

struct rt_dlmodule *dlmodule_exec(const char *pgname, const char *cmd, int cmd_size)
{
    struct rt_dlmodule *module = RT_NULL;

    module = dlmodule_load(pgname);
    if (module)
    {
        if (module->entry_addr)
        {
            /* exec this module */
            rt_thread_t tid;

            module->cmd_line = rt_strdup(cmd);

            /* check stack size and priority */
            if (module->priority > RT_THREAD_PRIORITY_MAX) module->priority = RT_THREAD_PRIORITY_MAX - 1;
            if (module->stack_size < 2048 || module->stack_size > (1024 * 32)) module->stack_size = 2048;

            tid = rt_thread_create(module->parent.name, _dlmodule_thread_entry, (void *)module,
                                   module->stack_size, module->priority, 10);
            if (tid)
            {
                tid->module_id = module;
                module->main_thread = tid;

                rt_thread_startup(tid);
            }
            else
            {
                /* destory dl module */
                dlmodule_destroy(module);
                module = RT_NULL;
            }
        }
    }

    return module;
}

void dlmodule_exit(int ret_code)
{
    rt_thread_t thread;
    struct rt_dlmodule *module;

    module = dlmodule_self();
    if (!module) return;

    /* disable scheduling */
    rt_enter_critical();

    /* module is not running */
    if (module->stat != RT_DLMODULE_STAT_RUNNING)
    {
        /* restore scheduling */
        rt_exit_critical();

        return;
    }

    /* set return code */
    module->ret_code = ret_code;

    /* do exit for this module */
    _dlmodule_exit();
    /* the stat of module was changed to CLOSING in _dlmodule_exit */

    thread = module->main_thread;
    if ((thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_CLOSE)
    {
        /* main thread already closed */
        rt_exit_critical();

        return ;
    }

    /* delete thread: insert to defunct thread list */
    rt_thread_delete(thread);
    /* enable scheduling */
    rt_exit_critical();
}

rt_uint32_t dlmodule_symbol_find(const char *sym_str)
{
    /* find in kernel symbol table */
    struct rt_module_symtab *index;

    for (index = _rt_module_symtab_begin; index != _rt_module_symtab_end; index ++)
    {
        if (rt_strcmp(index->name, sym_str) == 0)
        {
            //rt_kprintf("%s_1: %s %x\n", __func__, sym_str, (rt_uint32_t)index->addr);
            return (rt_uint32_t)index->addr;
        }
    }

    RT_ASSERT(dlmodule_ex_sym_resolver);

    if (dlmodule_ex_sym_resolver)
    {
        //rt_kprintf("%s_2: %s %x\n", __func__, sym_str, dlmodule_ex_sym_resolver(sym_str));
        return dlmodule_ex_sym_resolver(sym_str);
    }

    return 0;
}

rt_err_t dlmodule_register_ex_symbol_resolver(dlmodule_symbol_resolver resolver)
{
    if (dlmodule_ex_sym_resolver)
    {
        return RT_ERROR;
    }

    dlmodule_ex_sym_resolver = resolver;
    return RT_EOK;

}
rt_err_t dlmodule_unregister_ex_symbol_resolver(void)
{
    dlmodule_ex_sym_resolver = RT_NULL;

    return RT_EOK;
}

int rt_system_dlmodule_init(void)
{
#ifdef  __CLANG_ARM
    extern int RTMSymTab$$Base;
    extern int RTMSymTab$$Limit;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&RTMSymTab$$Base;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&RTMSymTab$$Limit;
#elif defined(__GNUC__) && !defined(__CC_ARM)
    extern int __rtmsymtab_start;
    extern int __rtmsymtab_end;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&__rtmsymtab_start;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&__rtmsymtab_end;
#elif defined (__CC_ARM)
    extern int RTMSymTab$$Base;
    extern int RTMSymTab$$Limit;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&RTMSymTab$$Base;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&RTMSymTab$$Limit;
#elif defined (__IAR_SYSTEMS_ICC__)
    _rt_module_symtab_begin = __section_begin("RTMSymTab");
    _rt_module_symtab_end   = __section_end("RTMSymTab");
#endif

    return 0;
}
INIT_COMPONENT_EXPORT(rt_system_dlmodule_init);

/**
 * This function will find the specified module.
 *
 * @param name the name of module finding
 *
 * @return the module
 */
struct rt_dlmodule *dlmodule_find(const char *name)
{
    rt_object_t object;
    struct rt_dlmodule *ret = RT_NULL;

    object = rt_object_find(name, RT_Object_Class_Module);
    if (object)
    {
        ret = (struct rt_dlmodule *) object;
    }

    return ret;
}
RTM_EXPORT(dlmodule_find);

void *g_dl_user_data;

void *dlmodule_get_user_data(const char *module_name, rt_uint32_t size)
{
    struct rt_dlmodule *module;
    void *user_data = RT_NULL;

    module = dlmodule_find(module_name);
    if (module)
    {
        if (!module->user_data)
        {
            module->user_data = dlm_malloc(size);
            module->user_data_size = size;
        }
        user_data = module->user_data;
    }
    if (!user_data) rt_kprintf("dlmodule_get_user_data: user data null %s\n", module_name);
    RT_ASSERT(user_data);

    g_dl_user_data = user_data;

    return user_data;
}
RTM_EXPORT(dlmodule_get_user_data);


void dlmodule_free_user_data(const char *module_name)
{
    struct rt_dlmodule *module;

    module = dlmodule_find(module_name);
    if (module && module->user_data)
    {
        dlm_free(module->user_data);
        module->user_data = RT_NULL;
        module->user_data_size = 0;
    }

    return;
}
RTM_EXPORT(dlmodule_free_user_data);

void *dlmodule_get_res_mng(struct rt_dlmodule *module)
{
    RT_ASSERT(module);

    return module->res_mng;
}
RTM_EXPORT(dlmodule_get_res_mng);


void dlmodule_set_res_mng(struct rt_dlmodule *module, void *res_mng)
{
    RT_ASSERT(module);

    module->res_mng = res_mng;
}
RTM_EXPORT(dlmodule_set_res_mng);


int list_symbols(void)
{
    extern int __rtmsymtab_start;
    extern int __rtmsymtab_end;

    /* find in kernel symbol table */
    struct rt_module_symtab *index;

    for (index = _rt_module_symtab_begin;
            index != _rt_module_symtab_end;
            index ++)
    {
        rt_kprintf("%s => 0x%08x\n", index->name, index->addr);
    }

    return 0;
}
MSH_CMD_EXPORT(list_symbols, list symbols information);

int list_module(void)
{
    struct rt_dlmodule *module;
    struct rt_list_node *list, *node;
    struct rt_object_information *info;

    info = rt_object_get_information(RT_Object_Class_Module);
    list = &info->object_list;

    rt_kprintf("module   ref      address \n");
    rt_kprintf("-------- -------- ------------\n");
    for (node = list->next; node != list; node = node->next)
    {
        module = (struct rt_dlmodule *)(rt_list_entry(node, struct rt_object, list));
        rt_kprintf("%-*.*s %-04d  0x%08x\n",
                   RT_NAME_MAX, RT_NAME_MAX, module->parent.name, module->nref, module->mem_space);
    }

    return 0;
}
MSH_CMD_EXPORT(list_module, list modules in system);

int list_module_sym(int argc, char **argv)
{
    struct rt_dlmodule *mod;
    uint32_t i;

    if (2 != argc)
    {
        rt_kprintf("Wrong argument\n");
        return -1;
    }
    mod = dlmodule_find(argv[1]);
    if (!mod)
    {
        rt_kprintf("mod %s not found\n", argv[1]);
        return -1;
    }

    rt_kprintf("sym   address \n");
    rt_kprintf("-------- ------------\n");
    for (i = 0; i < mod->nsym; i++)
    {
        rt_kprintf("%s   0x%08x\n",
                   mod->symtab[i].name, mod->symtab[i].addr);
    }

    return 0;
}
MSH_CMD_EXPORT(list_module_sym, list sym in the module);


