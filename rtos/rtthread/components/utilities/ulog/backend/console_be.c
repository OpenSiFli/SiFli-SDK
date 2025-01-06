/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-04     armink       the first version
 */

#include <rthw.h>
#include <ulog.h>

#ifdef ULOG_BACKEND_USING_CONSOLE

#if defined(ULOG_ASYNC_OUTPUT_BY_THREAD) && ULOG_ASYNC_OUTPUT_THREAD_STACK < 384
    #error "The thread stack size must more than 384 when using async output by thread (ULOG_ASYNC_OUTPUT_BY_THREAD)"
#endif

static struct ulog_backend console;
#ifdef ULOG_USING_ASYNC_OUTPUT
    static struct rt_semaphore output_sem;
#endif

#ifdef ULOG_USING_ASYNC_OUTPUT
static rt_err_t ulog_tx_done(rt_device_t dev, void *buffer)
{
    rt_sem_release(&output_sem);
    return RT_EOK;
}
#endif

__ROM_USED void ulog_console_backend_output(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw,
        const char *log, size_t len)
{
    rt_device_t dev = rt_console_get_device();

    // Don't output binary in console.
    if (is_raw == RAW_BIN)
        return;

#ifdef RT_USING_DEVICE
    if (dev == RT_NULL)
    {
        rt_hw_console_output(log);
    }
    else
    {
        rt_uint16_t old_flag = dev->open_flag;
        if (is_raw != RAW_BIN_MIX)
            dev->open_flag |= RT_DEVICE_FLAG_STREAM;
        else
            dev->open_flag &= ~RT_DEVICE_FLAG_STREAM;

#ifndef ULOG_USING_ASYNC_OUTPUT
        dev->open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
#endif /* ULOG_USING_ASYNC_OUTPUT */

        rt_device_write(dev, 0, log, len);
        dev->open_flag = old_flag;
#ifdef ULOG_USING_ASYNC_OUTPUT
        if (dev->open_flag & RT_DEVICE_FLAG_DMA_TX)
        {
            rt_sem_take(&output_sem, RT_WAITING_FOREVER);
        }
#else
#if !defined(BSP_USING_PC_SIMULATOR)
        //RT_ASSERT(0 == (dev->open_flag & RT_DEVICE_FLAG_DMA_TX));
#endif
#endif
    }
#else
    rt_hw_console_output(log);
#endif

}

__ROM_USED int ulog_console_backend_init(void)
{
#ifdef ULOG_USING_ASYNC_OUTPUT
    rt_device_t dev;
#endif
    ulog_init();
    console.output = ulog_console_backend_output;

#ifdef ULOG_USING_ASYNC_OUTPUT
    rt_sem_init(&output_sem, "cons_be", 0, RT_IPC_FLAG_FIFO);
    dev = rt_console_get_device();
    RT_ASSERT(dev);
    rt_device_set_tx_complete(dev, ulog_tx_done);
#endif

    ulog_backend_register(&console, "console", RT_TRUE);

    return 0;
}
INIT_PREV_EXPORT(ulog_console_backend_init);

#endif /* ULOG_BACKEND_USING_CONSOLE */
