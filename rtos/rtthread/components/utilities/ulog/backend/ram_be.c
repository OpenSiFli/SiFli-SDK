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
#include "rtdevice.h"
#include <ulog.h>
#include "ram_be.h"

#ifdef ULOG_BACKEND_USING_RAM

#define STRINGIFY_(val)       #val
#define STRINGIFY(val)       STRINGIFY_(val)

#if defined(__CC_ARM)
    /**@brief   Macro for defining the begin of ZI DATA section,
    *
    * all variables inside #SECTION_ZIDATA_BEGIN and #SECTION_ZIDATA_END are put in the section
    *
    * @param[in]   section_name    Name of the section.
    * @hideinitializer
    */
    #define SECTION_ZIDATA_BEGIN(section_name)    _Pragma(STRINGIFY(arm section zidata=STRINGIFY(section_name)))
#elif defined(__CLANG_ARM)
    #define SECTION_ZIDATA_BEGIN(section_name)    _Pragma(STRINGIFY(clang section bss=STRINGIFY(section_name)))
#elif defined(__GNUC__)
    #define SECTION_ZIDATA_BEGIN(section_name)    _Pragma(STRINGIFY(clang section bss=STRINGIFY(section_name)))
#else
    #define SECTION_ZIDATA_BEGIN(section_name)
#endif

#if defined(__CC_ARM)
    /**@brief   Macro for defining the end of ZI DATA section,
    *
    * all variables inside #SECTION_ZIDATA_BEGIN and #SECTION_ZIDATA_END are put in the section
    *
    * @hideinitializer
    */
    #define SECTION_ZIDATA_END                    _Pragma(STRINGIFY(arm section zidata))
#elif defined(__CLANG_ARM)
    #define SECTION_ZIDATA_END                    _Pragma(STRINGIFY(clang section bss=""))
#elif defined(__GNUC__)
    #define SECTION_ZIDATA_END                    _Pragma(STRINGIFY(clang section bss=""))
#else
    #define SECTION_ZIDATA_END
#endif


static struct ulog_backend ulog_ram_be;
#if defined(RT_USING_PM)
    static struct rt_device ulog_ram_be_pm_device;
#endif /* RT_USING_PM */

/** wr_offset used by dectect whether ram_buf is corrupted after sleep */
static rt_uint32_t ulog_be_ram_buf_wr_offset;

SECTION_ZIDATA_BEGIN(.l1_non_ret_bss_ulog_be_ram_buf)
static ulog_ram_be_buf_t ulog_be_ram_buf;
SECTION_ZIDATA_END


__ROM_USED void ulog_ram_backend_output(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw,
                                        const char *log, size_t len)
{
    size_t wr_len;
    rt_uint32_t rd_offset;
    rt_uint32_t wr_offset;

    /* truncate log if len is larger than buf size */
    if (len > ULOG_RAM_BE_BUF_SIZE)
    {
        wr_len = ULOG_RAM_BE_BUF_SIZE;
        rd_offset = len - wr_len;
    }
    else
    {
        wr_len = len;
        rd_offset = 0;
    }
    wr_offset = ulog_be_ram_buf.wr_offset;
    RT_ASSERT(wr_offset < ULOG_RAM_BE_BUF_SIZE);
    if ((wr_len + wr_offset) >= ULOG_RAM_BE_BUF_SIZE)
    {
        rt_memcpy((void *)&ulog_be_ram_buf.buf[wr_offset], (void *)(log + rd_offset),
                  ULOG_RAM_BE_BUF_SIZE - wr_offset);
        rd_offset += ULOG_RAM_BE_BUF_SIZE - wr_offset;
        wr_len -= (ULOG_RAM_BE_BUF_SIZE - wr_offset);
        wr_offset = 0;
        ulog_be_ram_buf.full = RT_TRUE;
    }
    if (wr_len > 0)
    {
        RT_ASSERT((wr_offset + wr_len) < ULOG_RAM_BE_BUF_SIZE);
        rt_memcpy((void *)&ulog_be_ram_buf.buf[wr_offset], (void *)(log + rd_offset),
                  wr_len);
        wr_offset += wr_len;
    }
    ulog_be_ram_buf_wr_offset = wr_offset;
    ulog_be_ram_buf.wr_offset = wr_offset;
}

__ROM_USED int ulog_ram_backend_init(void)
{
    ulog_init();

    ulog_ram_be.output = ulog_ram_backend_output;
    ulog_be_ram_buf.wr_offset = 0;
    ulog_be_ram_buf.full = RT_FALSE;
    ulog_backend_register(&ulog_ram_be, "ram", RT_TRUE);

    return 0;
}
INIT_PREV_EXPORT(ulog_ram_backend_init);

#if defined(RT_USING_PM)

static rt_err_t ulog_ram_be_pm_device_control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RESUME:
    {
        uint8_t mode = (uint8_t)((uint32_t)args);
        if ((PM_SLEEP_MODE_STANDBY == mode) && (ulog_be_ram_buf_wr_offset != ulog_be_ram_buf.wr_offset))
        {
            /* data is lost after sleep, init wr_offset */
            ulog_be_ram_buf.wr_offset = 0;
            ulog_be_ram_buf.full = RT_FALSE;
            ulog_be_ram_buf_wr_offset = 0;
        }
        break;
    }
    default:
    {
        break;
    }
    }
    return result;
}


#ifdef RT_USING_DEVICE_OPS
static const rt_device_ops pm_device_ops =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    ulog_ram_be_pm_device_control,
};
#endif


int ulog_ram_be_pm_init(void)
{
    rt_err_t err = RT_EOK;
    rt_device_t device;

    device = &ulog_ram_be_pm_device;

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &pm_device_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = ulog_ram_be_pm_device_control;
#endif
    device->user_data   = NULL;

    err = rt_device_register(device, "ram_be_pm", RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);
    RT_ASSERT(RT_EOK == err);

    return 0;
}
INIT_DEVICE_EXPORT(ulog_ram_be_pm_init);

#endif /* RT_USING_PM */


#if 0
rt_uint32_t ulog_ram_be_buf_size_get(void)
{
    rt_uint32_t size;

    if (ulog_be_ram_buf.full)
    {
        size = sizeof(ulog_be_ram_buf.buf);
    }
    else
    {
        size = ulog_be_ram_buf.wr_offset;
    }
    return size;
}


void ulog_ram_be_buf_get(ulog_ram_be_buf_export_cb_t export)
{
    rt_uint8_t *start;

    if (!export)
    {
        return;
    }

    if (ulog_be_ram_buf.full)
    {
        /* export the earlier part */
        RT_ASSERT(ulog_be_ram_buf.wr_offset < ULOG_RAM_BE_BUF_SIZE);
        export(&ulog_be_ram_buf.buf[ulog_be_ram_buf.wr_offset],
               ULOG_RAM_BE_BUF_SIZE - ulog_be_ram_buf.wr_offset);
    }

    /* export the rest part */
    if (ulog_be_ram_buf.wr_offset > 0)
    {
        export(&ulog_be_ram_buf.buf[0],
               ulog_be_ram_buf.wr_offset);
    }
}
#else
void *ulog_ram_be_buf_get(rt_uint32_t *size)
{
    if (size)
    {
        *size = sizeof(ulog_be_ram_buf);
    }

    return (void *)&ulog_be_ram_buf;
}

#endif

#endif /* ULOG_BACKEND_USING_RAM */
