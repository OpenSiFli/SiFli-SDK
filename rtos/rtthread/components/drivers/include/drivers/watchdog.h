/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-09-12     heyuanjie87  first version.
 */

#ifndef  __WATCHDOG_H__
#define  __WATCHDOG_H__

#include <rtthread.h>

#define RT_DEVICE_CTRL_WDT_GET_TIMEOUT    (0x20 + 1) /* get timeout(in seconds) */
#define RT_DEVICE_CTRL_WDT_SET_TIMEOUT    (0x20 + 2) /* set timeout(in seconds) */
#define RT_DEVICE_CTRL_WDT_GET_TIMELEFT   (0x20 + 3) /* get the left time before reboot(in seconds) */
#define RT_DEVICE_CTRL_WDT_KEEPALIVE      (0x20 + 4) /* refresh watchdog */
#define RT_DEVICE_CTRL_WDT_START          (0x20 + 5) /* start watchdog */
#define RT_DEVICE_CTRL_WDT_STOP           (0x20 + 6) /* stop watchdog */
#define RT_DEVICE_CTRL_WDT_GET_STATUS     (0x20 + 7) /* get watchdog status */
#define RT_DEVICE_CTRL_WDT_SET_STATUS     (0x20 + 8) /* set watchdog status */


struct rt_watchdog_ops;
struct rt_watchdog_device
{
    struct rt_device parent;
    const struct rt_watchdog_ops *ops;
};
typedef struct rt_watchdog_device rt_watchdog_t;

struct rt_watchdog_ops
{
    rt_err_t (*init)(rt_watchdog_t *wdt);
    rt_err_t (*control)(rt_watchdog_t *wdt, int cmd, void *arg);
};

rt_err_t rt_hw_watchdog_register(rt_watchdog_t *wdt,
                                 const char    *name,
                                 rt_uint32_t    flag,
                                 void          *data);
/*Pet watchdog*/
void rt_hw_watchdog_pet(void);

/* Install/Uninstall IDLE hook to pet watchdog*/
void rt_hw_watchdog_hook(int enable);

/* deinit watchdog */
void rt_hw_watchdog_deinit(void);

/* get watchdog status */
uint32_t rt_hw_watchdog_get_status(void);

/* set watchdog status */
void rt_hw_watchdog_set_status(uint32_t status);



#endif /* __WATCHDOG_H__ */
