/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "bf0_hal_patch.h"
#include "bf0_mbox_common.h"
#ifdef HAL_USING_HTOL
    #include <string.h>
    #include <stdlib.h>
    #include "utest.h"
    #include "bf0_hal.h"
    #include "tc_utils.h"

    #define _WDT_TEST        1
    #if _WDT_TEST
        static WDT_HandleTypeDef   WdtHandle;
    #endif
#endif

static rt_mailbox_t queue;

static ipc_queue_handle_t hl_ipc_queue;

static int32_t peer_cpu_rx_ind_l(ipc_queue_handle_t ipc_queue, size_t size)
{
    rt_err_t err;

    RT_ASSERT(RT_NULL != queue);
    err = rt_mb_send(queue, size);
    RT_ASSERT(RT_EOK == err);

    return err;
}

static char line[FINSH_CMD_SIZE * 2];
static char buf[FINSH_CMD_SIZE];
static void mbox_rx_thread_entry(void *p)
{
    while (1)
    {
        rt_uint32_t size;
        if (RT_EOK == rt_mb_recv(queue, &size, RT_WAITING_FOREVER))
        {
            char *prefix, *p;
            rt_size_t len;
            int i;

            if (size <= 0)
            {
                continue;
            }
            RT_ASSERT(RT_NULL != buf);

            prefix = "L";
            len = ipc_queue_read(hl_ipc_queue, (uint8_t *)buf, FINSH_CMD_SIZE - 1);
            buf[len] = '\0';
            rt_kprintf("%s", buf);
#if 0
            buf[len] = '\0';
            //rt_print_data(buf, 0, len);

            strcat(line, buf);
            p = line;
            len = strlen(line);
            for (i = 0; i < len; i++)
            {
                if (line[i] == '\n')
                {
                    line[i] = '\0';
                    rt_kprintf("[%s]%s\n", prefix, p);
                    p = &(line[i + 1]);
                }
            }
            if (p != line)
            {
                strcpy(line, p);
            }
#endif
        }
    }
}

static int test_init(void)
{
    rt_thread_t task_handle;

    queue = rt_mb_create("test", 512, RT_IPC_FLAG_FIFO);
    rt_kprintf("Start mailbox\n");
    hl_ipc_queue = sys_init_hl_ipc_queue(peer_cpu_rx_ind_l);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != hl_ipc_queue);

    task_handle = rt_thread_create("mbox_th", mbox_rx_thread_entry, RT_NULL, 2048, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT * 2);
    rt_thread_startup(task_handle);
    RT_ASSERT(0 == ipc_queue_open(hl_ipc_queue));

    return 0;
}

INIT_APP_EXPORT(test_init);
#ifdef HAL_USING_HTOL
extern void utest_run_all(int loops);
static void all_test_thread_entry(void *p)
{
    int i = 0;
    while (1)
    {
        utest_run_all(1);
        rt_thread_mdelay(300);
        i++;
        rt_kprintf("[-----] I = %d", i);
    }
}


static void all_test2_thread_entry(void *p)
{
    int ledcnt = 0;
    int pin = 52;
    while (1)
    {

        GPIO_InitTypeDef GPIO_InitStruct;
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_PULLDOWN, 1);
        GPIO_InitStruct.Pin = pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

        if (ledcnt % 2 == 0)
        {
            HAL_GPIO_WritePin(hwp_gpio1, pin, 1);
            rt_kprintf("GPIO GGGGGGGGGGGGGGGGGG+++++++++++++++++++++++++++++++++++++++++++++++");
        }
        else
        {
            HAL_GPIO_WritePin(hwp_gpio1, pin, 0);
            rt_kprintf("GPIO DDDDDDDDDDDDDDDDDD-----------------------------------------------");
        }


        rt_thread_mdelay(2000);
        //HAL_Delay(g_tmout1 / 3);
        ledcnt++;
    }
}

static int test_all(void)
{
    rt_thread_t task_handle;
    task_handle = rt_thread_create("utest_all_th", all_test_thread_entry, RT_NULL, 2048, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT * 2);
    rt_thread_startup(task_handle);
    return 0;
}
INIT_APP_EXPORT(test_all);

static int test_gpio(void)
{
    rt_thread_t task_handle;
    task_handle = rt_thread_create("utest_all_th", all_test2_thread_entry, RT_NULL, 2048, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT * 2);
    rt_thread_startup(task_handle);
    return 0;
}
INIT_APP_EXPORT(test_gpio);
#endif

#ifdef __CC_ARM
    #pragma arm section zidata="UNINITZI"
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section bss="UNINITZI"
#else
#endif

#if defined(RT_USING_PM)
char pm_test_cmd[100];
uint8_t    pm_test_run_flag = 0;
uint8_t    pm_test_wake_flag = 0;
uint8_t    pm_test_sleep_mode = 0;
uint32_t   pm_test_wake_src = 0;
uint32_t   pm_test_sleep_in = 0;
uint32_t   pm_test_sleep_out = 0;
uint32_t   pm_sleep_mode_idx = 0;
uint32_t   pm_sleep_cnt_idx = 0;

uint8_t pm_request_idl(rt_tick_t tick)
{
    return PM_SLEEP_MODE_IDLE;
}
#endif

#ifdef __CC_ARM
    #pragma arm section zidata
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section bss=""
#else
#endif




char case_run_cmd[100];
int msh_exec(char *cmd, rt_size_t length);
#ifdef HAL_USING_HTOL
    extern uint8_t lcpu_power_on();
    extern void rt_pin_mode(rt_base_t pin, rt_base_t mode);
    extern void rt_pin_write(rt_base_t pin, rt_base_t value);
    #define PIN_MODE_OUTPUT_OD      0x04
#endif
int main(void)
{

#ifdef HAL_USING_HTOL
    lcpu_power_on();
#endif
#if defined(RT_USING_PM)
    rt_kprintf("pm_test:(%d %d-%d) PMR_%x WSR_%x ISSR_%x\n", pm_test_run_flag, pm_sleep_mode_idx, pm_sleep_cnt_idx, hwp_hpsys_aon->PMR, hwp_hpsys_aon->WSR, hwp_hpsys_aon->ISSR);
    rt_pm_override_mode_select(&pm_request_idl);
    {
        pm_test_wake_flag = 0;
        pm_test_run_flag = 0;
        pm_test_sleep_mode = 0;
        pm_test_wake_src = 0;
        pm_test_sleep_in = 0;
        pm_test_sleep_out = 0;
        pm_sleep_mode_idx = 0;
        pm_sleep_cnt_idx = 0;
    }
#endif

#ifdef HAL_USING_HTOL
#if 1
    char hal_test_cmd[100];
    rt_sprintf(case_run_cmd, "%s", "utest_all");
    rt_kprintf("hal_test(%d):%s\n", rt_strlen(case_run_cmd), case_run_cmd);
    lcpu_power_on();
    msh_exec(case_run_cmd, rt_strlen(case_run_cmd));
#endif

#if _WDT_TEST
    {
        uint32_t iCnt = 0;
        uint32_t g_tmout1 = 30000;
        uint32_t g_tmout2 = 2000;

        WdtHandle.Instance = hwp_wdt1;
        WdtHandle.Init.Reload = g_tmout1 * 32 / 2;   // Counter is based on 32K clock, g_tmout is in ms
        WdtHandle.Init.Reload2 = g_tmout2 * 32;


        __HAL_WDT_STOP(&WdtHandle);
        if (HAL_WDT_Init(&WdtHandle) != HAL_OK)
        {
            /* Initialization Error */
            rt_kprintf("wdt test now %d\n", iCnt++);
        }


        // Interrupt generate for counter 0
        __HAL_WDT_INT(&WdtHandle, 0);
        __HAL_WDT_START(&WdtHandle);

        while (1)
        {
            //if(ledcnt%5 == 0)
            {
                HAL_WDT_Refresh(&WdtHandle);
                rt_kprintf("Pet WDT %d\n", iCnt++);
            }
            rt_thread_mdelay(5000);
            //HAL_Delay(g_tmout1 / 3);
        }

        __HAL_WDT_STOP(&WdtHandle);
    }
#endif
#endif
    while (1)
    {
        //rt_kprintf("test \n");
        rt_thread_mdelay(3600000);
    }
}



