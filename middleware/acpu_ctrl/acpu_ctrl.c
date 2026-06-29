/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
    file brief: ACPU controller which runs on HCPU
 */
#include <rtconfig.h>
#include <stdlib.h>
#include <board.h>
#include "rtthread.h"

#if defined(SOC_BF0_HCPU)&&!defined(SOC_BF0_ACPU)
#include "ipc_queue.h"
#include <string.h>
#include "acpu_ctrl.h"
#if defined(ANYKA_RUN_IN_ACPU) && defined(SOC_BF0_ACPU)
    #include "audio_3a_anyka.h"
#endif
#include "acpu_ctrl_private.h"
#include "bf0_mbox_common.h"
static struct rt_semaphore acpu_task_done_sema;

#ifdef ACPU_CALLER_ENABLED
static rt_mailbox_t g_call_mb;

/*
    notice: can not call acpu function in acpu_caller_entry()
*/
static void acpu_caller_entry(void *parameter)
{
    while (1)
    {
        acpu_ctrl_ipc_msg_t *p_msg;
        if (rt_mb_recv(g_call_mb, (rt_uint32_t *)&p_msg, RT_WAITING_FOREVER) == RT_EOK)
        {
            RT_ASSERT(p_msg);
            if (p_msg->task_id == HCPU_TASK_MALLOC)
            {
                size_t size = (size_t)p_msg->task_param;
                void *p = malloc(size);
                if (!p)
                {
                    rt_kprintf("malloc %d\n", size);
                    RT_ASSERT(0);
                }
                p_msg->ret_value = (uint32_t)p;
            }
            else if (p_msg->task_id == HCPU_TASK_FREE)
            {
                free((void *)p_msg->task_param);
            }
            else if (p_msg->task_id == HCPU_TASK_PRINTF)
            {
                rt_kprintf("%s", (char *)p_msg->task_param);
            }
            else
            {
                RT_ASSERT(0);
            }

            p_msg->is_rsp = 1;
            p_msg->ret_error_code = 0;

            size_t wr_size = ipc_queue_write(sys_get_ha_ipc_queue(), &p_msg, sizeof(acpu_ctrl_ipc_msg_t *), 1000);
            RT_ASSERT(wr_size == sizeof(acpu_ctrl_ipc_msg_t *));

            p_msg = NULL;
        }
    }
}
#endif /* ACPU_CALLER_ENABLED */

static int32_t queue_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    acpu_ctrl_ipc_msg_t *p_msg;
    size_t rd_size = ipc_queue_read(handle, &p_msg, sizeof(acpu_ctrl_ipc_msg_t *));
    RT_ASSERT(rd_size == sizeof(acpu_ctrl_ipc_msg_t *));
    RT_ASSERT(p_msg);

    if (p_msg->is_rsp)
    {
        if (p_msg->ret_error_code == ACPU_ERR_OK)
        {
            if (p_msg->sema) rt_sem_release(p_msg->sema);
        }
        else if (p_msg->ret_error_code == ACPU_ERR_ASSERT)
        {
            rt_kprintf("acpu assert:%s\n", p_msg->ret_value);
            RT_ASSERT(0);
        }
    }
#ifdef ACPU_CALLER_ENABLED
    else
    {
        rt_err_t err = rt_mb_send(g_call_mb, (rt_uint32_t)p_msg);
        RT_ASSERT(err == RT_EOK);
    }
#endif /* ACPU_CALLER_ENABLED */

    return 0;
}

RT_WEAK void *acpu_run_task(uint8_t task_name, void *param, uint32_t param_size, uint8_t *error_code)
{
    rt_err_t err;

    SCB_CleanInvalidateDCache();

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    //Create blocking semaphore
    char task_name_str[RT_NAME_MAX];
    rt_snprintf(task_name_str, sizeof(task_name_str), "tsk_%d", task_name);
    rt_sem_t sem = rt_sem_create(task_name_str, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sem);


    //Send message to ACPU via IPC queue
    size_t wr_size;
    size_t msg_size;
    acpu_ctrl_ipc_msg_t msg;
    acpu_ctrl_ipc_msg_t *p_msg = &msg;
    msg.is_rsp = 0;
    msg.task_id = task_name;
    msg.task_param_size = param_size;
    msg.task_param = param;
    msg.sema = sem;
    msg.ret_error_code = 0;
    msg.ret_value = 0;

    msg_size = sizeof(acpu_ctrl_ipc_msg_t *);
    wr_size = ipc_queue_write(sys_get_ha_ipc_queue(), &p_msg, msg_size, 1000);
    RT_ASSERT(wr_size == msg_size);



    //Wait for task completion, and delete semaphore
    err = rt_sem_take(sem, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
    rt_sem_delete(sem);

    //Make sure the message was processed by ACPU
    RT_ASSERT(1 == msg.is_rsp);
    RT_ASSERT(task_name == msg.task_id);

    // rt_kprintf("acpu error_code=%d\n", msg.ret_error_code);
    if (msg.ret_error_code == (uint8_t)ACPU_ERR_ASSERT)
    {
        rt_kprintf("acpu assert: %s\n", msg.ret_value);
        RT_ASSERT(0);
    }
    if (error_code)
    {
        *error_code = msg.ret_error_code;
    }


#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    return (void *)msg.ret_value;
}

RT_WEAK int acpu_init(void)
{
    rt_err_t err;
    int32_t r;

    ipc_queue_handle_t acpu_task_ntf_queue = sys_init_ha_ipc_queue(queue_rx_ind);

    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != acpu_task_ntf_queue);

    r = ipc_queue_open(acpu_task_ntf_queue);
    RT_ASSERT(0 == r);

#ifdef ACPU_CALLER_ENABLED
    g_call_mb = rt_mb_create("acpu_call_mb", 1, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_call_mb);

    rt_thread_t tid = rt_thread_create("acpu",
                                       acpu_caller_entry,
                                       NULL,
                                       1024,
                                       RT_THREAD_PRIORITY_HIGH,
                                       RT_THREAD_TICK_DEFAULT);
    rt_thread_startup(tid);

#endif /* ACPU_CALLER_ENABLED */


    acpu_power_on();

    return 0;
}
INIT_PRE_APP_EXPORT(acpu_init);



void acpu_power_on(void)
{
    HAL_RCC_ResetACPU();
    HAL_RCC_EnableModule(RCC_MOD_ACPU);
#ifdef SF32LB58X
    HAL_RCC_ReleaseACPU();
#else
    HAL_RCC_ReleaseACPU(APP_ACPU_CODE_START_ADDR);
#endif /* SF32LB58X */
}

void acpu_power_off(void)
{
    HAL_RCC_DisableModule(RCC_MOD_ACPU);
    HAL_RCC_ResetACPU();
}

__ROM_USED int acpu(int argc, char **argv)
{
    if (argc > 1)
    {

        if (strcmp("on", argv[1]) == 0)
        {
            acpu_power_on();
        }
        else if (strcmp("off", argv[1]) == 0)
        {
            acpu_power_off();
        }
        else
        {
            goto __ERR;
        }
        return 0;
    }
    else
    {
        goto __ERR;
    }

__ERR:
    rt_kprintf("wrong arg\n");

    return 0;
}
MSH_CMD_EXPORT(acpu, acpu control);

#endif /* SOC_BF0_HCPU */