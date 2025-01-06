/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#ifdef SF32LB52X
    #include "ipc_queue_device.h"
    #include "bf0_mbox_common.h"
#endif /* SF32LB52X */

#include "coremark.h"
#define LOG_TAG "main"
#include "log.h"



static struct rt_thread coremark_thread;
ALIGN(RT_ALIGN_SIZE)
RT_USED static uint8_t coremark_stack[4096];

static struct rt_semaphore coremark_sema;

#ifdef SF32LB52X
    #define CMD_ARG_MAX (8)

    struct rt_device console_device;
    #define CONSOLE_DEVICE_NAME "con"
    static ipc_queue_handle_t lh_ipc_queue;
    static rt_sem_t ipc_sem;
#endif /* SF32LB52X */

void rt_hw_systick_init(void);

// enable it for dvfs_test, custom_printf should be disabled in sconstruct.py, dvfs_test_timer_callback should be put in RAM
//#define DVFS_TEST

#ifdef DVFS_TEST
    static uint8_t dvfs_test_enabled;
    static rt_timer_t dvfs_test_timer;
#endif /* DVFS_TEST */


static void disable_module1(void)
{
#ifdef SF32LB58X
    HAL_RCC_DisableModule(RCC_MOD_DMAC3);
    HAL_RCC_DisableModule(RCC_MOD_MPI5);
    hwp_lpsys_cfg->ULPMCR |= 0x40000000; //disable ROM
#elif defined(SF32LB56X)
    HAL_RCC_DisableModule(RCC_MOD_DMAC2);
    HAL_RCC_DisableModule(RCC_MOD_MPI5);
    hwp_lpsys_cfg->ULPMCR |= 0x40000000; //disable ROM
#else
    HAL_RCC_DisableModule(RCC_MOD_DMAC2);
#endif /* SF32LB58X */
    //HAL_sw_breakpoint();

}

static void enable_module1(void)
{
#ifdef SF32LB58X
    hwp_lpsys_cfg->ULPMCR &= ~0x40000000;  //enable ROM
    HAL_RCC_EnableModule(RCC_MOD_DMAC3);
    HAL_RCC_EnableModule(RCC_MOD_MPI5);
#elif defined(SF32LB56X)
    hwp_lpsys_cfg->ULPMCR &= ~0x40000000;  //enable ROM
    HAL_RCC_EnableModule(RCC_MOD_DMAC2);
    HAL_RCC_EnableModule(RCC_MOD_MPI5);
#else
    HAL_RCC_EnableModule(RCC_MOD_DMAC2);
#endif /* SF32LB58X */

}

#if 0
static void disable_module2(void)
{

}

static void enable_module2(void)
{

}
#endif

int run_coremark(int argc, char *argv[])
{
    rt_err_t ret;
    uint32_t freq;
#ifdef SF32LB52X
    LPSYS_DvfsModeTypeDef dvfs_mode;
#else
    uint32_t hdiv;
#endif /* SF32LB52X */

    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
        return -1;
    }

    rt_kprintf("Current LCPU freq: %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_LCPU));

    freq = atoi(argv[1]);

#ifdef SF32LB52X
    if (argc > 2)
    {
        dvfs_mode = atoi(argv[2]);
        if (LPSYS_DVFS_MODE_S == dvfs_mode)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0xA, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
        ret = HAL_RCC_LCPU_ConfigHCLKByMode(freq, dvfs_mode);
        if (LPSYS_DVFS_MODE_D == dvfs_mode)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0x9, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
    }
    else
    {
        if (freq >= 48)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0xA, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
        ret = HAL_RCC_LCPU_ConfigHCLK(freq);
        if (freq < 48)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0x9, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
    }

    if (HAL_OK != ret)
    {
        RT_ASSERT(0);
    }
#else

    hdiv = 48 / freq;
    if (hdiv < 1)
    {
        hdiv = 1;
    }

    HAL_RCC_LCPU_SetDiv(hdiv, 1, 3);
#endif

    rt_kprintf("New LCPU freq: %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_LCPU));

    rt_hw_systick_init();

    ret = rt_sem_release(&coremark_sema);
    RT_ASSERT(RT_EOK == ret);

    return 0;
}
MSH_CMD_EXPORT(run_coremark, "Coremark benchmark")

int run_while_loop(int argc, char *argv[])
{
    rt_err_t ret;
    uint32_t freq;
    int32_t cnt;
#ifdef SF32LB52X
    LPSYS_DvfsModeTypeDef dvfs_mode;
#else
    uint32_t hdiv;
#endif /* SF32LB52X */

    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
        return -1;
    }

    rt_kprintf("Current LCPU freq: %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_LCPU));

    freq = atoi(argv[1]);

#ifdef SF32LB52X
    if (argc > 2)
    {
        dvfs_mode = atoi(argv[2]);
        if (LPSYS_DVFS_MODE_S == dvfs_mode)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0xA, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
        ret = HAL_RCC_LCPU_ConfigHCLKByMode(freq, dvfs_mode);
        if (LPSYS_DVFS_MODE_D == dvfs_mode)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0x9, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
    }
    else
    {
        if (freq >= 48)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0xA, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
        ret = HAL_RCC_LCPU_ConfigHCLK(freq);
        if (freq < 48)
        {
            MODIFY_REG(hwp_pmuc->BUCK_CR2, PMUC_BUCK_CR2_SET_VOUT_M_Msk,
                       MAKE_REG_VAL(0x9, PMUC_BUCK_CR2_SET_VOUT_M_Msk, PMUC_BUCK_CR2_SET_VOUT_M_Pos));
        }
    }
    if (HAL_OK != ret)
    {
        RT_ASSERT(0);
    }
#else

    hdiv = 48 / freq;
    if (hdiv < 1)
    {
        hdiv = 1;
    }

    HAL_RCC_LCPU_SetDiv(hdiv, 1, 3);
#endif

    freq = HAL_RCC_GetHCLKFreq(CORE_ID_LCPU);
    rt_kprintf("New LCPU freq: %d\n", freq);

    rt_hw_systick_init();


    rt_kprintf("Start\n");
    cnt = 10 * freq / 20;
    //HAL_DBG_DWT_Init();

    rt_base_t mask;
    mask = rt_hw_interrupt_disable();
    disable_module1();
    while (cnt > 0)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        cnt--;
    }

    enable_module1();
    rt_hw_interrupt_enable(mask);

    rt_kprintf("Done. Run for %f seconds\n", (float)HAL_DBG_DWT_GetCycles() / freq);

    return 0;
}
MSH_CMD_EXPORT(run_while_loop, "While loop benchmark")

#ifdef DVFS_TEST

static void dvfs_test_timer_callback(void *parameter)
{
    static uint8_t cnt;
    uint32_t level;

    level = rt_hw_interrupt_disable();

    HAL_Delay_us(50);

    cnt++;
    if (cnt & 1)
    {
        /* if dvfs_test_enabled == 2, stay in 96M */
        if (0 == (hwp_pmuc->DBL96_CR & PMUC_DBL96_CR_EN))
        {
            HAL_PMU_ConfigDBL96(true);
            hwp_mpi5->PSCLR = 2;
            HAL_RCC_LCPU_ClockSelectDBL96();
        }
    }
    else if (1 == dvfs_test_enabled)
    {
        /* switch to 48M using RC48 */
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
        hwp_mpi5->PSCLR = 1;
        HAL_PMU_ConfigDBL96(false);
    }

    rt_hw_interrupt_enable(level);
}


static int dvfs_test(int argc, char *argv[])
{
    rt_err_t ret;
    bool enabled;

    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
        return -1;
    }

    enabled = atoi(argv[1]);

    if (enabled)
    {
        if (dvfs_test_enabled)
        {
            rt_kprintf("dvfs test already enabled\n");
        }
        else
        {
            RT_ASSERT(!dvfs_test_timer);
            dvfs_test_timer = rt_timer_create("dvfs", dvfs_test_timer_callback,
                                              0, 100, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
            RT_ASSERT(dvfs_test_timer);
            ret = rt_timer_start(dvfs_test_timer);
            RT_ASSERT(RT_EOK == ret);
            dvfs_test_enabled = enabled;
            rt_kprintf("dvfs test enabled\n");
        }
    }
    else if (dvfs_test_enabled)
    {
        RT_ASSERT(dvfs_test_timer);
        rt_timer_delete(dvfs_test_timer);
        dvfs_test_timer = NULL;
        dvfs_test_enabled = 0;
        rt_kprintf("dvfs test disabled\n");
    }
    else
    {
        rt_kprintf("dvfs test not enabled yet\n");
    }
    return 0;
}
MSH_CMD_EXPORT(dvfs_test, "dvfs test")
#endif /* DVFS_TEST */

#if 1
void coremark_entry(void *parameter)
{
    rt_err_t ret;

    while (1)
    {
        ret = rt_sem_take(&coremark_sema, RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == ret);

        rt_base_t mask;
#ifndef DVFS_TEST
        mask = rt_hw_interrupt_disable();
        disable_module1();
        coremark(0, 0);
        enable_module1();
        rt_hw_interrupt_enable(mask);
        rt_kprintf("Done. ");
#else
        if (!dvfs_test_enabled)
        {
            mask = rt_hw_interrupt_disable();
            coremark(0, 0);
            rt_hw_interrupt_enable(mask);
        }
        else
        {
            while (dvfs_test_enabled)
            {
                coremark(0, 0);
            }
        }
#endif /* DVFS_TEST */
    }
}
#endif

#ifdef SF32LB52X
static int32_t ipc_rx_ind(ipc_queue_handle_t ipc_queue, size_t size)
{
    rt_err_t err;

    RT_ASSERT(RT_NULL != ipc_sem);
    err = rt_sem_release(ipc_sem);
    RT_ASSERT(RT_EOK == err);

    return err;
}

static int cmd_split(char *cmd, rt_size_t length, char *argv[CMD_ARG_MAX])
{
    char *ptr;
    rt_size_t position;
    rt_size_t argc;
    rt_size_t i;

    ptr = cmd;
    position = 0;
    argc = 0;

    while (position < length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t' || *ptr == ';') && position < length)
        {
            *ptr = '\0';
            ptr ++;
            position ++;
        }

        if (argc >= CMD_ARG_MAX)
        {
            LOG_I("Too many args ! We only Use:\n");
            for (i = 0; i < argc; i++)
            {
                LOG_I("%s ", argv[i]);
            }
            LOG_I("\n");
            break;
        }

        if (position >= length) break;

        /* handle string */
        if (*ptr == '"')
        {
            ptr ++;
            position ++;
            argv[argc] = ptr;
            argc ++;

            /* skip this string */
            while (*ptr != '"' && position < length)
            {
                if (*ptr == '\\')
                {
                    if (*(ptr + 1) == '"')
                    {
                        ptr ++;
                        position ++;
                    }
                }
                ptr ++;
                position ++;
            }
            if (position >= length) break;

            /* skip '"' */
            *ptr = '\0';
            ptr ++;
            position ++;
        }
        else
        {
            argv[argc] = ptr;
            argc ++;
            while ((*ptr != ' ' && *ptr != '\t' && *ptr != ';') && position < length)
            {
                ptr ++;
                position ++;
            }
            if (position >= length) break;
        }
    }

    return argc;
}


static void ipc_rx_thread_entry(void *p)
{
    while (1)
    {
        char *argv[CMD_ARG_MAX];
        int argc;
        if (RT_EOK == rt_sem_take(ipc_sem, RT_WAITING_FOREVER))
        {
            char *buf;
            rt_size_t len;
            rt_size_t size = ipc_queue_get_rx_size(lh_ipc_queue);

            if (size <= 0)
            {
                continue;
            }
            buf = rt_malloc(size + 1);
            RT_ASSERT(RT_NULL != buf);
            memset(buf, 0, size + 1);

            len = ipc_queue_read(lh_ipc_queue, (uint8_t *)buf, size);
            if (0 == len)
            {
                goto __CONTINUE;
            }

            /* split arguments */
            memset(argv, 0x00, sizeof(argv));
            argc = cmd_split(buf, size, argv);
            if (argc == 0)
            {
                goto __CONTINUE;
            }

            rt_kprintf("cmd1:%s\n", argv[0]);
            if (0 == strncmp(argv[0], "run_coremark", strlen("run_coremark")))
            {
                run_coremark(argc, argv);
            }
            else if (0 == strncmp(argv[0], "run_while_loop", strlen("run_while_loop")))
            {
                run_while_loop(argc, argv);
            }
__CONTINUE:
            rt_free(buf);
        }
    }
}


static void console_device_init(void)
{
    rt_thread_t task_handle;

    ipc_sem = rt_sem_create("ipc", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(ipc_sem);
    task_handle = rt_thread_create("ipc", ipc_rx_thread_entry, RT_NULL, 2048, RT_MAIN_THREAD_PRIORITY, 20);
    rt_thread_startup(task_handle);


    lh_ipc_queue = sys_init_lh_ipc_queue(ipc_rx_ind);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != lh_ipc_queue);
    ipc_queue_set_user_data(lh_ipc_queue, (uint32_t)&console_device);
    ipc_queue_device_register(&console_device, CONSOLE_DEVICE_NAME, lh_ipc_queue);
    RT_ASSERT(RT_EOK == rt_device_open(&console_device, RT_DEVICE_OFLAG_RDWR));

    rt_kprintf("console init done\n");

    rt_console_set_device(CONSOLE_DEVICE_NAME);

}

#endif /* SF32LB52X */

int main(void)
{
    rt_err_t ret;

    //HAL_sw_breakpoint();

#ifdef SF32LB52X
    console_device_init();
#endif /* SF32LB52X */

#if 1
    rt_sem_init(&coremark_sema, "coremark", 0, RT_IPC_FLAG_FIFO);

    ret = rt_thread_init(&coremark_thread, "coremark", coremark_entry, RT_NULL, coremark_stack, sizeof(coremark_stack),
                         RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(RT_EOK == ret);

    rt_thread_startup(&coremark_thread);
#endif

#ifdef SF32LB52X
    HAL_LPAON_DisableWakeupSrc(LPAON_WAKEUP_SRC_BT);
#endif /* SF32LB52X */

    while (1)
    {
        rt_thread_mdelay(1000000);
    }
    return RT_EOK;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

