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
#include "drv_touch.h"
#include "coremark.h"
#include "drv_flash.h"
#ifdef SF32LB52X
    #include "bf0_mbox_common.h"
#endif /* SF32LB52X */
#include "mem_section.h"

#define LOG_TAG "main"
#include "log.h"

#ifdef SF32LB52X
    static rt_mailbox_t ipc_mb;
    static ipc_queue_handle_t ipc_handle;
    static char buf[FINSH_CMD_SIZE];
    bool ipc_request;
    struct rt_timer rsp_timeout_timer;
#endif /* SF32LB52X */

// enable it for dvfs_test, custom_printf should be disabled in sconstruct.py, dvfs_test_timer_callback should be put in RAM
//#define DVFS_TEST

#ifdef DVFS_TEST
    static uint8_t dvfs_test_enabled;
    static rt_timer_t dvfs_test_timer;
    static uint32_t dvfs_ram0_buf[16 * 1024];
#endif /* DVFS_TEST */

void rt_hw_systick_init(void);
void rt_flash_wait_idle(uint32_t addr);

static void disable_module1(void)
{
#ifdef SF32LB55X
    HAL_RCC_DisableModule(RCC_MOD_PSRAMC);
#endif /* SF32LB55X */
    HAL_RCC_DisableModule(RCC_MOD_EXTDMA);
#ifndef SF32LB52X
    HAL_RCC_DisableModule(RCC_MOD_DMAC1);
#endif /* SF32LB52X */
#ifdef SF32LB58X
    HAL_RCC_DisableModule(RCC_MOD_MPI4);
#endif /* SF32LB58X */
}

static void enable_module1(void)
{
#ifdef SF32LB55X
    HAL_RCC_EnableModule(RCC_MOD_PSRAMC);
#endif /* SF32LB55X */
    HAL_RCC_EnableModule(RCC_MOD_EXTDMA);
    HAL_RCC_EnableModule(RCC_MOD_DMAC1);
#ifdef SF32LB58X
    HAL_RCC_EnableModule(RCC_MOD_MPI4);
#endif /* SF32LB58X */
}


static void disable_module2(void)
{
#ifdef SF32LB58X
    HAL_RCC_DisableModule(RCC_MOD_MPI1);
    HAL_RCC_DisableModule(RCC_MOD_MPI2);
#elif defined(SF32LB56X)
    HAL_RCC_DisableModule(RCC_MOD_MPI2);
#elif defined(SF32LB52X)
    rt_flash_wait_idle(MPI2_MEM_BASE);
    rt_flash_wait_idle(MPI1_MEM_BASE);
    HAL_RCC_DisableModule(RCC_MOD_MPI1);
    HAL_RCC_DisableModule(RCC_MOD_MPI2);
#endif /* SF32LB58X */
}

static void enable_module2(void)
{
#ifdef SF32LB58X
    HAL_RCC_EnableModule(RCC_MOD_MPI1);
    HAL_RCC_EnableModule(RCC_MOD_MPI2);
#elif defined(SF32LB56X)
    HAL_RCC_EnableModule(RCC_MOD_MPI2);
#elif defined(SF32LB52X)
    HAL_RCC_EnableModule(RCC_MOD_MPI1);
    HAL_RCC_EnableModule(RCC_MOD_MPI2);
#endif /* SF32LB58X */
}

int run_coremark(int argc, char *argv[])
{
    HAL_StatusTypeDef ret;
    rt_base_t level;
#ifdef SF32LB52X
    HPSYS_DvfsModeTypeDef dvfs_mode;
#endif /* SF32LB52X */

    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
        return -1;
    }
    uint32_t freq = atoi(argv[1]);

    disable_module1();


    rt_kprintf("Current HCPU freq: %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_HCPU));

#ifdef SF32lB55X
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_CLK_LP);
#endif /* SF32LB55X */

#ifdef SF32LB52X
    if (argc > 2)
    {
        dvfs_mode = atoi(argv[2]);
        ret = HAL_RCC_HCPU_ConfigHCLKByMode(freq, dvfs_mode);
    }
    else
    {
        ret = HAL_RCC_HCPU_ConfigHCLK(freq);
    }
    if (HAL_OK != ret)
    {
        RT_ASSERT(0);
    }
#else
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    ret = HAL_RCC_HCPU_EnableDLL1(freq * 1000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
#endif /* SF32LB52X */

    level = rt_hw_interrupt_disable();
    rt_hw_systick_init();
    rt_hw_interrupt_enable(level);
    RT_ASSERT(HAL_OK == ret);

    rt_kprintf("New HCPU freq: %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_HCPU));

    level = rt_hw_interrupt_disable();
    disable_module2();
    coremark(0, 0);
    enable_module2();
    rt_hw_interrupt_enable(level);

    enable_module1();

    return 0;
}
MSH_CMD_EXPORT(run_coremark, "Coremark benchmark")

int run_while_loop(int argc, char *argv[])
{
    HAL_StatusTypeDef ret;
    uint32_t cnt;
    rt_base_t level;
#ifdef SF32LB52X
    HPSYS_DvfsModeTypeDef dvfs_mode;
#endif /* SF32LB52X */

    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
        return -1;
    }
    uint32_t freq = atoi(argv[1]);


    disable_module1();

    rt_kprintf("Current HCPU freq: %d\n", HAL_RCC_GetHCLKFreq(CORE_ID_HCPU));

#ifdef SF32lB55X
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_CLK_LP);
#endif /* SF32LB55X */

#ifdef SF32LB52X
    if (argc > 2)
    {
        dvfs_mode = atoi(argv[2]);
        ret = HAL_RCC_HCPU_ConfigHCLKByMode(freq, dvfs_mode);
    }
    else
    {
        ret = HAL_RCC_HCPU_ConfigHCLK(freq);
    }
    if (HAL_OK != ret)
    {
        RT_ASSERT(0);
    }
#else
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    ret = HAL_RCC_HCPU_EnableDLL1(freq * 1000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
#endif
    level = rt_hw_interrupt_disable();
    rt_hw_systick_init();
    rt_hw_interrupt_enable(level);
    RT_ASSERT(HAL_OK == ret);

    freq = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU);
    rt_kprintf("New HCPU freq: %d\n", freq);

    rt_kprintf("Start\n");
    cnt = 10 * freq / 20;
    HAL_DBG_DWT_Init();

    level = rt_hw_interrupt_disable();
    disable_module2();
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
    enable_module2();
    rt_hw_interrupt_enable(level);

    rt_kprintf("Done. Run for %f seconds\n", (float)HAL_DBG_DWT_GetCycles() / freq);

    enable_module1();

    return 0;
}
MSH_CMD_EXPORT(run_while_loop, "While loop benchmark")


static void alarm_callback(rt_alarm_t alarm, time_t timestamp)
{

}

int shutdown(int argc, char *argv[])
{
    uint32_t wakeup_time;
    time_t now;
    struct tm *p_tm;
    struct tm new_tm;
    struct rt_alarm_setup alarm_setup;
    rt_alarm_t alarm = NULL;

    rt_err_t ret = RT_EOK;

    if (argc < 2)
    {
        wakeup_time = 0;
    }
    else
    {
        wakeup_time = atoi(argv[1]);
    }

    MODIFY_REG(hwp_pmuc->VRTC_CR, PMUC_VRTC_CR_VRTC_VBIT_Msk,
               MAKE_REG_VAL(0xB, PMUC_VRTC_CR_VRTC_VBIT_Msk, PMUC_VRTC_CR_VRTC_VBIT_Pos));

    if (wakeup_time > 0)
    {
        now = time(RT_NULL);

        rt_kprintf("Shutdown, wake up after %d second...\n", wakeup_time);

#if defined(SF32LB56X)
        /* RC10K is used actually */
        wakeup_time = (wakeup_time + 3) / 4;
        if (wakeup_time < 2)
        {
            wakeup_time = 2;
        }
#endif

        now += wakeup_time;

        gmtime_r(&now, &new_tm);

        alarm_setup.flag = RT_ALARM_ONESHOT;
        memcpy(&alarm_setup.wktime, &new_tm, sizeof(new_tm));

        alarm = rt_alarm_create(alarm_callback, &alarm_setup);
        rt_alarm_start(alarm);

        /* delay 10ms to wait print complete */
        rt_thread_mdelay(10);
        /* Enter hibernate mode, system would wakeup 10 second later */
        HAL_PMU_EnableRtcWakeup();
#ifdef BSP_USING_BOARD_EC_LB557XXX
        //PB43
        HAL_PMU_EnablePinWakeup(0, 1);
#elif defined(SF32LB55X)
        //PB48
        HAL_PMU_EnablePinWakeup(5, 1);
#elif defined(SF32LB58X) || defined(SF32LB56X)
//TODO:
        //PB54
        HAL_PMU_EnablePinWakeup(0, 0);

        //hwp_rtc->PBR0R = 0x10a0;
        hwp_rtc->PAWKUP = 0x003F003F;
        //hwp_rtc->PBWKUP = 0x003F003F;
        hwp_pmuc->WKUP_CNT = 0x000F000F;
#elif defined(SF32LB52X)
        /* Changes PA24 to pulldown to avoid leakage if it's connected to GND */
        HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLDOWN, 1);
        for (uint32_t i = PAD_PA28; i <= PAD_PA44; i++)
        {
            HAL_PIN_Set(i, i - PAD_PA28 + GPIO_A28, PIN_PULLDOWN, 1);
        }
        //disable ldo
        hwp_pmuc->PERI_LDO &=  ~(PMUC_PERI_LDO_EN_LDO18 | PMUC_PERI_LDO_EN_VDD33_LDO2 | PMUC_PERI_LDO_EN_VDD33_LDO3);

#else

#endif
        rt_hw_interrupt_disable();

#if defined(SF32LB56X)
        /* workaround as XT32K cannot be used in hibernate */
        HAL_PMU_LpCLockSelect(PMU_LPCLK_RC10);
        /* make sure clock has switched to RC10 */
        HAL_Delay_us(100);
        HAL_PMU_DisableXTAL32();
#endif /* SF32LB56X */

        HAL_PMU_EnterHibernate();

        /* while loop until system is down */
        while (1) {};
    }
    else
    {
        rt_kprintf("Shutdown, press key1 to wake up system...\n");
        /* delay 10ms to wait print complete */
        rt_thread_mdelay(10);
#ifdef BSP_USING_BOARD_EC_LB557XXX
        //PB43
        HAL_PMU_EnablePinWakeup(0, 1);
#elif defined(SF32LB55X)
        //PB48
        HAL_PMU_EnablePinWakeup(5, 1);
#elif defined(SF32LB58X) || defined(SF32LB56X)
        //PB54, high level wakeup
        HAL_PMU_SelectWakeupPin(0, 0); //PB54 for 58x, PB32 for 56x
        HAL_PMU_EnablePinWakeup(0, 0);

        //hwp_rtc->PBR0R = 0x10a0;
        hwp_rtc->PAWKUP = 0x003F003F;
        //hwp_rtc->PBWKUP = 0x003F003F;
        hwp_pmuc->WKUP_CNT = 0x000F000F;
#elif defined(SF32LB52X)
        HAL_PMU_SelectWakeupPin(0, 10); //PA34
        HAL_PMU_EnablePinWakeup(0, 0);
        /* Changes PA24 to pulldown to avoid leakage if it's connected to GND */
        HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLDOWN, 1);
        for (uint32_t i = PAD_PA28; i <= PAD_PA44; i++)
        {
            HAL_PIN_Set(i, i - PAD_PA28 + GPIO_A28, PIN_PULLDOWN, 1);
        }
        hwp_pmuc->PERI_LDO &=  ~(PMUC_PERI_LDO_EN_LDO18 | PMUC_PERI_LDO_EN_VDD33_LDO2 | PMUC_PERI_LDO_EN_VDD33_LDO3);
        hwp_pmuc->WKUP_CNT = 0x000F000F;
#else
#endif
        rt_hw_interrupt_disable();
        /* Enter shutdown mode, system can be woken up by KEY1 */
        HAL_PMU_EnterShutdown();
        /* while loop until system is down */
        while (1) {};
    }

    return 0;
}
MSH_CMD_EXPORT(shutdown, "Shutdown")

static void reconfig_flash_clock(void)
{
    FLASH_HandleTypeDef *handle;

#ifdef SF32LB55X
    handle = rt_flash_get_handle_by_addr(FLASH_BASE_ADDR);
    HAL_FLASH_SET_CLK_rom(handle, 3);

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_SRC_SYS);
    HAL_RCC_HCPU_DisableDLL2();
#elif !defined(SF32LB52X)
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_SRC_SYS);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM2, RCC_CLK_SRC_SYS);
    HAL_RCC_HCPU_DisableDLL2();
#endif /* SF32LB55X */

}


void aon_irq_handler_hook(uint32_t wsr)
{
    reconfig_flash_clock();
}

#ifdef SF32LB52X
static int32_t ipc_rx_ind(ipc_queue_handle_t ipc_queue, size_t size)
{
    rt_err_t err;

    RT_ASSERT(RT_NULL != ipc_mb);
    err = rt_mb_send(ipc_mb, size);
    RT_ASSERT(RT_EOK == err);

    return err;
}

static void ipc_rx_thread_entry(void *p)
{
    while (1)
    {
        rt_uint32_t size;
        if (RT_EOK == rt_mb_recv(ipc_mb, &size, RT_WAITING_FOREVER))
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
            while (size)
            {
                len = ipc_queue_read(ipc_handle, (uint8_t *)buf, FINSH_CMD_SIZE - 1);
                if (0 == len)
                {
                    break;
                }
                buf[len] = '\0';
                rt_kprintf("%s", buf);
                if (size > len)
                {
                    size -= len;
                }
                else
                {
                    size = 0;
                }
                if (strstr(buf, "Done"))
                {
                    rt_base_t level = rt_hw_interrupt_disable();
                    if (ipc_request)
                    {
                        ipc_request = false;
                        rt_timer_stop(&rsp_timeout_timer);
                        rt_pm_request(PM_SLEEP_MODE_IDLE);
                    }
                    rt_hw_interrupt_enable(level);
                }
            }
        }
    }
}


static int h2l_ipc_init(void)
{
    rt_thread_t task_handle;

    ipc_mb = rt_mb_create("ipc", 512, RT_IPC_FLAG_FIFO);
    rt_kprintf("Start mailbox\n");
    ipc_handle = sys_init_hl_ipc_queue(ipc_rx_ind);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != ipc_handle);
    task_handle = rt_thread_create("ipc", ipc_rx_thread_entry, RT_NULL, 2048, RT_MAIN_THREAD_PRIORITY, 20);
    rt_thread_startup(task_handle);
    RT_ASSERT(0 == ipc_queue_open(ipc_handle));
    return 0;
}

static char *cmd_line(int argc, char **argv)
{
    char *r = NULL, *p;
    int i;

    if (argc > 1)
    {
        r = argv[1];
        p = r + strlen(r);
        for (i = 2; i < argc; i++)
        {
            while (*p == '\0' && p < argv[i])
                *p++ = ' ';
            p += strlen(argv[i]);
        }
    }
    return r;
}

int tolcpu(int argc, char **argv)
{
    char *s;
    if (argc > 1)
    {
        if (IPC_QUEUE_INVALID_HANDLE != ipc_handle)
        {
            /* resconstruct command line, replace null-terminator with space character */
            s = cmd_line(argc, argv);
            rt_kprintf("s:%s\n", s);
            rt_base_t level = rt_hw_interrupt_disable();
            if (!ipc_request)
            {
                ipc_request = true;
                rt_hw_interrupt_enable(level);
                ipc_queue_write(ipc_handle, (uint8_t *)s, strlen(s), 10);
                rt_pm_release(PM_SLEEP_MODE_IDLE);

                //rt_err_t err = rt_timer_start(&rsp_timeout_timer);
                //RT_ASSERT(RT_EOK == err);
            }
            else
            {
                rt_hw_interrupt_enable(level);
                rt_kprintf("Previous command not completed\n");
            }
        }
    }
    else
    {
        rt_pm_release(PM_SLEEP_MODE_IDLE);
    }
    return 0;
}
MSH_CMD_EXPORT(tolcpu, forward lcpu command);

static void rsp_timeout(void *parameter)
{
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_kprintf("Response timeout\n");
    //RT_ASSERT(0);
    while (1);
}

#endif /* SF32LB52X */


#ifdef DVFS_TEST

L1_RET_RODATA_SECT(freq_list) static uint32_t freq_list[]           = {240, 144, 240, 48, 240, 24, 240, 144, 48, 144, 24, 144};
L1_RET_RODATA_SECT(dll2_freq_list) static uint32_t dll2_freq_list[] = {240, 240, 240, 0,  240, 0,  240, 240, 0,  240, 0,  240};

L1_RET_CODE_SECT(dvfs_test_timer_callback, static void dvfs_test_timer_callback(void *parameter))
{
    static uint8_t cnt;
    uint32_t level;
    uint32_t freq_num = sizeof(freq_list) / sizeof(freq_list[0]);
    HAL_StatusTypeDef status;

    if (cnt >= freq_num)
    {
        cnt = 0;
    }
    rt_kprintf("change dvfs freq:%d,%d\n", freq_list[cnt], dll2_freq_list[cnt]);
    level = rt_hw_interrupt_disable();
    //HAL_Delay_us(200);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_SYSCLK);
    HAL_RCC_HCPU_DisableDLL2();
    status = HAL_RCC_HCPU_ConfigHCLK(freq_list[cnt]);
    if (dll2_freq_list[cnt] > 0)
    {
        HAL_RCC_HCPU_EnableDLL2(dll2_freq_list[cnt] * 1000000);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
        BSP_SetFlash2DIV(4);
        BSP_Flash_hw2_init();
    }
    else
    {
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_SYSCLK);
        if (24 == freq_list[cnt])
        {
            BSP_SetFlash2DIV(2);
        }
        else
        {
            BSP_SetFlash2DIV(1);
        }
        BSP_Flash_hw2_init();
    }
    cnt++;
    RT_ASSERT(HAL_OK == status);
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
                                              0, 1000, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
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

void dvfs_mem_test(uint32_t addr, uint32_t len)
{
    uint8_t *buf;
    uint32_t i, j;
    uint32_t seed;
    uint32_t freq_list[]  = {240, 144, 48, 24};
    uint32_t freq_num = sizeof(freq_list) / sizeof(freq_list[0]);
    uint8_t exp_val;

    rt_timer_stop(dvfs_test_timer);

    for (j = 0; j < freq_num; j++)
    {
        seed = rt_tick_get();
        rt_base_t level = rt_hw_interrupt_disable();
        HAL_RCC_HCPU_ConfigHCLK(freq_list[j]);
        rt_hw_interrupt_enable(level);
        srand(seed);
        buf = (uint8_t *)addr;
        for (i = 0; i < len; i++)
        {
            buf[i] = rand() & 255;
        }

        /* check */
        srand(seed);
        for (i = 0; i < len; i++)
        {
            exp_val = rand() & 255;
            if (buf[i] != exp_val)
            {
                rt_kprintf("addr %p check fail\n", &buf[i]);
                rt_kprintf("[%d] %x,%x\n", i, buf[i], exp_val);
                RT_ASSERT(0);
            }
        }
    }
    rt_timer_start(dvfs_test_timer);
}

#endif /* DVFS_TEST */

//#define DEEPSLEEP_PIN_WAKEUP_TEST
#ifdef DEEPSLEEP_PIN_WAKEUP_TEST
#ifdef SF32LB52X
static int32_t pin_interrupt_interval;
static void pin_event_handler(void *args)
{
    static rt_tick_t last_tick;
    static int last_val = -1;
    int curr_val;
    rt_tick_t curr_tick;
    int32_t tick_diff;


    curr_tick = rt_tick_get();
    curr_val = rt_pin_read((rt_base_t)args);

    if (last_val == curr_val)
    {
        rt_kprintf("WARNING: val not change: %d,%d\n", last_val, curr_val);
    }

    if (last_tick != 0)
    {
        tick_diff = curr_tick - last_tick;
        if ((tick_diff > (pin_interrupt_interval + 5)) || (tick_diff < (-pin_interrupt_interval - 5)))
        {
            rt_kprintf("pin interrupt missed\n");
        }
        RT_ASSERT((tick_diff < (pin_interrupt_interval + 5)) && (tick_diff > (-pin_interrupt_interval - 5)));
    }
    last_val = curr_val;
    last_tick = curr_tick;

    LOG_I("pin:%d,%d", (uint32_t)args, rt_pin_read((rt_base_t)args));
}


int pin_test(int argc, char **argv)
{
    char *s;

    if (argc > 1)
    {
        pin_interrupt_interval = atoi(argv[1]);
    }
    else
    {
        pin_interrupt_interval = 1000;

    }
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    return 0;
}
MSH_CMD_EXPORT(pin_test, test pin wakeup);

#endif /* SF32LB52X */
#endif /* DEEPSLEEP_PIN_WAKEUP_TEST */

int main(void)
{
    HAL_StatusTypeDef status;
    int8_t pin;
#ifdef DVFS_TEST
    char *dvfs_argv[] = {"dvfs_test", "1"};
#endif /* DVFS_TEST */

#ifdef BSP_USING_BOARD_EC_LB557XXX
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 77);
#elif defined(BSP_USING_BOARD_EH_LB555) || defined(BSP_USING_BOARD_EH_LB555XXX_V2)
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 79);
#elif defined(SF32LB55X)
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 80);
#elif defined(SF32LB56X)
#if defined(BSP_USING_BOARD_EC_LB563XXX)
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 51);
#else
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio2, 34);
#endif /* BSP_USING_BOARD_EC_LB563XXX */
#elif defined(SF32LB52X)
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 24);
#else
    pin = HAL_HPAON_QueryWakeupPin(hwp_gpio1, 64);
#endif

    RT_ASSERT(pin >= 0);
    status = HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN0 + pin, AON_PIN_MODE_LOW);
    RT_ASSERT(HAL_OK == status);

#ifdef SF32LB52X
    h2l_ipc_init();
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_timer_init(&rsp_timeout_timer, "test", rsp_timeout, 0, rt_tick_from_millisecond(50000),
                  RT_TIMER_FLAG_SOFT_TIMER);

    lcpu_power_on();

#ifdef DEEPSLEEP_PIN_WAKEUP_TEST
    uint32_t button_pin = 34;

    HAL_RCC_EnableModule(RCC_MOD_GPIO1);
    HAL_HPAON_DisableWakeupSrc(HPAON_WAKEUP_SRC_PIN0 + pin);

    rt_pin_mode(button_pin, PIN_MODE_INPUT);
    rt_pin_attach_irq(button_pin, PIN_IRQ_MODE_RISING_FALLING, pin_event_handler, (void *)button_pin);

    rt_pin_irq_enable(button_pin, true);

#if 0
    button_pin = 30;
    rt_pin_mode(button_pin, PIN_MODE_INPUT);
    rt_pin_attach_irq(button_pin, PIN_IRQ_MODE_RISING_FALLING, pin_event_handler, (void *)button_pin);

    rt_pin_irq_enable(button_pin, true);

    button_pin = 33;
    rt_pin_mode(button_pin, PIN_MODE_INPUT);
    rt_pin_attach_irq(button_pin, PIN_IRQ_MODE_RISING_FALLING, pin_event_handler, (void *)button_pin);

    rt_pin_irq_enable(button_pin, true);
#endif

#endif /* DEEPSLEEP_PIN_WAKEUP_TEST */

#endif /* SF32LB52X */

    reconfig_flash_clock();

#ifdef DVFS_TEST
    dvfs_test(2, dvfs_argv);
#endif /* DVFS_TEST */

#ifdef DEEPSLEEP_PIN_WAKEUP_TEST
    uint32_t seed = rt_tick_get();
    srand(seed);
#endif /* DEEPSLEEP_PIN_WAKEUP_TEST */

    while (1)
    {

#ifdef DEEPSLEEP_PIN_WAKEUP_TEST
        rt_thread_mdelay((rand() % 100) + 100);
        continue;
#endif /* DEEPSLEEP_PIN_WAKEUP_TEST */


#ifndef DVFS_TEST
        rt_thread_mdelay(400000);
#else
        RT_ASSERT(0 == coremark(0, 0));
        dvfs_mem_test((uint32_t)dvfs_ram0_buf, sizeof(dvfs_ram0_buf));
        dvfs_mem_test(HPSYS_RAM2_BASE, HPSYS_RAM2_SIZE);
#endif
    }

    return RT_EOK;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

