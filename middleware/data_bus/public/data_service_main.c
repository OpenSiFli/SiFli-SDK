/**
  ******************************************************************************
  * @file   data_service.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include "rtdef.h"
#include "rtthread.h"
#include "string.h"
#include "data_service.h"
#include "board.h"
#ifdef USING_IPC_QUEUE
    #include "bf0_mbox_common.h"
    #include "ipc_queue.h"
#endif
#include "mem_section.h"


#define DS_CORE_ID_HCPU    (0)
#define DS_CORE_ID_LCPU    (1)
#define DS_CORE_ID_SIMU    (0)


#ifdef DATA_SVC_PROC_THREAD_STACK_SIZE
    #define DATA_PROCESS_THREAD_STACK_SIZE (DATA_SVC_PROC_THREAD_STACK_SIZE)
#else
    #if (defined(CORE_ID_CURRENT) && (CORE_ID_CURRENT == CORE_ID_HCPU))
        #define DATA_PROCESS_THREAD_STACK_SIZE   (4096)
    #else
        #define DATA_PROCESS_THREAD_STACK_SIZE   (2048)
    #endif
#endif /* DATA_SVC_PROC_THREAD_STACK_SIZE */

#ifndef DATA_SVC_PROC_THREAD_PRIORITY
    #define DATA_SVC_PROC_THREAD_PRIORITY (RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_LOWWER * 2)
#endif /* DATA_SVC_PROC_THREAD_PRIORITY */


#ifdef DATA_SVC_MBOX_THREAD_STACK_SIZE
    #define DATA_MBOX_TRHEAD_STACK_SIZE      (DATA_SVC_MBOX_THREAD_STACK_SIZE)
#else
    #define DATA_MBOX_TRHEAD_STACK_SIZE      (1024)
#endif /* DATA_SVC_MBOX_THREAD_STACK_SIZE */

#ifndef DATA_SVC_MBOX_THREAD_PRIORITY
    #define DATA_SVC_MBOX_THREAD_PRIORITY (RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_LOWWER)
#endif /* DATA_SVC_MBOX_THREAD_PRIORITY */


#ifndef DATA_SVC_PROC_THREAD_DISABLED
    #ifndef DATA_SVC_PROC_THREAD_DYNAMIC

        L1_NON_RET_BSS_SECT_BEGIN(data_process_thread_stack)
        ALIGN(RT_ALIGN_SIZE)
        static uint8_t data_process_thread_stack[DATA_PROCESS_THREAD_STACK_SIZE];
        L1_NON_RET_BSS_SECT_END

    #endif /* DATA_SVC_PROC_THREAD_DYNAMIC */
#endif /* !DATA_SVC_PROC_THREAD_DISABLED */

#ifndef DATA_SVC_MBOX_DISABLED
    #ifndef DATA_SVC_MBOX_THREAD_DYNAMIC
        L1_NON_RET_BSS_SECT_BEGIN(data_mbox_thread_stack)
        ALIGN(RT_ALIGN_SIZE)
        static uint8_t data_mbox_thread_stack[DATA_MBOX_TRHEAD_STACK_SIZE];
        L1_NON_RET_BSS_SECT_END

    #endif /* DATA_SVC_MBOX_THREAD_DYNAMIC */
#endif /* !DATA_SVC_MBOX_DISABLED */

/*
    Start data service,
*/
extern uint32_t DS_CORE_ID_CURRENT, DS_CORE_ID_PROXY;
extern uint32_t DS_PROXY_CHANNEL, TX_BUF_SIZE, TX_BUF_ADDR, TX_BUF_ADDR_ALIAS, RX_BUF_ADDR;

int datas_start_main(void)
{
    data_service_init_param_t init_param;

#ifndef DATA_SVC_MBOX_THREAD_DISABLED
#ifdef SOC_BF0_HCPU
    DS_CORE_ID_CURRENT = DS_CORE_ID_HCPU;
    DS_CORE_ID_PROXY = DS_CORE_ID_LCPU;
#elif defined(SOC_BF0_LCPU)
    DS_CORE_ID_CURRENT = DS_CORE_ID_LCPU;
    DS_CORE_ID_PROXY = DS_CORE_ID_HCPU;
#elif defined(BSP_USING_PC_SIMULATOR)
    DS_CORE_ID_CURRENT = DS_CORE_ID_SIMU;
    DS_CORE_ID_PROXY = (DS_CORE_ID_SIMU + 1);
#else
#error "Core ID is not defined."
#endif /* SOC_BF0_HCPU */

    DS_PROXY_CHANNEL = 1;
#ifdef SOC_BF0_HCPU
    TX_BUF_SIZE = HCPU2LCPU_MB_CH2_BUF_SIZE;
    TX_BUF_ADDR = HCPU2LCPU_MB_CH2_BUF_START_ADDR;
    TX_BUF_ADDR_ALIAS = HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR);
#if defined(BSP_CHIP_ID_COMPATIBLE) && defined(SF32LB55X)
    RX_BUF_ADDR = LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR) + ((__HAL_SYSCFG_GET_REVID() & HAL_CHIP_REV_ID_A3) >> 7) * (LPSYS_RAM_SIZE_A3 - LPSYS_RAM_SIZE);
#else
    RX_BUF_ADDR = LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR);
#endif /* SOC_BF0_HCPU */

#elif defined(SOC_BF0_LCPU)
    TX_BUF_SIZE = LCPU2HCPU_MB_CH2_BUF_SIZE;
    TX_BUF_ADDR = LCPU2HCPU_MB_CH2_BUF_START_ADDR;
    TX_BUF_ADDR_ALIAS = LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR);
    RX_BUF_ADDR = HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR);
#endif

#ifdef DATA_SVC_MBOX_THREAD_DYNAMIC
    init_param.mbox_thread_stack = NULL;
#else
    init_param.mbox_thread_stack = data_mbox_thread_stack;
#endif /* DATA_SVC_MBOX_THREAD_DYNAMIC */
    init_param.mbox_thread_stack_size = DATA_MBOX_TRHEAD_STACK_SIZE;
    init_param.mbox_thread_priority = DATA_SVC_MBOX_THREAD_PRIORITY;
#else
    init_param.mbox_thread_stack_size = 0;
#endif /* DATA_SVC_MBOX_THREAD_DISABLED */

#ifndef DATA_SVC_PROC_THREAD_DISABLED
#ifdef DATA_SVC_PROC_THREAD_DYNAMIC
    init_param.proc_thread_stack = NULL;
#else
    init_param.proc_thread_stack = data_process_thread_stack;
#endif /* DATA_SVC_PROC_THREAD_DYNAMIC */
    init_param.proc_thread_stack_size = DATA_PROCESS_THREAD_STACK_SIZE;
    init_param.proc_thread_priority = DATA_SVC_PROC_THREAD_PRIORITY;
#else
    init_param.proc_thread_stack_size = 0;
#endif /* DATA_SVC_PROC_THREAD_DISABLED */

    return datas_start(&init_param);

}
INIT_ENV_EXPORT(datas_start_main);

#ifndef RT_USING_ULOG
void ulog_output(rt_uint32_t level, const char *tag, rt_bool_t newline, const char *format, ...)
{
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
