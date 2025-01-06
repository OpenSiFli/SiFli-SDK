/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>

#include "rpmsg_platform.h"
#include "rpmsg_env.h"

#include "register.h"

#if defined(RL_USE_ENVIRONMENT_CONTEXT) && (RL_USE_ENVIRONMENT_CONTEXT == 1)
#error "This RPMsg-Lite port requires RL_USE_ENVIRONMENT_CONTEXT set to 0"
#endif

#include "ipc_queue.h"
#include "rtthread.h"


#ifdef SOC_BF0_HCPU
#define MAILBOX_IRQn  LCPU2HCPU_IRQn
#else
#define MAILBOX_IRQn  HCPU2LCPU_IRQn
#endif 

#define RPMSG_IPC_QUEUE_0    (5)
#define RPMSG_IPC_QUEUE_1    (6)

#define RPMSG_IPC_QUEUE_NUM  (2)


static int32_t isr_counter     = 0;
static int32_t disable_counter = 0;
static void *platform_lock;
static ipc_queue_handle_t rpmsg_ipc_queue[RPMSG_IPC_QUEUE_NUM];

#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
static LOCK_STATIC_CONTEXT platform_lock_static_ctxt;
#endif


static int32_t ipc_queue_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    uint32_t vector_id;
    int32_t r;

    r = ipc_queue_get_user_data(handle, &vector_id);
    RL_ASSERT(r == 0);

    env_isr(vector_id);

    return 0;
}

static void platform_global_isr_disable(void)
{
    __asm volatile("cpsid i");
}

static void platform_global_isr_enable(void)
{
    __asm volatile("cpsie i");
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    ipc_queue_cfg_t q_cfg;
    int32_t r;
    uint8_t idx;

    if (platform_lock != ((void *)0))
    {
        /* Register ISR to environment layer */
        env_register_isr(vector_id, isr_data);

        env_lock_mutex(platform_lock);

        /* only support two queue */
        RL_ASSERT(isr_counter < RPMSG_IPC_QUEUE_NUM);

        q_cfg.qid = RPMSG_IPC_QUEUE_0 + (vector_id & 1);
        q_cfg.tx_buf_size = 0;
        q_cfg.tx_buf_addr = (uint32_t)NULL;
        q_cfg.tx_buf_addr_alias = (uint32_t)NULL;
        q_cfg.rx_buf_addr = (uint32_t)NULL;
        q_cfg.rx_ind = ipc_queue_rx_ind;
        q_cfg.user_data = vector_id;

        idx = vector_id & 1;
        rpmsg_ipc_queue[idx] = ipc_queue_init(&q_cfg);
        RL_ASSERT(IPC_QUEUE_INVALID_HANDLE != rpmsg_ipc_queue[idx]);

        r = ipc_queue_open2(rpmsg_ipc_queue[idx]);
        RL_ASSERT(0 == r);

        RL_ASSERT(0 <= isr_counter);
        isr_counter++;

        env_unlock_mutex(platform_lock);
        return 0;
    }
    else
    {
        return -1;
    }
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
    int32_t r;
    uint8_t idx;

    if (platform_lock != ((void *)0))
    {
        env_lock_mutex(platform_lock);

        RL_ASSERT(0 < isr_counter);

        idx = vector_id & 1;
        r = ipc_queue_close2(rpmsg_ipc_queue[idx]);
        RL_ASSERT(0 == r);

        r = ipc_queue_deinit(rpmsg_ipc_queue[idx]);
        RL_ASSERT(0 == r);
        rpmsg_ipc_queue[idx] = IPC_QUEUE_INVALID_HANDLE;

        isr_counter--;

        /* Unregister ISR from environment layer */
        env_unregister_isr(vector_id);

        env_unlock_mutex(platform_lock);

        return 0;
    }
    else
    {
        return -1;
    }
}

void platform_notify(uint32_t vector_id)
{
    /* Only single RPMsg-Lite instance (LINK_ID) is defined for this dual core device. Extend
       this statement in case multiple instances of RPMsg-Lite are needed. */
    switch (RL_GET_LINK_ID(vector_id))
    {
        case RL_PLATFORM_SF32_M33_M33_LINK_ID:
            env_lock_mutex(platform_lock);
/* Write directly into the Mailbox register, no need to wait until the content is cleared
   (consumed by the receiver side) because the same value of the virtqueue ID is written
   into this register when triggering the ISR for the receiver side. The whole queue of
   received buffers for associated virtqueue is handled in the ISR then. */
            ipc_queue_write(rpmsg_ipc_queue[vector_id & 1], NULL, 0, 0);

            env_unlock_mutex(platform_lock);
            return;

        default:
            return;
    }
}

/**
 * platform_time_delay
 *
 * @param num_msec Delay time in ms.
 *
 * This is not an accurate delay, it ensures at least num_msec passed when return.
 */
void platform_time_delay(uint32_t num_msec)
{
    HAL_Delay(num_msec);
}

/**
 * platform_in_isr
 *
 * Return whether CPU is processing IRQ
 *
 * @return True for IRQ, false otherwise.
 *
 */
int32_t platform_in_isr(void)
{
    return (((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0UL) ? 1 : 0);
}

/**
 * platform_interrupt_enable
 *
 * Enable peripheral-related interrupt
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_enable(uint32_t vector_id)
{
    RL_ASSERT(0 < disable_counter);

    platform_global_isr_disable();
    disable_counter--;

    if (disable_counter == 0)
    {
        NVIC_EnableIRQ(MAILBOX_IRQn);
    }
    platform_global_isr_enable();

    return ((int32_t)vector_id);
}

/**
 * platform_interrupt_disable
 *
 * Disable peripheral-related interrupt.
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_disable(uint32_t vector_id)
{
    RL_ASSERT(0 <= disable_counter);

    platform_global_isr_disable();
    /* virtqueues use the same NVIC vector
       if counter is set - the interrupts are disabled */
    if (disable_counter == 0)
    {
        NVIC_DisableIRQ(MAILBOX_IRQn);
    }
    disable_counter++;
    platform_global_isr_enable();

    return ((int32_t)vector_id);
}

/**
 * platform_map_mem_region
 *
 * Dummy implementation
 *
 */
void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags)
{
}

/**
 * platform_cache_all_flush_invalidate
 *
 * Dummy implementation
 *
 */
void platform_cache_all_flush_invalidate(void)
{
}

/**
 * platform_cache_disable
 *
 * Dummy implementation
 *
 */
void platform_cache_disable(void)
{
}

/**
 * platform_vatopa
 *
 * Dummy implementation
 *
 */
uintptr_t platform_vatopa(void *addr)
{
    return ((uintptr_t)(char *)addr);
}

/**
 * platform_patova
 *
 * Dummy implementation
 *
 */
void *platform_patova(uintptr_t addr)
{
    return ((void *)(char *)addr);
}

/**
 * platform_init
 *
 * platform/environment init
 */
int32_t platform_init(void)
{

    /* Create lock used in multi-instanced RPMsg */
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    if (0 != env_create_mutex(&platform_lock, 1, &platform_lock_static_ctxt))
#else
    if (0 != env_create_mutex(&platform_lock, 1))
#endif
    {
        return -1;
    }

    return 0;
}

/**
 * platform_deinit
 *
 * platform/environment deinit process
 */
int32_t platform_deinit(void)
{



    /* Delete lock used in multi-instanced RPMsg */
    env_delete_mutex(platform_lock);
    platform_lock = ((void *)0);
    return 0;
}
