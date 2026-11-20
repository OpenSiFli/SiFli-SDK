/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SIFLI_M33_CPUPORT_H__
#define __SIFLI_M33_CPUPORT_H__

#include <rthw.h>

#ifdef RT_USING_SMP
/* Existing workqueue headers expect this architecture type. The prototype
 * deliberately serializes these locks with the same recursive kernel lock. */
typedef struct
{
    rt_ubase_t reserved;
} rt_hw_spinlock_t;

    #define rt_spin_lock_init(lock) ((void)(lock))
    #define rt_spin_lock_irqsave(lock) ((void)(lock), rt_cpus_lock())
    #define rt_spin_unlock_irqrestore(lock, level) ((void)(lock), rt_cpus_unlock(level))
#endif /* RT_USING_SMP */

#endif /* __SIFLI_M33_CPUPORT_H__ */
