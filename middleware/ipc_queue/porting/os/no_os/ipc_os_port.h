/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IPC_OS_PORT_H
#define IPC_OS_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#define os_interrupt_disable()        (0)

#define os_interrupt_enable(mask)

#define os_interrupt_enter()
#define os_interrupt_exit()

#define os_interrupt_start(irq_number,priority,sub_priority) \
{ \
    HAL_NVIC_SetPriority(irq_number, priority, sub_priority); \
    HAL_NVIC_EnableIRQ(irq_number); \
}

#define os_interrupt_stop(irq_number) \
     HAL_NVIC_DisableIRQ(irq_number);


/// @}  file

#ifdef __cplusplus
}
#endif

/// @}
#endif

