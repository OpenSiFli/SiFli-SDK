/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include "ipc_hw.h"

__WEAK void ipc_queue_data_ind_rom(uint32_t user_data)
{
    ipc_queue_data_ind(user_data);
}

static void hcpu2lcpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx);

static MAILBOX_HandleTypeDef h2l_rx_handle =
{
    .Instance = H2L_MAILBOX,
    .NotificationCallback = hcpu2lcpu_notification_callback,
};

__ROM_USED ipc_hw_t ipc_hw_obj =
{
    .ch =
    {
        [0] =
        {
            .cfg =
            {
                .rx =
                {
                    .handle.Instance = H2L_MAILBOX,
                    .handle.NotificationCallback = hcpu2lcpu_notification_callback,
                    .core  = CORE_ID_HCPU,
                    .irqn = HCPU2LCPU_IRQn,
                },
                .tx =
                {
                    .handle.Instance = L2H_MAILBOX,
                    .core  = CORE_ID_LCPU,
                    .irqn = LCPU2HCPU_IRQn,
                }
            }
        },
    }
};

static void hcpu2lcpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx)
{
    SF_ASSERT(q_idx < IPC_HW_QUEUE_NUM);

    /* BLE code in ROM still use old ipc_queue implementation in ROM,
     i.e.
        BLE: rom_ipc_queue.c->rom_ipc_hw->rom_ipc_hw_obj
        User code in RAM: ram_ipc_queue.c->rom_ipc_hw->rom_ipc_hw_obj
        HCPU2LCPU_IRQHandler in RAM: notification_callback in RAM
            if (ble queue)
            {
                rom_ipc_queue_data_ind
            }
            else
            {
                ram_ipc_queue_data_ind
            }

    new implementation in RAM changes the ipc_queue handle value range
    compared to old implementation in ROM,
    i.e.
    ROM: ipc_queue invalid handle is -1, valid handle starts from 0
    RAM: ipc_queue invalid handle is 0, valid handle starts from 10
    */
    if (ipc_hw_obj.ch[0].data.act_bitmap & (1UL << q_idx))
    {
        if (q_idx == 6)  /* the bt sco audio queue, it uses ROM implementation instead */
            ipc_queue_data_ind_rom(ipc_hw_obj.ch[0].data.user_data[q_idx]);
        else  /* other queue use RAM implementation */
            ipc_queue_data_ind(ipc_hw_obj.ch[0].data.user_data[q_idx]);
    }
}

void HCPU2LCPU_IRQHandler(void)
{
    /* enter interrupt */
    os_interrupt_enter();
    /* use replaced rx_handle to make RAM notification_callback version take effect*/
    HAL_MAILBOX_IRQHandler(&h2l_rx_handle);
    /* leave interrupt */
    os_interrupt_exit();
}



