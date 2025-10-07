/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include "ipc_hw.h"

static void lcpu2hcpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx);

ipc_hw_t ipc_hw_obj =
{
    .ch =
    {
        [0] =
        {
            .cfg =
            {
                .rx =
                {
                    .handle.Instance = L2H_MAILBOX,
                    .handle.NotificationCallback = lcpu2hcpu_notification_callback,
                    .core  = CORE_ID_LCPU,
                    .irqn = LCPU2HCPU_IRQn,
                },
                .tx =
                {
                    .handle.Instance = H2L_MAILBOX,
                    .core  = CORE_ID_HCPU,
                    .irqn = HCPU2LCPU_IRQn,

                }
            }
        },
    }
};

static void lcpu2hcpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx)
{
    SF_ASSERT(q_idx < IPC_HW_QUEUE_NUM);

    if (ipc_hw_obj.ch[0].data.act_bitmap & (1UL << q_idx))
    {
        ipc_queue_data_ind(ipc_hw_obj.ch[0].data.user_data[q_idx]);
    }
}

void LCPU2HCPU_IRQHandler(void)
{
    /* enter interrupt */
    os_interrupt_enter();
    HAL_MAILBOX_IRQHandler(&ipc_hw_obj.ch[0].cfg.rx.handle);
    /* leave interrupt */
    os_interrupt_exit();
}



