/**
  ******************************************************************************
  * @file   ipc_hw_port.c
  * @author Sifli software development team
  * @brief Sibles ipc_hw source.
 *
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
        if (q_idx == 0)  /* the first queue is used by ble, it uses ROM implementation instead */
            ipc_queue_data_ind_rom(ipc_hw_obj.ch[0].data.user_data[q_idx]);
        else  /* other queue use RAM implementation */
            ipc_queue_data_ind(ipc_hw_obj.ch[0].data.user_data[q_idx]);
    }
}


#if LB55X_CHIP_ID >= 3
    __ROM_USED
#endif
void HCPU2LCPU_IRQHandler(void)
{
    /* enter interrupt */
    os_interrupt_enter();
    /* use replaced rx_handle to make RAM notification_callback version take effect*/
    HAL_MAILBOX_IRQHandler(&h2l_rx_handle);
    /* leave interrupt */
    os_interrupt_exit();
}



