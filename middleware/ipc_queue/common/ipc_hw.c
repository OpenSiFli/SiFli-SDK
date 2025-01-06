/**
  ******************************************************************************
  * @file   ipc_hw.c
  * @author Sifli software development team
  * @brief Sibles ipc library source.
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
#include <string.h>
#include "ipc_queue.h"
#include "ipc_hw.h"



/** @brief trigger mailbox interrupt
 *
 * @param[in] hw_q_handle hw queue handle
 *
 */
void ipc_hw_trigger_interrupt(ipc_hw_q_handle_t *hw_q_handle)
{
    ipc_hw_ch_t *ch;

    SF_ASSERT(hw_q_handle->ch_id < IPC_HW_CH_NUM);
    SF_ASSERT(hw_q_handle->q_idx < IPC_HW_QUEUE_NUM);
    ch = &ipc_hw_obj.ch[hw_q_handle->ch_id];

    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&ch->cfg.tx.handle, hw_q_handle->q_idx);
}

int32_t ipc_hw_check_interrupt(ipc_hw_q_handle_t *hw_q_handle)
{
    ipc_hw_ch_t *ch;

    SF_ASSERT(hw_q_handle->ch_id < IPC_HW_CH_NUM);
    SF_ASSERT(hw_q_handle->q_idx < IPC_HW_QUEUE_NUM);
    ch = &ipc_hw_obj.ch[hw_q_handle->ch_id];

    return __HAL_MAILBOX_CHECK_CHANNEL_IT(&ch->cfg.tx.handle, hw_q_handle->q_idx);
}

/** @brief enable interrupt
 *
 *
 * @param[out] hw_q_handle hw queue handle
 * @param[in] qid queue id, numbering across all queues in all channels
 * @param[in] user_data user data
 * @retval status, 0: success, otherwise: fail
 *
 */
int32_t ipc_hw_enable_interrupt(ipc_hw_q_handle_t *hw_q_handle, uint8_t qid, uint32_t user_data)
{
    uint8_t ch_id;
    int32_t result = -1;
    uint8_t q_idx;
    ipc_hw_ch_t *ch;
    ipc_ch_cfg_t *ch_cfg;

    SF_ASSERT(hw_q_handle);

    ch_id = qid / IPC_HW_QUEUE_NUM;
    if (ch_id >= IPC_HW_CH_NUM)
    {
        return result;
    }

    ch = &ipc_hw_obj.ch[ch_id];
    ch_cfg = &ch->cfg;
    q_idx = qid - ch_id * IPC_HW_QUEUE_NUM;
    ch->data.user_data[q_idx] = user_data;
    ch->data.act_bitmap |= (1UL << q_idx);
    hw_q_handle->ch_id = ch_id;
    hw_q_handle->q_idx = q_idx;

    /* receiver enable NVIC IRQ */
    /* set the mailbox priority */
    HAL_NVIC_SetPriority(ch_cfg->rx.irqn, 3, 0);
    /* enable the mailbox global Interrupt */
    HAL_NVIC_EnableIRQ(ch_cfg->rx.irqn);

    /* sender unmask interrupt */
    __HAL_MAILBOX_UNMASK_CHANNEL_IT(&ch_cfg->tx.handle, q_idx);

    result = 0;

    return result;
}

int32_t ipc_hw_enable_interrupt2(ipc_hw_q_handle_t *hw_q_handle, uint8_t qid, uint32_t user_data)
{
    uint8_t ch_id;
    int32_t result = -1;
    uint8_t q_idx;
    ipc_hw_ch_t *ch;
    ipc_ch_cfg_t *ch_cfg;

    SF_ASSERT(hw_q_handle);

    ch_id = qid / IPC_HW_QUEUE_NUM;
    if (ch_id >= IPC_HW_CH_NUM)
    {
        return result;
    }

    ch = &ipc_hw_obj.ch[ch_id];
    ch_cfg = &ch->cfg;
    q_idx = qid - ch_id * IPC_HW_QUEUE_NUM;
    ch->data.user_data[q_idx] = user_data;
    ch->data.act_bitmap |= (1UL << q_idx);
    hw_q_handle->ch_id = ch_id;
    hw_q_handle->q_idx = q_idx;

    /* receiver enable NVIC IRQ */
    /* set the mailbox priority */
    HAL_NVIC_SetPriority(ch_cfg->rx.irqn, 3, 0);
    /* enable the mailbox global Interrupt */
    HAL_NVIC_EnableIRQ(ch_cfg->rx.irqn);

    /* receiver unmask interrupt */
#ifdef SOC_BF0_HCPU
    SF_ASSERT(ch_cfg->rx.core == CORE_ID_LCPU);
#ifdef SF32LB52X
    HAL_HPAON_WakeCore(ch_cfg->rx.core);
#endif /* SF32LB52X */
    /* receiver unmask interrupt */
    __HAL_MAILBOX_UNMASK_CHANNEL_IT(&ch_cfg->rx.handle, q_idx);
#ifdef SF32LB52X
    //TODO: how about other chip support reference counting? For 55x/56x/58x LCPU is always active when HCPU is active
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif /* SF32LB52X */
#elif defined(SOC_BF0_LCPU)
    SF_ASSERT(ch_cfg->rx.core == CORE_ID_HCPU);
    HAL_LPAON_WakeCore(ch_cfg->rx.core);
    /* receiver unmask interrupt */
    __HAL_MAILBOX_UNMASK_CHANNEL_IT(&ch_cfg->rx.handle, q_idx);
    HAL_LPAON_CANCEL_HP_ACTIVE_REQUEST();
#else
#error "Invalid core"
#endif

    result = 0;

    return result;
}


/** @brief disable interrupt
 *
 *
 * @param[in] hw_q_handle hw queue handle
 * @retval status, 0: success, otherwise: fail
 *
 */
int32_t ipc_hw_disable_interrupt(ipc_hw_q_handle_t *hw_q_handle)
{
    uint8_t ch_id;
    uint8_t q_idx;
    ipc_hw_ch_t *ch;
    ipc_ch_cfg_t *ch_cfg;

    SF_ASSERT(hw_q_handle);

    ch_id = hw_q_handle->ch_id;
    SF_ASSERT(ch_id < IPC_HW_CH_NUM);
    ch = &ipc_hw_obj.ch[ch_id];
    ch_cfg = &ch->cfg;
    q_idx = hw_q_handle->q_idx;
    SF_ASSERT(q_idx < IPC_HW_QUEUE_NUM);
    ch->data.act_bitmap &= ~(1UL << q_idx);

    /* rx */
    if (0 == ch->data.act_bitmap)
    {
        /* disable the mailbox global Interrupt */
        HAL_NVIC_DisableIRQ(ch_cfg->rx.irqn);
    }

    /* tx */
    __HAL_MAILBOX_MASK_CHANNEL_IT(&ch_cfg->tx.handle, q_idx);

#if 0
#ifdef RT_USING_PM
    rt_pm_device_unregister(&mailbox->parent);
#endif /* RT_USING_PM */
#endif
    return 0;
}

int32_t ipc_hw_disable_interrupt2(ipc_hw_q_handle_t *hw_q_handle)
{
    uint8_t ch_id;
    uint8_t q_idx;
    ipc_hw_ch_t *ch;
    ipc_ch_cfg_t *ch_cfg;

    SF_ASSERT(hw_q_handle);

    ch_id = hw_q_handle->ch_id;
    SF_ASSERT(ch_id < IPC_HW_CH_NUM);
    ch = &ipc_hw_obj.ch[ch_id];
    ch_cfg = &ch->cfg;
    q_idx = hw_q_handle->q_idx;
    SF_ASSERT(q_idx < IPC_HW_QUEUE_NUM);
    ch->data.act_bitmap &= ~(1UL << q_idx);

    /* rx */
    if (0 == ch->data.act_bitmap)
    {
        /* disable the mailbox global Interrupt */
        HAL_NVIC_DisableIRQ(ch_cfg->rx.irqn);
    }

    /* receiver mask the interrupt */
#ifdef SOC_BF0_HCPU
    SF_ASSERT(ch_cfg->rx.core == CORE_ID_LCPU);
#ifdef SF32LB52X
    HAL_HPAON_WakeCore(ch_cfg->rx.core);
#endif /* SF32LB52X */
    /* receiver unmask interrupt */
    __HAL_MAILBOX_MASK_CHANNEL_IT(&ch_cfg->rx.handle, q_idx);
#ifdef SF32LB52X
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#endif /* SF32LB52X */
#elif defined(SOC_BF0_LCPU)
    SF_ASSERT(ch_cfg->rx.core == CORE_ID_HCPU);
    HAL_LPAON_WakeCore(ch_cfg->rx.core);
    /* receiver unmask interrupt */
    __HAL_MAILBOX_MASK_CHANNEL_IT(&ch_cfg->rx.handle, q_idx);
    HAL_LPAON_CANCEL_HP_ACTIVE_REQUEST();
#else
#error "Invalid core"
#endif

    return 0;
}

