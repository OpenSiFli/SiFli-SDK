/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
    os_interrupt_start(ch_cfg->rx.irqn, 3, 0);

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
    os_interrupt_start(ch_cfg->rx.irqn, 3, 0);

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
        os_interrupt_stop(ch_cfg->rx.irqn);
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
        os_interrupt_stop(ch_cfg->rx.irqn);
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

