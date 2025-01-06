/**
  ******************************************************************************
  * @file   drv_mailbox.c
  * @author Sifli software development team
  * @brief HW Mailbox driver
 * @{
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

#include <board.h>

#if defined(BSP_USING_HWMAILBOX) || defined(_SIFLI_DOXYGEN_)

/** @defgroup drv_mailbox Hardware mailbox
 * @ingroup bsp_driver
 * @brief HW Mailbox driver
 * @{
 */


#include "drv_config.h"
#include "bf0_hal_mailbox.h"
#include "mailbox_config.h"

#define LOG_TAG             "drv.mailbox"
#include <drv_log.h>

#define MAILBOX_TIMEOUT 1000            // In ms

/** @brief hw mailbox struct type
 *
 */
struct bf0_hwmailbox
{
    rt_hwmailbox_t device;
    MAILBOX_HandleTypeDef handle;
    uint8_t core;
    uint8_t channel;
    IRQn_Type irqn;
    char *name;
    bool valid;
};

/** @brief bidirectional hw mailbox struct type
 *
 */
struct bf0_bidir_hwmailbox
{
    rt_bidir_hwmailbox_t device;
    char *name;
    char *rx_dev_name;
    char *tx_dev_name;
};

/** @brief hw mailbox objects list
 *
 */
static struct bf0_hwmailbox bf0_hwmailbox_obj[] =
{

#ifdef BSP_USING_MB_L2H_CH1
    MAILBOX_L2H_CH1_CONFIG,
#endif

#ifdef BSP_USING_MB_H2L_CH1
    MAILBOX_H2L_CH1_CONFIG,
#endif

#ifdef BSP_USING_MB_L2H_CH2
    MAILBOX_L2H_CH2_CONFIG,
#endif

#ifdef BSP_USING_MB_H2L_CH2
    MAILBOX_H2L_CH2_CONFIG,
#endif

};

/** @brief bidirectional hw mailbox objects list
 *
 */
static struct bf0_bidir_hwmailbox bf0_bidir_hwmailbox_obj[] =
{
#ifdef BSP_USING_BIDIR_MB1
    BIDIR_MB_1_CONFIG,
#endif /* BSP_USING_BIDIR_MB1 */

#ifdef BSP_USING_BIDIR_MB3
    BIDIR_MB_3_CONFIG,
#endif /* BSP_USING_BIDIR_MB3 */

    {0}
};


#ifdef RT_USING_PM

static int rt_hwmailbox_suspend(const struct rt_device *device, uint8_t mode)
{
    rt_hwmailbox_t *mailbox;
    int r = RT_EOK;

    mailbox = (rt_hwmailbox_t *)device;
    if (device->flag & RT_DEVICE_FLAG_ACTIVATED)
    {
        if (rt_ringbuffer_data_len(mailbox->ring_buffer) > 0)
        {
            r = -RT_EBUSY;
        }
    }

    return r;
}

static void rt_hwmailbox_resume(const struct rt_device *device, uint8_t mode)
{
    return ;
}

static const struct rt_device_pm_ops hwmailbox_pm_op =
{
    .suspend = rt_hwmailbox_suspend,
    .resume = rt_hwmailbox_resume,
};
#endif  /* RT_USING_PM */

/** @brief mailbox init
 *
 * @details do nothing
 *
 * @param[in] mailbox mailbox instance
 *
 */
static void mailbox_init(rt_hwmailbox_t *mailbox)
{

}

/** @brief trigger mailbox interrupt
 *
 *
 * @param[in] mailbox mailbox instance
 *
 */
static void mailbox_trigger(rt_hwmailbox_t *mailbox)
{
    struct bf0_hwmailbox *mb;

    mb = (struct bf0_hwmailbox *)mailbox->parent.user_data;
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&mb->handle, mb->channel);
}

/** @brief mailbox control
 *
 * @details use cmd: RT_DEVICE_CTRL_SET_INT to enable mailbox interrupt ,
 *             cmd: RT_DEVICE_CTRL_CLEAR_INT to disable mailbox interrupt
 *
 * @param[in] mailbox mailbox instance
 * @param[in] cmd command, could be RT_DEVICE_CTRL_SET_INT or RT_DEVICE_CTRL_CLR_INT
 * @param[in] arg not used
 *
 */
static rt_err_t mailbox_ctrl(rt_hwmailbox_t *mailbox, rt_uint32_t cmd, void *arg)
{
    struct bf0_hwmailbox *mb = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(mailbox != RT_NULL);

    mb = (struct bf0_hwmailbox *)mailbox->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SET_INT:
    {
        if (mb->irqn >= 0)
        {
            /* set the mailbox priority */
            HAL_NVIC_SetPriority(mb->irqn, 3, 0);
            /* enable the mailbox global Interrupt */
            HAL_NVIC_EnableIRQ(mb->irqn);
        }
        else
        {
            /* sender unmask interrupt */
            __HAL_MAILBOX_UNMASK_CHANNEL_IT(&mb->handle, mb->channel);
#ifdef RT_USING_PM
            rt_pm_device_register(&mailbox->parent, &hwmailbox_pm_op);
#endif /* RT_USING_PM */
        }

    }

    break;
    case RT_DEVICE_CTRL_CLR_INT:
    {
        if (mb->irqn >= 0)
        {
            uint32_t i;
            bool disable_allowed = true;

            for (i = 0; i < sizeof(bf0_hwmailbox_obj) / sizeof(bf0_hwmailbox_obj[0]); i++)
            {
                if (bf0_hwmailbox_obj[i].valid
                        && (mb->handle.Instance == bf0_hwmailbox_obj[i].handle.Instance)
                        && (bf0_hwmailbox_obj[i].device.parent.ref_count > 0))
                {
                    /* other channels of the same instance are being used */
                    disable_allowed = false;
                    break;
                }
            }
            if (disable_allowed)
            {
                /* disable the mailbox global Interrupt */
                HAL_NVIC_DisableIRQ(mb->irqn);
            }
        }
        else
        {
            __HAL_MAILBOX_MASK_CHANNEL_IT(&mb->handle, mb->channel);
#ifdef RT_USING_PM
            rt_pm_device_unregister(&mailbox->parent);
#endif /* RT_USING_PM */
        }
    }
    break;
    default:
    {
        result = -RT_ENOSYS;
    }
    break;
    }

    return result;
}


/** @brief mailbox lock
 * *
 * @param[in] mailbox mailbox instance
 * @param[in] lock 1 to lock, 0 to unlock
 *
 */
static void mailbox_lock(rt_hwmailbox_t *mailbox, uint8_t lock)
{
#ifdef SOC_BF_Z0
    struct bf0_hwmailbox *mb = RT_NULL;
    MUTEX_HandleTypeDef handle;

    RT_ASSERT(mailbox != RT_NULL);
    mb = (struct bf0_hwmailbox *)mailbox->parent.user_data;
    handle.Instance = HAL_MAILBOX_GetMutex(mb->core, mb->channel);
    if (lock)
        HAL_MAILBOX_LockEx(&handle, mb->channel, MAILBOX_TIMEOUT);
    else
        HAL_MAILBOX_UnLock(&handle, mb->channel);
#endif
}

/** @brief mailbox operations
 *
 */
static const struct rt_hwmailbox_ops _ops =
{
    .init = mailbox_init,
    .trigger = mailbox_trigger,
    .control = mailbox_ctrl,
    .lock = mailbox_lock,
};

/** @brief find mailbox object according to instance address and channel index
 *
 * @param[in] instance
 * @param[in] channel_idx
 *
 * @return mailbox object
 *
 */
static struct bf0_hwmailbox *mailbox_find(MAILBOX_CH_TypeDef *instance, uint8_t channel_idx)
{
    uint32_t i;
    struct bf0_hwmailbox *mailbox = RT_NULL;

    for (i = 0; i < sizeof(bf0_hwmailbox_obj) / sizeof(bf0_hwmailbox_obj[0]); i++)
    {
        if ((instance == bf0_hwmailbox_obj[i].handle.Instance)
                && (channel_idx == bf0_hwmailbox_obj[i].channel))
        {
            mailbox = &bf0_hwmailbox_obj[i];
            break;
        }

    }

    return mailbox;
}

/** @brief mailbox isr
 *
 * @param[in] instance
 *
 */
static void mailbox_isr(MAILBOX_CH_TypeDef *instance)
{
    uint32_t status;
    uint32_t clear_status;
    uint8_t channel_idx;
    struct bf0_hwmailbox *mailbox;

    status = instance->CxMISR;
    clear_status = status;
    instance->CxICR = clear_status;
    channel_idx = 0;
    while (status)
    {
        if (1 & status)
        {
            mailbox = mailbox_find(instance, channel_idx);
            if ((RT_NULL != mailbox) && (mailbox->device.parent.open_flag & RT_DEVICE_OFLAG_RDONLY))
            {
                rt_device_hwmailbox_isr(&mailbox->device);
            }
        }
        status >>= 1;
        channel_idx++;
    }
}

/** @brief LCPU2HCPU Mailbox IRQ Handler
 *
 *
 */
void LCPU2HCPU_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    mailbox_isr(L2H_MAILBOX);
    /* leave interrupt */
    rt_interrupt_leave();
}

/** @brief HCPU2LCPU Mailbox IRQ Handler
 *
 *
 */
__ROM_USED void HCPU2LCPU_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    mailbox_isr(H2L_MAILBOX);
    /* leave interrupt */
    rt_interrupt_leave();
}

/** @brief hw mailbox init
 *
 * @details register configured hw mailbox device
 *
 * @return result RT_EOK success, otherwise fail
 *
 */
__ROM_USED int rt_hw_mailbox_init(void)
{
    int i = 0;
    int result = RT_EOK;
    uint32_t flag;

    for (i = 0; i < sizeof(bf0_hwmailbox_obj) / sizeof(bf0_hwmailbox_obj[0]); i++)
    {
        bf0_hwmailbox_obj[i].device.ops  = &_ops;
        if (bf0_hwmailbox_obj[i].irqn >= 0)
        {
            flag = RT_DEVICE_FLAG_RDONLY;
        }
        else
        {
            flag = RT_DEVICE_FLAG_WRONLY;
        }
        if (rt_device_hwmailbox_register(&bf0_hwmailbox_obj[i].device,
                                         bf0_hwmailbox_obj[i].name,
                                         flag,
                                         &bf0_hwmailbox_obj[i]) == RT_EOK)
        {
            //LOG_D("%s register success", bf0_hwmailbox_obj[i].name);
            bf0_hwmailbox_obj[i].valid = true;
        }
        else
        {
            //LOG_E("%s register failed", bf0_hwmailbox_obj[i].name);
            bf0_hwmailbox_obj[i].valid = false;
            result = -RT_ERROR;
        }
    }

    for (i = 0; i < sizeof(bf0_bidir_hwmailbox_obj) / sizeof(bf0_bidir_hwmailbox_obj[0]) - 1; i++)
    {
        bf0_bidir_hwmailbox_obj[i].device.rx_device = (rt_hwmailbox_t *)rt_device_find(bf0_bidir_hwmailbox_obj[i].rx_dev_name);
        bf0_bidir_hwmailbox_obj[i].device.tx_device = (rt_hwmailbox_t *)rt_device_find(bf0_bidir_hwmailbox_obj[i].tx_dev_name);
        if (rt_device_bidir_hwmailbox_register(&bf0_bidir_hwmailbox_obj[i].device,
                                               bf0_bidir_hwmailbox_obj[i].name,
                                               RT_DEVICE_FLAG_RDWR,
                                               &bf0_bidir_hwmailbox_obj[i]) == RT_EOK)
        {
        }
        else
        {
            result = -RT_ERROR;
        }
    }

    return result;
}
//INIT_DEVICE_EXPORT(rt_hw_mailbox_init);


/// @} drv_mailbox

/// @} file


#endif /* BSP_USING_HWMAILBOX */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
