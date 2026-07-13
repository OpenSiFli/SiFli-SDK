/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv_can.h"

#ifdef BSP_USING_CAN

#define DBG_LEVEL            DBG_LOG
#define LOG_TAG              "drv.can"
#include <drv_log.h>

static struct bf0_can_config bf0_can_cfg[] =
{
#ifdef BSP_USING_CAN1
    {
        .device_name = "can1",
        .Instance    = hwp_can1,
        .irq_type    = CAN1_IRQn,
    },
#endif
#ifdef BSP_USING_CAN2
    {
        .device_name = "can2",
        .Instance    = hwp_can2,
        .irq_type    = CAN2_IRQn,
    },
#endif
};

static struct bf0_can bf0_can_obj[CAN_MAX];

/* ─── Forward declarations ──────────────────────────────────────────────── */
static void _can_irq_handler(struct bf0_can *drv_can);

/* ─── CAN operations implementation ─────────────────────────────────────── */

/**
 * @brief Configure CAN controller with given parameters.
 */
static rt_err_t _can_config(struct rt_can_device *can, struct can_configure *cfg)
{
    struct bf0_can *drv_can;
    int32_t err;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    drv_can = (struct bf0_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    err = HAL_CAN_CalcTiming(&drv_can->CanHandle, cfg->baud_rate);
    RT_ASSERT(err >= 0);

    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        drv_can->CanHandle.Init.Mode = CAN_MODE_NORMAL;
        break;
    case RT_CAN_MODE_LISTEN:
        drv_can->CanHandle.Init.Mode = CAN_MODE_SILENT;
        break;
    case RT_CAN_MODE_LOOPBACK:
        drv_can->CanHandle.Init.Mode = CAN_MODE_LOOPBACK;
        break;
    case RT_CAN_MODE_LOOPBACKANLISTEN:
        drv_can->CanHandle.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
        break;
    default:
        return -RT_EINVAL;
    }

    /* Initialize CAN peripheral */
    if (HAL_CAN_Init(&drv_can->CanHandle) != HAL_OK)
    {
        LOG_E("%s: HAL_CAN_Init failed", drv_can->name);
        return -RT_ERROR;
    }

    /* Apply default filter */
    HAL_CAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);

    HAL_CAN_Start(&drv_can->CanHandle);

    return RT_EOK;
}

/**
 * @brief Send one CAN message to hardware (PTB).
 *
 * @note For SiFli CAN, the hardware has only one PTB (Primary Transmit Buffer).
 *       The @p boxno parameter is preserved for API compatibility but is not
 *       used to select hardware mailboxes (all messages go through PTB).
 */
static rt_ssize_t _can_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    struct bf0_can *drv_can;
    struct rt_can_msg *pmsg;
    CAN_TxHeaderTypeDef txheader = {0};
    HAL_StatusTypeDef status;

    RT_ASSERT(can);
    RT_ASSERT(buf);

    drv_can = (struct bf0_can *)can->parent.user_data;
    pmsg    = (struct rt_can_msg *)buf;

    /* Validate DLC */
    if (pmsg->len > 8)
    {
        return -RT_EINVAL;
    }

    /* Check CAN state */
    if (drv_can->CanHandle.State != HAL_CAN_STATE_READY &&
            drv_can->CanHandle.State != HAL_CAN_STATE_LISTENING)
    {
        drv_can->CanHandle.ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;
        return -RT_ERROR;
    }

    /* Build TX header */
    if (pmsg->ide == RT_CAN_STDID)
    {
        txheader.IDE   = CAN_ID_STD;
        txheader.StdId = pmsg->id;
    }
    else
    {
        txheader.IDE   = CAN_ID_EXT;
        txheader.ExtId = pmsg->id;
    }

    txheader.RTR = (pmsg->rtr == RT_CAN_DTR) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
    txheader.DLC = pmsg->len;

    status = HAL_CAN_AddSecondaryTxMessage(&drv_can->CanHandle, &txheader, pmsg->data);
    if (status != HAL_OK)
    {
        LOG_W("%s: sendmsg failed, status=%d", drv_can->name, status);
        return -RT_ERROR;
    }

    (void)boxno;
    return RT_EOK;
}

/**
 * @brief Receive one CAN message from hardware RBUF.
 */
static rt_ssize_t _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    struct bf0_can *drv_can;
    struct rt_can_msg *pmsg;
    CAN_RxHeaderTypeDef rxheader = {0};
    HAL_StatusTypeDef status;

    RT_ASSERT(can);
    RT_ASSERT(buf);

    drv_can = (struct bf0_can *)can->parent.user_data;
    pmsg    = (struct rt_can_msg *)buf;

    /* Read one message from hardware RX FIFO */
    status = HAL_CAN_GetRxMessage(&drv_can->CanHandle, &rxheader, pmsg->data);
    if (status != HAL_OK)
    {
        return -RT_ERROR;
    }

    /* Empty FIFO */
    if (rxheader.DLC == 0 && rxheader.IDE == 0 && rxheader.RTR == 0)
    {
        return 0;
    }

    /* Convert to RT-Thread CAN message */
    if (rxheader.IDE == CAN_ID_STD)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id  = rxheader.StdId;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id  = rxheader.ExtId;
    }

    pmsg->rtr     = (rxheader.RTR == CAN_RTR_DATA) ? RT_CAN_DTR : RT_CAN_RTR;
    pmsg->len     = rxheader.DLC;
    pmsg->rxfifo  = fifo;
    pmsg->hdr_index = -1;  /* SiFli CAN doesn't expose filter match index */

    return RT_EOK;
}

/**
 * @brief Control CAN device (set/get parameters, interrupts, etc.)
 */
static rt_err_t _can_control(struct rt_can_device *can, int cmd, void *arg)
{
    struct bf0_can *drv_can;
    rt_uint32_t argval;
    rt_err_t res = RT_EOK;

    RT_ASSERT(can);
    drv_can = (struct bf0_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    switch (cmd)
    {
    /* ── Interrupt control ──────────────────────────────────────────── */
    case RT_DEVICE_CTRL_CLR_INT:
        argval = (rt_uint32_t)(rt_ubase_t)arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            HAL_CAN_DeactivateNotification(&drv_can->CanHandle,
                                           CAN_IT_RX_MSG_PENDING | CAN_IT_RX_FIFO_FULL |
                                           CAN_IT_RX_OVERRUN | CAN_IT_RX_ALMOST_FULL);
            NVIC_DisableIRQ(drv_can->config->irq_type);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            HAL_CAN_DeactivateNotification(&drv_can->CanHandle,
                                           CAN_IT_TX_PTB_EMPTY | CAN_IT_TX_STB_EMPTY);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            HAL_CAN_DeactivateNotification(&drv_can->CanHandle,
                                           CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE);
        }
        /* Check if any interrupts still active; if none, disable NVIC */
        if ((drv_can->CanHandle.Instance->IR &
                (CAN_IR_RIE  | CAN_IR_RFIE | CAN_IR_ROIE | CAN_IR_RAFIE |
                 CAN_IR_TPIE | CAN_IR_TSIE |
                 CAN_IR_EIE  | CAN_IR_BEIE | CAN_IR_EPIE)) == 0)
        {
            NVIC_DisableIRQ(drv_can->config->irq_type);
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t)(rt_ubase_t)arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            HAL_CAN_ActivateNotification(&drv_can->CanHandle,
                                         CAN_IT_RX_MSG_PENDING | CAN_IT_RX_FIFO_FULL |
                                         CAN_IT_RX_OVERRUN | CAN_IT_RX_ALMOST_FULL);
            HAL_NVIC_SetPriority(drv_can->config->irq_type, 1, 0);
            NVIC_EnableIRQ(drv_can->config->irq_type);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            HAL_CAN_ActivateNotification(&drv_can->CanHandle,
                                         CAN_IT_TX_PTB_EMPTY | CAN_IT_TX_STB_EMPTY);
            HAL_NVIC_SetPriority(drv_can->config->irq_type, 1, 0);
            NVIC_EnableIRQ(drv_can->config->irq_type);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            HAL_CAN_ActivateNotification(&drv_can->CanHandle,
                                         CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE |
                                         CAN_IT_ARBITRATION_LOST_ERROR);
            HAL_NVIC_SetPriority(drv_can->config->irq_type, 1, 0);
            NVIC_EnableIRQ(drv_can->config->irq_type);
        }
        break;

    /* ── Filter configuration ───────────────────────────────────────── */
    case RT_CAN_CMD_SET_FILTER:
    {
        struct rt_can_filter_config *filter_cfg;
        struct rt_can_filter_item  *pitem;
        rt_uint32_t count;

        if (RT_NULL == arg)
        {
            /* Reset to default filter */
            HAL_CAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);
        }
        else
        {
            filter_cfg = (struct rt_can_filter_config *)arg;
            RT_ASSERT(filter_cfg);
            count = filter_cfg->count;
            pitem = filter_cfg->items;

            while (count)
            {
                /* Validate filter bank index */
                if (pitem->hdr_bank >= 16 || pitem->hdr_bank < -1)
                {
                    count--;
                    pitem++;
                    continue;
                }

                /* If hdr_bank == -1, use count-1 as the bank index
                   (simple fallback: sequential assignment) */
                drv_can->FilterConfig.FilterBank =
                    (pitem->hdr_bank == -1) ? (filter_cfg->count - count) : pitem->hdr_bank;

                /* Build 29-bit filter ID */
                drv_can->FilterConfig.FilterId = pitem->id & 0x1FFFFFFF;

                /* Build 29-bit filter mask (inverted by HAL_CAN_ConfigFilter) */
                drv_can->FilterConfig.FilterMask = pitem->mask;

                /* IDE check */
                drv_can->FilterConfig.IDECheckEnable = ENABLE;
                drv_can->FilterConfig.IDEValue =
                    (pitem->ide == RT_CAN_EXTID) ? CAN_ID_EXT : CAN_ID_STD;

                /* Filter activation */
                drv_can->FilterConfig.FilterActivation =
                    (filter_cfg->actived) ? CAN_FILTER_ENABLE : CAN_FILTER_DISABLE;

                /* Apply filter */
                HAL_CAN_ConfigFilter(&drv_can->CanHandle, &drv_can->FilterConfig);

                count--;
                pitem++;
            }
        }
        break;
    }

    /* ── Mode setting ───────────────────────────────────────────────── */
    case RT_CAN_CMD_SET_MODE:
        argval = (rt_uint32_t)(rt_ubase_t)arg;
        if (argval != RT_CAN_MODE_NORMAL &&
                argval != RT_CAN_MODE_LISTEN &&
                argval != RT_CAN_MODE_LOOPBACK &&
                argval != RT_CAN_MODE_LOOPBACKANLISTEN)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.mode)
        {
            drv_can->device.config.mode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;

    /* ── Baud rate setting ──────────────────────────────────────────── */
    case RT_CAN_CMD_SET_BAUD:
        argval = (rt_uint32_t)(rt_ubase_t)arg;
        if (argval != CAN1MBaud   &&
                argval != CAN800kBaud &&
                argval != CAN500kBaud &&
                argval != CAN250kBaud &&
                argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                argval != CAN50kBaud  &&
                argval != CAN20kBaud  &&
                argval != CAN10kBaud)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.baud_rate)
        {
            drv_can->device.config.baud_rate = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;

    /* ── Privileged mode ────────────────────────────────────────────── */
    case RT_CAN_CMD_SET_PRIV:
        argval = (rt_uint32_t)(rt_ubase_t)arg;
        if (argval != RT_CAN_MODE_PRIV &&
                argval != RT_CAN_MODE_NOPRIV)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.privmode)
        {
            drv_can->device.config.privmode = argval;
        }
        break;

    /* ── Get status ─────────────────────────────────────────────────── */
    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t cnt_reg;

        cnt_reg = drv_can->CanHandle.Instance->CNT;
        drv_can->device.status.rcverrcnt = (cnt_reg >> 16) & 0xFF;
        drv_can->device.status.snderrcnt = (cnt_reg >> 24) & 0xFF;

        cnt_reg = drv_can->CanHandle.Instance->IR;
        drv_can->device.status.errcode = 0;
        if (cnt_reg & CAN_IR_EWARN) drv_can->device.status.errcode = 1;
        if (cnt_reg & CAN_IR_EPASS) drv_can->device.status.errcode = 2;
        if (cnt_reg & CAN_IR_BEIF)  drv_can->device.status.errcode = 4;

        rt_memcpy(arg, &drv_can->device.status, sizeof(drv_can->device.status));
        break;
    }

    /* ── Start / Stop ───────────────────────────────────────────────── */
    case RT_CAN_CMD_START:
        argval = (rt_uint32_t)(rt_ubase_t)arg;
        if (argval == 0)
        {
            HAL_CAN_Stop(&drv_can->CanHandle);
        }
        else
        {
            HAL_CAN_Start(&drv_can->CanHandle);
        }
        break;

    /* ── Delegate to HAL control ────────────────────────────────────── */
    default:
        res = -RT_ENOSYS;
        break;
    }

    return res;
}

/* ─── CAN operations table ──────────────────────────────────────────────── */

static const struct rt_can_ops _can_ops =
{
    _can_config,
    _can_control,
    _can_sendmsg,
    _can_recvmsg,
};


/* ─── HAL CAN callback overrides ────────────────────────────────────────── */

/**
 * @brief Override HAL CAN RxMsgPending callback.
 * Drains RX FIFO and forwards to RT-Thread CAN framework.
 */
void HAL_CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    struct bf0_can *drv_can = rt_container_of(hcan, struct bf0_can, CanHandle);
    rt_base_t level;

    /* Drain all pending RX frames */
    while (HAL_CAN_GetRxFifoFillLevel(hcan))
    {
        rt_hw_can_isr(&drv_can->device, RT_CAN_EVENT_RX_IND | (0 << 8));
    }

    /* Check overflow */
    if (hcan->Instance->CR & CAN_CR_ROV)
    {
        level = rt_hw_interrupt_disable();
        drv_can->device.status.dropedrcvpkg++;
        rt_hw_interrupt_enable(level);

        rt_hw_can_isr(&drv_can->device, RT_CAN_EVENT_RXOF_IND | (0 << 8));
    }
}

/**
 * @brief Override HAL CAN TxPtbComplete callback.
 */
void HAL_CAN_TxPrimaryCompleteCallback(CAN_HandleTypeDef *hcan)
{
    struct bf0_can *drv_can = rt_container_of(hcan, struct bf0_can, CanHandle);

    rt_hw_can_isr(&drv_can->device, RT_CAN_EVENT_TX_DONE | (0 << 8));
}

/**
 * @brief Override HAL CAN TxStbComplete callback.
 */
void HAL_CAN_TxSecondaryCompleteCallback(CAN_HandleTypeDef *hcan)
{
    struct bf0_can *drv_can = rt_container_of(hcan, struct bf0_can, CanHandle);

    rt_hw_can_isr(&drv_can->device, RT_CAN_EVENT_TX_DONE | (0 << 8));
}

/**
 * @brief Override HAL CAN Error callback.
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    struct bf0_can *drv_can = rt_container_of(hcan, struct bf0_can, CanHandle);
    rt_uint32_t ir;

    ir = hcan->Instance->IR;

    if (ir & CAN_IR_BEIF)
    {
        HAL_CAN_AbortTxRequest(hcan, true);
        HAL_CAN_AbortTxRequest(hcan, false);
    }

    /* Re-enable error interrupts */
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_ERROR | CAN_IT_BUSOFF |
                        CAN_IT_ERROR_PASSIVE | CAN_IT_ARBITRATION_LOST_ERROR);
}

/* ─── IRQ handler ───────────────────────────────────────────────────────── */

/**
 * @brief CAN interrupt handler (delegates to HAL_CAN_IRQHandler).
 *
 * This is the single NVIC handler for a CAN instance.
 * HAL_CAN_IRQHandler reads the IR register and dispatches to
 * the appropriate weak callbacks (RxMsgPendingCallback,
 * TxPtbCompleteCallback, etc.)
 */
static void _can_irq_handler(struct bf0_can *drv_can)
{
    RT_ASSERT(drv_can);
    HAL_CAN_IRQHandler(&drv_can->CanHandle);
}

/* ─── Per-instance NVIC handlers ────────────────────────────────────────── */

#ifdef BSP_USING_CAN1
void CAN1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_irq_handler(&bf0_can_obj[CAN1_INDEX]);
    rt_interrupt_leave();
}
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2
void CAN2_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_irq_handler(&bf0_can_obj[CAN2_INDEX]);
    rt_interrupt_leave();
}
#endif /* BSP_USING_CAN2 */

/**
 * @brief Initialize and register all CAN devices.
 */
int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
    CAN_FilterTypeDef filterConf = {0};
    rt_err_t result;

    config.privmode   = RT_CAN_MODE_NOPRIV;
    config.ticks      = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr     = 16;
#endif
    config.msgboxsz      = CAN_RX_FIFO_DEPTH;
    config.sndboxnumber  = CAN_SNDBOX_NUM;

    /* Default filter: accept all */
    filterConf.FilterId         = 0x00000000;
    filterConf.FilterMask       = 0x00000000;
    filterConf.FilterBank       = 0;
    filterConf.FilterActivation = CAN_FILTER_ENABLE;
    filterConf.IDECheckEnable   = DISABLE;
    filterConf.IDEValue         = CAN_ID_STD;

    for (int i = 0; i < CAN_MAX; i++)
    {
        bf0_can_obj[i].CanHandle.Instance  = bf0_can_cfg[i].Instance;
        bf0_can_obj[i].CanHandle.State     = HAL_CAN_STATE_RESET;
        bf0_can_obj[i].CanHandle.ErrorCode = HAL_CAN_ERROR_NONE;
        bf0_can_obj[i].FilterConfig        = filterConf;
        bf0_can_obj[i].config              = &bf0_can_cfg[i];
        bf0_can_obj[i].name                = (char *)bf0_can_cfg[i].device_name;
        bf0_can_obj[i].device.config       = config;

        result = rt_hw_can_register(&bf0_can_obj[i].device,
                                    bf0_can_cfg[i].device_name,
                                    &_can_ops,
                                    &bf0_can_obj[i]);
        if (result != RT_EOK)
        {
            LOG_E("%s: register failed, err=%d", bf0_can_cfg[i].device_name, result);
        }
        else
        {
            LOG_I("%s: registered OK", bf0_can_cfg[i].device_name);
        }
    }

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_can_init);

#endif /* BSP_USING_CAN */

