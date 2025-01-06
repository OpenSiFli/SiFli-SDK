/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2017-10-30     ZYH            the first version
 * 2019-12-19     tyustli           port to stm32 series
 */
#include "drv_usbh.h"
#include "board.h"
#include "bf0_hal.h"

static HCD_HandleTypeDef sifli_hhcd_fs;
static struct rt_completion urb_completion;
static volatile rt_bool_t connect_status = RT_FALSE;

#if (RT_DEBUG_USB==1)
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    va_end(args);
}
#endif

void OTG_FS_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_HCD_IRQHandler(&sifli_hhcd_fs);
    rt_interrupt_leave();
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
    uhcd_t hcd = (uhcd_t)hhcd->pData;
    if (!connect_status)
    {
        connect_status = RT_TRUE;
        RT_DEBUG_LOG(RT_DEBUG_USB, ("usb connected\n"));
        rt_usbh_root_hub_connect_handler(hcd, OTG_FS_PORT, RT_FALSE);
    }
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
    uhcd_t hcd = (uhcd_t)hhcd->pData;
    if (connect_status)
    {
        connect_status = RT_FALSE;
        RT_DEBUG_LOG(RT_DEBUG_USB, ("usb disconnnect\n"));
        rt_usbh_root_hub_disconnect_handler(hcd, OTG_FS_PORT);
        HAL_RCC_ResetModule(RCC_MOD_USBC);
        HAL_HCD_Start(hhcd);
    }
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum, HCD_URBStateTypeDef urb_state)
{
    rt_completion_done(&urb_completion);
}

static rt_err_t drv_reset_port(rt_uint8_t port)
{
    RT_DEBUG_LOG(RT_DEBUG_USB, ("reset port\n"));
    HAL_HCD_ResetPort(&sifli_hhcd_fs);
    return RT_EOK;
}

static int drv_pipe_xfer(upipe_t pipe, rt_uint8_t token, void *buffer, int nbytes, int timeouts)
{
    int timeout = timeouts;

    while (1)
    {
        if (!connect_status)
        {
            return -1;
        }
        rt_completion_init(&urb_completion);
        if (token == USBH_PID_SETUP)
            token = USB_PID_SETUP;

        if (pipe->inst->address != sifli_hhcd_fs.hc[pipe->pipe_index].dev_addr)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("Address changed:%d->%d\n", sifli_hhcd_fs.hc[pipe->pipe_index].dev_addr, pipe->inst->address));
            sifli_hhcd_fs.hc[pipe->pipe_index].dev_addr = pipe->inst->address;
        }

        HAL_HCD_HC_SubmitRequest(&sifli_hhcd_fs,
                                 pipe->pipe_index,
                                 (pipe->ep.bEndpointAddress & 0x80) >> 7,
                                 pipe->ep.bmAttributes,
                                 token,
                                 buffer,
                                 nbytes,
                                 0);
        rt_completion_wait(&urb_completion, timeout);
        rt_thread_mdelay(10);
        if (HAL_HCD_HC_GetState(&sifli_hhcd_fs, pipe->pipe_index) == HC_NAK)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("nak\n"));
            if (pipe->ep.bmAttributes == USB_EP_ATTR_INT)
            {
                rt_thread_delay((pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) > 0 ? (pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) : 1);
            }
            HAL_HCD_HC_Halt(&sifli_hhcd_fs, pipe->pipe_index);
            HAL_HCD_HC_Init(&sifli_hhcd_fs,
                            pipe->pipe_index,
                            pipe->ep.bEndpointAddress,
                            pipe->inst->address,
                            USB_OTG_SPEED_FULL,
                            pipe->ep.bmAttributes,
                            pipe->ep.wMaxPacketSize);
            continue;
        }
        else if (HAL_HCD_HC_GetState(&sifli_hhcd_fs, pipe->pipe_index) == HC_STALL)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("stall\n"));
            pipe->status = UPIPE_STATUS_STALL;
            if (pipe->callback != RT_NULL)
            {
                pipe->callback(pipe);
            }
            return -1;
        }
        else if (HAL_HCD_HC_GetState(&sifli_hhcd_fs, pipe->pipe_index) == URB_ERROR)
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("error\n"));
            pipe->status = UPIPE_STATUS_ERROR;
            if (pipe->callback != RT_NULL)
            {
                pipe->callback(pipe);
            }
            return -1;
        }
        else if (URB_DONE == HAL_HCD_HC_GetURBState(&sifli_hhcd_fs, pipe->pipe_index))
        {
            RT_DEBUG_LOG(RT_DEBUG_USB, ("ok\n"));
            pipe->status = UPIPE_STATUS_OK;
            if (pipe->callback != RT_NULL)
            {
                pipe->callback(pipe);
            }
            size_t size = HAL_HCD_HC_GetXferCount(&sifli_hhcd_fs, pipe->pipe_index);
            if (pipe->ep.bEndpointAddress & 0x80)
            {
                return size;
            }
            else if (pipe->ep.bEndpointAddress & 0x00)
            {
                return size;
            }
            return nbytes;
        }

        continue;
    }
}

static rt_uint16_t pipe_index = 0;
#if defined(SF32LB52X) || defined(SF32LB56X)
static rt_uint8_t  drv_get_free_pipe_index(uint8_t ep_addr)
{
    rt_uint8_t idx = 0;
    RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d pipe_index=%d, ep_addr=0x%x\n", __func__, __LINE__, pipe_index, ep_addr));
    if (ep_addr & 0x80)//rx
    {
        for (idx = 1; idx < 4; idx++)
        {
            if (!(pipe_index & (0x01 << idx)))
            {
                pipe_index |= (0x01 << idx);
                RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %dpipe_index=%d,idx=%d\n", __func__, __LINE__, pipe_index, idx));
                return idx;
            }
        }
    }
    else//tx
    {
        for (idx = 5; idx < 7; idx++)
        {
            if (!(pipe_index & (0x01 << idx)))
            {
                pipe_index |= (0x01 << idx);
                RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d pipe_index=%d,idx=%d \n", __func__, __LINE__, pipe_index, idx));
                return idx;
            }
        }
    }
    return 0xff;
}
#else
static rt_uint8_t  drv_get_free_pipe_index(uint8_t ep_addr)
{
    rt_uint8_t idx = 0;
    for (idx = 1; idx < 16; idx++)
    {
        if (!(pipe_index & (0x01 << idx)))
        {
            pipe_index |= (0x01 << idx);
            return idx;
        }
    }
    return 0xff;
}
#endif


static void drv_free_pipe_index(rt_uint8_t index)
{
    pipe_index &= ~(0x01 << index);
}

static rt_err_t drv_open_pipe(upipe_t pipe)
{
    pipe->pipe_index = drv_get_free_pipe_index(pipe->ep.bEndpointAddress);
    RT_DEBUG_LOG(RT_DEBUG_USB, ("open_pipe=%d, ep=%d, dev_addr\n", pipe->pipe_index, pipe->ep.bEndpointAddress, pipe->inst->address));
    HAL_HCD_HC_Init(&sifli_hhcd_fs,
                    pipe->pipe_index,
                    pipe->ep.bEndpointAddress,
                    pipe->inst->address,
                    USB_OTG_SPEED_FULL,
                    pipe->ep.bmAttributes,
                    pipe->ep.wMaxPacketSize);
    /* Set DATA0 PID token*/
    if (sifli_hhcd_fs.hc[pipe->pipe_index].ep_is_in)
    {
        sifli_hhcd_fs.hc[pipe->pipe_index].toggle_in = 0;
    }
    else
    {
        sifli_hhcd_fs.hc[pipe->pipe_index].toggle_out = 0;
    }
    return RT_EOK;
}

static rt_err_t drv_close_pipe(upipe_t pipe)
{
    HAL_HCD_HC_Halt(&sifli_hhcd_fs, pipe->pipe_index);
    drv_free_pipe_index(pipe->pipe_index);
    return RT_EOK;
}

static struct uhcd_ops _uhcd_ops =
{
    drv_reset_port,
    drv_pipe_xfer,
    drv_open_pipe,
    drv_close_pipe,
};

static rt_err_t sifli_hcd_init(rt_device_t device)
{
    HAL_StatusTypeDef state;

    HCD_HandleTypeDef *hhcd = (HCD_HandleTypeDef *)device->user_data;

    HAL_RCC_EnableModule(RCC_MOD_USBC);
    hhcd->Instance = hwp_usbc;
    hhcd->Init.Host_channels = 8;
    hhcd->Init.speed = HCD_SPEED_FULL;
    hhcd->Init.dma_enable = DISABLE;
    hhcd->Init.phy_itface = HCD_PHY_EMBEDDED;
    hhcd->Init.Sof_enable = DISABLE;
    state = HAL_HCD_Init(hhcd);
    if (state != HAL_OK)
    {
        return -RT_ERROR;
    }
    rt_kprintf("USB HCD started\n");
    HAL_HCD_Start(hhcd);
    hwp_usbc->dbgl = 0x80;
#ifdef USBH_USING_CONTROLLABLE_POWER
    rt_pin_mode(USBH_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(USBH_POWER_PIN, PIN_LOW);
#endif
    return RT_EOK;
}

int sifli_usbh_register(void)
{
    rt_err_t res = -RT_ERROR;

    uhcd_t uhcd = (uhcd_t)rt_malloc(sizeof(struct uhcd));
    if (uhcd == RT_NULL)
    {
        rt_kprintf("uhcd malloc failed\r\n");
        return -RT_ERROR;
    }

    rt_memset((void *)uhcd, 0, sizeof(struct uhcd));

    uhcd->parent.type = RT_Device_Class_USBHost;
    uhcd->parent.init = sifli_hcd_init;
    uhcd->parent.user_data = &sifli_hhcd_fs;

    uhcd->ops = &_uhcd_ops;
    uhcd->num_ports = OTG_FS_PORT;
    sifli_hhcd_fs.pData = uhcd;

    res = rt_device_register(&uhcd->parent, "usbh", RT_DEVICE_FLAG_DEACTIVATE);
    if (res != RT_EOK)
    {
        rt_kprintf("register usb host failed res = %d\r\n", res);
        return -RT_ERROR;
    }

    rt_usb_host_init("usbh");

    return RT_EOK;
}
INIT_DEVICE_EXPORT(sifli_usbh_register);
