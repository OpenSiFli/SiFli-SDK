/**
  ******************************************************************************
  * @file   drv_usbd.c
  * @author Sifli software development team
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

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "bf0_hal.h"
#include <string.h>
#include <drv_config.h>

//#define DRV_DEBUG
#define LOG_TAG             "drv.usbd"
#include <drv_log.h>

#ifdef SOC_SF32LB58X
    #define USBD_MPS    512
#else
    #define USBD_MPS    64
#endif

static PCD_HandleTypeDef _sifli_pcd;
static struct udcd _sifli_udc;
static struct ep_id _ep_pool[] =
{
    {0x0,  USB_EP_ATTR_CONTROL,     USB_DIR_INOUT,  64,       ID_ASSIGNED  },
    {0x1,  USB_EP_ATTR_BULK,        USB_DIR_IN,     USBD_MPS, ID_UNASSIGNED},
    {0x2,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    USBD_MPS, ID_UNASSIGNED},
    {0x3,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    USBD_MPS, ID_UNASSIGNED},
    {0x4,  USB_EP_ATTR_BULK,        USB_DIR_OUT,    USBD_MPS, ID_UNASSIGNED},
    {0x5,  USB_EP_ATTR_INT,         USB_DIR_IN,     USBD_MPS, ID_UNASSIGNED},
    {0x6,  USB_EP_ATTR_BULK,        USB_DIR_IN,     USBD_MPS, ID_UNASSIGNED},
    {0x7,  USB_EP_ATTR_BULK,        USB_DIR_IN,     USBD_MPS, ID_UNASSIGNED},
};

//void USBD_FS_IRQ_HANDLER(void)
void USBD_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_PCD_IRQHandler(&_sifli_pcd);
    /* leave interrupt */
    rt_interrupt_leave();

}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    /* open ep0 OUT and IN */
    HAL_PCD_EP_Open(pcd, 0x00, 0x40, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(pcd, 0x80, 0x40, EP_TYPE_CTRL);
    rt_usbd_reset_handler(&_sifli_udc);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    rt_usbd_ep0_setup_handler(&_sifli_udc, (struct urequest *)hpcd->Setup);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (epnum == 0)
    {
        rt_usbd_ep0_in_handler(&_sifli_udc);
    }
    else
    {
        rt_usbd_ep_in_handler(&_sifli_udc, 0x80 | epnum, hpcd->IN_ep[epnum].xfer_count);
    }
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    rt_usbd_connect_handler(&_sifli_udc);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    rt_usbd_sof_handler(&_sifli_udc);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    rt_usbd_disconnect_handler(&_sifli_udc);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (epnum != 0)
    {
        rt_usbd_ep_out_handler(&_sifli_udc, epnum, 0);   // hpcd->OUT_ep[epnum].xfer_count
    }
    else
    {
        rt_usbd_ep0_out_handler(&_sifli_udc, hpcd->OUT_ep[0].xfer_count);
    }
}

void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state)
{
    if (state == 1)
    {
        // Enable pull up
    }
    else
    {
        // Disable pull up
    }
}
void HAL_PCD_Set_RxscrACK(uint8_t epnum)
{
    __IO struct musb_epN_regs *epn = &(_sifli_pcd.Instance->ep[epnum].epN);
    epn->rxcsr = 0x00;
}
void HAL_PCD_Set_RxscrNCK(uint8_t epnum)
{
    __IO struct musb_epN_regs *epn = &(_sifli_pcd.Instance->ep[epnum].epN);
    epn->rxcsr = 0x20;
}
static uint8_t rx_buff_flag[16];

uint8_t HAL_PCD_Get_RxbuffControl(uint8_t ep_num)
{
    for (int i = 0; i < 16; i++)
    {
        if (ep_num == i) return rx_buff_flag[i];
    }
    return 0;
}

void HAL_PCD_Set_RxbuffControl(uint8_t ep_num, uint8_t flag)
{
    for (int i = 0; i < 16; i++)
    {
        if (ep_num == i)
        {
            rx_buff_flag[i] = flag;
            return;
        }
    }
}

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
    HAL_PCD_EP_SetStall(&_sifli_pcd, address);
    return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
    HAL_PCD_EP_ClrStall(&_sifli_pcd, address);
    return RT_EOK;
}

static rt_err_t _set_address(rt_uint8_t address)
{
    LOG_I("Set Address: %d(0x%x)\n", address, address);
    HAL_PCD_SetAddress(&_sifli_pcd, address);
    return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
    return RT_EOK;
}

static rt_err_t _ep_enable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    LOG_I("_ep_enable : %x: %d: %x\n", ep->ep_desc->bEndpointAddress, ep->ep_desc->wMaxPacketSize, ep->ep_desc->bmAttributes);
    HAL_PCD_EP_Open(&_sifli_pcd, ep->ep_desc->bEndpointAddress,
                    ep->ep_desc->wMaxPacketSize, ep->ep_desc->bmAttributes);
    for (int i = 0; i < 16; i++) rx_buff_flag[i] = 1;
    return RT_EOK;
}

static rt_err_t _ep_disable(uep_t ep)
{
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    HAL_PCD_EP_Close(&_sifli_pcd, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
    //LOG_D("_ep_read %d\n", address);
    size = HAL_PCD_EP_Receive(&_sifli_pcd, address, buffer);
    return size;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    //LOG_D("_ep_read_prepare %d, %d\n", address, size);
    //HAL_PCD_EP_Receive(&_sifli_pcd, address, buffer, size);
    HAL_PCD_EP_Prepare_Receive(&_sifli_pcd, address, buffer, size);
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    HAL_PCD_EP_Transmit(&_sifli_pcd, address, buffer, size);
    return size;
}

static rt_err_t _ep0_send_status(void)
{
    HAL_PCD_EP_Transmit(&_sifli_pcd, 0x00, NULL, 0);
    return RT_EOK;
}

static rt_err_t _suspend(void)
{
    return RT_EOK;
}

static rt_err_t _wakeup(void)
{
    return RT_EOK;
}

static rt_err_t _test_mode(rt_uint16_t tm, uint8_t *data, uint8_t len)
{
    HAL_PCD_TestMode(&_sifli_pcd, tm, data, len);
    return RT_EOK;
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    HAL_RCC_EnableModule(RCC_MOD_USBC);

#ifdef SOC_SF32LB58X
    //hwp_usbc->utmicfg12 = hwp_usbc->utmicfg12 | 0x3; //set xo_clk_sel
    hwp_usbc->ldo25 = hwp_usbc->ldo25 | 0xa; //set psw_en and ldo25_en
    HAL_Delay(1);
    hwp_usbc->swcntl3 = 0x1; //set utmi_en for USB2.0
    hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x40; //enable usb PLL.
#elif defined(SOC_SF32LB56X)||defined(SOC_SF32LB52X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN;
#elif defined(SOC_SF32LB55X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN;
#endif
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
#ifdef SOC_SF32LB58X
    hwp_usbc->usbcfg &= ~0x40;  // Disable usb PLL.
    hwp_usbc->swcntl3 = 0x0;
    hwp_usbc->ldo25 &= ~0xa;    // Disable psw_en and ldo25_en
#elif defined(SOC_SF32LB56X)||defined(SOC_SF32LB52X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN);
#elif defined(SOC_SF32LB55X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN);
#endif
    HAL_RCC_DisableModule(RCC_MOD_USBC);
}

static rt_err_t _init(rt_device_t device)
{
    PCD_HandleTypeDef *pcd;

    //__asm ("B .");
    /* Set LL Driver parameters */
    pcd = (PCD_HandleTypeDef *)device->user_data;
    pcd->Instance = hwp_usbc;
    memset(&pcd->Init, 0, sizeof pcd->Init);
    pcd->Init.dev_endpoints = 8;
    pcd->Init.speed = PCD_SPEED_FULL;
    pcd->Init.ep0_mps = 16;
    pcd->Init.phy_itface = PCD_PHY_EMBEDDED;
    /* Initialize LL Driver */

    HAL_PCD_Init(pcd);
    HAL_PCD_Start(pcd);
#ifndef SOC_SF32LB55X
    USB_ENABLE_PHY(pcd);
    USB_DISABLE_DOUBLE_BUFFER(pcd);
#endif
    return RT_EOK;
}

const static struct udcd_ops _udc_ops =
{
    _set_address,
    _set_config,
    _ep_set_stall,
    _ep_clear_stall,
    _ep_enable,
    _ep_disable,
    _ep_read_prepare,
    _ep_read,
    _ep_write,
    _ep0_send_status,
    _suspend,
    _wakeup,
    _test_mode,
};

int sifli_usbd_register(void)
{
    rt_memset((void *)&_sifli_udc, 0, sizeof(struct udcd));
    _sifli_udc.parent.type = RT_Device_Class_USBDevice;
    _sifli_udc.parent.init = _init;
    _sifli_udc.parent.user_data = &_sifli_pcd;
    _sifli_udc.ops = &_udc_ops;
    /* Register endpoint infomation */
    _sifli_udc.ep_pool = _ep_pool;
    _sifli_udc.ep0.id = &_ep_pool[0];
#ifdef SOC_SF32LB58X
    _sifli_udc.device_is_hs = 1;
#endif
    rt_device_register((rt_device_t)&_sifli_udc, "usbd", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE);
    rt_usb_device_init();
    return RT_EOK;
}
INIT_DEVICE_EXPORT(sifli_usbd_register);
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

//#define USBD_FUNC_TEST
#ifdef USBD_FUNC_TEST

#include "msh.h"
#define USB_SERIAL_PORT "vcom"

static rt_err_t vcom_rx_ind(rt_device_t dev, rt_size_t size)
{
    static uint8_t buf[128];
    int len;

    len = rt_device_read(dev, 0, buf, size);
    rt_kprintf("Recv:%d\n", len);
    HAL_DBG_print_data((char *)buf, 0, len);
    return RT_EOK;
}

int cmd_usbdtest(int argc, char *argv[])
{
    if (strcmp(argv[1], "console") == 0)
    {
        msh_set_console(USB_SERIAL_PORT);
    }
    else if (strcmp(argv[1], "com") == 0)
    {
        static rt_device_t com_dev;

        if (strcmp(argv[2], "open") == 0)
        {
            /* find new console device */
            com_dev = rt_device_find(USB_SERIAL_PORT);
            if (com_dev != RT_NULL)
            {
                rt_uint16_t oflag = RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX;
                if (rt_device_open(com_dev, oflag) == RT_EOK)
                    rt_device_set_rx_indicate(com_dev, vcom_rx_ind);
                else
                    rt_kprintf("Open vcom failed\n");
            }
        }
        else if (strcmp(argv[2], "write") == 0)
        {
            rt_device_write(com_dev, 0, "Test", 4);
        }

    }
    return 0;
}

MSH_CMD_EXPORT_ALIAS(cmd_usbdtest, usbd, Test USB device);
#endif  /* USBD_FUNC_TEST */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
