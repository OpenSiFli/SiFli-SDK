/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
    //rt_kprintf("_ep_read_prepare %d, %d\n", address, size);

    HAL_PCD_EP_Prepare_Receive(&_sifli_pcd, address, buffer, size);
    return size;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    //rt_kprintf("_ep_write %d, %d\n", address, size);
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

static rt_err_t _init(rt_device_t device)
{
    PCD_HandleTypeDef *pcd;

    //__asm ("B .");
    /* Set LL Driver parameters */
    pcd = (PCD_HandleTypeDef *)device->user_data;
    pcd->Instance = hwp_usbc;
    memset(&pcd->Init, 0, sizeof pcd->Init);
    pcd->Init.dev_endpoints = 8;
#ifdef SOC_SF32LB58X
    pcd->Init.speed = PCD_SPEED_HIGH;
#else
    pcd->Init.speed = PCD_SPEED_FULL;
#endif
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

__weak void BSP_USB_Power_Down(void)
{
}
__weak void BSP_USB_Power_Up(void)
{

}
static void enabled_usb_protocol(void)
{
    PCD_HandleTypeDef *usb_pcd = &_sifli_pcd;
    BSP_USB_Power_Up();
    HAL_PCD_MspInit(NULL);

    HAL_PCD_Init(usb_pcd);
    HAL_PCD_Start(usb_pcd);
#ifndef SOC_SF32LB55X
    USB_ENABLE_PHY(usb_pcd);
    USB_DISABLE_DOUBLE_BUFFER(usb_pcd);
#endif
}
static void disabled_usb_protocol(void)
{
    HAL_PCD_DisconnectCallback(NULL);
    HAL_PCD_MspDeInit(NULL);
    BSP_USB_Power_Down();
}

#if defined(RT_USING_PM)
static struct rt_device rt_usb_device;
static PCD_HandleTypeDef *usb_pcd;
static rt_err_t rt_usb_control(struct rt_device *dev, int cmd, void *args)
{
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RESUME:
    {

    }
    break;
    case RT_DEVICE_CTRL_SUSPEND:
    {

    }
    break;
    case RT_DEVICE_OFLAG_OPEN:
    {
        enabled_usb_protocol();
    }
    break;
    case RT_DEVICE_OFLAG_CLOSE:
    {
        disabled_usb_protocol();
    }
    break;
    default:
        break;
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
static const rt_device_ops usb_device_ops =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    rt_usb_control,
};
#endif
static int rt_usb_register_rt_device(void)
{
    rt_err_t err = RT_EOK;
    rt_device_t device;

    device = &rt_usb_device;

    device->type        = RT_Device_Class_Miscellaneous;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &usb_device_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_usb_control;
#endif
    device->user_data = &_sifli_pcd;

    err = rt_device_register(device, "usb_reg", RT_DEVICE_FLAG_RDWR);
    RT_ASSERT(RT_EOK == err);
    return 0;
}
INIT_APP_EXPORT(rt_usb_register_rt_device);
#endif

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

#if defined(SF32LB56X) || defined(SF32LB52X)
#define TEST_FAIL       0x0
#define TEST_PASS       0x1
#define TEST_UNFINISHED 0x2

static void wait(uint32_t cycle)
{

    for (uint32_t i = 0; i < cycle; i++)
    {
        __NOP();
    }

}

static uint8_t usbc_test_packet(void)
{
    uint32_t error_flag = 0;
    uint32_t usb_rx_len;
    uint32_t ep00_rdata;
    uint32_t ep1_rdata;
    uint32_t set_addr_flag;
    uint32_t ep0_fifo_addr = USBC_BASE + 0x20;
    uint32_t tx_src_addr = USBC_BASE + 0x8;
    uint32_t tx_src_addr2 = USBC_BASE + 0x10;
    uint32_t tx_dst_addr = 0x20018800;
    uint32_t rx_src_addr = 0x20018800;
    uint32_t rx_dst_addr = USBC_BASE + 0xc;
    int i, sel;

    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD;
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DP_EN;
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_USB_EN;
    hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_USBC;
    hwp_pinmux1->PAD_PA17 = hwp_pinmux1->PAD_PA17 & (~HPSYS_PINMUX_PAD_PA17_PE);
    hwp_pinmux1->PAD_PA18 = hwp_pinmux1->PAD_PA18 & (~HPSYS_PINMUX_PAD_PA18_PE);

    while (!(hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY_Msk));
    hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_DLL_EN;

    LOG_I("Enable DLL1\n");
    // enable DLL1 (120MHz)
    hwp_hpsys_cfg->CAU2_CR |= HPSYS_CFG_CAU2_CR_HPBG_EN;
    hwp_hpsys_cfg->CAU2_CR |= HPSYS_CFG_CAU2_CR_HPBG_VDDPSW_EN;
    hwp_hpsys_rcc->DLL1CR = (hwp_hpsys_rcc->DLL1CR & ~HPSYS_RCC_DLL1CR_STG_Msk) |
                            (0x9 << HPSYS_RCC_DLL1CR_STG_Pos);
    hwp_hpsys_rcc->DLL1CR = (hwp_hpsys_rcc->DLL1CR & ~HPSYS_RCC_DLL1CR_OUT_DIV2_EN_Msk) |
                            (0x1 << HPSYS_RCC_DLL1CR_OUT_DIV2_EN_Pos);
    hwp_hpsys_rcc->DLL1CR |= HPSYS_RCC_DLL1CR_EN;
    while (!(hwp_hpsys_rcc->DLL1CR & HPSYS_RCC_DLL1CR_READY_Msk));

    // select DLL1 as sys clock
    hwp_hpsys_rcc->CSR = (hwp_hpsys_rcc->CSR & ~HPSYS_RCC_CSR_SEL_SYS_Msk) |
                         (0x3 << HPSYS_RCC_CSR_SEL_SYS_Pos);

    hwp_hpsys_rcc->CFGR = (hwp_hpsys_rcc->CFGR & ~HPSYS_RCC_CFGR_HDIV_Msk) |
                          (0x1 << HPSYS_RCC_CFGR_HDIV_Pos);

    LOG_I("Enable DLL2\n");
    // enable DLL2 (240MHz)
    hwp_hpsys_rcc->DLL2CR = (hwp_hpsys_rcc->DLL2CR & ~HPSYS_RCC_DLL2CR_STG_Msk) |
                            (0x9 << HPSYS_RCC_DLL2CR_STG_Pos);
    hwp_hpsys_rcc->DLL2CR = (hwp_hpsys_rcc->DLL2CR & ~HPSYS_RCC_DLL2CR_OUT_DIV2_EN_Msk) |
                            (0x0 << HPSYS_RCC_DLL2CR_OUT_DIV2_EN_Pos);
    hwp_hpsys_rcc->DLL2CR |= HPSYS_RCC_DLL2CR_EN;
    while (!(hwp_hpsys_rcc->DLL2CR & HPSYS_RCC_DLL2CR_READY_Msk)) {};


    //sel = RAND(1, 0);
    //if(sel == 1){
    LOG_I("Configure USBC clock to SYSCLK/2\n");
    // select SYS clock as USBC source and DIV=2
    hwp_hpsys_rcc->CSR &= ~HPSYS_RCC_CSR_SEL_USBC;
    hwp_hpsys_rcc->USBCR = 0x2;
    wait(1000);
    //}
    //else {
    //  rt_krt_kprint("Configure USBC clock to DLL2/4\n");
    // select SYS clock as USBC source and DIV=4
    //  hwp_hpsys_rcc->USBCR = 0x4;
    //  hwp_hpsys_rcc->CSR |= HPSYS_RCC_CSR_SEL_USBC;
    //  wait(1000);
    //}

    NVIC_EnableIRQ(USBC_IRQn);

//---------------------------------------------------//
//-------------------set addr ep0--------------------//
//---------------------------------------------------//

    LOG_I("USBC LOG: *******get state*******!\n");
// rt_krt_kprint( "USBC INFO: devctl=0x%x ",hwp_usbc->devctl);
    //hwp_usbc->usbcfg = hwp_usbc->usbcfg & 0xef;
    //hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x20;
    //hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x40; //enable usb
    //hwp_usbc->power = 0x40;   //soft_con
    hwp_usbc->testmode = 0x20; //force_FS
    //hwp_usbc->intrusbe = hwp_usbc->intrusbe | 0x30;
    //hwp_usbc->rxmaxp = 0x40;
    //hwp_usbc->txmaxp = 0x40;
    //hwp_usbc->dpbtxdisl = 0x2;
    //hwp_usbc->dpbrxdisl = 0x2;
    hwp_usbc->devctl = hwp_usbc->devctl | 0x1; // set session
    hwp_usbc->testmode = 0x8; //force_FS
    while (1)
    {
        //-------usb rst int--------//
        hwp_usbc->fifox[0] = 0x00000000;
        hwp_usbc->fifox[0] = 0x00000000;
        hwp_usbc->fifox[0] = 0xAAAAAA00;
        hwp_usbc->fifox[0] = 0xAAAAAAAA;
        hwp_usbc->fifox[0] = 0xEEEEEEAA;
        hwp_usbc->fifox[0] = 0xEEEEEEEE;
        hwp_usbc->fifox[0] = 0xFFFFFEEE;
        hwp_usbc->fifox[0] = 0xFFFFFFFF;
        hwp_usbc->fifox[0] = 0xFFFFFFFF;
        hwp_usbc->fifox[0] = 0xDFBF7FFF;
        hwp_usbc->fifox[0] = 0xFDFBF7EF;
        hwp_usbc->fifox[0] = 0xDFBF7EFC;
        hwp_usbc->fifox[0] = 0xFDFBF7EF;
        (*(volatile unsigned char *)((0x50047020))) = (0x7e);

        //hwp_usbc->testmode = 0x28; //force_FS
        hwp_usbc->csr0_txcsr = 0x2;
        wait(20000);
        hwp_hpsys_rcc->RSTR2 |= HPSYS_RCC_RSTR2_USBC;
        wait(50);
        hwp_hpsys_rcc->RSTR2 &= ~HPSYS_RCC_RSTR2_USBC;
        wait(40);
        hwp_usbc->testmode = 0x28; //force_FS
        hwp_usbc->devctl = hwp_usbc->devctl | 0x1; // set session
    }
    //__WFI();

    //hwp_usbc->power = hwp_usbc->power | 0x8;
    //wait(2000);
    //hwp_usbc->power = hwp_usbc->power & 0xf7;

    //while(1);

    //if(usb_int_sta1!=0)
    //   rt_krt_kprint( "R\n");
    //if(usb_int_sta2!=0)
    //   return  TEST_FAIL;
    //if(usb_int_sta3!=0)
    //   return  TEST_FAIL;

    ////---------ep1 din----------//
    // hwp_usbc->index = 0x1;
    // hwp_usbc->rxmaxp = 0x40;
    // hwp_usbc->txmaxp = 0x40;
    // hwp_usbc->csr0_txcsr = 0x2000;  //tx*/

    // for(i=0;i<2;i++)
    //    hwp_usbc->fifox[1] = i+0x10000000;

    // hwp_usbc->csr0_txcsr = 0x2001;

    //__WFI();
    //if(usb_int_sta1!=0)
    //   error_flag=1;
    //if(usb_int_sta2==2)
    //   rt_krt_kprint( "1i\n");
    //else
    //   error_flag=1;
    //if(usb_int_sta3!=0)
    //   error_flag=1;

    ////---------ep1 dout----------//
    //hwp_usbc->csr0_txcsr = 0x0;  //rx
    //__WFI();
    //if(usb_int_sta1!=0)
    //   error_flag=1;
    //if(usb_int_sta2!=0)
    //   error_flag=1;
    //if(usb_int_sta3==2)
    //   rt_krt_kprint( "1o\n");
    //else
    //   error_flag=1;

    // usb_rx_len = hwp_usbc->rxcount;
    //
    // for(i=0;i<(usb_rx_len>>2);i++)
    // {
    //    ep00_rdata = hwp_usbc->fifox[1];
    //    if(ep00_rdata!=i+0x10000000)
    //      {error_flag=1;
    //        rt_krt_kprint( "E1\n");}
    // }

    //while(1);
// rt_krt_kprint( "USBC ERR: error ep1 expet=0x%x , get= 0x%x\n",i,ep1_rdata);
    if (error_flag == 1)
        return  TEST_FAIL;
    else
        return TEST_PASS; // or TEST_FAIL, add conditional checker in C
}

MSH_CMD_EXPORT(usbc_test_packet, eye_test);

#endif /* SF32LB56X || SF32LB52X */

