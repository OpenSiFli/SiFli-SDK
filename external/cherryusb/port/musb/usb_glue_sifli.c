/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbh_core.h"
#include "usb_musb_reg.h"

#undef USB_POWER_SOFTCONN
#undef USB_DEVCTL_FSDEV
#undef USB_DEVCTL_LSDEV
#undef USB_DEVCTL_SESSION
#undef USB_POWER_HSENAB
#undef USB_POWER_HSMODE
#undef USB_POWER_RESET
#undef USB_POWER_RESUME

#ifndef CONFIG_USB_MUSB_SIFLI
#error must define CONFIG_USB_MUSB_SIFLI when use sifli chips
#endif

#include "bf0_hal.h"

uint8_t usbd_get_musb_fifo_cfg(struct musb_fifo_cfg **cfg)
{
    *cfg = NULL; // No FIFO configuration for this implementation, readonly
    return 0;
}

uint8_t usbh_get_musb_fifo_cfg(struct musb_fifo_cfg **cfg)
{
    *cfg = NULL; // No FIFO configuration for this implementation, readonly
    return 0;
}

uint32_t usb_get_musb_ram_size(void)
{
    return 0xFFFF; // No specific RAM size for this implementation
}

void usbd_musb_delay_ms(uint8_t ms)
{
    /* implement later */
}

#ifdef PKG_CHERRYUSB_DEVICE
void usb_dc_low_level_init(uint8_t busid)
{
    HAL_RCC_EnableModule(RCC_MOD_USBC);

#ifdef SOC_SF32LB57X
    /* switch to 48MHz */
    hwp_usbc->mode_48m |= 2;
    hwp_hpsys_cfg->ANAU_CR |= HPSYS_CFG_ANAU_CR_EN_BG;
#endif /* SOC_SF32LB57X */

#ifdef SOC_SF32LB58X
    //hwp_usbc->utmicfg12 = hwp_usbc->utmicfg12 | 0x3; //set xo_clk_sel
    hwp_usbc->utmicfg23 = 0xd8;
    hwp_usbc->ldo25 = hwp_usbc->ldo25 | 0xa; //set psw_en and ldo25_en
    HAL_Delay(1);
    hwp_usbc->swcntl3 = 0x1;                    //set utmi_en for USB2.0
    hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x40; //enable usb PLL.
#elif defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X) || defined(SOC_SF32LB57X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN;
#elif defined(SOC_SF32LB55X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN;
#endif
#ifndef SOC_SF32LB55X
    hwp_usbc->usbcfg |= (USB_USBCFG_AVALID | USB_USBCFG_AVALID_DR);
    hwp_usbc->dpbrxdisl = 0xFE;
    hwp_usbc->dpbtxdisl = 0xFE;
#endif
    NVIC_EnableIRQ(USBC_IRQn);
    __HAL_SYSCFG_Enable_USB();
}

void usb_dc_low_level_deinit(uint8_t busid)
{
    NVIC_DisableIRQ(USBC_IRQn);
#ifdef SOC_SF32LB58X
    hwp_usbc->usbcfg &= ~0x40; // Disable usb PLL.
    hwp_usbc->swcntl3 = 0x0;
    hwp_usbc->ldo25 &= ~0xa; // Disable psw_en and ldo25_en
#elif defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X) || defined(SOC_SF32LB57X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN);
#elif defined(SOC_SF32LB55X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN);
#endif
    /* reset USB to make DP change to PULLDOWN state */
    hwp_hpsys_rcc->RSTR2 |= HPSYS_RCC_RSTR2_USBC;
    HAL_Delay_us(100);
    hwp_hpsys_rcc->RSTR2 &= ~HPSYS_RCC_RSTR2_USBC;
    HAL_RCC_DisableModule(RCC_MOD_USBC);
}
#endif

#ifdef PKG_CHERRYUSB_HOST
void usb_hc_low_level_init(struct usbh_bus *bus)
{
    HAL_RCC_EnableModule(RCC_MOD_USBC);

#ifdef SOC_SF32LB58X
    //hwp_usbc->utmicfg12 = hwp_usbc->utmicfg12 | 0x3; //set xo_clk_sel
    hwp_usbc->utmicfg23 = 0xd8;
    hwp_usbc->ldo25 = hwp_usbc->ldo25 | 0xa; //set psw_en and ldo25_en
    HAL_Delay(1);
    hwp_usbc->swcntl3 = 0x1;                    //set utmi_en for USB2.0
    hwp_usbc->usbcfg = hwp_usbc->usbcfg | 0x40; //enable usb PLL.
    hwp_usbc->dpbrxdisl = 0xff;
    hwp_usbc->dpbtxdisl = 0xff;
    hwp_usbc->utmicfg25 = hwp_usbc->utmicfg25 | 0xc0;
#elif defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X) || defined(SOC_SF32LB57X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN;
#elif defined(SOC_SF32LB55X)
    hwp_hpsys_cfg->USBCR |= HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN;
#endif
#ifndef SOC_SF32LB55X
    hwp_usbc->usbcfg |= (USB_USBCFG_AVALID | USB_USBCFG_AVALID_DR);
#ifndef SOC_SF32LB58X
    hwp_usbc->dpbrxdisl = 0xFE;
    hwp_usbc->dpbtxdisl = 0xFE;
#endif
#endif
    __HAL_SYSCFG_Enable_USB();
    __HAL_SYSCFG_USB_DM_PD();
    hwp_usbc->usbcfg &= 0xEF;
    hwp_usbc->dbgl = 0x80;
#ifdef SOC_SF32LB58X
    hwp_usbc->testmode = 0;
    hwp_usbc->power = USB_POWER_HSENAB | USB_POWER_SOFTCONN;
#else
    hwp_usbc->power |= USB_POWER_SOFTCONN;
#endif

    NVIC_EnableIRQ(USBC_IRQn);
}

void usb_hc_low_level_deinit(struct usbh_bus *bus)
{
    NVIC_DisableIRQ(USBC_IRQn);
#ifdef SOC_SF32LB58X
    hwp_usbc->usbcfg &= ~0x40; // Disable usb PLL.
    hwp_usbc->swcntl3 = 0x0;
    hwp_usbc->ldo25 &= ~0xa; // Disable psw_en and ldo25_en
#elif defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X) || defined(SOC_SF32LB57X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_DP_EN | HPSYS_CFG_USBCR_USB_EN);
#elif defined(SOC_SF32LB55X)
    hwp_hpsys_cfg->USBCR &= ~(HPSYS_CFG_USBCR_DM_PD | HPSYS_CFG_USBCR_USB_EN);
#endif
    /* reset USB to make DP change to PULLDOWN state */
    hwp_hpsys_rcc->RSTR2 |= HPSYS_RCC_RSTR2_USBC;
    HAL_Delay_us(100);
    hwp_hpsys_rcc->RSTR2 &= ~HPSYS_RCC_RSTR2_USBC;
    HAL_RCC_DisableModule(RCC_MOD_USBC);
}

void musb_reset_prev(void)
{
#if defined(SF32LB58X)
    hwp_usbc->utmicfg25 |= 0xc0;
    hwp_usbc->utmicfg21 = 0x23;
    hwp_usbc->swcntl2 = 0x7c;
    hwp_usbc->utmicfg0 = 0x30;
#endif
}

void musb_reset_asserted(void)
{
#if defined(SF32LB58X)
    hwp_usbc->rsvd0 = 0xc; //58
#endif
}

void musb_reset_post(void)
{
#if defined(SF32LB58X)
    hwp_usbc->rsvd0 = 0x0; //58
    hwp_usbc->utmicfg25 &= ~0xc0;
    hwp_usbc->utmicfg21 = 0x2f;
    hwp_usbc->swcntl2 = 0x40;
    hwp_usbc->utmicfg0 = 0x00;
#endif
}
#endif

void USBC_IRQHandler(void)
{
#ifdef BSP_USING_RTTHREAD    
    rt_interrupt_enter();
#endif /* BSP_USING_RTTHREAD */    
#ifdef PKG_CHERRYUSB_DEVICE
    USBD_IRQHandler(0);
#endif
#ifdef PKG_CHERRYUSB_HOST
    USBH_IRQHandler(0);
#endif
#ifdef BSP_USING_RTTHREAD    
    rt_interrupt_leave();
#endif /* BSP_USING_RTTHREAD */    
}
