/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bus abstraction dispatcher header (Plan B).
 *
 * Provides compile-time bus selection between SDIO and USB via
 * the AIC_BUS_TYPE macro. Upper layers use the aicwf_bus_* /
 * aicwf_hal_* APIs; this header maps them to the correct backend
 * (sdio or usb) at compile time.
 *
 * Configuration (in aic_cfg.mk or via CFLAGS):
 *   AIC_BUS_TYPE=SDIO  (default) → enables CONFIG_SDIO_SUPPORT
 *   AIC_BUS_TYPE=USB              → enables CONFIG_USB_SUPPORT
 *
 * All chip models (D80/DC/DW/D81/D81X2/D89X2) are supported via
 * the existing CONFIG_AIC8800* chip-selection macros.
 */

#ifndef _AIC_BUS_H_
#define _AIC_BUS_H_

#include "aic_cfg.h"

/* ---- Compile-time bus selection ------------------------------------ */
#ifndef AIC_BUS_TYPE
    #define AIC_BUS_TYPE SDIO
#endif

/* Derive CONFIG_*_SUPPORT from AIC_BUS_TYPE unless caller pre-defined them */
#if defined(AIC_BUS_SDIO) || (AIC_BUS_TYPE == SDIO)
    #ifndef CONFIG_SDIO_SUPPORT
        #define CONFIG_SDIO_SUPPORT
    #endif
#elif defined(AIC_BUS_USB) || (AIC_BUS_TYPE == USB)
    #ifndef CONFIG_USB_SUPPORT
        #define CONFIG_USB_SUPPORT
    #endif
    #ifndef CONFIG_AICWF_BUS_USB
        #define CONFIG_AICWF_BUS_USB
    #endif
#endif

/* ---- Bus-specific backend includes --------------------------------- */
/* Note: only the public port/inc headers and the published include/ interface
 * headers (drv_api/inc etc., see the copy_release_headers.py list) may be included;
 * host-internal headers (e.g. sdio_co.h/usb_co.h under layer_common/inc) are
 * strictly forbidden, otherwise the platform layer cannot build in release mode
 * (no host sources).
 */
#ifdef CONFIG_SDIO_SUPPORT
    #include "sdio_port.h"
    #include "aic_sdio_types.h"
    #include "sdio_def.h"
#endif

#ifdef CONFIG_USB_SUPPORT
    #include "usb_port.h"
    #include "aic_usb_types.h"
#endif

/* ---- Dispatcher macros --------------------------------------------- *
 * Upper layers should call the aicwf_bus_<op>(...) forms; these macros
 * expand to the active backend symbol. Functions with identical
 * signatures are guaranteed by the parallel API design of
 * sdio_port.h / usb_port.h (and sdio_co.h / usb_co.h).
 */

#ifdef CONFIG_SDIO_SUPPORT
    #define aicwf_bus_claim_host        aicwf_sdio_claim_host
    #define aicwf_bus_release_host      aicwf_sdio_release_host
    #define aicwf_bus_readsb            aicwf_sdio_readsb
    #define aicwf_bus_writesb           aicwf_sdio_writesb
    #define aicwf_bus_writesb_msg       aicwf_sdio_writesb_msg
    #define aicwf_bus_readb_cmd52       aicwf_sdio_readb_cmd52
    #define aicwf_bus_writeb_cmd52      aicwf_sdio_writeb_cmd52
    #define aicwf_bus_readb_cmd52_f2    aicwf_sdio_readb_cmd52_func2
    #define aicwf_bus_writeb_cmd52_f2   aicwf_sdio_writeb_cmd52_func2
    #define aicwf_bus_func_init         aicwf_sdio_func_init
    #define aicwf_bus_func_init_v3      aicwf_sdiov3_func_init
    #define aicwf_bus_release_func2     aicwf_sdio_release_func2
    #define aicwf_bus_host_enable_isr   aicwf_sdio_host_enable_isr
    #define aicwf_bus_set_irq_handler   aicwf_sdio_set_irq_handler
    #define aicwf_bus_interrupt_init    aicwf_sdio_interrupt_init
    #define aicwf_bus_isr               aicwf_sdio_isr
    #define aicwf_bus_drv_register      aicwf_sdio_drv_register
    #define aicwf_bus_host_init         aicwf_sdio_host_init
    #define aicwf_bus_host_deinit       aicwf_sdio_host_deinit
    #define aicwf_bus_host_predeinit    aicwf_sdio_host_predeinit
    #define aicwf_bus_reg_init          aicwf_sdio_reg_init
    #define aicwf_bus_rx_task           aicwf_sdio_rx_task
    #define aicwf_bus_buf_init          aicwf_sdio_buf_init
    #define aicwf_bus_buf_free          aicwf_sdio_buf_free
    #define aicwf_bus_tx_init           aicwf_sdio_tx_init
    #define aicwf_bus_send              aicwf_sdio_send
    #define aicwf_bus_send_check        aicwf_sdio_send_check
    #define aicwf_bus_flow_ctrl         aicwf_sdio_flow_ctrl
    #define aicwf_bus_tx_msg            aicwf_sdio_tx_msg
    #define aicwf_bus_func_flag_tx_set  aicwf_sdio_func_flag_tx_set
    #define aicwf_bus_func_flag_rx_set  aicwf_sdio_func_flag_rx_set
    #define aicwf_bus_crc8_ponl_107     aicwf_crc8_ponl_107
#elif defined(CONFIG_USB_SUPPORT)
    /*
    * USB does not need the following SDIO concepts (cf. Linux aicwf_usb.c):
    *   - claim_host/release_host (no SDIO host concept on USB)
    *   - readsb/writesb (USB uses URB bulk in/out)
    *   - readb_cmd52/writeb_cmd52 (no CMD52 register access on USB)
    *   - release_func2 (SDIO function 2 release, not applicable to USB)
    *   - host_enable_isr/set_irq_handler/interrupt_init/isr (USB uses URB completion)
    *   - func_init_v3 (SDIO v3 register init, not applicable to USB)
    * Therefore these aicwf_bus_* macros are not defined in USB mode.
    */
    #define aicwf_bus_func_init         aicwf_usb_func_init
    #define aicwf_bus_drv_register      aicwf_usb_drv_register
    #define aicwf_bus_host_init         aicwf_usb_host_init
    #define aicwf_bus_host_deinit       aicwf_usb_host_deinit
    #define aicwf_bus_host_predeinit    aicwf_usb_host_predeinit
    #define aicwf_bus_reg_init          aicwf_usb_reg_init
    #define aicwf_bus_rx_task           aicwf_usb_rx_task
    #define aicwf_bus_buf_init          aicwf_usb_buf_init
    #define aicwf_bus_buf_free          aicwf_usb_buf_free
    #define aicwf_bus_tx_init           aicwf_usb_tx_init
    #define aicwf_bus_send              aicwf_usb_send
    #define aicwf_bus_send_check        aicwf_usb_send_check
    #define aicwf_bus_flow_ctrl         aicwf_usb_flow_ctrl
    #define aicwf_bus_tx_msg            aicwf_usb_tx_msg
    #define aicwf_bus_func_flag_tx_set  aicwf_usb_func_flag_tx_set
    #define aicwf_bus_func_flag_rx_set  aicwf_usb_func_flag_rx_set
    #define aicwf_bus_crc8_ponl_107     aicwf_usb_crc8_ponl_107
#endif

/* ---- Bus-agnostic dev structure accessor --------------------------- *
 * Upper layers that need the bus device struct should use these names;
 * they map to the active bus's device struct.
 */
#ifdef CONFIG_SDIO_SUPPORT
    #define aic_bus_dev_t              struct aic_sdio_dev
    #define aic_bus_dev                sdio_dev
#elif defined(CONFIG_USB_SUPPORT)
    #define aic_bus_dev_t              struct aic_usb_dev
    #define aic_bus_dev                g_aic_usb_dev
#endif

#endif /* _AIC_BUS_H_ */
