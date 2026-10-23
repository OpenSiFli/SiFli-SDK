/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wifi_api.h"
#include "sys_api.h"      /* synced from baseline d4509b7: aic_pwrctl_enable_set declaration (used by the [DEMO] pwrctl block below) */
#include "wifi_if.h"      /* [AIC8800-PORT][SiFli] aic_wifi_init_mac declaration */
#include "netif_api.h"
#include "fmac_port.h"
#include "aic_bus.h"
#include "wifi_port_priv.h"
#include "netif_port.h"
#include "aic_assert.h"
#include "aic_log.h"
#include "plat_port.h"
#include "aic_chip_desc.h"
#include <rtthread.h>          /* [AIC8800-PORT][SiFli] originally FreeRTOS.h; RT_WAITING_FOREVER */
#ifdef AIC8800_USE_RT_WLAN
    #include "wlan_mgnt.h"         /* [AIC8800-PORT][SiFli] rt_wlan_set_mode / RT_WLAN_STATION */
#endif
#ifdef CONFIG_SDIO_SUPPORT
    #include <drivers/mmcsd_core.h> /* [AIC8800-PORT][SiFli] originally "mmcsd_core.h": MMC core types/interfaces */
    #include "sdio_port.h"          /* aicwf_sdio_drv_register declaration */

    /* [AIC8800-PORT][SiFli] mmcsd_get_stat is not declared in mmcsd_core.h (defined in mmcsd_core.c:632),
    * declared locally here in the same way as swt6621s/wifi_device.c:72. */
    extern uint8_t mmcsd_get_stat(void);
#endif

extern void wifi_driver_init(void);
extern void aic_wifi_cmd(char *atcmd, unsigned int athandle, unsigned char chan_num);

#define lwipx_ipv4_addr(addr) ((addr[0]) | (addr[1] << 8) | (addr[2] << 16) | (addr[3] << 24))

/**
 * @brief Wait for the bus device to become ready
 *
 * Calls the ready-wait and driver registration of the corresponding bus
 * according to the compile-time bus selection (AIC_BUS_TYPE).
 * - SDIO: block waiting for the SDIO device to be inserted and register the SDIO driver
 * - USB:  call the USB driver registration (returns directly at the stub stage)
 */
static void aicwf_aic_wifi_bus_waitready(void)
{
#ifdef CONFIG_SDIO_SUPPORT
    /* [AIC8800-PORT][SiFli] mmcsd_wait_cd_changed() must not be used to block on MMCSD_HOST_PLUGED:
     * mmcsd_core.c posts a message to mmcsd_hotpluge_mb only on successful SD card/eMMC probe
     * and on card removal, not on the SDIO probe branch, so in a pure SDIO scenario that wait
     * never returns and the main flow hangs.
     * Use polling on mmcsd_get_stat() instead (the probe flow has completed), consistent with
     * rt_check_sdio_ready() in swt6621s/wifi_device.c. */
    int timeout = 50;   /* wait at most 5s */

    while (timeout-- > 0)
    {
        if (mmcsd_get_stat())
            break;
        aicwf_rtos_msleep(100);
    }

    if (mmcsd_get_stat())
        DBG_SDIO_INF("detect sdio device\r\n");
    else
        DBG_SDIO_ERR("sdio detect timeout\r\n");

    aicwf_sdio_drv_register();
#elif defined(CONFIG_USB_SUPPORT)
    DBG_USB_INF("usb bus waitready (stub)\r\n");
    aicwf_usb_drv_register();
#endif
}

/**
 * @brief Wait for the WiFi driver initialization to complete
 *
 * Polls GetWiFiInitStat() until the driver initialization completes.
 */
static void aicwf_aic_wifi_wait_driver_init(void)
{
    while (!GetWiFiInitStat())
    {
        aicwf_rtos_msleep(200);
    }
}
/**
 * @brief AIC WiFi driver demo test entry
 *
 * Complete WiFi driver startup flow:
 * 1. Platform initialization
 * 2. Wait for the SDIO device to be ready
 * 3. Initialize the WiFi driver
 * 4. Wait for the driver initialization to complete
 * 5. Initialize MAC and the network interface
 * 6. Configure default IP/netmask
 * 7. Configure the GPIO triggers
 */
//start aic wifi driver
void aic_wifi_demo_test(void)
{
    uint8_t      addr_server[4] = {192, 168, 13, 1};
    uint8_t      addr_netmask[4] = {255, 255, 255, 0};
    uint32_t     server, mask;
    server = lwipx_ipv4_addr(addr_server);
    mask = lwipx_ipv4_addr(addr_netmask);

    /* [DEMO] SDIO bus pwrctl runtime switch — timing: must be before wifi_driver_init().
     * The pwrctl task/timer decides whether to be created inside the driver initialization
     * (rwnx_main) based on the initial value of g_aic_pwrctl_enable; later changes have no effect.
     *   aic_pwrctl_enable_set(0) = debug: bus always ACTIVE, ruling out sleep timing interference;
     *   aic_pwrctl_enable_set(1) = production default (1 when not called). */
#if defined(CONFIG_SDIO_BUS_PWRCTRL) || defined(CONFIG_USB_SUPPORT)
    aic_pwrctl_enable_set(0);
#endif

    aicwf_aic_wifi_bus_waitready();
    wifi_driver_init();
    aicwf_aic_wifi_wait_driver_init();
    aic_wifi_init_mac();
    // aicwf_net_if_plat_init();

#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: bind wlan0 to the rt_wlan STA role. The framework's _sta_mgnt.device
     * is only assigned inside rt_wlan_set_mode(); while unbound, wifi scan/join returns
     * -RT_EIO directly; the lwIP prot is also attached at this step, and the framework creates
     * and registers the netif (w0). Timing: must be after aic_wifi_init_mac() (the wlan_mode op
     * switches the chip mode). The AP side (wlan1) is not bound here: aic_wifi_mode_set has only
     * a single global mode, and binding AP at boot would switch the chip away; call
     * rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP) when SoftAP is needed. */
    rt_err_t wlan_err = rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    if (wlan_err != RT_EOK)
        DBG_APP_ERR("set_mode(%s, STA) failed: %d\r\n", RT_WLAN_DEVICE_STA_NAME, wlan_err);
#endif

    /* [DEMO] FW log level configuration — timing: must be called after the driver initialization
     * completes (after aic_wifi_init_mac), and depends on g_rwnx_hw and the message channel being
     * ready. One-stop enable: (1) 0x13 feedback channel master switch (2) lp_level master gate
     * (3) module filter (4) level threshold. Here all modules + >=INF are configured.
     * DC/DW family does not support FW log feedback (CFG_PRINT 0x13 channel), so skip it. */
    if (aic_chip_desc_chipid_get() != AIC_CHIP_ID_8800DC &&
            aic_chip_desc_chipid_get() != AIC_CHIP_ID_8800DW)
    {
        aic_wifi_fw_log_enable(AIC_FW_MOD_DEFAULT, AIC_FW_SEV_INF);
    }
#if defined(AIC_FEATURE_BT) && defined(AIC_FEATURE_NIMBLE)
    /* The BT patch has already been loaded by the driver initialization (host calls aicbt_init unconditionally);
     * start the NimBLE stack task here (provided by STACK/nimble). The symbol exists only if
     * AIC_FEATURE_NIMBLE pulls in nimble, otherwise linking fails; hence the AND with the BT gate.
     * nimble_stack_start checks aic_bt_fw_ready() internally and refuses to start if fw is not loaded. */
    {
        extern int nimble_stack_start(void);
        (void)nimble_stack_start();
    }
#endif


    aicwf_net_if_set_tousedip(server);
    aicwf_net_if_set_netmask(mask);

    aicwf_plat_gpio_trigger(DRV_TRG_SDIO_RX, 0);
    aicwf_plat_gpio_trigger(DRV_TRG_FHOST_RX, 0);
    aicwf_plat_gpio_trigger(DRV_TRG_FHOST_TX, 0);
    aicwf_plat_gpio_trigger(DRV_TRG_NET_TX, 0);
    aicwf_plat_gpio_trigger(DRV_TRG_SDIO, 0);
    aicwf_plat_gpio_trigger(DRV_TRG_POWCTR, 0);
    aicwf_plat_gpio_trigger(DRV_TRG_USB, 0);
}
/**
 * @brief AT command test interface
 *
 * Forwards the AT command received on the UART to the WiFi command handler.
 *
 * @param temp_uart_buf AT command string received on the UART
 */
//test aic wifi command
void cmd_test(const char *temp_uart_buf)
{
    aic_wifi_cmd((char *)temp_uart_buf, 0, 0);
}
