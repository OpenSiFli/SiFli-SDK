/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aic_log.h"
#include "plat_port.h"
#include "compiler.h"
#include "rtos_port.h"
#include <stdarg.h>
// [PLATFORM] RT-Thread platform headers
#ifdef RT_THREAD
    #include <rtthread.h>
    #include <drivers/pin.h>
#endif
#include "aic_assert.h"
#include <stdio.h>
#include "userconfig.h"

/* When PWRON is not directly connected to the SoC but attached to an I2C IO expander, use the
 * board-level expander interface to control power on/off */
#ifdef AIC_WIRELESS_PWR_VIA_IOEXP
    #include "aw9523.h"
#endif

#ifndef LINE
    #define LINE "\r\n"
#endif

/* Power-on pin: injected by SConscript from Kconfig AIC8800_RTOS_WIRELESS_PWR_GPIO;
 * a negative value means not connected/not configured — no GPIO operation is performed then and
 * power-on is guaranteed by the hardware. */
#ifndef AIC_WIRELESS_PWR_GPIO
    #define AIC_WIRELESS_PWR_GPIO (-1)
#endif

/* Supply enable pin on the IO expander (only used with AIC_WIRELESS_PWR_VIA_IOEXP),
 * encoded as in aw9523.h: 0xPp (P=port 0/1, p=bit 0~7). */
#ifndef AIC_WIRELESS_PWR_IOEXP_PIN
    #define AIC_WIRELESS_PWR_IOEXP_PIN (0x15)   /* AW9523 P1_5 = AW9523_PIN_WF_PWREN, WiFi supply enable */
#endif

/* "Power-on sequence" pin on the IO expander: driven only after the supply is established, so
 * the module starts after completing its power-on sequence.
 * On this board it is AW9523 P1_4 (AW9523_PIN_WF_PWRKEY); the level polarity matches the supply pin (1=active). */
#ifndef AIC_WIRELESS_PWR_IOEXP_PIN_KEY
    #define AIC_WIRELESS_PWR_IOEXP_PIN_KEY (0x14)
#endif

/* Power-on sequence parameters (ref. aic8800d40 driver aic8800_ele):
 * pull low -> delay -> pull high -> delay to wait for the chip ROM to boot */
#define AIC_WIFI_PWR_OFF_MS     (10)
#define AIC_WIFI_PWR_ON_MS      (200)

/* Task Priority */
uint32_t sdio_datrx_priority       = SDIO_DATRX_PRIORITY;
uint32_t sdio_oobirq_priority      = SDIO_OOBIRQ_PRIORITY;
uint32_t sdio_pwrclt_priority      = SDIO_PWRCLT_PRIORITY;
uint32_t fhost_cntrl_priority      = FHOST_CNTRL_PRIORITY;
uint32_t fhost_wpa_priority        = FHOST_WPA_PRIORITY;
uint32_t fhost_tx_priority         = FHOST_TX_PRIORITY;
uint32_t fhost_rx_priority         = FHOST_RX_PRIORITY;
uint32_t fhost_tcpack_priority     = FHOST_TCPACK_PRIORITY;
uint32_t cli_cmd_priority          = CLI_CMD_PRIORITY;
uint32_t rwnx_timer_priority       = RWNX_TIMER_PRIORITY;
uint32_t rwnx_wifi_priority        = RWNX_WIFI_PRIORITY;
uint32_t rwnx_apm_priority = RWNX_APM_STALOSS_PRIORITY;
uint32_t tcpip_priority            = TCPIP_PRIORITY;
uint32_t task_end_prio             = TASK_END_PRIO;

/* Task Stack Size */
uint32_t sdio_datrx_stack_size       = SDIO_DATRX_STACK_SIZE;
uint32_t sdio_oobirq_stack_size      = SDIO_OOOBIRQ_STACK_SIZE;
uint32_t sdio_pwrctl_stack_size      = SDIO_PWRCTL_STACK_SIZE;
uint32_t fhost_cntrl_stack_size      = FHOST_CNTRL_STACK_SIZE;
uint32_t fhost_wpa_stack_size        = FHOST_WPA_STACK_SIZE;
uint32_t fhost_tx_stack_size         = FHOST_TX_STACK_SIZE;
uint32_t fhost_rx_stack_size         = FHOST_RX_STACK_SIZE;
uint32_t fhost_tcpack_stack_size     = FHOST_TCPACK_STACK_SIZE;
uint32_t cli_cmd_stack_size          = CLI_CMD_STACK_SIZE;
uint32_t rwnx_timer_stack_size       = RWNX_TIMER_STACK_SIZE;
uint32_t rwnx_wifi_stack_size        = RWNX_WIFI_STACK_SIZE;
uint32_t rwnx_apm_stack_size         = RWNX_APM_STACK_SIZE;
/* driver param setting */
/* define WPA log level */
enum
{
    WPA_MSG_EXCESSIVE,
    WPA_MSG_MSGDUMP,
    WPA_MSG_DEBUG,
    WPA_MSG_INFO,
    WPA_MSG_WARNING,
    WPA_MSG_ERROR
};
//dcdc mode
int aicwifi_vrf_mode = VRF_DCDC_MODE;
//wpa debug level
int aicwifi_wpa_log_level = WPA_MSG_INFO; /* DEBUG floods RX_MGMT / neighbor BSS beacons */

#ifndef AIC_WIRELESS_PWR_VIA_IOEXP
    /* Power-on pin number (only used by the direct GPIO path); a negative value means invalid */
    static int aicwf_wifi_pwr_pin = -1;
#endif

/* Whether power control is usable: set when the direct GPIO path configures the pin successfully, or
 * when the IO expander initializes successfully.
 * Used by set/enable/disable/reset for a uniform check, replacing the previous "pin number < 0 means invalid" criterion. */
static rt_bool_t aicwf_wifi_pwr_ready = RT_FALSE;

/* Set once the first power-on sequence completes (triggered by the board-level INIT_BOARD); used to
 * suppress redundant power cycles afterwards.
 * The prebuilt library aic_wifi_open() still performs pin_init/disable/10ms/enable internally, but by then
 * SDIO has already been enumerated and runs in 4-bit/high-speed mode; a single chip power-on reset loses the
 * RCA and bus width configuration, and the host never re-enumerates, causing all subsequent CMD52
 * timeouts (real-device log 0x18001).
 * Ref. aic8800d40 driver: the same power cycle is disabled by #if 0 in aic8800_init(). */
static rt_bool_t aicwf_wifi_pwr_locked = RT_FALSE;

static void aicwf_wifi_pwr_pin_set(int on)
{
#ifdef AIC_WIRELESS_PWR_VIA_IOEXP
    /* Supply and power-on sequence are both controlled by the I2C IO expander:
     *   power on:  enable the supply first (P1_5), then drive the power-on sequence pin (P1_4);
     *   power off: release the power-on sequence pin first, then disable the supply. */
    if (aicwf_wifi_pwr_ready)
    {
        if (on)
        {
            aw9523_set_pin((uint8_t)AIC_WIRELESS_PWR_IOEXP_PIN, 1);
            aw9523_set_pin((uint8_t)AIC_WIRELESS_PWR_IOEXP_PIN_KEY, 1);
        }
        else
        {
            aw9523_set_pin((uint8_t)AIC_WIRELESS_PWR_IOEXP_PIN_KEY, 0);
            aw9523_set_pin((uint8_t)AIC_WIRELESS_PWR_IOEXP_PIN, 0);
        }
    }
#else
    if (aicwf_wifi_pwr_pin >= 0)
    {
        rt_pin_write(aicwf_wifi_pwr_pin, on ? PIN_HIGH : PIN_LOW);
    }
#endif
}

/**
 * @brief Initialize WiFi power control (platform-specific)
 * @note Direct GPIO path: the pin number comes from Kconfig AIC8800_RTOS_WIRELESS_PWR_GPIO
 *       (injected as AIC_WIRELESS_PWR_GPIO by SConscript); a negative value means unused, and all
 *       subsequent operations are skipped.
 *       IO expander path: calls the board-level aw9523_init(); failure is treated as unavailable.
 */
void aicwf_platform_pwr_wifi_pin_init(void)
{
    if (aicwf_wifi_pwr_ready)
    {
        return; /* already initialized; do not configure again */
    }

#ifdef AIC_WIRELESS_PWR_VIA_IOEXP
    /* PWRON is attached to the I2C IO expander; power on/off is controlled through the expander interface.
     * aw9523_init() depends on the I2C bus being ready, so on this branch platform_init.c lowers the
     * registration level of the power-on hook to INIT_PREV_EXPORT (still earlier than the SDIO host's INIT_DEVICE_EXPORT). */
    if (aw9523_init() == 0)
    {
        aicwf_wifi_pwr_ready = RT_TRUE;
        DBG_CMN_INF("wifi pwr on IO expander: supply pin=0x%02x, seq pin=0x%02x\n",
                    AIC_WIRELESS_PWR_IOEXP_PIN, AIC_WIRELESS_PWR_IOEXP_PIN_KEY);
    }
    else
    {
        DBG_CMN_ERR("wifi pwr: aw9523_init fail, skip power control\n");
    }
#else
    aicwf_wifi_pwr_pin = AIC_WIRELESS_PWR_GPIO;
    if (aicwf_wifi_pwr_pin >= 0)
    {
        rt_pin_mode(aicwf_wifi_pwr_pin, PIN_MODE_OUTPUT);
        aicwf_wifi_pwr_ready = RT_TRUE;
        DBG_CMN_INF("wifi pwr pin = %d\n", aicwf_wifi_pwr_pin);
    }
#endif
}

/**
 * @brief Enable the WiFi power pin (platform-specific)
 * @note After the first pull-high (board power-on sequence), aicwf_wifi_pwr_locked is set and
 *       no further operations are performed.
 */
void aicwf_platform_pwr_wifi_pin_enable(void)
{
    if (aicwf_wifi_pwr_locked)
    {
        return;
    }

    aicwf_wifi_pwr_pin_set(1);
    aicwf_wifi_pwr_locked = RT_TRUE;
}

/**
 * @brief Disable the WiFi power pin (platform-specific)
 * @note Calls after the power-on sequence completes are ignored: SDIO is already enumerated, and a
 *       power-down would reset the chip and make subsequent CMD52 time out; for a real
 *       power-off/reset use aicwf_sdio_pdn_wifi() / aicwf_sdio_reset_wifi() (they clear this protection first).
 */
void aicwf_platform_pwr_wifi_pin_disable(void)
{
    if (aicwf_wifi_pwr_locked)
    {
        return;
    }

    aicwf_wifi_pwr_pin_set(0);
}

/**
 * @brief Format a string into a buffer (sprintf for debugging)
 * @param buffer output buffer
 * @param fmt    format string
 * @param ...    variable arguments
 * @return number of characters written
 */
uint32_t aicwf_dbg_sprintf(char *buffer, const char *fmt, ...)
{
    uint32_t ret;
    const char *fmt_usr = (const char *) fmt;

    // print
    va_list args;
    va_start(args, fmt);

    ret = dbg_vsprintf(buffer, fmt_usr, args);

    va_end(args);

    return ret;
}

/**
 * @brief Format a string into a buffer (snprintf for debugging, with length limit)
 * @param buffer output buffer
 * @param size   buffer size
 * @param fmt    format string
 * @param ...    variable arguments
 * @return number of characters written
 */
uint32_t aicwf_dbg_snprintf(char *buffer, uint32_t size, const char *fmt, ...)
{
    uint32_t ret;
    const char *fmt_usr = (const char *) fmt;

    // print
    va_list args;
    va_start(args, fmt);

    ret = dbg_vsnprintf(buffer, size, fmt_usr, args);

    va_end(args);

    return ret;
}

/**
 * @brief Check whether the device is in production mode
 * @return 0=not production mode, non-zero=production mode
 */
uint32_t aicwf_hal_IsInProduction_Mode(void)
{
    return 0;
}

/**
 * @brief Switch the PP (power/performance) level
 * @param lvl PP level
 */
void aicwf_hal_switchPP(uint8_t lvl)
{

}

/**
 * @brief Reset the WiFi chip via SDIO
 * @return 0=success, non-zero=failure
 * @note This board has no dedicated reset pin; the reset is performed by the power cycle in
 *       aicwf_sdio_reset_wifi(); the SDIO bus itself is re-enumerated by the MMC core, so this
 *       returns success directly.
 */
int aicwf_sdio_reset(void)
{
    return 0;
}

/**
 * @brief Power down WiFi via SDIO (platform-specific)
 * @note Pulls the power pin low; no-op when no power-on pin is configured. This is an explicit
 *       power-off operation, so the power-on protection is cleared first.
 */
void aicwf_sdio_pdn_wifi(void)
{
    aicwf_wifi_pwr_locked = RT_FALSE;
    aicwf_platform_pwr_wifi_pin_disable();
}

/**
 * @brief Hardware reset the WiFi chip (platform-specific)
 * @note No dedicated reset pin; a power cycle is used as an equivalent reset (same sequence as
 *       aicwf_wifi_power_on).
 *       This is an explicit reset operation, so the power-on protection is cleared first;
 *       afterwards SDIO must be re-enumerated by the MMC core.
 */
void aicwf_sdio_reset_wifi(void)
{
    if (!aicwf_wifi_pwr_ready)
    {
        return;
    }

    aicwf_wifi_pwr_locked = RT_FALSE;
    aicwf_platform_pwr_wifi_pin_disable();
    aicwf_rtos_msleep(AIC_WIFI_PWR_OFF_MS);
    aicwf_platform_pwr_wifi_pin_enable();
    aicwf_rtos_msleep(AIC_WIFI_PWR_ON_MS);
}

/**
 * @brief AT command response output
 * @param handle interface handle
 * @param type   response type
 * @param level  log level
 * @param buf    response content string
 */
void aicwf_ATRESP(int handle, int type, int level, char *buf)
{
    (void)handle;
    (void)type;
    (void)level;
    /* [BUGFIX] (common backport) Output through the AIC debug log: serial captures only collect [AIC][*] DBG output;
     * the original implementation used only rt_kprintf under RT_THREAD / only DBG_CMN_INF (gated by a module
     * switch) otherwise, so scan/connect results (+OK / [n]:rssi=...) were invisible in AIC capture logs.
     * If the platform needs additional console output, add it here (e.g. RT-Thread rt_kprintf) */
    if (buf)
        DBG_APP_INF("%s\n", buf);
}
/**
 * @brief Hex dump a data buffer (for debugging)
 *
 * Prints the data contents in lines of 16 bytes; lines shorter than 16 bytes are handled automatically.
 *
 * @param tag  tag string
 * @param data data buffer
 * @param len  data length
 */
void aicwf_dump_buffer(char *tag, uint8_t *data, uint32_t len)
{
    int i, diff = 0;
    DBG_CMN_VRB("dump:%s, len(%d)", tag, len);
    for (i = 0; i < len; i += 16)
    {
        if ((i + 16) <= len)
        {
            DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x" LINE, &data[i],
                        data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                        data[i + 8], data[i + 9], data[i + 10], data[i + 11], data[i + 12], data[i + 13], data[i + 14], data[i + 15]);
        }
        else
        {
            diff = len - i;
            if (diff == 15)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8], data[i + 9], data[i + 10], data[i + 11], data[i + 12], data[i + 13], data[i + 14]);
            if (diff == 14)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8], data[i + 9], data[i + 10], data[i + 11], data[i + 12], data[i + 13]);
            if (diff == 13)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8], data[i + 9], data[i + 10], data[i + 11], data[i + 12]);
            if (diff == 12)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8], data[i + 9], data[i + 10], data[i + 11]);
            if (diff == 11)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8], data[i + 9], data[i + 10]);
            if (diff == 10)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8], data[i + 9]);
            if (diff == 9)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7],
                            data[i + 8]);
            if (diff == 8)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],  data[i + 4],  data[i + 5],  data[i + 6],  data[i + 7]);
            if (diff == 7)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],
                            data[i + 4], data[i + 5], data[i + 6]);
            if (diff == 6)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],
                            data[i + 4], data[i + 5]);
            if (diff == 5)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3],
                            data[i + 4]);
            if (diff == 4)
                DBG_CMN_VRB("[%x] %02x %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2],  data[i + 3]);
            if (diff == 3)
                DBG_CMN_VRB("[%x] %02x %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1], data[i + 2]);
            if (diff == 2)
                DBG_CMN_VRB("[%x] %02x %02x" LINE, &data[i],
                            data[i],   data[i + 1]);
            if (diff == 1)
                DBG_CMN_VRB("[%x] %02x" LINE, &data[i],
                            data[i]);
        }
    }
}

/**
 * @brief GPIO trigger control (RT-Thread platform implementation)
 * @param id    GPIO ID
 * @param value GPIO output value
 * @note Adjust according to the GPIO pin configuration of the specific hardware platform
 */
/* GPIO trigger pin mapping table: trigger id (DRV_TRG_*, a logical concept defined in plat_port.h)
 * → board-level GPIO pin (a physical concept). id and pin are independent; on the default platform
 * their values happen to be identical, so the default mapping table references DRV_TRG_* directly.
 * When porting to other platforms, only the pin values in this table and the actual output call
 * below need to be modified. */
#define AIC_TRG_GPIO_SDIO       DRV_TRG_SDIO
#define AIC_TRG_GPIO_SDIO_RX    DRV_TRG_SDIO_RX
#define AIC_TRG_GPIO_FHOST_RX   DRV_TRG_FHOST_RX
#define AIC_TRG_GPIO_FHOST_TX   DRV_TRG_FHOST_TX
#define AIC_TRG_GPIO_NET_TX     DRV_TRG_NET_TX
#define AIC_TRG_GPIO_POWCTR     DRV_TRG_POWCTR
#define AIC_TRG_GPIO_USB        DRV_TRG_USB

void aicwf_plat_gpio_trigger(uint8_t id, uint8_t value)
{
    uint8_t pin;

    /* id → pin mapping: the board-level pin for each trigger point is determined by the platform mapping table */
    switch (id)
    {
    case DRV_TRG_SDIO:
        pin = AIC_TRG_GPIO_SDIO;
        break;
    case DRV_TRG_SDIO_RX:
        pin = AIC_TRG_GPIO_SDIO_RX;
        break;
    case DRV_TRG_FHOST_RX:
        pin = AIC_TRG_GPIO_FHOST_RX;
        break;
    case DRV_TRG_FHOST_TX:
        pin = AIC_TRG_GPIO_FHOST_TX;
        break;
    case DRV_TRG_NET_TX:
        pin = AIC_TRG_GPIO_NET_TX;
        break;
    case DRV_TRG_POWCTR:
        pin = AIC_TRG_GPIO_POWCTR;
        break;
    case DRV_TRG_USB:
        pin = AIC_TRG_GPIO_USB;
        break;
    default:
        return; /* unknown trigger id, ignore */
    }

    /* [PLATFORM] Actual GPIO output call; each platform implements it with its own SDK API, e.g.:
     * RT-Thread: rt_pin_mode(pin, PIN_MODE_OUTPUT);
     *            rt_pin_write(pin, value ? PIN_HIGH : PIN_LOW);
     * other:     gpio_direction_output(pin, value); */
    (void)pin;
    (void)value;
}

// ============================================================
// HAL + LOG extern wrappers (called directly by host layer)
// ============================================================
#include <stdarg.h>

/* Functions moved from wifi_port.c */
extern void wifi_driver_action(unsigned int act);

/**
 * @brief Trigger a driver restart
 * @param mode restart mode code, passed to wifi_driver_action()
 */
static void aicwf_aic_driver_reboot(unsigned int mode)
{
    wifi_driver_action(mode);
}

/**
 * @brief Driver exception handling
 *
 * Performs the corresponding action based on the exception event type:
 * - 0x01/0x02/0x40/0x80/0x100: trigger a driver restart
 * - 0x04/0x20: trigger an assertion error
 * - others: no operation
 *
 * @param evt exception event code
 */
void aicwf_driver_unexcepted(unsigned int evt)
{
    switch (evt)
    {
    case 0x01:
        aicwf_aic_driver_reboot(0x03);
        break;
    case 0x02:
        aicwf_aic_driver_reboot(0x03);
        break;
    case 0x04:
        AIC_ASSERT_ERR(0);
        break;
    case 0x08:
        break;
    case 0x10:
        break;
    case 0x20:
        AIC_ASSERT_ERR(0);
        break;
    case 0x40:
        aicwf_aic_driver_reboot(0x03);
        break;
    case 0x80:
        aicwf_aic_driver_reboot(0x03);
        break;
    case 0x100:
        aicwf_aic_driver_reboot(0x03);
        break;
    case 0x200:
        break;

    default:
        break;
    }
}

/* LOG output — use rt_kprintf for RT-Thread platform */
/**
 * @brief Log output function (RT-Thread platform implementation)
 * @param fmt format string
 * @param ... variable arguments
 * @return return value of vsprintf (number of characters written)
 */
int aicwf_log_printf(const char *fmt, ...)
{
    va_list ap;
    char log_buf[256];
    int r;

    va_start(ap, fmt);
#ifdef RT_THREAD
    r = rt_vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
    va_end(ap);

    if (r > 0)
    {
        rt_kprintf("%s", log_buf);
    }
#else
    r = vsnprintf(log_buf, sizeof(log_buf), fmt, ap);
    va_end(ap);
    // [PLATFORM] Non-RT-Thread platforms must implement the output according to the actual environment
    // Prints to standard output by default
    if (r > 0)
    {
        fwrite(log_buf, 1, r, stdout);
    }
#endif
    return r;
}

/// Debug module environment definition. (moved here for host)
static struct debug_env_tag dbg_env =
{
    .filter_module = DBG_MOD_ALL,
    .filter_severity = DBG_SEV_IDX_INF,//DBG_SEV_IDX_VRB,//
};

/**
 * @brief Test module log level filtering
 *
 * Determines whether a log at the given severity level of the given module should be output.
 *
 * @param mod_idx module index
 * @param sev_idx severity level index
 * @return 1=the log should be output, 0=do not output
 */
int platform_test_module_severity(unsigned int mod_idx, unsigned int sev_idx)
{
    int ret = 0;
    if ((mod_idx < DBG_MOD_IDX_MAX) && (dbg_env.filter_module & (1 << mod_idx)) &&
            (dbg_env.filter_severity >= sev_idx))
    {
        ret = 1;
    }
    return ret;
}


/* ---- HAL implementations (extern linkage) ---- */
