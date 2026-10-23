/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
// ==========================================================================
// [AIC8800-PORT][SiFli] SDIO/MMC subsystem → RT-Thread SDIO framework of the SiFli SDK
//   Baseline targets TI mmcsd_core; the SiFli SDK uses RT-Thread SDIO (drivers/sdio.h).
// ==========================================================================
#include <rtthread.h>
#include <drivers/sdio.h>        /* rt_sdio_function/rt_sdio_driver + sdio_io_readb/writeb/sdio_register_driver/sdio_enable_func/sdio_attach_irq */
#include <drivers/mmcsd_core.h>  /* MMC/SD core */
#include <drivers/mmcsd_card.h>  /* rt_mmcsd_card + SDIO_MAX_FUNCTIONS */
#include "sdio_api.h"            /* driver host API (lib/include/drv_api/inc, already in CPPPATH) */
#include "sys_api.h"             /* driver host API */

// ==========================================================================
// [AIC8800-PORT][SiFli] Type compatibility layer: baseline TI-mmcsd names → RT-Thread SDIO names
//   Struct members (func: num/card/manufacturer/product/func_code/max_blk_size/
//   cur_blk_size; card: host/cis/cccr/sdio_function[]/sdio_function_num;
//   driver: name/probe/remove/id) and function names (sdio_register_driver/sdio_enable_func/
//   sdio_io_readb/writeb/sdio_attach_irq/sdio_io_rw_extended_block) are identical,
//   only the top-level type names differ, so macro bridging avoids renaming every site (47 sites)
//   and the regression risk that would bring.
//   SDIOM_MAX_FUNCS is defined as 3 in aic_sdio_types.h (driver-internal array bound); no mapping needed.
// ==========================================================================
#define sdio_func       rt_sdio_function
#define mmcsd_card      rt_mmcsd_card
#define sdio_driver_t   rt_sdio_driver
#define sdio_device_id  rt_sdio_device_id
#define sdio_irq_h_t    rt_sdio_irq_handler_t   /* replaces the line~433 fallback typedef */

/* [AIC8800-PORT][SiFli] The aic_sdio_* pass-through wrappers (static inline) are defined later in
 * this file in the "Driver functions define" section, but aicwf_sdio_drv_register / __aicwf_sdio_enable_function
 * are called earlier; GCC14 reports an error for "implicit declaration + static definition following a
 * non-static declaration", hence these forward declarations. */
static inline int aic_sdio_register_driver(struct sdio_driver_t *drv);
static inline int aic_sdio_enable_func(struct sdio_func *func);
// ==========================================================================
// [PLATFORM] AIC project internal headers below; normally no changes are needed
// ==========================================================================
#include "sdio_port.h"
#include "plat_port.h"
#include "rtos_port.h"
#include <errno.h>
#include "aic_log.h"
#include "co_math.h"
/* hal_co.h depends on host internal headers; only the required functions are declared */
extern void aicwf_hal_func_release_set(int value);
extern uint8_t aicwf_hal_func_release_get(void);
extern void aicwf_hal_flag_tx_set(bool value);
#ifdef CONFIG_OOB
#endif /* CONFIG_OOB */

/* ==========================================================================
 * [PLATFORM] SDIO bus abstraction layer — platform porting notes
 *
 * This file is the SDIO bus bridge between the AIC8800 driver and the target RTOS/SDK.
 * When porting to a new platform, only the platform-specific functions and data structures
 * marked [PLATFORM] at the top of this file need changes; do not modify the host driver
 * logic layered on top of these wrappers (host/driver/layer_common/src/sdio_co.c).
 *
 * ---- Standard SDIO card scan / enumeration (done by the platform MMC/SDIO core; this file consumes the result) ----
 * The standard sequence the host uses to identify an SDIO card is as follows
 * (completed by the platform MMC/SDIO subsystem before probe):
 *   CMD0  GO_IDLE_STATE       reset the card to idle
 *   CMD8  SEND_IF_COND        query voltage/interface conditions, distinguish SD from SDIO
 *   CMD5  IO_SEND_OP_COND     SDIO-specific: query IO capability (voltage/number of functions/ready),
 *                             equivalent to ACMD41 for SD cards
 *   CMD3  SEND_RELATIVE_ADDR  obtain RCA (relative card address)
 *   CMD7  SELECT_CARD         select the card by RCA; afterwards CMD52/CMD53 to function 0 are usable
 *
 * ---- Function mapping after card scan ----
 * An SDIO card has Functions 0..7, where:
 *   Function 0 is the mandatory common control function, carrying CCCR + FBR + CIS:
 *     CCCR (Card Common Control Registers): common control area, including I/O capability/interrupt enable (IEN/IENM)
 *     FBR  (Function Basic Registers): per-function interface code/block size/capability
 *     CIS  (Card Information Structure): vendor/product ID (VID/PID)
 *   Functions 1..7 are optional I/O functions (AIC8800 usually uses Function 1 for WiFi; DC/DW also has Function 2)
 * In __aicwf_aic_drv_init() this file maps the platform function list to
 * sdio_function_local[] by func->num; the host then accesses it via FUNC_0/FUNC_1/FUNC_2 indices.
 *
 * ---- CMD52 / CMD53 ----
 *   CMD52 (IO_RW_DIRECT): single-byte register read/write, used to access CCCR/FBR, enable I/O, set interrupts.
 *     Wrapped in this file as aicwf_sdio_readb_cmd52* / aicwf_sdio_writeb_cmd52*.
 *   CMD53 (IO_RW_EXTENDED): block/byte bulk read/write, used for firmware download and data transfer.
 *     Wrapped in this file as aicwf_sdio_readsb / aicwf_sdio_writesb / aicwf_sdio_writesb_msg.
 *
 * ---- Interrupt configuration ----
 *   SDIO IRQ is controlled jointly by IENM (Int Enable Master) in CCCR and each function's
 *   IEN (Int Enable). During probe this file always declares host support for SDIO IRQ
 *   (MMCSD_SUP_SDIO_IRQ); at runtime enabling/disabling is done via aicwf_sdio_isr(en),
 *   decoupled from whether RX is polled (aicwifi_sdio_rxdata_mode).
 *
 * ---- Platform-specific functions (must be adapted when porting) ----
 *   The following functions depend directly on the platform MMC/SDIO subsystem API and are the
 *   focus of porting:
 *     __aicwf_aic_drv_init / __aicwf_aic_dev_remove (probe/remove callbacks)
 *     aicwf_sdio_drv_register (driver registration)
 *     aicwf_sdio_claim_host / aicwf_sdio_release_host (host mutex)
 *     __aicwf_sdio_enable_function (function enable)
 *     aicwf_mmc_enable_isr (ISR enable)
 *     aic_sdio_io_rw_extended_block / aic_sdio_io_readb / aic_sdio_io_writeb (CMD52/53 low level)
 *     aic_sdio_register_driver / aic_sdio_enable_func (driver/function registration)
 *     aicwf_sdio_claim_irq / aicwf_sdio_release_irq (IRQ registration)
 *     aicwf_sdio_readsb / aicwf_sdio_writesb / aicwf_sdio_writesb_msg (CMD53 bulk transfer)
 *     __aicwf_sdio_get_func0 / __aicwf_sdio_f0_readb / aicwf_sdio_f0_writeb (Function 0 access)
 *   The remaining func_init / interrupt_init / set_block_size etc. are driver logic built on the
 *   wrappers above and normally need no changes.
 * ========================================================================== */

/* Runtime chipmatch global variables + getter — outside CONFIG_SDIO_SUPPORT, so
 * non-SDIO modes (e.g. USB) can still link this symbol (AICWF_CHIPID_UNMATCHED means
 * no match).
 * aicwf_get_chipid() in wifi_if.c uses this interface to prefer the runtime match result. */
uint16_t g_runtime_chipid = AICWF_CHIPID_UNMATCHED;
uint16_t g_runtime_vid = 0;
uint16_t g_runtime_pid = 0;

/* [PLATFORM][RAM] SDIO buffer pool budget (interface in sdio_api.h)
 *   RX:      2048B per packet, library default 64 => 128KB; set to 40 => 80KB
 *            (aligned with the aic8800d40 driver default; 24MHz 4-bit SDIO has ample throughput margin)
 *   TX aggregation: 1548B per packet, library default 28 => 42KB; set to 16 => 24KB
 * The two pools together drop from 170KB to 105KB, relieving exhaustion of the 251KB system
 * heap on the 57-lcd board (observed as [rtos]malloc fail -> start sta fail -7 during
 * wpa ADD_NETWORK).
 * Must be applied before aicwf_sdio_host_init(); aic_wifi_open() in the library first does
 * memset(sdio_dev, 0, 152), and this getter is the only platform callback between that
 * memset and host_init, so it is applied here. */
#define AIC_SDIO_RX_BUF_CNT     40
#define AIC_SDIO_TX_AGGR_CNT    16

uint16_t aicwf_get_runtime_chipid(void)
{
#ifdef CONFIG_SDIO_SUPPORT
    extern struct aic_sdio_dev sdio_dev;   /* defined in the CONFIG_SDIO_SUPPORT section of this file */

    aic_sdio_rx_buf_config(&sdio_dev, AIC_SDIO_RX_BUF_CNT);
    aic_sdio_tx_aggr_config(&sdio_dev, AIC_SDIO_TX_AGGR_CNT);
#endif
    return g_runtime_chipid;
}

#ifdef CONFIG_SDIO_SUPPORT

uint32_t  SDIO_DEVICEID = 0xc18d;   // 8800DC/DW/DL  // wait for using.

//-------------------------------------------------------------------
// Driver Variables define
//-------------------------------------------------------------------
int msgcfm_poll_en = 1;
uint8_t sdio_unexcept_test = 0;
struct aic_sdio_dev sdio_dev = {0,};

/* ==========================================================================
 * SDIO(DAT1) interrupt storm avoidance — keep only the single switch "do not unmask SDIO IRQ"
 *
 * Reason: SDHCI's SDHCI_INT_CARD_INT is a level interrupt (remains asserted while the card pulls DAT1 low),
 *       while the library's aicwf_sdio_rx_task unconditionally calls sdio_host_enable_isr(TRUE) at the end of
 *       every round, forming an infinite loop: "interrupt -> wake RX task -> read intstatus=0 -> unmask interrupt -> interrupt again immediately".
 *       That interrupt has a higher NVIC priority than the UART, so UART RX is never serviced => the shell gets no input,
 *       which shows up as the whole system hanging (prints "msh />" once, then no echo on Enter).
 *
 * Workaround: when g_sdio_irq_mask_forced = 1, every "enable SDIO interrupt" path in this file
 *       keeps the interrupt masked; packet reception falls back to the library's 10ms polling
 *       (aicwifi_sdio_rxdata_mode must also be set to 1, see example/aic8800_d80/src/main.c).
 * ========================================================================== */
uint32_t g_sdio_irq_mask_forced = 0;            /* 1 = do not unmask the SDIO interrupt */
/* Runtime-configurable SDIO behavior flags (see sdio_port.h) */
uint8_t config_rxtask_insdio = CONFIG_RXTASK_INSDIO_DEFAULT;
uint8_t config_irq_sus = CONFIG_IRQ_SUS_DEFAULT;
//sdio rxdata mode
unsigned char aicwifi_sdio_rxdata_mode = CONFIG_RX_POLLING_DEFAULT;
// [PLATFORM] sdio_func / SDIOM_MAX_FUNCS are defined by the platform SDIO headers
// TODO: [AIC8800-PORT] confirm that platform struct sdio_func member names match
struct sdio_func *sdio_function_local[SDIOM_MAX_FUNCS];

/* Platform ADMA Variables Import */
static void (*host_sdio_irq_cb)(void);


//-------------------------------------------------------------------
// Driver Import functions
//-------------------------------------------------------------------
/* Platform functions */
#define DRV_NAME "AIC8800D"
#define DEVICE_FUNC_CODE    0x07

/* SDIO device ID — a single entry, and VID-level matching only (no changes to the SoC sdio.c).
 * The RT-Thread SDIO framework treats rt_sdio_driver.id as a "single ID" (sdio_match_driver()
 * only passes id itself to sdio_match_card(), it does not iterate an array), so extra entries
 * have no effect.
 * The SoC sdio_match_card() compares only the function-level product: this card's function 1
 * CIS does not contain CISTPL_MANFID (measured f1prod=0), so filling product with 0 here matches;
 * SDIO_ANY_PROD_ID must not be used — the SoC implementation requires `id->product != SDIO_ANY_PROD_ID`,
 * so ANY (0xffff) would never match.
 * The exact model is still determined at runtime by aicwf_sdio_chipmatch() from card->cis.product. */
static struct sdio_device_id aic_sdio_ids[] =
{
#if defined(CONFIG_AIC8800D80) || defined(CONFIG_AIC8800D80N)
    /* D80 series: D80 / D80N / D80LN / D80WN / D40N / D40LN / D40WN / D80X2 */
    {DEVICE_FUNC_CODE, SDIO_VENDOR_ID_AIC8800D80, 0},
#elif defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW)
    {DEVICE_FUNC_CODE, SDIO_VENDOR_ID_AIC8800DC,  0},
#elif defined(CONFIG_AIC8800D)
    {DEVICE_FUNC_CODE, SDIO_VENDOR_ID_AIC8801,    0},
#endif
};

// [PLATFORM] sdio_driver_t is defined by the platform SDIO subsystem
// TODO: [AIC8800-PORT] confirm that platform struct sdio_driver_t members (.probe/.remove/.name/.id) have matching names
struct sdio_drv_priv
{
    struct sdio_driver_t wifi_drv;
    int drv_registered;
};
// [PLATFORM] mmcsd_card is defined by the platform MMC/SD subsystem
// TODO: [AIC8800-PORT] replace with the platform's MMC/SD card abstraction type
static rt_int32_t __aicwf_aic_drv_init(struct mmcsd_card *card);
static rt_int32_t __aicwf_aic_dev_remove(struct mmcsd_card *card);

/* Runtime chipmatch forward declaration — defined at the end of the file, so it must be visible
 * before __aicwf_aic_drv_init. Unlike the source project (ESP-IDF), which reads VID/PID via the FBR,
 * the current platform (TI mmcsd_core) already parses the CIS during probe and fills in
 * card->cis.manufacturer/product, so they can be used directly. */
static int aicwf_sdio_chipmatch(uint16_t vid, uint16_t pid);

static struct sdio_drv_priv sdio_drvpriv =
{
    .wifi_drv.probe = __aicwf_aic_drv_init,
    .wifi_drv.remove = __aicwf_aic_dev_remove,
    .wifi_drv.name = (char *)DRV_NAME,
    .wifi_drv.id = aic_sdio_ids,
};

/**
 * @brief SDIO driver probe callback: initialize when an SDIO card is inserted
 *
 * Records information for each SDIO function and decides, according to configuration,
 * whether to enable SDIO interrupt mode.
 *
 * @param card MMC/SDIO card structure
 * @return 0=success
 */

static rt_int32_t __aicwf_aic_drv_init(struct mmcsd_card *card)
{
    uint8_t i, num, func_idx;

    DBG_SDIO_VRB("====>__aicwf_aic_drv_init():sdio_version:%d,manufacturer:0x%04x,product:0x%04x\r\n", card->cccr.sdio_version, card->cis.manufacturer, card->cis.product);

    // initialize the local function list to NULL
    for (i = 0; i < SDIOM_MAX_FUNCS; i++)
        sdio_function_local[i] = NULL;

    /* RT-Thread: func0 is at sdio_function[0], I/O functions at [1..sdio_function_num];
     * sdio_function_num is the number of I/O functions (excluding func0), and the array index
     * is the function number, so valid indices are 0..sdio_function_num — collect func0 and all I/O functions. */
    num = card->sdio_function_num;
    if (num > SDIOM_MAX_FUNCS - 1)
        num = SDIOM_MAX_FUNCS - 1;
    // map by func->num: unmatched slots stay NULL
    for (i = 0; i <= num; i++)
    {
        struct sdio_func *f = card->sdio_function[i];
        if (f == NULL)
            continue;
        func_idx = f->num;
        if (func_idx >= SDIOM_MAX_FUNCS)
        {
            DBG_SDIO_ERR("__aicwf_aic_drv_init(): func->num=%d out of range, skip\n", func_idx);
            continue;
        }
        sdio_function_local[func_idx] = f;
        DBG_SDIO_INF("[%d]:%x %x\n", f->num, f->manufacturer, f->product);
        DBG_SDIO_INF("__aicwf_aic_drv_init():func:%d,maxblksize:%d,curblksize:%d\r\n", f->num, f->max_blk_size, f->cur_blk_size);
    }

    /* SDIO IRQ is a standard capability; always declare support to the platform MMC/SDIO core;
     * runtime enabling/disabling is controlled by aicwf_sdio_isr(), decoupled from "whether to poll". */
    card->host->flags |= MMCSD_SUP_SDIO_IRQ;

    /* Runtime chipmatch — the platform already parsed the CIS during probe; use card->cis.manufacturer/product directly.
     * The source project (ESP-IDF) uses aicwf_sdio_read_fbr_vid_pid() to read the raw CIS; the current platform (TI mmcsd_core)
     * has already parsed it, so no need to re-read the FBR. The match result is stored in g_runtime_chipid for wifi_if.c to prefer. */
    {
        uint16_t vid = (uint16_t)card->cis.manufacturer;
        uint16_t pid = (uint16_t)card->cis.product;
        int chipid = aicwf_sdio_chipmatch(vid, pid);
        g_runtime_vid = vid;
        g_runtime_pid = pid;
        /* chipmatch returns -1 on failure; AIC8801's valid match value is 0, so 0 cannot mean "no match" */
        g_runtime_chipid = (chipid < 0) ? AICWF_CHIPID_UNMATCHED : (uint16_t)chipid;
        DBG_SDIO_INF("[chipmatch] VID=0x%04X, PID=0x%04X -> chipid=%d\n", vid, pid, g_runtime_chipid);
    }

    return 0;
}

/**
 * @brief SDIO driver remove callback: clean up when the SDIO card is removed
 * @param card MMC/SDIO card structure
 * @return 0=success
 */
static rt_int32_t __aicwf_aic_dev_remove(struct mmcsd_card *card)
{
    DBG_SDIO_INF("====>__aicwf_aic_dev_remove()\n");
    return 0;
}

/**
 * @brief Register the AIC SDIO driver with the SDIO subsystem
 * @return registration result, 0=success
 */
int aicwf_sdio_drv_register(void)
{
    int err;
    err = aic_sdio_register_driver(&sdio_drvpriv.wifi_drv);

    DBG_SDIO_INF("====>aicwf_sdio_drv_register():ret:%d\n", err);

    return err;
}

void aicwf_sdio_claim_host(uint32_t func_num);
void aicwf_sdio_release_host(uint32_t func_num);

/**
 * @brief Enable the specified SDIO function (internal implementation)
 * @param index SDIO function index
 * @return 0=success, -1=failure
 */
static int __aicwf_sdio_enable_function(uint8_t index)
{
    int ret = 0;

    aicwf_sdio_claim_host(index);
    ret = aic_sdio_enable_func(sdio_function_local[index]);
    aicwf_sdio_release_host(index);
    if (ret != 0)
    {
        DBG_SDIO_ERR("aic_sdio_enable_func fail:%d!!\n", ret);
        return -1;
    }

    return 0;
}

/**
 * @brief Enable/disable the MMC SDIO interrupt
 * @param enable 1=enable, 0=disable
 */
// [PLATFORM] func->card->host->ops->enable_sdio_irq is a deep access into the platform MMC layer
// TODO: [AIC8800-PORT] replace with the platform-provided SDIO interrupt enable/disable API
static void aicwf_mmc_enable_isr(int32_t enable)
{
    struct sdio_func *func = NULL;
    uint8_t i;

    // find the first valid function to access card->host
    for (i = 0; i < SDIOM_MAX_FUNCS; i++)
    {
        if (sdio_function_local[i] != NULL)
        {
            func = sdio_function_local[i];
            break;
        }
    }

    if (!func)
        return;

    func->card->host->ops->enable_sdio_irq(func->card->host, enable);
}


//-------------------------------------------------------------------
// Driver Macros Functions
//-------------------------------------------------------------------
#define SDIO_PRE_OP() {\
    if(!(wifi_chip_status_get()))\
        return -1;\
}

#define SDIO_PRE_OP_B() {\
    if(!(wifi_chip_status_get()))\
        return false;\
}

#define SDIO_PRE_OPV(v) {\
    if(!(wifi_chip_status_get()))\
    {\
        *v=-1;\
        return;\
    }\
}

#define SDIO_PRE_OPVR(v) {\
    if(!(wifi_chip_status_get()))\
    {\
        *v=-1;\
        return 1;\
    }\
}

#define SDIO_WR_FAIL(ret){\
    if(ret) {\
        sdio_dev.wr_fail++;\
    }\
    else\
        sdio_dev.wr_fail = 0;\
    if(sdio_dev.wr_fail > 15)\
    {\
        aicwf_driver_unexcepted(0x04);\
    }\
}


#define SDIO_ANY_ID (~0)


//-------------------------------------------------------------------
// Driver functions define
//-------------------------------------------------------------------
/* ==========================================================================
 * Platform SDIO I/O Wrappers — implemented by the platform
 *
 * The aic_-prefixed functions below wrap the platform-specific low-level SDIO operations.
 * When porting to a new RTOS/SDK platform, replace the function bodies with the corresponding
 * platform SDIO API calls.
 *
 * The current implementation passes straight through to the platform SDK's sdio_* APIs and
 * serves only as a reference example.
 * ========================================================================== */

static inline int aic_sdio_io_rw_extended_block(struct sdio_func *func,
        int write, unsigned int addr, int incr_addr, uint8_t *buf, int blk_cnt)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO CMD53 block read/write API
    return sdio_io_rw_extended_block(func, write, addr, incr_addr, buf, blk_cnt);
}

static inline uint8_t aic_sdio_io_readb(struct sdio_func *func,
                                        unsigned int addr, int *err_ret)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO CMD52 byte read API
    /* [AIC8800-PORT][SiFli] The SDK's sdio_io_readb err parameter is rt_int32_t* (=long*),
     * while err_ret is int*; both are the same width on ARM32 (4B), so an explicit cast removes the GCC14 incompatible pointer error */
    return sdio_io_readb(func, addr, (rt_int32_t *)err_ret);
}

static inline int aic_sdio_io_writeb(struct sdio_func *func,
                                     unsigned int addr, uint8_t val)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO CMD52 byte write API
    return sdio_io_writeb(func, addr, val);
}

static inline int aic_sdio_register_driver(struct sdio_driver_t *drv)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO driver registration API
    return sdio_register_driver(drv);
}

static inline int aic_sdio_enable_func(struct sdio_func *func)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO function enable API
    return sdio_enable_func(func);
}

static inline int aic_sdio_attach_irq(struct sdio_func *func,
                                      sdio_irq_h_t handler)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO interrupt registration API
    return sdio_attach_irq(func, handler);
}

static inline int aic_sdio_detach_irq(struct sdio_func *func)
{
    // TODO: [AIC8800-PORT] replace with the platform SDIO interrupt release API
    return sdio_detach_irq(func);
}

/* AIC driver sdio base functions */
// [PLATFORM] the sdio_irq_h_t type is defined by the platform SDIO subsystem
// TODO: [AIC8800-PORT] if the platform headers already provide this typedef, this line can be removed
// Currently provided as a fallback definition; depends on platform struct sdio_function
/* [AIC8800-PORT][SiFli] fallback typedef disabled: sdio_irq_h_t is already provided by the compatibility macro
 * = rt_sdio_irq_handler_t at the top (from the SDK's mmcsd_card.h); re-typedef'ing it would conflict with the SDK */
// typedef void (sdio_irq_h_t)(struct sdio_function *);
/**
 * @brief Lock the SDIO host (bus mutex)
 * @param func_num SDIO function number (currently unused)
 */
void aicwf_sdio_claim_host(uint32_t func_num)
{
    (void)func_num;
    //aic sdio wr use onwer thread lock .
    //mmcsd_host_lock(func->card->host);
}

/**
 * @brief Unlock the SDIO host (bus mutex)
 * @param func_num SDIO function number (currently unused)
 */
void aicwf_sdio_release_host(uint32_t func_num)
{
    (void)func_num;
    //mmcsd_host_unlock(func->card->host);
}

/**
 * @brief Register the SDIO interrupt handler
 * @param func    SDIO function
 * @param handler interrupt handler
 * @return return value of aic_sdio_attach_irq()
 */
static int32_t aicwf_sdio_claim_irq(struct sdio_func *func,
                                    sdio_irq_h_t handler)
{
    return aic_sdio_attach_irq(func, handler);
}

/**
 * @brief Release the SDIO interrupt
 * @param func SDIO function
 * @return return value of aic_sdio_detach_irq()
 */
static int32_t aicwf_sdio_release_irq(struct sdio_func *func)
{
    return aic_sdio_detach_irq(func);
}

/**
 * @brief SDIO bulk read (CMD53 extended block read)
 * @param func_num SDIO function number
 * @param dst      destination buffer
 * @param addr     register address
 * @param count    number of bytes to read
 * @return 0=success, non-zero=failure
 */
int aicwf_sdio_readsb(uint32_t func_num, void *dst, unsigned int addr, int count)
{
    int ret;
    struct sdio_func *func = sdio_function_local[func_num];

    SDIO_PRE_OP();

#ifdef CONFIG_SDIO_ADMA
    /* [PLATFORM] The ADMA read branch requires a platform implementation: single-buffer DMA read + cache invalidate after the read
     * (cache lines written by DMA must be invalidated before the CPU reads them). See the ADMA contract
     * in port/inc/sdio_port.h and aidoc/adma-platform-porting-reference.md.
     * This reference platform does not implement it — block it explicitly, so that count under ADMA
     * semantics is not treated as a byte count and routed to a normal block read, silently corrupting data. */
#error "CONFIG_SDIO_ADMA: implement aicwf_sdio_readsb() ADMA branch for this platform (see port/inc/sdio_port.h ADMA contract)"
#endif

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    ret = aic_sdio_io_rw_extended_block(func, 0, addr, 0, (uint8_t *)dst, count);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    if (ret)
    {
        DBG_SDIO_INF("%s: %d\n", __func__, ret);
    }
    SDIO_WR_FAIL(ret);
    return ret;
}

/**
 * @brief SDIO bulk write (CMD53 extended block write)
 * @param func_num SDIO function number
 * @param addr     register address
 * @param src      source data buffer
 * @param count    number of bytes to write
 * @return 0=success, non-zero=failure
 */
int aicwf_sdio_writesb(uint32_t func_num, unsigned int addr, void *src, int count)
{
    int ret;
    struct sdio_func *func = sdio_function_local[func_num];
    SDIO_PRE_OP();

#ifdef CONFIG_SDIO_ADMA
    /* [PLATFORM] The ADMA write branch requires a platform implementation: src = const sdio_sg_list *,
     * count = number of segments → hand over to the platform DMA/descriptor mechanism (e.g. ASR/064:
     * NU_SDIODrv_Write_Memory_sg(addr, sg_list, segcnt)), cache clean + sync barrier per segment,
     * and use the equivalent of PROTECT_TX_ADMA to keep the buffer from being reused before TX completes.
     * See the ADMA contract in port/inc/sdio_port.h and aidoc/adma-platform-porting-reference.md. */
#error "CONFIG_SDIO_ADMA: implement aicwf_sdio_writesb() ADMA branch for this platform (see port/inc/sdio_port.h ADMA contract)"
#endif

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    ret = aic_sdio_io_rw_extended_block(func, 1, addr, 0, (uint8_t *)src, count);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    if (ret)
    {
        DBG_SDIO_INF("%s: %d\n", __func__, ret);
    }
    SDIO_WR_FAIL(ret);
    return ret;
}

/**
 * @brief SDIO bulk message write (CMD53 extended block write, message mode)
 *
 * Same as aicwf_sdio_writesb(); used for message frame transfer.
 *
 * @param func_num SDIO function number
 * @param addr     register address
 * @param src      source data buffer
 * @param count    number of bytes to write
 * @return 0=success, non-zero=failure
 */
int aicwf_sdio_writesb_msg(uint32_t func_num, unsigned int addr, void *src, int count)
{
    int ret;
    struct sdio_func *func = sdio_function_local[func_num];
    SDIO_PRE_OP();

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    ret = aic_sdio_io_rw_extended_block(func, 1, addr, 0, (uint8_t *)src, count);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    if (ret)
    {
        DBG_SDIO_INF("%s: %d\n", __func__, ret);
    }
    SDIO_WR_FAIL(ret);
    return ret;
}

/**
 * @brief Get an SDIO func handle usable for FUNC0 operations
 *
 * Most platform SDIO function lists do not include FUNC0, so sdio_function_local[FUNC_0]
 * is usually NULL. However, CMD52 read/write to FUNC0 (CCCR) only needs access to the
 * card->host bus; the ->card->host of any valid func can be used for accesses to the FUNC0
 * address space.
 *
 * @return the first available struct sdio_func *, NULL=none available
 */
static struct sdio_func *__aicwf_sdio_get_func0(void)
{
    uint8_t i;

    // prefer FUNC0
    if (sdio_function_local[FUNC_0] != NULL)
        return sdio_function_local[FUNC_0];

    // fallback: use any valid func to access card->host
    for (i = 1; i < SDIOM_MAX_FUNCS; i++)
    {
        if (sdio_function_local[i] != NULL)
            return sdio_function_local[i];
    }

    return NULL;
}

/**
 * @brief SDIO Function 0 single-byte read
 * @param addr    register address
 * @param err_ret output error code (may be NULL)
 * @return the byte value read
 */
static uint8_t __aicwf_sdio_f0_readb(unsigned int addr, int *err_ret)
{
    int ret;
    uint8_t b = 0;
    // a CMD52 to the FUNC0 address space can be issued through any func's card->host
    struct sdio_func *func = __aicwf_sdio_get_func0();

    if (!func)
    {
        if (err_ret)
            *err_ret = -1;
        return 0;
    }

    SDIO_PRE_OPVR(err_ret);

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    b = aic_sdio_io_readb(func, addr, &ret);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    if (err_ret)
        *err_ret = ret;

    if (ret)
    {
        DBG_SDIO_INF("%s: %d\n", __func__, ret);
    }
    SDIO_WR_FAIL(ret);
    return b;
}

/**
 * @brief SDIO Function 0 single-byte write
 * @param b       byte value to write
 * @param addr    register address
 * @param err_ret output error code (may be NULL)
 */
static void aicwf_sdio_f0_writeb(unsigned char b, unsigned int addr,
                                 int *err_ret)
{
    int ret;
    // a CMD52 to the FUNC0 address space can be issued through any func's card->host
    struct sdio_func *func = __aicwf_sdio_get_func0();

    if (!func)
    {
        if (err_ret)
            *err_ret = -1;
        return;
    }

    SDIO_PRE_OPV(err_ret);

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    ret = aic_sdio_io_writeb(func, addr, b);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    if (err_ret)
        *err_ret = ret;

    if (ret)
    {
        DBG_SDIO_INF("%s: %d\n", __func__, ret);
    }
    SDIO_WR_FAIL(ret);
}

/**
 * @brief Find an SDIO function by vendor ID and device ID
 * @param class  SDIO function class code
 * @param vendor vendor ID (SDIO_ANY_ID means match any)
 * @param device device ID (SDIO_ANY_ID means match any)
 * @return pointer to the matching SDIO function, NULL=not found
 */
// [PLATFORM] func->func_code/manufacturer/product/num are defined by the platform struct sdio_func
// TODO: [AIC8800-PORT] verify the platform struct sdio_func member names and adapt if necessary
static struct sdio_func *__aicwf_sdio_find_func(uint8_t class, uint16_t vendor, uint16_t device)
{
    struct sdio_func *func;
    uint8_t j;


    for (j = 0; j < SDIOM_MAX_FUNCS; j++)
    {
        func = sdio_function_local[j];
        if (func == NULL)
            continue;
        DBG_SDIO_VRB("sdio_func[%d]: class:0x%x, vendor:0x%x, device:0x%x\r\n",
                     j, func->func_code, func->manufacturer, func->product);
        //if (class != (uint8_t)SDIO_ANY_ID && class != func->class)
        //    continue;
        if ((vendor != (uint16_t)SDIO_ANY_ID) && (vendor != func->manufacturer))
            continue;
        if ((device != (uint16_t)SDIO_ANY_ID) && (device != func->product))
            continue;

        DBG_SDIO_INF("sdio find func num:%d\r\n", func->num);
        return func;
    }

    return NULL;
}

/**
 * @brief SDIO CMD52 single-byte read (generic, by function number)
 *
 * Uniform wrapper for CMD52 reads on Function 1/2; func_num is not 0.
 *
 * @param func_num SDIO function number (FUNC_1 or FUNC_2)
 * @param addr     register address
 * @param data     output data byte
 * @return true=success, false=failure
 */
bool aicwf_sdio_readb_cmd52_by_num(uint32_t func_num, uint32_t addr, uint8_t *data)
{
    int err;
    uint8_t b;
    struct sdio_func *func = sdio_function_local[func_num];
    SDIO_PRE_OP_B();

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    b = aic_sdio_io_readb(func, addr, &err);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    SDIO_WR_FAIL(err);
    if (err)
    {
        DBG_SDIO_ERR("__aicwf_sdio_readb_cmd52 fail %d!\n", err);
        return false;
    }
    *data = b;
    return true;
}

/**
 * @brief SDIO CMD52 single-byte write (generic, by function number)
 *
 * Uniform wrapper for CMD52 writes on Function 1/2; func_num is not 0.
 *
 * @param func_num SDIO function number (FUNC_1 or FUNC_2)
 * @param addr     register address
 * @param data     data byte to write
 * @return true=success, false=failure
 */
bool aicwf_sdio_writeb_cmd52_by_num(uint32_t func_num, uint32_t addr, uint8_t data)
{
    int err;
    struct sdio_func *func = sdio_function_local[func_num];
    SDIO_PRE_OP_B();

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    err = aic_sdio_io_writeb(func, addr, data);
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    SDIO_WR_FAIL(err);
    if (err)
    {
        DBG_SDIO_ERR("__aicwf_sdio_writeb_cmd52 fail %d!\n", err);
        return false;
    }
    return true;
}

/**
 * @brief SDIO CMD52 single-byte read (Function 1)
 * @param addr register address
 * @param data output data byte
 * @return true=success, false=failure
 */
bool aicwf_sdio_readb_cmd52(uint32_t addr, uint8_t *data)
{
    return aicwf_sdio_readb_cmd52_by_num(FUNC_1, addr, data);
}

/**
 * @brief SDIO CMD52 single-byte read (Function 2)
 * @param addr register address
 * @param data output data byte
 * @return true=success, false=failure
 */
bool aicwf_sdio_readb_cmd52_func2(uint32_t addr, uint8_t *data)
{
    return aicwf_sdio_readb_cmd52_by_num(FUNC_2, addr, data);
}

/**
 * @brief SDIO CMD52 single-byte write (Function 1)
 * @param addr register address
 * @param data data byte to write
 * @return true=success, false=failure
 */
bool aicwf_sdio_writeb_cmd52(uint32_t addr, uint8_t data)
{
    return aicwf_sdio_writeb_cmd52_by_num(FUNC_1, addr, data);
}

/**
 * @brief SDIO CMD52 single-byte write (Function 2)
 * @param addr register address
 * @param data data byte to write
 * @return true=success, false=failure
 */
bool aicwf_sdio_writeb_cmd52_func2(uint32_t addr, uint8_t data)
{
    return aicwf_sdio_writeb_cmd52_by_num(FUNC_2, addr, data);
}

#ifdef CONFIG_OOB
// This interface is from ASR1803-SDK.
void aicwf_sdio_set_irq_handler(void (*cb)(void));
static void aicwf__sdio_irq_enable(UINT32 base_addr, UINT16 mask)
{

}

static void aicwf_sdio_oobirq_task(void *argv)
{
    int ret = 0;
    uint32_t wait_time = 1000;
    while (1)
    {
        ret = aicwf_rtos_semaphore_wait(sdio_dev.sdio_oobirq_sema, wait_time);
        if (ret)
            DBG_SDIO_WRN("sdio_oobirq_sema timeout: %d\n", ret);
        aic_sdio_rx_task(0);
    }
}

static void aicwf_sdio_oob_irq_hdl(void)
{
    int count = 0;
    /* [AIC8800-PORT][BUGFIX] Hard interrupt context; use the fromisr variant to read the count
     * (get_count enters a critical section, which is not allowed in an ISR) */
    count = aicwf_rtos_semaphore_get_count_fromisr(sdio_dev.sdio_oobirq_sema);
    if (count == 0)
        aicwf_rtos_semaphore_signal(sdio_dev.sdio_oobirq_sema, true);
    else
        DBG_SDIO_INF("sdio oob isr: %d\n", count);
}

void aicwf_sdio_oob_enable(void)
{
    int ret = 0;

    //configure oob gpio
    //to add

    aicwf_sdio_set_irq_handler(NULL);

    //disable sdio interrupt
    DBG_SDIO_INF("%s: SDIOWIFI_INTR_CONFIG_REG Disable\n", __func__);
    ret = aicwf_sdio_writeb_cmd52_func2(sdiodev->sdio_reg.intr_config_reg, 0x0);
    if (ret == false)
    {
        DBG_SDIO_ERR("ERR: reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
    }

    ret = aicwf_rtos_task_create(aicwf_sdio_oobirq_task, "oobirq_task", SDIO_OOBIRQ_TASK,
                                 sdio_oobirq_stack_size, NULL, sdio_oobirq_priority, &sdiodev->sdio_oobirq_hdl);
    if (ret)
    {
        DBG_SDIO_ERR("ERR: oobirq_task create failed\n");
    }
    else
    {
        sdiodev->oob_enable = true;
    }
}
#endif /* CONFIG_OOB */

#ifndef CONFIG_OOB
void aicwf_sdio_oob_enable(void) { }
#endif

/**
 * @brief Low-level SDIO interrupt callback (from the MMC/SDIO layer)
 *
 * Forwards the interrupt to the registered host_sdio_irq_cb handler.
 *
 * @param func SDIO function that triggered the interrupt
 */
// [PLATFORM] struct sdio_function is defined by the platform SDIO subsystem
// TODO: [AIC8800-PORT] replace with the platform SDIO interrupt callback signature
static void aicwf_host_sdio_interrupt(struct sdio_func *func)
{
    if (host_sdio_irq_cb)
        host_sdio_irq_cb();
}

/**
 * @brief Set the SDIO interrupt handling callback
 * @param cb interrupt callback function pointer
 */
void aicwf_sdio_set_irq_handler(void (*cb)(void))
{
    host_sdio_irq_cb = cb;
}

/**
 * @brief Enable/disable the SDIO interrupt
 * @param en 1=enable, 0=disable
 */
void aicwf_sdio_isr(uint8_t en)
{
    /* SDIO(DAT1) interrupt storm avoidance: see the g_sdio_irq_mask_forced description in this file.
     * Polling mode does not release it either; packet reception falls back to the RX task's 10ms timeout polling. */
    if (en && (g_sdio_irq_mask_forced || aicwifi_sdio_rxdata_mode))
    {
        aicwf_mmc_enable_isr(0);
        return;
    }
    aicwf_mmc_enable_isr(en);
}

/**
 * @brief SDIO host interrupt enable control
 *
 * When enabling, only the MMC interrupt switch is controlled; when disabling, all registered
 * SDIO interrupts are released.
 *
 * @param enable true=enable, false=disable
 * @return true
 */
bool aicwf_sdio_host_enable_isr(bool enable)
{
    /* SDIO(DAT1) interrupt storm avoidance: the library's aicwf_sdio_rx_task unconditionally calls
     * enable=TRUE at the end of every round; with a level interrupt this immediately re-enters the ISR and forms a storm;
     * here it is masked, and packet reception falls back to 10ms polling with aicwifi_sdio_rxdata_mode=1. */
    if (enable && (g_sdio_irq_mask_forced || aicwifi_sdio_rxdata_mode))
    {
        aicwf_mmc_enable_isr(0);
        return true;
    }
    aicwf_mmc_enable_isr(enable);

    if (!enable)
    {
        aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
        aicwf_sdio_claim_host(FUNC_1);
        aicwf_sdio_release_irq(sdio_function_local[FUNC_1]);
        aicwf_sdio_release_host(FUNC_1);

#if (defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW))
        aicwf_sdio_claim_host(FUNC_2);
        aicwf_sdio_release_irq(sdio_function_local[FUNC_2]);
        aicwf_sdio_release_host(FUNC_2);
#endif
        aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);
    }
    return true;
}

/**
 * @brief SDIO interrupt handling (internal implementation)
 *
 * In non-OOB mode, wakes the RX task through a semaphore to process received data.
 * In OOB mode, it is handled independently by the OOB task.
 */
static void aicwf_sdio_irq_hdl(void)
{
#ifdef CONFIG_OOB
    if (sdio_dev.oob_enable == false)
#endif
    {
        //sdio_host_enable_isr(false);
        if (!config_rxtask_insdio)
        {
            aicwf_plat_gpio_trigger(DRV_TRG_SDIO, 1);
            if (config_irq_sus)
                aicwf_sdio_isr(false);
            //aicwf_rtos_semaphore_signal(sdio_dev.sdio_rx_sema, false);
            aicwf_rtos_semaphore_signal(sdio_dev.sdio_rx_sema, true);
            aicwf_plat_gpio_trigger(DRV_TRG_SDIO, 0);
        }
        else
        {
            //aicwf_sdio_isr(false);
            aic_sdio_rx_task(0);
            //aicwf_sdio_isr(true);
        }
    }
}

/**
 * @brief Initialize SDIO interrupts
 *
 * Registers interrupt handlers for Function 1 (and Function 2 for DC/DW),
 * and disables message-confirmation polling mode.
 *
 * @return 0=success
 */
int aicwf_sdio_interrupt_init(void)
{
    aicwf_sdio_set_irq_handler(aicwf_sdio_irq_hdl);

    aicwf_rtos_mutex_lock(sdio_dev.bus_txrx, -1);
    if (aicwifi_sdio_rxdata_mode)
    {
        /* Polling mode: do not register the SDIO interrupt (do not write CCCR INT_EN), otherwise the card
         * keeps pulling DAT1 low; packet reception is done entirely by the RX task's 10ms timeout polling. */
        DBG_SDIO_INF("%s: polling mode, skip sdio irq claim\n", __func__);
    }
    else
    {
        aicwf_sdio_claim_host(FUNC_1);
        aicwf_sdio_claim_irq(sdio_function_local[FUNC_1], aicwf_host_sdio_interrupt);
        aicwf_sdio_release_host(FUNC_1);
    }

#if (defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW))
    aicwf_sdio_claim_host(FUNC_2);
    aicwf_sdio_claim_irq(sdio_function_local[FUNC_2], aicwf_host_sdio_interrupt);
    aicwf_sdio_release_host(FUNC_2);
#endif
    aicwf_rtos_mutex_unlock(sdio_dev.bus_txrx);

    msgcfm_poll_en = 0;
    return 0;
}

/**
 * @brief Set the SDIO function block size (internal implementation)
 *
 * Sets the block size through the target function's FBR register and reads it back for verification.
 *
 * @param index   index of the target function in sdio_function_local[] (FUNC_1 or FUNC_2)
 * @param addr    FBR block size register address
 * @param blksize target block size (defaults to SDIOWIFI_FUNC_BLOCKSIZE when 0 or >512)
 * @return true=success, false=failure
 */
static bool __aicwf_sdio_set_block_size(uint8_t index, unsigned int addr, unsigned int blksize)
{
    unsigned char blk[2];
    int err = 0;

    struct sdio_func *func = sdio_function_local[index];
    if (!func)
    {
        DBG_SDIO_ERR("%s: sdio_function_local[%d] is NULL\n", __func__, index);
        return false;
    }

    DBG_SDIO_INF("%s: fn=%d, addr=%d, blksize=%d\n", __func__, func->num, addr, blksize);

    if ((blksize == 0) || (blksize > 512))
    {
        blksize = SDIOWIFI_FUNC_BLOCKSIZE;
    }

    /* The block size register is in the Function 0 (CCCR) address space; access it via the F0 read/write interface */
    blk[0] = blksize & 0x0ff;
    blk[1] = (blksize >> 8) & 0x0ff;

    aicwf_sdio_f0_writeb(blk[0], addr, &err);
    if (err)
    {
        DBG_SDIO_ERR("aic_sdio_set_block_size fail 0!\n");
        return false;
    }

    aicwf_sdio_f0_writeb(blk[1], addr + 1, &err);
    if (err)
    {
        DBG_SDIO_ERR("aic_sdio_set_block_size fail 1!\n");
        return false;
    }

    blk[0] = __aicwf_sdio_f0_readb(addr, &err);
    if (err)
    {
        DBG_SDIO_ERR("sdio_readb 0x%d fail 0!\n", addr);
        return false;
    }

    blk[1] = __aicwf_sdio_f0_readb(addr + 1, &err);
    if (err)
    {
        DBG_SDIO_ERR("sdio_readb 0x%x fail 1!\n", addr + 1);
        return false;
    }

    if (((unsigned int)(blk[1] << 8) | blk[0]) != blksize)
    {
        DBG_SDIO_ERR("%s: fail, target=%d, readback=%d!\n", __func__, blksize, ((unsigned int)(blk[1] << 8) | blk[0]));
        return false;
    }

    func->cur_blk_size = blksize;

    return true;
}

/**
 * @brief Release SDIO Function 2 (disable interrupts and release the IRQ)
 */
void aicwf_sdio_release_func2(void)
{
    int ret = 0;

    /* DL/DL_H disable release func2 via CONFIG_NO_SDIO_FUNC_RELEASE:
     * aicwf_sdio_func_init already calls aicwf_hal_func_release_set(0) at the end,
     * so the runtime check here makes that macro take effect (DL reuses the DC library and its
     * runtime chipid is also DC, so chipid alone cannot distinguish them; the runtime switch must decide) */
    if (!aicwf_hal_func_release_get())
    {
        DBG_SDIO_INF("%s: release func2 disabled, skip.\n", __func__);
        return;
    }

    /* flag_tx_set(false) is bound to release: after func2 is released, TX data goes through FUNC_1;
     * keep it together with release so the runtime switch above controls both uniformly */
    aicwf_hal_flag_tx_set(false);

    DBG_SDIO_INF("%s\n", __func__);
    aicwf_sdio_claim_host(FUNC_2);
    ret = aicwf_sdio_writeb_cmd52_func2(sdio_dev.sdio_reg.intr_config_reg, 0x0);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdio_dev.sdio_reg.intr_config_reg);
    }

    aicwf_sdio_release_irq(sdio_function_local[FUNC_2]);
    aicwf_sdio_release_host(FUNC_2);
}

/**
 * @brief SDIO function initialization (standard mode)
 *
 * Initializes Function 1 and Function 2: enable the functions, set the block size,
 * disable byte mode, enable SDIO interrupts.
 *
 * @param chipid  chip product ID
 * @param sdiodev SDIO device structure
 * @return 0=success, negative=failure
 */
int aicwf_sdio_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev)
{
    int32_t ret = 0;
    uint8_t block_bit0 = 0x1;
    uint8_t byte_mode_disable = 0x1;//1: no byte mode
    struct sdio_func *func;

    /* SDIO Function 1 */
    //aicwf_sdio_claim_host(sdio_function[1]);
    if (chipid == PRODUCT_ID_AIC8801)
    {
        /* Enable Function 0 */
        func = __aicwf_sdio_find_func(0x7, SDIO_VENDOR_ID_AIC8801, SDIO_DEVICE_ID_AIC8801);
        if (!func)
        {
            ret = -ENODEV;
            DBG_SDIO_ERR("%s: __aicwf_sdio_find_func func0 failed!!(%d)\n", __func__, ret);
            return ret;
        }

    }
    else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        /* Enable Function 0 */
        func = __aicwf_sdio_find_func(0x7, SDIO_VENDOR_ID_AIC8800DC, SDIO_DEVICE_ID_AIC8800DC);
        if (!func)
        {
            ret = -ENODEV;
            DBG_SDIO_ERR("%s: __aicwf_sdio_find_func func0 failed!!(%d)\n", __func__, ret);
            return ret;
        }
    }
    ret = __aicwf_sdio_enable_function(sdio_function_local[FUNC_1]->num);
    //aicwf_sdio_release_host(sdio_function_local[1]);

    ret = __aicwf_sdio_set_block_size(FUNC_1, sdiodev->sdio_reg.fun1_blk_size_reg, SDIOWIFI_FUNC_BLOCKSIZE);
    if (ret == false)
    {
        ret = -EREMOTEIO;
        DBG_SDIO_ERR("%s: blksize set failed\n", __func__);
        return ret;
    }

    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.register_block, block_bit0);
    if (ret == false)
    {
        ret = -EREMOTEIO;
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.register_block);
        return ret;
    }

    //1: no byte mode
    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.bytemode_enable_reg, byte_mode_disable);
    if (ret == false)
    {
        ret = -EREMOTEIO;
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
        return ret;
    }

    /* Chip-side interrupt enable must stay at 0x07: writing 0 makes the chip close the SDIO
     * path after "Start app", after which even CMD52 is no longer answered (measured: all timeouts after CALIB exec).
     * Polling mode only disables host-side interrupts (no claim_irq) and must not touch this register. */
    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.intr_config_reg, 0x07);
    if (ret == false)
    {
        ret = -EREMOTEIO;
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
        return ret;
    }
    DBG_SDIO_INF("aicwf_sdio_host_init:enable fun1 ok!\n");

    /* SDIO Function 2 */
    //aicwf_sdio_claim_host(sdio_function[2]);
    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        /* Enable Function 2 */
        func = __aicwf_sdio_find_func(0x7, SDIO_VENDOR_ID_AIC8800DC, 0xc18d);
        if (!func)
        {
            ret = -ENODEV;
            DBG_SDIO_ERR("%s: __aicwf_sdio_find_func func2 failed!!(%d)\n", __func__, ret);
            return ret;
        }
    }
    ret = __aicwf_sdio_enable_function(sdio_function_local[FUNC_2]->num);
    //aicwf_sdio_release_host(sdio_function[2]);

    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        ret = __aicwf_sdio_set_block_size(FUNC_2, sdiodev->sdio_reg.fun2_blk_size_reg, SDIOWIFI_FUNC_BLOCKSIZE);
        if (ret == false)
        {
            ret = -EREMOTEIO;
            DBG_SDIO_ERR("aicwf_sdio_host_init:func2: blksize set failed\n");
            return ret;
        }

        ret = aicwf_sdio_writeb_cmd52_func2(sdiodev->sdio_reg.register_block, block_bit0);
        if (ret < 0)
        {
            DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.register_block);
            return ret;
        }

        //1: no byte mode
        ret = aicwf_sdio_writeb_cmd52_func2(sdiodev->sdio_reg.bytemode_enable_reg, byte_mode_disable);
        if (ret < 0)
        {
            DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
            return ret;
        }

        //enable sdio interrupt
        ret = aicwf_sdio_writeb_cmd52_func2(sdiodev->sdio_reg.intr_config_reg, 0x7);
        if (ret < 0)
        {
            DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
            return ret;
        }
        DBG_SDIO_INF("%s:enable fun2 ok!\n", __func__);
    }

    /* Disable release of sdio function2 for DL models (runtime control; host library shared with DC) */
#ifdef CONFIG_NO_SDIO_FUNC_RELEASE
    aicwf_hal_func_release_set(0);
#endif

    return 0;
}

/**
 * @brief SDIO function initialization (SDIOv3 mode, D80 series)
 *
 * SDIOv3 initialization flow for D80 series chips:
 * enable Function 1, set the block size, configure the IO pad, disable byte mode, enable interrupts.
 *
 * @param chipid  chip product ID
 * @param sdiodev SDIO device structure
 * @return 0=success, negative=failure
 */
int aicwf_sdiov3_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev)
{
    int ret = 0;   /* [AIC8800-PORT][SiFli] was int32_t (=long); aicwf_sdio_f0_writeb takes int*, so int matches the function return type */
    uint8_t byte_mode_disable = 0x1;//1: no byte mode

    /* SDIO Function 1 */
    //aicwf_sdio_claim_host(sdio_function[1]);
    if (chipid == PRODUCT_ID_AIC8800D80 || chipid == PRODUCT_ID_AIC8800D80N)
    {
        /* On this platform the function CIS does not contain CISTPL_MANFID (func->manufacturer/product is always 0),
         * so function-level VID/PID cannot be used for matching; the model has already been confirmed by the
         * card-level CIS chipmatch during probe (chipid), hence no function-level lookup here — use FUNC_1 directly. */
    }
    ret = __aicwf_sdio_enable_function(sdio_function_local[FUNC_1]->num);
    if (ret)
    {
        DBG_SDIO_ERR("__aicwf_sdio_enable_function failed\n");
    }
    //aicwf_sdio_release_host(sdio_function[1]);

    ret = __aicwf_sdio_set_block_size(FUNC_1, sdiodev->sdio_reg.fun1_blk_size_reg, SDIOWIFI_FUNC_BLOCKSIZE);
    if (ret == false)
    {
        ret = -EREMOTEIO;
        DBG_SDIO_ERR("%s: blksize set failed\n", __func__);
        return ret;
    }
    DBG_SDIO_INF("sdio_host_init:sdio_set_block_size %d\n", SDIOWIFI_FUNC_BLOCKSIZE);

    aicwf_sdio_f0_writeb(0x7F, 0xF2, &ret);
    if (ret)
    {
        DBG_SDIO_ERR("set fn0 0xF2 fail %d\n", ret);
        return ret;
    }

#if 0 //set phase and sample
    {
        val |= SDIOCLK_FREE_RUNNING_BIT;
        aicwf_sdio_f0_writeb(val, 0xF0, &ret);
        if (ret)
        {
            DBG_SDIO_ERR("set iopad ctrl fail %d\n", ret);
            return ret;
        }
        aicwf_sdio_f0_writeb(0x0, 0xF8, &ret);
        if (ret)
        {
            DBG_SDIO_ERR("set iopad delay2 fail %d\n", ret);
            return ret;
        }
        aicwf_sdio_f0_writeb(0x20, 0xF1, &ret);
        if (ret)
        {
            DBG_SDIO_ERR("set iopad delay1 fail %d\n", ret);
            return ret;
        }
        aicwf_rtos_udelay(100);
    }
#endif

    //1: no byte mode
    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.bytemode_enable_reg, byte_mode_disable);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
        return ret;
    }

    /* Same as aicwf_sdio_func_init: chip-side interrupt enable must stay at 0x07; writing 0 makes the chip
     * stop answering CMD52 after "Start app". */
    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.intr_config_reg, 0x07);
    if (ret == false)
    {
        ret = -EREMOTEIO;
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
        return ret;
    }


#if 0
    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.wakeup_reg, 0x11);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.wakeup_reg);
        return ret;
    }

#if 1
    aicwf_rtos_udelay(5);
    ret = aicwf_sdio_readb_cmd52(sdiodev->sdio_reg.sleep_reg, &val1);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d read failed!\n", sdiodev->sdio_reg.sleep_reg);
        return ret;
    }

    if (!(val1 & 0x10))
    {
        DBG_SDIO_ERR("wakeup fail\n");
    }
    else
    {
        DBG_SDIO_INF("sdio ready\n");
    }
#endif
#endif

    return 0;
}

//-------------------------------------------------------------------
// Runtime chipmatch + interrupt enable after firmware load (ref. Linux aicsdio.c / source project sdio_port.c)
//-------------------------------------------------------------------

/**
 * @brief Match chipid against the hardware's actual VID/PID (ref. Linux aicsdio.c aicwf_sdio_chipmatch)
 *
 * Match order is the same as the Linux driver:
 *   - AIC8801:     VID=0x5449, PID=0x0145
 *   - AIC8800DC:   VID=0xc8a1, PID=0xc08d
 *   - AIC8800D80:  VID=0xc8a1, PID=0x0082
 *   - AIC8800D80N/DN: VID=0xc8a1, PID=0x9081/0x9082/0x9084/0x9085
 *   - AIC8800D80WN:   VID=0xc8a1, PID=0x9083/0x9086
 *   - AIC8800D80X2:   VID=0xc8a1, PID=0x2082
 *
 * @param vid hardware vendor ID (from card->cis.manufacturer)
 * @param pid hardware product ID (from card->cis.product)
 * @return PRODUCT_ID_* enum value (AIC8801 is 0), or -1 if unmatched (falls back to the compile-time macro)
 */
static int aicwf_sdio_chipmatch(uint16_t vid, uint16_t pid)
{
    if (vid == SDIO_VENDOR_ID_AIC8801 && pid == SDIO_DEVICE_ID_AIC8801)
    {
        DBG_SDIO_INF("%s: USE AIC8801\n", __func__);
        return PRODUCT_ID_AIC8801;
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800DC && pid == SDIO_DEVICE_ID_AIC8800DC)
    {
        DBG_SDIO_INF("%s: USE AIC8800DC\n", __func__);
        return PRODUCT_ID_AIC8800DC;
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800D80 && pid == SDIO_DEVICE_ID_AIC8800D80)
    {
        DBG_SDIO_INF("%s: USE AIC8800D80\n", __func__);
        return PRODUCT_ID_AIC8800D80;
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800D80N &&
             (pid == SDIO_DEVICE_ID_AIC8800D80N ||
              pid == SDIO_DEVICE_ID_AIC8800D80LN ||
              pid == SDIO_DEVICE_ID_AIC8800D40N ||
              pid == SDIO_DEVICE_ID_AIC8800D40LN))
    {
        DBG_SDIO_INF("%s: USE AIC8800D80N (pid=0x%04X)\n", __func__, pid);
        return PRODUCT_ID_AIC8800D80N;
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800D80N &&
             (pid == SDIO_DEVICE_ID_AIC8800D80WN ||
              pid == SDIO_DEVICE_ID_AIC8800D40WN))
    {
        DBG_SDIO_INF("%s: USE AIC8800D80WN (pid=0x%04X)\n", __func__, pid);
        return PRODUCT_ID_AIC8800D80N; /* shares the initialization path with D80N */
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800D80X2 && pid == SDIO_DEVICE_ID_AIC8800D80X2)
    {
        DBG_SDIO_INF("%s: USE AIC8800D80X2\n", __func__);
        return PRODUCT_ID_AIC8800D80X2;
    }
    else
    {
        DBG_SDIO_ERR("%s: unknown VID=0x%04X, PID=0x%04X, fallback to compile-time\n",
                     __func__, vid, pid);
        return -1;
    }
}

/**
 * @brief Set the chip-side SDIO interrupt after firmware load completes
 *
 * During the bootrom stage this register may not be ready (writes fail); call this function
 * after start_from_bootrom succeeds to write it again. Aligned with the source project's
 * aicwf_sdio_enable_irq_after_fw_boot().
 * 0x07 is written in both interrupt and polling modes: chip-side SDIO interrupt enable is a
 * prerequisite for the chip keeping the SDIO path working; writing 0 makes all CMD52
 * time out after firmware boot. Polling mode only refrains from claim_irq / releasing
 * CARD_INT on the host side.
 *
 * @return 0=success, -1=failure
 */
int aicwf_sdio_enable_irq_after_fw_boot(void)
{
    int ret;
    uint8_t irq_cfg = 0x07;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    DBG_SDIO_INF("aicwf_sdio_enable_irq_after_fw_boot: write intr_config_reg=0x%02x (rx_mode=%d)\n",
                 irq_cfg, aicwifi_sdio_rxdata_mode);

    ret = aicwf_sdio_writeb_cmd52(sdiodev->sdio_reg.intr_config_reg, irq_cfg);
    if (!ret)
    {
        DBG_SDIO_ERR("%s: post-fw-boot write intr_config_reg still failed!\n", __func__);
        return -1;
    }

    DBG_SDIO_INF("aicwf_sdio_enable_irq_after_fw_boot: intr_config_reg=0x%02x OK\n", irq_cfg);
    return 0;
}

#endif /* CONFIG_SDIO_SUPPORT */
