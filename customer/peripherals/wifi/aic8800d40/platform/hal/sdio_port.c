/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
#include "sys_al.h"
#include "wifi_al.h"
#include "sdio_al.h"
#include "sdio_def.h"
#include "sdio_port.h"
#include "wifi_port.h"
#include "rtos_port.h"
#include "rtos_errno.h"
#include "aic_plat_log.h"
#include "aic_plat_mem.h"
#include "aic_plat_hal.h"
#ifdef CONFIG_OOB
#include "aic_plat_gpio.h"
#endif /* CONFIG_OOB */
#include <drivers/mmcsd_card.h>

uint32_t  DEVICEID = 0xc18d;	// 8800DC/DW/DL  // wait for using.
#define DRIVER_CHIP_ID  PRODUCT_ID_AIC8800D80

#define CONFIG_SDIO_HOST_MUTEX  0 // enabled only if the platform not support sdio claim/release host api

//-------------------------------------------------------------------
// Driver Variables define
//-------------------------------------------------------------------
/* sdio base */
bool_l func_flag_tx = true;
bool_l func_flag_rx = true;

int msgcfm_poll_en = 1;
static uint32_t sdio_block_size;
uint8_t sdio_unexcept_test = 0;
struct aic_sdio_dev sdio_dev = {NULL,};

struct sdio_func *sdio_function[SDIOM_MAX_FUNCS];
static void (*sdio_irq_cb)(void*);
#if CONFIG_SDIO_HOST_MUTEX
rtos_mutex sdio_host_mutex = NULL;
#endif

//-------------------------------------------------------------------
// Driver Import functions
//-------------------------------------------------------------------


//-------------------------------------------------------------------
// Driver Macros Functions
//-------------------------------------------------------------------
#define	SDIO_PRE_OP() {\
	if(!(wifi_chip_status_get()))\
		return -1;\
}

#define	SDIO_PRE_OP_B() {\
	if(!(wifi_chip_status_get()))\
		return false;\
}

#define	SDIO_PRE_OPV(v) {\
	if(!(wifi_chip_status_get()))\
	{\
		*v=-1;\
		return;\
	}\
}

#define	SDIO_PRE_OPVR(v) {\
	if(!(wifi_chip_status_get()))\
	{\
		*v=-1;\
		return -1;\
	}\
}

#define	SDIO_WR_FAIL(ret){\
	if(ret)\
		sdio_dev.wr_fail++;\
	else\
		sdio_dev.wr_fail = 0;\
	if(sdio_dev.wr_fail > 15)\
	{\
		aic_driver_unexcepted(0x04);\
	}\
}


#define SDIO_ANY_ID (~0)

#if 0
static struct rt_sdio_device_id aicw_sdio_ids[] =
{
        {0,SDIO_VENDOR_ID_AIC8800DC,SDIO_DEVICE_ID_AIC8800DC},
};
#endif
//-------------------------------------------------------------------
// Driver functions define
//-------------------------------------------------------------------
/* AIC driver sdio base functions */
void aic_sdio_set_clock(uint32_t clk)
{
    struct sdio_func *func = sdio_function[FUNC_0];
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if (rt_sdio_card) {
        if (clk == 0) { // use freq_max
            clk = (rt_sdio_card->host->freq_max > 50000000) ? 50000000 : rt_sdio_card->host->freq_max;
        }
        DBG_SDIO_INF("host set clock %d Hz\n", clk);
        mmcsd_set_clock(rt_sdio_card->host, clk);
    }
}

int aic_sdio_enable_func(struct sdio_func *func)
{
    int ret = -1;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    //DBG_SDIO_INF("%s\n",__func__);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = (int)sdio_enable_func(rt_sdio_card->sdio_function[func_num]);
    }
    return ret;
}

int xhci_sdio_disable_func(struct sdio_func *func)
{
    int ret = -1;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    //DBG_SDIO_INF("%s\n",__func__);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = (int)sdio_disable_func(rt_sdio_card->sdio_function[func_num]);
    }
    return ret;
}

void sdio_claim_host(struct sdio_func *func)
{
    #if !CONFIG_SDIO_HOST_MUTEX
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if (rt_sdio_card) {
        //DBG_SDIO_INF("claim host=%x\n", g_wifi_if_sdio->host);
        mmcsd_host_lock(rt_sdio_card->host);
    }
    #else
    rtos_mutex_lock(sdio_host_mutex, -1);
    #endif
}

void sdio_release_host(struct sdio_func *func)
{
    #if !CONFIG_SDIO_HOST_MUTEX
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if (rt_sdio_card) {
        mmcsd_host_unlock(rt_sdio_card->host);
        //DBG_SDIO_INF("release host=%x\n", g_wifi_if_sdio->host);
    }
    #else
    rtos_mutex_unlock(sdio_host_mutex);
    #endif
}

unsigned char sdio_readb(struct sdio_func *func, unsigned int addr, int *err_ret)
{
    rt_int32_t ret;
    unsigned char b = 0;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        b = sdio_io_readb(rt_sdio_card->sdio_function[func_num], addr, &ret);
    }
    if (err_ret) {
        *err_ret = ret;
    }
    return b;
}

void sdio_writeb(struct sdio_func *func, unsigned char b, unsigned int addr, int *err_ret)
{
    rt_int32_t ret;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = sdio_io_writeb(rt_sdio_card->sdio_function[func_num], addr, b);
    }
    if (err_ret) {
        *err_ret = ret;
    }
}

int sdio_readsb(struct sdio_func *func, void *dst, unsigned int addr, int count)
{
    int ret = 0;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    //DBG_SDIO_VRB("%s, %d, cnt=%d\n", __func__, func_num, count);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = sdio_io_read_multi_fifo_b(rt_sdio_card->sdio_function[func_num], (rt_uint32_t)addr, dst, count);
    }
    return ret;
}

int sdio_writesb(struct sdio_func *func, unsigned int addr, void *src, int count)
{
    int ret = 0;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    //DBG_SDIO_VRB("%s, %d, cnt=%d\n", __func__, func_num, count);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = sdio_io_write_multi_fifo_b(rt_sdio_card->sdio_function[func_num], (rt_uint32_t)addr, src, count);
    }
    return ret;
}

static uint8_t sdio_f0_readb(unsigned int addr, int *err_ret)
{
    uint8_t b = 0;
    int func_num = FUNC_0;
    struct rt_mmcsd_card *rt_sdio_card = sdio_function[func_num]->drv_priv;
    if (rt_sdio_card) {
        rt_int32_t ret;
        b = sdio_io_readb(rt_sdio_card->sdio_function[func_num], addr, &ret);
        if (err_ret) {
            *err_ret = ret;
        }
    }
    return b;
}

static void sdio_f0_writeb(unsigned char b, unsigned int addr, int *err_ret)
{
    int func_num = FUNC_0;
    struct rt_mmcsd_card *rt_sdio_card = sdio_function[func_num]->drv_priv;
    if (rt_sdio_card) {
        rt_int32_t ret;
        ret = sdio_io_writeb(rt_sdio_card->sdio_function[func_num], addr, b);
        if (err_ret) {
            *err_ret = ret;
        }
    }
}

#if 0
bool sdio_readb_cmd52(uint32_t addr, uint8_t *data)
{
    int err;

	SDIO_PRE_OP_B();
    //AIC_LOG_PRINTF("sdio_readb_cmd52, addr: 0x%x", addr);
    sdio_claim_host(sdio_function[FUNC_1]);

    *data = sdio_io_readb(sdio_function[FUNC_1],addr,&err);
	SDIO_WR_FAIL(err);

    if(err) {
        //AIC_LOG_PRINTF("sdio_readb_cmd52 fail %d!\n", err);
        AIC_LOG_PRINTF("sdio_readb_cmd52 fail %d!\n", err);
        return FALSE;
    }

    //AIC_LOG_PRINTF("sdio_readb_cmd52 done, val=0x%x\n", val);
    sdio_release_host(sdio_function[FUNC_1]);
    return TRUE;
}

bool sdio_readb_cmd52_func2(uint32_t addr, uint8_t *data)
{
    int err;
	SDIO_PRE_OP_B();
    //AIC_LOG_PRINTF("sdio_readb_cmd52, addr: 0x%x", addr);
    sdio_claim_host(sdio_function[FUNC_2]);
    *data = sdio_io_readb(sdio_function[FUNC_2],addr,&err);
	SDIO_WR_FAIL(err);
    if(err) {
        AIC_LOG_PRINTF("sdio_readb_cmd52_func2 fail %d!\n", err);
        return FALSE;
    }

    //AIC_LOG_PRINTF("sdio_readb_cmd52, val=0x%x", val);
    sdio_release_host(sdio_function[FUNC_2]);
    return TRUE;
}

bool sdio_writeb_cmd52(uint32_t addr, uint8_t data)
{
    int err;

	SDIO_PRE_OP_B();
    sdio_claim_host(sdio_function[FUNC_1]);
    err = sdio_io_writeb(sdio_function[FUNC_1],addr,data);
	SDIO_WR_FAIL(err);
    if(err) {
        AIC_LOG_PRINTF("sdio_writeb_cmd52 fail %d!\n", err);
        return FALSE;
    }

    sdio_release_host(sdio_function[FUNC_1]);

    //AIC_LOG_PRINTF("sdio_writeb_cmd52 done, addr 0x%x, data=0x%x\n", addr, data);

    return TRUE;
}

static bool sdio_writeb_cmd52_func2(uint32_t addr, uint8_t data)
{
    int err;

	SDIO_PRE_OP_B();
    sdio_claim_host(sdio_function[FUNC_2]);
    err = sdio_io_writeb(sdio_function[FUNC_2],addr,data);
	SDIO_WR_FAIL(err);
    if(err) {
        AIC_LOG_PRINTF("sdio_writeb_cmd52_func2 fail %d!\n", err);
        return FALSE;
    }

    sdio_release_host(sdio_function[FUNC_2]);

    //AIC_LOG_PRINTF("sdio_writeb_cmd52, addr 0x%x, data=0x%x", addr, data);

    return TRUE;
}
#endif

#ifdef CONFIG_OOB
// This interface is from ASR1803-SDK.
static void _sdio_irq_enable(UINT32 base_addr, UINT16 mask)
{
    volatile UINT16* ptr16;

    ptr16 = (UINT16*) (base_addr + SD_NORM_INTR_STS_EBLE_offset);
    *ptr16 |= mask ;

    ptr16 = (UINT16*) (base_addr + SD_NORM_INTR_STS_INTR_EBLE_offset);
    *ptr16 |= mask ;
}

static void sdio_oobirq_task(void *argv)
{
    int ret = 0;
    uint32_t wait_time = 1000;
    while(1) {
        ret = rtos_semaphore_wait(sdio_dev.sdio_oobirq_sema, wait_time);
        if (ret)
            AIC_LOG_PRINTF("sdio_oobirq_sema timeout: %d\n", ret);
        aic_sdio_rx_task(0);
    }
}

static void aicwf_sdio_oob_irq_hdl(void)
{
    int count = 0;
    count = rtos_semaphore_get_count(sdio_dev.sdio_oobirq_sema);
    if(count == 0)
        rtos_semaphore_signal(sdio_dev.sdio_oobirq_sema, true);
    else
         AIC_LOG_PRINTF("sdio oob isr: %d\n",count);
}

void aicwf_sdio_oob_enable(void)
{
    int ret = 0;
    GPIOConfiguration config;
    struct aic_sdio_dev *sdiodev = &sdio_dev;
    uint32_t oob_gpio_num = sdiodev->sdio_gpio_num;

    config.pinDir = GPIO_IN_PIN;
    #if 1
    config.pinEd = GPIO_RISE_EDGE;
    config.pinPull = GPIO_PULLDN_ENABLE;
    #else
    config.pinEd = GPIO_FALL_EDGE;
    config.pinPull = GPIO_PULLUP_ENABLE;
    #endif

    config.isr = aicwf_sdio_oob_irq_hdl;
    ret = GpioInitConfiguration(oob_gpio_num, config);
    AIC_LOG_PRINTF("sdio-oob: gpio=%d, ret=%d, LVL=%d", oob_gpio_num, ret, GpioGetLevel(oob_gpio_num));

    AIC_LOG_PRINTF("%s: close host sdio-irq", __func__);
    _sdio_irq_enable(0xD4280800, 0x0);
    sdio_set_irq_handler(NULL);

    //disable sdio interrupt
    AIC_LOG_PRINTF("%s: SDIOWIFI_INTR_CONFIG_REG Disable\n", __func__);
    ret = aicwf_sdio_writeb_func2(sdiodev, sdiodev->sdio_reg.intr_config_reg, 0x0);
    if (ret == false) {
        AIC_LOG_PRINTF("ERR: reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
    }

    ret = rtos_task_create(sdio_oobirq_task, "oobirq_task", SDIO_OOBIRQ_TASK,
                           sdio_oobirq_stack_size, NULL, sdio_oobirq_priority, &sdiodev->sdio_oobirq_hdl);
    if (ret) {
        AIC_LOG_PRINTF("ERR: oobirq_task create failed\n");
    } else {
        sdiodev->oob_enable = true;
    }
}
#endif /* CONFIG_OOB */

void rtt_sdio_irq_handler(struct rt_sdio_function *rt_func)
{
    int func_num = rt_func->num;
    //AIC_LOG_PRINTF("%s, %d\n", __func__, func_num);
    if (func_num < SDIOM_MAX_FUNCS) {
        struct sdio_func *func = sdio_function[func_num];
        if (func->irq_handler) {
            func->irq_handler(func);
        }
    }
}

int aic_sdio_claim_irq(struct sdio_func *func, aicwf_sdio_irq_handler_t handler)
{
    int ret = 0;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if (func->irq_handler == NULL) {
        func->irq_handler = handler;
    }
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = (int)sdio_attach_irq(rt_sdio_card->sdio_function[func_num], rtt_sdio_irq_handler);
    }
    return ret;
}

int aic_sdio_release_irq(struct sdio_func *func)
{
    int ret = 0;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    if (func->irq_handler) {
        func->irq_handler = NULL;
    }
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = (int)sdio_detach_irq(rt_sdio_card->sdio_function[func_num]);
    }
    return ret;
}

void sdio_set_irq_handler(void (*cb)(void))
{
	sdio_irq_cb = cb;
}

bool sdio_host_enable_isr(bool enable)
{
    //if(sdio_irq_cb)
    //    sdio_attach_irq();
    return TRUE;
}

static uint32_t sdio_get_block_size(void)
{
    return sdio_block_size;
}

int aic_sdio_set_block_size(struct sdio_func *func, unsigned int blk_sz)
{
    int ret = 0;
    int func_num = func->num;
    struct rt_mmcsd_card *rt_sdio_card = func->drv_priv;
    //DBG_SDIO_INF("%s, %d, blksz=%d\n", __func__, func_num, blk_sz);
    if ((func_num <= SDIO_MAX_FUNCTIONS) && rt_sdio_card) {
        ret = (int)sdio_set_block_size(rt_sdio_card->sdio_function[func_num], (rt_uint32_t)blk_sz);
        if (!ret) {
            sdio_block_size = blk_sz;
        }
    }
    return ret;
}

void sdio_release_func2(void)
{
    int ret = 0;
    AIC_LOG_PRINTF("%s\n", __func__);
    sdio_claim_host(sdio_function[FUNC_2]);
    ret = aicwf_sdio_writeb_func2(&sdio_dev, sdio_dev.sdio_reg.intr_config_reg, 0x0);
    if (ret < 0) {
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdio_dev.sdio_reg.intr_config_reg);
    }

    //sdio_release_irq(sdio_function[FUNC_2]);
    sdio_release_host(sdio_function[FUNC_2]);
}

static void aicwf_sdio_irq_hdl(void *arg)
{
    //AIC_LOG_PRINTF("sdio isr, arg=%x\n",arg);
    #ifdef CONFIG_OOB
    if (sdio_dev.oob_enable == false)
    #endif
    {
        #if !CONFIG_RXTASK_INSDIO
        int count;
        //sdio_host_enable_isr(false);
        //count = rtos_semaphore_get_count(sdio_rx_sema);
        count = 0;
        if(count == 0)
            rtos_semaphore_signal(sdio_dev.sdio_rx_sema, true);
        else
            AIC_LOG_PRINTF("sdio isr->> %d\n",count);
        #else
        aic_sdio_rx_task(0);
        #endif
    }
}

int sdio_interrupt_init(struct aic_sdio_dev *sdiodev)
{
    int ret;
    AIC_LOG_PRINTF("%s, chipid=%d\n", __func__, sdiodev->chipid);
    // func1
    sdio_claim_host(sdio_function[FUNC_1]);
    aic_sdio_claim_irq(sdio_function[FUNC_1], aicwf_sdio_irq_hdl);
    //enable sdio interrupt
    sdio_writeb(sdio_function[FUNC_1], 0x7, sdiodev->sdio_reg.intr_config_reg, &ret);
    sdio_release_host(sdio_function[FUNC_1]);
    if (ret) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
        return ret;
    }
    if ((sdiodev->chipid == PRODUCT_ID_AIC8800DC) || (sdiodev->chipid == PRODUCT_ID_AIC8800DW)) {
        // func2
        sdio_claim_host(sdio_function[FUNC_2]);
        aic_sdio_claim_irq(sdio_function[FUNC_2], aicwf_sdio_irq_hdl);
        //enable sdio interrupt
        sdio_writeb(sdio_function[FUNC_2], 0x7, sdiodev->sdio_reg.intr_config_reg, &ret);
        sdio_release_host(sdio_function[FUNC_2]);
        if (ret) {
            ret = EREMOTEIO;
            AIC_LOG_PRINTF("func2 reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
            return ret;
        }
    }
    msgcfm_poll_en = 0;
    return 0;
}

int sdio_interrupt_deinit(struct aic_sdio_dev *sdiodev)
{
    int ret;
    AIC_LOG_PRINTF("%s, chipid=%d\n", __func__, sdiodev->chipid);
    // func1
    sdio_claim_host(sdio_function[FUNC_1]);
    aic_sdio_release_irq(sdio_function[FUNC_1]);
    //enable sdio interrupt
    sdio_writeb(sdio_function[FUNC_1], 0x0, sdiodev->sdio_reg.intr_config_reg, &ret);
    sdio_release_host(sdio_function[FUNC_1]);
    if (ret) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
        return ret;
    }
    if ((sdiodev->chipid == PRODUCT_ID_AIC8800DC) || (sdiodev->chipid == PRODUCT_ID_AIC8800DW)) {
        // func2
        sdio_claim_host(sdio_function[FUNC_2]);
        aic_sdio_release_irq(sdio_function[FUNC_2]);
        //enable sdio interrupt
        sdio_writeb(sdio_function[FUNC_2], 0x0, sdiodev->sdio_reg.intr_config_reg, &ret);
        sdio_release_host(sdio_function[FUNC_2]);
        if (ret) {
            ret = EREMOTEIO;
            AIC_LOG_PRINTF("func2 reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
            return ret;
        }
    }
    msgcfm_poll_en = 1;
    return 0;
}

int aicwf_sdio_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev)
{
    int32_t ret = 0;
    uint8_t block_bit0 = 0x1;
    uint8_t byte_mode_disable = 0x1;//1: no byte mode

    AIC_LOG_PRINTF("%s: chipid=%d\n", __func__, chipid);
    /* SDIO Function 1 */
    sdio_claim_host(sdio_function[FUNC_1]);

    ret = aic_sdio_set_block_size(sdio_function[FUNC_1], SDIOWIFI_FUNC_BLOCKSIZE);
    if (ret) {
        AIC_LOG_PRINTF("func1 blksize set failed, ret=%d\n", ret);
    }
    AIC_LOG_PRINTF("sdio_host_init:sdio_set_block_size %d\n", SDIOWIFI_FUNC_BLOCKSIZE);
    if (sdio_block_size != SDIOWIFI_FUNC_BLOCKSIZE) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("sdio_host_init: blksize set failed\n");
        return ret;
    }

    ret = aic_sdio_enable_func(sdio_function[FUNC_1]);
    if (ret) {
        AIC_LOG_PRINTF("sdio func1 enable failed, ret=%d\n", ret);
        return ret;
    }

    sdio_writeb(sdio_function[FUNC_1], block_bit0, sdiodev->sdio_reg.register_block, &ret);
    if (ret < 0) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.register_block);
        return ret;
    }

    //1: no byte mode
    sdio_writeb(sdio_function[FUNC_1], byte_mode_disable, sdiodev->sdio_reg.bytemode_enable_reg, &ret);
    if (ret < 0) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
        return ret;
    }

    #if 0
    //enable sdio interrupt
    sdio_writeb(sdio_function[FUNC_1], 0x7, sdiodev->sdio_reg.intr_config_reg, &ret);
    if (ret < 0) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
        return ret;
    }
    #endif

    sdio_release_host(sdio_function[1]);
    AIC_LOG_PRINTF("sdio_host_init:enable fun1 ok!\n");


    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW) {
        /* SDIO Function 2 */
        sdio_claim_host(sdio_function[FUNC_2]);

        //set sdio blocksize
        ret = aic_sdio_set_block_size(sdio_function[FUNC_2], SDIOWIFI_FUNC_BLOCKSIZE);
        if (ret) {
            AIC_LOG_PRINTF("func2 blksize set failed, ret=%d\n", ret);
        }
        AIC_LOG_PRINTF("sdio_host_init:func2:sdio_set_block_size %d\n", SDIOWIFI_FUNC_BLOCKSIZE);
        if (sdio_block_size != SDIOWIFI_FUNC_BLOCKSIZE) {
            AIC_LOG_PRINTF("sdio_host_init:func2: blksize set failed\n");
        }

        //set sdio enable func
        ret = aic_sdio_enable_func(sdio_function[FUNC_2]);
        if (ret) {
            AIC_LOG_PRINTF("sdio func2 enable failed, ret=%d\n", ret);
            return ret;
        }

        sdio_writeb(sdio_function[FUNC_2], block_bit0, sdiodev->sdio_reg.register_block, &ret);
        if (ret < 0) {
            AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.register_block);
            return ret;
        }

        //1: no byte mode
        sdio_writeb(sdio_function[FUNC_2], byte_mode_disable, sdiodev->sdio_reg.bytemode_enable_reg, &ret);
        if (ret < 0) {
            AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
            return ret;
        }

        #if 0
        //enable sdio interrupt
        sdio_writeb(sdio_function[FUNC_2], 0x7, sdiodev->sdio_reg.intr_config_reg, &ret);
        if (ret < 0) {
            AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
            return ret;
        }
        #endif

        sdio_release_host(sdio_function[FUNC_2]);
        AIC_LOG_PRINTF("sdio_host_init:enable fun2 ok!\n");
    }

    return 0;
}

int aicwf_sdiov3_func_init(uint16_t chipid, struct aic_sdio_dev *sdiodev)
{
    int32_t ret = 0;
    uint8_t byte_mode_disable = 0x1;//1: no byte mode

    AIC_LOG_PRINTF("%s: chipid=%d\n", __func__, chipid);

    /* SDIO Function 1 */
    sdio_claim_host(sdio_function[FUNC_1]);

    ret = aic_sdio_set_block_size(sdio_function[FUNC_1], SDIOWIFI_FUNC_BLOCKSIZE);
    if (ret) {
        AIC_LOG_PRINTF("func1 blksize set failed, ret=%d\n", ret);
    }
    AIC_LOG_PRINTF("sdio_host_init:sdio_set_block_size %d\n", SDIOWIFI_FUNC_BLOCKSIZE);
    if (sdio_block_size != SDIOWIFI_FUNC_BLOCKSIZE) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("sdio_host_init: blksize set failed\n");
        return ret;
    }

    ret = aic_sdio_enable_func(sdio_function[FUNC_1]);
    if (ret) {
        AIC_LOG_PRINTF("sdio func1 enable failed, ret=%d\n", ret);
        return ret;
    }

    #if 0 //set phase and sample
    {
        u8 val = 0;
        sdio_f0_writeb(0x7F, 0xF2, &ret);
        if (ret) {
            AIC_LOG_PRINTF("set fn0 0xF2 fail %d\n", ret);
            return ret;
        }
        val |= SDIOCLK_FREE_RUNNING_BIT;
        sdio_f0_writeb(val, 0xF0, &ret);
        if (ret) {
            AIC_LOG_PRINTF("set iopad ctrl fail %d\n", ret);
            return ret;
        }
        sdio_f0_writeb(0x0, 0xF8, &ret);
        if (ret) {
            AIC_LOG_PRINTF("set iopad delay2 fail %d\n", ret);
            return ret;
        }
        sdio_f0_writeb(0x20, 0xF1, &ret);
        if (ret) {
            AIC_LOG_PRINTF("set iopad delay1 fail %d\n", ret);
            return ret;
        }
        rtos_udelay(100);
    }
    #endif

    sdio_f0_writeb(0x7F, 0xF2, &ret);
    if (ret) {
        AIC_LOG_PRINTF("set fn0 0xF2 fail %d\n", ret);
        return ret;
    }

    aic_sdio_set_clock(0);

    //1: no byte mode
    sdio_writeb(sdio_function[FUNC_1], byte_mode_disable, sdiodev->sdio_reg.bytemode_enable_reg, &ret);
    if (ret < 0) {
        AIC_LOG_PRINTF("reg:%d write failed! ret=%d\n", sdiodev->sdio_reg.bytemode_enable_reg, ret);
        return ret;
    }

    #if 0
    sdio_claim_host(sdio_function[FUNC_0]);
	sdio_writeb(sdio_function[FUNC_0], 0x07, 0x04, &ret);
	if(ret != TRUE){
		AIC_LOG_PRINTF("set func0 int en fail %d\n", ret);
		sdio_release_host(sdio_function[FUNC_0]);
		return FALSE;
	}
    sdio_release_host(sdio_function[FUNC_0]);

    //enable sdio interrupt
    sdio_f0_writeb(0x07, 0x04, &ret);
    if (ret) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("set func0 int en fail %d\n", ret);
    }
    ret = sdio_writeb_cmd52(sdiodev->sdio_reg.intr_config_reg, 0x7);
    if (ret == FALSE) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
        return ret;
    }
    #endif

    sdio_release_host(sdio_function[FUNC_1]);
    AIC_LOG_PRINTF("sdio_host_init:enable fun1 ok!\n");

    return 0;
}

int aicwf_sdio_probe(struct sdio_func *func)
{
    int ret = 0;
    #if CONFIG_SDIO_HOST_MUTEX
    ret = rtos_mutex_create(&sdio_host_mutex, "sdio_host_mutex");
    if (ret) {
        AIC_LOG_PRINTF("Alloc sdio_host_mutex failed, ret=%d\n", ret);
        return ret;
    }
    #endif
    sdio_function[FUNC_0] = &func[0];
    sdio_function[FUNC_1] = &func[1];
    sdio_function[FUNC_2] = &func[2];
    #if 1
    aic_wifi_init(CONFIG_WIFIMODE_SELECT, CONFIG_CHIPID_SELECT, NULL);
    #else
    wifi_driver_init();
    #endif
    return ret;
}

int aicwf_sdio_remove(struct sdio_func *func)
{
    int ret = 0;
    //aic_driver_reboot(0x05);
    aic_wifi_deinit(WIFI_MODE_UNKNOWN);
    #if CONFIG_SDIO_HOST_MUTEX
    if (sdio_host_mutex) {
        rtos_mutex_delete(sdio_host_mutex);
        sdio_host_mutex = NULL;
    }
    #endif
    return ret;
}

#if 0
int aicw_sdio_register_init(void)
{
    int ret =0;
    aicwwifi_driver.name="aic8800";
    aicwwifi_driver.id  = aicw_sdio_ids;
    aicwwifi_driver.probe      = aicw_sdio_probe;
    aicwwifi_driver.remove     = aicw_sdio_remove;

    ret = rt_sdio_register(&aicwwifi_driver);
    if (ret){
        AIC_LOG_PRINTF("aicifi sdio driver register fail\n");
    }
    return ret;
}

int aicw_sdio_register_deinit()
{
    rt_sdio_deregister(&aicwwifi_driver);

    return 0;
}
#endif

