/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#if !defined(BSP_USING_PC_SIMULATOR)
#include "bf0_mbox_common.h"

#if (defined(BSP_USING_FLASH) || defined(BSP_USING_SPI_FLASH))
    #include "drv_flash.h"
#endif
#ifdef BSP_USING_PSRAM
    #include "drv_psram.h"
#endif

#ifdef BSP_USING_EXT_DMA
    #include "drv_ext_dma.h"
#endif

#ifdef USING_SEC_ENV
    #include "security.h"
#endif

#define LOG_TAG      "mw.sys"
#include "log.h"

#ifdef SOC_BF0_HCPU
    #define SYS_HL_IPC_QUEUE  (1)
    #define SYS_HL_TX_BUF_SIZE   (HCPU2LCPU_MB_CH2_BUF_SIZE)
    #define SYS_HL_TX_BUF_ADDR   (HCPU2LCPU_MB_CH2_BUF_START_ADDR)
    #define SYS_HL_TX_BUF_ADDR_ALIAS   (HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR))
    #define SYS_HL_RX_BUF_ADDR   (LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR))

    #define SYS_HL_DEBUG_QUEUE (7)

    #define SYS_HB_IPC_QUEUE  (9)
    #define SYS_HB_TX_BUF_SIZE   (HCPU2BCPU_MB_CH2_BUF_SIZE)
    #define SYS_HB_TX_BUF_ADDR   (HCPU2BCPU_MB_CH2_BUF_START_ADDR)
    #define SYS_HB_TX_BUF_ADDR_ALIAS   (HCPU_ADDR_2_BCPU_ADDR(HCPU2BCPU_MB_CH2_BUF_START_ADDR))
    #define SYS_HB_RX_BUF_ADDR   (BCPU_ADDR_2_HCPU_ADDR(BCPU2HCPU_MB_CH2_BUF_START_ADDR))

#elif defined(SOC_BF0_LCPU)
    #define SYS_LH_IPC_QUEUE  (1)
    #define SYS_LH_TX_BUF_SIZE   (LCPU2HCPU_MB_CH2_BUF_SIZE)
    #define SYS_LH_TX_BUF_ADDR   (LCPU2HCPU_MB_CH2_BUF_START_ADDR)
    #define SYS_LH_TX_BUF_ADDR_ALIAS   (LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR))
    #define SYS_LH_RX_BUF_ADDR   (HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR))

    #define SYS_LH_DEBUG_QUEUE    (7)

#endif

typedef struct
{
    hpsys_clk_setting_t hpsys_clk;
    lpsys_clk_setting_t lpsys_clk;
    blesys_clk_setting_t blesys_clk;
#ifdef SF32LB55X
    uint32_t flash1_clk;
    uint32_t flash2_clk;
    uint32_t flash3_clk;
    uint32_t flash4_clk;
    uint32_t psram_clk;
#else
    uint32_t mpi1_clk;
    uint32_t mpi2_clk;
    uint32_t mpi3_clk;
    uint32_t mpi4_clk;
    uint32_t mpi5_clk;
#endif
    struct rt_device_graphic_info lcd_info;
} clk_setting_t;

#ifdef USING_IPC_QUEUE
    #ifdef SOC_BF0_HCPU
        static ipc_queue_handle_t sys_hl_ipc_queue = IPC_QUEUE_INVALID_HANDLE;
        #if defined(RT_DEBUG) && defined(SF32LB55X)
            static ipc_queue_handle_t sys_hl_debug_queue = IPC_QUEUE_INVALID_HANDLE;
        #endif /* RT_DEBUG && SF32LB55X*/
    #endif /* SOC_BF0_HCPU */

    #ifdef SOC_BF0_LCPU
        static ipc_queue_handle_t sys_lh_ipc_queue = IPC_QUEUE_INVALID_HANDLE;
        #if defined(RT_DEBUG) && defined(SF32LB55X)
            static ipc_queue_handle_t sys_lh_debug_queue = IPC_QUEUE_INVALID_HANDLE;
        #endif /* RT_DEBUG && SF32LB55X */
    #endif /* SOC_BF0_LCPU */
#endif /* USING_IPC_QUEUE */

/** @addtogroup mailbox library
 * @ingroup middleware
 * @brief
 * @{
 */

#ifdef USING_IPC_QUEUE

#ifdef SOC_BF0_HCPU
ipc_queue_handle_t sys_init_hl_ipc_queue(ipc_queue_rx_ind_t rx_ind)
{
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = SYS_HL_IPC_QUEUE;
    q_cfg.tx_buf_size = SYS_HL_TX_BUF_SIZE;
    q_cfg.tx_buf_addr = SYS_HL_TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = SYS_HL_TX_BUF_ADDR_ALIAS;
#if defined(BSP_CHIP_ID_COMPATIBLE) && defined(SF32LB55X)
    q_cfg.rx_buf_addr = SYS_HL_RX_BUF_ADDR + ((__HAL_SYSCFG_GET_REVID() & HAL_CHIP_REV_ID_A3) >> 7) * (LPSYS_RAM_SIZE_A3 - LPSYS_RAM_SIZE);
#else
    q_cfg.rx_buf_addr = SYS_HL_RX_BUF_ADDR;
#endif
    q_cfg.rx_ind = rx_ind;
    q_cfg.user_data = 0;

    sys_hl_ipc_queue = ipc_queue_init(&q_cfg);

    return sys_hl_ipc_queue;
}

ipc_queue_handle_t sys_get_hl_ipc_queue(void)
{
    return sys_hl_ipc_queue;
}

#else // SOC_BF0_HCPU
ipc_queue_handle_t sys_init_lh_ipc_queue(ipc_queue_rx_ind_t rx_ind)
{
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = SYS_LH_IPC_QUEUE;
    q_cfg.tx_buf_size = SYS_LH_TX_BUF_SIZE;
    q_cfg.tx_buf_addr = SYS_LH_TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = SYS_LH_TX_BUF_ADDR_ALIAS;
    q_cfg.rx_buf_addr = SYS_LH_RX_BUF_ADDR;
    q_cfg.rx_ind = rx_ind;
    q_cfg.user_data = 0;

    sys_lh_ipc_queue = ipc_queue_init(&q_cfg);

    return sys_lh_ipc_queue;
}

ipc_queue_handle_t sys_get_lh_ipc_queue(void)
{
    return sys_lh_ipc_queue;
}

#endif // SOC_BF0_HCPU
#endif // USING_IPC_QUEUE

/* Trigger peer core assert. */
#ifdef RT_DEBUG
#if defined(SF32LB55X) && defined(USING_IPC_QUEUE)
#ifdef SOC_BF0_HCPU
static void assert_hook(const char *ex, const char *func, rt_size_t line)
{
    ipc_queue_write(sys_hl_debug_queue, NULL, 0, 0);
}

rt_err_t exception_handler(void *context)
{
    ipc_queue_write(sys_hl_debug_queue, NULL, 0, 0);
    return RT_ERROR;
}

int32_t debug_queue_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    LOG_E("LCPU crash");
    RT_ASSERT(0);
    return 0;
}

int sys_init_hl_debug_queue(void)
{
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = SYS_HL_DEBUG_QUEUE;
    q_cfg.tx_buf_size = 0;
    q_cfg.tx_buf_addr = (uint32_t)NULL;
    q_cfg.tx_buf_addr_alias = (uint32_t)NULL;
    q_cfg.rx_buf_addr = (uint32_t)NULL;
    q_cfg.rx_ind = debug_queue_rx_ind;
    q_cfg.user_data = 0;

    sys_hl_debug_queue = ipc_queue_init(&q_cfg);

    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != sys_hl_debug_queue);

    ipc_queue_open(sys_hl_debug_queue);

    rt_assert_set_hook(assert_hook);
    rt_hw_exception_install(exception_handler);

    return 0;
}
INIT_APP_EXPORT(sys_init_hl_debug_queue);

#endif  /* SOC_BF0_HCPU */

#ifdef SOC_BF0_LCPU
static void assert_hook(const char *ex, const char *func, rt_size_t line)
{
    ipc_queue_write(sys_lh_debug_queue, NULL, 0, 0);
}

static void assert_hook_rom(const char *ex, const char *func, rt_size_t line)
{
    rt_assert_handler(ex, func, line);
}

rt_err_t exception_handler(void *context)
{
    ipc_queue_write(sys_lh_debug_queue, NULL, 0, 0);
    return RT_ERROR;
}

rt_err_t exception_handler_rom(void *context)
{
    HAL_LPAON_WakeCore(CORE_ID_HCPU);
    ipc_queue_write(sys_lh_debug_queue, NULL, 0, 0);
    return RT_ERROR;
}

int32_t debug_queue_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    LOG_E("HCPU crash");
    RT_ASSERT(0);
    return 0;
}

int sys_init_lh_debug_queue(void)
{
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = SYS_LH_DEBUG_QUEUE;
    q_cfg.tx_buf_size = 0;
    q_cfg.tx_buf_addr = (uint32_t)NULL;
    q_cfg.tx_buf_addr_alias = (uint32_t)NULL;
    q_cfg.rx_buf_addr = (uint32_t)NULL;
    q_cfg.rx_ind = debug_queue_rx_ind;
    q_cfg.user_data = 0;

    sys_lh_debug_queue = ipc_queue_init(&q_cfg);

    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != sys_lh_debug_queue);

    ipc_queue_open(sys_lh_debug_queue);

    rt_assert_set_hook(assert_hook);
    rt_hw_exception_install(exception_handler);
#ifndef FPGA
    rt_assert_set_hook_rom(assert_hook_rom);
    rt_hw_exception_install_rom(exception_handler);
#endif
    return 0;
}
INIT_APP_EXPORT(sys_init_lh_debug_queue);

#endif /* SOC_BF0_LCPU */

#elif  !defined(SF32LB55X)
static void assert_hook(const char *ex, const char *func, rt_size_t line)
{
    // Avoid recurisive
    if (__HAL_SYSCFG_Get_Trigger_Assert_Flag() == 0)
    {
#ifdef SOC_BF0_HCPU
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
#else
        HAL_LPAON_WakeCore(CORE_ID_HCPU);

#if defined(SF32LB52X) && defined(USING_SEC_ENV)
        //sf52x in security mode, set hcpu security mode before trigger nmi irq
        set_hcpu_to_security_mode();
#endif
#endif
        __HAL_SYSCFG_Trigger_Assert();
#if defined(SF32LB52X) && defined(SOC_BF0_LCPU)
        rt_hw_fatal_error();
#endif
    }
}

rt_err_t exception_handler(void *context)
{
    // Avoid recurisive
    if (__HAL_SYSCFG_Get_Trigger_Assert_Flag() == 0)
    {
#ifdef SOC_BF0_HCPU
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
#else
        HAL_LPAON_WakeCore(CORE_ID_HCPU);

#if defined(SF32LB52X) && defined(USING_SEC_ENV)
        //sf52x in security mode, set hcpu security mode before trigger nmi irq
        set_hcpu_to_security_mode();
#endif
#endif
        __HAL_SYSCFG_Trigger_Assert();
    }
    return RT_ERROR;
}

void DBG_Trigger_IRQHandler(void)
{
    rt_interrupt_enter();
#ifdef SOC_BF0_HCPU
    LOG_E("LCPU Crash triggered, please check LCPU log for details.");
#else
    LOG_E("HCPU Crash triggered, please check HCPU log for details.");
#endif
    RT_ASSERT(0);
    rt_interrupt_leave();
}

int sys_init_debug_trigger(void)
{
    __HAL_SYSCFG_Enable_Assert_Trigger(1);

    rt_assert_set_hook(assert_hook);
    rt_hw_exception_install(exception_handler);

    return 0;
}
INIT_APP_EXPORT(sys_init_debug_trigger);
#endif // defined(SF32LB55X) && defined(USING_IPC_QUEUE)
#endif // RT_DEBUG

#ifdef BSP_USING_ADC1
void adc_resume(void)
{
    rt_device_t dev = rt_device_find("bat1");
    if (dev)
        rt_adc_init((rt_adc_device_t)dev);
}
#endif

#define HUMAN_FORMAT "%.2f%c"
static void print_clk_str(char *buf,  size_t buf_len, const char *format, uint32_t freq)
{
    char unit;
    float val;
    if (freq >= 1000000)
    {
        val = (float)freq / 1000000;
        unit = 'M';
    }
    else if (freq >= 1000)
    {
        val = (float)freq / 1000;
        unit = 'K';
    }
    else
    {
        val = (float)freq;
        unit = ' ';
    }

    if (!buf)
    {
        rt_kprintf(format, val, unit);
    }
    else
    {
        size_t used_room = strlen(buf);
        size_t left_room = buf_len - used_room;
        if (left_room > 1) //one for '\0'
        {
            rt_snprintf(buf + used_room, left_room, format, val, unit);
        }
    }

}

#define print_var_str(buf, buf_len, format, ...) do{\
        if (!buf)\
        {\
            rt_kprintf(format, ##__VA_ARGS__);\
        }\
        else \
        { \
            size_t used_room = strlen(buf);\
            size_t left_room = buf_len - used_room;\
            if (left_room > 1) /*one for '\0' */  \
            {  \
                rt_snprintf(buf + used_room, left_room, format, ##__VA_ARGS__);\
            }\
        } \
    }while(0)

/**
 * @brief
 * @param buf - print to console if NULL
 * @param buf_len -
 */
__ROM_USED void print_sysinfo(char *buf, uint32_t buf_len)
{
    rt_device_t device;
    rt_err_t err;
    clk_setting_t clk_setting;

    drv_get_hpsys_clk(&clk_setting.hpsys_clk);
    drv_get_lpsys_clk(&clk_setting.lpsys_clk);
    drv_get_blesys_clk(&clk_setting.blesys_clk);

#ifdef SF32LB55X
#if  (defined (BSP_USING_FLASH)||defined (BSP_USING_SPI_FLASH))
    clk_setting.flash1_clk = rt_flash_get_clk(FLASH_BASE_ADDR);
    clk_setting.flash2_clk = rt_flash_get_clk(FLASH2_BASE_ADDR);
#if ((defined BSP_ENABLE_FLASH3) || (defined BSP_ENABLE_QSPI3))
    clk_setting.flash3_clk = rt_flash_get_clk(FLASH3_BASE_ADDR);
#endif  /* BSP_ENABLE_FLASH3 */
#if ((defined BSP_ENABLE_FLASH4) || (defined BSP_ENABLE_QSPI4))
    clk_setting.flash4_clk = rt_flash_get_clk(FLASH4_BASE_ADDR);
#endif  /* BSP_ENABLE_FLASH4 */
#endif /* BSP_USING_FLASH */
#ifdef BSP_USING_PSRAM
    clk_setting.psram_clk = rt_psram_get_clk(PSRAM_BASE);
#endif /* BSP_USING_PSRAM */

#else

#if  defined (BSP_ENABLE_MPI1)
#ifdef BSP_USING_PSRAM1
    clk_setting.mpi1_clk = rt_psram_get_clk(MPI1_MEM_BASE);
#else
    clk_setting.mpi1_clk = rt_flash_get_clk(MPI1_MEM_BASE);
#endif /* BSP_USING_PSRAM1 */
#endif /* BSP_ENABLE_MPI1 */

#if  defined (BSP_ENABLE_MPI2)
#ifdef BSP_USING_PSRAM2
    clk_setting.mpi2_clk = rt_psram_get_clk(MPI2_MEM_BASE);
#else
    clk_setting.mpi2_clk = rt_flash_get_clk(MPI2_MEM_BASE);
#endif /* BSP_USING_PSRAM2 */
#endif /* BSP_ENABLE_MPI2 */

#if  defined (BSP_ENABLE_MPI3)
#ifdef BSP_USING_PSRAM3
    clk_setting.mpi3_clk = rt_psram_get_clk(MPI3_MEM_BASE);
#else
#if defined(BSP_MPI3_MODE_1) && !defined(SOC_SF32LB55X)
    clk_setting.mpi3_clk = rt_flash_get_clk(MPI3_MEM_BASE + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET);
#else
    clk_setting.mpi3_clk = rt_flash_get_clk(MPI3_MEM_BASE);
#endif
#endif /* BSP_USING_PSRAM3 */
#endif /* BSP_ENABLE_MPI3 */

#if  defined (BSP_ENABLE_MPI4)
#ifdef BSP_USING_PSRAM4
    clk_setting.mpi4_clk = rt_psram_get_clk(MPI4_MEM_BASE);
#else
#if defined(BSP_MPI4_MODE_1) && !defined(SOC_SF32LB55X)
    clk_setting.mpi4_clk = rt_flash_get_clk(MPI4_MEM_BASE + HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET);
#else
    clk_setting.mpi4_clk = rt_flash_get_clk(MPI4_MEM_BASE);
#endif
#endif /* BSP_USING_PSRAM4 */
#endif /* BSP_ENABLE_MPI4 */

#if  defined (BSP_ENABLE_MPI5)
#ifdef BSP_USING_PSRAM5
    clk_setting.mpi5_clk = rt_psram_get_clk(MPI5_MEM_BASE);
#else
    clk_setting.mpi5_clk = rt_flash_get_clk(MPI5_MEM_BASE);
#endif /* BSP_USING_PSRAM5 */
#endif /* BSP_ENABLE_MPI5 */

#endif  /* SF32LB55X */

    device = rt_device_find("lcd");
    if (device)
    {
        clk_setting.lcd_info.bandwidth = 0;
        if (device->open_flag & RT_DEVICE_OFLAG_OPEN)
        {
            err = rt_device_control(device, RTGRAPHIC_CTRL_GET_INFO, &clk_setting.lcd_info);
            RT_ASSERT(RT_EOK == err);
        }
    }
    else
    {
        memset(&clk_setting.lcd_info, 0, sizeof(clk_setting.lcd_info));
    }

    if (buf)  memset(buf, 0, buf_len);

    print_clk_str(buf, buf_len,   "HPSYS\nSYSCLK: "HUMAN_FORMAT"Hz\n", clk_setting.hpsys_clk.sysclk);
    print_clk_str(buf, buf_len,   "HCLK: "HUMAN_FORMAT"Hz\n", clk_setting.hpsys_clk.hclk);
    print_clk_str(buf, buf_len,   "PCLK1: "HUMAN_FORMAT"Hz\n", clk_setting.hpsys_clk.pclk1);
    print_clk_str(buf, buf_len,   "PCLK2: "HUMAN_FORMAT"Hz\n", clk_setting.hpsys_clk.pclk2);

    print_clk_str(buf, buf_len,   "\nLPSYS\nSYSCLK: "HUMAN_FORMAT"Hz\n", clk_setting.lpsys_clk.sysclk);
    print_clk_str(buf, buf_len,   "HCLK: "HUMAN_FORMAT"Hz\n", clk_setting.lpsys_clk.hclk);
    print_clk_str(buf, buf_len,   "PCLK1: "HUMAN_FORMAT"Hz\n", clk_setting.lpsys_clk.pclk1);
    print_clk_str(buf, buf_len,   "PCLK2: "HUMAN_FORMAT"Hz\n", clk_setting.lpsys_clk.pclk2);

    print_clk_str(buf, buf_len,   "\nBLESYS\nSYSCLK: "HUMAN_FORMAT"Hz\n", clk_setting.blesys_clk.sysclk);
    print_clk_str(buf, buf_len,   "HCLK: "HUMAN_FORMAT"Hz\n", clk_setting.blesys_clk.hclk);

#ifdef SF32LB55X
#if  ((defined BSP_USING_FLASH)||(defined BSP_USING_SPI_FLASH))
    print_clk_str(buf, buf_len,   "\nMemory\nFLASH1: "HUMAN_FORMAT"Hz\n", clk_setting.flash1_clk);
    print_clk_str(buf, buf_len,   "FLASH2: "HUMAN_FORMAT"Hz\n", clk_setting.flash2_clk);
#if ((defined BSP_ENABLE_FLASH3) || (defined BSP_ENABLE_QSPI3))
    print_clk_str(buf, buf_len,   "FLASH3: "HUMAN_FORMAT"Hz\n", clk_setting.flash3_clk);
#endif  /* BSP_ENABLE_FLASH3 */
#if ((defined BSP_ENABLE_FLASH4) || (defined BSP_ENABLE_QSPI4))
    print_clk_str(buf, buf_len,   "FLASH4: "HUMAN_FORMAT"Hz\n", clk_setting.flash4_clk);
#endif  /* BSP_ENABLE_FLASH4 */
#endif /* BSP_USING_NOR_FLASH */
#ifdef BSP_USING_PSRAM
    print_clk_str(buf, buf_len,   "PSRAM: "HUMAN_FORMAT"Hz\n", clk_setting.psram_clk);
#endif /* BSP_USING_PSRAM */

#else

    print_clk_str(buf, buf_len,   "\nMemory\n", 0);

#if  defined (BSP_ENABLE_MPI1)
    print_clk_str(buf, buf_len,   "MPI1: "HUMAN_FORMAT"Hz\n", clk_setting.mpi1_clk);
#endif /* BSP_ENABLE_MPI1 */
#if  defined (BSP_ENABLE_MPI2)
    print_clk_str(buf, buf_len,   "MPI2: "HUMAN_FORMAT"Hz\n", clk_setting.mpi2_clk);
#endif /* BSP_ENABLE_MPI2 */
#if  defined (BSP_ENABLE_MPI3)
    print_clk_str(buf, buf_len,   "MPI3: "HUMAN_FORMAT"Hz\n", clk_setting.mpi3_clk);
#endif /* BSP_ENABLE_MPI3 */
#if  defined (BSP_ENABLE_MPI4)
    print_clk_str(buf, buf_len,   "MPI4: "HUMAN_FORMAT"Hz\n", clk_setting.mpi4_clk);
#endif /* BSP_ENABLE_MPI4 */
#if  defined (BSP_ENABLE_MPI5)
    print_clk_str(buf, buf_len,   "MPI5: "HUMAN_FORMAT"Hz\n", clk_setting.mpi5_clk);
#endif /* BSP_ENABLE_MPI5 */

#endif /* SF32LB55X */

    /*********print LCD info ***********/
    print_clk_str(buf, buf_len,  "\nLCD\nBandwidth: "HUMAN_FORMAT"bps\n", clk_setting.lcd_info.bandwidth);
    print_var_str(buf, buf_len,  "%dx%d, align:%d\n", clk_setting.lcd_info.width,
                  clk_setting.lcd_info.height,
                  clk_setting.lcd_info.draw_align);
    if (clk_setting.lcd_info.is_round)
        print_var_str(buf, buf_len,  "round type,");
    else
        print_var_str(buf, buf_len,  "rect type,");

    switch (clk_setting.lcd_info.pixel_format)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        print_var_str(buf, buf_len,  "RGB565\n");
        break;
    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        print_var_str(buf, buf_len,  "RGB888\n");
        break;
    default:
        print_var_str(buf, buf_len,  "Other format\n");
        break;
    }
}

void *sifli_memset(void *s, int c, rt_ubase_t count)
{
#define LBLOCKSIZE      (sizeof(long))
#define UNALIGNED(X)    ((long)(X) & (LBLOCKSIZE - 1))
#define TOO_SMALL(LEN)  ((LEN) < LBLOCKSIZE)
#define EDMA_THD_SIZE       (256)
#define EDMA_MAX_SIZE       (0x400000)

    unsigned int i;
    char *m = (char *)s;
    unsigned long buffer;
    unsigned long *aligned_addr;
    unsigned int d = c & 0xff;  /* To avoid sign extension, copy C to an
                                unsigned variable.  */

    if (!TOO_SMALL(count) && !UNALIGNED(s))
    {
        /* If we get this far, we know that n is large and m is word-aligned. */
        aligned_addr = (unsigned long *)s;

        /* Store D into each char sized location in BUFFER so that
         * we can set large blocks quickly.
         */
        if (LBLOCKSIZE == 4)
        {
            buffer = (d << 8) | d;
            buffer |= (buffer << 16);
        }
        else
        {
            buffer = 0;
            for (i = 0; i < LBLOCKSIZE; i ++)
                buffer = (buffer << 8) | d;
        }
#if (defined(SOC_BF0_HCPU) && defined(BSP_USING_EXT_DMA))
        int fill, res, size;
        size = 0;
        while (count >= EDMA_THD_SIZE)
        {
            fill = count > EDMA_MAX_SIZE ?  EDMA_MAX_SIZE : (count / 4) << 2;
            res = EXT_DMA_Config(0, 1);
            if (res != 0)
                break;

            res = EXT_DMA_TRANS_SYNC((uint32_t)&buffer, (uint32_t)aligned_addr, fill / 4, 1000);
            if (res == 0)
            {
                aligned_addr += fill / 4;
                count -= fill;
                size += fill;
            }
            else
                break;
        }
        SCB_InvalidateDCache_by_Addr((void *)s, size);
        //SCB_InvalidateICache_by_Addr((void *)s, size);
#endif

        while (count >= LBLOCKSIZE * 4)
        {
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            count -= 4 * LBLOCKSIZE;
        }

        while (count >= LBLOCKSIZE)
        {
            *aligned_addr++ = buffer;
            count -= LBLOCKSIZE;
        }

        /* Pick up the remainder with a bytewise loop. */
        m = (char *)aligned_addr;
    }

    while (count--)
    {
        *m++ = (char)d;
    }

    return s;

#undef LBLOCKSIZE
#undef UNALIGNED
#undef TOO_SMALL
#undef EDMA_THD_SIZE
#undef EDMA_MAX_SIZE
}

#if (defined(SOC_BF0_HCPU) && defined(BSP_USING_EXT_DMA))
static void dma_err_cb()
{
    RT_ASSERT(0);
}
#endif /* SOC_BF0_HCPU && BSP_USING_EXT_DMA */

void *sifli_memcpy(void *dst, const void *src, rt_ubase_t count)
{
#define UNALIGNED(X, Y) \
    (((long)(X) & (sizeof (long) - 1)) | ((long)(Y) & (sizeof (long) - 1)))
#define BIGBLOCKSIZE    (sizeof (long) << 2)
#define LITTLEBLOCKSIZE (sizeof (long))
#define TOO_SMALL(LEN)  ((LEN) < BIGBLOCKSIZE)
#define EDMA_THD_SIZE       (256)
#define EDMA_MAX_SIZE       (0x400000)

    char *dst_ptr = (char *)dst;
    char *src_ptr = (char *)src;
    long *aligned_dst;
    long *aligned_src;
    int len = count;

    /* If the size is small, or either SRC or DST is unaligned,
    then punt into the byte copy loop.  This should be rare. */
    if (!TOO_SMALL(len) && !UNALIGNED(src_ptr, dst_ptr))
    {
        aligned_dst = (long *)dst_ptr;
        aligned_src = (long *)src_ptr;

#if (defined(SOC_BF0_HCPU) && defined(BSP_USING_EXT_DMA))
        int fill, res;
        while (len >= EDMA_THD_SIZE)
        {
            fill = len > EDMA_MAX_SIZE ?  EDMA_MAX_SIZE : (len / 4) << 2;
            res = EXT_DMA_Config(1, 1);
            if (res != 0)
                break;

            EXT_DMA_Register_Callback(EXT_DMA_XFER_CPLT_CB_ID, NULL);
            EXT_DMA_Register_Callback(EXT_DMA_XFER_ERROR_CB_ID, dma_err_cb);

            res = EXT_DMA_START_ASYNC((uint32_t)aligned_src, (uint32_t)aligned_dst, fill / 4);
            if (res == 0)
            {
                aligned_dst += fill / 4;
                aligned_src += fill / 4;
                len -= fill;

                EXT_DMA_Wait_ASYNC_Done();
            }
            else
                break;
        }
        SCB_InvalidateDCache_by_Addr((void *)dst, count - len);
        //SCB_InvalidateICache_by_Addr((void *)dst, count - len);
#endif
        /* Copy 4X long words at a time if possible. */
        while (len >= BIGBLOCKSIZE)
        {
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            len -= BIGBLOCKSIZE;
        }

        /* Copy one long word at a time if possible. */
        while (len >= LITTLEBLOCKSIZE)
        {
            *aligned_dst++ = *aligned_src++;
            len -= LITTLEBLOCKSIZE;
        }

        /* Pick up any residual with a byte copier. */
        dst_ptr = (char *)aligned_dst;
        src_ptr = (char *)aligned_src;
    }

    while (len--)
        *dst_ptr++ = *src_ptr++;

    return dst;
#undef UNALIGNED
#undef BIGBLOCKSIZE
#undef LITTLEBLOCKSIZE
#undef TOO_SMALL
#undef EDMA_THD_SIZE
#undef EDMA_MAX_SIZE
}

HAL_RAM_RET_CODE_SECT(sifli_record_module,  void sifli_record_module(uint16_t module))
{
#if defined(SOC_BF0_HCPU) && defined(USING_MODULE_RECORD)
    volatile uint32_t *p = &hwp_rtc->BKP0R + RTC_BACKUP_MODULE_RECORD;
    if (*p & RECORD_CRASH_FLASG_MASK)
    {
        return;
    }

    MODIFY_REG(*p, RECORD_MODULE_MASK, MAKE_REG_VAL(module, RECORD_MODULE_MASK, RECORD_MODULE_POS));
#endif
    return;
}

HAL_RAM_RET_CODE_SECT(sifli_record_psram_half_status,  void sifli_record_psram_half_status(uint8_t status))
{
#if defined(SOC_BF0_HCPU) && defined(USING_MODULE_RECORD)
    volatile uint32_t *p = &hwp_rtc->BKP0R + RTC_BACKUP_MODULE_RECORD;
    if (*p & RECORD_CRASH_FLASG_MASK)
    {
        return;
    }

    MODIFY_REG(*p, RECORD_PSRAM_HALF_STATUS_MASK, MAKE_REG_VAL(status, RECORD_PSRAM_HALF_STATUS_MASK,
               RECORD_PSRAM_HALF_STATUS_POS));
#endif
    return;
}

HAL_RAM_RET_CODE_SECT(sifli_record_crash_status,  void sifli_record_crash_status(uint8_t status))
{
#if defined(SOC_BF0_HCPU) && defined(USING_MODULE_RECORD)
    volatile uint32_t *p = &hwp_rtc->BKP0R + RTC_BACKUP_MODULE_RECORD;

    MODIFY_REG(*p, RECORD_CRASH_FLASG_MASK, MAKE_REG_VAL(status, RECORD_CRASH_FLASG_MASK,
               RECORD_CRASH_FLASG_POS));
#endif
    return;
}

HAL_RAM_RET_CODE_SECT(sifli_record_crash_save_process,  void sifli_record_crash_save_process(uint8_t value))
{
#if defined(SOC_BF0_HCPU) && defined(USING_MODULE_RECORD)
    volatile uint32_t *p = &hwp_rtc->BKP0R + RTC_BACKUP_MODULE_RECORD;

    MODIFY_REG(*p, RECORD_CRASH_SAVE_PROCESS_MASK, MAKE_REG_VAL(value, RECORD_CRASH_SAVE_PROCESS_MASK,
               RECORD_CRASH_SAVE_PROCESS_POS));
#endif
    return;
}

HAL_RAM_RET_CODE_SECT(sifli_record_wdt_irq_status,  void sifli_record_wdt_irq_status(uint8_t value))
{
#if defined(SOC_BF0_HCPU) && defined(USING_MODULE_RECORD)
    volatile uint32_t *p = &hwp_rtc->BKP0R + RTC_BACKUP_MODULE_RECORD;

    MODIFY_REG(*p, RECORD_WDT_IRQ_FLAG_MASK, MAKE_REG_VAL(value, RECORD_WDT_IRQ_FLAG_MASK,
               RECORD_WDT_IRQ_FLAG_POS));
#endif
    return;
}

void sifli_record_clear(void)
{
#if defined(SOC_BF0_HCPU) && defined(USING_MODULE_RECORD)
    volatile uint32_t *p = &hwp_rtc->BKP0R + RTC_BACKUP_MODULE_RECORD;
    *p = 0;
#endif
    return;
}

#if defined(FINSH_USING_MSH)
static char *cmd_line(int argc, char **argv)
{
    char *r = NULL, *p;
    int i;

    if (argc > 1)
    {
        r = argv[1];
        p = r + strlen(r);
        for (i = 2; i < argc; i++)
        {
            while (*p == '\0' && p < argv[i])
                *p++ = ' ';
            p += strlen(argv[i]);
        }
        strcat(r, "\n");
    }
    return r;
}

#if defined(SOC_BF0_HCPU) && !defined(CFG_BOOTLOADER)
uint8_t lcpu_power_on(void);
uint8_t lcpu_power_off(void);

__ROM_USED int lcpu(int argc, char **argv)
{
    if (argc > 1)
    {

        if (strcmp("on", argv[1]) == 0)
        {
            lcpu_power_on();
        }
        else if (strcmp("off", argv[1]) == 0)
        {
            //HAL_RCC_LCPU_reset(LPSYS_RCC_RSTR1_LCPU, 1);    // Enable reset
            lcpu_power_off();
        }
        else if (strcmp("sleep", argv[1]) == 0)
        {
            HAL_LPAON_Sleep();
        }
        else if (strcmp("wakeup", argv[1]) == 0)
        {
            uint8_t is_on = (uint8_t)atoi(argv[2]);
            if (is_on)
                HAL_HPAON_WakeCore(CORE_ID_LCPU);
            else
                HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
        }
        else
        {
            char *s = cmd_line(argc, argv);
            if (s)
            {
#ifdef USING_IPC_QUEUE
                if (IPC_QUEUE_INVALID_HANDLE != sys_hl_ipc_queue)
                {
                    ipc_queue_write(sys_hl_ipc_queue, (uint8_t *)s, strlen(s), 10);
                }
#endif /* USING_IPC_QUEUE */
            }
        }
    }
    return 0;
}
MSH_CMD_EXPORT(lcpu, forward lcpu command);

#endif

__ROM_USED int sysinfo(int argc, char **argv)
{
    rt_kprintf("--------System information---------\n");
    print_sysinfo(NULL, 0);
    rt_kprintf("\n");
    return 0;
}

MSH_CMD_EXPORT(sysinfo, Show system information);

#endif

#endif

/** @} */

