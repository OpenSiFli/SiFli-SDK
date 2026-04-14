/*
 * SPDX-FileCopyrightText: 2025-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>
#include <string.h>
#ifdef RWBT_ENABLE
    #include "rwip.h"
#endif
#include "board.h"
#include "ipc_queue.h"
#include "bf0_mbox_common.h"
#include "bf0_sibles_nvds.h"

#define BLE_HCI_FORWARD_PORT "uart1"

#define IO_MB_CH      (0)
#define TX_BUF_SIZE   HCPU2LCPU_MB_CH1_BUF_SIZE
#define TX_BUF_ADDR   HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);

#define RX_BUF_REV_B_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);

typedef struct
{
    ipc_queue_handle_t ipc_port;
    rt_device_t uart_port;
    rt_mailbox_t to_mb;
    rt_mailbox_t to_uart;
} ble_forward_env_t;

void wvt_local_hdl_entry(void *param);

static ble_forward_env_t g_ble_forward_env;
static rt_device_t g_ble_forward_device;
static rt_event_t g_ble_forward_evt;

static void ble_wvt_mailbox_init(void);
static void ble_wvt_uart_init(void);

#include "log.h"

/** Mount file system if using NAND, as BT NVDS is save in file*/
#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS) && !defined(ZBT)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "root"
int mnt_init(void)
{
    //TODO: how to get base address
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000), FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)), FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

static ble_forward_env_t *ble_forward_get_env(void)
{
    return &g_ble_forward_env;
}

static rt_err_t ble_wvt_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    ble_forward_env_t *env = ble_forward_get_env();
    rt_mb_send(env->to_mb, size);

    return 0;
}

static int32_t ble_wvt_mb_ind(ipc_queue_handle_t dev, size_t size)
{
    ble_forward_env_t *env = ble_forward_get_env();
    rt_mb_send(env->to_uart, size);

    return 0;
}

void hci_ipc_queue_write(uint8_t *ptr, uint32_t size)
{
    int written, offset = 0;
    ble_forward_env_t *env = ble_forward_get_env();

    if (IPC_QUEUE_INVALID_HANDLE != env->ipc_port)
    {
        rt_kprintf("Write to MB %d", size);
        written = ipc_queue_write(env->ipc_port, ptr, size, 10);
        while (written < size)
        {
            size -= written;
            offset += written;
            written = ipc_queue_write(env->ipc_port, ptr + offset, size, 10);
        }
        rt_kprintf("Written to MB %d", written);
    }
}

void ble_wvt_forward_to_mb_entry(void *param)
{
    ble_forward_env_t *env = ble_forward_get_env();
    //env->to_mb = rt_mb_create("fwd_mb", 16, RT_IPC_FLAG_FIFO);
    //ble_wvt_uart_init();
    rt_uint32_t size;
    uint8_t *ptr;
    int written, offset = 0;
    rt_size_t read_len;
    while (1)
    {
        rt_mb_recv(env->to_mb, &size, RT_WAITING_FOREVER);
        rt_kprintf("(TB)read size %d, mb ptr %x\r\n", size, env->ipc_port);
        if (!size)
            continue;
        ptr = rt_malloc(size);
        RT_ASSERT(ptr);

        // Read from uart
        read_len = rt_device_read(env->uart_port, 0, ptr, size);
        if (read_len == 0)
        {
            rt_free(ptr);
            continue;
        }
        if (read_len < size)
            size = read_len;

        rt_hexdump("hci_tob", 32, ptr, size);
        // Write to mailbox
        HAL_DBG_print_data((char *)ptr, 0, size);

        if (IPC_QUEUE_INVALID_HANDLE != env->ipc_port)
        {
            rt_kprintf("Write to MB %d\n", size);
            written = ipc_queue_write(env->ipc_port, ptr, size, 10);
            while (written < size)
            {
                size -= written;
                offset += written;
                written = ipc_queue_write(env->ipc_port, ptr + offset, size, 10);
            }
            rt_kprintf("Written to MB %d\n", written);
        }

        offset = 0;
        rt_free(ptr);
    }

}

static void ble_wvt_ble_power_on(void)
{
    // If RSTR ON, just clear.
    rt_kprintf("sw ver:1.2.13\n");
    sifli_nvds_init();
#if defined(BSP_BLE_SIBLES) && !defined(SF32LB55X)
    extern void bt_stack_nvds_init(void);
    bt_stack_nvds_init();
#endif

    lcpu_power_on();
    HAL_Delay(500);

    //CLEAR_BIT(hwp_pmuc->HRC_CR, PMUC_HRC_CR_OUT_EN);
#if 0
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_TICK_HXT48);
    CLEAR_BIT(hwp_pmuc->HRC_CR, PMUC_HRC_CR_EN);
    SysTick->CTRL  &= (SysTick_CTRL_TICKINT_Msk   | SysTick_CTRL_ENABLE_Msk);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_TICK, RCC_CLK_TICK_HXT48);
    HAL_SYSTICK_Config(800000 / RT_TICK_PER_SECOND);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_TICK_CLK);
#endif
}

static void ble_wvt_patch_install(void)
{
#ifdef BSP_USING_BCPU_PATCH
    extern void bcpu_patch_install();
    bcpu_patch_install();
#else
    //uint32_t *p = (uint32_t *)(BCPU_PATCH_CODE_START_ADDR + BCPU2HCPU_OFFSET);
    //*p = 0;
#endif
}

void ble_wvt_forward_to_uart_entry(void *param)
{
    ble_forward_env_t *env = ble_forward_get_env();
    rt_uint32_t size;
    uint8_t *ptr;
    int written;
    rt_size_t read_len;
    while (1)
    {
        rt_mb_recv(env->to_uart, &size, RT_WAITING_FOREVER);
        rt_kprintf("(TU)read size %d, uart status %x\r\n", size, env->uart_port);
        if (!size)
            continue;
        ptr = rt_malloc(size);
        RT_ASSERT(ptr);

        // Read from mailbox
        read_len = ipc_queue_read(env->ipc_port, ptr, size);
        if (read_len < size)
            size = read_len;
        HAL_DBG_print_data((char *)ptr, 0, size);
        rt_hexdump("hci_tou", 32, ptr, size);

        // Write to mailbox
        if (env->uart_port)
        {
            written = rt_device_write(env->uart_port, 0, ptr, size);
            RT_ASSERT(size == written);
        }
        rt_free(ptr);
    }

}

static void ble_wvt_mailbox_init(void)
{
    rt_err_t result;
    rt_thread_t tid;
    ble_forward_env_t *env =  ble_forward_get_env();
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = IO_MB_CH;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
#ifndef SF32LB52X
    /* Config IPC queue. */
    q_cfg.rx_buf_addr = RX_BUF_ADDR;

#else // SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
#if !defined(SF32LB52X_REV_B)
        q_cfg.rx_buf_addr = RX_BUF_ADDR;
#endif // !defined(SF32LB52X_REV_B)
    }
    else
    {
#if (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)) && defined(BF0_HCPU)
        q_cfg.rx_buf_addr = RX_BUF_REV_B_ADDR;
#endif // (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)) && defined(BF0_HCPU)
    }
#endif // !SF32LB52X
    q_cfg.rx_ind = ble_wvt_mb_ind;
    q_cfg.user_data = 0;

    env->ipc_port = ipc_queue_init(&q_cfg);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != env->ipc_port);
    RT_ASSERT(0 == ipc_queue_open(env->ipc_port));
}

static void ble_wvt_uart_init(void)
{
    rt_err_t result;
    rt_uint16_t oflag;
    ble_forward_env_t *env =  ble_forward_get_env();

    env->uart_port = rt_device_find(BLE_HCI_FORWARD_PORT);
    if (env->uart_port)
    {
        //oflag = RT_DEVICE_OFLAG_RDWR;
        oflag = RT_DEVICE_OFLAG_RDWR;

        if (env->uart_port->flag & RT_DEVICE_FLAG_DMA_RX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_RX;
        }

        result = rt_device_open(env->uart_port, oflag);
        RT_ASSERT(RT_EOK == result);

        rt_device_set_rx_indicate(env->uart_port, ble_wvt_uart_rx_ind);
    }

}

// init both uart and mailbox
int ble_wvt_forward_init(void)
{
    rt_thread_t tid;
    //HAL_sw_breakpoint();
    // Forward data to MB
    ble_forward_env_t *env = ble_forward_get_env();
    env->to_mb = rt_mb_create("fwd_mb", 16, RT_IPC_FLAG_FIFO);
    ble_wvt_uart_init();
    env->to_uart = rt_mb_create("fwd_uart", 16, RT_IPC_FLAG_FIFO);
    ble_wvt_mailbox_init();
    ble_wvt_patch_install();
    ble_wvt_ble_power_on();

    tid = rt_thread_create("fwd_mb", ble_wvt_forward_to_mb_entry, NULL, 2048, 15, 10);
    rt_thread_startup(tid);
    // Forward data to uart
    tid = rt_thread_create("fwd_uart", ble_wvt_forward_to_uart_entry, NULL, 4096, 15, 10);
    rt_thread_startup(tid);

    // Avoid LCPU enter sleep mode to access its UART
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    return 0;
}

INIT_APP_EXPORT(ble_wvt_forward_init);

int main(void)
{

    while (1)
    {
        //rt_kprintf("test \n");
        rt_thread_mdelay(3600000);
    }
}

