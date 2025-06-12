/**
 ******************************************************************************
 * @file   main.c
 * @author Sifli software development team
 ******************************************************************************
 */
/**
 * @attention
 * Copyright (c) 2025 - 2025,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated
 * circuit in a product or a software update for such product, must reproduce
 * the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#ifdef RWBT_ENABLE
    #include "rwip.h"
#endif
#include "bf0_mbox_common.h"
#include "bf0_sibles_nvds.h"
#include "board.h"
#include "ipc_queue.h"

#define IO_MB_CH (0)
#define TX_BUF_SIZE HCPU2LCPU_MB_CH1_BUF_SIZE
#define TX_BUF_ADDR HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define TX_BUF_ADDR_ALIAS                                                      \
    HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_ADDR LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);

#define RX_BUF_REV_B_ADDR                                                      \
    LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);

ipc_queue_handle_t ipc_port;

rt_mailbox_t to_btstack;

static void ble_wvt_mailbox_init(void);

#include "log.h"

/** Mount file system if using NAND, as BT NVDS is save in file*/
#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS) && !defined(ZBT)
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #include "drv_flash.h"
    #define NAND_MTD_NAME "root"
int mnt_init(void)
{
    // TODO: how to get base address
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR -
                             (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
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
uint8_t btstack_running = 0;
int send_to_malibox(uint8_t *ptr, int size)
{
    int written, offset = 0;
    if (IPC_QUEUE_INVALID_HANDLE != ipc_port)
    {
        // rt_kprintf("Write to MB %d\n", size);
        written = ipc_queue_write(ipc_port, ptr, size, 10);
        while (written < size)
        {
            size -= written;
            offset += written;
            written = ipc_queue_write(ipc_port, ptr + offset, size, 10);
        }
        // rt_kprintf("Written to MB %d\n", written);
    }
    return 0;
}

void btstack_run_loop_poll_data_sources_from_irq();
extern uint8_t *hci_tmp;
static int32_t ble_wvt_mb_ind(ipc_queue_handle_t dev, size_t size)
{
    // rt_kprintf("ble_wvt_mb_ind\n");
    if (btstack_running)
    {
        btstack_run_loop_poll_data_sources_from_irq();
        rt_mb_send(to_btstack, size);
    }
    else
    {
        ipc_queue_read(ipc_port, hci_tmp, size);
    }
    return 0;
}

static void ble_wvt_ble_power_on(void)
{
    // If RSTR ON, just clear.
    rt_kprintf("sw ver:1.2.13\n");
    sifli_nvds_init();
    // rt_kprintf("ble_wvt_forward_init\n");
#if defined(BSP_BLE_SIBLES) && !defined(SF32LB55X)
    extern void bt_stack_nvds_init(void);
    bt_stack_nvds_init();
#endif

    lcpu_power_on();
    HAL_Delay(500);

    // CLEAR_BIT(hwp_pmuc->HRC_CR, PMUC_HRC_CR_OUT_EN);
#if 0
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_TICK_HXT48);
    CLEAR_BIT(hwp_pmuc->HRC_CR, PMUC_HRC_CR_EN);
    SysTick->CTRL  &= (SysTick_CTRL_TICKINT_Msk   | SysTick_CTRL_ENABLE_Msk);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_TICK, RCC_CLK_TICK_HXT48);
    HAL_SYSTICK_Config(800000 / RT_TICK_PER_SECOND);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
#endif
}

static void ble_wvt_patch_install(void)
{
#ifdef BSP_USING_BCPU_PATCH
    extern void bcpu_patch_install();
    bcpu_patch_install();
#else
    // uint32_t *p = (uint32_t *)(BCPU_PATCH_CODE_START_ADDR +
    // BCPU2HCPU_OFFSET); *p = 0;
#endif
}

static void ble_wvt_mailbox_init(void)
{
    rt_err_t result;
    rt_thread_t tid;
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
    #if (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)) &&           \
        defined(BF0_HCPU)
        q_cfg.rx_buf_addr = RX_BUF_REV_B_ADDR;
    #endif // (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)) &&
           // defined(BF0_HCPU)
    }
#endif     // !SF32LB52X
    q_cfg.rx_ind = ble_wvt_mb_ind;
    q_cfg.user_data = 0;

    ipc_port = ipc_queue_init(&q_cfg);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != ipc_port);
    RT_ASSERT(0 == ipc_queue_open(ipc_port));
}

// init mailbox
int ble_wvt_forward_init(void)
{
    rt_thread_t tid;
    // HAL_sw_breakpoint();
    //  Forward data to MB
    to_btstack = rt_mb_create("fwd_btstack", 16, RT_IPC_FLAG_FIFO);
    ble_wvt_mailbox_init();
    ble_wvt_patch_install();
    ble_wvt_ble_power_on();

    // Avoid LCPU enter sleep mode to access its UART
    HAL_HPAON_WakeCore(CORE_ID_LCPU);

    return 0;
}

INIT_APP_EXPORT(ble_wvt_forward_init);
void btstack_thread(void *args);
int main(void)
{
    btstack_running = 1;
    btstack_thread(NULL);
    while (1)
    {
        // rt_kprintf("test \n");
        rt_thread_mdelay(3600000);
    }
}