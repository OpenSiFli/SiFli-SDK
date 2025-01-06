/**
  ******************************************************************************
  * @file   bluetooth.c
  * @author Sifli software development team
  * @brief SIFLI Bluetooth stack external implementation.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2020 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string.h>
#include "rtthread.h"
#include "os_adaptor.h"
#include "board.h"
#include "ble_stack.h"
#include "log.h"
#ifdef BSP_BLE_SIBLES
    #include "bf0_sibles_nvds.h"
    #include "bf0_sibles_internal.h"
    #ifdef BSP_BLE_CONNECTION_MANAGER
        #include "ble_connection_manager.h"
    #endif
#endif
#include "bluetooth_int.h"

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif


#if defined(BF0_LCPU) && defined(BSP_USING_DATA_SVC)
    #include "data_service.h"
#endif

#ifndef BSP_USING_PC_SIMULATOR
    #ifdef SOC_BF0_HCPU
        #define BTS_READ_GTIMER()  HAL_HPAON_READ_GTIMER()
    #elif defined(SOC_BF0_LCPU)
        #define BTS_READ_GTIMER()  HAL_LPAON_READ_GTIMER()
    #else
        #error "Invalid config"
    #endif /* SOC_BF0_HCPU */
#endif


//#define STACK_DEBUG

//#define FPGA_debug

#ifdef STACK_DEBUG

    #if defined(SOC_SF32LB55X)
        #define LPSYS_DEBUG_SWITCH_ADDRESS 0x4004F014
        #define LPSYS_BLE_DEBUG_SWITCH_ADDRESS 0x50050050
        #define LPSYS_DEBUG_SWITCH_BLE_PATTER 0xFF03FF03
    #elif (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X))
        #define LPSYS_DEBUG_SWITCH_ADDRESS 0x5000F014
        #define LPSYS_BLE_DEBUG_SWITCH_ADDRESS 0x50090850
        #define LPSYS_DEBUG_SWITCH_BLE_PATTER 0xFF05
    #else
        #define LPSYS_DEBUG_SWITCH_ADDRESS 0x4004F014
        #define LPSYS_BLE_DEBUG_SWITCH_ADDRESS 0x50050050
        #define LPSYS_DEBUG_SWITCH_BLE_PATTER 0xFF03FF03
    #endif

#endif



typedef void (*patch_init_handler)(void);



#ifdef SOC_SF32LB55X
    static uint32_t g_pa_ctrl_val;
#endif

#if defined(RT_USING_PM) && (defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && defined(BF0_LCPU)
    static struct rt_device g_ble_mac_dev;
#endif // defined(RT_USING_PM) && defined(SOC_SF32LB56X)

static void ble_xtal_less_init(void)
{
#ifndef BSP_USING_PC_SIMULATOR
    if (!HAL_LXT_ENABLED())
    {
        // RC10K should change the stack configuration.
        rom_config_set_default_xtal_enabled(0);
#ifdef SOC_SF32LB55X
        rom_config_set_default_rc_cycle(HAL_RC_CAL_GetLPCycle());
#else
        rom_config_set_default_rc_cycle(HAL_RC_CAL_GetLPCycle_ex());
#endif
    }
    rom_config_set_lld_prog_delay(2);
#endif //BSP_USING_PC_SIMULATOR
}


#ifdef SOC_SF32LB56X
    #ifdef BSP_USING_LCPU_PATCH_IN_RAM
        #define PATCH_ADDR LCPU_PATCH_ROM_RAM_START_ADDR
    #else
        #define PATCH_ADDR LCPU_PATCH_START_ADDR
    #endif
#else
    #define PATCH_ADDR LCPU_PATCH_START_ADDR
#endif

volatile patch_init_handler g_patch_handler;
static void patch_install(void)
{
#if defined(BF0_LCPU) && (!defined(FPGA))
    g_patch_handler = (patch_init_handler)(PATCH_ADDR + 1);
    if ((*(uint32_t *)PATCH_ADDR) != 0)
        g_patch_handler();
#endif
}

#ifdef FPGA_debug
#include "ke_task.h"
int32_t ble_event_process(uint16_t const msgid, void const *param,
                          uint16_t const dest_id, uint16_t const src_id)
{
    return 0;
}
#endif

#if (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && (defined(SOC_BF0_LCPU)) && (!defined(BSP_USING_PC_SIMULATOR))

extern void ble_isr_rom(void);
extern void bt_isr_rom(void);
extern void dm_isr_rom(void);
extern void dm_err_check_isr_rom();


void dm_err_check_clr()
{
    static uint32_t dmintstat0 = 0;
    static uint32_t iperrtype = 0;
    static uint32_t dmerrcnt = 0;//for debug

    dmintstat0 = *(volatile uint32_t *)(BT_MAC_BASE + 0x10);

    if (dmintstat0 & ((uint32_t)0x00010000))
    {
        uint32_t *intack0 = (uint32_t *)(BT_MAC_BASE + 0x14);

        iperrtype = *(volatile uint32_t *)(BT_MAC_BASE + 0x60);

        *intack0 = 0x00010000;
        dmerrcnt++;
        RT_ASSERT(0 == (iperrtype & 0xd));
    }
}
#ifndef SOC_SF32LB52X
void ble_isr(void)
{
    rt_base_t level;
    rt_interrupt_enter();
    level = rt_hw_interrupt_disable();
    ble_isr_rom();
    rt_hw_interrupt_enable(level);
    rt_interrupt_leave();
}

void bt_isr(void)
{
    rt_base_t level;
    rt_interrupt_enter();
    level = rt_hw_interrupt_disable();
    bt_isr_rom();
    rt_hw_interrupt_enable(level);
    rt_interrupt_leave();

}
#endif
void dm_isr(void)
{
    rt_base_t level;
    rt_interrupt_enter();
    level = rt_hw_interrupt_disable();
    dm_isr_rom();
    dm_err_check_clr();
    rt_hw_interrupt_enable(level);
    rt_interrupt_leave();
}

#endif

#if defined(BF0_HCPU) && defined(SOC_SF32LB52X)
void bluetooth_wakeup_lcpu(void)
{
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
}

void bluetooth_release_lcpu(void)
{
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
}
#endif


__ROM_USED void rt_hw_ble_int_init(void)
{

#if (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && (defined(SOC_BF0_LCPU)) && (!defined(BSP_USING_PC_SIMULATOR))
    HAL_NVIC_SetPriority(BLE_MAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BLE_MAC_IRQn);

    HAL_NVIC_SetPriority(BT_MAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BT_MAC_IRQn);

    HAL_NVIC_SetPriority(DM_MAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DM_MAC_IRQn);
#endif
}


static void bluetooth_isr_init(void)
{
#ifndef BSP_USING_PC_SIMULATOR
    rt_hw_ble_int_init();
#endif
}


#if defined(SOC_SF32LB55X) && !defined(BF0_HCPU)
RT_USED int32_t ble_standby_sleep_pre_handler()
{
    g_pa_ctrl_val = (hwp_ble_phy->TX_PA_CFG & BLE_PHY_TX_PA_CFG_PA_CTRL_TARGET_Msk) >> BLE_PHY_TX_PA_CFG_PA_CTRL_TARGET_Pos;
    return ble_standby_sleep_pre_handler_rom();
};

RT_USED void ble_standby_sleep_after_handler()
{
    hwp_ble_phy->TX_PA_CFG &= ~BLE_PHY_TX_PA_CFG_PA_CTRL_TARGET_Msk;
    hwp_ble_phy->TX_PA_CFG |= g_pa_ctrl_val << BLE_PHY_TX_PA_CFG_PA_CTRL_TARGET_Pos;
    ble_standby_sleep_after_handler_rom();
}
#endif

#if defined(RT_USING_PM) && (defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && defined(BF0_LCPU)

__WEAK int bluetooth_stack_suspend(void)
{
    return -RT_ERROR;
}


__WEAK void bluetooth_idle_hook_func(void)
{

}



static int bluetooth_pm_suspend(const struct rt_device *device, uint8_t mode)
{
    int ret = RT_EOK;
    if (((hwp_lpsys_aon->WER & LPSYS_AON_WER_BT) != 0) && (device == &g_ble_mac_dev))
    {
        if (mode >= PM_SLEEP_MODE_LIGHT)
        {
            ret = bluetooth_stack_suspend();
        }
        else
        {
            ret = -RT_ERROR;
            LOG_E("MAC sleep not support yet (%d)", mode);
        }
    }
    return ret;
}

#ifdef SOC_SF32LB52X
static uint8_t bluetooth_select_pm_mode(rt_tick_t tick)
{
    uint8_t mode = PM_SLEEP_MODE_IDLE;
    if (tick == RT_TICK_MAX &&
            bluetooth_stack_suspend() == RT_EOK)
    {
#ifdef PM_STANDBY_ENABLE
        mode = PM_SLEEP_MODE_STANDBY;
#elif defined(PM_DEEP_ENABLE)
        mode = PM_SLEEP_MODE_DEEP;
#else
        mode = PM_SLEEP_MODE_IDLE;
#endif
    }

    return mode;
}
#endif // SOC_SF32LB52X

const static struct rt_device_pm_ops g_ble_mac_pm_ops =
{
    .suspend = bluetooth_pm_suspend,
};




static void bluetooth_pm_init(void)
{
    rt_device_t device = &g_ble_mac_dev;

    device->type        = RT_Device_Class_PM;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    rt_pm_device_register(device, &g_ble_mac_pm_ops);
    rt_thread_idle_sethook(bluetooth_idle_hook_func);
#ifdef SOC_SF32LB52X
    rt_pm_override_mode_select(bluetooth_select_pm_mode);
#endif

}
#endif // defined(RT_USING_PM) && defined(SOC_SF32LB56X) && defined(BF0_LCPU)

// Standby handle for PRO
#if (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && (defined(SOC_BF0_LCPU)) && (!defined(BSP_USING_PC_SIMULATOR))

#ifndef SOC_SF32LB56X
    #define SPI_TEMP_LCPU_ADDRESS 0x204FA000
#else
    #define SPI_TEMP_LCPU_ADDRESS 0x2041F300
#endif


#if defined(FPGA)
#include "spi.h"

static uint32_t spi_addr;


static void ad9364_spi_init()
{
#if defined(SOC_SF32LB58X)
    hwp_pinmux2->PAD_PB21 = 0x53; //clk
    hwp_pinmux2->PAD_PB40 = 0x53; //cs
    hwp_pinmux2->PAD_PB22 = 0x53; //di
    hwp_pinmux2->PAD_PB25 = 0x53; //do/dio
#elif defined(SOC_SF32LB56X)
    hwp_pinmux2->PAD_PB19 = 0x51; //clk
    hwp_pinmux2->PAD_PB18 = 0x51; //cs
    hwp_pinmux2->PAD_PB20 = 0x51; //di
    hwp_pinmux2->PAD_PB21 = 0x51; //do/dio
#elif defined(SOC_SF32LB52X)
    hwp_pinmux1->PAD_PA29 = 0x52; //clk
    hwp_pinmux1->PAD_PA24 = 0x52; //cs
    hwp_pinmux1->PAD_PA25 = 0x52; //di
    hwp_pinmux1->PAD_PA28 = 0x52; //do/dio
#endif

#ifdef SOC_SF32LB52X
    hwp_spi1->TOP_CTRL &= ~SPI_TOP_CTRL_DSS_Msk;
    hwp_spi1->TOP_CTRL |= ((24 - 1) << SPI_TOP_CTRL_DSS_Pos);

    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SPH;

    hwp_spi1->TOP_CTRL |= SPI_TOP_CTRL_SSE;

    hwp_spi1->CLK_CTRL &= (~SPI_CLK_CTRL_CLK_DIV_Msk);
    hwp_spi1->CLK_CTRL |= (0x4 << SPI_CLK_CTRL_CLK_DIV_Pos);

    hwp_spi1->CLK_CTRL |= 0x01 << SPI_CLK_CTRL_CLK_SSP_EN_Pos;
#else

    //24bit data width
    hwp_spi3->TOP_CTRL &= ~SPI_TOP_CTRL_DSS_Msk;
    hwp_spi3->TOP_CTRL |= (23 << SPI_TOP_CTRL_DSS_Pos);

    //clk cfg
    //div 2 by default

    //sclk phase to 1, polarity stay 0
    //sclk is 0 when idle, and lanch data at posedge
    hwp_spi3->TOP_CTRL |= SPI_TOP_CTRL_SPH;

    //enable enable spi3
    hwp_spi3->TOP_CTRL |= SPI_TOP_CTRL_SSE;
#endif

}

#ifdef SOC_SF32LB52X
void ad9364_bt_cfg()
{
    //initialization
    hwp_spi1->FIFO_CTRL |= SPI_FIFO_CTRL_TSRE;
    hwp_spi1->INTE |= SPI_INTE_TIE;
    hwp_dmac1->CCR5 &= ~DMAC_CCR5_EN;
    hwp_dmac1->CCR4 &= ~DMAC_CCR4_EN;

    //memcpy((uint8_t *)SPI_TEMP_LCPU_ADDRESS, (uint8_t *)&spi_9364_table, sizeof(spi_9364_table));
    //hwp_bt_mac->DMRADIOCNTL4 = ((uint32_t)(&spi_9364_table) + 0x0a000000);
    //hwp_bt_mac->DMRADIOCNTL4 = SPI_TEMP_LCPU_ADDRESS;
//#ifdef SOC_BF0_HCPU
//    hwp_bt_mac->DMRADIOCNTL4 = ((uint32_t)(&spi_9364_table) + 0x0a000000);
//#else
    hwp_bt_mac->DMRADIOCNTL4 = spi_addr;
//#endif
    hwp_dmac1->CM0AR5 = (uint32_t) & (hwp_bt_mac->DMRADIOCNTL4);
    hwp_dmac1->CPAR5 = (uint32_t) & (hwp_dmac1->CM0AR4);
    hwp_dmac1->CNDTR5 = 0;
    hwp_dmac1->CCR5 = DMAC_CCR5_DIR | DMAC_CCR5_MINC | DMAC_CCR5_MEM2MEM;
    hwp_dmac1->CCR5 |= (0x2 << DMAC_CCR5_MSIZE_Pos);
    hwp_dmac1->CCR5 |= (0x2 << DMAC_CCR5_PSIZE_Pos);
    hwp_dmac1->CCR5 |= DMAC_CCR5_EN;
    //dmac2 hannel 4 for spi3 tx
    hwp_dmac1->CM0AR4 = 0;
    hwp_dmac1->CPAR4  = SPI1_BASE + 0x10;
    hwp_dmac1->CNDTR4 = 0;
    hwp_dmac1->CSELR1 = 0;
    hwp_dmac1->CSELR1 &= ~DMAC_CSELR1_C4S_Pos;
    hwp_dmac1->CSELR1 |= (28 << DMAC_CSELR1_C4S_Pos);
    hwp_dmac1->CCR4 = DMAC_CCR4_DIR | DMAC_CCR4_MINC;
    hwp_dmac1->CCR4 |= (0x2 << DMAC_CCR4_MSIZE_Pos);
    hwp_dmac1->CCR4 |= (0x2 << DMAC_CCR4_PSIZE_Pos);
    hwp_dmac1->CCR4 |= DMAC_CCR4_EN;

    //ptc controls gpio toggle
    hwp_gpio1->DOER = 0x3;
    hwp_ptc2->TCR1 = (PTC_OP_OR << PTC_TCR1_OP_Pos) | 99; //trigger ble_phytxstart
    hwp_ptc2->TAR1 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR1 = 0x3; //set PA00, PA01
    hwp_ptc2->TCR2 = (PTC_OP_AND << PTC_TCR2_OP_Pos) | 100; //trigger ble_txdone
    hwp_ptc2->TAR2 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR2 = 0xfffffffc; //clear PA00, PA01
    hwp_ptc2->TCR3 = (PTC_OP_OR << PTC_TCR3_OP_Pos) | 102; //trigger ble_phyrxstart
    hwp_ptc2->TAR3 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR3 = 0x1; //set PA00
    hwp_ptc2->TCR4 = (PTC_OP_AND << PTC_TCR4_OP_Pos) | 103; //trigger ble_rxdone
    hwp_ptc2->TAR4 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR4 = 0xfffffffe; //clear PPA00


    hwp_ptc2->TCR5 = (PTC_OP_WRITE << PTC_TCR5_OP_Pos) | 98; //trigger ble_rftxstart
    hwp_ptc2->TAR5 = (uint32_t) & (hwp_dmac1->CNDTR5);
    hwp_ptc2->TDR5 = 0x1; //start dmac2 channel5
    hwp_ptc1->TCR6 = (PTC_OP_WRITE << PTC_TCR6_OP_Pos) | 28; //trigger dmac2_done5
    hwp_ptc1->TAR6 = (uint32_t) & (hwp_dmac1->CNDTR4);
    hwp_ptc1->TDR6 = 0x4; //start dmac2 channel4
    hwp_ptc2->TCR7 = (PTC_OP_WRITE << PTC_TCR7_OP_Pos) | 101; //trigger ble_rfrxstart
    hwp_ptc2->TAR7 = (uint32_t) & (hwp_dmac1->CNDTR5);
    hwp_ptc2->TDR7 = 0x1; //start dmac2 channel5
    hwp_ptc1->TCR8 = (PTC_OP_WRITE << PTC_TCR8_OP_Pos) | 28; //trigger dmac2_done5
    hwp_ptc1->TAR8 = (uint32_t) & (hwp_dmac1->IFCR);
    hwp_ptc1->TDR8 = DMAC_ISR_GIF5; //clear dmac2_done5
}
#endif //SOC_SF32LB52X


static void ad9364_bt_recovery(void)
{


    ad9364_spi_init();
    extern void ad9364_bt_cfg();
    ad9364_bt_cfg();
}
#endif





#ifndef ROM_ATTR
RT_USED int32_t ble_standby_sleep_pre_handler()
{
    return ble_standby_sleep_pre_handler_rom();
};

RT_USED void ble_standby_sleep_after_handler()
{
#ifdef FPGA
    ad9364_bt_recovery();
#endif
#if (defined(SOC_SF32LB56X)||defined(SOC_SF32LB52X)) && !defined(BSP_USING_PC_SIMULATOR)
    HAL_RCC_SetMacFreq();
#endif
    ble_standby_sleep_after_handler_rom();
    // Reconfig
#if (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)|| defined(SOC_SF32LB52X)) && defined(BF0_LCPU)
    rf_ptc_config(0);
#endif
}
#endif // !ROM_ATTR
#endif

#if defined(SOC_SF32LB58X)
    #define NVDS_BUFF_START 0x204FFD00
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB56X)
    #define NVDS_BUFF_START 0x2041FD00
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB52X)
    #define NVDS_BUFF_START 0x2040FE00
    #define NVDS_BUFF_SIZE 512
#else
    #define NVDS_BUFF_START 0
    #define NVDS_BUFF_SIZE 0
#endif

#if (defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X))&& defined(BF0_HCPU)

typedef bool (*hostlib_get_trace_en_callback)(void);

extern void register_hostlib_trace_en_callback(hostlib_get_trace_en_callback fun);

bool bthost_get_hci_trace_en(void)
{
    sifli_nvds_read_tag_t tag;
    tag.tag = NVDS_STACK_TAG_TRACER_CONFIG;
    tag.length = NVDS_STACK_LEN_TRACER_CONFIG;
    uint32_t trc_config;
    uint8_t ret = sifli_nvds_read_tag_value(&tag, (uint8_t *)&trc_config);
    if ((ret == NVDS_OK) && (trc_config != 0x20))
    {
        return true;
    }
    else
    {
        return false;
    }
}

#endif

#ifndef BSP_USING_PC_SIMULATOR
uint32_t BTS2_GET_TIME(void)
{
    uint32_t freq = HAL_LPTIM_GetFreq();
    uint64_t curr_timer = BTS_READ_GTIMER();
    return (uint32_t)(curr_timer * 1000 / freq);
}
#endif

#if !defined(SOC_SF32LB55X) || !defined(BF0_HCPU)
__ROM_USED int bluetooth_init(void)
{
#ifdef BSP_USING_PC_SIMULATOR
    extern char g_uart_com[32];
    if (strlen(g_uart_com) == 0)
        return -1;
#else
#if 0//config BD ADDRESS
    {
        uint8_t *nvds_addr = (uint8_t *)NVDS_BUFF_START;//0x204FFD00;
        uint8_t dut_addr[8] = {0x01, 0x06, 0x00, 0x12, 0x34, 0x56, 0x89, 0xC0};
        uint8_t rtd_addr[8] = {0x01, 0x06, 0x01, 0x55, 0xAA, 0x55, 0xAA, 0xC0};
        uint16_t *used_mem = (uint16_t *)(nvds_addr + 4);

        *used_mem = 0;
        *(uint32_t *)nvds_addr = 0x4E564453;
#if 1 //dut
        memcpy((nvds_addr + (*used_mem) + 8), dut_addr, 8);
#else
        memcpy((nvds_addr + (*used_mem) + 8), rtd_addr, 8);
#endif
        *used_mem = *used_mem + 8;
        *(used_mem + 1) = 0;
#if 1 //log
        {
            //config trace
            //uint8_t config[6] = {0x2F, 0x4, 0x05, 0xCC, 0x11, 0x00};
            uint8_t config[6] = {0x2F, 0x4, 0x00, 0x00, 0x01, 0x00}; //only HCI
            memcpy((nvds_addr + (*used_mem) + 8), config, 6);
            *used_mem = *used_mem + 6;
        }
#endif
    }

#endif
#endif


#if (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)|| defined(SOC_SF32LB52X)) && defined(BF0_LCPU)
    rf_ptc_config(1);
#endif // SOC_SF32LB58X

#if 0
#ifdef SOC_SF32LB56X
#if (defined(BF0_LCPU) || defined(BF0_HCPU)&&defined(CFG_EMB)) &&!defined(BSP_USING_PC_SIMULATOR)
    {
        extern void ble_rf_fulcal(void);
        ble_rf_fulcal();
    }
#endif
#endif
#endif
    bluetooth_isr_init();

    ble_xtal_less_init();

#if defined(RT_USING_PM) && (defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && defined(BF0_LCPU) && !defined(ROM_ATTR)
    bluetooth_pm_init();

#ifdef FPGA
    spi_addr = hwp_bt_mac->DMRADIOCNTL4;
#endif

#endif // defined(RT_USING_PM) && defined(SOC_SF32LB56X)

    //HAL_sw_breakpoint();
    patch_install();
#if (defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)) && defined(BF0_HCPU)
    register_hostlib_trace_en_callback(bthost_get_hci_trace_en);
#endif

//#if defined(SOC_SF32LB58X)||defined(BSP_USING_PC_SIMULATOR)
    bluetooth_config();
//#endif

#ifdef BSP_BLE_SIBLES
#ifdef SOC_BF0_LCPU
    sifli_nvds_init();
    ble_nvds_config_prepare();
#endif //SOC_BF0_LCPU
    ble_boot(ble_event_process);
#if defined(BSP_BLE_CONNECTION_MANAGER) && !defined(BLE_CM_BOND_DISABLE)
    read_bond_infor_from_flash_start_up();
#endif
#else // BSP_BLE_SIBLES

#if (defined(SOC_SF32LB56X)||defined(SOC_SF32LB52X)) && !defined(BSP_USING_PC_SIMULATOR)
    HAL_RCC_SetMacFreq();
#endif


    // Increase task priority to avoid init flow mis-order
#ifndef SOC_SF32LB52X // No need switch priority
    uint8_t read_priority = RT_THREAD_PRIORITY_HIGH - 1;
    uint8_t ori_pri = rt_thread_self()->current_priority;
    rt_thread_control(rt_thread_self(), RT_THREAD_CTRL_CHANGE_PRIORITY, &read_priority);
#endif

#ifdef FPGA_debug
    ble_boot(ble_event_process);
#else //FPGA_debug
    ble_boot(NULL);
#endif // FPGA_debug

#ifndef SOC_SF32LB52X
    rt_thread_control(rt_thread_self(), RT_THREAD_CTRL_CHANGE_PRIORITY, &ori_pri);
#endif
#endif

#if defined(STACK_DEBUG) && !defined(BSP_USING_PC_SIMULATOR)
    *((volatile uint32_t *)(LPSYS_DEBUG_SWITCH_ADDRESS)) |= LPSYS_DEBUG_SWITCH_BLE_PATTER;
    *((volatile uint32_t *)(LPSYS_BLE_DEBUG_SWITCH_ADDRESS)) = 0x8383;
    //hwp_lpsys_aon->DBGMUX |= (2 << LPSYS_AON_DBGMUX_PB44_SEL_Pos) | (2 << LPSYS_AON_DBGMUX_PB43_SEL_Pos);
#endif
    return 0;

}
#endif // !defined(SOC_SF32LB55X) && !defined(BF0_HCPU)

#if defined(BF0_LCPU) && defined(BSP_USING_DATA_SVC)



static int32_t bt_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{

    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        // Do nothing
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        // Do nothing
        break;
    }
    case MSG_SERVICE_START_REQ:
    {
        bluetooth_init();
        datas_send_response(service, msg, 0);
        break;
    }
    default:
        break;
    }
    return 0;
}

static data_service_config_t bt_service_cb =
{
    .max_client_num = 1,
    .queue = RT_NULL,
    .msg_handler = bt_service_msg_handler,
};



int bluetooth_init_in_data_srv(void)
{
    datas_register("BT_EN", &bt_service_cb);
    return 0;
}
#endif //#if defined(BF0_LCPU) && defined(BSP_USING_DATA_SVC)

// SIMULATOR has differernt thread behavior
#if defined(BSP_USING_RTTHREAD) && (defined(BF0_LCPU) || (defined(BF0_HCPU) && (defined(CFG_EMB)) || defined(BSP_USING_PC_SIMULATOR)))
    #if defined(BF0_LCPU) && defined(BSP_USING_DATA_SVC) && !defined(BSP_BLE_SIBLES)
        INIT_ENV_EXPORT(bluetooth_init_in_data_srv);
    #else
        INIT_APP_EXPORT(bluetooth_init);
    #endif
#endif

#ifdef BSP_USING_PC_SIMULATOR
void bluetooth_wakeup_lcpu(void)
{
}

void bluetooth_release_lcpu(void)
{
}

uint8_t a2dp_set_speaker_volume(uint8_t volume)
{
    return volume;
}
void set_speaker_volume(uint8_t volume)
{
}

#endif

#ifndef BSP_USING_PC_SIMULATOR
// Following ASM functions are for different compiler,
// which force "B" to target function. This method could make sure malloc fun save
// actually caller address.

// Implementation of bt_mem_alloc
__asm(
    ".weak bt_mem_alloc \n"
    ".type bt_mem_alloc, %function\n"
    "bt_mem_alloc: \n"
    "b rt_malloc\n"
);


// Implementation of bt_mem_calloc
__asm(
    ".weak bt_mem_calloc \n"
    ".type bt_mem_calloc, %function\n"
    "bt_mem_calloc:\n"
    "b rt_calloc\n"
);


// Implementation of bt_mem_free
__asm(
    ".weak bt_mem_free \n"
    ".type bt_mem_free, %function\n"
    "bt_mem_free:\n"
    "b rt_free\n"
);
#else

void *bt_mem_alloc(rt_size_t size)
{
    return rt_malloc;
}

void *bt_mem_calloc(rt_size_t count, rt_size_t nbytes)
{
    return rt_calloc(count, nbytes);
}

void bt_mem_free(void *ptr)
{
    rt_free(ptr);
}


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

