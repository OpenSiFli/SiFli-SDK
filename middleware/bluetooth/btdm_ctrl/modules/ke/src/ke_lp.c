/**
 ****************************************************************************************
 *
 * @file ke_lp.c
 *
 * @brief SIFLI kernel low power implementation.
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup Low_power
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"        // stack configuration
#if defined(BF0_LCPU)
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "register.h"
#include "reg_blecore.h"
#include "reg_ipcore.h"
#include "reg_ipcore_bts.h"
#include "reg_btcore.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "ke_lp.h"
#include "rom_config.h"
#include "bf0_hal_patch.h"
#include "rf.h"
#include "rwip.h"
#include "btdm_patch.h"

/*
 * DEFINTIONS
 ****************************************************************************************
 */

typedef int32_t (*ke_enter_deep_sleep_handler)(void);
typedef void (*ke_enter_standby_sleep_handler)(void);
typedef void (*sifli_standby_register_handler)(void);
typedef void (*ke_lp_aon_irq_func)(void);

static int32_t ke_enter_deep_sleep(void);

static void ke_enter_standby_sleep(void);
static void sifli_standby_register_recovery(void);
static void sifli_standby_register_store(void);
static void ke_lp_aon_irq_handler(void);




typedef enum

{
    KE_LP_REG_CLKCNT,
    KE_LP_REG_FINECNT,
    KE_LP_REG_ISOCNT,
    KE_LP_REG_CLKNTGT1,
    KE_LP_REG_HMICRO1,
    KE_LP_REG_CLKNTGT2,
    KE_LP_REG_HMICRO2,
    KE_LP_REG_CLKNTGT3,
    KE_LP_REG_HMICRO3,
    KE_LP_REG_RWBLE,
    KE_LP_REG_WLCNTL,
    KE_LP_REG_WLCURRENTPTR,
    KE_LP_REG_RALCNTL,
    KE_LP_REG_RALCURRENTPTR,
    KE_LP_REG_SEARCH_TIME,
    KE_LP_REG_RAL_LOCAL_RND,
    KE_LP_REG_RAL_PEER_RND,
    KE_LP_REG_ISO_GPIOCNTL,
    KE_LP_REG_BT_CURRENTRXPTR,
    KE_LP_REG_BLE_CURRENTRXPTR,
    KE_LP_REG_BLE_ADVTIM,
    KE_LP_REG_PRE_FEATCH,
    KE_LP_ETPTR, // ETPTR in the same reg with rxdescriptor in IP50.
    KE_LP_BTON,
    KE_LP_REG_TOTAL,
} ke_lp_reg_type_t;

uint32_t g_rand_store;
#define RF_REG_COUNT (sizeof(BT_RFC_TypeDef)/4)
#define RF_MEM_SIZE 0x200 // 2K


#define PATCH_SHOULD_RE (MAX_PATCH_ENTRIES-PATCH_AON)
uint32_t g_reg_store[KE_LP_REG_TOTAL];
volatile uint32_t g_reg_store_rf[RF_REG_COUNT];
struct patch_entry_desc g_patch_retention[PATCH_SHOULD_RE];
uint32_t g_patch_cer;

#ifdef SOC_SF32LB58X
    volatile uint32_t g_mem_store_rf[RF_MEM_SIZE];
#endif

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

ke_sleep_os_hook_handler _ke_sleep_os_hook_handler = _ke_sleep_os_hook;


sifli_standby_register_handler _sifli_standby_register_recovery_handler = sifli_standby_register_recovery;
sifli_standby_register_handler _sifli_standby_register_store_handler = sifli_standby_register_store;
ke_lp_aon_irq_func _ke_lp_aon_irq_func = ke_lp_aon_irq_handler;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */





//Yier, may move to a dedicate sleep file.


#define RWIP_WFI_SLEEP_DURATION (15)
#define RWIP_WFI_STOP_DURATION (60)

rt_timer_t ke_sleep_t_handle;

//#define KE_LP_DEBUG

#ifdef KE_LP_DEBUG
    uint32_t g_s_1;
    uint32_t g_s_2;
    uint32_t g_s_3;
    uint32_t g_s_4;
    uint32_t g_s_5;
#endif


extern void sifli_mbox_bcpu2hcpu(uint8_t *param);

//#include "core_cm0plus.h"
#include "ke_msg.h"


#if 0
void ke_sleep_test_trigger_handler(void *parameter)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_TEST_TRIGGER_HANDLER_FUN_BIT, parameter);
    rt_kprintf("ke kb\r\n");
    //rt_pm_release(PM_SLEEP_MODE_LIGHT);
    rt_pm_request(PM_SLEEP_MODE_STANDBY);

#ifdef KE_LP_DEBUG
    g_s_1 = rt_tick_get();
#endif
}



void ke_sleep_request(uint32_t duration)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_REQUEST_FUN_BIT, duration);

    uint8_t mode = PM_SLEEP_MODE_IDLE;
#ifdef KE_LP_DEBUG
    g_s_4 = rt_tick_get();
    g_s_5 = duration;
#endif
    if (duration > 10)
    {

        duration -= 10;
        rt_tick_t tick = rt_tick_from_millisecond(duration);
        if (!ke_sleep_t_handle)
        {
            ke_sleep_t_handle = rt_timer_create("KS", ke_sleep_test_trigger_handler, NULL,
                                                tick,  RT_TIMER_FLAG_SOFT_TIMER);
        }
        else
        {
            rt_timer_stop(ke_sleep_t_handle);
            rt_timer_control(ke_sleep_t_handle, RT_TIMER_CTRL_SET_TIME, &tick);
        }
        rt_timer_start(ke_sleep_t_handle);
        rt_kprintf("r(%d)\r\n", duration);
        rt_pm_request(PM_SLEEP_MODE_LIGHT);
    }
    else
    {
        if (ke_sleep_t_handle)
        {
            rt_timer_stop(ke_sleep_t_handle);
        }
    }
}
#endif
#include <board.h>



static void ke_lp_aon_irq_handler(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_LP_AON_IRQ_HANDLER_FUN_BIT);

    // Switch to XT48
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);

}

void ble_aon_irq_handler(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, BLE_AON_IRQ_HANDLER_FUN_BIT);
    _ke_lp_aon_irq_func();
}

int32_t ble_deep_sleep_pre_handler()
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(KE_LP_PATCH_TYPE, BLE_DEEP_SLEEP_PRE_HANDLER_FUN_BIT, int32_t);
    if (((hwp_lpsys_aon->SLP_CTRL & LPSYS_AON_SLP_CTRL_BT_WKUP_Msk) >> LPSYS_AON_SLP_CTRL_BT_WKUP_Pos) == 1)
    {
        return -1;
    }
    return 0;

};

void ble_deep_sleep_after_handler()
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, BLE_DEEP_SLEEP_AFTER_HANDLER_FUN_BIT);
};



static void sifli_standby_register_store(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, SIFLI_STANDBY_REGISTER_STORE_FUN_BIT);
    rwip_time_t t = rwip_time_get();
    g_reg_store[KE_LP_REG_CLKCNT] = t.hs;
    // The get fine time is 312.5 - t.us
    g_reg_store[KE_LP_REG_FINECNT] = ip_finetimecnt_get();
    g_reg_store[KE_LP_REG_ISOCNT] = ip_isocntsamp_get();

    // Timer
    g_reg_store[KE_LP_REG_CLKNTGT1] = ip_clkntgt1_get();
    g_reg_store[KE_LP_REG_HMICRO1] = ip_hmicrosectgt1_get();
    g_reg_store[KE_LP_REG_CLKNTGT2] = ip_clkntgt2_get();
    g_reg_store[KE_LP_REG_HMICRO2] = ip_hmicrosectgt2_get();
    g_reg_store[KE_LP_REG_CLKNTGT3] = ip_clkntgt3_get();
    g_reg_store[KE_LP_REG_HMICRO3] = ip_hmicrosectgt3_get();

    // core reg - ble
    g_reg_store[KE_LP_REG_RWBLE] = ble_rwblecntl_get();
    g_reg_store[KE_LP_REG_WLCNTL] = ble_wpalcntl_get(); // white list
    g_reg_store[KE_LP_REG_WLCURRENTPTR] = ble_wpalcurrentptr_get();
    g_reg_store[KE_LP_REG_RALCNTL] = ble_ralcntl_get(); // RAL
    g_reg_store[KE_LP_REG_RALCURRENTPTR] = ble_ralcurrentptr_get();
    g_reg_store[KE_LP_REG_SEARCH_TIME] = ble_search_timeout_get();
    g_reg_store[KE_LP_REG_RAL_LOCAL_RND] = ble_ral_local_rnd_get();
    g_reg_store[KE_LP_REG_RAL_PEER_RND] = ble_ral_peer_rnd_get();

    g_reg_store[KE_LP_REG_ISO_GPIOCNTL] = ble_isogpiocntl_get();
    g_reg_store[KE_LP_REG_BT_CURRENTRXPTR] = bt_currentrxdescptr_get();
    g_reg_store[KE_LP_REG_BLE_CURRENTRXPTR] = ble_currentrxdescptr_get();

    g_reg_store[KE_LP_REG_BLE_ADVTIM] = ble_advtim_get();
    g_reg_store[KE_LP_REG_PRE_FEATCH] = ip_timgencntl_get();
    g_reg_store[KE_LP_ETPTR] = ip_etptr_get();
    g_reg_store[KE_LP_BTON] = bt_rwbtcntl_rwbten_getf();

#if defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)
    // Save RF related registers.
    hwp_dmac2->CPAR1 = (volatile uint32_t)&g_reg_store_rf;
    hwp_dmac2->CM0AR1 = BT_RFC_REG_BASE;
    hwp_dmac2->CNDTR1 = RF_REG_COUNT;
    hwp_dmac2->CCR1 = DMAC_CCR1_MEM2MEM | DMAC_CCR1_DIR; // DIR is 1, source is CM0AR, destination is CPAR
    hwp_dmac2->CCR1 |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    hwp_dmac2->CCR1 |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    hwp_dmac2->CCR1 |= DMAC_CCR1_EN;
    while ((hwp_dmac2->ISR & DMAC_ISR_TCIF1) == 0);

#else
    // Save RF related registers.
    hwp_dmac3->CPAR1 = (volatile uint32_t)&g_reg_store_rf;
    hwp_dmac3->CM0AR1 = BT_RFC_REG_BASE;
    hwp_dmac3->CNDTR1 = RF_REG_COUNT;
    hwp_dmac3->CCR1 = DMAC_CCR1_MEM2MEM | DMAC_CCR1_DIR; // DIR is 1, source is CM0AR, destination is CPAR
    hwp_dmac3->CCR1 |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_EN;
    while ((hwp_dmac3->ISR & DMAC_ISR_TCIF1) == 0);

    // Save RF related mem.
    hwp_dmac3->CPAR1 = (volatile uint32_t)&g_mem_store_rf;
    hwp_dmac3->CM0AR1 = BT_RFC_MEM_BASE;
    hwp_dmac3->CNDTR1 = RF_MEM_SIZE;
    hwp_dmac3->CCR1 = DMAC_CCR1_MEM2MEM | DMAC_CCR1_DIR; // DIR is 1, source is CM0AR, destination is CPAR
    hwp_dmac3->CCR1 |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_EN;
    while ((hwp_dmac3->ISR & DMAC_ISR_TCIF1) == 0);

#endif

    // Save Patch
    HAL_PATCH_save(&g_patch_retention[0], PATCH_SHOULD_RE, &g_patch_cer);

}

void sifli_standby_register_recovery(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, SIFLI_STANDBY_REGISTER_RECOVERY_FUN_BIT);
    ip_clkncntcorr_clkncntcorr_setf(g_reg_store[KE_LP_REG_CLKCNT]);
    ip_finecntcorr_setf(g_reg_store[KE_LP_REG_FINECNT]);
    // set the correction as an absolute delta in microseconds
    ip_isocntcorr_setf(g_reg_store[KE_LP_REG_ISOCNT]);

    /* Time only update after sleep correction enable and before enter active state */
    /* clk can use delta while isocnt use corr mode to replace sampe value but not increse. */
    ip_deepslcntl_deep_sleep_corr_en_setf(1);

    ip_clkntgt1_set(g_reg_store[KE_LP_REG_CLKNTGT1]);
    ip_hmicrosectgt1_set(g_reg_store[KE_LP_REG_HMICRO1]);
    ip_clkntgt2_set(g_reg_store[KE_LP_REG_CLKNTGT2]);
    ip_hmicrosectgt2_set(g_reg_store[KE_LP_REG_HMICRO2]);
    ip_clkntgt3_set(g_reg_store[KE_LP_REG_CLKNTGT3]);
    ip_hmicrosectgt3_set(g_reg_store[KE_LP_REG_HMICRO3]);

    ble_rwblecntl_set(g_reg_store[KE_LP_REG_RWBLE]);
    ble_wpalcntl_set(g_reg_store[KE_LP_REG_WLCNTL]);
    ble_wpalcurrentptr_set(g_reg_store[KE_LP_REG_WLCURRENTPTR]);
    ble_ralcntl_set(g_reg_store[KE_LP_REG_RALCNTL]); // RAL
    ble_ralcurrentptr_set(g_reg_store[KE_LP_REG_RALCURRENTPTR]);
    ble_search_timeout_set(g_reg_store[KE_LP_REG_SEARCH_TIME]);
    ble_ral_local_rnd_set(g_reg_store[KE_LP_REG_RAL_LOCAL_RND]);
    ble_ral_peer_rnd_set(g_reg_store[KE_LP_REG_RAL_PEER_RND]);
    /* Local/Peer RND.only affective after set init.*/
    ble_ral_local_rnd_lrnd_init_setf(1);
    ble_ral_peer_rnd_prnd_init_setf(1);

    ble_isogpiocntl_set(g_reg_store[KE_LP_REG_ISO_GPIOCNTL]);

    bt_currentrxdescptr_set(g_reg_store[KE_LP_REG_BT_CURRENTRXPTR]);
    ble_currentrxdescptr_set(g_reg_store[KE_LP_REG_BLE_CURRENTRXPTR]);

    ble_advtim_set(g_reg_store[KE_LP_REG_BLE_ADVTIM]);
    ip_timgencntl_set(g_reg_store[KE_LP_REG_PRE_FEATCH]);
    ip_etptr_set(g_reg_store[KE_LP_ETPTR]);
    bt_rwbtcntl_rwbten_setf(g_reg_store[KE_LP_BTON]);

    bt_rwbtcntl_cx_dnabort_setf(1);
    bt_rwbtcntl_cx_rxbsyena_setf(1);
    bt_rwbtcntl_cx_txbsyena_setf(1);


#if defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)
    // Recovery RF related registers.
    hwp_dmac2->CPAR1 = (volatile uint32_t)&g_reg_store_rf;
    hwp_dmac2->CM0AR1 = BT_RFC_REG_BASE;
    hwp_dmac2->CNDTR1 = RF_REG_COUNT;
    hwp_dmac2->CCR1 = DMAC_CCR1_MEM2MEM; // DIR is 0, source is CPAR, destination is CM0AR
    hwp_dmac2->CCR1 |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    hwp_dmac2->CCR1 |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    hwp_dmac2->CCR1 |= DMAC_CCR1_EN;
    while ((hwp_dmac2->ISR & DMAC_ISR_TCIF1) == 0);
    hwp_dmac2->IFCR = DMAC_IFCR_CGIF1;
    hwp_dmac2->CCR1 &= ~DMAC_CCR1_EN;

#else
    // Recovery RF related registers.
    hwp_dmac3->CPAR1 = (volatile uint32_t)&g_reg_store_rf;
    hwp_dmac3->CM0AR1 = BT_RFC_REG_BASE;
    hwp_dmac3->CNDTR1 = RF_REG_COUNT;
    hwp_dmac3->CCR1 = DMAC_CCR1_MEM2MEM; // DIR is 0, source is CPAR, destination is CM0AR
    hwp_dmac3->CCR1 |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_EN;
    while ((hwp_dmac3->ISR & DMAC_ISR_TCIF1) == 0);
    hwp_dmac3->IFCR = DMAC_IFCR_CGIF1;
    hwp_dmac3->CCR1 &= ~DMAC_CCR1_EN;

    // Recovery RF related mem.
    hwp_dmac3->CPAR1 = (volatile uint32_t)&g_mem_store_rf;
    hwp_dmac3->CM0AR1 = BT_RFC_MEM_BASE;
    hwp_dmac3->CNDTR1 = RF_MEM_SIZE;
    hwp_dmac3->CCR1 = DMAC_CCR1_MEM2MEM; // DIR is 0, source is CPAR, destination is CM0AR
    hwp_dmac3->CCR1 |= DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos);
    hwp_dmac3->CCR1 |= DMAC_CCR1_EN;
    while ((hwp_dmac3->ISR & DMAC_ISR_TCIF1) == 0);
    hwp_dmac3->IFCR = DMAC_IFCR_CGIF1;
    hwp_dmac3->CCR1 &= ~DMAC_CCR1_EN;
#endif


}



int32_t ble_standby_sleep_pre_handler()
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(KE_LP_PATCH_TYPE, BLE_STANDBY_SLEEP_PRE_HANDLER_FUN_BIT, int32_t);
    if (((hwp_lpsys_aon->SLP_CTRL & LPSYS_AON_SLP_CTRL_BT_WKUP_Msk) >> LPSYS_AON_SLP_CTRL_BT_WKUP_Pos) == 1)
    {
        return -1;
    }
    _sifli_standby_register_store_handler();
    return 0;
};


void ble_standby_sleep_after_handler()
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, BLE_STANDBY_SLEEP_AFTER_HANDLER_FUN_BIT);
    // Recovery Patch
    if (g_patch_cer)
        HAL_PATCH_install2(&g_patch_retention[0], PATCH_SHOULD_RE, g_patch_cer);

    rf_init(&rwip_rf);
    _sifli_standby_register_recovery_handler();
}






void ke_sleep_exit(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_EXIT_FUN_BIT);
#if 0
    if (ke_env.sleep_mode > PM_SLEEP_MODE_IDLE)
    {
        rt_pm_release(ke_env.sleep_mode);
        ke_env.sleep_mode = PM_SLEEP_MODE_NONE;
    }
#endif
}

// TODO: implement for A0
bool ke_MAC_sleep_check(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(KE_LP_PATCH_TYPE, KE_MAC_SLEEP_CHECK_FUN_BIT, bool);
    return (((hwp_lpsys_aon->SLP_CTRL & ((uint32_t)LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk)) >> LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos) == 1
            || (rwip_wakeup_check() == 1));
}
void ke_MAC_sleep_wakeup(bool wakeup)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_MAC_SLEEP_WAKEUP_FUN_BIT, wakeup);
    hwp_lpsys_aon->SLP_CTRL |= LPSYS_AON_SLP_CTRL_WKUP_REQ;
}

#if 0
/* 10 tick, current is 10ms */
#define KE_SLEEP_MAX (10)
static rt_tick_t sleep_counter, sleep_change;
static void ke_sleep_idle_hook(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_IDLE_HOOK_FUN_BIT);
    //rt_kprintf("idle hook(%d)\r\n", sleep_counter);
    if ((sleep_change - sleep_counter <= 0x0FFFFFFF)
            && (sleep_change - sleep_counter >= KE_SLEEP_MAX))
    {
        sleep_counter = sleep_change;
        rt_pm_request(PM_SLEEP_MODE_LIGHT);
    }
    sleep_change = rt_tick_get();
}

static void ke_sleep_context_switch_hook(struct rt_thread *from, struct rt_thread *to)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_CONTEXT_SWITCH_HOOK_FUN_BIT, from, to);
    char idle_t[] = "tidle";
    if (rt_strncmp(from->name, idle_t, sizeof(idle_t)))
        sleep_counter = sleep_change;
}
#endif

static uint8_t sleep_status;

#if 0
int ke_sleep_ble_mac_suspend(const struct rt_device *device, uint8_t mode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_BLE_MAC_SUSPEND_FUN_BIT, int, device, mode);
    int ret = RT_EOK;
    if (device == &g_ble_mac_dev)
    {
        if (mode >= PM_SLEEP_MODE_LIGHT)
        {
            //uint8_t status = 0;
            //uint8_t status = rwip_sleep();
            //sleep_status = rwip_sleep();
            if (((hwp_lpsys_aon->SLP_CTRL & LPSYS_AON_SLP_CTRL_BT_WKUP_Msk) >> LPSYS_AON_SLP_CTRL_BT_WKUP_Pos) == 1)

            {
                return -RT_ERROR;
            }

            else if (rwip_prevent_sleep_check() != 0 && rwip_prevent_sleep_check() != RW_DEEP_SLEEP)
            {
                return -RT_ERROR;
            }

            //rt_kprintf("MAC sleep ret %d", status);
        }
        else
        {
            ret = -RT_ERROR;
            //rt_kprintf("MAC sleep not support yet (%d)", mode);
        }

    }
    return ret;
}
#else
int ke_sleep_ble_mac_suspend(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(KE_LP_PATCH_TYPE, KE_SLEEP_BLE_MAC_SUSPEND_FUN_BIT, int);
    int ret = RT_EOK;

    //uint8_t status = 0;
    //uint8_t status = rwip_sleep();
    //sleep_status = rwip_sleep();
    if (((hwp_lpsys_aon->SLP_CTRL & LPSYS_AON_SLP_CTRL_BT_WKUP_Msk) >> LPSYS_AON_SLP_CTRL_BT_WKUP_Pos) == 1)

    {
        ret = -RT_ERROR;
    }

    else if (rwip_prevent_sleep_check() != 0 && rwip_prevent_sleep_check() != RW_DEEP_SLEEP)
    {
        ret = -RT_ERROR;
    }


    return ret;
}


#endif


//#include <cmsis_armclang.h>
//#include <cmsis_armcc.h>
void ke_idle_hook_func(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(KE_LP_PATCH_TYPE, KE_IDLE_HOOK_FUNC_FUN_BIT);
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    sleep_status = rwip_sleep();

    rt_hw_interrupt_enable(level);

}

int _ke_sleep_os_hook(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(KE_LP_PATCH_TYPE, _KE_SLEEP_OS_HOOK_FUN_BIT, int);
    return 0;
}




#endif //RT_USING_PM

