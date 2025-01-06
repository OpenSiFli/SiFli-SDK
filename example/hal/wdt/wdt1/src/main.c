#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>


/**
 * WDT Clock select.
 * 52x:
 *   If PMUC->CR_SEL_LPCLK = 0, watchdog clock selects rc10k
 *   If PMUC->CR_SEL_LPCLK = 1, watchdog clock selects rc32k
 *   52x select rc32k by default in HAL_Init().
 * 56x/58x:
 *   use rc10k.
 */
#if defined(SOC_SF32LB52X)
    #define WDT_CLOCK_FREQ LXT_FREQ
#else
    #if defined(LXT_DISABLE)||defined(SOC_SF32LB55X)
        #define WDT_CLOCK_FREQ HAL_LPTIM_GetFreq()
    #elif defined(FPGA)
        #define WDT_CLOCK_FREQ 12000  // For FPGA, RC10K is actually 12K
    #else
        #define WDT_CLOCK_FREQ 10000  // For ASIC, RC10K might vary
    #endif
#endif

/* WDT Reload1 */
#define BSP_WDT_TIMEOUT (10)

/* WDT Reload2 */
#define WDT_REBOOT_TIMEOUT (60)

/* WDT handle */
static WDT_HandleTypeDef hwdt;

/**
 * @brief WDT interrupt handler.
 */
void WDT_IRQHandler(void)
{
    rt_interrupt_enter();
    static int printed;
    if (printed == 0)
    {
        printed++;
        rt_kprintf("WDT timeout. Interrupt triggered.\n");
    }
    rt_interrupt_leave();
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\nWDT Example:\n");

#ifdef SOC_BF0_HCPU
    hwdt.Instance = hwp_wdt1;  /* WDT1 in HCPU. */
#else
    hwdt.Instance = hwp_wdt2;  /* WDT2 in LCPU. */
#endif

    /* Set respond mode. 0:reset only, 1:interrupt and reset. */
    __HAL_WDT_INT(&hwdt, 1);

    /* Add reboot cause for watchdog. */
    HAL_PMU_SetWdt((uint32_t)hwdt.Instance);

    /* Enable Watchdog to reboot whole system. */
    __HAL_SYSCFG_Enable_WDT_REBOOT(1);

    /* Set WDT reload. */
    hwdt.Init.Reload = WDT_CLOCK_FREQ * BSP_WDT_TIMEOUT;
    /* Set WDT reload2 . */
    hwdt.Init.Reload2 = WDT_CLOCK_FREQ * WDT_REBOOT_TIMEOUT;

    /* WDT initalization and enable.*/
    if (HAL_WDT_Init(&hwdt) != HAL_OK)
    {
        rt_kprintf("WDT INIT FAILED.\n");
        HAL_ASSERT(0);
    }

    rt_kprintf("WDT init OK. Timeout: %d(s) Reload2: %d(s)\n", BSP_WDT_TIMEOUT, WDT_REBOOT_TIMEOUT);
    rt_kprintf("WDT_CVR0:0x%X WDT_CVR1:0x%X\n", READ_REG(hwdt.Instance->WDT_CVR0), READ_REG(hwdt.Instance->WDT_CVR1));

    /* Infinite loop */
    while (1)
    {
        static uint32_t count = 0;

        /* Watchdog pet per 5 seconds. */
        rt_thread_mdelay(5000);
        HAL_WDT_Refresh(&hwdt);
        rt_kprintf("Watchdog feeding.\n");

        /* Stop watchdog petting after 10 times. */
        if (++ count >= 10)
        {
            while (1) {};
        }
    }

    return 0;
}

