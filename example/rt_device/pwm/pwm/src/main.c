#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

#define DBG_TAG "pwm"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* pwm example for RT-Thread based platform -----------------------------------------------*/

#define PWM_DEV_NAME "pwm2"
#define PWM_PERIOD (1 * 1000 * 1000) /*(ns) -> freq = 1,000,000,000/PWM_PERIOD (hz) */
#define PWM_CHANNEL 2

void pwm_set(uint8_t percentage, uint32_t period)
{
    LOG_I("pwm_set:percentage:%d,period:%d,freq:%dhz", percentage, period, 1000000000 / period);

    /* 1, pinmux set to pwm2 mode */
#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
#endif
//    LOG_I("hysys_GPTIM2_PINR1:%x",((hwp_hpsys_cfg->GPTIM1_PINR) & HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Msk)>>HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Pos);

    if (percentage > 100)
        percentage = 100;
    rt_uint32_t pulse = percentage * period / 100;

    struct rt_device_pwm *device = RT_NULL;
    device = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (!device)
    {
        LOG_I("find pwm2 err");
        return;
    }
    rt_device_open((struct rt_device *)device, RT_DEVICE_OFLAG_RDWR);
    rt_pwm_set(device, PWM_CHANNEL, period, pulse);
    rt_pwm_enable(device, PWM_CHANNEL);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Start gtimer pwm demo!\n");

    pwm_set(20, PWM_PERIOD);

    rt_kprintf("gtimer pwm demo end!\n");

    while (1)
    {
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }
    return RT_EOK;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

