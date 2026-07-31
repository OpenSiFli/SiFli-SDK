/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include <stdlib.h>
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

/* pwm + input capture example for RT-Thread based platform ----------------------------*/

#define PWM_DEV_NAME "pwmt1"
#if defined(SF32LB58X)
    #define INCAP_DEV_NAME "incap3c2"
#else
    /* 52x/56x/57x: GPTIM2_CH1 (incap2c1) */
    #define INCAP_DEV_NAME "incap2c1"
#endif
#define PWM_PERIOD (1 * 1000 * 1000) /*(ns) -> freq = 1,000,000,000/PWM_PERIOD (hz) */
#define PWM_CHANNEL 2

static struct rt_device *s_incap_dev = RT_NULL;
/* released by the rx_indicate callback (GPTIM ISR) when the capture
 * ringbuffer reaches its watermark; the main loop takes it and drains
 * and prints the captured data in thread context */
static struct rt_semaphore s_incap_sem;

/* serializes input capture start/stop (executed in the tshell thread through
 * the MSH commands) against the drain/print in the main thread: incap_stop()
 * closes the device and frees its ringbuffer, so it must never overlap an
 * ongoing rt_device_read() in incap_consume() */
static struct rt_mutex s_incap_mutex;

/* latest capture pair and pairing state, updated by incap_consume().
 * The pairing state is kept across watermark batches so a pair split over
 * two notifications is still matched; reset when capture is restarted. */
static struct rt_inputcapture_data s_pair_prev;
static rt_bool_t s_pair_has_prev = RT_FALSE;
static rt_bool_t s_pair_valid    = RT_FALSE;
static uint32_t  s_pair_high_us, s_pair_low_us;
static uint32_t  s_last_print_tick;

/* last duty/period set by pwm_set, restored by incap_start after it forces
 * the PWM output low to make the capture start level deterministic */
static uint8_t  s_pwm_percentage = 20;
static uint32_t s_pwm_period     = PWM_PERIOD;

/**
  * @brief  Drain the capture ringbuffer and print the latest HIGH/LOW pair
  *         (one complete period) with the computed duty cycle. Runs in
  *         thread context only. The pairing state is kept across batches so
  *         a pair split over two watermark notifications is still matched.
  */
static void incap_consume(void)
{
    struct rt_inputcapture_data data;
    rt_device_t dev;

    /* the drain must not run concurrently with incap_stop()/incap_start():
     * they close/open the device and free/recreate the ringbuffer */
    rt_mutex_take(&s_incap_mutex, RT_WAITING_FOREVER);

    dev = s_incap_dev;
    if (dev == RT_NULL)
    {
        rt_mutex_release(&s_incap_mutex);
        return;
    }

    /* Drain everything buffered so the ringbuffer cannot overflow between
     * prints; only the latest completed pair is remembered. */
    while (rt_device_read(dev, 0, &data, 1) == 1)
    {
        if (s_pair_has_prev)
        {
            /* any pair (HIGH+LOW or LOW+HIGH) = one complete period */
            s_pair_high_us  = s_pair_prev.is_high ? s_pair_prev.pulsewidth_us : data.pulsewidth_us;
            s_pair_low_us   = s_pair_prev.is_high ? data.pulsewidth_us : s_pair_prev.pulsewidth_us;
            s_pair_valid    = RT_TRUE;
            s_pair_has_prev = RT_FALSE;
        }
        else
        {
            s_pair_prev = data;
            s_pair_has_prev = RT_TRUE;
        }
    }

    /* throttle: print at most one pair per second (a 1kHz PWM produces
     * 2000 edges per second, far more than the console can print) */
    if (s_pair_valid &&
            (s_last_print_tick == 0 ||
             (rt_tick_get() - s_last_print_tick) >= rt_tick_from_millisecond(1000)))
    {
        uint32_t total = s_pair_high_us + s_pair_low_us;
        uint32_t duty  = total > 0 ? (s_pair_high_us * 100 / total) : 0;

        rt_kprintf("captured: pulse=%u us, HIGH\n", (unsigned int)s_pair_high_us);
        rt_kprintf("captured: pulse=%u us, LOW,  duty=%u%%\n", (unsigned int)s_pair_low_us, duty);
        s_last_print_tick = rt_tick_get();
        s_pair_valid = RT_FALSE;
    }

    rt_mutex_release(&s_incap_mutex);
}

/**
  * @brief  rx_indicate callback, called from the GPTIM interrupt context when
  *         the ringbuffer reaches its watermark. It only releases a semaphore
  *         (ISR-safe): printing here would delay the capture interrupts
  *         (console output is synchronous), captured edges would be lost and
  *         the measured pulse widths would be corrupted. Reading and printing
  *         are done by the main loop.
  */
static rt_err_t incap_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&s_incap_sem);
    return RT_EOK;
}

/**
  * @brief  Set PWM duty cycle.
  * @param  percentage: duty cycle (0 ~ 100)
  * @param  period: PWM period in ns
  */
static void pwm_set(uint8_t percentage, uint32_t period)
{
    if (period == 0)
        period = PWM_PERIOD;

    rt_kprintf("pwm_set:percentage:%d,period:%d,freq:%dhz\n", percentage, period, 1000000000 / period);

    /* 1, pinmux set to pwm output */
#if defined(SF32LB52X)
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);
#elif defined (SF32LB58X)
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
#elif defined (SF32LB56X)
    HAL_PIN_Set(PAD_PA36, GPTIM1_CH2, PIN_NOPULL, 1);
#elif defined (SF32LB57X)
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
#endif

    if (percentage > 100)
        percentage = 100;
    rt_uint32_t pulse = percentage * period / 100;

    s_pwm_percentage = percentage;
    s_pwm_period     = period;

    struct rt_device_pwm *device = RT_NULL;
    device = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (!device)
    {
        rt_kprintf("find pwmt1 err\n");
        return;
    }
    rt_device_open((struct rt_device *)device, RT_DEVICE_OFLAG_RDWR);
    rt_pwm_set(device, PWM_CHANNEL, period, pulse);
    rt_pwm_enable(device, PWM_CHANNEL);
}

/**
  * @brief  Initialize input capture device and start capturing.
  */
static int incap_start(int argc, char **argv)
{
    uint8_t  save_pct;
    uint32_t save_period;

    (void)argc;
    (void)argv;

    rt_mutex_take(&s_incap_mutex, RT_WAITING_FOREVER);

    if (s_incap_dev != RT_NULL)
    {
        rt_kprintf("input capture already started\n");
        rt_mutex_release(&s_incap_mutex);
        return -1;
    }

    /* configure input capture pin (connect PWM output pin to this pin with a jumper wire) */
#if defined(SF32LB52X)
    HAL_PIN_Set(PAD_PA37, GPTIM2_CH1, PIN_NOPULL, 1);
#elif defined (SF32LB58X)
    HAL_PIN_Set(PAD_PB01, GPTIM3_CH2, PIN_NOPULL, 0);   /* PB01: LCPU pad, requires hcpu=0 */
#elif defined (SF32LB56X)
    HAL_PIN_Set(PAD_PA39, GPTIM2_CH1, PIN_NOPULL, 1);
#elif defined (SF32LB57X)
    HAL_PIN_Set(PAD_PA21, GPTIM2_CH1, PIN_NOPULL, 1);
#endif

    /* The capture driver assumes the input starts low (first edge = rising).
     * The PWM may be at an arbitrary phase when 'incap_start' is entered, so
     * force the output low before opening the capture and restore the duty
     * cycle previously set by pwm_set afterwards. */
    save_pct    = s_pwm_percentage;
    save_period = s_pwm_period;
    pwm_set(0, save_period);

    s_incap_dev = rt_device_find(INCAP_DEV_NAME);
    if (s_incap_dev == RT_NULL)
    {
        rt_kprintf("find %s failed\n", INCAP_DEV_NAME);
        pwm_set(save_pct, save_period); /* restore PWM duty cycle */
        rt_mutex_release(&s_incap_mutex);
        return -1;
    }

    /* set rx_indicate for watermark notification; the callback only releases
     * a semaphore, the main loop drains the buffer and prints */
    rt_device_set_rx_indicate(s_incap_dev, incap_rx_ind);

    if (rt_device_open(s_incap_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open %s failed\n", INCAP_DEV_NAME);
        s_incap_dev = RT_NULL;
        pwm_set(save_pct, save_period); /* restore PWM duty cycle */
        rt_mutex_release(&s_incap_mutex);
        return -1;
    }

    /* clear any noise data captured during init */
    rt_device_control(s_incap_dev, INPUTCAPTURE_CMD_CLEAR_BUF, RT_NULL);

    /* drop notifications left over from a previous session: once the ringbuffer
     * is cleared there is nothing to drain, so a stale release would only wake
     * the main loop up to consume an empty buffer */
    rt_sem_control(&s_incap_sem, RT_IPC_CMD_RESET, RT_NULL);

    /* reset the consumer's pairing state (may be stale from a previous run) */
    s_pair_has_prev = RT_FALSE;
    s_pair_valid    = RT_FALSE;

    /* restore the PWM duty cycle: the first rising edge is now seen by the
     * capture as a rising edge, so HIGH/LOW marking is correct */
    pwm_set(save_pct, save_period);

    rt_mutex_release(&s_incap_mutex);
    rt_kprintf("input capture started on %s\n", INCAP_DEV_NAME);
    return 0;
}
MSH_CMD_EXPORT(incap_start, start input capture);

/**
  * @brief  Stop input capture and close the device.
  */
static int incap_stop(int argc, char **argv)
{
    rt_device_t dev;

    (void)argc;
    (void)argv;

    rt_mutex_take(&s_incap_mutex, RT_WAITING_FOREVER);

    dev = s_incap_dev;
    if (dev == RT_NULL)
    {
        rt_kprintf("input capture not started\n");
        rt_mutex_release(&s_incap_mutex);
        return -1;
    }

    /* clear the device pointer first so the main loop stops consuming,
     * then stop notifications and close (close frees the ringbuffer) */
    s_incap_dev = RT_NULL;
    rt_device_set_rx_indicate(dev, RT_NULL);
    rt_device_close(dev);
    rt_kprintf("input capture stopped\n");

    rt_mutex_release(&s_incap_mutex);
    return 0;
}
MSH_CMD_EXPORT(incap_stop, stop input capture);

/**
  * @brief  MSH command: set PWM duty cycle.
  *         Usage: pwm_set <percentage> <period_us>
  */
static int cmd_pwm_set(int argc, char **argv)
{
    uint8_t percentage = 20;
    uint32_t period = PWM_PERIOD;

    if (argc >= 2)
    {
        int pct = atoi(argv[1]);
        if (pct < 0 || pct > 100)
        {
            rt_kprintf("warning: invalid percentage %d, valid range is 0~100, use 20\n", pct);
            pct = 20;
        }
        percentage = (uint8_t)pct;
    }
    if (argc >= 3)
    {
        int us = atoi(argv[2]);
        /* clamp to 1us ~ 65000us: the capture driver measures pulse widths
         * with a 1MHz clock and a 16-bit counter (max 65535us) */
        if (us < 1 || us > 65000)
        {
            rt_kprintf("warning: invalid period_us %d, valid range is 1~65000, use 1000\n", us);
            us = 1000;
        }
        period = (uint32_t)us * 1000; /* us to ns */
    }

    pwm_set(percentage, period);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_pwm_set, pwm_set, set PWM duty cycle: pwm_set <percentage> <period_us>);

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* the capture consumer below and the MSH commands (incap_start/incap_stop)
     * run in different threads, so the semaphore and the mutex are initialized
     * before anything else */
    rt_sem_init(&s_incap_sem, "incap_sem", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&s_incap_mutex, "incap_mtx", RT_IPC_FLAG_PRIO);

    rt_kprintf("Start gtimer pwm + input capture demo!\n");

    /* start PWM output */
    pwm_set(20, PWM_PERIOD);

    rt_kprintf("gtimer pwm + input capture demo started!\n");
    rt_kprintf("connect PWM output pin to INCAP input pin with jumper wire\n");

    /* capture consumer: the rx_indicate callback (GPTIM ISR) releases
     * s_incap_sem on every watermark; drain the ringbuffer and print the
     * latest pair here in thread context (at most once per second) */
    while (1)
    {
        if (s_incap_dev == RT_NULL)
        {
            rt_thread_mdelay(1000);
            continue;
        }

        if (rt_sem_take(&s_incap_sem, rt_tick_from_millisecond(1000)) == RT_EOK)
        {
            /* incap_consume() drains the whole ringbuffer, so notifications
             * queued while it was running are redundant: coalesce them into
             * this one wakeup instead of consuming an empty buffer again */
            rt_sem_control(&s_incap_sem, RT_IPC_CMD_RESET, RT_NULL);
            incap_consume();
        }
        else
            rt_kprintf("no capture event, check the jumper wire!\n");
    }
    return RT_EOK;
}
