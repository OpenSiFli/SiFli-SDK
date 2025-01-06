/**
  ******************************************************************************
  * @file   vibrator.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include "vibrator.h"
#include "drv_common.h"
#include "drv_io.h"
#ifdef PMIC_CTRL_ENABLE
    #include "pmic_controller.h"
#endif
//#define DRV_DEBUG
#define LOG_TAG              "drv.vibrator"
#include <drv_log.h>

//#define USING_VIBRATOR_DEVICE

#define VIBRATOR_ENABLE_PIN_NUMBER      44
#define VIBRATOR_PWM_PIN_NUMBER         45

#define VIBRATOR_ENABLE_PAD             (PAD_PA00 + VIBRATOR_ENABLE_PIN_NUMBER)
#define VIBRATOR_ENABLE_GPIO            (GPIO_A0 + VIBRATOR_ENABLE_PIN_NUMBER)

static rt_timer_t  timer_handle;
static rt_uint32_t on_time;
static rt_uint32_t off_time;
static rt_uint32_t repleat_times;
static rt_uint8_t is_busy = 0;
static rt_uint8_t is_on;

void vibrator_timeout_handler(void *parameter)
{
    rt_tick_t tick;
    is_on = is_on ^ 1;
    if (is_on == 0)
    {
        repleat_times--;
        if (repleat_times == 0)
        {
            rt_timer_delete(timer_handle);
            timer_handle = NULL;
            LOG_D("stop");
            is_busy = 0;
            return;
        }
        LOG_D("on");
        tick = rt_tick_from_millisecond(on_time);
        rt_timer_control(timer_handle, RT_TIMER_CTRL_SET_TIME, &tick);
#ifdef PMIC_CTRL_ENABLE
        pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);
#else
        rt_pin_write(VIBRATOR_ENABLE_PIN_NUMBER, 1);
#endif
        rt_pin_write(VIBRATOR_PWM_PIN_NUMBER, 1);
    }
    else
    {
        LOG_D("off");
        tick = rt_tick_from_millisecond(off_time);
        rt_timer_control(timer_handle, RT_TIMER_CTRL_SET_TIME, &tick);
#ifdef PMIC_CTRL_ENABLE
        pmic_device_control(PMIC_OUT_LDO30_VOUT, 0, 1);
#else
        rt_pin_write(VIBRATOR_ENABLE_PIN_NUMBER, 0);
#endif
        rt_pin_write(VIBRATOR_PWM_PIN_NUMBER, 0);
    }
    rt_timer_start(timer_handle);
}

static rt_size_t vibrator_dev_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_uint32_t *time = (rt_uint32_t *)buffer;

    if (size != 3 * sizeof(rt_uint32_t))
        return -RT_EINVAL;

    on_time = time[0]; //on time
    off_time = time[1]; //off time
    repleat_times  = time[3];
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);
#else
    HAL_PIN_Set(VIBRATOR_ENABLE_PAD, VIBRATOR_ENABLE_GPIO, PIN_PULLUP, 1);
    rt_pin_mode(VIBRATOR_ENABLE_PIN_NUMBER, PIN_MODE_OUTPUT);
    rt_pin_write(VIBRATOR_ENABLE_PIN_NUMBER, 1);
#endif
    is_on = 0;

    timer_handle  = rt_timer_create("vibrator", vibrator_timeout_handler,  NULL,
                                    rt_tick_from_millisecond(on_time), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(timer_handle);
    rt_timer_start(timer_handle);
    return size;
}

static rt_err_t vibrator_dev_open(rt_device_t device, rt_uint16_t oflag)
{
    if (is_busy)
        return -EBUSY;
    is_busy  = 1;
    return RT_EOK;
}

static rt_err_t vibrator_dev_close(rt_device_t device)
{
    if (timer_handle)
    {
        rt_timer_stop(timer_handle);
        rt_timer_delete(timer_handle);
#ifdef PMIC_CTRL_ENABLE
        pmic_device_control(PMIC_OUT_LDO30_VOUT, 0, 1);
#else
        rt_pin_write(VIBRATOR_ENABLE_PIN_NUMBER, 0);
        rt_pin_write(VIBRATOR_PWM_PIN_NUMBER, 0);
#endif
        timer_handle = NULL;
    }
    is_busy  = 0;
    return RT_EOK;
}

#ifdef USING_VIBRATOR_DEVICE
static struct rt_device vibrator_dev;

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops vibrator_dev_ops =
{
    RT_NULL,
    vibrator_dev_open,
    vibrator_dev_close,
    RT_NULL,
    vibrator_dev_write,
    RT_NULL
};

#endif

int sifli_vibrator_init(void)
{
    /* make sure only one vibrator device */
    RT_ASSERT(!rt_device_find("vibrator"));

    rt_memset(&vibrator_dev, 0, sizeof(vibrator_dev));

    vibrator_dev.type    = RT_Device_Class_Char;

    /* register vibrator device */
#ifdef RT_USING_DEVICE_OPS
    vibrator_dev.ops     = &vibrator_dev_ops;
#else
    vibrator_dev.open    = vibrator_dev_open;
    vibrator_dev.close   = vibrator_dev_close;
    vibrator_dev.write   = vibrator_dev_write;
#endif

    rt_device_register(&vibrator_dev, "vibrator", RT_DEVICE_FLAG_RDWR);

    return 0;
}
INIT_DEVICE_EXPORT(sifli_vibrator_init);

#endif

rt_err_t vibrator_open()
{
    return vibrator_dev_open(NULL, 0);
}
rt_size_t vibrator_write(rt_uint32_t on_time_ms, rt_uint32_t off_time_ms, rt_uint32_t repeat_times)

{
    rt_uint32_t data[3];
    data[0] = on_time_ms;
    data[1] = off_time_ms;
    data[2] = repeat_times;
    return vibrator_dev_write(NULL, 0, data, sizeof(data));
}
rt_err_t vibrator_close()
{
    return vibrator_dev_close(NULL);
}


//#define DRV_VIBRATOR_TEST
#ifdef DRV_VIBRATOR_TEST

#include <string.h>


int vibrator_test(int argc, char *argv[])
{
    rt_err_t ret;
    rt_uint32_t data[3] = {100, 1000, 9};

    LOG_D("vibrator_test....\n");
    vibrator_open();
    LOG_D("vibrator_write....\n");
    ret = vibrator_write(data[0], data[1], data[2]);
    LOG_D("vibrator_write return %d\n", ret);
    rt_thread_mdelay(10000);
    vibrator_close();

#ifdef USING_VIBRATOR_DEVICE
    rt_device_t dev = rt_device_find("vibrator");
    if (dev)
    {
        rt_device_open(dev, 0);
        rt_device_write(dev, 0, data, sizeof(data));
        rt_thread_mdelay(10000);
        rt_device_close(dev);
    }
#endif


    return 0;
}

MSH_CMD_EXPORT(vibrator_test, vibrator test);
#endif //DRV_VIBRATOR_TEST


