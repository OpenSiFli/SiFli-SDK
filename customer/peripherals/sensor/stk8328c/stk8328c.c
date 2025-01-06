/**
  ******************************************************************************
  * @file   stk8328c.c
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

#include "stk8328c.h"
#include "board.h"
#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif

#ifdef ACC_USING_STK8328C

#define DRV_DEBUG
#define LOG_TAG              "drv.stk"
#include <drv_log.h>


#define STK8328C_DEV_NAME        "slw_dev"

#define STK8328C_I2C_ADDR_H (0x1F)
#define STK8328C_I2C_ADDR_L (0x0F)

#define STK8328C_I2C_ADDR    STK8328C_I2C_ADDR_H

//#define STK8328C_USE_INT
#define STK8328C_UES_INT1  0
#define STK8328C_UES_INT2  1


struct STK8328C_CONT_T
{
    struct rt_i2c_bus_device *bus_handle;
    uint8_t open_flag;
    uint8_t dev_addr;
    uint8_t whoami;
};

static sifdev_sensor_ctx_t sens_cont;
static struct STK8328C_CONT_T stk8328c_content;
#ifdef STK8328C_USE_INT
    static struct rt_semaphore stk_int_sem;
    rt_thread_t stk8328c_thread = NULL;
#endif


static int stk8328c_power_on()
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_err_t ret = RT_EOK;
#if (STK8328C_POW_PIN >= 0)
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        rt_kprintf("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;

    m.pin = STK8328C_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = STK8328C_POW_PIN;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    rt_kprintf("stk8328c_power_on\n");
#endif
    return ret;
}

static int stk8328c_power_off()
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_err_t ret;
#if (STK8328C_POW_PIN >= 0)
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        rt_kprintf("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;

    m.pin = STK8328C_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = STK8328C_POW_PIN;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    rt_kprintf("stk8328c_power_off\r\n");
#endif
    return ret;
}

void stk8328c_power_onoff(bool onoff)
{
    if (onoff)
    {
#ifdef PMIC_CONTROL_SERVICE
        pmic_service_control(PMIC_CONTROL_GSENSOR, 1);
#else
        stk8328c_power_on();
#endif
    }
    else
    {
#ifdef PMIC_CONTROL_SERVICE
        pmic_service_control(PMIC_CONTROL_GSENSOR, 0);
#else
        stk8328c_power_off();
#endif
    }
}


static int stk8328c_i2c_init(void)
{
    /* get i2c bus device */
    stk8328c_content.bus_handle = (struct rt_i2c_bus_device *)rt_device_find(STK8328C_BUS_NAME);
    if (RT_Device_Class_I2CBUS != stk8328c_content.bus_handle->parent.type)
    {
        stk8328c_content.bus_handle = NULL;
    }
    if (stk8328c_content.bus_handle)
    {
        rt_device_open((rt_device_t)stk8328c_content.bus_handle, RT_DEVICE_FLAG_RDWR);
    }
    else
    {
        LOG_E("bus not find\n");
        return RT_ERROR;
    }

    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 5000,
            .max_hz = 400000,
        };

        rt_i2c_configure(stk8328c_content.bus_handle, &configuration);
    }

    return 0;
}

static int stk8328c_i2c_deinit(void)
{
    int ret;
    if (stk8328c_content.bus_handle != NULL)
        ret = rt_device_close((rt_device_t)(stk8328c_content.bus_handle));

    return ret;
}

static int32_t stk_i2c_write(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_int8_t res = 0;
    uint8_t value[2];

    struct STK8328C_CONT_T *handle = (struct STK8328C_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
        struct rt_i2c_msg msgs;

        value[0] = reg;
        value[1] = *data;

        msgs.addr  = STK8328C_I2C_ADDR;   /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = &value[0];             /* Send data pointer */
        msgs.len   = sizeof(value);

        if (rt_i2c_transfer(handle->bus_handle, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = 0,
                .timeout = 5000,
                .max_hz = 400000,
            };

            rt_i2c_configure(stk8328c_content.bus_handle, &configuration);

            res = -RT_ERROR;
        }


    }


    return res;

}

static int32_t stk_i2c_read(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res = 0;
    struct STK8328C_CONT_T *handle = (struct STK8328C_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = STK8328C_I2C_ADDR;    /* slave address */
        msgs[0].flags = RT_I2C_WR;        /* write flag */
        msgs[0].buf   = &reg;              /* Send data pointer */
        msgs[0].len   = 1;

        msgs[1].addr  = STK8328C_I2C_ADDR;    /* slave address */
        msgs[1].flags = RT_I2C_RD;        /* write flag */
        msgs[1].buf   = data;              /* Send data pointer */
        msgs[1].len   = len;

        if (rt_i2c_transfer(handle->bus_handle, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = 0,
                .timeout = 5000,
                .max_hz = 400000,
            };

            rt_i2c_configure(stk8328c_content.bus_handle, &configuration);

            res = -RT_ERROR;
        }
    }

    return res;
}

#ifdef STK8328C_USE_INT

/* SC7A22 interrupt init, it use gpio input as int */
// check edge
static void stk8328c_int1_handle(void *args)
{
    LOG_E("stk8328c_int1_handle\n");
    rt_sem_release(&stk_int_sem);
}

static void stk8328c_int2_handle(void *args)
{
    LOG_E("stk8328c_int2_handle\n");
    rt_sem_release(&stk_int_sem);
}

static int stk8328c_gpio_int_enable(void)
{
    struct rt_device_pin_mode m;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at sc7a22\n");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
#if STK8328C_UES_INT1
    // int pin cfg
    m.pin = STK8328C_INT1;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable int
    rt_pin_mode(STK8328C_INT1, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, stk8328c_int1_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif
    // for int2
#if STK8328C_UES_INT2
    // int pin cfg
    m.pin = STK8328C_INT2;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable int
    rt_pin_mode(STK8328C_INT2, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, stk8328c_int2_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif

    return 0;

}

static int stk8328c_gpio_int_disable(void)
{
    struct rt_device_pin_mode m;

#if STK8328C_UES_INT1
    // int pin cfg
    m.pin = STK8328C_INT1;

    // enable int
    //rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, sc7a22_int_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 0);
#endif
    // for int2
#if STK8328C_UES_INT2
    // int pin cfg
    m.pin = STK8328C_INT2;
    rt_pin_irq_enable(m.pin, 0);

#endif

    return 0;

}


void stk8328c_sensor_task(void *params)  //20ms
{
    int32_t ret;

    // add by whj for thread loop
    LOG_E("stk8328c_sensor_task\n");
    while (1)
    {
        //rt_thread_delay(20);    // no irq, use delay instead
        rt_sem_take(&stk_int_sem, RT_WAITING_FOREVER) ;
        //rt_kprintf("stk8328c_sensor_task!\r\n");

    }
}
#endif // STK8328C_USE_INT


int stk8328c_reg_init(void)
{
    int ret;
    //soft reset
    ret = stk8328c_reset_set(&sens_cont, STK_SOFT_RESET_VAL);
    rt_thread_mdelay(50);

    //set range
    ret = stk8328c_xl_full_scale_set(&sens_cont, STK832x_RANGE_16G);

    //set eng mode
    ret = stk8328c_eng_mode_set(&sens_cont, STK_ENG_MODE_VAL);

    //i2c watch dog
    ret = stk8328c_i2c_wdt_set(&sens_cont, STK_I2C_WDT_VAL);

    //set bandwidth
    ret = stk8328c_band_width_set(&sens_cont, STK_BAND_WIDTH_VAL);

    //set es mode
    ret = stk8328c_es_mode_set(&sens_cont, STK_ES_MODE_VAL);

    //fifo mode
    ret = stk8328c_fifo_mode_set(&sens_cont, STK_FIFO_MODE_VAL);

    //power mode
    ret = stk8328c_power_mode_set(&sens_cont, STK_PWR_MODE_VAL);

    return ret;
}

int stk8328c_reg_deinit(void)
{
    int ret;
    ret = stk8328c_power_mode_set(&sens_cont, STK_SUSPEND_MODE);
    if (ret != 0) goto exit;

exit:
    return ret;
}

int stk8328c_init(void)
{
    int res;

    stk8328c_content.dev_addr = STK8328C_I2C_ADDR;
    stk8328c_content.bus_handle = NULL;
    stk8328c_content.whoami = 0;
    stk8328c_content.open_flag = 0;

    stk8328c_power_onoff(1);
    rt_thread_mdelay(50);

    res = stk8328c_i2c_init();
    if (res != 0)
    {
        LOG_E("stk8328c_i2c_init fail\n");
        stk8328c_i2c_deinit();
        return res;
    }

    sens_cont.read_reg = stk_i2c_read;
    sens_cont.write_reg = stk_i2c_write;
    sens_cont.handle = (void *)&stk8328c_content;

    // check who am I and try i2c slave address
    res = stk8328c_device_id_get(&sens_cont, &(stk8328c_content.whoami));
    LOG_E("res 1 = %d when address = %d\n", res, stk8328c_content.whoami);
    if (stk8328c_content.whoami != STK8329_CHIPID_VAL)
    {
        // try another address
        stk8328c_content.dev_addr = STK8328C_I2C_ADDR;
        res = stk8328c_device_id_get(&sens_cont, &(stk8328c_content.whoami));
        LOG_E("res 2 = %d when address = %d\n", res, stk8328c_content.whoami);
        if (stk8328c_content.whoami != STK8329_CHIPID_VAL)
        {
            // 2 address all fail
            LOG_E("stk8328c i2c slave init fail\n");
            stk8328c_i2c_deinit();
            return 1;
        }
    }

    LOG_E("stk8328c init done, i2c address = 0x%x, whoAmI 0x%x\n", stk8328c_content.dev_addr, stk8328c_content.whoami);

    return 0;
}

int stk8328c_open(void)
{
    int ret = 0;

    if (stk8328c_content.open_flag == 1) // opened before
        return 0;


    ret = stk8328c_reg_init();
    rt_kprintf("stk8328c_reg_init ret %d\n", ret);


#ifdef STK8328C_USE_INT
    // start a thread to check data available
    rt_sem_init(&stk_int_sem, "slw_int", 0, RT_IPC_FLAG_FIFO);
    stk8328c_thread = rt_thread_create("sc7a22", stk8328c_sensor_task, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (stk8328c_thread != NULL)
    {
        rt_thread_startup(stk8328c_thread);
        LOG_D("6d thread started\n");
    }
    else
        LOG_E("Create 6d thread fail\n");

    stk8328c_gpio_int_enable();

    //if (stk8328c_content.int_en == 1) // interrupt enable, so gyro and acce data ready should be used.
    {
#if STK8328C_UES_INT1
        // set gsensor side int1
#endif

#if STK8328C_UES_INT2
        // set gsensor side int2
#endif
    }
#endif
    stk8328c_content.open_flag = 1;

    return ret;
}

int stk8328c_close(void)
{
    if (stk8328c_content.open_flag == 0) // closed before
        return 0;

#ifdef STK8328C_USE_INT
    stk8328c_gpio_int_disable();
    rt_sem_release(&stk_int_sem);

    if (stk8328c_thread != NULL)
    {
        rt_thread_delete(stk8328c_thread);
        stk8328c_thread = NULL;
    }
#endif

    stk8328c_reg_deinit();


    stk8328c_power_onoff(0);
    rt_device_close((rt_device_t)(stk8328c_content.bus_handle));
    stk8328c_content.open_flag = 0;

    return 0;
}

int stk8328c_set_fifo_mode(uint8_t val)
{
    int ret;
    stk8328c_fifo_mode_set(&sens_cont, val);
    return ret;
}

uint8_t stk8328c_get_fifo_count(void)
{
    uint8_t value = 0;
    stk8328c_fifo_count_get(&sens_cont, &value);

    value = value & 0x7F;

    return value;
}


int stk8328c_read_fifo(uint8_t *buf, int len)
{
    rt_err_t ret;
    if (buf == NULL || len == 0)
        return 0;

    stk8328c_read_fifo_data(&sens_cont, buf, len);

    return len;
}

int stk8328c_set_fifo_threshold(int thd)
{

    return 0;
}

int stk8328c_get_waterm_status(void)
{
    uint8_t value;

    return (int)value;
}

int stk8328c_get_overrun_status(void)
{
    uint8_t value;

    return (int)value;
}

int stk8328c_get_fifo_full_status(void)
{
    uint8_t value;

    return (int)value;
}

int stk8328c_get_fifo_empty_status(void)
{
    uint8_t value;

    return (int)value;
}




// function for handle and address --------------------
uint32_t stk8328c_get_bus_handle(void)
{
    return (uint32_t)(stk8328c_content.bus_handle);
}
uint8_t stk8328c_get_dev_addr(void)
{
    return stk8328c_content.dev_addr;
}

uint8_t stk8328c_get_dev_id(void)
{
    return stk8328c_content.whoami;
}

int stk8328c_self_check(void)
{
    int ret = 0;
    uint8_t whoami;
    ret = stk8328c_device_id_get(&sens_cont, &whoami);
    if (whoami != STK8329_CHIPID_VAL)
    {
        return -1;
    }

    return 0;
}



#endif /*ACC_USING_SC7A22*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
