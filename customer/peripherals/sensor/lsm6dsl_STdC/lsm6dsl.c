/**
  ******************************************************************************
  * @file   lsm6dsl.c
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

#include <string.h>

#include "lsm6dsl_reg.h"
#include "lsm6dsl.h"
#include "board.h"

#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif
#ifdef ACC_USING_LSM6DSL

#define DRV_DEBUG
#define LOG_TAG              "drv.lsm"
#include <drv_log.h>


// move it to menuconfig if needed
//#define LSM6DSL_I2C_BUS         "i2c3"
#define LSM6DSL_DEV_NAME        "lsm_dev"

#define LSM6DSL_I2C_ADDR_H (0x6b)
#define LSM6DSL_I2C_ADDR_L (0x6a)

#define LSM6DSL_I2C_ADDR    LSM6DSL_I2C_ADDR_H

//#define LSM6DSL_USE_INT
#define LSM6DSL_UES_INT1  0
#define LSM6DSL_UES_INT2  1


//#define LSM6DSL_INT_GPIO_BIT        (8)

/* There are 2 interrupt for lsm6dsl, use gpio4 for test int2 */
//#define LSM6DSL_INT2_GPIO_BIT       (4)


#define LSM6DSL_GYRO_DATA_EN        (1<<0)
#define LSM6DSL_ACCE_DATA_EN        (1<<1)
#define LSM6DSL_TEMP_DATA_EN        (1<<2)
#define LSM6DSL_STEP_DATA_EN        (1<<3)

#define LSM6DSL_GYRO_FIFO_EN        (1<<0)
#define LSM6DSL_ACCE_FIFO_EN        (1<<1)
#define LSM6DSL_TEMP_FIFO_EN        (1<<2)
#define LSM6DSL_PEDO_FIFO_EN        (1<<3)

typedef union
{
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union
{
    int16_t i16bit;
    uint8_t u8bit[2];
} axis1bit16_t;

struct LSM6DSL_CONT_T
{
#if (LSM6DSL_USING_I2C == 1)
    struct rt_i2c_bus_device *bus_handle;
#else   // SPI
    struct rt_spi_device *bus_handle;
#endif  // LSM6DSL_USING_I2C
    void *parent;
    uint8_t open_flag;
    uint8_t dev_addr;
    uint8_t whoami;
    uint8_t data_valid; // to notify data ready for each sensor 1 bit
    uint8_t fifo_en;    // to save fifo mode enable for each sensor, 1 bit
    uint16_t fifo_odr;  // to save each ODR, 4 bits for each sensor, it can decide fifo output data array
    axis3bit16_t gyro_value;
    axis3bit16_t acce_value;
    axis1bit16_t temp_value;
    axis1bit16_t pedo_value;
};

static sifdev_sensor_ctx_t sens_cont;
static struct LSM6DSL_CONT_T lsm6dsl_content;
#ifdef LSM6DSL_USE_INT
    static struct rt_semaphore lsm_int_sem;
    rt_thread_t lsm6d_thread = NULL;
#endif
extern rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name);

static int lsm6dsl_power_onoff(uint8_t on)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_err_t ret = RT_EOK;
#if (LSM6DSL_POW_PIN >= 0)
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        //rt_kprintf("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;

    m.pin = LSM6DSL_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = LSM6DSL_POW_PIN;
    st.status = on;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
#endif
    return ret;

}
/***
 * @name       lsm6dsl_pmic_control(bool en)
 * @brief       pmic control power for gsensor .
 * @param       bool en :  true   open;false close

 * @output
 * @retval      void
 ***/
#if defined(PMIC_CONTROL_SERVICE)

void lsm6dsl_pmic_control(bool en)
{
    pmic_control_t pow_gsensor;
    pmic_service_control(PMIC_CONTROL_GSENSOR, en);
    rt_kprintf("lsm6dsl power :%d\n", en);
    return;
}
#endif
//#define USER_CTL_CS
static int lsm6dsl_i2c_init()
{
    rt_err_t rst = RT_EOK;
#if (LSM6DSL_USING_I2C == 1)

    /* get i2c bus device */
    lsm6dsl_content.bus_handle = rt_i2c_bus_device_find(LSM6DSL_BUS_NAME);

    if (lsm6dsl_content.bus_handle)
    {
        LOG_D("Find i2c bus device %s\n", LSM6DSL_BUS_NAME);

        rst = rt_device_open((rt_device_t)lsm6dsl_content.bus_handle, RT_DEVICE_FLAG_RDWR);

        if (RT_EOK != rst)
        {
            rt_kprintf("lsm6dsl_i2c open err!");
            return -2;
        }
    }
    else
    {
        rt_kprintf("Can not found i2c bus %s, lsm6dsl init fail\n", LSM6DSL_BUS_NAME);
        return 1;
    }

#else   // SPI
    /* get i2c bus device */
    rt_err_t ret;
    rt_device_t spi_bus = rt_device_find(LSM6DSL_BUS_NAME);
    //lsm6dsl_content.bus_handle = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));

    if (spi_bus)
    {
        struct rt_spi_configuration cfg1;
        LOG_D("Find spi bus %s\n", LSM6DSL_BUS_NAME);
        lsm6dsl_content.bus_handle = (struct rt_spi_device *)rt_device_find(LSM6DSL_DEV_NAME);

        if (lsm6dsl_content.bus_handle == NULL)
        {
            ret = rt_hw_spi_device_attach(LSM6DSL_BUS_NAME, LSM6DSL_DEV_NAME);

            lsm6dsl_content.bus_handle = (struct rt_spi_device *)rt_device_find(LSM6DSL_DEV_NAME);

            if (lsm6dsl_content.bus_handle == NULL)
            {
                LOG_D("Register LSM6DSL spi device fail\n");
                return -1;
            }
        }
        ret = rt_device_open((rt_device_t)(lsm6dsl_content.bus_handle), RT_DEVICE_FLAG_RDWR);
        if (ret != RT_EOK)
        {
            LOG_D("open %s fail\n", LSM6DSL_DEV_NAME);
            return -1;
        }

        //rt_spi_bus_attach_device(lsm6dsl_content.bus_handle, "spi_lsm", LSM6DSL_BUS_NAME, NULL);

        //rt_spi_take_bus(lsm6dsl_content.bus_handle);
        //rt_spi_take(lsm6dsl_content.bus_handle);
        //rt_spi_release_bus(lsm6dsl_content.bus_handle);
#ifndef USER_CTL_CS
        cfg1.data_width = 16; //8; //16;    // auto cs, need total 16 bits to read data in low 8 bits
#else
        cfg1.data_width = 8; //8; //16;
#endif
        cfg1.max_hz = 12 * 1000 * 1000; // 6m
        //cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_SLAVE ;
        cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_MASTER ;
        cfg1.frameMode = RT_SPI_MOTO; //RT_SPI_TI;

        ret = rt_spi_configure(lsm6dsl_content.bus_handle, &cfg1);
        if (ret != RT_EOK)
            rt_kprintf("rt_spi_configure fail.");
        ret = rt_spi_take_bus(lsm6dsl_content.bus_handle);
        if (ret != RT_EOK)
            rt_kprintf("rt_spi_take_bus fail.");
        //rt_spi_take(lsm6dsl_content.bus_handle);

    }
    else
    {
        rt_kprintf("Can not found spi bus %s, init fail\n", LSM6DSL_BUS_NAME);
        return -1;
    }

#endif  // LSM6DSL_USING_I2C
    return 0;
}

static void lsm6dsl_i2c_deinit(void)
{
    if (lsm6dsl_content.bus_handle != NULL)
        rt_device_close((rt_device_t)lsm6dsl_content.bus_handle);
}

static int32_t lsm_i2c_write(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res;

    struct LSM6DSL_CONT_T *handle = (struct LSM6DSL_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
#if (LSM6DSL_USING_I2C == 1)
        uint16_t addr16 = (uint16_t)reg;
        res = rt_i2c_mem_write(handle->bus_handle, handle->dev_addr, addr16, 8, data, len);
        if (res > 0)
            return 0;
        else
            return -2;
#else // SPI
#ifndef USER_CTL_CS

        uint16_t addr16 = (uint16_t)reg;
        uint16_t buf;
        uint16_t cnt = len;

        //LOG_I("Write addr 0x%x , len %d\n", reg, len);
        while (cnt > 0)
        {
            buf = (addr16 << 8) | *data;
            rt_size_t res = rt_spi_transfer(handle->bus_handle, &buf, NULL, 1);
            if (res == 0)
            {
                LOG_I("SPI transmit fail \n");
                return -1;
            }
            cnt--;
            data++;
            addr16++;
        }
        return 0;
#else
        uint16_t addr = reg;
        rt_err_t ret = rt_spi_send_then_send(handle->bus_handle, &addr, 1, data, len);
        if (ret == 0)
            return 0;
        else
            return -2;
#endif
#endif
    }

    return -3;
}

static int32_t lsm_i2c_read(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res;
    struct LSM6DSL_CONT_T *handle = (struct LSM6DSL_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
#if (LSM6DSL_USING_I2C == 1)
        uint16_t addr16 = (uint16_t)reg;
        res = rt_i2c_mem_read(handle->bus_handle, handle->dev_addr, addr16, 8, data, len);
        if (res > 0)
            return 0;
        else
            return -2;
#else // SPI
#ifndef USER_CTL_CS

        uint16_t addr16 = (uint16_t)reg;
        uint16_t cnt = len;
        uint16_t buf;
        uint16_t value;

        //rt_kprintf("Read addr 0x%x , len %d\n", reg, len);
        while (cnt > 0)
        {
            buf = (addr16 | 0x80) << 8;

            rt_size_t res = rt_spi_transfer(handle->bus_handle, &buf, &value, 1);
            //rt_kprintf("buf value res 0x%x 0x%x 0x%x\n", buf, value, res); // for test
            if (res == 0)
            {
                rt_kprintf("SPI transmit fail \n");
                return -1;
            }
            *data = (uint16_t)(value & 0xff);
            cnt--;
            addr16++;
            data++;
        }
        return 0;
#else
        rt_err_t ret = RT_EOK;
        uint16_t buf;
        buf = reg | 0x80;

        ret = rt_spi_send_then_recv(handle->bus_handle, &buf, 1, data, len);

        if (ret == RT_EOK)
        {
            //LOG_D("SPI read data 0x%x\n", *data);
            return 0;
        }
        else
        {
            rt_kprintf("SPI read fail %d\n", ret);
            return -2;
        }
#endif
#endif
    }

    return -3;
}

#ifdef LSM6DSL_USE_INT

/* LSM6DSL interrupt init, it use gpio input as int */
// check edge
static void lsm6dsl_int1_handle(void *args)
{
    rt_kprintf("lsm6dsl_int1_handle\n");
    rt_sem_release(&lsm_int_sem);
#if 0
    //int value = (int)args;
    //LOG_I("lsm6dsl_int_handle %d\n", value);
    lsm6dsl_int1_route_t value = {0};
    lsm6dsl_pin_int1_route_get(&sens_cont, &value);
    if (value.int1_step_detector)
    {
        LOG_I("step detect\n");
    }
    if (value.int1_full_flag)
    {
        LOG_I("fifo full\n");
    }
    if (value.int1_fifo_ovr)
    {
        LOG_I("Fifo overflow\n");
    }
    if (value.int1_fth)
    {
        LOG_I("Fifo threshold\n");
    }
#endif
}

static void lsm6dsl_int2_handle(void *args)
{
    rt_kprintf("lsm6dsl_int2_handle\n");
    rt_sem_release(&lsm_int_sem);
}

static int lsm6dsl_gpio_int_enable(void)
{
    struct rt_device_pin_mode m;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        rt_kprintf("GPIO pin device not found at LSM6DSL\n");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
#if LSM6DSL_UES_INT1
    // int pin cfg
    m.pin = LSM6DSL_INT_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable LSM int
    rt_pin_mode(LSM6DSL_INT_GPIO_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, lsm6dsl_int1_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif
    // for int2
#if LSM6DSL_UES_INT2
    // int pin cfg
    m.pin = LSM6DSL_INT2_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable LSM int
    rt_pin_mode(LSM6DSL_INT2_GPIO_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, lsm6dsl_int2_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif

    return 0;

}

static int lsm6dsl_gpio_int_disable(void)
{
    struct rt_device_pin_mode m;

#if LSM6DSL_UES_INT1
    // int pin cfg
    m.pin = LSM6DSL_INT_GPIO_BIT;

    // enable LSM int
    //rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, lsm6dsl_int_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 0);
#endif
    // for int2
#if LSM6DSL_UES_INT2
    // int pin cfg
    m.pin = LSM6DSL_INT2_GPIO_BIT;
    rt_pin_irq_enable(m.pin, 0);

#endif

    return 0;

}


void lsm6dsl_sensor_task(void *params)  //20ms
{
    int32_t ret;
    lsm6dsl_all_sources_t alsrc;

    // add by whj for thread loop
    rt_kprintf("lsm6dsl_sensor_task1\n");
    while (1)
    {
        //rt_thread_delay(20);    // no irq, use delay instead
        rt_sem_take(&lsm_int_sem, RT_WAITING_FOREVER) ;
        //rt_kprintf("lsm6dsl_sensor_task2\r\n");
        memset(&alsrc, 0, sizeof(lsm6dsl_all_sources_t));
        lsm6dsl_all_sources_get(&sens_cont, &alsrc);

        if (alsrc.func_src1.step_detected)
        {
            //LOG_I("step detect\n");
            ret = lsm6dsl_step_counter_get(&sens_cont, lsm6dsl_content.pedo_value.u8bit);
            LOG_D("step counter = %d\n", lsm6dsl_content.pedo_value.i16bit);
            if (ret == 0)
                lsm6dsl_content.data_valid |= LSM6DSL_STEP_DATA_EN;
        }
        if (alsrc.func_src2.wrist_tilt_ia)
        {
            LOG_D("wrist tilt reason: %d, %d, %d, %d, %d, %d\n", alsrc.a_wrist_tilt_mask.wrist_tilt_mask_xneg, alsrc.a_wrist_tilt_mask.wrist_tilt_mask_xpos,
                  alsrc.a_wrist_tilt_mask.wrist_tilt_mask_yneg, alsrc.a_wrist_tilt_mask.wrist_tilt_mask_ypos,
                  alsrc.a_wrist_tilt_mask.wrist_tilt_mask_zneg, alsrc.a_wrist_tilt_mask.wrist_tilt_mask_zpos);
        }
        if (alsrc.status_reg.xlda)
        {
            //LOG_D("acc data rdy\n");
            memset(lsm6dsl_content.acce_value.u8bit, 0x00, 3 * sizeof(int16_t));
            ret = lsm6dsl_acceleration_raw_get(&sens_cont, lsm6dsl_content.acce_value.u8bit);
            if (ret == 0)
                lsm6dsl_content.data_valid |= LSM6DSL_ACCE_DATA_EN;
            else
                lsm6dsl_content.data_valid &= (~LSM6DSL_ACCE_DATA_EN);
        }
        if (alsrc.status_reg.gda)
        {
            //LOG_D("gyro data rdy\n");
            memset(lsm6dsl_content.gyro_value.u8bit, 0x00, 3 * sizeof(int16_t));
            ret = lsm6dsl_angular_rate_raw_get(&sens_cont, lsm6dsl_content.gyro_value.u8bit);
            if (ret == 0)
                lsm6dsl_content.data_valid |= LSM6DSL_GYRO_DATA_EN;
            else
                lsm6dsl_content.data_valid &= (~LSM6DSL_GYRO_DATA_EN);
        }
        if (alsrc.status_reg.tda)
        {
            //LOG_D("temperatue data rdy\n");
            memset(lsm6dsl_content.temp_value.u8bit, 0x00, sizeof(int16_t));
            ret = lsm6dsl_temperature_raw_get(&sens_cont, lsm6dsl_content.temp_value.u8bit);
            if (ret == 0)
                lsm6dsl_content.data_valid |= LSM6DSL_TEMP_DATA_EN;
            else
                lsm6dsl_content.data_valid &= (~LSM6DSL_TEMP_DATA_EN);
        }
        if (alsrc.wrist_tilt_ia.wrist_tilt_ia_xneg)
        {
            LOG_D("awt xneg\n");
        }
        if (alsrc.wake_up_src.ff_ia)
        {
            LOG_D("free fall\n");
        }
        if (alsrc.wake_up_src.sleep_state_ia)
        {
            LOG_D("sleep\n");
        }
        if (alsrc.wake_up_src.wu_ia)
        {
            LOG_D("wake up\n");
        }
        if (alsrc.wake_up_src.x_wu)
        {
            LOG_D("x sleep\n");
        }
        if (alsrc.wake_up_src.y_wu)
        {
            LOG_D("y sleep\n");
        }
        if (alsrc.wake_up_src.z_wu)
        {
            LOG_D("z sleep\n");
        }

    }
}
#endif // LSM6DSL_USE_INT

int lsm6dsl_init(void)
{
    int res;

    lsm6dsl_content.dev_addr = LSM6DSL_I2C_ADDR_L;
    lsm6dsl_content.bus_handle = NULL;
    lsm6dsl_content.whoami = 0;
    lsm6dsl_content.parent = &sens_cont;
    memset(lsm6dsl_content.gyro_value.u8bit, 0x00, 3 * sizeof(int16_t));
    memset(lsm6dsl_content.acce_value.u8bit, 0x00, 3 * sizeof(int16_t));
    memset(lsm6dsl_content.temp_value.u8bit, 0x00, sizeof(int16_t));
    memset(lsm6dsl_content.pedo_value.u8bit, 0x00, sizeof(int16_t));
    lsm6dsl_content.data_valid = 0;
    lsm6dsl_content.fifo_en = 0;
    lsm6dsl_content.fifo_odr = 0;
    lsm6dsl_content.open_flag = 0;

    lsm6dsl_power_onoff(1);
#if defined(PMIC_CONTROL_SERVICE)

    //opengsensor 3.3v
    lsm6dsl_pmic_control(true);
#endif
    rt_thread_mdelay(10);

    res = lsm6dsl_i2c_init();
    if (res != 0)
    {
        rt_kprintf("lsm6dsl_i2c_init fail\n");
        lsm6dsl_i2c_deinit();
        return res;
    }

    sens_cont.read_reg = lsm_i2c_read;
    sens_cont.write_reg = lsm_i2c_write;
    sens_cont.handle = (void *)&lsm6dsl_content;

    // check who am I and try i2c slave address
    res = lsm6dsl_device_id_get(&sens_cont, &(lsm6dsl_content.whoami));
    //rt_kprintf("res 1 = %d when address = %d\n",res,lsm6dsl_content.whoami);
    if (lsm6dsl_content.whoami != LSM6DSL_ID)
    {
        // try another address
        lsm6dsl_content.dev_addr = LSM6DSL_I2C_ADDR_H;
        res = lsm6dsl_device_id_get(&sens_cont, &(lsm6dsl_content.whoami));
        rt_kprintf("res 2 = %d when address = %d\n", res, lsm6dsl_content.whoami);
        if (lsm6dsl_content.whoami != LSM6DSL_ID)
        {
            // 2 address all fail
            //rt_kprintf("LSM6DSL I2C SLAVE init fail, erro id=0x%x\n",lsm6dsl_content.whoami);
            lsm6dsl_i2c_deinit();
            return 1;
        }
    }

    rt_kprintf("LSM6DSL init done, ID= 0x%x (should be 0x6A)\n", lsm6dsl_content.whoami);

    return 0;
}

int lsm6dsl_open(void)
{
    int res;
    uint8_t rst;

    if (lsm6dsl_content.open_flag == 1) // opened before
        return 0;

    // clear previous value
    memset(lsm6dsl_content.gyro_value.u8bit, 0x00, 3 * sizeof(int16_t));
    memset(lsm6dsl_content.acce_value.u8bit, 0x00, 3 * sizeof(int16_t));
    memset(lsm6dsl_content.temp_value.u8bit, 0x00, sizeof(int16_t));
    memset(lsm6dsl_content.pedo_value.u8bit, 0x00, sizeof(int16_t));
    lsm6dsl_content.data_valid = 0;

    /*
     *  Restore default configuration
     */
    res = lsm6dsl_reset_set(&sens_cont, PROPERTY_ENABLE);
    do
    {
        res = lsm6dsl_reset_get(&sens_cont, &rst);
    }
    while (rst);
    //LOG_I("Reset res = %d, rst = %d\n",res, rst);
    /*
     * Set full scale
     */
    lsm6dsl_xl_full_scale_set(&sens_cont, LSM6DSL_16g);
#ifdef USING_GYRO_SENSOR
    lsm6dsl_gy_full_scale_set(&sens_cont, LSM6DSL_2000dps);
#endif
    /*
     *  Enable Block Data Update
     */
    lsm6dsl_block_data_update_set(&sens_cont, PROPERTY_ENABLE);
    /*
     * Set Output Data Rate, for awt and pedometer, acc odr should not less than 26hz
     */
    //lsm6dsl_xl_data_rate_set(&sens_cont, LSM6DSL_XL_ODR_26Hz);
    //lsm6dsl_gy_data_rate_set(&sens_cont, LSM6DSL_GY_ODR_12Hz5);
    lsm6dsl_xl_data_rate_set(&sens_cont, LSM6DSL_XL_ODR_52Hz);
#ifdef USING_GYRO_SENSOR
    lsm6dsl_gy_data_rate_set(&sens_cont, LSM6DSL_GY_ODR_52Hz);
#endif

    lsm6dsl_xl_power_mode_set(&sens_cont, LSM6DSL_XL_HIGH_PERFORMANCE);
#ifdef USING_GYRO_SENSOR
    lsm6dsl_gy_power_mode_set(&sens_cont, LSM6DSL_GY_HIGH_PERFORMANCE);
#endif

#if defined (GSENSOR_UES_FIFO)
#ifdef USING_GYRO_SENSOR
    //fifo odr only a reg, so if to set diff odr, this samller one is in front.
    lsm6dsl_fifo_enable(LSM6DSL_FIFO_GYRO, LSM6DSL_FIFO_ODR_52Hz);
    lsm6dsl_fifo_enable(LSM6DSL_FIFO_XL, LSM6DSL_FIFO_ODR_52Hz);

    lsm6dsl_set_fifo_threshold(52 * 3 * 2);  // in according to 1s get once data, 52hz x 3axis(threshold LSB=2bytes), acce and gyro
#else
    lsm6dsl_fifo_enable(LSM6DSL_FIFO_XL, LSM6DSL_FIFO_ODR_52Hz);
    lsm6dsl_set_fifo_threshold(52 * 3);
#endif
#endif

#ifdef LSM_USING_AWT
    lsm6dsl_awt_enable(1);
#endif /*  LSM_USING_AWT */

#ifdef LSM_USING_PEDO
    lsm6dsl_pedo_enable(1);
#endif /*  LSM_USING_PEDO */

    /*
     * Configure filtering chain(No aux interface)
     */
    /* Accelerometer - analog filter */
    //lsm6dsl_xl_filter_analog_set(&sens_cont, LSM6DSL_XL_ANA_BW_400Hz);

    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    //lsm6dsl_xl_lp1_bandwidth_set(&sens_cont, LSM6DSL_XL_LP1_ODR_DIV_4);

    /* Accelerometer - LPF1 + LPF2 path */
    //lsm6dsl_xl_lp2_bandwidth_set(&sens_cont, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);

    /* Accelerometer - High Pass / Slope path */
    //lsm6dsl_xl_reference_mode_set(&sens_cont, PROPERTY_DISABLE);
    //lsm6dsl_xl_hp_bandwidth_set(&sens_cont, LSM6DSL_XL_HP_ODR_DIV_100);

    /* Gyroscope - filtering chain */
    //lsm6dsl_gy_band_pass_set(&sens_cont, LSM6DSL_HP_260mHz_LP1_STRONG);

#ifdef LSM6DSL_USE_INT
    // start a thread to check data available
    rt_sem_init(&lsm_int_sem, "lsm_int", 0, RT_IPC_FLAG_FIFO);
    lsm6d_thread = rt_thread_create("lsm6d", lsm6dsl_sensor_task, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (lsm6d_thread != NULL)
    {
        rt_thread_startup(lsm6d_thread);
        LOG_D("6d thread started\n");
    }
    else
        rt_kprintf("Create 6d thread fail\n");

    lsm6dsl_gpio_int_enable();

    //if (lsm6dsl_content.int_en == 1) // interrupt enable, so gyro and acce data ready should be used.
    {
#if LSM6DSL_UES_INT1
        lsm6dsl_int1_route_t value1;
        //memset(&value1, 0, sizeof(lsm6dsl_int1_route_t));
        lsm6dsl_pin_int1_route_get(&sens_cont, &value1);
        value1.int1_drdy_xl = 1;
        value1.int1_drdy_g = 1;
        lsm6dsl_pin_int1_route_set(&sens_cont, value1);
#endif

#if LSM6DSL_UES_INT2
        lsm6dsl_int2_route_t value2;
        //memset(&value2, 0, sizeof(lsm6dsl_int1_route_t));
        lsm6dsl_pin_int2_route_get(&sens_cont, &value2);
        value2.int2_drdy_xl = 1;
        value2.int2_drdy_g = 1;
        lsm6dsl_pin_int2_route_set(&sens_cont, value2);
#endif
    }
#endif
    lsm6dsl_content.open_flag = 1;
    rt_kprintf("open gsesnor  lsm6dsl ok.\n");

    return 0;
}

int lsm6dsl_close(void)
{
    int res;

    if (lsm6dsl_content.open_flag == 0) // closed before
        return 0;

#ifdef LSM6DSL_USE_INT
    lsm6dsl_gpio_int_disable();
    rt_sem_release(&lsm_int_sem);

    if (lsm6d_thread != NULL)
    {
        rt_thread_delete(lsm6d_thread);
        lsm6d_thread = NULL;
    }
#endif

#ifdef LSM_USING_AWT
    /*
     * Disable AWT
     */
    //lsm6dsl_wrist_tilt_sens_set(&sens_cont, 0);
    lsm6dsl_awt_enable(0);
#endif
#ifdef LSM_USING_PEDO
    lsm6dsl_pedo_enable(0);
#endif /*  LSM_USING_PEDO */


    lsm6dsl_xl_power_mode_set(&sens_cont, LSM6DSL_XL_NORMAL);
#ifdef USING_GYRO_SENSOR
    lsm6dsl_gy_power_mode_set(&sens_cont, LSM6DSL_GY_NORMAL);
#endif
    res = lsm6dsl_reset_set(&sens_cont, PROPERTY_DISABLE);

    lsm6dsl_content.fifo_en = 0;
    lsm6dsl_content.fifo_odr = 0;
    lsm6dsl_content.open_flag = 0;

    lsm6dsl_power_onoff(0);
#if defined(PMIC_CONTROL_SERVICE)

    //close gsensor 3.3v
    lsm6dsl_pmic_control(false);
#endif

    return 0;
}

// functions for fifo --------------------------------------------------
int lsm6dsl_fifo_enable(lsm6dsl_fifo_func_t func, lsm6dsl_fifo_odr_t rate)
{
    lsm6dsl_odr_fifo_t odr1, odr2;

    // check if function enabled before, if enabled, return directly, do not set odr
    if (lsm6dsl_content.fifo_en & (1 << ((int)func)))
        return 0;

    // Todo, when pedometer enable, just support step detect write, or fifo overflow too quickly
    //if(lsm6dsl_content.fifo_en & LSM6DSL_PEDO_FIFO_EN)
    //    lsm6dsl_fifo_data_rate_set(&sens_cont, LSM6DSL_FIFO_DISABLE);

    lsm6dsl_fifo_data_rate_get(&sens_cont, &odr1);  // get current odr
    odr2 = (lsm6dsl_odr_fifo_t)(rate & 0xf);
    //LOG_D("Old ODR %d, new ODR %d\n", (int)odr1, (int)odr2);
    //if (odr2 > odr1) // set to high rate, and make sure function opened before should changed correctly
    //{
    //    lsm6dsl_fifo_data_rate_set(&sens_cont, odr2);
    //}

    switch (func)
    {
    case LSM6DSL_FIFO_GYRO:
    {
        lsm6dsl_content.fifo_en |= LSM6DSL_GYRO_FIFO_EN;    // enable flag
        lsm6dsl_content.fifo_odr &= 0xfff0;
        lsm6dsl_content.fifo_odr |= odr2;   // save gyro fifo odr

        lsm6dsl_gy_power_mode_set(&sens_cont, LSM6DSL_GY_NORMAL);
        if (odr2 > odr1) // with high rate, set to high rate and no dec
        {
            lsm6dsl_fifo_data_rate_set(&sens_cont, odr2);
            lsm6dsl_fifo_gy_batch_set(&sens_cont, LSM6DSL_FIFO_GY_NO_DEC);
        }
        else // with dec, do not change odr
        {
            int del = odr1 - odr2 + 1; // increase 1, odr double, base 1
            lsm6dsl_fifo_gy_batch_set(&sens_cont, (lsm6dsl_dec_fifo_gyro_t) del);
        }
        //LOG_D("GYRO fifo enable\n");
        break;
    }
    case LSM6DSL_FIFO_XL:
    {
        lsm6dsl_content.fifo_en |= LSM6DSL_ACCE_FIFO_EN;    // set flag
        lsm6dsl_content.fifo_odr &= 0xff0f;
        lsm6dsl_content.fifo_odr |= (odr2 << 4); // save acce fifo odr

        lsm6dsl_xl_power_mode_set(&sens_cont, LSM6DSL_XL_NORMAL);
        if (odr2 > odr1) // with high rate, no dec
        {
            lsm6dsl_fifo_data_rate_set(&sens_cont, odr2);
            lsm6dsl_fifo_xl_batch_set(&sens_cont, LSM6DSL_FIFO_XL_NO_DEC);
        }
        else // with dec
        {
            int del = odr1 - odr2 + 1; // increase 1, odr double, base 1
            lsm6dsl_fifo_xl_batch_set(&sens_cont, (lsm6dsl_dec_fifo_xl_t) del);
        }
        //LOG_D("Accelerator fifo enable\n");
        break;
    }
    case LSM6DSL_FIFO_TEMP:
    {
        break;
    }
    case LSM6DSL_FIFO_STEP:
    {
        lsm6dsl_content.fifo_en |= LSM6DSL_PEDO_FIFO_EN;
        lsm6dsl_content.fifo_odr &= 0xfff;
        //TODO pedometer do not support odr now, add it later if need
        //lsm6dsl_content.fifo_odr |= (odr2<<12);   // save pedometer odr

        lsm6dsl_fifo_dataset_4_batch_set(&sens_cont, LSM6DSL_FIFO_DS4_NO_DEC);
        lsm6dsl_fifo_pedo_and_timestamp_batch_set(&sens_cont, 1);
        lsm6dsl_fifo_write_trigger_set(&sens_cont, LSM6DSL_TRG_STEP_DETECT); // step triger fifo write
        // for step triger, ODR seems no use, clear it?
        lsm6dsl_fifo_data_rate_set(&sens_cont, odr2);
        //LOG_D("Pedometer fifo enable\n");
        break;
    }
    default:
        break;
    }

    lsm6dsl_fifo_mode_set(&sens_cont, LSM6DSL_FIFO_MODE);

    return 0;
}

int lsm6dsl_fifo_disable(lsm6dsl_fifo_func_t func)
{
    switch (func)
    {
    case LSM6DSL_FIFO_GYRO:
    {
        lsm6dsl_fifo_gy_batch_set(&sens_cont, LSM6DSL_FIFO_GY_DISABLE);
        lsm6dsl_gy_power_mode_set(&sens_cont, LSM6DSL_GY_HIGH_PERFORMANCE);
        lsm6dsl_content.fifo_en &= ~LSM6DSL_GYRO_FIFO_EN;    // disable flag
        lsm6dsl_content.fifo_odr &= 0xfff0; // clear odr
        //LOG_D("GYRO fifo disable\n");
        break;
    }
    case LSM6DSL_FIFO_XL:
    {
        lsm6dsl_fifo_xl_batch_set(&sens_cont, LSM6DSL_FIFO_XL_DISABLE);
        lsm6dsl_xl_power_mode_set(&sens_cont, LSM6DSL_XL_HIGH_PERFORMANCE);
        lsm6dsl_content.fifo_en &= ~LSM6DSL_ACCE_FIFO_EN;    // disable flag
        lsm6dsl_content.fifo_odr &= 0xff0f; // clear odr
        //LOG_D("ACCE fifo disable\n");
        break;
    }
    case LSM6DSL_FIFO_STEP:
    {
        lsm6dsl_content.fifo_en &= ~LSM6DSL_PEDO_FIFO_EN;
        lsm6dsl_content.fifo_odr &= 0xfff;

        lsm6dsl_fifo_dataset_4_batch_set(&sens_cont, LSM6DSL_FIFO_DS4_DISABLE);
        lsm6dsl_fifo_pedo_and_timestamp_batch_set(&sens_cont, 0);
        lsm6dsl_fifo_write_trigger_set(&sens_cont, LSM6DSL_TRG_XL_GY_DRDY); // step triger fifo write
        //LOG_D("STEP fifo disable\n");
        break;
    }
    default:
        break;
    }

    if (lsm6dsl_content.fifo_en == 0)   // all module fifo mode closed
        lsm6dsl_fifo_mode_set(&sens_cont, LSM6DSL_BYPASS_MODE);

    return 0;
}

int lsm6dsl_get_fifo_count(void)
{
    uint16_t value;

    lsm6dsl_block_data_update_set(&sens_cont, 1);
    lsm6dsl_fifo_data_level_get(&sens_cont, &value);

    return (int)value;
}

int lsm6dsl_read_fifo(uint8_t *buf, int len)
{
    if (buf == NULL || len == 0)
        return 0;

    for (int i = 0; i < len / 2; i++)
    {
        lsm6dsl_fifo_raw_data_get(&sens_cont, buf + i * 2, 2);
    }

    return len;
}



int lsm6dsl_set_fifo_threshold(int thd)
{
    // fifo default thd 2048, it can be changed if stop_on_fth set
    // do not set stop_on_fth, this value only affect water mark flag
    lsm6dsl_fifo_stop_on_wtm_set(&sens_cont, 1);
    return lsm6dsl_fifo_watermark_set(&sens_cont, thd & 0x7ff);
}

int lsm6dsl_get_waterm_status(void)
{
    uint8_t value;
    lsm6dsl_fifo_wtm_flag_get(&sens_cont, &value);

    return (int)value;
}

int lsm6dsl_get_overrun_status(void)
{
    uint8_t value;
    lsm6dsl_fifo_overrun_flag_get(&sens_cont, &value);

    return (int)value;
}

int lsm6dsl_get_fifo_full_status(void)
{
    uint8_t value;
    lsm6dsl_fifo_fullsmart_flag_get(&sens_cont, &value);

    return (int)value;
}

int lsm6dsl_get_fifo_empty_status(void)
{
    uint8_t value;
    lsm6dsl_fifo_empty_flag_get(&sens_cont, &value);

    return (int)value;
}

int lsm6dsl_set_fifo_mode(lsm6dsl_fifo_mode_t val)
{
    int32_t ret = lsm6dsl_fifo_mode_set(&sens_cont, val);
    return ret;
}

int lsm6dsl_get_fifo_pattern(void)
{
    uint16_t value;
    lsm6dsl_fifo_pattern_get(&sens_cont, &value);
    return (int)value;
}

int lsm6dsl_get_fifo_data_arr(void)
{
    return (int)lsm6dsl_content.fifo_odr;
}

// function for awt
int lsm6dsl_awt_enable(int en)
{
    int res;
    // enable AWT fucntion, other configure seems not work
    uint8_t value;
    lsm6dsl_a_wrist_tilt_mask_t wt_mask;
    lsm6dsl_int2_route_t intval;

    if (en != 0) // enable
    {
        wt_mask.wrist_tilt_mask_xneg = 1;
        wt_mask.wrist_tilt_mask_xpos = 1;
        wt_mask.wrist_tilt_mask_yneg = 1;
        wt_mask.wrist_tilt_mask_ypos = 1;
        wt_mask.wrist_tilt_mask_zneg = 1;
        wt_mask.wrist_tilt_mask_zpos = 1;

        // make sure accelerator ODR larger than 26hz
        //lsm6dsl_xl_data_rate_set(&sens_cont, LSM6DSL_XL_ODR_26Hz);

        /*
         * Set tilt mask
         */
        res = lsm6dsl_tilt_src_set(&sens_cont, &wt_mask);
        if (res != 0)
            LOG_D("lsm6dsl_tilt_src_set fail\n");

        value = 10;
        res = lsm6dsl_tilt_latency_set(&sens_cont, &value);
        if (res != 0)
            LOG_D("lsm6dsl_tilt_latency_set fail\n");

        value = 10;
        res = lsm6dsl_tilt_threshold_set(&sens_cont, &value);
        if (res != 0)
            LOG_D("lsm6dsl_tilt_threshold_set fail\n");
        /*
         * Enable AWT
         */
        lsm6dsl_wrist_tilt_sens_set(&sens_cont, 1);
        // enable interrupt
#ifdef LSM6DSL_USE_INT
        //memset(&intval, 0, sizeof(lsm6dsl_int2_route_t));
        lsm6dsl_pin_int2_route_get(&sens_cont, &intval);
        intval.int2_wrist_tilt = 1;
        lsm6dsl_pin_int2_route_set(&sens_cont, intval);
#endif
    }
    else // disable
    {
        /*
         * Disable AWT
         */
#ifdef LSM6DSL_USE_INT

        //memset(&intval, 0, sizeof(lsm6dsl_int2_route_t));
        lsm6dsl_pin_int2_route_get(&sens_cont, &intval);
        intval.int2_wrist_tilt = 0;
        lsm6dsl_pin_int2_route_set(&sens_cont, intval);
#endif
        lsm6dsl_wrist_tilt_sens_set(&sens_cont, 0);
    }

    return 0;
}


// function for pedometer ----------------------------
int lsm6dsl_pedo_enable(int en)
{
    lsm6dsl_int1_route_t value;
    if (en != 0) // enable
    {
        // make sure accelerator ODR larger than 26hz
        //lsm6dsl_xl_data_rate_set(&sens_cont, LSM6DSL_XL_ODR_26Hz);

        // pedo fs change to +- 4g?
        //lsm6dsl_pedo_full_scale_set(&sens_cont,1);
        //lsm6dsl_pedo_threshold_set(&sens_cont,14);
        //lsm6dsl_xl_full_scale_set(&sens_cont, LSM6DSL_4g);

        // enable function and pedometer
        lsm6dsl_pedo_sens_set(&sens_cont, 1);
        //lsm6dsl_tilt_sens_set(&sens_cont,1);

        // reset the step counter to 0, it will not reset after function disable
        lsm6dsl_pedo_step_reset_set(&sens_cont, 1);
        lsm6dsl_pedo_step_reset_set(&sens_cont, 0);
#ifdef LSM6DSL_USE_INT
        // enable interrupt
        //memset(&value, 0, sizeof(lsm6dsl_int1_route_t));
        lsm6dsl_pin_int1_route_get(&sens_cont, &value);
        value.int1_step_detector = 1;
        lsm6dsl_pin_int1_route_set(&sens_cont, value);
#endif
    }
    else // disable
    {
#ifdef LSM6DSL_USE_INT
        // disable interrupt
        //memset(&value, 0, sizeof(lsm6dsl_int1_route_t));
        lsm6dsl_pin_int1_route_get(&sens_cont, &value);
        value.int1_step_detector = 0;
        lsm6dsl_pin_int1_route_set(&sens_cont, value);
#endif
        // disable function and pedometer
        lsm6dsl_pedo_sens_set(&sens_cont, 0);
        //lsm6dsl_tilt_sens_set(&sens_cont,1);

        // reset the step counter to 0, it will not reset after function disable
        lsm6dsl_pedo_step_reset_set(&sens_cont, 1);
        lsm6dsl_pedo_step_reset_set(&sens_cont, 0);
    }

    return 0;
}

int lsm6dsl_pedo_fifo2step(uint8_t *buf, int len)
{
    /*
        just for only pedo mode.
        1 array with 6 bytes
        3 bytes for timestamp, 1 byte not used, 2 byte for number of steps
        return the latest step
    */
    int i, cnt;
    uint16_t *tbuf = (uint16_t *)buf;
    uint32_t timestamp;
    uint16_t step = 0;

    if (buf == NULL || len == 0)
        return 0;

    cnt = len / 6;

    for (i = 0; i < cnt; i++)
    {
        timestamp = (uint32_t)(*tbuf);
        timestamp = (uint32_t)((*(tbuf + 1)) >> 8) | (timestamp << 8);
        step = *(tbuf + 2);
        LOG_D("T %d, S %d\n", timestamp, step);
        tbuf += 3;
    }

    return (int)step;
}

// function for handle and address --------------------
uint32_t lsm6dsl_get_bus_handle(void)
{
    return (uint32_t)(lsm6dsl_content.bus_handle);
}
uint8_t lsm6dsl_get_dev_addr(void)
{
    return lsm6dsl_content.dev_addr;
}

uint8_t lsm6dsl_get_dev_id(void)
{
    return lsm6dsl_content.whoami;
}

int lsm6dsl_self_check(void)
{
    int ret = 0;
    uint8_t whoami;
    ret = lsm6dsl_device_id_get(&sens_cont, &whoami);
    if (whoami != LSM6DSL_ID)
    {
        return -1;
    }

    return 0;
}

typedef struct lsm6dsl_st_avg_data_tag
{
    uint8_t u8Index;
    int16_t s16AvgBuffer[8];
} LSM6DSL_ST_AVG_DATA;

static void lsm6dsl_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
    uint8_t i;

    *(pAvgBuffer + ((*pIndex) ++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for (i = 0; i < 8; i ++)
    {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

int lsm6dsl_gyro_read(int *psX, int *psY, int *psZ)
{
    if (lsm6dsl_content.data_valid & LSM6DSL_GYRO_DATA_EN)
    {
        // output int16 or float?
        *psX = lsm6dsl_content.gyro_value.i16bit[0];
        *psY = lsm6dsl_content.gyro_value.i16bit[1];
        *psZ = lsm6dsl_content.gyro_value.i16bit[2];

        //*psX = lsm6dsl_from_fs2000dps_to_mdps(lsm6dsl_content.gyro_value.i16bit[0]);
        //*psY = lsm6dsl_from_fs2000dps_to_mdps(lsm6dsl_content.gyro_value.i16bit[1]);
        //*psZ = lsm6dsl_from_fs2000dps_to_mdps(lsm6dsl_content.gyro_value.i16bit[2]);

        return 0;
    }
    return 1;
}

int lsm6dsl_accel_read(int *psX, int *psY, int *psZ)
{
    if (lsm6dsl_content.data_valid & LSM6DSL_ACCE_DATA_EN)
    {
#if 0
        int i;
        int32_t s32OutBuf[3] = {0};
        static LSM6DSL_ST_AVG_DATA sstAvgBuf[3];
        for (i = 0; i < 3; i ++)
        {
            lsm6dsl_CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, lsm6dsl_content.acce_value.i16bit[i], s32OutBuf + i);
        }
        *psX = s32OutBuf[0];
        *psY = s32OutBuf[1];
        *psZ = s32OutBuf[2];
#else
        *psX = lsm6dsl_content.acce_value.i16bit[0];
        *psY = lsm6dsl_content.acce_value.i16bit[1];
        *psZ = lsm6dsl_content.acce_value.i16bit[2];
#endif
        // output int16 or float?
        // need convert to float ?
        //*psX = lsm6dsl_from_fs2g_to_mg( lsm6dsl_content.acce_value.i16bit[0]);
        //*psY = lsm6dsl_from_fs2g_to_mg( lsm6dsl_content.acce_value.i16bit[1]);
        //*psZ = lsm6dsl_from_fs2g_to_mg( lsm6dsl_content.acce_value.i16bit[2]);
        return 0;
    }
    return 1;
}

int lsm6dsl_tempra_read(float *tempra)
{
    if (lsm6dsl_content.data_valid & LSM6DSL_TEMP_DATA_EN)
    {
        *tempra = lsm6dsl_from_lsb_to_celsius(lsm6dsl_content.temp_value.i16bit);

        return 0;
    }
    return 1;
}

int lsm6dsl_step_read(int32_t *step)
{
    if (lsm6dsl_content.data_valid & LSM6DSL_STEP_DATA_EN)
    {
        *step = (int32_t)lsm6dsl_content.pedo_value.i16bit;

        return 0;
    }

    return 1;
}


void lsm6dsl_accel_set_range(uint8_t range)
{
    lsm6dsl_xl_full_scale_set(&sens_cont, range);
}

void lsm6dsl_gyro_set_range(uint8_t range)
{
    lsm6dsl_gy_full_scale_set(&sens_cont, range);
}

#if defined(RT_USING_FINSH)&&!defined(LCPU_MEM_OPTIMIZE)
    #define DRV_6DSL_TEST
#endif
#ifdef DRV_6DSL_TEST

//#include "counter.h"

#define LSM_TEST_FIFO_CNT       (60)
int16_t fifo_buf[LSM_TEST_FIFO_CNT];
int lsm6d_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        if (lsm6dsl_init() == 0)
        {
            lsm6dsl_open();
            LOG_I("lsm6dsl open success\n");
        }
        else
        {
            LOG_I("lsm6dsl open fail\n");
        }
    }
    else if (strcmp(argv[1], "-close") == 0)
    {
        lsm6dsl_close();
    }
    else if (strcmp(argv[1], "-fifo") == 0)
    {
        int en = atoi(argv[2]);
        if (en == 1)
        {
            lsm6dsl_fifo_enable(0, 2);
            lsm6dsl_fifo_enable(1, 2);
        }
        else
        {
            lsm6dsl_fifo_disable(0);
            lsm6dsl_fifo_disable(1);
        }
    }
    else if (strcmp(argv[1], "-fcnt") == 0)
    {
        int i, len;
        int cnt = lsm6dsl_get_fifo_count();
        LOG_I("fifo deep %d\n", cnt);
        len = cnt > LSM_TEST_FIFO_CNT ? LSM_TEST_FIFO_CNT : cnt;
        lsm6dsl_read_fifo((uint8_t *)fifo_buf, len * 2);
        cnt = lsm6dsl_get_fifo_count();
        LOG_I("fifo deep after read %d:  %d\n", LSM_TEST_FIFO_CNT, cnt);
        for (i = 0; i < LSM_TEST_FIFO_CNT; i++)
        {
            LOG_I("%d ", fifo_buf[i]);
            if (i % 3 == 2)
                LOG_I("\n");
        }
    }
    else if (strcmp(argv[1], "-waterm") == 0)
    {
        int cnt, waterm;
        lsm6dsl_set_fifo_threshold(100 * 6); // 100 array for 3 acc and 3 gyro
        cnt = lsm6dsl_get_fifo_count();
        LOG_I("org fifo deep %d\n", cnt);
        lsm6dsl_fifo_enable(0, 2);
        lsm6dsl_fifo_enable(1, 2);
        do
        {
            rt_thread_delay(5);
            waterm = lsm6dsl_get_waterm_status();
        }
        while (waterm == 0);
        cnt = lsm6dsl_get_fifo_count();
        LOG_I("final fifo deep %d\n", cnt);
    }
    else if (strcmp(argv[1], "-pedo") == 0)
    {
        int en = 1;
        if (argc >= 3)
            en = atoi(argv[2]);
        if (en == 0)
        {
            lsm6dsl_pedo_enable(0);
            LOG_I("LSM pedometer disable\n");
        }
        else
        {
            lsm6dsl_pedo_enable(1);
            LOG_I("LSM pedometer enable\n");
        }
    }
    else if (strcmp(argv[1], "-fstep") == 0)
    {
        lsm6dsl_fifo_enable(3, 1);
        LOG_I("Enable pedometer to fifo mode\n");
    }
    else if (strcmp(argv[1], "-fpout") == 0)
    {
        int cnt = lsm6dsl_get_fifo_count();
        uint8_t *buf = malloc(cnt * 2);
        int res = lsm6dsl_read_fifo(buf, cnt * 2);
        int step = lsm6dsl_pedo_fifo2step(buf, cnt * 2);
        LOG_I("Step: %d\n", step);
        free(buf);
    }
    else if (strcmp(argv[1], "-awen") == 0)
    {
        int en = 1;
        if (argc >= 3)
            en = atoi(argv[2]);
        if (en == 0)
        {
            lsm6dsl_awt_enable(0);
            LOG_I("LSM awt disable\n");
        }
        else
        {
            lsm6dsl_awt_enable(1);
            LOG_I("LSM awt enable\n");
        }
    }
    else if (strcmp(argv[1], "-acce") == 0)
    {
        int x, y, z;
        if (lsm6dsl_accel_read(&x, &y, &z) == 0)
        {
            LOG_I("accX = %d, accY = %d, accZ = %d\n", x, y, z);
        }
        else
        {
            LOG_I("get accelerator fail\n");
        }
    }
    else if (strcmp(argv[1], "-gyro") == 0)
    {
        int x, y, z;
        if (lsm6dsl_gyro_read(&x, &y, &z) == 0)
        {
            LOG_I("gyroX = %d, gyroY = %d, gyroZ = %d\n", x, y, z);
        }
        else
        {
            LOG_I("get gyro fail\n");
        }
    }
    else if (strcmp(argv[1], "-temp") == 0)
    {
        float tempr;
        if (lsm6dsl_tempra_read(&tempr) == 0)
        {
            LOG_I("Temperature = %f degC\n", tempr);
        }
        else
        {
            LOG_I("get temperature fail\n");
        }
    }
#if 0
    else if (strcmp(argv[1], "-plib") == 0) // pedometer lib-c
    {
        int res;
        int32_t accx = 0, accy = 0, accz = 0;
        int32_t loop = 50 * 60 * 5;
        int32_t logcnt = 0;
        SportDataType data = {0};
        if (argc >= 3)
            loop = 50 * 60 * atoi(argv[2]);

        Sport_Init();
        Set_Parameter(175, 80);

        do
        {
            res = lsm6dsl_accel_read(&accx, &accy, &accz);
            if (res != 0)
                LOG_I("get accel data fail\n");
            Sport_Calculator(accx, accy, accz); // suppose parameter mg based
            logcnt++;
            if (logcnt >= 50 * 2)
            {
                logcnt = 0;
                Read_SportData(&data);
                LOG_I("Step count %d\n", data.steps);
                LOG_I("Input 0x%x, 0x%x, 0x%x\n", accx, accy, accz);
            }

            loop--;
            rt_thread_delay(20);    // continue check with 50hz
        }
        while (loop > 0);
    }
#endif
    else if (strcmp(argv[1], "-awt") == 0)
    {
#ifdef LSM_USING_AWT
        int32_t ret;
        lsm6dsl_func_src2_t          func_src2;
        lsm6dsl_wrist_tilt_ia_t      wrist_tilt_ia;
        lsm6dsl_a_wrist_tilt_mask_t  a_wrist_tilt_mask;
        uint8_t value;
        ret = lsm6dsl_read_reg(&sens_cont, LSM6DSL_FUNC_SRC2,
                               (uint8_t *) & (func_src2), 1);
        LOG_I("func_src:           %d: 0x%x\n", ret, func_src2);

        ret = lsm6dsl_read_reg(&sens_cont, LSM6DSL_WRIST_TILT_IA,
                               (uint8_t *) & (wrist_tilt_ia), 1);
        LOG_I("wrist_tilt_id: %d, %d, %d, %d, %d, %d\n",
              wrist_tilt_ia.wrist_tilt_ia_xneg, wrist_tilt_ia.wrist_tilt_ia_xpos,
              wrist_tilt_ia.wrist_tilt_ia_yneg, wrist_tilt_ia.wrist_tilt_ia_ypos,
              wrist_tilt_ia.wrist_tilt_ia_zneg, wrist_tilt_ia.wrist_tilt_ia_zpos);

        ret = lsm6dsl_wrist_tilt_sens_get(&sens_cont, &value);
        LOG_I("wrist_tilt_sens:    %d:  0x%x\n", ret, value);

        ret = lsm6dsl_tilt_latency_get(&sens_cont, &value);
        LOG_I("tilt_latency:       %d:  0x%x\n", ret, value);

        ret = lsm6dsl_tilt_threshold_get(&sens_cont, &value);
        LOG_I("tilt_threshold:     %d:  0x%x\n", ret, value);

        lsm6dsl_tilt_src_get(&sens_cont, &a_wrist_tilt_mask);
        LOG_I("tilt_src: %d, %d, %d, %d, %d, %d\n", a_wrist_tilt_mask.wrist_tilt_mask_xneg,
              a_wrist_tilt_mask.wrist_tilt_mask_xpos, a_wrist_tilt_mask.wrist_tilt_mask_yneg,
              a_wrist_tilt_mask.wrist_tilt_mask_ypos, a_wrist_tilt_mask.wrist_tilt_mask_zneg,
              a_wrist_tilt_mask.wrist_tilt_mask_zpos);
#endif /*  LSM_USING_AWT */
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(lsm6d_test, __cmd_lsm, Test hw lsm);
#endif //DRV_6DSL_TEST

#endif /*ACC_USING_LSM6DSL*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
