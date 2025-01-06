/**
  ******************************************************************************
  * @file   sc7a22.c
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

#include "sc7a22.h"
#include "board.h"
#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif

#ifdef ACC_USING_SC7A22

#define DRV_DEBUG
#define LOG_TAG              "drv.slw"
#include <drv_log.h>


// move it to menuconfig if needed
//#define SC7A22_I2C_BUS         "i2c3"
#define SC7A22_DEV_NAME        "slw_dev"

#define SC7A22_I2C_ADDR_H (0x19)
#define SC7A22_I2C_ADDR_L (0x18)

#define SC7A22_I2C_ADDR    SC7A22_I2C_ADDR_H

//#define SC7A22_USE_INT
#define SC7A22_UES_INT1  0
#define SC7A22_UES_INT2  1


struct SC7A22_CONT_T
{
#if (SC7A22_USING_I2C == 1)
    struct rt_i2c_bus_device *bus_handle;
#else   // SPI
    struct rt_spi_device *bus_handle;
#endif  // SC7A22_USING_I2C
    uint8_t open_flag;
    uint8_t dev_addr;
    uint8_t whoami;
};

static sifdev_sensor_ctx_t sens_cont;
static struct SC7A22_CONT_T sc7a22_content;
#ifdef SC7A22_USE_INT
    static struct rt_semaphore slw_int_sem;
    rt_thread_t sc7a22_thread = NULL;
#endif
extern rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name);

static int sc7a22_power_on()
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_err_t ret = RT_EOK;
#if (SC7A22_POW_PIN >= 0)
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        rt_kprintf("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;

    m.pin = SC7A22_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = SC7A22_POW_PIN;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    rt_kprintf("sc7a22_power_on\r\n");
#endif
    return ret;
}

static int sc7a22_power_off()
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    rt_err_t ret = RT_EOK;
#if (SC7A22_POW_PIN >= 0)

    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        rt_kprintf("GPIO pin device not found at motor ctrl\n");
        return RT_EIO;
    }

    ret = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) return ret;

    m.pin = SC7A22_POW_PIN;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = SC7A22_POW_PIN;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    ret = rt_device_close(device);
    rt_kprintf("sc7a22_power_off\r\n");
#endif
    return ret;
}

void sc7a22_power_onoff(bool onoff)
{
    if (onoff)
    {
#ifdef PMIC_CONTROL_SERVICE
        pmic_service_control(PMIC_CONTROL_GSENSOR, 1);
#else
        sc7a22_power_on();
#endif
    }
    else
    {
#ifdef PMIC_CONTROL_SERVICE
        pmic_service_control(PMIC_CONTROL_GSENSOR, 0);
#else
        sc7a22_power_off();
#endif
    }
}

//#define USER_CTL_CS
static int sc7a22_i2c_init()
{
#if (SC7A22_USING_I2C == 1)

    /* get i2c bus device */
    sc7a22_content.bus_handle = rt_i2c_bus_device_find(SC7A22_BUS_NAME);
    if (sc7a22_content.bus_handle)
    {
        rt_kprintf("Find i2c bus device %s\n", SC7A22_BUS_NAME);
    }
    else
    {
        rt_kprintf("Can not found i2c bus %s, init fail\n", SC7A22_BUS_NAME);
        return -1;
    }
#else   // SPI
    /* get i2c bus device */
    rt_err_t ret;
    rt_device_t spi_bus = rt_device_find(SC7A22_BUS_NAME);
    //sc7a22_content.bus_handle = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));

    if (spi_bus)
    {
        struct rt_spi_configuration cfg1;
        rt_kprintf("Find spi bus %s\n", SC7A22_BUS_NAME);
        sc7a22_content.bus_handle = (struct rt_spi_device *)rt_device_find(SC7A22_DEV_NAME);
        if (sc7a22_content.bus_handle == NULL)
        {
            ret = rt_hw_spi_device_attach(SC7A22_BUS_NAME, SC7A22_DEV_NAME);
            sc7a22_content.bus_handle = (struct rt_spi_device *)rt_device_find(SC7A22_DEV_NAME);
            if (sc7a22_content.bus_handle == NULL)
            {
                rt_kprintf("Register SC7A22 spi device fail\n");
                return -1;
            }
        }
        ret = rt_device_open((rt_device_t)(sc7a22_content.bus_handle), RT_DEVICE_FLAG_RDWR);

#ifndef USER_CTL_CS
        cfg1.data_width = 16; // auto cs, need total 16 bits to read data in low 8 bits
#else
        cfg1.data_width = 8;
#endif
        cfg1.max_hz = 8 * 1000 * 1000; // 8m
        cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_MASTER ;
        cfg1.frameMode = RT_SPI_MOTO; //RT_SPI_TI;

        ret = rt_spi_configure(sc7a22_content.bus_handle, &cfg1);

        ret = rt_spi_take_bus(sc7a22_content.bus_handle);


    }
    else
    {
        rt_kprintf("Can not found spi bus %s, init fail\n", SC7A22_BUS_NAME);
        return -1;
    }

#endif  // SC7A22_USING_I2C
    return 0;
}

static int sc7a22_i2c_deinit(void)
{
    int ret;
    if (sc7a22_content.bus_handle != NULL)
        ret = rt_device_close((rt_device_t)(sc7a22_content.bus_handle));

    return ret;
}

static int32_t slw_i2c_write(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res;

    struct SC7A22_CONT_T *handle = (struct SC7A22_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
#if (SC7A22_USING_I2C == 1)
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

        //rt_kprintf("Write addr 0x%x , len %d\n", reg, len);
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

static int32_t slw_i2c_read(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res;
    struct SC7A22_CONT_T *handle = (struct SC7A22_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
#if (SC7A22_USING_I2C == 1)
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
            //rt_kprintf("SPI read data 0x%x\n", *data);
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

static int32_t slw_spi_multi_bytes_read(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_err_t ret = RT_ERROR;
    struct SC7A22_CONT_T *handle = (struct SC7A22_CONT_T *)ctx;

    uint8_t buf;
    buf = (reg | 0xC0);

    if (handle && handle->bus_handle && data)
    {
        //rt_kprintf("slw_spi_multi_bytes_read Read addr 0x%x , len %d\n", buf, len);
        ret = rt_spi_send_then_recv(handle->bus_handle, &buf, 1, data, len);

        //rt_kprintf("data[x] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);

    }

    return ret;
}

#ifdef SC7A22_USE_INT

/* SC7A22 interrupt init, it use gpio input as int */
// check edge
static void sc7a22_int1_handle(void *args)
{
    rt_kprintf("sc7a22_int1_handle\n");
    rt_sem_release(&slw_int_sem);
}

static void sc7a22_int2_handle(void *args)
{
    rt_kprintf("sc7a22_int2_handle\n");
    rt_sem_release(&slw_int_sem);
}

static int sc7a22_gpio_int_enable(void)
{
    struct rt_device_pin_mode m;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        rt_kprintf("GPIO pin device not found at sc7a22\n");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
#if SC7A22_UES_INT1
    // int pin cfg
    m.pin = SC7A22_INT_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable int
    rt_pin_mode(SC7A22_INT_GPIO_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, sc7a22_int1_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif
    // for int2
#if SC7A22_UES_INT2
    // int pin cfg
    m.pin = SC7A22_INT2_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable int
    rt_pin_mode(SC7A22_INT2_GPIO_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, sc7a22_int2_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif

    return 0;

}

static int sc7a22_gpio_int_disable(void)
{
    struct rt_device_pin_mode m;

#if SC7A22_UES_INT1
    // int pin cfg
    m.pin = SC7A22_INT_GPIO_BIT;

    // enable int
    //rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, sc7a22_int_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 0);
#endif
    // for int2
#if SC7A22_UES_INT2
    // int pin cfg
    m.pin = SC7A22_INT2_GPIO_BIT;
    rt_pin_irq_enable(m.pin, 0);

#endif

    return 0;

}


void sc7a22_sensor_task(void *params)  //20ms
{
    int32_t ret;

    // add by whj for thread loop
    rt_kprintf("sc7a22_sensor_task1\n");
    while (1)
    {
        //rt_thread_delay(20);    // no irq, use delay instead
        rt_sem_take(&slw_int_sem, RT_WAITING_FOREVER) ;
        //rt_kprintf("sc7a22_sensor_task2\r\n");

    }
}
#endif // SC7A22_USE_INT
int sc7a22_reg_init(void)
{
    int ret;
    ret = sc7a22_reset_set(&sens_cont, SC7A22_RESET_VAL);
    if (ret != 0) goto exit;
    //LOG_I("Reset res = %d, rst = %d\n",res, rst);

    ret = sc7a22_i2c_enable_set(&sens_cont, SC7A22_I2C_ENABLE_VAL);
    if (ret != 0) goto exit;

    ret = sc7a22_xl_filter_set(&sens_cont, SC7A22_CTRL0_VAL);
    if (ret != 0) goto exit;
    /*
     * Set full scale
     */
    ret = sc7a22_xl_full_scale_set(&sens_cont, SC7A22_CTRL4_VAL);
    if (ret != 0) goto exit;

    /*
     * Set Output Data Rate
     */
    ret = sc7a22_xl_data_rate_set(&sens_cont, SC7A22_CTRL1_VAL);
    if (ret != 0) goto exit;

    //fifo set
    ret = sc7a22_fifo_mode_set(&sens_cont, SC7A22_FIFO_MODE_VAL);
exit:
    return ret;
}

int sc7a22_reg_deinit(void)
{
    return 0;
}

int sc7a20_reg_init(void)
{
    int ret;
    //fifo bypass
    ret = sc7a22_fifo_mode_set(&sens_cont, SC7A22_FIFO_BYPASS_VAL);
    if (ret != 0) goto exit;

    //odr
    ret = sc7a22_xl_data_rate_set(&sens_cont, SC7A22_CTRL1_VAL);
    if (ret != 0) goto exit;

    //16g, spi 4 line, hold old data
    ret = sc7a22_xl_full_scale_set(&sens_cont, SC7A22_CTRL4_VAL);
    if (ret != 0) goto exit;

    //fifo enbale
    ret = sc7a22_fifo_enable_set(&sens_cont, SC7A22_CTRL5_VAL);
    if (ret != 0) goto exit;

    //fifo mode
    ret = sc7a22_fifo_mode_set(&sens_cont, SC7A22_FIFO_MODE_VAL);

#if 0 // for test
    rt_thread_mdelay(5);
    uint8_t buff1;
    sc7a22_read_reg(&sens_cont, SC7A22_CTRL1_REG, &buff1, 1);
    rt_kprintf("sc7a20_reg_init reg20 0x%x\n", buff1);

    rt_thread_mdelay(5);
    uint8_t buff2;
    sc7a22_read_reg(&sens_cont, SC7A22_CTRL4_REG, &buff2, 1);
    rt_kprintf("sc7a20_reg_init reg23 0x%x\n", buff2);

    rt_thread_mdelay(5);
    uint8_t buff3;
    sc7a22_read_reg(&sens_cont, SC7A22_FIFO_CTRL_REG, &buff3, 1);
    rt_kprintf("sc7a20_reg_init reg2E 0x%x\n", buff3);
#endif
exit:
    return ret;
}

int sc7a20_reg_deinit(void)
{
    int ret;
    ret = sc7a22_xl_data_rate_set(&sens_cont, SC7A22_CLOSE_VAL);
    if (ret != 0) goto exit;

exit:
    return ret;
}

int sc7a22_init(void)
{
    int res;

    sc7a22_content.dev_addr = SC7A22_I2C_ADDR_L;
    sc7a22_content.bus_handle = NULL;
    sc7a22_content.whoami = 0;
    sc7a22_content.open_flag = 0;

    sc7a22_power_onoff(1);
    rt_thread_mdelay(50);

    res = sc7a22_i2c_init();
    if (res != 0)
    {
        rt_kprintf("sc7a22_i2c_init fail\n");
        sc7a22_i2c_deinit();
        return res;
    }

    sens_cont.read_reg = slw_i2c_read;
    sens_cont.write_reg = slw_i2c_write;
    sens_cont.handle = (void *)&sc7a22_content;

    // check who am I and try i2c slave address
    res = sc7a22_device_id_get(&sens_cont, &(sc7a22_content.whoami));
    rt_kprintf("res 1 = %d when address = %d\n", res, sc7a22_content.whoami);
    if (sc7a22_content.whoami != SC7A22_ID)
    {
        // try another address
        sc7a22_content.dev_addr = SC7A22_I2C_ADDR_H;
        res = sc7a22_device_id_get(&sens_cont, &(sc7a22_content.whoami));
        //rt_kprintf("res 2 = %d when address = %d\n",res,sc7a22_content.whoami);
        if (sc7a22_content.whoami != SC7A22_ID)
        {
            // 2 address all fail
            rt_kprintf("sc7a22 i2c slave init fail\n");
            sc7a22_i2c_deinit();
            return 1;
        }
    }

    rt_kprintf("sc7a22 init done, i2c address = 0x%x, whoAmI 0x%x\n", sc7a22_content.dev_addr, sc7a22_content.whoami);

    return 0;
}

int sc7a22_open(void)
{
    int ret = 0;

    if (sc7a22_content.open_flag == 1) // opened before
        return 0;

#if SC7A22_IC
    sc7a22_reg_init();
    rt_kprintf("sc7a22_reg_init ret %d\n", ret);
#endif

#if SC7A20_IC
    ret = sc7a20_reg_init();
    rt_kprintf("sc7a20_reg_init ret %d\n", ret);
#endif

#ifdef SC7A22_USE_INT
    // start a thread to check data available
    rt_sem_init(&slw_int_sem, "slw_int", 0, RT_IPC_FLAG_FIFO);
    sc7a22_thread = rt_thread_create("sc7a22", sc7a22_sensor_task, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (sc7a22_thread != NULL)
    {
        rt_thread_startup(sc7a22_thread);
        rt_kprintf("6d thread started\n");
    }
    else
        rt_kprintf("Create 6d thread fail\n");

    sc7a22_gpio_int_enable();

    //if (sc7a22_content.int_en == 1) // interrupt enable, so gyro and acce data ready should be used.
    {
#if SC7A22_UES_INT1
        // set gsensor side int1
#endif

#if SC7A22_UES_INT2
        // set gsensor side int2
#endif
    }
#endif
    sc7a22_content.open_flag = 1;

    return ret;
}

int sc7a22_close(void)
{
    if (sc7a22_content.open_flag == 0) // closed before
        return 0;

#ifdef SC7A22_USE_INT
    sc7a22_gpio_int_disable();
    rt_sem_release(&slw_int_sem);

    if (sc7a22_thread != NULL)
    {
        rt_thread_delete(sc7a22_thread);
        sc7a22_thread = NULL;
    }
#endif
#if SC7A20_IC
    sc7a20_reg_deinit();
#endif
#if SC7A22_IC
    sc7a22_reg_deinit();
#endif
    sc7a22_power_onoff(0);
    rt_device_close((rt_device_t)(sc7a22_content.bus_handle));
    sc7a22_content.open_flag = 0;

    return 0;
}

int sc7a22_set_fifo_mode(uint8_t val)
{
    int ret;
    sc7a22_fifo_mode_set(&sens_cont, val);
    return ret;
}

uint8_t sc7a22_get_fifo_count(void)
{
    uint8_t value = 0;
    sc7a22_fifo_count_get(&sens_cont, &value);
#if SC7A20_IC
    value = value & 0x1F; // sc7a20 is 1f, sc7a22 is 7f
#endif

#if SC7A22_IC
    value = value & 0x7F;
#endif

    return value;
}

int sc7a22_read_fifo(uint8_t *buf, int len)
{
    int i;
    if (buf == NULL || len == 0)
        return 0;

    // if sc7a20_read_fifo can not be used to read sc7a22, this fun need to tunning.
    //sc7a22_fifo_raw_data_get(&sens_cont, buf + i * 2, 2);


    return len;
}

int sc7a20_read_fifo(uint8_t *buf, int len)
{
    rt_err_t ret;
    if (buf == NULL || len == 0)
        return 0;

    // spi multi bytes read, and read 1 byte every time, set data_width 8bits.
    struct rt_spi_configuration cfg1;
    cfg1.data_width = 8;
    cfg1.max_hz = 8 * 1000 * 1000; // 8m
    cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_MASTER ;
    cfg1.frameMode = RT_SPI_MOTO; //RT_SPI_TI;

    ret = rt_spi_configure(sc7a22_content.bus_handle, &cfg1);
    //rt_kprintf("sc7a20_read_fifo ret1 %d\n",ret);

    sens_cont.read_reg = slw_spi_multi_bytes_read;
    sc7a22_fifo_raw_data_get(&sens_cont, buf, len);


    //resume spi data width 16bits
    cfg1.data_width = 16;
    cfg1.max_hz = 8 * 1000 * 1000; // 8m
    cfg1.mode = RT_SPI_MODE_3 | RT_SPI_MSB | RT_SPI_MASTER ;
    cfg1.frameMode = RT_SPI_MOTO; //RT_SPI_TI;

    ret = rt_spi_configure(sc7a22_content.bus_handle, &cfg1);
    //rt_kprintf("sc7a20_read_fifo ret2 %d\n",ret);


    sens_cont.read_reg = slw_i2c_read;

    return len;
}

int sc7a22_set_fifo_threshold(int thd)
{

    return 0;
}

int sc7a22_get_waterm_status(void)
{
    uint8_t value;

    return (int)value;
}

int sc7a22_get_overrun_status(void)
{
    uint8_t value;

    return (int)value;
}

int sc7a22_get_fifo_full_status(void)
{
    uint8_t value;

    return (int)value;
}

int sc7a22_get_fifo_empty_status(void)
{
    uint8_t value;

    return (int)value;
}




// function for handle and address --------------------
uint32_t sc7a22_get_bus_handle(void)
{
    return (uint32_t)(sc7a22_content.bus_handle);
}
uint8_t sc7a22_get_dev_addr(void)
{
    return sc7a22_content.dev_addr;
}

uint8_t sc7a22_get_dev_id(void)
{
    return sc7a22_content.whoami;
}

int sc7a22_self_check(void)
{
    int ret = 0;
    uint8_t whoami;
    ret = sc7a22_device_id_get(&sens_cont, &whoami);
    if (whoami != SC7A22_ID)
    {
        return -1;
    }

    return 0;
}

void sc7a22_accel_set_range(uint8_t range)
{
    sc7a22_xl_full_scale_set(&sens_cont, range);
}



#endif /*ACC_USING_SC7A22*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
