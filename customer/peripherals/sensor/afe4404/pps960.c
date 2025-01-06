/**
  ******************************************************************************
  * @file   pps960.c
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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include "board.h"
#include "afe4404_hw.h"
#include "agc_V3_1_19.h"
#include "hqerror.h"
#include "pps960.h"
#include "i2c.h"

#include "Mac_Hr.h"


#define DRV_DEBUG
#define LOG_TAG              "drv.afe"
#include <drv_log.h>


// move it to menuconfig if needed
//#define AFE_I2C_BUS         "i2c3"
#define PPS960_ADDR (0x58)
//#define PPS960_ADDR (0x6b)

#define PPS_HR_PERIOD_MS        (40)
#define PPS_USE_TIMER

typedef struct
{
    struct rt_i2c_bus_device *afe_i2cbus;
    rt_device_t afe_pin_dev;
    uint8_t start_flag;
} pps_context_t;

static pps_context_t pps_ctx;

//uint16_t acc_check = 0;
//static struct rt_i2c_bus_device *afe_i2cbus = NULL;
//static rt_device_t afe_pin_dev = NULL;

#ifdef PPS_USE_TIMER
    rt_timer_t pps_timer = NULL;
    static void pps_timeout_entry(void *parameter);

#else
    void pps960_sensor_task(void *params);
    rt_thread_t afe_thread = NULL;
    static void AFE4404_AFE_ADCReady(void *args);
    static struct rt_semaphore data_rdy_sem;
#endif

void PPS_DELAY_MS(uint32_t ms)
{
    rt_thread_delay(ms);
}

void PPS960_sample_open(void)
{
    // Initialize HRM variables
    //hr_hw_init();
#ifdef PPS_USE_TIMER
    pps_timer = rt_timer_create("timer_pps",
                                pps_timeout_entry,
                                RT_NULL,
                                rt_tick_from_millisecond(PPS_HR_PERIOD_MS),
                                RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    if (pps_timer != NULL)
        rt_timer_start(pps_timer);
    else
        LOG_E("Create timer fail\n");

#else
    rt_sem_init(&data_rdy_sem, "pps_drdy", 0, RT_IPC_FLAG_FIFO);

    afe_thread = rt_thread_create("afe", pps960_sensor_task, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (afe_thread != NULL)
    {
        rt_thread_startup(afe_thread);
        LOG_D("hr thread started\n");
    }
    else
        LOG_E("Create heart rate thread fail\n");


    struct rt_device_pin_mode m;
    m.pin = HRM_RDY_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    // enable data ready int
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_FALLING, AFE4404_AFE_ADCReady, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);
#endif
}

void PPS960_sample_close()
{
#ifdef PPS_USE_TIMER
    if (pps_timer)
        rt_timer_stop(pps_timer);
    pps_timer = NULL;
#else
    if (afe_thread != NULL)
        rt_thread_delete(afe_thread);

    afe_thread = NULL;

    rt_pin_irq_enable(HRM_RDY_GPIO_BIT + 1, 0);
    rt_pin_detach_irq(HRM_RDY_GPIO_BIT + 1);

    rt_sem_release(&data_rdy_sem);
#endif
}

int AFE4404_GPIO_Init()
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at AFE\n");
        return -1;
    }
    pps_ctx.afe_pin_dev = device;
    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    // ready int pin cfg
    m.pin = HRM_RDY_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable data ready int
    //rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_FALLING, AFE4404_AFE_ADCReady, (void *)(rt_uint32_t)m.pin);
    //rt_pin_irq_enable(m.pin, 1);

    // reset pin cfg
    m.pin = HRM_RST_GPIO_BIT;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    // reset
    st.pin = HRM_RST_GPIO_BIT;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    st.pin = HRM_RST_GPIO_BIT;
    st.status = 1;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));

    return 0;
}

int AFE4404_I2C_Init()
{
    /* get i2c bus device */
    pps_ctx.afe_i2cbus = rt_i2c_bus_device_find(AFE_I2C_BUS);
    if (pps_ctx.afe_i2cbus)
    {
        LOG_D("Find i2c bus device %s\n", AFE_I2C_BUS);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", AFE_I2C_BUS);
        return -1;
    }

    return 0;
}


int pps960_Init_gpio(void)
{
    //PPS960 PIN CONFIG
    //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    //w_io_io(hrrdy, VFS_IO_IT_RISING, VFS_PULLDOWN);
    //w_io_io(hrrst, VFS_IO_OUT, VFS_PULLUP);
    //w_io_io(hrpower, VFS_IO_OUT, VFS_PULLUP);
    //w_io_write(hrrst, 0);
    //w_io_write(hrpower, 0);
    return AFE4404_GPIO_Init();
}

void pps960_Rest_HW(void)
{
    struct rt_device_pin_status st;

    if (pps_ctx.afe_pin_dev == NULL)
        return;
    //RESTZ Pin
    //w_io_write(hrrst, 0);//low
    // reset
    st.pin = HRM_RST_GPIO_BIT;
    st.status = 0;
    rt_device_write(pps_ctx.afe_pin_dev, 0, &st, sizeof(struct rt_device_pin_status));

    PPS_DELAY_MS(10);
    //w_io_write(hrrst, 1); //high
    st.pin = HRM_RST_GPIO_BIT;
    st.status = 1;
    rt_device_write(pps_ctx.afe_pin_dev, 0, &st, sizeof(struct rt_device_pin_status));
    PPS_DELAY_MS(10);
}
void pps960_Rest_SW(void)
{
    //software rest
    //PPS960_readReg(0x23);
    PPS960_writeReg(0, 0x8);
    PPS_DELAY_MS(50);
}

void pps960_Disable_HW(void)
{
    pps_ctx.start_flag = 0;

    //RESTZ Pin low
    //w_io_write(hrrst, 0); //low
    //w_io_write(hrpower, 0); //low
    struct rt_device_pin_status st;

    if (pps_ctx.afe_pin_dev == NULL)
        return;
    st.pin = HRM_RST_GPIO_BIT;
    st.status = 0;
    rt_device_write(pps_ctx.afe_pin_dev, 0, &st, sizeof(struct rt_device_pin_status));
}

void pps960_Enable_HW(void)
{
    //RX_SUP Control PIN enable
    //nrf_gpio_pin_write(PPS_RX_SUP_PIN, 1);//high
    //TX_SUP Control PIN enable
    //nrf_gpio_pin_write(PPS_TX_SUP_PIN, 1);//high
    //w_io_write(hrpower, 1);
    //PPS_DELAY_MS(10);
    //w_io_write(hrrst, 1);
    struct rt_device_pin_status st;

    if (pps_ctx.afe_pin_dev == NULL)
        return;

    st.pin = HRM_RST_GPIO_BIT;
    st.status = 1;
    rt_device_write(pps_ctx.afe_pin_dev, 0, &st, sizeof(struct rt_device_pin_status));
    PPS_DELAY_MS(10);

}

int init_pps960_sensor(void)
{
    int res = 0;
    res = pps960_Init_gpio();
    if (res != 0)
        return res;

    res = AFE4404_I2C_Init();
    if (res != 0)
        return res;

    //pps960_Enable_HW();

    //int res,i;
    //for(i=0; i<30; i++)
    //{
    //    res = Mac_HR_Validcheck();
    //     LOG_I("Mac_HR_Validcheck = %d\n",res);
    // }

    //PPS960_sample_open();
    return 0;
}

int open_pps960(void)
{
    if (pps_ctx.start_flag == 1) // opened before
        return 0;

    pps960_Rest_HW();
    pps960_Rest_SW();

    init_PPS960_register();
    Mac_Init();
    PPS960_init();

    if (PPS960_readReg(0X01) != 0X000014)
        return -1;//

    pps_ctx.start_flag = 1;
    PPS960_sample_open();
    LOG_D("HR pps960 opened\n");
    return 0;
}

int close_pps960(void)
{
    if (pps_ctx.start_flag == 0) // closed before
        return 0;

    pps_ctx.start_flag = 0;
    //valid_flag = -1;
    PPS960_sample_close();
    pps960_Disable_HW();
    LOG_D("HR pps960 closed\n");

    return 0;
}

static uint16_t hr_buf[8];
static int valid_flag = -1;

void pps960_hr_filter(uint16_t value)
{
    if ((valid_flag == -1) && ((value == 0) || (value == 50))) // hr lib alway return 50 at beginning
        return;

    valid_flag++;
    hr_buf[valid_flag & 0x7] = value;
    //if (valid_flag < 50)
    //    LOG_I("%d  %d\n", valid_flag, value);
}

#ifdef PPS_USE_TIMER
static void pps_timeout_entry(void *parameter)
{
    uint16_t pps_count;
    //LOG_I("current tick: %ld\n", rt_tick_get());
    if (pps_ctx.start_flag)
    {
        ALGSH_retrieveSamplesAndPushToQueue();//read pps raw data
        //move ALGSH_dataToAlg(); to message queue loop. and then send message at here.
        ALGSH_dataToAlg();
        if (PPS_get_skin_detect())
        {
            pps_count = pps_getHR();
            pps960_hr_filter(pps_count);
        }
        else
        {
            valid_flag = -1;
        }
    }
}
#else

// check falling edge
static void AFE4404_AFE_ADCReady(void *args)
{
    int value = (int)args;
    //LOG_I("AFE4404_AFE_ADCReady %d\n", value);
    rt_sem_release(&data_rdy_sem);
}

void pps960_sensor_task(void *params)  //40ms
{
    uint16_t pps_count;
    // add by whj for thread loop
    while (1)
    {
        // use data ready interrupt instead timer
        rt_sem_take(&data_rdy_sem, RT_WAITING_FOREVER) ;
        //rt_thread_delay(PPS_HR_PERIOD_MS);    // no irq, use delay instead
        if (pps_ctx.start_flag)
        {
            ALGSH_retrieveSamplesAndPushToQueue();//read pps raw data
            //move ALGSH_dataToAlg(); to message queue loop. and then send message at here.
            ALGSH_dataToAlg();
            if (PPS_get_skin_detect())
            {
                pps_count = pps_getHR();
                pps960_hr_filter(pps_count);
            }
        }

        // add to reset to avoid timer delta increase?
        //if()
        //Mac_Init();
    }
}
#endif // PPS_USE_TIMER

uint32_t pps960_get_hr(void)
{
    int32_t total, i;

    //return (uint32_t)pps_getHR();
    if (valid_flag < 7)
        return 0;

    total = 0;
    for (i = 0; i < 8; i++)
        total += (int32_t)hr_buf[i];

    //LOG_D("output hr %d\n", total >> 3);

    return (total >> 3);
}

uint32_t pps960_get_i2c_handle(void)
{
    return (uint32_t)pps_ctx.afe_i2cbus;
}

uint8_t pps960_get_dev_addr(void)
{
    return (uint8_t)PPS960_ADDR;
}

int pps960_self_check(void)
{
    struct rt_device_pin_status st;

    // check register first, register 1 should init to 0x14
    if (PPS960_readReg(0X01) != 0X000014)
        return 1;//

    // check gpio
    if (pps_ctx.afe_pin_dev == NULL)
        return 2;
    st.pin = HRM_RST_GPIO_BIT;
    rt_device_read(pps_ctx.afe_pin_dev, 0, &st, sizeof(struct rt_device_pin_status));
    if (st.status == 0)
        return 3;

    return 0;
}


void PPS960_writeReg(uint8_t regaddr, uint32_t wdata)
{
    uint8_t temp[4];
    struct rt_i2c_msg msgs;
    int ret;

    uint32_t wd = wdata;
    temp[0] = regaddr;
    temp[1] = (wd >> 16) & 0xff;
    temp[2] = (wd >> 8) & 0xff;
    temp[3] = wd & 0xff;
    //int ret = HAL_I2C_Master_Transmit(&hi2c1, PPS960_ADDR<<1, temp, 4, 100);
    msgs.addr = PPS960_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = temp;
    msgs.len = 4;
    ret = rt_i2c_transfer(pps_ctx.afe_i2cbus, &msgs, 1);
    if (ret > 0)
    {
        //LOG_D("PPS960_writeReg ret = %d\n", ret);
    }
    else
    {
        LOG_D("PPS960_writeReg fail\n");
    }
}
uint32_t PPS960_readReg(uint8_t regaddr)
{
    //int ret;
    uint8_t temp[4] = { 0 };
    uint32_t rdtemp;
    uint8_t addr8 = (uint8_t)regaddr;
    /*if (HAL_I2C_Master_Transmit(&hi2c1, PPS960_ADDR << 1, &regaddr, 1, 100) == HAL_OK)
    {
        if (ret = HAL_I2C_Master_Receive(&hi2c1, (PPS960_ADDR << 1) + 1, temp, 3, 100) == HAL_OK)
        {
            rdtemp = temp[0] << 16 | temp[1] << 8 | temp[2];
            return rdtemp;
        }
    }*/
    if (rt_i2c_mem_read(pps_ctx.afe_i2cbus, PPS960_ADDR, regaddr, 8, temp, 3) > 0)
        //if (HAL_I2C_Mem_Read(&hi2c1, PPS960_ADDR << 1, regaddr, I2C_MEMADD_SIZE_8BIT, temp, 3, 100) == HAL_OK)
    {
        rdtemp = temp[0] << 16 | temp[1] << 8 | temp[2];
        return rdtemp;
    }
    else
    {
        LOG_D("PPS960_readReg fail\n");
    }
    return 0XFFFFFFFF;
}


#define DRV_HR_TEST
#ifdef DRV_HR_TEST

rt_thread_t pps_test_thread = NULL;

void pps960_test_task2(void *params)
{
    int8_t skin;
    uint32_t value;

    while (1)
    {
        skin = PPS_get_skin_detect();
        if (skin > 0)
        {
            value = pps960_get_hr();
            if (value > 0)
                LOG_I("skin %d, hr %d\n", skin, value);
        }
        rt_thread_delay(2000);  // output each 2 second
    }
}

void PPS960_output_start(void)
{
    pps_test_thread = rt_thread_create("ppsout", pps960_test_task2, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (pps_test_thread != NULL)
    {
        rt_thread_startup(pps_test_thread);
        LOG_I("hr output thread started\n");
    }
    else
        LOG_I("Create heart rate output thread fail\n");

}


int pps960_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-init") == 0)
    {
        if (init_pps960_sensor() >= 0)
            LOG_I("Init pps960 success\n");
        else
            LOG_I("Init pps960 fail\n");
    }
    else if (strcmp(argv[1], "-open") == 0)
    {
        int res;
        res = open_pps960();
        if (res == 0)
            LOG_I("HR open success\n");
        else
            LOG_I("HR open fail\n");
    }
    else if (strcmp(argv[1], "-close") == 0)
    {
        close_pps960();
    }
    else if (strcmp(argv[1], "-start") == 0)
    {
        PPS960_output_start();
    }
    else if (strcmp(argv[1], "-stop") == 0)
    {
        PPS960_sample_close();
    }
    else if (strcmp(argv[1], "-o") == 0)
    {
        //pps960_test_task2(NULL);
    }
    else if (strcmp(argv[1], "-hr") == 0)
    {
        uint32_t hr = pps960_get_hr();
        LOG_I("heart rate %d\n", hr);
    }
    else if (strcmp(argv[1], "-r") == 0)
    {
        int reg = atoi(argv[2]);
        int value = PPS960_readReg(reg);
        LOG_I("reg %d value %d\n", reg, value);
    }
    else if (strcmp(argv[1], "-w") == 0)
    {
        int reg = atoi(argv[2]);
        int value = atoi(argv[3]);
        PPS960_writeReg(reg, value);
        LOG_I("reg %d value %d\n", reg, value);
    }
    else if (strcmp(argv[1], "-reset") == 0)
    {
        pps960_Rest_HW();
        LOG_I("heart rate reset\n");
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(pps960_test, __cmd_hr, Test hw hr);

#endif  //DRV_HR_TEST

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
