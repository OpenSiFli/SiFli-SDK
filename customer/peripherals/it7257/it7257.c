/**
  ******************************************************************************
  * @file   it7257.c
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

#include <rtthread.h>
#include "board.h"
#include "it7257.h"
#include "touch.h"
//#include "EventRecorder.h"

/* Define -------------------------------------------------------------------*/

#define DBG_LEVEL          DBG_ERROR  // DBG_LOG //
#define LOG_TAG              "drv.it7257"
#include <drv_log.h>

#define IT_DEV_ADDR                    (0x46)
#define IT_REG_BUF_QUERY               (0x80)
#define IT_REG_POINT_INFO              (0xE0)


#define IT_MAX_WIDTH                   (240)
#define IT_MAX_HEIGHT                  (240)
#define IT_FINGER_NUM                  (3)

/* result of reading with IT_REG_BUF_QUERY bits */
#define IT_CMD_STATUS_BITS              0x07
#define IT_CMD_STATUS_DONE              0x00
#define IT_CMD_STATUS_BUSY              0x01
#define IT_CMD_STATUS_ERROR             0x02
#define IT_CMD_STATUS_NO_CONN           0x07
#define IT_PT_INFO_BITS                 0xF8
#define IT_PT_INFO_YES                  0x80



#define IT_PD_FLAGS_DATA_TYPE_BITS      0xF0
/* other types (like chip-detected gestures) exist but we do not care */
#define IT_PD_FLAGS_DATA_TYPE_TOUCH 0x00
#define IT_PD_FLAGS_IDLE_TO_ACTIVE      0x10
/* a bit for each finger data that is valid (from lsb to msb) */
#define IT_PD_FLAGS_HAVE_FINGERS        0x07
#define IT_PD_PALM_FLAG_BIT     0x01
#define IT_FD_PRESSURE_BITS     0x0F
#define IT_FD_PRESSURE_NONE     0x00
#define IT_FD_PRESSURE_LIGHT        0x02



typedef struct ft_finger_data_
{
    uint8_t x_lo;
    /* bit0~3: high 4bits of X, bit4~7: high 4bits of Y */
    uint8_t hi;
    uint8_t y_lo;
    uint8_t pressure;
} ft_finger_data_t;

typedef struct ft_point_data_
{
    uint8_t flags;
    uint8_t gesture_id;
    //ft_finger_data_t fd[IT_FINGER_NUM];
    /// TODO: for now only support 1 finger
    ft_finger_data_t fd;
} ft_point_data_t;



// rotate to left with 90, 180, 270
// rotate to left with 360 for mirror
//#define IT_ROTATE_LEFT                 (90)

/* function and value-----------------------------------------------------------*/

static short it7257_count_x(unsigned char *xp);
static short it7257_count_y(unsigned char *xp);
static void it7257_correct_pos(touch_msg_t *ppos);
static rt_err_t write_reg(uint8_t reg, rt_uint8_t data);
static rt_err_t read_regs(rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf);

static struct rt_i2c_bus_device *ft_bus = NULL;
static struct touch_drivers it7257_driver;

void it7257_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("it7257 touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(it7257_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    uint8_t tp_num;
    rt_err_t err;
    ft_point_data_t point_data;

    LOG_D("read_point");
    rt_touch_irq_pin_enable(1);

    if (ft_bus && p_msg)
    {
        err = read_regs(IT_REG_POINT_INFO, sizeof(point_data), (rt_uint8_t *)&point_data);

        if (RT_EOK != err)
        {
            goto ERROR_HANDLE;
        }

        if (point_data.flags & 1)
        {
            p_msg->event = TOUCH_EVENT_DOWN;
            p_msg->x = point_data.fd.x_lo | (((uint16_t)point_data.fd.hi & 0xF) << 8);
            p_msg->y = point_data.fd.y_lo | (((uint16_t)point_data.fd.hi & 0xF0) << 4);


            p_msg->x = IT_MAX_WIDTH - p_msg->x;
            if (p_msg->x < 0)
            {
                p_msg->x = 0;
            }

            p_msg->y = IT_MAX_HEIGHT - p_msg->y;
            if (p_msg->y < 0)
            {
                p_msg->y = 0;
            }



            LOG_D("Down event, x = %d, y = %d\n", p_msg->x, p_msg->y);

            return RT_EEMPTY;
        }
        else
        {
            p_msg->x = 0;
            p_msg->y = 0;
            p_msg->event = TOUCH_EVENT_UP;

            LOG_D("Up event, x = %d, y = %d\n", p_msg->x, p_msg->y);
            return RT_EEMPTY;
        }




    }
    else
    {
        //LOG_I("spi or handle error\n");
    }

ERROR_HANDLE:
    p_msg->x = 0;
    p_msg->y = 0;
    p_msg->event = TOUCH_EVENT_UP;
    LOG_E("Error, return Up event, x = %d, y = %d\n", p_msg->x, p_msg->y);

    return RT_ERROR;
}

static rt_err_t write_reg(uint8_t reg, rt_uint8_t data)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};

    msgs.addr  = IT_DEV_ADDR;    /* slave address */
    msgs.flags = RT_I2C_WR;        /* write flag */
    msgs.buf   = buf;              /* Send data pointer */
    msgs.len   = 2;

    if (rt_i2c_transfer(ft_bus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_err_t read_regs(rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = IT_DEV_ADDR;    /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &reg;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = IT_DEV_ADDR;    /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = buf;              /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(ft_bus, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}


static void it7257_init(void)
{
}



static rt_err_t init(void)
{
    rt_err_t err;
    struct touch_message msg;

    LOG_D("it7257 init irq");

    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, it7257_irq_handler, NULL);
    rt_touch_irq_pin_enable(1); //Must enable before read I2C


    LOG_D("it7257 init OK");
    return RT_EOK;

}

static rt_err_t deinit(void)
{
    LOG_D("it7257 deinit");

    rt_touch_irq_pin_enable(0);
    return RT_EOK;

}

static rt_bool_t probe(void)
{

    ft_bus = (struct rt_i2c_bus_device *)rt_device_find(TOUCH_DEVICE_NAME);
    if (RT_Device_Class_I2CBUS != ft_bus->parent.type)
    {
        ft_bus = NULL;
    }
    if (ft_bus)
    {
        rt_device_open((rt_device_t)ft_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    }
    else
    {
        LOG_I("bus not find\n");
        return RT_FALSE;
    }


    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 500,
            .max_hz  = 400000,
        };

        rt_i2c_configure(ft_bus, &configuration);
    }

    {
        uint32_t id = 0;
        rt_err_t err = read_regs(0x00, sizeof(id), (rt_uint8_t *)&id);
        if (RT_EOK == err)
        {
            LOG_E("it7257 read id %d ", id);
            LOG_I("it7257 probe OK");
        }
        else
        {
            LOG_E("it7257 read id ERROR ");
        }
    }

    return RT_TRUE;
}

static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};

static int rt_it7257_init(void)
{
    it7257_driver.probe = probe;
    it7257_driver.ops = &ops;
    it7257_driver.user_data = RT_NULL;
    it7257_driver.isr_sem = rt_sem_create("it7257", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&it7257_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(rt_it7257_init);


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
