/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "board.h"
#include "bl6133.h"
#include "drv_touch.h"
#include "drv_io.h"
//#include "EventRecorder.h"

/* Define -------------------------------------------------------------------*/

#define I2C_ADDR                    (0x2C)

#define BL_MAX_WIDTH                   (454)
#define BL_MAX_HEIGHT                  (454)

#pragma pack(push, 1) //make sure no padding bytes between report_id and desc

typedef struct _touch_desc_t
{
    uint8_t GestureCode;
    uint8_t PointNum;

    uint8_t PosXH : 6;
    uint8_t Event : 2;
    uint8_t PosXL;

    uint8_t PosYH : 4;
    uint8_t PosID : 4;
    uint8_t PosYL;

    uint8_t Reserved1;
    uint8_t Reserved2;
} touch_desc_t;

#pragma pack(pop)

// rotate to left with 90, 180, 270
// rotate to left with 360 for mirror
//#define IT_ROTATE_LEFT                 (90)

/* function and value-----------------------------------------------------------*/

static struct rt_i2c_bus_device *i2c_bus = NULL;
static uint32_t start_tick;
static struct touch_drivers bl6133_driver;

#ifdef BL_UPDATE_FIRMWARE_ENABLE
int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len, unsigned char rw)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = i2c_addr;    /* slave address */
    msgs.flags = (0 == rw) ? RT_I2C_WR : RT_I2C_RD;        /* r/w flag */
    msgs.buf   = buf;              /* Send data pointer */
    msgs.len   = len;

    if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

int bl_read_fw(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *buf, int len)
{
    if (rt_i2c_mem_read(i2c_bus, i2c_addr, reg_addr, 8, buf, len) == len)
        return 0;
    else
        return -RT_ERROR;

}

void bl_ts_set_intmode(char mode)
{
    if (0 == mode)  //GPIO mode
    {
        //CTP_SET_I2C_EINT_OUTPUT;

        rt_pin_mode(TOUCH_IRQ_PIN, PIN_MODE_OUTPUT);
    }
    else if (1 == mode) //INT mode
    {
        //CTP_SET_I2C_EINT_INPUT;

        rt_pin_mode(TOUCH_IRQ_PIN, PIN_MODE_INPUT);
    }
}

void bl_ts_set_intup(char level)
{
    if (level == 1)
        rt_pin_write(TOUCH_IRQ_PIN, 1);
    else if (level == 0)
        rt_pin_write(TOUCH_IRQ_PIN, 0); //CTP_SET_I2C_EINT_LOW;
}

void bl_ts_reset_wakeup(void)
{
    LOG_D("bl_ts_reset_wakeup");

    //CTP_SET_RESET_PIN_OUTPUT;
    BSP_TP_Reset(1); //CTP_SET_RESET_PIN_HIGH;
    rt_thread_mdelay(20);
    BSP_TP_Reset(0); //CTP_SET_RESET_PIN_LOW;
    rt_thread_mdelay(20);
    BSP_TP_Reset(1); //CTP_SET_RESET_PIN_HIGH;
    rt_thread_mdelay(20);
}

#endif /* BL_UPDATE_FIRMWARE_ENABLE */

void bl6133_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("bl6133 touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(bl6133_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("bl6133 read_point");
    rt_touch_irq_pin_enable(1);

    touch_desc_t touch_msg;

    if (rt_i2c_mem_read(i2c_bus, I2C_ADDR, 0x01, 8, &touch_msg, sizeof(touch_msg)) == sizeof(touch_msg))
    {
        rt_uint16_t x, y;

        x = (touch_msg.PosXH << 8) | touch_msg.PosXL;
        y = (touch_msg.PosYH << 8) | touch_msg.PosYL;

        LOG_D("PosIDID=%d, PointNum=%d,Event=%d, GestureCode=%d, X=%d, Y=%d\n",
              touch_msg.PosID, touch_msg.PointNum, touch_msg.Event, touch_msg.GestureCode, x, y);

        if (touch_msg.PointNum)
        {
            ///x = BL_MAX_WIDTH - pt_desc->record[i].x;
            // y = BL_MAX_HEIGHT - pt_desc->record[i].y;

            if (touch_msg.Event == 1)
            {
                LOG_D("Lift, X=%d, Y=%d\n", x, y);
                p_msg->event = TOUCH_EVENT_UP;
                p_msg->x = 0;
                p_msg->y = 0;
                ret = RT_EEMPTY;

            }
            else if (touch_msg.Event == 2)
            {
                LOG_D("Down, X=%d, Y=%d\n", x, y);
                p_msg->event = TOUCH_EVENT_DOWN;
                p_msg->x = x;
                p_msg->y = y;
                ret = RT_EEMPTY;
            }
            else
            {
            }

        }
        else
        {
            LOG_D("Lift, X=%d, Y=%d\n", x, y);
            p_msg->event = TOUCH_EVENT_UP;
            p_msg->x = 0;
            p_msg->y = 0;
            ret = RT_EEMPTY;
        }

    }

    return ret;
}

static rt_err_t init(void)
{
    struct touch_message msg;

    LOG_I("bl6133 init");

    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, bl6133_irq_handler, NULL);
    rt_touch_irq_pin_enable(1); //Must enable before read I2C

    start_tick = rt_tick_get();

#if 0
    while (RT_EINVAL != read_point(&msg))
    {
        //Read all previous commands report.
        if ((rt_tick_get() - start_tick) > RT_TICK_PER_SECOND)
        {
            LOG_E("bl6133 init read point TIMEOUT!");
            break;
        }
    }
#else
    read_point(&msg);
#endif /* 0 */

    LOG_D("bl6133 init OK");
    return RT_EOK;

}

static rt_err_t deinit(void)
{
    LOG_D("bl6133 deinit");

    rt_touch_irq_pin_enable(0);

    return RT_EOK;
}

static rt_bool_t probe(void)
{
    rt_err_t err;

    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(TOUCH_DEVICE_NAME);
    if (RT_Device_Class_I2CBUS != i2c_bus->parent.type)
    {
        i2c_bus = NULL;
    }
    if (i2c_bus)
    {
        rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
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
            .timeout = 50,
            .max_hz  = 200000,
        };

        rt_i2c_configure(i2c_bus, &configuration);
    }

    rt_touch_irq_pin_enable(0);

#if 0//def BL_UPDATE_FIRMWARE_ENABLE
    LOG_I("bl6133 auto updata fw......");
#ifdef  RESET_PIN_WAKEUP
    bl_ts_reset_wakeup();
#endif
    if (bl_auto_update_fw() < 0)
    {
        LOG_I("bl6133 auto updata fw Fail.");

    }
    else
    {

        LOG_I("bl6133 auto updata fw Sucess.");
    }
#endif /* BL_UPDATE_FIRMWARE_ENABLE */

    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 500,
            .max_hz  = 400000,
        };

        rt_i2c_configure(i2c_bus, &configuration);
    }

    LOG_I("bl6133 probe OK");

    return RT_TRUE;
}

static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};

static int rt_bl6133_init(void)
{
    bl6133_driver.probe = probe;
    bl6133_driver.ops = &ops;
    bl6133_driver.user_data = RT_NULL;
    bl6133_driver.isr_sem = rt_sem_create("bl6133", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&bl6133_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(rt_bl6133_init);

