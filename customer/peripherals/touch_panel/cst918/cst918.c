/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "board.h"
#include "cst918.h"
#include "drv_touch.h"
#include "drv_io.h"
//#include "EventRecorder.h"

/* Define -------------------------------------------------------------------*/

#define  DBG_LEVEL            DBG_ERROR  //DBG_LOG //
#define LOG_TAG              "drv.cst918"
#include <drv_log.h>

#define CST918_UPDATA_ENABLE            /* ʹ������ */

#define TOUCH_SLAVE_ADDRESS             (0x1A)  /* ������ַ */

#define TOUCH_CHIP_ID_CST716            (0x20)
#define TOUCH_CHIP_ID_CST816T           (0xB5)
#define TOUCH_CHIP_ID_CST918            (0x396242e)
#define TOUCH_CHIP_ID_CST918A           (0x396241a)

/*register address*/
#define FTS_REG_CHIP_ID                 0xD204     /* ��ȡID */
#define FTS_REG_FW_VER                  0xD208      /* ȡҽ汾 */
#define FTS_REG_MODE_DEBUG_INFO         0xD101     /* debugģʽ */
#define FTS_REG_MODE_DEEP_SLEEP         0xD105     /* sleepģʽ */
#define FTS_REG_MODE_NORMOL             0xD109     /* ��������ģʽ */
#define FTS_REG_GET_POSI                0xD000     /* ��ȡ�����λ�?*/

#define TOUCH_WRITE_MAX                 (32)    /* дд뻺 */

/* function and value-----------------------------------------------------------*/
static struct rt_i2c_bus_device *ft_bus = NULL;
static struct touch_drivers cst918_driver;

rt_err_t i2c_base_write(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = TOUCH_SLAVE_ADDRESS;    /* slave address */
    msgs.flags = RT_I2C_WR;        /* write flag */
    msgs.buf   = buf;              /* Send data pointer */
    msgs.len   = len;

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

rt_err_t i2c_base_read(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = TOUCH_SLAVE_ADDRESS;    /* Slave address */
    msgs.flags = RT_I2C_RD;        /* Read flag */
    msgs.buf   = buf;              /* Read data pointer */
    msgs.len   = len;              /* Number of bytes read */

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

#if  0

rt_err_t cst918_i2c_write(uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t buf[TOUCH_WRITE_MAX + 2];

    if (len > TOUCH_WRITE_MAX)
    {
        len = TOUCH_WRITE_MAX;
    }

    buf[0] = reg >> 8;  //cmd
    buf[1] = reg;       //cmd
    rt_memcpy((buf + 2), data, len);

    /* rt_hw_us_delay(20); */

    if (RT_EOK == i2c_base_write(buf, (len + 2)))
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

rt_err_t cst918_i2c_read(const uint16_t reg, uint8_t *p_data, uint8_t len)
{
    rt_int8_t res = 0;
    rt_uint8_t buf[2];
    struct rt_i2c_msg msgs[2];

    buf[0] = reg >> 8;  //cmd
    buf[1] = reg;       //cmd
    msgs[0].addr  = TOUCH_SLAVE_ADDRESS;    /* Slave address */
    msgs[0].flags = RT_I2C_WR;              /* Read flag */
    msgs[0].buf   = buf;                    /* Read data pointer */
    msgs[0].len   = 2;                      /* Number of bytes read */

    msgs[1].addr  = TOUCH_SLAVE_ADDRESS;    /* Slave address */
    msgs[1].flags = RT_I2C_RD;              /* Read flag */
    msgs[1].buf   = p_data;                 /* Read data pointer */
    msgs[1].len   = len;                    /* Number of bytes read */

    if (rt_i2c_transfer(ft_bus, msgs, 2) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

#else

uint32_t cst918_i2c_write(uint16_t reg, uint8_t *p_data, uint16_t len)
{
    if ((p_data) && (len != 0))
    {
        uint32_t res;

        res = rt_i2c_mem_write(ft_bus, TOUCH_SLAVE_ADDRESS, reg, 16, p_data, len);  /* not I2C_MEMADD_SIZE_16BIT !!!  */

        return res;
    }
    else
    {
        rt_uint8_t buf[2];

        buf[0] = reg >> 8;  //cmd
        buf[1] = reg;       //cmd

        return i2c_base_write(buf, 2);
    }
}

uint32_t cst918_i2c_read(const uint16_t reg, uint8_t *p_data, uint8_t len)
{
    uint32_t res;

    res = rt_i2c_mem_read(ft_bus, TOUCH_SLAVE_ADDRESS, reg, 16, p_data, len);  /* not I2C_MEMADD_SIZE_16BIT !!!  */

    return res;
}
#endif

void cst918_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("cst918 touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(cst918_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("cst918 read_point");
    rt_touch_irq_pin_enable(1);

    {
        uint8_t rbuf[4] = {0};
        uint8_t press = 0;
        uint8_t data = 0xAB;

        cst918_i2c_read(FTS_REG_GET_POSI, (uint8_t *)rbuf, 4);
        cst918_i2c_write(FTS_REG_GET_POSI, &data, 1);

        press = rbuf[0] & 0x0F;
        if (press == 0x06)
        {
            p_msg->event = TOUCH_EVENT_DOWN;
            //touch_info->state = 1;
        }
        else
        {
            p_msg->event = TOUCH_EVENT_UP;
            //touch_info->state = 0;
        }

        p_msg->x = (((uint16_t)(rbuf[1])) << 4) | (rbuf[3] >> 4);
        p_msg->y = (((uint16_t)(rbuf[2])) << 4) | (rbuf[3] & 0x0f);

        LOG_D("event=%d, X=%d, Y=%d\n", p_msg->event, p_msg->x, p_msg->y);

    }

    return RT_EEMPTY; //No more data to be read
}

#ifdef CST918_UPDATA_ENABLE
    extern int cst9xx_boot_update_fw(void);
#endif /* CST918_UPDATA_ENABLE */

static rt_err_t init(void)
{
    struct touch_message msg;
    rt_uint8_t test_buff[8];

    LOG_D("cst918 init");

    BSP_TP_Reset(0);
    rt_thread_mdelay(3);
    BSP_TP_Reset(1);
    rt_thread_mdelay(50);

#ifdef  CST918_UPDATA_ENABLE
    cst9xx_boot_update_fw();
#else
    cst918_i2c_write(FTS_REG_MODE_DEBUG_INFO, test_buff, 0);
    /* cst918_get_chip_id(&m_touch_id); */
#endif

    cst918_i2c_write(FTS_REG_MODE_NORMOL, test_buff, 0);

    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, cst918_irq_handler, NULL);
    rt_touch_irq_pin_enable(1);     //Must enable before read I2C

    LOG_D("cst918 init OK");
    return RT_EOK;

}

static rt_err_t deinit(void)
{
    LOG_D("cst918 deinit");

    rt_touch_irq_pin_enable(0);

#if  0
    //uint8_t value = 0;
    const uint8_t sleep_cmd = CHIP_59_ENTER_SLEEP;
    if (!m_touch_init)
    {
        return;
    }

    rt_kprintf("touch_suspend");
    if (m_touch_id == TOUCH_CHIP_ID_CST816T || m_touch_id == TOUCH_CHIP_ID_CST918 || m_touch_id == TOUCH_CHIP_ID_CST918A)
    {
        m_touch_state = 0;
        if (dev_i2c_rtl876x_init_start(TOUCH_I2C_INDEX, TOUCH_I2C_SPEED) == USER_SUCCESS)
        {
            //private_printf("fun:%s line:%d",__FUNCTION__, __LINE__);
            cst918_i2c_write(0xD105, NULL, 0);
            dev_i2c_rtl876x_init_stop(TOUCH_I2C_INDEX);
        }
    }

    Pad_Config(TOUCH_INT, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);
#endif

    return RT_EOK;
}

static rt_bool_t probe(void)
{
    rt_err_t err;

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

    LOG_I("cst918 probe OK");

    return RT_TRUE;
}

static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};

static int rt_cst918_init(void)
{
    cst918_driver.probe = probe;
    cst918_driver.ops = &ops;
    cst918_driver.user_data = RT_NULL;
    cst918_driver.isr_sem = rt_sem_create("cst918", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&cst918_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(rt_cst918_init);

