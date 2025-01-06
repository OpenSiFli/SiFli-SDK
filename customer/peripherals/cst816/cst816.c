/**
  ******************************************************************************
  * @file   cst816.c
  * @author Sifli software development team
  ******************************************************************************
*/

#include <rtthread.h>
#include "board.h"
#include "cst816.h"
#include "drv_touch.h"
#include "drv_io.h"
#include "mem_section.h"

/* Define -------------------------------------------------------------------*/

#define  DBG_LEVEL            DBG_LOG  //DBG_LOG //
#define LOG_TAG              "drv.cst816"
#include <drv_log.h>



#ifdef LCD_USING_ED_LB5XSPI18501
    #define CST816_UPDATA_ENABLE            /* ʹ������ */
#endif /* LCD_USING_ED_LB5XSPI18501 */

#define TOUCH_SLAVE_ADDRESS             (0x15)  /* ������ַ */

#define TOUCH_CHIP_ID_CST716            (0x20)
#define TOUCH_CHIP_ID_CST816T           (0xB5)
#define TOUCH_CHIP_ID_CST816D           (0xB6)


/*register address*/
#define FTS_REG_CHIP_ID                 0xA7       /* ID */
#define FTS_REG_MODE_DEEP_SLEEP         0xD105     /* sleepģʽ */
#define FTS_REG_MODE_NORMOL             0xD109     /* ��������ģʽ */
#define FTS_REG_GET_POSI                0xD000     /* ��ȡ�����λ��?*/





/* function and value-----------------------------------------------------------*/
#define CST816_USE_DMA_RW //Enable I2C DMA read/write

#ifdef CST816_USE_DMA_RW
    /*
    The DMA r/w buffer,
    must be aligned to '__SCB_DCACHE_LINE_SIZE', if it is PSRAM.
    */
    #ifdef BSP_USING_PSRAM

        /* Using PSRAM buffer */
        ALIGN(__SCB_DCACHE_LINE_SIZE)
        L2_NON_RET_BSS_SECT_BEGIN(rbuf)
        static uint8_t rbuf[__SCB_DCACHE_LINE_SIZE * 1];
        L2_NON_RET_BSS_SECT_END

    #else

        /* Using SRAM buffer */
        L1_NON_RET_BSS_SECT_BEGIN(rbuf)
        static uint8_t rbuf[5];
        L2_NON_RET_BSS_SECT_END

    #endif /* BSP_USING_PSRAM */
#endif /* CST816_USE_DMA_RW */

static struct rt_i2c_bus_device *ft_bus = NULL;
static struct touch_drivers cst816_driver;


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

//

rt_err_t cst816_i2c_read_reg8(uint8_t reg, uint8_t *p_data, uint8_t len)
{
    rt_int8_t res = 0;
    //rt_uint8_t buf[2];
    struct rt_i2c_msg msgs[2];

    //buf[0] = reg >> 8;  //cmd
    //buf[1] = reg;       //cmd
    msgs[0].addr  = TOUCH_SLAVE_ADDRESS;    /* Slave address */
    msgs[0].flags = RT_I2C_WR;              /* Read flag */
    msgs[0].buf   = &reg;                    /* Read data pointer */
    msgs[0].len   = 1;                      /* Number of bytes read */

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




#if  0

rt_err_t cst816_i2c_write(uint16_t reg, uint8_t *data, uint16_t len)
{
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

rt_err_t cst816_i2c_read(const uint16_t reg, uint8_t *p_data, uint8_t len)
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

rt_size_t cst816_i2c_write(uint8_t device_addr, uint16_t reg, uint8_t *p_data, uint16_t len)
{
    rt_size_t res;

    res = rt_i2c_mem_write(ft_bus, device_addr, reg, 16, p_data, len);  /* not I2C_MEMADD_SIZE_16BIT !!!  */

    return res;
}


rt_size_t cst816_i2c_read(uint8_t device_addr, const uint16_t reg, uint8_t *p_data, uint16_t len)
{
    rt_size_t res;

    res = rt_i2c_mem_read(ft_bus, device_addr, reg, 16, p_data, len);  /* not I2C_MEMADD_SIZE_16BIT !!!  */

    return res;
}

#endif


void cst816_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    //LOG_D("cst816 touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(cst816_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    rt_err_t ret = RT_ERROR;

    //LOG_D("cst816 read_point");
    rt_touch_irq_pin_enable(1);

    {
#ifndef CST816_USE_DMA_RW
        uint8_t rbuf[5] = {0};
#endif /* CST816_USE_DMA_RW */
        uint8_t press = 0;

        cst816_i2c_read_reg8(0x02, rbuf, 5);
        //cst816_i2c_read(0x15,0x02, rbuf, 5);

        //  rt_kprintf("rbuf=%x,%x,%x,%x,%x\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4]);

        press = rbuf[0] & 0x0F;
        if (press == 0x01)
        {
            p_msg->event = TOUCH_EVENT_DOWN;
            //touch_info->state = 1;
        }
        else
        {
            p_msg->event = TOUCH_EVENT_UP;
            //touch_info->state = 0;
        }

        p_msg->x = (((uint16_t)(rbuf[1] & 0x0F)) << 8) | rbuf[2];
        p_msg->y = (((uint16_t)(rbuf[3] & 0X0F)) << 8) | rbuf[4];

        // rt_kprintf("event=%d, X=%d, Y=%d\n", p_msg->event, p_msg->x, p_msg->y);

    }

    return RT_EEMPTY; //No more data to be read
}

#ifdef CST816_UPDATA_ENABLE
    extern  bool ctp_hynitron_update(void);
#endif /* CST816_UPDATA_ENABLE */

static rt_err_t init(void)
{
    LOG_D("cst816 init");

#if 0
    BSP_TP_Reset(1);
    rt_thread_mdelay(20);
    BSP_TP_Reset(0);
    rt_thread_mdelay(20);
    BSP_TP_Reset(1);
    rt_thread_mdelay(100);
#endif
#ifdef  CST816_UPDATA_ENABLE
    ctp_hynitron_update();
#else
    uint8_t id = 0;
    cst816_i2c_read_reg8(FTS_REG_CHIP_ID, &id, 1);
    rt_kprintf("cts816 id=0x%x\n", id);
#endif
    /*
        {
            struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = 0,
                .timeout = 5000,
                .max_hz  = 400000,
            };

            rt_i2c_configure(ft_bus, &configuration);
        }
    */
    LOG_I("cst816 probe OK");


    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, cst816_irq_handler, NULL);
    rt_touch_irq_pin_enable(1);     //Must enable before read I2C


    LOG_D("cst816 init OK");
    return RT_EOK;

}

static rt_err_t deinit(void)
{
    LOG_D("cst816 deinit");

    rt_touch_irq_pin_enable(0);

#if  0
    //uint8_t value = 0;
    const uint8_t sleep_cmd = CHIP_59_ENTER_SLEEP;
    if (!m_touch_init)
    {
        return;
    }

    rt_kprintf("touch_suspend");
    if (m_touch_id == TOUCH_CHIP_ID_CST816T || m_touch_id == TOUCH_CHIP_ID_CST816 || m_touch_id == TOUCH_CHIP_ID_CST816A)
    {
        m_touch_state = 0;
        if (dev_i2c_rtl876x_init_start(TOUCH_I2C_INDEX, TOUCH_I2C_SPEED) == USER_SUCCESS)
        {
            //private_printf("fun:%s line:%d",__FUNCTION__, __LINE__);
            cst816_i2c_write(0xD105, NULL, 0);
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
#ifdef CST816_USE_DMA_RW
        rt_device_open((rt_device_t)ft_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_TX | RT_DEVICE_FLAG_DMA_RX);
#else
        rt_device_open((rt_device_t)ft_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
#endif /* CST816_USE_DMA_RW */
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
            .timeout = 5000,
            .max_hz  = 400000,
        };

        rt_i2c_configure(ft_bus, &configuration);
    }

    LOG_I("cst816 probe OK");

    return RT_TRUE;
}


static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};


static int rt_cst816_init(void)
{
    cst816_driver.probe = probe;
    cst816_driver.ops = &ops;
    cst816_driver.user_data = RT_NULL;
    cst816_driver.isr_sem = rt_sem_create("cst816", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&cst816_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(rt_cst816_init);

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

