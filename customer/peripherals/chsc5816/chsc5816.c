

/********************************chsc5816 tp********************************/
#include <rtthread.h>
#include <rtdevice.h>

#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_touch.h"
#include "chsc5816.h"


#define  CHSC5816_TP_RST    70
#define CHSC5816_TP_INT 80

#define  DBG_LEVEL            DBG_ERROR  //DBG_LOG //
#define LOG_TAG              "drv.chsc5816"
#include <drv_log.h>
#define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)


static struct touch_drivers chsc5816tp_driver;
//static struct rt_i2c_bus_device *chsc5816tp_i2c_bus = NULL;
static rt_device_t chsc5816tp_i2c_bus = RT_NULL;

static uint8_t handshake_flag = 0;
static uint8_t tp_inited = 0;

static rt_err_t chsc5816tp_i2c_write(uint8_t *buf, uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[1];

    msgs[0].addr  = CHSC5816_I2C_ADDR;    /* slave address */
    msgs[0].flags = RT_I2C_WR;        /* write flag */
    msgs[0].buf   = buf;              /* Send data pointer */
    msgs[0].len   = len;

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)chsc5816tp_i2c_bus->user_data, msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}


static rt_size_t chsc5816tp_i2c_xfer(uint8_t *reg, uint16_t wlen, uint8_t *rbuf, uint16_t rlen)
{
    struct rt_i2c_msg msgs[2];
    rt_size_t res = 0;

    msgs[0].addr  = CHSC5816_I2C_ADDR;    /* slave address */
    msgs[0].flags = RT_I2C_WR;        /* write flag */
    msgs[0].buf   = reg;              /* Send data pointer */
    msgs[0].len   = wlen;

    msgs[1].addr  = CHSC5816_I2C_ADDR;    /* slave address */
    msgs[1].flags = RT_I2C_RD;        /* write flag */
    msgs[1].buf   = rbuf;              /* Send data pointer */
    msgs[1].len   = rlen;

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)chsc5816tp_i2c_bus->user_data, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }

    return res;
}

/*
reg      - register address, must 4B aligned
buffer  - data buffer
len      - data length, must 4B aligned

return:
    0     - pass
 others - fail
*/
int32_t semi_touch_read_bytes(uint32_t reg, uint8_t *buffer, uint16_t len)
{
    int32_t ret = RT_EOK;
    uint32_t once;
    uint8_t writeBuff[4];

    while (len > 0)
    {
        once = (len > MAX_IO_BUFFER_LEN) ? MAX_IO_BUFFER_LEN : len;
        writeBuff[0] = (uint8_t)(reg >> 24);
        writeBuff[1] = (uint8_t)(reg >> 16);
        writeBuff[2] = (uint8_t)(reg >> 8);
        writeBuff[3] = (uint8_t)(reg);
        if (chsc5816tp_i2c_xfer(writeBuff, 4, buffer, (uint16_t)once) != RT_EOK)
        {
            ret = -1;
            break;
        }

        reg    += once;
        buffer += once;
        len    -= once;
    }

    return ret;
}


/*
reg       - register address, must 4B aligned
buffer   - data buffer
len       - data length, must 4B aligned

return:
    0     - pass
 others - fail
*/
int32_t semi_touch_write_bytes(uint32_t reg, uint8_t *buffer, uint16_t len)
{
    int32_t ret = RT_EOK;
    uint8_t writeBuff[MAX_IO_BUFFER_LEN];
    uint32_t once, k;

    while (len > 0)
    {
        once = (len < (MAX_IO_BUFFER_LEN - 4)) ? len : (MAX_IO_BUFFER_LEN - 4);

        writeBuff[0] = (uint8_t)(reg >> 24);
        writeBuff[1] = (uint8_t)(reg >> 16);
        writeBuff[2] = (uint8_t)(reg >> 8);
        writeBuff[3] = (uint8_t)(reg);
        for (k = 0; k < once; k++)
        {
            writeBuff[k + 4] = buffer[k];
        }
        if (chsc5816tp_i2c_write(writeBuff, (uint16_t)once + 4) != RT_EOK)
        {
            ret = -1;
            break;
        }

        reg    += once;
        buffer += once;
        len    -= once;
    }

    return ret;
}

void semi_touch_reset(void)
{
    BSP_TP_Reset(0);
    HAL_Delay(5);
    BSP_TP_Reset(1);
    HAL_Delay(5);

    rt_kprintf("semi_touch_reset-- \r\n");
}


int32_t semi_touch_dect(void)
{
    uint32_t retry;
    uint8_t u32Data[4] = {0};
    for (retry = 0; retry < 3; retry++)
    {
        semi_touch_reset();

        if (!semi_touch_read_bytes(0x2000001c, u32Data, 4)) //Read TP ID.
        {
            rt_kprintf("\nTP PPP u32Data[0],u32Data[1],u32Data[2],u32Data[3], X,Y:%x,%x,%x,%x \n", u32Data[0], u32Data[1], u32Data[2], u32Data[3]);
            return 0;
        }
    }
    return -1;
}

sm_touch_dev st_dev;

void semi_touch_setup_check(void)
{
    int32_t retry = 0;
    uint32_t naFlag = 0;
    img_header_t image_header;
    img_header_t image_confirm;

    //clean boot status
    semi_touch_write_bytes(0x20000018, (uint8_t *)&naFlag, 4);

    semi_touch_reset();

    st_dev.fw_ver    = 0;
    st_dev.vid_pid   = 0;
    st_dev.setup_ok  = 0;//默认状态为失败
    image_header.sig = 0;
    for (retry = 0; retry < 10; retry++)
    {
        HAL_Delay(10);

        if (semi_touch_read_bytes(0x20000014, (uint8_t *)&image_header, sizeof(image_header)) != 0)
        {
            continue;
        }

        HAL_Delay(3);
        if (semi_touch_read_bytes(0x20000014, (uint8_t *)&image_confirm, sizeof(image_confirm)) != 0)
        {
            continue;
        }
        if ((image_header.sig != image_confirm.sig) ||
                (image_header.vid_pid != image_confirm.vid_pid) ||
                (image_header.raw_offet != image_confirm.raw_offet) ||
                (image_header.dif_offet != image_confirm.dif_offet) ||
                (image_header.fw_ver != image_confirm.fw_ver))
        {
            rt_kprintf("chsc::double check, retry\n");
            continue;
        }

        if (image_header.sig == 0x43534843) //"CHSC"
        {
            st_dev.fw_ver   = image_header.fw_ver;
            st_dev.vid_pid  = image_header.vid_pid;
            st_dev.setup_ok = 1;//pass
            break;
        }
        else if (image_header.sig == 0x4F525245)  //boot self check fail
        {
            break;
        }
    }

    return;
}

void chsc5816tp_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(chsc5816tp_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t semi_touch_get_points(touch_msg_t p_msg)
{
    uint8_t data[12];
    rt_err_t ret = RT_ERROR;

    rt_touch_irq_pin_enable(1);

    // MAX 1 points: 8byte
    // MAX 2 points: 12byte
    if (semi_touch_read_bytes(0x2000002c, data, 12))
    {
        rt_kprintf("chsc:read pixel data fail\n");
        p_msg->event = TOUCH_EVENT_UP;
        p_msg->x     = 0;
        p_msg->y     = 0;
        return RT_ERROR;
    }

    p_msg->event = TOUCH_EVENT_UP;
    p_msg->x = 0;
    p_msg->y = 0;
    if ((data[0] == 0xff) && (data[1] <= 2)) //data[0]:0xff-data,data[1]:point number
    {
        if (data[1] > 0) //1,2 point,0 抬起
        {
            ret = RT_EEMPTY;
            p_msg->event = TOUCH_EVENT_DOWN;
            p_msg->x = ((data[5] << 8) & 0xf00) | data[2];
            p_msg->y = ((data[5] << 4) & 0xf00) | data[3];
            rt_kprintf("\nTP QQQ data[5],data[0],data[1], X,Y:%x,%x,%x,%x,%x\n", data[5], data[0], data[1], p_msg->x, p_msg->y);
        }
    }

    return ret;
}

static rt_err_t semi_touch_init(void)
{
    struct touch_message msg;

    rt_err_t ret = RT_EOK;

    rt_kprintf("semi_touch_init++\n");

    if (semi_touch_dect())
    {
        rt_kprintf("chsc:not detected\n");
        return RT_EIO;
    }

    semi_touch_setup_check();

    semi_check_and_update(&st_dev);
    semi_touch_reset();

    // enable int
    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, chsc5816tp_irq_handler, RT_NULL);
    rt_touch_irq_pin_enable(1);

    rt_kprintf("semi_touch_init OK\n");


    return ret;
}

static rt_err_t deinit(void)
{
    rt_kprintf("chsc5816tp deinit\n");

    rt_touch_irq_pin_enable(0);


    return RT_EOK;
}

static rt_bool_t probe(void)
{
    rt_err_t err;


    chsc5816tp_i2c_bus = rt_device_find(TOUCH_DEVICE_NAME);
    if (chsc5816tp_i2c_bus)
    {
        rt_device_open(chsc5816tp_i2c_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    }
    else
    {
        rt_kprintf("bus not find\n");
        return RT_FALSE;
    }

    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 5000,
            .max_hz = 200000,
        };

        rt_i2c_configure((struct rt_i2c_bus_device *)chsc5816tp_i2c_bus->user_data, &configuration);
    }


    rt_kprintf("chsc5816tp probe OK\n");

    return RT_TRUE;
}


static struct touch_ops ops =
{
    semi_touch_get_points,
    semi_touch_init,
    deinit
};


static int rt_chsc5816tp_init(void)
{
    chsc5816tp_driver.probe = probe;
    chsc5816tp_driver.ops = &ops;
    chsc5816tp_driver.user_data = RT_NULL;
    chsc5816tp_driver.isr_sem = rt_sem_create("chsc5816tp", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&chsc5816tp_driver);

    return 0;
}

INIT_COMPONENT_EXPORT(rt_chsc5816tp_init);
void touch_esd_handler(void)
{
    int32_t status = 0, retry = 0;

    for (retry = 0; retry < 3; retry++)
    {
        semi_touch_read_bytes(0x20000018, (uint8_t *)&status, 4);
        if (status == 0x43534843) //"CHSC"
        {
            break;
        }
        semi_touch_reset();

        rt_thread_mdelay(5);//Delay 5ms
    }
#if 1//(defined(EN_ESD_AUTO_UPD)) && (defined(OY8_CHSC_XSJ))
    if (status != 0x43534843)
    {
        //if(semi_check_and_update(chsc_upd_data, sizeof(chsc_upd_data)) != 0){
        return;
        //}
    }
    else
    {
        status = 0;
        semi_touch_write_bytes(0x20000018, (uint8_t *)&status, 4);
    }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
