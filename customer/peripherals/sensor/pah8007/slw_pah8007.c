/**
  ******************************************************************************
  * @file   slw_pah8007.c
  * @author Sifli software development team
  ******************************************************************************
*/



#include "slw_pah8007.h"

#include "ulog.h"

#ifdef RT_USING_SENSOR

//#define DBG_TAG "sensor.pah8007"
//#define DBG_LVL DBG_INFO
//#include <rtdbg.h>


#define PAH8007_ADDR               0x48


static struct rt_i2c_bus_device *i2c_bus = RT_NULL;


void pah8007_write_reg(uint8_t reg, uint8_t data)
{
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;


    if (i2c_bus)
    {

        value[0] = reg;
        value[1] = data;

        msgs[0].addr = PAH8007_ADDR;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf = value;
        msgs[0].len = 2;

        res = rt_i2c_transfer(i2c_bus, msgs, 1);
        if (res == 1)
        {

        }
        else
        {
            LOG_D("pah8007_write_regs fail\n");
        }
    }
}


void pah8007_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    struct rt_i2c_msg msgs[2];
    uint32_t res;
    uint8_t i;
    LOG_D("pah8007_read_reg:0x%x->0x%x,len:%x\r\n", reg, buf[0], len);

    if (i2c_bus)
    {

        msgs[0].addr = PAH8007_ADDR;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf = &reg;
        msgs[0].len = 1;

        msgs[1].addr = PAH8007_ADDR;
        msgs[1].flags = RT_I2C_RD;
        msgs[1].buf = buf;
        msgs[1].len = len;

        res = rt_i2c_transfer(i2c_bus, msgs, 2);
        if (res == 2)
        {
            for (i = 0; i <= len; i++)
            {
                LOG_D("pah8007_read_reg:0x%x[%x]->0x%x\r\n", reg, i, buf[i]);
            }
        }
        else
        {
            LOG_D("pah8007_read_reg fail:reg:0x%x\n", reg);
        }
    }
}


void *pah8007_get_bus(void)
{
    return (void *)i2c_bus;
}

uint8_t pah8007_get_dev_addr(void)
{
    return PAH8007_ADDR;
}


#include "pah_8007.h"


int pah8007_init(void)
{
    uint8_t pah8007_id = 0;
    rt_err_t rst = RT_EOK;
    /*get i2c bus device*/
    i2c_bus = rt_i2c_bus_device_find(PAH8007_I2C_BUS);

    if (i2c_bus)
    {

        LOG_D("Find i2c bus device %s\n", PAH8007_I2C_BUS);
        rt_kprintf("pah8007_init Find i2c bus device2 %s\r\n", PAH8007_I2C_BUS);
    }
    else
    {

        LOG_E("Can not find i2c bus %s ,pah8007_init fail\n", PAH8007_I2C_BUS);
        rt_kprintf("pah8007_init Find i2c bus device2 %s\r\n", PAH8007_I2C_BUS);
        return -1;
    }
#if 1 //cgdeng
    rst = rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != rst)
    {
        LOG_E("pah8007_init open err!");
        return RT_FALSE;
    }
#endif
#if 0
    pah8007_read_reg(0x00, &pah8007_id, 1);

    rt_kprintf("PAH8007 init : ChipID[0x%x] \n", pah8007_id);

    if (pah8007_id == 0x63)
    {

        LOG_D("PAH8007 init successful : ChipID[0x%x] \n", pah8007_id);
    }
    else
    {
        LOG_D("PAH8007 init fail\n");
        return -1;
    }
#endif


    if (!pah_sensor_init())
    {
        return -1;
    }


    return 0;
}

#endif // RT_USING_SENSOR
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
