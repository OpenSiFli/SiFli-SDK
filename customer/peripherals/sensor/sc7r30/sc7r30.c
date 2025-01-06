#include <rtthread.h>
#include "board.h"
#include "sc7r30.h"


#define LOG_TAG              "i2c.7r30"
#include <drv_log.h>

#ifdef HR_USING_SC7R30

//#define SC7R30_I2C_BUS          "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define SC7R30_ADDR               0x38   /* 从机地址 */

/*
*  SC7R30 chip id
*/
//#define SC7R30_CHIP_ID              (0x71)
//cmd
//#define SC7R30_ID_CMD               0X0F

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;     /* I2C总线设备句柄 */
static rt_bool_t initialized = RT_FALSE;                /* 传感器初始化状态 */


void sc7r30_write_reg(uint8_t reg, uint8_t data)
{
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;


    if (i2c_bus)
    {

        value[0] = reg;
        value[1] = data;

        msgs[0].addr = SC7R30_ADDR;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf = value;
        msgs[0].len = 2;

        res = rt_i2c_transfer(i2c_bus, msgs, 1);
        if (res == 1)
        {

        }
        else
        {
            LOG_D("sc7r30_write_regs fail\n");
        }
    }
}


void sc7r30_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (i2c_bus)
    {

        msgs[0].addr = SC7R30_ADDR;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf = &reg;
        msgs[0].len = 1;

        msgs[1].addr = SC7R30_ADDR;
        msgs[1].flags = RT_I2C_RD;
        msgs[1].buf = buf;
        msgs[1].len = len;

        res = rt_i2c_transfer(i2c_bus, msgs, 2);
        if (res == 2)
        {
        }
        else
        {
            LOG_D("sc7r30_read_reg fail\n");
        }
    }
}



int sc7r30_init(void)
{
    uint8_t sc7r30_id = 0;

    /*get i2c bus device*/
    i2c_bus = rt_i2c_bus_device_find(SC7R30_I2C_BUS);

    if (i2c_bus)
    {

        LOG_D("Find i2c bus device %s\n", SC7R30_I2C_BUS);
    }
    else
    {

        LOG_E("Can not find i2c bus %s ,sc7r30_init fail\n", SC7R30_I2C_BUS);
        return -1;
    }
#if 0
    sc7r30_read_reg(0x0f, &sc7r30_id, 1);

    if (sc7r30_id == 0x71)
    {

        LOG_D("SC7R30 init successful : ChipID[0x%x] \n", sc7r30_id);
    }
    else
    {
        LOG_D("SC7R30 init fail\n");
        return -1;
    }
#endif
    return 0;
}


void *sc7r30_get_bus(void)
{
    return (void *)i2c_bus;
}

uint8_t sc7r30_get_dev_addr(void)
{
    return SC7R30_ADDR;
}

uint8_t sc7r30_get_id(void)
{
    //return    SC7R30_CHIP_ID;
    return 0;
}

int sc7r30_acce_read(int *ps_x, int *ps_y, int *ps_z)
{
    return 0;
}



#define DRV_SC7R30_TEST

#ifdef DRV_SC7R30_TEST
#include <string.h>

int sc7r30_test(int argc, char *argv[])
{

    if (argc < 2)
    {
        LOG_I("Invalid parameter \n");
        return 1;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        int res = sc7r30_init();
        if (!res)
        {
            LOG_I("Open sc7r30 success\n");
        }
        else
        {
            LOG_I("Open sc7r30 fail\n");
        }
    }

    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t id = 0;
        sc7r30_read_reg(0x0F, &id, 1);
        LOG_I("sc7r30 chip id 0x%x", id);
    }

    return 0;
}


FINSH_FUNCTION_EXPORT_ALIAS(sc7r30_test, __cmd_sc7r, Test driver sc7r30);

#endif

#endif