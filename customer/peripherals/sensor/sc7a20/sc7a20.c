#include <rtthread.h>
#include "board.h"
#include "sc7a20.h"


#define LOG_TAG              "i2c.7a20c"
#include "ulog.h"

#ifdef ACC_USING_SC7A20

//#define SC7A20_I2C_BUS          "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define SC7A20_ADDR               0x19   /* 从机地址 */

/*
*  SC7A20 chip id
*/
//#define SC7A20_CHIP_ID              (0x11)
//cmd
//#define SC7A20_ID_CMD               0X0F

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;     /* I2C总线设备句柄 */
static rt_bool_t initialized = RT_FALSE;                /* 传感器初始化状态 */


void sc7a20_write_reg(uint8_t reg, uint8_t data)
{
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;


    if (i2c_bus)
    {

        value[0] = reg;
        value[1] = data;

        msgs[0].addr = SC7A20_ADDR;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf = value;
        msgs[0].len = 2;

        res = rt_i2c_transfer(i2c_bus, msgs, 1);
        if (res == 1)
        {

        }
        else
        {
            LOG_D("sc7a20_write_regs fail\n");
        }
    }
}


void sc7a20_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (i2c_bus)
    {

        msgs[0].addr = SC7A20_ADDR;
        msgs[0].flags = RT_I2C_WR;
        msgs[0].buf = &reg;
        msgs[0].len = 1;

        msgs[1].addr = SC7A20_ADDR;
        msgs[1].flags = RT_I2C_RD;
        msgs[1].buf = buf;
        msgs[1].len = len;

        res = rt_i2c_transfer(i2c_bus, msgs, 2);
        if (res == 2)
        {
        }
        else
        {
            LOG_D("sc7a20_read_reg fail\n");
        }
    }
}



int sc7a20_init(void)
{
    uint8_t chip_id = 0;
    rt_err_t rst = RT_EOK;
    /*get i2c bus device*/
    i2c_bus = rt_i2c_bus_device_find(SC7A20_I2C_BUS);

    if (i2c_bus)
    {

        LOG_D("Find i2c bus device %s\n", SC7A20_I2C_BUS);
    }
    else
    {

        LOG_E("Can not find i2c bus %s ,sc7a20_init fail\n", SC7A20_I2C_BUS);
        return -1;
    }

    rst = rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != rst)
    {
        LOG_E("sc7a20_init open err!");
        return RT_FALSE;
    }
#if 0
    sc7a20_read_reg(SC7A20_ID_CMD, &chip_id, 1);

    if (chip_id == SC7A20_CHIP_ID)
    {

        LOG_D("SC7A20 init successful : ChipID[0x%x] \n", chip_id);
    }
    else
    {
        LOG_D("SC7A20 init fail\n");
        return -1;
    }
#endif
    return 0;
}


void *sc7a20_get_bus(void)
{
    return (void *)i2c_bus;
}

uint8_t sc7a20_get_dev_addr(void)
{
    return SC7A20_ADDR;
}

uint8_t sc7a20_get_id(void)
{
    //return    SC7A20_CHIP_ID;
    return 0;
}

int sc7a20_acce_read(int *ps_x, int *ps_y, int *ps_z)
{
    return 0;
}



#define DRV_SC7A20_TEST

#ifdef DRV_SC7A20_TEST
#include <string.h>

extern void Triaxial_Sensor_Disable(void);
int sc7a20_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        LOG_I("Invalid parameter \n");
        return 1;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        int res = sc7a20_init();
        if (!res)
        {
            LOG_I("Open sc7a20 success\n");
        }
        else
        {
            LOG_I("Open sc7a20 fail\n");
        }
    }

    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t sc7a20_id = 0;
        sc7a20_read_reg(0x0F, &sc7a20_id, 1);
        LOG_I("sc7r30 chip id 0x%x", sc7a20_id);
    }
    if (strcmp(argv[1], "-close") == 0)
    {
        uint8_t sc7a20_id = 0;
        Triaxial_Sensor_Disable();
        LOG_I("sc7r30 chip disable");
    }


    return 0;
}


FINSH_FUNCTION_EXPORT_ALIAS(sc7a20_test, __cmd_sc7a, Test driver sc7a20);


#endif

#endif