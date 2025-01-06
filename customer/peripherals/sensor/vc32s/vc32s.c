#include <rtthread.h>
#if defined(HR_USING_VC32S)
#include "vc32s.h"
#include "vcHr02Hci.h"
#include "drivers/pin.h"
#include "bf0_hal_aon.h"
#ifdef PMIC_CONTROL_SERVICE
    #include "pmic_service.h"
#endif

#define VC32S_SLAVE_ADDRESS    0x33
#define VC32S_ID                0x21


static struct rt_i2c_bus_device *vc32s_i2cbus = NULL;
static uint8_t vc32s_id = 0;

static rt_err_t vc32s_i2c_read(unsigned char devAddr,
                               unsigned char regAddr,
                               unsigned char len,
                               unsigned char *data)
{
    rt_int8_t res = 0;
    rt_uint8_t buf[2];
    buf[0] = regAddr;  //cmd
    struct rt_i2c_msg msgs[2];
    msgs[0].addr  = VC32S_SLAVE_ADDRESS;    /* Slave address */
    msgs[0].flags = RT_I2C_WR;              /* Read flag */
    msgs[0].buf   = buf;                    /* Read data pointer */
    msgs[0].len   = 1;                      /* Number of bytes read */

    msgs[1].addr  = VC32S_SLAVE_ADDRESS;    /* Slave address */
    msgs[1].flags = RT_I2C_RD;              /* Read flag */
    msgs[1].buf   = data;                   /* Read data pointer */
    msgs[1].len   = len;                    /* Number of bytes read */

    //rt_kprintf("vc32s_i2c_read \n");

    if (rt_i2c_transfer(vc32s_i2cbus, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_err_t vc32_i2c_base_write(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = VC32S_SLAVE_ADDRESS;    /* slave address */
    msgs.flags = RT_I2C_WR;              /* write flag */
    msgs.buf   = buf;                    /* Send data pointer */
    msgs.len   = len;

    if (rt_i2c_transfer(vc32s_i2cbus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static unsigned char vc32s_i2c_write(unsigned char devAddr,
                                     unsigned char regAddr,
                                     unsigned char len,
                                     unsigned char *data)
{
    rt_uint8_t buf[32];

    buf[0] = regAddr;   //cmd
    rt_memcpy((buf + 1), data, len);

    /* rt_hw_us_delay(20); */

    if (RT_EOK == vc32_i2c_base_write(buf, (len + 1)))
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

/*
 * @brief  Register writing.
 *
 *
 **/
vcHr02Ret_t vcHr02WriteRegisters(uint8_t startAddress, uint8_t *p_data, uint8_t len)
{
    vc32s_i2c_write(VC32S_SLAVE_ADDRESS, startAddress, len, p_data);
    return VCHR02RET_ISOK;
}


/*
 * @brief  Register reading.
 *
 *
 **/
vcHr02Ret_t vcHr02ReadRegisters(uint8_t startAddress, uint8_t *p_data, uint8_t len)
{

    uint32_t res;
    res = vc32s_i2c_read(VC32S_SLAVE_ADDRESS, startAddress, len, p_data);
    return res;
}

static rt_err_t vc32s_init(void)
{
    uint8_t chip_id = 0;

    vcHr02ReadRegisters(0x00, &chip_id, 1);
    rt_kprintf("hr_vc32s_init chip_id =0x%x(should be 0x21)\n", chip_id);

    if (chip_id == VC32S_ID)
    {
        vc32s_id = chip_id;
        return RT_EOK;
    }

    return RT_ERROR;
}

rt_err_t hr_hw_init(void)
{
    vc32s_i2cbus = (struct rt_i2c_bus_device *)rt_device_find(VC32S_I2C_BUS);
    if (RT_Device_Class_I2CBUS != vc32s_i2cbus->parent.type)
    {
        vc32s_i2cbus = NULL;
    }

    if (vc32s_i2cbus)
    {
        rt_device_open((rt_device_t)vc32s_i2cbus, RT_DEVICE_FLAG_RDWR);
    }
    else
    {
        rt_kprintf("bus not find\n");
        return RT_ERROR;
    }

    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 1000,
            .max_hz = 200000,
        };

        rt_i2c_configure(vc32s_i2cbus, &configuration);
    }

    if (RT_EOK != vc32s_init())
    {
        hr_hw_deinit();
        return RT_ERROR;
    }

    rt_kprintf("hr_hw_init ok\n");

    return RT_EOK;
}

void hr_hw_deinit(void)
{
    if (vc32s_i2cbus)
    {
        rt_device_close((rt_device_t)vc32s_i2cbus);
        vc32s_i2cbus = NULL;
    }
}

void hr_hw_power_onoff(bool     onoff)
{
#if (VC32S_POW_PIN >= 0)
    if (onoff)
    {
        rt_pin_mode(HRS3300_POW_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(HRS3300_POW_PIN, PIN_HIGH);
        rt_kprintf("hr_power_on\n");
    }
    else
    {
        rt_pin_mode(HRS3300_POW_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(HRS3300_POW_PIN, PIN_LOW);
        rt_kprintf("hr_power_off\n");
    }
#endif
}

rt_err_t hr_hw_self_check(void)
{
    return RT_EOK;
}

uint32_t vc32s_get_i2c_handle(void)
{
    return (uint32_t)vc32s_i2cbus;
}


uint8_t vc32s_get_dev_addr(void)
{
    return (uint8_t)VC32S_SLAVE_ADDRESS;
}

uint8_t vc32s_get_dev_id(void)
{
    return vc32s_id;
}
#endif //
