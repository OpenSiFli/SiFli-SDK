#include <rtthread.h>

#if defined(HR_USING_HRS3300)
#include "hx_hrs3300.h"
#include "hx_hrs3300_reg_init.h"
#include "drivers/pin.h"

// hrs3300 customer config
const uint8_t  hrs3300_bp_timeout_grade = 0;  // max 15
const uint8_t  hrs3300_agc_init_stage = 0x04;  // init AGC state
const uint8_t  hrs3300_bp_power_grade = 0;
const uint8_t  hrs3300_accurate_first_shot = 0;
const uint8_t  hrs3300_up_factor = 3;
const uint8_t  hrs3300_up_shift = 2;
const uint16_t hrs3300_AMP_LTH = 120;
const uint16_t hrs3300_hr_AMP_LTH = 150;
const uint16_t hrs3300_hr_PVAR_LTH = 10;
// hrs3300 customer config end

#define HRS3300_I2C_ADDR    0x44
#define HRS3300_ID          0x21

static struct rt_i2c_bus_device *hrs3300_i2cbus = NULL;

//20161117 added by ericy for "low power in no_touch state"
uint8_t reg_0x7f ;
uint8_t reg_0x80 ;
uint8_t reg_0x81 ;
uint8_t reg_0x82 ;
//20161117 added by ericy for "low power in no_touch state"

static uint8_t hrs3300_id = 0;
static bool hr_power_up = false ;

static rt_err_t hr_i2c_write(uint8_t *buf, uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = HRS3300_I2C_ADDR;    /* slave address */
    msgs.flags = RT_I2C_WR;        /* write flag */
    msgs.buf   = buf;              /* Send data pointer */
    msgs.len   = len;

    if (rt_i2c_transfer(hrs3300_i2cbus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_size_t hr_i2c_read(uint8_t *reg, uint16_t wlen, uint8_t *rbuf, uint16_t rlen)
{
    struct rt_i2c_msg msgs[2];
    rt_size_t res = 0;

    msgs[0].addr  = HRS3300_I2C_ADDR;    /* slave address */
    msgs[0].flags = RT_I2C_WR;        /* write flag */
    msgs[0].buf   = reg;              /* Send data pointer */
    msgs[0].len   = wlen;

    msgs[1].addr  = HRS3300_I2C_ADDR;    /* slave address */
    msgs[1].flags = RT_I2C_RD;        /* write flag */
    msgs[1].buf   = rbuf;              /* Send data pointer */
    msgs[1].len   = rlen;

    res = rt_i2c_transfer(hrs3300_i2cbus, msgs, 2);

    return res;
}

static bool Hrs3300_get_power_state(void)
{
    return hr_power_up;
}

static void Hrs3300_set_power_state(bool up)
{
    hr_power_up = up;
}

bool Hrs3300_write_reg(uint8_t addr, uint8_t data)
{
    uint8_t data_buf[2];
    data_buf[0] = addr;
    data_buf[1] = data;

    hr_i2c_write(data_buf, sizeof(data_buf));

    return 0;
}

uint8_t Hrs3300_read_reg(uint8_t addr)
{
    uint8_t data_buf;
    hr_i2c_read(&addr, 1, &data_buf, 1);
    return data_buf;
}

void Hrs3300_chip_enable(void)
{
    Hrs3300_write_reg(0x16, 0x78);
    Hrs3300_write_reg(0x01, 0xd0);
    Hrs3300_write_reg(0x0c, 0x2e);
    hr_power_up = true;
}

void Hrs3300_chip_disable(void)
{
    Hrs3300_write_reg(0x01, 0x08);
    Hrs3300_write_reg(0x02, 0x80);
    Hrs3300_write_reg(0x0c, 0x4e);
    hr_power_up = false;
}

uint16_t Hrs3300_read_hrs(void)
{
    uint8_t  databuf[3];
    uint16_t data;

    databuf[0] = Hrs3300_read_reg(0x09);    // addr09, bit
    databuf[1] = Hrs3300_read_reg(0x0a);    // addr0a, bit
    databuf[2] = Hrs3300_read_reg(0x0f);    // addr0f, bit

    data = ((databuf[0] << 8) | ((databuf[1] & 0x0F) << 4) | (databuf[2] & 0x0F));

    return data;
}

uint16_t Hrs3300_read_als(void)
{
    uint8_t  databuf[3];
    uint16_t data;

    databuf[0] = Hrs3300_read_reg(0x08);    // addr09, bit [10:3]
    databuf[1] = Hrs3300_read_reg(0x0d);    // addr0a, bit [17:11]
    databuf[2] = Hrs3300_read_reg(0x0e);    // addr0f, bit [2:0]

    data = ((databuf[0] << 3) | ((databuf[1] & 0x3F) << 11) | (databuf[2] & 0x07));

    if (data > 32767) data = 32767;  // prevent overflow of other function

    return data;
}

uint32_t Hrs3300_get_device_handler(void)
{
    return (uint32_t)hrs3300_i2cbus;
}

uint8_t Hrs3300_get_dev_addr(void)
{
    return (uint8_t)HRS3300_I2C_ADDR;
}

uint8_t Hrs3300_get_dev_id(void)
{
    return hrs3300_id;
}

static void Hrs3300_close_bloodpress(void)
{

    //stop_bp_timer();
    //User_Health_para_init();
    //Set_Sample_Signal(ENUM_SAMPLE_IDEL);
}

static rt_err_t Hrs3300_chip_init(void)
{
    int i = 0;
    uint8_t id = 0;

    for (i = 0; i < INIT_ARRAY_SIZE; i++)
    {
        if (Hrs3300_write_reg(init_register_array[i][0], init_register_array[i][1]) != 0)
        {
            goto ret;
        }
    }

    //20161117 added by ericy for "low power in no_touch state"
    if (!hr_power_up)
    {
        reg_0x7f = Hrs3300_read_reg(0x7f) ;
        reg_0x80 = Hrs3300_read_reg(0x80) ;
        reg_0x81 = Hrs3300_read_reg(0x81) ;
        reg_0x82 = Hrs3300_read_reg(0x82) ;
        hr_power_up = true;
    }
    //20161117 added by ericy for "low power in no_touch state"

    id = Hrs3300_read_reg(0x0);

    rt_kprintf("<<< hrs3300 init done id = %d \n", id); // 0x21
    if (id != HRS3300_ID) goto ret;

    hrs3300_id = id;
    Hrs3300_chip_disable();

    return RT_EOK;

ret:
    return RT_ERROR;
}

rt_err_t hr_hw_init(void)
{
    hrs3300_i2cbus = (struct rt_i2c_bus_device *)rt_device_find(HRS3300_I2C_BUS);

    if (RT_Device_Class_I2CBUS != hrs3300_i2cbus->parent.type)
    {
        hrs3300_i2cbus = NULL;
    }

    if (hrs3300_i2cbus)
    {
        rt_device_open((rt_device_t)hrs3300_i2cbus, RT_DEVICE_FLAG_RDWR);
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

        rt_i2c_configure(hrs3300_i2cbus, &configuration);
    }

    if (RT_EOK != Hrs3300_chip_init())
    {
        rt_kprintf("hr_hw_init fail!\n");
        hr_hw_deinit();
        return RT_ERROR;
    }

    rt_kprintf("hr_hw_init ok\n");

    return RT_EOK;
}

void hr_hw_deinit(void)
{
    if (hrs3300_i2cbus)
    {
        rt_device_close((rt_device_t)hrs3300_i2cbus);
        hrs3300_i2cbus = NULL;
    }
}

void hr_hw_power_onoff(bool     onoff)
{
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
}

rt_err_t hr_hw_self_check(void)
{
    struct rt_device_pin_status st;

    // check gpio
    if (rt_pin_read(HRS3300_POW_PIN) == 0)
        return RT_ERROR;

    return RT_EOK;
}


#endif //
