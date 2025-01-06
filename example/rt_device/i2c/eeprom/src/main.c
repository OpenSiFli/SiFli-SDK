#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

#define DBG_TAG "eeprom_i2c"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* i2c  */

static struct rt_i2c_bus_device *i2c_bus = NULL;

/* i2c */
//Version 5.0

#define EEPROM_I2C_ADDRESS         0x50  // 7bit device address of EEPROM

/***************************************************
EEPROM interfaces
****************************************************/

uint8_t RECEIVED = 0;
uint8_t TEST_ADDR[] =
{
    0x01,
    0x02,
    0x03,
    0x04
};
uint8_t TEST_DATA[] =
{
    0x11,
    0x22,
    0x33,
    0x44
};


/**
 * @brief Initialization work before power on EEPROM and
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return bool:if true,the operation is successful;otherwise is failed
 * @retval
 */
unsigned char EEPROM_init(void)
{

    uint8_t slaveAddr = EEPROM_I2C_ADDRESS; // 7bit address of device
    HAL_StatusTypeDef ret;

    //1. pin mux
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA41, I2C2_SCL, PIN_PULLUP, 1); // i2c io select
    HAL_PIN_Set(PAD_PA42, I2C2_SDA, PIN_PULLUP, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PB28, I2C6_SCL, PIN_PULLUP, 0); // i2c io select
    HAL_PIN_Set(PAD_PB29, I2C6_SDA, PIN_PULLUP, 0);
#endif

    // 2. i2c init
    // find i2c2
#ifdef SF32LB52X
    i2c_bus = rt_i2c_bus_device_find("i2c2");
#elif defined(SF32LB58X)
    i2c_bus = rt_i2c_bus_device_find("i2c6");
#endif

    rt_kprintf("i2c_bus:0x%x\n", i2c_bus);
    if (i2c_bus)
    {
#ifdef SF32LB52X
        rt_kprintf("Find i2c bus device I2C2\n");
#elif defined(SF32LB58X)
        rt_kprintf("Find i2c bus device I2C6\n");
#endif
        // open rt_device i2c2
        rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_FLAG_RDWR);
        //rt_i2c_open(i2c_bus, RT_DEVICE_FLAG_RDWR);
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 500, //Waiting for timeout period (ms)
            .max_hz = 400000, //I2C rate (hz)
        };
        // config I2C parameter
        rt_i2c_configure(i2c_bus, &configuration);
    }
    else
    {
#ifdef SF32LB52X
        LOG_E("Can not found i2c bus I2C2, init fail\n");
#elif defined(SF32LB58X)
        LOG_E("Can not found i2c bus I2C6, init fail\n");
#endif
        return -1;
    }
    return 0;
}

void EEPROM_write_data(uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef ret;
    ret = rt_i2c_mem_write(i2c_bus, EEPROM_I2C_ADDRESS, addr, 8, &data, 1);
    LOG_D("i2c write reg:0x%x,data:0x%x,ret:%d\n", addr, data, ret);
}




void EEPROM_read_data(uint8_t addr, uint8_t *pdata)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    ret = rt_i2c_mem_read(i2c_bus, EEPROM_I2C_ADDRESS, addr, 8, pdata, 1);  // ret: return data size
    LOG_D("i2c read reg:0x%x,pdata:0x%x,ret:%d\n", addr, *pdata, ret);
}

void delayms(unsigned short int ms)
{
    HAL_Delay(ms);
}


/// @brief read and write eeprom to test
/// @param
void EEPROM_test(void)
{
    unsigned char i = 0;

    for (i = 0; i < 4; i++)
    {
        EEPROM_write_data(TEST_ADDR[i], TEST_DATA[i]);
        delayms(5); //5ms delay for AT240C8SC write time cycle
    }

    for (i = 0; i < 4; i++)
    {
        EEPROM_read_data(TEST_ADDR[i], &RECEIVED);
    }
}




void  EEPROM_example(void)
{
    EEPROM_init();
    EEPROM_test();
}

uint8_t g_main_flag = 0;
/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("main!!\n");
    LOG_D("Start i2c eeprom rtt demo!\n"); // Output a start message on console using printf function
    EEPROM_example();
    LOG_D("i2c end!\n"); // Output a end message on console using printf function

    while (1)
    {
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }

    return RT_EOK;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

