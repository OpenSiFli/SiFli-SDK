#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"


void delayms(unsigned short int ms)
{
    HAL_Delay(ms);
}

/* i2c  */

static I2C_HandleTypeDef i2c_Handle = {0};


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



/// @brief Initialization work before power on EEPROM
/// @param
void EEPROM_init(void)
{
    uint8_t slaveAddr = EEPROM_I2C_ADDRESS; // 7bit address of device
    HAL_StatusTypeDef ret;
    //1. pin mux
#ifdef SF32LB52X
    HAL_RCC_EnableModule(RCC_MOD_I2C2); // enable i2c2
#define EXAMPLE_I2C I2C2 // i2c number of cpu
#define EXAMPLE_I2C_IRQ I2C2_IRQn // i2c number of interruput when using interrupte mode 
    HAL_PIN_Set(PAD_PA41, I2C2_SCL, PIN_PULLUP, 1); // i2c io select
    HAL_PIN_Set(PAD_PA42, I2C2_SDA, PIN_PULLUP, 1);
#elif defined(SF32LB58X)
#define EXAMPLE_I2C I2C6 // i2c number of cpu
#define EXAMPLE_I2C_IRQ I2C6_IRQn // i2c number of interruput when using interrupte mode 
    HAL_PIN_Set(PAD_PB28, I2C6_SCL, PIN_PULLUP, 0); // i2c io select
    HAL_PIN_Set(PAD_PB29, I2C6_SDA, PIN_PULLUP, 0);
#endif

    // 2. i2c init
    i2c_Handle.Instance = EXAMPLE_I2C;
    i2c_Handle.Mode = HAL_I2C_MODE_MASTER; // i2c master mode
    i2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // i2c 7bits device address mode
    i2c_Handle.Init.ClockSpeed = 400000; // i2c speed (hz)
    i2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    ret = HAL_I2C_Init(&i2c_Handle);
    rt_kprintf("EEPROM_init%d\n", ret);
}


/// @brief write data to eeprom
/// @param addr data address
/// @param data data write to eeprom
void EEPROM_write_data(uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    __HAL_I2C_ENABLE(&i2c_Handle);  // for master, enable it before transmit

    // 3. write value to register
    buf[0] = addr; // memory address to write
    buf[1] = data; // data to write

    ret = HAL_I2C_Master_Transmit(&i2c_Handle, EEPROM_I2C_ADDRESS, buf, 2, 1000);

    rt_kprintf("i2c write addr:0x%x,data:0x%x,ret:%d\n", addr, data, ret);
    if (ret != 0)rt_kprintf("EEPROM_write_data ErrorCode:0x%x\n", i2c_Handle.ErrorCode);

    i2c_Handle.ErrorCode = 0;

    __HAL_I2C_DISABLE(&i2c_Handle); // for master, disable it after transmit to reduce error status
}


void EEPROM_read_data(uint8_t addr, uint8_t *pdata)
{
    HAL_StatusTypeDef ret;
    uint8_t buf = 0;

    __HAL_I2C_ENABLE(&i2c_Handle);  // for master, enable it before transmit

    // 5. Read register value after write

    // device need stop condition before read
    // ret = HAL_I2C_Master_Transmit(&i2c_Handle, EEPROM_I2C_ADDRESS, &addr, 1, 1000);
    // ret = HAL_I2C_Master_Receive(&i2c_Handle, EEPROM_I2C_ADDRESS, (uint8_t *)pdata, 2, 1000);

    // device can read without stop condition
    ret = HAL_I2C_Mem_Read(&i2c_Handle, EEPROM_I2C_ADDRESS, addr, 1, &buf, 1, 1000);
    *pdata = buf;

    rt_kprintf("i2c read reg:0x%x,pdata:0x%x,ret:%d\n", addr, *pdata, ret);

    __HAL_I2C_DISABLE(&i2c_Handle); // for master, disable it after transmit to reduce error status
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


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Start i2c eeprom demo!\n"); // Output a start message on console using printf function
    EEPROM_example();
    rt_kprintf("i2c eeprom end!\n"); // Output a end message on console using printf function
    while (1)
        return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

