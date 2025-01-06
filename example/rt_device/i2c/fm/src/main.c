#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

#define DBG_TAG "fm_i2c"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* i2c  */

static struct rt_i2c_bus_device *i2c_bus = NULL;

/* RDA FM drv */
//Version 5.0

#define RDA_I2C_ADDRESS         0x11  /* read 0x22, write 0x23*/

/***************************************************
RDAFM interfaces
****************************************************/

unsigned short int gChipID = 0;
unsigned short int RDAFM_REGW[8] = {0};
unsigned short int RDA580XN_init_reg[] =
{
    0x0002,
    0xC001,
    0xC001,
    0x0000,
    0x0400,
    0x86ef, //05h
};

/**
* @brief: the high byte and low byte swap each other
* @date: 2024-10-15
* @param: input:
* @return: the value that swaped
*/
uint16_t swap_u16(uint16_t input)
{
    return (uint16_t)(((input << 8) & 0xff00) | ((input >> 8) & 0xff));
}

void RDAFM_write_data(uint8_t addr, uint16_t data)
{
    HAL_StatusTypeDef ret;
    uint16_t reg_data = swap_u16(data);
    ret = rt_i2c_mem_write(i2c_bus, RDA_I2C_ADDRESS, addr, 8, &reg_data, 2);  /* not I2C_MEMADD_SIZE_16BIT !!!  */
    LOG_D("i2c write reg:0x%x,data:0x%x,ret:%d\n", addr, data, ret);
}

void RDAFM_read_data(uint8_t addr, uint16_t *pdata)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    ret = rt_i2c_mem_read(i2c_bus, RDA_I2C_ADDRESS, addr, 8, pdata, 2);  /* not I2C_MEMADD_SIZE_16BIT !!!  */
    *pdata = swap_u16(*pdata);
    LOG_D("i2c read reg:0x%x,pdata:0x%x,ret:%d\n", addr, *pdata, ret);
}

void delayms(unsigned short int ms)
{
    HAL_Delay(ms);
}

/**
 * @brief: power on RDAFM
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return void
 * @retval
 */
void RDAFM_power_on(void)
{
    unsigned char i = 0;

    RDAFM_write_data(0x02, RDAFM_REGW[0]);
    delayms(50);

    RDAFM_read_data(0x0E, &gChipID);
    delayms(10);
    LOG_D("fm chipid:0x%x\n", gChipID);

    for (i = 0; i < 8; i++)
        RDAFM_REGW[i] = RDA580XN_init_reg[i];

    RDAFM_write_data(0x02, RDA580XN_init_reg[2]);

    delayms(600);

    for (i = 0x02; i < (sizeof(RDA580XN_init_reg) / sizeof(RDA580XN_init_reg[0])); i++)
        RDAFM_write_data(i, RDA580XN_init_reg[i]);

    delayms(50);         //dealy 50 ms
}

/**
 * @brief RDAFM power off function
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return void
 * @retval
 */
void  RDAFM_power_0ff(void)
{
    RDAFM_REGW[2] &= (~1);
    RDAFM_write_data(0x02, RDAFM_REGW[2]);
}

/**
 * @brief Set RDAFM into mute mode
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param bool mute: if mute is true,then set mute; if mute is false,then set no mute
 * @return void
 * @retval
 */
void RDAFM_set_mute(unsigned char mute)
{
    if (mute)
        RDAFM_REGW[2] &= ~(1 << 14);
    else
        RDAFM_REGW[2] |= (1 << 14);

    RDAFM_write_data(0x02, RDAFM_REGW[2]);
    delayms(50);    //dealy 50 ms
}

/**
 * @brief Cover the frequency to channel value
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param unsigned short int frequency: frequency  value
 *        frequency unit: 10KHz
 * @return unsigned short int: channel value
 * @retval
 */
unsigned short int RDAFM_freq_to_chan(unsigned short int frequency)
{
    unsigned char channelSpacing;
    unsigned short int bottomOfBand;
    unsigned short int channel;

    if ((RDAFM_REGW[3] & 0x000C) == 0x0000)
        bottomOfBand = 8700;
    else if ((RDAFM_REGW[3] & 0x000C) == 0x0004)
        bottomOfBand = 7600;
    else if ((RDAFM_REGW[3] & 0x000C) == 0x0008)
        bottomOfBand = 7600;
    if ((RDAFM_REGW[3] & 0x0003) == 0x0000)
        channelSpacing = 10;
    else if ((RDAFM_REGW[3] & 0x0003) == 0x0001)
        channelSpacing = 20;
    else if ((RDAFM_REGW[3] & 0x0003) == 0x0002)
        channelSpacing = 5;

    channel = (frequency - bottomOfBand) / channelSpacing;

    return (channel);
}

/**
 * @brief Cover the channel to frequency value
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param unsigned short int chan: channel value
 * @return unsigned short int: frequency value
 *        frequency unit: 10KHz
 * @retval
 */
unsigned short int RDAFM_chan_to_freq(unsigned short int chan)
{
    unsigned char channelSpacing;
    unsigned short int bottomOfBand;
    unsigned short int freq;

    if ((RDAFM_REGW[3] & 0x000C) == 0x0000)
        bottomOfBand = 8700;
    else if ((RDAFM_REGW[3] & 0x000C) == 0x0004)
        bottomOfBand = 7600;
    else if ((RDAFM_REGW[3] & 0x000C) == 0x0008)
        bottomOfBand = 7600;
    if ((RDAFM_REGW[3] & 0x0003) == 0x0000)
        channelSpacing = 10;
    else if ((RDAFM_REGW[3] & 0x0003) == 0x0001)
        channelSpacing = 20;
    else if ((RDAFM_REGW[3] & 0x0003) == 0x0002)
        channelSpacing = 5;

    freq = bottomOfBand + chan * channelSpacing;

    return (freq);
}

/**
 * @brief Set frequency function
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param unsigned short int curFreq:frequency value
 *        frequency unit: 10KHz
 * @return void
 * @retval
 */
void RDAFM_set_freq(unsigned short int curFreq)
{
    unsigned short int curChan;
    unsigned short int tReg = 0;

    curChan = RDAFM_freq_to_chan(curFreq);
    RDAFM_REGW[3] = ((curChan << 6) | 0x10 | (RDAFM_REGW[3] & 0x0f));
    RDAFM_write_data(0x03, RDAFM_REGW[3]);

    delayms(50);     //delay 50 ms
}

/**
 * @brief Station judge for auto search
 * @In auto search mode,uses this function to judge the frequency if has a station
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param unsigned short int freq:frequency value
 *        frequency unit: 10KHz
 * @return bool: if return true,the frequency has a true station;otherwise doesn't have a station
 * @retval
 */
int RDAFM_valid_stop(unsigned short int freq)
{
    unsigned short int tReg = 0;

    RDAFM_set_freq(freq);

    RDAFM_read_data(0x0B, &tReg);

    //check FM_TURE in 0B register
    if ((tReg & 0x0100) == 0)
    {
        return 0;
    }
    else
    {
        /*
            if(freq==960)
            {
                return 0;
            }
        */
        return 1;
    }
}

/**
 * @brief Get the RSSI of the current frequency
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return unsigned char: the RSSI
 * @retval
 */
unsigned char RDAFM_get_rssi(void)
{
    unsigned short int tReg = 0;

    RDAFM_read_data(0x0B, &tReg);
    delayms(50);    //dealy 50 ms

    return (tReg >> 9); /*return rssi*/
}

/**
 * @brief Set FM volume
 * @It has better use the system volume operation to replace this function
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param unsigned char level: volume value
 * @return void
 * @retval
 */
void RDAFM_set_volume(unsigned char level)
{
    RDAFM_REGW[5] = ((RDAFM_REGW[5] & 0xfff0) | (level & 0x0f));

    RDAFM_write_data(0x05, RDAFM_REGW[5]);
    delayms(50);    //Dealy 50 ms
}

/**
 * @brief Initialization work before power on RDAFM and
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return bool:if true,the operation is successful;otherwise is failed
 * @retval
 */
unsigned char RDAFM_init(void)
{

    uint8_t slaveAddr = RDA_I2C_ADDRESS; /* 7bit address of device*/
    HAL_StatusTypeDef ret;

    //----------------------------------------------
    //1. pin mux
    HAL_PIN_Set(PAD_PA40, I2C2_SCL, PIN_PULLUP, 1); /* i2c io select */
    HAL_PIN_Set(PAD_PA39, I2C2_SDA, PIN_PULLUP, 1);

    //----------------------------------------------
    // 2. i2c init

    /* find i2c2 */
    i2c_bus = rt_i2c_bus_device_find("i2c2");
    rt_kprintf("i2c_bus:0x%x\n", i2c_bus);
    if (i2c_bus)
    {
        rt_kprintf("Find i2c bus device I2C2\n");
        /* open rt_device i2c2 */
        rt_device_open((rt_device_t)i2c_bus, RT_DEVICE_FLAG_RDWR);
        //rt_i2c_open(i2c_bus, RT_DEVICE_FLAG_RDWR);
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 500, //Waiting for timeout period (ms)
            .max_hz = 400000, //I2C rate (hz)
        };
        /* config I2C parameter */
        rt_i2c_configure(i2c_bus, &configuration);
    }
    else
    {
        LOG_E("Can not found i2c bus I2C2, init fail\n");
        return -1;
    }
    return 0;

}

/**
 * @brief RDAFM test
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return bool:if true,the operation is successful;otherwise is failed
 * @retval
 */
int  RDAFM_test(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */
    LOG_D("Start i2c demo!\n");
    RDAFM_init();
    RDAFM_power_on();
    RDAFM_set_freq(9390);   //test in 93.9MHz
    LOG_D("fm 94.2Mhz is %d\n", RDAFM_valid_stop(9420)); /* set the freqency to 94.2Mhz,and check this station if is a truely station*/

    LOG_D("i2c end!\n");
    return HAL_OK;
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
    RDAFM_test(); /* FM I2C TEST*/
    while (1)
    {
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }

    return RT_EOK;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

