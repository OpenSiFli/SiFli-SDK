#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"


/* i2c  */

static I2C_HandleTypeDef i2c_Handle = {0};

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

void RDAFM_write_data(uint8_t addr, uint16_t data)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[3];

    __HAL_I2C_ENABLE(&i2c_Handle);  // for master, enable it before transmit

    // 4. Write value to register
    buf[0] = addr;
    buf[1] = (uint8_t)(data >> 8);
    buf[2] = (uint8_t)(data & 0x00ff);

    ret = HAL_I2C_Master_Transmit(&i2c_Handle, RDA_I2C_ADDRESS, (uint8_t *)buf, 3, 1000);
    // check if read result same to write
    rt_kprintf("i2c write reg:0x%x,data:0x%x,ret:%d\n", addr, data, ret);

    __HAL_I2C_DISABLE(&i2c_Handle); // for master, disable it after transmit to reduce error status


}

void RDAFM_read_data(uint8_t addr, uint16_t *pdata)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    __HAL_I2C_ENABLE(&i2c_Handle);  // for master, enable it before transmit

    // 5. Read register value after write
    /* start 0x22,reg0,stop start 0x23,data0,data1 stop */
//    ret = HAL_I2C_Master_Transmit(&i2c_Handle, RDA_I2C_ADDRESS, &addr, 1, 1000);
//    ret = HAL_I2C_Master_Receive(&i2c_Handle, RDA_I2C_ADDRESS, (uint8_t *)pdata, 2, 1000);

    /* start 0x22,reg0, restart 0x23,data0,data1 stop */
    ret = HAL_I2C_Mem_Read(&i2c_Handle, RDA_I2C_ADDRESS, addr, 1, buf, 2, 1000);
    *pdata = (uint16_t)buf[0] << 8 | ((uint16_t)buf[1] & 0x00ff);

    rt_kprintf("i2c read reg:0x%x,pdata:0x%x,ret:%d\n", addr, *pdata, ret);

    __HAL_I2C_DISABLE(&i2c_Handle); // for master, disable it after transmit to reduce error status

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
    rt_kprintf("fm chipid:0x%x\n", gChipID);

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

    //----------------------------------------------
    // 2. i2c init

    i2c_Handle.Instance = EXAMPLE_I2C;
    i2c_Handle.Mode = HAL_I2C_MODE_MASTER; /* i2c master mode */
    i2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_Handle.Init.ClockSpeed = 400000; /* i2c speed (hz)*/
    i2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    ret = HAL_I2C_Init(&i2c_Handle);
    return ret;

}

/**
 * @brief RDAFM test
 * @author RDA Ri'an Zeng
 * @date 2008-11-05
 * @param void
 * @return bool:if true,the operation is successful;otherwise is failed
 * @retval
 */
void  RDAFM_test(void)
{
    RDAFM_init();
    RDAFM_power_on();
    RDAFM_set_freq(9390);   //test in 93.9MHz
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */
    rt_kprintf("Start i2c demo!\n");

    RDAFM_test();
    rt_kprintf("fm 94.2Mhz is %d\n", RDAFM_valid_stop(9420)); /* set the freqency to 94.2Mhz,and check this station if is a truely station*/

    rt_kprintf("i2c end!\n");
    while (1);
    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

