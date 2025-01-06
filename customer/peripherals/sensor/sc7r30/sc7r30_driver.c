#include <rtthread.h>
#include "board.h"
#include "sc7r30.h"
#include "sc7r30_driver.h"
//#include "SL_SC7R30_BPM_Algo_Driver.h"
#include "SL_SC7R30_BPM_Application.h"


#define LOG_TAG              "drv.7r30"
#define DBG_LEVEL            DBG_INFO
#include <drv_log.h>

#define SC7R30_CHIP_ID              (0x71)

static uint8_t hr_value;



unsigned char SL_SC7R30_I2c_Spi_Write(unsigned char reg, unsigned char data)
{
    sc7r30_write_reg(reg, data);
    return 0;
}

unsigned char SL_SC7R30_I2c_Spi_Read(unsigned char reg, unsigned char len, unsigned char *buf)
{
    sc7r30_read_reg(reg, buf, len);
    return 0;
}



uint8_t HeartRat_ReadData(void)
{
#if defined(__CC_ARM) || defined(__CLANG_ARM)
    hr_value = SL_SC7R30_BPM_ALGO();
#endif
    LOG_D("hr_value=%d\n", hr_value);

    return hr_value;
}


static int HeartRate_Sensor_CheckID(void)
{
    uint8_t id = 0;

    SL_SC7R30_I2c_Spi_Read(0x0f, 1, &id);
    if (id == SC7R30_CHIP_ID)
    {
        LOG_I("sc7r30_id = [0x%x]\n", id);
        return 0;
    }
    else
    {
        return -1;
    }

}


int HeartRate_Init(void)
{
#if defined(__CC_ARM) || defined(__CLANG_ARM)
    int ret = 0;

    ret = sc7r30_init();
    if (ret < 0)
    {
        return ret;
    }
    ret = HeartRate_Sensor_CheckID();
    if (ret == 0)
    {
        if (SL_SC7R30_BPM_INIT() == 0x01)
        {
            LOG_I("Heart Rate Sensor OK!!\n");
        }
        else
        {
            LOG_I("Heart Rate Sensor ERROR!!   ret = %d\n", ret);
            return -1;
        }
    }
    else
    {

        LOG_I("Heart Rate Sensor ID ERROR!!\n");
        return -1;
    }
#endif
    return 0;
}


