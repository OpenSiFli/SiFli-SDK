#include "mmc56x3.h"

#include <rtthread.h>
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.mag"
#include <drv_log.h>

static struct rt_i2c_bus_device * MMC56x3_bus;

static int MMC56x3_I2C_Init(const char *name)
{
    /* get i2c bus device */
    MMC56x3_bus = rt_i2c_bus_device_find(name);
    if (MMC56x3_bus)
    {
        LOG_D("Find i2c bus device %s\n", name);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", name);
        return -1;
    }

    return 0;
}

uint8_t MMC56x3_ReadID(void)
{
    uint8_t id[1];
    RT_ASSERT(rt_i2c_mem_read(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_PRODUCT_ID, 8, id, 1) > 0);
    return id[0];
}

/**
 * @brief Sets whether we are in continuous read mode (1) or one-shot (0)
 * 
 * @param mode 
 */
void MMC56x3_SetContinuousMode(uint8_t mode)
{
    uint8_t temp[1];
    temp[0] = 0x08;
    RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL0_REG, 8, temp, 1) > 0);
    RT_ASSERT(rt_i2c_mem_read(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL2_REG, 8, temp, 1) > 0);
    if (mode)
    {
        temp[0] |= 0x10;
    }
    else
    {
        temp[0] &= ~0x10;
    }
    RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL2_REG, 8, temp, 1) > 0);
}

void MMC56x3_Reset(void)
{
    uint8_t temp[1];
    temp[0] = 0x80;
    RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL1_REG, 8, temp, 1) > 0);
    rt_thread_mdelay(20);
    temp[0] = 0x08;
    RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL0_REG, 8, temp, 1) > 0);
    rt_thread_mdelay(1);
    temp[0] = 0x10;
    RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL0_REG, 8, temp, 1) > 0);
    rt_thread_mdelay(1);
    MMC56x3_SetContinuousMode(0); // default to continuous mode
}

void MMC56x3_SetDataRate(uint16_t rate)
{
    if (rate > 255)
    {
        rate = 1000;
    }
    uint8_t temp[1];
    if (rate == 1000)
    {
        temp[0] = 255;
        RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC5603_ODR_REG, 8, temp, 1) > 0);
        RT_ASSERT(rt_i2c_mem_read(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL2_REG, 8, temp, 1) > 0);
        temp[0] |= 0x80;
        RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL2_REG, 8, temp, 1) > 0);
    }
    else
    {
        temp[0] = rate;
        RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC5603_ODR_REG, 8, temp, 1) > 0);
        RT_ASSERT(rt_i2c_mem_read(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL2_REG, 8, temp, 1) > 0);
        temp[0] &= ~0x80;
        RT_ASSERT(rt_i2c_mem_write(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_CTRL2_REG, 8, temp, 1) > 0);
    }
}

rt_err_t MMC56x3_Init(struct rt_sensor_config *cfg)
{
    if (MMC56x3_I2C_Init(cfg->intf.dev_name) != 0)
    {
        return -RT_ERROR;
    }

    uint8_t id = MMC56x3_ReadID();
    rt_kprintf("MMC56x3 ID = %d\n", id);
    if (id != MMC56X3_CHIP_ID)
    {
        LOG_E("MMC56x3 ID error\n");
        return -RT_ERROR;
    }
    MMC56x3_Reset();
    MMC56x3_SetDataRate(100);
    MMC56x3_SetContinuousMode(1);
    return RT_EOK;
}

mmc56x3_data_t MMC56x3_ReadData(void)
{
    mmc56x3_data_t data = {0};
    uint8_t buffer[8];
    RT_ASSERT(rt_i2c_mem_read(MMC56x3_bus, MMC56X3_DEFAULT_ADDRESS, MMC56X3_OUT_X_L, 8, buffer, 8) > 0);
    int32_t x,y,z;
    x = (uint32_t)buffer[0] << 12 | (uint32_t)buffer[1] << 4 |
        (uint32_t)buffer[6] >> 4;
    y = (uint32_t)buffer[2] << 12 | (uint32_t)buffer[3] << 4 |
        (uint32_t)buffer[7] >> 4;
    z = (uint32_t)buffer[4] << 12 | (uint32_t)buffer[5] << 4 |
        (uint32_t)buffer[8] >> 4;
    // fix center offsets
    x -= (uint32_t)1 << 19;
    y -= (uint32_t)1 << 19;
    z -= (uint32_t)1 << 19;

    data.x = x * 0.0625;
    data.y = y * 0.0625;
    data.z = z * 0.0625;
    return data;
}