#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

#define DBG_TAG "spi1"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* spi example for RT-Thread based platform -----------------------------------------------*/
#include "drv_spi.h"

#define NOR_FLASH_DEVICE_NAME     "nor_flash"

#define SPI_BUS_NAME "spi1"

static struct rt_spi_device *spi_dev_handle = {0};

static struct rt_spi_configuration spi_dev_cfg = {0};

rt_err_t spi_dev_init(void)
{
    rt_err_t rst = RT_EOK;
    /* 1, pinmux set to spi1 mode */
#ifdef  SF32LB52X
    HAL_PIN_Set(PAD_PA24, SPI1_DIO, PIN_PULLDOWN, 1);       // SPI1 (Nor flash)
    HAL_PIN_Set(PAD_PA25, SPI1_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);

#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PA21, SPI1_DO, PIN_PULLDOWN, 1);       // SPI1 (Nor flash)
    HAL_PIN_Set(PAD_PA20, SPI1_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);
#endif
    /* 2, find/open/config spi1 device  */
    rt_device_t spi_bus = rt_device_find(SPI_BUS_NAME);

    if (spi_bus)
    {
        rt_device_open(spi_bus, RT_DEVICE_FLAG_RDWR);
        LOG_D("Find spi bus %s:%x\n", SPI_BUS_NAME, spi_bus);
        spi_dev_handle = (struct rt_spi_device *)rt_device_find(NOR_FLASH_DEVICE_NAME);
        if (spi_dev_handle == NULL)
        {
            rst = rt_hw_spi_device_attach(SPI_BUS_NAME, NOR_FLASH_DEVICE_NAME);
            spi_dev_handle = (struct rt_spi_device *)rt_device_find(NOR_FLASH_DEVICE_NAME);
            if (spi_dev_handle == NULL)
            {
                LOG_E("Register spi_dev spi device fail\n");
                return -RT_ERROR;
            }
        }
        rst = rt_device_open((rt_device_t)(spi_dev_handle), RT_DEVICE_FLAG_RDWR);//|RT_DEVICE_FLAG_DMA_TX);

        spi_dev_cfg.data_width = 8; //bit
        spi_dev_cfg.max_hz = 20 * 1000 * 1000; // hz
        spi_dev_cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER;
        spi_dev_cfg.frameMode = RT_SPI_MOTO; //RT_SPI_TI;
        rst = rt_spi_configure(spi_dev_handle, &spi_dev_cfg);
        LOG_D("rt_spi_configure result:%d", rst);
    }

    /* rt_pin_mode(spi_dev_CS_PIN, PIN_MODE_OUTPUT); */
    /* rt_pin_write(spi_dev_CS_PIN, PIN_HIGH); */

    return rst;
}

int32_t spidev_register_write(rt_uint8_t *reg_addr, rt_uint8_t reg_length, rt_uint8_t *value, rt_uint8_t size)
{
    LOG_D("spidev_register_write addr:0x%x value:0x%x", *reg_addr, *value);
    rt_err_t ret;
    /* rt_pin_write(spi_dev_CS_PIN, PIN_LOW); */
    rt_spi_send_then_send(spi_dev_handle, reg_addr, reg_length, value, size);
    /* rt_pin_write(spi_dev_CS_PIN, PIN_HIGH); */

    return RT_EOK;
}
int32_t spidev_write(rt_uint8_t *reg_addr, rt_uint8_t reg_length)
{

    /* type1: use rt_spi_send() */
    return rt_spi_send((struct rt_spi_device *)spi_dev_handle, reg_addr, reg_length);

    /* type2: use rt_spi_transfer_message() */
    struct rt_spi_message msg1, msg2;

    msg1.send_buf   = reg_addr;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = reg_length;
    msg1.cs_take    = 1;
    msg1.cs_release = 1;
    msg1.next       = RT_NULL;

    rt_spi_transfer_message((struct rt_spi_device *)spi_dev_handle, &msg1);
    return RT_EOK;
}

int32_t spidev_register_read(rt_uint8_t *reg_addr, rt_uint8_t reg_length, rt_uint8_t *value, rt_uint8_t size)
{
    rt_err_t ret;
    /* rt_pin_write(spi_dev_CS_PIN, PIN_LOW); */
    rt_spi_send_then_recv(spi_dev_handle, reg_addr, reg_length, value, size);
    /* rt_pin_write(spi_dev_CS_PIN, PIN_HIGH); */
    LOG_D("rt_spi_transfer recv reg:%x value:%x", *reg_addr, *value);

    return RT_EOK;
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    uint8_t reg_data[] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    uint8_t reg[2] = {0x9f, 0xff};
    uint8_t read_data[16] = {0};

    rt_kprintf("Start spi demo!\n");
    rt_thread_mdelay(100);
    spi_dev_init();

    rt_thread_mdelay(10);
    spidev_write(&reg[1], 1);
    spidev_register_read(reg, 1, read_data, 16);
    rt_kprintf("spi read:");
    for (uint8_t i = 0; i < 16; i++)
    {
        rt_kprintf("0x%x,", read_data[i]);
    }
    rt_kprintf("\n");
    spidev_register_write(reg_data, 3, &reg_data[3], 13);
    rt_kprintf("spi demo end!\n");

    while (1)
    {
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }
    return RT_EOK;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

