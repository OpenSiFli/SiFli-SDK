/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "app_common.h"
#include "drv_spi.h"
#define SPI_BUS_NAME "spi2"
#define SPI_DEVICE_NAME   "jcprinter"

#define LOG_TAG "spi_app"
#include "log.h"

struct PRINTER_CONT_T
{
    struct rt_spi_device *bus_handle;
};

struct PRINTER_CONT_T printer_device = {0};
static struct rt_spi_configuration printer_cfg = {0};
static rt_device_t spi_bus;

static rt_err_t spi2_init(void)
{
    rt_err_t rst = RT_EOK;
    spi_bus = rt_device_find(SPI_BUS_NAME);

    if (spi_bus)
    {
        rt_device_open(spi_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_TX);
        LOG_D("Find spi bus %s:%x\n", SPI_BUS_NAME, spi_bus);

        printer_device.bus_handle = (struct rt_spi_device *)rt_device_find(SPI_DEVICE_NAME);
        if (printer_device.bus_handle == NULL)
        {
            LOG_D(" spi bus SPI_DEVICE_NAME %x\n", printer_device.bus_handle);
            rst = rt_hw_spi_device_attach(SPI_BUS_NAME, SPI_DEVICE_NAME);
            printer_device.bus_handle = (struct rt_spi_device *)rt_device_find(SPI_DEVICE_NAME);
            if (printer_device.bus_handle == NULL)
            {
                LOG_E("Register printer spi device fail\n");
                return -RT_ERROR;
            }
        }
        rst = rt_device_open((rt_device_t)(printer_device.bus_handle), RT_DEVICE_FLAG_RDWR);

        printer_cfg.data_width = 8;
        printer_cfg.max_hz = 20 * 1000 * 1000; // 50m
        printer_cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER;
        printer_cfg.frameMode = RT_SPI_MOTO; //RT_SPI_TI;
        rst = rt_spi_configure(printer_device.bus_handle, &printer_cfg);
        LOG_D("rt_spi_configure result:%d", rst);
    }
    else
    {
        LOG_D("spi2 didn't register! err !\n");
    }
    LOG_D("spi2_init end!\n");

    /* rt_pin_mode(gh3x2x_CS_PIN, PIN_MODE_OUTPUT); */
    /* rt_pin_write(gh3x2x_CS_PIN, PIN_HIGH); */

    return rst;
}

int32_t spidev_register_write(rt_uint8_t *reg_addr, rt_uint8_t reg_length, rt_uint8_t *value, rt_uint32_t size)
{
    LOG_D("spidev_register_write reg_addr:0x%x,reg_length:0x%x, value:0x%x,value_size:0x%x\n", *reg_addr, reg_length, *value, size);
    rt_err_t ret;
    struct rt_spi_device *device = (struct rt_spi_device *)printer_device.bus_handle;
    ret = rt_spi_take_bus(device);
    if (ret != RT_EOK)
    {
        LOG_E("reg_init rt_spi_take_bus error");
        return RT_ERROR;
    }

    ret = rt_spi_take(device);
    if (ret != RT_EOK)
    {
        LOG_E("reg_init rt_spi_take error");
        return RT_ERROR;
    }

    /* rt_pin_write(CS_PIN, PIN_LOW); */
    rt_spi_send_then_send(device, reg_addr, reg_length, value, size);
    /* rt_pin_write(CS_PIN, PIN_HIGH); */

    ret = rt_spi_release(device);
    if (ret != RT_EOK)
    {
        LOG_E("reg_init rt_spi_release error");
        return RT_ERROR;
    }

    ret = rt_spi_release_bus(device);
    if (ret != RT_EOK)
    {
        LOG_E("reg_init rt_spi_releasee_bus error");
        return RT_ERROR;
    }

    return RT_EOK;
}

void Printer_SpiWriteReg(uint16_t usRegAddr, uint16_t usRegValue)
{

    uint8_t value[2] = {0};
    uint8_t puchWriteBuf[7];
    puchWriteBuf[0] = 0xF0;
    puchWriteBuf[1] = usRegAddr >> 8;
    puchWriteBuf[2] = usRegAddr & 0x00FF;
    puchWriteBuf[3] = 0;
    puchWriteBuf[4] = 2;
    puchWriteBuf[5] = usRegValue >> 8;
    puchWriteBuf[6] = usRegValue & 0x00FF;
    rt_kprintf("spi_write_reg:%x,%x,value:%x,%x\n", puchWriteBuf[1], puchWriteBuf[2], puchWriteBuf[5], puchWriteBuf[6]);
    spidev_register_write(puchWriteBuf, 2, puchWriteBuf, 7);

}

int cmd_spi(int argc, char **argv)
{

    uint32_t value, addr, len, speed;
    int i, id, ret;
    uint8_t *buf;
    uint8_t reg[2] = {0x11, 0x22};

    if ((argc < 2) || (argc > 4))
    {
        rt_kprintf("err parameter! argc:%d", argc);
        rt_kprintf("        example:                               ");
        rt_kprintf("             pwm w 20 1000  /* write,20Mhz,1000bytes */ ");
        return (-RT_EINVAL);
    }
    speed = atoi(argv[2]);
    len = atoi(argv[3]);

    if ((speed < 1) || (speed > 48))
        speed = 20; //20Mhz
    rt_kprintf("cmd_spi:%s,speed:%d,len:%d,\n", argv[1], speed, len);

    HAL_PIN_Set(PAD_PA40, SPI2_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, SPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, SPI2_DIO, PIN_PULLUP, 1);

    if (strcmp(argv[1], "w") == 0)
    {

        printer_cfg.data_width = 8;
        printer_cfg.max_hz = speed * 1000 * 1000; // 50m
        printer_cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER;
        printer_cfg.frameMode = RT_SPI_MOTO; //RT_SPI_TI;
        rt_spi_configure(printer_device.bus_handle, &printer_cfg);

        rt_kprintf("cmd_spi w len:%d,speed:%d \n", len, speed);
        if (len > 4096) // set max lenght to 4096 to avoid malloc bufer too large
            len = 4096;

        buf = malloc(len);
        if (buf == NULL)
        {
            LOG_I("Alloc buffer len %d fail\n", len);
            return 0;
        }
        value = 0;
        for (i = 0; i < len; i++)
            buf[i] = (uint8_t)((value + i) & 0xff);

        spidev_register_write(reg, 2, buf, len);
    }
    else
    {
        Printer_SpiWriteReg(0x5566, 0x7788);
        rt_kprintf("cmd_spi err\n");
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_spi, __cmd_spi, spi cmd functions); /* 导出到 msh 命令列表中 */

void spi_task(void *parameter)
{
    rt_kprintf("spi_task entry!\n");
    HAL_PIN_Set(PAD_PA40, SPI2_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, SPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, SPI2_DIO, PIN_PULLUP, 1);

    spi2_init();
    Printer_SpiWriteReg(0x1122, 0x3344);

    HAL_PIN_Set(PAD_PA40, GPIO_A40, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1);
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

