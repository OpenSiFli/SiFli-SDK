/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-08-03     thread-liu        the first version
 */

#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>

#if defined(BSP_USING_GC032A)
#include <string.h>
#include <drv_gc032a.h>
#include <drv_dcmi.h>

#define DRV_DEBUG
#define LOG_TAG     "drv.gc032a"
#include <drv_log.h>

#define DEV_ADDRESS      (0x42 >> 1) /* GC032A address */
#define I2C_NAME        CAMERA_I2C_DEVICE_NAME

static struct rt_i2c_bus_device *i2c_bus  = RT_NULL;

static rt_err_t read_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msg[2];

    RT_ASSERT(bus != RT_NULL);

    msg[0].addr  = DEV_ADDRESS;
    msg[0].flags = RT_I2C_WR;
    msg[0].buf   = &reg;
    msg[0].len   = 2;

    msg[1].addr  = DEV_ADDRESS;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = len;
    msg[1].buf   = buf;

    if (rt_i2c_transfer(bus, msg, 2) == 2)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}

/* i2c write reg */
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];
    struct rt_i2c_msg msgs;

    RT_ASSERT(bus != RT_NULL);

    buf[0] = reg ;
    buf[1] = data;

    msgs.addr = DEV_ADDRESS;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 2;

    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        //LOG_I("i2c write %x %x", reg, data);
        return RT_EOK;
    }

    return RT_ERROR;
}

void gc032a_2bit_spi_drv(void)
{
#define GC032A_OUTPUT_MODE_CCIR656_2BIT
//#define GC032A_OUTPUT_MODE_PACKET_DDR_2BIT

#if defined(GC032A_OUTPUT_MODE_CCIR656_2BIT)
    /*System*/
    write_reg(i2c_bus, 0xf3, 0x83);
    write_reg(i2c_bus, 0xf5, 0x0c);
    write_reg(i2c_bus, 0xf7, 0x00);
    write_reg(i2c_bus, 0xf8, 0x41); //PLL 01  //03
    write_reg(i2c_bus, 0xf9, 0x4f);
    write_reg(i2c_bus, 0xfa, 0x10);
    write_reg(i2c_bus, 0xfc, 0x02);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x81, 0x03);

    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x77, 0x64);
    write_reg(i2c_bus, 0x78, 0x40);
    write_reg(i2c_bus, 0x79, 0x60);

    /*Analog&Cisctl*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x03, 0x01);
    write_reg(i2c_bus, 0x04, 0x20);
    write_reg(i2c_bus, 0x05, 0x01);
    write_reg(i2c_bus, 0x06, 0xaf);
    write_reg(i2c_bus, 0x07, 0x00);
    write_reg(i2c_bus, 0x08, 0x08);
    write_reg(i2c_bus, 0x0a, 0x00);
    write_reg(i2c_bus, 0x0c, 0x00);
    write_reg(i2c_bus, 0x0d, 0x01);
    write_reg(i2c_bus, 0x0e, 0xe8);
    write_reg(i2c_bus, 0x0f, 0x02);
    write_reg(i2c_bus, 0x10, 0x88);
    write_reg(i2c_bus, 0x17, 0x54);
    write_reg(i2c_bus, 0x19, 0x08);
    write_reg(i2c_bus, 0x1a, 0x0a);
    write_reg(i2c_bus, 0x1f, 0x40);
    write_reg(i2c_bus, 0x20, 0x30);
    write_reg(i2c_bus, 0x2e, 0x80);
    write_reg(i2c_bus, 0x2f, 0x2b);
    write_reg(i2c_bus, 0x30, 0x1a);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x03, 0x02);
    write_reg(i2c_bus, 0x06, 0x60);
    write_reg(i2c_bus, 0x05, 0xd7);
    write_reg(i2c_bus, 0x12, 0x89);

    /*SPI*/
    write_reg(i2c_bus, 0xfe, 0x03);
    write_reg(i2c_bus, 0x51, 0x01);
    write_reg(i2c_bus, 0x52, 0x5a);
    write_reg(i2c_bus, 0x53, 0x64); //[7]crc [3:2]line_num
    write_reg(i2c_bus, 0x54, 0x20);
    write_reg(i2c_bus, 0x55, 0x00); //20
    write_reg(i2c_bus, 0x59, 0x10);
    write_reg(i2c_bus, 0x5a, 0x01);
    write_reg(i2c_bus, 0x5b, 0x80);
    write_reg(i2c_bus, 0x5c, 0x02);
    write_reg(i2c_bus, 0x5d, 0xe0);
    write_reg(i2c_bus, 0x5e, 0x01);
    write_reg(i2c_bus, 0x64, 0x06); //[1]sck always 06 04
    write_reg(i2c_bus, 0x65, 0xff); //head sync code
    write_reg(i2c_bus, 0x66, 0xff);
    write_reg(i2c_bus, 0x67, 0xff);

    //SYNC code
    write_reg(i2c_bus, 0x60, 0xca); //frame_start -- not need
    write_reg(i2c_bus, 0x61, 0x80); //line sync start
    write_reg(i2c_bus, 0x62, 0x9d); //line sync end
    write_reg(i2c_bus, 0x63, 0xb6); //frame end */
    //write_reg(i2c_bus,0x60,0x01);  //frame_start -- not need
    //write_reg(i2c_bus,0x61,0x02);  //line sync start
    //write_reg(i2c_bus,0x62,0x40);  //line sync end
    //write_reg(i2c_bus,0x63,0x00);  //frame end */

    /*blk*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x18, 0x02);

    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x40, 0x22);
    write_reg(i2c_bus, 0x45, 0x00);
    write_reg(i2c_bus, 0x46, 0x00);
    write_reg(i2c_bus, 0x49, 0x20);
    write_reg(i2c_bus, 0x4b, 0x3c);
    write_reg(i2c_bus, 0x50, 0x20);
    write_reg(i2c_bus, 0x42, 0x10);

    /*isp*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x0a, 0xc5);
    write_reg(i2c_bus, 0x45, 0x00);
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x40, 0xff);
    write_reg(i2c_bus, 0x41, 0x25);
    write_reg(i2c_bus, 0x42, 0xcf);
    write_reg(i2c_bus, 0x43, 0x10);
    write_reg(i2c_bus, 0x44, 0x83); //////////////rgb565
    //write_reg(i2c_bus,0x44,0x06);//////////////rgb565
    write_reg(i2c_bus, 0x46, 0x22);
    write_reg(i2c_bus, 0x49, 0x03);
    write_reg(i2c_bus, 0x52, 0x02);
    write_reg(i2c_bus, 0x54, 0x00);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x22, 0xf6);

    /*Shading*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xc1, 0x38);
    write_reg(i2c_bus, 0xc2, 0x4c);
    write_reg(i2c_bus, 0xc3, 0x00);
    write_reg(i2c_bus, 0xc4, 0x32);
    write_reg(i2c_bus, 0xc5, 0x24);
    write_reg(i2c_bus, 0xc6, 0x16);
    write_reg(i2c_bus, 0xc7, 0x08);
    write_reg(i2c_bus, 0xc8, 0x08);
    write_reg(i2c_bus, 0xc9, 0x00);
    write_reg(i2c_bus, 0xca, 0x20);
    write_reg(i2c_bus, 0xdc, 0x8a);
    write_reg(i2c_bus, 0xdd, 0xa0);
    write_reg(i2c_bus, 0xde, 0xa6);
    write_reg(i2c_bus, 0xdf, 0x75);

    /*AWB*/ /*20170110*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x7c, 0x09);
    write_reg(i2c_bus, 0x65, 0x06);
    write_reg(i2c_bus, 0x7c, 0x08);
    write_reg(i2c_bus, 0x56, 0xf4);
    write_reg(i2c_bus, 0x66, 0x0f);
    write_reg(i2c_bus, 0x67, 0x84);
    write_reg(i2c_bus, 0x6b, 0x80);
    write_reg(i2c_bus, 0x6d, 0x12);
    write_reg(i2c_bus, 0x6e, 0xb0);
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x90, 0x00);
    write_reg(i2c_bus, 0x91, 0x00);
    write_reg(i2c_bus, 0x92, 0xf4);
    write_reg(i2c_bus, 0x93, 0xd5);
    write_reg(i2c_bus, 0x95, 0x0f);
    write_reg(i2c_bus, 0x96, 0xf4);
    write_reg(i2c_bus, 0x97, 0x2d);
    write_reg(i2c_bus, 0x98, 0x0f);
    write_reg(i2c_bus, 0x9a, 0x2d);
    write_reg(i2c_bus, 0x9b, 0x0f);
    write_reg(i2c_bus, 0x9c, 0x59);
    write_reg(i2c_bus, 0x9d, 0x2d);
    write_reg(i2c_bus, 0x9f, 0x67);
    write_reg(i2c_bus, 0xa0, 0x59);
    write_reg(i2c_bus, 0xa1, 0x00);
    write_reg(i2c_bus, 0xa2, 0x00);
    write_reg(i2c_bus, 0x86, 0x00);
    write_reg(i2c_bus, 0x87, 0x00);
    write_reg(i2c_bus, 0x88, 0x00);
    write_reg(i2c_bus, 0x89, 0x00);
    write_reg(i2c_bus, 0xa4, 0x00);
    write_reg(i2c_bus, 0xa5, 0x00);
    write_reg(i2c_bus, 0xa6, 0xd4);
    write_reg(i2c_bus, 0xa7, 0x9f);
    write_reg(i2c_bus, 0xa9, 0xd4);
    write_reg(i2c_bus, 0xaa, 0x9f);
    write_reg(i2c_bus, 0xab, 0xac);
    write_reg(i2c_bus, 0xac, 0x9f);
    write_reg(i2c_bus, 0xae, 0xd4);
    write_reg(i2c_bus, 0xaf, 0xac);
    write_reg(i2c_bus, 0xb0, 0xd4);
    write_reg(i2c_bus, 0xb1, 0xa3);
    write_reg(i2c_bus, 0xb3, 0xd4);
    write_reg(i2c_bus, 0xb4, 0xac);
    write_reg(i2c_bus, 0xb5, 0x00);
    write_reg(i2c_bus, 0xb6, 0x00);
    write_reg(i2c_bus, 0x8b, 0x00);
    write_reg(i2c_bus, 0x8c, 0x00);
    write_reg(i2c_bus, 0x8d, 0x00);
    write_reg(i2c_bus, 0x8e, 0x00);
    write_reg(i2c_bus, 0x94, 0x50);
    write_reg(i2c_bus, 0x99, 0xa6);
    write_reg(i2c_bus, 0x9e, 0xaa);
    write_reg(i2c_bus, 0xa3, 0x0a);
    write_reg(i2c_bus, 0x8a, 0x00);
    write_reg(i2c_bus, 0xa8, 0x50);
    write_reg(i2c_bus, 0xad, 0x55);
    write_reg(i2c_bus, 0xb2, 0x55);
    write_reg(i2c_bus, 0xb7, 0x05);
    write_reg(i2c_bus, 0x8f, 0x00);
    write_reg(i2c_bus, 0xb8, 0xb3);
    write_reg(i2c_bus, 0xb9, 0xb6);

    /*CC*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xd0, 0x40);
    write_reg(i2c_bus, 0xd1, 0xf8);
    write_reg(i2c_bus, 0xd2, 0x00);
    write_reg(i2c_bus, 0xd3, 0xfa);
    write_reg(i2c_bus, 0xd4, 0x45);
    write_reg(i2c_bus, 0xd5, 0x02);
    write_reg(i2c_bus, 0xd6, 0x30);
    write_reg(i2c_bus, 0xd7, 0xfa);
    write_reg(i2c_bus, 0xd8, 0x08);
    write_reg(i2c_bus, 0xd9, 0x08);
    write_reg(i2c_bus, 0xda, 0x58);
    write_reg(i2c_bus, 0xdb, 0x02);
    write_reg(i2c_bus, 0xfe, 0x00);

    /*Gamma*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xba, 0x00);
    write_reg(i2c_bus, 0xbb, 0x04);
    write_reg(i2c_bus, 0xbc, 0x0a);
    write_reg(i2c_bus, 0xbd, 0x0e);
    write_reg(i2c_bus, 0xbe, 0x22);
    write_reg(i2c_bus, 0xbf, 0x30);
    write_reg(i2c_bus, 0xc0, 0x3d);
    write_reg(i2c_bus, 0xc1, 0x4a);
    write_reg(i2c_bus, 0xc2, 0x5d);
    write_reg(i2c_bus, 0xc3, 0x6b);
    write_reg(i2c_bus, 0xc4, 0x7a);
    write_reg(i2c_bus, 0xc5, 0x85);
    write_reg(i2c_bus, 0xc6, 0x90);
    write_reg(i2c_bus, 0xc7, 0xa5);
    write_reg(i2c_bus, 0xc8, 0xb5);
    write_reg(i2c_bus, 0xc9, 0xc2);
    write_reg(i2c_bus, 0xca, 0xcc);
    write_reg(i2c_bus, 0xcb, 0xd5);
    write_reg(i2c_bus, 0xcc, 0xde);
    write_reg(i2c_bus, 0xcd, 0xea);
    write_reg(i2c_bus, 0xce, 0xf5);
    write_reg(i2c_bus, 0xcf, 0xff);

    /*Auto Gamma*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x5a, 0x08);
    write_reg(i2c_bus, 0x5b, 0x0f);
    write_reg(i2c_bus, 0x5c, 0x15);
    write_reg(i2c_bus, 0x5d, 0x1c);
    write_reg(i2c_bus, 0x5e, 0x28);
    write_reg(i2c_bus, 0x5f, 0x36);
    write_reg(i2c_bus, 0x60, 0x45);
    write_reg(i2c_bus, 0x61, 0x51);
    write_reg(i2c_bus, 0x62, 0x6a);
    write_reg(i2c_bus, 0x63, 0x7d);
    write_reg(i2c_bus, 0x64, 0x8d);
    write_reg(i2c_bus, 0x65, 0x98);
    write_reg(i2c_bus, 0x66, 0xa2);
    write_reg(i2c_bus, 0x67, 0xb5);
    write_reg(i2c_bus, 0x68, 0xc3);
    write_reg(i2c_bus, 0x69, 0xcd);
    write_reg(i2c_bus, 0x6a, 0xd4);
    write_reg(i2c_bus, 0x6b, 0xdc);
    write_reg(i2c_bus, 0x6c, 0xe3);
    write_reg(i2c_bus, 0x6d, 0xf0);
    write_reg(i2c_bus, 0x6e, 0xf9);
    write_reg(i2c_bus, 0x6f, 0xff);

    /*Gain*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x70, 0x50);

    /*AEC*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x4f, 0x01);
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x0d, 0x00); //08 add 20170110
    write_reg(i2c_bus, 0x12, 0xa0);
    write_reg(i2c_bus, 0x13, 0x3a);
    write_reg(i2c_bus, 0x44, 0x83); ////////////////////////////rgb565
    write_reg(i2c_bus, 0x1f, 0x30);
    write_reg(i2c_bus, 0x20, 0x40);

    write_reg(i2c_bus, 0x3e, 0x20);
    write_reg(i2c_bus, 0x3f, 0x2d);
    write_reg(i2c_bus, 0x40, 0x40);
    write_reg(i2c_bus, 0x41, 0x5b);
    write_reg(i2c_bus, 0x42, 0x82);
    write_reg(i2c_bus, 0x43, 0xb7);
    write_reg(i2c_bus, 0x04, 0x0a);
    write_reg(i2c_bus, 0x02, 0x79);
    write_reg(i2c_bus, 0x03, 0xc0);

    /*measure window*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xcc, 0x08);
    write_reg(i2c_bus, 0xcd, 0x08);
    write_reg(i2c_bus, 0xce, 0xa4);
    write_reg(i2c_bus, 0xcf, 0xec);

    /*DNDD*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x81, 0xb8);
    write_reg(i2c_bus, 0x82, 0x12);
    write_reg(i2c_bus, 0x83, 0x0a);
    write_reg(i2c_bus, 0x84, 0x01);
    write_reg(i2c_bus, 0x86, 0x50);
    write_reg(i2c_bus, 0x87, 0x18);
    write_reg(i2c_bus, 0x88, 0x10);
    write_reg(i2c_bus, 0x89, 0x70);
    write_reg(i2c_bus, 0x8a, 0x20);
    write_reg(i2c_bus, 0x8b, 0x10);
    write_reg(i2c_bus, 0x8c, 0x08);
    write_reg(i2c_bus, 0x8d, 0x0a);

    /*Intpee*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x8f, 0xaa);
    write_reg(i2c_bus, 0x90, 0x9c);
    write_reg(i2c_bus, 0x91, 0x52);
    write_reg(i2c_bus, 0x92, 0x03);
    write_reg(i2c_bus, 0x93, 0x03);
    write_reg(i2c_bus, 0x94, 0x08);
    write_reg(i2c_bus, 0x95, 0x44);
    write_reg(i2c_bus, 0x97, 0x00);
    write_reg(i2c_bus, 0x98, 0x00);

    /*ASDE*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xa1, 0x30);
    write_reg(i2c_bus, 0xa2, 0x41);
    write_reg(i2c_bus, 0xa4, 0x30);
    write_reg(i2c_bus, 0xa5, 0x20);
    write_reg(i2c_bus, 0xaa, 0x30);
    write_reg(i2c_bus, 0xac, 0x32);

    /*YCP*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xd1, 0x3c);
    write_reg(i2c_bus, 0xd2, 0x3c);
    write_reg(i2c_bus, 0xd3, 0x38);
    write_reg(i2c_bus, 0xd6, 0xf4);
    write_reg(i2c_bus, 0xd7, 0x1d);
    write_reg(i2c_bus, 0xdd, 0x73);
    write_reg(i2c_bus, 0xde, 0x84);

    /*Banding*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x05, 0x01); //hb
    write_reg(i2c_bus, 0x06, 0xe4);
    write_reg(i2c_bus, 0x07, 0x00); //vb
    write_reg(i2c_bus, 0x08, 0x08);

    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x25, 0x00); //step
    write_reg(i2c_bus, 0x26, 0x24);

    write_reg(i2c_bus, 0x27, 0x01); //7.1fps
    write_reg(i2c_bus, 0x28, 0xb0);
    write_reg(i2c_bus, 0x29, 0x01); //7.1fps
    write_reg(i2c_bus, 0x2a, 0xb0);
    write_reg(i2c_bus, 0x2b, 0x01); //7.1fps
    write_reg(i2c_bus, 0x2c, 0xb0);
    write_reg(i2c_bus, 0x2d, 0x01); //7.1fps
    write_reg(i2c_bus, 0x2e, 0xb0);
    write_reg(i2c_bus, 0x2f, 0x02); //6.24fps
    write_reg(i2c_bus, 0x30, 0x40);
    write_reg(i2c_bus, 0x31, 0x02); //4.99fps
    write_reg(i2c_bus, 0x32, 0xd0);
    write_reg(i2c_bus, 0x33, 0x03); //4.16fps
    write_reg(i2c_bus, 0x34, 0x60);
    write_reg(i2c_bus, 0x3c, 0x30);
    write_reg(i2c_bus, 0xfe, 0x00);

#elif defined(GC032A_OUTPUT_MODE_PACKET_DDR_2BIT)
    /*System*/
    write_reg(i2c_bus, 0xf3, 0x83); //ff//1f//01 data output
    write_reg(i2c_bus, 0xf5, 0x0f);
    write_reg(i2c_bus, 0xf7, 0x01);
    write_reg(i2c_bus, 0xf8, 0x01);
    write_reg(i2c_bus, 0xf9, 0x4e);
    write_reg(i2c_bus, 0xfa, 0x00);
    write_reg(i2c_bus, 0xfc, 0x02);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x81, 0x03);

    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x77, 0x64);
    write_reg(i2c_bus, 0x78, 0x40);
    write_reg(i2c_bus, 0x79, 0x60);
    /*Analog&Cisctl*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x03, 0x01);
    write_reg(i2c_bus, 0x04, 0xf8);
    write_reg(i2c_bus, 0x05, 0x01);
    write_reg(i2c_bus, 0x06, 0xa8);
    write_reg(i2c_bus, 0x07, 0x00);
    write_reg(i2c_bus, 0x08, 0x10);

    write_reg(i2c_bus, 0x0a, 0x00);
    write_reg(i2c_bus, 0x0c, 0x00);
    write_reg(i2c_bus, 0x0d, 0x01);
    write_reg(i2c_bus, 0x0e, 0xe8);
    write_reg(i2c_bus, 0x0f, 0x02);
    write_reg(i2c_bus, 0x10, 0x88);
    write_reg(i2c_bus, 0x17, 0x54);
    write_reg(i2c_bus, 0x19, 0x08);
    write_reg(i2c_bus, 0x1a, 0x0a);
    write_reg(i2c_bus, 0x1f, 0x40);
    write_reg(i2c_bus, 0x20, 0x30);
    write_reg(i2c_bus, 0x2e, 0x80);
    write_reg(i2c_bus, 0x2f, 0x2b);
    write_reg(i2c_bus, 0x30, 0x1a);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x03, 0x02);
    write_reg(i2c_bus, 0x05, 0xd7);
    write_reg(i2c_bus, 0x06, 0x60);
    write_reg(i2c_bus, 0x08, 0x80);
    write_reg(i2c_bus, 0x12, 0x49);

    /*SPI*/
    write_reg(i2c_bus, 0xfe, 0x03);
    write_reg(i2c_bus, 0x52, 0x38);
    write_reg(i2c_bus, 0x53, 0x64);
    write_reg(i2c_bus, 0x54, 0x20);
    write_reg(i2c_bus, 0x55, 0x00);
    write_reg(i2c_bus, 0x59, 0x10);
    write_reg(i2c_bus, 0x5a, 0x40); //00 //yuv
    write_reg(i2c_bus, 0x5b, 0x80);
    write_reg(i2c_bus, 0x5c, 0x02);
    write_reg(i2c_bus, 0x5d, 0xe0);
    write_reg(i2c_bus, 0x5e, 0x01);
    write_reg(i2c_bus, 0x51, 0x03);
    write_reg(i2c_bus, 0x64, 0x06);
    write_reg(i2c_bus, 0xfe, 0x00);

    /*blk*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x18, 0x02);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x40, 0x22);
    write_reg(i2c_bus, 0x45, 0x00);
    write_reg(i2c_bus, 0x46, 0x00);
    write_reg(i2c_bus, 0x49, 0x20);
    write_reg(i2c_bus, 0x4b, 0x3c);
    write_reg(i2c_bus, 0x50, 0x20);
    write_reg(i2c_bus, 0x42, 0x10);

    /*isp*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x0a, 0xc5);
    write_reg(i2c_bus, 0x45, 0x00);
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x40, 0xff);
    write_reg(i2c_bus, 0x41, 0x25);
    write_reg(i2c_bus, 0x42, 0xcf);
    write_reg(i2c_bus, 0x43, 0x10);
    write_reg(i2c_bus, 0x44, 0x83);
    write_reg(i2c_bus, 0x46, 0x22);
    write_reg(i2c_bus, 0x49, 0x03);
    write_reg(i2c_bus, 0x52, 0x02);
    write_reg(i2c_bus, 0x54, 0x00);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x22, 0xf6);

    /*Shading*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xc1, 0x38);
    write_reg(i2c_bus, 0xc2, 0x4c);
    write_reg(i2c_bus, 0xc3, 0x00);
    write_reg(i2c_bus, 0xc4, 0x32);
    write_reg(i2c_bus, 0xc5, 0x24);
    write_reg(i2c_bus, 0xc6, 0x16);
    write_reg(i2c_bus, 0xc7, 0x08);
    write_reg(i2c_bus, 0xc8, 0x08);
    write_reg(i2c_bus, 0xc9, 0x00);
    write_reg(i2c_bus, 0xca, 0x20);
    write_reg(i2c_bus, 0xdc, 0x8a);
    write_reg(i2c_bus, 0xdd, 0xa0);
    write_reg(i2c_bus, 0xde, 0xa6);
    write_reg(i2c_bus, 0xdf, 0x75);

    /*AWB*/ /*20170110*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x7c, 0x09);
    write_reg(i2c_bus, 0x65, 0x06);
    write_reg(i2c_bus, 0x7c, 0x08);
    write_reg(i2c_bus, 0x56, 0xf4);
    write_reg(i2c_bus, 0x66, 0x0f);
    write_reg(i2c_bus, 0x67, 0x84);
    write_reg(i2c_bus, 0x6b, 0x80);
    write_reg(i2c_bus, 0x6d, 0x12);
    write_reg(i2c_bus, 0x6e, 0xb0);
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x90, 0x00);
    write_reg(i2c_bus, 0x91, 0x00);
    write_reg(i2c_bus, 0x92, 0xf4);
    write_reg(i2c_bus, 0x93, 0xd5);
    write_reg(i2c_bus, 0x95, 0x0f);
    write_reg(i2c_bus, 0x96, 0xf4);
    write_reg(i2c_bus, 0x97, 0x2d);
    write_reg(i2c_bus, 0x98, 0x0f);
    write_reg(i2c_bus, 0x9a, 0x2d);
    write_reg(i2c_bus, 0x9b, 0x0f);
    write_reg(i2c_bus, 0x9c, 0x59);
    write_reg(i2c_bus, 0x9d, 0x2d);
    write_reg(i2c_bus, 0x9f, 0x67);
    write_reg(i2c_bus, 0xa0, 0x59);
    write_reg(i2c_bus, 0xa1, 0x00);
    write_reg(i2c_bus, 0xa2, 0x00);
    write_reg(i2c_bus, 0x86, 0x00);
    write_reg(i2c_bus, 0x87, 0x00);
    write_reg(i2c_bus, 0x88, 0x00);
    write_reg(i2c_bus, 0x89, 0x00);
    write_reg(i2c_bus, 0xa4, 0x00);
    write_reg(i2c_bus, 0xa5, 0x00);
    write_reg(i2c_bus, 0xa6, 0xd4);
    write_reg(i2c_bus, 0xa7, 0x9f);
    write_reg(i2c_bus, 0xa9, 0xd4);
    write_reg(i2c_bus, 0xaa, 0x9f);
    write_reg(i2c_bus, 0xab, 0xac);
    write_reg(i2c_bus, 0xac, 0x9f);
    write_reg(i2c_bus, 0xae, 0xd4);
    write_reg(i2c_bus, 0xaf, 0xac);
    write_reg(i2c_bus, 0xb0, 0xd4);
    write_reg(i2c_bus, 0xb1, 0xa3);
    write_reg(i2c_bus, 0xb3, 0xd4);
    write_reg(i2c_bus, 0xb4, 0xac);
    write_reg(i2c_bus, 0xb5, 0x00);
    write_reg(i2c_bus, 0xb6, 0x00);
    write_reg(i2c_bus, 0x8b, 0x00);
    write_reg(i2c_bus, 0x8c, 0x00);
    write_reg(i2c_bus, 0x8d, 0x00);
    write_reg(i2c_bus, 0x8e, 0x00);
    write_reg(i2c_bus, 0x94, 0x50);
    write_reg(i2c_bus, 0x99, 0xa6);
    write_reg(i2c_bus, 0x9e, 0xaa);
    write_reg(i2c_bus, 0xa3, 0x0a);
    write_reg(i2c_bus, 0x8a, 0x00);
    write_reg(i2c_bus, 0xa8, 0x50);
    write_reg(i2c_bus, 0xad, 0x55);
    write_reg(i2c_bus, 0xb2, 0x55);
    write_reg(i2c_bus, 0xb7, 0x05);
    write_reg(i2c_bus, 0x8f, 0x00);
    write_reg(i2c_bus, 0xb8, 0xb3);
    write_reg(i2c_bus, 0xb9, 0xb6);

    /*CC*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xd0, 0x40);
    write_reg(i2c_bus, 0xd1, 0xf8);
    write_reg(i2c_bus, 0xd2, 0x00);
    write_reg(i2c_bus, 0xd3, 0xfa);
    write_reg(i2c_bus, 0xd4, 0x45);
    write_reg(i2c_bus, 0xd5, 0x02);

    write_reg(i2c_bus, 0xd6, 0x30);
    write_reg(i2c_bus, 0xd7, 0xfa);
    write_reg(i2c_bus, 0xd8, 0x08);
    write_reg(i2c_bus, 0xd9, 0x08);
    write_reg(i2c_bus, 0xda, 0x58);
    write_reg(i2c_bus, 0xdb, 0x02);
    write_reg(i2c_bus, 0xfe, 0x00);

    /*Gamma*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xba, 0x00);
    write_reg(i2c_bus, 0xbb, 0x04);
    write_reg(i2c_bus, 0xbc, 0x0a);
    write_reg(i2c_bus, 0xbd, 0x0e);
    write_reg(i2c_bus, 0xbe, 0x22);
    write_reg(i2c_bus, 0xbf, 0x30);
    write_reg(i2c_bus, 0xc0, 0x3d);
    write_reg(i2c_bus, 0xc1, 0x4a);
    write_reg(i2c_bus, 0xc2, 0x5d);
    write_reg(i2c_bus, 0xc3, 0x6b);
    write_reg(i2c_bus, 0xc4, 0x7a);
    write_reg(i2c_bus, 0xc5, 0x85);
    write_reg(i2c_bus, 0xc6, 0x90);
    write_reg(i2c_bus, 0xc7, 0xa5);
    write_reg(i2c_bus, 0xc8, 0xb5);
    write_reg(i2c_bus, 0xc9, 0xc2);
    write_reg(i2c_bus, 0xca, 0xcc);
    write_reg(i2c_bus, 0xcb, 0xd5);
    write_reg(i2c_bus, 0xcc, 0xde);
    write_reg(i2c_bus, 0xcd, 0xea);
    write_reg(i2c_bus, 0xce, 0xf5);
    write_reg(i2c_bus, 0xcf, 0xff);

    /*Auto Gamma*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x5a, 0x08);
    write_reg(i2c_bus, 0x5b, 0x0f);
    write_reg(i2c_bus, 0x5c, 0x15);
    write_reg(i2c_bus, 0x5d, 0x1c);
    write_reg(i2c_bus, 0x5e, 0x28);
    write_reg(i2c_bus, 0x5f, 0x36);
    write_reg(i2c_bus, 0x60, 0x45);
    write_reg(i2c_bus, 0x61, 0x51);
    write_reg(i2c_bus, 0x62, 0x6a);
    write_reg(i2c_bus, 0x63, 0x7d);
    write_reg(i2c_bus, 0x64, 0x8d);
    write_reg(i2c_bus, 0x65, 0x98);
    write_reg(i2c_bus, 0x66, 0xa2);
    write_reg(i2c_bus, 0x67, 0xb5);
    write_reg(i2c_bus, 0x68, 0xc3);
    write_reg(i2c_bus, 0x69, 0xcd);
    write_reg(i2c_bus, 0x6a, 0xd4);
    write_reg(i2c_bus, 0x6b, 0xdc);
    write_reg(i2c_bus, 0x6c, 0xe3);
    write_reg(i2c_bus, 0x6d, 0xf0);
    write_reg(i2c_bus, 0x6e, 0xf9);
    write_reg(i2c_bus, 0x6f, 0xff);

    /*Gain*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x70, 0x50);

    /*AEC*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x4f, 0x01);
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x0d, 0x00); //08 add 20170110
    write_reg(i2c_bus, 0x12, 0xa0);
    write_reg(i2c_bus, 0x13, 0x3a);
    write_reg(i2c_bus, 0x44, 0x04);
    write_reg(i2c_bus, 0x1f, 0x30);
    write_reg(i2c_bus, 0x20, 0x40);

    write_reg(i2c_bus, 0x3e, 0x20);
    write_reg(i2c_bus, 0x3f, 0x2d);
    write_reg(i2c_bus, 0x40, 0x40);
    write_reg(i2c_bus, 0x41, 0x5b);
    write_reg(i2c_bus, 0x42, 0x82);
    write_reg(i2c_bus, 0x43, 0xb7);
    write_reg(i2c_bus, 0x04, 0x0a);
    write_reg(i2c_bus, 0x02, 0x79);
    write_reg(i2c_bus, 0x03, 0xc0);

    /*measure window*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xcc, 0x08);
    write_reg(i2c_bus, 0xcd, 0x08);
    write_reg(i2c_bus, 0xce, 0xa4);
    write_reg(i2c_bus, 0xcf, 0xec);

    /*DNDD*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x81, 0xb8);
    write_reg(i2c_bus, 0x82, 0x12);
    write_reg(i2c_bus, 0x83, 0x0a);
    write_reg(i2c_bus, 0x84, 0x01);
    write_reg(i2c_bus, 0x86, 0x50);
    write_reg(i2c_bus, 0x87, 0x18);
    write_reg(i2c_bus, 0x88, 0x10);
    write_reg(i2c_bus, 0x89, 0x70);
    write_reg(i2c_bus, 0x8a, 0x20);
    write_reg(i2c_bus, 0x8b, 0x10);
    write_reg(i2c_bus, 0x8c, 0x08);
    write_reg(i2c_bus, 0x8d, 0x0a);

    /*Intpee*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x8f, 0xaa);
    write_reg(i2c_bus, 0x90, 0x9c);
    write_reg(i2c_bus, 0x91, 0x52);
    write_reg(i2c_bus, 0x92, 0x03);
    write_reg(i2c_bus, 0x93, 0x03);
    write_reg(i2c_bus, 0x94, 0x08);
    write_reg(i2c_bus, 0x95, 0x44);
    write_reg(i2c_bus, 0x97, 0x00);
    write_reg(i2c_bus, 0x98, 0x00);

    /*ASDE*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xa1, 0x30);
    write_reg(i2c_bus, 0xa2, 0x41);
    write_reg(i2c_bus, 0xa4, 0x30);
    write_reg(i2c_bus, 0xa5, 0x20);
    write_reg(i2c_bus, 0xaa, 0x30);
    write_reg(i2c_bus, 0xac, 0x32);

    /*YCP*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xd1, 0x3c);
    write_reg(i2c_bus, 0xd2, 0x3c);
    write_reg(i2c_bus, 0xd3, 0x38);
    write_reg(i2c_bus, 0xd6, 0xf4);
    write_reg(i2c_bus, 0xd7, 0x1d);
    write_reg(i2c_bus, 0xdd, 0x73);
    write_reg(i2c_bus, 0xde, 0x84);

    /*Banding*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x05, 0x01);
    write_reg(i2c_bus, 0x06, 0xa8);
    write_reg(i2c_bus, 0x07, 0x00);
    write_reg(i2c_bus, 0x08, 0x10);

    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x25, 0x00);
    write_reg(i2c_bus, 0x26, 0x54);

    write_reg(i2c_bus, 0x27, 0x01);
    write_reg(i2c_bus, 0x28, 0xf8); //16.6fps
    write_reg(i2c_bus, 0x29, 0x02);
    write_reg(i2c_bus, 0x2a, 0xa0); //12.5fps
    write_reg(i2c_bus, 0x2b, 0x03);
    write_reg(i2c_bus, 0x2c, 0x48); //10fps
    write_reg(i2c_bus, 0x2d, 0x03);
    write_reg(i2c_bus, 0x2e, 0xf0); //8.33fps
    write_reg(i2c_bus, 0x2f, 0x05);
    write_reg(i2c_bus, 0x30, 0x94); //5.88fps
    write_reg(i2c_bus, 0x31, 0x07);
    write_reg(i2c_bus, 0x32, 0x8c); //4.34fps
    write_reg(i2c_bus, 0x33, 0x08);
    write_reg(i2c_bus, 0x34, 0x34); //3.99fps
    write_reg(i2c_bus, 0x3c, 0x30);
    write_reg(i2c_bus, 0xfe, 0x00);
#endif
}

void gc032a_8bit_dvp_drv(void)
{
    /*System*/
    write_reg(i2c_bus, 0xf3, 0xff);
    write_reg(i2c_bus, 0xf5, 0x06);
    write_reg(i2c_bus, 0xf7, 0x01);
    write_reg(i2c_bus, 0xf8, 0x03); //0x03 - MCLK * 1, 0x06 - MCLK * 2
    write_reg(i2c_bus, 0xf9, 0xce);
    write_reg(i2c_bus, 0xfa, 0x00);
    write_reg(i2c_bus, 0xfc, 0x02);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x81, 0x03);

    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x77, 0x64);
    write_reg(i2c_bus, 0x78, 0x40);
    write_reg(i2c_bus, 0x79, 0x60);
    /*ANALOG & CISCTL*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x03, 0x01);
    write_reg(i2c_bus, 0x04, 0xce);
    write_reg(i2c_bus, 0x05, 0x01);
    write_reg(i2c_bus, 0x06, 0xad);
    write_reg(i2c_bus, 0x07, 0x00);
    write_reg(i2c_bus, 0x08, 0x10);
    write_reg(i2c_bus, 0x0a, 0x00);
    write_reg(i2c_bus, 0x0c, 0x00);
    write_reg(i2c_bus, 0x0d, 0x01);
    write_reg(i2c_bus, 0x0e, 0xe8);
    write_reg(i2c_bus, 0x0f, 0x02);
    write_reg(i2c_bus, 0x10, 0x88);
    write_reg(i2c_bus, 0x17, 0x54);
    write_reg(i2c_bus, 0x19, 0x08);
    write_reg(i2c_bus, 0x1a, 0x0a);
    write_reg(i2c_bus, 0x1f, 0x40);
    write_reg(i2c_bus, 0x20, 0x30);
    write_reg(i2c_bus, 0x2e, 0x80);
    write_reg(i2c_bus, 0x2f, 0x2b);
    write_reg(i2c_bus, 0x30, 0x1a);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x03, 0x02);
    write_reg(i2c_bus, 0x05, 0xd7);
    write_reg(i2c_bus, 0x06, 0x60);
    write_reg(i2c_bus, 0x08, 0x80);
    write_reg(i2c_bus, 0x12, 0x09);

    /* BLK*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x18, 0x02);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x40, 0x22);
    //write_reg(i2c_bus,0x44,0x10); //global offset
    write_reg(i2c_bus, 0x45, 0x00);
    write_reg(i2c_bus, 0x46, 0x01);
    write_reg(i2c_bus, 0x49, 0x20);
    write_reg(i2c_bus, 0x4b, 0x3c);
    //write_reg(i2c_bus,0x4d,0x0f);
    //write_reg(i2c_bus,0x4e,0x0f);
    write_reg(i2c_bus, 0x50, 0x20);
    write_reg(i2c_bus, 0x42, 0x10);

    /*ISP*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x0a, 0xc5);
    write_reg(i2c_bus, 0x45, 0x00);
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x40, 0xff);
    write_reg(i2c_bus, 0x41, 0x25);
    write_reg(i2c_bus, 0x42, 0xcf);
    write_reg(i2c_bus, 0x43, 0x10);
    write_reg(i2c_bus, 0x44, 0x83);
    //write_reg(i2c_bus,0x44,0x06);
    write_reg(i2c_bus, 0x46, 0x22);
    write_reg(i2c_bus, 0x49, 0x03);
    write_reg(i2c_bus, 0x52, 0x02);
    write_reg(i2c_bus, 0x54, 0x00);
    write_reg(i2c_bus, 0xfe, 0x02);
    write_reg(i2c_bus, 0x22, 0xf6);

    /*Shading*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xc1, 0x38);
    write_reg(i2c_bus, 0xc2, 0x4c);
    write_reg(i2c_bus, 0xc3, 0x00);
    write_reg(i2c_bus, 0xc4, 0x32);
    write_reg(i2c_bus, 0xc5, 0x24);
    write_reg(i2c_bus, 0xc6, 0x16);
    write_reg(i2c_bus, 0xc7, 0x08);
    write_reg(i2c_bus, 0xc8, 0x08);
    write_reg(i2c_bus, 0xc9, 0x00);
    write_reg(i2c_bus, 0xca, 0x20);
    write_reg(i2c_bus, 0xdc, 0x8a);
    write_reg(i2c_bus, 0xdd, 0xa0);
    write_reg(i2c_bus, 0xde, 0xa6);
    write_reg(i2c_bus, 0xdf, 0x75);

    /*AWB*//*20170110*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x7c, 0x09);
    write_reg(i2c_bus, 0x65, 0x06);
    write_reg(i2c_bus, 0x7c, 0x08);
    write_reg(i2c_bus, 0x56, 0xf4);
    write_reg(i2c_bus, 0x66, 0x0f);
    write_reg(i2c_bus, 0x67, 0x84);
    write_reg(i2c_bus, 0x6b, 0x80);
    write_reg(i2c_bus, 0x6d, 0x12);
    write_reg(i2c_bus, 0x6e, 0xb0);
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x90, 0x00);
    write_reg(i2c_bus, 0x91, 0x00);
    write_reg(i2c_bus, 0x92, 0xf4);
    write_reg(i2c_bus, 0x93, 0xd5);
    write_reg(i2c_bus, 0x95, 0x0f);
    write_reg(i2c_bus, 0x96, 0xf4);
    write_reg(i2c_bus, 0x97, 0x2d);
    write_reg(i2c_bus, 0x98, 0x0f);
    write_reg(i2c_bus, 0x9a, 0x2d);
    write_reg(i2c_bus, 0x9b, 0x0f);
    write_reg(i2c_bus, 0x9c, 0x59);
    write_reg(i2c_bus, 0x9d, 0x2d);
    write_reg(i2c_bus, 0x9f, 0x67);
    write_reg(i2c_bus, 0xa0, 0x59);
    write_reg(i2c_bus, 0xa1, 0x00);
    write_reg(i2c_bus, 0xa2, 0x00);
    write_reg(i2c_bus, 0x86, 0x00);
    write_reg(i2c_bus, 0x87, 0x00);
    write_reg(i2c_bus, 0x88, 0x00);
    write_reg(i2c_bus, 0x89, 0x00);
    write_reg(i2c_bus, 0xa4, 0x00);
    write_reg(i2c_bus, 0xa5, 0x00);
    write_reg(i2c_bus, 0xa6, 0xd4);
    write_reg(i2c_bus, 0xa7, 0x9f);
    write_reg(i2c_bus, 0xa9, 0xd4);
    write_reg(i2c_bus, 0xaa, 0x9f);
    write_reg(i2c_bus, 0xab, 0xac);
    write_reg(i2c_bus, 0xac, 0x9f);
    write_reg(i2c_bus, 0xae, 0xd4);
    write_reg(i2c_bus, 0xaf, 0xac);
    write_reg(i2c_bus, 0xb0, 0xd4);
    write_reg(i2c_bus, 0xb1, 0xa3);
    write_reg(i2c_bus, 0xb3, 0xd4);
    write_reg(i2c_bus, 0xb4, 0xac);
    write_reg(i2c_bus, 0xb5, 0x00);
    write_reg(i2c_bus, 0xb6, 0x00);
    write_reg(i2c_bus, 0x8b, 0x00);
    write_reg(i2c_bus, 0x8c, 0x00);
    write_reg(i2c_bus, 0x8d, 0x00);
    write_reg(i2c_bus, 0x8e, 0x00);
    write_reg(i2c_bus, 0x94, 0x50);
    write_reg(i2c_bus, 0x99, 0xa6);
    write_reg(i2c_bus, 0x9e, 0xaa);
    write_reg(i2c_bus, 0xa3, 0x0a);
    write_reg(i2c_bus, 0x8a, 0x00);
    write_reg(i2c_bus, 0xa8, 0x50);
    write_reg(i2c_bus, 0xad, 0x55);
    write_reg(i2c_bus, 0xb2, 0x55);
    write_reg(i2c_bus, 0xb7, 0x05);
    write_reg(i2c_bus, 0x8f, 0x00);
    write_reg(i2c_bus, 0xb8, 0xb3);
    write_reg(i2c_bus, 0xb9, 0xb6);


    /*CC*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xd0, 0x40);
    write_reg(i2c_bus, 0xd1, 0xf8);
    write_reg(i2c_bus, 0xd2, 0x00);
    write_reg(i2c_bus, 0xd3, 0xfa);
    write_reg(i2c_bus, 0xd4, 0x45);
    write_reg(i2c_bus, 0xd5, 0x02);

    write_reg(i2c_bus, 0xd6, 0x30);
    write_reg(i2c_bus, 0xd7, 0xfa);
    write_reg(i2c_bus, 0xd8, 0x08);
    write_reg(i2c_bus, 0xd9, 0x08);
    write_reg(i2c_bus, 0xda, 0x58);
    write_reg(i2c_bus, 0xdb, 0x02);
    write_reg(i2c_bus, 0xfe, 0x00);

    /*Gamma*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xba, 0x00);
    write_reg(i2c_bus, 0xbb, 0x04);
    write_reg(i2c_bus, 0xbc, 0x0a);
    write_reg(i2c_bus, 0xbd, 0x0e);
    write_reg(i2c_bus, 0xbe, 0x22);
    write_reg(i2c_bus, 0xbf, 0x30);
    write_reg(i2c_bus, 0xc0, 0x3d);
    write_reg(i2c_bus, 0xc1, 0x4a);
    write_reg(i2c_bus, 0xc2, 0x5d);
    write_reg(i2c_bus, 0xc3, 0x6b);
    write_reg(i2c_bus, 0xc4, 0x7a);
    write_reg(i2c_bus, 0xc5, 0x85);
    write_reg(i2c_bus, 0xc6, 0x90);
    write_reg(i2c_bus, 0xc7, 0xa5);
    write_reg(i2c_bus, 0xc8, 0xb5);
    write_reg(i2c_bus, 0xc9, 0xc2);
    write_reg(i2c_bus, 0xca, 0xcc);
    write_reg(i2c_bus, 0xcb, 0xd5);
    write_reg(i2c_bus, 0xcc, 0xde);
    write_reg(i2c_bus, 0xcd, 0xea);
    write_reg(i2c_bus, 0xce, 0xf5);
    write_reg(i2c_bus, 0xcf, 0xff);

    /*Auto Gamma*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x5a, 0x08);
    write_reg(i2c_bus, 0x5b, 0x0f);
    write_reg(i2c_bus, 0x5c, 0x15);
    write_reg(i2c_bus, 0x5d, 0x1c);
    write_reg(i2c_bus, 0x5e, 0x28);
    write_reg(i2c_bus, 0x5f, 0x36);
    write_reg(i2c_bus, 0x60, 0x45);
    write_reg(i2c_bus, 0x61, 0x51);
    write_reg(i2c_bus, 0x62, 0x6a);
    write_reg(i2c_bus, 0x63, 0x7d);
    write_reg(i2c_bus, 0x64, 0x8d);
    write_reg(i2c_bus, 0x65, 0x98);
    write_reg(i2c_bus, 0x66, 0xa2);
    write_reg(i2c_bus, 0x67, 0xb5);
    write_reg(i2c_bus, 0x68, 0xc3);
    write_reg(i2c_bus, 0x69, 0xcd);
    write_reg(i2c_bus, 0x6a, 0xd4);
    write_reg(i2c_bus, 0x6b, 0xdc);
    write_reg(i2c_bus, 0x6c, 0xe3);
    write_reg(i2c_bus, 0x6d, 0xf0);
    write_reg(i2c_bus, 0x6e, 0xf9);
    write_reg(i2c_bus, 0x6f, 0xff);

    /*Gain*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x70, 0x50);

    /*AEC*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x4f, 0x00);
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x0d, 0x00); //08 add 20170110
    write_reg(i2c_bus, 0x12, 0x09);
    write_reg(i2c_bus, 0x13, 0x3a);
    write_reg(i2c_bus, 0x44, 0x04);
    write_reg(i2c_bus, 0x1f, 0x30);
    write_reg(i2c_bus, 0x20, 0x40);
    write_reg(i2c_bus, 0x26, 0x9a);
    write_reg(i2c_bus, 0x3e, 0x20);
    write_reg(i2c_bus, 0x3f, 0x2d);
    write_reg(i2c_bus, 0x40, 0x40);
    write_reg(i2c_bus, 0x41, 0x5b);
    write_reg(i2c_bus, 0x42, 0x82);
    write_reg(i2c_bus, 0x43, 0xb7);
    write_reg(i2c_bus, 0x04, 0x0a);
    write_reg(i2c_bus, 0x02, 0x79);
    write_reg(i2c_bus, 0x03, 0xc0);

    /*measure window*/
    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0xcc, 0x08);
    write_reg(i2c_bus, 0xcd, 0x08);
    write_reg(i2c_bus, 0xce, 0xa4);
    write_reg(i2c_bus, 0xcf, 0xec);

    /*DNDD*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x81, 0xb8);
    write_reg(i2c_bus, 0x82, 0x12);
    write_reg(i2c_bus, 0x83, 0x0a);
    write_reg(i2c_bus, 0x84, 0x01);
    write_reg(i2c_bus, 0x86, 0x50);
    write_reg(i2c_bus, 0x87, 0x18);
    write_reg(i2c_bus, 0x88, 0x10);
    write_reg(i2c_bus, 0x89, 0x70);
    write_reg(i2c_bus, 0x8a, 0x20);
    write_reg(i2c_bus, 0x8b, 0x10);
    write_reg(i2c_bus, 0x8c, 0x08);
    write_reg(i2c_bus, 0x8d, 0x0a);

    /*Intpee*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x8f, 0xaa);
    write_reg(i2c_bus, 0x90, 0x9c);
    write_reg(i2c_bus, 0x91, 0x52);
    write_reg(i2c_bus, 0x92, 0x03);
    write_reg(i2c_bus, 0x93, 0x03);
    write_reg(i2c_bus, 0x94, 0x08);
    write_reg(i2c_bus, 0x95, 0x44);
    write_reg(i2c_bus, 0x97, 0x00);
    write_reg(i2c_bus, 0x98, 0x00);

    /*ASDE*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xa1, 0x30);
    write_reg(i2c_bus, 0xa2, 0x41);
    write_reg(i2c_bus, 0xa4, 0x30);
    write_reg(i2c_bus, 0xa5, 0x20);
    write_reg(i2c_bus, 0xaa, 0x30);
    write_reg(i2c_bus, 0xac, 0x32);

    /*YCP*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0xd1, 0x3c);
    write_reg(i2c_bus, 0xd2, 0x3c);
    write_reg(i2c_bus, 0xd3, 0x38);
    write_reg(i2c_bus, 0xd6, 0xf4);
    write_reg(i2c_bus, 0xd7, 0x1d);
    write_reg(i2c_bus, 0xdd, 0x73);
    write_reg(i2c_bus, 0xde, 0x84);

    /*Banding*/
    write_reg(i2c_bus, 0xfe, 0x00);
    write_reg(i2c_bus, 0x05, 0x01);
    write_reg(i2c_bus, 0x06, 0xad);
    write_reg(i2c_bus, 0x07, 0x00);
    write_reg(i2c_bus, 0x08, 0x0a);

    write_reg(i2c_bus, 0xfe, 0x01);
    write_reg(i2c_bus, 0x25, 0x00);
    write_reg(i2c_bus, 0x26, 0x9a);

    write_reg(i2c_bus, 0x27, 0x01);
    write_reg(i2c_bus, 0x28, 0xce);
    write_reg(i2c_bus, 0x29, 0x03);
    write_reg(i2c_bus, 0x2a, 0x02);
    write_reg(i2c_bus, 0x2b, 0x04);
    write_reg(i2c_bus, 0x2c, 0x36);
    write_reg(i2c_bus, 0x2d, 0x07);
    write_reg(i2c_bus, 0x2e, 0xd2);
    write_reg(i2c_bus, 0x2f, 0x0b);
    write_reg(i2c_bus, 0x30, 0x6e);
    write_reg(i2c_bus, 0x31, 0x0e);
    write_reg(i2c_bus, 0x32, 0x70);
    write_reg(i2c_bus, 0x33, 0x12);
    write_reg(i2c_bus, 0x34, 0x0c);
    write_reg(i2c_bus, 0x3c, 0x30);
    write_reg(i2c_bus, 0xfe, 0x00);
}

static int gc0320_mclk_output(bool enable)
{
    struct rt_device_pwm *device = RT_NULL;
    int pwm_channel = 1;
#ifdef USE_PTM_DVP_8WIRE
    uint32_t period = 84 * 3; // 4MHz
#else
    uint32_t period = 84; // 12MHz
#endif
    device = (struct rt_device_pwm *)rt_device_find("pwmt2");
    if (!device)
    {
        LOG_I("find pwmt2 device failed");
        return -1;
    }

    if (!enable)
    {
        rt_pwm_disable(device, pwm_channel);
    }
    else
    {
        rt_pwm_set(device, pwm_channel, period, period / 2); // 50% duty cycle
        rt_pwm_enable(device, pwm_channel);
    }
    return 0;
}

int rt_gc032a_init(uint8_t interface)
{
    rt_uint16_t i = 0;
    rt_err_t result = RT_EOK;
    rt_device_t dcmi_dev = RT_NULL;

    LOG_I("rt_gc032a_init");
    i2c_bus = rt_i2c_bus_device_find(I2C_NAME);
    if (i2c_bus == RT_NULL)
    {
        LOG_E("can't find %s deivce", I2C_NAME);
        return RT_ERROR;
    }

    if (RT_EOK != rt_i2c_open(i2c_bus, RT_DEVICE_FLAG_RDWR))
    {
        LOG_E("can't open %s deivce", I2C_NAME);
        return RT_ERROR;
    }
    struct rt_i2c_configuration configuration =
    {
        .mode = 0,
        .addr = 0,
        .timeout = 500,
        .max_hz  = 100000,
    };
    rt_i2c_configure(i2c_bus, &configuration);

    gc0320_mclk_output(true);
    if (interface == 0)
    {
        gc032a_8bit_dvp_drv();
    }
    else if (interface == 1)
    {
        gc032a_2bit_spi_drv();
    }

    rt_i2c_close(i2c_bus);

    LOG_I("Wait for gc032a power on...");
    rt_thread_mdelay(2000); //Wait for stable
    LOG_I("rt_gc032a_init done");
    return RT_EOK;
}

int rt_gc032a_deinit(void)
{
    gc0320_mclk_output(false);
    return RT_EOK;
}

int camera_sample(int argc, char **argv)
{
    if (strcmp(argv[1], "spi") == 0)
    {
        LOG_I("gc032a init spi interface");
        rt_gc032a_init(1);
    }
    else if (strcmp(argv[1], "dvp") == 0)
    {
        LOG_I("gc032a init dvp interface");
        rt_gc032a_init(0);
    }

    return 0;
}
MSH_CMD_EXPORT(camera_sample, record picture to a jpg file);

#endif
