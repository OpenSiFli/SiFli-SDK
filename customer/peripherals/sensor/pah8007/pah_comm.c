#include <rtthread.h>
#include "board.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
/*****************************
Customer Platform link Function
*****************************/
#include "pah_8007.h"

/*
void pah_print(
    int32_t                         fileline,
    const char                      *funcname,
    const char                      *filename,
    const char                      *format, ...)
{
    char dest[512];
    va_list argptr;
    int char_num = 0;
    if (!format)
        return;

    va_start(argptr, format);
    char_num = vsprintf(dest, format, argptr);
    va_end(argptr);

    pxi_nrf_uart_transmit((uint8_t*)dest, (uint16_t)(char_num ));
}
*/

void pah_delay_ms(uint32_t ms)
{
    /*
        customer need to fill in here with correct delay implementation of platform
        客户需要填入正确的平台延时实现
    */
    //rt_thread_mdelay(ms);
    HAL_Delay(ms);
}

uint64_t pah_get_tick_count(void)
{
#ifdef USE_TRUE_SYS_TICK
    uint64_t real_tick_count;

    /*
        custoemr need to use a real system tick implementation here to return tick count value in milliseconds unit
        ex: if you use STM32, you can call HAL_GetTick() to have tick count in milliseconds

        客户需要填入获取真实系统滴答的实现方式，以返回一个以毫秒为单位的滴答值
        示例：如果你使用的是STM32平台，可以调用HAL_GetTick()获取毫秒单位的滴答
    */
    //DEBUG_PRINT("real_tick_count = %llu \n", real_tick_count);
    extern uint32_t HAL_GetTick(void);
    real_tick_count = HAL_GetTick();

    return real_tick_count;
#else
    /* this is a simple method to get a pseudo "tick count", it's NOT ACCURATE, just for test, try it with precaution
       这是一个简单的方式获取伪滴答值，准确度较差，只是测试目的，请谨慎选用
    */

    static uint64_t fake_tick_count = 0;

    /* everytime call this interface, the fake_tick_count will increase exactly by 40, to emulate "40ms"
       polling duration, if you change this value to 30, it will emulate "30ms" polling duration

       每次调用此pah_get_tick_count()接口，fake_tick_count会固定累加40，以模拟40ms轮询周期，如果改写值为30，
       则变成模拟30ms轮询周期
    */
    fake_tick_count += 40;

    //DEBUG_PRINT("fake_tick_count = %llu \n", fake_tick_count);

    return fake_tick_count;
#endif
}

void pah8007_write_reg(uint8_t reg, uint8_t data);
void pah8007_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);

void pah_comm_i2c_write(uint8_t addr, const uint8_t *data)
{
    //DEBUG_PRINT("R:0x%02x,D:0x%02x\n",addr,*data);
    /*
        this is the interface of PAH8007 I2C communication, please fill in with I2C implementation
        of your platform, be noted with the data type of two parameters in this interface

        1st parameter, "addr" : this is the target address of register you want to write data into
        2nd parameter, "data" : this is the pointer which point to the data you want to write

        此函数为PAH8007 I2C通讯的接口，请用实际使用的平台的I2C实现填入此接口，注意接口的函数参数的类型
        第一个参数，"addr"：此为想要写入数据的目标寄存器的地址
        第二个参数，"data"：此为指针变量，是打算写入目标寄存器的数据变量的地址
    */

    pah8007_write_reg(addr, *data);

}

void pah_comm_i2c_read(uint8_t addr, uint8_t *data, size_t read_size)
{
    /*
        this is the interface of PAH8007 I2C communication, please fill in with I2C implementation
        of your platform, be noted with the data type of three parameters in this interface

        1st parameter, "addr" : this is the source address of register you want to read data from
        2nd parameter, "data" : this is the pointer which point to the target variable you want to store the readed data
        3rd parameter, "read_size" : this is the number of data's bytes you want to read

        此函数为PAH8007 I2C通讯的接口，请用实际使用的平台的I2C实现填入此接口，注意接口的函数参数的类型
        第一个参数，"addr"：此为想要读取数据的源寄存器的地址
        第二个参数，"data"：此为指针变量，是打算存储从源寄存器读到数据的变量的地址
        第三个参数，"read_size"：此为想要读取的数据字节个数
    */
    pah8007_read_reg(addr, data, read_size);
}
