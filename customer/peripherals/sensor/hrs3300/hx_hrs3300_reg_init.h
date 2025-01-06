#ifndef _HX_HRS3300_REG_INIT_H_
#define _HX_HRS3300_REG_INIT_H_
#include <stdint.h>

const uint8_t init_register_array[][2] =
{
//{0x01, 0xd0},   //11010000  bit[7]=1,HRS enable;bit[6:4]=101,wait time=50ms,bit[3]=0,LED DRIVE=22 mA
//{0x01, 0xf0},   //11010000  bit[7]=1,HRS enable;bit[6:4]=101,wait time=50ms,bit[3]=0,LED DRIVE=22 mA v13.05
    {0x0c, 0x4e},   //00001110  bit[6]=0,LED DRIVE=22mA;bit[5]=0,sleep mode;p_pulse=1110,duty=50%
    {0x16, 0x78},   //01111000  bits[7:4]=0111,HRS 15bits
    {0x17, 0x0d},   //00001101  bits[7:5]=011,HRS gain 16*;bit[1]=0,HRS data=AD0
    {0x02, 0x00},
    {0x03, 0x00},
    {0x04, 0x00},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x00},
    {0x0b, 0x00},
    {0x0c, 0x6e},
};
#define INIT_ARRAY_SIZE (sizeof(init_register_array)/sizeof(init_register_array[0]))


#endif   // _HRS3300_REG_INIT_H_
