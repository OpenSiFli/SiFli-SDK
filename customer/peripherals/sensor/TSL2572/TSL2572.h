#ifndef _TSL2572_H_
#define _TSL2572_H_

#include <stdint.h>

#define TSL25721_ID           (0x34)
#define TSL25723_ID           (0x3d)
#define TSL2572_ADDRESS       (0x39)

#define COMMAND_BIT           (0xA0)
//Register (0x00)
#define ENABLE_REGISTER       (0x00)
// FUNCTION BIT
#define ENABLE_POWERON        (0x01)
#define ENABLE_POWEROFF       (0x00)
#define ENABLE_AEN            (0x02)
#define ENABLE_WEN            (0x08)
#define ENABLE_AIEN           (0x10)
#define ENABLE_SAI            (0x40)
//#define ENABLE_NPIEN          (0x80)

#define CONTROL_REGISTER      (0x0f)
//#define SRESET                (0x80)
//AGAIN
#define LOW_AGAIN             (0X00)//Low gain (1x)
#define MEDIUM_AGAIN          (0X01)//Medium gain (8x)
#define HIGH_AGAIN            (0X02)//High gain (16x)
#define MAX_AGAIN             (0x03)//Max gain (120x)

#define ATIME_REGISTER        (0x01)
//ATIME
// atime(x) = ((256-(x))*2.73)ms
#define ATIME_699MS           (0x00)//699 millis    MAX COUNT 65535 
#define ATIME_175MS           (0xc0)//175 millis    MAX COUNT 65535 
#define ATIME_101MS           (0xdb)//300 millis    MAX COUNT 37888 
#define ATIME_27D3MS          (0xF6)//27.3 millis   MAX COUNT 10240 
#define ATIME_2D73MS          (0xFF)//2.73 millis   MAX COUNT 1024 

#define WTIME_REGISTER        (0x03)
// wtime(x) = ((256-(x))*2.73)ms , wait time

#define AILTL_REGISTER        (0x04)
#define AILTH_REGISTER        (0x05)
#define AIHTL_REGISTER        (0x06)
#define AIHTH_REGISTER        (0x07)
//#define NPAILTL_REGISTER      (0x08)
//#define NPAILTH_REGISTER      (0x09)
//#define NPAIHTL_REGISTER      (0x0A)
//#define NPAIHTH_REGISTER      (0x0B)

#define PERSIST_REGISTER      (0x0C)
// Bits 3:0
// 0000          Every ALS cycle generates an interrupt
// 0001          Any value outside of threshold range
// 0010          2 consecutive values out of range
// 0011          3 consecutive values out of range
// 0100          5 consecutive values out of range
// 0101          10 consecutive values out of range
// 0110          15 consecutive values out of range
// 0111          20 consecutive values out of range
// 1000          25 consecutive values out of range
// 1001          30 consecutive values out of range
// 1010          35 consecutive values out of range
// 1011          40 consecutive values out of range
// 1100          45 consecutive values out of range
// 1101          50 consecutive values out of range
// 1110          55 consecutive values out of range
// 1111          60 consecutive values out of range

#define CONFIG_REGISTER       (0x0D)
#define WLONG_BIT             (1<<1)    // wait long
#define AGL_BIT               (1<<2)    // ALS gain level

#define ID_REGISTER           (0x12)

#define STATUS_REGISTER       (0x13)
#define AVALID_BIT            (1<<0)    // als valid
#define AINT_BIT              (1<<4)    // als interrupt

#define CHAN0_LOW             (0x14)
#define CHAN0_HIGH            (0x15)
#define CHAN1_LOW             (0x16)
#define CHAN1_HIGH            (0x17)

// TODO: This value related to chip? (762.0) from TSL2591
//LUX_DF   GA * 60   GA is the Glass Attenuation factor
#define TSL_GA              (1.00)
#define LUX_DF                (TSL_GA*60)
// LUX_DF                408.0
#define MAX_COUNT_699MS           (65535)   //0xffff
#define MAX_COUNT_175MS           (65535)   //0xffff
#define MAX_COUNT_101MS           (37888)   //
#define MAX_COUNT_27D3MS          (10240)   //
#define MAX_COUNT_2D73MS          (1024)    //
/***********************************************************************************/
int TSL2572_Init(void);
uint32_t TSL2572_Read_Lux(void);
void TSL2572_SET_InterruptThreshold(uint16_t SET_LOW, uint16_t SET_HIGH);
void TSL2572_SET_LuxInterrupt(uint16_t SET_LOW, uint16_t SET_HIGH);
uint32_t TSL2572_Read_FullSpectrum(void);
uint16_t TSL2572_Read_Infrared(void);
uint32_t TSL2572_Read_Visible(void);

uint8_t TSL2572_Get_ID();
void *TSL2572_Get_Bus();

#endif  // _TSL2572_H_