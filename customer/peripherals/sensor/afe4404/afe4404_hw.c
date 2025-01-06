/**
  ******************************************************************************
  * @file   afe4404_hw.c
  * @author Sifli software development team
  * @brief   Control and operation functions for TI AFE 4404
*
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include "afe4404_hw.h"
//#include "mod_mcu_control_hw.h" // LifeQ specific hardware control
//#include "PROCESSOR/twi_hw.h" // LifeQ specific twi interface

#include <string.h>
//#include <delay.h>
#include <stdbool.h>
//#include <math.h>
#include <stdio.h>

//#include "hw_config.h"

//#include "i2c_sw.c"

volatile int hrvalue, hrskin;
#include <stdint.h>
//#include <platform_devices.h>
//#include "nordic_common.h"
//#include "nrf.h"
//#include "app_error.h"
//#include "boards.h"
//#include "softdevice_handler.h"
//#include "app_timer.h"
//#include "bsp.h"
#include "board.h"
#include "Mac_Hr.h"
#include "afe4404_hw.h"
#include "phys_calc.h"
#include "pp_config.h"
#include "agc_V3_1_19.h"
#include "pps960.h"
#include "level1metrics_calc.h"
//#include "demos/common.h"
//#define OUPUT_AFE_CLK
//#define DRV_DEBUG
#define LOG_TAG              "drv.afe"
#include <drv_log.h>


__weak void pps964a_t1_cb(uint32_t x)
{

}
#if 0
__weak int Partron_ALG(double LED1, double AccX, double AccY, double AccZ)
{
    return 0;
}
#endif


////#include "boards.h"
////#include "app_util_platform.h"
////#include "nrf_delay.h"
////#include "app_timer.h"
////#include "sdk_config.h"
////
//////#define NRF_LOG_MODULE_NAME "afe"
////#include "nrf_log.h"
////#include "nrf_log_ctrl.h"
////
////#include "my_debug.h"
////DEBUG_SET_LEVEL(DEBUG_LEVEL_TEST);

#define PPS_DEBUG(format, ...) // printf (format, ##__VA_ARGS__)


#define TRUE 1
#define FALSE 0

uint8_t green_led_on_flag = 0;

#define REGISTERS_TO_WRITE_LENGTH 20
uint32_t RegistersToWrite[REGISTERS_TO_WRITE_LENGTH];
uint32_t LocationsOfRegistersToWrite[REGISTERS_TO_WRITE_LENGTH];
uint8_t AmountOfRegistersToWrite = 0;

afe_data_struct_t afe_struct1;
afe_32_bit_data_struct_t afe_32bit_struct1;
afe_32_RAW_bit_data_struct_t afe_32bit_RAW_struct1;
afe_data_etc_t etcdata;


uint16_t LEDbuffHead = 0 ;
uint16_t LEDbuffTail = 0 ;

uint8_t afeSettingsExecuted = FALSE;


uint32_t liveAFERegister[80] ;
#define AFE_BUFF_LEN 60 //For LIFEQ
uint16_t LED1buffer[AFE_BUFF_LEN];
uint16_t LED2buffer[AFE_BUFF_LEN];
uint16_t LEDA1buffer[AFE_BUFF_LEN];
uint16_t LEDA2buffer[AFE_BUFF_LEN];

uint16_t AccXbuff[AFE_BUFF_LEN];
uint16_t AccYbuff[AFE_BUFF_LEN];
uint16_t AccZbuff[AFE_BUFF_LEN];
uint64_t AFE_SettingsBuff[AFE_BUFF_LEN];

uint64_t settingsCheckSampleQueue [2];

uint8_t firstsample = TRUE;
uint16_t HRM_sample_counts = 0;
uint8_t GledOn = 0, GledOff = 0;
uint8_t cnt_y = 0, cnt_n = 0;

static uint8_t detectChangeInAfeSettings(uint64_t settings);
#if 1//def PPS960_DEBUG
    uint16_t AFE4404_getRf_Partron(uint8_t rfNum);
    int8_t AFE4404_getAmbientCur_Partron(uint8_t channel);
    uint8_t afe4404_getLedCur_Partron(uint8_t channel);
#endif
#if 0
uint32_t afe4404_init_registers [60] =
{
    0x00, //0
    20, //0,
    450,
    1000,
    1450,
    520, //500,
    950,
    1020, //1000,
    1450,
    0,
    450,
    1520,  //1500,
    1950,
    507,
    999,
    1007,
    1499,
    1507,
    1999,
    2007,
    2499,
    500,
    506,
    1000,
    1006,
    1500,
    1506,
    2000,
    2006,
    19999, //Register 29 - PRP
    0x000100, //30 - NUMAV - Number of averages <3:0>, TIMEREN <8> -> start the internal timing/state machine
    0x00, //31 - NFI
    0x008002, //32 - EN_SEP_GAIN <15> , TIA_GAIN_SEP <2:0>, TIA_CF_SEP <5:3> , 0x08002 is 5pF and 100kOhm
    0x000002, //33
    0x00003f, //34 - ILED1<5:0> , ILED2 <11:6> , ILED3 <17:12>
    0x020200, //35 - OSC_ENABLE <9> , DYNAMIC4<3> , DYNAMIC3<4> , DYNAMIC2<14> , DYNAMIC1<20>  ----  0x104018 are the dynamic power down bits
    0x00, //36 -
    0x00, //37 -
    0x00, //38 - D16 - enable_filter_short , 0x10026
    0x00, //39 -
    0xC00000, //40  bits 22 and 23 set "to enhance performance of current silicon version"
    0x00, //41
    0x00, //42
    0x00, //43
    0x00, //44
    0x00, //45
    0x00, //46
    0x00, //47
    0x00, //48
    0x00, //49
    2600, //50
    19500, //51
    0x00, //52
    0x00, //53
    500, //54
    950, //55
    0x00, //56
    0x000005, //57
    0x000000, //58 All Isub values set to 0uA
    0x00 //59
};
#endif
const uint32_t init_registers[86] =
{

    /*//25Hz LED on 50us
        0x01,0x00000A,
        0x02,0x000031,
        0x03,0x000152,
        0x04,0x000183,
        0x05,0x0000B3,
        0x06,0x0000DA,
        0x07,0x00015C,
        0x08,0x000183,
        0x09,0x000000,
        0x0A,0x000031,
        0x0B,0x000205,
        0x0C,0x00022C,
        0x0D,0x000035,
        0x0E,0x0000A7,
        0x0F,0x0000DE,
        0x10,0x000150,
        0x11,0x000187,
        0x12,0x0001F9,
        0x13,0x000230,
        0x14,0x0002A2,
        0x15,0x000033,
        0x16,0x000034,
        0x17,0x0000DC,
        0x18,0x0000DD,
        0x19,0x000185,
        0x1A,0x000186,
        0x1B,0x00022E,
        0x1C,0x00022F,
        0x1D,0x009C3F,
        0x1E,0x000100,
        0x20,0x008002,
        0x21,0x000002,
        0x22,0x000000,//
        0x23,0x124218,
        0x29,0x000200,
        //0x31,0x000020,
        0x32,0x00036A,
        0x33,0x009B77,
        0x36,0x0000A9,
        0x37,0x0000DA,
        0x39,0x000005,
        0x3A,0x000000,
        0x3D,0x000000,
        0x3D,0x000000,
    */

// 100us
    0x01, 0x000014,
    0x02, 0x000063,
    0x03, 0x0001B6,
    0x04, 0x000219,
    0x05, 0x0000EF,
    0x06, 0x00013E,
    0x07, 0x0001CA,
    0x08, 0x000219,
    0x09, 0x000000,
    0x0A, 0x000063,
    0x0B, 0x0002A5,
    0x0C, 0x0002F4,
    0x0D, 0x000067,
    0x0E, 0x0000D9,
    0x0F, 0x000142,
    0x10, 0x0001B4,
    0x11, 0x00021D,
    0x12, 0x00028F,
    0x13, 0x0002F8,
    0x14, 0x00036A,
    0x15, 0x000065,
    0x16, 0x000066,
    0x17, 0x000140,
    0x18, 0x000141,
    0x19, 0x00021B,
    0x1A, 0x00021C,
    0x1B, 0x0002F6,
    0x1C, 0x0002F7,
    0x1D, 0x009C3F,
    0x1E, 0x000100,
    0x20, 0x008002,
    0x21, 0x000002,
    0x22, 0x000500,
    0x23, 0x124218,
    0x29, 0x000200,
    0x32, 0x000432,
    0x33, 0x009B77,
    0x36, 0x0000DB,
    0x37, 0x00013E,
    0x39, 0x000005,
    0x3A, 0x000000,
    //0x31,0x000020,
    0x3D, 0x000000,
    0x3D, 0x000000,

    /*
    //200us
        0x01,0x000014,
        0x02,0x0000C7,
        0x03,0x00027E,
        0x04,0x000345,
        0x05,0x000153,
        0x06,0x000206,
        0x07,0x000292,
        0x08,0x000345,
        0x09,0x000000,
        0x0A,0x0000C7,
        0x0B,0x0003D1,
        0x0C,0x000484,
        0x0D,0x0000CB,
        0x0E,0x00013D,
        0x0F,0x00020A,
        0x10,0x00027C,
        0x11,0x000349,
        0x12,0x0003BB,
        0x13,0x000488,
        0x14,0x0004FA,
        0x15,0x0000C9,
        0x16,0x0000CA,
        0x17,0x000208,
        0x18,0x000209,
        0x19,0x000347,
        0x1A,0x000348,
        0x1B,0x000486,
        0x1C,0x000487,
        0x1D,0x009C3F,
        0x1E,0x000100,
        0x20,0x008006,
        0x21,0x000006,
        0x22,0x000000,
        0x23,0x124218,
        0x29,0x000200,
        //0x31,0x000020,
        0x32,0x0005C2,
        0x33,0x009B77,
        0x36,0x00013F,
        0x37,0x000206,
        0x39,0x000005,
        0x3A,0x000000,
        0x3D,0x000000,
        0x3D,0x000000,
        */

};

void PPS960_init(void)
{
    //PPS_DEBUG("init 0\n");

    AFE4404_enableRead();

#ifdef LIFEQ
    //Initialize The LifeQ Algorithm
    ////hqret_t r1 = PP_Set_Defaults();
    ////hqret_t r2 = PP_Set_Gs( DEVICE_CONFIG_ACC_4G );
    ////hqret_t r3 = PP_Set_Device_AFE( AFE_TI4404 );

    ////hqret_t r4 = PP_Set_SkinDetect( OFF );

    //////PP_Set_SkinDetectThreshold(7000, 70000);
    //////PP_Set_SkinDetectACThreshold();
    ////hqret_t r5 = PP_Init( DEVICE_DATA_INPUT_FS_25HZ );
    //////hqret_t r6 = PP_Set_Peak_Initialisation(ON);
    ////hqret_t r6 = PP_Set_ActivityMode( PP_CONFIG_AUTO );

    ////hqret_t r7 = PP_Reset();
    //printf("r1 r2 r3 r4 r5 r6 r7:%d %d %d %d %d %d %d\r\n",r1,r2,r3,r4,r5,r6,r7);

#endif
    AGC_InitializeAGC();
    //Partron_UpdateSamplingRate(25,1.25);
    //   OS_DELAY_MS(100);
    //AFE4404_setMaxCurrentMode(1);//TRUE

    settingsCheckSampleQueue[0] = 0;
    settingsCheckSampleQueue[1] = 0;
    green_led_on_flag = 0;
    GledOn = 0;
    GledOff = 0;
    cnt_y = 0;
    cnt_n = 0;




}


uint16_t pps_getHR(void)
{
#ifdef LIFEQ
    //return PP_GetLifeQ_HR();
    //return PP_Get_HR();
    return hrvalue;
#else
    return hrvalue;

#endif
    return 0;
}

int Max(uint32_t a, uint32_t b)
{

    if (a > b)
        return a;
    else
        return b;
}

int Min(uint32_t a, uint32_t b)
{
    if (a < b)
        return a;
    else
        return b;
}




/*************************/
uint32_t LED2STC_M;
uint32_t LED2ENDC_M;
uint32_t LED1LEDSTC_M;
uint32_t LED1LEDENDC_M;
uint32_t ALED2STC_M;
uint32_t ALED2ENDC_M;
uint32_t LED1STC_M;
uint32_t LED1ENDC_M;
uint32_t LED2LEDSTC_M;
uint32_t LED2LEDENDC_M;
uint32_t ALED1STC_M;
uint32_t ALED1ENDC_M;
uint32_t LED2CONVST_M;
uint32_t LED2CONVEND_M;
uint32_t ALED2CONVST_M;
uint32_t ALED2CONVEND_M;
uint32_t LED1CONVST_M;
uint32_t LED1CONVEND_M;
uint32_t ALED1CONVST_M;
uint32_t ALED1CONVEND_M;
uint32_t ADCRSTSTCT0_M;
uint32_t ADCRSTENDCT0_M;
uint32_t ADCRSTSTCT1_M;
uint32_t ADCRSTENDCT1_M;
uint32_t ADCRSTSTCT2_M;
uint32_t ADCRSTENDCT2_M;
uint32_t ADCRSTSTCT3_M;
uint32_t ADCRSTENDCT3_M;
uint32_t PRPCOUNT_M;
uint32_t LED3LEDSTC_M;
uint32_t LED3LEDENDC_M;
uint32_t PDNCYCLESTC_M;
uint32_t PDNCYCLEENDC_M;

/***************************/

void Partron_UpdateSamplingRate(uint16_t freq, double duty)
{
    uint32_t tarr[60] = {0};                              // temporary array representing AFE settings registers
    uint32_t CLK_FREQ = 4000000;
    uint8_t clkdiv = 4;
    uint8_t NumAV = 1;

    LED2LEDSTC_M = 0;
    LED2LEDENDC_M = (uint32_t)(((duty / 100) * CLK_FREQ) / (clkdiv * freq) - 1);
    LED2STC_M = (LED2LEDSTC_M + Min((uint32_t)(80 / clkdiv), (uint32_t)(0.2 * (LED2LEDENDC_M - LED2LEDSTC_M + 1))));
    LED2ENDC_M = LED2LEDENDC_M;
    ADCRSTSTCT0_M = (LED2ENDC_M + 2);
    ADCRSTENDCT0_M = (ADCRSTSTCT0_M + Max((uint32_t)(6 / clkdiv), 1));
    LED2CONVST_M = (ADCRSTENDCT0_M + 1);
    LED2CONVEND_M = (LED2CONVST_M - 1 + (uint32_t)(Max(LED2LEDENDC_M, (NumAV + 2) * 50 * (4 / clkdiv) + (60 / clkdiv))));
    LED3LEDSTC_M = (LED2CONVEND_M + 2);
    LED3LEDENDC_M = (LED3LEDSTC_M + LED2LEDENDC_M);
    ALED2STC_M = (LED3LEDSTC_M + Min((uint32_t)(80 / clkdiv), (uint32_t)(0.2 * (LED3LEDENDC_M - LED3LEDSTC_M + 1))));
    ALED2ENDC_M = LED3LEDENDC_M;
    ADCRSTSTCT1_M = (ALED2ENDC_M + 2);
    ADCRSTENDCT1_M = (ADCRSTSTCT1_M + Max((uint32_t)(6 / clkdiv), 1));
    ALED2CONVST_M = (ADCRSTENDCT1_M + 1);
    ALED2CONVEND_M = (ALED2CONVST_M - 1 + (uint32_t)(Max(LED2LEDENDC_M, (NumAV + 2) * 50 * (4 / clkdiv) + (60 / clkdiv))));
    LED1LEDSTC_M = (ALED2CONVEND_M + 2);
    LED1LEDENDC_M = (LED1LEDSTC_M + LED2LEDENDC_M);
    LED1STC_M = (LED1LEDSTC_M + Min((uint32_t)(80 / clkdiv), (uint32_t)(0.2 * (LED1LEDENDC_M - LED1LEDSTC_M + 1))));
    LED1ENDC_M = LED1LEDENDC_M;
    ADCRSTSTCT2_M = (LED1ENDC_M + 2);
    ADCRSTENDCT2_M = (ADCRSTSTCT2_M + Max((uint32_t)(6 / clkdiv), 1));
    LED1CONVST_M = (ADCRSTENDCT2_M + 1);
    LED1CONVEND_M = (LED1CONVST_M - 1 + (uint32_t)(Max(LED2LEDENDC_M, (NumAV + 2) * 50 * (4 / clkdiv) + (60 / clkdiv))));
    ALED1STC_M = (LED1CONVEND_M + 2 + Min((uint32_t)(80 / clkdiv), (uint32_t)(0.2 * (LED2LEDENDC_M - LED2LEDSTC_M + 1))));
    ALED1ENDC_M = (ALED1STC_M + LED2ENDC_M - LED2STC_M);
    ADCRSTSTCT3_M = (ALED1ENDC_M + 2);

    ADCRSTENDCT3_M = (ADCRSTSTCT3_M + Max((uint32_t)(6 / clkdiv), 1));
    ALED1CONVST_M = (ADCRSTENDCT3_M + 1);
    ALED1CONVEND_M = (ALED1CONVST_M - 1 + (uint32_t)(Max(LED2LEDENDC_M, (NumAV + 2) * 50 * (4 / clkdiv) + (60 / clkdiv))));
    PDNCYCLESTC_M = ((uint32_t)(ALED1CONVEND_M + 200 * 0.000001 * (CLK_FREQ / clkdiv)));
    PDNCYCLEENDC_M = (uint32_t)((CLK_FREQ / (clkdiv * freq)) - 1 - (200 * 0.000001 * (CLK_FREQ / clkdiv)));
    PRPCOUNT_M = (((CLK_FREQ / freq) / clkdiv) - 1);

    AFE4404_disableAFE();
    /*
    some points to note
     - stop sampling slightly before end of 'sample slot'
     - suggested adc reset is 6 pulses, hence 6 and 7 offset of reset and conversion
     - counts starts at 0, so -1 at end for correct amount of counts
     - powerdown cycle to start >50us after last action, stop >200us before end of prp
    */

    tarr[1] = LED2STC_M;// sample led 2 start count
    tarr[2] = LED2ENDC_M;                // sample led 2 end count
    tarr[3] = LED1LEDSTC_M;                   // led 1 led start count
    tarr[4] = LED1LEDENDC_M;             // led 1 led end count
    tarr[5] = ALED2STC_M;                     // sample ambient led 2 start count
    tarr[6] = ALED2ENDC_M;               // sample ambient led 2 end count
    tarr[7] = LED1STC_M;                      // sample led 1 start count
    tarr[8] = LED1ENDC_M;                // sample led 1 end count
    tarr[9] =  LED2LEDSTC_M;                   // led 2 led start count
    tarr[10] = LED2LEDENDC_M; // led 2 led end count
    tarr[11] = ALED1STC_M;                    // sample ambient led 1 start count
    tarr[12] = ALED1ENDC_M;              // sample ambient led 1 end count
    tarr[13] = LED2CONVST_M;              // led 2 convert start count
    tarr[14] = LED2CONVEND_M;             // led 2 convert end count
    tarr[15] = ALED2CONVST_M;             // led 2 ambient convert start count
    tarr[16] = ALED2CONVEND_M;            // led 2 ambient convert end count
    tarr[17] = LED1CONVST_M;              // led 1 convert start count
    tarr[18] = LED1CONVEND_M;             // led 1 convert end count
    tarr[19] = ALED1CONVST_M;             // led 1 ambient convert start count
    tarr[20] = ALED1CONVEND_M;            // led 1 ambient convert end count
    tarr[21] = ADCRSTSTCT0_M;                     // adc reset 0 start count
    tarr[22] = ADCRSTENDCT0_M;                 // adc reset 0 end count
    tarr[23] = ADCRSTSTCT1_M;                     // adc reset 1 start count
    tarr[24] = ADCRSTENDCT1_M;                 // adc reset 1 end count
    tarr[25] = ADCRSTSTCT2_M;                     // adc reset 2 start count
    tarr[26] = ADCRSTENDCT2_M;                 // adc reset 2 end count
    tarr[27] = ADCRSTSTCT3_M;                     // adc reset 3 start count
    tarr[28] = ADCRSTENDCT3_M;                 // adc reset 3 end count
    tarr[29] = PRPCOUNT_M;       // pulse repetition period
    tarr[54] = LED3LEDSTC_M;                  // led 3 start
    tarr[55] = LED3LEDENDC_M;            // led 3 end
    tarr[50] = PDNCYCLESTC_M;    // power down cycle start
    tarr[51] = PDNCYCLEENDC_M;              // power down cycle end

    AFE4404_enableWrite();



    for (uint8_t i = 0; i < 60; i++)
    {
        if (tarr[i] != 0)
        {
            LIFEQTWI_writeReg(i, tarr[i]);

            //zPrintf(USART1,"address : %x, data : %x \n",i, tarr[i]);

        }
    }

    AFE4404_enableRead();
    AFE4404_enableInternalTimer();
}
//Calling this function shuts down AFE, sets new timing and starts device up again
/* NOTE!!  duty here refers to amount of microseconds for each 'sample slot' */
void AFE4404_UpdateSamplingRate(uint16_t freq, uint16_t duty, uint16_t clkdiv)
{
    uint32_t tarr[60] = {0};                              // temporary array representing AFE settings registers
    //uint8_t  clkdiv = 4;                            // 4mhz div 4 for afe timing
    uint8_t  ppu = 4 / clkdiv;                      // pulses per microsecond (us)
    uint16_t pps = ppu * duty;                      // pulses per slot - how many pulses each 'sample slot' is, given freq and duty
    AFE4404_disableAFE();
    /*
    some points to note
     - stop sampling slightly before end of 'sample slot'
     - suggested adc reset is 6 pulses, hence 6 and 7 offset of reset and conversion
     - counts starts at 0, so -1 at end for correct amount of counts
     - powerdown cycle to start >50us after last action, stop >200us before end of prp
    */

    tarr[1] = (LED2STC * pps);                      // sample led 2 start count
    tarr[2] = (LED2ENDC * pps) - 50;                // sample led 2 end count
    tarr[3] = (LED1LEDSTC * pps);                   // led 1 led start count
    tarr[4] = (LED1LEDENDC * pps) - 50;             // led 1 led end count
    tarr[5] = (ALED2STC * pps);                     // sample ambient led 2 start count
    tarr[6] = (ALED2ENDC * pps) - 50;               // sample ambient led 2 end count
    tarr[7] = (LED1STC * pps);                      // sample led 1 start count
    tarr[8] = (LED1ENDC * pps) - 50;                // sample led 1 end count
    tarr[9] = (LED2LEDSTC * pps);                   // led 2 led start count
    tarr[10] = (LED2LEDENDC * pps) - 50;            // led 2 led end count
    tarr[11] = (ALED1STC * pps);                    // sample ambient led 1 start count
    tarr[12] = (ALED1ENDC * pps) - 50;              // sample ambient led 1 end count
    tarr[13] = (LED2CONVST * pps) + 7;              // led 2 convert start count
    tarr[14] = (LED2CONVEND * pps) - 1;             // led 2 convert end count
    tarr[15] = (ALED2CONVST * pps) + 7;             // led 2 ambient convert start count
    tarr[16] = (ALED2CONVEND * pps) - 1;            // led 2 ambient convert end count
    tarr[17] = (LED1CONVST * pps) + 7;              // led 1 convert start count
    tarr[18] = (LED1CONVEND * pps) - 1;             // led 1 convert end count
    tarr[19] = (ALED1CONVST * pps) + 7;             // led 1 ambient convert start count
    tarr[20] = (ALED1CONVEND * pps) - 1;            // led 1 ambient convert end count
    tarr[21] = (ADCRST0 * pps);                     // adc reset 0 start count
    tarr[22] = (ADCRST0 * pps) + 6;                 // adc reset 0 end count
    tarr[23] = (ADCRST1 * pps);                     // adc reset 1 start count
    tarr[24] = (ADCRST1 * pps) + 6;                 // adc reset 1 end count
    tarr[25] = (ADCRST2 * pps);                     // adc reset 2 start count
    tarr[26] = (ADCRST2 * pps) + 6;                 // adc reset 2 end count
    tarr[27] = (ADCRST3 * pps);                     // adc reset 3 start count
    tarr[28] = (ADCRST3 * pps) + 6;                 // adc reset 3 end count
    tarr[29] = (4000000 / clkdiv / freq) - 1;       // pulse repetition period
    tarr[54] = (LED3LEDSTC * pps);                  // led 3 start
    tarr[55] = (LED3LEDENDC * pps) - 50;            // led 3 end
    tarr[50] = (PDNCYCLESTC * pps) + (ppu * 55);     // power down cycle start
    tarr[51] = tarr[29] - (ppu * 220);              // power down cycle end

    tarr[41] = (tarr[41] & 0xFFFFFFE1) | (2 << 1);     // clkdiv for internal timing - clkdiv of 4 requires the value '2' in 41[1:4]



    for (uint8_t i = 0; i < 60; i++)
    {
        if (tarr[i] != 0)
        {
            LIFEQTWI_writeReg(i, tarr[i]);

//          convert32to8(tarr[i],pdata_AFE4404);
        }
    }
    AFE4404_enableInternalTimer();
}



/*
   some points to note
    - stop sampling slightly before end of 'sample slot'
    - suggested adc reset is 6 pulses, hence 6 and 7 offset of reset and conversion
    - counts starts at 0, so -1 at end for correct amount of counts
    - powerdown cycle to start >50us after last action, stop >200us before end of prp
   */
/*
    // clip numAve to 7
    if (numAve > 7)
    {
        return HQ_ERR_DATA_OUT_OF_RANGE;
        numAve = 7;
    }

    uint32_t tarr[60] = {0};                                                     // temporary array representing AFE settings registers
    uint8_t  clkdiv = 4;                                                            // 4mhz div 4 for afe timing
    uint8_t  ppu = 4 / clkdiv;                                                   // pulses per microsecond (us)
    uint16_t pps = ppu * samplePeriod;                                   // pulses per sample slot - how many pulses each 'sample slot' is, given freq and duty
    uint16_t ppc = 0;                                                             // as above for convert
    uint8_t adcMhz = 4;                                                         // adc clock
    uint16_t convduty = ((numAve + 2) * 200 / adcMhz) + 15; // time req for conversion
    uint8_t count = 0;

    // check if convduty shorter than duty
    if (convduty < samplePeriod)
    {
        convduty = samplePeriod;
    }
    // set ppc
    ppc = ppu * convduty;

    for(count = 0; count < sizeof(afe4404_init_registers) / sizeof(afe4404_init_registers[0]); count++)
    {
        liveAFERegister[count] = afe4404_init_registers[count];
    }

    #define LED2SAMP        0
    #define ALED2SAMP       1
    #define LED1SAMP        2
    #define ALED1SAMP       3

    #define LED2CONV        LED2SAMP
    #define ALED2CONV       ALED2SAMP
    #define LED1CONV        LED1SAMP
    #define ALED1CONV       ALED1SAMP

    #define ADCRST0         0
    #define ADCRST1         1
    #define ADCRST2         2
    #define ADCRST3         3

    #define DPDSLEEP        200
    #define DPDWAKE         200

    #define LEDSETTLE       20
    #define LEDEND          2
    #define CNVEND          2
    #define RSTWIDTH        6
    #define RSTTAIL         4

    tarr[1] = (LED2SAMP * pps) + LEDSETTLE;                     // sample led 2 start count
    tarr[2] = ((LED2SAMP + 1) * pps) - LEDEND;                  // sample led 2 end count
    tarr[3] = (LED1SAMP * pps);                                         // led 1 led start count
    tarr[4] = ((LED1SAMP + 1) * pps) - LEDEND;                  // led 1 led end count
    tarr[5] = (ALED2SAMP * pps) + LEDSETTLE;                   // sample ambient led 2 start count
    tarr[6] = ((ALED2SAMP + 1) * pps) - LEDEND;                // sample ambient led 2 end count
    tarr[7] = (LED1SAMP * pps) + LEDSETTLE;                     // sample led 1 start count
    tarr[8] = ((LED1SAMP + 1) * pps) - LEDEND;                  // sample led 1 end count
    tarr[9] = (LED2SAMP * pps);                                         // led 2 led start count
    tarr[10] = ((LED2SAMP + 1) * pps) - LEDEND;                // led 2 led end count
    tarr[11] = (ALED1SAMP * pps) + LEDSETTLE;                 // sample ambient led 1 start count
    tarr[12] = ((ALED1SAMP + 1) * pps) - LEDEND;              // sample ambient led 1 end count
    tarr[13] = pps + (LED2CONV * ppc) + RSTWIDTH + RSTTAIL;    // led 2 convert start count
    tarr[14] = pps + ((LED2CONV + 1) * ppc) - CNVEND;      // led 2 convert end count
    tarr[15] = pps + (ALED2CONV * ppc) + RSTWIDTH + RSTTAIL;  // led 2 ambient convert start count
    tarr[16] = pps + ((ALED2CONV + 1) * ppc) - CNVEND;    // led 2 ambient convert end count
    tarr[17] = pps + (LED1CONV * ppc) + RSTWIDTH + RSTTAIL;    // led 1 convert start count
    tarr[18] = pps + ((LED1CONV + 1) * ppc) - CNVEND;      // led 1 convert end count
    tarr[19] = pps + (ALED1CONV * ppc) + RSTWIDTH + RSTTAIL;  // led 1 ambient convert start count
    tarr[20] = pps + ((ALED1CONV + 1) * ppc) - CNVEND;    // led 1 ambient convert end count
    tarr[21] = pps + (ADCRST0 * ppc);                                // adc reset 0 start count
    tarr[22] = pps + (ADCRST0 * ppc) + RSTWIDTH;            // adc reset 0 end count
    tarr[23] = pps + (ADCRST1 * ppc);                                // adc reset 1 start count
    tarr[24] = pps + (ADCRST1 * ppc) + RSTWIDTH;            // adc reset 1 end count
    tarr[25] = pps + (ADCRST2 * ppc);                                // adc reset 2 start count
    tarr[26] = pps + (ADCRST2 * ppc) + RSTWIDTH;            // adc reset 2 end count
    tarr[27] = pps + (ADCRST3 * ppc);                                // adc reset 3 start count
    tarr[28] = pps + (ADCRST3 * ppc) + RSTWIDTH;            // adc reset 3 end count
    tarr[29] = ((4000000 * 10) / clkdiv / freqTimesTen) - 1;   // pulse repetition period
    tarr[54] = (ALED2SAMP * pps);                                      // led 3 start
    tarr[55] = ((ALED2SAMP + 1) * pps) - LEDEND;               // led 3 end
    tarr[50] = pps + ((ALED1CONV + 1) * ppc) + DPDSLEEP; // power down cycle start
    tarr[51] = tarr[29] - DPDWAKE;                                      // power down cycle end
    tarr[82] = tarr[50]; // start ofADC ready
    tarr[83] = tarr[50] + 30; // end ofADC ready

    // check if prp enough for all samples
    if (!(tarr[20] < tarr[29]))
    {
        return RET_FAIL;
    }

    // check if freq (prp) valid range
    if (!(tarr[29] < 64000))
    {
        return RET_FAIL;
    }

    // if dpd time too little, disregard
    if (tarr[51] <= tarr[50])
    {
        allowDPD = false;
    }

    // turn on AFE
    AFE4404_hwPowerOn();
    AFE4404_enableWrite();

    uint8_t tempCount = 0;
    for(tempCount = 1; tempCount < 30; tempCount++)
    {
        AFE4404_writeRegWithoutWriteEnable(tempCount, tarr[tempCount]);
    }
    AFE4404_writeRegWithoutWriteEnable(54, tarr[54]); //LED3 START
    AFE4404_writeRegWithoutWriteEnable(55, tarr[55]); //LED3 END

    AFE4404_writeRegWithoutWriteEnable(32, afe4404_init_registers[32]); // Bit 15 enables separate gains Thus | 0x8000 enables separate gains
    AFE4404_writeRegWithoutWriteEnable(33, (afe4404_init_registers[33] | 0x00008000)); // | 0x00008000 is to enable diff gains for each channel
    AFE4404_writeRegWithoutWriteEnable(58, afe4404_init_registers[58]); //AFE4405_writeRegWithoutWriteEnable(58, 0x100000);
    AFE4404_writeRegWithoutWriteEnable(34, afe4404_init_registers[34]); //Led currents
//  AFE4405_writeRegWithoutWriteEnable(35, AFE4405_init_registers[35]); //0x12421C - 0x200 Osc mode
    AFE4404_writeRegWithoutWriteEnable(39, afe4404_init_registers[39]); //Added in an attempt to do osc calibration
    AFE4404_writeRegWithoutWriteEnable(56, afe4404_init_registers[56]); //Added in an attempt to do osc calibration
    AFE4404_writeRegWithoutWriteEnable(40, afe4404_init_registers[40]); //Added on TI's request. "To enhance ADC performance"
    AFE4404_writeRegWithoutWriteEnable(57, afe4404_init_registers[57]); // clkdiv prp

    AFE4404_writeRegWithoutWriteEnable(52, afe4404_init_registers[52]); // If forced ADC RDY timing is used, this sets the start time
    AFE4404_writeRegWithoutWriteEnable(53, afe4404_init_registers[53]); // If forced ADC RDY timing is used, this sets the end time

    if (allowDPD)
    {
        AFE4404_writeRegWithoutWriteEnable(50, tarr[50]);
        AFE4404_writeRegWithoutWriteEnable(51, tarr[51]);
    }
    else
    {
        AFE4404_writeRegWithoutWriteEnable(50, 63000); // move power down out of prp
        AFE4404_writeRegWithoutWriteEnable(51, 63001);
    }

    if(useExternalClock)
    {
        AFE4404_writeRegWithoutWriteEnable(35, afe4404_init_registers[35] & 0xFFFFDFF); //Same as init reg, except we clear bit 9, which sets AFE to use external clock
        AFE4404_writeRegWithoutWriteEnable(49, 0x05);
    }
    else
    {
        AFE4404_writeRegWithoutWriteEnable(35, afe4404_init_registers[35]); //0x12421C - 0x200 Osc mode
    }
    //Write ADC ready start and end
    AFE4404_writeRegWithoutWriteEnable(82, tarr[82]);
    AFE4404_writeRegWithoutWriteEnable(83, tarr[83]);

    AFE4404_writeRegWithoutWriteEnable(30, afe4404_init_registers[30] | (uint32_t)(numAve)); //Timer Enable<8> , ADC averages <3:0>

    AFE4404_enableInternalTimer();
    return RET_OK;
}
*/



void AFE4404_enableInternalTimer(void)
{
    uint32_t reg0x1E = AFE4404_getRegisterFromLiveArray(0x1E) | 0x000100;
    AFE4404_writeRegWithWriteEnable(0x1E, reg0x1E);   // Timer Enable <8>
}


//Disable the internal state machine timer
void AFE4404_disableAFE(void)
{
    AFE4404_writeRegWithoutWriteEnable(30, 0x000000);   //Timer Enable<8> , ADC averages <3:0>
}


void AFE4404_retrieveRawAFEValues(afe_32_RAW_bit_data_struct_t *afe_RAW_32bitstruct, afe_32_bit_data_struct_t *afe_32bitstruct, afe_data_struct_t *afe_struct)
{
    uint32_t tempVal = 0;
    tempVal = AFE4404_readRegWithoutReadEnable(44);
    //tempVal = AFE4404_readRegWithoutReadEnable(0x40);
    afe_RAW_32bitstruct->afeMeasLED1 = (int32_t) tempVal;
    afe_32bitstruct->afeMeasLED1 = afeConvP32(tempVal);
    afe_struct->afeMeasLED1 = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(45);
    //tempVal=0;
    afe_RAW_32bitstruct->afeMeasALED1 = (int32_t) tempVal;
    afe_32bitstruct->afeMeasALED1 = afeConvP32(tempVal);
    afe_struct->afeMeasALED1 =  afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(42);
    afe_RAW_32bitstruct->afeMeasLED2 = (int32_t) tempVal;
    afe_32bitstruct->afeMeasLED2 = afeConvP32(tempVal);
    afe_struct->afeMeasLED2 = afeConvP16(tempVal);
#if ENABLE_RED_LED  //cole add for disable RED LED
    tempVal = AFE4404_readRegWithoutReadEnable(43);
#endif
    afe_RAW_32bitstruct->afeMeasALED2 = (int32_t) tempVal;
    afe_32bitstruct->afeMeasALED2 = afeConvP32(tempVal);
    afe_struct->afeMeasALED2 = afeConvP16(tempVal);



}

#if 0
void AFE4404_retrieve32BitAFEValues(afe_32_bit_data_struct_t *afe_32bitstruct, afe_data_struct_t *afe_struct)
{
    uint32_t tempVal = 0;
    tempVal = AFE4404_readRegWithoutReadEnable(44);
    afe_32bitstruct->afeMeasLED1 = afeConvP32(tempVal);
    afe_struct->afeMeasLED1 = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(45);
    afe_32bitstruct->afeMeasALED1 =  afeConvP32(tempVal);
    afe_struct->afeMeasALED1 =  afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(42);
    afe_32bitstruct->afeMeasLED2 = afeConvP32(tempVal);
    afe_struct->afeMeasLED2 = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(43);
    afe_32bitstruct->afeMeasALED2 = afeConvP32(tempVal);
    afe_struct->afeMeasALED2 = afeConvP16(tempVal);
}


void AFE4404_retrieve16BitAFEValues(afe_data_struct_t *afe_struct)
{
    uint32_t tempVal = 0;
    tempVal = AFE4404_readRegWithoutReadEnable(44);
    afe_struct->afeMeasLED1 = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(45);
    afe_struct->afeMeasALED1 =  afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(42);
    afe_struct->afeMeasLED2 = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(43);
    afe_struct->afeMeasALED2 = afeConvP16(tempVal);
}
#endif
/*
void AFE4404_retrievePpgSample (sensor_ppg_sample_t *sampleStruct)
{
    uint32_t tempVal = 0;
    tempVal = AFE4404_readRegWithoutReadEnable(44);
    sampleStruct->greenPpg = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(45);
    sampleStruct->blankPpg = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(42);
    sampleStruct->redPpg = afeConvP16(tempVal);

    tempVal = AFE4404_readRegWithoutReadEnable(43);
    sampleStruct->irLed = afeConvP16(tempVal);

    sampleStruct->settingsVal = AFE4404_getAFESettingsUint();
}
*/

int32_t afeConvP32(int32_t val)
{
    int32_t ConvertTemp = 0;
    ConvertTemp = (val << 8);
    ConvertTemp = ConvertTemp / 256;

    return (int32_t)ConvertTemp;
}


uint16_t afeConvP16(int32_t val)
{
    int32_t ConvertTemp = 0;
    uint8_t checkTemp = (val >> 21);
    //If the 3 MSBs are 0b000 or 0b111 , the captured ADC value is within range
    if ((checkTemp == 7) || (checkTemp == 0))
    {
        ConvertTemp = (val << 8);
        ConvertTemp = ConvertTemp / 16384;
        ConvertTemp = ConvertTemp + 32768;
    }
    //If these bits are not 0b000 or 0b111, the ADC captured an invalid value. Thus now clip the return value
    else
    {
        // 0b001 Positive and higher than full scale , return max uint16
        if (checkTemp == 1)
        {
            ConvertTemp = 0xFFFF;
        }
        //Negative and lower than full scale , return 0
        else
        {
            ConvertTemp = 0;
        }
    }
    return (uint16_t)ConvertTemp;
}


void AFE4404_enableRead(void)
{

    //set_target_address(0x58);
    PPS960_writeReg(0, 1);
}


uint32_t AFE4404_readRegWithReadEnable(uint8_t reg)
{
    AFE4404_enableRead();
    return (LIFEQTWI_readReg(reg));
}


uint32_t AFE4404_readRegWithoutReadEnable(uint8_t reg)
{
    return (LIFEQTWI_readReg(reg));
}


void AFE4404_enableWrite(void)
{
    //set_target_address(0x58);
    PPS960_writeReg(0, 0);
}


void AFE4404_writeRegWithWriteEnable(uint8_t reg, uint32_t registerValue)
{
    AFE4404_enableWrite();
    LIFEQTWI_writeReg(reg, registerValue);
    liveAFERegister[reg] = registerValue;
}


void AFE4404_writeRegWithoutWriteEnable(uint8_t reg, uint32_t registerValue)
{
    LIFEQTWI_writeReg(reg, registerValue);
    liveAFERegister[reg] = registerValue;
}


hqret_t AFE4404_setRf(uint8_t rfNum, uint16_t rfValueInKiloOhms)
{
    uint32_t readValue = 0;
    uint8_t registerNum = 0;
    if (rfNum == 1)
        registerNum = 33;
    else if (rfNum == 2)
        registerNum = 32;
    else
        return RET_FAIL;

    switch (rfValueInKiloOhms)
    {
    case 500:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0000; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 250:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0001; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 100:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0002; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 50:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0003; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 25:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0004; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 10:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0005; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 1000:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0006; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 2000:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0007; //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    default:
        return RET_FAIL;
        break;
    }
    return RET_OK;
}


hqret_t AFE4404_directSetRf(uint8_t rfNum, uint16_t rfValueInKiloOhms)
{
    uint32_t readValue = 0;
    uint8_t registerNum = 0;
    if (rfNum == 1)
        registerNum = 33;
    else if (rfNum == 2)
        registerNum = 32;
    else
        return RET_FAIL;

    switch (rfValueInKiloOhms)
    {
    case 500:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0000; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 250:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0001; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 100:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0002; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 50:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0003; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 25:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0004; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 10:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0005; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 1000:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0006; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 2000:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFF8; //
        readValue = readValue | 0x0007; //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    default:
        return RET_FAIL;
        break;
    }
    return RET_OK;
}


uint16_t AFE4404_getRf(uint8_t rfNum)
{
    uint16_t possibleRf[9] = {500, 250, 100, 50, 25, 10, 1000, 2000};
    uint32_t readValue = 0;
    if (rfNum == 1)
    {
        readValue = AFE4404_getRegisterFromLiveArray(33);
        readValue = readValue & 0x07;
        return possibleRf[readValue];
    }
    else if (rfNum == 2)
    {
        readValue = AFE4404_getRegisterFromLiveArray(32);
        readValue = readValue & 0x07;
        return possibleRf[readValue];
    }
    else
    {
        return 0; //ERROR STATE
    }
}


hqret_t AFE4404_incrementRf(uint8_t channel)
{
    if ((channel == 0) || (channel > 2))
    {
        return RET_FAIL;
    }
    uint16_t possibleRf[9] = {10, 25, 50, 100, 250, 500, 1000, 2000};
    uint16_t tempRf = AFE4404_getRf(channel);
    uint8_t count = 0;
    for (count = 0; count < 9; count++)
    {
        if (possibleRf[count] == tempRf)
        {
            break;
        }
    }
    if (count == 7)
    {
        return RET_FAIL;
    }
    else
    {
        AFE4404_setRf(channel, possibleRf[count + 1]);
    }
    return RET_OK;
}


hqret_t AFE4404_decrementRf(uint8_t channel)
{
    if ((channel == 0) || (channel > 2))
    {
        return RET_FAIL;
    }
    uint16_t possibleRf[9] = {10, 25, 50, 100, 250, 500, 1000, 2000};
    uint16_t tempRf = AFE4404_getRf(channel);
    uint8_t count = 0;
    for (count = 0; count < 9; count++)
    {
        if (possibleRf[count] == tempRf)
        {
            break;
        }
    }
    if (count == 0)
    {
        return RET_FAIL;
    }
    else
    {
        AFE4404_setRf(channel, possibleRf[count - 1]);
    }
    return RET_OK;
}

hqret_t AFE4404_directSetCf(uint8_t cfNum, uint8_t cfValueInPicoFarhadsTimes10)
{
    uint32_t readValue = 0;
    uint8_t registerNum = 0;
    if (cfNum == 1)
        registerNum = 33;
    else if (cfNum == 2)
        registerNum = 32;
    else
        return RET_FAIL;

    switch (cfValueInPicoFarhadsTimes10)
    {
    case 50:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0000 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 25:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0001 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 100:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0002 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 75:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0003 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 200:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0004 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 175:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0005 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 250:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0006 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    case 225:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0007 << 3);   //
        AFE4404_writeRegWithWriteEnable(registerNum, readValue);
        break;
    default:
        return RET_FAIL;
        break;
    }
    return RET_OK;
}

hqret_t AFE4404_setCf(uint8_t cfNum, uint8_t cfValueInPicoFarhadsTimes10)
{
    uint32_t readValue = 0;
    uint8_t registerNum = 0;
    if (cfNum == 1)
        registerNum = 33;
    else if (cfNum == 2)
        registerNum = 32;
    else
        return RET_FAIL;

    switch (cfValueInPicoFarhadsTimes10)
    {
    case 50:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0000 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 25:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0001 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 100:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0002 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;

    case 75:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0003 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 200:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0004 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 175:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0005 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 250:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0006 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    case 225:
        readValue = AFE4404_getRegisterFromLiveArray(registerNum);
        readValue = readValue & 0xFFC7; //
        readValue = readValue | (0x0007 << 3);   //
        AFE4404_addWriteCommandToQueue(registerNum, readValue);
        break;
    default:
        return RET_FAIL;
        break;
    }
    return RET_OK;
}


uint8_t AFE4404_getCf(uint8_t cfNum)
{
    uint8_t possibleCf[9] = {50, 25, 100, 75, 200, 175, 250, 225};
    uint32_t readValue = 0;
    if (cfNum == 1)
    {
        readValue = AFE4404_getRegisterFromLiveArray(33);
        readValue = readValue & 0x38;
        readValue = (readValue >> 3);
        return possibleCf[readValue];
    }
    else if (cfNum == 2)
    {
        readValue = AFE4404_getRegisterFromLiveArray(32);
        readValue = readValue & 0x38;
        readValue = (readValue >> 3);
        return possibleCf[readValue];
    }
    else
    {
        return 0; // ERROR STATE
    }
}


// Channels are : LED1 , LED1 A,LED2 ,  LED2 A
hqret_t AFE4404_setAmbientCurrent(uint8_t channel, int8_t currentInMicroAmpsTimes10)
{
    uint8_t currentPolarityNegative = FALSE;
    if (currentInMicroAmpsTimes10 < 0)
    {
        currentPolarityNegative = TRUE;
        currentInMicroAmpsTimes10 = -currentInMicroAmpsTimes10;
    }

    uint8_t count = 0;
    uint8_t validCurrents[17] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    uint8_t validCurrentPosition = 0;
    uint8_t ValidSetCurrent = FALSE;
    for (count = 0; count < 17; count++)
    {
        if (validCurrents[count] == currentInMicroAmpsTimes10)
        {
            ValidSetCurrent = TRUE;
            validCurrentPosition = count;
            break;
        }
    }
    if (!ValidSetCurrent)
    {
        return RET_FAIL; // Selected Current out of bounds
    }

    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(58);
    uint32_t FiveBitsToWrite = ((uint32_t)validCurrentPosition);
    if (currentPolarityNegative)
    {
        FiveBitsToWrite = FiveBitsToWrite | 0x10; // Set bit for negative Isub
    }
    FiveBitsToWrite = FiveBitsToWrite & 0x0000001F; // Only allow 5 bits
    switch (channel)
    {
    case 4: // LED 2 A
        readRegister = readRegister & 0xFFFFFFE0; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 0;
        break;
    case 1:
        readRegister = readRegister & 0xFFFFFC1F; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 5;
        break;
    case 2:
        readRegister = readRegister & 0xFFFF83FF; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 10;
        break;
    case 3:
        readRegister = readRegister & 0xFFF07FFF; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 15;
        break;
    default:
        return RET_FAIL;
        break;
    }
    AFE4404_addWriteCommandToQueue(58, readRegister);
    return RET_OK;
}


// Channels are : LED1 , LED1 A,LED2 ,  LED2 A
hqret_t AFE4404_directSetAmbientCurrent(uint8_t channel, int8_t currentInMicroAmpsTimes10)
{
    uint8_t currentPolarityNegative = FALSE;
    if (currentInMicroAmpsTimes10 < 0)
    {
        currentPolarityNegative = TRUE;
        currentInMicroAmpsTimes10 = -currentInMicroAmpsTimes10;
    }

    uint8_t count = 0;
    uint8_t validCurrents[17] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    uint8_t validCurrentPosition = 0;
    uint8_t ValidSetCurrent = FALSE;
    for (count = 0; count < 17; count++)
    {
        if (validCurrents[count] == currentInMicroAmpsTimes10)
        {
            ValidSetCurrent = TRUE;
            validCurrentPosition = count;
            break;
        }
    }
    if (!ValidSetCurrent)
    {
        return RET_FAIL; // Selected Current out of bounds
    }

    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(58);
    uint32_t FiveBitsToWrite = ((uint32_t)validCurrentPosition);
    if (currentPolarityNegative)
    {
        FiveBitsToWrite = FiveBitsToWrite | 0x10; // Set bit for negative Isub
    }
    FiveBitsToWrite = FiveBitsToWrite & 0x0000001F; // Only allow 5 bits
    switch (channel)
    {
    case 4: // LED 2 A
        readRegister = readRegister & 0xFFFFFFE0; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 0;
        break;
    case 1:
        readRegister = readRegister & 0xFFFFFC1F; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 5;
        break;
    case 2:
        readRegister = readRegister & 0xFFFF83FF; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 10;
        break;
    case 3:
        readRegister = readRegister & 0xFFF07FFF; // Clear 5 bits where Isub will be stored
        readRegister = readRegister | FiveBitsToWrite << 15;
        break;
    default:
        return RET_FAIL;
        break;
    }
    AFE4404_writeRegWithWriteEnable(58, readRegister);
    return RET_OK;
}


// Channels are : LED1 , LED1 A ,LED2 , LED2 A
int8_t AFE4404_getAmbientCurrent(uint8_t channel)
{
    if ((channel > 4) || (channel == 0))
    {
        return 0xFF; // Selected channel out of bounds
    }
    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(58);
    int8_t retrievedCurrent = 0x0;
    uint8_t retrievedCurrentWithoutSign = 0x0;
    uint8_t validCurrents[17] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    switch (channel)
    {
    case 4:
        retrievedCurrentWithoutSign = readRegister & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x10) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 1:
        retrievedCurrentWithoutSign = (readRegister >> 5) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x200) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 2:
        retrievedCurrentWithoutSign = (readRegister >> 10) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x4000) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 3:
        retrievedCurrentWithoutSign = (readRegister >> 15) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x80000) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    default:
        return 0xFF;
        break;
    }
}


hqret_t AFE4404_incrementAmbientSubCurrent(uint8_t channel)
{
    if ((channel == 0) || (channel > 4))
    {
        return RET_FAIL;
    }
    int8_t current = AFE4404_getAmbientCurrent(channel);
    if (current == -75)
    {
        return RET_FAIL;
    }
    else
    {
        AFE4404_setAmbientCurrent(channel, (current - 5));
    }
    return RET_OK;
}


hqret_t AFE4404_decrementAmbientSubCurrent(uint8_t channel)
{
    if ((channel == 0) || (channel > 4))
    {
        return RET_FAIL;
    }
    int8_t current = AFE4404_getAmbientCurrent(channel);
    if (current == 75)
    {
        return RET_FAIL;
    }
    else
    {
        AFE4404_setAmbientCurrent(channel, (current + 5));
    }
    return RET_OK;
}


uint8_t AFE4404_isIsubValid(int8_t currentInMicroAmpsTimes10)
{
    if (currentInMicroAmpsTimes10 < 0)
    {
        currentInMicroAmpsTimes10 = -currentInMicroAmpsTimes10;
    }
    uint8_t count = 0;
    uint8_t validCurrents[17] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    for (count = 0; count < 17; count++)
    {
        if (validCurrents[count] == currentInMicroAmpsTimes10)
        {
            return RET_OK;
        }
    }
    return RET_FAIL;
}


hqret_t AFE4404_setLedCurrent(uint8_t ledNumber, uint8_t currentTapSetting)
{
    if (currentTapSetting > 63)
    {
        return RET_FAIL; // Selected current out of bounds
    }
    uint32_t readValue = AFE4404_getRegisterFromLiveArray(34);
    switch (ledNumber)
    {
    case 1:
        readValue = readValue & 0xFFFFC0;
        readValue = readValue | (uint32_t)(currentTapSetting);
        break;
    case 2:
        readValue = readValue & 0xFFF03F;
        readValue = readValue | (uint32_t)(currentTapSetting << 6);
        break;
    case 3:
        readValue = readValue & 0xFC0FFF;
        readValue = readValue | (uint32_t)(currentTapSetting << 12);
        break;
    default:
        return RET_FAIL;
        break;
    }
    AFE4404_addWriteCommandToQueue(34, readValue);
    return RET_OK;
}


hqret_t AFE4404_setLedCurrentWithDutyCycleLimitation(uint8_t ledNumber, uint8_t currentTapSetting)
{
    uint8_t hundredMiliAmpMode = FALSE;
    uint32_t readValue;
    uint8_t led1, led2, led3;
    uint16_t ledOnDuration, prpCount, maxLedCurrentDutyCycle;

    if (ledNumber > 3)
    {
        return RET_FAIL;
    }
    if (currentTapSetting > 63)
    {
        return RET_FAIL;
    }

    // First determine whether the device is in high current mode
    // D17 reg 35 set to 1 for 100mA mode
    readValue  = AFE4404_getRegisterFromLiveArray(35);
    if ((readValue & 0x20000) != 0)
    {
        hundredMiliAmpMode = TRUE;
    }
    // Now determine the led currents used atm
    readValue = AFE4404_getRegisterFromLiveArray(34);
    led1 = (uint8_t)(readValue & 0x3F);
    led2 = (uint8_t)((readValue >> 6) & 0x3F);
    led3 = (uint8_t)((readValue >> 12) & 0x3F);

    // Determine the led on duration
    ledOnDuration = (uint16_t)(AFE4404_getRegisterFromLiveArray(2));

    // Determine prp count
    prpCount = (uint16_t)(AFE4404_getRegisterFromLiveArray(29));
    prpCount = prpCount + 1; // To use this value to calculate duty cycle, we need to increment with one

    if (hundredMiliAmpMode)
        maxLedCurrentDutyCycle = (uint16_t)((float)prpCount * 0.03);
    else
        maxLedCurrentDutyCycle = (uint16_t)((float)prpCount * 0.1);

    maxLedCurrentDutyCycle = maxLedCurrentDutyCycle * 63; // This value now contains the maximum time * led current value to be within 3% or 10 %

    switch (ledNumber)
    {
    case 1:
    {
        // Check whether the led current setting chosen does exceed the total maximum
        if ((led2 * ledOnDuration) + (led3 * ledOnDuration) + (currentTapSetting * ledOnDuration) > maxLedCurrentDutyCycle)
        {
            return RET_FAIL;
        }
        return AFE4404_setLedCurrent(ledNumber, currentTapSetting);
    }
    break;
    case 2:
    {
        // Check whether the led current setting chosen does exceed the total maximum
        if ((led1 * ledOnDuration) + (led3 * ledOnDuration) + (currentTapSetting * ledOnDuration) > maxLedCurrentDutyCycle)
        {
            return RET_FAIL;
        }
        return AFE4404_setLedCurrent(ledNumber, currentTapSetting);
    }
    break;
    case 3:
    {
        // Check whether the led current setting chosen does exceed the total maximum
        if ((led2 * ledOnDuration) + (led1 * ledOnDuration) + (currentTapSetting * ledOnDuration) > maxLedCurrentDutyCycle)
        {
            return RET_FAIL;
        }
        return AFE4404_setLedCurrent(ledNumber, currentTapSetting);
    }
    break;
    default:
        return RET_FAIL;
        break;
    }
    return RET_FAIL;
}


hqret_t AFE4404_directSetLedCurrent(uint8_t ledNumber, uint8_t currentTapSetting)
{
    if (currentTapSetting > 63)
    {
        return RET_FAIL; // Selected current out of bounds
    }
    uint32_t readValue = AFE4404_getRegisterFromLiveArray(34);
    switch (ledNumber)
    {
    case 1:
        readValue = readValue & 0xFFFFC0;
        readValue = readValue | (uint32_t)(currentTapSetting);
        break;
    case 2:
        readValue = readValue & 0xFFF03F;
        readValue = readValue | (uint32_t)(currentTapSetting << 6);
        break;
    case 3:
        readValue = readValue & 0xFC0FFF;
        readValue = readValue | (uint32_t)(currentTapSetting << 12);
        break;
    default:
        return RET_FAIL;
        break;
    }
    AFE4404_writeRegWithWriteEnable(34, readValue);
    return RET_OK;
}


uint8_t AFE4404_getLedCurrent(uint8_t ledNumber)
{
    uint32_t readValue = AFE4404_getRegisterFromLiveArray(34);
    switch (ledNumber)
    {
    case 1:
        readValue = readValue & 0x3f;
        break;
    case 2:
        readValue = (readValue >> 6) & 0x3f;
        break;
    case 3:
        readValue = (readValue >> 12) & 0x3f;
        break;
    default:
        return 0xFF;
        break;
    }
    return (uint8_t)readValue;
}


hqret_t AFE4404_incrementLedCurrent(uint8_t ledNumber)
{
    if ((ledNumber == 0) || (ledNumber > 3))
    {
        return RET_FAIL;
    }
    uint8_t ledCurrent = AFE4404_getLedCurrent(ledNumber);
    if (ledCurrent == 63)
    {
        return RET_NO;
    }
    else
    {
        AFE4404_setLedCurrent(ledNumber, ledCurrent + 1);
    }
    return RET_OK;
}


hqret_t AFE4404_decrementLedCurrent(uint8_t ledNumber)
{
    if ((ledNumber == 0) || (ledNumber > 3))
    {
        return RET_FAIL;
    }
    uint8_t ledCurrent = AFE4404_getLedCurrent(ledNumber);
    if (ledCurrent == 0)
    {
        return RET_NO;
    }
    else
    {
        AFE4404_setLedCurrent(ledNumber, ledCurrent - 1);
    }
    return RET_OK;
}

hqret_t AFE4404_getMaxLedCurrent(uint8_t *maxCurrent)
{
    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(35);   //reg 0x23. LEDx4 is bit 18, LDE2X is bit 17

    if ((readRegister & 0x060000) == 0x000000)      //None set, 50mA mode
    {
        *maxCurrent = 50;
        return RET_OK;
    }
    else if ((readRegister & 0x060000) == 0x060000)        //Both set, 150mA mode
    {
        *maxCurrent = 150;
        return RET_OK;
    }
    else if ((readRegister & 0x060000) == 0x020000)        //Ledx2 set, 100mA mode
    {
        *maxCurrent = 100;
        return RET_OK;
    }
    else if ((readRegister & 0x060000) == 0x040000)        //led x 4 set, 200mA mode
    {
        *maxCurrent = 200;
        return RET_OK;
    }
    return RET_FAIL;
}

hqret_t AFE4404_setMaxLedCurrent(uint8_t maxCurrent)
{
    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(35);   //reg 0x23. LEDx4 is bit 18, LDE2X is bit 17
    readRegister = readRegister & ~0x060000; //clear bits 17 and 18
    switch (maxCurrent)
    {
    case 50:   //x2 = 0, x 4 = 0
    {
        //Both 0, register stays the same
    }
    break;
    case 100:   //x2 = 1, x4 = 0
    {
        readRegister = readRegister | 0x020000;
    }
    break;
    case 150:   //x2 = 1, x4 = 1
    {
        readRegister = readRegister | 0x060000;
    }
    break;
    case 200:   //x2 = 0, x4 = 1
    {
        readRegister = readRegister | 0x040000;
    }
    break;
    default:
        return RET_FAIL;
    }
    AFE4404_addWriteCommandToQueue(0x23, readRegister);
    return RET_OK;
}


void AFE4404_setMaxCurrentMode(uint8_t highPowerEnable)
{
    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(35);
    if (highPowerEnable)
    {
        AFE4404_addWriteCommandToQueue(35, (readRegister | 0x20200));
    }
    else
    {
        AFE4404_addWriteCommandToQueue(35, (readRegister & 0xFDFFFF));
    }
}


void AFE4404_addWriteCommandToQueue(uint8_t reg, uint32_t registerValue)
{
    LocationsOfRegistersToWrite[AmountOfRegistersToWrite] = reg;
    RegistersToWrite[AmountOfRegistersToWrite] = registerValue;
    liveAFERegister[reg] = registerValue;
    AmountOfRegistersToWrite++;
}


void AFE4404_serviceAFEWriteQueue(void)
{
    if (AmountOfRegistersToWrite > 0)
    {
        uint8_t count = 0;
        for (count = 0; count < REGISTERS_TO_WRITE_LENGTH; count++)
        {
            AFE4404_writeRegWithWriteEnable(LocationsOfRegistersToWrite[count], RegistersToWrite[count]);

            AmountOfRegistersToWrite--;
            if (AmountOfRegistersToWrite == 0)
            {
                break;
            }
        }
        afeSettingsExecuted = TRUE;
    }
}


uint32_t AFE4404_getRegisterFromLiveArray(uint8_t reg)
{
    return liveAFERegister[reg];
}


void AFE4404_getAFESettingsArr(uint8_t *dataArr)
{
    // Build a single 44 bit register that represents the following afe settings currently in use:
    // Rf1 & Rf2
    // Isub 1 to Isub4
    // Led currents
    uint64_t settingsRegister = 0;
    uint16_t tempArr [3] = {0};
    settingsRegister |= (uint64_t)(AFE4404_getLedCurrent(1) << 0);
    settingsRegister |= (uint64_t)(AFE4404_getLedCurrent(2) << 6);
    settingsRegister |= (uint64_t)(AFE4404_getLedCurrent(3) << 12);
    settingsRegister |= ((uint64_t)((AFE4404_getRegisterFromLiveArray(58) & 0xFFFFF)) << 18);
    settingsRegister |= ((uint64_t)(AFE4404_getRegisterFromLiveArray(33) & 0x07) << 38);
    settingsRegister |= ((uint64_t)(AFE4404_getRegisterFromLiveArray(32) & 0x07) << 41);

    tempArr[0] = (uint16_t)(settingsRegister & 0xFFFF);
    tempArr[1] = (uint16_t)((settingsRegister >> 16) & 0xFFFF);
    tempArr[2] = (uint16_t)((settingsRegister >> 32) & 0xFFFF);

    memcpy(&dataArr[0], &tempArr[0], 6);
}


uint64_t AFE4404_getAFESettingsUint(void)
{
    // Build a single 44 bit register that represents the following afe settings currently in use:
    // Rf1 & Rf2
    // Isub 1 to Isub4
    // Led currents
    uint64_t settingsRegister = 0;
    settingsRegister |= (uint64_t)(AFE4404_getLedCurrent(1) << 0);        // 6 bits
    settingsRegister |= (uint64_t)(AFE4404_getLedCurrent(2) << 6);        // 6 bits
    settingsRegister |= (uint64_t)(AFE4404_getLedCurrent(3) << 12);        // 6 bits
    settingsRegister |= ((uint64_t)((AFE4404_getRegisterFromLiveArray(58) & 0xFFFFF)) << 18);            // 20 bits
    settingsRegister |= ((uint64_t)(AFE4404_getRegisterFromLiveArray(33) & 0x07) << 38);         // 3 bits //RF1
    settingsRegister |= ((uint64_t)(AFE4404_getRegisterFromLiveArray(32) & 0x07) << 41);         // 3 bits //RF2
    return settingsRegister;
}


// Input uinit64 settings array, returns the Rf value
uint16_t AFE4404_settingsUintGetRf(uint8_t channel, uint64_t inputArr)
{
    uint8_t rfVal = 0;
    uint16_t possibleRf[8] = {500, 250, 100, 50, 25, 10, 1000, 2000};
    if (channel == 1)
    {
        rfVal = ((inputArr >> 38) & 0x07);
        return possibleRf[rfVal];
    }
    if (channel == 2)
    {
        rfVal = ((inputArr >> 41) & 0x07);
        return possibleRf[rfVal];
    }
    return 0;
}


int8_t AFE4404_settingsUintGetIsub(uint8_t channel, uint64_t inputArr)
{
    uint32_t readRegister = (inputArr >> 18);   // ReadRegister is now an exact copy of what was read from the AFE
    readRegister = readRegister & 0xFFFFF;
    int8_t retrievedCurrent = 0x0;
    uint8_t retrievedCurrentWithoutSign = 0x0;
    uint8_t validCurrents[16] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    switch (channel)
    {
    case 4: //RED
        retrievedCurrentWithoutSign = readRegister & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x10) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 1: //GREEN
        retrievedCurrentWithoutSign = (readRegister >> 5) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x200) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 2: // AMB
        retrievedCurrentWithoutSign = (readRegister >> 10) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x4000) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 3: //IR
        retrievedCurrentWithoutSign = (readRegister >> 15) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   // Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x80000) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    default:
        return 0xFF;
        break;
    }
}


uint8_t AFE4404_settingsUintGetLedCurrent(uint8_t ledNumber, uint64_t inputArr)
{
    uint32_t readValue = inputArr & 0x03FFFF;
    switch (ledNumber)
    {
    case 1:
        readValue = readValue & 0x3f;
        break;
    case 2:
        readValue = (readValue >> 6) & 0x3f;
        break;
    case 3:
        readValue = (readValue >> 12) & 0x3f;
        break;
    default:
        return 0xFF;
        break;
    }
    return (uint8_t)readValue;
}


uint8_t AFE4404_settingsChanged(void)
{
    if (afeSettingsExecuted)
    {
        afeSettingsExecuted = FALSE;
        return TRUE;
    }
    return FALSE;
}


hqret_t AFE4404_setNumberOfAverages(uint8_t numAvgs)
{
    if (numAvgs > 15)
    {
        return RET_FAIL;
    }
    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(30);
    readRegister = readRegister & 0xFFFFF0;
    readRegister = readRegister | ((uint32_t)numAvgs & 0x0f);
    AFE4404_addWriteCommandToQueue(30, readRegister);
    return RET_OK;
}


uint8_t AFE4404_getNumberOfAverages(void)
{
    uint32_t readRegister = AFE4404_getRegisterFromLiveArray(30);
    return (uint8_t)(readRegister & 0x0f);
}


hqret_t AFE4404_decrementNumberOfAverages(void)
{
    uint8_t retrievedAvg = AFE4404_getNumberOfAverages();
    retrievedAvg = retrievedAvg - 1;
    hqret_t ret = AFE4404_setNumberOfAverages(retrievedAvg);
    return ret;
}


hqret_t AFE4404_incrementNumberOfAverages(void)
{
    uint8_t retrievedAvg = AFE4404_getNumberOfAverages();
    retrievedAvg = retrievedAvg + 1;
    hqret_t ret = AFE4404_setNumberOfAverages(retrievedAvg);
    return ret;
}


hqret_t AFE4404_adjustPRPAndPowerDownCycles(uint16_t updatedPrpValue)
{
    AFE4404_addWriteCommandToQueue(29, updatedPrpValue);
    AFE4404_addWriteCommandToQueue(51, updatedPrpValue - DPDWAKE);
    return RET_OK;
}


uint32_t AFE4404_getPrpInUse(void)
{
    return (uint32_t)AFE4404_getRegisterFromLiveArray(29);
}

void LIFEQTWI_writeReg(uint8_t regaddr, uint32_t wdata)
{
    PPS960_writeReg(regaddr, wdata);
}

uint32_t LIFEQTWI_readReg(uint8_t regaddr)
{
    return PPS960_readReg(regaddr);
}



void ClrAccBuffer(void)
{
    //AccBuffTail = AccBuffHead;
}

//uint16_t GetAccBuffLenth(void)
//{
//       uint16_t AccBuffHeadLenth = 0;
//       if(AccBuffHead!=AccBuffTail)
//       {
//               if(AccBuffHead>AccBuffTail)
//               {
//                       AccBuffHeadLenth = AccBuffHead - AccBuffTail;
//               }else{
//                       AccBuffHeadLenth = AccBuffHead + AFE_BUFF_LEN - AccBuffTail;
//               }
//       }
//       return AccBuffHeadLenth;
//}
//uint16_t LEDbuffHead = 0 ;
//uint16_t LEDbuffTail = 0 ;
uint16_t GetLEDBuffLenth(void)
{
    uint16_t LEDBuffHeadLenth = 0;
    if (LEDbuffHead != LEDbuffTail)
    {
        if (LEDbuffHead > LEDbuffTail)
        {
            LEDBuffHeadLenth = LEDbuffHead - LEDbuffTail;
        }
        else
        {
            LEDBuffHeadLenth = LEDbuffHead + AFE_BUFF_LEN - LEDbuffTail;
        }
    }
    return LEDBuffHeadLenth;
}



void ALGSH_retrieveSamplesAndPushToQueue(void)
{
    static uint64_t retrievedAfeSettings = 0;


    AFE4404_retrieveRawAFEValues(&afe_32bit_RAW_struct1, &afe_32bit_struct1, &afe_struct1);   //Retrieve PPG samples from the AFE
    retrievedAfeSettings = AFE4404_getAFESettingsUint(); // Retrive AFE settings for the current sample set
    AFE4404_serviceAFEWriteQueue(); // Write command in queue to AFE
#if 0 //def PPS960_DEBUG
    uint16_t rf[2];
    int8_t amb_current[4];
    uint8_t led_current[3];
    uint8_t i = 0;
    for (i = 0; i < 2; i++)
        rf[i] = AFE4404_getRf_Partron(i + 1);
    for (i = 0; i < 4; i++)
        amb_current[i] = AFE4404_getAmbientCur_Partron(i + 1);
    for (i = 0; i < 3; i++)
        led_current[i] = afe4404_getLedCur_Partron(i + 1);

    MY_DEBUG_INFO(terminal_0, "%d %d %d %d %d %d %d %d %d %d\r\n", afe_struct1.afeMeasLED1, rf[0], rf[1], amb_current[0], amb_current[1], amb_current[2], amb_current[3], led_current[0], led_current[1], led_current[2]);
#endif
    //        afe4404_read_Acc(&specificForces); // Retrieve values from the accelerometer
    // printf("%d \n\r",LEDbuffHead);
    /*Circular buffer wrap around check */
    uint16_t tmphead = (LEDbuffHead + 1);   // next position in circular buffer

    if (tmphead >= AFE_BUFF_LEN)
    {
        tmphead = 0;
    }


    LED1buffer[tmphead] = afe_struct1.afeMeasLED1;
    LEDA1buffer[tmphead] = afe_struct1.afeMeasALED1;
    LED2buffer[tmphead] = afe_struct1.afeMeasLED2;
    LEDA2buffer[tmphead] = afe_struct1.afeMeasALED2;
//MY_DEBUG_INFO(terminal_0,"LED1 %d\r\n",LED1buffer[tmphead]);
//        AccXbuff[tmphead] = specificForces.accX;
//        AccYbuff[tmphead] = specificForces.accY;
//        AccZbuff[tmphead] = specificForces.accZ;

    AccXbuff[tmphead] = 32768;
    AccYbuff[tmphead] = 32768;
    AccZbuff[tmphead] = 32768;

    AFE_SettingsBuff[tmphead] = retrievedAfeSettings;
    LEDbuffHead = tmphead;


}


void ALGSH_dataToAlg()
{

    uint16_t tmptail;
    if (LEDbuffHead != LEDbuffTail)
    {
        tmptail = (LEDbuffTail + 1);


        //printf("tmptail");
        if (tmptail >= AFE_BUFF_LEN)
        {
            tmptail = 0;
        }

        static uint64_t lastAFEset = 0;

        if (firstsample)    // This skips the very first sample since it is 0's
        {
            lastAFEset = 0;
            firstsample = FALSE;
            LEDbuffTail = tmptail;// increase tail index
            return;
        }

        etcdata.tempIsubtimesTen1 = AFE4404_settingsUintGetIsub(1, lastAFEset);
        etcdata.tempIsubtimesTen2 = AFE4404_settingsUintGetIsub(2, lastAFEset);
        etcdata.tempIsubtimesTen3 = AFE4404_settingsUintGetIsub(3, lastAFEset);
        etcdata.tempIsubtimesTen4 = AFE4404_settingsUintGetIsub(4, lastAFEset);

        etcdata.tempRf1 = AFE4404_settingsUintGetRf(1, lastAFEset);
        etcdata.tempRf2 = AFE4404_settingsUintGetRf(2, lastAFEset);


        etcdata.tempcf1 = AFE4404_getCf(1);
        etcdata.tempcf2 = AFE4404_getCf(2);

        etcdata.ledCurrent1 = AFE4404_settingsUintGetLedCurrent(1, lastAFEset);
        etcdata.ledCurrent2 = AFE4404_settingsUintGetLedCurrent(2, lastAFEset);
        etcdata.ledCurrent3 = AFE4404_settingsUintGetLedCurrent(3, lastAFEset);



        alg_input_selection_t stateSelector = PP_HEART_RATE;
#if 0
        //NRF_LOG_RAW_INFO("LED1,LEDA1,LED2,RF1,RF2,Isub1,Isub2,Isub3,Cur1,Cur2: %d %d %d ",LED1buffer[LEDbuffTail],LEDA1buffer[LEDbuffTail],LED2buffer[LEDbuffTail]);
        //int lifeQhrm = pps_getHR();
        int snrValue = PP_GetHRConfidence(); //for snr check

        NRF_LOG_RAW_INFO("%d %d ",   lifeQhrm, snrValue);
        NRF_LOG_RAW_INFO("%d %d %d ",  AccXbuff[LEDbuffTail], AccYbuff[LEDbuffTail], AccZbuff[LEDbuffTail]);
        NRF_LOG_RAW_INFO("%d %d %d ",  LED1buffer[LEDbuffTail], LEDA1buffer[LEDbuffTail], LED2buffer[LEDbuffTail]);
        NRF_LOG_RAW_INFO("%d %d ",   etcdata.tempRf1, etcdata.tempRf2);
        NRF_LOG_RAW_INFO("%d %d %d ", etcdata.tempIsubtimesTen1, etcdata.tempIsubtimesTen2, etcdata.tempIsubtimesTen3);
        NRF_LOG_RAW_INFO("%d %d\r\n", etcdata.ledCurrent1, etcdata.ledCurrent2);

#endif
        //NRF_LOG_ERROR("LED1 %d\r\n",LED1buffer[LEDbuffTail]);

        // This function sends the data to the AGC
        agc_state_t agcState = AGC_ServiceAGC(LED1buffer[LEDbuffTail],
                                              LEDA1buffer[LEDbuffTail], LED2buffer[LEDbuffTail], LEDA2buffer[LEDbuffTail],
                                              detectChangeInAfeSettings(AFE_SettingsBuff[LEDbuffTail]));




        if ((agcState == AGC_STATE_CALIBRATION) || (agcState == AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT) || (agcState == AGC_STATE_CALIBRATION_ERRORSTATE))
        {
            stateSelector = PP_CALIBRATION;

//                        printf("AGC_isCalibrating\r\n");

        }
        else
        {
            stateSelector = PP_HEART_RATE;
//                        printf("PP_HEART_RATE\r\n");
            HRM_sample_counts++;

        }


        /*Populate LED samples structs */

        led_sample_t greenLed;
        led_sample_t blankLed;
        led_sample_t redLed;
        led_sample_t irLed;

        greenLed.sample = LED1buffer[LEDbuffTail];
        greenLed.led = LQ_LED_GREEN;
        greenLed.isub = etcdata.tempIsubtimesTen1;
        greenLed.led_curr_mAx10 = etcdata.ledCurrent1 * 16;
        greenLed.rf_kohm = etcdata.tempRf1;

        blankLed.sample = LEDA1buffer[LEDbuffTail];
        blankLed.led = LQ_LED_AMBIENT;
        blankLed.isub = etcdata.tempIsubtimesTen2;
        blankLed.led_curr_mAx10 = 0;
        blankLed.rf_kohm = etcdata.tempRf1;

        redLed.sample = LEDA2buffer[LEDbuffTail];
        redLed.led = LQ_LED_RED;
        redLed.isub = etcdata.tempIsubtimesTen4;
        redLed.led_curr_mAx10 = etcdata.ledCurrent3 * 16;
        redLed.rf_kohm = etcdata.tempRf2;

        irLed.sample = LED2buffer[LEDbuffTail];
        irLed.led = LQ_LED_INFRARED;
        irLed.isub = etcdata.tempIsubtimesTen3;
        irLed.led_curr_mAx10 = etcdata.ledCurrent2 * 16;
        irLed.rf_kohm = etcdata.tempRf2;
        pps964a_t1_cb(greenLed.sample);

//                       led_sample_t irLed;
//             irLed.sample = irLedBuf[LEDbuffTail];
//             irLed.led = LQ_LED_INFRARED;
//             irLed.isub = AFE4404_settingsUintGetIsub(3,AFE_SettingsBuff[LEDbuffTail] );//evdata.tempIsubtimesTen3;
//             irLed.led_curr_mAx10 = AFE4404_settingsUintGetLedCurrent(2,AFE_SettingsBuff[LEDbuffTail]) * 16;
//             irLed.rf_kohm = AFE4404_settingsUintGetRf(2,AFE_SettingsBuff[LEDbuffTail]);


        //int32_t tmpIrDC = ((0-irLed.isub)*65536/10+irLed.sample-32768)/100;
        //printf("rd,i,rf,cur:%d %d %d %d %d\r\n",irLed.sample,irLed.isub,irLed.rf_kohm,irLed.led_curr_mAx10,tmpIrDC);
        PPS_skin_detect(irLed);   //skin detect rawdata
        hrskin = PPS_get_skin_detect();
        PPS_Gled_on_by_skin_etect((uint8_t)hrskin);
        PPS_Gled_auto_ctrl(greenLed);
//===========================================================================================
//添加算法接口
//=============================================================================================
#ifdef LIFEQ
        //MY_DEBUG_INFO(terminal_0,"skin:%d---green_led_on_flag:%d\r\n",skin,green_led_on_flag);
        if (1 == hrskin && 3 == green_led_on_flag)    // skin detect control add data to algorithm
        {
            //static int dbg_flag = 0;
            // This function sends the data to the Alg
            //hqret_t algRet = PP_addSamplesTI4404( AccXbuff[LEDbuffTail], AccYbuff[LEDbuffTail], AccZbuff[LEDbuffTail],
            //                                      &greenLed, &blankLed, &redLed, &irLed, stateSelector );
            // WHJ: open it if algorithn lib add.
            hrvalue = Partron_ALG((float) greenLed.sample, 0, 0, 0);
            //dbg_flag++;
            //if ((dbg_flag & 0x3f) == 0) // output log for more than 10 second
            //    rt_kprintf("sample %8d , hr = %6d\n", greenLed.sample, hrvalue);
            //hrvalue = Partron_ALG(greenLed.sample,((double)AccXbuff[LEDbuffTail])/8192, ((double)AccYbuff[LEDbuffTail])/8192, ((double)AccZbuff[LEDbuffTail])/8192);
            //if( algRet != RET_OK ) {
            //printf("AddAfeSample returns ERROR \n");
            //}
        }//if(PPS_get_skin_detect())
#endif
        lastAFEset = AFE_SettingsBuff[LEDbuffTail];

        LEDbuffTail = tmptail;// increase tail index


    }

}

uint16_t GetHRSampleCount(void)
{
    return HRM_sample_counts;
}

void ClrHRSampleCount(void)
{
    HRM_sample_counts = 0;
}

uint32_t threshold_high = 1600; // high value - low value >32768/100+1
uint32_t threshold_low = 1200;
uint8_t is_skin = 0;
void PPS_set_skin_detect_thr(uint32_t high, uint32_t low)
{
    threshold_high = high;
    threshold_low = low;
    //is_skin=0;
}

void PPS_skin_detect(led_sample_t irLed)
{
    //irLed.sample = irLedBuf[tmptail];
    //irLed.led = LQ_LED_INFRARED;
    //irLed.isub = evdata.tempIsubtimesTen3;
    //irLed.led_curr_mAx10 = evdata.ledCurrent2 * 16;
    //irLed.rf_kohm = evdata.tempRf2;
    //Vtia(diff) = Vtia(+)-Vtia=2 * Iin * RF =>Iin = Vtia/2/Rf
    //1uA=2V/2/Rf/1000 000
    //1uA:400 0000 for ADC Raw data //1uA:56636 for ADC conver uint16
    //IrDc=irLed1.isub*65536/1uA+irLed1.sample-32768
    static int irLed_tmp = 0, old_isub;


    int32_t tempIrDC = ((0 - irLed.isub) * 65536 / 10 + irLed.sample - 32768) / 100;
    if (tempIrDC > threshold_high)    // threshold high value
    {
        cnt_y++;
        cnt_n = 0;
    }
    else if (tempIrDC < threshold_low)      // threshold low value
    {
        cnt_n++;
        cnt_y = 0;
    }
    if (cnt_y >= 10)    // 10 time to check
    {
        cnt_y = 10;
        is_skin = 1;
    }
    if (cnt_n >= 10)    // 10 time to check
    {
        cnt_n = 10;
        is_skin = 0;
    }

    if (irLed.sample == 65532 && irLed.isub == 0)
    {
        irLed_tmp++;
        if (irLed_tmp > 200)
        {
            AFE4404_writeRegWithWriteEnable(0x3a, 0xc6318);
            irLed_tmp = 0;
        }
    }
    else
    {
        irLed_tmp = 0;
        if (irLed.isub != old_isub)
        {
            old_isub = irLed.isub;
        }
    }


    int tmp_0x3a = AFE4404_readRegWithReadEnable(0x3a);
    //liveAFERegister[0x3a]
    LOG_D("rd,i,rf,cur:%d %d %d %d %d %d %x %x\r\n", irLed.sample, irLed.isub, irLed.rf_kohm, irLed.led_curr_mAx10, tempIrDC, is_skin, liveAFERegister[0x3a], tmp_0x3a);


}

uint8_t PPS_get_skin_detect(void)
{
    return is_skin;
}

void PPS_Gled_on_by_skin_etect(uint8_t onSkin)
{
    if (onSkin)
    {
        GledOn++;
        GledOff = 0;
    }
    else
    {
        GledOff++;
        GledOn = 0;
    }
    if (GledOn >= 1 * 25)    //1*25,25sample is one second,so it will delay 1 second to open green LED
    {
        //if(GledOn &&( 0 == green_led_on_flag)) { //1*25,25sample is one second,so it will delay 1 second to open green LED
        GledOn = 0;
        if (green_led_on_flag < 3)
        {
            //AFE4404_setLedCurrent(1,63);// Green  //DEFAULT_GREEN_CURRENT=55
            green_led_on_flag++;
        }
    }
    if (GledOff >= 25)    //3*25,25sample is one second,so it will delay 3 second to close green LED
    {
        GledOff = 0;
        if (green_led_on_flag > 1)
        {
            //AFE4404_setLedCurrent(1,0);// Green
            green_led_on_flag = 0;
        }
    }
}



#define GRN_LED_CTRL_CNT 5
void PPS_Gled_auto_ctrl(led_sample_t grnLed)
{
    static uint16_t GrnLedCnt = 0;
    uint16_t tmpCur = 0;
    if (grnLed.sample > 60000 && grnLed.isub == -75)
    {
        GrnLedCnt++;
        if (GrnLedCnt > GRN_LED_CTRL_CNT)
        {
            GrnLedCnt = 0;
            tmpCur = AFE4404_getLedCurrent(1);
            if (tmpCur > 32)
            {
                tmpCur -= 4;
                AFE4404_setLedCurrent(1, tmpCur);
            }
        }
    }
    else
    {
        GrnLedCnt = 0;
    }
}


static uint8_t detectChangeInAfeSettings(uint64_t settings)
{
    uint8_t retVal = FALSE;
    if (settingsCheckSampleQueue[0] != settingsCheckSampleQueue[1])
    {
        retVal = TRUE;
    }
    settingsCheckSampleQueue[1] = settingsCheckSampleQueue[0];
    settingsCheckSampleQueue[0] = settings;
    return retVal;
}
void init_PPS960_register(void)
{
    int i;
#if 1
    PPS_DEBUG("init_register\r\n");
    PPS960_writeReg(0x0, 0x0);
    for (i = 0; i < 86; i++, i++)
    {
        PPS960_writeReg(init_registers[i], init_registers[i + 1]);
        liveAFERegister[init_registers[i]] = init_registers[i + 1];
    }
    PPS960_writeReg(0x0, 0x1);

#else

    //set_target_address(0x58);
    for (i = 1; i < 29; i++)
    {
        PPS960_writeReg(i, afe4404_init_registers[i]);
    }
    PPS960_writeReg(29, afe4404_init_registers[29]);
    PPS960_writeReg(54, afe4404_init_registers[54]);
    PPS960_writeReg(55, afe4404_init_registers[55]);
    PPS960_writeReg(32, afe4404_init_registers[32]);
    PPS960_writeReg(33, afe4404_init_registers[33]);
    PPS960_writeReg(58, afe4404_init_registers[58]);
    PPS960_writeReg(34, afe4404_init_registers[34]);
    PPS960_writeReg(35, afe4404_init_registers[35]);
    PPS960_writeReg(39, afe4404_init_registers[39]);
    PPS960_writeReg(56, afe4404_init_registers[56]);
    PPS960_writeReg(40, afe4404_init_registers[40]);
    PPS960_writeReg(57, afe4404_init_registers[57]);
    PPS960_writeReg(50, afe4404_init_registers[50]);
    PPS960_writeReg(51, afe4404_init_registers[51]);
    PPS960_writeReg(30, afe4404_init_registers[30]);
#endif
}
#if 1//def PPS960_DEBUG
uint16_t AFE4404_getRf_Partron(uint8_t rfNum)
{
    uint16_t possibleRf[8] = {500, 250, 100, 50, 25, 10, 1000, 2000};
    uint32_t readValue = 0;

    AFE4404_enableRead();
    if (rfNum == 1)
    {
        readValue =  AFE4404_readRegWithoutReadEnable(33);
        readValue = readValue & 0x07;
        return possibleRf[readValue];
    }
    else if (rfNum == 2)
    {
        readValue = AFE4404_readRegWithoutReadEnable(32);
        readValue = readValue & 0x07;
        return possibleRf[readValue];
    }
    else
    {
        return 0; //ERROR STATE
    }
}


int8_t AFE4404_getAmbientCur_Partron(uint8_t channel)
{
    if ((channel > 4) || (channel == 0))
    {
        return 0xFF; //Selected channel out of bounds
    }
    AFE4404_enableRead();
    uint32_t readRegister = AFE4404_readRegWithoutReadEnable(58);
    int8_t retrievedCurrent = 0x0;
    uint8_t retrievedCurrentWithoutSign = 0x0;
    uint8_t validCurrents[16] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    switch (channel)
    {
    case 4:
        retrievedCurrentWithoutSign = readRegister & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   //Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x10) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 1:
        retrievedCurrentWithoutSign = (readRegister >> 5) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   //Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x200) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 2:
        retrievedCurrentWithoutSign = (readRegister >> 10) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   //Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x4000) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    case 3:
        retrievedCurrentWithoutSign = (readRegister >> 15) & 0x0F;
        retrievedCurrent = retrievedCurrent | (int8_t)validCurrents[retrievedCurrentWithoutSign];   //Combine value without sign and value which possibly contains a sign
        if ((readRegister & 0x80000) != 0x00)
        {
            retrievedCurrent = retrievedCurrent * -1;
        }
        return retrievedCurrent;
        break;
    default:
        return 0xFF;
        break;
    }
}


uint8_t afe4404_getLedCur_Partron(uint8_t channel)
{

    AFE4404_enableRead();
    uint32_t curvalue = AFE4404_readRegWithoutReadEnable(34);



    switch (channel)
    {
    case 1:
        curvalue = curvalue & 0x0000003f;
        break;
    case 2:
        curvalue = (curvalue >> 6) & 0x0000003f;
        break;
    case 3:
        curvalue = (curvalue >> 12) & 0x0000003f;
        break;
    default:
        return 0xFF;
        break;

    }

    return (uint8_t)curvalue;
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
