/**
  ******************************************************************************
  * @file   AD_touch.h
  * @author Sifli software development team
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

#ifndef AD_TOUCH_H
#define AD_TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lv_drv_conf.h"

#if USE_AD_TOUCH

#define  _SUPPRESS_PLIB_WARNING
#include <plib.h>

#include "GenericTypeDefs.h"

#include <stdint.h>
#include <stdbool.h>
#include "lvgl/lv_hal/lv_hal_indev.h"

#define DISP_ORIENTATION    0
#define DISP_HOR_RESOLUTION 320
#define DISP_VER_RESOLUTION 240

/*GetMaxX Macro*/
#if (DISP_ORIENTATION == 90) || (DISP_ORIENTATION == 270)
#define GetMaxX()   (DISP_VER_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 0) || (DISP_ORIENTATION == 180)
#define GetMaxX()   (DISP_HOR_RESOLUTION - 1)
#endif

/*GetMaxY Macro*/
#if (DISP_ORIENTATION == 90) || (DISP_ORIENTATION == 270)
#define GetMaxY()   (DISP_HOR_RESOLUTION - 1)
#elif (DISP_ORIENTATION == 0) || (DISP_ORIENTATION == 180)
#define GetMaxY()   (DISP_VER_RESOLUTION - 1)
#endif

/*********************************************************************
 * HARDWARE PROFILE FOR THE RESISTIVE TOUCHSCREEN
 *********************************************************************/

#define TOUCH_ADC_INPUT_SEL AD1CHS

// ADC Sample Start
#define TOUCH_ADC_START AD1CON1bits.SAMP

// ADC Status
#define TOUCH_ADC_DONE AD1CON1bits.DONE

#define RESISTIVETOUCH_ANALOG 1
#define RESISTIVETOUCH_DIGITAL 0

// ADC channel constants
#define ADC_XPOS ADC_CH0_POS_SAMPLEA_AN12
#define ADC_YPOS ADC_CH0_POS_SAMPLEA_AN13

// ADC Port Control Bits
#define ADPCFG_XPOS AD1PCFGbits.PCFG12 //XR
#define ADPCFG_YPOS AD1PCFGbits.PCFG13 //YD

// X port definitions
#define ResistiveTouchScreen_XPlus_Drive_High()     LATBbits.LATB12 = 1
#define ResistiveTouchScreen_XPlus_Drive_Low()      LATBbits.LATB12 = 0         //LAT_XPOS
#define ResistiveTouchScreen_XPlus_Config_As_Input()    TRISBbits.TRISB12 = 1 //TRIS_XPOS
#define ResistiveTouchScreen_XPlus_Config_As_Output()   TRISBbits.TRISB12 = 0

#define ResistiveTouchScreen_XMinus_Drive_High()    LATFbits.LATF0 = 1
#define ResistiveTouchScreen_XMinus_Drive_Low()     LATFbits.LATF0 = 0         //LAT_XNEG
#define ResistiveTouchScreen_XMinus_Config_As_Input()   TRISFbits.TRISF0 = 1 //TRIS_XNEG
#define ResistiveTouchScreen_XMinus_Config_As_Output()  TRISFbits.TRISF0 = 0

// Y port definitions
#define ResistiveTouchScreen_YPlus_Drive_High() LATBbits.LATB13 = 1
#define ResistiveTouchScreen_YPlus_Drive_Low()  LATBbits.LATB13 = 0         //LAT_YPOS
#define ResistiveTouchScreen_YPlus_Config_As_Input()    TRISBbits.TRISB13 = 1 //TRIS_YPOS
#define ResistiveTouchScreen_YPlus_Config_As_Output()   TRISBbits.TRISB13 = 0

#define ResistiveTouchScreen_YMinus_Drive_High() LATFbits.LATF1 = 1
#define ResistiveTouchScreen_YMinus_Drive_Low()     LATFbits.LATF1 = 0         //LAT_YNEG
#define ResistiveTouchScreen_YMinus_Config_As_Input()   TRISFbits.TRISF1 = 1 //TRIS_YNEG
#define ResistiveTouchScreen_YMinus_Config_As_Output()  TRISFbits.TRISF1 = 0

// Default calibration points
#define TOUCHCAL_ULX 0x0348
#define TOUCHCAL_ULY 0x00CC
#define TOUCHCAL_URX 0x00D2
#define TOUCHCAL_URY 0x00CE
#define TOUCHCAL_LLX 0x034D
#define TOUCHCAL_LLY 0x0335
#define TOUCHCAL_LRX 0x00D6
#define TOUCHCAL_LRY 0x032D

void ad_touch_init(void);
bool ad_touch_read(lv_indev_data_t *data);
int16_t ad_touch_handler(void);

#endif /* USE_AD_TOUCH */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AD_TOUCH_H */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/