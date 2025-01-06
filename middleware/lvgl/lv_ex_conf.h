/**
  ******************************************************************************
  * @file   lv_ex_conf.h
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

#if 0 /*Set it to "1" to enable the content*/

    #ifndef LV_EX_CONF_H
        #define LV_EX_CONF_H

        /*******************
        * GENERAL SETTING
        *******************/
        #define LV_EX_PRINTF       0       /*Enable printf-ing data*/
        #define LV_EX_KEYBOARD     0       /*Add PC keyboard support to some examples (`lv_drivers` repository is required)*/
        #define LV_EX_MOUSEWHEEL   0       /*Add 'encoder' (mouse wheel) support to some examples (`lv_drivers` repository is required)*/

        /*******************
        *   TEST USAGE
        *******************/
        #define LV_USE_TESTS        0

        /*******************
        * TUTORIAL USAGE
        *******************/
        #define LV_USE_TUTORIALS   0


        /*********************
        * APPLICATION USAGE
        *********************/

        /* Test the graphical performance of your MCU
        * with different settings*/
        #define LV_USE_BENCHMARK   0

        /*A demo application with Keyboard, Text area, List and Chart
        * placed on Tab view */
        #define LV_USE_DEMO        0
        #if LV_USE_DEMO
            #define LV_DEMO_WALLPAPER  1    /*Create a wallpaper too*/
            #define LV_DEMO_SLIDE_SHOW 0    /*Automatically switch between tabs*/
        #endif

        /*MCU and memory usage monitoring*/
        #define LV_USE_SYSMON      0

        /*A terminal to display received characters*/
        #define LV_USE_TERMINAL    0

        /*Touch pad calibration with 4 points*/
        #define LV_USE_TPCAL       0

    #endif /*LV_EX_CONF_H*/

#endif /*End of "Content enable"*/

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
