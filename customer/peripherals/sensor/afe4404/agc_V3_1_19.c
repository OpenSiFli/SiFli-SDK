/**
  ******************************************************************************
  * @file   agc_V3_1_19.c
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

#define EMBEDDED
//#define POST

#include <string.h>
/*
#ifdef POST
#include "agc_V3.h"
#include "agc_interface.h"
#endif

#ifdef EMBEDDED
#include "AFE/afe_interface.h"
#include "AFE/AFE4404/agc_V3_1_19.h"
#include "UTILS/hqdbg.h"
#endif
*/

#include "afe4404_hw.h"
#include "agc_V3_1_19.h"

#define TRUE 1
#define FALSE 0


//Macros used in AGC
#ifndef AGC_max
    #define AGC_max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef AGC_min
    #define AGC_min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

#ifndef AGC_abs
    #define AGC_abs( a ) ( ((a) > 0) ? (a) : (-(a)) )
#endif


//Objective values in Volt Codes
#define OBJMAXVALLEDCONT ((int32_t) 7000000)
#define OBJMINVALLEDCONT ((int32_t) 6000000)

#define MAXVOLTCODELIMIT ((uint16_t) 55000)
#define MINVOLTCODELIMIT ((uint16_t) 10000)

#define SATURATELOW ((uint16_t) 1000)
#define SATURATEHIGH ((uint16_t) 64000)

//Objective values in Values
#define OBJVOLTAC1 ((int32_t) 100000000)
#define OBJVOLTAC2 ((int32_t) 100000000)

#define MAXVOLTCODELIMITFORRFADJUSTMENT ((int32_t) 50000)
#define MINVOLTCODELIMITFORRFADJUSTMENT ((int32_t) 15000)

#define FILTERSTABILIZATIONTIME ((uint8_t) 15)

//Default 50Hz Values
uint8_t ADJUSTMENTTIMEOUTLIMIT = 50;
uint8_t SAMPLESTOREEVALUATE = 100;
//uint8_t ADJUSTMENTTIMEOUTLIMIT = 25;
//uint8_t SAMPLESTOREEVALUATE = 50;

uint16_t DCEXPDECAYWEIGHTNUM = 50;
uint16_t DCEXPDECAYWEIGHTDENUM = 1000;
uint16_t ACEXPDECAYWEIGHTNUM = 20;
uint16_t ACEXPDECAYWEIGHTDENUM = 1000;

#define OFFOPTIONSLENGTH  (16)
#define AMPOPTIONSLENGTH (8)

int8_t OFFOPTIONS[OFFOPTIONSLENGTH] = {0, -5, -10, -15, -20, -25, -30, -35, -40, -45, -50, -55, -60, -65, -70, -75};
uint16_t AMPOPTIONS[AMPOPTIONSLENGTH] = {10, 25, 50, 100, 250, 500, 1000, 2000};

uint16_t AGC_InitAmpPreCalib1 = 250;
uint8_t AGC_InitAmpPosPreCalib1 = 4;
uint16_t AGC_InitAmpPreCalib2 = 250;
uint8_t AGC_InitAmpPosPreCalib2 = 4;

uint16_t AGC_InitAmpPostCalib1 = 500;
uint8_t AGC_InitAmpPosPostCalib1 = 5;
uint16_t AGC_InitAmpPostCalib2 = 500;
uint8_t AGC_InitAmpPosPostCalib2 = 5;

int8_t AGC_InitOff1 = -60;
uint8_t AGC_InitOffPos1 = 12;
int8_t AGC_InitOff2 = -5;
uint8_t AGC_InitOffPos2 = 1;
int8_t AGC_InitOff3 = -60;
uint8_t AGC_InitOffPos3 = 12;
int8_t AGC_InitOff4 = -60;
uint8_t AGC_InitOffPos4 = 12;

uint8_t AGC_InitLEDCur1 = 32;
uint8_t AGC_InitLEDCur2 = 32;
uint8_t AGC_InitLEDCur3 = 32;

agc_struct_t AGCStruct;

/*char tmpArr[50];*/

hqret_t AGC_SetSamplingFrequency(float SamplingFrequency)
{
    hqret_t ReturnValue = RET_FAIL;
    if (SamplingFrequency < 55.0 && SamplingFrequency > 45.0)
    {
        ReturnValue = RET_OK;

        ADJUSTMENTTIMEOUTLIMIT = 50;
        SAMPLESTOREEVALUATE = 100;

        DCEXPDECAYWEIGHTNUM = 50;
        DCEXPDECAYWEIGHTDENUM = 1000;
        ACEXPDECAYWEIGHTNUM = 20;
        ACEXPDECAYWEIGHTDENUM = 1000;
    }
    else if (SamplingFrequency < 20.0 && SamplingFrequency > 30.0)
    {
        ReturnValue = RET_OK;

        ADJUSTMENTTIMEOUTLIMIT = 25;
        SAMPLESTOREEVALUATE = 50;

        DCEXPDECAYWEIGHTNUM = 90;
        DCEXPDECAYWEIGHTDENUM = 1000;
        ACEXPDECAYWEIGHTNUM = 35;
        ACEXPDECAYWEIGHTDENUM = 1000;
    }
    return (ReturnValue);
}

hqret_t AGC_SetChannelLEDControl(uint8_t ChannelNumber, bool ChannelLEDControl)
{
    hqret_t ReturnValue = RET_FAIL;
    switch (ChannelNumber)
    {
    case 1:
        ReturnValue = RET_OK;
        AGCStruct.AGCCont_LEDCur[0] = ChannelLEDControl;
        break;
    case 3:
        ReturnValue = RET_OK;
        AGCStruct.AGCCont_LEDCur[1] = ChannelLEDControl;
        break;
    case 4:
        ReturnValue = RET_OK;
        AGCStruct.AGCCont_LEDCur[2] = ChannelLEDControl;
        break;
    default:
        break;
    }
    return (ReturnValue);
}

void AGC_SetOffControl(bool OffControl)
{
    AGCStruct.AGCCont_Off = OffControl;
}

void AGC_SetAmpControl(bool AmpControl)
{
    AGCStruct.AGCCont_Amp = AmpControl;
}

hqret_t AGC_TurnLEDOff(uint8_t ChannelNumber)
{
    hqret_t ReturnValue = RET_FAIL;
    switch (ChannelNumber)
    {
    case 1:
        ReturnValue = RET_OK;
        AGC_SetChannelLEDControl(ChannelNumber, FALSE);
        AGCStruct.Settings_LEDCur[0] = 0;
        AGC_InitLEDCur1 = 0;
        AGCStruct.Adj_LEDCur[0] = TRUE;
        break;
    case 3:
        ReturnValue = RET_OK;
        AGC_SetChannelLEDControl(ChannelNumber, FALSE);
        AGCStruct.Settings_LEDCur[1] = 0;
        AGC_InitLEDCur2 = 0;
        AGCStruct.Adj_LEDCur[1] = TRUE;
        break;
    case 4:
        ReturnValue = RET_OK;
        AGC_SetChannelLEDControl(ChannelNumber, FALSE);
        AGCStruct.Settings_LEDCur[2] = 0;
        AGC_InitLEDCur3 = 0;
        AGCStruct.Adj_LEDCur[2] = TRUE;
        break;
    default:
        break;
    }
    if (ReturnValue == RET_OK)
    {
        if (AGCStruct.AGCCont_AGCState == AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT ||
                AGCStruct.AGCCont_AGCState == AGC_STATE_CALIBRATION ||
                AGCStruct.AGCCont_AGCState == AGC_STATE_CALIBRATION_ERRORSTATE)
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT;
            AGCStruct.AGCCont_Calib_State = CALIB_STATE_INIT_SETTINGS;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
            AdjustAFE();
        }
        else if (AGCStruct.AGCCont_AGCState == AGC_STATE_WAITINGFORADJUSTMENT ||
                 AGCStruct.AGCCont_AGCState == AGC_STATE_INITIALIZEFILTER ||
                 AGCStruct.AGCCont_AGCState == AGC_STATE_FILTERSTABLE ||
                 AGCStruct.AGCCont_AGCState == AGC_STATE_ERRORSTATE)
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
            AdjustAFE();
        }
    }
    return (ReturnValue);
}

hqret_t AGC_TurnLEDOn(uint8_t ChannelNumber)
{
    hqret_t ReturnValue = RET_FAIL;
    switch (ChannelNumber)
    {
    case 1:
        ReturnValue = RET_OK;
        AGC_SetChannelLEDControl(ChannelNumber, TRUE);
        AGC_InitLEDCur1 = 32;
        break;
    case 3:
        ReturnValue = RET_OK;
        AGC_SetChannelLEDControl(ChannelNumber, TRUE);
        AGC_InitLEDCur2 = 32;
        break;
    case 4:
        ReturnValue = RET_OK;
        AGC_SetChannelLEDControl(ChannelNumber, TRUE);
        AGC_InitLEDCur3 = 32;
        break;
    default:
        break;
    }
    if (ReturnValue == RET_OK)
    {
        AGC_Recalibrate();
    }
    return (ReturnValue);
}

void AGC_InitializeAGC(void)
{
    memset(&AGCStruct, 0X00, sizeof(AGCStruct));

    //Initialising Variables
    AGCStruct.Counter_SamplesSinceStateChange = 0;
    AGCStruct.Settings_currentAFEsettings = 0;
    AGCStruct.Settings_oneSampleAgoAFESettings = 0;
    AGCStruct.Settings_twoSamplesAgoAFESettings = 0;

    AGCStruct.AGCCont_Off = TRUE;

    AGCStruct.AGCCont_Amp = FALSE;//TRUE;//cole modify for skin detect stable,default RF=500K;
    AGCStruct.AGCCont_UnableToIncreaseAmp = FALSE;

    AGCStruct.AGCCont_LEDCur[0] = FALSE;
    AGCStruct.AGCCont_LEDCur[1] = FALSE;
    AGCStruct.AGCCont_LEDCur[2] = FALSE;

    AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
    AGCStruct.AGCCont_Calib_State = CALIB_STATE_SELECT_OFF;

    AFE4404_setLedCurrent(1, 63);   // Green LED current
    AFE4404_setLedCurrent(2, 25);   // IR ,12->9
    AFE4404_setLedCurrent(3, 0);   // Red

    AFE4404_setCf(1, 100);
    AFE4404_setCf(2, 100);
    AFE4404_setRf(1, 1000);   //green/amb
    AFE4404_setRf(2, 1000);   //ir/red


    AFE4404_setAmbientCurrent(1, 0);
    AFE4404_setAmbientCurrent(2, 0);
    AFE4404_setAmbientCurrent(3, 0);
    AFE4404_setAmbientCurrent(4, 0);

    //AGC_SetSamplingFrequency(25);

}

void AGC_setAgcState(agc_state_t setState)
{
    AGCStruct.AGCCont_AGCState = setState;
}

agc_state_t AGC_ServiceAGCWithAFESettings(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code, uint64_t AFESettings)
{
    AGCStruct.Settings_currentAFEsettings = AFESettings;

    agc_state_t ReturnValue = AGC_STATE_CALIBRATION;

    if (AGCStruct.Settings_twoSamplesAgoAFESettings != 0)
    {
        if (AGCStruct.Settings_twoSamplesAgoAFESettings != AGCStruct.Settings_oneSampleAgoAFESettings)
        {
            ReturnValue = AGC_ServiceAGC(Ch1Code, Ch2Code, Ch3Code, Ch4Code, TRUE);
        }
        else
        {
            ReturnValue = AGC_ServiceAGC(Ch1Code, Ch2Code, Ch3Code, Ch4Code, FALSE);
        }
    }
    AGCStruct.Settings_twoSamplesAgoAFESettings = AGCStruct.Settings_oneSampleAgoAFESettings;
    AGCStruct.Settings_oneSampleAgoAFESettings = AGCStruct.Settings_currentAFEsettings;

    return (ReturnValue);
}

agc_state_t AGC_ServiceAGC(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code, bool AdjustmentMade)
{
    AGCStruct.Counter_SamplesSinceStateChange = AGCStruct.Counter_SamplesSinceStateChange + 1;
    // AGC State Machine
    switch (AGCStruct.AGCCont_AGCState)
    {
    case AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT:
        // Waits until adjustment has been made or TIMEOUT occurs
        if (AdjustmentMade)
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        else if (AGCStruct.Counter_SamplesSinceStateChange > ADJUSTMENTTIMEOUTLIMIT)
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION_ERRORSTATE;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    case AGC_STATE_CALIBRATION:
        // Ensure unexpected adjustment did not occur
        if (!AdjustmentMade)
        {
            if (AGCStruct.Counter_SamplesSinceStateChange > 2)
            {
                //Update signals summary metrics
                UpdateSignalState(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
                UpdateDCSignalSummary(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
                // Check if AFE adjustment is suggested, adjustAFE and transition to WAIT state
                if (CheckCalibrationAdjustment(Ch1Code, Ch2Code, Ch3Code, Ch4Code))
                {
                    AdjustAFE();
                    AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT;
                    AGCStruct.Counter_SamplesSinceStateChange = 0;
                }
                // Check if calibrated and transition
                if (AGCStruct.AGCCont_Calib_State == CALIB_STATE_CALIBRATED)
                {
                    // Check if adjustment has been made before transitioning to INITIALIZEFILTER or WAITINGFORADJUSTMENT
                    if (AGCStruct.Counter_SamplesSinceStateChange == 0)
                    {
                        AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
                    }
                    else
                    {
                        AGCStruct.AGCCont_AGCState = AGC_STATE_INITIALIZEFILTER;
                        AGCStruct.Counter_SamplesSinceStateChange = 0;
                    }
                }
            }
        }
        else
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION_ERRORSTATE;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    case AGC_STATE_CALIBRATION_ERRORSTATE:
        // Realigns AGC with AFE before attempting to resume calibration
        if (AGCStruct.Counter_SamplesSinceStateChange > 0)
        {
            AlignAGCWithAFE();
            AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION;
            AGCStruct.AGCCont_Calib_State = CALIB_STATE_INIT_SETTINGS;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    case AGC_STATE_WAITINGFORADJUSTMENT:
        // Waits until adjustment has been made or TIMEOUT occurs
        if (AdjustmentMade)
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_INITIALIZEFILTER;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        else if (AGCStruct.Counter_SamplesSinceStateChange > ADJUSTMENTTIMEOUTLIMIT)
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_ERRORSTATE;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    case AGC_STATE_ERRORSTATE:
        // Realigns AGC with AFE before attempting to resume with calculations
        if (AGCStruct.Counter_SamplesSinceStateChange > 0)
        {
            AlignAGCWithAFE();
            AGCStruct.AGCCont_AGCState = AGC_STATE_INITIALIZEFILTER;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    case AGC_STATE_INITIALIZEFILTER:
        // Ensure unexpected adjustment did not occur
        if (!AdjustmentMade)
        {
            // If stabilization time not reached, update signal states, check saturation and adjust, otherwise initialise filter
            if (AGCStruct.Counter_SamplesSinceStateChange <= FILTERSTABILIZATIONTIME)
            {
                UpdateSignalState(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
                if (SignalSaturated())
                {
                    if (CheckSaturationAdjustment())
                    {
                        AdjustAFE();
                        AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
                        AGCStruct.Counter_SamplesSinceStateChange = 0;
                    }
                }
                else
                {
                    InitializeFilter(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
                }
                // If stabilization time reached and no adjustment necessary, transition to stable filter
                if ((AGCStruct.Counter_SamplesSinceStateChange == FILTERSTABILIZATIONTIME) &&
                        (AGCStruct.Counter_SamplesSinceStateChange != 0))
                {
                    AGCStruct.AGCCont_AGCState = AGC_STATE_FILTERSTABLE;
                    AGCStruct.Counter_SamplesSinceStateChange = 0;
                }
            }
        }
        else
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_ERRORSTATE;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    case AGC_STATE_FILTERSTABLE:
        // Ensure unexpected adjustment did not occur
        if (!AdjustmentMade)
        {
            //Update various signal states and checks
            UpdateSignalState(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
            UpdateMinMax(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
            UpdateDCSignalSummary(Ch1Code, Ch2Code, Ch3Code, Ch4Code);
            UpdateACSignalSummary(Ch1Code, Ch2Code, Ch3Code, Ch4Code);

            // Check Saturation, bounds and re-evaluation
            if (SignalSaturated())
            {
                if (CheckSaturationAdjustment())
                {
                    AdjustAFE();
                    AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
                    AGCStruct.Counter_SamplesSinceStateChange = 0;
                }
            }
            else if (!SignalInBounds())
            {
                bool CheckVal1 = CheckOffAdjustement();
                bool CheckVal2 = CheckLEDCurAdjustement();
                bool CheckVal3 = CheckAmpAdjustment();
                if (CheckVal1 | CheckVal2 | CheckVal3)
                {
                    AdjustAFE();
                    AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
                    AGCStruct.Counter_SamplesSinceStateChange = 0;
                }
            }
            else if (ReEvaluateAC())
            {
                if (CheckAmpAdjustment())
                {
                    AdjustAFE();
                    AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
                    AGCStruct.Counter_SamplesSinceStateChange = 0;
                }
                if (AGCStruct.AGCCont_UnableToIncreaseAmp)
                {
                    //bool CheckVal1 = CheckOffAdjustement();
                    bool CheckVal2 = CheckLEDCurAdjustement();
                    if (CheckVal2)
                    {
                        AdjustAFE();
                        AGCStruct.AGCCont_AGCState = AGC_STATE_WAITINGFORADJUSTMENT;
                        AGCStruct.Counter_SamplesSinceStateChange = 0;
                    }
                }
            }
        }
        else
        {
            AGCStruct.AGCCont_AGCState = AGC_STATE_ERRORSTATE;
            AGCStruct.Counter_SamplesSinceStateChange = 0;
        }
        break;

    default:
        break;
    }
    return (AGCStruct.AGCCont_AGCState);
}

void AGC_Recalibrate(void)
{
    AlignAGCWithAFE();
    AGCStruct.AGCCont_AGCState = AGC_STATE_CALIBRATION;
    AGCStruct.AGCCont_Calib_State = CALIB_STATE_INIT_SETTINGS;
    AGCStruct.Counter_SamplesSinceStateChange = 0;
}

void UpdateMinMax(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code)
{
    if (Ch1Code > AGCStruct.SigSum_MaxCode[0])
    {
        AGCStruct.SigSum_MaxCode[0] = Ch1Code;
    }
    else if (Ch1Code < AGCStruct.SigSum_MinCode[0])
    {
        AGCStruct.SigSum_MinCode[0] = Ch1Code;
    }
    if (Ch2Code > AGCStruct.SigSum_MaxCode[1])
    {
        AGCStruct.SigSum_MaxCode[1] = Ch2Code;
    }
    else if (Ch2Code < AGCStruct.SigSum_MinCode[1])
    {
        AGCStruct.SigSum_MinCode[1] = Ch2Code;
    }
    if (Ch3Code > AGCStruct.SigSum_MaxCode[2])
    {
        AGCStruct.SigSum_MaxCode[2] = Ch3Code;
    }
    else if (Ch3Code < AGCStruct.SigSum_MinCode[2])
    {
        AGCStruct.SigSum_MinCode[2] = Ch3Code;
    }
    if (Ch4Code > AGCStruct.SigSum_MaxCode[3])
    {
        AGCStruct.SigSum_MaxCode[3] = Ch4Code;
    }
    else if (Ch4Code < AGCStruct.SigSum_MinCode[3])
    {
        AGCStruct.SigSum_MinCode[3] = Ch4Code;
    }
}

void UpdateSignalState(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code)
{
    uint16_t voltCodes[4];
    uint8_t counter = 0;

    voltCodes[0] = Ch1Code;
    voltCodes[1] = Ch2Code;
    voltCodes[2] = Ch3Code;
    voltCodes[3] = Ch4Code;
    for (counter = 0; counter < 4; counter ++)
    {
        if (voltCodes[counter] < SATURATELOW)
        {
            AGCStruct.SigSum_ChannelState[counter] = CHANNEL_STATE_SATURATELOW;
        }
        else if (voltCodes[counter] < MINVOLTCODELIMIT)
        {
            AGCStruct.SigSum_ChannelState[counter] = CHANNEL_STATE_OUTOFLOWERBOUND;
        }
        else if (voltCodes[counter] < MAXVOLTCODELIMIT)
        {
            AGCStruct.SigSum_ChannelState[counter] = CHANNEL_STATE_STABLE;
        }
        else if (voltCodes[counter] < SATURATEHIGH)
        {
            AGCStruct.SigSum_ChannelState[counter] = CHANNEL_STATE_OUTOFUPPERBOUND;
        }
        else
        {
            AGCStruct.SigSum_ChannelState[counter] = CHANNEL_STATE_SATURATEHIGH;
        }
    }
}

void UpdateDCSignalSummary(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code)
{
    int64_t tmpVal = 0;
    uint16_t voltCodes[4];
    uint8_t counter = 0;

    voltCodes[0] = Ch1Code;
    voltCodes[1] = Ch2Code;
    voltCodes[2] = Ch3Code;
    voltCodes[3] = Ch4Code;

    for (counter = 0; counter < 4; counter++)
    {
        if (AGCStruct.SigSum_FiltValDC[counter] ==  0)
        {
            AGCStruct.SigSum_FiltValDC[counter] = voltCodes[counter];
        }
        else
        {
            tmpVal = ((int64_t)(AGCStruct.SigSum_FiltValDC[counter])) * ((int64_t)(DCEXPDECAYWEIGHTDENUM) - (int64_t)(DCEXPDECAYWEIGHTNUM));
            tmpVal = tmpVal + (((int64_t)(DCEXPDECAYWEIGHTNUM)) * ((int64_t)(voltCodes[counter])));
            tmpVal = tmpVal / ((int64_t)(DCEXPDECAYWEIGHTDENUM));
            AGCStruct.SigSum_FiltValDC[counter] = (int32_t)(tmpVal);
        }
    }
}

void UpdateACSignalSummary(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code)
{
    int32_t tmpVal = 0;
    uint16_t voltCodes[4];
    uint16_t newValueAbsAC[4] = {0, 0, 0, 0};
    uint8_t counter = 0;

    voltCodes[0] = Ch1Code;
    voltCodes[1] = Ch2Code;
    voltCodes[2] = Ch3Code;
    voltCodes[3] = Ch4Code;

    for (counter = 0; counter < 4; counter++)
    {
        newValueAbsAC[counter] = AGC_abs(((int64_t)(AGCStruct.SigSum_FiltValDC[counter]) - (int64_t)(voltCodes[counter])));
        if (AGCStruct.SigSum_FiltValAC[counter] == 0)
        {
            AGCStruct.SigSum_FiltValAC[counter] = newValueAbsAC[counter];
        }
        else
        {
            tmpVal = ((int64_t)(AGCStruct.SigSum_FiltValAC[counter])) * ((int64_t)(ACEXPDECAYWEIGHTDENUM) - (int64_t)(ACEXPDECAYWEIGHTNUM));
            tmpVal = tmpVal + (((int64_t)(ACEXPDECAYWEIGHTNUM)) * ((int64_t)(newValueAbsAC[counter])));
            tmpVal = tmpVal / ((int64_t)(ACEXPDECAYWEIGHTDENUM));
            AGCStruct.SigSum_FiltValAC[counter] = (int32_t)(tmpVal);
        }
    }
}

bool CheckCalibrationAdjustment(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code)
{
    uint8_t counter = 0;
    uint8_t channelInd[3] = {0, 2, 3};
    int32_t tmpConst[3];
    int32_t tmpVal [3];
    int32_t CTR [3];
    bool retVal = FALSE;

    if (AGCStruct.AGCCont_Calib_State == CALIB_STATE_SELECT_LED_CURRENT)
    {
        tmpConst[0] = (4000000 / (int32_t)AGCStruct.Settings_Amp[0]) >> 4;
        tmpConst[1] = (4000000 / (int32_t)AGCStruct.Settings_Amp[1]) >> 4;
        tmpConst[2] = (4000000 / (int32_t)AGCStruct.Settings_Amp[1]) >> 4;
    }

    switch (AGCStruct.AGCCont_Calib_State)
    {
    case CALIB_STATE_INIT_SETTINGS:
        AGCStruct.Settings_Amp[0] = AGC_InitAmpPreCalib1;
        AGCStruct.Settings_Amp_Pos[0] = AGC_InitAmpPosPreCalib1;
        AGCStruct.Settings_Amp[1] = AGC_InitAmpPreCalib2;
        AGCStruct.Settings_Amp_Pos[1] = AGC_InitAmpPosPreCalib2;
        AGCStruct.Adj_Amp[0] = TRUE;
        AGCStruct.Adj_Amp[1] = TRUE;

        AGCStruct.Settings_Off[0] = AGC_InitOff1;
        AGCStruct.Settings_Off_Pos[0] = AGC_InitOffPos1;
        AGCStruct.Settings_Off[1] = AGC_InitOff2;
        AGCStruct.Settings_Off_Pos[1] = AGC_InitOffPos2;
        AGCStruct.Settings_Off[2] = AGC_InitOff3;
        AGCStruct.Settings_Off_Pos[2] = AGC_InitOffPos3;
        AGCStruct.Settings_Off[3] = AGC_InitOff4;
        AGCStruct.Settings_Off_Pos[3] = AGC_InitOffPos4;
        AGCStruct.Adj_Off[0] = TRUE;
        AGCStruct.Adj_Off[1] = TRUE;
        AGCStruct.Adj_Off[2] = TRUE;
        AGCStruct.Adj_Off[3] = TRUE;

        AGCStruct.Settings_LEDCur[0] = AGC_InitLEDCur1;
        AGCStruct.Settings_LEDCur[1] = AGC_InitLEDCur2;
        AGCStruct.Settings_LEDCur[2] = AGC_InitLEDCur3;

        AGCStruct.Adj_LEDCur[0] = TRUE;
        AGCStruct.Adj_LEDCur[1] = TRUE;
        AGCStruct.Adj_LEDCur[2] = TRUE;

        AGCStruct.AGCCont_Calib_State = CALIB_STATE_BIN_SEARCH_0;

        retVal = TRUE;
        break;
    case CALIB_STATE_BIN_SEARCH_0:
        for (counter = 0; counter < 3; counter ++)
        {
            if (AGCStruct.AGCCont_LEDCur[counter])
            {
                if (ChannelTooHigh(channelInd[counter]))
                {
                    AGCStruct.Settings_LEDCur[counter] = 16;
                    AGCStruct.Adj_LEDCur[counter] = TRUE;
                    retVal = TRUE;
                }
                if (ChannelTooLow(channelInd[counter]))
                {
                    AGCStruct.Settings_LEDCur[counter] = 48;
                    AGCStruct.Adj_LEDCur[counter] = TRUE;
                    retVal = TRUE;
                }
            }
        }
        AGCStruct.AGCCont_Calib_State = CALIB_STATE_BIN_SEARCH_1;
        break;
    case CALIB_STATE_BIN_SEARCH_1:
        for (counter = 0; counter < 3; counter ++)
        {
            if (AGCStruct.AGCCont_LEDCur[counter])
            {
                if (ChannelTooLow(channelInd[counter]))
                {
                    if (AGCStruct.Settings_LEDCur[counter] == 16)
                    {
                        AGCStruct.Settings_LEDCur[counter] = 24;
                        AGCStruct.Adj_LEDCur[counter] = TRUE;
                        retVal = TRUE;
                    }
                    else if (AGCStruct.Settings_LEDCur[counter] == 32)
                    {
                        AGCStruct.Settings_LEDCur[counter] = 40;
                        AGCStruct.Adj_LEDCur[counter] = TRUE;
                        retVal = TRUE;
                    }
                    else if (AGCStruct.Settings_LEDCur[counter] == 48)
                    {
                        AGCStruct.Settings_LEDCur[counter] = 56;
                        AGCStruct.Adj_LEDCur[counter] = TRUE;
                        retVal = TRUE;
                    }
                }
                if (ChannelTooHigh(channelInd[counter]))
                {
                    if (AGCStruct.Settings_LEDCur[counter] == 16)
                    {
                        AGCStruct.Settings_LEDCur[counter] = 8;
                        AGCStruct.Adj_LEDCur[counter] = TRUE;
                        retVal = TRUE;
                    }
                    else if (AGCStruct.Settings_LEDCur[counter] == 32)
                    {
                        AGCStruct.Settings_LEDCur[counter] = 24;
                        AGCStruct.Adj_LEDCur[counter] = TRUE;
                        retVal = TRUE;
                    }
                    else if (AGCStruct.Settings_LEDCur[counter] == 48)
                    {
                        AGCStruct.Settings_LEDCur[counter] = 40;
                        AGCStruct.Adj_LEDCur[counter] = TRUE;
                        retVal = TRUE;
                    }
                }
            }
        }
        AGCStruct.AGCCont_Calib_State = CALIB_STATE_SELECT_LED_CURRENT;
        break;
    case CALIB_STATE_SELECT_LED_CURRENT:
        for (counter = 0; counter < 3; counter++)
        {
            if (AGCStruct.AGCCont_LEDCur[counter])
            {
                tmpVal[counter] = (tmpConst[counter] * (AGCStruct.SigSum_FiltValDC[channelInd[counter]] - 0x7fff)) >> 10;
                tmpVal[counter] = tmpVal[counter] * 75 - AGCStruct.Settings_Off[channelInd[counter]] * 100000;
                CTR[counter] = ((float)(tmpVal[counter])) / ((float)(AGCStruct.Settings_LEDCur[counter]));
                if (AGCStruct.Settings_LEDCur[counter] != (uint8_t) AGC_min(63, (uint16_t)(((float)(6000000)) / CTR[counter])))
                {
                    AGCStruct.Settings_LEDCur[counter] = (uint8_t) AGC_min(63, (uint16_t)(((float)(6000000)) / CTR[counter]));
                    AGCStruct.Adj_LEDCur[counter] = TRUE;
                    retVal = TRUE;
                }
            }
        }
        AGCStruct.AGCCont_Calib_State = CALIB_STATE_SELECT_OFF;
        break;
    case CALIB_STATE_SELECT_OFF:
        retVal = CheckOffAdjustement();
        AGCStruct.AGCCont_Calib_State = CALIB_STATE_SELECT_AMP;
        break;
    case CALIB_STATE_SELECT_AMP:
        if (AGC_InitAmpPostCalib1 != AGC_InitAmpPreCalib1)
        {
            AGCStruct.Settings_Amp[0] = AGC_InitAmpPostCalib1;
            AGCStruct.Settings_Amp_Pos[0] = AGC_InitAmpPosPostCalib1;
            AGCStruct.Adj_Amp[0] = TRUE;
            retVal = TRUE;
        }
        if (AGC_InitAmpPostCalib2 != AGC_InitAmpPreCalib2)
        {
            AGCStruct.Settings_Amp[1] = AGC_InitAmpPostCalib2;
            AGCStruct.Settings_Amp_Pos[1] = AGC_InitAmpPosPostCalib2;
            AGCStruct.Adj_Amp[1] = TRUE;
        }
        AGCStruct.AGCCont_Calib_State = CALIB_STATE_ENSURENONSATURATION;
        break;
    case CALIB_STATE_ENSURENONSATURATION:
        if (SignalSaturated())
        {
            retVal = CheckSaturationAdjustment();
        }
        else
        {
            retVal = FALSE;
            AGCStruct.AGCCont_Calib_State = CALIB_STATE_CALIBRATED;
        }
        break;
    case CALIB_STATE_CALIBRATED:
        break;
    default:
        break;
    }
    return (retVal);
}

bool CheckOffAdjustement(void)
{
    if (AGCStruct.AGCCont_Off)
    {
        int32_t tmpConst1 = (4000000 / (int32_t)AGCStruct.Settings_Amp[0]) >> 4;
        int32_t tmpConst2 = (4000000 / (int32_t)AGCStruct.Settings_Amp[1]) >> 4;
        int8_t counter1 = 0;
        int8_t counter2 = 0;
        int32_t LowerBound = 0;
        int32_t HigherBound = 0;
        int8_t OffChoice = 0;
        int32_t tmpVal [4];

        tmpVal[0] = (tmpConst1 * (AGCStruct.SigSum_FiltValDC[0] - 0x7fff)) >> 10;
        tmpVal[0] = tmpVal[0] * 75 - AGCStruct.Settings_Off[0] * 100000;
        tmpVal[1] = (tmpConst1 * (AGCStruct.SigSum_FiltValDC[1] - 0x7fff)) >> 10;
        tmpVal[1] = tmpVal[1] * 75 - AGCStruct.Settings_Off[1] * 100000;
        tmpVal[2] = (tmpConst2 * (AGCStruct.SigSum_FiltValDC[2] - 0x7fff)) >> 10;
        tmpVal[2] = tmpVal[2] * 75 - AGCStruct.Settings_Off[2] * 100000;
        tmpVal[3] = (tmpConst2 * (AGCStruct.SigSum_FiltValDC[3] - 0x7fff)) >> 10;
        tmpVal[3] = tmpVal[3] * 75 - AGCStruct.Settings_Off[3] * 100000;


        for (counter1 = 0; counter1 < 4; counter1++)
        {
            OffChoice = OFFOPTIONSLENGTH;
            for (counter2 = 0; counter2 < OFFOPTIONSLENGTH; counter2++)
            {
                // Find possible OFFOPTIONS by creating a lower bound halfway between the current and
                //      previous option and a upper bound halfway between the current and next option
                if (counter2 != 0)
                {
                    LowerBound = -10000 * (10 * ((int32_t)(OFFOPTIONS[counter2 - 1]) + (int32_t)(OFFOPTIONS[counter2])) / 2);
                }
                else
                {
                    LowerBound = -2147483647;
                }
                if (counter2 != OFFOPTIONSLENGTH - 1)
                {
                    HigherBound = -10000 * (10 * ((int32_t)(OFFOPTIONS[counter2]) + (int32_t)(OFFOPTIONS[counter2 + 1])) / 2);
                }
                else
                {
                    HigherBound = 2147483647;
                }
                if (tmpVal[counter1] >= LowerBound && tmpVal[counter1] <= HigherBound)
                {
                    OffChoice = counter2;
                }
            }
            // Adjust value if not the same
            if (AGCStruct.Settings_Off[counter1] != OFFOPTIONS[OffChoice])
            {
                AGCStruct.Settings_Off[counter1] = OFFOPTIONS[OffChoice];
                AGCStruct.Settings_Off_Pos[counter1] = OffChoice;
                AGCStruct.Adj_Off[counter1] = TRUE;
            }
        }

        return ((AGCStruct.Adj_Off[0] || AGCStruct.Adj_Off[1] || AGCStruct.Adj_Off[2] || AGCStruct.Adj_Off[3]));
    }
    else
    {
        return (FALSE);
    }
}

bool CheckAmpAdjustment(void)
{
    if (AGCStruct.AGCCont_Amp)
    {
        int32_t tmpConst1 = (4000000 / (int32_t)AGCStruct.Settings_Amp[0]) >> 4;
        int32_t tmpConst2 = (4000000 / (int32_t)AGCStruct.Settings_Amp[1]) >> 4;

        int32_t tmpVal [4];
        uint16_t objAmp [2];

        uint8_t counter1 = 0;
        uint8_t counter2 = 0;
        uint8_t oldAmpPosition[2];
        uint8_t newAmpPosition[2];
        uint16_t LowerBound = 0;
        uint16_t HigherBound = 0;

        int32_t predictedVolt1;
        int32_t predictedVolt2;

        oldAmpPosition[0] = AGCStruct.Settings_Amp_Pos[0];
        oldAmpPosition[1] = AGCStruct.Settings_Amp_Pos[1];
        newAmpPosition[0] = 0;
        newAmpPosition[1] = 0;

        tmpVal[0] = (tmpConst1 * (AGCStruct.SigSum_FiltValAC[0])) >> 10;
        tmpVal[0] = tmpVal[0] * 75;
        tmpVal[1] = (tmpConst1 * (AGCStruct.SigSum_FiltValAC[1])) >> 10;
        tmpVal[1] = tmpVal[1] * 75;
        tmpVal[2] = (tmpConst2 * (AGCStruct.SigSum_FiltValAC[2])) >> 10;
        tmpVal[2] = tmpVal[2] * 75;
        tmpVal[3] = (tmpConst2 * (AGCStruct.SigSum_FiltValAC[3])) >> 10;
        tmpVal[3] = tmpVal[3] * 75;

        tmpVal[0] = (OBJVOLTAC1 / 2 / (AGC_max((int32_t)tmpVal[0], (int32_t)tmpVal[1]) + 1));
        tmpVal[2] = (OBJVOLTAC2 / 2 / (AGC_max((int32_t)tmpVal[2], (int32_t)tmpVal[3]) + 1));

        objAmp[0] = (uint16_t) AGC_max(AGC_min(tmpVal[0], 32766), 0);
        objAmp[1] = (uint16_t) AGC_max(AGC_min(tmpVal[2], 32766), 0);

        for (counter1 = 0; counter1 < 2; counter1++)
        {
            // Find possible AMPOPTIONS by creating a lower bound halfway between the current and
            //      previous option and a upper bound halfway between the current and next option
            for (counter2 = 0; counter2 < AMPOPTIONSLENGTH; counter2++)
            {
                if (counter2 != 0)
                {
                    LowerBound = (AMPOPTIONS[counter2 - 1] + AMPOPTIONS[counter2]) / 2;
                }
                else
                {
                    LowerBound = 0;
                }
                if (counter2 != AMPOPTIONSLENGTH - 1)
                {
                    HigherBound = (AMPOPTIONS[counter2] + AMPOPTIONS[counter2 + 1]) / 2;
                }
                else
                {
                    HigherBound = 65535;
                }

                if ((objAmp[counter1] >= LowerBound) && (objAmp[counter1] < HigherBound))
                {
                    newAmpPosition[counter1] = counter2;
                }
            }

            // Next AMPOPTIONS predicted effect of desired adjustment
            predictedVolt1 = (((int32_t)AGCStruct.SigSum_MaxCode[counter1 * 2]) - 32768) * (int32_t)(AMPOPTIONS[oldAmpPosition[counter1] + 1]) / ((int32_t)(AMPOPTIONS[oldAmpPosition[counter1]])) + 32768;
            predictedVolt2 = (((int32_t)AGCStruct.SigSum_MaxCode[counter1 * 2 + 1]) - 32768) * (int32_t)(AMPOPTIONS[oldAmpPosition[counter1] + 1]) / ((int32_t)(AMPOPTIONS[oldAmpPosition[counter1]])) + 32768;

            if (newAmpPosition[counter1] > oldAmpPosition[counter1])
            {

                // Valid predicted voltages for adjustments, increment oldAmpPosition by 1
                if ((predictedVolt1 < MAXVOLTCODELIMITFORRFADJUSTMENT) && (predictedVolt2 < MAXVOLTCODELIMITFORRFADJUSTMENT) &&
                        (predictedVolt1 > MINVOLTCODELIMITFORRFADJUSTMENT) && (predictedVolt2 > MINVOLTCODELIMITFORRFADJUSTMENT))
                {
                    newAmpPosition[counter1] = oldAmpPosition[counter1] + 1;
                    AGCStruct.Settings_Amp[counter1] = AMPOPTIONS[newAmpPosition[counter1]];
                    AGCStruct.Adj_Amp[counter1] = TRUE;
                }
                // Invalid predicted voltages
                else
                {
                    // MAXVOLTCODELIMIT or MINVOLTCODELIMIT exceeded and oldAmpPos > 0, decrement by 1
                    if ((AGCStruct.SigSum_MaxCode[counter1 * 2] > MAXVOLTCODELIMIT) || (AGCStruct.SigSum_MaxCode[counter1 * 2 + 1] > MAXVOLTCODELIMIT) ||
                            (AGCStruct.SigSum_MinCode[counter1 * 2] < MINVOLTCODELIMIT) || (AGCStruct.SigSum_MinCode[counter1 * 2 + 1] < MINVOLTCODELIMIT))
                    {
                        if (oldAmpPosition[counter1] > 0)
                        {
                            newAmpPosition[counter1] = oldAmpPosition[counter1] - 1;
                            AGCStruct.Settings_Amp[counter1 ] = AMPOPTIONS[newAmpPosition[counter1]];
                            AGCStruct.Adj_Amp[counter1] = TRUE;
                            AGCStruct.AGCCont_UnableToIncreaseAmp = TRUE;
                        }
                    }
                    // Do not change Rf
                    else
                    {
                        newAmpPosition[counter1] = oldAmpPosition[counter1];
                        AGCStruct.AGCCont_UnableToIncreaseAmp = TRUE;
                    }
                }
            }
            else if (newAmpPosition[counter1] == oldAmpPosition[counter1])
            {
                // MAXVOLTCODELIMIT or MINVOLTCODELIMIT exceeded and oldAmpPos > 0, decrement by 1
                if ((AGCStruct.SigSum_MaxCode[counter1 * 2] > MAXVOLTCODELIMIT) || (AGCStruct.SigSum_MaxCode[counter1 * 2 + 1] > MAXVOLTCODELIMIT) ||
                        (AGCStruct.SigSum_MinCode[counter1 * 2] < MINVOLTCODELIMIT) || (AGCStruct.SigSum_MinCode[counter1 * 2 + 1] < MINVOLTCODELIMIT))
                {
                    if (oldAmpPosition[counter1] > 0)
                    {
                        newAmpPosition[counter1] = oldAmpPosition[counter1] - 1;
                        AGCStruct.Settings_Amp[counter1] = AMPOPTIONS[newAmpPosition[counter1]];
                        AGCStruct.Adj_Amp[counter1] = TRUE;
                    }
                }
                // Do not change Rf
                else
                {
                    newAmpPosition[counter1] = oldAmpPosition[counter1];
                }
            }
            else if (newAmpPosition[counter1] < oldAmpPosition[counter1])
            {
                if (oldAmpPosition[counter1] > 0)
                {
                    newAmpPosition[counter1] = oldAmpPosition[counter1] - 1;
                    AGCStruct.Settings_Amp[counter1] = AMPOPTIONS[newAmpPosition[counter1]];
                    AGCStruct.Adj_Amp[counter1] = TRUE;
                }
            }
        }
        if ((AGCStruct.Adj_Amp[0] || AGCStruct.Adj_Amp[1]))
        {
            AGCStruct.Settings_Amp_Pos[0] = newAmpPosition[0];
            AGCStruct.Settings_Amp_Pos[1] = newAmpPosition[1];
            return (TRUE);
        }
        else
        {
            return (FALSE);
        }
    }
    else
    {
        return (FALSE);
    }
}

bool CheckLEDCurAdjustement(void)
{
    if (AGCStruct.AGCCont_LEDCur[0] || AGCStruct.AGCCont_LEDCur[1] || AGCStruct.AGCCont_LEDCur[2])
    {

        int32_t tmpConst1 = (4000000 / (int32_t)AGCStruct.Settings_Amp[0]) >> 4;
        int32_t tmpConst2 = (4000000 / (int32_t)AGCStruct.Settings_Amp[1]) >> 4;

        int32_t tmpVal [4];

        tmpVal[0] = (tmpConst1 * (AGCStruct.SigSum_FiltValDC[0] - 0x7fff)) >> 10;
        tmpVal[0] = tmpVal[0] * 75 - AGCStruct.Settings_Off[0] * 100000;
        tmpVal[1] = (tmpConst1 * (AGCStruct.SigSum_FiltValDC[1] - 0x7fff)) >> 10;
        tmpVal[1] = tmpVal[1] * 75 - AGCStruct.Settings_Off[1] * 100000;
        tmpVal[2] = (tmpConst2 * (AGCStruct.SigSum_FiltValDC[2] - 0x7fff)) >> 10;
        tmpVal[2] = tmpVal[2] * 75 - AGCStruct.Settings_Off[2] * 100000;
        tmpVal[3] = (tmpConst2 * (AGCStruct.SigSum_FiltValDC[3] - 0x7fff)) >> 10;
        tmpVal[3] = tmpVal[3] * 75 - AGCStruct.Settings_Off[3] * 100000;

        if (AGCStruct.AGCCont_LEDCur[0])
        {
            // Evaluate channel 1 for possible LED current adjustment
            if (((tmpVal[0] - tmpVal[1]) > OBJMAXVALLEDCONT) && (AGCStruct.Settings_LEDCur[0] > 3))
            {
                //Decrease channel x setting
                AGCStruct.Settings_LEDCur[0] = AGCStruct.Settings_LEDCur[0] - 1;
                AGCStruct.Adj_LEDCur[0] = TRUE;
            }
            else if (((tmpVal[0] - tmpVal[1]) < OBJMINVALLEDCONT) && (AGCStruct.Settings_LEDCur[0] < 62))
            {
                //Increase channel x setting
                AGCStruct.Settings_LEDCur[0] = AGCStruct.Settings_LEDCur[0] + 1;
                AGCStruct.Adj_LEDCur[0] = TRUE;
            }
        }

        if (AGCStruct.AGCCont_LEDCur[1])
        {
            // Evaluate channel 3 for possible LED current adjustment
            if (((tmpVal[2] - tmpVal[1]) > OBJMAXVALLEDCONT) && (AGCStruct.Settings_LEDCur[1] > 3))
            {
                //Decrease channel x setting
                AGCStruct.Settings_LEDCur[1] = 12;//AGCStruct.Settings_LEDCur[1] - 1; //cole modify fixed ir led current
                AGCStruct.Adj_LEDCur[1] = TRUE;
            }
            else if (((tmpVal[2] - tmpVal[1]) < OBJMINVALLEDCONT) && (AGCStruct.Settings_LEDCur[1] < 62))
            {
                //Increase channel x setting
                AGCStruct.Settings_LEDCur[1] = 12;//AGCStruct.Settings_LEDCur[1] + 1;//cole modify fixed ir led current
                AGCStruct.Adj_LEDCur[1] = TRUE;
            }
        }

        if (AGCStruct.AGCCont_LEDCur[2])
        {
            // Evaluate channel 4 for possible LED current adjustment
            if (((tmpVal[3] - tmpVal[1]) > OBJMAXVALLEDCONT) && (AGCStruct.Settings_LEDCur[2] > 3))
            {
                //Decrease channel x setting
                AGCStruct.Settings_LEDCur[2] = 12;//AGCStruct.Settings_LEDCur[2] - 1;//cole modify fixed red led current
                AGCStruct.Adj_LEDCur[2] = TRUE;
            }
            else if (((tmpVal[3] - tmpVal[1]) < OBJMINVALLEDCONT) && (AGCStruct.Settings_LEDCur[2] < 62))
            {
                //Increase channel x setting
                AGCStruct.Settings_LEDCur[2] = 12;//AGCStruct.Settings_LEDCur[2] + 1;//cole modify fixed red led current
                AGCStruct.Adj_LEDCur[2] = TRUE;
            }
        }
        return ((AGCStruct.Adj_LEDCur[0] || AGCStruct.Adj_LEDCur[1] || AGCStruct.Adj_LEDCur[2]));
    }
    else
    {
        return (FALSE);
    }
}

bool CheckSaturationAdjustment(void)
{
    bool CheckVal1 = CheckChannelSaturationAdjustment(0);
    bool CheckVal2 = CheckChannelSaturationAdjustment(1);
    bool CheckVal3 = CheckChannelSaturationAdjustment(2);
    bool CheckVal4 = CheckChannelSaturationAdjustment(3);
    return (CheckVal1 || CheckVal2 || CheckVal3 || CheckVal4);
}

bool CheckChannelSaturationAdjustment(uint8_t ChannelNumber)
{
    agc_channel_state_t ChannelState = AGCStruct.SigSum_ChannelState[ChannelNumber];
    uint8_t AmpInd[4] = {0, 0, 1, 1};
    uint8_t LEDInd[4] = {0, 3, 1, 2};
    bool retVal = FALSE;
    //Checks if HIGH or LOW saturation reached
    if (ChannelState == CHANNEL_STATE_SATURATEHIGH)
    {
        // Adjust offset value positive if offset adjustment enabled (value gets subtracted thus lowering offset)
        if (AGCStruct.AGCCont_Off)
        {
            if (AGCStruct.Settings_Off_Pos[ChannelNumber] < OFFOPTIONSLENGTH - 1)
            {
                AGCStruct.Settings_Off_Pos[ChannelNumber] = AGCStruct.Settings_Off_Pos[ChannelNumber] + 1;
                AGCStruct.Settings_Off[ChannelNumber] = OFFOPTIONS[AGCStruct.Settings_Off_Pos[ChannelNumber]];
                AGCStruct.Adj_Off[ChannelNumber] = TRUE;
                retVal = TRUE;
            }
        }
        // Decrease Rf rating to previous valid size if Rf adjustment enabled and no offset adjustment made
        if (AGCStruct.AGCCont_Amp && (!(retVal)))
        {
            if (AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]] != 0)
            {
                AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]] = AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]] - 1;
                AGCStruct.Settings_Amp[AmpInd[ChannelNumber]] = AMPOPTIONS[AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]]];
                AGCStruct.Adj_Amp[AmpInd[ChannelNumber]] = TRUE;
                retVal = TRUE;
            }
        }
        // Decrease LED current if LED current adjustment enabled AND (no offset OR Rf adjustment made )
        if (ChannelNumber != 1 && (!(retVal)))
        {
            if (AGCStruct.AGCCont_LEDCur[LEDInd[ChannelNumber]] && AGCStruct.Settings_LEDCur[LEDInd[ChannelNumber]] > 3)
            {
                AGCStruct.Settings_LEDCur[LEDInd[ChannelNumber]] = AGCStruct.Settings_LEDCur[LEDInd[ChannelNumber]] - 1;
                AGCStruct.Adj_LEDCur[LEDInd[ChannelNumber]] = TRUE;
                retVal = TRUE;
            }
        }
    }
    else if (ChannelState == CHANNEL_STATE_SATURATELOW)
    {
        // Adjust offset value negatively if offset adjustment enabled (value gets subtracted thus raising offset)
        if (AGCStruct.AGCCont_Off)
        {
            if (AGCStruct.Settings_Off_Pos[ChannelNumber] > 0)
            {
                AGCStruct.Settings_Off_Pos[ChannelNumber] = AGCStruct.Settings_Off_Pos[ChannelNumber] - 1;
                AGCStruct.Settings_Off[ChannelNumber] = OFFOPTIONS[AGCStruct.Settings_Off_Pos[ChannelNumber]];
                AGCStruct.Adj_Off[ChannelNumber] = TRUE;
                retVal = TRUE;
            }
        }
        // Decrease Rf rating to next valid size if Rf adjustment enabled and no offset adjustment made
        if (AGCStruct.AGCCont_Amp && (!(retVal)))
        {
            if (AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]] != 0)
            {
                AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]] = AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]] - 1;
                AGCStruct.Settings_Amp[AmpInd[ChannelNumber]] = AMPOPTIONS[AGCStruct.Settings_Amp_Pos[AmpInd[ChannelNumber]]];
                AGCStruct.Adj_Amp[AmpInd[ChannelNumber]] = TRUE;
                retVal = TRUE;
            }
        }
        // Increase LED current if LED current adjustment enabled AND (no offset OR Rf adjustment made )
        if (ChannelNumber != 1 && (!(retVal)))
        {
            if (AGCStruct.AGCCont_LEDCur[LEDInd[ChannelNumber]] && AGCStruct.Settings_LEDCur[LEDInd[ChannelNumber]] < 62)
            {
                AGCStruct.Settings_LEDCur[LEDInd[ChannelNumber]] = AGCStruct.Settings_LEDCur[LEDInd[ChannelNumber]] + 1;
                AGCStruct.Adj_LEDCur[LEDInd[ChannelNumber]] = TRUE;
                retVal = TRUE;
            }
        }
    }
    return (retVal);
}

bool SignalSaturated(void)
{
    bool retVal1 = (AGCStruct.SigSum_ChannelState[0] == CHANNEL_STATE_SATURATEHIGH || AGCStruct.SigSum_ChannelState[0] == CHANNEL_STATE_SATURATELOW);
    bool retVal2 = (AGCStruct.SigSum_ChannelState[1] == CHANNEL_STATE_SATURATEHIGH || AGCStruct.SigSum_ChannelState[1] == CHANNEL_STATE_SATURATELOW);
    bool retVal3 = (AGCStruct.SigSum_ChannelState[2] == CHANNEL_STATE_SATURATEHIGH || AGCStruct.SigSum_ChannelState[2] == CHANNEL_STATE_SATURATELOW);
    bool retVal4 = (AGCStruct.SigSum_ChannelState[3] == CHANNEL_STATE_SATURATEHIGH || AGCStruct.SigSum_ChannelState[3] == CHANNEL_STATE_SATURATELOW);
    return (retVal1 || retVal2 || retVal3 || retVal4);
}

bool SignalInBounds(void)
{
    bool retVal1 = (AGCStruct.SigSum_ChannelState[0] == CHANNEL_STATE_STABLE);
    bool retVal2 = (AGCStruct.SigSum_ChannelState[1] == CHANNEL_STATE_STABLE);
    bool retVal3 = (AGCStruct.SigSum_ChannelState[2] == CHANNEL_STATE_STABLE);
    bool retVal4 = (AGCStruct.SigSum_ChannelState[3] == CHANNEL_STATE_STABLE);
    bool retVal = (retVal1 && retVal2 && retVal3 && retVal4);
    return (retVal);
}

bool ReEvaluateAC(void)
{
    bool retVal = FALSE;
    if (AGCStruct.Counter_SamplesSinceStateChange % SAMPLESTOREEVALUATE == 0)
    {
        AGCStruct.Counter_SamplesSinceStateChange = 0;
        retVal = TRUE;
    }
    return (retVal);
}

bool ChannelTooHigh(uint8_t ChannelNumber)
{
    return (AGCStruct.SigSum_ChannelState[ChannelNumber] == CHANNEL_STATE_SATURATEHIGH || AGCStruct.SigSum_ChannelState[ChannelNumber] == CHANNEL_STATE_OUTOFUPPERBOUND);
}

bool ChannelTooLow(uint8_t ChannelNumber)
{
    return (AGCStruct.SigSum_ChannelState[ChannelNumber] == CHANNEL_STATE_SATURATELOW || AGCStruct.SigSum_ChannelState[ChannelNumber] == CHANNEL_STATE_OUTOFLOWERBOUND);
}

void InitializeFilter(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code)
{
    static uint32_t Ch1CodeBuf [FILTERSTABILIZATIONTIME];
    static uint32_t Ch2CodeBuf [FILTERSTABILIZATIONTIME];
    static uint32_t Ch3CodeBuf [FILTERSTABILIZATIONTIME];
    static uint32_t Ch4CodeBuf [FILTERSTABILIZATIONTIME];
    static uint32_t CumSumDC[4] = {0, 0, 0, 0};

    // Populate buffer with new value
    Ch1CodeBuf[AGCStruct.Counter_SamplesSinceStateChange - 1] = Ch1Code;
    Ch2CodeBuf[AGCStruct.Counter_SamplesSinceStateChange - 1] = Ch2Code;
    Ch3CodeBuf[AGCStruct.Counter_SamplesSinceStateChange - 1] = Ch3Code;
    Ch4CodeBuf[AGCStruct.Counter_SamplesSinceStateChange - 1] = Ch4Code;

    // Clears cumulative sum on first call to function
    if (AGCStruct.Counter_SamplesSinceStateChange == 1)
    {
        CumSumDC[0] = 0;
        CumSumDC[1] = 0;
        CumSumDC[2] = 0;
        CumSumDC[3] = 0;
    }
    // Sum values for avg window
    CumSumDC[0] = CumSumDC[0] + Ch1Code;
    CumSumDC[1] = CumSumDC[1] + Ch2Code;
    CumSumDC[2] = CumSumDC[2] + Ch3Code;
    CumSumDC[3] = CumSumDC[3] + Ch4Code;

    // Calculate avg DC value for current window length
    AGCStruct.SigSum_FiltValDC[0] = CumSumDC[0] / AGCStruct.Counter_SamplesSinceStateChange;
    AGCStruct.SigSum_FiltValDC[1] = CumSumDC[1] / AGCStruct.Counter_SamplesSinceStateChange;
    AGCStruct.SigSum_FiltValDC[2] = CumSumDC[2] / AGCStruct.Counter_SamplesSinceStateChange;
    AGCStruct.SigSum_FiltValDC[3] = CumSumDC[3] / AGCStruct.Counter_SamplesSinceStateChange;

    // Calculate current offset (AC) to filtered (avg) DC value
    AGCStruct.SigSum_FiltValAC[0] = AGC_abs(AGCStruct.SigSum_FiltValDC[0] - Ch1Code);
    AGCStruct.SigSum_FiltValAC[1] = AGC_abs(AGCStruct.SigSum_FiltValDC[1] - Ch2Code);
    AGCStruct.SigSum_FiltValAC[2] = AGC_abs(AGCStruct.SigSum_FiltValDC[2] - Ch3Code);
    AGCStruct.SigSum_FiltValAC[3] = AGC_abs(AGCStruct.SigSum_FiltValDC[3] - Ch4Code);

    // Update AC and DC signals when stabilised
    if (AGCStruct.Counter_SamplesSinceStateChange == FILTERSTABILIZATIONTIME)
    {
        uint8_t counter = 0;
        for (counter = 0; counter < FILTERSTABILIZATIONTIME; counter++)
        {
            UpdateDCSignalSummary(Ch1CodeBuf[counter], Ch2CodeBuf[counter], Ch3CodeBuf[counter], Ch4CodeBuf[counter]);
            UpdateACSignalSummary(Ch1CodeBuf[counter], Ch2CodeBuf[counter], Ch3CodeBuf[counter], Ch4CodeBuf[counter]);
        }
    }
}

void AdjustAFE(void)
{
    uint8_t counter;
    for (counter = 0; counter < 2; counter++)
    {
        if (AGCStruct.Adj_Amp[counter])
        {
            AFE4404_setRf(counter + 1, AGCStruct.Settings_Amp[counter]);
        }
    }
    for (counter = 0; counter < 4; counter++)
    {
        if (AGCStruct.Adj_Off[counter])
        {
            AFE4404_setAmbientCurrent(counter + 1, AGCStruct.Settings_Off[counter]);
        }
    }
    for (counter = 0; counter < 3; counter++)
    {
        if (AGCStruct.Adj_LEDCur[counter])
        {
            AFE4404_setLedCurrent(counter + 1, AGCStruct.Settings_LEDCur[counter]);
        }
    }
    ResetAGCStruct();
}

void ResetAGCStruct(void)
{
    AGCStruct.AGCCont_UnableToIncreaseAmp = FALSE;

    AGCStruct.Adj_Amp[0] = FALSE;
    AGCStruct.Adj_Amp[1] = FALSE;

    AGCStruct.Adj_Off[0] = FALSE;
    AGCStruct.Adj_Off[1] = FALSE;
    AGCStruct.Adj_Off[2] = FALSE;
    AGCStruct.Adj_Off[3] = FALSE;

    AGCStruct.Adj_LEDCur[0] = FALSE;
    AGCStruct.Adj_LEDCur[1] = FALSE;
    AGCStruct.Adj_LEDCur[2] = FALSE;

    AGCStruct.SigSum_MaxCode[0] = 0;
    AGCStruct.SigSum_MaxCode[1] = 0;
    AGCStruct.SigSum_MaxCode[2] = 0;
    AGCStruct.SigSum_MaxCode[3] = 0;

    AGCStruct.SigSum_MinCode[0] = 0xffff;
    AGCStruct.SigSum_MinCode[1] = 0xffff;
    AGCStruct.SigSum_MinCode[2] = 0xffff;
    AGCStruct.SigSum_MinCode[3] = 0xffff;

    AGCStruct.SigSum_FiltValAC[0] = 0;
    AGCStruct.SigSum_FiltValAC[1] = 0;
    AGCStruct.SigSum_FiltValAC[2] = 0;
    AGCStruct.SigSum_FiltValAC[3] = 0;

    AGCStruct.SigSum_FiltValDC[0] = 0;
    AGCStruct.SigSum_FiltValDC[1] = 0;
    AGCStruct.SigSum_FiltValDC[2] = 0;
    AGCStruct.SigSum_FiltValDC[3] = 0;
}

void AlignAGCWithAFE(void)
{

    if (AGCStruct.Settings_currentAFEsettings != 0)
    {
        AGCStruct.Settings_Off[0] = AFE4404_settingsUintGetIsub(1, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_Off[1] = AFE4404_settingsUintGetIsub(2, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_Off[2] = AFE4404_settingsUintGetIsub(3, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_Off[3] = AFE4404_settingsUintGetIsub(4, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_Amp[0] = AFE4404_settingsUintGetRf(1, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_Amp[1] = AFE4404_settingsUintGetRf(2, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_LEDCur[0] = AFE4404_settingsUintGetLedCurrent(1, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_LEDCur[1] = AFE4404_settingsUintGetLedCurrent(2, AGCStruct.Settings_currentAFEsettings);
        AGCStruct.Settings_LEDCur[2] = AFE4404_settingsUintGetLedCurrent(3, AGCStruct.Settings_currentAFEsettings);
    }
    else
    {
        AGCStruct.Settings_Off[0] = AFE4404_getAmbientCurrent(1);
        AGCStruct.Settings_Off[1] = AFE4404_getAmbientCurrent(2);
        AGCStruct.Settings_Off[2] = AFE4404_getAmbientCurrent(3);
        AGCStruct.Settings_Off[3] = AFE4404_getAmbientCurrent(4);
        AGCStruct.Settings_Amp[0] = AFE4404_getRf(1);
        AGCStruct.Settings_Amp[1] = AFE4404_getRf(2);
        AGCStruct.Settings_LEDCur[0] = AFE4404_getLedCurrent(1);
        AGCStruct.Settings_LEDCur[1] = AFE4404_getLedCurrent(2);
        AGCStruct.Settings_LEDCur[2] = AFE4404_getLedCurrent(3);
    }
}

uint8_t AGC_getMajorVer(void)
{
    return AGC_VERSION_MAJOR;
}

uint8_t AGC_getMinorVer(void)
{
    return AGC_VERSION_MINOR;
}

uint8_t AGC_getBuildVer(void)
{
    return AGC_VERSION_BUILD;
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
