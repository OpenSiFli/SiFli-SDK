/**
  ******************************************************************************
  * @file   agc_V3_1_19.h
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

#ifndef AGC_V3_H_
#define AGC_V3_H_

#define AGC_VERSION_MAJOR (uint8_t) 3
#define AGC_VERSION_MINOR (uint8_t) 1
#define AGC_VERSION_BUILD (uint8_t) 19

#include <stdint.h>
//#include <stdbool.h>
//#include <stdint.h>
//#include "hw_config.h"
#include "hqerror.h"

/*
#ifdef POST
#include "hqerror.h"
#endif

#ifdef EMBEDDED
#include "hqerror.h"
#endif
*/


#ifndef bool
    #define bool unsigned char
#endif

typedef enum
{
    AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT = 0,
    AGC_STATE_CALIBRATION = 1,
    AGC_STATE_CALIBRATION_ERRORSTATE = 2,
    AGC_STATE_WAITINGFORADJUSTMENT = 3,
    AGC_STATE_INITIALIZEFILTER = 4,
    AGC_STATE_FILTERSTABLE = 5,
    AGC_STATE_ERRORSTATE = 6
} agc_state_t;

typedef enum
{
    CHANNEL_STATE_SATURATELOW = 0,
    CHANNEL_STATE_SATURATEHIGH = 1,
    CHANNEL_STATE_OUTOFLOWERBOUND = 2,
    CHANNEL_STATE_OUTOFUPPERBOUND = 3,
    CHANNEL_STATE_STABLE = 4
} agc_channel_state_t;

typedef enum
{
    CALIB_STATE_INIT_SETTINGS = 0,
    CALIB_STATE_BIN_SEARCH_0 = 1,
    CALIB_STATE_BIN_SEARCH_1 = 2,
    CALIB_STATE_SELECT_LED_CURRENT = 3,
    CALIB_STATE_SELECT_OFF = 4,
    CALIB_STATE_SELECT_AMP = 5,
    CALIB_STATE_ENSURENONSATURATION = 6,
    CALIB_STATE_CALIBRATED = 7
} agc_calib_state_t;

typedef struct
{
    bool AGCCont_Off;
    bool AGCCont_Amp;
    bool AGCCont_UnableToIncreaseAmp;
    bool AGCCont_LEDCur[3];
    agc_state_t AGCCont_AGCState;
    agc_calib_state_t AGCCont_Calib_State;

    int32_t SigSum_FiltValDC[4];
    int32_t SigSum_FiltValAC[4];
    uint16_t SigSum_MaxCode[4];
    uint16_t SigSum_MinCode[4];
    agc_channel_state_t SigSum_ChannelState[4];

    uint16_t Settings_Amp[2];
    uint8_t Settings_Amp_Pos[2];
    int8_t Settings_Off[4];
    uint8_t Settings_Off_Pos[4];
    uint8_t Settings_LEDCur[3];
    uint64_t Settings_currentAFEsettings;
    uint64_t Settings_oneSampleAgoAFESettings;
    uint64_t Settings_twoSamplesAgoAFESettings;

    bool Adj_Amp[2];
    bool Adj_Off[4];
    bool Adj_LEDCur[3];

    uint32_t Counter_SamplesSinceStateChange;
} agc_struct_t;

/**
 *  \brief Sets the sampling frequency the AGC expects
 *
 *  \param [in] Sampling frequency can be either 25.0 or 50.0
 *
 *  \details Sets the sampling frequency the AGC expects to either 25 Hz or 50Hz
 */
hqret_t AGC_SetSamplingFrequency(float SamplingFrequency);

/**
 *  \brief Sets whether the AGC attempts to control LED current for each of the individual channels
 *
 *  \param [in] ChannelNumber can be either 1, 3 or 4 since channel 2 is associated with ambient measurements
 *  \param [in] ChannelLEDControl can be either true or false
 *
 *  \details If LED control is turned off, the AGC continues attempting to control values read from AFE, but does so without changing LED current any longer
 */
hqret_t AGC_SetChannelLEDControl(uint8_t ChannelNumber, bool ChannelLEDControl);

/**
 *  \brief Sets whether the AGC attempts to control Isub
 *
 *  \param [in] OffControl can be either true or false
 *
 *  \details If OffControl is turned off, the AGC continues attempting to control values read from AFE, but does so without changing Isub any longer
 */
void AGC_SetOffControl(bool OffControl);

/**
 *  \brief Sets whether the AGC attempts to control Amp
 *
 *  \param [in] AmpControl can be either true or false
 *
 *  \details If AmpControl is turned off, the AGC continues attempting to control values read from AFE, but does so without changing Rf any longer
 */
void AGC_SetAmpControl(bool AmpControl);

/**
 *  \brief Turns one of the LEDs off
 *
 *  \param [in] ChannelNumber can be either 1, 3 or 4 since channel 2 is associated with ambient measurements
 *
 *  \details If an LED is turned off, the LED current is set to zero and the corresponding LED control is turned off
 */
hqret_t AGC_TurnLEDOff(uint8_t ChannelNumber);

/**
 *  \brief Turns one of the LEDs on
 *
 *  \param [in] ChannelNumber can be either 1, 3 or 4 since channel 2 is associated with ambient measurements
 *
 *  \details If an LED is turned on, the LED control is turned back on and the AGC is set to re initialize
 */
hqret_t AGC_TurnLEDOn(uint8_t ChannelNumber);

/**
 *  \brief Initialises the AGC state machine
 *
 *  \param [in] IsubControlFlag selects Isub control
 *  \param [in] RfControlFlag selects Rf control
 *  \param [in] LEDControlFlag Selects led control and initialises the state machine in calibration mode if TRUE or filter initialisation mode if FALSE
 *
 *  \details Initialises the variables needed by the AGC and sets AFE parameters
 */
void AGC_InitializeAGC(void);

/**
 *  \brief FOR FUTURE IMPLEMETATON
 *
 *  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
 *  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
 *  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
 *  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
 *  \param [in] AFESettings 64 bit value read from AFE indicating the AFE settings
 *  \return State of the AGC State machine
 *
 *  \details Call AGC_ServiceAGCWithAFESettings(...). Rest TBA
 */
agc_state_t AGC_ServiceAGCWithAFESettings(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code, uint64_t AFESettings);

/**
 *  \brief Service the AGC with 4 channel values provided
 *
 *  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
 *  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
 *  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
 *  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
 *  \param [in] AdjustmentMade Flag stating if an adjustment has been made
 *  \return State of the AGC State machine
 *
 *  \details AGC_ServiceAGC contains and services the AGC state machine. the different states are:
 *          AGC_STATE_CALIBRATION
 *          AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT
 *          AGC_STATE_CALIBRATION_ERRORSTATE
 *          AGC_STATE_INITIALIZEFILTER
 *          AGC_STATE_WAITINGFORADJUSTMENT
 *          AGC_STATE_FILTERSTABLE
 *          AGC_STATE_ERRORSTATE
 *      Normal transitioning of the state machine include initialisation into either AGC_STATE_CALIBRATION or AGC_STATE_INITIALIZEFILTER states.
 *          If AGC_STATE_CALIBRATION, adjustments will be made causing a transition to AGC_STATE_CALIBRATION_WAITINGFORADJUSTMENT and
 *          back until calibrated. If an error occurs it will transition to AGC_STATE_CALIBRATION_ERRORSTATE, the AGC will be realigned with the AFE
 *          before continuing calibration otherwise to AGC_STATE_INITIALIZEFILTER or AGC_STATE_WAITINGFORADJUSTMENT depending if the
 *          final step in the calibration required a change in the AFE settings.
 *          In AGC_STATE_INITIALIZEFILTER if adjustments need to be made it will transition to AGC_STATE_WAITINGFORADJUSTMENT or
 *          AGC_STATE_FILTERSTABLE depending if a stable filter state has been reached, alternatively the state machine will transition to
 *          AGC_STATE_ERRORSTATE if an unexpected adjustment occurred. If the filter has not stabilised an adjustment will be made transitioning to
 *          AGC_STATE_WAITINGFORADJUSTMENT state, else a stable condition has been reached allowing transition to AGC_STATE_FILTERSTABLE. In
 *          AGC_STATE_FILTERSTABLE the filter will update necessary signals and check conditions for saturation, bounds or the necessity for
 *          re-evaluation. Upon recommendation of an AFE adjustment, transition to AGC_STATE_WAITINGFORADJUSTMENT will occur.
 *
 */
agc_state_t AGC_ServiceAGC(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code, bool AdjustmentMade);

/**
*  \brief Aligns AGC with AFE and reinitialises calibration state
*
*  \return NA
*/
void AGC_Recalibrate(void);

/**
*  \brief Updates min and max values for channe
*
*  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
*  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
*  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
*  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
*  \return NA
*/
void UpdateMinMax(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code);

/**
 *  \brief Check and report channel state
 *
*  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
*  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
*  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
*  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
 *  \return NA
 */
void UpdateSignalState(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code);

/**
*  \brief Initialise or update DC signal
*
*  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
*  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
*  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
*  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
*  \return NA
*/
void UpdateDCSignalSummary(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code);

/**
*  \brief Initialise or update AC signal
*
*  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
*  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
*  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
*  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
*  \return NA
*/
void UpdateACSignalSummary(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code);

/**
*  \brief Brief
*
*  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
*  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
*  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
*  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
*  \return TRUE if calibration adjustment is necessary, FALSE if not
*
*
*  \details Functions runs the calibration state machine, consisting of the following states:
*       CALIB_STATE_INIT_SETTINGS
*       CALIB_STATE_BIN_SEARCH_0
*       CALIB_STATE_BIN_SEARCH_1
*       CALIB_STATE_SELECT_LED_CURRENT
*       CALIB_STATE_SELECT_OFF
*       CALIB_STATE_SELECT_AMP
*       CALIB_STATE_ENSURENONSATURATION
*       CALIB_STATE_CALIBRATED
*   State machine starts in CALIB_STATE_INIT_SETTINGS, setting all the initial values for amp, off and LEDcur, and transitions to
*       CALIB_STATE_BIN_SEARCH_0. CALIB_STATE_BIN_SEARCH_0 and CALIB_STATE_BIN_SEARCH_1 checks if initial / current
*       conditions cause to high or low channel values, adjusts accordingly and transitions to CALIB_STATE_SELECT_LED_CURRENT.
*       CALIB_STATE_SELECT_LED_CURRENT calculates and sets LEDcur. Thereafter there is a transition to CALIB_STATE_SELECT_OFF which
*       adjusts offset and transitions to CALIB_STATE_SELECT_AMP. CALIB_STATE_SELECT_AMP sets amp if pre and post calibration
*       values aren't the same, transitions to CALIB_STATE_ENSURENONSATURATION where saturation is checked and if not saturated,
*       transitions to CALIB_STATE_CALIBRATED.
*/
bool CheckCalibrationAdjustment(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code);

/**
 *  \brief Check and adjust offset value if within a valid range
 *
 *  \return TRUE if an adjustment is recommended, FALSE otherwise
 */
bool CheckOffAdjustement(void);

/**
 *  \brief Check and adjust Amp values if it's within a valid range
 *
 *  \return TRUE if an adjustment is recommended, FALSE otherwise
 */
bool CheckAmpAdjustment(void);

/**
 *  \brief Check and adjust LED current values if it's within a valid range
 *
 *  \return TRUE if an adjustment is recommended, FALSE otherwise
 */
bool CheckLEDCurAdjustement(void);

/**
 *  \brief Suggests an adjustment in the case of signal saturation
 *
 *  \return TRUE if an adjustment is recommended, FALSE otherwise
 */
bool CheckSaturationAdjustment(void);

/**
 *  \brief Suggests an adjustment for a single channel in the case of that channel saturating
 *
 *  \param [in] ChannelNumber Channel that has saturated
 *  \return  TRUE if an adjustment is recommended, FALSE otherwise
 *
 *  \details Checks if channel signal is saturated HIGH or LOW and recommends an AFE adjustment.
 */
bool CheckChannelSaturationAdjustment(uint8_t ChannelNumber);

/**
 *  \brief Check all signals for saturation
 *
 *  \return TRUE if any signal is saturated, FALSE otherwise
 */
bool SignalSaturated(void);

/**
 *  \brief Check if all signals are in bounds
 *
 *  \return TRUE if in all channels in bounds, FALSE otherwise
 */
bool SignalInBounds(void);

/**
 *  \brief Checks amount of samples acquired from last evaluation and re-evaluates the Rf setting if necessary
 *
 *  \return TRUE if sufficient time has passed since last evaluation of Rf, FALSE otherwise
 */
bool ReEvaluateAC(void);

/**
 * \brief Check high condition
 *
 *  \param [in] ChannelNumber Channel to check
 *  \return TRUE if too high, FALSE otherwise
 */
bool ChannelTooHigh(uint8_t ChannelNumber);

/**
 *  \brief Check low condition
 *
 *  \param [in] ChannelNumber Channel to check
 *  \return TRUE if too low, false otherwise
 */
bool ChannelTooLow(uint8_t ChannelNumber);

/**
 *  \brief Update filter values for initialisation
 *
*  \param [in] Ch1Code LED value on channel 1, HealthQ config = LED1 = green
*  \param [in] Ch2Code LED value on channel 2, HealthQ config = LED1A = ambient
*  \param [in] Ch3Code LED value on channel 3, HealthQ config = LED2 = red
*  \param [in] Ch4Code LED value on channel 4, HealthQ config = LED2A = IR
 *  \return NA
 *
 *  \details Receives the current measurements for channels 1, 2, 3 and 4. Builds a buffer of samples and uses these to calculate accurate initial values
 *  for the calculation DC and AC components of each channel. Once complete, the signal component values for each channel are updated.
 */
void InitializeFilter(uint16_t Ch1Code, uint16_t Ch2Code, uint16_t Ch3Code, uint16_t Ch4Code);

/**
 *  \brief Adjust AFE amp, offset and / or LED current as indicated for the appropriate channel(s)
 *
 *  \return NA
 */
void AdjustAFE(void);

/**
 *  \brief Defaults all variables in AGCStruct
 *
 *  \return NA
 */
void ResetAGCStruct(void);

/**
 *  \brief Aligns the AGC variables with that of the AFE
 *
 *  \return NA
 *
 *  \details Sets AGC to either the current values stored in the AFE or the current measurements acquired from the AFE
 */
void AlignAGCWithAFE(void);

/**
 *  \brief Sets the AGC state
 *
 *  \return NA
 *
 *  \details Sets AGC state @see agc_state_t
 */
void AGC_setAgcState(agc_state_t setState);



uint8_t AGC_getMajorVer(void);
uint8_t AGC_getMinorVer(void);
uint8_t AGC_getBuildVer(void);
#endif /* AGC_V3_H_ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
