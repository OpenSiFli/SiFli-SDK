/**
  ******************************************************************************
  * @file   afe4404_hw.h
  * @author Sifli software development team
  * @brief   Control and operation functions for AFE 4404
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

#ifndef AFE4404_HW_INCLUDED
#define AFE4404_HW_INCLUDED

#include "hqerror.h"   // LifeQ error definitions file
#include "phys_calc.h"
#include "pp_config.h"
#include "agc_V3_1_19.h"
//#include "hw_config.h"

//#include "i2c_sw.h"

//#include "app_error.h"


/* Weight definitions for variable sampling */


//old
#define LED2STC         0
#define LED2ENDC        1
#define LED1LEDSTC      2
#define LED1LEDENDC     3
#define ALED2STC        1
#define ALED2ENDC       2
#define LED1STC         2
#define LED1ENDC        3
#define LED2LEDSTC      0
#define LED2LEDENDC     1
#define ALED1STC        3
#define ALED1ENDC       4
#define LED2CONVST      1
#define LED2CONVEND     2
#define ALED2CONVST     2
#define ALED2CONVEND    3
#define LED1CONVST      3
#define LED1CONVEND     4
#define ALED1CONVST     4
#define ALED1CONVEND    5
#define ADCRST0         1
#define ADCRST1         2
#define ADCRST2         3
#define ADCRST3         4
#define PRPCOUNT        40
#define LED3LEDSTC      1
#define LED3LEDENDC     2
#define PDNCYCLESTC     5
#define PDNCYCLEENDC    40


//new

#define LED2SAMP        0
#define ALED2SAMP       1
#define LED1SAMP        2
#define ALED1SAMP       3

#define LED2CONV        LED2SAMP
#define ALED2CONV       ALED2SAMP
#define LED1CONV        LED1SAMP
#define ALED1CONV       ALED1SAMP
/*
#define ADCRST0         0
#define ADCRST1         1
#define ADCRST2         2
#define ADCRST3         3
*/
#define DPDSLEEP        200
#define DPDWAKE         200

#define LEDSETTLE       20
#define LEDEND          2
#define CNVEND          2
#define RSTWIDTH        6

//#define PPS960_DEBUG
#define LIFEQ

#define ENABLE_RED_LED (0) //0:disable red LED

void Partron_UpdateSamplingRate(uint16_t freq, double duty);

typedef struct
{
    uint16_t afeMeasLED1;
    uint16_t afeMeasALED1;
    uint16_t afeMeasLED2;
    uint16_t afeMeasALED2;
    uint16_t afeDiffLED1;
    uint16_t afeDiffLED2;
    uint8_t LifeQhrm ;
    uint32_t afeMeasLED2MALED2;
    uint32_t afeMeasLED1MALED1;
} afe_data_struct_t;

typedef struct
{
    int32_t afeMeasLED1;
    int32_t afeMeasALED1;
    int32_t afeMeasLED2;
    int32_t afeMeasALED2;
    int32_t afeDiffLED1;
    int32_t afeDiffLED2;
    uint32_t afeMeasLED2MALED2;
    uint32_t afeMeasLED1MALED1;
} afe_32_bit_data_struct_t;

typedef struct
{
    uint32_t afeMeasLED1;
    uint32_t afeMeasALED1;
    uint32_t afeMeasLED2;
    uint32_t afeMeasALED2;
    uint32_t afeMeasLED2MALED2;
    uint32_t afeMeasLED1MALED1;
} afe_32_RAW_bit_data_struct_t;

typedef struct
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;

} specific_force_t;


typedef struct
{
    int8_t tempIsubtimesTen1;
    int8_t tempIsubtimesTen2;
    int8_t tempIsubtimesTen3;
    int8_t tempIsubtimesTen4;
    uint16_t tempRf1;
    uint16_t tempRf2;
    uint8_t tempcf1;
    uint8_t tempcf2;
    uint8_t ledCurrent1;
    uint8_t ledCurrent2;
    uint8_t ledCurrent3;

} afe_data_etc_t;

typedef struct
{
    uint8_t only_read_ppg[24];
    uint8_t ledCurrent1;
    uint8_t ledCurrent2;
    uint8_t ledCurrent3;
    int8_t tempIsubtimesTen1;
    int8_t tempIsubtimesTen2;
    int8_t tempIsubtimesTen3;
    int8_t tempIsubtimesTen4;
    uint16_t rfvalue1;
    uint16_t rfvalue2;
    uint8_t cfvalue1;
    uint8_t cfvalue2;

} tx_data_t;


/* TI AFE4404 Silicon Revision Identifier codes  */

typedef enum
{
    AFE4404_REV_A = 0x04,
    AFE4404_REV_B = 0x14,
    AFE4404_REV_UNKNOWN = 0x00
} afe_silicon_rev_t;


/**
 * @brief Retrieve the AFE4404 chips silicon revision.
 *      Important due to tolerance differences between chips
 *
 * @return silicon revision number as specified in afe_silicon_rev_t [A, B, unknown]
 */
afe_silicon_rev_t AFE4404_getSiliconRivision(void);


/**
 * @brief Deprecated function used to initialise the AFE4404
 * Please use AFE4404_initFs()
 *
 * @return RET_OK
 */
hqret_t AFE4404_init_Q(void);


/**
 * @brief Deprecated function which Initialises the AFE4404 with specific settings using the internal clock
 * Please use AFE4404_initFsWithExternalClk() which allows selecting either internal or external clocks
 *
 * @param freqTimesTen - Sampling frequency of the AFE4404 - 25 Hz and 50 Hz operation validated
 * @param samplePeriod - Period in microseconds that a channel is emitting (on) during a sampling instance (Duty Cycle)
 * @param numAve - The amount of ADC conversions to average over. Range from [0, 15] where 0 equals no averaging
 * @param allowDPD - Allow dynamic power down. Only applicable from rev. B onward.
 *                   Power down the AFE, ADC, etc. between two cycles of emitting and sampling LEDs
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_initFs(uint16_t freqTimesTen,
                       uint16_t samplePeriod,
                       uint8_t numAve,
                       uint8_t allowDPD);


/**
 * @brief Initialises the AFE 4404 with the option of using an external clock for timing
 *
 * @param freqTimesTen - Sampling frequency of the AFE4404 - 25 Hz and 50 Hz operation validated
 * @param samplePeriod - Period in microseconds that a channel is emitting (on) during a sampling instance (Duty Cycle)
 * @param numAve - The amount of ADC conversions to average over. Range from [0, 15] where 0 equals no averaging
 * @param allowDPD - Allow dynamic power down. Only applicable from rev. B
 *      onward. Power down the AFE, ADC, etc. between two cycles of
 *      emitting and sampling LEDs
 * @param useExternalClock - Enable external clock usage for timing
 *
 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 * Typical configurations:
 *          25 Hz - AFE4404_initFsWithExternalClk(250, 500, 0, true,true)
 *          50 Hz - AFE4404_initFsWithExternalClk(500, 500, 0, true,true)
 */
hqret_t AFE4404_initFsWithExternalClk(uint16_t freqTimesTen,
                                      uint16_t samplePeriod,
                                      uint8_t numAve,
                                      uint8_t allowDPD,
                                      uint8_t useExternalClock);


/**
 * @brief Enable the AFE's internal timing engine which generates all the clock
 *  phases for synchronised transmit drive, receive sampling and data
 *  conversion.
 *
 */
void AFE4404_enableInternalTimer(void);


/**
 * @brief Disable the AFE's internal timing engine; effectively disabling the
 *      capability of emitting and sampling of LEDs
 */
void AFE4404_disableAFE(void);


/**
 * @brief Resets the AFE according to the datasheet timing specifications
 *
 * #PARTNER_TODO - Replace internal pin pull up / down fnuction with correct IOPIN for partner set up
 *
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_hwReset(void);


/**
 * @brief Power down the AFE according to the pin input settings and timing
 *      specifications as provided on the datasheet
 *
 * #PARTNER_TODO - Replace internal pin pull up / down fnuction with correct IOPIN for partner set up
 *
 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_hwShutDown(void);


/**
 * @brief Power on the AFE according to the pin input settings and timing
 *      specifications as provided on the datasheet
 *
 * #PARTNER_TODO - Replace internal pin pull up / down fnuction with correct IOPIN for partner set up
 *
 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_hwPowerOn(void);


/**
 * @brief Test communication with the AFE by writing a specific value to a specific register and ready that register.
 *
 * @return True if the value is the same; False otherwise
 */
hqret_t AFE4404_testComs(void);


/**
 * @brief Trim / Tune the AFE internal timing register for accurate clock
 *      calibration
 *
 * @param trimValue - Value between 0 - 127 to tune the AFE's OSC
 *
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_adjustOscTuningRegister(uint8_t trimValue);


/**
 * @brief Configure the OSC with default values as stated in
 *  afe4404_init_registers
 */
void AFE4404_setOsc(void);


/**
 * @brief Retrieve AFE values and populate raw 32bit, 32bit and 16bit data structures
 *
 * @param afe_RAW_32bitstruct - Pointer to where to store the raw data received
 * @param afe_32bitstruct - Pointer to where to store the converted 32bit data
 *              received
 * @param afe_struct - Pointer to where to store the converted 16bit data
 *          received
 */
void AFE4404_retrieveRawAFEValues(afe_32_RAW_bit_data_struct_t *afe_RAW_32bitstruct, afe_32_bit_data_struct_t *afe_32bitstruct, afe_data_struct_t *afe_struct);


/**
 * @brief Retrieve the LED measurement values form the AFE4404 and store them
 *      in both 32 and 16 bit data representations
 *
 * @param afe_32bitstruct - Pointer to 32bit afe data structure to store retrieved values
 * @param afe_struct - Pointer to 16bit afe data structure to store retrieved values
 */
void AFE4404_retrieve32BitAFEValues(afe_32_bit_data_struct_t *afe_32bitstruct, afe_data_struct_t *afe_struct);


/**
 * @brief Retrieve the LED measurement values form the AFE4404 and store them
 *      in a 16 bit data representation
 *
 * @param afe_struct - Pointer to 16bit afe data structure to store retrieved values
 */
void AFE4404_retrieve16BitAFEValues(afe_data_struct_t *afe_struct);


/**
 * @brief Converts a 32bit afe value in two's complement to a normal 32bit value
 *
 * @param val - An afe value to be converted
 * @return Converted value
 */
int32_t afeConvP32(int32_t val);


/**
 * @brief Converts a 32bit afe value (two's complement) while checking if it is a valid ADC
 *      captured value to 16bit value
 *
 * @param val - A value formatted in two's complement
 * @return Converted value
 */
uint16_t afeConvP16(int32_t val);


/**
 * @brief Enables reading from the AFE
 */
void AFE4404_enableRead(void);


/**
 * @brief Enables reading from the AFE before sending a read request
 *
 * @param reg - The register to read
 *
 * @return The value received
 */
uint32_t AFE4404_readRegWithReadEnable(uint8_t reg);


/**
 * @brief Force a read command from the register specified
 *
 * @param reg - Register to read on the AFE
 * @return The value obtained from the request
 */
uint32_t AFE4404_readRegWithoutReadEnable(uint8_t reg);


/**
 * @brief Enables writing to the AFE
 */
void AFE4404_enableWrite(void);


/**
 * @brief Enables writing to the AFE before sending data
 *
 * @param reg - The register to write to
 * @param registerValue - The value to be written
 */
void AFE4404_writeRegWithWriteEnable(uint8_t reg, uint32_t registerValue);


/**
 * @brief Force a write command to the register specified
 *
 * @param reg - The register to write to
 * @param registerValue - The value to write
 */
void AFE4404_writeRegWithoutWriteEnable(uint8_t reg, uint32_t registerValue);


/**
 * @brief Set the rf value of the TIA at rfNum in the AFEWriteQueue
 *
 * @param rfNum - The rf channel to set
 * @param rfValueInKiloOhms - A valid value for the rf gain setting
 *      [500, 250, 100, 50, 25, 10, 1000, 2000] kOhm
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_setRf(uint8_t rfNum, uint16_t rfValueInKiloOhms);


/**
 * @brief Set the RF gain settings bypassing the command queue
 *
 * @param rfNum - rf channel to set
 * @param rfValueInKiloOhms - A valid value for the rf gain setting
 *      [500, 250, 100, 50, 25, 10, 1000, 2000] kOhm
 *
 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_directSetRf(uint8_t rfNum, uint16_t rfValueInKiloOhms);


/**
 * @brief Get the value stored at rfNum from the liveAFERegister
 *
 * @param rfNum - RF slot number of the value to return
 * @return RF value at rfNum in discrete range as specified in AFE4404_setRf's
 *      description
 */
uint16_t AFE4404_getRf(uint8_t rfNum);


/**
 * @brief Increment the rf setting of the channel in question
 *
 * @param channel - The rf channel [1, 2] on the TIA to increment
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_incrementRf(uint8_t channel);


/**
 * @brief Decrement the rf setting of the channel in question
 *
 * @param channel - The rf channel [1, 2] on the TIA to decrement
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_decrementRf(uint8_t channel);


/**
 * @brief Set the cf value of the TIA for cfNum in the AFEWriteQueue
 *
 * @param cfNum - The cf channel to set
 * @param cfValueInPicoFarhadsTimes10 - A valid value for the cf setting
 *      [5.0, 2.5, 10.0, 7.5, 20.0, 17.5, 25.0, 22.5] pF. (Note these
 *      values need to be multiplied by 10)
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_setCf(uint8_t cfNum, uint8_t cfValueInPicoFarhadsTimes10);


/**
 * @brief Get the value stored at cf(x) from the liveAFERegister
 *
 * @param cfNum - CF slot number of the value to return
 * @return CF value at cfNum in discrete range as specified in AFE4404_setCf's
 *      description
 *
 */
uint8_t AFE4404_getCf(uint8_t cfNum);


/**
 * @brief Set the ambient current (Isub) to subtract for a specific channel in the AFEWriteQueue
 *
 * @param channel - The channel on which to subtract the current value provided
 * @param currentInMicroAmpsTimes10 - The current value x 10 to subtract
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_setAmbientCurrent(uint8_t channel, int8_t currentInMicroAmpsTimes10);


/**
 * @brief Set the ambient current (Isub) value directly, bypassing the command queue
 *
 * @param channel - The channel on which to set the ambient current
 * @param currentInMicroAmpsTimes10 - The current in mA x 10 to set

 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_directSetAmbientCurrent(uint8_t channel, int8_t currentInMicroAmpsTimes10);


/**
 * @brief Get the ambient current for a specified channel from the liveAFERegister
 *
 * @param channel - Channel number of the stored ambient current to return
 * @return
 */
int8_t AFE4404_getAmbientCurrent(uint8_t channel);


/**
 * @brief Increment the ambient subtraction current (Isub) of the channel in question
 *
 * @param channel - The channel [1, 2, 3, 4] on which to increment the ambient subtraction current
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_incrementAmbientSubCurrent(uint8_t channel);


/**
 * @brief Decrement the ambient subtraction current (isub) of the channel in question
 *
 * @param channel - The channel [1, 2, 3, 4] on which to decrement the ambient subtraction current
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_decrementAmbientSubCurrent(uint8_t channel);


/**
 * @brief Evaluates the value specified against the discrete steps available for Isub
 *
 * @param currentInMicroAmpsTimes10 - The current x 10 to be tested
 *
 * @return RET_OK if the current is a valid discrete value and RET_FAIL otherwise
 */
uint8_t AFE4404_isIsubValid(int8_t currentInMicroAmpsTimes10);


/**
 * @brief Set the current provided to a specific LED channel in the AFEWriteQueue
 *
 * @param ledNumber - Channel (LED) number of which the current will be set
 * @param currentTapSetting - The setting related to a specific discrete current as specified on TI datasheet
 *
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_setLedCurrent(uint8_t ledNumber, uint8_t currentTapSetting);


/**
 * @brief Set the led current tap setting of the channel specified and ensure
 *      that the duty cycle limitation is met
 *
 * @param ledNumber - LED channel number to set
 * @param currentTapSetting - The setting related to a specific discrete current
 *              as specified on TI datasheet
 *
 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_setLedCurrentWithDutyCycleLimitation(uint8_t ledNumber, uint8_t currentTapSetting);


/**
 * @brief Set the LED current directly, bypassing the command queue
 *
 * @param ledNumber - The channel on which to change / set the current
 * @param currentTapSetting - The setting related to a specific discrete current
 *              as specified on TI datasheet

 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_directSetLedCurrent(uint8_t ledNumber, uint8_t currentTapSetting);


/**
 * @brief Get the current tap value of a specific LED channel from the liveAFERegister
 *
 * @param ledNumber - The channel (LED) number of which to get the current tap value
 * @return The current tap setting which can be converted to current by table as specified by TI
 */
uint8_t AFE4404_getLedCurrent(uint8_t ledNumber);


/**
 * @brief Increment the led current of the channel in question
 *
 * @param ledNumber - The LED channel [1, 2, 3] on which to increment the current
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_incrementLedCurrent(uint8_t ledNumber);


/**
 * @brief Decrement the led current of the channel in question
 *
 * @param ledNumber - The LED channel [1, 2, 3] on which to decrement the current
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_decrementLedCurrent(uint8_t ledNumber);


/**
 * @brief Allow the AFE4404 to go into high power mode, allowing 100mA LED current
 *
 * @param highPowerEnable - True to enable, False to disable
 */
void AFE4404_setMaxCurrentMode(uint8_t highPowerEnable);


/**
 * @brief Adds a command to the AFE command queue for transmission when deemed ready
 *
 * @param reg - The register number to write to on the AFE4404 chip in decimal value
 * @param registerValue - The value to be written to the specified register
 */
void AFE4404_addWriteCommandToQueue(uint8_t reg, uint32_t registerValue);


/**
 * @brief Check if there are any commands in the AFEWriteQueue and transmit them to the AFE
 */
void AFE4404_serviceAFEWriteQueue(void);


/**
 * @brief Get a value from a local array which resembles the value as stored on the AFE
 *
 * @param reg - The register to be read from
 * @return The value currently stored in the live array
 */
uint32_t AFE4404_getRegisterFromLiveArray(uint8_t reg);


/**
 * @brief Get the current AFE settings array from the live array
 *      [0 - 5]   - LED 1 current
 *      [6 - 11]  - LED 2 current
 *      [12 - 17] - LED 3 current
 *      [18 - 37] - Ambient subtraction currents
 *      [38 - 40] - RF1
 *      [41 - 43] - RF2
 *
 * @param dataArr - address where the AFE settings should be stored
 */
void AFE4404_getAFESettingsArr(uint8_t *dataArr);


/**
 * @brief Get the current AFE settings from the live array in a uint64 type
 *      [0 - 5]   - LED 1 current
 *      [6 - 11]  - LED 2 current
 *      [12 - 17] - LED 3 current
 *      [18 - 37] - Ambient subtraction currents
 *      [38 - 40] - RF1
 *      [41 - 43] - RF2
 *
 * @return uint64 as specified above
 */
uint64_t AFE4404_getAFESettingsUint(void);


/**
 * @brief Extracts the rf setting for the applicable channel specified from a
 *      64bit AFE settings array
 *
 * @param channel - The rf channel number
 * @param inputArr - A 64 bit uint AFE settings array
 *
 * @return RF value for channel specified
 */
uint16_t AFE4404_settingsUintGetRf(uint8_t channel, uint64_t inputArr);


/**
 * @brief Extracts the Isub setting for the applicable channel specified from a
 *      64bit AFE settings array
 *
 * @param channel - The isub channel
 * @param inputArr - A 64 bit uint AFE settings array
 *
 * @return Isub value for channel specified
 */
int8_t AFE4404_settingsUintGetIsub(uint8_t channel, uint64_t inputArr);


/**
 * @brief Extracts the LED setting for the applicable channel specified from a
 *      64bit AFE settings array
 * @param ledNumber - The LED channel in question
 * @param inputArr - A 64 bit uint AFE setting
 *
 * @return The current setting of LED requested
 */
uint8_t AFE4404_settingsUintGetLedCurrent(uint8_t ledNumber, uint64_t inputArr);


/**
 * @brief Check if the AFE settings have changed
 *
 * @return True if changes occurred; False if not
 */
uint8_t AFE4404_settingsChanged(void);


/**
 * @brief Set the number of ADC conversion to average over
 *
 * @param numAvgs - Range from [0, 15] where 0 equals no averaging
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_setNumberOfAverages(uint8_t numAvgs);


/**
 * @brief Retrieve the averaging number of ADC
 *
 * @return uint8 representing the (averaging number - 1)
 */
uint8_t AFE4404_getNumberOfAverages(void);


/**
 * @brief Decrement the ADC's number of averages
 *
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_decrementNumberOfAverages(void);


/**
 * @brief Increment the ADC's number of averages
 *
 * @return RET_OK if successful or RET_FAIL otherwise
 */
hqret_t AFE4404_incrementNumberOfAverages(void);


/**
 * @brief Adjust the pulse repetition period. Caution should be used with this
 *      function as it may set the period to small
 *
 * @param updatedPrpValue - The new pulse repetition period to set on the AFE
 *
 * @return RET_OK if the function terminated successfully and RET_FAIL otherwise
 */
hqret_t AFE4404_adjustPRPAndPowerDownCycles(uint16_t updatedPrpValue);


/**
 * @brief Retrieve the current pulse repetition period used by the AFE
 *
 * @return The prp value
 */
uint32_t AFE4404_getPrpInUse(void);

void PPS960_writeReg(uint8_t regaddr, uint32_t wdata);
uint32_t PPS960_readReg(uint8_t regaddr);
void LIFEQTWI_writeReg(uint8_t regaddr, uint32_t wdata);
uint32_t LIFEQTWI_readReg(uint8_t regaddr);
void AFE4404_UpdateSamplingRate(uint16_t freq, uint16_t duty, uint16_t clkdiv);
void ALGSH_retrieveSamplesAndPushToQueue(void);
void init_PPS960_register(void);
void PPS960_init(void);
void ALGSH_dataForAgc(void);
void ALGSH_dataToAlg(void);
uint16_t GetLEDBuffLenth(void);
uint16_t GetAccBuffLenth(void);
uint16_t GetHRSampleCount(void);
void ClrHRSampleCount(void);
uint16_t pps_getHR(void);
void PPS_skin_detect(led_sample_t irLed);
uint8_t PPS_get_skin_detect(void);
void PPS_Gled_on_by_skin_etect(uint8_t onSkin);
void PPS_Gled_auto_ctrl(led_sample_t grnLed);
void PPS960_writeReg(uint8_t regaddr, uint32_t wdata);
uint32_t PPS960_readReg(uint8_t regaddr);
//void twi_init (void);
// void twi_config(void);
//ret_code_t twi_master_init(void);
//ACC use FIFO,its raw data will be read by timer, timer period about 1s.
//#define PPS_ACC_USE_FIFO (1)

#endif  // AFE4404_HW_INCLUDED/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
