/**
  ******************************************************************************
  * @file   hqerror.h
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

#ifndef HQERR_H_
#define HQERR_H_

// SAFE_CALL is a convenient macro to call a function and exit with the relevant failure code if the function failed
#define     SAFE_CALL(x)                    {hqret_t call_return = (x) ; if (call_return) return call_return;}

/**
 The following enum defines error codes that should be
 used if errors are detected or reported.
 These error codes will be used across the different compilers
 and processors (up to the server). Since enums sizes are not the
 same on all compilers, care must be taken if the enum is stored
 in a file or transmitted via a link.
 The assumption is therefor made that enums are 2 bytes, 16-bits,
 when saved or transmitted. It is therefore good practice to always
 cast the enum to an unsigned 16 bit value before saving or transmitting it.
 e.g. (uint16_t) ERR_COMMS_TIMEOUT;
 In cases where only  8 bits are available the least 8 significant bits should be used.
 The upper 8 bits are free for the function to use for what ever purpose except for
 the first 10 enums where the 8 most significant bits should always be 0.
*/


typedef enum
{
    // return values - in most cases to indicate success or failure
    RET_OK                           = 0,   // default return on success
    RET_FAIL                         = 1,
    RET_YES                          = 2,
    RET_NO                           = 3,
    RET_CANCEL                       = 4,
    RET_IDLE                         = 5,
    RET_TASK_STARTED_OK              = 6,   // return when a task was started sucessfully, e.g. disk write, but the returned before completion - might therefore still fail
    RET_NOT_INITIALIZED              = 7,

    RET_DATA_AVAILABLE               = 9,
    RET_DATA_CURRENTLY_NOT_AVAILABLE = 10,

    // data related errors
    HQ_ERR_DATA_NOT_AVAILABLE       = 11,
    HQ_ERR_DATA_INVALID             = 12,
    HQ_ERR_NUM_OVERFLOW             = 13,
    HQ_ERR_DATA_OUT_OF_RANGE        = 14,
    HQ_ERR_MEM_CRC                  = 15,
    HQ_ERR_DATA_ARR_TOO_LONG        = 16,
    HQ_ERR_DATA_WRONG_FORMAT        = 17,
    HQ_ERR_DATA_NOT_INITIALIZED     = 18,
    HQ_ERR_PROCESSING_INCOMPLETE    = 19,   // // eg if still busy with a calculation


    // memory related errors
    HQ_ERR_MEM_NOT_INITIALIZED      = 20,
    HQ_ERR_NULL_POINTER             = 21,
    HQ_ERR_OUT_OF_MEM               = 22,
    //ERR_OUT_OF_MEM           = 22,  // for compatibility only and will be removed. Use the error code above.
    HQ_ERR_MEMORY                   = 23,
    HQ_ERR_ARR_SIZE_NULL            = 24,
    HQ_ERR_ARR_OUT_OF_BOUNDS        = 25,
    //ERR_ARR_OUT_OF_BOUNDS    = 25,  // for compatibility only and will be removed. Use the error code above.
    HQ_ERR_BUFFER_FULL              = 26,
    HQ_WARN_MEM_LOW                 = 129,  // last operation was sucessfull but indicate NAND/harddrive/SD almost full - will be depricated when using circular writes

    // hardware related errors
    HQ_ERR_HW_FAILURE               = 31,
    HQ_ERR_SENSOR_FAILURE           = 32, //Only zeroes returned from AFE
    HQ_ERR_UNDER_VOLTAGE            = 33,
    HQ_ERR_OVER_VOLTAGE             = 34,
    HQ_ERR_CURRENT_TRIP             = 35,
    HQ_ERR_CALIBRATION_FAILED       = 36,
    HQ_ERR_HW_SWITCHED_OFF          = 37,
    HQ_ERR_HW_NOT_PRESENT           = 38,
    HQ_ERR_HW_NOT_ACTIVE            = 39,


    // communication errors
    HQ_ERR_COMMS_CRC                = 41,
    HQ_ERR_COMMS_TIMEOUT            = 42,
    HQ_ERR_COMMS_HANDSHAKE          = 43,
    HQ_ERR_COMMS_KEY                = 44,
    HQ_ERR_COMMS_ID                 = 45,
    HQ_ERR_COMMS_MSG_LENGTH         = 46,
    HQ_ERR_COMMS_INVALID_ADDRESS    = 47,

    // operation errors
    HQ_ERR_BUSY                     = 51,
    HQ_ERR_OPERATION_INVALID        = 52,
    HQ_ERR_OPERATION_NOT_SUPPORTED  = 53,
    HQ_ERR_OPERATION_NOT_SUPPORTED_IN_MODE = 54,
    HQ_ERR_MODE_NOT_SUPPORTED       = 55,
    HQ_ERR_TIMEOUT                  = 56,
    HQ_ERR_VALIDATION_FAILED        = 57,


    // session related errors
    HQ_ERR_INVALID_STATE            = 61,
    HQ_ERR_ALG                      = 65,

    HQ_ERR_UNKNOWN                  = 0xEE,
    //HQ_ERR_UNKNOWN                 = 0xEEEE,  // old declaration


    // lifeqscore errors
    ERR_AGE_OUT_OF_RANGE            = HQ_ERR_DATA_OUT_OF_RANGE | (51 << 8),
    ERR_WEIGHT_OUT_OF_RANGE         = HQ_ERR_DATA_OUT_OF_RANGE | (52 << 8),
    ERR_HEIGHT_OUT_OF_RANGE         = HQ_ERR_DATA_OUT_OF_RANGE | (53 << 8),
    ERR_HR_REST_OUT_OF_RANGE        = HQ_ERR_DATA_OUT_OF_RANGE | (54 << 8),
    ERR_HRMAX_OUT_OF_RANGE          = HQ_ERR_DATA_OUT_OF_RANGE | (55 << 8),
    ERR_HRMAX_SMALLER_THAN_HRREST   = HQ_ERR_DATA_INVALID      | (56 << 8),
    ERR_VO2MAX_OUT_OF_RANGE         = HQ_ERR_DATA_OUT_OF_RANGE | (57 << 8),
    ERR_BF_OUT_OF_RANGE             = HQ_ERR_DATA_OUT_OF_RANGE | (58 << 8),
    ERR_NOT_INITIALIZED             = HQ_ERR_DATA_NOT_INITIALIZED | (59 << 8),
    ERR_GENDER_OUT_OF_RANGE         = HQ_ERR_DATA_OUT_OF_RANGE | (60 << 8),

    //algorithm input errors
    ERR_GREEN_BLANK_RF_OUT_OF_RANGE = HQ_ERR_DATA_OUT_OF_RANGE | (101 << 8),
    ERR_RED_IR_RF_OUT_OF_RANGE      = HQ_ERR_DATA_OUT_OF_RANGE | (102 << 8),
    ERR_GREEN_ISUB_OUT_OF_RANGE     = HQ_ERR_DATA_OUT_OF_RANGE | (103 << 8),
    ERR_BLANK_ISUB_OUT_OF_RANGE     = HQ_ERR_DATA_OUT_OF_RANGE | (104 << 8),
    ERR_RED_ISUB_OUT_OF_RANGE       = HQ_ERR_DATA_OUT_OF_RANGE | (105 << 8),
    ERR_IR_ISUB_OUT_OF_RANGE        = HQ_ERR_DATA_OUT_OF_RANGE | (106 << 8),
    ERR_UNKNOWN_ALG_INPUT           = HQ_ERR_DATA_OUT_OF_RANGE | (107 << 8),
    ERR_ALG_NOT_ENABLED             = HQ_ERR_DATA_OUT_OF_RANGE | (108 << 8),
    ERR_ALG_AFE_INPUT_WRONG         = RET_FAIL | (109 << 8), // Input function used in algorithm does not correspond with PP_Set_Device_AFE.


    // Application specific errors
    HQ_DSP_WARN_NOTCH_DESIGN        = HQ_ERR_ALG               | (201 << 8),

} hqret_t;


#endif /* HQERR_H_ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
