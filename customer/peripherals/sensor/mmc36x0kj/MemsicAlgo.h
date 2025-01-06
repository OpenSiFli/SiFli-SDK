/*****************************************************************************
* Copyright (C), 2019, MEMSIC Inc.
* File Name     : MemsicAlgo.h
* Author        : MEMSIC AE Algoritm Team
* Description   : This file is the head file of MemsicAlgo.lib. It provides the
*                 interface function declarations of the lib.
* History       : 1. Data           : 20190902
*                 2. Author         : YGP
*                 3. Modification   : V3.1
*****************************************************************************/
#ifndef _MEMSICALGORITHM_H_
#define _MEMSICALGORITHM_H_

#define ANDROID_PLATFORM    1
#define EVB_PLATFORM        2
#define WINDOWS_PLATFORM    3
#define PLATFORM            EVB_PLATFORM

#if((PLATFORM == ANDROID_PLATFORM)||(PLATFORM == EVB_PLATFORM))
    #define DLL_API
    #define _stdcall
    #elif(PLATFORM == WINDOWS_PLATFORM)
    #define DLL_API extern "C" __declspec(dllexport)
#endif

#if(PLATFORM == ANDROID_PLATFORM)
    #include "MemsicConfig.h"
    #include <utils/Log.h>  //ALOGE
#endif
#if 0
    #ifndef MEMSIC_ALGORITHM_DATA_TYPES
        #define MEMSIC_ALGORITHM_DATA_TYPES
        typedef   signed char   int8;   // signed 8-bit number    (-128 to +127)
        typedef unsigned char   uint8;  // unsigned 8-bit number  (+0 to +255)
        typedef   signed short  int16;  // signed 16-bt number    (-32,768 to +32,767)
        typedef unsigned short  uint16; // unsigned 16-bit number (+0 to +65,535)
        typedef   signed int    int32;  // signed 32-bit number   (-2,147,483,648 to +2,147,483,647)
        typedef unsigned int    uint32; // unsigned 32-bit number (+0 to +4,294,967,295)
    #endif
#endif
#if(PLATFORM == ANDROID_PLATFORM)
    /*******************************************************************************************
    * Function Name : AlgoInitial
    * Description   : Initialize the algorithm.
    * Input         : None
    * Output        : None.
    * Return        : 1 --- succeed.
    *                -1 --- fail.
    ********************************************************************************************/
    int AlgoInitial(void);
    #elif((PLATFORM == EVB_PLATFORM)||(PLATFORM == WINDOWS_PLATFORM))
    /*******************************************************************************************
    * Function Name : InitialAlgorithm
    * Description   : Initial the corrected parameters for magnetic sensor.
    * Input         : sim[0-8] ---soft iron corrected parameters;
    *                 mag_para[0-2]---mag offset;
    *                 mag_para[3]  ---mag radius or magnetic field strength;
    * Output        : None.
    * Return        : 1 --- succeed.
    *                -1 --- fail.
    ********************************************************************************************/
    DLL_API int _stdcall InitialAlgorithm(float *sim, float *mag_para);
#endif
/*******************************************************************************************
* Function Name : MainAlgorithmProcess
* Description   : Pass the Acc and Mag raw data to the algorithm library.
* Input         : acc[0-2] --- Acceleration raw data of three axis, unit is g;
*                 mag[0-2] --- Magnetic raw data of three axis, unit is gauss;
*                 enable_cali --- 1 means automatic mag calibration is enabled, 0 not;
* Output        : None.
* Return        : 1 --- succeed.
*                -1 --- fail.
********************************************************************************************/
DLL_API int _stdcall MainAlgorithmProcess(float *acc, float *mag, int enable_cali);

/*******************************************************************************************
* Function Name : GetMagAccuracy
* Description   : Get the accuracy of the calibration algorithm.
* Input         : None
* Output        : None
* Return        : 0 --- Unreliable.
*               : 1 --- Low.
*               : 2 --- Medium.
*               : 3 --- High.
********************************************************************************************/
DLL_API int _stdcall GetMagAccuracy(void);

/*******************************************************************************************
* Function Name : GetMagSaturation
* Description   : Get the magnetic sensor saturation status.
* Input         : None
* Output        : None
* Return        : 0 --- Indicate that magnetic sensor works normally.
*               : 1 --- Indicate that magnetic sensor is saturated, need to do SET action.
********************************************************************************************/
DLL_API int _stdcall GetMagSaturation(void);

/*******************************************************************************************
* Function Name : GetCalPara
* Description   : Get the calibrated parameters of magnetic sensor.
* Input         : None
* Output        : cp[0-2] --- magnetic field offset, unit is gauss.
*                 cp[3] --- strength of the magnetic field, unit is gauss.
* Return        : 0 --- no need to save the calibrated magnetometer parameter.
*                 1 --- need to save the new reliable calibrated magnetometer parameter.
********************************************************************************************/
DLL_API int _stdcall GetCalPara(float *cp);

/*******************************************************************************************
* Function Name : GetCalMag
* Description   : Get the MEMSIC calibrated magnetic output.
* Input         : None
* Output        : cm[0-2]  --- calibrated magnetic sensor data, unit is gauss.
* Return        : debug use
********************************************************************************************/
DLL_API int _stdcall GetCalMag(float *cm);

#endif

