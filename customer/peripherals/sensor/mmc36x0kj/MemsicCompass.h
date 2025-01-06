/*****************************************************************************
* Copyright (C), 2018, MEMSIC Inc.
* File Name     : MemsicAlgo.h
* Author        : MEMSIC AE Algoritm Team
* Description   : This file is the head file of MemsicAlgo.lib. It provides the
*                 interface function declarations of the lib.
* History       : 1. Data           : 20180424
*                 2. Author         : Yan Guopu
*                 3. Modification   :
*****************************************************************************/
#ifndef _MEMSIC_COMPASS_H_
#define _MEMSIC_COMPASS_H_

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
    #include <utils/Log.h>  //ALOGE
#endif
#if 0
    #ifndef MEMSIC_COMPASS_DATA_TYPES
        #define MEMSIC_COMPASS_DATA_TYPES
        typedef   signed char   int8;   // signed 8-bit number    (-128 to +127)
        typedef unsigned char   uint8;  // unsigned 8-bit number  (+0 to +255)
        typedef   signed short  int16;  // signed 16-bt number    (-32,768 to +32,767)
        typedef unsigned short  uint16; // unsigned 16-bit number (+0 to +65,535)
        typedef   signed int    int32;  // signed 32-bit number   (-2,147,483,648 to +2,147,483,647)
        typedef unsigned int    uint32; // unsigned 32-bit number (+0 to +4,294,967,295)
    #endif
#endif
/*******************************************************************************************
* Function Name : InitialCompassAlgo
* Description   : Initial the filter coeficient for the compass filter.
* Input         : fc -- fliter = (1-fc)*old + fc*new; the fc smaller, the filter stronger.
* Output        : None.
* Return        : 1 --- succeed.
*                -1 --- fail.
********************************************************************************************/
DLL_API int _stdcall InitialCompassAlgo(float fc);

/*******************************************************************************************
* Function Name : GetCalOri
* Description   : Get the MEMSIC calibrated orientation vector.
* Input         : acc[0-2] --- Accelerometer data of three axis, unit is g.
*               : mag[0-2] --- Magnetic calibrated data of three axis, unit is gauss.
* Output        : co[0-2]  --- orientation data, azimuth, pitch, roll, unit is degree.
* Return        : 1 --- succeed.
*                -1 --- fail.
********************************************************************************************/
DLL_API int _stdcall GetCalOri(float *acc, float *mag, float *co);

#endif

