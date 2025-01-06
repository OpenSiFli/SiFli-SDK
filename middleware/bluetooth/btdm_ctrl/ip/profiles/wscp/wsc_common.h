/**
 ****************************************************************************************
 *
 * @file wscp_common.h
 *
 * @brief Header File - Weight SCale Profile common types.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


#ifndef _WSCP_COMMON_H_
#define _WSCP_COMMON_H_

#include "rwip_config.h"

#if (BLE_WSC_CLIENT || BLE_WSC_SERVER)

    #include "prf_types.h"

    /*
    * DEFINES
    ****************************************************************************************
    */
    #define WS_MEASUREMENT_UNSUCCESSFUL (0xFFFF)

    //Name: Weight Scale Feature
    /// Time Stamp Supported
    #define WSC_TIME_STAMP_SUPPORTED (1<<0)
    /// Multiple Users Supported
    #define WSC_MULTIPLE_USERS_SUPPORTED (1<<1)
    /// BMI Supported
    #define WSC_BMI_SUPPORTED_SUPPORTED (1<<2)
    /// Weight Measurement Resolution
    #define WSC_WEIGHT_MEASUREMENT_RESOLULION (0x0f << 3)
    /// 0   Not Specified
    #define WSC_WGHT_RESOLULION_NOT_SPECIFIED (0)
    /// 1   Resolution of 0.5 kg or 1 lb
    #define WSC_WGHT_RESOLULION_05kg_1lb      (1 << 3)
    /// 2   Resolution of 0.2 kg or 0.5 lb
    #define WSC_WGHT_RESOLULION_02kg_05lb     (2 << 3)
    /// 3   Resolution of 0.1 kg or 0.2 lb
    #define WSC_WGHT_RESOLULION_01kg_02lb     (3 << 3)
    /// 4   Resolution of 0.05 kg or 0.1 lb
    #define WSC_WGHT_RESOLULION_005kg_01lb    (4 << 3)
    /// 5   Resolution of 0.02 kg or 0.05 lb
    #define WSC_WGHT_RESOLULION_002kg_005lb   (5 << 3)
    /// 6   Resolution of 0.01 kg or 0.02 lb
    #define WSC_WGHT_RESOLULION_001kg_002lb   (6 << 3)
    /// 7   Resolution of 0.005 kg or 0.01 lb
    #define WSC_WGHT_RESOLULION_0005kg_001lb  (7 << 4)
    /// 8 - 15  Reserved for future use
    /// Height Measurement Resolution
    #define WSC_HEIGHT_MEASUREMENT_RESOLULION  (7 << 7)
    /// 0   Not Specified
    #define WSC_HGHT_RESOLULION_NOT_SPECIFIED  (0)
    /// 1   Resolution of 0.01 meter or 1 inch
    #define WSC_HGHT_RESOLULION_001mtr_1inch   (1 << 7)
    /// 2   Resolution of 0.005 meter or 0.5 inch
    #define WSC_HGHT_RESOLULION_0005mtr_05inch (2 << 7)
    /// 3   Resolution of 0.001 meter or 0.1 inch
    #define WSC_HGHT_RESOLULION_0001mtr_01inch (3 << 7)
    /// 4 - 7   Reserved for future use

    #define CCC_IND_ENABLED (1<<1)


    // Weight Scale Measurement Flags define
    // bit0     Measurement Units
    // 0    SI (Weight and Mass in units of kilogram (kg) and Height in units of meter)     C1
    // 1    Imperial (Weight and Mass in units of pound (lb) and Height in units of inch (in))  C2
    #define WS_MEAS_FLAGS_UNITS_IMPERIAL    (1<<0)
    // bit1     Time stamp present
    // 0    False
    // 1    True    C3
    #define WS_MEAS_FLAGS_TIMESTAMP_PRESENT (1<<1)
    // bit2     User ID present
    // 0    False
    // 1    True    C4
    #define WS_MEAS_FLAGS_USER_ID_PRESENT   (1<<2)
    // bit3     BMI and Height present
    // 0    False
    // 1    True    C5
    #define WS_MEAS_FLAGS_BMI_PRESENT   (1<<3)


    #define WSC_FLAGS_MASK (WS_MEAS_FLAGS_UNITS_IMPERIAL | WS_MEAS_FLAGS_TIMESTAMP_PRESENT | WS_MEAS_FLAGS_USER_ID_PRESENT | WS_MEAS_FLAGS_BMI_PRESENT)

    #define WS_MEAS_USER_ID_UNKNOWN_USER    (0xff)

    /*
    * ENUMERATIONS
    ****************************************************************************************
    */


    /*
    * STRUCTURES
    ****************************************************************************************
    */




#endif /* (BLE_WSC_CLIENT || BLE_WSC_SERVER) */

#endif /* _WSCP_COMMON_H_ */
