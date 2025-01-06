/**
 ****************************************************************************************
 *
 * @file bcs_common.h
 *
 * @brief Header File - Body Composition Service common types.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


#ifndef _BCS_COMMON_H_
#define _BCS_COMMON_H_

#include "rwip_config.h"

#if 1 //(BLE_BCS_CLIENT || BLE_BCS_SERVER)

#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define BCS_MEASUREMENT_UNSUCCESSFUL (0xFFFF)

//Name: Body Composition Feature
/// Time Stamp Supported
#define BCS_TIME_STAMP_SUPPORTED (1<<0)
/// Multiple Users Supported
#define BCS_MULTIPLE_USERS_SUPPORTED (1<<1)
/// Basal Metabolism Supported
#define BCS_BASAL_METAB_SUPPORTED (1<<2)
/// Muscle percentate supported
#define BCS_MUSCLE_PERCENTAGE_SUPPORTED (1 << 3)
/// Muscle mass supported
#define BCS_MUSCLE_MASS_SUPPORTED (1<<4)
/// Fat Free mass Supported
#define BCS_FAT_FREE_MASS_SUPPORTED (1<<5)
/// Soft lean mass Supported
#define BCS_SOFT_LEAN_MASS_SUPPORTED (1 << 6)
/// Body Water Mass Supported
#define BCS_BODY_WATER_MASS_SUPPORTED (1<<7)
/// Impedance Supported
#define BCS_IMPEDANCE_SUPPORTED (1<<8)
/// Weight Supported
#define BCS_WEIGHT_SUPPORTED (1<<9)
/// Height Supported
#define BCS_HEIGHT_SUPPORTED (1<<10)

/// 0   Not Specified
#define BCS_WGHT_RESOLULION_NOT_SPECIFIED (0)
/// 1   Resolution of 0.5 kg or 1 lb
#define BCS_WGHT_RESOLULION_05kg_1lb      (1 << 11)
/// 2   Resolution of 0.2 kg or 0.5 lb
#define BCS_WGHT_RESOLULION_02kg_05lb     (2 << 11)
/// 3   Resolution of 0.1 kg or 0.2 lb
#define BCS_WGHT_RESOLULION_01kg_02lb     (3 << 11)
/// 4   Resolution of 0.05 kg or 0.1 lb
#define BCS_WGHT_RESOLULION_005kg_01lb    (4 << 11)
/// 5   Resolution of 0.02 kg or 0.05 lb
#define BCS_WGHT_RESOLULION_002kg_005lb   (5 << 11)
/// 6   Resolution of 0.01 kg or 0.02 lb
#define BCS_WGHT_RESOLULION_001kg_002lb   (6 << 11)
/// 7   Resolution of 0.005 kg or 0.01 lb
#define BCS_WGHT_RESOLULION_0005kg_001lb  (7 << 11)
/// 8 - 15  Reserved for future use
/// Height Measurement Resolution
#define BCS_HEIGHT_MEASUREMENT_RESOLULION  (7 << 15)
/// 0   Not Specified
#define BCS_HGHT_RESOLULION_NOT_SPECIFIED  (0)
/// 1   Resolution of 0.01 meter or 1 inch
#define BCS_HGHT_RESOLULION_001mtr_1inch   (1 << 15)
/// 2   Resolution of 0.005 meter or 0.5 inch
#define BCS_HGHT_RESOLULION_0005mtr_05inch (2 << 15)
/// 3   Resolution of 0.001 meter or 0.1 inch
#define BCS_HGHT_RESOLULION_0001mtr_01inch (3 << 15)
/// 4 - 7   Reserved for future use


#define CCC_IND_ENABLED (1<<1)


// Body Composition Measurement Flags define
// bit0     Measurement Units
// 0    SI (Weight and Mass in units of kilogram (kg) and Height in units of meter)     C1
// 1    Imperial (Weight and Mass in units of pound (lb) and Height in units of inch (in))  C2
#define BCS_MEAS_FLAGS_UNITS_IMPERIAL   (1<<0)
// bit1     Time stamp present
// 0    False
// 1    True    C3
#define BCS_MEAS_FLAGS_TIMESTAMP_PRESENT    (1<<1)
// bit2     User ID present
// 0    False
// 1    True    C4
#define BCS_MEAS_FLAGS_USER_ID_PRESENT  (1<<2)
// bit3     Basal Metabolism present
// 0    False
// 1    True    C5
#define BCS_MEAS_FLAGS_BASAL_METAB_PRESENT  (1<<3)
// bit4     Muscle Percentage present
// 0    False
// 1    True    C6
#define BCS_MEAS_FLAGS_MUSCLE_PERCENT_PRESENT   (1<<4)
// bit5     Muscle Mass present
// 0    False
// 1    True    C6
#define BCS_MEAS_FLAGS_MUSCLE_MASS_PRESENT  (1<<5)
// bit6     Fat Free Mass present
// 0    False
// 1    True    C7
#define BCS_MEAS_FLAGS_FAT_FREE_MASS_PRESENT    (1<<6)
// bit7     Soft Lean Mass present
// 0    False
// 1    True
#define BCS_MEAS_FLAGS_SOFT_LEAN_MASS_PRESENT   (1<<7)
// bit8    Body Water Mass present
// 0    False
// 1    True
#define BCS_MEAS_FLAGS_BODY_WATER_MASS_PRESENT  (1<<8)
// bit9     Impedance present
// 0    False
// 1    True
#define BCS_MEAS_FLAGS_IMPEDANCE_PRESENT    (1<<9)
// bit10     Weight present
// 0    False
// 1    True
#define BCS_MEAS_FLAGS_WEIGHT_PRESENT   (1<<10)
// bit11     Height present
// 0    False
// 1    True
#define BCS_MEAS_FLAGS_HEIGHT_PRESENT   (1<<11)
// bit12 Multipacket Measurement
// 0    False
// 1    True
#define BCS_MEAS_FLAGS_MULTIPACKET_MEAS (1<<12)

#define BCS_MEAS_FLAGS_MASK   ( BCS_MEAS_FLAGS_UNITS_IMPERIAL | BCS_MEAS_FLAGS_TIMESTAMP_PRESENT | BCS_MEAS_FLAGS_USER_ID_PRESENT | BCS_MEAS_FLAGS_BASAL_METAB_PRESENT \
                              | BCS_MEAS_FLAGS_MUSCLE_PERCENT_PRESENT | BCS_MEAS_FLAGS_MUSCLE_MASS_PRESENT|  BCS_MEAS_FLAGS_FAT_FREE_MASS_PRESENT | BCS_MEAS_FLAGS_SOFT_LEAN_MASS_PRESENT \
                              | BCS_MEAS_FLAGS_BODY_WATER_MASS_PRESENT | BCS_MEAS_FLAGS_IMPEDANCE_PRESENT | BCS_MEAS_FLAGS_WEIGHT_PRESENT | BCS_MEAS_FLAGS_HEIGHT_PRESENT | BCS_MEAS_FLAGS_MULTIPACKET_MEAS)

#define BCS_MEAS_USER_ID_UNKNOWN_USER   (0xff)

//
#define BCS_MEAS_FLAGS_VALID  0x8FFF

/*
* ENUMERATIONS
****************************************************************************************
*/


/*
 * STRUCTURES
 ****************************************************************************************
 */




#endif /* (BLE_BCS_CLIENT || BLE_BCS_SERVER) */

#endif /* _BCS_COMMON_H_ */
