/**
 ****************************************************************************************
 *
 * @file rwapp_config.h
 *
 * @brief Application configuration definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


#ifndef _RWAPP_CONFIG_H_
#define _RWAPP_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup app
 * @brief Application configuration definition
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/******************************************************************************************/
/* -------------------------   BLE APPLICATION SETTINGS      -----------------------------*/
/******************************************************************************************/

#define CFG_APP_SEC

#if defined(CFG_PRF_SIBLES)
    #define CFG_APP_SIBLE 1
#endif

/// Health Thermometer Application
#if defined(CFG_APP_HT)
    #define BLE_APP_HT           1
#else // defined(CFG_APP_HT)
    #define BLE_APP_HT           0
#endif // defined(CFG_APP_HT)

/// HID Application
#if defined(CFG_APP_HID)
    #define BLE_APP_HID          1
#endif // defined(CFG_APP_HID)

/// DIS Application
#if defined(CFG_APP_DIS)
    #define BLE_APP_DIS          1
#else // defined(CFG_APP_DIS)
    #define BLE_APP_DIS          0
#endif // defined(CFG_APP_DIS)

/// Sifli BLE Application
#if defined(CFG_APP_SIBLE)
    #define BLE_APP_SIBLE          1
#else // defined(CFG_APP_SIBLE)
    #define BLE_APP_SIBLE          0
#endif // defined(CFG_APP_SIBLE)

/// Audio Application
#if defined(CFG_APP_AM0)
    #define BLE_APP_AM0          1
#else // defined(CFG_APP_AM0)
    #define BLE_APP_AM0          0
#endif // defined(CFG_APP_AM0)

/// Battery Service Application
#if (BLE_APP_HID)
    #define BLE_APP_BATT         1
#else
    #define BLE_APP_BATT         0
#endif //(BLE_APP_HID)

/// Security Application
#if (defined(CFG_APP_SEC) || BLE_APP_HID || BLE_APP_AM0)
    #define BLE_APP_SEC          1
#else //(defined(CFG_APP_SEC) || BLE_APP_HID || BLE_APP_AM0)
    #define BLE_APP_SEC          0
#endif //(defined(CFG_APP_SEC) || BLE_APP_HID || BLE_APP_AM0)

/// Secure Connection
#if (BLE_APP_SEC)
    #define BLE_APP_SEC_CON      1
#else //(BLE_APP_AM0)
    #define BLE_APP_SEC_CON      0
#endif ////(BLE_APP_AM0)

/// Hearing Aid Service Configuration
#if (BLE_APP_AM0)
    #define AM0_APP_OPTIONAL_CHARACTERISTICS        0
    /// Default Settings for Optional Characteristics
    /// Specification Default Value
    #define AM0_APP_DEFAULT_TREBLE                  0
    #define AM0_APP_DEFAULT_BASS                    0
    /// Following have no specification Defaults - as application dependent.
    /// Please change to suit product.
    #define AM0_APP_DEFAULT_MIXED_VOL_STEP          5
    #define AM0_APP_DEFAULT_MIXED_VOL               10
    #define AM0_APP_DEFAULT_MIC_VOL_STEP            8
    #define AM0_APP_DEFAULT_STREAM_VOL_STEP         7
    #define AM0_APP_DEFAULT_MIC_SENSITIVITY         127
    #define AM0_APP_DEFAULT_SENSITIVITY_STEP        12
    /// Default is to have no active streaming program id.
    #define AM0_APP_DEFAULT_ACT_STREAM_PROG_ID      0
#else
    #define AM0_APP_OPTIONAL_CHARACTERISTICS        0
#endif //BLE_APP_AM0

/// @} rwapp_config

#endif /* _RWAPP_CONFIG_H_ */
