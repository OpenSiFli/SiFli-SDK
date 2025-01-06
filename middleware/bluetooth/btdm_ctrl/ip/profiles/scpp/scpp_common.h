/**
 ****************************************************************************************
 *
 * @file scpp_common.h
 *
 * @brief Header File - Scan Parameters Profile common types.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


#ifndef _SCPP_COMMON_H_
#define _SCPP_COMMON_H_

/**
 ****************************************************************************************
 * @addtogroup SCPP Scan Parameters Profile
 * @ingroup PROFILE
 * @brief Scan Parameters Profile
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Scan Interval Window Characteristic Value Structure
struct scpp_scan_intv_wd
{
    /// Scan interval configuration
    uint16_t le_scan_intv;
    /// Scan window configuration
    uint16_t le_scan_window;
};



/// @} scpp_common

#endif /* _SCPP_COMMON_H_ */
