/**
 ****************************************************************************************
 *
 * @file uds_common.h
 *
 * @brief Header File - User Data Service Profile common types.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


#ifndef _UDS_COMMON_H_
#define _UDS_COMMON_H_

#include "rwip_config.h"

#if (BLE_UDS_SERVER)

#include "prf_types.h"
#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */
// user control point maximum size
#define UDS_USER_CTRL_PT_MAX_LEN (19)
// CCC maximum size
#define UDSS_CCC_SIZE sizeof(uint16_t)

#define UDSS_EXT_SIZE sizeof(uint16_t)

// string maximum size
#define UDSS_STRING_MAX_SIZE (255)

// date maximum size
#define UDSS_DATE_MAX_SIZE (sizeof(struct prf_date))

// uint8 maximum size
#define UDSS_UINT8_MAX_SIZE (sizeof(uint8_t))

// uint16 maximum size
#define UDSS_UINT16_MAX_SIZE (sizeof(uint16_t))

#define UDS_USER_ID_UNKNOWN_USER    (0xff)


// R/W  0x2A8A  ATT_FORMAT_utf8s    0x19    first_name      EXT -   -
// R/W  0x2A90  ATT_FORMAT_utf8s    0x19    last_name       EXT -   -
// R/W  0x2A87  ATT_FORMAT_utf8s    0x19    email_address       EXT -   -
// R/W  0x2A80  ATT_FORMAT_uin8 0x04    age year        -   -
// R/W  0x2A85          date_of_birth
// uint16   0x06    year            1582    9999
// uint8    0x04    month           0   12
// uint8    0x04    day         0   31
// R/W  0x2A8C  ATT_FORMAT_uint8    0x04    gender          -   -
// R/W  0x2A98  ATT_FORMAT_uint16   0x06    weight          -   -
// R/W  0x2A8E  ATT_FORMAT_uint16   0x06    height          -   -
// R/W  0x2A96  ATT_FORMAT_uint8    0x04    vo2_max         -   -
// R/W  0x2A8D  ATT_FORMAT_uint8    0x04    max_heart_rate          -   -
// R/W  0x2A92  ATT_FORMAT_uint8    0x04    resting_heart_rate          -   -
// R/W  0x2A91  ATT_FORMAT_uint8    0x04    maximum_recommended_heart_rate          -   -
// R/W  0x2A7F  ATT_FORMAT_uint8    0x04    aerobic_threshhold          -   -
// R/W  0x2A83  ATT_FORMAT_uint8    0x04    anaerobic_threshhold            -   -
// R/W  0x2A93  ATT_FORMAT_uint8    0x04    sport_type_for_aerobic_and_anaerobic_thresholds         -   -
// R/W  0x2A86          date_of_threshold_assessment
// uint16   0x06    year            1582    9999
// uint8    0x04    month           0   12
// uint8    0x04    day         0   31
// R/W  0x2A97  ATT_FORMAT_uint16   0x06    waist_circumference         -   -
// R/W  0x2A8F  ATT_FORMAT_uint16   0x06    hip_circumference           -   -
// R/W  0x2A88  ATT_FORMAT_uint8    0x04    fat_burn_heart_rate_lower_limit         -   -
// R/W  0x2A89  ATT_FORMAT_uint8    0x04    fat_burn_heart_rate_upper_limit         -   -
// R/W  0x2A7E  ATT_FORMAT_uint8    0x04    aerobic_heart_rate_lower_limit          -   -
// R/W  0x2A84  ATT_FORMAT_uint8    0x04    aerobic_heart_rate_upper_limit          -   -
// R/W  0x2A81  ATT_FORMAT_uint8    0x04    anaerobic_heart_rate_lower_limit            -   -
// R/W  0x2A82  ATT_FORMAT_uint8    0x04    anaerobic_heart_rate_upper_limit            -   -
// R/W  0x2A8B          five_zone_heart_rate_limits
// uint8    0x04    Very light / Light Limit            -   -
// uint8    0x04    Light / Moderate Limit          -   -
// uint8    0x04    Moderate / Hard Limit           -   -
// uint8    0x04    Hard / Maximum Limit            -   -
// R/W  0x2A94          three_zone_heart_rate_limits
// uint8    0x04    Light (Fat burn) / Moderate (Aerobic) Limit         -   -
// uint8    0x04    Moderate (Aerobic) / Hard (Anaerobic) Limit         -   -
// R/W  0x2A95  ATT_FORMAT_uint8    0x04    two_zone_heart_rate_limit           -   -
// R/W  0x2AA2  ATT_FORMAT_utf8s    0x19    language        EXT



// R/W  0x2A8A  ATT_FORMAT_utf8s    0x19    first_name      EXT -   -
// R/W  0x2A90  ATT_FORMAT_utf8s    0x19    last_name       EXT -   -
// R/W  0x2A87  ATT_FORMAT_utf8s    0x19    email_address       EXT -   -
// R/W  0x2A80  ATT_FORMAT_uin8 0x04        age                 year        -   -
// R/W  0x2A85                              date_of_birth
// R/W  0x2A8C  ATT_FORMAT_uint8    0x04    gender          -   -
// R/W  0x2A98  ATT_FORMAT_uint16   0x06    weight          -   -
// R/W  0x2A8E  ATT_FORMAT_uint16   0x06    height          -   -
// R/W  0x2A96  ATT_FORMAT_uint8    0x04    vo2_max         -   -
// R/W  0x2A8D  ATT_FORMAT_uint8    0x04    max_heart_rate          -   -
// R/W  0x2A92  ATT_FORMAT_uint8    0x04    resting_heart_rate          -   -
// R/W  0x2A91  ATT_FORMAT_uint8    0x04    maximum_recommended_heart_rate          -   -
// R/W  0x2A7F  ATT_FORMAT_uint8    0x04    aerobic_threshhold          -   -
// R/W  0x2A83  ATT_FORMAT_uint8    0x04    anaerobic_threshhold            -   -
// R/W  0x2A93  ATT_FORMAT_uint8    0x04    sport_type_for_aerobic_and_anaerobic_thresholds         -   -
// R/W  0x2A86                              date_of_threshold_assessment
// R/W  0x2A97  ATT_FORMAT_uint16   0x06    waist_circumference         -   -
// R/W  0x2A8F  ATT_FORMAT_uint16   0x06    hip_circumference           -   -
// R/W  0x2A88  ATT_FORMAT_uint8    0x04    fat_burn_heart_rate_lower_limit         -   -
// R/W  0x2A89  ATT_FORMAT_uint8    0x04    fat_burn_heart_rate_upper_limit         -   -
// R/W  0x2A7E  ATT_FORMAT_uint8    0x04    aerobic_heart_rate_lower_limit          -   -
// R/W  0x2A84  ATT_FORMAT_uint8    0x04    aerobic_heart_rate_upper_limit          -   -
// R/W  0x2A81  ATT_FORMAT_uint8    0x04    anaerobic_heart_rate_lower_limit            -   -
// R/W  0x2A82  ATT_FORMAT_uint8    0x04    anaerobic_heart_rate_upper_limit            -   -
// R/W  0x2A8B                              five_zone_heart_rate_limits
// R/W  0x2A94                              three_zone_heart_rate_limits
// R/W  0x2A95  ATT_FORMAT_uint8    0x04    two_zone_heart_rate_limit           -   -
// R/W  0x2AA2  ATT_FORMAT_utf8s    0x19    language        EXT



enum uds_char_id
{
    /// Strings: utf8s
    UDS_CHAR_ID_FIRST_NAME                                      = 0,
    UDS_CHAR_ID_LAST_NAME                                       = 1,
    UDS_CHAR_ID_EMAIL_ADDRESS                                   = 2,
    UDS_CHAR_ID_LANGUAGE                                        = 3,
    /// Date
    UDS_CHAR_ID_DATE_OF_BIRTH                                   = 4,
    UDS_CHAR_ID_DATE_OF_THRESHOLD_ASSESSMENT                    = 5,
    /// uint16
    UDS_CHAR_ID_WEIGHT                                          = 6,
    UDS_CHAR_ID_HEIGHT                                          = 7,
    UDS_CHAR_ID_WAIST_CIRCUMFERENCE                             = 8,
    UDS_CHAR_ID_HIP_CIRCUMFERENCE                               = 9,
    /// uint8
    UDS_CHAR_ID_AGE                                             = 10,
    UDS_CHAR_ID_GENDER                                          = 11,
    UDS_CHAR_ID_VO2_MAX                                         = 12,
    UDS_CHAR_ID_MAX_HEART_RATE                                  = 13,
    UDS_CHAR_ID_RESTING_HEART_RATE                              = 14,
    UDS_CHAR_ID_MAXIMUM_RECOMMENDED_HEART_RATE                  = 15,
    UDS_CHAR_ID_AEROBIC_THRESHHOLD                              = 16,
    UDS_CHAR_ID_ANAEROBIC_THRESHHOLD                            = 17,
    UDS_CHAR_ID_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS = 18,
    UDS_CHAR_ID_FAT_BURN_HEART_RATE_LOWER_LIMIT                 = 19,
    UDS_CHAR_ID_FAT_BURN_HEART_RATE_UPPER_LIMIT                 = 20,
    UDS_CHAR_ID_AEROBIC_HEART_RATE_LOWER_LIMIT                  = 21,
    UDS_CHAR_ID_AEROBIC_HEART_RATE_UPPER_LIMIT                  = 22,
    UDS_CHAR_ID_ANAEROBIC_HEART_RATE_LOWER_LIMIT                = 23,
    UDS_CHAR_ID_ANAEROBIC_HEART_RATE_UPPER_LIMIT                = 24,
    UDS_CHAR_ID_TWO_ZONE_HEART_RATE_LIMIT                       = 25,
    /// set
    UDS_CHAR_ID_FIVE_ZONE_HEART_RATE_LIMITS                     = 26,
    UDS_CHAR_ID_THREE_ZONE_HEART_RATE_LIMITS                    = 27,
//#ifdef USE_EXTRA_CHARS
    UDS_CHAR_ID_USER_DEFINED_1                                  = 28,
    UDS_CHAR_ID_USER_DEFINED_2                                  = 29,
    UDS_CHAR_ID_USER_DEFINED_3                                  = 30,
    UDS_CHAR_ID_USER_DEFINED_4                                  = 31,
//#endif //def USE_EXTRA_CHARS
    /// control
    UDS_CHAR_ID_DB_CHG_INC                                      = 32,
    UDS_CHAR_ID_USER_INDEX                                      = 33,
    UDS_CHAR_ID_USER_CTRL_PT                                    = 34,
    // UNKNOWN
    UDS_CHAR_ID_UNKNOWN                                         = 255,
};

// User Control Point Op-codes
enum uds_ctrl_pt_op_code
{
    UDS_OP_CODE_RESERVED_00             = 0x00,
    UDS_OP_CODE_REGISTER_NEW_USER       = 0x01,
    UDS_OP_CODE_CONSENT                 = 0x02,
    UDS_OP_CODE_DELETE_USER_DATA        = 0x03,
    UDS_OP_CODE_RESPONSE_CODE           = 0x20,
};

// User Control Point Response
enum uds_ctrl_pt_response
{
    UDS_OP_RESPONSE_RESERVED_00             = 0x00,
    UDS_OP_RESPONSE_SUCCESS                 = 0x01,
    UDS_OP_RESPONSE_OP_CODE_NOT_SUPPORTED   = 0x02,
    UDS_OP_RESPONSE_INVALID_PARAMETER       = 0x03,
    UDS_OP_RESPONSE_OPERATION_FAILED        = 0x04,
    UDS_OP_RESPONSE_USER_NOT_AUTHORIZED     = 0x05,
};

/*
 * STRUCTURES
 ****************************************************************************************
 */
/**
 * date structure
 * size = 4 bytes
 */
/// Date profile information
struct prf_date
{
    /// year date element
    uint16_t year;
    /// month date element
    uint8_t month;
    /// day date element
    uint8_t day;
};


/**
 * utf8_s string
 */
struct utf_8_name
{
    uint8_t length;
    uint8_t name[__ARRAY_EMPTY];
};






#endif /* (BLE_UDS_SERVER) */
#endif /* _UDS_COMMON_H_ */

