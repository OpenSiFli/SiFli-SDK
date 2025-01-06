/**
 ****************************************************************************************
 *
 * @file udss.c
 *
 * @brief User Data Service Profile implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup UDSS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_UDS_SERVER)
#include "uds_common.h"

// preconfigure Extended Descriptor value at  the init stage - defined by default
#define PREINIT_EXT_DESC

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"

#include "udss.h"
#include "udss_task.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "co_utils.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
// default read perm
#define RD_P   (PERM(RD, ENABLE)        | PERM(RP, NO_AUTH))
// default write perm


#define WR_P   (PERM(WRITE_REQ, ENABLE) | PERM(WP, AUTH))
// default notify perm
#define NTF_P  (PERM(NTF, ENABLE)       | PERM(NP, AUTH))
// ind perm
#define IND_P  (PERM(IND, ENABLE)       | PERM(IP, AUTH))

#define WR_P_UA (PERM(WRITE_REQ, ENABLE) | PERM(WP, UNAUTH))

// extended Descriptor Present
#define EXT_P  (PERM(EXT,ENABLE))

// enable Read Indication (1 = Value not present in Database)
#define RI_P  (PERM(RI, ENABLE))
//#define RI_P  0

/// Full UDSS Database Description - Used to add attributes into the database
const struct attm_desc udss_att_db[] =
{
    ///                                       ATT UUID                      | Permission        | EXT PERM | MAX ATT SIZE
    // User Data Service Service Declaration
    [UDSS_IDX_SVC]                   =    {ATT_DECL_PRIMARY_SERVICE, RD_P, 0, 0},

    // DataBase Index Increment Characteristic Declaration
    [UDSS_IDX_DB_CHG_INC_CHAR]       =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // DataBase Index Increment Characteristic Value
    [UDSS_IDX_DB_CHG_INC_VAL]        =    {ATT_CHAR_DATABASE_CHANGE_INCREMENT, RD_P | WR_P | NTF_P, RI_P, sizeof(uint32_t)},
    // Client Characteristic Configuration Descriptor
    [UDSS_IDX_DB_CHG_INC_DESC_CCC]   =    {ATT_DESC_CLIENT_CHAR_CFG, RD_P | WR_P, RI_P, UDSS_CCC_SIZE},

    // User Index Characteristic Declaration
    [UDSS_IDX_USER_INDEX_CHAR]       =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Index Characteristic Value
    [UDSS_IDX_USER_INDEX_VAL]        =    {ATT_CHAR_USER_INDEX, RD_P, RI_P, sizeof(uint8_t)},

    // User Control Point Characteristic Declaration
    [UDSS_IDX_USER_CTRL_PT_CHAR]     =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Control Point Characteristic Value
    [UDSS_IDX_USER_CTRL_PT_VAL]      =    {ATT_CHAR_USER_CONTROL_POINT, WR_P | IND_P, 0, UDS_USER_CTRL_PT_MAX_LEN},
    // Client Characteristic Configuration Descriptor
    [UDSS_IDX_USER_CTRL_PT_DESC_CCC] =    {ATT_DESC_CLIENT_CHAR_CFG, RD_P | WR_P, RI_P, UDSS_CCC_SIZE},

    // Strings: utf8s
    // User Data Characteristic Declaration
    [UDS_IDX_FIRST_NAME_CHAR]        =    {ATT_DECL_CHARACTERISTIC, RD_P | EXT_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_FIRST_NAME_VAL]         =    {ATT_CHAR_FIRST_NAME, RD_P | WR_P, RI_P, UDSS_STRING_MAX_SIZE},
    // Characteristic Extended Properties Descriptor
    [UDS_IDX_FIRST_NAME_EXT]         =    {ATT_DESC_CHAR_EXT_PROPERTIES, RD_P, 0, UDSS_EXT_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_LAST_NAME_CHAR]         =    {ATT_DECL_CHARACTERISTIC, RD_P | EXT_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_LAST_NAME_VAL]          =    {ATT_CHAR_LAST_NAME, RD_P | WR_P, RI_P, UDSS_STRING_MAX_SIZE},
    // Characteristic Extended Properties Descriptor
    [UDS_IDX_LAST_NAME_EXT]          =    {ATT_DESC_CHAR_EXT_PROPERTIES, RD_P, 0, UDSS_EXT_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_EMAIL_ADDRESS_CHAR]     =    {ATT_DECL_CHARACTERISTIC, RD_P | EXT_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_EMAIL_ADDRESS_VAL]      =    {ATT_CHAR_EMAIL_ADDRESS, RD_P | WR_P, RI_P, UDSS_STRING_MAX_SIZE},
    // Characteristic Extended Properties Descriptor
    [UDS_IDX_EMAIL_ADDRESS_EXT]      =    {ATT_DESC_CHAR_EXT_PROPERTIES, RD_P, 0, UDSS_EXT_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_LANGUAGE_CHAR]          =    {ATT_DECL_CHARACTERISTIC, RD_P | EXT_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_LANGUAGE_VAL]           =    {ATT_CHAR_LANGUAGE, RD_P | WR_P, RI_P, UDSS_STRING_MAX_SIZE},
    // Characteristic Extended Properties Descriptor
    [UDS_IDX_LANGUAGE_EXT]           =    {ATT_DESC_CHAR_EXT_PROPERTIES, RD_P, 0, UDSS_EXT_SIZE},

    // Date:
    // User Data Characteristic Declaration
    [UDS_IDX_DATE_OF_BIRTH_CHAR]     =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_DATE_OF_BIRTH_VAL]      =    {ATT_CHAR_DATE_OF_BIRTH, RD_P | WR_P, RI_P, UDSS_DATE_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_DATE_OF_THRESHOLD_ASSESSMENT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_DATE_OF_THRESHOLD_ASSESSMENT_VAL] = {ATT_CHAR_DATE_OF_THRESHOLD_ASSESSMENT, RD_P | WR_P, RI_P, UDSS_DATE_MAX_SIZE},

    // uint16:
    // User Data Characteristic Declaration
    [UDS_IDX_WEIGHT_CHAR]            =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_WEIGHT_VAL]             =    {ATT_CHAR_WEIGHT, RD_P | WR_P, RI_P, UDSS_UINT16_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_HEIGHT_CHAR]            =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_HEIGHT_VAL]             =    {ATT_CHAR_HEIGHT, RD_P | WR_P, RI_P, UDSS_UINT16_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_WAIST_CIRCUMFERENCE_CHAR] =   {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_WAIST_CIRCUMFERENCE_VAL] =   {ATT_CHAR_WAIST_CIRCUMFERENCE, RD_P | WR_P, RI_P, UDSS_UINT16_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_HIP_CIRCUMFERENCE_CHAR] =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_HIP_CIRCUMFERENCE_VAL]  =    {ATT_CHAR_HIP_CIRCUMFERENCE, RD_P | WR_P, RI_P, UDSS_UINT16_MAX_SIZE},

    // uint8:
    // User Data Characteristic Declaration
    [UDS_IDX_AGE_CHAR]               =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_AGE_VAL]                =    {ATT_CHAR_AGE, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_GENDER_CHAR]            =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_GENDER_VAL]             =    {ATT_CHAR_GENDER, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_VO2_MAX_CHAR]           =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_VO2_MAX_VAL]            =    {ATT_CHAR_VO2_MAX, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_MAX_HEART_RATE_CHAR]    =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_MAX_HEART_RATE_VAL]     =    {ATT_CHAR_MAX_HEART_RATE, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_RESTING_HEART_RATE_CHAR] =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_RESTING_HEART_RATE_VAL] =    {ATT_CHAR_RESTING_HEART_RATE, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_MAXIMUM_RECOMMENDED_HEART_RATE_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_MAXIMUM_RECOMMENDED_HEART_RATE_VAL] = {ATT_CHAR_MAXIMUM_RECOMMENDED_HEART_RATE, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_AEROBIC_THRESHHOLD_CHAR] =    {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_AEROBIC_THRESHHOLD_VAL] =    {ATT_CHAR_AEROBIC_THRESHHOLD, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_ANAEROBIC_THRESHHOLD_CHAR] =  {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_ANAEROBIC_THRESHHOLD_VAL] =  {ATT_CHAR_ANAEROBIC_THRESHHOLD, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS_VAL] = {ATT_CHAR_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_FAT_BURN_HEART_RATE_LOWER_LIMIT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_FAT_BURN_HEART_RATE_LOWER_LIMIT_VAL] = {ATT_CHAR_FAT_BURN_HEART_RATE_LOWER_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_FAT_BURN_HEART_RATE_UPPER_LIMIT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_FAT_BURN_HEART_RATE_UPPER_LIMIT_VAL] = {ATT_CHAR_FAT_BURN_HEART_RATE_UPPER_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_AEROBIC_HEART_RATE_LOWER_LIMIT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_AEROBIC_HEART_RATE_LOWER_LIMIT_VAL]  = {ATT_CHAR_AEROBIC_HEART_RATE_LOWER_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_AEROBIC_HEART_RATE_UPPER_LIMIT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_AEROBIC_HEART_RATE_UPPER_LIMIT_VAL]  = {ATT_CHAR_AEROBIC_HEART_RATE_UPPER_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_ANAEROBIC_HEART_RATE_LOWER_LIMIT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_ANAEROBIC_HEART_RATE_LOWER_LIMIT_VAL] = {ATT_CHAR_ANAEROBIC_HEART_RATE_LOWER_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_ANAEROBIC_HEART_RATE_UPPER_LIMIT_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_ANAEROBIC_HEART_RATE_UPPER_LIMIT_VAL] = {ATT_CHAR_ANAEROBIC_HEART_RATE_UPPER_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_TWO_ZONE_HEART_RATE_LIMIT_CHAR] =  {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_TWO_ZONE_HEART_RATE_LIMIT_VAL] =  {ATT_CHAR_TWO_ZONE_HEART_RATE_LIMIT, RD_P | WR_P, RI_P, UDSS_UINT8_MAX_SIZE},

    // set:
    // User Data Characteristic Declaration
    [UDS_IDX_FIVE_ZONE_HEART_RATE_LIMITS_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_FIVE_ZONE_HEART_RATE_LIMITS_VAL] = {ATT_CHAR_FIVE_ZONE_HEART_RATE_LIMITS, RD_P | WR_P, RI_P, 4 * UDSS_UINT8_MAX_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_THREE_ZONE_HEART_RATE_LIMITS_CHAR] = {ATT_DECL_CHARACTERISTIC, RD_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_THREE_ZONE_HEART_RATE_LIMITS_VAL] = {ATT_CHAR_THREE_ZONE_HEART_RATE_LIMITS, RD_P | WR_P, RI_P, 2 * UDSS_UINT8_MAX_SIZE},

#ifdef USE_EXTRA_CHARS
    // User Data Characteristic Declaration
    [UDS_IDX_USER_DEFINED_1_CHAR]          =    {ATT_DECL_CHARACTERISTIC, RD_P | EXT_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_USER_DEFINED_1_VAL]           =    {ATT_CHAR_LANGUAGE, RD_P | WR_P, RI_P, UDSS_STRING_MAX_SIZE},
    // Characteristic Extended Properties Descriptor
    [UDS_IDX_USER_DEFINED_1_EXT]           =    {ATT_DESC_CHAR_EXT_PROPERTIES, RD_P, 0, UDSS_EXT_SIZE},

    // User Data Characteristic Declaration
    [UDS_IDX_USER_DEFINED_2_CHAR]          =    {ATT_DECL_CHARACTERISTIC, RD_P | EXT_P, 0, 0},
    // User Data Characteristic Value
    [UDS_IDX_USER_DEFINED_2_VAL]           =    {ATT_CHAR_LANGUAGE, RD_P | WR_P, RI_P, UDSS_STRING_MAX_SIZE},
    // Characteristic Extended Properties Descriptor
    [UDS_IDX_USER_DEFINED_2_EXT]           =    {ATT_DESC_CHAR_EXT_PROPERTIES, RD_P, 0, UDSS_EXT_SIZE},
#endif //def USE_EXTRA_CHARS
};



/**
 ****************************************************************************************
 * @brief convert Characteristic type to 'handle'
 *
 * @param[in] char_id   Characteristic type index see @enum uds_char_id.
 * @param[out] handle_raw  Handle from the GATT request.
 *
 * @return ATT_ERR_INVALID_HANDLE if this characteristic is not present.
 *
 ****************************************************************************************
 */
uint8_t udss_char_2_handle(uint16_t *handle_raw, uint8_t char_id)
{
    struct udss_env_tag *udss_env = PRF_ENV_GET(UDSS, udss);
    uint16_t handle = udss_env->shdl;
    int i;

    switch (char_id)
    {
    case UDS_CHAR_ID_DB_CHG_INC:       /// DataBase Change Increment
    {
        handle += UDSS_IDX_DB_CHG_INC_VAL;
    }
    break;
    case UDS_CHAR_ID_USER_INDEX:       /// User Index
    {
        handle += UDSS_IDX_USER_INDEX_VAL;
    }
    break;
    case UDS_CHAR_ID_USER_CTRL_PT:     /// User Control Point
    {
        handle += UDSS_IDX_USER_CTRL_PT_VAL;
    }
    break;
    default:
    {
        uint32_t mask = udss_env->char_mask; // bitfield of enabled characteristics
        //move to the first Characteristic handle
        handle += UDS_IDX_FIRST_NAME_VAL;

        if ((char_id >= UDS_CHAR_ID_DB_CHG_INC) || ((mask & ((uint32_t)1 << char_id)) == 0))
        {
            return ATT_ERR_INVALID_HANDLE;
        }

        // first 4 characteristics have 3 attributes(handles)
        for (i = 0; i < (UDS_CHAR_ID_LANGUAGE - UDS_CHAR_ID_FIRST_NAME + 1); i++)
        {
            if (char_id == 0)
            {
                break;
            }

            if (mask & 1)
            {
                handle += 3;
            }
            mask >>= 1;
            char_id --;
        }

        if (char_id)
        {
            // next 24 characteristics have 2 attributes(handles)
            for (i = 0; i < (UDS_CHAR_ID_THREE_ZONE_HEART_RATE_LIMITS - UDS_CHAR_ID_DATE_OF_BIRTH + 1); i++)
            {
                if (char_id == 0)
                {
                    break;
                }

                if (mask & 1)
                {
                    handle += 2;
                }
                mask >>= 1;
                char_id --;
            }
        }
        // still not found
        if (char_id)
        {
            return ATT_ERR_INVALID_HANDLE;
        }
    }
    break;
    }

    * handle_raw = handle;
    return ATT_ERR_NO_ERROR;
}


/**
 ****************************************************************************************
 * @brief convert 'handle' to Characteristic type
 *
 * @param[in]  handle_raw  Handle from the GATT request.
 *
 * @return Characteristic type index see @enum uds_char_id.
 ****************************************************************************************
 */

uint8_t udss_handle_2_char(uint16_t handle_raw)
{
    // result Characteristic Index
    uint8_t res_char = UDS_CHAR_ID_UNKNOWN;

    struct udss_env_tag *udss_env = PRF_ENV_GET(UDSS, udss);
    uint16_t handle = handle_raw - udss_env->shdl;
    int i;

    switch (handle)
    {
    case UDSS_IDX_DB_CHG_INC_VAL:       /// DataBase Change Increment
    {
        res_char = UDS_CHAR_ID_DB_CHG_INC;
    }
    break;
    case UDSS_IDX_USER_INDEX_VAL:       /// User Index
    {
        res_char = UDS_CHAR_ID_USER_INDEX;
    }
    break;
    case UDSS_IDX_USER_CTRL_PT_VAL:     /// User Control Point
    {
        res_char = UDS_CHAR_ID_USER_CTRL_PT;
    }
    break;
    default:
    {
        if (handle > UDSS_IDX_USER_CTRL_PT_DESC_CCC)
        {
            int8_t char_id = 0; // count for the characteristic
            uint32_t mask = udss_env->char_mask; // bitfield of enabled characteristics
            handle -= UDS_IDX_FIRST_NAME_CHAR; // extract all permanent characteristics

            // first 4 characteristics have 3 attributes(handles)
            for (i = 0; i < (UDS_CHAR_ID_LANGUAGE - UDS_CHAR_ID_FIRST_NAME + 1); i++)
            {
                if (mask & 1)
                {
                    if (handle >= 3)
                    {
                        handle -= 3;
                    }
                    else
                    {
                        res_char = char_id;
                        return res_char;
                    }
                }
                char_id ++;
                mask >>= 1;
            }

            // next 24 characteristics have 2 attributes(handles)
            for (i = 0; i < (UDS_CHAR_ID_THREE_ZONE_HEART_RATE_LIMITS - UDS_CHAR_ID_DATE_OF_BIRTH + 1); i++)
            {
                if (mask & 1)
                {
                    if (handle >= 2)
                    {
                        handle -= 2;
                    }
                    else
                    {
                        res_char = char_id;
                        return res_char;
                    }
                }
                char_id ++;
                mask >>= 1;
            }
        }
    }
    break;
    }

    return res_char;
}



/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the UDSS module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    env        Collector or Service allocated environment data.
 * @param[in|out] start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     app_task   Application task number.
 * @param[in]     sec_lvl    Security level (AUTH, EKS and MI field of @see enum attm_value_perm_mask)
 * @param[in]     param      Configuration parameters of profile collector or service (32 bits aligned)
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */

static uint8_t udss_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, struct udss_db_cfg *param)
{
    //------------------ create the attribute database for the profile -------------------
    // DB Creation Status
    uint8_t status = ATT_ERR_NO_ERROR;
    // temp index
    int i;

    // Bitfield for the ATT database
    // mask of enabled attributes per Characteristic
    uint16_t flag;
    // keeps bit position of the next bit placing
    uint16_t bit_start;
    //
    uint8_t byte_offs;
    uint8_t flag_shift;
    uint32_t bitmask;

    /// Service attribute flags
    uint8_t cfg_flag[12];
    uint32_t mask = param->char_mask; // bitfield of enabled characteristics

    memset(&cfg_flag[0], 0, sizeof(cfg_flag));
    // Mandatory Characteristics
    cfg_flag[0] = 0xff;
    cfg_flag[1] = 0x01;

    bit_start = 9;
    // Optional
    // first 4 characteristics have 3 attributes(handles)
    flag = 0x07; // 3 attributes per characteristic
    for (i = 0; i < (UDS_CHAR_ID_LANGUAGE - UDS_CHAR_ID_FIRST_NAME + 1); i++)
    {
        if (mask & 1)
        {
            //Find the byte to put the next enables attributes bitfield in 'flag'
            byte_offs = bit_start / BITS_IN_BYTE;
            // the start bit in this byte to start writing
            flag_shift = bit_start - (byte_offs * BITS_IN_BYTE);
            bitmask = flag << flag_shift; //prepare bitfield
            cfg_flag[byte_offs] |= (uint8_t)(bitmask & 0xff);
            if ((bitmask >> BITS_IN_BYTE) & 0xff)
            {
                cfg_flag[byte_offs + 1] |= (uint8_t)((bitmask >> BITS_IN_BYTE) & 0xff);
            }
        }
        mask >>= 1;
        //shift by the number of bits as the count of all attributes per characteristic
        bit_start += (UDS_IDX_LAST_NAME_CHAR - UDS_IDX_FIRST_NAME_CHAR);
    }

    // next 24 characteristics have 2 attributes(handles)
    flag = 0x03; // 2 attributes per characteristic
    for (i = 0; i < (UDS_CHAR_ID_THREE_ZONE_HEART_RATE_LIMITS - UDS_CHAR_ID_DATE_OF_BIRTH + 1); i++)
    {
        if (mask & 1)
        {
            //Find the byte to put the next enables attributes bitfield in 'flag'
            byte_offs = bit_start / BITS_IN_BYTE;
            // the start bit in this byte to start writing
            flag_shift = bit_start - (byte_offs * BITS_IN_BYTE);
            bitmask = flag << flag_shift; //prepare bitfield
            cfg_flag[byte_offs] |= (uint8_t)(bitmask & 0xff);
            if ((bitmask >> BITS_IN_BYTE) & 0xff)
            {
                cfg_flag[byte_offs + 1] |= (uint8_t)((bitmask >> BITS_IN_BYTE) & 0xff);
            }
        }
        mask >>= 1;
        //shift by the number of bits as the count of all attributes per characteristic
        bit_start += (UDS_IDX_HEIGHT_CHAR - UDS_IDX_WEIGHT_CHAR);
    }

    // Add service in the database
    status = attm_svc_create_db(start_hdl, ATT_SVC_USER_DATA, &cfg_flag[0],
                                UDSS_IDX_ATT_NB, NULL, env->task, &udss_att_db[0],
                                (sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)) | PERM(SVC_MI, ENABLE));

    // Check if an error has occured
    if (status == ATT_ERR_NO_ERROR)
    {
        // Allocate UDSS required environment variable
        struct udss_env_tag *udss_env =
            (struct udss_env_tag *) ke_malloc(sizeof(struct udss_env_tag), KE_MEM_ATT_DB);

        // Initialize UDSS environment
        env->env           = (prf_env_t *) udss_env;
        udss_env->shdl     = *start_hdl;
        udss_env->char_mask = param->char_mask;


        // Mono Instantiated APP task
        udss_env->prf_env.app_task    = app_task
                                        | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        // Multi Instantiated task
        udss_env->prf_env.prf_task    = env->task | PERM(PRF_MI, ENABLE);

        // initialize environment variable
        env->id                     = TASK_ID_UDSS;
        udss_task_init(&(env->desc));

        // If we are here, database has been fulfilled with success, go to idle state
        for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX ; idx++)
        {
            /* Put UDSS in disabled state */
            ke_state_set(KE_BUILD_ID(env->task, idx), UDSS_STATE_FREE);
        }

        {
            // fill default EXT descriptor of the characteristics
            uint8_t value[2];
            uint8_t ext_handles[] = {UDS_IDX_FIRST_NAME_EXT, UDS_IDX_LAST_NAME_EXT, UDS_IDX_EMAIL_ADDRESS_EXT, UDS_IDX_LANGUAGE_EXT};
            // prepare the value for the extended descriptor
            co_write16p(&value[0], ATT_EXT_WRITABLE_AUX);
            // for every characteristic in the list
            for (i = 0; i < sizeof(ext_handles) ; i++)
            {
                if (param->char_mask & (1 << i))
                {
                    // if Extended descriptor exist - fill it with predefined value
                    attm_att_set_value(*start_hdl + ext_handles[i], 2, 0, &value[0]);
                }
            }
        }
    }

    return (status);
}
/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
static void udss_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    /* Put UDSS in disabled state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), UDSS_STATE_FREE);

}

/**
 ****************************************************************************************
 * @brief Destruction of the UDSS module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void udss_destroy(struct prf_task_env *env)
{
    struct udss_env_tag *udss_env = (struct udss_env_tag *) env->env;

    // free profile environment variables
    env->env = NULL;
    ke_free(udss_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void udss_create(struct prf_task_env *env, uint8_t conidx)
{
    //struct udss_env_tag* udss_env = (struct udss_env_tag*) env->env;
    ke_state_set(KE_BUILD_ID(env->task, conidx), UDSS_STATE_IDLE);
}



/**
 ****************************************************************************************
 * @brief Pack Characteristic value depending on Characteristic type
 *
 * @param[in]         char_id - Characteristic Type.
 * @param[in]         *value  - Structure that keep Characteristic vales.
 * @param[out]        *data   - Output buffer for packed data.
 * @param[out]        plength - Length of the encoded data.
 *
 * @return status of the operation.
 ****************************************************************************************
 */
uint8_t udss_pack_char_value(uint8_t char_id, union char_val *value, uint8_t *data, uint16_t *plength)
{
    uint16_t length = 0;
    uint8_t  status = GAP_ERR_NO_ERROR;

    switch (char_id)
    {
    /// Date
    case UDS_CHAR_ID_DATE_OF_BIRTH:
    case UDS_CHAR_ID_DATE_OF_THRESHOLD_ASSESSMENT:
        co_write16p(&data[0], value->date.year);
        data[2] = value->date.month;
        data[3] = value->date.day;
        length = 4;
        break;
    /// uint16
    case UDS_CHAR_ID_WEIGHT:
    case UDS_CHAR_ID_HEIGHT:
    case UDS_CHAR_ID_WAIST_CIRCUMFERENCE:
    case UDS_CHAR_ID_HIP_CIRCUMFERENCE:
        co_write16p(&data[0], value->weight);
        length = 2;
        break;
    /// uint8
    case UDS_CHAR_ID_AGE:
    case UDS_CHAR_ID_GENDER:
    case UDS_CHAR_ID_VO2_MAX:
    case UDS_CHAR_ID_MAX_HEART_RATE:
    case UDS_CHAR_ID_RESTING_HEART_RATE:
    case UDS_CHAR_ID_MAXIMUM_RECOMMENDED_HEART_RATE:
    case UDS_CHAR_ID_AEROBIC_THRESHHOLD:
    case UDS_CHAR_ID_ANAEROBIC_THRESHHOLD:
    case UDS_CHAR_ID_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS:
    case UDS_CHAR_ID_FAT_BURN_HEART_RATE_LOWER_LIMIT:
    case UDS_CHAR_ID_FAT_BURN_HEART_RATE_UPPER_LIMIT:
    case UDS_CHAR_ID_AEROBIC_HEART_RATE_LOWER_LIMIT:
    case UDS_CHAR_ID_AEROBIC_HEART_RATE_UPPER_LIMIT:
    case UDS_CHAR_ID_ANAEROBIC_HEART_RATE_LOWER_LIMIT:
    case UDS_CHAR_ID_ANAEROBIC_HEART_RATE_UPPER_LIMIT:
    case UDS_CHAR_ID_TWO_ZONE_HEART_RATE_LIMIT:
        data[0] = value->age;
        length = 1;
        break;
    /// set
    case UDS_CHAR_ID_FIVE_ZONE_HEART_RATE_LIMITS:
        data[0] = value->set[0];
        data[1] = value->set[1];
        data[2] = value->set[2];
        data[3] = value->set[3];
        length = 4;
        break;
    case UDS_CHAR_ID_THREE_ZONE_HEART_RATE_LIMITS:
        data[0] = value->set[0];
        data[1] = value->set[1];
        length = 2;
        break;
    case UDS_CHAR_ID_DB_CHG_INC:
        co_write32p(&data[0], value->db_chg_inc);
        length = 4;
        break;
    case UDS_CHAR_ID_USER_INDEX:
        data[0] = value->user_idx;
        length = 1;
        break;

    // no packing for these Characteristics - done separately
    case UDS_CHAR_ID_USER_CTRL_PT:
    /// Strings: utf8s
    case UDS_CHAR_ID_FIRST_NAME:
    case UDS_CHAR_ID_LAST_NAME:
    case UDS_CHAR_ID_EMAIL_ADDRESS:
    case UDS_CHAR_ID_LANGUAGE:
    default :
        status = PRF_ERR_INVALID_PARAM;
        break;
    }

    (*plength) += length;
    return status;
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// UDSS Task interface required by profile manager
const struct prf_task_cbs udss_itf =
{
    (prf_init_fnct) udss_init,
    udss_destroy,
    udss_create,
    udss_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *udss_prf_itf_get(void)
{
    return &udss_itf;
}


#endif //(BLE_UDS_SERVER)

/// @} UDSS
