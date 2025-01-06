/**
 ****************************************************************************************
 *
 * @file nvds.h
 *
 * @brief Non Volatile Data Storage (NVDS) driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */
#ifndef _NVDS_H_
#define _NVDS_H_

/**
 ****************************************************************************************
 * @addtogroup NVDS
 * @ingroup COMMON
 * @brief Non Volatile Data Storage (NVDS)
 *
 *   Parameters management
 *   there are two compilation options:
 *     + NVDS_8BIT_TAGLENGTH :
 *       if set, each TAG has a maximum length of 256 bytes
 *       if not set, each TAG has a maximum length of 65536 bytes
 *     + NVDS_PACKED :
 *       if not set, all the TAG header structures and TAG data contents are stored with an
 *       alignment on 32 bit boundary
 *       if set, all the TAG header structures and TAG data contents are stored
 *       consecutively without gaps (as would be a structure with pragma packed)
 *     + NVDS_READ_WRITE :
 *       if not set, only GET action on TAGs is provided.
 *       if set, PUT/DEL/LOCK actions are provided in addition of GET action.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>           // boolean definition
#include <stdint.h>            // integer definition


/*
 * DEFINES
 ****************************************************************************************
 */

/// NVDS is defined as read-write
#define NVDS_READ_WRITE          1

/// NVDS is defined as packed
#define NVDS_PACKED              1

/// NVDS has 8-bit length tags
#define NVDS_8BIT_TAGLENGTH      1

/// Type of the tag length (8 or 16 bits)
#if (NVDS_8BIT_TAGLENGTH)
    typedef uint8_t  nvds_tag_len_t;
#else
    typedef uint16_t nvds_tag_len_t;
#endif // NVDS_8BIT_TAGLENGTH


/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/// Possible Returned Status
enum NVDS_STATUS
{
    /// NVDS status OK
    NVDS_OK,
    /// generic NVDS status KO
    NVDS_FAIL,
    /// NVDS TAG unrecognized
    NVDS_TAG_NOT_DEFINED,
    /// No space for NVDS
    NVDS_NO_SPACE_AVAILABLE,
    /// Length violation
    NVDS_LENGTH_OUT_OF_RANGE,
    /// NVDS parameter locked
    NVDS_PARAM_LOCKED,
    /// NVDS corrupted
    NVDS_CORRUPT
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize NVDS.
 * @param[in]  base         NVDS base address
 * @param[in]  length       NVDS length
 * @return NVDS_OK
 ****************************************************************************************
 */
uint8_t nvds_init(uint8_t *base, uint32_t len);

/**
 ****************************************************************************************
 * @brief Look for a specific tag and return, if found and matching (in length), the
 *        DATA part of the TAG.
 *
 * If the length does not match, the TAG header structure is still filled, in order for
 * the caller to be able to check the actual length of the TAG.
 *
 * @param[in]  tag     TAG to look for whose DATA is to be retrieved
 * @param[in]  length  Expected length of the TAG
 * @param[out] buf     A pointer to the buffer allocated by the caller to be filled with
 *                     the DATA part of the TAG
 *
 * @return  NVDS_OK                  The read operation was performed
 *          NVDS_LENGTH_OUT_OF_RANGE The length passed in parameter is different than the TAG's
 ****************************************************************************************
 */
uint8_t nvds_get(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf);

#if (NVDS_READ_WRITE == 1)

    /**
    ****************************************************************************************
    * @brief Look for a specific tag and delete it (Status set to invalid)
    *
    * Implementation notes
    * 1. The write function call return status is not handled
    *
    * @param[in]  tag    TAG to mark as deleted
    *
    * @return NVDS_OK                TAG found and deleted
    *         NVDS_PARAM_LOCKED    TAG found but can not be deleted because it is locked
    *         (others)        return values from function call @ref nvds_browse_tag
    ****************************************************************************************
    */
    uint8_t nvds_del(uint8_t tag);

    /**
    ****************************************************************************************
    * @brief Look for a specific tag and lock it (Status lock bit set to LOCK).
    *
    * The write function call return status is not handled
    *
    * @param[in]  tag    TAG to mark as locked
    *
    * @return NVDS_OK    TAG found and locked
    *         (others)        return values from function call @ref nvds_browse_tag
    ****************************************************************************************
    */
    uint8_t nvds_lock(uint8_t tag);

    /**
    ****************************************************************************************
    * @brief This function adds a specific TAG to the NVDS.
    *
    * Steps:
    * 1) parse all the TAGs to:
    * 1.1) calculate the total size of all the valid TAGs
    * 1.2) erase the existing TAGs that have the same ID
    * 1.3) check if we can use the same TAG area in case of an EEPROM
    * 1.4) check that the TAG is not locked
    * 2) if we have to add the new TAG at the end fo the NVDS (cant use same area):
    * 2.1) allocate the appropriate amount of memory
    * 2.2) purge the NVDS
    * 2.3) free the memory allocated
    * 2.4) check that there is now enough room for the new TAG or return
    *      NO_SPACE_AVAILABLE
    * 3) add the new TAG
    *
    * @param[in]  tag     TAG to look for whose DATA is to be retrieved
    * @param[in]  length  Expected length of the TAG
    * @param[in]  buf     Pointer to the buffer containing the DATA part of the TAG to add to
    *                     the NVDS
    *
    * @return NVDS_OK                  New TAG correctly written to the NVDS
    *         NVDS_PARAM_LOCKED        New TAG is trying to overwrite a TAG that is locked
    *         NO_SPACE_AVAILABLE       New TAG can not fit in the available space in the NVDS
    ****************************************************************************************
    */
    uint8_t nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf);

#endif //(NVDS_READ_WRITE == 1)

/// @} NVDS

#endif // _NVDS_H_
