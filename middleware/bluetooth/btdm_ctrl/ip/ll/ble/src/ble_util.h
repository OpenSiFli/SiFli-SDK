/**
 ****************************************************************************************
 *
 * @file ble_util.h
 *
 * @brief BLE utility functions
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

#ifndef BLE_UTIL_H_
#define BLE_UTIL_H_

/**
 ****************************************************************************************
 * @defgroup BLE_UTIL BLE EM buffer management system
 * @ingroup ROOT
 * @brief BLE EM buffer management system
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer
#include "co_bt.h"           // BT standard definitions

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/**
 ******************************************************************************
 * @brief Compare 2 BLE instants (connection event counter)
 * @param[in]   instant_a   1st operand value (connection event counter)
 * @param[in]   instant_b   2nd operand value (connection event counter)
 * @return      result      True: B is greater or equal to A | False: B is smaller than A
 ******************************************************************************
 */
#define BLE_UTIL_INSTANT_PASSED(instant_a, instant_b)    ((uint16_t)((instant_b) - (instant_a)) < 32767)

/**
 ******************************************************************************
 * @brief Compute difference between two event counter
 * @param[in]   evt_cnt_a   1st operand value (connection event counter)
 * @param[in]   evt_cnt_b   2nd operand value (connection event counter)
 * @return      result    return the time difference from evt_cnt_a to evt_cnt_b
 *                           - result < 0  => evt_cnt_b is in the past
 *                           - result == 0 => evt_cnt_a is equal to evt_cnt_b
 *                           - result > 0  => evt_cnt_b is in the future
 ******************************************************************************
 */
#define BLE_UTIL_EVT_CNT_DIFF(evt_cnt_a, evt_cnt_b)    ((((uint16_t) (evt_cnt_b)) - ((uint16_t) (evt_cnt_a)) > 32768) \
                                                   ? ((int16_t)(-(((uint16_t) (evt_cnt_a)) - ((uint16_t) (evt_cnt_b))))) \
                                                   : ((int16_t)(-(((uint16_t) (evt_cnt_b)) - ((uint16_t) (evt_cnt_a))))))

/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ******************************************************************************
 * @brief Count the number of good channels in a LE map
 * @param[in]  p_ch_map  Channel map (bit fields for the 40 BT RF channels)
 * @return  Number of good channels
 ******************************************************************************
 */
uint8_t ble_util_nb_good_channels(const struct le_ch_map *p_ch_map);

/**
 *****************************************************************************************
 * @brief Get BLE packet duration in us according to PHY and packet size
 *
 * @param[in] len PDU size in octets
 * @param[in] rate PHY Rate (@see enum lld_phy)
 *
 * @return packet duration in us.
 *****************************************************************************************
 */
uint16_t ble_util_pkt_dur_in_us(uint8_t len, uint8_t rate);

#if (BLE_OBSERVER)
    /**
    * Unpack the BIGInfo ACAD data fields into a structure.
    *
    * @param[out] p_big_info BIGInfo unpacked structure
    * @param[in]  p_data     Packed Data array
    * @param[in]  data_len   Packed Data array length
    */
    uint8_t ble_util_big_info_unpack(struct big_info *p_big_info, const uint8_t *p_data, uint8_t data_len);
#endif //(BLE_OBSERVER)

/// @} BLE_UTIL

#endif // BLE_UTIL_H_
