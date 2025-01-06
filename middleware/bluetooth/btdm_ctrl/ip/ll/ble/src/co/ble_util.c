/**
 ****************************************************************************************
 *
 * @file ble_util.c
 *
 * @brief BLE utility functions
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup BLE_UTIL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>
#include "arch.h"
#include "co_bt.h"
#include "co_utils.h"        // common utility definition
#include "ble_util.h"


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t ble_util_nb_good_channels(const struct le_ch_map *map)
{
    uint8_t nb_good_channels = 0;

    // Count number of good channels
    for (int i = (LE_CH_MAP_LEN - 1) ; i >= 0 ; i--)
    {
        uint8_t byte = map->map[i];
        nb_good_channels += NB_ONE_BITS(byte);
    }

    return nb_good_channels;
}

uint16_t ble_util_pkt_dur_in_us(uint8_t len, uint8_t rate)
{
    uint16_t pdu_len_us = 0;

    switch (rate)
    {
    case CO_RATE_1MBPS:
    {
        pdu_len_us = PDU_1MBPS_LEN_US(len);
    }
    break;
    case CO_RATE_2MBPS:
    {
        pdu_len_us = PDU_2MBPS_LEN_US(len);
    }
    break;
    case CO_RATE_125KBPS:
    {
        pdu_len_us = PDU_125KBPS_LEN_US(len);
    }
    break;
    case CO_RATE_500KBPS:
    {
        pdu_len_us = PDU_500KBPS_LEN_US(len);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, len, rate);
    }
    break;
    }

    return pdu_len_us;
}

#if (BLE_OBSERVER)
uint8_t ble_util_big_info_unpack(struct big_info *p_big_info, const uint8_t *p_data, uint8_t data_len)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t phy;
    uint32_t temp;

    // Bytes 0-3
    temp = co_read32p(&(p_data[BIG_OFFSET_POS]));
    p_big_info->big_offset       = GETF(temp, BIG_OFFSET);
    p_big_info->big_offset_unit  = GETF(temp, BIG_OFFSET_UNIT);
    p_big_info->iso_interval     = GETF(temp, BIG_ISO_INTERVAL);
    p_big_info->num_bis          = GETF(temp, BIG_NUM_BIS);

    // Byte 4
    p_big_info->bn               = GETF(p_data[BIG_BN_POS],  BIG_BN);
    p_big_info->nse              = GETF(p_data[BIG_NSE_POS], BIG_NSE);

    // Bytes 5-7
    temp = co_read24p(&(p_data[BIG_SUB_INTERVAL_POS]));
    p_big_info->sub_interval     = GETF(temp, BIG_SUB_INTERVAL);
    p_big_info->pto              = GETF(temp, BIG_PTO);

    // Bytes 8-10
    temp = co_read24p(&(p_data[BIG_BIS_SPACING_POS]));
    p_big_info->bis_spacing      = GETF(temp, BIG_BIS_SPACING);
    p_big_info->irc              = GETF(temp, BIG_IRC);

    // Byte 11
    p_big_info->max_pdu          = p_data[BIG_MAX_PDU_POS];

    // Bytes 13-16
    p_big_info->seed_access_addr = co_read32p(&(p_data[BIG_SEED_ACCESS_ADDRESS_POS]));

    // Bytes 17-20
    temp = co_read32p(&(p_data[BIG_SDU_INTERVAL_POS]));
    p_big_info->sdu_interval     = GETF(temp, BIG_SDU_INTERVAL);
    p_big_info->max_sdu          = GETF(temp, BIG_MAX_SDU);

    // Bytes 21-22
    p_big_info->base_crc_init    = co_read16p(&(p_data[BIG_BASE_CRC_INIT_POS]));

    // Bytes 23-27
    memcpy(&(p_big_info->ch_map), &(p_data[BIG_CH_MAP_LSB_POS]), LE_CH_MAP_LEN - 1);
    p_big_info->ch_map.map[LE_CH_MAP_LEN - 1] = GETF(p_data[BIG_CH_MAP_MSB_POS], BIG_CH_MAP_MSB);
    // PHY value in BIGInfo (0/1/2) is offset by 1 from typical LL PHY representation (1/2/3)
    phy = GETF(p_data[BIG_PHY_POS], BIG_PHY) + 1;
    // Check if the PHY is supported
    if ((phy <= PHY_CODED_VALUE)
#if !(BLE_PHY_2MBPS_SUPPORT)
            && (phy != PHY_2MBPS_VALUE)
#endif //(BLE_PHY_2MBPS_SUPPORT)
#if !(BLE_PHY_CODED_SUPPORT)
            && (phy != PHY_CODED_VALUE)
#endif //(BLE_PHY_CODED_SUPPORT)
       )
    {
        p_big_info->phy = co_phy_value_to_mask[phy];
    }
    else // Unsupported/RFU PHY
    {
        status = CO_ERROR_UNSUPPORTED;
    }

    // Bytes 28-32
    memcpy(p_big_info->bis_pkt_cnt, &(p_data[BIG_BIS_PLD_COUNT_LSB_POS]), BLE_PLD_CNT_SIZE - 1);
    p_big_info->bis_pkt_cnt[BLE_PLD_CNT_SIZE - 1] = GETF(p_data[BIG_BIS_PLD_COUNT_MSB_POS], BIG_BIS_PLD_COUNT_MSB);
    p_big_info->framing                           = GETF(p_data[BIG_FRAMING_POS], BIG_FRAMING);

    if (data_len == BLE_EXT_ACAD_BIG_INFO_ENC_LEN)
    {
        p_big_info->encrypted = true;
        // Bytes 33-40
        memcpy(p_big_info->giv,  &(p_data[BIG_GIV_POS]),  IV_LEN);
        // Bytes 41-56
        memcpy(p_big_info->gskd, &(p_data[BIG_GSKD_POS]), KEY_LEN);
    }
    else if (data_len == BLE_EXT_ACAD_BIG_INFO_LEN)
    {
        p_big_info->encrypted = false;
    }
    else
    {
        status = CO_ERROR_INVALID_LMP_PARAM;
    }

    return (status);
}
#endif //(BLE_OBSERVER)

/// @} BLE_UTIL
