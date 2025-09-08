/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#include "fmna_battery_platform.h"

#include "fmna_nfc.h"

fmna_bat_state_level_t fmna_battery_platform_get_battery_level(void)
{
    //TODO: Get actual battery level.
    fmna_bat_state_level_t bat_state_level = BAT_STATE_FULL;

    // Set the NFC URL battery status key.
    fmna_nfc_set_url_key(URL_KEY_BATT_STATUS, &bat_state_level);

    return bat_state_level;
}

