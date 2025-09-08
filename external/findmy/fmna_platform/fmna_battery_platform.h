/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_battery_platform_h
#define fmna_battery_platform_h

typedef enum
{
    BAT_STATE_FULL = 0,
    BAT_STATE_MEDIUM,
    BAT_STATE_LOW,
    BAT_STATE_CRITICALLY_LOW
} fmna_bat_state_level_t;

fmna_bat_state_level_t fmna_battery_platform_get_battery_level(void);

#endif /* fmna_battery_platform_h */
