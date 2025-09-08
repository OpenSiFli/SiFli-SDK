/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#include "fmna_motion_detection_platform.h"

#include "fmna_util.h"

#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "fmna_mdp"
#include "log.h"

void fmna_motion_detection_platform_init(void)
{
    LOG_I("fmna_motion_detection_platform_init");
}

void fmna_motion_detection_platform_deinit(void)
{
    LOG_I("fmna_motion_detection_platform_deinit");
}

bool fmna_motion_detection_platform_is_motion_detected(void)
{
    LOG_I("fmna_motion_detection_platform_is_motion_detected");

    return false;
}

