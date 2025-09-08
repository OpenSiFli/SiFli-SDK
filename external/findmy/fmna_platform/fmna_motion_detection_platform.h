/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#ifndef fmna_motion_detection_platform_h
#define fmna_motion_detection_platform_h

#include "fmna_constants.h"
#include "fmna_platform_includes.h"

void fmna_motion_detection_platform_init(void);
void fmna_motion_detection_platform_deinit(void);
bool fmna_motion_detection_platform_is_motion_detected(void);

#endif /* fmna_motion_detection_platform_h */
