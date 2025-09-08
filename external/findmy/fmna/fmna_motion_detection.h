/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#ifndef fmna_motion_detection_h
#define fmna_motion_detection_h

void fmna_motion_detection_init(void);
void fmna_motion_detection_stop(void);
void fmna_motion_detection_start(void);
void fmna_motion_detection_start_active_polling(void);

#ifdef DEBUG
    void fmna_motion_detection_set_separated_ut_backoff_timeout_seconds(uint32_t separated_ut_backoff_timeout_seconds);
#endif //DEBUG

#endif /* fmna_motion_detection_h */
