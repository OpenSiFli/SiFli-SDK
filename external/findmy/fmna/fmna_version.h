/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#ifndef fmna_version_h
#define fmna_version_h

#define FW_VERSION_MAJOR_NUMBER     1
#define FW_VERSION_MINOR_NUMBER     0
#define FW_VERSION_REVISION_NUMBER  18

void fmna_version_init(void);
uint32_t fmna_version_get_fw_version(void);

#endif /* fmna_version_h */
