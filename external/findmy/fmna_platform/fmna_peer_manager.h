/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_peer_manager_h
#define fmna_peer_manager_h

#include "fmna_constants.h"

// Check how many devices we are paired to.
uint32_t fmna_pm_peer_count(void);

/// Delete all BT pairing records.
void fmna_pm_delete_bonds(void);

/// Initializes Nordic Peer Manager module.
void peer_manager_init(void);

#endif /* fmna_peer_manager_h */
