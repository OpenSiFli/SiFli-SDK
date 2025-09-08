/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_paired_owner_control_point_h
#define fmna_paired_owner_control_point_h

#include "fmna_gatt.h"

/// Function for handling the different Paired Owner opcodes.
/// @param data Buffer of data of Paired Owner opcode and possible operands
void fmna_paired_owner_rx_handler(uint16_t conn_handle, uint8_t const *data, uint16_t length);

#endif /* fmna_paired_owner_control_point_h */


