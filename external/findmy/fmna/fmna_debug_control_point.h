

/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#ifndef fmna_debug_control_point_h
#define fmna_debug_control_point_h

#include "fmna_gatt.h"

#ifdef DEBUG

    /// Function for handling the different Debug opcodes.
    /// @param data Buffer of data of debug opcode and possible operands
    void fmna_debug_control_point_rx_handler(uint16_t conn_handle, uint8_t const *data, uint16_t length);

#endif // DEBUG

#endif /* fmna_debug_control_point_h */


