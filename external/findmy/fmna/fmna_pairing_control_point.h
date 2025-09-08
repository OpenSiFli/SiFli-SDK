/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_pairing_control_point_h
#define fmna_pairing_control_point_h

void fmna_pairing_control_point_unpair(void);
fmna_ret_code_t fmna_pairing_control_point_append_to_rx_buffer(uint8_t const *data, uint16_t length);
void fmna_pairing_control_point_handle_rx(void);

#endif /* fmna_pairing_control_point_h */
