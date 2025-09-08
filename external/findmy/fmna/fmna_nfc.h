/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#ifndef fmna_nfc_h
#define fmna_nfc_h

typedef enum
{
    URL_KEY_BATT_STATUS,
    URL_KEY_SERIAL_NUMBER_RAW,
    URL_KEY_SERIAL_NUMBER_ENCRYPTED,
    URL_KEY_OP,
    URL_KEY_BT_MAC_ADDR,
} FMNA_NFC_URL_Key_t;

/// Initialize NFC module.
///
/// @details    Load initial URL values and initialize NFC platform-dependent module.
void fmna_nfc_init(void);

/// Update NFC URL key w/ value.
///
/// @details    Set the appropriate key in the URL and reload the URL.
///             Try to reinitialize/ re-init the NFC tag after reloading the URL.
///
/// @param[in]  nfc_url_key               URL key (@ref FMNA_NFC_URL_Key_t) to set.
/// @param[in]  nfc_url_key_data    Data for URL key.
void fmna_nfc_set_url_key(FMNA_NFC_URL_Key_t nfc_url_key, void *nfc_url_key_data);

/// Load NFC URL keys for paired state, e.g. encrypted serial number, "tap" op string.
void fmna_nfc_load_paired_url(void);

/// Load NFC URL keys for unpaired state, e.g. BT MAC address, raw serial number.
void fmna_nfc_load_unpaired_url(void);

/// NFC field lost notification.
///
/// @details    Regenerate encrypted serial number for NFC URL.
///             Should be called by fmna_nfc_platform.
void fmna_nfc_field_off(void);

#endif /* fmna_nfc_h */
