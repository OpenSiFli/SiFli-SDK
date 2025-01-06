/**
 ****************************************************************************************
 *
 * @file ble_util_buf.h
 *
 * @brief Main API file for the BLE EM buffer management system
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef BLE_UTIL_BUF_H_
#define BLE_UTIL_BUF_H_

/**
 ****************************************************************************************
 * @defgroup BLE_UTIL_BUF BLE EM buffer management system
 * @ingroup ROOT
 * @brief BLE EM buffer management system
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer
#include "co_list.h"         // common list definition

/*
 * DEFINES
 ****************************************************************************************
 */

/// ACL data: handle and data length decoding
#define BLE_EM_ACL_DATA_LEN_LSB   (0)
#define BLE_EM_ACL_DATA_LEN_MASK  (0x03FF)
/// Packet boundary flag
#define BLE_EM_ACL_PBF_LSB        (12)
#define BLE_EM_ACL_PBF_MASK       (0x3000)
/// Broadcast flag
#define BLE_EM_ACL_BF_LSB         (14)
#define BLE_EM_ACL_BF_MASK        (0xC000)

#if (BLE_ISO_PRESENT)
    /// Descriptor or buffer index invalid
    #define BLE_UTIL_ISO_INDEX_INVALID 0xFF
#endif // (BLE_ISO_PRESENT)

/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */

/// EM LLCP buffer element
struct ble_em_llcp_buf_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM buffer pointer
    uint16_t buf_ptr;
    /// Data length (in bytes)
    uint8_t length;
};

/// EM ACL buffer element
struct ble_em_acl_buf_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM buffer pointer
    uint16_t buf_ptr;
    /// Data length (in bytes) + Data Flags (PBF + BF)
    uint16_t data_len_flags;
};

/// EM advertising data buffer element
struct ble_em_adv_buf_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM buffer pointer
    uint16_t buf_ptr;
    /// Data length (in bytes)
    uint16_t length;
};

#if (BLE_ISO_PRESENT)
/// EM ISO TX/RX Buffer
struct ble_em_iso_buf_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM buffer pointer
    uint16_t em_ptr;
    /// Buffer index
    uint8_t  buf_idx;
};
#endif // (BLE_ISO_PRESENT)


#if (BLE_CIS || BLE_BIS)
/// EM HOP Sequence
struct ble_em_hop_seq_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM pointer
    uint16_t em_ptr;
};
#endif // (BLE_CIS || BLE_BIS)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize BLE EM buffers management system
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void ble_util_buf_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Allocate a LLCP TX buffer
 *
 * @return  Pointer to LLCP TX buffer, NULL if no buffer available
 ****************************************************************************************
 */
struct ble_em_llcp_buf_elt *ble_util_buf_llcp_tx_alloc(void);

/**
 ****************************************************************************************
 * @brief Free a LLCP TX buffer
 *
 * @param[in]  buf  Pointer to LLCP TX buffer
 ****************************************************************************************
 */
void ble_util_buf_llcp_tx_free(uint16_t buf);

/**
 ****************************************************************************************
 * @brief Allocate a RX buffer
 *
 * @return  Pointer to RX buffer, NULL if no buffer available
 ****************************************************************************************
 */
uint16_t ble_util_buf_rx_alloc(void);

/**
 ****************************************************************************************
 * @brief Free a RX buffer
 *
 * @param[in]  buf  Exchange memory address of RX buffer
 ****************************************************************************************
 */
void ble_util_buf_rx_free(uint16_t buf);

#if (BLE_HOST_PRESENT)
    /**
    ****************************************************************************************
    * @brief Retrieve RX Buffer element structure to reuse it on HCI data reception
    *
    * @param[in]  buf  Exchange memory address of RX buffer
    *
    * @return RX Buffer element structure
    ****************************************************************************************
    */
    struct ble_em_acl_buf_elt *ble_util_buf_elt_rx_get(uint16_t buf);
#endif // (BLE_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Allocate a ACL TX buffer
 *
 * @return  Pointer to ACL TX buffer, NULL if no buffer available
 ****************************************************************************************
 */
uint16_t ble_util_buf_acl_tx_alloc(void);


/**
 ****************************************************************************************
 * @brief Retrieve TX buffer Element
 *
 * @return  Pointer to ACL TX buffer element
 ****************************************************************************************
 */
struct ble_em_acl_buf_elt *ble_util_buf_acl_tx_elt_get(uint16_t buf);

/**
 ****************************************************************************************
 * @brief Free a ACL TX buffer
 *
 * @param[in]  buf  Pointer to ACL TX buffer
 ****************************************************************************************
 */
void ble_util_buf_acl_tx_free(uint16_t buf);

/**
 ****************************************************************************************
 * @brief Allocate an advertising data TX buffer
 *
 * @return  Pointer to advertising data TX buffer, NULL if no buffer available
 ****************************************************************************************
 */
uint16_t ble_util_buf_adv_tx_alloc(void);

/**
 ****************************************************************************************
 * @brief Free an advertising data TX buffer
 *
 * @param[in]  buf  Pointer to advertising data TX buffer
 ****************************************************************************************
 */
void ble_util_buf_adv_tx_free(uint16_t buf);

#if (BLE_ISO_PRESENT)
    /**
    ****************************************************************************************
    * @brief Allocate an Isochronous data TX/RX buffer
    *
    * @return  Index of Isochronous data TX/RX buffer, Invalid if no buffer available
    ****************************************************************************************
    */
    uint8_t ble_util_buf_iso_alloc(void);

    /**
    ****************************************************************************************
    * @brief Retrieve ISO buffer Exchange memory pointer
    *
    * @param[in]  index  Index of Isochronous data TX/RX buffer
    *
    * @return Corresponding Exchange memory pointer.
    ****************************************************************************************
    */
    uint16_t ble_util_buf_iso_emptr_get(uint8_t index);

    /**
    ****************************************************************************************
    * @brief Free an Isochronous data TX/RX buffer
    *
    * @param[in]  index  Index of Isochronous data TX/RX buffer
    ****************************************************************************************
    */
    void ble_util_buf_iso_free(uint8_t index);

    /**
    ****************************************************************************************
    * @brief Initialize BLE ISO descriptors management system
    *
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void ble_util_isodesc_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Allocate an ISO descriptor
    *
    * @return Descriptor index allocated
    ****************************************************************************************
    */
    uint8_t ble_util_isodesc_alloc(void);

    /**
    ****************************************************************************************
    * @brief Free an ISO descriptor
    *
    * @param[in] desc_idx Index number of ISO descriptor
    ****************************************************************************************
    */
    void ble_util_isodesc_free(uint8_t desc_idx);

#endif // (BLE_ISO_PRESENT)


#if(BLE_CIS | BLE_BIS)
    /**
    ****************************************************************************************
    * @brief Allocate an Isochronous hopping sequence exchange memory field
    *
    * @return EM Pointer of the Hopping sequence
    ****************************************************************************************
    */
    uint16_t ble_util_hop_seq_alloc(void);

    /**
    ****************************************************************************************
    * @brief Free an Isochronous hopping sequence
    *
    * @param[in] em_ptr EM Pointer of the Hopping sequence
    ****************************************************************************************
    */
    void ble_util_hop_seq_free(uint16_t em_ptr);
#endif // (BLE_CIS | BLE_BIS)
/// @} BLE_UTIL_BUF

#endif // BLE_UTIL_BUF_H_
