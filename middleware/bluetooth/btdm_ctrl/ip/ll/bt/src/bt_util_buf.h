/**
 ****************************************************************************************
 *
 * @file bt_util_buf.h
 *
 * @brief Main API file for the BT EM buffer management system
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef BT_UTIL_BUF_H_
#define BT_UTIL_BUF_H_

/**
 ****************************************************************************************
 * @defgroup BT_UTIL_BUF BT EM buffer management system
 * @ingroup ROOT
 * @brief BT EM buffer management system
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
#define BT_EM_ACL_DATA_LEN_LSB   (0)
#define BT_EM_ACL_DATA_LEN_MASK  (0x03FF)
/// Packet boundary flag
#define BT_EM_ACL_PBF_LSB        (12)
#define BT_EM_ACL_PBF_MASK       (0x3000)
/// Broadcast flag
#define BT_EM_ACL_BF_LSB         (14)
#define BT_EM_ACL_BF_MASK        (0xC000)

// LMP message: length and flag decoding
/// LMP length
#define BT_RX_LMP_LEN_LSB        (0)
#define BT_RX_LMP_LEN_MASK       (0x1F)
/// Broadcast flag
#define BT_RX_LMP_BF_POS         (7)
#define BT_RX_LMP_BF_BIT         (0x80)

/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */

/// EM LMP buffer element
struct bt_em_lmp_buf_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM buffer pointer
    uint16_t buf_ptr;
    /// Data length
    uint8_t length;
};

/// EM ACL buffer element
struct bt_em_acl_buf_elt
{
    /// List header
    struct co_list_hdr hdr;

    /// L2CAP packet start timestamp (in BT half-slots)
    uint32_t l2cap_start_ts;

    /// EM buffer pointer
    uint16_t buf_ptr;
    /// Data length + Data Flags (PBF + BF)
    uint16_t data_len_flags;
};

#if VOICE_OVER_HCI
/// EM Sync buffer element
struct bt_em_sync_buf_elt
{
    /// List header
    struct co_list_hdr hdr;
    /// EM buffer pointer
    uint16_t buf_ptr;
    /// Data length
    uint8_t data_len;
    /// Packet status flags
    uint8_t packet_status_flag;
    /// Synchronous link identifier
    uint8_t sync_link_id;
};
#endif // VOICE_OVER_HCI


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize BT EM buffers management system
 *
 * @param[in]   reset    True: reset during operations | False: power-on reset
 ****************************************************************************************
 */
void bt_util_buf_init(bool reset);

/**
 ****************************************************************************************
 * @brief Allocate a LMP TX buffer
 *
 * @return  Pointer to LMP TX buffer, NULL if no buffer available
 ****************************************************************************************
 */
struct bt_em_lmp_buf_elt *bt_util_buf_lmp_tx_alloc(void);

/**
 ****************************************************************************************
 * @brief Free a LMP TX buffer
 *
 * @param[in]  buf  Pointer to LMP TX buffer
 ****************************************************************************************
 */
void bt_util_buf_lmp_tx_free(uint16_t buf);

/**
 ****************************************************************************************
 * @brief Allocate a ACL RX buffer
 *
 * @return  Pointer to ACL RX buffer, NULL if no buffer available
 ****************************************************************************************
 */
struct bt_em_acl_buf_elt *bt_util_buf_acl_rx_alloc(void);

/**
 ****************************************************************************************
 * @brief Free a ACL RX buffer
 *
 * @param[in]  buf  Pointer to ACL RX buffer
 ****************************************************************************************
 */
void bt_util_buf_acl_rx_free(uint16_t buf);

#if(HOST_PRESENT)
    /**
    ****************************************************************************************
    * @brief Retrieve RX ACL Buffer element structure to reuse it on HCI data reception
    *
    * @param[in]  buf  Exchange memory address of RX buffer
    *
    * @return RX ACL Buffer element structure
    ****************************************************************************************
    */
    struct bt_em_acl_buf_elt *bt_util_buf_elt_rx_get(uint16_t buf);
#endif //(HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief Allocate a ACL TX buffer
 *
 * @return  Pointer to ACL TX buffer, NULL if no buffer available
 ****************************************************************************************
 */
uint16_t bt_util_buf_acl_tx_alloc(void);


/**
 ****************************************************************************************
 * @brief Retrieve TX buffer Element
 *
 * @return  Pointer to ACL TX buffer element
 ****************************************************************************************
 */
struct bt_em_acl_buf_elt *bt_util_buf_acl_tx_elt_get(uint16_t buf);

/**
 ****************************************************************************************
 * @brief Free a ACL TX buffer
 *
 * @param[in]  buf  Pointer to ACL TX buffer
 ****************************************************************************************
 */
void bt_util_buf_acl_tx_free(uint16_t buf);

#if VOICE_OVER_HCI
    /**
    ****************************************************************************************
    * @brief Initialize synchronous buffer allocation system
    *
    * @param[in]  sync_id      Synchronous link identifier
    * @param[in]  tx_buf_nb    Number of synchronous TX buffers
    * @param[in]  tx_buf_size  Size of synchronous TX buffers (data payload only)
    * @param[in]  rx_buf_nb    Number of synchronous RX buffers
    * @param[in]  rx_buf_size  Size of synchronous RX buffers (including HCI header and data payload)
    *
    * @return  Pointer to Sync RX buffer, NULL if no buffer available
    ****************************************************************************************
    */
    void bt_util_buf_sync_init(uint8_t sync_id, uint8_t tx_buf_nb, uint8_t tx_buf_size, uint8_t rx_buf_nb, uint8_t rx_buf_size);

    /**
    ****************************************************************************************
    * @brief Clear synchronous buffer allocation system
    *
    * @param[in]  sync_id      Synchronous link identifier
    *
    * @return  Pointer to Sync RX buffer, NULL if no buffer available
    ****************************************************************************************
    */
    void bt_util_buf_sync_clear(uint8_t sync_id);

    /**
    ****************************************************************************************
    * @brief Allocate a Sync TX buffer
    *
    * @param[in]  sync_id  Synchronous link identifier
    * @param[in]  size     Size of the buffer
    *
    * @return  Pointer to Sync TX buffer, NULL if no buffer available
    ****************************************************************************************
    */
    uint16_t bt_util_buf_sync_tx_alloc(uint8_t sync_id, uint8_t size);


    /**
    ****************************************************************************************
    * @brief Retrieve TX buffer Element
    *
    * @param[in]  sync_id  Synchronous link identifier
    *
    * @return  Pointer to Sync TX buffer element
    ****************************************************************************************
    */
    struct bt_em_sync_buf_elt *bt_util_buf_sync_tx_elt_get(uint8_t sync_id, uint16_t buf);

    /**
    ****************************************************************************************
    * @brief Free a Sync TX buffer
    *
    * @param[in]  sync_id  Synchronous link identifier
    * @param[in]  buf      Pointer to Sync TX buffer
    ****************************************************************************************
    */
    void bt_util_buf_sync_tx_free(uint8_t sync_id, uint16_t buf);

    /**
    ****************************************************************************************
    * @brief Allocate a Sync RX buffer
    *
    * @param[in]  sync_id  Synchronous link identifier
    * @param[in]  size     Size of the buffer
    *
    * @return  Pointer to Sync RX buffer, NULL if no buffer available
    ****************************************************************************************
    */
    struct bt_em_sync_buf_elt *bt_util_buf_sync_rx_alloc(uint8_t sync_id, uint8_t size);

    /**
    ****************************************************************************************
    * @brief Free a Sync RX buffer
    *
    * @param[in]  sync_id  Synchronous link identifier
    * @param[in]  buf      Pointer to Sync RX buffer
    ****************************************************************************************
    */
    void bt_util_buf_sync_rx_free(uint8_t sync_id, uint16_t buf);
#endif // VOICE_OVER_HCI


/// @} BT_UTIL_BUF

#endif // BT_UTIL_BUF_H_
