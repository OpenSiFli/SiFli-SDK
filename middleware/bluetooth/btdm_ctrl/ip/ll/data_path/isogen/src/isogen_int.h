/**
 ****************************************************************************************
 *
 * @file isogen_int.h
 *
 * @brief Declaration of the functions used for Isochronous Payload Generator transport layer
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

#ifndef ISOGEN_INT_H_
#define ISOGEN_INT_H_

/**
 ****************************************************************************************
 * @addtogroup ISOGEN_INT
 * @ingroup ISOGEN
 * @brief Isochronous Payload Generator transport layer Internals
 *
 * This module implements the primitives used for Isochronous Payload Generator transport layer
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISOGEN)

#include <stdint.h>
#include <stdbool.h>
#include "arch.h"
#include "isogen.h"
#include "co_list.h"
#include "co_utils.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * PROTOCOL MESSAGE STRUCTURES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Isochronous Buffer management structure
struct isogen_tx_info
{
    /// SDU interval (in us)
    uint32_t sdu_interval;
    /// Reference time of next SDU (in us, based on Bluetooth timestamp)
    uint32_t next_sdu_ref_time;
    /// SDU packet counter
    uint32_t sdu_cnt;
    /// TX max SDU length
    uint16_t max_sdu;
    /// Buffer index
    uint8_t  buf_idx;
    /// Number of Buffer
    uint8_t  nb_buf;

    /// SDU buffers - nb_buffer element
    uint8_t *buf[__ARRAY_EMPTY];
};

/// Isochronous Buffer management structure
struct isogen_rx_info
{
    /// SDU packet counter
    uint32_t sdu_cnt;
    /// Expected packet length (in bytes)
    uint16_t max_sdu;
    /// Number of SDU buffers
    uint8_t  nb_buf;
    /// Buffer index
    uint8_t  buf_idx;
    /// Used to know if first valid SDU received (only used in framed mode)
    bool     first_sdu_received;

    /// SDU buffers - nb_buffer + 1 element
    uint8_t *buf[__ARRAY_EMPTY];
};

struct isogen_stat
{
    /// Statistics - Number of transmission attempts
    uint32_t nb_tx;
    /// Statistics - Number of transmission attempts that succeed
    uint32_t nb_tx_ok;
    /// Statistics - Number of Not granted packet packets
    uint32_t nb_tx_not_granted;
    /// Statistics - Number of reception attempt
    uint32_t nb_rx;
    /// Statistics - Number of reception attempts that succeed
    uint32_t nb_rx_ok;
    /// Statistics - Number of Not granted packet packets
    uint32_t nb_rx_not_granted;
    /// Statistics - Number of wrongly received packet (invalid data)
    uint32_t nb_rx_data_err;
    /// Statistics - Number of CRC Errors
    uint32_t nb_rx_crc_err;
    /// Statistics - Number of SYNC Errors
    uint32_t nb_rx_sync_err;
    /// Statistics - Number of received empty packets
    uint32_t nb_rx_empty;
    /// ISO test mode - Valid received test ISO Data SDU
    uint32_t rx_pkt_cnt;
    /// ISO test mode - SDU made up of one or more invalid or missing PDUs
    uint32_t missed_pkt_cnt;
    /// ISO test mode - SDU where the size or packet counter does not match the expected size or value
    uint32_t failed_pkt_cnt;
    /// used to know if some valid data are available
    bool     started;
    /// Active ISO Gen direction, bit field (bit position from enum iso_dp_direction)
    uint8_t  direction;
    /// ISO Tx/Rx test payload type (@see enum le_iso_test_payl_type)
    uint8_t  payload_type;
};

/// Isochronous Payload over HCI TL environment definition
struct isogen_env_tag
{
    /// Host to controller information
    struct isogen_tx_info  *p_tx[BLE_ACTIVITY_MAX];
    /// Controller to host information
    struct isogen_rx_info  *p_rx[BLE_ACTIVITY_MAX];
    /// Pointer array of channel statistics
    struct isogen_stat     *p_stat[BLE_ACTIVITY_MAX];
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// Isochronous Payload Generator environment
extern struct isogen_env_tag      isogen_env;
/// Host To Controller interface
extern const struct data_path_itf isogen_tx_itf;
/// Controller to Host interface
extern const struct data_path_itf isogen_rx_itf;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Create statistics information
 *
 * @param[in] channel       Isochronous channel handle
 * @param[in] direction     TX or RX
 ****************************************************************************************
 */
void isogen_stat_create(uint8_t channel, uint8_t direction);

/**
 ****************************************************************************************
 * @brief Cleanup statistics information
 *
 * @param[in] channel   Isochronous channel handle
 * @param[in] direction TX or RX
 ****************************************************************************************
 */
void isogen_stat_cleanup(uint8_t channel, uint8_t direction);

#endif //(BLE_ISOGEN)

/// @} ISOGEN

#endif // ISOGEN_INT_H_
