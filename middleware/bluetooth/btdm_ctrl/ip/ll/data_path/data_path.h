/**
 ****************************************************************************************
 *
 * @file data_path.h
 *
 * @brief Main API file for the Link Layer Data path manager
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

#ifndef DATA_PATH_H_
#define DATA_PATH_H_

/**
 ****************************************************************************************
 * @defgroup DATA_PATH Link Layer ISO data path
 * @ingroup ROOT
 * @brief Link Layer ISO data path
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_ISO_PRESENT)

#include <stdbool.h>        // boolean definition
#include <stdint.h>         // integer definition


/*
 * DEFINES
 ****************************************************************************************
 */

/// Isochronous Channel data path selection
enum dp_type
{
    // -------- VENDOR SPECIFIC --------- //

    /// ISO Payload Generator
    ISO_DP_ISOGEN                   = 0xF2,
    /// Bluebud TWS
    ISO_DP_BLUEBUD_TWS              = 0xF3,
};


/// Invalid iso status field
enum sdu_status
{
    /// Correct SDU reception
    SDU_OK                 = 0x0,
    /// SDU partially received no error detected
    SDU_PARTIALLY_RECEIVED = 0x1,
    /// Some ISO packet in SDU reception has been lost
    SDU_PKT_LOST           = 0x2,
    /// A CRC error detected during reception
    SDU_CRC_ERR            = 0x3,
    /// SDU size exceed (due to a length error into an ISO packet)
    SDU_SIZE_EXCEED        = 0x4,
    /// No sync detected during reception of the packet
    SDU_SYNC_ERR           = 0x5,
    /// Wrong fragment received or unexpected LLID
    SDU_UNEXPECTED_FORMAT  = 0x6,
    /// SDU Not granted (never used for transmission or reception)
    SDU_NOT_GRANTED        = 0x7,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/**
 * Start the data path driver
 *
 * @param[in] conhdl             Connection handle of a CIS or BIS (Range 0x0000-0x0EFF)
 * @param[in] sdu_interval       SDU interval (in us)
 * @param[in] trans_latency      Transport latency (in us)
 * @param[in] max_sdu            Maximum SDU size (in bytes) (12 bits meaningful)
 *
 * @return status of the operation (@see enum co_error)
 */
typedef uint8_t (*data_path_start_cb)(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);

/**
 * Stop the data path driver
 *
 * @param[in] conhdl  Connection handle of a CIS or BIS (Range 0x0000-0x0EFF)
 * @param[in] reason  Reason for stopping data path
 */
typedef void (*data_path_stop_cb)(uint16_t conhdl, uint8_t reason);

/**
 ****************************************************************************************
 * Get SDU buffer for transmission or reception on an Isochronous link
 *
 * @param[in]     conhdl       Connection handle of a CIS or BIS (Range 0x0000-0x0EFF)
 * @param[in|out] p_ref_time   ISOAL provides the reference anchor point for the new SDU, data path returns
 *                             the SDU reference time (only for input direction) (in us, based on Bluetooth timestamp)
 * @param[out]    p_sdu_len    Size of SDU (in bytes) (input: size of the data to transmit, output: size of the buffer)
 *
 * @return Pointer of the SDU buffer
 ****************************************************************************************
 */
typedef uint8_t *(*data_path_sdu_get_cb)(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len);

/**
 * Indicate SDU transmission or reception is completed
 *
 * @param[in] conhdl   Connection handle of a CIS or BIS (Range 0x0000-0x0EFF)
 * @param[in] sdu_len  Size of SDU received (only for output direction)
 * @param[in] ref_time SDU synchronization reference (only for output direction) (in us, based on Bluetooth timestamp)
 * @param[in] p_buf    Pointer of the SDU buffer
 * @param[in] status   Reception or transmission buffer status (@see enum sdu_status)
 */
typedef void (*data_path_sdu_done_cb)(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status);

/**
 ****************************************************************************************
 * Indicate the local time of the synchronization signal raised previously by the HW
 *
 * Note: only one data path can use the local synchronization mechanism.
 *
 * @param[in]     t_gpio      Timestamp of the GPIO signal previously raised by the HW (in us, based on Bluetooth timestamp)
 ****************************************************************************************
 */
typedef void (*data_path_local_sync_cb)(uint32_t t_gpio);

/**
 ****************************************************************************************
 * Indicate the local time of an ISO event, when receiving a packet from the CIS-master or BIS-transmitter
 *
 * Note: only for CIS-slave or BIS-receiver
 *
 * @param[in]     evt_cnt           CIG/BIG event counter
 * @param[in]     ref_anchor_pt     Timestamp of the CIG/BIG reference anchor point (in us, based on Bluetooth timestamp)
 ****************************************************************************************
 */
typedef void (*data_path_peer_sync_cb)(uint16_t evt_cnt, uint32_t ref_anchor_pt);


/// Data path driver interface container
struct data_path_itf
{
    /// Start the data path driver
    data_path_start_cb cb_start;

    /// Stop the data path driver
    data_path_stop_cb cb_stop;

    /// Get SDU buffer for transmission or reception on an Isochronous link
    data_path_sdu_get_cb cb_sdu_get;

    /// Indicate SDU transmission or reception is completed
    data_path_sdu_done_cb cb_sdu_done;

    /// Indicate the local time of the synchronization signal raised previously by the HW
    data_path_local_sync_cb cb_local_sync;

    /// Indicate the local time of an ISO event, when receiving a packet from the CIS-master or BIS-transmitter
    data_path_peer_sync_cb cb_peer_sync;
};


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */



/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BLE Data Path driver
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void data_path_init(bool is_reset);


/**
 ****************************************************************************************
 * @brief Configure the specified VS datapath
 *
 * @param[in]  type    Type of data path interface (@see enum iso_dp_type)
 * @param[in]  len     Length of the vendor-specific configuration data
 * @param[in]  cfg     Vendor-specific configuration data for the data path being configured
 *
 * @return status 0 for success, or reason code otherwise
 ****************************************************************************************
 */
uint8_t data_path_config(uint8_t type, uint8_t len, const uint8_t *cfg);

/**
 ****************************************************************************************
 * @brief Retrieve the data path interface according to the direction
 *
 * @param[in]  type      Type of data path interface (@see enum iso_dp_type)
 * @param[in]  direction Data Path direction (@see enum iso_rx_tx_select)
 *
 * @return Pointer to the interface of the data path driver, NULL if no driver found
 ****************************************************************************************
 */
//const struct data_path_itf *data_path_itf_get(uint8_t type, uint8_t direction);
typedef const struct data_path_itf *(*data_path_itf_get_handler)(uint8_t type, uint8_t direction);
extern data_path_itf_get_handler _data_path_itf_get_handler;
#define data_path_itf_get(type, direction)  _data_path_itf_get_handler(type, direction)

/**
 ****************************************************************************************
 * Get if current data path is disabled or not.
 *
 * @param[in] dp         Data path
 *
 * @return True if data path is disabled, False else.
 ****************************************************************************************
 */
bool data_path_is_disabled(const struct data_path_itf *dp);

#endif // (BLE_ISO_PRESENT)
/// @} DATA_PATH

#endif // DATA_PATH_H_
