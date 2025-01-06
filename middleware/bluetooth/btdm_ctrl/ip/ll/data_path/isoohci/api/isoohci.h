/**
 ****************************************************************************************
 *
 * @file isoohci.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over HCI Transport layer
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

#ifndef ISOOHCI_H_
#define ISOOHCI_H_

/**
 ****************************************************************************************
 * @addtogroup ISOOHCI
 * @ingroup ISO
 * @brief Isochronous Payload over HCI Transport Layer
 *
 * This module implements the primitives used for Isochronous Payload over HCI Transport layer
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISOOHCI)

    #include <stdint.h>
    #include <stdbool.h>
    #include "co_hci.h"

    #include "data_path.h"

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
    * TYPE DEFINITIONS
    ****************************************************************************************
    */


    /*
    * GLOBAL VARIABLE DECLARATIONS
    ****************************************************************************************
    */

    /*
    * FUNCTION DECLARATIONS
    ****************************************************************************************
    */

    /**
    ****************************************************************************************
    * @brief Initialize Isochronous over HCI TL module
    *
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void isoohci_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Retrieve the isochronous data path interface
    *
    * @param[in] Data path direction (@see enum iso_rx_tx_select)
    *
    * @return isochronous data path interface
    ****************************************************************************************
    */
    const struct data_path_itf *isoohci_itf_get(uint8_t  direction);

    /**
    ****************************************************************************************
    * @brief Allocate buffer for HCI ISO Data packet
    *
    * @param[in] hdr   Packet header
    *
    * @return Pointer to the buffer
    ****************************************************************************************
    */
    uint8_t *isoohci_in_buf_alloc(struct hci_iso_hdr *hdr);

    /**
    ****************************************************************************************
    * @brief Indicate an HCI ISO Data packet has been received
    *
    * @param[in] hdr   Packet header
    * @param[in] buf   Isochronous data buffer
    ****************************************************************************************
    */
    void isoohci_in_buf_received(struct hci_iso_hdr *hdr, uint8_t *buf);

    /**
    ****************************************************************************************
    * @brief Get timing information of the last transmitted SDU
    *
    * @param[in]  act_id         Activity identifier
    * @param[out] pkt_seq_nb     The packet sequence number of an SDU (Range 0x0000-0xFFFF)
    * @param[out] time_stamp     The time stamp, in microseconds, of the reference anchor point of a transmitted SDU (Range 0x00000000-0xFFFFFFFF)
    * @param[out] time_offset    The time offset, in microseconds, that is associated with a transmitted SDU (Range 0x000000-0xFFFFFF)
    *
    * @return Error status (0 if success)
    ****************************************************************************************
    */
    uint8_t isoohci_in_tx_sync_get(uint8_t act_id, uint16_t *pkt_seq_nb, uint32_t *time_stamp, uint32_t *time_offset);

    /**
    ****************************************************************************************
    * @brief Release the isochronous out buffer
    *
    * @param[in] iso_sdu_ptr   Isochronous SDU pointer
    ****************************************************************************************
    */
    void isoohci_out_buf_release(uint8_t *iso_sdu_ptr);


#endif //(BLE_ISOOHCI)

/// @} ISOOHCI

#endif // ISOOHCI_H_
