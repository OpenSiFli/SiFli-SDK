/**
 ****************************************************************************************
 *
 * @file isogen.h
 *
 * @brief Declaration of the functions used for Isochronous Payload Generator transport layer
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

#ifndef ISOGEN_H_
#define ISOGEN_H_

/**
 ****************************************************************************************
 * @addtogroup ISOGEN
 * @ingroup ISO
 * @brief Isochronous Payload Generator transport tayer
 *
 * This module implements the primitives used for Isochronous Payload Generator transport layer.
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
    * @brief Initialize Isochronous Payload Generator TL module
    *
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void isogen_init(bool is_reset);

    /**
    ****************************************************************************************
    * @brief Retrieve the Isochronous Payload Generator data path interface for the indicated
    * direction.
    *
    * @param[in] direction     Data path direction (@see enum iso_rx_tx_select)
    *
    * @return Required interface.
    ****************************************************************************************
    */
    const struct data_path_itf *isogen_itf_get(uint8_t direction);

    /**
    ****************************************************************************************
    * @brief Returns the Rx statistics of the channel specified
    *
    * @param[in] channel           Channel for which the RX statistics are required
    * @param[in] received_pkt_cnt  Pointer to received packet count variable
    * @param[in] missed_pkt_cnt    Pointer to missed packet count variable
    * @param[in] failed_pkt_cnt    Pointer to failed packet count variable
    *
    * @return status
    ****************************************************************************************
    */
    uint8_t isogen_get_rx_stat(uint8_t channel, uint32_t *received_pkt_cnt, uint32_t *missed_pkt_cnt, uint32_t *failed_pkt_cnt);

    /**
    ****************************************************************************************
    * @brief Set payload type for test mode
    *
    * @param[in] channel           Channel
    * @param[in] direction         Data path direction (@see enum iso_rx_tx_select)
    * @param[in] payload_type      ISO Tx/Rx test Payload Type  @see enum le_iso_test_payl_type
    *
    * @return status
    ****************************************************************************************
    */
    uint8_t isogen_payload_type_set(uint8_t channel, uint8_t direction, uint8_t payload_type);

    /**
    ****************************************************************************************
    * @brief Set initial payload counter for test mode
    *
    * @param[in] channel     Channel
    * @param[in] direction   Data direction (@see enum iso_rx_tx_select)
    * @param[in] sdu_cnt     SDU counter
    *
    * @return status
    ****************************************************************************************
    */
    uint8_t isogen_sdu_cnt_set(uint8_t channel, uint8_t direction, uint32_t sdu_cnt);

#endif //(BLE_ISOGEN)

/// @} ISOGEN

#endif // ISOGEN_H_
