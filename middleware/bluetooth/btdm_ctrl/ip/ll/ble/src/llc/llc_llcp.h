/**
 ****************************************************************************************
 *
 * @file llc_llcp.h
 *
 * @brief Functions for control pdu transmission/reception handling
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef LLC_CNTL_H_
#define LLC_CNTL_H_

/**
 ****************************************************************************************
 * @addtogroup LLCLLCP LLCLLCP
 * @ingroup LLC
 * @brief Functions for control pdu transmission/reception handling
 *
 * This module implements the functions allowing the handling of the transmission and
 * reception of the control pdu.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "co_llcp.h"

#if (BLE_PERIPHERAL || BLE_CENTRAL)

    /*
    * MACROS
    ****************************************************************************************
    */

    /*
    * type definition
    ****************************************************************************************
    */
    /**
    ****************************************************************************************
    * @brief Callback used to inform procedure when LLCP packet transmitted has been
    *        baseband acknowledged
    *
    * @param[in] link_id        Link identifier on which the pdu will be sent.
    * @param[in] op_code        LLCP operation code
    *
    ****************************************************************************************
    */
    typedef void (*llc_llcp_tx_cfm_cb)(uint8_t link_id, uint8_t op_code);

    /**
    ****************************************************************************************
    * @brief Callback used to handle LLCP message
    *
    * @param[in] link_id        Link identifier on which the pdu will be sent.
    * @param[in] pdu            LLCP PDU information received
    * @param[in] event_cnt      Event counter value when PDU has been received
    *
    * @return status code of handler:
    *    - CO_ERROR_NO_ERROR:               Nothing more to do
    *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
    *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
    ****************************************************************************************
    */
    typedef uint8_t (*llc_llcp_pdu_handler_func_t)(uint8_t link_id, union llcp_pdu *pdu, uint16_t event_cnt);

    /*
    * FUNCTION DECLARATIONS
    ****************************************************************************************
    */


    /**
    ****************************************************************************************
    * @brief Sends the programmed LLCP PDU on the given link identifier.
    *
    * @param[in] link_id   The link identifier on which the LLCP has to be transmitted
    * @param[in] pdu       Pointer on the PDU to be sent
    * @param[in] tx_cfm_cb Pointer to the function to call when LLCP packet TX is done.
    ****************************************************************************************
    */
    void llc_llcp_send(uint8_t link_id, union llcp_pdu *pdu, llc_llcp_tx_cfm_cb tx_cfm_cb);



    /**
    ****************************************************************************************
    * @brief Link state have changed, check if some pending LLCP has to be transmitted
    *
    * @param[in] link_id   The link identifier
    ****************************************************************************************
    */
    void llc_llcp_tx_check(uint8_t link_id);


    /**
    ****************************************************************************************
    * @brief Sends the (extended) reject indication pdu.
    *
    * This function allocates an sets header and parameters of the pdu before pushing it in
    * the tx queue.
    *
    * @param[in] link_id       Link identifier on which the PDU will be sent.
    * @param[in] rej_opcode    Rejected Operation code.
    * @param[in] reason        The reason to be put in the Reject Indication PDU
    * @param[in] tx_cfm_cb     Pointer to the function to call when LLCP packet TX is done.
    ****************************************************************************************
    */
    void llc_ll_reject_ind_pdu_send(uint8_t link_id, uint8_t rej_opcode, uint8_t reason, llc_llcp_tx_cfm_cb tx_cfm_cb);

#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)



/// @} LLCLLCP

#endif // LLC_CNTL_H_
