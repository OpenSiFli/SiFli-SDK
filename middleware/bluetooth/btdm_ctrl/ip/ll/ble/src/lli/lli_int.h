/**
 ****************************************************************************************
 *
 * @file lli_int.h
 *
 * @brief Link layer ISO internal definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef LLI_INT_H_
#define LLI_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LLI Link layer ISO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_ISO_PRESENT)
#include <stdint.h>          // Standard integer
#include <stdbool.h>         // Standard boolean
#include "lli.h"             // LLI global definitions

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of instances of the LLI task
#define LLI_IDX_MAX                       (1)

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// ISO Channel type
enum lli_iso_chan_type
{
#if (BLE_CIS)
    /// Connected Isochronous Stream Channel
    LLI_ISO_CHAN_CIS = 0,
#endif // (BLE_CIS)
#if (BLE_BIS)
    /// Broadcast Isochronous Stream Channel
    LLI_ISO_CHAN_BIS,
#endif // (BLE_BIS)
#if (BLE_ISO_MODE_0)
    /// Audio Mode 0 Channel
    LLI_ISO_CHAN_AM0,
#endif // (BLE_ISO_MODE_0)
    /// Undefined channel
    LLI_ISO_CHAN_UNDEF,
};

/// ISO Group type
enum lli_iso_group_type
{
#if (BLE_CIS)
    /// Connected Isochronous Group
    LLI_ISO_GROUP_CIG = 0,
#endif // (BLE_CIS)
#if (BLE_BIS)
    /// Broadcast Isochronous Group
    LLI_ISO_GROUP_BIG,
#endif // (BLE_BIS)
    /// Undefined Group
    LLI_ISO_GROUP_UNDEF,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// ISO Group environment structure (common for BIS and CIS)
struct lli_group_env
{
    /// Group Type (BIG or CIG), (@see enum lli_iso_group_type)
    uint8_t                     type;
    /// group handle associated with streams
    uint8_t                     hdl;
};

/// Isochronous environment structure
struct lli_env_tag
{
#if (BLE_BIS || BLE_CIS)
    /// List of setup ISO groups
    struct lli_group_env      *group[BLE_ISO_GROUP_MAX];
    /// Preferred sub event margin between two sub events (IFS + margin) (in us)
    uint32_t                   sub_evt_margin;
#endif // (BLE_BIS || BLE_CIS)

    /// Framing mode (@see enum iso_frame)
    uint8_t                     framing[BLE_ACTIVITY_MAX];
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 *****************************************************************************************
 */

/// Isochronous environment
extern struct lli_env_tag lli_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (BLE_BIS || BLE_CIS)
    /**
    ****************************************************************************************
    * @brief Get environment structure for an ISO Group identified by its group handle.
    *
    * @param[in]  grp_hdl    Allocated group handle
    * @param[in]  iso_type   Isochronous Group Type (@see enum lli_iso_group_type)
    * @param[out] pp_env     Address of pointer containing address of allocated environment
    *
    * @return - CO_ERROR_NO_ERROR           if channel handle is valid and environment has been found
    *         - CO_ERROR_INVALID_HCI_PARAM  if channel handle is not valid
    *         - CO_ERROR_COMMAND_DISALLOWED if either environment has not been found or found environment
    *                                       is not environment for an ISO stream
    ****************************************************************************************
    */
    uint8_t lli_group_env_get(uint16_t grp_hdl, uint8_t iso_type, struct lli_group_env **pp_env);

    /**
    ****************************************************************************************
    * @brief Allocate new Isochronous Group environment structure and if needed check if an
    * activity is available in LLM
    *
    * @param[in]  iso_type Isochronous Group type (@see enum lli_iso_group_type)
    * @param[in]  size     Size of environment variable to allocate
    * @param[out] pp_env   Allocate environment variable
    *
    * @return - CO_ERROR_NO_ERROR                    If environment correctly allocated
    *         - CO_ERROR_CONN_REJ_LIMITED_RESOURCES  If not possible to allocate new channel memory
    *         - CO_ERROR_CON_LIMIT_EXCEED            If no activity can be assigned to new stream
    ****************************************************************************************
    */
    uint8_t lli_group_create(uint8_t iso_type, uint16_t size, struct lli_group_env **pp_env);

    /**
    ****************************************************************************************
    * @brief Free an allocated Isochronous Group environment
    *
    * @param[in] grp_hdl    Group handle
    ****************************************************************************************
    */
    void lli_group_cleanup(uint16_t grp_hdl);
#endif // (BLE_BIS || BLE_CIS)


#if (BLE_CIS)
/**
 ****************************************************************************************
 * @brief Initialization of the LLI CI module
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void lli_ci_init(bool is_reset);

/**
 ****************************************************************************************
 * Indicate an ACL link has stopped
 *
 * @param[in] link_id     Link Identifier
 * @param[in] reason Link termination reason
 ****************************************************************************************
 */
void lli_cis_link_stop_ind(uint8_t link_id, uint8_t reason);


/**
 ****************************************************************************************
 * Load specific CIS TX / RX data path
 *
 * @param[in] act_id         Activity identifier
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 * @param[in] data_path_type Type of RX/TX data path interface (@see enum iso_dp_type)
 *
 * @return status of the executed procedure (@see enum co_error)
 ****************************************************************************************
 */
uint8_t lli_cis_data_path_set(uint8_t act_id, uint8_t direction, uint8_t data_path_type);

/**
 ****************************************************************************************
 * Remove specific CIS TX / RX data path
 *
 * @param[in] act_id         Activity identifier
 * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
 *
 * @return status of the executed procedure (@see enum co_error)
 ****************************************************************************************
 */
uint8_t lli_cis_data_path_remove(uint8_t act_id, uint8_t direction);

/**
 ****************************************************************************************
 * Read CIS statistics
 *
 * @param[in]  act_id                   Activity identifier
 * @param[out] tx_unacked_packets       Pointer to number of Tx unacked packets
 * @param[out] tx_flushed_packets       Pointer to number of Tx flushed packets
 * @param[out] tx_last_subevent_packets Pointer to number of Tx last subevent packets
 * @param[out] retransmitted_packets    Pointer to number of retransmitted packets
 * @param[out] crc_error_packets        Pointer to number of CRC error packets
 * @param[out] rx_unreceived_packets    Pointer to number of Rx unreceived packets
 * @param[out] duplicate_packets        Pointer to number of duplicate packets
 *
 * @return status of the executed procedure (@see enum co_error)
 ****************************************************************************************
 */
uint8_t lli_cis_stats_get(uint8_t act_id, uint32_t *tx_unacked_packets, uint32_t *tx_flushed_packets, uint32_t *tx_last_subevent_packets,
                          uint32_t *retransmitted_packets, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets,
                          uint32_t *duplicate_packets);
#endif // (BLE_CIS)

#if (BLE_BIS)

    /**
    ****************************************************************************************
    * @brief Initialization of the LLI BI module
    *
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lli_bi_init(bool is_reset);

    /**
    ****************************************************************************************
    * Load specific BIS TX / RX data path
    *
    * @param[in] act_id         Activity identifier
    * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
    * @param[in] data_path_type Type of RX/TX data path interface (@see enum iso_dp_type)
    *
    * @return status of the executed procedure (@see enum co_error)
    ****************************************************************************************
    */
    uint8_t lli_bis_data_path_set(uint8_t act_id, uint8_t direction, uint8_t data_path_type);

    /**
    ****************************************************************************************
    * Remove specific BIS TX / RX data path
    *
    * @param[in] act_id         Activity identifier
    * @param[in] direction      Data Path direction (@see enum iso_rx_tx_select)
    *
    * @return status of the executed procedure (@see enum co_error)
    ****************************************************************************************
    */
    uint8_t lli_bis_data_path_remove(uint8_t act_id, uint8_t direction);

    /**
    ****************************************************************************************
    * Read BIS statistics
    *
    * @param[in] act_id                    Activity identifier
    * @param[out] crc_error_packets        Pointer to number of CRC error packets
    * @param[out] rx_unreceived_packets    Pointer to number of Rx unreceived packets
    *
    * @return status of the executed procedure (@see enum co_error)
    ****************************************************************************************
    */
    uint8_t lli_bis_stats_get(uint8_t act_id, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets);
#endif // (BLE_BIS)

#if (BLE_ISO_MODE_0)
    /**
    ****************************************************************************************
    * @brief Initialization of the LLI AM0 module
    *
    * @param[in] is_reset  True if reset is requested, false for an initialization
    ****************************************************************************************
    */
    void lli_am0_init(bool is_reset);

    /**
    ****************************************************************************************
    * Indicate ACL link stopped
    *
    * @param[in] link_id     Link Identifier
    * @param[in] reason      Link termination reason
    ****************************************************************************************
    */
    void lli_am0_link_stop_ind(uint8_t link_id, uint8_t reason);
#endif // (BLE_ISO_MODE_0)

#endif // (BLE_ISO_PRESENT)
/// @} LLI

#endif // LLI_INT_H_
