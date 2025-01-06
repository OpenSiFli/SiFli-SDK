/**
****************************************************************************************
*
* @file lc_sco.h
*
* @brief Definitions for LC SCO procedures
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef LC_SCO_H_
#define LC_SCO_H_

/**
 ****************************************************************************************
 * @defgroup LCSCO Synchronous Connection Oriented connection
 * @ingroup LC
 * @brief Responsible for handling SCO procedures.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if (MAX_NB_SYNC > 0)

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean
#include "ke_task.h"        // kernel task

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for (e)SCO parameters
struct lc_sco_air_params_tag
{
    ///Transaction ID
    uint8_t  tr_id;
    /// eSCO handle
    uint8_t  esco_hdl;
    /// eSCo LT Address
    uint8_t  esco_lt_addr;
    /// Timing control flags
    uint8_t  flags;
    /// Dsco or Desco (in slots)
    uint8_t  d_esco;
    /// Interval (in slots)
    uint8_t  t_esco;
    /// Retransmission window size (in slots)
    uint8_t  w_esco;
    /// Packet M->S type
    uint8_t  m2s_pkt_type;
    /// Packet S->M type
    uint8_t  s2m_pkt_type;
    /// Packet M->S length for eSCO link (in bytes)
    uint16_t m2s_pkt_len;
    /// Packet S->M length for eSCO link (in bytes)
    uint16_t s2m_pkt_len;
    /// Air Mode
    uint8_t  air_mode;
    /// Negotiation state
    uint8_t  nego_state;
};

/// Structure for Host synchronous parameters
struct lc_sco_host_params_tag
{
    uint32_t      tx_bw;                // Transmit Bandwidth (in B/sec)
    uint32_t      rx_bw;                // Receive Bandwidth (in B/sec)
    uint8_t       tx_cod_fmt[5];        // Transmit Coding Format
    uint8_t       rx_cod_fmt[5];        // Receive Coding Format
    uint16_t      tx_cod_fr_sz;         // Transmit Codec Frame Size (in B)
    uint16_t      rx_cod_fr_sz;         // Receive Codec Frame Size (in B)
    uint32_t      in_bw;                // Input Bandwidth (in B/sec)
    uint32_t      out_bw;               // Output Bandwidth (in B/sec)
    uint8_t       in_cod_fmt[5];        // Input Coding Format
    uint8_t       out_cod_fmt[5];       // Output Coding Format
    uint16_t      in_cod_data_sz;       // Input Coded Data Size (in bits)
    uint16_t      out_cod_data_sz;      // Output Coded Data Size (in bits)
    uint8_t       in_data_fmt;          // Input PCM Data Format
    uint8_t       out_data_fmt;         // Output PCM Data Format
    uint8_t       in_msb_pos;           // Input PCM Sample Payload MSB Position (in bits)
    uint8_t       out_msb_pos;          // Output PCM Sample Payload MSB Position (in bits)
    uint8_t       in_data_path;         // Input Data Path (VoHCI / PCM / Other ...)
    uint8_t       out_data_path;        // Output Data Path
    uint8_t       in_tr_unit_sz;        // Input Transport Unit Size (in bits)
    uint8_t       out_tr_unit_sz;       // Output Transport Unit Size (in bits)
    uint16_t      max_lat;              // Max Latency (in ms)
    uint16_t      packet_type;          // Packet Type (HV1 / HV2 / HV3 / EV3 / EV4 / EV5 / 2EV3 / 3EV3 / 2EV5 / 3EV5)
    uint8_t       retx_eff;             // Retransmission Effort (No, opt power, opt quality, don't care)
};

/// LC BT SCO DATA interface
struct lc_sco_data_tag
{
    /// length of the data
    uint8_t  length;
    uint8_t  packet_status;
    /// reserved for feature use
    uint16_t rsvd1;
    /// data pointer
    uint8_t  *data_ptr;
};
typedef struct bt_sco_callback_para
{
    uint8_t handle_type;
    union
    {
        uint32_t u32_value;
        struct lc_sco_data_tag sco_data;
    } un_para;
} bt_sco_callback_para_t;
enum sco_handle_type
{
    AUDIO_PATH_SCO_TX_DATA,
    AUDIO_PATH_SCO_RX_DATA,
    AUDIO_PATH_SCO_CMP_EVT,
    AUDIO_PATH_SCO_CHG_EVT,
};

typedef uint8_t (*bt_sco_data_handle_t)(void *p_param);
extern bt_sco_data_handle_t bt_sco_data_handle_fun;
extern void lc_save_sco_chg_para(struct hci_sync_con_chg_evt *event);
/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the LC_SCO module
 ****************************************************************************************
 */
void lc_sco_init(void);

/**
 ****************************************************************************************
 * @brief Reset LC_SCO module
 ****************************************************************************************
 */
void lc_sco_reset(void);

/**
 ****************************************************************************************
 * @brief Detach all SCO connections on a ACL link
 *
 * This function starts the disconnection of all SCO links on a ACL connection.
 *
 * When the full disconnection procedure is requested, each disconnection procedure
 * loops to this function to restart the next disconnection until no more SCO on the ACL
 * is present.
 *
 * Host is NOT notified for each SCO link disconnection.
 *
 * param[in]    pid      Process identifier (task / instance)
 * param[in]    reason   Detach reason (error code)
 *
 * return status
 ****************************************************************************************
 */
uint8_t lc_sco_detach(ke_task_id_t pid, uint8_t reason);

/**
 ****************************************************************************************
 * @brief Release all SCO connections on a ACL link
 *
 * This function releases all SCO links on a ACL connection, without notification to the
 * peer device. However, Host is notified for each SCO link disconnection.
 *
 * param[in]    pid      Process identifier (task / instance)
 * param[in]    reason   Detach reason (error code)
 ****************************************************************************************
 */
void lc_sco_release(ke_task_id_t pid, uint8_t reason);

uint8_t lc_sco_host_request(ke_task_id_t pid, bool old_style, bool host_initiated, uint16_t conhdl, struct lc_sco_host_params_tag *p_host_params);
uint8_t lc_sco_host_accept(ke_task_id_t pid, uint16_t opcode, struct lc_sco_host_params_tag *p_host_params);
void lc_sco_host_reject(ke_task_id_t pid, bool very_old_style, uint8_t status);
uint8_t lc_sco_host_request_disc(ke_task_id_t pid, uint16_t conhdl, uint8_t reason);

void lc_sco_peer_request(ke_task_id_t pid, uint8_t req_type, struct lc_sco_air_params_tag *p_peer_params);
void lc_sco_peer_accept(ke_task_id_t pid);
void lc_sco_peer_reject(ke_task_id_t pid, uint8_t reason);

void lc_sco_peer_request_disc(ke_task_id_t pid, uint8_t req_type, uint8_t scohdl, uint8_t reason);
void lc_sco_peer_accept_disc(ke_task_id_t pid);
void lc_sco_peer_reject_disc(ke_task_id_t pid, uint8_t reason);

void lc_sco_baseband_ack(ke_task_id_t pid, uint8_t lmp_opcode);
void lc_sco_timeout(ke_task_id_t pid);

/**
 ****************************************************************************************
 * @brief This function is used to update the SCO offset.
 *
 * @param[in] sync_linkid       SCO link ID
 * @param[in] sco_offset        SCO offset in slots
 *
 * @return True if SCO link present
 ****************************************************************************************
 */
void lc_sco_offset_update(uint8_t sync_linkid, uint16_t sco_offset);

#if (EAVESDROPPING_SUPPORT)
    void lc_sco_send_delayed_host_req(ke_task_id_t pid);
#endif // EAVESDROPPING_SUPPORT

#endif // (MAX_NB_SYNC > 0)

/// @} LCSCO
#endif // LC_SCO_H_
