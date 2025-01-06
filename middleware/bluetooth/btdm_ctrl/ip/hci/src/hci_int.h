/**
 ****************************************************************************************
 *
 * @file hci.h
 *
 * @brief This file contains definitions related to the HCI module.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef HCI_INT_H_
#define HCI_INT_H_

/**
 ****************************************************************************************
 * @addtogroup HCI Host Controller Interface
 *@{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#if (HCI_PRESENT)

#include <stddef.h>          // standard definition
#include <stdint.h>          // standard integer
#include "co_bt.h"           // BT standard definitions
#if(HOST_PRESENT)
    #include "co_djob.h"         // Defer handling of HCI Message
#endif //(HOST_PRESENT)

#include "hci.h"
#include "ke_msg.h"          // Kernel message definitions


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Possible destination field values within lower layers
enum HCI_MSG_DEST_LL
{
    MNG,
    CTRL,
    BLE_MNG,
    BLE_CTRL,
    BT_MNG,
    BT_CTRL_CONHDL,
    BT_CTRL_BD_ADDR,
    BT_BCST,
    DBG,
    BLE_ISO,
    COMMON,
#if (EAVESDROPPING_SUPPORT)
    ED,
#endif // EAVESDROPPING_SUPPORT
    LL_UNDEF,
};

#if (HCI_TL_SUPPORT)
/// Status returned by generic packer-unpacker
enum HCI_PACK_STATUS
{
    HCI_PACK_OK,
    HCI_PACK_IN_BUF_OVFLW,
    HCI_PACK_OUT_BUF_OVFLW,
    HCI_PACK_WRONG_FORMAT,
    HCI_PACK_ERROR,
};
#endif //(HCI_TL_SUPPORT)

#if  (BT_EMB_PRESENT)
/// Status of BT ACL connection at HCI level
enum HCI_BT_ACL_CON_STATUS
{
    /// ACL link not active
    HCI_BT_ACL_STATUS_NOT_ACTIVE,
    /// ACL link ID associated with BD address
    HCI_BT_ACL_STATUS_BD_ADDR,
    /// ACL link ID associated with BD address + connection handle
    HCI_BT_ACL_STATUS_BD_ADDR_CONHDL,
};
#endif //(BT_EMB_PRESENT)


/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */
/// HCI command descriptor structure
typedef struct hci_cmd_desc hci_cmd_desc_t;
/// HCI event descriptor structure
typedef struct hci_evt_desc hci_evt_desc_t;


#if (BT_EMB_PRESENT)
/// HCI Environment context structure
typedef struct hci_bt_acl_con
{
    /**
     * BT ACL connection status
     * - 0x00: Not active
     * - 0x01: link ID associated with BD address
     * - 0x02: link ID associated with BD address + connection handle
     */
    uint8_t state;

    /// BD address associated with link ID
    struct bd_addr bd_addr;
} hci_bt_acl_con_t;

/// Condition based on class of device
struct classofdevcondition
{
    /// Class of device
    struct devclass classofdev;
    /// Device mask
    struct devclass class_mask;
};

/// Condition based on device class
union cond
{
    /// Device class
    struct classofdevcondition device_class;
    /// BD address
    struct bd_addr bdaddr;
};

/// Event Filter Record
typedef struct hci_evt_filter
{
    bool        in_use;
    uint8_t     type;
    uint8_t     condition;
    uint8_t     auto_accept;
    union cond  param;
} hci_evt_filter_t;
#endif //(BT_EMB_PRESENT)

/// HCI Environment context structure
struct hci_env_tag
{
    /// Event mask
    struct evt_mask evt_msk;

    /// Event mask page 2
    struct evt_mask evt_msk_page_2;

#if (BLE_EMB_PRESENT)
    /// LE Event mask
    struct evt_mask le_evt_msk;
#endif // (BLE_EMB_PRESENT)
#if (BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)
    /// Link association table for BLE link-oriented messages routing
    bool ble_con_state[BLE_ACTIVITY_MAX];
#endif //(BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)

#if (BT_EMB_PRESENT)
    /// Link association table for BT link-oriented messages routing
    hci_bt_acl_con_t bt_acl_con_tab[MAX_NB_ACTIVE_ACL];

    /// Event filters
    hci_evt_filter_t evt_filter[HCI_FILTER_NB];

    /**
     * Current auto-accept command opcode, used to filter the associated CS event
     * Note: assume that there would be 1 auto accept command at the same time
     * Note: could be HCI_Accept_Con, HCI_Accept_sync_Con
     */
    uint16_t auto_accept_opcode;

#if (MAX_NB_SYNC > 0)
    /**
     * Voice settings used when SCO connection is auto-accepted
     */
    uint16_t voice_settings;
#endif //(MAX_NB_SYNC > 0)

    /**
     * Auto-reject flag, used to filter the complete event when a request has been auto-rejected
     */
    bool auto_reject;

#endif //(BT_EMB_PRESENT)

    /// Command descriptor
    const hci_cmd_desc_t *p_cmd_desc;
    /// Command descriptor for command complete or command status event
    const hci_cmd_desc_t *p_cmd_evt_desc;
    /// Event descriptor pointer
    const hci_evt_desc_t *p_evt_desc;

    /**
     * Connection Accept Timeout
     *
     * Size: 2 Octets
     * Range: 0x0001 to 0xB540
     * Default: 0x1F40
     * Mandatory Range: 0x00A0 to 0xB540
     * Time = N * 0.625 msec
     * Time Range: 0.625 msec to 29 sec
     * Time Default: BR/EDR 5 sec
     */
    uint16_t con_accept_to;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

#if BLE_HOST_PRESENT
    extern const uint8_t hl_task_type[];
#endif //BLE_HOST_PRESENT

#if (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)
    /**
    * Indicate if HCI is used by external Host, or internal Host.
    * Used in Full mode only. By default HCI is used by internal Host.
    * HCI switches to external host as soon as an HCI command is received.
    * This variable should not be reset.
    */
    extern bool hci_ext_host;
#endif // (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)

///HCI environment context
extern struct hci_env_tag hci_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (RW_DEBUG)
    /**
    ****************************************************************************************
    * @brief Check all command/event descriptor tables to ensure that they are sorted in opcode
    * ascending order. An ASSERT_ERR will be triggered if not.
    *****************************************************************************************
    */
    void hci_check_desc_tabs_order();
#endif // (RW_DEBUG)

/**
****************************************************************************************
* @brief Look for a command descriptor that could match with the specified opcode
*
* @param[in]  opcode   Command opcode
*
* @return     Pointer the command descriptor (NULL if not found)
*****************************************************************************************
*/
const hci_cmd_desc_t *hci_msg_cmd_desc_get(uint16_t opcode);

#if (BT_HOST_OTHER_STACK)
    const hci_cmd_desc_t *hci_msg_cmd_desc_get_for_dual_stack(uint16_t opcode);
#endif // BT_HOST_OTHER_STACK

/**
****************************************************************************************
* @brief Look for an event descriptor that could match with the specified event code
*
* @param[in]  code   event code
*
* @return     Pointer the event descriptor (NULL if not found)
*****************************************************************************************
*/
const hci_evt_desc_t *hci_msg_evt_desc_get(uint8_t code);


/**
****************************************************************************************
* @brief Look for an event descriptor that could match with the specified DBG subcode
*
* @param[in]  subcode   DBG event subcode
*
* @return     Pointer the event descriptor (NULL if not found)
*****************************************************************************************
*/
const hci_evt_desc_t *hci_msg_dbg_evt_desc_get(uint8_t subcode);

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    /**
    ****************************************************************************************
    * @brief Look for an event descriptor that could match with the specified LE subcode
    *
    * @param[in]  subcode   LE event subcode
    *
    * @return     Pointer the event descriptor (NULL if not found)
    *****************************************************************************************
    */
    const hci_evt_desc_t *hci_msg_le_evt_desc_get(uint8_t subcode);

#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (HCI_TL_SUPPORT)
#if  (EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Get the maximum parameter size for a specific command
    *
    * This function is used by TL to know the theoretical maximum parameters size for a
    * specific HCI command.
    * Note: if the command is not supported by HCI (unknown), the maximum possible value of
    * 255 bytes is returned.
    *
    * @param[in]  p_cmd       Pointer to HCI command descriptor
    *
    * @return     The command maximum parameters size / 255 if command is unknown
    *****************************************************************************************
    */
    uint8_t hci_msg_cmd_get_max_param_size(const hci_cmd_desc_t *p_cmd);

    /**
    ****************************************************************************************
    * @brief Reject a received HCI command
    *
    * This function creates a CS or CC HCI message to reject a command.
    *
    * @param[in]  p_cmd       Pointer to HCI command descriptor
    * @param[in]  opcode      Received HCI command OP Code
    * @param[in]  error       Status error code to transmit
    * @param[in]  p_payload   Payload of HCI command received
    *****************************************************************************************
    */
    void hci_msg_cmd_reject_send(const hci_cmd_desc_t *p_cmd, uint16_t opcode, uint8_t error, uint8_t *p_payload);

#endif // (EMB_PRESENT)

#if (HOST_PRESENT)
    /**
    ****************************************************************************************
    * @brief Get if a HCI Command status event is expected for a transmitted command
    *
    * @param[in]  p_cmd       Pointer to HCI command descriptor
    *
    * @return True if CS expected, False otherwise
    *****************************************************************************************
    */
    bool hci_msg_cmd_status_exp(const hci_cmd_desc_t *p_cmd);

    #if(!EMB_PRESENT)
        /**
        ****************************************************************************************
        * @brief Extract connection handle from a given format message
        *
        * This function returns the handle contained in a descriptor
        *
        * @param[in]   p_evt_desc    Kernel message containing the HCI command
        * @param[in]   p_payload     Pointer to payload data
        * @param[in]   format  String containning the format
        * @return      Connection Handle
        *****************************************************************************************
        */
        uint16_t hci_msg_evt_look_for_conhdl(const hci_evt_desc_t *p_evt_desc, const uint8_t *p_payload);
    #endif // (!EMB_PRESENT)
#endif // (HOST_PRESENT)

/**
 *****************************************************************************************
 * @brief Unpack received command into an output buffer
 *
 * @param[in]  p_cmd       Pointer to HCI command descriptor
 * @param[out] p_out       Pointer to output Data Buffer
 * @param[in]  p_in        Pointer to input Data Buffer
 * @param[out] p_out_len   Pointer to oOutput size of packed data (in bytes)
 * @param[in]  in_len      Input buffer size (in bytes)
 *
 * @return Status of packing/unpacking execution
 *****************************************************************************************
 */
uint8_t hci_msg_cmd_pkupk(const hci_cmd_desc_t *p_cmd, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len,
                          uint16_t in_len);

/**
 *****************************************************************************************
 * @brief Pack a command complete event into an output buffer
 *
 * @param[in]  p_cmd       Pointer to HCI command descriptor
 * @param[out] p_out       Pointer to output Data Buffer
 * @param[in]  p_in        Pointer to input Data Buffer
 * @param[out] p_out_len   Pointer to oOutput size of packed data (in bytes)
 * @param[in]  in_len      Input buffer size (in bytes)
 *
 * @return Status of packing/unpacking execution
 *****************************************************************************************
 */
uint8_t hci_msg_cmd_cmp_pkupk(const hci_cmd_desc_t *p_cmd, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len,
                              uint16_t in_len);
/**
 *****************************************************************************************
 * @brief Pack an event into an output buffer
 *
 * @param[in]  p_evt       Pointer to HCI event descriptor
 * @param[out] p_out       Pointer to output Data Buffer
 * @param[in]  p_in        Pointer to input Data Buffer
 * @param[out] p_out_len   Pointer to oOutput size of packed data (in bytes)
 * @param[in]  in_len      Input buffer size (in bytes)
 *
 * @return Status of packing/unpacking execution
 *****************************************************************************************
 */
uint8_t hci_msg_evt_pkupk(const hci_evt_desc_t *p_evt, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len,
                          uint16_t in_len);

/**
 ****************************************************************************************
 * @brief Initialize HIC TL part
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 *****************************************************************************************
 */
void hci_tl_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Send an HCI message over TL
 *
 * @param[in]   msg   Kernel message carrying the HCI message
 *****************************************************************************************
 */
void hci_tl_send(struct ke_msg *msg);

#if (BT_HOST_PRESENT && !EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Send an HCI message over TL on a pre-packed format

    * @param[in] type      HCI Message Type (@see enum enum hci_msg_type)
    * @param[in] length    Length of HCI Buffer
    * @param[in] p_buf     Pointer to buffer that contains HCI data in packed format
    * @param[in] cb_done   Callback to execute when Controller stack has handle the HCI message
    *****************************************************************************************
    */
    void hci_tl_pk_send(uint8_t type, uint16_t length, uint8_t *p_buf, hci_done_cb cb_done);
#endif // (BT_HOST_PRESENT && !EMB_PRESENT)
#endif //(HCI_TL_SUPPORT)

#if defined(HOST_PRESENT) || defined(EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Controller destination layer for received HCI command
    *
    * @param[in]  p_cmd       Pointer to HCI command descriptor
    *
    * @return LL Destination (@see enum HCI_MSG_DEST_LL)
    *****************************************************************************************
    */
    uint8_t hci_msg_cmd_ll_dest_get(const hci_cmd_desc_t *p_cmd);

    /**
    ****************************************************************************************
    * @brief Compute Message Task destination from HCI ll destination and payload data
    *
    * @param[in]  ll_dest   HCI message LL destination (@see enum HCI_MSG_DEST_LL)
    * @param[in]  length    HCI message length
    * @param[in]  p_data    Pointer to HCI message data
    *
    * @return Task destination
    *****************************************************************************************
    */
    uint16_t hci_msg_task_dest_compute(uint8_t ll_dest, uint16_t length, const uint8_t *p_data);
#endif // (EMB_PRESENT)

#if (HOST_PRESENT)
    /**
    ****************************************************************************************
    * @brief Host destination transport layer for received HCI event
    *
    * @param[in]  p_evt       Pointer to HCI event descriptor
    *
    * @return HL transport layer Destination (@see enum HCI_MSG_DEST_HL)
    *****************************************************************************************
    */
    uint8_t hci_msg_evt_get_hl_tl_dest(const hci_evt_desc_t *p_evt);

    /**
    ****************************************************************************************
    * @brief Retrieve Host handler local identifier
    *
    * @param[in]  p_evt       Pointer to HCI event descriptor
    *
    * @return Host handler local identifier
    *****************************************************************************************
    */
    uint8_t hci_msg_evt_host_lid_get(const hci_evt_desc_t *p_evt);
#endif // (HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief Initialize Flow Control Structure
 *
 *****************************************************************************************
 */
void hci_fc_init(void);

/**
 ****************************************************************************************
 * @brief count ACL packets sent to Host
 *
 *****************************************************************************************
 */
void hci_fc_acl_packet_sent(void);

/**
 ****************************************************************************************
 * @brief count SCO packets sent to Host
 *
 *****************************************************************************************
 */
void hci_fc_sync_packet_sent(void);

/**
 ****************************************************************************************
 * @brief Calculate number of ACL packets slots available on Host side
 *
 * @return number of packets available
 *****************************************************************************************
 */
uint16_t hci_fc_check_host_available_nb_acl_packets(void);

/**
 ****************************************************************************************
 * @brief Calculate number of SCO packets slots available on Host side
 *
 * @return number of packets available
 *****************************************************************************************
 */
uint16_t hci_fc_check_host_available_nb_sync_packets(void);

#if (HCI_TL_SUPPORT)
    const char *hci_msg_cmd_ret_para_get(const hci_cmd_desc_t *p_cmd);
#endif

#endif //HCI_PRESENT

/// @} HCI

#endif // HCI_INT_H_
