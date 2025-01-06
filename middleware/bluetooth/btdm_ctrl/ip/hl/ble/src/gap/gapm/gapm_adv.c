/**
 ****************************************************************************************
 *
 * @file gapm_adv.c
 *
 * @brief Generic Access Profile Manager - Advertising manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ADV Generic Access Profile Manager - Advertising manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_BROADCASTER)

#include <string.h>

#include "gap.h"
#include "gapc.h"
#include "gapm_task.h"
#include "gapm_int.h"
#include "ke_mem.h"
#include "hci.h"
#include "co_utils.h"
#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Mask allowing to map prop field of GAPM_ACTIVITY_CREATE_CMD message structure on
/// adv_evt_properties field of LE Set Extended Advertising Parameter Command HCI
/// message structure.
#define GAPM_ADV_EVT_PROP_MASK                  (0x007F)

/// Bit field for Periodic Advertising Properties field (LE Set Periodic Advertising Parameters command)
#define GAPM_ADV_PERIOD_PROP_TX_POWER_BIT       (6)

/// Maximum advertising data fragment length that can be provided to the controller through HCI
#define GAPM_ADV_FRGMT_ADV_DATA_LENGTH          (251)
/// Maximum advertising data fragment length that can be provided to the controller through HCI
#define GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH     (251)
/// Maximum advertising data fragment length that can be provided to the controller through HCI
#define GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH      (252)

/// Length of AD Type flags
#define GAPM_ADV_AD_TYPE_FLAGS_LENGTH           (3)
/// Length of AD Type flags information
#define GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH      (2)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Available value for scan_req_notif_en field in LE Set Extended Advertising Parameters HCI command
enum gapm_adv_scan_req_notif_en
{
    /// Scan request notifications disabled
    GAPM_ADV_SCAN_REQ_NOTIF_DIS = 0,
    /// Scan request notification enabled
    GAPM_ADV_SCAN_REQ_NOTIF_EN,
};

/// Bit field bit positions for advertising activity info parameter
enum gapm_adv_info_bit_pos
{
    /// Indicate that scan response data can be set (meaning advertising is a scannable advertising)
    GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA_POS = 0,
    /// Indicate that advertising data can be set
    ///     Use of advertising data is excluded in following cases:
    ///         - Legacy directed advertising
    ///         - Scannable extended advertising (undirected or directed)
    GAPM_ADV_INFO_ACPT_ADV_DATA_POS,
    /// Indicate that advertising data has been set at least one time for the activity
    GAPM_ADV_INFO_ADV_DATA_SET_POS,
    /// Indicate that scan response data has been set (mandatory only for extended advertising)
    GAPM_ADV_INFO_SCAN_RSP_DATA_SET_POS,
    /// Indicate that periodic advertising data has been set (mandatory only for periodic advertising)
    GAPM_ADV_INFO_PER_ADV_DATA_SET_POS,
    /// Indicate that notification of Scan Requests has been required by application
    GAPM_ADV_INFO_SCAN_REQ_NTF_POS,
    /// Indicate that advertising is running
    GAPM_ADV_INFO_ADV_EN_POS,
    /// Indicate that periodic advertising is running
    GAPM_ADV_INFO_PER_ADV_EN_POS,
};

/// Bit field bit values for advertising activity info parameter
enum gapm_adv_info_bit
{
    /// Indicate that scan response data can be set (meaning advertising is a scannable advertising)
    GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA_BIT = CO_BIT(GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA_POS),
    /// Indicate that advertising data can be set
    ///     Use of advertising data is excluded in following cases:
    ///         - Legacy directed advertising
    ///         - Scannable extended advertising (undirected or directed)
    GAPM_ADV_INFO_ACPT_ADV_DATA_BIT      = CO_BIT(GAPM_ADV_INFO_ACPT_ADV_DATA_POS),
    /// Indicate that advertising data has been set at least one time for the activity
    GAPM_ADV_INFO_ADV_DATA_SET_BIT       = CO_BIT(GAPM_ADV_INFO_ADV_DATA_SET_POS),
    /// Indicate that scan response data has been set (mandatory only for extended advertising)
    GAPM_ADV_INFO_SCAN_RSP_DATA_SET_BIT  = CO_BIT(GAPM_ADV_INFO_SCAN_RSP_DATA_SET_POS),
    /// Indicate that periodic advertising data has been set (mandatory only for periodic advertising)
    GAPM_ADV_INFO_PER_ADV_DATA_SET_BIT   = CO_BIT(GAPM_ADV_INFO_PER_ADV_DATA_SET_POS),
    /// Indicate that notification of Scan Requests has been required by application
    GAPM_ADV_INFO_SCAN_REQ_NTF_BIT       = CO_BIT(GAPM_ADV_INFO_SCAN_REQ_NTF_POS),
    /// Indicate that advertising is running
    GAPM_ADV_INFO_ADV_EN_BIT             = CO_BIT(GAPM_ADV_INFO_ADV_EN_POS),
    /// Indicate that periodic advertising is running
    GAPM_ADV_INFO_PER_ADV_EN_BIT         = CO_BIT(GAPM_ADV_INFO_PER_ADV_EN_POS),
};

/*
 * MACROS
 ****************************************************************************************
 */



/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a LE Set Extended Advertising Parameters command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_CREATE_CMD message parameters
 ****************************************************************************************
 */
#ifndef RWIP_BLE_4
__STATIC void gapm_adv_send_hci_le_set_ext_adv_param_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_activity_create_adv_cmd *p_param)
{
    // Allocate HCI command message
    struct hci_le_set_ext_adv_param_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE,
            hci_le_set_ext_adv_param_cmd);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;

    // Advertising event properties
    //    Following bits of field in HCI command can be directly mapped on those in received command
    //       - Connectable advertising bit (0)
    //       - Scannable advertising bit (1)
    //       - Directed advertising bit (2)
    //       - High Duty Cycle directed connectable advertising bit (3)
    //       - Legacy advertising bit (4)
    //       - Anonymous advertising (5)
    //       - Include TX power in extended headed (6)
    p_cmd->adv_evt_properties = (p_param->adv_param.prop & GAPM_ADV_EVT_PROP_MASK);

    // TODO [LT] comment: Add co_read24 and co_write24
    // Set configuration for advertising on primary channel
    //    Primary advertising minimum interval
    p_cmd->prim_adv_intv_min[0] = (uint8_t)p_param->adv_param.prim_cfg.adv_intv_min;
    p_cmd->prim_adv_intv_min[1] = (uint8_t)(p_param->adv_param.prim_cfg.adv_intv_min >> 8);
    p_cmd->prim_adv_intv_min[2] = (uint8_t)(p_param->adv_param.prim_cfg.adv_intv_min >> 16);
    //    Primary advertising maximum interval
    p_cmd->prim_adv_intv_max[0] = (uint8_t)p_param->adv_param.prim_cfg.adv_intv_max;
    p_cmd->prim_adv_intv_max[1] = (uint8_t)(p_param->adv_param.prim_cfg.adv_intv_max >> 8);
    p_cmd->prim_adv_intv_max[2] = (uint8_t)(p_param->adv_param.prim_cfg.adv_intv_max >> 16);
    //    Primary advertising channel map
    p_cmd->prim_adv_ch_map = p_param->adv_param.prim_cfg.chnl_map;
    //    Primary advertising PHY
    p_cmd->prim_adv_phy = p_param->adv_param.prim_cfg.phy;

    if (p_actv_adv->common.subtype >= GAPM_ADV_TYPE_EXTENDED)
    {
        // Set configuration for advertising on secondary channel
        //    Secondary advertising max skip
        p_cmd->sec_adv_max_skip = p_param->adv_param.second_cfg.max_skip;
        //    Secondary advertising PHY
        p_cmd->sec_adv_phy = p_param->adv_param.second_cfg.phy;
        //    Advertising SID
        p_cmd->adv_sid = p_param->adv_param.second_cfg.adv_sid;
    }

    //  Own address type
    p_cmd->own_addr_type = gapm_actv_get_hci_own_addr_type(p_actv_adv->common.own_addr_type);

    //  Peer address if directed advertising is used or if resolvable private address generated by controller has to be used
    if ((GETB(p_param->adv_param.prop, GAPM_ADV_PROP_DIRECTED))
            || (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT))
    {
        p_cmd->peer_addr_type = p_param->adv_param.peer_addr.addr_type;
        memcpy(&p_cmd->peer_addr.addr[0], &p_param->adv_param.peer_addr.addr.addr[0], BD_ADDR_LEN);
    }

    // Advertising filter policy
    p_cmd->adv_filt_policy = p_param->adv_param.filter_pol;
    // Advertising TX power
    p_cmd->adv_tx_pwr = p_param->adv_param.max_tx_pwr;
    // Enable scan request notification
    if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCAN_REQ_NTF_EN))
    {
        p_cmd->scan_req_notif_en = GAPM_ADV_SCAN_REQ_NOTIF_EN;

        // Keep in mind that notification for received scan requests is enabled
        SETB(p_actv_adv->common.info, GAPM_ADV_INFO_SCAN_REQ_NTF, true);
    }
    else
    {
        p_cmd->scan_req_notif_en = GAPM_ADV_SCAN_REQ_NOTIF_DIS;
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE;
}

#else /* RWIP_BLE_4 */
__STATIC void gapm_adv_send_hci_le_set_param_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_activity_create_adv_cmd *p_param)
{
    // Allocate HCI command message
    struct hci_le_set_adv_param_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_ADV_PARAM_CMD_OPCODE,
            hci_le_set_adv_param_cmd);

    p_cmd->adv_intv_max = p_param->adv_param.prim_cfg.adv_intv_max;
    p_cmd->adv_intv_min = p_param->adv_param.prim_cfg.adv_intv_min;
    p_cmd->adv_type = ADV_CONN_UNDIR; //TODO

    //  Own address type
    p_cmd->own_addr_type = gapm_actv_get_hci_own_addr_type(p_actv_adv->common.own_addr_type);

    //  Peer address if directed advertising is used or if resolvable private address generated by controller has to be used
    if ((GETB(p_param->adv_param.prop, GAPM_ADV_PROP_DIRECTED))
            || (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT))
    {
        p_cmd->peer_addr_type = p_param->adv_param.peer_addr.addr_type;
        memcpy(&p_cmd->peer_addr.addr[0], &p_param->adv_param.peer_addr.addr.addr[0], BD_ADDR_LEN);
    }

    p_cmd->adv_chnl_map = ADV_ALL_CHNLS_EN;
    // Advertising filter policy
    p_cmd->adv_filt_policy = p_param->adv_param.filter_pol;


    if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCAN_REQ_NTF_EN))
    {
        // Keep in mind that notification for received scan requests is enabled
        SETB(p_actv_adv->common.info, GAPM_ADV_INFO_SCAN_REQ_NTF, true);
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_ADV_PARAM_CMD_OPCODE;
}

#endif /* RWIP_BLE_4 */


/**
 ****************************************************************************************
 * @brief Send a HCI LE Set Extended Advertising Data command to the controller.
 *
 * @param[in] p_actv_adv    Advertising activity for which the advertising data has to be set.
 * @param[in] p_param       Pointer to the received GAPM_SET_ADV_DATA_CMD message.
 * It contains the report to send to the controller
 *
 * @return Length of report fragment copied in the command.
 ****************************************************************************************
 */

#ifndef RWIP_BLE_4
__STATIC uint8_t gapm_adv_send_hci_le_set_ext_adv_data_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_set_adv_data_cmd *p_param)
{
    // HCI command message to be sent
    struct hci_le_set_ext_adv_data_cmd *p_cmd;
    // Offset for writing
    uint8_t wr_offset = 0;
    // Compute length of data to written
    uint8_t wr_length;
    // Set operation code
    uint8_t set_operation;

    // Check if data must be fragmented (AD type flags have to be added)
    if (p_param->length <= (GAPM_ADV_FRGMT_ADV_DATA_LENGTH - GAPM_ADV_AD_TYPE_FLAGS_LENGTH))
    {
        set_operation = ADV_DATA_OP_COMPLETE;

        // No AD_TYPE for beacons
        if (p_actv_adv->mode != GAPM_ADV_MODE_BEACON && p_actv_adv->mode != GAPM_ADV_MODE_CUSTOMIZE)
        {
            // Will add AD type flags
            wr_offset = GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
        }
        wr_length = p_param->length;
    }
    else
    {
        // Check if we build the first, the last or an intermediate fragment
        if (p_actv_adv->data_offset == 0)
        {
            set_operation = ADV_DATA_OP_FIRST_FRAG;

            if (p_actv_adv->mode != GAPM_ADV_MODE_BEACON && p_actv_adv->mode != GAPM_ADV_MODE_CUSTOMIZE)
            {
                // Will add AD type flags
                wr_offset = GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
                wr_length = GAPM_ADV_FRGMT_ADV_DATA_LENGTH - GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
            }
            else
            {
                // No AD_TYPE for beacons
                wr_length = GAPM_ADV_FRGMT_ADV_DATA_LENGTH;
            }
        }
        else
        {
            // Get remaining length to be written
            uint16_t rem_length = p_param->length - p_actv_adv->data_offset;

            // Check if last fragment or intermediate fragment
            if (rem_length <= GAPM_ADV_FRGMT_ADV_DATA_LENGTH)
            {
                set_operation = ADV_DATA_OP_LAST_FRAG;

                // Write the whole remaining data
                wr_length = rem_length;
            }
            else
            {
                set_operation = ADV_DATA_OP_INTERMEDIATE_FRAG;

                // More fragment are needed
                wr_length = GAPM_ADV_FRGMT_ADV_DATA_LENGTH;
            }
        }
    }

    p_cmd = KE_MSG_ALLOC_DYN(HCI_COMMAND,
                             0, HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE,
                             hci_le_set_ext_adv_data_cmd, wr_length + wr_offset);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;
    // Set operation, fragment preferences and length
    p_cmd->operation = set_operation;
    p_cmd->frag_pref = ADV_DATA_MAY_FRAG;
    p_cmd->data_len = wr_length + wr_offset;

    // Set the AD type flags
    if (wr_offset)
    {
        // Length of ad type flags
        p_cmd->data[0] = GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH;
        p_cmd->data[1] = GAP_AD_TYPE_FLAGS;
        //for bt/ble dual mode,just set here ,temporarily .
        //p_cmd->data[2] = GAP_BR_EDR_NOT_SUPPORTED;
        p_cmd->data[2] = GAP_BR_EDR_NOT_SUPPORTED | GAP_SIMUL_BR_EDR_LE_HOST | GAP_SIMUL_BR_EDR_LE_CONTROLLER;

        // Set mode in ad_type
        switch (p_actv_adv->mode)
        {
        // General discoverable mode
        case GAPM_ADV_MODE_GEN_DISC:
        {
            p_cmd->data[2] |= GAP_LE_GEN_DISCOVERABLE_FLG;
        }
        break;

        // Limited discoverable mode
        case GAPM_ADV_MODE_LIM_DISC:
        {
            p_cmd->data[2] |= GAP_LE_LIM_DISCOVERABLE_FLG;
        }
        break;
        // one-click connect for dual mode
        case GAPM_ADV_MODE_DULMODE:
        {
            p_cmd->data[2] = GAP_LE_GEN_DISCOVERABLE_FLG;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    // Copy the data
    memcpy(&p_cmd->data[wr_offset], &p_param->data[p_actv_adv->data_offset], wr_length);

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE;

    return (wr_length);
}

#else /* RWIP_BLE_4 */

__STATIC uint8_t gapm_adv_send_hci_le_set_adv_data_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_set_adv_data_cmd *p_param)
{
    // HCI command message to be sent
    struct hci_le_set_adv_data_cmd *p_cmd;
    // Offset for writing
    uint8_t wr_offset = 0;
    // Compute length of data to written
    uint8_t wr_length;
    // Set operation code
    //uint8_t set_operation;

    // Check if data must be fragmented (AD type flags have to be added)
    if (p_param->length <= (ADV_DATA_LEN - GAPM_ADV_AD_TYPE_FLAGS_LENGTH))
    {
        //set_operation = ADV_DATA_OP_COMPLETE;

        // No AD_TYPE for beacons
        if (p_actv_adv->mode != GAPM_ADV_MODE_BEACON)
        {
            // Will add AD type flags
            wr_offset = GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
        }
        wr_length = p_param->length;
    }
    else
    {
        // Check if we build the first, the last or an intermediate fragment
        if (p_actv_adv->data_offset == 0)
        {
            //set_operation = ADV_DATA_OP_FIRST_FRAG;

            if (p_actv_adv->mode != GAPM_ADV_MODE_BEACON)
            {
                // Will add AD type flags
                wr_offset = GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
                wr_length = ADV_DATA_LEN - GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
            }
            else
            {
                // No AD_TYPE for beacons
                wr_length = ADV_DATA_LEN;
            }
        }
        else
        {
            // Get remaining length to be written
            uint16_t rem_length = p_param->length - p_actv_adv->data_offset;

            // Check if last fragment or intermediate fragment
            if (rem_length <= ADV_DATA_LEN)
            {
                //set_operation = ADV_DATA_OP_LAST_FRAG;

                // Write the whole remaining data
                wr_length = rem_length;
            }
            else
            {
                //set_operation = ADV_DATA_OP_INTERMEDIATE_FRAG;

                // More fragment are needed
                wr_length = ADV_DATA_LEN;
            }
        }
    }

    p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
                         0, HCI_LE_SET_ADV_DATA_CMD_OPCODE,
                         hci_le_set_adv_data_cmd); //, wr_length + wr_offset

    p_cmd->adv_data_len = wr_length + wr_offset;

    // Set the AD type flags
    if (wr_offset)
    {
        // Length of ad type flags
        p_cmd->data.data[0] = GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH;
        p_cmd->data.data[1] = GAP_AD_TYPE_FLAGS;
        p_cmd->data.data[2] = GAP_BR_EDR_NOT_SUPPORTED;

        // Set mode in ad_type
        switch (p_actv_adv->mode)
        {
        // General discoverable mode
        case GAPM_ADV_MODE_GEN_DISC:
        {
            p_cmd->data.data[2] |= GAP_LE_GEN_DISCOVERABLE_FLG;
        }
        break;

        // Limited discoverable mode
        case GAPM_ADV_MODE_LIM_DISC:
        {
            p_cmd->data.data[2] |= GAP_LE_LIM_DISCOVERABLE_FLG;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    // Copy the data
    memcpy(&p_cmd->data.data[wr_offset], &p_param->data[p_actv_adv->data_offset], wr_length);

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_ADV_DATA_CMD_OPCODE;

    return (wr_length);
}


#endif


/**
 ****************************************************************************************
 * @brief Send a HCI LE Set External Scan Response Data command to the controller.
 * This function is in charge of splitting the scan response data provided by the
 * application in several fragment.
 *
 * @param[in] p_actv_adv    Pointer to the advertising activity structure.
 * @param[in] p_param       Pointer to the received GAPM_SET_ADV_DATA_CMD message
 *
 * @return Length of fragment that has been sent to the controller
 ****************************************************************************************
 */
#ifndef RWIP_BLE_4
__STATIC uint8_t gapm_adv_send_hci_le_set_ext_scan_rsp_data_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_set_adv_data_cmd *p_param)
{
    // HCI command message to be sent
    struct hci_le_set_ext_scan_rsp_data_cmd *p_cmd;
    // Compute length of data to written
    uint8_t wr_length;
    // Write offset
    uint8_t wr_offset = 0;
    // Additional length
    uint8_t add_len = (GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_ADV_DATA))
                      ? 0 : GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
    // Set operation code
    uint8_t set_operation;

    // Check if data must be fragmented
    if (p_param->length <= (GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH - add_len))
    {
        set_operation = ADV_DATA_OP_COMPLETE;

        // Copy whole provided data
        wr_length = p_param->length;
        wr_offset = add_len;
    }
    else
    {
        // Check if we build the first, the last or an intermediate fragment
        if (p_actv_adv->data_offset == 0)
        {
            set_operation = ADV_DATA_OP_FIRST_FRAG;

            // Will add AD type flags
            wr_length = GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH - add_len;
            wr_offset = add_len;
        }
        else
        {
            // Get remaining length to be written
            uint16_t rem_length = p_param->length - p_actv_adv->data_offset;

            // Check if last fragment or intermediate fragment
            if (rem_length <= GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH)
            {
                set_operation = ADV_DATA_OP_LAST_FRAG;

                // Write the whole remaining data
                wr_length = rem_length;
            }
            else
            {
                set_operation = ADV_DATA_OP_INTERMEDIATE_FRAG;

                // More fragment are needed
                wr_length = GAPM_ADV_FRGMT_SCAN_RSP_DATA_LENGTH;
            }
        }
    }

    p_cmd = KE_MSG_ALLOC_DYN(HCI_COMMAND,
                             0, HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE,
                             hci_le_set_ext_scan_rsp_data_cmd, wr_length + wr_offset);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;
    // Set operation, fragment preferences and length
    p_cmd->operation = set_operation;
    p_cmd->frag_pref = ADV_DATA_MAY_FRAG;
    p_cmd->data_len = wr_length + wr_offset;

    // Set the AD type flags
    if (wr_offset)
    {
        // Length of ad type flags
        p_cmd->data[0] = GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH;
        p_cmd->data[1] = GAP_AD_TYPE_FLAGS;
        p_cmd->data[2] = GAP_BR_EDR_NOT_SUPPORTED;

        // Set mode in ad_type
        switch (p_actv_adv->mode)
        {
        // General discoverable mode
        case GAPM_ADV_MODE_GEN_DISC:
        {
            p_cmd->data[2] |= GAP_LE_GEN_DISCOVERABLE_FLG;
        }
        break;

        // Limited discoverable mode
        case GAPM_ADV_MODE_LIM_DISC:
        {
            p_cmd->data[2] |= GAP_LE_LIM_DISCOVERABLE_FLG;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    // Copy the data
    memcpy(&p_cmd->data[wr_offset], &p_param->data[p_actv_adv->data_offset], wr_length);

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE;

    return (wr_length);
}

#else

__STATIC uint8_t gapm_adv_send_hci_le_set_scan_rsp_data_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_set_adv_data_cmd *p_param)
{
    // HCI command message to be sent
    struct hci_le_set_scan_rsp_data_cmd *p_cmd;
    // Compute length of data to written
    uint8_t wr_length;
    // Write offset
    uint8_t wr_offset = 0;
    // Additional length
    uint8_t add_len = (GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_ADV_DATA))
                      ? 0 : GAPM_ADV_AD_TYPE_FLAGS_LENGTH;

    /* begin: for test  */
    //add_len = GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
    /* end: */
    // Set operation code
    //uint8_t set_operation;

    //TRC_PRINTF("add_len:%d, %d", add_len, p_param->length);

    // Check if data must be fragmented
    if (p_param->length <= (SCAN_RSP_DATA_LEN - add_len))
    {
        //set_operation = ADV_DATA_OP_COMPLETE;

        // Copy whole provided data
        wr_length = p_param->length;
        wr_offset = add_len;
    }
    else
    {
        // Check if we build the first, the last or an intermediate fragment
        if (p_actv_adv->data_offset == 0)
        {
            //set_operation = ADV_DATA_OP_FIRST_FRAG;

            // Will add AD type flags
            wr_length = SCAN_RSP_DATA_LEN - add_len;
            wr_offset = add_len;
        }
        else
        {
            // Get remaining length to be written
            uint16_t rem_length = p_param->length - p_actv_adv->data_offset;

            // Check if last fragment or intermediate fragment
            if (rem_length <= SCAN_RSP_DATA_LEN)
            {
                //set_operation = ADV_DATA_OP_LAST_FRAG;

                // Write the whole remaining data
                wr_length = rem_length;
            }
            else
            {
                //set_operation = ADV_DATA_OP_INTERMEDIATE_FRAG;

                // More fragment are needed
                wr_length = SCAN_RSP_DATA_LEN;
            }
        }
    }

    p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
                         0, HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE,
                         hci_le_set_scan_rsp_data_cmd);

    //TRC_PRINTF("scan_rsp:0x%x,%d,%d", (uint32_t)p_cmd,wr_length,wr_offset);

    p_cmd->scan_rsp_data_len = wr_length + wr_offset;

    // Set the AD type flags
    if (wr_offset)
    {
        // Length of ad type flags
        p_cmd->data.data[0] = GAPM_ADV_AD_TYPE_FLAGS_INFO_LENGTH;
        p_cmd->data.data[1] = GAP_AD_TYPE_FLAGS;
        p_cmd->data.data[2] = GAP_BR_EDR_NOT_SUPPORTED;

        // Set mode in ad_type
        switch (p_actv_adv->mode)
        {
        // General discoverable mode
        case GAPM_ADV_MODE_GEN_DISC:
        {
            p_cmd->data.data[2] |= GAP_LE_GEN_DISCOVERABLE_FLG;
        }
        break;

        // Limited discoverable mode
        case GAPM_ADV_MODE_LIM_DISC:
        {
            p_cmd->data.data[2] |= GAP_LE_LIM_DISCOVERABLE_FLG;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    // Copy the data
    memcpy(&p_cmd->data.data[wr_offset], &p_param->data[p_actv_adv->data_offset], wr_length);

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE;

    return (wr_length);
}


#endif


/**
 ****************************************************************************************
 * @brief Send a LE Set Extended Advertising Enable command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_START_CMD message parameters if advertising has
 * to be enabled, NULL pointer if has to be disabled
 ****************************************************************************************
 */

#ifndef RWIP_BLE_4
__STATIC void gapm_adv_send_hci_le_set_ext_adv_en_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_activity_start_cmd *p_param)
{
    // Allocate HCI command message
    struct hci_le_set_ext_adv_en_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE,
            hci_le_set_ext_adv_en_cmd);

    // Enable or disable
    p_cmd->enable = (p_param) ? true : false;
    // Only one set is considered
    p_cmd->nb_sets = 1;
    // Use local advertising identifier as advertising handle
    p_cmd->sets[0].adv_hdl = p_actv_adv->common.idx;

    if (p_param)
    {
        if (p_actv_adv->mode == GAPM_ADV_MODE_LIM_DISC)
        {
            // Force duration for limited discovery mode
            p_cmd->sets[0].duration = GAP_TMR_LIM_ADV_TIMEOUT;
        }
        else
        {
            p_cmd->sets[0].duration = p_param->u_param.adv_add_param.duration;
        }
        p_cmd->sets[0].max_ext_adv_evt = p_param->u_param.adv_add_param.max_adv_evt;
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE;
}

#else

__STATIC void gapm_adv_send_hci_le_set_adv_en_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_activity_start_cmd *p_param)
{
    // Allocate HCI command message
    struct hci_le_set_adv_en_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
                                          0, HCI_LE_SET_ADV_EN_CMD_OPCODE,
                                          hci_le_set_adv_en_cmd);

    // Enable or disable
    p_cmd->adv_en = (p_param) ? true : false;

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_ADV_EN_CMD_OPCODE;
}


#endif

#ifndef GAPM_SLIM
/**
 ****************************************************************************************
 * @brief Send a LE Set Periodic Advertising Parameters command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_adv_send_hci_le_set_per_adv_param_cmd(struct gapm_actv_adv_tag *p_actv_adv)
{
    // Retrieve GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_create_adv_cmd *p_param =
        (struct gapm_activity_create_adv_cmd *)gapm_get_operation_ptr(GAPM_OP_AIR);
    // Allocate HCI command message
    struct hci_le_set_per_adv_param_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE,
            hci_le_set_per_adv_param_cmd);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;
    // Advertising interval
    p_cmd->adv_intv_min = p_param->adv_param.period_cfg.adv_intv_min;
    p_cmd->adv_intv_max = p_param->adv_param.period_cfg.adv_intv_max;
    // Check if advertising PDU shall contain TX Power
    if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_PER_TX_PWR))
    {
        p_cmd->adv_prop = GAPM_ADV_PERIOD_PROP_TX_POWER_BIT;
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Send a HCI LE Set Periodic Advertising Data to the controller.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure for which the periodic
 * advertising data has to be set.
 * @param[in] p_param       Pointer to the received GAPM_SET_ADV_DATA_CMD message.
 *
 * @return Length of periodic advertising data fragment provided to the controller.
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_send_hci_le_set_per_adv_data_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_set_adv_data_cmd *p_param)
{
    // HCI command message to be sent
    struct hci_le_set_per_adv_data_cmd *p_cmd;
    // Compute length of data to written
    uint8_t wr_length;
    // Set operation code
    uint8_t set_operation;

    // Check if data must be fragmented
    if (p_param->length <= GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH)
    {
        set_operation = ADV_DATA_OP_COMPLETE;

        // Copy whole provided data
        wr_length = p_param->length;
    }
    else
    {
        // Check if we build the first, the last or an intermediate fragment
        if (p_actv_adv->data_offset == 0)
        {
            set_operation = ADV_DATA_OP_FIRST_FRAG;

            // Will add AD type flags
            wr_length = GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH;
        }
        else
        {
            // Get remaining length to be written
            uint16_t rem_length = p_param->length - p_actv_adv->data_offset;

            // Check if last fragment or intermediate fragment
            if (rem_length <= GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH)
            {
                set_operation = ADV_DATA_OP_LAST_FRAG;

                // Write the whole remaining data
                wr_length = rem_length;
            }
            else
            {
                set_operation = ADV_DATA_OP_INTERMEDIATE_FRAG;

                // More fragment are needed
                wr_length = GAPM_ADV_FRGMT_PER_ADV_DATA_LENGTH;
            }
        }
    }

    p_cmd = KE_MSG_ALLOC_DYN(HCI_COMMAND,
                             0, HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE,
                             hci_le_set_per_adv_data_cmd, wr_length);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;
    // Set operation and written length
    p_cmd->operation = set_operation;
    p_cmd->data_len = wr_length;

    // Copy the data
    memcpy(&p_cmd->data[0], &p_param->data[p_actv_adv->data_offset], wr_length);

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE;

    return (wr_length);
}

/**
 ****************************************************************************************
 * @brief Send a LE Set Periodic Advertising Enable command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 * @param[in] enable        Indicate if periodic advertising must be enabled or disabled
 ****************************************************************************************
 */
__STATIC void gapm_adv_send_hci_le_set_per_adv_en_cmd(struct gapm_actv_adv_tag *p_actv_adv,
        bool enable)
{
    // Allocate HCI command message
    struct hci_le_set_per_adv_en_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_PER_ADV_EN_CMD_OPCODE,
            hci_le_set_per_adv_en_cmd);

    // Enable or disable
    p_cmd->enable = (uint8_t)enable;
    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_SET_PER_ADV_EN_CMD_OPCODE;
}


#endif // !GAPM_SLIM
/**
 ****************************************************************************************
 * @brief Send a LE Remove Advertising Set command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_adv_send_hci_le_rmv_adv_set_cmd(struct gapm_actv_adv_tag *p_actv_adv)
{
    // Allocate HCI command message
    struct hci_le_rem_adv_set_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
                                           0, HCI_LE_RMV_ADV_SET_CMD_OPCODE,
                                           hci_le_rem_adv_set_cmd);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv_adv->common.idx;

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_adv->common.next_exp_opcode = HCI_LE_RMV_ADV_SET_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Allocate, fill and send a GAPM_EXT_ADV_REPORT_IND message to the application
 * after reception of a Scan Request Received notification from the controller.
 *
 * @param[in] p_actv_adv        Activity for which the notification has been received
 * @param[in] p_event           Event message containing scan request information
 ****************************************************************************************
 */
__STATIC void gapm_adv_send_scan_request_ind(struct gapm_actv_adv_tag *p_actv_adv,
        struct hci_le_scan_req_rcvd_evt const *p_event)
{
    // Allocate a GAPM_SCAN_REQUEST_IND message
    struct gapm_scan_request_ind *p_ind = KE_MSG_ALLOC(GAPM_SCAN_REQUEST_IND,
                                          p_actv_adv->common.requester, TASK_GAPM,
                                          gapm_scan_request_ind);

    // Fill the message
    p_ind->actv_idx = p_actv_adv->common.idx;
    p_ind->trans_addr.addr_type = p_event->scan_addr_type;
    memcpy(&p_ind->trans_addr.addr.addr[0], &p_event->scan_addr.addr[0], BD_ADDR_LEN);

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Function used to manage sending of fragments for an advertising/scan response
 * data. It is called upon reception of GAPM_SET_ADV_DATA_CMD message from the application
 * and each time a command complete event is received from controller for following
 * HCI commands:
 *     - HCI LE Set Extended Advertising Data command
 *     - HCI LE Set Extended Scan Response Data command
 *     - HCI LE Set Periodic Advertising Data command
 *
 * @param[in] p_actv_adv    Pointer to the advertising activity structure. data_offset
 *                          value is used to monitor if the whole content of the
 *                          data has been sent to the controller
 * @param[in] p_param       Pointer to the received GAPM_SET_ADV_DATA_CMD message
 * @param[in] cmp_evt       Indicate if function is called after reception of an
 *                          HCI command complete event
 *
 * @return True if a new fragment has been pushed to the controller, else false
 ****************************************************************************************
 */
__STATIC bool gapm_adv_set_data(struct gapm_actv_adv_tag *p_actv_adv,
                                struct gapm_set_adv_data_cmd *p_param,
                                bool cmp_evt)
{
    // Status returned by the function
    bool fragment_pushed = true;

    do
    {
        if (cmp_evt)
        {
            // Check if remaining data has to be downloaded
            if (p_actv_adv->data_offset == p_param->length)
            {
                // Keep in mind that advertising data has been set
                if (p_param->operation == GAPM_SET_ADV_DATA)
                {
                    SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_DATA_SET, true);
                }
                else if (p_param->operation == GAPM_SET_SCAN_RSP_DATA)
                {
                    SETB(p_actv_adv->common.info, GAPM_ADV_INFO_SCAN_RSP_DATA_SET, true);
                }
                else
                {
                    SETB(p_actv_adv->common.info, GAPM_ADV_INFO_PER_ADV_DATA_SET, true);
                }

                // No need to push another fragment
                fragment_pushed = false;
                break;
            }
        }

        // Send next fragment
        switch (p_param->operation)
        {
        case GAPM_SET_ADV_DATA:
        {
#ifndef RWIP_BLE_4
            p_actv_adv->data_offset += gapm_adv_send_hci_le_set_ext_adv_data_cmd(p_actv_adv, p_param);
#else
            p_actv_adv->data_offset += gapm_adv_send_hci_le_set_adv_data_cmd(p_actv_adv, p_param);
#endif
        }
        break;

        case GAPM_SET_SCAN_RSP_DATA:
        {
#ifndef RWIP_BLE_4
            p_actv_adv->data_offset += gapm_adv_send_hci_le_set_ext_scan_rsp_data_cmd(p_actv_adv, p_param);
#else
            p_actv_adv->data_offset += gapm_adv_send_hci_le_set_scan_rsp_data_cmd(p_actv_adv, p_param);
#endif
        }
        break;
#ifndef GAPM_SLIM
        case GAPM_SET_PERIOD_ADV_DATA:
        {
            p_actv_adv->data_offset += gapm_adv_send_hci_le_set_per_adv_data_cmd(p_actv_adv, p_param);
        }
        break;
#endif //!GAPM_SLIM
        default:
        {
            // Should not happen
        }
        break;
        }
    }
    while (0);

    return (fragment_pushed);
}

/**
 ****************************************************************************************
 * @brief Check if advertising has be properly prepared and can be started.
 * Following rules apply:
 *     - If advertising data must be provided, it has to be set at least once by the application
 * using the GAPM_SET_ADV_DATA_CMD even if its length is 0.
 *     - If scan response data must be provided, it has to be set at least once
 * by the application using the GAPM_SET_ADV_DATA_CMD except if advertising is a legacy
 * advertising.
 *     - If advertising is a periodic advertising, periodic advertising data has to
 * be provided at least once.
 *
 * @param[in] p_actv_adv    Pointer to the advertising activity structure
 *
 * @return GAP_ERR_NO_ERROR if advertising can be started, else GAP_ERR_COMMAND_DISALLOWED
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_check(struct gapm_actv_adv_tag *p_actv_adv)
{
    // Error code, command disallowed by default
    uint8_t error = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        // If advertising data can be set, advertising data has to be set at least once
        if (GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_ADV_DATA)
                && !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_DATA_SET))
        {
            break;
        }

        // If scan response data can be set, it has to be set at least once, except
        // if advertising is a legacy advertising
        if (GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA)
                && (p_actv_adv->common.subtype != GAPM_ADV_TYPE_LEGACY)
                && !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_SCAN_RSP_DATA_SET))
        {
            break;
        }

        // If periodic advertising, periodic advertising data has to be set
        if ((p_actv_adv->common.subtype == GAPM_ADV_TYPE_PERIODIC)
                && !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_PER_ADV_DATA_SET))
        {
            break;
        }

        error = GAP_ERR_NO_ERROR;
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Check provided advertising parameters in order to know if advertising
 * activity can be created:
 *      - Check if roles supported by the device allow advertising
 *      - Check advertising properties
 *      - Check privacy configuration
 * For several parameters such as advertising interval, channel map, ... host relies
 * on checks performed by the controller.
 *
 * @param[in] p_param   Pointer to the received GAPM_ACTIVITY_CREATE_CMD message.
 *
 * @return GAP_ERR_NO_ERROR if advertising activity can be created, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_check_param(struct gapm_activity_create_adv_cmd *p_param)
{
    // Error code, invalid parameter by default
    uint8_t error = GAP_ERR_INVALID_PARAM;

    do
    {
        // Current role must at least be broadcaster
        uint8_t needed_role = GAP_ROLE_BROADCASTER;

        if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE))
        {
#if (BLE_PERIPHERAL)
            if (gapm_env.connections == BLE_CONNECTION_MAX)
            {
                // No more connection can be established
                error = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }

            needed_role = GAP_ROLE_PERIPHERAL;
#else
            // Cannot create a connectable advertising if peripheral role is not supported
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
#endif //(BLE_PERIPHERAL)
        }

        // Check discovery mode
        if (p_param->adv_param.disc_mode >= GAPM_ADV_MODE_MAX)
        {
            break;
        }

        // General and limited discoverable modes cannot be used if peripheral role is not supported
        if ((p_param->adv_param.disc_mode != GAPM_ADV_MODE_NON_DISC)
                && (p_param->adv_param.disc_mode != GAPM_ADV_MODE_BEACON))
        {
            needed_role = GAP_ROLE_PERIPHERAL;
        }

        // Check if this operation is supported by current role.
        if (!GAPM_IS_ROLE_SUPPORTED(needed_role))
        {
            // role not supported
            error = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // Check advertising type
        if (p_param->adv_param.type > GAPM_ADV_TYPE_PERIODIC)
        {
            break;
        }

        // Checks for directed advertising
        if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_DIRECTED))
        {
            // Device must be non discoverable and non scannable if directed advertising is used
            if ((p_param->adv_param.disc_mode != GAPM_ADV_MODE_NON_DISC)
                    || (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCANNABLE)))
            {
                break;
            }
        }

        // Checks for legacy advertising
        if (p_param->adv_param.type == GAPM_ADV_TYPE_LEGACY)
        {
            // Set legacy bit in the properties
            p_param->adv_param.prop |= ADV_LEGACY;

            // Anonymous and Extended TX Power bits must be set to 0
            if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_ANONYMOUS) ||
                    GETB(p_param->adv_param.prop, GAPM_ADV_PROP_TX_PWR))
            {
                break;
            }

            if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE))
            {
                // Undirected connectable advertising must be scannable
                if (!GETB(p_param->adv_param.prop, GAPM_ADV_PROP_DIRECTED))
                {
                    if (!GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCANNABLE))
                    {
                        break;
                    }
                }
                // Directed connectable advertising must not be scannable
                else
                {
                    if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCANNABLE))
                    {
                        break;
                    }
                }
            }

            // Direct advertising must be connectable
            if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_DIRECTED) &&
                    !GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE))
            {
                break;
            }
        }
        else
        {
            // Make sure legacy mode won't be used
            p_param->adv_param.prop &= ~ADV_LEGACY;

            // High duty cycle directed advertising cannot be used
            if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_HDC))
            {
                break;
            }

            // Extended advertising
            if (p_param->adv_param.type == GAPM_ADV_TYPE_EXTENDED)
            {
                // The advertisement shall not be both connectable and scannable
                if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE)
                        && GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCANNABLE))
                {
                    break;
                }

                // Anonymous mode is only available for non connectable non scannable non discoverable advertising
                if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_ANONYMOUS))
                {
                    if ((p_param->adv_param.disc_mode == GAPM_ADV_MODE_GEN_DISC)
                            || (p_param->adv_param.disc_mode == GAPM_ADV_MODE_LIM_DISC)
                            || GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE)
                            || GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCANNABLE))
                    {
                        break;
                    }
                }
            }
            // Periodic advertising
            else
            {
                // Connectable, anonymous, scannable bit must be set to 0
                if (GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE) ||
                        GETB(p_param->adv_param.prop, GAPM_ADV_PROP_SCANNABLE) ||
                        GETB(p_param->adv_param.prop, GAPM_ADV_PROP_ANONYMOUS))
                {
                    break;
                }
            }
        }

        // Check filter policy
        if (p_param->adv_param.type < GAPM_ADV_TYPE_PERIODIC)
        {
            if (p_param->adv_param.filter_pol > ADV_ALLOW_SCAN_WLST_CON_WLST)
            {
                break;
            }

            // While a device is in limited/general discoverable mode the Host configures the Controller as follows:
            //  - The Host shall set the advertising filter policy to "process scan and
            //    connection requests from all devices".
            if ((p_param->adv_param.disc_mode == GAPM_ADV_MODE_GEN_DISC)
                    || (p_param->adv_param.disc_mode == GAPM_ADV_MODE_LIM_DISC))
            {
                if (p_param->adv_param.filter_pol != ADV_ALLOW_SCAN_ANY_CON_ANY)
                {
                    break;
                }
            }
        }

        // Check privacy if enabled
        if (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT)
        {
            error = GAP_ERR_PRIVACY_CFG_PB;

            // If advertising is connectable, non-resolvable private address cannot be used
            if ((GETB(p_param->adv_param.prop, GAPM_ADV_PROP_CONNECTABLE))
                    && (p_param->own_addr_type == GAPM_GEN_NON_RSLV_ADDR))
            {
                break;
            }
        }

        error = GAP_ERR_NO_ERROR;
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Check if activity current state and type allow to set data.
 *
 * @param[in] p_actv_adv    Pointer to the advertising activity structure
 * @param[in] p_param       GAPM_SET_ADV_DATA_CMD parameters
 *
 * @return true if advertising data can be set, else false.
 ****************************************************************************************
 */
__STATIC bool gapm_adv_can_set_data(struct gapm_actv_adv_tag *p_actv_adv,
                                    struct gapm_set_adv_data_cmd *p_param)
{
    // Returned status
    bool status = false;

    do
    {
        // Check that activity is well an advertising activity
        if (p_actv_adv->common.type != GAPM_ACTV_TYPE_ADV)
        {
            break;
        }

        // Data can be set only if activity state is CREATED or STARTED
        if (!((p_actv_adv->common.state == GAPM_ACTV_CREATED)
                || (p_actv_adv->common.state == GAPM_ACTV_STARTED)))
        {
            break;
        }

        if (p_param->operation == GAPM_SET_ADV_DATA)
        {
            // Check if advertising data can be set
            if (!GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_ADV_DATA))
            {
                break;
            }
        }
        else if (p_param->operation == GAPM_SET_SCAN_RSP_DATA)
        {
            // Check if scan response data can be set
            if (!GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA))
            {
                break;
            }
        }
        else if (p_param->operation == GAPM_SET_PERIOD_ADV_DATA)
        {
            // Periodic advertising data can only be set for a periodic advertising
            if (p_actv_adv->common.subtype != GAPM_ADV_TYPE_PERIODIC)
            {
                break;
            }
        }

        status = true;
    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Verify if advertising data type must be unique
 *
 * @param[in] adv_type  Type of advertising data
 *
 * @return True if unique, False else
 ****************************************************************************************
 */
__STATIC bool gapm_adv_is_advtype_unique(uint8_t type)
{
    // Advertising type check which shall be unique
    switch (type)
    {
    case GAP_AD_TYPE_MORE_16_BIT_UUID:
    case GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
    case GAP_AD_TYPE_MORE_32_BIT_UUID:
    case GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID:
    case GAP_AD_TYPE_MORE_128_BIT_UUID:
    case GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID:
    case GAP_AD_TYPE_SHORTENED_NAME:
    case GAP_AD_TYPE_COMPLETE_NAME:
    case GAP_AD_TYPE_APPEARANCE:
    case GAP_AD_TYPE_ADV_INTV:
    case GAP_AD_TYPE_PUB_TGT_ADDR:
    case GAP_AD_TYPE_RAND_TGT_ADDR:
    case GAP_AD_TYPE_LE_BT_ADDR:
    case GAP_AD_TYPE_LE_ROLE:
    case GAP_AD_TYPE_FLAGS:
    {
        return true;
    }

    default:
    {
        return false;
    }
    }
}

/**
 ****************************************************************************************
 * @brief Check content of advertising or scan response data provided by application.
 *
 * @param[in] length        Length of provided data
 * @param[in] p_data        Pointer to the advertising or scan response data
 *
 * @return true if provided advertising data is well formatted, else false
 ****************************************************************************************
 */
__STATIC bool gapm_adv_check_data_sanity(uint16_t length, uint8_t *p_data)
{
    // Returned status
    bool status = true;
    // Cursor
    uint8_t *p_cursor = p_data;
    // End of data
    uint8_t *p_end_cursor = p_data + length;
    // Check for duplicate information in advertising or scan response data.
    uint8_t dup_filter[GAP_AD_TYPE_BITFIELD_BYTES];

    // Clear presence status of unique advertising type
    memset(&dup_filter[0], 0, GAP_AD_TYPE_BITFIELD_BYTES);

    // AD type flags must not be set by application
    // Sometimes need to set flags by application user.
    // GAP_AD_TYPE_SET_BIT(dup_filter, GAP_AD_TYPE_FLAGS);

    while (p_cursor < p_end_cursor)
    {
        // Extract AD type
        uint8_t ad_type = *(p_cursor + 1);

        // Check if it's AD Type which shall be unique
        if (gapm_adv_is_advtype_unique(ad_type))
        {
            if (!GAP_AD_TYPE_CHECK_BIT(dup_filter, ad_type))
            {
                // Mark the advertising type as found
                GAP_AD_TYPE_SET_BIT(dup_filter, ad_type);
            }
            else
            {
                // Advertising type has been found twice
                break;
            }
        }

        /* Go to next advertising info */
        p_cursor += (*p_cursor + 1);
    }

    // Check if total advertising length is valid with advertising data info
    if (p_cursor != p_end_cursor)
    {
        status = false;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Check if advertising data parameters are valid
 *
 * @param[in] p_actv_adv    Pointer to the advertising activity structure
 * @param[in] p_param       GAPM_SET_ADV_DATA_CMD parameters
 *
 * @return GAP_ERR_NO_ERROR if data and data parameters are valid, GAP_ERR_INVALID_PARAM
 * if a data parameter is not valid, GAP_ERR_ADV_DATA_INVALID if data is not well formatted
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_check_data_param(struct gapm_actv_adv_tag *p_actv_adv,
        struct gapm_set_adv_data_cmd *p_param)
{
    // Returned status
    uint8_t status = GAP_ERR_INVALID_PARAM;
    // Length to be provided
    uint16_t length = p_param->length;

    do
    {
        // No presence of AD type flags for beacons
        if (p_actv_adv->mode != GAPM_ADV_MODE_BEACON)
        {
            // AD type flags are added to any advertising data or to scan response data
            // if advertising data cannot be set, we have to take length of this field in account
            if ((p_param->operation == GAPM_SET_ADV_DATA) ||
                    ((p_param->operation == GAPM_SET_SCAN_RSP_DATA)
                     && !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_ADV_DATA)))
            {
                length += GAPM_ADV_AD_TYPE_FLAGS_LENGTH;
            }
        }

        // Check if data length is not too long for controller
        if (length > gapm_env.max_adv_data_len)
        {
            break;
        }

        // If advertising is a legacy advertising, length of advertising data cannot be greater
        // than 31 bytes
        if (p_actv_adv->common.subtype == GAPM_ADV_TYPE_LEGACY)
        {
            if (length > GAP_ADV_DATA_LEN)
            {
                break;
            }
        }

        // Perform a advertising/scan response data sanity check
        if ((p_param->operation == GAPM_SET_ADV_DATA) || (p_param->operation == GAPM_SET_SCAN_RSP_DATA))
        {
            if (!gapm_adv_check_data_sanity(p_param->length, &p_param->data[0]))
            {
                status = GAP_ERR_ADV_DATA_INVALID;
                break;
            }
        }

        status = GAP_ERR_NO_ERROR;
    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is CREATING:
 *    - LE Set Extended Advertising Parameters command (HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE)
 *        -> For all kind of advertising
 *    - LE Set Periodic Advertising Parameters command (HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE)
 *        -> For periodic advertising only
 *    - LE Remove Advertising Set command (HCI_LE_RMV_ADV_SET_CMD_OPCODE)
 *        -> For periodic advertising only if LE Set Periodic Advertising Parameters command
 *           complete event is returned with an error status
 *
 * @param[in|out] p_actv_adv     Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_creating_handler(struct gapm_actv_adv_tag *p_actv_adv,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    do
    {
        if (opcode == HCI_LE_RMV_ADV_SET_CMD_OPCODE)
        {
            // Advertising set has been removed, previous error status can be returned
            break;
        }

        p_actv_adv->kept_status = RW_ERR_HCI_TO_HL(p_event->status);

        if (p_actv_adv->kept_status == GAP_ERR_NO_ERROR)
        {
            if (opcode == HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE)
            {
                // Keep TX power
                p_actv_adv->tx_pwr = ((struct hci_le_set_ext_adv_param_cmd_cmp_evt const *)p_event)->sel_tx_pwr;
#ifndef GAPM_SLIM
                if (p_actv_adv->common.subtype == GAPM_ADV_TYPE_PERIODIC)
                {
                    // Send a LE Set Periodic Advertising Parameters command to the controller
                    gapm_adv_send_hci_le_set_per_adv_param_cmd(p_actv_adv);
                }
#endif // !GAPM_SLIM
            }
        }
        else
        {
#ifndef GAPM_SLIM
            if (opcode == HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE)
            {
                // Send a LE Remove Advertising Set Command
                gapm_adv_send_hci_le_rmv_adv_set_cmd(p_actv_adv);
            }
#endif //GAPM_SLIM
        }
    }
    while (0);

    // Check if another HCI command complete event message is expected
    if (p_actv_adv->common.next_exp_opcode == HCI_NO_OPERATION_CMD_OPCODE)
    {
        if (p_actv_adv->kept_status == GAP_ERR_NO_ERROR)
        {
            gapm_env.nb_adv_actv++;
        }

        // Operation is over
        gapm_actv_created((struct gapm_actv_tag *)p_actv_adv, p_actv_adv->kept_status);
    }
}


/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STARTING:
 *    - LE Set Extended Advertising Enable command (HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE)
 *        -> For all kind of advertising
 *    - LE Set Periodic Advertising Enable command (HCI_LE_SET_PER_ADV_EN_CMD_OPCODE)
 *        -> For periodic advertising only
 *
 * @param[in, out] p_actv_adv     Pointer to the activity structure for which the event has
 *                               been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_starting_handler(struct gapm_actv_adv_tag *p_actv_adv,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    do
    {
        // Check if an error has been raised during the operation (request to disable advertising has been sent)
        if ((opcode == HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE) && (p_actv_adv->kept_status != GAP_ERR_NO_ERROR))
        {
            // Advertising has been stopped
            SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN, false);
            break;
        }

        // Map the controller error status on a GAP error status
        p_actv_adv->kept_status = RW_ERR_HCI_TO_HL(p_event->status);

        if (p_actv_adv->kept_status == GAP_ERR_NO_ERROR)
        {
            switch (opcode)
            {
            case (HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE):
            {
                // Keep in mind that advertising is running
                SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN, true);
#ifndef GAPM_SLIM
                if ((p_actv_adv->common.subtype == GAPM_ADV_TYPE_PERIODIC)
                        && !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_PER_ADV_EN))
                {
                    // Send a LE Set Periodic Advertising Enable command to the controller
                    gapm_adv_send_hci_le_set_per_adv_en_cmd(p_actv_adv, true);
                }
#endif // !GAPM_SLIM
            }
            break;
            case (HCI_LE_SET_ADV_EN_CMD_OPCODE):
            {
                // Keep in mind that advertising is running
                SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN, true);

                if ((p_actv_adv->common.subtype == GAPM_ADV_TYPE_PERIODIC)
                        && !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_PER_ADV_EN))
                {
                    // Send a LE Set Periodic Advertising Enable command to the controller
                    //TODO:
                    //gapm_adv_send_hci_le_set_per_adv_en_cmd(p_actv_adv, true);
                }
            } break;

            case (HCI_LE_SET_PER_ADV_EN_CMD_OPCODE):
            {
                // Keep in mind that periodic advertising is running
                SETB(p_actv_adv->common.info, GAPM_ADV_INFO_PER_ADV_EN, true);
            }
            break;

            default:
            {
                // Nothing
            } break;
            }
        }
        else
        {
            if (opcode == HCI_LE_SET_PER_ADV_EN_CMD_OPCODE)
            {
                // Send a LE Set Extended Advertising Enable (Disable) to the controller
#ifndef RWIP_BLE_4
                gapm_adv_send_hci_le_set_ext_adv_en_cmd(p_actv_adv, NULL);
#else
                gapm_adv_send_hci_le_set_adv_en_cmd(p_actv_adv, NULL);
#endif
            }
            // else advertising activity has not been started, hence operation can be directly stopped
        }
    }
    while (0);

    // Check if another HCI command complete event message is expected
    if (p_actv_adv->common.next_exp_opcode == HCI_NO_OPERATION_CMD_OPCODE)
    {
        // Operation is over
        gapm_actv_started((struct gapm_actv_tag *)p_actv_adv, p_actv_adv->kept_status);
    }
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STOPPING:
 *    - LE Set Extended Advertising Enable command (HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE)
 *        -> For all kind of advertising
 *    - LE Set Periodic Advertising Enable command (HCI_LE_SET_PER_ADV_EN_CMD_OPCODE)
 *        -> For periodic advertising only
 *
 * @param[in|out] p_actv_adv     Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_stopping_handler(struct gapm_actv_adv_tag *p_actv_adv,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    if (opcode == HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE)
    {
        // Advertising has been stopped
        SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN, false);

#ifndef GAPM_SLIM
        if (p_actv_adv->common.subtype == GAPM_ADV_TYPE_PERIODIC)
        {
            // Send a LE Set Periodic Advertising Enable command to the controller
            gapm_adv_send_hci_le_set_per_adv_en_cmd(p_actv_adv, false);
        }
#endif // !GAPM_SLIM
    }
    else
    {
        // Periodic advertising has been stopped
        SETB(p_actv_adv->common.info, GAPM_ADV_INFO_PER_ADV_EN, false);
    }

    // Check if another HCI command complete event message is expected
    if (p_actv_adv->common.next_exp_opcode == HCI_NO_OPERATION_CMD_OPCODE)
    {
        // Operation is over
        gapm_actv_stopped((struct gapm_actv_tag *)p_actv_adv, GAP_ERR_NO_ERROR);
    }
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is DELETING:
 *    - LE Remove Advertising Set command (HCI_LE_RMV_ADV_SET_CMD_OPCODE)
 *        -> For all kind of advertising
 *
 * @param[in|out] p_actv_adv     Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_deleting_handler(struct gapm_actv_adv_tag *p_actv_adv,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    gapm_env.nb_adv_actv--;

    // Operation is over
    gapm_actv_deleted((struct gapm_actv_tag *)p_actv_adv);
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STARTING:
 *    - LE Set Extended Advertising Enable command (HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE)
 *        -> For all kind of advertising
 *    - LE Set Periodic Advertising Enable command (HCI_LE_SET_PER_ADV_EN_CMD_OPCODE)
 *        -> For periodic advertising only
 *
 * @param[in|out] p_actv_adv     Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_set_data_handler(struct gapm_actv_adv_tag *p_actv_adv,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Map controller error code on GAP error code
    uint8_t error = RW_ERR_HCI_TO_HL(p_event->status);
    // Retrieve GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_param
        = (struct gapm_set_adv_data_cmd *)gapm_get_operation_ptr(GAPM_OP_AIR);

    // Check if an error has been raised or if the whole data has been transfered
    if ((error != GAP_ERR_NO_ERROR) || !gapm_adv_set_data(p_actv_adv, p_param, true))
    {
        // Operation is over
        gapm_send_complete_evt(GAPM_OP_AIR, error);
    }
}

/**
 ****************************************************************************************
 * @brief Start an advertising activity.
 *
 * @param[in] p_actv        Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_START_CMD message parameters
 ****************************************************************************************
 */
__STATIC uint8_t gapm_adv_start(struct gapm_actv_tag *p_actv, struct gapm_activity_start_cmd *p_param)
{
    // Cast the activity structure in order to retrieve advertising parameters
    struct gapm_actv_adv_tag *p_actv_adv = (struct gapm_actv_adv_tag *)p_actv;
    // Check new provided advertising parameters
    uint8_t error = gapm_adv_check(p_actv_adv);

    do
    {
        // Check if advertising data has been set
        if (error != GAP_ERR_NO_ERROR)
        {
            break;
        }

        // Clear kept status
        p_actv_adv->kept_status = GAP_ERR_NO_ERROR;
        // Keep activity identifier in mind
        gapm_env.actv_idx = p_actv_adv->common.idx;

        if (p_actv_adv->common.state == GAPM_ACTV_STARTED)
        {
            // Verify that advertising is not running
            if (GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN))
            {
                error = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }
        }

        // Start advertising
#ifndef RWIP_BLE_4
        gapm_adv_send_hci_le_set_ext_adv_en_cmd(p_actv_adv, p_param);
#else
        gapm_adv_send_hci_le_set_adv_en_cmd(p_actv_adv, p_param);

#endif
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Stop one advertising activity. Activity state must have been set to STOPPING state
 * before calling this function
 *
 * @param[in] p_actv    Pointer to the activity structure to be stopped
 ****************************************************************************************
 */
__STATIC void gapm_adv_stop(struct gapm_actv_tag *p_actv)
{
    // Cast the activity structure in order to retrieve advertising parameters
    struct gapm_actv_adv_tag *p_actv_adv = (struct gapm_actv_adv_tag *)p_actv;

    // Clear kept status
    p_actv_adv->kept_status = GAP_ERR_NO_ERROR;
    // Keep activity identifier in mind
    gapm_env.actv_idx = p_actv_adv->common.idx;

    if (GETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN))
    {
        // Send a LE Set Extended Advertising Enable (Disable) command to the controller
#ifndef RWIP_BLE_4
        gapm_adv_send_hci_le_set_ext_adv_en_cmd(p_actv_adv, NULL);
#else
        gapm_adv_send_hci_le_set_adv_en_cmd(p_actv_adv, NULL);

#endif

    }
    else
    {
#ifndef GAPM_SLIM
        // Send a LE Set Periodic Advertising Enable (Disable command to the controller
        gapm_adv_send_hci_le_set_per_adv_en_cmd(p_actv_adv, false);
#endif // !GAPM_SLIM
    }
}

/**
 ****************************************************************************************
 * @brief Remove one advertising activity. It is considered here that advertising
 * activity is in DELETING state.
 *
 * @param[in] p_actv    Pointer to the activity structure to be deleted.
 ****************************************************************************************
 */
__STATIC void gapm_adv_delete(struct gapm_actv_tag *p_actv)
{
    // Cast the activity structure in order to retrieve advertising parameters
    struct gapm_actv_adv_tag *p_actv_adv = (struct gapm_actv_adv_tag *)p_actv;

    // Clear kept status
    p_actv_adv->kept_status = GAP_ERR_NO_ERROR;
    // Keep activity identifier in mind
    gapm_env.actv_idx = p_actv_adv->common.idx;

    // Send LE Remove Advertising Set command
    gapm_adv_send_hci_le_rmv_adv_set_cmd(p_actv_adv);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint8_t gapm_adv_create(uint8_t actv_idx, struct gapm_activity_create_adv_cmd *p_param)
{
    uint8_t error;
    // Allocated advertising activity
    struct gapm_actv_adv_tag *p_actv_adv;

    do
    {
        // Check if a new advertising set can be used in controller
        if (gapm_env.nb_adv_actv == gapm_env.max_adv_set)
        {
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check provided advertising parameters
        error = gapm_adv_check_param(p_param);

        if (error != GAP_ERR_NO_ERROR)
        {
            break;
        }

        // Allocate an activity structure
        p_actv_adv = (struct gapm_actv_adv_tag *)gapm_actv_alloc(actv_idx, sizeof(struct gapm_actv_adv_tag));

        // Check if enough memory has been found
        if (!p_actv_adv)
        {
            error = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // Fill the activity structure
        p_actv_adv->common.type          = GAPM_ACTV_TYPE_ADV;
        p_actv_adv->common.subtype       = p_param->adv_param.type;
        p_actv_adv->common.own_addr_type = p_param->own_addr_type;
        p_actv_adv->common.requester     = gapm_get_requester(GAPM_OP_AIR);
        p_actv_adv->kept_status          = GAP_ERR_NO_ERROR;
        p_actv_adv->mode                 = p_param->adv_param.disc_mode;

        // Set callback functions
        p_actv_adv->common.cb_actv_start  = gapm_adv_start;
        p_actv_adv->common.cb_actv_stop   = gapm_adv_stop;
        p_actv_adv->common.cb_actv_delete = gapm_adv_delete;

        // Keep activity identifier in mind
        gapm_env.actv_idx = actv_idx;

        // Keep in mind if advertising data is expected
        //     -> Not expected if legacy directed advertising or scannable extended advertising
        if (!((p_actv_adv->common.subtype == GAPM_ADV_TYPE_LEGACY) && (p_param->adv_param.prop & ADV_DIRECT))
                && !((p_actv_adv->common.subtype != GAPM_ADV_TYPE_LEGACY) && (p_param->adv_param.prop & ADV_SCAN)))
        {
            SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_ADV_DATA, true);
        }

        // Keep in mind if advertising is scan response data is expected
        if (p_param->adv_param.prop & ADV_SCAN)
        {
            SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ACPT_SCAN_RSP_DATA, true);
        }


#ifndef RWIP_BLE_4
        // Send a LE Set Extended Advertising Parameters command to the controller
        gapm_adv_send_hci_le_set_ext_adv_param_cmd(p_actv_adv, p_param);
#else
        gapm_adv_send_hci_le_set_param_cmd(p_actv_adv, p_param);
#endif

    }
    while (0);

    return (error);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles request of setting advertising data:
 *  - GAPM_SET_ADV_DATA: Set advertising data
 *  - GAPM_SET_SCAN_RSP_DATA: Set scan response data
 *  - GAPM_SET_PERIOD_ADV_DATA: Set periodic advertising data
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_set_adv_data_cmd_handler(ke_msg_id_t const msgid,
                                  struct gapm_set_adv_data_cmd *p_param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_ADV_DATA,
                                      GAPM_SET_SCAN_RSP_DATA,
                                      GAPM_SET_PERIOD_ADV_DATA,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Error code
        uint8_t error;
        // Get indicated activity
        struct gapm_actv_adv_tag *p_actv_adv;

        do
        {
            // Check provided activity index
            if (p_param->actv_idx >= GAPM_ACTV_NB)
            {
                error = GAP_ERR_INVALID_PARAM;
                break;
            }

            // Get indicated activity
            p_actv_adv = (struct gapm_actv_adv_tag *)gapm_env.actvs[p_param->actv_idx];

            // Check if activity well exists and if data can be downloaded
            if (!p_actv_adv || !gapm_adv_can_set_data(p_actv_adv, p_param))
            {
                error = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }

            error = gapm_adv_check_data_param(p_actv_adv, p_param);

            // Check if parameters are valid
            if (error != GAP_ERR_NO_ERROR)
            {
                break;
            }

            // Initialize data offset
            p_actv_adv->data_offset = 0;
            // Keep advertising identifier in the environment
            gapm_env.actv_idx = p_param->actv_idx;

            // Send the provided data to the controller
            gapm_adv_set_data(p_actv_adv, p_param, false);
        }
        while (0);

        // If an error has been raised, inform the application
        if (error != GAP_ERR_NO_ERROR)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_AIR, error);
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Common handler for following HCI command complete events:
 *  - LE Set Advertising Set Random Address Command (HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE)
 *  - LE Set Extended Advertising Parameters Command (HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE)
 *  - LE Set Extended Advertising Data Command (HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE)
 *  - LE Set Extended Scan Response Data Command (HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE)
 *  - LE Set Extended Advertising Enable Command (HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE)
 *  - LE Set Periodic Advertising Parameters Command (HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE)
 *  - LE Set Periodic Advertising Data Command (HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE)
 *  - LE Set Periodic Advertising Enable Command (HCI_LE_SET_PER_ADV_EN_CMD_OPCODE)
 *  - LE Remove Advertising Set Command (HCI_LE_RMV_ADV_SET_CMD_OPCODE)
 *  - LE Clear Advertising Sets Command (HCI_LE_CLEAR_ADV_SETS_CMD_OPCODE)
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_cmd_cmp_evt_adv_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Advertising activity structure
    struct gapm_actv_adv_tag *p_actv_adv;

    do
    {
        // Find activity for which the command complete event has been received
        if (!gapm_actv_retrieve_cmd_cmp_evt((struct gapm_actv_tag **)&p_actv_adv, opcode))
        {
            break;
        }

        // Call the proper handler based on current activity state
        switch (p_actv_adv->common.state)
        {
        case GAPM_ACTV_CREATING:
        {
            hci_le_cmd_cmp_evt_creating_handler(p_actv_adv, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STARTED:
        case GAPM_ACTV_CREATED:
        {
            hci_le_cmd_cmp_evt_set_data_handler(p_actv_adv, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STARTING:
        {
            hci_le_cmd_cmp_evt_starting_handler(p_actv_adv, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STOPPING:
        {
            hci_le_cmd_cmp_evt_stopping_handler(p_actv_adv, opcode, p_event);
        }
        break;

        case GAPM_ACTV_DELETING:
        {
            hci_le_cmd_cmp_evt_deleting_handler(p_actv_adv, opcode, p_event);
        }
        break;

        default:
        {
            // Should not happen
            ASSERT_ERR(0);
        }
        break;
        }
    }
    while (0);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Scan Request Received event.
 * This event indicates that a SCAN_REQ PDU or an AUX_SCAN_REQ PDU has been received.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_scan_req_rcvd_evt_handler(uint16_t opcode, struct hci_le_scan_req_rcvd_evt const *p_event)
{
    // Advertising activity structure
    struct gapm_actv_adv_tag *p_actv_adv;

    do
    {
        // Check provided advertising handle
        if (p_event->adv_hdl >= GAPM_ACTV_NB)
        {
            // Drop the event
            break;
        }

        // Retrieve the indicated activity
        p_actv_adv = (struct gapm_actv_adv_tag *)gapm_env.actvs[p_event->adv_hdl];

        // Check if event can be expected
        if (!p_actv_adv
                || (p_actv_adv->common.type != GAPM_ACTV_TYPE_ADV)
                || (p_actv_adv->common.state != GAPM_ACTV_STARTED)
                || !GETB(p_actv_adv->common.info, GAPM_ADV_INFO_SCAN_REQ_NTF))
        {
            // Drop the event
            break;
        }

        // Send a message to the application
        gapm_adv_send_scan_request_ind(p_actv_adv, p_event);
    }
    while (0);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Advertising Set Terminated Event.
 * This event indicates that the controller has terminated advertising in the advertising
 * set specified by the advertising handle parameter.
 * The event is generated in following cases:
 *    - Connection has been created (in that case connection handle of new connection is
 *    provided)
 *    - Advertising duration elapsed (status parameter code set to Advertising Timeout (0x3C)
 *    - Max extended advertising events value has been reached (status parameter set to
 *    Limit Reached (0x43)
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_adv_set_term_evt_handler(uint16_t opcode, struct hci_le_adv_set_term_evt const *p_event)
{
    // Advertising activity structure
    struct gapm_actv_adv_tag *p_actv_adv;
    // Reason
    uint8_t reason = GAP_ERR_NO_ERROR;

    do
    {
        // Check provided advertising handle
        if (p_event->adv_hdl >= GAPM_ACTV_NB)
        {
            // Drop the event
            break;
        }

        // Retrieve the indicated activity
        p_actv_adv = (struct gapm_actv_adv_tag *)gapm_env.actvs[p_event->adv_hdl];

        // Check if event can be expected
        if (!p_actv_adv
                || (p_actv_adv->common.type != GAPM_ACTV_TYPE_ADV)
                || (p_actv_adv->common.state != GAPM_ACTV_STARTED))
        {
            // Drop the event
            break;
        }

        // Keep in mind that advertising is over
        SETB(p_actv_adv->common.info, GAPM_ADV_INFO_ADV_EN, false);

        switch (p_event->status)
        {
#if (BLE_CENTRAL || BLE_PERIPHERAL)
        case CO_ERROR_NO_ERROR:
        {
            // If resolvable private address or non-resolvable private address has been used for advertising,
            // and controller privacy was not enabled, we need to keep the address generated by the host as
            // a local address for pairing.
            if ((p_actv_adv->common.own_addr_type != GAPM_STATIC_ADDR)
                    && (!(gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT)))
            {
                // Convert connection handle into connection index
                uint8_t conidx = gapc_get_conidx(p_event->conhdl);

                // Check that connection exists
                if (conidx != GAP_INVALID_CONIDX)
                {
                    gapc_set_local_addr(conidx, &p_actv_adv->common.addr.addr[0]);
                }
            }
        }
        break;
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

        case CO_ERROR_ADV_TO:
        case CO_ERROR_LIMIT_REACHED:
        {
            reason = GAP_ERR_TIMEOUT;
        }
        break;

        default:
        {
            // Nothing more to be done
        }
        break;
        }

        // Inform the application that activity has been stopped
        gapm_actv_stopped((struct gapm_actv_tag *)p_actv_adv, reason);
    }
    while (0);

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_BROADCASTER)

/// @} GAPM_ADV
