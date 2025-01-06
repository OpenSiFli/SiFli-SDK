/**
 ****************************************************************************************
 *
 * @file hci_fc.c
 *
 * @brief HCI Flow Control module source file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#if (HCI_PRESENT)

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_list.h"         // list definition

#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_event.h"        // kernel event definition
#include "ke_mem.h"          // kernel memory definition
#include "ke_timer.h"        // kernel timer definition


/*
 * DEFINES
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */
/// Flow control structure
struct host_set_fc
{
    /// flow control enabled
    bool acl_flow_cntl_en;
    /// host packet number max
    uint16_t acl_pkt_nb;
    /// current packet available
    uint16_t curr_pkt_nb;

#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
    /// flow control enabled
    bool sync_flow_cntl_en;
    /// host packet number max
    uint16_t sync_pkt_nb;
#endif // VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)
};

struct counter_fc
{
    /// counter for number of ACL packets sent to Host
    uint16_t acl_pkt_sent;
#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
    /// counter for number of SYNC packets sent to Host
    uint16_t sync_pkt_sent;
#endif // VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)
};

struct hci_fc_tag
{
    /// Flow Control
    struct host_set_fc host_set;
    struct counter_fc cntr;
};


/*
 * CONSTANTS DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI FC environment context
__STATIC struct hci_fc_tag hci_fc_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */



/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_fc_init(void)
{
    memset(&hci_fc_env, 0, sizeof(hci_fc_env));
}

uint8_t hci_fc_acl_buf_size_set(uint16_t acl_pkt_len, uint16_t nb_acl_pkts)
{
    uint8_t status = CO_ERROR_UNSUPPORTED;

    /*
     * Downlink fragmentation is not supported, the packet length must be greater than the maximum data size that
     * the device can receive (largest air packet)
     */
#if BT_EMB_PRESENT
    if (acl_pkt_len >= DH5_3_PACKET_SIZE)
#elif BLE_EMB_PRESENT
    if (acl_pkt_len >= LE_MAX_OCTETS)
#endif // BT_EMB_PRESENT / BLE_EMB_PRESENT
    {
        status = CO_ERROR_NO_ERROR;

        // Store only the number of buffers (packet length is not saved as downlink fragmentation is not supported)
        hci_fc_env.host_set.acl_pkt_nb  = nb_acl_pkts;
    }

    return (status);
}

#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
uint8_t hci_fc_sync_buf_size_set(uint8_t sync_pkt_len, uint16_t nb_sync_pkts)
{
    // Store only the number of buffers (packet length is not saved as downlink fragmentation is not supported)
    hci_fc_env.host_set.sync_pkt_nb  = nb_sync_pkts;

    return (CO_ERROR_NO_ERROR);
}
#endif // VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)

uint8_t hci_fc_acl_en(bool flow_enable)
{
    uint8_t status = CO_ERROR_NO_ERROR;

#if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
    // Check that there is no BLE link
    for (uint8_t conhdl = 0 ; conhdl < BLE_ACTIVITY_MAX ; conhdl++)
    {
        if (hci_env.ble_con_state[BLE_CONHDL_TO_LINKID(conhdl)])
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
    }
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    // Check that there is no BLE link
    for (uint8_t conhdl = BT_ACL_CONHDL_MIN ; conhdl <= BT_ACL_CONHDL_MAX ; conhdl++)
    {
        if (hci_env.bt_acl_con_tab[(conhdl - BT_ACL_CONHDL_MIN)].state != HCI_BT_ACL_STATUS_NOT_ACTIVE)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
    }
#endif //BT_EMB_PRESENT

    if (status == CO_ERROR_NO_ERROR)
    {
        hci_fc_env.host_set.acl_flow_cntl_en = flow_enable;
    }

    return status;
}

#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
void hci_fc_sync_en(bool flow_enable)
{
    hci_fc_env.host_set.sync_flow_cntl_en = flow_enable;
}
#endif //VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)

void hci_fc_acl_packet_sent(void)
{
    if (hci_fc_env.host_set.acl_flow_cntl_en == true)
    {
        hci_fc_env.cntr.acl_pkt_sent++;
    }
}

#if BT_EMB_PRESENT
#if VOICE_OVER_HCI
void hci_fc_sync_packet_sent(void)
{
    if (hci_fc_env.host_set.sync_flow_cntl_en == true)
    {
        hci_fc_env.cntr.sync_pkt_sent++;
    }
}
#endif //VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)

void hci_fc_host_nb_acl_pkts_complete(uint16_t acl_pkt_nb)
{
    if (hci_fc_env.cntr.acl_pkt_sent > acl_pkt_nb)
    {
        hci_fc_env.cntr.acl_pkt_sent -= acl_pkt_nb;
    }
    else
    {
        hci_fc_env.cntr.acl_pkt_sent = 0;
    }
}

#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
void hci_fc_host_nb_sync_pkts_complete(uint16_t sync_pkt_nb)
{
    if (hci_fc_env.cntr.sync_pkt_sent > sync_pkt_nb)
    {
        hci_fc_env.cntr.sync_pkt_sent -= sync_pkt_nb;
    }
    else
    {
        hci_fc_env.cntr.sync_pkt_sent = 0;
    }
}
#endif //VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)

uint16_t hci_fc_check_host_available_nb_acl_packets(void)
{
    uint16_t cnt = 0;
    // if flow control is not enabled we can send number of packets
    if (hci_fc_env.host_set.acl_flow_cntl_en != true)
    {
        cnt = 0xFFFF;// maximum packets
    }
    else if (hci_fc_env.host_set.acl_pkt_nb > hci_fc_env.cntr.acl_pkt_sent)
    {
        cnt = hci_fc_env.host_set.acl_pkt_nb - hci_fc_env.cntr.acl_pkt_sent;
    }
    return cnt;
}

#if (BT_EMB_PRESENT)
#if (VOICE_OVER_HCI)
uint16_t hci_fc_check_host_available_nb_sync_packets(void)
{
    uint16_t cnt = 0;
    // if flow control is not enabled we can send number of packets
    if (hci_fc_env.host_set.sync_flow_cntl_en != true)
    {
        cnt = 0xFFFF;// maximum packets
    }
    else if (hci_fc_env.host_set.sync_pkt_nb > hci_fc_env.cntr.sync_pkt_sent)
    {
        cnt = hci_fc_env.host_set.sync_pkt_nb - hci_fc_env.cntr.sync_pkt_sent;
    }
    return cnt;
}
#endif //VOICE_OVER_HCI
#endif // (BT_EMB_PRESENT)

#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#endif //(HCI_PRESENT)

/// @} HCI
