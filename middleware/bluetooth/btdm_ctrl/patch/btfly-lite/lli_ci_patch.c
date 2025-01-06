/**
 ****************************************************************************************
 *
 * @file lli_cis.c
 *
 * @brief Definition of the functions used by the link layer Connected Isochronous Stream module
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // Stack configuration

#if (BLE_CIS)

#include <string.h>
#include "rwip.h"           // IP definitions
#include "lli_int.h"        // LLI internal definitions
#include "lli.h"            // LLI definitions
#include "llm.h"            // LLM definitions
#include "llc.h"            // For definitions of LLC kernel messages
#include "lld.h"            // LLD definitions
#include "hci.h"            // For HCI definitions
#include "co_utils.h"       // For GETF macro
#include "ble_util.h"       // BLE utility functions
#include "data_path.h"      // Isochronous channel Data path
#include "sch_plan.h"       // Schedule planning
#include "co_math.h"        // Mathematics API
#include "ke_mem.h"         // kernel memory

#if (BLE_ISOGEN)
    #include "isogen.h"         // ISOGEN definitions
#endif //(BLE_ISOGEN)
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Retrieve CIG environment
#define LLI_CIG_ENV_GET(grp_hdl, env_ptr) lli_group_env_get(grp_hdl, LLI_ISO_GROUP_CIG, (struct lli_group_env**) (env_ptr))


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// State of a CIS
enum lli_cis_state
{
    /// Stream exists
    CIS_EXISTS,
    /// Stream waits host status (peripheral only)
    CIS_WAIT_HOST,
    /// Stream is negotiating creation
    CIS_CREATE_NEGO,
    /// Stream is waiting establishment
    CIS_WAIT_EST,
    /// Stream is established
    CIS_EST,
    /// Stream is Stopping
    CIS_STOPPING,
};


/*
 * TYPE DEFINITION
 ****************************************************************************************
 */

/// CIS environment structure
struct lli_cis_env
{
    /// List header
    co_list_hdr_t                hdr;
    /// CIS parameters used during negotiation
    struct lli_cis_param         cis_param;

    /// RX and TX Isochronous Data path
    const struct data_path_itf  *p_dp[ISO_SEL_MAX];

    /// Connection Link identifier
    uint8_t                      link_id;
    /// Handle of Group (unique identifier in controller)
    uint8_t                      grp_hdl;
    /// Reject or disable reason
    uint8_t                      reason;
    /// Role
    uint8_t                      role;
    /// Activity identifier
    uint8_t                      act_id;
    /// CIS state (@see enum lli_cis_state)
    uint8_t                      state;
};

/// CIG environment structure
struct lli_cig_env
{
    /// ISO Stream information
    struct lli_group_env         group;
    /// list of CIS present in the group
    struct co_list               streams;

    /// CIG parameters used during negotiation
    struct lli_cig_param         cig_param;

    /// Sub-event Interval, valid only for interleave scheduling (in us)
    uint32_t                     sub_interval;
    /// Planning activity offset (in half-slots)
    uint16_t                     act_offset;
    /// Slave connection link identifier (invalid if master role)
    uint8_t                      link_id;
    /// Number of CIS
    uint8_t                      nb_cis;
    /// CIG identifier
    uint8_t                      cig_id;
    /// Role of the CIG
    uint8_t                      role;
    /// Group packing method (@see enum iso_packing)
    uint8_t                      packing;
#if (BLE_CENTRAL)
    // Indicates if the CIG is not configurable (true) or configurable (false)
    bool not_configurable;
#endif // (BLE_CENTRAL)
};

/// CI Environment structure to manage the CIS creation operation
struct lli_ci_env_tag
{
    /// CIS Creation command
    struct hci_le_create_cis_cmd const *p_create_cmd;

    /// Callback to execute when a CIS is started
    lli_cis_start_evt_cb                cb_start_evt;

    /// use to know which channel is under creation
    uint8_t cursor;
};

#if !(BLE_CIS_IN_ROM)

/*
 * LOCAL VARIABLE DEFINITION
 ****************************************************************************************
 */
/// CI Environment variable
struct lli_ci_env_tag lli_ci_env;

/// CIS environment structure
struct lli_cis_env *lli_cis_env[BLE_ACTIVITY_MAX];

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Return CIS activity ID
 *
 * @param[in] link_id  Link identifier
 * @param[in] cig_id   CIG identifier
 * @param[in] cis_id   CIS identifier
 ****************************************************************************************
 */
__STATIC uint8_t lli_cis_act_id_get(uint8_t link_id, uint8_t cig_id, uint8_t cis_id)
{
    uint8_t cnt;

    for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
    {
        struct lli_cis_env *p_cis_env = lli_cis_env[cnt];

        // Check if CIS exists and has the same identifiers
        if ((p_cis_env != NULL)
                // Only check link_id when peripheral
                && ((link_id == BLE_INVALID_LINK_ID) || (p_cis_env->link_id == link_id))
                && (p_cis_env->cis_param.cig_id == cig_id)
                && (p_cis_env->cis_param.cis_id == cis_id))
        {
            break;
        }
    }

    // Return the allocated CIS handle
    return (cnt);
}

#if (BLE_CENTRAL)
/// Convert PHY to a valid rate
__STATIC uint8_t lli_ci_get_rate(uint8_t phy)
{
    if (NB_ONE_BITS(phy) > 1)
    {
#if (BLE_PHY_CODED_SUPPORT)
        if (phy & PHY_CODED_BIT)
        {
            phy = PHY_CODED_BIT;
        }
#endif // (BLE_PHY_CODED_SUPPORT)
        if (phy & PHY_1MBPS_BIT)
        {
            phy = PHY_1MBPS_BIT;
        }
#if (BLE_PHY_2MBPS_SUPPORT)
        if (phy & PHY_2MBPS_BIT)
        {
            phy = PHY_2MBPS_BIT;
        }
#endif // (BLE_PHY_2MBPS_SUPPORT)
    }

    return (co_phy_to_rate[co_phy_mask_to_value[phy]]);
}
#endif // (BLE_CENTRAL)

/**
 ****************************************************************************************
 * Compute Sub-Event duration in us
 *
 * @param[in] tx_size     transmission size
 * @param[in] tx_rate     transmission rate
 * @param[in] rx_size     reception size
 * @param[in] rx_rate     reception rate
 * @param[in] mic_present True if mic is transmitted over the air, False else
 *
 * @return Sub-Event duration in us
 ****************************************************************************************
 */
__STATIC uint32_t lli_ci_se_dur_compute(uint8_t tx_size, uint8_t tx_rate, uint8_t rx_size, uint8_t rx_rate, bool mic_present)
{
    uint32_t sub_evt_dur;
    // Compute packet duration over the air to have the minimum sub_evt duration
    sub_evt_dur  = (tx_size > 0) // check if mic len must be added if TX packet len != 0
                   ? ble_util_pkt_dur_in_us(tx_size + (mic_present ? MIC_LEN : 0), tx_rate)
                   : ble_util_pkt_dur_in_us(0, tx_rate);
    sub_evt_dur += (rx_size > 0) // check if mic len must be added if RX packet len != 0
                   ? ble_util_pkt_dur_in_us(rx_size + (mic_present ? MIC_LEN : 0), rx_rate)
                   : ble_util_pkt_dur_in_us(0, rx_rate);
    sub_evt_dur += BLE_IFS_DUR + BLE_MSS_DUR;

    return (sub_evt_dur);
}

/**
 ****************************************************************************************
 * @brief Sends HCI LE CIS Established Event to the host
 *
 * @param[in] p_cig_env       Pointer to the environment structure for the CIG
 * @param[in] p_cis_env       Pointer to the environment structure for the CIS enabled
 * @param[in] status          Status to report
 * @param[in] rate_m2s        Master to slave rate
 * @param[in] rate_s2m        Slave to master rate
 ****************************************************************************************
 */
__STATIC void lli_cis_established_evt_send(struct lli_cig_env *p_cig_env, struct lli_cis_env *p_cis_env,
        uint8_t status, uint8_t rate_m2s, uint8_t rate_s2m)
{
    // Allocate HCI message
    struct hci_le_cis_established_evt *p_evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_cis_established_evt);

    // Fill the message
    p_evt->subcode        = HCI_LE_CIS_ESTABLISHED_EVT_SUBCODE;
    p_evt->conhdl         = BLE_ACTID_TO_CISHDL(p_cis_env->act_id);
    p_evt->status         = status;
    if (status == CO_ERROR_NO_ERROR)
    {
        struct lli_cis_param *p_cis_param = &(p_cis_env->cis_param);
        struct lli_cig_param *p_cig_param = &(p_cig_env->cig_param);

        p_evt->cig_sync_delay = p_cig_param->cig_sync_delay;
        p_evt->cis_sync_delay = p_cig_param->cig_sync_delay - p_cis_param->cis_offset_in_cig;
        p_evt->trans_latency_m2s = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->tx_trans_latency : p_cig_param->rx_trans_latency;
        p_evt->trans_latency_s2m = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->rx_trans_latency : p_cig_param->tx_trans_latency;
        p_evt->phy_m2s        = co_rate_to_phy[rate_m2s];
        p_evt->phy_s2m        = co_rate_to_phy[rate_s2m];
        p_evt->nse            = p_cis_param->nse;
        p_evt->bn_m2s         = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->tx_bn : p_cis_param->rx_bn;
        p_evt->bn_s2m         = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->rx_bn : p_cis_param->tx_bn;
        p_evt->ft_m2s         = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->tx_ft : p_cig_param->rx_ft;
        p_evt->ft_s2m         = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->rx_ft : p_cig_param->tx_ft;
        p_evt->max_pdu_m2s    = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->tx_max_pdu : p_cis_param->rx_max_pdu;
        p_evt->max_pdu_s2m    = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->rx_max_pdu : p_cis_param->tx_max_pdu;
        p_evt->iso_interval   = p_cig_param->iso_interval;
    }
    else
    {
        p_evt->cig_sync_delay = 0;
        p_evt->cis_sync_delay = 0;
        p_evt->trans_latency_m2s = 0;
        p_evt->trans_latency_s2m = 0;
        p_evt->phy_m2s        = 0;
        p_evt->phy_s2m        = 0;
        p_evt->nse            = 0;
        p_evt->bn_m2s         = 0;
        p_evt->bn_s2m         = 0;
        p_evt->ft_m2s         = 0;
        p_evt->ft_s2m         = 0;
        p_evt->max_pdu_m2s    = 0;
        p_evt->max_pdu_s2m    = 0;
        p_evt->iso_interval   = 0;
    }

    // Send the message
    hci_send_2_host(p_evt);
}

/**
 ****************************************************************************************
 * @brief Sends HCI CIS Disconnect Complete Event to the host
 *
 * @param[in] p_cis_env    Pointer to the environment structure for the disabled channel
 * @param[in] reason       disconnection reason
 ****************************************************************************************
 */
__STATIC void lli_cis_disc_cmp_evt_send(struct lli_cis_env *p_cis_env, uint8_t reason)
{
    uint16_t conhdl  = BLE_ACTID_TO_CISHDL(p_cis_env->act_id);
    struct hci_disc_cmp_evt *disc_cc = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_DISC_CMP_EVT_CODE, hci_disc_cmp_evt);

    disc_cc->status  = CO_ERROR_NO_ERROR;
    disc_cc->conhdl  = conhdl;
    disc_cc->reason  = reason;

    // send the message
    hci_send_2_host(disc_cc);
}

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Sends HCI LE CIS Req Event to the host
 *
 * @param[in] p_cis_env    Pointer to the environment structure for the channel
 ****************************************************************************************
 */
__STATIC void lli_cis_request_evt_send(struct lli_cis_env *p_cis_env)
{
    struct lli_cis_param *p_cis_param = &(p_cis_env->cis_param);
    // Allocate HCI message
    struct hci_le_cis_request_evt *p_evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_cis_request_evt);

    // Fill the message
    p_evt->subcode              = HCI_LE_CIS_REQUEST_EVT_SUBCODE;
    p_evt->acl_conhdl           = BLE_LINKID_TO_CONHDL(p_cis_env->link_id);
    p_evt->cis_conhdl           = BLE_ACTID_TO_CISHDL(p_cis_env->act_id);
    p_evt->cig_id               = p_cis_param->cig_id;
    p_evt->cis_id               = p_cis_param->cis_id;

    // Send the message
    hci_send_2_host(p_evt);
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Allocate environment structure for a new CIS.
 *
 * @param[out] pp_cis_env   Address of pointer containing address of allocated environment
 *
 * @return Status of the allocation (@see enum co_error)
 ****************************************************************************************
 */
__STATIC uint8_t lli_cis_create(struct lli_cis_env **pp_cis_env)
{
    // get available activity
    uint8_t status;

    do
    {
        struct lli_cis_env *p_cis_env;
        uint8_t  act_id = BLE_ACTIVITY_MAX;

        status = llm_activity_free_get(&act_id);

        // Verify that an activity identifier is available
        if (status != CO_ERROR_NO_ERROR)
            break;

        // Allocate environment memory
        p_cis_env = (struct lli_cis_env *) ke_malloc_system(sizeof(struct lli_cis_env), KE_MEM_ENV);

        // Verify that the memory is allocated
        if (p_cis_env == NULL)
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            break;
        }

        // Clear the memory content
        memset(p_cis_env, 0, sizeof(struct lli_cis_env));

        // Return address of environment
        *pp_cis_env = p_cis_env;

        // Reserve an activity
        llm_activity_cis_reserve(act_id);

        // Store link identifiers
        p_cis_env->link_id = BLE_INVALID_LINK_ID;
        p_cis_env->act_id  = act_id;

        // Initialize data path to disable
        p_cis_env->p_dp[ISO_SEL_RX] = data_path_itf_get(ISO_DP_DISABLE, ISO_SEL_RX);
        p_cis_env->p_dp[ISO_SEL_TX] = data_path_itf_get(ISO_DP_DISABLE, ISO_SEL_TX);

    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Cleanup a CIS.
 *
 * @param[in] p_cis_env   Pointer to the CIS envirnoment
 ****************************************************************************************
 */
__STATIC void lli_cis_cleanup(struct lli_cis_env *p_cis_env)
{
    // Check if data path must be removed
    if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_RX]))
    {
        lli_cis_data_path_remove(p_cis_env->act_id, ISO_SEL_RX);
    }
    if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_TX]))
    {
        lli_cis_data_path_remove(p_cis_env->act_id, ISO_SEL_TX);
    }

    // Free the associated activity
    llm_activity_free_set(p_cis_env->act_id);

    // Free the stream environment
    lli_cis_env[p_cis_env->act_id] = NULL;
    ke_free(p_cis_env);
}

/**
 * Get Group parameter information
 *
 * @param[in] cig_id     CIG identifier.
 * @param[in] role       Master of slave role
 * @param[in] link_id    Link identifier (meaningful only on slave side)
 *
 * @return Stream parameters information
 */
__STATIC struct lli_cig_env *lli_cig_get(uint8_t cig_id, uint8_t role, uint8_t link_id)
{
    struct lli_cig_env *p_env = NULL;

    // Counter
    uint8_t cnt;

    for (cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
    {
        // check if stream exist
        if ((LLI_CIG_ENV_GET(cnt, &p_env) == CO_ERROR_NO_ERROR)
                && (p_env->cig_id  == cig_id)
                && (p_env->role    == role)
                && (p_env->link_id == link_id))
        {
            break;
        }
    }

    if (cnt == BLE_ISO_GROUP_MAX)
    {
        p_env = NULL;
    }

    // Return the stream parameters
    return (p_env);
}

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * Handle creation of CIS one by one
 ****************************************************************************************
 */
__STATIC void lli_cis_create_cont(void)
{
    // Initialize returned status
    uint8_t cursor;

    for (cursor = lli_ci_env.cursor ; cursor < lli_ci_env.p_create_cmd->cis_count ; cursor++)
    {
        // Pointer to the CIS Channel information structure
        struct lli_cis_env *p_cis_env;
        struct lli_cig_env *p_cig_env = NULL;

        uint8_t  act_id = BLE_CISHDL_TO_ACTID(lli_ci_env.p_create_cmd->params[cursor].cis_conhdl);

        // Retrieve CIS environment
        p_cis_env = lli_cis_env[act_id];

        if (p_cis_env != NULL)
        {
            uint8_t status = CO_ERROR_NO_ERROR;
            uint16_t act_offset;
            uint16_t act_margin;
            uint8_t cnt;

            // Retrieve CIG environment
            LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

            for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
            {
                struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

                // Check if CIS exists and has the same identifiers
                if ((p_search_cis_env != NULL)
                        && (p_search_cis_env->grp_hdl == p_cis_env->grp_hdl)
                        && (p_search_cis_env->state >= CIS_CREATE_NEGO))
                {
                    break;
                }
            }

            // Check that negotiation is performed for first CIS
            if (cnt >= BLE_ACTIVITY_MAX)
            {
                struct lli_cig_param *p_cig_param = &(p_cig_env->cig_param);

                // Environment of other CIS of the CIG
                struct lli_cis_env *p_oth_cis_env;

                uint32_t cis_offset = 0;
                // CIG Sync Delay us
                uint32_t cig_sync_delay = 0;

                if (p_cig_env->packing == ISO_PACKING_INTERLEAVED)
                {
                    // **************************************
                    // Step 1, compute the channel offset and CIG interval
                    // **************************************
                    p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->streams));

                    // browse all available streams
                    while (p_oth_cis_env != NULL)
                    {
                        struct lli_cis_param *p_oth_cis_param = &(p_oth_cis_env->cis_param);

                        p_oth_cis_param->cis_offset_in_cig = cis_offset;
                        // update the CIS offset
                        cis_offset += p_oth_cis_param->sub_evt_dur + lli_env.sub_evt_margin;

                        // load next element
                        p_oth_cis_env = (struct lli_cis_env *) p_oth_cis_env->hdr.next;
                    }

                    // Group sub-event interval correspond to maximum CIS offset
                    p_cig_env->sub_interval = cis_offset;

                    // **************************************
                    // Step 2, Compute CIG/CIS Sync delay, update channel sub-event interval
                    // **************************************
                    p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->streams));

                    // browse all available streams
                    while (p_oth_cis_env != NULL)
                    {
                        uint32_t cis_sync_delay;
                        struct lli_cis_param *p_oth_cis_param = &(p_oth_cis_env->cis_param);

                        // Set the channel offset, stream interval
                        p_oth_cis_param->sub_interval = p_cig_env->sub_interval;

                        // Compute CIS Sync delay
                        cis_sync_delay = p_oth_cis_param->cis_offset_in_cig
                                         + (p_cig_env->sub_interval * (p_oth_cis_param->nse - 1))
                                         + p_oth_cis_param->sub_evt_dur;

                        // update CIG sync delay if needed
                        if (cis_sync_delay > cig_sync_delay)
                        {
                            cig_sync_delay = cis_sync_delay;
                        }

                        // load next element
                        p_oth_cis_env = (struct lli_cis_env *) p_oth_cis_env->hdr.next;
                    }
                }
                else // Sequential
                {
                    // **************************************
                    // Compute channel offsets, stream delay, use sub event duration as sub interval
                    // **************************************
                    p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->streams));

                    // browse all available streams
                    while (p_oth_cis_env != NULL)
                    {
                        uint32_t cis_sync_delay;
                        struct lli_cis_param *p_oth_cis_param = &(p_oth_cis_env->cis_param);

                        // Set the CIS offset, stream interval
                        p_oth_cis_param->cis_offset_in_cig = cis_offset;
                        p_oth_cis_param->sub_interval = p_oth_cis_param->sub_evt_dur + lli_env.sub_evt_margin;

                        // Compute CIS Sync delay
                        cis_sync_delay = p_oth_cis_param->sub_interval * p_oth_cis_param->nse;

                        // update CIG Sync delay (remove margin to be at end of sub event)
                        cig_sync_delay = cis_offset + cis_sync_delay - lli_env.sub_evt_margin;

                        // load next element
                        p_oth_cis_env = (struct lli_cis_env *) p_oth_cis_env->hdr.next;
                        cis_offset += cis_sync_delay;
                    }
                }

                // If computed stream bandwidth is too large to fit in a channel interval, reject the CIS creation
                if (cig_sync_delay > ((uint32_t)p_cig_param->iso_interval * SLOT_SIZE * 2))
                {
                    status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                }
                else
                {
                    // Request an offset range to the planner
                    struct sch_plan_req_param req_param;
                    req_param.interval_max    = p_cig_param->iso_interval << 2;
                    req_param.interval_min    = p_cig_param->iso_interval << 2;
                    req_param.duration_min    = (2 * cig_sync_delay + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE;
                    req_param.duration_max    = (2 * cig_sync_delay + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE;
                    req_param.offset_min      = 0;
                    req_param.offset_max      = req_param.interval_max - 1;
                    req_param.margin          = 0;
                    req_param.pref_period     = 0;
                    req_param.conhdl          = lli_ci_env.p_create_cmd->params[cursor].cis_conhdl;
                    req_param.conhdl_ref      = BLE_LINKID_TO_CONHDL(p_cis_env->link_id);

                    if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
                    {
                        act_offset = req_param.offset_min;
                        act_margin = req_param.offset_max - req_param.offset_min;
                    }
                    else
                    {
                        // No available offset found, let slave provide preferred parameters
                        act_offset = 0;
                        act_margin = req_param.interval_max - 1;
                    }

                    p_cig_param->cig_sync_delay = cig_sync_delay;
                }
            }
            else
            {
                // Timings are fixed for subsequent CISes
                act_offset = p_cig_env->act_offset;
                act_margin = 0;
            }

            if (status == CO_ERROR_NO_ERROR)
            {
                // Start negotiation at Connection level (handled by LLC CIS module)
                status = llc_cis_create_req(p_cis_env->link_id,  p_cis_env->act_id, act_offset, act_margin);

                // If negotiation is started
                if (status == CO_ERROR_NO_ERROR)
                {
                    // Indicate CIS under negotiation
                    p_cis_env->state = CIS_CREATE_NEGO;
                }

                break;
            }

            // Send message to inform host that CIS establishment fails
            lli_cis_established_evt_send(p_cig_env, p_cis_env, status, 0, 0);
        }
    }

    lli_ci_env.cursor = cursor;

    // operation is over perform command clean-up
    if (lli_ci_env.cursor >= lli_ci_env.p_create_cmd->cis_count)
    {
        ke_free(ke_param2msg(lli_ci_env.p_create_cmd));
        lli_ci_env.p_create_cmd = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Compute CIG/CIS parameters
 *
 * @param[in]   p_cmd        Set CIG parameters command
 * @param[out]  p_cig_param  CIG parameters
 * @param[in]   p_evt        Set CIS parameters command complete event (used to obtain the pointers to each CIS environment)
 ****************************************************************************************
 */
__STATIC uint8_t lli_cig_compute_params(const struct hci_le_set_cig_params_cmd *p_cmd, struct lli_cig_param *p_cig_param, const struct hci_le_set_cig_params_cmd_cmp_evt *p_evt)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t cig_framing = p_cmd->framing;
    uint16_t cig_iso_intv = 0xFFFF;

    // Parse all the CISes in the CIG
    for (uint8_t i = 0; i < p_cmd->cis_count; i++)
    {
        uint8_t framing = cig_framing;

        struct lli_cis_env   *p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_evt->conhdl[i])];
        struct lli_cis_param *p_cis_param;

        ASSERT_ERR(p_cis_env != NULL);

        p_cis_param = &(p_cis_env->cis_param);

        p_cis_param->nse = 0;

        // For each direction (Tx and Rx)
        for (uint8_t direction = 0; direction < 2; direction++)
        {
            uint32_t sdu_interval  = (direction == 0) ? p_cmd->sdu_interval_m2s : p_cmd->sdu_interval_s2m;
            uint16_t max_sdu       = (direction == 0) ? p_cis_param->tx_max_sdu : p_cis_param->rx_max_sdu;
            uint16_t iso_intv      = (direction == 0) ? p_cmd->sdu_interval_m2s / (SLOT_SIZE * 2) : p_cmd->sdu_interval_s2m / (SLOT_SIZE * 2);
            uint16_t trans_latency = (direction == 0) ? p_cmd->max_trans_latency_m2s : p_cmd->max_trans_latency_s2m;
            uint8_t ft = 1;
            uint8_t bn, max_pdu;

            // If there is no data in this direction
            if ((sdu_interval == 0) || (max_sdu == 0))
            {
                max_sdu = 0;
                iso_intv = 0;

                continue;
            }

            // If the SDU interval is not a multiple of 1.25 ms
            if (CO_MOD(sdu_interval, (SLOT_SIZE * 2)))
            {
                framing = ISO_FRAMED_MODE;
            }

            if (framing == ISO_UNFRAMED_MODE)
            {
                // iso_intv is either sdu_interval or 2*sdu_interval_
                uint8_t factor = ((trans_latency * 1000) >= 2 * iso_intv * (SLOT_SIZE * 2)) ? 2 : 1;
                uint8_t pdu_nb_per_sdu;

                if (max_sdu <= BLE_ISO_MAX_PAYLOAD_SIZE)
                {
                    pdu_nb_per_sdu = 1;
                    max_pdu = max_sdu;
                }
                else
                {
                    pdu_nb_per_sdu = CO_DIVIDE_CEIL(max_sdu, BLE_ISO_MAX_PAYLOAD_SIZE);
                    max_pdu = CO_DIVIDE_CEIL(max_sdu, pdu_nb_per_sdu);
                }

                iso_intv = iso_intv * factor;
                bn = factor * pdu_nb_per_sdu;
                ft = (trans_latency * 1000) / (iso_intv * (SLOT_SIZE * 2));
            }
            else // (framing == ISO_FRAMED_MODE)
            {
                uint8_t overhead = BLE_ISOAL_SEG_HEADER_SIZE + BLE_ISOAL_TIME_OFFSET_SIZE;
                uint8_t pdu_nb_per_sdu;

                if (max_sdu <= BLE_ISO_MAX_PAYLOAD_SIZE - overhead)
                {
                    pdu_nb_per_sdu = 1;
                    max_pdu = max_sdu + overhead;
                }
                else
                {
                    pdu_nb_per_sdu = CO_DIVIDE_CEIL(max_sdu, (BLE_ISO_MAX_PAYLOAD_SIZE - overhead));
                    max_pdu = CO_DIVIDE_CEIL(max_sdu, pdu_nb_per_sdu) + overhead;
                }

                bn = pdu_nb_per_sdu;
                ft = (trans_latency * 1000 - sdu_interval) / (iso_intv * (SLOT_SIZE * 2)) - 1;
            }

            if (ft == 0)
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }

            // Save the PDU size and burst number for each direction (Tx and Rx)
            if (direction == 0) // Tx
            {
                p_cis_param->tx_max_pdu = max_pdu;
                p_cis_param->tx_bn = bn;

                if (i == 0)
                {
                    p_cig_param->tx_ft = ft;
                }
            }
            else // Rx
            {
                p_cis_param->rx_max_pdu = max_pdu;
                p_cis_param->rx_bn = bn;

                if (i == 0)
                {
                    p_cig_param->rx_ft = ft;
                }
            }

            // If any direction in any CIS requires framed PDUs
            if ((cig_framing == ISO_UNFRAMED_MODE) && framing)
            {
                cig_framing = ISO_FRAMED_MODE;
            }

            // Save the minimum ISO interval of the CIG
            if ((iso_intv != 0) && (iso_intv < cig_iso_intv))
            {
                cig_iso_intv = iso_intv;
            }
        }

        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Compute packet duration over the air to have the minimum sub_evt duration
        p_cis_param->sub_evt_dur = lli_ci_se_dur_compute(p_cis_param->tx_max_pdu, p_cis_param->tx_rate, p_cis_param->rx_max_pdu, p_cis_param->rx_rate, true);

        // Save the number of subevents
        {
            uint8_t rtn = co_max(p_cmd->params[i].rtn_m2s, p_cmd->params[i].rtn_s2m);
            uint8_t ft = co_max(p_cig_param->tx_ft, p_cig_param->rx_ft);
            uint8_t bn = co_max(p_cis_param->tx_bn, p_cis_param->rx_bn);
            uint8_t nse = (framing == ISO_UNFRAMED_MODE) ? ((rtn + 1) * bn) / (bn * (ft - 1) + 1) : (rtn + bn) / ft;
            // Consider a bandwidth use of (6/10)
            uint8_t max_nse = (cig_iso_intv * SLOT_SIZE * 2 * 6) / (p_cis_param->sub_evt_dur * 10);

            p_cis_param->nse = co_max(co_min(nse, max_nse), bn);
        }
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        // Save computed CIG parameters
        p_cig_param->framing = cig_framing;
        p_cig_param->iso_interval = cig_iso_intv;
    }

    return (status);
}
#endif // (BLE_CENTRAL)



/**
 ****************************************************************************************
 * @brief Inform that the CIS can be started
 *
 * @param[in] act_id         Activity ID
 * @param[in] access_addr    CIS access address
 * @param[in] evt_cnt        Connection Event counter as reference point for CIS start
 * @param[in] con_offset     Offset after connection reference in us when the CIS can be started
 * @param[in] act_offset     Activity offset, integer part (in half-slots)
 * @param[in] act_offset_hus Activity offset, fractional part (in half-us)
 * @param[in] air_time       Air duration required by the activity (in us)
 * @param[in] encrypted      Indicates if the CIS is encrypted or not
 *
 * @return Status
 ****************************************************************************************
 */
__STATIC uint8_t lli_cis_start(uint8_t act_id, uint32_t access_addr, uint16_t evt_cnt, uint32_t con_offset, uint16_t act_offset, uint16_t act_offset_hus, uint32_t air_time, bool encrypted)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env = lli_cis_env[act_id];
    struct lli_cig_env *p_cig_env;

    // Check if CIS exists
    if ((p_cis_env != NULL) && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        struct lli_cig_param *p_cig_param = &(p_cig_env->cig_param);
        struct lli_cis_param *p_cis_param = &(p_cis_env->cis_param);
        struct lld_cis_params drv_param;
        uint32_t cis_start_hs;
        uint16_t cis_start_hus;
        uint8_t cnt;

        for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
        {
            struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

            // Check if CIS exists and has the same identifiers
            if ((p_search_cis_env != NULL)
                    && (p_search_cis_env->grp_hdl == p_cis_env->grp_hdl)
                    && (p_search_cis_env->state >= CIS_WAIT_EST))
            {
                break;
            }
        }

        // Slave must compute the scheduling method as soon as more than one CIS must be established
#if (BLE_PERIPHERAL)
        if ((p_cis_env->role == ROLE_SLAVE))
        {
            if (cnt < BLE_ACTIVITY_MAX)
            {
                struct lli_cis_env *p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->streams));
                // packing is interleaved if both CIS has same interval and both CIS has group offset within this interval
                if ((p_cis_env->cis_param.sub_interval == p_oth_cis_env->cis_param.sub_interval)
                        && (p_cis_env->cis_param.cis_offset_in_cig     < p_cis_env->cis_param.sub_interval)
                        && (p_oth_cis_env->cis_param.cis_offset_in_cig < p_cis_env->cis_param.sub_interval))
                {
                    p_cig_env->packing = ISO_PACKING_INTERLEAVED;
                }
                else
                {
                    p_cig_env->packing = ISO_PACKING_SEQUENTIAL;
                }
            }
            else
            {
                p_cig_env->packing = ISO_PACKING_SEQUENTIAL;
            }

            // add new channel in the list
            co_list_push_back(&(p_cig_env->streams), &(p_cis_env->hdr));
        }
#endif // (BLE_PERIPHERAL)

        // Fill driver parameters
        drv_param.iso_interval     = p_cig_param->iso_interval;
        drv_param.sub_interval     = p_cis_param->sub_interval;
        drv_param.act_id           = p_cis_env->act_id;
        drv_param.grp_hdl          = p_cis_env->grp_hdl;
        drv_param.role             = ((p_cis_env->role == ROLE_MASTER) ? MASTER_ROLE : SLAVE_ROLE);
        drv_param.tx_ft            = p_cig_param->tx_ft;
        drv_param.rx_ft            = p_cig_param->rx_ft;
        drv_param.tx_rate          = p_cis_param->tx_rate;
        drv_param.rx_rate          = p_cis_param->rx_rate;
        drv_param.nse              = p_cis_param->nse;
        drv_param.access_addr      = access_addr;
        drv_param.con_link_id      = p_cis_env->link_id;
        drv_param.con_evt_cnt      = evt_cnt;
        drv_param.con_offset       = con_offset;
        drv_param.framing          = p_cig_param->framing;
        drv_param.tx_max_pdu       = p_cis_param->tx_max_pdu;
        drv_param.rx_max_pdu       = p_cis_param->rx_max_pdu;
        drv_param.tx_bn            = p_cis_param->tx_bn;
        drv_param.rx_bn            = p_cis_param->rx_bn;
        drv_param.tx_sdu_interval  = p_cig_param->tx_sdu_interval;
        drv_param.rx_sdu_interval  = p_cig_param->rx_sdu_interval;
        drv_param.tx_max_sdu       = p_cis_param->tx_max_sdu;
        drv_param.rx_max_sdu       = p_cis_param->rx_max_sdu;

        // Perform transport latency computation - can be done once.
        if (cnt >= BLE_ACTIVITY_MAX)
        {
            // compute iso interval in microseconds
            uint32_t iso_interval_us = ((uint32_t) p_cig_param->iso_interval) * 1250;

            // Unframed mode - compute transport latency
            if (p_cig_param->framing == ISO_UNFRAMED_MODE)
            {
                // Transport_Latency = CIG Sync_Delay + (FT - 1) * ISO_Interval + ((ISO_Interval / SDU interval)-1) * SDU interval
                // Transport_Latency = CIG Sync_Delay + FT * ISO_Interval - SDU interval
                p_cig_param->tx_trans_latency = p_cig_param->cig_sync_delay + (p_cig_param->tx_ft * iso_interval_us)
                                                - p_cig_param->tx_sdu_interval;
                p_cig_param->rx_trans_latency = p_cig_param->cig_sync_delay + (p_cig_param->rx_ft * iso_interval_us)
                                                - p_cig_param->rx_sdu_interval;
            }
            // Framed mode
            else
            {
                // Transport_Latency = CIG Sync_Delay + FT * ISO_Interval + SDU_Interval
                p_cig_param->tx_trans_latency = p_cig_param->cig_sync_delay + (p_cig_param->tx_ft * iso_interval_us)
                                                + p_cig_param->tx_sdu_interval;
                p_cig_param->rx_trans_latency = p_cig_param->cig_sync_delay + (p_cig_param->rx_ft * iso_interval_us)
                                                + p_cig_param->rx_sdu_interval;;
            }
        }


        drv_param.tx_trans_latency = p_cig_param->tx_trans_latency;
        drv_param.rx_trans_latency = p_cig_param->rx_trans_latency;

        drv_param.cis_spacing  = p_cis_param->cis_offset_in_cig;
        drv_param.cig_sync_delay = p_cig_param->cig_sync_delay;

        // Compute packet duration over the air
        drv_param.air_exch_dur = lli_ci_se_dur_compute(p_cis_param->tx_max_pdu, p_cis_param->tx_rate,
                                 p_cis_param->rx_max_pdu, p_cis_param->rx_rate, encrypted);

        drv_param.packing      = p_cig_env->packing;

        status = lld_cis_start(p_cis_env->act_id, &drv_param, &cis_start_hs, &cis_start_hus);

        if (status == CO_ERROR_NO_ERROR)
        {
            struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(p_cis_env->act_id);

            // Load RX/TX Data Path
            if ((drv_param.tx_max_sdu > 0) && (p_cis_env->p_dp[ISO_SEL_TX] != NULL))
            {
                lld_isoal_datapath_set(p_cis_env->act_id, ISO_SEL_TX, p_cis_env->p_dp[ISO_SEL_TX]);

                if (p_cis_env->p_dp[ISO_SEL_TX]->cb_local_sync != NULL)
                {
                    // Enable the local synchronization on this CIS
                    lld_cis_local_sync_en(p_cis_env->act_id, ISO_SEL_TX, p_cis_env->p_dp[ISO_SEL_TX]->cb_local_sync);
                }

#if (BLE_PERIPHERAL)
                if (p_cis_env->p_dp[ISO_SEL_TX]->cb_peer_sync != NULL)
                {
                    // Enable the peer synchronization on this CIS
                    lld_cis_peer_sync_en(p_cis_env->act_id, ISO_SEL_TX, p_cis_env->p_dp[ISO_SEL_TX]->cb_peer_sync);
                }
#endif // (BLE_PERIPHERAL)

            }
            if ((drv_param.rx_max_sdu > 0) && (p_cis_env->p_dp[ISO_SEL_RX] != NULL))
            {
                lld_isoal_datapath_set(p_cis_env->act_id, ISO_SEL_RX, p_cis_env->p_dp[ISO_SEL_RX]);

                if (p_cis_env->p_dp[ISO_SEL_RX]->cb_local_sync != NULL)
                {
                    // Enable the local synchronization on this CIS
                    lld_cis_local_sync_en(p_cis_env->act_id, ISO_SEL_RX, p_cis_env->p_dp[ISO_SEL_RX]->cb_local_sync);
                }

#if (BLE_PERIPHERAL)
                if (p_cis_env->p_dp[ISO_SEL_RX]->cb_peer_sync != NULL)
                {
                    // Enable the peer synchronization on this CIS
                    lld_cis_peer_sync_en(p_cis_env->act_id, ISO_SEL_RX, p_cis_env->p_dp[ISO_SEL_RX]->cb_peer_sync);
                }
#endif // (BLE_PERIPHERAL)
            }

#if (BLE_CENTRAL)
            uint8_t cnt;

            for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
            {
                struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

                // Check if CIS exists and has the same identifiers
                if ((p_search_cis_env != NULL)
                        && (p_search_cis_env->grp_hdl == lli_cis_env[act_id]->grp_hdl)
                        && (p_search_cis_env->state >= CIS_WAIT_EST))
                {
                    break;
                }
            }

            // First CIS enabled, store the activity offset information
            if ((p_cis_env->role == ROLE_MASTER) && (cnt >= BLE_ACTIVITY_MAX))
            {
                p_cig_env->act_offset = act_offset;
            }
#endif // (BLE_CENTRAL)

            // Register CIS in the planner
            plan_elt->offset       = act_offset;
            plan_elt->interval     = p_cig_param->iso_interval << 2;
            plan_elt->duration_min = (act_offset_hus + 2 * air_time + HALF_SLOT_SIZE - 1) / HALF_SLOT_SIZE;
            plan_elt->duration_max = plan_elt->duration_min;
            plan_elt->margin       = 1 * (p_cis_env->role == ROLE_SLAVE);
            plan_elt->conhdl       = BLE_ACTID_TO_CISHDL(p_cis_env->act_id);
            plan_elt->conhdl_ref   = BLE_LINKID_TO_CONHDL(p_cis_env->link_id);
            plan_elt->cb_move      = NULL;
            plan_elt->mobility     = SCH_PLAN_MB_LVL_0;

            sch_plan_set(plan_elt);

            p_cis_env->state = CIS_WAIT_EST;

#if (BLE_CENTRAL)
            // The CIG moves to the active state
            p_cig_env->not_configurable = true;
#endif // (BLE_CENTRAL)

            // inform that new CIS is granted
            if ((lli_ci_env.cb_start_evt != NULL))
            {
                lli_ci_env.cb_start_evt(p_cis_env->act_id, drv_param.con_link_id, p_cis_param,
                                        drv_param.iso_interval, drv_param.access_addr, cis_start_hs, cis_start_hus);
            }

            // Store framing
            lli_env.framing[p_cis_env->act_id] = p_cig_param->framing;
        }
    }
    else
    {
        status = CO_ERROR_UNSPECIFIED_ERROR;
    }

    return status;
}


/*
 * GLOBAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */



void lli_ci_param_get(uint8_t act_id, struct lli_cig_param **pp_cig_param, struct lli_cis_param **pp_cis_param)
{
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env = lli_cis_env[act_id];
    struct lli_cig_env *p_cig_env;

    // Check if CIS exists
    if ((p_cis_env != NULL) && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        if (pp_cig_param != NULL)
            *pp_cig_param = &(p_cig_env->cig_param);

        if (pp_cis_param != NULL)
            *pp_cis_param = &(p_cis_env->cis_param);
    }
}

#if (BLE_PERIPHERAL)
uint8_t lli_cis_create_req(uint8_t link_id, struct ll_cis_req *p_pdu, uint8_t framing, uint8_t *p_act_id)
{
    // Returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_cig_env *p_cig_env = NULL;

    do
    {
        // Channel handle allocated for the new channel
        uint8_t act_id;
        // Pointer to allocated CIS environment
        struct lli_cis_env *p_cis_env;
        struct lli_cig_param *p_cig_param;
        struct lli_cis_param *p_cis_param;

        // Get environement of the stream
        p_cig_env = lli_cig_get(p_pdu->cig_id, ROLE_SLAVE, link_id);
        if (p_cig_env == NULL)
        {
            // Allocate stream environment
            status = lli_group_create(LLI_ISO_GROUP_CIG, sizeof(struct lli_cig_env), (struct lli_group_env **) & (p_cig_env));

            // Check that stream environment properly loaded, else exist
            if (status != CO_ERROR_NO_ERROR)
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
                break;
            }

            p_cig_param = &(p_cig_env->cig_param);
            p_cig_env->role                  = ROLE_SLAVE;
            p_cig_env->cig_id                = p_pdu->cig_id;
            p_cig_env->sub_interval          = 0;
            p_cig_env->link_id               = link_id;
            p_cig_env->nb_cis                = 0;
            co_list_init(&(p_cig_env->streams));

            p_cig_param->iso_interval        = p_pdu->iso_interval;
            p_cig_param->tx_sdu_interval     = p_pdu->sdu_interval_s2m;
            p_cig_param->rx_sdu_interval     = p_pdu->sdu_interval_m2s;
            p_cig_param->framing             = framing;
            p_cig_param->tx_ft               = p_pdu->ft_s2m;
            p_cig_param->rx_ft               = p_pdu->ft_m2s;
        }
        else
        {
            uint8_t cis_act_id = lli_cis_act_id_get(link_id, p_pdu->cig_id, p_pdu->cis_id);
            p_cig_param = &(p_cig_env->cig_param);

            // Check that for this connection handle there is not already a CIS with the same cig_id/cis_id
            if (cis_act_id < BLE_ACTIVITY_MAX)
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
                break;
            }

            // check that all channel interval of the CIG are equals
            if ((p_cig_param->iso_interval    != p_pdu->iso_interval)
                    || (p_cig_param->tx_sdu_interval != p_pdu->sdu_interval_s2m)
                    || (p_cig_param->rx_sdu_interval != p_pdu->sdu_interval_m2s)
                    || (p_cig_param->tx_ft           != p_pdu->ft_s2m)
                    || (p_cig_param->rx_ft           != p_pdu->ft_m2s))
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
                break;
            }
        }

        // Try to allocate a new CIS environment
        status = lli_cis_create(&p_cis_env);

        // Stop operation is not possible to allocate memory
        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_UNSPECIFIED_ERROR;
            break;
        }

        // Store CIS environment pointer
        lli_cis_env[p_cis_env->act_id] = p_cis_env;

        // Set connection handle
        p_cis_env->link_id            = link_id;
        p_cis_env->role               = ROLE_SLAVE;
        p_cis_env->grp_hdl            = p_cig_env->group.hdl;

        act_id                      = p_cis_env->act_id;

        // Store received CIS Channel parameters
        p_cis_param = &(p_cis_env->cis_param);
        p_cis_param->cig_id           = p_pdu->cig_id;
        p_cis_param->cis_id           = p_pdu->cis_id;
        p_cis_param->nse              = p_pdu->nse;
        p_cis_param->tx_rate          = p_pdu->phy_s2m;
        p_cis_param->tx_max_pdu       = p_pdu->max_pdu_s2m;
        p_cis_param->tx_max_sdu       = p_pdu->max_sdu_s2m;
        p_cis_param->tx_bn            = GETF(p_pdu->bn, LLCP_CIS_BN_S2M);
        p_cis_param->rx_rate          = p_pdu->phy_m2s;
        p_cis_param->rx_max_pdu       = p_pdu->max_pdu_m2s;
        p_cis_param->rx_max_sdu       = p_pdu->max_sdu_m2s;
        p_cis_param->rx_bn            = GETF(p_pdu->bn, LLCP_CIS_BN_M2S);
        p_cis_param->sub_interval     = p_pdu->sub_interval;


        // Send HCI LE CIS Channel Request Event to the Host
        lli_cis_request_evt_send(p_cis_env);

        // mark that procedure is waiting for host response
        p_cis_env->state = CIS_WAIT_HOST;
        p_cig_env->nb_cis++;

        // Return the activity ID
        *p_act_id = act_id;
    }
    while (0);

    if ((status != CO_ERROR_NO_ERROR) && (p_cig_env != NULL) && (p_cig_env->nb_cis == 0))
    {
        lli_group_cleanup(p_cig_env->group.hdl);
    }

    return (status);
}
#endif //(BLE_PERIPHERAL)

void lli_cis_create_end(uint8_t act_id, uint8_t status, uint32_t access_addr, uint16_t evt_cnt, uint32_t con_offset, uint16_t act_offset, uint16_t act_offset_hus, uint32_t air_time, bool encrypted)
{
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env = lli_cis_env[act_id];
    struct lli_cig_env *p_cig_env;

    // Check if CIS exists
    if ((p_cis_env != NULL) && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        struct lli_cis_param *p_cis_param = &(p_cis_env->cis_param);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Start the CIS
            status = lli_cis_start(act_id, access_addr, evt_cnt, con_offset, act_offset, act_offset_hus, air_time, encrypted);
        }
        else
        {
#if (BLE_CENTRAL)
            if (p_cis_env->role == ROLE_MASTER)
            {
                // Send HCI CIS Enable Complete Event to the Host
                lli_cis_established_evt_send(p_cig_env, p_cis_env, status, p_cis_param->tx_rate, p_cis_param->rx_rate);

                // Clean the connection handle
                p_cis_env->link_id = BLE_INVALID_LINK_ID;

                // Move state
                p_cis_env->state = CIS_EXISTS;
            }
#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
            // Check if device is slave
            if (p_cis_env->role == ROLE_SLAVE)
            {
                // Remove CIS from CIG
                co_list_extract(&(p_cig_env->streams), &(p_cis_env->hdr));
                p_cig_env->nb_cis--;

                // Clean the stream environment
                lli_cis_cleanup(p_cis_env);

                // Check if CIG can be removed
                if ((p_cig_env->nb_cis == 0))
                {
                    // Clean-up the CIG
                    lli_group_cleanup(p_cig_env->group.hdl);
                }
            }
#endif //(BLE_PERIPHERAL)
        }
    }
    else
    {
        ASSERT_INFO(0, act_id, status);
    }

#if (BLE_CENTRAL)
    if (lli_ci_env.p_create_cmd != NULL)
    {
        // check if another CIS must be created
        lli_ci_env.cursor++;
        lli_cis_create_cont();
    }
#endif // (BLE_CENTRAL)
}

void lli_cis_stop_req(uint8_t link_id, struct ll_cis_terminate_ind *p_pdu)
{
    uint8_t act_id = lli_cis_act_id_get(link_id, p_pdu->cig_id, p_pdu->cis_id);

    if (act_id < BLE_ACTIVITY_MAX)
    {
        // Pointer to the indicated CIS environment
        struct lli_cis_env *p_cis_env = lli_cis_env[act_id];

        // Check if CIS exists and in a suitable state
        if ((p_cis_env != NULL) && (p_cis_env->state >= CIS_WAIT_EST))
        {
            // Store reason
            p_cis_env->reason = p_pdu->err_code;

            // Mark that a procedure is on-going on CIG
            p_cis_env->state = CIS_STOPPING;

            // Stop the CIS driver
            lld_cis_stop(p_cis_env->act_id);
        }
    }
}


void lli_cis_start_evt_set(lli_cis_start_evt_cb evt_cb)
{
    lli_ci_env.cb_start_evt = evt_cb;
}


#endif // !(BLE_CIS_IN_ROM)



/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

#if (BLE_CENTRAL)
int hci_le_set_cig_params_cmd_handler_patch(const struct hci_le_set_cig_params_cmd *p_cmd, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_LE_SET_CIG_PARAMS_CMD_HANDLER_FUN_BIT, int, p_cmd, opcode);
    // Allocate the command complete event to be sent to the host
    struct hci_le_set_cig_params_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode,
            hci_le_set_cig_params_cmd_cmp_evt);
#if !(BLE_CIS_IN_ROM)
    // Initialize returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_cig_env *p_cig_env = NULL;

    do
    {
        uint8_t cursor;
        uint8_t cnt;
        // Pointer to the CIS information structure
        struct lli_cis_env   *p_cis_env;
        struct lli_cig_param *p_cig_param;

        // Check if feature is locally supported
        uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // Check provided parameters
        if ((p_cmd->cig_id > BLE_CIG_MAX_ID)                       || (p_cmd->packing >= ISO_PACKING_MAX)
                || (p_cmd->sdu_interval_m2s  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_m2s  > BLE_ISO_MAX_SDU_INTERVAL)
                || (p_cmd->sdu_interval_s2m  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_s2m  > BLE_ISO_MAX_SDU_INTERVAL)
                || (p_cmd->sca > SCA_20PPM)
                || (p_cmd->packing >= ISO_PACKING_MAX)
                || (p_cmd->framing >= ISO_FRAME_MODE_MAX)
                || (p_cmd->max_trans_latency_m2s < BLE_ISO_MIN_TRANS_LATENCY) || (p_cmd->max_trans_latency_m2s > BLE_ISO_MAX_TRANS_LATENCY)
                || (p_cmd->max_trans_latency_s2m < BLE_ISO_MIN_TRANS_LATENCY) || (p_cmd->max_trans_latency_s2m > BLE_ISO_MAX_TRANS_LATENCY)
                || (p_cmd->cis_count > BLE_CIS_MAX_CNT))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Get environment of the CIG
        p_cig_env = lli_cig_get(p_cmd->cig_id, ROLE_MASTER, BLE_INVALID_LINK_ID);

        // If Group does not exist
        if (p_cig_env == NULL)
        {
            // Allocate CIG environment
            status = lli_group_create(LLI_ISO_GROUP_CIG, sizeof(struct lli_cig_env),
                                      (struct lli_group_env **) & (p_cig_env));

            // Check that CIG environment properly loaded, else exit
            if (status != CO_ERROR_NO_ERROR)
                break;

            co_list_init(&(p_cig_env->streams));
            p_cig_env->role           = ROLE_MASTER;
            p_cig_env->cig_id         = p_cmd->cig_id;
            p_cig_env->sub_interval   = 0;
            p_cig_env->link_id        = BLE_INVALID_LINK_ID;
            p_cig_env->nb_cis         = 0;
        }
        // If the CIG is not configurable
        else if (p_cig_env->not_configurable)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
        {
            struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

            // Check if CIS exists and has the same identifiers
            if ((p_search_cis_env != NULL)
                    && (p_search_cis_env->grp_hdl == p_cig_env->group.hdl)
                    && (p_search_cis_env->state >= CIS_CREATE_NEGO))
            {
                break;
            }
        }

        // check if it's possible to insert CIS (stream busy or CIS_ID already present)
        if (cnt < BLE_ACTIVITY_MAX)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        p_cig_param = &(p_cig_env->cig_param);


        // Browse all streams
        for (cursor = 0 ; cursor < p_cmd->cis_count ; cursor++)
        {
            uint8_t tx_rate, rx_rate;
            uint8_t cis_act_id;
            // parameters needed to check group validity
            uint16_t sub_evt_dur;
            const struct hci_le_cis_param *p_params = &(p_cmd->params[cursor]);
            struct lli_cis_param *p_cis_param;

            // Check provided parameters
            if ((p_params->cis_id > BLE_CIS_MAX_ID)
                    || (p_params->max_sdu_m2s > BLE_ISO_MAX_SDU_SIZE) || (p_params->max_sdu_s2m > BLE_ISO_MAX_SDU_SIZE)
                    || (p_params->rtn_m2s > BLE_CIS_MAX_RTN)          || (p_params->rtn_s2m > BLE_CIS_MAX_RTN))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Check if PHY proposed are supported
            if (((p_params->phy_m2s & supp_phy_loc_msk) == 0) || ((p_params->phy_s2m & supp_phy_loc_msk) == 0))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }

            // Select a PHY for each direction (Tx/Rx)
            tx_rate = lli_ci_get_rate(p_params->phy_m2s);
            rx_rate = lli_ci_get_rate(p_params->phy_s2m);

            // Compute packet duration over the air to have the minimum sub_evt duration
            sub_evt_dur = lli_ci_se_dur_compute(p_params->max_sdu_m2s, tx_rate, p_params->max_sdu_s2m, rx_rate, true);

            cis_act_id = lli_cis_act_id_get(BLE_INVALID_LINK_ID, p_cig_env->cig_id, p_params->cis_id);
            // check if cis_id already present in group
            if (cis_act_id >= BLE_ACTIVITY_MAX)
            {
                // Try to allocate a new CIS environment
                status = lli_cis_create(&p_cis_env);

                // check if a stream can be allocated
                if (status != CO_ERROR_NO_ERROR)
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                    break;
                }

                // put CIS into the queue
                co_list_push_back(&(p_cig_env->streams), &(p_cis_env->hdr));

                // Store CIS environment pointer
                lli_cis_env[p_cis_env->act_id] = p_cis_env;
            }
            else
            {
                p_cis_env = lli_cis_env[cis_act_id];
            }

            // Check if a data path has already been set up
            if (((!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_TX])) && (p_params->max_sdu_m2s == 0))
                    || ((!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_RX])) && (p_params->max_sdu_s2m == 0)))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            p_cis_param = &(p_cis_env->cis_param);

            // Keep provided parameters
            p_cis_env->role              = ROLE_MASTER;
            p_cis_env->grp_hdl           = p_cig_env->group.hdl;

            // These parameters do not change
            p_cis_param->cis_id          = p_params->cis_id;
            p_cis_param->cig_id          = p_cmd->cig_id;
            p_cis_param->tx_max_sdu      = p_params->max_sdu_m2s;
            p_cis_param->rx_max_sdu      = p_params->max_sdu_s2m;
            p_cis_param->tx_rate         = tx_rate;
            p_cis_param->rx_rate         = rx_rate;
            p_cig_param->tx_ft           = 1;
            p_cig_param->rx_ft           = 1;
            p_cig_param->framing         = p_cmd->framing;

            // These parameters are initialized here and will be updated later
            p_cis_param->tx_max_pdu      = p_params->max_sdu_m2s;
            p_cis_param->rx_max_pdu      = p_params->max_sdu_s2m;
            p_cis_param->tx_bn           = (p_params->max_sdu_m2s > 0 ? 1 : 0);
            p_cis_param->rx_bn           = (p_params->max_sdu_s2m > 0 ? 1 : 0);
            p_cis_param->nse             = co_max(p_cis_param->tx_bn * (p_params->rtn_m2s + 1),
                                                  p_cis_param->rx_bn * (p_params->rtn_s2m + 1));
            p_cis_param->sub_evt_dur     = sub_evt_dur;

            // Set CIS handle in the command complete event message
            p_evt->conhdl[cursor]                = BLE_ACTID_TO_CISHDL(p_cis_env->act_id);

            // Add the new CIS in the CIG
            p_cis_env->state = CIS_EXISTS;
            p_cig_env->nb_cis++;
        }

        if (status == CO_ERROR_NO_ERROR)
        {
            // Compute parameters for the CIG and all the CISes
            status = lli_cig_compute_params(p_cmd, p_cig_param, p_evt);
        }

        // an error occurs, stop the command
        if (status != CO_ERROR_NO_ERROR)
        {
            uint8_t nb_added_chan = cursor;
            // Browse all streams
            for (cursor = 0 ; cursor < nb_added_chan ; cursor++)
            {
                p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_evt->conhdl[cursor])];
                ASSERT_ERR(p_cis_env != NULL);

                // mark CIS removed
                p_cig_env->nb_cis--;

                // Free the associated activity
                co_list_extract(&(p_cig_env->streams), &(p_cis_env->hdr));

                // Clean the stream environment
                lli_cis_cleanup(p_cis_env);
            }
            break;
        }

        // update scheduling parameters
        //p_cig_param->iso_interval          = (p_cmd->sdu_interval_m2s / (SLOT_SIZE * 2));
        p_cig_param->tx_sdu_interval       = p_cmd->sdu_interval_m2s;
        p_cig_param->rx_sdu_interval       = p_cmd->sdu_interval_s2m;

        p_cig_env->packing                 = p_cmd->packing;

    }
    while (0);


    // Send the command complete event
    p_evt->cig_id = p_cmd->cig_id;
    p_evt->cis_count = (status == CO_ERROR_NO_ERROR) ? p_cmd->cis_count : 0;
    p_evt->status    = status;
    hci_send_2_host(p_evt);

    // if an error occurs, check if CIG environment should be cleaned-up
    if ((status != CO_ERROR_NO_ERROR) && (p_cig_env != NULL) && (p_cig_env->nb_cis == 0))
    {
        lli_group_cleanup(p_cig_env->group.hdl);
    }
#else // BLE_CIS_IN_ROM
    // Send the command complete event
    p_evt->cig_id = p_cmd->cig_id;
    p_evt->cis_count = 0;
    p_evt->status    = CO_ERROR_COMMAND_DISALLOWED;
    hci_send_2_host(p_evt);
#endif // BLE_CIS_IN_ROM
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_set_cig_params_test_cmd_handler_patch(struct hci_le_set_cig_params_test_cmd *p_cmd, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_LE_SET_CIG_PARAMS_TEST_CMD_HANDLER_FUN_BIT, int, p_cmd, opcode);
    // Allocate the command complete event to be sent to the host
    struct hci_le_set_cig_params_test_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_set_cig_params_test_cmd_cmp_evt);

#if !(BLE_CIS_IN_ROM)
    // Initialize returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_cig_env *p_cig_env = NULL;
    struct lli_cig_param *p_cig_param;

    do
    {
        uint8_t cursor;
        uint8_t cnt;
        // Pointer to the CIS information structure
        struct lli_cis_env   *p_cis_env;

        uint8_t ft_m2s = 1;
        uint8_t ft_s2m = 1;
        uint32_t iso_interval_us = p_cmd->iso_interval * (SLOT_SIZE * 2);

        // Check if feature is locally supported
        uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // Check provided parameters
        if ((p_cmd->cig_id > BLE_CIG_MAX_ID)                       || (p_cmd->iso_interval > BLE_ISO_MAX_INTERVAL)
                || (p_cmd->sdu_interval_m2s  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_m2s  > BLE_ISO_MAX_SDU_INTERVAL)
                || (p_cmd->sdu_interval_s2m  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_s2m  > BLE_ISO_MAX_SDU_INTERVAL)
                || (p_cmd->ft_s2m < BLE_CIS_MIN_FT)                       || (p_cmd->ft_s2m > BLE_CIS_MAX_FT)
                || (p_cmd->ft_m2s < BLE_CIS_MIN_FT)                       || (p_cmd->ft_m2s > BLE_CIS_MAX_FT)
                || (p_cmd->sca > SCA_20PPM)
                || (p_cmd->packing >= ISO_PACKING_MAX)                    || (p_cmd->framing >= ISO_FRAME_MODE_MAX))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        if (p_cmd->framing == ISO_UNFRAMED_MODE)
        {
            // ISO interval and SDU interval must be multiple
            if ((CO_MOD(iso_interval_us, p_cmd->sdu_interval_m2s) != 0)
                    || (CO_MOD(iso_interval_us, p_cmd->sdu_interval_s2m) != 0))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        // Get environment of the CIG
        p_cig_env = lli_cig_get(p_cmd->cig_id, ROLE_MASTER, BLE_INVALID_LINK_ID);

        // If Group does not exist
        if (p_cig_env == NULL)
        {
            // Allocate CIG environment
            status = lli_group_create(LLI_ISO_GROUP_CIG, sizeof(struct lli_cig_env),
                                      (struct lli_group_env **) & (p_cig_env));

            // Check that CIG environment properly loaded, else exist
            if (status != CO_ERROR_NO_ERROR)
                break;

            co_list_init(&(p_cig_env->streams));
            p_cig_env->role           = ROLE_MASTER;
            p_cig_env->cig_id         = p_cmd->cig_id;
            p_cig_env->sub_interval   = 0;
            p_cig_env->link_id        = BLE_INVALID_LINK_ID;
            p_cig_env->nb_cis         = 0;
        }
        // If the CIG is not configurable
        else if (p_cig_env->not_configurable)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
        {
            struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

            // Check if CIS exists and has the same identifiers
            if ((p_search_cis_env != NULL)
                    && (p_search_cis_env->grp_hdl == p_cig_env->group.hdl)
                    && (p_search_cis_env->state >= CIS_CREATE_NEGO))
            {
                break;
            }
        }

        // check if it's possible to insert CIS (stream busy or CIS_ID already present)
        if (cnt < BLE_ACTIVITY_MAX)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Browse all streams
        for (cursor = 0 ; cursor < p_cmd->cis_count ; cursor++)
        {
            uint8_t tx_rate, rx_rate;
            uint8_t cis_act_id;
            // parameters needed to check group validity
            uint32_t total_evt_dur = 0;
            uint16_t sub_evt_dur;
            struct hci_le_cis_test_param *p_params = &(p_cmd->params[cursor]);
            struct lli_cis_param *p_cis_param;


            // Check provided parameters
            if ((p_params->cis_id > BLE_CIS_MAX_ID)
                    || (p_params->max_pdu_m2s > BLE_ISO_MAX_PAYLOAD_SIZE)  || (p_params->max_pdu_s2m > BLE_ISO_MAX_PAYLOAD_SIZE)
                    || (p_params->max_sdu_m2s > BLE_ISO_MAX_SDU_SIZE)      || (p_params->max_sdu_s2m > BLE_ISO_MAX_SDU_SIZE)
                    || (p_params->nse < BLE_ISO_MIN_NSE)                   || (p_params->nse > BLE_ISO_MAX_NSE)
                    || (p_params->bn_s2m > BLE_CIS_MAX_BN)                 || (p_params->bn_m2s > BLE_CIS_MAX_BN)
                    || (p_params->bn_s2m > p_params->nse)                  || (p_params->bn_m2s > p_params->nse))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Force SDU and PDU size to 0 if BN - 0
            if (p_params->bn_m2s == 0)
            {
                p_params->max_pdu_m2s = 0;
                p_params->max_sdu_m2s = 0;
            }

            // Force SDU and PDU size to 0 if BN - 0
            if (p_params->bn_s2m == 0)
            {
                p_params->max_pdu_s2m = 0;
                p_params->max_sdu_s2m = 0;
            }

            // check size of parameters
            if (p_cmd->framing == ISO_UNFRAMED_MODE)
            {
                // per SDU interval a rounded number of burst is required TODO [FBE] typo
                if ((CO_MOD(p_params->bn_m2s, iso_interval_us / p_cmd->sdu_interval_m2s) != 0)
                        || (CO_MOD(p_params->bn_s2m, iso_interval_us / p_cmd->sdu_interval_s2m) != 0))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
                else
                {
                    uint16_t max_sdu_m2s = p_params->max_pdu_m2s * ((p_params->bn_m2s * p_cmd->sdu_interval_m2s) / iso_interval_us);
                    uint16_t max_sdu_s2m = p_params->max_pdu_s2m * ((p_params->bn_s2m * p_cmd->sdu_interval_s2m) / iso_interval_us);

                    // check that maximum SDU size requested can be transmitted
                    if ((p_params->max_sdu_m2s > max_sdu_m2s) || (p_params->max_sdu_s2m > max_sdu_s2m))
                    {
                        status = CO_ERROR_INVALID_HCI_PARAM;
                        break;
                    }
                }
            }
            else
            {
                uint16_t max_sdu_m2s = ((p_params->max_pdu_m2s * p_params->bn_m2s * p_cmd->sdu_interval_m2s)
                                        - (BLE_ISOAL_SEG_HEADER_SIZE * p_params->bn_m2s)) / iso_interval_us
                                       - BLE_ISOAL_TIME_OFFSET_SIZE;

                uint16_t max_sdu_s2m = ((p_params->max_pdu_s2m * p_params->bn_s2m * p_cmd->sdu_interval_s2m)
                                        - (BLE_ISOAL_SEG_HEADER_SIZE * p_params->bn_s2m)) / iso_interval_us
                                       - BLE_ISOAL_TIME_OFFSET_SIZE;

                // TODO [FBE] this is a small sanity check to ensure that at least a SDU can be transmitted,
                // but it does not work with all segmentation capabilities
                if ((p_params->max_sdu_m2s > max_sdu_m2s) || (p_params->max_sdu_s2m > max_sdu_s2m))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
            }

            // Check if PHY proposed are supported
            if (((p_params->phy_m2s & supp_phy_loc_msk) == 0) || ((p_params->phy_s2m & supp_phy_loc_msk) == 0))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }

            // Only one PHY must be configured
            if ((NB_ONE_BITS(p_params->phy_m2s) != 1)         || (NB_ONE_BITS(p_params->phy_s2m) != 1))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            tx_rate = co_phy_to_rate[co_phy_mask_to_value[p_params->phy_m2s]];
            rx_rate = co_phy_to_rate[co_phy_mask_to_value[p_params->phy_s2m]];

            // Compute packet duration over the air to have the minimum sub_evt duration
            sub_evt_dur = lli_ci_se_dur_compute(p_params->max_pdu_m2s, tx_rate, p_params->max_pdu_s2m, rx_rate, true);

            // compute the event duration and check if parameters can fit
            total_evt_dur = ((sub_evt_dur) * (p_params->nse - 1)) + sub_evt_dur;

            // check if all sub event can enter into the channel interval
            if (total_evt_dur > ((uint32_t)p_cmd->iso_interval * SLOT_SIZE * 2))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            cis_act_id = lli_cis_act_id_get(BLE_INVALID_LINK_ID, p_cmd->cig_id, p_params->cis_id);

            // check if cis_id already present in group
            if (cis_act_id >= BLE_ACTIVITY_MAX)
            {
                // Try to allocate a new CIS environment
                status = lli_cis_create(&p_cis_env);

                // check if a stream can be allocated
                if (status != CO_ERROR_NO_ERROR)
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                    break;
                }

                // put CIS into the queue
                co_list_push_back(&(p_cig_env->streams), &(p_cis_env->hdr));

                // Store CIS environment pointer
                lli_cis_env[p_cis_env->act_id] = p_cis_env;
            }
            else
            {
                p_cis_env = lli_cis_env[cis_act_id];
            }

            // Check if a data path has already been set up
            if (((!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_TX])) && (p_params->max_sdu_m2s == 0))
                    || ((!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_RX])) && (p_params->max_sdu_s2m == 0)))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            p_cis_param = &(p_cis_env->cis_param);

            // Keep provided parameters
            p_cis_env->role                      = ROLE_MASTER;
            p_cis_env->grp_hdl                   = p_cig_env->group.hdl;

            // CIS parameters
            p_cis_param->cis_id                  = p_params->cis_id;
            p_cis_param->cig_id                  = p_cmd->cig_id;
            p_cis_param->nse                     = p_params->nse;
            p_cis_param->tx_max_sdu              = p_params->max_sdu_m2s;
            p_cis_param->rx_max_sdu              = p_params->max_sdu_s2m;
            p_cis_param->tx_max_pdu              = p_params->max_pdu_m2s;
            p_cis_param->rx_max_pdu              = p_params->max_pdu_s2m;
            p_cis_param->tx_bn                   = p_params->bn_m2s;
            p_cis_param->rx_bn                   = p_params->bn_s2m;
            p_cis_param->tx_rate                 = tx_rate;
            p_cis_param->rx_rate                 = rx_rate;
            p_cis_param->sub_evt_dur             = sub_evt_dur;

            ft_m2s                               = p_cmd->ft_m2s;
            ft_s2m                               = p_cmd->ft_s2m;

            // Set channel handle in the command complete event message
            p_evt->conhdl[cursor]                = BLE_ACTID_TO_CISHDL(p_cis_env->act_id);

            // Add the new CIS in the CIG
            p_cis_env->state = CIS_EXISTS;
            p_cig_env->nb_cis++;
        }

        // an error occurs, stop the command
        if (status != CO_ERROR_NO_ERROR)
        {
            uint8_t nb_added_chan = cursor;
            // Browse all streams
            for (cursor = 0 ; cursor < nb_added_chan ; cursor++)
            {
                p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_evt->conhdl[cursor])];
                ASSERT_ERR(p_cis_env != NULL);

                // mark CIS removed
                p_cig_env->nb_cis--;

                // Free the associated activity
                co_list_extract(&(p_cig_env->streams), &(p_cis_env->hdr));

                // Clean the stream environment
                lli_cis_cleanup(p_cis_env);
            }
            break;
        }

        p_cig_param = &(p_cig_env->cig_param);

        // update scheduling parameters
        p_cig_param->iso_interval        = p_cmd->iso_interval;
        p_cig_param->tx_sdu_interval     = p_cmd->sdu_interval_m2s;
        p_cig_param->rx_sdu_interval     = p_cmd->sdu_interval_s2m;
        p_cig_param->framing             = p_cmd->framing;
        p_cig_env->packing               = p_cmd->packing;
        p_cig_param->tx_ft               = ft_m2s;
        p_cig_param->rx_ft               = ft_s2m;

    }
    while (0);

    // Send the command complete event
    p_evt->cig_id = p_cmd->cig_id;
    p_evt->cis_count = (status == CO_ERROR_NO_ERROR) ? p_cmd->cis_count : 0;
    p_evt->status    = status;
    hci_send_2_host(p_evt);

    // if an error occurs, check if CIG environment should be cleaned-up
    if ((status != CO_ERROR_NO_ERROR) && (p_cig_env != NULL) && (p_cig_env->nb_cis == 0))
    {
        lli_group_cleanup(p_cig_env->group.hdl);
    }
#else //BLE_CIS_IN_ROM
    p_evt->cig_id = p_cmd->cig_id;
    p_evt->cis_count = 0;
    p_evt->status    = CO_ERROR_COMMAND_DISALLOWED;
    hci_send_2_host(p_evt);
#endif //BLE_CIS_IN_ROM
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_create_cis_cmd_handler_patch(struct hci_le_create_cis_cmd const *p_cmd, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_LE_CREATE_CIS_CMD_HANDLER_FUN_BIT, int, p_cmd, opcode);

    int msg_status = KE_MSG_CONSUMED;
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
#if !(BLE_CIS_IN_ROM)
    // Initialize returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t cursor = 0;
    // Pointer to the CIS Channel information structure
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env = NULL;

    // Check if the Isochronous Channels (Host Support) feature bit is set
    if (!llm_le_feature_check(BLE_FEAT_CIS_HOST_SUPPORT))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // Check if command not already under execution
    else if (lli_ci_env.p_create_cmd != NULL)
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        for (cursor = 0 ; cursor < p_cmd->cis_count ; cursor++)
        {
            uint8_t role;

            // Retrieve CIS environment
            p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_cmd->params[cursor].cis_conhdl)];

            // Provided CIS does not exist
            if (p_cis_env == NULL)
            {
                status = CO_ERROR_UNKNOWN_CONNECTION_ID;
                break;
            }

            // check if CIS is configured for master role
            if (p_cis_env->role != ROLE_MASTER)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            // Retrieve CIG environment
            status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

            if (status != CO_ERROR_NO_ERROR)
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
                ASSERT_ERR(0);
                // Provided CIS does not exist
                break;
            }

            // Check that no negotiation is on-going
            if ((p_cis_env->state == CIS_CREATE_NEGO) || (p_cis_env->state == CIS_STOPPING))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            // Check that stream is not already enabled
            if (p_cis_env->state >= CIS_WAIT_EST)
            {
                status = CO_ERROR_CON_ALREADY_EXISTS;
                break;
            }

            // Keep the provided connection handle
            p_cis_env->link_id = BLE_CONHDL_TO_LINKID(p_cmd->params[cursor].acl_conhdl);

            // retrieve PHY used by CIS
            status = llc_role_get(p_cis_env->link_id, &role);
            if (status != CO_ERROR_NO_ERROR)
            {
                // Provided CIS does not exist
                break;
            }

            // check if role is master
            if (role != MASTER_ROLE)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            //If two different elements of the CIS_Connection_Handle arrayed parameter identify the same CIS
            for (uint8_t i = 0; i < cursor; i++)
            {
                if (p_cmd->params[i].cis_conhdl == p_cmd->params[cursor].cis_conhdl)
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
            }
            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }
        }
    }

    // Send the command complete event
    p_evt->status   = status;
    hci_send_2_host(p_evt);

    if (status == CO_ERROR_NO_ERROR)
    {
        // keep command to execute it later
        lli_ci_env.p_create_cmd = p_cmd;
        lli_ci_env.cursor = 0;
        msg_status = KE_MSG_NO_FREE;

        // Continue CIS creation
        lli_cis_create_cont();
    }
#else // BLE_CIS_IN_ROM
    p_evt->status   = CO_ERROR_COMMAND_DISALLOWED;
    hci_send_2_host(p_evt);
#endif // BLE_CIS_IN_ROM
    // Message can be consumed
    return (msg_status);
}
#endif //(BLE_CENTRAL)

int hci_disconnect_cis_cmd_handler_patch(struct hci_disconnect_cmd const *p_param, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_DISCONNECT_CIS_CMD_HANDLER_FUN_BIT, int, p_param, opcode);

    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, p_param->conhdl, opcode, hci_cmd_stat_event);

#if !(BLE_CIS_IN_ROM)
    // Command status
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_param->conhdl)];
    struct lli_cig_env *p_cig_env = NULL;
    struct lli_cis_param *p_cis_param = NULL;

    // Check that stream exists
    if (p_cis_env != NULL)
    {
        // Retrieve CIG environment
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_UNSPECIFIED_ERROR;
            ASSERT_ERR(0);
        }
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        uint8_t cnt;

        p_cis_param = &(p_cis_env->cis_param);

        for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
        {
            struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

            // Check if a create or terminate procedure is ongoing for another CIS within the same group
            if ((p_search_cis_env != NULL)
                    && (p_search_cis_env->grp_hdl == p_cig_env->group.hdl)
                    && (p_search_cis_env->cis_param.cis_id != p_cis_param->cis_id)
                    && ((p_search_cis_env->state == CIS_CREATE_NEGO) || (p_search_cis_env->state == CIS_STOPPING)))
            {
                break;
            }
        }

        // If another negotiation is on-going, or CIS is not in a suitable state at peripheral or at central
        if ((cnt < BLE_ACTIVITY_MAX)
                || ((p_cis_env->role == ROLE_SLAVE) && (p_cis_env->state != CIS_EST))
                || (p_cis_env->state < CIS_CREATE_NEGO)
                || (p_cis_env->state == CIS_STOPPING))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    }

    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    if (status == CO_ERROR_NO_ERROR)
    {
#if (BLE_CENTRAL)
        // If busy on CIS connection establishment
        if (p_cis_env->state == CIS_CREATE_NEGO)
        {
            // Attempt to cancel the LLCP, free resources, and inform host that CIS establishment is cancelled
            if (llc_cis_cancel(p_cis_env->link_id, CO_ERROR_OPERATION_CANCELED_BY_HOST) == CO_ERROR_NO_ERROR)
            {
                // Send HCI CIS Enable Complete Event to the Host
                lli_cis_established_evt_send(p_cig_env, p_cis_env, CO_ERROR_OPERATION_CANCELED_BY_HOST, p_cis_param->tx_rate, p_cis_param->rx_rate);

                // No procedure on-going, disconnect is complete
                lli_cis_disc_cmp_evt_send(p_cis_env, CO_ERROR_CON_TERM_BY_LOCAL_HOST);

                // Move state
                p_cis_env->state = CIS_EXISTS;

                // Free Host parameters
                if (lli_ci_env.p_create_cmd)
                {
                    ke_free(ke_param2msg(lli_ci_env.p_create_cmd));
                    lli_ci_env.p_create_cmd = NULL;
                }
            }
        }
        else
#endif //(BLE_CENTRAL)
        {
            // Provide the request to the LLC CIS module
            if (llc_cis_stop_req(p_cis_env->link_id, p_cis_env->act_id, p_param->reason) == CO_ERROR_NO_ERROR)
            {
                // Mark that a procedure is on-going on stream
                p_cis_env->state = CIS_STOPPING;

                // Stop the CIS driver
                lld_cis_stop(p_cis_env->act_id);

                // Keep the reason
                p_cis_env->reason = CO_ERROR_CON_TERM_BY_LOCAL_HOST;
            }
        }
    }
#else // BLE_CIS_IN_ROM
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;


    p_evt->status = status;
    hci_send_2_host(p_evt);
#endif // BLE_CIS_IN_ROM
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_remove_cig_cmd_handler_patch(struct hci_le_remove_cig_cmd const *p_param, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_LE_REMOVE_CIG_CMD_HANDLER_FUN_BIT, int, p_param, opcode);

    // Allocate the status event to be sent to the host
    struct hci_le_remove_cig_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_remove_cig_cmd_cmp_evt);
#if !(BLE_CIS_IN_ROM)
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t cnt;

    struct lli_cig_env *p_cig_env;

    p_cig_env = lli_cig_get(p_param->cig_id, ROLE_MASTER, BLE_INVALID_LINK_ID);

    for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
    {
        struct lli_cis_env *p_search_cis_env = lli_cis_env[cnt];

        // Check if CIS exists and has the same identifiers
        if ((p_search_cis_env != NULL)
                && (p_search_cis_env->grp_hdl == p_cig_env->group.hdl)
                && (p_search_cis_env->state >= CIS_CREATE_NEGO))
        {
            break;
        }
    }

    // Check that nb value is not higher than array size
    if (p_cig_env == NULL)
    {
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    else if (cnt < BLE_ACTIVITY_MAX)
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        while (!co_list_is_empty(&(p_cig_env->streams)))
        {
            // Pointer to the indicated CIS environment
            struct lli_cis_env *p_cis_env = (struct lli_cis_env *) co_list_pop_front(&(p_cig_env->streams));

            // Clean the stream environment
            lli_cis_cleanup(p_cis_env);
        }

        // Clean-up group environment
        lli_group_cleanup(p_cig_env->group.hdl);
    }
#else // BLE_CIS_IN_ROM

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#endif // BLE_CIS_IN_ROM
    // Send the command complete event
    p_evt->status = status;
    p_evt->cig_id = p_param->cig_id;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

#if (BLE_PERIPHERAL)
int hci_le_accept_cis_req_cmd_handler_patch(struct hci_le_accept_cis_req_cmd const *p_param, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_LE_ACCEPT_CIS_REQ_CMD_HANDLER_FUN_BIT, int, p_param, opcode);

    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, p_param->conhdl, opcode, hci_cmd_stat_event);

#if !(BLE_CIS_IN_ROM)
    // Command status
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_param->conhdl)];
    struct lli_cig_env *p_cig_env;

    // Check that stream exists
    if (p_cis_env != NULL)
    {
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        // Check if command reception is expected
        if (p_cis_env->state == CIS_WAIT_HOST)
        {
            // Provide host response to the LLC CIS module
            llc_cis_create_rsp(p_cis_env->link_id, CO_ERROR_NO_ERROR);

            // mark that host request has been performed
            p_cis_env->state = CIS_CREATE_NEGO;
        }
        else
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    }
#else // BLE_CIS_IN_ROM
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
#endif //BLE_CIS_IN_ROM
    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_reject_cis_req_cmd_handler_patch(struct hci_le_reject_cis_req_cmd *p_param, uint16_t opcode)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, HCI_LE_REJECT_CIS_REQ_CMD_HANDLER_FUN_BIT, int, p_param, opcode);

    // Allocate the status event to be sent to the host
    struct hci_basic_conhdl_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, p_param->conhdl, opcode, hci_basic_conhdl_cmd_cmp_evt);

#if !(BLE_CIS_IN_ROM)
    // Command status
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env = lli_cis_env[BLE_CISHDL_TO_ACTID(p_param->conhdl)];
    struct lli_cig_env *p_cig_env;

    // Check that stream exists
    if (p_cis_env != NULL)
    {
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        // Check if command reception is expected
        if (p_cis_env->state == CIS_WAIT_HOST)
        {
            // Adjust error code
            if (p_param->reason == CO_ERROR_NO_ERROR)
            {
                p_param->reason = CO_ERROR_OPERATION_CANCELED_BY_HOST;
            }

            // Provide host response to the LLC CIS module
            llc_cis_create_rsp(p_cis_env->link_id, p_param->reason);

            // Clean the stream environment
            lli_cis_cleanup(p_cis_env);

            // Remove CIS from CIG
            co_list_extract(&(p_cig_env->streams), &(p_cis_env->hdr));
            p_cig_env->nb_cis--;

            // Check if CIG can be removed
            if ((p_cig_env->nb_cis == 0))
            {
                // Clean-up the CIG
                lli_group_cleanup(p_cig_env->group.hdl);
            }
        }
        else
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    }

#else // BLE_CIS_IN_ROM
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
#endif // BLE_CIS_IN_ROM

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->conhdl = p_param->conhdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handles the CIS stop indication message.
 ****************************************************************************************
 */
int lld_cis_stop_ind_handler_patch(ke_msg_id_t const msgid,
                                   struct lld_cis_stop_ind *p_param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, LLD_CIS_STOP_IND_HANDLER_FUN_BIT, int, msgid, p_param, dest_id, src_id);

#if !(BLE_CIS_IN_ROM)
    do
    {
        // Pointer to the CIS Channel information structure
        struct lli_cis_env *p_cis_env = lli_cis_env[p_param->act_id];
        struct lli_cig_env *p_cig_env;
        // Reason
        uint8_t reason;

        if ((p_cis_env == NULL) || (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) != CO_ERROR_NO_ERROR))
        {
            // Provided channel does not exist, drop the message
            ASSERT_INFO(0, p_param->act_id, p_param->reason);
            break;
        }

        // If CIS terminated due to MIC failure, close the associated BLE link (and all other associated CISes)
        if (p_param->reason == CO_ERROR_TERMINATED_MIC_FAILURE)
        {
            // Check other
            llc_disconnect(p_cis_env->link_id, CO_ERROR_TERMINATED_MIC_FAILURE, true);
        }

        // CIS disconnection reason
        reason = (p_cis_env->state == CIS_STOPPING) ? p_cis_env->reason : p_param->reason;

        // Clean the connection handle
        p_cis_env->link_id = BLE_INVALID_LINK_ID;

        // Send HCI LE CIS Disconnection Complete Event
        lli_cis_disc_cmp_evt_send(p_cis_env, reason);

        p_cis_env->state = CIS_EXISTS;

        // On slave side, the CIS is considered removed and can no more be reused
        if (p_cis_env->role == ROLE_SLAVE)
        {
            // Check if datapath must be removed
            if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_RX]))
            {
                lli_cis_data_path_remove(p_cis_env->act_id, ISO_SEL_RX);
            }
            if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_TX]))
            {
                lli_cis_data_path_remove(p_cis_env->act_id, ISO_SEL_TX);
            }

            // Remove CIS from CIG
            co_list_extract(&(p_cig_env->streams), &(p_cis_env->hdr));
            p_cig_env->nb_cis--;

            // Clean the stream environment
            lli_cis_cleanup(p_cis_env);

            if (p_cig_env->nb_cis == 0)
            {
                lli_group_cleanup(p_cig_env->group.hdl);
            }
        }
    }
    while (0);
#endif // BLE_CIS_IN_ROM
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the CIS established indication message.
 ****************************************************************************************
 */
int lld_cis_estab_ind_handler_patch(ke_msg_id_t const msgid,
                                    struct lld_cis_estab_ind *p_param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, LLD_CIS_ESTAB_IND_HANDLER_FUN_BIT, int, msgid, p_param, dest_id, src_id);
#if !(BLE_CIS_IN_ROM)
    // Pointer to the CIS Channel information structure
    struct lli_cis_env *p_cis_env = lli_cis_env[p_param->act_id];
    struct lli_cig_env *p_cig_env;

    if ((p_cis_env != NULL) && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        struct lli_cis_param *p_cis_param = &(p_cis_env->cis_param);

        // Send HCI CIS Enable Complete Event to the Host
        lli_cis_established_evt_send(p_cig_env, p_cis_env, p_param->status,
                                     ((p_cis_env->role == ROLE_MASTER) ? p_cis_param->tx_rate : p_cis_param->rx_rate),
                                     ((p_cis_env->role == ROLE_MASTER) ? p_cis_param->rx_rate : p_cis_param->tx_rate));

        // Move the CIS state
        p_cis_env->state = (p_param->status == CO_ERROR_NO_ERROR) ? CIS_EST : CIS_EXISTS;

#if BLE_PWR_CTRL
        if (p_param->status == CO_ERROR_NO_ERROR)
        {
            uint8_t con_tx_rate;
            // Get connection tx rates
            if (CO_ERROR_NO_ERROR == lld_con_tx_rate_get(p_cis_env->link_id, &con_tx_rate))
            {
                // If the CIS tx rate differs, signal power change indication
                if (p_cis_param->tx_rate != con_tx_rate)
                {
                    struct lld_cis_pwr_change_ind *msg = KE_MSG_ALLOC(LLD_CIS_PWR_CHANGE_IND, KE_BUILD_ID(TASK_LLC, p_cis_env->link_id), TASK_NONE, lld_cis_pwr_change_ind);
                    msg->tx_rate = p_cis_param->tx_rate; // CIS transmit rate
                    msg->en = true; // enabled
                    ke_msg_send(msg);
                }
            }
        }
#endif // BLE_PWR_CTRL
    }
    else
    {
        ASSERT_INFO(0, 0, p_param->act_id);
    }
#endif // BLE_CIS_IN_ROM
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

void lli_ci_init_patch(bool is_reset)
{
    //FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLI_CI_PATCH_TYPE, LLI_CI_INIT_FUN_BIT, init_type);
#if !(BLE_CIS_IN_ROM)
    if (is_reset)
    {
        // Free CIS creation operation if exists
        if (lli_ci_env.p_create_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lli_ci_env.p_create_cmd));
        }

        // Clean all allocated CIS environments
        for (uint8_t i = 0; i < BLE_ACTIVITY_MAX; i++)
        {
            if (lli_cis_env[i] != NULL)
            {
                ke_free(lli_cis_env[i]);
            }
        }
    }

    memset(lli_cis_env, 0, sizeof(lli_cis_env));
    lli_ci_env.cb_start_evt = NULL;
    lli_ci_env.p_create_cmd = NULL;
#endif // BLE_CIS_IN_ROM
}

bool lli_cis_is_present_patch(uint8_t link_id)
{
    //FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, LLI_CIS_IS_PRESENT_FUN_BIT, bool, link_id);
#if !(BLE_CIS_IN_ROM)

    // Counter
    uint8_t cnt;

    for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
    {
        struct lli_cis_env *p_cis_env = lli_cis_env[cnt];

        if ((p_cis_env != NULL) && (p_cis_env->link_id == link_id))
        {
            break;
        }
    }

    // Return the allocated channel handle
    return (cnt < BLE_ACTIVITY_MAX);
#else // !(BLE_CIS_IN_ROM)
    return 0;
#endif // !(BLE_CIS_IN_ROM)
}

void lli_cis_link_stop_ind_patch(uint8_t link_id, uint8_t reason)
{
    //FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLI_CI_PATCH_TYPE, LLI_CIS_LINK_STOP_IND_FUN_BIT, link_id, reason);
#if !(BLE_CIS_IN_ROM)

    // Search for a CIS related to this ACL
    for (uint8_t act_id = 0; act_id < BLE_ACTIVITY_MAX; act_id++)
    {
        struct lli_cis_env *p_cis_env = lli_cis_env[act_id];

        // Check the CIS is on this ACL, and in a suitable state
        if ((p_cis_env != NULL) && (p_cis_env->link_id == link_id) && (p_cis_env->state >= CIS_WAIT_EST) && (p_cis_env->state <= CIS_EST))
        {
            // Disconnection reason
            p_cis_env->reason = reason;

            // Mark that a procedure is on-going on CIG
            p_cis_env->state = CIS_STOPPING;

            // Stop the CIS driver
            lld_cis_stop(p_cis_env->act_id);
        }
    }
#endif // !(BLE_CIS_IN_ROM)
}

uint8_t lli_cis_data_path_set_patch(uint8_t act_id, uint8_t direction, uint8_t data_path_type)
{
    //FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, LLI_CIS_DATA_PATH_SET_FUN_BIT, uint8_t, act_id, direction, data_path_type);
#if !(BLE_CIS_IN_ROM)

    uint8_t status = CO_ERROR_NO_ERROR;
    do
    {
        struct lli_cis_env *p_cis_env = lli_cis_env[act_id];
        struct lli_cig_env *p_cig_env = NULL;
        ASSERT_ERR(direction < ISO_SEL_MAX);

        // An error occurs, do not continue data-path loading
        if ((p_cis_env == NULL)
                || (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) != CO_ERROR_NO_ERROR))
        {
            status = CO_ERROR_UNKNOWN_CONNECTION_ID;
            break;
        }

        // Check BN
        {
            uint8_t bn = (direction == ISO_SEL_TX) ? p_cis_env->cis_param.tx_bn : p_cis_env->cis_param.rx_bn;
            if (bn == 0)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
        }

        // Check the SDU size in ISO test mode
#if (BLE_ISOGEN)
        if (data_path_type == ISO_DP_ISOGEN)
        {
            uint16_t max_sdu_size = (direction == ISO_SEL_TX) ? p_cis_env->cis_param.tx_max_sdu : p_cis_env->cis_param.rx_max_sdu;
            if (max_sdu_size < ISO_TEST_PKT_CNT_SIZE)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
        }
#endif //(BLE_ISOGEN)

        // Check the SDU size in ISOOHCI
#if (BLE_ISOOHCI)
        if ((data_path_type == ISO_DP_ISOOHCI) && (direction == ISO_SEL_TX) && (p_cis_env->cis_param.tx_max_sdu > BLE_HCI_ISO_IN_SDU_BUF_SIZE))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif //(BLE_ISOOHCI)

        // Check if the data path has already been set up
        if (!data_path_is_disabled(p_cis_env->p_dp[direction]))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Load data path
        p_cis_env->p_dp[direction] = data_path_itf_get(data_path_type, direction);

        // Check if the data path has been found
        if (data_path_is_disabled(p_cis_env->p_dp[direction]))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

#if (BLE_PERIPHERAL)
        // Check if waiting on HCI_LE_Accept_CIS_Request
        if ((p_cis_env->role == ROLE_SLAVE) && (p_cis_env->state == CIS_WAIT_HOST))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
#endif // (BLE_PERIPHERAL)

        // Check that Isochronous stream is enabled
        if (p_cis_env->state >= CIS_WAIT_EST)
        {
            // Update the data path to use
            status = lld_isoal_datapath_set(p_cis_env->act_id, direction, p_cis_env->p_dp[direction]);

            if (p_cis_env->p_dp[direction]->cb_local_sync != NULL)
            {
                // Enable the local synchronization on this CIS
                lld_cis_local_sync_en(p_cis_env->act_id, direction, p_cis_env->p_dp[direction]->cb_local_sync);
            }

#if (BLE_PERIPHERAL)
            if (p_cis_env->p_dp[direction]->cb_peer_sync != NULL)
            {
                lld_cis_peer_sync_en(p_cis_env->act_id, direction, p_cis_env->p_dp[direction]->cb_peer_sync);
            }
#endif // (BLE_PERIPHERAL)

#if (BLE_ISOGEN)
            // Special packet counter initialization for ISO transmit test mode / Unframed
            if ((data_path_type == ISO_DP_ISOGEN) && (p_cig_env->cig_param.framing == ISO_UNFRAMED_MODE))
            {
                GLOBAL_INT_DISABLE();

                uint32_t pld_cnt, sdu_cnt;

                // Get current payload counter
                lld_cis_pld_cnt_get(p_cis_env->act_id, direction, &pld_cnt);

                // Calculate the associated SDU packet counter
                if (direction == ISO_SEL_TX)
                {
                    sdu_cnt = pld_cnt / (p_cis_env->cis_param.tx_bn * (p_cig_env->cig_param.tx_sdu_interval / (p_cig_env->cig_param.iso_interval * 2 * SLOT_SIZE)));
                }
                else
                {
                    sdu_cnt = pld_cnt / (p_cis_env->cis_param.rx_bn * (p_cig_env->cig_param.rx_sdu_interval / (p_cig_env->cig_param.iso_interval * 2 * SLOT_SIZE)));
                }

                // Initialize the SDU counter
                isogen_sdu_cnt_set(p_cis_env->act_id, direction, sdu_cnt);

                GLOBAL_INT_RESTORE();
            }
#endif //(BLE_ISOGEN)
        }
    }
    while (0);

    return (status);
#else // !(BLE_CIS_IN_ROM)
    return CO_ERROR_COMMAND_DISALLOWED;
#endif // !(BLE_CIS_IN_ROM)
}

uint8_t lli_cis_data_path_remove_patch(uint8_t act_id, uint8_t direction)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, LLI_CIS_DATA_PATH_REMOVE_FUN_BIT, uint8_t, act_id, direction);
#if !(BLE_CIS_IN_ROM)
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    struct lli_cis_env *p_cis_env = lli_cis_env[act_id];

    do
    {
        struct lli_cig_env *p_cig_env = NULL;

        ASSERT_ERR(direction < ISO_SEL_MAX);

        // Check if CIS exists
        if (p_cis_env == NULL)
            break;

        // Check if data path is enabled
        if (data_path_is_disabled(p_cis_env->p_dp[direction]))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Get CIG environment
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

        if (status != CO_ERROR_NO_ERROR)
            break;

        // Check that Isochronous stream is enabled
        if (p_cis_env->state < CIS_WAIT_EST)
            break;

        if (p_cis_env->p_dp[direction]->cb_local_sync != NULL)
        {
            // Enable the local synchronization on this CIS
            lld_cis_local_sync_dis(act_id, direction, p_cis_env->p_dp[direction]->cb_local_sync);
        }

#if (BLE_PERIPHERAL)
        if (p_cis_env->p_dp[direction]->cb_peer_sync != NULL)
        {
            lld_cis_peer_sync_dis(act_id, direction, p_cis_env->p_dp[direction]->cb_peer_sync);
        }
#endif // (BLE_PERIPHERAL)

        // Remove the data path
        lld_isoal_datapath_remove(act_id, direction);

    }
    while (0);

    if (status == CO_ERROR_NO_ERROR)
    {
        // Unregister the data path
        p_cis_env->p_dp[direction] = data_path_itf_get(ISO_DP_DISABLE, direction);
    }

    return (status);
#else // !(BLE_CIS_IN_ROM)
    return CO_ERROR_COMMAND_DISALLOWED;
#endif // !(BLE_CIS_IN_ROM)
}

uint8_t lli_cis_stats_get_patch(uint8_t act_id, uint32_t *tx_unacked_packets, uint32_t *tx_flushed_packets, uint32_t *tx_last_subevent_packets,
                                uint32_t *retransmitted_packets, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets,
                                uint32_t *duplicate_packets)
{
    //FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLI_CI_PATCH_TYPE, LLI_CIS_STATS_GET_FUN_BIT, uint8_t, 8, act_id, tx_unacked_packets, tx_flushed_packets, tx_last_subevent_packets,
    //                      retransmitted_packets, crc_error_packets, rx_unreceived_packets, duplicate_packets);
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
#if !(BLE_CIS_IN_ROM)
    struct lli_cis_env *p_cis_env = lli_cis_env[act_id];

    if (p_cis_env != NULL)
    {
        status = lld_cis_stats_get(p_cis_env->act_id, tx_unacked_packets, tx_flushed_packets, tx_last_subevent_packets,
                                   retransmitted_packets, crc_error_packets, rx_unreceived_packets,
                                   duplicate_packets);

        // tx_last_subevent_packets is only valid when in the slave role
        if (p_cis_env->role == ROLE_MASTER)
        {
            *tx_last_subevent_packets = 0;
        }

    }
#endif // !(BLE_CIS_IN_ROM)
    return (status);
}

#endif //(BLE_CIS)

/// @} LLI
