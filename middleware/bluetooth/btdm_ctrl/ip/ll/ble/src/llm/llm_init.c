/**
 ****************************************************************************************
 *
 * @file llm_init.c
 *
 * @brief Definition of the functions used by the link layer manager
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#include "em_map_ble.h"
#if (BLE_CENTRAL)

#include <string.h>
#include "co_bt.h"          // BLE standard definitions
#include "co_utils.h"
#include "co_math.h"

#include "ke_mem.h"             // kernel memory
#include "ke_timer.h"       // kernel timer definitions

#include "sch_plan.h"           // Scheduling Planner

#include "llm.h"          // link layer manager definitions
#include "llc.h"          // link layer controller definitions
#include "lld.h"          // link layer driver definitions

#include "llm_int.h"      // link layer manager internal definitions

#include "rwip.h"

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT


/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_ADV_LEGACY_ITF)
int hci_le_create_con_cmd_handler(struct hci_le_create_con_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        // Start Initiating
        struct lld_init_params init_par;
        struct sch_plan_req_param req_param;
        uint8_t act_id = 0;
        uint16_t ce_len_min, ce_len_max;

        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
            break;

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;

        // Check if initiating is already started
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if (llm_env.act_info[act_id].state == LLM_INITIATING)
                break;
        }

        if (act_id < BLE_ACTIVITY_MAX_CFG)
            break;

        // Check if not already connected to this device
        if (((param->init_filt_policy == INIT_FILT_IGNORE_WLST)) && llm_is_dev_connected(&param->peer_addr, param->peer_addr_type))
        {
            status = CO_ERROR_CON_ALREADY_EXISTS;
            break;
        }

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Parameter checking
        if ((param->scan_window > param->scan_intv)  || (param->con_intv_min > param->con_intv_max) ||
                (param->scan_window > SCAN_WINDOW_MAX)   || (param->scan_window < SCAN_WINDOW_MIN)      ||
                (param->scan_intv > SCAN_INTERVAL_MAX)   || (param->scan_intv < SCAN_INTERVAL_MIN)      ||
                (param->ce_len_min > param->ce_len_max)  || (param->superv_to > CON_SUP_TO_MAX)         ||
                (param->con_intv_min < CON_INTERVAL_MIN) || (param->con_intv_max > CON_INTERVAL_MAX)    ||
                (param->superv_to < CON_SUP_TO_MIN)      || (param->con_latency > CON_LATENCY_MAX)      ||
                ((param->peer_addr_type > ADDR_RPA_OR_RAND) || (param->own_addr_type > ADDR_RPA_OR_RAND)) ||
                // The Supervision_Timeout parameter defines the link supervision timeout for the connection. The
                // Supervision_Timeout in milliseconds shall be larger than (1 + Conn_Latency) * Conn_Interval_Max * 2,
                // where Conn_Interval_Max is given in milliseconds. (See [Vol 6] Part B, Section 4.5.2).
                // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
                // (hci_sup_to * 10) <= ((1+latency)* interval*1.25*2)
                //to simplify computation and remove floating point we refactor everything by 4/10.
                // (hci_sup_to * 4) <= ((1+latency) * interval
                (((uint32_t)param->superv_to << 2) <= ((1 + (uint32_t)param->con_latency) * (uint32_t)param->con_intv_max)))
        {
            break;
        }

        // Check if the local address type is static private and no static private address is available
        if (((param->own_addr_type & ADDR_MASK) == ADDR_RAND)
                && (co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr) == true))
        {
            break;
        }

        // Allocate link identifier
        status = llm_activity_free_get(&act_id);

        // Check if link identifier found
        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Minimum_CE_Length and Maximum_CE_Length are informative parameters. Should not exceed connection interval
        ce_len_min = co_min(param->ce_len_min, 2 * param->con_intv_min);
        ce_len_min = co_max(ce_len_min, 2); // Reserve at least two slots
        ce_len_max = co_min(param->ce_len_max, 2 * param->con_intv_max);
        ce_len_max = co_max(ce_len_max, 2); // Reserve at least two slots

        // Input parameters
        req_param.interval_max    = param->con_intv_max * 4;
        req_param.interval_min    = param->con_intv_min * 4;
        req_param.duration_min    = ce_len_min * 2;
        req_param.duration_max    = ce_len_max * 2;
        req_param.offset_min      = 0;
        req_param.offset_max      = req_param.interval_max - 1;
        req_param.margin          = 1;
        req_param.pref_period     = 0;
        req_param.conhdl          = BLE_LINKID_TO_CONHDL(act_id);
        req_param.conhdl_ref      = req_param.conhdl;

        if (sch_plan_req(&req_param) != SCH_PLAN_ERROR_OK)
        {
            status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
            break;
        }
        ASSERT_ERR(llm_env.act_info[act_id].host_params == NULL);

        // Allocate an extended HCI command for storing parameters
        llm_env.act_info[act_id].host_params = KE_MSG_ALLOC(0, 0, 0, hci_le_ext_create_con_cmd);

        // Convert legacy HCI command to extended format
        if (llm_env.act_info[act_id].host_params != NULL)
        {
            struct hci_le_ext_create_con_cmd *ext_param = (struct hci_le_ext_create_con_cmd *) llm_env.act_info[act_id].host_params;
            ext_param->init_phys            = PHY_1MBPS_BIT;
            ext_param->phy[0].scan_interval = param->scan_intv;
            ext_param->phy[0].scan_window   = param->scan_window;
            ext_param->init_filter_policy   = param->init_filt_policy;
            ext_param->own_addr_type        = param->own_addr_type;
            ext_param->peer_addr_type       = param->peer_addr_type;
            ext_param->peer_addr            = param->peer_addr;
            ext_param->phy[0].con_intv_min  = param->con_intv_min;
            ext_param->phy[0].con_intv_max  = param->con_intv_max;
            ext_param->phy[0].con_latency   = param->con_latency;
            ext_param->phy[0].superv_to     = param->superv_to;
            ext_param->phy[0].ce_len_min    = ce_len_min;
            ext_param->phy[0].ce_len_max    = ce_len_max;
        }

        // Assign the allocated activity identifier to initiating
        llm_env.act_info[act_id].state = LLM_INITIATING;

        status = CO_ERROR_NO_ERROR;

        // Create the connection on this link ID
        memcpy(&init_par.peer_addr.addr[0], &param->peer_addr.addr[0], BD_ADDR_LEN);

        // Start with all channels marked as enabled (unknown or good)
        memset(&init_par.ch_map.map[0], 0xFF, LE_CH_MAP_LEN);
        init_par.ch_map.map[LE_CH_MAP_LEN - 1] &= 0x1F;

        init_par.init_phys      = PHY_1MBPS_BIT;
        init_par.phy[0].intv    = param->scan_intv;
        init_par.phy[0].win     = param->scan_window;
        init_par.phy[0].con_intv    = req_param.interval / 4;
        init_par.phy[0].con_offset  = req_param.offset_min;
        init_par.phy[0].con_latency = param->con_latency;
        init_par.phy[0].superv_to   = param->superv_to;
        init_par.act_id         = act_id;
        init_par.own_addr_type  = param->own_addr_type;
        init_par.peer_addr_type = param->peer_addr_type;
        init_par.filter_policy  = param->init_filt_policy;
        init_par.ext_init = false;
        init_par.addr_resolution_en = llm_env.addr_resolution_en;

        // Assign the local BD address, depending on address type
        switch (param->own_addr_type)
        {
        case  ADDR_PUBLIC:
        case  ADDR_RPA_OR_PUBLIC:
        {
            // Get local public address
            init_par.own_addr = llm_env.local_pub_addr;
        }
        break;
        case  ADDR_RAND:
        case  ADDR_RPA_OR_RAND:
        {
            // Get local random address
            init_par.own_addr = llm_env.local_rand_addr;
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, param->own_addr_type, 0);
        }
        break;
        }

        status = lld_init_start(&init_par);

        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_HARDWARE_FAILURE;
            break;
        }
    }
    while (0);

    // Send CS event
    llm_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_ADV_LEGACY_ITF)

int hci_le_ext_create_con_cmd_handler(struct hci_le_ext_create_con_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        // Start Initiating
        struct lld_init_params init_par;
        struct sch_plan_req_param req_param[MAX_INIT_PHYS];
        uint8_t act_id = 0;
        uint8_t num_init_phys = 0;
        int i;

        memset(req_param, 0, sizeof(req_param));

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Check if initiating is already started
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if (llm_env.act_info[act_id].state == LLM_INITIATING)
                break;
        }

        if (act_id < BLE_ACTIVITY_MAX_CFG)
            break;

        // Check if not already connected to this device
        if (((param->init_filter_policy == INIT_FILT_IGNORE_WLST)) && llm_is_dev_connected(&param->peer_addr, param->peer_addr_type))
        {
            status = CO_ERROR_CON_ALREADY_EXISTS;
            break;
        }

#if !(BLE_PHY_CODED_SUPPORT)
        if (PHY_CODED_BIT & param->init_phys)
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif

        // If at least one of the RFU bits is set
        if (param->init_phys > (PHY_1MBPS_BIT | PHY_2MBPS_BIT | PHY_CODED_BIT))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Check provided parameters: init_phys
        if (~(PHY_1MBPS_BIT | PHY_2MBPS_BIT | PHY_CODED_BIT) & param->init_phys)
        {
            break;
        }

        num_init_phys = NB_ONE_BITS(param->init_phys);

        // Check provided parameters: scan_window, scan_intv
        for (i = 0; i < num_init_phys; i++)
        {
            //values for the LE 2M PHY are ignored
            if ((PHY_2MBPS_BIT & param->init_phys) && (i == (PHY_1MBPS_BIT & param->init_phys)))
            {
                continue;
            }

            if ((param->phy[i].scan_window > param->phy[i].scan_interval) ||
                    (param->phy[i].scan_window < EXT_SCAN_WINDOW_MIN) || (param->phy[i].scan_interval < EXT_SCAN_INTERVAL_MIN))
            {
                break;
            }
        }

        if (i < num_init_phys)
            break;


        // Parameter checking
        for (i = 0; i < num_init_phys; i++)
        {
            if ((param->phy[i].con_intv_min > param->phy[i].con_intv_max) ||
                    (param->phy[i].ce_len_min > param->phy[i].ce_len_max) || (param->phy[i].superv_to > CON_SUP_TO_MAX) ||
                    (param->phy[i].con_intv_min < CON_INTERVAL_MIN) || (param->phy[i].con_intv_max > CON_INTERVAL_MAX) ||
                    (param->phy[i].superv_to < CON_SUP_TO_MIN) || (param->phy[i].con_latency > CON_LATENCY_MAX) ||
                    ((param->phy[i].superv_to * 10) < (((1 + param->phy[i].con_latency) * param->phy[i].con_intv_max * 5 + 1) >> 1)))
            {
                break;
            }
        }

        if (i < num_init_phys)
            break;

        // Parameter checking
        if ((param->peer_addr_type > ADDR_MASK) || (param->own_addr_type > ADDR_RPA_OR_RAND))
        {
            break;
        }

        // Check if the local address type is static private and no static private address is available
        if (((param->own_addr_type & ADDR_MASK) == ADDR_RAND)
                && (co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr) == true))
        {
            break;
        }

        // Allocate link identifier
        status = llm_activity_free_get(&act_id);

        // Check if link identifier found
        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Derive parameters
        for (i = 0; i < num_init_phys; i++)
        {
            uint16_t ce_len_min, ce_len_max;

            // Minimum_CE_Length and Maximum_CE_Length are informative parameters. Should not exceed connection interval
            ce_len_min = co_min(param->phy[i].ce_len_min, 2 * param->phy[i].con_intv_min);
            ce_len_min = co_max(ce_len_min, 2); // Reserve at least two slots
            ce_len_max = co_min(param->phy[i].ce_len_max, 2 * param->phy[i].con_intv_max);
            ce_len_max = co_max(ce_len_max, ce_len_min);

            // Input parameters
            req_param[i].interval_min    = param->phy[i].con_intv_min * 4;
            req_param[i].interval_max    = param->phy[i].con_intv_max * 4;
            req_param[i].duration_min    = ce_len_min * 2;
            req_param[i].duration_max    = ce_len_max * 2;
            req_param[i].offset_min      = 0;
            req_param[i].offset_max      = req_param[i].interval_max - 1;
            req_param[i].margin          = 1;
            req_param[i].pref_period     = 0;
            req_param[i].conhdl          = BLE_LINKID_TO_CONHDL(act_id);
            req_param[i].conhdl_ref      = req_param[i].conhdl;

            if (sch_plan_req(&req_param[i]) != SCH_PLAN_ERROR_OK)
            {
                status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
                break;
            }
        }

        if (i < num_init_phys)
        {
            status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
            break;
        }


        ASSERT_ERR(llm_env.act_info[act_id].host_params == NULL);

        // Keep pointer to HCI command
        llm_env.act_info[act_id].host_params = param;
        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        // Assign the allocated activity identifier to initiating
        llm_env.act_info[act_id].state = LLM_INITIATING;

        // Create the connection on this link ID
        memcpy(&init_par.peer_addr.addr[0], &param->peer_addr.addr[0], BD_ADDR_LEN);

        // Start with all channels marked as enabled (unknown or good)
        memset(&init_par.ch_map.map[0], 0xFF, LE_CH_MAP_LEN);
        init_par.ch_map.map[LE_CH_MAP_LEN - 1] &= 0x1F;

        init_par.init_phys = param->init_phys;

        for (i = 0; i < num_init_phys; i++)
        {
            init_par.phy[i].intv = param->phy[i].scan_interval;
            init_par.phy[i].win = param->phy[i].scan_window;
            init_par.phy[i].con_intv = req_param[i].interval / 4;
            init_par.phy[i].con_offset = req_param[i].offset_min;
            init_par.phy[i].con_latency = param->phy[i].con_latency;
            init_par.phy[i].superv_to = param->phy[i].superv_to;
        }

        /*
         * Adjustment for multiple scans. The Scan_Interval[i] and Scan_Window[i] parameters are recommendations.
         * The actual values used are implementation specific. LE 2M PHY values ignored. HCI:7.8.66.
         */
        if ((init_par.init_phys & (PHY_1MBPS_BIT | PHY_CODED_BIT)) == (PHY_1MBPS_BIT | PHY_CODED_BIT))
        {
            struct lld_init_phy_params *initpar_c = &init_par.phy[num_init_phys - 1];
            struct lld_init_phy_params *initpar_1m = &init_par.phy[0];

            uint32_t agg_win = initpar_1m->win + initpar_c->win;

            if (initpar_1m->intv < agg_win)
            {
                initpar_1m->intv = agg_win;
            }

            if (initpar_c->intv < agg_win)
            {
                initpar_c->intv = agg_win;
            }
        }

        init_par.act_id         = act_id;
        init_par.own_addr_type  = param->own_addr_type;
        init_par.peer_addr_type = param->peer_addr_type;
        init_par.filter_policy  = param->init_filter_policy;
        init_par.ext_init       = true;
        init_par.addr_resolution_en = llm_env.addr_resolution_en;

        // Assign the local BD address, depending on address type
        switch (param->own_addr_type)
        {
        case  ADDR_PUBLIC:
        case  ADDR_RPA_OR_PUBLIC:
        {
            // Get local public address
            init_par.own_addr = llm_env.local_pub_addr;
        }
        break;
        case  ADDR_RAND:
        case  ADDR_RPA_OR_RAND:
        {
            // Get local random address
            init_par.own_addr = llm_env.local_rand_addr;
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, param->own_addr_type, 0);
        }
        break;
        }

        status = lld_init_start(&init_par);

        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_HARDWARE_FAILURE;
            break;
        }
    }
    while (0);

    // Send CS event
    llm_cmd_stat_send(opcode, status);

    return (ret_status);
}

int hci_le_create_con_cancel_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t act_id = 0;

    // Find link identifier in initiating state
    for (act_id = 0 ; act_id < BLE_ACTIVITY_MAX_CFG ; act_id++)
    {
        if (llm_env.act_info[act_id].state == LLM_INITIATING)
            break;
    }

    // Check if link identifier found
    if (act_id < BLE_ACTIVITY_MAX_CFG)
    {
        ASSERT_ERR(llm_env.act_info[act_id].host_params != NULL);

        // Stop Initiating
        uint8_t status = lld_init_stop();

        if (status == CO_ERROR_COMMAND_DISALLOWED)
        {
            /*
             * LLM is waiting for initiating End indication, whereas LLD has finished Initiating ...
             * => Re-post message in order to re-process after initiating End indication
             */
            ke_msg_forward(param, TASK_LLM, opcode);

            return KE_MSG_NO_FREE;
        }

        // Free HCI command
        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
        llm_env.act_info[act_id].host_params = NULL;
    }
    else
    {
        // Send CC event
        llm_cmd_cmp_send(opcode, CO_ERROR_COMMAND_DISALLOWED);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_init_end_ind, struct lld_init_end_ind)
{
    uint8_t act_id = param->act_id;
    bool connected = param->connected;

    ASSERT_INFO(act_id < BLE_ACTIVITY_MAX_CFG, act_id, 0);

    // Check if Initiating procedure ongoing
    if (llm_env.act_info[act_id].state == LLM_INITIATING)
    {
        // Point Host parameters
        struct hci_le_ext_create_con_cmd *ext_param = (struct hci_le_ext_create_con_cmd *) llm_env.act_info[act_id].host_params;

        if (llm_env.act_info[act_id].host_params != NULL)
        {
            if (connected)
            {
                // Start LLC for connection establishment
                struct llc_init_parameters llc_par;
                uint8_t position;

                llc_par.interval        = param->interval;
                llc_par.latency         = param->latency;
                llc_par.timeout         = param->timeout;
                memcpy(&llc_par.ch_map.map[0], &param->ch_map.map[0], LE_CH_MAP_LEN);
                llc_par.master_sca      = param->master_sca;
                llc_par.role            = MASTER_ROLE;
                llc_par.rate            = param->rate;
                llc_par.suggested_max_tx_octets = llm_env.suggested_max_tx_octets;
                llc_par.suggested_max_tx_time   = llm_env.suggested_max_tx_time;
#if (EAVESDROPPING_SUPPORT)
                llc_par.suggested_max_rx_octets = llm_env.suggested_max_rx_octets;
                llc_par.suggested_max_rx_time   = llm_env.suggested_max_rx_time;
#endif //(EAVESDROPPING_SUPPORT)
#if (BT_53 && BLE_PERIPHERAL)
                llc_par.ch_class_rep_intv       = llm_env.ch_class_rep_intv;
#endif // (BT_53 && BLE_PERIPHERAL)
                llc_par.tx_phys                 = llm_env.tx_phys;
                llc_par.rx_phys                 = llm_env.rx_phys;
#if (BLE_PAST)
                llc_par.past_mode               = llm_env.past_mode;
                llc_par.past_skip               = llm_env.past_skip;
                llc_par.past_sync_to            = llm_env.past_sync_to;
                llc_par.past_cte_type           = llm_env.past_cte_type;
#endif // (BLE_PAST)
#if (BT_53)
                llc_par.subrate_min = llm_env.subrate_info.subrate_min;
                llc_par.subrate_max = llm_env.subrate_info.subrate_max;
                llc_par.subrate_max_latency = llm_env.subrate_info.max_latency;
                llc_par.subrate_cont_num = llm_env.subrate_info.cont_num;
                llc_par.subrate_timeout = llm_env.subrate_info.timeout;
#endif // (BT_53)

                llc_start(act_id, &llc_par);

                // Enable channel map monitoring
                llm_ch_map_timer_enable();

                {
                    // Register activity into planner
                    struct sch_plan_elt_tag *new_plan_elt = &llm_env.act_info[act_id].plan_elt;

                    new_plan_elt->offset = param->con_offset;
                    new_plan_elt->interval = 4 * param->interval;
                    new_plan_elt->duration_min = co_max(ext_param->phy[param->phy_idx].ce_len_min * 2, 4); // Reserve at least two slots
                    new_plan_elt->duration_max = co_max(ext_param->phy[param->phy_idx].ce_len_max * 2, new_plan_elt->duration_min);
                    new_plan_elt->conhdl = BLE_LINKID_TO_CONHDL(act_id);
                    new_plan_elt->conhdl_ref = new_plan_elt->conhdl;
                    new_plan_elt->margin = 1;
                    if (llm_env.con_move_en)
                    {
                        new_plan_elt->cb_move = &llc_con_move_cbk;
                        new_plan_elt->mobility = SCH_PLAN_MB_LVL_4;
                    }
                    else
                    {
                        new_plan_elt->cb_move = NULL;
                        new_plan_elt->mobility = SCH_PLAN_MB_LVL_0;
                    }
                    sch_plan_set(new_plan_elt);
                }

                // Update link identifier
                llm_env.act_info[act_id].info.con.bd_addr = param->peer_id_addr;
                llm_env.act_info[act_id].info.con.addr_type = param->peer_addr_type & ADDR_MASK;
                llm_env.act_info[act_id].state = LLM_CONNECTED;
                llm_env.act_info[act_id].info.con.role = ROLE_MASTER;

                // Check if the device is in the device list
                position = llm_dev_list_search(&param->peer_id_addr, param->peer_addr_type);

                // Remove device from EM white list, if it is in white list
                if ((position < BLE_WHITELIST_MAX) && GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
                {
                    lld_white_list_rem(position, (struct bd_addr *) &param->peer_id_addr, param->peer_addr_type);
                }

                // Register linkID / BD address at HCI level
                hci_ble_conhdl_register(act_id);
            }
            else
            {
                // Free activity identifier
                llm_env.act_info[act_id].state = LLM_FREE;
            }
        }
        else
        {
            // Create connection has been canceled by the Host, report connection cancel completion
            llm_cmd_cmp_send(HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE, CO_ERROR_NO_ERROR);

            // Free activity identifier
            llm_env.act_info[act_id].state = LLM_FREE;

            // If a connection was created during the cancel procedure, ignore the new connection
            connected = false;
        }

        // Report HCI_LE_Connection_Complete or HCI_LE_Enhanced_Connection_Complete event
        if (llm_le_evt_mask_check(LE_EVT_MASK_ENH_CON_CMP_EVT_BIT))
        {
            struct hci_le_enh_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_enh_con_cmp_evt);
            memset(evt, 0, sizeof(struct hci_le_enh_con_cmp_evt));
            evt->subcode = HCI_LE_ENH_CON_CMP_EVT_SUBCODE;
            evt->status = (connected) ? CO_ERROR_NO_ERROR : CO_ERROR_UNKNOWN_CONNECTION_ID;
            if (connected)
            {
                evt->role = MASTER_ROLE;
                evt->conhdl = BLE_LINKID_TO_CONHDL(act_id);

                evt->peer_addr = param->peer_id_addr;

                if (ext_param->own_addr_type & ADDR_RPA_MASK)
                {
                    memcpy(&evt->loc_rslv_priv_addr.addr[0], &param->local_rpa.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    memset(&evt->loc_rslv_priv_addr.addr[0], 0, BD_ADDR_LEN);
                }
                if (param->peer_addr_type & ADDR_RPA_MASK)
                {
                    memcpy(&evt->peer_rslv_priv_addr.addr[0], &param->peer_rpa.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    memset(&evt->peer_rslv_priv_addr.addr[0], 0, BD_ADDR_LEN);
                }
                evt->peer_addr_type = param->peer_addr_type;
                evt->con_interval   = param->interval;
                evt->con_latency    = param->latency;
                evt->sup_to         = param->timeout;
                evt->clk_accuracy   = param->master_sca;
            }
            hci_send_2_host(evt);
        }
        else
        {
            struct hci_le_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_con_cmp_evt);
            memset(evt, 0, sizeof(struct hci_le_con_cmp_evt));
            evt->subcode = HCI_LE_CON_CMP_EVT_SUBCODE;
            evt->status = (connected) ? CO_ERROR_NO_ERROR : CO_ERROR_UNKNOWN_CONNECTION_ID;
            if (connected)
            {
                evt->role = MASTER_ROLE;
                evt->conhdl = BLE_LINKID_TO_CONHDL(act_id);
                if (param->peer_addr_type & ADDR_RPA_MASK)
                {
                    evt->peer_addr_type = ADDR_RAND;
                    memcpy(&evt->peer_addr.addr[0], &param->peer_rpa.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    evt->peer_addr_type = param->peer_addr_type & ADDR_MASK;
                    memcpy(&evt->peer_addr.addr[0], &param->peer_id_addr.addr[0], BD_ADDR_LEN);
                }
                evt->con_interval   = param->interval;
                evt->con_latency    = param->latency;
                evt->sup_to         = param->timeout;
                evt->clk_accuracy   = param->master_sca;
            }
            hci_send_2_host(evt);
        }

        if (llm_env.act_info[act_id].host_params != NULL)
        {
            // Free HCI command
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
            llm_env.act_info[act_id].host_params = NULL;
        }

        // Send the LE Channel Selection Algorithm event
        if (connected)
        {
            struct hci_le_ch_sel_algo_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_ch_sel_algo_evt);
            event->subcode = HCI_LE_CH_SEL_ALGO_EVT_SUBCODE;
            event->conhdl = BLE_LINKID_TO_CONHDL(act_id);
            event->ch_sel_algo = param->ch_sel_2;
            hci_send_2_host(event);
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, act_id, llm_env.act_info[param->act_id].state);
    }

    return (KE_MSG_CONSUMED);
}


#endif //(BLE_CENTRAL)

/// @} LLM
