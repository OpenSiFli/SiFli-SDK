/**
 ****************************************************************************************
 *
 * @file llm.c
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

#include "rwip_config.h"  // stack configuration

#include <string.h>
#include "co_bt.h"        // BLE standard definitions
#include "co_math.h"
#include "co_utils.h"
#include "ble_util.h"     // BLE utility functions

#include "ke_mem.h"       // kernel memory
#include "ke_timer.h"
#include "ke_task.h"      // kernel task definition

#include "llm.h"          // link layer manager definitions
#include "llc.h"          // link layer driver definitions
#if ((BLE_BIS) && (BLE_BROADCASTER))
    #include "lli.h"          // link Layer ISO definitions
#endif // ((BLE_BIS) && (BLE_BROADCASTER))
#include "lld.h"          // link layer driver definitions

#include "llm_int.h"      // link layer manager internal definitions

#include "rwip.h"

#if HCI_PRESENT
    #include "hci.h"          // host controller interface
#endif //HCI_PRESENT
#include "em_map_ble.h"
#include "rom_config.h"
/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */
/// Local LE supported features
__STATIC const struct le_features llm_default_le_feats =
{
    {
        BLE_FEATURES_BYTE0, BLE_FEATURES_BYTE1, BLE_FEATURES_BYTE2, BLE_FEATURES_BYTE3,
        BLE_FEATURES_BYTE4, BLE_FEATURES_BYTE5, BLE_FEATURES_BYTE6, BLE_FEATURES_BYTE7
    }
};

/// LE default event mask
__STATIC const struct evt_mask le_default_evt_mask = {BLE_EVT_MASK};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLM environment variable
struct llm_env_tag llm_env;


/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t llm_per_adv_chain_dur(uint16_t len, uint8_t phy)
{
    ASSERT_ERR(len <= BLE_CFG_MAX_ADV_DATA_LEN);

    uint8_t pkt_nb = len / 240 + 1;
    uint8_t frag_size = (pkt_nb > 1) ? BLE_ADV_FRAG_SIZE_TX + 1 : co_min(len + 15, BLE_ADV_FRAG_SIZE_TX + 1);
    uint32_t dur_hus = pkt_nb * 2 * ble_util_pkt_dur_in_us(frag_size, phy - 1) + (pkt_nb - 1) * 2 * BLE_AFS_DUR;

    return (dur_hus / HALF_SLOT_SIZE + 1);
}

void llm_cmd_cmp_send(uint16_t opcode, uint8_t status)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}

void llm_cmd_stat_send(uint16_t opcode, uint8_t status)
{
    // allocate the status event message
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL || BLE_OBSERVER)
bool llm_is_dev_connected(struct bd_addr const *peer_addr, uint8_t peer_addr_type)
{
    uint8_t act_id = 0;

    // Check all entries of the connection information table
    for (act_id = 0 ; act_id < BLE_ACTIVITY_MAX_CFG ; act_id++)
    {
        // Check if there is a connection
        if (llm_env.act_info[act_id].state == LLM_CONNECTED)
        {
            // Compare BD address and BD address type
            // Mask the RPA bit from stored peer address type to Public or Random
            if (co_bdaddr_compare(peer_addr, &llm_env.act_info[act_id].info.con.bd_addr) && ((peer_addr_type & ADDR_MASK) == (llm_env.act_info[act_id].info.con.addr_type & ADDR_MASK)))
                break;
        }
    }

    return (act_id < BLE_ACTIVITY_MAX_CFG);
}
#endif //(BLE_CENTRAL || BLE_PERIPHERAL || BLE_OBSERVER)

#if (EAVESDROPPING_SUPPORT)
uint8_t llm_nb_active_link_get(void)
{
    uint8_t nb_active_link = 0;
    for (uint16_t act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        // Check if there is a connection
        if (llm_env.act_info[act_id].state == LLM_CONNECTED)
        {
            nb_active_link++;
        }
    }

    return (nb_active_link);
}
#endif //(EAVESDROPPING_SUPPORT)

#if (BLE_CENTRAL || BLE_OBSERVER)
bool llm_is_dev_synced(struct bd_addr const *peer_addr, uint8_t peer_addr_type, uint8_t adv_sid)
{
    uint8_t act_id = 0;

    // Check all entries of the connection information table
    for (act_id = 0 ; act_id < BLE_ACTIVITY_MAX_CFG ; act_id++)
    {
        // Check if there is a connection
        if (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCED
                || llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING
#if (BLE_PAST)
                || llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING_FROM_SYNC_TRANSF
#endif // (BLE_PAST)
           )
        {
            // Compare BD address, BD address type and ADV SID
            // Mask the RPA bit from stored peer address type to Public or Random
            if (co_bdaddr_compare(peer_addr, &llm_env.act_info[act_id].info.sync.bd_addr)
                    && ((peer_addr_type & ADDR_MASK) == (llm_env.act_info[act_id].info.sync.addr_type & ADDR_MASK))
                    && (adv_sid == llm_env.act_info[act_id].info.sync.adv_sid))
                break;
        }
    }

    return (act_id < BLE_ACTIVITY_MAX_CFG);
}
#endif //(BLE_CENTRAL || BLE_OBSERVER)

uint8_t llm_dev_list_empty_entry(void)
{
    uint8_t position;

    // Parse the list linearly
    for (position = 0; position < BLE_WHITELIST_MAX ; position++)
    {
        // Check if list entry is used
        if (!GETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED))
            break;
    }

    return position;
}

uint8_t llm_dev_list_search(const struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t position;

    // Parse the list linearly
    for (position = 0; position < BLE_WHITELIST_MAX ; position++)
    {
        // Check if list entry is used
        if (!GETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED))
            continue;

        // Compare address type
        if (llm_env.dev_list[position].addr_type != bd_addr_type)
            continue;

        // Compare BD address
        if (!co_bdaddr_compare(&llm_env.dev_list[position].addr, bd_addr))
            continue;

        break;
    }

    return position;
}

uint8_t llm_ral_search(const struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t position;

    for (position = 0 ; position < BLE_RAL_MAX ; position++)
    {
        if (llm_env.ral[position].addr_type != 0xFF)
        {
            // Compare the 2 BD addresses
            if (((bd_addr_type & ADDR_RAND) == llm_env.ral[position].addr_type) && (co_bdaddr_compare(&llm_env.ral[position].bd_addr, bd_addr)))
                break;
        }
    }

    return position;
}

bool llm_ral_is_empty(void)
{
    uint8_t position;

    // Parse the resolving list linearly
    for (position = 0; position < BLE_RAL_MAX ; position++)
    {
        // If entry is valid
        if (llm_env.ral[position].addr_type != 0xFF)
            break;
    }

    return (position == BLE_RAL_MAX);
}

#if (BLE_CENTRAL || BLE_BROADCASTER)
void llm_ch_map_compute(struct le_ch_map *ch_map)
{
    // Get Host channel classification
    struct le_ch_map *host_ch_class = &llm_env.ch_map_info.host_ch_class;

    // Get local channel map based on assessment
    rwip_ch_map_ble_get(ch_map);

    // Combine channel map with host classification
    for (int i = 0 ; i < BLE_DATA_CHANNEL_NB ; i++)
    {
        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;

        // Host indicates the channel as bad
        if ((((host_ch_class->map[byte_idx] >> bit_pos) & 0x1) == 0x00) || (((ch_map->map[byte_idx] >> bit_pos) & 0x1) == 0x00))
        {
            ch_map->map[byte_idx] &= ~(1 << bit_pos);
        }
    }

    // If the channel map does not contain enough enabled channel, activate channels in order to have
    // a minimum of DATA_CHANNEL_USED_NB_MIN channels activated.
    llm_fill_ch_map(ch_map);
}

void llm_fill_ch_map(struct le_ch_map *ch_map)
{
    uint8_t nbchgood;
    // Count number of good channels
    nbchgood = ble_util_nb_good_channels(ch_map);

    // Check if the map has a sufficient number of used channels
    if (nbchgood < DATA_CHANNEL_USED_NB_MIN)
    {
        rwip_ch_map_ble_fill((DATA_CHANNEL_USED_NB_MIN - nbchgood), &llm_env.ch_map_info.host_ch_class, ch_map);
    }
}

void llm_ch_map_update(void)
{
#if ((BLE_BIS) && (BLE_BROADCASTER))
    bool big_ch_map_update = false;
#endif // ((BLE_BIS) && (BLE_BROADCASTER))
    bool ch_map_timer = false;

    // Current channel map
    struct le_ch_map ch_map;

    // Compute channel map
    llm_ch_map_compute(&ch_map);

#if (BLE_BROADCASTER)
    // Set the secondary advertising channel map.
    lld_sec_adv_ch_map_set(&ch_map);
#endif // (BLE_BROADCASTER)

    // Search for master active link(s), BIG or periodic advertising
    for (int i = 0; i < BLE_ACTIVITY_MAX_CFG; i++)
    {
#if (BLE_CH_SCAN_SUPPORT)
        if (llm_env.act_info[i].state == LLM_CH_SCAN_EN)
        {
            struct le_ch_class ch_class;

            // Compute channel classification
            llm_ch_class_compute(&ch_class);

            // Set the channel scan channel map.
            lld_ch_scan_ch_map_update(&ch_class);
            ch_map_timer = true;
        }
#endif // (BLE_CH_SCAN_SUPPORT)

#if (BLE_CENTRAL)
        // Check if activity is a master link
        if ((llm_env.act_info[i].state == LLM_CONNECTED) && (llm_env.act_info[i].info.con.role == ROLE_MASTER))
        {
            // Update connection channel map
            llc_ch_map_update(i, &ch_map);
            ch_map_timer = true;
        }
#endif // (BLE_CENTRAL)

#if (BLE_BROADCASTER)
        // Check if activity is a periodic advertising
        if (llm_env.act_info[i].state == LLM_PER_ADV_EN)
        {
            // Request periodic advertising to update the channel map
            lld_per_adv_ch_map_update(i, &ch_map);
            ch_map_timer = true;
        }
#if (BLE_BIS)
        else if (llm_env.act_info[i].state == LLM_BIS_RSVD)
        {
            big_ch_map_update = true;
            ch_map_timer = true;
        }
#endif // (BLE_BIS)
#endif // (BLE_BROADCASTER)
    }

#if ((BLE_BIS) && (BLE_BROADCASTER))
    if (big_ch_map_update)
    {
        // Update BIG channel map
        lli_bi_ch_map_update(&ch_map);
    }
#endif // ((BLE_BIS) && (BLE_BROADCASTER))

    // If channel map management is needed
    if (ch_map_timer)
    {
        // Restart channel map update timer
        ke_timer_set(LLM_CH_MAP_TO, TASK_LLM, BLE_CH_MAP_UPDATE_PERIOD * 1000);
    }
    else
    {
        // Indicate channel map management as inactive
        llm_env.ch_map_info.active = false;
    }
}
#endif //(BLE_CENTRAL || BLE_BROADCASTER)

void llm_ch_class_compute(struct le_ch_class *ch_class)
{
    // Get Host channel classification
    struct le_ch_map *host_ch_class = llm_host_ch_class_get();

    // Get local channel classification based on assessment
    rwip_ch_class_ble_get(ch_class);

    // Compute the channel map
    for (uint8_t i = 0 ; i < BLE_DATA_CHANNEL_NB ; i++)
    {
        /*
         * Each bit of the host channel classification corresponds to:
         *      0: bad channel
         *      1: unknown channel (treated as good)
         */

        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;
        uint8_t ch_class_byte_idx = i >> 2;
        uint8_t ch_class_bit_pos = (i & 0x3) << 1;

        // Host indicates the channel as bad
        if (((host_ch_class->map[byte_idx] >> bit_pos) & 0x1) == 0x00)
        {
            ch_class->map[ch_class_byte_idx] &= ~(3 << ch_class_bit_pos);
            ch_class->map[ch_class_byte_idx] |= (LE_CH_CLASS_STATUS_BAD << ch_class_bit_pos);
            continue;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Handles the resolvable private address renew time out message.
 ****************************************************************************************
 */
__STATIC void llm_rpa_renew_to_handler(co_timer_periodic_t *p_timer)
{
    // Force renewal of resolvable private addresses
    lld_rpa_renew();
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void llm_init(bool is_reset)
{
    uint8_t length;
    uint8_t supp_phy_msk = PHY_1MBPS_BIT;
    uint32_t *rsvd_ral, *rsvd_act_info, *rsvd_dev;
    bluetooth_act_configt_t *rom_cfg = rom_config_get_default_link_config();
    RW_ASSERT(rom_cfg);

    if (!is_reset)
    {
        // Init memory
        if (llm_env.act_info)
            ke_free(llm_env.act_info);
        if (llm_env.ral)
            ke_free(llm_env.ral);
        if (llm_env.dev_list)
            ke_free(llm_env.dev_list);

        llm_env.act_info = (struct llm_act_info *)ke_malloc_system(sizeof(struct llm_act_info) * rom_cfg->ble_max_act, KE_MEM_ENV);

        llm_env.ral = (struct ral_entry *)ke_malloc_system(sizeof(struct ral_entry) * BLE_RAL_MAX, KE_MEM_ENV);

        llm_env.dev_list = (struct dev_list_entry *) ke_malloc_system(sizeof(struct dev_list_entry) * BLE_WHITELIST_MAX, KE_MEM_ENV);
        // Create LLM task
        ke_task_create(TASK_LLM, &TASK_DESC_LLM);
    }
    else
    {
        // Check if host parameters are stored
        for (uint8_t act_id = 0; act_id < rom_cfg->ble_max_act; act_id++)
        {
            if (llm_env.act_info[act_id].host_params != NULL)
            {
                ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
            }

#if BLE_CONLESS_CTE_TX
            if ((llm_env.act_info[act_id].state == LLM_ADV_RSVD) || (llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
            {
                if (llm_env.act_info[act_id].info.adv.cte_tx_params != NULL)
                {
                    ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.cte_tx_params));
                }
            }
#endif // BLE_CONLESS_CTE_TX

#if BLE_CONLESS_CTE_RX
            if ((llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCED) || (llm_env.act_info[act_id].state == LLM_PER_SCAN_STOPPING))
            {
                if (llm_env.act_info[act_id].info.sync.cte_rx_params != NULL)
                {
                    ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.sync.cte_rx_params));
                }
            }
#endif // BLE_CONLESS_CTE_RX

#if (EAVESDROPPING_SUPPORT)
#if (BLE_BROADCASTER || BLE_PERIPHERAL)
            if ((llm_env.act_info[act_id].state == LLM_ADV_RSVD) || (llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
            {
                // Release custom advertising parameters if they exist
                if (llm_env.act_info[act_id].info.adv.cust_adv_params != NULL)
                {
                    ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.cust_adv_params));
                    llm_env.act_info[act_id].info.adv.cust_adv_params = NULL;
                }

                // Release manual advertising parameters if they exist
                if (llm_env.act_info[act_id].info.adv.manual_adv_params != NULL)
                {
                    ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.manual_adv_params));
                    llm_env.act_info[act_id].info.adv.manual_adv_params = NULL;
                }
            }
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)
#endif //(EAVESDROPPING_SUPPORT)
        }
    }

    SETB(supp_phy_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
    SETB(supp_phy_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

    rsvd_ral = (uint32_t *)llm_env.ral;
    rsvd_act_info = (uint32_t *)llm_env.act_info;
    rsvd_dev = (uint32_t *)llm_env.dev_list;
    memset(&llm_env, 0, sizeof(llm_env));
    memset(rsvd_ral, 0, sizeof(struct ral_entry) * BLE_RAL_MAX);
    memset(rsvd_act_info, 0, sizeof(struct llm_act_info) * rom_cfg->ble_max_act);
    memset(rsvd_dev, 0, sizeof(struct dev_list_entry) * BLE_WHITELIST_MAX);
    llm_env.ral = (struct ral_entry *)rsvd_ral;
    llm_env.act_info = (struct llm_act_info *)rsvd_act_info;
    llm_env.dev_list = (struct dev_list_entry *)rsvd_dev;

    // Initialize non-zero HCI configuration parameters
    llm_env.rpa_renew_to            = RPA_TO_DFT;

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    llm_env.suggested_max_tx_octets = BLE_MIN_OCTETS;
    llm_env.suggested_max_tx_time   = BLE_MIN_TIME;
    llm_env.tx_phys                 = (supp_phy_msk);
    llm_env.rx_phys                 = (supp_phy_msk);
#if (EAVESDROPPING_SUPPORT)
    llm_env.suggested_max_rx_octets = BLE_MAX_OCTETS;
    llm_env.suggested_max_rx_time   = BLE_MAX_TIME;
    llm_env.ehn_set_data_length_enabled = false;
#endif //(EAVESDROPPING_SUPPORT)
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

#if (BT_53) && (BLE_CENTRAL)
    llm_env.subrate_info.subrate_min = CON_SUBRATE_DEFAULT;
    llm_env.subrate_info.subrate_max = CON_SUBRATE_DEFAULT;
    llm_env.subrate_info.max_latency = CON_SUBRATE_LATENCY_DEFAULT;
    llm_env.subrate_info.cont_num = CON_SUBRATE_CONTINUE_NUM_DEFAULT;
    llm_env.subrate_info.timeout =  CON_SUP_TO_MAX;
#endif // (BT_53) && (BLE_CENTRAL)

#if (BT_53) && (BLE_PERIPHERAL)
    // Channel classification reporting interval
    length = PARAM_LEN_CH_CLASS_REP_INTV;
    if (rwip_param.get(PARAM_ID_CH_CLASS_REP_INTV, &length, (uint8_t *)&llm_env.ch_class_rep_intv) != PARAM_OK)
    {
        llm_env.ch_class_rep_intv = 0;
    }
#endif // (BT_53) && (BLE_PERIPHERAL)

    // Initialize LE event mask
    memcpy(&llm_env.le_event_mask.mask[0], &le_default_evt_mask.mask[0], EVT_MASK_LEN);

    // Initialize Host channel classification
    memset(&llm_env.ch_map_info.host_ch_class.map[0], 0xFF, LE_CH_MAP_LEN);
    llm_env.ch_map_info.host_ch_class.map[LE_CH_MAP_LEN - 1] &= 0x1F;

    // BD address
    length = BD_ADDR_LEN;
    if (rwip_param.get(PARAM_ID_BD_ADDRESS, &length, (uint8_t *)&llm_env.local_pub_addr) != PARAM_OK)
    {
        memcpy(&llm_env.local_pub_addr, &co_default_bdaddr, sizeof(co_default_bdaddr));
    }

    length = PARAM_LEN_PRIVATE_KEY_P256;
    rwip_param.get(PARAM_ID_LE_PRIVATE_KEY_P256, &length, &llm_env.secret_key256[0]);

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    uint8_t act_move_cfg;
    length = PARAM_LEN_ACTIVITY_MOVE_CONFIG;
    if (rwip_param.get(PARAM_ID_ACTIVITY_MOVE_CONFIG, &length, &act_move_cfg) != PARAM_OK)
    {
        // If no value is set in persistent storage enable the feature by default
        llm_env.con_move_en = true;
    }
    else
    {
        // Bit 0 denotes support for BLE connections
        llm_env.con_move_en = (act_move_cfg & 0x01);
    }
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_OBSERVER)
    length = PARAM_LEN_SCAN_EXT_ADV;
    if (rwip_param.get(PARAM_ID_SCAN_EXT_ADV, &length, (uint8_t *) &llm_env.ext_scan) != PARAM_OK)
    {
        // Extended scanning is enabled by default
        llm_env.ext_scan = true;
    }
#endif //(BLE_OBSERVER)

    // Initialize RAL
    for (uint8_t position = 0; position < BLE_RAL_MAX; position++)
    {
        // Set invalid address type
        llm_env.ral[position].addr_type = 0xFF;
    }

    // Initialize local features
    memcpy(&llm_env.le_feats.feats[0], &llm_default_le_feats.feats[0], LE_FEATS_LEN);

#if (BLE_HOST_PRESENT)
#if (BLE_ISO_PRESENT)
    llm_env.le_feats.feats[BLE_FEAT_CIS_HOST_SUPPORT >> 3] |= (1 << (BLE_FEAT_CIS_HOST_SUPPORT & 0x7));
#endif //(BLE_ISO_PRESENT)
#if (BT_53)
    llm_env.le_feats.feats[BLE_FEAT_CON_SUBRATING_HOST_SUPPORT >> 3] |= (1 << (BLE_FEAT_CON_SUBRATING_HOST_SUPPORT & 0x7));
#endif //(BLE_ISO_PRESENT)
#endif //(BLE_HOST_PRESENT)

    // Start the timer that triggers renewal of resolvable private address
    co_timer_periodic_init(&(llm_env.rpa_renew_timer), llm_rpa_renew_to_handler);
    co_timer_periodic_set(&(llm_env.rpa_renew_timer), CO_TIME_SEC_TO_MS(llm_env.rpa_renew_to));
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
void llm_link_disc(uint8_t link_id)
{
    struct sch_plan_elt_tag *plan_elt = &llm_env.act_info[link_id].plan_elt;
    uint8_t position;

    // Indicate that link does not require active clock anymore
    llm_clk_acc_set(link_id, false);

    // Free link identifier and LT address
    llm_env.act_info[link_id].state = LLM_FREE;

    // Unregister the connection from bandwidth allocation system
    sch_plan_rem(plan_elt);

    // Check if the device is in the device list
    position = llm_dev_list_search(&llm_env.act_info[link_id].info.con.bd_addr, llm_env.act_info[link_id].info.con.addr_type);

    // Restore presence of device in EM white list, if it is in white list
    if ((position < BLE_WHITELIST_MAX) && GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
    {
        lld_white_list_add(position, &llm_env.act_info[link_id].info.con.bd_addr, llm_env.act_info[link_id].info.con.addr_type);
    }

    // Unregister linkID at HCI level
    hci_ble_conhdl_unregister(link_id);
}

void llm_clk_acc_set(uint8_t act_id, bool clk_acc)
{
    if (clk_acc)
    {
        llm_env.act_clk_acc |= (1 << act_id);

        // Switch to active clock immediately
        rwip_prevent_sleep_set(RW_BLE_ACTIVE_MODE);
    }
    else
    {
        llm_env.act_clk_acc &= ~(1 << act_id);

        // Check if no BLE connection needs the active clock anymore
        if (llm_env.act_clk_acc == 0)
        {
            rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);
        }
    }
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

struct le_ch_map *llm_host_ch_class_get(void)
{
    return &llm_env.ch_map_info.host_ch_class;
}


bool llm_le_evt_mask_check(uint8_t event_id)
{
    return ((llm_env.le_event_mask.mask[event_id >> 3] & (1 << (event_id & 0x07))) != 0);
}

void llm_le_features_get(struct le_features *feats)
{
    memcpy(&feats->feats[0], &llm_env.le_feats.feats[0], LE_FEATS_LEN);
}

bool llm_le_feature_check(uint8_t feature)
{
    return ((llm_env.le_feats.feats[feature >> 3] & (1 << (feature & 0x7))) != 0);
}

struct ral_entry *llm_ral_get(void)
{
    return &llm_env.ral[0];
}

struct sch_plan_elt_tag *llm_plan_elt_get(uint8_t act_id)
{
    return (&llm_env.act_info[act_id].plan_elt);
}

uint8_t llm_activity_free_get(uint8_t *act_id)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // If not present, try to allocate an activity identifier
    for (*act_id = 0; *act_id < BLE_ACTIVITY_MAX_CFG; (*act_id)++)
    {
        if (llm_env.act_info[*act_id].state == LLM_FREE)
        {
            // Initialize activity info
            memset(&llm_env.act_info[*act_id].info, 0, sizeof(llm_env.act_info[*act_id].info));

            break;
        }
    }

    // If not possible to start a new advertising activity, reject the command
    if (*act_id >= BLE_ACTIVITY_MAX_CFG)
    {
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
    }

    return (status);
}

void llm_activity_free_set(uint8_t act_id)
{
    llm_env.act_info[act_id].state = LLM_FREE;
    // clean-up possible interval in use
    sch_plan_rem(&(llm_env.act_info[act_id].plan_elt));
}

#if (BLE_CIS)
void llm_activity_cis_reserve(uint8_t act_id)
{
    llm_env.act_info[act_id].state = LLM_CIS_RSVD;
}
#endif // (BLE_CIS)

#if (BLE_BIS)
void llm_activity_bis_reserve(uint8_t act_id)
{
    llm_env.act_info[act_id].state = LLM_BIS_RSVD;

#if (BLE_BROADCASTER)
    // Enable channel map monitoring
    llm_ch_map_timer_enable();
#endif // (BLE_BROADCASTER)
}
#endif // (BLE_BIS)

#if (BLE_ISO_MODE_0)
bool llm_link_active(uint8_t act_id)
{
    return (llm_env.act_info[act_id].state == LLM_CONNECTED);
}
#endif // (BLE_ISO_MODE_0)

#if BT_DUAL_MODE
bool llm_activity_ongoing_check(void)
{
    int act_id;

    // check if any BLE activities ongoing
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        if ((1L << llm_env.act_info[act_id].state) &
                ~((1L << LLM_FREE) | (1L << LLM_SCAN_RSVD) | (1L << LLM_ADV_RSVD) | (1L << LLM_PER_ADV_RSVD) | (1L << LLM_CON_RSVD)))
        {
            break;
        }
    }

    return (act_id < BLE_ACTIVITY_MAX_CFG);
}
#endif //BT_DUAL_MODE

#if (EAVESDROPPING_SUPPORT)
bool llm_ehn_set_data_length_enabled_get(void)
{
    return llm_env.ehn_set_data_length_enabled;
}
#endif //(EAVESDROPPING_SUPPORT)

#if (BLE_ADV_LEGACY_ITF)
bool llm_is_adv_itf_legacy()
{
    return (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY);
}

void llm_adv_itf_extended_set()
{
    if (llm_env.adv_itf_version == LLM_ADV_ITF_UNKNOWN)
    {
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
    }
}
#endif //(BLE_ADV_LEGACY_ITF)

void llm_ch_map_timer_enable()
{
    // Check if master channel map monitoring is already active
    if (!llm_env.ch_map_info.active)
    {
        llm_env.ch_map_info.active = true;

        // Start channel map update timer
        ke_timer_set(LLM_CH_MAP_TO, TASK_LLM, BLE_CH_MAP_UPDATE_PERIOD * 1000);
    }
}

/// @} LLM
