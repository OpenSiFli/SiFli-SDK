/**
 ****************************************************************************************
 *
 * @file llm.h
 *
 * @brief Main API file for the Link Layer manager
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef LLM_H_
#define LLM_H_

/**
 ****************************************************************************************
 * @defgroup LLM Link Layer Manager
 * @ingroup ROOT
 * @brief BLE Lower Layers
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_task.h"      // Task definitions
#include "co_bt.h"          // BLE standard definitions

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Parameters for syncing to a periodic advertiser
struct llm_per_adv_sync_params
{
    /// Activity identifier
    uint8_t act_id;

    /// SyncInfo
    struct sync_info syncinfo;

    /// The clock value of the reference time (in half-slot)
    uint32_t base_cnt;

    /// The fine timer value the reference time (in half-us)
    uint16_t fine_cnt;

    /// ID (provided by the Host)
    uint16_t       id;

    /// The bit rate of periodic advertising (@see enum lld_rate)
    uint8_t rate;

    /// Advertising SID
    uint8_t adv_sid;

    /// Advertising address type (0: Public / 1: Random)
    uint8_t adv_addr_type;

    /// Advertiser address
    struct bd_addr adv_addr;

    /// Advertiser RPA (unresolved RPA if advertiser address is a resolved identity address, 0 otherwise)
    struct bd_addr adv_rpa;

    /// max Skip after receive
    uint16_t skip;

    /// Sync timeout (Time=N*10ms)
    uint16_t sync_to;

    /// Additional drift to consider on given time reference (in half-us)
    uint16_t add_drift;

    /// Advertising report initial state (enable/disable)
    bool adv_rep_en;

#if (BT_53)
    /// ADI filtering (enable/disable)
    bool adi_filt_en;
#endif // (BT_53)

    /// Specifies whether to only synchronize to periodic advertising with certain types of CTE (@see enum sync_cte_type)
    uint8_t sync_cte_type;
};


/*
 * MESSAGES
 ****************************************************************************************
 */

/// Message API of the LLM task
/*@TRACE
 * llm_encrypt_ind      = aes_func_res_ind*/
enum llm_msg_id
{
    LLM_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_LLM),//!< LLM_MSG_ID_FIRST

#if (BLE_OBSERVER)
    /// Send ACAD Data information
    LLM_ACAD_DATA_IND,                             //!< LLM_ACAD_DATA_IND
#endif // (BLE_OBSERVER)

    /*
     * ************** Msg LLM->LLM****************
     */
    LLM_SCAN_PERIOD_TO,                            //!< LLM_SCAN_PERIOD_TO
    LLM_CH_MAP_TO,                                 //!< LLM_CH_MAP_TO
#if (BLE_CENTRAL || BLE_BROADCASTER)
    LLM_NEW_HOST_CLASS_TO,
#endif //(BLE_CENTRAL || BLE_BROADCASTER)
    /// Inform that encryption has been performed
    LLM_ENCRYPT_IND,                               //!< LLM_ENCRYPT_IND
};

/// LLM Encryption Request parameters structure
/*@TRACE*/
struct llm_encrypt_req
{
    ///Long term key structure
    struct ltk     key;
    ///Pointer to buffer with plain data to encrypt - 16 bytes
    uint8_t        plain_data[ENC_DATA_LEN];
};

/// LLM Encryption indication structure
/*@TRACE*/
struct llm_encrypt_rsp
{
    /// Status of the encryption
    uint8_t status;
    ///Encrypted data
    uint8_t encrypted_data[ENC_DATA_LEN];
};


/// AES function execution result
/*@TRACE*/
struct llm_encrypt_ind
{
    /// Status of AES execution
    uint8_t status;
    /// Result of the
    uint8_t result[KEY_LEN];
};


#if (BLE_OBSERVER)
/// LLM ACAD data indication structure
/*@TRACE*/
struct llm_acad_data_ind
{
    /// Sync handle
    uint16_t sync_handle;
    /// Status the indication - use to detect loss of periodic sync
    uint8_t  status;
    /// ADV Data Type
    uint8_t  ad_type;
    /// Reference event counter of the periodic advertiser when report received
    uint16_t ref_evt_cnt;

    /// ACAD Data
    union
    {
        /// BIGInfo
        struct big_info big_info;
    } data;


};
#endif // (BLE_OBSERVER)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BLE LLM task
 *
 * This function initializes the the LLM task, as well as the environment of the LLM
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void llm_init(bool is_reset);

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /**
    ****************************************************************************************
    * @brief This function handles LE link disconnection.
    *
    * @param[in] link_id            Link Identifier
    ****************************************************************************************
    */
    void llm_link_disc(uint8_t link_id);

    /**
    ****************************************************************************************
    * @brief Indicate activity requires accurate clock
    *
    * @param[in] act_id      Activity ID
    * @param[in] clk_acc     True if activity requires accurate clock
    ****************************************************************************************
    */
    void llm_clk_acc_set(uint8_t act_id, bool clk_acc);
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief This function is used to get the host channel classification
 *
 * @return Pointer to master channel map
 ****************************************************************************************
 */
struct le_ch_map *llm_host_ch_class_get(void);

/**
 ****************************************************************************************
 * @brief This function is used to check if an event is unmasked/masked
 *
 * @param[in]  event_id  Bit position of the event in the mask (see standard specification part II.E.7.8.1)
 *
 * @return True: event allowed | False: event masked
 ****************************************************************************************
 */
bool llm_le_evt_mask_check(uint8_t event_id);

/**
 ****************************************************************************************
 * @brief Retrieve LE Local Supported features
 *
 * @return Local feature mask
 ****************************************************************************************
 */
void llm_le_features_get(struct le_features *feats);

/**
 ****************************************************************************************
 * @brief Check if the feature is supported
 *
 * @param[in] feature  Feature to check (@see enum ble_feature)
 *
 * @return True if feature is supported, False if not supported
 ****************************************************************************************
 */
bool llm_le_feature_check(uint8_t feature);

/**
 ****************************************************************************************
 * @brief Get pointer of the RAL table in RAM
 *
 * @return    pointer of the RAL table in RAM
 ****************************************************************************************
 */
struct ral_entry *llm_ral_get(void);

/**
 ****************************************************************************************
 * @brief This function returns a pointer to the planner element of an activity
 *
 * @param[in] act_id            Activity Identifier
 *
 * @return Pointer to the associated planner element
 ****************************************************************************************
 */
struct sch_plan_elt_tag *llm_plan_elt_get(uint8_t act_id);

/**
 ****************************************************************************************
 * @brief Get available activity index
 *
 * @param[out] act_id            Activity Identifier allocated
 *
 * @return  - CO_ERROR_NO_ERROR:         if succeed
 *          - CO_ERROR_CON_LIMIT_EXCEED: if no activity available
 ****************************************************************************************
 */
uint8_t llm_activity_free_get(uint8_t *act_id);

/**
 ****************************************************************************************
 * @brief Free activity handle related to the activity
 *
 * @param[in] act_id            Activity Identifier
 ****************************************************************************************
 */
void llm_activity_free_set(uint8_t act_id);

#if (BLE_CIS)
    /**
    ****************************************************************************************
    * @brief Reserve an activity for a CIS
    *
    * @param[in] act_id            Activity Identifier
    ****************************************************************************************
    */
    void llm_activity_cis_reserve(uint8_t act_id);
#endif // (BLE_CIS)

#if (BLE_BIS)
    /**
    ****************************************************************************************
    * @brief Reserve an activity for a BIS
    *
    * @param[in] act_id            Activity Identifier
    ****************************************************************************************
    */
    void llm_activity_bis_reserve(uint8_t act_id);
#endif // (BLE_BIS)

#if (BLE_ISO_MODE_0)
    /**
    ****************************************************************************************
    * @brief Check if an activity is an active link
    *
    * @param[out] act_id            Activity Identifier
    *
    * @return  true if in connected state, false else
    ****************************************************************************************
    */
    bool llm_link_active(uint8_t act_id);
#endif // (BLE_ISO_MODE_0)

#if BT_DUAL_MODE
    /**
    ****************************************************************************************
    * @brief Get if Activity is ongoing
    *
    * @return  true if active
    ****************************************************************************************
    */
    bool llm_activity_ongoing_check(void);
#endif //BT_DUAL_MODE

/**
 ****************************************************************************************
 * @brief Get advertising activity identifiers
 *
 * @param[in]  adv_hdl           Advertising handle
 * @param[out] ext_adv_id        Extended advertising ID
 * @param[out] per_adv_id        Periodic advertising ID
 *
 * @return  - CO_ERROR_NO_ERROR:               if succeed
 *          - CO_ERROR_UNKNOWN_ADVERTISING_ID: if no advertising set exists
 *          - CO_ERROR_COMMAND_DISALLOWED:     if periodic advertising not in progress
 ****************************************************************************************
 */
uint8_t llm_adv_act_id_get(uint8_t adv_hdl, uint8_t *ext_adv_id, uint8_t *per_adv_id);

/**
 ****************************************************************************************
 * @brief Get periodic advertising train identifiers
 *
 * @param[in]  act_id        Activity ID of extended advertising
 * @param[out] sid           Advertising SID
 * @param[out] atype         Advertising address type (0: public, 1: random)
 * @param[out] adva          Advertising address (RPA if address is resolvable)
 *
 * @return  - CO_ERROR_NO_ERROR:               if succeed
 *          - CO_ERROR_COMMAND_DISALLOWED:     if periodic advertising not in progress
 ****************************************************************************************
 */
uint8_t llm_adv_set_id_get(uint8_t act_id, uint8_t *sid, uint8_t *atype, struct bd_addr *adva);

#if (BLE_BIS)
    /**
    ****************************************************************************************
    * @brief Retrieve Periodic Advertising activity identifier
    *
    * @param[in]  adv_hdl      Advertising handle of a Periodic Advertiser
    * @param[out] ext_act_id   Returned extended advertiser activity identifier
    * @param[out] per_act_id   Returned periodic advertiser activity identifier
    *
    * @return CO_ERROR_NO_ERROR if the periodic advertiser is found
    ****************************************************************************************
    */
    uint8_t llm_adv_per_id_get(uint8_t adv_hdl, uint8_t *ext_act_id, uint8_t *per_act_id);

    /**
    ****************************************************************************************
    * @brief Attach an BIG to a periodic advertising driver
    *
    * @param[in] ext_act_id   Extended advertising activity identifier
    * @param[in] per_act_id   Periodic advertising activity identifier
    * @param[in] big_act_id   Activity ID allocated for BIG
    *
    * @return CO_ERROR_NO_ERROR if the periodic advertiser is found
    ****************************************************************************************
    */
    uint8_t llm_adv_big_attach(uint8_t ext_act_id, uint8_t per_act_id, uint8_t big_act_id);

    /**
    ****************************************************************************************
    * @brief Detach an BIG from a periodic advertising driver
    *
    * @param[in] ext_act_id   Extended advertising activity identifier
    * @param[in] per_act_id   Periodic advertiser activity identifier
    * @param[in] big_act_id   Activity ID allocated for BIG
    *
    * @return CO_ERROR_NO_ERROR if the periodic advertiser is found
    ****************************************************************************************
    */
    uint8_t llm_adv_big_detach(uint8_t ext_act_id, uint8_t per_act_id, uint8_t big_act_id);
#endif // (BLE_BIS)

/**
 ****************************************************************************************
 * @brief Attach a task with a periodic scanner to receive specific AD Type data
 *
 * @param[in] sync_act_id Activity identifier of the periodic sync
 * @param[in] ad_type     AD Type to retrieve in ACAD data
 * @param[in] task        Task that expect the LLM_ACAD_DATA_IND message
 *
 * @return CO_ERROR_NO_ERROR if the periodic sync is found
 ****************************************************************************************
 */
uint8_t llm_scan_sync_acad_attach(uint8_t sync_act_id, uint8_t ad_type, uint16_t task);


/**
 ****************************************************************************************
 * @brief Detach periodic scanner ACAD Data watcher
 *
 * @param[in] sync_act_id Activity identifier of the periodic sync
 *
 * @return CO_ERROR_NO_ERROR if the periodic sync is found
 ****************************************************************************************
 */
uint8_t llm_scan_sync_acad_detach(uint8_t sync_act_id);


/**
 ****************************************************************************************
 * @brief Attach a task with a periodic scanner to receive specific AD Type data
 *
 * @param[in] sync_act_id     Sync activity ID
 * @param[in] ad_type         AD Type to retrieve in ACAD data
 * @param[in] task            Task that expect the LLM_ACAD_DATA_IND message
 *
 * @return CO_ERROR_NO_ERROR if the periodic sync is found
 ****************************************************************************************
 */
uint8_t llm_scan_sync_acad_attach(uint8_t sync_act_id, uint8_t ad_type, uint16_t task);

#if (BLE_PAST)
    /**
    ****************************************************************************************
    * @brief Get periodic sync information
    *
    * @param[in]  sync_act_id       Sync activity ID
    * @param[out] sid               Extended advertising set identifier
    * @param[out] atype             Extended advertiser address type (0: public | 1: random)
    * @param[out] adva              Extended advertiser address (for resolvable addresses, this is the unresolved version)
    *
    * @return  - CO_ERROR_NO_ERROR:               if succeed
    *          - CO_ERROR_UNKNOWN_ADVERTISING_ID: if the sync activity does not exist
    ****************************************************************************************
    */
    uint8_t llm_scan_sync_info_get(uint8_t sync_act_id, uint8_t *sid, uint8_t *atype, struct bd_addr *adva);

    /**
    ****************************************************************************************
    * @brief Synchronize to a periodic advertising
    *
    * param[in] params    Input parameters (@see llm_per_adv_sync_params)
    *
    * @return 0 if synchronization attempt is started, >0 if issue
    ****************************************************************************************
    */
    uint8_t llm_per_adv_sync(struct llm_per_adv_sync_params *params);
#endif // (BLE_PAST)

#if (EAVESDROPPING_SUPPORT)
    /**
    ****************************************************************************************
    * @brief Get whether Enhanced set data length is enabled
    *
    * @return    enh set data length enabled
    ****************************************************************************************
    */
    bool llm_ehn_set_data_length_enabled_get(void);
#endif //(EAVESDROPPING_SUPPORT)

#if (BLE_ADV_LEGACY_ITF)
    /**
    ****************************************************************************************
    * @brief Check if the current advertising interface version is legacy
    *
    * @return true if advertising interface version is legacy, false otherwise
    ****************************************************************************************
    */
    bool llm_is_adv_itf_legacy();

    /**
    ****************************************************************************************
    * @brief Set the current advertising interface version to extended
    ****************************************************************************************
    */
    void llm_adv_itf_extended_set();
#endif //(BLE_ADV_LEGACY_ITF)

/**
 ****************************************************************************************
 * @brief Entry point to update the channel map
 ****************************************************************************************
 */
void llm_ch_map_update(void);

/**
 ****************************************************************************************
 * @brief This function is used to activate channels in case not enough channel are enabled.
 * A minimum of DATA_CHANNEL_USED_NB_MIN channels are activated.
 *
 * @param[in]  ch_map     Pointer to channel map
 ****************************************************************************************
 */
void llm_fill_ch_map(struct le_ch_map *ch_map);

/**
 ****************************************************************************************
 * @brief This function is used to compute the channel map based on the combination of the
 * local assessment and the host classification
 *
 * @param[in]  ch_map     Pointer to channel map
 ****************************************************************************************
 */
void llm_ch_map_compute(struct le_ch_map *ch_map);

/**
 ****************************************************************************************
 * @brief This function is used to compute the channel classification based on the combination
 * of the local assessment and the host classification
 *
 * @param[in]  ch_class    Pointer to channel classification
 ****************************************************************************************
 */
void llm_ch_class_compute(struct le_ch_class *ch_class);

/**
 ****************************************************************************************
 * @brief This function is used enable the channel map timer
 ****************************************************************************************
 */
void llm_ch_map_timer_enable();

/// @} LLM

#endif // LLM_H_
