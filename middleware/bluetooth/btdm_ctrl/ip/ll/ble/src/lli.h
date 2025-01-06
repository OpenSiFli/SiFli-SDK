/**
 ****************************************************************************************
 *
 * @file lli.h
 *
 * @brief Main API file for the Link Layer ISO
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef LLI_H_
#define LLI_H_

/**
 ****************************************************************************************
 * @defgroup LLI Link Layer ISO
 * @ingroup ROOT
 * @brief BLE Lower Layers
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISO_PRESENT)

#include "rwip_task.h"      // Task definitions
#include "co_bt.h"          // BLE standard definitions
#include "co_llcp.h"        // Definition of LL_CIS_REQ PDU
#include "lld.h"            // LLD Definitions

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
#if (BLE_CIS)

/// CIS Group Parameters
struct lli_cig_param
{
    /// TX SDU interval (in us)
    uint32_t        tx_sdu_interval;
    /// RX SDU interval (in us)
    uint32_t        rx_sdu_interval;

    /// TX Transport latency (in us)
    uint32_t        tx_trans_latency;
    /// RX Transport latency (in us)
    uint32_t        rx_trans_latency;

    /// CIG_Sync_Delay in us (computed when enabling first channel)
    uint32_t        cig_sync_delay;

    /// isochronous Interval in multiple of 1.25 ms (Range 0x0004-0xC80)
    uint16_t        iso_interval;

    /// TX Master to Slave Flush timeout
    uint8_t         tx_ft;
    /// RX Slave to Master Flush timeout
    uint8_t         rx_ft;

    /// CIG Frame mode (@see enum iso_frame)
    uint8_t         framing;
};

/// CIS Channel Parameters
struct lli_cis_param
{
    /// CIG identifier
    uint8_t         cig_id;
    /// CIS identifier
    uint8_t         cis_id;

    /// TX Max SDU size (12 bits meaningful) (in bytes)
    uint16_t        tx_max_sdu;
    /// RX Max SDU size (12 bits meaningful) (in bytes)
    uint16_t        rx_max_sdu;

    /// Number of Sub Event
    uint8_t         nse;

    /// TX Physical Channel rate selected
    uint8_t         tx_rate;
    /// TX burst number: number of new payloads per event (0: no new packet -  Range 0x01-0x1F)
    uint8_t         tx_bn;
    /// TX maximum PDU size (in bytes)
    uint8_t         tx_max_pdu;

    /// RX Physical Channel rate Info
    uint8_t         rx_rate;
    /// RX burst number: number of new payloads per event (0: no new packet -  Range 0x01-0x1F)
    uint8_t         rx_bn;
    /// RX maximum PDU size (in bytes)
    uint8_t         rx_max_pdu;

    /// CIS offset in the CIG (in us)
    uint32_t        cis_offset_in_cig;

    /// Sub-event interval (in us) (Range 400us - Connection interval)
    uint32_t        sub_interval;

    /// Sub Event duration for the configured PHY (in us)
    uint16_t        sub_evt_dur;
};

/**
 ****************************************************************************************
 * @brief Callback executed when a new CIS activity is granted
 *
 * @param[in]  cis_act_id         CIS Activity identifier
 * @param[in]  acl_act_id         ACL Activity identifier
 * @param[in]  p_cis_params       Pointer to CIS  parameters structure
 * @param[in]  iso_interval       ISO interval in multiple of 1.25 ms
 * @param[in]  access_addr        CIS access address
 * @param[in]  cis_start_hs       CIS start instant in half slot (312.5us step)
 * @param[in]  cis_start_hus      CIS start instant in half microseconds (0.5us step)
 *
 ****************************************************************************************
 */
typedef void (*lli_cis_start_evt_cb)(uint8_t cis_act_id, uint8_t acl_act_id,
                                     struct lli_cis_param *p_cis_params,
                                     uint16_t iso_interval, uint32_t access_addr,
                                     uint32_t cis_start_hs, uint16_t cis_start_hus);
#endif // (BLE_CIS)

#if (BLE_BIS)
/**
 ****************************************************************************************
 * @brief Callback executed when new BIS activities are granted during a BIG start
 *
 * @param[in]  p_big_params          Pointer to CIS  parameters structure
 * @param[in]  p_big_info            Pointer to BIG Sync Info format
 * @param[in]  p_big_start_time      Pointer to BIG start time structure
 *
 ****************************************************************************************
 */
typedef void (*lli_bis_start_evt_cb)(struct lld_big_params *p_big_params, struct big_info *p_big_info,
                                     struct lld_big_start_time *p_big_start_time);
#endif // (BLE_BIS)


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BLE LLI task
 *
 * This function initializes the the LLI task.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void lli_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Inform that a link has terminated to immediately stop corresponding ISO channels
 *
 * @param[in] link_id Link identifier
 * @param[in] reason  Link termination reason
 ****************************************************************************************
 */
void lli_link_stop_ind(uint8_t link_id, uint8_t reason);

#if (BLE_ISO_MODE_0)
    /**
    ****************************************************************************************
    * @brief Check if link is used for audio mode 0
    *
    * @param[in] link_id Link identifier
    *
    * @return   True if link is used for audio mode 0
    ****************************************************************************************
    */
    bool lli_am0_check(uint8_t link_id);
#endif // (BLE_ISO_MODE_0)

#if (BLE_CIS)

    /**
    ****************************************************************************************
    * @brief Return CIG and CIS parameters for the provided channel handle
    *
    * @param[in]  act_id        Activity ID
    * @param[out] pp_cig_param  CIG parameters
    * @param[out] pp_cis_param  CIS parameters
    ****************************************************************************************
    */
    void lli_ci_param_get(uint8_t act_id, struct lli_cig_param **pp_cig_param, struct lli_cis_param **pp_cis_param);

    /**
    ****************************************************************************************
    * @brief Return if CIS is present on an ACL link
    *
    * @param[in] link_id  Link identifier
    ****************************************************************************************
    */
    bool lli_cis_is_present(uint8_t link_id);


    #if (BLE_PERIPHERAL)
        /**
        ****************************************************************************************
        * @brief Function called by LLC CIS module upon reception of LL_CIS_REQ PDU it is used to
        * ask for creation of the CIS channel
        *
        * @param[in]     link_id      Link identifier
        * @param[in]     p_pdu        Pointer to the received LL_CIS_REQ pdu
        * @param[in]     chan_bw      Channel required bandwidth (in us)
        * @param[in]     framing      Framing mode 0 - Unframed Mode, 1 - Framed Mode
        * @param[out]    p_act_id     Pointer to a variable that will contain the allocated activity ID
        *
        * @return CIS Creation status
        ****************************************************************************************
        */
        uint8_t lli_cis_create_req(uint8_t link_id, struct ll_cis_req *p_pdu, uint8_t framing, uint8_t *p_act_id);
    #endif //(BLE_PERIPHERAL)

    /**
    ****************************************************************************************
    * @brief Function used by LLC CIS module when CIS creation negotiation is finshed
    *
    * @param[in] act_id          Activity ID for created CIS channel
    * @param[in] status          Status of CIS channel creation
    * @param[in] access_addr     CIS Channel access address
    * @param[in] evt_cnt         Connection Event counter as reference point for CIS start
    * @param[in] con_offset      Offset after connection reference in us when the CIS can be started
    * @param[in] act_offset      Activity offset, integer part (in half-slots)
    * @param[in] act_offset_hus  Activity offset, fractional part (in half-us)
    * @param[in] air_time        Air duration required by the activity (in us)
    * @param[in] encrypted       Indicates if the CIS is encrypted or not
    ****************************************************************************************
    */
    void lli_cis_create_end(uint8_t act_id, uint8_t status, uint32_t access_addr, uint16_t evt_cnt, uint32_t con_offset, uint16_t act_offset, uint16_t act_offset_hus, uint32_t cis_evt_dur, bool encrypted);

    /**
    ****************************************************************************************
    * @brief Function used by LLC CIS module upon reception of LL_CIS_TERM_IND PDU
    *
    * @param[in]     link_id      Link identifier
    * @param[in]     p_pdu        Pointer to the received LL_CIS_TERM_IND pdu
    ****************************************************************************************
    */
    void lli_cis_stop_req(uint8_t link_id, struct ll_cis_terminate_ind *p_pdu);

    /**
    ****************************************************************************************
    * @brief Function used by LLC CIS module upon completion of LLCP CIS Termination procedure
    *
    * @param[in] act_id          Activity ID of the CIS
    ****************************************************************************************
    */
    void lli_cis_stop_end(uint8_t act_id);

    /**
    ****************************************************************************************
    * @brief Set an event callback to be executed when a CIS activity is granted
    *
    * @param[in] evt_cb Callback to execute
    *****************************************************************************************
    */
    void lli_cis_start_evt_set(lli_cis_start_evt_cb evt_cb);
#endif // (BLE_CIS)

#if (BLE_BIS)
    /**
    ****************************************************************************************
    * @brief Set an event callback to be executed when a BIS activity is granted
    *
    * @param[in] evt_cb Callback to execute
    *****************************************************************************************
    */
    void lli_bis_start_evt_set(lli_bis_start_evt_cb evt_cb);

    #if (BLE_BROADCASTER)
        /**
        ****************************************************************************************
        *  @brief Update LLC channel map (if needed)
        *
        * @param[in] p_llm_ch_map   Pointer to LLM channel map
        ****************************************************************************************
        */
        void lli_bi_ch_map_update(struct le_ch_map *p_llm_ch_map);
    #endif // (BLE_BROADCASTER)
#endif // (BLE_BIS)

#endif //(BLE_ISO_PRESENT)

/// @} LLI

#endif // LLI_H_
