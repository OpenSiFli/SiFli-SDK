/**
****************************************************************************************
*
* @file lm_sco.h
*
* @brief Link Manager synchronous link functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef LM_SCO_H_
#define LM_SCO_H_
/**
****************************************************************************************
* @defgroup LMSCO SCO Functions
* @ingroup LM
* @brief LM module for synchronous functions.
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if (MAX_NB_SYNC > 0)

#include "lc_sco.h"

/*
 * DEFINITION
 * ***************************************************************************************
 */
/// Commands for synchronous link
#define SYNC_HL_REQ             0
#define SYNC_PEER_REQ           1
#define SYNC_HL_RSP             2
#define SYNC_PEER_RSP           3
#define SYNC_HL_MODIF           4
#define SYNC_PEER_MODIF         5
#define SYNC_UPDATE             6

/*
 * FUNCTION DECLARATIONS
 * ***************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief This function is used to initialize LM Sco Data.
 ****************************************************************************************
 */
void lm_init_sync(void);
/**
 ****************************************************************************************
 * @brief This function is used to reset LM Sco Data.
 ****************************************************************************************
 */
void lm_reset_sync(void);

/**
 ****************************************************************************************
 * @brief This function is used to check and store informations about a SCO link.
 *
 * @param[in]  sco_link_id           Synchronous link ID
 * @param[in]  RemoteFeatures        Remote device features
 * @param[in]  Role                  Master or Slave
 * @param[in]  SyncType              SCO or eSCO
 * @param[in]  SyncLtAddr            Synchronous link LT address
 * @param[out] *SyncHdl              Synchronous link handle
 * @param[in]  SyncTxBw              TX bandwidth
 * @param[in]  SyncRxBw              RX bandwidth
 * @param[in]  SyncLat               Maximum allowed Latency
 * @param[in]  SyncVoice             Voice settings
 * @param[in]  SyncAirMode           Air Mode
 * @param[in]  SyncReTx              Retransmission effort
 * @param[in]  SyncPacketType        Allowed packet types
 * @param[out] sco_params            SCO parameters
 * @param[in]  Request               Request originator
 *
 * @return Status
 ****************************************************************************************
 */
uint8_t lm_add_sync(uint8_t sco_link_id, struct features *RemoteFeatures,
                    uint8_t LinkId,      uint8_t Role,           uint8_t *SyncType,
                    uint32_t SyncTxBw,   uint32_t SyncRxBw,       uint16_t SyncLat,        uint16_t VoiceSetting,
                    uint8_t SyncReTx,        uint16_t SyncPacketType,
                    struct lc_sco_air_params_tag *sco_params, uint8_t Request);

/**
 ****************************************************************************************
 * @brief This function is used to check if a Sync link can be modified.
 *
 * @param[in] sco_link_id       Synchronous link ID
 * @param[in] SyncType              SCO or eSCO
 * @param[in] SyncTxBw              TX bandwidth
 * @param[in] SyncRxBw              RX bandwidth
 * @param[in] SyncLat               Maximum allowed Latency
 * @param[in] SyncVoice             Voice settings
 * @param[in] SyncReTx              Retransmission effort
 * @param[in] SyncPacketType        Allowed packet types
 * @param[out] sco_params           SCO parameters
 * @param[in] Request               Request type
 *
 * @return Status
 *
 ****************************************************************************************
 */
uint8_t lm_modif_sync(uint8_t sco_link_id, uint8_t SyncType,
                      uint32_t SyncTxBw, uint32_t SyncRxBw, uint16_t SyncLat,
                      uint16_t SyncVoice, uint8_t SyncReTx, uint16_t SyncPacketType,
                      struct lc_sco_air_params_tag *sco_params, uint8_t Request);

/**
 ****************************************************************************************
 * @brief This function is used to check the Accept Sync connection parameters.
 *
 * @param[in] sco_link_id       Synchronous link ID
 * @param[in] SyncTxBw          TX bandwidth
 * @param[in] SyncRxBw          RX bandwidth
 * @param[in] SyncLat           Maximum allowed Latency
 * @param[in] SyncVoice         Voice settings
 * @param[in] SyncReTx          Retransmission effort
 * @param[in] SyncPacketType    Allowed packet types
 * @param[out] sco_params       SCO parameters
 *
 * @return Status
 *
 ****************************************************************************************
 */
uint8_t lm_check_sync_hl_rsp(uint8_t sco_link_id, uint32_t SyncTxBw,  uint32_t SyncRxBw,
                             uint16_t SyncLat,    uint16_t SyncVoice, uint8_t SyncReTx, uint16_t SyncPacketType,
                             struct lc_sco_air_params_tag *sco_params);

/**
 ****************************************************************************************
 * @brief This function is used to remove stored synchronous link information.
 *
 * @param[in] sco_link_id       Synchronous link ID
 ****************************************************************************************
 */
void lm_remove_sync(uint8_t sco_link_id);

/**
 ****************************************************************************************
 * @brief This function is used to Restore parameters for a synchronous link.
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[out] sco_params      SCO parameters
 * @param[in] Request          Request type
 *
 * @return Status
 ****************************************************************************************
 */
uint8_t lm_get_sync_param(uint8_t sco_link_id, struct lc_sco_air_params_tag *sco_params, uint8_t Request);


/**
 ****************************************************************************************
 * @brief This function is used to get the Sync Handle of a Sync link.
 *
 * @param[in] sco_link_id       Synchronous link ID
 *
 * @return Synchronous link handle
 ****************************************************************************************
 */
uint8_t lm_get_synchdl(uint8_t sco_link_id);

/**
 ****************************************************************************************
 * @brief This function is used to check if an ACL link has at least 1 synchronous link.
 *
 * @param[in] sco_link_id       Synchronous link ID
 * @param[in] type              Check SCO or eSCO specifically (ACL_TYPE if the type does not matter)
 *
 * @return True if SCO link present
 ****************************************************************************************
 */
bool lm_look_for_sync(uint8_t link_id, uint8_t type);

/**************************************************************************************
 * @brief Indicate the end of a SCO negotiation
 **************************************************************************************/
void lm_sco_nego_end(void);

#endif //(MAX_NB_SYNC > 0)

///@} LMSCO

#endif // LM_SCO_H_
