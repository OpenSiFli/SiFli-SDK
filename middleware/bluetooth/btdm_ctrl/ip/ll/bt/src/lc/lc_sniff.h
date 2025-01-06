/**
****************************************************************************************
*
* @file lm_lowpow.h
*
* @brief Link Manager Utilities functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef LM_LOWPOW_H_
#define LM_LOWPOW_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean

/**
****************************************************************************************
* @defgroup LMLOWPOW Low Power Functions
* @ingroup LM
* @brief LM module for low power mode management (sniff, hold, park).
* @{
****************************************************************************************
*/
/// Commands for Sniff
#define SNIFF_HL_REQ                          0
#define SNIFF_PEER_REQ                        1

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes Sniff block.
 ****************************************************************************************
 */
void lc_sniff_init(bool reset);

/**
 ****************************************************************************************
 * @brief Clear sniff parameters of a specific link
 ****************************************************************************************
 */
void lc_sniff_clear(uint8_t link_id);


/**
 ****************************************************************************************
 * @brief Get sniff interval and subrate
 * @param[in]   link_id      Link identifier
 * @param[out]  interval     Sniff interval (in slots)
 * @param[out]  max_subrate  Maximum sniff subrate (in number of sniff events, 0 if no subrating)
 ****************************************************************************************
 */
void lc_sniff_interval_get(uint8_t link_id, uint16_t *interval, uint8_t *max_subrate);

/**
 ****************************************************************************************
 * @brief This function is used to update the sniff offset
 *
 * @param[in] link_id             Link identifier
 * @param[in] sniff_offset        Sniff offset in half-slots
 ****************************************************************************************
 */
void lc_sniff_offset_update(uint8_t link_id, uint16_t sniff_offset);


void lc_sniff_host_request(ke_task_id_t pid, uint16_t max_int, uint16_t min_int, uint16_t attempt, uint16_t timeout);
void lc_sniff_peer_request(ke_task_id_t pid, uint8_t flags, uint16_t d_sniff, uint16_t t_sniff, uint16_t sniff_attempt, uint16_t sniff_to);
void lc_sniff_peer_accept(ke_task_id_t pid);
void lc_sniff_peer_reject(ke_task_id_t pid, uint8_t reason);
void lc_sniff_baseband_ack(ke_task_id_t pid, uint8_t lmp_opcode);
void lc_sniff_timeout(ke_task_id_t pid);


/**
 ****************************************************************************************
 * @brief Unsniff link
 *
 * This function initiating an unsniff LMP procedure.
 *
 * param[in]    pid      Process identifier (task / instance)
 ****************************************************************************************
 */
void lc_sniff_unsniff(ke_task_id_t pid);
void lc_sniff_unsniff_peer_request(ke_task_id_t pid);
void lc_sniff_unsniff_peer_accept(ke_task_id_t pid);
void lc_sniff_unsniff_peer_reject(ke_task_id_t pid, uint8_t reason);

void lc_sniff_ssr_nego(ke_task_id_t pid);
void lc_sniff_ssr_host_request(ke_task_id_t pid, uint16_t max_lat, uint16_t min_rem_to, uint16_t min_loc_to);
void lc_sniff_ssr_peer_request(ke_task_id_t pid, uint8_t  max_subrate, uint16_t min_to, uint32_t instant);
void lc_sniff_ssr_peer_response(ke_task_id_t pid, uint8_t  max_subrate, uint16_t min_to, uint32_t instant);
void lc_sniff_ssr_peer_reject(ke_task_id_t pid, uint8_t reason);
void lc_sniff_ssr_baseband_ack(ke_task_id_t pid);
void lc_sniff_ssr_instant(ke_task_id_t pid);

///@} LMLOWPOW

#endif // LM_LMLOWPOW_H_
