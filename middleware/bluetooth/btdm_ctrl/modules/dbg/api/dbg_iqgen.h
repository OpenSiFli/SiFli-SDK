/**
****************************************************************************************
*
* @file dbg_iqgen.h
*
* @brief I&Q Samples Generator API.
*
* Copyright (C) RivieraWaves 2018
*
*
****************************************************************************************
*/

#ifndef DBG_IQGEN_H_
#define DBG_IQGEN_H_

/**
 ****************************************************************************************
 * @addtogroup DBGIQGEN
 * @ingroup IQ
 * @brief Debug SW - I&Q samples Generator.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if BLE_IQ_GEN

#include "reg_iqgen.h"       // I&Q sample generator register functions

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// Maximum number of supported antenna patterns
#define DBG_IQGEN_MAX_ANTENNA (8)

/// bit defintitions for debug configuration
#define DBG_IQGEN_PATTERN_MODE_BIT   (1<<0)
#define DBG_IQGEN_TIMING_MODE_BIT    (1<<1)
#define DBG_IQGEN_ANT_ID_MODE_BIT    (2<<1)


/// I&Q samples generation control
enum dbg_iqgen_ctnl
{
    /// I&Q samples up count
    DBG_IQGEN_UPCOUNT = 0,
    /// I&Q samples down count
    DBG_IQGEN_DOWNCOUNT = 1,
    /// I&Q samples fixed value
    DBG_IQGEN_FIXEDVAL = 2,
    /// I&Q samples PRBS
    DBG_IQGEN_PRBSPATTERN = 3,
};

/// I&Q samples generation antenna sweep pattern
enum dbg_iqgen_antenna_pattern
{
    /// antenna up-sweep
    DBG_IQGEN_ANT_UPSWEEP = 0,
    /// antenna up-down sweep
    DBG_IQGEN_ANT_UPDNSWEEP = 1,
};

/// I&Q samples generation timing mode
enum dbg_iqgen_timing_mode
{
    /// 1us switching/sampling interval
    DBG_IQGEN_1US_INTV = 0,
    /// 2us switching/sampling interval
    DBG_IQGEN_2US_INTV = 1,
};

/// antenna switch enable mode
enum dbg_iqgen_antenna_switch_mode
{
    /// Accepts antenna ID switching inputs from baseband
    DBG_IQGEN_BASEBAND_SWITCHING = 0,
    /// Antenna switching is internally enabled
    DBG_IQGEN_INTERNAL_SWITCHING = 1,

};


/*
 * VARIABLE DECLARATION
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable I&Q sample generator
 *
 * @param[in]  nb_antenna    Number of antenna (valid range 1-8)
 * @param[in]  pattern_mode  Antenna sweep pattern (@see enum dbg_iqgen_antenna_pattern)
 * @param[in]  timing_mode   Timing interval mode (@see enum dbg_iqgen_timing_mode)
 * @param[in]  ant_switch_mode Antenna switching mode (@see enum dbg_iqgen_antenna_switch_mode)
 *
 ****************************************************************************************
 */
void dbg_iqgen_config_enable(uint8_t nb_antenna, uint8_t pattern_mode, uint8_t timing_mode, uint8_t ant_switch_mode);

/**
 ****************************************************************************************
 * @brief Disable I&Q sample generator
 *
 ****************************************************************************************
 */
void dbg_iqgen_disable();

/**
 ****************************************************************************************
 * @brief Configure I&Q samples for a specific antenna
 *
 * @param[in]  antenna ID   antenna id (valid range 0-7)
 * @param[in]  i_cntl       configuration mode for I-samples (@see enum dbg_iqgen_cntl)
 * @param[in]  q_cntl       configuration mode for Q-samples (@see enum dbg_iqgen_cntl)
 * @param[in]  i_val        I-samples initial value
 * @param[in]  q_val        Q-samples initial value
 *
 ****************************************************************************************
 */
void dbg_iqgen_antenna_config(uint8_t antenna_id, uint8_t i_cntl, uint8_t q_cntl, uint8_t i_val, uint8_t q_val);


/**
****************************************************************************************
* @brief I&Q Generator configure
*
* @param[in]  param       configuration structure (@see hci_dbg_iqgen_cfg_cmd)
****************************************************************************************
*/
uint8_t dbg_iqgen_config(struct hci_dbg_iqgen_cfg_cmd const *cfg);

#endif //BLE_IQ_GEN

/// @} DBGIQGEN

#endif // DBG_IQGEN_H_
